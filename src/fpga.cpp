#include <unistd.h>
#include <ctype.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "fpga.h"
#include "util.h"
#include "fmem.h"

#define TOHOST_OFFSET 0
#define FROMHOST_OFFSET 8
#define FIRST_VIRTIO_IRQ 0

static int debug_virtio = 0;
static int debug_stray_io = 1;

// Assign the initial value of the FPGA singleton to null.
FPGA *fpga = NULL;
// This function is responsible for setting up the FPGA singleton
void fpga_singleton_init(int id, const Rom &rom, const char *tun_iface) {
    if (fpga != NULL) {
        fprintf(stderr, "ERROR: Called init_fpga_singleton() multiple times\r\n");
        abort();
    }
    fpga = new FPGA(id, rom, tun_iface);
}
// Call dma_read on the FPGA singleton, taking the DMA lock to ensure other DMA transactions don't interfere with the selector FD.
// Can be passed to C interfaces as a plain function pointer.
void fpga_singleton_dma_read(uint64_t addr, uint8_t * data, size_t num_bytes) {
    fpga->dma_read(addr, data, num_bytes);
}
// Call dma_write on the FPGA singleton, taking the DMA lock to ensure other DMA transactions don't interfere with the selector FD.
// Can be passed to C interfaces as a plain function pointer.
void fpga_singleton_dma_write(uint64_t addr, const uint8_t * data, size_t num_bytes) {
    fpga->dma_write(addr, data, num_bytes);
}

class FPGA_io {
private:
    int mmio_fd;
    int dma_fd;
    int irq_fd;
    int selector_fd;
public:
    FPGA_io(int id) : mmio_fd(-1), dma_fd(-1), irq_fd(-1) { // XXX What is "id" for? What was AWSP2_ResponseWrapper?
        // Initialise Memory-mapped IO
        // Open FMEM device for the management interface of the "Virtual Device",
        // a peripheral that captures reads and writes on one interface and provides
        // controlled responses on the mangement side.  FMEM provides a file-descriptor-
        // based memory-mapped IO interface with custom ioctls.
        char* fmemdev = getenv("RISCV_VIRTUAL_DEVICE_FMEM_DEV");
        char filename[256] = "/dev/fmem_sys0_virtual_device";
        if (fmemdev) {
            strncpy(filename, fmemdev, 255);
            filename[255] = '\0';
        }
        mmio_fd = open(filename, O_RDWR);
        fprintf(stdout, "MMIO fd: %d\n", mmio_fd);
        // Open address window selector fmem device
        fmemdev = getenv("RISCV_ADDRESS_SELECTOR_FMEM_DEV");
        if (fmemdev) {
            strncpy(filename, fmemdev, 255);
        } else {
            strncpy(filename, "/dev/fmem_sys0_address_selector", 255);
        }
        filename[255] = '\0';
        selector_fd = open(filename, O_RDWR);
        if (selector_fd < 0) {
            fprintf(stderr, "ERROR: Failed to open address selector device file: %s\r\n", strerror(errno));
            abort();
        }
        fprintf(stdout, "selector fd: %d\n", selector_fd);
        // Open DMA fmem file descriptor.
        // This one allows access to coherent shared memory with the guest.
        fmemdev = getenv("RISCV_DMA_FMEM_DEV");
        if (fmemdev) {
            strncpy(filename, fmemdev, 255);
        } else {
            strncpy(filename, "/dev/fmem_h2f_dflt_1G", 255);
        }
        filename[255] = '\0';
        dma_fd = open(filename, O_RDWR);
        if (dma_fd < 0) {
            fprintf(stderr, "ERROR: Failed to open fmem dma device file: %s\r\n", strerror(errno));
            abort();
        }
        fprintf(stdout, "DMA fd: %d\n", dma_fd);
        fflush(stdout);
        fmem_write32(mmio_fd, VD_ENABLE, 1); // Enable the virtual device.  That is, start capturing all reads and writes.
        // Opem IRQ fmem device
        // This is a couple registers that allow setting and clearing interrupts for the guest.
        fmemdev = getenv("RISCV_INTERRUPT_FMEM_DEV");
        strncpy(filename, "/dev/fmem_sys0_interrupts", 255);
        if (fmemdev) {
            strncpy(filename, fmemdev, 255);
        }
        filename[255] = '\0';
        irq_fd = open(filename, O_RDWR);
        if (irq_fd < 0) {
            fprintf(stderr, "ERROR: Failed to open fmem interrupts device file: %s\r\n", strerror(errno));
            abort();
        }
    }
    bool emulated_mmio_has_request() {
        return (fmem_read8(mmio_fd, VD_REQ_LEVEL) != 0);
    }
    //void close_dma();
    int get_dma_fd() { return dma_fd; }
    int get_irq_fd() { return irq_fd; }
    /*
    dma_set_window, dma_read8, dma_read32, dma_write8, dma_write32 all assume the dma_mutex have been taken before calling them.
    */
    uint32_t dma_set_window(uint64_t addr); // Returns the offset to use when fmem-ing the DMA window
    uint8_t dma_read8(uint64_t raddr);
    uint32_t dma_read32(uint64_t raddr);
    void dma_write8(uint64_t waddr, uint8_t wdata);
    void dma_write32(uint64_t waddr, uint32_t wdata);

    void emulated_mmio_respond();
    void console_putchar(uint64_t wdata);
    virtual void uart_tohost(uint8_t ch);
};
/*
void FPGA_io::close_dma()
{
    if (dma_fd >= 0)
        close(dma_fd);
}
*/

// A bitmask indicating the width of the DMA window - here, 32 bits
#define DMA_WINDOW_MASK 0x3FFFFFFFu
// fmem_{read,write} truncate the address to 32-bits (thanks, C++ numeric coercion >:P)
// so dma_set_window returns a truncated address masked by the window
// => the DMA window must be at most 32-bits wide.
static_assert(DMA_WINDOW_MASK <= std::numeric_limits<uint32_t>::max());

// Keep a static to remember what the last DMA offset is.
// It will always have the window-bits masked out, so set the initial value to have those bits set
// => the first time we check (offset != last_offset) it will always evaluate to false.
static uint64_t last_offset = DMA_WINDOW_MASK; 

uint32_t FPGA_io::dma_set_window(uint64_t addr) {
    // Assuming DMA mutex has been taken
    uint64_t offset = addr & (~DMA_WINDOW_MASK);
    if (offset != last_offset) {
        if (selector_fd >= 0) {
            printf("writing address selector (write) 0x0 == 0x%" PRIx64 "\n",
                    offset);
            int error = fmem_write64(selector_fd, 0, offset);
            if (error != 0) {
                printf("error with address selector (write) 0x0 == 0x%" PRIx64 "\n",
                       offset);
            }
            last_offset = offset;
        }
        else {
            fprintf(stderr, "ERROR: Attempted write unusable fmem address selector device file: %s\r\n", strerror(errno));
            abort();
        };
    }
    // Return the masked address, which is always within the window size.
    return (uint32_t)(addr & DMA_WINDOW_MASK);
}

uint8_t FPGA_io::dma_read8(uint64_t raddr) {
    // Assuming DMA mutex has been taken
    uint32_t dma_offset = dma_set_window(raddr);
    if (dma_fd >= 0) return fmem_read8(dma_fd, dma_offset);
    else {
        fprintf(stderr, "ERROR: Attempted read from unusable fmem dma device file: %s\r\n", strerror(errno));
        abort();
    };
}

uint32_t FPGA_io::dma_read32(uint64_t raddr) {
    // Assuming DMA mutex has been taken
    uint32_t dma_offset = dma_set_window(raddr);
    if (dma_fd >= 0) return fmem_read32(dma_fd, dma_offset);
    else {
        fprintf(stderr, "ERROR: Attempted read from unusable fmem dma device file: %s\r\n", strerror(errno));
        abort();
    };
}

void FPGA_io::dma_write8(uint64_t waddr, uint8_t wdata) {
    // Assuming DMA mutex has been taken
    uint32_t dma_offset = dma_set_window(waddr);
    //printf("dma_write8 addr 0x%016lx data 0x%u\r\n", waddr, wdata);
    if (dma_fd >= 0) fmem_write8(dma_fd, dma_offset, wdata);
    else {
        fprintf(stderr, "ERROR: Attempted write to unusable fmem dma device file: %s\r\n", strerror(errno));
        abort();
    };
}

void FPGA_io::dma_write32(uint64_t waddr, uint32_t wdata) {
    // Assuming DMA mutex has been taken
    //printf("dma_write32 addr 0x%016lx data 0x%u\r\n", waddr, wdata);
    uint32_t dma_offset = dma_set_window(waddr);
    if (dma_fd >= 0) fmem_write32(dma_fd, dma_offset, wdata);
    else {
        fprintf(stderr, "ERROR: Attempted write to unusable fmem dma device file: %s\r\n", strerror(errno));
        abort();
    };
}

/*
void FPGA_io::irq_status ( const uint32_t levels )
{
    fpga->misc_rsp_data = levels;
    sem_post(&fpga->sem_misc_response);
}
*/

void
FPGA_io::emulated_mmio_respond() {
    if (fmem_read8(mmio_fd, VD_IS_WRITE)) {
        uint32_t waddr = fmem_read32(mmio_fd, VD_WRITE_ADDR);
        uint64_t wdata = fmem_read64(mmio_fd, VD_WRITE_DATA);
        uint8_t wstrb = fmem_read8(mmio_fd, VD_WRITE_BYEN);
        PhysMemoryRange *pr = fpga->virtio_devices.get_phys_mem_range(waddr);
        if (pr) {
            int size_log2 = 2;
            uint32_t offset = waddr - pr->addr;
            if (waddr & 4) {
                wdata = (wdata >> 32) & 0xFFFFFFFF;;
            }
            if (debug_virtio) printf("virtio waddr %08x offset %08x wdata %08lx wstrb %02x \r\n", waddr, offset, wdata, wstrb);
            pr->write_func(pr->opaque, offset, wdata, size_log2);
        } else if (waddr == fpga->tohost_addr) {
            // tohost
            uint8_t dev = (wdata >> 56) & 0xFF;
            uint8_t cmd = (wdata >> 48) & 0xFF;
            uint64_t payload = wdata & 0x0000FFFFFFFFFFFFul;
            if (dev == 1 && cmd == 1) {
                console_putchar(payload);
            } else if (dev == 0 && cmd == 0) {
                int code;
                if (payload == 1) {
                    code = 0;
                    printf("PASS\r\n");
                } else {
                    code = payload >> 1;
                    printf("FAIL: error %u\r\n", code);
                }
                fpga->stop_io(code);
            } else {
                printf("\r\nHTIF: dev=%d cmd=%02x payload=%08lx\r\n", dev, cmd, payload);
            }
        } else if (waddr == fpga->fromhost_addr) {
            //fprintf(stderr, "\r\nHTIF: addr %08x wdata=%08lx\r\n", addr, wdata);
        } else if (waddr == fpga->sifive_test_addr) {
            // Similar to HTIF, but the address is in the device tree so an
            // unmodified BBL can use it. It gets used for shutdown so we make it
            // silent.
            int status = wdata & 0xFFFF;
            if (status == 0x3333) {
                // FAIL
                int code = (wdata >> 16) & 0xFFFF;
                fpga->stop_io(code);
            } else if (status == 0x5555) {
                // PASS
                fpga->stop_io(0);
            } else if (status == 0x7777) {
                // RESET
                fpga->stop_io(EXIT_CODE_RESET);
            } else {
                fprintf(stderr, "\r\nSiFive Test Finisher: status=%04x\r\n", status);
            }
        } else {
            if (debug_stray_io) printf("Stray io! waddr %08x io_wdata wdata=%lx wstrb=%x\r\n", waddr, wdata, wstrb);
        }
    } else { // must be a read request
        uint32_t araddr = fmem_read32(mmio_fd, VD_READ_ADDR);
        uint16_t arlen = 0;//fmem_read8(VD_FLIT_SIZE); // Non-0 arlen is likely to break something.
        uint16_t arid = fmem_read32(mmio_fd, VD_REQ_ID);
        PhysMemoryRange *pr = fpga->virtio_devices.get_phys_mem_range(araddr);
        if (arlen != 0) printf("ERROR: fromhost araddr %08x arlen %d\r\n", araddr, arlen);
        else if (pr) {
            uint32_t offset = araddr - pr->addr;
            int size_log2 = 2;
            uint64_t val = pr->read_func(pr->opaque, offset, size_log2);
            if ((offset % 8) == 4)
                val = (val << 32); // Assuming a 64-bit virtualised data width.
            fmem_write64(mmio_fd, VD_READ_DATA,val);
            if (debug_virtio)
                printf("virtio araddr %0x device addr %08lx offset %08x len %d val %08lx\r\n",
                        araddr, pr->addr, offset, arlen, val);
        } else if (fpga->rom.base <= araddr && araddr < fpga->rom.limit) {
            int offset = (araddr - fpga->rom.base) / 8;
            //fprintf(stderr, "rom offset %x data %08lx\r\n", (int)(araddr - fpga->rom.base), fpga->rom.data[offset]);
            fmem_write64(mmio_fd, VD_READ_DATA,fpga->rom.data[offset]);
        } else if (araddr == fpga->fromhost_addr) {
            uint8_t ch = 0;
            
            if (fpga->htif_enabled && fpga->dequeue_stdin(&ch)) {
                uint64_t cmd = (1ul << 56) | (0ul << 48) | ch;
                fmem_write64(mmio_fd, VD_READ_DATA,cmd);
            } else {
                fmem_write64(mmio_fd, VD_READ_DATA,0);
            }
        } else if (araddr == fpga->sifive_test_addr) {
            fmem_write64(mmio_fd, VD_READ_DATA,0);
        } else {
            if (araddr != 0x10001000 && araddr != 0x10001008 && araddr != 0x50001000 && araddr != 0x50001008)
                if (debug_stray_io) fprintf(stderr, "io_araddr araddr=%08x arlen=%d\r\n", araddr, arlen);
            fmem_write64(mmio_fd, VD_READ_DATA,0);
        }
    }
    fmem_write32(mmio_fd, VD_SEND_RESP, 1); // Send any response.
}

void FPGA_io::uart_tohost(uint8_t ch) {
    console_putchar(ch);
    fprintf(stdout, "uart{%x}\r\n", ch); fflush(stdout);
}

void FPGA_io::console_putchar(uint64_t wdata) {
    fputc(wdata, stdout);
    fflush(stdout);
}

FPGA::FPGA(int id, const Rom &rom, const char *tun_iface)
    : io(0), rom(rom), virtio_devices(FIRST_VIRTIO_IRQ, tun_iface), irq_state(0),
      ctrla_seen(0), sifive_test_addr(0x50000000), htif_enabled(0), uart_enabled(0)
{
    sem_init(&sem_misc_response, 0, 0);
    io = new FPGA_io(id);
    virtio_devices.set_virtio_dma_funcs();
    set_htif_base_addr(0x10001000);
}

FPGA::~FPGA() {
}
/* // Assume these are unused for fmem world for now.
void FPGA::map_pcis_dma()
{
    size_t dram_size = 2 * 1024 * 1024 * 1024ul;
    off_t dram_offset = 2 * 1024 * 1024 * 1024ul;
    dma_fd = open("/dev/portal_dma_pcis", O_RDWR);
    if (dma_fd < 0) {
        fprintf(stderr, "error: opening /dev/portal_dma_pcis %s\r\n", strerror(errno));
        abort();
    }
    // The portal_dma_pcis device driver does not respect the mmap offset, so
    // we map starting from 0 and manually offset the pointer when passing to
    // set_dram_buffer.
    dram_mapping = (uint8_t *)mmap(0, dram_offset + dram_size, PROT_READ|PROT_WRITE, MAP_SHARED, dma_fd, 0);
    dram_mapping_size = dram_offset + dram_size;
    fprintf(stderr, "PCIS DMA DRAM mapping %08lx size 0x%lx fd %d\r\n", (long)dram_mapping, (long)dram_size, dma_fd);
    if (dram_mapping == MAP_FAILED) {
        fprintf(stderr, "mmap PCIS DMA failed %s\r\n", strerror(errno));
        dram_mapping = NULL;
        abort();
    }
}

void FPGA::unmap_pcis_dma()
{
    if (dram_mapping)
        munmap(dram_mapping, dram_mapping_size);
    if (dma_fd >= 0)
        close(dma_fd);
    dram_mapping = 0;
    dma_fd = -1;
}


void FPGA::close_dma()
{
    io->close_dma();
}
*/

void FPGA::dma_read(uint64_t addr, uint8_t *data, size_t size) {
    std::lock_guard<std::mutex> lock(dma_mutex);
    //printf("dma_read addr: %016lx size: %lu\r\n", addr, size);
    size_t i = 0;
    if ((addr & 3) == 0) {
        for (; i<size; i+=4) {
            ((uint32_t *)(data+i))[0] = io->dma_read32(addr+i);
        }
        if (i > size) {
            i -= 4; // undo add that pushed us over so the byte loop
                    // can clean up
        }
    }
    for (; i<size; i++) data[i] = io->dma_read8(addr+i);
    //printf("dma_read done, data[0]: 0x%x\r\n", data[0]);
}

void FPGA::dma_write(uint64_t addr, const uint8_t *data, size_t size) {
    std::lock_guard<std::mutex> lock(dma_mutex);
    //printf("dma_write addr: %016lx size: %lu data[0]: 0x%x\r\n", addr, size, data[0]);
    size_t i = 0;
    if ((addr & 3) == 0) {
        for (; i < size; i += 4) {
            io->dma_write32(addr+i, ((const uint32_t *)(data+i))[0]);
        }
        if (i > size) {
            i -= 4; // undo add that pushed us over so the byte loop
                    // can clean up
        }
    }
    for (; i < size; i++) io->dma_write8(addr+i, data[i]);
    //printf("dma_write done\r\n");
}

/* XXX Implement IRQs somehow.  Just stubbed out for now. */

void FPGA::irq_set_levels(uint32_t w1s)
{
    std::lock_guard<std::mutex> lock(misc_request_mutex);
    fmem_write32(io->get_irq_fd(), 0, w1s);
    irq_state |= w1s;
    //request->irq_set_levels(w1s);
}

void FPGA::irq_clear_levels(uint32_t w1c)
{
    std::lock_guard<std::mutex> lock(misc_request_mutex);
    fmem_write32(io->get_irq_fd(), 4, w1c);
    irq_state &= ~w1c;
    //request->irq_clear_levels(w1c);
}

int FPGA::read_irq_status ()
{
    std::lock_guard<std::mutex> lock(misc_request_mutex);

    //request->read_irq_status();
    //wait_misc_response();
    //return misc_rsp_data;
    return (int)irq_state;
    
}
/* ----------- XXX IRQs XXX ----------------*/

void FPGA::enqueue_stdin(char *buf, size_t num_chars)
{
    size_t j = 0;
    for (size_t i = 0; i < num_chars; ++i) {
        if (ctrla_seen) {
            ctrla_seen = 0;
            switch (buf[i]) {
                case 'x':
                    stop_io(0);
                    fprintf(stderr, "\r\nTerminated\r\n");
                    return;
                case 'r':
                    stop_io(EXIT_CODE_RESET);
                    return;
                case 'h':
                    fprintf(stderr, "\r\n");
                    fprintf(stderr, "C-a h   print this help\r\n");
                    fprintf(stderr, "C-a r   reset the system\r\n");
                    fprintf(stderr, "C-a x   exit\r\n");
                    fprintf(stderr, "C-a C-a send C-a\r\n");
                    continue;
                // C-a itself, and any unrecognised characters, are just passed
                // through as if C-a wasn't pressed.
                case 1:
                default:
                    break;
            }
        } else if (buf[i] == 1) {
            ctrla_seen = 1;
            continue;
        }

        buf[j++] = buf[i];
    }
    num_chars = j;

    if (virtio_devices.has_virtio_console_device()) {
        while (num_chars > 0) {
            ssize_t sent = ::write(virtio_stdio_pipe[1], buf, num_chars);
            if (sent < 0) {
                if (errno == EINTR) continue;
                abort();
            }
            buf += sent;
            num_chars -= sent;
        }
    } else {
        std::lock_guard<std::mutex> lock(stdin_mutex);
        for (size_t i = 0; i < num_chars; i++) {
            stdin_queue.push(buf[i]);
        }
    }
}

int FPGA::dequeue_stdin(uint8_t *chp)
{
    std::lock_guard<std::mutex> lock(stdin_mutex);
    if (stdin_queue.size()) {
        *chp = stdin_queue.front();
        stdin_queue.pop();
        return 1;
    } else {
        return 0;
    };
}

void FPGA::process_stdin()
{
    int stdin_fd = STDIN_FILENO;
    int fd_max = -1;
    fd_set rfds, wfds, efds;
    int stop_fd = stop_stdin_pipe[0];

    for (;;) {
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);
        FD_ZERO(&efds);
        FD_SET(stdin_fd, &rfds);
        FD_SET(stop_fd, &rfds);
        fd_max = std::max(stdin_fd, stop_fd);

        select(fd_max + 1, &rfds, &wfds, &efds, NULL);
        if (FD_ISSET(stop_fd, &rfds)) {
            break;
        }
        if (FD_ISSET(stdin_fd, &rfds)) {
            // Read from stdin and enqueue for HTIF/UART get char
            char buf[128];
            memset(buf, 0, sizeof(buf));
            ssize_t ret = read(stdin_fd, buf, sizeof(buf));
            if (ret > 0) {
                enqueue_stdin(buf, ret);
            }
        }
    }

    close(stop_fd);
    if (virtio_devices.has_virtio_console_device()) {
        close(virtio_stdio_pipe[1]);
    }
}

void *FPGA::process_stdin_thread(void *null_arg)
{
    fpga->process_stdin();
    return NULL;
}

void FPGA::start_io()
{
    printf("start_io\r\n");
    if (!done_termios) {
        struct termios stdin_termios;
        struct termios stdout_termios;

        tcgetattr(STDIN_FILENO, &stdin_termios);
        tcgetattr(STDOUT_FILENO, &stdout_termios);
        orig_stdin_termios = stdin_termios;
        orig_stdout_termios = stdout_termios;
        atexit(&reset_termios);

        cfmakeraw(&stdin_termios);
        cfmakeraw(&stdout_termios);
        stdin_termios.c_cc[VMIN] = 1;
        stdout_termios.c_cc[VMIN] = 1;
        stdin_termios.c_cc[VTIME] = 0;
        stdout_termios.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &stdin_termios);
        tcsetattr(STDOUT_FILENO, TCSAFLUSH, &stdout_termios);
        tcsetattr(STDOUT_FILENO, TCSAFLUSH, &stdout_termios);

        done_termios = true;
    }

    pipe(stop_stdin_pipe);
    fcntl(stop_stdin_pipe[1], F_SETFL, O_NONBLOCK);
    pthread_create(&stdin_thread, NULL, &process_stdin_thread, NULL);
    pthread_setname_np(stdin_thread, "Console input");

    if (virtio_devices.has_virtio_console_device()) {
        pipe(virtio_stdio_pipe);
        virtio_devices.set_virtio_stdin_fd(virtio_stdio_pipe[0]);
    }

    virtio_devices.start();
}

void FPGA::stop_io(int code)
{
    exit_code = code;

    char dummy = 'X';
    ::write(stop_stdin_pipe[1], &dummy, sizeof(dummy));
    close(stop_stdin_pipe[1]);

    virtio_devices.stop();
}

int FPGA::join_io()
{
    pthread_join(stdin_thread, NULL);

    virtio_devices.join();

    return exit_code;
}

struct termios FPGA::orig_stdin_termios;
struct termios FPGA::orig_stdout_termios;
bool FPGA::done_termios = false;

void FPGA::reset_termios()
{
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_stdin_termios);
    tcsetattr(STDOUT_FILENO, TCSAFLUSH, &orig_stdout_termios);
}

void FPGA::set_htif_base_addr(uint64_t baseaddr)
{
    tohost_addr = baseaddr + TOHOST_OFFSET;
    fromhost_addr = baseaddr + FROMHOST_OFFSET;
}

void FPGA::set_tohost_addr(uint64_t addr)
{
    tohost_addr = addr;
}

void FPGA::set_fromhost_addr(uint64_t addr)
{
    fromhost_addr = addr;
}

void FPGA::set_htif_enabled(bool enabled)
{
    htif_enabled = enabled;
}

void FPGA::set_uart_enabled(bool enabled)
{
    uart_enabled = enabled;
}

bool FPGA::emulated_mmio_has_request()
{
    return(io->emulated_mmio_has_request());
}

void FPGA::emulated_mmio_respond()
{
    io->emulated_mmio_respond();
}

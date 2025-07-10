/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Ruslan Bukin <br@bsdpad.com>
 * Copyright (c) 2022 Jon Woodruff <Jonathan.Woodruff@cl.cam.ac.uk>
 *
 * This work was supported by Innovate UK project 105694, "Digital Security
 * by Design (DSbD) Technology Platform Prototype".
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <sys/types.h>
#include <sys/uio.h>
#include <sys/ioctl.h>

struct fmem_request {
    uint32_t offset;
    uint32_t data;
    uint32_t access_width;
};

enum {
    FMEM_READ   = _IOWR('X', 1, struct fmem_request),
    FMEM_WRITE  = _IOWR('X', 2, struct fmem_request),
};

uint32_t fmem_read(int fd, uint32_t offset, uint8_t width)
{
    struct fmem_request req;
    int error;
    // Sanitise to a 32-bit access, as something in the chain
    // only supports 32-bit currently.
    uint32_t adr_mask = 0xFFFFFFFC;
    req.offset = offset & adr_mask;
    req.access_width = 4;

    //printf("o:%x\r\n", req.offset);
    fflush(stdout);
    error = ioctl(fd, FMEM_READ, &req);
    if (error == 0){
        uint32_t wide = req.data;
        // These next two lines could be simpler, but shifting by >=32 is undefined.
        uint32_t dat_mask = -1;
        if (width == 1) dat_mask = 0xFF;
        if (width == 2) dat_mask = 0xFFFF;
        //fprintf(stdout, "read! offset: %x, req.data: %x, adr_mask: %x, dat_mask: %x \r\n", offset, req.data, adr_mask, dat_mask);
        //fflush(stdout);
        return ((wide >> ((offset & ~adr_mask)*8)) & dat_mask);
    } else return (0);
}
uint8_t fmem_read8(int fd, uint32_t offset)
{
    return fmem_read(fd, offset, 1);
}
uint16_t fmem_read16(int fd, uint32_t offset)
{
    return fmem_read(fd, offset, 2);
}
uint32_t fmem_read32(int fd, uint32_t offset)
{
    return fmem_read(fd, offset, 4);
}
uint64_t fmem_read64(int fd, uint32_t offset)
{
    uint64_t hi = fmem_read32(fd, offset+4);
    return (hi<<32) | fmem_read32(fd, offset);
}
uint64_t fmem_write(int fd, uint32_t offset, uint32_t data, uint8_t width)
{
    struct fmem_request req;
    int error;

    req.offset = offset;
    req.data = data;
    req.access_width = width;
    //fprintf(stdout, "write! offset: %x, req.data: %x, width: %x \r\n", offset, req.data, width);
    //fflush(stdout);
    error = ioctl(fd, FMEM_WRITE, &req);
    return (error);
}
uint64_t fmem_write8(int fd, uint32_t offset, uint8_t data)
{
    return fmem_write(fd, offset, data, 1);
}
uint64_t fmem_write16(int fd, uint32_t offset, uint16_t data)
{
    return fmem_write(fd, offset, data, 2);
}
uint64_t fmem_write32(int fd, uint32_t offset, uint32_t data)
{
    return fmem_write(fd, offset, data, 4);
}
uint64_t fmem_write64(int fd, uint32_t offset, uint64_t data)
{
    int error = fmem_write32(fd, offset, data);
    if (error) return error;
    return (fmem_write32(fd, offset+4, data>>32));
}

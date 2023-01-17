This is bits of the original ReadMe for TinyEMU.
I've removed the parts that refer to functionality
that has been intentionally removed in this repository,
and left content that might remain to some degree.

TinyEMU System Emulator by Fabrice Bellard
==========================================

1) Features
-----------

...

- VirtIO console, network, block device, input and 9P filesystem

- Graphical display with SDL

- JSON configuration file

- Remote HTTP block device and filesystem

- small code, easy to modify, no external dependancies

...

2) Installation
---------------

- The libraries libcurl, OpenSSL and SDL should be installed. On a Fedora
  system you can do it with:

  sudo dnf install openssl-devel libcurl-devel SDL-devel

  It is possible to compile the programs without these libraries by
  commenting CONFIG_FS_NET and/or CONFIG_SDL in the Makefile.

...

- Use 'make' to compile the binaries.

- You can optionally install the program to '/usr/local/bin' with:

  make install

3) Usage
--------

...

3.2 Invocation
--------------

usage: temu [options] config_file
options are:
-m ram_size       set the RAM size in MB
-rw               allow write access to the disk image (default=snapshot)
-ctrlc            the C-c key stops the emulator instead of being sent to the
                  emulated software
-append cmdline   append cmdline to the kernel command line
-no-accel         disable VM acceleration (KVM, x86 machine only)

Console keys:
Press C-a x to exit the emulator, C-a h to get some help.

3.3 Network usage
-----------------

The easiest way is to use the "user" mode network driver. No specific
configuration is necessary.

TinyEMU also supports a "tap" network driver to redirect the network
traffic from a VirtIO network adapter.

You can look at the netinit.sh script to create the tap network
interface and to redirect the virtual traffic to Internet thru a
NAT. The exact configuration may depend on the Linux distribution and
local firewall configuration.

The VM configuration file must include:

eth0: { driver: "tap", ifname: "tap0" }

and configure the network in the guest system with:

ifconfig eth0 192.168.3.2
route add -net 0.0.0.0 gw 192.168.3.1 eth0

3.4 Network filesystem
----------------------

TinyEMU supports the VirtIO 9P filesystem to access local or remote
filesystems. For remote filesystems, it does HTTP requests to download
the files. The protocol is compatible with the vfsync utility. In the
"mount" command, "/dev/rootN" must be used as device name where N is
the index of the filesystem. When N=0 it is omitted.

The build_filelist tool builds the file list from a root directory. A
simple web server is enough to serve the files.

The '.preload' file gives a list of files to preload when opening a
given file.

3.5 Network block device
------------------------

TinyEMU supports an HTTP block device. The disk image is split into
small files. Use the 'splitimg' utility to generate images. The URL of
the JSON blk.txt file must be provided as disk image filename.

4) Technical notes
------------------

...

4.3) HTIF console

The standard HTIF console uses registers at variable addresses which
are deduced by loading specific ELF symbols. TinyEMU does not rely on
an ELF loader, so it is much simpler to use registers at fixed
addresses (0x40008000). A small modification was made in the
"riscv-pk" boot loader to support it. The HTIF console is only used to
display boot messages and to power off the virtual system. The OS
should use the VirtIO console.

...


5) License / Credits
--------------------

TinyEMU is released under the MIT license. If there is no explicit
license in a file, the license from MIT-LICENSE.txt applies.

The SLIRP library has its own license (two clause BSD license).

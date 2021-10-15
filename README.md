# BWUDP – bantamweight UDP server/client

These routines provide very lightweight access to Ethernet interfaces
such as the Badger firmware (FPGA HDL) provided as part of the LBNL
[Bedrock](https://github.com/BerkeleyLab/Bedrock) package.

The file [BWUDP_manual.html](BWUDP_manual.html) documents the C API.

BWUDP should be considered in the same genre as [lwIP](https://en.wikipedia.org/wiki/LwIP)
and [uIP](https://en.wikipedia.org/wiki/UIP_\(micro_IP\)), but has fewer features,
is even lighter-weight (smaller binary footprint), and is not derived from them.

The only test provided here is a simple (and useless) native compile, showing that
the C code is standards-compiant.  Just type "make".

License: [BSD-3-Clause](https://spdx.org/licenses/BSD-3-Clause.html) (see individual C files)

Primary author: Eric Norum

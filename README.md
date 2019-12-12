# Bootloaders for the Pixhawk board family

[![Build Status](https://travis-ci.org/PX4/Bootloader.svg?branch=master)](https://travis-ci.org/PX4/Bootloader)

## Build instructions

Build all targets:

```
git submodule sync --recursive
git submodule update --init --recursive
make
```

The binaries will be in `build/BOARDNAME/BOARDNAME.elf`. Two files are built: ELF files for use with JTAG adapters and BIN files for direct onboard upgrading.

Build a specific board: Please check the `Makefile` for specific build targets.

## License

License: LGPL for libopencm3, BSD for core bootloader (see LICENSE.md)

## Contact

  * Chat: [Dronecode Slack](http://slack.px4.io)
  * Forum: [Dronecode Discuss](http://discuss.px4.io)

## Bootloader Usage

Typical use case. The PX4 IO. [px4pipbl.pdf](https://github.com/PX4/Bootloader/files/3955700/px4pipbl.pdf)

To avoid accidental erasure or bad image loading:

The booaloder need to receive `PROTO_GET_SYNC` and `PROTO_GET_DEVICE` Prior to receiving `PROTO_CHIP_ERASE`    
The booaloder need to receive `PROTO_GET_SYNC` and `PROTO_GET_DEVICE` and `PROTO_PROG_MULTI` and `PROTO_GET_CRC` Prior to receiving `PROTO_BOOT`

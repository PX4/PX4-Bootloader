# Bootloader for the Pixhawk board family

[![GitHub Actions Status](https://github.com/PX4/PX4-Bootloader/workflows/Build%20and%20Deploy/badge.svg?branch=master)](https://github.com/PX4/PX4-Bootloader/actions?query=branch%3Amaster)

## Build instructions

Build all targets:

```
git submodule sync --recursive
git submodule update --init --recursive
make
```

The binaries will be in `build/BOARDNAME/BOARDNAME.elf`. Two files are built: ELF files for use with JTAG adapters and BIN files for direct onboard upgrading.

Build a specific board: Please check the [Makefile](Makefile) for specific build targets.

## License

License: LGPL for libopencm3, BSD for core bootloader (see LICENSE.md)

## Contact

  * Chat: [Dronecode Slack](http://slack.px4.io)
  * Forum: [Dronecode Discuss](http://discuss.px4.io)

## Bootloader Usage

The typical use case as used for the the PX4IO is described in [px4pipbl.pdf](https://github.com/PX4/Bootloader/files/3955700/px4pipbl.pdf).

To avoid accidental erasure or bad image loading:

- The bootloader needs to receive `PROTO_GET_SYNC` and `PROTO_GET_DEVICE` prior to receiving `PROTO_CHIP_ERASE`.
- The bootloader needs to receive `PROTO_GET_SYNC` and `PROTO_GET_DEVICE` and `PROTO_PROG_MULTI` and `PROTO_GET_CRC` prior to receiving `PROTO_BOOT`.

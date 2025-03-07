# Bootloader for the Pixhawk board family

[![GitHub Actions Status](https://github.com/PX4/PX4-Bootloader/actions/workflows/main.yaml/badge.svg)](https://github.com/PX4/PX4-Bootloader/actions/workflows/main.yaml)

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

License: LGPL3 for libopencm3, BSD-3-clause for core bootloader (see [LICENSE.md](LICENSE.md))

## Contact

  * Chat: [Dronecode Discord](https://discord.gg/dronecode)
  * Forum: [Dronecode Discuss](https://discuss.px4.io)

## Bootloader Usage

The typical use case as used for the the PX4IO is described in [px4pipbl.pdf](https://github.com/PX4/Bootloader/files/3955700/px4pipbl.pdf).

To avoid accidental erasure or bad image loading:

- The bootloader needs to receive `PROTO_GET_SYNC` and `PROTO_GET_DEVICE` prior to receiving `PROTO_CHIP_ERASE`.
- The bootloader needs to receive `PROTO_GET_SYNC` and `PROTO_GET_DEVICE` and `PROTO_PROG_MULTI` and `PROTO_GET_CRC` prior to receiving `PROTO_BOOT`.

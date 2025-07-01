# Bootloader for PX4 on STM32F1/F3/F4/F7

[![GitHub Actions Status](https://github.com/PX4/PX4-Bootloader/actions/workflows/main.yaml/badge.svg)](https://github.com/PX4/PX4-Bootloader/actions/workflows/main.yaml)

**Note that bootloaders for STM32H7 are in [PX4](https://github.com/PX4/PX4-Autopilot) tree.**

This repository is just for legacy bootloaders on STM32F1/F3/F4/F7.

## Board IDs board_types.txt

This repository also serves to keep track of board IDs for PX4 in [board_types.txt](https://github.com/PX4/PX4-Bootloader/blob/main/board_types.txt).

It should be kept in sync with [ArduPilot's board_types.txt](https://github.com/ArduPilot/ardupilot/blob/master/Tools/AP_Bootloader/board_types.txt) as much as possible.

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

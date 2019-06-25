# Bootloaders for the Pixhawk board family

[![Build Status](https://travis-ci.org/PX4/Bootloader.svg?branch=master)](https://travis-ci.org/PX4/Bootloader)

## Build instructions

Build all targets:

```
git submodule init
git submodule update
make
```

The binaries will be in `build/BOARDNAME/BOARDNAME.elf`. Two files are built: ELF files for use with JTAG adapters and BIN files for direct onboard upgrading.

Build a specific board: Please check the `Makefile` for specific build targets.

## License

License: LGPL for libopencm3, BSD for core bootloader (see LICENSE.md)

## Contact

  * Chat: [Dronecode Slack](http://slack.px4.io)
  * Forum: [Dronecode Discuss](http://discuss.px4.io)

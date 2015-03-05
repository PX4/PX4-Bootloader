#
# Common Makefile for the PX4 bootloaders
#

#
# Paths to common dependencies
#
export BL_BASE		?= $(wildcard .)
export LIBOPENCM3	?= $(wildcard libopencm3)

#
# Tools
#
export CC	 	 = arm-none-eabi-gcc
export OBJCOPY		 = arm-none-eabi-objcopy

#
# Common configuration
#
export FLAGS		 = -std=gnu99 \
			   -Os \
			   -g \
			   -Wundef \
			   -Wall \
			   -fno-builtin \
			   -I$(LIBOPENCM3)/include \
			   -ffunction-sections \
			   -nostartfiles \
			   -lnosys \
			   -Wl,-gc-sections \
			   -Werror

export COMMON_SRCS	 = bl.c

#
# Bootloaders to build
#
TARGETS			 = px4fmu_bl px4fmuv2_bl px4flow_bl stm32f4discovery_bl px4io_bl aerocore_bl mavstation_bl

# px4io_bl px4flow_bl

all:	$(TARGETS)


clean:
	rm -f *.elf *.bin

#
# Specific bootloader targets.
#
# Pick a Makefile from Makefile.f1, Makefile.f4
# Pick an interface supported by the Makefile (USB, UART, I2C)
# Specify the board type.
#

px4fmu_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make -f Makefile.f4 TARGET=fmu INTERFACE=USB BOARD=FMU USBDEVICESTRING="\\\"PX4 BL FMU v1.x\\\"" USBPRODUCTID="0x0010"

px4fmuv2_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make -f Makefile.f4 TARGET=fmuv2 INTERFACE=USB BOARD=FMUV2 USBDEVICESTRING="\\\"PX4 BL FMU v2.x\\\"" USBPRODUCTID="0x0011"

stm32f4discovery_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make -f Makefile.f4 TARGET=discovery INTERFACE=USB BOARD=DISCOVERY USBDEVICESTRING="\\\"PX4 BL DISCOVERY\\\"" USBPRODUCTID="0x0001"

px4flow_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make -f Makefile.f4 TARGET=flow INTERFACE=USB BOARD=FLOW USBDEVICESTRING="\\\"PX4 FLOW v1.3\\\"" USBPRODUCTID="0x0015"

aerocore_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make -f Makefile.f4 TARGET=aerocore INTERFACE=USB BOARD=AEROCORE USBDEVICESTRING="\\\"Gumstix BL AEROCORE\\\"" USBPRODUCTID="0x1001"

# Default bootloader delay is *very* short, just long enough to catch
# the board for recovery but not so long as to make restarting after a 
# brownout problematic.
#
px4io_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make -f Makefile.f1 LINKER_FILE=stm32f1.ld F1_APP_LOAD_ADDRESS=0x08001000 F1_APP_SIZE_MAX=0xf000 TARGET=io INTERFACE=USART BOARD=IO PX4_BOOTLOADER_DELAY=200

mavstation_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make -f Makefile.f1 LINKER_FILE=12K-stm32f1.ld F1_APP_LOAD_ADDRESS=0x08003000 F1_APP_SIZE_MAX=0x1C000 TARGET=mavstation INTERFACE=USB BOARD=MAVSTATION USBDEVICESTRING="\\\"MAVSTATION v0.1\\\"" USBPRODUCTID="0x0014"

#
# Binary management
#
.PHONY: deploy
deploy:
	zip Bootloader.zip *.bin

#
# Submodule management
#

$(LIBOPENCM3): checksubmodules
	make -C $(LIBOPENCM3) lib

.PHONY: checksubmodules
checksubmodules: updatesubmodules
	$(Q) ($(BL_BASE)/Tools/check_submodules.sh)

.PHONY: updatesubmodules
updatesubmodules:
	$(Q) (git submodule init)
	$(Q) (git submodule update)

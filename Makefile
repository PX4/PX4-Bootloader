#
# Common Makefile for the PX4 bootloaders
#

# This option runs the standard link-time optimizer (lto).
# To learn more, see "Using the GNU Compiler Collection".
#FLTO = -flto

# newlib-nano
#NANO = --specs=nano.specs

#
# Paths to common dependencies
#
export BL_BASE		?= $(wildcard .)
export LIBOPENCM3	?= $(wildcard libopencm3)

#
# Tools
#
export CC	 	 = arm-none-eabi-gcc $(FLTO)
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
			   --specs=nosys.specs \
			   -Wl,-gc-sections \
			   -Werror $(NANO)

export COMMON_SRCS	 = bl.c cdcacm.c  usart.c

#
# Bootloaders to build
#
TARGETS			 = px4fmu_bl px4fmuv2_bl px4flow_bl px4discovery_bl px4aerocore_bl px4io_bl px4mavstation_bl

# px4io_bl px4flow_bl

all:	$(TARGETS)

distclean: clean
	make -C $(LIBOPENCM3) clean

clean:
	rm -f *.elf *.bin

#
# Specific bootloader targets.
#

px4fmu_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make -f Makefile.f4 TARGET_HW=PX4_FMU_V1 LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4fmuv2_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make -f Makefile.f4 TARGET_HW=PX4_FMU_V2  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4discovery_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make -f Makefile.f4 TARGET_HW=PX4_DISCOVERY_V1  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4flow_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make -f Makefile.f4 TARGET_HW=PX4_FLOW_V1  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4aerocore_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make -f Makefile.f4 TARGET_HW=PX4_AEROCORE_V1 LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

# Default bootloader delay is *very* short, just long enough to catch
# the board for recovery but not so long as to make restarting after a 
# brownout problematic.
#
px4io_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make -f Makefile.f1 TARGET_HW=PX4_PIO_V1 LINKER_FILE=stm32f1.ld TARGET_FILE_NAME=$@

px4mavstation_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make -f Makefile.f1 TARGET_HW=PX4_MAVSTATION_V1 LINKER_FILE=12K-stm32f1.ld TARGET_FILE_NAME=$@

#
# Binary management
#
.PHONY: deploy
deploy:
	zip Bootloader.zip *.bin

#
# Submodule management
#

# What better place for "cp"? here? checksubmodules (last line)? updatesubmodules (last line)?
$(LIBOPENCM3): checksubmodules
	cp locm3/Makefile.0 libopencm3/Makefile
	cp locm3/Makefile.1 libopencm3/lib/stm32/f1/Makefile
	cp locm3/Makefile.4 libopencm3/lib/stm32/f4/Makefile
	make -C $(LIBOPENCM3) FLTO=$(FLTO) NANO=$(NANO) lib

.PHONY: checksubmodules
checksubmodules: updatesubmodules
	$(Q) ($(BL_BASE)/Tools/check_submodules.sh)

.PHONY: updatesubmodules
updatesubmodules:
	$(Q) (git submodule init)
	$(Q) (git submodule update)

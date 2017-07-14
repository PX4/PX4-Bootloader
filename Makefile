#
# Common Makefile for the PX4 bootloaders
#

#
# Paths to common dependencies
#
export BL_BASE		?= $(wildcard .)
export LIBOPENCM3	?= $(wildcard libopencm3)
MKFLAGS=--no-print-directory
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
			   -Wl,-g \
			   -Werror

export COMMON_SRCS	 = bl.c cdcacm.c  usart.c

#
# Bootloaders to build
#
TARGETS	= \
	aerofcv1_bl \
	auavx2v1_bl \
	crazyflie_bl \
	mindpxv2_bl \
	px4aerocore_bl \
	px4discovery_bl \
	px4flow_bl \
	px4fmu_bl \
	px4fmuv2_bl \
	px4fmuv4_bl \
	px4fmuv4pro_bl \
	px4fmuv5_bl \
	px4io_bl \
	px4iov3_bl \
	tapv1_bl

all:	$(TARGETS) sizes

clean:
	cd libopencm3 && make --no-print-directory clean && cd ..
	rm -f *.elf *.bin # Remove any elf or bin files contained directly in the Bootloader directory
	rm -rf build_* # Remove build directories

#
# Specific bootloader targets.
#

auavx2v1_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=AUAV_X2V1  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4fmu_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FMU_V1 LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4fmuv2_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FMU_V2  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4fmuv4_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FMU_V4  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4fmuv4pro_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FMU_V4_PRO LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@ EXTRAFLAGS=-DSTM32F469

px4fmuv5_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=PX4_FMU_V5 LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

mindpxv2_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=MINDPX_V2 LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4discovery_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_DISCOVERY_V1  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4flow_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FLOW_V1  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4aerocore_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_AEROCORE_V1 LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

crazyflie_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=CRAZYFLIE LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

# Default bootloader delay is *very* short, just long enough to catch
# the board for recovery but not so long as to make restarting after a
# brownout problematic.
#
px4io_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f1 TARGET_HW=PX4_PIO_V1 LINKER_FILE=stm32f1.ld TARGET_FILE_NAME=$@

px4iov3_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f3 TARGET_HW=PX4_PIO_V3 LINKER_FILE=stm32f3.ld TARGET_FILE_NAME=$@

tapv1_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=TAP_V1 LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

aerofcv1_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=AEROFC_V1 LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

#
# Show sizes
#
.PHONY: sizes
sizes:
	@-find build_* -name '*.elf' -type f | xargs size 2> /dev/null || :

#
# Binary management
#
.PHONY: deploy
deploy:
	zip -j Bootloader.zip build_*/*.bin

#
# Submodule management
#

$(LIBOPENCM3): checksubmodules
	${MAKE} -C $(LIBOPENCM3) lib

.PHONY: checksubmodules
checksubmodules: updatesubmodules
	$(Q) ($(BL_BASE)/Tools/check_submodules.sh)

.PHONY: updatesubmodules
updatesubmodules:
	$(Q) (git submodule init)
	$(Q) (git submodule update)

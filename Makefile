#
# Common Makefile for the PX4 bootloaders
#

#
# Paths to common dependencies
#
export BUILD_DIR_ROOT ?= build
export BL_BASE		?= $(wildcard .)
export LIBOPENCM3	?= $(wildcard libopencm3)
MKFLAGS=--no-print-directory
#
# Tools
#
export CC	 	 = arm-none-eabi-gcc
export OBJCOPY		 = arm-none-eabi-objcopy

USBS_SRC=usbs_src
#
# Common configuration
#
export FLAGS		 = -std=gnu99 \
			   -Os \
			   -g \
			   -Wundef \
			   -Wall \
			   -fno-builtin \
			   -I$(BL_BASE)/$(LIBOPENCM3)/include \
			   -I$(BL_BASE)/. \
			   -ffunction-sections \
			   -nostartfiles \
			   -lnosys \
			   -Wl,-gc-sections \
			   -Wl,-g \
			   -Werror

export COMMON_SRCS	 = bl.c
export ARCH_SRCS	 = cdcacm.c  usart.c
export COMMON_SRCS_USBS	 = $(USBS_SRC)/usbs_bl.c $(USBS_SRC)/cdcacm.c  $(USBS_SRC)/usart.c $(USBS_SRC)/safe.c $(USBS_SRC)/tea.c
export USBS_BL_DIR=$(shell pwd)
px4_dir=$(shell pwd)

#
# Bootloaders to build
#
TARGETS	= \
	nxphlitev3_bl \
	aerofcv1_bl \
	auavx2v1_bl \
	crazyflie_bl \
	mindpxv2_bl \
	omnibusf4sd_bl \
	px4aerocore_bl \
	px4discovery_bl \
	px4flow_bl \
	px4fmu_bl \
	px4fmuv2_bl \
	px4fmuv3_bl \
	px4fmuv4_bl \
	px4fmuv4pro_bl \
	px4fmuv5_bl \
	px4io_bl \
	px4iov3_bl \
	tapv1_bl \
	cube_f4_bl \
	cube_f7_bl \
	usbs_px4fmuv4_bl \
	usbs_f4_bl \
	usbs_f4_bl_update \
	usbs_px4fmuv4_bl_update \
	usbs_px4io_bl \
	music_play_bl \
	music_play_bl_update \
	music_play_bl_update_enc \
	usbs_f1_bl \
	usbs_f1_bl_update \
	usbs_f1_bl_update_enc \
	music_play_12Kbl \
	music_play_16Kbl 

all:	$(TARGETS) sizes

clean:
	cd libopencm3 && make --no-print-directory clean && cd ..
	rm -f *.elf *.bin # Remove any elf or bin files contained directly in the Bootloader directory
	rm -rf build # Remove build directories

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

usbs_px4fmuv4_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	@mkdir -p build/bl
	@mkdir -p build/$@/$(USBS_SRC)
	@rm -rf $(USBS_SRC)
	@cp -a USBS/$@ $(USBS_SRC)
	@cp -a USBS/core/* $(USBS_SRC)/
	${MAKE} ${MKFLAGS} -f  $(USBS_SRC)/Makefile.f4 TARGET_HW=USBS_FMU_V4 LINKER_FILE=$(USBS_SRC)/stm32f4.ld TARGET_FILE_NAME=$@ USBS_SRC_DIR=$(USBS_SRC)
	@cp build/$@/$@.bin build/bl/$@_08000000.bin
	@rm -rf $(USBS_SRC)
usbs_px4fmuv4_bl_update: usbs_px4fmuv4_bl $(MAKEFILE_LIST) $(LIBOPENCM3)
	@mkdir -p build/bl
	@rm -rf build/bl/*.data
	@cp build/usbs_px4fmuv4_bl/usbs_px4fmuv4_bl.bin build/bl/usbs_bl.data
	@mkdir -p build/$@/$(USBS_SRC)
	@rm -rf $(USBS_SRC)
	@cp -a USBS/$@ $(USBS_SRC)
	@cp -a USBS/core/* $(USBS_SRC)/
	${MAKE} ${MKFLAGS} -f  $(USBS_SRC)/Makefile.f4 TARGET_HW=USBS_FMU_V4 LINKER_FILE=$(USBS_SRC)/stm32f4.ld TARGET_FILE_NAME=$@ USBS_SRC_DIR=$(USBS_SRC)
	$(px4_dir)//Tools/px_mkfw.py --prototype $(px4_dir)/Images//px4fmu-v4.prototype --git_identity $(px4_dir)/ --image build/$@/$@.bin > build/bl/$@.px4
	@cp build/$@/$@.bin build/bl/$@_08004000.bin
	@rm -rf $(USBS_SRC)
	@rm -rf build/bl/*.data
usbs_f4_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	@mkdir -p build/bl
	@mkdir -p build/$@/$(USBS_SRC)
	@rm -rf $(USBS_SRC)
	@cp -a USBS/$@ $(USBS_SRC)
	@cp -a USBS/core/* $(USBS_SRC)/
	${MAKE} ${MKFLAGS} -f  $(USBS_SRC)/Makefile.f4 TARGET_HW=USBS_FMU_V4 LINKER_FILE=$(USBS_SRC)/stm32f4.ld TARGET_FILE_NAME=$@ USBS_SRC_DIR=$(USBS_SRC)
	@cp build/$@/$@.bin build/bl/$@_08000000.bin
	@rm -rf $(USBS_SRC)
usbs_f4_bl_update: usbs_f4_bl $(MAKEFILE_LIST) $(LIBOPENCM3)
	@mkdir -p build/bl
	@rm -rf build/bl/*.data
	@cp build/$</$<.bin build/bl/usbs_bl.data
	@mkdir -p build/$@/$(USBS_SRC)
	@rm -rf $(USBS_SRC)
	@cp -a USBS/$@ $(USBS_SRC)
	@cp -a USBS/core/* $(USBS_SRC)/
	@cp -a USBS/update/* $(USBS_SRC)/
	${MAKE} ${MKFLAGS} -f  $(USBS_SRC)/Makefile.f4 TARGET_HW=USBS_FMU_V4 LINKER_FILE=$(USBS_SRC)/stm32f4.ld TARGET_FILE_NAME=$@ USBS_SRC_DIR=$(USBS_SRC)
	$(px4_dir)//Tools/px_mkfw.py --prototype $(px4_dir)/Images//px4fmu-v4.prototype --git_identity $(px4_dir)/ --image build/$@/$@.bin > build/bl/$@.px4
	@cp build/$@/$@.bin build/bl/$@_08004000.bin
	@rm -rf $(USBS_SRC)
	@rm -rf build/bl/*.data
usbs_f1_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	@mkdir -p build/bl
	@mkdir -p build/$@/$(USBS_SRC)
	@rm -rf $(USBS_SRC)
	@cp -a USBS/$@ $(USBS_SRC)
	@cp -a USBS/core/* $(USBS_SRC)/
	${MAKE} ${MKFLAGS} -f  USBS/$@/Makefile.f1 TARGET_HW=MUSIC_PLAY LINKER_FILE=USBS/$@/stm32f1.ld TARGET_FILE_NAME=$@ USBS_SRC_DIR=$(USBS_SRC)
	@cp build/$@/$@.bin build/bl/$@_08000000.bin
	@rm -rf $(USBS_SRC)
usbs_f1_bl_update: usbs_f1_bl $(MAKEFILE_LIST) $(LIBOPENCM3)
	@mkdir -p build/bl
	@rm -rf build/bl/*.data
	@cp build/$</$<.bin build/bl/usbs_bl.data
	@mkdir -p build/$@/$(USBS_SRC)
	@rm -rf $(USBS_SRC)
	@cp -a USBS/$@ $(USBS_SRC)
	@cp -a USBS/core/* $(USBS_SRC)/
	@cp -a USBS/update/* $(USBS_SRC)/
	${MAKE} ${MKFLAGS} -f  USBS/$@/Makefile.f1 TARGET_HW=MUSIC_PLAY LINKER_FILE=USBS/$@/stm32f1.ld TARGET_FILE_NAME=$@ USBS_SRC_DIR=$(USBS_SRC)
	$(px4_dir)//Tools/px_mkfw.py --prototype $(px4_dir)/Images//px4fmu-v4.prototype --git_identity $(px4_dir)/ --image build/$@/$@.bin > build/bl/$@.px4
	@cp build/$@/$@.bin build/bl/$@_08003000.bin
	@rm -rf $(USBS_SRC)
	@rm -rf build/bl/*.data
usbs_f1_bl_update_enc: usbs_f1_bl $(MAKEFILE_LIST) $(LIBOPENCM3)
	@mkdir -p build/bl
	@rm -rf build/bl/*.data
	@cp build/$</$<.bin build/bl/usbs_bl.data
	@mkdir -p build/$@/$(USBS_SRC)
	@rm -rf $(USBS_SRC)
	@cp -a USBS/$@ $(USBS_SRC)
	@cp -a USBS/core/* $(USBS_SRC)/
	@cp -a USBS/update/* $(USBS_SRC)/
	${MAKE} ${MKFLAGS} -f  USBS/$@/Makefile.f1 TARGET_HW=MUSIC_PLAY LINKER_FILE=USBS/$@/stm32f1.ld TARGET_FILE_NAME=$@ USBS_SRC_DIR=$(USBS_SRC)
	$(px4_dir)//Tools/px_mkfw.py --prototype $(px4_dir)/Images//px4fmu-v4.prototype --git_identity $(px4_dir)/ --image build/$@/$@.bin > build/bl/$@.px4
	@cp build/$@/$@.bin build/bl/$@_08003000.bin
	@rm -rf $(USBS_SRC)
	@rm -rf build/bl/*.data

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

cube_f4_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=CUBE_F4  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

cube_f7_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=CUBE_F7 LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

# Default bootloader delay is *very* short, just long enough to catch
# the board for recovery but not so long as to make restarting after a
# brownout problematic.
#
px4io_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f1 TARGET_HW=PX4_PIO_V1 LINKER_FILE=stm32f1.ld TARGET_FILE_NAME=$@

music_play_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	@mkdir -p build/bl
	@mkdir -p build/$@/$(USBS_SRC)
	@rm -rf $(USBS_SRC)
	@cp -a USBS/$@ $(USBS_SRC)
	@cp -a USBS/core/* $(USBS_SRC)/
#	${MAKE} ${MKFLAGS} -f  $(USBS_SRC)/Makefile.f4 TARGET_HW=USBS_FMU_V4 LINKER_FILE=$(USBS_SRC)/stm32f4.ld TARGET_FILE_NAME=$@ USBS_SRC_DIR=$(USBS_SRC)
	${MAKE} ${MKFLAGS} -f  USBS/$@/Makefile.f1 TARGET_HW=MUSIC_PLAY LINKER_FILE=USBS/$@/stm32f1.ld TARGET_FILE_NAME=$@ USBS_SRC_DIR=$(USBS_SRC)
	@cp build/$@/$@.bin build/bl/$@_08000000.bin
	@rm -rf $(USBS_SRC)
music_play_bl_update: music_play_bl $(MAKEFILE_LIST) $(LIBOPENCM3)
	@mkdir -p build/bl
	@rm -rf build/bl/*.data
	@cp build/$</$<.bin build/bl/usbs_bl.data
	@mkdir -p build/$@/$(USBS_SRC)
	@rm -rf $(USBS_SRC)
	@cp -a USBS/$@ $(USBS_SRC)
	@cp -a USBS/core/* $(USBS_SRC)/
	${MAKE} ${MKFLAGS} -f  USBS/$@/Makefile.f1 TARGET_HW=MUSIC_PLAY LINKER_FILE=USBS/$@/stm32f1.ld TARGET_FILE_NAME=$@ USBS_SRC_DIR=$(USBS_SRC)
	$(px4_dir)//Tools/px_mkfw.py --prototype $(px4_dir)/Images//px4fmu-v4.prototype --git_identity $(px4_dir)/ --image build/$@/$@.bin > build/bl/$@.px4
	@cp build/$@/$@.bin build/bl/$@_08003000.bin
	@rm -rf $(USBS_SRC)
	@rm -rf build/bl/*.data
music_play_bl_update_enc: music_play_bl $(MAKEFILE_LIST) $(LIBOPENCM3)
	@mkdir -p build/bl
	@rm -rf build/bl/*.data
	@cp build/$</$<.bin build/bl/usbs_bl.data
	@mkdir -p build/$@/$(USBS_SRC)
	@rm -rf $(USBS_SRC)
	@cp -a USBS/$@ $(USBS_SRC)
	@cp -a USBS/core/* $(USBS_SRC)/
	${MAKE} ${MKFLAGS} -f  USBS/$@/Makefile.f1 TARGET_HW=MUSIC_PLAY LINKER_FILE=USBS/$@/stm32f1.ld TARGET_FILE_NAME=$@ USBS_SRC_DIR=$(USBS_SRC)
	$(px4_dir)//Tools/px_mkfw.py --prototype $(px4_dir)/Images//px4fmu-v4.prototype --git_identity $(px4_dir)/ --image build/$@/$@.bin > build/bl/$@.px4
	@cp build/$@/$@.bin build/bl/$@_08003000.bin
	@rm -rf $(USBS_SRC)
	@rm -rf build/bl/*.data
music_play_12Kbl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	@mkdir -p build/bl
	@mkdir -p build/$@/$(USBS_SRC)
	@rm -rf $(USBS_SRC)
	@cp -a USBS/$@ $(USBS_SRC)
	@cp -a USBS/core/* $(USBS_SRC)/
	${MAKE} ${MKFLAGS} -f  USBS/$@/Makefile.f1 TARGET_HW=MUSIC_PLAY12K LINKER_FILE=USBS/$@/stm32f1.ld TARGET_FILE_NAME=$@ USBS_SRC_DIR=$(USBS_SRC)
	@cp build/$@/$@.bin build/bl/$@_08000000.bin
	@rm -rf $(USBS_SRC)
music_play_16Kbl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	@mkdir -p build/bl
	@mkdir -p build/$@/$(USBS_SRC)
	@rm -rf $(USBS_SRC)
	@cp -a USBS/$@ $(USBS_SRC)
	@cp -a USBS/core/* $(USBS_SRC)/
	${MAKE} ${MKFLAGS} -f  USBS/$@/Makefile.f1 TARGET_HW=MUSIC_PLAY16K LINKER_FILE=USBS/$@/stm32f1.ld TARGET_FILE_NAME=$@ USBS_SRC_DIR=$(USBS_SRC)
	@cp build/$@/$@.bin build/bl/$@_08000000.bin
	@rm -rf $(USBS_SRC)


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
#	@-find build_* -name '*.elf' -type f | xargs size 2> /dev/null || :
	@-find build/*/ -name '*.elf' -type f | xargs size 2> /dev/null || :

#
# Binary management
#
.PHONY: deploy
deploy:
	zip -j Bootloader.zip build/*/*.bin

#
# Submodule management
#

$(LIBOPENCM3): checksubmodules
	${MAKE} -C $(LIBOPENCM3) lib

.PHONY: checksubmodules
checksubmodules:
	$(Q) ($(BL_BASE)/Tools/check_submodules.sh)

.PHONY: updatesubmodules
updatesubmodules:
	$(Q) (git submodule init)
	$(Q) (git submodule update)

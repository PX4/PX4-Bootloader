#
# Common Makefile for the PX4 bootloaders
#

#
# Paths to common dependencies
#
export BUILD_DIR_ROOT ?= build
export BL_BASE		?= $(wildcard .)
export LIBOPENCM3	?= $(wildcard libopencm3)
export LIBKINETIS  	?= $(wildcard lib/kinetis/NXP_Kinetis_Bootloader_2_0_0)
export LIBCRYPTO  	?= $(wildcard monocypher)
MKFLAGS=--no-print-directory

SRC_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

COLOR_BLUE = \033[0;94m
NO_COLOR   = \033[m

define colorecho
+@echo -e '${COLOR_BLUE}${1} ${NO_COLOR}'
endef

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
			   -I$(BL_BASE)/$(LIBOPENCM3)/include \
			   -I$(BL_BASE)/$(LIBCRYPTO)/src\
			   -I$(BL_BASE)/. \
			   -ffunction-sections \
			   -nostartfiles \
			   -lnosys \
			   -Wl,-gc-sections \
			   -Wl,-g \
			   -Werror \
			   -Xlinker -Map=output.map 

export COMMON_SRCS	 = bl.c crypto.c \
						monocypher/src/monocypher.c\
						monocypher/src/sha512.c
					
export ARCH_SRCS	 = cdcacm.c  usart.c

#
# Bootloaders to build
# Note: px4fmuv3_bl is the same as px4fmuv2_bl except for a different USB device
# string
#
TARGETS	= \
	aerofcv1_bl \
	auavx2v1_bl \
	avx_v1_bl \
	crazyflie_bl \
	cube_f4_bl \
	fmuk66v3_bl \
	kakutef7_bl \
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
	px4fmuv5_crypto_bl \
	px4io_bl \
	px4iov3_bl \
	tapv1_bl \
	smartap_pro_bl

all:	$(TARGETS) sizes

clean:
	cd libopencm3 && make --no-print-directory clean && cd ..
	rm -f *.elf *.bin # Remove any elf or bin files contained directly in the Bootloader directory
	rm -rf build # Remove build directories

#
# Specific bootloader targets.
#

fmuk66v3_bl: $(MAKEFILE_LIST) $(LIBKINETIS)
	${MAKE} ${MKFLAGS} -f  Makefile.k66 TARGET_HW=FMUK66_V3  LINKER_FILE=kinetisk66.ld TARGET_FILE_NAME=$@

auavx2v1_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=AUAV_X2V1  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

kakutef7_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=KAKUTEF7 LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

px4fmu_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FMU_V1 LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4fmuv2_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FMU_V2  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4fmuv3_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FMU_V3  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4fmuv4_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FMU_V4  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4fmuv4pro_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FMU_V4_PRO LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@ EXTRAFLAGS=-DSTM32F469

px4fmuv5_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=PX4_FMU_V5 LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@
	
px4fmuv5_crypto_bl:$(MAKEFILE_LIST) $(LIBOPENCM3) $(LIBCRYPTO)
	${MAKE} ${MKFLAGS} -f  Makefile.f7.crypto TARGET_HW=PX4_FMU_V5_CRYPTO LINKER_FILE=stm32f7_crypto.ld TARGET_FILE_NAME=$@

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

omnibusf4sd_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=OMNIBUSF4SD LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

cube_f4_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=CUBE_F4  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

cube_f7_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=CUBE_F7 LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

avx_v1_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=AV_X_V1 LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

smartap_pro_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=SMARTAP_PRO LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

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

# Astyle
# --------------------------------------------------------------------
.PHONY: check_format format

check_format:
	$(call colorecho,'Checking formatting with astyle')
	@$(SRC_DIR)/Tools/check_code_style_all.sh
	@cd $(SRC_DIR) && git diff --check

format:
	$(call colorecho,'Formatting with astyle')
	@$(SRC_DIR)/Tools/check_code_style_all.sh --fix

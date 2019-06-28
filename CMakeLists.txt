############################################################################
#
# Copyright (c) 2019 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

set(PX4_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(PX4_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

#=============================================================================
# git
#
execute_process(
	COMMAND git describe --always --tags
	OUTPUT_VARIABLE PX4_GIT_TAG
	OUTPUT_STRIP_TRAILING_WHITESPACE
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	)

#=============================================================================
# configuration
#

# find PX4 config
#  look for in tree board config that matches CONFIG input
if(NOT PX4_CONFIG_FILE)

	file(GLOB_RECURSE board_configs
		RELATIVE "${CMAKE_SOURCE_DIR}/boards"
		"boards/*.cmake"
		)

	set(PX4_CONFIGS ${board_configs} CACHE STRING "PX4 board configs" FORCE)

	foreach(filename ${board_configs})
		# parse input CONFIG into components to match with existing in tree configs
		#  the platform prefix (eg nuttx_) is historical, and removed if present
		string(REPLACE ".cmake" "" filename_stripped ${filename})
		string(REPLACE "/" ";" config ${filename_stripped})
		list(LENGTH config config_len)

		if(${config_len} EQUAL 3)


			list(GET config 0 vendor)
			list(GET config 1 model)
			list(GET config 2 label)

			set(board "${vendor}${model}")

			# <VENDOR>_<MODEL>_<LABEL> (eg px4_fmu-v2_default)
			# <VENDOR>_<MODEL>_default (eg px4_fmu-v2) # allow skipping label if "default"
			if ((${CONFIG} MATCHES "${vendor}_${model}_${label}") OR # match full vendor, model, label
			    ((${label} STREQUAL "default") AND (${CONFIG} STREQUAL "${vendor}_${model}")) # default label can be omitted
			)
				set(PX4_CONFIG_FILE "${CMAKE_SOURCE_DIR}/boards/${filename}" CACHE FILEPATH "path to PX4 CONFIG file" FORCE)
				break()
			endif()

			# <BOARD>_<LABEL> (eg px4_fmu-v2_default)
			# <BOARD>_default (eg px4_fmu-v2) # allow skipping label if "default"
			if ((${CONFIG} MATCHES "${board}_${label}") OR # match full board, label
			    ((${label} STREQUAL "default") AND (${CONFIG} STREQUAL "${board}")) # default label can be omitted
			)
				set(PX4_CONFIG_FILE "${CMAKE_SOURCE_DIR}/boards/${filename}" CACHE FILEPATH "path to PX4 CONFIG file" FORCE)
				break()
			endif()
		endif()
	endforeach()
endif()

if(NOT PX4_CONFIG_FILE)
	message(FATAL_ERROR "PX4 config file not set, try one of ${PX4_CONFIGS}")
endif()

if(NOT PX4_CONFIG_FILE)
	message(FATAL_ERROR "PX4 config file not set, try one of ${PX4_CONFIGS}")
endif()

message(STATUS "PX4 config file: ${PX4_CONFIG_FILE}")
include(px4_add_bootloader)
include(${PX4_CONFIG_FILE})
message(STATUS "PX4 config: ${PX4_CONFIG}")

# CMake build type (Debug Release RelWithDebInfo MinSizeRel)
if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "MinSizeRel" CACHE STRING "Build type" FORCE)
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel;")

#=============================================================================

message(STATUS "PX4 version: ${PX4_GIT_TAG}")
message(STATUS "cmake build type: ${CMAKE_BUILD_TYPE}")

#=============================================================================
# project definition
#

#set(CMAKE_SYSTEM_PROCESSOR "cortex-m7" CACHE INTERNAL "system processor" FORCE)
#set(CMAKE_TOOLCHAIN_FILE cmake/Toolchain-arm-none-eabi.cmake CACHE INTERNAL "toolchain file" FORCE)

project(px4_bootloader CXX C ASM)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


#=============================================================================
# libopencm3
#
include(px4_git)
px4_add_git_submodule(TARGET git_libopencm3 PATH "libopencm3")

# stm32f1
add_custom_command(
	OUTPUT ${CMAKE_SOURCE_DIR}/libopencm3/lib/libopencm3_stm32f1.a
	COMMAND make --silent --no-print-directory -C ${CMAKE_SOURCE_DIR}/libopencm3 lib/stm32/f1
	DEPENDS git_libopencm3
	USES_TERMINAL
)
add_library(opencm3_stm32f1 STATIC IMPORTED GLOBAL)
set_target_properties(opencm3_stm32f1 PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/libopencm3/lib/libopencm3_stm32f1.a)
add_custom_target(opencm3_stm32f1_build DEPENDS ${CMAKE_SOURCE_DIR}/libopencm3/lib/libopencm3_stm32f1.a)

# stm32f4
add_custom_command(
	OUTPUT ${CMAKE_SOURCE_DIR}/libopencm3/lib/libopencm3_stm32f4.a
	COMMAND make --silent --no-print-directory -C ${CMAKE_SOURCE_DIR}/libopencm3 lib/stm32/f4
	DEPENDS git_libopencm3
	USES_TERMINAL
)
add_library(opencm3_stm32f4 STATIC IMPORTED GLOBAL)
set_target_properties(opencm3_stm32f4 PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/libopencm3/lib/libopencm3_stm32f4.a)
add_custom_target(opencm3_stm32f4_build DEPENDS ${CMAKE_SOURCE_DIR}/libopencm3/lib/libopencm3_stm32f4.a)

# stm32f7
add_custom_command(
	OUTPUT ${CMAKE_SOURCE_DIR}/libopencm3/lib/libopencm3_stm32f7.a
	COMMAND make --silent --no-print-directory -C ${CMAKE_SOURCE_DIR}/libopencm3 lib/stm32/f7
	DEPENDS git_libopencm3
	USES_TERMINAL
)
add_library(opencm3_stm32f7 STATIC IMPORTED GLOBAL)
set_target_properties(opencm3_stm32f7 PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/libopencm3/lib/libopencm3_stm32f7.a)
add_custom_target(opencm3_stm32f7_build DEPENDS ${CMAKE_SOURCE_DIR}/libopencm3/lib/libopencm3_stm32f7.a)

#=============================================================================
# libkinetis
#
include(px4_git)
px4_add_git_submodule(TARGET git_libkinetis PATH "lib/kinetis/NXP_Kinetis_Bootloader_2_0_0")

set(LIBKINETIS_DIR ${CMAKE_SOURCE_DIR}/lib/kinetis/NXP_Kinetis_Bootloader_2_0_0)

#=============================================================================
# bootloader
#
add_compile_options(
	-Wall
	#-Wextra
	-Werror

	-Wundef

	-ffunction-sections
	-fno-builtin

	-nostartfiles
	-lnosys

	-g3
)

# compiler specific flags
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
	# force color for clang (needed for clang + ccache)
	add_compile_options(-fcolor-diagnostics)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9)
		# force color for gcc > 4.9
		add_compile_options(-fdiagnostics-color=always)
	endif()
endif()


include_directories(.)
include_directories(${PX4_BOARD_DIR})

set(FW_NAME ${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}.elf)

set(SRCS)
if(PX4_BOARD_ARCH MATCHES "stm32f1")
	include_directories(libopencm3/include)

	list(APPEND SRCS
		stm32/cdcacm.c
		stm32/usart.c

		main_f1.c
	)

	add_definitions(-DSTM32F1)
	set(LINKER_FILE ${CMAKE_SOURCE_DIR}/stm32f1.ld)

elseif(PX4_BOARD_ARCH MATCHES "stm32f4")
	include_directories(libopencm3/include)

	list(APPEND SRCS
		stm32/cdcacm.c
		stm32/usart.c

		main_f4.c
	)

	add_definitions(-DSTM32F4)
	set(LINKER_FILE ${CMAKE_SOURCE_DIR}/stm32f4.ld)

elseif(PX4_BOARD_ARCH MATCHES "stm32f7")
	include_directories(libopencm3/include)

	list(APPEND SRCS
		stm32/cdcacm.c
		stm32/usart.c

		main_f7.c
	)

	add_definitions(-DSTM32F7)
	set(LINKER_FILE ${CMAKE_SOURCE_DIR}/stm32f7.ld)

	set_source_files_properties(stm32/cdcacm.c PROPERTIES COMPILE_FLAGS -DSTM32F4)

elseif(PX4_BOARD_ARCH MATCHES "kinetis")

	# hack that should be cleaned up
	include_directories(libopencm3/include)

	include_directories(
		${LIBKINETIS_DIR}/src
		${LIBKINETIS_DIR}/src/autobaud
		${LIBKINETIS_DIR}/src/bm_usb
		${LIBKINETIS_DIR}/src/drivers
		${LIBKINETIS_DIR}/src/drivers/common
		${LIBKINETIS_DIR}/src/include
		${LIBKINETIS_DIR}/src/platform
		${LIBKINETIS_DIR}/src/platform/CMSIS/Include
		${LIBKINETIS_DIR}/src/platform/devices
		${LIBKINETIS_DIR}/src/platform/devices/MK66F18
		${LIBKINETIS_DIR}/src/startup
		${LIBKINETIS_DIR}/src/usb/device
		${LIBKINETIS_DIR}/src/usb/device/class
		${LIBKINETIS_DIR}/src/usb/include
		${LIBKINETIS_DIR}/src/usb/osa
		${LIBKINETIS_DIR}/targets/MK66F18/src
	)

	list(APPEND SRCS
		${LIBKINETIS_DIR}/src/drivers/flash/fsl_flash.c
		${LIBKINETIS_DIR}/src/drivers/gpio/fsl_gpio.c
		${LIBKINETIS_DIR}/src/drivers/smc/smc.c
		${LIBKINETIS_DIR}/src/platform/devices/MK66F18/fsl_clock.c
		${LIBKINETIS_DIR}/src/startup/startup.c
		${LIBKINETIS_DIR}/src/usb/device/class/usb_device_cdc_acm.c
		${LIBKINETIS_DIR}/src/usb/device/class/usb_device_class.c
		${LIBKINETIS_DIR}/src/usb/device/usb_device_ch9.c
		${LIBKINETIS_DIR}/src/usb/device/usb_device_dci.c
		${LIBKINETIS_DIR}/src/usb/device/usb_device_khci.c
		${LIBKINETIS_DIR}/src/usb/osa/usb_osa_bm.c
		${LIBKINETIS_DIR}/targets/MK66F18/src/startup/gcc/startup_MK66F18.S
		${LIBKINETIS_DIR}/targets/MK66F18/src/startup/system_MK66F18.c
	
		kinetis/cdcacm.c
		kinetis/usart.c
		kinetis.c
		main_k66.c
	)

	add_definitions(
		-DBL_CONFIG_HS_USB_HID=0
		-DBL_CONFIG_HS_USB_MSC=0
		-DBL_CONFIG_USB_CDC=1
		-DBL_TARGET_FLASH
		-DCPU_IS_ARM_CORTEX_M4=1
		-DCPU_MK66FN2M0VMD18=1
		-DFREEDOM=0
	
		-DDEBUG=0
		-DNDEBUG=1
	
		-DSTM32F4
		-DTOWER=0
		-DUSB_STACK_BM
	)

	set(LINKER_FILE ${CMAKE_SOURCE_DIR}/kinetisk66.ld)

endif()

add_definitions(-DTARGET_HW_${PX4_BOARD_NAME})

add_executable(px4-bootloader
	bl.c

	${SRCS}

)
set_target_properties(px4-bootloader PROPERTIES OUTPUT_NAME ${FW_NAME})

target_link_libraries(px4-bootloader PRIVATE

	-ffunction-sections
	-nostartfiles
	-lnosys

	-Wl,--script=${LINKER_FILE}
	-Wl,-Map=${PX4_CONFIG}.map
	-Wl,--warn-common
	-Wl,--gc-sections
)

if(PX4_BOARD_ARCH MATCHES "stm32f1")
	target_link_libraries(px4-bootloader PRIVATE opencm3_stm32f1)
	add_dependencies(px4-bootloader opencm3_stm32f1_build)
elseif(PX4_BOARD_ARCH MATCHES "stm32f4")
	target_link_libraries(px4-bootloader PRIVATE opencm3_stm32f4)
	add_dependencies(px4-bootloader opencm3_stm32f4_build)
elseif(PX4_BOARD_ARCH MATCHES "stm32f7")
	target_link_libraries(px4-bootloader PRIVATE opencm3_stm32f7 opencm3_stm32f4)
	add_dependencies(px4-bootloader opencm3_stm32f4_build)
	add_dependencies(px4-bootloader opencm3_stm32f7_build)
endif()


# .bin file
add_custom_command(
	OUTPUT ${CMAKE_BINARY_DIR}/${PX4_BOARD}.bin
	COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:px4-bootloader> ${CMAKE_BINARY_DIR}/${PX4_BOARD}.bin
	DEPENDS px4-bootloader
)
add_custom_target(px4-bl.binb DEPENDS ${CMAKE_BINARY_DIR}/${PX4_BOARD}.bin)

# .hex file
add_custom_command(
	OUTPUT ${CMAKE_BINARY_DIR}/${PX4_BOARD}.hex
	COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:px4-bootloader> ${CMAKE_BINARY_DIR}/${PX4_BOARD}.hex
	DEPENDS px4-bootloader
)
add_custom_target(px4-bl.hex DEPENDS ${CMAKE_BINARY_DIR}/${PX4_BOARD}.hex)


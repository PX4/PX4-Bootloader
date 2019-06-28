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

include(px4_base)

#=============================================================================
#
#	px4_add_bootloader
#
#	This function creates a PX4 bootloader.
#
#	Usage:
#		px4_add_bootloader(
#			PLATFORM <string>
#			VENDOR <string>
#			MODEL <string>
#			[ TOOLCHAIN <string> ]
#			[ ARCHITECTURE <string> ]
#			[ SERIAL_PORTS <list> ]
#			)
#
#	Input:
#		PLATFORM		: PX4 platform name (posix, nuttx, qurt)
#		VENDOR			: name of board vendor/manufacturer/brand/etc
#		MODEL			: name of board model
#		LABEL			: optional label, set to default if not specified
#		TOOLCHAIN		: cmake toolchain
#		ARCHITECTURE		: name of the CPU CMake is building for (used by the toolchain)
#		SERIAL_PORTS		: mapping of user configurable serial ports and param facing name
#
#
#	Example:
#		px4_add_bootloader(
#			VENDOR px4
#			MODEL fmu-v5
#			TOOLCHAIN arm-none-eabi
#			ARCHITECTURE cortex-m7
#			SERIAL_PORTS
#				GPS1:/dev/ttyS0
#				TEL1:/dev/ttyS1
#				TEL2:/dev/ttyS2
#				TEL4:/dev/ttyS3
#			)
#
function(px4_add_bootloader)

	px4_parse_function_args(
		NAME px4_add_bootloader
		ONE_VALUE
			VENDOR
			MODEL
			LABEL
			TOOLCHAIN
			ARCHITECTURE
		MULTI_VALUE
			SERIAL_PORTS
		OPTIONS
		REQUIRED
			VENDOR
			MODEL
		ARGN ${ARGN})

	set(PX4_BOARD_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE STRING "PX4 board directory" FORCE)

	set(PX4_BOARD ${VENDOR}_${MODEL} CACHE STRING "PX4 board" FORCE)

	# board name is uppercase with no underscores when used as a define
	string(TOUPPER ${PX4_BOARD} PX4_BOARD_NAME)
	string(REPLACE "-" "_" PX4_BOARD_NAME ${PX4_BOARD_NAME})
	set(PX4_BOARD_NAME ${PX4_BOARD_NAME} CACHE STRING "PX4 board define" FORCE)

	set(PX4_BOARD_VENDOR ${VENDOR} CACHE STRING "PX4 board vendor" FORCE)
	set(PX4_BOARD_MODEL ${MODEL} CACHE STRING "PX4 board model" FORCE)

	if(LABEL)
		set(PX4_BOARD_LABEL ${LABEL} CACHE STRING "PX4 board label" FORCE)
	else()
		set(PX4_BOARD_LABEL "bootloader" CACHE STRING "PX4 board label" FORCE)
	endif()

	set(PX4_CONFIG "${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_${PX4_BOARD_LABEL}" CACHE STRING "PX4 config" FORCE)

	if(ARCHITECTURE)
		if(ARCHITECTURE MATCHES "stm32f1")
			set(CMAKE_SYSTEM_PROCESSOR "cortex-m3" CACHE INTERNAL "system processor" FORCE)
			set(PX4_BOARD_ARCH "stm32f1" CACHE STRING "PX4 board architecture" FORCE)
		elseif(ARCHITECTURE MATCHES "stm32f4")
			set(CMAKE_SYSTEM_PROCESSOR "cortex-m4" CACHE INTERNAL "system processor" FORCE)
			set(PX4_BOARD_ARCH "stm32f4" CACHE STRING "PX4 board architecture" FORCE)
		elseif(ARCHITECTURE MATCHES "stm32f7")
			set(CMAKE_SYSTEM_PROCESSOR "cortex-m7" CACHE INTERNAL "system processor" FORCE)
			set(PX4_BOARD_ARCH "stm32f7" CACHE STRING "PX4 board architecture" FORCE)
		elseif(ARCHITECTURE MATCHES "kinetis")
			set(CMAKE_SYSTEM_PROCESSOR "cortex-m4" CACHE INTERNAL "system processor" FORCE)
			set(PX4_BOARD_ARCH "kinetis" CACHE STRING "PX4 board architecture" FORCE)
		endif()
	endif()

	if(TOOLCHAIN)
		set(CMAKE_TOOLCHAIN_FILE Toolchain-${TOOLCHAIN} CACHE INTERNAL "toolchain file" FORCE)
	endif()

	if(BOOTLOADER)
		set(config_bl_file ${BOOTLOADER} CACHE INTERNAL "bootloader" FORCE)
	endif()

	if(SERIAL_PORTS)
		set(board_serial_ports ${SERIAL_PORTS} PARENT_SCOPE)
	endif()

endfunction()

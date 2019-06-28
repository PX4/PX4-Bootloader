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

# Enforce the presence of the GIT repository
#
# We depend on our submodules, so we have to prevent attempts to
# compile without it being present.
ifeq ($(wildcard .git),)
    $(error YOU HAVE TO USE GIT TO DOWNLOAD THIS REPOSITORY. ABORTING.)
endif

# Help
# --------------------------------------------------------------------
# Don't be afraid of this makefile, it is just passing
# arguments to cmake to allow us to keep the wiki pages etc.
# that describe how to build the px4 firmware
# the same even when using cmake instead of make.
#
# Example usage:
#
# make px4_fmu-v2_bootloader 			(builds)
# make px4_fmu-v2_bootloader upload 	(builds and uploads)
# make px4_fmu-v2_bootloader test 		(builds and tests)
#
# This tells cmake to build the nuttx px4_fmu-v2 default config in the
# directory build/px4_fmu-v2_bootloader and then call make
# in that directory with the target upload.

# explicity set default build target
all: px4_fmu-v5

# define a space character to be able to explicitly find it in strings
space := $(subst ,, )

# Parsing
# --------------------------------------------------------------------
# assume 1st argument passed is the main target, the
# rest are arguments to pass to the makefile generated
# by cmake in the subdirectory
FIRST_ARG := $(firstword $(MAKECMDGOALS))
ARGS := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
j ?= 4

NINJA_BIN := ninja
ifndef NO_NINJA_BUILD
	NINJA_BUILD := $(shell $(NINJA_BIN) --version 2>/dev/null)

	ifndef NINJA_BUILD
		NINJA_BIN := ninja-build
		NINJA_BUILD := $(shell $(NINJA_BIN) --version 2>/dev/null)
	endif
endif

ifdef NINJA_BUILD
	PX4_CMAKE_GENERATOR := Ninja
	PX4_MAKE := $(NINJA_BIN)

	ifdef VERBOSE
		PX4_MAKE_ARGS := -v
	else
		PX4_MAKE_ARGS :=
	endif
else
	ifdef SYSTEMROOT
		# Windows
		PX4_CMAKE_GENERATOR := "MSYS\ Makefiles"
	else
		PX4_CMAKE_GENERATOR := "Unix\ Makefiles"
	endif
	PX4_MAKE = $(MAKE)
	PX4_MAKE_ARGS = -j$(j) --no-print-directory
endif

SRC_DIR := $(shell dirname "$(realpath $(lastword $(MAKEFILE_LIST)))")

ifdef PX4_CMAKE_BUILD_TYPE
	CMAKE_ARGS += -DCMAKE_BUILD_TYPE=${PX4_CMAKE_BUILD_TYPE}
endif

# Functions
# --------------------------------------------------------------------
# describe how to build a cmake config
define cmake-build
	@$(eval BUILD_DIR = "$(SRC_DIR)/build/$(1)")
	@# check if the desired cmake configuration matches the cache then CMAKE_CACHE_CHECK stays empty
	@$(call cmake-cache-check)
	@# make sure to start from scratch when switching from GNU Make to Ninja
	@if [ $(PX4_CMAKE_GENERATOR) = "Ninja" ] && [ -e $(BUILD_DIR)/Makefile ]; then rm -rf $(BUILD_DIR); fi
	@# only excplicitly configure the first build, if cache file already exists the makefile will rerun cmake automatically if necessary
	@if [ ! -e $(BUILD_DIR)/CMakeCache.txt ] || [ $(CMAKE_CACHE_CHECK) ]; then \
		mkdir -p $(BUILD_DIR) \
		&& cd $(BUILD_DIR) \
		&& cmake "$(SRC_DIR)" -G"$(PX4_CMAKE_GENERATOR)" $(CMAKE_ARGS) \
		|| (rm -rf $(BUILD_DIR)); \
	fi
	@# run the build for the specified target
	@cmake --build $(BUILD_DIR) -- $(PX4_MAKE_ARGS) $(ARGS)
endef

# check if the options we want to build with in CMAKE_ARGS match the ones which are already configured in the cache inside BUILD_DIR
define cmake-cache-check
	@# change to build folder which fails if it doesn't exist and CACHED_CMAKE_OPTIONS stays empty
	@# fetch all previously configured and cached options from the build folder and transform them into the OPTION=VALUE format without type (e.g. :BOOL)
	@$(eval CACHED_CMAKE_OPTIONS = $(shell cd $(BUILD_DIR) 2>/dev/null && cmake -L 2>/dev/null | sed -n 's/\([^[:blank:]]*\):[^[:blank:]]*\(=[^[:blank:]]*\)/\1\2/gp' ))
	@# transform the options in CMAKE_ARGS into the OPTION=VALUE format without -D
	@$(eval DESIRED_CMAKE_OPTIONS = $(shell echo $(CMAKE_ARGS) | sed -n 's/-D\([^[:blank:]]*=[^[:blank:]]*\)/\1/gp' ))
	@# find each currently desired option in the already cached ones making sure the complete configured string value is the same
	@$(eval VERIFIED_CMAKE_OPTIONS = $(foreach option,$(DESIRED_CMAKE_OPTIONS),$(strip $(findstring $(option)$(space),$(CACHED_CMAKE_OPTIONS)))))
	@# if the complete list of desired options is found in the list of verified options we don't need to reconfigure and CMAKE_CACHE_CHECK stays empty
	@$(eval CMAKE_CACHE_CHECK = $(if $(findstring $(DESIRED_CMAKE_OPTIONS),$(VERIFIED_CMAKE_OPTIONS)),,y))
endef

COLOR_BLUE = \033[0;94m
NO_COLOR   = \033[m

define colorecho
+@echo -e '${COLOR_BLUE}${1} ${NO_COLOR}'
endef

# Get a list of all config targets boards/*/*.cmake
ALL_CONFIG_TARGETS := $(shell find boards -maxdepth 3 -mindepth 3 -name 'bootloader.cmake' -print | sed -e 's/boards\///' | sed -e 's/\.cmake//' | sed -e 's/\//_/g' | sort)

# ADD CONFIGS HERE
# --------------------------------------------------------------------
#  Do not put any spaces between function arguments.

# All targets.
$(ALL_CONFIG_TARGETS):
	@$(eval PX4_CONFIG = $@)
	@$(eval CMAKE_ARGS += -DCONFIG=$(PX4_CONFIG))
	@$(call cmake-build,$(PX4_CONFIG)$(BUILD_DIR_SUFFIX))

# Filter for only default targets to allow omiting the "_bootloader" postfix
CONFIG_TARGETS_BOOTLOADER := $(patsubst %_bootloader,%,$(filter %_bootloader,$(ALL_CONFIG_TARGETS)))
$(CONFIG_TARGETS_BOOTLOADER):
	@$(eval PX4_CONFIG = $@_bootloader)
	@$(eval CMAKE_ARGS += -DCONFIG=$(PX4_CONFIG))
	@$(call cmake-build,$(PX4_CONFIG)$(BUILD_DIR_SUFFIX))

all_config_targets: $(ALL_CONFIG_TARGETS)
all_bootloader_targets: $(CONFIG_TARGETS_BOOTLOADER)

# All targets with just dependencies but no recipe must either be marked as phony (or have the special @: as recipe).
.PHONY: all posix all_config_targets all_bootloader_targets

# Other targets
# --------------------------------------------------------------------

.PHONY: px4fmu_firmware

# px4fmu bootloader firmware
px4fmu_firmware: \
	check_airmind_mindpx-v2_bootloader \
	check_av_x-v1_bootloader \
	check_bitcraze_crazyflie_bootloader \
	check_intel_aerofc-v1_bootloader \
	check_mro_x21-v1_bootloader \
	check_nxp_fmuk66-v3_bootloader \
	check_omnibus_f4sd_bootloader \
	check_px4_fmu-v2_bootloader \
	check_px4_fmu-v3_bootloader \
	check_px4_fmu-v4_bootloader \
	check_px4_fmu-v4pro_bootloader \
	check_px4_fmu-v5_bootloader \
	check_px4_io-v2_bootloader \
	sizes

.PHONY: sizes quick_check

sizes:
	@-find build -name *.elf -type f | xargs size 2> /dev/null || :

# All default targets that don't require a special build environment
check: px4fmu_firmware check_format

# quick_check builds a single nuttx and posix target, runs testing, and checks the style
quick_check: check_px4_fmu-v5_bootloader check_format

check_%:
	@echo
	$(call colorecho,'Building' $(subst check_,,$@))
	@$(MAKE) --no-print-directory $(subst check_,,$@)
	@echo

# Astyle
# --------------------------------------------------------------------
.PHONY: check_format format

check_format:
	$(call colorecho,'Checking formatting with astyle')
	@"$(SRC_DIR)"/Tools/check_code_style_all.sh
	@cd "$(SRC_DIR)" && git diff --check

format:
	$(call colorecho,'Formatting with astyle')
	@"$(SRC_DIR)"/Tools/astyle/check_code_style_all.sh --fix

# Cleanup
# --------------------------------------------------------------------
.PHONY: clean submodulesclean submodulesupdate distclean

clean:
	@rm -rf "$(SRC_DIR)"/build

submodulesclean:
	@git submodule foreach --quiet --recursive git clean -ff -x -d
	@git submodule update --quiet --init --recursive --force || true
	@git submodule sync --recursive
	@git submodule update --init --recursive --force

submodulesupdate:
	@git submodule update --quiet --init --recursive || true
	@git submodule sync --recursive
	@git submodule update --init --recursive

distclean:
	@git submodule deinit -f .
	@git clean -ff -x -d -e ".project" -e ".cproject" -e ".idea" -e ".settings" -e ".vscode"

# Help / Error
# --------------------------------------------------------------------

# All other targets are handled by PX4_MAKE. Add a rule here to avoid printing an error.
%:
	$(if $(filter $(FIRST_ARG),$@), \
		$(error "$@ cannot be the first argument. Use '$(MAKE) help|list_config_targets' to get a list of all possible [configuration] targets."),@#)

# Print a list of non-config targets (based on http://stackoverflow.com/a/26339924/1487069)
help:
	@echo "Usage: $(MAKE) <target>"
	@echo "Where <target> is one of:"
	@$(MAKE) -pRrq -f $(lastword $(MAKEFILE_LIST)) : 2>/dev/null | \
		awk -v RS= -F: '/^# File/,/^# Finished Make data base/ {if ($$1 !~ "^[#.]") {print $$1}}' | sort | \
		egrep -v -e '^[^[:alnum:]]' -e '^($(subst $(space),|,$(ALL_CONFIG_TARGETS)))$$' -e '_bootloader$$' -e '^(posix|eagle|Makefile)'
	@echo
	@echo "Or, $(MAKE) <config_target> [<make_target(s)>]"
	@echo "Use '$(MAKE) list_config_targets' for a list of configuration targets."

# Print a list of all config targets.
list_config_targets:
	@for targ in $(patsubst %_default,%[_bootloader],$(ALL_CONFIG_TARGETS)); do echo $$targ; done

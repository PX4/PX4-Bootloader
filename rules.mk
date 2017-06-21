#
# Common rules for makefiles for the PX4 bootloaders
#

BUILD_DIR	 = build_$(TARGET_FILE_NAME)

OBJS		:= $(addprefix $(BUILD_DIR)/, $(patsubst %.c,%.o,$(SRCS)))
DEPS		:= $(OBJS:.o=.d)

ELF		 = $(BUILD_DIR)/$(TARGET_FILE_NAME).elf
BINARY		 = $(BUILD_DIR)/$(TARGET_FILE_NAME).bin

all:		$(BUILD_DIR) $(ELF) $(BINARY)

# Compile and generate dependency files
$(BUILD_DIR)/%.o:	%.c
	@echo Generating object $@
	$(CC) -c -MMD $(FLAGS) -o $@ $*.c

# Make the build directory
$(BUILD_DIR):
	mkdir $(BUILD_DIR)

$(ELF):		$(OBJS) $(MAKEFILE_LIST)
	$(CC) -o $@ $(OBJS) $(FLAGS)

$(BINARY):	$(ELF)
	$(OBJCOPY) -O binary $(ELF) $(BINARY)

# Dependencies for .o files
-include $(DEPS)

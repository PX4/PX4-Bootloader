#
# Common rules for makefiles for the PX4 bootloaders
#

OBJS		:= $(patsubst %.c,%.o,$(SRCS))
DEPS		:= $(OBJS:.o=.d)

BUILD_DIR	 = build_$(TARGET_FILE_NAME)

ELF		 = $(TARGET_FILE_NAME).elf
BINARY		 = $(TARGET_FILE_NAME).bin

all:		$(ELF) $(BINARY)

# Compile and generate dependency files
$(OBJS):	$(SRCS)
	mkdir -p $(BUILD_DIR)
	$(CC) -c $(FLAGS) $*.c -o $(BUILD_DIR)/$*.o
	$(CC) -MM $(FLAGS) $*.c > $(BUILD_DIR)/$*.d

$(ELF):		$(OBJS) $(MAKEFILE_LIST)
	$(CC) -o $(BUILD_DIR)/$@ $(addprefix $(BUILD_DIR)/, $(OBJS)) $(FLAGS)

$(BINARY):	$(ELF)
	$(OBJCOPY) -O binary $(BUILD_DIR)/$(ELF) $(BUILD_DIR)/$(BINARY)

# Dependencies for .o files
-include $(BUILD_DIR)/$(DEPS)

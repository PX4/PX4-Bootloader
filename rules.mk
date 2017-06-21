#
# Common rules for makefiles for the PX4 bootloaders
#

OBJS		:= $(patsubst %.c,%.o,$(SRCS))
DEPS		:= $(OBJS:.o=.d)

ELF		 = $(TARGET_FILE_NAME).elf
BINARY		 = $(TARGET_FILE_NAME).bin

all:		$(ELF) $(BINARY)

# Compile and generate dependency files
$(OBJS):	$(SRCS)
	$(CC) -c $(FLAGS) $*.c -o $*.o
	$(CC) -MM $(FLAGS) $*.c > $*.d

$(ELF):		$(OBJS) $(MAKEFILE_LIST)
	$(CC) -o $@ $(OBJS) $(FLAGS)

$(BINARY):	$(ELF)
	$(OBJCOPY) -O binary $(ELF) $(BINARY)


# Dependencies for .o files
-include $(DEPS)

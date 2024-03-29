# Path you your toolchain installation, leave empty if already in system PATH
TOOLCHAIN_ROOT = 

# Path to the STM32 codebase, make sure to update the submodule to get the code
# Needs to be specified with a slash (/) at the end!
# Register at https://www.st.com/en/embedded-software/stm32cubeg4.html
# and download both, version 1.5.0 as well as the patch to version 1.5.1
VENDOR_ROOT = $(HOME)/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.1/
# VENDOR_ROOT = ../STM32Cube/STM32Cube_FW_G4_V1.5.1/

# Clone from https://github.com/majbthrd/elf2dfuse, `make` to compile
ELF2DFUSE_ROOT = $(HOME)/elf2dfuse/
# ELF2DFUSE_ROOT = ../elf2dfuse/

###############################################################################

# Project specific
TARGET = main.elf
BINARY = main.bin
SRC_DIR = src/
INC_DIR = inc/

# Toolchain
CC = $(TOOLCHAIN_ROOT)arm-none-eabi-gcc
DB = $(TOOLCHAIN_ROOT)arm-none-eabi-gdb

# Project sources
SRC_FILES = $(wildcard $(SRC_DIR)*.c) $(wildcard $(SRC_DIR)*/*.c)
ASM_FILES = $(wildcard $(SRC_DIR)*.s) $(wildcard $(SRC_DIR)*/*.s)
LD_SCRIPT = STM32G431CB.ld

# Project includes
INCLUDES = -I$(INC_DIR)

# Vendor sources: Note that files in "Templates" are normally copied into project for customization,
# but that is not necessary for this simple project.
ASM_FILES += $(VENDOR_ROOT)Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc/startup_stm32g431xx.s

# Vendor includes
INCLUDES += -I$(VENDOR_ROOT)Drivers/CMSIS/Core/Include
INCLUDES += -I$(VENDOR_ROOT)Drivers/CMSIS/Device/ST/STM32G4xx/Include

# Compiler Flags
CFLAGS  = -g -O1 -Wall -Wextra -Warray-bounds -Wno-unused-parameter -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 -ffunction-sections -fdata-sections
CFLAGS += -mcpu=cortex-m4
CFLAGS += -DSTM32G431xx
CFLAGS += $(INCLUDES)

# Linker Flags
LFLAGS = -Wl,--gc-sections -Wl,-T$(LD_SCRIPT) -specs=nano.specs -specs=nosys.specs

###############################################################################

# This does an in-source build. An out-of-source build that places all object
# files into a build directory would be a better solution, but the goal was to
# keep this file very simple.

CXX_OBJS = $(SRC_FILES:.c=.o)
ASM_OBJS = $(ASM_FILES:.s=.o)
ALL_OBJS = $(ASM_OBJS) $(CXX_OBJS)

.PHONY: clean gdb-server_stlink gdb-server_openocd gdb-client

all: $(TARGET)

# Compile
$(CXX_OBJS): %.o: %.c
$(ASM_OBJS): %.o: %.s
$(ALL_OBJS):
	@echo "[CC] $@"
	@$(CC) $(CFLAGS) -c $< -o $@

# Link
%.elf: $(ALL_OBJS)
	@echo "[LD] $@"
	@$(CC) $(CFLAGS) $(LFLAGS) $(ALL_OBJS) -lm -o $@

# Clean
clean:
	@rm -f $(ALL_OBJS) $(TARGET)

# Flash using an STLink
flash-with-programmer: $(TARGET)
	arm-none-eabi-objcopy -O binary $(TARGET) $(BINARY)
	st-flash --reset write $(BINARY) 0x8000000

# Flash over USB (push DFU button while attaching power)
flash: $(TARGET)
	arm-none-eabi-objcopy -O binary $(TARGET) $(BINARY)
	$(ELF2DFUSE_ROOT)/elf2dfuse main.elf main.dfu
	dfu-util -D main.dfu -a0

# ##############################################################################################

PROJ=main

# choose C standard
DIALECT = gnu99

# define linker script
LDSCRIPT = cfg/lpc55s69-evk.ld

# choose a libc: REDLIB, NEWLIB, NANO, NANO_FLOAT
LIBC=NEWLIB

# libc semihosting?
USE_SEMIHOST=0

# Define the debug console : DBGPORT/UART
DBGCON = UART
#DBGCON = DBGPORT

# List all default C defines here
DDEFS = -DSERIAL_PORT_TYPE_UART
DDEFS +=  -DNOSDK_DEBUGCONSOLE
DDEFS += -DEMBED -DFLOAT32 -DWITHGRAPHICS -DSD_ENABLED

#ifeq ($(DBGCON),UART)
#	DDEFS += -DSDK_DEBUGCONSOLE=1 -DSERIAL_PORT_TYPE_UART=1
#else
#	DDEFS += -DSDK_DEBUGCONSOLE=0 -DSERIAL_PORT_TYPE_SWO=1
#endif

# List all default directories to look for include files here
DINCDIR = . source CMSIS startup/device lib/utilities

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS = lib/libpower_hardabi.a

# C source files here
SRC  = source/semihost_hardfault.c \
       startup/device/system_LPC55S69_cm33_core0.c \
       startup/boot_multicore_slave.c startup/startup_lpc55s69_cm33_core0.c \
       lib/utilities/fsl_assert.c lib/utilities/fsl_debug_console.c lib/utilities/fsl_str.c \
       source/calc.c source/funcs.c source/spread.c \
       source/sysdep.c source/edit.c source/scan.c \
       source/solver.c source/stack.c source/graphics.c \
       source/io.c source/sysdep_pcm.c source/dsp.c source/OA_algorithm.c\
       source/mma8652fc.c
       
ASRC =

include lib/board/lib.mk
include lib/component/lib.mk
include lib/fatfs/lib.mk
include lib/codec/lib.mk
include lib/drivers/lib.mk
include lib/sdmmc/lib.mk
include lib/lcd/lib.mk

#
# End of user defines
#############################################################################

#############################################################################
# Start of default section
#
TARGET  = arm-none-eabi-
CC      = $(TARGET)gcc
OBJCOPY = $(TARGET)objcopy
AS      = $(TARGET)gcc -x assembler-with-cpp -c
SIZE    = $(TARGET)size
OBJDUMP = $(TARGET)objdump

# choose target CPU
CPUFLAGS = -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb
DDEFS += -DCPU_LPC55S69JBD100_cm33_core0 -DARM_MATH_CM33 -DCPU_LPC55S69JBD100 -DCPU_LPC55S69JBD100_cm33
DDEFS += -D__MCUXPRESSO -D__USE_CMSIS

# Compilation mode handling: dbg/opt/release
MODE ?= dbg

ifeq (${MODE},release)
	OPT= -O2 -fomit-frame-pointer
	USE_SEMIHOST = 0
	USE_SWO = 0
else ifeq (${MODE},dbg)
	OPT= -g3 -O0 -fno-omit-frame-pointer -D__DEBUG__
	UADEFS += -D__DEBUG__
else ifeq (${MODE},opt)
	OPT= -g -O2 -fno-omit-frame-pointer -D__DEBUG__
	UADEFS += -D__DEBUG__
endif


ifeq (${USE_SEMIHOST},0)
	SRC += lib/libc_syscalls.c
else
	DDEFS += -DUSE_SEMIHOSTING
endif

ifeq (${USE_SWO},1)
    UADEFS += -DUSE_RTT
    UDEFS += -DUSE_RTT
endif
#
# End of default section
##############################################################################################

INCDIR  = $(patsubst %,-I%,$(DINCDIR))
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR))
DEFS    = $(DDEFS)
ADEFS   = $(DADEFS)
LIBS    = $(DLIBS)
OBJS    = $(SRC:%.c=%.o) $(ASRC:.s=.o)


ASFLAGS = -Wa,-mimplicit-it=always $(CPUFLAGS) $(INCDIR)
CFLAGS  = -std=$(DIALECT) $(DEBUG) $(OPT) -fno-common -Wall -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin -fmerge-constants -fmacro-prefix-map="../$(@D)/"=. $(CPUFLAGS) $(INCDIR)
LDFLAGS = -T$(LDSCRIPT) -Wl,-Map=main.map,--gc-sections,-print-memory-usage,--sort-section=alignment,--cref $(CPUFLAGS)
ifeq (${LIBC},REDLIB)
	DEFS += -D__REDLIB__ 
    LDFLAGS += --specs=redlib.specs
else ifeq (${LIBC},NANO)
	DEFS += -D__NEWLIB__ 
    LDFLAGS += --specs=nano.specs
else ifeq (${LIBC},NANO_FLOAT)
	DEFS += -D__NEWLIB__ 
    LDFLAGS += --specs=nano.specs -u _scanf_float -u _printf_float
else
	DEFS += -D__NEWLIB__ 
endif
ifeq (${USE_SEMIHOST},1)
    LDFLAGS += --specs=rdimon.specs
endif

# Generate dependency information
ASFLAGS += $(DEFS) -MD -MP -MF .dep/$(@F).d
CFLAGS += $(DEFS) -MD -MP -MF .dep/$(@F).d

#
# makefile rules
#
all: main.elf
	@echo "MODE: ${MODE}"

debug:
	@make MODE=dbg

release:
	@make MODE=release

opt:
	@make MODE=opt

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
	
%o: %s
	$(AS) $(ASFLAGS) $< -o $@

$(PROJ).elf: $(OBJS) $(LDSCRIPT) /home/loquais/ENIB/Electronique/S7/SEN/calc_starter_A2024/slave_project/main.elf.o
	$(CC) -o $@ $(filter-out %.ld, $^) $(LDFLAGS) $(LIBS)
	$(OBJDUMP) -h $@ 
	
clean:
	-rm -f $(OBJS)
	-rm -f $(PROJ)
	-rm -f *.map *.elf *.bin *.hex
	-rm -f *.xml *.dtd *.swd
	-rm -fR .dep/*

all: $(PROJ)

$(PROJ): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)  -lcr_newlib_nohost -lm -lc -lgcc
	$(OBJDUMP) -h $@

# 
# Include the dependency files, should be the last of the makefile
#
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

.PHONY: all clean


ARMGCC_PATH= /media/ProgramData/ARM/GNU/gcc-arm-none-eabi-4_7-2013q3/bin/
ARMGCCLIB_PATH = /media/ProgramData/ARM/GNU/gcc-arm-none-eabi-4_7-2013q3/arm-none-eabi/lib/armv6-m
ARMLIBGCC_PATH = /media/ProgramData/ARM/GNU/gcc-arm-none-eabi-4_7-2013q3/arm-none-eabi/lib/armv6-m
export ARMGCC_PATH 

#STlinkPath =/media/ProgramData/ARM/GNU/texane-stlink-a303d9f
#STlinkPath =/media/ProgramData/ARM/GNU/stlink
#STlinkPath =/media/ProgramData/ARM/GNU/stlink-master
STlinkPath =/media/ProgramData/testGCC/stlink-master

#
COMPILE_OPTS= -g2 -mcpu=cortex-m0 -mthumb  -O3  -fsingle-precision-constant   #-mfloat-abi=hard -mfpu=fpv4-sp-d16
COMPILE_OPTS+=-mlittle-endian 
COMPILE_OPTS+=-ffreestanding -nostdlib -nostartfiles -nodefaultlibs
# ST specific definitions..
#COMPILE_OPTS+=-DI2C_Address=2 # stm32f4xx.h uses stm32f4xx_conf.h (must be Std../inc/devices..)
COMPILE_OPTS+=-DUSE_STDPERIPH_DRIVER=1 # stm32f4xx.h uses stm32f4xx_conf.h (must be Std../inc/devices..)
COMPILE_OPTS+=-DPLL_SOURCE_HSE=1
COMPILE_OPTS+=-DSTM32F0XX_LD=1
COMPILE_OPTS+=-DHSE_VALUE=12000000
#COMPILE_OPTS+=-DARM_MATH_CM0=1
#COMPILE_OPTS+=-DSTM32F0XX=1
#COMPILE_OPTS+=-DARM_MATH_CM4=1
#COMPILE_OPTS+=-D__FPU_USED=1
#COMPILE_OPTS+=-D__FPU_PRESENT=1

export COMPILE_OPTS


OBJECTS = $(patsubst %.c,%.o,$(wildcard *.c)) 
INCLUDE_DIRS = -I ./ 

# Linker File (Device specific)
#LinkerFile = STM32F050C4_FLASH.ld
LinkerFile = STM32F050G6U6_FLASH.ld
#LinkerFile = gcc.ld
OBJECTS +=startup_stm32f0xx.o



# Include ST stdPeriph Library + CMSIS
INCLUDE_DIRS += -I Libraries/STM32F0xx_StdPeriph_Driver/inc
INCLUDE_DIRS += -I Libraries/CMSIS/Include
INCLUDE_DIRS += -I Libraries/CMSIS/Device

LIBS= -L Libraries/STM32F0xx_StdPeriph_Driver   -lSTM32F0xx_StdPeriph_Driver
 
#LIBS+= -L Libraries/CMSIS   -lCMSIS_DSP
LIBS+=-L$(ARMGCCLIB_PATH) -lm -lc    
LIBS+=-L$(ARMLIBGCC_PATH) -lgcc -lg -lnosys


## STM32F4 Discovery Utilities
#INCLUDE_DIRS += -I Libraries/Utilities/STM32F4-Discovery
#OBJECTS+=$(patsubst %.c,%.o,$(wildcard Libraries/Utilities/STM32F4-Discovery/*.c))



MAIN_OUT = main
MAIN_OUT_ELF = $(MAIN_OUT).elf
MAIN_OUT_BIN = $(MAIN_OUT).bin
MAIN_OUT_LST = $(MAIN_OUT).lst 

CC = $(ARMGCC_PATH)arm-none-eabi-gcc
CFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS) -Wl,-T,$(LinkerFile) -Wl,--gc-sections 
AS = $(ARMGCC_PATH)arm-none-eabi-gcc
ASFLAGS = -g0 -Wall -mcpu=cortex-m4 -mthumb -c 
LD = $(ARMGCC_PATH)arm-none-eabi-gcc
LDFLAGS = $(CFLAGS) #-Map=$@.map  -T$(LinkerFile) -nostartfiles
 

OBJCP = $(ARMGCC_PATH)arm-none-eabi-objcopy
OBJCPFLAGS = -O binary 
OBJDUMP = $(ARMGCC_PATH)arm-none-eabi-objdump
SIZE = $(ARMGCC_PATH)arm-none-eabi-size 

#STLINK=1
OPENOCD=1
#TELNET=1
COMMANDS = -c init -c targets -c "reset halt" -c "flash write_image erase $(MAIN_OUT_ELF)" -c "verify_image $(MAIN_OUT_ELF)" -c "reset run" -c shutdown

 

#all: $(MAIN_OUT_ELF) $(MAIN_OUT_BIN)
all: $(MAIN_OUT_BIN)
	

$(MAIN_OUT_BIN): $(MAIN_OUT_ELF)
	$(OBJCP) $(OBJCPFLAGS) $< $@
	$(OBJDUMP) -h -S -C -r $< > $(MAIN_OUT_LST)
	$(SIZE) $(MAIN_OUT_ELF)
		
$(MAIN_OUT_ELF): $(OBJECTS)
	#$(MAKE) -C Libraries/CMSIS
	$(MAKE) -C Libraries/STM32F0xx_StdPeriph_Driver
	$(CC) $(CFLAGS) -Wl,-Map=$@.map $^ -o $@   $(LIBS)
	#$(LD) $(LDFLAGS)  $^ -o $@   $(LIBS)
	
# flash

flash: $(MAIN_OUT_BIN)

ifdef STLINK
	$(STlinkPath)/st-flash write $(MAIN_OUT_BIN) 0x8000000
endif
ifdef OPENOCD
	openocd -f stm32f4discoveryRESETNONE.cfg  $(COMMANDS)
endif

ifdef TELNET 
	(echo "reset halt; flash write_image erase $(MAIN_OUT_ELF); verify_image $(MAIN_OUT_ELF); reset run; exit")|nc localhost 4444
endif


clean:
	-rm -f $(OBJECTS)
	-rm -f $(MAIN_OUT_ELF) $(MAIN_OUT_BIN)

cleanall:
	$(MAKE) -C Libraries/STM32F0xx_StdPeriph_Driver clean
	-rm -f $(OBJECTS)
	-rm -f $(MAIN_OUT_ELF) $(MAIN_OUT_BIN)


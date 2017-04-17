################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../Geocalc.c \
../Hal.c \
../LSM303_simple.c \
../twimaster.c \
../uart_handler.c \
../ui.c \
../hmc5883l.c \
../light_ws2812.c \
../Main.c \
../uart.c \
../vector.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
Geocalc.o \
Hal.o \
LSM303_simple.o \
twimaster.o \
uart_handler.o \
ui.o \
hmc5883l.o \
light_ws2812.o \
Main.o \
uart.o \
vector.o

OBJS_AS_ARGS +=  \
Geocalc.o \
Hal.o \
LSM303_simple.o \
twimaster.o \
uart_handler.o \
ui.o \
hmc5883l.o \
light_ws2812.o \
Main.o \
uart.o \
vector.o

C_DEPS +=  \
Geocalc.d \
Hal.d \
LSM303_simple.d \
twimaster.d \
uart_handler.d \
ui.d \
hmc5883l.d \
light_ws2812.d \
Main.d \
uart.d \
vector.d

C_DEPS_AS_ARGS +=  \
Geocalc.d \
Hal.d \
LSM303_simple.d \
twimaster.d \
uart_handler.d \
ui.d \
hmc5883l.d \
light_ws2812.d \
Main.d \
uart.d \
vector.d

OUTPUT_FILE_PATH +=Main.elf

OUTPUT_FILE_PATH_AS_ARGS +=Main.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

# AVR32/GNU C Compiler























./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DF_CPU=8000000  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atmega328p -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 4.8.1
	$(QUOTE)C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="Main.map" -Wl,-u,vfprintf -Wl,--start-group  -Wl,--end-group -Wl,-L"D:\Dropbox\3. Jordy en JJ\Projects JJ\BikeNav\Code\AVR Studio\BikeNav\Main\LSM303"  -Wl,--gc-sections -mmcu=atmega328p -lprintf_flt  
	@echo Finished building target: $@
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "Main.elf" "Main.hex"
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "Main.elf" "Main.eep" || exit 0
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "Main.elf" > "Main.lss"
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "Main.elf" "Main.srec"
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-size.exe" "Main.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "Main.elf" "Main.a" "Main.hex" "Main.lss" "Main.eep" "Main.map" "Main.srec" "Main.usersignatures"
	
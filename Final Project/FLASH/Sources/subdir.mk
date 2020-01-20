################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/BoardSupport.c" \
"../Sources/I2C.c" \
"../Sources/LCD.c" \
"../Sources/UART.c" \
"../Sources/arm_cm0.c" \
"../Sources/main.c" \
"../Sources/mcg.c" \
"../Sources/sa_mtb.c" \

C_SRCS += \
../Sources/BoardSupport.c \
../Sources/I2C.c \
../Sources/LCD.c \
../Sources/UART.c \
../Sources/arm_cm0.c \
../Sources/main.c \
../Sources/mcg.c \
../Sources/sa_mtb.c \

OBJS += \
./Sources/BoardSupport.o \
./Sources/I2C.o \
./Sources/LCD.o \
./Sources/UART.o \
./Sources/arm_cm0.o \
./Sources/main.o \
./Sources/mcg.o \
./Sources/sa_mtb.o \

C_DEPS += \
./Sources/BoardSupport.d \
./Sources/I2C.d \
./Sources/LCD.d \
./Sources/UART.d \
./Sources/arm_cm0.d \
./Sources/main.d \
./Sources/mcg.d \
./Sources/sa_mtb.d \

OBJS_QUOTED += \
"./Sources/BoardSupport.o" \
"./Sources/I2C.o" \
"./Sources/LCD.o" \
"./Sources/UART.o" \
"./Sources/arm_cm0.o" \
"./Sources/main.o" \
"./Sources/mcg.o" \
"./Sources/sa_mtb.o" \

C_DEPS_QUOTED += \
"./Sources/BoardSupport.d" \
"./Sources/I2C.d" \
"./Sources/LCD.d" \
"./Sources/UART.d" \
"./Sources/arm_cm0.d" \
"./Sources/main.d" \
"./Sources/mcg.d" \
"./Sources/sa_mtb.d" \

OBJS_OS_FORMAT += \
./Sources/BoardSupport.o \
./Sources/I2C.o \
./Sources/LCD.o \
./Sources/UART.o \
./Sources/arm_cm0.o \
./Sources/main.o \
./Sources/mcg.o \
./Sources/sa_mtb.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/BoardSupport.o: ../Sources/BoardSupport.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/BoardSupport.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/BoardSupport.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/I2C.o: ../Sources/I2C.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/I2C.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/I2C.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/LCD.o: ../Sources/LCD.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/LCD.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/LCD.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/UART.o: ../Sources/UART.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/UART.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/UART.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/arm_cm0.o: ../Sources/arm_cm0.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/arm_cm0.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/arm_cm0.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/main.o: ../Sources/main.c
	@echo 'Building file: $<'
	@echo 'Executing target #6 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/main.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/main.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/mcg.o: ../Sources/mcg.c
	@echo 'Building file: $<'
	@echo 'Executing target #7 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/mcg.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/mcg.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/sa_mtb.o: ../Sources/sa_mtb.c
	@echo 'Building file: $<'
	@echo 'Executing target #8 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/sa_mtb.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/sa_mtb.o"
	@echo 'Finished building: $<'
	@echo ' '



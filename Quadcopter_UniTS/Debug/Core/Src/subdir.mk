################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/attitude.c \
../Core/Src/control.c \
../Core/Src/filter.c \
../Core/Src/imu.c \
../Core/Src/main.c \
../Core/Src/math_extension.c \
../Core/Src/motor.c \
../Core/Src/orientation.c \
../Core/Src/quaternion.c \
../Core/Src/radio.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c 

OBJS += \
./Core/Src/attitude.o \
./Core/Src/control.o \
./Core/Src/filter.o \
./Core/Src/imu.o \
./Core/Src/main.o \
./Core/Src/math_extension.o \
./Core/Src/motor.o \
./Core/Src/orientation.o \
./Core/Src/quaternion.o \
./Core/Src/radio.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o 

C_DEPS += \
./Core/Src/attitude.d \
./Core/Src/control.d \
./Core/Src/filter.d \
./Core/Src/imu.d \
./Core/Src/main.d \
./Core/Src/math_extension.d \
./Core/Src/motor.d \
./Core/Src/orientation.d \
./Core/Src/quaternion.d \
./Core/Src/radio.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H7A3xxQ -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/attitude.cyclo ./Core/Src/attitude.d ./Core/Src/attitude.o ./Core/Src/attitude.su ./Core/Src/control.cyclo ./Core/Src/control.d ./Core/Src/control.o ./Core/Src/control.su ./Core/Src/filter.cyclo ./Core/Src/filter.d ./Core/Src/filter.o ./Core/Src/filter.su ./Core/Src/imu.cyclo ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/math_extension.cyclo ./Core/Src/math_extension.d ./Core/Src/math_extension.o ./Core/Src/math_extension.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/orientation.cyclo ./Core/Src/orientation.d ./Core/Src/orientation.o ./Core/Src/orientation.su ./Core/Src/quaternion.cyclo ./Core/Src/quaternion.d ./Core/Src/quaternion.o ./Core/Src/quaternion.su ./Core/Src/radio.cyclo ./Core/Src/radio.d ./Core/Src/radio.o ./Core/Src/radio.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su

.PHONY: clean-Core-2f-Src


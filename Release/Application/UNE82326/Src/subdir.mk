################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/UNE82326/Src/serial_une82326.c \
../Application/UNE82326/Src/une82326.c \
../Application/UNE82326/Src/une82326_device_table.c \
../Application/UNE82326/Src/une82326_protocol.c 

O_SRCS += \
../Application/UNE82326/Src/serial_une82326.o \
../Application/UNE82326/Src/une82326.o \
../Application/UNE82326/Src/une82326_device_table.o \
../Application/UNE82326/Src/une82326_protocol.o 

OBJS += \
./Application/UNE82326/Src/serial_une82326.o \
./Application/UNE82326/Src/une82326.o \
./Application/UNE82326/Src/une82326_device_table.o \
./Application/UNE82326/Src/une82326_protocol.o 

C_DEPS += \
./Application/UNE82326/Src/serial_une82326.d \
./Application/UNE82326/Src/une82326.d \
./Application/UNE82326/Src/une82326_device_table.d \
./Application/UNE82326/Src/une82326_protocol.d 


# Each subdirectory must supply rules for building sources it contributes
Application/UNE82326/Src/%.o Application/UNE82326/Src/%.su Application/UNE82326/Src/%.cyclo: ../Application/UNE82326/Src/%.c Application/UNE82326/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-UNE82326-2f-Src

clean-Application-2f-UNE82326-2f-Src:
	-$(RM) ./Application/UNE82326/Src/serial_une82326.cyclo ./Application/UNE82326/Src/serial_une82326.d ./Application/UNE82326/Src/serial_une82326.o ./Application/UNE82326/Src/serial_une82326.su ./Application/UNE82326/Src/une82326.cyclo ./Application/UNE82326/Src/une82326.d ./Application/UNE82326/Src/une82326.o ./Application/UNE82326/Src/une82326.su ./Application/UNE82326/Src/une82326_device_table.cyclo ./Application/UNE82326/Src/une82326_device_table.d ./Application/UNE82326/Src/une82326_device_table.o ./Application/UNE82326/Src/une82326_device_table.su ./Application/UNE82326/Src/une82326_protocol.cyclo ./Application/UNE82326/Src/une82326_protocol.d ./Application/UNE82326/Src/une82326_protocol.o ./Application/UNE82326/Src/une82326_protocol.su

.PHONY: clean-Application-2f-UNE82326-2f-Src


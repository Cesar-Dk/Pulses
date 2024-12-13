################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/LocalTool/Src/aqualabo_modbus.c \
../Application/LocalTool/Src/generic_modbus.c \
../Application/LocalTool/Src/generic_modbus_table.c \
../Application/LocalTool/Src/mbcrc.c \
../Application/LocalTool/Src/modbus.c \
../Application/LocalTool/Src/modbus_sensors.c \
../Application/LocalTool/Src/modbus_sensors_log.c 

O_SRCS += \
../Application/LocalTool/Src/aqualabo_modbus.o \
../Application/LocalTool/Src/generic_modbus.o \
../Application/LocalTool/Src/generic_modbus_table.o \
../Application/LocalTool/Src/mbcrc.o \
../Application/LocalTool/Src/modbus.o \
../Application/LocalTool/Src/modbus_sensors.o \
../Application/LocalTool/Src/modbus_sensors_log.o 

OBJS += \
./Application/LocalTool/Src/aqualabo_modbus.o \
./Application/LocalTool/Src/generic_modbus.o \
./Application/LocalTool/Src/generic_modbus_table.o \
./Application/LocalTool/Src/mbcrc.o \
./Application/LocalTool/Src/modbus.o \
./Application/LocalTool/Src/modbus_sensors.o \
./Application/LocalTool/Src/modbus_sensors_log.o 

C_DEPS += \
./Application/LocalTool/Src/aqualabo_modbus.d \
./Application/LocalTool/Src/generic_modbus.d \
./Application/LocalTool/Src/generic_modbus_table.d \
./Application/LocalTool/Src/mbcrc.d \
./Application/LocalTool/Src/modbus.d \
./Application/LocalTool/Src/modbus_sensors.d \
./Application/LocalTool/Src/modbus_sensors_log.d 


# Each subdirectory must supply rules for building sources it contributes
Application/LocalTool/Src/%.o Application/LocalTool/Src/%.su Application/LocalTool/Src/%.cyclo: ../Application/LocalTool/Src/%.c Application/LocalTool/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-LocalTool-2f-Src

clean-Application-2f-LocalTool-2f-Src:
	-$(RM) ./Application/LocalTool/Src/aqualabo_modbus.cyclo ./Application/LocalTool/Src/aqualabo_modbus.d ./Application/LocalTool/Src/aqualabo_modbus.o ./Application/LocalTool/Src/aqualabo_modbus.su ./Application/LocalTool/Src/generic_modbus.cyclo ./Application/LocalTool/Src/generic_modbus.d ./Application/LocalTool/Src/generic_modbus.o ./Application/LocalTool/Src/generic_modbus.su ./Application/LocalTool/Src/generic_modbus_table.cyclo ./Application/LocalTool/Src/generic_modbus_table.d ./Application/LocalTool/Src/generic_modbus_table.o ./Application/LocalTool/Src/generic_modbus_table.su ./Application/LocalTool/Src/mbcrc.cyclo ./Application/LocalTool/Src/mbcrc.d ./Application/LocalTool/Src/mbcrc.o ./Application/LocalTool/Src/mbcrc.su ./Application/LocalTool/Src/modbus.cyclo ./Application/LocalTool/Src/modbus.d ./Application/LocalTool/Src/modbus.o ./Application/LocalTool/Src/modbus.su ./Application/LocalTool/Src/modbus_sensors.cyclo ./Application/LocalTool/Src/modbus_sensors.d ./Application/LocalTool/Src/modbus_sensors.o ./Application/LocalTool/Src/modbus_sensors.su ./Application/LocalTool/Src/modbus_sensors_log.cyclo ./Application/LocalTool/Src/modbus_sensors_log.d ./Application/LocalTool/Src/modbus_sensors_log.o ./Application/LocalTool/Src/modbus_sensors_log.su

.PHONY: clean-Application-2f-LocalTool-2f-Src


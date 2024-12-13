################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/MBUS/Src/mbus.c \
../Application/MBUS/Src/mbus_protocol.c \
../Application/MBUS/Src/mbus_protocol_aux.c \
../Application/MBUS/Src/mbus_slave_table_manager.c \
../Application/MBUS/Src/serial_mbus.c 

O_SRCS += \
../Application/MBUS/Src/mbus.o \
../Application/MBUS/Src/mbus_protocol.o \
../Application/MBUS/Src/mbus_protocol_aux.o \
../Application/MBUS/Src/mbus_slave_table_manager.o \
../Application/MBUS/Src/serial_mbus.o 

OBJS += \
./Application/MBUS/Src/mbus.o \
./Application/MBUS/Src/mbus_protocol.o \
./Application/MBUS/Src/mbus_protocol_aux.o \
./Application/MBUS/Src/mbus_slave_table_manager.o \
./Application/MBUS/Src/serial_mbus.o 

C_DEPS += \
./Application/MBUS/Src/mbus.d \
./Application/MBUS/Src/mbus_protocol.d \
./Application/MBUS/Src/mbus_protocol_aux.d \
./Application/MBUS/Src/mbus_slave_table_manager.d \
./Application/MBUS/Src/serial_mbus.d 


# Each subdirectory must supply rules for building sources it contributes
Application/MBUS/Src/%.o Application/MBUS/Src/%.su Application/MBUS/Src/%.cyclo: ../Application/MBUS/Src/%.c Application/MBUS/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-MBUS-2f-Src

clean-Application-2f-MBUS-2f-Src:
	-$(RM) ./Application/MBUS/Src/mbus.cyclo ./Application/MBUS/Src/mbus.d ./Application/MBUS/Src/mbus.o ./Application/MBUS/Src/mbus.su ./Application/MBUS/Src/mbus_protocol.cyclo ./Application/MBUS/Src/mbus_protocol.d ./Application/MBUS/Src/mbus_protocol.o ./Application/MBUS/Src/mbus_protocol.su ./Application/MBUS/Src/mbus_protocol_aux.cyclo ./Application/MBUS/Src/mbus_protocol_aux.d ./Application/MBUS/Src/mbus_protocol_aux.o ./Application/MBUS/Src/mbus_protocol_aux.su ./Application/MBUS/Src/mbus_slave_table_manager.cyclo ./Application/MBUS/Src/mbus_slave_table_manager.d ./Application/MBUS/Src/mbus_slave_table_manager.o ./Application/MBUS/Src/mbus_slave_table_manager.su ./Application/MBUS/Src/serial_mbus.cyclo ./Application/MBUS/Src/serial_mbus.d ./Application/MBUS/Src/serial_mbus.o ./Application/MBUS/Src/serial_mbus.su

.PHONY: clean-Application-2f-MBUS-2f-Src


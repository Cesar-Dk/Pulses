################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/MQTT/Src/mqtt_buffer.c \
../Application/MQTT/Src/mqtt_frames.c \
../Application/MQTT/Src/mqtt_task.c \
../Application/MQTT/Src/mqtt_timer.c 

O_SRCS += \
../Application/MQTT/Src/mqtt_buffer.o \
../Application/MQTT/Src/mqtt_frames.o \
../Application/MQTT/Src/mqtt_task.o \
../Application/MQTT/Src/mqtt_timer.o 

OBJS += \
./Application/MQTT/Src/mqtt_buffer.o \
./Application/MQTT/Src/mqtt_frames.o \
./Application/MQTT/Src/mqtt_task.o \
./Application/MQTT/Src/mqtt_timer.o 

C_DEPS += \
./Application/MQTT/Src/mqtt_buffer.d \
./Application/MQTT/Src/mqtt_frames.d \
./Application/MQTT/Src/mqtt_task.d \
./Application/MQTT/Src/mqtt_timer.d 


# Each subdirectory must supply rules for building sources it contributes
Application/MQTT/Src/%.o Application/MQTT/Src/%.su Application/MQTT/Src/%.cyclo: ../Application/MQTT/Src/%.c Application/MQTT/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-MQTT-2f-Src

clean-Application-2f-MQTT-2f-Src:
	-$(RM) ./Application/MQTT/Src/mqtt_buffer.cyclo ./Application/MQTT/Src/mqtt_buffer.d ./Application/MQTT/Src/mqtt_buffer.o ./Application/MQTT/Src/mqtt_buffer.su ./Application/MQTT/Src/mqtt_frames.cyclo ./Application/MQTT/Src/mqtt_frames.d ./Application/MQTT/Src/mqtt_frames.o ./Application/MQTT/Src/mqtt_frames.su ./Application/MQTT/Src/mqtt_task.cyclo ./Application/MQTT/Src/mqtt_task.d ./Application/MQTT/Src/mqtt_task.o ./Application/MQTT/Src/mqtt_task.su ./Application/MQTT/Src/mqtt_timer.cyclo ./Application/MQTT/Src/mqtt_timer.d ./Application/MQTT/Src/mqtt_timer.o ./Application/MQTT/Src/mqtt_timer.su

.PHONY: clean-Application-2f-MQTT-2f-Src


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/Sensor/Src/ad.c \
../Application/Sensor/Src/generic_sensor.c \
../Application/Sensor/Src/generic_sensor_log.c \
../Application/Sensor/Src/i2c_sensor.c \
../Application/Sensor/Src/sensor_log.c \
../Application/Sensor/Src/ventosa.c 

O_SRCS += \
../Application/Sensor/Src/ad.o \
../Application/Sensor/Src/generic_sensor.o \
../Application/Sensor/Src/generic_sensor_log.o \
../Application/Sensor/Src/i2c_sensor.o \
../Application/Sensor/Src/sensor_log.o \
../Application/Sensor/Src/ventosa.o 

OBJS += \
./Application/Sensor/Src/ad.o \
./Application/Sensor/Src/generic_sensor.o \
./Application/Sensor/Src/generic_sensor_log.o \
./Application/Sensor/Src/i2c_sensor.o \
./Application/Sensor/Src/sensor_log.o \
./Application/Sensor/Src/ventosa.o 

C_DEPS += \
./Application/Sensor/Src/ad.d \
./Application/Sensor/Src/generic_sensor.d \
./Application/Sensor/Src/generic_sensor_log.d \
./Application/Sensor/Src/i2c_sensor.d \
./Application/Sensor/Src/sensor_log.d \
./Application/Sensor/Src/ventosa.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Sensor/Src/%.o Application/Sensor/Src/%.su Application/Sensor/Src/%.cyclo: ../Application/Sensor/Src/%.c Application/Sensor/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-Sensor-2f-Src

clean-Application-2f-Sensor-2f-Src:
	-$(RM) ./Application/Sensor/Src/ad.cyclo ./Application/Sensor/Src/ad.d ./Application/Sensor/Src/ad.o ./Application/Sensor/Src/ad.su ./Application/Sensor/Src/generic_sensor.cyclo ./Application/Sensor/Src/generic_sensor.d ./Application/Sensor/Src/generic_sensor.o ./Application/Sensor/Src/generic_sensor.su ./Application/Sensor/Src/generic_sensor_log.cyclo ./Application/Sensor/Src/generic_sensor_log.d ./Application/Sensor/Src/generic_sensor_log.o ./Application/Sensor/Src/generic_sensor_log.su ./Application/Sensor/Src/i2c_sensor.cyclo ./Application/Sensor/Src/i2c_sensor.d ./Application/Sensor/Src/i2c_sensor.o ./Application/Sensor/Src/i2c_sensor.su ./Application/Sensor/Src/sensor_log.cyclo ./Application/Sensor/Src/sensor_log.d ./Application/Sensor/Src/sensor_log.o ./Application/Sensor/Src/sensor_log.su ./Application/Sensor/Src/ventosa.cyclo ./Application/Sensor/Src/ventosa.d ./Application/Sensor/Src/ventosa.o ./Application/Sensor/Src/ventosa.su

.PHONY: clean-Application-2f-Sensor-2f-Src


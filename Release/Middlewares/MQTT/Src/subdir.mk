################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/MQTT/Src/MQTTConnectClient.c \
../Middlewares/MQTT/Src/MQTTDeserializePublish.c \
../Middlewares/MQTT/Src/MQTTPacket.c \
../Middlewares/MQTT/Src/MQTTSerializePublish.c \
../Middlewares/MQTT/Src/MQTTSubscribeClient.c \
../Middlewares/MQTT/Src/MQTTUnsubscribeClient.c 

O_SRCS += \
../Middlewares/MQTT/Src/MQTTConnectClient.o \
../Middlewares/MQTT/Src/MQTTDeserializePublish.o \
../Middlewares/MQTT/Src/MQTTPacket.o \
../Middlewares/MQTT/Src/MQTTSerializePublish.o \
../Middlewares/MQTT/Src/MQTTSubscribeClient.o \
../Middlewares/MQTT/Src/MQTTUnsubscribeClient.o 

OBJS += \
./Middlewares/MQTT/Src/MQTTConnectClient.o \
./Middlewares/MQTT/Src/MQTTDeserializePublish.o \
./Middlewares/MQTT/Src/MQTTPacket.o \
./Middlewares/MQTT/Src/MQTTSerializePublish.o \
./Middlewares/MQTT/Src/MQTTSubscribeClient.o \
./Middlewares/MQTT/Src/MQTTUnsubscribeClient.o 

C_DEPS += \
./Middlewares/MQTT/Src/MQTTConnectClient.d \
./Middlewares/MQTT/Src/MQTTDeserializePublish.d \
./Middlewares/MQTT/Src/MQTTPacket.d \
./Middlewares/MQTT/Src/MQTTSerializePublish.d \
./Middlewares/MQTT/Src/MQTTSubscribeClient.d \
./Middlewares/MQTT/Src/MQTTUnsubscribeClient.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/MQTT/Src/%.o Middlewares/MQTT/Src/%.su Middlewares/MQTT/Src/%.cyclo: ../Middlewares/MQTT/Src/%.c Middlewares/MQTT/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-MQTT-2f-Src

clean-Middlewares-2f-MQTT-2f-Src:
	-$(RM) ./Middlewares/MQTT/Src/MQTTConnectClient.cyclo ./Middlewares/MQTT/Src/MQTTConnectClient.d ./Middlewares/MQTT/Src/MQTTConnectClient.o ./Middlewares/MQTT/Src/MQTTConnectClient.su ./Middlewares/MQTT/Src/MQTTDeserializePublish.cyclo ./Middlewares/MQTT/Src/MQTTDeserializePublish.d ./Middlewares/MQTT/Src/MQTTDeserializePublish.o ./Middlewares/MQTT/Src/MQTTDeserializePublish.su ./Middlewares/MQTT/Src/MQTTPacket.cyclo ./Middlewares/MQTT/Src/MQTTPacket.d ./Middlewares/MQTT/Src/MQTTPacket.o ./Middlewares/MQTT/Src/MQTTPacket.su ./Middlewares/MQTT/Src/MQTTSerializePublish.cyclo ./Middlewares/MQTT/Src/MQTTSerializePublish.d ./Middlewares/MQTT/Src/MQTTSerializePublish.o ./Middlewares/MQTT/Src/MQTTSerializePublish.su ./Middlewares/MQTT/Src/MQTTSubscribeClient.cyclo ./Middlewares/MQTT/Src/MQTTSubscribeClient.d ./Middlewares/MQTT/Src/MQTTSubscribeClient.o ./Middlewares/MQTT/Src/MQTTSubscribeClient.su ./Middlewares/MQTT/Src/MQTTUnsubscribeClient.cyclo ./Middlewares/MQTT/Src/MQTTUnsubscribeClient.d ./Middlewares/MQTT/Src/MQTTUnsubscribeClient.o ./Middlewares/MQTT/Src/MQTTUnsubscribeClient.su

.PHONY: clean-Middlewares-2f-MQTT-2f-Src


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/json/Src/jsmn.c \
../Application/json/Src/json.c 

O_SRCS += \
../Application/json/Src/jsmn.o \
../Application/json/Src/json.o 

OBJS += \
./Application/json/Src/jsmn.o \
./Application/json/Src/json.o 

C_DEPS += \
./Application/json/Src/jsmn.d \
./Application/json/Src/json.d 


# Each subdirectory must supply rules for building sources it contributes
Application/json/Src/%.o Application/json/Src/%.su Application/json/Src/%.cyclo: ../Application/json/Src/%.c Application/json/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-json-2f-Src

clean-Application-2f-json-2f-Src:
	-$(RM) ./Application/json/Src/jsmn.cyclo ./Application/json/Src/jsmn.d ./Application/json/Src/jsmn.o ./Application/json/Src/jsmn.su ./Application/json/Src/json.cyclo ./Application/json/Src/json.d ./Application/json/Src/json.o ./Application/json/Src/json.su

.PHONY: clean-Application-2f-json-2f-Src


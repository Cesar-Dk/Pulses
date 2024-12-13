################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/dlms_log/Src/dlms_log.c 

O_SRCS += \
../Application/dlms_log/Src/dlms_log.o 

OBJS += \
./Application/dlms_log/Src/dlms_log.o 

C_DEPS += \
./Application/dlms_log/Src/dlms_log.d 


# Each subdirectory must supply rules for building sources it contributes
Application/dlms_log/Src/%.o Application/dlms_log/Src/%.su Application/dlms_log/Src/%.cyclo: ../Application/dlms_log/Src/%.c Application/dlms_log/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-dlms_log-2f-Src

clean-Application-2f-dlms_log-2f-Src:
	-$(RM) ./Application/dlms_log/Src/dlms_log.cyclo ./Application/dlms_log/Src/dlms_log.d ./Application/dlms_log/Src/dlms_log.o ./Application/dlms_log/Src/dlms_log.su

.PHONY: clean-Application-2f-dlms_log-2f-Src


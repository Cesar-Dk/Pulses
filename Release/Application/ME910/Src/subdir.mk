################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/ME910/Src/ME910.c \
../Application/ME910/Src/comm_serial.c \
../Application/ME910/Src/comm_udp.c \
../Application/ME910/Src/udp_protocol.c 

O_SRCS += \
../Application/ME910/Src/ME910.o \
../Application/ME910/Src/comm_serial.o \
../Application/ME910/Src/comm_udp.o \
../Application/ME910/Src/udp_protocol.o 

OBJS += \
./Application/ME910/Src/ME910.o \
./Application/ME910/Src/comm_serial.o \
./Application/ME910/Src/comm_udp.o \
./Application/ME910/Src/udp_protocol.o 

C_DEPS += \
./Application/ME910/Src/ME910.d \
./Application/ME910/Src/comm_serial.d \
./Application/ME910/Src/comm_udp.d \
./Application/ME910/Src/udp_protocol.d 


# Each subdirectory must supply rules for building sources it contributes
Application/ME910/Src/%.o Application/ME910/Src/%.su Application/ME910/Src/%.cyclo: ../Application/ME910/Src/%.c Application/ME910/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-ME910-2f-Src

clean-Application-2f-ME910-2f-Src:
	-$(RM) ./Application/ME910/Src/ME910.cyclo ./Application/ME910/Src/ME910.d ./Application/ME910/Src/ME910.o ./Application/ME910/Src/ME910.su ./Application/ME910/Src/comm_serial.cyclo ./Application/ME910/Src/comm_serial.d ./Application/ME910/Src/comm_serial.o ./Application/ME910/Src/comm_serial.su ./Application/ME910/Src/comm_udp.cyclo ./Application/ME910/Src/comm_udp.d ./Application/ME910/Src/comm_udp.o ./Application/ME910/Src/comm_udp.su ./Application/ME910/Src/udp_protocol.cyclo ./Application/ME910/Src/udp_protocol.d ./Application/ME910/Src/udp_protocol.o ./Application/ME910/Src/udp_protocol.su

.PHONY: clean-Application-2f-ME910-2f-Src


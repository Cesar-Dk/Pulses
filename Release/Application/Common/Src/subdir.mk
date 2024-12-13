################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/Common/Src/battery.c \
../Application/Common/Src/circular_buffer.c \
../Application/Common/Src/common_lib.c \
../Application/Common/Src/crc32.c \
../Application/Common/Src/flash_l4.c \
../Application/Common/Src/fw_update.c \
../Application/Common/Src/leds.c \
../Application/Common/Src/params.c \
../Application/Common/Src/rtc_system.c \
../Application/Common/Src/serial_modbus.c \
../Application/Common/Src/shutdown.c \
../Application/Common/Src/spi_flash.c \
../Application/Common/Src/test_prod.c \
../Application/Common/Src/tick.c 

O_SRCS += \
../Application/Common/Src/battery.o \
../Application/Common/Src/circular_buffer.o \
../Application/Common/Src/common_lib.o \
../Application/Common/Src/crc32.o \
../Application/Common/Src/flash_l4.o \
../Application/Common/Src/fw_update.o \
../Application/Common/Src/leds.o \
../Application/Common/Src/params.o \
../Application/Common/Src/rtc_system.o \
../Application/Common/Src/serial_modbus.o \
../Application/Common/Src/shutdown.o \
../Application/Common/Src/spi_flash.o \
../Application/Common/Src/test_prod.o \
../Application/Common/Src/tick.o 

OBJS += \
./Application/Common/Src/battery.o \
./Application/Common/Src/circular_buffer.o \
./Application/Common/Src/common_lib.o \
./Application/Common/Src/crc32.o \
./Application/Common/Src/flash_l4.o \
./Application/Common/Src/fw_update.o \
./Application/Common/Src/leds.o \
./Application/Common/Src/params.o \
./Application/Common/Src/rtc_system.o \
./Application/Common/Src/serial_modbus.o \
./Application/Common/Src/shutdown.o \
./Application/Common/Src/spi_flash.o \
./Application/Common/Src/test_prod.o \
./Application/Common/Src/tick.o 

C_DEPS += \
./Application/Common/Src/battery.d \
./Application/Common/Src/circular_buffer.d \
./Application/Common/Src/common_lib.d \
./Application/Common/Src/crc32.d \
./Application/Common/Src/flash_l4.d \
./Application/Common/Src/fw_update.d \
./Application/Common/Src/leds.d \
./Application/Common/Src/params.d \
./Application/Common/Src/rtc_system.d \
./Application/Common/Src/serial_modbus.d \
./Application/Common/Src/shutdown.d \
./Application/Common/Src/spi_flash.d \
./Application/Common/Src/test_prod.d \
./Application/Common/Src/tick.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Common/Src/%.o Application/Common/Src/%.su Application/Common/Src/%.cyclo: ../Application/Common/Src/%.c Application/Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-Common-2f-Src

clean-Application-2f-Common-2f-Src:
	-$(RM) ./Application/Common/Src/battery.cyclo ./Application/Common/Src/battery.d ./Application/Common/Src/battery.o ./Application/Common/Src/battery.su ./Application/Common/Src/circular_buffer.cyclo ./Application/Common/Src/circular_buffer.d ./Application/Common/Src/circular_buffer.o ./Application/Common/Src/circular_buffer.su ./Application/Common/Src/common_lib.cyclo ./Application/Common/Src/common_lib.d ./Application/Common/Src/common_lib.o ./Application/Common/Src/common_lib.su ./Application/Common/Src/crc32.cyclo ./Application/Common/Src/crc32.d ./Application/Common/Src/crc32.o ./Application/Common/Src/crc32.su ./Application/Common/Src/flash_l4.cyclo ./Application/Common/Src/flash_l4.d ./Application/Common/Src/flash_l4.o ./Application/Common/Src/flash_l4.su ./Application/Common/Src/fw_update.cyclo ./Application/Common/Src/fw_update.d ./Application/Common/Src/fw_update.o ./Application/Common/Src/fw_update.su ./Application/Common/Src/leds.cyclo ./Application/Common/Src/leds.d ./Application/Common/Src/leds.o ./Application/Common/Src/leds.su ./Application/Common/Src/params.cyclo ./Application/Common/Src/params.d ./Application/Common/Src/params.o ./Application/Common/Src/params.su ./Application/Common/Src/rtc_system.cyclo ./Application/Common/Src/rtc_system.d ./Application/Common/Src/rtc_system.o ./Application/Common/Src/rtc_system.su ./Application/Common/Src/serial_modbus.cyclo ./Application/Common/Src/serial_modbus.d ./Application/Common/Src/serial_modbus.o ./Application/Common/Src/serial_modbus.su ./Application/Common/Src/shutdown.cyclo ./Application/Common/Src/shutdown.d ./Application/Common/Src/shutdown.o ./Application/Common/Src/shutdown.su ./Application/Common/Src/spi_flash.cyclo ./Application/Common/Src/spi_flash.d ./Application/Common/Src/spi_flash.o ./Application/Common/Src/spi_flash.su ./Application/Common/Src/test_prod.cyclo ./Application/Common/Src/test_prod.d ./Application/Common/Src/test_prod.o ./Application/Common/Src/test_prod.su ./Application/Common/Src/tick.cyclo ./Application/Common/Src/tick.d ./Application/Common/Src/tick.o ./Application/Common/Src/tick.su

.PHONY: clean-Application-2f-Common-2f-Src


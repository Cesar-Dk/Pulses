################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/ST/STM32_UART_EMUL_Library/Src/subdir.mk
-include Middlewares/MQTT/Src/subdir.mk
-include Drivers/STM32U5xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Application/json/Src/subdir.mk
-include Application/dlms_log/Src/subdir.mk
-include Application/dlms_client/Src/subdir.mk
-include Application/dlms/Src/subdir.mk
-include Application/dlms/Inc/subdir.mk
-include Application/UNE82326/Src/subdir.mk
-include Application/Sensor/Src/subdir.mk
-include Application/Pulses/Src/subdir.mk
-include Application/MQTT/Src/subdir.mk
-include Application/ME910/Src/subdir.mk
-include Application/MBUS/Src/subdir.mk
-include Application/LocalTool/Src/subdir.mk
-include Application/Datalogger/Src/subdir.mk
-include Application/Common/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := PIPE20_21_600028
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
PIPE20_21_600028.elf \

MAP_FILES += \
PIPE20_21_600028.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
PIPE20_21_600028.list \

OBJCOPY_HEX += \
PIPE20_21_600028.hex \

OBJCOPY_BIN += \
PIPE20_21_600028.bin \


# All Target
all: main-build

# Main-build Target
main-build: PIPE20_21_600028.elf secondary-outputs

# Tool invocations
PIPE20_21_600028.elf PIPE20_21_600028.map: $(OBJS) $(USER_OBJS) ./STM32U575CIUX_FLASH_custom.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "PIPE20_21_600028.elf" $(OBJS) $(USER_OBJS) $(LIBS) -mcpu=cortex-m33 -T"./STM32U575CIUX_FLASH_custom.ld" --specs=nosys.specs -Wl,-Map="PIPE20_21_600028.map" -Wl,--cref -Wl,--gc-sections -Wl,--verbose -static -Wl,--print-memory-usage --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group

	@echo 'Finished building target: $@'
	@echo ' '


default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

PIPE20_21_600028.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "PIPE20_21_600028.list"
	@echo 'Finished building: $@'
	@echo ' '

PIPE20_21_600028.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "PIPE20_21_600028.hex"
	@echo 'Finished building: $@'
	@echo ' '

PIPE20_21_600028.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "PIPE20_21_600028.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) PIPE20_21_600028.bin PIPE20_21_600028.elf PIPE20_21_600028.hex PIPE20_21_600028.list PIPE20_21_600028.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets

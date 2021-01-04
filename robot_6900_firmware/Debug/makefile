################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/spi_handler/subdir.mk
-include Core/Src/lidar/subdir.mk
-include Core/Src/command_parser/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
robot_6900_firmware.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
robot_6900_firmware.list \

OBJCOPY_BIN += \
robot_6900_firmware.bin \


# All Target
all: robot_6900_firmware.elf secondary-outputs

# Tool invocations
robot_6900_firmware.elf: $(OBJS) $(USER_OBJS) D:\Google\ Drive\5ETI_ESE_PRJ2021\CubeIDE\ Workspace\Robot_6900_firmware\robot_6900_firmware\STM32F303VCTX_FLASH.ld
	arm-none-eabi-gcc -o "robot_6900_firmware.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Google Drive\5ETI_ESE_PRJ2021\CubeIDE Workspace\Robot_6900_firmware\robot_6900_firmware\STM32F303VCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="robot_6900_firmware.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

robot_6900_firmware.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "robot_6900_firmware.list"
	@echo 'Finished building: $@'
	@echo ' '

robot_6900_firmware.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "robot_6900_firmware.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

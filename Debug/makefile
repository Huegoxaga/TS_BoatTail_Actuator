################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
TS_BoatTail_Actuator.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
TS_BoatTail_Actuator.list \

OBJCOPY_BIN += \
TS_BoatTail_Actuator.bin \


# All Target
all: TS_BoatTail_Actuator.elf secondary-outputs

# Tool invocations
TS_BoatTail_Actuator.elf: $(OBJS) $(USER_OBJS) C:\Users\Owner\STM32CubeIDE\workspace_1.2.0\TS_BoatTail_Actuator\STM32L476RGTX_FLASH.ld
	arm-none-eabi-g++ -o "TS_BoatTail_Actuator.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Owner\STM32CubeIDE\workspace_1.2.0\TS_BoatTail_Actuator\STM32L476RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="TS_BoatTail_Actuator.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

TS_BoatTail_Actuator.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "TS_BoatTail_Actuator.list"
	@echo 'Finished building: $@'
	@echo ' '

TS_BoatTail_Actuator.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "TS_BoatTail_Actuator.bin"
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

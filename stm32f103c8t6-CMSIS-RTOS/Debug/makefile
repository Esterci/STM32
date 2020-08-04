################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
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
stm32f103c8t6-CMSIS-RTOS.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
stm32f103c8t6-CMSIS-RTOS.list \

OBJCOPY_BIN += \
stm32f103c8t6-CMSIS-RTOS.bin \


# All Target
all: stm32f103c8t6-CMSIS-RTOS.elf secondary-outputs

# Tool invocations
stm32f103c8t6-CMSIS-RTOS.elf: $(OBJS) $(USER_OBJS) C:\Users\ester\STM32CubeIDE\workspace_1.3.0\stm32f103c8t6-CMSIS-RTOS\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "stm32f103c8t6-CMSIS-RTOS.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\ester\STM32CubeIDE\workspace_1.3.0\stm32f103c8t6-CMSIS-RTOS\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="stm32f103c8t6-CMSIS-RTOS.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

stm32f103c8t6-CMSIS-RTOS.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "stm32f103c8t6-CMSIS-RTOS.list"
	@echo 'Finished building: $@'
	@echo ' '

stm32f103c8t6-CMSIS-RTOS.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "stm32f103c8t6-CMSIS-RTOS.bin"
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

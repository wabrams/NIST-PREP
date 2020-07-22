################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include emlib/subdir.mk
-include CMSIS/EFR32BG22/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: timer_frequency_generation.axf

# Tool invocations
timer_frequency_generation.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -T "timer_frequency_generation.ld" -Xlinker --gc-sections -Xlinker -Map="timer_frequency_generation.map" -mfpu=fpv5-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o timer_frequency_generation.axf "./CMSIS/EFR32BG22/startup_efr32bg22.o" "./CMSIS/EFR32BG22/system_efr32bg22.o" "./emlib/em_cmu.o" "./emlib/em_core.o" "./emlib/em_emu.o" "./emlib/em_gpio.o" "./emlib/em_system.o" "./emlib/em_timer.o" "./src/main.o" -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: timer_frequency_generation.hex'
	arm-none-eabi-objcopy -O ihex "timer_frequency_generation.axf" "timer_frequency_generation.hex"
	@echo ' '

	@echo 'Building bin file: timer_frequency_generation.bin'
	arm-none-eabi-objcopy -O binary "timer_frequency_generation.axf" "timer_frequency_generation.bin"
	@echo ' '

	@echo 'Building s37 file: timer_frequency_generation.s37'
	arm-none-eabi-objcopy -O srec "timer_frequency_generation.axf" "timer_frequency_generation.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "timer_frequency_generation.axf" -A
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) timer_frequency_generation.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

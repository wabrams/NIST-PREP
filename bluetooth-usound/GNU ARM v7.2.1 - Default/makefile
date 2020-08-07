################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include protocol/bluetooth/lib/EFR32BG22/GCC/subdir.mk
-include platform/service/sleeptimer/src/subdir.mk
-include platform/emlib/src/subdir.mk
-include platform/emdrv/sleep/src/subdir.mk
-include platform/emdrv/nvm3/src/subdir.mk
-include platform/emdrv/gpiointerrupt/src/subdir.mk
-include platform/emdrv/dmadrv/src/subdir.mk
-include platform/Device/SiliconLabs/EFR32BG22/Source/GCC/subdir.mk
-include platform/Device/SiliconLabs/EFR32BG22/Source/subdir.mk
-include hardware/kit/common/drivers/subdir.mk
-include hardware/kit/common/bsp/subdir.mk
-include app/bluetooth/common/util/subdir.mk
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
all: bluetooth-usound.axf

# Tool invocations
bluetooth-usound.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -T "C:\Users\Will\SimplicityStudio\v4_workspace\bluetooth-usound\efr32bg22c224f512im40.ld" -Xlinker --gc-sections -Xlinker -Map="bluetooth-usound.map" -mfpu=fpv5-sp-d16 -mfloat-abi=hard --specs=nano.specs -o bluetooth-usound.axf -Wl,--start-group "./app/bluetooth/common/util/infrastructure.o" "./app.o" "./application_properties.o" "./gatt_db.o" "./init_app.o" "./init_board.o" "./init_mcu.o" "./main.o" "./pti.o" "./simple_dsp.o" "./hardware/kit/common/bsp/bsp_stk.o" "./hardware/kit/common/drivers/i2cspm.o" "./hardware/kit/common/drivers/mx25flash_spi.o" "./hardware/kit/common/drivers/retargetio.o" "./hardware/kit/common/drivers/retargetserial.o" "./platform/Device/SiliconLabs/EFR32BG22/Source/GCC/startup_efr32bg22.o" "./platform/Device/SiliconLabs/EFR32BG22/Source/system_efr32bg22.o" "./platform/emdrv/dmadrv/src/dmadrv.o" "./platform/emdrv/gpiointerrupt/src/gpiointerrupt.o" "./platform/emdrv/nvm3/src/nvm3_default.o" "./platform/emdrv/nvm3/src/nvm3_hal_flash.o" "./platform/emdrv/nvm3/src/nvm3_lock.o" "./platform/emdrv/sleep/src/sleep.o" "./platform/emlib/src/em_assert.o" "./platform/emlib/src/em_burtc.o" "./platform/emlib/src/em_cmu.o" "./platform/emlib/src/em_core.o" "./platform/emlib/src/em_cryotimer.o" "./platform/emlib/src/em_crypto.o" "./platform/emlib/src/em_emu.o" "./platform/emlib/src/em_eusart.o" "./platform/emlib/src/em_gpio.o" "./platform/emlib/src/em_i2c.o" "./platform/emlib/src/em_iadc.o" "./platform/emlib/src/em_ldma.o" "./platform/emlib/src/em_msc.o" "./platform/emlib/src/em_pdm.o" "./platform/emlib/src/em_rmu.o" "./platform/emlib/src/em_rtcc.o" "./platform/emlib/src/em_se.o" "./platform/emlib/src/em_system.o" "./platform/emlib/src/em_timer.o" "./platform/emlib/src/em_usart.o" "./platform/service/sleeptimer/src/sl_sleeptimer.o" "./platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.o" "./platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o" "C:\Users\Will\SimplicityStudio\v4_workspace\bluetooth-usound\protocol\bluetooth\lib\EFR32BG22\GCC\libbluetooth.a" "C:\Users\Will\SimplicityStudio\v4_workspace\bluetooth-usound\protocol\bluetooth\lib\EFR32BG22\GCC\binapploader.o" "C:\Users\Will\SimplicityStudio\v4_workspace\bluetooth-usound\platform\emdrv\nvm3\lib\libnvm3_CM33_gcc.a" "C:\Users\Will\SimplicityStudio\v4_workspace\bluetooth-usound\platform\radio\rail_lib\autogen\librail_release\librail_efr32xg22_gcc_release.a" "C:\Users\Will\SimplicityStudio\v4_workspace\bluetooth-usound\protocol\bluetooth\lib\EFR32BG22\GCC\libmbedtls.a" -lm -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: bluetooth-usound.hex'
	arm-none-eabi-objcopy -O ihex "bluetooth-usound.axf" "bluetooth-usound.hex"
	@echo ' '

	@echo 'Building bin file: bluetooth-usound.bin'
	arm-none-eabi-objcopy -O binary "bluetooth-usound.axf" "bluetooth-usound.bin"
	@echo ' '

	@echo 'Building s37 file: bluetooth-usound.s37'
	arm-none-eabi-objcopy -O srec "bluetooth-usound.axf" "bluetooth-usound.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "bluetooth-usound.axf" -A
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) bluetooth-usound.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

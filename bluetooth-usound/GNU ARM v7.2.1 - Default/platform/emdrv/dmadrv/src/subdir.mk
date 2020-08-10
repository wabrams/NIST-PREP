################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../platform/emdrv/dmadrv/src/dmadrv.c 

OBJS += \
./platform/emdrv/dmadrv/src/dmadrv.o 

C_DEPS += \
./platform/emdrv/dmadrv/src/dmadrv.d 


# Each subdirectory must supply rules for building sources it contributes
platform/emdrv/dmadrv/src/dmadrv.o: ../platform/emdrv/dmadrv/src/dmadrv.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DHAL_CONFIG=1' '-DNVM3_DEFAULT_NVM_SIZE=24576' '-D__HEAP_SIZE=0xD00' '-D__STACK_SIZE=0x800' '-D__StackLimit=0x20000000' '-DEFR32BG22C224F512IM40=1' -I"C:\Users\Will\git\ECEN3360\bluetooth-usound" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\emlib\src" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\emlib\inc" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\Device\SiliconLabs\EFR32BG22\Include" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\CMSIS\Include" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\hardware\kit\common\halconfig" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\emdrv\nvm3\src" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\emdrv\dmadrv\src" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\service\sleeptimer\inc" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\emdrv\nvm3\inc" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\hardware\kit\common\drivers" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\hardware\kit\common\bsp" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\radio\rail_lib\chip\efr32\efr32xg2x" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\protocol\bluetooth\ble_stack\inc\common" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\Device\SiliconLabs\EFR32BG22\Source\GCC" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\emdrv\uartdrv\inc" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\service\sleeptimer\src" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\emdrv\dmadrv\inc" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\radio\rail_lib\common" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\protocol\bluetooth\ble_stack\inc\soc" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\service\sleeptimer\config" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\Device\SiliconLabs\EFR32BG22\Source" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\halconfig\inc\hal-config" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\common\inc" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\app\bluetooth\common\util" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\hardware\kit\EFR32BG22_BRD4184A\config" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\emdrv\gpiointerrupt\src" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\emdrv\common\inc" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\radio\rail_lib\protocol\ble" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\emdrv\sleep\src" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\emdrv\sleep\inc" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\bootloader\api" -I"C:\Users\Will\git\ECEN3360\bluetooth-usound\platform\bootloader" -Og -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv5-sp-d16 -mfloat-abi=hard -MMD -MP -MF"platform/emdrv/dmadrv/src/dmadrv.d" -MT"platform/emdrv/dmadrv/src/dmadrv.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



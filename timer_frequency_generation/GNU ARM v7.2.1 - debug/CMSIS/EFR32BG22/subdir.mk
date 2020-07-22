################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.7/platform/Device/SiliconLabs/EFR32BG22/Source/system_efr32bg22.c 

S_UPPER_SRCS += \
C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.7/platform/Device/SiliconLabs/EFR32BG22/Source/GCC/startup_efr32bg22.S 

OBJS += \
./CMSIS/EFR32BG22/startup_efr32bg22.o \
./CMSIS/EFR32BG22/system_efr32bg22.o 

C_DEPS += \
./CMSIS/EFR32BG22/system_efr32bg22.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/EFR32BG22/startup_efr32bg22.o: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.7/platform/Device/SiliconLabs/EFR32BG22/Source/GCC/startup_efr32bg22.S
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Assembler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -c -x assembler-with-cpp -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.7//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.7//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.7//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.7//hardware/kit/common/drivers" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.7//platform/Device/SiliconLabs/EFR32BG22/Include" '-DRETARGET_VCOM=1' '-DEFR32BG22C224F512IM40=1' -mfpu=fpv5-sp-d16 -mfloat-abi=softfp -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMSIS/EFR32BG22/system_efr32bg22.o: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.7/platform/Device/SiliconLabs/EFR32BG22/Source/system_efr32bg22.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DRETARGET_VCOM=1' '-DEFR32BG22C224F512IM40=1' -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.7//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.7//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.7//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.7//hardware/kit/common/drivers" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.7//platform/Device/SiliconLabs/EFR32BG22/Include" -O2 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv5-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"CMSIS/EFR32BG22/system_efr32bg22.d" -MT"CMSIS/EFR32BG22/system_efr32bg22.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk/app/common/util/app_log/app_log.c 

OBJS += \
./gecko_sdk_4.4.5/app/common/util/app_log/app_log.o 

C_DEPS += \
./gecko_sdk_4.4.5/app/common/util/app_log/app_log.d 


# Each subdirectory must supply rules for building sources it contributes
gecko_sdk_4.4.5/app/common/util/app_log/app_log.o: C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk/app/common/util/app_log/app_log.c gecko_sdk_4.4.5/app/common/util/app_log/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG24B310F1536IM48=1' '-DHARDWARE_BOARD_DEFAULT_RF_BAND_2400=1' '-DHARDWARE_BOARD_SUPPORTS_1_RF_BAND=1' '-DHARDWARE_BOARD_SUPPORTS_RF_BAND_2400=1' '-DHFXO_FREQ=39000000' '-DSL_BOARD_NAME="BRD2601B"' '-DSL_BOARD_REV="A01"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"F:\SS5projects\imu_anomaly_detection\config" -I"F:\SS5projects\imu_anomaly_detection\autogen" -I"F:\SS5projects\imu_anomaly_detection" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//extension/machine_learning_applications/component/htm/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//extension/machine_learning_applications/component/libfort" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32MG24/Include" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//app/common/util/app_assert" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//app/common/util/app_log" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Core/Include" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/configuration_over_swo/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/driver/debug/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/dmadrv/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/gpiointerrupt/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/icm20689/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/imu/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/service/iostream/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/driver/leddrv/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/peripheral/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/service/sleeptimer/inc" -I"C:/Users/ijnak/SimplicityStudio/SDKs/gecko_sdk//platform/service/udelay/inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -Wno-unused-parameter -Wno-missing-field-initializers -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.4.5/app/common/util/app_log/app_log.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



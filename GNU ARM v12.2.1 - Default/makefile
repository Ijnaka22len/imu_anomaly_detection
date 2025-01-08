################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include machine_learning_applications_1.0.0/component/libfort/subdir.mk
-include machine_learning_applications_1.0.0/component/htm/src/subdir.mk
-include gecko_sdk_4.4.5/platform/service/udelay/src/subdir.mk
-include gecko_sdk_4.4.5/platform/service/system/src/subdir.mk
-include gecko_sdk_4.4.5/platform/service/sleeptimer/src/subdir.mk
-include gecko_sdk_4.4.5/platform/service/iostream/src/subdir.mk
-include gecko_sdk_4.4.5/platform/service/device_init/src/subdir.mk
-include gecko_sdk_4.4.5/platform/peripheral/src/subdir.mk
-include gecko_sdk_4.4.5/platform/emlib/src/subdir.mk
-include gecko_sdk_4.4.5/platform/emdrv/gpiointerrupt/src/subdir.mk
-include gecko_sdk_4.4.5/platform/emdrv/dmadrv/src/subdir.mk
-include gecko_sdk_4.4.5/platform/driver/leddrv/src/subdir.mk
-include gecko_sdk_4.4.5/platform/driver/debug/src/subdir.mk
-include gecko_sdk_4.4.5/platform/common/toolchain/src/subdir.mk
-include gecko_sdk_4.4.5/platform/common/src/subdir.mk
-include gecko_sdk_4.4.5/platform/Device/SiliconLabs/EFR32MG24/Source/subdir.mk
-include gecko_sdk_4.4.5/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown_usart/subdir.mk
-include gecko_sdk_4.4.5/hardware/driver/imu/src/subdir.mk
-include gecko_sdk_4.4.5/hardware/driver/icm20689/src/subdir.mk
-include gecko_sdk_4.4.5/hardware/driver/configuration_over_swo/src/subdir.mk
-include gecko_sdk_4.4.5/hardware/board/src/subdir.mk
-include gecko_sdk_4.4.5/app/common/util/app_log/subdir.mk
-include autogen/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := imu_anomaly_detection
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: main-build

# Main-build Target
main-build: imu_anomaly_detection.axf

# Tool invocations
imu_anomaly_detection.axf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -T "F:\SS5projects\imu_anomaly_detection\autogen\linkerfile.ld" -u _printf_float -Wl,--no-warn-rwx-segments -Xlinker --gc-sections -Xlinker -Map="imu_anomaly_detection.map" -mfpu=fpv5-sp-d16 -mfloat-abi=hard --specs=nano.specs -o imu_anomaly_detection.axf -Wl,--start-group "./accelerometer.o" "./app.o" "./main.o" "./autogen/sl_board_default_init.o" "./autogen/sl_device_init_clocks.o" "./autogen/sl_event_handler.o" "./autogen/sl_iostream_handles.o" "./autogen/sl_iostream_init_eusart_instances.o" "./autogen/sl_simple_led_instances.o" "./gecko_sdk_4.4.5/app/common/util/app_log/app_log.o" "./gecko_sdk_4.4.5/hardware/board/src/sl_board_control_gpio.o" "./gecko_sdk_4.4.5/hardware/board/src/sl_board_init.o" "./gecko_sdk_4.4.5/hardware/driver/configuration_over_swo/src/sl_cos.o" "./gecko_sdk_4.4.5/hardware/driver/icm20689/src/sl_icm20689.o" "./gecko_sdk_4.4.5/hardware/driver/imu/src/sl_imu_dcm.o" "./gecko_sdk_4.4.5/hardware/driver/imu/src/sl_imu_fuse.o" "./gecko_sdk_4.4.5/hardware/driver/imu/src/sl_imu_icm20689.o" "./gecko_sdk_4.4.5/hardware/driver/imu/src/sl_imu_math.o" "./gecko_sdk_4.4.5/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown_usart/sl_mx25_flash_shutdown.o" "./gecko_sdk_4.4.5/platform/Device/SiliconLabs/EFR32MG24/Source/startup_efr32mg24.o" "./gecko_sdk_4.4.5/platform/Device/SiliconLabs/EFR32MG24/Source/system_efr32mg24.o" "./gecko_sdk_4.4.5/platform/common/src/sl_assert.o" "./gecko_sdk_4.4.5/platform/common/src/sl_slist.o" "./gecko_sdk_4.4.5/platform/common/src/sl_string.o" "./gecko_sdk_4.4.5/platform/common/src/sl_syscalls.o" "./gecko_sdk_4.4.5/platform/common/toolchain/src/sl_memory.o" "./gecko_sdk_4.4.5/platform/driver/debug/src/sl_debug_swo.o" "./gecko_sdk_4.4.5/platform/driver/leddrv/src/sl_led.o" "./gecko_sdk_4.4.5/platform/driver/leddrv/src/sl_simple_led.o" "./gecko_sdk_4.4.5/platform/emdrv/dmadrv/src/dmadrv.o" "./gecko_sdk_4.4.5/platform/emdrv/gpiointerrupt/src/gpiointerrupt.o" "./gecko_sdk_4.4.5/platform/emlib/src/em_burtc.o" "./gecko_sdk_4.4.5/platform/emlib/src/em_cmu.o" "./gecko_sdk_4.4.5/platform/emlib/src/em_core.o" "./gecko_sdk_4.4.5/platform/emlib/src/em_emu.o" "./gecko_sdk_4.4.5/platform/emlib/src/em_eusart.o" "./gecko_sdk_4.4.5/platform/emlib/src/em_gpio.o" "./gecko_sdk_4.4.5/platform/emlib/src/em_ldma.o" "./gecko_sdk_4.4.5/platform/emlib/src/em_msc.o" "./gecko_sdk_4.4.5/platform/emlib/src/em_prs.o" "./gecko_sdk_4.4.5/platform/emlib/src/em_system.o" "./gecko_sdk_4.4.5/platform/emlib/src/em_timer.o" "./gecko_sdk_4.4.5/platform/emlib/src/em_usart.o" "./gecko_sdk_4.4.5/platform/peripheral/src/peripheral_sysrtc.o" "./gecko_sdk_4.4.5/platform/service/device_init/src/sl_device_init_dcdc_s2.o" "./gecko_sdk_4.4.5/platform/service/device_init/src/sl_device_init_dpll_s2.o" "./gecko_sdk_4.4.5/platform/service/device_init/src/sl_device_init_emu_s2.o" "./gecko_sdk_4.4.5/platform/service/device_init/src/sl_device_init_hfxo_s2.o" "./gecko_sdk_4.4.5/platform/service/device_init/src/sl_device_init_lfxo_s2.o" "./gecko_sdk_4.4.5/platform/service/device_init/src/sl_device_init_nvic.o" "./gecko_sdk_4.4.5/platform/service/iostream/src/sl_iostream.o" "./gecko_sdk_4.4.5/platform/service/iostream/src/sl_iostream_eusart.o" "./gecko_sdk_4.4.5/platform/service/iostream/src/sl_iostream_retarget_stdio.o" "./gecko_sdk_4.4.5/platform/service/iostream/src/sl_iostream_stdlib_config.o" "./gecko_sdk_4.4.5/platform/service/iostream/src/sl_iostream_uart.o" "./gecko_sdk_4.4.5/platform/service/sleeptimer/src/sl_sleeptimer.o" "./gecko_sdk_4.4.5/platform/service/sleeptimer/src/sl_sleeptimer_hal_burtc.o" "./gecko_sdk_4.4.5/platform/service/sleeptimer/src/sl_sleeptimer_hal_sysrtc.o" "./gecko_sdk_4.4.5/platform/service/sleeptimer/src/sl_sleeptimer_hal_timer.o" "./gecko_sdk_4.4.5/platform/service/system/src/sl_system_init.o" "./gecko_sdk_4.4.5/platform/service/system/src/sl_system_process_action.o" "./gecko_sdk_4.4.5/platform/service/udelay/src/sl_udelay.o" "./gecko_sdk_4.4.5/platform/service/udelay/src/sl_udelay_armv6m_gcc.o" "./machine_learning_applications_1.0.0/component/htm/src/sl_htm.o" "./machine_learning_applications_1.0.0/component/htm/src/sl_htm_encoder.o" "./machine_learning_applications_1.0.0/component/htm/src/sl_htm_sdr.o" "./machine_learning_applications_1.0.0/component/htm/src/sl_htm_sp.o" "./machine_learning_applications_1.0.0/component/htm/src/sl_htm_tm.o" "./machine_learning_applications_1.0.0/component/htm/src/sl_htm_tm_cell.o" "./machine_learning_applications_1.0.0/component/htm/src/sl_htm_tm_column.o" "./machine_learning_applications_1.0.0/component/htm/src/sl_htm_tm_segment.o" "./machine_learning_applications_1.0.0/component/htm/src/sl_htm_tm_synapse.o" "./machine_learning_applications_1.0.0/component/htm/src/sl_htm_tm_types.o" "./machine_learning_applications_1.0.0/component/htm/src/sl_htm_utils.o" "./machine_learning_applications_1.0.0/component/libfort/fort.o" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building out file: imu_anomaly_detection.out'
	arm-none-eabi-objcopy "imu_anomaly_detection.axf" "imu_anomaly_detection.out"
	@echo ' '

	@echo 'Building hex file: imu_anomaly_detection.hex'
	arm-none-eabi-objcopy -O ihex "imu_anomaly_detection.axf" "imu_anomaly_detection.hex"
	@echo ' '

	@echo 'Building bin file: imu_anomaly_detection.bin'
	arm-none-eabi-objcopy -O binary "imu_anomaly_detection.axf" "imu_anomaly_detection.bin"
	@echo ' '

	@echo 'Building s37 file: imu_anomaly_detection.s37'
	arm-none-eabi-objcopy -O srec "imu_anomaly_detection.axf" "imu_anomaly_detection.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "imu_anomaly_detection.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) imu_anomaly_detection.axf
	-@echo ' '

.PHONY: all clean dependents main-build

-include ../makefile.targets

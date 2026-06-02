################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/Ti/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/new/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang" -I"C:/Users/new/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/Debug" -I"D:/Ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"D:/Ti/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1086911071: ../gpio_toggle_output.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"D:/Ti/Sysconfigir/sysconfig_cli.bat" --script "C:/Users/new/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/gpio_toggle_output.syscfg" -o "." -s "D:/Ti/mspm0_sdk_2_01_00_03/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-1086911071 ../gpio_toggle_output.syscfg
device.opt: build-1086911071
device.cmd.genlibs: build-1086911071
ti_msp_dl_config.c: build-1086911071
ti_msp_dl_config.h: build-1086911071
Event.dot: build-1086911071

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/Ti/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/new/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang" -I"C:/Users/new/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/Debug" -I"D:/Ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"D:/Ti/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: D:/Ti/mspm0_sdk_2_01_00_03/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/Ti/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/new/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang" -I"C:/Users/new/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/Debug" -I"D:/Ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"D:/Ti/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '



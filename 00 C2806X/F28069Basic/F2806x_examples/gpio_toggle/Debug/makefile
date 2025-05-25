################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

EMPTY := 
SPACE := $(EMPTY) $(EMPTY)
CG_TOOL_ROOT := C:/Program Files/Texas Instruments/ccsv4/tools/compiler/C2000 Code Generation Tools 5.2.10

ORDERED_OBJS += \
$(GEN_CMDS_QUOTED) \
-l"rts2800_fpu32_fast_supplement.lib" \
-l"rts2800_fpu32.lib" \
"./F2806x_usDelay.obj" \
"./F2806x_SysCtrl.obj" \
"./F2806x_PieVect.obj" \
"./F2806x_PieCtrl.obj" \
"./F2806x_Gpio.obj" \
"./F2806x_GlobalVariableDefs.obj" \
"./F2806x_DefaultIsr.obj" \
"./F2806x_CodeStartBranch.obj" \
"./Example_2806xGpioToggle.obj" \
-l"IQmath_fpu32.lib" \
"C:/TI/controlSUITE/device_support/f2806x/v100/F2806x_common/cmd/28069_RAM_lnk.cmd" \
"../IQmath.lib" \
"../IQmath_fpu32.lib" \
"../rts2800_fpu32_fast_supplement.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Example_2806xGpioToggle.out

# Tool invocations
Example_2806xGpioToggle.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: Linker'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/C2000 Code Generation Tools 5.2.10/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --large_memory_model --unified_memory --cla_support=cla0 --float_support=fpu32 --output_all_syms --cdebug_asm_data -z -m"Example_2806xGpioToggle.map" --stack_size=0x200 --verbose_diagnostics --issue_remarks --warn_sections -i"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/C2000 Code Generation Tools 5.2.10/lib" -i"C:/TI/controlSUITE/device_support/f2806x/v100/F2806x_common/lib" -i"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/C2000 Code Generation Tools 5.2.10/include" -i"C:/TI/controlSUITE/device_support/f2806x/v100/F2806x_examples/gpio_toggle" -i"/F2806x_common/lib" -i"/v15c/lib" -i"/V100/lib" --priority --reread_libs --entry_point=code_start --rom_model -o "Example_2806xGpioToggle.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C2000_EXECUTABLE_OUTPUTS__QTD) "Example_2806xGpioToggle.out"
	-$(RM) ".\Example_2806xGpioToggle.obj" ".\F2806x_CodeStartBranch.obj" ".\F2806x_DefaultIsr.obj" ".\F2806x_GlobalVariableDefs.obj" ".\F2806x_Gpio.obj" ".\F2806x_PieCtrl.obj" ".\F2806x_PieVect.obj" ".\F2806x_SysCtrl.obj" ".\F2806x_usDelay.obj" 
	-$(RM) ".\F2806x_CodeStartBranch.pp" ".\F2806x_usDelay.pp" 
	-$(RM) ".\Example_2806xGpioToggle.pp" ".\F2806x_DefaultIsr.pp" ".\F2806x_GlobalVariableDefs.pp" ".\F2806x_Gpio.pp" ".\F2806x_PieCtrl.pp" ".\F2806x_PieVect.pp" ".\F2806x_SysCtrl.pp" 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

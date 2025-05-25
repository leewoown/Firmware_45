################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/include" --include_path="/packages/ti/xdais" --include_path="D:/F28069Basic/F2806x_headers/include" --include_path="D:/F28069Basic/F2806x_common/include" --include_path="D:/libs/math/IQmath/v15c/include" --include_path="D:/libs/math/FPUfastRTS/V100/include" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --large_memory_model --unified_memory --cla_support=cla0 --float_support=fpu32 --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_CodeStartBranch.obj: D:/F28069Basic/F2806x_common/source/F2806x_CodeStartBranch.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/include" --include_path="/packages/ti/xdais" --include_path="D:/F28069Basic/F2806x_headers/include" --include_path="D:/F28069Basic/F2806x_common/include" --include_path="D:/libs/math/IQmath/v15c/include" --include_path="D:/libs/math/FPUfastRTS/V100/include" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --large_memory_model --unified_memory --cla_support=cla0 --float_support=fpu32 --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_CpuTimers.obj: D:/F28069Basic/F2806x_common/source/F2806x_CpuTimers.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/include" --include_path="/packages/ti/xdais" --include_path="D:/F28069Basic/F2806x_headers/include" --include_path="D:/F28069Basic/F2806x_common/include" --include_path="D:/libs/math/IQmath/v15c/include" --include_path="D:/libs/math/FPUfastRTS/V100/include" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --large_memory_model --unified_memory --cla_support=cla0 --float_support=fpu32 --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_DefaultIsr.obj: D:/F28069Basic/F2806x_common/source/F2806x_DefaultIsr.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/include" --include_path="/packages/ti/xdais" --include_path="D:/F28069Basic/F2806x_headers/include" --include_path="D:/F28069Basic/F2806x_common/include" --include_path="D:/libs/math/IQmath/v15c/include" --include_path="D:/libs/math/FPUfastRTS/V100/include" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --large_memory_model --unified_memory --cla_support=cla0 --float_support=fpu32 --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_GlobalVariableDefs.obj: D:/F28069Basic/F2806x_headers/source/F2806x_GlobalVariableDefs.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/include" --include_path="/packages/ti/xdais" --include_path="D:/F28069Basic/F2806x_headers/include" --include_path="D:/F28069Basic/F2806x_common/include" --include_path="D:/libs/math/IQmath/v15c/include" --include_path="D:/libs/math/FPUfastRTS/V100/include" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --large_memory_model --unified_memory --cla_support=cla0 --float_support=fpu32 --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_PieCtrl.obj: D:/F28069Basic/F2806x_common/source/F2806x_PieCtrl.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/include" --include_path="/packages/ti/xdais" --include_path="D:/F28069Basic/F2806x_headers/include" --include_path="D:/F28069Basic/F2806x_common/include" --include_path="D:/libs/math/IQmath/v15c/include" --include_path="D:/libs/math/FPUfastRTS/V100/include" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --large_memory_model --unified_memory --cla_support=cla0 --float_support=fpu32 --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_PieVect.obj: D:/F28069Basic/F2806x_common/source/F2806x_PieVect.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/include" --include_path="/packages/ti/xdais" --include_path="D:/F28069Basic/F2806x_headers/include" --include_path="D:/F28069Basic/F2806x_common/include" --include_path="D:/libs/math/IQmath/v15c/include" --include_path="D:/libs/math/FPUfastRTS/V100/include" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --large_memory_model --unified_memory --cla_support=cla0 --float_support=fpu32 --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_SysCtrl.obj: D:/F28069Basic/F2806x_common/source/F2806x_SysCtrl.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/include" --include_path="/packages/ti/xdais" --include_path="D:/F28069Basic/F2806x_headers/include" --include_path="D:/F28069Basic/F2806x_common/include" --include_path="D:/libs/math/IQmath/v15c/include" --include_path="D:/libs/math/FPUfastRTS/V100/include" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --large_memory_model --unified_memory --cla_support=cla0 --float_support=fpu32 --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_usDelay.obj: D:/F28069Basic/F2806x_common/source/F2806x_usDelay.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/include" --include_path="/packages/ti/xdais" --include_path="D:/F28069Basic/F2806x_headers/include" --include_path="D:/F28069Basic/F2806x_common/include" --include_path="D:/libs/math/IQmath/v15c/include" --include_path="D:/libs/math/FPUfastRTS/V100/include" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --large_memory_model --unified_memory --cla_support=cla0 --float_support=fpu32 --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '



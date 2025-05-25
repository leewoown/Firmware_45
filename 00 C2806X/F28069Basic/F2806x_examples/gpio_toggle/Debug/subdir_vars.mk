################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
LIB_SRCS += \
../IQmath.lib \
../IQmath_fpu32.lib \
../rts2800_fpu32_fast_supplement.lib 

C_SRCS += \
../Example_2806xGpioToggle.c \
../F2806x_DefaultIsr.c \
../F2806x_GlobalVariableDefs.c \
../F2806x_Gpio.c \
../F2806x_PieCtrl.c \
../F2806x_PieVect.c \
../F2806x_SysCtrl.c 

ASM_SRCS += \
../F2806x_CodeStartBranch.asm \
../F2806x_usDelay.asm 

CMD_SRCS += \
C:/TI/controlSUITE/device_support/f2806x/v100/F2806x_common/cmd/28069_RAM_lnk.cmd 

ASM_DEPS += \
./F2806x_CodeStartBranch.pp \
./F2806x_usDelay.pp 

OBJS += \
./Example_2806xGpioToggle.obj \
./F2806x_CodeStartBranch.obj \
./F2806x_DefaultIsr.obj \
./F2806x_GlobalVariableDefs.obj \
./F2806x_Gpio.obj \
./F2806x_PieCtrl.obj \
./F2806x_PieVect.obj \
./F2806x_SysCtrl.obj \
./F2806x_usDelay.obj 

C_DEPS += \
./Example_2806xGpioToggle.pp \
./F2806x_DefaultIsr.pp \
./F2806x_GlobalVariableDefs.pp \
./F2806x_Gpio.pp \
./F2806x_PieCtrl.pp \
./F2806x_PieVect.pp \
./F2806x_SysCtrl.pp 

OBJS__QTD += \
".\Example_2806xGpioToggle.obj" \
".\F2806x_CodeStartBranch.obj" \
".\F2806x_DefaultIsr.obj" \
".\F2806x_GlobalVariableDefs.obj" \
".\F2806x_Gpio.obj" \
".\F2806x_PieCtrl.obj" \
".\F2806x_PieVect.obj" \
".\F2806x_SysCtrl.obj" \
".\F2806x_usDelay.obj" 

ASM_DEPS__QTD += \
".\F2806x_CodeStartBranch.pp" \
".\F2806x_usDelay.pp" 

C_DEPS__QTD += \
".\Example_2806xGpioToggle.pp" \
".\F2806x_DefaultIsr.pp" \
".\F2806x_GlobalVariableDefs.pp" \
".\F2806x_Gpio.pp" \
".\F2806x_PieCtrl.pp" \
".\F2806x_PieVect.pp" \
".\F2806x_SysCtrl.pp" 

C_SRCS_QUOTED += \
"../Example_2806xGpioToggle.c" \
"../F2806x_DefaultIsr.c" \
"../F2806x_GlobalVariableDefs.c" \
"../F2806x_Gpio.c" \
"../F2806x_PieCtrl.c" \
"../F2806x_PieVect.c" \
"../F2806x_SysCtrl.c" 

ASM_SRCS_QUOTED += \
"../F2806x_CodeStartBranch.asm" \
"../F2806x_usDelay.asm" 



// TI File $Revision: /main/2 $ 
// Checkin $Date: January 4, 2011   10:02:48 $ 
//###########################################################################
//
// FILE:    Example_2806xFPU_hardware.c
//
// TITLE:   F2806x Device FPUGetting Started Program.
//
// ASSUMPTIONS:
//
//    This program requires the F2806x header files.
//
//    Other then boot mode configuration, no other hardware configuration
//    is required.
//
//
//    As supplied, this project is configured for "boot to SARAM"
//    operation.  The F2806x Boot Mode table is shown below.
//
//       $Boot_Table:
//
//    While an emulator is connected to your device, the TRSTn pin = 1,
//    which sets the device into EMU_BOOT boot mode. In this mode, the
//    peripheral boot modes are as follows:
//
//      Boot Mode:       EMU_KEY        EMU_BMODE
//                       (0xD00)	     (0xD01)
//      ---------------------------------------
//      Wait             !=0x55AA        X
//      I/O              0x55AA	         0x0000
//      SCI              0x55AA	         0x0001
//      Wait             0x55AA	         0x0002
//      Get_Mode         0x55AA	         0x0003
//      SPI              0x55AA	         0x0004
//      I2C              0x55AA	         0x0005
//      OTP              0x55AA	         0x0006
//      ECANA            0x55AA	         0x0007
//      SARAM            0x55AA	         0x000A	  <-- "Boot to SARAM"
//      Flash            0x55AA	         0x000B
//      Wait             0x55AA          Other
//
//   Write EMU_KEY to 0xD00 and EMU_BMODE to 0xD01 via the debugger
//   according to the Boot Mode Table above. Build/Load project,
//   Reset the device, and Run example
//
//   $End_Boot_Table
//
// DESCRIPTION:
//
// The code calculates two y=mx+b equations.  The variables are all
// 32-bit floating-point.
//
// Two projects are supplied in F2806x_examples_ccsv4:
//
// Example_2806xFPU_hardware (floating-point):
//
//    If the fpu_hardware project is used then the compiler
//    will generate floating point instructions to do these calculations.
//    To compile the project for floating point, the following Build Options were used:
//    1. Project->Properties-> C/C++ Build window-> Basic Settings->
//       C2000 Compiler Vx.x
//           a. in All Options textbox: add "--float_support=fpu32" .
//           b. OR in Runtime Model Options, under "Specify floating point support
//             (--float_support) pull-down menu: Select "fpu32".
//    2. Project->Properties-> C/C++ Build window-> Basic Settings->
//       C2000 Linker Vx.x-> File Search Path
//           a. In "Include linker file or command file as input (--library, -l)"
//              box, click green plus sign and add  rts2800_fpu32.lib
//              (run-time support library).
//    3. Not included in this example: If the project includes any other libraries,
//       they must also be compiled with floating point instructions.
//
// Example_2806xFPU_software (fixed-point emulates floating-point with software):
//
//    If the fpu_software project is used, then the compiler
//    will only used fixed point instructions.  This means the runtime
//    support library will be used to emulate floating point.
//    This will also run on C28x devices without the floating point unit.
//    To compile the project for fixed point, the following Build Options were used:
//    1. Project->Properties-> C/C++ Build window-> Basic Settings->
//       C2000 Compiler Vx.x
//           a. in All Options textbox: "--float_support=fpu32" is removed.
//           b. OR in Runtime Model Options, under "Specify floating point support
//             (--float_support) pull-down menu: Select "None".
//    2. Project->Properties-> C/C++ Build window-> Basic Settings->
//       C2000 Linker Vx.x-> File Search Path
//           a. In "Include linker file or command file as input (--library, -l)"
//              box, click green plus sign and add  rts2800.lib or rts2800_ml.lib
//              (run-time support library).
//    3. Not included in this example: If the project includes any other libraries,
//       they must also be compiled with fixed point instructions.
//
// Watch Variables:
//           y1
//           y2
//           FPU registers (optional)
//
//###########################################################################
// $TI Release: 2806x C/C++ Header Files and Peripheral Examples V1.00 $ 
// $Release Date: January 11, 2011 $ 
//###########################################################################


#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

float y1, y2;
float m1, m2;
float x1, x2;
float b1, b2;


void main(void)
{

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2806x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO:
// This example function is found in the F2806x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example


// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2806x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2806x_DefaultIsr.c.
// This function is found in F2806x_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.


// Step 5. User specific code, enable interrupts:

//
// Calculate two y=mx+b equations.

   y1 = 0;
   y2 = 0;
   m1 = .5;
   m2 = .6;
   x1 = 3.4;
   x2 = 7.3;
   b1 = 4.2;
   b2 = 8.9;

   y1 = m1*x1 + b1;
   y2 = m2*x2 + b2;


   ESTOP0;  // This is a software breakpoint
}


//===========================================================================
// No more.
//===========================================================================

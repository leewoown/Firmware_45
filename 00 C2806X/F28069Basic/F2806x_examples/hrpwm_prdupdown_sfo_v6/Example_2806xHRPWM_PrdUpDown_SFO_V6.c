// TI File $Revision: /main/2 $
// Checkin $Date: January 4, 2011   10:03:03 $
//###########################################################################
//
// FILE:    Example_2806xHRPWM_PrdUpDown_SFO_V6.c
//
// TITLE:   F2806x Device HRPWM SFO V6 High-Resolution Period (Up-Down Count) example
//
//    This program requires the F2806x header files, which include
//    the following files required for this example:
//    SFO_V6.h and SFO_TI_Build_V6.lib
//
//    Monitor ePWM1A (GPIO0) pin on an oscilloscope.
//
//
//    As supplied, this project is configured for "boot to SARAM"
//    operation.  The F2806x Boot Mode table is shown below.
//    For information on configuring the boot mode of an eZdsp,
//    please refer to the documentation included with the eZdsp,
//
//    $Boot_Table:
//
//    While an emulator is connected to your device, the TRSTn pin = 1,
//    which sets the device into EMU_BOOT boot mode. In this mode, the
//    peripheral boot modes are as follows:
//
//      Boot Mode:       EMU_KEY        EMU_BMODE
//                       (0xD00)         (0xD01)
//      ---------------------------------------
//      Wait             !=0x55AA        X
//      I/O              0x55AA          0x0000
//      SCI              0x55AA          0x0001
//      Wait             0x55AA          0x0002
//      Get_Mode         0x55AA          0x0003
//      SPI              0x55AA          0x0004
//      I2C              0x55AA          0x0005
//      OTP              0x55AA          0x0006
//      eCANA            0x55AA          0x0007
//      SARAM            0x55AA          0x000A   <-- "Boot to SARAM"
//      Flash            0x55AA          0x000B
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
//       This example modifies the MEP control registers to show edge displacement
//       for high-resolution period with ePWM in Up-Down count mode
//       due to the HRPWM control extension of the respective ePWM module.
//
//       This example calls the following TI's MEP Scale Factor Optimizer (SFO)
//       software library V6 functions:
//
//
//       int SFO();
//            updates MEP_ScaleFactor dynamically when HRPWM is in use
//            updates HRMSTEP register (exists only in EPwm1Regs register space)
//            with MEP_ScaleFactor value
//            - returns 2 if error: MEP_ScaleFactor is greater than maximum value of 255
//              (Auto-conversion may not function properly under this condition)
//            - returns 1 when complete for the specified channel
//            - returns 0 if not complete for the specified channel

//
//
//       This example is intended to explain the HRPWM configuration for high
//       resolution period/frequency. The code can be
//       optimized for code efficiency. Refer to TI's Digital power application
//       examples and TI Digital Power Supply software libraries for details.
//
//       ePWM1A (GPIO0) will have fine edge movement due to the HRPWM logic
//
//       =======================================================================
//       NOTE: For more information on using the SFO software library, see the
//       F2806x High-Resolution Pulse Width Modulator (HRPWM) Reference Guide
//       =======================================================================
//
//  To load and run this example:
//            1. **!!IMPORTANT!!** - in SFO_V6.h, set PWM_CH to the max used 
//               HRPWM channel # plus one. For example, for the FF2806x, the
//               maximum number of HRPWM channels is 7. 7+1=8, so set
//               #define PWM_CH 8 in SFO_V6.h. (Default is 5)
//				 - Note for this specific example, you could set #define PWM_CH 2 
//                 (because it only uses ePWM1), but to cover all examples, PWM_CH
//                 is currently set to a default value of 5.
//            
//            2. Load the code and add the following watch variables to the watch window:
//               - UpdateFine
//               - PeriodFine
//               - EPwm1Regs.TBPRD
//               - EPwm1Regs.TBPRDHR   
//            3. Run this example at maximum SYSCLKOUT (60 or 40 MHz)
//            4. Activate Real time mode
//            5. Run the code
//            6. Watch ePWM1A waveform on a Oscillosope
//            7. In the watch window:
//               Set the variable UpdateFine = 1  to observe the ePWMxA output
//               with HRPWM capabilites (default)
//               Observe the period/frequency of the waveform changes in fine MEP steps
//            8. In the watch window:
//               Change the variable UpdateFine to 0, to observe the
//               ePWMxA output without HRPWM capabilites
//               
//
//###########################################################################
// $TI Release: 2806x C/C++ Header Files and Peripheral Examples V1.00 $
// $Release Date: January 11, 2011 $
//###########################################################################

#include "DSP28x_Project.h"         // DSP280xx Headerfile
#include "SFO_V6.h"

// *!!IMPORTANT!!*
// UPDATE NUMBER OF HRPWM CHANNELS + 1  USED IN SFO_V6.H
// i.e.: #define PWM_CH 5   // Configured for 4 HRPWM channels, but FF2806x has a maximum of 7 HRPWM channels (8=7+1)

// Declare your function prototypes here
//---------------------------------------------------------------
void HRPWM_Config(int);
void error(void);

// General System variables - useful for debug
Uint16 UpdateFine, PeriodFine, status;

//====================================================================
// The following declarations are required in order to use the SFO
// library functions:
//
int MEP_ScaleFactor; // Global variable used by teh SFO library
                     // Result can be used for all HRPWM channels
                     // This variable is also copied to HRMSTEP
                     // register by SFO() function.

// Array of pointers to EPwm register structures:
// *ePWM[0] is defined as dummy value not used in the example
volatile struct EPWM_REGS *ePWM[PWM_CH] =
             {  &EPwm1Regs, &EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs};

//====================================================================

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
   InitEPwmGpio();

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

// Step 4. Initialize all the Device Peripherals:
// This function is found in F2806x_InitPeripherals.c
// InitPeripherals();  // Not required for this example

// For this example, only initialize the ePWM
// Step 5. User specific code, enable interrupts:

    UpdateFine = 1;
    PeriodFine = 0;
    status = SFO_INCOMPLETE;

   // Calling SFO() updates the HRMSTEP register with calibrated MEP_ScaleFactor.
   // HRMSTEP must be populated with a scale factor value prior to enabling
   // high resolution period control.

    while  (status== SFO_INCOMPLETE){  // Call until complete
        status = SFO();
        if (status == SFO_ERROR) {
            error();    // SFO function returns 2 if an error occurs & # of MEP steps/coarse step
        }              // exceeds maximum of 255.
    }

// Some useful PWM period vs Frequency values
//  TBCLK = 60 MHz
//===================
//  Period   Freq
//  1000     30 KHz
//  800    37.5 KHz
//  600      50 KHz
//  500      60 KHz
//  250     120 KHz
//  200     150 KHz
//  100     300 KHz
//  50      600 KHz
//  30        1 MHz
//  25      1.2 MHz
//  20      1.5 MHz
//  12      2.5 MHz
//  10        3 MHz
//  9       3.3 MHz
//  8       3.8 MHz
//  7       4.3 MHz
//  6       5.0 MHz
//  5       6.0 MHz


//====================================================================
// ePWM and HRPWM register initialization
//====================================================================
   HRPWM_Config(20);        // ePWMx target
   for(;;)
   {
        // Sweep PeriodFine as a Q16 number from 0.2 - 0.999
        for(PeriodFine = 0x3333; PeriodFine < 0xFFBF; PeriodFine++)
        {

            if(UpdateFine)
            {
            /*
            // Because auto-conversion is enabled, the desired
            // fractional period must be written directly to the
            // TBPRDHR (or TBPRDHRM) register in Q16 format
            // (lower 8-bits are ignored)

            EPwm1Regs.TBPRDHR = PeriodFine;

            // The hardware will automatically scale
            // the fractional period by the MEP_ScaleFactor
            // in the HRMSTEP register (which is updated
            // by the SFO calibration software).

            // Hardware conversion:
            // MEP delay movement = ((TBPRDHR(15:0) >> 8) *  HRMSTEP(7:0) + 0x80) >> 8

            */
                EPwm1Regs.TBPRDHR = PeriodFine; //In Q16 format

            }
            else
            {
            // No high-resolution movement on TBPRDHR.
                 EPwm1Regs.TBPRDHR = 0;
            }

            // Call the scale factor optimizer lib function SFO(0)
            // periodically to track for any change due to temp/voltage.
            // This function generates MEP_ScaleFactor by running the
            // MEP calibration module in the HRPWM logic. This scale
            // factor can be used for all HRPWM channels. HRMSTEP
            // register is automatically updated by the SFO function.

            status = SFO(); // in background, MEP calibration module continuously updates MEP_ScaleFactor
            if (status == SFO_ERROR) {
                error();   // SFO function returns 2 if an error occurs & # of MEP steps/coarse step
            }              // exceeds maximum of 255.


        } // end PeriodFine for loop

    }     // end infinite for loop

}         // end main

//=============================================================
// FUNCTION:    HRPWM_Config
// DESCRIPTION: Configures ePWM1 and sets up HRPWM
//              on ePWM1A
//
// PARAMETERS:  period - desired PWM period in TBCLK counts
// RETURN:      N/A
//=============================================================

void HRPWM_Config(period)
{

// ePWM channel register configuration with HRPWM
// ePWMxA toggle low/high with MEP control on Rising edge
   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;     // Disable TBCLK within the EPWM
   EDIS;
   
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;             // set Shadow load
    EPwm1Regs.TBPRD = period;                          // PWM frequency = 1/(2*TBPRD)
    EPwm1Regs.CMPA.half.CMPA = period / 2;             // set duty 50% initially
    EPwm1Regs.CMPA.half.CMPAHR = (1 << 8);             // initialize HRPWM extension
    EPwm1Regs.CMPB = period / 2;                       // set duty 50% initially
    EPwm1Regs.TBPHS.all = 0;
    EPwm1Regs.TBCTR = 0;

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;     // Select up-down count mode
    EPwm1Regs.TBCTL.bit.PHSEN = TB_ENABLE;             // TBCTR phase load on SYNC (required for updown count HR control
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;              // TBCLK = SYSCLKOUT
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 11;

    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;      // LOAD CMPA on CTR = 0
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;


    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;                 // PWM toggle high/low
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CAU = AQ_SET;                 // PWM toggle high/low
    EPwm1Regs.AQCTLB.bit.CAD = AQ_CLEAR;

    EALLOW;
    EPwm1Regs.HRCNFG.all = 0x0;
    EPwm1Regs.HRCNFG.bit.EDGMODE = HR_BEP;          // MEP control on both edges
    EPwm1Regs.HRCNFG.bit.CTLMODE = HR_CMP;          // CMPAHR and TBPRDHR HR control
    EPwm1Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO_PRD; // load on CTR = 0 and CTR = TBPRD
    EPwm1Regs.HRCNFG.bit.AUTOCONV = 1;              // Enable autoconversion for HR period

    EPwm1Regs.HRPCTL.bit.TBPHSHRLOADE = 1;          // Enable TBPHSHR sync (required for updwn count HR control)
    EPwm1Regs.HRPCTL.bit.HRPE = 1;                  // Turn on high-resolution period control.


    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;           // Enable TBCLK within the EPWM
    EPwm1Regs.TBCTL.bit.SWFSYNC = 1;                // Synchronize high resolution phase to start HR period
    EDIS;
    
}


void error (void) {
    ESTOP0;         // Stop here and handle error
}

// No more

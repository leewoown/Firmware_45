// TI File $Revision: /main/2 $
// Checkin $Date: January 4, 2011   10:02:56 $
//###########################################################################
//
// FILE:    Example_2806xHRPWM_Duty_SFO_V6.c
//
// TITLE:   F2806x Device HRPWM SFO V6 Duty Cycle example
//
//    This program requires the F2806x header files, which include
//    the following files required for this example:
//    SFO_V6.h and SFO_TI_Build_V6.lib
//
//    Monitor ePWM1A-ePWM4A (GPIO0-GPIO7) pins on an oscilloscope.
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
//       due to the HRPWM control extension of the respective ePWM module.
//
//       This example calls the following TI's MEP Scale Factor Optimizer (SFO)
//       software library V6 functions:
//
//
//       int SFO();
//            updates MEP_ScaleFactor dynamically when HRPWM is in use
//            updates HRMSTEP register (exists only in EPwm1Regs register space but valid for all channels)
//            with MEP_ScaleFactor value
//            - returns 2 if error: MEP_ScaleFactor is greater than maximum value of 255
//              (Auto-conversion may not function properly under this condition)
//            - returns 1 when complete for the specified channel
//            - returns 0 if not complete for the specified channel
//
//
//
//       This example is intended to explain the HRPWM capabilities. The code can be
//       optimized for code efficiency. Refer to TI's Digital power application
//       examples and TI Digital Power Supply software libraries for details.
//
//       All ePWM1A-7A channels will have fine
//       edge movement due to the HRPWM logic
//
//       =======================================================================
//       NOTE: For more information on using the SFO software library, see the
//       F2806x High-Resolution Pulse Width Modulator (HRPWM) Reference Guide
//       =======================================================================
//
//  To load and run this example:
//            1. **!!IMPORTANT!!** - in SFO_V6.h, set PWM_CH to the max number of
//               HRPWM channels plus one. For example, for the FF2806x, the
//               maximum number of HRPWM channels is 7. 7+1=8, so set
//               #define PWM_CH 8 in SFO_V6.h. (Default is 8)
//            2. In this file, set #define AUTOCONVERT to 1 to enable MEP step auto-conversion
//               logic. Otherwise, to manually perform MEP calculations in software, clear
//               to 0.
//            2. Run this example at maximum SYSCLKOUT (60MHz)
//            3. Load the Example_2806xHRPWM_Duty_SFO_V6.gel and observe variables in the watch window
//            4. Activate Real time mode
//            5. Run the code
//            6. Watch ePWM A channel waveforms on a Oscillosope
//            7. In the watch window:
//               Set the variable UpdateFine = 1  to observe the ePWMxA output
//               with HRPWM capabilites (default)
//               Observe the duty cycle of the waveform change in fine MEP steps
//            8. In the watch window:
//               Change the variable UpdateFine to 0, to observe the
//               ePWMxA output without HRPWM capabilites
//               Observe the duty cycle of the waveform change in coarse SYSCLKOUT cycle steps.
//
//###########################################################################
// $TI Release: 2806x C/C++ Header Files and Peripheral Examples V1.00 $
// $Release Date: January 11, 2011 $
//###########################################################################

#include "DSP28x_Project.h"         // DSP280xx Headerfile
#include "SFO_V6.h"

// *!!IMPORTANT!!*
// UPDATE NUMBER OF HRPWM CHANNELS + 1  USED IN SFO_V6.H
// i.e.: #define PWM_CH 5   // FF2806x has a maximum of 7 HRPWM channels (8=7+1)

#define AUTOCONVERT 1       // 1 = Turn auto-conversion ON, 0 = Turn auto-conversion OFF

// Declare your function prototypes here
//---------------------------------------------------------------
void HRPWM_Config(int);
void error(void);

// General System variables - useful for debug
Uint16 UpdateFine, DutyFine, status, CMPA_reg_val, CMPAHR_reg_val;


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
    // Local variables
    int i;
    Uint32 temp;

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2802x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO:
// This example function is found in the DSP2802x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example
   InitEPwmGpio();

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2802x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2802x_DefaultIsr.c.
// This function is found in DSP2802x_PieVect.c.
   InitPieVectTable();

// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2802x_InitPeripherals.c
// InitPeripherals();  // Not required for this example

// For this example, only initialize the ePWM
// Step 5. User specific code, enable interrupts:
    UpdateFine = 1;
    DutyFine   = 0;
    status = SFO_INCOMPLETE;

   // Calling SFO() updates the HRMSTEP register with calibrated MEP_ScaleFactor.
   // MEP_ScaleFactor/HRMSTEP must be filled with calibrated value in order to
   // use in equations below.

    while  (status== SFO_INCOMPLETE){  // Call until complete
        status = SFO();
        if (status == SFO_ERROR) {
            error();    // SFO function returns 2 if an error occurs & # of MEP steps/coarse step
        }              // exceeds maximum of 255.
    }


// Some useful Period vs Frequency values
//  SYSCLKOUT =     60 MHz       40 MHz
//  --------------------------------------
//  Period          Frequency    Frequency
//  1000            60 kHz       40 kHz
//  800             75 kHz       50 kHz
//  600             100 kHz      67 kHz
//  500             120 kHz      80 kHz
//  250             240 kHz      160 kHz
//  200             300 kHz      200 kHz
//  100             600 kHz      400 kHz
//  50              1.2 Mhz      800 kHz
//  25              2.4 Mhz      1.6 MHz
//  20              3.0 Mhz      2.0 MHz
//  12              5.0 MHz      3.3 MHz
//  10              6.0 MHz      4.0 MHz
//  9               6.7 MHz      4.4 MHz
//  8               7.5 MHz      5.0 MHz
//  7               8.6 MHz      5.7 MHz
//  6               10.0 MHz     6.6 MHz
//  5               12.0 MHz     8.0 MHz

//====================================================================
// ePWM and HRPWM register initialization
//====================================================================

   HRPWM_Config(10);        // ePWMx target
   EALLOW;


   for(;;)
   {
        // Sweep DutyFine as a Q15 number from 0.2 - 0.999
        for(DutyFine = 0x2300; DutyFine < 0x7000; DutyFine++)
        {

            if(UpdateFine)
            {
            /*
            // CMPA_reg_val is calculated as a Q0.
            // Since DutyFine is a Q15 number, and the period is Q0
            // the product is Q15. So to store as a Q0, we shift right
            // 15 bits.

            CMPA_reg_val = ((long)DutyFine * EPwm1Regs.TBPRD)>>15;

            // This next step is to obtain the remainder which was
            // truncated during our 15 bit shift above.
            // compute the whole value, and then subtract CMPA_reg_val
            // shifted LEFT 15 bits:
            temp = ((long)DutyFine * EPwm1Regs.TBPRD) ;
            temp = temp - ((long)CMPA_reg_val<<15);

            ** If auto-conversion is disabled, the following step can be skipped.
            // If autoconversion is enabled, the SFO function will write
            // the MEP_ScaleFactor to the HRMSTEP register and the hardware
            // will automatically scale the remainder in the CMPAHR register
            // by the MEP_ScaleFactor.
            // Because the remainder calculated above (temp) is in Q15 format,
            // it must be shifted left by 1 to convert to Q16 format for the
            // hardware to properly convert.
            CMPAHR_reg_val = temp<<1;

            ** If auto-conversion is enabled, the following step is performed
               automatically in hardware and can be skipped
            // This obtains the MEP count in digits, from
            // 0,1, .... MEP_Scalefactor.
            // 0x0080 (0.5 in Q8) is converted to 0.5 in Q15 by shifting left 7.
            // This is added to fractional duty*MEP_SF product in order to round
            // the decimal portion of the product up to the next integer if the decimal
            // portion is >=0.5.
            //
            //Once again since this is Q15
            // convert to Q0 by shifting:
            CMPAHR_reg_val = (temp*MEP_ScaleFactor+(0x0080<<7))>>15;

            ** If auto-conversion is enabled, the following step is performed
               automatically in hardware and can be skipped
            // Now the lower 8 bits contain the MEP count.
            // Since the MEP count needs to be in the upper 8 bits of
            // the 16 bit CMPAHR register, shift left by 8.
            CMPAHR_reg_val = CMPAHR_reg_val << 8;

            ** If auto-conversion is enabled, the following step is performed
               automatically in hardware and can be skipped
            // Add the offset and rounding
            CMPAHR_reg_val += 0x0080;

            // Write the values to the registers as one 32-bit or two 16-bits
            EPwm1Regs.CMPA.half.CMPA = CMPA_reg_val;
            EPwm1Regs.CMPA.half.CMPAHR = CMPAHR_reg_val;
            */

            // All the above operations may be condensed into
            // the following form:
            // EPWM1 calculations

                for(i=1;i<PWM_CH;i++)
                {
                    CMPA_reg_val = ((long)DutyFine * (*ePWM[i]).TBPRD)>>15;
                    temp = ((long)DutyFine * (*ePWM[i]).TBPRD) ;
                    temp = temp - ((long)CMPA_reg_val<<15);

                   #if (AUTOCONVERT)
                    CMPAHR_reg_val = temp<<1; // convert to Q16
                   #else

                    CMPAHR_reg_val = ((temp*MEP_ScaleFactor)+(0x0080<<7))>>15;
                    CMPAHR_reg_val = CMPAHR_reg_val << 8;
                   #endif

                   // Example for a 32 bit write to CMPA:CMPAHR
                    (*ePWM[i]).CMPA.all = ((long)CMPA_reg_val)<<16 | CMPAHR_reg_val; // loses lower 8-bits

                 }

            }
            else
            {
            // CMPA_reg_val is calculated as a Q0.
            // Since DutyFine is a Q15 number, and the period is Q0
            // the product is Q15. So to store as a Q0, we shift right
            // 15 bits.

                for(i=1;i<PWM_CH;i++)
                {
                 (*ePWM[i]).CMPA.half.CMPA = ((long)DutyFine * (*ePWM[i]).TBPRD>>15);
                }
            }



        // Call the scale factor optimizer lib function SFO()
        // periodically to track for any change due to temp/voltage.
        // This function generates MEP_ScaleFactor by running the
        // MEP calibration module in the HRPWM logic. This scale
        // factor can be used for all HRPWM channels. The SFO()
        // function also updates the HRMSTEP register with the
        // scale factor value.

            status = SFO(); // in background, MEP calibration module continuously updates MEP_ScaleFactor
            if (status == SFO_ERROR) {
                error();   // SFO function returns 2 if an error occurs & # of MEP steps/coarse step
            }              // exceeds maximum of 255.

        } // end DutyFine for loop

    }     // end infinite for loop

}         // end main

//=============================================================
// FUNCTION:    HRPWM_Config
// DESCRIPTION: Configures all ePWM channels and sets up HRPWM
//              on ePWMxA channels
//
// PARAMETERS:  period - desired PWM period in TBCLK counts
// RETURN:      N/A
//=============================================================

void HRPWM_Config(period)
{
Uint16 j;
// ePWM channel register configuration with HRPWM
// ePWMxA toggle low/high with MEP control on Rising edge

   for (j=1;j<PWM_CH;j++)
   {
    (*ePWM[j]).TBCTL.bit.PRDLD = TB_SHADOW;             // set Immediate load
    (*ePWM[j]).TBPRD = period-1;                        // PWM frequency = 1 / period
    (*ePWM[j]).CMPA.half.CMPA = period / 2;             // set duty 50% initially
    (*ePWM[j]).CMPA.half.CMPAHR = (1 << 8);             // initialize HRPWM extension
    (*ePWM[j]).CMPB = period / 2;                       // set duty 50% initially
    (*ePWM[j]).TBPHS.all = 0;
    (*ePWM[j]).TBCTR = 0;

    (*ePWM[j]).TBCTL.bit.CTRMODE = TB_COUNT_UP;
    (*ePWM[j]).TBCTL.bit.PHSEN = TB_DISABLE;
    (*ePWM[j]).TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    (*ePWM[j]).TBCTL.bit.HSPCLKDIV = TB_DIV1;
    (*ePWM[j]).TBCTL.bit.CLKDIV = TB_DIV1;
    (*ePWM[j]).TBCTL.bit.FREE_SOFT = 11;

    (*ePWM[j]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    (*ePWM[j]).CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    (*ePWM[j]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    (*ePWM[j]).CMPCTL.bit.SHDWBMODE = CC_SHADOW;


    (*ePWM[j]).AQCTLA.bit.ZRO = AQ_SET;               // PWM toggle high/low
    (*ePWM[j]).AQCTLA.bit.CAU = AQ_CLEAR;
    (*ePWM[j]).AQCTLB.bit.ZRO = AQ_SET;
    (*ePWM[j]).AQCTLB.bit.CBU = AQ_CLEAR;

    EALLOW;
    (*ePWM[j]).HRCNFG.all = 0x0;
    (*ePWM[j]).HRCNFG.bit.EDGMODE = HR_FEP;          // MEP control on falling edge
    (*ePWM[j]).HRCNFG.bit.CTLMODE = HR_CMP;
    (*ePWM[j]).HRCNFG.bit.HRLOAD  = HR_CTR_ZERO;
    #if (AUTOCONVERT)
    (*ePWM[j]).HRCNFG.bit.AUTOCONV = 1;              // Enable auto-conversion logic
    #endif
    (*ePWM[j]).HRPCTL.bit.HRPE = 0; // Turn off high-resolution period control.
    EDIS;

    }
}


void error (void) {
    ESTOP0;         // Stop here and handle error
}


// No more


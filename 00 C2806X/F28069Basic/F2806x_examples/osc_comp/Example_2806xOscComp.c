// TI File $Revision: /main/2 $ 
// Checkin $Date: January 4, 2011   10:03:20 $
//###########################################################################
//
// FILE:    Example_2806xOscComp.c
//
// TITLE:   Example Internal Oscillator Compensation
//
// ASSUMPTIONS:
//
//    This program requires the F2806x header files.
//
//    This program makes use of variables stored in OTP during factory
//    test on F2806x TMS devices.
//    These OTP locations on pre-TMS devices may not be populated.
//    Ensure that the following memory locations in TI OTP are populated
//    (not 0xFFFF) before use:
//
//    0x3D7E90 to 0x3D7EA4
//
//    As supplied, this project is configured for "boot to SARAM"
//    operation.  The F2806x Boot Mode table is shown below.
//
//    $Boot_Table:
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
//    This program shows how to use the internal oscillator compensation
//    functions in F2806x_OscComp.c. The temperature sensor is sampled
//    and the raw temp sensor value is passed to the oscillator compensation
//    function, which uses this parameter to compensate for
//    frequency drift of the internal oscillator over temperature
//
//
//    Watch Variables:
//      temp
//      SysCtrlRegs.INTOSC1TRIM
//      SysCtrlRegs.INTOSC2TRIM
//
//
//###########################################################################
// $TI Release: 2806x C/C++ Header Files and Peripheral Examples V1.00 $ 
// $Release Date: January 11, 2011 $
//###########################################################################

#include "DSP28x_Project.h"     // DSP28x Headerfile


int16 temp;   //Temperature sensor reading

void main()
{
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2806x_SysCtrl.c file.
       InitSysCtrl();

    // Step 2. Initalize GPIO:
    // Enable XCLOCKOUT to allow monitoring of oscillator 1
       EALLOW;
       GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3; //enable XCLOCKOUT through GPIO mux
       SysCtrlRegs.XCLK.bit.XCLKOUTDIV = 2; //XCLOCKOUT = SYSCLK

    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
       DINT;

    // Initialize PIE control registers to their default state.
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
    // InitPeripherals(); // Not required for this example

    // Configure the ADC:
    // Initialize the ADC
       InitAdc();

       EALLOW;
       AdcRegs.ADCCTL1.bit.TEMPCONV  = 1; //Connect channel A5 internally to the temperature sensor
       AdcRegs.ADCSOC0CTL.bit.CHSEL  = 5; //Set SOC0 channel select to ADCINA5
       AdcRegs.ADCSOC1CTL.bit.CHSEL  = 5; //Set SOC1 channel select to ADCINA5
       AdcRegs.ADCSOC0CTL.bit.ACQPS  = 6; //Set SOC0 acquisition period to 7 ADCCLK
       AdcRegs.ADCSOC1CTL.bit.ACQPS  = 6; //Set SOC1 acquisition period to 7 ADCCLK
       AdcRegs.INTSEL1N2.bit.INT1SEL = 1; //Connect ADCINT1 to EOC1
       AdcRegs.INTSEL1N2.bit.INT1E  =  1; //Enable ADCINT1


    // Note: two channels have been connected to the temp sensor
    // so that the first sample can be discarded to avoid the
    // ADC first sample issue.  See the device errata.

    // Set the flash OTP wait-states to minimum. This is important
    // for the performance of the compensation function.
       FlashRegs.FOTPWAIT.bit.OTPWAIT = 1;

    //Main program loop
    for(;;)
    {
        // Sample temperature sensor. Note: the end application will
        // be responsible for deciding how and when to sample the
        // temperature sensor so that the value can be passed to the
        // compensation function

           //Force start of conversion on SOC0 and SOC1
           AdcRegs.ADCSOCFRC1.all = 0x03;

           //Wait for end of conversion.
           while(AdcRegs.ADCINTFLG.bit.ADCINT1 == 0){}  //Wait for ADCINT1
           AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;        //Clear ADCINT1

           //Get temp sensor sample result from SOC1
           temp = AdcResult.ADCRESULT1;

          //Use temp sensor measurement to perform oscillator compensation even as temperature changes.
            Osc1Comp(temp);
            Osc2Comp(temp); //Also possible to recalibrate osc. 2

        //...other application tasks here...
    }
}




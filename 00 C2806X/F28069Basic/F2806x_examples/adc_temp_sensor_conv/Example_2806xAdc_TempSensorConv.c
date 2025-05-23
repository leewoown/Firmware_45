// TI File $Revision: /main/3 $
// Checkin $Date: January 5, 2011   17:25:55 $
//###########################################################################
//
// FILE:    Example_2806xAdc_TempSensorConv.c
//
// TITLE:   Example ADC Temperature Sensor Conversion to Degrees Celsius/Degrees Kelvin
//
// ASSUMPTIONS:
//
//    This program requires the F2806x header files.
//
//    This program makes use of variables stored in OTP during factory
//    test on F2806x TMS devices only.
//    These OTP locations on pre-TMS devices may not be populated.
//    Ensure that the following memory locations in TI OTP are populated
//    (not 0xFFFF) before use:
//
//    0x3D7E90 to 0x3D7EA4
//
//    As supplied, this project is configured for "boot to SARAM"
//    operation.  The F2806x Boot Mode table is shown below.
//
//    $Boot_Table
//    While an emulator is connected to your device, the TRSTn pin = 1,
//    which sets the device into EMU_BOOT boot mode. In this mode, the
//    peripheral boot modes are as follows:
//
//      Boot Mode:   EMU_KEY        EMU_BMODE
//                   (0xD00)         (0xD01)
//      ---------------------------------------
//      Wait         !=0x55AA        X
//      I/O          0x55AA          0x0000
//      SCI          0x55AA          0x0001
//      Wait         0x55AA          0x0002
//      Get_Mode     0x55AA          0x0003
//      SPI          0x55AA          0x0004
//      I2C          0x55AA          0x0005
//      OTP          0x55AA          0x0006
//      ECANA        0x55AA          0x0007
//      Wait         0x55AA          0x0008
//      SARAM        0x55AA          0x000A   <-- "Boot to SARAM"
//      Flash        0x55AA          0x000B
//      Wait         0x55AA          Other
//
//   Write EMU_KEY to 0xD00 and EMU_BMODE to 0xD01 via the debugger
//   according to the Boot Mode Table above. Build/Load project,
//   Reset the device, and Run example
//
//   $End_Boot_Table
//
// DESCRIPTION:
//
//    This program shows how to convert a raw ADC temperature sensor reading into
//    deg. C or deg. K.
//
//    Watch Variables
//        temp
//        degC
//        degK
//
//###########################################################################
// $TI Release: 2806x C/C++ Header Files and Peripheral Examples V1.00 $
// $Release Date: January 11, 2011 $
//###########################################################################

#include "DSP28x_Project.h"     // DSP28x Headerfile

#define CONV_WAIT 1L //Micro-seconds to wait for ADC conversion. Longer than necessary.

int16 temp; //raw temperature sensor reading
int16 degC; //temperature in deg. C
int16 degK; //temperature in deg. K

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
       AdcRegs.ADCSOC0CTL.bit.ACQPS  = 25; 	//Set SOC0 acquisition period to 26 ADCCLK
       AdcRegs.INTSEL1N2.bit.INT1SEL = 0; 	//Connect ADCINT1 to EOC0
       AdcRegs.INTSEL1N2.bit.INT1E  =  1; //Enable ADCINT1

    // Set the flash OTP wait-states to minimum. This is important
    // for the performance of the temperature conversion function.
       FlashRegs.FOTPWAIT.bit.OTPWAIT = 1;

    //Main program loop - continually sample temperature
    for(;;)
    {
        //Sample the temp sensor...

           //Force start of conversion on SOC0
           AdcRegs.ADCSOCFRC1.all = 0x01;

           //Wait for end of conversion.
           while(AdcRegs.ADCINTFLG.bit.ADCINT1 == 0){}  //Wait for ADCINT1
           AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;        //Clear ADCINT1

           //Get temp sensor sample result from SOC0
           temp = AdcResult.ADCRESULT0;

        //Convert the raw temperature sensor measurement into temperature
        degC = GetTemperatureC(temp);
        degK = GetTemperatureK(temp);
    }
}




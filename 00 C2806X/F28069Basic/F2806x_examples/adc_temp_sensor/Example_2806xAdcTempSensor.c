// TI File $Revision: /main/3 $
// Checkin $Date: January 5, 2011   17:08:18 $
//###########################################################################
//
// FILE:   Example_2806xAdcTempSensor.c
//
// TITLE:  F2806x ADC Temperatrue Sensor Example Program.
//
// ASSUMPTIONS:
//
//   This program requires the F2806x header files.
//
//   Make sure the CPU clock speed is properly defined in
//   F2806x_Examples.h before compiling this example.
//
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
//
// Description:
//
//   This example sets up the PLL in x16/2 mode.
//
//   For 80 MHz devices (default)
//   (assuming a 10Mhz input clock).
//
//   Interrupts are enabled and the ePWM1 is set up to generate a periodic
//   ADC SOC interrupt - ADCINT1. One channel is converted -  ADCINA5, which is internally
//	 connected to the temperature sensor.
//
//   Watch Variables:
//
//         TempSensorVoltage[10] Last 10 ADCRESULT0 values
//         ConversionCount  Current result number 0-9
//         LoopCount        Idle loop counter
//
//
//###########################################################################
// $TI Release: 2806x C/C++ Header Files and Peripheral Examples V1.00 $
// $Release Date: January 11, 2011 $
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

// Prototype statements for functions found within this file.
interrupt void adc_isr(void);

// Global variables used in this example:
Uint16 LoopCount;
Uint16 ConversionCount;
Uint16 TempSensorVoltage[10];


void main()
{

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2806x_SysCtrl.c file.
   InitSysCtrl();


// Step 2. Initialize GPIO:
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
   EALLOW;  // This is needed to write to EALLOW protected register
   PieVectTable.ADCINT1 = &adc_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize the ADC:
// This function is found in F2806x_Adc.c
   InitAdc();  // For this example, init the ADC

// Step 5. Configure ADC to sample the temperature sensor on ADCIN5:
// The output of Piccolo temperature sensor can be internally connected to the ADC through ADCINA5
// via the TEMPCONV bit in the ADCCTL1 register. When this bit is set, any voltage applied to the external
// ADCIN5 pin is ignored.
	EALLOW;
	AdcRegs.ADCCTL1.bit.TEMPCONV 	= 1;	//Connect internal temp sensor to channel ADCINA5.
	EDIS;

// Step 6. Continue configuring ADC to sample the temperature sensor on ADCIN5:
// Since the temperature sensor is connected to ADCIN5, configure the ADC to sample channel ADCIN5
// as well as the ADC SOC trigger and ADCINTs preferred. This example uses EPWM1A to trigger the ADC
// to start a conversion and trips ADCINT1 at the end of the conversion.

// ADC Configuration for Temp Sensor	
	EALLOW;
	AdcRegs.ADCCTL1.bit.INTPULSEPOS	= 1;	//ADCINT1 trips after AdcResults latch
	AdcRegs.INTSEL1N2.bit.INT1E     = 1;	//Enabled ADCINT1
	AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;	//Disable ADCINT1 Continuous mode
	AdcRegs.INTSEL1N2.bit.INT1SEL	= 0;	//setup EOC0 to trigger ADCINT1 to fire
	AdcRegs.ADCSOC0CTL.bit.CHSEL 	= 5;	//set SOC0 channel select to ADCINA5 (which is internally connected to the temperature sensor)
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL 	= 5;	//set SOC0 start trigger on EPWM1A
	AdcRegs.ADCSOC0CTL.bit.ACQPS 	= 25;	//set SOC0 S/H Window to 26 ADC Clock Cycles, (25 ACQPS plus 1)
	EDIS;


// Step 7. User specific code, enable interrupts:

// Enable ADCINT1 in PIE
   PieCtrlRegs.PIEIER1.bit.INTx1 = 1;	// Enable INT 1.1 in the PIE
   IER |= M_INT1; 						// Enable CPU Interrupt 1
   EINT;          						// Enable Global interrupt INTM
   ERTM;          						// Enable Global realtime interrupt DBGM

   LoopCount = 0;
   ConversionCount = 0;


// Assumes ePWM1 clock is already enabled in InitSysCtrl();
   EPwm1Regs.ETSEL.bit.SOCAEN	= 1;		// Enable SOC on A group
   EPwm1Regs.ETSEL.bit.SOCASEL	= 4;		// Select SOC from from CPMA on upcount
   EPwm1Regs.ETPS.bit.SOCAPRD 	= 1;		// Generate pulse on 1st event
   EPwm1Regs.CMPA.half.CMPA 	= 0x0080;	// Set compare A value
   EPwm1Regs.TBPRD 				= 0xFFFF;	// Set period for ePWM1
   EPwm1Regs.TBCTL.bit.CTRMODE 	= 0;		// count up and start

// Wait for ADC interrupt
   for(;;)
   {
      LoopCount++;
   }

}


interrupt void  adc_isr(void)
{

  TempSensorVoltage[ConversionCount] = AdcResult.ADCRESULT0;

  // If 20 conversions have been logged, start over
  if(ConversionCount == 9)
  {
     ConversionCount = 0;
  }
  else ConversionCount++;

  AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;		//Clear ADCINT1 flag reinitialize for next SOC
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

  return;
}



// TI File $Revision: /main/2 $ 
// Checkin $Date: January 4, 2011   10:02:05 $ 
//###########################################################################
//
// FILE:   Example_2806xDMA_Ram_to_Ram.c
//
// TITLE:  F2806x DMA Ram to Ram
// ASSUMPTIONS:
//
//    This program requires the F2806x header files.
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
//
// DESCRIPTION:
//
// Code will perform a block copy from L5 SARAM to L6 SARAM of 1024 words.  Transfer will be started
// by Timer0.  Will use 32-bit datasize to decrease the transfer time.
// Code will end in local_DINTCH1_ISR once the transfer is complete
//
// Watch Variables:
//      DMABuf1
//      DMABuf2
//
//###########################################################################
// $TI Release: 2806x C/C++ Header Files and Peripheral Examples V1.00 $ 
// $Release Date: January 11, 2011 $ 
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File


#define BUF_SIZE   1024  // Sample buffer size




// DMA Defines
#define CH1_TOTAL               DATA_POINTS_PER_CHANNEL
#define CH1_WORDS_PER_BURST     ADC_CHANNELS_TO_CONVERT



#pragma DATA_SECTION(DMABuf1,"DMARAML5");
#pragma DATA_SECTION(DMABuf2,"DMARAML6");

volatile Uint16 DMABuf1[1024];
volatile Uint16 DMABuf2[1024];

volatile Uint16 *DMADest;
volatile Uint16 *DMASource;

interrupt void local_DINTCH1_ISR(void);


void main(void)
{
   Uint16 i;
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
   EALLOW;	// Allow access to EALLOW protected registers
   PieVectTable.DINTCH1= &local_DINTCH1_ISR;
   EDIS;   // Disable access to EALLOW protected registers

   IER = M_INT7 ;	                             //Enable INT7 (7.1 DMA Ch1)
   EnableInterrupts();
   CpuTimer0Regs.TCR.bit.TSS  = 1;               //Stop Timer0 for now


//Step 5. User specific code, enable interrupts:
  // Initialize DMA
	DMAInitialize();

	// Initialize Tables
   for (i=0; i<BUF_SIZE; i++)
   {
     DMABuf1[i] = 0;
	 DMABuf2[i] = i;
   }


// Configure DMA Channel
    DMADest   = &DMABuf1[0];
	DMASource = &DMABuf2[0];
   	DMACH1AddrConfig(DMADest,DMASource);
	DMACH1BurstConfig(31,2,2);         //Will set up to use 32-bit datasize, pointers are based on 16-bit words
	DMACH1TransferConfig(31,2,2);      //so need to increment by 2 to grab the correct location
	DMACH1WrapConfig(0xFFFF,0,0xFFFF,0);
	//Use timer0 to start the x-fer.
	//Since this is a static copy use one shot mode, so only one trigger is needed
	//Also using 32-bit mode to decrease x-fer time
    DMACH1ModeConfig(DMA_TINT0,PERINT_ENABLE,ONESHOT_ENABLE,CONT_DISABLE,SYNC_DISABLE,SYNC_SRC,OVRFLOW_DISABLE,THIRTYTWO_BIT,CHINT_END,CHINT_ENABLE);

	StartDMACH1();


   //Init the timer 0

   CpuTimer0Regs.TIM.half.LSW = 512;    //load low value so we can start the DMA quickly
   CpuTimer0Regs.TCR.bit.SOFT = 1;      //Allow to free run even if halted
   CpuTimer0Regs.TCR.bit.FREE = 1;
   CpuTimer0Regs.TCR.bit.TIE  = 1;      //Enable the timer0 interrupt signal
   CpuTimer0Regs.TCR.bit.TSS  = 0;      //restart the timer 0
   for(;;){}

}

// INT7.1
interrupt void local_DINTCH1_ISR(void)     // DMA Channel 1
{

  // To receive more interrupts from this PIE group, acknowledge this interrupt
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
   asm ("      ESTOP0");
   for(;;);
}




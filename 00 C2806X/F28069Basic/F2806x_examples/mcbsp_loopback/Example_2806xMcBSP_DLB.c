//###########################################################################
//
// FILE:	Example_2806xMCBSP_FFDLB.c
//
// TITLE:	F2806x Device McBSP Digital Loop Back program.
//
// ASSUMPTIONS:
//
//    This program requires the F2806x header files.
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
//          Digital loopback tests for the McBSP peripheral.
//
//          Three different serial word sizes can be tested.
//
//          Before compiling this project:
//          * Select the serial word size (8/16/32) by using
//            the #define statements at the beginning of the code.
//

//
//          This example does not use interrupts.  Instead, a polling
//          method is used to check the receive data.  The incoming
//          data is checked for accuracy.  If an error is found the error()
//          function is called and execution stops.
//
//          This program will execute until terminated by the user.
//
//    Watch Variables:
//          sdata1
//          sdata2
//          rdata1
//          rdata2
//          rdata1_point
//          rdata2_point
//
//###########################################################################
// $TI Release: 2806x C/C++ Header Files and Peripheral Examples V1.00 $ 
// $Release Date: January 11, 2011 $
//###########################################################################


#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

// Choose a word size.  Uncomment one of the following lines
#define WORD_SIZE    8    // Run a loopback test in 8-bit mode
//#define WORD_SIZE 16      // Run a loopback test in 16-bit mode
//#define WORD_SIZE 32        // Run a loopback test in 32-bit mode


// Prototype statements for functions found within this file.
void mcbsp_xmit(int a, int b);
void error(void);

// Global data for this example
Uint16 sdata1 = 0x000;    // Sent Data
Uint16 rdata1 = 0x000;    // Recieved Data

Uint16 sdata2 = 0x000;    // Sent Data
Uint16 rdata2 = 0x000;    // Recieved Data

Uint16 rdata1_point;
Uint16 rdata2_point;


void main(void)
{
   Uint16 datasize;

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2806x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO:
// This example function is found in the F2806x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example
// For this example, only enable the GPIO for McBSP-A
   InitMcbspaGpio();

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
// InitPeripherals();     // Not required for this example

   datasize = WORD_SIZE;
   InitMcbspa();          // Initalize the Mcbsp-A in loopback test mode

// Step 5. User specific code, enable interrupts:

      if(datasize == 8)             // Run a loopback test in 8-bit mode
      {
          InitMcbspa8bit();
          sdata2 = 0x0000;           // value is a don't care for 8-bit mode
          sdata1 = 0x0000;           // 8-bit value to send
		  rdata2_point = 0x0000;     // value is a don't care for 8-bit mode
          rdata1_point = sdata1;
          for(;;)
          {
              mcbsp_xmit(sdata1,sdata2);
              sdata1++;
              sdata1 = sdata1 & 0x00FF;                     // Keep it to 8-bits

              while(McbspaRegs.SPCR1.bit.RRDY == 0 ) { }    // Check for receive
              rdata1 = McbspaRegs.DRR1.all;                 // read DRR1
              if(rdata1 != rdata1_point) error();
              rdata1_point++;
              rdata1_point = rdata1_point & 0x00FF;         // Keep it to 8-bits

               asm("    nop");                                   // Good place for a breakpoint
                                                                 // Check: rdatax_point = sdatax
                                                                 //        rdata1 = sdata1 - 1
          }
       }


      else if(datasize == 16)            // Run a loopback test in 16-bit mode
      {
          InitMcbspa16bit();
          sdata2 = 0x0000;           // value is a don't care for 16-bit mode
          sdata1 = 0x0000;           // 16-bit value to send
		  rdata2_point = 0x0000;     // value is a don't care for 16-bit mode
          rdata1_point = sdata1;
          for(;;)
          {
              mcbsp_xmit(sdata1,sdata2);
              sdata1++;

              while(McbspaRegs.SPCR1.bit.RRDY == 0 ) { }     // Check for receive
              rdata1 = McbspaRegs.DRR1.all;                  // read DRR1
              if(rdata1 != rdata1_point) error();
              rdata1_point++;

               asm("    nop");                                   // Good place for a breakpoint
                                                                 // Check: rdatax_point = sdatax
                                                                 //        rdata1 = sdata1 - 1
          }
       }


      else if(datasize == 32)            // Run a loopback test in 16-bit mode
      {
          InitMcbspa32bit();
          sdata1 = 0x0000;
          sdata2 = 0xFFFF;
          rdata1_point = sdata1;
          rdata2_point = sdata2;
          for(;;)
          {
              mcbsp_xmit(sdata1,sdata2);
              sdata1++;
              sdata2--;

              while(McbspaRegs.SPCR1.bit.RRDY == 0 ) { }  // Check for receive
              rdata2 = McbspaRegs.DRR2.all;
              rdata1 = McbspaRegs.DRR1.all;
              if(rdata1 != rdata1_point) error();
              if(rdata2 != rdata2_point) error();
              rdata1_point++;
              rdata2_point--;

              asm("    nop");                             // Good place for a breakpoint
                                                          // Check: rdatax_point = sdatax
                                                          //        rdata1 = sdata1 - 1
														  //        rdata2 = sdata2 + 1
           }
       }
}



// Some Useful local functions

void error(void)
{
    asm("     ESTOP0");  // test failed!! Stop!
    for (;;);
}

void mcbsp_xmit(int a, int b)
{
    McbspaRegs.DXR2.all=b;
    McbspaRegs.DXR1.all=a;
}

//===========================================================================
// No more.
//===========================================================================



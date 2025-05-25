//###########################################################################
//
// FILE:   Example_2806xMCBSP_FFDLB_int.c
//
// TITLE:  F2806x Device McBSP Digital Loop Back porgram
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
// This program is a McBSP example that uses the internal loopback of
// the peripheral.  Both Rx and Tx interrupts are enabled.
//
// Incrementing values from 0x0000 to 0x00FF are being sent and received.
//
// This pattern is repeated forever.
//
// By default for the McBSP examples, the McBSP sample rate generator (SRG) input
// clock frequency is LSPCLK 80E6/4.
//
//   Watch Variables:
//                sdata
//                rdata
//                rdata_point
//
//###########################################################################
// $TI Release: 2806x C/C++ Header Files and Peripheral Examples V1.00 $ 
// $Release Date: January 11, 2011 $ 
//###########################################################################



#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

// Prototype statements for functions found within this file.

interrupt void Mcbsp_TxINTA_ISR(void);
interrupt void Mcbsp_RxINTA_ISR(void);
void mcbsp_init_dlb(void);
void error(void);

// Global data variables used for this example
Uint16 sdata;    // Sent Data
Uint16 rdata;    // Recieved Data
Uint16 rdata_point; // Keep track of where we
                    // are in the data stream


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
// Setup only the GP I/O only for McBSP-A functionality
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

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;	// Allow access to EALLOW protected registers
   PieVectTable.MRINTA= &Mcbsp_RxINTA_ISR;
   PieVectTable.MXINTA= &Mcbsp_TxINTA_ISR;
   EDIS;   // Disable access to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in F2806x_InitPeripherals.c
// InitPeripherals(); // Not required for this example
   mcbsp_init_dlb();      // For this example, only initialize the Mcbsp


// Step 5. User specific code, enable interrupts:

   sdata = 0;
   rdata_point = sdata;

// Enable interrupts required for this example
   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
   PieCtrlRegs.PIEIER6.bit.INTx5=1;     // Enable PIE Group 6, INT 5
   PieCtrlRegs.PIEIER6.bit.INTx6=1;     // Enable PIE Group 6, INT 6
   IER=0x20;                            // Enable CPU INT6
   EINT;                                // Enable Global Interrupts

// Step 6. IDLE loop. Just sit and loop forever (optional):
   for(;;);

}


// Step 7. Insert all local Interrupt Service Routines (ISRs) and functions here:


void error(void)
{
    asm("     ESTOP0"); // Test failed!! Stop!
    for (;;);
}

void mcbsp_init_dlb()
{

//*************** RESET MCBSP
   McbspaRegs.SPCR2.bit.FRST=0; // Frame Sync generator reset
   McbspaRegs.SPCR2.bit.GRST=0; // Sample Rate generator Reset
   McbspaRegs.SPCR2.bit.XRST=0; // Transmitter reset
   McbspaRegs.SPCR1.bit.RRST=0; // Receiver reset

//*************** Initialize McBSP Registers
// McBSP register settings for Digital loop back
   McbspaRegs.SPCR2.all=0x0000; // XRST =0
   McbspaRegs.SPCR1.all=0x8000; // RRST =0, DLB enabled
   McbspaRegs.RCR2.all=0x0001;  // RDATDLY = 1
   McbspaRegs.RCR1.all=0x0;
   McbspaRegs.XCR2.all=0x0001;  // XDATDLY = 1
   McbspaRegs.XCR1.all=0x0;

   McbspaRegs.SRGR2.all=0x3140;
   McbspaRegs.SRGR1.all=0x010f;
   McbspaRegs.MCR2.all=0x0;
   McbspaRegs.MCR1.all=0x0;
   McbspaRegs.PCR.all=0x0A00;

   McbspaRegs.MFFINT.bit.XINT = 1; // Enable Transmit Interrupts
   McbspaRegs.MFFINT.bit.RINT = 1; // Enable Receive Interrupts

//************* Enable Sample rate generator
   McbspaRegs.SPCR2.bit.GRST=1;
   delay_loop();                   // Wait at least 2 SRG clock cycles

//************ Enable TX/RX unit
   McbspaRegs.SPCR2.bit.XRST=1;
   McbspaRegs.SPCR1.bit.RRST=1;

//************ Frame Sync generator reset
   McbspaRegs.SPCR2.bit.FRST=1;
}

interrupt void Mcbsp_TxINTA_ISR(void)
{
    McbspaRegs.DXR1.all= sdata;
    sdata = (sdata+1)& 0x00FF ;
    // To receive more interrupts from this PIE group, acknowledge this interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

interrupt void Mcbsp_RxINTA_ISR(void)
{
    rdata=McbspaRegs.DRR1.all;
    if (rdata != ( (rdata_point) & 0x00FF) ) error();
    rdata_point = (rdata_point+1) & 0x00FF;
    // To receive more interrupts from this PIE group, acknowledge this interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

//===========================================================================
// No more.
//===========================================================================


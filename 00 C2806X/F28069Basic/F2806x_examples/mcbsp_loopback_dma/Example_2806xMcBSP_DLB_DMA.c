//###########################################################################
//
// FILE:   Example_2806xMCBSP_DLB_DMA.c
//
// TITLE:  F2806x Device McBSP Digital Loop Back  with DMA program
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
// the peripheral and utilizes the DMA to transfer data from one buffer
// to the McBSP, and then from the McBSP to another buffer.
//
// Initially, sdata[] is filled with values from 0x0000- 0x007F.  The DMA
// moves the values in sdata[] one by one to the DXRx registers of the McBSP.
// These values are transmitted and subsequently received by the McBSP.
// Then, the DMA moves each data value to rdata[] as it is received by the McBSP.
//
// Three different serial word sizes can be tested.
//
//          Before compiling this project:
//          * Select the serial word size (8/16/32) by using
//            the #define statements at the beginning of the code.
//
// The program loops forever after all values have been transferred to sdata.
// It is up to the user to stop the program.
//
//
// By default for the McBSP examples, the McBSP sample rate generator (SRG) input
// clock frequency is LSPCLK 80E6/4 assuming SYSCLKOUT = 80 MHz.
//
//   Watch Variables:
//                sdata
//                rdata
//
//
//###########################################################################
// $TI Release: 2806x C/C++ Header Files and Peripheral Examples V1.00 $ 
// $Release Date: January 11, 2011 $ 
//###########################################################################


#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

// Choose a word size.  Uncomment one of the following lines
#define WORD_SIZE  8      // Run a loopback test in 8-bit mode
//#define WORD_SIZE 16      // Run a loopback test in 16-bit mode
//#define WORD_SIZE 32      // Run a loopback test in 32-bit mode

// Prototype statements for functions found within this file.

interrupt void local_D_INTCH1_ISR(void);
interrupt void local_D_INTCH2_ISR(void);
void mcbsp_init_dlb(void);
void init_dma(void);
void init_dma_32(void);
void start_dma(void);
void error(void);


// Place sdata and rdata buffers in DMA-accessible RAM (L4 for this example)
#pragma DATA_SECTION(sdata, "DMARAML5")
#pragma DATA_SECTION(rdata, "DMARAML5")
Uint16 sdata[128];    // Sent Data
Uint16 rdata[128];    // Recieved Data
Uint16 data_size;     // Word Length variable

void main(void)
{
   Uint16 i;
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
   PieVectTable.DINTCH1= &local_D_INTCH1_ISR;
   PieVectTable.DINTCH2= &local_D_INTCH2_ISR;
   EDIS;   // Disable access to EALLOW protected registers


// Step 4. Initialize all the Device Peripherals:
// This function is found in F2806x_InitPeripherals.c
// InitPeripherals(); // Not required for this example

// Step 5. User specific code, enable interrupts:
   data_size = WORD_SIZE;
   for (i=0; i<128; i++)
   {
       sdata[i] = i;      // Fill sdata with values between 0 and 0x007F
	   rdata[i] = 0;      // Initialize rdata to all 0x0000.
   }
   if (data_size == 32)
   {
     init_dma_32();     // DMA Initialization for 32-bit transfers
   }  else
   {
     init_dma();       // 1. When using DMA, initialize DMA with peripheral interrupts first.
   }
   start_dma();
   mcbsp_init_dlb();   // 2.  Then initialize and release peripheral (McBSP) from Reset.


// Enable interrupts required for this example
   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
   PieCtrlRegs.PIEIER6.bit.INTx5=1;     // Enable PIE Group 6, INT 5
   PieCtrlRegs.PIEIER6.bit.INTx6=1;     // Enable PIE Group 6, INT 6
   PieCtrlRegs.PIEIER7.bit.INTx1 = 1;	// Enable PIE Group 7, INT 1 (DMA CH1)
   PieCtrlRegs.PIEIER7.bit.INTx2 = 1;	// Enable PIE Group 7, INT 2 (DMA CH2)

   IER=0x60;                            // Enable CPU INT groups 6 and 7
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

    McbspaRegs.SPCR2.all=0x0000;		// Reset FS generator, sample rate generator & transmitter
	McbspaRegs.SPCR1.all=0x0000;		// Reset Receiver, Right justify word
	McbspaRegs.SPCR1.bit.DLB = 1;       // Enable DLB mode. Comment out for non-DLB mode.

	McbspaRegs.MFFINT.all=0x0;			// Disable all interrupts

    McbspaRegs.RCR2.all=0x0;			// Single-phase frame, 1 word/frame, No companding	(Receive)
    McbspaRegs.RCR1.all=0x0;

    McbspaRegs.XCR2.all=0x0;			// Single-phase frame, 1 word/frame, No companding	(Transmit)
    McbspaRegs.XCR1.all=0x0;

    McbspaRegs.SRGR2.bit.CLKSM = 1;		// CLKSM=1 (If SCLKME=0, i/p clock to SRG is LSPCLK)
	McbspaRegs.SRGR2.bit.FPER = 31;		// FPER = 32 CLKG periods

    McbspaRegs.SRGR1.bit.FWID = 0;      // Frame Width = 1 CLKG period
    McbspaRegs.SRGR1.bit.CLKGDV = 0;	// CLKG frequency = LSPCLK/(CLKGDV+1)

    McbspaRegs.PCR.bit.FSXM = 1;		// FSX generated internally, FSR derived from an external source
	McbspaRegs.PCR.bit.CLKXM = 1;		// CLKX generated internally, CLKR derived from an external source



    //*************** Initialize McBSP Data Length
    if(data_size == 8)             // Run a loopback test in 8-bit mode
    {
      InitMcbspa8bit();
    }
    if(data_size == 16)            // Run a loopback test in 16-bit mode
    {
      InitMcbspa16bit();
    }
    if(data_size == 32)            // Run a loopback test in 32-bit mode
    {
      InitMcbspa32bit();
    }

    //************* Enable Sample rate generator
    McbspaRegs.SPCR2.bit.GRST=1; // Enable the sample rate generator
    delay_loop();                // Wait at least 2 SRG clock cycles
    McbspaRegs.SPCR2.bit.XRST=1; // Release TX from Reset
    McbspaRegs.SPCR1.bit.RRST=1; // Release RX from Reset
    McbspaRegs.SPCR2.bit.FRST=1; // Frame Sync Generator reset
}

// DMA Initialization for data size <= 16-bit

void init_dma()
{
  EALLOW;
  DmaRegs.DMACTRL.bit.HARDRESET = 1;
  asm(" NOP");						   // Only 1 NOP needed per Design
  DmaRegs.CH1.MODE.bit.CHINTE = 0;
  // Channel 1, McBSPA transmit
  DmaRegs.CH1.BURST_SIZE.all = 0;		// 1 word/burst
  DmaRegs.CH1.SRC_BURST_STEP = 0;		// no effect when using 1 word/burst
  DmaRegs.CH1.DST_BURST_STEP = 0;		// no effect when using 1 word/burst
  DmaRegs.CH1.TRANSFER_SIZE = 127;		// Interrupt every frame (127 bursts/transfer)
  DmaRegs.CH1.SRC_TRANSFER_STEP = 1;	// Move to next word in buffer after each word in a burst
  DmaRegs.CH1.DST_TRANSFER_STEP = 0;	// Don't move destination address
  DmaRegs.CH1.SRC_ADDR_SHADOW = (Uint32) &sdata[0];			// Start address = buffer
  DmaRegs.CH1.SRC_BEG_ADDR_SHADOW = (Uint32) &sdata[0];		// Not needed unless using wrap function
  DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32) &McbspaRegs.DXR1.all;		// Start address = McBSPA DXR
  DmaRegs.CH1.DST_BEG_ADDR_SHADOW = (Uint32) &McbspaRegs.DXR1.all;	// Not needed unless using wrap function
  DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;	// Clear peripheral interrupt event flag
  DmaRegs.CH1.CONTROL.bit.SYNCCLR = 1;		// Clear sync flag
  DmaRegs.CH1.CONTROL.bit.ERRCLR = 1;	// Clear sync error flag
  DmaRegs.CH1.DST_WRAP_SIZE = 0xFFFF;		// Put to maximum - don't want destination wrap
  DmaRegs.CH1.SRC_WRAP_SIZE = 0xFFFF;		// Put to maximum - don't want source wrap
  DmaRegs.CH1.MODE.bit.SYNCE = 0;         		// No sync signal
  DmaRegs.CH1.MODE.bit.SYNCSEL = 0;       		// No sync signal
  DmaRegs.CH1.MODE.bit.CHINTE = 1;			// Enable channel interrupt
  DmaRegs.CH1.MODE.bit.CHINTMODE = 1;		// Interrupt at end of transfer
  DmaRegs.CH1.MODE.bit.PERINTE = 1;			// Enable peripheral interrupt event
  DmaRegs.CH1.MODE.bit.PERINTSEL = DMA_MXEVTA;		// Peripheral interrupt select = McBSP MXSYNCA
  DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;  		// Clear any spurious interrupt flags

  // Channel 2, McBSPA Receive
  DmaRegs.CH2.MODE.bit.CHINTE = 0;
  DmaRegs.CH2.BURST_SIZE.all = 0;		// 1 word/burst
  DmaRegs.CH2.SRC_BURST_STEP = 0;		// no effect when using 1 word/burst
  DmaRegs.CH2.DST_BURST_STEP = 0;		// no effect when using 1 word/burst
  DmaRegs.CH2.TRANSFER_SIZE = 127;		// Interrupt every 127 bursts/transfer
  DmaRegs.CH2.SRC_TRANSFER_STEP = 0;	// Don't move source address
  DmaRegs.CH2.DST_TRANSFER_STEP = 1;	// Move to next word in buffer after each word in a burst
  DmaRegs.CH2.SRC_ADDR_SHADOW = (Uint32) &McbspaRegs.DRR1.all;			// Start address = McBSPA DRR
  DmaRegs.CH2.SRC_BEG_ADDR_SHADOW = (Uint32) &McbspaRegs.DRR1.all;		// Not needed unless using wrap function
  DmaRegs.CH2.DST_ADDR_SHADOW = (Uint32) &rdata[0];		// Start address = Receive buffer (for McBSP-A)
  DmaRegs.CH2.DST_BEG_ADDR_SHADOW = (Uint32) &rdata[0];	// Not needed unless using wrap function
  DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1;	// Clear peripheral interrupt event flag
  DmaRegs.CH2.CONTROL.bit.SYNCCLR = 1;		// Clear sync flag
  DmaRegs.CH2.CONTROL.bit.ERRCLR = 1;	// Clear sync error flag
  DmaRegs.CH2.DST_WRAP_SIZE = 0xFFFF;		// Put to maximum - don't want destination wrap
  DmaRegs.CH2.SRC_WRAP_SIZE = 0xFFFF;		// Put to maximum - don't want source wrap
  DmaRegs.CH2.MODE.bit.CHINTE = 1;			// Enable channel interrupt
  DmaRegs.CH2.MODE.bit.CHINTMODE = 1;		// Interrupt at end of transfer
  DmaRegs.CH2.MODE.bit.PERINTE = 1;			// Enable peripheral interrupt event
  DmaRegs.CH2.MODE.bit.PERINTSEL = DMA_MREVTA;  // Peripheral interrupt select = McBSP MRSYNCA
  DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1;  		// Clear any spurious interrupt flags
  EDIS;
}

// DMA Initialization for data size > 16-bit and <= 32-bit.

void init_dma_32()
{
  EALLOW;
  DmaRegs.DMACTRL.bit.HARDRESET = 1;
  asm(" NOP");						   // Only 1 NOP needed per Design

  // Channel 1, McBSPA transmit
  DmaRegs.CH1.BURST_SIZE.all = 1;		// 2 word/burst
  DmaRegs.CH1.SRC_BURST_STEP = 1;		// increment 1 16-bit addr. btwn words
  DmaRegs.CH1.DST_BURST_STEP = 1;		// increment 1 16-bit addr. btwn words
  DmaRegs.CH1.TRANSFER_SIZE = 63;		// Interrupt every 63 bursts/transfer
  DmaRegs.CH1.SRC_TRANSFER_STEP = 1;	// Move to next word in buffer after each word in a burst
  DmaRegs.CH1.DST_TRANSFER_STEP = 0xFFFF;	// Go back to DXR2
  DmaRegs.CH1.SRC_ADDR_SHADOW = (Uint32) &sdata[0];	// Start address = buffer
  DmaRegs.CH1.SRC_BEG_ADDR_SHADOW = (Uint32) &sdata[0];		    // Not needed unless using wrap function
  DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32) &McbspaRegs.DXR2.all;		// Start address = McBSPA DXR2
  DmaRegs.CH1.DST_BEG_ADDR_SHADOW = (Uint32) &McbspaRegs.DXR2.all;	// Not needed unless using wrap function
  DmaRegs.CH1.CONTROL.bit.SYNCCLR = 1;		// Clear sync flag
  DmaRegs.CH1.CONTROL.bit.ERRCLR = 1;	// Clear sync error flag
  DmaRegs.CH1.DST_WRAP_SIZE = 0xFFFF;		// Put to maximum - don't want destination wrap
  DmaRegs.CH1.SRC_WRAP_SIZE = 0xFFFF;		// Put to maximum - don't want source wrap
  DmaRegs.CH1.MODE.bit.SYNCE = 0;         		// No sync signal
  DmaRegs.CH1.MODE.bit.SYNCSEL = 0;       		// No sync signal
  DmaRegs.CH1.MODE.bit.CHINTE = 1;			// Enable channel interrupt
  DmaRegs.CH1.MODE.bit.CHINTMODE = 1;		// Interrupt at end of transfer
  DmaRegs.CH1.MODE.bit.PERINTE = 1;			// Enable peripheral interrupt event
  DmaRegs.CH1.MODE.bit.PERINTSEL = DMA_MXEVTA;		// Peripheral interrupt select = McBSP MXSYNCA
  DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;  		// Clear any spurious interrupt flags

  // Channel 2, McBSPA Receive
  DmaRegs.CH2.BURST_SIZE.all = 1;		// 2 words/burst
  DmaRegs.CH2.SRC_BURST_STEP = 1;		// Increment 1 16-bit addr. btwn words
  DmaRegs.CH2.DST_BURST_STEP = 1;	    // Increment 1 16-bit addr. btwn words
  DmaRegs.CH2.TRANSFER_SIZE = 63;		// Interrupt every 63 bursts/transfer
  DmaRegs.CH2.SRC_TRANSFER_STEP = 0xFFFF;	// Decrement  back to DRR2
  DmaRegs.CH2.DST_TRANSFER_STEP = 1;	// Move to next word in buffer after each word in a burst
  DmaRegs.CH2.SRC_ADDR_SHADOW = (Uint32) &McbspaRegs.DRR2.all;			// Start address = McBSPA DRR
  DmaRegs.CH2.SRC_BEG_ADDR_SHADOW = (Uint32) &McbspaRegs.DRR2.all;		// Not needed unless using wrap function
  DmaRegs.CH2.DST_ADDR_SHADOW = (Uint32) &rdata[0];		// Start address = Receive buffer (for McBSP-A)
  DmaRegs.CH2.DST_BEG_ADDR_SHADOW = (Uint32) &rdata[0];	// Not needed unless using wrap function
  DmaRegs.CH2.CONTROL.bit.SYNCCLR = 1;		// Clear sync flag
  DmaRegs.CH2.CONTROL.bit.ERRCLR = 1;	// Clear sync error flag
  DmaRegs.CH2.DST_WRAP_SIZE = 0xFFFF;		// Put to maximum - don't want destination wrap
  DmaRegs.CH2.SRC_WRAP_SIZE = 0xFFFF;		// Put to maximum - don't want source wrap
  DmaRegs.CH2.MODE.bit.CHINTE = 1;			// Enable channel interrupt
  DmaRegs.CH2.MODE.bit.CHINTMODE = 1;		// Interrupt at end of transfer
  DmaRegs.CH2.MODE.bit.PERINTE = 1;			// Enable peripheral interrupt event
  DmaRegs.CH2.MODE.bit.PERINTSEL = DMA_MREVTA;	// Peripheral interrupt select = McBSP MRSYNCA
  DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1;  		// Clear any spurious interrupt flags
  EDIS;
}
void start_dma (void)
{
  EALLOW;
  DmaRegs.CH1.CONTROL.bit.RUN = 1;	         // Start DMA Transmit from McBSP-A
  DmaRegs.CH2.CONTROL.bit.RUN = 1;           // Start DMA Receive from McBSP-A

  EDIS;
}
// INT7.1
interrupt void local_D_INTCH1_ISR(void)		// DMA Ch1
{
   	EALLOW;									// NEED TO EXECUTE EALLOW INSIDE ISR !!!
	DmaRegs.CH1.CONTROL.bit.RUN=0;		    // Re-enable DMA CH1. Should be done every transfer
   	PieCtrlRegs.PIEACK.all = PIEACK_GROUP7; // To receive more interrupts from this PIE group, acknowledge this interrupt

    EDIS;
	return;
}

// INT7.2
interrupt void local_D_INTCH2_ISR(void)		// DMA Ch2
{
    Uint16 i;
    EALLOW;									// NEED TO EXECUTE EALLOW INSIDE ISR !!!
	DmaRegs.CH2.CONTROL.bit.RUN = 0;		// Re-enable DMA CH2. Should be done every transfer
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7; // To receive more interrupts from this PIE group, acknowledge this interrupt
    for (i=0; i<128; i++)
		{
		if(data_size == 8)
        {
          if( (rdata[i]&0x00FF) !=(sdata[i]&0x00FF)) error( ); // Check for correct received data
        }
        else if (data_size == 16)
		{
		  if (rdata[i] != sdata[i])  error();  // STOP if there is an error !!
        }
		else if (data_size == 32)
		{
		  if ((rdata[i])!=(sdata[i])) error ();
		}
   }
	 EDIS;
	 return;

}

//===========================================================================
// No more.
//===========================================================================



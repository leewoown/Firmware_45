// TI File $Revision: /main/2 $ 
// Checkin $Date: January 4, 2011   10:02:19 $ 
//###########################################################################
//
// FILE:    Example_2806xEpwmDeadBand.c
//
// TITLE:   Check PWM deadband generation
//
// ASSUMPTIONS:
//
//    This program requires the F2806x header files.  
//
//    Monitor ePWM1 - ePWM3 on an Oscilloscope as described
//    below.
//
//       EPWM1A is on GPIO0
//       EPWM1B is on GPIO1
//
//       EPWM2A is on GPIO2
//       EPWM2B is on GPIO3
//
//       EPWM3A is on GPIO4
//       EPWM3B is on GPIO5
//
//    As supplied, this project is configured for "boot to SARAM" 
//    operation.  The F2806x Boot Mode table is shown below.  
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
//    This example configures ePWM1, ePWM2 and ePWM3 for:
//    - Count up/down
//    - Deadband
// 
//    3 Examples are included:
//    * ePWM1: Active low PWMs
//    * ePWM2: Active low complementary PWMs
//    * ePWM3: Active high complementary PWMs
//
//    Each ePWM is configured to interrupt on the 3rd zero event
//    when this happens the deadband is modified such that
//    0 <= DB <= DB_MAX.  That is, the deadband will move up and
//    down between 0 and the maximum value. 
//
//
//    View the EPWM1A/B, EPWM2A/B and EPWM3A/B waveforms 
//    via an oscilloscope
//
//
//###########################################################################
// $TI Release: 2806x C/C++ Header Files and Peripheral Examples V1.00 $ 
// $Release Date: January 11, 2011 $ 
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

// Prototype statements for functions found within this file.
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
interrupt void epwm1_isr(void);
interrupt void epwm2_isr(void);
interrupt void epwm3_isr(void);

// Global variables used in this example
Uint32  EPwm1TimerIntCount;
Uint32  EPwm2TimerIntCount;
Uint32  EPwm3TimerIntCount;
Uint16  EPwm1_DB_Direction;
Uint16  EPwm2_DB_Direction;
Uint16  EPwm3_DB_Direction;

// Maximum Dead Band values
#define EPWM1_MAX_DB   0x03FF
#define EPWM2_MAX_DB   0x03FF
#define EPWM3_MAX_DB   0x03FF

#define EPWM1_MIN_DB   0
#define EPWM2_MIN_DB   0
#define EPWM3_MIN_DB   0

// To keep track of which way the Dead Band is moving
#define DB_UP   1
#define DB_DOWN 0

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

// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
// These functions are in the F2806x_EPwm.c file
   InitEPwm1Gpio();
   InitEPwm2Gpio();
   InitEPwm3Gpio();
   
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
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.EPWM1_INT = &epwm1_isr;
   PieVectTable.EPWM2_INT = &epwm2_isr;
   PieVectTable.EPWM3_INT = &epwm3_isr;      
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in F2806x_InitPeripherals.c
// InitPeripherals();  // Not required for this example

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
   EDIS;

   InitEPwm1Example();    
   InitEPwm2Example();
   InitEPwm3Example();

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;
   
// Step 5. User specific code, enable interrupts
// Initalize counters:   
   EPwm1TimerIntCount = 0;
   EPwm2TimerIntCount = 0;
   EPwm3TimerIntCount = 0;
   
// Enable CPU INT3 which is connected to EPWM1-3 INT:
   IER |= M_INT3;

// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx3 = 1;

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

// Step 6. IDLE loop. Just sit and loop forever (optional):
   for(;;)
   {
       asm("          NOP");
   }

} 

interrupt void epwm1_isr(void)
{
   if(EPwm1_DB_Direction == DB_UP)
   {
       if(EPwm1Regs.DBFED < EPWM1_MAX_DB)
       {
          EPwm1Regs.DBFED++;
          EPwm1Regs.DBRED++;
       }
       else
       {
          EPwm1_DB_Direction = DB_DOWN;
          EPwm1Regs.DBFED--;
          EPwm1Regs.DBRED--;
       }
   }
   else
   {
       if(EPwm1Regs.DBFED == EPWM1_MIN_DB)
       {
          EPwm1_DB_Direction = DB_UP;
          EPwm1Regs.DBFED++;
          EPwm1Regs.DBRED++;
       }
       else
       {
          EPwm1Regs.DBFED--;
          EPwm1Regs.DBRED--;
       }
   }
   EPwm1TimerIntCount++;

   // Clear INT flag for this timer
   EPwm1Regs.ETCLR.bit.INT = 1;
   
   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}

interrupt void epwm2_isr(void)
{

   if(EPwm2_DB_Direction == DB_UP)
   {
       if(EPwm2Regs.DBFED < EPWM2_MAX_DB)
       {
          EPwm2Regs.DBFED++;
          EPwm2Regs.DBRED++;
       }
       else
       {
          EPwm2_DB_Direction = DB_DOWN;
          EPwm2Regs.DBFED--;
          EPwm2Regs.DBRED--;
       }
   }
   else
   {
       if(EPwm2Regs.DBFED == EPWM2_MIN_DB)
       {
          EPwm2_DB_Direction = DB_UP;
          EPwm2Regs.DBFED++;
          EPwm2Regs.DBRED++;
       }
       else
       {
          EPwm2Regs.DBFED--;
          EPwm2Regs.DBRED--;
       }
   }

   EPwm2TimerIntCount++;

   // Clear INT flag for this timer
   EPwm2Regs.ETCLR.bit.INT = 1;
   
   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}

interrupt void epwm3_isr(void)
{
   if(EPwm3_DB_Direction == DB_UP)
   {
       if(EPwm3Regs.DBFED < EPWM3_MAX_DB)
       {
          EPwm3Regs.DBFED++;
          EPwm3Regs.DBRED++;
       }
       else
       {
          EPwm3_DB_Direction = DB_DOWN;
          EPwm3Regs.DBFED--;
          EPwm3Regs.DBRED--;
       }
   }
   else
   {
       if(EPwm3Regs.DBFED == EPWM3_MIN_DB)
       {
          EPwm3_DB_Direction = DB_UP;
          EPwm3Regs.DBFED++;
          EPwm3Regs.DBRED++;
       }
       else
       {
          EPwm3Regs.DBFED--;
          EPwm3Regs.DBRED--;
       }
   }

   EPwm3TimerIntCount++;

   // Clear INT flag for this timer
   EPwm3Regs.ETCLR.bit.INT = 1;
   
   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}

void InitEPwm1Example()
{
   
   EPwm1Regs.TBPRD = 6000;                        // Set timer period
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                      // Clear counter

   // Setup TBCLK
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV4;

   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   // Setup compare 
   EPwm1Regs.CMPA.half.CMPA = 3000;
   
   // Set actions
   EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on Zero
   EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
   

   EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM1A on Zero
   EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;

   // Active Low PWMs - Setup Deadband
   EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_LO;
   EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm1Regs.DBRED = EPWM1_MIN_DB;
   EPwm1Regs.DBFED = EPWM1_MIN_DB;
   EPwm1_DB_Direction = DB_UP;
   
   // Interrupt where we will change the Deadband
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event   

   
}

void InitEPwm2Example()
{

   EPwm2Regs.TBPRD = 6000;                        // Set timer period
   EPwm2Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                      // Clear counter
   
   // Setup TBCLK
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV4;          // Slow just to observe on the scope

   // Setup compare 
   EPwm2Regs.CMPA.half.CMPA = 3000;

   // Set actions
   EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM2A on Zero
   EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
   

   EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;           // Set PWM2A on Zero
   EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;

   // Active Low complementary PWMs - setup the deadband
   EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
   EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm2Regs.DBRED = EPWM2_MIN_DB;
   EPwm2Regs.DBFED = EPWM2_MIN_DB;
   EPwm2_DB_Direction = DB_UP;

   // Interrupt where we will modify the deadband
   EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on Zero event
   EPwm2Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
   EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;            // Generate INT on 3rd event   
   
}

void InitEPwm3Example()
{

   EPwm3Regs.TBPRD = 6000;                         // Set timer period
   EPwm3Regs.TBPHS.half.TBPHS = 0x0000;            // Phase is 0
   EPwm3Regs.TBCTR = 0x0000;                       // Clear counter
   
   
   // Setup TBCLK
   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
   EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV4;          // Slow so we can observe on the scope

   // Setup compare 
   EPwm3Regs.CMPA.half.CMPA = 3000;

   // Set actions
   EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM3A on Zero
   EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
   

   EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM3A on Zero
   EPwm3Regs.AQCTLB.bit.CAD = AQ_SET;

   // Active high complementary PWMs - Setup the deadband
   EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
   EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm3Regs.DBRED = EPWM3_MIN_DB;
   EPwm3Regs.DBFED = EPWM3_MIN_DB;
   EPwm3_DB_Direction = DB_UP;
   
   // Interrupt where we will change the deadband
   EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
   EPwm3Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
   EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;             // Generate INT on 3rd event   
   
}

//===========================================================================
// No more.
//===========================================================================

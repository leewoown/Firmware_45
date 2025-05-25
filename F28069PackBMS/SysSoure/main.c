

/**
 * main.c
 */

#include "DSP28x_Project.h"
#include "parameter.h"
#include "SysVariable.h"
#include "ProtectRelay.h"
#include "BATAlgorithm.h"
#include "BATCalc.h"
#include "SysSpiCan.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

/*
 *
 */
void InitGpio(void);

void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);

/*
 *
 */
void InitECanaGpio(void);
void InitECana(void);
void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3);


void InitSpiGpio();
//void InitSpiBATIC(void);
void InitSpiCAN(void);
void SPI_Write(unsigned int WRData);
unsigned int SPI_Read(void);
float MeasureSPISpeedHandle(Uint16 testLen);

void TestCANSPIWriteBytesHandle(void);
void CANSPIReadBytesHandle (Uint16 cmd, Uint16 addr, char RxBuf[], Uint16 len);
void CANSPIWriteBytesHandle(Uint16 cmd, Uint16 addr, char TxBuf[], Uint16 len);

void MCP2515ResetHandle(void);
void MCP2515SetCNFHandle(char cnf1, char cnf2, char cnf3);
void MCP2515SetNormalModeHandle(void);

void MCP2515InitHandle(CANBReg *p);




//void SPI_Write(unsigned int WRData);
//unsigned int SPI_Read(void);
//void BAT_InitSPI(void);
//void SPI_BATWrite(unsigned int WRData);

/*
 *
 */
void SysTimerINIT(SystemReg *s);
void SysVarINIT(SystemReg *s);
void CANRegVarINIT(CANAReg *P);
void ModuleInit(ModulemReg *P);
void DigitalInput(SystemReg *sys);
void DigitalOutput(SystemReg *sys);
/*
 *
 */
void ProtectRlySateCheck(PrtectRelayReg *P);
void ProtectRlyVarINIT(PrtectRelayReg *P);
void ProtectRlyOnInit(PrtectRelayReg *P);
void ProtectRlyOnHandle(PrtectRelayReg *P);
void ProtectRlyOffInit(PrtectRelayReg *P);
void ProtectRlyOffHandle(PrtectRelayReg *P);
void ProtectRlyEMSHandle(PrtectRelayReg *P);


void CalKokam100AhRegsInit(SocReg *P);
void CalKokam100AhSocInit(SocReg *P);
void Calkokam100AhSocHandle(SocReg *P);

/*
 *
 */
void SysCurrentHandle(SystemReg *s);

/*
 *
 */
void SysCommErrHandle(SystemReg *P);

/*
 *
 */
extern void BatCalcRegsInit(BatCalcReg *P);
extern void BatCalcVoltHandle(BatCalcReg *P);
extern void BatCalcTempsHandle(BatCalcReg *P);

/*
 *
 */
int float32ToInt(float32 Vaule, Uint32 Num);
/*
 *
 */

/*
 *
 */

/*
 *
 */
//int LTC6804_read_cmd(char address, short command, char data[], int len);
//int LTC6804_write_cmd(char address, short command, char data[], int len);
//void init_PEC15_Table(void);
//unsigned short pec15(char *data, int len);
//int SlaveBMSIint(SlaveReg *s);
//void SlaveVoltagHandler(SlaveReg *s);

/*
 *  인터럽트 함수 선언
 */
interrupt void cpu_timer0_isr(void);
interrupt void ISR_CANRXINTA(void);
//interrupt void cpu_timer2_isr(void);

SystemReg       SysRegs;
ModulemReg      ModRegs;
PrtectRelayReg  PrtectRelayRegs;
CANAReg         CANARegs;
CANBReg         CANBRegs;
SocReg          Kam100AHSocRegs;
BatCalcReg      BatCalcRegs;
float32         NCMsocTestVoltAGV =3.210;

extern unsigned int   CANTXFALAG=0;
extern unsigned int   CANTXFALAG1=0;
extern unsigned int   CANTXFALAG12=0;
//extern unsigned int    CellVoltUnBalaneFaulCount=0;
void main(void)
{
//    struct ECAN_REGS ECanaShadow;
    InitSysCtrl();
    /*
     * To check the clock status of the C2000 in operation
     */
  //  GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3; //enable XCLOCKOUT through GPIO mux
  //  SysCtrlRegs.XCLK.bit.XCLKOUTDIV = 0; //XCLOCKOUT = 1/2* SYSCLK

// Step 2. Initalize GPIO:
// This example function is found in the DSP2803x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// For this example use the following configuration:
// Step 3. Clear all interrupts and initialize PIE vector table:
    DINT;
// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2803x_PieCtrl.c file.
    InitPieCtrl();
// Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    EALLOW;  // This is needed to write to EALLOW protected registers

    /*
     *  인터럽트 함수 선언
     */
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.ECAN0INTA  = &ISR_CANRXINTA;
//    PieVectTable.TINT2 = &cpu_timer2_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers
    InitGpio();
   // GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3; //enable XCLOCKOUT through GPIO mux
  //  SysCtrlRegs.XCLK.bit.XCLKOUTDIV = 2; //XCLOCKOUT = SYSCLK
    InitSpiGpio();
    InitSpiCAN();
    // TEST
    InitECanGpio();
    InitECan();

    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    InitFlash();

    ConfigCpuTimer(&CpuTimer0, 60, 1000);
    CpuTimer0Regs.PRD.all = 80000;// 90000 is 1msec
    //   ConfigCpuTimer(&CpuTimer1, 80, 1000000);
    //   ConfigCpuTimer(&CpuTimer2, 80, 1000000);
    CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
    //  CpuTimer1Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
    //  CpuTimer2Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
//    InitAdc();
//    AdcOffsetSelfCal();
    EALLOW;
    EDIS;    // This is needed to disable write to EALLOW protected registers
    IER |= M_INT1;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT9;//test
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;      // Enable ECAN-A interrupt of PIE group 9
//  PieCtrlRegs.PIEIER9.bit.INTx1 = 1;      // SCIA RX interrupt of PIE group
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM
    SysRegs.SysMachine =System_STATE_INIT;
    PrtectRelayRegs.StateMachine=STATE_INIT;
    SysRegs.PackStateReg.bit.CANCOMEnable=0;
    SysRegs.PackStateReg.bit.INITOK=0;
    while(1)
    {
        SysRegs.Maincount++;

        switch(SysRegs.SysMachine)
        {
            case System_STATE_INIT:
                  SysTimerINIT(&SysRegs);
                  SysVarINIT(&SysRegs);
                  CANRegVarINIT(&CANARegs);
                  ModuleInit(&ModRegs);
                  BatCalcRegsInit(&BatCalcRegs);
                  ProtectRlyVarINIT(&PrtectRelayRegs);
                  CalKokam100AhRegsInit(&Kam100AHSocRegs);
                  // Function SysMachine
                  SysRegs.SysMachine=System_STATE_STANDBY;
                  Kam100AHSocRegs.state=SOC_STATE_IDLE;
                  PrtectRelayRegs.StateMachine=STATE_STANDBY;
                  if(SysRegs.PackStateReg.bit.SysProtect==1)
                  {
                   //   SysRegs.SysMachine=System_STATE_PROTECTER;
                  }
                  SysRegs.PackStateReg.bit.SysSTATE=1;
            break;
            case System_STATE_STANDBY:
                  SysRegs.PackStateReg.bit.CANCOMEnable=1;
                  SysRegs.DigitalOutPutReg.bit.PWRLAMPOUT=0;
                  SysRegs.PackStateReg.bit.INITOK=1;
                  delay_ms(1000);
                  SysRegs.PackStateReg.bit.CANCOMEnable=1;
                  // Function SysMachine
                  SysRegs.SysMachine=System_STATE_READY;
                  Kam100AHSocRegs.state= SOC_STATE_RUNNING;
                  PrtectRelayRegs.StateMachine=STATE_WakeUpReady;
                  if(SysRegs.PackStateReg.bit.SysProtect==1)
                  {
                  //    SysRegs.SysMachine=System_STATE_PROTECTER;
                  }
                  SysRegs.PackStateReg.bit.SysSTATE =2;
            break;
            case System_STATE_READY:
                 SysRegs.PackStateReg.bit.CANCOMEnable=1;
                 SysRegs.DigitalOutPutReg.bit.PWRLAMPOUT=0;

                 if(SysRegs.PackStateReg.bit.SysProtect==1)
                 {
                    PrtectRelayRegs.State.bit.SysFaultState=1;
                 }
                 if(CANARegs.PMSCMDRegs.bit.RUNStatus01==1)
                 {
                     PrtectRelayRegs.State.bit.WakeUpEN=1;
                 }
                 // Function SysMachine

                 if(PrtectRelayRegs.State.bit.WakeuPOnEND==1)
                 {
                     SysRegs.SysMachine=System_STATE_RUNING;
                 }
                 if(SysRegs.PackStateReg.bit.SysProtect==1)
                 {
                  //   SysRegs.SysMachine=System_STATE_PROTECTER;
                 }
                 SysRegs.PackStateReg.bit.SysSTATE = 3;
            break;
            case System_STATE_RUNING:
                 Kam100AHSocRegs.state= SOC_STATE_RUNNING;
                 SysRegs.PackStateReg.bit.CANCOMEnable=1;
                 SysRegs.DigitalOutPutReg.bit.PWRLAMPOUT=1;
                 if(CANARegs.PMSCMDRegs.bit.RUNStatus01==0)
                 {
                     PrtectRelayRegs.State.bit.WakeUpEN=0;
                 }
                 if(PrtectRelayRegs.State.bit.WakeuPOffEND==1)
                 {
                     SysRegs.SysMachine=System_STATE_STANDBY;
                 }
                 if(SysRegs.PackStateReg.bit.SysProtect==1)
                 {
                //   SysRegs.SysMachine=System_STATE_PROTECTER;
                 }
                 SysRegs.PackStateReg.bit.SysSTATE = 4;
            break;
            case System_STATE_PROTECTER:
                 SysRegs.PackStateReg.bit.CANCOMEnable=1;
                 CANARegs.PMSCMDRegs.all=0;
                 PrtectRelayRegs.StateMachine=STATE_ProtectpOFF;
                 if(CANARegs.PMSCMDRegs.bit.PrtctReset01==1)
                 {
                     CANARegs.PMSCMDRegs.bit.PrtctReset01=0;
                     SysTimerINIT(&SysRegs);
                     SysVarINIT(&SysRegs);
                     CANRegVarINIT(&CANARegs);
                     ModuleInit(&ModRegs);
                     BatCalcRegsInit(&BatCalcRegs);
                     ProtectRlyVarINIT(&PrtectRelayRegs);
                     CalKokam100AhRegsInit(&Kam100AHSocRegs);
                     delay_ms(200);
                   //  SysRegs.SysMachine=System_STATE_INIT;
                 }
                 SysRegs.PackStateReg.bit.SysSTATE =5;
            break;
            case System_STATE_DATALOG:
                 SysRegs.PackStateReg.bit.CANCOMEnable=1;
            break;
            case System_STATE_ProtectHistory:

            break;
            case System_STATE_MANUALMode:

            break;
            case System_STATE_CLEAR:

            break;
            default :
            break;
        }
        if(SysRegs.Maincount>3000){SysRegs.Maincount=0;}
        if(CANTXFALAG==1)
        {
            MCP2515InitHandle(&CANBRegs);
            CANTXFALAG=0;
       }
   //     CANBRegs.SPISpeedHz=MeasureSPISpeedHandle(100);
    }
  /*  if(SysRegs.SysMachine==System_STATE_READY)//||(SysRegs.SysMachine==System_STATE_RUNING)||(SysRegs.SysMachine==System_STATE_PROTECTER))
    {
        SysRegs.SysMachine=System_STATE_STANDBY;
    }*/

}

interrupt void cpu_timer0_isr(void)
{
   SysRegs.MainIsr1++;
   SysRegs.SysRegTimer5msecCount++;
   SysRegs.SysRegTimer10msecCount++;
   SysRegs.SysRegTimer50msecCount++;
   SysRegs.SysRegTimer100msecCount++;
   SysRegs.SysRegTimer300msecCount++;
   SysRegs.SysRegTimer500msecCount++;
   SysRegs.SysRegTimer1000msecCount++;
   SysRegs.CellVoltsampling++;
   if(SysRegs.SysRegTimer5msecCount   >=SysRegTimer5msec)    {SysRegs.SysRegTimer5msecCount=0;}
   if(SysRegs.SysRegTimer10msecCount  >=SysRegTimer10msec)   {SysRegs.SysRegTimer10msecCount=0;}
   if(SysRegs.SysRegTimer50msecCount  >=SysRegTimer50msec)   {SysRegs.SysRegTimer50msecCount=0;}
   if(SysRegs.SysRegTimer100msecCount >=SysRegTimer100msec)  {SysRegs.SysRegTimer100msecCount=0;}
   if(SysRegs.SysRegTimer300msecCount >SysRegTimer300msec)   {SysRegs.SysRegTimer300msecCount=0;}
   if(SysRegs.SysRegTimer500msecCount >SysRegTimer500msec)   {SysRegs.SysRegTimer500msecCount=0;}
   if(SysRegs.SysRegTimer1000msecCount>SysRegTimer1000msec)  {SysRegs.SysRegTimer1000msecCount=0;}
   if(SysRegs.PackStateReg.bit.CANRXCountReset==1)
   {
       ModuleInit(&ModRegs);
       SysRegs.PackStateReg.bit.CANRXCountReset=0;
   }
   /*
    * DigitalInput detection
    */
   DigitalInput(&SysRegs);
   /*
    *
    */

   PrtectRelayRegs.State.bit.NRlyDI=SysRegs.DigitalInputReg.bit.NAUX;
   PrtectRelayRegs.State.bit.PRlyDI=SysRegs.DigitalInputReg.bit.PAUX;
   ProtectRlySateCheck(&PrtectRelayRegs);
   SysRegs.DigitalOutPutReg.bit.NRlyOUT=PrtectRelayRegs.State.bit.NRlyDO;
   SysRegs.DigitalOutPutReg.bit.PRlyOUT=PrtectRelayRegs.State.bit.PRlyDO;
   SysRegs.DigitalOutPutReg.bit.ProRlyOUT=PrtectRelayRegs.State.bit.PreRlyDO;
   SysRegs.PackProtectReg.bit.PackRlyErr=PrtectRelayRegs.State.bit.RlyFaulttSate;

  /*
   * current sensing detection
  */
   SysCurrentHandle(&SysRegs);
   /*
    *
    */
   memcpy(&BatCalcRegs.MDCellMaxVolt[0], &ModRegs.MDCellMaxVolt[0],sizeof(Uint32)*7);
   memcpy(&BatCalcRegs.MDCellMinVolt[0], &ModRegs.MDCellMinVolt[0],sizeof(Uint32)*7);
   memcpy(&BatCalcRegs.MDTotalVolt[0],   &ModRegs.MDTotalVolt[0],sizeof(Uint32)*7);
   memcpy(&BatCalcRegs.MDMaxVoltPo[0],   &ModRegs.MDMaxVoltPo[0],sizeof(Uint32)*7);
   memcpy(&BatCalcRegs.MDMinVoltPo[0],   &ModRegs.MDMinVoltPo[0],sizeof(Uint32)*7);
   BatCalcVoltHandle(&BatCalcRegs);
   SysRegs.PackVoltageF= BatCalcRegs.PackPTCANF;
   SysRegs.PackCellMaxVoltageF= BatCalcRegs.PackCellMaxVoltF;
   SysRegs.PackCellMinVoltageF= BatCalcRegs.PackCellMinVoltF;
   SysRegs.PackCellAgvVoltageF= BatCalcRegs.PackCellAgvVoltF;
   SysRegs.PackCellDivVoltageF= BatCalcRegs.PackCellDivVoltF;
   SysRegs.PackCellMaxVoltPos = BatCalcRegs.PackCellMaxVoltPos;
   SysRegs.PackCellMinVoltPos = BatCalcRegs.PackCellMinVoltPos;

   /*
    * SOC Algorithm
    */
   Kam100AHSocRegs.CellAgvVoltageF = SysRegs.PackCellAgvVoltageF;
   Kam100AHSocRegs.SysSoCCTF       = SysRegs.PackCurrentF;
   Kam100AHSocRegs.SysSoCCTAbsF    = SysRegs.PackCurrentAsbF;
   Calkokam100AhSocHandle(&Kam100AHSocRegs);
   if(Kam100AHSocRegs.SoCStateRegs.bit.CalMeth==0)
   {
       SysRegs.PackSOCF=Kam100AHSocRegs.SysSocInitF;
   }
   if(Kam100AHSocRegs.SoCStateRegs.bit.CalMeth==1)
   {
       SysRegs.PackSOCF=Kam100AHSocRegs.SysSOCF;
   }


   /* ModRegs.MDCellMinTemps
    *
    *
    *
    * ModRegs.MDCellMaxTemps
    * ModRegs.MDCellMinTemp
    * ModRegs.MDMinTempsPo
    * ModRegs.MDMaxTempsPo
    */
   memcpy(&BatCalcRegs.MDCellMaxTemps[0], &ModRegs.MDCellMaxTemps[0],sizeof(Uint32)*7);
   memcpy(&BatCalcRegs.MDCellMinTemps[0], &ModRegs.MDCellMinTemps[0],sizeof(Uint32)*7);
   memcpy(&BatCalcRegs.MDMaxTempsPo[0],   &ModRegs.MDMaxTempsPo[0],sizeof(Uint32)*7);
   memcpy(&BatCalcRegs.MDMinTempsPo[0],   &ModRegs.MDMinTempsPo[0],sizeof(Uint32)*7);
   BatCalcTempsHandle(&BatCalcRegs);
   SysRegs.PackCellMaxTemperatureF= BatCalcRegs.PackCellMaxTempsF;
   SysRegs.PackCellMinTemperatureF= BatCalcRegs.PackCellMinTempsF;
   SysRegs.PackCellAgvTemperatureF= BatCalcRegs.PackCellAgvTempsF;
   SysRegs.PackCellDivTemperatureF= BatCalcRegs.PackCellDivTempsF;
   SysRegs.PackCellMaxTmepsPos    = BatCalcRegs.PackCellMaxTempsPos;
   SysRegs.PackCellMinTmepsPos    = BatCalcRegs.PackCellMinTempsPos;

   /*
    * 에러 검출
    */
   SysCommErrHandle(&SysRegs);
   if(SysRegs.DigitalInputReg.bit.EMGSWStauts==1)
   {
       SysRegs.PackProtectReg.bit.PackEMSSWErr=1;
   }
   else
   {
       SysRegs.PackProtectReg.bit.PackEMSSWErr=0;
   }
   if(SysRegs.PackAlarmReg.all != 0)
   {
    //   SysRegs.PackStateReg.bit.SysAalarm=1;
   }
   else
   {
     SysRegs.PackStateReg.bit.SysAalarm=0;
   }
  // Cal80VSysFaultCheck(&SysRegs);
   if(SysRegs.PackFaultReg.all != 0)
   {

     SysRegs.PackStateReg.bit.SysFault=1;
   }
   else
   {
     SysRegs.PackStateReg.bit.SysFault=0;
   }

   if(SysRegs.PackProtectReg.all != 0)
   {
       SysRegs.PackStateReg.bit.SysProtect=1;
   }
   /*
    *
    */
   if(CANARegs.PMSCMDRegs.bit.PrtctReset01==1)
   {
       SysRegs.SysMachine=System_STATE_INIT;
   }
   /*
    *
    */

   //PrtectRelayRegs.State.bit.NRelayDI= SysRegs.PackDigitalInputReg.bit.NAUX;
  // PrtectRelayRegs.State.bit.PRelayDI= SysRegs.PackDigitalInputReg.bit.PAUX;
  // PrtectRelayRegs.State.bit.WakeUpEN=1;
  // WakeUpHandle(&PrtectRelayRegs);
  // SysRegs.PackDigitalOutPutReg.bit.RRlyOUT=PrtectRelayRegs.State.bit.PRelayDO;
  // SysRegs.PackDigitalOutPutReg.bit.NRlyOUT=PrtectRelayRegs.State.bit.NRelayDO;
  // SysRegs.PackDigitalOutPutReg.bit.CHARlyOUT=PrtectRelayRegs.State.bit.PreRelayDO;

   switch(SysRegs.SysRegTimer5msecCount)
   {
       case 1:

       break;
       default :
       break;

   }
   switch(SysRegs.SysRegTimer10msecCount)
   {
       case 1:
       break;
       case 2:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   SysRegs.PackSOHF = 100.0;
                   CANARegs.PackPT = (unsigned int)(SysRegs.PackVoltageF*10);
                   CANARegs.PackCT = (int)(SysRegs.PackCurrentF*10.0);
                   CANARegs.PackSOC =(unsigned int)(SysRegs.PackSOCF*10);
                   CANARegs.PackSOH =(unsigned int)(SysRegs.PackSOHF*10);
                   CANARegs.PackID =0X611;
                   CANATX(CANARegs.PackID ,8,CANARegs.PackPT,CANARegs.PackCT,CANARegs.PackSOC,CANARegs.PackSOH);
               }
             //  CANARegs.PackID =0X611;
             //  CANATX(CANARegs.PackID ,8,0X5555,0XAAAA,0,0);
       break;
       case 3:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.PackSate                        = SysRegs.PackStateReg.bit.SysSTATE;
                   CANARegs.PackProtetSate                  = SysRegs.PackStateReg.bit.SysProtectSate;
                   CANARegs.PackSateInfo                    =  ComBine(CANARegs.PackProtetSate, CANARegs.PackSate);
                   CANARegs.PackStatus.bit.PackNeg_Rly     = PrtectRelayRegs.State.bit.NRlyDI;
                   CANARegs.PackStatus.bit.PackPos_Rly     = PrtectRelayRegs.State.bit.PRlyDI;
                   CANARegs.PackStatus.bit.PackPreChar_Rly = PrtectRelayRegs.State.bit.ProRlyDI;
                   CANARegs.PackStatus.bit.PackMSD_AUX     = SysRegs.PackStateReg.bit.MSDERR;
                   CANARegs.PackStatus.bit.PackEMG_SW      = SysRegs.PackStateReg.bit.EMGSWERR;
                   CANARegs.PackStatus.bit.PackWaterleak   = SysRegs.PackStateReg.bit.WaterLeakERR;
                   CANARegs.PackAh = (int)(SysRegs.PackAhF*10);
                   CANARegs.PackID =0X612;
                   CANATX(CANARegs.PackID ,8,CANARegs.PackSateInfo,CANARegs.PackStatus.all,CANARegs.PackAh,0X000);
               }
       break;

       default :
       break;
   }
   switch(SysRegs.SysRegTimer50msecCount)
   {
       case 1:

       break;
       case 5:

       break;
       case 10:


       break;
       case 20:

       break;
       case 30 :


       break;
       default :
       break;
   }

   switch(SysRegs.SysRegTimer100msecCount)
   {
       case 5:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   //Cell MiN Voltage Offset 10mV
                   CANARegs.PackBalanVolt= (unsigned int)((SysRegs.PackCellMinVoltageF+C_BalanceVoltOffset)*1000);
                   CANARegs.PackID =0X300;
                   CANATX(CANARegs.PackID,8,0X000,0X000,CANARegs.PackTemperatureAVG,CANARegs.PackBalanVolt);
               }
       break;
       case 8:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                 CANARegs.PackID =0X613;
                 CANATX(CANARegs.PackID,8,SysRegs.PackAlarmReg.Word.DataL,SysRegs.PackFaultReg.Word.DataL,SysRegs.PackProtectReg.Word.DataL,SysRegs.PackProtectReg.Word.DataH);
               }
       break;
       case 11:

                //At 80MHZ, operation time is 0.151msec
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                 SysRegs.PackCHAPWRContintyF    =  43.0;
                 SysRegs.PackDisCHAPWRContintyF =  92.7;
                 SysRegs.PackCHAPWRPeakF        =  60.0;
                 SysRegs.PackDisCHAPWRPeakF     =  150,0;
                 CANARegs.PackCHAPWRContinty    = (unsigned int)(SysRegs.PackCHAPWRContintyF*10);
                 CANARegs.PackDisCHAPWRContinty = (unsigned int)(SysRegs.PackDisCHAPWRContintyF*10);
                 CANARegs.PackCHAPWRPeak        = (unsigned int)(SysRegs.PackCHAPWRPeakF*10);
                 CANARegs.PackDisCHAPWRPeak     = (unsigned int)(SysRegs.PackDisCHAPWRPeakF*10);
                 CANARegs.PackID =0X614;
                 CANATX(CANARegs.PackID,8,CANARegs.PackCHAPWRContinty,CANARegs.PackDisCHAPWRContinty,CANARegs.PackCHAPWRPeak,CANARegs.PackDisCHAPWRPeak);
               }
       break;
       case 14:
                if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                {
                   CANARegs.PackVoltageMax = (unsigned int)(SysRegs.PackCellMaxVoltageF*1000);
                   CANARegs.PackVoltageMin = (unsigned int)(SysRegs.PackCellMinVoltageF*1000);
                   CANARegs.PackVoltageAgv = (unsigned int)(SysRegs.PackCellAgvVoltageF*1000);
                   CANARegs.PackVoltageDiv = (unsigned int)(SysRegs.PackCellDivVoltageF*1000);
                   CANARegs.PackID =0X615;
                   CANATX(CANARegs.PackID,8,CANARegs.PackVoltageMax,CANARegs.PackVoltageMin,CANARegs.PackVoltageAgv,CANARegs.PackVoltageDiv);
                }
       break;
       case 17:
                if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                {
                    CANARegs.PackTemperaturelMAX    = (unsigned int)(SysRegs.PackCellMaxTemperatureF*10);
                    CANARegs.PackTemperaturelMIN    = (unsigned int)(SysRegs.PackCellMinTemperatureF*10);
                    CANARegs.PackTemperatureAVG     = (unsigned int)(SysRegs.PackCellAgvTemperatureF*10);
                    CANARegs.PackTemperatureDiv     = (unsigned int)(SysRegs.PackCellDivTemperatureF*10);
                    CANARegs.PackID =0X616;
                    CANATX(CANARegs.PackID,8,CANARegs.PackTemperaturelMAX,CANARegs.PackTemperaturelMIN,CANARegs.PackTemperatureAVG,CANARegs.PackTemperatureDiv);
                }
       break;
       case 20:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.PackVoltageMaxNum          = SysRegs.PackCellMaxVoltPos;
                   CANARegs.PackVoltageMinNum          = SysRegs.PackCellMinVoltPos;
                   CANARegs.PackTemperatureMaxNUM      = SysRegs.PackCellMaxTmepsPos;
                   CANARegs.PackTemperatureMinNUM      = SysRegs.PackCellMinTmepsPos;
                   CANARegs.PackID =0X617;
                   CANATX(CANARegs.PackID,8,CANARegs.PackVoltageMaxNum,CANARegs.PackVoltageMinNum,CANARegs.PackTemperatureMaxNUM ,CANARegs.PackTemperatureMinNUM );
               }
       break;
       case 23:

       break;
       case 26:

       break;
       case 30:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {

               }
       break;
       case 35:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {

               }
       break;
       case 38:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {

               }
       break;
       default:
       break;
   }
   switch(SysRegs.SysRegTimer300msecCount)
   {
       case 1:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.MD1_TotalVolt    = ModRegs.MDTotalVolt[0];
                   CANARegs.MD1_CellAgvVolt  = ModRegs.MDCellAgvVolt[0];
                   CANARegs.MD1_CellAgvTemps = ModRegs.MDCellAgvTemps[0];
                   CANARegs.MD1Staue.all     = ModRegs.MDstatusbit[0];
                   CANARegs.PackID =0X618;
                   CANATX(CANARegs.PackID,8,CANARegs.MD1_TotalVolt,CANARegs.MD1_CellAgvVolt,CANARegs.MD1_CellAgvTemps ,CANARegs.MD1Staue.all);

               }
       break;
       case 10:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.MD2_TotalVolt    = ModRegs.MDTotalVolt[1];
                   CANARegs.MD2_CellAgvVolt  = ModRegs.MDCellAgvVolt[1];
                   CANARegs.MD2_CellAgvTemps = ModRegs.MDCellAgvTemps[1];
                   CANARegs.MD2Staue.all     = ModRegs.MDstatusbit[1];
                   CANARegs.PackID =0X619;
                   CANATX(CANARegs.PackID,8,CANARegs.MD2_TotalVolt,CANARegs.MD2_CellAgvVolt,CANARegs.MD2_CellAgvTemps ,CANARegs.MD2Staue.all);
               }
       break;
       case 20:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.MD3_TotalVolt    = ModRegs.MDTotalVolt[2];
                   CANARegs.MD3_CellAgvVolt  = ModRegs.MDCellAgvVolt[2];
                   CANARegs.MD3_CellAgvTemps = ModRegs.MDCellAgvTemps[2];
                   CANARegs.MD3Staue.all     = ModRegs.MDstatusbit[2];
                   CANARegs.PackID =0X61A;
                   CANATX(CANARegs.PackID,8,CANARegs.MD3_TotalVolt,CANARegs.MD3_CellAgvVolt,CANARegs.MD3_CellAgvTemps ,CANARegs.MD3Staue.all);
               }
       break;
       case 30:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.MD4_TotalVolt    = ModRegs.MDTotalVolt[3];
                   CANARegs.MD4_CellAgvVolt  = ModRegs.MDCellAgvVolt[3];
                   CANARegs.MD4_CellAgvTemps = ModRegs.MDCellAgvTemps[3];
                   CANARegs.MD4Staue.all     = ModRegs.MDstatusbit[3];
                   CANARegs.PackID =0X61B;
                   CANATX(CANARegs.PackID,8,CANARegs.MD4_TotalVolt,CANARegs.MD4_CellAgvVolt,CANARegs.MD2_CellAgvTemps ,CANARegs.MD2Staue.all);
               }
       break;
       case 40:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.MD5_TotalVolt    = ModRegs.MDTotalVolt[4];
                   CANARegs.MD5_CellAgvVolt  = ModRegs.MDCellAgvVolt[4];
                   CANARegs.MD5_CellAgvTemps = ModRegs.MDCellAgvTemps[4];
                   CANARegs.MD5Staue.all     = ModRegs.MDstatusbit[4];
                   CANARegs.PackID =0X61C;
                   CANATX(CANARegs.PackID,8,CANARegs.MD4_TotalVolt,CANARegs.MD4_CellAgvVolt,CANARegs.MD2_CellAgvTemps ,CANARegs.MD2Staue.all);
               }
       break;
       case 50:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.MD6_TotalVolt    = ModRegs.MDTotalVolt[5];
                   CANARegs.MD6_CellAgvVolt  = ModRegs.MDCellAgvVolt[5];
                   CANARegs.MD6_CellAgvTemps = ModRegs.MDCellAgvTemps[5];
                   CANARegs.MD6Staue.all     = ModRegs.MDstatusbit[5];
                   CANARegs.PackID =0X61D;
                   CANATX(CANARegs.PackID,8,CANARegs.MD4_TotalVolt,CANARegs.MD4_CellAgvVolt,CANARegs.MD2_CellAgvTemps ,CANARegs.MD2Staue.all);
               }
       break;
       case 60:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.MD7_TotalVolt    = ModRegs.MDTotalVolt[6];
                   CANARegs.MD7_CellAgvVolt  = ModRegs.MDCellAgvVolt[6];
                   CANARegs.MD7_CellAgvTemps = ModRegs.MDCellAgvTemps[6];
                   CANARegs.MD7Staue.all     = ModRegs.MDstatusbit[6];
                   CANARegs.PackID =0X61E;
                   CANATX(CANARegs.PackID,8,CANARegs.MD4_TotalVolt,CANARegs.MD4_CellAgvVolt,CANARegs.MD2_CellAgvTemps ,CANARegs.MD2Staue.all);
               }
       break;

       default :
       break;
   }
   switch(SysRegs.SysRegTimer500msecCount)
   {
       case 1:
       break;

       case 40:

       break;
       case 60:

       break;
       case 80:

       break;
       case 100:

       break;
       case 150:

       break;
       case 200:

       break;
       case 250:

       break;
       default :
       break;
   }
   switch(SysRegs.SysRegTimer1000msecCount)
   {
       case 1:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.SwVerProducttype      = ComBine(Product_Version,Product_Type);
                   CANARegs.BatConfParallelSerial = ComBine(Product_SysCellVauleP,Product_SysCellVauleS);
                   CANARegs.PackID =0X610;
                   CANATX(CANARegs.PackID,8,CANARegs.SwVerProducttype,CANARegs.BatConfParallelSerial,(unsigned int)(Product_Voltage*10),(unsigned int)(Product_Capacity*10));
               }

       break;
       default :
       break;
   }
   DigitalOutput(&SysRegs);

   //

   // Acknowledge this interrupt to receive more interrupts from group 1
   if(SysRegs.MainIsr1>3000) {SysRegs.MainIsr1=0;}
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
interrupt void ISR_CANRXINTA(void)
{
    struct ECAN_REGS ECanaShadow;
    CANARegs.MailBoxRxCount++;
    if(CANARegs.MailBoxRxCount>3000){CANARegs.MailBoxRxCount=0;}
    if(ECanaRegs.CANGIF0.bit.GMIF0 == 1)
    {

        if(ECanaRegs.CANRMP.bit.RMP0==1)
        {
            ModRegs.MD41XRxcount[0]++;
            SysRegs.MD1CANRxCount=0;
             switch(ECanaMboxes.MBOX0.MSGID.bit.STDMSGID)
             {
                #if(PackNum==1)
                    case (0x110):

                                ModRegs.MD41XRxcount[1]++;
                                ModRegs.MDCellVoltQty[0]    = ECanaMboxes.MBOX0.MDL.byte.BYTE0;
                                ModRegs.MDCellTempsQty[0]   = ECanaMboxes.MBOX0.MDL.byte.BYTE1;
                                ModRegs.MDFirmwareVer[0]    = ECanaMboxes.MBOX0.MDL.byte.BYTE2;
                                ModRegs.MDFirmwareRev[0]    = ECanaMboxes.MBOX0.MDL.byte.BYTE3;
                                ModRegs.MDNorCapacity[0]    = ECanaMboxes.MBOX0.MDH.byte.BYTE4;
                                ModRegs.MDNorVolt[0]        = ECanaMboxes.MBOX0.MDH.byte.BYTE5;
                                ModRegs.PackMinVolteRec[0]  = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);
                    break;
                    case (0x111):
                                ModRegs.MD41XRxcount[2]++;
                                ModRegs.MDCellMaxVolt[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                                ModRegs.MDCellMinVolt[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                                ModRegs.MDCellAgvVolt[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                                ModRegs.MDCellDivVolt[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);

                    break;
                    case (0x112):

                                ModRegs.MD41XRxcount[3]++;
                                ModRegs.MDCellMaxTemps[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                                ModRegs.MDCellMinTemps[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                                ModRegs.MDCellAgvTemps[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                                ModRegs.MDCellDivTemps[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);

                    break;
                    case (0x113):
                                ModRegs.MD41XRxcount[5]++;
                                ModRegs.MDTotalVolt[0]     = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                                ModRegs.MDMaxVoltPo[0]     = ECanaMboxes.MBOX0.MDL.byte.BYTE2;
                                ModRegs.MDMinVoltPo[0]     = ECanaMboxes.MBOX0.MDL.byte.BYTE3;
                                ModRegs.MDMaxTempsPo[0]    = ECanaMboxes.MBOX0.MDH.byte.BYTE4;
                                ModRegs.MDMinTempsPo[0]    = ECanaMboxes.MBOX0.MDH.byte.BYTE5;
                                ModRegs.MDstatusbit[0]     = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);
                    break;

                    default:
                    break;
                #endif
                #if(PackNum==2)
                    case (0x212)://1

                                ModRegs.MD41XRxcount[1]++;
                                ModRegs.MDCellVoltQty[0]    = ECanaMboxes.MBOX0.MDL.byte.BYTE0;
                                ModRegs.MDFirmwareVer[0]    = ECanaMboxes.MBOX0.MDL.byte.BYTE1;
                                ModRegs.MDNorVolt[0]        = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                                ModRegs.MDNorCapacity[0]    = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                                ModRegs.PackMinVolteRec[0]  = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);
                    break;
                    case (0x213)://2
                                ModRegs.MD41XRxcount[2]++;
                                ModRegs.MDCellMaxVolt[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                                ModRegs.MDCellMinVolt[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                                ModRegs.MDCellAgvVolt[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                                ModRegs.MDCellDivVolt[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);

                    break;
                    case (0x214)://3

                                ModRegs.MD41XRxcount[3]++;
                                ModRegs.MDCellMaxTemps[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                                ModRegs.MDCellMinTemps[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                                ModRegs.MDCellAgvTemps[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                                ModRegs.MDCellDivTemps[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);

                    break;
                    case (0x215)://4
                                ModRegs.MD41XRxcount[4]++;
                                ModRegs.MDTotalVolt[0]     = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                                ModRegs.MDMaxVoltPo[0]     = ECanaMboxes.MBOX0.MDL.byte.BYTE2;
                                ModRegs.MDMinVoltPo[0]     = ECanaMboxes.MBOX0.MDL.byte.BYTE3;
                                ModRegs.MDMaxTempsPo[0]    = ECanaMboxes.MBOX0.MDH.byte.BYTE4;
                                ModRegs.MDMinTempsPo[0]    = ECanaMboxes.MBOX0.MDH.byte.BYTE5;
                                ModRegs.MDstatusbit[0]     = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);
                    break;


                    default:
                    break;
              #endif
              #if(PackNum==3)
                case (0x311):

                            ModRegs.MD41XRxcount[1]++;
                            ModRegs.MDCellVoltQty[0]    = ECanaMboxes.MBOX0.MDL.byte.BYTE0;
                            ModRegs.MDFirmwareVer[0]    = ECanaMboxes.MBOX0.MDL.byte.BYTE1;
                            ModRegs.MDNorCapacity[0]    = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                            ModRegs.MDNorVolt[0]        = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                            ModRegs.PackMinVolteRec[0]  = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);
                break;
                case (0x312):
                            ModRegs.MD41XRxcount[2]++;
                            ModRegs.MDCellMaxVolt[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                            ModRegs.MDCellMinVolt[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                            ModRegs.MDCellAgvVolt[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                            ModRegs.MDCellDivVolt[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);

                break;
                case (0x313):

                            ModRegs.MD41XRxcount[3]++;
                            ModRegs.MDCellMaxTemps[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                            ModRegs.MDCellMinTemps[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                            ModRegs.MDCellAgvTemps[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                            ModRegs.MDCellDivTemps[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);

                break;
                case (0x314):
                            ModRegs.MD41XRxcount[5]++;
                            ModRegs.MDTotalVolt[0]     = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                            ModRegs.MDMaxVoltPo[0]     = ECanaMboxes.MBOX0.MDL.byte.BYTE2;
                            ModRegs.MDMinVoltPo[0]     = ECanaMboxes.MBOX0.MDL.byte.BYTE3;
                            ModRegs.MDMaxTempsPo[0]    = ECanaMboxes.MBOX0.MDH.byte.BYTE4;
                            ModRegs.MDMinTempsPo[0]    = ECanaMboxes.MBOX0.MDH.byte.BYTE5;
                            ModRegs.MDstatusbit[0]     = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);
                break;

                default:
                break;
            #endif
            #if(PackNum==4)
              case (0x411):

                          ModRegs.MD41XRxcount[1]++;
                          ModRegs.MDCellVoltQty[0]    = ECanaMboxes.MBOX0.MDL.byte.BYTE0;
                          ModRegs.MDFirmwareVer[0]    = ECanaMboxes.MBOX0.MDL.byte.BYTE1;
                          ModRegs.MDNorCapacity[0]    = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                          ModRegs.MDNorVolt[0]        = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                          ModRegs.PackMinVolteRec[0]  = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);
              break;
              case (0x412):
                          ModRegs.MD41XRxcount[2]++;
                          ModRegs.MDCellMaxVolt[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                          ModRegs.MDCellMinVolt[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                          ModRegs.MDCellAgvVolt[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                          ModRegs.MDCellDivVolt[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);

              break;
              case (0x413):

                          ModRegs.MD41XRxcount[3]++;
                          ModRegs.MDCellMaxTemps[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                          ModRegs.MDCellMinTemps[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                          ModRegs.MDCellAgvTemps[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                          ModRegs.MDCellDivTemps[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);

              break;
              case (0x414):
                          ModRegs.MD41XRxcount[5]++;
                          ModRegs.MDTotalVolt[0]     = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                          ModRegs.MDMaxVoltPo[0]     = ECanaMboxes.MBOX0.MDL.byte.BYTE2;
                          ModRegs.MDMinVoltPo[0]     = ECanaMboxes.MBOX0.MDL.byte.BYTE3;
                          ModRegs.MDMaxTempsPo[0]    = ECanaMboxes.MBOX0.MDH.byte.BYTE4;
                          ModRegs.MDMinTempsPo[0]    = ECanaMboxes.MBOX0.MDH.byte.BYTE5;
                          ModRegs.MDstatusbit[0]     = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);
              break;

              default:
              break;
            #endif
             }
             ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
             ECanaShadow.CANRMP.all=0;
             ECanaShadow.CANRMP.bit.RMP0 = 1;
             ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

        }
        if(ECanaRegs.CANRMP.bit.RMP1==1) //0XR2~4 , R1~4)
        {
            ModRegs.MD42XRxcount[0]++;
            SysRegs.MD2CANRxCount=0;
            switch(ECanaMboxes.MBOX1.MSGID.bit.STDMSGID)
            {
             #if(PackNum==1)
               case (0x120):
                               ModRegs.MD42XRxcount[1]++;
                               ModRegs.MDCellVoltQty[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE0;
                               ModRegs.MDCellTempsQty[1]   = ECanaMboxes.MBOX1.MDL.byte.BYTE1;
                               ModRegs.MDFirmwareVer[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE2;
                               ModRegs.MDFirmwareRev[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE3;
                               ModRegs.MDNorCapacity[1]    = ECanaMboxes.MBOX1.MDH.byte.BYTE4;
                               ModRegs.MDNorVolt[1]        = ECanaMboxes.MBOX1.MDH.byte.BYTE5;
                               ModRegs.PackMinVolteRec[1]  = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

               break;
               case (0x121):
                               ModRegs.MD42XRxcount[2]++;
                               ModRegs.MDCellMaxVolt[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                               ModRegs.MDCellMinVolt[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                               ModRegs.MDCellAgvVolt[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE4);
                               ModRegs.MDCellDivVolt[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

               break;
               case (0x122):

                               ModRegs.MD42XRxcount[3]++;
                               ModRegs.MDCellMaxTemps[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                               ModRegs.MDCellMinTemps[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                               ModRegs.MDCellAgvTemps[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE4);
                               ModRegs.MDCellDivTemps[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

               break;
               case (0x123):
                               ModRegs.MD42XRxcount[4]++;
                               ModRegs.MDTotalVolt[1]     = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                               ModRegs.MDMaxVoltPo[1]     = ECanaMboxes.MBOX1.MDL.byte.BYTE2;
                               ModRegs.MDMinVoltPo[1]     = ECanaMboxes.MBOX1.MDL.byte.BYTE3;
                               ModRegs.MDMaxTempsPo[1]    = ECanaMboxes.MBOX1.MDH.byte.BYTE4;
                               ModRegs.MDMinTempsPo[1]    = ECanaMboxes.MBOX1.MDH.byte.BYTE5;
                               ModRegs.MDstatusbit[1]     = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);
               break;
             #endif
             #if(PackNum==2)
               case (0x222):
                               ModRegs.MD42XRxcount[1]++;
                               ModRegs.MDCellVoltQty[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE0;
                               ModRegs.MDFirmwareVer[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE1;
                               ModRegs.MDNorVolt[1]        = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                               ModRegs.MDNorCapacity[1]    = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE5);
                               ModRegs.PackMinVolteRec[1]  = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

               break;
               case (0x223):
                               ModRegs.MD42XRxcount[2]++;
                               ModRegs.MDCellMaxVolt[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                               ModRegs.MDCellMinVolt[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                               ModRegs.MDCellAgvVolt[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE4);
                               ModRegs.MDCellDivVolt[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

               break;
               case (0x224):

                               ModRegs.MD42XRxcount[3]++;
                               ModRegs.MDCellMaxTemps[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                               ModRegs.MDCellMinTemps[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                               ModRegs.MDCellAgvTemps[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE4);
                               ModRegs.MDCellDivTemps[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

               break;
               case (0x225):
                               ModRegs.MD42XRxcount[4]++;
                               ModRegs.MDTotalVolt[1]     = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                               ModRegs.MDMaxVoltPo[1]     = ECanaMboxes.MBOX1.MDL.byte.BYTE2;
                               ModRegs.MDMinVoltPo[1]     = ECanaMboxes.MBOX1.MDL.byte.BYTE3;
                               ModRegs.MDMaxTempsPo[1]    = ECanaMboxes.MBOX1.MDH.byte.BYTE4;
                               ModRegs.MDMinTempsPo[1]    = ECanaMboxes.MBOX1.MDH.byte.BYTE5;
                               ModRegs.MDstatusbit[1]     = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);
               break;
             #endif
            #if(PackNum==3)
              case (0x322):
                              ModRegs.MD42XRxcount[1]++;
                              ModRegs.MDCellVoltQty[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE0;
                              ModRegs.MDCellTempsQty[1]   = ECanaMboxes.MBOX1.MDL.byte.BYTE1;
                              ModRegs.MDFirmwareVer[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE2;
                              ModRegs.MDFirmwareRev[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE3;
                              ModRegs.MDNorCapacity[1]    = ECanaMboxes.MBOX1.MDH.byte.BYTE4;
                              ModRegs.MDNorVolt[1]        = ECanaMboxes.MBOX1.MDH.byte.BYTE5;
                              ModRegs.PackMinVolteRec[1]  = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

              break;
              case (0x321):
                              ModRegs.MD42XRxcount[2]++;
                              ModRegs.MDCellMaxVolt[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

              break;
              case (0x322):

                              ModRegs.MD42XRxcount[3]++;
                              ModRegs.MDCellMaxTemps[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

              break;
              case (0x323):
                              ModRegs.MD42XRxcount[4]++;
                              ModRegs.MDTotalVolt[1]     = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[1]     = ECanaMboxes.MBOX1.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[1]     = ECanaMboxes.MBOX1.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[1]    = ECanaMboxes.MBOX1.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[1]    = ECanaMboxes.MBOX1.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[1]     = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);
              break;
            #endif
            #if(PackNum==4)
              case (0x420):
                              ModRegs.MD42XRxcount[1]++;
                              ModRegs.MDCellVoltQty[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE0;
                              ModRegs.MDCellTempsQty[1]   = ECanaMboxes.MBOX1.MDL.byte.BYTE1;
                              ModRegs.MDFirmwareVer[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE2;
                              ModRegs.MDFirmwareRev[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE3;
                              ModRegs.MDNorCapacity[1]    = ECanaMboxes.MBOX1.MDH.byte.BYTE4;
                              ModRegs.MDNorVolt[1]        = ECanaMboxes.MBOX1.MDH.byte.BYTE5;
                              ModRegs.PackMinVolteRec[1]  = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

              break;
              case (0x421):
                              ModRegs.MD42XRxcount[2]++;
                              ModRegs.MDCellMaxVolt[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

              break;
              case (0x422):

                              ModRegs.MD42XRxcount[3]++;
                              ModRegs.MDCellMaxTemps[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

              break;
              case (0x423):
                              ModRegs.MD42XRxcount[4]++;
                              ModRegs.MDTotalVolt[1]     = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[1]     = ECanaMboxes.MBOX1.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[1]     = ECanaMboxes.MBOX1.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[1]    = ECanaMboxes.MBOX1.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[1]    = ECanaMboxes.MBOX1.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[1]     = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);
              break;
            #endif

               default:
               break;
            }
            ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
            ECanaShadow.CANRMP.all=0;
            ECanaShadow.CANRMP.bit.RMP1 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;
        }
        if(ECanaRegs.CANRMP.bit.RMP2==1) //0XR31~4 , R1~4)
        {
            ModRegs.MD43XRxcount[0]++;
            SysRegs.MD3CANRxCount=0;
            switch(ECanaMboxes.MBOX2.MSGID.bit.STDMSGID)
            {
                #if(PackNum==1)
                  case (0x130):
                              ModRegs.MD43XRxcount[1]++;
                              ModRegs.MDCellVoltQty[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE0;
                              ModRegs.MDCellTempsQty[2]   = ECanaMboxes.MBOX2.MDL.byte.BYTE1;
                              ModRegs.MDFirmwareVer[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE2;
                              ModRegs.MDFirmwareRev[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE3;
                              ModRegs.MDNorCapacity[2]    = ECanaMboxes.MBOX2.MDH.byte.BYTE4;
                              ModRegs.MDNorVolt[2]        = ECanaMboxes.MBOX2.MDH.byte.BYTE5;
                              ModRegs.PackMinVolteRec[2]  = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                  case (0x131):
                              ModRegs.MD43XRxcount[2]++;
                              ModRegs.MDCellMaxVolt[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                  case (0x132):

                              ModRegs.MD43XRxcount[3]++;
                              ModRegs.MDCellMaxTemps[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                  case (0x133):
                              ModRegs.MD43XRxcount[4]++;
                              ModRegs.MDTotalVolt[2]     = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[2]     = ECanaMboxes.MBOX2.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[2]     = ECanaMboxes.MBOX2.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[2]    = ECanaMboxes.MBOX2.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[2]    = ECanaMboxes.MBOX2.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[2]     = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;

                #endif
                #if(PackNum==2)
                  case (0x232):
                              ModRegs.MD43XRxcount[1]++;
                              ModRegs.MDCellVoltQty[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE0;
                              ModRegs.MDFirmwareVer[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE1;
                              ModRegs.MDNorVolt[2]        = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                              ModRegs.MDNorCapacity[2]    = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                              ModRegs.PackMinVolteRec[2]  = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                  case (0x233):
                              ModRegs.MD43XRxcount[2]++;
                              ModRegs.MDCellMaxVolt[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                  case (0x234):

                              ModRegs.MD43XRxcount[3]++;
                              ModRegs.MDCellMaxTemps[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                  case (0x235):
                              ModRegs.MD43XRxcount[4]++;
                              ModRegs.MDTotalVolt[2]     = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[2]     = ECanaMboxes.MBOX2.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[2]     = ECanaMboxes.MBOX2.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[2]    = ECanaMboxes.MBOX2.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[2]    = ECanaMboxes.MBOX2.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[2]     = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                #endif
                #if(PackNum==3)
                  case (0x330):
                              ModRegs.MD43XRxcount[1]++;
                              ModRegs.MDCellVoltQty[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE0;
                              ModRegs.MDCellTempsQty[2]   = ECanaMboxes.MBOX2.MDL.byte.BYTE1;
                              ModRegs.MDFirmwareVer[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE2;
                              ModRegs.MDFirmwareRev[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE3;
                              ModRegs.MDNorCapacity[2]    = ECanaMboxes.MBOX2.MDH.byte.BYTE4;
                              ModRegs.MDNorVolt[2]        = ECanaMboxes.MBOX2.MDH.byte.BYTE5;
                              ModRegs.PackMinVolteRec[2]  = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                  case (0x331):
                              ModRegs.MD43XRxcount[2]++;
                              ModRegs.MDCellMaxVolt[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                  case (0x332):

                              ModRegs.MD43XRxcount[3]++;
                              ModRegs.MDCellMaxTemps[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                  case (0x333):
                              ModRegs.MD43XRxcount[4]++;
                              ModRegs.MDTotalVolt[2]     = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[2]     = ECanaMboxes.MBOX2.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[2]     = ECanaMboxes.MBOX2.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[2]    = ECanaMboxes.MBOX2.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[2]    = ECanaMboxes.MBOX2.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[2]     = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                #endif
                #if(PackNum==4)
                  case (0x430):
                              ModRegs.MD43XRxcount[1]++;
                              ModRegs.MDCellVoltQty[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE0;
                              ModRegs.MDCellTempsQty[2]   = ECanaMboxes.MBOX2.MDL.byte.BYTE1;
                              ModRegs.MDFirmwareVer[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE2;
                              ModRegs.MDFirmwareRev[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE3;
                              ModRegs.MDNorCapacity[2]    = ECanaMboxes.MBOX2.MDH.byte.BYTE4;
                              ModRegs.MDNorVolt[2]        = ECanaMboxes.MBOX2.MDH.byte.BYTE5;
                              ModRegs.PackMinVolteRec[2]  = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                  case (0x431):
                              ModRegs.MD43XRxcount[2]++;
                              ModRegs.MDCellMaxVolt[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                  case (0x432):

                              ModRegs.MD43XRxcount[3]++;
                              ModRegs.MDCellMaxTemps[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                  case (0x433):
                              ModRegs.MD43XRxcount[4]++;
                              ModRegs.MDTotalVolt[2]     = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[2]     = ECanaMboxes.MBOX2.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[2]     = ECanaMboxes.MBOX2.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[2]    = ECanaMboxes.MBOX2.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[2]    = ECanaMboxes.MBOX2.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[2]     = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                  break;
                #endif
                default:
                break;
            }
            ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
            ECanaShadow.CANRMP.all=0;
            ECanaShadow.CANRMP.bit.RMP2= 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

        }
        if(ECanaRegs.CANRMP.bit.RMP3==1)
        {
            ModRegs.MD44XRxcount[0]++;
            SysRegs.MD4CANRxCount=0;
            switch(ECanaMboxes.MBOX3.MSGID.bit.STDMSGID)
            {
             #if(PackNum==1)
                case (0x140):
                            ModRegs.MD44XRxcount[1]++;
                            ModRegs.MDCellVoltQty[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE0;
                            ModRegs.MDCellTempsQty[3]   = ECanaMboxes.MBOX3.MDL.byte.BYTE1;
                            ModRegs.MDFirmwareVer[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE2;
                            ModRegs.MDFirmwareRev[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE3;
                            ModRegs.MDNorCapacity[3]    = ECanaMboxes.MBOX3.MDH.byte.BYTE4;
                            ModRegs.MDNorVolt[3]        = ECanaMboxes.MBOX3.MDH.byte.BYTE5;
                            ModRegs.PackMinVolteRec[3]  = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

                break;
                case (0x141):
                            ModRegs.MD44XRxcount[2]++;
                            ModRegs.MDCellMaxVolt[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                            ModRegs.MDCellMinVolt[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                            ModRegs.MDCellAgvVolt[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                            ModRegs.MDCellDivVolt[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

                break;
                case (0x142):

                            ModRegs.MD44XRxcount[3]++;
                            ModRegs.MDCellMaxTemps[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                            ModRegs.MDCellMinTemps[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                            ModRegs.MDCellAgvTemps[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                            ModRegs.MDCellDivTemps[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

                break;
                case (0x143):
                            ModRegs.MD44XRxcount[4]++;
                            ModRegs.MDTotalVolt[3]     = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                            ModRegs.MDMaxVoltPo[3]     = ECanaMboxes.MBOX3.MDL.byte.BYTE2;
                            ModRegs.MDMinVoltPo[3]     = ECanaMboxes.MBOX3.MDL.byte.BYTE3;
                            ModRegs.MDMaxTempsPo[3]    = ECanaMboxes.MBOX3.MDH.byte.BYTE4;
                            ModRegs.MDMinTempsPo[3]    = ECanaMboxes.MBOX3.MDH.byte.BYTE5;
                            ModRegs.MDstatusbit[3]     = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);
                break;
            #endif
            #if(PackNum==2)
               case (0x242):
                           ModRegs.MD44XRxcount[1]++;
                           ModRegs.MDCellVoltQty[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE0;
                           ModRegs.MDFirmwareVer[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE1;
                           ModRegs.MDNorVolt    [3]    = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                           ModRegs.MDNorCapacity[3]    = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                           ModRegs.PackMinVolteRec[3]  = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

               break;
               case (0x243):
                           ModRegs.MD44XRxcount[2]++;
                           ModRegs.MDCellMaxVolt[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                           ModRegs.MDCellMinVolt[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                           ModRegs.MDCellAgvVolt[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                           ModRegs.MDCellDivVolt[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

               break;
               case (0x244):

                           ModRegs.MD44XRxcount[3]++;
                           ModRegs.MDCellMaxTemps[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                           ModRegs.MDCellMinTemps[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                           ModRegs.MDCellAgvTemps[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                           ModRegs.MDCellDivTemps[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

               break;
               case (0x245):
                           ModRegs.MD44XRxcount[4]++;
                           ModRegs.MDTotalVolt[3]     = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                           ModRegs.MDMaxVoltPo[3]     = ECanaMboxes.MBOX3.MDL.byte.BYTE2;
                           ModRegs.MDMinVoltPo[3]     = ECanaMboxes.MBOX3.MDL.byte.BYTE3;
                           ModRegs.MDMaxTempsPo[3]    = ECanaMboxes.MBOX3.MDH.byte.BYTE4;
                           ModRegs.MDMinTempsPo[3]    = ECanaMboxes.MBOX3.MDH.byte.BYTE5;
                           ModRegs.MDstatusbit[3]     = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);
               break;
            #endif
            #if(PackNum==3)
               case (0x340):
                           ModRegs.MD44XRxcount[1]++;
                           ModRegs.MDCellVoltQty[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE0;
                           ModRegs.MDCellTempsQty[3]   = ECanaMboxes.MBOX3.MDL.byte.BYTE1;
                           ModRegs.MDFirmwareVer[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE2;
                           ModRegs.MDFirmwareRev[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE3;
                           ModRegs.MDNorCapacity[3]    = ECanaMboxes.MBOX3.MDH.byte.BYTE4;
                           ModRegs.MDNorVolt[3]        = ECanaMboxes.MBOX3.MDH.byte.BYTE5;
                           ModRegs.PackMinVolteRec[3]  = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

               break;
               case (0x341):
                           ModRegs.MD44XRxcount[2]++;
                           ModRegs.MDCellMaxVolt[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                           ModRegs.MDCellMinVolt[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                           ModRegs.MDCellAgvVolt[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                           ModRegs.MDCellDivVolt[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

               break;
               case (0x342):

                           ModRegs.MD44XRxcount[3]++;
                           ModRegs.MDCellMaxTemps[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                           ModRegs.MDCellMinTemps[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                           ModRegs.MDCellAgvTemps[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                           ModRegs.MDCellDivTemps[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

               break;
               case (0x343):
                           ModRegs.MD44XRxcount[4]++;
                           ModRegs.MDTotalVolt[3]     = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                           ModRegs.MDMaxVoltPo[3]     = ECanaMboxes.MBOX3.MDL.byte.BYTE2;
                           ModRegs.MDMinVoltPo[3]     = ECanaMboxes.MBOX3.MDL.byte.BYTE3;
                           ModRegs.MDMaxTempsPo[3]    = ECanaMboxes.MBOX3.MDH.byte.BYTE4;
                           ModRegs.MDMinTempsPo[3]    = ECanaMboxes.MBOX3.MDH.byte.BYTE5;
                           ModRegs.MDstatusbit[3]     = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);
               break;
            #endif
            #if(PackNum==4)
               case (0x440):
                           ModRegs.MD44XRxcount[1]++;
                           ModRegs.MDCellVoltQty[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE0;
                           ModRegs.MDCellTempsQty[3]   = ECanaMboxes.MBOX3.MDL.byte.BYTE1;
                           ModRegs.MDFirmwareVer[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE2;
                           ModRegs.MDFirmwareRev[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE3;
                           ModRegs.MDNorCapacity[3]    = ECanaMboxes.MBOX3.MDH.byte.BYTE4;
                           ModRegs.MDNorVolt[3]        = ECanaMboxes.MBOX3.MDH.byte.BYTE5;
                           ModRegs.PackMinVolteRec[3]  = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

               break;
               case (0x441):
                           ModRegs.MD44XRxcount[2]++;
                           ModRegs.MDCellMaxVolt[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                           ModRegs.MDCellMinVolt[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                           ModRegs.MDCellAgvVolt[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                           ModRegs.MDCellDivVolt[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

               break;
               case (0x442):

                           ModRegs.MD44XRxcount[3]++;
                           ModRegs.MDCellMaxTemps[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                           ModRegs.MDCellMinTemps[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                           ModRegs.MDCellAgvTemps[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                           ModRegs.MDCellDivTemps[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

               break;
               case (0x443):
                           ModRegs.MD44XRxcount[4]++;
                           ModRegs.MDTotalVolt[3]     = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                           ModRegs.MDMaxVoltPo[3]     = ECanaMboxes.MBOX3.MDL.byte.BYTE2;
                           ModRegs.MDMinVoltPo[3]     = ECanaMboxes.MBOX3.MDL.byte.BYTE3;
                           ModRegs.MDMaxTempsPo[3]    = ECanaMboxes.MBOX3.MDH.byte.BYTE4;
                           ModRegs.MDMinTempsPo[3]    = ECanaMboxes.MBOX3.MDH.byte.BYTE5;
                           ModRegs.MDstatusbit[3]     = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);
               break;
            #endif
                default:
                break;
            }
            ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
            ECanaShadow.CANRMP.all=0;
            ECanaShadow.CANRMP.bit.RMP3 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

        }
        if(ECanaRegs.CANRMP.bit.RMP4==1)
        {
            ModRegs.MD45xRxcount[0]++;
            SysRegs.MD5CANRxCount=0;
            switch(ECanaMboxes.MBOX4.MSGID.bit.STDMSGID)
            {
                #if(PackNum==1)
                  case (0x150):
                                ModRegs.MD45xRxcount[1]++;
                                ModRegs.MDCellVoltQty[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE0;
                                ModRegs.MDCellTempsQty[4]   = ECanaMboxes.MBOX4.MDL.byte.BYTE1;
                                ModRegs.MDFirmwareVer[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE2;
                                ModRegs.MDFirmwareRev[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE3;
                                ModRegs.MDNorCapacity[4]    = ECanaMboxes.MBOX4.MDH.byte.BYTE4;
                                ModRegs.MDNorVolt[4]        = ECanaMboxes.MBOX4.MDH.byte.BYTE5;
                                ModRegs.PackMinVolteRec[4]  = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                  case (0x151):
                                ModRegs.MD45xRxcount[2]++;
                                ModRegs.MDCellMaxVolt[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                                ModRegs.MDCellMinVolt[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                                ModRegs.MDCellAgvVolt[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                                ModRegs.MDCellDivVolt[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                  case (0x152):
                                ModRegs.MD45xRxcount[3]++;
                                ModRegs.MDCellMaxTemps[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                                ModRegs.MDCellMinTemps[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                                ModRegs.MDCellAgvTemps[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                                ModRegs.MDCellDivTemps[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                  case (0x153):
                                ModRegs.MD45xRxcount[4]++;
                                ModRegs.MDTotalVolt[4]     = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                                ModRegs.MDMaxVoltPo[4]     = ECanaMboxes.MBOX4.MDL.byte.BYTE2;
                                ModRegs.MDMinVoltPo[4]     = ECanaMboxes.MBOX4.MDL.byte.BYTE3;
                                ModRegs.MDMaxTempsPo[4]    = ECanaMboxes.MBOX4.MDH.byte.BYTE4;
                                ModRegs.MDMinTempsPo[4]    = ECanaMboxes.MBOX4.MDH.byte.BYTE5;
                                ModRegs.MDstatusbit[4]     = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                #endif
                #if(PackNum==2)
                  case (0x252):
                              ModRegs.MD45xRxcount[1]++;
                              ModRegs.MDCellVoltQty[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE0;
                              ModRegs.MDFirmwareVer[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE1;
                              ModRegs.MDNorVolt[4]        = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                              ModRegs.MDNorCapacity[4]    = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                              ModRegs.PackMinVolteRec[4]  = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                  case (0x253):
                              ModRegs.MD45xRxcount[2]++;
                              ModRegs.MDCellMaxVolt[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                  case (0x254):
                              ModRegs.MD45xRxcount[3]++;
                              ModRegs.MDCellMaxTemps[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                  case (0x255):
                              ModRegs.MD45xRxcount[4]++;
                              ModRegs.MDTotalVolt[4]     = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[4]     = ECanaMboxes.MBOX4.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[4]     = ECanaMboxes.MBOX4.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[4]    = ECanaMboxes.MBOX4.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[4]    = ECanaMboxes.MBOX4.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[4]     = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                 #endif
                #if(PackNum==3)
                  case (0x350):
                              ModRegs.MD45xRxcount[1]++;
                              ModRegs.MDCellVoltQty[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE0;
                              ModRegs.MDCellTempsQty[4]   = ECanaMboxes.MBOX4.MDL.byte.BYTE1;
                              ModRegs.MDFirmwareVer[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE2;
                              ModRegs.MDFirmwareRev[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE3;
                              ModRegs.MDNorCapacity[4]    = ECanaMboxes.MBOX4.MDH.byte.BYTE4;
                              ModRegs.MDNorVolt[4]        = ECanaMboxes.MBOX4.MDH.byte.BYTE5;
                              ModRegs.PackMinVolteRec[4]  = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                  case (0x351):
                              ModRegs.MD45xRxcount[2]++;
                              ModRegs.MDCellMaxVolt[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                  case (0x352):
                              ModRegs.MD45xRxcount[3]++;
                              ModRegs.MDCellMaxTemps[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                  case (0x353):
                              ModRegs.MD45xRxcount[4]++;
                              ModRegs.MDTotalVolt[4]     = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[4]     = ECanaMboxes.MBOX4.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[4]     = ECanaMboxes.MBOX4.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[4]    = ECanaMboxes.MBOX4.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[4]    = ECanaMboxes.MBOX4.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[4]     = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                #endif
                #if(PackNum==4)
                  case (0x450):
                              ModRegs.MD45xRxcount[1]++;
                              ModRegs.MDCellVoltQty[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE0;
                              ModRegs.MDCellTempsQty[4]   = ECanaMboxes.MBOX4.MDL.byte.BYTE1;
                              ModRegs.MDFirmwareVer[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE2;
                              ModRegs.MDFirmwareRev[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE3;
                              ModRegs.MDNorCapacity[4]    = ECanaMboxes.MBOX4.MDH.byte.BYTE4;
                              ModRegs.MDNorVolt[4]        = ECanaMboxes.MBOX4.MDH.byte.BYTE5;
                              ModRegs.PackMinVolteRec[4]  = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                  case (0x451):
                              ModRegs.MD45xRxcount[2]++;
                              ModRegs.MDCellMaxVolt[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                  case (0x452):
                              ModRegs.MD45xRxcount[3]++;
                              ModRegs.MDCellMaxTemps[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                  case (0x453):
                              ModRegs.MD45xRxcount[4]++;
                              ModRegs.MDTotalVolt[4]     = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[4]     = ECanaMboxes.MBOX4.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[4]     = ECanaMboxes.MBOX4.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[4]    = ECanaMboxes.MBOX4.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[4]    = ECanaMboxes.MBOX4.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[4]     = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                  break;
                #endif
                default:
                break;
            }
            ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
            ECanaShadow.CANRMP.all=0;
            ECanaShadow.CANRMP.bit.RMP4 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

         }
        if(ECanaRegs.CANRMP.bit.RMP5==1)
        {
            ModRegs.MD46XRxcount[0]++;
            SysRegs.MD6CANRxCount=0;
            switch(ECanaMboxes.MBOX5.MSGID.bit.STDMSGID)
            {
                #if(PackNum==1)
                  case (0x160):
                              ModRegs.MD46XRxcount[1]++;
                              ModRegs.MDCellVoltQty[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE0;
                              ModRegs.MDCellTempsQty[5]   = ECanaMboxes.MBOX5.MDL.byte.BYTE1;
                              ModRegs.MDFirmwareVer[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE2;
                              ModRegs.MDFirmwareRev[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE3;
                              ModRegs.MDNorCapacity[5]    = ECanaMboxes.MBOX5.MDH.byte.BYTE4;
                              ModRegs.MDNorVolt[5]        = ECanaMboxes.MBOX5.MDH.byte.BYTE5;
                              ModRegs.PackMinVolteRec[5]  = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                  case (0x161):
                              ModRegs.MD46XRxcount[2]++;
                              ModRegs.MDCellMaxVolt[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE5,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                  case (0x162):

                              ModRegs.MD46XRxcount[3]++;
                              ModRegs.MDCellMaxTemps[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE5,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                  case (0x163):
                              ModRegs.MD46XRxcount[4]++;
                              ModRegs.MDTotalVolt[5]     = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[5]     = ECanaMboxes.MBOX5.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[5]     = ECanaMboxes.MBOX5.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[5]    = ECanaMboxes.MBOX5.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[5]    = ECanaMboxes.MBOX5.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[5]     = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                #endif
                #if(PackNum==2)
                  case (0x262):
                              ModRegs.MD46XRxcount[1]++;
                              ModRegs.MDCellVoltQty[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE0;
                              ModRegs.MDFirmwareVer[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE1;
                              ModRegs.MDNorVolt[5]        = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                              ModRegs.MDNorCapacity[5]    = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                              ModRegs.PackMinVolteRec[5]  = ComBine(ECanaMboxes.MBOX7.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                  case (0x263):
                              ModRegs.MD46XRxcount[2]++;
                              ModRegs.MDCellMaxVolt[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE5,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                  case (0x264):

                              ModRegs.MD46XRxcount[3]++;
                              ModRegs.MDCellMaxTemps[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE5,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                  case (0x265):
                              ModRegs.MD46XRxcount[4]++;
                              ModRegs.MDTotalVolt[5]     = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[5]     = ECanaMboxes.MBOX5.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[5]     = ECanaMboxes.MBOX5.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[5]    = ECanaMboxes.MBOX5.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[5]    = ECanaMboxes.MBOX5.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[5]     = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                #endif
                #if(PackNum==3)
                  case (0x360):
                              ModRegs.MD46XRxcount[1]++;
                              ModRegs.MDCellVoltQty[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE0;
                              ModRegs.MDCellTempsQty[5]   = ECanaMboxes.MBOX5.MDL.byte.BYTE1;
                              ModRegs.MDFirmwareVer[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE2;
                              ModRegs.MDFirmwareRev[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE3;
                              ModRegs.MDNorCapacity[5]    = ECanaMboxes.MBOX5.MDH.byte.BYTE4;
                              ModRegs.MDNorVolt[5]        = ECanaMboxes.MBOX5.MDH.byte.BYTE5;
                              ModRegs.PackMinVolteRec[5]  = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                  case (0x361):
                              ModRegs.MD46XRxcount[2]++;
                              ModRegs.MDCellMaxVolt[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE5,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                  case (0x362):

                              ModRegs.MD46XRxcount[3]++;
                              ModRegs.MDCellMaxTemps[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE5,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                  case (0x363):
                              ModRegs.MD46XRxcount[4]++;
                              ModRegs.MDTotalVolt[5]     = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[5]     = ECanaMboxes.MBOX5.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[5]     = ECanaMboxes.MBOX5.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[5]    = ECanaMboxes.MBOX5.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[5]    = ECanaMboxes.MBOX5.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[5]     = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                #endif
                #if(PackNum==4)
                  case (0x460):
                              ModRegs.MD46XRxcount[1]++;
                              ModRegs.MDCellVoltQty[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE0;
                              ModRegs.MDCellTempsQty[5]   = ECanaMboxes.MBOX5.MDL.byte.BYTE1;
                              ModRegs.MDFirmwareVer[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE2;
                              ModRegs.MDFirmwareRev[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE3;
                              ModRegs.MDNorCapacity[5]    = ECanaMboxes.MBOX5.MDH.byte.BYTE4;
                              ModRegs.MDNorVolt[5]        = ECanaMboxes.MBOX5.MDH.byte.BYTE5;
                              ModRegs.PackMinVolteRec[5]  = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                  case (0x461):
                              ModRegs.MD46XRxcount[2]++;
                              ModRegs.MDCellMaxVolt[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE5,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                  case (0x462):

                              ModRegs.MD46XRxcount[3]++;
                              ModRegs.MDCellMaxTemps[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE5,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                  case (0x463):
                              ModRegs.MD46XRxcount[4]++;
                              ModRegs.MDTotalVolt[5]     = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[5]     = ECanaMboxes.MBOX5.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[5]     = ECanaMboxes.MBOX5.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[5]    = ECanaMboxes.MBOX5.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[5]    = ECanaMboxes.MBOX5.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[5]     = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                  break;
                #endif
                default:
                break;
            }
            ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
            ECanaShadow.CANRMP.all=0;
            ECanaShadow.CANRMP.bit.RMP5 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

        }
        if(ECanaRegs.CANRMP.bit.RMP6==1)
        {
            ModRegs.MD47XRxcount[0]++;
            SysRegs.MD7CANRxCount=0;
            switch(ECanaMboxes.MBOX6.MSGID.bit.STDMSGID)
            {
              #if(PackNum==1)
                case (0x170):
                            ModRegs.MD47XRxcount[1]++;
                            ModRegs.MDCellVoltQty[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE0;
                            ModRegs.MDCellTempsQty[6]   = ECanaMboxes.MBOX6.MDL.byte.BYTE1;
                            ModRegs.MDFirmwareVer[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE2;
                            ModRegs.MDFirmwareRev[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE3;
                            ModRegs.MDNorCapacity[6]    = ECanaMboxes.MBOX6.MDH.byte.BYTE4;
                            ModRegs.MDNorVolt[6]        = ECanaMboxes.MBOX6.MDH.byte.BYTE5;
                            ModRegs.PackMinVolteRec[6]  = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

                break;
                case (0x171):
                            ModRegs.MD47XRxcount[2]++;
                            ModRegs.MDCellMaxVolt[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                            ModRegs.MDCellMinVolt[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                            ModRegs.MDCellAgvVolt[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                            ModRegs.MDCellDivVolt[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

                break;
                case (0x172):

                            ModRegs.MD47XRxcount[3]++;
                            ModRegs.MDCellMaxTemps[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                            ModRegs.MDCellMinTemps[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                            ModRegs.MDCellAgvTemps[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                            ModRegs.MDCellDivTemps[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

                break;
                case (0x173):
                            ModRegs.MD47XRxcount[4]++;
                            ModRegs.MDTotalVolt[6]     = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                            ModRegs.MDMaxVoltPo[6]     = ECanaMboxes.MBOX6.MDL.byte.BYTE2;
                            ModRegs.MDMinVoltPo[6]     = ECanaMboxes.MBOX6.MDL.byte.BYTE3;
                            ModRegs.MDMaxTempsPo[6]    = ECanaMboxes.MBOX6.MDH.byte.BYTE4;
                            ModRegs.MDMinTempsPo[6]    = ECanaMboxes.MBOX6.MDH.byte.BYTE5;
                            ModRegs.MDstatusbit[6]     = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);
                break;
              #endif
            #if(PackNum==2)
              case (0x272):
                          ModRegs.MD47XRxcount[1]++;
                          ModRegs.MDCellVoltQty[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE0;
                          ModRegs.MDFirmwareVer[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE1;
                          ModRegs.MDNorVolt[6]        = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                          ModRegs.MDNorCapacity[6]    = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                          ModRegs.PackMinVolteRec[6]  = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

              break;
              case (0x273):
                          ModRegs.MD47XRxcount[2]++;
                          ModRegs.MDCellMaxVolt[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                          ModRegs.MDCellMinVolt[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                          ModRegs.MDCellAgvVolt[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                          ModRegs.MDCellDivVolt[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

              break;
              case (0x274):

                          ModRegs.MD47XRxcount[3]++;
                          ModRegs.MDCellMaxTemps[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                          ModRegs.MDCellMinTemps[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                          ModRegs.MDCellAgvTemps[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                          ModRegs.MDCellDivTemps[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

              break;
              case (0x275):
                          ModRegs.MD47XRxcount[4]++;
                          ModRegs.MDTotalVolt[6]     = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                          ModRegs.MDMaxVoltPo[6]     = ECanaMboxes.MBOX6.MDL.byte.BYTE2;
                          ModRegs.MDMinVoltPo[6]     = ECanaMboxes.MBOX6.MDL.byte.BYTE3;
                          ModRegs.MDMaxTempsPo[6]    = ECanaMboxes.MBOX6.MDH.byte.BYTE4;
                          ModRegs.MDMinTempsPo[6]    = ECanaMboxes.MBOX6.MDH.byte.BYTE5;
                          ModRegs.MDstatusbit[6]     = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);
              break;
            #endif
            #if(PackNum==3)
              case (0x370):
                          ModRegs.MD47XRxcount[1]++;
                          ModRegs.MDCellVoltQty[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE0;
                          ModRegs.MDCellTempsQty[6]   = ECanaMboxes.MBOX6.MDL.byte.BYTE1;
                          ModRegs.MDFirmwareVer[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE2;
                          ModRegs.MDFirmwareRev[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE3;
                          ModRegs.MDNorCapacity[6]    = ECanaMboxes.MBOX6.MDH.byte.BYTE4;
                          ModRegs.MDNorVolt[6]        = ECanaMboxes.MBOX6.MDH.byte.BYTE5;
                          ModRegs.PackMinVolteRec[6]  = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

              break;
              case (0x371):
                          ModRegs.MD47XRxcount[2]++;
                          ModRegs.MDCellMaxVolt[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                          ModRegs.MDCellMinVolt[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                          ModRegs.MDCellAgvVolt[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                          ModRegs.MDCellDivVolt[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

              break;
              case (0x372):

                          ModRegs.MD47XRxcount[3]++;
                          ModRegs.MDCellMaxTemps[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                          ModRegs.MDCellMinTemps[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                          ModRegs.MDCellAgvTemps[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                          ModRegs.MDCellDivTemps[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

              break;
              case (0x373):
                          ModRegs.MD47XRxcount[4]++;
                          ModRegs.MDTotalVolt[6]     = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                          ModRegs.MDMaxVoltPo[6]     = ECanaMboxes.MBOX6.MDL.byte.BYTE2;
                          ModRegs.MDMinVoltPo[6]     = ECanaMboxes.MBOX6.MDL.byte.BYTE3;
                          ModRegs.MDMaxTempsPo[6]    = ECanaMboxes.MBOX6.MDH.byte.BYTE4;
                          ModRegs.MDMinTempsPo[6]    = ECanaMboxes.MBOX6.MDH.byte.BYTE5;
                          ModRegs.MDstatusbit[6]     = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);
              break;
            #endif
            #if(PackNum==4)
              case (0x470):
                          ModRegs.MD47XRxcount[1]++;
                          ModRegs.MDCellVoltQty[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE0;
                          ModRegs.MDCellTempsQty[6]   = ECanaMboxes.MBOX6.MDL.byte.BYTE1;
                          ModRegs.MDFirmwareVer[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE2;
                          ModRegs.MDFirmwareRev[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE3;
                          ModRegs.MDNorCapacity[6]    = ECanaMboxes.MBOX6.MDH.byte.BYTE4;
                          ModRegs.MDNorVolt[6]        = ECanaMboxes.MBOX6.MDH.byte.BYTE5;
                          ModRegs.PackMinVolteRec[6]  = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

              break;
              case (0x471):
                          ModRegs.MD47XRxcount[2]++;
                          ModRegs.MDCellMaxVolt[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                          ModRegs.MDCellMinVolt[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                          ModRegs.MDCellAgvVolt[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                          ModRegs.MDCellDivVolt[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

              break;
              case (0x472):

                          ModRegs.MD47XRxcount[3]++;
                          ModRegs.MDCellMaxTemps[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                          ModRegs.MDCellMinTemps[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                          ModRegs.MDCellAgvTemps[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                          ModRegs.MDCellDivTemps[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

              break;
              case (0x473):
                          ModRegs.MD47XRxcount[4]++;
                          ModRegs.MDTotalVolt[6]     = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                          ModRegs.MDMaxVoltPo[6]     = ECanaMboxes.MBOX6.MDL.byte.BYTE2;
                          ModRegs.MDMinVoltPo[6]     = ECanaMboxes.MBOX6.MDL.byte.BYTE3;
                          ModRegs.MDMaxTempsPo[6]    = ECanaMboxes.MBOX6.MDH.byte.BYTE4;
                          ModRegs.MDMinTempsPo[6]    = ECanaMboxes.MBOX6.MDH.byte.BYTE5;
                          ModRegs.MDstatusbit[6]     = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);
              break;
            #endif
                default:
                break;

            }
            ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
            ECanaShadow.CANRMP.all=0;
            ECanaShadow.CANRMP.bit.RMP6 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

        }
        if(ECanaRegs.CANRMP.bit.RMP29==1)
        {
            CANARegs.MailBox1RxCount++;
            if(ECanaMboxes.MBOX29.MSGID.bit.STDMSGID==0x3C2)
            {
                SysRegs.CTRxCount=0;
                if(CANARegs.MailBox1RxCount>3000){CANARegs.MailBox1RxCount=0;}
                SysRegs.CurrentData.byte.CurrentH   = (ECanaMboxes.MBOX29.MDL.byte.BYTE0<<8)|(ECanaMboxes.MBOX29.MDL.byte.BYTE1);
                SysRegs.CurrentData.byte.CurrentL   = (ECanaMboxes.MBOX29.MDL.byte.BYTE2<<8)|(ECanaMboxes.MBOX29.MDL.byte.BYTE3);

            }
            ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
            ECanaShadow.CANRMP.all=0;
            ECanaShadow.CANRMP.bit.RMP29 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;
        }
        if(ECanaRegs.CANRMP.bit.RMP30==1)
        {
            CANARegs.MailBox2RxCount++;
            #if(PackNum==1)
            if(ECanaMboxes.MBOX30.MSGID.bit.STDMSGID==0x700)
            {
                SysRegs.MasterRxCount=0;
               if(CANARegs.MailBox1RxCount>3000){CANARegs.MailBox1RxCount=0;}
               CANARegs.PMSCMDRegs.all = (ECanaMboxes.MBOX30.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX30.MDL.byte.BYTE0);
              if(CANARegs.PMSCMDRegs.bit.PrtctReset01==1)
              {
                   CANARegs.PMSCMDRegs.bit.RUNStatus01=0;
              }
               SysRegs.SysCanRxCount=0;
            }
            #endif
            #if(PackNum==2)
            if(ECanaMboxes.MBOX30.MSGID.bit.STDMSGID==0x701)
            {
                SysRegs.MasterRxCount=0;
               if(CANARegs.MailBox1RxCount>3000){CANARegs.MailBox1RxCount=0;}
               CANARegs.PMSCMDRegs.all = (ECanaMboxes.MBOX30.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX30.MDL.byte.BYTE0);
               CANARegs.PMSCMDRegs.all = (ECanaMboxes.MBOX30.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX30.MDL.byte.BYTE0);
               CANARegs.PMSCMDRegs.all = (ECanaMboxes.MBOX30.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX30.MDL.byte.BYTE0);
              if(CANARegs.PMSCMDRegs.bit.PrtctReset01==1)
              {
                   CANARegs.PMSCMDRegs.bit.RUNStatus01=0;
              }
               SysRegs.SysCanRxCount=0;
            }
            #endif
            #if(PackNum==3)
            if(ECanaMboxes.MBOX30.MSGID.bit.STDMSGID==0x702)
            {
                SysRegs.MasterRxCount=0;
               if(CANARegs.MailBox1RxCount>3000){CANARegs.MailBox1RxCount=0;}
               CANARegs.PMSCMDRegs.all = (ECanaMboxes.MBOX30.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX30.MDL.byte.BYTE0);
              if(CANARegs.PMSCMDRegs.bit.PrtctReset01==1)
              {
                   CANARegs.PMSCMDRegs.bit.RUNStatus01=0;
              }
               SysRegs.SysCanRxCount=0;
            }
            #endif
            #if(PackNum==4)
            if(ECanaMboxes.MBOX30.MSGID.bit.STDMSGID==0x703)
            {
                SysRegs.MasterRxCount=0;
               if(CANARegs.MailBox1RxCount>3000){CANARegs.MailBox1RxCount=0;}
               CANARegs.PMSCMDRegs.all = (ECanaMboxes.MBOX30.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX30.MDL.byte.BYTE0);
              if(CANARegs.PMSCMDRegs.bit.PrtctReset01==1)
              {
                   CANARegs.PMSCMDRegs.bit.RUNStatus01=0;
              }
               SysRegs.SysCanRxCount=0;
            }
            #endif
            ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
            ECanaShadow.CANRMP.all=0;
            ECanaShadow.CANRMP.bit.RMP30 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

        }
    }
    /*
     *
     */
   /* ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
    ECanaShadow.CANRMP.all = 0;
    ECanaShadow.CANRMP.bit.RMP0  = 1;  // 0x5N1 Response Data
    ECanaShadow.CANRMP.bit.RMP1  = 1;  // 0x5N2 Response Data
    ECanaShadow.CANRMP.bit.RMP2  = 1;  // 0x5N3 Response Data
    ECanaShadow.CANRMP.bit.RMP3  = 1;  // 0x5N3 Response Data
    ECanaShadow.CANRMP.bit.RMP4  = 1;  // 0x5N3 Response Data
    ECanaShadow.CANRMP.bit.RMP5  = 1;  // 0x5N3 Response Data
    ECanaShadow.CANRMP.bit.RMP6  = 1;  // 0x5N3 Response Data
    ECanaShadow.CANRMP.bit.RMP7  = 1;  // 0x5N3 Response Data
    ECanaShadow.CANRMP.bit.RMP8  = 1;  // 0x5N3 Response Data
    ECanaShadow.CANRMP.bit.RMP9  = 1;  // 0x5N3 Response Data
    ECanaShadow.CANRMP.bit.RMP29 = 1;  // 0x60A Response Data
    ECanaShadow.CANRMP.bit.RMP30 = 1;  // 0x3C2 Current Sensor Response Data
    ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;
    */
   /*
    *
    */
    ECanaShadow.CANME.all=ECanaRegs.CANME.all;
    ECanaShadow.CANME.bit.ME0=1;    //0x5NA MCU Rx Enable
    ECanaShadow.CANME.bit.ME1=1;    //0x5NA MCU Rx Enable
    ECanaShadow.CANME.bit.ME2=1;    //0x5NB MCU Rx Enable
    ECanaShadow.CANME.bit.ME3=1;    //0x5NC MCU Rx Enable
    ECanaShadow.CANME.bit.ME4=1;    //0x5ND MCU Rx Enable
    ECanaShadow.CANME.bit.ME5=1;    //0x5ND MCU Rx Enable
    ECanaShadow.CANME.bit.ME6=1;    //0x5ND MCU Rx Enable
    ECanaShadow.CANME.bit.ME7=1;    //0x5ND MCU Rx Enable
    ECanaShadow.CANME.bit.ME8=1;    //0x5ND MCU Rx Enable
    ECanaShadow.CANME.bit.ME29=1;   //0x60A RTC init Rx Enable
    ECanaShadow.CANME.bit.ME30=1;   //0x3C2 MCU Rx Enable
    ECanaShadow.CANME.bit.ME31=1;   //CAN-A Tx Enable
    ECanaRegs.CANME.all=ECanaShadow.CANME.all;


    /*
     *
     */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    ///IER |= 0x0100;                  // Enable INT9
    //EINT;

}//EOF
/*
interrupt void cpu_timer2_isr(void)
{  EALLOW;
   CpuTimer2.InterruptCount++;
   // The CPU acknowledges the interrupt.
  // A_OVCHACurrent;
   EDIS;
}
*/

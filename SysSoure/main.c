

/**
 * main.c
 */

#include "DSP28x_Project.h"
#include "parameter.h"
#include "SysVariable.h"
#include "ProtectRelay.h"
#include "BATAlgorithm.h"
#include "BATCalc.h"
//#include "BATCellModel.h"
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

//void BatteryModel_1msTask();
//void BatteryModel_UserInit();

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
//void ProtectRlySateCheck(PrtectRelayReg *P);
void ProtectRlyVarINIT(PrtectRelayReg *P);
void ProtectRelayHandle(PrtectRelayReg *P);
void ProtectRlyOnInit(PrtectRelayReg *P);
void ProtectRlyOffInit(PrtectRelayReg *P);



void CalKokam100AhRegsInit(SocReg *P);
void CalKokam100AhSocInit(SocReg *P);
void Calkokam100AhSocHandle(SocReg *P);

/*
 *
 */
void SysCurrentHandle(SystemReg *s);
void SysCommErrHandle(SystemReg *P);
void SysUnitBMSStatus(SystemReg *P);
/*
 *
 */
void CalSysAlarmtCheck(SystemReg *s);
void CalSysFaultCheck(SystemReg *s);
void CalSysProtectCheck(SystemReg *s);
/*
 *
 */
void BatCalcRegsInit(BatCalcReg *P);
void BatCalcVoltHandle(BatCalcReg *P);
void BatCalcTempsHandle(BatCalcReg *P);
void BatCalcIRHandle(BatCalcReg *P);
/*
 *
 */
int float32ToInt(float32 Vaule, Uint32 Num);
/*
 *
 */
float32 GetDischargeLimit30s_A(float32 tempC_f, float32 socPct_f);
float32 GetDischargeLimitCont_A(float32 tempC_f, float32 socPct_f);
float32 GetChargeLimit10s_A(float32 tempC_f, float32 socPct_f);
float32 GetChargeLimitCont_A(float32 tempC_f, float32 socPct_f);
/*
 *
 */
void TestCurrentlimt_StepUpdate_200ms(SystemReg *p);

/*
 *
 */
float RandVaule(float inputVaule);
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
  //  InitSpiGpio();
  ///  InitSpiCAN();
    // TEST

    InitECanGpio();
   // InitECana();

    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    InitFlash();

    ConfigCpuTimer(&CpuTimer0, 80, 1000);
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
    PrtectRelayRegs.StateMachine=STATERly_INIT;
    SysRegs.PackStateReg.bit.CANCOMEnable=0;
    SysRegs.PackStateReg.bit.INITOK=0;
   // gBattery_InitSocPct=20.0f;
    //gBattery_CapacityAh=80.0f;
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
                  // TODOS : [완료] (01, 0x60A 멀티플렉서용 모듈 RxCount 포인터 1회 설정)
                  CANARegs.MDXRxcountP[0]=ModRegs.MD1XRxcount; CANARegs.MDXRxcountP[1]=ModRegs.MD2XRxcount;
                  CANARegs.MDXRxcountP[2]=ModRegs.MD3XRxcount; CANARegs.MDXRxcountP[3]=ModRegs.MD4XRxcount;
                  CANARegs.MDXRxcountP[4]=ModRegs.MD5XRxcount; CANARegs.MDXRxcountP[5]=ModRegs.MD6XRxcount;
                  CANARegs.MDXRxcountP[6]=ModRegs.MD7XRxcount;
                //  gBattery_InitSocPct=20.0f;
               //   gBattery_CapacityAh=80.0f;
                  BatCalcRegsInit(&BatCalcRegs);
                  ProtectRlyVarINIT(&PrtectRelayRegs);
                //  BatteryModel_UserInit();
                  // Function SysMachine
                  CalKokam100AhRegsInit(&Kam100AHSocRegs);
                  Kam100AHSocRegs.state=SOC_STATE_IDLE;
                  PrtectRelayRegs.StateMachine=STATERly_INIT;
                  SysRegs.SysMachine=System_STATE_STANDBY;
                  DigitalInput(&SysRegs);
                  delay_ms(100);
                  InitECana();
                  if(SysRegs.PackStateReg.bit.SysPrtct==1)
                  {
                    //SysRegs.SysMachine=System_STATE_PROTECTER;
                  }
                  SysRegs.PackStateReg.bit.SysSeqState=1;
                  SysRegs.PackID =0x0000;
               //   SysRegs.PackID==0x0000;
               //   InitECana();
                  delay_ms(10);

            break;
            case System_STATE_STANDBY:
                  SysRegs.DigitalOutPutReg.bit.PWRLAMPOUT=0;
                  PrtectRelayRegs.StateMachine=STATERly_STANDBY;
                  if(SysRegs.PackStateReg.bit.SysPrtct==1)
                  {
                    //  SysRegs.PackStateReg.bit.INITOK=1;
                  }
                  delay_ms(1000);
                  // Function SysMachine
                  /* 셀전압이 유효해진 시점(STANDBY)에서 전압 기반 초기 SOC 1회 산정 */
                  CalKokam100AhRegsInit(&Kam100AHSocRegs);
                  Kam100AHSocRegs.CellAgvVoltageF = SysRegs.PackCellAgvVoltageF;
                  CalKokam100AhSocInit(&Kam100AHSocRegs);
                  Kam100AHSocRegs.state= SOC_STATE_RUNNING;
                  SysRegs.SysMachine=System_STATE_READY;
                  PrtectRelayRegs.StateMachine=STATERly_Ready;
                  SysRegs.PackStateReg.bit.CANCOMEnable=1;
                  SysRegs.PackStateReg.bit.INITOK=1;
                  SysRegs.PackStateReg.bit.SysSeqState=2;
            break;
            case System_STATE_READY:
                 SysRegs.PackStateReg.bit.CANCOMEnable=1;
                 SysRegs.DigitalOutPutReg.bit.PWRLAMPOUT=0;
                 if(CANARegs.PMSCMDRegs.bit.RUNStatus01==1)
                 {
                     PrtectRelayRegs.State.bit.WakeUpEN=1;
                 }
                 // Function SysMachine
                 if(PrtectRelayRegs.State.bit.WakeuPOnEND==1)
                 {
                     SysRegs.SysMachine=System_STATE_RUNING;
                 }
                   SysRegs.PackStateReg.bit.SysSeqState = 3;
            break;
            case System_STATE_RUNING:
                 Kam100AHSocRegs.state= SOC_STATE_RUNNING;
                 SysRegs.PackStateReg.bit.CANCOMEnable=1;
                 SysRegs.DigitalOutPutReg.bit.PWRLAMPOUT=1;
                 /*--------------------------------------------------------------
                  * 260618 : RUNING 정상 구간에서 SysSeqState가 갱신되지 않아
                  *          READY 값(3)이 남던 문제. 진입 시 항상 4(Run) 부여.
                  *--------------------------------------------------------------*/
                 SysRegs.PackStateReg.bit.SysSeqState = 4;   // RUNING 진입 시 항상 Run 표시   // TODOS : [검증] (90, 260618_Note2, RUNING 상시 SysSeqState=4)
                 if(CANARegs.PMSCMDRegs.bit.RUNStatus01==0)
                 {
                     PrtectRelayRegs.State.bit.WakeUpEN=0;
                     //SysRegs.PackStateReg.bit.SysSeqState = 4;   // 260618 : 위로 이동(상시 부여)
                 }
                 if(PrtectRelayRegs.State.bit.WakeuPOffEND==1)
                 {
                     SysRegs.SysMachine=System_STATE_READY;

                 }
                 
               //  SysRegs.PackStateReg.bit.SysSTATE = 4;
            break;
            case System_STATE_PROTECTER:
                 SysRegs.PackStateReg.bit.CANCOMEnable=1;
                 CANARegs.PMSCMDRegs.all=0;
                 if(CANARegs.PMSCMDRegs.bit.PrtctReset01==1)
                 {
                     CANARegs.PMSCMDRegs.bit.PrtctReset01=0;
                     SysTimerINIT(&SysRegs);
                     SysVarINIT(&SysRegs);
                     CANRegVarINIT(&CANARegs);
                   //  ModuleInit(&ModRegs);
                     BatCalcRegsInit(&BatCalcRegs);
                     ProtectRlyVarINIT(&PrtectRelayRegs);
                     CalKokam100AhRegsInit(&Kam100AHSocRegs);
                     delay_ms(200);
                     SysRegs.SysMachine=System_STATE_INIT;
                 }
                 SysRegs.PackStateReg.bit.SysSeqState = 5;
            break;
            case System_STATE_DATALOG:
            //     SysRegs.PackStateReg.bit.CANCOMEnable=1;
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
         //   MCP2515InitHandle(&CANBRegs);
            CANTXFALAG=0;
       }
        PrtectRelayRegs.State.bit.NRlyDI=SysRegs.DigitalInputReg.bit.NAUX;
        PrtectRelayRegs.State.bit.PRlyDI=SysRegs.DigitalInputReg.bit.PAUX;
        ProtectRelayHandle(&PrtectRelayRegs);
        SysRegs.DigitalOutPutReg.bit.NRlyOUT=PrtectRelayRegs.State.bit.NRlyDO;
        SysRegs.DigitalOutPutReg.bit.PRlyOUT=PrtectRelayRegs.State.bit.PRlyDO;
        SysRegs.DigitalOutPutReg.bit.ProRlyOUT=PrtectRelayRegs.State.bit.PreRlyDO;
        SysRegs.PackProtectReg.bit.PackRlyErr=PrtectRelayRegs.State.bit.RlyFaulttSate;
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
 //  BatteryModel_1msTask();
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
 /*  if(SysRegs.PackStateReg.bit.CANRXCountReset==1)
   {
      // ModuleInit(&ModRegs);
     ///  SysRegs.PackStateReg.bit.CANRXCountReset=0;
   }
   */
   /*
    * DigitalInput detection
    */
   DigitalInput(&SysRegs);
   //InitECan();
   /*
    *
    */
  /*
   * current sensing detection
  */
   SysCurrentHandle(&SysRegs);
   /*
    *
    */

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
   /*
    * 에러 검출
    */
  // SysCommErrHandle(&SysRegs);
 //  if(SysRegs.DigitalInputReg.bit.EMGSWStauts==1)
 //  {
  //     SysRegs.PackProtectReg.bit.PackEMSSWErr=1;
  // }
  // else
  // {
  //     SysRegs.PackProtectReg.bit.PackEMSSWErr=0;
  // }
  // CANARegs.PackProtetSate=0;
     SysRegs.PackFaultReg.all=0;
     SysRegs.PackProtectReg.all=0;
   if(SysRegs.PackStateReg.bit.INITOK==1)
   {
      // TODOS : [완료] (02, R01 경고/Fault/보호 판정 활성화 (1ms ISR 주기))
      CalSysAlarmtCheck(&SysRegs);
     // CalSysFaultCheck(&SysRegs);
     // CalSysProtectCheck(&SysRegs);
   }
 //  CalSysAlarmtCheck(&SysRegs);
   if((SysRegs.PackAlarmReg.all > 0)&&(SysRegs.PackStateReg.bit.INITOK==1))
   {
       SysRegs.PackStateReg.bit.SysAalarm=1;
       CANARegs.PackProtetSate=1;
   }
   else
   {
     SysRegs.PackStateReg.bit.SysAalarm=0;
     CANARegs.PackProtetSate=0;
   }

   if((SysRegs.PackFaultReg.all >0)&&(SysRegs.PackStateReg.bit.INITOK==1))
   {
     SysRegs.PackStateReg.bit.SysFault=1;
     CANARegs.PackProtetSate=2;
   }
   else
   {
       SysRegs.PackStateReg.bit.SysFault=0;
       CANARegs.PackProtetSate=0;
   }
   if((SysRegs.PackProtectReg.all> 0)&&(SysRegs.PackStateReg.bit.INITOK==1))
   {
       SysRegs.PackStateReg.bit.SysPrtct=1;
       CANARegs.PackProtetSate=3;
   }
   switch(SysRegs.SysRegTimer5msecCount)
   {
       case 1:
               if(CANARegs.PMSCMDRegs.bit.PrtctReset01==1)
               {
                   SysRegs.SysMachine=System_STATE_INIT;
               }
       break;
       default :
       break;

   }
   switch(SysRegs.SysRegTimer10msecCount)
   {
       case 1:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   SysRegs.PackSOHF = 100.0;
                   CANARegs.PackPT = (unsigned int)(SysRegs.PackVoltageF*10);
                   CANARegs.PackCT = (int)(SysRegs.PackCurrentF*10.0);
                   CANARegs.PackSOC =(int)(SysRegs.PackSOCF*10);
                   CANARegs.PackSOH =(unsigned int)(SysRegs.PackSOHF*10);
                   // TODOS : [완료] (03, 0x601 BPA_Calc1 (PackPT/PackCT/PackSOC/PackSOH 각 ×10))
                   CANARegs.PackID =0X601|SysRegs.PackID;
                   CANATX(CANARegs.PackID ,8,CANARegs.PackPT,CANARegs.PackCT,CANARegs.PackSOC,CANARegs.PackSOH);
               }
       break;
       case 2:

       break;
       case 5:
       break;
       case 7:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                     // TODOS : [완료] (05, 0x602 BPA_State 비트맵 재배치 (Balance/Aux 릴레이/EMG/Water_Leak/OffGas))
                     /*--------------------------------------------------------------
                      * 260615 : 0x602 SB0~31 R59 정렬
                      *   - Divice_Status(SB0~7): SysSeqState→Rack상태 매핑
                      *   - Protect_Status(SB8~15): Prtct>Fault>Alarm 우선순위(3/2/1/0)
                      *   - PackStatus(SB16~22): MSD/EMG/Water를 SB20~22로, Aux3·OffGas 제거
                      *--------------------------------------------------------------*/
                     //CANARegs.PackSate                        = SysRegs.PackStateReg.bit.SysSeqState;
                     //CANARegs.PackProtetSate                  = 0;
                     /*--------------------------------------------------------------
                      * 260618 : case 번호를 실제 SysSeqState(1=INIT,2=STANDBY,3=READY,
                      *          4=RUNING,5=PROTECTER)에 정렬. 기존은 한 칸 밀려
                      *          READY가 Run으로, OFF복귀가 Protect로 잘못 표시됨.
                      *--------------------------------------------------------------*/
                     //case 1:  CANARegs.PackSate = 2; break;   // STANDBY   -> Standby
                     //case 2:  CANARegs.PackSate = 3; break;   // READY     -> Ready
                     //case 3:  CANARegs.PackSate = 4; break;   // RUNING    -> Run
                     //case 4:  CANARegs.PackSate = 5; break;   // PROTECTER -> Protect
                     switch(SysRegs.PackStateReg.bit.SysSeqState)   // map internal seq to R59 device state   // TODOS : [검증] (74, 0x602 Divice_Status 매핑)
                     {
                         case 1:  CANARegs.PackSate = 2; break;   // INIT      -> Standby (기존 유지)   // TODOS : [검증] (89, 260618_Note2, SysSeqState case 정렬)
                         case 2:  CANARegs.PackSate = 2; break;   // STANDBY   -> Standby
                         case 3:  CANARegs.PackSate = 3; break;   // READY     -> Ready
                         case 4:  CANARegs.PackSate = 4; break;   // RUNING    -> Run
                         case 5:  CANARegs.PackSate = 5; break;   // PROTECTER -> Protect
                         default: CANARegs.PackSate = 0; break;   // 그외      -> None
                     }
                     if(SysRegs.PackStateReg.bit.SysPrtct)       { CANARegs.PackProtetSate = 3; }   // Protect   // TODOS : [검증] (75, 0x602 Protect_Status 단계)
                     else if(SysRegs.PackStateReg.bit.SysFault)  { CANARegs.PackProtetSate = 2; }   // Fault
                     else if(SysRegs.PackStateReg.bit.SysAalarm) { CANARegs.PackProtetSate = 1; }   // Alarm
                     else                                        { CANARegs.PackProtetSate = 0; }   // None
                     CANARegs.PackSateInfo                    = ComBine(CANARegs.PackProtetSate, CANARegs.PackSate);
                     CANARegs.PackStatus.bit.PackBalance      = SysRegs.PackStateReg.bit.SysBalanceEn;   // TODOS : [검증] (73, 0x602 PackStatus R59 정렬)
                     CANARegs.PackStatus.bit.PackNeg_Rly      = PrtectRelayRegs.State.bit.NRlyDI;
                     CANARegs.PackStatus.bit.PackPos_Rly      = PrtectRelayRegs.State.bit.PRlyDI;
                     CANARegs.PackStatus.bit.PackPreChar_Rly  = PrtectRelayRegs.State.bit.ProRlyDI;
                     //CANARegs.PackStatus.bit.PackNeg_Rly_Aux  = PrtectRelayRegs.State.bit.NRlyDI;   // (R59 삭제)
                     //CANARegs.PackStatus.bit.PackPos_Rly_Aux  = PrtectRelayRegs.State.bit.PRlyDI;   // (R59 삭제)
                     //CANARegs.PackStatus.bit.PackPreChar_AUX  = PrtectRelayRegs.State.bit.ProRlyDI; // (R59 삭제)
                     CANARegs.PackStatus.bit.PackMSD_AUX      = SysRegs.PackStateReg.bit.MSDERR;          // SB20
                     CANARegs.PackStatus.bit.PackEmg_STOP_SW  = SysRegs.DigitalInputReg.bit.EMGSWStauts;  // SB21
                     // TODOS : [완료] (07, 0x602 PackWater_Leak = SysUnitBMSStatus() 7모듈 WaterleakFault OR 종합 결과 사용)
                     CANARegs.PackStatus.bit.PackWater_Leak   = SysRegs.PackModuleAllRegs.bit.WaterleakFault; // SB22
                     //CANARegs.PackStatus.bit.PackOffGas       = 0;   // (R59 0x602 미정의)
                     // TODOS : [완료] (06, 0x602 bit56 Sys_BATIC_Err = SysUnitBMSStatus() 7모듈 BATICErrFault OR 종합 결과 사용)
                     SysRegs.PackStateReg.bit.SysBATIC_Err    = SysRegs.PackModuleAllRegs.bit.BATICErrFault;
                     CANARegs.PackID =0X602|SysRegs.PackID;
                     CANATX(CANARegs.PackID ,8,CANARegs.PackSateInfo,CANARegs.PackStatus.all,SysRegs.PackStateReg.Word.DataL,SysRegs.PackStateReg.Word.DataH);
               }
       break;
       default :
       break;
   }
   switch(SysRegs.SysRegTimer50msecCount)
   {
       case 1:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                  // TODOS : [완료] (07, 0x500 BPA_Module 모듈별 밸런스/보호 상태)
                  CANARegs.PackID =0X500|(SysRegs.PackID+0x0010);
                  CANATX(CANARegs.PackID,8,0X00,0X00,CANARegs.PackTemperatureAVG,CANARegs.PackBalanVolt);
               }
       break;
       case 5:
                // TODOS : [완료] (07, 모듈 동작상태 검출)
                SysRegs.PackModuleRegs[0].all = ModRegs.MDstatusbit[0];
                SysRegs.PackModuleRegs[1].all = ModRegs.MDstatusbit[1];
                SysRegs.PackModuleRegs[2].all = ModRegs.MDstatusbit[2];
                SysRegs.PackModuleRegs[3].all = ModRegs.MDstatusbit[3];
                SysRegs.PackModuleRegs[4].all = ModRegs.MDstatusbit[4];
                SysRegs.PackModuleRegs[5].all = ModRegs.MDstatusbit[5];
                SysRegs.PackModuleRegs[6].all = ModRegs.MDstatusbit[6];
                SysUnitBMSStatus(&SysRegs);
       break;
       case 10:
               if(SysRegs.PackStateReg.bit.INITOK==1)
               {
                   SysRegs.PackDisCHAPWRPeakF       = GetDischargeLimit30s_A(SysRegs.PackCellAgvTemperatureF, SysRegs.PackSOCF);
                   SysRegs.PackDisCHAPWRContintyF   = GetDischargeLimitCont_A(SysRegs.PackCellAgvTemperatureF, SysRegs.PackSOCF);
                   SysRegs.PackCHAPWRPeakF          = GetChargeLimit10s_A(SysRegs.PackCellAgvTemperatureF, SysRegs.PackSOCF);
                   SysRegs.PackCHAPWRContintyF      = GetChargeLimitCont_A(SysRegs.PackCellAgvTemperatureF, SysRegs.PackSOCF);
               }
       break;
       case 20:
               // TODOS : [완료] (08, 셀 온도 상태 계산)
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
       break;
       case 30 :
               // TODOS : [완료] (04, 셀 전압 상태 계산)
               memcpy(&BatCalcRegs.MDCellMaxVolt[0], &ModRegs.MDCellMaxVolt[0],sizeof(Uint16)*7);
               memcpy(&BatCalcRegs.MDCellMinVolt[0], &ModRegs.MDCellMinVolt[0],sizeof(Uint16)*7);
               memcpy(&BatCalcRegs.MDTotalVolt[0],   &ModRegs.MDTotalVolt[0],sizeof(Uint16)*7);
               memcpy(&BatCalcRegs.MDMaxVoltPo[0],   &ModRegs.MDMaxVoltPo[0],sizeof(Uint16)*7);
               memcpy(&BatCalcRegs.MDMinVoltPo[0],   &ModRegs.MDMinVoltPo[0],sizeof(Uint16)*7);
               BatCalcVoltHandle(&BatCalcRegs);
               SysRegs.PackVoltageF= BatCalcRegs.PackPTCANF;
               SysRegs.PackCellMaxVoltageF= BatCalcRegs.PackCellMaxVoltF;
               // TODOS : [검증] (66, 셀 최소전압 임시코드(Max-5mV) 제거 → 실제 계산값 PackCellMinVoltF 반영)
               SysRegs.PackCellMinVoltageF= BatCalcRegs.PackCellMinVoltF;
               SysRegs.PackCellAgvVoltageF= BatCalcRegs.PackCellAgvVoltF;
               SysRegs.PackCellDivVoltageF= BatCalcRegs.PackCellDivVoltF;
               SysRegs.PackCellMaxVoltPos = BatCalcRegs.PackCellMaxVoltPos;
               SysRegs.PackCellMinVoltPos = BatCalcRegs.PackCellMinVoltPos;

       break;
       default :
       break;
   }

   switch(SysRegs.SysRegTimer100msecCount)
   {

       case 1:

       break;
       case 3:
       break;

       case 5:

       break;
       case 8:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   // TODOS : [완료] (09, 0x603 BPA_Prtct (Alarm.DataL/Fault.DataL/Protect.DataL/Protect.DataH))
                   CANARegs.PackID =0X603|SysRegs.PackID;
                   CANATX(CANARegs.PackID,8,SysRegs.PackAlarmReg.Word.DataL,SysRegs.PackFaultReg.Word.DataL,SysRegs.PackProtectReg.Word.DataL,SysRegs.PackProtectReg.Word.DataH);
               }
       break;
       case 11:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                    CANARegs.PackCHAPWRContinty    = (unsigned int)(SysRegs.PackCHAPWRContintyF*10);
                    CANARegs.PackDisCHAPWRContinty = (unsigned int)(SysRegs.PackDisCHAPWRContintyF*10);
                    CANARegs.PackCHAPWRPeak        = (unsigned int)(SysRegs.PackCHAPWRPeakF*10);
                    CANARegs.PackDisCHAPWRPeak     = (unsigned int)(SysRegs.PackDisCHAPWRPeakF*10);
                    // TODOS : [완료] (10, 0x604 BPA_PWRCalc (충/방전 연속·피크 전력 각 ×10))
                    CANARegs.PackID =0X604|SysRegs.PackID;
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
                   // TODOS : [완료] (11, 0x605 BPA_Cell_Vcalc (셀전압 Max/Min/Avg/Div 각 ×1000))
                   CANARegs.PackID =0X605|SysRegs.PackID;
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
                    // TODOS : [완료] (12, 0x606 BPA_Cell_Tcalc (셀온도 Max/Min/Avg/Div 각 ×10))
                    CANARegs.PackID =0X606|SysRegs.PackID;
                    CANATX(CANARegs.PackID,8,CANARegs.PackTemperaturelMAX,CANARegs.PackTemperaturelMIN,CANARegs.PackTemperatureAVG,CANARegs.PackTemperatureDiv);
                }
       break;
       case 20:

                if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                {
                    /*--------------------------------------------------------------
                     * 260615 : 0x607 용도변경 IRcalc→Cell_Pocalc (R59)
                     *          셀 전압/온도 Max·Min 위치값(각 16bit, 1~168)
                     *--------------------------------------------------------------*/
                    //CANARegs.CellIRMAX     = (unsigned int)(SysRegs.PackCellMaxIRF*10);
                    //CANARegs.CellIRlMIN    = (unsigned int)(SysRegs.PackCellMinIRF*10);
                    //CANARegs.CellIRAVG     = (unsigned int)(SysRegs.PackCellAgvIRF*10);
                    //CANARegs.CellIRDiv     = (unsigned int)(SysRegs.PackCellDivIRF*10);
                    // TODOS : [완료] (13, 0x607 BPA_Cell_IRcalc (셀IR Max/Min/Avg/Div 각 ×10))
                    CANARegs.PackID =0X607|SysRegs.PackID;
                    //CANATX(CANARegs.PackID,8,CANARegs.CellIRMAX,CANARegs.CellIRlMIN,CANARegs.CellIRAVG,CANARegs.CellIRDiv);
                    CANATX(CANARegs.PackID,8,
                           SysRegs.PackCellMaxVoltPos,     // MaxVoltNum (SB0)    // TODOS : [검증] (81, 0x607 IR→셀위치 Pocalc)
                           SysRegs.PackCellMinVoltPos,     // MinVoltNum (SB16)
                           SysRegs.PackCellMaxTmepsPos,    // MaxTempNum (SB32)
                           SysRegs.PackCellMinTmepsPos);   // MinTempNum (SB48)
                }
       break;
       case 23:
                /*--------------------------------------------------------------
                 * 260615 : 옛 0x608 위치값은 0x607(Cell_Pocalc)로 이전, 0x608은 모듈Info1(case1)로 이동
                 *          → case23 비활성 (위치값 중복/ID 충돌 방지)
                 *--------------------------------------------------------------*/
                // TODOS : [삭제] (83, 옛 0x608 위치값 폐기 - 0x607로 이전, 0x608은 모듈Info1로 재편)
                //if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                //{
                //    CANARegs.PackVoltageMaxMinNum       = ComBine(SysRegs.PackCellMinVoltPos,SysRegs.PackCellMaxVoltPos);
                //    CANARegs.PackTemperatureMaxMinNum   = ComBine(SysRegs.PackCellMinTmepsPos,SysRegs.PackCellMaxTmepsPos);
                //    CANARegs.PackIRMaxMinNUM            = ComBine(SysRegs.PackCellMinIRPos,SysRegs.PackCellMaxIRPos);
                //    CANARegs.PackID =0X608|SysRegs.PackID;
                //    CANATX(CANARegs.PackID,8,CANARegs.PackVoltageMaxMinNum,CANARegs.PackTemperatureMaxMinNum,CANARegs.PackIRMaxMinNUM,0X000);
                //}
       break;
       case 26:

       break;
       case 30:

       break;
       case 35:

       break;
       case 38:

       break;
       default:
       break;
   }
   switch(SysRegs.SysRegTimer300msecCount)
   {
       case 1:
                if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   /*--------------------------------------------------------------
                    * 260615 : 0x609→0x608 모듈Info1 (R59) - ID 변경 / Data3 PackMinVolteRec→0(미사용)
                    *          ModuleNum + TotalVolt + 상태비트(SysMDstatus, SB32~42)
                    *--------------------------------------------------------------*/
                   CANARegs.MDTotalVolt[CANARegs.MDNumCountA]        = (Uint16)((float32)ModRegs.MDTotalVolt[CANARegs.MDNumCountA]*0.1);
                   CANARegs.MDstatusbit[CANARegs.MDNumCountA]        = SysRegs.PackModuleRegs [CANARegs.MDNumCountA].all;
                   //CANARegs.PackMinVolteRec[CANARegs.MDNumCountA]    = ModRegs.PackMinVolteRec[CANARegs.MDNumCountA];   // R59 0x608 미사용
                   // TODOS : [검증] (15, 0x609 BM_State 멀티플렉서 (BMNum/MDTotalVolt/MDstatusbit 16bit/PackMinVolteRec, MDNumCountA 0~6 순환))
                   //CANARegs.PackID =0X609|SysRegs.PackID;
                   CANARegs.PackID =0X608|SysRegs.PackID;   // TODOS : [검증] (82, 0x608 모듈Info1: ModuleNum/TotalVolt/상태비트)
                   //CANATX(CANARegs.PackID,8,CANARegs.MDNumCountA,CANARegs.MDTotalVolt[CANARegs.MDNumCountA],CANARegs.MDstatusbit[CANARegs.MDNumCountA],CANARegs.PackMinVolteRec[CANARegs.MDNumCountA]);
                   CANATX(CANARegs.PackID,8,CANARegs.MDNumCountA,CANARegs.MDTotalVolt[CANARegs.MDNumCountA],CANARegs.MDstatusbit[CANARegs.MDNumCountA],0);
                   if(++CANARegs.MDNumCountA>=7)
                   {
                       CANARegs.MDNumCountA=0;
                   }
               }
               else
               {
                   CANARegs.MDNumCountA=0;
               }
       break;
       case 5:
                if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                {
                   /*--------------------------------------------------------------
                    * 260616 : 0x609 모듈Info2 (R59) - MDNumCountB 독립 멀티플렉서
                    *          셀전압 Max/Min/Avg (ModRegs Uint16, factor 0.001, mV)
                    *          0x60A 셀온도(case15)와 대칭 구조
                    *--------------------------------------------------------------*/
                   CANARegs.PackID =0X609|SysRegs.PackID;   // TODOS : [검증] (94, 0x609 모듈Info2: 셀전압 Max/Min/Avg)
                   CANATX(CANARegs.PackID,8,CANARegs.MDNumCountB,ModRegs.MDCellMaxVolt[CANARegs.MDNumCountB],ModRegs.MDCellMinVolt[CANARegs.MDNumCountB],ModRegs.MDCellAgvVolt[CANARegs.MDNumCountB]);
                   if(++CANARegs.MDNumCountB>=7)
                   {
                       CANARegs.MDNumCountB=0;
                   }
                }
                else
                {
                   CANARegs.MDNumCountB=0;
                }
       break;
       case 10:
                /*--------------------------------------------------------------
                 * 260616 : 300ms 송신 ID 오름차순 재배치 (0x608~0x60F 순서대로)
                 *          1=0x608(상태) 5=0x609(전압) 10=0x60A(온도) 15=0x60B(편차)
                 *          20=0x60C(RxCnt) 30=0x60D(Dug1) 40=0x60E(디버그) 50=0x60F(예비)
                 *          (구) 10=0x60D 15=0x60A 20=0x60B 30=0x60C 40=0x60D(중복) 50=0x60E 60=0x60F
                 *--------------------------------------------------------------*/
                if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                {
                   /* 0x60A 모듈Info3 셀온도 Max/Min/Avg (MDNumCountD, 원시 int16, factor 0.1) */
                   CANARegs.PackID =0X60A|SysRegs.PackID;   // TODOS : [검증] (95, 0x60A 셀온도 - 재배치 case15->10)
                   CANATX(CANARegs.PackID,8,CANARegs.MDNumCountD,(Uint16)ModRegs.MDCellMaxTemps[CANARegs.MDNumCountD],(Uint16)ModRegs.MDCellMinTemps[CANARegs.MDNumCountD],(Uint16)ModRegs.MDCellAgvTemps[CANARegs.MDNumCountD]);
                   if(++CANARegs.MDNumCountD>=7)
                   {
                       CANARegs.MDNumCountD=0;
                   }
                }
                else
                {
                   CANARegs.MDNumCountD=0;
                }
       break;
       case 15:
                if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                {
                   /* 0x60B 모듈Info4 셀편차 (MDNumCountF) DivVolt/DivTemps/InResis(0) */
                   CANARegs.PackID =0X60B|SysRegs.PackID;   // TODOS : [검증] (96, 0x60B 셀편차 - 재배치 case20->15)
                   CANATX(CANARegs.PackID,8,CANARegs.MDNumCountF,ModRegs.MDCellDivVolt[CANARegs.MDNumCountF],ModRegs.MDCellDivTemps[CANARegs.MDNumCountF],0);
                   if(++CANARegs.MDNumCountF>=7){CANARegs.MDNumCountF=0;}
                }
       break;
       case 20:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   /* 0x60C 모듈별 RxCount 멀티플렉서 (MDNumCountC, BMNum + RxCount[0]~[6] 패킹) */
                   CANARegs.MDRxCurP   = CANARegs.MDXRxcountP[CANARegs.MDNumCountC];
                   CANARegs.MDRxCount1 = ComBine(CANARegs.MDRxCurP[0],CANARegs.MDNumCountC);
                   CANARegs.MDRxCount2 = ComBine(CANARegs.MDRxCurP[2],CANARegs.MDRxCurP[1]);
                   CANARegs.MDRxCount3 = ComBine(CANARegs.MDRxCurP[4],CANARegs.MDRxCurP[3]);
                   CANARegs.MDRxCount4 = ComBine(CANARegs.MDRxCurP[6],CANARegs.MDRxCurP[5]);
                   CANARegs.PackID =0X60C|SysRegs.PackID;   // TODOS : [검증] (97, 0x60C RxCount - 재배치 case30->20)
                   CANATX(CANARegs.PackID,8,CANARegs.MDRxCount1 ,CANARegs.MDRxCount2 ,CANARegs.MDRxCount3 ,CANARegs.MDRxCount4);
                   if(++CANARegs.MDNumCountC>=7)
                   {
                       CANARegs.MDNumCountC=0;
                   }
               }
               else
               {
                   CANARegs.MDNumCountC=0;
               }
       break;
       case 30:
                if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                {
                   /* 0x60D Dug1: PackSWID/DI.all/DO.all/PackAh */
                   CANARegs.PackID =0X60D|SysRegs.PackID;   // TODOS : [검증] (98, 0x60D Dug1 - 재배치 case10->30)
                   CANARegs.PackAh = (int16)(Kam100AHSocRegs.SysAhF * 10);
                   CANATX(CANARegs.PackID,8,SysRegs.PackSWID,SysRegs.DigitalInputReg.all,SysRegs.DigitalOutPutReg.all,CANARegs.PackAh);
                }
       break;
       case 40:
                if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                {
                   /* 0x60E Dug2 디버그 멀티플렉서: [0]idx(0~6) [1]MBOXn 설정 RX_ID [2][3]예비(0) — 분석기 RX필터 확인용 */
                   CANARegs.PackID =0X60E|SysRegs.PackID;   // TODOS : [검증] (99, 0x60D->0x60E 디버그 - 0x60D 중복 제거)
                   CANATX(CANARegs.PackID,8,
                          CANARegs.MDNumCountE,                                          /* [0] 메일박스 인덱스(0~6) */
                          (&ECanaMboxes.MBOX0)[CANARegs.MDNumCountE].MSGID.bit.STDMSGID, /* [1] 해당 MBOX 설정 RX ID (eCAN MBOX는 연속배치라 포인터 인덱싱) */
                          0X0000,0X0000);                                                /* [2][3] 예비(0) */
                   if(++CANARegs.MDNumCountE>=7){CANARegs.MDNumCountE=0;}
                }
       break;
       case 50:
                if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                {
                   /* 0x60F Dug3 예비 (현재 미송신, 규약 미적용) */
                   CANARegs.PackID =0X60F|SysRegs.PackID;   // TODOS : [검증] (100, 0x60F 예비 - 재배치 case60->50)
                  // CANATX(CANARegs.PackID,8,CANARegs.PackID,CANARegs.MailBoxRxCount,CANARegs.MailBox1RxCount,CANARegs.MailBox2RxCount);
                }
       break;
       case 60:
                /* TODOS : [완료] (101, case60 미사용 - 재배치로 0x60F는 case50으로 이전됨) */
       break;
       case 65:

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
                   CANARegs.SWVER      = ComBine(Product_Version,SW_Type);
                   CANARegs.BatConfParallelSerial = ComBine(Product_SysCellVauleP,Product_SysCellVauleS);
                   CANARegs.PackID =0X600|SysRegs.PackID;
                   // TODOS : [완료] (22, 0x600 BPA_PROD 송신 순서 규약 일치 (byte2-3 전압, byte4-5 용량, byte6-7 셀구성 S/P))
                   CANATX(CANARegs.PackID,8,CANARegs.SWVER,(unsigned int)(Product_Voltage*10),(unsigned int)(Product_Capacity*10),CANARegs.BatConfParallelSerial);
               }
       break;
       default :
       break;
   }

   DigitalOutput(&SysRegs);
  // InitECan();
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
        // TODOS : [완료] (01, RMP 클리어를 비트필드 RMW 대신 메일박스별 즉시 클리어(shadow .all 기록)로 변경 - 동시 TX 시 손실 창 최소화)
        if(ECanaRegs.CANRMP.bit.RMP0==1) // 모듈 1
        {
            ModRegs.MD1XRxcount[0]++;
            if(ModRegs.MD1XRxcount[0]>200){ModRegs.MD1XRxcount[0]=0;}
            SysRegs.MD1CANRxCount=0;
            CANARegs.CanMBOX0Mask= (ECanaMboxes.MBOX0.MSGID.bit.STDMSGID & 0x000Fu);
            if(CANARegs.CanMBOX0Mask==2)
            {
                ModRegs.MD1XRxcount[1]++;
                if(ModRegs.MD1XRxcount[1]>200){ModRegs.MD1XRxcount[1]=0;}
                ModRegs.MDCellVoltQty[0]    = ECanaMboxes.MBOX0.MDL.byte.BYTE0;
                ModRegs.MDFirmwareVer[0]    = ECanaMboxes.MBOX0.MDL.byte.BYTE1;
                ModRegs.MDNorCapacity[0]    = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                ModRegs.MDNorVolt[0]        = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                ModRegs.PackMinVolteRec[0]  = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX0Mask==3)
            {
                ModRegs.MD1XRxcount[2]++;
                if(ModRegs.MD1XRxcount[2]>200){ModRegs.MD1XRxcount[2]=0;}
                ModRegs.MDCellMaxVolt[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                ModRegs.MDCellMinVolt[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                ModRegs.MDCellAgvVolt[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                ModRegs.MDCellDivVolt[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX0Mask==4)
            {
                ModRegs.MD1XRxcount[3]++;
                if(ModRegs.MD1XRxcount[3]>200){ModRegs.MD1XRxcount[3]=0;}
                ModRegs.MDCellMaxTemps[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                ModRegs.MDCellMinTemps[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                ModRegs.MDCellAgvTemps[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                ModRegs.MDCellDivTemps[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX0Mask==5)
            {
                ModRegs.MD1XRxcount[4]++;
                if(ModRegs.MD1XRxcount[4]>200){ModRegs.MD1XRxcount[4]=0;}
                ModRegs.MDTotalVolt[0]      = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                ModRegs.MDMaxVoltPo[0]      = ECanaMboxes.MBOX0.MDL.byte.BYTE2;
                ModRegs.MDMinVoltPo[0]      = ECanaMboxes.MBOX0.MDL.byte.BYTE3;
                ModRegs.MDMaxTempsPo[0]     = ECanaMboxes.MBOX0.MDH.byte.BYTE4;
                ModRegs.MDMinTempsPo[0]     = ECanaMboxes.MBOX0.MDH.byte.BYTE5;
                ModRegs.MDstatusbit[0]      = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);
            }
            ECanaShadow.CANRMP.all = 0;
            ECanaShadow.CANRMP.bit.RMP0 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;   // RMP0만 클리어
        }
        if(ECanaRegs.CANRMP.bit.RMP1==1) // 모듈 2
        {
            ModRegs.MD2XRxcount[0]++;
            if(ModRegs.MD2XRxcount[0]>200){ModRegs.MD2XRxcount[0]=0;}
            SysRegs.MD2CANRxCount=0;
            CANARegs.CanMBOX1Mask= (ECanaMboxes.MBOX1.MSGID.bit.STDMSGID & 0x000Fu);
            if(CANARegs.CanMBOX1Mask==2)
            {
                ModRegs.MD2XRxcount[1]++;
                if(ModRegs.MD2XRxcount[1]>200){ModRegs.MD2XRxcount[1]=0;}
                ModRegs.MDCellVoltQty[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE0;
                ModRegs.MDFirmwareVer[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE1;
                ModRegs.MDNorCapacity[1]    = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                ModRegs.MDNorVolt[1]        = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE4);
                ModRegs.PackMinVolteRec[1]  = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX1Mask==3)
            {
                ModRegs.MD2XRxcount[2]++;
                if(ModRegs.MD2XRxcount[2]>200){ModRegs.MD2XRxcount[2]=0;}
                ModRegs.MDCellMaxVolt[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                ModRegs.MDCellMinVolt[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                ModRegs.MDCellAgvVolt[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE4);
                ModRegs.MDCellDivVolt[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX1Mask==4)
            {
                ModRegs.MD2XRxcount[3]++;
                if(ModRegs.MD2XRxcount[3]>200){ModRegs.MD2XRxcount[3]=0;}
                ModRegs.MDCellMaxTemps[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                ModRegs.MDCellMinTemps[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                ModRegs.MDCellAgvTemps[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE4);
                ModRegs.MDCellDivTemps[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX1Mask==5)
            {
                ModRegs.MD2XRxcount[4]++;
                if(ModRegs.MD2XRxcount[4]>200){ModRegs.MD2XRxcount[4]=0;}
                ModRegs.MDTotalVolt[1]      = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                ModRegs.MDMaxVoltPo[1]      = ECanaMboxes.MBOX1.MDL.byte.BYTE2;
                ModRegs.MDMinVoltPo[1]      = ECanaMboxes.MBOX1.MDL.byte.BYTE3;
                ModRegs.MDMaxTempsPo[1]     = ECanaMboxes.MBOX1.MDH.byte.BYTE4;
                ModRegs.MDMinTempsPo[1]     = ECanaMboxes.MBOX1.MDH.byte.BYTE5;
                ModRegs.MDstatusbit[1]      = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

            }
            ECanaShadow.CANRMP.all = 0;
            ECanaShadow.CANRMP.bit.RMP1 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;   // RMP1만 클리어
        }
        if(ECanaRegs.CANRMP.bit.RMP2==1) // 모듈 3
        {
            ModRegs.MD3XRxcount[0]++;
            SysRegs.MD3CANRxCount=0;
            if(ModRegs.MD3XRxcount[0]>200){ModRegs.MD3XRxcount[0]=0;}
            CANARegs.CanMBOX2Mask=(ECanaMboxes.MBOX2.MSGID.bit.STDMSGID & 0x000Fu);
            if(CANARegs.CanMBOX2Mask==2)
            {
                ModRegs.MD3XRxcount[1]++;
                ModRegs.MDCellVoltQty[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE0;
                ModRegs.MDFirmwareVer[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE1;
                ModRegs.MDNorCapacity[2]    = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                ModRegs.MDNorVolt[2]        = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                ModRegs.PackMinVolteRec[2]  = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                if(ModRegs.MD3XRxcount[1]>200){ModRegs.MD3XRxcount[1]=0;}
            }
            else if(CANARegs.CanMBOX2Mask==3)
            {
                ModRegs.MD3XRxcount[2]++;
                if(ModRegs.MD3XRxcount[2]>200){ModRegs.MD3XRxcount[2]=0;}
                ModRegs.MDCellMaxVolt[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                ModRegs.MDCellMinVolt[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                ModRegs.MDCellAgvVolt[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                ModRegs.MDCellDivVolt[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX2Mask==4)
            {
                ModRegs.MD3XRxcount[3]++;
                if(ModRegs.MD3XRxcount[3]>200){ModRegs.MD3XRxcount[3]=0;}
                ModRegs.MDCellMaxTemps[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                ModRegs.MDCellMinTemps[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                ModRegs.MDCellAgvTemps[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                ModRegs.MDCellDivTemps[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX2Mask==5)
            {
                ModRegs.MD3XRxcount[4]++;
                if(ModRegs.MD3XRxcount[4]>200){ModRegs.MD3XRxcount[4]=0;}
                ModRegs.MDTotalVolt[2]      = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                ModRegs.MDMaxVoltPo[2]      = ECanaMboxes.MBOX2.MDL.byte.BYTE2;
                ModRegs.MDMinVoltPo[2]      = ECanaMboxes.MBOX2.MDL.byte.BYTE3;
                ModRegs.MDMaxTempsPo[2]     = ECanaMboxes.MBOX2.MDH.byte.BYTE4;
                ModRegs.MDMinTempsPo[2]     = ECanaMboxes.MBOX2.MDH.byte.BYTE5;
                ModRegs.MDstatusbit[2]      = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
            }
            ECanaShadow.CANRMP.all = 0;
            ECanaShadow.CANRMP.bit.RMP2 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;   // RMP2만 클리어
        }
        if(ECanaRegs.CANRMP.bit.RMP3==1) // 모듈 4
        {
            ModRegs.MD4XRxcount[0]++;
            SysRegs.MD4CANRxCount=0;
            if(ModRegs.MD4XRxcount[0]>200){ModRegs.MD4XRxcount[0]=0;}
            CANARegs.CanMBOX3Mask=(ECanaMboxes.MBOX3.MSGID.bit.STDMSGID & 0x000Fu);
            if(CANARegs.CanMBOX3Mask==2)
            {
                ModRegs.MD4XRxcount[1]++;
                if(ModRegs.MD4XRxcount[1]>200){ModRegs.MD4XRxcount[1]=0;}
                ModRegs.MDCellVoltQty[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE0;
                ModRegs.MDFirmwareVer[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE1;
                ModRegs.MDNorCapacity[3]    = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                ModRegs.MDNorVolt[3]        = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                ModRegs.PackMinVolteRec[3]  = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX3Mask==3)
            {
                ModRegs.MD4XRxcount[2]++;
                if(ModRegs.MD4XRxcount[2]>200){ModRegs.MD4XRxcount[2]=0;}
                ModRegs.MDCellMaxVolt[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                ModRegs.MDCellMinVolt[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                ModRegs.MDCellAgvVolt[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                ModRegs.MDCellDivVolt[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX3Mask==4)
            {
                ModRegs.MD4XRxcount[3]++;
                if(ModRegs.MD4XRxcount[3]>200){ModRegs.MD4XRxcount[3]=0;}
                ModRegs.MDCellMaxTemps[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                ModRegs.MDCellMinTemps[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                ModRegs.MDCellAgvTemps[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                ModRegs.MDCellDivTemps[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX3Mask==5)
            {
                ModRegs.MD4XRxcount[4]++;
                if(ModRegs.MD4XRxcount[4]>200){ModRegs.MD4XRxcount[4]=0;}
                ModRegs.MDTotalVolt[3]      = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                ModRegs.MDMaxVoltPo[3]      = ECanaMboxes.MBOX3.MDL.byte.BYTE2;
                ModRegs.MDMinVoltPo[3]      = ECanaMboxes.MBOX3.MDL.byte.BYTE3;
                ModRegs.MDMaxTempsPo[3]     = ECanaMboxes.MBOX3.MDH.byte.BYTE4;
                ModRegs.MDMinTempsPo[3]     = ECanaMboxes.MBOX3.MDH.byte.BYTE5;
                ModRegs.MDstatusbit[3]      = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);
            }
            ECanaShadow.CANRMP.all = 0;
            ECanaShadow.CANRMP.bit.RMP3 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;   // RMP3만 클리어
        }
        if(ECanaRegs.CANRMP.bit.RMP4==1) // 모듈 5
        {
            SysRegs.MD5CANRxCount=0;
            ModRegs.MD5XRxcount[0]++;
            if(ModRegs.MD5XRxcount[0]>200){ModRegs.MD5XRxcount[0]=0;}
            CANARegs.CanMBOX4Mask=(ECanaMboxes.MBOX4.MSGID.bit.STDMSGID & 0x000Fu);
            if(CANARegs.CanMBOX4Mask==2)
            {
                ModRegs.MD5XRxcount[1]++;
                if(ModRegs.MD5XRxcount[1]>200){ModRegs.MD5XRxcount[1]=0;}
                ModRegs.MDCellVoltQty[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE0;
                ModRegs.MDFirmwareVer[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE1;
                ModRegs.MDNorCapacity[4]    = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                ModRegs.MDNorVolt[4]        = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                ModRegs.PackMinVolteRec[4]  = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX4Mask==3)
            {
                ModRegs.MD5XRxcount[2]++;
                if(ModRegs.MD5XRxcount[2]>200){ModRegs.MD5XRxcount[2]=0;}
                ModRegs.MDCellMaxVolt[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                //TODS : [현장 대응] 모듈 5의 최소 셀전압 이상하게 들어오는 현상으로 인해 임시로 모듈 4번 인덱스에 저장하도록 수정
             //   ModRegs.MDCellMinVolt[4] = ModRegs.MDCellMinVolt[3];
                ModRegs.MDCellMinVolt[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                ModRegs.MDCellAgvVolt[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                ModRegs.MDCellDivVolt[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX4Mask==4)
            {
                ModRegs.MD5XRxcount[3]++;
                if(ModRegs.MD5XRxcount[3]>200){ModRegs.MD5XRxcount[3]=0;}
                ModRegs.MDCellMaxTemps[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                ModRegs.MDCellMinTemps[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                ModRegs.MDCellAgvTemps[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                ModRegs.MDCellDivTemps[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX4Mask==5)
            {
                ModRegs.MD5XRxcount[4]++;
                if(ModRegs.MD5XRxcount[4]>200){ModRegs.MD5XRxcount[4]=0;}
                ModRegs.MDTotalVolt[4]      = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                ModRegs.MDMaxVoltPo[4]      = ECanaMboxes.MBOX4.MDL.byte.BYTE2;
                ModRegs.MDMinVoltPo[4]      = ECanaMboxes.MBOX4.MDL.byte.BYTE3;
                ModRegs.MDMaxTempsPo[4]     = ECanaMboxes.MBOX4.MDH.byte.BYTE4;
                ModRegs.MDMinTempsPo[4]     = ECanaMboxes.MBOX4.MDH.byte.BYTE5;
                ModRegs.MDstatusbit[4]      = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
            }
            ECanaShadow.CANRMP.all = 0;
            ECanaShadow.CANRMP.bit.RMP4 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;   // RMP4만 클리어
        }
        if(ECanaRegs.CANRMP.bit.RMP5==1) // 모듈 6
        {
            SysRegs.MD6CANRxCount=0;
            ModRegs.MD6XRxcount[0]++;
            if(ModRegs.MD6XRxcount[0]>200){ModRegs.MD6XRxcount[0]=0;}
            CANARegs.CanMBOX5Mask= (ECanaMboxes.MBOX5.MSGID.bit.STDMSGID & 0x000Fu);
            if(CANARegs.CanMBOX5Mask==2)
            {
                ModRegs.MD6XRxcount[1]++;
                if(ModRegs.MD6XRxcount[1]>200){ModRegs.MD6XRxcount[1]=0;}
                ModRegs.MDCellVoltQty[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE0;
                ModRegs.MDFirmwareVer[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE1;
                ModRegs.MDNorCapacity[5]    = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                ModRegs.MDNorVolt[5]        = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE5,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                ModRegs.PackMinVolteRec[5]  = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX5Mask==3)
            {
                ModRegs.MD6XRxcount[2]++;
                if(ModRegs.MD6XRxcount[2]>200){ModRegs.MD6XRxcount[2]=0;}
                ModRegs.MDCellMaxVolt[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                ModRegs.MDCellMinVolt[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                ModRegs.MDCellAgvVolt[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE5,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                ModRegs.MDCellDivVolt[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX5Mask==4)
            {
                ModRegs.MD6XRxcount[3]++;
                if(ModRegs.MD6XRxcount[3]>200){ModRegs.MD6XRxcount[3]=0;}
                ModRegs.MDCellMaxTemps[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                ModRegs.MDCellMinTemps[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                ModRegs.MDCellAgvTemps[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE5,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                ModRegs.MDCellDivTemps[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX5Mask==5)
            {
                ModRegs.MD6XRxcount[4]++;
                if(ModRegs.MD6XRxcount[4]>200){ModRegs.MD6XRxcount[4]=0;}
                ModRegs.MDTotalVolt[5]      = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                ModRegs.MDMaxVoltPo[5]      = ECanaMboxes.MBOX5.MDL.byte.BYTE2;
                ModRegs.MDMinVoltPo[5]      = ECanaMboxes.MBOX5.MDL.byte.BYTE3;
                ModRegs.MDMaxTempsPo[5]     = ECanaMboxes.MBOX5.MDH.byte.BYTE4;
                ModRegs.MDMinTempsPo[5]     = ECanaMboxes.MBOX5.MDH.byte.BYTE5;
                ModRegs.MDstatusbit[5]      = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
            }
            ECanaShadow.CANRMP.all = 0;
            ECanaShadow.CANRMP.bit.RMP5 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;   // RMP5만 클리어
        }
        if(ECanaRegs.CANRMP.bit.RMP6==1) // 모듈 7
        {
            SysRegs.MD7CANRxCount=0;
            ModRegs.MD7XRxcount[0]++;
            if(ModRegs.MD7XRxcount[0]>200){ModRegs.MD7XRxcount[0]=0;}
            CANARegs.CanMBOX6Mask= (ECanaMboxes.MBOX6.MSGID.bit.STDMSGID & 0x000Fu);
            if(CANARegs.CanMBOX6Mask==2)
            {
                ModRegs.MD7XRxcount[1]++;
                if(ModRegs.MD7XRxcount[1]>200){ModRegs.MD7XRxcount[1]=0;}
                ModRegs.MDCellVoltQty[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE0;
                ModRegs.MDFirmwareVer[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE1;
                ModRegs.MDNorCapacity[6]    = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                ModRegs.MDNorVolt[6]        = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                ModRegs.PackMinVolteRec[6]  = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX6Mask==3)
            {
                ModRegs.MD7XRxcount[2]++;
                if(ModRegs.MD7XRxcount[2]>200){ModRegs.MD7XRxcount[2]=0;}
                ModRegs.MDCellMaxVolt[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                ModRegs.MDCellMinVolt[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                ModRegs.MDCellAgvVolt[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                ModRegs.MDCellDivVolt[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX6Mask==4)
            {
                ModRegs.MD7XRxcount[3]++;
                if(ModRegs.MD7XRxcount[3]>200){ModRegs.MD7XRxcount[3]=0;}
                ModRegs.MDCellMaxTemps[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                ModRegs.MDCellMinTemps[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                ModRegs.MDCellAgvTemps[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                ModRegs.MDCellDivTemps[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);
            }
            else if(CANARegs.CanMBOX6Mask==5)
            {
                ModRegs.MD7XRxcount[4]++;
                if(ModRegs.MD7XRxcount[4]>200){ModRegs.MD7XRxcount[4]=0;}
                ModRegs.MDTotalVolt[6]      = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                ModRegs.MDMaxVoltPo[6]      = ECanaMboxes.MBOX6.MDL.byte.BYTE2;
                ModRegs.MDMinVoltPo[6]      = ECanaMboxes.MBOX6.MDL.byte.BYTE3;
                ModRegs.MDMaxTempsPo[6]     = ECanaMboxes.MBOX6.MDH.byte.BYTE4;
                ModRegs.MDMinTempsPo[6]     = ECanaMboxes.MBOX6.MDH.byte.BYTE5;
                ModRegs.MDstatusbit[6]      = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);
            }
            ECanaShadow.CANRMP.all = 0;
            ECanaShadow.CANRMP.bit.RMP6 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;   // RMP6만 클리어
        }
        if(ECanaRegs.CANRMP.bit.RMP29==1) // CAN CAN 
        {
            SysRegs.CTRxCount=0;
            CANARegs.MailBox1RxCount++;
            if(CANARegs.MailBox1RxCount>200){CANARegs.MailBox1RxCount=0;}
            SysRegs.CurrentData.byte.CurrentH   = (ECanaMboxes.MBOX29.MDL.byte.BYTE0<<8)|(ECanaMboxes.MBOX29.MDL.byte.BYTE1);
            SysRegs.CurrentData.byte.CurrentL   = (ECanaMboxes.MBOX29.MDL.byte.BYTE2<<8)|(ECanaMboxes.MBOX29.MDL.byte.BYTE3);

            ECanaShadow.CANRMP.all = 0;
            ECanaShadow.CANRMP.bit.RMP29 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;   // RMP29만 클리어
        }
        if(ECanaRegs.CANRMP.bit.RMP30==1)
        {
            CANARegs.MailBox2RxCount++;
            SysRegs.MasterRxCount=0;
            if(CANARegs.MailBox2RxCount>200) {CANARegs.MailBox2RxCount=0;}
            CANARegs.PMSCMDRegs.all = ComBine(ECanaMboxes.MBOX30.MDL.byte.BYTE1,ECanaMboxes.MBOX30.MDL.byte.BYTE0);
            if(CANARegs.PMSCMDRegs.bit.PrtctReset01==1)
            {
                CANARegs.PMSCMDRegs.bit.RUNStatus01=0;
            }

            ECanaShadow.CANRMP.all = 0;
            ECanaShadow.CANRMP.bit.RMP30 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;   // RMP30만 클리어
        }
    }
    ECanaRegs.CANGIF0.all = ECanaRegs.CANGIF0.all;
    ECanaRegs.CANGIF1.all = ECanaRegs.CANGIF1.all;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

}
/*
interrupt void cpu_timer2_isr(void)
{  EALLOW;
   CpuTimer2.InterruptCount++;
   // The CPU acknowledges the interrupt.
  // A_OVCHACurrent;
   EDIS;
}
*/

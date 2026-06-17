
#include "parameter.h"
#include "SysVariable.h"
#include "DSP28x_Project.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

extern void SysTimerINIT(SystemReg *s);
extern void CANRegVarINIT(CANAReg *P);
extern void SysVarINIT(SystemReg *s);
extern void ModuleInit(ModulemReg *P);
extern void DigitalInput(SystemReg *sys);
extern void DigitalOutput(SystemReg *sys);
extern void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3);

extern void SysCommErrHandle(SystemReg *P);
extern void SysCurrentHandle(SystemReg *s);
extern void SysUnitBMSStatus(SystemReg *P);

extern void CalSysAlarmtCheck(SystemReg *s);
extern void CalSysFaultCheck(SystemReg *s);
extern void CalSysProtectCheck(SystemReg *s);

extern int float32ToInt(float32 Vaule, Uint32 Num);
//extern SystemReg       SysRegs;
void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3)
{
    /*--------------------------------------------------------------
     * 260615 : CANATX 재작성 - MBOX31 단일공유 안전화 + 송신 race(0x601 dip) 제거
     *   기존 : 송신요청 후 완료까지 블로킹 대기(최대 3ms) -> 버스부하시 메인 지연/dip
     *   변경 : 시작에서 직전 송신완료만 확인, 송신요청 후 즉시 복귀(non-blocking)
     *--------------------------------------------------------------*/
    struct ECAN_REGS ECanaShadow;
    unsigned int CANWatchDog=0;
    unsigned int Data0Low, Data0High, Data1Low, Data1High;
    unsigned int Data2Low, Data2High, Data3Low, Data3High;

    /* 1) 직전 송신 완료 보장 (MBOX31 재사용 충돌 방지) - 평소 즉시통과, 부하시만 대기 */
    while (ECanaRegs.CANTRS.bit.TRS31 == 1U)   // wait until previous send is done
    {
        if (++CANWatchDog > 20000U)   // TODOS : [검증] (71, 송신 race 방지: 직전 송신완료 대기 타임아웃 3ms)
        {
            break; // give up previous send
        }
    }
    if (ECanaRegs.CANTA.bit.TA31 == 1U)        // clear previous done flag (write-1-clear)
    {
        ECanaRegs.CANTA.bit.TA31 = 1U;
    }

    Data0Low  = 0x00ff&Data0;
    Data0High = 0x00ff&(Data0>>8);
    Data1Low  = 0x00ff&Data1;
    Data1High = 0x00ff&(Data1>>8);
    Data2Low  = 0x00ff&Data2;
    Data2High = 0x00ff&(Data2>>8);
    Data3Low  = 0x00ff&Data3;
    Data3High = 0x00ff&(Data3>>8);

    EALLOW;
    ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    ECanaShadow.CANME.bit.ME31=0;
    ECanaRegs.CANME.bit.ME31= ECanaShadow.CANME.bit.ME31;

    ECanaMboxes.MBOX31.MSGID.all = 0UL;
    ECanaMboxes.MBOX31.MSGID.bit.IDE = 0U;                 // 표준 프레임
    ECanaMboxes.MBOX31.MSGID.bit.STDMSGID = (Uint16)(ID & 0x07FFU); // 11-bit

    ECanaMboxes.MBOX31.MSGCTRL.bit.RTR = 0U;
    ECanaMboxes.MBOX31.MSGCTRL.bit.DLC = (Length > 8U) ? 8U : Length; // clamp to 8



    ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    ECanaShadow.CANME.bit.ME31 = 1U;
    ECanaRegs.CANME.all = ECanaShadow.CANME.all;
    EDIS;

    ECanaMboxes.MBOX31.MDL.byte.BYTE0=Data0Low;
    ECanaMboxes.MBOX31.MDL.byte.BYTE1=Data0High;
    ECanaMboxes.MBOX31.MDL.byte.BYTE2=Data1Low;
    ECanaMboxes.MBOX31.MDL.byte.BYTE3=Data1High;
    ECanaMboxes.MBOX31.MDH.byte.BYTE4=Data2Low;
    ECanaMboxes.MBOX31.MDH.byte.BYTE5=Data2High;
    ECanaMboxes.MBOX31.MDH.byte.BYTE6=Data3Low;
    ECanaMboxes.MBOX31.MDH.byte.BYTE7=Data3High;


    ECanaRegs.CANTRS.bit.TRS31 = 1U;   // request transmit (완료는 다음 호출에서 확인, non-blocking)

}
void SysTimerINIT(SystemReg *s)
{
    s->SysMachine=System_STATE_INIT;
    s->Maincount=0;
    s->MainIsr1=0;
    s->CANRXCOUNT=0;
    s->CANRXMailBox00Count=0;
    s->CANRXMailBox01Count=0;
    s->CANRXMailBox02Count=0;
    s->CANRXMailBox03Count=0;
    s->CANRXMailBox04Count=0;
    s->SysRegTimer5msecCount=0;
    s->SysRegTimer10msecCount=0;
    s->SysRegTimer50msecCount=0;
    s->SysRegTimer100msecCount=0;
    s->SysRegTimer300msecCount=0;
    s->SysRegTimer500msecCount=0;
    s->BalanceModeCount=0;
    s->BalanceTimeCount=0;

    s->LEDSycCount=0;
    s->LEDFaultCount=0;
    s->LEDCanCount=0;
    s->PWRLAMPCount=0;
    s->StartBATOUTOnCount=0;
    s->StartBATOUTOffCount=0;
}
void SysVarINIT(SystemReg *s)
{

    s->PMSysCMDResg.all=0;
    s->PackStateReg.all=0;
    s->PackAlarmReg.all=0;
    s->PackFaultReg.all=0;
    s->PackProtectReg.all=0;
    s->PackFaulBuftReg.all=0;
    s->DigitalInputReg.all=0;
    s->DigitalOutPutReg.all=0;
    s->CurrentData.all=0x80000000;
    // TODOS : [완료] (23, 0x607 셀 내부저항 계산값/위치 초기화)
    s->PackCellMaxIRF=0;
    s->PackCellMinIRF=0;
    s->PackCellAgvIRF=0;
    s->PackCellDivIRF=0;
    s->PackCellMaxIRPos=0;
    s->PackCellMinIRPos=0;
  //  s->PackCOMERR.all=0;

    s->Test=0;
    s->Maincount=0;
    s->MainIsr1=0;
    s->CANRXCOUNT=0;
    s->CANRXMailBox00Count=0;
    s->CANRXMailBox01Count=0;
    s->CANRXMailBox02Count=0;
    s->CANRXMailBox03Count=0;
    s->CANRXMailBox04Count=0;
    s->SysRegTimer5msecCount=0;
    s->SysRegTimer10msecCount=0;
    s->SysRegTimer50msecCount=0;
    s->SysRegTimer100msecCount=0;
    s->SysRegTimer300msecCount=0;
    s->SysRegTimer500msecCount=0;
    s->SysRegTimer1000msecCount=0;
    s->CellVoltsampling=0;
    s->SysCanRxCount=0;
    s->AlarmStatecount=0;
    s->PackFaultStatecount=0;
    s->Bat12VFaultStatecount=0;
    s->ProtectStatecount=0;
    s->RelayCheck=0;
    s->PackCellMaxVoltPos=0;
    s->PackCellMinVoltPos=0;
    s->PackVoltageF=0;
    s->PackCurrentF=0;
    s->PackCurrentAsbF=0;
    s->PackCellMaxVoltageF=0;
    s->PackCellMinVoltageF=0;
    s->PackCellDivVoltageF=0;
    s->PackCellAgvVoltageF=0;
    s->PackCellMaxTemperatureF=0;
    s->PackCellMinTemperatureF=0;
    s->PackCellDivTemperatureF=0;
    s->PackCellAgvTemperatureF=0;
    s->PackCellMaxTmepsPos=0;
    s->PackCellMinTmepsPos=0;

    s->PackCHAPWRContintyF=5;//5
    s->PackDisCHAPWRContintyF=5;
    s->PackCHAPWRPeakF=5;
    s->PackDisCHAPWRPeakF=5;
    s->PackSOCF=0;
    s->PackSOHF=0;
    s->PackAhF=0;
    s->PackISOResisF=0;


    s->MD1CANRxCount=0;
    s->MD2CANRxCount=0;
    s->MD3CANRxCount=0;
    s->MD4CANRxCount=0;
    s->MD5CANRxCount=0;
    s->MD6CANRxCount=0;
    s->MD7CANRxCount=0;
    s->CTRxCount=0;
    s->MasterRxCount=0;

    s->PackInMDCANrxReg.all=0;

    memset(s->PackModuleRegs, 0, sizeof(s->PackModuleRegs));

/*    s->PackModule1Regs.all=0;
    s->PackModule2Regs.all=0;
    s->PackModule3Regs.all=0;
    s->PackModule4Regs.all=0;
    s->PackModule5Regs.all=0;
    s->PackModule6Regs.all=0;
    s->PackModule7Regs.all=0;
    s->PackModuleAllRegs.all=0;
*/
    s->PackInMDCANrxReg.all=0;
    s->MDInitok.all=0;
    s->MDFault.all=0;
    s->MDBalaMode.all=0;
    s->MDWaterleak.all=0;
    s->MDCellVoltFualt.all=0;
    s->MDCellTempFualt.all=0;
    s->MDBATICErr.all=0;
    s->MDCTComErr.all=0;
    s->MDSBComErr.all=0;
    s->MDHMIComErr.all=0;
    s->MDVoltCanTx.all=0;
    s->MDTempCanTx.all=0;
    s->HmiModeEn.all=0;
    s->PackModeEn.all=0;
    s->PackProtectReg.all=0;
    s->PackFaultReg.all=0;
    memset(&s->SysAlarmCont[0],0,sizeof(Uint16)*16);
    memset(&s->SysFalutCont[0],0,sizeof(Uint16)*16);
}
void CANRegVarINIT(CANAReg *P)
{
    P->SWTypeVer=0;
    P->PackID=0;
    P->PackSWID=0;
    P->MDNumCountE=0;   // TODOS : [검증] (32, 0x60E 디버그 멀티플렉서 인덱스 초기화)
    P->PackSate=0;
    P->PackProtetSate=0;
    P->PackSateInfo=0;
    /*
     *
     */
    P->CellNumStart=0;
    P-> NumberShift=0;
    P->CellVotlageNumber=0;;
    P->CellVotlageMaxNumber=0;;
    P->CellVoltageNum=0;
    P->PMSCMDRegs.all=0;
    P->PackDigitalOutPutReg.all=0;
    P->CellNumTStart=0;
    P->NumberTShift=0;
    P->CellTemperatureNumber=0;
    P->CellTemperatureMaxNumber=0;
    P->CellTemperatureNum=0;
    /*
     *
     */
    P->PackConfing=0;
    P->PackStatus.all=0;
    P->PackConFig=0;
    P->PackSOC=0;
    P->PackSOH=0;
    P->PackAh=0;
    P->PackCT=0;
    P->PackPT=0;
    P->PackCHAPWRContinty=0;
    P->PackCHAPWRPeak=0;
    P->PackDisCHAPWRContinty=0;
    P->PackDisCHAPWRPeak=0;
    P->PackVoltageMax=0;
    P->PackVoltageMin=0;
    P->PackBalanVolt=4500;
    P->PackVoltageAgv=0;
    P->PackVoltageDiv=0;
    P->PackVoltageMaxNum=0;
    P->PackVoltageMinNum=0;
    P->PackPackVotageBuf=0;

    P->PackTemperaturelMAX=0;
    P->PackTemperaturelMIN=0;
    P->PackTemperatureAVG=0;
    P->PackTemperatureDiv=0;
    P->PackTemperatureMaxNUM=0;
    P->PackTemperatureMinNUM=0;


    P->MailBoxRxCount=0;
    P->MailBox0RxCount=0;
    P->MailBox1RxCount=0;
    P->MailBox2RxCount=0;
    P->MailBox3RxCount=0;

    P->SWVER=0;
    P->BatConfParallelSerial=0;
    // TODOS : [완료] (24, 0x607 셀 내부저항 송신값 + 위치정보 합산필드 초기화)
    P->CellIRMAX=0;
    P->CellIRlMIN=0;
    P->CellIRAVG=0;
    P->CellIRDiv=0;
    P->PackVoltageMaxMinNum=0;
    P->PackTemperatureMaxMinNum=0;
    P->PackIRMaxMinNUM=0;
    memset(&P->MDTotalVolt[0],0,7);
    // TODOS : [검증] (25, 0x60A RxCount 송신버퍼(MDRxCount1~4) + 현재모듈 포인터 초기화)
    P->MDRxCount1=0; P->MDRxCount2=0; P->MDRxCount3=0; P->MDRxCount4=0;
    P->MDRxCurP=0;
    P->MDXRxcountP[0]=0; P->MDXRxcountP[1]=0; P->MDXRxcountP[2]=0; P->MDXRxcountP[3]=0;
    P->MDXRxcountP[4]=0; P->MDXRxcountP[5]=0; P->MDXRxcountP[6]=0;

}
void ModuleInit(ModulemReg *P)
{
    memset(&P->MDCellVoltQty[0],0,7);
    memset(&P->MDCellTempsQty[0],0,7);
    memset(&P->MDFirmwareVer[0],0,7);
    memset(&P->MDFirmwareRev[0],0,7);
    memset(&P->MDNorVolt[0],0,7);
    memset(&P->MDNorCapacity[0],0,7);
    memset(&P->PackMinVolteRec[0],0,7);

    memset(&P->MDCellMaxVolt[0],0,7);
    memset(&P->MDCellMinVolt[0],0,7);
    memset(&P->MDCellAgvVolt[0],0,7);
    memset(&P->MDCellDivVolt[0],0,7);

    memset(&P->MDCellMaxTemps[0],0,7);
    memset(&P->MDCellMinTemps[0],0,7);
    memset(&P->MDCellAgvTemps[0],0,7);
    memset(&P->MDCellDivTemps[0],0,7);

    memset(&P->MDCTComErr[0],0,7);
    memset(&P->MDSubComErr[0],0,7);
    memset(&P->MDBatICCOMErr[0],0,7);
    memset(&P->MDWaterLeakErr[0],0,7);

    memset(&P->MDTotalVolt[0],0,7);
    memset(&P->MDMaxVoltPo[0],0,7);
    memset(&P->MDMinVoltPo[0],0,7);
    memset(&P->MDMaxTempsPo[0],0,7);
    memset(&P->MDMinTempsPo[0],0,7);
    memset(&P->MDstatusbit[0],0,7);



    memset(&P->MD1XRxcount[0],0,7);
    memset(&P->MD2XRxcount[0],0,7);
    memset(&P->MD3XRxcount[0],0,7);
    memset(&P->MD4XRxcount[0],0,7);
    memset(&P->MD5XRxcount[0],0,7);
    memset(&P->MD6XRxcount[0],0,7);
    memset(&P->MD7XRxcount[0],0,7);

}


void SysUnitBMSStatus(SystemReg *P)
{
    /*
     * 하드웨어 인터럽트에서 50msec 마다 아래와 같이 read 있음
     */
    //switch(SysRegs.SysRegTimer500msecCount)
    //SysRegs.PackModule1Regs.all = ModRegs.MDstatusbit[0];
    //SysRegs.PackModule2Regs.all = ModRegs.MDstatusbit[1];
    //SysRegs.PackModule3Regs.all = ModRegs.MDstatusbit[2];
    //SysRegs.PackModule4Regs.all = ModRegs.MDstatusbit[3];
    //SysRegs.PackModule5Regs.all = ModRegs.MDstatusbit[4];
    //SysRegs.PackModule6Regs.all = ModRegs.MDstatusbit[5];
    //SysRegs.PackModule7Regs.all = ModRegs.MDstatusbit[6];


    const Uint16 MODULE_CNT = 7u;
    Uint16 i;
    /* ANY(OR) 누적용 */
    Uint16 any_fault=0u, any_leak=0u, any_cellv=0u, any_cellt=0u;
    Uint16 any_batic=0u, any_ct=0u, any_mb=0u, any_hmi=0u, any_balance=0u;
    Uint16 all_initok=1u, all_voltcan=1u, all_tempcan=1u, all_hmien=1u, all_packen=1u;
    for (i = 0u; i < MODULE_CNT; i++)
    {
        /* ALL(AND) */
        all_initok  &= (Uint16)P->PackModuleRegs[i].bit.INITOK;
        all_voltcan &= (Uint16)P->PackModuleRegs[i].bit.CellVoltCAN;
        all_tempcan &= (Uint16)P->PackModuleRegs[i].bit.CellTempCAN;
        all_hmien   &= (Uint16)P->PackModuleRegs[i].bit.HmiEn;
        all_packen  &= (Uint16)P->PackModuleRegs[i].bit.PackEn;

        /* ANY(OR) */
        any_fault   |= (Uint16)P->PackModuleRegs[i].bit.Fault;
        any_leak    |= (Uint16)P->PackModuleRegs[i].bit.WaterleakFault;
        any_cellv   |= (Uint16)P->PackModuleRegs[i].bit.CellVoltageFault;
        any_cellt   |= (Uint16)P->PackModuleRegs[i].bit.CellTemperatureFault;
        any_batic   |= (Uint16)P->PackModuleRegs[i].bit.BATICErrFault;
        any_ct      |= (Uint16)P->PackModuleRegs[i].bit.CTCOMErrFault;
        any_mb      |= (Uint16)P->PackModuleRegs[i].bit.MBCOMErrFault;
        any_hmi     |= (Uint16)P->PackModuleRegs[i].bit.HMICOMErrFault;
        any_balance |= (Uint16)P->PackModuleRegs[i].bit.BalanceEnable;

    }
    P->PackModuleAllRegs.bit.INITOK               = all_initok;      /* ALL */
    P->PackModuleAllRegs.bit.Fault                = any_fault;       /* ANY */
    P->PackModuleAllRegs.bit.WaterleakFault       = any_leak;        /* ANY */
    P->PackModuleAllRegs.bit.CellVoltageFault     = any_cellv;       /* ANY */
    P->PackModuleAllRegs.bit.CellTemperatureFault = any_cellt;       /* ANY */
    P->PackModuleAllRegs.bit.BATICErrFault        = any_batic;       /* ANY */
    P->PackModuleAllRegs.bit.CTCOMErrFault        = any_ct;          /* ANY */
    P->PackModuleAllRegs.bit.MBCOMErrFault        = any_mb;          /* ANY */
    P->PackModuleAllRegs.bit.HMICOMErrFault       = any_hmi;         /* ANY */
    P->PackModuleAllRegs.bit.BalanceEnable        = any_balance;     /* ANY */
    P->PackModuleAllRegs.bit.CellVoltCAN          = all_voltcan;     /* ALL */
    P->PackModuleAllRegs.bit.CellTempCAN          = all_tempcan;     /* ALL */
    P->PackModuleAllRegs.bit.HmiEn                = all_hmien;       /* ALL */
    P->PackModuleAllRegs.bit.PackEn               = all_packen;      /* ALL */
}
void SysCommErrHandle(SystemReg *P)
{
    P->MD1CANRxCount++;
    P->MD2CANRxCount++;
    P->MD3CANRxCount++;
    P->MD4CANRxCount++;
    P->MD5CANRxCount++;
    P->MD6CANRxCount++;
    P->MD7CANRxCount++;
    P->CTRxCount++;
    P->MasterRxCount++;
    if(P->MasterRxCount>5000)
    {
        P->MasterRxCount =5100;
      //  P->PackStateReg.bit.EXCANCOMERR=1;
        P->PackProtectReg.bit.PackExComErr=1;
    }
    if(P->CTRxCount>5000)
    {
        P->CTRxCount=5100;
      //  P->PackStateReg.bit.CTCOMErr=1;
      //  P->PackProtectReg.bit.PackCTComErr=1;
    }
    if(P->PackInMDCANrxReg.all>0)
    {
        // TODOS : [검증] (26, CANCOMERR 삭제 -> SysUnitComErr(0x602 bit55)로 기능 이전)
        P->PackStateReg.bit.SysUnitComErr=1;

    }
    if(P->MD1CANRxCount>1000)
    {
        P->MD1CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD01=1;
    }
    if(P->MD2CANRxCount>1000)
    {
        P->MD2CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD02=1;
    }
    if(P->MD3CANRxCount>1000)
    {
        P->MD3CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD03=1;
    }
    if(P->MD4CANRxCount>1000)
    {
        P->MD4CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD04=1;
    }
    if(P->MD5CANRxCount>1000)
    {
        P->MD5CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD05=1;
    }
    if(P->MD6CANRxCount>1000)
    {
        P->MD6CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD06=1;
    }
    if(P->MD7CANRxCount>1000)
    {
        P->MD7CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD07=1;
    }
    /*
    if(P->PackModule1Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD01=1;}
    if(P->PackModule2Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD02=1;}
    if(P->PackModule3Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD03=1;}
    if(P->PackModule4Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD04=1;}
    if(P->PackModule5Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD05=1;}
    if(P->PackModule6Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD06=1;}
    if(P->PackModule7Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD07=1;}
    //
    if(P->PackModule1Regs.bit.CTCOMErrFault==1){P->ModuleCTCANErrCReg.bit.MD01=1;}
    if(P->PackModule2Regs.bit.CTCOMErrFault==1){P->ModuleCTCANErrCReg.bit.MD02=1;}
    if(P->PackModule3Regs.bit.CTCOMErrFault==1){P->ModuleCTCANErrCReg.bit.MD03=1;}
    if(P->PackModule4Regs.bit.CTCOMErrFault==1){P->ModuleCTCANErrCReg.bit.MD04=1;}
    if(P->PackModule5Regs.bit.CTCOMErrFault==1){P->ModuleCTCANErrCReg.bit.MD05=1;}
    if(P->PackModule6Regs.bit.CTCOMErrFault==1){P->ModuleCTCANErrCReg.bit.MD06=1;}
    if(P->PackModule7Regs.bit.CTCOMErrFault==1){P->ModuleCTCANErrCReg.bit.MD07=1;}

    if(P->PackModule1Regs.bit.MBCOMErrFault==1){P->ModuleSubCANErrCReg.bit.MD01=1;}
    if(P->PackModule2Regs.bit.MBCOMErrFault==1){P->ModuleSubCANErrCReg.bit.MD02=1;}
    if(P->PackModule3Regs.bit.MBCOMErrFault==1){P->ModuleSubCANErrCReg.bit.MD03=1;}
    if(P->PackModule4Regs.bit.MBCOMErrFault==1){P->ModuleSubCANErrCReg.bit.MD04=1;}
    if(P->PackModule5Regs.bit.MBCOMErrFault==1){P->ModuleSubCANErrCReg.bit.MD05=1;}
    if(P->PackModule6Regs.bit.MBCOMErrFault==1){P->ModuleSubCANErrCReg.bit.MD06=1;}
    if(P->PackModule7Regs.bit.MBCOMErrFault==1){P->ModuleSubCANErrCReg.bit.MD07=1;}
    */
  //  if(P->PackStateReg.bit.EXCANCOMERR==1|| P->PackStateReg.bit.CTCOMErr==1   || P->PackStateReg.bit.INCANCOMERR==1)
  //  {
      //  P->PackStateReg.bit.CANCOMERR=1;
  //  }

}
void SysCurrentHandle(SystemReg *s)
{
    long  CurrentCT  = 0;
    float32 Currentbuf = 0;
    CurrentCT  = s->CurrentData.all;
    CurrentCT  =  CurrentCT - 0x80000000;

    Currentbuf      =  ((float32)CurrentCT)/1000;     // (mA to A) CAB500 resolution 1mA
    s->PackCurrentF  = C_CTDirection * Currentbuf;    // Decide Current sensor's direction

    if(s->PackCurrentF>=500.0)
    {
        s->PackCurrentF=500.0;
    }
    if(s->PackCurrentF<=-500.0)
    {
        s->PackCurrentF=-500.0;
    }
    if(s->PackCurrentF < 0)
    {
        s->PackCurrentAsbF =-1.0 * s->PackCurrentF;
    }
    else
    {
        s->PackCurrentAsbF =s->PackCurrentF;
    }
    // TODOS : [검증] (27, 충/방전 모드 판정 - 전류 부호 기준(SOC 누적 규약상 +=충전, -=방전). 실차 부호 검증 필요)
    if(s->PackCurrentF < 0.0)
    {
        s->PackStateReg.bit.SysDisCharMode = 1u;   // 방전
    }
    else
    {
        s->PackStateReg.bit.SysDisCharMode = 0u;   // 충전
    }
}

/* ----------------------------------------------------------------------------
 * 경고(Warning, BPA_Protect_Status 0x01) 판정 - R01, 모든 임계/복귀값 parameter.h 참조
 * TODOS : [검증] (28, R01 경고 판정 활성화(일원화))
 * -------------------------------------------------------------------------- */
void CalSysAlarmtCheck(SystemReg *s)
{
    const Uint16 AlarmDelayCnt = C_AlarmFaultDelayCnt;   /* 100ms @ 1ms(cpu_timer0) ISR */
    /*----------------------------------------------------------------------
     0. 과전류 경고 (절대전류 기준, 충/방전 분리)
        방전 : ON = 150.0A, OFF = 145.5A    충전 : ON = 70.0A, OFF = 67.2A
    ----------------------------------------------------------------------*/
    if(s->PackStateReg.bit.SysDisCharMode == 1u)   /* 방전 */
    {
        if(Hyst_On(s->PackCurrentAsbF, C_PackOVPackCurrentDisCharAlarm))
        {
            if(s->SysAlarmCont[0] < AlarmDelayCnt) { ++s->SysAlarmCont[0]; }
            if(s->SysAlarmCont[0] >= AlarmDelayCnt) { s->PackAlarmReg.bit.PackVCur_OC = 1u; }
        }
        else
        {
            if(s->PackAlarmReg.bit.PackVCur_OC == 0u) { s->SysAlarmCont[0] = 0u; }
            if(Hyst_Off(s->PackCurrentAsbF, C_PackOVPackCurrentDisCharAlarmRls))
            {
                s->SysAlarmCont[0] = 0u;
                s->PackAlarmReg.bit.PackVCur_OC = 0u;
            }
        }
    }
    else                                           /* 충전 */
    {
        if(Hyst_On(s->PackCurrentAsbF, C_PackOVPackCurrentCharAlarm))
        {
            if(s->SysAlarmCont[0] < AlarmDelayCnt) { ++s->SysAlarmCont[0]; }
            if(s->SysAlarmCont[0] >= AlarmDelayCnt) { s->PackAlarmReg.bit.PackVCur_OC = 1u; }
        }
        else
        {
            if(s->PackAlarmReg.bit.PackVCur_OC == 0u) { s->SysAlarmCont[0] = 0u; }
            if(Hyst_Off(s->PackCurrentAsbF, C_PackOVPackCurrentCharAlarmRls))
            {
                s->SysAlarmCont[0] = 0u;
                s->PackAlarmReg.bit.PackVCur_OC = 0u;
            }
        }
    }
    /*----------------------------------------------------------------------
     1. SOC High 경고   ON = 95.0%(이상), OFF = 90.25%(미만)
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackSOCF, C_PackOVPkACKSOCAlarm))
    {
        if(s->SysAlarmCont[1] < AlarmDelayCnt) { ++s->SysAlarmCont[1]; }
        if(s->SysAlarmCont[1] >= AlarmDelayCnt) { s->PackAlarmReg.bit.PackVSOC_OV = 1u; }
    }
    else
    {
        if(s->PackAlarmReg.bit.PackVSOC_OV == 0u) { s->SysAlarmCont[1] = 0u; }
        if(Hyst_Off(s->PackSOCF, C_PackOVPkACKSOCAlarmRls))
        {
            s->SysAlarmCont[1] = 0u;
            s->PackAlarmReg.bit.PackVSOC_OV = 0u;
        }
    }

    /*----------------------------------------------------------------------
     2. SOC Low 경고    ON = 15.0%(미만), OFF = 16.0%(이상)
    ----------------------------------------------------------------------*/
    if(Hyst_Off(s->PackSOCF, C_PackUDPkACKSOCAlarm))
    {
        if(s->SysAlarmCont[2] < AlarmDelayCnt) { ++s->SysAlarmCont[2]; }
        if(s->SysAlarmCont[2] >= AlarmDelayCnt) { s->PackAlarmReg.bit.PackVSOC_UN = 1u; }
    }
    else
    {
        if(s->PackAlarmReg.bit.PackVSOC_UN == 0u) { s->SysAlarmCont[2] = 0u; }
        if(Hyst_On(s->PackSOCF, C_PackUDPkACKSOCAlarmRls))
        {
            s->SysAlarmCont[2] = 0u;
            s->PackAlarmReg.bit.PackVSOC_UN = 0u;
        }
    }

    /*----------------------------------------------------------------------
     3. 팩 과전압 경고   ON = 688.8V(이상), OFF = 681.912V(미만)
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackVoltageF, C_PackOVPackVoltageAlarm))
    {
        if(s->SysAlarmCont[3] < AlarmDelayCnt) { ++s->SysAlarmCont[3]; }
        if(s->SysAlarmCont[3] >= AlarmDelayCnt) { s->PackAlarmReg.bit.PackVolt_OV = 1u; }
    }
    else
    {
        if(s->PackAlarmReg.bit.PackVolt_OV == 0u) { s->SysAlarmCont[3] = 0u; }
        if(Hyst_Off(s->PackVoltageF, C_PackOVPackVoltageAlarmRls))
        {
            s->SysAlarmCont[3] = 0u;
            s->PackAlarmReg.bit.PackVolt_OV = 0u;
        }
    }

    /*----------------------------------------------------------------------
     4. 팩 저전압 경고   ON = 520.8V(이하), OFF = 526.008V(이상)
    ----------------------------------------------------------------------*/
    if(Hyst_Off(s->PackVoltageF, C_PackUDPackVoltageAlarm))
    {
        if(s->SysAlarmCont[4] < AlarmDelayCnt) { ++s->SysAlarmCont[4]; }
        if(s->SysAlarmCont[4] >= AlarmDelayCnt) { s->PackAlarmReg.bit.PackVolt_UN = 1u; }
    }
    else
    {
        if(s->PackAlarmReg.bit.PackVolt_UN == 0u) { s->SysAlarmCont[4] = 0u; }
        if(Hyst_On(s->PackVoltageF, C_PackUDPackVoltageAlarmRls))
        {
            s->SysAlarmCont[4] = 0u;
            s->PackAlarmReg.bit.PackVolt_UN = 0u;
        }
    }

    /*----------------------------------------------------------------------
     5. 팩 과온 경고   ON = 45.0C(이상), OFF = 43.2C(미만)   기준: 팩 평균 셀 온도
         (R01 미정의, 기존 parameter.h 팩온도 상수 사용)
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackCellAgvTemperatureF, C_PackOVPackTemperatureAlarm))
    {
        if(s->SysAlarmCont[5] < AlarmDelayCnt) { ++s->SysAlarmCont[5]; }
        if(s->SysAlarmCont[5] >= AlarmDelayCnt) { s->PackAlarmReg.bit.PackTemp_OT = 1u; }
    }
    else
    {
        if(s->PackAlarmReg.bit.PackTemp_OT == 0u) { s->SysAlarmCont[5] = 0u; }
        if(Hyst_Off(s->PackCellAgvTemperatureF, C_PackOVPackTemperatureAlarmRls))
        {
            s->SysAlarmCont[5] = 0u;
            s->PackAlarmReg.bit.PackTemp_OT = 0u;
        }
    }

    /*----------------------------------------------------------------------
     6. 팩 저온 경고 (충/방전 분리)   기준: 팩 평균 셀 온도   (셀 저온과 동일 규약)
         방전 : ON = 5.0C(이하), OFF = 5.2C(이상)    충전 : ON = 10.0C(이하), OFF = 10.4C(이상)
    ----------------------------------------------------------------------*/
    if(s->PackStateReg.bit.SysDisCharMode == 1u)   /* 방전 */
    {
        if(Hyst_Off(s->PackCellAgvTemperatureF, C_PackUNPackDisCharTemperatureAlarm))
        {
            if(s->SysAlarmCont[6] < AlarmDelayCnt) { ++s->SysAlarmCont[6]; }
            if(s->SysAlarmCont[6] >= AlarmDelayCnt) { s->PackAlarmReg.bit.PackTemp_UT = 1u; }
        }
        else
        {
            if(s->PackAlarmReg.bit.PackTemp_UT == 0u) { s->SysAlarmCont[6] = 0u; }
            if(Hyst_On(s->PackCellAgvTemperatureF, C_PackUNPackDisCharTemperatureAlarmRls))
            {
                s->SysAlarmCont[6] = 0u;
                s->PackAlarmReg.bit.PackTemp_UT = 0u;
            }
        }
    }
    else                                           /* 충전 */
    {
        if(Hyst_Off(s->PackCellAgvTemperatureF, C_PackUNPackCharTemperatureAlarm))
        {
            if(s->SysAlarmCont[6] < AlarmDelayCnt) { ++s->SysAlarmCont[6]; }
            if(s->SysAlarmCont[6] >= AlarmDelayCnt) { s->PackAlarmReg.bit.PackTemp_UT = 1u; }
        }
        else
        {
            if(s->PackAlarmReg.bit.PackTemp_UT == 0u) { s->SysAlarmCont[6] = 0u; }
            if(Hyst_On(s->PackCellAgvTemperatureF, C_PackUNPackCharTemperatureAlarmRls))
            {
                s->SysAlarmCont[6] = 0u;
                s->PackAlarmReg.bit.PackTemp_UT = 0u;
            }
        }
    }
    /*----------------------------------------------------------------------
     7. 셀 과전압 경고   ON = 4.10V(이상), OFF = 4.059V(미만)   기준: 최대 셀 전압
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackCellMaxVoltageF, C_PackOVCellVoltageAlarm))
    {
        if(s->SysAlarmCont[7] < AlarmDelayCnt) { ++s->SysAlarmCont[7]; }
        if(s->SysAlarmCont[7] >= AlarmDelayCnt) { s->PackAlarmReg.bit.CellVolt_OV = 1u; }
    }
    else
    {
        if(s->PackAlarmReg.bit.CellVolt_OV == 0u) { s->SysAlarmCont[7] = 0u; }
        if(Hyst_Off(s->PackCellMaxVoltageF, C_PackOVCellVoltageAlarmRls))
        {
            s->SysAlarmCont[7] = 0u;
            s->PackAlarmReg.bit.CellVolt_OV = 0u;
        }
    }
    /*----------------------------------------------------------------------
     8. 셀 저전압 경고   ON = 3.10V(이하), OFF = 3.131V(이상)   기준: 최소 셀 전압
    ----------------------------------------------------------------------*/
    if(Hyst_Off(s->PackCellMinVoltageF, C_PackUDCellVoltageAlarm))
    {
        if(s->SysAlarmCont[8] < AlarmDelayCnt) { ++s->SysAlarmCont[8]; }
        if(s->SysAlarmCont[8] >= AlarmDelayCnt) { s->PackAlarmReg.bit.CellVolt_UN = 1u; }
    }
    else
    {
        if(s->PackAlarmReg.bit.CellVolt_UN == 0u) { s->SysAlarmCont[8] = 0u; }
        if(Hyst_On(s->PackCellMinVoltageF, C_PackUDCellVoltageAlarmRls))
        {
            s->SysAlarmCont[8] = 0u;
            s->PackAlarmReg.bit.CellVolt_UN = 0u;
        }
    }
    /*----------------------------------------------------------------------
     9. 셀 전압 편차 경고   ON = 100mV(이상), OFF = 95mV(미만)
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackCellDivVoltageF, C_PackDIVCellVoltageAlarm))
    {
        if(s->SysAlarmCont[9] < AlarmDelayCnt) { ++s->SysAlarmCont[9]; }
        if(s->SysAlarmCont[9] >= AlarmDelayCnt) { s->PackAlarmReg.bit.CellVolt_BL = 1u; }
    }
    else
    {
        if(s->PackAlarmReg.bit.CellVolt_BL == 0u) { s->SysAlarmCont[9] = 0u; }
        if(Hyst_Off(s->PackCellDivVoltageF, C_PackDIVCellVoltageAlarmRls))
        {
            s->SysAlarmCont[9] = 0u;
            s->PackAlarmReg.bit.CellVolt_BL = 0u;
        }
    }

    /*----------------------------------------------------------------------
     10. 셀 과온 경고 (충/방전 동일)   ON = 45.0C(이상), OFF = 43.2C(미만)   기준: 최대 셀 온도
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackCellMaxTemperatureF, C_PackOVCellTemperatureAlarm))
    {
        if(s->SysAlarmCont[10] < AlarmDelayCnt) { ++s->SysAlarmCont[10]; }
        if(s->SysAlarmCont[10] >= AlarmDelayCnt) { s->PackAlarmReg.bit.CellTemp_OT = 1u; }
    }
    else
    {
        if(s->PackAlarmReg.bit.CellTemp_OT == 0u) { s->SysAlarmCont[10] = 0u; }
        if(Hyst_Off(s->PackCellMaxTemperatureF, C_PackOVCellTemperatureAlarmRls))
        {
            s->SysAlarmCont[10] = 0u;
            s->PackAlarmReg.bit.CellTemp_OT = 0u;
        }
    }

    /*----------------------------------------------------------------------
     11. 셀 저온 경고 (충/방전 분리)   기준: 최소 셀 온도
        방전 : ON = 5.0C(이하), OFF = 5.2C(이상)    충전 : ON = 10.0C(이하), OFF = 10.4C(이상)
    ----------------------------------------------------------------------*/
    if(s->PackStateReg.bit.SysDisCharMode == 1u)   /* 방전 */
    {
        if(Hyst_Off(s->PackCellMinTemperatureF, C_PackUDCellDisCharTemperatureAlarm))
        {
            if(s->SysAlarmCont[11] < AlarmDelayCnt) { ++s->SysAlarmCont[11]; }
            if(s->SysAlarmCont[11] >= AlarmDelayCnt) { s->PackAlarmReg.bit.CellTemp_UT = 1u; }
        }
        else
        {
            if(s->PackAlarmReg.bit.CellTemp_UT == 0u) { s->SysAlarmCont[11] = 0u; }
            if(Hyst_On(s->PackCellMinTemperatureF, C_PackUDCellDisCharTemperatureAlarmRls))
            {
                s->SysAlarmCont[11] = 0u;
                s->PackAlarmReg.bit.CellTemp_UT = 0u;
            }
        }
    }
    else                                           /* 충전 */
    {
        if(Hyst_Off(s->PackCellMinTemperatureF, C_PackUDCellCharTemperatureAlarm))
        {
            if(s->SysAlarmCont[11] < AlarmDelayCnt) { ++s->SysAlarmCont[11]; }
            if(s->SysAlarmCont[11] >= AlarmDelayCnt) { s->PackAlarmReg.bit.CellTemp_UT = 1u; }
        }
        else
        {
            if(s->PackAlarmReg.bit.CellTemp_UT == 0u) { s->SysAlarmCont[11] = 0u; }
            if(Hyst_On(s->PackCellMinTemperatureF, C_PackUDCellCharTemperatureAlarmRls))
            {
                s->SysAlarmCont[11] = 0u;
                s->PackAlarmReg.bit.CellTemp_UT = 0u;
            }
        }
    }

    /*----------------------------------------------------------------------
     12. 셀 온도 편차 경고   ON = 10.0C(이상), OFF = 9.5C(미만)
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackCellDivTemperatureF, C_PackDIVCellTemperatureAlarm))
    {
        if(s->SysAlarmCont[12] < AlarmDelayCnt) { ++s->SysAlarmCont[12]; }
        if(s->SysAlarmCont[12] >= AlarmDelayCnt) { s->PackAlarmReg.bit.CellTemp_BL = 1u; }
    }
    else
    {
        if(s->PackAlarmReg.bit.CellTemp_BL == 0u) { s->SysAlarmCont[12] = 0u; }
        if(Hyst_Off(s->PackCellDivTemperatureF, C_PackDIVCellTemperatureAlarmRls))
        {
            s->SysAlarmCont[12] = 0u;
            s->PackAlarmReg.bit.CellTemp_BL = 0u;
        }
    }

    /*----------------------------------------------------------------------
     13~15. 셀 내부저항(IR) 경고 (신규, 0x603 bit13-15)
            측정 미구현 시(PackCellMaxIRF==0) 전체 가드로 비활성 → 오판정 방지
            임계/복귀값 placeholder(parameter.h). 측정/임계 확정 필요
            TODOS : [검증] (52, 셀 IR 경고 - 측정/임계 확정 후 가드 해제)
    ----------------------------------------------------------------------*/
    /*--------------------------------------------------------------
     * 260615 : 0x603 Warn R58/R59 재배치로 CellIR 경고(옛 bit13~15) 폐기
     *          SystemAlarm_BIT에서 CellIR_OV/UN/DIV 삭제됨 → IR 판정 블록 비활성(빌드 에러 해소)
     *          R59는 해당 자리를 UnbaPWR 경고로 대체. 측정/임계 확정 시 재설계
     *--------------------------------------------------------------*/
#if 0   // TODOS : [삭제] (76, R59 IR경고 폐기 - CellIR_* 필드 삭제로 비활성)
    if(s->PackCellMaxIRF > 0.0f)   /* IR 측정 유효 시에만 판정 */
    {
        /* 13. 셀 IR 과대   ON = 100.0(이상), OFF = 95.0(미만)   기준: 최대 셀 IR */
        if(Hyst_On(s->PackCellMaxIRF, C_PackOVCellIRAlarm))
        {
            if(s->SysAlarmCont[13] < AlarmDelayCnt) { ++s->SysAlarmCont[13]; }
            if(s->SysAlarmCont[13] >= AlarmDelayCnt) { s->PackAlarmReg.bit.CellIR_OV = 1u; }
        }
        else
        {
            if(s->PackAlarmReg.bit.CellIR_OV == 0u) { s->SysAlarmCont[13] = 0u; }
            if(Hyst_Off(s->PackCellMaxIRF, C_PackOVCellIRAlarmRls))
            {
                s->SysAlarmCont[13] = 0u;
                s->PackAlarmReg.bit.CellIR_OV = 0u;
            }
        }

        /* 14. 셀 IR 과소   ON = 1.0(이하), OFF = 1.05(이상)   기준: 최소 셀 IR */
        if(Hyst_Off(s->PackCellMinIRF, C_PackUDCellIRAlarm))
        {
            if(s->SysAlarmCont[14] < AlarmDelayCnt) { ++s->SysAlarmCont[14]; }
            if(s->SysAlarmCont[14] >= AlarmDelayCnt) { s->PackAlarmReg.bit.CellIR_UN = 1u; }
        }
        else
        {
            if(s->PackAlarmReg.bit.CellIR_UN == 0u) { s->SysAlarmCont[14] = 0u; }
            if(Hyst_On(s->PackCellMinIRF, C_PackUDCellIRAlarmRls))
            {
                s->SysAlarmCont[14] = 0u;
                s->PackAlarmReg.bit.CellIR_UN = 0u;
            }
        }

        /* 15. 셀 IR 편차   ON = 50.0(이상), OFF = 47.5(미만)   기준: 셀 IR 편차 */
        if(Hyst_On(s->PackCellDivIRF, C_PackDIVCellIRAlarm))
        {
            if(s->SysAlarmCont[15] < AlarmDelayCnt) { ++s->SysAlarmCont[15]; }
            if(s->SysAlarmCont[15] >= AlarmDelayCnt) { s->PackAlarmReg.bit.CellIR_DIV = 1u; }
        }
        else
        {
            if(s->PackAlarmReg.bit.CellIR_DIV == 0u) { s->SysAlarmCont[15] = 0u; }
            if(Hyst_Off(s->PackCellDivIRF, C_PackDIVCellIRAlarmRls))
            {
                s->SysAlarmCont[15] = 0u;
                s->PackAlarmReg.bit.CellIR_DIV = 0u;
            }
        }
    }
#endif
}

/* ----------------------------------------------------------------------------
 * Fault (BPA_Protect_Status 0x02) 판정 - R01, 임계/복귀값 parameter.h 참조
 * TODOS : [검증] (29, R01 Fault 판정 활성화)
 * -------------------------------------------------------------------------- */
void CalSysFaultCheck(SystemReg *s)
{

    /*----------------------------------------------------------------------
     0. 과전류 Fault (절대전류 기준, 충/방전 분리)
        방전 : ON = 200.0A, OFF = 170.0A    충전 : ON = 80.0A, OFF = 76.0A
    ----------------------------------------------------------------------*/
    if(s->PackStateReg.bit.SysDisCharMode == 1u)   /* 방전 */
    {
        if(Hyst_On(s->PackCurrentAsbF, C_PackOVPackCurrentDisCharFault))
        {
            if(s->SysFalutCont[0] < C_OcFaultDelayCnt) { ++s->SysFalutCont[0]; }
            if(s->SysFalutCont[0] >= C_OcFaultDelayCnt) { s->PackFaultReg.bit.PackVCur_OC = 1u; }
        }
        else
        {
            if(s->PackFaultReg.bit.PackVCur_OC == 0u) { s->SysFalutCont[0] = 0u; }
            if(Hyst_Off(s->PackCurrentAsbF, C_PackOVPackCurrentDisCharFaultRls))
            {
                s->SysFalutCont[0] = 0u;
                s->PackFaultReg.bit.PackVCur_OC = 0u;
            }
        }
    }
    else                                           /* 충전 */
    {
        if(Hyst_On(s->PackCurrentAsbF, C_PackOVPackCurrentCharFault))
        {
            if(s->SysFalutCont[0] < C_OcFaultDelayCnt) { ++s->SysFalutCont[0]; }
            if(s->SysFalutCont[0] >= C_OcFaultDelayCnt) { s->PackFaultReg.bit.PackVCur_OC = 1u; }
        }
        else
        {
            if(s->PackFaultReg.bit.PackVCur_OC == 0u) { s->SysFalutCont[0] = 0u; }
            if(Hyst_Off(s->PackCurrentAsbF, C_PackOVPackCurrentCharFaultRls))
            {
                s->SysFalutCont[0] = 0u;
                s->PackFaultReg.bit.PackVCur_OC = 0u;
            }
        }
    }
    /*----------------------------------------------------------------------
     1. SOC High Fault   ON = 100.0%(이상), OFF = 97.0%(미만)
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackSOCF, C_PackOVPackSOCFault))
    {
        if(s->SysFalutCont[1] < C_SocOvFaultDelayCnt) { ++s->SysFalutCont[1]; }
        if(s->SysFalutCont[1] >= C_SocOvFaultDelayCnt) { s->PackFaultReg.bit.PackVSOC_OV = 1u; }
    }
    else
    {
        if(s->PackFaultReg.bit.PackVSOC_OV == 0u) { s->SysFalutCont[1] = 0u; }
        if(Hyst_Off(s->PackSOCF, C_PackOVPackSOCFaultRls))
        {
            s->SysFalutCont[1] = 0u;
            s->PackFaultReg.bit.PackVSOC_OV = 0u;
        }
    }

    /*----------------------------------------------------------------------
     2. SOC Low Fault    ON = 5.0%(미만), OFF = 5.5%(이상)
    ----------------------------------------------------------------------*/
    if(Hyst_Off(s->PackSOCF, C_PackUDPackSOCFault))
    {
        if(s->SysFalutCont[2] < C_SocUnFaultDelayCnt) { ++s->SysFalutCont[2]; }
        if(s->SysFalutCont[2] >= C_SocUnFaultDelayCnt) { s->PackFaultReg.bit.PackVSOC_UN = 1u; }
    }
    else
    {
        if(s->PackFaultReg.bit.PackVSOC_UN == 0u) { s->SysFalutCont[2] = 0u; }
        if(Hyst_On(s->PackSOCF, C_PackUDPackSOCFaultRls))
        {
            s->SysFalutCont[2] = 0u;
            s->PackFaultReg.bit.PackVSOC_UN = 0u;
        }
    }

    /*----------------------------------------------------------------------
     3. 팩 과전압 Fault   ON = 697.2V(이상), OFF = 693.714V(미만)
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackVoltageF, C_PackOVPackVoltageFault))
    {
        if(s->SysFalutCont[3] < C_VoltOvFaultDelayCnt) { ++s->SysFalutCont[3]; }
        if(s->SysFalutCont[3] >= C_VoltOvFaultDelayCnt) { s->PackFaultReg.bit.PackVolt_OV = 1u; }
    }
    else
    {
        if(s->PackFaultReg.bit.PackVolt_OV == 0u) { s->SysFalutCont[3] = 0u; }
        if(Hyst_Off(s->PackVoltageF, C_PackOVPackVoltageFaultRls))
        {
            s->SysFalutCont[3] = 0u;
            s->PackFaultReg.bit.PackVolt_OV = 0u;
        }
    }

    /*----------------------------------------------------------------------
     4. 팩 저전압 Fault   ON = 504.0V(이하), OFF = 511.56V(이상)
    ----------------------------------------------------------------------*/
    if(Hyst_Off(s->PackVoltageF, C_PackUDPackVoltageFault))
    {
        if(s->SysFalutCont[4] < C_VoltUnFaultDelayCnt) { ++s->SysFalutCont[4]; }
        if(s->SysFalutCont[4] >= C_VoltUnFaultDelayCnt) { s->PackFaultReg.bit.PackVolt_UN = 1u; }
    }
    else
    {
        if(s->PackFaultReg.bit.PackVolt_UN == 0u) { s->SysFalutCont[4] = 0u; }
        if(Hyst_On(s->PackVoltageF, C_PackUDPackVoltageFaultRls))
        {
            s->SysFalutCont[4] = 0u;
            s->PackFaultReg.bit.PackVolt_UN = 0u;
        }
    }

    /*----------------------------------------------------------------------
     5. 팩 과온 Fault   ON = 50.0C(이상), OFF = 47.5C(미만)   기준: 팩 평균 셀 온도
         (R01 미정의, 기존 parameter.h 팩온도 상수 사용)
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackCellAgvTemperatureF, C_PackOVPackTemperatureFault))
    {
        if(s->SysFalutCont[5] < C_PackTempOtFaultDelayCnt) { ++s->SysFalutCont[5]; }
        if(s->SysFalutCont[5] >= C_PackTempOtFaultDelayCnt) { s->PackFaultReg.bit.PackTemp_OT = 1u; }
    }
    else
    {
        if(s->PackFaultReg.bit.PackTemp_OT == 0u) { s->SysFalutCont[5] = 0u; }
        if(Hyst_Off(s->PackCellAgvTemperatureF, C_PackOVPackTemperatureFaultRls))
        {
            s->SysFalutCont[5] = 0u;
            s->PackFaultReg.bit.PackTemp_OT = 0u;
        }
    }

    /*----------------------------------------------------------------------
     6. 팩 저온 Fault (충/방전 분리)   기준: 팩 평균 셀 온도   (셀 저온과 동일 규약)
         방전 : ON = 0.0C(이하), OFF = 3.0C(이상)    충전 : ON = 0.0C(이하), OFF = 7.0C(이상)
    ----------------------------------------------------------------------*/
    if(s->PackStateReg.bit.SysDisCharMode == 1u)   /* 방전 */
    {
        if(Hyst_Off(s->PackCellAgvTemperatureF, C_PackUNPackDisCharTemperatureFault))
        {
            if(s->SysFalutCont[6] < C_PackTempUtFaultDelayCnt) { ++s->SysFalutCont[6]; }
            if(s->SysFalutCont[6] >= C_PackTempUtFaultDelayCnt) { s->PackFaultReg.bit.PackTemp_UT = 1u; }
        }
        else
        {
            if(s->PackFaultReg.bit.PackTemp_UT == 0u) { s->SysFalutCont[6] = 0u; }
            if(Hyst_On(s->PackCellAgvTemperatureF, C_PackUNPackDisCharTemperatureFaultRls))
            {
                s->SysFalutCont[6] = 0u;
                s->PackFaultReg.bit.PackTemp_UT = 0u;
            }
        }
    }
    else                                           /* 충전 */
    {
        if(Hyst_Off(s->PackCellAgvTemperatureF, C_PackUNPackCharTemperatureFalut))
        {
            if(s->SysFalutCont[6] < C_PackTempUtFaultDelayCnt) { ++s->SysFalutCont[6]; }
            if(s->SysFalutCont[6] >= C_PackTempUtFaultDelayCnt) { s->PackFaultReg.bit.PackTemp_UT = 1u; }
        }
        else
        {
            if(s->PackFaultReg.bit.PackTemp_UT == 0u) { s->SysFalutCont[6] = 0u; }
            if(Hyst_On(s->PackCellAgvTemperatureF, C_PackUNPackCharTemperatureFaultRls))
            {
                s->SysFalutCont[6] = 0u;
                s->PackFaultReg.bit.PackTemp_UT = 0u;
            }
        }
    }

    /*----------------------------------------------------------------------
     7. 셀 과전압 Fault   ON = 4.15V(이상), OFF = 4.12095V(미만)   기준: 최대 셀 전압
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackCellMaxVoltageF, C_PackOVCellVoltageFault))
    {
        if(s->SysFalutCont[7] < C_CellVoltOvFaultDelayCnt) { ++s->SysFalutCont[7]; }
        if(s->SysFalutCont[7] >= C_CellVoltOvFaultDelayCnt) { s->PackFaultReg.bit.CellVolt_OV = 1u; }
    }
    else
    {
        if(s->PackFaultReg.bit.CellVolt_OV == 0u) { s->SysFalutCont[7] = 0u; }
        if(Hyst_Off(s->PackCellMaxVoltageF, C_PackOVCellVoltageFaultRls))
        {
            s->SysFalutCont[7] = 0u;
            s->PackFaultReg.bit.CellVolt_OV = 0u;
        }
    }

    /*----------------------------------------------------------------------
     8. 셀 저전압 Fault   ON = 3.00V(이하), OFF = 3.06V(이상)   기준: 최소 셀 전압
    ----------------------------------------------------------------------*/
    if(Hyst_Off(s->PackCellMinVoltageF, C_PackUDCellVoltageFault))
    {
        if(s->SysFalutCont[8] < C_CellVoltUnFaultDelayCnt) { ++s->SysFalutCont[8]; }
        if(s->SysFalutCont[8] >= C_CellVoltUnFaultDelayCnt) { s->PackFaultReg.bit.CellVolt_UN = 1u; }
    }
    else
    {
        if(s->PackFaultReg.bit.CellVolt_UN == 0u) { s->SysFalutCont[8] = 0u; }
        if(Hyst_On(s->PackCellMinVoltageF, C_PackUDCellVoltageFaultRls))
        {
            s->SysFalutCont[8] = 0u;
            s->PackFaultReg.bit.CellVolt_UN = 0u;
        }
    }

    /*----------------------------------------------------------------------
     9. 셀 전압 편차 Fault   ON = 300mV(이상), OFF = 210mV(미만)
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackCellDivVoltageF, C_PackDIVCellVoltageFault))
    {
        if(s->SysFalutCont[9] < C_CellVoltBlFaultDelayCnt) { ++s->SysFalutCont[9]; }
        if(s->SysFalutCont[9] >= C_CellVoltBlFaultDelayCnt) { s->PackFaultReg.bit.CellVolt_BL = 1u; }
    }
    else
    {
        if(s->PackFaultReg.bit.CellVolt_BL == 0u) { s->SysFalutCont[9] = 0u; }
        if(Hyst_Off(s->PackCellDivVoltageF, C_PackDIVCellVoltageFaultRls))
        {
            s->SysFalutCont[9] = 0u;
            s->PackFaultReg.bit.CellVolt_BL = 0u;
        }
    }

    /*----------------------------------------------------------------------
     10. 셀 과온 Fault (충/방전 동일)   ON = 50.0C(이상), OFF = 47.5C(미만)   기준: 최대 셀 온도
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackCellMaxTemperatureF, C_PackOVCellTemperatureFault))
    {
        if(s->SysFalutCont[10] < C_CellTempOtFaultDelayCnt) { ++s->SysFalutCont[10]; }
        if(s->SysFalutCont[10] >= C_CellTempOtFaultDelayCnt) { s->PackFaultReg.bit.CellTemp_OT = 1u; }
    }
    else
    {
        if(s->PackFaultReg.bit.CellTemp_OT == 0u) { s->SysFalutCont[10] = 0u; }
        if(Hyst_Off(s->PackCellMaxTemperatureF, C_PackOVCellTemperatureFaultRls))
        {
            s->SysFalutCont[10] = 0u;
            s->PackFaultReg.bit.CellTemp_OT = 0u;
        }
    }

    /*----------------------------------------------------------------------
     11. 셀 저온 Fault (충/방전 분리)   기준: 최소 셀 온도
        방전 : ON = 0.0C(이하), OFF = 3.0C(이상)    충전 : ON = 0.0C(이하), OFF = 7.0C(이상)
    ----------------------------------------------------------------------*/
    if(s->PackStateReg.bit.SysDisCharMode == 1u)   /* 방전 */
    {
        if(Hyst_Off(s->PackCellMinTemperatureF, C_PackUDCellDisCharTemperatureFault))
        {
            if(s->SysFalutCont[11] < C_CellTempUtFaultDelayCnt) { ++s->SysFalutCont[11]; }
            if(s->SysFalutCont[11] >= C_CellTempUtFaultDelayCnt) { s->PackFaultReg.bit.CellTemp_UT = 1u; }
        }
        else
        {
            if(s->PackFaultReg.bit.CellTemp_UT == 0u) { s->SysFalutCont[11] = 0u; }
            if(Hyst_On(s->PackCellMinTemperatureF, C_PackUDCellDisCharTemperatureFaultRls))
            {
                s->SysFalutCont[11] = 0u;
                s->PackFaultReg.bit.CellTemp_UT = 0u;
            }
        }
    }
    else                                           /* 충전 */
    {
        if(Hyst_Off(s->PackCellMinTemperatureF, C_PackUDCellCharTemperatureFault))
        {
            if(s->SysFalutCont[11] < C_CellTempUtFaultDelayCnt) { ++s->SysFalutCont[11]; }
            if(s->SysFalutCont[11] >= C_CellTempUtFaultDelayCnt) { s->PackFaultReg.bit.CellTemp_UT = 1u; }
        }
        else
        {
            if(s->PackFaultReg.bit.CellTemp_UT == 0u) { s->SysFalutCont[11] = 0u; }
            if(Hyst_On(s->PackCellMinTemperatureF, C_PackUDCellCharTemperatureFaultRls))
            {
                s->SysFalutCont[11] = 0u;
                s->PackFaultReg.bit.CellTemp_UT = 0u;
            }
        }
    }

    /*----------------------------------------------------------------------
     12. 셀 온도 편차 Fault   ON = 13.0C(이상), OFF = 11.7C(미만)
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackCellDivTemperatureF, C_PackDIVCellTemperatureFault))
    {
        if(s->SysFalutCont[12] < C_CellTempBlFaultDelayCnt) { ++s->SysFalutCont[12]; }
        if(s->SysFalutCont[12] >= C_CellTempBlFaultDelayCnt) { s->PackFaultReg.bit.CellTemp_BL = 1u; }
    }
    else
    {
        if(s->PackFaultReg.bit.CellTemp_BL == 0u) { s->SysFalutCont[12] = 0u; }
        if(Hyst_Off(s->PackCellDivTemperatureF, C_PackDIVCellTemperatureFaultRls))
        {
            s->SysFalutCont[12] = 0u;
            s->PackFaultReg.bit.CellTemp_BL = 0u;
        }
    }

    /*----------------------------------------------------------------------
     13~15. 셀 내부저항(IR) Fault (신규, 0x603 bit13-15)
            측정 미구현 시(PackCellMaxIRF==0) 전체 가드로 비활성 → 오판정 방지
            임계/복귀값 placeholder(parameter.h). 측정/임계 확정 필요
            TODOS : [검증] (57, 셀 IR Fault - 측정/임계 확정 후 가드 해제)
    ----------------------------------------------------------------------*/
#if 0   // TODOS : [삭제] (79, R59 IR Fault 폐기 - CellIR_* 필드 삭제로 비활성)
    if(s->PackCellMaxIRF > 0.0f)   /* IR 측정 유효 시에만 판정 */
    {
        /* 13. 셀 IR 과대   기준: 최대 셀 IR */
        if(Hyst_On(s->PackCellMaxIRF, C_PackOVCellIRFault))
        {
            if(s->SysFalutCont[13] < C_CellIrOvFaultDelayCnt) { ++s->SysFalutCont[13]; }
            if(s->SysFalutCont[13] >= C_CellIrOvFaultDelayCnt) { s->PackFaultReg.bit.CellIR_OV = 1u; }
        }
        else
        {
            if(s->PackFaultReg.bit.CellIR_OV == 0u) { s->SysFalutCont[13] = 0u; }
            if(Hyst_Off(s->PackCellMaxIRF, C_PackOVCellIRFaultRls))
            {
                s->SysFalutCont[13] = 0u;
                s->PackFaultReg.bit.CellIR_OV = 0u;
            }
        }

        /* 14. 셀 IR 과소   기준: 최소 셀 IR */
        if(Hyst_Off(s->PackCellMinIRF, C_PackUDCellIRFault))
        {
            if(s->SysFalutCont[14] < C_CellIrUnFaultDelayCnt) { ++s->SysFalutCont[14]; }
            if(s->SysFalutCont[14] >= C_CellIrUnFaultDelayCnt) { s->PackFaultReg.bit.CellIR_UN = 1u; }
        }
        else
        {
            if(s->PackFaultReg.bit.CellIR_UN == 0u) { s->SysFalutCont[14] = 0u; }
            if(Hyst_On(s->PackCellMinIRF, C_PackUDCellIRFaultRls))
            {
                s->SysFalutCont[14] = 0u;
                s->PackFaultReg.bit.CellIR_UN = 0u;
            }
        }

        /* 15. 셀 IR 편차   기준: 셀 IR 편차 */
        if(Hyst_On(s->PackCellDivIRF, C_PackDIVCellIRFault))
        {
            if(s->SysFalutCont[15] < C_CellIrDivFaultDelayCnt) { ++s->SysFalutCont[15]; }
            if(s->SysFalutCont[15] >= C_CellIrDivFaultDelayCnt) { s->PackFaultReg.bit.CellIR_DIV = 1u; }
        }
        else
        {
            if(s->PackFaultReg.bit.CellIR_DIV == 0u) { s->SysFalutCont[15] = 0u; }
            if(Hyst_Off(s->PackCellDivIRF, C_PackDIVCellIRFaultRls))
            {
                s->SysFalutCont[15] = 0u;
                s->PackFaultReg.bit.CellIR_DIV = 0u;
            }
        }
    }
#endif
}

/* ----------------------------------------------------------------------------
 * 보호 (Protection, BPA_Protect_Status 0x03) 판정 - R01, 즉시 차단(Shutdown/Relay OPEN)
 * 한 번 set 되면 latch (시스템 리셋 시 해제). 임계값 parameter.h 참조
 * TODOS : [검증] (30, R01 보호 판정 활성화(즉시 차단))
 * -------------------------------------------------------------------------- */
void CalSysProtectCheck(SystemReg *s)
{
    /*----------------------------------------------------------------------
     0. 과전류 보호 (즉시 차단, latch)   방전 = 250.0A(이상), 충전 = 100.0A(이상)
    ----------------------------------------------------------------------*/
    if(s->PackStateReg.bit.SysDisCharMode == 1u)   /* 방전 */
    {
        if(Hyst_On(s->PackCurrentAsbF, C_PackOVPackCurrentDisCharProtect))
        {
            s->PackProtectReg.bit.PackVCT_OV = 1u;
        }
    }
    else                                           /* 충전 */
    {
        if(Hyst_On(s->PackCurrentAsbF, C_PackOVPackCurrentCharProtect))
        {
            s->PackProtectReg.bit.PackVCT_OV = 1u;
        }
    }

    /* 1. SOC 과충전 보호   103.0%(이상) */
    if(Hyst_On(s->PackSOCF, C_PackOVPackSOCProtect))
    {
        s->PackProtectReg.bit.PackVSOC_OV = 1u;
    }

    /* 2. SOC 과방전 보호   0.0%(미만) */
    if(Hyst_Off(s->PackSOCF, C_PackUDPackSOCProtect))
    {
        s->PackProtectReg.bit.PackVSOC_UN = 1u;
    }

    /* 3. 팩 과전압 보호   702.2V(이상) */
    if(Hyst_On(s->PackVoltageF, C_PackOVPackVoltageProtect))
    {
        s->PackProtectReg.bit.PackVolt_OV = 1u;
    }

    /* 4. 팩 저전압 보호   478.8V(이하) */
    if(Hyst_Off(s->PackVoltageF, C_PackUDPackVoltageProtect))
    {
        s->PackProtectReg.bit.PackVolt_UN = 1u;
    }

    /* 5. 팩 과온 보호   54.0C(이상)   기준: 팩 평균 셀 온도 */
    if(Hyst_On(s->PackCellAgvTemperatureF, C_PackOVPackTemperatureProtect))
    {
        s->PackProtectReg.bit.PackTemp_OV = 1u;
    }
    /*----------------------------------------------------------------------
     6. 팩 저온 보호 (충/방전 분리)   방전 = -15.0C(이하), 충전 = -10.0C(이하)   기준: 팩 평균 셀 온도
    ----------------------------------------------------------------------*/
    if(s->PackStateReg.bit.SysDisCharMode == 1u)   /* 방전 */
    {
        if(Hyst_Off(s->PackCellAgvTemperatureF, C_PackUNPackDisCharTemperatureProtect))
        {
            s->PackProtectReg.bit.PackTemp_UN = 1u;
        }
    }
    else                                           /* 충전 */
    {
        if(Hyst_Off(s->PackCellAgvTemperatureF, C_PackUNPackCharTemperatureProtect))
        {
            s->PackProtectReg.bit.PackTemp_UN = 1u;
        }
    }

    /* 7. 셀 과전압 보호   4.18V(이상)   기준: 최대 셀 전압 */
    if(Hyst_On(s->PackCellMaxVoltageF, C_PackOVCellVoltageProtect))
    {
        s->PackProtectReg.bit.CellVolt_OV = 1u;
    }

    /* 8. 셀 저전압 보호   2.85V(이하)   기준: 최소 셀 전압 */
    if(Hyst_Off(s->PackCellMinVoltageF, C_PackUDCellVoltageProtect))
    {
        s->PackProtectReg.bit.CellVolt_UN = 1u;
    }

    /* 9. 셀 전압 편차 보호   400mV(이상) */
    if(Hyst_On(s->PackCellDivVoltageF, C_PackDIVCellVoltageProtect))
    {
        s->PackProtectReg.bit.CellVolt_BL = 1u;
    }

    /* 10. 셀 과온 보호   54.0C(이상)   기준: 최대 셀 온도 */
    if(Hyst_On(s->PackCellMaxTemperatureF, C_PackOVCellTemperatureProtect))
    {
        s->PackProtectReg.bit.CellTemp_OV = 1u;
    }

    /*----------------------------------------------------------------------
     11. 셀 저온 보호 (충/방전 분리)   방전 = -15.0C(이하), 충전 = -10.0C(이하)   기준: 최소 셀 온도
    ----------------------------------------------------------------------*/
    if(s->PackStateReg.bit.SysDisCharMode == 1u)   /* 방전 */
    {
        if(Hyst_Off(s->PackCellMinTemperatureF, C_PackUDCellDisCharTemperatureProtect))
        {
            s->PackProtectReg.bit.CellTemp_UN = 1u;
        }
    }
    else                                           /* 충전 */
    {
        if(Hyst_Off(s->PackCellMinTemperatureF, C_PackUDCellCharTemperatureProtect))
        {
            s->PackProtectReg.bit.CellTemp_UN = 1u;
        }
    }

    /* 12. 셀 온도 편차 보호   15.0C(이상) */
    if(Hyst_On(s->PackCellDivTemperatureF, C_PackDIVCellTemperatureProtectt))
    {
        s->PackProtectReg.bit.CellTemp_BLT = 1u;
    }

    /*----------------------------------------------------------------------
     13~15. 셀 내부저항(IR) 보호 (신규, 0x603 bit13-15, 즉시 차단)
            측정 미구현 시(PackCellMaxIRF==0) 가드로 비활성 → 오판정 방지. 임계 placeholder
            TODOS : [검증] (59, 셀 IR 보호 - 측정/임계 확정 후 가드 해제)
    ----------------------------------------------------------------------*/
#if 0   // TODOS : [삭제] (80, R59 IR 보호 폐기 - CellIR_* 필드 삭제로 비활성)
    if(s->PackCellMaxIRF > 0.0f)   /* IR 측정 유효 시에만 판정 */
    {
        /* 13. 셀 IR 과대   기준: 최대 셀 IR */
        if(Hyst_On(s->PackCellMaxIRF, C_PackOVCellIRProtect))
        {
            s->PackProtectReg.bit.CellIR_OV = 1u;
        }

        /* 14. 셀 IR 과소   기준: 최소 셀 IR */
        if(Hyst_Off(s->PackCellMinIRF, C_PackUDCellIRProtect))
        {
            s->PackProtectReg.bit.CellIR_UN = 1u;
        }

        /* 15. 셀 IR 편차   기준: 셀 IR 편차 */
        if(Hyst_On(s->PackCellDivIRF, C_PackDIVCellIRProtect))
        {
            s->PackProtectReg.bit.CellIR_DIV = 1u;
        }
    }
#endif
    /*----------------------------------------------------------------------
     16. 방전 불평형전력 보호 (즉시 차단)   ON = 300kW(이상)   기준: 방전 전력 피크
         ※ 불평형전력 전용 측정값 없어 기존 피크전력(PackDisCHAPWRPeakF)으로 대체
         TODOS : [검증] (62, 불평형전력 보호 - 피크전력 대체, 전용 측정 구현 시 교체)
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackDisCHAPWRPeakF, C_PackDisCharUnBalPWRProtect))
    {
        s->PackProtectReg.bit.PackUnbaDisCh_UbPWR = 1u;
    }
    /*----------------------------------------------------------------------
     17. 충전 불평형전력 보호 (즉시 차단)   ON = 150kW(이상)   기준: 충전 전력 피크(PackCHAPWRPeakF)
    ----------------------------------------------------------------------*/
    if(Hyst_On(s->PackCHAPWRPeakF, C_PackCharUnBalPWRProtect))
    {
        s->PackProtectReg.bit.PackUnbaCahr_UbPWR = 1u;
    }
}
int float32ToInt(float32 Vaule, Uint32 Num)
{
    Uint32 intVaule=0;
    intVaule = roundf(Vaule*10)/10;

    return (Uint32)intVaule;
}

void DigitalInput(SystemReg *sys)
{
   // unsigned int IDVaule=0;
    if(IDSW00==0)
    {
        sys->DigitalInputReg.bit.DipSW00=1;
    }
    else
    {
        sys->DigitalInputReg.bit.DipSW00=0;
    }
    if(IDSW01==0)
    {
        sys->DigitalInputReg.bit.DipSW01=1;
    }
    else
    {
        sys->DigitalInputReg.bit.DipSW01=0;
    }
    if(IDSW02==0)
    {
        sys->DigitalInputReg.bit.DipSW02=1;
    }
    else
    {
        sys->DigitalInputReg.bit.DipSW02=0;
    }
    /*--------------------------------------------------------------
     * 260613 : GPIO19(IDSW03) HW 이슈 - 외부 회로(RS485B 잔재)로 핀이 low 고정
     *          → SW03 입력 무시, 강제 0 처리 (HW 수정 전까지 임시 우회)
     *          Rack 식별은 SW00/SW01(PackSWID 0~3)만으로 충분
     *--------------------------------------------------------------*/
    //if(IDSW03==0)
    //{
    //    sys->DigitalInputReg.bit.DipSW03=1;
    //}
    //else
    //{
    //    sys->DigitalInputReg.bit.DipSW03=0;
    //}
    sys->DigitalInputReg.bit.DipSW03=0;   // TODOS : [검증] (63, GPIO19 HW고장 우회: SW03 강제 0)
    if(CANRX0INT==0)
    {
        sys->DigitalInputReg.bit.CANRX0=1;
    }
    else
    {
        sys->DigitalInputReg.bit.CANRX0=0;
    }
    if(CANRX1INT==0)
    {
        sys->DigitalInputReg.bit.CANRX1=1;
    }
    else
    {
        sys->DigitalInputReg.bit.CANRX1=0;
    }

    if(PRlyState==0)
    {
        sys->DigitalInputReg.bit.PAUX=1;
    }
    else
    {
        sys->DigitalInputReg.bit.PAUX=0;
    }
    if(NRlyState==0)
    {
        sys->DigitalInputReg.bit.NAUX=1;
    }
    else
    {
        sys->DigitalInputReg.bit.NAUX=0;
    }
    if(EMGSWDI==0)
    {
        sys->DigitalInputReg.bit.EMGSWStauts=1;
    }
    else
    {
        sys->DigitalInputReg.bit.EMGSWStauts=0;
    }
    /*--------------------------------------------------------------
     * 260612 (No.31) : PackID 인식 로직 개선 + 디버깅 변수 분리
     *   - PackSWID 신설: 실제 DIP 니블(0~15) 보존(클램프X) → 디버깅용
     *   - 유효화 조건 ==0x00f0(TEST만) → >0x0030(0x40 이상 전부)
     *     DIP 4~15(TEST 0xF0 포함) → 0x0000(Rack1) 인식
     *--------------------------------------------------------------*/
    //--- 변경 전 ---
    //IDVaule = sys->DigitalInputReg.all & 0x000f;
    //sys->PackID = IDVaule << 4;
    //if(sys->PackID==0x00f0) { sys->PackID=0x0000; }

    //--- 변경 후 ---
    /*--------------------------------------------------------------
     * 260613 : GPIO19(SW03) HW 고장으로 bit3 신뢰불가 → PackSWID 계산에서 bit3 마스킹
     *          (SW03 강제0 우회와 이중 안전). Rack 식별은 SW00/SW01(0~3)만 사용
     *--------------------------------------------------------------*/
    //sys->PackSWID = sys->DigitalInputReg.all & 0x000f;   // (구) DIP 니블 4비트 전체
    sys->PackSWID = sys->DigitalInputReg.all & 0x0007;     // 디버깅 표시용(현재 PackID 미반영)
    sys->PackID   = sys->PackSWID << 4;
    if(sys->PackID > 0x0030)
    {
        sys->PackID = 0x0000;                            // 0x40 이상 → Rack1
    }
}
void DigitalOutput(SystemReg *sys)
{

    if(sys->DigitalOutPutReg.bit.NRlyOUT==1)
    {
        NRlyOn;
    }
    else
    {
        NRlyOff;
    }
    if(sys->DigitalOutPutReg.bit.ProRlyOUT==1)
    {
        PRORlyOn;
    }
    else
    {
        PRORlyOff;
    }
    if(sys->DigitalOutPutReg.bit.PRlyOUT==1)
    {
        PRlyOn;
    }
    else
    {
        PRlyOff;
    }
    if(sys->DigitalOutPutReg.bit.PWRLAMPOUT==1)
    {
        PWRLAMPOn;
    }
    else
    {
        sys->PWRLAMPCount++;
        if(sys->PWRLAMPCount>1200)
        {
            PWRLAMPTog;
            sys->PWRLAMPCount=0;
        }
    }
    if(sys->DigitalOutPutReg.bit.EMGSWLAMPOUT==1)
    {
        EMGSWLAMPOn;
    }
    else
    {
        EMGSWLAMPOff;
    }
    if(sys->DigitalOutPutReg.bit.LEDAlarmOUT==1)
    {
        sys->LEDFaultCount++;
        if(sys->LEDFaultCount>1200)
        {
            LEDFault_T;
            sys->LEDFaultCount=0;
        }
    }
    if(sys->DigitalOutPutReg.bit.LEDFaultOUT==1)
    {
        sys->LEDFaultCount++;

    }

    if(sys->DigitalOutPutReg.bit.LEDProtectOUT==1)
    {
        sys->LEDFaultCount++;
        if(sys->LEDFaultCount>200)
        {
            LEDFault_T;
            sys->LEDFaultCount=0;
        }
    }
    if((sys->DigitalOutPutReg.bit.LEDAlarmOUT==0)&&(sys->DigitalOutPutReg.bit.LEDFaultOUT==0)&&(sys->DigitalOutPutReg.bit.LEDProtectOUT==0))
    {
        LEDFault_H;
    }
    if(sys->DigitalOutPutReg.bit.LEDSysOUT==1)
    {
        sys->LEDSycCount++;
        if(sys->LEDSycCount>500)
        {
      //      LEDSysState_T;
            sys->LEDSycCount=0;
        }
    }
    else
    {
       // LEDSysState_H;
    }
    if(sys->DigitalOutPutReg.bit.LEDCAnOUT==1)
    {
  
        sys->LEDCanCount++;
        if(sys->LEDCanCount>500)
        {
            LEDCANState_T;
            sys->LEDCanCount=0;
        }
    }
    else
    {
        LEDCANState_H;
    }

}

void TimerinitHandle(TimerReg *timer)
{
  timer->state = TIMER_STATE_IDLE;
  timer->TimeCount = 0;
  timer->Start=0;
  timer->Stop=0;
  timer->OutState=0;
  timer->Reset=0;
  timer->TimerVaule=0;
}
void ProtectRelayTimerHandle(TimerReg *timer)
{
  switch (timer->state)
  {
    case TIMER_STATE_IDLE:
      // 타이머 시작
      if (timer->Start==1)
      {
        timer->state = TIMER_STATE_RUNNING;
      }
      if (timer->Reset==1)
      {
         timer->state = TIMER_STATE_CLEAR;
      }
      break;
    case TIMER_STATE_RUNNING:
      // 타이머 만료 확인
      if (timer->TimeCount >= timer->TimerVaule)
      {
        timer->state = TIMER_STATE_EXPIRED;
      }
      // 타이머 증가
      timer->TimeCount++;
      break;
    case TIMER_STATE_EXPIRED:
         // 타이머 만료 처리
         timer->OutState = 1;
         // 타이머 재시작
         if (timer->Reset==1)
         {
            timer->state = TIMER_STATE_CLEAR;
         }
    break;
    case TIMER_STATE_CLEAR:
         timer->state = TIMER_STATE_IDLE;
         timer->TimeCount = 0;
         timer->Start=0;
         timer->Stop=0;
         timer->OutState=0;
         timer->TimerVaule=0;
         timer->Reset=0;
    break;

  }
}
void TestCurrentlimt_StepUpdate_200ms(SystemReg *p)
{
    /* SOC 1% 증가 */
    p->socPct_f += 1.0f;

    if (p->socPct_f > 100.0f)
    {
        p->socPct_f = 0.0f;

        /* Temp 1℃ 증가 */
        p->tempC_f += 1.0f;

        if (p->tempC_f > 60.0f)
        {
            p->tempC_f = -30.0f;
        }
    }
}

//extern MODReg MODRegs;
/*
extern void InitECan(void);
extern void SciaTxchar(char Txchar);

extern void SPI_Write(unsigned int WRData);
extern unsigned int SPI_READ(void);
extern void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3 );
 
extern void ModuleCellMIN(SlaveReg *S);
extern void ModuleCellMAX(SlaveReg *S);
//extern void BatteryTeperaureCal(BatteryReg *bat,int Count);




extern void TEST_LTC6804(void);
extern int LTC6804_Init(void);

extern void SlaveBMSSPIEnable_low(void);
extern void SlaveBMSSPIEnable_high(void);
extern void SlaveBms_WakeUp(void);
extern int SlaveBMSCellReadCommand(SlaveReg *s);
extern void SlaveBmsBalance(SlaveReg *s);


extern float32 AbsVaule(float32 input);
extern void CellVoltagetoFloat(SlaveReg *s, ModuleReg *m);

extern void SystemAlarm(PackReg *bat,SystemReg *sys);
extern void SystemRegsInit(SystemReg *s);
extern void systemParmRead(SystemReg *sys,PackReg *bat);
extern void sysDigitalInput(SystemReg *sys);
extern void sysDigitalOutput(SystemReg *sys);
extern void SysCurrentOffset(SystemReg *sys);
extern void SystemforceBalance(ModuleReg *m);


extern void DigitalInput(SystemReg *sys);
extern void DigitalOutput(SystemReg *sys);
extern void SlaveRegsInit(SlaveReg *s);
extern void ModuleRegsInit(ModuleReg *m);
extern void PackRegsInit(PackReg *b);
extern void SystemInit(SystemReg *s);
extern void CellTempCalFloat(SlaveReg *s, ModuleReg *m);
extern void CellVOLTFloatTOInt(ModuleReg *s, CANTXReg *t);
extern void CellTempFloatToInt(ModuleReg *s, CANTXReg *t);

void CellTempFloatToInt(ModuleReg *s, CANTXReg *t)
{
	t->BatteryTempCell[t->CellNumer]  =(int)(s->CellTempF[0]*10.0);
	t->BatteryTempCell[t->CellNumer+1]=(int)(s->CellTempF[1]*10.0);
	t->BatteryTempCell[t->CellNumer+2]=(int)(s->CellTempF[2]*10.0);
	t->BatteryTempCell[t->CellNumer+3]=(int)(s->CellTempF[3]*10.0);
}
void CellVOLTFloatTOInt(ModuleReg *s, CANTXReg *t)
{
	t->BatteryVoltageCell[t->CellNumer]=(unsigned int)(s->CellVoltageF[0]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+1]=(unsigned int)(s->CellVoltageF[1]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+2]=(unsigned int)(s->CellVoltageF[2]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+3]=(unsigned int)(s->CellVoltageF[3]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+4]=(unsigned int)(s->CellVoltageF[4]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+5]=(unsigned int)(s->CellVoltageF[5]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+6]=(unsigned int)(s->CellVoltageF[6]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+7]=(unsigned int)(s->CellVoltageF[7]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+8]=(unsigned int)(s->CellVoltageF[8]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+9]=(unsigned int)(s->CellVoltageF[9]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+10]=(unsigned int)(s->CellVoltageF[10]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+11]=(unsigned int)(s->CellVoltageF[11]*1000.0);


}
void TEST_LTC6804(void)
{
	LTC680x_CS;
	SPI_Write(0X55);
	SPI_Write(0X55);
	LTC680X_DS;
}
void Alarm_LED_handle(SystemReg *sys)
{

}
void Emergency_LED_handle(SystemReg *sys)
{
 
}
void SlaveRegsInit(SlaveReg *s)
{
	
}
void ModuleRegsInit(ModuleReg *m)
{

}
void PackRegsInit(PackReg *b)
{
//	b->PackVoltage=0.0;
//	b->PackTemp=0.0;
//	b->PackSOC=0.0; 

}
void SystemRegsInit(SystemReg *s)
{
	LTC680X_DS;
	s->initCount=0;
	s->DigitalOutPutReg.all=0;
	s->DigitalInputReg.all=0;
	s->SystemStateARegs.all=0x0000;
	s->Timer50msec=0;
	s->Timer250msec=0;
	s->Timer500msec=0;
	s->Timer1000msec=0;
	s->Timer1500msec=0;
	s->Timer2000msec=0;
	s->Timer2500msec=0;
 
}
void SystemInit(SystemReg *s)
{
	
	if(s->initCount<500)
	{
		s->SystemStateARegs.bit.INITOK=0;
	}
	if(s->initCount>=500)
	{
		s->SystemStateARegs.bit.INITOK=1;
		s->initCount=501;
	}
}

void SystemAlarm(PackReg *bat,SystemReg *sys)
{

}
void SystemFault(PackReg *bat, SystemReg *sys)
{

}
void SysCurrentOffset(SystemReg *sys)
{
}
void SocBalance(ModuleReg *m) 
{
	
}
void AutoBalance(ModuleReg *m) 
{
	if(m->CellVoltageF[0]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell00=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell00=1;
	}
	if(m->CellVoltageF[1]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell01=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell01=1;
	}
	if(m->CellVoltageF[2]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell02=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell02=1;
	}
	if(m->CellVoltageF[3]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell03=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell03=1;
	}
	if(m->CellVoltageF[4]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell04=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell04=1;
	}
	if(m->CellVoltageF[5]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell05=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell05=1;
	}
	if(m->CellVoltageF[6]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell06=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell06=1;
	}
	if(m->CellVoltageF[7]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell07=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell07=1;
	}
	if(m->CellVoltageF[8]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell08=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell08=1;
	}
	if(m->CellVoltageF[9]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell09=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell09=1;
	}
	if(m->CellVoltageF[10]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell10=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell10=1;
	}
	if(m->CellVoltageF[11]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell11=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell11=1;
	}
}
void SystemforceBalance(ModuleReg *m)
{
	if(m->CellVoltageF[0]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell00=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell00=1;
	}
	
	if(m->CellVoltageF[1]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell01=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell01=1;
	}
	
	if(m->CellVoltageF[2]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell02=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell02=1;
	}
	
	if(m->CellVoltageF[3]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell03=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell03=1;
	}
	
	if(m->CellVoltageF[4]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell04=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell04=1;
	}
	
	if(m->CellVoltageF[5]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell05=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell05=1;
	}
	
	if(m->CellVoltageF[6]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell06=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell06=1;
	}
	
	if(m->CellVoltageF[7]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell07=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell07=1;
	}
	
	if(m->CellVoltageF[8]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell08=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell08=1;
	}
	
	if(m->CellVoltageF[9]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell09=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell09=1;
	}
	
	if(m->CellVoltageF[10]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell10=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell10=1;
	}
	
	if(m->CellVoltageF[11]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell11=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell11=1;
	}

}

float32 AbsVaule(float32 input)
{
	float32 AbsCurrent=0.0;
	if(input>=0)
	{
		AbsCurrent=input;
	}
	else if(input<0)
	{
		AbsCurrent=-1.0*input;
	}	
	return AbsCurrent;
}

void CellTempCalFloat(SlaveReg *s, ModuleReg *m)
{
	float32 tmpF0=0.0;
	float32 tmpF1=0.0;
	float32 tmpF2=0.0;
	float32 tmpF3=0.0;

	float32 x02=0.0;
	float32 x0=0.0;
	float32 x12=0.0;
	float32 x1=0.0;
	float32 x22=0.0;
	float32 x2=0.0;
	float32 x32=0.0;
	float32 x3=0.0;

	tmpF0=(float)(s->CellTempBuffer[0]*0.0001);
	tmpF1=(float)(s->CellTempBuffer[1]*0.0001);
	tmpF2=(float)(s->CellTempBuffer[2]*0.0001);
	tmpF3=(float)(s->CellTempBuffer[3]*0.0001);

	x02= tmpF0*tmpF0;
	x0=tmpF0;
	m->CellTempF[0]= 3.5081*x02-41.81*x0+107.59;

	x12= tmpF1*tmpF1;
	x1=tmpF1;
	m->CellTempF[1]= 3.5081*x12-41.81*x1+107.59;


	x22= tmpF2*tmpF2;
	x2=tmpF2;
	m->CellTempF[2]= 3.5081*x22-41.81*x2+107.59;

	x32= tmpF3*tmpF3;
	x3=tmpF3;
	m->CellTempF[3]= 3.5081*x32-41.81*x3+107.59;

}
void CellVoltagetoFloat(SlaveReg *s, ModuleReg *m)
{
	int i;
	float32 sum=0.0;
	
	for (i = 0; i < ModuleCellNum; i++)
	{	
	//	m->CellVoltageF[i]=(float32)s->CellVoltage[i]/10000;
		s->CellVoltagebuffer[i]=s->CellVoltage[i]/10;
		m->CellVoltageF[i]=(float32)(s->CellVoltagebuffer[i]*0.001);

	//	m->CellVoltageF[i]=m->CellVoltageF[i]+0.0005;
		sum=sum+m->CellVoltageF[i];
	}
	m->ModuleVoltageCellMAX=(float32)(s->CellVoltageMax*0.001);
	m->ModuleVoltageCellMIN=(float32)(s->CellVoltageMin*0.001);
	m->ModuleVoltageCellAVG=m->ModuleVoltageF/12.0;
	m->ModuleVoltageCellCap=m->ModuleVoltageCellMAX-m->ModuleVoltageCellMIN;
	m->ModuleVoltageF=sum;
	
}
// 입력 인자
// S->CellNumber : 
void ModuleCellMAX(SlaveReg *S)
{
	int i;
	S->CellVoltageMax=S->CellVoltage[0];
	S->CellVoltageMaxNum=0;
	for(i=0; i<S->CellNumber;i++)
	{
		if(S->CellVoltageMax< S->CellVoltage[i])
		{
			S->CellVoltageMax = S->CellVoltage[i];	
			S->CellVoltageMaxNum=i;
		}
	}
	
}
// 입력 인자
// S->CellNumber : 
void ModuleCellMIN(SlaveReg *S)
{
	int i;
	S->CellVoltageMin=S->CellVoltage[0];
	S->CellVoltageMinNum=0;
	for(i=0; i<S->CellNumber;i++)
	{
		if(S->CellVoltageMin> S->CellVoltage[i])
		{
			S->CellVoltageMin = S->CellVoltage[i];	
			S->CellVoltageMinNum=i;
		}
	}
}
void BatteryTeperaureCal(PackReg *bat,int Count)
{
	//bat->BatteryTempCellF[i]=(-4.67*num3+35.688*num2)-102.18*num1+91.477;
}
void SlaveBMSSPIEnable_low(void)
{
	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
}

void SlaveBMSSPIEnable_high(void)
{
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
}
void SlaveBms_WakeUp(void)
{
	// need to check
	SlaveBMSSPIEnable_low();
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	SlaveBMSSPIEnable_high();
}
int SlaveBMSCellReadCommand(SlaveReg *s)
{
	int i;
	SlaveBms_WakeUp();
	SlaveBMSSPIEnable_low();
//	s->WError= LTC6804_write(s->ID, s->Command, 0, 0);

	if((s->WError!=0)&&(s->len !=0))
	{
		for (i = 0; i < s->len; i++) 
		{
			s->ADCX[i]= SPI_READ();
		}
		s->pecr = (SPI_READ() << 8) & 0xff00;
		s->pecr |= (SPI_READ() & 0x00ff);
  //		s->pecg = pec15(s->ADCX[i], s->len);
		if (s->pecr ==s->pecg) 
		{
			s->RError=0;
			s->ErrorCount= 0;
		} 
		else 
		{
			s->RError=1;
			s->ErrorCount++;
		}
	}
	if(s->RError==1)
	{
		if(s->Command==LTC6804_CMD_RDCVA)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVA;
		}
		if(s->Command==LTC6804_CMD_RDCVB)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVB;
		}
		if(s->Command==LTC6804_CMD_RDCVC)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVC;
		}
		if(s->Command==LTC6804_CMD_RDCVD)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVD;
		}
	}
	SlaveBMSSPIEnable_high();
	return s->ErrorCode;	
}

int CellBalanceSet(int pack_id, CellBalance_t cell)
{
	int ret;
	char addr = LTC6804.address[pack_id];
	char *tab = (char *)LTC6804_init_table;
	tab[4] = (cell.all & 0xff);
	tab[5] = (tab[5] & 0xf0) | ((cell.all >> 8) & 0x0f);
	ret = LTC6804_write_cmd(addr, LTC6804_CMD_WRCFG, tab, sizeof(LTC6804_init_table));
	return ret;
}


//-----------------------------------------------------------------------

void SciaTxchar(char Txchar)
{
	while(!(SciaTxReadyFlag));
	SciaRegs.SCITXBUF=Txchar;
}
void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3)
{
	unsigned int Data0Low, Data0High, Data1Low, Data1High;
	unsigned int Data2Low, Data2High, Data3Low, Data3High;
	struct ECAN_REGS ECanaShadow;


	Data0Low  = 0x00ff&Data0;
	Data0High = 0x00ff&(Data0>>8);

	Data1Low  = 0x00ff&Data1;
	Data1High = 0x00ff&(Data1>>8);

	Data2Low  = 0x00ff&Data2;
	Data2High = 0x00ff&(Data2>>8);

	Data3Low  = 0x00ff&Data3;
	Data3High = 0x00ff&(Data3>>8);


	// 현재 CAN-A MBOX31가 전송 중이면 리턴함
	if(ECanaRegs.CANTRS.bit.TRS31== 1) return;
	//현재 CAN-A MBOX31가 전송 상태 체크함.
  	if(ECanaShadow.CANTA.bit.TA31== 0) // 0 : Send Fail,  1 : Send OK
	{
		ECanaShadow.CANTA.all = 0;
      	ECanaShadow.CANTA.bit.TA31= 1;     	         	// Clear TA5
       	ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;
   	}


	// CAN-A 송수신 체크
//	if(COM_FAULT_MODE)	CanDataCnt++;			// reset at Rx Routine
//	if(CanDataCnt > COMER_set)	COMER_FS=1;
//	Can_TX_ERR_CNT = ECanaRegs.CANTEC.bit.TEC;
//	Can_RX_ERR_CNT = ECanaRegs.CANREC.bit.REC;


	// 전송하는 데이터 ID 설정함.
	ECanaRegs.CANME.bit.ME31= 0;
	ECanaMboxes.MBOX31.MSGID.bit.STDMSGID=ID;
	ECanaRegs.CANME.bit.ME31= 1;
	// 전송하는 데이터 길이?8 설정함.


	ECanaMboxes.MBOX31.MSGCTRL.bit.DLC=Length; 	// 전송하는 데이터 길이
	// DATA 전송
	ECanaMboxes.MBOX31.MDL.byte.BYTE0=Data0Low;
	ECanaMboxes.MBOX31.MDL.byte.BYTE1=Data0High;

	ECanaMboxes.MBOX31.MDL.byte.BYTE2=Data1Low;
	ECanaMboxes.MBOX31.MDL.byte.BYTE3=Data1High;

	ECanaMboxes.MBOX31.MDH.byte.BYTE4=Data2Low;
	ECanaMboxes.MBOX31.MDH.byte.BYTE5=Data2High;

	ECanaMboxes.MBOX31.MDH.byte.BYTE6=Data3Low;
	ECanaMboxes.MBOX31.MDH.byte.BYTE7=Data3High;

	// CAN-A MBOX31 전송 시작함.
	ECanaRegs.CANTRS.bit.TRS31= 1;
	// CAN-A MBOX31전송 완료 체크함.
	while(!ECanaShadow.CANTA.bit.TA31);
	ECanaShadow.CANTA.all = 0;
   	ECanaShadow.CANTA.bit.TA31=1;     	         	// Clear TA5
   	ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;
	ECanaMboxes.MBOX31.MSGID.all	= 0;

	// From Here 해당 MailBox를 CAN Enable 시킴  이것을 왜 다시 해 주어야 하�?
	EALLOW;     // EALLOW enables access to protected bits

	ECanaShadow.CANME.all = ECanaRegs.CANME.all;

	ECanaShadow.CANME.bit.ME0= 1;
	ECanaShadow.CANME.bit.ME1= 1;
	ECanaShadow.CANME.bit.ME2= 1;
	ECanaShadow.CANME.bit.ME3= 1;
	ECanaShadow.CANME.bit.ME4= 1;
	ECanaShadow.CANME.bit.ME5= 1;
	ECanaShadow.CANME.bit.ME6= 1;
	ECanaShadow.CANME.bit.ME7= 1;
	ECanaShadow.CANME.bit.ME8= 1;
	ECanaShadow.CANME.bit.ME9= 1;
	ECanaShadow.CANME.bit.ME10= 1;
	ECanaShadow.CANME.bit.ME11= 1;
	ECanaShadow.CANME.bit.ME12= 1;
	ECanaShadow.CANME.bit.ME13= 1;
	ECanaShadow.CANME.bit.ME14= 1;
	ECanaShadow.CANME.bit.ME15= 1;
	ECanaShadow.CANME.bit.ME16= 1;
	ECanaShadow.CANME.bit.ME17= 1;
	ECanaShadow.CANME.bit.ME18= 1;
	ECanaShadow.CANME.bit.ME19= 1;
	ECanaShadow.CANME.bit.ME20= 1;
	ECanaShadow.CANME.bit.ME21= 1;
	ECanaShadow.CANME.bit.ME22= 1;
	ECanaShadow.CANME.bit.ME23= 1;
	ECanaShadow.CANME.bit.ME24= 1;
	ECanaShadow.CANME.bit.ME25= 1;
	ECanaShadow.CANME.bit.ME26= 1;
	ECanaShadow.CANME.bit.ME27= 1;
	ECanaShadow.CANME.bit.ME28= 1;
	ECanaShadow.CANME.bit.ME29= 1;
	ECanaShadow.CANME.bit.ME30= 1;
	ECanaShadow.CANME.bit.ME31= 1;
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;
	InitECan();

	EDIS;

	// To Here 해당 MailBox를 CAN Enable 시킴  이것을 왜 다시 해 주어야 하지?

}//EOF

//----------------------------------------------------------------------------------디지털 INPUT/OUTPUT 관련 함수


void DigitalOutput(SystemReg *sys)
{
	if(sys->DigitalOutPutReg.bit.Relay==1)
  	{
  		Relay_ON; 
  	}
	else 
	{
		Relay_OFF;
	}
}

void DataConversion(SystemReg *sys)
{

}
*/



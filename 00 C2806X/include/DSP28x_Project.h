
//###########################################################################
//
// FILE:   DSP28x_Project.h
//
// TITLE:  DSP28x Project Headerfile and Examples Include File
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// $Copyright:
// Copyright (C) 2009-2024 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

#ifndef DSP28x_PROJECT_H
#define DSP28x_PROJECT_H

//
// Included Files
//
#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "F2806x_Examples.h"   	// F2806x Examples Include File
#include "parameter.h"

//EDLAY 매크로 선언 --------------------------------------------------------------------//

// TI SDK 1.10의 소스 DSP2803x_usDelay.asm에서 제공하는 DELAY_US 함수를 사용

// TI SDK 1.10의 소스 DSP2803x_usDelay.asm에서 제공하는 DELAY_US 함수를 사용
#define delay_us(us)        DELAY_US(us)
// TI SDK 1.10의 소스 DSP2803x_usDelay.asm에서 제공하는 DELAY_US 함수를 사용
#define delay_ms(ms)        DELAY_US(ms*1000)
//#define  SPI_Read()          SPI_READ()

/*-------------------------------------------------------------------------------
Next, definitions used in main file.
-------------------------------------------------------------------------------*/
#define TRUE    1
#define FALSE   0
#define TRUE    1
#define ON      1
#define OFF     0

#define BIT0_POS        0
#define BIT1_POS        1
#define BIT2_POS        2
#define BIT3_POS        3
#define BIT4_POS        4
#define BIT5_POS        5
#define BIT6_POS        6
#define BIT7_POS        7
#define BIT8_POS        8
#define BIT9_POS        9
#define BIT10_POS       10
#define BIT11_POS       11
#define BIT12_POS       12
#define BIT13_POS       13
#define BIT14_POS       14
#define BIT15_POS       15

/* Bit Mask Data 정의 */
#define BIT0_MASK       0x0001
#define BIT1_MASK       0x0002
#define BIT2_MASK       0x0004
#define BIT3_MASK       0x0008
#define BIT4_MASK       0x0010
#define BIT5_MASK       0x0020
#define BIT6_MASK       0x0040
#define BIT7_MASK       0x0080
#define BIT8_MASK       0x0100
#define BIT9_MASK       0x0200
#define BIT10_MASK      0x0400
#define BIT11_MASK      0x0800
#define BIT12_MASK      0x1000
#define BIT13_MASK      0x2000
#define BIT14_MASK      0x4000
#define BIT15_MASK      0x8000


#define     Shift_RIGHT(val, bit)   ((val) >> (al))
#define     Shift_LEFT(val,  bit)   ((val) << (val))
#define     ComBine(Val_H, Val_L)   (((Val_H) << 8) |(Val_L))
#define     BIT_MASK(bit)           (1 << (bit))
#define     GetBit(val, bit)        (((val) & BIT_MASK(bit)) >> (bit))
#define     SetBit(val, bit)        (val |= BIT_MASK(bit))
#define     ClearBit(val, bit)      (val &= ~BIT_MASK(bit))
#define     ToggleBit(val, bit)     (val ^= BIT_MASK(bit))
#define     bit_is_set(val, bit)    (val & BIT_MASK(bit))
#define     bit_is_clear(val, bit)  (~val & BIT_MASK(bit))

struct Data_WORD
{
    unsigned int DataL;
    unsigned int DataH;
};
struct ParentDeviceCMD_BIT
{
    // bits   description
   unsigned int     POWEREN                 :1;   // 0
   unsigned int     SW01                    :1;   // 2
   unsigned int     SW02                    :1;   // 3
   unsigned int     SW03                    :1;   // 4
   unsigned int     SW04                    :1;   // 5
   unsigned int     SW05                    :1;   // 6
   unsigned int     SW06                    :1;   // 6
   unsigned int     SW07                    :1;   // 7
   unsigned int     SW08                    :1;   // 8
   unsigned int     SW09                    :1;   // 9
   unsigned int     SW10                    :1;   // 11
   unsigned int     SW11                    :1;   // 12
   unsigned int     SW12                    :1;   // 13
   unsigned int     SW13                    :1;   // 14
   unsigned int     SW14                    :1;   // 15
   unsigned int     SW15                    :1;   // 16
};
union ParentDeviceCMD_REG
{
   unsigned int     all;
   struct ParentDeviceCMD_BIT bit;
};

struct DigitalInPut_BIT
{       // bits   description
   unsigned int     IDSW                    :2;   // 0
   unsigned int     PAUX                    :1;   // 2
   unsigned int     NAUX                    :1;   // 3
   unsigned int     EMGSWStauts             :1;   // 4
   unsigned int     CANRX0                  :1;   // 5
   unsigned int     CANRX1                  :1;   // 6
   unsigned int     SW07                    :1;   // 7
   unsigned int     SW08                    :1;   // 8
   unsigned int     SW09                    :1;   // 9
   unsigned int     SW10                    :1;   // 11
   unsigned int     SW11                    :1;   // 12
   unsigned int     SW12                    :1;   // 13
   unsigned int     SW13                    :1;   // 14
   unsigned int     SW14                    :1;   // 15
   unsigned int     SW15                    :1;   // 16
};
union DigitalInput_REG
{
   unsigned int     all;
   struct DigitalInPut_BIT bit;
};

struct DigitalOutPut_BIT
{       // bits   description
    unsigned int     PRlyOUT                :1; // 0
    unsigned int     NRlyOUT                :1; // 1
    unsigned int     ProRlyOUT              :1; // 2
    unsigned int     StartBATOUT            :1; // 3
    unsigned int     IMDTOPOUT              :1; // 4
    unsigned int     IMDBOTOUT              :1; // 5
    unsigned int     LEDSysOUT              :1; // 6
    unsigned int     LEDCAnOUT              :1; // 7
    unsigned int     LEDAlarmOUT            :1; // 8
    unsigned int     LEDFaultOUT            :1; // 9
    unsigned int     LEDProtectOUT          :1; // 10
    unsigned int     PWRLAMPOUT             :1; // 11
    unsigned int     EMGSWLAMPOUT           :1; // 12
    unsigned int     DO013                  :1; // 14
    unsigned int     DO014                  :1; // 14
    unsigned int     DO015                  :1; // 15
};
union DigitalOutPut_REG
{
   unsigned int     all;
   struct DigitalOutPut_BIT bit;
};
typedef enum
{
   System_STATE_INIT,
   System_STATE_STANDBY,
   System_STATE_READY,
   System_STATE_RUNING,
   System_STATE_PROTECTER,
   System_STATE_DATALOG,
   System_STATE_ProtectHistory,
   System_STATE_MANUALMode,
   System_STATE_CLEAR
} SysState;
struct SystemStauts_BIT
{       // bits   description
    unsigned int     SW00         :1; // 0
    unsigned int     SW01         :1; // 1
    unsigned int     SW02         :1; // 2
    unsigned int     SW03         :1; // 3
    unsigned int     SW04         :1; // 4
    unsigned int     SW05         :1; // 5
    unsigned int     SW06         :1; // 6
    unsigned int     SW07         :1; // 7
    unsigned int     SW08         :1; // 8
    unsigned int     SW09         :1; // 9
    unsigned int     SW10         :1; // 9
    unsigned int     SW11         :1; // 9
    unsigned int     SW12         :1; // 9
    unsigned int     SW13         :1; // 9
    unsigned int     SW14         :1; // 9
    unsigned int     SW15         :1; // 9
};
union SystemStauts_REG
{
   unsigned int           all;
   struct SystemStauts_BIT bit;
};
struct MDErrStauts_BIT
{       // bits   description
    unsigned int     MD01         :1; // 0
    unsigned int     MD02         :1; // 1
    unsigned int     MD03         :1; // 2
    unsigned int     MD04         :1; // 3
    unsigned int     MD05         :1; // 4
    unsigned int     MD06         :1; // 5
    unsigned int     MD07         :1; // 6
    unsigned int     MD08         :1; // 7
    unsigned int     MD09         :1; // 8
    unsigned int     SW10         :1; // 9
    unsigned int     SW11         :1; // 10
    unsigned int     SW12         :1; // 11
    unsigned int     SW13         :1; // 12
    unsigned int     SW14         :1; // 13
    unsigned int     SW15         :1; // 14
    unsigned int     SW16         :1; // 15
};
union MDErrStauts_REG
{
   unsigned int           all;
   struct MDErrStauts_BIT bit;
};
struct SysMDstatus_BIT
{       // bits   description
    unsigned int     INITOK                     :1;    // 0
    unsigned int     status01                   :1;    // 1
    unsigned int     Fault                      :1;    // 2
    unsigned int     BalanceStartStop           :1;    // 3
    unsigned int     BalanceEnable              :1;    // 4
    unsigned int     WaterleakFault             :1;    // 5, FAN Delection
    unsigned int     CellVoltageFault           :1;    // 6
    unsigned int     CellTemperatureFault       :1;    // 7
    unsigned int     BATIC1ErrFault             :1;    // 8
    unsigned int     CTCOMErrFault              :1;    // 9
    unsigned int     MBCOMErrFault              :1;    // 10
    unsigned int     HMICOMErrFault             :1;    // 11
    unsigned int     CellVoltCAN                :1;    // 12
    unsigned int     CellTempCAN                :1;    // 12
    unsigned int     PackEn                     :1;    // 14
    unsigned int     HmiEn                      :1;    // 15
};
union SysMDstatus_REG
{
   unsigned int     all;
   struct SysMDstatus_BIT bit;
};
struct SystemState_BIT
{       // bits   description
    unsigned int     SysSTATE            :3; // 0,1,2
    unsigned int     SysProtectSate      :1; // 3,4,5
    unsigned int     ISOSPICOMERR        :1; // 6
    unsigned int     INCANCOMERR         :1; // 7
    unsigned int     EXCANCOMERR         :1; // 8
    unsigned int     TCPIPTOMERR         :1; // 9
    unsigned int     RS485COMERR         :1; // 10
    unsigned int     WaterLeakERR        :1; // 11
    unsigned int     EMGSWERR            :1; // 12
    unsigned int     ISORegERR           :1; // 13
    unsigned int     MSDERR              :1; // 14
    unsigned int     INITOK              :1; // 15
    unsigned int     CANCOMEnable        :1; // 16
    unsigned int     BalanceMode         :1; // 17
    unsigned int     RTCRD               :1; // 18
    unsigned int     RTCWR               :1; // 19
    unsigned int     NVRAMRD             :1; // 20
    unsigned int     NVRAMWR             :1; // 21
    unsigned int     CANRXCountReset     :1; // 22
    unsigned int     SysAalarm           :1; // 23
    unsigned int     SysFault            :1; // 24
    unsigned int     SysProtect          :1; // 25
    unsigned int     CTCOMErr            :1; // 26
    unsigned int     MDBATICErr          :1; // 27
    unsigned int     MDSUBCANErr         :1; // 28
    unsigned int     MDCTCANErr          :1; // 28
    unsigned int     SW30                :1; // 30
    unsigned int     SW31                :1; // 31
};
union SystemState_REG
{
   struct SystemState_BIT   bit;
   struct Data_WORD        Word;
   Uint32                   all;
};
struct SystemAlarm_BIT
{       // bits   description
    unsigned int     PackVDISCHACT_OV          :1; // 0
    unsigned int     PackVCHACT_OV             :1; // 1
    unsigned int     PackVSOC_OV               :1; // 2
    unsigned int     PackVSOC_UN               :1; // 3
    unsigned int     PackVolt_OV               :1; // 4
    unsigned int     PackVolt_UN               :1; // 5
    unsigned int     CellVolt_OV               :1; // 6
    unsigned int     CellVolt_UN               :1; // 7
    unsigned int     CellVolt_BL               :1; // 8
    unsigned int     DisChaCellTemp_OT         :1; // 9
    unsigned int     CharCellTemp_OT           :1; // 10
    unsigned int     DISCHA_CellTemp_UT        :1; // 11
    unsigned int     CHA_CellTemp_UT           :1; // 12
    unsigned int     CellTemp_BLT              :1; // 13
    unsigned int     PackUnbaDisCh_UbPWR       :1; // 14
    unsigned int     PackUnbaCahr_UbPWR        :1; // 15
    unsigned int     SW16                :1; // 16
    unsigned int     SW17                :1; // 17
    unsigned int     SW18                :1; // 18
    unsigned int     SW19                :1; // 19
    unsigned int     SW20                :1; // 20
    unsigned int     SW21                :1; // 21
    unsigned int     SW22                :1; // 22
    unsigned int     SW23                :1; // 23
    unsigned int     SW24                :1; // 24
    unsigned int     SW25                :1; // 25
    unsigned int     SW26                :1; // 26
    unsigned int     SW27                :1; // 27
    unsigned int     SW28                :1; // 28
    unsigned int     SW29                :1; // 29
    unsigned int     SW30                :1; // 30
    unsigned int     SW31                :1; // 31
};
union SystemAlarm_REG
{
   struct SystemAlarm_BIT   bit;
   struct Data_WORD        Word;
   Uint32                   all;
};
struct SystemFault_BIT
{       // bits   description
    unsigned int     PackVDISCHACT_OV          :1; // 0
    unsigned int     PackVCHACT_OV             :1; // 1
    unsigned int     PackVSOC_OV               :1; // 2
    unsigned int     PackVSOC_UN               :1; // 3
    unsigned int     PackVolt_OV               :1; // 4
    unsigned int     PackVolt_UN               :1; // 5
    unsigned int     CellVolt_OV               :1; // 6
    unsigned int     CellVolt_UN               :1; // 7
    unsigned int     CellVolt_BL               :1; // 8
    unsigned int     DisChaCellTemp_OT         :1; // 9
    unsigned int     CharCellTemp_OT           :1; // 10
    unsigned int     DISCHA_CellTemp_UT        :1; // 11
    unsigned int     CHA_CellTemp_UT           :1; // 12
    unsigned int     CellTemp_BLT               :1; // 13
    unsigned int     PackUnbaDisCh_UbPWR       :1; // 14
    unsigned int     PackUnbaCahr_UbPWR        :1; // 15
    unsigned int     SW16                :1; // 16
    unsigned int     SW17                :1; // 17
    unsigned int     SW18                :1; // 18
    unsigned int     SW19                :1; // 19
    unsigned int     SW20                :1; // 20
    unsigned int     SW21                :1; // 21
    unsigned int     SW22                :1; // 22
    unsigned int     SW23                :1; // 23
    unsigned int     SW24                :1; // 24
    unsigned int     SW25                :1; // 25
    unsigned int     SW26                :1; // 26
    unsigned int     SW27                :1; // 27
    unsigned int     SW28                :1; // 28
    unsigned int     SW29                :1; // 29
    unsigned int     SW30                :1; // 30
    unsigned int     SW31                :1; // 31
};
union SystemFault_REG
{
   struct SystemFault_BIT  bit;
   struct Data_WORD       Word;
   Uint32                 all;
};
struct SystemProtect_BIT
{       // bits   description
    unsigned int     PackVDISCHACT_OV          :1; // 0
    unsigned int     PackVCHACT_OV             :1; // 1
    unsigned int     PackVSOC_OV               :1; // 2
    unsigned int     PackVSOC_UN               :1; // 3
    unsigned int     PackVolt_OV               :1; // 4
    unsigned int     PackVolt_UN               :1; // 5
    unsigned int     CellVolt_OV               :1; // 6
    unsigned int     CellVolt_UN               :1; // 7
    unsigned int     CellVolt_BL               :1; // 8
    unsigned int     DisChaCellTemp_OT         :1; // 9
    unsigned int     CharCellTemp_OT           :1; // 10
    unsigned int     DISCHA_CellTemp_UT        :1; // 11
    unsigned int     CHA_CellTemp_UT           :1; // 12
    unsigned int     CellTemp_BLT               :1; // 13
    unsigned int     PackUnbaDisCh_UbPWR       :1; // 14
    unsigned int     PackUnbaCahr_UbPWR        :1; // 15
    unsigned int     PackRlyErr                :1; // 16
    unsigned int     PackINComErr              :1; // 17
    unsigned int     PackExComErr              :1; // 18
    unsigned int     PackCTComErr              :1; // 19
    unsigned int     PackWaterleakErr          :1; // 20
    unsigned int     PackEMSSWErr              :1; // 21
    unsigned int     SW22                :1; // 22
    unsigned int     SW23                :1; // 23
    unsigned int     SW24                :1; // 24
    unsigned int     SW25                :1; // 25
    unsigned int     SW26                :1; // 26
    unsigned int     SW27                :1; // 27
    unsigned int     SW28                :1; // 28
    unsigned int     SW29                :1; // 29
    unsigned int     SW30                :1; // 30
    unsigned int     SW31                :1; // 31
};
union SystemProtect_REG
{
   Uint32                   all;
   struct Data_WORD         Word;
   struct SystemProtect_BIT bit;
};
struct Current_byte
{
    unsigned int CurrentL;
    unsigned int CurrentH;
};
union Currnet_Reg
{
    long                all;
    struct Current_byte byte;
};
/*struct WORD2BYTE_byte
{
    unsigned int BYTEL;
    unsigned int BYTEH;
};
union WORD2BYTE_Reg
{
    unsigned int           all;
    struct WORD2BYTE_byte byte;
};
*/
typedef struct System_Date
{
    /*
     *
     */
    Uint16  Test;
    Uint16  Maincount;
    Uint16  MainIsr1;
    Uint16  CANRXCOUNT;
    Uint16  CANRXMailBox00Count;
    Uint16  CANRXMailBox01Count;
    Uint16  CANRXMailBox02Count;
    Uint16  CANRXMailBox03Count;
    Uint16  CANRXMailBox04Count;
    Uint16  SysRegTimer5msecCount;
    Uint16  SysRegTimer10msecCount;
    Uint16  SysRegTimer50msecCount;
    Uint16  SysRegTimer100msecCount;
    Uint16  SysRegTimer300msecCount;
    Uint16  SysRegTimer500msecCount;
    Uint16  SysRegTimer1000msecCount;
    Uint16  CellVoltsampling;
    Uint16  SysCanRxCount;
    Uint16  AlarmStatecount;
    Uint16  PackFaultStatecount;
    Uint16  Bat12VFaultStatecount;
    Uint16  ProtectStatecount;
    Uint16  BalanceModeCount;
    Uint16  BalanceTimeCount;
    Uint16  RelayCheck;
    float32 PackVoltageF;
    float32 PackCurrentF;
    float32 PackCurrentAsbF;
    float32 PackCellMaxVoltageF;
    float32 PackCellMinVoltageF;
    float32 PackCellDivVoltageF;
    float32 PackCellAgvVoltageF;
    Uint16  PackCellMaxVoltPos;
    Uint16  PackCellMinVoltPos;
    float32 PackCellMaxTemperatureF;
    float32 PackCellMinTemperatureF;
    float32 PackCellDivTemperatureF;
    float32 PackCellAgvTemperatureF;
    Uint16  PackCellMaxTmepsPos;
    Uint16  PackCellMinTmepsPos;
    float32 SystemBMS

    float32 PackCHAPWRContintyF;
    float32 PackDisCHAPWRContintyF;
    float32 PackCHAPWRPeakF;
    float32 PackDisCHAPWRPeakF;
    float32 PackSOCF;
    float32 PackSOHF;
    float32 PackAhF;
    float32 PackISOResisF;
//    float32 Bat12VCellVoltage[Sys12VCellVoltCount];
    //float32 PackCellTe[Sys80VCellVoltCount];
    //float32 Bat12VCellVoltage[Sys12VCellVoltCount];
    /*
     *
    */
    unsigned int LEDSycCount;
    unsigned int LEDFaultCount;
    unsigned int LEDCanCount;
    unsigned int PWRLAMPCount;
    unsigned int StartBATOUTOnCount;
    unsigned int StartBATOUTOffCount;

    Uint16 MD1CANRxCount;
    Uint16 MD2CANRxCount;
    Uint16 MD3CANRxCount;
    Uint16 MD4CANRxCount;
    Uint16 MD5CANRxCount;
    Uint16 MD6CANRxCount;
    Uint16 MD7CANRxCount;
    Uint16 CTRxCount;
    Uint16 MasterRxCount;

    SysState    SysMachine;
    union       ParentDeviceCMD_REG         PMSysCMDResg;
    union       SysMDstatus_REG             PackModule1Regs;
    union       SysMDstatus_REG             PackModule2Regs;
    union       SysMDstatus_REG             PackModule3Regs;
    union       SysMDstatus_REG             PackModule4Regs;
    union       SysMDstatus_REG             PackModule5Regs;
    union       SysMDstatus_REG             PackModule6Regs;
    union       SysMDstatus_REG             PackModule7Regs;
    union       SystemState_REG             PackStateReg;
    union       SystemAlarm_REG             PackAlarmReg;
    union       SystemFault_REG             PackFaultReg;
    union       SystemProtect_REG           PackProtectReg;
    union       SystemFault_REG             PackFaulBuftReg;
  //  union       SystemProtect_REG           ProtectReg;

    union       Currnet_Reg          CurrentData;
    union       DigitalInput_REG     DigitalInputReg;
    union       DigitalOutPut_REG    DigitalOutPutReg;

    union       MDErrStauts_REG      PackInMDCANrxReg;   // Pack BMS에서 PACK-Module1~7 간에 CAN 상태 체크

    union       MDErrStauts_REG      ModuleStatusInitok;
    union       MDErrStauts_REG      ModuleStatusFault;
    union       MDErrStauts_REG      ModuleStatusBalanceStart;
    union       MDErrStauts_REG      ModuleStatusBalanceEnable;
    union       MDErrStauts_REG      ModuleStatusWleagErr;
    union       MDErrStauts_REG      ModuleStatusBATIC1Err;
    union       MDErrStauts_REG      ModuleStatusCTCOMErr;
    union       MDErrStauts_REG      ModuleStatusSBComErr;
    union       MDErrStauts_REG      ModuleStatusHMIComErr;

    union       MDErrStauts_REG      ModuleStatusPackEn;
    union       MDErrStauts_REG      ModuleWaterLeakReg; // UINT BMS 1 ~7의 Waterleak 상태 체크
    union       MDErrStauts_REG      ModuleBatICReg;     // UINT BMS 1 ~7의  BATIC 통신 상태 체크
    union       MDErrStauts_REG      ModuleCTCANErrCReg; // UINT BMS 1 ~7의  CT 간 CAN 상태 체크
    union       MDErrStauts_REG      ModuleSubCANErrCReg;// UINT BMS 1 ~7의  SUB-BMS 간 CAN 상태 체크
    union       MDErrStauts_REG      ModuleStateInitOk;// UINT BMS 1 ~7의  SUB-BMS 간 CAN 상태 체크

  //  union       WORD2BYTE_Reg         PackCOMERR;

}SystemReg;
typedef struct Module_Date
{
    Uint16 MD41XRxcount[7];
    Uint16 MD42XRxcount[7];
    Uint16 MD43XRxcount[7];
    Uint16 MD44XRxcount[7];
    Uint16 MD45xRxcount[7];
    Uint16 MD46XRxcount[7];
    Uint16 MD47XRxcount[7];



    Uint16 MDCellVoltQty[7];
    Uint16 MDCellTempsQty[7];
    Uint16 MDFirmwareVer[7];
    Uint16 MDFirmwareRev[7];
    Uint16 MDNorVolt[7];
    Uint16 MDNorCapacity[7];
    Uint16 PackMinVolteRec[7];


    Uint16 MDCellMaxVolt[7];
    Uint16 MDCellMinVolt[7];
    Uint16 MDCellAgvVolt[7];
    Uint16 MDCellDivVolt[7];


    int16 MDCellMaxTemps[7];
    int16 MDCellMinTemps[7];
    int16 MDCellAgvTemps[7];
    Uint16 MDCellDivTemps[7];


    Uint16 MDCTComErr[7];
    Uint16 MDSubComErr[7];
    Uint16 MDBatICCOMErr[7];
    Uint16 MDWaterLeakErr[7];



    Uint16 MDTotalVolt[7];
    Uint16 MDMaxVoltPo[7];
    Uint16 MDMinVoltPo[7];
    Uint16 MDMaxTempsPo[7];
    Uint16 MDMinTempsPo[7];
    Uint16 MDstatusbit[7];


   // union MDCTState_REG MDCTComErrStaute;
   // union MDCTState_REG MDSubComErrStaute;
   // union MDCTState_REG MDBatICCOMErrStaute;
   // union MDCTState_REG MDWaterLeakErrStaute;

}ModulemReg;
typedef enum
{
  TIMER_STATE_IDLE,
  TIMER_STATE_RUNNING,
  TIMER_STATE_EXPIRED,
  TIMER_STATE_CLEAR
}TimerState;
typedef struct
{
  TimerState state;
  int TimeCount;
  int Start,Stop,Reset,OutState;
  unsigned int TimerVaule;
}TimerReg;
struct BATStatus_BIT
{       // bits   description
    unsigned int     PackBalance          :1; // 0
    unsigned int     PackNeg_Rly          :1; // 1
    unsigned int     PackPos_Rly          :1; // 2
    unsigned int     PackPreChar_Rly      :1; // 3
    unsigned int     PackMSD_AUX          :1; // 4
    unsigned int     PackEMG_SW           :1; // 5
    unsigned int     PackWaterleak        :1; // 6
    unsigned int     Stauts07             :1; // 7
    unsigned int     Stauts08             :1; // 8
    unsigned int     Stauts09             :1; // 9
    unsigned int     Stauts10             :1; // 10
    unsigned int     Stauts11             :1; // 11
    unsigned int     Stauts12             :1; // 12
    unsigned int     Stauts13             :1; // 13
    unsigned int     Stauts14             :1; // 14
    unsigned int     Stauts15             :1; // 15
};
union BATStatus_REG
{
   unsigned int     all;
   struct BATStatus_BIT bit;
};

struct VCUCOMMAND_BIT
{       // bits   description
   unsigned int     RUNStatus01          :1; // 0
   unsigned int     BRAModule01          :1; // 1
   unsigned int     RUNStatus03          :1; // 2
   unsigned int     RUNStatus04          :1; // 3
   unsigned int     RUNStatus05          :1; // 4
   unsigned int     RUNStatus06          :1; // 5
   unsigned int     RUNStatus07          :1; // 6
   unsigned int     RUNStatus08          :1; // 7
   unsigned int     PrtctReset01         :1; // 8
   unsigned int     PrtctReset02         :1; // 9
   unsigned int     PrtctReset03         :1; // 10
   unsigned int     PrtctReset04         :1; // 11
   unsigned int     PrtctReset05         :1; // 12
   unsigned int     PrtctReset06         :1; // 13
   unsigned int     PrtctReset07         :1; // 14
   unsigned int     PrtctReset08         :1; // 15
};
union VCUCOMMAND_REG
{
   unsigned int     all;
   struct VCUCOMMAND_BIT bit;
};


struct MoudleState_BIT
{       // bits   description
   unsigned int     RUNStatus01          :1; // 0
   unsigned int     RUNStatus02          :1; // 1
   unsigned int     RUNStatus03          :1; // 2
   unsigned int     RUNStatus04          :1; // 3
   unsigned int     RUNStatus05          :1; // 4
   unsigned int     RUNStatus06          :1; // 5
   unsigned int     RUNStatus07          :1; // 6
   unsigned int     RUNStatus08          :1; // 7
   unsigned int     PrtctReset01         :1; // 8
   unsigned int     PrtctReset02         :1; // 9
   unsigned int     PrtctReset03         :1; // 10
   unsigned int     PrtctReset04         :1; // 11
   unsigned int     PrtctReset05         :1; // 12
   unsigned int     PrtctReset06         :1; // 13
   unsigned int     PrtctReset07         :1; // 14
   unsigned int     PrtctReset08         :1; // 15
};
union MoudleState_REG
{
   unsigned int     all;
   struct MoudleState_BIT bit;
};




typedef struct CANA_DATA
{
    Uint16 PackID;
    Uint16 SWTypeVer;
    /*
     *
     */
    Uint16 CellNumStart;
    Uint16 NumberShift;
    Uint16 CellVotlageNumber;
    Uint16 CellVotlageMaxNumber;
    Uint16 CellVoltageNum;
    Uint16 CellVoltagteInf;
    Uint16 CellVoltagteTotalNum;
    Uint16 CellVoltagteTotalNumShift;
    Uint16 CellVoltagteStartNum;
    Uint16 CellNumTStart;
    Uint16 NumberTShift;
    Uint16 CellTemperatureNumber;
    Uint16 CellTemperatureMaxNumber;
    Uint16 CellTemperatureNum;
    Uint16 PackConfing;
    /*
     *
     */

    union BATStatus_REG               PackStatus;
    union DigitalOutPut_REG           PackDigitalOutPutReg;
    union VCUCOMMAND_REG              PMSCMDRegs;
//    union VCUCOMMAND_REG              PMSCMDRegs;
    Uint16 PackSate;
    Uint16 PackProtetSate;
    Uint16 PackSateInfo;
    Uint16 PackConFig;
    int16  PackSOC;
    Uint16 PackSOH;
    Uint16 PackAh;
    int16  PackCT;
    Uint16 PackPT;
    Uint16 PackCHAPWRContinty;
    Uint16 PackCHAPWRPeak;
    Uint16 PackDisCHAPWRContinty;
    Uint16 PackDisCHAPWRPeak;
    Uint16 PackVoltageMax;
    Uint16 PackVoltageMin;
    Uint16 PackBalanVolt;
    Uint16 PackVoltageAgv;
    Uint16 PackVoltageDiv;
    Uint16 PackVoltageMaxNum;
    Uint16 PackVoltageMinNum;
    Uint16 PackPackVotageBuf;
    int16  PackTemperaturelMAX;
    int16  PackTemperaturelMIN;
    int16  PackTemperatureAVG;
    Uint16 PackTemperatureDiv;
    Uint16 PackTemperatureMaxNUM;
    Uint16 PackTemperatureMinNUM;
    Uint16 SwVerProducttype;
    Uint16 BatConfParallelSerial;


    Uint16 MD1_TotalVolt;
    Uint16 MD2_TotalVolt;
    Uint16 MD3_TotalVolt;
    Uint16 MD4_TotalVolt;
    Uint16 MD5_TotalVolt;
    Uint16 MD6_TotalVolt;
    Uint16 MD7_TotalVolt;

    Uint16 MD1_CellAgvVolt;
    Uint16 MD2_CellAgvVolt;
    Uint16 MD3_CellAgvVolt;
    Uint16 MD4_CellAgvVolt;
    Uint16 MD5_CellAgvVolt;
    Uint16 MD6_CellAgvVolt;
    Uint16 MD7_CellAgvVolt;

    int16 MD1_CellAgvTemps;
    int16 MD2_CellAgvTemps;
    int16 MD3_CellAgvTemps;
    int16 MD4_CellAgvTemps;
    int16 MD5_CellAgvTemps;
    int16 MD6_CellAgvTemps;
    int16 MD7_CellAgvTemps;


    union   MoudleState_REG MD1Staue;
    union   MoudleState_REG MD2Staue;
    union   MoudleState_REG MD3Staue;
    union   MoudleState_REG MD4Staue;
    union   MoudleState_REG MD5Staue;
    union   MoudleState_REG MD6Staue;
    union   MoudleState_REG MD7Staue;
//    union  WORD2BYTE_Reg    SwVerProducttype;
//    union  WORD2BYTE_Reg    BatConfParallelSerial;
    /*
     *
     */

    /*
     *
     */
    Uint16 MailBoxRxCount;
    Uint16 MailBox0RxCount;
    Uint16 MailBox1RxCount;
    Uint16 MailBox2RxCount;
    Uint16 MailBox3RxCount;
//  Uint16 CANID;



}CANAReg;
#endif  // end of DSP28x_PROJECT_H definition


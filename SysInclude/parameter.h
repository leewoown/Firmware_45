/* ==============================================================================
System Name:  현대자동차 수소 지게차 80V

File Name:		PARAMETER.H

Description:	현대
          	    Orientation Control for a Three Phase AC Induction Motor. 

Originator:		Digital control systems Group - Texas Instruments

Note: In this software, the default inverter is supposed to be DMC1500 board.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
=================================================================================  */
//#include "build.h"
//#include "math.h"
//#include "IQmathLib.h"
#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "F2806x_Examples.h"    // F2806x Examples Include File
#include "DSP28x_Project.h"
#ifndef PARAMETER_H
#define PARAMETER_H

/* Bit 위치 정의(주로 위에서 정의한 매크로 함수에서 사용하기 위해 정의함) */


#define	SCIA_BUFRX		50			// Monstar와 맞추어야 함

//#define UL_BYTE(x)		    (x >> 16)
//#define HI_BYTE(x)		    (x >> 8)
//#define LO_BYTE(x)          (x & 0xff)
//#define MAKE_WORD(msb,lsb)	((msb<<8) | (lsb))
//#define WordLShift(md,ml)   (md<<ml)
//#define WordRShift(md,ml)   (md>>ml)



// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 	    10
#define SYSTEM_FREQUENCY    60
#define CPUCLK			       60000000L					// CPU Main Clock

//#define CPU_CLOCK_SPEED     6.6667L   			    // for a 150MHz CPU clock speed
#define CPU_CLOCK_SPEED     11.111L                     // for a 90MHz CPU clock speed
#define ADC_usDELAY 	    5000L



#define	Uint16Max		    65536


/*
#define TxA_RDY_flag        SciaRegs.SCICTL2.bit.TXRDY
#define TxB_RDY_flag	    ScibRegs.SCICTL2.bit.TXRDY
#define TxC_RDY_flag        ScicRegs.SCICTL2.bit.TXRDY
#define TxA_Empty_flag      SciaRegs.SCICTL2.bit.TXEMPTY
#define TxB_Empty_flag      ScibRegs.SCICTL2.bit.TXEMPTY
#define TxC_Empty_flag	    ScicRegs.SCICTL2.bit.TXEMPTY
#define AD_START   	   	    AdcRegs.ADCTRL2.bit.SOC_SEQ1
#define IS_AD_BUSY 	        AdcRegs.ADCST.bit.SEQ1_BSY
*/

/*
 * LED00 indicates SYSTEM STATE status
 */
#define LEDSysState_H              GpioDataRegs.GPBSET.bit.GPIO58=1 //System Fault State  LED
#define LEDSysState_L              GpioDataRegs.GPBCLEAR.bit.GPIO58=1
#define LEDSysState_T              GpioDataRegs.GPBTOGGLE.bit.GPIO58=1

/*
 *  LED01 indicates SYSTEM Fault status
 */
#define LEDFault_H              GpioDataRegs.GPBSET.bit.GPIO44=1
#define LEDFault_L              GpioDataRegs.GPBCLEAR.bit.GPIO44=1
#define LEDFault_T              GpioDataRegs.GPBTOGGLE.bit.GPIO44=1

/*
 * LED02 indicates SYSTEM CANSTAT status
 */

#define LEDCANState_H            GpioDataRegs.GPBSET.bit.GPIO51=1
#define LEDCANState_L            GpioDataRegs.GPBCLEAR.bit.GPIO51=1
#define LEDCANState_T            GpioDataRegs.GPBTOGGLE.bit.GPIO51=1


/*
 * DIP SW
 */
#define IDSW00           GpioDataRegs.GPADAT.bit.GPIO16
#define IDSW01           GpioDataRegs.GPADAT.bit.GPIO17
#define IDSW02           GpioDataRegs.GPADAT.bit.GPIO18
#define IDSW03           GpioDataRegs.GPADAT.bit.GPIO19

/*
 * SPI chip RTC CS, MFP DIO
 */

#define RTC_CS            GpioDataRegs.GPACLEAR.bit.GPIO9=1
#define RTC_DS            GpioDataRegs.GPASET.bit.GPIO9=1
#define RTC_MF            GpioDataRegs.GPCDAT.bit.GPIO8
/*
 * SPI chip NVRAM CS
 */
//#define NvramCS            GpioDataRegs.GPBCLEAR.bit.GPIO11=1
//#define NvramDS            GpioDataRegs.GPBSET.bit.GPIO11=1

/*
 *  SPI TCP IP CS, REST, RDY, INT
 */
#define TcpCS             GpioDataRegs.GPBCLEAR.bit.GPIO42=1
#define TcpDS             GpioDataRegs.GPBSET.bit.GPIO42=1

#define TcpResetOn        GpioDataRegs.GPACLEAR.bit.GPIO15=1
#define TcpResetOff       GpioDataRegs.GPASET.bit.GPIO15=1

#define TcpINT            GpioDataRegs.GPADAT.bit.GPIO13

/*
 *  SPI CAN FOR EN, CANINT, CANRX0INT, CANRX1INT,
 */
#define CANBCS             GpioDataRegs.GPBCLEAR.bit.GPIO43=1
#define CANBDS             GpioDataRegs.GPBSET.bit.GPIO43=1

#define CANINT             GpioDataRegs.GPADAT.bit.GPIO12=1
#define CANRX0INT          GpioDataRegs.GPADAT.bit.GPIO14
#define CANRX1INT          GpioDataRegs.GPBDAT.bit.GPIO52

/*
 * RS845 EN
 */
#define RS485EN            GpioDataRegs.GPBCLEAR.bit.GPIO50=1
#define RS485DS            GpioDataRegs.GPBSET.bit.GPIO50=1
/*
 *  BAT IC EN
 */
#define BATEN             GpioDataRegs.GPACLEAR.bit.GPIO10=1
#define BATDS             GpioDataRegs.GPASET.bit.GPIO10=1

/*
 * 80VBAT PROTECT Relay OUT, AUX
 */

#define PRlyOn              GpioDataRegs.GPASET.bit.GPIO22=1
#define PRlyOff             GpioDataRegs.GPACLEAR.bit.GPIO22=1
#define PRlyState           GpioDataRegs.GPADAT.bit.GPIO23

#define NRlyOn              GpioDataRegs.GPASET.bit.GPIO6=1
#define NRlyOff             GpioDataRegs.GPACLEAR.bit.GPIO6=1
#define NRlyState           GpioDataRegs.GPADAT.bit.GPIO7

#define PRORlyOn           GpioDataRegs.GPBSET.bit.GPIO32=1
#define PRORlyOff          GpioDataRegs.GPBCLEAR.bit.GPIO32=1



#define PWRLAMPOn          GpioDataRegs.GPASET.bit.GPIO20=1
#define PWRLAMPOFF         GpioDataRegs.GPACLEAR.bit.GPIO20=1
#define PWRLAMPTog         GpioDataRegs.GPATOGGLE.bit.GPIO20=1

#define EMGSWDI            GpioDataRegs.GPBDAT.bit.GPIO33
#define EMGSWLAMPOn        GpioDataRegs.GPASET.bit.GPIO21=1
#define EMGSWLAMPOff       GpioDataRegs.GPACLEAR.bit.GPIO21=1

/*
 * Insulation Resistance Measurement, IMDTopON. IMDTopOFF,IMDBOTOn,IMDBOTOff
 */

#define IMDTOPOn           GpioDataRegs.GPASET.bit.GPIO26=1
#define IMDTopOff          GpioDataRegs.GPACLEAR.bit.GPIO26=1
#define IMDBOTOn           GpioDataRegs.GPASET.bit.GPIO27=1
#define IMDBOTOff          GpioDataRegs.GPACLEAR.bit.GPIO27=1


// Bit 연산시 일반적으로 쓰이는 부분을 매크로 함수로 정의함  

#define BIT_MASK(bit)			(1 << (bit))
#define GetBit(val, bit)		(((val) & BIT_MASK(bit)) >> (bit))
#define SetBit(val, bit)		(val |= BIT_MASK(bit))
#define ClearBit(val, bit)		(val &= ~BIT_MASK(bit))
#define ToggleBit(val, bit)		(val ^= BIT_MASK(bit))
#define bit_is_set(val, bit)	(val & BIT_MASK(bit))
#define bit_is_clear(val, bit)	(~val & BIT_MASK(bit))

//------------------------------------------------------------------------------------------------
//#define A_PTR(y)			*(volatile unsigned int *)(y)
//#define FPGA_Addr(X)		*(volatile unsigned int *)(0x4000+X) 
//#define PARA_Addr(X)		*(volatile unsigned int *)(0x4200+X)

#define PBYTE(X)                *(volatile unsigned char      *)(X)
#define PWORD(X)                *(volatile unsigned int       *)(X)
#define PLONG(X)                *(volatile unsigned long      *)(X)
#define PLLONG(X)               *(volatile unsigned long long *)(X)

#define SysRegTimer5msec     5
#define SysRegTimer10msec    10
#define SysRegTimer50msec    50
#define SysRegTimer100msec   100
#define SysRegTimer300msec   300
#define SysRegTimer500msec   500
#define SysRegTimer1000msec  1000
//#define CellVoltsampling100msec 100

/*-------------------------------------------------------------------------------
 TMS320F28069 CLK SET UP 
-------------------------------------------------------------------------------*/
#define	CPUCLK				   60000000L							// CPU Main Clock
/*-------------------------------------------------------------------------------
 TMS320F28069 CLK SET UP 
-------------------------------------------------------------------------------*/
#define	SCIA_LSPCLK				(CPUCLK/4)							// Peripheral Low Speed Clock for SCI-A
#define	SCIA_BAUDRATE			9600L								// SCI-A Baudrate
#define	SCIA_BRR_VAL			(SCIA_LSPCLK/(8*SCIA_BAUDRATE)-1)	// SCI-A BaudRate 설정 Register 값

#define	SCIB_LSPCLK				(CPUCLK/4)							// Peripheral Low Speed Clock for SCI-B
#define	SCIB_BAUDRATE			9600L								// SCI-B Baudrate
#define	SCIB_BRR_VAL			(SCIB_LSPCLK/(8*SCIB_BAUDRATE)-1)	// SCI-B BaudRate 설정 Register 값

#define	SCIC_LSPCLK				(CPUCLK/4)							// Peripheral Low Speed Clock for SCI-C
#define	SCIC_BAUDRATE			9600L								// SCI-C Baudrate
#define	SCIC_BRR_VAL			(SCIC_LSPCLK/(8*SCIC_BAUDRATE)-1)	// SCI-C BaudRate 설정 Register 값

/*-------------------------------------------------------------------------------
Parameter
-------------------------------------------------------------------------------*/

// Define the Power Source Parameter
#define PI 							3.14159265358979
#define PIn							-3.14159265358979
#define PI2							6.283185307
#define WE							376.9911184
#define AdcNormalizerBipolar        0.00048828125           // 1 / 4096으로 나눗값
#define AdcNormalizerUnipolar       0.000244140625          // 1 / 2048으로 나눗값
#define AdcNormalizerpolar          0.000322997416          // 1 / 3096으로 나눗값
#define Inverse3					0.333333333			//1/3
#define InverseSQRT3				0.577350269			//1/root3
#define SQRT3						1.732050808			//root3
#define WL_Grid				 		0.150796447368		// 2pi * 60Hz * 400uH
#define TwoBySQRT3 					1.154700			// TwoBySQRT3   = 2/root3
#define Vdc_Minimum					50.0
#define Inverse_Vdc_Minimum			0.02
#define SectoHour                   0.00027778 // 1(h)/3600(sec)
#define Func_Hz                     20 // 1(h)/3600(sec)


/*
 * 162S1P, BATTERY PACK Protect Parameter setup
 */

//#define     Pack_ID                     1


/*
 *
    000 (0) : Battery system initial
    001 (1) : Battery System Ready
    010 (2) : Battery system charging
    011 (3) : Battery system discharging
    100 (4) : Battery system Balancing
 */



#define     Product_SysCellVauleS              168
#define     Product_SysCellVauleP              1
#define     Product_Voltage                    618.2 // 3.664*24
#define     Product_Capacity                   100.0  //
#define     SW_Type                            0   // TODOS : [검증] (32, Product_Type->SW_Type, 초기값 0 (BRA_SW_Type))
#define     Product_Version                    1   // 24.09.21

#define     PackSysVoltMax                     6972 //4.2*24
#define     PackSysVoltMin                     504  //2.8*2.4
#define     PackNum                            0

#define     C_PackCellShutdownFault            2.8
#define     C_PackBalacneLimtVoltage           3.0
#define     C_PackBalanceDivVoltage            0.01 //10mV
#define     C_PackBalanceCurrent               5.0

#define     C_CTDirection               1.0
#define     CurrentDirection            1.0
#define     Cell_Capacity               52

#define     SysVCellVoltCount         168
#define     SysVCellTempCount         168
#define     SysNoramlState              0
#define     SysAlarmlState              1
#define     SysFaltState                2
#define     SysProtecttState            3

#define     SysMondeInitial             0
#define     SysMondeReady               1
#define     SysMondeBalancing           4

#define     NoModePreRelayOnCntVaule    40 //2sec
#define     NoModePRelayOnCntVaule      80 //2sec
#define     NoModePreRelayOffCntVaule   140//3sec

#define     OffModePRelayOnCntVaule     40 //2sec
#define     OffModeNRelayOnCntVaule     80 //2sec
#define     OffModeProRelayOnCntVaule   140//3sec


#define     C_BalanceVoltOffset          0.01

// R01 경고/Fault 판정 Delay : 100ms @ 1ms(cpu_timer0) ISR -> 100 count
#define     C_AlarmFaultDelayCnt                         100

// TODOS : [검증] (33, 방전 경고 과전류 70->150A (R01 반영))
// Alarm Set Vaule  //
#define     C_PackOVPackCurrentCharAlarm                 70.0
#define     C_PackOVPackCurrentDisCharAlarm              150.0   
#define     C_PackOVPkACKSOCAlarm                        95.0
#define     C_PackUDPkACKSOCAlarm                        15.0
#define     C_PackOVPackVoltageAlarm                     688.8   // Cell 4.20V * 24
#define     C_PackUDPackVoltageAlarm                     520.8   // Cell 3.00V * 24
#define     C_PackOVPackTemperatureAlarm                 45.0
#define     C_PackUNPackCharTemperatureAlarm             10.0   // TODOS : [검증] (53, 팩 저온 충/방전 셀과 동일 규약으로 교정: 충전 5->10C)
#define     C_PackUNPackDisCharTemperatureAlarm          5.0    // TODOS : [검증] (53, 팩 저온 충/방전 셀과 동일 규약으로 교정: 방전 10->5C)
#define     C_PackOVCellVoltageAlarm                     4.10
#define     C_PackUDCellVoltageAlarm                     3.10
#define     C_PackDIVCellVoltageAlarm                    0.1
#define     C_PackOVCellTemperatureAlarm                 45.0   // TODOS : [검증] (34, 셀 경고 과온 55->45C (R01 반영))
#define     C_PackUDCellCharTemperatureAlarm             10.0   // TODOS : [검증] (35, 충전 셀 경고 저온 5->10C (R01 반영, 충/방전 값 교정))
#define     C_PackUDCellDisCharTemperatureAlarm          5.0    // TODOS : [검증] (36, 방전 셀 경고 저온 10->5C (R01 반영, 충/방전 값 교정))
#define     C_PackDIVCellTemperatureAlarm                10.0
// 셀 내부저항(IR) 경고 트리거(13~15) - R01 미정의, 측정 미구현, placeholder   // TODOS : [검증] (51, 셀IR 경고 임계 미정)
#define     C_PackOVCellIRAlarm                          100.0   // 13 셀 IR 과대
#define     C_PackUDCellIRAlarm                            1.0   // 14 셀 IR 과소
#define     C_PackDIVCellIRAlarm                          50.0   // 15 셀 IR 편차
// TODOS : [검증] (37, R01 명시 Release Point 상수화)
// R01 Release(reset) Point - Alarm 히스테리시스
#define     C_PackOVPackCurrentCharAlarmRls              67.2
#define     C_PackOVPackCurrentDisCharAlarmRls           145.5
#define     C_PackOVPkACKSOCAlarmRls                     90.25
#define     C_PackUDPkACKSOCAlarmRls                     16.0
#define     C_PackOVPackVoltageAlarmRls                  681.9
#define     C_PackUDPackVoltageAlarmRls                  526.0
#define     C_PackOVPackTemperatureAlarmRls              43.2   // 5  팩 과온 복귀   // TODOS : [검증] (31, 팩 과온/저온 경고 복귀값)
#define     C_PackUNPackCharTemperatureAlarmRls          10.4   // 6  팩 저온(충전) 복귀(교정)
#define     C_PackUNPackDisCharTemperatureAlarmRls       5.2    // 6  팩 저온(방전) 복귀(교정)
#define     C_PackOVCellVoltageAlarmRls                  4.059
#define     C_PackUDCellVoltageAlarmRls                  3.131
#define     C_PackDIVCellVoltageAlarmRls                 0.095
#define     C_PackOVCellTemperatureAlarmRls              43.2
#define     C_PackUDCellCharTemperatureAlarmRls          10.4
#define     C_PackUDCellDisCharTemperatureAlarmRls       5.2
#define     C_PackDIVCellTemperatureAlarmRls             9.5
#define     C_PackOVCellIRAlarmRls                        95.0   // 13 셀 IR 과대 복귀
#define     C_PackUDCellIRAlarmRls                        1.05   // 14 셀 IR 과소 복귀
#define     C_PackDIVCellIRAlarmRls                       47.5   // 15 셀 IR 편차 복귀


// TODOS : [검증] (38, 방전 Fault 과전류 80->200A (R01 반영))
//Fault Set Vaule
#define     C_PackOVPackCurrentCharFault                 80.0
#define     C_PackOVPackCurrentDisCharFault              200.0   
#define     C_PackOVPackSOCFault                         100.0
#define     C_PackUDPackSOCFault                         5.0//-0.1
#define     C_PackOVPackVoltageFault                     697.2  // Cell 4.25V * 24
#define     C_PackUDPackVoltageFault                     504.0  // Cell 2.80V * 24
#define     C_PackOVPackTemperatureFault                 50.0
#define     C_PackUNPackCharTemperatureFalut             0.0
#define     C_PackUNPackDisCharTemperatureFault          0.0
#define     C_PackOVCellVoltageFault                     4.15
#define     C_PackUDCellVoltageFault                     3.00
#define     C_PackDIVCellVoltageFault                    0.3
#define     C_PackOVCellTemperatureFault                 50.0
#define     C_PackUDCellCharTemperatureFault             0.0
#define     C_PackUDCellDisCharTemperatureFault          0.0
#define     C_PackDIVCellTemperatureFault                13.0   // TODOS : [검증] (39, 셀 온도편차 Fault 15->13C (R01 반영))
// 셀 내부저항(IR) Fault 트리거(13~15) - R01 미정의, 측정 미구현, placeholder   // TODOS : [검증] (56, 셀IR Fault 임계 미정)
#define     C_PackOVCellIRFault                          120.0   // 13 셀 IR 과대
#define     C_PackUDCellIRFault                            0.5    // 14 셀 IR 과소
#define     C_PackDIVCellIRFault                          70.0    // 15 셀 IR 편차
#define     C_IOSresistanceFault                         45000
// TODOS : [검증] (40, R01 명시 Release Point 상수화)
// R01 Release(reset) Point - Fault 히스테리시스        
#define     C_PackOVPackCurrentCharFaultRls             76.0
#define     C_PackOVPackCurrentDisCharFaultRls          170.0
#define     C_PackOVPackSOCFaultRls                     97.0
#define     C_PackUDPackSOCFaultRls                     5.5
#define     C_PackOVPackVoltageFaultRls                 693.7
#define     C_PackUDPackVoltageFaultRls                 511.5
// 팩 평균온도 Fault 복귀값 (R01 미정의, 셀과 동일 규약: 충전 복귀↑)   // TODOS : [검증] (55, 팩 과온/저온 Fault 복귀값)
#define     C_PackOVPackTemperatureFaultRls             47.5   // 5  팩 과온 복귀
#define     C_PackUNPackCharTemperatureFaultRls         7.0    // 6  팩 저온(충전) 복귀
#define     C_PackUNPackDisCharTemperatureFaultRls      3.0    // 6  팩 저온(방전) 복귀
#define     C_PackOVCellVoltageFaultRls                 4.120
#define     C_PackUDCellVoltageFaultRls                 3.06
#define     C_PackDIVCellVoltageFaultRls                0.21
#define     C_PackOVCellTemperatureFaultRls             47.5
#define     C_PackUDCellCharTemperatureFaultRls         7.0   // TODOS : [검증] (54, Fault 충전 셀저온 복귀 5->7C)
#define     C_PackUDCellDisCharTemperatureFaultRls      3.0
#define     C_PackDIVCellTemperatureFaultRls            11.7
#define     C_PackOVCellIRFaultRls                       114.0   // 13 셀 IR 과대 복귀
#define     C_PackUDCellIRFaultRls                         0.55  // 14 셀 IR 과소 복귀
#define     C_PackDIVCellIRFaultRls                       66.5   // 15 셀 IR 편차 복귀

// Fault 항목별 Delay 카운트 (1ms ISR 기준, 기본 100=100ms). 항목별 개별 튜닝   // TODOS : [튜닝] (60, Fault 항목별 delay 분리)
#define     C_OcFaultDelayCnt                            100   // 0  과전류
#define     C_SocOvFaultDelayCnt                         100   // 1  SOC High
#define     C_SocUnFaultDelayCnt                         100   // 2  SOC Low
#define     C_VoltOvFaultDelayCnt                        100   // 3  팩 과전압
#define     C_VoltUnFaultDelayCnt                        100   // 4  팩 저전압
#define     C_PackTempOtFaultDelayCnt                    100   // 5  팩 과온
#define     C_PackTempUtFaultDelayCnt                    100   // 6  팩 저온
#define     C_CellVoltOvFaultDelayCnt                    100   // 7  셀 과전압
#define     C_CellVoltUnFaultDelayCnt                    100   // 8  셀 저전압
#define     C_CellVoltBlFaultDelayCnt                    100   // 9  셀 전압편차
#define     C_CellTempOtFaultDelayCnt                    100   // 10 셀 과온
#define     C_CellTempUtFaultDelayCnt                    100   // 11 셀 저온
#define     C_CellTempBlFaultDelayCnt                    100   // 12 셀 온도편차
#define     C_CellIrOvFaultDelayCnt                      100   // 13 셀 IR 과대
#define     C_CellIrUnFaultDelayCnt                      100   // 14 셀 IR 과소
#define     C_CellIrDivFaultDelayCnt                     100   // 15 셀 IR 편차


//Protect Set Vaule
#define     C_PackOVPackCurrentCharProtect                 100.0
#define     C_PackOVPackCurrentDisCharProtect              250.0  // TODOS : [검증] (41, 방전 보호 과전류 100->250A (R01 반영))
#define     C_PackOVPackSOCProtect                         103.0
#define     C_PackUDPackSOCProtect                          0.0   // TODOS : [검증] (42, 보호 SOC Low -0.1->0% (R01 반영))
#define     C_PackOVPackVoltageProtect                     702.2  // Cell 4.25V * 24
#define     C_PackUDPackVoltageProtect                     478.8   // Cell 2.80V * 24
#define     C_PackOVPackTemperatureProtect                 54.0
#define     C_PackUNPackCharTemperatureProtect             -10.0
#define     C_PackUNPackDisCharTemperatureProtect          -15.0
#define     C_PackOVCellVoltageProtect                     4.18
#define     C_PackUDCellVoltageProtect                     2.85
#define     C_PackDIVCellVoltageProtect                    0.4    // TODOS : [검증] (43, 셀 전압편차 보호 0.5->0.4V (R01 반영))
#define     C_PackOVCellTemperatureProtect                 54.0   // TODOS : [검증] (44, 셀 보호 과온 55->54C (R01 반영))
#define     C_PackUDCellCharTemperatureProtect             -10.0
#define     C_PackUDCellDisCharTemperatureProtect          -15.0
#define     C_PackDIVCellTemperatureProtectt                15.0
// 셀 내부저항(IR) 보호 - R01 미정의, 측정 미구현. 즉시차단, 임계 placeholder   // TODOS : [검증] (58, 셀IR 보호 임계값 미정 - 측정 구현 후 확정)
#define     C_PackOVCellIRProtect                          140.0   // 셀 IR 과대(placeholder)
#define     C_PackUDCellIRProtect                            0.3   // 셀 IR 과소(placeholder)
#define     C_PackDIVCellIRProtect                          90.0   // 셀 IR 편차(placeholder)
// 불평형전력 보호 (bit16,17) [kW] - 즉시차단   // TODOS : [검증] (61, 불평형전력 보호 설정값 추가)
#define     C_PackDisCharUnBalPWRProtect                   300.0   // 16 방전 불평형전력
#define     C_PackCharUnBalPWRProtect                      150.0   // 17 충전 불평형전력
#define     C_IOSresistanceProtect                         45000



#define SectoHour                   0.00027778 // 1(h)/3600(sec)
#define Func_Hz                     20 // 1(h)/3600(sec)
#define SocCumulativeTime           0.00027778 //1/3600
#define SocCurrentSampleTime        0.05


#endif  // end of PARAMETER.H definition



//===========================================================================
// No more.
//===========================================================================

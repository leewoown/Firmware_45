/* ==============================================================================
System Name:  �����ڵ��� ���� ������ 80V

File Name:		PARAMETER.H

Description:	����
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

/* Bit ��ġ ����(�ַ� ������ ������ ��ũ�� �Լ����� ����ϱ� ���� ������) */


#define	SCIA_BUFRX		50			// Monstar�� ���߾�� ��

//#define UL_BYTE(x)		    (x >> 16)
//#define HI_BYTE(x)		    (x >> 8)
//#define LO_BYTE(x)          (x & 0xff)
//#define MAKE_WORD(msb,lsb)	((msb<<8) | (lsb))
//#define WordLShift(md,ml)   (md<<ml)
//#define WordRShift(md,ml)   (md>>ml)



// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 	    10
#define SYSTEM_FREQUENCY    90
#define CPUCLK			    90000000L					// CPU Main Clock
#define CPLDCLK			    100000000L					// CPLD Clock

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
#define IDSW01           GpioDataRegs.GPADAT.bit.GPIO16
#define IDSW02           GpioDataRegs.GPADAT.bit.GPIO17

/*
 * SPI chip RTC CS, MFP DIO
 */

#define RTC_CS            GpioDataRegs.GPACLEAR.bit.GPIO9=1
#define RTC_DS            GpioDataRegs.GPASET.bit.GPIO9=1
#define RTC_MF            GpioDataRegs.GPCDAT.bit.GPIO8
/*
 * SPI chip NVRAM CS
 */
#define NvramCS            GpioDataRegs.GPBCLEAR.bit.GPIO11=1
#define NvramDS            GpioDataRegs.GPBSET.bit.GPIO11=1

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


// Bit ����� �Ϲ������� ���̴� �κ��� ��ũ�� �Լ��� ������  

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
#define CellVoltsampling100msec 100

/*-------------------------------------------------------------------------------
 TMS320F28335 CLK SET UP 
-------------------------------------------------------------------------------*/
#define	CPUCLK				   90000000L							// CPU Main Clock
/*-------------------------------------------------------------------------------
 TMS320F28335 CLK SET UP 
-------------------------------------------------------------------------------*/
#define	SCIA_LSPCLK				(CPUCLK/4)							// Peripheral Low Speed Clock for SCI-A
#define	SCIA_BAUDRATE			9600L								// SCI-A Baudrate
#define	SCIA_BRR_VAL			(SCIA_LSPCLK/(8*SCIA_BAUDRATE)-1)	// SCI-A BaudRate ���� Register ��

#define	SCIB_LSPCLK				(CPUCLK/4)							// Peripheral Low Speed Clock for SCI-B
#define	SCIB_BAUDRATE			9600L								// SCI-B Baudrate
#define	SCIB_BRR_VAL			(SCIB_LSPCLK/(8*SCIB_BAUDRATE)-1)	// SCI-B BaudRate ���� Register ��

#define	SCIC_LSPCLK				(CPUCLK/4)							// Peripheral Low Speed Clock for SCI-C
#define	SCIC_BAUDRATE			9600L								// SCI-C Baudrate
#define	SCIC_BRR_VAL			(SCIC_LSPCLK/(8*SCIC_BAUDRATE)-1)	// SCI-C BaudRate ���� Register ��

/*-------------------------------------------------------------------------------
Parameter
-------------------------------------------------------------------------------*/

// Define the Power Source Parameter
#define PI 							3.14159265358979
#define PIn							-3.14159265358979
#define PI2							6.283185307
#define WE							376.9911184
#define AdcNormalizerBipolar        0.00048828125           // 1 / 4096���� ������
#define AdcNormalizerUnipolar       0.000244140625          // 1 / 2048���� ������
#define AdcNormalizerpolar          0.000322997416          // 1 / 3096���� ������
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
#define     Product_Type                       2   // 24.09.21
#define     Product_Version                    1   // 24.09.21

#define     PackSysVoltMax                     6972 //4.2*24
#define     PackSysVoltMin                     504  //2.8*2.4
#define     PackNum                            2

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

//#define     UnbalancePower          72

//Alarm Set Vaule
/*
#define     A_OVDisCurrent         -150.0
#define     A_OVCHACurrent         60.0
#define     A_OVPackSOC            95.0
#define     A_UDPackSOC            15.0
#define     A_OVPackVoltage        713.4
#define     A_UDPackVoltage        556.8//3.2
#define     A_OVCellVoltage        4.1
#define     A_UDCellVoltage        3.2
#define     A_DIVCellVoltage       0.1    // ���� Cell 100���� ����
#define     A_OVCellTemperature    35.0
#define     A_UDCellTemperature    5.0
#define     A_DIVCellTemperature   5.0
#define     A_UnbalancePower       150
#define     A_RelayState           0
#define     A_MSDState             0
*/

// Alarm Set Vaule  //
#define     C_PackOVPackCurrentAlarm                     450.0//450.0//480.0
#define     C_PackOVPkACKSOCAlarm                        100.0
#define     C_PackUDPkACKSOCAlarm                        5.0
#define     C_PackOVPackVoltageAlarm                     108.8   // Cell 4.20V * 24
#define     C_PackUDPackVoltageAlarm                     72.0   // Cell 3.00V * 24
#define     C_PackOVPackTemperatureAlarm                 55.0
#define     C_PackUNPackTemperatureAlarm                -15.0
#define     C_PackOVCellVoltageAlarm                     4.20
#define     C_PackUDCellVoltageAlarm                     3.00
#define     C_PackDIVCellVoltageAlarm                    0.2
#define     C_PackOVCellTemperatureAlarm                 55.0
#define     C_PackUDCellTemperatureAlarm                -15.0
#define     C_PackDIVCellTemperatureAlarm                10.0

//Fault Set Vaule
#define     C_PackFaultDelayCount                        10
#define     C_ISOSPICount                                  50
#define     C_CANCount                                     50
#define     C_RleyCount                                    1
#define     C_PackOVPackCurrentFault                     500.0//500.0
#define     C_PackOVPackSOCFault                         101.0
#define     C_PackUDPackSOCFault                          -0.1//-0.1
#define     C_PackOVPackVoltageFault                     102.0  // Cell 4.25V * 24
#define     C_PackUDPackVoltageFault                     67.2   // Cell 2.80V * 24
#define     C_PackOVPackTemperatureFault                 60.0
#define     C_PackUNPackTemperatureFault                -30.0
#define     C_PackOVCellVoltageFault                     4.25
#define     C_PackUDCellVoltageFault                     2.50
#define     C_PackDIVCellVoltageFault                    0.5
#define     C_PackOVCellTemperatureFault                 60.0
#define     C_PackUDCellTemperatureFault                -25.0
#define     C_PackDIVCellTemperatureFault                15.0
#define     C_IOSresistanceFault                         45000


//Protect Set Vaule
#define     C_PackFaultDelayCount                          10
#define     C_ISOSPICount                                  50
#define     C_CANCount                                     50
#define     C_RleyCount                                    1
#define     C_PackOVPackCurrentProtect                     500.0//500.0
#define     C_PackOVPackSOCProtect                         101.0
#define     C_PackUDPackSOCProtect                          -0.1//-0.1
#define     C_PackOVPackVoltageProtect                     102.0  // Cell 4.25V * 24
#define     C_PackUDPackVoltageProtect                     67.2   // Cell 2.80V * 24
#define     C_PackOVPackTemperatureProtect                 60.0
#define     C_PackUNPackTemperatureProtect                -30.0
#define     C_PackOVCellVoltageProtect                     4.25
#define     C_PackUDCellVoltageProtect                     2.50
#define     C_PackDIVCellVoltageProtect                    0.5
#define     C_PackOVCellTemperatureProtect                 60.0
#define     C_PackUDCellTemperatureProtect                -25.0
#define     C_PackDIVCellTemperatureProtect                15.0
#define     C_IOSresistanceProtect                         45000


#define     C_PackCurrentProtectDelay                       0
#define     C_PackSOCProtectDelay                           0
#define     C_PackSOCProtectDelay                           0
#define     C_PackVoltageProtectDelay                       0
#define     C_PackVoltageProtectDelay                       0
#define     C_PackTemperatureProtectDelay                   0
#define     C_PackTemperatureProtectDelay                   0
#define     C_CellVoltageOVProtectDelay                     0
#define     C_CellVoltageUDProtectDelay                     0
#define     C_CellVoltageDIVProtectDelay                    0
#define     C_CellTemperatureOVProtectDelay                 0
#define     C_UDCellTemperatureOVProtectDelay               0
#define     C_CellTemperatureDIVProtectDelay                0



#define SectoHour                   0.00027778 // 1(h)/3600(sec)
#define Func_Hz                     20 // 1(h)/3600(sec)
#define SocCumulativeTime           0.00027778 //1/3600
#define SocCurrentSampleTime        0.05


#endif  // end of PARAMETER.H definition



//===========================================================================
// No more.
//===========================================================================

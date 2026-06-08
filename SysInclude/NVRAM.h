/*
 * NVRAM.h
 *
 *  Created on: 2025. 11. 9.
 *  Author: leewoowon
 */

#ifndef SYSINCLUDE_NVRAM_H_
#define SYSINCLUDE_NVRAM_H_

//#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
//#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "F2806x_Examples.h"    // F2806x Examples Include File
#include "DSP28x_Project.h"

#define NVR_WREN_CMD        0x03
#define NVR_WRDI_CMD        0x02
#define NVR_RDSR_CMD        0x04
#define NVR_WRSR_CMD        0x06
#define NVR_READ_CMD        0x05
#define NVR_WRITE_CMD       0x01
#define NVR_SLEEP_CMD       0x13
#define NVR_WAKE_CMD        0x12
#define NvramCS   GpioDataRegs.GPACLEAR.bit.GPIO11 = 1
#define NvramDS   GpioDataRegs.GPASET.bit.GPIO11 = 1

typedef enum
{
   Nvram_INIT,
   Nvram_STANDBY,
   Nvram_READY,
   Nvram_EventRead,
   Nvram_EventWirte,
   Nvram_SoCWite,
   Nvram_SoCResd,
   Nvram_ManualRead,
   Nvram_ManualWite,
   Nvram_ForceReset
}NvramState;
struct NvramStatus_BIT
{       // bits   description
    unsigned int     Active        :1; // 0
    unsigned int     WDReturn      :1; // 1
    unsigned int     RDReturn      :1; // 2
    unsigned int     SW03          :1; // 3
    unsigned int     SW04          :1; // 4
    unsigned int     SW05          :1; // 5
    unsigned int     SW06          :1; // 6
    unsigned int     SW07          :1; // 7
    unsigned int     SW08          :1; // 8
    unsigned int     SW09          :1; // 9
    unsigned int     SW10          :1; // 10
    unsigned int     SW11          :1; // 11
    unsigned int     SW12          :1; // 12
    unsigned int     SW13          :1; // 13
    unsigned int     SW14          :1; // 14
    unsigned int     SW15          :1; // 15
    unsigned int     SW16          :1; // 16
    unsigned int     SW17          :1; // 17
    unsigned int     SW18          :1; // 18
    unsigned int     SW19          :1; // 19
    unsigned int     SW20          :1; // 20
    unsigned int     SW21          :1; // 21
    unsigned int     SW22          :1; // 22
    unsigned int     SW23          :1; // 23
    unsigned int     SW24          :1; // 24
    unsigned int     SW25          :1; // 25
    unsigned int     SW26          :1; // 26
    unsigned int     SW27          :1; // 27
    unsigned int     SW28          :1; // 28
    unsigned int     SW29          :1; // 29
    unsigned int     SW30          :1; // 30
    unsigned int     SW31          :1; // 31
};
union NvramStatus_REG
{
   Uint32                   all;
   struct Data_WORD         Word;
   struct NvramStatus_BIT bit;
};
typedef struct NVRamR_Date
{
    Uint8 WDCMD;
    Uint8 WDAdder;
    const Uint8 *WDPtr;
    Uint16 WDLen;
    Uint8 RDCMD;
    Uint8 RDAdder;
    Uint8 *RDPtr;
    Uint16 RDLen;
    Uint16 DebegCnt;
    Uint32 SysTick;
    Uint32 NvrSeq;
    union NvramStatus_REG   NvramStatus;
}NvramReg;

#endif /* SYSINCLUDE_NVRAM_H_ */

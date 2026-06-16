/* ==============================================================================
*/
#ifndef BATCellModel_H

#define BATCellModel_H

#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "F2806x_Examples.h"    // F2806x Examples Include File
#include "DSP28x_Project.h"

#ifdef __cplusplus
 extern "C"
 {
#endif

#define BATTERY_MODEL_DT_SEC           (0.001f)
#define BATTERY_CURRENT_DEADBAND_A     (0.5f)
#define BATTERY_SOC_MIN_PCT            (0.0f)
#define BATTERY_SOC_MAX_PCT            (100.0f)

#define BATTERY_CELL_MAX_V             (4.250f)
#define BATTERY_CELL_MIN_V             (2.500f)

typedef struct
{
    float socPct;     /* SOC [%] */
    float value;      /* OCV[V], R0[mOhm], R1[mOhm] */
} BatteryTablePoint;

typedef struct
{
    float initSocPct;     /* 초기 SOC [%] */
    float socPct;         /* 현재 SOC [%] */

    float capacityAh;     /* 셀 용량 [Ah] */
    float capacityAs;     /* 셀 용량 [As] */

    float currentA;       /* 현재 전류 [A], 충전>0 / 방전<0 */

    float ocvV;           /* OCV [V] */
    float r0Ohm;          /* R0 [Ohm] */
    float r1Ohm;          /* R1 [Ohm] */
    float c1F;            /* C1 [F] */
    float tauSec;         /* R1*C1 [sec] */

    float irV;            /* I*R0 전압 성분 [V] */
    float rcV;            /* RC 분극 전압 [V] */

    float tvV;            /* 단자전압 [V] */
    float outV;           /* 최종 출력 전압 [V] */

    const BatteryTablePoint *pOcvTable;
    Uint16 ocvTableSize;

    const BatteryTablePoint *pChargeR0Table;
    Uint16 chargeR0TableSize;

    const BatteryTablePoint *pDischargeR0Table;
    Uint16 dischargeR0TableSize;

    const BatteryTablePoint *pChargeR1Table;
    Uint16 chargeR1TableSize;

    const BatteryTablePoint *pDischargeR1Table;
    Uint16 dischargeR1TableSize;

} BatteryModel;

/* Trace / Expressions용 전역 변수 */
extern float gBattery_InitSocPct;
extern float gBattery_CapacityAh;
extern float gBattery_InputCurrentA;
extern float gBattery_C1_F;

extern float gTrace_SOC;
extern float gTrace_OCV;
extern float gTrace_R0;
extern float gTrace_R1;
extern float gTrace_RC;
extern float gTrace_IR;
extern float gTrace_TV;
extern float gTrace_OutV;

extern BatteryModel gBatteryModel;

float BatteryModel_Lookup(const BatteryTablePoint *pTable,
                          Uint16 tableSize,
                          float socPct);

void BatteryModel_Init(BatteryModel *pModel,
                       float initSocPct,
                       float capacityAh,
                       float c1F,
                       const BatteryTablePoint *pOcvTable,
                       Uint16 ocvTableSize,
                       const BatteryTablePoint *pChargeR0Table,
                       Uint16 chargeR0TableSize,
                       const BatteryTablePoint *pDischargeR0Table,
                       Uint16 dischargeR0TableSize,
                       const BatteryTablePoint *pChargeR1Table,
                       Uint16 chargeR1TableSize,
                       const BatteryTablePoint *pDischargeR1Table,
                       Uint16 dischargeR1TableSize);

void BatteryModel_Reset(BatteryModel *pModel, float initSocPct);
void BatteryModel_Step(BatteryModel *pModel, float currentA);

void BatteryModel_UserInit(void);
void BatteryModel_1msTask(void);
#ifdef __cplusplus
 }
#endif

#endif  // end of BATCellModel_H definition


//===========================================================================
// No more.
//===========================================================================

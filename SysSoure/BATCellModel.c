#include "DSP28x_Project.h"
#include "BATAlgorithm.h"
#include "BATCellModel.h"
#include "stdio.h"
#include "math.h"
#include "string.h"


static float BatteryModel_Abs(float x);
static float BatteryModel_Clamp(float value, float minVal, float maxVal);

/*-----------------------------------------------------------
 Trace / Debug용 전역 변수
-----------------------------------------------------------*/
float gBattery_InitSocPct   = 50.0f;      /* 초기 SOC [%] */
float gBattery_CapacityAh   = 55.0f;      /* 단일 셀 용량 [Ah] */
float gBattery_InputCurrentA = 0.0f;      /* 입력 전류 [A] */
float gBattery_C1_F         = 5000.0f;    /* RC 모델 C1 [F] */

float gTrace_SOC   = 0.0f;
float gTrace_OCV   = 0.0f;
float gTrace_R0    = 0.0f;
float gTrace_R1    = 0.0f;
float gTrace_RC    = 0.0f;
float gTrace_IR    = 0.0f;
float gTrace_TV    = 0.0f;
float gTrace_OutV  = 0.0f;

BatteryModel gBatteryModel;

/*-----------------------------------------------------------
 OCV Table
-----------------------------------------------------------*/
static const BatteryTablePoint OcvTable[] =
{
    {  0.0f, 3.458f },
    {  2.0f, 3.472f },
    {  4.0f, 3.485f },
    {  6.0f, 3.499f },
    {  8.0f, 3.512f },
    { 10.0f, 3.526f },
    { 12.0f, 3.540f },
    { 14.0f, 3.553f },
    { 16.0f, 3.567f },
    { 18.0f, 3.580f },
    { 20.0f, 3.594f },
    { 22.0f, 3.601f },
    { 24.0f, 3.607f },
    { 26.0f, 3.614f },
    { 28.0f, 3.620f },
    { 30.0f, 3.627f },
    { 32.0f, 3.633f },
    { 34.0f, 3.640f },
    { 36.0f, 3.646f },
    { 38.0f, 3.653f },
    { 40.0f, 3.659f },
    { 42.0f, 3.669f },
    { 44.0f, 3.679f },
    { 46.0f, 3.688f },
    { 48.0f, 3.698f },
    { 50.0f, 3.708f },
    { 52.0f, 3.728f },
    { 54.0f, 3.748f },
    { 56.0f, 3.769f },
    { 58.0f, 3.789f },
    { 60.0f, 3.809f },
    { 62.0f, 3.828f },
    { 64.0f, 3.846f },
    { 66.0f, 3.865f },
    { 68.0f, 3.883f },
    { 70.0f, 3.902f },
    { 72.0f, 3.921f },
    { 74.0f, 3.940f },
    { 76.0f, 3.958f },
    { 78.0f, 3.977f },
    { 80.0f, 3.996f },
    { 82.0f, 4.016f },
    { 84.0f, 4.036f },
    { 86.0f, 4.055f },
    { 88.0f, 4.075f },
    { 90.0f, 4.095f },
    { 92.0f, 4.115f },
    { 94.0f, 4.135f },
    { 96.0f, 4.154f },
    { 98.0f, 4.174f },
    {100.0f, 4.194f }
};

/*-----------------------------------------------------------
 R0 Table (20s 기준, mOhm)
-----------------------------------------------------------*/
static const BatteryTablePoint R0Table[] =
{
    {  0.0f, 0.900f },
    {  2.0f, 0.898f },
    {  4.0f, 0.896f },
    {  6.0f, 0.894f },
    {  8.0f, 0.892f },
    { 10.0f, 0.890f },
    { 12.0f, 0.890f },
    { 14.0f, 0.890f },
    { 16.0f, 0.890f },
    { 18.0f, 0.890f },
    { 20.0f, 0.890f },
    { 22.0f, 0.890f },
    { 24.0f, 0.890f },
    { 26.0f, 0.890f },
    { 28.0f, 0.890f },
    { 30.0f, 0.890f },
    { 32.0f, 0.884f },
    { 34.0f, 0.878f },
    { 36.0f, 0.872f },
    { 38.0f, 0.866f },
    { 40.0f, 0.860f },
    { 42.0f, 0.858f },
    { 44.0f, 0.856f },
    { 46.0f, 0.854f },
    { 48.0f, 0.852f },
    { 50.0f, 0.850f },
    { 52.0f, 0.856f },
    { 54.0f, 0.862f },
    { 56.0f, 0.868f },
    { 58.0f, 0.874f },
    { 60.0f, 0.880f },
    { 62.0f, 0.886f },
    { 64.0f, 0.892f },
    { 66.0f, 0.898f },
    { 68.0f, 0.904f },
    { 70.0f, 0.910f },
    { 72.0f, 0.922f },
    { 74.0f, 0.934f },
    { 76.0f, 0.946f },
    { 78.0f, 0.958f },
    { 80.0f, 0.970f },
    { 82.0f, 1.006f },
    { 84.0f, 1.042f },
    { 86.0f, 1.078f },
    { 88.0f, 1.114f },
    { 90.0f, 1.150f },
    { 92.0f, 1.186f },
    { 94.0f, 1.222f },
    { 96.0f, 1.258f },
    { 98.0f, 1.294f },
    {100.0f, 1.330f }
};

/*-----------------------------------------------------------
 R1 Table (R0의 일부를 polarization 성분으로 사용, mOhm)
 실측 RC가 없으므로 임시 근사값
-----------------------------------------------------------*/
static const BatteryTablePoint R1Table[] =
{
    {  0.0f, 0.450f },
    {  2.0f, 0.449f },
    {  4.0f, 0.448f },
    {  6.0f, 0.447f },
    {  8.0f, 0.446f },
    { 10.0f, 0.445f },
    { 12.0f, 0.445f },
    { 14.0f, 0.445f },
    { 16.0f, 0.445f },
    { 18.0f, 0.445f },
    { 20.0f, 0.445f },
    { 22.0f, 0.445f },
    { 24.0f, 0.445f },
    { 26.0f, 0.445f },
    { 28.0f, 0.445f },
    { 30.0f, 0.445f },
    { 32.0f, 0.442f },
    { 34.0f, 0.439f },
    { 36.0f, 0.436f },
    { 38.0f, 0.433f },
    { 40.0f, 0.430f },
    { 42.0f, 0.429f },
    { 44.0f, 0.428f },
    { 46.0f, 0.427f },
    { 48.0f, 0.426f },
    { 50.0f, 0.425f },
    { 52.0f, 0.428f },
    { 54.0f, 0.431f },
    { 56.0f, 0.434f },
    { 58.0f, 0.437f },
    { 60.0f, 0.440f },
    { 62.0f, 0.443f },
    { 64.0f, 0.446f },
    { 66.0f, 0.449f },
    { 68.0f, 0.452f },
    { 70.0f, 0.455f },
    { 72.0f, 0.461f },
    { 74.0f, 0.467f },
    { 76.0f, 0.473f },
    { 78.0f, 0.479f },
    { 80.0f, 0.485f },
    { 82.0f, 0.503f },
    { 84.0f, 0.521f },
    { 86.0f, 0.539f },
    { 88.0f, 0.557f },
    { 90.0f, 0.575f },
    { 92.0f, 0.593f },
    { 94.0f, 0.611f },
    { 96.0f, 0.629f },
    { 98.0f, 0.647f },
    {100.0f, 0.665f }
};

#define OCV_TABLE_SIZE   ((Uint16)(sizeof(OcvTable) / sizeof(OcvTable[0])))
#define R0_TABLE_SIZE    ((Uint16)(sizeof(R0Table)  / sizeof(R0Table[0])))
#define R1_TABLE_SIZE    ((Uint16)(sizeof(R1Table)  / sizeof(R1Table[0])))

/*-----------------------------------------------------------
 내부 함수
-----------------------------------------------------------*/
static float BatteryModel_Abs(float x)
{
    if (x < 0.0f)
    {
        return -x;
    }

    return x;
}

static float BatteryModel_Clamp(float value, float minVal, float maxVal)
{
    if (value < minVal)
    {
        return minVal;
    }

    if (value > maxVal)
    {
        return maxVal;
    }

    return value;
}

/*-----------------------------------------------------------
 Lookup Table 선형 보간
-----------------------------------------------------------*/
float BatteryModel_Lookup(const BatteryTablePoint *pTable,
                          Uint16 tableSize,
                          float socPct)
{
    Uint16 i;
    float x0;
    float x1;
    float y0;
    float y1;
    float ratio;

    if ((pTable == 0) || (tableSize == 0u))
    {
        return 0.0f;
    }

    if (tableSize == 1u)
    {
        return pTable[0].value;
    }

    if (socPct <= pTable[0].socPct)
    {
        return pTable[0].value;
    }

    if (socPct >= pTable[tableSize - 1u].socPct)
    {
        return pTable[tableSize - 1u].value;
    }

    for (i = 0u; i < (tableSize - 1u); i++)
    {
        x0 = pTable[i].socPct;
        x1 = pTable[i + 1u].socPct;

        if ((socPct >= x0) && (socPct <= x1))
        {
            y0 = pTable[i].value;
            y1 = pTable[i + 1u].value;

            if ((x1 - x0) == 0.0f)
            {
                return y0;
            }

            ratio = (socPct - x0) / (x1 - x0);
            return y0 + ((y1 - y0) * ratio);
        }
    }

    return pTable[tableSize - 1u].value;
}

/*-----------------------------------------------------------
 모델 초기화
-----------------------------------------------------------*/
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
                       Uint16 dischargeR1TableSize)
{
    if (pModel == 0)
    {
        return;
    }

    pModel->initSocPct = BatteryModel_Clamp(initSocPct,
                                            BATTERY_SOC_MIN_PCT,
                                            BATTERY_SOC_MAX_PCT);

    pModel->socPct = pModel->initSocPct;

    pModel->capacityAh = capacityAh;
    pModel->capacityAs = capacityAh * 3600.0f;

    pModel->currentA = 0.0f;

    pModel->ocvV = 0.0f;
    pModel->r0Ohm = 0.0f;
    pModel->r1Ohm = 0.0f;
    pModel->c1F = c1F;
    pModel->tauSec = 0.0f;

    pModel->irV = 0.0f;
    pModel->rcV = 0.0f;
    pModel->tvV = 0.0f;
    pModel->outV = 0.0f;

    pModel->pOcvTable = pOcvTable;
    pModel->ocvTableSize = ocvTableSize;

    pModel->pChargeR0Table = pChargeR0Table;
    pModel->chargeR0TableSize = chargeR0TableSize;

    pModel->pDischargeR0Table = pDischargeR0Table;
    pModel->dischargeR0TableSize = dischargeR0TableSize;

    pModel->pChargeR1Table = pChargeR1Table;
    pModel->chargeR1TableSize = chargeR1TableSize;

    pModel->pDischargeR1Table = pDischargeR1Table;
    pModel->dischargeR1TableSize = dischargeR1TableSize;

    pModel->ocvV = BatteryModel_Lookup(pModel->pOcvTable,
                                       pModel->ocvTableSize,
                                       pModel->socPct);

    pModel->tvV = pModel->ocvV;
    pModel->outV = pModel->ocvV;
}

void BatteryModel_Reset(BatteryModel *pModel, float initSocPct)
{
    if (pModel == 0)
    {
        return;
    }

    pModel->initSocPct = BatteryModel_Clamp(initSocPct,
                                            BATTERY_SOC_MIN_PCT,
                                            BATTERY_SOC_MAX_PCT);

    pModel->socPct = pModel->initSocPct;
    pModel->currentA = 0.0f;
    pModel->irV = 0.0f;
    pModel->rcV = 0.0f;

    pModel->ocvV = BatteryModel_Lookup(pModel->pOcvTable,
                                       pModel->ocvTableSize,
                                       pModel->socPct);

    pModel->tvV = pModel->ocvV;
    pModel->outV = pModel->ocvV;
}

/*-----------------------------------------------------------
 1ms 주기 계산
-----------------------------------------------------------*/
void BatteryModel_Step(BatteryModel *pModel, float currentA)
{
    float deltaSocPct;
    float r0_mOhm;
    float r1_mOhm;
    float targetRcV;
    float alpha;

    if (pModel == 0)
    {
        return;
    }

    pModel->currentA = currentA;

    /* 1. SOC 적산 */
    if (pModel->capacityAs > 0.0f)
    {
        deltaSocPct = (currentA * BATTERY_MODEL_DT_SEC / pModel->capacityAs) * 100.0f;
        pModel->socPct += deltaSocPct;

        pModel->socPct = BatteryModel_Clamp(pModel->socPct,
                                            BATTERY_SOC_MIN_PCT,
                                            BATTERY_SOC_MAX_PCT);
    }

    /* 2. OCV 계산 */
    pModel->ocvV = BatteryModel_Lookup(pModel->pOcvTable,
                                       pModel->ocvTableSize,
                                       pModel->socPct);

    /* 3. R0, R1 계산 */
    if (currentA > 0.0f)
    {
        r0_mOhm = BatteryModel_Lookup(pModel->pChargeR0Table,
                                      pModel->chargeR0TableSize,
                                      pModel->socPct);

        r1_mOhm = BatteryModel_Lookup(pModel->pChargeR1Table,
                                      pModel->chargeR1TableSize,
                                      pModel->socPct);
    }
    else
    {
        r0_mOhm = BatteryModel_Lookup(pModel->pDischargeR0Table,
                                      pModel->dischargeR0TableSize,
                                      pModel->socPct);

        r1_mOhm = BatteryModel_Lookup(pModel->pDischargeR1Table,
                                      pModel->dischargeR1TableSize,
                                      pModel->socPct);
    }

    pModel->r0Ohm = r0_mOhm * 0.001f;
    pModel->r1Ohm = r1_mOhm * 0.001f;

    /* 4. IR 성분 */
    if (BatteryModel_Abs(currentA) <= BATTERY_CURRENT_DEADBAND_A)
    {
        pModel->irV = 0.0f;
    }
    else
    {
        pModel->irV = currentA * pModel->r0Ohm;
    }

    /* 5. RC polarization */
    if ((BatteryModel_Abs(currentA) <= BATTERY_CURRENT_DEADBAND_A) ||
        (pModel->r1Ohm <= 0.0f) ||
        (pModel->c1F <= 0.0f))
    {
        /* 전류가 거의 없으면 RC 성분이 서서히 0으로 감쇠 */
        if ((pModel->r1Ohm > 0.0f) && (pModel->c1F > 0.0f))
        {
            pModel->tauSec = pModel->r1Ohm * pModel->c1F;
            alpha = BATTERY_MODEL_DT_SEC / pModel->tauSec;
            pModel->rcV += alpha * (0.0f - pModel->rcV);
        }
        else
        {
            pModel->tauSec = 0.0f;
            pModel->rcV = 0.0f;
        }
    }
    else
    {
        pModel->tauSec = pModel->r1Ohm * pModel->c1F;
        targetRcV = currentA * pModel->r1Ohm;
        alpha = BATTERY_MODEL_DT_SEC / pModel->tauSec;
        pModel->rcV += alpha * (targetRcV - pModel->rcV);
    }

    /* 6. 최종 단자전압 */
    pModel->tvV = pModel->ocvV + pModel->irV + pModel->rcV;

    pModel->tvV = BatteryModel_Clamp(pModel->tvV,
                                     BATTERY_CELL_MIN_V,
                                     BATTERY_CELL_MAX_V);

    pModel->outV = pModel->tvV;
}

/*-----------------------------------------------------------
 사용자 초기화
-----------------------------------------------------------*/
void BatteryModel_UserInit(void)
{
    BatteryModel_Init(&gBatteryModel,
                      gBattery_InitSocPct,
                      gBattery_CapacityAh,
                      gBattery_C1_F,
                      OcvTable,
                      OCV_TABLE_SIZE,
                      R0Table,
                      R0_TABLE_SIZE,
                      R0Table,
                      R0_TABLE_SIZE,
                      R1Table,
                      R1_TABLE_SIZE,
                      R1Table,
                      R1_TABLE_SIZE);

    gTrace_SOC  = gBatteryModel.socPct;
    gTrace_OCV  = gBatteryModel.ocvV;
    gTrace_R0   = gBatteryModel.r0Ohm;
    gTrace_R1   = gBatteryModel.r1Ohm;
    gTrace_RC   = gBatteryModel.rcV;
    gTrace_IR   = gBatteryModel.irV;
    gTrace_TV   = gBatteryModel.tvV;
    gTrace_OutV = gBatteryModel.outV;
}

/*-----------------------------------------------------------
 1ms Task
-----------------------------------------------------------*/
void BatteryModel_1msTask(void)
{
    BatteryModel_Step(&gBatteryModel, gBattery_InputCurrentA);

    gTrace_SOC  = gBatteryModel.socPct;
    gTrace_OCV  = gBatteryModel.ocvV;
    gTrace_R0   = gBatteryModel.r0Ohm;
    gTrace_R1   = gBatteryModel.r1Ohm;
    gTrace_RC   = gBatteryModel.rcV;
    gTrace_IR   = gBatteryModel.irV;
    gTrace_TV   = gBatteryModel.tvV;
    gTrace_OutV = gBatteryModel.outV;
}

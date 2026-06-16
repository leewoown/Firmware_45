#include "DSP28x_Project.h"
#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "BATCalc.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

#if ShipPack_168S1P
extern void BatCalcRegsInit(BatCalcReg *P);
extern void BatCalcVoltHandle(BatCalcReg *P);
extern void BatCalcTempsHandle(BatCalcReg *P);
extern void BatCalcIRHandle(BatCalcReg *P);
extern float RandVaule(float inputVaule);
float RandVaule(float inputVaule)
{
    static Uint32 seed = 1234567UL;
    float randNorm;
    float randOffset;

    /* LCG Random Generator */
    seed = (Uint32)(1664525UL * seed + 1013904223UL);

    /* 0 ~ 1 */
    randNorm = (float)(seed & 0xFFFFU) / 65535.0f;

    /* -0.4 ~ +0.5 */
    randOffset = -0.4f + (randNorm * 0.9f);

    /* 입력 전류 + 랜덤 */
    return (inputVaule + randOffset);
}
void BatCalcRegsInit(BatCalcReg *P)
{
    memset(&P->MDCellMaxVolt[0],0,7);
    memset(&P->MDCellMinVolt[0],0,7);
    memset(&P->MDCellAgvVolt[0],0,7);
    memset(&P->MDCellDivVolt[0],0,7);

    memset(&P->MDCellMaxTemps[0],0.0,7);
    memset(&P->MDCellMinTemps[0],0.0,7);
    memset(&P->MDCellAgvTemps[0],0.0,7);
    memset(&P->MDCellDivTemps[0],0.0,7);

    memset(&P->MDCellMaxVoltF[0],0.0,7);
    memset(&P->MDCellMinVoltF[0],0.0,7);
    memset(&P->MDCellAgvVoltF[0],0.0,7);
    memset(&P->MDCellDivVoltF[0],0.0,7);

    memset(&P->MDCellMaxTempsF[0],0.0,7);
    memset(&P->MDCellMinTempsF[0],0.0,7);   // TODOS : [완료] (65, RegsInit 복붙오류 수정: Min/Agv/Div TempsF 초기화 누락 복구)
    memset(&P->MDCellAgvTempsF[0],0.0,7);
    memset(&P->MDCellDivTempsF[0],0.0,7);


    memset(&P->MDTotalVolt[0],0,7);
    memset(&P->MDTotalVoltF[0],0.0,7);

    memset(&P->MDMaxVoltPo[0],0.0,7);
    memset(&P->MDMinVoltPo[0],0.0,7);
    memset(&P->MDMaxTempsPo[0],0.0,7);
    memset(&P->MDMinTempsPo[0],0.0,7);


    /*
     *
     */
    P->PackPTADC=0;
    P->PackPTCAN=0;
    P->PackPTADCF=0;
    P->PackPTCANF=0;
    /*
     *
     */
    P->PackCTADC=0;
    P->PackCTCAN=0;
    P->PackCTAbs=0;
    P->PackCTAgv=0;
    P->PackCTADCF=0.0;
    P->PackCTCANF=0.0;
    P->PackCTAbsF=0.0;
    P->PackCTAgvF=0.0;
    /*
     *
     */
;
    P->PackCellMaxVoltF=0.0;
    P->PackCellMinVoltF=0.0;
    P->PackCellDivVoltF=0.0;
    P->PackCellAgvVoltF=0.0;

    P->PackCellMaxTempsF=0.0;
    P->PackCellMinTempsF=0.0;
    P->PackCellAgvTempsF=0.0;
    P->PackCellDivTempsF=0.0;


    P->PackCellMaxVoltPos=0;
    P->PackCellMinVoltPos=0;
    P->PackCellMaxTempsPos=0;
    P->PackCellMinTempsPos=0;
}
void BatCalcVoltHandle(BatCalcReg *P)
{
    Uint16  Count;
    Uint16  const MoudleEa =7;
    Uint16  const PackCellEa =24*7;
    float32 PackVoltageBufF=0;
    /*
     * 정수를 소수점 변환하는 루틴
     */
    for(Count=0; Count<MoudleEa; Count++)
    {
        P->MDCellMaxVoltF[Count] =(float32) P->MDCellMaxVolt[Count]  *0.001f;
        P->MDCellMinVoltF[Count] =(float32) P->MDCellMinVolt[Count]  *0.001f;
        P->MDCellAgvVoltF[Count] =(float32) P->MDCellAgvVolt[Count]  *0.001f;
        P->MDCellDivVoltF[Count] =(float32) P->MDCellDivVolt[Count]  *0.001f;    // TODOS : [검증] (63, 셀전압 편차 자기참조 버그 수정: MDCellDivVoltF→MDCellDivVolt 원본 정수 변환)
        P->MDTotalVoltF[Count]   =(float32) P->MDTotalVolt[Count]    *0.01f;
    }
    for(Count=0; Count<MoudleEa; Count++)
    {
        PackVoltageBufF = PackVoltageBufF+P->MDTotalVoltF[Count];
    }
    /*
     *
     */
    float32 CellMaxVoltF=-1.0;
    float32 CellMinVoltF= 5.0;
    Uint16 MDCellMaxVoltPos =0;
    Uint16 MDCellMinVoltPos =0;

    for(Count=0; Count<MoudleEa; Count++)
    {
        const float32 Vmax = P->MDCellMaxVoltF[Count];
        const float32 Vmin = P->MDCellMinVoltF[Count];
        if(Vmax > CellMaxVoltF )
        {
            CellMaxVoltF=Vmax;
            MDCellMaxVoltPos = Count+1;
        }
        if(Vmin < CellMinVoltF)
        {
            CellMinVoltF=Vmin;
            MDCellMinVoltPos = Count+1;
        }

    }


    P->PackPTCANF        = PackVoltageBufF;
    P->PackCellMaxVoltF  = CellMaxVoltF;
    P->PackCellMinVoltF  = CellMinVoltF;
    P->PackCellAgvVoltF  = P->PackPTCANF/(float32)PackCellEa;
    P->PackCellDivVoltF  = CellMaxVoltF-CellMinVoltF;
    P->PackCellMaxVoltPos =  (MDCellMaxVoltPos*24)+P->MDMaxVoltPo[MDCellMaxVoltPos-1];   // TODOS : [검증] (64, 셀최대전압 위치 인덱스 오용 수정: MDCellMinVoltPos→MDCellMaxVoltPos / base 통일은 보류)
    P->PackCellMinVoltPos =  (MDCellMinVoltPos*24)+P->MDMinVoltPo[MDCellMinVoltPos-1];

}
void BatCalcTempsHandle(BatCalcReg *P)
{
    Uint16 Count = 0u;

    /* 모듈 개수 / 모듈당 셀 개수 */
    Uint16 const MoudleEa = 7u;
    Uint16 const CellPerModuleEA = 24u;

    /* 최대/최소 온도 (원값 / 보정값) */
    float32 CellMaxTempsF = -100.0f;
    float32 CellMinTempsF = 200.0f;
    float32 CellMaxTempsAdF = 0.0f;
    float32 CellMinTempsAdF = 0.0f;

    /* 모듈 위치 */
    Uint16 MDCellMaxTempsPos = 0u;
    Uint16 MDCellMinTempsPos = 0u;

    /* 모듈 내 셀 위치 */
    Uint16 ModuleMaxCellPos = 0u;
    Uint16 ModuleMinCellPos = 0u;

    /* 팩 기준 셀 Index */
    Uint16 PackCellMaxIndex = 0u;
    Uint16 PackCellMinIndex = 0u;

    /* 포인터 NULL 방지 */
    if(P == 0)
    {
        return;
    }

    /* =========================================================
     * 1. 정수 → 실수 변환 (0.1 스케일)
     * ========================================================= */
    for(Count = 0u; Count < MoudleEa; Count++)
    {
        P->MDCellMaxTempsF[Count] = (float32)P->MDCellMaxTemps[Count] * 0.1f;
        P->MDCellMinTempsF[Count] = (float32)P->MDCellMinTemps[Count] * 0.1f;
    }

    /* =========================================================
     * 2. 전체 모듈 중 최대 / 최소 온도 탐색
     *    - 초기값(-100 / 200) 기준으로 전체 모듈 비교
     *    - 0번 값 의존 방식 사용하지 않음
     * ========================================================= */
    for(Count = 0u; Count < MoudleEa; Count++)
    {
        /* 최대 온도 탐색 */
        if(CellMaxTempsF <= P->MDCellMaxTempsF[Count])
        {
            CellMaxTempsF = P->MDCellMaxTempsF[Count];
            MDCellMaxTempsPos = Count;
        }

        /* 최소 온도 탐색 */
        if(CellMinTempsF >= P->MDCellMinTempsF[Count])
        {
            CellMinTempsF = P->MDCellMinTempsF[Count];
            MDCellMinTempsPos = Count;
        }
    }

    /* =========================================================
     * 3. 온도 보정 (센서 오차 보정용)
     *   - BETWEEN: (x >= lo) && (x < hi)
     *   - 구간별 오프셋 적용
     *   - 범위 외 값은 원값 유지
     * ========================================================= */

    /* -------- Max 온도 보정 -------- */
    CellMaxTempsAdF = CellMaxTempsF;

    if(BETWEEN(CellMaxTempsF, 20.0f, 25.0f)){CellMaxTempsAdF = CellMaxTempsF - 4.2f;}
    if(BETWEEN(CellMaxTempsF, 25.0f, 30.0f)){CellMaxTempsAdF = CellMaxTempsF - 4.7f;}
    if(BETWEEN(CellMaxTempsF, 30.0f, 35.0f)){CellMaxTempsAdF = CellMaxTempsF - 3.8f;}
    if(BETWEEN(CellMaxTempsF, 35.0f, 40.0f)){CellMaxTempsAdF = CellMaxTempsF - 4.7f;}
    if(BETWEEN(CellMaxTempsF, 40.0f, 45.0f)){CellMaxTempsAdF = CellMaxTempsF - 4.7f;}
    if(BETWEEN(CellMaxTempsF, 45.0f, 50.0f)){CellMaxTempsAdF = CellMaxTempsF - 4.7f;}
    if(BETWEEN(CellMaxTempsF, 50.0f, 55.0f)){CellMaxTempsAdF = CellMaxTempsF - 4.8f;}
    if(BETWEEN(CellMaxTempsF, 55.0f, 60.0f)){CellMaxTempsAdF = CellMaxTempsF - 6.0f;}
    if(BETWEEN(CellMaxTempsF, 60.0f, 65.0f)){CellMaxTempsAdF = CellMaxTempsF - 6.0f;}

    /* -------- Min 온도 보정 -------- */
    CellMinTempsAdF = CellMinTempsF;

    if(BETWEEN(CellMinTempsF, 20.0f, 25.0f)){CellMinTempsAdF = CellMinTempsF - 4.2f;}
    if(BETWEEN(CellMinTempsF, 25.0f, 30.0f)){CellMinTempsAdF = CellMinTempsF - 4.7f;}
    if(BETWEEN(CellMinTempsF, 30.0f, 35.0f)){CellMinTempsAdF = CellMinTempsF - 3.8f;}
    if(BETWEEN(CellMinTempsF, 35.0f, 40.0f)){CellMinTempsAdF = CellMinTempsF - 4.7f;}
    if(BETWEEN(CellMinTempsF, 40.0f, 45.0f)){CellMinTempsAdF = CellMinTempsF - 4.7f;}
    if(BETWEEN(CellMinTempsF, 45.0f, 50.0f)){CellMinTempsAdF = CellMinTempsF - 4.7f;}
    if(BETWEEN(CellMinTempsF, 50.0f, 55.0f)){CellMinTempsAdF = CellMinTempsF - 4.8f;}
    if(BETWEEN(CellMinTempsF, 55.0f, 60.0f)){CellMinTempsAdF = CellMinTempsF - 6.0f;}
    if(BETWEEN(CellMinTempsF, 60.0f, 65.0f)){CellMinTempsAdF = CellMinTempsF - 6.0f;}

    /* =========================================================
     * 4. 셀 위치 계산 (모듈 + 셀 위치 → Pack Index)
     * ========================================================= */
    ModuleMaxCellPos = P->MDMaxTempsPo[MDCellMaxTempsPos];
    ModuleMinCellPos = P->MDMinTempsPo[MDCellMinTempsPos];

    /* 범위 보호 */
    if(ModuleMaxCellPos >= CellPerModuleEA)
    {
        ModuleMaxCellPos = 0u;
    }

    if(ModuleMinCellPos >= CellPerModuleEA)
    {
        ModuleMinCellPos = 0u;
    }

    /* Pack 기준 Index 변환 */
    PackCellMaxIndex = (MDCellMaxTempsPos * CellPerModuleEA) + ModuleMaxCellPos;
    PackCellMinIndex = (MDCellMinTempsPos * CellPerModuleEA) + ModuleMinCellPos;

    /* =========================================================
     * 5. 최종 결과 저장 (보정값 기준)
     * ========================================================= */
    P->PackCellMaxTempsF = CellMaxTempsAdF;
    P->PackCellMinTempsF = CellMinTempsAdF;

    /* 평균 온도 */
    P->PackCellAgvTempsF = (CellMaxTempsAdF + CellMinTempsAdF) * 0.5f;

    /* 온도 편차 */
    P->PackCellDivTempsF = CellMaxTempsAdF - CellMinTempsAdF;

    /* 위치 (1-base) */
    P->PackCellMaxTempsPos = PackCellMaxIndex + 1u;
    P->PackCellMinTempsPos = PackCellMinIndex + 1u;
}
void BatCalcIRHandle(BatCalcReg *P)
{
    
}

#endif



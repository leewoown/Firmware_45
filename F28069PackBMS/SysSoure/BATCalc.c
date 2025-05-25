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
    memset(&P->MDCellMaxTempsF[0],0.0,7);
    memset(&P->MDCellMaxTempsF[0],0.0,7);
    memset(&P->MDCellMaxTempsF[0],0.0,7);


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
    Uint16  Count=0;
    Uint16  BreakCountA=0;
    Uint16  BreakCountB=0;
    Uint16  BreakCountC=0;

    float32 CellMaxVoltF=0;
    float32 CellMinVoltF=0;


    //float32 CellMaxTempsF=0;
    //float32 CellMinTempsF=0;

    float32 PackVoltageBufF=0;

    Uint16 MDCellMaxVoltPos =0;
    Uint16 MDCellMinVoltPos =0;
   // Uint16 MDCellMaxTempsPos =0;
    //Uint16 MDCellMinTempsPos =0;


    /*
     * 정수를 소수점 변환하는 루틴
     */
    for(Count=0; Count<ModuleEA; Count++)
    {
        BreakCountA++;
        P->MDCellMaxVoltF[Count]=(float32) P->MDCellMaxVolt[Count]*0.001;
        P->MDCellMinVoltF[Count]=(float32) P->MDCellMinVolt[Count]*0.001;
     //   P->MDCellAgvVoltF[Count]=(float32) P->MDCellAgvVolt[Count]*0.001;
      //  P->MDCellDivVoltF[Count]=(float32) P->MDCellDivVoltF[Count]*0.001;
        P->MDTotalVoltF[Count]=(float32)P->MDTotalVolt[Count]*0.01;
        if(BreakCountA>10) {break;}
    }
    /*
     *
     */

    for(Count=0; Count<ModuleEA; Count++)
    {
        BreakCountB++;

        PackVoltageBufF = PackVoltageBufF+P->MDTotalVoltF[Count];
        if(BreakCountB>10) {break;}
    }
    /*
     *
     */
    CellMaxVoltF  = P->MDCellMaxVoltF[0];
    CellMinVoltF  = P->MDCellMinVolt[0];
    //CellMaxTempsF = P->MDCellMaxTempsF[0];
    //CellMinTempsF = P->MDCellMinTempsF[0];
    for(Count=0; Count<ModuleEA; Count++)
    {
        BreakCountC++;
        if(CellMaxVoltF <=P->MDCellMaxVoltF[Count])
        {
            CellMaxVoltF=P->MDCellMaxVoltF[Count];
            MDCellMaxVoltPos = Count+1;
        }
        if(CellMinVoltF >=P->MDCellMinVolt[Count])
        {
            CellMinVoltF=P->MDCellMinVoltF[Count];
            MDCellMinVoltPos = Count+1;
        }
/*        if(CellMaxTempsF <=P->MDCellMaxTempsF[Count])
        {
            CellMaxTempsF=P->MDCellMaxTempsF[Count];
            MDCellMaxTempsPos = Count+1;
        }
        if(CellMinTempsF >=P->MDCellMinTempsF[Count])
        {
            CellMinTempsF=P->MDCellMinTempsF[Count];
            MDCellMinTempsPos = Count+1;
        }
*/
        if(BreakCountC>10) {break;}
    }

    P->PackPTCANF        = PackVoltageBufF;
    P->PackCellMaxVoltF  = CellMaxVoltF;
    P->PackCellMinVoltF  = CellMinVoltF;
    P->PackCellAgvVoltF  = P->PackPTCANF/PackCellEA;
    P->PackCellDivVoltF  = CellMaxVoltF-CellMinVoltF;
    P->PackCellMaxVoltPos =  (MDCellMaxVoltPos*24)+P->MDMaxVoltPo[MDCellMinVoltPos-1];
    P->PackCellMinVoltPos =  (MDCellMinVoltPos*24)+P->MDMinVoltPo[MDCellMinVoltPos-1];
 //   P->PackCellMaxTempsPos = MDCellMaxTempsPos*24;
 //   P->PackCellMinTempsPos = MDCellMinTempsPos*24;

}
void BatCalcTempsHandle(BatCalcReg *P)
{
    Uint16  Count=0;
    Uint16  BreakCountA=0;
    Uint16  BreakCountB=0;

    float32 CellMaxTempsF=0;
    float32 CellMinTempsF=0;

     Uint16 MDCellMaxTempsPos =0;
     Uint16 MDCellMinTempsPos =0;


    /*
     * 정수를 소수점 변환하는 루틴
     */
    for(Count=0; Count<ModuleEA; Count++)
    {
        BreakCountA++;
        P->MDCellMaxTempsF[Count]=(float32) P->MDCellMaxTemps[Count]*0.1;
        P->MDCellMinTempsF[Count]=(float32) P->MDCellMinTemps[Count]*0.1;
        if(BreakCountA>10) {break;}
    }

    CellMaxTempsF  = P->MDCellMaxTempsF[0];
    CellMinTempsF  = P->MDCellMinTempsF[0];

    for(Count=0; Count<ModuleEA; Count++)
    {
        BreakCountB++;
        if(CellMaxTempsF <=P->MDCellMaxTempsF[Count])
        {
            CellMaxTempsF=P->MDCellMaxTempsF[Count];
            MDCellMaxTempsPos = Count+1;
        }
        if(CellMinTempsF >=P->MDCellMinTempsF[Count])
        {
            CellMinTempsF=P->MDCellMinTempsF[Count];
            MDCellMinTempsPos = Count+1;
        }
        if(BreakCountB>10) {break;}
    }

    P->PackCellMaxTempsF  = CellMaxTempsF;
    P->PackCellMinTempsF  = CellMinTempsF;
    P->PackCellAgvTempsF  = (CellMaxTempsF+CellMinTempsF)*0.5;
    P->PackCellDivTempsF  = CellMaxTempsF-CellMinTempsF;

    P->PackCellMaxTempsPos =  (MDCellMaxTempsPos*24)+P->MDMaxTempsPo[MDCellMaxTempsPos];
    P->PackCellMinTempsPos =  (MDCellMinTempsPos*24)+P->MDMinTempsPo[MDCellMinTempsPos];
}
#endif



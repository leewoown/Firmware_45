


#include "DSP28x_Project.h"
#include "parameter.h"
#include "ProtectRelay.h"
#include <stdio.h>
#include <math.h>
#include <string.h>


extern void ProtectRlySateCheck(PrtectRelayReg *P);
extern void ProtectRlyVarINIT(PrtectRelayReg *P);
extern void ProtectRlyOnInit(PrtectRelayReg *P);
extern void ProtectRlyOnHandle(PrtectRelayReg *P);
extern void ProtectRlyOffInit(PrtectRelayReg *P);
extern void ProtectRlyOffHandle(PrtectRelayReg *P);
extern void ProtectRlyEMSHandle(PrtectRelayReg *P);

void ProtectRlyVarINIT(PrtectRelayReg *P)
{
    P->State.all=0;
    P->WakeupOn_ProRlyOnCount=0;
    P->WakeupOn_ProRlyOffCount=0;
    P->WakeupOn_PRlyOnCount=0;
    P->WakeupOn_NRlyOnCount=0;
    P->WakeupOff_PRlyOffCount=0;
    P->WakeupOff_NRlyOffCount=0;
    P->Protect_ProRlyOnCount=0;
    P->Protect_ProRlyOffCount=0;
    P->Protect_PRlyOffCount=0;
    P->Protect_NRlyOffCount=0;
    P->WakeupOn_TimeCount=0;
    P->WakeupOff_TimeCount=0;
    P->StateMachine=STATERly_INIT;
}
void ProtectRelayHandle(PrtectRelayReg *P)
{
    switch(P->StateMachine)
    {
        case STATERly_INIT :
              ProtectRlyVarINIT(P);
              P->StateMachine = STATERly_STANDBY;
        break;
        case STATERly_STANDBY :
             P->State.bit.WakeuPOffEND=0;
             P->State.bit.WakeuPOnEND=0;
        break;
        case STATERly_Ready :
            P->State.bit.WakeuPOffEND=0;
            P->State.bit.WakeuPOnEND=0;

            if((P->State.bit.PRlyDI==1)||(P->State.bit.NRlyDI==1))
            {
                P->State.bit.RlyFaulttSate = 1;
                P->StateMachine = STATERly_PrtectSeq;
            }
            else
            {
                P->State.bit.RlyFaulttSate = 0;
                if(P->State.bit.WakeUpEN==1)
                {
                    P->StateMachine = STATERly_OnSeq;
                }
            }
        break;
        case STATERly_OnSeq :
             P->State.bit.NRlyDO=1;
             delay_ms(100);
             if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==0)&&(P->State.bit.PRlyDI==0))
             {
                 P->State.bit.PreRlyDO=1;
                 P->State.bit.ProRlyDI=1;
                 delay_ms(50);
             }
             if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==1)&&(P->State.bit.PRlyDI==0))
             {
                 P->State.bit.PRlyDO=1;
                 delay_ms(100);
             }
             if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==1)&&(P->State.bit.PRlyDI==1))
             {
                 P->State.bit.PreRlyDO=0;
                 P->State.bit.ProRlyDI=0;
             }
             if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==0)&&(P->State.bit.PRlyDI==1))
             {
                 P->State.bit.WakeuPOffEND=0;
                 P->State.bit.WakeuPOnEND=1;
                 if(P->State.bit.WakeUpEN==0)
                 {
                     P->StateMachine = STATERly_OffSeq;
                 }
             }
             /* 260721 : 닫힘 시퀀스가 시간 내 완료 안 됨 = DI 단선 / 릴레이 미투입.
              *          프리차지가 계속 켜진 채 방치되는 것을 막기 위해 PrtectSeq로 강제 전환. */
             if((P->State.bit.WakeuPOnEND==0) && (P->WakeupOn_TimeCount >= RlyCloseFailTimeOut_ms))
             {
                 P->State.bit.RlyFaulttSate = 1;
                 P->StateMachine = STATERly_PrtectSeq;
             }

        break;
        case STATERly_OffSeq:
            /* 260709 : Open 순서 지정 - 1.Positive 릴레이 먼저, 2.Negative 릴레이 나중 */
            P->State.bit.PRlyDO=0;
            delay_ms(300);
            P->State.bit.NRlyDO=0;
            delay_ms(50);
            if((P->State.bit.NRlyDI==0)&&(P->State.bit.ProRlyDI==0)&&(P->State.bit.PRlyDI==0))
            {
                P->State.bit.WakeuPOffEND=1;
                P->State.bit.WakeuPOnEND=0;
                if(P->State.bit.WakeUpEN==0)
                 {
                     P->StateMachine = STATERly_Ready;
                 }
            }
            /* 260721 : 열림 시퀀스가 시간 내 완료 안 됨 = 접점 융착 의심. */
            if((P->State.bit.WakeuPOffEND==0) && (P->WakeupOff_TimeCount >= RlyOpenWeldTimeOut_ms))
            {
                P->State.bit.RlyFaulttSate = 1;
                P->StateMachine = STATERly_PrtectSeq;
            }
        break;

        case STATERly_PrtectSeq:
       //      P->State.bit.PreRlyDO=1;
       //      P->State.bit.ProRlyDI=1;
       //     delay_ms(100);
             P->State.bit.PRlyDO=0;
             delay_ms(50);
             P->State.bit.NRlyDO=0;
             P->State.bit.PreRlyDO=0;
             P->State.bit.ProRlyDI=0;
        break;
        default :
        break;

    }
}

void ProtectRlyOnInit(PrtectRelayReg *P)
{
    P->WakeupOn_ProRlyOnCount=0;
    P->WakeupOn_PRlyOnCount=0;
    P->WakeupOn_ProRlyOffCount=0;
    P->State.bit.WakeuPOffEND=0;
   // P->State.bit.WakeuPOnEND=0;
    P->WakeupOff_TimeCount=0;
}
void ProtectRlyOffInit(PrtectRelayReg *P)
{
    P->WakeupOff_NRlyOffCount=0;
    P->State.bit.WakeuPOnEND=0;
    P->WakeupOn_TimeCount=0;
    P->WakeupOff_TimeCount=0;
}






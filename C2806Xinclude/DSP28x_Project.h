
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

//EDLAY �뜝�럩�뀋�뜝�럡�땽�뜝�럥裕쏉옙�쐻占쎈윥占쎄샵�뜝�럥�맶�뜝�럥�쐾占쎄슈占쎌뿪占쎌굲�뜝�럥�듋�뜝�럥�뇢�뜝�럥�깫�뜝�럥�룚�뜝�럥�맶�뜝�럥�쑅�뜝�럥援겼뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪占쎈쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썲뜝�럥�맶�뜝�럥�쑅鶯ㅼ렮堉뱄옙�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쐾�뜝�럥�룄 --------------------------------------------------------------------//

// TI SDK 1.10�뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�꺐占썩벀�걠占쎄뎡 �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀯옙�쐻占쎈윥占쏙옙癲ル슢罹숂땟戮녹삕筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맚嶺뚮Ĳ猷귨옙援� DSP2803x_usDelay.asm�뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쑅占쎈쨨野껋눖�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썲뜝�럥�맶�뜝�럥�쑅鶯ㅼ룊�삕 �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑋�댖怨뺣꺏占쎌굲�뜝�럩留띰옙�쐻占쎈윥占쎈첋癲딅냲�삕占쎈쐻占쎈쑟�얜퀬占썩뫀�떝占쎈쵆�쐻占쎈윥�뜝�뜴�쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈ぇ占쎈쐻占쎈윪筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맚嶺뚮Ĳ猷귨옙援� DELAY_US �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀧솾�꺂�뒧占쎈뎔�뜝�럥�뼔占쎈쐻占쎈윪筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쑅占쎈닱熬곎딆굲�뜝�럩留띰옙�쐻占쎈윥占쎌몗占쎈쐻占쎈윞�꽴�뙋�삕占쎌맶�뜝�럥�쑅�뜝�럥�럪占쎈쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썲뜝�럥�맶�뜝�럥�쑅占쎈쐻占쎈뼩占쎌뵰占쎄뎡占쎈쐻占쎈윪筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑅嶺뚋우삕

// TI SDK 1.10�뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�꺐占썩벀�걠占쎄뎡 �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀯옙�쐻占쎈윥占쏙옙癲ル슢罹숂땟戮녹삕筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맚嶺뚮Ĳ猷귨옙援� DSP2803x_usDelay.asm�뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쑅占쎈쨨野껋눖�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썲뜝�럥�맶�뜝�럥�쑅鶯ㅼ룊�삕 �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑋�댖怨뺣꺏占쎌굲�뜝�럩留띰옙�쐻占쎈윥占쎈첋癲딅냲�삕占쎈쐻占쎈쑟�얜퀬占썩뫀�떝占쎈쵆�쐻占쎈윥�뜝�뜴�쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈ぇ占쎈쐻占쎈윪筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맚嶺뚮Ĳ猷귨옙援� DELAY_US �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀧솾�꺂�뒧占쎈뎔�뜝�럥�뼔占쎈쐻占쎈윪筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쑅占쎈닱熬곎딆굲�뜝�럩留띰옙�쐻占쎈윥占쎌몗占쎈쐻占쎈윞�꽴�뙋�삕占쎌맶�뜝�럥�쑅�뜝�럥�럪占쎈쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썲뜝�럥�맶�뜝�럥�쑅占쎈쐻占쎈뼩占쎌뵰占쎄뎡占쎈쐻占쎈윪筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑅嶺뚋우삕
#define delay_us(us)        DELAY_US(us)
// TI SDK 1.10�뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�꺐占썩벀�걠占쎄뎡 �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀯옙�쐻占쎈윥占쏙옙癲ル슢罹숂땟戮녹삕筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맚嶺뚮Ĳ猷귨옙援� DSP2803x_usDelay.asm�뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쑅占쎈쨨野껋눖�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썲뜝�럥�맶�뜝�럥�쑅鶯ㅼ룊�삕 �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑋�댖怨뺣꺏占쎌굲�뜝�럩留띰옙�쐻占쎈윥占쎈첋癲딅냲�삕占쎈쐻占쎈쑟�얜퀬占썩뫀�떝占쎈쵆�쐻占쎈윥�뜝�뜴�쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈ぇ占쎈쐻占쎈윪筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맚嶺뚮Ĳ猷귨옙援� DELAY_US �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀧솾�꺂�뒧占쎈뎔�뜝�럥�뼔占쎈쐻占쎈윪筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쑅占쎈닱熬곎딆굲�뜝�럩留띰옙�쐻占쎈윥占쎌몗占쎈쐻占쎈윞�꽴�뙋�삕占쎌맶�뜝�럥�쑅�뜝�럥�럪占쎈쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썲뜝�럥�맶�뜝�럥�쑅占쎈쐻占쎈뼩占쎌뵰占쎄뎡占쎈쐻占쎈윪筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑅嶺뚋우삕
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

/* Bit Mask Data �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑋�뜝�럩二㎩뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�꺐占썩벀�걠占쎄뎡 */
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


#define     Shift_RIGHT(val, bit)            ((val) >> (al))
#define     Shift_LEFT(val,  bit)            ((val) << (val))
#define     ComBine(Val_H, Val_L)            (((Val_H) << 8) |(Val_L))
#define     BETWEEN(x, lo, hi)               ((x) >= (lo) && (x) < (hi))
#define     BIT_MASK(bit)                    (1 << (bit))
#define     GetBit(val, bit)                 (((val) & BIT_MASK(bit)) >> (bit))
#define     SetBit(val, bit)                 (val |= BIT_MASK(bit))
#define     ClearBit(val, bit)               (val &= ~BIT_MASK(bit))
#define     ToggleBit(val, bit)              (val ^= BIT_MASK(bit))
#define     bit_is_set(val, bit)             (val & BIT_MASK(bit))
#define     bit_is_clear(val, bit)           (~val & BIT_MASK(bit))
#define     Hyst_On(Value, SetValue)         ((Value) >= (SetValue))
#define     Hyst_Off(Value, RstValue)        ((Value) <= (RstValue))
#define     IS_OVER_AND_UNDER(A, MIN, MAX)   ((A) >= (MIN) && (A) <= (MAX))
#define     IS_ABOVE_AND_UNDER(A, MIN, MAX)  ((A) >  (MIN) && (A) <= (MAX))
#define     IS_OVER_AND_BELOW(A, MIN, MAX)   ((A) >= (MIN) && (A) <  (MAX))
#define     IS_ABOVE_AND_BELOW(A, MIN, MAX)  ((A) >  (MIN) && (A) <  (MAX))

#define     CAN_ALIGN_1BYTE(id)              ((Uint16)((id) & 0x07F0))  // 0xFF(X) : LSB4=0
#define     CAN_ALIGN_2BYTE(id)              ((Uint16)((id) & 0x070F))  // 0xF(X)F : [7:4]=0

// ===== LAM 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁���삕占쎈퉳�뜝�럥堉붷ㅇ�슪�삕占쎌뇢�뜝�럥�뒌嶺뚮씭�뵛占쎌굲�뜝�럩留띰옙�쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�㏃녇占쎄틓占쎈뮝占쎈쑏筌믩끃�굲�뜝�럥�맶�뜝�럥�쑋嶺뚮씭�뵛占쎌굲�뜝�럩留띰옙�쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁빢�삕占쎌맶�뜝�럥�쑅勇싲즾維낉옙�굲�뜝�럩留뜹뜝�럥�맶占쎈쐻�뜝占�(1=don't-care) : LAM_H占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎌몞占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝癲ル슢履뉐뜝�떥�궡留띰옙�쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁���삕占쎌맶�뜝�럥�쑅勇싲즾維낉옙�굲�뜝�럩留뜹뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁빆�쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎌쟽占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎌뱺占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗癲ル슣鍮뽳옙堉ｏ옙鍮듿뜝�룞�삕占쎌맶�뜝�럥�몷�뜝�럩占쏙옙�쐻占쎈윥占쎄덱�뚣뀿�쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪占쎄섀占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝�뜝�럥夷ⓨ뜝�럥흮占쎄뎡占쎈쐻占쎈윪筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐앯솾�꺂�뒧占쎈뎐�뜝�럥�꺙占쎈쐻占쎈윪�뤃�먯삕占쎌맶�뜝�럥�쑋嶺뚮씭�뵛占쎌굲�뜝�럩留띰옙�쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윞�꽴硫⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗�뜝�럥夷③뇦猿뗫닑占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎌몞�뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쑅�뜝�럥�쑌占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫�뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪占쎈쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎌몞占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝癲ル슢�뵰占쎌몧�뜝�럩留띰옙�쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�κ텣�뜝�룞�삕占쎄콬�뜝�럩議롳옙逾볠�⑤슢�삺�뜝�럩援뀐옙�쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎌몞占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝癲ル슢履뉔뜮�맮�삕占쎈�뗰옙�쐻占쎈윞占쎈빟占쎈쐻占쎈윥獒뺤룊�삕占쎌맶�뜝�럥�쑅占쎈쐻占쎈윥占쎈룦�뜝�럩�읁�뜝�럥�맶�뜝�럥�뱻�뜝�럥占썲뜝�럥�맶�뜝�럥�뱻�뜝�럩紐∽옙�쐻占쎈윥�댆�뜴�쐻占쎈윪筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럥�걞�뜝�럥�맶�뜝�럥�쑅�뜏類ｋ염占쎌굲�뜝�럩留띰옙�쐻占쎈윥占쎌몗占쎈쐻占쎈윞占쏙옙�솾袁㏉�쀯옙維믣뜝�럥�뼔占쎈쐻占쎈윪筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩留썲뜝�럥�맶�뜝�럥�쑅�뜝�럩�읇 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎌몞占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝癲ル슢履뉐뜝�떥釉붾���뜝�럥�맶�뜝�럥吏쀥뜝�럩援뀐옙�쐻占쎈윞�굜�껊쐻占쎈윥占쏙옙占쎈쐻占쎈윥占쎄묽占쎈쐻占쎈윪�뤃�먯삕占쎌맶�뜝�럥�쐾�뜝�럥�젃 =====
#define     CAN_LAM_1BYTE                    ((Uint16)0x003C)   // LSB4 don占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎌몞占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝癲ル슢履뉔뜮�냵�삕占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗癲ル슣�돵�뜝�떥�궡留띰옙�쐻占쎈윥占쎌몝�뜝�럥夷⑨옙�쐻�뜝占�-care 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎌몞占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝癲ル슢履뉐뜝�룞�삕�뜝�럩援뀐옙�쐻占쎈윪筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쐾占쎄슈�눧類ㅼ굲�뜝�럩留띰옙�쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫�뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪占쎈쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 0x..0~0x..F
#define     CAN_LAM_2BYTE                    ((Uint16)0x03C0)   // [7:4] don占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎌몞占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝癲ル슢履뉔뜮�냵�삕占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗癲ル슣�돵�뜝�떥�궡留띰옙�쐻占쎈윥占쎌몝�뜝�럥夷⑨옙�쐻�뜝占�-care 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎌몞占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윪筌륁�λ쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝癲ル슢履뉐뜝�룞�삕�뜝�럩援뀐옙�쐻占쎈윪筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쐾占쎄슈�눧類ㅼ굲�뜝�럩留띰옙�쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫�뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪占쎈쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 0xF(X)F


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
   unsigned int     DipSW00                 :1;   // 0
   unsigned int     DipSW01                 :1;   // 1
   unsigned int     DipSW02                 :1;   // 2
   unsigned int     DipSW03                 :1;   // 3
   unsigned int     PAUX                    :1;   // 4
   unsigned int     NAUX                    :1;   // 5
   unsigned int     EMGSWStauts             :1;   // 6
   unsigned int     CANRX0                  :1;   // 7
   unsigned int     CANRX1                  :1;   // 8
   unsigned int     SW09                    :1;   // 9
   unsigned int     SW10                    :1;   // 10
   unsigned int     SW11                    :1;   // 11
   unsigned int     SW12                    :1;   // 12
   unsigned int     SW13                    :1;   // 13
   unsigned int     SW14                    :1;   // 14
   unsigned int     SW15                    :1;   // 15

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
   System_STATE_CLEAR,
   System_STATE_RESET
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
    unsigned int     Fault                      :1;    // 1
    unsigned int     BalanceStartStop           :1;    // 2
    unsigned int     WaterleakFault             :1;    // 3, FAN Delection
    unsigned int     CellVoltageFault           :1;    // 4
    unsigned int     CellTemperatureFault       :1;    // 5
    unsigned int     BATICErrFault             :1;    // 6
    unsigned int     CTCOMErrFault              :1;    // 7
    unsigned int     MBCOMErrFault              :1;    // 8
    unsigned int     HMICOMErrFault             :1;    // 9
    unsigned int     BalanceEnable              :1;    // 10
    unsigned int     CellVoltCAN                :1;    // 11
    unsigned int     CellTempCAN                :1;    // 12
    unsigned int     HmiEn                      :1;    // 13
    unsigned int     PackEn                     :1;    // 14
    unsigned int     NotUsed15                  :1;    // 15
};
union SysMDstatus_REG
{
   unsigned int     all;
   struct SysMDstatus_BIT bit;
};
struct SystemState_BIT
{       // bits   description / 0x602 Sys_* (규약명)
    // TODOS : [완료] (45, 0x602 bit32-63 Sys_* 규약 일치 재배치 (UnitRead/PmsRead 미적용, CANCOMERR/ISOSPICOMERR/IMDRegErr 삭제, BATIC/PmsCAN/UnitCom 신규))
    unsigned int     SysSeqState            :3; // 0-2   Sys_SysSeqState
    unsigned int     RlySeqState            :3; // 3-5   Sys_RlySeqState
    unsigned int     SocSeqState            :2; // 6-7   Sys_SocSeqState
    unsigned int     INITOK                 :1; // 8     Sys_INITOK
    unsigned int     SysBalanceEn           :1; // 9     Sys_BalanceEn
    unsigned int     SysDisCharMode         :1; // 10    Sys_DisCharMode
    unsigned int     SysAalarm              :1; // 11    Sys_Alarm
    unsigned int     SysFault               :1; // 12    Sys_Fault
    unsigned int     SysPrtct               :1; // 13    Sys_Prtct
    unsigned int     CANCOMEnable           :1; // 14    Sys_CanEnable
    unsigned int     NRlyDOStatus           :1; // 15    Sys_NRlyStatus
    unsigned int     PRlyDOStatus           :1; // 16    Sys_PRlyStatus
    unsigned int     PreRlyDOStatus         :1; // 17    Sys_PreRlyStatus
    unsigned int     HMICOMMode             :1; // 18    Sys_AdminMode
    unsigned int     HMIBalanceMode         :1; // 19    Sys_AdminBalaMode
    unsigned int     MSDERR                 :1; // 20    Sys_MSDERR
    unsigned int     RlyERR                 :1; // 21    Sys_RlyERR
    unsigned int     SysPmsCANErr           :1; // 22    Sys_PmsCANErr  (신규)
    unsigned int     SysUnitComErr          :1; // 23    Sys_UnitComErr (신규, 구 CANCOMERR 기능 이전)
    unsigned int     SysBATIC_Err           :1; // 24    Sys_BATIC_Err  (신규, 모듈 BATIC 에러 종합)
    unsigned int     VCURlyWakeUp           :1; // 25    Sys_VCURlyWakeUp
    unsigned int     ChargerWakeUp          :1; // 26    Sys_ChargerWakeUp
    unsigned int     SysSocMode             :1; // 27    Sys_SocMode
    unsigned int     SysBalaMode            :1; // 28    Sys_BalaMode
    unsigned int     PwrHoldRlyDOStatus     :1; // 29    Sys_PwrHoldRly
    unsigned int     CellVoltOk             :1; // 30    Sys_CellVoltOk
    unsigned int     CellTempsOk            :1; // 31    Sys_CellTempsOk
};
union  SystemState_REG
{
   struct SystemState_BIT   bit;
   struct Data_WORD        Word;
   Uint32                   all;
};
struct SystemAlarm_BIT
{       // bits   description
  //  unsigned int     PackVDISCHACT_OV          :1; // 0
  //  unsigned int     PackVCHACT_OV             :1; // 1
    /*--------------------------------------------------------------
     * 260616 : 0x603 Warn(bit0-15) 배치 R57->R58 전면 재편 (Master R5 기준)
     *          - bit1 예비(Not_Warn2) 삽입 / SOC,Volt,Cell,Temp 순서 재배치
     *          - 구 bit13-15 CellIR 경고 삭제(옛코드 주석보존), UnbaPWR 2비트 부활
     *--------------------------------------------------------------*/
    //unsigned int     PackVCur_OC                :1; // 0   BRA_Wn_CT_OV    (R57)
    //unsigned int     PackVSOC_OV                :1; // 1   BRA_Wn_SOC_OV
    //unsigned int     PackVSOC_UN                :1; // 2   BRA_Wn_SOC_UN
    //unsigned int     PackVolt_OV                :1; // 3   BRA_Wn_Volt_OV
    //unsigned int     PackVolt_UN                :1; // 4   BRA_Wn_Volt_UN
    //unsigned int     PackTemp_OT                :1; // 5   BRA_Wn_Temp_OV
    //unsigned int     PackTemp_UT                :1; // 6   BRA_Wn_Temp_UN
    //unsigned int     CellVolt_OV                :1; // 7   Cell_Wn_Volt_OV
    //unsigned int     CellVolt_UN                :1; // 8   Cell_Wn_Volt_UN
    //unsigned int     CellVolt_BL                :1; // 9   Cell_Wn_Volt_DIV
    //unsigned int     CellTemp_OT                :1; // 10  Cell_Wn_Temp_OV
    //unsigned int     CellTemp_UT                :1; // 11  Cell_Wn_Temp_UN
    //unsigned int     CellTemp_BL                :1; // 12  Cell_Wn_Temp_DIV
    //unsigned int     CellIR_OV                  :1; // 13  Cell_Wn_IR_OV   (R58 Warn 삭제)
    //unsigned int     CellIR_UN                  :1; // 14  Cell_Wn_IR_UN   (R58 Warn 삭제)
    //unsigned int     CellIR_DIV                 :1; // 15  Cell_Wn_IR_DIV  (R58 Warn 삭제)
    unsigned int     PackVCur_OC                :1; // 0   BRA1_Wn_OC                    // TODOS : [검증] (72, 0x603 Warn R58 재배치)
    unsigned int     NotWarn1                   :1; // 1   Not_Warn2 (예비)
    unsigned int     PackVSOC_OV                :1; // 2   BRA1_Wn_SOC_OV
    unsigned int     PackVSOC_UN                :1; // 3   BRA1_Wn_SOC_Un
    unsigned int     PackVolt_OV                :1; // 4   BRA1_Wn_OV
    unsigned int     PackVolt_UN                :1; // 5   BRA1_Wn_UV
    unsigned int     CellVolt_OV                :1; // 6   BRA1_Wn_Cell_OV
    unsigned int     CellVolt_UN                :1; // 7   BRA1_Wn_Cell_UV
    unsigned int     CellVolt_BL                :1; // 8   BRA1_Wn_Cell_UnbalV
    unsigned int     PackTemp_OT                :1; // 9   BRA1_Wn_PackOT
    unsigned int     PackTemp_UT                :1; // 10  BRA1_Wn_PackUT
    unsigned int     CellTemp_OT                :1; // 11  BRA1_Wn_CellOT
    unsigned int     CellTemp_UT                :1; // 12  BRA1_Wn_CellUT
    unsigned int     CellTemp_BL                :1; // 13  BRA1_Wn_Cell_UnbalT
    unsigned int     PackUnbaDisCh_UbPWR        :1; // 14  BRA1_Wn_Discharger_UnbaPWR
    unsigned int     PackUnbaCahr_UbPWR         :1; // 15  BRA1_Wn_Charger_UnbaPWR
    unsigned int     SW16                       :1; // 16
    unsigned int     SW17                       :1; // 17
    unsigned int     SW18                       :1; // 18
    unsigned int     SW19                       :1; // 19
    unsigned int     SW20                       :1; // 20
    unsigned int     SW21                       :1; // 21
    unsigned int     SW22                       :1; // 22
    unsigned int     SW23                       :1; // 23
    unsigned int     SW24                       :1; // 24
    unsigned int     SW25                       :1; // 25
    unsigned int     SW26                       :1; // 26
    unsigned int     SW27                       :1; // 27
    unsigned int     SW28                       :1; // 28
    unsigned int     SW29                       :1; // 29
    unsigned int     SW30                       :1; // 30
    unsigned int     SW31                       :1; // 31
};
union SystemAlarm_REG
{
   struct SystemAlarm_BIT   bit;
   struct Data_WORD        Word;
   Uint32                   all;
};
struct SystemFault_BIT
{       // bits   description (0x603 Data1 = R59 SB16~31)
//    unsigned int     PackVDISCHACT_OV          :1; // 0
//    unsigned int     PackVCHACT_OV             :1; // 1
    // TODOS : [완료] (47, 0x603 Fault(bit16-31) 일치 — bit13-15 셀IR Fault 신규, UnbaPWR(구13-14) 삭제)
    /*--------------------------------------------------------------
     * 260615 : 0x603 Fault(SB16~31) R59 정렬 (Warn과 동일 배치)
     *          - bit1 예비(Not_Fault2) 삽입 → SOC/Volt/Cell/Temp 한 칸씩 이동
     *          - bit13~15 CellIR 삭제 → Cell_UnbalT/Discharger·Charger_UnbaPWR 부활
     *--------------------------------------------------------------*/
    //unsigned int     PackVCur_OC                :1; // 0   BRA_FLT_CT_OV    (R57 옛 배치)
    //unsigned int     PackVSOC_OV                :1; // 1
    //unsigned int     PackVSOC_UN                :1; // 2
    //unsigned int     PackVolt_OV                :1; // 3
    //unsigned int     PackVolt_UN                :1; // 4
    //unsigned int     PackTemp_OT                :1; // 5
    //unsigned int     PackTemp_UT                :1; // 6
    //unsigned int     CellVolt_OV                :1; // 7
    //unsigned int     CellVolt_UN                :1; // 8
    //unsigned int     CellVolt_BL                :1; // 9
    //unsigned int     CellTemp_OT                :1; // 10
    //unsigned int     CellTemp_UT                :1; // 11
    //unsigned int     CellTemp_BL                :1; // 12
    //unsigned int     CellIR_OV                  :1; // 13  (R59 삭제)
    //unsigned int     CellIR_UN                  :1; // 14  (R59 삭제)
    //unsigned int     CellIR_DIV                 :1; // 15  (R59 삭제)
    unsigned int     PackVCur_OC                :1; // 0   BRA1_FLT_OC                  // TODOS : [검증] (77, 0x603 Fault R59 정렬)
    unsigned int     NotFault1                  :1; // 1   Not_Fault2 (예비)
    unsigned int     PackVSOC_OV                :1; // 2   BRA1_FLT_SOC_OV
    unsigned int     PackVSOC_UN                :1; // 3   BRA1_FLT_SOC_Un
    unsigned int     PackVolt_OV                :1; // 4   BRA1_FLT_OV
    unsigned int     PackVolt_UN                :1; // 5   BRA1_FLT_UV
    unsigned int     CellVolt_OV                :1; // 6   BRA1_FLT_Cell_OV
    unsigned int     CellVolt_UN                :1; // 7   BRA1_FLT_Cell_UV
    unsigned int     CellVolt_BL                :1; // 8   BRA1_FLT_Cell_UnbalV
    unsigned int     PackTemp_OT                :1; // 9   BRA1_FLT_PackOT
    unsigned int     PackTemp_UT                :1; // 10  BRA1_FLT_PackUT
    unsigned int     CellTemp_OT                :1; // 11  BRA1_FLT_CellOT
    unsigned int     CellTemp_UT                :1; // 12  BRA1_FLT_CellUT
    unsigned int     CellTemp_BL                :1; // 13  BRA1_FLT_Cell_UnbalT
    unsigned int     PackUnbaDisCh_UbPWR        :1; // 14  BRA1_FLT_Discharger_UnbaPWR
    unsigned int     PackUnbaCahr_UbPWR         :1; // 15  BRA1_FLT_Charger_UnbaPWR
    unsigned int     SW16                       :1; // 16
    unsigned int     SW17                       :1; // 17
    unsigned int     SW18                       :1; // 18
    unsigned int     SW19                       :1; // 19
    unsigned int     SW20                       :1; // 20
    unsigned int     SW21                       :1; // 21
    unsigned int     SW22                       :1; // 22
    unsigned int     SW23                       :1; // 23
    unsigned int     SW24                       :1; // 24
    unsigned int     SW25                       :1; // 25
    unsigned int     SW26                       :1; // 26
    unsigned int     SW27                       :1; // 27
    unsigned int     SW28                       :1; // 28
    unsigned int     SW29                       :1; // 29
    unsigned int     SW30                       :1; // 30
    unsigned int     SW31                       :1; // 31
};
union SystemFault_REG
{
   struct SystemFault_BIT  bit;
   struct Data_WORD       Word;
   Uint32                 all;
};
struct SystemProtect_BIT
{       // bits   description
    // TODOS : [완료] (48, 0x603 Protect(bit32-60) 행순서 일치 — 셀IR/BATICCOM/MSD/SysEmgStop/IMD/OffGas 신규)
    /*--------------------------------------------------------------
     * 260615 : 0x603 Protect(SB32~56) R59 정렬
     *          - bit1 예비(Not_Prtct2) 삽입 → SOC/Volt/Cell/Temp 한 칸씩 이동
     *          - bit13~15 CellIR 삭제 → Cell_UnbalT/Discharger·Charger_UnbaPWR(bit13~15) 부활
     *          - 후반 에러 R59순: Rly/IN_COM/EX_COM/CT_COM/Water/EMS/MSD/IMD/OffGas
     *          - R59 미정의 BATICComErr/SysEmgStopErr 삭제
     *--------------------------------------------------------------*/
    //unsigned int     PackVCT_OV                :1; // 0   BRA1_Prtct_OC       (R57 옛 배치)
    //unsigned int     PackVSOC_OV               :1; // 1
    //unsigned int     PackVSOC_UN               :1; // 2
    //unsigned int     PackVolt_OV               :1; // 3
    //unsigned int     PackVolt_UN               :1; // 4
    //unsigned int     PackTemp_OV               :1; // 5
    //unsigned int     PackTemp_UN               :1; // 6
    //unsigned int     CellVolt_OV               :1; // 7
    //unsigned int     CellVolt_UN               :1; // 8
    //unsigned int     CellVolt_BL               :1; // 9
    //unsigned int     CellTemp_OV               :1; // 10
    //unsigned int     CellTemp_UN               :1; // 11
    //unsigned int     CellTemp_BLT              :1; // 12
    //unsigned int     CellIR_OV                 :1; // 13  (R59 삭제)
    //unsigned int     CellIR_UN                 :1; // 14  (R59 삭제)
    //unsigned int     CellIR_DIV                :1; // 15  (R59 삭제)
    //unsigned int     PackUnbaDisCh_UbPWR       :1; // 16  (R59 bit14로 이동)
    //unsigned int     PackUnbaCahr_UbPWR        :1; // 17  (R59 bit15로 이동)
    //unsigned int     PackRlyErr                :1; // 18
    //unsigned int     PackCTComErr              :1; // 19
    //unsigned int     PackExComErr              :1; // 20
    //unsigned int     PackINComErr              :1; // 21
    //unsigned int     PackBATICComErr           :1; // 22  (R59 삭제)
    //unsigned int     PackMSDErr                :1; // 23
    //unsigned int     PackEMSSWErr              :1; // 24
    //unsigned int     PackSysEmgStopErr         :1; // 25  (R59 삭제)
    //unsigned int     PackWaterleakErr          :1; // 26
    //unsigned int     PackIMDErr                :1; // 27
    //unsigned int     PackOffGasErr             :1; // 28
    unsigned int     PackVCT_OV                :1; // 0   BRA1_Prtct_OC                // TODOS : [검증] (78, 0x603 Protect R59 정렬)
    unsigned int     NotPrtct1                 :1; // 1   Not_Prtct2 (예비)
    unsigned int     PackVSOC_OV               :1; // 2   BRA1_Prtct_SOC_OV
    unsigned int     PackVSOC_UN               :1; // 3   BRA1_Prtct_SOC_Un
    unsigned int     PackVolt_OV               :1; // 4   BRA1_Prtct_OV
    unsigned int     PackVolt_UN               :1; // 5   BRA1_Prtct_UV
    unsigned int     CellVolt_OV               :1; // 6   BRA1_Prtct_Cell_OV
    unsigned int     CellVolt_UN               :1; // 7   BRA1_Prtct_Cell_UV
    unsigned int     CellVolt_BL               :1; // 8   BRA1_Prtct_Cell_UnbalV
    unsigned int     PackTemp_OV               :1; // 9   BRA1_Prtct_PackOT
    unsigned int     PackTemp_UN               :1; // 10  BRA1_Prtct_PackUT
    unsigned int     CellTemp_OV               :1; // 11  BRA1_Prtct_CellOT
    unsigned int     CellTemp_UN               :1; // 12  BRA1_Prtct_CellUT
    unsigned int     CellTemp_BLT              :1; // 13  BRA1_Prtct_Cell_UnbalT
    unsigned int     PackUnbaDisCh_UbPWR       :1; // 14  BRA1_Prtct_Discharger_UnbaPWR
    unsigned int     PackUnbaCahr_UbPWR        :1; // 15  BRA1_Prtct_Charger_UnbaPWR
    unsigned int     PackRlyErr                :1; // 16  BRA1_PrtctRly_Err
    unsigned int     PackINComErr              :1; // 17  BRA1_Prtct_IN_COM_Err  (BM>Rack)
    unsigned int     PackExComErr              :1; // 18  BRA1_Prtct_EX_COM_Err  (RACK>SYS BMS)
    unsigned int     PackCTComErr              :1; // 19  BRA1_Prtct_CT_COM_Err
    unsigned int     PackWaterleakErr          :1; // 20  BRA1_Prtct_Water_Leak_Err
    unsigned int     PackEMSSWErr              :1; // 21  BRA1_Prtct_EMS_SW_Err
    unsigned int     PackMSDErr                :1; // 22  BRA1_Prtct_MSD_Err
    unsigned int     PackIMDErr                :1; // 23  BRA1_Prtct_IMD_Err
    unsigned int     PackOffGasErr             :1; // 24  BRA1_Prtct_OffGas_Err
    unsigned int     SW25                      :1; // 25
    unsigned int     SW26                      :1; // 26
    unsigned int     SW27                      :1; // 27
    unsigned int     SW28                      :1; // 28
    unsigned int     SW29                      :1; // 29
    unsigned int     SW30                      :1; // 30
    unsigned int     SW31                      :1; // 31
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
    Uint16  PackID;
    Uint16  PackSWID;
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
    Uint16  ProtectOpenWaitCount;   // 260709 : PROTECTER 진입 후 대기시간(ms), 5초 초과 시 원인 무관 강제 open
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
    // TODOS : [완료] (49, 셀 내부저항 위치 (측정 미구현, 0))
    Uint16  PackCellMaxIRPos;
    Uint16  PackCellMinIRPos;
    // TODOS : [완료] (50, 0x607 셀 내부저항 계산값 (측정 미구현 시 0))
    float32 PackCellMaxIRF;
    float32 PackCellMinIRF;
    float32 PackCellAgvIRF;
    float32 PackCellDivIRF;
  //  float32 SystemBMS

    float32 PackCHAPWRContintyF;
    float32 PackDisCHAPWRContintyF;
    float32 PackCHAPWRPeakF;
    float32 PackDisCHAPWRPeakF;
    float32 PackSOCF;
    float32 PackSOHF;
    float32 PackAhF;
    float32 PackISOResisF;
    float32 socPct_f;
    float32 tempC_f ;
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
/*
 *
 */
    Uint16  SysAlarmCont[16];
    Uint16  SysFalutCont[16];
    Uint16 MD1CANRxCount;
    Uint16 MD2CANRxCount;
    Uint16 MD3CANRxCount;
    Uint16 MD4CANRxCount;
    Uint16 MD5CANRxCount;
    Uint16 MD6CANRxCount;
    Uint16 MD7CANRxCount;
    Uint16 CTRxCount;
    Uint16 MasterRxCount;
    Uint16 MasterCommGraceCount; /* 260720 : 부팅 후 BPU 통신 대기 유예시간(ms) - PackExComErr 오탐 방지 */

    SysState    SysMachine;

    union       ParentDeviceCMD_REG         PMSysCMDResg;

    union       SystemState_REG             PackStateReg;
    union       SystemAlarm_REG             PackAlarmReg;
    union       SystemFault_REG             PackFaultReg;
    union       SystemProtect_REG           PackProtectReg;
    union       SystemFault_REG             PackFaulBuftReg;

    union       Currnet_Reg                 CurrentData;
    union       DigitalInput_REG            DigitalInputReg;
    union       DigitalOutPut_REG           DigitalOutPutReg;


    union       SysMDstatus_REG             PackModuleRegs[7];
    union       SysMDstatus_REG             PackModuleAllRegs;

    union       MDErrStauts_REG             PackInMDCANrxReg;
    union       MDErrStauts_REG             MDInitok;
    union       MDErrStauts_REG             MDFault;
    union       MDErrStauts_REG             MDBalaMode;
    union       MDErrStauts_REG             MDWaterleak;
    union       MDErrStauts_REG             MDCellVoltFualt;
    union       MDErrStauts_REG             MDCellTempFualt;
    union       MDErrStauts_REG             MDBATICErr;
    union       MDErrStauts_REG             MDCTComErr;
    union       MDErrStauts_REG             MDSBComErr;
    union       MDErrStauts_REG             MDHMIComErr;
    union       MDErrStauts_REG             MDVoltCanTx;
    union       MDErrStauts_REG             MDTempCanTx;
    union       MDErrStauts_REG             HmiModeEn;
    union       MDErrStauts_REG             PackModeEn;

}SystemReg;
typedef struct Module_Date
{


    Uint16 MD1XRxcount[7];
    Uint16 MD2XRxcount[7];

    Uint16 MD3XRxcount[7];
    Uint16 MD4XRxcount[7];

    Uint16 MD5XRxcount[7];
    Uint16 MD6XRxcount[7];

    Uint16 MD7XRxcount[7];
    Uint16 MD8XRxcount[7];

    Uint16 MD9XRxcount[7];
    Uint16 MDAXRxcount[7];

    Uint16 MDBXRxcount[7];
    Uint16 MDCXRxcount[7];

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

    Uint16 MDInResis[7];
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
{       // bits   description (0x602 Data1 = R59 SB16~31)
    // TODOS : [완료] (51, 0x602 BPA_State 비트맵 재배치 (Aux 릴레이/EMG/OffGas 추가))
    /*--------------------------------------------------------------
     * 260615 : 0x602 PackStatus(SB16~31) R59 정렬
     *          - R59 미정의 Aux 릴레이3(bit4~6)·OffGas(bit10) 제거
     *          - MSD_AUX/EMG_SW/Water_leak 을 SB20~22(bit4~6)로 당김
     *--------------------------------------------------------------*/
    //unsigned int     PackBalance          :1; // 0
    //unsigned int     PackNeg_Rly          :1; // 1
    //unsigned int     PackPos_Rly          :1; // 2
    //unsigned int     PackPreChar_Rly      :1; // 3
    //unsigned int     PackNeg_Rly_Aux      :1; // 4   (R59 삭제)
    //unsigned int     PackPos_Rly_Aux      :1; // 5   (R59 삭제)
    //unsigned int     PackPreChar_AUX      :1; // 6   (R59 삭제)
    //unsigned int     PackMSD_AUX          :1; // 7
    //unsigned int     PackEmg_STOP_SW      :1; // 8
    //unsigned int     PackWater_Leak       :1; // 9
    //unsigned int     PackOffGas           :1; // 10  (R59 0x602 미정의)
    unsigned int     PackBalance          :1; // 0   BRA1_Balance     (SB16)   // TODOS : [검증] (73, 0x602 PackStatus R59 정렬)
    unsigned int     PackNeg_Rly          :1; // 1   BRA1_NegRly      (SB17)
    unsigned int     PackPos_Rly          :1; // 2   BRA1_PosRly      (SB18)
    unsigned int     PackPreChar_Rly      :1; // 3   BRA1_PreCharRly  (SB19)
    unsigned int     PackMSD_AUX          :1; // 4   BRA1_MSD_AUX     (SB20)
    unsigned int     PackEmg_STOP_SW      :1; // 5   BRA1_EMG_SW      (SB21)
    unsigned int     PackWater_Leak       :1; // 6   BRA1_Water_leak  (SB22)
    unsigned int     Stauts07             :1; // 7   (SB23 예비)
    unsigned int     Stauts08             :1; // 8   (SB24 예비)
    unsigned int     Stauts09             :1; // 9   (SB25 예비)
    unsigned int     Stauts10             :1; // 10  (SB26 예비)
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
   unsigned int     RUNStatus01          :1; // 0  MAS1_WakeUp
   unsigned int     RUNStatus02         :1; // 1
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

struct DebugState_BIT
{       // bits   description
   unsigned int     DebStatus00         :1; // 15
   unsigned int     DebStatus01          :1; // 0
   unsigned int     DebStatus02          :1; // 1
   unsigned int     DebStatus03          :1; // 2
   unsigned int     DebStatus04          :1; // 3
   unsigned int     DebStatus05          :1; // 4
   unsigned int     DebStatus06          :1; // 5
   unsigned int     DebStatus07          :1; // 6
   unsigned int     DebStatus08          :1; // 7
   unsigned int     DebStatus09          :1; // 8
   unsigned int     DebStatus10         :1; // 9
   unsigned int     DebStatus11         :1; // 10
   unsigned int     DebStatus12         :1; // 11
   unsigned int     DebStatus13         :1; // 12
   unsigned int     DebStatus14         :1; // 13
   unsigned int     DebStatus15         :1; // 14

};
union DebugState_REG
{
   unsigned int     all;
   struct DebugState_BIT bit;
};



typedef struct CANA_DATA
{
    Uint16 PackID;
    Uint16 PackSWID;
    Uint16 SWTypeVer;
    Uint16 MDNumCountA;
    Uint16 MDNumCountB;
    Uint16 MDNumCountC;
    Uint16 MDNumCountD;
    Uint16 MDNumCountE;
    Uint16 MDNumCountF;

    Uint16  MDRxCount1;
    Uint16  MDRxCount2;
    Uint16  MDRxCount3;
    Uint16  MDRxCount4;

 

    // TODOS : [검증] (52, 0x60A 멀티플렉서용 모듈별 RxCount 포인터 배열 (ModRegs.MD{N}XRxcount 가리킴))
    Uint16 *MDXRxcountP[7];
    Uint16 *MDRxCurP;        // 0x60A 현재 모듈 카운터 포인터 (지역 r 대체)

    Uint16 CanMBOX0Mask;
    Uint16 CanMBOX1Mask;
    Uint16 CanMBOX2Mask;
    Uint16 CanMBOX3Mask;
    Uint16 CanMBOX4Mask;
    Uint16 CanMBOX5Mask;
    Uint16 CanMBOX6Mask;
    Uint16 CanMBOX29Mask;
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

    Uint16  MDTotalVolt [7];
    float32 MDTotalVoltF[7];
    Uint16  MDstatusbit[7];
    Uint16  PackMinVolteRec[7];
    Uint16  MD1_TotalVolt;
    Uint16  MD1_CellAgvVolt;
    int16   MD1_CellAgvTemps;
    Uint16  MD2_TotalVolt;
    Uint16  MD2_CellAgvVolt;
    int16   MD2_CellAgvTemps;
    Uint16  MD3_TotalVolt;
    Uint16  MD3_CellAgvVolt;
    int16   MD3_CellAgvTemps;
    Uint16  MD4_TotalVolt;
    Uint16  MD4_CellAgvVolt;
    int16   MD4_CellAgvTemps;
    Uint16  MD5_TotalVolt;
    Uint16  MD5_CellAgvVolt;
    int16   MD5_CellAgvTemps;
    Uint16  MD6_TotalVolt;
    Uint16  MD6_CellAgvVolt;
    int16   MD6_CellAgvTemps;
    Uint16  MD7_TotalVolt;
    Uint16  MD7_CellAgvVolt;
    int16   MD7_CellAgvTemps;

    Uint16  MDCellMaxVolt[7];
    Uint16  MDCellMinVolt[7];
    Uint16  MDCellAgvVolt[7];
    Uint16  MDCellDivVolt[7];

    int16 MDCellMaxTemps[7];
    int16 MDCellMinTemps[7];
    int16 MDCellAgvTemps[7];
    Uint16 MDCellDivTemps[7];

    int16 MDCellMaxIR[7];
    int16 MDCellMinIR[7];
    int16 MDCellAgvIR[7];
    Uint16 MDCellDivIR[7];

    Uint16 MDInResis[7];
    /*
     *
     */
    union BATStatus_REG               PackStatus;
    union DigitalOutPut_REG           PackDigitalOutPutReg;
    union VCUCOMMAND_REG              PMSCMDRegs;
//    union VCUCOMMAND_REG              PMSCMDRegs;
    Uint16  SysCellMinV;      // 260714 : 0x700 MAS1_Cell_MinV 원본값 (raw)
    Uint16  SysCellAgvT;      // 260714 : 0x700 MAS1_Cell_AGVT 원본값 (raw)
    Uint16 PackSate;
    Uint16 PackProtetSate;
    Uint16 PackSateInfo;
    Uint16 PackConFig;
    int16  PackSOC;
    Uint16 PackSOH;
    int16 PackAh;
    Uint16 PackISORegs;
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
    // TODOS : [완료] (53, 0x607 위치정보 송신용 ComBine(Max,Min) 합산 필드)
    Uint16 PackVoltageMaxMinNum;
    Uint16 PackTemperatureMaxMinNum;
    Uint16 PackIRMaxMinNUM;
    // TODOS : [완료] (54, 0x607 셀 내부저항 송신값 (factor 0.1, mohm))
    Uint16 CellIRMAX;
    Uint16 CellIRlMIN;
    Uint16 CellIRAVG;
    Uint16 CellIRDiv;
    Uint16 PackPackVotageBuf;
    int16  PackTemperaturelMAX;
    int16  PackTemperaturelMIN;
    int16  PackTemperatureAVG;
    Uint16 PackTemperatureDiv;
    Uint16 PackTemperatureMaxNUM;
    Uint16 PackTemperatureMinNUM;
    Uint16 SWVER;
    Uint16 BatConfParallelSerial;

/*
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

*/
/*
    union   MoudleState_REG MD1Staue;
    union   MoudleState_REG MD2Staue;
    union   MoudleState_REG MD3Staue;
    union   MoudleState_REG MD4Staue;
    union   MoudleState_REG MD5Staue;
    union   MoudleState_REG MD6Staue;
    union   MoudleState_REG MD7Staue;
    union   DebugState_REG  DebugStaueA;
    union   DebugState_REG  DebugStaueB;
    union   DebugState_REG  DebugStaueC;
    union   DebugState_REG  DebugStaueD;
    union   DebugState_REG  DebugStaueE;
    union   DebugState_REG  DebugStaueF;
    union   DebugState_REG  DebugStaueG;
    union   DebugState_REG  DebugStaueH;
*/
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


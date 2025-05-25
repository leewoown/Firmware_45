
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <stdio.h>
#include <string.h>
//extern MODReg MODRegs;

extern void InitECan(void);
extern void SciaTxchar(char Txchar);

extern void SPI_Write(unsigned int WRData);
extern unsigned int SPI_READ(void);
extern void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3 );
 
extern void ModuleCellMIN(SlaveReg *S);
extern void ModuleCellMAX(SlaveReg *S);
//extern void BatteryTeperaureCal(BatteryReg *bat,int Count);




extern void TEST_LTC6804(void);
extern int LTC6804_Init(void);

extern void SlaveBMSSPIEnable_low(void);
extern void SlaveBMSSPIEnable_high(void);
extern void SlaveBms_WakeUp(void);
extern int SlaveBMSCellReadCommand(SlaveReg *s);
extern void SlaveBmsBalance(SlaveReg *s);


extern float32 AbsVaule(float32 input);
extern void CellVoltagetoFloat(SlaveReg *s, ModuleReg *m);

extern void SystemAlarm(PackReg *bat,SystemReg *sys);
extern void SystemRegsInit(SystemReg *s);
extern void systemParmRead(SystemReg *sys,PackReg *bat);
extern void sysDigitalInput(SystemReg *sys);
extern void sysDigitalOutput(SystemReg *sys);
extern void SysCurrentOffset(SystemReg *sys);
extern void SystemforceBalance(ModuleReg *m);


extern void DigitalInput(SystemReg *sys);
extern void DigitalOutput(SystemReg *sys);
extern void SlaveRegsInit(SlaveReg *s);
extern void ModuleRegsInit(ModuleReg *m);
extern void PackRegsInit(PackReg *b);
extern void SystemInit(SystemReg *s);
extern void CellTempCalFloat(SlaveReg *s, ModuleReg *m);
extern void CellVOLTFloatTOInt(ModuleReg *s, CANTXReg *t);
extern void CellTempFloatToInt(ModuleReg *s, CANTXReg *t);

void CellTempFloatToInt(ModuleReg *s, CANTXReg *t)
{
	t->BatteryTempCell[t->CellNumer]  =(int)(s->CellTempF[0]*10.0);
	t->BatteryTempCell[t->CellNumer+1]=(int)(s->CellTempF[1]*10.0);
	t->BatteryTempCell[t->CellNumer+2]=(int)(s->CellTempF[2]*10.0);
	t->BatteryTempCell[t->CellNumer+3]=(int)(s->CellTempF[3]*10.0);
}
void CellVOLTFloatTOInt(ModuleReg *s, CANTXReg *t)
{
	t->BatteryVoltageCell[t->CellNumer]=(unsigned int)(s->CellVoltageF[0]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+1]=(unsigned int)(s->CellVoltageF[1]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+2]=(unsigned int)(s->CellVoltageF[2]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+3]=(unsigned int)(s->CellVoltageF[3]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+4]=(unsigned int)(s->CellVoltageF[4]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+5]=(unsigned int)(s->CellVoltageF[5]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+6]=(unsigned int)(s->CellVoltageF[6]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+7]=(unsigned int)(s->CellVoltageF[7]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+8]=(unsigned int)(s->CellVoltageF[8]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+9]=(unsigned int)(s->CellVoltageF[9]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+10]=(unsigned int)(s->CellVoltageF[10]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+11]=(unsigned int)(s->CellVoltageF[11]*1000.0);


}
void TEST_LTC6804(void)
{
	LTC680x_CS;
	SPI_Write(0X55);
	SPI_Write(0X55);
	LTC680X_DS;
}
void Alarm_LED_handle(SystemReg *sys)
{

}
void Emergency_LED_handle(SystemReg *sys)
{
 
}
void SlaveRegsInit(SlaveReg *s)
{
	
}
void ModuleRegsInit(ModuleReg *m)
{

}
void PackRegsInit(PackReg *b)
{
//	b->PackVoltage=0.0;
//	b->PackTemp=0.0;
//	b->PackSOC=0.0; 

}
void SystemRegsInit(SystemReg *s)
{
	LTC680X_DS;
	s->initCount=0;
	s->DigitalOutPutReg.all=0;
	s->DigitalInputReg.all=0;
	s->SystemStateARegs.all=0x0000;
	s->Timer50msec=0;
	s->Timer250msec=0;
	s->Timer500msec=0;
	s->Timer1000msec=0;
	s->Timer1500msec=0;
	s->Timer2000msec=0;
	s->Timer2500msec=0;
 
}
void SystemInit(SystemReg *s)
{
	
	if(s->initCount<500)
	{
		s->SystemStateARegs.bit.INITOK=0;
	}
	if(s->initCount>=500)
	{
		s->SystemStateARegs.bit.INITOK=1;
		s->initCount=501;
	}
}

void SystemAlarm(PackReg *bat,SystemReg *sys)
{

}
void SystemFault(PackReg *bat, SystemReg *sys)
{

}
void SysCurrentOffset(SystemReg *sys)
{
}
void SocBalance(ModuleReg *m) 
{
	
}
void AutoBalance(ModuleReg *m) 
{
	if(m->CellVoltageF[0]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell00=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell00=1;
	}
	if(m->CellVoltageF[1]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell01=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell01=1;
	}
	if(m->CellVoltageF[2]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell02=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell02=1;
	}
	if(m->CellVoltageF[3]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell03=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell03=1;
	}
	if(m->CellVoltageF[4]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell04=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell04=1;
	}
	if(m->CellVoltageF[5]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell05=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell05=1;
	}
	if(m->CellVoltageF[6]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell06=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell06=1;
	}
	if(m->CellVoltageF[7]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell07=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell07=1;
	}
	if(m->CellVoltageF[8]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell08=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell08=1;
	}
	if(m->CellVoltageF[9]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell09=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell09=1;
	}
	if(m->CellVoltageF[10]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell10=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell10=1;
	}
	if(m->CellVoltageF[11]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell11=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell11=1;
	}
}
void SystemforceBalance(ModuleReg *m)
{
	if(m->CellVoltageF[0]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell00=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell00=1;
	}
	
	if(m->CellVoltageF[1]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell01=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell01=1;
	}
	
	if(m->CellVoltageF[2]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell02=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell02=1;
	}
	
	if(m->CellVoltageF[3]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell03=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell03=1;
	}
	
	if(m->CellVoltageF[4]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell04=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell04=1;
	}
	
	if(m->CellVoltageF[5]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell05=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell05=1;
	}
	
	if(m->CellVoltageF[6]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell06=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell06=1;
	}
	
	if(m->CellVoltageF[7]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell07=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell07=1;
	}
	
	if(m->CellVoltageF[8]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell08=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell08=1;
	}
	
	if(m->CellVoltageF[9]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell09=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell09=1;
	}
	
	if(m->CellVoltageF[10]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell10=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell10=1;
	}
	
	if(m->CellVoltageF[11]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell11=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell11=1;
	}

}

float32 AbsVaule(float32 input)
{
	float32 AbsCurrent=0.0;
	if(input>=0)
	{
		AbsCurrent=input;
	}
	else if(input<0)
	{
		AbsCurrent=-1.0*input;
	}	
	return AbsCurrent;
}

void CellTempCalFloat(SlaveReg *s, ModuleReg *m)
{
	float32 tmpF0=0.0;
	float32 tmpF1=0.0;
	float32 tmpF2=0.0;
	float32 tmpF3=0.0;

	float32 x02=0.0;
	float32 x0=0.0;
	float32 x12=0.0;
	float32 x1=0.0;
	float32 x22=0.0;
	float32 x2=0.0;
	float32 x32=0.0;
	float32 x3=0.0;

	tmpF0=(float)(s->CellTempBuffer[0]*0.0001);
	tmpF1=(float)(s->CellTempBuffer[1]*0.0001);
	tmpF2=(float)(s->CellTempBuffer[2]*0.0001);
	tmpF3=(float)(s->CellTempBuffer[3]*0.0001);

	x02= tmpF0*tmpF0;
	x0=tmpF0;
	m->CellTempF[0]= 3.5081*x02-41.81*x0+107.59;

	x12= tmpF1*tmpF1;
	x1=tmpF1;
	m->CellTempF[1]= 3.5081*x12-41.81*x1+107.59;


	x22= tmpF2*tmpF2;
	x2=tmpF2;
	m->CellTempF[2]= 3.5081*x22-41.81*x2+107.59;

	x32= tmpF3*tmpF3;
	x3=tmpF3;
	m->CellTempF[3]= 3.5081*x32-41.81*x3+107.59;

}
void CellVoltagetoFloat(SlaveReg *s, ModuleReg *m)
{
	int i;
	float32 sum=0.0;
	
	for (i = 0; i < ModuleCellNum; i++)
	{	
	//	m->CellVoltageF[i]=(float32)s->CellVoltage[i]/10000;
		s->CellVoltagebuffer[i]=s->CellVoltage[i]/10;
		m->CellVoltageF[i]=(float32)(s->CellVoltagebuffer[i]*0.001);

	//	m->CellVoltageF[i]=m->CellVoltageF[i]+0.0005;
		sum=sum+m->CellVoltageF[i];
	}
	m->ModuleVoltageCellMAX=(float32)(s->CellVoltageMax*0.001);
	m->ModuleVoltageCellMIN=(float32)(s->CellVoltageMin*0.001);
	m->ModuleVoltageCellAVG=m->ModuleVoltageF/12.0;
	m->ModuleVoltageCellCap=m->ModuleVoltageCellMAX-m->ModuleVoltageCellMIN;
	m->ModuleVoltageF=sum;
	
}
// ÀÔ·Â ÀÎÀÚ 
// S->CellNumber : 
void ModuleCellMAX(SlaveReg *S)
{
	int i;
	S->CellVoltageMax=S->CellVoltage[0];
	S->CellVoltageMaxNum=0;
	for(i=0; i<S->CellNumber;i++)
	{
		if(S->CellVoltageMax< S->CellVoltage[i])
		{
			S->CellVoltageMax = S->CellVoltage[i];	
			S->CellVoltageMaxNum=i;
		}
	}
	
}
// ÀÔ·Â ÀÎÀÚ 
// S->CellNumber : 
void ModuleCellMIN(SlaveReg *S)
{
	int i;
	S->CellVoltageMin=S->CellVoltage[0];
	S->CellVoltageMinNum=0;
	for(i=0; i<S->CellNumber;i++)
	{
		if(S->CellVoltageMin> S->CellVoltage[i])
		{
			S->CellVoltageMin = S->CellVoltage[i];	
			S->CellVoltageMinNum=i;
		}
	}
}
void BatteryTeperaureCal(PackReg *bat,int Count)
{
	//bat->BatteryTempCellF[i]=(-4.67*num3+35.688*num2)-102.18*num1+91.477;
}
void SlaveBMSSPIEnable_low(void)
{
	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
}

void SlaveBMSSPIEnable_high(void)
{
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
}
void SlaveBms_WakeUp(void)
{
	// need to check
	SlaveBMSSPIEnable_low();
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	SlaveBMSSPIEnable_high();
}
int SlaveBMSCellReadCommand(SlaveReg *s)
{
	int i;
	SlaveBms_WakeUp();
	SlaveBMSSPIEnable_low();
//	s->WError= LTC6804_write(s->ID, s->Command, 0, 0);

	if((s->WError!=0)&&(s->len !=0))
	{
		for (i = 0; i < s->len; i++) 
		{
			s->ADCX[i]= SPI_READ();
		}
		s->pecr = (SPI_READ() << 8) & 0xff00;
		s->pecr |= (SPI_READ() & 0x00ff);
  //		s->pecg = pec15(s->ADCX[i], s->len);
		if (s->pecr ==s->pecg) 
		{
			s->RError=0;
			s->ErrorCount= 0;
		} 
		else 
		{
			s->RError=1;
			s->ErrorCount++;
		}
	}
	if(s->RError==1)
	{
		if(s->Command==LTC6804_CMD_RDCVA)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVA;
		}
		if(s->Command==LTC6804_CMD_RDCVB)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVB;
		}
		if(s->Command==LTC6804_CMD_RDCVC)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVC;
		}
		if(s->Command==LTC6804_CMD_RDCVD)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVD;
		}
	}
	SlaveBMSSPIEnable_high();
	return s->ErrorCode;	
}
/*
int CellBalanceSet(int pack_id, CellBalance_t cell)
{
	int ret;
	char addr = LTC6804.address[pack_id];
	char *tab = (char *)LTC6804_init_table;
	tab[4] = (cell.all & 0xff);
	tab[5] = (tab[5] & 0xf0) | ((cell.all >> 8) & 0x0f);
	ret = LTC6804_write_cmd(addr, LTC6804_CMD_WRCFG, tab, sizeof(LTC6804_init_table));
	return ret;
}*/


//-----------------------------------------------------------------------

void SciaTxchar(char Txchar)
{
	while(!(SciaTxReadyFlag));
	SciaRegs.SCITXBUF=Txchar;
}
void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3)
{
	unsigned int Data0Low, Data0High, Data1Low, Data1High;
	unsigned int Data2Low, Data2High, Data3Low, Data3High;
	struct ECAN_REGS ECanaShadow;


	Data0Low  = 0x00ff&Data0;
	Data0High = 0x00ff&(Data0>>8);

	Data1Low  = 0x00ff&Data1;
	Data1High = 0x00ff&(Data1>>8);

	Data2Low  = 0x00ff&Data2;
	Data2High = 0x00ff&(Data2>>8);

	Data3Low  = 0x00ff&Data3;
	Data3High = 0x00ff&(Data3>>8);


	// ÇöÀç CAN-A MBOX31°¡ Àü¼Û ÁßÀÌ¸é ¸®ÅÏÇÔ
	if(ECanaRegs.CANTRS.bit.TRS31== 1) return;
	//ÇöÀç CAN-A MBOX31°¡ Àü¼Û »óÅÂ Ã¼Å©ÇÔ.
  	if(ECanaShadow.CANTA.bit.TA31== 0) // 0 : Send Fail,  1 : Send OK
	{
		ECanaShadow.CANTA.all = 0;
      	ECanaShadow.CANTA.bit.TA31= 1;     	         	// Clear TA5
       	ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;
   	}

/*
	// CAN-A ¼Û¼ö½Å Ã¼Å©
	if(COM_FAULT_MODE)	CanDataCnt++;			// reset at Rx Routine
	if(CanDataCnt > COMER_set)	COMER_FS=1;
	Can_TX_ERR_CNT = ECanaRegs.CANTEC.bit.TEC;
	Can_RX_ERR_CNT = ECanaRegs.CANREC.bit.REC;
*/
	// Àü¼ÛÇÏ´Â µ¥ÀÌÅÍ ID ¼³Á¤ÇÔ.
	ECanaRegs.CANME.bit.ME31= 0;
	ECanaMboxes.MBOX31.MSGID.bit.STDMSGID=ID;
	ECanaRegs.CANME.bit.ME31= 1;
	// Àü¼ÛÇÏ´Â µ¥ÀÌÅÍ ±æÀÌ?8 ¼³Á¤ÇÔ.


	ECanaMboxes.MBOX31.MSGCTRL.bit.DLC=Length; 	// Àü¼ÛÇÏ´Â µ¥ÀÌÅÍ ±æÀÌ
	// DATA Àü¼Û
	ECanaMboxes.MBOX31.MDL.byte.BYTE0=Data0Low;
	ECanaMboxes.MBOX31.MDL.byte.BYTE1=Data0High;

	ECanaMboxes.MBOX31.MDL.byte.BYTE2=Data1Low;
	ECanaMboxes.MBOX31.MDL.byte.BYTE3=Data1High;

	ECanaMboxes.MBOX31.MDH.byte.BYTE4=Data2Low;
	ECanaMboxes.MBOX31.MDH.byte.BYTE5=Data2High;

	ECanaMboxes.MBOX31.MDH.byte.BYTE6=Data3Low;
	ECanaMboxes.MBOX31.MDH.byte.BYTE7=Data3High;

	// CAN-A MBOX31 Àü¼Û ½ÃÀÛÇÔ.
	ECanaRegs.CANTRS.bit.TRS31= 1;
	// CAN-A MBOX31Àü¼Û ¿Ï·á Ã¼Å©ÇÔ.
	while(!ECanaShadow.CANTA.bit.TA31);
	ECanaShadow.CANTA.all = 0;
   	ECanaShadow.CANTA.bit.TA31=1;     	         	// Clear TA5
   	ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;
	ECanaMboxes.MBOX31.MSGID.all	= 0;

	// From Here ÇØ´ç MailBox¸¦ CAN Enable ½ÃÅ´  ÀÌ°ÍÀ» ¿Ö ´Ù½Ã ÇØ ÁÖ¾î¾ß ÇÏö?
	EALLOW;     // EALLOW enables access to protected bits

	ECanaShadow.CANME.all = ECanaRegs.CANME.all;

	ECanaShadow.CANME.bit.ME0= 1;
	ECanaShadow.CANME.bit.ME1= 1;
	ECanaShadow.CANME.bit.ME2= 1;
	ECanaShadow.CANME.bit.ME3= 1;
	ECanaShadow.CANME.bit.ME4= 1;
	ECanaShadow.CANME.bit.ME5= 1;
	ECanaShadow.CANME.bit.ME6= 1;
	ECanaShadow.CANME.bit.ME7= 1;
	ECanaShadow.CANME.bit.ME8= 1;
	ECanaShadow.CANME.bit.ME9= 1;
	ECanaShadow.CANME.bit.ME10= 1;
	ECanaShadow.CANME.bit.ME11= 1;
	ECanaShadow.CANME.bit.ME12= 1;
	ECanaShadow.CANME.bit.ME13= 1;
	ECanaShadow.CANME.bit.ME14= 1;
	ECanaShadow.CANME.bit.ME15= 1;
	ECanaShadow.CANME.bit.ME16= 1;
	ECanaShadow.CANME.bit.ME17= 1;
	ECanaShadow.CANME.bit.ME18= 1;
	ECanaShadow.CANME.bit.ME19= 1;
	ECanaShadow.CANME.bit.ME20= 1;
	ECanaShadow.CANME.bit.ME21= 1;
	ECanaShadow.CANME.bit.ME22= 1;
	ECanaShadow.CANME.bit.ME23= 1;
	ECanaShadow.CANME.bit.ME24= 1;
	ECanaShadow.CANME.bit.ME25= 1;
	ECanaShadow.CANME.bit.ME26= 1;
	ECanaShadow.CANME.bit.ME27= 1;
	ECanaShadow.CANME.bit.ME28= 1;
	ECanaShadow.CANME.bit.ME29= 1;
	ECanaShadow.CANME.bit.ME30= 1;
	ECanaShadow.CANME.bit.ME31= 1;
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;
	InitECan();

	EDIS;

	// To Here ÇØ´ç MailBox¸¦ CAN Enable ½ÃÅ´  ÀÌ°ÍÀ» ¿Ö ´Ù½Ã ÇØ ÁÖ¾î¾ß ÇÏÁö?

}//EOF

//----------------------------------------------------------------------------------µðÁöÅÐ INPUT/OUTPUT °ü·Ã ÇÔ¼ö
void DigitalInput(SystemReg *sys)
{
	
	if(ID_SW00==0)
	{
		sys->DigitalInputReg.bit.SW00=1;
	}
	else if(ID_SW00==1)
	{
		sys->DigitalInputReg.bit.SW00=0;
	}

	if(ID_SW01==0)
	{
		sys->DigitalInputReg.bit.SW01=1;
	}
	else if(ID_SW01==1)
	{
		sys->DigitalInputReg.bit.SW01=0;
	}

	if(ID_SW02==0)
	{
		sys->DigitalInputReg.bit.SW02=1;
	}
	else if(ID_SW02==1)
	{
		sys->DigitalInputReg.bit.SW02=0;
	}

	if(ID_SW03==0)
	{
		sys->DigitalInputReg.bit.SW03=1;
	}
	else if(ID_SW03==1)
	{
		sys->DigitalInputReg.bit.SW03=0;
	}
	sys->BMSID=sys->DigitalInputReg.all&0x000f;
}

void DigitalOutput(SystemReg *sys)
{
	if(sys->DigitalOutPutReg.bit.Relay==1)
  	{
  		Relay_ON; 
  	}
	else 
	{
		Relay_OFF;
	}
}

void DataConversion(SystemReg *sys)
{

}




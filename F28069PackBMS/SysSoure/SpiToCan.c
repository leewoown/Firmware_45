



#include "DSP28x_Project.h"
#include "F2806x_Device.h"     // F2806x Headerfile Include File
#include "F2806x_Examples.h"   // F2806x Examples Include File
#include "parameter.h"
#include "ProtectRelay.h"
#include "SysSpiCan.h"
#include <stdio.h>
#include <math.h>
#include <string.h>



extern void InitSpiGpio();
extern void InitSpiBATIC(void);
extern void InitSpiCAN(void);
extern void SPI_Write(unsigned int WRData);
extern unsigned int SPI_Read(void);


extern void TestCANSPIWriteBytesHandle(void);
extern void CANSPIReadBytesHandle (Uint16 cmd, Uint16 addr, char rxBuf[], Uint16 len);
extern void CANSPIWriteBytesHandle(Uint16 cmd, Uint16 addr, char TxBuf[], Uint16 len);

extern void MCP2515ResetHandle(void);
extern void MCP2515SetCNFHandle(char cnf1, char cnf2, char cnf3);
extern void MCP2515SetNormalModeHandle(void);

extern void MCP2515InitHandle(CANBReg *p);


void CANSPIWriteBytesHandle(Uint16 cmd, Uint16 addr, char TxBuf[], Uint16 len)
{
    // 1. MCP2515 ���� (CS LOW)
    Uint16 i =0;
    //MCP2515CSEnable;

    // 2. ��ɾ� ���� (��: 0x02 = WRITE, 0x40 = LOAD TX BUFFER ��)
    SPI_Write(cmd);

    // 3. �ּ� ���� (�������� �ּ� �Ǵ� ���� ��ġ)
    SPI_Write(addr);

    // 4. ������ ���� (len ��ŭ �ݺ�)
    for (i = 0; i < len; i++)
    {
        SPI_Write(TxBuf[i]);
    }

    // 5. MCP2515 ���� ���� (CS HIGH)
    //MCP2515CSDisble;
}
void CANSPIReadBytesHandle(Uint16 cmd, Uint16 addr, char rxBuf[], Uint16 len)
{
    Uint16 i=0;
    // 1. READ ��� ����
    SPI_Write(cmd);     // ���� MCP_READ = 0x03

    // 2. ���� �ּ� ����
    SPI_Write(addr);

    // 3. ���ϴ� ���̸�ŭ ���� �б� (auto-increment ������)
    for (i = 0; i < len; i++)
    {
        rxBuf[i] = SPI_Read();  // SPI_Read()�� 8��Ʈ ���� �Լ�
    }

}
void TestCANSPIWriteBytesHandle(void)
{
    char configData[3] = {0x03, 0x90, 0x02};  // CNF1, CNF2, CNF3 ������
    CANSPIWriteBytesHandle(0x02,0x2A,configData,3);
}

void MCP2515ResetHandle(void)
{
    // 1. CS Ȱ��ȭ
    MCP2515CSEnable;
    // 2. RESET ��ɾ� ����
    SPI_Write(MCP_RESET);  // 0xC0
    // 3. CS ��Ȱ��ȭ
    MCP2515CSDisble;
    // 4. MCP2515�� ���������� �ʱ�ȭ �Ϸ��� ������ ���
    delay_us(10000);  // 10ms ���
}

void MCP2515setConfgModeHanlde(void)
{
    MCP2515CSEnable;
    char mode = MODE_CONFIG;
    MCP2515CSEnable;
    CANSPIWriteBytesHandle(MCP_WRITE,MCP_CANCTRL,&mode,1);
    MCP2515CSDisble;
    delay_us(1000);  // ���� �� ����ȭ�� ���� 1ms ���
}
void MCP2515SetNormalModeHandle(void)
{
    MCP2515CSEnable;
    char configData[3];
    configData[0]= MCP_WRITE;
    configData[1]= MCP_CANCTRL;
    configData[2]= MODE_NORMAL;
    CANSPIWriteBytesHandle(MCP_WRITE,MCP_CANCTRL,configData,1);
    delay_us(1000);  // ��� ��ȯ ����ȭ ���
    MCP2515CSDisble;
}
void MCP2515SetCNFHandle(char cnf1, char cnf2, char cnf3)
{
    // MCP2515SetCNFHandle(MCP_20MHz_125kBPS_CFG1,MCP_20MHz_125kBPS_CFG2,MCP_20MHz_125kBPS_CFG3);
    // #define MCP_20MHz_500kBPS_CFG1 (0x00)
    // #define MCP_20MHz_500kBPS_CFG2 (0xD1)
    // #define MCP_20MHz_500kBPS_CFG3 (0x03)

    // #define MCP_20MHz_250kBPS_CFG1 (0x01)
    // #define MCP_20MHz_250kBPS_CFG2 (0xD1)
    // #define MCP_20MHz_250kBPS_CFG3 (0x03)

    // #define MCP_20MHz_125kBPS_CFG1 (0x03)
    // #define MCP_20MHz_125kBPS_CFG2 (0xD1)
    // #define MCP_20MHz_125kBPS_CFG3 (0x03)

    char configData[3];
    configData[0] = cnf1;  // CNF1 (0x2A)
    configData[1] = cnf2;  // CNF2 (0x29)
    configData[2] = cnf3;  // CNF3 (0x28)
    MCP2515CSEnable;
    // MCP_WRITE ���(0x02), ���� �ּ� = CNF1 (0x2A), ������ ���� = 3
    CANSPIWriteBytesHandle(MCP_WRITE, MCP_CNF1, configData,3);

    MCP2515CSDisble;
    delay_us(1000);  // ���� �� ����ȭ�� ���� 1ms ���
}
void MCP2515InitHandle(CANBReg *p)
{
   // char readBack[1] = {0};
    p->DedugRegs.all=0x0000;

    MCP2515ResetHandle();  // ���ο� DELAY_US(10000) ���ԵǾ�� ��
    MCP2515setConfgModeHanlde(); //


    MCP2515SetCNFHandle(MCP_20MHz_500kBPS_CFG1,MCP_20MHz_500kBPS_CFG2,MCP_20MHz_500kBPS_CFG3); //���� Delay Us(1000)

    MCP2515CSEnable;
    SPI_Write(0x03);  // 0x03
    SPI_Write(0x2A);  // 0x2a
    p->Rxbuf[0]=SPI_Read();
   // p->len=1;
  //  CANSPIReadBytesHandle (MCP_READ, MCP_CNF1, p->Rxbuf, p->len);
    MCP2515CSDisble;

    if(p->Rxbuf[0]!=MCP_20MHz_500kBPS_CFG1)
    {
        p->DedugRegs.bit.Dedug00=1;
    }
    else
    {
        p->DedugRegs.bit.Dedug00=0;
    }

/*
    MCP2515CSEnable;
    MCP2515SetNormalModeHandle();
    MCP2515CSDisble;
    delay_us(1000);  // ����ȭ ���
*/
  //  return 1;
}


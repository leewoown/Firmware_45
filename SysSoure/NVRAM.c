
/*
 * NVRAM.C
 *
 *  Created on: 2025. 11. 9.
 *      Author: leewo
 */

#include "DSP28x_Project.h"
#include "NVRAM.h"
extern void SPI_Write(unsigned int WRData);
extern unsigned int SPI_Read(void);
/******************************************************************************************
 *
 *  NVRAM MR25H40
 *
 *  When SPI clock is 1MHz, data transfer rate is 100bytes per 1 mSec
 *  If mass data should to be sent in interrupt routine, you should be careful of timing
 *
 ******************************************************************************************/

void NVRAMSPIInit(void)
{
    //
    // Initialize SPI-A/B
    //
    SpiaRegs.SPICCR.bit.SPISWRESET      = 0;    // SPI SW Reset hold
    SpiaRegs.SPICCR.bit.SPILBK          = 0;    // Disable Loopback
    SpiaRegs.SPICCR.bit.SPICHAR         = 0x7;  // 8bit character

    SpiaRegs.SPICTL.bit.SPIINTENA       = 0;    // Disable SPI int
    SpiaRegs.SPICTL.bit.TALK            = 1;    // Tx enable
    SpiaRegs.SPICTL.bit.MASTER_SLAVE    = 1;    // Master mode
    SpiaRegs.SPICCR.bit.CLKPOLARITY     = 1;    // Falling edge output
    SpiaRegs.SPICTL.bit.CLK_PHASE       = 0;    // without delay mode
    SpiaRegs.SPICTL.bit.OVERRUNINTENA   = 0;    // Disable OverRun int

    //SpiaRegs.SPIBRR                   = 50;        //150 / 5 - 1; // Baud rate = LSPCLK/(SPIBRR+1), ~500k (for 6804 testing)
    SpiaRegs.SPIBRR                     = 10;         //=170; // Baud rate = LSPCLK/(SPIBRR+1) �빟 15/(3+1) = 3.75 MHz
                                                    // when SPIBRR 3 to 127
    SpiaRegs.SPICCR.bit.SPISWRESET      = 1;    // SPI SW Reset release
}

void NvramSpiWrite(NvramReg *p)
{
    /*
     * �궗�슜�삁
     * NvramRegs.WDCMD   = NVR_READ_CMD;
     * NvramRegs.WDAdder = 0x10u;           // Write �떆�옉 二쇱냼
     * NvramRegs.WDPtr   = WriteBuf;        // �벐湲� 踰꾪띁 �떆�옉 二쇱냼
     * NvramRegs.WDLen   = sizeof(WriteBuf);
     */


    Uint16 i = 0;
    Uint16 DataLen;
    const Uint8 *pArray;
    NVRAMSPIInit();
    delay_us(10);
    if(p->NvramStatus.bit.Active==1)
    {
        p->NvramStatus.bit.WDReturn=0;
    }
    p->NvramStatus.bit.Active=1;
    // Set configuration
    // SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;        // Rising edge output
    // SpiaRegs.SPIBRR                 = 10;       // Baud rate
    pArray   = p->WDPtr;                           // A�쓽 �떆�옉 二쇱냼
    DataLen  = p->WDLen;
    NvramCS;
    delay_us(5);
    SPI_Write(p->WDCMD);
    if(p->WDAdder!= 0xffu)
    {
        SPI_Write(p->WDAdder);           // No address field, address = 0xff
    }
    for (i = 0; i < DataLen; i++)
    {
        SPI_Write(pArray[i]);
    }
    NvramDS;
    delay_us(5);
    p->NvramStatus.bit.Active=0;
    p->NvramStatus.bit.WDReturn=1;

}

void NvramSpiRead(NvramReg *p)
{
    /*
     * �궗�슜�삁
     * NvramRegs.RDCMD   = NVR_READ_CMD;
     * NvramRegs.RDAdder = 0x10u;           // Read �떆�옉 二쇱냼
     * NvramRegs.RDPtr   = ReadBuf;         // �씫湲� 踰꾪띁�냼
     * NvramRegs.RDLen   = sizeof(ReadBuf);
     */
    Uint16 i = 0;
    Uint8 *pdst;
    NVRAMSPIInit();
    delay_us(10);
    if(p->NvramStatus.bit.Active==1)
    {
        p->NvramStatus.bit.RDReturn=0;
    }
    p->NvramStatus.bit.Active=1;

    // Set configuration
  //  SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;        // Rising edge output
  //  SpiaRegs.SPIBRR                 = 10;       // Baud rate

    NvramCS;
    delay_us(5);
    SPI_Write(p->RDCMD);
    if(p->RDAdder!= 0xffu) // No address field, address = 0xff
    {
        SPI_Write(p->RDAdder);
    }
    pdst = (Uint8*)p->RDPtr;
    for (i = 0; i < p->RDLen; i++)
    {
        *pdst++ = SPI_Read();
    }
    NvramDS;
    delay_us(5);
    p->NvramStatus.bit.Active=0;
    p->NvramStatus.bit.RDReturn=1;
}
void NVRAMRegsInitHandle(NvramReg *p)
{
    NVRAMSPIInit();
    delay_us(30);
    NvramCS;
    SPI_Write(NVR_WREN_CMD);
    SPI_Write(0xff);
    NvramDS;

    NvramCS;
    SPI_Write(NVR_WRSR_CMD);
    SPI_Write(0x00);
    SPI_Write(0x82);
    NvramDS;
}

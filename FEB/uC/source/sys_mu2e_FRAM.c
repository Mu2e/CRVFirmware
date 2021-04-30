//********************************************************
//  TI-RM48L uController powered
//  Fermilab Terry Kiper 2016-2020
//
//   FRAM 'FM25CL64B' Driver Source File
//   Program and Read FRAM memory chip 'FM25CL64B'
//   Hardware i/o pins setup for using TI-RM48L series uController
//
//   my_spiReceiveData  'does not control chip sel line'
//   my_spiTransmitData 'does not control chip sel line'
//
//********************************************************

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "ver_io.h" 
#include "hal_stdtypes.h"
#include "spi.h"
#include "sys_mu2e_FRAM.h"

extern  spiDAT1_t dataconfig_FRAM;
extern  char tBuf[];

uint32  my_spiTransmitData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * srcbuff);
uint32  my_spiReceiveData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * destbuff);

uint8 Op[] = {0, fWRSR, fWRITE, fREAD, fWRDI, fRDSR, fWREN};


//doc page 5 of 21
//FM25CL64B  F-RAM 64-Kbit as 8K×8  (20 Mhz SPI Interface)
//High-endurance 100 trillion (1014) read/writes
//FM25CL64B may be driven by a microcontroller with its SPI
//peripheral running in either of the following two modes:
//SPI Mode 0 (CPOL = 0, CPHA = 0)
//SPI Mode 3 (CPOL = 1, CPHA = 1)
//For both these modes, the input data is latched in on the rising
//edge of SCK starting from the first rising edge after CS goes
//active. If the clock starts from a HIGH state (in mode 3), the first
//rising edge after the clock toggles is considered. The output data
//is available on the falling edge of SCK.

//Transmit a byte to FRAM Chip 'FM25640B', Nonvolatile RAM (8,192 x 8 bits)
//via SPI_3, ALTERA FPGA
//Altera FPGA, DATA_RDY @ CLK_RISE_EDGE 'SSI_FRF_MOTO_MODE_0'
//Serial inputs are registered on the rising edge of SCK
//The SO output is driven from the falling edge of SCK.
//CS must go inactive after an  operation is complete and before a new op-code
//can be issued. There is one valid op-code only per active chip select.
//access... chip sel then,  op-code and a two-byte address, data
//Any number of bytes can be written sequentially and each byte is written to
//memory immediately after it is clocked in (after the 8th clock).
//The rising edge of /CS terminates a WRITE op-code operation.


//F-RAM Write Data Routine
// input   addr16
//         data8 * cnt
//
//uint32 spiTransmitData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint16 * srcbuff)
//FRAM_RD(uint16 addr, uint16* buf, uint16 cnt); //f-ram read data block
void FRAM_WR(uint16 Adr, uint8* buf, uint16 cnt)
{
    uint8 addH, addL;
    addH= Adr>>8;
    addL= Adr&0xff;
    if(cnt==0) return;
    dataconfig_FRAM.CS_HOLD = TRUE;             //Hold 'cs' low

    sendFRAM(&Op[fWRITE]);                      //send fram OP CODE
    //send addr 2 bytes, Adr+1, then Adr
    sendFRAM(&addH);                            //send fram ADDR HI
    sendFRAM(&addL);                            //send fram ADDR LO
    //after loop final byte sends with 'cs' normal format    
    cnt--;          
    while (cnt--)
        sendFRAM(buf++);
    //final byte, when done sets chip sel back high
    dataconfig_FRAM.CS_HOLD = FALSE;            //Normal 'cs' high
    sendFRAM(buf);
}

//FRAM Write to 16bit buffer source
void FRAM_WR_BUF16(uint16 FRAMAdr, uint16* ptr16, uint16 Wcnt)
{
    unsigned short* tB= (unsigned short*)tBuf;
    if(Wcnt==0) return;
    for (int i=0; i<Wcnt; i++)
      *tB++=  *ptr16++;
    //store to FRAM
    FRAM_WR_ENABLE();                       //WREN op-code, issued prior to Wr_Op
    FRAM_WR(FRAMAdr, (uint8*)&tBuf, Wcnt<<1);   //byte count FRAM
    FRAM_WR_DISABLE();
}

//FRAM Read from 16bit buffer source
void FRAM_RD_BUF16(uint16 FRAMAdr, uint16* ptr16, uint16 Wcnt)
{
    unsigned short* tB= (unsigned short*)tBuf;
    if(Wcnt==0) return;
    FRAM_RD(FRAMAdr, (uint8*)&tBuf, Wcnt<<1);   //byte count FRAM
    for (int i=0; i<Wcnt; i++)
       *ptr16++= *tB++;
    //store to FRAM
}

//F-RAM Read Data Routine
// input   addr16
// returns data8 * cnt
//using SPI3_CS0 'CS0' (V10) 
void FRAM_RD(uint16 rAdr, uint8* buf, uint16 cnt)
{
    uint8 addH, addL;
    if(cnt==0) return;
    addH= rAdr>>8;
    addL= rAdr&0xff;
    dataconfig_FRAM.CS_HOLD = TRUE;             //Hold 'cs' low
    sendFRAM(&Op[fREAD]);                       //send fram OP CODE
    sendFRAM(&addH); 
    sendFRAM(&addL); 
    //get data byte(s)
    //after loop final byte sends with 'cs' normal format    
    cnt--;          
    while (cnt--)
        {
        getFRAM(buf++);
        }
    dataconfig_FRAM.CS_HOLD = FALSE;            //Normal 'cs' high
    getFRAM(buf);
}



//Send one byte to FRAM (spi3_cs0) (V10) 8bit mode
//Chip Sel is not modified here
void  sendFRAM(uCHR* dat8)              //f-ram, read one char
{
    my_spiTransmitData(spiREG3, &dataconfig_FRAM, 1, dat8);
}

//Read one byte to F-RAM
//Chip Sel is not modified here
void getFRAM(uint8_t* buf)
{
    my_spiReceiveData(spiREG3, &dataconfig_FRAM, 1, buf);          
}


//Enable writes to FM25640B
//Write Enable Latch to be set.
//A flag bit in the status reg, WEL, indicates the state of the latch
//WEL=1 indicates that writes are permitted.
//Status Register Bits
//Bit    7    6   5   4   3    2    1    0
//Name WPEN   0   0   0   BP1  BP0  WEL  0
//
void FRAM_WR_STATUS(uint8_t cData)
{
    sendFRAM(&Op[fWREN]);                   //send fram OP CODE
    sendFRAM(&cData);                        //send fram OP CODE
}

//WPEN is high, the /WP pin controls write access to the status register.
//Read Status Register on FM25640B
uint8_t FRAM_RD_STATUS(void)
{
    unsigned char retVal;
    dataconfig_FRAM.CS_HOLD = TRUE;             //Hold 'cs' low
    sendFRAM(&Op[fRDSR]);                   //send fram OP CODE
    dataconfig_FRAM.CS_HOLD = FALSE;            //Normal 'cs' high
    getFRAM(&retVal);
    return retVal;
}

//WREN op-code must be issued prior to any Wr_Op
//
void FRAM_WR_ENABLE(void)
{
    sendFRAM(&Op[fWREN]);                   //send fram OP CODE
}


//Disable writes to FM25640B
void FRAM_WR_DISABLE(void)
{
    sendFRAM(&Op[fWRDI]);                   //send fram OP CODE
}


uint32 my_spiReceiveData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * destbuff)
//@brief Receives Data using polling method
//@param[in] spi           - Spi module base address
//@param[in] dataconfig_t  - Spi DAT1 register configuration
//@param[in] blocksize     - number of data
//@param[in] destbuff      - Pointer to the destination data (16 bit).
//@return flag register value.
//This function transmits blocksize number of data from source buffer using polling method.
//does not control chip sel line
{
/* USER CODE BEGIN (6) */
/* USER CODE END */
    uint32 Chip_Select_Hold = (dataconfig_t->CS_HOLD) ? 0x10000000U : 0U;
    uint32 WDelay = (dataconfig_t->WDEL) ? 0x04000000U : 0U;
    SPIDATAFMT_t DataFormat = dataconfig_t->DFSEL;
    uint8 ChipSelect = dataconfig_t->CSNR;

    while(blocksize != 0U)
    {
        if((spi->FLG & 0x000000FFU) !=0U)
        {
          break;
        }
       // tek Chip_Select_Hold not controlled in this function
        //tek if(blocksize == 1U)
        //{
        //   Chip_Select_Hold = 0U;
        //}
        /*SAFETYMCUSW 51 S MR:12.3 <APPROVED> "Needs shifting for 32-bit value" */
        spi->DAT1 = ((uint32)DataFormat << 24U) |
                    ((uint32)ChipSelect << 16U) |
                    (WDelay)            |
                    (Chip_Select_Hold)  |
                    (0x00000000U);
        /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Hardware status bit read check" */
        while((spi->FLG & 0x00000100U) != 0x00000100U)
        {
        } /* Wait */
        /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are only allowed in this driver" */
        *destbuff = (uint16)spi->BUF;
        /*SAFETYMCUSW 567 S MR:17.1,17.4 <APPROVED> "Pointer increment needed" */
        destbuff++;
        blocksize--;
    }
    return (spi->FLG & 0xFFU);
}


//original source 'SPI Driver Implementation File '
//my_spiReceiveData 'does not control chip sel line'

//my_spiTransmitData 'does not control chip sel line'
//@brief Transmits Data using polling method
//@param[in] spi           - Spi module base address
//@param[in] dataconfig_t  - Spi DAT1 register configuration
//@param[in] blocksize     - number of data
//@param[in] srcbuff       - Pointer to the source data ( 16 bit).
//@return flag register value.
//This function transmits blocksize number of data from source buffer using polling method.
uint32 my_spiTransmitData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * srcbuff)
{
    volatile uint32 SpiBuf,timer32=0;
    uint16 Tx_Data;
    uint32 Chip_Select_Hold = (dataconfig_t->CS_HOLD) ? 0x10000000U : 0U;
    uint32 WDelay = (dataconfig_t->WDEL) ? 0x04000000U : 0U;
    SPIDATAFMT_t DataFormat = dataconfig_t->DFSEL;
    uint8 ChipSelect = dataconfig_t->CSNR;

    while(blocksize != 0U)
    {
        if((spi->FLG & 0x000000FFU) !=0U)
        {
           break;
        }
       // tek Chip_Select_Hold not controlled in this function
       //if(blocksize == 1U)
       // {
       //    Chip_Select_Hold = 0U;
       // }
        /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are only allowed in this driver" */
        Tx_Data = *srcbuff;

        spi->DAT1 =  ((uint32)DataFormat << 24U) |
                     ((uint32)ChipSelect << 16U) |
                     (WDelay)           |
                     (Chip_Select_Hold) |
                     (uint32)Tx_Data;
        /*SAFETYMCUSW 567 S MR:17.1,17.4 <APPROVED> "Pointer increment needed" */
        srcbuff++;
        /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Hardware status bit read check" */
        while((spi->FLG & 0x00000100U) != 0x00000100U)
            {
            //breakout timer since this is used in notifications() now
            //tek Jun2017
            if (timer32>1000000)
                {
                blocksize=1;
                break;
                }
            } /* Wait */
        SpiBuf = spi->BUF;
        blocksize--;
    }
    return (spi->FLG & 0xFFU);
}





//apply scale factor to input data
void cFRAMrd(int fAddr, int short *Slope, int short *peds)
{
    int offset= fAddr;
    //store constants, addr range 0x030-047
    if((fAddr>=addr030) && (fAddr<=addr047))
        fAddr = const_FPGADACs0; 
    //store constants, addr range 0x430-447
    else if((fAddr>=addr430) && (fAddr<=addr447))
        fAddr = const_FPGADACs1; 
    //store constants, addr range 0x830-847
    else if((fAddr>=addr830) && (fAddr<=addr847))
        fAddr = const_FPGADACs2; 
    //store constants, addr range 0xC30-C47
    else if((fAddr>=addrC30) && (fAddr<=addrC47))
        fAddr = const_FPGADACs3; 
    else
        fAddr = const_FPGADACs0;        //use on invalid address(testing)
    
    offset -=addr030;                       
    offset <<=1;
    fAddr+= offset;
    FRAM_RD(fAddr, (uint8*)Slope, 2);    //reads 'cnt' bytes
    FRAM_RD(fAddr+0x30,(uint8*)peds,2);  //reads 'cnt' bytes
}



//DAC CONSTANTs correction table
//each addr 0,1,2...47 stores longword
//addr param must be valid
//Write 32 bit data to FRAM
void cFRAMwr(int fAddr, int short *Slope)
{
    FRAM_WR_ENABLE();                       //WEL bit clr on 'CS' rising
    //store constants, addr range 0x030-047
    if((fAddr>=addr030) && (fAddr<=addr047))
        fAddr= const_FPGADACs0+ fAddr;      //form dest address was3000
    //store constants, addr range 0x430-447
    else if((fAddr>=addr430) && (fAddr<=addr447))
        fAddr= const_FPGADACs1+ fAddr;      //form dest address was3000
    //store constants, addr range 0x830-847
    else if((fAddr>=addr830) && (fAddr<=addr847))
        fAddr= const_FPGADACs2+ fAddr;      //form dest address was3000
    //store constants, addr range 0xC30-C47
    else if((fAddr>=addrC30) && (fAddr<=addrC47))
        fAddr= const_FPGADACs3+ fAddr;      //form dest address was3000
    FRAM_WR(fAddr, (uint8*)Slope, 4);      //write 4 bytes
}


#define fDACcnt  24
//apply scale factor to input data
void cFRAMfill(int short *Slope, int short *Ped)
{
    //Slope base = 0x8000;
    //Offset bade= 0x0;
    for(int i=0,k=const_FPGADACs0+(fDACcnt<<1),j=const_FPGADACs0; i< fDACcnt; i++, j+=2,k+=2)
        {
        //WREN op-code must be issued prior to any Wr_Op  
        //WREN op-code must be issued prior to any Wr_Op  
        FRAM_WR_ENABLE();
        FRAM_WR(j, (uint8*)Slope, 2);   //16bit writes (wr 2 bytes) 
        FRAM_WR_ENABLE();
        FRAM_WR(k, (uint8*)Ped, 2);     //16bit writes (wr 2 bytes) 
        }
    
    //store constants, addr range 0x430-447
    for(int i=0,k=const_FPGADACs1+(fDACcnt<<1),j=const_FPGADACs1; i< fDACcnt; i++, j+=2,k+=2)
        {
        //WREN op-code must be issued prior to any Wr_Op  
        FRAM_WR_ENABLE();
        FRAM_WR(j, (uint8*)Slope, 2);   //16bit writes (wr 2 bytes) 
        FRAM_WR_ENABLE();
        FRAM_WR(k, (uint8*)Ped, 2);     //16bit writes (wr 2 bytes) 
        }

    //store constants, addr range 0x830-847
    for(int i=0,k=const_FPGADACs2+(fDACcnt<<1),j=const_FPGADACs2; i< fDACcnt; i++, j+=2,k+=2)
        {
        //WREN op-code must be issued prior to any Wr_Op  
        //WREN op-code must be issued prior to any Wr_Op  
        FRAM_WR_ENABLE();
        FRAM_WR(j, (uint8*)Slope, 2);   //16bit writes (wr 2 bytes) 
        FRAM_WR_ENABLE();
        FRAM_WR(k, (uint8*)Ped, 2);     //16bit writes (wr 2 bytes) 
        }

    //store constants, addr range 0xC30-C47
    for(int i=0,k=const_FPGADACs3+(fDACcnt<<1),j=const_FPGADACs3; i< fDACcnt; i++, j+=2,k+=2)
        {
        //WREN op-code must be issued prior to any Wr_Op  
        //WREN op-code must be issued prior to any Wr_Op  
        FRAM_WR_ENABLE();
        FRAM_WR(j, (uint8*)Slope,2);    //16bit writes (wr 2 bytes) 
        FRAM_WR_ENABLE();
        FRAM_WR(k, (uint8*)Ped, 2);     //16bit writes (wr 2 bytes) 
        }
}

extern short   g_signD16[3];    
//Equate Register Address to FRAM storage Address
int cFRAMparam2(int reg)              //register to FRAM Address
{
    if ((reg>=addr030) && (reg<=addr047)) 
        g_signD16[0]= const_FPGADACs0;
    else if ((reg>=addr430) && (reg<=addr447))
        g_signD16[0]= const_FPGADACs1;
    else if ((reg>=addr830) & (reg<=addr847))
        g_signD16[0]= const_FPGADACs2;
    else if ((reg>=addrC30) && (reg<=addrC47))
        g_signD16[0]= const_FPGADACs3;
    else
        return -1;      //flag bad data

    reg = reg&0xff;                     //0xff range now with fRam offset later
    //adjust base, addr30 will be a 0x00 offset in FRAM page
    reg= ((reg-addr030)<<1);            //2byte offset per addr
    reg +=g_signD16[0];                 //add fpga page constant 
return reg;
}

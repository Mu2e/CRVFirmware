//********************************************************
//  @file ZestETMI_SPI.c
//  Fermilab Terry Kiper 2016-2021
//
//  SPI Port Interface Module 'ZestETM1'
//  RM48 Micro Controller
//  RM48 Additions


// *************** SPI SELECT FUNCITONS **********************
// *************** SPI SELECT FUNCITONS **********************
#include "hal_stdtypes.h"
#include "spi.h"

#include "ZestETM1_SOCKET.h"

#include "ZestSock_TEK.h"
#include "ZestETM1_SPI.h"                   //mu2e Orange Tree SPI Interface
#include "fram.h"

extern  spiDAT1_t dataconfig_ZEST8;
extern  spiDAT1_t dataconfig_ZEST16;

void  sendZEST8(uint8* dat);                 //send byte

//Send one Word to ZEST
//Chip Sel is not modified here
void  sendZEST16(uint16* dat16)             //send word
{
    spiTransmitData16(spiREG5, &dataconfig_ZEST16, 1, dat16);
}


//ZEST sio port 16Bit Write
void ZEST_WR(uint16 Adr, uint16* buf)
{
    Adr= Adr | 0xCC00;
    uint16 d16;
  //d16= SwapWord((uint16*)*buf);
    d16= *buf;
    dataconfig_ZEST16.CS_HOLD = TRUE;             //Hold 'cs' low
    sendZEST16(&Adr);                             //send ADDR HI

    dataconfig_ZEST16.CS_HOLD = FALSE;            //Normal 'cs' high
  //sendZEST16((uint16*)buf);
    sendZEST16(&d16);
}


//ZEST sio port 8Bit Write
void ZEST_WR8(uint16 Adr, uint8* buf)
{
    Adr= Adr | 0xCC00;
    uint8 d8;
    dataconfig_ZEST16.CS_HOLD = TRUE;            //Hold 'cs' low
    sendZEST16(&Adr);                            //send ADDR HI
    
  //d16= SwapWord((uint16*)*buf);
    d8= *buf;
    dataconfig_ZEST16.CS_HOLD = FALSE;           //Normal 'cs' high
    sendZEST8(&d8);
}


//Read one Byte from ZEST SPI Port
void  sendZEST8(uint8* dat)                     //send byte
{
    spiTransmitData8(spiREG5, &dataconfig_ZEST8, 1, dat);
}


//Read one Word from ZEST SPI Port
void getZEST16(uint16* buf)
{
    spiReceiveData16(spiREG5, &dataconfig_ZEST16, 1, buf);          
}


//ZEST sio port read
void ZEST_RD(uint16 rAdr, uint16* buf)
{
    rAdr= rAdr | 0x7C00;
    dataconfig_ZEST16.CS_HOLD = TRUE;       //Hold 'cs' low
    //send addr
    sendZEST16(&rAdr); 
    //wait while busy on last send
    spiREG5->DAT1= 0x15FE0000;              //fixed value from spiReceiveData()
    while((spiREG5->FLG & 0x00000100U) != 0x00000100U)
      {  } //Wait
    //clear old data recd reg
    spiREG5->BUF;
    dataconfig_ZEST16.CS_HOLD = FALSE;      //Normal 'cs' high
    getZEST16(buf);
}


uint32 spiTransmitData8(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * srcbuff)
{
    volatile uint32 SpiBuf, TT;
    uint16 Tx_Data;
    uint32 Chip_Select_Hold = (dataconfig_t->CS_HOLD) ? 0x10000000U : 0U;
    uint32 WDelay = (dataconfig_t->WDEL) ? 0x04000000U : 0U;
    SPIDATAFMT_t DataFormat = dataconfig_t->DFSEL;
    uint8 ChipSelect = dataconfig_t->CSNR;
    while(blocksize != 0U)
    {
        if((spi->FLG & 0x000000FFU) !=0U)
        { break;}
       // tek  if(blocksize == 1U)  'chip select now controller by calling function
       // { Chip_Select_Hold = 0U; }
        /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are only allowed in this driver" */
        Tx_Data = *srcbuff;
        spi->DAT1 =  ((uint32)DataFormat << 24U) | 
                     ((uint32)ChipSelect << 16U) |
                     (WDelay)           |
                     (Chip_Select_Hold) |
                     (uint32)Tx_Data;
        TT =  ((uint32)DataFormat << 24U) | 
                     ((uint32)ChipSelect << 16U) |
                     (WDelay)           |
                     (Chip_Select_Hold) |
                     (uint32)Tx_Data;        
        srcbuff++;
        while((spi->FLG & 0x00000100U) != 0x00000100U)
          {  }         /* Wait */
        SpiBuf = spi->BUF;
        blocksize--;
    }
    return (spi->FLG & 0xFFU);
}


uint32 spiTransmitData16(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint16 * srcbuff)
{
    volatile uint32 SpiBuf;
    uint16 Tx_Data;
    uint32 Chip_Select_Hold = (dataconfig_t->CS_HOLD) ? 0x10000000U : 0U;
    uint32 WDelay = (dataconfig_t->WDEL) ? 0x04000000U : 0U;
    SPIDATAFMT_t DataFormat = dataconfig_t->DFSEL;
    uint8 ChipSelect = dataconfig_t->CSNR;
    while(blocksize != 0U)
    {
        if((spi->FLG & 0x000000FFU) !=0U)
        {  break; }
       // tek  if(blocksize == 1U)  'chip select now controller by calling function
       // { Chip_Select_Hold = 0U; }
        /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are only allowed in this driver" */
        Tx_Data = *srcbuff;
        spi->DAT1 =  ((uint32)DataFormat << 24U) |
                     ((uint32)ChipSelect << 16U) |
                     (WDelay)           |
                     (Chip_Select_Hold) |
                     (uint32)Tx_Data;
        srcbuff++;
        while((spi->FLG & 0x00000100U) != 0x00000100U)
          {  }         /* Wait */
        SpiBuf = spi->BUF;
        blocksize--;
    }
    return (spi->FLG & 0xFFU);
}


//spiReceiveData16 'does not control chip sel line'
//spiTransmitData16 'does not control chip sel line'
uint32 spiReceiveData16(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint16 * destbuff)
{
    uint32 Chip_Select_Hold = (dataconfig_t->CS_HOLD) ? 0x10000000U : 0U;
    uint32 WDelay = (dataconfig_t->WDEL) ? 0x04000000U : 0U;
    SPIDATAFMT_t DataFormat = dataconfig_t->DFSEL;
    uint8 ChipSelect = dataconfig_t->CSNR;
    while(blocksize != 0U)
    {
        if((spi->FLG & 0x000000FFU) !=0U)
        { break; }
       // tek  if(blocksize == 1U)  'chip select now controller by calling function
       // { Chip_Select_Hold = 0U; }
        spi->DAT1 = ((uint32)DataFormat << 24U) |
                    ((uint32)ChipSelect << 16U) |
                    (WDelay)            |
                    (Chip_Select_Hold)  |
                    (0x00000000U);
        while((spi->FLG & 0x00000100U) != 0x00000100U)
          {  }    /* Wait */
        *destbuff = (uint16)spi->BUF;
        destbuff++;
        blocksize--;
    }
    return (spi->FLG & 0xFFU);
}

// *************** SPI SELECT FUNCITONS END **********************
// *************** SPI SELECT FUNCITONS EBD **********************


//*******************************************************************************
// ZestETMI_SIO.c functions
// '_mu2e_Zest_SPI.h'
//*******************************************************************************

#ifndef _mu2e_Zest_SPI
#define _mu2e_Zest_SPI

//internal fuctions
uint32 spiTransmitData16(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint16 * srcbuff);
uint32 spiReceiveData16(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint16 * destbuff);
uint32 spiTransmitData8(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * destbuff);

//ZEST sio port read
void ZEST_RD(uint16 rAdr, uint16* buf);
void ZEST_WR(uint16 rAdr, uint16* buf);

#endif
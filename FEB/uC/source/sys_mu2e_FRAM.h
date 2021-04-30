#ifndef _fram
#define _fram

//F-RAM
#define fWRSR       0x01        //Write Status Register   0000_0001b
#define fWRITE      0x02        //Write Memory Data       0000_0010b
#define fREAD       0x03        //Read Memory Data        0000_0011b
#define fWRDI       0x04        //Write Disable           0000_0100b
#define fRDSR       0x05        //Read Status Register    0000_0101b
#define fWREN       0x06        //Set Write Enable Latch  0000_0110b

#define fs_WEL      0x02        //Write Enable Latch Bit
#define fs_WPEN     0x80        //FRAM, WPEN is high, /WP pin controls write access

void        FRAM_RD_BUF16(uint16 FRAMAdr, uint16* ptr16, uint16 Wcnt);
void        FRAM_WR_BUF16(uint16 DestAdr, uint16* buf, uint16 cnt);
void        FRAM_WR_ENABLE(void);   //WREN op-code must be issued prior to any Wr_Op
void        FRAM_WR_DISABLE(void);  //disable writes
void        FRAM_WR_STATUS(uint8_t cData);  //writes register
void        FRAM_WR(uint16 Adr, uint8* D8Buf, uint16 cnt);

uint8_t     FRAM_RD_STATUS(void);   //f-ram read 8 bit status register
void        FRAM_RD(uint16 addr, uint8* buf, uint16 cnt); //f-ram read data block

void        sendFRAM(uCHR*);        //f-ram, read one char
void        getFRAM(uCHR*);         //f-ram, write one char


void        cFRAMwr(int addr, int short*);
void        cFRAMrd(int fAddr, int short *, int short *);
void        cFRAMfill(int short*, int short*);
int         cFRAMparam2(int reg);       //register to FRAM Address


uint32  my_spiTransmitData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * srcbuff);
uint32  my_spiReceiveData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * destbuff);

#endif		/* _fram */

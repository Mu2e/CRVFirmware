//*******************************************************************************
// fram.c functions
// 'fram.h'
//*******************************************************************************

#ifndef _mu2e_fram
#define _mu2e_fram

//internal fuctions

uint32 my_spiTransmitData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * srcbuff);
uint32 my_spiReceiveData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * destbuff);

void        FRAM_RD_BUF16(uint16 FRAMAdr, uint16* ptr16, uint16 Wcnt);
void        FRAM_WR_BUF16(uint16 DestAdr, uint16* buf, uint16 cnt);
void        FRAM_WR_ENABLE(void);   //WREN op-code must be issued prior to any Wr_Op
void        FRAM_WR_DISABLE(void);  //disable writes
void        FRAM_WR_STATUS(uint8_t cData);  //writes register
uint8_t     FRAM_RD_STATUS(void);   //f-ram read 8 bit status register

void        FRAM_WR(uint16 Adr, uint8* D8Buf, uint16 cnt);  //fram writes data block
void        FRAM_RD(uint16 addr, uint8* buf, uint16 cnt);   //f-ram read data block

void        sendFRAM(unsigned char*);        //f-ram, read one char
void        getFRAM(unsigned char*);         //f-ram, write one char

//F-RAM
#define fWRSR       0x01                    //Write Status Register   0000_0001b
#define fWRITE      0x02                    //Write Memory Data       0000_0010b
#define fREAD       0x03                    //Read Memory Data        0000_0011b
#define fWRDI       0x04                    //Write Disable           0000_0100b
#define fRDSR       0x05                    //Read Status Register    0000_0101b
#define fWREN       0x06                    //Set Write Enable Latch  0000_0110b
#define fs_WEL      0x02                    //Write Enable Latch Bit
#define fs_WPEN     0x80                    //FRAM, WPEN is high, /WP pin controls write access


#define const_FPGADACs0  0x0A00                 //constants bias, led, afe dacs fpga0
#define const_FPGADACs1  0x0B00                 //constants bias, led, afe dacs fpga1
#define const_FPGADACs2  0x0C00                 //constants bias, led, afe dacs fpga2
#define const_FPGADACs3  0x0D00                 //constants bias, led, afe dacs fpga3
#define const_FPGADACend 0x0DFF                 //constants bias, led, afe dacs addr max

//FRAM  0x1000 repeats

//constants
#define addr030       0x030                     //dac addr begin fpga0
#define addr047       0x047                     //dac addr range end
#define addr430       0x430                     //dac addr begin fpga1
#define addr447       0x447                     //dac addr range end
#define addr830       0x830                     //dac addr begin fpga2
#define addr847       0x847                     //dac addr range end
#define addrC30       0xC30                     //dac addr begin fpga3
#define addrC47       0xC47                     //dac addr range end


#endif
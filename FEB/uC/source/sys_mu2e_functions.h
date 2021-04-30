#ifndef _sys_mu2e_functions
#define _sys_mu2e_functions


int         SET_SDADDR_WRx(int chipOffset, int HI, int LO);
int         SET_SDADDR_RDx(int chipOffset, int HI, int LO);

int         readAFE_Slow(u_32Bit fpga, int reg);
void        wrAFE_Slow(int fpga, int reg, int data);

uint16*     OneWireRead(int fpga, int TMP100, int ch);
int         OneWireTrigRead(void);

int         ldRam_DataFile(int, int, u_16Bit*, u_16Bit*);//FLASH Loader
void        dsav();                                 //read/store fpga regs to FRAM


uint16_t    movStrBlk32(unsigned short *dst, unsigned short *src, int cnt);
uint16_t    movStr16(unsigned short *src, unsigned short *dst, short int cnt);
uint16_t    movStr32(unsigned short *src, unsigned short *dst, short int cnt);

uint16      movStr16_NOICDEST(unsigned short *src, unsigned short *dst, long int cnt);
uint16      movStr16_NOICSRC(unsigned short *src, unsigned short *dst, long int cnt);
uint16      movStr16_NOINC(unsigned short *src, unsigned short *dst, long int cnt);

void        sockDisplay(int prt, int param1, char* paramPtr); //socket status reg 0x403,0x503,...
void        MemCopy32(unsigned long *dst, unsigned long *src, int bytes);

//fpga i/o
void        wr16FPGA (long offset, short d16);
int         rw16FPGA (long offset);

int         SET_SD_ADDR_RD1(int chipOffset, int HI, int LO);
int         SET_SD_ADDR_WR1(int chipOffset, int HI, int LO);

#endif		/* _sys_mu2e_functions */

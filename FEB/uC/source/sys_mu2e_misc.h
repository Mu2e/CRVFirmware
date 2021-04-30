#ifndef _sys_mu2e_misc
#define _sys_mu2e_misc


int     PgmFlash_FromSDramData(int);
int     LDF(int prt, int fpga, char* eBufB_Sock);  //rec data file, store in SDRAM
int     loadFLASH(int, int);
int     loadFLASH_NEW(int, char*);
int     eraseFLASH();
int     eraseFLASH_Sector(int saddr, int start, int prt);
int     fl_copyFPGA(int prt);
int     flashXFER(int, int, int);
int     flashStatus(int);
void    dmaConfigCtrlPacket1(uint32 sadd,uint32 dadd,uint32 dsize);
int     activeChannels(sPTR ptr, int trigs, int fpga1of4, int prt, int power);
void    FPGA_Trig_Cnts_Update();
int     fillEmptyEvnts(u_32Bit offset, int prt);
int     PGM_SDramFakeData(int prt);
int     ld_FAKE2sdRAM(int prt, int show);


int     wrFPGAMemory(int fpga, int d16);
int     PHY_LEN(int wait);

#define S29JL064J_SECTOR0 0             //Actual S29JL064J ADR=0x0 @Sector 0
#define S29JL064J_SECTOR41 (0x220000/2) //Actual S29JL064J ADR=0x110000 @Sector 41   
#define S29JL064J_SECTOR71 (0x400000/2) //Actual S29JL064J ADR=0x200000 @Sector 71   

#define SECT_FAKE_Sz    ((133-71)*2)    //Sector71-Sector133, holds fake data
//#define SECT_FAKE_Sz        48        //ERASE up to 3,145,680 bytes


#endif		/* _sys_mu2e_misc */

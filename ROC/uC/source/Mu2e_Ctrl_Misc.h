//*******************************************************************************
// Mu2e_Ctrl_Misc.c functions
// 'Mu2e_Ctrl_Misc.h'
//*******************************************************************************

#ifndef _mu2e_cntrl_misc
#define _mu2e_cntrl_misc

//internal fuctions
int     loadFLASH(int, int);
int     eraseFLASH();
int     eraseFLASH_Sector(int, int, int);
int     flashXFER(int, int);
int     flashStatus(int);
int     loadSpartan6_FPGA(int);
int     SockKeyWait(int, uint16, uint16*);

int     SET_SDADDR_WRx(int, int, int);
int     SET_SDADDR_RDx(int, int, int);

int     stab_PoolFunc(int,int);
void    dmaConfigCtrlPacket1(uint32 sadd,uint32 dadd,uint32 dsize);
int     ePHY_FIFO_LOAD(int sndLenW);
int     ePHY_SEND(int poePrt, int broadcast);
int     GTP1_Rec_TEST(void);

int     PHY_LOAD_DAQ_K28SEND_BCAST(int cmdType, int phyPort, sPTR xBuf, int wLen);
int     PHY_LOAD_DAQ_K28SEND_BCAST_MINI(int cmdType, int phyPort, sPTR xBuf, int wLen);


int     link_ID_Chk(int prt);
int     PHY_LOADER_POOL(int PrtPOE, int broadCast);
int     PHY_LOADER_POOL_BCAST(int PrtPOE, int broadCast);

int     EmptyAll_LVDS_FIFOs();
int     GTP1_Rec_Trigs();

#define S29JL064J_SECTOR0 (0x0)         //Actual S29JL064J ADR=0x0 @Sector 0
#define S29JL064J_SECTOR40 (0x220000/2) //Actual S29JL064J ADR=0x110000 @Sector 41   

#endif
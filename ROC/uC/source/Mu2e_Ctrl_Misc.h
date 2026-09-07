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

int     PHY_LOAD_DAQ_K28SEND_BCAST(int cmdType, int phyPort, sPTR xBuf, int wLen);
int     PHY_LOAD_DAQ_K28SEND_BCAST_MINI(int cmdType, int phyPort, sPTR xBuf, int wLen);
int     PHY_LOAD_DAQ_K28SEND_BCAST_MINI_TESTER(int cmdType, int PrtPOE, sPTR xBuf, int ubCnt);


int     link_ID_Chk(int prt);
int     PHY_LOADER_POOL(int PrtPOE, int broadCast);
int     PHY_LOADER_POOL_BCAST(int PrtPOE, int broadCast);

int     EmptyAll_LVDS_FIFOs();
int     GTP1_Rec_Trigs();

//int     SendFile_SrcSock(int prt, char* eBufB_Sock, int poePrt);

//adding storage for FEB image file to be downloaded via ROC to FEBs
//
int     SendFile_SrcSector71(int prt, int poePrt, int count, u_16Bit cksum, u_32Bit imageSz);

#define SectAddr71  (0x400000/2)        //Actual S29JL064J Word ADR=0x200000 "RFI 400000 to display" (upper BackUp image)                           
#define FileSectCntH   61               //each sector is 64K Byte

#define S29JL064J_SECTOR0 (0x0)         //Actual S29JL064J ADR=0x0 @Sector 0        "RFI 000000 to display" 
#define S29JL064J_SECTOR41 (0x220000/2) //Actual S29JL064J ADR=0x110000 @Sector 41  "RFI 220000 to display" 
#define S29JL064J_SECTOR71 (SectAddr71) //Actual S29JL064J ADR=0x400000 @Sector 71  "RFI 400000 to display" 


#endif

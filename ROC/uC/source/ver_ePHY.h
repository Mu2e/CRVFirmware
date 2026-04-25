//*******************************************************************************
// mu2e CRV Readout Controller 
// 'ver_ePHY.h'
// Fermilab Terry Kiper 2016-2021
//*******************************************************************************

#ifndef _VER_ePHY
#define _VER_ePHY


//DAQ command types 0x80 -- 8F
#define eCMD_DAQ_DY2       0x82 //dy.2
//#define eCMD_DAQ_DY2uB_TST 0x83   //dy.2 ubtest small non standard packets


//valid command range at the FEB rec coding is 0x70-0x7F for data echo via putBuf()
#define eCMD_BEG          0x70
#define eCMD_END          0x7F
#define eCMD71_CONSOLE    0x71   
#define eCMD72_STAB       0x72   
#define eCMD73_LDRAMINIT  0x73   
#define eCMD74_LDRAMFLUSH 0x74
#define eCMD75_CONSOLEBIN 0x75   
//#define eCMD79_LINK_DAQ   0x76   
//#define eCMD79_LINK_INIT  0x79   
#define eCMD76_BINARY_EOF 0x76
#define eCMD77_BINARY_DAT 0x77              //FPGA File Data type, Download packet, tek added Apr2025
#define ePayLdBinPreamble 0xA55A            //POE 'ePhy' binary download preamble header added to each packet
#define eECHO_ON          1
#define eECHO_OFF         0

#define ePayLdMax256    256             //payLoad Size Max 'PayLdMax' ie..buffer space
#define ePayLdSizMinWrds 32             //payLoad Size minimum (does not include packet src,dest,siz, ucHeader)
#define ePayLdMaxAndHdr ePayLdMax256+32 //add extra for packet ethernet overhead src,dest,len 'PayLdMaxAndHdr'
#define ePayLdSizLimit256  ePayLdMax256 //payLoad Size limit, 512 words, Make sure FEB has rec buffer this size !!!


#define PAC_CHLEN_MIN   64+4            //minimum packet length for standard network receivers, add checksum(4chars)
#define PAC_CHAR_LEN    48              //+14 char overHead hdr + 2chr terminators
#define ePHYCMDHDR_SZ_BYTS   (6+6+2+6 )
#define ePHYCMDHDR_SZ_WRDS  ((6+6+2+6)/2) 
#define ePHY_HDR_SZ     (sizeof(ePHY_HDR)-uCmdBufSz44)/2 

//status block buffers
#define eCMD72_MAX_WRDs174   174        //word size of STAT data pool
#define     sBLOCKSIZ   38              //ON FEBs
#define     sSMALLSIZ   22              //ON FEBs

#define DWNLD_3_VALID   0x0620              //2 BYTES fpga FEB image valid download
#define DWNLD_3_COUNT   0x0624              //4 BYTES fpga FEB image byte cnt
#define DWNLD_3_CSUM    0x0628              //2 BYTES fpga FEB image cksum


// ROC POE FEB Flash Download 
int PHY_LOADER_FLASH(char* paramPtr, int Sock, int PrtPOE, int echo, int broadCast, uint16 SndCnt);
int PHY_LOADER_EOF(char* paramPtr, int Sock, int PrtPOE, int echo, int broadCast, uint16 SndCntByts);
int PHY_LOADER_eCMD77(char* paramPtr, int Sock, int PrtPOE, int echo, int broadCast); 


//#define LVDS_tOUT5000  5000             //lvds rec data timeout, used in 'Mu2e_Cntrl_DAQ_LVDS.c'

int PHY_LOADER_CONSOLE(char*, int, int, int, int);
int PHY_LOADER_MINI_uB_REQ(int, int, int);
int assignLinkPort(int param1, int reset);
int linkInitALL(int prt);
int PHY_LOADER_LNK(int PrtPOE, int broadCast);
int PhyXmitBsy(int PrtPOE);
#endif
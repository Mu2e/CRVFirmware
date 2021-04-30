//*******************************************************************************
// mu2e CRV FEB ( this file matches FEB Controller) 'ver_ePHY.h'
// Fermilab Terry Kiper 2016-2020
//*******************************************************************************

#ifndef _VER_ePHY
#define _VER_ePHY

//handler mode, in interrupt on outside
//#define USE_INTRDAQ     

//DAQ command types 0x80 -- 8F
#define eCMD_DAQ_DY2       0x82   //dy.2
//#define eCMD_DAQ_DY2_MINI  0x83   //dy.2 ubtest small non standard packets



//valid command range at the FEB rec coding is 0x70-0x7F for data echo via putBuf()
#define eCMD_BEG          0x70
#define eCMD_END          0x7F
#define eCMD71_CONSOLE    0x71   
#define eCMD72_STAB       0x72           //'STATUS BLOCK' for getting Pool data block request 

#define eCMD73_LDRAMINIT  0x73   
#define eCMD74_LDRAMFLUSH 0x74
#define eCMD75_CONSOLEBIN 0x75   
//#define eCMD79_LINK_DAQ   0x76   
//#define eCMD79_LINK_INIT  0x79   

#define ePayLdMax 2048                   //payLoad Size Max 'PayLdMax' ie..buffer space
#define ePayLdMaxAndHdr ePayLdMax+64+16  //add extra for packet ethernet overhead src,dest,len 'PayLdMaxAndHdr',reserve
#define ePayLdSizLimit  ePayLdMax        //payLoad Size limit, 512 words, Make sure FEB has rec buffer this size !!!
#define ePayLdSizMinWrds 32              //payLoad Size minimum (does not include packet src,dest,siz)
#define ePHYCMDHDR_SZ    6+6+2+6 


#define PacBufs 10
#define PayLdMaxBytes ePayLdMaxAndHdr<<1   //add extra for packet ethernet overhead src,dest,len,uC_HDR

//PHY definitions
struct phyHeader
            {
            unsigned int  PacEna;
            unsigned int  PacActive;
            unsigned int  PacNext;
            unsigned int  PacCmdLen;
            unsigned int  PacType;
            unsigned int  PacBytCnt[PacBufs];
            unsigned char PacPayLd1024w[PacBufs][PayLdMaxBytes]; //plus eHdr overhead
            };

#define ePHY_PAC_MIN_Sz8W 8 //8 word minimum size

//PHY PACKET STATUS         //all daq packets, status/counter
struct phyPACSTAT
            {
            unsigned int 
            EMACTxBsy,
            EMACRxRdy,
            //uBunPerPac,
            //DropReq,
            //eSEND_LATCH,
            //RecPackCntsLat,
            RecPackCnts;
            uint64 SndTotWrds;
            uint64 uBunReqTot;
            };

//PHY PACKET CONTROL         //daq packets I/O Control
struct phyPAC_CNTRL
            {
            u_32Bit 
            pacSiz,
            uHdrIdx,
            uBunReqTot,
            RetSzBytes,
            gBunErr,
            g_idx,
            uBDatSz,
            uBunHI,
            uBunLO,
            g_DEBUG_MODE,
            unused;
            };


struct phyPAC_ERRORS
            {
            u_32Bit 
            gBunErr,
            TOflag,
            uBunReqErrs,
            uBunOvrErrs,
            //uBunOvrErrSoft,
            eREC_BSY_DROP,
            //xmtWrdCntMin,
            eRecHung;
            };



//MU2E CONTROLLER FPGA logic now handles 'PREAMBLE'
//uint8_t PREAMBLE[6U]= {0x55U, 0x55U, 0x55U, 0x55U, 0x55U, 0xD5U}; 

//************* Ethernet Packet Preamble *****************
// The preamble of an Ethernet packet consists of a 56-bit (seven-byte) pattern of 
// alternating 1 and 0 bits, allowing devices on the network to easily synchronize 
// their receiver clocks, which is followed by the SFD to mark a new incoming frame. 
// For Ethernet variants transmitting single bits instead of larger symbols, the 
// on-the-wire bit pattern for the preamble together with the SFD portion of the 
// frame is 10101010 10101010 10101010 10101010 10101010 10101010 10101010 10101011;
// [4]:sections 4.2.5 and 3.2.2 since octets are transmitted least-significant 
// bit first, the corresponding hexadecimal representation 
// is 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0xD5.



#endif
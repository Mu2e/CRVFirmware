//*******************************************************************************
// mu2e CRV Readout Controller 
// 'ver_io.h'
// Fermilab Terry Kiper 2016-2021
//
//*******************************************************************************

#ifndef _VER_IO
#define _VER_IO

#define MU2Ever   499
//code version, Major(1), Minor(00)

typedef	unsigned char   u_8Bit;                 //8-bit value
typedef	unsigned short  u_16Bit;                //16-bit value
typedef	unsigned int    u_32Bit;                //32-bit value

typedef unsigned int    uINT;
typedef unsigned long   uLNG;
typedef unsigned short  uSHT;
typedef unsigned char   uCHR;

typedef char*           volatile cPTR;
typedef unsigned short* volatile sPTR;
typedef unsigned short* snvPTR;
typedef unsigned long*  volatile lPTR;
typedef void*           volatile vPTR;


#define MAX_WORD_PER_FPGA (0x02000000*4)  //max data to allow on rdb commands

// shortcut macros for reading/writing to 32-bit memory mapped registers
#define REG32(addr) *(volatile unsigned int *)(addr)
#define REG16(addr) *(volatile unsigned short *)(addr)


/*
** usefull for bitmask operations
*/
#define BIT0  ( 0x00000001 )
#define BIT1  ( 0x00000002 )
#define BIT2  ( 0x00000004 )
#define BIT3  ( 0x00000008 )
#define BIT4  ( 0x00000010 )
#define BIT5  ( 0x00000020 )
#define BIT6  ( 0x00000040 )
#define BIT7  ( 0x00000080 )
#define BIT8  ( 0x00000100 )
#define BIT9  ( 0x00000200 )
#define BIT10 ( 0x00000400 )
#define BIT11 ( 0x00000800 )
#define BIT12 ( 0x00001000 )
#define BIT13 ( 0x00002000 )
#define BIT14 ( 0x00004000 )
#define BIT15 ( 0x00008000 )
#define BIT16 ( 0x00010000 )
#define BIT17 ( 0x00020000 )
#define BIT18 ( 0x00040000 )
#define BIT19 ( 0x00080000 )
#define BIT20 ( 0x00100000 )
#define BIT21 ( 0x00200000 )
#define BIT22 ( 0x00400000 )
#define BIT23 ( 0x00800000 )
#define BIT24 ( 0x01000000 )
#define BIT25 ( 0x02000000 )
#define BIT26 ( 0x04000000 )
#define BIT27 ( 0x08000000 )
#define BIT28 ( 0x10000000 )
#define BIT29 ( 0x20000000 )
#define BIT30 ( 0x40000000 )
#define BIT31 ( 0x80000000 )


//----------   RM48 Memory Map -------------
//Start         End           HDK
//Address       Address 
//0x0000 0000   0x002FFFFF    Flash
//0x0800 0000   0x0803FFFF    RAM
//0x0840 0000   0x0843 FFFF   RAM-ECC
//0x6000 0000   0x63FF FFFF   CS2 Async RAM
//0x6400 0000   0x67FF FFFF   CS3 Async RAM
//0x6800 0000   0x7BFF FFFF   CS4 Async RAM
//0x8000 0000   0x87FF FFFF   CS0 Sync SDRAM 


//ver TI RM48L board base address,  4-FPGA on board
#define fOffset         0x00000400                  //fpgas addr offset
#define fpgaBase0       0x60000000                  //FPGA  base address0   (uC hardware, chip sel 2)
#define fpgaBase1       0x60000800                  //FPGA  base address1   (uC hardware, chip sel 2)
#define fpgaBase2       0x60001000                  //FPGA  base address2   (uC hardware, chip sel 2)
#define fpgaBase3       0x60001800                  //FPGA  base address3   (uC hardware, chip sel 2)
#define flashBase       0x64000000                  //FLASH base addr       (uC hardware, chip sel 3)
#define ZestETM1        0x68000000                  //ETHERNET base address (uC hardware, chip sel 4)

//fpga short pointers offsets, this value doubles when used 
#define fPtrOffset2     0x0400
#define fPtrOffset3     0x0800
#define fPtrOffset4     0x0C00

#define fPtrSDramCnt    0x0009                      //SDram count reg offset

#define pFLASHbase    (sPTR) flashBase              //short ptr, parallel flash base address
#define f_Addr_CSR    (sPTR)(fpgaBase0+ 0x00)        //FPGA CSR reg
//#define CSR0         *(sPTR)(fpgaBase0+ (0x00))      //CSR (FPGA0)


//Harmonica Jack RJ45 LED registers
#define csLED1       *(sPTR)(fpgaBase0+ (0x10*2))   //LEDs RJ45 1of6
#define csLED2       *(sPTR)(fpgaBase0+ (0x11*2))   //LEDs RJ45 2of6
#define csLED3       *(sPTR)(fpgaBase0+ (0x12*2))   //LEDs RJ45 3of6
#define csLED4       *(sPTR)(fpgaBase0+ (0x13*2))   //LEDs RJ45 4of6
#define csLED5       *(sPTR)(fpgaBase0+ (0x14*2))   //LEDs RJ45 5of6
#define csLED6       *(sPTR)(fpgaBase0+ (0x15*2))   //LEDs RJ45 6of6


#define GTP_RX_CSR   *(sPTR)(fpgaBase0+ (0x02*2))   //GTP RX FIFO CSR

#define GTP0_RQ_PAC0D *(sPTR)(fpgaBase0+ (0x0D*2))  //GTP0 Sorted Data Req Packet Buffer
#define GTP0_RQ_CNT0E *(sPTR)(fpgaBase0+ (0x0E*2))  //GTP0 Sorted Data Req Packet Buffer count

#define GTP0_DCS_PAC0D *(sPTR)(fpgaBase0+ (0x50*2)) //GTP0 Sorted DCS Packet Buffer
#define GTP0_DCS_CNT0E *(sPTR)(fpgaBase0+ (0x51*2)) //GTP0 Sorted DCS Packet Buffer count

#define GTP0_PREAM   *(sPTR)(fpgaBase0+ (0x1A*2))   //GTP0 PREAMBLE 
#define GTP1_PREAM   *(sPTR)(fpgaBase0+ (0x1B*2))   //GTP1 PREAMBLE 
#define GTP0_PAYLD   *(sPTR)(fpgaBase0+ (0x1C*2))   //GTP0 PAYLOAD 
#define GTP1_PAYLD   *(sPTR)(fpgaBase0+ (0x1D*2))   //GTP1 PAYLOAD 
#define GTP0_XMIT    *(sPTR)(fpgaBase0+ (0x1E*2))   //GTP0 CKSUM/XMIT
#define GTP1_XMIT    *(sPTR)(fpgaBase0+ (0x1F*2))   //GTP1 CKSUM/XMIT
#define GTP0_RECFIFO *(sPTR)(fpgaBase0+ (0x20*2))   //GTP0 RECEIVE FIFO
#define GTP1_RECFIFO *(sPTR)(fpgaBase0+ (0x21*2))   //GTP1 RECEIVE FIFO
#define GTP0_DCS_TX  *(sPTR)(fpgaBase0+ (0x53*2))   //GTP0 DCS BUFFER

#define fLNK_RECFIFO *(sPTR)(fpgaBase0+ (0x27*2))   //FPGA Link Rec FIFO status


#define LNG_SDR_WR   *(lPTR)(fpgaBase0+ (0x02*2))   //SDram read address long (32bit ptr)
#define SDramWrHI    *(sPTR)(fpgaBase0+ (0x02*2))   //SDram HIGH write address register in FPGA
#define SDramWrLO    *(sPTR)(fpgaBase0+ (0x03*2))   //SDram LOW  write address register in FPGA

#define LNG_SDR_RD   *(lPTR)(fpgaBase0+ (0x04*2))   //SDram read address long (32bit ptr)
#define SDramRdHI    *(sPTR)(fpgaBase0+ (0x04*2))   //SDram HIGH write address register in FPGA
#define SDramRdLO    *(sPTR)(fpgaBase0+ (0x05*2))   //SDram LOW  write address register in FPGA

#define SDR_RD16SWP  *(sPTR)(fpgaBase0+ (0x06*2))   //SDram uC read 16bit as byte swapped data
#define SDR_RD16     *(sPTR)(fpgaBase0+ (0x07*2))   //SDram uC read/write 16bit normal format

#define ACT_PORTS_HI *(sPTR)(fpgaBase0+ (0x08*2))   //Active Port Dectected (rec'd clock) upper poe ch 17-14
#define ACT_PORTS_LO *(sPTR)(fpgaBase0+ (0x09*2))   //Active Port Dectected (rec'd clock) lower poe ch 16-00

//#define CLKFANOUT    *(sPTR)(fpgaBase0+ (0x3A*2))   //CDCUN1208LPRHBR read/write 16bit
#define sUPTIMEHI    *(sPTR)(fpgaBase0+ (0x6C*2))   //up time HI
#define sUPTIMELO    *(sPTR)(fpgaBase0+ (0x6D*2))   //up time LO

#define sTST_CNT_HI  *(sPTR)(fpgaBase0+ (0x34*2))   //32 bit test counter HIGH 16
#define sTST_CNT_LO  *(sPTR)(fpgaBase0+ (0x35*2))   //32 bit test counter LOW 16


// shortcut macros for reading/writing to 32-bit memory mapped registers
#define REG32(addr) *(volatile unsigned int *)(addr)
#define REG16(addr) *(volatile unsigned short *)(addr)

//#define off_sdWRH       (0x002)                     //sdWR High Addr offset from fpga base
//#define off_sdWRL       (0x003)                     //sdWR Low Addr offset from fpga base
//#define off_MIG         (0x009)                     //MIG offset from fpga base

//#define PHY_XMT_ENA     *(sPTR)(fpgaBase1+ (0x00E*2))   //ePHY XMIT CH 0-7 ENABLE AS BIT0-7 (FF==ALL ENABLED)
//#define PHY_FIFO        *(sPTR)(fpgaBase1+ (0x011*2))   //ePHY XMIT DATA FIFO WR
//#define PHY_XMIT_PAC    *(sPTR)(fpgaBase1+ (0x012*2))   //ePHY XMIT STATUS R/W (read 1=empty), (wr 1 to xmit)
//#define PHY_FIFOCNT     *(sPTR)(fpgaBase1+ (0x013*2))   //ePHY BROADCAST FIFO WORD COUNT RD

//offset address to ethernet phy regs
#define oPHY0E_XMTMASK  (0x00E)                     //ePHY XMIT CH 0-7 ENABLE MASK AS BIT0-7 (FF==ALL ENABLED)
#define oPHY11_BCAST_FILLFIFO (0x011)               //ePHY XMIT BROADCAST FIFO DATA
#define oPHY12_XMIT     (0x012)                     //ePHY XMIT STATUS R/W (RD 1=empty), (WR 1 to xmit)
#define oPHY13_WRDCNT   (0x013)                     //ePHY BROADCAST FIFO WORD COUNT RD
#define oPHY16_RXSTAT   (0x016)                     //PHY 8-Port Rec data Status (bit7-0, 1=empty)
#define oPHY18_1F_RXCNT (0x018)                     //PHY Rec Word Cnt port0 (1of8)

#define oPHY20_27_RXDAT (0x020)                     //PHY Rec data port0 (1of8)
#define oFM30_37_RXDAT  (0x030)                     //lvds RX data 0x30-37

//offset to lvds fm reg
                                                    //oFMData30[] array LVDS rx bufs
                                                    //oFMWrdCnt38[] array LVDS word cnt
#define oFMStat40       (0x040)                     //LVDS FM Status reg offset
#define oFMPerr41       (0x041)                     //LVDS FM RX Port Parity Error (WR BIT8 up==ClrBuffer)
#define FMRstBit8       (0x100)                     //LVDS FM RX Port clear rec buffer 'oFMPerr39'

#define POE01  1                                    //1st ch of fpga1 for board cast packet purpose
#define POE09  9                                    //1st ch of fpga2 for board cast packet purpose
#define POE17  17                                   //1st ch of fpga3 for board cast packet purpose

//ePHY RX Status Bits
#define ePHY_RX_STA_416 *(sPTR)(fpgaBase1+(0x16*2)) //ePHY Rx buffer Non empty Status ports 1of8
#define ePHY_RX_STA_816 *(sPTR)(fpgaBase2+(0x16*2)) //ePHY Rx buffer Non empty Status ports 1of8
#define ePHY_RX_STA_C16 *(sPTR)(fpgaBase3+(0x16*2)) //ePHY Rx buffer Non empty Status ports 1of8


#define SDramWrHI1   *(sPTR)(fpgaBase1+ (0x02*2))   //SDram HIGH write address register in FPGA1
#define SDramWrLO1   *(sPTR)(fpgaBase1+ (0x03*2))   //SDram LOW  write address register in FPGA1
#define SDramWrHI2   *(sPTR)(fpgaBase2+ (0x02*2))   //SDram HIGH write address register in FPGA2
#define SDramWrLO2   *(sPTR)(fpgaBase2+ (0x03*2))   //SDram LOW  write address register in FPGA2
#define SDramWrHI3   *(sPTR)(fpgaBase3+ (0x02*2))   //SDram HIGH write address register in FPGA3
#define SDramWrLO3   *(sPTR)(fpgaBase3+ (0x03*2))   //SDram LOW  write address register in FPGA3


//#define SDramRdHI1   *(sPTR)(fpgaBase1+ (0x04*2))   //SDram HIGH read address register in FPGA1
//#define SDramRdLO1   *(sPTR)(fpgaBase1+ (0x05*2))   //SDram LOW  read address register in FPGA1
//#define SDramRdHI2   *(sPTR)(fpgaBase2+ (0x04*2))   //SDram HIGH read address register in FPGA2
//#define SDramRdLO2   *(sPTR)(fpgaBase2+ (0x05*2))   //SDram LOW  read address register in FPGA2
//#define SDramRdHI3   *(sPTR)(fpgaBase3+ (0x04*2))   //SDram HIGH read address register in FPGA3
//#define SDramRdLO3   *(sPTR)(fpgaBase3+ (0x05*2))   //SDram LOW  read address register in FPGA3

//feb readout fpga1
#define f1_RD16SWP   *(sPTR)(fpgaBase1+ (0x06*2))   //SDram uC read 16bit as byte swapped data
#define f1_RD16      *(sPTR)(fpgaBase1+ (0x07*2))   //SDram uC read 16bit as byte
#define fpga1binSzHi *(sPTR)(fpgaBase1+ (0x6A*2))   //word cnt via phy high
#define fpga1binSzLo *(sPTR)(fpgaBase1+ (0x6B*2))   //word cnt via phy low

//feb readout fpga2
#define f2_RD16SWP   *(sPTR)(fpgaBase2+ (0x06*2))   //SDram uC read 16bit as byte swapped data
#define f2_RD16      *(sPTR)(fpgaBase2+ (0x07*2))   //SDram uC read 16bit as byte
#define fpga2binSzHi *(sPTR)(fpgaBase2+ (0x6A*2))   //word cnt via phy high
#define fpga2binSzLo *(sPTR)(fpgaBase2+ (0x6B*2))   //word cnt via phy low

#define ePHY301_BCAST_DATA *(sPTR)(fpgaBase0+(0x301*2))//ePhy link global_24 data (fifo) loader
#define ePHY302_BCAST_XMIT *(sPTR)(fpgaBase0+(0x302*2))//ePhy link global_24 data (broadcast) xmit



//reg bit assignments
#define FMRXENA     BIT3


//error triggers
#define TrgSinBitErr (unsigned int *) 0xF00803F0
#define TrgDouBitErr (unsigned int *) 0xF00803F8

//new stuff mu2e controller End

//FRAM
#define hdrSize          15                 //QIE HEADER SIZE in words
#define DWNLD_VALID      0xAA55
#define PASSCODE         123                //simple password for param entry

#define fram_FPGA0_REG   0x0000             //store fpga regs chip 0
#define fram_FPGA1_REG   0x0100             //store fpga regs chip 1
#define fram_FPGA2_REG   0x0200             //store fpga regs chip 2
#define fram_FPGA3_REG   0x0300             //store fpga regs chip 3
#define fram_serNUMB     0x0400             //controller serial number
#define fram_cntrlNUMB   0x0402             //controller daq number 1-nn
#define fram_Spill_REGs  0x0410


#define FRAM_AddrNet500 0x0500              //network stuff store byte addr base
#define DWNLD_1_VALID   0x0600              //2 BYTES fpga1 valid download
#define DWNLD_1_COUNT   0x0604              //4 BYTES fpga1 byte cnt
#define DWNLD_1_CSUM    0x0608              //2 BYTES fpga1 cksum
#define SERIAL_NUMB_ADR 0x060A              //2 BYTES board serial number
#define DWNLD_2_VALID   0x0610              //2 BYTES fpga2 valid download
#define DWNLD_2_COUNT   0x0614              //4 BYTES fpga2 byte cnt
#define DWNLD_2_CSUM    0x0618              //2 BYTES fpga2 cksum
#define fram_NERROR_CNT 0x061A              //2 BYTES nError counter


//F-RAM
#define fWRSR           0x01                //Write Status Register   0000_0001b
#define fWRITE          0x02                //Write Memory Data       0000_0010b
#define fREAD           0x03                //Read Memory Data        0000_0011b
#define fWRDI           0x04                //Write Disable           0000_0100b
#define fRDSR           0x05                //Read Status Register    0000_0101b
#define fWREN           0x06                //Set Write Enable Latch  0000_0110b
#define fs_WEL          0x02                //Write Enable Latch Bit
#define fs_WPEN         0x80                //FRAM, WPEN High, /WP pin controls wr access


// Flag Bits for Register (genFlag )
#define ADC_Trig        0x0001              //flag as ADC triggered
#define ADC_Rdy         0x0002              //flag as ADC data is avail
#define uTimeOut        0x0004              //uS timer flag 0==timeout
#define hDelay          0x0008              //delay flag nHet Delay function
#define POE_CHECK_DUE   0x0010              //POE CHECK DUE via systick counter
#define ZEST_ETM1_OK    0x0020              //Orange Tree gigabit ethernet active
#define DAQREQ_2FEB     0x0040              //FEB DAQ data request
#define DAQuB_Trig_OLD  0x0080              //uBunch trigger mode flag
#define ECHO_ACT_PORT   0x0100              //echo link poe port number with newline prompt
//#define POOL_Active     0x0200               //data req for link check active
#define ZEST_ETM1_INTR  0x0400              //Orange Tree gigabit ethernet intr check
#define Check_DCS       0x0800              //Check for DCS packages
#define PoolReqNow      0x1000              //Start poe link feb finder req data
#define PoolReqGetData  0x2000              //Req Time out FEB get pool data now
#define ID_ReqNow       0x4000              //Start poe link feb id req data
#define DAQuB_Trig_NEW  0x8000              //uBunch trigger mode flag 1 (for testing short packets)


#define ID_RegTime      4                   //link id number check time in Secs
#define ReqPoolTime     30000               //link req pooled data time in mSecs
#define GetPoolTime     (ReqPoolTime+500)   //link get pooled data after delay in mSecs


// Flag Bits for Register (iFlag)
#define CONFIGFAIL      0x0001              //FPGA config status
#define OTREE_CONFIG    0x0002              //ORG TREE Zest board config error
#define iNoPrompt       0x0004              //echo new line prompt
#define iPHY_BINMODE   0x0008              //Sending binary file on saved data structure

//flash variables
#define adr555 (0x555 << 1)                 //flash chip command codes
#define adr2aa (0x2aa << 1)                 //flash chip command codes
#define VALID           0xaa55              //valid data mask for saved data structure

//sector to erase in parallel flash
#define SECTORES        40     

#define putchar     __putchar
#define cmdbufsiz       128                 //uart1 command line buffer size
#define CmdSiz80         80                 //limit repeat command line size

#define     UART scilinREG                  //lin module configured as UART
#define     USB_inBufSz 256
#define     ADC_CHS  4                      //4 ACTVIE channels       


//#define PROMPT_ER   SendStrTxI("FPGA_Error>");  //err prompt

//LINK INITs
#define CLR_ENTRY       0xFF

// ASCII Constants
#define BACKSP          0x08
#define ACK_            0x06
#define NACK_           0x15
#define SP              0x20
#define DELETE          0x7F

//NUM_SOCKETS                               //defined in ZestETMI_SOCKET.h
#define tty             232                 //i/o stream port uart0
#define Sock0           0                   //i/o stream port socket 0
#define Sock1           1                   //i/o stream port socket 1
#define Sock2           2                   //i/o stream port socket 2
#define Sock3           3                   //i/o stream port socket 3
#define DCS             8                   //DCS interface
#define NoPmt1          9                   //port adj to prevent prompts after certain commands
#define eCmdBufBytSiz   128                 //socket(s) command line buffer size

//socket xt/rt byte siz
#define TXRX_BytSiz1600  1600               //buffer max byte count (SOCKET SIZE)

#define     LED_DRIVERS 6                   //RJ45 CONNECTOR LED DRIVER PORTS '74LVC595APW'
#define     POEChipCnt  6                   //POE CHIPs 'LTC4266A' per board
#define     POECHs      4                   //Channels per'LTC4266A' chip
#define     POECHsALL   POEChipCnt*POECHs   //Channels per'LTC4266A' chip * chip count


#define InBufSiz_512  512
#pragma pack(4)                                 //force 32bit boundary on all types
struct vB {
            //Warning, RdPtr,WrPtr caused pgm resets when assigned volatile
            volatile int RdPtr;                          
            volatile int WrPtr;
            volatile int Cnt;
            char prompt[20];
            char Buf[InBufSiz_512];
        };



#pragma pack(4)                                 //force 32bit boundary on all types
struct uBuns {
            u_32Bit uDly;                          
            u_16Bit Port;
            u_16Bit Mode;
            u_16Bit SmPacMode;
            u_16Bit Flag;
            u_16Bit errFLAG;                //ephy bcast link bsy error
            u_16Bit errRESYNC;
        };

#pragma pack(2)                             //force 16bit boundary on all types
struct netinfo_s {
            unsigned char ipAddr[4];        //ip
            unsigned char gateWay[4];       //gate   (network)
            unsigned char netMask[4];       //mask
            u_16Bit sTelnet0;               //sock0 port number
            u_16Bit sTelnet1;               //sock1 port number
            u_16Bit sTelnet2;               //sock2 port number
            u_16Bit sTelnet3;               //sock3
            u_16Bit Spare1;                 //Spare
            u_16Bit Spare2;                 //Spare
            u_16Bit Spare3;                 //Spare
            u_16Bit UpDate;                 //UpDate flag to Update Orange Tree IP
            u_16Bit Valid;                  //data saved flag
            u_16Bit unused;                 //required
           };


//Ethernet/Wiznet power up defaults 
//will be overwritten if user FLASH has valid data for these settings
static const struct netinfo_s defNetInfo = {
            {131, 225, 53, 82},             //local IP  default null setup 'HLSWH4'
            {131, 225, 56, 200 },           //Gateway Wilson Hall
            {255, 255, 255, 00 },           //Mask for all
            5000,                           //telnet0 port char based
            5001,                           //telnet1 port char based
            5002,                           //telnet2 port char based
            5003,                           //telnet3 port char based
            1,                              //spare1
            2,                              //spare1
            3,                              //spare2
            0,                              //UdDate Flag
            0,                              //Valid Download Flag
            0                               //required, quick fix
        };



#define numbMsg 60
#define MsgBytMax 32

//PHY definitions
struct phyHeader
            {
            unsigned int  enable;
            unsigned int  msgCnt;
            unsigned int  msgBytes[numbMsg];
            unsigned char msgStr[numbMsg][MsgBytMax];
            };


//CONFIGURING THE Si5338 WITHOUT DEFAULT ADDR= 0x70
#define own_add_w 0x10
#define own_add_r 0x11


#define pool_POE_VLT    33                  //array location to hold 16bit poe volts
#define pool_POE_CUR    34                  //array location to hold 16bit poe current
#define ltc_RSTPB       0x1A                //LTC4266 port reset register
#define ltc_OnOffPB     0x19                //LTC4266 port power on/off 

//TDC Spill Header Words
#define cCntlHdrSizWrd (8)                  //main controller header size in words
#define cCntlHdrSizByt (8*2)                //main controller header size in bytes

#pragma pack(4)                             //force 16bit boundary on all types
//warning on pack(4),dont mix 32 and 16 words if you dont want unused memory words
struct Controller_tdcHdr
            {
            u_16Bit tSpilWrdCntL;           //(all FPGAs plus hdr size of 8 words)
            u_16Bit tSpilWrdCntH;           //32 bit value tSpilWrdCntX is endian swapped
            u_16Bit tSpilTrgCntL;           //32bit as low/high words
            u_16Bit tSpilTrgCntH;           //32bit value tSpilTrgCntX is endian swapped
            u_16Bit tSpilCyc;
            u_16Bit tMask,
                    tID,
                    tStatus,
                    tbitFlags;
            };


//system timer in this structure and incremented in file notification.c, function rtiNotification()
struct msTimers{
           u_32Bit volatile  g_timeMs,
                    g_SockMs,
                    g_SockIntrSec,
                    gBusy_mSec,
                    g_wTicks,
                    tdcSpillGateCnt,
                    SpillGateTimeout;
                    //FEB_Ld_toutSec;
            };


//system timer in this structure and incremented in file notification.c, function rtiNotification()
struct sLVDS {
           u_32Bit volatile  IDChkSec, 
                    IDPrt,
                    PoolChkmSec,
                    PoolMode,
                    PoolReqType;
            };



                    
//FRAM FPGA stored register count
#define FPGAregCount   0x72                 //fpga reg count to store in FRAM

//Misc data to store in FRAM
#define Ser_Cntrl_ByteSz     4              //size of SetUp, 2 16bit words
struct SetUpInfo_s {
            u_16Bit serNumb;                //board serial number
            u_16Bit CntrlNumb;              //DAQ controller board number
           };


#define SpillInfo_sByteSz    2             //size of SetUp 'fram_Spill_REGs=FRAM addr'
struct SpillInfo_s {
            u_16Bit SpillGateTimeout;       //tdc SpillGateTimeout in mSec
           };


//sendbin() control structure
struct blocksnd {
            uLNG gSndWordCnt;
            lPTR gSndSrc;
            int  gSndPrt;                   //port number
            int  gDMA_Fpga2Mem;
            int  gDMA_Mem2Wiz;
            int  gDMA_TimDlys[5];
            int  volatile gBusy;
            int  gTestMode;
            };


//UDP definitions
#pragma pack(4)                             //force 16bit boundary on all types
struct ePHY_STRUCT
            {
            unsigned int volatile mode;
            unsigned int volatile xSize;
            unsigned int spare;
            };

#define RST280  0x11

//local sram, Stored to FRAM
struct uC_Store_FRAM  
            {
            u_16Bit nErrorCnt;              //ErrLatch nError counter
            };

//local sram
struct uC_Store
            {
            u_32Bit DwnLd_sCNT;             //4 bytes
            u_16Bit DwnLd_sSUM;             //2 bytes
            };

//local storage of SOCKET download programming of FLASH
struct uSums {
              u_32Bit DwnLd_sCNT;               //4 bytes count
              u_16Bit DwnLd_sSUM;               //2 bytes chksum
              u_16Bit FL_SOCK_CHKSUM;           //2 bytes FLASH load using socket checked sum
              u_32Bit FL_SOCK_CHKSIZE;          //4 bytes FLASH load using socket checked filesize
};



  
//fpga reg setup structure (defaults, flash and restore)
typedef struct FPGA_Registers{
   unsigned short Addr;
   unsigned short RegCnt;
   unsigned short Value[8];
} FPGA_RegS;


typedef struct FebDCSRply {
    unsigned short cnt;
    int add;
    char val[4];
} sFebDCSRply;

//ePHY LINK Board numbering for 'Happy Bus
typedef struct HappyBusReg{
   int volatile CmdLenB,
   PoeBrdCh,
   CmdType,
   CntRecd,   
   //Active,                      //flags the command type
   Socket,
   SavePrt,  
   FMRecvErr,
   Stat,                        //bit 2 download BIN file to FEB option active
   SlowReply;                   //slow reply flag
   
   volatile unsigned int WaitCnt, //uses LVDS_TIMEOUT value
   FM_DAT,
   FM_STA,
   FM_PAR,
   ePHY_DATAVAIL_BIT;       //ethernet phy status 1of8 BIT0-BIT7
   sPTR ePHY_SndFIFO;       //ethernet phy broadcast pointer
   sPTR ePHY_CH_ENA;        //ethernet phy 1of8 chan enable
   sPTR ePHY_SndPAC;        //ethernet phy send ptr
   sPTR ePHY_RecFIFO;       //ethernet phy rec data reg ptr, 1of8
   sPTR ePHY_STAT;          //ethernet phy status reg ptr, 1of8
} sHappyBusReg;




#pragma pack(4)             //force 16bit boundary on all types

//ePHY LINK Board numbering for 'Happy Bus
typedef struct sLVDS_ePHY_REG{
   unsigned int FM40_STAp;          //lvds FM link status
   unsigned int FM41_PARp;          //lvds FM link parity
   unsigned short* FM30_DATp;       //lvds FM link rx data port
   unsigned int ePHY_BIT;           //ethernet phy status 1of8 BIT0-BIT7
   
   unsigned short* ePHY18_RECCNTp;  //ethernet phy rec word cnt, 1of8
   unsigned short* ePHY20_RECBUFp;  //ethernet phy rec data reg ptr, 1of8
   unsigned short* ePHY11_BCAST_FILLFIFOp; //ethernet phy broadcast pointer
   unsigned short* ePHY0E_XMSKp;    //ethernet phy broadcast pointer
   unsigned short* ePHY12_XMITp;    //ethernet phy broadcast pointer
   unsigned short* ePHY16_STAp;     //ethernet phy status reg ptr
} sLVDS_ePHY;


//time of day struct
struct time  {                                  //24 hour time
            uSHT    sec,
                    min,
                    hr,
                    day;
            uLNG    totalSec;
            };



//testing internal fpga logic uBunch xmits
struct ky {
    int     mode;
    uSHT    k28y2[40];
    };



//ePHY LINK Board numbering for 'Happy Bus
//ePHY packet format

#define uCmdBufSz44  44
struct ePHYregS{
   unsigned short e_SRC[3];     //phy dest      6 bytes
   unsigned short e_DST[3];     //phy src       6 bytes
   unsigned short e_PAYLDLEN;   //phy payload   2 bytes
   unsigned short u_CMDLEN;     //uHdr cmdlen   2 bytes
   unsigned short u_BRDNUM;     //uHdr brdNum   2 bytes
   unsigned short u_CMDTYP;     //uHdr cmdType  2 bytes
   char     u_CmdBuf[uCmdBufSz44];//command string buffer
};


#define uBunSz256     256

struct ePHYdaqS{
   unsigned short e_SRC[3];      //phy dest      6 bytes
   unsigned short e_DST[3];      //phy src       6 bytes
   unsigned short e_PAYLDLEN;    //phy payload   2 bytes
   unsigned short u_CMDLEN;     //uHdr cmdlen   2 bytes
   unsigned short u_BRDNUM;     //uHdr brdNum   2 bytes
   unsigned short u_CMDTYP;     //uHdr cmdType  2 bytes
   unsigned short u_uBunchBuf[uBunSz256];  //int data size
};

#define PacSize 256+10
//
//Bin file download struct
struct ePHY_BinFile{
    unsigned short CMDTYP;           //xmit data cmdType 1 word 'eCMD77_FPGA_CNTRL'
    unsigned short WrdCnt;           //xmit data words to send
    unsigned short binBuf[PacSize];  //xmit data buffer
};

            
//*********** HET1 OUTPUTS *************
//*********** HET1 OUTPUTS *************
            
//FLASH RESET       (E3) HET1_11 
#define FlashRst_LO    hetREG1->DCLR=  BIT11;   //500nS low pulse, wait 50-nS to read
#define FlashRst_HI    hetREG1->DSET=  BIT11;
 
//TP48              (T1) HET1_7 
#define h_TP48_LO     hetREG1->DCLR=  BIT7;
#define h_TP48_HI     hetREG1->DSET=  BIT7;

//SUSPEND           (V7) HET1_9
#define Suspend_LO    hetREG1->DCLR=  BIT9;
#define Suspend_HI    hetREG1->DSET=  BIT9;
          
//LED1of3 ..........LED3of3  
//LED Drive to get white
//  Current forward=08ma  Red
//  Current forward=14ma  Grn
//  Current forward=18ma  Blu

//LED_BLUE          (V2) HET1_1
#define     BLU     1    
#define LED_BLU1      hetREG1->DCLR=  BIT1; //ON
#define LED_BLU0      hetREG1->DSET=  BIT1; //OFF

//LED_GRN           (P2) HET1_20
#define     GRN     20    
#define LED_GRN1      hetREG1->DCLR=  BIT20; //ON
#define LED_GRN0      hetREG1->DSET=  BIT20; //OFF

//LED_RED           (P1) HET1_24
#define     RED     24    
#define LED_RED1      hetREG1->DCLR=  BIT24; //ON
#define LED_RED0      hetREG1->DSET=  BIT24; //OFF
//LED_OFF           (Px) HET1_13,20,24
#define LEDs_OFF      hetREG1->DSET= (BIT1|BIT20|BIT24);

//HET1_6 normally unused, is now an output to control oTree Daughter board power
//Orange Tree board can locked up by aggressive network port scanning
//oTreePwr          (W3) HET1_6
#define oTreeOn       hetREG1->DCLR=  BIT6;    //new, on SBND REV A 2020 boards
#define oTreeOff      hetREG1->DSET=  BIT6;    //new, on SBND REV A 2020 boards

//FAN1              (D19) HET1_10
#define FAN1_LO        hetREG1->DCLR=  BIT10;
#define FAN1_HI        hetREG1->DSET=  BIT10;
//FAN2              (A11) HET1_14
#define FAN2_LO        hetREG1->DCLR=  BIT14;
#define FAN2_HI        hetREG1->DSET=  BIT14;

//TEST PNT 47         (N1) HET1_15    
#define h_TP47_LO       hetREG1->DCLR=  BIT15;
#define h_TP47_HI       hetREG1->DSET=  BIT15;

//CLR_ERR           (A14) HET1_26  nError reset
#define CLR_ERR_LO     hetREG1->DCLR=  BIT26;
#define CLR_ERR_HI     hetREG1->DSET=  BIT26;

//DSR               (B11) HET1_30
#define DSR_LO         hetREG1->DCLR=  BIT30;
#define DSR_HI         hetREG1->DSET=  BIT30;
//DTR               (A13) HET1_17
#define DTR_LO         hetREG1->DCLR=  BIT17;
#define DTR_HI         hetREG1->DSET=  BIT17;

//ucLED             (B3) HET1_22
#define ucLED_LO       hetREG1->DCLR=  BIT22;
#define ucLED_HI       hetREG1->DSET=  BIT22;

//RDWR_B            (J4) HET1_23
#define RDWR_B_LO      hetREG1->DCLR=  BIT23;
#define RDWR_B_HI      hetREG1->DSET=  BIT23;


//InitB0 ..........InitB3  
//#define INIT_Bx[] = {29,16,27,6}; //Init array list of 'init' het port#

//Read InitB1 pin   (A4)  HET1_16
#define InitB1_Read    ((hetREG1->DIN>>16) & 1U)

//Read InitB2 pin   (K19) HET1_28
#define InitB2_Read    ((hetREG1->DIN>>28) & 1U)   //for ver1 board was 27

//Read InitB3 pin   (W5)  HET1_2
#define InitB3_Read    ((hetREG1->DIN>>02) & 1U)


//Read InitB0 pin   (H4)  HET1_21
#define InitB0_Read     ((hetREG1->DIN>>21) & 1U)
//Read InitB0 pin   (G19) SPI1_ENABLE PIN
//#define InitB0_Read    ((spiPORT1->DIN >> 8U) & 1U)   //spiPORT2->DOUT=(uint32)0U<< SPI_PIN_ENA; //= 8U,
//#define InitB0_Read spiREG2->PC3 = (uint32)((uint32)0U << 10U);  //spiPORT2->DOUT=(uint32)0U<< SPI_PIN_SIMO_2; 



    
//*********** HET1 INPUTS ************
//*********** HET1 INPUTS ************

//TMP05A            (E18) HET1_8
//TMP05A Temperature Input used by 'HET APP CODE'

//ERROR LATCH       (B12) HET1_4 INPUT  nError Monitor
#define hERR_Latch  ((hetREG1->DIN >> 4) & 1U) 

//FLASH RDY         (J1) HET1_18 INPUT
#define FLASH_RDY   ((hetREG1->DIN >> 18) & 1U)
            
//DTR USB BRIDGE    (A13) HET1_17
#define hDTR        ((hetREG1->DIN >> 17) & 1U))


//*********** GIO OUTPUTS ************
//*********** GIO OUTPUTS ************

//CSI_B0 ..........CSI_B3  
//CSI_B0            (G1) GIOB4 OUTPUT
#define CSI_B0_LO   gioPORTB->DCLR = (uint32)1U << 4;
#define CSI_B0_HI   gioPORTB->DSET = (uint32)1U << 4;
//CSI_B1            (G2) GIOB5 OUTPUT
#define CSI_B1_LO   gioPORTB->DCLR = (uint32)1U << 5;
#define CSI_B1_HI   gioPORTB->DSET = (uint32)1U << 5;
//CSI_B2            (F2) GIOB2 OUTPUT
#define CSI_B2_LO   gioPORTB->DCLR = (uint32)1U << 6;
#define CSI_B2_HI   gioPORTB->DSET = (uint32)1U << 6;
//CSI_B3            (F1) GIOB7 OUTPUT
#define CSI_B3_LO   gioPORTB->DCLR = (uint32)1U << 7;
#define CSI_B3_HI   gioPORTB->DSET = (uint32)1U << 7;



//PROG_B0 ..........PROG_B3  
//PROG_B0           (M2) GIOB0 OUTPUT
#define PROG0_LO    gioPORTB->DCLR = (uint32)1U << 0;
#define PROG0_HI    gioPORTB->DSET = (uint32)1U << 0;
//PROG_B1           (K12) GIOB1 OUTPUT
#define PROG1_LO    gioPORTB->DCLR = (uint32)1U << 1;
#define PROG1_HI    gioPORTB->DSET = (uint32)1U << 1;
//PROG_B2           (F2) GIOB2 OUTPUT
#define PROG2_LO    gioPORTB->DCLR = (uint32)1U << 2;
#define PROG2_HI    gioPORTB->DSET = (uint32)1U << 2;
//PROG_B3           (W10) GIOB3 OUTPUT
#define PROG3_LO    gioPORTB->DCLR = (uint32)1U << 3;
#define PROG3_HI    gioPORTB->DSET = (uint32)1U << 3;
//PROG_ALL          BITS 3,4,5,6 GIOBx OUTPUT
#define PROGx_LO    gioPORTB->DCLR = (uint32)(1U<<0)+(1U<<1)+(1U<<2)+(1U<<3);
#define PROGx_HI    gioPORTB->DSET = (uint32)(1U<<0)+(1U<<1)+(1U<<2)+(1U<<3);


//DONE0 ..........DONE3  
//DONE0             (A5) GIOA0 INPUT
#define DONE0       ((gioPORTA->DIN >> 0) & 1U)
//DONE1             (C1) GIOA2 INPUT
#define DONE1       ((gioPORTA->DIN >> 2) & 1U)
//DONE2             (C2) GIOA1 INPUT
#define DONE2       ((gioPORTA->DIN >> 1) & 1U)
//DONE3             (E1) GIOA3 INPUT
#define DONE3       ((gioPORTA->DIN >> 3) & 1U)

//ETHER IRQ         (B5) GIOA5 INPUT
#define ETH_IRQ     ((gioPORTA->DIN >> 5) & 1U)

//FPGA IRQ          (A6) GIOA6 INPUT
#define FPGA_IRQ    ((gioPORTA->DIN >> 4) & 1U)



//*********** SPI INPUTS / OUTPUTS (#include "spi.h")************
//*********** SPI INPUTS / OUTPUTS ************
//ADC_RESET         (D1, SPI2_SIMO2) OUTPUT
    /** - SPI2 Port output values */
//  spiREG2->PC3 =    (uint32)((uint32)1U << 0U)  /* SCS[0] */
//                    | (uint32)((uint32)1U << 1U)  /* SCS[1] */
//                    | (uint32)((uint32)0U << 8U)  /* ENA */
//                    | (uint32)((uint32)0U << 9U)  /* CLK */
//                    | (uint32)((uint32)1U << 10U)  /* SIMO */
//                    | (uint32)((uint32)0U << 11U); /* SOMI */
#define FLASH_WP_LO   spiREG2->PC3 = (uint32)((uint32)0U << 10U);  //spiPORT2->DOUT=(uint32)0U<< SPI_PIN_SIMO_2; 
#define FLASH_WP_HI   spiREG2->PC3 = (uint32)((uint32)1U << 10U);  //spiPORT2->DOUT=(uint32)1U<< SPI_PIN_SIMO_2;

// Toggle HET pin0
//gioSetPort(hetPORT1, gioGetPort(hetPORT1) ^ 0x00000001);


//two ways of setting pointers, via assignLinkPort() or direct
/*
    HappyBus.FM_DAT = fbase+ (oFMData30[brdNum]*2);       	//lvds data
        IOPs[rg].FM_DATp= (uSHT*)FP+ ((oFMDATA30+(bit))); 	//lvds data 0x30-37 pointer

	HappyBus.FM_STA = fbase+ (oFMStat40*2);             	//lvds status
        IOPs[rg].FM_STAp= FP+ (oFMStat40*2);    			//lvds status

	HappyBus.FM_PAR = fbase+ (oFMPerr41*2);             	//lvds parity error
        IOPs[rg].FM_PARp= FP+ (oFMPerr41*2);       			//lvds parity error
   
    HappyBus.ePHY_DATAVAIL_BIT= (1<<(brdNum-1));             //ePHY data avail status bit
        IOPs[rg].ePHYFF_RECp= (uSHT*)FP+(oPHY_RDFF20+(bit)); //ePHY rec data fifo pointer 0x20

	HappyBus.ePHY_RecFIFO= (uSHT*)fbase+(oPHY_RDFF20*brdNum); //ePHY rec data fifo
    
    HappyBus.ePHY_CH_ENA= (uSHT*)fbase+(oPHY_XMT_ENA);      //Broadcast chans 0-7 enables
        IOPs[rg].ePHYXMT_ENAp= (uSHT*)FP+ (oPHY_XMT_ENA);   //Broadcast chans 0-7 enables

	HappyBus.ePHY_SndFIFO= (uSHT*)fbase+(oPHY_WRFF11);      //ePHY xmit data fifo
        IOPs[rg].ePHYFF_XMTp=  (uSHT*)FP+ (oPHY_WRFF11);    //ePHY xmit data fifo

	HappyBus.ePHY_STAT= (uSHT*)fbase+(oPHY_rxSTA16);        //PHY 8-Port Rec data Status (bit7-0, 1=empty)
        IOPs[rg].ePHY_STAp=    (uSHT*)FP+ (oPHY_rxSTA16);   //PHY 8-Port Rec data Status (bit7-0, 1=empty)

	HappyBus.ePHY_SndPAC= (uSHT*)fbase+(oPHY_XMIT12);       //ePHY XMIT R/W (RD 1=empty), (WR 1 to xmit)
        IOPs[rg].ePHYXMT_PACp= (uSHT*)FP+ (oPHY_XMIT12);    //ePHY XMIT R/W (RD 1=empty), (WR 1 to xmit)

	
    REG16(HappyBus.FM_PAR)= FMRstBit8;                  	//BIT8, clear buffers                    
        IOPs[rg].ePHY_BIT= (1<<bit);                        //ePHY data avail status bit
	
*/

#endif
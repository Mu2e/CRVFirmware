//********************************************************
//  @file sys_mu2e_case2.c
//  RM48 Code
//  RM48 Hercules device supports little-endian [LE] format
//
//  Program Xilinx Spartan 6 fpga
//  Program and Read Spansion Inc S29JL064J 64 Megabit 
//                  (8M x 8-Bit/4M x 16-Bit) Flash Memory
//  Program and Read FRAM memory chip 'FM25CL64B'
//  tek 2016
//********************************************************



#include "sys_common.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "het.h"
#include "gio.h"
#include "emac.h"
#include "adc.h"


//sys_mu2e proto-types
#include "ver_io.h"
//#include "ver_help.h"
#include "ver_ePHY.h"

#include "spi.h"
#include "hw_reg_access.h"
#include "sys_core.h"
#include "sys_dma.h"
#include "reg_sci.h"

#include "sys_mu2e.h"
#include "sys_mu2e_misc.h"
#include "sys_mu2e_functions.h"
#include "fram.h"



//fram
#include "fram.h"


//wiznet stuff chip W5300
#include "socket.h"

//system
extern int  resetEntry();                   //file 'sys_intvecs.asm'
#define     putchar     __putchar

//spi port control config
extern spiDAT1_t   dataconfig_ADC;
extern spiDAT1_t   dataconfig_PGA;
extern spiDAT1_t   dataconfig_FRAM;
extern spiDAT1_t   dataconfig_EQUAL;
extern spiDAT1_t   dataconfig_FRAM_HLD;            //hold 'cs' active low


//Wiznet biffers
extern uint8  check_sendok_flag[MAX_SOCK_NUM];//Flag to check if first send or not.
extern int  recDatSz[];         //store last rec'd packet data size (diagnostics)

//Wiznet
extern int         wizMain(void);
extern char*       inet_ntoa(unsigned long addr);
extern int  loopback_tcps(SOCKET s, uint16 port, uint8* buf,uint16 mode);
extern int  loopback_udp(SOCKET s, uint16 port, uint8* buf,uint16 mode);
extern void GW_processing(int mSecs, int port);
extern char  eRecDatBuf[4][CHAR_BUF_SZ_1600];   //socket(s) command line buffer size

//FLASH Chip
extern struct phyHeader phyHdr;
extern uint8     Op[];



//terminal i/o buffers
extern char        Buf1500[];
extern uSHT        BUF8192[];
extern char        g_lineBuf[];
extern char        tBufHoldUsb[];
extern char        tBufHoldSoc[];
extern char        tBuf[];


//EMAC structure
extern hdkif_t hdkif_data[MAX_EMAC_INSTANCE];   //defined in 'emac.c'

//Structures
extern struct      netinfo_s    netInfo;               //Network Active setup data
extern struct      Controller_tdcHdr   Cntrl_tdcHDR, tHDR_STORE;
extern struct      structFPGA_Trig_Cnts FPGA_Trig_Stat;
extern struct      blocksnd    BinSt;
extern struct      blocksndWrd BinStRDX;
extern volatile    struct msTimers mStime; 
extern struct      udpHeader udpHdr;
extern struct      udpMessage  udpMsg;
extern struct      HappyBusReg HappyBus;      //cmdSiz, brd, Cndtype, sndwcnt, myLinkNumber, myLinkNumbTemp, revErrs, srcDataBuf;

//ePHY download struc
extern struct      uSums uPhySums;


extern      struct g_sciTransfer g_sciTransfer_t[2U];
extern      struct g_sciTransfer
            {
             uint32   mode;         /* Used to check for TX interrupt Enable */  
             uint32   tx_length;    /* Transmit data length in number of Bytes */
             uint32   rx_length;    /* Receive data length in number of Bytes */  
             uint8    * tx_data;    /* Transmit data pointer */  	
             uint8    * rx_data;    /* Receive data pointer */  
            }g_sciTransfer_t[2U];

//dma structure
extern g_dmaCTRL g_dmaCTRLPKT;                         //dma control packet config stack

//network structure
extern struct      netinfo_s netInfo;                  //Wiznet Setup

//com-port setup
extern char*       cbufptr;
extern int         cmdIndex;
extern char*       g_paramPtr;
extern int         short tREG30;  //temp test registger

extern uint32      volatile iFlag, genFlag, BootFlag;
extern uint16      g_i2cErrors, g_i2cErr, g_dat16;
extern uint32      g_dat32, g_Pass;
extern uint32      g_SendBinReqSizeOver;

//com portbuffers
extern char        cmdbuf[];
extern uint8       comBuf[];             //input commands process buffer
extern uint8_t     USB_inBuf[USB_inBufSz];
extern struct      vB USB_Rec;

//header buffers
extern uint16_t    rdbHdrSOCK[cCntlHdrSizWrd];         //hold spill hdr for socket send using 'RDB'
extern uint16_t    rdbHdrSOCK_NULL[cCntlHdrSizWrd];    //8 word null header

//status block buffers
#define     sBLKSIZ38   38
#define     sBLKSIZ22   22
extern struct stBlock PoolData;   //replaces 'statusBlock[5][sBLKSIZ38+2]' for data pooling

extern uint16      statusBlock[5][sBLKSIZ38+2];
//OneWire buffers for pooled data
extern int         oneWireErr;             //oneWireData bad readings
extern int         oneWireTemp[4][4];      //oneWireData 4-FPGAs, 8ch-TEMPs
extern unsigned long oneWireRom[4][4];     //oneWireData 4-FPGAs, 8ch-ROMs

//Wiznet Buffers
extern int         eTout[MAX_SOCK_NUM];                //store socket timeout status from intr read
extern uint8       I_STATUS[MAX_SOCK_NUM];             //store socket status for backround processing

extern char        eCmdBuf[MaxSock][eRecSz1024];

extern char*       eRecDatPtr[MaxSock];                //socket(s) command line buffer size
extern char*       eCmdDatPtr[MaxSock];                //socket(s) data prts
extern unsigned    int sLen[MaxSock];


//adc buffer, 16 channels plus one temp/work channel [17]
extern adcData_t   adc_data[17];               



//DMA Wiznet
#define     D_SIZE      4*256
extern uint16      RDB_dmaDATA[];        //FPGA-MEMORY-WIZNET buffer in sys ram 


//emac stuff
extern int     g_EMACTxRdy;
extern int     g_EMACRxRdy;

//Uptime capture from 'sys_main.c'
extern uint32  Up_Time;

//misc
extern int     esm_Ch;
extern int     esm_Cnt;
extern int     fpga_ld_cnt;
extern int     g_lcnt;
extern int     g_pCnt;
extern int     g_pCntPerSec;
extern int     adc_ms;
extern int     g_Sock;
extern int     d_nErr;
extern short   g_signD16[];    
extern long    param1, param2, param3;         //signed intergers
extern int     CmbActiveCnt[];                //count for act ch on each fpga

//nHet temperature readings and PWM
extern unsigned int Ris2Fall, Fall2Ris, cmdTime; 
extern int     Duty_Period1,Duty_Period2,Duty_Period3,Duty_Period4;
extern int     Set_Duty_Period1;

//Version Info
extern div_t   divV;                   //struc for div() quot,rem  code ver ##
extern div_t   divR;                   //struc for div() quot,rem


#define FP_REGS_MAX     22      //(nn regs groups(ROWs), plus one Later Code count from 1)
#define FP_REGS_VALID0 0xAFE0   //mask off lowest nibble
#define FP_REGS_VALID1 0xAFE1
#define FP_REGS_VALID2 0xAFE2
#define FP_REGS_VALID3 0xAFE3
#define rLOADED         00      //Number of actual register loaded, Filled in by code
#define fMux            0
#define fGain           3       //3==gain of 1 on pgm280 chip

//fpga(4 ea) reg setup structure (defaults)
extern FPGA_RegS  f_RegConst [FP_REGS_MAX];

//fpga(4ea) reg setup structure (flashed and restored)
extern FPGA_RegS1   f_RegHold[4][FP_REGS_MAX];



//uC_Store PreSet as {SerNumb, MuxCh, Gain, Trig, LnkDir, nErr, xPhy, wDogTimOut}
extern struct          uC_Store uC_FRAM;    
extern struct          uSums u_SRAM;     //DwnLdSdRamCnt, DwnLdSdRamSum,stat,time


extern uint8_t	emacAddressAFE[6U];


#define PACs  2 

#pragma  pack(4)                                    //force 16bit boundary on all types
extern pbuf_t   packs[]; 
extern pbuf_t   pack[PACs]; 



extern const   char HelpMenu[];
extern const   char HelpMenu2[];
extern const   char HelpMenu3[];
extern const   char HelpMenuFLASH[];
extern const   char HelpMenuSTAB[];
extern const   int   netVars[];


//Check if rec'd command is valid and handle it.
int process2(int prt, char *cmdPtr)
{
    sPTR p_sAddr;
    cPTR p_cAddr;
    char *tok = 0, *paramPtr= 0;
    unsigned int parErr=0;
    short ped;
    unsigned short Slp;
    unsigned int Slp32;

    paramPtr = cmdPtr;                                  //ptr to data buffer    
    tok = mytok(paramPtr, &paramPtr);                   //find 1st token, should be acsii 'CMD'
    if (!tok)
        {
        putBuf(prt,"\r\n>",3);                          //echo prompt
        return 1;
        }
    else if (prt==tty)                                  //give USB crlf
          putBuf(prt,"\r\n",2);                         //send to current active port

    switch (*tok)                                       //command parameter search
        {
        case 'D':
                //DSAVE fix to READ FPGA REGS
                if (!strcmp(tok, "DSAV"))              //read/store fpga regs to FRAM
                    {
                    uint16 framAddr;
                    uint16 cnt=0, d16, rcounter; 
                    u_32Bit addr32;
                    u_32Bit FPGA_BASE= fpgaBase;

                    for(int chip=0; chip<4; chip++)
                        {
                        framAddr= ((fPAGE_FPGA0+4) +(0x100*chip));
                        rcounter=0;
                        addr32 = (u_32Bit)FPGA_BASE;  //check if afe chips enabled
                        
                        //read FPGAs(4) and store data to FRAM
                        //FP_REGS_MAX number of register to address
                        //cnt= Words per Base Reg(with addr increment) to write
                        for(cnt=1; cnt<FP_REGS_MAX; cnt++)
                            { 
                            addr32 = (u_32Bit)FPGA_BASE + (f_RegConst[cnt].Addr<<1);  //actual 32 bit address
                            for (int i=0; i< f_RegConst[cnt].RegCnt; i++, addr32+=2)
                                {
                                if (f_RegConst[cnt].Addr <0x100)            //fpga low range
                                    d16= *(sPTR)addr32;
                                else if (f_RegConst[cnt].Addr >=0x300)      //fpga high range
                                    d16= *(sPTR)addr32;
                                else                                        //afe addr range
                                    {
                                    //use default setup
                                    d16= f_RegConst[cnt].Value[i];
                                    }
                                //now store read data to FRAM 
                                f_RegHold[chip][cnt].Value[i]= d16;
                                FRAM_WR_BUF16(framAddr, &d16, 1); //fRamAddr, fpgaAddr, write Cnt
                                framAddr += 2;
                                rcounter++;
                                }
                            }
                        FPGA_BASE += 0x800; //offset to next fpga 1of4
                        }
                    //Save valid flag
                    f_RegHold[0][0].Value[0]= FP_REGS_VALID0;   //base add hold flag
                    f_RegHold[0][0].Value[1]= rcounter;         //base addr holds load cnt
                    f_RegHold[1][0].Value[0]= FP_REGS_VALID1;   //base add hold flag
                    f_RegHold[1][0].Value[1]= rcounter;         //base addr holds load cnt
                    f_RegHold[2][0].Value[0]= FP_REGS_VALID2;   //base add hold flag
                    f_RegHold[2][0].Value[1]= rcounter;         //base addr holds load cnt
                    f_RegHold[3][0].Value[0]= FP_REGS_VALID3;   //base add hold flag
                    f_RegHold[3][0].Value[1]= rcounter;         //base addr holds load cnt

                    FRAM_WR_BUF16(fPAGE_FPGA0, &f_RegHold[0][0].Value[0], 2);     //fRamAddr, fpgaAddr, write Cnt
                    FRAM_WR_BUF16(fPAGE_FPGA1, &f_RegHold[1][0].Value[0], 2);     //fRamAddr, fpgaAddr, write Cnt
                    FRAM_WR_BUF16(fPAGE_FPGA2, &f_RegHold[2][0].Value[0], 2);     //fRamAddr, fpgaAddr, write Cnt
                    FRAM_WR_BUF16(fPAGE_FPGA3, &f_RegHold[3][0].Value[0], 2);     //fRamAddr, fpgaAddr, write Cnt

                    //store {SerNumb, MuxCh, Gain, Trig, LnkDir, nErr, xPhy, wDogTimOut}
                    FRAM_WR_BUF16(fPAGE_0400, (uint16*)&uC_FRAM, FRAM_400wsz); //addr, data, WordCnt
                    putBuf(prt,"FRAM UPDATED\r\n",0);       //send to current active port
                    break;
                    }
                else if (!strcmp(tok, "DREC"))              //FLASH- data restore to registers
                    {
                    param1= arg_dec(&paramPtr,1);           //get 1st param, 0=load constant data
                    if ((param1==0) || (param1==1))         //two choices
                        {
                        //read saved FRAM data and Load to FPGA Registers
                        dataRecall(param1, fpgaBase);
                        dataRecall(param1, fpgaBase1);
                        dataRecall(param1, fpgaBase2);
                        dataRecall(param1, fpgaBase3);
                        putBuf(prt,"FPGA Regs Updated, Broadcast Range 0x300 Up restored using FPGA1 setup\r\n",0); //send to current active port
                        }
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'W':
                if (!strcmp(tok, "WG"))                //Wiznet GateWay Init, for testing only
                    {
                    //for updating gw arp cache
                    //sprintf(tBuf, "Request update on Gateway arp cache\r\n");
                    //putBuf(prt,tBuf,0);
                    GW_processing(700, prt);    
                    break;
                    }
                else if (!strcmp(tok, "WI"))                //Wiznet Init, for testing only
                    {
                    int i;
                    //Wiznet Reset low for at least 2 uSec
                    WIZRST_LO
                    uDelay(10);  
                    WIZRST_HI       
                    uDelay(100);  
                    //test if Wiznet chip W5300 active and ACKs with valid data
                    i= getMR();
                    if (( i==0xB900 ) || ( i==0xB800 ))
                        genFlag |= WiznetACK;
                    sprintf(tBuf,"\r\nWiznet Mode= %x (B800,B900=valid)\r\n", getMR());
                    putBuf(tty,tBuf,0);
                    //Wiznet startup
                    //wiznet not anwering on bus, dont initialize it
                    if (genFlag & WiznetACK) 
                        {
                        //Wiznet needs delay, wiat 200mS for PLL lock
                        mDelay(100);
                        if (wizMain()==1)                   //wiznet memory error, disable it
                            genFlag &= ~WiznetACK;
                        sprintf(tBuf,"\r\nWiznet Mode= %x (B800,B900=valid)\r\n", getMR());
                        putBuf(tty,tBuf,0);
                        }
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'E':
                //nERROR is a push-pull output and is normally driven High. The Error Signaling Module (ESM) 
                //drives this pin low to indicate any group2 and group3 error condition. Also, you can 
                //configure the ESM to drive this pin low on any group1 error condition.
                //When the nERROR pin gets driven low on an error condition the error flag must be cleared 
                //and then 0x5 written to the ESMKEY register in order for the nERROR pin to go High after 
                //the programmed duration has expired.
                //nRST:   write 0x8000 to SYSECR register.
                //nERROR: write 0xA to ESMEKR register.
                            
                // Check if there were ESM group3 errors during power-up.
                // These could occur during eFuse auto-load or during reads from flash OTP
                // during power-up. Device operation is not reliable and not recommended
                // in this case.
                // An ESM group3 error only drives the nERROR pin low. An external circuit
                // that monitors the nERROR pin must take the appropriate action to ensure that
                // the system is placed in a safe state, as determined by the application.
                //
      
                if (!strcmp(tok, "ER0"))        //clr nError Flip/Flop
                    {
                    uC_FRAM.nErrCnt=0;
                    FRAM_WR_ENABLE();                      
                    FRAM_WR(fPAGE_0400, (uint8*)&uC_FRAM.nErrCnt, 2); //addr,data,cnt
                    break;
                    }
                if (!strcmp(tok, "ER1"))        //test single bit errro
                    {
                    //configure the ESM to drive this pin low on any group1 error condition
                    //esmEnableInterrupt(35);
                    //esmEnableError(35);
                    d_nErr= TrgSinBitErr;       //single bit err wont cause reset
                    mDelay(500);
                    //CLR_ERR, must keep low to allow nError to Reset Board
                    CLR_ERR_HI
                    CLR_ERR_LO
                    break;
                    }
                else if (!strcmp(tok, "ER2"))   //test double bit error
                    {
                    sprintf(tBuf,"nError Test, uC should now reboot.\r\n");
                    putBuf(prt, tBuf,0);
                    mDelay(1);
                    //Software reset will not clear nError once set. (cmd cleared in software)
                    //Have to do PwrReset to toggle uC TRST pin, this allows clean boot.
                    d_nErr= TrgDouBitErr;               //dou bit err will trig nError (extrn reset logic)
                    break;
                   }
                else  {parErr=0xf;   break; }
        case 'F':
                if (!strcmp(tok, "FD"))             //FPGA Load, direct from USB
                   {
                  //param1=arg_dec(&paramPtr,1);    //get 1st param U39B
                  //sprintf(tBuf,"FPGA Direct Load from USB, chip 1of4 =%d\r\n", param1);
                    sprintf(tBuf,"FPGA Direct Load from USB, Under Construction\r\n");
                    putBuf(prt, tBuf,0);
                    //loadSpartan6_FPGA(param1);
                    break;
                    }
                else if (!strcmp(tok, "FE"))        //FLASH erase all
                   {
                    sprintf(tBuf,"TOTAL Parallel Flash Erase, Enter 1 to erase, else exit\r\n");
                    putBuf(prt, tBuf,0);
                    param1=arg_dec(&paramPtr,0);    //now get okay todo
                    if(param1!= 1)
                     break;
                    eraseFLASH();                
                    break;
                    }
                else if (!strcmp(tok, "FES"))        //FLASH erase sectors
                   {
                    sprintf(tBuf,"FLASH Erase Sector, Enter 1 to erase, else exit\r\n");
                    putBuf(prt, tBuf,0);
                    param1=arg_dec(&paramPtr,0);    //get 1st param U39B
                    if(param1!= 1)
                     break;
                    param1=arg_dec(&paramPtr,SECTORES);    //get 1st param U39B
                    sprintf(tBuf,"FLASH Erase Sectors %d\r\n", param1);
                    putBuf(prt, tBuf,0);
                    eraseFLASH_Sector(param1, prt);
                    break;
                    }
                else if ((!strcmp(tok, "FL1")) || (!strcmp(tok, "FL")))  //FLASH Load of FPGA File via USB
                   {
                    //disable watchdog timer, file notifications.c
                    iFlag &= ~WHATCHDOGENA;          

                    sprintf(tBuf,"S29JL064J: Flash Loader\r\n");
                    putBuf(prt, tBuf,0);
                    PROGx_LO                    
                    uDelay(5);                      //min 300nS
                    PROGx_HI                    
                    uDelay(100);                    //fpga reset takes about 800uS
                    
                    flashStatus(prt);               //fills Buf1500 with ascii msgs
                    uDelay(100);                   
                    eraseFLASH_Sector(SECTORES, prt);
                    sprintf(tBuf,"S29JL064J: Total sector erased %d (%d Bytes)\r\n", SECTORES, (SECTORES-7)*0x7fff);
                    putBuf(tty, tBuf,0);
                    sprintf(tBuf,"S29JL064J: Select Binary file. (MTTY Shortcut Key F5)\r\n");
                    putBuf(prt, tBuf,0);
                    loadFLASH(0,prt);
                    //flashXFER(4, prt);
                    break;
                    }
                else if (!strcmp(tok, "FT"))        //FLASH Load of FPGA File via USB
                   {
                    param1=arg_hex(&paramPtr,4);    //get 1st param (addr)
                    if(param1>4)
                        { parErr++;  break; }
                    if(param1<4)
                      sprintf(tBuf,"FLASH xFer to FPGA %d\r\n",param1);
                    else
                      sprintf(tBuf,"FLASH xFer to FPGAs 0,1,2,3\r\n");
                    putBuf(prt, tBuf,0);
                    flashXFER(param1,prt);                
                    break;
                    }
                else if (!strcmp(tok, "FS"))        //FLASH Status
                   {
                    //int chip=1;
                    u_16Bit D16;
                    u_32Bit D32;
                    FRAM_RD(DWNLD_ADDR_COUNT,(uint8*)&D32, 4);
                    FRAM_RD(DWNLD_ADDR_CSUM, (uint8*)&D16, 2);
                    flashStatus(prt);                  //fills Buf1500 with ascii msgs
                    sprintf(Buf1500,"Flash FPGA ByteCnt = %d\r\n",D32);
                    sprintf(tBuf, "Flash FPGA SumCheck= %X\r\n",D16);
                    strcat (Buf1500,tBuf);
                    putBuf(prt, Buf1500,0);
                    break;
                    }
                else if (!strcmp(tok, "FI"))        //fpga reset using init poin
                   {
                    sprintf(tBuf,"fpga reset to all 4 chips using PROGRAM_B\r\n");
                    putBuf(prt, tBuf,0);
                    //reset fpga(s)
                    PROGx_LO                        //min 300nS
                    uDelay(5);
                    PROGx_HI                    
                    uDelay(1000);                   //fpga reset takes about 800uS
                    break;
                    }                
                else if (!strcmp(tok, "FMS"))       //MF SEND, use LVDS PORT 16bit xmits
                   {
                    if (HappyBus.BrdNumb==0) 
                        HappyBus.BrdNumb=1;
                    HappyBus.SndWrds= sizeof(PoolData)/2;   //size in words            
                    HappyBus.Src= (snvPTR)PoolData.hdr;     //src data ptr
                    *HappyBus.Src++= HappyBus.BrdNumb;      //brd numb 1-24
                    *HappyBus.Src++= eCMD75_CONSOLEBIN;     //flag as binary data
                    *HappyBus.Src++= HappyBus.SndWrds;      //wrd cnt
                    HappyBus.Src= (snvPTR)PoolData.hdr;     //reinit src begin
                    uTimer(4);
                    //NOTE PROMPTS GETS SENT BACK ON LVDS ON EXIT HERE   
                    genFlag |= NO_ECHO;                     //no ASCII reply 
                     
/*                    for (int i=0; (i<sBLKSIZ22); i++) //sSMALLSIZ==22
                        {
                        FMDataSnd= statusBlock[0][i];  //PoolData now filled in backgnd for pooling data
                        uDelay(2);
                        }
                    //4 FPGA DATA
                    for(int f=1; f<5; f++)
                      for (int i=0; (i<sBLKSIZ38); i++) //sBLOCKSIZ==38
                        {
                        FMDataSnd= statusBlock[f][i];
                        uDelay(2);
                        }
*/                    
                    break;
                    }
                else if (!strcmp(tok, "FZ"))        //testing flash non 0xffff check
                   {
                    param1=arg_hex(&paramPtr,0x800); //get 1st param (range to check)
                    sPTR saddr= pFLASHbase;
                    int d16,err=0,i;
                    sprintf(tBuf,"Check/Display 'Un-Erased' FLASH data (FlashBase=%X)\r\n",pFLASHbase);
                    putBuf(prt, tBuf,0);
                    for (i=0; i<param1+1; i++)      
                        {
                        d16= *saddr;
                        if(d16!=0xffff)
                            {
                            sprintf(tBuf,"Addr= %X  Data==%4X\r\n", saddr, d16);
                            putBuf(prt, tBuf,0);
                            if (++err >= 20) break;
                            }
                        saddr++;
                        }
                    sprintf(tBuf,"Total checked words= 0x%X (show 20 lines max)\r\n",i);
                    putBuf(prt, tBuf,0);
                    break;
                    }
                 else if  (!strcmp(tok, "FRD"))                 //FRAM READ
                    {
                    u_16Bit D16;
                    if ( (param1=arg_hex(&paramPtr, 0)) ==-1) //get 1st param
                        { parErr++;  break; }
                    FRAM_RD(param1, (uint8*)&D16, 2);
                    sprintf(Buf1500,"%X\r\n",D16);
                    putBuf(prt, Buf1500,0);                     //send to current active port
                    break;
                    }
                 else if (!strcmp(tok, "FWR"))                  //FRAM UPDATE
                    {
                    if ( (param1=arg_hex(&paramPtr,-1)) ==-1)   //get 1st param, addr
                        { parErr++;  break; }
                    if ( (param2=arg_hex(&paramPtr,-1)) ==-1)   //get 2nd param, data
                        { parErr++;  break; }
                    FRAM_WR_ENABLE();                           //WREN op-code must be issued prior to any Wr_Op
                    FRAM_WR(param1, (uint8*)&param2, 2);        //2-8bit writes
                    break;
                    }
                else if (!strcmp(tok,"FDUMP"))  //'FDUMP' read flash status register
                    {
                    uint16_t Adr;
                    uint16_t D16BufIn[10];
                    int lp=0;
                    Adr=arg_hex(&paramPtr,0);              //get 1st param (addr)
                    sprintf(Buf1500,"Read fRam block (16bit big endian per line)\r\n");
                    putBuf(prt, Buf1500,0);                     //send to current active port
                    while (lp++< 16)
                        {
                        FRAM_RD(Adr, (uint8*)D16BufIn, 8);      //FRAM size (8K x 8 ) display as 8 words per line
                        sprintf(Buf1500,"FRAM=%04X  %4x %4x %4x %4x  ",Adr&0x1fff, D16BufIn[0],D16BufIn[1],D16BufIn[2],D16BufIn[3]);
                        Adr += 8;       // 4 (16 bit reads)
                        FRAM_RD(Adr, (uint8*)D16BufIn, 8);
                        sprintf(tBuf,"%4x %4x %4x %4x\r\n\r", D16BufIn[0],D16BufIn[1],D16BufIn[2],D16BufIn[3]);//extra \r is for lvds return data in case odd data count
                        Adr += 8;       // 4 (16 bit reads)
                        strcat(Buf1500, tBuf);
                        putBuf(prt, Buf1500,0);             //send to current active port
                        }
                    break;
                    }
                else if  (!strcmp(tok, "FRERASE"))          //erase FRAM FM25CL64B, organized as 8K×8
                    {                                       
                    uint16_t Adr=0;
                    u_8Bit D8Buf=0;
                    param1= arg_hex(&paramPtr,0);           //get 1st param begin address
                    param2= arg_hex(&paramPtr,0);           //get 2nd param end addr
                    if(param2==0)                           //no cnt, exit
                      break;
                    if(param2>0xFFF)                        //limit 4Kx16
                      break;
                    if (param2<param1)
                      break;
                    Adr= param1; 
                    FRAM_WR_ENABLE();                       //WREN op-code must be issued prior to any Wr_Op
                    dataconfig_FRAM.CS_HOLD = TRUE;         //Hold 'cs' low
                    sendFRAM(&Op[fWRITE]);                  //send fram OP CODE
                    sendFRAM((uint8_t*)&Adr+1);             //send fram ADDR HI
                    sendFRAM((uint8_t*)&Adr);               //send fram ADDR HI
                    //erase fram memory
                    param2 -= param1;                       //length to erase as bytes
                    while (param2--)
                        sendFRAM(&D8Buf);
                    //final byte, when done sets chip sel back high
                    dataconfig_FRAM.CS_HOLD = FALSE;        //Normal 'cs' high
                    sendFRAM(&D8Buf);
                    break;
                    }
                else if  (!strcmp(tok, "FRS"))              //FRAM status register read
                    {
                    int i;
                    FRAM_WR_STATUS(0x0a);                   //send '8 bit' data F-RAM
                    i= FRAM_RD_STATUS();
                    sprintf(Buf1500,   "fRam Status=%02X\r\n", i );
                    putBuf(prt, Buf1500,0);             //send to current active port
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'G':
                if (!strcmp(tok, "GAIN"))
                   {
                    int gain;
                    gain=arg_dec(&paramPtr,-1);      //get 1st param (def gain ==1)
                    if (gain==-1)
                        {
                        gain= pga280read(0)>>3;     //lower 3 bits are mux status
                        gain = 1<<((gain)-3);       //gain 1,2,4,8,16...
                        sprintf(tBuf,"gain=%d\r\n",gain);
                        putBuf(prt, tBuf,0);
                        }
                    param2=arg_dec(&paramPtr,0);     //now get 2nd param (reset==RST280)
                    //for a gain of 1 we write a (3)
                    if      (gain==1) {gain=3;}
                    else if (gain==2) {gain=4;}
                    else if (gain==4) {gain=5;}
                    else if (gain==8) {gain=6;}
                    else if (gain==16) {gain=7;}
                    else if (gain==32) {gain=8;}
                    else if (gain==64) {gain=9;}
                    else if (gain==128) {gain=10;}
                    else {{ parErr++;  break; }}      //default
                    //sprintf(tBuf,"Gain=%d ,Valid Gains=1,2,4,8,16,32,64,128\r\n",hld);
                    //sprintf(tBuf,"Gain=%d\r\n",hld);
                    //putBuf(prt, tBuf,0);
                    uC_FRAM.pga280Gain= gain;   //flash storage var
                    pga280(gain, param2);
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'H':
                if ((!strcmp(tok, "H1")) || (!strcmp(tok, "HE")))
                    {
                    //putBuf(prt, (char *)HelpMenu, sizeof(HelpMenu)-1);  
                    break;
                    }
                else if (!strcmp(tok, "HDR"))
                    {
                    *Buf1500=0;             //null buffer to be filled
                    if (iFlag& XMIT_PHY_ON) //enabled
                        sprintf(Buf1500,"RDB DATA READ MODE   : CRV CONTROLLER BUFFER(ePHY)\r\n");
                    else
                        sprintf(Buf1500,"RDB DATA READ MODE   : CRV FEB SOCKET (WizNet)\r\n");
                    
                    param1=arg_dec(&paramPtr,0);             
                    header0TTY(prt, param1);
                    putBuf(prt, Buf1500,0);
                    break;
                    }
                else if (!strcmp(tok, "H2"))
                    {
                    //putBuf(prt, (char *)HelpMenu2, sizeof(HelpMenu2)-1);      
                    break;
                    }
                else if (!strcmp(tok, "HF"))
                    {
                    //putBuf(prt, (char *)HelpMenuFLASH, sizeof(HelpMenuFLASH)-1);        
                    break;
                    }
                else if (!strcmp(tok, "HT"))
                    {
                    //putBuf(prt, (char *)HelpMenu3, sizeof(HelpMenu3)-1);        
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'I':
                if (!strcmp(tok, "ID"))
                   {
                    u_16Bit D16;
                    u_32Bit D32;
                    param1=arg_dec(&paramPtr,0);  
                    sprintf(Buf1500,"Module Type       : Mu2e CRV FEB Board\r\n");
                    sprintf(tBuf,   "LOCAL IP ADDR     : %u.%u.%u.%u\r\n", *(netInfo.ipAddr),
                        *(netInfo.ipAddr+1),*(netInfo.ipAddr+2),*(netInfo.ipAddr+3));
                    strcat(Buf1500, tBuf);
                                  
                    sprintf(tBuf,"Compile Date      : %s\r\n", __DATE__);
                    strcat(Buf1500, tBuf);
                    sprintf(tBuf,"Firmware Ver      : %d.%02d\r\n", divV.quot, divV.rem );
                    strcat(Buf1500, tBuf);
                    
                    //FRAM_RD(SERIAL_NUMB_ADR, (uint16_t*)&D16,2);    //read 2 bytes
                    sprintf(tBuf,"Serial Number     : %d\r\n",uC_FRAM.serNumb);
                    strcat (Buf1500,tBuf);
                    FRAM_RD(DWNLD_ADDR_COUNT,(uint8*)&D32, 4);    //read 4 bytes
                    FRAM_RD(DWNLD_ADDR_CSUM, (uint8*)&D16, 2);
                    sprintf(tBuf   ,"Flash ByteCnt     : %d\r\n",D32);
                    strcat (Buf1500,tBuf);
                    sprintf(tBuf,   "Flash SumCheck    : %04X\r\n",D16);
                    strcat (Buf1500,tBuf);

                    //nError ECC error counter
                    sprintf(tBuf,   "ECC Reboot Count  : %d\r\n",uC_FRAM.nErrCnt);
                    strcat (Buf1500,tBuf);
                    
                    //poe port number from FEB Controller, also used as link id number
                    sprintf(tBuf,   "POEPortNumb 1of24 : %d\r\n\n", HappyBus.myLinkNumber); //extra char if odd byte cnt on 16bit LVDS xfer
                    strcat (Buf1500,tBuf);
                    
                    putBuf(prt, Buf1500,0);
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'L':
                if (!strcmp(tok, "LDRAM"))     //Load data file to SDRAM from FEB Controller cmd 'LDFILEFEB'
                   {
                    static int recPaks=0;
                    int d16, EndWrdCnt;
                    u_16Bit* eBufB= (u_16Bit*)eRecDatPtr[g_Sock];                    
                    u_16Bit* SumPHYPtr= (u_16Bit*)&uPhySums;
                  
                    param1= HappyBus.CmdSizW; 
                    if (param1==-1)
                        break;
                    if (HappyBus.CmdType == eCMD73_LDRAMINIT)   //init handler
                        {
                        for(int i=0; i<6;i++)                          
                            *SumPHYPtr++ = d16= *eBufB++;       //store cnt and chksum
                        sprintf(Buf1500,"\r\nFile Size Expected: RecdBytes=%-6d   CheckSum=%04X\r\n", uPhySums.DwnLd_sCNT, uPhySums.DwnLd_sSUM&0xffff);
                        putBuf(0, Buf1500, 0);
                        
                        //set write fpga Addr via special sequence
                        SET_SDADDR_RDx(0,0,0);
                        SET_SDADDR_WRx(0,0,0);

                        //clear download counters
                        u_SRAM.DwnLd_sSUM=0;
                        u_SRAM.DwnLd_sCNT=0; 
                                                
                        recPaks=1;                       
                        FMDataSnd= recPaks;         //lvds reply to ephy, send brd num as 'ack'
                        break; 
                        }
                    //data handler
                    recPaks++;
                    *Buf1500=0;
                    if ((HappyBus.CmdType& 0xff)== eCMD74_LDRAMFLUSH) //end of file flush buffer handler
                        EndWrdCnt= HappyBus.CmdType>>8;
                    else
                        EndWrdCnt= param1;
                    
                    while(param1-- > 0)
                        {
                        d16= *eBufB++;              //write to fpga SD_Ram
                        SDR_RD16= d16;              //store to sdRam1
                        //HISTDAT16= d16;           //write fpga2 Histogram FIFO 
                        if(EndWrdCnt)
                            {
                            EndWrdCnt--;
                            u_SRAM.DwnLd_sSUM += ((d16>>8)&0xff);
                            u_SRAM.DwnLd_sSUM += (d16&0xff);
                            u_SRAM.DwnLd_sCNT += 2;
                            }
                        }
                    //sprintf(tBuf,"Pac %d\r\n",recPaks );
                    //putBuf(0, tBuf,0);
                    //LVDS ACK TO CONTROLLER
                    FMDataSnd= recPaks;             //lvds reply to ephy, send brd num as 'ack'
                    
                    //check if done
                    if(u_SRAM.DwnLd_sCNT==uPhySums.DwnLd_sCNT)
                        {                          
                        sprintf(tBuf ,"File Size Recd FEB: RecdBytes=%-6d   CheckSum=%04X     Total_Packets=%d\r\n",u_SRAM.DwnLd_sCNT, u_SRAM.DwnLd_sSUM&0xffff, recPaks);
                        putBuf(0, tBuf, 0);   
                        //feb controller has a 1mS status port check timeout, delay then send
                        mDelay(1);
                        //send char on lvds to ack load complete
                        FMDataSnd= ACK;
                        }
                    
                    //NOTE PROMPTS GETS SENT BACK ON LVDS ON EXIT HERE   
                    genFlag |= NO_ECHO;                     //no ASCII reply     
                    break; 
                   }                              
                else if (!strcmp(tok, "LI"))            //lvds link init assign id number
                    {    
                    param1=arg_hex(&paramPtr,0);        //get 1st param
                    //NOTE PROMPTS GETS SENT BACK ON LVDS ON EXIT HERE   
                    //genFlag |= NO_ECHO;               //no ASCII reply  
                    HappyBus.myLinkNumber= param1;
                    sprintf(tBuf,"%02X", param1);
                    tBuf[2]=0;      //append extra null need at controllers word reads
                    tBuf[3]=0;      //append extra null need at controllers word reads
                    putBuf(prt, tBuf,4);
                    genFlag &= ~ePOE_INIT_REQ;          //clear flag
                    break;
                    }
                else if (!strcmp(tok, "LCHK"))          //lvds link reinit check
                    {    
                    param1=arg_hex(&paramPtr,0);        //get 1st param
                    //NOTE PROMPTS GETS SENT BACK ON LVDS ON EXIT HERE   
                    //genFlag |= NO_ECHO;               //no ASCII reply  
                    //if(genFlag & ePOE_INIT_REQ)
                        {
                        genFlag &= ~ePOE_INIT_REQ;      //clear flag
                        HappyBus.myLinkNumber= param1;
                        sprintf(tBuf,"%02X", param1);
                        tBuf[2]=0;      //append extra null need at controllers word reads
                        tBuf[3]=0;      //append extra null need at controllers word reads
                        putBuf(prt, tBuf,4);
                        }
                    break;
                    }
                else if (!strcmp(tok, "LSTAB"))     //DATA POOL REQUEST LVDS 'eCMD72_STAB'
                    {                               //takes 3uS, update and set ptrs for bckgnd 'lvds port xmit'
                hHI_TP46;                    
                    //valid command range at the FEB rec coding is 0x70-0x7F for data echo via putBuf()   
                    //statusBlock mostly filled in background by 'OneWireTrigRead()' 
                    if ((HappyBus.CmdType<0x70) || (HappyBus.CmdType>0x80) ) //err trap
                        {
                        HappyBus.PHYrecvErr++;
                        break;
                        }
                        HappyBus.SndWrds= sizeof(PoolData)/2;   //size in words            
                        HappyBus.Src= (snvPTR)PoolData.hdr;     //done, init to src begin
                        *HappyBus.Src++= HappyBus.BrdNumb;      //send data on LVDS PORT
                        *HappyBus.Src++= HappyBus.CmdType;      //send data on LVDS PORT
                        *HappyBus.Src++= HappyBus.SndWrds;      //send data on LVDS PORT
                       
                        uint16* tb= (snvPTR)PoolData.uC;       //use array[0] for board block data, ..array[1,2,3,4]hold fpga data
                        *tb++= uC_FRAM.serNumb;                 //serial number
                        *tb++= s0SP_COUNT;                      //spill cycle counter
                                        
                        //get adc readings 800 nSec
                        *tb++= (uint16)readTemperature()*10;    //int temp * 10, (this then includes 10th-of-a-degree)                        
    
                        //takes 1.5 uSec total
                        for (int i=0; i<16; i++)
                            *tb++= adc_data[i].value;
                                            
                        //afe reads to slow, do in background if needed
                        //*tb++= 0x1111;//readAFE(1, 0x103);               //read 16bit AFE reg trig cntrol reg  
                        *tb++= sTrigCntrl;
                        
                        //readAFE(1, 0x104);                    //read 16bit AFE reg pipeline length  
                        *tb++= sPipeLinDly;                     //pipeline delay (0==256) 12.56nS per cnt

                        //readAFE(1, 0x105);
                        *tb= sSampleLen;                        //read 16bit AFE reg sample length       
                            
                        //statusBlock is filled in bacground by 'OneWireTrigRead()'
                        HappyBus.Src= (snvPTR)PoolData.hdr;     //reset to beg for backgrnd xmits 
                        genFlag |= NO_ECHO;
                        //data sent on lvds via interrupts trigger by uTimer
                        uTimer(4);     
                hLO_TP46;
                    break;
                    }                                
          
                else if (!strcmp(tok, "LDFLASH"))             //Program Flash with file stored in SDRAM
                   {
                    int d16;
                    sprintf(Buf1500,"Cmd 'LDSTAT' shows status, Note: Programming hangs I/O for 20 Sec\r\n\r");
                    putBuf(prt, Buf1500,0);

                    //Load (*.bin file ) to Flash using FPGA2_sdRam2 data, assume cmd 'LDFILE' already got file
                    uPhySums.DwnLd_sCNT=0;
                    uPhySums.DwnLd_sSUM=0;             
                    SET_SDADDR_RDx(0,0,0);              //fpgaOffset, addrHI, addrLO
                    
                    //now do local check to verify
                    //compute checkSum for valid data             
                    for (int i=0; i<u_SRAM.DwnLd_sCNT/2; i++)
                        {
                        d16= REG16(&SDR_RD16);          //data port, using fpga(2of4) sdRam
                        uPhySums.DwnLd_sSUM += ((d16>>8)&0xff);
                        uPhySums.DwnLd_sSUM += (d16&0xff);
                        uPhySums.DwnLd_sCNT+=2;
                        }  
                                         
                    //If file sizes match, Programming FLASH takes about 20 sec
                    d16= LDFLASH(prt);                       //using fpga2 of 1of4 sdRam memory 0x407
                    if(d16) //non zero == error
                        {
                        sprintf(Buf1500,"FEB ERROR\r\n\r");
                        putBuf(prt, Buf1500,0);
                        }
                    break;
                   }          
                else if (!strcmp(tok, "LDFILE"))        //Load data file to SDRAM
                   {
                    char* eBufB= (char*)eRecDatBuf[g_Sock];
                    param1= arg_dec(&paramPtr,1);  
                    LDF(prt, param1, eBufB);            //port, fpga1of4, buf
                    break; 
                   }
                else if (!strcmp(tok, "LDSTAT"))        //Load FEB status
                   {
                    //Valid download Check,  
                     //u_SRAM   regs filled by 'LDRAM() downloaded data
                     //uPhySums regs filled by 'LDRAM() passed from controller on request for valid size+chksum
                    u_SRAM.FL_XFER= 0;
                    if((u_SRAM.DwnLd_sCNT==uPhySums.DwnLd_sCNT) && (u_SRAM.DwnLd_sSUM==uPhySums.DwnLd_sSUM)&& (uPhySums.DwnLd_sCNT !=0)) //match original file specs
                        u_SRAM.FL_XFER= 2;
                    if(u_SRAM.FL_XFER==2)
                        sprintf(Buf1500,"\r\nFL_XFER  = GOOD\r\n");
                    else 
                        sprintf(Buf1500,"\r\nFL_XFER  = UNK\r\n");
                      
                    if(u_SRAM.FL_ERASE==2)
                        sprintf(tBuf,"FL_ERASE = GOOD\r\n");
                    else
                        sprintf(tBuf,"FL_ERASE = UNK\r\n");
                    strcat(Buf1500, tBuf);

                    if(u_SRAM.FL_PGM==2)
                        sprintf(tBuf,"FL_PROG  = GOOD\r\n");
                    else
                        sprintf(tBuf,"FL_PROG  = UNK\r\n");
                    strcat(Buf1500, tBuf);

                    if(u_SRAM.FL_BOOT==2)
                        sprintf(tBuf,"FPGALOAD = GOOD\r\n\r");
                    else
                        sprintf(tBuf,"FPGALOAD = UNK\r\n\r");
                    strcat(Buf1500, tBuf);
                    putBuf(prt, Buf1500,0);
                    break; 
                   }                
                else if (!strcmp(tok, "LINK"))              //LINK, Sets lvds link direction
                    {    
                    param1=arg_dec(&paramPtr,-1);           //get 1st param
                    if (param1==-1)
                        {
                        if (uC_FRAM.LinkDir==1)
                            sprintf(tBuf,"%d= XMIT\r\n", uC_FRAM.LinkDir);
                        else
                            sprintf(tBuf,"%d= REC\r\n", uC_FRAM.LinkDir);
                        putBuf(prt, tBuf,0);
                        break;
                        }
                    if  (param1==0)   {LINK_DIR_LO uC_FRAM.LinkDir= param1;} //hetREG1->DCLR=BIT19
                    else if(param1==1){LINK_DIR_HI uC_FRAM.LinkDir= param1;} // hetREG1->DSET=BIT19
                    else parErr++;
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'M':
                tok[2]= 0;                              //limit string compare to 2 chars
                if (!strcmp(tok, "MU"))                 //pga280 input 'MUX' channel select
                    {    
                    param1=arg_dec(&paramPtr,4);        //get 1st param
                    if(param1<4)
                    {
                    uC_FRAM.pgaMuxCh=param1;                       //global var
                    //16 HDMI Trim Volt Chs per fpga, select 1of4 MuxChip(s) ADG1609
                    if (uC_FRAM.pgaMuxCh==0)      {MUXA3_LO; MUXA2_LO;}
                    else if (uC_FRAM.pgaMuxCh==1) {MUXA3_LO; MUXA2_HI;}
                    else if (uC_FRAM.pgaMuxCh==2) {MUXA3_HI; MUXA2_LO;}
                    else if (uC_FRAM.pgaMuxCh==3) {MUXA3_HI; MUXA2_HI;}
                    }
                    sprintf(tBuf,"MuxCh=%d\r\n", uC_FRAM.pgaMuxCh);
                    putBuf(prt, tBuf,0);
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'N':
                if (!strcmp(tok, "NETRST"))                 //restoring network defaults
                    {
                    param1=arg_dec(&paramPtr,0);            //get 1st param
                    if(param1==192)                         //use fermilab ip setuup 
                        memcpy(&netInfo, &defNetInfo, sizeof(netInfo));
                    else
                        memcpy(&netInfo, &defNetInfo192, sizeof(netInfo));
                    printf("Closing sockets, restarting network driver\r\n\n");
                    mDelay(800);                            //allowtime for msg to xmit
                    wizMain();                              //test code for wiznet 5300
                    mDelay(100);                            //allow wiznet pll logic to stabalize
                    break;
                    }
                else if (!strcmp(tok, "NETSAV"))            //restoring network defaults
                    {
                    netInfo.Valid= VALID;                   //flag as valid
                    FRAM_WR_ENABLE();                       //WREN op-code must be issued prior to any Wr_Op
                    FRAM_WR(fPAGE_Net500, (uint8*)&netInfo, sizeof(netInfo)); //write 2 bytes
                    FRAM_WR_DISABLE();
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'P':
                //tok[4]= 0;
                if (!strcmp(tok, "PACS"))               //PACKET SEND
                   {
                    int loop=0;
                    g_lcnt=0;
                    param1=arg_dec(&paramPtr,1);        //now get 1st repeat param
                    param2=arg_dec(&paramPtr,8);        //now get 2nd size param
                    while (loop++ < param1) 
                        {
                        create_packet(param2);          //byte count even and greater than 2
                        g_EMACTxRdy=1;
                        EMACTransmit(&hdkif_data[0], &pack[0]);
                        //some delay needed else false early done??? ~75uS on '1038 byte packet'
                        uDelay(20);
                        
                        //Set flag that send is pending/active, will be clear in 'EMACTxIntISR()'
                        //EMACTxIntISR(void) must set g_EMACTxRdy=0 for this to work 
                        while(g_EMACTxRdy);

                        g_lcnt++;
                       }
                    break;
                    }
                else if (!strcmp(tok, "PACR"))      //PACVIEW or PACREC
                   {
                  //phyHdr.enable=1; //done under "CASE T1ENA"
                    int key,retVal=0, ActPak;
                    param1=arg_dec(&paramPtr,100); //items count to display
                    //if (param1 > numbPack) param1=numbPack;
                    sprintf(tBuf,"Ethernet PHY RecIntrPerSec= %d\r\n", g_pCntPerSec);
                    putBuf(prt, tBuf,0);
                    sprintf(tBuf,"Enter 'CR' to exit loop\r\n");
                    putBuf(prt, tBuf,0);
                    //phyHdr.msgCnt=0;
                    phyHdr.PacEna=1;
                    for(int i=0;i<param1; i++)
                        {
                        while (g_EMACRxRdy==0)
                            {
                            if (USB_Rec.Cnt) {retVal=1; break; }
                            retVal= SockKeyWait(100, prt, &key);  //wait time in mS, returns FALSE on Timeout
                            if(retVal==1)
                                break;
                            }
                        g_EMACRxRdy=0;
                        ActPak= phyHdr.PacNext;
                        if(ActPak>0) ActPak--;
                        sprintf(tBuf,"Packet %d_of_%d      OrigSize=%d\r\n", i, param1, phyHdr.PacBytCnt[ActPak]);
                        putBuf(prt, tBuf,0);
                        HexDump((char*)phyHdr.PacPayLd512[ActPak], phyHdr.PacBytCnt[ActPak], prt);  //phyHdr.msgBytes[i]);
                        putBuf(prt, "\r\n",2);
                        if(retVal==1)
                          break;
                        }
                    break;
                    }
                else if (!strcmp(tok, "PACE"))              //PACENA 
                   {
                    param1=arg_dec(&paramPtr,0);            //test EMAC port, enable cap bufs for data display
                    if (param1==1)
                        phyHdr.PacEna=1;
                    else
                        phyHdr.PacEna=0;
                    break;
                    }
                else if (!strcmp(tok, "PS"))                //MDIO... Phy link status
                   {
                    param1=arg_dec(&paramPtr,0);            //tesm EMAC port, enable cap bufs for data display
                    if(!Dp83640LinkStatusGet(hdkif_data->mdio_base, 1, 10))
                        sprintf(tBuf,"Link is up\r\n");
                    else
                        sprintf(tBuf,"Link is down\r\n");
                    putBuf(prt, tBuf,0);
                    break;
                    }
                else if (!strcmp(tok, "PR"))                //MDIO... Read Register
                   {
                    volatile uint16 linkStatus = 0U;
                    boolean retVal = TRUE;
                    
                    /* First read the BSR of the PHY */
                    retVal= MDIOPhyRegRead(hdkif_data->mdio_base, 1, (uint32)PHY_BSR, &linkStatus);
                    if(retVal != TRUE)
                        sprintf(tBuf,"read failed\r\n");
                    else
                        sprintf(tBuf,"data= %X\r\n",linkStatus);
                    putBuf(prt, tBuf,0);
                    break;
                    }
                else if (!strcmp(tok, "PWR"))               //POWER ENABLE
                   {
                    param1=arg_dec(&paramPtr,6);            
                    if(param1==0) //all off
                        {
                        CSR0= 3;                            //pwr dwn AFE Chips on this fpga, 3=off
                        CSR1= 3;                            //pwr dwn AFE Chips on this fpga
                        CSR2= 3;                            //pwr dwn AFE Chips on this fpga
                        CSR3= 3;                            //pwr dwn AFE Chips on this fpga
                        sprintf(tBuf,"Power Down All\r\n");
                        }
                    else if(param1==1) //1 on
                        {
                        CSR0 = 0x20;                        //Reset to AFE fifo, cnt, seq, power up
                        sprintf(tBuf,"Power Up GRP1\r\n");
                        }
                    else if(param1==2) //2 on
                        {
                        CSR1 = 0x20;                        //Reset to AFE fifo, cnt, seq, power up
                        sprintf(tBuf,"Power Up GRP2\r\n");
                        }
                    else if(param1==3) //3 on
                        {
                        CSR2 = 0x20;                        //Reset to AFE fifo, cnt, seq, power up
                        sprintf(tBuf,"Power Up GRP3\r\n");
                        }
                    else if(param1==4) //4 on
                        {
                        CSR3 = 0x20;                        //Reset to AFE fifo, cnt, seq, power up
                        sprintf(tBuf,"Power Up GRP4\r\n");
                        }
                    else if(param1==5) //all on
                        {
                        //General reset to AFE fifo, counter, sequencer and power up
                        CSR0 = 0x20;                        //pwr up AFE Chips on this fpga, 0=run
                        CSR1 = 0x20;                        //pwr up AFE Chips on this fpga
                        CSR2 = 0x20;                        //pwr up AFE Chips on this fpga
                        CSR3 = 0x20;                        //pwr up AFE Chips on this fpga
                        sprintf(tBuf,"Power Up All\r\n");
                        }
                    else  {parErr++;   break; }
                    putBuf(prt, tBuf,0);
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'Q':
                if (!strcmp(tok, "QUIT"))                   //'QUIT' close ethernet socket
                   {
                    param2= arg_dec(&paramPtr, 9);          //get 1st param, rtn 0 for no entry
                    if (param2==9) param2=g_Sock;
                    if ((param2>=0) && (param2<4))          //valid sockets 0-3
                        {
                        setSn_CR(param2,Sn_CR_CLOSE);       //close socket
                        }
                     break;
                    }
                else  {parErr=0xf;   break; }
        case 'R':
                if (!strcmp(tok, "RDX"))                    //Binary data out PHY port
                    {
                    uint32_t SpCntWrds;                      
                    
                hHI_TP45;                   
                    param1= arg_hex(&paramPtr,-1);          //get 1st param1, word count
                    param2= arg_hex(&paramPtr,0);           //now get 2nd param2, reset mode
                    //if (param1> (MAX_WORD_PER_FPGA*4));   //limit word count
                    //    param1= (MAX_WORD_PER_FPGA*4);
                                    
                    //0 will stop any active request
                    if (param1==0)
                        {
                        BinStRDX.gSndWrdCnt=0;              //stops any active background xfers
                        FPGA_Trig_Stat.OverMaxCounter=0;    //clr counter
                        break;
                        }
                    //already busy, exit
                    else if (BinStRDX.gSndWrdCnt)           //nonzero, rdb already active
                        break;

                    //ddr port read pointers not reset if param2 nonZero
                    if(param2==0)
                        { LNG_SDR_RD0=0;  LNG_SDR_RD1=0; LNG_SDR_RD2=0;  LNG_SDR_RD3=0; }

                    //Store trigger cnts of all fpga(s) for displayed diagnostics
                    //Also checks for overflows 
                    FPGA_Trig_Cnts_Update();        

                    //start with fpga0 (1of4)
                    FPGA_Trig_Stat.Fpga1of4= 0;    
                    
                    //Now on 1st FPGA, use Words Cnt for PHY xFERs , later FPGAs handled in sendRDX()
                    FPGA_Trig_Stat.WrdToSnd= FPGA_Trig_Stat.WrdCnt[FPGA_Trig_Stat.Fpga1of4]; //use word cnt ON ePHY
                    if (FPGA_Trig_Stat.Mask[FPGA_Trig_Stat.Fpga1of4] == 0)
                        FPGA_Trig_Stat.WrdToSnd=0;

                    SpCntWrds= FPGA_Trig_Stat.TotWrdCnt;            //FPGAs Reg 'Checked for OverMax'
                                    
                    //build Main 'Controller Header'
                    Cntrl_tdcHDR.tSpilTrgCntL= s0TRGCNT_HI;         //reverse now go big endian
                    Cntrl_tdcHDR.tSpilTrgCntH= s0TRGCNT_LO;         //reverse now go big endian
                    Cntrl_tdcHDR.tSpilCyc= s0SP_COUNT;              //16bit read
                    Cntrl_tdcHDR.tMask= ((sIN_MASK3<<12) + (sIN_MASK2<<8) + (sIN_MASK1<<4) + sIN_MASK0);
                    Cntrl_tdcHDR.tID= uC_FRAM.serNumb;
                    Cntrl_tdcHDR.tStatus= sSpillStatus;
                                    
                    //Note: full data count should be requested, else hdr repeats for each 'RDB'
                    //swap high/low 32 bit word count (endian little to big)
                    SpCntWrds += cCntlHdrSizWrd;                    //add cCntlHdrSizWrd for header only
                    uint16_t *hdr= (uint16_t*)&Cntrl_tdcHDR.tSpilWrdCntL;
                    *(hdr+1)= SpCntWrds;
                    *hdr= SpCntWrds>>16;
                    SpCntWrds -= cCntlHdrSizWrd;                    //remove cCntlHdrSizWrd to word cnt
                    //store last header at time of readout for display routine
                    memcpy(&tHDR_STORE, &Cntrl_tdcHDR, sizeof(tHDR_STORE)); //dest, src, bytes cnt

                    //1st send header
                    //send(prt,(uint8*)&Cntrl_tdcHDR,cCntlHdrSizByt);   //send AS Byte count
                    //creatpacket for PHY
                    fill_daq_packet(cCntlHdrSizByt, (u_8Bit*)&Cntrl_tdcHDR); //ByteCount, data buffer ptr
                    g_EMACTxRdy=1;      //'EMACTxIntISR()' g_EMACTxRdy=0 for done status

                    //send data to MAC
                    EMACTransmit(&hdkif_data[0], &packs[0]);      //two or more packet chained
                    //some delay needed else false early done??? ~75uS on '1038 byte packet'
                    uDelay(25);
                    //wait for eTX to finish
                    //set flag that send is pending/active, will be clear in 'EMACTxIntISR()'
                    while(g_EMACTxRdy); //tek
                    
                    //check data count, no data then only header was sent
                    if ( SpCntWrds == 0)                            //no data
                        {
                        BinStRDX.gSndWrdCnt=0;
                        break;
                        }
                    
                    //after 1st packet, background loop will send remaining data using non-zero 'gSndWrdCnt'
                    if (param1 != -1)                               //user req size valid, use it
                        BinStRDX.gSndWrdCnt= param1;                //use user req size
                    else
                        BinStRDX.gSndWrdCnt= SpCntWrds;             //feb computed wrd cnt
                      
                    FPGA_Trig_Stat.RDBreqSiz= BinStRDX.gSndWrdCnt;  //store word cnt for diagnostics
                    BinStRDX.gSndSrc= (lPTR)&SDR_RD16SWP;           //uC DDR ram data reg
                    mStime.gBusy_mSec=0;
                    BinStRDX.gSndPrt= prt;
                    BinStRDX.gSndMode=1;                    //0=reg mode, 1=daq mode
                    sendRDX(prt);
                    break;
                    }
                else if (!strcmp(tok, "RDB"))               //read bin TDC data
                    {
                    sPTR saddr;
                    uint16_t free;
                    uint32_t SpCntWrds;
                    param1= arg_hex(&paramPtr,-1);          //get 1st param, word count
                    param2= arg_hex(&paramPtr,0);           //now get 2nd param, reset mode
                    
                    //note, param1 is a signed 32 int 0x80000000 is negative number                    
                    if (param1==0)                          //0 will stop any active request
                        {BinSt.gSndBytCnt=0; break; }       //stops any active background xfers
                    else if (BinSt.gSndBytCnt)              //already busy, exit
                        break;
                  
                    //ddr port read pointers not reset if param2 nonZero
                    if(param2==0)
                        { LNG_SDR_RD0=0;  LNG_SDR_RD1=0; LNG_SDR_RD2=0;  LNG_SDR_RD3=0; }
                    
                    //Store trigger cnts of all fpga(s) for SendBin() and displayed diagnostics
                    //Also checks for overflows 
                    FPGA_Trig_Cnts_Update();                    
                    SpCntWrds= FPGA_Trig_Stat.TotWrdCnt;            //32bit fpga registered count
                    
                    //start with fpga0 (1of4); do before next line code
                    FPGA_Trig_Stat.Fpga1of4= 0; 
                    
                    //Now on 1st FPGA, save Words to Byte CNT, later FPGAs handled in sendBin()
                    FPGA_Trig_Stat.BytToSnd= FPGA_Trig_Stat.WrdCnt[FPGA_Trig_Stat.Fpga1of4]<<1;
                    if (FPGA_Trig_Stat.Mask[FPGA_Trig_Stat.Fpga1of4] == 0)
                        FPGA_Trig_Stat.BytToSnd=0;
                                       
                    //build Main 'Controller Header'
                    Cntrl_tdcHDR.tSpilTrgCntL= s0TRGCNT_HI;         //reverse now go big endian
                    Cntrl_tdcHDR.tSpilTrgCntH= s0TRGCNT_LO;         //reverse now go big endian
                    Cntrl_tdcHDR.tSpilCyc= s0SP_COUNT;              //16bit read
                    Cntrl_tdcHDR.tMask= ((sIN_MASK3<<12) + (sIN_MASK2<<8) + (sIN_MASK1<<4) + sIN_MASK0);
                    Cntrl_tdcHDR.tID= uC_FRAM.serNumb;
                    Cntrl_tdcHDR.tStatus= sSpillStatus;
                    
                    //Note: full data count should be requested, else hdr repeats for each 'RDB'
                    //swap high/low 32 bit word count (endian little to big)
                    SpCntWrds += cCntlHdrSizWrd;                    //add cCntlHdrSizWrd for header only
                    uint16_t *hdr= (uint16_t*)&Cntrl_tdcHDR.tSpilWrdCntL;
                    *(hdr+1)= SpCntWrds;
                    *hdr= SpCntWrds>>16;
                    SpCntWrds -= cCntlHdrSizWrd;                    //remove cCntlHdrSizWrd to word cnt
                    //store last header at time of readout for display routine
                    memcpy(&tHDR_STORE, &Cntrl_tdcHDR, sizeof(tHDR_STORE)); //dest, src, bytes cnt

                    //ethernet socket check
                    free =getSn_TX_FSR(prt);
                    if (free < 100)                                 //get freesize in (bytes)
                        uDelay(800);                                //should never happen, but allow some wait

                    //1st Send Header, let 'wiznet' do litte/big endian adjust on uC memory Header
                    setMR(getMR() & ~MR_FS);                        //enable byte swap when sending 16bit data
                    send(prt,(uint8*)&Cntrl_tdcHDR,cCntlHdrSizByt); //send AS Byte count                   
                    setMR(getMR() | MR_FS);                         //return to default
                    //check data count, no data then only header was sent
                    if ( SpCntWrds == 0)                            //no data
                        {
                        BinSt.gSndBytCnt=0;
                        break;
                        }
                    
                    //after 1st packet, background loop will send remaining data using non-zero 'BinSt.gSndBytCnt'
                    if (param1 == -1)                       //user req size valid, use it
                        param1= SpCntWrds;             
                    
                    //finally, limit word count
                    if (param1> MAX_WORD_PER_FPGA*4)     
                        param1= MAX_WORD_PER_FPGA*4;

                    FPGA_Trig_Stat.RDBreqSiz= param1;       //store word cnt for diagnostics
                    BinSt.gSndBytCnt= param1<<1;            //words to byte count
                    BinSt.gSndSrc= (lPTR)&SDR_RD16SWP;      //uC DDR ram data reg                    
                    mStime.gBusy_mSec=0;
                    BinSt.gSndPrt= prt;
                    BinSt.gSndMode=1;                       //0=reg mode, 1=daq mode
                    sendBin(prt);
                    break;
                    }
                else if (!strcmp(tok, "RDBR"))              //read REGISTER data as bin
                    {
                    sPTR saddr;
                    uint16_t free;
                    param1= arg_hex(&paramPtr,-1);          //get 1st param, addr to read
                    param2= arg_hex(&paramPtr,-1);          //now get 2nd param, word count
                    param3= arg_hex(&paramPtr,0);           //now get 3nd param, reset mode
                    if (param2> 0x10000000)                 //limit word count
                        param2= 0x10000000;

                    //addr out of range
                    if (param1> 0x1000)
                    { BinSt.gSndBytCnt=0;  break;}          //error
                    //0 will stop any active request
                    if (param1==0)
                        {BinSt.gSndBytCnt=0;  break;}       //stops any active background xfers
                    else if (BinSt.gSndBytCnt) break;       //rdb already active, exit
                    //ethernet socket check
                    free =getSn_TX_FSR(prt);
                    if (free < 100)                         //get freesize in (bytes), 'hdr xmit needs 9 bytes'
                        uDelay(800);                        //should never happen, but allow some wait
                    
                    //after 1st packet, background loop will send remaining data using non-zero 'gSndBytCnt'
                    BinSt.gSndBytCnt= param2<<1;            //use user req size
                    BinSt.gSndMode=0;                       //0=reg mode, 1=daq mode
                    BinSt.gSndSrcSptr= (sPTR)fpgaBase+param1; //uC DDR ram data reg
                    mStime.gBusy_mSec=0;
                    BinSt.gSndPrt= prt;
                    sendBinNonDAQ(prt);
                    break;
                    }
                else if (!strcmp(tok, "RD"))                //16 bit EPI BUSS FPGA READ
                    {
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param
                        { parErr++;  break; }
                    //address only, no board number
                    param1 &= 0xFFFF;
                    sPTR saddr = (sPTR) fpgaBase;
                    saddr += param1;                        //adding to pointer, incr by ptr size
                    g_dat16= *saddr;

                    sprintf(Buf1500,"%04X\r\n",g_dat16);
                    putBuf(prt, Buf1500,0);                 //send to current active port
                    break;
                    }
                else if (!strcmp(tok, "RDSEL"))             //RDB data readout enable/disable
                    {
                    if ( (param1=arg_dec(&paramPtr,-1))==-1)//get 1st param
                        { 
                        if (iFlag & XMIT_PHY_ON)
                            sprintf(Buf1500," Controller RDB Readout Mode =1\r\n");
                        else
                            sprintf(Buf1500," FEB RDB Readout Mode =0\r\n");
                        putBuf(prt, Buf1500,0);                 //send to current active port
                        break; 
                        }
                    if(param1==0) //feb
                        {
                        uC_FRAM.xmitPHY= 0;
                        iFlag &= ~XMIT_PHY_ON;               //disable
                        sprintf(Buf1500," FEB RDB Readout Mode\r\n");
                        putBuf(prt, Buf1500,0);                 //send to current active port
                        }
                    else if(param1==1) //controller
                        {
                        uC_FRAM.xmitPHY= 1;
                        iFlag |= XMIT_PHY_ON;                //enable
                        sprintf(Buf1500," Controller RDB Readout Mode\r\n");
                        putBuf(prt, Buf1500,0);                 //send to current active port
                        }
                    else
                        break;
                    uC_FRAM.xmitPHY= param1;
                    FRAM_WR_ENABLE();                      //WREN op-code issued prior to Wr_Op
                    //DAQ xmit on PHY (on/off)
                    FRAM_WR(fPAGE_0400, (uint8*)&uC_FRAM.xmitPHY, 2); //addr,data,cnt
                    FRAM_WR_DISABLE();
                    break;
                    }
                else if (!strcmp(tok, "RF"))                //16 bit EPI BUSS FLASH READ
                    {
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param
                        { parErr++;  break; }
                    //address only, no board number
                    param1 &= 0xFFFF;
                    sPTR saddr = (sPTR) flashBase;
                    saddr += param1;                        //adding to pointer, incr by ptr size
                    g_dat16= *saddr;

                    sprintf(Buf1500,"%04X\r\n",g_dat16);
                    putBuf(prt, Buf1500,0);                 //send to current active port
                    break;
                    }
                //else if (!strcmp(tok, "RD0"))//'zero'       //system status registers
                //    {
                //    lPTR laddr = (lPTR) param1; 
                //    sprintf(Buf1500,"%08X %08X %08X %08X\r\n",systemREG2->DIEIDL_REG0,systemREG2->DIEIDH_REG1,systemREG2->DIEIDL_REG2,systemREG2->DIEIDH_REG3);
                //    putBuf(prt, Buf1500,0);                 //send to current active port
                //    break;
                //    }
                else if (!strcmp(tok, "RDI"))               //read 32bit data from fpga address and increment
                    {
                    //rdm mode is best for smaller xfers as it does NOT support
                    //background xmits and will tie up all ports until finished
                    sPTR saddr= f_Addr_CSR;
                    int i;
                   //get 1st param now
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param, rd addr hex
                        { param1= 1;}                       //default start addr 1
                    //address only, no board number
                    param1 &= 0x7FFFF;
                    if ( (param2=arg_dec(&paramPtr,-1))==-1)//now get read cnt dec
                        { param2= 16;}                      //cnt per line
                    if (param2 > 256)                       //limit size less than Buf1500;
                        param2= 256;                        //5 chars per word, uses 1280 bytes (+crlf overhead)

                    *Buf1500=0;
                    saddr +=param1;
                    for (i=1; i<param2+1; i++)          //Note: Buf1500 is 1500 bytes
                        {
                        sprintf (tBuf,"%4x ", *saddr++);
                        strcat(Buf1500, tBuf);
                        if (i%16==0)                        //14 words per line, now 16
                            {
                            strcat(Buf1500,"\r\n");
                            putBuf(prt, Buf1500,0);          
                            *Buf1500=0;                     //set string size ==0
                            }
                        }
                    //if data in buffer, send it
                    if(*Buf1500)
                        {
                        strcat(Buf1500, "\r\n");
                        putBuf(prt, Buf1500,0);          
                        }
                    break;
                    }
                else if (!strcmp(tok, "RFI"))               //read 32bit data from FLASH address and increment
                    {
                    sPTR saddr= pFLASHbase;
                    int d16;
                    //get 1st param now
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param, rd addr hex
                        { param1= 0;}                       //default start addr 1
                    //address only, no board number
                    param1 &= 0xFFFFFFF;
                    //User enters BYTE ADDR, change to word addr
                    param1 = param1/2;
                    if ( (param2=arg_dec(&paramPtr,-1))==-1)//now get read cnt dec
                        { param2= 256;}                     //cnt per line
                    if (param2 > 1000)                      //limit size less than Buf1500;
                        param2= 1000;                       //5 chars per line, 1000 char buffer
                    
                    *Buf1500=0;
                    saddr +=param1;
                    for (int i=1; i<param2+1; i++)          //Note: Buf1500 is 1500 bytes
                      {
                       d16= *saddr++;
                       sprintf (tBuf,"%04x ", d16);
                       strcat(Buf1500, tBuf);
                       if (i%8==0)                          //8 words per line
                           {
                           strcat(Buf1500,"\r\n");
                           putBuf(prt, Buf1500,0);          //Note:putBuf to hBus will change fpga ptrs
                           *Buf1500=0;                      //set string size ==0
                           }
                      }
                    putBuf(prt,"\r\n",2);
                    break;
                    }
           else if (!strcmp(tok, "RDM"))                    //read 32bit data from fpga address
                    {
                    //rdm mode is best for smaller xfers as it does NOT support
                    //background xmits and will tie up all ports until finished
                    int index;
                    //get 1st param now
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param, lword read addr hex
                        { parErr++;  break; }
                    //address only, no board number
                    param1 &= 0x7FFFF;
                    if ( (param2=arg_hex(&paramPtr,-1))==-1)//now get 2nd param, lword read cnt hex
                        { parErr++;  break; }
                    param3= arg_dec(&paramPtr, 8);          //now get 3rd param, lword read cnt hex
                    if ((param3<0) || (param3>16))          //limit sizes
                        param3=8;
                    sPTR saddr = (sPTR) fpgaBase;
                    saddr += param1;                        //adding to pointer, incr by ptr size
                    *Buf1500=0;
                    for (int i=1; i<param2+1; i++)          //Note: Buf1500 is 1500 bytes
                      {
                       saddr = (sPTR)fpgaBase+ param1;
                       index= *saddr;

                       sprintf (tBuf,"%4x ",index);
                       strcat(Buf1500, tBuf);
                       //read lower addr to advance pointer
                       if (i%param3==0)                     //words per line
                           {
                           strcat(Buf1500,"\r\n");
                           putBuf(prt, Buf1500,0);          //Note:putBuf to hBus will change fpga ptrs
                           *Buf1500=0;                      //set string size ==0
                           }
                      }
                    break;
                    }
                else if (!strcmp(tok, "RDEBLK"))            //ethernet block read/display 256 bytes
                    {
                    sPTR saddr = (sPTR) wizBase;            //now reading with ethernet offset
                    int short dat16;
                    param1=arg_hex(&paramPtr,0);            //get 1st param, def=0
                    saddr += param1/2;
                    *Buf1500=0;                             //set string size ==0
                    for (int i=0;i<32*4;i++)                    //Note: Buf1500 is 1500 bytes
                      {
                       if (i%8==0)
                           { 
                           if (i%32==0)
                               {
                                strcat(Buf1500,"\r\n");
                                putBuf(prt, Buf1500,0);     //send to current active port
                                *Buf1500=0;                 //set string size ==0
                               }
                           sprintf(tBuf,"\r\n%04X=",((int)saddr&0xffff) );
                           strcat(Buf1500, tBuf);
                           }
                       dat16 = *saddr++;
                       sprintf (tBuf," %04X",dat16&0xffff);
                       strcat(Buf1500, tBuf);
                      }
                    //cptr = (cPTR) WizReg0;
                    //*cptr=0x40;                           //enable memory test mode on wiznet
                    strcat(Buf1500,"\r\n");
                    putBuf(prt, Buf1500,0);                 //send to current active port
                    break;
                    }
                else if (!strcmp(tok, "RESET"))                    
                    {
                      //allow last usb chars to xmit
                      uDelay(1000);
                      _disable_interrupt_();
                      resetEntry();
                      break;
                    }
                else  {parErr=0xf;   break; }
        case 'S':
                if (!strcmp(tok, "STAB"))   //read/return status block
                    {                       //"LSTAB" returns DATA POOL Reqs 'eCMD72_STAB' on LVDS 
                    int mode=0;
                    char hBuf[10];
                    param1= arg_dec(&paramPtr,4);  //get 1st param, fpga to process 1-4
                    if(param1==0)
                        {
                        //putBuf(prt, (char *)HelpMenuSTAB, sizeof(HelpMenuSTAB)-1);        
                        break;
                        }

                    if((param1>4) || (param1<1))
                        {
                        mode=1;
                        param1=1;
                        }

                    stab(param1);           //update data buffer

                    *Buf1500=0;         
                    //read uC header
                    for (int ch=0,j=1; ch<sBLKSIZ22; ch++,j++)        //sSMALLSIZ==22     
                        {
                        sprintf (hBuf,"%4X ",statusBlock[0][ch]);
                        strcat(Buf1500, hBuf);
                        if (j%16==0)                //14 words per line, now 16
                            strcat(Buf1500,"\r\n");
                        }
                    strcat(Buf1500,"\r\n");
                    putBuf(prt, Buf1500,0);         //send to current active port
                    if (mode==1) 
                      break;
                    //loop displaying fpga data
                    for (int fpga=1; fpga<=param1; fpga++)
                        {
                        //reset output buffer for strcat use
                        *Buf1500=0;         
                        //read fpga(s) and HDMI port data
                        for (int ch=0,j=1; ch<sBLKSIZ38; ch++,j++)           
                            {
                            sprintf (hBuf,"%4X ",statusBlock[fpga][ch]);
                            strcat(Buf1500, hBuf);
                            if (j%16==0)                //14 words per line, now 16
                                strcat(Buf1500,"\r\n");
                            }
                        strcat(Buf1500,"\r\n");
                        putBuf(prt, Buf1500,0);         //send to current active port
                        }
                    break;
                    }

               else if (!strcmp(tok, "STAR"))                //timed read setup of 'stab' data
                    {
                    param1= arg_dec(&paramPtr,0);       //get 1st param
                    if (param1==1)
                        { 
                        genFlag |= STAR_REQ;
                        g_Pass=0;
                        sprintf(Buf1500,"Status Update On\r\n");
                        putBuf(sock0, Buf1500,0);       //send to current active port
                        }
                    else
                        {
                        genFlag &= ~STAR_REQ;
                        sprintf(Buf1500,"Status Update Off\r\n");
                        putBuf(prt, Buf1500,0);         //send to current active port
                        }
                    break;
                    }
               else if (!strcmp(tok, "STAT"))
                    {
                    *Buf1500=0;                         //null buffer to be filled
                    headerType(prt);                    //show status registers
                    putBuf(prt, Buf1500,0);
                    putBuf(prt, "\r\n\r",2);
                    break;
                    }
               
               else if (!strcmp(tok, "SOCK"))           //socket status reg 0x403,0x503,...
                    {
                    int d8;
                    uint8 datAr[4];
                    param1=arg_dec(&paramPtr,0);
                    sprintf (Buf1500," Sock Status  xBufSz  rDatSz   xTOut    Connection\r\n");
                    for (int ch=0; ch<sockW3; ch++)     //8sock, only show enabled ones
                        {
                        sprintf (tBuf,"   %1d    %02X     %04X    %04X   %4X      ", ch, getSn_SSR(ch),
                                 getSn_TX_FSR(ch), recDatSz[ch], eTout[ch] );
                        strcat(Buf1500, tBuf);

                        d8= getSn_SSR(ch);
                        if (d8==SOCK_CLOSED)
                            sprintf(tBuf,"CLOSED\r\n");
                        else if (d8==SOCK_ARP)
                            sprintf(tBuf,"ARP\r\n");
                        else if (d8==SOCK_INIT)
                            sprintf(tBuf,"INIT\r\n");
                        else if (d8==SOCK_LISTEN)
                            sprintf(tBuf,"LISTEN\r\n");

                        else if (d8==SOCK_SYNRECV)
                            sprintf(tBuf,"Connect-request(SYN packet)\r\n");
                        else if (d8==SOCK_ESTABLISHED)
                            {
                            getSn_DIPR(ch, datAr);          //ip
                            //getSn_PORTR(ch, port);        //port
                            sprintf(tBuf,"CONNECTED %u.%u.%u.%u\r\n", datAr[0],datAr[1],datAr[2],datAr[3]);
                            }
                        else if (d8==SOCK_CLOSE_WAIT)
                            {
                            sprintf(tBuf,"CLOSE_WAIT\r\n");
                            close(ch);
                            I_STATUS[ch]=SOCK_CLOSED;       //force status check
                            }
                        else if (d8==SOCK_UDP)
                            {
                            if (udpHdr.destip)
                                {
                                char * cptr= inet_ntoa(udpHdr.destip);
                                sprintf(tBuf,"SOCK_UDP  %s" , cptr );
                                strcat(Buf1500, tBuf);
                                sprintf(tBuf," (%u)\r\n",udpHdr.destport);
                                udpHdr.destip= 0;           //zero, next time show any new udp
                                udpHdr.destport= 0;
                                }
                            else
                                {
                                sprintf(tBuf,"SOCK_UDP\r\n");
                                }
                             }
                        else
                            sprintf(tBuf,"*** ERROR ***  StatusReg=%02X (Normal=0x14)\r\n", d8);
                        strcat(Buf1500, tBuf);
                        }
                    if (eTout[0]+eTout[1]+eTout[2])         //anytime show message
                        {
                        sprintf(tBuf," DiagMsg Send_Time_Out_Cnt nonzero, increase retry timeout\r\n");
                        putBuf(tty, tBuf,0);                //send to current active port
                        strcat(Buf1500, tBuf);
                        }    
                    strcat(Buf1500, "\r\n\r");              //extra char if odd byte cnt on 16bit LVDS xfer
                    putBuf(prt, Buf1500,0);  
                                        
                    if(param1==1)
                        {
                        //show  ASCII sock send delays
                        //sprintf(tBuf,"ASCSndDly >20mSec : %d\r\n",  BinSt.gASC_TimDlys[0] );
                        //strcat(Buf1500, tBuf);
                        //sprintf(tBuf,"ASCSndDly 10-20mS : %d\r\n",  BinSt.gASC_TimDlys[1] );
                        //strcat(Buf1500, tBuf);
                        //sprintf(tBuf,"ASCSndDly 5-10mS  : %d\r\n",  BinSt.gASC_TimDlys[2] );
                        //mod to limit printouts
                        sprintf(Buf1500," -------------------------------------\r\n FEB Send Delays all Socks\r\n");
                        sprintf(tBuf," mSec        ASC        BIN \r\n");
                        strcat(Buf1500, tBuf);
                        
                        sprintf(tBuf,"  > 5     %6d     %6d\r\n", BinSt.gASC_TimDlys[2]+BinSt.gASC_TimDlys[0]+BinSt.gASC_TimDlys[0],
                                                                BinSt.gDMA_TimDlys[2]+ BinSt.gDMA_TimDlys[1]+ BinSt.gDMA_TimDlys[0] );
                        strcat(Buf1500, tBuf);
                        
                        sprintf(tBuf,"  1-5     %6d     %6d\r\n", BinSt.gASC_TimDlys[3], BinSt.gDMA_TimDlys[3] );
                        strcat(Buf1500, tBuf);

                        sprintf(tBuf," .1-1     %6d     %6d\r\n\r", BinSt.gASC_TimDlys[4], BinSt.gDMA_TimDlys[4] );
                        strcat(Buf1500, tBuf);
                        putBuf(prt, Buf1500,0);  

                        //show 'rdb cmd sock send delays'
                        //mod to limit printouts
                        //sprintf(tBuf,"BinSndDly >20mSec : %d\r\n",  BinSt.gDMA_TimDlys[0] );
                        //strcat(Buf1500, tBuf);
                        //sprintf(tBuf,"BinSndDly 10-20mS : %d\r\n",  BinSt.gDMA_TimDlys[1] );
                        //strcat(Buf1500, tBuf);
                        //sprintf(tBuf,"BinSndDly 5-10mS  : %d\r\n",  BinSt.gDMA_TimDlys[2] );
                        //clear counters
                        for (int i=0; i<5; i++)
                            {
                            BinSt.gDMA_TimDlys[i]=0;
                            BinSt.gASC_TimDlys[i]=0;
                            }                        
                        }
                    break;
                    }
               else if (!strcmp(tok, "SR0"))                    //SET SDRAM READ ADDR
                    {
                    param1=arg_hex(&paramPtr,0);                //SDram READ ADDR
                    param2=arg_hex(&paramPtr,16);               //READ COUNT
                    sdRamRead(0, param1, param2, prt);          //sdRamRead(chip,addr,count,port)
                    break;
                    }
               else if (!strcmp(tok, "SR1"))                     //SET SDRAM READ ADDR
                    {
                    param1=arg_hex(&paramPtr,0);                //SDram READ ADDR
                    param2=arg_hex(&paramPtr,16);               //READ COUNT
                    sdRamRead(1, param1, param2, prt);          //sdRamRead(chip,addr,count,port)
                    break;
                    }
               else if (!strcmp(tok, "SR2"))                     //SET SDRAM READ ADDR
                    {
                    param1=arg_hex(&paramPtr,0);                //SDram READ ADDR
                    param2=arg_hex(&paramPtr,16);               //READ COUNT
                    sdRamRead(2, param1, param2, prt);          //sdRamRead(chip,addr,count,port)
                    break;
                    }
               else if (!strcmp(tok, "SR3"))                     //SET SDRAM READ ADDR
                    {
                    param1=arg_hex(&paramPtr,0);                //SDram READ ADDR
                    param2=arg_hex(&paramPtr,16);               //READ COUNT
                    sdRamRead(3, param1, param2, prt);          //sdRamRead(chip,addr,count,port)
                    break;
                    }
               else if (!strcmp(tok, "SW0"))                    //SET SDRAM write ADDR
                    {
                    uint32 addr32;
                    addr32=arg_hex(&paramPtr,0);                //SDram Write ADDR
                    param1=arg_hex(&paramPtr,0x1111);           //wr data (def=0x1111)
                    sdRamWrite(0, addr32, param1, prt);
                    break;
                    }
               else if (!strcmp(tok, "SW1"))                    //SET SDRAM FPGA1
                    {
                    uint32 addr32;
                    addr32=arg_hex(&paramPtr,0);                //SDram Write ADDR
                    param1=arg_hex(&paramPtr,0x1111);           //wr data (def=0x1111)
                    sdRamWrite(1, addr32, param1, prt);
                    break;
                    }
               else if (!strcmp(tok, "SW2"))                    //SET SDRAM FPGA2
                    {
                    uint32 addr32;
                    addr32=arg_hex(&paramPtr,0);                //SDram Write ADDR
                    param1=arg_hex(&paramPtr,0x2222);           //wr data (def=0x2222)
                    sdRamWrite(2, addr32, param1, prt);
                    break;
                    }
               else if (!strcmp(tok, "SW3"))                    //SET SDRAM FPGA3
                    {
                    uint32 addr32;
                    addr32=arg_hex(&paramPtr,0);                //SDram Write ADDR
                    param1=arg_hex(&paramPtr,0x3333);           //wr data (def=0x3333)
                    sdRamWrite(3, addr32, param1, prt);
                    break;
                    }
               else if (!strcmp(tok, "S0"))                     //TEST code fpga read/writes
                    {                                           //MT46H128M16LF – 32 Meg x 16 x 4 Banks
                    int datH, datL;
                    int i,j,k;

                    //reset sdRam controller (before writes only, overwrites some data)
                    CSR0= (0x10 | (CSR0));
                    uDelay(500);
                    u_32Bit addr32 = 0x0;
                    if ( (param1=arg_dec(&paramPtr,-1))==-1)  //get 1st param, addr hex to test
                        { param1= 32;}                        //default start addr 'TB CONTRL R/W TEST COUNT REGS'
                    while (1)
                        {
                        //set write fpga Addr
                        SDramWrHI=addr32>>16;
                        SDramWrLO=addr32;
                        for (i=1,j=0,k=0; i<=param1; i++, k++ )
                            {
                            if (k > 7)                      //8 lwords per addr cycle
                                {
                                k=0;
                                addr32 += 32;               //8_32bitsLngWords == 32_bytes
                                //set write fpga Addr
                                SDramWrHI=addr32>>16;
                                SDramWrLO=addr32;
                                }
                            SDR_RD16=j++;
                            SDR_RD16=j++;
                            }
                        //set read fpga addr
                        addr32=0;
                        SDramRdHI=addr32>>16;
                        SDramRdLO=addr32;
                     sprintf(Buf1500, " Cnt   datHI   datLO     WrAddr       RdAddr     Stat   MIG_FIFOCNT\r\n");
                     putBuf(prt, Buf1500,0);                 //send to current active port

                       for (i=1,k=0; i<=param1; i++, k++)
                            {
                            if (k > 7)                      //8 lwords per addr cycle
                                {
                                k=0;
                                //set read fpga Addr
                                addr32 += 32;               //8_32bitsLngWords == 32_bytes
                                //write fpga Addr
                                SDramRdHI=addr32>>16;
                                SDramRdLO=addr32;

                                //sprintf(Buf1500, "%2d   addr32=%X\r\n",i, addr32);
                                //putBuf(prt, Buf1500,0);      

                                putBuf(prt, "\r\n",0);
                                }
                            datH= SDR_RD16;
                            datL= SDR_RD16;
                            sprintf(Buf1500, "%4X   %4X    %4X     %04X%04X     %04X%04X   %04X   %04X\r\n",i, datH, datL, 
                                    SDramWrHI,SDramWrLO, SDramRdHI,SDramRdLO, SDramStat, SDramCnt);
                            putBuf(prt, Buf1500,0);                 //send to current active port
                            }
                            break;
                        }
                    sprintf(Buf1500, "Test Done Addr= %08X\r\n",addr32 );
                    putBuf(prt, Buf1500,0);                     //send to current active port
                    break;
                    }
                else if (!strcmp(tok, "S4"))                    //TEST code fpga read/writes
                    {                                          
                    //MT46H128M16LF – 32 Meg x 16 x 4 Banks
                    //testing 32 Meg x 16 x 4 Banks ==  0x3ff ffff long words
                    //max loop (0x7fffffff)/32Dec == 0x3ff ffff == 67,108,863dec
                    param1=arg_dec(&paramPtr,1);                //get 1st param, fpga 1-4
                    
                    if (param1<1) param1=1;
                    if (param1>4) param1=1;
                    param2=arg_hex(&paramPtr,0x8000000);        //now get 2nd param
                    sprintf(Buf1500, "Max Words=0x8000000 (8E+6),   MT46H128M16LF 32Meg x 16 x 4 Banks\r\n");
                    putBuf(prt, Buf1500,0);                     //send to current active port
                    sdRamTest(param1, param2, prt);             //chip(fpga) 1-4
                    break;
                    }
                else if (!strcmp(tok, "SET"))
                    {
                    int i;
                    if ( (param1=arg_dec(&paramPtr,-1)) ==-1)   //now get 2nd param, option select
                        {
                        dispNet();                              //format network data to ascii
                        putBuf(prt, Buf1500,0);                 //send to current active port
                        break;
                        }
                    unsigned char *shtPtr;                      //load ram stucture with Atmel Flash data page IDINFO_PAGE
                    if      (param1==1) shtPtr = (uCHR*)&netInfo.ipAddr;
                    else if (param1==2) shtPtr = (uCHR*)&netInfo.gateWay;
                    else if (param1==3) shtPtr = (uCHR*)&netInfo.netMask;
                    else if (param1==4) shtPtr = (uCHR*)&netInfo.macAddr;
                    else if (param1==5) shtPtr = (uCHR*)&netInfo.sTelnet0;
                    else if (param1==6) shtPtr = (uCHR*)&netInfo.sTelnet1;
                    else if (param1==7) shtPtr = (uCHR*)&netInfo.sTelnet2;
                    else if (param1==8) shtPtr = (uCHR*)&netInfo.sTelnet3;
                    else if (param1==9) shtPtr = (uCHR*)&netInfo.sTcpTimeOut;
                    else if (param1==10)shtPtr = (uCHR*)&netInfo.sTcpRetry;
                    else break;

                    if (param1<5)
                        {i=0;
                         char * p;
                         p= paramPtr;
                         while (*p)
                              {
                                if (*p == '.')
                                    *p=' ';
                                p++;                            //replace '.' with ' ' until 'cr'
                                i++;
                              }

                         for (i=0; i<netVars[param1-1]; i++)
                           {
                           if (param1==4)
                               param2= arg_hex(&paramPtr,-1);  //now get macAddr hex
                           else
                               param2= arg_dec(&paramPtr,-1);  //now get params dec
                           if (param2<0)
                               { parErr++; break; }
                           *shtPtr++ = (uCHR)param2;
                           }
                        }
                    else
                         {
                         param2 = arg_dec(&paramPtr,-1);        //now get params dec
                         *(uLNG*)shtPtr++ = param2;
                         }
                    break;
                    }
                else if (!strcmp(tok, "SN"))                    //set serial number
                    {
                    param1=arg_dec(&paramPtr,-1);               //get 1st param, password
                    param2=arg_dec(&paramPtr,-1);               //get 2nd param, password
                    //note, struct 'sFRAM_MISC' filled from FRAM on power up
                    if ( (param1==123) && (param2!=-1) )        //simple password followed by sn
                        {
                        uC_FRAM.serNumb= param2;
                        FRAM_WR_ENABLE();                      //WREN op-code issued prior to Wr_Op
                        //update serial number only
                        FRAM_WR(fPAGE_0400, (uint8*)&uC_FRAM.serNumb, 2); //addr,data,cnt
                        FRAM_WR_DISABLE();
                        sprintf(tBuf,"New SerNumb=%d\r\n\n",uC_FRAM.serNumb);
                        }
                    else
                        sprintf(tBuf,"SerNumb=%d\r\n\n",uC_FRAM.serNumb);
                    putBuf(prt, tBuf,0);                        //send to current active port
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'T': if (!strcmp(tok, "TH"))
                    {   
                    float tempF;
                    tempF= readTemperature();
                    sprintf(tBuf,"TempF=%2.1f\r\n", tempF);
                    putBuf(prt,tBuf,0);                     //send it
                    break;
                    }
                else if (!strcmp(tok, "TRIG"))              //TRIG SOURCE
                    {   
                    uint16_t d16;  
                    param1=arg_dec(&paramPtr,-1);           //get 1st param
                    
                    if (param1==-1)
                        {
                        if (uC_FRAM.TrgSrcCh==1)
                            sprintf(tBuf,"%d= LEMO PULSE MODE\r\n", uC_FRAM.TrgSrcCh);
                        else
                            sprintf(tBuf,"%d= RJ45 FM MODE\r\n", uC_FRAM.TrgSrcCh);
                        putBuf(prt, tBuf,0);
                        break;
                        }
                    d16 = sTrigCntrl;
                    if(param1==0)      {SRCSEL_LO  
                                        uC_FRAM.TrgSrcCh= param1; 
                                        hHI_ShtDwn; 
                                        sTrigCntrl=(d16|FMHI_PULLO);}  //rj45, set FPGA Reg
                    else if(param1==1) {SRCSEL_HI  
                                        uC_FRAM.TrgSrcCh= param1; 
                                        hLO_ShtDwn; 
                                        sTrigCntrl=(d16&~FMHI_PULLO);} //lemo, set FPGA Reg
                    else parErr++;
                    break;
                    }
                else  {parErr=0xf;   break; }
        default:
                if (prt>=sock2)
                putBuf(prt,"\r\nsyntax error?\r\n",0);      //send to current active port
                break;
        }//end switch(*tok)

    if (parErr)
        {
        if(parErr==0xf)
            putBuf(prt,"\r\n?cmd\r\n",8);                   //cmd err, send to current active port
        else
            putBuf(prt,"\r\n?parm\r\n",9);                  //param err, send to current active port
        }
    if (genFlag & NO_ECHO)
        genFlag &= ~NO_ECHO;                                //one time 
    else
        { 
        if (prt!=ePhyIO)
            putBuf(prt,">",1);                              //new line prompt all telnet ports
        }
   
return 0; //end of process()
}




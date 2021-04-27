
//********************************************************
//  @file Mu2e_Cntrl_DAQ_LVDS.c
//  Fermilab Terry Kiper 2016-2021
//
//  RM48 Code
//  RM48 Hercules device supports little-endian [LE] format
//
//  Program and Read LVDS Input Ports
//********************************************************

#include "sys_common.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "het.h"
#include "gio.h"
#include "hw_reg_access.h"
#include "ver_io.h"
#include "ver_ePHY.h"
#include "Mu2e_Ctrl_Misc.h"
#include "Mu2e_Ctrl.h"

extern const    int  offPHYDATAPORT[];
extern const    int  oFMData30[];
extern const    int  PHYSTATUSBIT[];

extern volatile struct msTimers  mStime; 
extern struct   HappyBusReg HappyBus;   //cmdlen, brdNumb, cmdtype, cntrecd, active, prt, timer, stat, ptrD,ptrS;
extern char     tBuf[];
extern uint16   lvdsBuf128[];
extern uint32   genFlag;
extern int      adc_ms;                 //using adc update time as ref for link re-init
extern char     Buf1500[];

extern struct   sLVDS_ePHY_REG IOPs[];  //testing link assignment regs structure
//extern uint16_t POE_PORTS_ACTIVE[];     //24 port plus 1 to hold 'act port cnt'


//LVDS PORT Testing
int pfmget(int rg45Port)                    //clears buffer if data avail
{
    int d16, timo=0;
    sPTR prtAddrS, prtAddrD;
                   
    if (rg45Port<=8)                        //fpga(s)..fpgaBase0-fpgaBase3
        {
        prtAddrD= (sPTR)fpgaBase1+(oFMData30[rg45Port]);
        prtAddrS= (sPTR)fpgaBase1+oFMStat40;
        }
    else if (rg45Port<=16) 
        {
        rg45Port -= 8;
        prtAddrD= (sPTR)fpgaBase2+(oFMData30[rg45Port]);
        prtAddrS= (sPTR)fpgaBase2+oFMStat40;
        }
    else if (rg45Port<=24) 
        {
        rg45Port -= 16;
        prtAddrD= (sPTR)fpgaBase3+(oFMData30[rg45Port]);
        prtAddrS= (sPTR)fpgaBase3+oFMStat40;
        }
                        
    d16= *prtAddrS;
    d16= d16 & PHYSTATUSBIT[rg45Port];          //status bit for 1of8 ports
    if (d16==1)        //0=return status no data
      return 0;
    
    //mode=0, read to clear buffer  
    while(d16==0)
        {
        d16= *prtAddrD;                         //port recd' data
        d16= *prtAddrS;
        d16= d16 &  PHYSTATUSBIT[rg45Port];     //status 1of8 ports
        //tek mar2018 add timeout       
        if(timo++> 4096)
          return 0;
        }    
    return 1;   //return status, 1=data rec'd and cleared
}                




//LVDS FM REC FIFO SIZE = 1024 WORDS 
//lvds fm data avail?  
//note: Commands are sent to FEB using command "LC" on ePHY link
//only "LC" request data from FEB, data returns here on FM-LVDS link 
//pooled data not received here, see 'lc_LSTAB()' in 'Mu2e_Ctrl_DAQ_PHY.c'
//
// Data received here has a 3 word header
//      Word 1 Port Number 1of24
//      Word 3 Req Cmd Type
//      Word 4 Return word Count
//      Then requested data
//
//How do we get into this lvds port checking routine
//  entry:
//          HappyBus.PoeBrdCh for lvds buffer status check
//  exit;
//      if HappyBus.WaitCnt > 0, expecting reply data afer 'LC' command
//      if HappyBus.WaitCnt=0, last command finished 
//     
//  This function check status reads 1 word of data and exits
//  LVDSBusCheck() takes 200nS(no data), 2.5uS(if data, gets 1 word) 
//
int HappyBusCheck()
{
    u_16Bit d16;
    static int BinMode=0, CmdLenB;
    static int err=0;
    char CHARS[4];
    u_16Bit D16;
    int seq;
        
    seq= HappyBus.PoeBrdCh;
    d16= *(u_16Bit*)IOPs[seq].FM40_STAp;    //check data avail status
    
    if ( (d16& IOPs[seq].ePHY_BIT)==0)      //0= data available
        { 
        HappyBus.CntRecd++;          
        //1st check the normal 3 word header
        if (HappyBus.CntRecd <4)            //new reply,save hdr params
            {
            d16= REG16(HappyBus.FM_DAT);    //get lvds data word
            //check for invalid 0
            if(HappyBus.CntRecd==1)         //1st HDR word
                {
                BinMode=0;
                if ((d16 != HappyBus.PoeBrdCh) && (d16 !=0 ))  //=0 for re-init req   
                    err++;                  //error
                }
            else if(HappyBus.CntRecd==2)    //2nd HDR word
                {
                if (d16 == eCMD75_CONSOLEBIN)
                    BinMode=1;
                else if (d16 != HappyBus.CmdType)
                    err++;                  //error unless special case of binary data returned
                }
            else 
                {
                if (d16<2)    
                    err++;                  //error
                else                        //3rd HDR word, data len>3
                    HappyBus.CmdLenB= d16;  //returned data length
                }
            if(err)
                HappyBus.FMRecvErr++;
            HappyBus.WaitCnt= 20000;        //wait 10mS, ~500nS per HappyBusCheck() call
            }
        //header checking done, now handle returned data
        else 
            //LVDS Data words from FEB in blocks up to 256 words
            //     Delays of 250us or 800uS between blocks
            //     Each word is 250nS on wire
            //WaitCnt is a passcount of times we return to check data avail status
            //Break in data is greater than WaitCnt=400, assume packet is done
            //If packet timeout then next packet will include 3 word header before data
            {      
            if (BinMode)                                    //trap binary data for display
                {
                D16= *(u_16Bit*)IOPs[seq].FM30_DATp;
                if (err==0)  
                    {
                    sprintf(tBuf,"%4X ",D16 );
                    putBuf(HappyBus.Socket,tBuf, 5);        //HappyBus.ASCIIPrt ASCII xMIT I/O, ie SOCK,TTY,ePHY
                    }
                HappyBus.WaitCnt= 400;                      //wait 800uS, ~2uS per Null BackGround pass
                }
            else if(HappyBus.CmdType== eCMD71_CONSOLE)
                {
                *(u_16Bit*)CHARS=*(u_16Bit*)IOPs[seq].FM30_DATp;
                if (!err)  
                    {
                    //Console display here, allow ascii chars and crlf
                    if ((( (CHARS[0]>= ' ') && (CHARS[0]< 128)) || (CHARS[0]=='\r') || (CHARS[0]=='\n')) && 
                        (( (CHARS[1]>= ' ') && (CHARS[1]< 128)) || (CHARS[1]=='\r') || (CHARS[1]=='\n')) ||
                            CHARS[1]==0 )    //allow null 2nd char if only char is valid
                        //2 ASCII Chars per 16Bit port read, THIS SLOWS DOWN RECEIVING DATA
                        putBuf(HappyBus.Socket,(char*)CHARS, 2); //HappyBus.ASCIIPrt ASCII xMIT I/O, ie SOCK,TTY,ePHY
                    else
                        CHARS[1]=0;                     //added for testing only
                    }
                CmdLenB= HappyBus.CmdLenB;
                if (HappyBus.CntRecd== CmdLenB)
                    {
                    HappyBus.CntRecd=0;                 //done for this request 
                    HappyBus.WaitCnt=3000;              //done, but maybe more being sent
                    }
                else
                    HappyBus.WaitCnt=3000;              //not done, but maybe more being sent
                }
            }
        }
    
    //no data avail?, incr time out counter
    if(HappyBus.WaitCnt)                            //~2uS per Null pass
        {
        HappyBus.WaitCnt--;
        if (err)                                    //clear error after all done            
            err=0;
        }
    return 0;
}



//Check data buffer status of active lvds channel
//returns status and gets data if avail in var d16
//used by LDPGMFEB()
int LvdsDataAvailNotUsed(int* d16)                  //0= data available  
{
    int stat;
    stat= *(u_16Bit*)HappyBus.FM_STA;        //read LVDS ports status
    stat= stat & HappyBus.ePHY_DATAVAIL_BIT; //0= data available    
    *d16= REG16(HappyBus.FM_DAT);
    return stat;
}



//xmit command on ePhy link to feb, called by command "LC"
int linkCmdFunc(int prt, char* paramPtr)
{                                       //SEND CMD STRING ON ePHY port to FEB                      
    //int len=0;               
    //restore original port addr structure
    assignLinkPort(HappyBus.SavePrt, 1);                    
    HappyBus.CmdType= eCMD71_CONSOLE;   //happybus command type
    //PHY_LOADER_CONSOLE set HappyBus.Active and HappyBus.Timer
    PHY_LOADER_CONSOLE(paramPtr, prt, HappyBus.PoeBrdCh, eECHO_ON, 0);   //cmdBuf,sock,Port,echoMode,broadCast                   
    return 1;
}

        
//Empty by reading lvds fifos upto 4096 words using status flag
//used by cmd 'LDFEB'
int EmptyAll_LVDS_FIFOs()
{
    int cnt;
    //Try global reset, if this works us it, lower code is backup clearing
    *(uSHT*)IOPs[POE01].FM41_PARp= FMRstBit8;//FPGA2 lvds fifo buf and parErr clr              
    *(uSHT*)IOPs[POE09].FM41_PARp= FMRstBit8;//FPGA3 lvds fifo buf and parErr clr               
    *(uSHT*)IOPs[POE17].FM41_PARp= FMRstBit8;//FPGA4 lvds fifo buf and parErr clr               
    
    //if global reset works below clearing not needed
    for (int seq=1, d16; seq<25; seq++)
        {
        cnt=0;
        d16= *(u_16Bit*)IOPs[seq].FM40_STAp;    //check data avail status
        while( (d16& IOPs[seq].ePHY_BIT)==0)    //0= data available per chan bit
            {
            d16=*(u_16Bit*)IOPs[seq].FM30_DATp;   //dump data
            if(cnt++<4096)
              break;
            d16= *(u_16Bit*)IOPs[seq].FM40_STAp;//check data avail status
            }
        }
    return 1;
}

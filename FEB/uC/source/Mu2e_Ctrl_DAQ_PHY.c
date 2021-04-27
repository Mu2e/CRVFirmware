
//********************************************************
//  @file Mu2e_Cntrl_DAQ_PHY.c
//  Fermilab Terry Kiper 2016-2021
//
//  RM48 Code
//  RM48 Hercules device supports little-endian [LE] format
//
//  Program and Read Ethernet PHY Ports
//********************************************************

#include "sys_common.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "het.h"
#include "gio.h"
#include "hw_reg_access.h"

#include "ver_io.h"
#include "Mu2e_Ctrl_Misc.h"
#include "Mu2e_Ctrl.h"
#include "ver_ePHY.h"

extern struct   ePHYregS ePHY_HDR;
extern struct   uC_Store u_SRAM;        //DwnLdSdRamCnt, DwnLdSdRamSum
extern char     Buf1500[];
extern char     tBuf[];
extern struct   HappyBusReg HappyBus;   //cmdlen, brdNumb, cmdtype, cntrecd, active, prt, timer, stat, ptrD,ptrS;
extern struct   ePHYregS ePHY_HDR;
extern struct   uC_Store u_SRAM;        //DwnLdSdRamCnt, DwnLdSdRamSum

extern uint16   lvdsBuf128[];
extern uint16_t POE_PORTS_ACTIVE[];     //24 port plus 1 to hold 'act port cnt'
extern uint8_t	ePHYAdd[];
extern uint32   genFlag, iFlag;

extern const int oFMData30[];
extern const int PHYSTATUSBIT[];
extern const int oFMWrdCnt38[];
extern struct    sLVDS_ePHY_REG IOPs[];  //testing link assignment regs structure

extern struct   msTimers mStime;
extern struct   sLVDS lvLnk;

//test code
extern int g_tempDelay;
extern struct ky k28;
extern struct uBuns uBReq;


char   cmdBuf20[20];
char   eTuf200[200];

int PHY_LOAD_DAQ1(int cmdType, int phyPort, sPTR, int cnt);

#define ePHY_uBReqMax64   64
struct ePHYubReq{
   unsigned char u_CMDTYP;                      //cmdType  1 bytes
   unsigned char u_uBcnt;                       //uB Words per packet (two words per uB request)
   unsigned short u_uB64wBuf[ePHY_uBReqMax64];  //uB reqs
};


struct ePHYdaqS ePHY_HDR_DAQ= {
                    {0xD2D1, 0xD4D3, 0xD6D5},   //mac dest  ,One time init only used for diag pac viewing
                    {0xE2E1, 0xE4E3, 0xE6E5},   //mac src   ,One time init only used for diag pac viewing
                    0X3000,5,1,eCMD_DAQ_DY2,    //PayLoadLen(ByteSwap), Cmdlen, brdNum, cmdType
                    0x00,0x00,0x00,0x00, 'u','B','U','N'}; //DataBuf uBIT16[uBunSz256]

struct ePHYregS ePHY_HDR_LSTAB= {               //get pooled data
                    {0xD2D1, 0xD4D3, 0xD6D5},   //mac dest  ,One time init only used for diag pac viewing
                    {0xE2E1, 0xE4E3, 0xE6E5},   //mac src   ,One time init only used for diag pac viewing
                    0X3000,5,1,eCMD72_STAB,     //PayLoadLen(ByteSwap), Cmdlen, brdNum, cmdType
                    'L','S','T','A','B','\r'};  //cmdString--u_CmdBuf[uCmdBufSz44]

extern u_16Bit      stab_Pool[24][eCMD72_MAX_WRDs174];  //data pool


struct ePHYregS ePHY_HDR_CON= {
                    {0xD2D1, 0xD4D3, 0xD6D5},   //mac dest  ,One time init only used for diag pac viewing
                    {0xE2E1, 0xE4E3, 0xE6E5},   //mac src   ,One time init only used for diag pac viewing
                    0X3000,5,1,eCMD71_CONSOLE,  //PayLoadLen(ByteSwap), Cmdlen, brdNum, cmdType
                    'H','E','L','P','\r'};      //cmdString--u_CmdBuf[uCmdBufSz44]

struct ePHYubReq ePHY_HDR_uBReq;



//Ethernet PHY FIFO Loader
int ePHY_FIFO_LOAD(int cmdSIZE)
{      
    //eDST and e_SRC setup only neded for diag, doesn't not care in our useage
    //ePHY_HDR.e_DST=   One time init already done, 6 chars len 
    //ePHY_HDR.e_SRC=   One time init already done, 6 chars len
  
    //load pac Header to ePHY FIFO      (src,dest,cnt)
    movStr16_NOICDEST((snvPTR)&ePHY_HDR,  HappyBus.ePHY_SndFIFO, ePHY_HDR_SZ);      //load header
    
    //load ePHY eCommand to ePHY FIFO   (src,dest,cnt)
    movStr16_NOICDEST((snvPTR)&ePHY_HDR.u_CmdBuf, HappyBus.ePHY_SndFIFO, cmdSIZE);  //load ASCII CMD 
        
    //move FPGA data to uC Buffer       (src,dest,cnt)
    movStr16_NOICSRC((sPTR)&f1_RD16, (snvPTR)&Buf1500, ePayLdSizLimit256);          //get sdRam data
    //move uC Buffer to ePHY FIFO       (src,dest,cnt)
    movStr16_NOICDEST((snvPTR)&Buf1500, HappyBus.ePHY_SndFIFO, ePayLdSizLimit256+2);//load Payload +2(CheckSum)
    return 0;
}


int ePHY_FIFO_LOAD_CHKSUM(int cmdSIZE)
{      
    //eDST and e_SRC setup only neded for diag, routing doesn't care in our useage
    //ePHY_HDR.e_DST=  One time init already done, 6 chars len 
    //ePHY_HDR.e_SRC=  One time init already done, 6 chars len
  
    //load pac dest,src,len Header to ePHY FIFO
    movStr16_NOICDEST((unsigned short*)&ePHY_HDR, HappyBus.ePHY_SndFIFO, ePHY_HDR_SZ);      //load header                       
    //load ePHY eCommand to ePHY FIFO
    movStr16_NOICDEST((unsigned short*)&ePHY_HDR.u_CmdBuf, HappyBus.ePHY_SndFIFO, cmdSIZE); //load ASCII CMD 
    
    //move FPGA data to uC Buffer
    movStr16((sPTR)&u_SRAM, (unsigned short*)&Buf1500, sizeof(u_SRAM));                     //get(DwnLd_sCNT,DwnLd_sSUM)
    //move data from uC Buffer to ePHY FIFO
    movStr16_NOICDEST((unsigned short*)&Buf1500, HappyBus.ePHY_SndFIFO, ePayLdSizMinWrds+2); //load Payload +2(CheckSum)      
    return 0;
}



//Ethernet PHY FIFO Loader
int ePHY_FIFO_LOAD_ALL(int cmdSIZE)
{      
    //eDST and e_SRC setup only neded for diag, doesn't not care in our useage
    //ePHY_HDR.e_DST=   One time init already done, 6 chars len 
    //ePHY_HDR.e_SRC=   One time init already done, 6 chars len
  
    //load pac Header to ePHY FIFO      (src,dest,cnt)
    movStr16_NOICDEST((snvPTR)&ePHY_HDR,  &ePHY301_BCAST_DATA, ePHY_HDR_SZ);       //load header
    
    //load ePHY eCommand to ePHY FIFO   (src,dest,cnt)
    movStr16_NOICDEST((snvPTR)&ePHY_HDR.u_CmdBuf, &ePHY301_BCAST_DATA, cmdSIZE);   //load ASCII CMD 
        
    //move FPGA data to uC Buffer       (src,dest,cnt)
    movStr16_NOICSRC((sPTR)&f1_RD16, (snvPTR)&Buf1500, ePayLdSizLimit256);      //get sdRam data
    //move uC Buffer to ePHY FIFO       (src,dest,cnt)
    movStr16_NOICDEST((snvPTR)&Buf1500, &ePHY301_BCAST_DATA, ePayLdSizLimit256+2); //load Payload +2(CheckSum)
    return 0;
}


int ePHY_FIFO_LOAD_CHKSUM_ALL(int cmdSIZE)
{      
    //eDST and e_SRC setup only neded for diag, routing doesn't care in our useage
    //ePHY_HDR.e_DST=  One time init already done, 6 chars len 
    //ePHY_HDR.e_SRC=  One time init already done, 6 chars len
  
    //load pac dest,src,len Header to ePHY FIFO
    movStr16_NOICDEST((unsigned short*)&ePHY_HDR, &ePHY301_BCAST_DATA, ePHY_HDR_SZ);      //load header                       
    //load ePHY eCommand to ePHY FIFO
    movStr16_NOICDEST((unsigned short*)&ePHY_HDR.u_CmdBuf, &ePHY301_BCAST_DATA, cmdSIZE); //load ASCII CMD 
    
    //move FPGA data to uC Buffer
    movStr16((sPTR)&u_SRAM, (unsigned short*)&Buf1500, sizeof(u_SRAM));                     //get(DwnLd_sCNT,DwnLd_sSUM)
    //move data from uC Buffer to ePHY FIFO
    movStr16_NOICDEST((unsigned short*)&Buf1500, &ePHY301_BCAST_DATA, ePayLdSizMinWrds+2); //load Payload +2(CheckSum)      
    return 0;
}





//Send Data File to FEB uisng ePHY port
//feb will after load done and a 1mS delay send char 'ACK_'  on lvds if load is good
//feb ACKs on command 'LDRAM'
//poePort must already be set by assignLinkPort()
//******************************************************
//assume bin file data already into local sdRAM on FPGA2
//******************************************************
//Input :   Var xports==1 is single port xmit, 24==send to all ports
//RetVal:   1==okay, 0==fail
//
int SEND_2_FEB(int sock, int xports)
{
    static int LDpass=0;  
    int errFLAG=0, cmdSIZE, pacCnt=0, retVal;
	int SendWrdCnt= u_SRAM.DwnLd_sCNT/2;
                    
	//set write sdRAM Addr via special sequence
	SET_SDADDR_RDx(fPtrOffset2,0,0);        //set sdRam2 RD ADDR
                            
	//clear rec data fifo (old data)                    
	REG16(HappyBus.FM_PAR)= FMRstBit8;      //BIT8, clear buffers
                    
	//PAYLOAD_SIZE + PAYLOAD are now in cBuf[]  
	HappyBus.CmdType= eCMD71_CONSOLE;
	HappyBus.CmdLenB= 1;                    //echo (returned) cmd length

	//include 2 char terminators in word count
	cmdSIZE= (strlen(ePHY_HDR.u_CmdBuf)/2);  
    for(int cnt=0; cnt < cmdSIZE; cnt++)
            ePHY_HDR.u_CmdBuf[cnt++]= 0;    //pack all nulls

	ePHY_HDR.e_PAYLDLEN= ePayLdSizLimit256; //send command "LDRAM" to FEB                   
	sprintf(ePHY_HDR.u_CmdBuf,"LDRAM\r");   //Must be even cnt
	//uController header size before payload data
                    
	ePHY_HDR.u_BRDNUM= HappyBus.PoeBrdCh; 
	ePHY_HDR.u_CMDTYP= eCMD73_LDRAMINIT;                    
	ePHY_HDR.u_CMDLEN= ePayLdSizLimit256;

	while (1)
		{
        //wait for ePHY Xmit FIFO empty  
        if (PhyXmitBsy(HappyBus.PoeBrdCh))         //check 1, assume other done
            {
            //comment out print line at some point
            printf("K28SEND Phy Busy timeout\r\n");
            uBReq.errFLAG++;
            }
        //d16= *(sPTR)(fpgaBase1+(oPHY13_WRDCNT*2)); //my testing status
                
		//special case flag.. to reset sdRam pointer in FEB before data arrives
		if (pacCnt!=0) 
			ePHY_HDR.u_CMDTYP=  eCMD71_CONSOLE;  
                        
		//if end, send reg packet size, command type 'eCMD74_LDRAMFLUSH' will flag as to real data count
		if (SendWrdCnt < ePayLdSizLimit256) 
			{
			ePHY_HDR.u_CMDTYP= eCMD74_LDRAMFLUSH;  //FEB needs last packet indicator
            ePHY_HDR.u_CMDLEN= SendWrdCnt;
			SendWrdCnt= 0;
			}
		else
            {
            if (pacCnt!=0)                          //1st pass inits feb, sending chksum data this pass
                SendWrdCnt -= ePayLdSizLimit256;    //adj remaing word to send cnt 
            }
				
		//load ePHY FIFO with full packet, ready for xmit
		if (pacCnt!=0)
            {
            if(xports==24)
                ePHY_FIFO_LOAD_ALL(cmdSIZE);        //send 'file data'  3 FPGAs
            else
                ePHY_FIFO_LOAD(cmdSIZE);            //send 'file data'
            }
		else
            {
            if(xports==24)
                ePHY_FIFO_LOAD_CHKSUM_ALL(cmdSIZE); //send filesize and checksum 1st for comparison
            else 
                ePHY_FIFO_LOAD_CHKSUM(cmdSIZE);     //send filesize and checksum 1st for comparison
            }

        //d16= *(sPTR)(fpgaBase1+(oPHY13_WRDCNT*2));  //my testing status        
        //ePHY FIFO has now loaded, send it
        if(xports==24)
            {
            //Send Packet on all ports
            //*IOPs[PrtPOE].ePHYXMT_ENAp= IOPs[PrtPOE].ePHY_BIT;  //send one port, else all
            //Load data to all ports, use address 'ePHY_BCAST_DATA'   
            *IOPs[1].ePHY0E_XMSKp= 0xFF;        //enable all port   
            *IOPs[9].ePHY0E_XMSKp= 0xFF;        //enable all port   
            *IOPs[17].ePHY0E_XMSKp=0xFF;        //enable all port   
            //send now using ePhy link global_24 data (broadcast) xmit    
            ePHY302_BCAST_XMIT= 1;
            }    
        else
            {
            ePHY_SEND(HappyBus.PoeBrdCh,0);     //1==send to all
            }        
		pacCnt++;                               //diag counter                   
		//wait for 'FEB ACK' on LVDS 'FM' Port 1of24
		LDpass=0;
        int d16; 
		do  {
			uDelay(100);
			if(LDpass++>50000)                    //wait 50 mSec, plenty of time
                {errFLAG++; break;}           
            //mode=0, read to clear buffer  
            d16= *(u_16Bit*)HappyBus.FM_DAT;               //port recd' data
            d16= *(u_16Bit*)HappyBus.FM_STA;
            d16= d16 &  HappyBus.ePHY_DATAVAIL_BIT; //status 1of8 ports
            //return status, 1=data rec'd and cleared            
			}while (d16==0);
                        
		//break when done or errFLAG
	    if ((SendWrdCnt==0) || errFLAG)
			break;   
        uDelay(500);
       }
	//done
	if(errFLAG)
        {
		sprintf(Buf1500,"CNTRL: SENT =%-d BYTES  CHKSUM=%X,   FEB REC'D PACs=%d (FEB ACK FAILED)\r\n",u_SRAM.DwnLd_sCNT, u_SRAM.DwnLd_sSUM, pacCnt);
        putBuf(sock, Buf1500,0);      
		sprintf(Buf1500,"CNTRL: Errors may be caused FPGA DAQ Process, Reset board and try aqain\r\n");
        putBuf(sock, Buf1500,0);      
        retVal=0;           //0=FAILED
        }
	else
        {
		sprintf(Buf1500,"CNTRL: SENT =%-d BYTES  CHKSUM=%X,   FEB REC'D PACs=%d (FEB 'ACK' ALL Packets)\r\n",u_SRAM.DwnLd_sCNT, u_SRAM.DwnLd_sSUM, pacCnt+1);
        putBuf(sock, Buf1500,0);      
        retVal=1;           // 1=PASSED
        }
    //febs send final 'ACK_' char (as good download) after a 1mS delay so pfmget() wont clear the buffer
    //this ACK_ char can be read out later to show load is done and VALID    
	return retVal;
}


//assign ePHY link port to use 1of24 (as 8 per 1of3 fpgas)
//set all pointers needed for both ePHY and LVDS ports
//clear LVDS RECV BUFFER
//function time required 1uS
//new pointer structure IOPs[1of24] is prebuilt with there ptrs.
//menu driven functions may still be on older assignLinkPort()
int assignLinkPort(int brdNum, int reset)
    {                                   //Set Bus Board Numb 1-24, 0=all
    //int fbase;
    if (HappyBus.SavePrt==0) 
        HappyBus.SavePrt=1;             //just in case, force valid ptrs
    if (brdNum==0) 
        brdNum=1;                       //just in case, force valid ptrs

    HappyBus.PoeBrdCh= brdNum;
    
    //Newer coding
    //todo check why pooling on 2nd brd fails
    HappyBus.FM_DAT = (uINT)IOPs[brdNum].FM30_DATp;
    HappyBus.FM_STA = (uINT)IOPs[brdNum].FM40_STAp;
    HappyBus.FM_PAR = (uINT)IOPs[brdNum].FM41_PARp;

    HappyBus.ePHY_DATAVAIL_BIT= IOPs[brdNum].ePHY_BIT;
    HappyBus.ePHY_RecFIFO= IOPs[brdNum].ePHY20_RECBUFp;
    HappyBus.ePHY_CH_ENA= IOPs[brdNum].ePHY0E_XMSKp;
    
    HappyBus.ePHY_SndFIFO=IOPs[brdNum].ePHY11_BCAST_FILLFIFOp;
    HappyBus.ePHY_STAT=   IOPs[brdNum].ePHY16_STAp;
    HappyBus.ePHY_SndPAC= IOPs[brdNum].ePHY12_XMITp;
    
  /*  
    //fpga(s)..fpgaBase1-fpgaBase3
    if (brdNum<=8)                  
        {
        fbase= fpgaBase1;
        }
    else if (brdNum<=16) 
        {
        brdNum -= 8;
        fbase= fpgaBase2;
        }
    else if (brdNum<=24) 
        {
        brdNum -= 16;
        fbase= fpgaBase3;
        }                    
    HappyBus.FM_DAT = fbase+ (oFMData30[brdNum]*2);     //lvds data
    HappyBus.FM_STA = fbase+ (oFMStat40*2);             //lvds status
    HappyBus.FM_PAR = fbase+ (oFMPerr41*2);             //lvds parity error    
    HappyBus.ePHY_DATAVAIL_BIT= (1<<(brdNum-1));              //ePHY data avail status bit
    HappyBus.ePHY_RecFIFO= (uSHT*)fbase+(oPHY_RDFF20*brdNum); //ePHY rec data fifo    
    HappyBus.ePHY_CH_ENA= (uSHT*)fbase+(oPHY_XMT_ENA);      //Broadcast chans 0-7 enables
    HappyBus.ePHY_SndFIFO= (uSHT*)fbase+(oPHY_WRFF11);      //ePHY xmit data fifo
    HappyBus.ePHY_STAT= (uSHT*)fbase+(oPHY_rxSTA16);        //PHY 8-Port Rec data Status (bit7-0, 1=empty)
    HappyBus.ePHY_SndPAC= (uSHT*)fbase+(oPHY_XMIT12);       //ePHY XMIT R/W (RD 1=empty), (WR 1 to xmit)        
    */
    
    //clear rec data fifo (old data)    
    if(reset==1)
        REG16(HappyBus.FM_PAR)= FMRstBit8;                  //BIT8, clear buffer, fpga operation                   
    return 0;
}                



//Check link for active FEB boards
//FPGA register holds FEB board status via LVDS Rtn Signal(Clk/Data) being Active
//This function takes 2uS
int link_ID_Chk(int prt)        //old command sent "LCHK" to FEBs
{
    int pBits;

    pBits= ( (ACT_PORTS_HI<<16)+ ACT_PORTS_LO);
    POE_PORTS_ACTIVE[0]=0;
    for(int i=1,j=0; i<25; i++,j++)
        {
        if((pBits&(1<<j))==0)           //one fpga register holds status bits
            {
            POE_PORTS_ACTIVE[i]=0;      //bit set if fpga detects 'LVDS FM' clock
            continue;
            }
        POE_PORTS_ACTIVE[0]++;          //FEB attached total count
        POE_PORTS_ACTIVE[i]=1;          //store as active in array 1of24
        }    
    genFlag &= ~ID_ReqNow;  
    return 0;
}



//Send link data pool request via PHY, rec data on LVDS
//RequestMode High, Sends data pool req as '1 Broadcast per fpga'
//RequestMode Low,  Receives data
//Var lvLnk.PoolMode=1 enables timers to get here
int PoolDataReqGood(int Sock)  //old lc_LSTAB
{
    static int poePrt=1; // RequestMode=1;
    int d16;
    
    if(lvLnk.PoolReqType==0)
        {
        //wait for any ePHY Xmit to finish      //should happen but just in case
        //if (PhyXmitBsy(POE01))                  //just check 1of3 xmits for busy
        uDelay(20);           
        //genFlag |= POOL_Active;         
        
        *(uSHT*)IOPs[POE01].FM41_PARp= FMRstBit8; //FPGA2 lvds fifo buf and parErr clr              
        PHY_LOADER_POOL(POE01, 0);              //send command ePHY port
        mDelay(100);           
        
//todo Why, Requires long delay if UB3 DAQ Request are active
//todo Unk, Does ephy xmits clear lvds return fifo ???

        *(uSHT*)IOPs[POE09].FM41_PARp= FMRstBit8; //FPGA3 lvds fifo buf and parErr clr               
        PHY_LOADER_POOL(POE09, 0);              //send command ePHY port
        mDelay(100);           

        *(uSHT*)IOPs[POE17].FM41_PARp= FMRstBit8; //FPGA4 lvds fifo buf and parErr clr               
        PHY_LOADER_POOL(POE17, 0);              //send command ePHY port
        lvLnk.PoolReqType=1;                    //set mode to read returned data mode
        poePrt=6;
        }
    
    //tek 04-16-20, Data returns from FEB in blocks of 256 words,
    //variable delay ~256uS upto ~750uS between blocks depending on send count 
    //Pool Data Block from FEB (22words + (4*38) words), total=174 words
    //FEB LVDS FM REC/XMIT FIFO SIZE = 1024 WORDS 
    //
    if(genFlag & PoolReqGetData)                //time out yet, if so data should be available
        {
        if(poePrt==25)                          //port 1-24 were req send, now 25 is data reply check   
            {
            //flag as Done
            poePrt=1;
            lvLnk.PoolReqType=0;               //set mode back to request mode
            //these flags only cleard here after all '1of24' febs checked
            genFlag &= ~(PoolReqNow | PoolReqGetData); //clear flags
            lvLnk.PoolChkmSec=0;
            return 0;
            }
        //backgnd timer allow enough wait for data to show if FEB busy doing uBunch requests
        //ready for next 1of24 cycle, return later to read next poe port
        //check if data avail, leave old data
        if (POE_PORTS_ACTIVE[poePrt]==1)
            {
            d16= *(u_16Bit*)IOPs[poePrt].FM40_STAp; //check data avail status, 'assume full data blk received'   
            if ((d16& IOPs[poePrt].ePHY_BIT)==0)    //0= data available
                {
                if(POE_PORTS_ACTIVE[poePrt])        //get data for active ports only
                    {
                    //Move 16bits per count, Incr Dest Reg Only
                    movStr16_NOICSRC(IOPs[poePrt].FM30_DATp, stab_Pool[poePrt-1], eCMD72_MAX_WRDs174);
                    }
                }
            }
        poePrt++;
        }
    return 0;
}



//Send link data pool request via PHY, rec data on LVDS
//RequestMode High, Sends data pool req as '1 Broadcast per fpga'
//RequestMode Low,  Receives data
//Var lvLnk.PoolMode=1 enables timers to get here
int PoolDataReq(int Sock)  //old lc_LSTAB
{
  //THIS CODE WONT WORK IF DAQ ACTIVE, DAQ WILL GRAB THE RETURNED DATA UNLESS DAQs PAUSED
  //CANT USE, SEE NEW VERSION THAT WILL DO ONE PORT PORT 'REQUEST/READ' AT A TIME
    static int poePrt=1; // RequestMode=1;
    int d16;
    
    if(lvLnk.PoolReqType==0)
        {
        //wait for any ePHY Xmit to finish      //should happen but just in case
        PhyXmitBsy(POE01);                      //just check 1of3 xmits for busy
        PhyXmitBsy(POE09);                      //just check 2of3 xmits for busy
        PhyXmitBsy(POE17);                      //just check 3of3 xmits for busy
                
        //todo Why, Requires long delay if UB3 DAQ Request are active
        //todo Unk, Does ephy xmits clear lvds return fifo ???
          
        //this code mod now request data with 100mS between each req
        //*(uSHT*)IOPs[POE01].FM41_PARp= FMRstBit8; //FPGA2 lvds fifo buf and parErr clr              
        //*(uSHT*)IOPs[POE09].FM41_PARp= FMRstBit8; //FPGA2 lvds fifo buf and parErr clr              
        //*(uSHT*)IOPs[POE17].FM41_PARp= FMRstBit8; //FPGA2 lvds fifo buf and parErr clr 

        PHY_LOADER_POOL_BCAST(POE01, 0);        //send command to all ePHY ports
        lvLnk.PoolReqType=1;
        poePrt=1;
        }
    
    //tek 04-16-20, Data returns from FEB in blocks of 256 words,
    //variable delay ~256uS upto ~750uS between blocks depending on send count 
    //Pool Data Block from FEB (22words + (4*38) words), total=174 words
    //FEB LVDS FM REC/XMIT FIFO SIZE = 1024 WORDS 
    //
    if(genFlag & PoolReqGetData)                //time out yet, if so data should be available
        {
        if(poePrt==25)                          //port 1-24 were req send, now 25 is data reply check   
            {
            //flag as Done
            poePrt=1;
            lvLnk.PoolReqType=0;               //set mode back to request mode
            //these flags only cleard here after all '1of24' febs checked
            genFlag &= ~(PoolReqNow | PoolReqGetData); //clear flags
            lvLnk.PoolChkmSec=0;
            return 0;
            }
        //backgnd timer allow enough wait for data to show if FEB busy doing uBunch requests
        //ready for next 1of24 cycle, return later to read next poe port
        //check if data avail, leave old data
        if (POE_PORTS_ACTIVE[poePrt]==1)
            {
            d16= *(u_16Bit*)IOPs[poePrt].FM40_STAp; //check data avail status, 'assume full data blk received'   
            if ((d16& IOPs[poePrt].ePHY_BIT)==0)    //0= data available
                {
                if(POE_PORTS_ACTIVE[poePrt])        //get data for active ports only
                    {
                    //Move 16bits per count, Incr Dest Reg Only
                    movStr16_NOICSRC(IOPs[poePrt].FM30_DATp, stab_Pool[poePrt-1], eCMD72_MAX_WRDs174);
                    }
                }
            }
        poePrt++;
        }
    return 0;
}




//Send Packet on ePHY Link(s)
//If broadcast var set, send on all 8 ePHY links of one FPGA
int ePHY_SEND(int poePrt, int broadCast)
{
    if(broadCast==0)
        {
        int bit= HappyBus.ePHY_DATAVAIL_BIT;
        //enable single chan on XmitPort
        //*HappyBus.ePHY_CH_ENA= bit;     //takes 4 uC instructions
        *IOPs[poePrt].ePHY0E_XMSKp= IOPs[poePrt].ePHY_BIT;//takes 8 uC instructions
        }       
    else
        {
        //enable all 8 chans on XmitPort
        //*HappyBus.ePHY_CH_ENA= 0xFF;    
        *IOPs[poePrt].ePHY0E_XMSKp= 0xFF;    
        }
      
    //end of packet trig, send pack, add checksum                    
    //*HappyBus.ePHY_SndPAC= 1;         
    *IOPs[poePrt].ePHY12_XMITp= 1;
    return broadCast;
}




//Check ePHY Xmit Status (1of3 FPGAs)
//PrtPOE 1-8 uses fpga2, PrtPOE 9-16 uses fpga3, PrtPOE 17-24 uses fpga3 
//if busy return 1;
int PhyXmitBsy(int PrtPOE)
{
    u_32Bit wcnt=0, d16;
    do {
        d16= *IOPs[PrtPOE].ePHY12_XMITp & BIT1; //xmit status done?
        if(wcnt++>500000)                       //need break if fpga not active
           return 1;
	} while (d16!=BIT1);             
    return 0;
}


//Phy Xmit Buffer loading
//Expects 9 Words Requests
//Dump most, Keep some 
//
int uBunXmitBufLoad1(u_16Bit *uBDatPtr, int ldCnt)
{
    int dmp, i=5, uBunReq=0;
    
    while (ldCnt--)  //must have complete 9 words uBun Reqs
        {
     // dmp= GTP0_REQ_PAC;          //dump 1st k28.d2y word         
          
        dmp= GTP0_RQ_PAC0D;         //dump 2nd xFer byte cnt
        dmp= GTP0_RQ_PAC0D;         //dump 3rd PAC type word
        //2nd word should be 0x20 for testing
        if( (dmp&0xff)==0x20)
            dmp++;
        else 
            {
            uBReq.errRESYNC++;
            continue;
            }
        
        
        //keep time stamp low word, middle word
        dmp= GTP0_RQ_PAC0D;         //save uBun Number lower 16 
        *uBDatPtr++= dmp;       
        
        dmp= GTP0_RQ_PAC0D;         //save uBun Number middle 16 
        *uBDatPtr++= dmp;       

        //dump remaining 5 words of the 9 word uBun Req
        while(i--)
            dmp= GTP0_RQ_PAC0D;     //timestamp high
        i=5;
        uBunReq +=4;                //words cnt to send, 4of9 in each uB recd on GTP
        }
    return uBunReq;
}


//tek Apr30,2020 new code

u_16Bit Pac_9Words[10];

//Phy Xmit Buffer loading
//Expects 9 Words Requests
//Dump most, Keep some 
//
int uBunXmitBufLoad(u_16Bit *uBDatPtr, int ldCnt)
{
    int dmp, i=5, uBunReq=0, LpSyncErr=0, idx, badDat=0;
    
    while (ldCnt--)  //must have complete 9 words uBun Reqs
        {
        idx=1;
     // dmp= GTP0_REQ_PAC;          //dump 1st k28.d2y word         
          
        Pac_9Words[idx++]= GTP0_RQ_PAC0D;   //dump 2nd xFer byte cnt
        dmp= GTP0_RQ_PAC0D;   		//dump 3rd PAC type word
        //2nd word should be 0x20 for testing
        if( (dmp&0xff)!=0x20)
            {
            if(LpSyncErr==0)  
                {
                LpSyncErr++;
                sprintf(tBuf,"\r\n uBunch Sync Not xx20: %04X", dmp);                   
                putBuf(Sock0, tBuf,0);                   
                //todo: continue assums we have 9 more words 'or another packet in buf'
                badDat++;                
                continue;  
                }
            else
                {
                LpSyncErr++;
                sprintf(tBuf,"  %04X", dmp);
                putBuf(Sock0, tBuf,0);                   
                //todo: continue assums we have 9 more words 'or another packet in buf'
                badDat++;
                continue;
                }
            }
        else
            Pac_9Words[idx++]=dmp;       
        
        //keep time stamp low word, middle word
        dmp= GTP0_RQ_PAC0D;         //save uBun Number lower 16 
        *uBDatPtr++= dmp; 			//save uBun Number lower 16 	      
        Pac_9Words[idx++]=dmp;
        
        dmp= GTP0_RQ_PAC0D; 		//save uBun Number middle 16 
        *uBDatPtr++= dmp; 			//save uBun Number middle 16       
        Pac_9Words[idx++]=dmp;

        //dump remaining 5 words of the 9 word uBun Req
        while(i--)
            Pac_9Words[idx++]= GTP0_RQ_PAC0D; //timestamp high
        i=5;
        uBunReq +=4;                //words cnt to send, 4of9 in each uB recd on GTP
        }
    //return just one error per uBunch load
    if(LpSyncErr)
        {
        uBReq.errRESYNC++;
      //sprintf(Buf1500, "\r\n uB Request ReSync  : %-lu   (Skipped Above Words=%d)\r\n Fixed 9_Word_Req   : ", uBReq.errRESYNC,badDat);        
        sprintf(Buf1500, "\r\n Fixed 9 Word Req    : ");        
        for (i=1; i<10;i++)
            {
            sprintf(tBuf,"%04X  ", Pac_9Words[i]);
            strcat(Buf1500,tBuf);
            }
        strcat(Buf1500,"\r\n");
        putBuf(Sock0, Buf1500,0);        
        }
    return uBunReq;
}




#define uBuPacArMax  10             //max uB Requests
uSHT uBuPacArray10[uBuPacArMax*9]; //uBun Req Packet Storage, max room 10Reqs, each=9wrds

//FIBER LINK GTP Receive Packet Handler for testing (hardware loopback)
//GTP1 Rec FIFO holds nnn K28 (8Wrd+2Wrd) packet(s)
//Only save uBun lower 32Bits timestamp (will be 32BitAddrPtr for FEB)
//Buffer nn 32BitAddrPtr into larger packet before sending to FEB
//FEB will return data from that 32BitAddrPtr
//The testing function access is setup by command UB0-UB4
//
int GTP1_Rec_TEST()                 //trig requesting
{
    int k28_in, csr, k28_Reqs, cDat;
  //int uBunsLoaded;
    int rxStat4,rxStat8,rxStatC;
    static u_16Bit* uBDatPtr= uBuPacArray10;
    //static u_32Bit timeout=0;
    h_TP48_LO
            
    //wait for ePHY Xmit FIFO empty  
    while (PhyXmitBsy(HappyBus.PoeBrdCh))
        {
        LED_RED1
        LEDs_OFF        //QUICK LED PULSE
        h_TP48_HI
        h_TP48_LO
        };
    //assume busy done, could recheck
    //ePHY buffers must be empty before filling request
    rxStat4 = (ePHY_RX_STA_416&0xF);    //ePhy 8 lower bits rec fifo not empty
    rxStat8 = (ePHY_RX_STA_816&0xF);    //ePhy 8 lower bits rec fifo not empty
    rxStatC = (ePHY_RX_STA_C16&0xF);    //ePhy 8 lower bits rec fifo not empty
    if (rxStat4+ rxStat8+ rxStatC)      //any set
        {
        h_TP48_HI                       //quick toggle for scoping
        h_TP48_LO
        }
    
    csr= REG16(fpgaBase0+(0x400*2));
    if( (csr&0xFF)== 0xA8)              //If data keep in input fifo check for empty
        {
        if (rxStat4+ rxStat8+ rxStatC)  //look at input fifos
            {
            LED_RED1  
            for(int j=0; j<20; j++);
            LEDs_OFF                    //QUICK LED PULSE
            //if(timeout++> 6000)       //big time busy, break out of loop
            //    {
            //    sprintf(tBuf,"FPGAreg27 Not Empty =%X\r\n", fLNK_RECFIFO);
            //    putBuf(uBReq.Port, tBuf,0);
            //    timeout=0;
            //    uBReq.Flag &=~DAQREQ_2FEB;//end test
            //    }
            return 1;
            }
        }
    //check ubunch request fifo
    k28_in= GTP0_RQ_CNT0E;              //uBun buf wrd cnt 'RegE'   
    if(k28_in==0)
        {    
        if (uBReq.Mode==3)
            {
            //new stuff when empty reloading data request
            REG16(fpgaBase0+(0xf*2)) = 500;
            REG16(fpgaBase0+(0x32*2))= (1000>>16);    //bunch cnt high
            REG16(fpgaBase0+(0x33*2))= (1000&0xffff); //bunch cnt low
            REG16(fpgaBase0)= 0x101;            //one loop test mode
            uDelay(200);                        //allow time to ub reqs load
            }
        else
            {
            //no data end testing
            //timeout=0;
            uBReq.Flag &=~DAQREQ_2FEB;  //end test  
            }
        return 0;
        }
      
    //if k28_in buffer almost empty, if mode3 then retrigger a new request cycle
    if ((uBReq.Mode==3) && (k28_in<9))      //need 1 packets, 9 wrds ea
        {
        //RELOAD 
        //dump remaining words uBun Req, no cnt reg at this time
        //prevent optimizing out
        for(int empty, j=0; j<100; j++)
            empty=GTP0_RQ_PAC0D;                    //empty/dump fifo                    
        //reset daq fpga Interlink FIFOs
        //ePHY buffers must be empty before filling request
        rxStat4 = ePHY_RX_STA_416;
        rxStat8 = (ePHY_RX_STA_816<<8);
        rxStatC = (ePHY_RX_STA_C16<<8);
        //REG16(fpgaBase0+(0xf*2))= 10*10;
        REG16(fpgaBase0)= 0x101;                //continous run mode
      //REG16(fpgaBase0)= 0x301;                //one loop test, ubunch and heartbeat
        uDelay(200);
        }   

    //GTP RX FIFO CSR, buffer word count
    //must have 9 words available to decode uBun Req
    k28_in= GTP0_RQ_CNT0E;                  //uBun buf word count 'RegE'
  //if (k28_in < (9*uBReq.uBPerPacRq))      //need 1-8 Req per packet, (*9words each ubReq)
    if (k28_in < 9)                         //only need 1 packet now
        {
        LED_BLU1
        //for(int j=0; j<20; j++);
        //LEDs_OFF                                //QUICK LED PULSE
      //h_TP48_HI
        return 0;
        }
        
    //tek mod aug 2018, let daq packets control leds    
    LED_GRN1
      
    //fill packet with up to 2 uBunch Request if available
    k28_Reqs= k28_in/9;
    if(k28_Reqs>2)
        k28_Reqs=2;

    h_TP48_HI   
      
    //diag test here, addr 0, CSR  
    //Bit7 Reset packet former state machine 1=Reset
    cDat= REG16(fpgaBase0+(0x000));     
    REG16(fpgaBase0+(0x000))= cDat|0x80;
    
    
    //save 2 of 9 words req
    uBunXmitBufLoad(uBDatPtr, k28_Reqs);
     
    _disable_interrupt_();          //disable uC intr 
    
    //command UB0 set uBReq.SmPacMode=0 for standard min 64 byte packets
    //command UB4 set uBReq.SmPacMode=1 for non standard min 6 byte packets
    if(uBReq.SmPacMode==0)
        {
        //now sending from 3 FPGAs takes ~25uSec
        //standard min 64 byte packets
        PHY_LOAD_DAQ_K28SEND_BCAST(eCMD_DAQ_DY2, HappyBus.PoeBrdCh, uBuPacArray10, k28_Reqs);   //cmdBuf,Port,echoMode, plus full packet word cnt
        }
    else
        //standard min 6 byte packets
        PHY_LOAD_DAQ_K28SEND_BCAST_MINI(eCMD_DAQ_DY2, HappyBus.PoeBrdCh, uBuPacArray10, k28_Reqs);  // each ubCnt equals '2 16bit words'
    
    _enable_interrupt_();          //enable uC intr

    
    h_TP48_LO      
    //optional delay time
    uDelay(uBReq.uDly);   
    //reset buf ptr
    uBDatPtr= uBuPacArray10;
    //timeout=0;
    LEDs_OFF                //allow some light time, then QUICK LED PULSE
    return k28_in;
}



//test code
#define Wrd_SizPer_Rq       2       //size, 2 words used per uB Req
#define Req_Per_Packet      1
#define uBunMaxLWRDs2  (Wrd_SizPer_Rq * Req_Per_Packet)

//uB Req buffer 
uSHT uBunIDs[uBunMaxLWRDs2+2];      //store to compare to rtn data


//This function access is controller by cmd 'TRIG' and 'TRIG1'
//At some point this function will enable for normal DAQ data taking
//Getting here using 'TRIG1' should become the normal
//
int GTP1_Rec_Trigs()                //cmd 'TRIG' handler
{
    int k28_in, dat16;
    static int uBunWrd=0, idx=0, uBunReq=0;;
    
    //GTP RX FIFO CSR, buffer word count
    k28_in= GTP0_RQ_CNT0E;          //uBun data req packet buffer word count
    
    //wait for ePHY Xmit FIFO empty  
    if (PhyXmitBsy(HappyBus.PoeBrdCh))
        uDelay(300);    //should never happen, its fast
    
    while (k28_in)                  //data avail? , assum if any data then full 9 words in fifo
        {
        uBunReq++;
     // dat16= GTP0_REQ_PAC;        //dump 1st k28.d2y word         
        dat16= GTP0_RQ_PAC0D;       //dump 2nd xFer byte cnt
        dat16= GTP0_RQ_PAC0D;       //dump 3rd PAC type word

        //keep time stamp low word, middle word
        dat16= GTP0_RQ_PAC0D;       //save uBun Number lower 16 
        uBuPacArray10[uBunWrd++]= dat16;       
        uBunIDs[idx++]= dat16;
        
        dat16= GTP0_RQ_PAC0D;       //save uBun Number middle 16 
        uBuPacArray10[uBunWrd++]= dat16;       
        uBunIDs[idx++]= dat16;

        //dump rest of 8word xfer
        dat16= GTP0_RQ_PAC0D;       //timestamp high
        dat16= GTP0_RQ_PAC0D;       //resevered
        dat16= GTP0_RQ_PAC0D;       //resevered
        dat16= GTP0_RQ_PAC0D;       //resevered
        dat16= GTP0_RQ_PAC0D;       //crc
        
        //************************************************
        //******     fill minimun req size        ********
        //************************************************               
        if (uBunWrd < (uBunMaxLWRDs2))  //maybe concentrate uB Reqs before sending to FEB
             return k28_in;
        
        h_TP48_HI   //scope test point for testing         
        if(uBReq.Flag & DAQuB_Trig_OLD)  
            //TRIG command sets up standard packet mode with min packet size=64+ bytes (flag DAQuB_Trig)
            PHY_LOAD_DAQ_K28SEND_BCAST(eCMD_DAQ_DY2, HappyBus.PoeBrdCh, uBuPacArray10, uBunReq);     //cmdBuf,Port,echoMode
        else
            //TRIG1 command sets up non standard packet mode with min packet size= 6 bytes    (flag DAQuB_TrigNew)
            PHY_LOAD_DAQ_K28SEND_BCAST_MINI(eCMD_DAQ_DY2, HappyBus.PoeBrdCh, uBuPacArray10, uBunReq);//cmdBuf,Port,echoMode
        
        uBunWrd=0;
        uBunReq=0;
        idx=0;
        k28_in= GTP0_RQ_CNT0E;  
        h_TP48_LO   //scope test point for testing
        }  
    return k28_in;
}



#define PAYLDB48     48     //PAYLD24 as bytes
#define PAYLD24w     24     //with chksum

//load ePHY buffer and trigger xmit
//pass ascii paramter buffer, port, echo_prompt_on_off
//loads fpga ephy xmit fifo directly, then xmits packet
//Note *** PACKET LEN MINIMUM '64 bytes' else dest RM48 EMAC wont detect it ***
//64 bytes does not include preample so data must be 46 bytes, 64-46=18 (18=SRC(6),DST(6),TYPE(2), CHKSUM DONE_IN_FPGA(4)
//returns port number for echo purpose requirement

#pragma optimize=speed
int PHY_LOAD_DAQ_K28SEND_BCAST(int cmdType, int PrtPOE, sPTR xBuf, int wLen)
{   //SEND CMD STRING ON ePHY port to FEB
    //LVDS FM REC FIFO SIZE = 512 WORDS 
     
    //wait for ePHY Xmit FIFO empty  
    if (PhyXmitBsy(PrtPOE))
        printf("K28SEND Phy Busy timeout\r\n");
      //return 1;
    
    //DAQ command
    //** send 10 Word Header, src, dest, paclen, uC_Header[3] **//
    ePHY_HDR_DAQ.e_PAYLDLEN= SwapBytes(PAYLDB48);      //lengh as bytes count little endian
    ePHY_HDR_DAQ.u_CMDLEN= wLen;                       //repeat len for now
    ePHY_HDR_DAQ.u_BRDNUM= HappyBus.PoeBrdCh;          //cnt=16  hdr board number
    ePHY_HDR_DAQ.u_CMDTYP= cmdType;                    //cnt=18  hdr command type, must be DAQ or POOL command type

    //Now Load 10 words (20 bytes) DEST,SRC,LEN
    movStr16_NOICDEST((unsigned short*)&ePHY_HDR_DAQ, &ePHY301_BCAST_DATA, 10); //dest,src,len(1Wrd),uC_Hdr(3Wrds)

    //Now Load a minimum of 24 words (48 bytes) this has overhead for checkSum
    //todo fix cleanup and note wLen comes in as is ubreq req count, not as actual word count (needs to be wLen*2)
    if (wLen<PAYLD24w)
        wLen += PAYLD24w-wLen;
    movStr16_NOICDEST(xBuf, &ePHY301_BCAST_DATA, wLen); //no increment on dest 

    // Send Packet on all ports
    // *IOPs[PrtPOE].ePHYXMT_ENAp= IOPs[PrtPOE].ePHY_BIT;  //send one port, else all
    *IOPs[1].ePHY0E_XMSKp= 0xFF;                    //enable all port   
    *IOPs[9].ePHY0E_XMSKp= 0xFF;                    //enable all port   
    *IOPs[17].ePHY0E_XMSKp=0xFF;                    //enable all port   
    
    //send now using ePhy link global_24 data (broadcast) xmit    
    ePHY302_BCAST_XMIT= 1;
    return PrtPOE;
}



//load ePHY buffer and trigger xmit
//single port request pool data sends command 'LSTAB'      
int PHY_LOADER_POOL_SINGLE_PORT(int PrtPOE)         //NOT broadcast mode
    {                           
    //wait for ePHY Xmit FIFO empty  
    if (PhyXmitBsy(PrtPOE))
        uDelay(50);  
      
    ePHY_HDR_LSTAB.e_PAYLDLEN= PAC_CHLEN_MIN-ePHYCMDHDR_SZ_BYTS;
    ePHY_HDR_LSTAB.u_BRDNUM= PrtPOE;
    ePHY_HDR_LSTAB.u_CMDTYP= eCMD72_STAB;

    //command hdr data
    movStr16_NOICDEST((unsigned short*)&ePHY_HDR_LSTAB, IOPs[PrtPOE].ePHY11_BCAST_FILLFIFOp, PAC_CHLEN_MIN>>1); //min size to work 
    
    //ePHY FIFO has now loaded, send it
    *IOPs[PrtPOE].ePHY0E_XMSKp= IOPs[PrtPOE].ePHY_BIT; //enable single port
    
    //end of packet trig, send pack, add checksum                    
    *IOPs[PrtPOE].ePHY12_XMITp= 1;
    return 0;
}



//load ePHY buffer and trigger xmit
//pass port, mode (1Ch or 8Ch) //OLD PHY_LOADER_STAB
int PHY_LOADER_POOL_BCAST(int PrtPOE, int broadCast) //allways in broadcast mode
    {  
    int csr;
    //wait for ePHY Xmit FIFO empty  
    if (PhyXmitBsy(PrtPOE))
        uDelay(300);  
      //return 1;

    //tek 05-14-20 added fifo reset
    csr= REG16(fpgaBase0+(0x300));      //0x300: Write to the CSR registers (0x400,0x800,0xC00)
    REG16(fpgaBase0+(0x300))= csr|BIT0; //Rx FIFO Reset Bit0
      
    ePHY_HDR_LSTAB.e_PAYLDLEN= PAC_CHLEN_MIN-ePHYCMDHDR_SZ_BYTS;
    ePHY_HDR_LSTAB.u_BRDNUM= PrtPOE;        //board number not used at FEB
    ePHY_HDR_LSTAB.u_CMDTYP= eCMD72_STAB;   //for data pooling, feb decodes as valid bcast command

    //command hdr data
    movStr16_NOICDEST((unsigned short*)&ePHY_HDR_LSTAB, &ePHY301_BCAST_DATA, PAC_CHLEN_MIN>>1); //min size to work 
    
    // Send Packet on all ports
    *IOPs[1].ePHY0E_XMSKp= 0xFF;            //enable all port   
    *IOPs[9].ePHY0E_XMSKp= 0xFF;            //enable all port   
    *IOPs[17].ePHY0E_XMSKp=0xFF;            //enable all port  
    
    //send now using ePhy link global_24 data (broadcast) xmit    
    ePHY302_BCAST_XMIT= 1;                  //global xmit broadcast
    return 0;
}



//load ePHY buffer and trigger xmit
//pass ascii paramter buffer, port, echo_prompt_on_off
//uses eTuf200[] that limits commands to 200 bytes
int PHY_LOADER_CONSOLE(char* paramPtr, int Sock, int PrtPOE, int echo, int broadCast)
    {                           //SEND CMD STRING ON ePHY port to FEB
    //LVDS FM REC FIFO SIZE = 512 WORDS 
    int cnt=0,  errFLAG=0;                    
     
    //wait for ePHY Xmit FIFO empty  
    if (PhyXmitBsy(HappyBus.PoeBrdCh))  //this requires call to assignLinkPort() 1st
        {
        uDelay(300);
        errFLAG++;
        }
    
    //pacLen pre loaded, never changes as sending min packet size
    ePHY_HDR_CON.e_PAYLDLEN= PAC_CHLEN_MIN-ePHYCMDHDR_SZ_BYTS;
    ePHY_HDR_CON.u_BRDNUM= PrtPOE;
    ePHY_HDR_CON.u_CMDTYP= eCMD71_CONSOLE;
    
//tek check if the next line needed????  
    //tek Apr2020, comment next line, FEB ethernet doesn't check mac address now
    //movStr16((unsigned short*)&ePHYAdd,(sPTR)eTuf200,6); //cnt=0  phy addr dest, then src
    for(cnt=0; cnt<30; cnt++)      //append command from in buffer
        {
        ePHY_HDR_CON.u_CmdBuf[cnt]= *paramPtr++;    //append ASCII command from input buffer
        if (ePHY_HDR_CON.u_CmdBuf[cnt]==0)          //null marks end of string
            {   
            ePHY_HDR_CON.u_CmdBuf[cnt++]= '\r';     //add cmd term
            ePHY_HDR_CON.u_CmdBuf[cnt++]= 0;        //finish pack all nulls, only for debug remove later
            break;
            }
        }
    
    //command hdr data
    movStr16_NOICDEST((unsigned short*)&ePHY_HDR_CON, IOPs[PrtPOE].ePHY11_BCAST_FILLFIFOp, PAC_CHLEN_MIN>>1); //min size to work          
    ePHY_SEND(PrtPOE, broadCast);           //ePHY FIFO has now loaded, send it
    HappyBus.Socket= Sock;                  //HappyBus.ASCIIPrt ASCII xMIT I/O, ie SOCK,TTY,ePHY 
    HappyBus.CntRecd=0;
    if (echo==0)
        HappyBus.WaitCnt= 10;               //once active timeout(>100mS), 2uS per Null pass
    //Sock= NoPmt;                          //dont display prompt
    iFlag |= iNoPrompt;                     //flag as no prompt on terminal 
    return Sock;
}





//-----------------------------------------------------------------------------
//        tek aug2019 testing if small packets can be received at FEB
//-----------------------------------------------------------------------------

// as of Aug2019
//uBunch return data format
//1       WordCnt
//2       uB Req# High
//3       uB Req# Low
//4       uB Req# Status (see status format)
//5...    data up to uB_Sz512W max
//
//Returned Status Format
//BIT0    fpga0 uB# error
//BIT1    fpga1 uB# error
//BIT2    fpga2 uB# error
//BIT3    fpga3 uB# error
//BIT4    data overflow, uB_Sz512W max
//BIT5    fpga fifo not empty before new request


//tek aug2019 testing if small packets can be received at FEB
//see command 'PSEND1'

/*
#define ePHY_uBReqMax   64
struct ePHYubReq{
   unsigned char u_CMDTYP;                  //cmdType  1 bytes
   unsigned char u_uBcnt;                   //uB Words per packet (two words per uB request)
   unsigned short u_uBBuf[ePHY_uBReqMax];   //uB reqs
}; */



//tek 02/27/20
//this smaller non standard ethernet packet of ub Request work okay
//reduced non standard packet size (FEB board ethernet receiver setup for non standard packets)
//
#pragma optimize=speed
int PHY_LOAD_DAQ_K28SEND_BCAST_MINI(int cmdType, int PrtPOE, sPTR xBuf, int ubCnt)  // each ubCnt equals '2 16bit words'
{   
    //wait for ePHY Xmit FIFO empty 
    //this 'bsy' should never happen, break out in case fpga not initialized
    PhyXmitBsy(PrtPOE);
    
    ePHY_HDR_uBReq.u_CMDTYP= eCMD_DAQ_DY2;  //for DAQ, feb decodes as valid bcast command
    ePHY_HDR_uBReq.u_uBcnt= ubCnt;          //byte for number of MicroBunch Requests (ubReq)
    ubCnt <<= 1;                            //each ubCnt is equal to 2 16bit words in buffe
    //mov16... src, dest, cnt
    movStr16((snvPTR)xBuf, (snvPTR)&ePHY_HDR_uBReq.u_uB64wBuf, ubCnt);  //wordCnt to send plus 1 wrd hdr
    ubCnt +=1;                              //1 word header (2 bytes above)
    //mov16noIncr... src, dest, cnt
    movStr16_NOICDEST((snvPTR)&ePHY_HDR_uBReq, &ePHY301_BCAST_DATA, ubCnt); //wordCnt to send plus 1 wrd hdr
    
    
    //this code with 3 writes to 'ePHY_BCAST_DATA' does work, thinking all writes have to be in one group for fpga logic
    //DAQ ubReq header, direct load of xmit fifo
    //ePHY_BCAST_DATA= eCMD_DAQ_DY2;          //byte 'daq req command type'
    //ePHY_BCAST_DATA= ubCnt;                 //byte 'ub Req count
    //ubCnt <<= 1;                            //each ubCnt is equal to 2 16bit words in buffe
    //DAQ ubReq data, fill buffer using ePhy link global_24 data (fifo) loader
    //movStr16_NOICDEST((unsigned short*)&xBuf, &ePHY_BCAST_DATA, ubCnt); //wordCnt to send plus 1 wrd hdr
    
    // Send Packet on all ports
    *IOPs[1].ePHY0E_XMSKp= 0xFF;            //enable all port   
    *IOPs[9].ePHY0E_XMSKp= 0xFF;            //enable all port   
    *IOPs[17].ePHY0E_XMSKp=0xFF;            //enable all port  
    
    //send now using ePhy link global_24 data (broadcast) xmit    
    ePHY302_BCAST_XMIT= 1;                  //global xmit broadcast
    return PrtPOE;
}












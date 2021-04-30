//******************************************************************
// @file sys_mu2e_daq.c 
//  Fermilab Terry Kiper 2016-2020
//  mu2e controller board
//  RM48 Micro Controller
//  RM48 Hercules device supports little-endian [LE] format
//  Read/Write EMAC block for i/o on uC EPHY Port
//******************************************************************


/* Include Files */
#include "sys_common.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sys_dma.h"
#include "emac.h"
//sys_mu2e proto-types
#include "ver_io.h"
#include "ver_ePHY.h"
#include "w5300.h"

#include "sys_mu2e.h"
#include "sys_mu2e_functions.h"


extern  char* eRecDatPtr[MaxSock];          //socket(s) data prts
extern  unsigned int sLen[MaxSock];
extern  struct HappyBusReg HappyBus;        //Init as board, wcnt, stat;


//EMAC structure
extern hdkif_t hdkif_data[MAX_EMAC_INSTANCE];   //defined in 'emac.c'
extern struct phyHeader phyHdr;
extern struct phyPACSTAT paKet;             //all daq packets, status/counter

struct phyPAC_CNTRL pCtrl;                  //all daq packets i/o control
struct phyPAC_ERRORS PaketStats;            //storage for packet errors, counters


//RDX control structure
extern struct blocksndWrd BinStRDX;
extern struct structFPGA_Trig_Cnts FPGA_Trig_Stat;
extern  uint32  volatile iFlag;


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


/*Ethernet Frame Structure 'emac.h' for reference
typedef struct ethernet_frame
  {
	uint8 dest_addr[6];     //Destination MAC Address
	uint8 src_addr[6];      //Source MAC Address
	uint16 frame_length;    //Data Frame Length
	uint8 data[1500];       //Data
  }ethernet_frame_t;

  'emac.h'
  * Pointer to the actual ethernet packet/packet fragment to be transmitted.
  * The packet needs to be in the following format:
  * Destination MAC Address (6 bytes)| Source MAC Address (6 bytes)| Length/Type (2 bytes)| Data (46- 1500 bytes)
  * The data can be split up over multiple pbufs which are linked as a linked list.
*/  



//uB Link list and buffers to store DAQ FIFOs data
#define uB_Sz512W         512           //Limit amount of data returned per uB or (1024 Bytes)
#define uB_Sz1024B       (uB_Sz512W*2)
#define uB_Reqs8            8           //Max number of Controller Buffered uB Req (must have ret data buffer space)

//define link list size, must handle (MaxBytes_to_Send (32767)/ (1500)BytesPerPack )
#define PACKs21             21          //init nn packet list of (1500 Bytes or 750words) each

#pragma pack(4)                         //force 16bit boundary on all types
pbuf_t  PacLnkLst[PACKs21];             //Link List of 21 Packets, holds max (21*1500= 31500 Bytes 7B0CH)

#define PacBytes1500        1500        //use 1500, normally 1530 is absolute Max Bytes


//FEB EMACxmit has PACKs(8) * MAX_TRANSFER_UNIT(1514 Bytes)
//Change words to byte count, 1024Bytes*(4)FPGABuffers*(8)MaxUbRequest
uint16_t      xWrds32767[(uB_Sz1024B*4*uB_Reqs8)+128]; //room for '8 full ubun req' plus headers

// Init EMAC Packet hdr bufs and XmitData Ptrs, called by cmd 'RDX' and 'eCMD_DAQ_DY2_Handler()'
// The standard packets needs to be in the following format:
//   | Destination MAC Address (6 bytes)
//   | Source MAC Address (6 bytes)
//   | Length/Type (2 bytes)
//   | Data (46- 1500 bytes)
// The data can be split up over multiple pBufs as a linked list.
// function time > 2uS
//
//  FEB non standard packets, its all data, must send at least 46 bytes for uC EMAC sequencers


//Link List Builder, uC EMAC uses this list to DMA data out EMAC Phy Port
//
void fill_daq_packetOLD(int szBytCnt, u_8Bit *dataPtr)  //byte count even and greater than 2
{     
    int i,ovrflow=0, pacCnt=PACKs21;     
    //error check
    if (szBytCnt < MAX_TRANSFER_UNIT)
      pacCnt=1;
    else
        {
        ovrflow= szBytCnt/ pacCnt;              //divide then mult and see if cnt match
        ovrflow= szBytCnt- ovrflow* pacCnt;     //if counts dont match add extra to packs[1].len
        }
    //fill in packet 'pbuf'...packs[n]
    for(i=0; i<pacCnt; i++)     
        {     
        PacLnkLst[i].tot_len= szBytCnt;         //len to of all buffers (bytes)
        PacLnkLst[i].len= (szBytCnt/pacCnt);    //len to this buffer
        if(i==0) 
            PacLnkLst[i].len +=ovrflow;         //add overflow to 1st 'pbuf'
        PacLnkLst[i].payload = (u_8Bit*)dataPtr;
       
        //ready data ptr for next pass
        dataPtr+= PacLnkLst[i].len;             //incr byte ptr by byte count        
        
        //next emac header
        if(i!= (pacCnt-1))
            PacLnkLst[i].next = &PacLnkLst[i+1]; //ref for next packet, repeat loop
        }
    PacLnkLst[pacCnt-1].next= NULL;         //final emac packet marked as end
}



//Link List Builder, uC EMAC uses this list to DMA data out EMAC Phy Port
//Build list by dividing ByteCntIn by max BytesPerPack, NewJul2019
void fill_daq_packet(int szBytCnt, u_8Bit *dataPtr)  //byte count even and greater than 2
{     
    int i=0,ovrflow=0, pacCnt, pacSiz=szBytCnt; //PACKs14 max link list packets   
    //error check
    if (szBytCnt < PacBytes1500)
        pacCnt=1;
    else
        {
        pacCnt= (szBytCnt/PacBytes1500)+1;
        pacSiz= szBytCnt/ pacCnt;               //divide then mult and see if cnt match
        ovrflow= szBytCnt- (pacSiz*pacCnt);     //if counts dont match add extra to packs[1].len
        }
    //fill in packet 'pbuf'...packs[n]
    for(; i<pacCnt; i++)     
        {     
        PacLnkLst[i].tot_len= szBytCnt;         //len to of all buffers (bytes)
        PacLnkLst[i].len= pacSiz;               //len to this buffer
        if(i==0) 
            PacLnkLst[i].len +=ovrflow;         //add overflow to 1st 'pbuf'
        PacLnkLst[i].payload= (u_8Bit*)dataPtr; //store data ptr
       
        //ready data ptr for next pass
        dataPtr+= PacLnkLst[i].len;             //incr byte ptr by byte count        
        
        //next emac header
        if(i!= (pacCnt-1))
            PacLnkLst[i].next= &PacLnkLst[i+1]; //ref for next packet, repeat loop
        }
    PacLnkLst[pacCnt-1].next= NULL;             //final emac packet marked as end
}




//#pragma no_optimization //optimize=speed  'for reference'
//**********************************************************************
//*****    check for rec packets on rj45 ethernet  (ePHY)       ********
//*****                                                         ********
//*****    RM48 uC Internal EMAC hardware,                      ********
//*****    Beta... file 'emac.c' function EMACTxIntISR()        ********
//*****    Mar2018 file 'notifications.c'  emacRxNotification() ********
//*****                                                         ********
//*****    EMAC RX Buffer descriptor data structure             ********
//*****    Refer TRM for details buf descriptor structure.      ********
//**********************************************************************
int phy_Pac_check()
{
    int cmdByteSz=0;
    if (phyHdr.PacActive == phyHdr.PacNext)
        {
        if (paKet.EMACRxRdy==0)         //Rec'd packet 'g_EMACRxRdy==1'
            return 0;  
        }
    
    //******************************************************************           
    //*********   DAQ uBunch Packet Process Handler Check    ***********
    //******************************************************************           

    //tek aug2019 mod, daq short packet request if 1st char matches process short packet here
    //tek aug2019 mod, daq short packet request if 1st char matches process short packet here
    cPTR chPtr= (char *) phyHdr.PacPayLd1024w[phyHdr.PacActive]; 
    if (*chPtr++== eCMD_DAQ_DY2)
        {
        //*** 4uS overhead to get to 'eCMD_DAQ_DY2_Handler()' in intr routine ***
        paKet.RecPackCnts++;                //sysTick timer clears this every 1000mS
        paKet.EMACRxRdy=0;                  //reset flag set in emac rec handler
        phyHdr.PacCmdLen= *chPtr++;  
        eCMD_DAQ_DY2_Handler((uSHT*)(chPtr),eCMD_DAQ_DY2, phyHdr.PacCmdLen );
        //packet processed, point to next packet in buffer
        phyHdr.PacActive++;
        if (phyHdr.PacActive>= PacBufs)     //'ver_io.h'
            phyHdr.PacActive=0; 
        return 0;                           //done here
        }
    
    
    //tek aug2019 code below is unchanged and will still work
    //pointer to received packet, skip 7 words, look at Cmd_Len
    sPTR PTR16= (uSHT*) phyHdr.PacPayLd1024w[phyHdr.PacActive]+7; 
    paKet.RecPackCnts++;                //sysTick timer clears this every 1000mS
    paKet.EMACRxRdy=0;                  //reset flag set in emac rec handler
    //save phyHdr
    phyHdr.PacCmdLen= *PTR16++;         //cmd length
    PTR16++;                            //brd number
    phyHdr.PacType= *PTR16++;           //pac type
    
    if (phyHdr.PacType== eCMD_DAQ_DY2)
        {
        //*** 4uS overhead to get to 'eCMD_DAQ_DY2_Handler()' in intr routine ***
        eCMD_DAQ_DY2_Handler(PTR16, phyHdr.PacType, phyHdr.PacCmdLen );
        //packet processed, point to next packet in buffer
        phyHdr.PacActive++;
        if (phyHdr.PacActive>= PacBufs) //'ver_io.h'
            phyHdr.PacActive=0; 
        return 0;                       //done here
        }

    //******************************************************************           
    //*********      ALL Other Packet Processed Here         ***********
    //*********      Data should contain valid FEB command   ***********
    //******************************************************************           
    cmdByteSz= phyHdr.PacBytCnt[phyHdr.PacActive];
    if(cmdByteSz)
        {
        if(cmdByteSz> PayLdMaxBytes )
            sLen[ePhyIO]= PayLdMaxBytes;
        //activePac ready for next intr recd paket
        eRecDatPtr[ePhyIO]= (char*) phyHdr.PacPayLd1024w[phyHdr.PacActive]; 
        sPTR PTR16= (unsigned short*)eRecDatPtr[ePhyIO]+7; //skip 7 words (dst,src,len)
        
        //1st 6 chars are saved for happy bus header
        HappyBus.CmdSizW = *PTR16++;        //word count hex
        HappyBus.BrdNumb = *PTR16++;        //brd numb hex
        HappyBus.CmdType = *PTR16++;        //command type hex
        
        //on entry 'HappyBus.BrdNumb' now holds 'cntrl' and 'board number'
        //store as temp 'cntrl' number, must be verified by "LCHK" command
        HappyBus.CntrlNumb= (HappyBus.BrdNumb>>8); //upper controller numb
        HappyBus.BrdNumb &= 0xff;           //low byte board numb

        //tek jul2017 may need to adjust bytesize since ptr is adjusted by 14        
        if (cmdByteSz> ePHYCMDHDR_SZ)
            {
            cmdByteSz -= ePHYCMDHDR_SZ;
            eRecDatPtr[ePhyIO]+=ePHYCMDHDR_SZ;  //skip mac numbers and pacLength
            }
        }
    
    //data moved from emac local packet buffer 1ofnn see 'ver_ePHY.h'
    //packet moved to local command line buffer, point to next packet in buffer
    phyHdr.PacActive++;
    if (phyHdr.PacActive>= PacBufs)             //ver_io.h
        phyHdr.PacActive=0; 
    
    return cmdByteSz;                           //done

    
//#ifdef USE_INTRDAQ     //'ver_ePHY.h'
  //  if (HappyBus.CmdType== eCMD_DAQ_DY2)
  //      return 0;    
//#endif    
  //  if (HappyBus.CmdType== eCMD72_STAB)    //trap because BCASTs brdNumb may not match
  //      return cmdByteSz;
  //else if (HappyBus.CmdType== eCMD79_LINK_INIT)
  //    return cmdByteSz;
  //else if (HappyBus.BrdNumb== HappyBus.myLinkNumber)
  //allow all commands, u2eController has individual ports 1-24
  //else if ((HappyBus.BrdNumb>0) && (HappyBus.BrdNumb<25))
  //    return cmdByteSz;
  //  else
  //  return 0;
}
    


extern struct  msTimers mStime; 

//read packet length, needed by 'LDF' function
//
int PHY_LEN(int wait)
{
  mStime.g_wTicks=0;
  int len;
  while(1)
      {
      if(mStime.g_wTicks > wait)
        return 0;                           //timeout, return 'FALSE'
      else 
        {
        //socket, port, buffer, mode(Sn_MR for no delay)
        len= phy_Pac_check();  
        if (len)                            //rec char count
            return len;                     //data avail, return 'TRUE'
        }
      }    
}



//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
//-------------     New Version DAQ Handler Oct 2018 using FIFOs   ---------------------
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------


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
//BIT4    fpga0 xmit fifo no empty after previous readout cycle
//BIT5    fpga1 xmit fifo no empty after previous readout cycle
//BIT6    fpga2 xmit fifo no empty after previous readout cycle
//BIT7    fpga3 xmit fifo no empty after previous readout cycle
//BIT8    fpga fifo not empty before new request


//#pragma optimize=speed
#define gFPGA1      0x000           //FPGA 1of4, may be reference as fpga 0-3 elsewhere       
#define gFPGA2      0x400           //FPGA 2of4
#define gFPGA3      0x800           //FPGA 3of4
#define gFPGA4      0xC00           //FPGA 4of4

//READs micro bunch address and returns data (uc now gets data from fpga FIFO) 
//buffers for max requests of uB_Reqs8, each with word count of uB_Sz512W
//set flag reg to prevent 'Telnet Process' checking when in IRQ_MODE
//
int eCMD_DAQ_DY2_Handler(uSHT* PTR16, int cmdTyp, int ubCnt )
{
    int i=0,w,cmds=0, stat;
    u_16Bit ubShiftL, ubShiftH;
    u_32Bit ubShift;
    //unsigned short DatDump;
    sPTR ucSDramLOx; 
    
    if(ubCnt==0)        //error
        {
        hHI_TP45;          
        hLO_TP45;          
        return 0;
        }
    
    //wait, slight chance that 'xWrds32767' buffers may be overwritten before DMA Xmit
    //set flag that send is pending/active, will be clear in 'EMACTxIntISR()'
    //g_EMACTxBsy cleared in emacTxNotification()... may get missed
    //
    //todo: this should not happen, check counter to verify, see uC CMD 'UB'
    //
    for(w=0; w<200000; w++) 
        {
        if(paKet.EMACTxBsy==0)      //break out when xmit done
            break;
        }
    if(w>=200000)                 
        PaketStats.TOflag++;        //for testing diagnostics, see uC CMD 'UB'

    //paKet.uBunPerPac= ubCnt;
    
    //init counters
    pCtrl.g_idx= 0;
    pCtrl.RetSzBytes=0;
    
    //todo tek aug2019
    //this code does not account for "fpgas/AFE chips" power down mode
        
    //create packet for PHY DMA xmit       
    //cmdLen is number of uBunch Req in packet from controller   
    //memory buffer avail for max of uB_Reqs8 uB request of uB_Sz512W (words each)
    for(w=0; w<ubCnt; w++)  
        {
        //init error flags on each pass
        pCtrl.gBunErr=0;
          
        hLO_TP45;         
        cmds++;
        pCtrl.uBunLO= *PTR16++;
        pCtrl.uBunHI= *PTR16++; 
        pCtrl.pacSiz=0;                
        
        paKet.uBunReqTot++;
        //check if any data still in fifo   (none expected unless overrun,pwrUp)
        //must do or data will never be unpacked at mu2e_Controller correctly
        //BCST_GEO_317 bits 0-3 Status; bits 4-7 Empty flags (for all 4 fpgas)
        //tek aug2019 if fifo have clr bit, then clr fifo on exit
        //depending on the time it takes from clr to ready
        //clear event FIFOs for all 4 FPGAs
      
        BCST_GEO_317= 0x100;    //clear event fifo
        stat= BCST_GEO_317;     //added this for testing only
        
        //wCnt= eFIFO_wCNT1;      //for testing
        //wCnt= eFIFO_wCNT2;
        //wCnt= eFIFO_wCNT3;
        //wCnt= eFIFO_wCNT4;
        
        //FIFOs now empty
        //Global broadcast, all fpgas fill fifos with new data
        //After global address set 0x312,0x313 wait for data fifo auto fill go non empty
        //REG16(fpgaBase+0x312*2)= pCtrl.uBunHI;  //ub High                    
        //REG16(fpgaBase+0x313*2)= pCtrl.uBunLO;  //ub Low   
        
        BCST_GEO_312= pCtrl.uBunHI;     //ub High                    
        BCST_GEO_313= pCtrl.uBunLO;     //ub Low    

        /*
        ubShift= pCtrl.uBunHI<<16;
        ubShift |= pCtrl.uBunLO;
        ubShift = ubShift<<12;
        ubShiftL= ubShift>>16;      //ub Low shift 
        ubShiftH= ubShift;          //ub Low shift 
        
        //Trigger FIFO loading of new data, for testing
        SDPTR_BCASTH0= pCtrl.uBunHI;     //ub High
        SDPTR_BCASTL0= pCtrl.uBunLO;     //ub Low           
        SDPTR_BCASTH1= pCtrl.uBunHI;     //ub High
        SDPTR_BCASTL1= pCtrl.uBunLO;     //ub Low
        SDPTR_BCASTH2= pCtrl.uBunHI;     //ub High
        SDPTR_BCASTL2= pCtrl.uBunLO;     //ub Low
        SDPTR_BCASTH3= pCtrl.uBunHI;     //ub High
        SDPTR_BCASTL3= pCtrl.uBunLO;     //ub Low
        */
        
        //Global broadcast addr BCST_GEO_317, read Event FIFO Status
        //Wait for fpgas data to fill fifo (non empty)
        while (1)
            { 
            stat= ((BCST_GEO_317) &0xF);    //FPGA fifo event data rdy flags=1)
            if(stat==0xf) break;
            if(++i>10000)                   //Diagnostics, short wait, fpga should already be done
                {
                PaketStats.TOflag++;
                pCtrl.gBunErr |= BIT8;      //error flag if not empty 
                break;
                }
            }
    //tek 05-13-20 seems to need extra delay
    //BCST_GEO_317 may not be working
        uDelay(5);
        
        //read all fpgas
        if (ubGetData(gFPGA1, &DAQ_FIFO1))  //get uB data from fifo
            pCtrl.gBunErr |= BIT0;          //set overflow flag?        
        if (ubGetData(gFPGA2,&DAQ_FIFO2))   //get uB data from fifo
            pCtrl.gBunErr |= BIT1;          //set overflow flag?
        if (ubGetData(gFPGA3,&DAQ_FIFO3))   //get uB data from fifo
            pCtrl.gBunErr |= BIT2;          //set overflow flag?
        if (ubGetData(gFPGA4,&DAQ_FIFO4))   //get uB data from fifo
            pCtrl.gBunErr |= BIT3;          //set overflow flag?
            
        //Event Done, Update Total Word Count in Header
        xWrds32767[pCtrl.uHdrIdx]  = pCtrl.pacSiz;  //4word Header Ubun wrdCnt
        xWrds32767[pCtrl.uHdrIdx+3]= pCtrl.gBunErr; //4word Header Ubun errors
        pCtrl.RetSzBytes += pCtrl.pacSiz;           //set total rtn size
        hHI_TP45;    
        }
    
    hLO_TP45;   
    //Packets normally must meet a minimum size 
    //Not a problem, Min packet always has 4*4 word header (good)
    //Check data size, Minimum ePHY packet size is 8 words, pad out if needed    
    pCtrl.RetSzBytes<<=1;               //convert word to byte count for 'fill_daq_packet()'    
    fill_daq_packet(pCtrl.RetSzBytes, (u_8Bit*)&xWrds32767); //ByteCount, data buffer ptr           
    paKet.EMACTxBsy=1;                  //flag tx as busy                      
    //send data to MAC , uses chained pack list of last uB Req, possilly upto 8 Request
    EMACTransmit(&hdkif_data[0], &PacLnkLst[0]); 
    hHI_TP45;          
    return 0;
}         



//single define from 'w5300.h'
#define SOCK_ESTABLISHED   0x17                 /**< TCP connection is established. */

//external
char    msgBuf[200];
char    msgBuf1[100];

//Get micro-bunch (uB) data
//Verify uBunch# in memory(now FIFO) matches requested uBunch# 
//Read 4 FPGAs
//  If 1st FPGA add 4 word header before data 
//  If uB# error, only 4 word header returned
//  If no error, fills uC buffer from FPGA(1-4) from FIFOs
//  Returns error codes for uB mismatch and data over-run
//
//New version Feb2020 uses BIT15 of word count set as overflow flag (done in FPGA)
//  1st Check BIT15 of word count, set overflow var if needed
//  2nd Clear BIT15 of to get normal word count.
//
//Max buffer size per FPGA set by 'uB_Sz512W'

#pragma optimize=speed
int ubGetData(int fpga1of4, sPTR rAddr)
{
    u_16Bit UbunHI_Ram, UbunLO_Ram, err=0, ovrflow=0; 
    u_32Bit uB32REQ, uB32MEM;
    
    static u_16Bit fpgaAct=0, oneErr=0;
    pCtrl.uBDatSz= *rAddr;                  //get data word cnt, Note must be unsigned integer for math

    if(pCtrl.uBDatSz & BIT15)               //fpga sets BIT15 if overflow
        { 
        ovrflow=1;                          //initial default value=zero
        pCtrl.uBDatSz &= ~BIT15;            //remove overflag bit15 from data size
        }
    pCtrl.uBDatSz -=3;                      //skip 3WrdMemHdrSiz(wCnt,uBhi,uBlo)
    UbunHI_Ram= *rAddr;                     //get data bun# High16 from fifo
    UbunLO_Ram= *rAddr;                     //get data bun# LOW16 from fofo   
    
    //Check uBunRequest# to Data in sdRam for match
    if((pCtrl.uBunHI==UbunHI_Ram) && (pCtrl.uBunLO==UbunLO_Ram))
        {
        //-----------------------------------------------------------------------
        //------------- GOOD uBunch Request -------------------------------------
        //------------- Request Matches ram data stored in buffer----------------
        //-----------------------------------------------------------------------
        if(fpga1of4==0)                   
            {
            //-----------------------------------------------------------------------
            //----- On 1st FPGA processing as in 1of4 fpgas create header -----------
            //----- Header Stored in 1st 4 wrds of uB return buffer   ---------------
            //-----------------------------------------------------------------------
            //1st fpga check, add in a global header for this group of 4 fpga readout
            fpgaAct=0;
            oneErr=0;                               //limit err msg to one on each event 
            pCtrl.uHdrIdx= pCtrl.g_idx;
            pCtrl.g_idx++;                          //4word Header, skip wrd cnt here, filled in after 4 fpga processed
            xWrds32767[pCtrl.g_idx++]= UbunHI_Ram;  //4word Header, Ubun High 'from sdRam'
            xWrds32767[pCtrl.g_idx++]= UbunLO_Ram;  //4word Header, Ubun Low  'from sdRam'
            pCtrl.g_idx++;                          //4word Header, skip Status until end
            pCtrl.pacSiz +=4;
            paKet.SndTotWrds += 4;                  //add 4_word_header size            
            }
        //Get uB 'microbunch' Data
        if(ovrflow==0) //okay here, no overflow
            {
            //-----------------------------------------------------------------------
            //---------- GOOD uBunch Req, No-Overflow, Okay to process --------------
            //-----------------------------------------------------------------------          
            //
            if(pCtrl.uBDatSz>0)                     //Empty event, no data to move
                {
                //block move data, ...(SRC(FIFO), DEST(ucMemory), WordCount))
                movStr16_NOICSRC((snvPTR)rAddr, (snvPTR)&xWrds32767[pCtrl.g_idx], pCtrl.uBDatSz); 
                pCtrl.g_idx  += pCtrl.uBDatSz;
                pCtrl.pacSiz += pCtrl.uBDatSz;
                paKet.SndTotWrds += pCtrl.uBDatSz;  //add data words read
                }
            } //done with good uB Request Process 'No Overflow'
        else
            { 
            //---------------------------------------------------------------------------------
            //--- GOOD uBunch Req Match, BUT DATA OVERFLOW FLAG SET   -------------------------
            //--- Set flags but Sends Data pre-overflow using count controlled in FPGA logic --
            //--- Change readout size and set overflow software check error flag    -----------
            //---------------------------------------------------------------------------------
            
            //block move data, ...(SRC(FIFO), DEST(ucMemory), WordCount))
            movStr16_NOICSRC((snvPTR)rAddr, (snvPTR)&xWrds32767[pCtrl.g_idx], pCtrl.uBDatSz); 
            pCtrl.g_idx  += pCtrl.uBDatSz;
            pCtrl.pacSiz += pCtrl.uBDatSz;
            //uBunSz may be truncated if to large
            paKet.SndTotWrds += pCtrl.uBDatSz;      //add data words read                                          
            PaketStats.uBunOvrErrs++;               //BIT15 overflag here set by FPGA 
            pCtrl.gBunErr |=(BIT4<<fpgaAct);        //to much data flag per fpga
            }            
        }   //done with good uB Request Process 'Overflow Flag Set' 
    else
        //-----------------------------------------------------------------------
        //----------------------- BAD uBunch Request ----------------------------  
        //---- Request Number Does Not Match Ram Data Moved to FIFO -------------
        //---- Update error counters, clear out FIFO data with global reset -----
        //-----------------------------------------------------------------------      
        {
        if(fpga1of4==0)                             //1st fpga check, include the global header
            {
            fpgaAct=0;                              //done 4 fpgas, reset counter
            oneErr=0;                               //limit err msg to one on each event 
            //error, no match on uBunch Req IDs, return 4_word_header
            paKet.SndTotWrds += 4;   
            pCtrl.uHdrIdx= pCtrl.g_idx;
            pCtrl.g_idx++;                          //4word Header skip wrd cnt until end
            xWrds32767[pCtrl.g_idx++]= pCtrl.uBunHI;//4word Header Ubun High 'requested'
            xWrds32767[pCtrl.g_idx++]= pCtrl.uBunLO;//4word Header Ubun Low  'requested'
            pCtrl.g_idx++;                          //4word Header skip Status until end
            pCtrl.pacSiz +=4;
            PaketStats.uBunReqErrs++;               //on going error total for display cmd'UB'
            }
        //-----------------------------------------------------------------------
        //tek Apr16,2020
        //clear old data since uBunch numbers did not match
        BCST_GEO_317= 0x100;    //clear event fifo now           
        //-----------------------------------------------------------------------
            
            //-----------------------------------------------------------------------
            //------------------ BAD uBunch Request, no match  -----------------------  
            //--- tek March2020, added Diag Msg to send out SOCK or USB   -----------
            //--- see command 'DEBUG' to enable this mode and select port  ----------
            //-----------------------------------------------------------------------
            if ((pCtrl.g_DEBUG_MODE) && (oneErr==0))     
                {
                if(PaketStats.uBunReqErrs<10)
                    {
                    oneErr++;  //limit err msg to one on each event (thats 1 msg per four fpgas) 
                    uB32MEM= (( UbunHI_Ram<<16)+( UbunLO_Ram));
                    uB32REQ= ((pCtrl.uBunHI<<16)+(pCtrl.uBunLO)); 
                    uint64 ubnum;
                    int rdLow, rdHigh;
                    rdHigh= REG16(fpgaBase+0x004*2);//upper sdRam rd ptr 
                    rdLow = REG16(fpgaBase+0x005*2);//lower sdRam rd ptr 
                    rdHigh= ((rdHigh<<16) + rdLow);
                                        
                    sprintf(msgBuf, " uBun# Rec'd    : %-8X\r\n", uB32REQ);
                    ubnum= uB32REQ;
                    ubnum= ubnum<<10;
                    sprintf(msgBuf1," uBun# in SDRam : %-08X  SdRdPtr= %-08X  ReqRecdShift10= %llX\r\n", uB32MEM, rdHigh, ubnum);
                    strcat (msgBuf,msgBuf1);
                    
                    if ((pCtrl.g_DEBUG_MODE==1) && (getSn_SSR(pCtrl.g_DEBUG_MODE)==SOCK_ESTABLISHED))  
                        putBuf(sock1, msgBuf,0);            //use sock1 link
                    else if (pCtrl.g_DEBUG_MODE==2)         //send on lvds fm, this is the cmd 'LC' link
                        putBuf(ePhyIO, msgBuf,0);           //use ePhyIO link
                    else if (pCtrl.g_DEBUG_MODE==3)         //out to usb port, user should set baud high to prevent slowdowns
                        putBuf(tty, msgBuf,0);              //use USB output
                    else if(PaketStats.uBunReqErrs==10)
                        {
                        sprintf(msgBuf,"uB SDRam Msg  : First 10 errors listed, cmd 'UB' to reset count\r\n");                    
                        putBuf(sock2, msgBuf,0);                            
                        }                  
                    }
                }
        err++;                                              //error counter
        }
    
    fpgaAct++;
    return(err);
}




  

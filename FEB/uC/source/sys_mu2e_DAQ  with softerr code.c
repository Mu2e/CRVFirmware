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

//dma structure
g_dmaCTRL g_dmaCtrl_RDX;                    //dma control packet config stack

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




//DMA Wiznet for uC cmd 'RDX',  for 'Fermilab Test Beam', may be obsolete
#define RDX_SZWRDS_INT      0x400       //Words or 0x800 bytes for total 2048dec 'sendRDX()'
#pragma pack(4)                         //force 16bit boundary on all types
uint16  RDX0_dmaDATA[RDX_SZWRDS_INT+4]; //fpga to uC DMA buffer, add padding 'sendRDX'
uint16  RDX1_dmaDATA[RDX_SZWRDS_INT+4]; //fpga to uC DMA buffer, add padding 'sendRDX'



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



//-------------------- DMA CODE ------------------------------
/** void dmaConfigCtrlPacket(uint32 sadd,uint32 dadd,uint32 dsize)
*
*   configuring dma control packet stack
*
*       sadd  > source address
*       dadd  > destination  address
*       dsize > data size
*
*   @ note : after configuring the stack the control packet needs to be set by calling dmaSetCtrlPacket()
*
*  note  The frame/element count fields of the the  ITCOUNT register is only 13 bit wide, 
*  note  hence a max of 8191 frames/elemets. bits 15:13 are ignored - so 8192 is same as 0.
*/
void dmaConfigCtrlRDX(uint32 sadd,uint32 dadd,uint32 dsize)
{
  g_dmaCtrl_RDX.SADD      = sadd;			  /* source address             */
  g_dmaCtrl_RDX.DADD      = dadd;			  /* destination  address       */
  g_dmaCtrl_RDX.CHCTRL    = 0;                /* channel control            */
  g_dmaCtrl_RDX.FRCNT	  = 1;                /* frame count                */
  g_dmaCtrl_RDX.ELCNT     = dsize;            /* Element / Frame            */
  g_dmaCtrl_RDX.ELDOFFSET = 0;                /* element destination offset */
  g_dmaCtrl_RDX.ELSOFFSET = 0;		          /* element destination offset */
  g_dmaCtrl_RDX.FRDOFFSET = 0;		          /* frame destination offset   */
  g_dmaCtrl_RDX.FRSOFFSET = 0;                /* frame destination offset   */
  g_dmaCtrl_RDX.PORTASGN  = 4;                /* assign dma #               */
  g_dmaCtrl_RDX.RDSIZE    = ACCESS_16_BIT;	  /* read size                  */
  g_dmaCtrl_RDX.WRSIZE    = ACCESS_16_BIT; 	  /* write size                 */
  g_dmaCtrl_RDX.TTYPE     = FRAME_TRANSFER ;  /* transfer type              */
//g_dmaCtrl_RDX.ADDMODERD = ADDR_INC1;        /* address mode read          */
//g_dmaCtrl_RDX.ADDMODEWR = ADDR_INC1;        /* address mode write         */
//g_dmaCtrl_RDX.ADDMODEWR = ADDR_OFFSET;      /* address mode write         */
  g_dmaCtrl_RDX.AUTOINIT  = AUTOINIT_ON;      /* autoinit                   */
}


//dma 0 used to copy data from FPGA to uC Memory buffer max buffer =RDX_SIZE =2048
int trigDMA0_fpga2mem1(uint16_t wordCnt, uint16_t *RD16SWP )  
{
	dmaDisable();
	//assigning dma request: channel-0 with request line - 1
	dmaReqAssign(0,1 );
    //prevent DMA lockup, force count > zero
    if(wordCnt==0)
      wordCnt=8;
	//configuring dma control packets   (srcadd, destadd, datasize)

    //alternate buffers
    if(BinStRDX.gActBuf) 
        dmaConfigCtrlRDX((uint32)RD16SWP, (uint32)&RDX1_dmaDATA, wordCnt );
    else
        dmaConfigCtrlRDX((uint32)RD16SWP, (uint32)&RDX0_dmaDATA, wordCnt );
      
	g_dmaCtrl_RDX.RDSIZE = ACCESS_16_BIT;    //change read size to 16 bits
    g_dmaCtrl_RDX.ADDMODERD = ADDR_FIXED;    //address mode read fpga fixed addr
    g_dmaCtrl_RDX.ADDMODEWR = ADDR_INC1;     //address mode write uC buf
	// setting dma control packets, upto 32 control packets are supported
	dmaSetCtrlPacket(DMA_CH0 ,g_dmaCtrl_RDX);

	//setting the dma channel 0 to trigger on software request
	dmaSetChEnable(DMA_CH0, DMA_SW);
	dmaEnableInterrupt(DMA_CH0, FTC);
	//enabling dma module
	dmaEnable();
    return 0;
}
        

//#define TESTDATA
//The 'RDX' command for testing mostly, 
//    Request 'FEBs' binary data via 'PHY PORT'
//    Returns FEB data, previous trig required
//Send binary data using ethernet 'PHY' port on uC
//function time ~160uS for whole function, early exit if ePHY xmits busy
//if busy, main() returns here in ~14uS and rechecks if ePHY ready
//
int sendRDX()
{
    static uint16_t    dWordsMoved=0, fpgaEmp=0;
    u_16Bit* dPtr;
      
     if (BinStRDX.gSndWrdCnt==0)
        {
        iFlag &= ~DAQ_BUSY;                     //uC iFlag reg,  DAQ 'rdb' done
        return 0;
        }
       
    //move 16bits data words
    if (dWordsMoved==0)                         //Note: datWrdsMoved should always an even number
        {
        //DMA's data,   FPGA==> uC_dmaBuf
        //DMA Stream enable
        BinStRDX.gDMA_Fpga2Mem=1;  //mark as active
        
        //max fill each time
        if (BinStRDX.gSndWrdCnt >= RDX_SZWRDS_INT )
            dWordsMoved= RDX_SZWRDS_INT;
        else 
            dWordsMoved= BinStRDX.gSndWrdCnt;

#ifndef TESTDATA        
        //using real feb sdRam data
        //limit word cnt to what remains in active fpga
        if (dWordsMoved > FPGA_Trig_Stat.WrdToSnd) //words to send here
            {
            dWordsMoved= FPGA_Trig_Stat.WrdToSnd;
            fpgaEmp=1;
            }
        //trap zero cout
        if (dWordsMoved > 0)
            {
            if (FPGA_Trig_Stat.Fpga1of4==0)
                trigDMA0_fpga2mem1(dWordsMoved, &SDR_RD16_0);      //move from FPGA-to-MEMORY Buffer 'RDB_dmaDATA[D_SIZE]'  
            else if (FPGA_Trig_Stat.Fpga1of4==1)
                trigDMA0_fpga2mem1(dWordsMoved, &SDR_RD16_1);      //move from FPGA-to-MEMORY Buffer 'RDB_dmaDATA[D_SIZE]'  
            else if (FPGA_Trig_Stat.Fpga1of4==2)
                trigDMA0_fpga2mem1(dWordsMoved, &SDR_RD16_2);      //move from FPGA-to-MEMORY Buffer 'RDB_dmaDATA[D_SIZE]'  
            else if (FPGA_Trig_Stat.Fpga1of4==3)
                trigDMA0_fpga2mem1(dWordsMoved, &SDR_RD16_3);      //move from FPGA-to-MEMORY Buffer 'RDB_dmaDATA[D_SIZE]'  
            //if req larger than data pool start over at fpga 0
            else if (FPGA_Trig_Stat.Fpga1of4>3)
                {
                trigDMA0_fpga2mem1(dWordsMoved, &sTstCntLO);       //out of data, send test counter as data
                }
            }

        //End of fpga buffer, go to next fpga
        if (fpgaEmp==1)
            {
             do {
                if (FPGA_Trig_Stat.Fpga1of4 < 4)
                    FPGA_Trig_Stat.Fpga1of4++;
                if (FPGA_Trig_Stat.Fpga1of4==4)
                    {
                    //ERROR COUNTER
                    FPGA_Trig_Stat.OverRun++;
         //tek added may2018 2 lines
                    BinStRDX.gSndWrdCnt=0;
                    break;
                    }
                fpgaEmp=0;
                FPGA_Trig_Stat.WrdToSnd= FPGA_Trig_Stat.WrdCnt[FPGA_Trig_Stat.Fpga1of4]; //use word cnt on PHT xFERs
                if (FPGA_Trig_Stat.Mask[FPGA_Trig_Stat.Fpga1of4] == 0)
                    FPGA_Trig_Stat.WrdToSnd=0;
                } while (FPGA_Trig_Stat.WrdToSnd==0);  //if fpag(n) has no data, then skip
            }
        else  
            {
            FPGA_Trig_Stat.WrdToSnd -= dWordsMoved;
            }
#else        
    //use sequential test data on FPGA for testing
        FPGA_Trig_Stat.Fpga1of4=4;
        trigDMA0_fpga2mem1(dWordsMoved, &sTstCntLO);   //move from FPGA-to-MEMORY Buffer 'RDB_dmaDATA[D_SIZE]'  
#endif
        } 
    
    //possible no bytes were moved from 1of4 fpga(s), don't continue else dma will lock up code
    if (dWordsMoved== 0)
        return  BinStRDX.gSndWrdCnt;
       

    //DMA FROM FPGA 2 uC done?
    //while (BinStRDX.gDMA_Fpga2Mem==1);
    //tek add timeout check here
    
    if (BinStRDX.gDMA_Fpga2Mem==1)          //if busy, come back later
        return  BinStRDX.gSndWrdCnt;

    //set flag that send is pending/active, will be clear in 'EMACTxIntISR()'
    while(paKet.EMACTxBsy); //tek

    //eTX needs at least 3 words in a packet to send
    //if we are at end hold and split data on final 2 packets 
    if ((BinStRDX.gSndWrdCnt==(RDX_SZWRDS_INT+1)) || (BinStRDX.gSndWrdCnt==(RDX_SZWRDS_INT+2)))  //do memcopy to hold partial buffer
        {
        if (FPGA_Trig_Stat.Fpga1of4==0)      dPtr= &SDR_RD16_0;         
        else if (FPGA_Trig_Stat.Fpga1of4==1) dPtr= &SDR_RD16_1;         
        else if (FPGA_Trig_Stat.Fpga1of4==2) dPtr= &SDR_RD16_2;         
        else if (FPGA_Trig_Stat.Fpga1of4==3) dPtr= &SDR_RD16_3;          
        else if (FPGA_Trig_Stat.Fpga1of4==4) dPtr= &sTstCntLO;         

        if(BinStRDX.gActBuf) 
            {
            *(RDX1_dmaDATA+(RDX_SZWRDS_INT))= *dPtr;        //1st word
            if (BinStRDX.gSndWrdCnt==RDX_SZWRDS_INT+2)
                *(RDX1_dmaDATA+(RDX_SZWRDS_INT+1))= *dPtr;  //2nd word if avail                
            }
        else
            {
            *(RDX0_dmaDATA+(RDX_SZWRDS_INT))= *dPtr;        //1st word
            if (BinStRDX.gSndWrdCnt==RDX_SZWRDS_INT+2)
                *(RDX0_dmaDATA+(RDX_SZWRDS_INT+1))= *dPtr;  //2nd word if avail                
            }
        //adjust word counters
        if (BinStRDX.gSndWrdCnt==(RDX_SZWRDS_INT+1))
          dWordsMoved+=1;
        else
          dWordsMoved+=2;
        }
    
    //now adjust stored byte count
    BinStRDX.gSndWrdCnt -= dWordsMoved;         //freeSize is always <= MAX BUFFER
    
    //creatpacket
    //alternate buffers
    if(BinStRDX.gActBuf) 
        fill_daq_packet(dWordsMoved<<1,(u_8Bit*)RDX1_dmaDATA); //ByteCnt, data buf ptr
    else
        fill_daq_packet(dWordsMoved<<1,(u_8Bit*)RDX0_dmaDATA); //ByteCnt, data buf ptr

    //'EMACTxIntISR()' added line "g_EMACTxBsy=0 for done status;
    paKet.EMACTxBsy=1;
    //send data to MAC
    //EMACTransmit() returns TRUE if valid data is sent and is transmitted.
    EMACTransmit(&hdkif_data[0], &PacLnkLst[0]);  //two packet chained

    //alternate buffers
    BinStRDX.gActBuf= BinStRDX.gActBuf? 0:1; 
    
    //**  Speed things up                           **
    //**  Now continue loop, dont wait for eTX done **
    //**  On next pass eTX should be done           **
    //**  so Load buffers for next eTX              **
    //**  then Check eTX and wait if needed         **
    //**  Then continue reload eTX                  **
    
    //some delay needed else false early done??? ~75uS on '1038 byte packet'
    //uDelay(20);
    //set flag that send is pending/active, will be clear in 'EMACTxIntISR()'
    // while(g_EMACTxBsy); //tek
    
    //packet buffer has been filled and sent
     if (BinStRDX.gSndWrdCnt==0)
        {
        iFlag &= ~DAQ_BUSY;                     //uC iFlag reg,  DAQ 'rdb' done
        }
    dWordsMoved= 0;                             //ready for next pass
    return  BinStRDX.gSndWrdCnt;
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
        if (paKet.EMACRxRdy==0)         //Rec'd packet 'g_EMACRxRdy==1'
            return 0;  
    
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
        HappyBus.myLnkCntrlTmp= (HappyBus.BrdNumb>>8); //upper controller numb
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
    
#ifdef USE_INTRDAQ     //'ver_ePHY.h'
    if (HappyBus.CmdType== eCMD_DAQ_DY2)
        return 0;    
#endif    
    if (HappyBus.CmdType== eCMD79_LINK_INIT)
        return cmdByteSz;
    else if (HappyBus.CmdType== eCMD72_STAB)
        return cmdByteSz;
  //else if (HappyBus.BrdNumb== HappyBus.myLinkNumber)
  //allow all commands, u2eController has individual ports 1-24
    else if ((HappyBus.BrdNumb>0) && (HappyBus.BrdNumb<25))
        return cmdByteSz;
    else
        return 0;
}
    


extern struct  msTimers mStime; 

//read packet length, needed by 'LDF' function
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

//READs micro bunch address and returns data, 
//buffers for max requests of uB_Reqs8, each with word count of uB_Sz512W
//set flag reg to prevent 'Telnet Process' checking when in IRQ_MODE
//
int eCMD_DAQ_DY2_Handler(uSHT* PTR16, int cmdTyp, int ubCnt )
{
    int i=0,w,cmds=0, stat;
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
    pCtrl.gBunErr=0;
    pCtrl.RetSzBytes=0;
    
    //todo tek aug2019
    //this code does not account for "fpgas/AFE chips" power down mode
        
    //creatpacket for PHY DMA xmit       
    //cmdLen is number of uBunch Req in packet from controller   
    //memory buffer avail for max of uB_Reqs8 uB request of uB_Sz512W (words each)
    for(w=0; w<ubCnt; w++)  
        {
        int wCnt=0;
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
        
        wCnt= eFIFO_wCNT1;      //for testing
        wCnt= eFIFO_wCNT2;
        wCnt= eFIFO_wCNT3;
        wCnt= eFIFO_wCNT4;
        
        //FIFOs now empty
        //Global broadcast, all fpgas fill fifos with new data
        //After global address set 0x312,0x313 wait for data fifo auto fill go non empty
        //REG16(fpgaBase+0x312*2)= pCtrl.uBunHI;  //ub High                    
        //REG16(fpgaBase+0x313*2)= pCtrl.uBunLO;  //ub Low    
        BCST_GEO_312= pCtrl.uBunHI;     //ub High                    
        BCST_GEO_313= pCtrl.uBunLO;     //ub Low    
 

        //Trigger FIFO loading of new data
        //SDPTR_BCASTH0= pCtrl.uBunHI;     //ub High
        //SDPTR_BCASTL0= pCtrl.uBunLO;     //ub Low           
        //SDPTR_BCASTH1= pCtrl.uBunHI;     //ub High
        //SDPTR_BCASTL1= pCtrl.uBunLO;     //ub Low
        //SDPTR_BCASTH2= pCtrl.uBunHI;     //ub High
        //SDPTR_BCASTL2= pCtrl.uBunLO;     //ub Low
        //SDPTR_BCASTH3= pCtrl.uBunHI;     //ub High
        //SDPTR_BCASTL3= pCtrl.uBunLO;     //ub Low
        uDelay(5);
        
        //Global broadcast addr BCST_GEO_317, read Event FIFO Status
        //Wait for fpgas data to fill fifo (non empty)
        while (1)
            { 
            stat= BCST_GEO_317 &0xF;        //FPGA fifo event data rdy flags=1)
            if(stat==0xf) break;
            if(++i>10000)                   //Diagnostics, short wait, fpga should already be done
                {
                PaketStats.TOflag++;
                pCtrl.gBunErr |= BIT8;      //error flag if not empty 
                break;
                }
            }
        
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
    //No problem, Min packet always has 4*4 word header (good)
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

#pragma optimize=speed
int ubGetData(int fpga1of4, sPTR rAddr)
{
    u_16Bit UbunHI_Ram, UbunLO_Ram, err=0, ovrflow=0; 
    static u_16Bit fpga=0;
    pCtrl.uBDatSz= *rAddr;                  //get data word cnt, Note must be unsigned integer for math

    if(pCtrl.uBDatSz & BIT15)               //fpga sets BIT15 if overflow
        { 
        ovrflow=1;                          //initial default value=zero
        pCtrl.uBDatSz &= ~BIT15;            //remove overflag bit15 from data size
        }
    pCtrl.uBDatSz -=3;                      //skip 3WrdMemHdrSiz(wCnt,uBhi,uBlo)
    UbunHI_Ram= *rAddr;                     //get data bun# high16 from fifo
    UbunLO_Ram= *rAddr;                     //get data bun# LOW16 from fofo
    
    
    //Test uBunReq# to data in sdRam for match
    //if((UbunLO_Ram.uBunHI==UbunHI_Ram) && (pCtrl.uBunLO==UbunLO_Ram))
    if((pCtrl.uBunHI==UbunHI_Ram) && (pCtrl.uBunLO==UbunLO_Ram))
        {
        //Request Matches ram data stored in buffer
        //On 1st FPGA processing, header must be created/stored as 1st 4 words of uB return data
        if(fpga1of4==0)                   
            {
            //1st fpga check, include the global header
            fpga=0;
            pCtrl.uHdrIdx= pCtrl.g_idx;
            pCtrl.g_idx++;                          //4word Header, skip wrd cnt here, filled in after 4 fpga processed
            xWrds32767[pCtrl.g_idx++]= UbunHI_Ram;  //4word Header, Ubun High 'from sdRam'
            xWrds32767[pCtrl.g_idx++]= UbunLO_Ram;  //4word Header, Ubun Low  'from sdRam'
            pCtrl.g_idx++;                          //4word Header, skip Status until end
            pCtrl.pacSiz +=4;
            paKet.SndTotWrds += 4;                  //add 4_word_header size            
            }
        //get microbunch data if reqsiz > 0 and <Max
        if(ovrflow==0) //okay here
            {
            //No-Overflow, Okay to process
            if(pCtrl.uBDatSz>0)                     //may not be needed but prevents boom
                {
                //tek Feb2020, FPGA code sets flag if size greater than uB_Sz512W
                //leaving this original if(...) code as backup checking for now
                //SOFTWARE RECHECK of overflow, remove at some point tek Mar2020
                //if(pCtrl.uBDatSz>uB_Sz512W)             //Overflow check uC compare (duplicate check, nows its done in FPGA)
                //    {
                //    //Change readout size and set overflow software check error flag
                //    pCtrl.uBDatSz= uB_Sz512W;       //limit, room for (24*10_Wrd_Samples)
                //    PaketStats.uBunOvrErrSoft++;    //uC check for overflag here
                //    pCtrl.gBunErr |=(BIT4<<fpga);   //to much data flag per fpga
                //    }
                //block move data, movStr16_NOICSRC(SRC 'no incr FIFO', DEST, WordCount)
                movStr16_NOICSRC((snvPTR)rAddr, (snvPTR)&xWrds32767[pCtrl.g_idx], pCtrl.uBDatSz); 
                pCtrl.g_idx  += pCtrl.uBDatSz;
                pCtrl.pacSiz += pCtrl.uBDatSz;
                //uBunSz may be truncated if to large
                paKet.SndTotWrds += pCtrl.uBDatSz;   //add data words read
                }
            else
                err++;                                      //error counter
            }
        else
            { 
            //Overflow, Error processing
            //Handle overflow here, set flags but Sends Data pre-overflow using count controlled in FPGA logic (tek feb2020)
            //Change readout size and set overflow software check error flag
            //if(pCtrl.uBDatSz>uB_Sz512W)             //Overflow check uC compare (duplicate check, nows its done in FPGA)
            //    {
            //    pCtrl.uBDatSz= uB_Sz512W;           //limit, room for (24*10_Wrd_Samples)
            //    PaketStats.uBunOvrErrSoft++;        //uC check for overflag here
            //    }              
            //block move data, movStr16_NOICSRC(SRC 'no incr FIFO', DEST, WordCount)
            movStr16_NOICSRC((snvPTR)rAddr, (snvPTR)&xWrds32767[pCtrl.g_idx], pCtrl.uBDatSz); 
            pCtrl.g_idx  += pCtrl.uBDatSz;
            pCtrl.pacSiz += pCtrl.uBDatSz;
            //uBunSz may be truncated if to large
            paKet.SndTotWrds += pCtrl.uBDatSz;       //add data words read                                          
            PaketStats.uBunOvrErrs++;               //BIT15 overflag here set by FPGA 
            pCtrl.gBunErr |=(BIT4<<fpga);           //to much data flag per fpga
            }            
        }
    
    else
        //Request Number Does Not Match Ram Data Moved to FIFO
        {
        if(fpga1of4==0)                             //1st fpga check, include the global header
            {
            u_32Bit uB32REQ, uB32MEM;
            fpga=0;
            //error, no match on uBunch Req IDs, return 4_word_header
            paKet.SndTotWrds += 4;   
            pCtrl.uHdrIdx= pCtrl.g_idx;
            pCtrl.g_idx++;                          //4word Header skip wrd cnt until end
            xWrds32767[pCtrl.g_idx++]= pCtrl.uBunHI;//4word Header Ubun High 'requested'
            xWrds32767[pCtrl.g_idx++]= pCtrl.uBunLO;//4word Header Ubun Low  'requested'
            pCtrl.g_idx++;                          //4word Header skip Status until end
            pCtrl.pacSiz +=4;
            PaketStats.uBunReqErrs++;               //on going error total for display cmd'UB'

            //tek March2020 added Diag Msg to 1 of 3 ports
            //if Sock3 mode and socket connected, send it if socket connected
            if ((getSn_SSR(sock2)==SOCK_ESTABLISHED)&&(pCtrl.g_DEBUG_MODE==sock2))
                { 
                uB32MEM= (( UbunHI_Ram<<16)+( UbunLO_Ram));
                uB32REQ= ((pCtrl.uBunHI<<16)+(pCtrl.uBunLO));                    
                sprintf(msgBuf," uB Rec'd : %08X\r\n", uB32REQ);
                sprintf(msgBuf1,   " uB SDRam : %08X\r\n", uB32MEM);
                strcat (msgBuf,msgBuf1);
                if (pCtrl.g_DEBUG_MODE==sock2)      //send to socket 2 (we have sockets 0,1,2)
                    putBuf(sock2, msgBuf,0);  
                else if (pCtrl.g_DEBUG_MODE==ePhyIO)//send on lvds fm, this is the cmd 'LC' link
                    putBuf(ePhyIO, msgBuf,0); 
                else if (pCtrl.g_DEBUG_MODE==tty)   //out to usb port, user should set baud high to prevent slowdowns
                    putBuf(tty, msgBuf,0); 
                }
            }
        err++;                                      //error counter
        }
    fpga++;
    return(err);
}




  

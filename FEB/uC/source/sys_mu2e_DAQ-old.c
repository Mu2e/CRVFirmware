//******************************************************************
// @file sys_mu2e_daq.c 
//  Fermilab Terry Kiper 2016-2019
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
g_dmaCTRL g_dmaCtrl_RDX;                        //dma control packet config stack

//eTX buffer for final data packet less than 3 words
uint16_t RDX_Buf_EOT[10];

extern  uint32  volatile iFlag;

void    fill_daq_packet(int size2DataCnt, u_8Bit *dataPtr); //mac phy packet loader 'sys_mu2e_daq.c'


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



//DMA Wiznet
#define RDX_SZWRDS_INT      0x400       //words or 0x800 bytes for total 2048dec 'sendRDX()'
#define PACKs14             14          //set up for 14 packets of (1514 Bytes) each

#pragma pack(4)                         //force 16bit boundary on all types
uint16  RDX0_dmaDATA[RDX_SZWRDS_INT+4]; //fpga to uC DMA buffer, add padding 'sendRDX'
uint16  RDX1_dmaDATA[RDX_SZWRDS_INT+4]; //fpga to uC DMA buffer, add padding 'sendRDX'

#pragma pack(4)                         //force 16bit boundary on all types
pbuf_t  PacKits[PACKs14]; 


//1 uBun max of 512 words each * 4 fpgas == 2048 words per uBun
//8 req max per pack from Controller == 2048*8= 16,384 Words
//FEB EMACxmit has PACKs(8) * MAX_TRANSFER_UNIT(1514 Bytes)
uint16_t    xWrds16384[2048*8];        //room for '8 full ubun req', req '>11 PACKs' of 1500 byt each


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



// Fill EMAC Packet hdr buffer and xmitData array, called by cmd 'RDX'
// The packet needs to be in the following format:
//   | Destination MAC Address (6 bytes)
//   | Source MAC Address (6 bytes)
//   | Length/Type (2 bytes)
//   | Data (46- 1500 bytes)
// The data can be split up over multiple pbufs which are linked as a linked list.
//function time > 2uS
//
void fill_daq_packet(int szBytCnt, u_8Bit *dataPtr)  //byte count even and greater than 2
{     
    int i,ovrflow=0, pacCnt=PACKs14;     
    //error check
    if (szBytCnt < MAX_TRANSFER_UNIT)
      pacCnt=1;
    else
        {
        ovrflow= szBytCnt/ pacCnt;              //divide then mult and see if cnt match
        ovrflow= szBytCnt- (ovrflow*pacCnt);    //if counts dont match add extra to packs[1].len
        }
    //fill in packet 'pbuf'...packs[n]
    for(i=0; i<pacCnt; i++)     
        {     
        PacKits[i].tot_len= szBytCnt;           //len to of all buffers (bytes)
        PacKits[i].len= (szBytCnt/pacCnt);      //len to this buffer
        if(i==0) 
            PacKits[i].len +=ovrflow;           //add overflow to 1st 'pbuf'
        PacKits[i].payload = (u_8Bit*)dataPtr;
       
        //ready data ptr for next pass
        dataPtr+= PacKits[i].len;               //incr byte ptr by byte count        
        
        //next emac header
        if(i!= (pacCnt-1))
            PacKits[i].next = &PacKits[i+1];    //ref for next packet, repeat loop
        }
    PacKits[pacCnt-1].next= NULL;               //final emac packet marked as end
}

#define TRANSFER_BYTES          1500            //1540 max bytes

void fill_daq_packet1(int szBytCnt, u_8Bit *dataPtr)  //byte count even and greater than 2
{     
    int i,ovrflow=0, pacCnt=PACKs14, pacSiz;     
    //error check
    if (szBytCnt < TRANSFER_BYTES)
      pacCnt=1;
    else
        {
        pacCnt= (szBytCnt/TRANSFER_BYTES)+1;
        pacSiz= szBytCnt/ pacCnt;              //divide then mult and see if cnt match
        ovrflow= szBytCnt- (pacSiz*pacCnt);    //if counts dont match add extra to packs[1].len
        //if(ovrflow)                            //incr, overflow due to non full packet
        //  pacCnt++;
        }
    //fill in packet 'pbuf'...packs[n]
    for(i=0; i<pacCnt; i++)     
        {     
        PacKits[i].tot_len= szBytCnt;           //len to of all buffers (bytes)
        PacKits[i].len= pacSiz;                 //len to this buffer
        if(i==0) 
            PacKits[i].len +=ovrflow;          //add overflow to 1st 'pbuf'
        PacKits[i].payload = (u_8Bit*)dataPtr;
       
        //ready data ptr for next pass
        dataPtr+= PacKits[i].len;              //incr byte ptr by byte count        
        
        //next emac header
        if(i!= (pacCnt-1))
            PacKits[i].next = &PacKits[i+1];   //ref for next packet, repeat loop
        }
    PacKits[pacCnt-1].next= NULL;              //final emac packet marked as end
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
    EMACTransmit(&hdkif_data[0], &PacKits[0]);  //two packet chained

    //alternate buffers
    BinStRDX.gActBuf= BinStRDX.gActBuf? 0:1; 
    
    //**  Speed things up                           **
    //**  Now continue loop, dont wait for eTX done **
    //**  On next pass eTX should be done           **
    //**  so Load buffers for next eTX              **
    //**  then Check eTX and wait if needed         **
    //**  Then contine reload eTX                   **
    
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
      //hLO_TP45; 
        return 0;                       //done here
        }
    //added testing mode that uses already downloaded test data
    else if (phyHdr.PacType== eCMD_DAQ_DY2uB_TST)
        {
        //*** 4uS overhead to get to 'eCMD_DAQ_DY2_Handler()' in intr routine ***
        eCMD_DAQ_DY2_Handler_TST(PTR16, phyHdr.PacType, phyHdr.PacCmdLen );
        //packet processed, point to next packet in buffer
        phyHdr.PacActive++;
        if (phyHdr.PacActive>= PacBufs) //'ver_io.h'
            phyHdr.PacActive=0; 
      //hLO_TP45; 
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




#pragma optimize=speed
int ubGetDataOld(int offset, sPTR rAddr)
{
    u_16Bit UbunHI_Ram, UbunLO_Ram, err=0;  

    //set BroadCast Address to sdRam 0-3
    //SDPTR_BCASTH= pCtrl.uBunHI;
    //SDPTR_BCASTL= pCtrl.uBunLO;
    //uDelay(1);
    
    
    pCtrl.uBunSz= *rAddr;                   //get data word cont with limit
    pCtrl.uBunSz -= 3;                      //dont include header size ..Note must be unsigned integer for math
    UbunHI_Ram= *rAddr;                     //get data bun# high16
    UbunLO_Ram= *rAddr;                     //get data bun# LOW16
    
    //test uBunReq# to data in sdRam for match
    //
    if ((pCtrl.uBunHI==UbunHI_Ram) && (pCtrl.uBunLO==UbunLO_Ram))
        {
        //request matches ram data stored in buffer
        if(offset==0)   //1st fpga check, include the global header
            {
            pCtrl.uHdrIdx= pCtrl.g_idx;
            pCtrl.g_idx++;                          //4word Header skip wrd cnt until end
            xWrds16384[pCtrl.g_idx++]= UbunHI_Ram;  //4word Header Ubun High 'from sdRam'
            xWrds16384[pCtrl.g_idx++]= UbunLO_Ram;  //4word Header Ubun Low  'from sdRam'
            pCtrl.g_idx++;                          //4word Header skip Status until end
            pCtrl.pacSiz +=4;
            paKet.eSEND_TOT += 4;                   //add 4_word_header size
            }
        //get microbunch data if reqsiz>3
        if(pCtrl.uBunSz>0)
            {
            //test for bufoverflow
            if(pCtrl.uBunSz>0x100)                  //0x100 words, buffer must be 100
                {
                pCtrl.uBunSz= (10*24); //0x100;      //room for (24*10_Word samples)
                PaketStats.uBunOvrErrs++;
                pCtrl.gBunErr |=0x10;               //to much data
                }
            movStr16_NOICSRC((snvPTR)rAddr, (snvPTR)&xWrds16384[pCtrl.g_idx], pCtrl.uBunSz); 
            pCtrl.g_idx  += pCtrl.uBunSz;
            pCtrl.pacSiz += pCtrl.uBunSz;
            //uBunSz may be truncated if to large
            paKet.eSEND_TOT += pCtrl.uBunSz;        //add data words read
            }
        }
    else
        {
        //request does not match Ram Data store to buffer
        //
        if(offset==0)                               //1st fpga check, include the global header
            {
            paKet.eSEND_TOT += 4;   //error, no match on uBunch Req IDs, return 4_word_header
            pCtrl.uHdrIdx= pCtrl.g_idx;
            pCtrl.g_idx++;                          //4word Header skip wrd cnt until end
            xWrds16384[pCtrl.g_idx++]= pCtrl.uBunHI;//4word Header Ubun High 'requested'
            xWrds16384[pCtrl.g_idx++]= pCtrl.uBunLO;//4word Header Ubun Low  'requested'
            pCtrl.g_idx++;                          //4word Header skip Status until end
            pCtrl.pacSiz +=4;
            PaketStats.uBunReqErrs++;               //on going error total for display cmd'UB'
            }
        err++;                                      //error counter
        }
    return(err);
}

  

/*****************************************************************************/
/*****************************************************************************/
/**************      TEST MODE USED FAKE DATA                    *************/
/**************      FAKE DATA MUST BE DOWNLOADED BEFORE USED    *************/
/*****************************************************************************/
/*****************************************************************************/
//
#pragma optimize=speed
//READs micro bunch address and returns data, max 256 words(minus Hdr) per ubun req
//set flag reg to prevent 'Telnet Process' checking when in IRQ_MODE
//xWrds10400 has room for '10 full uBun req', requires '8 ePACKs' of 1500 bytes each
int eCMD_DAQ_DY2_Handler_TST(uSHT* PTR16, int cmdTyp, int cmdLen )
{
    int j,w,cmds=0;
    u_32Bit uBun;
    sPTR ucSDramLOx; 
    
    pCtrl.g_idx=0;
    paKet.uBunPerPac= cmdLen;
    
    //init header status
    pCtrl.g_idx= 0;
    pCtrl.gBunErr=0;
    pCtrl.RetSzBytes=0;
    
    //creatpacket for PHY        
    //cmdLen is number of uBunch Req in packet from controller    
    for(j=0; j<cmdLen; j++)  
        {
    hLO_TP45;         
        cmds++;
        pCtrl.uBunLO= *PTR16++;
        pCtrl.uBunHI= *PTR16++; 
        pCtrl.pacSiz=0;                
        
    //tek, this broadcast needs checking out more        
    //tek, this broadcast needs checking out more  
        
        //set BroadCast Address to sdRam 0-3
        //SDPTR_BCASTH= pCtrl.uBunHI;
        //SDPTR_BCASTL= pCtrl.uBunLO;
        //uDelay(1);
        
        uBun= (pCtrl.uBunHI<<16) + pCtrl.uBunLO;
        uBun <<=9;
        pCtrl.uBunHI_SHT = uBun >> 16;
        pCtrl.uBunLO_SHT = uBun & 0xffff;                
                
        //read all fpgas
        SET_SDADDR_RDx((0x000),pCtrl.uBunHI_SHT, pCtrl.uBunLO_SHT);    //fpgaOffset,addrH,addrL                 
        if (ubGetDataOld(0x000, &SDR_RD16_0))   //get SDR_RD16_0 data
            pCtrl.gBunErr |=0x01;
        
        SET_SDADDR_RDx((0x400),pCtrl.uBunHI_SHT, pCtrl.uBunLO_SHT);    //fpgaOffset,addrH,addrL                 
        if (ubGetDataOld(0x400,&SDR_RD16_1))       //get SDR_RD16_1 data
            pCtrl.gBunErr |=0x02;

        SET_SDADDR_RDx((0x800),pCtrl.uBunHI_SHT, pCtrl.uBunLO_SHT);    //fpgaOffset,addrH,addrL                 
        if (ubGetDataOld(0x800,&SDR_RD16_2))       //get SDR_RD16_2 data
            pCtrl.gBunErr |=0x04;
        
        SET_SDADDR_RDx((0xc00),pCtrl.uBunHI_SHT, pCtrl.uBunLO_SHT);    //fpgaOffset,addrH,addrL                 
        if (ubGetDataOld(0xc00,&SDR_RD16_3))       //get SDR_RD16_3 data
            pCtrl.gBunErr |=0x08;
            
        //update total word count header
        xWrds16384[pCtrl.uHdrIdx]  = pCtrl.pacSiz;  //4word Header Ubun wrdCnt
        xWrds16384[pCtrl.uHdrIdx+3]= pCtrl.gBunErr; //4word Header Ubun errors
        pCtrl.RetSzBytes += pCtrl.pacSiz;
        hHI_TP45;          
        }
    hLO_TP45;          

    //Check data size, Minimum ePHY packet size is 8 words, pad out if needed
    //Worst case Minimum size with no hit data is 4 words header only
    if (pCtrl.RetSzBytes < ePHY_PAC_MIN_Sz8)     //ePHY_PAC_MIN_Sz8=8 'ver_ePHY.h'
        {
        pCtrl.RetSzBytes = ePHY_PAC_MIN_Sz8;
        for(; pCtrl.g_idx < ePHY_PAC_MIN_Sz8; pCtrl.g_idx++) //fill to minimum size 
            xWrds16384[pCtrl.g_idx]=0x1234;
        }
    pCtrl.RetSzBytes<<=1;                 //convert word to byte count
     
    //set flag that send is pending/active, will be clear in 'EMACTxIntISR()'
    //g_EMACTxBsy cleared in emacTxNotification()... may get missed
    //todo, this should not happen, check counter to verify
    for(w=0; w<2000000; w++)
        {
        if(paKet.EMACTxBsy==0)
            break;
        }
    if(w>=2000000)
        PaketStats.TOflag++;       //test added for allow break point for testing

    //tek, todo pERRORS structure for error counters
    //tek, todo pERRORS structure for error counters
              
    fill_daq_packet(pCtrl.RetSzBytes, (u_8Bit*)&xWrds16384); //ByteCount, data buffer ptr
            
    paKet.EMACTxBsy=1;              //flag tx as busy                  
    EMACTransmit(&hdkif_data[0], &PacKits[0]); //send data to MAC , two or more packet chained
    hHI_TP45;          
    return 0;
}         



//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
//----------------------   New Version DAQ Handler Oct 2018  ---------------------------
//----------------------   New Version DAQ Handler Oct 2018  ---------------------------
//----------------------   New Version DAQ Handler Oct 2018  ---------------------------
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------

#pragma optimize=speed
//READs micro bunch address and returns data, max 256 words(minus Hdr) per ubun req
//set flag reg to prevent 'Telnet Process' checking when in IRQ_MODE
//xWrds10400 has room for '10 full uBun req', requires '8 ePACKs' of 1500 bytes each
int eCMD_DAQ_DY2_Handler(uSHT* PTR16, int cmdTyp, int cmdLen )
{
    int i=0,j,w,cmds=0;
    //u_32Bit uBun;
    sPTR ucSDramLOx; 
    
    //wait, slight chance that 'xWrds16384' buffers may be overwritten before ether xmit
    //set flag that send is pending/active, will be clear in 'EMACTxIntISR()'
    //g_EMACTxBsy cleared in emacTxNotification()... may get missed
    //todo, this should not happen, check counter to verify
    for(w=0; w<2000000; w++)
        {
        if(paKet.EMACTxBsy==0)
            break;
        }
    if(w>=2000000)
        PaketStats.TOflag++;       //test added for allow break point for testing

    paKet.uBunPerPac= cmdLen;
    
    //init header status
    pCtrl.g_idx= 0;
    pCtrl.gBunErr=0;
    pCtrl.RetSzBytes=0;
    
    //creatpacket for PHY        
    //cmdLen is number of uBunch Req in packet from controller    
    for(j=0; j<cmdLen; j++)  
        {
        hLO_TP45;         
        cmds++;
        pCtrl.uBunLO= *PTR16++;
        pCtrl.uBunHI= *PTR16++; 
        pCtrl.pacSiz=0;                
        
        //set uBunch Address Global to all 4 FPGAs
        REG16(fpgaBase+0x312*2)= pCtrl.uBunHI;  //ub High                    
        REG16(fpgaBase+0x313*2)= pCtrl.uBunLO;  //ub Low                    
        
        //wait for data to fill fifo
        while (1)
            { 
             i++;
             w= BCST_GEO_317 &0xF;
             if(w==0xf)
               break;
             if(i>500) 
               PaketStats.TOflag++;
            }
        
        //read all fpgas
        if (ubGetDataNew(0x000, &DAQ_FIFO1))    //get SDR_RD16_0 data
            pCtrl.gBunErr |= 0x01;   
        if( w=(BCST_GEO_317&BIT4))              //bit4 fpga 1 empty flag
            pCtrl.gBunErr |= BIT4;   

        
        if (ubGetDataNew(0x400,&DAQ_FIFO2))     //get SDR_RD16_1 data
            pCtrl.gBunErr |= 0x02;
        if (ubGetDataNew(0x800,&DAQ_FIFO3))     //get SDR_RD16_2 data
            pCtrl.gBunErr |= 0x04;        
        if (ubGetDataNew(0xc00,&DAQ_FIFO4))     //get SDR_RD16_3 data
            pCtrl.gBunErr |= 0x08;
            
        //update total word count header
        xWrds16384[pCtrl.uHdrIdx]  = pCtrl.pacSiz;  //4word Header Ubun wrdCnt
        xWrds16384[pCtrl.uHdrIdx+3]= pCtrl.gBunErr; //4word Header Ubun errors
        pCtrl.RetSzBytes += pCtrl.pacSiz;
        hHI_TP45;          
        }
    hLO_TP45;          

    //Check data size, Minimum ePHY packet size is 8 words, pad out if needed
    //Worst case Minimum size with no hit data is 4 words header only
    if (pCtrl.RetSzBytes < ePHY_PAC_MIN_Sz8)     //ePHY_PAC_MIN_Sz8=8 'ver_ePHY.h'
        {
        PaketStats.xmtWrdCntMin++;
        pCtrl.RetSzBytes = ePHY_PAC_MIN_Sz8;
        for(; pCtrl.g_idx < ePHY_PAC_MIN_Sz8; pCtrl.g_idx++) //fill to minimum size 
            xWrds16384[pCtrl.g_idx]=0x1234;
        }
    pCtrl.RetSzBytes<<=1;                 //convert word to byte count
     
    fill_daq_packet(pCtrl.RetSzBytes, (u_8Bit*)&xWrds16384); //ByteCount, data buffer ptr
            
    paKet.EMACTxBsy=1;              //flag tx as busy                  
    EMACTransmit(&hdkif_data[0], &PacKits[0]); //send data to MAC , two or more packet chained
    hHI_TP45;          
    return 0;
}         




#pragma optimize=speed
int ubGetDataNew(int offset, sPTR rAddr)
{
    u_16Bit UbunHI_Ram, UbunLO_Ram, err=0;  
    pCtrl.uBunSz= *rAddr;                   //get data word cont with limit
    pCtrl.uBunSz -= 3;                      //dont include header size ..Note must be unsigned integer for math
    UbunHI_Ram= *rAddr;                     //get data bun# high16
    UbunLO_Ram= *rAddr;                     //get data bun# LOW16
    
    //test uBunReq# to data in sdRam for match
    //
    if ((pCtrl.uBunHI==UbunHI_Ram) && (pCtrl.uBunLO==UbunLO_Ram))
        {
        //request matches ram data stored in buffer
        if(offset==0)   //1st fpga check, include the global header
            {
            pCtrl.uHdrIdx= pCtrl.g_idx;
            pCtrl.g_idx++;                          //4word Header skip wrd cnt until end
            xWrds16384[pCtrl.g_idx++]= UbunHI_Ram;  //4word Header Ubun High 'from sdRam'
            xWrds16384[pCtrl.g_idx++]= UbunLO_Ram;  //4word Header Ubun Low  'from sdRam'
            pCtrl.g_idx++;                          //4word Header skip Status until end
            pCtrl.pacSiz +=4;
            paKet.eSEND_TOT += 4;                   //add 4_word_header size
            }
        //get microbunch data if reqsiz>3
        if(pCtrl.uBunSz>0)
            {
            //test for bufoverflow
            if(pCtrl.uBunSz>0x100)                  //0x100 words, buffer must be 100
                {
                pCtrl.uBunSz= (10*24); //0x100;      //room for (24*10_Word samples)
                PaketStats.uBunOvrErrs++;
                pCtrl.gBunErr |=0x10;               //to much data
                }
            movStr16_NOICSRC((snvPTR)rAddr, (snvPTR)&xWrds16384[pCtrl.g_idx], pCtrl.uBunSz); 
            pCtrl.g_idx  += pCtrl.uBunSz;
            pCtrl.pacSiz += pCtrl.uBunSz;
            //uBunSz may be truncated if to large
            paKet.eSEND_TOT += pCtrl.uBunSz;        //add data words read
            }
        }
    else
        {
        //request does not match Ram Data store to buffer
        //
        if(offset==0)                               //1st fpga check, include the global header
            {
            paKet.eSEND_TOT += 4;   //error, no match on uBunch Req IDs, return 4_word_header
            pCtrl.uHdrIdx= pCtrl.g_idx;
            pCtrl.g_idx++;                          //4word Header skip wrd cnt until end
            xWrds16384[pCtrl.g_idx++]= pCtrl.uBunHI;//4word Header Ubun High 'requested'
            xWrds16384[pCtrl.g_idx++]= pCtrl.uBunLO;//4word Header Ubun Low  'requested'
            pCtrl.g_idx++;                          //4word Header skip Status until end
            pCtrl.pacSiz +=4;
            PaketStats.uBunReqErrs++;               //on going error total for display cmd'UB'
            }
        err++;                                      //error counter
        }
    return(err);
}

  








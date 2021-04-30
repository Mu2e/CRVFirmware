//******************************************************************
// @file sys_mu2e_RDX.c 
//  Fermilab Terry Kiper 2016-2020
//  mu2e controller board
//  RM48 Micro Controller
//  RM48 Hercules device supports little-endian [LE] format
//  Read/Write EMAC Register Transmit using EPHY Port
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


//EMAC structure
extern hdkif_t hdkif_data[MAX_EMAC_INSTANCE];   //defined in 'emac.c'
extern struct phyHeader phyHdr;
extern struct phyPACSTAT paKet;             //all daq packets, status/counter

//RDX control structure
extern struct blocksndWrd BinStRDX;
extern struct structFPGA_Trig_Cnts FPGA_Trig_Stat;
extern  uint32  volatile iFlag;

#pragma pack(4)                         //force 16bit boundary on all types
extern pbuf_t  PacLnkLst[];      //Link List of 21 Packets, holds max (21*1500= 31500 Bytes 7B0CH)

//dma structure
g_dmaCTRL g_dmaCtrl_RDX;                    //dma control packet config stack



//DMA Wiznet for uC cmd 'RDX',  for 'Fermilab Test Beam', may be obsolete
#define RDX_SZWRDS_INT      0x400       //Words or 0x800 bytes for total 2048dec 'sendRDX()'

#pragma pack(4)                         //force 16bit boundary on all types
uint16  RDX0_dmaDATA[RDX_SZWRDS_INT+4]; //fpga to uC DMA buffer, add padding 'sendRDX'
uint16  RDX1_dmaDATA[RDX_SZWRDS_INT+4]; //fpga to uC DMA buffer, add padding 'sendRDX'






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


  

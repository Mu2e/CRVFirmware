/** @file notification.c 
*   @brief User Notification Definition File
*   @date 07-July-2017
*   @version 04.07.00
*
*   This file  defines  empty  notification  routines to avoid
*   linker errors, Driver expects user to define the notification. 
*   The user needs to either remove this file and use their custom 
*   notification function or place their code sequence in this file 
*   between the provided USER CODE BEGIN and USER CODE END.
*
*/

/* 
* Copyright (C) 2009-2016 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* Include Files */

#include "esm.h"
#include "sys_selftest.h"
#include "adc.h"
#include "gio.h"
#include "sci.h"
#include "spi.h"
#include "het.h"
#include "rti.h"
#include "i2c.h"
#include "sys_dma.h"
#include "emac.h" 

/* USER CODE BEGIN (0) */

#include "ver_io.h" 

extern int esm_Ch;
extern int esm_Cnt;
extern uint32 genFlag, iFlag;
extern struct vB USB_Rec;

extern struct msTimers mStime;
extern struct uC_Store uC_FRAM;
extern struct HappyBusReg HappyBus;


//emac stuff
#include "emac.h"
#include "ver_ePHY.h"
#include "sys_mu2e.h"
#include "sys_mu2e_functions.h"

extern rxch_t *mu_rxch_int;
extern struct phyHeader phyHdr;

struct phyPACSTAT paKet;                //all daq packets, status/counter
extern struct phyPAC_ERRORS PaketStats; //storage for packet errors, counters




/* USER CODE END */
void esmGroup1Notification(uint32 channel)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (1) */
  //tek 
  esm_Ch= channel;
  esm_Cnt++;
  
  //esmTriggerErrorPinReset();            //same as esmREG->EKR = 0x5U;
  //esmClearStatus(0,channel);
  //esmActivateNormalOperation();
 
  //checkFlashECC();
  
    /* Routine to check operation of ECC logic inside CPU for accesses to program flash */
    volatile uint32 flashread = 0U;

    /* clear single-bit error flag */
        flashWREG->FEDACSTATUS = 0x2U;

        /* clear ESM flag */
        esmREG->SR1[0U] = 0x40U;

        /* Enable diagnostic mode and select diag mode 7 */
        flashWREG->FDIAGCTRL = 0x00050007U;

        /* Select ECC diagnostic mode, two bits of ECC to be corrupted */
        flashWREG->FPAROVR = 0x00005A03U;

        /* Set the trigger for the diagnostic mode */
        flashWREG->FDIAGCTRL |= 0x01000000U;

        /* read from flash location from mirrored memory map this will cause a data abort */
        flashread = flashBadECC2;

        /* Read FUNCERRADD register */
        flashread = flashWREG->FUNCERRADD;

        /* disable diagnostic mode */
        flashWREG->FDIAGCTRL = 0x000A0007U;
      
  
    /* clear ESM group1 channel 6 flag */
    //esmREG->SR1[0U] = 0x40U;
    // fmcECCcheck();    
  
    /* clear STC global status flags */
    //  stcREG->STCGSTAT = 0x40U;                
   

    /* clear self-check mode */
    //   stcREG->STCSCSCR = 0x05U;                
                
    /* The nERROR pin will become inactive once the LTC counter expires */
    // esmREG->EKR = 0x5U;
                 
/* USER CODE END */
}

/* USER CODE BEGIN (2) */
/* USER CODE END */
void esmGroup2Notification(uint32 channel)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (3) */
    while(1);    
/* USER CODE END */
}

/* USER CODE BEGIN (4) */
/* USER CODE END */
void memoryPort0TestFailNotification(uint32 groupSelect, uint32 dataSelect, uint32 address, uint32 data)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (5) */
/* USER CODE END */
}

/* USER CODE BEGIN (6) */
/* USER CODE END */
void memoryPort1TestFailNotification(uint32 groupSelect, uint32 dataSelect, uint32 address, uint32 data)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (7) */
/* USER CODE END */
}

/* USER CODE BEGIN (8) */
extern int adc_ms;
extern struct uSums u_SRAM;     //DwnLdSdRamCnt, DwnLdSdRamSum,stat,time

/* USER CODE END */
void rtiNotification(uint32 notification)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (9) */

    //heartbeat led and 1mS update code here
    static int m=0;
    if (USB_Rec.Cnt< 2)
        DSR_LO                              //LOW enables data flow

    //********************************************************************
    //**********  rtiNOTIFICATION_COMPARE0             *******************
    //**********  interrupt handler, 1 milli-Second    *******************
    //********************************************************************
    if (notification== rtiNOTIFICATION_COMPARE0) //RTI Compare Req 0 intr notification
        {
        if (mStime.g_wARP++>(ARP_TIME))     //wait 5 min, then flag ARP req
            {
            genFlag |= ARP_REQ;
            mStime.g_wARP=0;
            }
        
        //status for feb controller pgm flash
        if(u_SRAM.LdStatus_Tick)
            {
            FMDataSnd= u_SRAM.LdStatus_Tick;
            u_SRAM.LdStatus_Tick=0;
            }
        
        //keyboard input timer    
        mStime.g_wTicks++;                  //emac recption testing            

        //led timer    
        mStime.g_timeMs++;                  //sysTick 1mSec timer
        if(m==0)
            hHI_ucLED                       //hetREG1->DSET=  BIT22;
        else if(m==250)
            hLO_ucLED                       //hetREG1->DCLR=  BIT22;
        if(m==500)
            hHI_ucLED
        else if(m==750)
            hLO_ucLED
        if (m++==1000)
            {
            m=0;
            }

        //oneWireRead read timing, CMB device link readout time (approx 9.5 mSec per req)
        if (mStime.g_OneWireTime)           //check if timer non-zero
                {
                if (mStime.g_OneWireTime++ == 15)   //every 15 mSec
                    {
                    genFlag |= OneWireDatRdy;       //req next chan
                    }
                }
        
        //oneWireRead auto update Scan Interval Timer
        if (mStime.g_OneWireNewScanDelay)             //check if timer non-zero
            if (mStime.g_OneWireNewScanDelay++>10000) //10 seconds
                {
                mStime.g_OneWireNewScanDelay=0;       //stop scan delay check
                genFlag |= OneWireDatReq;             //start new scan
                }

        //Flag for data buffer update
        if (mStime.g_ADCms++> 10000)
            {
            //Flag for data buffer update
            genFlag |= (ADC_REFRESH+ ADC_TRIG);
            mStime.g_ADCms=0;
            }
          
        //monitor bais over voltage current status bit
        //set flag if trip is temporary set
        if (canOVCstat)
            {
            genFlag |= OVC_TRIP;                //trip auto clears after 80mS, latch trip status
            }

        /*
        //add if flash load valid  
        if (iFlag & WHATCHDOGENA)
            {
            if(mStime.WatchDog++> (2000*10))    //10Sec timeout for code/hardware lock
                {
                //add code to write fRAM stats
                FRAM_WR_ENABLE();               //WREN op-code issued prior to Wr_Op
                //wDogTimOut
                uC_FRAM.wDogTimOutCnt++;
                FRAM_WR(fPAGE_0400, (uint8*)&uC_FRAM.wDogTimOut, 2); //addr,data,cnt
                _disable_interrupt_();
                resetEntry();
                }
            }
        */      
        
        }  //end ofrtiNOTIFICATION_COMPARE0 section
    
    //********************************************************************
    //*************  rtiNOTIFICATION_COMPARE1          *******************
    //*************  interrupt handler for uDelay()    *******************
    //********************************************************************
    else if (notification== rtiNOTIFICATION_COMPARE1)
        {
         //Enable RTI Compare 1 interrupt notification
         rtiDisableNotification(rtiNOTIFICATION_COMPARE1);
         genFlag &= ~uTimeOut;                //clr timeout bit
        }
    
    //********************************************************************
    //***  rtiNOTIFICATION_COMPARE1                                     **
    //***  interrupt handler for uTimer(), 1.3uS code seqment RTI CMP2  **
    //********************************************************************
    else if (notification== rtiNOTIFICATION_COMPARE2)
        {
#define def_cmpCnt  (10*256)        //~10 cnts per uS (allow delay of 256uS)
#define SndSiz256   (256)           //max words to send per pass    
#define AddDly      (256*3)         //add extra holdoff delay for larger xfers, Controller needs more time 
        
        //tek 02-27-20 Setup for interrupt handler to return data pool data
        //tek 02-27-20 as of this date the block is (22+(4*38) 16bit words
        //tek 02-27-20 Takes 4*16uS to xfer 4*64 words (~250nS per word)
        //tek 02-27-20 Repeats as needed after a ~256uS upto ~750uS Delay
        //tek 02-27-20 Breaks xmits into segments, allow ubunch code to have more time  
                    
         int cmpCnt=def_cmpCnt;
         //Disable RTI Compare 2 interrupt notification
         rtiDisableNotification(rtiNOTIFICATION_COMPARE2);
         genFlag &= ~uTimeOut;      //clr timeout bit
         
        //set for a def_cmpCnt or def_cmpCnt*2 delayed re-entry
        //limit period to allow other code like packet xmits priority     
        //todo set interrupt priority lower than ethernet receive interrupts
        //hHI_TP45; //only for scope time testing
        //Send data on LVDS FM PORT if word count non zero?
        if (HappyBus.SndWrds)
            {
            int timeout=0;            
            if (HappyBus.SndWrds>AddDly)                //allow more time for large xfers
                cmpCnt=(def_cmpCnt*2);                  //controller needs time to read data before overrun              
            //send data block on LVDS PORT (256 words)
            for(int i=0; i<SndSiz256; i++)              //limit max 256 words per intr
                {
                FMDataSnd= *HappyBus.Src++;             //fpga xmits at 250nS per word
                if ((--HappyBus.SndWrds)==0)
                    break;
                }         
            rtiDisableNotification(rtiNOTIFICATION_COMPARE2);         
            rtiStopCounter(rtiCOUNTER_BLOCK1);
            while(!rtiResetCounter(rtiCOUNTER_BLOCK1))  //counter stopped before reset
              if (timeout++>1000) break;                //normally no delay here            
            rtiSetPeriod(rtiCOMPARE2, cmpCnt);
            rtiREG1->CMP[rtiCOMPARE2].UDCPx = cmpCnt;  //Added to the compare value on each compare match
            rtiREG1->CMP[rtiCOMPARE2].COMPx = cmpCnt;  //Compared with selected free running counter
            rtiStartCounter(rtiCOUNTER_BLOCK1);
            rtiEnableNotification(rtiNOTIFICATION_COMPARE2);
            //timeout via interrupt
            }
         else
            {
            rtiDisableNotification(rtiNOTIFICATION_COMPARE2);         
            rtiStopCounter(rtiCOUNTER_BLOCK1);
            }
        //hLO_TP45; //only for scope time testing
        }
    //interrupt handler, unused so far
    else if (notification== rtiNOTIFICATION_COMPARE3)
        {
         //Enable RTI Compare 3 interrupt notification
         rtiDisableNotification(rtiNOTIFICATION_COMPARE3);
        }
    
    
    /* USER CODE END */
}

/* USER CODE BEGIN (10) */
/* USER CODE END */
void adcNotification(adcBASE_t *adc, uint32 group)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (11) */
/* USER CODE END */
}

/* USER CODE BEGIN (12) */
/* USER CODE END */
void gioNotification(gioPORT_t *port, uint32 bit)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (19) */
/* USER CODE END */
}

/* USER CODE BEGIN (20) */
/* USER CODE END */
void i2cNotification(i2cBASE_t *i2c, uint32 flags)      
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (21) */
/* USER CODE END */
}

/* USER CODE BEGIN (22) */
/* USER CODE END */

void sciNotification(sciBASE_t *sci, uint32 flags)     
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (29) */
/* USER CODE END */
}

/* USER CODE BEGIN (30) */
/* USER CODE END */
void spiNotification(spiBASE_t *spi, uint32 flags)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (31) */
/* USER CODE END */
}

/* USER CODE BEGIN (32) */
/* USER CODE END */
void spiEndNotification(spiBASE_t *spi)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (33) */
/* USER CODE END */
}

/* USER CODE BEGIN (34) */
/* USER CODE END */

void pwmNotification(hetBASE_t * hetREG,uint32 pwm, uint32 notification)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (35) */
/* USER CODE END */
}

/* USER CODE BEGIN (36) */
/* USER CODE END */
void edgeNotification(hetBASE_t * hetREG,uint32 edge)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (37) */
/* USER CODE END */
}

/* USER CODE BEGIN (38) */
/* USER CODE END */
void hetNotification(hetBASE_t *het, uint32 offset)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (39) */
    hetREG1->INTENAC= 1<<8;
    genFlag &= ~hDelay;

/* USER CODE END */
}

/* USER CODE BEGIN (40) */
/* USER CODE END */


/* USER CODE BEGIN (43) */
/* USER CODE END */


/* USER CODE BEGIN (47) */
/* USER CODE END */


/* USER CODE BEGIN (50) */
/* USER CODE END */


/* USER CODE BEGIN (53) */
extern struct  blocksnd BinSt;
extern struct  blocksndWrd BinStRDX;

/* USER CODE END */

void dmaGroupANotification(dmaInterrupt_t inttype, uint32 channel)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (54) */
  
  //ch 0, Fpga to Mem
    if(channel==0)       
        {
        if(BinSt.gDMA_Fpga2Mem==1)
            BinSt.gDMA_Fpga2Mem=2;
        else if(BinSt.gDMA_Fpga2Mem==2)
            BinSt.gDMA_Fpga2Mem=3;
        }
    
    //ch 1, Mem to Wiznet
    if(channel==1)       
        {
        if(BinSt.gDMA_Mem2Wiz==1)
            BinSt.gDMA_Mem2Wiz=2;
        else if(BinSt.gDMA_Mem2Wiz==2)
            BinSt.gDMA_Mem2Wiz=3;
        }

      //ch 0, Fpga to Mem PHY xFER MODE
    if(channel==0)       
        {
        if(BinStRDX.gDMA_Fpga2Mem==1)
            BinStRDX.gDMA_Fpga2Mem=2;
        else if(BinStRDX.gDMA_Fpga2Mem==2)
            BinStRDX.gDMA_Fpga2Mem=3;
        }
    
    //ch 1, Mem to Wiznet PHY xFER MODE
    if(channel==1)       
        {
        if(BinStRDX.gDMA_Mem2Wiz==1)
            BinStRDX.gDMA_Mem2Wiz=2;
        else if(BinStRDX.gDMA_Mem2Wiz==2)
            BinStRDX.gDMA_Mem2Wiz=3;
        }

/* USER CODE END */
}
/* USER CODE BEGIN (55) */
/* USER CODE END */

/* USER CODE BEGIN (56) */
/* USER CODE END */
void emacTxNotification(hdkif_t *hdkif)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (57) */
    //g_EMACTxBsy=0;   //moved to EMACTxIntISR(void) in 'emac.c'
    //hHI_TP45;
   // hLO_TP45;

/* USER CODE END */
}

/* USER CODE BEGIN (58) */

struct emac_tx_int_status emac0TXS;


/* USER CODE END */
/* USER CODE BEGIN (58) */
/* USER CODE END */

void emacRxNotification(hdkif_t *hdkif)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (59) */
  rxch_t *rxch_int;
  volatile emac_rx_bd_t *curr_bd; // *last_bd;
  uint32 rxptr;   
  uint32 rxCnt;   
    if (paKet.EMACRxRdy==1)                 //rec still busy
        {
        if (phyHdr.PacNext==phyHdr.PacActive)  //buffers all full
            {
            PaketStats.eREC_BSY_DROP++;     //counter
            return;                         //skip it
            }
        }
  
    //The receive structure that holds data about a particular receive channel
    rxch_int = &(hdkif->rxchptr);

    //Get the buffer descriptors which contain the earliest filled data
    curr_bd = rxch_int->active_head;
    //last_bd = rxch_int->active_tail;

    //Process the descriptors as long as data is available
    //when the DMA is receiving data, SOP flag will be set

    //Start processing once the packet is loaded
    if((curr_bd->flags_pktlen & EMAC_BUF_DESC_OWNER)!= EMAC_BUF_DESC_OWNER ) 
        { 
        //this bd chain will be freed after processing
        rxch_int->free_head = curr_bd;
  
        //Start processing once the packet is loaded
        //this bd chain will be freed after processing
        //rxch_int->free_head = curr_bd;
        hHI_TP46;
        rxptr= rxch_int->free_head->bufptr;       //tek
        rxCnt= rxch_int->free_head->bufoff_len;   //tek
        if(rxCnt>ePayLdMaxAndHdr)                 //MsgBytMax==64
            rxCnt=ePayLdMaxAndHdr;
        phyHdr.PacBytCnt[phyHdr.PacNext]= rxCnt;
        //16bit moves, cnt is in word count Note: limit stored bytes to 'ePayLdMax'
        movStr16((sPTR)rxptr,(sPTR)&phyHdr.PacPayLd1024w[phyHdr.PacNext], rxCnt/2);
        hLO_TP46;
        
        phyHdr.PacNext++;
        if (phyHdr.PacNext>= PacBufs)           //out of buffer?
            phyHdr.PacNext=0;
        paKet.EMACRxRdy=1;                      //flag as rec'd packet
        }
}
/* USER CODE END */


/* USER CODE BEGIN (60) */
/* USER CODE END */

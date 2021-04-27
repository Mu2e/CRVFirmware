/** @file notification.c 
*   @brief User Notification Definition File
*   @date 05-Oct-2016
*   @version 04.06.00
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

/* USER CODE BEGIN (0) */
//  Fermilab Terry Kiper 2016-2021
//  @file notification.c
//  part interrupt handler


#include "ver_io.h" 
#include "mu2e_Ctrl_i2c.h"          //mu2e i2c link functions
#include "ZestETM1_SOCKET.h"
extern void nError();

extern int esm_Ch;
extern int esm_Cnt;
extern adcData_t adc_data[];
extern uint32 genFlag;
extern struct vB USB_Rec;
extern struct msTimers mStime;
extern struct sLVDS lvLnk;
extern struct HappyBusReg HappyBus;

/* USER CODE END */
void esmGroup1Notification(uint32 channel)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (1) */
  //tek 
  
  RG45LEDS(0xaaaa);
  esm_Ch= channel;
  esm_Cnt++;
  //nError();
  
  
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
extern int g_IntrCnt;
extern void SocketISRTEK(SOCKET s);
/* USER CODE END */
void rtiNotification(uint32 notification)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (9) */

    //heartbeat led and 1mS update code here
    static int m=0;
    static uint16_t rg45=0;
    if (USB_Rec.Cnt< 2)
          DSR_LO                            //LOW enables data flow
  
    if (notification==1)
        {
        mStime.g_timeMs++;                  //sysTick 1mSec timer used in mDelay
        mStime.gBusy_mSec++;
        mStime.g_wTicks++;
 
        //tek mod aug 2018, let daq packs control leds
        LEDs_OFF

        if(m==0)
            ucLED_HI                        //ucLED_HI
        else if(m==500)
            ucLED_LO                        //ucLED_LO
          
        if ((genFlag & hDelay)==0)          //read adc if uS delay not active
            {
            if (adc_ms++==500)
                {
                adcStartConversion(adcREG1,adcGROUP1);
                genFlag |= ADC_Trig;
                }
              else if (adc_ms++==502)       //allow plenty of time for samples
                {
                adc_ms=0;
                genFlag |= ADC_Rdy;         //main loop check, data should be rdy
                }
             }          
        if(++rg45== 50)                     //counts active ports, read one ch per 50 mSec
            {
            rg45=0;
            genFlag |= POE_CHECK_DUE;       //update POE port's status
            }
        
        
        //data pooling request and readout timers
        //at present its on a 30 second update
        if(lvLnk.PoolMode==1)
            {
            //if pool request enabled, req data, wait read data or get timeout
            lvLnk.PoolChkmSec++;              
            //Pool Data Request timers, req,get,getTimeout
            if(lvLnk.PoolChkmSec== ReqPoolTime) 
                genFlag &= ~PoolReqNow;            
            if (lvLnk.PoolChkmSec== ReqPoolTime) 
                genFlag |= PoolReqNow;              //cleared after returned data checked
            else if (lvLnk.PoolChkmSec== GetPoolTime)//return data pool data should be ready
                genFlag |= PoolReqGetData;
            
            if (lvLnk.PoolChkmSec>= GetPoolTime+500)//return data pool data should be ready
                {
                //Cycles complete normally in test above, this is abnormal cleanup
                genFlag &= ~PoolReqNow;             //off
                lvLnk.PoolChkmSec=0;                //reset seq timer
                }
             }
        else
            {
            //added to prevent cycle lookout of happy bus if 'Pool Data' didnt finish correctly
            lvLnk.PoolChkmSec=0;                 //timer reset 
            genFlag &= ~(PoolReqNow | PoolReqGetData); //clear all pool flags
            }
                  
        //msec intr here
        if (m++==1000)
            {
            //1 second counter routines
            m=0;
                        
            //active FEB Scan, used fpga status register now (every 4 sec)
            if (++lvLnk.IDChkSec== ID_RegTime)      //reads fpga to see if FEBs active
                {
                genFlag |= ID_ReqNow;               //main code loop check this
                lvLnk.IDChkSec=0;                   //repeat
                }
            }  //end 1 second counter routines
        
        if (mStime.g_SockIntrSec++ > 1000*10)       //10 Sec chk in case missed by Interrupt
            {
            genFlag |= ZEST_ETM1_INTR;              //set flag to check in main loop
            mStime.g_SockIntrSec=0;
            }                             
        } //end notification==1

    if (notification==2)
        {
         //Enable RTI Compare 1 interrupt notification
         rtiDisableNotification(rtiNOTIFICATION_COMPARE1);
         genFlag &= ~uTimeOut;                      //clr timeout bit
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
//Two source for interupts  
//Use below call to enable interrupts
//gioEnableNotification(gioPORTA, BIT5) ; //ETHERNET INTERRUPT ENABLE ETHERNET ZEST BRD 
//gioEnableNotification(gioPORTA, BIT4) ; //ETHERNET INTERRUPT ENABLE FPGA LOGIC

  //ETHER IRQ         (B5) GIOA5 INPUT
  SocketISRTEK(0);  

  //FPGA IRQ          (A6) GIOA4 INPUT
  //DAQ_Interrupt(under construction)
  
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
extern struct      blocksnd BinSt;
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
   
/* USER CODE END */
}
/* USER CODE BEGIN (55) */
/* USER CODE END */

/* USER CODE BEGIN (56) */
/* USER CODE END */

/* USER CODE BEGIN (58) */
/* USER CODE END */

/* USER CODE BEGIN (60) */
/* USER CODE END */

/** @file sys_main.c 
*   @brief Application main file
*   @date 05-Oct-2016
*   @version 04.06.00
*
*   This file contains an empty main function,
*   which can be used for the application.
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


/* USER CODE BEGIN (0) */
//  @file sys_main.c
//  Fermilab Terry Kiper 2016-2021
//
//  RM48 Code
//  RM48 Hercules device supports little-endian [LE] format
//
//  Initial Setup that calls main 'main_mu2e()'


/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */


/* USER CODE BEGIN (1) */
#include "esm.h"
#include "het.h"
#include "rti.h"
#include "spi.h"
#include "emif.h"
#include "sci.h"
#include "i2c.h"
#include "adc.h"
#include "hw_reg_access.h"
#include "ver_io.h"
#include "sys_core.h"
#include "reg_esm.h"
#include "gio.h"
#include "ver_io.h"

void main_mu2e(void);

/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
/*
#define memory mem with size = 4G;
#define region VECTORS = mem:[from 0x00000000 size 0x00000020];
#define region FLASH   = mem:[from 0x00000020 size 0x0017FFE0] | mem:[from 0x00180000 size 0x00180000];
#define region STACK   = mem:[from 0x0803EB00 size 0x00001500];
#define region RAM     = mem:[from 0x08000000 size 0x0003eb00];
#define block HEAP with size = 0x800, alignment = 8{ };
*/

/* USER CODE END */


int main(void)
{
/* USER CODE BEGIN (3) */
  
    /* Initialize Stack Pointers */
    //_coreInitStackPointer_();
  
    rtiInit();
    spiInit();
    i2cInit();
    //disable all i2c interrupts
    //i2cEnableNotification(i2cREG1, 0);          
    
    i2cSetOwnAdd(i2cREG1,own_add_w);

    //initializes the GIO module and set the GIO ports to the initial values
    gioInit();
    
    //Enable RTI Compare 0 interrupt notification
    rtiEnableNotification(rtiNOTIFICATION_COMPARE0);
    //Start RTI Counter Block 0
	rtiStartCounter(rtiCOUNTER_BLOCK0);
    //Returns setup param
    rtiGetPeriod(rtiCOUNTER_BLOCK0);

   //Enable RTI Compare 1 interrupt notification
    rtiEnableNotification(rtiNOTIFICATION_COMPARE1);

    /** - clear ESM error using Error Key Register  */
    //esmREG->EKR  = 0x5;

    //esmInit(); this is done by 'HalCodeGen' in sys_startup.c
    //esmEnableInterrupt(0);
    //_enable_interrupt_();
    
    sciInit();
   
    //initialize ADC
    //Group1 -> Channel 0 and 1                       
    //HW trigger trigger source as GIOB  Pin 0        
    adcInit();                 
    
    //EMIF Setup, 16 bit parallel port, 3 chip selects
    emif_ASYNC1Init();
    emif_ASYNC2Init();
    emif_ASYNC3Init();
    
    hetInit();
    
    //go to mu2e main code
    main_mu2e();
  
/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
/* USER CODE END */

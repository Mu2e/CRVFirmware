/** @file pom.c
*   @brief POM Driver Implementation File
*   @date 05-Oct-2016
*   @version 04.06.00
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



/* USER CODE BEGIN (0) */
/* USER CODE END */

#include "pom.h"

/* USER CODE BEGIN (1) */
/* USER CODE END */

REGION_CONFIG_t regn_conf_st[32] = {{0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},      
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES},
                                    {0x00000000U,0x00000000U,(uint32)SIZE_64BYTES}};



/** @fn void POM_Region_Config(REGION_CONFIG_t *Reg_Config_Ptr,REGION_t Region_Num)
*   @brief set the prog start address,overlay address,and size in the respective register for specified region number.
*   @param[in] Reg_Config_Ptr   - this will have the prog start address and overlay addresses and size which have to be set in the registers
*   @param[in] Region_Num  - Region number is used to access registers(for the specified region number)
*
*/
/* SourceId : POM_SourceId_001 */
/* DesignId : POM_DesignId_001 */
/* Requirements : HL_SR340 */
void POM_Region_Config(REGION_CONFIG_t *Reg_Config_Ptr,REGION_t Region_Num)
{
        pomREG->POMRGNCONF_ST[Region_Num].POMPROGSTART = Reg_Config_Ptr->Prog_Reg_Sta_Addr;
        pomREG->POMRGNCONF_ST[Region_Num].POMOVLSTART = Reg_Config_Ptr->Ovly_Reg_Sta_Addr;
        pomREG->POMRGNCONF_ST[Region_Num].POMREGSIZE = Reg_Config_Ptr->Reg_Size;
}


/** @fn void POM_Reset(void)
*   @brief Reset POM module.
*/
/* SourceId : POM_SourceId_002 */
/* DesignId : POM_DesignId_002 */
/* Requirements : HL_SR341 */
void POM_Reset(void)
{
        pomREG->POMGLBCTRL = 0x5U;  
}


/** @fn void void POM_Init(void)
*   @brief Initializes the POM driver
*
*   This function initializes the POM driver single function handles all the regions,timeouts are also handled.
*   POM_Enable() function must be called after POM_Init() function.
*/
/* SourceId : POM_SourceId_003 */
/* DesignId : POM_DesignId_003 */
/* Requirements : HL_SR342 */
void POM_Init(void)
{
        uint32 region;
        pomREG->POMGLBCTRL = INTERNAL_RAM | 0x00000005U;  
        for(region = 0U;region<1U;region++)
        {
                POM_Region_Config(&regn_conf_st[region],region);        
        }
}


/** @fn void POM_Enable(void)
*   @brief Enable POM module.
*/
/* SourceId : POM_SourceId_004 */
/* DesignId : POM_DesignId_004 */
/* Requirements : HL_SR343 */
void POM_Enable(void)
{
        pomREG->POMGLBCTRL = ((pomREG->POMGLBCTRL & 0xFFFFFFF0U) | (uint32)0x0000000AU);
}

/* USER CODE BEGIN (2) */
/* USER CODE END */

/** @fn void pomGetConfigValue(pom_config_reg_t *config_reg, config_value_type_t type)
*   @brief Get the initial or current values of the POM configuration registers
*
*	@param[in] *config_reg: pointer to the struct to which the initial or current 
*                           value of the configuration registers need to be stored
*	@param[in] type: 	whether initial or current value of the configuration registers need to be stored
*						- InitialValue: initial value of the configuration registers will be stored 
*                                       in the struct pointed by config_reg
*						- CurrentValue: initial value of the configuration registers will be stored 
*                                       in the struct pointed by config_reg
*
*   This function will copy the initial or current value (depending on the parameter 'type') 
*   of the configuration registers to the struct pointed by config_reg
*
*/
/* SourceId : POM_SourceId_005 */
/* DesignId : POM_DesignId_005 */
/* Requirements : HL_SR344 */
void pomGetConfigValue(pom_config_reg_t *config_reg, config_value_type_t type)
{
	if (type == InitialValue)
	{
		config_reg->CONFIG_POMGLBCTRL     = POM_POMGLBCTRL_CONFIGVALUE;     
		config_reg->CONFIG_POMPROGSTART0  = POM_POMPROGSTART0_CONFIGVALUE; 
		config_reg->CONFIG_POMOVLSTART0   = POM_POMOVLSTART0_CONFIGVALUE;
		config_reg->CONFIG_POMREGSIZE0    = POM_POMREGSIZE0_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART1  = POM_POMPROGSTART1_CONFIGVALUE; 
		config_reg->CONFIG_POMOVLSTART1   = POM_POMOVLSTART1_CONFIGVALUE;  
		config_reg->CONFIG_POMREGSIZE1    = POM_POMREGSIZE1_CONFIGVALUE;   
		config_reg->CONFIG_POMPROGSTART2  = POM_POMPROGSTART2_CONFIGVALUE; 
		config_reg->CONFIG_POMOVLSTART2   = POM_POMOVLSTART2_CONFIGVALUE;  
		config_reg->CONFIG_POMREGSIZE2    = POM_POMREGSIZE2_CONFIGVALUE;   
		config_reg->CONFIG_POMPROGSTART3  = POM_POMPROGSTART3_CONFIGVALUE; 
		config_reg->CONFIG_POMOVLSTART3   = POM_POMOVLSTART3_CONFIGVALUE;  
		config_reg->CONFIG_POMREGSIZE3    = POM_POMREGSIZE3_CONFIGVALUE;   
		config_reg->CONFIG_POMPROGSTART4  = POM_POMPROGSTART4_CONFIGVALUE; 
		config_reg->CONFIG_POMOVLSTART4   = POM_POMOVLSTART4_CONFIGVALUE;  
		config_reg->CONFIG_POMREGSIZE4    = POM_POMREGSIZE4_CONFIGVALUE;   
		config_reg->CONFIG_POMPROGSTART5  = POM_POMPROGSTART5_CONFIGVALUE; 
		config_reg->CONFIG_POMOVLSTART5   = POM_POMOVLSTART5_CONFIGVALUE;  
		config_reg->CONFIG_POMREGSIZE5    = POM_POMREGSIZE5_CONFIGVALUE;   
		config_reg->CONFIG_POMPROGSTART6  = POM_POMPROGSTART6_CONFIGVALUE; 
		config_reg->CONFIG_POMOVLSTART6   = POM_POMOVLSTART6_CONFIGVALUE;  
		config_reg->CONFIG_POMREGSIZE6    = POM_POMREGSIZE6_CONFIGVALUE;   
		config_reg->CONFIG_POMPROGSTART7  = POM_POMPROGSTART7_CONFIGVALUE; 
		config_reg->CONFIG_POMOVLSTART7   = POM_POMOVLSTART7_CONFIGVALUE;  
		config_reg->CONFIG_POMREGSIZE7    = POM_POMREGSIZE7_CONFIGVALUE;   
		config_reg->CONFIG_POMPROGSTART8  = POM_POMPROGSTART8_CONFIGVALUE; 
		config_reg->CONFIG_POMOVLSTART8   = POM_POMOVLSTART8_CONFIGVALUE;  
		config_reg->CONFIG_POMREGSIZE8    = POM_POMREGSIZE8_CONFIGVALUE;   
		config_reg->CONFIG_POMPROGSTART9  = POM_POMPROGSTART9_CONFIGVALUE; 
		config_reg->CONFIG_POMOVLSTART9   = POM_POMOVLSTART9_CONFIGVALUE;  
		config_reg->CONFIG_POMREGSIZE9    = POM_POMREGSIZE9_CONFIGVALUE;   
		config_reg->CONFIG_POMPROGSTART10 = POM_POMPROGSTART10_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART10  = POM_POMOVLSTART10_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE10   = POM_POMREGSIZE10_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART11 = POM_POMPROGSTART11_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART11  = POM_POMOVLSTART11_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE11   = POM_POMREGSIZE11_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART12 = POM_POMPROGSTART12_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART12  = POM_POMOVLSTART12_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE12   = POM_POMREGSIZE12_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART13 = POM_POMPROGSTART13_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART13  = POM_POMOVLSTART13_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE13   = POM_POMREGSIZE13_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART14 = POM_POMPROGSTART14_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART14  = POM_POMOVLSTART14_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE14   = POM_POMREGSIZE14_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART15 = POM_POMPROGSTART15_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART15  = POM_POMOVLSTART15_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE15   = POM_POMREGSIZE15_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART16 = POM_POMPROGSTART16_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART16  = POM_POMOVLSTART16_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE16   = POM_POMREGSIZE16_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART17 = POM_POMPROGSTART17_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART17  = POM_POMOVLSTART17_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE17   = POM_POMREGSIZE17_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART18 = POM_POMPROGSTART18_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART18  = POM_POMOVLSTART18_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE18   = POM_POMREGSIZE18_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART19 = POM_POMPROGSTART19_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART19  = POM_POMOVLSTART19_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE19   = POM_POMREGSIZE19_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART20 = POM_POMPROGSTART20_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART20  = POM_POMOVLSTART20_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE20   = POM_POMREGSIZE20_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART21 = POM_POMPROGSTART21_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART21  = POM_POMOVLSTART21_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE21   = POM_POMREGSIZE21_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART22 = POM_POMPROGSTART22_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART22  = POM_POMOVLSTART22_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE22   = POM_POMREGSIZE22_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART23 = POM_POMPROGSTART23_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART23  = POM_POMOVLSTART23_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE23   = POM_POMREGSIZE23_CONFIGVALUE; 
		config_reg->CONFIG_POMPROGSTART24 = POM_POMPROGSTART24_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART24  = POM_POMOVLSTART24_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE24   = POM_POMREGSIZE24_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART25 = POM_POMPROGSTART25_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART25  = POM_POMOVLSTART25_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE25   = POM_POMREGSIZE25_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART26 = POM_POMPROGSTART26_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART26  = POM_POMOVLSTART26_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE26   = POM_POMREGSIZE26_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART27 = POM_POMPROGSTART27_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART27  = POM_POMOVLSTART27_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE27   = POM_POMREGSIZE27_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART28 = POM_POMPROGSTART28_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART28  = POM_POMOVLSTART28_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE28   = POM_POMREGSIZE28_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART29 = POM_POMPROGSTART29_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART29  = POM_POMOVLSTART29_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE29   = POM_POMREGSIZE29_CONFIGVALUE; 
		config_reg->CONFIG_POMPROGSTART30 = POM_POMPROGSTART30_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART30  = POM_POMOVLSTART30_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE30   = POM_POMREGSIZE30_CONFIGVALUE;  
		config_reg->CONFIG_POMPROGSTART31 = POM_POMPROGSTART31_CONFIGVALUE;
		config_reg->CONFIG_POMOVLSTART31  = POM_POMOVLSTART31_CONFIGVALUE; 
		config_reg->CONFIG_POMREGSIZE31   = POM_POMREGSIZE31_CONFIGVALUE;  
	}                                    
	else                                 
	{                                    
	/*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
		config_reg->CONFIG_POMGLBCTRL     = pomREG->POMGLBCTRL;
		config_reg->CONFIG_POMPROGSTART0  = pomREG->POMRGNCONF_ST[0].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART0   = pomREG->POMRGNCONF_ST[0].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE0    = pomREG->POMRGNCONF_ST[0].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART1  = pomREG->POMRGNCONF_ST[1].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART1   = pomREG->POMRGNCONF_ST[1].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE1    = pomREG->POMRGNCONF_ST[1].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART2  = pomREG->POMRGNCONF_ST[2].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART2   = pomREG->POMRGNCONF_ST[2].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE2    = pomREG->POMRGNCONF_ST[2].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART3  = pomREG->POMRGNCONF_ST[3].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART3   = pomREG->POMRGNCONF_ST[3].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE3    = pomREG->POMRGNCONF_ST[3].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART4  = pomREG->POMRGNCONF_ST[4].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART4   = pomREG->POMRGNCONF_ST[4].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE4    = pomREG->POMRGNCONF_ST[4].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART5  = pomREG->POMRGNCONF_ST[5].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART5   = pomREG->POMRGNCONF_ST[5].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE5    = pomREG->POMRGNCONF_ST[5].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART6  = pomREG->POMRGNCONF_ST[6].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART6   = pomREG->POMRGNCONF_ST[6].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE6    = pomREG->POMRGNCONF_ST[6].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART7  = pomREG->POMRGNCONF_ST[7].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART7   = pomREG->POMRGNCONF_ST[7].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE7    = pomREG->POMRGNCONF_ST[7].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART8  = pomREG->POMRGNCONF_ST[8].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART8   = pomREG->POMRGNCONF_ST[8].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE8    = pomREG->POMRGNCONF_ST[8].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART9  = pomREG->POMRGNCONF_ST[9].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART9   = pomREG->POMRGNCONF_ST[9].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE9    = pomREG->POMRGNCONF_ST[9].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART10 = pomREG->POMRGNCONF_ST[10].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART10  = pomREG->POMRGNCONF_ST[10].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE10   = pomREG->POMRGNCONF_ST[10].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART11 = pomREG->POMRGNCONF_ST[11].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART11  = pomREG->POMRGNCONF_ST[11].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE11   = pomREG->POMRGNCONF_ST[11].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART12 = pomREG->POMRGNCONF_ST[12].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART12  = pomREG->POMRGNCONF_ST[12].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE12   = pomREG->POMRGNCONF_ST[12].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART13 = pomREG->POMRGNCONF_ST[13].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART13  = pomREG->POMRGNCONF_ST[13].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE13   = pomREG->POMRGNCONF_ST[13].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART14 = pomREG->POMRGNCONF_ST[14].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART14  = pomREG->POMRGNCONF_ST[14].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE14   = pomREG->POMRGNCONF_ST[14].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART15 = pomREG->POMRGNCONF_ST[15].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART15  = pomREG->POMRGNCONF_ST[15].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE15   = pomREG->POMRGNCONF_ST[15].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART16 = pomREG->POMRGNCONF_ST[16].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART16  = pomREG->POMRGNCONF_ST[16].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE16   = pomREG->POMRGNCONF_ST[16].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART17 = pomREG->POMRGNCONF_ST[17].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART17  = pomREG->POMRGNCONF_ST[17].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE17   = pomREG->POMRGNCONF_ST[17].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART18 = pomREG->POMRGNCONF_ST[18].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART18  = pomREG->POMRGNCONF_ST[18].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE18   = pomREG->POMRGNCONF_ST[18].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART19 = pomREG->POMRGNCONF_ST[19].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART19  = pomREG->POMRGNCONF_ST[19].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE19   = pomREG->POMRGNCONF_ST[19].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART20 = pomREG->POMRGNCONF_ST[20].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART20  = pomREG->POMRGNCONF_ST[20].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE20   = pomREG->POMRGNCONF_ST[20].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART21 = pomREG->POMRGNCONF_ST[20].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART21  = pomREG->POMRGNCONF_ST[21].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE21   = pomREG->POMRGNCONF_ST[21].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART22 = pomREG->POMRGNCONF_ST[21].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART22  = pomREG->POMRGNCONF_ST[22].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE22   = pomREG->POMRGNCONF_ST[22].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART23 = pomREG->POMRGNCONF_ST[22].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART23  = pomREG->POMRGNCONF_ST[23].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE23   = pomREG->POMRGNCONF_ST[23].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART24 = pomREG->POMRGNCONF_ST[23].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART24  = pomREG->POMRGNCONF_ST[24].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE24   = pomREG->POMRGNCONF_ST[24].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART25 = pomREG->POMRGNCONF_ST[24].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART25  = pomREG->POMRGNCONF_ST[25].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE25   = pomREG->POMRGNCONF_ST[25].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART26 = pomREG->POMRGNCONF_ST[25].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART26  = pomREG->POMRGNCONF_ST[26].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE26   = pomREG->POMRGNCONF_ST[26].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART27 = pomREG->POMRGNCONF_ST[26].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART27  = pomREG->POMRGNCONF_ST[27].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE27   = pomREG->POMRGNCONF_ST[27].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART28 = pomREG->POMRGNCONF_ST[27].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART28  = pomREG->POMRGNCONF_ST[28].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE28   = pomREG->POMRGNCONF_ST[28].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART29 = pomREG->POMRGNCONF_ST[28].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART29  = pomREG->POMRGNCONF_ST[29].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE29   = pomREG->POMRGNCONF_ST[29].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART30 = pomREG->POMRGNCONF_ST[30].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART30  = pomREG->POMRGNCONF_ST[30].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE30   = pomREG->POMRGNCONF_ST[30].POMREGSIZE;
		config_reg->CONFIG_POMPROGSTART31 = pomREG->POMRGNCONF_ST[31].POMPROGSTART;
		config_reg->CONFIG_POMOVLSTART31  = pomREG->POMRGNCONF_ST[31].POMOVLSTART;
		config_reg->CONFIG_POMREGSIZE31   = pomREG->POMRGNCONF_ST[31].POMREGSIZE;

	}
}



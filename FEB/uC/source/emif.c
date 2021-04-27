/** @file emif.c
*   @brief emif Driver Implementation File
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

#include "emif.h"

/* USER CODE BEGIN (1) */
/* USER CODE END */


/** @fn void emif_ASYNC1Init(void)
*   @brief Initializes the emif Driver for ASYNC memories
*
*   This function initializes the emif driver for Asynchronous memories like Nor and Nand Flashes,Asynchronous RAM.
*/
/* SourceId : EMIF_SourceId_002 */
/* DesignId : EMIF_DesignId_002 */
/* Requirements: HL_SR335 */
void emif_ASYNC1Init(void)
{
/* USER CODE BEGIN (4) */
/* USER CODE END */
    emifREG->CE2CFG = 0x00000000U;
    emifREG->CE2CFG = (uint32)((uint32)0U << 31U)|
                      (uint32)((uint32)0U << 30U)|
                      (uint32)((uint32)0U << 26U)|
                      (uint32)((uint32)4U << 20U)|
                      (uint32)((uint32)0U << 17U)|
                      (uint32)((uint32)0U << 13U)|
                      (uint32)((uint32)4U << 7U)|
                      (uint32)((uint32)0U << 4U)|
                      (uint32)((uint32)0U << 2U)|
                      (uint32)((uint32)emif_16_bit_port);

    emifREG->AWCC   = (emifREG->AWCC & 0xC0FF0000U)|
                      (uint32)((uint32)emif_pin_high << 29U)|
                      (uint32)((uint32)emif_pin_low << 28U)|
                      (uint32)((uint32)emif_wait_pin0 << 16U)|
                      (uint32)((uint32)0U);

    emifREG->PMCR   = (emifREG->PMCR & 0xFFFFFF00U)|
                      (uint32)((uint32)0U << 2U)|
                      (uint32)((uint32)emif_8_words << 1U)|
                      (uint32)((uint32)0U);
/* USER CODE BEGIN (5) */
/* USER CODE END */
}

/** @fn void emif_ASYNC2Init(void)
*   @brief Initializes the emif Driver for ASYNC memories
*
*   This function initializes the emif driver for Asynchronous memories like Nor and Nand Flashes,Asynchronous RAM.
*/
/* SourceId : EMIF_SourceId_003 */
/* DesignId : EMIF_DesignId_002 */
/* Requirements: HL_SR335 */
void emif_ASYNC2Init(void)
{
/* USER CODE BEGIN (6) */
/* USER CODE END */

    emifREG->CE3CFG = 0x00000000U;
    emifREG->CE3CFG = (uint32)((uint32)0U << 31U)|
                      (uint32)((uint32)0U << 30U)|
                      (uint32)((uint32)0U << 26U)|
                      (uint32)((uint32)7U << 20U)|
                      (uint32)((uint32)0U << 17U)|
                      (uint32)((uint32)0U << 13U)|
                      (uint32)((uint32)7U << 7U)|
                      (uint32)((uint32)0U << 4U)|
                      (uint32)((uint32)0U << 2U)|
                      (uint32)((uint32)emif_16_bit_port);

    emifREG->AWCC   = (emifREG->AWCC & 0xC0FF0000U)|
                      (uint32)((uint32)emif_pin_high << 29U)|
                      (uint32)((uint32)emif_pin_low << 28U)|
                      (uint32)((uint32)emif_wait_pin0 << 18U)|
                      (uint32)((uint32)0U);

    emifREG->PMCR   = (emifREG->PMCR & 0xFFFF00FFU)|
                      (uint32)((uint32)0U << 10U)|
                      (uint32)((uint32)emif_8_words << 9U)|
                      (uint32)((uint32)0U << 8U);
/* USER CODE BEGIN (7) */
/* USER CODE END */

}


/** @fn void emif_ASYNC3Init(void)
*   @brief Initializes the emif Driver for ASYNC memories
*
*   This function initializes the emif driver for Asynchronous memories like Nor and Nand Flashes,Asynchronous RAM.
*/
/* SourceId : EMIF_SourceId_004 */
/* DesignId : EMIF_DesignId_002 */
/* Requirements: HL_SR335 */
void emif_ASYNC3Init(void)
{
/* USER CODE BEGIN (8) */
/* USER CODE END */

    emifREG->CE4CFG = 0x00000000U;
    emifREG->CE4CFG = (uint32)((uint32)0U << 31U)|
                      (uint32)((uint32)0U << 30U)|
                      (uint32)((uint32)0U << 26U)|
                      (uint32)((uint32)4U << 20U)|
                      (uint32)((uint32)0U << 17U)|
                      (uint32)((uint32)0U << 13U)|
                      (uint32)((uint32)10U << 7U)|
                      (uint32)((uint32)0U << 4U)|
                      (uint32)((uint32)0U << 2U)|
                      (uint32)((uint32)emif_16_bit_port);

    emifREG->AWCC   = (emifREG->AWCC & 0xC0FF0000U)|
                      (uint32)((uint32)emif_pin_high << 29U)|
                      (uint32)((uint32)emif_pin_low << 28U)|
                      (uint32)((uint32)emif_wait_pin0 << 20U)|
                      (uint32)((uint32)0U);

    emifREG->PMCR   = (emifREG->PMCR & 0xFF00FFFFU) |
                      (uint32)((uint32)0U << 18U)|
                      (uint32)((uint32)emif_8_words << 17U)|
                      (uint32)((uint32)0U << 16U);

/* USER CODE BEGIN (9) */
/* USER CODE END */
}

/* USER CODE BEGIN (10) */
/* USER CODE END */

/** @fn void emifGetConfigValue(emif_config_reg_t *config_reg, config_value_type_t type)
*   @brief Get the initial or current values of the EMIF configuration registers
*
*   @param[in] *config_reg: pointer to the struct to which the initial or current
*                           value of the configuration registers need to be stored
*   @param[in] type:    whether initial or current value of the configuration registers need to be stored
*                       - InitialValue: initial value of the configuration registers will be stored
*                                       in the struct pointed by config_reg
*                       - CurrentValue: initial value of the configuration registers will be stored
*                                       in the struct pointed by config_reg
*
*   This function will copy the initial or current value (depending on the parameter 'type')
*   of the configuration registers to the struct pointed by config_reg
*
*/
/* SourceId : EMIF_SourceId_005 */
/* DesignId : EMIF_DesignId_003 */
/* Requirements: HL_SR336 */
void emifGetConfigValue(emif_config_reg_t *config_reg, config_value_type_t type)
{
    if (type == InitialValue)
    {
        config_reg->CONFIG_AWCC    = EMIF_AWCC_CONFIGVALUE;
        config_reg->CONFIG_SDCR    = EMIF_SDCR_CONFIGVALUE;
        config_reg->CONFIG_SDRCR   = EMIF_SDRCR_CONFIGVALUE;
        config_reg->CONFIG_CE2CFG  = EMIF_CE2CFG_CONFIGVALUE;
        config_reg->CONFIG_CE3CFG  = EMIF_CE3CFG_CONFIGVALUE;
        config_reg->CONFIG_CE4CFG  = EMIF_CE4CFG_CONFIGVALUE;
        config_reg->CONFIG_CE5CFG  = EMIF_CE5CFG_CONFIGVALUE;
        config_reg->CONFIG_SDTIMR  = EMIF_SDTIMR_CONFIGVALUE;
        config_reg->CONFIG_SDSRETR = EMIF_SDSRETR_CONFIGVALUE;
        config_reg->CONFIG_INTMSK  = EMIF_INTMSK_CONFIGVALUE;
        config_reg->CONFIG_PMCR    = EMIF_PMCR_CONFIGVALUE;
    }
    else
    {
    /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
        config_reg->CONFIG_AWCC     = emifREG->AWCC;
        config_reg->CONFIG_SDCR     = emifREG->SDCR;
        config_reg->CONFIG_SDRCR    = emifREG->SDRCR ;
        config_reg->CONFIG_CE2CFG   = emifREG->CE2CFG;
        config_reg->CONFIG_CE3CFG   = emifREG->CE3CFG;
        config_reg->CONFIG_CE4CFG   = emifREG->CE4CFG;
        config_reg->CONFIG_CE5CFG   = emifREG->CE5CFG;
        config_reg->CONFIG_SDTIMR   = emifREG->SDTIMR;
        config_reg->CONFIG_SDSRETR  = emifREG->SDSRETR;
        config_reg->CONFIG_INTMSK   = emifREG->INTMSK;
        config_reg->CONFIG_PMCR     = emifREG->PMCR;
    }
}



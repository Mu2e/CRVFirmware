//**************************************************************
//  @file sys_mu2e_Status.c
//  Fermilab Terry Kiper 2016-2020
//  RM48 Code
//  RM48 Hercules device supports little-endian [LE] format
//
//  Format status displays blocks
//**************************************************************

#include "sys_common.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "het.h"
#include "hw_reg_access.h"
#include "sys_dma.h"
#include "adc.h"
#include "ver_io.h"

#include "sys_mu2e_misc.h"


//extern structures
extern struct   Controller_tdcHdr   Cntrl_tdcHDR, tHDR_STORE;
extern struct   structFPGA_Trig_Cnts FPGA_Trig_Stat;
extern volatile struct msTimers mStime; 
extern struct   uC_Store uC_FRAM;     
extern g_dmaCTRL g_dmaCTRLPKT1; 

//extern buffers, vars
extern const float adcScale[];
extern adcData_t   adc_data[17];    //16 channels plus one temp/work channel [17]
extern char     Buf1500[1500];
extern char     tBuf[400];

extern uint32   volatile genFlag;
extern int      oneWireErr;         //oneWireData bad readings
extern int      CmbActiveCnt[];



//Telnet Version
//format Trig diagnosic header0
void headerStatus(int prt, int clr)
{
    u_32Bit d32,trigs, power;
    d32= (tHDR_STORE.tSpilWrdCntL<<16) + tHDR_STORE.tSpilWrdCntH; //HDR data,big Endian reverse here
    sprintf(tBuf,"Addr  8 Word Header  : RDB Cmd Updates Header\r\n");
    strcat(Buf1500, tBuf);

    //----paragraph marker start
    sprintf(tBuf,"(06A) Spill WrdCnt+8 : %-8X  (%d)DEC\r\n", d32,d32);
    strcat(Buf1500, tBuf);
    
    trigs = (tHDR_STORE.tSpilTrgCntL<<16) + tHDR_STORE.tSpilTrgCntH;  //HDR data,big Endian reverse here
    sprintf(tBuf,"(066) Spill Trig Cnt : %-8X  (%d)\r\n" ,trigs, trigs);
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"(068) Spill Cycle    : %-4X      (%d)\r\n",tHDR_STORE.tSpilCyc, tHDR_STORE.tSpilCyc);
    strcat(Buf1500, tBuf);

    sprintf(tBuf,"(n21) Chan Mask Reg  : %-4X\r\n",tHDR_STORE.tMask);
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"( uC) Board ID       : %-4X\r\n",tHDR_STORE.tID);
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"(076) Spill Status   : %-4X      4=GateOpen, 2=SpillEnd, 1=Busy\r\n",tHDR_STORE.tStatus);
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"---------------------\r\n");
    strcat(Buf1500, tBuf);

    //update registers
    FPGA_Trig_Cnts_Update();
    //spill word counters
    sPTR sptr= &CSR0;
    for(int i=0,k=0x6A; i<4; i++,k+=fOffset, sptr+=fOffset)
        {
        if ((*sptr&3)==3)
            sprintf(tBuf,"(%03X) Wrd Cnt FPGA%1d  : %-8X  CSR0=%04X  CMBs_Connected=%d  (AFEs OFF)\r\n",k,i, FPGA_Trig_Stat.WrdCnt[i], *sptr,CmbActiveCnt[i]);
        else 
          sprintf(tBuf,  "(%03X) Wrd Cnt FPGA%1d  : %-8X  CSR0=%04X  CMBs_Connected=%d\r\n",k,i, FPGA_Trig_Stat.WrdCnt[i], *sptr, CmbActiveCnt[i]);
        strcat(Buf1500, tBuf);
        }
    sprintf(tBuf,"---------------------\r\n");
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"( uC) Total Word Cnt : %X\r\n",FPGA_Trig_Stat.TotWrdCnt);
    strcat(Buf1500, tBuf);
   
    sprintf(tBuf,"( uC) LastUserReqSiz : %X\r\n",FPGA_Trig_Stat.RDBreqSiz);
    strcat(Buf1500, tBuf);

    sprintf(tBuf,"( uC) UserReq OvrRun : %X\r\n",FPGA_Trig_Stat.OverRun);
    strcat(Buf1500, tBuf);
    if (clr)
      FPGA_Trig_Stat.OverRun=0;

    sprintf(tBuf,"( uC) FPGA1of4 OvrRun: %-4X      BIT(3,0)HI WrdCnt>0x%08X, TotErrs=%d\r\n",FPGA_Trig_Stat.OverMax, MAX_WORD_PER_FPGA, FPGA_Trig_Stat.OverMaxCounter);
    strcat(Buf1500, tBuf);
    
    sprintf(tBuf,"(305) Sample Length  : %-4X      (%d) ADC Samples Per Trig (254 Max)\r\n",sSampleLen, sSampleLen);
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"---------------------\r\n");
    strcat(Buf1500, tBuf);
    power = CSR0 & (BIT1+BIT0);
    activeChannels(&sIN_MASK0, trigs, 0, prt, power);
    power = CSR1 & (BIT1+BIT0);
    activeChannels(&sIN_MASK1, trigs, 1, prt, power);
    power = CSR2 & (BIT1+BIT0);
    activeChannels(&sIN_MASK2, trigs, 2, prt, power);
    power = CSR3 & (BIT1+BIT0);
    activeChannels(&sIN_MASK3, trigs, 3, prt, power);
    sprintf(tBuf,"---------------------\r\n\r");
    strcat(Buf1500, tBuf);
}   



//format Trig diagnosic header1
//copy of header1WEB with line ending \r\n mode
void header1TTY(int prt)
{
    u_32Bit d32;
    //add code to show setup mode register and uC_FRAM regs
    sprintf(tBuf,"-------- fpga regs --------\r\n");
    strcat(Buf1500, tBuf);

    sprintf(tBuf, "(072) Time Stamp     : %X%-4X\r\n", sTIMSTAMPHI,sTIMSTAMPLO);
    strcat(Buf1500, tBuf);

    sprintf(tBuf, "(074) Pulse Trg Dly  : %X\r\n", sPULTRIGdly);
    strcat(Buf1500, tBuf);
    
    sprintf(tBuf, "(075) Spill Error    : %X\r\n", sSPILLERROR);
    strcat(Buf1500, tBuf);

    sprintf(tBuf, "(300) FlashGate CSR  : %X\r\n", sFlashGate);
    strcat(Buf1500, tBuf);

    sprintf(tBuf, "(301) FlshGateOnTime : %-4X (%-5.2fnS)\r\n", sFlashOnTim, (float)sFlashOnTim*6.28);
    strcat(Buf1500, tBuf);

    sprintf(tBuf, "(302) FlshGateOffTime: %-4X (%-5.2fnS)\r\n", sFlashOffTim, (float)sFlashOffTim*6.28);
    strcat(Buf1500, tBuf);
                    
    sprintf(tBuf, "(303) Trigger Cntrl  : %X\r\n", sTrigCntrl);
    strcat(Buf1500, tBuf);

    d32= sPipeLinDly;    //pipeline delay (0==256) 12.56nS per cnt
    if (d32==0)
      d32=256;
    sprintf(tBuf, "(304) PipeLine Dly   : %-4X (%-5.2fnS)\r\n", d32, (float)d32*12.56);
    strcat(Buf1500, tBuf);
                            
    sprintf(tBuf, "(305) Sample Length  : %X\r\n", sSampleLen);
    strcat(Buf1500, tBuf);

    d32 = (sTstPulFreqH<<16) +sTstPulFreqL;   //big Endian reverse here
    sprintf(tBuf, "(306) Test Pulser    : %X\r\n", d32);
    strcat(Buf1500, tBuf);

    sprintf(tBuf, "(308) TstPulsLen(sec): %X\r\n", sTstPulSpDur);
    strcat(Buf1500, tBuf);

    sprintf(tBuf, "(309) TstPulsGap(sec): %X\r\n", sTstPulIntSp);
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"--------- uC regs ---------\r\n");
    strcat(Buf1500, tBuf);
                    
    sprintf(tBuf,   "( uC) MuxChan        : %d\r\n", uC_FRAM.pgaMuxCh);
    strcat(Buf1500, tBuf);

    if (uC_FRAM.TrgSrcCh==1)
      sprintf(tBuf, "( uC) Trig Src LEMO  : %d\r\n", uC_FRAM.TrgSrcCh);
    else
      sprintf(tBuf, "( uC) Trig Src RJ45  : %d\r\n", uC_FRAM.TrgSrcCh);
    strcat(Buf1500, tBuf);

    if (uC_FRAM.LinkDir==1)
      sprintf(tBuf, "( uC) Link Dir XMIT  : %d\r\n", uC_FRAM.LinkDir);
    else
      sprintf(tBuf, "( uC) Link Dir REC   : %d\r\n", uC_FRAM.LinkDir);
    strcat(Buf1500, tBuf);

    //get ADC bias readings
    static float flt;
    for (int i=8,j=0; i<16; i++,j++)
        {
        flt= adc_data[i].value * adcScale[i];
        sprintf(tBuf, "( uC) BiasVolt Ch%1d   : %5.2f\r\n",j,flt);
        strcat(Buf1500, tBuf);
        }   
    //Flag for ADC data buffer update, 
    genFlag |= (ADC_REFRESH+ ADC_TRIG);
}



//Check FPGA Mask Register sIN_MASK0 address 0x21
//todo: this may not be accurate depending of fpga code, may need to read all fgpa stat reg 0x21
int activeChannels(sPTR ptr, int trigs, int fpga1of4, int prt, int power)
{
    int chans=0;
    u_32Bit dd32;
    if (*ptr & BIT0)
       chans +=4;
    if (*ptr & BIT1)
       chans += 4;
    if (*ptr & BIT2)
       chans += 4;
    if (*ptr & BIT3)
       chans += 4;
    dd32=  sSampleLen * chans;      //read sample len register fpga0
    dd32 = (dd32 * trigs);
    dd32+= (8* trigs); 
    if (power==3)
      sprintf(tBuf,"FPGA%d Word Count Calc: 0         PwrDown\r\n",fpga1of4);
    else if (chans>0)
      sprintf(tBuf,"FPGA%d Word Count Calc: %-8X  (%dsLEN)*(%2dWrds)*(%dTRGs) + HDRs=(%dTRGs)*8\r\n",  fpga1of4,  dd32, sSampleLen, chans, trigs, trigs );
    else
      sprintf(tBuf,"FPGA%d Word Count Calc: 0         MaskBits=%d  (No Valid Data)\r\n",fpga1of4, chans);
    strcat(Buf1500, tBuf);
    return chans;
}

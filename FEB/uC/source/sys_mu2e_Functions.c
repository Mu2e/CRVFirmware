//********************************************************
//  @file sys_mu2e_functions.c
//  Fermilab Terry Kiper 2016-2020
//  RM48 Code
//  RM48 Hercules device supports little-endian [LE] format
//
//  Read/Write FPGA and SDram Address Registers
//  Read FPGA ONE WIRE I/O CMB chips
//  Data pool reads of OneWire chips
//  Proto-type version uses SILABS SI5338C CLOCK GEN (I2C interface)
//********************************************************


/* Include Files */
#include "sys_common.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "het.h"
#include "gio.h"
#include "can.h"

#include "spi.h"
#include "sci.h"
#include "i2c.h"
#include "hw_reg_access.h"
#include "sys_core.h"
#include "sys_dma.h"
#include "reg_sci.h"

//sys_mu2e proto-types
#include "ver_io.h"
#include "sys_mu2e_FRAM.h"
#include "sys_mu2e.h"
#include "sys_mu2e_misc.h"
#include "sys_mu2e_functions.h"
#include "ver_ePHY.h"

#include "register_map.h"       //timing setup for clock chip SI5338 via siLabs app


extern const   int   AFEadr[];
extern int     CmbActiveCnt[4];         //count for act ch on each fpga
extern volatile    struct msTimers mStime; 

//OneWire buffers for pooled data
extern int         oneWireErr;          //oneWireData bad readings
extern int         oneWireTemp[4][4];   //oneWireData 4-FPGAs, 8ch-TEMPs
extern unsigned long oneWireRom[4][4];  //oneWireData 4-FPGAs, 8ch-ROMs
//extern const   int   AFEadr[]= {0x101, 0x102, 0x133, 0x134, 0x135, 0x201, 0x202, 0x233, 0x234, 0x235};   


extern struct stBlock PoolData;         //replaces 'statusBlock[5][sBLKSIZ38+2]' for data pooling
extern uint32 volatile genFlag, iFlag;

uint16      g_i2cErrors=0, g_i2cErr=0;



//FPGA (Bsee 0) write (addr,data16)
void wr16FPGA(long offset, short d16)
    {
    sPTR saddr = (sPTR) fpgaBase;
    saddr += offset;                    //adding to pointer, incr by ptr size
    *saddr= d16;
    }

//FPGA  (Bsee 0) read(addr)
int rw16FPGA(long offset)
    {
    int ret16;
    sPTR saddr = (sPTR) fpgaBase;
    saddr += offset;                    //adding to pointer, incr by ptr size
    ret16= *saddr;
    return(ret16);
    }


//Put the CPU in idle mode by executing the CPU idle instruction.
//asm(“ WFI”)
#pragma optimize= none
int IDLE()      //IDLE CPU
{
asm ("   WFI\n");
return 0;
}


//Reverse bits of parameter in the register R0, return word value is passed back
//to its caller in register R0.
#pragma optimize= none
unsigned short revB(short int c)
    {
    asm(" RBIT R0, R0 \n");                     //Asm lng rev bits in 32 register
    asm(" MOV R0, R0, LSR #16 \n");             //shift and return 16_bits in R0
  //asm(" BX lr\n");
    return c;
    }


//Reverse bits order in register R0, return byte value is passed back
//to its caller in register R0.
#pragma optimize= none
u_8Bit revB_byte(u_8Bit c)                      //reverse 8 bits
    {
    asm(" RBIT R0, R0 \n");                     //Asm lng rev bits in 32 register
    asm(" MOV R0, R0, LSR #24 \n");             //shift and return 8_bits in R0
  //asm(" BX lr\n");
    return c;
    }


//byte swap
#pragma optimize= none
unsigned short SwapByte(unsigned short x)
{
    __asm("    rev16   r0, r0\n"
          "    bx      lr\n");                  // need this to make sure r0 is returned
    return(x);                                  // return makes compiler happy - ignored
}


//16bit moves, cnt is in word count
//Move 16bits per count, Incr Src Req Only
#pragma optimize= none
uint16 movStr16_NOICDEST(unsigned short *src, unsigned short *dst, long int cnt)
{
asm ("PUSH {r0-r3}\n"               //dont need to push on reg0-3 ???
    "LDMloop:\n" 
    "   LDRH r3,[r0],#2\n"          //r0=src
    "   STRH r3,[r1]\n"             //r1=dst

    "   SUBS r2, r2, #1\n"          //r2=cnt
    "   BGT.N LDMloop\n"
    "   POP {r0-r3}\n");    
    return (unsigned int)src;
    }


//16bit moves, cnt is in word count
//Move 16bits per count, Incr Dest Reg Only
#pragma optimize= none
uint16 movStr16_NOICSRC(unsigned short *src, unsigned short *dst, long int cnt)
{
asm ("PUSH {r0-r3}\n"               //dont need to push on reg0-3 ???
    "LDMloop:\n" 
    "   LDRH r0,[r3]\n"             //r3=src
    "   STRH r0,[r1],#2\n"          //r1=dst

    "   SUBS r2, r2, #1\n"          //r2=cnt
    "   BGT.N LDMloop\n"
    "   POP {r0-r3}\n");    
    return (unsigned int)cnt;
    }

#pragma optimize= none
//16bit moves, cnt is in word count movStr16_NOINCR(src,dst,Wcnt)
//no increment on src or dest
uint16 movStr16_NOINC(unsigned short *src, unsigned short *dst, long int cnt)
{
asm ("PUSH {r0-r3}\n"               //dont need to push on reg0-3 ???
    "LDMloop:\n" 
    "   LDRH r3,[r0]\n"             //r0=src
    "   STRH r3,[r1]\n"             //r1=dst

    "   SUBS r2, r2, #1\n"          //r2=cnt
    "   BGT.N LDMloop\n"
    "   POP {r0-r3}\n");    
    return (unsigned int)src;
    }


//Load-Multiple memory copy
//Use LDM and STM instructions, transferring 8 words per iteration. Due to the 
//extra registers used these must be stored to the stack and later restored.

//The .n suffix is used in ARM UAL Thumb-2 assembly where you have a 
//choice of instruction encodings: 16- or 32-bit. It forces a 16-bit 
//encoding to be used.
#pragma optimize= none
unsigned short movStrBlk32(unsigned short *dst, unsigned short *src, int cnt)
{
asm ("PUSH {r4-r10}\n"                  //save r4-410 on stack
    "LDMloop:\n" 
    "   LDMIA r1!, {r3 - r10}\n"        //src uses 8 regs, 4 bytes ea
    "   STMIA r0!, {r3 - r10}\n"        //dst uses 8 regs, 4 bytes ea
    "   SUBS r2, r2, #32\n"             //decrement cont by 32 ie.(8regs*4bytes)
    "   BGE.N LDMloop\n"                //loop
    "   POP {r4-r10}\n");    
    return *dst;
    }
                    
//32bit moves, cnt is in lngword count
#pragma optimize= none
unsigned short movStr32(unsigned short *src, unsigned short *dst, short int cnt)
{
asm ("PUSH {r4-r10}\n" 
    "LDMloop:\n" 
    "   LDR r3,[r0],#4\n" 
    "   STR r3,[r1],#4\n" 

    "   SUBS r2, r2, #1\n"
    "   BGT.N LDMloop\n"
    "   POP {r4-r10}\n");    
    return (unsigned int)src;
    }


//16bit moves, cnt is in word count
#pragma optimize= none
unsigned short movStr16(unsigned short *src, unsigned short *dst, short int cnt)
{
asm ("PUSH {r0-r3}\n"   //dont need to push on reg0-3 ???
    "LDMloop:\n" 
    "   LDRH r3,[r0],#2\n" 
    "   STRH r3,[r1],#2\n" 

    "   SUBS r2, r2, #1\n"
    "   BGT.N LDMloop\n"
    "   POP {r0-r3}\n");    
    return (unsigned int)src;
    }




#pragma diag_suppress=Pe940                 //supress compiler warnings
#pragma inline = forced                      
char* get_SP( void )
    {
     /* On function exit,
     function return value should be present in R0 */
     asm( "MOV R0, SP" );
    }     



void MemCopy32(unsigned long *dst, unsigned long *src, int bytes)
{
  for (int i = 0; i < (bytes + 3) / 4; i++)
    *dst++ = *src++;
}



//network int to asci displey
char* inet_ntoa(unsigned long addr)
{
    static char addr_str[16];
    memset(addr_str,0,16);
    sprintf(addr_str,"%u.%u.%u.%u",(uint8)(addr & 0xFF),(uint8)(addr>>8 & 0xFF),
                                       (uint8)(addr>>16 & 0xFF),(uint8)(addr>>24 & 0xFF));
    return addr_str;
}



//AFE Register Req/Get data requesting
//  1st Request Data
//  2nd Get Data
//AFE Reg Request/Get Data 
int AFE_Req_Get_Data(u_32Bit fpga, int reg, int type) //type0=reqData, type1=getData
{
    uint16 d16;
    u_32Bit addr32;
    static u_32Bit AFEBASE,AFEGET;
    if (type==0)
        {
        //form AFE address
        fpga--;                         //adjust fpga from 1-4 to 0-3
        if (fpga)                       //fpga on 0x400 boundarys
            fpga= (0x800*fpga);         //fpga buss address offset
        AFEGET= (u_32Bit)AFE_RdData+fpga;           //data port for fpga '1of4'
        addr32= (u_32Bit)fpgaBase+fpga+(reg<<1);    //actual 32 bit address
        AFEBASE=(u_32Bit)fpgaBase+fpga+((reg&0x0F00)<<1); //afe chip 1 or 2
        *(sPTR)AFEBASE= 2;              //read mode
        *(sPTR)addr32=0;                //dummy read to clk data out of device
        }
    else if (type==1)
        {
        //tek Nov2017 was 20 now 10
        //hDelayuS(15);
        d16= *(sPTR)AFEGET;             //read data
        *(sPTR)AFEBASE= 0;              //return to default write mode
        }
    return d16;
}



//AFE Register Req/Get data requesting
//  1st Request Data 
//  Wait 10uS
//  2nd Get Data
//read 16bit AFE register
//takes about 13uS with fixed 10uS delay waiting for data
int readAFE_Slow(u_32Bit fpga, int reg)
{
    uint16 d16;
    u_32Bit addr32, AFEBASE,AFEGET;
    //form AFE address
    fpga--;                         //adjust fpga from 1-4 to 0-3
    if (fpga)                       //fpga on 0x400 boundarys
        fpga= (0x800*fpga);         //fpga buss address offset
    AFEGET= (u_32Bit)AFE_RdData+fpga;           //data port for fpga '1of4'
    addr32= (u_32Bit)fpgaBase+fpga+(reg<<1);    //actual 32 bit address
    AFEBASE=(u_32Bit)fpgaBase+fpga+((reg&0x0F00)<<1); //afe chip 1 or 2
    *(sPTR)AFEBASE= 2;              //read mode
    *(sPTR)addr32=0;                //dummy read to clk data out of device
  //tek Nov2017 was 20 now 10
    hDelayuS(10);
    //uDelay(10);      
    d16= *(sPTR)AFEGET;             //read data
    *(sPTR)AFEBASE= 0;              //return to default write mode
    return d16;
}




//write 16bit AFE register, fpga 0-3
void wrAFE_Slow(int fpga, int reg, int data)
{
    u_32Bit addr32, AFEcsr;
    if (fpga)                       //fpga on 0x400 boundarys
        fpga= (0x800*fpga);         //fpga buss address offset
    //form AFE address
    AFEcsr= (u_32Bit)AFE_0x100+ fpga;
    addr32= (u_32Bit)fpgaBase+ fpga + (reg<<1);   //actual 32 bit address
    *(sPTR)AFEcsr= 0;           //write mode is pwr up default reg0=0
    *(sPTR)addr32=data;         //write data
    hDelayuS(50);
    //uDelay(50);
}




//Set SDram Read Address and wait for status to show burst buffers ready
//Note FPGA SDram control logic, setting rd ptrs wait for ram burst to fill buffer
//FPGA BUS ADDR 1of4 0x0000, 0x0400, 0x800, 0xC00
//
//0x008 MIG Status Register Bit9 'Read Data Fifo Empty == 1'
int SET_SDADDR_RDx(int chipOffset, int HI, int LO)
{
    int i=0;
	//set read addr to base, doesnt always change

    sPTR sdAddr= (sPTR)(fpgaBase+(chipOffset*2)+ (0x04*2));
    *sdAddr++= HI;
    *sdAddr= LO;
    //for (; i< 80; i++);  //min delay may not be needed, call time may be enough
    i=0;
    while (SDramMIGStat& BIT9)  //wait for 'MIG read data FIFO to go 0==Non_Empty'
        {                       //allows time for new data to fill fifo
        if(i++> 40)            //breakout, 'Potentially infinite loop'
            break;
        }
    return 0;
}


//Set SDram Write Address and wait for status to show burst buffers ready
//Note FPGA SDram control logic, setting write pointers will flush ram burst (16 words) 
//into memory at the new write address overwritting existing data.
//FPGA BUS ADDR 1of4 0x0000, 0x0400, 0x0800, 0x0C00
//
//0x008 MIG Status Register Bit5 'Write Data Fifo Empty == 1'
int SET_SDADDR_WRx(int chipOffset, int HI, int LO)
{
	//set read addr to base, doesnt always change
    sPTR sdAddr= (sPTR)(fpgaBase+(chipOffset*2)+ (0x02*2));
    *sdAddr++= HI;
    *sdAddr= LO;
    int i=0;
    //for (; i< 80; i++);  //min delay may not be needed, call time may be enough 
    while((SDramMIGStat&BIT5)==0)//wait for 'MIG WRITE data FIFO to go 1==Empty'
        {                        //allows time for fifo to flush old data
        if(i++> 80)              //breakout, 'Potentially infinite loop'
            break;
        }
    return 0;
}


typedef union _un_adcData {
	uint16	u16dat[5];
	u_8Bit  u8dat[10];
}un_OneWire;



#define idxDAC 0            //index to DACs within status block data
#define idxTmp 24           //index to temperatures within status block data
#define idxAFE idxTmp+4     //index to afe read data within status block data

//runs 'On Demand' by cmds 'CMB' and 'STAB' plus every 15Sec in 'notification.c'
//req one wire data Refresh, fills in background at 15mS/Channel
//part one update request (each CMB device update takes minimum 10mS)
//part two read updated data
//genFlag 'OneWireDatRdy' set when data is ready
//function run time is 4-20uSec
int OneWireTrigRead(void)
{
    static uint16 Fpga=0, Ch0_3=0, cycCnt=1, AFEch=0;
    static uint16* fPtr= (uint16*)s0AFE_DACbias;
    static un_OneWire one;
    uint16 vOFF, chan, *dPtr, d16;
    static uint16 readMODE= ONE_TEMP;    
    static int CmbUpdateCnt[4];         //count for act ch on each fpga
    
//hHI_TP45;    
    //Part one, 'Trig Data Request' if flag set 
    if (genFlag & OneWireDatReq)
        {
//putBuf(1,"CMB REQ\r\n",0);     tek testing
        if (Ch0_3 >3)                       //ready for 1of4 passes
            {
            Ch0_3=0;
            
            //read DAC channels
            if (Fpga==0)     fPtr= (uint16*)s0AFE_DACbias;
            else if(Fpga==1) fPtr= (uint16*)s1AFE_DACbias;
            else if(Fpga==2) fPtr= (uint16*)s2AFE_DACbias;
            else             fPtr= (uint16*)s3AFE_DACbias;
            for (int ii=0; ii<24; ii++)
                PoolData.afe[Fpga][ii]= *fPtr++;      //store AFE register 1_of_10        
            Fpga++;
            if (Fpga>3)
                Fpga=0;
            }
        
        //update afe registers, 4 FPGAs with 10 Chs each
        d16= readAFE_Slow(Fpga+1, AFEadr[AFEch]);  //takes ~13uSec
        PoolData.afe[Fpga][idxAFE+AFEch]= d16;
        if (++AFEch>9)
            AFEch=0;
       
        //start ROM reads Ch 1-16 for sequence
        if (cycCnt>16)          
            readMODE= ONE_ROM;
        vOFF= (0x400*Fpga);             //fpga on 0x400 boundarys
        dPtr= (sPTR)ONEWIREdata + vOFF;
        uint16 *CNTRLptr= (sPTR)ONEWIREcntr+ vOFF;
        uint16 *CMDptr=   (sPTR)ONEWIREcmd+ vOFF;
        chan= (1<<Ch0_3);
        *(sPTR)CNTRLptr= chan;          //sel channel
        *(sPTR)CMDptr= readMODE;        //mode read temp or ROM
        genFlag &= ~OneWireDatReq;      //clear data req 
        mStime.g_OneWireTime=1;         //rdy timer for next ch
        genFlag |= OneWireRefresh;      //flag state as refresh active
  //hLO_TP45;        
        return 0;
        }
    else
        {
        //Part two read data now, busy done was checked in 1mS Sys-Tick-Intr function         
        vOFF= (0x400*Fpga);             //fpga on 0x400 boundarys
        dPtr= (sPTR)ONEWIREdata + vOFF;
        unsigned long d64;

//putBuf(1,"CMB GET\r\n",0);         tek testing
        
        for (int i=0;i<6;i++)
            one.u16dat[i]= *(sPTR)dPtr++;   //read 1 data word

        //if reading temp return word 5
        if(readMODE==ONE_TEMP)
            {
            if(one.u16dat[4] == 0x550)  //skip, its same as default data after a reset
                oneWireErr++;
            else
                {
                oneWireTemp[Fpga][Ch0_3]= one.u16dat[4]; 
                //statusBlock[Fpga+1][Ch+idxTmp]= one.u16dat[4];   //store temperature
                PoolData.afe[Fpga][Ch0_3+idxTmp]= one.u16dat[4];   //store temperature
                }
            }
        else
            {
            //return data as (lower 4 of 6 bytes) plus toss crc and famcode0x28
            //MSB-LSB  ([5]<<32)+ ([4]<<24)+ ([7]<<16)+ ([6]<<8) + [9]
            //oneWireData 4-FPGAs -- 4ch-ROMs 
            d64= ((one.u16dat[3]<<16)+ one.u16dat[4]);    
            oneWireRom[Fpga][Ch0_3]= ((one.u8dat[4]<<24)+(d64>>8));
            if (oneWireRom[Fpga][Ch0_3])       //if non zero add to count
                CmbUpdateCnt[Fpga]++;
            }
        cycCnt++;
        if (cycCnt>32)                      //counts to read 16_TEMPs + 16_ROMs
            {
            Ch0_3=0;
            Fpga=0;
            cycCnt=1;
            mStime.g_OneWireTime=0;         //stop timer
            readMODE= ONE_TEMP;             //read TEMPs first
            genFlag &= ~OneWireRefresh;     //flag refresh complete
            //delay for triggering new scan
            mStime.g_OneWireNewScanDelay++; //start delay timer for new scan 5
            //total cmb update per fpga when scan complete
            for (int i=0; i<4; i++)
                CmbActiveCnt[i]= CmbUpdateCnt[i];       
            for (int i=0; i<4; i++)
                CmbUpdateCnt[i]=0;          //total cmb counted on each pass            
            }
        else
            {
            mStime.g_OneWireTime=1;         //reset timer
            genFlag |= OneWireDatReq;       //req next chan read
            //data for this channel now stored, do next ch
            Ch0_3++;  
            }
        genFlag &= ~OneWireDatRdy;          //clear data ready, finished all chs 
        mStime.g_OneWireNewScanDelay++; //start delay timer for new scan 5
//    hLO_TP45;        
        return 1;
        }
}



extern FPGA_RegS    f_RegConst [FP_REGS_MAX];
extern FPGA_RegS1   f_RegHold[4][FP_REGS_MAX];
extern struct       uC_Store uC_FRAM;    
extern struct       uSums u_SRAM;     //DwnLdSdRamCnt, DwnLdSdRamSum,stat,time


//dsav function used in main() 'case: D'
void dsav()             //read/store fpga regs to FRAM
    {
    uint16 framAddr;
    uint16 cnt=0, d16, rcounter; 
    u_32Bit addr32;
    u_32Bit FPGA_BASE= fpgaBase;
     for(int chip=0; chip<4; chip++)
        {
        framAddr= ((fPAGE_FPGA0+4) +(0x100*chip));
        rcounter=0;
        addr32 = (u_32Bit)FPGA_BASE;  
        //check if afe chips enabled
        //read FPGAs(4) and store data to FRAM
        //FP_REGS_MAX number of register to address
        //cnt= Words per Base Reg(with addr increment) to write
        for(cnt=1; cnt<FP_REGS_MAX; cnt++)
            { 
            addr32 = (u_32Bit)FPGA_BASE + (f_RegConst[cnt].Addr<<1);  //actual 32 bit address
            for (int i=0; i< f_RegConst[cnt].RegCnt; i++, addr32+=2)
                {
                if (f_RegConst[cnt].Addr <0x100)            //fpga low range
                    d16= *(sPTR)addr32;
                else if (f_RegConst[cnt].Addr >=0x300)      //fpga high range
                    d16= *(sPTR)addr32;
                else                                        //afe addr range
                    {
                    //use default setup
                    d16= f_RegConst[cnt].Value[i];
                    }
                //now store read data to FRAM 
                f_RegHold[chip][cnt].Value[i]= d16;
                FRAM_WR_BUF16(framAddr, &d16, 1); //fRamAddr, fpgaAddr, write Cnt
                framAddr += 2;
                rcounter++;
                }
            }
        FPGA_BASE += 0x800; //offset to next fpga 1of4
        }
    //Save valid flag
    f_RegHold[0][0].Value[0]= FP_REGS_VALID0;   //base add hold flag
    f_RegHold[0][0].Value[1]= rcounter;         //base addr holds load cnt
    f_RegHold[1][0].Value[0]= FP_REGS_VALID1;   //base add hold flag
    f_RegHold[1][0].Value[1]= rcounter;         //base addr holds load cnt
    f_RegHold[2][0].Value[0]= FP_REGS_VALID2;   //base add hold flag
    f_RegHold[2][0].Value[1]= rcounter;         //base addr holds load cnt
    f_RegHold[3][0].Value[0]= FP_REGS_VALID3;   //base add hold flag
    f_RegHold[3][0].Value[1]= rcounter;         //base addr holds load cnt
    FRAM_WR_BUF16(fPAGE_FPGA0, &f_RegHold[0][0].Value[0], 2);     //fRamAddr, fpgaAddr, write Cnt
    FRAM_WR_BUF16(fPAGE_FPGA1, &f_RegHold[1][0].Value[0], 2);     //fRamAddr, fpgaAddr, write Cnt
    FRAM_WR_BUF16(fPAGE_FPGA2, &f_RegHold[2][0].Value[0], 2);     //fRamAddr, fpgaAddr, write Cnt
    FRAM_WR_BUF16(fPAGE_FPGA3, &f_RegHold[3][0].Value[0], 2);     //fRamAddr, fpgaAddr, write Cnt
    //store {SerNumb, MuxCh, Gain, Trig, LnkDir, nErr, xPhy, wDogTimOut}
    FRAM_WR_BUF16(fPAGE_0400, (uint16*)&uC_FRAM, FRAM_400wsz); //addr, data, WordCnt
}




//ePHY download struc
extern struct      uSums uPhySums;
extern char        Buf1500[];
extern struct      HappyBusReg HappyBus;
extern char        tBuf[];

//ldram function used in main() 'case: L'
//receives fpga bin data file from controller (controllers xmit file command 'LDFEB')
//Data received on ePHY port and stored in FPGA1 sdRAM  (fpga1,fpga2,fpga3,fpga4) 
//FEB acknowledges each packet with one word xmit on LVDS port
//If received wrd cnt matches expected word cnt FEB sends ACK, else sends NACK
//return status 1=pass, 0=fail
//
int ldRam_DataFile( int prt, int param1, u_16Bit* eBufB, u_16Bit* SumPHYPtr )     //Load data file to SDRAM from FEB Controller cmd 'LDFILEFEB'
{
    static int recPaks=0;
    int d16, EndWrdCnt;
                  
    if (HappyBus.CmdType == eCMD73_LDRAMINIT)   //init handler
        {
        for(int i=0; i<6;i++)                          
            {                          
            d16= *eBufB++;                      //store cnt and chksum
            *SumPHYPtr++ = d16;                 //store cnt and chksum
            }
        sprintf(Buf1500,"\r\nSource File= %-6d   CkSum=%04X\r\n", uPhySums.DwnLd_sCNT, uPhySums.DwnLd_sSUM&0xffff);
        putBuf(0, Buf1500, 0);
                        
        //set write fpga Addr via special sequence
        SET_SDADDR_RDx(0,0,0);
        SET_SDADDR_WRx(0,0,0);

        //clear fake datga download counters
        u_SRAM.DwnLd_sSUM=0;
        u_SRAM.DwnLd_sCNT=0; 
                                                
        recPaks=1;                       
        FMDataSnd= recPaks;         //lvds reply to ephy, send brd num as 'ack'
        //recd init packet only, exit
        return 1;                 //1==okay
        }
    //data handler
    recPaks++;
    //*Buf1500=0;
    //if ((HappyBus.CmdType& 0xff)== eCMD74_LDRAMFLUSH) //end of file flush buffer handler
    //    EndWrdCnt= param1; //HappyBus.CmdType>>8;
    //else
        EndWrdCnt= param1;
                    
    while(param1-- > 0)
        {
        d16= *eBufB++;              //write to fpga SD_Ram
        SDR_RD16= d16;              //store to sdRam1
        //HISTDAT16= d16;           //write fpga2 Histogram FIFO 
        if(EndWrdCnt)
            {
            EndWrdCnt--;
            u_SRAM.DwnLd_sSUM += ((d16>>8)&0xff);
            u_SRAM.DwnLd_sSUM += (d16&0xff);
            u_SRAM.DwnLd_sCNT += 2;
            }
        }
    //sprintf(tBuf,"Pac %d\r\n",recPaks );
    //putBuf(0, tBuf,0);
    //LVDS ACK TO CONTROLLER
    FMDataSnd= recPaks;             //lvds reply to ephy, send brd num as 'ack'
           
    //check if done
    if(u_SRAM.DwnLd_sCNT==uPhySums.DwnLd_sCNT)
        {                          
        //flush fpga sdram contollers wr buffer (up to 16 words needed)
        for(int i=0;i<16;i++)
            SDR_RD16= d16;              

        sprintf(tBuf ,"FEB  :Rec'd  File: %-6d   CkSum=%04X     Total_Packets=%d\r\n",u_SRAM.DwnLd_sCNT, u_SRAM.DwnLd_sSUM&0xffff, recPaks);
        putBuf(0, tBuf, 0);   
        if (u_SRAM.DwnLd_sSUM==uPhySums.DwnLd_sSUM)
            FMDataSnd= ACK;
        else
            {
            FMDataSnd= NACK;
            sprintf(tBuf ,"FEB  : File Xfer Error\r\n");
            putBuf(0, tBuf, 0);  
            return 0;              //0==fail
            }
        }
                    
    //NOTE PROMPTS GETS SENT BACK ON LVDS ON EXIT HERE   
    genFlag |= NO_ECHO;             //no ASCII reply   
    return 1;                     //1==okay
}                              



//wiznet stuff chip W5300
#include "socket.h"

extern int         recDatSz[];                  //store last rec'd packet data size (diagnostics)
extern int         eTout[];                     //store socket timeout status from intr read
extern uint8       I_STATUS[];                  //store socket status for backround processing
extern struct      udpHeader udpHdr;
extern struct      blocksnd    BinSt;
extern char*       inet_ntoa(unsigned long addr);


void sockDisplay(int prt, int param1, char* paramPtr)   //socket status reg 0x403,0x503,...
    {
    int d8;
    uint8 datAr[4];
    param1=arg_dec(&paramPtr,0);
    *Buf1500=0;

#ifdef _FEB_PRODUCTION             //Production boards layout allow extra feature         
    if(WIZLink==1)
        {
        sprintf(tBuf," Local Ethernet Cable **Not Connected**\r\n");      
        strcat(Buf1500, tBuf);
        }
#endif    
    sprintf(tBuf," Sock Status  xBufSz  rDatSz   xTOut    Connection\r\n");
    strcat(Buf1500, tBuf);
    for (int ch=0; ch<=sock2; ch++)             //8sock, only show enabled ones
        {
        sprintf (tBuf,"   %1d    %02X     %04X    %04X   %4X      ", ch, getSn_SSR(ch),
                getSn_TX_FSR(ch), recDatSz[ch], eTout[ch] );
        strcat(Buf1500, tBuf);

        d8= getSn_SSR(ch);
        if (d8==SOCK_CLOSED)
            sprintf(tBuf,"CLOSED\r\n");
        else if (d8==SOCK_ARP)
            sprintf(tBuf,"ARP\r\n");
        else if (d8==SOCK_INIT)
            sprintf(tBuf,"INIT\r\n");
        else if (d8==SOCK_LISTEN)
            sprintf(tBuf,"LISTEN\r\n");
        else if (d8==SOCK_SYNRECV)
            sprintf(tBuf,"Connect-request(SYN packet)\r\n");
        else if (d8==SOCK_ESTABLISHED)
            {
            getSn_DIPR(ch, datAr);          //ip
            //getSn_PORTR(ch, port);        //port
            sprintf(tBuf,"CONNECTED %u.%u.%u.%u\r\n", datAr[0],datAr[1],datAr[2],datAr[3]);
            }
        else if (d8==SOCK_CLOSE_WAIT)
            {
            sprintf(tBuf,"CLOSE_WAIT\r\n");
            close(ch);
            I_STATUS[ch]=SOCK_CLOSED;       //force status check
            }
        else if (d8==SOCK_UDP)
            {
            if (udpHdr.destip)
                {
                char * cptr= inet_ntoa(udpHdr.destip);
                sprintf(tBuf,"SOCK_UDP  %s" , cptr );
                strcat(Buf1500, tBuf);
                sprintf(tBuf," (%u)\r\n",udpHdr.destport);
                udpHdr.destip= 0;           //zero, next time show any new udp
                udpHdr.destport= 0;
                }
            else
                {
                sprintf(tBuf,"SOCK_UDP\r\n");
                }
             }
        else
            sprintf(tBuf,"*** ERROR ***  StatusReg=%02X (Normal=0x14)\r\n", d8);
        strcat(Buf1500, tBuf);
        }
    putBuf(prt, Buf1500,0);  
    
    //3 sockets, timouts set in function send_full()
    eTout[0]=0;
    eTout[1]=0;
    eTout[2]=0;                        
    strcat(Buf1500, "\r\n\r");              //extra char if odd byte cnt on 16bit LVDS xfer
                                        
    if(param1==1)
        {
        //show  ASCII sock send delays
        //sprintf(tBuf,"ASCSndDly >20mSec : %d\r\n",  BinSt.gASC_TimDlys[0] );
        //strcat(Buf1500, tBuf);
        //sprintf(tBuf,"ASCSndDly 10-20mS : %d\r\n",  BinSt.gASC_TimDlys[1] );
        //strcat(Buf1500, tBuf);
        //sprintf(tBuf,"ASCSndDly 5-10mS  : %d\r\n",  BinSt.gASC_TimDlys[2] );
        //mod to limit printouts
        sprintf(Buf1500," -------------------------------------\r\n FEB Send Delays all Socks\r\n");
        sprintf(tBuf," mSec        ASC        BIN \r\n");
        strcat(Buf1500, tBuf);
                        
        sprintf(tBuf,"  > 5     %6d     %6d\r\n", BinSt.gASC_TimDlys[2]+BinSt.gASC_TimDlys[0]+BinSt.gASC_TimDlys[0],
                                                BinSt.gDMA_TimDlys[2]+ BinSt.gDMA_TimDlys[1]+ BinSt.gDMA_TimDlys[0] );
        strcat(Buf1500, tBuf);
                        
        sprintf(tBuf,"  1-5     %6d     %6d\r\n", BinSt.gASC_TimDlys[3], BinSt.gDMA_TimDlys[3] );
        strcat(Buf1500, tBuf);

        sprintf(tBuf," .1-1     %6d     %6d\r\n\r", BinSt.gASC_TimDlys[4], BinSt.gDMA_TimDlys[4] );
        strcat(Buf1500, tBuf);
        putBuf(prt, Buf1500,0);  

        //show 'rdb cmd sock send delays'
        //mod to limit printouts
        //sprintf(tBuf,"BinSndDly >20mSec : %d\r\n",  BinSt.gDMA_TimDlys[0] );
        //strcat(Buf1500, tBuf);
        //sprintf(tBuf,"BinSndDly 10-20mS : %d\r\n",  BinSt.gDMA_TimDlys[1] );
        //strcat(Buf1500, tBuf);
        //sprintf(tBuf,"BinSndDly 5-10mS  : %d\r\n",  BinSt.gDMA_TimDlys[2] );
        //clear counters
        for (int i=0; i<5; i++)
            {
            BinSt.gDMA_TimDlys[i]=0;
            BinSt.gASC_TimDlys[i]=0;
            }                        
        }
    
}


//tek March 2021 following code here down is not used


/*
//Send block of data using RM48L seq logic
// in:  data count
//      data ptr
// ret: 0=fail
//ack  --- active low level
//nack --- active high level
int i2cSendData(uint8_t slvAddr, uint8_t slvReg, uint16_t cnt, uint8_t *dBlk )
{
    int i, iTimout=0;
    //Set the Destination Slave address 
    i2cSetSlaveAdd(i2cREG1, slvAddr);   

    //Check for Bus Busy
    while ((i2cREG1->STR & I2C_BUSBUSY) )
       i2cREG1->MDR = 0;		    //reset I2C so SCL isn't held low
    
    //Disable I2C during configuration
    i2cREG1->MDR= 0;
    //for (int i=0;i<100;i++);
    uDelay(10);
    //Set the I2C controller to write len bytes
    i2cREG1->CNT=cnt+1;               //used when I2C_REPEATMODE bit is set
    
    //reset change this later
    i= (I2C_RESET_OUT | I2C_MASTER | I2C_TRANSMITTER | I2C_START_COND); //I2C_REPEATMODE
    i2cREG1->MDR= i;
  //for (int i=0;i<100;i++);
    uDelay(10);

    //Transmit Slv Reg Select byte
    // Wait for "XRDY" flag to transmit data or "ARDY" if we get NACKed
    while ( !(i2cREG1->STR & (I2C_TX|I2C_ARDY)) );
    // If a NACK occurred then SCL is held low and STP bit cleared
    if ( i2cREG1->STR & I2C_NACK )
        {
        i2cREG1->MDR = 0;		    //reset I2C so SCL isn't held low
        return 1;                   //1==fail
        }
    i2cREG1->DXR= slvReg;       //slave chips register 

    //Transmit data 
    for (i=0; i< cnt; i++)
        {
        // Wait for "XRDY" flag to transmit data or "ARDY" if we get NACKed
        while ( !(i2cREG1->STR & (I2C_TX|I2C_ARDY)) )
            {
            uDelay(10);
            if (iTimout++> 100)
                return 1;
            }
        
        // If a NACK occurred then SCL is held low and STP bit cleared
        if ( i2cREG1->STR & I2C_NACK )
            {
            i2cREG1->MDR = 0;		    //reset I2C so SCL isn't held low
            return 1;                   //1==fail
            }
        i2cREG1->DXR= dBlk[i];          //slave chip data to receive
        //while (i2cREG1->MDR!= 0);     //busy
        }
    i2cREG1->MDR |= I2C_STOP_COND;      // Generate STOP
	while(i2cREG1->STR & I2C_STOP_COND);

    //allow sequence finish, else future code may interfer
    uDelay(200);                        //uS Delay    
    return 0;
}
 

//will hang here if optimized beyond 'LOW', leave in code as a warning
#pragma optimize=low

//receive block of data using RM48L seq logic
// in:  reg to read
//      read count
//      data ptr
// ret: 0=fail
//ack  --- active low level
//nack --- active high level
int i2cRecvData(uint8_t slvAddr, uint8_t slvReg, uint16_t rdcnt, uint8_t *dBlk)
{
    int i, iTimout=0;
    //Set the Destination Slave address 
    i2cSetSlaveAdd(i2cREG1, slvAddr);   
    
    //Check for Bus Busy
    while ((i2cREG1->STR & I2C_BUSBUSY) )
       i2cREG1->MDR = 0;		        //reset I2C so SCL isn't held low
    
    //Disable I2C during configuration
    i2cREG1->MDR= 0;

    //Set the I2C controller to write len bytes
    i2cREG1->CNT=1;                     //used when I2C_REPEATMODE bit is set
    
    //reset change this later (I2C_REPEATMODE)
    i2cREG1->MDR= (I2C_RESET_OUT| I2C_MASTER| I2C_TRANSMITTER| I2C_FREE_RUN| I2C_START_COND);

    //Transmit data 
    //Wait for "XRDY" flag to transmit data or "ARDY" if we get NACKed
    while ( !(i2cREG1->STR & (I2C_TX|I2C_ARDY)) )
        if (iTimout++> 500000)
              return 1;
        
    //If a NACK occurred then SCL is held low and STP bit cleared
    if ( i2cREG1->STR & I2C_NACK )
        {
        i2cREG1->MDR = 0;		        //reset I2C so SCL isn't held low
        return 1;                       //1==fail
        }
    i2cREG1->DXR= slvReg;  
    iTimout=0;
    //wait for ARDY before beginning the read phase of the transaction
    while ( !(i2cREG1->STR & (I2C_ARDY)) )
            {
            uDelay(10);
            if (iTimout++> 100)
                return 1;
            }
 
    // Set the I2C controller to read len bytes //
    i2cREG1->CNT=rdcnt;                 //used when I2C_REPEATMODE bit is set
  
    i= (I2C_RESET_OUT | I2C_MASTER | I2C_FREE_RUN | I2C_STOP_COND | I2C_START_COND ); 
    i2cREG1->MDR= i;
 
    //Receive data
    for (i = 0; i < rdcnt; i++)
        {
        uDelay(50);
        //Wait for I2C to read data or or "ARDY" if we get NACKed
        while( !(i2cREG1->STR & (I2C_RX|I2C_ARDY)) ) {};
        //If a NACK occurred then SCL is held low and STP bit cleared
        if ( i2cREG1->STR & I2C_NACK )
            {
            i2cREG1->MDR= 0;		    // reset I2C so SCL isn't held low
            return 1;
            }
        //Make sure that you got the RRDY signal
        while( !(i2cREG1->STR & I2C_RX) ) {};
        dBlk[i] = i2cREG1->DRR;
      }
    return 0;
}


//i2c send/rec command/data temp array
uint8_t     i2Data[10];

//Modify i2c si5338 chip Reg   ...Read ... Modify .... Write
uint8_t i2c_RdModWr_5338(uint8_t wrReg, uint8_t rdReg, uint8_t mask)
{
    //do a read-modify-write    (1st...read)
    g_i2cErr= i2cRecvData(Dev5338, rdReg, 1, i2Data);
    if (g_i2cErr)
        {
        g_i2cErrors++;
        putBuf(tty, "i2c Err(3)\r\n",0);
        return 1;
        }
      
    i2Data[0] &= mask;                  //5338 reg mask
    //do a read-modify-write    (2nd...write)
    g_i2cErr += i2cSendData(Dev5338, wrReg, 1, i2Data );  //cnt, &data, rtns data in i2Data[]
    if (g_i2cErr)
        {
        g_i2cErrors++;
        putBuf(tty, "i2c Err(3)\r\n",0);
        return 1;
        }
    return 0;           //return 0 for no error
}


*/


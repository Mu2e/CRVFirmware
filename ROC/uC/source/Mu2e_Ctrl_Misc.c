//********************************************************
//  @file Mu2e_Cntrl_Misc.c
//  Fermilab Terry Kiper 2016-2021
//
//  RM48 Code
//  RM48 Hercules device supports little-endian [LE] format
//
//  Program Xilinx Spartan 6 fpga
//  Program and Read Spansion Inc S29JL064J 64 Megabit 
//                  (8M x 8-Bit/4M x 16-Bit) Flash Memory
//  Program and Read FRAM memory chip 'FM25CL64B'
//********************************************************

/*
Spansion S29JL064J 64Megabit (8M x 8-Bit/4M x 16-Bit) Flash Memory

Bank1  Sector   SectorAddr A21–A12      Addr16 (not byte addr)
        SA0         0000000000          00000h–00FFFh    SA0-SA7   8K-BYTES  (08x)
        SA22        0001111xxx          78000h–7FFFFh    SA8-SA22 64K-BYTES  (15x)
        total 983,025 bytes
          

Bank2  Sector   SectorAddr A21–A12      Addr16 (not byte addr)
        SA23        0010000xxx          80000h–87FFFh       SA73-SA118 64K-BYTES
        SA70        0111111xxx          1F8000h–1FFFFFh    
        total 3,145,680 bytes

Bank3  Sector   SectorAddr A21–A12      Addr16 (not byte addr)
        SA71        1000000xxx          200000h–207FFFh     SA23-SA70 64K-BYTES
        SA118       1101111xxx          378000h–37FFFFh    
        total 3,145,680 bytes

Bank4  Sector   SectorAddr A21–A12      Addr16 (not byte addr)
        SA119       1110000xxx          380000h–387FFFh     SA119-SA133 64K-BYTES (15x)
        SA141       1111111111          3FF000h–3FFFFFh     SA134-SA141 8K-BYTES  (08x)
        total 983,025 bytes

*/



#include "sys_common.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "het.h"
#include "gio.h"
#include "hw_reg_access.h"
#include "sys_core.h"
#include "sys_dma.h"
#include "reg_spi.h"
#include "spi.h"

#include "fram.h"
#include "ver_io.h"
#include "Mu2e_Ctrl.h"
#include "Mu2e_Ctrl_Misc.h"
#include "ver_ePHY.h"


g_dmaCTRL       g_dmaCTRLPKT1; 
extern  uint32  g_timeMs;
extern  char    Buf1500[1500];
extern  char    tBuf[400];

extern struct   msTimers mStime; 
extern struct   ePHY_STRUCT ePHY_PORT;

extern struct   uC_Store u_SRAM;
extern struct   vB USB_Rec;
extern void     uDelay(uint32);
extern void     mDelay(uint32 cnt);

extern  int     getBufBin();
extern  u_8Bit  revB_byte(u_8Bit);          //reverse 8 bits
extern  void    putBuf(int prt, char* sBuf,  int len);

int     g_Cnt, g_Done,cSum;

extern struct  sLVDS_ePHY_REG IOPs[];       //testing link assignment regs structure



//load FPGA with data from USB input port
//The BUSY signals can be left unconnected if readback is not needed.
//Note: Some Spartan FPGA families use the term parallel mode. Parallel mode 
//is equivalent to the SelectMAP mode in function.

//SelectMAP configuration is run greater than 50 MHz for Virtex families, the BUSY
//line must be monitored to ensure that data is transferred. BUSY going High indicates
//that the last data byte was not transferred and must remain on the data bus.

//Configuration data is loaded one byte at each rising CCLK edge, and 
//the MSB of each byte is presented on the D0 pin, not the D7 pin.


//Initially, the FPGA is driving INIT_B low and PROGRAM_B low, to indicate that 
//it is powering up. Once it is ready to clear the configuration latches, 
//PROGRAM_B and then INIT_B will be "released" so that they can float high. 
//They are in fact driven by the external pull-up resistor. The Micro can then 
//prolong this stage by driving either PROGRAM_B or INIT_B low.

//RM48 Hercules device supports the little-endian [LE] format

//#define INIT_Bx[]={29,16,27,6}; //Init array list of 'init' het port#
//#define hLO_INIT_B0     hetREG1->DCLR=  BIT29;
//#define hHI_INIT_B0     hetREG1->DSET=  BIT29;

//#define CSI_Bx[]={2,3,4,5};  //Init array list of chip select gio port#
//#define gLO_CSI_B0  gioPORTA->DCLR = (uint32)1U << 2;
//#define gHI_CSI_B0  gioPORTA->DSET = (uint32)1U << 2;


//fpga load direct from USB port using binary data file
int loadSpartan6_FPGA(int chip)
{
    sPTR pFlash= pFLASHbase;             //short ptr, parallel flash base address
    uint8_t d8;
    uint16_t d16;
    int lp=0;
    
    
    //Clear all counters
    cSum=0;
    g_Cnt=0;
    g_Done=0;
    
    //RDWR_B low before CSI_Bx, 0=Write to fpga    
    //keep low if read-back not needed
    RDWR_B_LO                           
    
    
    //The PROGRAM_B pin can be held active (Low) for as long as necessary, and the
    //device clears the configuration memory twice after PROGRAM_B is released.
    PROGx_LO                            //min 300nS
    uDelay(5);
    //Hold the INIT_B pin Low during initialization. When INIT_B has gone High,
    //configuration cannot be delayed

    //after reset delay needed    

 //tek, remove comment when fpga all work    
    PROG0_LO                //min 300nS
    uDelay(5);
    PROG0_HI                       
    //after reset delay needed    
    uDelay(1500);                        //fpga reset takes about 800uS
   
    //If the INIT_B pin is held Low externally, the device waits at this point in 
    //the initialization process until the pin is released.	
    if (DONE0)
        {
        CSI_B0_HI                       //set CSI_B0 high
        sprintf(tBuf,"LoadSpartan6: DONE0 ALREADY HIGH....Exit loader\r\n");
        putBuf(tty, tBuf,0);
        return -1;                      //nStatus error status
        }
	
          
    if (InitB0_Read==0)                     //wait
        {
        sprintf(tBuf,"LoadSpartan6: InitB0 Low Error? (expect High)\r\n");
        putBuf(tty, tBuf,0);
        return -1;                          //nStatus error status
        }

    //enable FPGA for programming (1 of 4)
    CSI_B0_LO                               //set CSI_B0 low    
                   
    while(lp++ < 100000)
        if (InitB0_Read==1) break;          //wait      
    //timeout error (normal range is 4500 @220Mhz Processor Speed)
    
    if (lp >= 50000)
        {
        sprintf(tBuf,"LoadSpartan6: Timeout InitBx Post Reset Low Error? (expect High)\r\n");
        putBuf(tty, tBuf,0);
        return -1;                      //nStatus error status
        }

    //delay needed after reset, else load fails
    uDelay(100);
   
    //SetInitB0_OutPut                      //Config pin (A3) HET1_29 as OutPut
    //send data fpga
    sprintf(tBuf,"LoadSpartan6: Select Binary file to send. (MTTY Shortcut Key F5)\r\n");
    putBuf(tty, tBuf,0);
    while(1)
       {
       //read binary data (with timeout)
        d8 = getBufBin();               //read binary data
        d16= (d8<<8);
        d8 = getBufBin();               //read binary data
        d16= (d16 + d8);                //make it upper byte
        if (g_Done)
            break;

        //RM48 Hercules device supports the little-endian [LE] format
        //Note; hardware is setup to put data on bus with reads, fpga stobes it in
        d16= *pFlash++;                  //parallel flash base address (uC hardware, chip sel 3)
                
        if (DONE0)                      //1=config done
            break;

        if (g_Cnt > 900000)
            {
            sprintf(tBuf,"LoadSpartan6: DONE Status Failed, Rec'd byte cnt= %d\r\n",g_Cnt);
            putBuf(tty, tBuf,0);
            break;
            }
        }

    if (DONE0)                          //1=config done. 
        {
        sprintf(tBuf,"LoadSpartan6: DONE0 Set High\r\n");
        putBuf(tty, tBuf,0);
        }

    //suggested to add 8 additional clocks
    //reads of 0xffff data (required) on additional clocks
    for (int i=0;i<8;i++)
        d16= *pFlash;      

    
    //DONE indicates configuration is complete. Can be held Low externally to
    //synchronize startup with other FPGAs. 
    CSI_B0_HI                           //set CSI_B0 high

    //set RDWR_B high after CSI_B0 set high. 
    RDWR_B_HI                       

    //Now check 'Done pin'
    if (DONE0)                          //config done
        {
        sprintf(tBuf,"LoadSpartan6: DONEx HIGH....LOAD PASSED\r\n");
        putBuf(tty, tBuf,0);        //show last message on load status now
        }
      
    if (USB_Rec.Cnt>4)
        {
        sprintf(Buf1500,"LoadSpartan6: Config Failed\r\n");
        sprintf(tBuf,"LoadSpartan6: Config Abort, dumping remaining data\r\n");
        strcat(Buf1500, tBuf);
        sprintf(tBuf,"LoadSpartan6: FPGA Config Failed\r\n");
        strcat(Buf1500, tBuf);
        putBuf(tty, Buf1500,0);
        }
    //iqnore data
    while (USB_Rec.Cnt)                 //in case ptrs get lost
        {
        uDelay(2000);
        _disable_interrupt_();
        USB_Rec.RdPtr=0;
        USB_Rec.WrPtr=0;
        USB_Rec.Cnt=0;
        _enable_interrupt_();
        }
    if (DONE0==0)                       //1=config done,  User now sees fpgas as 1-4.
        sprintf(tBuf,"LoadSpartan6: FPGA1 Config Failed, Rec'd byte cnt= %d\r\n",g_Cnt);
    else
        sprintf(tBuf,"LoadSpartan6: FPGA1 Config Okay, Rec'd byte cnt= %d\r\n",g_Cnt);
    putBuf(tty, tBuf,0);
    return 0;   
}


#define GET_DATA_TIMEOUT    60000*5     //Wait up to 3 min reference to mS counter

//read one byte binary data from input stream (download file)
int getBufBin()
{
    uint8_t d8;
    uint32_t tout;
    static uint32_t inCnt=0;
    if (g_Cnt==0)
      tout=GET_DATA_TIMEOUT;            //initial wait time, then exit if no input
    else
      tout=1000;                        //after data starts, timeout 1 mSec between chars

    if (inCnt < USB_Rec.Cnt)
        inCnt = USB_Rec.Cnt;   
    //wait (with timeout) while no char avail
    mStime.g_timeMs=0;
    while (USB_Rec.Cnt ==0)
        {
        if (tout < mStime.g_timeMs)     //max wait time, assume we are done at this point
            {
            g_Done=1;
            return 0;
            }
        }
    d8= USB_Rec.Buf[USB_Rec.RdPtr++];
    USB_Rec.Cnt--;
    g_Cnt++;
    
    //reset storage ptr if at end of buffer
    if (USB_Rec.RdPtr >= InBufSiz_512)
        USB_Rec.RdPtr   = 0;

    cSum +=d8;
    return d8;                                  //return byte count
}






//flash programming
//Programming is a four-bus-cycle operation. The program command sequence is initiated 
//by writing two unlock write cycles, followed by the program set-up command. The program
//address and data are written next, which in turn initiate the Embedded Program algorithm.

//Programming is allowed in any sequence and across sector boundaries. A bit cannot be 
//programmed from 0 back to a 1. Attempting to do so may cause that bank to set DQ5 = 1, 
//or cause the DQ7 and DQ6 status bits to indicate the operation was successful. However, 
//a succeeding read will show that the data is still 0. Only erase op can convert a 0 to a 1.


/* 
Sector erase is a six bus cycle operation. The sector erase command sequence 
is initiated by writing two unlock cycles, followed by a set-up command. Two 
additional unlock cycles are written, and are then followed by the address 
of the sector to be erased.

Chip Erase Word Commands
  add data
  555 AA 
  2AA 55 
  555 80
  555 AA
  2AA 55
  555 10


Chip  Program Word Commands
  add data
  555 AA 
  2AA 55 
  555 A0
  Add Data (program addr, data)  //program time typ 6uS
*/

//Ready/Busy... low (Busy), the device is actively erasing or programming
//Program/Erase Valid to RY/BY# Delay 90nS
int flashStatus(int prt)
{
    int ret;
    FLASH_WP_LO                     //FLASH_WP (LOW) (V6) HET1_5
    FlashRst_LO                     //reset to ready 35uS max
    uDelay(50);
    //FLASH RESET (keep high) (K18)HET1_0, 500nS low time
    FlashRst_HI                     //reset to ready 35uS max
    //If the output is low(Busy), the device is actively erasing or programming
    ret= FLASH_RDY;                 //read returns 0,1   FLASH RDY (V7) HET1_9

    //read flash info mode
    *(snvPTR) (flashBase+adr555)= 0xAA;
    *(snvPTR) (flashBase+adr2aa)= 0x55;
    *(snvPTR) (flashBase+adr555)= 0x90;

    //read manufacture ID
    tBuf[0]= *(snvPTR) (flashBase+0);          //addr  0
    tBuf[1]= *(snvPTR) (flashBase+2);          //addr 1   (*2)
    tBuf[2]= *(snvPTR) (flashBase+(0xE<<1));   //addr 0xE (*2)
    tBuf[3]= *(snvPTR) (flashBase+(0xF<<1));   //addr 0xF (*2)

    sprintf(Buf1500,"Flash ManId : %02x  (Good=01))\r\n", tBuf[0]);
    sprintf(tBuf,   "Flash DevId : %02x%02x%02x (Good=7e0201)\r\n", tBuf[1],tBuf[2],tBuf[3]);
    strcat (Buf1500,tBuf);
    putBuf(prt, Buf1500,0);
    
    FLASH_WP_LO                     //FLASH_WP (LOW) (V6) HET1_5
    FlashRst_LO                     //reset to ready 35uS max
    uDelay(50);
    //FLASH RESET (keep high) (K18)HET1_0, 500nS low time
    FlashRst_HI                      //reset to ready 35uS max
    return ret;
}



#include "reg_sci.h"  //using uart putchar direct
//Flash Load from USB port using binary data file
int loadFLASH(int ADDR, int prt)
{
    uint8_t d8;
    uint16_t d16;
    int stat, cont=1,var32;
    sPTR saddr;
    //Clear all counters
    cSum=0;
    g_Cnt=0;
    g_Done=0;
    
//tek, remove when fpga all work    
  //PROGx_LO                            //min 300nS
    
    while(FLASH_RDY==0)
        {
        putBuf(prt,"loadFLASH: Flash not ready\r\n",0); //send to current active port
        mDelay(1000);
        return 1;
        }
    sprintf(tBuf,"loadFLASH: Begin Load Flash\r\n");
    putBuf(prt, tBuf,0);
    
    //flash set unprotect mode, (V6) HET1_5
    FLASH_WP_HI
    saddr = (snvPTR) flashBase;
    saddr += ADDR;

    sPTR sdRam = (sPTR) fpgaBase0+0x407;
    
    while(cont==1)
       {
        if(prt==tty)
            {
           //read binary data (with timeout) and send to Altera Cyclone
            d8 = getBufBin();           //read binary data
            //revB_byte(d8);            //reverse bit order in Byte
            d16= (d8<<8);
            //if (g_Done)
            //    break;
            d8 = getBufBin();           //read binary data
            //d8= revB_byte(d8);        //reverse bit order in Byte
            d16= (d16 + d8);            //make it upper byte
            if (g_Done)
                break;
            }
        else
            {
            d16= *sdRam; //SDR_RD16SWP1;  //get data from fpga1 sdRam memory
            cSum += (d16>>8);
            cSum += d16&0xff;
            g_Cnt+=2;
            if (g_Cnt>=u_SRAM.DwnLd_sCNT)
                break;
            }

//tek, remove when fpga all work    
//PROGx_LO                              //min 300nS

        //RM48 Hercules device supports the little-endian [LE] format
        //reverse data done in sendFPGA(), send LSB first
        //d16= revB_byte(d16);            //reverse bit order

        *(snvPTR) (flashBase+adr555)= 0xAA;
        *(snvPTR) (flashBase+adr2aa)= 0x55;
        *(snvPTR) (flashBase+adr555)= 0xA0;
   // hHI_TP47        
        *saddr++= d16;
   // uDelay(1);
        stat=0;
        while(FLASH_RDY==0)
            {
            if (stat++ > 0xfffff)
                {
                putBuf(prt, "loadFLASH: Exit loader, Chip Not Erased Before Load???\r\n",0);
                cont=0;
                mDelay(1000);
                break;
                }
            }
        //if (DONE0 == 1)           //1=config done. (C2) GIOA1
        //    break;
        
        if (g_Cnt > 900000)
            {
            sprintf(tBuf,"loadFLASH: Load: Load Failed, Rec'd byte cnt= %d, CkSum= %X\r\n",g_Cnt,cSum&0xffff);
            putBuf(prt, tBuf,0);
            mDelay(1000);
            break;
            }
      //without echo download 'MTTY' shows green status line on progress
      //if ((g_Cnt % 0x3000)==0) 
      //    scilinREG->TD = '.'; //UART
        }
    CSI_B0_HI                           
    CSI_B1_HI                           
    CSI_B2_HI                           
    CSI_B3_HI                           
    
    sprintf(tBuf,"loadFLASH: End Load Flash,  Rec'd byte cnt= %d, CkSum= %X\r\n",g_Cnt,cSum&0xffff);
    putBuf(prt, tBuf,0);
    mDelay(100);
    if(ADDR==0)
        {
        FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_1_COUNT, (uint8_t*)&g_Cnt,4);    //write 4 bytes
        FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_1_CSUM, (uint8_t*)&cSum,4);      //write 4 bytes
        FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
        var32= DWNLD_VALID;
        FRAM_WR(DWNLD_1_VALID, (uint8_t*)&var32,4);    //write 4 bytes
        }
    else
        {
        FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_2_COUNT, (uint8_t*)&g_Cnt,4);    //write 4 bytes
        FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_2_CSUM, (uint8_t*)&cSum,4);      //write 4 bytes
        FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
        var32= DWNLD_VALID;
        FRAM_WR(DWNLD_2_VALID, (uint8_t*)&var32,4);    //write 4 bytes
        }
    
    //flashStatus(tty);
    mDelay(100);
    return cSum&0xFFFF;    
}




//flash load from USB port using binary data file
int eraseFLASH()
{
    int stat;
    sPTR saddr;
    cPTR caddr;
      
    //flash low (Busy), the device erasing or programming  
    stat= FLASH_RDY;                
    if (stat==0)
      return 1;     //busy then exit

    //flash set unprotect mode, (V6) HET1_5
    FLASH_WP_HI                     

    *(snvPTR) (flashBase+adr555)= 0xAA;
    *(snvPTR) (flashBase+adr2aa)= 0x55;
    *(snvPTR) (flashBase+adr555)= 0x20;

    *(snvPTR) (flashBase+adr555)= 0xAA;
    *(snvPTR) (flashBase+adr2aa)= 0x55;
    *(snvPTR) (flashBase+adr555)= 0x80;

    *(snvPTR) (flashBase+adr555)= 0xAA;
    *(snvPTR) (flashBase+adr2aa)= 0x55;
    *(snvPTR) (flashBase+adr555)= 0x10;

    uDelay(50);
    //wait for erase to finish, whole chip takes ~70 seconds
    while(FLASH_RDY==0)
        {
        putBuf(tty, ".",0);
        uDelay(1000000);
        }
    stat++;
    uDelay(50);
    //flash set to protect mode, (V6) HET1_5
    FLASH_WP_LO                     
    uDelay(50);
    putBuf(tty, "\r\n",0);

    //NULL status var
    stat=0;
    FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
    FRAM_WR(DWNLD_1_VALID, (uint8_t*)&stat,2); //write 2 bytes
    FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
    FRAM_WR(DWNLD_1_COUNT, (uint8_t*)&stat,4); //write 4 bytes
    FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
    FRAM_WR(DWNLD_1_CSUM, (uint8_t*)&stat,4);  //write 4 bytes

    FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
    FRAM_WR(DWNLD_2_VALID, (uint8_t*)&stat,2); //write 2 bytes
    FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
    FRAM_WR(DWNLD_2_COUNT, (uint8_t*)&stat,4); //write 4 bytes
    FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
    FRAM_WR(DWNLD_2_CSUM, (uint8_t*)&stat,4);  //write 4 bytes
    
    //ready flash
    flashStatus(tty);
    return 0;
}

//S29JL064J
//In the pre-programming step Erase algorithm, all bytes programmed to 00h 
//before erasure.(not included in time below)
//Chip Erase Time    71.0 Sec
//Sector Erase Time   0.5 Sec
//Chip Program Time  25.2 Sec
//Word Program Time   6.0 uSec

//#define SECTORES   50     
//#define SECTORES   40     //ERASE 500k Words, fpga file take 802,294 byts
#define SECTORE_SA8 0x8000  //fpga file load base address in parallel flash  (1st 32K sector map in chip)       
#define SECTORE_SA0 0       //fpga file load base address in parallel flash        
    
//flash load from USB port using binary data file
int eraseFLASH_Sector(int secCount, int start, int port)
{
    sPTR saddr;
    int sector=0, cnt=0;
    if(start> 0)
      cnt=9999;
    //flash low (Busy), the device erasing or programming  
    if (FLASH_RDY==0)       //Ready/Busy... low (Busy)      
      return 1;             //busy then exit
    sprintf(tBuf,"S29JL064J: Erasing %d Sectors\r\n",secCount);
    putBuf(port, tBuf,0);
    uDelay(500);
    sprintf(tBuf,"S29JL064J: First Sector Addr= %8X\r\n",(snvPTR)flashBase+ start);
    putBuf(port, tBuf,0);
    
    //flash set unprotect mode, (V6) HET1_5
    FLASH_WP_HI 
    uDelay(100);      
    saddr= (snvPTR)flashBase+ start;
    //send all erase commands 1st
    while (1)
        {
//tek, remove when fpga all work    
  //PROGx_LO                            //min 300nS
          
        //note: CYCLES SA7--SA8 HAS ADDRESS JUMP
        //could modify code to jump from SA7(0000000111) to SA7(0000001xxx)', it then skips 8 null cycles
          
        //set address
         if (cnt< 16)
           saddr += 0x1000/2;
         else if (cnt==9999)
            {
           saddr= (snvPTR)flashBase+ start;
           cnt=17;
            }
         else if (cnt==16)
           saddr= (snvPTR)flashBase+ 0x10000/2;   //this may not be needed, we are here after 16 cycles
         else if (cnt>16)
            saddr +=0x8000/2;
         cnt++; 
   
   
         //erase sector code sequence
        *(snvPTR) (flashBase+adr555)= 0xAA;
        *(snvPTR) (flashBase+adr2aa)= 0x55;
        *(snvPTR) (flashBase+adr555)= 0x80;
        *(snvPTR) (flashBase+adr555)= 0xAA;
        *(snvPTR) (flashBase+adr2aa)= 0x55;
        *saddr= 0x30;                           //erase command  
        
        //wait for erase to finish, whole chip takes ~70 seconds
        uDelay(300);
        while(FLASH_RDY==0)
            {
            uDelay(100);
            }
        putBuf(port, ".",1);
        if (sector++ > secCount)
            break;
        }
    //flash set to protect mode, (V6) HET1_5
    FLASH_WP_LO  

    //NULL cnt var
    //clear FRAM status on last FLASH load
    cnt=0;
    if(start==0)
        {
        FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_1_VALID, (uint8_t*)&cnt,2);  //write 2 bytes
        FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_1_COUNT, (uint8_t*)&cnt,4);  //write 4 bytes
        FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_1_CSUM, (uint8_t*)&cnt,4);   //write 4 bytes
        }
    else
        {
        FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_2_VALID, (uint8_t*)&cnt,2);  //write 2 bytes
        FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_2_COUNT, (uint8_t*)&cnt,4);  //write 4 bytes
        FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_2_CSUM, (uint8_t*)&cnt,4);   //write 4 bytes
        }
      
      
    sprintf(tBuf,"\r\nS29JL064J: Final Sector Addr= %8X\r\n",saddr);
    putBuf(port, tBuf,0);
     //ready flash
    return 0;
}


#define File_wSIZsm  780000/2       //binary file size in words (may be less than real size)   
#define File_wSIZlg  800000/2       //binary file size in words (may be less than real size)   
#define File_wSIZmax 880000/2       //binary file size in words worst case max   

#define D_SIZE16      8192          //16bitSiz (if ACCESS_16_BIT)
#define D_SIZE32      4096          //32bitSiz (if ACCESS_32_BIT)
#define D_SIZE64      2048          //64bitSiz (if ACCESS_64_BIT)

int flashXFER(int chip, int prt)
{
    sPTR sAddr= (sPTR) flashBase;  ////sAddr Actual S29JL064J address= 0x0 @Sector 0
    uint32_t cnt=0, sum=0, wrdCnt=0;
    int lp=0;
    uint16_t d16;
    uint32_t d32[2];                //returns max of 64 bit value

    //RDWR_B low before CSI_Bx, 0=Write to fpga    
    //keep low if read-back not needed
    RDWR_B_LO                           //(J4) HET1_23
      
    //Clear all counters
    cSum=0;
    g_Done=0;
    if(chip==0)
        {
        //get stored status
        FRAM_RD(DWNLD_1_VALID, (uint8_t*)&d16, 2);
        if (d16== DWNLD_VALID)
            {
            //get stored load count, 32bits
            FRAM_RD(DWNLD_1_COUNT, (uint8_t*)&g_Cnt, 4); 
            g_Cnt /=2;                      //bytes to word cnt
            }
        else
            g_Cnt= File_wSIZmax;
        }
    else
        {
        //get stored status
        FRAM_RD(DWNLD_2_VALID, (uint8_t*)&d16, 2); 
        if (d16== DWNLD_VALID)
            {
            //get stored load count, 32bits
            FRAM_RD(DWNLD_2_COUNT, (uint8_t*)&g_Cnt, 4); 
            g_Cnt /=2;                  //bytes to word cnt
            }
        else
            g_Cnt= File_wSIZmax;
        sAddr += (S29JL064J_SECTOR40);         //Actual S29JL064J address= 0x110000 @Sector 41   
        }

    
    //new hardware circuit, individual fpga(s) resets with PROB_B0
    switch (chip)                       //chip 1 of 4
        {
        case 0: PROG0_LO                //min 300nS
                uDelay(5);
                PROG0_HI                       
                break;
        case 1: PROG1_LO                //min 300nS
                uDelay(5);
                PROG1_HI                       
                break;
        case 2: PROG2_LO                //min 300nS
                uDelay(5);
                PROG2_HI                       
                break;
        case 3: PROG3_LO                //min 300nS
                uDelay(5);
                PROG3_HI                       
                break;
        case 4: PROGx_LO                //min 300nS
                uDelay(5);
                PROGx_HI                       
                break;
        }
    //after reset delay needed    
    uDelay(1500);               //fpga reset takes about 800uS
    
    switch (chip)               //chip 1 of 4
        {
        case 0:    if (DONE0)   //User now sees fpgas as 1-4
                        {
                        sprintf(tBuf,"LoadSpartan6: DONE1 ALREADY HIGH....Exit loader\r\n");
                        putBuf(prt, tBuf,0);
                        return 1;
                        }
                   break;
        case 1:    if (DONE1)
                        {
                        sprintf(tBuf,"LoadSpartan6: DONE2 ALREADY HIGH....Exit loader\r\n");
                        putBuf(prt, tBuf,0);
                        return 1;
                        }
                   break;
        case 2:    if (DONE2)
                        {
                        sprintf(tBuf,"LoadSpartan6: DONE3 ALREADY HIGH....Exit loader\r\n");
                        putBuf(prt, tBuf,0);
                        return 1;
                        }
                   break;
        case 3:    if (DONE3)
                        {
                        sprintf(tBuf,"LoadSpartan6: DONE4 ALREADY HIGH....Exit loader\r\n");
                        putBuf(prt, tBuf,0);
                        return 1;
                        }
                   break;
        }
     
    //InitBx should be HIGH at this point
    switch (chip)   //chip 1 of 4
        {           //User now sees fpgas as 1-4
        case 0:    if (InitB0_Read==0)                     //wait
                       {
                        sprintf(tBuf,"LoadSpartan6: FPGA1_Init Low Error? (expect High)...Exit loader\r\n");
                        putBuf(prt, tBuf,0);
                        return -1;                      //nStatus error status
                       }
                   break;
        case 1:    if (InitB1_Read==0)                     //wait
                       {
                        sprintf(tBuf,"LoadSpartan6: FPGA2_Init Low Error? (expect High)...Exit loader\r\n");
                        putBuf(prt, tBuf,0);
                        return -1;                      //nStatus error status
                       }
                   break;
        case 2:    if (InitB2_Read==0)                     //wait
                       {
                        sprintf(tBuf,"LoadSpartan6: FPGA3_Init Low Error? (expect High)...Exit loader\r\n");
                        putBuf(prt, tBuf,0);
                        return -1;                      //nStatus error status
                       }
                   break;
        case 3:    if (InitB3_Read==0)                     //wait
                       {
                        sprintf(tBuf,"LoadSpartan6: FPGA4_Init Low Error? (expect High)...Exit loader\r\n");
                        putBuf(prt, tBuf,0);
                        return -1;                      //nStatus error status
                       }
                   break;
        }
    //}//if chip4 loop end      

    //enable FPGA for programming (1 of 4)
    switch (chip)                       //chip 1 of 4
        {
        case 0:    CSI_B0_LO                           //set CSI_B0 low
                   break;
        case 1:    CSI_B1_LO                           //set CSI_B1 low
                   break;
        case 2:    CSI_B2_LO                           //set CSI_B2 low
                   break;
        case 3:    CSI_B3_LO                           //set CSI_B3 low
                   break;
        case 4:    CSI_B0_LO                           
                   CSI_B1_LO                           
                   CSI_B2_LO                           
                   CSI_B3_LO                           
                   break;
        }
    
    //The PROGRAM_B pin can be held active (Low) for as long as necessary, and the
    //device clears the configuration memory twice after PROGRAM_B is released.
    //PROGx_LO                        //min 300nS
    //uDelay(5);
    //PROGx_HI                        //
   
    //after CSI_Bx goes low wait for InitBx to return high
    switch (chip)                       //chip 1 of 4
        {
        case 0:    while(lp++ < 200000)
                          if (InitB0_Read==1) break;    //wait
                   break;
        case 1:    while(lp++ < 200000)
                          if (InitB1_Read==1) break;    //wait
                   break;
        case 2:    while(lp++ < 200000)
                        {
                          if (InitB2_Read==1) break;    //wait
                          if(lp++ < 1000) break;
                        }
                   break;
        case 3:    while(lp++ < 200000)
                          if (InitB3_Read==1) break;    //wait
                   break;
        case 4:    while(lp++ < 200000)
                          if (InitB0_Read==1) break;    //wait use chip 0 for this
                   break;
        }

    //timeout error (normal range is 4500 @220Mhz Processor Speed)
    if (lp >= 50000)
        {
        sprintf(tBuf,"LoadSpartan6: Timeout InitB%1d Post Reset Low Error? (expect High)...Exit loader\r\n",chip);
        putBuf(prt, tBuf,0);
        switch (chip)                       //chip 1 of 4
            {
            case 0:    CSI_B0_HI
                       break;
            case 1:    CSI_B1_HI
                       break;
            case 2:    CSI_B2_HI
                       break;
            case 3:    CSI_B3_HI
                       break;
            }
        return -1;                      //nStatus error status
        }
    //delay needed after reset, else load fails
    uDelay(100);
    *tBuf=0;                                    //NULL buffer in case no text entered below

#define USE_DMA     
#ifndef USE_DMA
    //non DMA Code version
    //read data blocks using movStr16_NOIC
    cnt= g_Cnt;
    //send most or all the data to fpga
    movStr16_NOINC((sPTR)flashBase, &d16, cnt);      
    sAddr += cnt;
    while (cnt++ <= g_Cnt+10)
        {
        //now send remaining data to fpga checking 'done pin' after each xfer
        d16= *sAddr++;
        sum+=d16;
        //if (cnt < File_wSIZsm)
        //  continue;
        //code continous after DMA snippet
        
#else
    //DMA Code version
    while (cnt < g_Cnt)
        {
        //DMA Max size is 8191 (elements) bytes (ref manual page 596), 
        //Current Transfer Count Register (CTCOUNT) [offset = 808h]
        //send DMA block until done, DMA size (4096 32bit xfers)
          
        dmaDisable();
        //assigning dma request: channel-0 with request line - 1
        dmaReqAssign(0,1 );
        //configuring dma control packets   (srcadd, destadd, datasize)
        dmaConfigCtrlPacket1((uint32)sAddr, (uint32)&d32, D_SIZE64 ); //read from flash base
        g_dmaCTRLPKT1.RDSIZE = ACCESS_64_BIT;	  //change read size to 64bits now
        g_dmaCTRLPKT1.ADDMODEWR = ADDR_FIXED;     //address mode write
        if((wrdCnt+D_SIZE16) > g_Cnt)
            {
            //lp variable == zero when done
            lp= g_Cnt- wrdCnt;
            wrdCnt += lp;                   //word counter
            cnt += lp;
            sAddr += lp;
            //done
            //suggested to add 8 additional clocks
            //reads of 0xffff data (required) on additional clocks
            //lp +=8;
            g_dmaCTRLPKT1.ELCNT= lp;        //Element / Frame
            g_dmaCTRLPKT1.RDSIZE = ACCESS_16_BIT;	  //change read size to 32bits now
            }
        else
            {
            wrdCnt += D_SIZE16;             //word counter
            cnt += D_SIZE16;
            sAddr += D_SIZE16;
            }
        //setting dma control packets, upto 32 control packets are supported
        dmaSetCtrlPacket(DMA_CH0,g_dmaCTRLPKT1);
        //setting the dma channel to trigger on software request
        dmaSetChEnable(DMA_CH0, DMA_SW);
        //enabling dma module
        dmaEnable();
        while ( dmaREG->DMASTAT & BIT1 != 1);        //1==CH ACTIVE
      //while(dmaGetInterruptStatus(DMA_CH0, BTC) != TRUE); //DONE STATUS
        //now send remaining data to fpga checking 'done pin' after each xfer
        //smaller xfer size is flag that have finished 
        if(g_dmaCTRLPKT1.ELCNT != lp)
            {
            d16= *sAddr++;
            sum+=d16;
            continue;
            }
#endif
        //Now check 'Done pin'
        if (DONE0 && (chip==0))                             //config done
            g_Cnt=0;                                        //loop break
        else if (DONE1 && (chip==1))                        //config done
            g_Cnt=0;                                        //loop break
        else if (DONE2 && (chip==2))                        //config done
            g_Cnt=0;                                        //loop break
        else if (DONE3 && (chip==3))                        //config done
            g_Cnt=0;                                        //loop break
        else if (DONE0 && (chip==4))                        //config done
            g_Cnt=0;                                        //loop break
        }
    
//#ifndef USE_DMA
    //suggested to add 8 additional clocks
    //reads of 0xffff data (required) on additional clocks
    for (int i=0;i<30;i++)
        d16= *sAddr++;                //additional clks to set done flag
//#endif    
    //DONE indicates configuration is complete. Can be held Low externally to
    //synchronize startup with other FPGAs. 
    //enable FPGA for programming (1 of 4), to speed it up, set all high (normal)
    CSI_B0_HI                           
    CSI_B1_HI                           
    CSI_B2_HI                           
    CSI_B3_HI                           
      
    //set RDWR_B high after CSI_B0 set high. 
    RDWR_B_HI                       //(J4) HET1_23

    //Now check 'Done pin',  User now sees fpgas as 1-4.
    if (DONE0 && (chip==0))                             //config done
        {
        sprintf(tBuf,"LoadSpartan6: DONE1 HIGH....LOAD PASSED\r\n");
        putBuf(prt, tBuf,0);    //show last message on load status now
        }
    else if (DONE1 && (chip==1))                        //config done
        {
        sprintf(tBuf,"LoadSpartan6: DONE2 HIGH....LOAD PASSED\r\n");
        putBuf(prt, tBuf,0);    //show last message on load status now
        }
    else if (DONE2 && (chip==2))                        //config done
        {
        sprintf(tBuf,"LoadSpartan6: DONE3 HIGH....LOAD PASSED\r\n");
        putBuf(prt, tBuf,0);    //show last message on load status now
        }
    else if (DONE3 && (chip==3))                        //config done
        {
        sprintf(tBuf,"LoadSpartan6: DONE4 HIGH....LOAD PASSED\r\n");
        putBuf(prt, tBuf,0);    //show last message on load status now
        }
    else if (DONE0 && (chip==4))                        //config done
        {
        sprintf(tBuf,"LoadSpartan6: DONEx HIGH....LOAD(s) PASSED\r\n");
        putBuf(prt, tBuf,0);    //show last message on load status now
        }

    if (USB_Rec.Cnt>4)
        {
        sprintf(Buf1500,"LoadSpartan6: Config Failed\r\n");
        sprintf(tBuf,"LoadSpartan6: Config Abort, dumping remaining data\r\n");
        strcat(Buf1500, tBuf);
        sprintf(tBuf,"LoadSpartan6: FPGA Config Failed\r\n");
        strcat(Buf1500, tBuf);
        putBuf(prt, Buf1500,0);
        }
    while (USB_Rec.Cnt>4)              //in case ptrs get lost
        {
        uDelay(2000);
        _disable_interrupt_();
        USB_Rec.RdPtr=0;
        USB_Rec.WrPtr=0;
        USB_Rec.Cnt=0;
        _enable_interrupt_();
        }
    if (g_Cnt)
        sprintf(tBuf,"LoadSpartan6: **CONFIG FAILED** WordCnt=%d, ByteCnt=%d\r\n",cnt, cnt*2);
    else
        sprintf(tBuf,"LoadSpartan6: ByteCnt=%d\r\n", cnt*2);
    putBuf(prt, tBuf,0);
    return 0;   
}


/** void dmaConfigCtrlPacket(uint32 sadd,uint32 dadd,uint32 dsize)
*   configuring dma control packet stack
*       sadd  > source address
*       dadd  > destination  address
*       dsize > data size
*   @ note : after configuring the stack the control packet needs to be set by calling dmaSetCtrlPacket()
*  note  The frame/element count fields of the the  ITCOUNT register is only 13 bit wide, 
*  note  hence a max of 8191 frames/elemets. bits 15:13 are ignored - so 8192 is same as 0.
*/
void dmaConfigCtrlPacket1(uint32 sadd,uint32 dadd,uint32 dsize)
{
  g_dmaCTRLPKT1.SADD      = sadd;			   /* source address             */
  g_dmaCTRLPKT1.DADD      = dadd;			   /* destination  address       */
  g_dmaCTRLPKT1.CHCTRL    = 0;                 /* channel control            */
  g_dmaCTRLPKT1.FRCNT	  = 1;                 /* frame count                */
  g_dmaCTRLPKT1.ELCNT     = dsize;             /* Element / Frame            */
  g_dmaCTRLPKT1.ELDOFFSET = 0;                 /* element destination offset */
  g_dmaCTRLPKT1.ELSOFFSET = 0;		           /* element destination offset */
  g_dmaCTRLPKT1.FRDOFFSET = 0;		           /* frame destination offset   */
  g_dmaCTRLPKT1.FRSOFFSET = 0;                 /* frame destination offset   */
  g_dmaCTRLPKT1.PORTASGN  = 4;                 /* assign dma #               */
  g_dmaCTRLPKT1.RDSIZE    = ACCESS_16_BIT;	   /* read size                  */
  g_dmaCTRLPKT1.WRSIZE    = ACCESS_16_BIT; 	   /* write size                 */
  g_dmaCTRLPKT1.TTYPE     = FRAME_TRANSFER ;   /* transfer type              */
  g_dmaCTRLPKT1.ADDMODERD = ADDR_INC1;         /* address mode read          */
  g_dmaCTRLPKT1.ADDMODEWR = ADDR_FIXED;        /* address mode write         */
//g_dmaCTRLPKT1.ADDMODEWR = ADDR_OFFSET;       /* address mode write         */
  g_dmaCTRLPKT1.AUTOINIT  = AUTOINIT_ON;       /* autoinit                   */
}


const uint32 s_het1pwmPolarity[8U] = { 3U, 3U, 3U, 3U, 3U, 3U, 3U, 3U, };
hetSIGNAL_t setup_pwm;
void pwmSetDuty_full(hetRAMBASE_t * hetRAM, uint32 pwm, uint32 fuse_pwmDuty,uint32 pwmdutty);

//TEST CODE, MAY DELETE AT SOME POINT, TEK OCT 2016
void pwmSetDuty_full(hetRAMBASE_t * hetRAM, uint32 pwm, uint32 fuse_pwmDuty, uint32 pwmdutty)
{
    uint32 action;
    uint32 pwmPolarity = 0U;
    float var;

    if (hetRAM == hetRAM1) 
        {
        pwmPolarity = s_het1pwmPolarity[pwm];
        }

    if (fuse_pwmDuty == 0U) 
        {
        action = (pwmPolarity == 3U) ? 0U : 2U;
        } 
    else if (fuse_pwmDuty >= 100U) 
        {
        action = (pwmPolarity == 3U) ? 2U : 0U;
        } 
    else
        {
        action = pwmPolarity;
        }
    var= (uint32) pwmdutty * 0.86; //((pwmPeriod * pwmDuty) / 100U)

    hetRAM->Instruction[(pwm << 1U) + 41U].Control = ((hetRAM->Instruction[(pwm
    << 1U) + 41U].Control) & (~(uint32) (0x00000018U)))
    | (action << 3U);

    hetRAM->Instruction[(pwm << 1U) + 41U].Data = ((uint32)var << 7U) + 128U;
}





//rec data file from tty or sock port, store in 1of3 FPGA SDRAM
//Load File to sdRam
int LDF(int prt, int fpga, char* eBufB_Sock)               //rec data file, store in FPGA_1 SDRAM
{
    uint8_t d8;
    uint16 key, d16;
    int len, count=0, sum=0, offset;
    extern int g_Done, g_Cnt;
    char* eBufB= eBufB_Sock; //(char*)eRecDatBuf[g_Sock];
    
    //fpga1 upper level fpga, has no sdRam
    //set write address for sdRam(s) 1,2,3 in actual fpga(2,3,4) 
    if(fpga==1)
        {offset=0x000; }        //chip offset  1of4 fpgas
    if(fpga==2)
        {offset=0x400*2; }      //*** using REG16() so double addr here ***
    else if(fpga==3)
        {offset=0x800*2; }      //*** using REG16() so double addr here ***
    else if(fpga==4)
        {offset=0xC00*2; }      //*** using REG16() so double addr here ***

    //set write sdRAM Addr via special sequence
    SET_SDADDR_WRx(offset/2,0,0);   //addr gets double at function, divide here 1st

    if (prt==tty)
        {
        sprintf(tBuf,"SDram: waiting for data file (10 seconds) ...\r\n");
        putBuf(prt, tBuf,0);
        g_Cnt=0;        //must zero for 1st chr wait to work
        g_Done=0;       //must zero to prevent early exit
        while(1)
           {
            //read binary data (with timeout) and send to Altera Cyclone
            d16 = getBufBin();              //read usb data port
            if (g_Done)
                break;
            d8 = getBufBin();               //read usb data port
            //add up chksum
            sum += d16;
            sum += d8;
            //bytes->word
            d16 <<= 8;
            d16 += d8;
            count+=2;                            
            //set write fpga Addr to sdRam2,3,4 (no sdRam0)
            REG16((fpgaBase0+offset)+(0x07*2))= d16;  //SDR_RD16 (non swap data) 
           }
        }
    else
        {
        sprintf(tBuf,"SDram: waiting for data file (10 seconds)...  (uTelnet use key '|')\r\n");
        putBuf(prt, tBuf,0);
        //wait for data
        len= SockKeyWait(60000, prt, &key); //CharCnt=SockKeyWait(mSecWait, Port#, 1st Char in RecBuf)
        if(len==0)
            {
            sprintf(tBuf,"uC_Dram: Len=0, 'RDF' wait on file 60 Second timeout\r\n");
            putBuf(prt, tBuf,0);
            }
        else while(len>0)
            {
            if(len==1)
                {
                count++;
                d16= *eBufB++; //store 1st of two words
                //get more data
                len= SockKeyWait(500, prt, &key); //CharCnt=SockKeyWait(mSecWait, Port#, 1st Char in RecBuf)
                if(len==0)
                    {
                    sprintf(tBuf,"SDram: Len=0\r\n");
                    putBuf(prt, tBuf,0);
                    break;
                    }
                //eBufW= (uSHT*)eRecDatBuf[g_Sock];
                eBufB= eBufB_Sock;  //(char*)eRecDatBuf[g_Sock];
                d8= *eBufB++;       //store 2nd of two words
                sum += d16;
                sum += d8;
                d16= (d16<<8)+d8;
            //set write fpga Addr to sdRam2,3,4 (no sdRam0)
            REG16((fpgaBase0+offset)+(0x07*2))= d16;  //SDR_RD16 (non swap data) 
                count++;
                len--;
                }
            else
                {
                d16= *eBufB++;  //get byte
                d8 = *eBufB++;  //get byte
                sum += d16;
                sum += d8;
                d16= (d16<<8)+d8;
                
            //set write fpga Addr to sdRam2,3,4 (no sdRam0)
            REG16((fpgaBase0+offset)+(0x07*2))= d16;  //SDR_RD16 (non swap data)                   
                count+=2;
                len -=2;
                }
            if(len==0)
                {
                len= SockKeyWait(500, prt, &key); //CharCnt=SockKeyWait(mSecWait, Port#, 1st Char in RecBuf)
                //eBufW= (uSHT*)eRecDatBuf[g_Sock];
                eBufB= eBufB_Sock; //(char*)eRecDatBuf[g_Sock];
                }
            }
        }
    //send extra bytes to force SDram Buffer Burst Write Cycle
    for(int i=0; i<32; i++)
        {
        REG16((fpgaBase0+offset)+(0x07*2))= 0x1234; 
        uDelay(1);
        }
    u_SRAM.DwnLd_sCNT= count;
    u_SRAM.DwnLd_sSUM= sum&0xffff;
    
    //allow writes to finish
    uDelay(1);
    //set read sdRAM Addr via special sequence
    SET_SDADDR_RDx(offset/2,0,0);   //addr gets double at function, divide here 1st  
    sprintf(tBuf,"uC_Dram: Recd %d Bytes, ChkSum=%04X\r\n", count,sum&0xffff);
    putBuf(prt, tBuf,0);
    return count;
}                    



extern struct      uSums uPhySums;

//*****************  BEGIN NET LDFILE *****************************************************
//*****************************************************************************************
//*****************************************************************************************
//Naming as used on FEB
//sum= loadFLASH(S29JL064J_SECTOR0,holdprt);  //Actual S29JL064J address= 0x0 @Sector 0
//int loadFLASH(int ADDR, int prt) USE THIS NAME SAME AS 

int loadFLASH_SOCK(int prt, char* eBufB_Sock, int fpga)  //rec data file, pgm flash for fpga1or2
{
    uint8_t d8;
    uint16 d16, key;   
    int len, stat;
    extern int g_Done, g_Cnt;
    char* eBufB= eBufB_Sock; //(char*)eRecDatBuf[g_Sock];
    int valid= DWNLD_VALID;                     //always valid if we get here
    
    sPTR saddr;    
    saddr = (snvPTR) flashBase;
    if(fpga==1)
        saddr += S29JL064J_SECTOR0;         //fpga1
    else                        
        saddr += S29JL064J_SECTOR40;        //fpga2
    
    //sum=loadFLASH(S29JL064J_SECTOR40, prt);   //Actual S29JL064J address= 0x110000 @Sector 41
	//sum=loadFLASH(S29JL064J_SECTOR0, prt);    //Actual S29JL064J address= 0x0 @Sector 0

    //flash set unprotect mode, (V6) HET1_5
    FLASH_WP_HI
    
    sprintf(tBuf,"FLASH_PGM: Select Binary file. TeraTerm MENU File,SendFile Bin[yes]\r\n");
    putBuf(prt, tBuf,0);
    sprintf(tBuf,"FLASH_PGM: TeraTerm may not work sending Binary files, check online\r\n");
    putBuf(prt, tBuf,0);          
    sprintf(tBuf,"FLASH_PGM: Waiting for data file (60 seconds)...  (uTelnet use key '|')\r\n");
    putBuf(prt, tBuf,0);
    //wait for data
    len= SockKeyWait(60000, prt, &key); //CharCnt=SockKeyWait(mSecWait, Port#, 1st Char in RecBuf)
    if(len==0)
        {
        sprintf(tBuf,"FLASH_PGM: Len=0, 'RDF' wait on file 60 Second timeout\r\n");
        putBuf(prt, tBuf,0);
        }
    else while(len>0)
        {
        if(len==1)
            {
            uPhySums.FL_SOCK_CHKSIZE++;
            d16= *eBufB++; //store 1st of two words
            
            //get more data
            len= SockKeyWait(500, prt, &key); //CharCnt=SockKeyWait(mSecWait, Port#, 1st Char in RecBuf)
            if(len==0)
                {
                sprintf(tBuf,"SDram: Len=0\r\n");
                putBuf(prt, tBuf,0);
                break;
                }
            //eBufW= (uSHT*)eRecDatBuf[g_Sock];
            eBufB= eBufB_Sock;  //(char*)eRecDatBuf[g_Sock];
            d8= *eBufB++;       //store 2nd of two words
            uPhySums.FL_SOCK_CHKSUM += d16;
            uPhySums.FL_SOCK_CHKSUM += d8;
            d16= (d16<<8)+d8;
         
            //*********************************************
            // PGM FLASH ONE WORD AT A TIME     
            //*********************************************
            //RM48 Hercules device supports the little-endian [LE] format
            //reverse data done in sendFPGA(), send LSB first
            //d16= revB_byte(d16);            //reverse bit order
            *(snvPTR) (flashBase+adr555)= 0xAA;
            *(snvPTR) (flashBase+adr2aa)= 0x55;
            *(snvPTR) (flashBase+adr555)= 0xA0;
            *saddr++= d16;
            stat=0;
            while(FLASH_RDY==0)
                {
                if (stat++ > 0xfffff)
                    {
                    putBuf(prt, "loadFLASH: Exit loader, Chip Not Erased Before Load???\r\n",0);
                    len=0;      //force break out
                    mDelay(1000);
                    break;
                    }
                }
           //*********************************************
           // PGM FLASH END
           //*********************************************
            uPhySums.FL_SOCK_CHKSIZE++;
            len--;
            }
        else
            {
            d16= *eBufB++;  //get byte
            d8 = *eBufB++;  //get byte                
            uPhySums.FL_SOCK_CHKSUM += d16;
            uPhySums.FL_SOCK_CHKSUM += d8;
            d16= (d16<<8)+d8;                
            //*********************************************
            // PGM FLASH ONE WORD AT A TIME     
            //*********************************************
            //RM48 Hercules device supports the little-endian [LE] format
            //reverse data done in sendFPGA(), send LSB first
            //d16= revB_byte(d16);            //reverse bit order
            *(snvPTR) (flashBase+adr555)= 0xAA;
            *(snvPTR) (flashBase+adr2aa)= 0x55;
            *(snvPTR) (flashBase+adr555)= 0xA0;
            *saddr++= d16;
            stat=0;
            while(FLASH_RDY==0)
               {
                if (stat++ > 0xfffff)
                   {
                    putBuf(prt, "loadFLASH: Exit loader, Chip Not Erased Before Load???\r\n",0);
                    len=0;      //force break out
                    mDelay(1000);
                    break;
                    }
                }
           //*********************************************
           // PGM FLASH END
           //*********************************************
             uPhySums.FL_SOCK_CHKSIZE+=2;
             len -=2;
            }
        if(len==0)
            {
            len= SockKeyWait(500, prt, &key); //CharCnt=SockKeyWait(mSecWait, Port#, 1st Char in RecBuf)
            eBufB= eBufB_Sock; //(char*)eRecDatBuf[g_Sock];
            }
        
        u_SRAM.DwnLd_sCNT= uPhySums.FL_SOCK_CHKSIZE;
        u_SRAM.DwnLd_sSUM= uPhySums.FL_SOCK_CHKSUM &0xffff;
        }
   
    if(fpga==1)
        {    
        FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_1_VALID, (uint8_t*)&valid,2);     //write 2 bytes
        FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_1_COUNT, (uint8*)&u_SRAM.DwnLd_sCNT,4);    //write 4 bytes
        FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_1_CSUM, (uint8*)&u_SRAM.DwnLd_sSUM,4);      //write 4 bytes
        }
    else
        {    
        FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_2_VALID, (uint8_t*)&valid,2);     //write 2 bytes
        FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_2_COUNT, (uint8*)&u_SRAM.DwnLd_sCNT,4);    //write 4 bytes
        FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
        FRAM_WR(DWNLD_2_CSUM, (uint8*)&u_SRAM.DwnLd_sSUM,4);      //write 4 bytes
        }
    
    sprintf(tBuf,"FLASH_PGM: Recd %d Bytes, ChkSum=%04X\r\n", u_SRAM.DwnLd_sCNT, u_SRAM.DwnLd_sSUM);
    putBuf(prt, tBuf,0);
    return uPhySums.FL_SOCK_CHKSIZE;
}                    
//*****************  END NET LDFILE *******************************************************
//*****************************************************************************************
//*****************************************************************************************




extern struct   HappyBusReg HappyBus;

//data pool update
int g_DataPoolReq(int prt)
{
    //request data pool update 'LSTAB'
    //if not busy   
    return 0;
}


//********************************************************
//  @file sys_mu2e_misc.c
//  Fermilab Terry Kiper 2016-2020
//  RM48 Code
//  RM48 Hercules device supports little-endian [LE] format
//
//  Program Xilinx Spartan 6 fpga
//  Program and Read Spansion Inc S29JL064J 64 Megabit 
//                  (8M x 8-Bit/4M x 16-Bit) Flash Memory
//  Program and Read FRAM memory chip 'FM25CL64B'
//********************************************************


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
//it is powering up. Once it is ready to to clear the configuration latches, 
//PROGRAM_B and then INIT_B will be "released" so that they can float high. 
//They are in fact driven by the external pull-up resistor. The Micro can then 
//prolong this stage by driving either PROGRAM_B or INIT_B low.

//Spartan-6 FPGA Configuration, page 36
//Configuration memory is cleared sequentially any time the device is powered up, after the
//PROGRAM_B pin is pulsed Low. 
//INIT_B is internally driven Low during initialization.
//If the INIT_B pin is held Low externally, the device waits at this point 
//in the initialization process until the pin is released.
//The minimum Low pulse time for PROGRAM_B is defined by the TPROGRAM timing
//parameter. The PROGRAM_B pin can be held active (Low) for as long as necessary, and the
//device clears the configuration memory twice after PROGRAM_B is released.

//Spartan-6 FPGA Configuration, page 87
//In a Slave configuration mode, additional clocks are needed after DONE goes High to
//complete the startup events. In Master configuration mode, the FPGA provides these
//clocks. The number of clocks necessary varies depending on the settings selected for the
//startup events. A general rule is to apply eight clocks (with DIN all 1’s) after DONE has
//gone High. More clocks are necessary if the startup is configured to wait for the DCM and
//PLLs to lock (LCK_CYCLE).


/*
Spansion S29JL064J 64Megabit (8M x 8-Bit/4M x 16-Bit) Flash Memory

Bank1  Sector   SectorAddr A21–A12      Addr16 (not byte addr)
        SA0         0000000000          00000h–00FFFh    SA0-SA7   8K-BYTES  (08x)
        SA22        0001111xxx          78000h–7FFFFh    SA8-SA22 64K-BYTES  (15x)
        total 983,025 bytes
          

Bank2  Sector   SectorAddr A21–A12      Addr16 (not byte addr)
        SA23        0010000xxx          80000h–87FFFh       SA23-SA70 64K-BYTES  
        SA40        0100001xxx          108000h–10FFFFh
        SA41        0100010xxx          110000h–117FFFh
        SA70        0111111xxx          1F8000h–1FFFFFh    
        total 3,145,680 bytes

Bank3  Sector   SectorAddr A21–A12      Addr16 (not byte addr)
        SA71        1000000xxx          200000h–207FFFh     SA71-SA118 64K-BYTES
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
#include "ver_io.h"
#include "het.h"
#include "gio.h"

#include "spi.h"
#include "hw_reg_access.h"
#include "sys_core.h"
#include "sys_dma.h"
#include "reg_sci.h"

#include "sys_mu2e.h"
#include "sys_mu2e_misc.h"
#include "sys_mu2e_functions.h"
#include "sys_mu2e_FRAM.h"


//extern buffers, vars
extern  char    Buf1500[];
extern  char    tBuf[];
extern  char    bBuf1000[];             //stores boot up messages
extern  uint32  g_timeMs;
extern  uint32  volatile iFlag;
extern  uint32  genFlag;

//extern struct
extern  struct  vB USB_Rec;
extern struct   msTimers mStime; 

extern struct   uSums u_SRAM;
extern struct   uSums uPhySums;
extern struct   HappyBusReg HappyBus;
extern struct   uC_Store uC_FRAM;


//struct
g_dmaCTRL   g_dmaCTRLPKT1; 

//vars
int     g_Cnt, g_Done, cSum;


//fpga load direct from USB port using binary data file
int loadSpartan6_FPGA(int chip)
{
    sPTR pFlash= pFLASHbase;             //short ptr, parallel flash base address
    uint8_t d8;
    uint16_t d16;
    //Clear all counters
    cSum=0;
    g_Cnt=0;
    g_Done=0;
    
    //If the Suspend feature is not used, the SUSPEND pin must be connected to gnd.
    Suspend_LO                          //set suspend low, not used

    //set RDWR_B low before CSI_B0, 0=Write to fpga      
    RDWR_B_LO                           //(J4) HET1_23

      //enable FPGA for programming (1 of 4)
    CSI_B0_LO                           //set CSI_B0 low

    //The PROGRAM_B pin can be held active (Low) for as long as necessary, and the
    //device clears the configuration memory twice after PROGRAM_B is released.
    PROGx_LO                            //min 300nS
    uDelay(5);
    //Hold the INIT_B pin Low during initialization. When INIT_B has gone High,
    //configuration cannot be delayed
    PROGx_HI
    uDelay(10);
      
    //If the INIT_B pin is held Low externally, the device waits at this point in 
    //the initialization process until the pin is released.	
    uDelay(10);
    if (InitB0_Read==1)                 //if INIT not LOW, Error. (A3) HET1_29
        {
        CSI_B0_HI                       //set CSI_B0 high
        sprintf(tBuf,"LoadSpartan6 : InitB High Error? (expect High)\r\n");
        putBuf(tty, tBuf,0);
        return -1;                      //nStatus error status
        }

    //SetInitB0_OutPut                    //Config pin (A3) HET1_29 as OutPut
    //send data fpga
    sprintf(tBuf,"LoadSpartan6 : Select Binary file to send. (MTTY Shortcut Key F5)\r\n");
    putBuf(tty, tBuf,0);
    while(1)
       {
       //read binary data (with timeout) and send to Altera Cyclone
        d8 = getBufBin();               //read binary data
        d16= revB_byte(d8);             //reverse bit order
        if (g_Done)
            break;
        
        d8 = getBufBin();               //read binary data
        d8= revB_byte(d8);              //reverse bit order
        d16= d16 + (d8<<8);               //make it upper byte
        if (g_Done)
            break;

        //RM48 Hercules device supports the little-endian [LE] format
        //reverse data done in sendFPGA(), send LSB first
        //d16= revB_byte(d16);            //reverse bit order
        *pFlash= d16;                   //parallel flash base address (uC hardware, chip sel 3)
        d16=*pFlash;
        pFlash++;
                
        if (DONE0)                      //1=config done. (C2) GIOA1
            break;

        if (g_Cnt > 900000)
            {
            sprintf(tBuf,"LoadSpartan6 : DONE Status Failed, Rec'd byte cnt= %d\r\n",g_Cnt);
            putBuf(tty, tBuf,0);
            break;
            }
        }
    //reset usb flow control
    DSR_LO                                    //LOW enables data flow

    for (int i=0;i<100;i++)
        {
        if (DONE0)                  //1=config done. (C2) GIOA1
            break;
        *pFlash++;                    //additional clks to set done flag
        }

    if (DONE0)                      //1=config done. (C2) GIOA1
        {
        sprintf(tBuf,"LoadSpartan6 : DONE0 Set High\r\n");
        putBuf(tty, tBuf,0);
        }
    for (int i=0; i<10; i++)
        *pFlash++;                    //additional clks to set done flag

    
    //DONE indicates configuration is complete. Can be held Low externally to
    //synchronize startup with other FPGAs. 
    CSI_B0_HI                       //set CSI_B0 high. (C1) GIOA2
    //set RDWR_B high after CSI_B0 set high. 
    //RDWR_B_HI                       //(J4) HET1_23

    if (USB_Rec.Cnt>4)
        {
        sprintf(Buf1500,"LoadSpartan6 : Config Failed\r\n");
        sprintf(tBuf,"LoadSpartan6 : Config Abort, dumping remaining data\r\n");
        strcat(Buf1500, tBuf);
        sprintf(tBuf,"LoadSpartan6 : FPGA Config Failed\r\n");
        strcat(Buf1500, tBuf);
        putBuf(tty, Buf1500,0);
        }
    //iqnore data
    while (USB_Rec.Cnt)              //in case ptrs get lost
        {
        uDelay(2000);
        _disable_interrupt_();
        USB_Rec.RdPtr=0;
        USB_Rec.WrPtr=0;
        USB_Rec.Cnt=0;
        _enable_interrupt_();
        }
    if (DONE0==0)                     //1=config done. (C2) GIOA1
        sprintf(tBuf,"LoadSpartan6 : FPGA Config Failed ???\r\nRec'd byte cnt= %d\r\n",g_Cnt);
    else
        sprintf(tBuf,"LoadSpartan6 : FPGA Config Okay ***\r\nRec'd byte cnt= %d\r\n",g_Cnt);
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
      tout=GET_DATA_TIMEOUT;                //initial 45 second wait, then exit if no input
    else
      tout=1000;                            //after data starts, timeout 1 mSec between chars

    if (inCnt < USB_Rec.Cnt)
        inCnt = USB_Rec.Cnt;   
    //wait (with timeout) while no char avail
    mStime.g_timeMs=0;
    while (USB_Rec.Cnt ==0)
        {
        if (tout < mStime.g_timeMs)         //max wait time, assume we are done at this point
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
    return d8;                              //return byte count
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
    
    FLASH_WP_LO                     //FLASH_WP (LOW) (V6) HET1_5
    FlashRst_LO                     //reset to ready 35uS max
    uDelay(50);
    //FLASH RESET (keep high) (K18)HET1_0, 500nS low time
    FlashRst_HI                      //reset to ready 35uS max
      
    sprintf(Buf1500,"FlashMan : %02x  (Good=01)\r\n\r", tBuf[0]);
    sprintf(tBuf,   "FlashDev : %02x%02x%02x (Good=7e0201)\r\r\n", tBuf[1],tBuf[2],tBuf[3]);
    strcat (Buf1500,tBuf);
    //due to slower link debug monitor, use putBuf() later after more strcat()s
    //putBuf(prt, Buf1500,0);
    return ret;
}



#include "reg_sci.h"  //using uart putchar direct
//Flash Load from USB port using binary data file
//or from stored data in sdRAM
int loadFLASH(int ADDR, int prt)
{
    uint8_t d8;
    uint16_t d16;
    int stat, cont=1;
    sPTR saddr;
    //Clear all counters
    cSum=0;
    g_Cnt=0;
    g_Done=0;
        
    
    while(FLASH_RDY==0)
        {
        putBuf(prt,"Flash not ready\r\n",0); 
        u_SRAM.FL_PGM=1;                //1==failed
        return 1;
        }
    //flash set unprotect mode, (V6) HET1_5
    FLASH_WP_HI
    saddr = (snvPTR) flashBase;
    saddr += ADDR;

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
            //RM48 Hercules device supports the little-endian [LE] format
            //reverse data done in sendFPGA(), send LSB first
            //d16= revB_byte(d16);            //reverse bit order
            *(snvPTR) (flashBase+adr555)= 0xAA;
            *(snvPTR) (flashBase+adr2aa)= 0x55;
            *(snvPTR) (flashBase+adr555)= 0xA0;
            *saddr++= d16;
            }
        else
            {
            d16= SDR_RD16;              //fpga1 of 1of4 sdRam memory 0x07
            cSum += (d16>>8);
            cSum += d16&0xff;
            g_Cnt+=2;            
            *(snvPTR) (flashBase+adr555)= 0xAA;
            *(snvPTR) (flashBase+adr2aa)= 0x55;
            *(snvPTR) (flashBase+adr555)= 0xA0;
            *saddr++= d16;            
            if (g_Cnt>=u_SRAM.DwnLd_sCNT)
                break;
            }

        stat=0;
        while(FLASH_RDY==0)
            {
            if (stat++ > 0xfffff)
                {
                putBuf(tty, "FLASH: Exit loader, Chip Not Erased Before Load???\r\n",0);
                cont=0;
                //mDelay(200);
                break;
                }
            }
        
        if (g_Cnt > 900000)
            {
            sprintf(tBuf,"FLASH: Load: Load Failed, Rec'd byte cnt= %d, CkSum= %X\r\n",g_Cnt,cSum);
            putBuf(tty, tBuf,0);
           // mDelay(200);
            break;
            }
      //without echo download 'MTTY' shows colored status line on progress
      //if ((g_Cnt % 0x3000)==0) 
      //    scilinREG->TD = '.'; //UART
        }


    //flush fpga sdram controller buffer
    for(int i=0;i<16;i++)
        {
        *(snvPTR) (flashBase+adr555)= 0xAA;
        *(snvPTR) (flashBase+adr2aa)= 0x55;
        *(snvPTR) (flashBase+adr555)= 0xA0;
        *saddr++= d16;  
        stat=0;
        while(FLASH_RDY==0)
            {
            if (stat++ > 0xfffff)
                {
                putBuf(tty, "FLASH: Exit loader, Chip Not Erased Before Load???\r\n",0);
                cont=0;
                //mDelay(200);
                break;
                }
            }            
        }

    if (g_Cnt < 700000)
        {
        u_SRAM.FL_PGM=1;                            //1==failed
        sprintf(tBuf,"S29JL064J: Load Failed, File size small.\r\n");
        }
    else
        {
        u_SRAM.FL_PGM=2;                            //2==good
        sprintf(tBuf,"S29JL064J: End of File\r\nS29JL064J: Load Okay\r\nS29JL064J: ByteCnt+Pads= %d, CkSum= %X\r\n",g_Cnt,(cSum&0xffff));
        }
    putBuf(tty, tBuf,0);
    FRAM_WR_ENABLE();                              //WREN op-code, issued prior to Wr_Op
    FRAM_WR(MAIN_ADDR_COUNT, (uint8*)&g_Cnt,4);    //write 4 bytes
    FRAM_WR_ENABLE();                              //WREN op-code, issued prior to Wr_Op
    FRAM_WR(MAIN_ADDR_CSUM, (uint8*)&cSum,4);      //write 4 bytes
    FRAM_WR_ENABLE();                              //WREN op-code, issued prior to Wr_Op
    d16= DWNLD_VALID;
    FRAM_WR(MAIN_ADDR_VALID, (uint8*)&d16,4);      //write 4 bytes
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
    FRAM_WR(MAIN_ADDR_VALID, (uint8*)&stat,2); //write 2 bytes
    FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
    FRAM_WR(MAIN_ADDR_COUNT, (uint8*)&stat,4); //write 4 bytes
    FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
    FRAM_WR(MAIN_ADDR_CSUM, (uint8*)&stat,4);  //write 4 bytes
    
    //ready flash
    //flashStatus(tty);
    return 0;
}


//'ver_io.h' defines SECTORES   40     //ERASE 500k Words, fpga file take 802,294 byts
#define SECTORE_SA8 0x8000  //fpga file load base address in parallel flash  (1st 32K sector map in chip)       
#define SECTORE_SA0 0       //fpga file load base address in parallel flash        
    
//Flash Sector(s) Erase
int eraseFLASH_Sector(int secCount, int start, int prt)
{
    sPTR saddr;
    int sector=0, cnt=0, hAddr;
    if(start> 0)
      cnt=9999;
    
    //flash set unprotect mode, (V6) HET1_5
    FLASH_WP_HI                     
    
    //flash low (Busy), the device erasing or programming  
    if (FLASH_RDY==0)       //Ready/Busy... low (Busy)      
      return 1;             //busy then exit
    
    //simply wrd addr for display
    hAddr= ((int)flashBase+ start)&0xffffff;
    sprintf(Buf1500,"S29JL064J: Erasing %d Sectors\r\n\r",secCount);
    sprintf(tBuf,"S29JL064J: First Sector Addr= %-6X\r\n\r", hAddr);
    strcat(Buf1500,tBuf);
    if(prt!=ePhyIO)
        putBuf(prt, tBuf,0);
    
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
         //tek jan2018 why is this needed ???
         //todo, add print statement or trap on this condition           
            }
         else if (cnt==16)
           saddr= (snvPTR)flashBase+ 0x10000/2;  //this may not be needed, we are here after 16 cycles
         else if (cnt>16)
            saddr +=0x8000/2;
         cnt++; 
   
     
         //erase sector code 
        *(snvPTR) (flashBase+adr555)= 0xAA;
        *(snvPTR) (flashBase+adr2aa)= 0x55;
        *(snvPTR) (flashBase+adr555)= 0x80;
        *(snvPTR) (flashBase+adr555)= 0xAA;
        *(snvPTR) (flashBase+adr2aa)= 0x55;
        // saddr=   (snvPTR)(flashBase;       //sector address
        *saddr= 0x30;    
        
        uDelay(300);
        //wait for erase to finish, whole chip takes ~70 seconds
        while(FLASH_RDY==0)
            {
            uDelay(25);
            }
        genFlag |= NO_ECHO;                 //no ASCII reply  
        if(prt!=ePhyIO)
            putBuf(prt, ".",1);
        
        if (sector++ >= secCount)
            break;
        }
    //flash set to protect mode, (V6) HET1_5
    FLASH_WP_LO  

    //NULL cnt var
    cnt=0;
      
    
  //tek Oct2018 check this code out, secCount==0 ???  
    //if erasing upper flash dont clear vars used on lower flash
    if(secCount==0)
        {
        //clear FRAM status on last FLASH load
        FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
        FRAM_WR(MAIN_ADDR_VALID, (uint8*)&cnt,2);   //write 2 bytes
        FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
        FRAM_WR(MAIN_ADDR_COUNT, (uint8*)&cnt,4);   //write 4 bytes
        FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
        FRAM_WR(MAIN_ADDR_CSUM, (uint8*)&cnt,4);    //write 4 bytes
        }
    hAddr= (int)saddr>>2;           //simply wrd addr for display    
    hAddr= (hAddr&0xffffff) + start;//simply wrd addr for display
    sprintf(tBuf,"\r\nS29JL064J: Final Sector Addr= %6X\r\n\r",hAddr);
    if(prt!=ePhyIO)
        putBuf(prt, tBuf,0);
    return 1;       //1==Okay
}



#define File_wSIZsm  780000/2       //binary file size in words (may be less than real size)   
#define File_wSIZlg  800000/2       //binary file size in words (may be less than real size)   
#define File_wSIZmax 880000/2       //binary file size in words worst case max   

#define D_SIZE16      8192          //16bitSiz (if ACCESS_16_BIT)
#define D_SIZE32      4096          //32bitSiz (if ACCESS_32_BIT)
#define D_SIZE64      2048          //64bitSiz (if ACCESS_64_BIT)

int flashXFER(int chip, int prt, int base)
{
    sPTR saddr= (sPTR) flashBase;
    uint32_t cnt=0, sum=0, wrdCnt=0;
    int lp=0, done=0;
    uint16_t d16;
    uint32_t d32[2];                //returns max of 64 bit value

    //get data from flash base addr or backup addr
    if(base==0)
      saddr += S29JL064J_SECTOR41;  //SECTOR41, Actual S29JL064J address= 0x110000 @Sector 41
      
    //Clear all counters
    cSum=0;
    g_Done=0;
    
    //load min explected file size
    g_Cnt= File_wSIZmax;
    FRAM_RD(MAIN_ADDR_VALID, (uint8*)&d16, 2);  //get stored status
    if (d16== DWNLD_VALID)
        {
        FRAM_RD(MAIN_ADDR_COUNT, (uint8*)&g_Cnt, 4); //get stored load count, 32bits
        g_Cnt /=2;                          //bytes to word cnt
        }
    //add check just in case
    if (g_Cnt < File_wSIZsm)
        g_Cnt= File_wSIZmax;
    
    //new hardware circuit, individual fpga(s) resets with PROB_B0
    switch (chip)                           //chip 1 of 4
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
    uDelay(2000);                       //fpga reset takes about 800uS
    
    if(chip==4)
    for(int i=0;i<4;i++)
    {
    switch (chip)                       //chip 1 of 4
        {
        case 0:    if (DONE0)
                        {
                        sprintf(tBuf,"Spartan6 : DONE0 ALREADY HIGH....Exit loader\r\n");
                        putBuf(prt, tBuf,0);
                        return 1;
                        }
                   break;
        case 1:    if (DONE1)
                        {
                        sprintf(tBuf,"Spartan6 : DONE1 ALREADY HIGH....Exit loader\r\n");
                        putBuf(prt, tBuf,0);
                        return 1;
                        }
                   break;
        case 2:    if (DONE2)
                        {
                        sprintf(tBuf,"Spartan6 : DONE2 ALREADY HIGH....Exit loader\r\n");
                        putBuf(prt, tBuf,0);
                        return 1;
                        }
                   break;
        case 3:    if (DONE3)
                        {
                        sprintf(tBuf,"Spartan6 : DONE3 ALREADY HIGH....Exit loader\r\n");
                        putBuf(prt, tBuf,0);
                        return 1;
                        }
                   break;
        }
    }//if chip4 loop end
    
    //RDWR_B low before CSI_Bx, 0=Write to fpga    
    //keep low if read-back not needed
    RDWR_B_LO                           //(J4) HET1_23
      
    //InitBx should be HIGH at this point
    if(chip==4)
    for(int i=0;i<4;i++)
    {
    switch (chip)                       //chip 1 of 4
        {
        case 0:    if (InitB0_Read==0)                     //wait
                       {
                        sprintf(tBuf,"Spartan6 : InitB0 Low Error? (expect High)\r\n");
                        putBuf(prt, tBuf,0);
                        return -1;                      //nStatus error status
                       }
                   break;
        case 1:    if (InitB1_Read==0)                     //wait
                       {
                        sprintf(tBuf,"Spartan6 : InitB1 Low Error? (expect High)\r\n");
                        putBuf(prt, tBuf,0);
                        return -1;                      //nStatus error status
                       }
                   break;
        case 2:    if (InitB2_Read==0)                     //wait
                       {
                        sprintf(tBuf,"Spartan6 : InitB2 Low Error? (expect High)\r\n");
                        putBuf(prt, tBuf,0);
                        return -1;                      //nStatus error status
                       }
                   break;
        case 3:    if (InitB3_Read==0)                     //wait
                       {
                        sprintf(tBuf,"Spartan6 : InitB3 Low Error? (expect High)\r\n");
                        putBuf(prt, tBuf,0);
                        return -1;                      //nStatus error status
                       }
                   break;
        }
    }//if chip4 loop end      

    //enable FPGA for programming (1 of 4)
    switch (chip)                                       //chip 1 of 4
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
    //PROGx_HI             
   
    //after CSI_Bx goes low wait for InitBx to return high
    switch (chip)                                       //chip 1 of 4
        {
        case 0:    while(lp++ < 100000)
                          if (InitB0_Read==1) break;    //wait
                   break;
        case 1:    while(lp++ < 100000)
                          if (InitB1_Read==1) break;    //wait
                   break;
        case 2:    while(lp++ < 100000)
                          if (InitB2_Read==1) break;    //wait
                   break;
        case 3:    while(lp++ < 100000)
                          if (InitB3_Read==1) break;    //wait
                   break;
        case 4:    while(lp++ < 100000)
                          if (InitB0_Read==1) break;    //wait use chip 0 for this
                   break;
        }

    //timeout error (normal range is 4500 @220Mhz Processor Speed)
    if (lp >= 50000)
        {
        sprintf(tBuf,"Spartan6 : Timeout InitBx Post Reset Low Error? (expect High)\r\n");
        putBuf(prt, tBuf,0);
        return -1;                      //nStatus error status
        }
    //delay needed after reset, else load fails
    uDelay(100);
    *tBuf=0;                                    //NULL buffer in case no text entered below

#define USE_DMA     
#ifndef USE_DMA
    //non DMA Code version 75mSec 800KB
    //read data blocks using movStr16_NOICDEST
    cnt= g_Cnt;
    //send most or all the data to fpga
    movStr16_NOICDEST((sPTR)saddr, &d16, cnt);      
    saddr += cnt;
    while (cnt++ <= g_Cnt+10)
        {
        //now send remaining data to fpga checking 'done pin' after each xfer
        d16= *saddr++;
        sum+=d16;
        //if (cnt < File_wSIZsm)
        //  continue;
        //code continous after DMA snippet
        
#else
    //DMA Code version 45mSec 800KB
    while (cnt < g_Cnt)
        {
        //DMA Max size is 8191 (elements) bytes (ref manual page 596), 
        //Current Transfer Count Register (CTCOUNT) [offset = 808h]
        //send DMA block until done, DMA size (4096 32bit xfers)
          
        dmaDisable();
        //assigning dma request: channel-0 with request line - 1
        dmaReqAssign(0,1 );
        //configuring dma control packets   (srcadd, destadd, datasize)
        dmaConfigCtrlPacket1((uint32)saddr, (uint32)&d32, D_SIZE64 ); //read from flash base
        g_dmaCTRLPKT1.RDSIZE = ACCESS_64_BIT;	  //change read size to 32bits now
        g_dmaCTRLPKT1.ADDMODEWR = ADDR_FIXED;     //address mode write
        if((wrdCnt+D_SIZE16) > g_Cnt)
            {
            //lp variable == zero when done
            lp= g_Cnt- wrdCnt;
            wrdCnt += lp;                   //word counter
            cnt += lp;
            saddr += lp;
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
            saddr += D_SIZE16;
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
            d16= *saddr++;
            sum+=d16;
            continue;
            }
#endif
        //Now check 'Done pin'
        if (DONE0 && (chip==0))                             //config done
            done=1;                                         //loop break
        else if (DONE1 && (chip==1))                        //config done
            done=1;                                         //loop break
        else if (DONE2 && (chip==2))                        //config done
            done=1;                                         //loop break
        else if (DONE3 && (chip==3))                        //config done
            done=1;                                         //loop break
        else if (DONE0 && (chip==4))                        //config done
            done=1;                                         //loop break
        }
    
//#ifndef USE_DMA
    //suggested to add 8 additional clocks
    //reads of 0xffff data (required) on additional clocks
    for (int i=0;i<8;i++)
        d16= *saddr;                //additional clks to set done flag
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

    //Now check 'Done pin'
    if (DONE0 && (chip==0))                             //config done
        {
        sprintf(tBuf,"Spartan6 : DONE0 HIGH....LOAD PASSED\r\n");
        putBuf(prt, tBuf,0);    //show last message on load status now
        }
    else if (DONE1 && (chip==1))                        //config done
        {
        sprintf(tBuf,"Spartan6 : DONE1 HIGH....LOAD PASSED\r\n");
        putBuf(prt, tBuf,0);    //show last message on load status now
        }
    else if (DONE2 && (chip==2))                        //config done
        {
        sprintf(tBuf,"Spartan6 : DONE2 HIGH....LOAD PASSED\r\n");
        putBuf(prt, tBuf,0);    //show last message on load status now
        }
    else if (DONE3 && (chip==3))                        //config done
        {
        sprintf(tBuf,"Spartan6 : DONE3 HIGH....LOAD PASSED\r\n");
        putBuf(prt, tBuf,0);    //show last message on load status now
        }
    else if (DONE0 && (chip==4))                        //config done
        {
        sprintf(tBuf,"Spartan6 : DONEx HIGH....LOAD(s) PASSED\r\n");
        putBuf(prt, tBuf,0);    //show last message on load status now
        }

    if (USB_Rec.Cnt>4)
        {
        sprintf(Buf1500,"Spartan6 : Config Failed\r\n");
        sprintf(tBuf,"Spartan6 : Config Abort, dumping remaining data\r\n");
        strcat(Buf1500, tBuf);
        sprintf(tBuf,"Spartan6 : FPGA Config Failed\r\n");
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
    //clr iFlag bits first
    iFlag &= ~(CONFIGFAIL | CONFIGBACKUP);
    if (done==0)
        {
        sprintf(tBuf,"Spartan6 : **CONFIG FAILED** WordCnt=%d, ByteCnt=%d\r\n\r",cnt, cnt*2);
        putBuf(prt, tBuf,0);        //extra char if odd byte cnt on 16bit LVDS xfer
        iFlag |= CONFIGFAIL;
        return -1;
        }
    else
        {
        sprintf(tBuf,"Spartan6 : ByteCnt+Pads= %d\r\n\r", cnt*2); //in DMA mode we get no checksum to show
        putBuf(prt, tBuf,0);        //extra char if odd byte cnt on 16bit LVDS xfer
        //store sum and count for backup flash useage
        FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
        FRAM_WR(BACKUP_WCOUNT, (uint8*)&g_Cnt,4);       //write 4 bytes
        
        //check and flag if using backup flash code
        if(base==FLBACKUP)
            {
            sprintf(tBuf,"Spartan6 : FLASH ERROR, USING OLDER BACKUP CONFIG FILE\r\n\r");
            putBuf(prt, tBuf,0);        //extra char if odd byte cnt on 16bit LVDS xfer
            iFlag |= CONFIGBACKUP;
            }
      
        }
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
  g_dmaCTRLPKT1.SADD      = sadd;			  /* source address             */
  g_dmaCTRLPKT1.DADD      = dadd;			  /* destination  address       */
  g_dmaCTRLPKT1.CHCTRL    = 0;                 /* channel control            */
  g_dmaCTRLPKT1.FRCNT	  = 1;                 /* frame count                */
  g_dmaCTRLPKT1.ELCNT     = dsize;             /* Element / Frame            */
  g_dmaCTRLPKT1.ELDOFFSET = 0;                 /* element destination offset */
  g_dmaCTRLPKT1.ELSOFFSET = 0;		          /* element destination offset */
  g_dmaCTRLPKT1.FRDOFFSET = 0;		          /* frame destination offset   */
  g_dmaCTRLPKT1.FRSOFFSET = 0;                 /* frame destination offset   */
  g_dmaCTRLPKT1.PORTASGN  = 4;                 /* assign dma #               */
  g_dmaCTRLPKT1.RDSIZE    = ACCESS_16_BIT;	  /* read size                  */
  g_dmaCTRLPKT1.WRSIZE    = ACCESS_16_BIT; 	  /* write size                 */
  g_dmaCTRLPKT1.TTYPE     = FRAME_TRANSFER ;   /* transfer type              */
  g_dmaCTRLPKT1.ADDMODERD = ADDR_INC1;         /* address mode read          */
  g_dmaCTRLPKT1.ADDMODEWR = ADDR_FIXED;        /* address mode write         */
//g_dmaCTRLPKT1.ADDMODEWR = ADDR_OFFSET;       /* address mode write         */
  g_dmaCTRLPKT1.AUTOINIT  = AUTOINIT_ON;       /* autoinit                   */
}




//LOAD DATA FILE from TTY or SOCK PORT, store in FPGA SDRAM
//
int LDF(int prt, int fpga, char* eBufB_Sock)               //rec data file, store in SDRAM
{
    uint8_t d8;
    int key, d16, wait=30000;
    int len, count=0, sum=0, offset=0;
    extern int g_Done, g_Cnt;
    char* eBufB= eBufB_Sock; //(char*)eRecDatBuf[g_Sock];
    
    //add in if fifo reads fail 'tek Apr2017'
    //REG16((f_Addr_CSR)+fPtrOffset2)=0x11; //sdram MIG+rxFFI0 reset
    //mDelay(100);
    
       
    if(fpga==1)
        {offset=0x000; }     //chip offset  10f4 fpgas
    if(fpga==2)
        {offset=0x400*2; }     
    else if(fpga==3)
        {offset=0x800*2; }   
    else if(fpga==4)
        {offset=0xC00*2; }   

    //set write sdRAM Addr via special sequence
    SET_SDADDR_WRx(offset,0,0);
    
   // uSHT statusMIG = (uSHT)fpgaBase+fPtrOffset1+ fPtrSDramCnt;
    uSHT* sdAddr= (uSHT*)(fpgaBase+offset)+ (0x09);
    
    if (prt==ePhyIO)
        wait= 10000;
     
    if (prt==tty)
        {
        sprintf(tBuf,"FEBsRAM: waiting for data file (45seconds) ...\r\n");
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
            //set write fpga sdRam2
            REG16((fpgaBase+offset)+(0x07*2))= d16;   //SDR_RD16 (non swap data) 
           }
        }
    else
        {
        if (prt!=ePhyIO)
            {
            sprintf(tBuf,"FEBsRAM: waiting for data file (45seconds)...  (uTelnet use key '|')\r\n");
            putBuf(prt, tBuf,0);
            }
        if (prt!=ePhyIO)
            //wait for data
            len= SockKeyWait(wait, prt, &key); //CharCnt=SockKeyWait(mSecWait, Port#, 1st Char in RecBuf)
        else
            len= PHY_LEN(wait);

        if(len==0)
            {
            sprintf(tBuf,"FEBsRAM: TimeOut Rec'd Bytes=0, timeout\r\n");
            putBuf(prt, tBuf,0);
            return 0;
            }
        else while(len>0)
            {
            if(len==1)
                {
                count++;
                d16= *eBufB++; //store 1st of two words
                //get more data
                if (prt!=ePhyIO)
                    len= SockKeyWait(500, prt, &key); //CharCnt=SockKeyWait(mSecWait, Port#, 1st Char in RecBuf)
                else
                    len= PHY_LEN(1000);  
                if(len==0)
                    {
                    sprintf(tBuf,"FEBsRAM: Done\r\n");
                    putBuf(prt, tBuf,0);
                    break;
                    }
                //eBufW= (uSHT*)eRecDatBuf[g_Sock];
                eBufB= eBufB_Sock; //(char*)eRecDatBuf[g_Sock];
                d8= *eBufB++;   //store 2nd of two words
                sum += d16;
                sum += d8;
                d16= (d16<<8)+d8;
                //write fpga2 
                REG16((fpgaBase+offset)+(0x07*2))= d16;   //SDR_RD16 (non swap data) 
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
                //write fpga2 
                REG16((fpgaBase+offset)+(0x07*2))= d16;   //SDR_RD16 (non swap data) 
                count+=2;
                len -=2;
                }
            if(len==0)
                {
                //get more data
                if (prt!=ePhyIO)
                    len= SockKeyWait(500, prt, &key); //CharCnt=SockKeyWait(mSecWait, Port#, 1st Char in RecBuf)
                else
                    len= PHY_LEN(500);  
                eBufB= eBufB_Sock; //(char*)eRecDatBuf[g_Sock];
                }
             //wait for load ~20 seconds
            //if(count%20000==0)
            //    putBuf(prt, "..",2);
            }
        u_SRAM.DwnLd_sCNT= count;
        u_SRAM.DwnLd_sSUM= sum&0xffff;
        
        //commom rountine 'LDFLASH_FromSDram used for programming must set 'uPhySums'
        //uPhySums normally used for file xfers from FEB Controller
        uPhySums.DwnLd_sCNT= count;
        uPhySums.DwnLd_sSUM= sum&0xffff; 
        }
    
   //statusMIG
    sdAddr= (uSHT*)(fpgaBase+offset)+ (0x09);
    //send extra bytes to force SDram Buffer Burst Write Cycle
    for(int i=0; i<32; i++)
        {
        REG16((fpgaBase+offset)+(0x07*2))= 0x1234;       
        if ((*sdAddr & 0xff)==0) //check burst fifo
            i=100;
        }
    //allow writes to finish
    uDelay(1);

    //set read sdRAM Addr via special sequence
    SET_SDADDR_RDx(offset,0,0);
    sprintf(tBuf,"FEBsRAM: Recd %d Bytes, ChkSum=0x%04X\r\n", count,sum&0xffff);
    putBuf(prt, tBuf,0);
    return 0;
}                    



extern struct      uSums uPhySums;

//*****************  BEGIN NET LDFILE *****************************************************
//*****************************************************************************************
//*****************************************************************************************
//Naming as used on FEB
//sum= loadFLASH(S29JL064J_SECTOR0,holdprt);  //Actual S29JL064J address= 0x0 @Sector 0
//int loadFLASH(int ADDR, int prt) USE THIS NAME SAME AS 

int loadFLASH_NEW(int prt, char* eBufB_Sock)               //rec data file, store in FPGA_1 SDRAM
{
    uint8_t d8;
    uint16_t d16; 
    int len, key, stat;
    extern int g_Done, g_Cnt;
    char* eBufB= eBufB_Sock; //(char*)eRecDatBuf[g_Sock];
    
    
    sPTR saddr;    
    saddr = (snvPTR) flashBase;
    saddr += S29JL064J_SECTOR0;
    
    //sum=loadFLASH(S29JL064J_SECTOR40, prt);   //Actual S29JL064J address= 0x110000 @Sector 41
	//sum=loadFLASH(S29JL064J_SECTOR0, prt);    //Actual S29JL064J address= 0x0 @Sector 0

    //flash set unprotect mode, (V6) HET1_5
    FLASH_WP_HI
    
        {
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
              //count++;
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

                //sum += d16;
                //sum += d8;
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
                
              //count++;
                uPhySums.FL_SOCK_CHKSIZE++;
                len--;
                }
            else
                {
                d16= *eBufB++;  //get byte
                d8 = *eBufB++;  //get byte
                
                uPhySums.FL_SOCK_CHKSUM += d16;
                uPhySums.FL_SOCK_CHKSUM += d8;
              //sum += d16;
              //sum += d8;
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
                //count+=2;
                uPhySums.FL_SOCK_CHKSIZE+=2;
                len -=2;
                }
            if(len==0)
                {
                len= SockKeyWait(500, prt, &key); //CharCnt=SockKeyWait(mSecWait, Port#, 1st Char in RecBuf)
                eBufB= eBufB_Sock; //(char*)eRecDatBuf[g_Sock];
                }
            }
        u_SRAM.DwnLd_sCNT= uPhySums.FL_SOCK_CHKSIZE;
        u_SRAM.DwnLd_sSUM= uPhySums.FL_SOCK_CHKSUM &0xffff;
        }
   
    FRAM_WR_ENABLE();                              //WREN op-code, issued prior to Wr_Op
    FRAM_WR(MAIN_ADDR_COUNT, (uint8*)&u_SRAM.DwnLd_sCNT,4);    //write 4 bytes
    FRAM_WR_ENABLE();                              //WREN op-code, issued prior to Wr_Op
    FRAM_WR(MAIN_ADDR_CSUM, (uint8*)&u_SRAM.DwnLd_sSUM,4);      //write 4 bytes
    
    sprintf(tBuf,"FLASH_PGM: Recd %d Bytes, ChkSum=%04X\r\n", u_SRAM.DwnLd_sCNT, u_SRAM.DwnLd_sSUM);
    putBuf(prt, tBuf,0);
    return uPhySums.FL_SOCK_CHKSIZE;
}                    
//*****************  END NET LDFILE *******************************************************
//*****************************************************************************************
//*****************************************************************************************







//********************************************************
//  @file sys_mu2e_FlashCopy.c
//  Fermilab Terry Kiper 2016-2020
//  RM48 Code
//  RM48 Hercules device supports little-endian [LE] format
//
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
        SA40        0100001xxx          108000h–10FFFFh
        SA41        0100010xxx          110000h–117FFFh     // SA41--SA70= EFFFF or 983,039(Dec) Words
        SA70        0111111xxx          1F8000h–1FFFFFh     //
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

//#define S29JL064J_SECTOR0 0             //Actual S29JL064J ADR=0x0 @Sector 0
//#define S29JL064J_SECTOR41 (0x220000/2) //Actual S29JL064J ADR=0x110000 @Sector 41   



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
extern  uint32  g_timeMs;
extern  uint32  genFlag;

//extern struct
extern struct   uSums uPhySums;
extern struct   uC_Store uC_FRAM;
extern struct   HappyBusReg HappyBus;


//Make backup copy FPGA Flash Memory in Upper Sectors of same FLASH Chip
//Flash lower buffer at address 0x000000, use read data cmd 'RFI 000000' to view
//Flash upper buffer at address wrd addr 0x110000, use read data cmd 'RFI 110000' to view
//copies base FLASH memory to Upper FLASH memory
int fl_copyFPGA(int prt)
{
    int d16, stat, dcount, DwnLd_sSUM=0, DwnLd_sCNT=0;
    sPTR saddrBASE, saddrUPPER;
    //Clear all counters
        
    //get stored fpga file size from FRAM
    FRAM_RD(MAIN_ADDR_COUNT,(uint8*)&dcount, 4);
    dcount /=2;

   //NOTE PROMPTS GETS SENT BACK ON LVDS ON EXIT HERE   
    genFlag |= NO_ECHO;                 //no ASCII reply                      
    sprintf(tBuf,"S29JL064J: Image Backup, Busy 22 sec\r\n\r");
    putBuf(prt, tBuf,0);
    if(prt!=ePhyIO)
        flashStatus(prt);                 //displays status
    uDelay(100);                   
    //erase upper flash memory starting at sector 41
    eraseFLASH_Sector(40, S29JL064J_SECTOR41, prt); //Actual S29JL064J address= 0x110000 @Sector 41

   //flash set unprotect mode, (V6) HET1_5
    FLASH_WP_HI
    saddrBASE = (snvPTR) flashBase;
    saddrUPPER = (snvPTR) flashBase+ S29JL064J_SECTOR41;

    //loop on 'DWNLD_1_COUNT'
    while(dcount--)
        {
        //RM48 Hercules device sequence, 4 writes todo 1 word
        *(snvPTR) (flashBase+adr555)= 0xAA;
        *(snvPTR) (flashBase+adr2aa)= 0x55;
        *(snvPTR) (flashBase+adr555)= 0xA0;
        d16= *saddrBASE++;
        *saddrUPPER++= d16;
        
        stat=0;
        while(FLASH_RDY==0)             //will fail if not erased
            {
            uDelay(10);  
            if (stat++ > 20000)
                {
                genFlag |= NO_ECHO;     //no ASCII reply                      
                putBuf(prt, "loadFLASH: Error, Chip Not Erased Before Load???\r\n",0);
                stat=0;
                break;
                }
            }
        if (stat==0)
            break;
        }
    
    //now do local check to verify
    //compute checkSum for valid data   
    saddrUPPER = (snvPTR) flashBase+ S29JL064J_SECTOR41;
    //get stored fpga backup file size from FRAM
    FRAM_RD(BACKUP_WCOUNT,(uint8*)&dcount, 4);
   
    for (int i=0; i<dcount; i++)
        {
        d16= *saddrUPPER++;             //data port, using fpga(2of4) sdRam
        DwnLd_sSUM += ((d16>>8)&0xff);
        DwnLd_sSUM += (d16&0xff);
        DwnLd_sCNT+=2;
        }      
    FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
    FRAM_WR(BACKUP_BYTES, (uint8*)&DwnLd_sCNT,4);   //write 4 bytes
    FRAM_WR_ENABLE();                               //WREN op-code, issued prior to Wr_Op
    FRAM_WR(BACKUP_CSUM, (uint8*)&DwnLd_sSUM,2);    //write 2 bytes
        
    sprintf(Buf1500,"Done, WrdCnt=%d  CheckSum=%4X\r\n\r", DwnLd_sCNT, DwnLd_sSUM&0xffff);
    if(prt!=ePhyIO)
        putBuf(prt, Buf1500,0);
    return 0;
}
   

extern struct   uSums u_SRAM;
#include "reg_sci.h"  //using uart putchar direct


//Flash Load from stored data in sdRAM
//int loadFLASH(int ADDR, int prt)
int loadToFLA_FAKE(int ADDR, int wSize, int prt)
{
    uint16_t d16;
    int stat, holdprt;
    int w_Cnt=0, cSum=0;
    
    sPTR saddr=(snvPTR) flashBase;
    saddr += ADDR;
    
    if(prt==ePhyIO)                 //flag to show status on tty of ePhyIO requesting
      prt= tty;    
     
    //flash set unprotect mode, (V6) HET1_5
    FLASH_WP_HI

    while(FLASH_RDY==0)
        {
        if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
            putBuf(tty,"Flash not ready\r\n",0); 
        u_SRAM.FL_PGM=1;                //1==failed here
        return 0;                   //0==failed here
        }
    
    wSize = wSize>>1;               //doing 16bit reads here
    while(1)
       {
        d16= SDR_RD16;              //fpga1 of 1of4 sdRam memory 0x07
        cSum += (d16>>8);
        cSum += d16&0xff;
        w_Cnt++;            
        *(snvPTR) (flashBase+adr555)= 0xAA;
        *(snvPTR) (flashBase+adr2aa)= 0x55;
        *(snvPTR) (flashBase+adr555)= 0xA0;
        *saddr++= d16;            

        stat=0;
        while(FLASH_RDY==0)
            {
            if (stat++ > 0xfffff)
                {
                sprintf(tBuf,"loadToFLA_FAKE: Exit loader, Chip Not Erased? Addr=%X\r\n", saddr);   
                if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
                    putBuf(prt, tBuf,0);
                return 0;   //failed
                }
            }
        wSize--;
        if(wSize<=0)
            break;
        }

    if (w_Cnt < 30000)  //loaded word count
        {
        u_SRAM.FL_PGM=1;                            //1==failed
        sprintf(Buf1500,"S29JL064J: Load Failed, File size small.\r\n");
        }
    else
        {
        u_SRAM.FL_PGM=2;                            //2==good
        sprintf(Buf1500,"S29JL064J: End of File\r\n");
        sprintf(tBuf,"S29JL064J: Load Okay\r\n");
        strcat(Buf1500, tBuf);
        sprintf(tBuf,"S29JL064J: ByteCnt=%d, CkSum=%X\r\n",w_Cnt<<1,(cSum&0xffff));
        strcat(Buf1500, tBuf);
        stat= (int)saddr>>1;
        stat &=0xffffff;
        sprintf(tBuf,"S29JL064J: WordCnt=%d, End WrdAddr=%X\r\n", w_Cnt, stat);
        strcat(Buf1500, tBuf);
        }
    if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout       
        putBuf(prt, Buf1500,0);
    //FRAM_WR_ENABLE();                              //WREN op-code, issued prior to Wr_Op
    //FRAM_WR(MAIN_ADDR_COUNT, (uint8*)&g_Cnt,4);    //write 4 bytes
    //FRAM_WR_ENABLE();                              //WREN op-code, issued prior to Wr_Op
    //FRAM_WR(MAIN_ADDR_CSUM, (uint8*)&cSum,4);      //write 4 bytes
    //FRAM_WR_ENABLE();                              //WREN op-code, issued prior to Wr_Op
    //d16= DWNLD_VALID;
    //FRAM_WR(MAIN_ADDR_VALID, (uint8*)&d16,4);      //write 4 bytes
    return cSum&0xFFFF;    
}




//Load (*.bin file ) to Flash using FPGAx_sdRamx data, assume cmd 'RDF'  or 'LDFILE' already got file
//Check for valid data in sdRam
//If fpga is not configured this function wont work, needs sdRam access via fpga logic
//If valid data erase flash and program flash with new sdRam data, reboot fpga section, 'DREC'
int PgmFlash_FromSDramData(int prt)
{
    uint16_t holdprt=prt, len;
    int sum=0;
        
    if(prt==ePhyIO)     //show status messages on tty of ePhyIO requesting
      prt= tty;    
    
    //Valid download Check
    u_SRAM.FL_XFER= 0;
    if((u_SRAM.DwnLd_sCNT==uPhySums.DwnLd_sCNT) && (u_SRAM.DwnLd_sSUM==uPhySums.DwnLd_sSUM)&& (uPhySums.DwnLd_sCNT !=0)) //match original file specs
        {
        u_SRAM.FL_XFER= 1;    
        sprintf(Buf1500,"FEB  : DownLoad Verify Size and CheckSum Good..\r\n");
        }
    else
        {
        //invalid data, exit
        sprintf(Buf1500 ,"SD_RAM ReScanning : ByteCount=%d   CheckSum=%04X\r\n",u_SRAM.DwnLd_sCNT, u_SRAM.DwnLd_sSUM&0xffff);
        sprintf(tBuf, "SD_RAM Verify Err : Local Data vs Download CheckSum Error\r\n");
        strcat(Buf1500, tBuf);
        putBuf(prt, Buf1500,0);
        return 1;   
        }
    if (holdprt==ePhyIO)
        {
        len=strlen(Buf1500);
        putBuf(ePhyIO, Buf1500,len);
        mDelay(10);
        }
    else
        putBuf(prt, Buf1500,0);
     
    //Continue on, erase flash and re-write it
    u_SRAM.FL_XFER= 2;    
    //FLAG AS PASSED
    sprintf(Buf1500,"FEB  : Erasing %d Sectors Now.. Then Programming.. Wait 20 Secs..\r\n", SECTORES);
    
    len=strlen(Buf1500);
    if (holdprt==ePhyIO)
        {
        putBuf(ePhyIO, Buf1500,len);
        mDelay(10);
        }
    else
        putBuf(prt, Buf1500,0);
   
    u_SRAM.FL_ERASE=1;                          //erase cycle busy
    eraseFLASH_Sector(SECTORES, 0, prt);
    u_SRAM.FL_ERASE=2;                          //erase cycle done
    
    //set write fpga Addr
    SET_SDADDR_RDx(0,0,0);                      //fpgaOffset, addrHI, addrLO
    //u_SRAM.FL_PGM=2;                          //2==good
    sum= loadFLASH(S29JL064J_SECTOR0,holdprt);  //Actual S29JL064J address= 0x0 @Sector 0
    
    u_SRAM.LdTime= ((sUPTIMEHI<<16)+sUPTIMELO); //use fpga uptime counter
    
    if (sum==u_SRAM.DwnLd_sSUM)                 //if match, re-reload fpga0
        {
        sprintf(Buf1500,"FEB  : Load OK, Check Status Now..\r\n");       
        len=strlen(Buf1500);
        while(HappyBus.SndWrds);
        if (holdprt==ePhyIO)
            {
            putBuf(ePhyIO, Buf1500,len);
            mDelay(10);
            }
        else
            putBuf(prt, Buf1500,0);

        //uses tty port here,not lvds
        //reload fpga with new flashed data
        sum= flashXFER(4, prt, FLBASE);
        if (sum==-1)
            u_SRAM.FL_BOOT=1;                   //1==FAILED
        else
            u_SRAM.FL_BOOT=2;                   //2==PASSED
        //recall FPGA Setup Data from FRAM reloaded structure
        dataRecall(1, fpgaBase);                //fpga0
        dataRecall(1, fpgaBase1);               //fpga1
        dataRecall(1, fpgaBase2);               //fpga2
        dataRecall(1, fpgaBase3);               //fpga3
        }
    else
        {
        sprintf(Buf1500,"FEB  : FAILED\r\n");
        len=strlen(Buf1500);
        putBuf(prt, Buf1500,0);                 //send to local and remote ports
        if (holdprt==ePhyIO)
            putBuf(ePhyIO, Buf1500,len);
        }      
    return 0;                                       //0=okay
}


         

//Program (Fake.bin file) to Flash using FPGAx_sdRamx Fake Data already in memory
//Check for valid data in sdRam
//If valid data erase flash and program flash with new sdRam data
int PGM_SDramFakeData(int prt)
{
    uint16_t holdprt=prt;
    int sum=0;
    
    if(prt==ePhyIO)                 //flag to show status on tty of ePhyIO requesting
      prt= tty;    
    
    //Valid download Check
    u_SRAM.FL_XFER= 0;
    if((u_SRAM.DwnLd_sCNT==uPhySums.DwnLd_sCNT) && (u_SRAM.DwnLd_sSUM==uPhySums.DwnLd_sSUM)&& (uPhySums.DwnLd_sCNT !=0)) //match original file specs
        {
        u_SRAM.FL_XFER= 1;    
        //if Read Status Only Mode then exit
        sprintf(tBuf,"DownLoad : GOOD, ChkSum and WordCnt Match\r\n");
        if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
            putBuf(prt, tBuf,0);
        }
    else
        {
        //invalid data, exit
        sprintf(Buf1500 ,"SD_RAM ReScanning : ByteCount=%d   CheckSum=%04X\r\n",u_SRAM.DwnLd_sCNT, u_SRAM.DwnLd_sSUM&0xffff);
        sprintf(tBuf, "SD_RAM Verify Err : Data Does not Match or All Zeros\r\n");
        strcat(Buf1500, tBuf);
        if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
            putBuf(0, Buf1500, 0);   
        return 0;   
        }
    
    uC_FRAM.Fake_SrcAddr= S29JL064J_SECTOR71;   //store src addr
    
    //set write fpga Addr
    SET_SDADDR_RDx(0,0,0);                      //fpgaOffset, addrHI, addrLO
    //u_SRAM.FL_PGM=2;                          //2==good
    
    sum= loadToFLA_FAKE( S29JL064J_SECTOR71, uPhySums.DwnLd_sCNT, prt);       
    //loadToFLA_FAKE returns==0 if failed 
    
    u_SRAM.LdTime= ((sUPTIMEHI<<16)+sUPTIMELO); //use fpga uptime counter    
    if (sum==u_SRAM.DwnLd_sSUM)                 //if match, re-reload fpga0
        {
        sprintf(tBuf,"FEB_LOAD : OKAY, CHECK STATUS NOW\r\n");
        if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
            putBuf(prt, tBuf,0);        
        
        //load struct
        uC_FRAM.Fake_sCNT= uPhySums.DwnLd_sCNT; //store counter
        uC_FRAM.Fake_sSUM= uPhySums.DwnLd_sSUM; //store counter        
        
        //store struct to FRAM
        FRAM_WR_ENABLE();                      
        FRAM_WR(ad_Fake_sSUM, (uint8*)&uC_FRAM.Fake_sSUM, 2);   //write 2 bytes
        FRAM_WR_ENABLE();
        
        FRAM_WR(ad_Fake_sCNT, (uint8*)&uC_FRAM.Fake_sCNT, 4);   //write 4 bytes
        FRAM_WR_ENABLE();
        
        //loaded with const for now
        FRAM_WR_BUF16(ad_Fake_SrcAddr, (uint16*)&uC_FRAM.Fake_SrcAddr, 4);
        FRAM_WR_DISABLE();
        }
    else
        {
        sprintf(tBuf,"FEB_LOAD : FAILED\r\n");
        if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
            putBuf(prt, tBuf,0);
        return 1;           //0=fail
        }      
    return 1;               //1=okay
}



int fillFake(u_32Bit offset, int FileEnd, int show, int prt);
sPTR rFLA;
int  FFwrdCnt;
int  fake_idx;
//Load (Fake.bin file ) from Flash to sdRam
//FPGA addr offsets 0x000, 0x400, 0x800, 0xC00
int ld_FAKE2sdRAM(int prt, int show)
{
    u_16Bit  d16, sum=0, fSum;
    int holdprt= prt;
    u_32Bit ADR32, wCount, FileBeg1,FileBeg2,FileBeg3,FileBeg4;
    int err=0;
    
    if(prt==ePhyIO)                 //flag to show status on tty of ePhyIO requesting
        prt= tty;    

    FFwrdCnt=0;
    //set up sdRam
    CSR0= 0x100;
    CSR1= 0x100;
    CSR2= 0x100;
    CSR3= 0x100;

    //zero wr addr
    SET_SDADDR_WRx(0x000,0,0);              //fpgaOffset,addrH,addrL     
    SET_SDADDR_WRx(0x400,0,0);              //fpgaOffset,addrH,addrL     
    SET_SDADDR_WRx(0x800,0,0);              //fpgaOffset,addrH,addrL     
    SET_SDADDR_WRx(0xc00,0,0);              //fpgaOffset,addrH,addrL     
    //zero rd addr
    SET_SDADDR_RDx(0x000,0,0);              //fpgaOffset,addrH,addrL     
    SET_SDADDR_RDx(0x400,0,0);              //fpgaOffset,addrH,addrL     
    SET_SDADDR_RDx(0x800,0,0);              //fpgaOffset,addrH,addrL     
    SET_SDADDR_RDx(0xc00,0,0);              //fpgaOffset,addrH,addrL     
    
    //stop daq sequencer
    REG16(fpgaBase+0x30e*2)= 0;                    
    
    //update
    FRAM_RD(ad_Fake_sSUM,(uint8*)&uC_FRAM.Fake_sSUM, 2 );
    FRAM_RD(ad_Fake_sCNT,(uint8*)&uC_FRAM.Fake_sCNT, 4 );
    FRAM_RD(ad_Fake_SrcAddr,(uint8*)&uC_FRAM.Fake_SrcAddr, 4);    
    
    wCount= uC_FRAM.Fake_sCNT/2;
    ADR32= uC_FRAM.Fake_SrcAddr;
    fSum= uC_FRAM.Fake_sSUM;   
    rFLA= pFLASHbase + ADR32;           //set ptr flash base+ offset
    
    //fpga 1of4, Addr 00000000
    SET_SDADDR_WRx(0,0,0);    
    if(wCount>0)
      while(wCount)
        {
        //read uB group from flash
        d16= *rFLA++;                   //read flash
        SDR_RD16= d16;                  //store in sdRam
        sum += (d16>>8);
        sum += (d16& 0xff);             //update CheckSum
        wCount--;
        }
    
    if(fSum==sum)                       //re-checking check sum
        sprintf(tBuf,"CkSum= %04X\r\n", sum);
    else
        sprintf(tBuf,"ChkSum in SdRam Error= %04X\r\n", sum);
    if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
        putBuf(prt, tBuf,0);            //send to current active port
    
    rFLA= pFLASHbase + ADR32;           //reset ptr flash base+ offset  
    fake_idx=0; 
    fake_idx=6*2;                       //add in bytes used for 3 32bit pointers
    //init ptr
    FileBeg1 =  0;  

    //init ptr
    FileBeg2 =  ((*rFLA++)<<16);        //16bit high
    FileBeg2 |= *rFLA++;                //16bit low
    FileBeg2 <<=1;                      //word to byte cnt
    
    //init ptr
    FileBeg3 =  ((*rFLA++)<<16);        //16bit high
    FileBeg3 |= *rFLA++;                //16bit low
    FileBeg3 <<=1;                      //word to byte cnt

    //init ptr
    FileBeg4 =  ((*rFLA++)<<16);        //16bit high
    FileBeg4 |= *rFLA++;                //16bit low
    FileBeg4 <<=1;                      //word to byte cnt
    
    sprintf(tBuf,"FileBeg1= %08X\r\nFileBeg2= %08X\r\nFileBeg3= %08X\r\nFileBeg4= %08X\r\n", FileBeg1,FileBeg2,FileBeg3,FileBeg4);
    if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
        putBuf(prt, tBuf,0);            //send to current active port
    
    sprintf(tBuf,"FPGA1 sdRam fill no hits\r\n");
    if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
        putBuf(prt, tBuf,0);    //send to current active port
    fillEmptyEvnts(fPtrOffset1, prt);
    
    sprintf(tBuf,"FPGA2 sdRam fill no hits\r\n");
    if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
        putBuf(prt, tBuf,0);    //send to current active port
    fillEmptyEvnts(fPtrOffset2, prt);

    sprintf(tBuf,"FPGA3 sdRam fill no hits\r\n");
    if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
        putBuf(prt, tBuf,0);    //send to current active port
    fillEmptyEvnts(fPtrOffset3, prt);

    sprintf(tBuf,"FPGA4 sdRam fill no hits\r\n");
    if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
        putBuf(prt, tBuf,0);    //send to current active port
    fillEmptyEvnts(fPtrOffset4, prt);
    
    if(FileBeg2<16)
        {
        sprintf(tBuf,"File size error, no data to move\r\n");
        if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
            putBuf(prt, tBuf,0);    //send to current active port
        return 0;
        }
    //fill f1Ptr sdRam
    fillFake(fPtrOffset1, FileBeg2, show, prt); //use FileBeg2 as end of FileBeg1 data
    sprintf(tBuf,"WordCnt1= %6X\r\n",FFwrdCnt);
    if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
        putBuf(prt, tBuf,0);    //send to current active port
   
    //fill f2Ptr sdRam
    fillFake(fPtrOffset2, FileBeg3, show, prt); //use FileBeg3 as end of FileBeg2 data
    sprintf(tBuf,"WordCnt2= %6X\r\n",FFwrdCnt);
    if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
        putBuf(prt, tBuf,0);    //send to current active port
    
    //fill f3Ptr sdRam
    fillFake(fPtrOffset3, FileBeg4, show, prt); //use FileBeg4 as end of FileBeg3 data
    sprintf(tBuf,"WordCnt3= %6X\r\n",FFwrdCnt);
    if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
        putBuf(prt, tBuf,0);    //send to current active port

    wCount= uC_FRAM.Fake_sCNT;
    //fill f4Ptr sdRam
    fillFake(fPtrOffset4, wCount, show, prt);   //use TotalFileSize-FileBeg4 as end of FileBeg4 data
    sprintf(tBuf,"WordCnt4= %6X\r\n",FFwrdCnt);
    if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
        putBuf(prt, tBuf,0);    //send to current active port

    if(err)
        {
        sprintf(tBuf,"Error count=%d\r\n",err);
        if (holdprt!=ePhyIO)        //msg wont show at controller due to timeout
            putBuf(prt, tBuf,0);    //send to current active port
        return 0;
        } 
    return 1;   //1=okay
}



//MT46H128M16LFDD-48 WT:C – 32 Meg x 16 x 4 Banks
//128Meg x 16 ==  0x800 0000 words
//Fill complete (0x8000000) sdRam memory with FEB simulated test data
//Every 0x100 pages load incrementing test data pattern
//
int fillEmptyEvnts(u_32Bit offset, int prt)
{
    u_32Bit uBun=0, i=3, cnt, xCnt;
    u_16Bit uBunH,uBunL, d16=0x1111;
    sPTR ucSDramLOx;
    
    //CSR0= BIT3;                 //MIG DDR Interface Reset, Must delay 3-5 milli-seconds before using sdRam
    //CSR1= BIT3;                 //MIG DDR Interface Reset, Must delay 3-5 milli-seconds before using sdRam
    //CSR2= BIT3;                 //MIG DDR Interface Reset, Must delay 3-5 milli-seconds before using sdRam
    //CSR3= BIT3;                 //MIG DDR Interface Reset, Must delay 3-5 milli-seconds before using sdRam
    //mDelay(10);
    
    //set write sdRAM Addr via special sequence
    SET_SDADDR_WRx(offset,0,0);
    ucSDramLOx= ((&SDR_RD16)+offset);

    sPTR SDramPort= (&SDR_RD16)+offset;
    cnt= 0x8000000;
    while (uBun< cnt) //0x7FFFF00)         //begin of next fpga file
        {
        uBunH=uBun>>16;   
        uBunL=uBun &0xFFFF;   
        SET_SDADDR_WRx(offset,uBunH, uBunL);
        *SDramPort= 3;   
        *SDramPort= uBunH;   
        *SDramPort= uBunL;  
        xCnt= 256-i;
        //while(i++< 256)
        //fill unused memory with pattern for easy scanning when viewing
        if(xCnt)
            {
          //*SDramPtr= 0x1111;  
             movStr16_NOINC((snvPTR)&d16, (snvPTR)SDramPort, xCnt); 
            }
        i=3+xCnt;
        uBun += 0x100;
        }
    return 0;
}



//load fake data to sdram
//fake data must already be downloaded to FLASH Memory
//
int fillFake(u_32Bit offset, int FileEnd, int show, int prt)
{
    u_16Bit  d16, fill ,uBunH16, uBunL16, holdprt=prt;
    u_32Bit wcnt, err=0;
    u_32Bit uBunH, uBunL, uBun32;
    sPTR SDramPort= (&SDR_RD16)+offset;
       
    if(prt==ePhyIO)             //flag to show status on tty of ePhyIO requesting
      prt= tty;    
     
    while (fake_idx< FileEnd)   //begin of next fpga file
        {
        wcnt= *rFLA++; 
        if ((wcnt==0) || (wcnt>0x100))
            err++;

        uBunH= *rFLA++;
        uBunL= *rFLA++;
        
        uBunH16= uBunH;
        uBunL16= uBunL;
        
        uBun32= ((uBunH<<16) + uBunL);
        uBun32 <<=1;
        uBunH = (uBun32 >> 16);
        uBunL = (uBun32 & 0xffff);                
        
        FFwrdCnt +=3;               //add wrdcnt,ubunHi,ubunLo
        //set sdram address fpga 1of4
        SET_SDADDR_WRx(offset,uBunH, uBunL);
        *SDramPort= wcnt;   
        *SDramPort= uBunH16;   
        *SDramPort= uBunL16;   
        if(show)
            {
            sprintf(tBuf,"FileIndx= %06X    %4X    %4X:%04X     %4X:%04X (<<1)\r\n",fake_idx, wcnt, uBunH16,uBunL16, uBunH,uBunL);
            if (holdprt!=ePhyIO)    //msg wont show at controller due to timeout
                putBuf(prt, tBuf,0);//send to current active port
            }
        
        if((uBunH==0xf) && (uBunL==0x41dc))
          err++;
        
        if(wcnt<3) 
            return 0;               //0==fail
        
        for(int j=0; j<wcnt-3; j++) //word cnt count itself
            {
            FFwrdCnt++;
            d16= *rFLA++;           //read FLASH
            *SDramPort= d16;        //write SDram
            }
        //fill with enough words of force burst write
        fill= wcnt%16;
        while(fill++<16)
            *SDramPort= 0x7777;  
        
        fake_idx += (wcnt*2);       //byte count indx
        //if(show)
        //    {
        //    sprintf(tBuf,"uBun= %04X:%04X     wcnt= %-6X   PostIdx= %-6X(Wrds)   d16= %04X\r\n", uBunH,uBunL, wcnt, fake_idx, d16);
        //    putBuf(prt, tBuf,0);                //send to current active port
        //    }
        }
    return 1;       //1==okay
}



 







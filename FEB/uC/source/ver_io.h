//*******************************************************************************
// mu2e RM48L 'ver_io.h'
// Fermilab Terry Kiper 2016-2019
//*******************************************************************************

#ifndef _VER_IO
#define _VER_IO

#include "het.h"


#define MU2EvER   423
//code version, Major(1), Minor(00)

typedef	unsigned char   u_8Bit;                 //8-bit value
typedef	unsigned short  u_16Bit;                //16-bit value
typedef	unsigned int    u_32Bit;                //32-bit value

typedef unsigned int    uINT;
typedef unsigned long   uLNG;
typedef unsigned short  uSHT;
typedef unsigned char   uCHR;

typedef char*           volatile cPTR;
typedef unsigned short* volatile sPTR;
typedef unsigned short* snvPTR;
typedef unsigned long*  volatile lPTR;
typedef void*           volatile vPTR;


//Bit Mask
#define BIT0  ( 0x00000001 )
#define BIT1  ( 0x00000002 )
#define BIT2  ( 0x00000004 )
#define BIT3  ( 0x00000008 )
#define BIT4  ( 0x00000010 )
#define BIT5  ( 0x00000020 )
#define BIT6  ( 0x00000040 )
#define BIT7  ( 0x00000080 )
#define BIT8  ( 0x00000100 )
#define BIT9  ( 0x00000200 )
#define BIT10 ( 0x00000400 )
#define BIT11 ( 0x00000800 )
#define BIT12 ( 0x00001000 )
#define BIT13 ( 0x00002000 )
#define BIT14 ( 0x00004000 )
#define BIT15 ( 0x00008000 )
#define BIT16 ( 0x00010000 )
#define BIT17 ( 0x00020000 )
#define BIT18 ( 0x00040000 )
#define BIT19 ( 0x00080000 )
#define BIT20 ( 0x00100000 )
#define BIT21 ( 0x00200000 )
#define BIT22 ( 0x00400000 )
#define BIT23 ( 0x00800000 )
#define BIT24 ( 0x01000000 )
#define BIT25 ( 0x02000000 )
#define BIT26 ( 0x04000000 )
#define BIT27 ( 0x08000000 )
#define BIT28 ( 0x10000000 )
#define BIT29 ( 0x20000000 )
#define BIT30 ( 0x40000000 )
#define BIT31 ( 0x80000000 )


#define MAX_WORD_PER_FPGA (0x07800000)

// shortcut macros for reading/writing to 32-bit memory mapped registers
#define REG32(addr) *(volatile unsigned int *)(addr)
#define REG16(addr) *(volatile unsigned short *)(addr)

//fpga short pointers offsets, this value doubles when used 
#define fPtrOffset1     0x0000
#define fPtrOffset2     0x0400
#define fPtrOffset3     0x0800
#define fPtrOffset4     0x0C00

#define fPtrSDramCnt    0x0009                      //SDram count reg offset


//ver STM43F4xx board base address
#define fpgaBase        0x60000000                  //fpga base address0   (uC hardware, chip sel 2)
#define fpgaBase1       0x60000800                  //fpga base address1   (uC hardware, chip sel 2)
#define fpgaBase2       0x60001000                  //fpga base address2   (uC hardware, chip sel 2)
#define fpgaBase3       0x60001800                  //fpga base address3   (uC hardware, chip sel 2)
#define flashBase       0x64000000                  //flash base addr      (uC hardware, chip sel 3)
#define wizBase         0x68000000                  //wiznet base address  (uC hardware, chip sel 4)
#define fOffset         0x00000400                  //fpgas addr offset

#define pFLASHbase    (sPTR) flashBase              //short ptr, parallel flash base address
#define f_Addr_CSR    (sPTR)(fpgaBase+ 0x00)        //FPGA CSR reg
#define CSR0         *(sPTR)(fpgaBase+ (0x00))      //CSR (FPGA0)

#define LNG_SDR_WR   *(lPTR)(fpgaBase+ (0x02*2))    //SDram read address long (32bit ptr)
#define SDramWrHI    *(sPTR)(fpgaBase+ (0x02*2))    //SDram HIGH write address register in FPGA
#define SDramWrLO    *(sPTR)(fpgaBase+ (0x03*2))    //SDram LOW  write address register in FPGA

#define LNG_SDR_RD   *(lPTR)(fpgaBase+ (0x04*2))    //SDram read address long (32bit ptr)
#define SDramRdHI    *(sPTR)(fpgaBase+ (0x04*2))    //SDram HIGH write address register in FPGA
#define SDramRdLO    *(sPTR)(fpgaBase+ (0x05*2))    //SDram LOW  write address register in FPGA

#define HISTPTR14    *(sPTR)(fpgaBase+ (0x14*2))    //SDram uC read/write 16bit normal format
#define HISTDAT16    *(sPTR)(fpgaBase+ (0x16*2))    //SDram uC read/write 16bit normal format

#define SDR_RD16SWP  *(sPTR)(fpgaBase+ (0x06*2))    //SDram uC read 16bit as byte swapped data
#define SDR_RD16     *(sPTR)(fpgaBase+ (0x07*2))    //SDram uC read/write 16bit normal format
#define SDramMIGStat *(sPTR)(fpgaBase+ (0x08*2))    //SDram status reg

#define SDramCnt     *(sPTR)(fpgaBase+ (0x09*2))    //SDram count reg
#define FMDataSnd    *(sPTR)(fpgaBase+ (0x0A*2))    //LVDS FM Xmit Data 16bit port


#define sMuxCntrl    *(sPTR)(fpgaBase+ (0x20*2))    //analog mux control
#define sIN_MASK0    *(sPTR)(fpgaBase+ (0x21*2))    //input mask address
#define sTstCntHI    *(sPTR)(fpgaBase+ (0x22*2))    //test counter hi
#define sTstCntLO    *(sPTR)(fpgaBase+ (0x23*2))    //test counter lo

#define ONEWIREcmd  (u_32Bit)(fpgaBase+ (0x24*2))   //one wire command reg
    #define ONE_TEMP   BIT8
    #define ONE_ROM    BIT9

#define ONEWIREcntr (u_32Bit)(fpgaBase+ (0x25*2))   //one wire control reg
    #define ONE_RST    BIT6
    #define ONE_BSY    BIT7

#define ONEWIREdata  (u_32Bit)(fpgaBase+0x26*2)     //one wire return data reg 0x28-2A
#define sAFE_FFstat  *(sPTR)(fpgaBase+ (0x2F*2))    //AFE fifo status
#define s0AFE_DACbias (u_32Bit)(fpgaBase+(0x30*2))  //AFE bias dacs fpga0
#define s1AFE_DACbias (u_32Bit)(fpgaBase1+(0x30*2)) //AFE bias dacs fpga1
#define s2AFE_DACbias (u_32Bit)(fpgaBase2+(0x30*2)) //AFE bias dacs fpga2
#define s3AFE_DACbias (u_32Bit)(fpgaBase3+(0x30*2)) //AFE bias dacs fpga3

#define s0TRGCNT_HI  *(sPTR)(fpgaBase+ (0x66*2))    //Spill Trigger count addr High 
#define s0TRGCNT_LO  *(sPTR)(fpgaBase+ (0x67*2))    //Spill Trigger count addr Low

#define s0SP_COUNT   *(sPTR)(fpgaBase+ (0x068*2))   //Spill counter addr
#define s0WORD_CNTH  *(sPTR)(fpgaBase+ (0x06A*2))   //Spill word count high
#define s0WORD_CNTL  *(sPTR)(fpgaBase+ (0x06B*2))   //Spill word count low
#define sUPTIMEHI    *(sPTR)(fpgaBase+ (0x06C*2))   //up time HI
#define sUPTIMELO    *(sPTR)(fpgaBase+ (0x06D*2))   //up time LO
#define sTIMSTAMPHI  *(sPTR)(fpgaBase+ (0x072*2))   //time stamp hi
#define sTIMSTAMPLO  *(sPTR)(fpgaBase+ (0x073*2))   //time stamp lo

#define sPULTRIGdly  *(sPTR)(fpgaBase+ (0x074*2))   //pulser trigger delay value
#define sSPILLERROR  *(sPTR)(fpgaBase+ (0x075*2))   //spill error
#define sSpillStatus *(sPTR)(fpgaBase+ (0x076*2))   //spill status
    #define SPIL_BUSY     BIT0                      //seq busy
    #define SPIL_END      BIT1                      //end of spill(goes high)
    #define SPIL_GATE     BIT2                      //spill active (high)

#define sAFE_FIFO_AR  (fpgaBase+ (0x080*2))         //AFE-FIFO 0x80-0x8f
#define sAFE_RdData  *(sPTR)(fpgaBase+ (0x0FF*2))   //AFE-rd-data
#define AFE_RdData    (fpgaBase+ (0x0FF*2))         //AFE-rd-data

#define sAFE_0x100   *(sPTR)(fpgaBase+ (0x100*2))   //AFE-Reg at addr 100
#define AFE_0x100     (fpgaBase+ (0x100*2))         //AFE-Reg at addr 100

#define sFlashGate   *(sPTR)(fpgaBase+ (0x300*2))   //flash gate control
#define sFlashOnTim  *(sPTR)(fpgaBase+ (0x301*2))   //flash gate on time
#define sFlashOffTim *(sPTR)(fpgaBase+ (0x302*2))   //flash gate off time
#define sTrigCntrl   *(sPTR)(fpgaBase+ (0x303*2))   //trigger control 0=lemo, 1=rj45, 2=reset deserializer
    #define TRIG_SOFT   BIT0                        //software trigger, set high
    #define FMHI_PULLO  BIT1                        //trig source 0=lemo pulse, 1=fm on rj45 input 
    #define TRIG_INH    BIT2                        //trigg inhibit, goes high on trigger(wr 1 allows more trigs)
    #define TRG_INHen   BIT3                        //trigg inhibit enable (allows bit2 function)
    #define SPIL_INH    BIT4                        //spill inhibit, goes high at end of spill
    #define SPIL_INHen  BIT5                        //spill inhibit enable (allows bit4 function)
#define sPipeLinDly  *(sPTR)(fpgaBase+ (0x304*2))   //pile line delay
#define sSampleLen   *(sPTR)(fpgaBase+ (0x305*2))   //sample length

#define LNG_TSTPULSE *(lPTR)(fpgaBase+ (0x306*2))   //test pulser(32bit ptr)
#define sTstPulFreqH *(sPTR)(fpgaBase+ (0x306*2))   //test pulser freg high
#define sTstPulFreqL *(sPTR)(fpgaBase+ (0x307*2))   //test pulser freg low
#define sTstPulSpDur *(sPTR)(fpgaBase+ (0x308*2))   //test pulser spill duration
#define sTstPulIntSp *(sPTR)(fpgaBase+ (0x309*2))   //test pulser spill interspill spill duraton

#define SDPTR_BCST_ADH *(sPTR)(fpgaBase+ (0x310*2)) //Broadcast sdRam read addr upper
#define SDPTR_BCST_ADL *(sPTR)(fpgaBase+ (0x311*2)) //Broadcast sdRam read addr lower

#define SDPTR_BCASTHA (0x312)                       //Broadcast sdRam read addr
#define SDPTR_BCASTLA (0x313)                       //Broadcast sdRam read addr

#define SDPTR_BCASTH0 *(sPTR)(fpgaBase+ (SDPTR_BCASTHA*2)) //broadcast sdRam read addr
#define SDPTR_BCASTL0 *(sPTR)(fpgaBase+ (SDPTR_BCASTLA*2)) //broadcast sdRam read addr
#define SDPTR_BCASTH1 *(sPTR)(fpgaBase1+ (SDPTR_BCASTHA*2)) //broadcast sdRam read addr
#define SDPTR_BCASTL1 *(sPTR)(fpgaBase1+ (SDPTR_BCASTLA*2)) //broadcast sdRam read addr
#define SDPTR_BCASTH2 *(sPTR)(fpgaBase2+ (SDPTR_BCASTHA*2)) //broadcast sdRam read addr
#define SDPTR_BCASTL2 *(sPTR)(fpgaBase2+ (SDPTR_BCASTLA*2)) //broadcast sdRam read addr
#define SDPTR_BCASTH3 *(sPTR)(fpgaBase3+ (SDPTR_BCASTHA*2)) //broadcast sdRam read addr
#define SDPTR_BCASTL3 *(sPTR)(fpgaBase3+ (SDPTR_BCASTLA*2)) //broadcast sdRam read addr


#define BCST_GEO_312 *(sPTR)(fpgaBase+ (0x312*2))   //Broadcast DAQ GEO Address
#define BCST_GEO_313 *(sPTR)(fpgaBase+ (0x313*2))   //Broadcast DAQ GEO Address

#define BCST_GEO_314 *(sPTR)(fpgaBase+ (0x314*2))   //Broadcast GEO Address Ctrl# and Brd#
#define BCST_GEO_317 *(sPTR)(fpgaBase+ (0x317*2))   //Rd/Wr Global FPGA Event FIFO Reg, 0xf==data ready in fifos

#define DAQ_FIFO1     *(sPTR)(fpgaBase+ (0x00C*2))  //Read FPGA0 daq fifo data reg
#define DAQ_FIFO2     *(sPTR)(fpgaBase+ (0x40C*2))  //Read FPGA0 daq fifo data reg
#define DAQ_FIFO3     *(sPTR)(fpgaBase+ (0x80C*2))  //Read FPGA0 daq fifo data reg
#define DAQ_FIFO4     *(sPTR)(fpgaBase+ (0xc0C*2))  //Read FPGA0 daq fifo data reg

#define eFIFO_wCNT1   *(sPTR)(fpgaBase+ (0x00D*2))  //Event fifo words used counter FPAG1
#define eFIFO_wCNT2   *(sPTR)(fpgaBase+ (0x40D*2))  //Event fifo words used counter FPAG2
#define eFIFO_wCNT3   *(sPTR)(fpgaBase+ (0x80D*2))  //Event fifo words used counter FPAG3
#define eFIFO_wCNT4   *(sPTR)(fpgaBase+ (0xC0D*2))  //Event fifo words used counter FPAG4

#define CSR1        *(sPTR)(fpgaBase1+ (0x00))      //CSR (FPGA1)
#define SDramWrHI1  *(sPTR)(fpgaBase1+ (0x02*2))    //SDram HIGH write address register in FPGA
#define CSR2        *(sPTR)(fpgaBase2+ (0x00))      //CSR (FPGA2)
#define CSR3        *(sPTR)(fpgaBase3+ (0x00))      //CSR (FPGA3)


#define sAFE_01  *(sPTR)(fpgaBase+ (0x101*2))       //Ultra Sound Fpga0 AFE5807 power enable
#define sAFE_02  *(sPTR)(fpgaBase+ (0x201*2))       //Ultra Sound Fpga0 AFE5807 power enable
#define sAFE_03  *(sPTR)(fpgaBase+ (0x135*2))       //Ultra Sound Fpga0 AFE5807 power enable
#define sAFE_04  *(sPTR)(fpgaBase+ (0x235*2))       //Ultra Sound Fpga0 AFE5807 power enable

#define sAFE_11  *(sPTR)(fpgaBase1+ (0x101*2))      //Ultra Sound Fpga1 AFE5807 power enable
#define sAFE_12  *(sPTR)(fpgaBase1+ (0x201*2))      //Ultra Sound Fpga1 AFE5807 power enable
#define sAFE_13  *(sPTR)(fpgaBase1+ (0x135*2))      //Ultra Sound Fpga1 AFE5807 power enable
#define sAFE_14  *(sPTR)(fpgaBase1+ (0x235*2))      //Ultra Sound Fpga1 AFE5807 power enable

#define sAFE_21  *(sPTR)(fpgaBase2+ (0x101*2))      //Ultra Sound Fpga2 AFE5807 power enable
#define sAFE_22  *(sPTR)(fpgaBase2+ (0x201*2))      //Ultra Sound Fpga2 AFE5807 power enable
#define sAFE_23  *(sPTR)(fpgaBase2+ (0x135*2))      //Ultra Sound Fpga2 AFE5807 power enable
#define sAFE_24  *(sPTR)(fpgaBase2+ (0x235*2))      //Ultra Sound Fpga2 AFE5807 power enable

#define sAFE_31  *(sPTR)(fpgaBase3+ (0x101*2))      //Ultra Sound Fpga3 AFE5807 power enable
#define sAFE_32  *(sPTR)(fpgaBase3+ (0x201*2))      //Ultra Sound Fpga3 AFE5807 power enable
#define sAFE_33  *(sPTR)(fpgaBase3+ (0x135*2))      //Ultra Sound Fpga3 AFE5807 power enable
#define sAFE_34  *(sPTR)(fpgaBase3+ (0x235*2))      //Ultra Sound Fpga3 AFE5807 power enable

#define sDACsBIAS_0  *(sPTR)(fpgaBase+ (0x30*2))    //DAC Regs chip0 0x30-0x3f
#define sDACsBIAS_1  *(sPTR)(fpgaBase1+(0x30*2))    //DAC Regs chip1 0x30-0x3f
#define sDACsBIAS_2  *(sPTR)(fpgaBase2+(0x30*2))    //DAC Regs chip2 0x30-0x3f
#define sDACsBIAS_3  *(sPTR)(fpgaBase3+(0x30*2))    //DAC Regs chip3 0x30-0x3f

#define sDAC_FLASH0  *(sPTR)(fpgaBase+ (0x40*2))    //FLASHER DAC Regs chip0 0x40-0x43
#define sDAC_FLASH1  *(sPTR)(fpgaBase0+(0x40*2))    //FLASHER DAC Regs chip1 0x40-0x43
#define sDAC_FLASH2  *(sPTR)(fpgaBase1+(0x40*2))    //FLASHER DAC Regs chip2 0x40-0x43
#define sDAC_FLASH3  *(sPTR)(fpgaBase2+(0x40*2))    //FLASHER DAC Regs chip3 0x40-0x43

//cnts on fpga0 done, now do cnts on fpga 1-3
#define s1WORD_CNTH  *(sPTR)(fpgaBase+ (0x46A*2))   //Spill word count high
#define s1WORD_CNTL  *(sPTR)(fpgaBase+ (0x46B*2))   //Spill word count low
#define s2WORD_CNTH  *(sPTR)(fpgaBase+ (0x86A*2))   //Spill word count high
#define s2WORD_CNTL  *(sPTR)(fpgaBase+ (0x86B*2))   //Spill word count low
#define s3WORD_CNTH  *(sPTR)(fpgaBase+ (0xC6A*2))   //Spill word count high
#define s3WORD_CNTL  *(sPTR)(fpgaBase+ (0xC6B*2))   //Spill word count low

//mask on fpga0 done, now do mask on fpga 1-3
#define sIN_MASK1    *(sPTR)(fpgaBase+ (0x421*2))   //input mask address
#define sIN_MASK2    *(sPTR)(fpgaBase+ (0x821*2))   //input mask address
#define sIN_MASK3    *(sPTR)(fpgaBase+ (0xC21*2))   //input mask address

//SDram Read Address 4of4
#define LNG_SDR_RD0  *(lPTR)(fpgaBase+ (0x004*2))    //SDram read address fpga0
#define LNG_SDR_RD1  *(lPTR)(fpgaBase+ (0x404*2))    //SDram read address fpga1
#define LNG_SDR_RD2  *(lPTR)(fpgaBase+ (0x804*2))    //SDram read address fpga2
#define LNG_SDR_RD3  *(lPTR)(fpgaBase+ (0xC04*2))    //SDram read address fpga3

//SDram Write Address 4of4
#define LNG_SDR_WR0  *(lPTR)(fpgaBase+ (0x002*2))    //SDram write address fpga0
#define LNG_SDR_WR1  *(lPTR)(fpgaBase+ (0x402*2))    //SDram write address fpga1
#define LNG_SDR_WR2  *(lPTR)(fpgaBase+ (0x802*2))    //SDram write address fpga2
#define LNG_SDR_WR3  *(lPTR)(fpgaBase+ (0xC02*2))    //SDram write address fpga3

//Swap data ptr (1of4)
#define SDR_RD16SWP0  *(sPTR)(fpgaBase+ (0x006*2))   //SDram uC read 16bit as byte swapped data fpga0
#define SDR_RD16SWP1  *(sPTR)(fpgaBase+ (0x406*2))   //SDram uC read 16bit as byte swapped data fpga1
#define SDR_RD16SWP2  *(sPTR)(fpgaBase+ (0x806*2))   //SDram uC read 16bit as byte swapped data fpga2
#define SDR_RD16SWP3  *(sPTR)(fpgaBase+ (0xC06*2))   //SDram uC read 16bit as byte swapped data fpga3

//Swap data ptr (1of4)
#define SDR_RD16_0  *(sPTR)(fpgaBase+ (0x007*2))   //SDram uC read 16bit as byte unswapped data fpga0
#define SDR_RD16_1  *(sPTR)(fpgaBase+ (0x407*2))   //SDram uC read 16bit as byte unswapped data fpga1
#define SDR_RD16_2  *(sPTR)(fpgaBase+ (0x807*2))   //SDram uC read 16bit as byte unswapped data fpga2
#define SDR_RD16_3  *(sPTR)(fpgaBase+ (0xC07*2))   //SDram uC read 16bit as byte unswapped data fpga3


//uController Error Triggers
#define TrgSinBitErr *(sPTR) 0xF00803F0
#define TrgDouBitErr *(sPTR) 0xF00803F8

//FRAM SETUP
#define hdrSize          15                     //QIE HEADER SIZE in words
#define DWNLD_VALID      0xAA55
#define PASSCODE         123                    //simple password for param entry

#define fPAGE_FPGA0      0x0000                 //store fpga regs chip 0
#define fPAGE_FPGA1      0x0100                 //store fpga regs chip 1
#define fPAGE_FPGA2      0x0200                 //store fpga regs chip 2
#define fPAGE_FPGA3      0x0300                 //store fpga regs chip 3

#define fPAGE_0400       0x0400                 //uC Misc for MUX,GAIN,TRIG,LINKDIR,nERROR
#define fPAGE_Net500     0x0500                 //network stuff store byte addr base

#define MAIN_ADDR_VALID  0x0600                 //fpga misc MAIN FPGA FLAG
#define MAIN_ADDR_COUNT  0x0604                 //fpga misc MAIN FPGA DOWNLOAD CNT
#define MAIN_ADDR_CSUM   0x0608                 //fpga misc MAIN FPGA DOWNLOAD CSUM
#define BACKUP_WCOUNT    0x060A                 //fpga misc store count for backup use
#define BACKUP_BYTES     0x0610                 //fpga image backup bytes
#define BACKUP_CSUM      0x0614                 //fpga image backup chksum


#define const_FPGADACs0  0x0A00                 //constants bias, led, afe dacs fpga0
#define const_FPGADACs1  0x0B00                 //constants bias, led, afe dacs fpga1
#define const_FPGADACs2  0x0C00                 //constants bias, led, afe dacs fpga2
#define const_FPGADACs3  0x0D00                 //constants bias, led, afe dacs fpga3
#define const_FPGADACend 0x0DFF                 //constants bias, led, afe dacs addr max
//FRAM  0x1000 repeats

//constants
#define SAVE30      0x030                       //reserve 30 bytes between offset grp and ped grp on each FRAM PAGE
#define addr030     0x030                       //dac addr begin fpga0
#define addr047     0x047                       //dac addr range end
#define addr430     0x430                       //dac addr begin fpga1
#define addr447     0x447                       //dac addr range end
#define addr830     0x830                       //dac addr begin fpga2
#define addr847     0x847                       //dac addr range end
#define addrC30     0xC30                       //dac addr begin fpga3
#define addrC47     0xC47                       //dac addr range end

//F-RAM
#define fWRSR       0x01                        //Write Status Register   0000_0001b
#define fWRITE      0x02                        //Write Memory Data       0000_0010b
#define fREAD       0x03                        //Read Memory Data        0000_0011b
#define fWRDI       0x04                        //Write Disable           0000_0100b
#define fRDSR       0x05                        //Read Status Register    0000_0101b
#define fWREN       0x06                        //Set Write Enable Latch  0000_0110b
#define fs_WEL      0x02                        //Write Enable Latch Bit
#define fs_WPEN     0x80                        //FRAM, WPEN is high, /WP pin controls write access


// Flag Bits for Register (genFlag )
#define ADC_REFRESH     0x0001                  //ADC Data local Buf refresh 
#define ADC_READY       0x0002                  //flag as ADC data is avail
#define ADC_TRIG        0x0004                  //ADC new data req
#define uTimeOut        0x0008                  //uS timer flag 0==timeout
#define not used        0x0010                  //
#define hDelay          0x0020                  //delay flag nHet Delay function
#define ARP_REQ         0x0040                  //send network ARP, incase gateway cant find board
#define ADC_REQ         0x0080                  //ADC_REQ=1 active read in progress
#define ADC_DONE        0x0100                  //ADC_DONE=1 Read in finished
#define SPILLGATEOPEN   0x0200                  //SEQ spill gate active
#define SPILL_ABORT     0x0400                  //SPILL BUSY timeout, abort seq
#define OneWireDatReq   0x0800                  //request data via one wire
#define OneWireDatRdy   0x1000                  //requested data now ready
#define OneWireRefresh  0x2000                  //oneWire data refresh active
#define NO_ECHO         0x4000                  //console, no echo reply
#define OVC_TRIP        0x8000                  //bais voltage trip flag



// Flag Bits for Register (iFlag)
#define CONFIGFAIL      0x01                    //alter cyclone config status
#define CONFIGBACKUP    0x02                    //alter cyclone config status use backup file
#define iWiznetACK      0x04                    //wiznet chip valid response
#define SI5338FAIL      0x08                    //SI5338 init flag 1==failed
#define DAQ_BUSY        0x10                    //spill active check, DAQ process ready
#define XMIT_PHY_ON     0x20                    //end of spill check daq, enable==1
#define WHATCHDOGENA    0x40                    //whatdog timer enable
#define CMB_RD_ENA      0x80                    //CMB readout enable bit


//flash variables
#define adr555 (0x555 << 1)                     //flash chip command codes
#define adr2aa (0x2aa << 1)                     //flash chip command codes
#define VALID           0xaa55                  //valid data mask for saved data structure
#define FLBASE          1                       //use flash base to load fpga
#define FLBACKUP        0                       //use flash backup to load fpga

//sector to erase in parallel flash
#define SECTORES        40                      //ERASE 500k Words, fpga file take 802,294 byts

#define     UART scilinREG                      //lin module configured as UART
#define     USB_inBufSz 256

//wiznet defs and dma
#define CHAR_BUF_SZ_1600      1600              //dma buffer max byte count (SOCKET SIZE)

//wiznet defs and dma
#define D_SIZE_BYTE  2048
#define D_SIZE_WRDS  2048/2


#define putchar     __putchar
#define cmdbufsiz       128                     //uart1 command line buffer size
#define CmdSiz80         80                     //limit repeat command line size

//#define PROMPT_ER   SendStrTxI("FPGA_Error>");  //err prompt

// ASCII Constants
#define BACKSP          0x08
#define ACK             0x06
#define NACK            0x15
#define SP              0x20
#define DELETE          0x7F

#define MaxSock         5                       //max socket used, defined for rec buffer, eRecDatBuf[]
#define tty             232                     //i/o stream port uart0
#define sock0           0                       //i/o stream port socket 0  line based telnet
#define sock1           1                       //i/o stream port socket 1  line based telnet
#define sock2           2                       //i/o stream port socket 2, line based telnet
#define ePhyIO          3                       //mac phy link (does not use wiznets buffers)
#define LoopEnd         3                       //look 0-3 times, loop 3 checks local ePHY input data

#define sockARP         4                       //ARP request on time interval
#define eRecSz1024   1024                       //socket(s) cmd line buf size, must match 'setSn_MSSR'

#define ARP_TIME     (1000*60*5)                //ARP request, once ever 5 minutes;
#define ARP_TIME_NOW (ARP_TIME-3000)            //ARP request, now;


// USB/TTY Baud Rates
#define BAUD115200  115200
#define BAUD230400  230400
#define BAUD460800  460800
#define BAUD921600  921600


//*********** HET1 OUTPUTS *************
//*********** HET1 OUTPUTS *************
            
//FLASH RESET       (K18) HET1_0 
#define FlashRst_LO    hetREG1->DCLR=  BIT0;   //500nS low pulse, wait 50-nS to read
#define FlashRst_HI    hetREG1->DSET=  BIT0;
 
//SRCSEL            (V2) HET1_1
//TMP05A Temperature Input

//15V_ENA           (U1) HET1_3
#define LO_15V_ENA     hetREG1->DCLR=  BIT3;
#define HI_15V_ENA     hetREG1->DSET=  BIT3;

//FLASH_WP          (V6) HET1_5
#define FLASH_WP_LO    hetREG1->DCLR=  BIT5;
#define FLASH_WP_HI    hetREG1->DSET=  BIT5;

//TP46              (T1) HET1_7
#define hLO_TP46        hetREG1->DCLR=  BIT7
#define hHI_TP46        hetREG1->DSET=  BIT7
            
//ETHERNET_DOWN     (N2) HET1_13
#define EthDown_LO     hetREG1->DCLR=  BIT13;
#define EthDown_HI     hetREG1->DSET=  BIT13;
            
//Suspend           (A11) HET1_14
#define Suspend_LO      hetREG1->DCLR=  BIT14;
#define Suspend_HI      hetREG1->DSET=  BIT14;
            
//TEST PNT          (N1) HET1_15    
#define hLO_TP45        hetREG1->DCLR=  BIT15
#define hHI_TP45        hetREG1->DSET=  BIT15

//CLR_ERR           (A13) HET1_17  nError reset
#define CLR_ERR_LO      hetREG1->DCLR=  BIT17;
#define CLR_ERR_HI      hetREG1->DSET=  BIT17;

//DSR               (J1) HET1_18
#define DSR_LO          hetREG1->DCLR=  BIT18;
#define DSR_HI          hetREG1->DSET=  BIT18;

//LINK_DIR          (B13) HET1_19
#define LINK_DIR_LO     hetREG1->DCLR=  BIT19;
#define LINK_DIR_HI     hetREG1->DSET=  BIT19;

//WIZRST            (P2) HET1_20
#define WIZRST_LO      hetREG1->DCLR=  BIT20;
#define WIZRST_HI      hetREG1->DSET=  BIT20;

//ucLED             (B3) HET1_22
#define hLO_ucLED       hetREG1->DCLR=  BIT22;
#define hHI_ucLED       hetREG1->DSET=  BIT22;

//RDWR_B            (J4) HET1_23
#define RDWR_B_LO      hetREG1->DCLR=  BIT23;
#define RDWR_B_HI      hetREG1->DSET=  BIT23;

//SHUTDOWN          (M3) HET1_25
#define hLO_ShtDwn      hetREG1->DCLR=  BIT25;
#define hHI_ShtDwn      hetREG1->DSET=  BIT25;

//#define INIT_Bx[] = {29,16,27,6}; //Init array list of 'init' het port#

//Read InitB3 pin   (W3) HET1_6
#define InitB3_Read    ((hetREG1->DIN>>6) & 1U)
            
//Read InitB1 pin   (A4) HET1_16
#define InitB1_Read    ((hetREG1->DIN>>16) & 1U)

//Read InitB2 pin   (A9) HET1_27
#define InitB2_Read    ((hetREG1->DIN>>27) & 1U)

//Read InitB0 pin   (A3) HET1_29
#define InitB0_Read    ((hetREG1->DIN>>29) & 1U)

    
//*********** HET1 INPUTS ************
//*********** HET1 INPUTS ************

//ERROR LATCH       (B12) HET1_4 INPUT  nError Monitor
#define hERR_Latch  ((hetREG1->DIN >> 4) & 1U) 

//FLASH RDY         (V7) HET1_9 INPUT
#define FLASH_RDY   ((hetREG1->DIN >> 9) & 1U)
            
//DTR USB BRIDGE    (E3) HET1_11
#define hDTR        ((hetREG1->DIN >> 11) & 1U))

//CARRIER SENSE: MII mode pin is asserted high when receive medium is non-idle
//ETHER CAR_SENSE   (B4) HET1_12 INPUT
#define hCAR_SENSE  ((hetREG1->DIN >> 12) & 1U)

//DONE2             (H4) HET1_21
#define DONE2       ((hetREG1->DIN >> 21) & 1U)

//WIZNET IRQ        (J17)HET1_31
#define hWIZ_IRQ    ((hetREG1->DIN >> 31) & 1U)


            
//*********** SPI $ GIO OUUTS ************
//*********** SPI $ GIO OUTPUTS ************

//EQUALIZER CS         (B2, SPI3_CS2) OUTPUT
//#define EQCS_LO      spiPORT5->DCLR=(uint32)1U<<27;
//#define EQCS_HI      spiPORT5->DSET=(uint32)1U<<27;

//ADC_START         (G16, SPI5_SOMI3) OUTPUT
#define ADCSTART_LO  spiPORT5->DCLR=(uint32)1U<<27;
#define ADCSTART_HI  spiPORT5->DSET=(uint32)1U<<27;

//ADC_RESET         (G17, SPI5_SIMO3) OUTPUT
#define ADCRESET_LO  spiPORT5->DCLR=(uint32)1U<<19;
#define ADCRESET_HI  spiPORT5->DSET=(uint32)1U<<19;
            
//ETHERNET_RESET    (W6, SPI5_CS2) OUTPUT
#define ETH_RST_LO  spiPORT5->DCLR=(uint32)1U<<2;
#define ETH_RST_HI  spiPORT5->DSET=(uint32)1U<<2;
            
//#define CSI_Bx[]={2,3,4,5};  //Init array list of chip select gio port#

//CSI_B0            (C1) GIOA2 OUTPUT
#define CSI_B0_LO   gioPORTA->DCLR = (uint32)1U << 2;
#define CSI_B0_HI   gioPORTA->DSET = (uint32)1U << 2;

//CSI_B1            (E1) GIOA3 OUTPUT
#define CSI_B1_LO   gioPORTA->DCLR = (uint32)1U << 3;
#define CSI_B1_HI   gioPORTA->DSET = (uint32)1U << 3;

//CSI_B2            (A6) GIOA4 OUTPUT
#define CSI_B2_LO   gioPORTA->DCLR = (uint32)1U << 4;
#define CSI_B2_HI   gioPORTA->DSET = (uint32)1U << 4;

//CSI_B3            (B5) GIOA5 OUTPUT
#define CSI_B3_LO   gioPORTA->DCLR = (uint32)1U << 5;
#define CSI_B3_HI   gioPORTA->DSET = (uint32)1U << 5;

//PROG_B0           (J2) GIOB6 OUTPUT
#define PROG0_LO    gioPORTB->DCLR = (uint32)1U << 6;
#define PROG0_HI    gioPORTB->DSET = (uint32)1U << 6;

//PROG_B1           (G1) GIOB4 OUTPUT
#define PROG1_LO    gioPORTB->DCLR = (uint32)1U << 4;
#define PROG1_HI    gioPORTB->DSET = (uint32)1U << 4;

//PROG_B5           (G2) GIOB5 OUTPUT
#define PROG2_LO    gioPORTB->DCLR = (uint32)1U << 5;
#define PROG2_HI    gioPORTB->DSET = (uint32)1U << 5;

//PROG_B3           (W10) GIOB3 OUTPUT
#define PROG3_LO    gioPORTB->DCLR = (uint32)1U << 3;
#define PROG3_HI    gioPORTB->DSET = (uint32)1U << 3;

//PROG_ALL          BITS 3,4,5,6 GIOBx OUTPUT
#define PROGx_LO    gioPORTB->DCLR = (uint32)(1U<<3)+(1U<<4)+(1U<<5)+(1U<<6);
#define PROGx_HI    gioPORTB->DSET = (uint32)(1U<<3)+(1U<<4)+(1U<<5)+(1U<<6);

//*********** SPI $ GIO INPUTS ************
//*********** SPI $ GIO INPUTS ************
           
//MUX_A2            (M2) GIOB0 OUTPUT
#define MUXA2_LO    gioPORTB->DCLR = (uint32)1U ;
#define MUXA2_HI    gioPORTB->DSET = (uint32)1U ;

//MUX_A3            (K2) GIOB1 OUTPUT
#define MUXA3_LO    gioPORTB->DCLR = (uint32)1U << 1;
#define MUXA3_HI    gioPORTB->DSET = (uint32)1U << 1;

//SRC_SEL           (F1) GIOB7 OUTPUT
#define SRCSEL_LO   gioPORTB->DCLR = (uint32)1U << 7;
#define SRCSEL_HI   gioPORTB->DSET = (uint32)1U << 7;

//ADCRDY            (F2) GIOB2 INPUT
#define GIO_ADC_RDY ((gioPORTB->DIN >> 2) & 1U)

//DONE0             (C2) GIOA1 INPUT
#define DONE0       ((gioPORTA->DIN >> 1) & 1U)

//DONE3             (A5) GIOA0 INPUT
#define DONE3       ((gioPORTA->DIN >> 0) & 1U)

//DONE1             (H3) GIOA6 INPUT
#define DONE1       ((gioPORTA->DIN >> 6) & 1U)

//PII STAT          (M1) GIOA7 INPUT
#define PII_STAT    ((gioPORTA->DIN >> 7) & 1U)

#define ADC_CS      0xFEU  //(N3) SPI2CS0, 'spi.h'==SPI_CS_0
#define PGA_CS      0xFDU  //(D3) SPI2CS1, 'spi.h'==SPI_CS_1

//COLLISION DETECT: In 10Base/100Base-TX half-duplex modes, this pin is 
//asserted HIGH only when both transmit and receive media are non-idle. (F3)
           
// Toggle HET pin0
//gioSetPort(hetPORT1, gioGetPort(hetPORT1) ^ 0x00000001);




//NOTE: CAN PORTS will be used on FEB Version 4 and up...
//CAN PORTS 1,2,3 AS INPUTS, SEE CANINIT() setup 
//
//CAN1tx 'WixLink'  (A10) 1 bit input  (setup up as input in CANINIT)
#define CAN1txRD    (canREG1->TIOC & 1U) //CAN1tx GIO Port read only

//CAN1rx            (B10) 1 bit input (setup up as input in CANINIT, not used)
//not used #define CAN1rxRD    (canREG1->RIOC & 1U) //CAN1rx GIO Port read
//
//CAN2tx            (H2) 1 bit input (setup up as output in CANINIT, not used)
// not used #define CAN2txRD    (canREG2->TIOC & 1U) //CAN2tx GIO Port read
//CAN2rx            (H1) 1 bit input (setup up as input in CANINIT, not used)
// not used #define CAN2rxRD    (canREG2->RIOC & 1U) //CAN2rx GIO Port read
//
//CAN3tx 'OVCRst'   (M18) 1 bit output (setup up as output in CANINIT)
#define CAN3txRD    (canREG3->TIOC & 1U) //CAN3tx GIO Port read
//CAN3rx 'OVCTrip'  (M19) 1 bit input (setup up as input in CANINIT)
#define CAN3rxRD    (canREG3->RIOC & 1U) //CAN3rx GIO Port read


// CAN PORTS 1,2,3, if output mode min pulse width ~350nS, SEE CANINIT() setup
//
//not used CANtx2     (H2) 1 bit 
//not used CANrx2     (H1) 1 bit
//
//CANtx3            (M18) 1 bit output CANtx3 high,low
#define CAN3txHI    canREG3->TIOC = ((canREG3->TIOC & 0xFFFFFFFDU) | 2U) //CAN3tx GIO Port write 1
#define CAN3txLO    canREG3->TIOC = ((canREG3->TIOC & 0xFFFFFFFDU) | 0U) //CAN3tx GIO Port write 0
//CANrx3            (M19) 1 bit setup as an input


//CAN names used in coding 'BIAS over current circuit control and status'
#define canOVCstat  CAN3rxRD   //read status, trip== 80mSec high pulse
#define canOVCRstLO CAN3txLO   //set OVCRst port low
#define canOVCRstHI CAN3txHI   //set OVCRst port high


//CAN name used in coding  '' 
#define WIZLink CAN1txRD   //reads Wiznet Link Cable Connected Status
//COLLISION DETECT: In 10Base/100Base-TX half-duplex modes, this pin is 
//asserted HIGH only when both transmit and receive media are non-idle. (A10)           





#define InBufSiz_512  512
#pragma pack(4)                                 //force 32bit boundary on all types
struct vB {
            //Warning, RdPtr,WrPtr caused pgm resets when assigned volatile
            volatile int RdPtr;                          
            volatile int WrPtr;
            volatile int Cnt;
            unsigned char Buf[InBufSiz_512];
        };


#pragma pack(2)                                 //force 16bit boundary on all types
struct netinfo_s {
            unsigned char ipAddr[4];            //ip
            unsigned char gateWay[4];           //gate   (network)
            unsigned char netMask[4];           //mask
            unsigned char macAddr[6];           //mac
            u_16Bit sTelnet0;                   //sock0 port number
            u_16Bit sTelnet1;                   //sock1 port number
            u_16Bit sTelnet2;                   //sock2 port number
            u_16Bit sTelnet3;                   //sock3
            u_16Bit sTcpTimeOut;                //TCP timeout,(100uS/bit)
            u_16Bit sTcpRetry;                  //TCP retry count
            u_16Bit Spare1;                     //Spare1
            u_16Bit Spare2;                     //Spare2
            u_16Bit Valid;                      //data saved flag
            u_16Bit reserved;                   //required, quick fix, save routine comes up short by one
           };



//Ethernet/Wiznet power up defaults (will be overwritten if user FLASH has valid data for these settings)
static const struct netinfo_s defNetInfo0 = {
    {131, 225, 53, 84},                     //local IP  default null setup
    {131, 225, 55, 200 },                   //Gateway Wilson Hall
    {255, 255, 255, 00 },                   //Mask for all
    {0x00,0x80,0x55,0xBD,0x00,0x15},        //Mac number 'HLSWH5'

    5000,                                   //telnet0 port char based
    5001,                                   //telnet1 port char based
    5002,                                   //telnet2 port char based
    5003,                                   //telnet3 port char based
    100,                                    //TCP timeout,(100uS/bit), 100=10mS, 50=5mS //use min of 100 to prevent re-transmits
    6,                                      //sets up the TCP retry count.
    1,                                      //spare1
    2,                                      //spare2
    0,                                      //valid download flag
    0                                       //required, quick fix, save routine comes up short by one
};

static const struct netinfo_s defNetInfo = {
    {131, 225, 52, 177},                    //local IP  default null setup
    {131, 225, 52, 200 },                   //Gateway Wilson Hall
    {255, 255, 255, 00 },                   //Mask for all
    {0x00,0x80,0x55,0xEE,0x00,0x05},        //Mac number 'dcrc6'

    5000,                                   //telnet0 port char based
    5001,                                   //telnet1 port char based
    5002,                                   //telnet2 port char based
    5003,                                   //telnet3 port char based
    100,                                    //TCP timeout,(100uS/bit), 100=10mS, 50=5mS //use min of 100 to prevent re-transmits
    6,                                      //sets up the TCP retry count.
    1,                                      //spare1
    2,                                      //spare2
    0,                                      //valid download flag
    0                                       //required, quick fix, save routine comes up short by one
};

static const struct netinfo_s defNetInfo192 = {
    {192, 169, 0, 19 },                     //local IP
    {192, 169, 0,  1  },                    //Gateway Wilson Hall
    {255, 255, 255, 00 },                   //Mask for all
    {02,01,02,03,04,05},                    //Mac number
    5000,                                   //daq port 0 line based
    5001,                                   //daq port 1 line based
    5002,                                   //telnet port char based
    5003,                                   //UDP socket port, default
    50,                                     //TCP timeout,(100uS/bit), 100=10mS, 50=5mS
    6,                                      //sets up the TCP retry count.
    0,                                      //valid setup flag
    0                                       //socket 3 status
};


//#define HET25OFF    hetREG1->DCLR=  BIT25;
//#define HET25ON     hetREG1->DSET=  BIT25;

//CONFIGURING THE Si5338 WITHOUT DEFAULT ADDR= 0x70
#define own_add_w 0x10
#define own_add_r 0x11
#define slv_add_w (0x20)
#define slv_add_r (0x21)
#define bsize    16
#define Dev5338  0x70


//TDC Spill Header Words
#define cCntlHdrSizWrd (8)                      //main controller header size in words
#define cCntlHdrSizByt (8*2)                    //main controller header size in bytes

#pragma pack(4)                                 //force 16bit boundary on all types
//warning on pack(4),dont mix 32 and 16 words if you dont want unused memory words
struct Controller_tdcHdr
            {
            u_16Bit tSpilWrdCntL;               //(all FPGAs plus hdr size of 8 words)
            u_16Bit tSpilWrdCntH;               //32 bit value tSpilWrdCntX is endian swapped
            u_16Bit tSpilTrgCntL;               //32bit as low/high words
            u_16Bit tSpilTrgCntH;               //32bit value tSpilTrgCntX is endian swapped
            u_16Bit tSpilCyc;
            u_16Bit tMask,
                    tID,
                    tStatus,
                    tbitFlags;
            };


//status block buffers
#define     sBLKSIZ38   38
#define     sBLKSIZ22   22

struct  stBlock {
                //uint16 hdr[3];
                uint16 uC[sBLKSIZ22];
                uint16 afe[4][sBLKSIZ38];
};


struct structFPGA_Trig_Cnts
            {
            u_32Bit WrdCnt[5];                  //CNTs per fgpa words (4 fpga plus 5th out_of_range_reg counter)
            u_32Bit TotWrdCnt;                  //CNTs total
            u_32Bit RDBreqSiz;                  //CNTs requested to read
            u_32Bit BytToSnd;                   //bytes to send per fpga (use this counter word counter)
            u_32Bit WrdToSnd;                   //word to send per fpga (use this counter byte counter)
            u_32Bit OverRun;                    //counter, req size larger than stored data size
            u_16Bit Mask[5];                    //TRIG MASK ALL
            u_16Bit Fpga1of4;                   //selected fpga 1of4
            u_16Bit OverMax;                    //fpga 1of4 data count to high
            u_16Bit OverMaxCounter;             //fpga OverMax total counter
            };

//system timer in this structure and incremented in file notification.c, function rtiNotification()
struct msTimers{
           u_32Bit  volatile g_timeMs,
                    WatchDog,
                    g_pCntPerSec,
                    g_pCnt,
                    g_wTicks,
                    g_wARP,
                    g_wSTAR,
                    g_ADCms,
                    //g_SockmSec,
                    g_OneWireTime,
                    g_OneWireNewScanDelay;
            };
                    

//sendbin() control structure
struct blocksnd {
            uLNG gSndBytCnt;
            lPTR gSndSrc;
            sPTR gSndSrcSptr;
            int  gSndPrt;                       //port number
            int  gSndMode;                      //reg or daq data mode
            int  gSndLen;
            int  gDMA_Fpga2Mem;
   volatile int  gDMA_Mem2Wiz; 
            int  gDMA_TimDlys[5];
            int  gASC_TimDlys[5];
            };


//sendRDX() control structure
struct blocksndWrd {
            uLNG gSndWrdCnt;                    //word count 
            lPTR gSndSrc;
            sPTR gSndSrcSptr;
            int  gSndPrt;                       //port number
            int  gSndMode;                      //reg or daq data mode
            int  gSndLen;
            int  gActBuf;
            int  gDMA_Fpga2Mem;
            int  gDMA_Mem2Wiz;
            int  gDMA_TimDlys[5];
            int  gASC_TimDlys[5];
            };


//UDP definitions
#pragma pack(4)                                 //force 16bit boundary on all types
struct udpHeader
            {
            unsigned int destip;
            unsigned int destport;
            unsigned int sndErr;
            };

#define RST280  0x11


//FRAM FPGA stored register count
#define FPGAregCount   0x72                     //fpga reg count to store in FRAM

//Misc data to store in FRAM page 0x400
#define idx_serNumb     0
#define idx_pgaMuxCh    2
#define idx_pga280Gain  4
#define idx_TrgSrcCh    6

#define idx_LinkDir     8
#define idx_nErrCnt     10
#define idx_Valid       12
#define idx_wDogTOCnt   14

#define idx_FpgaEccCnt  16                      //16 bit
#define idx_NewLinPrmpt 18                      //16 bit
#define idx_Fake_sSUM   20                      //16 bit
#define idx_Baud        22                      //32 bit

#define idx_Fake_sCNT   26                      //32 bit
#define idx_Fake_SrcAdr 30                      //32 bit
#define idx_End        (30+4)                   //end




#define ad_Fake_sSUM    0x416                   //16 bit
#define ad_Fake_sCNT    0x418                   //32 bit
#define ad_Fake_SrcAddr 0x41C                   //32 bit (41C,41D,41E,41F)

//totol bytes use her
#define FRAM_400wsz    idx_End                  //size of SetUp in 'Bytes'

//uC data storage
struct uC_Store
            {
            u_16Bit serNumb;                    //board serial number
            u_16Bit pgaMuxCh;                   //HDMI Trim Volt MuxChip(s) ADG1609 1of4
            u_16Bit pga280Gain;                 //Prg Gain Amp for Trim Volt input
            u_16Bit TrgSrcCh;                   //trig input src, rg45 or lemo
            
            u_16Bit LinkDir;                    //LINK lvds direction
            u_16Bit nErrCnt;                    //ErrLatch nError counter
            u_16Bit Valid;                      //old xmitPHY DAQ xmit on PHY (on/off), now flags FRAM data good
            u_16Bit wDogTimOutCnt;              //watch dog timer timeout counter
            
            u_16Bit FpgaEccCnt;                 //ECC error counter for FPGAs                       
            u_16Bit NewLinPrmpt;                //2 new line prompt control
            u_16Bit Fake_sSUM;                  //2 bytes chksum fake test data option
            u_32Bit TTYbaud;                    //32bit usb/tty baud rate

            u_32Bit Fake_sCNT;                  //4 bytes count fake test data option
            u_32Bit Fake_SrcAddr;               //4 bytes flash source addr, fake test data option
            };
  
//ePHY download struc
struct uSums {
              u_32Bit DwnLd_sCNT;               //4 bytes count
              u_16Bit DwnLd_sSUM;               //2 bytes chksum
              u_16Bit LdStatus_Tick;            //2 bytes last load status
              u_16Bit FL_XFER;                  //2 bytes TRANSFER status
              u_16Bit FL_ERASE;                 //2 bytes ERASE status
              u_16Bit FL_PGM;                   //2 bytes PGM status
              u_16Bit FL_BOOT;                  //2 bytes FPGA BOOTUP status
              u_16Bit FL_SDram;                 //2 bytes FPGA BOOTUP status
              u_32Bit LdTime;                   //4 bytes last load time fpga upTime
              u_16Bit FL_SOCK_CHKSUM;           //2 bytes FLASH load using socket checked sum
              u_32Bit FL_SOCK_CHKSIZE;          //4 bytes FLASH load using socket checked filesize
};



#pragma pack(2)                                 //force 16bit boundary on all types
struct udpMessage
            {
            u_16Bit  *Ptr;
            u_16Bit  type,
                     msgId,
                     initial,
                     reqSiz,
                     rData[0x410],              //size max (BYTES PLUS ROOM FOR UNPACKED TO PACK OVERRUN) as 1472-8=1464, (header=8)
                     overRun,
                     SockErr,                   //sock
                     udpReqs,                   //udp command counter
                     udpSendDly;                //delay over 1mS on upd xmit
            };


#define FP_REGS_MAX     23      //nn regs groups(ROWs), plus one Later Code Counts from 1
#define FP_REGS_VALID0 0xAFE0   //mask off lowest nibble
#define FP_REGS_VALID1 0xAFE1
#define FP_REGS_VALID2 0xAFE2
#define FP_REGS_VALID3 0xAFE3
#define rLOADED         00      //Number of actual register loaded, Filled in by code
#define fMux            0
#define fGain           3       //3==gain of 1 on pgm280 chip


//fpga reg setup structure for Constants(defaults, flash and restore)
typedef struct FPGA_Regs{
   unsigned short Addr;
   unsigned short RegCnt;
   unsigned short Value[8];
} FPGA_RegS;


//fpga reg setup structure for Save/Recall
typedef struct FPGA_Regs1{
   unsigned short Value[8];
} FPGA_RegS1;



typedef struct HappyBusReg{
   unsigned short CmdSizW;
   unsigned short BrdNumb;
   unsigned short CmdType;
   unsigned short SndWrds;
   //unsigned short myLinkNumber;     //filled in by init from controller
   //unsigned short myLnkCntrlTmp;    //filled in by init from controller
   unsigned short CntrlNumb;            //controller number from controller 
   unsigned short PHYrecvErr;
   unsigned short *Src;
} sHappyBusReg;


//time of day struct
struct time  {                                  //24 hour time
            uSHT    sec,
                    min,
                    hr,
                    day;
            uLNG    totalSec;
            };



//New for Diaqnostic Buffers 03-11-20 tek
struct ubdiaq {
        u_16Bit ReqH;
        u_16Bit ReqL;
        u_16Bit MemH;
        u_16Bit MemL;  
        u_16Bit ubstat;
};


/* 
//from "types.h"
typedef uint8			u_char;		//8-bit value 
typedef uint8 			SOCKET;
typedef uint16			u_short;	// 16-bit value 
typedef uint16			u_int;		// 16-bit value 
typedef uint32			u_long;		// 32-bit value 

typedef union _un_adcData {
	uint32	iVal;
	uint8  cVal[3];
}un_adcData;
*/

#endif
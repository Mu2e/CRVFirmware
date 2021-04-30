
//******************************************************************
// @file sys_mu2e.c //  Fermilab Terry Kiper 2016-2020
//  mu2e FEB board
//  RM48 Micro Controller
//  RM48 Hercules device supports little-endian [LE] format

//
//  This function is called after file sys_main.c
//  Program Xilinx Spartan 6 fpga
//  Program and Read Spansion Inc S29JL064J 64 Megabit 
//                  (8M x 8-Bit/4M x 16-Bit) Flash Memory
//  Program and Read FRAM memory chip 'FM25CL64B'

//  The RM48L952 device integrates the ARM Cortex-R4F Floating-Point CPU which 
//  offers providing up to 365 DMIPS. 
//  The RM48L952 supports
//      runs up to 220 MHz
//      little-endian [LE] format
//      3MB of integrated flash
//      256KB RAM with single-bit error correction and double-bit error detection
//******************************************************************


//using V2 hardware
//tek 03-10-15: Ver100 Iniatial V2 code mods
//tek 03-25-15: Ver120 nError external hardware STM6510 timer in sysTick 1mS routine
//tek 05-12-15: Ver121 Release version used at Test Beam 
//tek 03-25-15: Ver122 Added struct uC_FRAN vars 'Mux,Gain,...' to FRAM save/recall functions
//tek 04-14-15: Ver123 Added HTML port 80 status
//tek 07-01-15: Ver124 fixed Mux gain channel selector cmd "mux"
//tek 07-07-15: Ver125 update stab() data reading, now moves data to structure 
//tek 08-03-15: Ver126 added adc1 function and pooing adc channel checking with new 'var adc_ms'
//tek 11-18-15: Ver127 mod to sendBin to now send all 4 fpga data
//tek 11-30-15: Ver128 mod to sendBin shows warning if reg larger than stored fpga word cnt
//tek 02-01-16: Ver129 added zero len string check in putBuf()
//tek 02-05-16: Ver130 mods to prevent possible ESM resets
//tek 03-09-16: Ver131 mode ADC_ads1259() reads now signed fltpoint data for cmd 'A0'
//tek 05-31-16: Ver132 port 80 display mods
//tek 06-01-16: Ver133 LVDS test code added
//tek 06-10-16: Ver134 added 5 minute ARP timeup using flag ARP_REQ
//tek 06-10-16: Ver134 added auto display of status using STAR_REQ for temp 'radiation test'
//tek 09-19-16: Ver135 added test code for RJ45 phy driver link testing
//tek 10-20-16: Ver136 reads on oneWire chs 'Temps+Roms' now done in background
//tek 10-20-16: Ver137 moved HTML code to file 'sys_mu2e_web.c', use web address= ip address
//tek 10-24-16: Ver138 uC ADC buffer updates only on data req, additional code cleanup
//tek 02-23-17: Ver139 added Wiznet ASCII xmit delay counters, see 'ID' cmd
//tek 02-23-17: Ver139 added sendbin-NON-DAQ data using cmd 'RDBR'
//tek 02-27-17: Ver140 added command echo of '>' on NULL line input
//tek 04-05-17: Ver141 help menu cleanup
//tek 04-18-17: Ver201 mod sock processing mode to allow multi commands per packet
//tek 04-20-17: Ver201 added cmd 'RDX' to return 'RDB' data over mac Phy
//tek 05-10-17: Ver201 added DAC RD WR commands for 'Calibration Correction' 
//tek 06-05-17: Ver202 added timeout in my_spiTransmitData(), added WatchDog timer
//tek 06-06-17: Ver300 minor mods nErr, 1st load on PreProduction FEBs
//tek 06-14-17: Ver301 cmd DSAV and DREC mods
//tek 06-19-17: Ver302 cmd DSAV and DREC mods/fixes done
//tek 06-20-17: Ver303 DAC slope and pedestal commands ready
//tek 06-23-17: Ver304 RDB() fpga word overmax checking
//tek 07-12-17: Ver306 ePHY xfer re-coding, now seeing 12Mbyte xfer speed
//tek 07-13-17: Ver307 ePHY 'fill_daq_packet()' fix for xfers less than 6 bytes send extra bytes
//tek 07-14-17: Ver308 ePHY 'fill_daq_packet()' fix for xfers less than 6 bytes, split packets
//tek 07-14-17: Ver308 cmd 'RDB' data words vs bytes count cleanup
//tek 07-17-17: Ver309 lvds fm port funcions, cmd 'FMS' data words vs bytes count cleanup
//tek 07-19-17: Ver310 cleanup ePHY EMAC receive port coding
//tek 07-19-17: Ver311 added socket downloads of fpga flash file
//tek 08-16-17: Ver312 added receiver(ePHY) and driver(FM-LVDS) code for feb--controller link
//tek 08-16-17: Ver312 added receiver(ePHY) and driver(FM-LVDS) code for feb--controller link
//tek 08-30-17: Ver320 improved receiver(ePHY) and driver(FM-LVDS), sdRAM test now good with new fpga code
//tek 09-27-17: Ver321 improved link to/form 'feb controller'
//tek 10-19-17: Ver322 DACs calibration code ready, beta
//tek 11-15-17: Ver323 update memory tester, added lvds i/o intr mode
//tek 01-08-18: Ver324 flash file copy to backup active fpga bin data 'added 'fl_copyFPGA()'
//tek 01-08-18: Ver324 change FLASH writes times from 5 ticks to 6 ticks in emif_ASYNC2Init() emif.c
//tek 01-08-18: Ver324 change lvds xmits from 6 to 13uS backtoback send wait timeout
//tek 03-08-18: Ver325 optimized lvds and ePhy port code
//tek 03-14-18: Ver326 added ePHY DAQ packet decoding
//tek 03-22-18: Ver327 modes for fast data pooling and link inits, new DAQ format
//tek 04-17-18: Ver328 fixed 'S4' ramTest fpga Addr Blocks
//tek 07-19-18: Ver329 fixed "A0" ADC_ads1259 reads, this delays code 100mSec per read
//tek 07-30-18: Ver330 DAQ data return ready for initial testing 'eCMD_DAQ_DY2_Handler()' 
//tek 08-03-18: Ver331 DAQ data cleanup 
//tek 09-11-18: Ver332 dsav constants cleanup
//tek 09-21-18: Ver333 tweaks 'eCMD_DAQ_DY2_Handler()' code 
//tek 09-24-18: Ver334 more tweaks 'eCMD_DAQ_DY2_Handler()' code 
//tek 09-26-18: Ver335 Link init controller number(upper byte) and port number(lower)
//tek 09-27-18: Ver336 FEBs will responds to ePHY link CMDs without board ID set by 'LCHK'
//tek 10-01-18: Ver337 Added ubReqs error counters
//tek 10-02-18: Ver338 mods to allow for test 'fake' data downloads and some file name changes
//tek 10-04-18: Ver339 mods to allow for test 'fake' data downloads
//tek 10-10-18: Ver340 daq test data code added, see help 'HF' for LDFE,LPFP,LDFC
//tek 10-11-18: Ver341 daq readout has changed, now data is in FIFO, not SDram, func 'eCMD_DAQ_DY2_Handler()'
//tek 10-16-18: Ver342 fake data functions now ready for testing, controller req uBunch stored in FEB sdRam
//tek 10-17-18: Ver343 mods to help and fake data funtions
//tek 10-19-18: Ver344 more mods to help and fake data functions
//tek 10-31-18: Ver345 send_full() function added error message in on "Caller app"
//tek 03-01-19: Ver346 removed ref to rdb(pause) and cmd 'RDSEL', FermiLab TestBeam,  not used
//tek 03-12-19: Ver347 minor cleanup on unused code before sending to ARGONNE LAB
//tek 03-25-19: Ver348 moved g_EMACTxBsy check to begin of eCMD_DAQ_DY2_Handler() in file 'sys_mu2e_daq.c '
//tek 05-22-19: Ver349 trapped newline char '>' while RDB commands are active, added No_Prompt to limit ASCII prompts 
//tek 05-23-19: Ver350 added No_Prompt option setting under command 'ECHO',
//tek 05-29-19  Ver351 Cmd 'RD,RDB,RDBR,WR' One Time New Line Prompt Block on exit for DAQ Apps 'NO_ECHO'
//tek 06-05-19  Ver352 ADC sample now Non-Continous Mode, plus enabled Sample_Cap_Discharge 
//tek 06-26-19  Ver353 fix non DMA FLASH loader, normally not used, tty prompt cleanup for 'WR,RD' commands
//tek 06-27-19  Ver354 added breakout counter to ADC_ads1259()
//tek 07-24-19  Ver355 mods to uB request error status sys_mu2e_daq.c func 'eCMD_DAQ_DY2_Handler()'
//tek 08-26-19  Ver356 eCMD_DAQ_DY2_Handler uB packet size increase from 256W to 512W
//tek 08-26-19  Ver357 modified 'emac.c' EMACRxPromiscuousEnable() to receive short non standard packets
//tek 08-26-19  Ver357 todo modify file notification.c emacRxNotification() to handle new short packet format
//tek 08-26-19  Ver357 todo modify file notification.c emacRxNotification() to handle new short packet format
//tek 08-28-19  Ver358 moved bootup FRAM load ahead of dataRecall() mods to uB Status word
//tek 12-05-19  Ver359 added test functions cmd 'FMS' sends 'incr word' on LVDS port
//tek 12-06-19  Ver360 added baud rate command for TTY/USB port
//tek 12-06-19  Ver360 modified RTI and fixed Pooled data early intr in code @ "LSTAB"
//tek 12-12-19  Ver361 lvds port xmits mods using putBuf()
//tek 02-13-20  Ver400 new production circuit boards, new CAN Bus i/o Volt/Cur trip, see cmd 'OVC'
//tek 02-19-20  Ver401 uB request overflow, uses BIT15 flag, see ubGetData() in file 'sys_mu2e_daq.c'
//tek 02-24-20  Ver402 cmd 'OVC' modes doto 80mSec trip status
//tek 02-27-20  Ver403 data pools now returned as blocks of 64 words with 64uS delays
//tek 02-27-20  Ver404 uBunch Req to FEB, PHY link coded to use small non standard ethernet packets (avail but not swithed over yet)
//tek 03-04-20  Ver405 data pools now returned as blocks of 64 words with 128uS delays, small controller input buffer
//tek 03-10-20  Ver406 misc fixes adc resolution on pooling and uB DAQ overflow bit
//tek 03-12-20  Ver407 added testing vars to ubDAQ code
//tek 03-19-20  Ver408 added debug option for checking uBunch Req Errors
//tek 03-24-20  Ver409 mods to fpga downloader from controller
//tek 03-27-20  Ver410 code comment cleanup, sys_mu2e_Web now sys_mu2e_Status
//tek 04-01-20  Ver411 more mods to fpga downloader from controller
//tek 04-01-20  Ver411 add cmd 'FLSOCK', uses socket connection to UPDATE FLAS01
//tek 04-09-20  Ver412 mods to readAFE() 
//tek 04-13-20  Ver413 mods in notification.c 'rtiNOTIFICATION_COMPARE2' lvds xmit timing
//tek 04-13-20  Ver414 mods in DAQ ubGetData() to clear fifo on uB Errors
//tek 04-17-20  Ver415 expanded error diagnostics in ubGetData()
//tek 04-22-20  Ver416 expanded error diagnostics
//tek 04-28-20  Ver417 mods to command handler recd phy data
//tek 05-13-20  Ver418 fixed uBunch status word
//tek 05-15-20  Ver419 added fpga addr map to help menu
//tek 09-03-20  Ver420 mods to uB event error diag messages in ubGetData()
//tek 11-30-20  Ver421 added cmd CMBENA to ena/dis background CMB ch readout
//tek 03-01-21  Ver422 fixed CanBus CAN1tx i/o port as input and added Wiznet Cable Connection check
//tek 03-19-21  Ver423 #define added _FEB_PRODUCTION, allows ethernet cable detection, updated Wiznet ARP gw_processing and keepAlive





//Comment out following line for Pre_Production_Boards
//
//#define _FEB_PRODUCTION   //production boards have CAN bus i/o ports allowing 
                            //for disabling of Wiznet Network if no cable attached
//
//comment out if Pre_Productions Boards
//
//Note: code will hang here if optimized beyond 'LOW'
//Note: code will hang here if optimized beyond 'LOW'

//*************************************************************************
//***                                                                   ***
//*** Compile Optimize 'NONE' prevents random nError reset              ***
//*** IE. #pragma optimize=speed                                        ***
//*** Use this pragma directive to decrease optimization level, or to   ***
//*** turn off some specific optimizations. This pragma directive only  ***
//*** affects the function that follows immediately after the directive ***
//***                                                                   ***
//*************************************************************************


//------------------------- Note Change -----------------------------
//after a 'HALCODEGEN' build the following stack fix may be needed 
//change stacks in sys_core.asm on each HAL-CODE-GEN --base- 0x08038000 for RM48L952

//------------------------- Note Change -----------------------------
//new projects need this mode in *.icf file
//tek *** change linker file.icf from mem:__ICFEDIT_intvec_start__ { readonly section .intvec};
//tek *** change linker file.icf to   mem:__ICFEDIT_intvec_start__ { readonly section .intvecs};
//

//------------------------- Note Change -----------------------------
//
//after a 'HALCODEGEN' build 'adc.c' 
//      - Enable/Disable continuous conversion
//      adcREG1->GxMODECR[1U] = (uint32)ADC_12_BIT
//                          | (uint32)0x00000020U
//                          | (uint32)0x00000000U
//                          | (uint32)0x00000000U;  //0x00000000U=single conversion tek mod June2019
//
//      - Setup discharge prescaler
//      - Enable/Disable discharge
//      adcREG1->G1SAMPDISEN = (uint32)((uint32)2U << 8U) //discharge clk cycles,tek mod June2019
//                         | (uint32)0x00000001U;         //discharge cap enable,tek mod June2019
//


//------------------------- Note Change -----------------------------
//
//after a 'HALCODEGEN' build 'for UART/LIN usb data' 
//in file 'sci.c', func 'linHighLevelInterrupt(), 'case 11U' change code to below
//    case 11U:
//          //receive tek mod
//          byte= (uint8)(scilinREG->RD & 0x000000FFU);
//          USB_Rec.Buf[USB_Rec.WrPtr]= byte;
//          USB_Rec.WrPtr++;
//          //reset storage ptr if at end of buffer
//          if (USB_Rec.WrPtr >= InBufSiz_512)
//              USB_Rec.WrPtr= 0;
//          USB_Rec.Cnt++;
//        break;


//------------------------- Note Change -----------------------------
//after a 'HALCODEGEN' build 'for UART/LIN usb data' 
//in file 'sci.c', change following structure 
//from 'static   struct g_sciTransfer'
//to            'struct g_sciTransfer'


//------------------------- Note Change -----------------------------
//after a 'HALCODEGEN' build 'for PHY on RJ45 receive data' 
//Modify 'emac.c' 
//void EMACTxIntISR(void)
    // added line below to end of this function, used to be in above 'emacTxNotification()'
//    paKet.EMACTxBsy=0;   //tek added in status 
//}

//------------------------- Note Change -----------------------------
//Modify 'notifications.c' file 
//add following lines or newer codes as needed 

//emac stuff
//#include "emac.h"
//#include "ver_ePHY.h"
//#include "sys_mu2e.h"
//extern rxch_t *mu_rxch_int;
//extern struct phyHeader phyHdr;
//struct phyPACSTAT paKet;      //all daq packets, status/counter


//void emacRxNotification(hdkif_t *hdkif)
//{
///*  enter user code between the USER CODE BEGIN and USER CODE END. */
///* USER CODE BEGIN (59) */
//  rxch_t *rxch_int;
//  volatile emac_rx_bd_t *curr_bd, *last_bd;
//  uint32 rxptr;   
//  uint32 rxCnt;   
//    if (paKet.EMACRxRdy==1)             //rec still busy
//        {
//        if (phyHdr.PacNext==phyHdr.PacActive)  //buffers all full
//            {
//            paKet.eREC_BUSY++;  //counter
//            return;                 //skip it
//            }
//        }
//  
//    //The receive structure that holds data about a particular receive channel
//    rxch_int = &(hdkif->rxchptr);
//
//    //Get the buffer descriptors which contain the earliest filled data
//    curr_bd = rxch_int->active_head;
//    last_bd = rxch_int->active_tail;
//
//    //Process the descriptors as long as data is available
//    //when the DMA is receiving data, SOP flag will be set
//
//    //Start processing once the packet is loaded
//    if((curr_bd->flags_pktlen & EMAC_BUF_DESC_OWNER)!= EMAC_BUF_DESC_OWNER ) 
//        {
//        //this bd chain will be freed after processing
//        rxch_int->free_head = curr_bd;
//  
//        //Start processing once the packet is loaded
//        //this bd chain will be freed after processing
//        //rxch_int->free_head = curr_bd;
//        hHI_TP46;
//        rxptr= rxch_int->free_head->bufptr;       //tek
//        rxCnt= rxch_int->free_head->bufoff_len;   //tek
//        if(rxCnt>ePayLdMaxAndHdr)                 //MsgBytMax==64
//            rxCnt=ePayLdMaxAndHdr;
//        phyHdr.PacBytCnt[phyHdr.PacNext]= rxCnt;
//        //16bit moves, cnt is in word count Note: limit stored bytes to 'ePayLdMax'
//        movStr16((sPTR)rxptr,(sPTR)&phyHdr.PacPayLd1024w[phyHdr.PacNext], rxCnt/2);
//        hLO_TP46;
//        
//        phyHdr.PacNext++;
//        if (phyHdr.PacNext>= PacBufs)           //out of buffer?
//            phyHdr.PacNext=0;
//        paKet.EMACRxRdy=1;                          //flag as rec'd packet
//        }
//}
///* USER CODE END */


//------------------------- Note Change -----------------------------
//file mode 'emac.c'
//tek comment out 'EMACLinkSetup()' to speed things up 02-27-15
//after 'Dp83640LinkStatusGet()' comment out 'EMACLinkSetup()' see below
//  if(EMACLinkSetup(hdkif) != EMAC_ERR_OK) {
//    retVal = EMAC_ERR_CONNECT;
//  }   else {
//  }


//------------------------- Note Change -----------------------------
//file mode 'emac.c'
//ADD ONE LINE AFTER
//Initialize the EMAC, EMAC Control and MDIO modules
//EMACInit(hdkif->emac_ctrl_base, hdkif->emac_base);
//MDIOInit(hdkif->mdio_base, MDIO_FREQ_INPUT, MDIO_FREQ_OUTPUT);
//
//ADD  
//EMACRxPromiscuousEnable(hdkif->emac_base, 0); //(base, ch)   'Enable Promiscuous Mode'
//
//The RXPROMCH bit in RXMBPENABLE selects the promiscuous channel to receive frames selected by
//the RXCMFEN, RXCSFEN, RXCEFEN, and RXCAFEN bits. These four bits allow reception of MAC
//control frames, short frames, error frames, and all frames (promiscuous), respectively.
//
//tek addded 'EMACTransmit()' hung up on reset breakout june 2018 
//ADD ONE LINE AFTER, while((curr_bd->flags_pktlen & EMAC_BUF_DESC_SOP) == EMAC_BUF_DESC_SOP) {
//  if (RecHung++ > 8000000)
//    {
//    RecHung=0;
//    PaketStats.eRecHung++;       
//    resetEntry();
//    }


//------------------------- Note Change -----------------------------
//  //USER CODE BEGIN (7)
    //TEK 09-2014, required if you want the EMIF port to actually work!!!
    //The MPU setup was configured through Halcogen, the config function isn't actually 
    //called by the generated code. Please place the following calls in sys_startup.c
//  _mpuInit_();    //add this line
//  _mpuEnable_();  //add this line


//------------------------- Note Change -----------------------------
//tek *** change linker file.icf from --place at address mem:__ICFEDIT_intvec_start__ { readonly section .intvec};
//tek *** change linker file.icf to   --place at address mem:__ICFEDIT_intvec_start__ { readonly section .intvecs};
// ****** use local copy of modified linker file 'RM48L952.icf'  *******

//------------------------- Note Change -----------------------------
//change stacks in sys_core.asm on each HAL-CODE-GEN --base- 0x08038000 for RM48L952
//
//don't want this in file 'sys_core.asm'
//userSp  .word 0x08000000+0x00001700
//svcSp   .word 0x08000000+0x00001700+0x00000100
//fiqSp   .word 0x08000000+0x00001700+0x00000100+0x00000200
//irqSp   .word 0x08000000+0x00001700+0x00000100+0x00000200+0x00000600
//abortSp .word 0x08000000+0x00001700+0x00000100+0x00000200+0x00000600+0x00000100
//undefSp .word 0x08000000+0x00001700+0x00000100+0x00000200+0x00000600+0x00000100+0x00000100
//
/*
//manually modified halcogen code as of july 2015  in file 'sys_core.asm' 
//fill down from top of RAM at 0x0803FFFF
userSp  dcd 0x0803DE00+0x00001700
svcSp   dcd 0x0803DE00+0x00001700+0x00000100
fiqSp   dcd 0x0803DE00+0x00001700+0x00000100+0x00000200
irqSp   dcd 0x0803DE00+0x00001700+0x00000100+0x00000200+0x00000600
abortSp dcd 0x0803DE00+0x00001700+0x00000100+0x00000200+0x00000600+0x00000100
undefSp dcd 0x0803DE00+0x00001700+0x00000100+0x00000200+0x00000600+0x00000100+0x00000100
*/

//------------------------- Note Change -----------------------------
/*-Sizes in local dir linker icf file
//define symbol __ICFEDIT_size_cstack__   = 0x1700;
//define symbol __ICFEDIT_size_svcstack__ = 0x100;
//define symbol __ICFEDIT_size_irqstack__ = 0x600;
//define symbol __ICFEDIT_size_fiqstack__ = 0x200;
//define symbol __ICFEDIT_size_undstack__ = 0x100;
//define symbol __ICFEDIT_size_abtstack__ = 0x100;
//define symbol __ICFEDIT_size_heap__     = 0x4000;
//End of ICF editor section. ###ICF###*/

//-- RM48 compile/link info in local file 'sys_Help_RM48_Compile_Link.h' ---
//-- RM48 compile/link info in local file 'sys_Help_RM48_Compile_Link.h' ---


//SDOCM00114735
//Safety Library fails to build with IAR compiler
//Release Note
 
//Safety Library fails to build with IAR compiler
//Version Info:
//IAR C/C++ Compiler for ARM
//  7.10.3.6832 (7.10.3.6832)
//Workaround 
///*BEGIN COMMENT*/
//----------------------------
//For compiling the project in IAR, the following lines in the definition of SL_ESM_Init() under 
//sl_esm.c needs to be changed
//FROM
//                sl_vimRAM->ISR[1]  = &sl_esm_high_intr_handler;
//                sl_vimRAM->ISR[21] = &sl_esm_low_intr_handler;
//TO
//                sl_vimRAM->ISR[1]  = (sl_t_isrFuncPTR)&sl_esm_high_intr_handler;
//                sl_vimRAM->ISR[21] = (sl_t_isrFuncPTR)&sl_esm_low_intr_handler;
//----------------------------
///*END COMMENT*/



//BOOT LOADER INFO  
//The RM48 device has got two main TCM Flash banks (0 and 1) both are 1.5MB in size, 
//in addition there is Bank 7 (64kB) which is intended only for data storage. The 
//first sector (0) of Flash Bank 0 is 32kB in size which should be enough for a 
//serial BL (Chuck estimated 2kB) but might be tight for a BL with USB, FAT or 
//Ethernet functionality.
//
//Create your main application as normal, but have some way (hardware jumper, network 
//command, media connect, etc.) to vector from your main application to the starting
//address of your boot loader code in the next bank. You can do a jump in C like this:
//Remember that before you jump, switch off your interrupts!
//
//JMP EXAMPLE
//jump to bank 1 boot loader starting address
//u32JumpStart = (Luint32)0x00280000;
//((void (*)(void))u32JumpStart)();


/* Include Files */
#include "sys_common.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>


#include "esm.h"
#include "het.h"
#include "rti.h"
#include "spi.h"
#include "emif.h"
#include "sci.h"
#include "adc.h"
#include "sys_dma.h"
#include "gio.h"
#include "emac.h"
#include "can.h"

#include "hw_reg_access.h"
#include "sys_core.h"
#include "reg_system.h"

#include "nHetCap.h"
#include "reg_het.h"
#include "hal_stdtypes.h"

//sys_mu2e proto-types
#include "ver_io.h"
#include "ver_help.h"
#include "ver_ePHY.h"

#include "sys_mu2e_misc.h"
#include "sys_mu2e_functions.h"
#include "sys_mu2e.h"

//fram
#include "sys_mu2e_FRAM.h"

//wiznet stuff chip W5300
#include "socket.h"


//system
extern int  resetEntry();                   //file 'sys_intvecs.asm'
#define     putchar     __putchar

//spi port control config
spiDAT1_t   dataconfig_ADC;
spiDAT1_t   dataconfig_PGA;
spiDAT1_t   dataconfig_FRAM;
spiDAT1_t   dataconfig_EQUAL;
spiDAT1_t   dataconfig_FRAM_HLD;            //hold 'cs' active low


//Wiznet biffers
extern uint8  check_sendok_flag[];//Flag to check if first send or not.
extern int  recDatSz[];         //store last rec'd packet data size (diagnostics)

//Wiznet
int         wizMain(void);
char*       inet_ntoa(unsigned long addr);
extern int  loopback_tcps(SOCKET s, uint16 port, uint8* buf,uint16 mode);
extern int  loopback_udp(SOCKET s, uint16 port, uint8* buf,uint16 mode);
extern void GW_processing(int mSecs, int port);
extern char  eRecDatBuf[4][CHAR_BUF_SZ_1600];   //socket(s) command line buffer size

//FLASH Chip
struct phyHeader phyHdr;
extern uint8     Op[];



//terminal i/o buffers
char        Buf1500[1500];
uSHT        BUF2048[2048+64];
char        g_lineBuf[100];
char        tBufHoldUsb[CmdSiz80+1];
char        tBufHoldSoc[CmdSiz80+1];
char        tBuf[600];


//EMAC structure
extern hdkif_t hdkif_data[MAX_EMAC_INSTANCE];   //defined in 'emac.c'

//Structures
struct      netinfo_s    netInfo;               //Network Active setup data
struct      Controller_tdcHdr   Cntrl_tdcHDR, tHDR_STORE;
struct      structFPGA_Trig_Cnts FPGA_Trig_Stat;
struct      blocksnd    BinSt;
struct      blocksndWrd BinStRDX;
volatile    struct msTimers mStime; 
struct      udpHeader udpHdr;
struct      udpMessage  udpMsg;
struct      HappyBusReg HappyBus= {0,1,0,0, 0,0,BUF2048};   //cmdSiz, brd, Cndtype, sndwcnt,   CntrlNumb, revErrs, srcDataBuf;
//struct    HappyBusReg HappyBus= {0,1,0,0, 0,0,1,0,BUF2048};   //cmdSiz, brd, Cndtype, sndwcnt,  myLinkNumber, myLinkCntrlTMP, myLinkCntrlOK, revErrs, srcDataBuf;

//ePHY download struc
struct      uSums uPhySums;

extern      struct phyPAC_CNTRL pCtrl;                          //all daq packets i/o control
extern      struct g_sciTransfer g_sciTransfer_t[2U];
extern      struct g_sciTransfer
            {
             uint32   mode;         /* Used to check for TX interrupt Enable */  
             uint32   tx_length;    /* Transmit data length in number of Bytes */
             uint32   rx_length;    /* Receive data length in number of Bytes */  
             uint8    * tx_data;    /* Transmit data pointer */  	
             uint8    * rx_data;    /* Receive data pointer */  
            }g_sciTransfer_t[2U];

//dma structure
g_dmaCTRL g_dmaCTRLPKT;                         //dma control packet config stack

//com-port setup
char*       cbufptr = g_lineBuf;
int         cmdIndex =0;
char*       g_paramPtr;
int         short tREG30= 0x500;  //temp test registger

uint32      volatile iFlag=0, genFlag=0, BootFlag=0;
uint16      g_dat16;
uint32      g_dat32, g_Pass=0;
uint32      g_SendBinReqSizeOver=0;

//com portbuffers
char        cmdbuf[100];
uint8       comBuf[80];             //input commands process buffer
uint8_t     USB_inBuf[USB_inBufSz];
struct      vB USB_Rec;

//header buffers
uint16_t    rdbHdrSOCK[cCntlHdrSizWrd];         //hold spill hdr for socket send using 'RDB'
uint16_t    rdbHdrSOCK_NULL[cCntlHdrSizWrd];    //8 word null header

//status block buffers
#define     sBLKSIZ38   38
#define     sBLKSIZ22   22
struct stBlock PoolData;   //replaces 'statusBlock[5][sBLKSIZ38+2]' for data pooling

uint16      statusBlock[5][sBLKSIZ38+2];

//OneWire buffers for pooled data
int         oneWireErr;             //oneWireData bad readings
int         oneWireTemp[4][4];      //oneWireData 4-FPGAs, 8ch-TEMPs
unsigned long oneWireRom[4][4];     //oneWireData 4-FPGAs, 8ch-ROMs

//Wiznet Buffers
int         eTout[MAX_SOCK_NUM]= {0,0,0,0, 0,0,0,0} ;  //store socket timeout status from intr read
uint8       I_STATUS[MAX_SOCK_NUM];             //store socket status for backround processing


char        eCmdBuf[MaxSock][eRecSz1024];
char*       eRecDatPtr[MaxSock];                //socket(s) command line buffer size
char*       eCmdDatPtr[MaxSock];                //socket(s) data prts
unsigned    int sLen[MaxSock];


//adc buffer, 16 channels plus one temp/work channel [17]
adcData_t   adc_data[17];               



//DMA Wiznet
#define     D_SIZE      4*256
uint16      RDB_dmaDATA[D_SIZE];            //FPGA-MEMORY-WIZNET buffer in sys ram 


//emac stuff
extern struct phyPACSTAT paKet;             //all daq packets, status/counter
extern struct phyPAC_ERRORS PaketStats;     //storage for packet errors, counters


//Uptime capture from 'sys_main.c'
uint32  Up_Time;

//misc
int     esm_Ch=0;
int     esm_Cnt=0;
int     fpga_ld_cnt=0;
int     g_lcnt=0;
int     adc_ms=0;
int     g_Sock=0;
int     d_nErr=0;
short   g_signD16[3];    
long    param1, param2, param3;         //signed intergers
int     CmbActiveCnt[4];                //count for act ch on each fpga

//nHet temperature readings and PWM
unsigned int Ris2Fall, Fall2Ris, cmdTime; 
int     Duty_Period1,Duty_Period2,Duty_Period3,Duty_Period4;
int     Set_Duty_Period1;

//Version Info
div_t   divV;                   //struc for div() quot,rem  code ver ##
div_t   divR;                   //struc for div() quot,rem


//fpga(4 ea) reg setup structure (defaults)
FPGA_RegS  f_RegConst [FP_REGS_MAX+1] = {  
        { 0x00, 0x02, FP_REGS_VALID0,rLOADED}, //Valid Flag and Reg Load Count  **row1
        
        //rAdr, rCnt, Data[0-8] data filled in as needed                        
        { 0x00, 0x01, 0x0C},    //addr 000 CSR                                  **row2
        { 0x20, 0x01, 0x04},    //addr 020 MUX                                  **row3
        { 0x21, 0x01, 0x0FFFF}, //addr 021 CH MASK                              **row4
        { 0x22, 0x01, 0x00},    //addr 022 R/W TEST Cnt HI                      **row5
        { 0x23, 0x01, 0x03},    //addr 023 R/W TEST Cnt LO                      **row6
				
        { 0x30, 0x08, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800}, //addr 030 DAC TRIM BIAS (8 CH)  **row7
        { 0x38, 0x08, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800}, //addr 038 DAC TRIM BIAS (8 CH)  **row8
        { 0x40, 0x04, 0x000, 0x000, 0x000, 0x000},  //LED Flasher               **row9
        { 0x44, 0x02, 0x000, 0x000},                //BIAS DACs                 **row10
        { 0x46, 0x02, 900, 900},	                //AFE VGA DACs              **row11
	    { 0x74, 0x01, 0x00},                        //PULSE Trig Delay          **row12
        { 0x80, 0x08, 0xFFC, 0xFFC, 0xFFC, 0xFFC, 0xFFC, 0xFFC, 0xFFC, 0xFFC}, //addr 080 DAC PEDESTAL REGs (8 CH)  **row13
        { 0x88, 0x08, 0xFFC, 0xFFC, 0xFFC, 0xFFC, 0xFFC, 0xFFC, 0xFFC, 0xFFC}, //addr 088 DAC PEDESTAL REGs (8 CH)  **row13
		
        //afe chip @0x100
        {0x100, 0x04, 0x000, 0x000, 0x000, 0x000},                              //**row14
        {0x104, 0x08, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},  //**row15
        {0x133, 0x02, 0x2004, 0x4140},                                          //**row16
		//afe chip @0x200
        {0x200, 0x04, 0x000, 0x000, 0x000, 0x000},                              //**row17
        {0x204, 0x08, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},  //**row18
        {0x233, 0x02, 0x2004,0x4140},                                           //**row19
        //addr range 300-308
        {0x300, 0x08, 0x000, 0x004, 0x008, 0x002, 0x010, 0x080, 0x000, 0x237},  //**row20
        //addr range 308-30B
        {0x308, 0x04, 0x002, 0x004, 0x000, 0x0C0},                              //**row21
    };

//fpga(4ea) reg setup structure (flashed and restored)
FPGA_RegS1   f_RegHold[4][FP_REGS_MAX+1];

//uC_Store PreSet as {SerNumb, MuxCh, Gain, TrigSrc, LnkDir, nErr, valid, wDogTimOut, 
//                    fpgaECC, EchoFlag, fakeSum16, TTYbaud, fakeCnt32,fakeAddr32}
struct      uC_Store uC_FRAM;  
struct      uC_Store uC_Const= {0,1,1,0, 1,0,VALID,0, 0,1,0,BAUD460800, 0,0};  //fram defaults 
struct      uSums u_SRAM= {0,0,0,0};     //DwnLdSdRamCnt, DwnLdSdRamSum,stat,time

uint8_t	emacAddressAFE[6U] = {0x0U, 0x08U, 0xeeU, 0x03U, 0xa6U, 0x6cU};



#pragma  pack(4)                                //force 16bit boundary on all types
#define PACs  2 
pbuf_t   pack[PACs];                            //used in test command function 'PACS'

#pragma  pack(4)                                //force 16bit boundary on all types
extern  pbuf_t  PacLnkLst[];                    //Packet linked list size



// ****************************************
// **          Code Entry Point          **
// ****************************************
void main_mu2e(void)
{
    int i, retVal;
    divR = div(__VER__, 100);                   //Compiler vers number is integer * 100
    divV = div(MU2EvER, 100);                   //Code vers number is integer * 100
      
    HappyBus.SndWrds=0;                     //prevent bootup lvds buffer flush
    
    //Configure system response to error conditions signaled to the ESM group1
    //This function can be configured from the ESM tab of HALCoGen
    //The nError pin clears on a PORRST\. But with calling the esmInit() 
    //in the HalCoGen-Code, the nError pin will get set again.    
    //read external hardware flip/flop 'NC7SZ175' error latch bit
    d_nErr = hERR_Latch;        //store register status, latch is cleared later

    //init socket buffer pointers
    for (int i=0; i<MaxSock; i++)
        {
        sLen[i]=0;
        eRecDatPtr[i]= eRecDatBuf[i];
        eCmdDatPtr[i]= eCmdBuf[i];              //socket(s) data prts
        }
    
    //Spi DAT1 register configuration, Must do before FRAM access
    //SPI2 setup, SPI2_CS0 'CS0' (N3)       //spiDAT1_t dataconfig_ADC;
    dataconfig_ADC.CS_HOLD = FALSE;
    dataconfig_ADC.WDEL    = TRUE;
    dataconfig_ADC.DFSEL   = SPI_FMT_0;     //data format for 'spi2_cs0' 8bit
    dataconfig_ADC.CSNR    = SPI_CS_0;

    //SPI2 setup, SPI2_CS1 'CS1' (D3)       //spiDAT1_t dataconfig_PGA;
    dataconfig_PGA.CS_HOLD = FALSE;
    dataconfig_PGA.WDEL    = TRUE;
    dataconfig_PGA.DFSEL   = SPI_FMT_1;     //data format for 'spi2_cs1' 16bit
    dataconfig_PGA.CSNR    = SPI_CS_1; 
  
    //SPI3 setup, SPI3_CS0 'CS0' (V10)      //spiDAT1_t dataconfig_FRAM;
    dataconfig_FRAM.CS_HOLD = FALSE;        //keep 'cs' active low
    dataconfig_FRAM.WDEL    = TRUE;         //delay
    dataconfig_FRAM.DFSEL   = SPI_FMT_0;    //data format for 'spi3_cs0' 8bit
    dataconfig_FRAM.CSNR    = SPI_CS_0;     //ChipSelect (0-7)


    //SPI3 setup, SPI3_CS2 'CS2' (B2)       //spiDAT1_t dataconfig_EQUAL EQ_GS3140
    dataconfig_EQUAL.CS_HOLD = FALSE;       //keep 'cs' active low
    dataconfig_EQUAL.WDEL    = TRUE;        //delay
    dataconfig_EQUAL.DFSEL   = SPI_FMT_2;   //data format for 'spi3_cs2' 16bit clk phase0
    dataconfig_EQUAL.CSNR    = SPI_CS_2;    //ChipSelect (0-7)

    //Equalizer Chip GS3140 SPI Data Port Write 16Bit
    //Using SPI3_CS2 (B2) 
    //EQ_GS3140(uint16 *data );    
    
    //tirgger now, cleans up registers
    hDelayuS(1);                //Hardware delay using RM48L nHET timer
            
    CSI_B0_HI
    CSI_B1_HI
    CSI_B2_HI
    CSI_B3_HI

    Suspend_LO                                  //Set suspend low, not used
    //reset fpga(s)
    PROGx_LO                                    //min 300nS
    uDelay(5);
    PROGx_HI                                
    uDelay(2000);                               //fpga reset takes about 800uS
    DSR_LO                                      //LOW enables data flow
      
    //esmTriggerErrorPinReset();
    esmActivateNormalOperation();   //tek look at this

    //below code here for reference only
    //Disable Group 1 Channels Error Signals propagation to the error pin.
    //esmDisableError(uint64 channels);
        
    //flash reset normal
    FlashRst_HI
    //set ethernet reset (DAQ LINK DRIVER) high
    EthDown_HI
    //Wiznet Reset low for at least 2 uSec (i/o Pin Reset)
    ETH_RST_LO    
    uDelay(20);
    ETH_RST_HI

    //UART_SCI_LIN SetUp, use interrupt mode
    /* clear error flags */
    UART->FLR = ((uint32) SCI_FE_INT | (uint32) SCI_OE_INT | (uint32) SCI_PE_INT);
    //needs setup time befor checking
    uDelay(1000);
    //g_sciTransfer_t[1].rx_length = sizeof(mu_BOOT_MSG1);
    //g_sciTransfer_t[1].rx_data   = mu_BOOT_MSG1;
      
       
    //get uC REG data from FRAM, {SerNumb, MuxCh, Gain, Trig, LnkDir, nErr, xPhy, wDogTimOut}
    //load FRAM before dataRecall()
	FRAM_RD(fPAGE_0400+idx_Baud, (uint8*)&uC_FRAM.TTYbaud, 4); //get baud(4bytes) from FRAM
    //check FRAM stored baud valid, then change SCI port to stored baud
    if(uC_FRAM.TTYbaud==BAUD115200)
        sciSetBaudrate(UART, BAUD115200);          //Set Baud
    else if(uC_FRAM.TTYbaud==BAUD230400)
        sciSetBaudrate(UART, BAUD230400);          //Set Baud
    else if(uC_FRAM.TTYbaud==BAUD460800)
        sciSetBaudrate(UART, BAUD460800);          //Set Baud
    else if(uC_FRAM.TTYbaud==BAUD921600)
        sciSetBaudrate(UART, BAUD921600);          //Set Baud
      
    //boot msg
    putBuf(tty, "\r\n", 2);
    putBuf(tty, (char*)mu_BOOT_MSG1, 0);
          
    //ready flash chip
    flashStatus(tty);                   //show flash status
    putBuf(tty, Buf1500,0);
    
    //load fpga from flash data
    retVal= flashXFER(4, tty, FLBASE);  //load all readout FPGA(s)
    //CONFIG FAILED, try FLASH backup
    if (retVal == -1)                   
        {
        uDelay(500);
        flashXFER(4, tty, FLBACKUP);    //load FPGA(s) with backup data
        }
           
    //needs setup time before loading registers
    uDelay(500);
    
    //afe_A on each fpga
    wrAFE_Slow(0, 0x100, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    wrAFE_Slow(1, 0x100, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    wrAFE_Slow(2, 0x100, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    wrAFE_Slow(3, 0x100, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    //afe_B on each fpga
    wrAFE_Slow(0, 0x200, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    wrAFE_Slow(1, 0x200, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    wrAFE_Slow(2, 0x200, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    wrAFE_Slow(3, 0x200, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    
    //General reset to AFE fifo, counter, sequencer
    CSR0= (0x20 | (CSR0));
    CSR1= (0x20 | (CSR1));
    CSR2= (0x20 | (CSR2));
    CSR3= (0x20 | (CSR3));
    
    //get uC REG data from FRAM, {SerNumb, MuxCh, Gain, Trig, LnkDir, nErr, xPhy, wDogTimOut}
    //load FRAM before dataRecall()
    FRAM_RD(fPAGE_0400, (uint8*)&uC_FRAM, FRAM_400wsz); //repeat load, already loaded by dataRecall() tek may2019 fix
      
    //recall FPGA Setup Data from FRAM reloaded structure
    dataRecall(1, fpgaBase);       //fpga0
    dataRecall(1, fpgaBase1);      //fpga1
    dataRecall(1, fpgaBase2);      //fpga2
    dataRecall(1, fpgaBase3);      //fpga3
          
    //Flashed Startup Network, copy flash data to Ram vars (network, baud, ser#)
    FRAM_RD(fPAGE_Net500, (u_8Bit*)&netInfo, sizeof(netInfo)); //16bit writes (write 2 bytes)
    
    //now check if flash data is valid, if not load defs    **** Startup Params Network ****
    if ( netInfo.Valid != VALID)
        {
        putBuf(tty, (char*)mu_BOOT_MSG2,0);                 //Network Setup InValid
        memcpy(&netInfo, &defNetInfo,   sizeof(netInfo));   //load structure with defaults
        }
    
    //ADC_RESET                             (G17, SPI5_SIMO3)
    ADCRESET_LO                             //ADS1259 chip
    uDelay(10);                             //Needs min of 8 fclks
    ADCRESET_HI                             //ADS1259 chip
    ADCSTART_LO                             //ADC Inactive Low
    
    //start adc conversion,  1.206mV per bit
    adcStartConversion(adcREG1,adcGROUP1);
    /* ... wait and read the conversion count */
    while((adcIsConversionComplete(adcREG1,adcGROUP1))==0);
    adcGetData(adcREG1, adcGROUP1,&adc_data[0]);
    //id    = adc_data[0].id;
    //value = adc_data[0].value;

	sprintf(tBuf, "\r\nCompiler Version %s\r\n", __VERSION__ );
    putBuf(tty,tBuf,0);
    
    if (uC_FRAM.NewLinPrmpt==0) 
        putBuf(tty,"Socket NewLine Echo (CRLF>) Off\r\n",0);      

    //Wiznet Reset low for at least 2 uSec
    WIZRST_LO
    uDelay(10);  
    WIZRST_HI       
    uDelay(500);  

    //test if Wiznet chip W5300 active and ACKs with valid data
    i= getMR();
    if (( i==0xB900 ) || ( i==0xB800 ))
        iFlag |= iWiznetACK;
    sprintf(tBuf,"\r\nWiznet Mode= %X (B800,B900=Valid)\r\n", getMR());
    putBuf(tty,tBuf,0);

    //if Wiznet chip not anwering on bus, don't initialize it
    if (iFlag & iWiznetACK) 
        {
        //Wiznet needs delay after reset, wiat 200mS for PLL lock
        if (wizMain()==1)           //wiznet memory error, disable it
            iFlag &= ~iWiznetACK;   //error disable wiznet
        }
    
    //Clear nError latch, do a cycle to clk flip/flop,
    CLR_ERR_HI
    CLR_ERR_LO

    //Initializat DMA Module.
 	DMA_Initialization();
       
    //inits EMACInit() and MDIOInit()
    i= EMACHWInit(emacAddressAFE);         
    if (i!= EMAC_ERR_OK)
        //BootFlag |= MSG3;
        putBuf(tty, (char*)mu_BOOT_MSG3,0);
    else
        //BootFlag |= MSG4;
        putBuf(tty, (char*)mu_BOOT_MSG4,0);
    
#ifdef _FEB_PRODUCTION          //Production board layout allows this test
    //check if network cable connected (WIZLink=0, cable connected)
    if(WIZLink==0)
        sprintf(tBuf,"Local Ethernet Cable Connected\r\n\n");
    else 
        sprintf(tBuf,"Local Ethernet Cable **Not Connected**\r\n\n");      
    putBuf(tty,tBuf,0);      
#endif    
    
    //this is already setup
    //EMACRxPromiscuousEnable() setup in file 'emac.c', must be enabled
  
    //enable receiving of DAQ Request via 'PHY' port
    phyHdr.PacEna=1;
    
    //Enable watchdog timer, file notifications.c
    iFlag |= WHATCHDOGENA;          
    
    //trig 1st ADC Read
    genFlag |= (ADC_REFRESH+ ADC_TRIG);
              
    //make sure DMA byte cnt zero 
    BinSt.gSndBytCnt=0;
    
    //nError readout 'Hardware Mods'
    //Adding second power up timer chip 'STM6510' to prevent clocking the nError
    //latch chip 'NC7SZ175' during normal boot up tests.
    //Read external hardware flip/flop 'NC7SZ175' error latch bit
    
    //hHI_TP46;     //4 testing only
    //hLO_TP46;     //4 testing only
    if(d_nErr==0)
        {
        if (Up_Time > 0xFF0000)
            {
          //sprintf(tBuf,"Up_Time, Detected Soft Start, UpTimeSec= %d\r\n", Up_Time);
            sprintf(Buf1500,"******************************************\r\n");
            sprintf(tBuf,   "** nError okay, Power Cycle nError Test **\r\n");
            strcat(Buf1500,tBuf);
            sprintf(tBuf,   "******************************************\r\n");
            strcat(Buf1500, tBuf);
            putBuf(tty,Buf1500,0);         
            }
        else
            nError();
        }
    
    //background request for CMB data update
    OneWireTrigRead();              //reads CMB data, 4-20uSec

    //adcStopConversion(adcREG1,adcGROUP1);  //tek Jun2019 mod
     
    
    //tek added for noise testing Nov2020
    iFlag &= ~CMB_RD_ENA;          //Disable CMB reads
    putBuf(tty,"CMB Readouts Disabled, see cmd CMBENA\r\n\n",0);
    
    
#ifdef _FEB_PRODUCTION          
    sprintf(tBuf,"FEB Production Board Layout Pin Mapping Compile Option\r\n\n");
#else 
    sprintf(tBuf,"FEB Pre_Production Board Layout Pin Mapping Compile Option\r\n\n");
#endif    
    putBuf(tty,tBuf,0);      
    
    
    //gateway ack req
    GW_processing(500, tty);    

    
    //********************************************************************
    //****************      main loop starts here     ********************
    //****************      main loop starts here     ********************
    //********************************************************************
    while(1)    //Cycle, Checking USB and Sock(s) for input
        {
          
        //**************************************************************
        //******      check if char avail, UART (via USB)        *******
        //**************************************************************        
        if(USB_Rec.Cnt)
            {
            //process 1 input char
            cbufptr =  getline(cbufptr, cmdbufsiz, &cmdIndex);
            //ready  UART/USB Receive
            if (USB_Rec.Cnt==0)                     //in case ptrs get lost
                {
                _disable_interrupt_();
                USB_Rec_Init();
                _enable_interrupt_(); 
                }
            
            if ((*cbufptr=='\r') || (*cbufptr==0x5c))
                {
                *cbufptr = 0;                       //ove rwrite 'cr' with Null terminator/Arnold
                process(tty, g_lineBuf);            //respond to ascii command
                cmdIndex=0;
                cbufptr= g_lineBuf;                 //cmdline check done, reset cmd bufptr
                }
            } //end usb check
        
        //**************************************************************
        //****  Check if new uC ADC Data req by user 'ADC_REFRESH'  ****
        //**************************************************************        
        if (paKet.EMACRxRdy==0)         //skip if ePHY DAQ port busy
            adcRefresh();
       
        //**************************************************************
        //** ARP_REQ flag set on a 5min interval see 'notification.c' **
        //** Only if no sockets are connected get Gateways attention  **
        //** sock4 used for keep alive ARP packet, see 'wizmain.c'    **
        //**************************************************************        
        if (genFlag & ARP_REQ)  
            arpKeepAlive();  

        //**************************************************************
        //****    Trig CMB oneWire Reads if flaged               *******
        //****    CMB reads must be enabled by cmd 'CMBENA'      *******
        //**************************************************************  
        if (paKet.EMACRxRdy==0)         //skip if ePHY DAQ port busy
        if (genFlag & (OneWireDatRdy|OneWireDatReq) )
            if(iFlag & CMB_RD_ENA)      //allow disabling due to noise issues
                OneWireTrigRead();      //reads CMB data, 4-20uSec
        
        //**************************************************************
        //******   WatchDog Timer uC Reset, Uses WHATCHDOGENA    *******
        //******   Commented out code check in notifications.c   *******
        //**************************************************************        
        //mStime.WatchDog=0;        //zero out timer will not reset uC
        
                
        //***************************************************************
        //********    fpga checkUP for CRC Error flag status     ********
        //***************************************************************  
        if (paKet.EMACRxRdy==0)     //skip if ePHY DAQ port busy
        {
        i= fpga_Check();           //5==check all fpga status
        if (i)                     //updates cmd 'ID' reg "FPGA ECC Count"
            {
            uC_FRAM.FpgaEccCnt++;
            FRAM_WR_ENABLE();                      
            FRAM_WR(fPAGE_0400+idx_FpgaEccCnt, (uint8*)&uC_FRAM.FpgaEccCnt, 2); //addr,data,cnt fPAGE_0400
            FRAM_WR_DISABLE();
            fpga_ld_cnt++;
            if (fpga_ld_cnt < 5)           //LIMIT messages
                {
                //note use msg in loader  
                sprintf(tBuf,"  FPGA ECC_ERR (BIT0=FPGA0,...=%X) , Total=0x%X (show 1st 5 err)\r\n\n",i, uC_FRAM.FpgaEccCnt);
                putBuf(0, tBuf,0);          //force msg to sock 0
                putBuf(ePhyIO, tBuf,0);     //send msg on ePHY link for controller to display
                }
            }       
        }       
        //***************************************************************
        //********           WIZNET ETHERNET CHECK            ***********
        //********  If Wiznet error, skip else error lockup   ***********
        //***************************************************************        
        if (!(iFlag & iWiznetACK)) 
            continue;           //wiznet ethernet failed return to while(1)
      

        //**************************************************************
        //***  ePHY port on uC speed testing func, see cmd 'RDX'   *****
        //***  Command 'RDX' starts test                           *****
        //***  Any remaining data from 'RDX' command to send?      *****
        //***  Send next packets if wiznet okay                    *****
        //***  If Wiznet error, skip else error lockup             *****
        //**************************************************************           
        if (BinStRDX.gSndWrdCnt)    //ePHY port on uC speed testing func
            {
            if (iFlag & iWiznetACK) //wiznet has to be active else lookup
                sendRDX();
            }


        //*************************************************************************
        //***  Wiznet socket check done, now check uC built in MAC PHY        *****
        //***  Check for incoming cmd packet, Note: DAQ Hardware (not Wiznet) *****
        //***  Any remaining data from last recd buf to send?                 *****
        //***  Send it before processing next cmd                             *****
        //*************************************************************************           
        
        //Sequence on each ethernet port 'Socket', Check for Rec Data Available
        if ((g_Sock == ePhyIO) && (sLen[ePhyIO]==0)) 
            {
            //***************************************************************
            //********        uC ePhy DAQ Port checked first      ***********
            //***************************************************************   
            sLen[ePhyIO]= phy_Pac_check();      //sLen[] holds rec byte count    
            }       
        else
            {
            //***************************************************************
            //********        If WizNet OKay, Check Sockets          ********
            //********        Ethernet Cable must be connected       ********
            //*************************************************************** 
            int nFlag;                  //temp var
            //update WizACK flag by reading Wiznet Chip id data
            nFlag= getMR();         //expect 0xB900 or 0xB800
            if ((nFlag==0xB900) || (nFlag==0xB800))
#ifndef _FEB_PRODUCTION             //not production but Pre_production boards          
                nFlag= 1;           //non zero status good
#else            
//Production Brds code here      
                iFlag |= iWiznetACK;     //status good
            else
                iFlag &= ~iWiznetACK;    //status error           
            nFlag= (iFlag& iWiznetACK);  //nFlag=1=Wiznet Ack Okay
            if ((WIZLink==1)||(nFlag==0))//WIZLink=1=NoCable Attached  or nFlag=0=Error
                sLen[g_Sock]=0;          //reset socket rec'd byte cnt, dump possibly bad byte cnt            
            if ((nFlag)&&(WIZLink==0))   //Continue if socket is okay and cable connected
#endif  
#ifndef _FEB_PRODUCTION                  //not production but Pre_production boards          
            if (nFlag)                   //note: Bad Wiznet Status reads can hang up uC code
#endif              
                {
                //1st check for any active 'DAQ Block Data Move'
                //if active, sends until gBusy==0
                if ((BinSt.gSndPrt==g_Sock) && (BinSt.gSndBytCnt!=0))//binary block send mode active
                    {
                    if (BinSt.gSndMode)                     //0=reg mode, 1=daq mode
                        sendBin(BinSt.gSndPrt);             //send bin data DAQ Req, 'cmd RDB'
                    else
                        sendBinNonDAQ(BinSt.gSndPrt);       //send bin data non DAQ Req, 'cmd RDBR'
                    }
                //Now Check Sockets, Cycle thru socket(s) checking for new rec'd data        
                if ((g_Sock == sock0) && (sLen[sock0]==0))  //seq on each sock
                    {
                    //read socket buffer, multi commands may be in buffer, one cmd processed per pass
                    eRecDatPtr[g_Sock]= eRecDatBuf[g_Sock];
                    //socket, port, buffer, mode(Sn_MR for no delay)
                    sLen[sock0]= loopback_tcps(g_Sock,netInfo.sTelnet0,(uint8*)eRecDatBuf[g_Sock],0);  
                    }
                else if ((g_Sock == sock1) && (sLen[sock1]==0)) //seq on each sock
                    {
                    //read socket buffer, multi commands may be in buffer, one cmd processed per pass
                    eRecDatPtr[g_Sock]= eRecDatBuf[g_Sock];
                    //socket, port, buffer, mode
                    sLen[sock1]= loopback_tcps(g_Sock,netInfo.sTelnet1,(uint8*)eRecDatBuf[g_Sock],0); 
                    }
                else if ((g_Sock == sock2) && (sLen[sock2]==0)) //seq on each sock
                    {
                    //read socket buffer, multi commands may be in buffer, one cmd processed per pass
                    eRecDatPtr[g_Sock]= eRecDatBuf[g_Sock];
                    //socket, port, buffer, mode
                    sLen[sock2]= loopback_tcps(g_Sock,netInfo.sTelnet2,(uint8*)eRecDatBuf[g_Sock],0);
                    }
                }
            }
        
        //*************************************************************************
        //***   If bufs have data process it for valid '\r' terminated cmds   *****
        //***   Buffer may have more commands depending on sLen[g_Sock]       *****
        //***   only one command processed per socket loop                    *****
        //*************************************************************************           
        if (sLen[g_Sock])                               //char based tty telnet port
            {                                           //each packet may contain only on char.
            char ch;
            I_STATUS[g_Sock]=0;
            unsigned int cnt=0;
            
            //overflow
            if(sLen[g_Sock]>= eRecSz1024)               //to much, dump it
                cnt= 0;
            
            for (cnt=0; cnt< sLen[g_Sock]; )
                {
                cnt++;
                ch= myupper(*eRecDatPtr[g_Sock]);       //upper case each char, data from sockets rec buffer
                eRecDatPtr[g_Sock]++;
                *eCmdDatPtr[g_Sock]= ch;
                if((ch== BACKSP)||(ch== DELETE))        //backspace clean up
                   {
                   //dont send telnet to UDP port, will hang up
                   if (eCmdDatPtr[g_Sock]> eCmdBuf[g_Sock]) //ptr must be greater than buf beg
                        {
                        eCmdDatPtr[g_Sock]--;
                        *eCmdDatPtr[g_Sock]=0; 
                        putBuf(g_Sock,"\b \b",3);       //clean up remote terminal char
                        }
                   break;
                   }
                else if(ch < ' ')
                   {//skip newline char, assume it came along with a '\r'                    
                    if (ch != '\n')                     //'cr' only for command terminator
                        {
                        *eCmdDatPtr[g_Sock]= 0;         //mark end of buffer with null
                        if(eCmdBuf[g_Sock]!=0)
                            process(g_Sock,(char*)eCmdBuf[g_Sock]); //respond to ascii cmd
                        }
                    //any control char ? plus 'cr', reset buffer
                    //ready buffer for next cmd load
                    eCmdDatPtr[g_Sock]= eCmdBuf[g_Sock];
                    //exit this socket command line check even if sLen not zero
                    //return after normal loop of tty and socket(s) data avail check
                    break;
                   }
                eCmdDatPtr[g_Sock]++;  
                //overflow check, cleanup
                //tek 12-12-18, fix pointer compare, when did this happen??     
                //if ( eRecDatPtr[g_Sock]> (eCmdBuf[g_Sock]+eRecSiz) ) //full, just flush it
                if ( sLen[g_Sock] > eRecSz1024  )           //full, just flush it
                     eRecDatPtr[g_Sock]= eCmdBuf[g_Sock];   //end of buffer cleanUp                
                }
            sLen[g_Sock]-= cnt;
            if (g_Sock== ePhyIO)        //only allow one command per buffer if 'Phy' port
                sLen[ePhyIO]=0;         //'Phy' buffer has extra padding 'to force phy xfer'
            }        
        
        //**********************************************           
        //*********   Cylce all ports        ***********
        //**********************************************           
        if (++g_Sock > LoopEnd)     //scan sock0, sock1, sock2 and ePhyIO (LoopEnd==3)
            g_Sock=0;
        }                           //end main while loop
}




//USB character xmit using uC UART to CP2104 
void sciUartSendText(sciBASE_t *sci, uint8 *text,uint32 length)
{
    unsigned timer;
    while(length)
        {
        timer=0;
        //warning, code has no breakout
        while ((UART->FLR & 0x4)== 4)   //wait until busy
        {
         if (timer++> 10000)
             {
             sciSendByte(UART,'\r');    //send char, may prevent "FLR status hangup on boot"
             break;                     //requires power cycle, give up
             }
        }
        sciSendByte(UART,*text++);      //Xmit single character, see 'sci.c'
        length--;        
        }
}



//Get Command line of USB data terminated with 'cr'
//Move received data to Cmd Buf
char * getline( char* cptr, int cmdbufLen, int * cmdIndex)
{
    int inCh ;
    for (;1;)
        {
      //inCh= sciReceiveByte(UART);
        inCh= USB_Rec.Buf[USB_Rec.RdPtr];
        USB_Rec.RdPtr++;
        //sub cnt, only called here because of USB_Rec.Cnt>0
        USB_Rec.Cnt--;
        //reset storage ptr if at end of buffer
        if (USB_Rec.RdPtr >= InBufSiz_512)
            USB_Rec.RdPtr   = 0;
        
        inCh = toupper(inCh);                       //upper it
        *cptr= inCh;
        if (inCh == BACKSP)
            {
            if ( cptr > g_lineBuf )                 //stop at beginning
                {
                putchar('\b');
                putchar(' ');
                putchar('\b');
                cptr--;
                }
            break;
            }
        if (inCh == '\r')                           //EOL
            {
            putchar(inCh);                          //echo rec'd character
            break;
            }
        if (*cmdIndex >= (cmdbufLen-2))             //Sizeof(cmdbuf)) plus append Null
            {
            *cptr= '\r';                            //buf full, force EOL
            break;
            }
        *cmdIndex+= 1;                              //incr char counter
        cptr++;
        putchar(inCh);                              //echo rec'd character
        break;
        }
    return cptr ;                                   //return byte count
}


//this function used by putch and printf
size_t __write(int Handle, const unsigned char * Buf, size_t size)
{
    int nChars = 0;
    //Check for stdout and stderr
    //(only necessary if file descriptors are enabled.
    if (Handle != 1 && Handle != 2)
        {
        return 1;
        }
    for (; size > 0; --size)
        {
         putchar(*Buf++);
        ++nChars;
        }
    return nChars;
}



#include <yfuns.h>
size_t __read(int handle, unsigned char * buffer, size_t size)
{
  /* Remove the #if #endif pair to enable the implementation */
#if 1    
  int nChars = 0;
  /* This template only reads from "standard in", for all other file
   * handles it returns failure. */
  if (handle != _LLIO_STDIN)
  {
    return _LLIO_ERROR;
  }

  for (/* Empty */; size > 0; --size)
  {
    int c = *g_paramPtr++; //uart_getchar(TERM_PORT);
    if (c < 0)
      break;

    *buffer++ = c;
    ++nChars;
  }
  return nChars;
#else
  /* Always return error code when implementation is disabled. */
  return _LLIO_ERROR;

#endif
}

//Output char string to active i/o stream, USB, Socket(s)
void putBuf(int prt, char* sBuf,  int len)
{
    char * p = sBuf;
    char fill[4];
    if (len==0)  
        {while (*p++) len++;}               //size of Null term data
    if(len==0)
        {
        len=0;                               //diagnotics, this should not happen
        return;
        }

    //now send to active socket or USB
    //send  socket(ch,buffer,length)
    if (prt==sock0)                         //sock0 telnet user mode
        send(sock0, (uint8*) sBuf, len);    
    else if (prt==sock1)                    //sock1 telnet user mode
        send(sock1, (uint8*) sBuf, len);    
    else if (prt==sock2)                    //sock2, telnet user mode
        send(sock2, (uint8*) sBuf, len);    

    
   else if (prt==ePhyIO)           //FEB I/O, uses LVDS port here 
        {
         if (len==1)                //only one char to send, maybe from a constant buffer
            {                       //add char and send from ramBuf
            fill[0]= '\r';          //filler
            fill[1]= *sBuf;         //actual char
            sBuf= fill;             //change pointer
            }
         else
            {
            len++;                  //if odd bytes, must send extra char that sld be NULL
            len= len/2;             //Replys on LVDS FM PORT with even byte count
            }

        //send in background using rtiNOTIFICATION_COMPARE2 intr in 'notifications.c'
        if (HappyBus.SndWrds==0)
            {
            //valid command range at the FEB rec coding is 0x70-0x7F for data echo via putBuf()              
            if ((HappyBus.CmdType<0x70) || (HappyBus.CmdType>0x80) || (HappyBus.BrdNumb> 24) ) //err trap
                {
                HappyBus.PHYrecvErr++;
                return;
                }
            _disable_interrupt_();
            HappyBus.SndWrds= len+3;
            HappyBus.Src= (snvPTR)BUF2048;          //done, init to src begin
            *HappyBus.Src++= HappyBus.BrdNumb;      //send data on LVDS PORT
            *HappyBus.Src++= HappyBus.CmdType;      //send data on LVDS PORT
            *HappyBus.Src++= HappyBus.SndWrds;      //send data on LVDS PORT
            movStr16((snvPTR)sBuf, (snvPTR)HappyBus.Src, len+3);  //word to lwords
            HappyBus.Src= (snvPTR)BUF2048;          //reset to beg for backgrnd xmits
            _enable_interrupt_();
            }
        else if (HappyBus.SndWrds+len < (sizeof(BUF2048)>>1)) //add more to buf if not full (ints to charBuf)
            {
            if(len!=0)  //traps sending prompt char '>'
                {
                _disable_interrupt_();
                snvPTR BufInx= HappyBus.Src + HappyBus.SndWrds;
                *BufInx++= HappyBus.BrdNumb;      //send data on LVDS PORT
                *BufInx++= HappyBus.CmdType;      //send data on LVDS PORT
                HappyBus.SndWrds += len+3;
                *BufInx++= len+3;                //send data on LVDS PORT               
                movStr16((snvPTR)sBuf, (snvPTR)BufInx, len+3);  //word to lwords
                _enable_interrupt_();
                }
            }
        else
            {
            //buffer full
            sprintf((char*)BUF2048,"\r\nBUF2048 Overrun\r\n");
            HappyBus.SndWrds= 34/2;
            //reset ptr now that .wCnt has changed, more data may come
            HappyBus.Src= (snvPTR)BUF2048;
            }

        //trigger timer interrupt rtiNOTIFICATION_COMPARE2 in 'notifications.c
        LVDS_Intr_Time(1);                      //Triggers interrupt xmit sequence  
        }
      
    else if (prt==tty)
        sciUartSendText(UART,(uint8*)sBuf,len); //send ascii 'tty'
    return;
}


//------------------------------------------------------------------------------
//Read NULL terminted ascii string, Return 1st token begPtr and endPtr, else NULL
//

char *mytok(char *str, char **paramPtrp)
{
    char *s = *paramPtrp, *p = 0;
    if (str) s = str;                             //first call: use str
    //note no buffer end check below other than NULL
    while (*s && *s==' ') s++;                    //start token after initial spaces
    if (!*s) {                                    //no new token found => NULL
        *paramPtrp = s;
        return 0;
    }
    str = s;                                    //start of what we'll return
    p = s;                                      //now see where token ends
    while (*p && *p!=' ') p++;                  //advance until space or NULL
    if (*p) {                                   //haven't seen end of string =>
        *p = 0;                                 //terminate token
        s = p+1;                                //where we'll resume next call
    } else {                                    //we're at end of string =>
    s = p;                                      //next call we'll find NULL
    }
    *paramPtrp = s;                             //*paramPtrp points to paramPtr of string
    return str;                                 //return token (or NULL)
}



//------------------------------------------------------------------------------
//Max 32 Bit Hex Token, using signed data defval returned on data type error.
long arg_hex(char **paramPtrp, long defval)
{
    long val = defval;
    char *tok = mytok(*paramPtrp, paramPtrp);
    char *tokPtr = tok;
    while (*tokPtr!=0)
        if (isxdigit(*tokPtr++)==0)
        return defval;      //check for bad data type

    if (!tok || !((*tok>='0' && *tok<='9') ||
        (*tok>='A' && *tok<='F')))
    return defval;

    val = 0;
    while (1) {
        if (*tok>='0' && *tok<='9') {
        val = ((val<<4) & 0xffffffff) | ((*tok)-'0');
        } else if (*tok>='A' && *tok<='F') {
        val = ((val<<4) & 0xffffffff) | (0xA+(*tok)-'A');
        } else {
        break;
        }
        tok++;
    }
    return val;
}



//------------------------------------------------------------------------------
//Max 32 Bit Decimal Token, using signed data defval returned on data type error.
long arg_dec(char **paramPtrp, long defval)
{
    long val = defval;
    char *tok = mytok(*paramPtrp, paramPtrp);
    char *tokPtr = tok;
    while (*tokPtr!=0)
        if (isdigit(*tokPtr++)==0) return defval;       //check for bad data type


    if (!tok || !((*tok>='0' && *tok<='9'))) return defval;
    val = 0;
    while (1) {
        if (*tok>='0' && *tok<='9') {
        val = ((val*10) & 0xffffffff) + ((*tok)-'0');
        } else {
        break;
        }
        tok++;
    }
    return val;
}




//Non interrupt version of xmits 'polling'
//while( !(USARTx->SR & USART_FLAG_RXNE)); //polling rec not empty
//while( !(USART2->SR & USART_FLAG_TXE));  //polling xmit empty

//load uart transmit buffer with one character
//1st wait for active xmits to end
int putchar(int  c)
{
    sciUartSendText(UART,(uint8*)&c,1);   //send 1 byte
    
    return (c);
}




//Input Command Handler
//Check if rec'd command is valid and handle it
int process(int prt, char *cmdPtr)                  
{
    sPTR p_sAddr;
    cPTR p_cAddr;
    char *tok = 0, *paramPtr= 0, cmdBeg;
    unsigned int parErr=0;
    short ped;
    unsigned short Slp;
    unsigned int Slp32;

    cmdBeg= *cmdPtr;
    paramPtr = cmdPtr;                                  //ptr to data buffer       
    tok= mytok(paramPtr, &paramPtr);                    //find 1st token, should be acsii 'CMD'
    
    //new line prompt logic
    //USB and Sockets always block '\r\n>' echo for all 'R'eads, or cmds that begin with 'R'
    //USB and Sockets always block '\r\n>' echo for all 'W'rites or cmds that begin with 'W'
    //'ECHO OFF' for USB has no other impact
    //'ECHO OFF' for Sockets stops all '\r\n>' prompts
    
    if (tok)                                            //no token found, do error check then return
        {
        //if not USB, Always Stop Prompt for 'Read' and 'Writes', helps DAQ coding
        if (uC_FRAM.NewLinPrmpt==1)                     //No_Prompt Mode?
            {
            if(prt==tty)
                putBuf(prt,"\r\n",2);                   //tty usb okay
            else if(prt!=ePhyIO)                        //no echo on ePhyIO, screws up data pool
                {
                if((cmdBeg!='R') && (cmdBeg!='W'))      //Sockets, STOP NEW LINE ECHO ON READS and WRITES
                    putBuf(prt,"\r\n",2);               //non rd,wr echo prompt
                }
            }
        }
    else
        { //no token, empty command input line, add special prompts for errors
        //TEST BEAM CODE MODS, Prompts mess up DAQ coding
        if (uC_FRAM.NewLinPrmpt==1)                    //Echo On, no token returns prompts
            {
            //continue, more error checks
            if (iFlag & CONFIGFAIL)                     //fpga error
                {
                sprintf(Buf1500,"FPGA_ERR>");
                putBuf(prt, Buf1500,0);
                }
            else if (iFlag & CONFIGBACKUP)              //fpga backup error
                {
                sprintf(Buf1500,"FPGA_BACKUP>");
                putBuf(prt, Buf1500,0);
                }
            else if (!(iFlag & iWiznetACK))             //if Wiznet error
                {
                sprintf(Buf1500,"SOCKET_ERR>");
                putBuf(prt, Buf1500,0);
                }      
            else
                putBuf(prt,"\r\n>",3);                                      
            }
        else //tty allows Echo Off no token input lines to returns prompts
            if(prt==tty)
                putBuf(prt,"\r\n>",3);                                      
        
        return 1;
        } //end of no token error checking
    

    switch (*tok)                                       //command parameter search
        {
        case 'A':
                if (!strcmp(tok, "ADC"))                //get uC adc results
                   {
                    static float flt;
                    int i;
                    param2= arg_dec(&paramPtr,0);      //get 1st param, 1==send data only, no text
                    //Flag for data buffer update
                    genFlag |= (ADC_REFRESH+ ADC_TRIG);                                       
                    *Buf1500=0;
                    for (i=0; i<16; i++)
                        {
                        flt= adc_data[i].value * adcScale[i];
                        if (param2==0)
                            sprintf(tBuf,"%s  %5.2f  ch%02d=%4X\r\n", adcName[i], flt, i, adc_data[i].value);
                          //sprintf(tBuf,"%s  %5.2f\r\n", adcName[i], flt);
                        else
                            sprintf(tBuf," %4.2f",flt);
                        strcat(Buf1500, tBuf);
                        }
                    //show temperature
                    flt= readTemperature();
                    if (param2==0)
                        sprintf(tBuf,"%s  %5.2f\r\n", adcName[i], flt);
                    else
                        sprintf(tBuf," %5.2f\r\n",flt);
                    strcat(Buf1500, tBuf);
                    strcat(Buf1500, "\r\n\r");              //extra char if odd byte cnt on 16bit LVDS xfer
                    putBuf(prt, Buf1500,0);                    
                    break;
                    }
                else if (!strcmp(tok, "A0"))                //ADC_ads1259 reads using 8bit spi mode
                    {
                    float f24, f24avg=0;                    
                    param1= arg_dec(&paramPtr,1);           //get 1st param, 1==send data only, no text
                    if (param1>10)
                        param1=10;
                    
                    //todo get data via intr in background 'ADC_REQ','ADC_DONE'
                    for(int i=0; i< param1; i++)
                        {
                        //Positive full-scale  7FFFFFh
                        //Negative full-scale  800000h      
                        f24= ADC_ads1259(param1);           //read ch, takes 100mSec
                        f24avg += f24;
                        sprintf(tBuf,"%8.6f\r\n", f24);
                        putBuf(prt, tBuf,0);                //send to current active port                        "
                        }
                    if(param1>1)
                        {
                        f24avg= f24avg/param1;
                        sprintf(tBuf,"%8.6f avg\r\n", f24avg);
                        putBuf(prt, tBuf,0);                //send to current active port
                        }
                    break;
                    }  
                else if (!strcmp(tok, "AFERD"))             //16 bit AFE chip read via fpga 
                    {
                    param1= arg_dec(&paramPtr,1);           //1st param, fpga to process 1-4
                    if((param1>4)||(param1<1)) 
                      { parErr++;  break; }
                    param2= arg_hex(&paramPtr,0x133);       //2nd = address
                    param3= arg_hex(&paramPtr,1);           //3rd = count
                    for (int i=1; i<=param3; i++)
                        {
                        g_dat16= readAFE_Slow(param1, param2++); //read 16bit AFE reg                
                        if (i%16==0)                        //14 words per line, now 16
                            sprintf(Buf1500,"%4x\r\n",g_dat16);
                        else
                            sprintf(Buf1500,"%4x ",g_dat16);
                        putBuf(prt, Buf1500,0);             //send to current active port
                        }
                    putBuf(prt,"\r\n",2);                   //send to current active port
                    break;
                    }
                else if (!strcmp(tok, "AFEWR"))             //16 bit AFE chip read via fpga 
                    {
                    param1= arg_dec(&paramPtr,1);           //get 1st param, fpga to process 1-4
                    if((param1>4)||(param1<1)) 
                      param1=1;
                    param2= arg_hex(&paramPtr,0x133);       //get 2nd param address
                    param3= arg_hex(&paramPtr,0);           //get 3rd param  data
                    wrAFE_Slow(param1-1, param2, param3);   //fpga as (0-3), reg, data
                    break;
                    }
                else if (!strcmp(tok, "AFERESET"))          //AFE chip Soft reset
                    {
                    param1=arg_dec(&paramPtr,0);            //get 1st param
                    if ((param1<1) || (param1>5))
                        { parErr++;  break; }
                    if (param1<5)
                        {
                        param1 -=1;
                        //afe_A and afe_B 
                        wrAFE_Slow(param1, 0x100, 0x01);    //fpga(0-3),reg,data (afe soft reset)
                        wrAFE_Slow(param1, 0x200, 0x01);    //fpga(0-3),reg,data (afe soft reset)
                        }
                    else 
                        { //5==do all
                        for (int fpga=0; fpga<4; fpga++)
                            {
                            //afe_A and afe_B 
                            wrAFE_Slow(fpga, 0x100, 0x01);   //fpga(0-3),reg,data (afe soft reset)
                            wrAFE_Slow(fpga, 0x200, 0x01);   //fpga(0-3),reg,data (afe soft reset)
                            }
                        }
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'C':
                if (!strcmp(tok, "CMB"))                    //cmb reads, temp(.0625mV per bit), rom
                    {
                    float degC;
                    param1=arg_dec(&paramPtr,0);            //get 1st param
                    *Buf1500=0;                             //init start as NULL for strcat
                    if (param1)                             //return as raw data
                        {
                        for (int i=0,fpga=0; fpga<4; fpga++)
                            {
                            for (int ch=0; ch<4; ch++, i++)
                                {
                                if(oneWireRom[fpga][ch]==0) 
                                    continue;               //skip if no connections
                                degC= oneWireTemp[fpga][ch]*.0625;  //temp'C'(.0625mV per bit)
                                sprintf(tBuf,"%-4X %X\r\n", (int)degC*10, oneWireRom[fpga][ch]);                    
                                strcat (Buf1500,tBuf);
                                }
                            }
                        putBuf(prt, Buf1500, 0);           
                        break;
                        }
                    sprintf(Buf1500,"Ch    DegC     ROM_ID  (Errs=%d)\r\n",oneWireErr);
                    for (int i=0,fpga=0; fpga<4; fpga++)
                        {
                        for (int ch=0; ch<4; ch++, i++)
                            {
                            if(oneWireRom[fpga][ch]==0) 
                                continue;                       //skip if no connections
                              
                            degC= oneWireTemp[fpga][ch]*.0625;  //temp'C'(.0625mV per bit)
                            sprintf(tBuf,"%2d    %4.1f    %8X\r\n", i+1, degC, oneWireRom[fpga][ch]);                    
                            strcat (Buf1500,tBuf);
                            }
                        }
                    strcat (Buf1500,"\r\n");
                    putBuf(prt, Buf1500, 0);                      
                    oneWireErr=0;                           //clear error counter
                    break;
                    }
                else if (!strcmp(tok, "CMBENA"))            //CMB Readout enable/disable option
                   {
                    param1= arg_dec(&paramPtr, 2);          //get 1st param, rtn 0 for no entry
                    if (param1==1)
                        {
                        iFlag |= CMB_RD_ENA;                //Enable CMB reads at OneWireTrigRead()
                        putBuf(prt,"CMB Readout Enable\r\n",0);
                        }
                    else if (param1==0)
                        {
                        iFlag &= ~CMB_RD_ENA;               //Disable CMB reads
                        putBuf(prt,"CMB Readout Disable\r\n",0);
                        }
                      
                     break;
                    }                                
                if (!strcmp(tok, "CLOSE"))                  //'QUIT' close ethernet socket
                   {
                    param2= arg_dec(&paramPtr, 9);          //get 1st param, rtn 0 for no entry
                    if (param2==9) param2=g_Sock;
                    if ((param2>=0) && (param2<4))          //valid sockets 0-3
                        {
                        setSn_CR(param2,Sn_CR_CLOSE);       //close socket
                        }
                     break;
                    }                
                else  {parErr=0xf;   break; }
        case 'D':
                if (!strcmp(tok, "DRD"))      //DAC Read with Calabration correction            
                    {
                    short signParm16;
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)   //get 1st param (addr)
                        { parErr++;  break; }
                    //get floating point input
                    cFRAMrd(param1,(short*)&Slp,(short*)&ped); //read slope,ped from 'FRAM' 
                    
                    //read 'DAC Reg' via fpga
                    sPTR saddr = (sPTR) fpgaBase;
                    saddr += param1;  
                    param2= *saddr;                 //read DAC reg 0x30-0x3f

                    //offset 
                    ped <<= 4;                      //shift<<4, 12Bits to 16Bit Signed
                    signParm16= param2<<4;
                    ped = signParm16 + ped;
                    ped >>=4;
                    ped &= 0xfff;
                    
                    //2nd inverse slope compared to cmd 'DWR'
                    Slp32= Slp;
                    Slp32= (ped * 0x8000)/Slp;                    
                    Slp32 &= 0xfff;
                    
                    sprintf(Buf1500,"%04X\r\n",Slp32);
                    putBuf(prt, Buf1500,0);                 
                    break;
                    }                
                else if (!strcmp(tok, "DWR"))       //DAC Write with Calabration correction
                    {
                    short signParm16;
                    sPTR saddr = (sPTR) fpgaBase;                    
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param (addr)
                        { parErr++;  break; }
                    if ( (param2=arg_hex(&paramPtr,-1))==-1)//get 2nd param (data)
                        { parErr++;  break; } 
                    cFRAMrd(param1,(short*)&Slp,(short*)&ped); //read slope,ped from 'FRAM'   

                    //1st inverse slope compared to cmd 'DRD'
                    param2=  (Slp * param2)>>15;
                    ped = ped<<4;                   //shift<<4, 12Bits to 16Bit Signed
                    signParm16= param2<<4;          //shift 12bit input to 16Bit Signed
                    ped = signParm16 - ped;         //input minus stored ped, Signed math
                    ped >>=4;                       //return input value to 12Bits, now in ped var
                    ped &= 0xfff;

                    //write DAC register with fRam 'Correction Factor applied'
                    saddr += param1;            //adding to pointer, incr by ptr size
                    *saddr= ped;                //wr scaled DACs value via fpga
                    break;
                    }    
                else if (!strcmp(tok, "DSR"))             //READ FRAM DAC CH Slp/Ped
                    {
                    int short slp;
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param (addr)
                        { parErr++;  break; }
                    //Read DAC CHs slope and pedestal
                    cFRAMrd(param1,(short*)&slp,(short*)&ped); //read slope    
                    
                    sprintf(Buf1500,"slp=%0x  ped=%0x\r\n",(slp&0xffff), ped);
                    putBuf(prt, Buf1500,0);                 //send to current active port
                    break;
                    }
                else if (!strcmp(tok, "DSF"))               //Store to FRAM DAC dsf(addr, slp, ped)
                    {
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param (addr)
                        { parErr++;  break; }
                    if ( (param2=arg_hex(&paramPtr,-1))==-1)//get 2nd param (offset)
                        { parErr++;  break; }
                    if ( (param3=arg_hex(&paramPtr,-1))==-1)//get 3rd param (ped)
                        { parErr++;  break; }      
                    
                    //Valid? FRAM BASE ADDR 1of4 'a00,b00,c00,d00'
                    param1= cFRAMparam2(param1);            
                    if(param1==-1)
                    {parErr=0xf;  break;}
                    
                    //Store DAC correction values
                    FRAM_WR_ENABLE();
                    FRAM_WR(param1,(uint8*)&param2,2);      //(wr 2 bytes) 
                    FRAM_WR_ENABLE();
                    FRAM_WR(param1+0x30,(uint8*)&param3,2); //(wr 2 bytes) 
                    break;
                    }
                else if (!strcmp(tok, "DSI"))           //Init FRAM DACs Slp/Ped
                    {
                    param1=arg_hex(&paramPtr, 0x8000);  //get 1st param (slope)
                    param2=arg_hex(&paramPtr,0);        //get 2nd param (ped)
                    
                    //use cmd 'FRERASE A00 DFF' to clear old FRAM vars
                    //store constants, addr range 0x030-047
                    //use cmd 'FRERASE A00 DFF' to clear old FRAM vars
                    if (param1==99)
                        {
                        genFlag |= NO_ECHO;             //no ASCII reply 
                        sprintf(tBuf,"FRERASE A00 DFF" );  
                        process(prt, tBuf); 
                        }
                    else
                        //Fill DACs constants (scale correction values)
                        cFRAMfill((short*)&param1,(short*)&param2);  //fill in fram DACs slp/ped
                    break;
                    }
                
                //DSAVE fix to READ FPGA REGS
                else if (!strcmp(tok, "DSAV"))              //read/store fpga regs to FRAM
                    {
                    dsav();
                    putBuf(prt,"FRAM UPDATED\r\n",0);       //send to current active port
                    break;
                    }
                else if (!strcmp(tok, "DREC"))              //FLASH- data restore to registers
                    {
                    param1= arg_dec(&paramPtr,1);           //get 1st param, 0=load constant data
                    if ((param1==0) || (param1==1))         //two choices
                        {
                        //read saved FRAM data and Load to FPGA Registers
                        dataRecall(param1, fpgaBase);
                        dataRecall(param1, fpgaBase1);
                        dataRecall(param1, fpgaBase2);
                        dataRecall(param1, fpgaBase3);
                        putBuf(prt,"FPGA Regs Updated, Broadcast Range 0x300 Up restored using FPGA1 setup\r\n",0); //send to current active port
                        }
                    break;
                    }
             
                else if (!strcmp(tok, "DEBUG"))         //Enable command debug mode
                    {
                    param1= arg_hex(&paramPtr,0);       //1st param, port number 1, ePhy/lvds, TTY
                    
                    //port will be tty or sock 1,2,tty(3)
                    if (param1== 1)                     //SOCK1
                        {
                        pCtrl.g_DEBUG_MODE= param1;
                        sprintf(Buf1500,"Send uBunch request errors to Sock1\r\n");
                        putBuf(prt, Buf1500,0);         //send to current active port
                        }
                    else if (param1== 2)                //ePhyIO(lvds back to controller)
                        {
                        pCtrl.g_DEBUG_MODE= param1;
                        sprintf(Buf1500,"Send uBunch request errors to Sock2\r\n");
                        putBuf(prt, Buf1500,0);         //send to current active port
                        }
                    else if (param1== 3)                //USB
                        {
                        pCtrl.g_DEBUG_MODE= param1;
                        sprintf(Buf1500,"Send uBunch Req Errors to USB, Best to set baud 921600 due to delays\r\n");
                        putBuf(prt, Buf1500,0);         //send to current active port
                        }
                    else                                //end debug port number
                        {
                        pCtrl.g_DEBUG_MODE= 0;
                        sprintf(Buf1500,"Debug mode Off\r\n");
                        putBuf(prt, Buf1500,0);         //send to current active port
                        break;
                        }                    
                    break;
                    }                
                else  {parErr=0xf;   break; }
        case 'W':
                if (!strcmp(tok, "WR"))                     //16bit EPI BUS write with 'fRam sub-range'
                    {
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param (addr)
                        { parErr++;  break; }
                    param1 &= 0x7FFFF;

                    if ( (param2=arg_hex(&paramPtr,-1))==-1)//get 2nd param (data)
                        { parErr++;  break; }
                                       
                    //write data to fpga register
                    sPTR saddr = (sPTR) fpgaBase;
                    saddr += param1;                        //adding to pointer, incr by ptr size
                    *saddr= param2;
                    //One Time New LIne Prompt Block on exit for DAQ Apps
                    if(prt!=tty)
                        genFlag |= NO_ECHO;                     //no ASCII reply     
                    break;
                    }
                if (!strcmp(tok, "WRI"))                    //16bit EPI BUS write with 'fRam sub-range'
                    {
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param (addr)
                        { parErr++;  break; }
                    param1 &= 0x7FFFF;

                    if ( (param2=arg_hex(&paramPtr,-1))==-1)//get 2nd param (data)
                        { parErr++;  break; }
                    if ( (param3=arg_dec(&paramPtr,-1))==-1)//get 2nd param (data)
                        { parErr++;  break; }
                    param3 &= 0x3F;                         //limit loop cnt
                                       
                    //write data to fpga register
                    sPTR saddr = (sPTR) fpgaBase;
                        saddr += param1;                    //adding to pointer, incr by ptr size
                    for (int i=0; i<param3; i++)
                        *saddr++= param2;

                    //sprintf(tBuf, "%d writes done\r\n",param3);
                    putBuf(prt,"\r\n",0);
                    break;
                    }
                else if (!strcmp(tok, "WG"))                //Wiznet GateWay Init, for testing only
                    {
                    //for updating gw arp cache
                    //sprintf(tBuf, "Request update on Gateway arp cache\r\n");
                    //putBuf(prt,tBuf,0);
                    GW_processing(700, prt);    
                    break;
                    }
                else if (!strcmp(tok, "WI"))                //Wiznet Init, for testing only
                    {
                    int i;
                    //Wiznet Reset low for at least 2 uSec
                    WIZRST_LO
                    uDelay(10);  
                    WIZRST_HI       
                    uDelay(100);  
                    //test if Wiznet chip W5300 active and ACKs with valid data
                    i= getMR();
                    if (( i==0xB900 ) || ( i==0xB800 ))
                        iFlag |= iWiznetACK;
                    sprintf(tBuf,"\r\nWiznet Mode= %x (B800,B900=valid)\r\n", getMR());
                    putBuf(tty,tBuf,0);
                    //Wiznet startup
                    //wiznet not anwering on bus, dont initialize it
                    if (iFlag & iWiznetACK) 
                        {
                        //Wiznet needs delay, wiat 200mS for PLL lock
                        mDelay(100);
                        if (wizMain()==1)                   //wiznet memory error, disable it
                            iFlag &= ~iWiznetACK;
                        sprintf(tBuf,"\r\nWiznet Mode= %x (B800,B900=valid)\r\n", getMR());
                        putBuf(tty,tBuf,0);
                        }
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'E':
                if (!strcmp(tok, "ECHO"))                   //set new line prompt on,off 'No_Prompt'
                    {
                    param1=arg_dec(&paramPtr,-1);           //get 1st param, password
                    if ((param1==0) || (param1==1))         //off..on newline prompt control
                        {
                        uC_FRAM.NewLinPrmpt= param1;
                        FRAM_WR_ENABLE();                   //WREN op-code issued prior to Wr_Op
                        //update serial number only
                        FRAM_WR(fPAGE_0400+idx_NewLinPrmpt, (uint8*)&uC_FRAM.NewLinPrmpt, 2); //addr,data,cnt
                        FRAM_WR_DISABLE();
                        }
                    if (uC_FRAM.NewLinPrmpt) 
                        sprintf(tBuf,"Echo=1=On\r\n");
                    else
                        sprintf(tBuf,"Echo=0=Off\r\n");
                    putBuf(prt, tBuf,0);
                    break;
                    }      
                
                //nERROR is a push-pull output and is normally driven High. The Error Signaling Module (ESM) 
                //drives this pin low to indicate any group2 and group3 error condition. Also, you can 
                //configure the ESM to drive this pin low on any group1 error condition.
                //When the nERROR pin gets driven low on an error condition the error flag must be cleared 
                //and then 0x5 written to the ESMKEY register in order for the nERROR pin to go High after 
                //the programmed duration has expired.
                //nRST:   write 0x8000 to SYSECR register.
                //nERROR: write 0xA to ESMEKR register.
                            
                // Check if there were ESM group3 errors during power-up.
                // These could occur during eFuse auto-load or during reads from flash OTP
                // during power-up. Device operation is not reliable and not recommended
                // in this case.
                // An ESM group3 error only drives the nERROR pin low. An external circuit
                // that monitors the nERROR pin must take the appropriate action to ensure that
                // the system is placed in a safe state, as determined by the application.
                //
      
                if (!strcmp(tok, "ER0"))        //clr nError Flip/Flop
                    {
                    uC_FRAM.nErrCnt=0;
                    FRAM_WR_ENABLE();                      
                    FRAM_WR(fPAGE_0400+idx_nErrCnt, (uint8*)&uC_FRAM.nErrCnt, 2); //addr,data,cnt
                    break;
                    }
                if (!strcmp(tok, "ER1"))        //test single bit errro
                    {
                    //configure the ESM to drive this pin low on any group1 error condition
                    //esmEnableInterrupt(35);
                    //esmEnableError(35);
                    d_nErr= TrgSinBitErr;       //single bit err wont cause reset
                    mDelay(500);
                    //CLR_ERR, must keep low to allow nError to Reset Board
                    CLR_ERR_HI
                    CLR_ERR_LO
                    break;
                    }
                else if (!strcmp(tok, "ER2"))   //test double bit error
                    {
                    sprintf(tBuf,"nError Test, uC should now reboot.\r\n");
                    putBuf(prt, tBuf,0);
                    mDelay(1);
                    //Software reset will not clear nError once set. (cmd cleared in software)
                    //Have to do PwrReset to toggle uC TRST pin, this allows clean boot.
                    d_nErr= TrgDouBitErr;           //dou bit err will trig nError (extrn reset logic)
                    break;
                   }
                
                else if (!strcmp(tok, "EQR"))       //testing READS S3140
                    {
                    //Command Word 16Bits for write access Bit15(RdHigh/WrLow)
                    //And desired Local Address Reg 16bits
                    param1= arg_hex(&paramPtr, 0);  //get 1st param (addr)
                    if(param1==0)
                       {parErr++;  break;}          //addr zero for addr setup
                    
                    param2= arg_dec(&paramPtr, 1);  //get 2nd param, read count
                    if (param2> 13)
                        param2= 13;
                    
                    param1 |= (BIT15+BIT12);
                    EQR_GS3140((u_16Bit*)&param1, (uint16*)tBuf, param2); //Read src, rtnDest, wordCnt
                    int j=0;
                    while(param2--)
                        {
                        sprintf(Buf1500,"A=%X  D=%X\r\n", param1++, tBuf[j]);
                        putBuf(prt, Buf1500,0);
                        j+=2;
                        }
                    break;
                   }
                else if (!strcmp(tok, "EQW"))        //testing WRITES S3140
                    {
                    //Command Word 16Bits for write access Bit15(RdHigh/WrLow)
                    //And desired Local Address Reg 16bits
                    u_16Bit Cmd[2];
                    param1= arg_hex(&paramPtr, 1);  //get 1st param (addr)
                    if(param1==0)
                       {parErr++;  break;}          //addr zero for addr setup
                      
                    param2= arg_hex(&paramPtr, 0);  //get 2nd param (data)
                    Cmd[0]= param1;
                    Cmd[1]= param2;
                    
                   // for (int i=0; i<10000; i++)
                        {
                        EQW_GS3140(Cmd, 2);         //Read addr, wordCnt
                        mDelay(100);
                        }
                    break;
                    }
                else if (!strcmp(tok, "EQRESET"))   //testing WRITES S3140
                    {
                    //RESET DEVICE
                    tBuf[0]= 0xAA;
                    tBuf[1]= 0xDD;
                    tBuf[2]= 0xAD;
                    //first 3 writes
                  //EQW_GS3140((u_16Bit*)tBuf, 3);                //addr, wordCnt
                    dataconfig_EQUAL.CS_HOLD = TRUE;    //Hold 'cs' low
                    my_spiTransmitData(spiREG3, &dataconfig_EQUAL, 3, (u_8Bit*)tBuf);

                    //now two reads
                    spiReceiveData(spiREG3, &dataconfig_EQUAL, 2, (u_16Bit*)tBuf);
                    dataconfig_EQUAL.CS_HOLD = FALSE;    //release'cs'
                    
                    break;
                   }
                else  {parErr=0xf;   break; }
        case 'F':
                if (!strcmp(tok, "FCOPY"))             //flash copy/backup sectors 0-40 to sectores 41-80
                   {
                    if ( ((iFlag&CONFIGFAIL)) || (iFlag&CONFIGBACKUP) ) 
                        {
                        sprintf(Buf1500,"Can not Copy due to errors, reload FLASH using cmd 'FL1'\r\n");
                        putBuf(prt, Buf1500,0);
                        break;
                        }
                    param1= arg_dec(&paramPtr,0);
                    if (param1==1)
                        {
                        fl_copyFPGA(prt);
                        }
                    else
                        {
                        int  DwnLd_sSUM=0, DwnLd_sCNT=0;
                        FRAM_RD(BACKUP_BYTES,(uint8*)&DwnLd_sCNT, 4);
                        FRAM_RD(BACKUP_CSUM,(uint8*)&DwnLd_sSUM, 2);                        
                        sprintf(Buf1500,"BackUp Bytes=%d  ChkSum=%4X\r\n\r", DwnLd_sCNT, DwnLd_sSUM&0xffff);
                        putBuf(prt, Buf1500,0);
                        }
                    break;
                   }      
                if (!strcmp(tok, "FCOMP"))              //compare flash memory base to backup
                   {                                    //holds FPGA config files
                    int d16U, d16B, err=0, WrdCnt, DwnLd_sCNT=0;
                    u_16Bit SumMain, DwnLd_sSUM=0;
                    u_32Bit CntMain;
                    sPTR saddrBASE, saddrUPPER;
                    
                    //now do local check to verify
                    saddrBASE = (snvPTR) flashBase;
                    saddrUPPER = (snvPTR) flashBase+ S29JL064J_SECTOR41;                                        
                    //get stored fpga backup file size from FRAM
                    FRAM_RD(BACKUP_WCOUNT,(uint8*)&WrdCnt, 4);
                    for (int i=0; i<WrdCnt; i++)
                        {
                        d16U= *saddrUPPER;              //data port, using fpga(2of4) sdRam
                        d16B= *saddrBASE;
                        if(d16U != d16B)
                            {
                            sprintf(Buf1500,"ADDR=%06X      BASE=%4X     BACKUP=%4x\r\n", (int)saddrBASE&0XFFFFFF, d16B, d16U);
                            putBuf(prt, Buf1500,0);
                            if(err++>10)
                                {
                                putBuf(prt, "Exit test after 10 errors \r\n",0);
                                break;
                                }
                            }
                        saddrUPPER++;          
                        saddrBASE++;
                        
                        //compute checkSum for valid data   ADR=0x110000 @Sector 41
                        DwnLd_sSUM += (d16U>>8);
                        DwnLd_sSUM += (d16U&0xff);
                        DwnLd_sCNT+=2;
                        }                      
                    FRAM_RD(MAIN_ADDR_COUNT,(uint8*)&CntMain, 4);
                    FRAM_RD(MAIN_ADDR_CSUM, (uint8*)&SumMain, 2);
                    sprintf(Buf1500," MAIN    BytCnt=%d  ckSum=%4X\r\n", CntMain, SumMain);
                    if(err)
                        sprintf(tBuf," BACKup  BytCnt=%d  ckSum=%4X  Errors=%d\r\n", DwnLd_sCNT, DwnLd_sSUM, err);
                    else
                        sprintf(tBuf," BACKup  BytCnt=%d  ckSum=%4X\r\n\r\r", DwnLd_sCNT, DwnLd_sSUM);
                    strcat(Buf1500, tBuf);
                    putBuf(prt, Buf1500,0);
                    break;
                    }                
                else if (!strcmp(tok, "FD"))             //FPGA Load, direct from USB
                   {
                    sprintf(tBuf,"FPGA Direct Load from USB, Under Construction\r\n");
                    putBuf(prt, tBuf,0);
                    //loadSpartan6_FPGA(param1);
                    break;
                    }
                else if (!strcmp(tok, "FERASE"))    //FLASH erase all
                   {
                    sprintf(tBuf,"TOTAL Parallel Flash Erase, Enter 1 to erase, else exit\r\n");
                    putBuf(prt, tBuf,0);
                    param1=arg_dec(&paramPtr,0);    //get okay todo
                    if(param1!= 1)
                     break;
                    eraseFLASH();                
                    break;
                    }
                else if (!strcmp(tok, "FES"))        //FLASH erase sectors
                   {
                    sprintf(tBuf,"FLASH Erase Sector, Enter 1 to erase, else exit\r\n");
                    putBuf(prt, tBuf,0);
                    param1=arg_dec(&paramPtr,0);    //get 1st param U39B
                    if(param1!= 1)
                     break;
                    param1=arg_dec(&paramPtr,SECTORES);    //get 1st param U39B
                    if(param1> 141) param1=141;
                    sprintf(tBuf,"FLASH Erase Sectors %d\r\n", param1);
                    putBuf(prt, tBuf,0);
                    eraseFLASH_Sector(param1, 0, prt);
                    break;
                    }
                else if ((!strcmp(tok, "FL1")) || (!strcmp(tok, "FL")))  //FLASH Load of FPGA File via USB
                   {
                    //disable watchdog timer, file notifications.c
                    iFlag &= ~WHATCHDOGENA;          
                    if(prt!=tty)
                        {
                        putBuf(prt,"Use USB port only\r\n",0);              //tty usb okay
                        break;
                        }                    
                    sprintf(tBuf,"S29JL064J: Flash Loader\r\n");
                    putBuf(prt, tBuf,0);
                    PROGx_LO                    
                    uDelay(5);                      //min 300nS
                    PROGx_HI                    
                    uDelay(100);                    //fpga reset takes about 800uS
                    
                    flashStatus(prt);               //fills Buf1500 with ascii msgs
                    putBuf(prt, tBuf,0);
                    uDelay(100);                   
                    eraseFLASH_Sector(SECTORES, 0, prt);
                    sprintf(tBuf,"S29JL064J: Total sector erased %d (%d Bytes)\r\n", SECTORES, (SECTORES-7)*0x7fff);
                    putBuf(tty, tBuf,0);
                    sprintf(tBuf,"S29JL064J: Select Binary file. TeraTerm MENU File,SendFile Bin[yes]\r\n");
                    putBuf(prt, tBuf,0);
                    loadFLASH(0,prt);
                    //flashXFER(4, prt);
                    break;
                    }
                else if (!strcmp(tok, "FLSOCK"))       //FLASH Load via SOCKETs
                   {
                    //disable watchdog timer, file notifications.c
                    iFlag &= ~WHATCHDOGENA;          
                    if(prt==tty)
                        {
                        putBuf(prt,"Use Socket ports only\r\n",0);              //tty usb okay
                        break;
                        }          
                    
                    while(FLASH_RDY==0)
                        {
                        putBuf(prt,"loadFLASH: Flash not ready\r\n",0); //send to current active port
                        break;
                        }
                    
                    sprintf(tBuf,"S29JL064J: Flash Loader\r\n");
                    putBuf(prt, tBuf,0);
                    flashStatus(prt);                           //fills Buf1500 with ascii msgs
                    putBuf(prt, tBuf,0);
                    uDelay(100);                   
                    
                    sprintf(tBuf,"FLASH_PGM: FLASH Erase Now\r\n");
                    putBuf(prt, tBuf,0);
                    eraseFLASH_Sector(SECTORES, 0, prt);
                    sprintf(tBuf,"S29JL064J: Total sector erased %d (%d Bytes)\r\n", SECTORES, (SECTORES-7)*0x7fff);
                    putBuf(prt, tBuf,0);
                    uPhySums.FL_SOCK_CHKSIZE= 0;
                    uPhySums.FL_SOCK_CHKSUM=0;
                    
                    loadFLASH_NEW(prt,(char*)eRecDatBuf[g_Sock]);
                    sprintf(tBuf,"FLASH_PGM: Finished, use cmd 'FT' to reload FPGAs\r\n");
                    putBuf(prt, tBuf,0);
                    uDelay(200);                   
                    //flashXFER(4, prt);
                    break;
                    }
                else if (!strcmp(tok, "FT"))        //FLASH Load of FPGA File via USB
                   {
                    int retVal;
                    param1=arg_hex(&paramPtr,4);    //get 1st param (addr)
                    if(param1>4)
                        { parErr++;  break; }
                    if(param1<4)
                      sprintf(tBuf,"FLASH xFer to FPGA %d\r\n",param1);
                    else
                      sprintf(tBuf,"FLASH xFer to FPGAs 0,1,2,3\r\n");
                    putBuf(prt, tBuf,0);
                    
                    //load fpga from flash data
                    retVal= flashXFER(param1, prt, FLBASE);  //load all readout FPGA(s)
                    //CONFIG FAILED, try FLASH backup
                    if (retVal == -1)                   
                        {
                        uDelay(500);
                        flashXFER(param1, prt, FLBACKUP);    //load FPGA(s) with backup data
                        }                                        
                    //flashXFER(param1,prt, FLBASE);   
                    
                    //following lines of code Setup up FPGAs for DAQ 
                    //needs setup time before loading registers
                    uDelay(500);                    
                    //afe_A on each fpga
                    wrAFE_Slow(0, 0x100, 0x01);  //fpga(0-3),reg,data (afe soft reset)
                    wrAFE_Slow(1, 0x100, 0x01);  //fpga(0-3),reg,data (afe soft reset)
                    wrAFE_Slow(2, 0x100, 0x01);  //fpga(0-3),reg,data (afe soft reset)
                    wrAFE_Slow(3, 0x100, 0x01);  //fpga(0-3),reg,data (afe soft reset)
                    //afe_B on each fpga
                    wrAFE_Slow(0, 0x200, 0x01);  //fpga(0-3),reg,data (afe soft reset)
                    wrAFE_Slow(1, 0x200, 0x01);  //fpga(0-3),reg,data (afe soft reset)
                    wrAFE_Slow(2, 0x200, 0x01);  //fpga(0-3),reg,data (afe soft reset)
                    wrAFE_Slow(3, 0x200, 0x01);  //fpga(0-3),reg,data (afe soft reset)
                    
                    //General reset to AFE fifo, counter, sequencer
                    CSR0= (0x20 | (CSR0));
                    CSR1= (0x20 | (CSR1));
                    CSR2= (0x20 | (CSR2));
                    CSR3= (0x20 | (CSR3));
                    
                    //get uC REG data from FRAM, {SerNumb, MuxCh, Gain, Trig, LnkDir, nErr, xPhy, wDogTimOut}
                    //load FRAM before dataRecall()
                    FRAM_RD(fPAGE_0400, (uint8*)&uC_FRAM, FRAM_400wsz); //repeat load, already loaded by dataRecall() tek may2019 fix
                      
                    //recall FPGA Setup Data from FRAM reloaded structure
                    dataRecall(1, fpgaBase);       //fpga0
                    dataRecall(1, fpgaBase1);      //fpga1
                    dataRecall(1, fpgaBase2);      //fpga2
                    dataRecall(1, fpgaBase3);      //fpga3
                    break;
                    }
                else if (!strcmp(tok, "FS"))        //FLASH Status
                   {
                    //int chip=1;
                    u_16Bit D16;
                    u_32Bit D32;
                    FRAM_RD(MAIN_ADDR_COUNT,(uint8*)&D32, 4);
                    FRAM_RD(MAIN_ADDR_CSUM, (uint8*)&D16, 2);
                    flashStatus(prt);                  //fills Buf1500 with ascii msgs
                    sprintf(tBuf,"Flash FPGA ByteCnt = %d\r\n",D32);
                    strcat (Buf1500,tBuf);
                    sprintf(tBuf, "Flash FPGA SumCheck= %X\r\n",D16);
                    strcat (Buf1500,tBuf);
                    putBuf(prt, Buf1500,0);
                    break;
                    }
                else if (!strcmp(tok, "FI"))        //fpga reset using init poin
                   {
                    sprintf(tBuf,"fpga reset to all 4 chips using PROGRAM_B\r\n");
                    putBuf(prt, tBuf,0);
                    //reset fpga(s)
                    PROGx_LO                        //min 300nS
                    uDelay(5);
                    PROGx_HI                    
                    uDelay(1000);                   //fpga reset takes about 800uS
                    break;
                    }                
                else if (!strcmp(tok, "FZ"))        //testing flash non 0xffff check
                   {
                    param1=arg_hex(&paramPtr,0x800); //get 1st param (range to check)
                    sPTR saddr= pFLASHbase;
                    int d16,err=0,i;
                    sprintf(tBuf,"Check/Display 'Un-Erased' FLASH data (FlashBase=%X)\r\n",pFLASHbase);
                    putBuf(prt, tBuf,0);
                    for (i=0; i<param1+1; i++)      
                        {
                        d16= *saddr;
                        if(d16!=0xffff)
                            {
                            sprintf(tBuf,"Addr= %X  Data==%4X\r\n", saddr, d16);
                            putBuf(prt, tBuf,0);
                            if (++err >= 20) break;
                            }
                        saddr++;
                        }
                    sprintf(tBuf,"Total checked words= 0x%X (show 20 lines max)\r\n",i);
                    putBuf(prt, tBuf,0);
                    break;
                    }
                 else if  (!strcmp(tok, "FRD"))                 //FRAM READ
                    {
                    u_16Bit D16;
                    if ( (param1=arg_hex(&paramPtr, 0)) ==-1) //get 1st param
                        { parErr++;  break; }
                    FRAM_RD(param1, (uint8*)&D16, 2);
                    sprintf(Buf1500,"%X\r\n",D16);
                    putBuf(prt, Buf1500,0);                     //send to current active port
                    break;
                    }
                 else if (!strcmp(tok, "FWR"))                  //FRAM UPDATE
                    {
                    if ( (param1=arg_hex(&paramPtr,-1)) ==-1)   //get 1st param, addr
                        { parErr++;  break; }
                    if ( (param2=arg_hex(&paramPtr,-1)) ==-1)   //get 2nd param, data
                        { parErr++;  break; }
                    FRAM_WR_ENABLE();                           //WREN op-code must be issued prior to any Wr_Op
                    FRAM_WR(param1, (uint8*)&param2, 2);        //2-8bit writes
                    break;
                    }
                else if (!strcmp(tok,"FDUMP"))  //'FDUMP' read flash status register
                    {
                    uint16_t Adr;
                    uint16_t D16BufIn[10];
                    int lp=0;
                    Adr=arg_hex(&paramPtr,0);              //get 1st param (addr)
                    sprintf(Buf1500,"Read fRam block (16bit big endian per line)\r\n");
                    putBuf(prt, Buf1500,0);                     //send to current active port
                    while (lp++< 16)
                        {
                        FRAM_RD(Adr, (uint8*)D16BufIn, 8);      //FRAM size (8K x 8 ) display as 8 words per line
                        sprintf(Buf1500,"FRAM=%04X  %4x %4x %4x %4x  ",Adr&0x1fff, D16BufIn[0],D16BufIn[1],D16BufIn[2],D16BufIn[3]);
                        Adr += 8;       // 4 (16 bit reads)
                        FRAM_RD(Adr, (uint8*)D16BufIn, 8);
                        sprintf(tBuf,"%4x %4x %4x %4x\r\n\r", D16BufIn[0],D16BufIn[1],D16BufIn[2],D16BufIn[3]);//extra \r is for lvds return data in case odd data count
                        Adr += 8;       // 4 (16 bit reads)
                        strcat(Buf1500, tBuf);
                        putBuf(prt, Buf1500,0);             //send to current active port
                        }
                    break;
                    }
                else if  (!strcmp(tok, "FRERASE"))          //erase FRAM FM25CL64B, organized as 8K×8
                    {                                       
                    uint16_t Adr=0;
                    u_8Bit D8Buf=0;
                    param1= arg_hex(&paramPtr,0);           //get 1st param begin address
                    param2= arg_hex(&paramPtr,0);           //get 2nd param end addr
                    if(param2==0)                           //no cnt, exit
                      break;
                    if(param2>0xFFF)                        //limit 4Kx16
                      break;
                    if (param2<param1)
                      break;
                    Adr= param1; 
                    FRAM_WR_ENABLE();                       //WREN op-code must be issued prior to any Wr_Op
                    dataconfig_FRAM.CS_HOLD = TRUE;         //Hold 'cs' low
                    sendFRAM(&Op[fWRITE]);                  //send fram OP CODE
                    sendFRAM((uint8_t*)&Adr+1);             //send fram ADDR HI
                    sendFRAM((uint8_t*)&Adr);               //send fram ADDR HI
                    //erase fram memory
                    param2 -= param1;                       //length to erase as bytes
                    while (param2--)
                        sendFRAM(&D8Buf);
                    //final byte, when done sets chip sel back high
                    dataconfig_FRAM.CS_HOLD = FALSE;        //Normal 'cs' high
                    sendFRAM(&D8Buf);
                    break;
                    }
                else if  (!strcmp(tok, "FRS"))              //FRAM status register read
                    {
                    int i;
                    FRAM_WR_STATUS(0x0a);                   //send '8 bit' data F-RAM
                    i= FRAM_RD_STATUS();
                    sprintf(Buf1500,   "fRam Status=%02X\r\n", i );
                    putBuf(prt, Buf1500,0);             //send to current active port
                    break;
                    }
                else if  (!strcmp(tok, "FMS"))          //FM LVDS port testing
                    {
                    int i, pattern;
                    param1=arg_hex(&paramPtr,0);        //get 1st param cnt
                    pattern=arg_hex(&paramPtr,0);       //get 2nd param data
                    //send data on LVDS PORT
                    for(i=0;i<param1;i++)
                        FMDataSnd= pattern++;             
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'G':
                if (!strcmp(tok, "GAIN"))
                   {
                    int gain;
                    gain=arg_dec(&paramPtr,-1);      //get 1st param (def gain ==1)
                    if (gain==-1)
                        {
                        gain= pga280read(0)>>3;     //lower 3 bits are mux status
                        gain = 1<<((gain)-3);       //gain 1,2,4,8,16...
                        sprintf(tBuf,"gain=%d\r\n",gain);
                        putBuf(prt, tBuf,0);
                        break;
                        }
                    param2=arg_dec(&paramPtr,0);     //get 2nd param (reset==RST280)
                    //for a gain of 1 we write a (3)
                    if      (gain==1) {gain=3;}
                    else if (gain==2) {gain=4;}
                    else if (gain==4) {gain=5;}
                    else if (gain==8) {gain=6;}
                    else if (gain==16) {gain=7;}
                    else if (gain==32) {gain=8;}
                    else if (gain==64) {gain=9;}
                    else if (gain==128) {gain=10;}
                    else {{ parErr++;  break; }}      //default
                    //sprintf(tBuf,"Gain=%d ,Valid Gains=1,2,4,8,16,32,64,128\r\n",hld);
                    //sprintf(tBuf,"Gain=%d\r\n",hld);
                    //putBuf(prt, tBuf,0);
                    uC_FRAM.pga280Gain= gain;   //flash storage var
                    pga280(gain, param2);
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'H':
                if ((!strcmp(tok, "H1")) || (!strcmp(tok, "HE")))
                    {
                    putBuf(prt, (char *)HelpMenu, sizeof(HelpMenu)-1);  
                    break;
                    }
                else if (!strcmp(tok, "HDR"))
                    {
                    *Buf1500=0;             //null buffer to be filled
                    param1=arg_dec(&paramPtr,0);             
                    headerStatus(prt, param1);  //param1 to clear ovrRun counter
                    putBuf(prt, Buf1500,0);
                    break;
                    }
                else if (!strcmp(tok, "H2"))
                    {
                    putBuf(prt, (char *)HelpMenu2, sizeof(HelpMenu2)-1);      
                    break;
                    }
                else if (!strcmp(tok, "HF"))
                    {
                    putBuf(prt, (char *)HelpMenuFLASH, sizeof(HelpMenuFLASH)-1);        
                    break;
                    }
                else if (!strcmp(tok, "HT"))
                    {
                    putBuf(prt, (char *)HelpMenu3, sizeof(HelpMenu3)-1);        
                    break;                    
                    }
                else if (!strcmp(tok, "HA"))
                    {
                    putBuf(prt, (char *)HelpMenuAdrMap, sizeof(HelpMenuAdrMap)-1);        
                    break;                    
                    }
                else if (!strcmp(tok, "HS"))
                    {
                    putBuf(prt, (char *)HelpMenuSTAB, sizeof(HelpMenuSTAB)-1);        
                    break;                    
                    }                    
                else  {parErr=0xf;   break; }
        case 'I':
                if (!strcmp(tok, "ID"))
                   {
                    u_16Bit D16;
                    u_32Bit D32;
                    struct time ti;
                    char  txtBuf40[40];                    
                    param1=arg_hex(&paramPtr,0); 
                    
                    sprintf(Buf1500,"Module Type    : Mu2e CRV FEB\r\n");
                    sprintf(tBuf,   "Local IP Addr  : %u.%u.%u.%u\r\n", *(netInfo.ipAddr),
                        *(netInfo.ipAddr+1),*(netInfo.ipAddr+2),*(netInfo.ipAddr+3));
                    strcat(Buf1500, tBuf);
                                  
                    strncpy(txtBuf40,__VERSION__,31);  //shorten IAR response
                	sprintf(tBuf,"Compiler Ver   : %s\r\n", txtBuf40 );
                    strcat(Buf1500, tBuf);
                	//sprintf(tBuf,"Compiler Ver   : %s\r\n", __VERSION__ );
                    //strcat(Buf1500, tBuf);
                    sprintf(tBuf,"uC Compile Date: %s\r\n", __DATE__);
                    strcat(Buf1500, tBuf);
                    sprintf(tBuf,"uC Firmware Ver: %d.%02d\r\n", divV.quot, divV.rem );
                    strcat(Buf1500, tBuf);
                    
                    FRAM_RD(MAIN_ADDR_COUNT,(uint8*)&D32, 4);    //read 4 bytes
                    FRAM_RD(MAIN_ADDR_CSUM, (uint8*)&D16, 2);
                    sprintf(tBuf   ,"FPGA FL ByteCnt: %d\r\n",D32);
                    strcat (Buf1500,tBuf);
                    sprintf(tBuf,   "FPGA FL SumChk : %04X\r\n",D16);
                    strcat (Buf1500,tBuf);
                    sprintf(tBuf,   "---------------:\r\n");
                    strcat (Buf1500,tBuf);

                    //Add Uptime counter
                  //ti.totalSec= (86400 + (23*3600) + (59*60) + 59); //test numbers
                    ti.totalSec= (sUPTIMEHI<<16) + sUPTIMELO;
                    ti.sec = (ti.totalSec  %60);
                    ti.min = ((ti.totalSec /60) %60);
                    ti.hr  = ((ti.totalSec /3600) %24);
                    ti.day = ti.totalSec   /86400;
                    sprintf(tBuf,   "DAY DD HH:MM:SS: %02d %02d:%02d:%02d\r\n", ti.day,ti.hr,ti.min,ti.sec);
                    strcat (Buf1500,tBuf);
                                        
                    sprintf(tBuf,   "ePHY_Rec LockUp: %d\r\n", PaketStats.eRecHung);
                    strcat (Buf1500,tBuf);
                    
                    //nError ECC error counter
                    sprintf(tBuf,   "uC ECC ReBoots : %d\r\n",uC_FRAM.nErrCnt);
                    strcat (Buf1500,tBuf);
                    //FPGA ECC error counter
                    sprintf(tBuf,   "FPGA ECC Errors: %d\r\n",uC_FRAM.FpgaEccCnt);
                    strcat (Buf1500,tBuf);
                    
                    //FRAM_RD(SERIAL_NUMB_ADR, (uint16_t*)&D16,2);    //read 2 bytes
                    sprintf(tBuf,"Serial Number  : %d\r\n\n",uC_FRAM.serNumb);
                    strcat (Buf1500,tBuf);
                    //poe port number from FEB Controller, also used as link id number
                    //sprintf(tBuf,   "POEPortNumb 1of24 : %d\r\n\n", HappyBus.myLinkNumber); //extra char if odd byte cnt on 16bit LVDS xfer
                    //strcat (Buf1500,tBuf);
                    
                    putBuf(prt, Buf1500,0);
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'L':
            /*  if (!strcmp(tok, "LCHK"))               //lvds link reinit check, used of Link Init
                    {    
                    //NOTE PROMPTS GETS SENT BACK ON LVDS ON EXIT HERE   
                    //uses commands header board number for assignment
                     
                    //new link init code, just send board number as 16bits
                    //HappyBus.BrdNumb loaded before we get here 'phy_Pac_check()'
                    //Upper byte now holds mu2e controller number
                      
                    //broadcast ctrl# and brd#
                    BCST_GEO_314= HappyBus.BrdNumb;     //has ctrl# and brd#
                    
                   // HappyBus.myLnkCntrlOK= HappyBus.myLnkCntrlTmp;//store cntrl#
                    
                    //broadcast ctrl# and brd#
                   // BCST_GEO_314= ((HappyBus.myLnkCntrlOK<<8)+ HappyBus.BrdNumb);     
                    
                    HappyBus.BrdNumb &= 0xff;           //store brd# only       
                    if (HappyBus.SndWrds==0)
                        FMDataSnd= HappyBus.BrdNumb; 
                    //HappyBus.myLinkNumber= HappyBus.BrdNumb;

                    //sprintf(tBuf,"%02X", HappyBus.BrdNumb);
                    //tBuf[2]=0;      //append extra null needed at controllers word reads
                    //tBuf[3]=0;      //append extra null needed at controllers word reads
                    //putBuf(prt, tBuf,4);
                    break;
                    }
          */
                if (!strcmp(tok, "LSTAB"))     //DATA POOL REQUEST LVDS 'eCMD72_STAB'
                    {                               //takes 3uS, update and set ptrs for bckgnd 'lvds port xmit'
                    //hHI_TP46;                    
                    //valid command range at the FEB rec coding is 0x70-0x7F for data echo via putBuf()   
                    //statusBlock mostly filled in background by 'OneWireTrigRead()' 
                    if ((HappyBus.CmdType<eCMD_BEG) || (HappyBus.CmdType>eCMD_END) ) //err trap
                        {
                        HappyBus.PHYrecvErr++;
                        break;
                        }
                    //wait for any data in buffer to finish sending
                    for(int i=0;i<10000; i++)               //dont wait forever if fpga problem
                        {
                        if(HappyBus.SndWrds==0)             //exit when any old data has been sent
                            break;                            
                        }
                    _disable_interrupt_();
                    uint16* tb= (snvPTR)PoolData.uC;        //use array[0] for board block data, ..array[1,2,3,4]hold fpga data
                        
                    *tb++= uC_FRAM.serNumb;                 //serial number
                    *tb++= s0SP_COUNT;                      //spill cycle counter
                                        
                    //get adc readings 800 nSec
                    float ftemp;
                    ftemp=(readTemperature()*100);          //floating value of pulse readings                       
                    *tb++= (int)(ftemp);                    //includes 100th-of-a-degree)                        
    
                    //takes 1.5 uSec total
                    for (int i=0; i<16; i++)
                        *tb++= adc_data[i].value;
                                            
                    //afe reads to slow to do here, already pooled in background
                    *tb++= sTrigCntrl;                        
                    *tb++= sPipeLinDly;                     //pipeline delay (0==256) 12.56nS per cnt
                    *tb= sSampleLen;                        //read 16bit AFE reg sample length       
                        
                    genFlag |= NO_ECHO;
                    //PoolData, replaces 'statusBlock[5][sBLKSIZ38+2]' for data pooling via 'LSTAB'
                    //statusBlock is filled in bacground by 'OneWireTrigRead()'
                    HappyBus.SndWrds= sizeof(PoolData)/2;   //size in words            
                    HappyBus.Src= (snvPTR)PoolData.uC;      //reset to beg for backgrnd xmits 

                    //data sent on lvds via interrupts trigger by LVDS_Intr_Time
                    _enable_interrupt_();
                    LVDS_Intr_Time(1);                      //Triggers interrupt xmit seq 174 word block
                    //hLO_TP46;
                    break;
                    }   
                else if (!strcmp(tok, "LDSCAN"))            //Scan file stored in SDRAM testing
                   {                                          
                    //Program FLASH (takes ~20 Secs)
                    int d16, cnt= u_SRAM.DwnLd_sCNT;

                    //Load (*.bin file ) to Flash using FPGA1_sdRam1 data, assume cmd 'LDFILE' already got file
                    u_SRAM.DwnLd_sCNT=0;
                    u_SRAM.DwnLd_sSUM=0;             
                    SET_SDADDR_RDx(0,0,0);              //fpgaOffset, addrHI, addrLO
                    
                    //now do local check to verify data still good
                    //compute checkSum, LDFLASH_FromSDram() will use results            
                    for (int i=0; i<cnt/2; i++)
                        {
                        d16= REG16(&SDR_RD16);          //data port, using fpga(2of4) sdRam
                        u_SRAM.DwnLd_sSUM += ((d16>>8)&0xff);
                        u_SRAM.DwnLd_sSUM += (d16&0xff);
                        u_SRAM.DwnLd_sCNT+=2;
                        }  
                    sprintf(Buf1500 ,"SD_RAM ReScanning : ByteCount=%d   CheckSum=%04X\r\n",u_SRAM.DwnLd_sCNT, u_SRAM.DwnLd_sSUM&0xffff);
                    sprintf(tBuf, "SD_RAM Verify Err : Data Does not Match or All Zeros\r\n");
                    strcat(Buf1500, tBuf);
                    putBuf(0, Buf1500, 0);   
                    break;
                   }          
                else if (!strcmp(tok, "LDFLASH"))       //Program Flash with data file stored in SDRAM
                   {                                    //THIS cmd MAY HAVE COME FROM CONTROLLERs cmd 'LDPGMFEB'
                    int d16, cnt= u_SRAM.DwnLd_sCNT;
                    u_SRAM.DwnLd_sCNT=0;
                    u_SRAM.DwnLd_sSUM=0;             
                    SET_SDADDR_RDx(0,0,0);              //fpgaOffset, addrHI, addrLO
                    
                    //now do local check to verify data still good
                    //compute checkSum, LDFLASH_FromSDram() will use results            
                    for (int i=0; i<cnt/2; i++)
                        {
                        d16= REG16(&SDR_RD16);          //data port, using fpga(2of4) sdRam
                        u_SRAM.DwnLd_sSUM += ((d16>>8)&0xff);
                        u_SRAM.DwnLd_sSUM += (d16&0xff);
                        u_SRAM.DwnLd_sCNT+=2;
                        }  

                    //************************************************************************                    
                    //****  If file sizes match, Programming FLASH takes about 20 sec   ******   
                    //***** If valid Load, reboot fpga section, and do a 'DREC'         ******
                    //************************************************************************                    
                    d16= PgmFlash_FromSDramData(prt);   //using fpga1 of 1of4 sdRam memory 0x07
                    
                    if(d16)                             //NonZero= error
                        {
                        sprintf(Buf1500,"FEB  : LDFLASH ERROR\r\n\r");
                        putBuf(prt, Buf1500,0);
                        }
                    break;
                   }          
                else if (!strcmp(tok, "LDFILE"))        //Load data file to SDRAM
                   {
                    char* eBufB= (char*)eRecDatBuf[g_Sock];
                    param1= arg_dec(&paramPtr,1);  
                    LDF(prt, param1, eBufB);            //port, fpga1of4, buf
                    break; 
                   }
                else if (!strcmp(tok, "LDSTAT"))        //Load FEB status
                   {
                    //Valid download Check,  
                    //u_SRAM   regs filled by 'LDRAM() downloaded data
                    //uPhySums regs filled by 'LDRAM() passed from controller to verify Size+ChkSum
                    u_SRAM.FL_XFER= 0;
                    if((u_SRAM.DwnLd_sCNT==uPhySums.DwnLd_sCNT) && (u_SRAM.DwnLd_sSUM==uPhySums.DwnLd_sSUM)&& (uPhySums.DwnLd_sCNT !=0)) //match original file specs
                        u_SRAM.FL_XFER= 2;
                    if(u_SRAM.FL_XFER==2)
                        sprintf(Buf1500,"\r\nFEB_TRANSFER = GOOD\r\n");
                    else 
                        sprintf(Buf1500,"\r\nFEB_TRANSFER = UNKNOWN\r\n");
                      
                    if(u_SRAM.FL_ERASE==2)
                        sprintf(tBuf,"FEB_ERASE    = GOOD\r\n");
                    else
                        sprintf(tBuf,"FEB_ERASE    = UNKNOWN\r\n");
                    strcat(Buf1500, tBuf);

                    if(u_SRAM.FL_PGM==2)
                        sprintf(tBuf,"FEB_PROGRAM  = GOOD\r\n");
                    else
                        sprintf(tBuf,"FEB_PROGRAM  = UNKNOWN\r\n");
                    strcat(Buf1500, tBuf);

                    if(u_SRAM.FL_BOOT==2)
                        sprintf(tBuf,"FEB_FPGA_INIT= GOOD\r\n\r");
                    else
                        sprintf(tBuf,"FEB_FPGA_INIT= UNKNOWN\r\n\r");
                    strcat(Buf1500, tBuf);
                    
                    if(u_SRAM.FL_SDram==2)
                        sprintf(tBuf,"FEB_FAKE_LAOD= GOOD\r\n\r");
                    else
                        sprintf(tBuf,"FEB_FAKE_LOAD= UNKNOWN\r\n\r");
                    strcat(Buf1500, tBuf);
                    
                    putBuf(prt, Buf1500,0);
                    break; 
                   }                
                else if (!strcmp(tok, "LINK"))              //LINK, Sets lvds link direction
                    {    
                    param1=arg_dec(&paramPtr,-1);           //get 1st param
                    if (param1==-1)
                        {
                        if (uC_FRAM.LinkDir==1)
                            sprintf(tBuf,"%d= XMIT\r\n", uC_FRAM.LinkDir);
                        else
                            sprintf(tBuf,"%d= REC\r\n", uC_FRAM.LinkDir);
                        putBuf(prt, tBuf,0);
                        break;
                        }
                    if  (param1==0)   {LINK_DIR_LO uC_FRAM.LinkDir= param1;} //hetREG1->DCLR=BIT19
                    else if(param1==1){LINK_DIR_HI uC_FRAM.LinkDir= param1;} // hetREG1->DSET=BIT19
                    else parErr++;
                    break;
                    }
                else if (!strcmp(tok, "LDRAM"))     //Load data file to SDRAM1 (1of4) from FEB Controller 
                   {                                //get here from Controller Cmds 'LDPGMFEB' or 'SEND_2_FEB'
                    int retVal;
                    //HappyBus.CmdSizW holds recd word cnt, FPGA1(1of4) sdRAM holds data file
                    retVal= ldRam_DataFile(prt, HappyBus.CmdSizW, (u_16Bit*)eRecDatPtr[g_Sock], (u_16Bit*)&uPhySums);
                    
                    //retVal word is Pass=1/Fail=0 status sent back Controller on lvds link
                    if(prt==ePhyIO)                 //ePhyIO port send 1char pass/fail status
                        FMDataSnd= retVal;          //send data on LVDS PORT
                    break;
                   }
                else if (!strcmp(tok, "LDFP"))      //Load Flash with SDram Fake Data
                    {  
                    int retVal;
                    retVal= PGM_SDramFakeData(prt);
                    if(prt==ePhyIO)                 //ePhyIO port send 1char pass/fail status
                        FMDataSnd= retVal;          //send data on LVDS PORT
                    break;
                    }
                else if (!strcmp(tok, "LDFE"))      //Load Flash Init, 1st erase flash fake data sectors
                    {  
                    int retVal;
                    FLASH_WP_LO                     //FLASH_WP (LOW) (V6) HET1_5
                    FlashRst_LO                     //reset to ready 35uS max
                    uDelay(50);
                    //FLASH RESET (keep high) (K18)HET1_0, 500nS low time
                    FlashRst_HI                     //reset to ready 35uS max
                    uDelay(50);                    
                    //clear old flash dwnload status
                    u_SRAM.FL_ERASE=0;
                    u_SRAM.FL_PGM=0;
                    u_SRAM.FL_XFER=0;
                    u_SRAM.FL_BOOT=0;                    
                    u_SRAM.FL_SDram=0;                                          
                   //Flash Sector(s) Erase
                    u_SRAM.FL_ERASE=1;              //erase cycle busy
                    //Erase 62 Sectors of 64KBytes each, 4,063,170 Bytes (3DFFC2)
                    retVal= eraseFLASH_Sector(SECT_FAKE_Sz, S29JL064J_SECTOR71, prt);    
                    u_SRAM.FL_ERASE=2;              //erase cycle done
                    if(prt==ePhyIO)                 //ePhyIO port send 1char pass/fail status
                        FMDataSnd= retVal;          //send data on LVDS PORT
                    break;
                    }
                else if (!strcmp(tok, "LDFC"))      //Copy Fake Flash data to SDram
                    {  
                    int retVal;
                    param1=arg_dec(&paramPtr,0);    //get 1st param, display mode 
                    retVal= ld_FAKE2sdRAM(prt, param1);
                    if(retVal==1)
                        u_SRAM.FL_SDram=2;          //2==good
                    else
                        u_SRAM.FL_SDram=1;          //1==unknown
                    if(prt==ePhyIO)                 //ePhyIO port send 1char pass/fail status
                        FMDataSnd= retVal;          //send data on LVDS PORT
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'M':
                tok[2]= 0;                              //limit string compare to 2 chars
                if ((!strcmp(tok, "MU"))||(!strcmp(tok, "MUX"))) //pga280 input 'MUX' channel select
                    {    
                    param1=arg_dec(&paramPtr,4);        //get 1st param
                    if(param1<4)
                    {
                    uC_FRAM.pgaMuxCh=param1;                       //global var
                    //16 HDMI Trim Volt Chs per fpga, select 1of4 MuxChip(s) ADG1609
                    if (uC_FRAM.pgaMuxCh==0)      {MUXA3_LO; MUXA2_LO;}
                    else if (uC_FRAM.pgaMuxCh==1) {MUXA3_LO; MUXA2_HI;}
                    else if (uC_FRAM.pgaMuxCh==2) {MUXA3_HI; MUXA2_LO;}
                    else if (uC_FRAM.pgaMuxCh==3) {MUXA3_HI; MUXA2_HI;}
                    }
                    sprintf(tBuf,"MuxCh=%d\r\n", uC_FRAM.pgaMuxCh);
                    putBuf(prt, tBuf,0);
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'N':
                if (!strcmp(tok, "NETRST"))                 //restoring network defaults
                    {
                    param1=arg_dec(&paramPtr,0);            //get 1st param
                    if(param1==192)                         //use fermilab ip setuup 
                        memcpy(&netInfo, &defNetInfo, sizeof(netInfo));
                    else
                        memcpy(&netInfo, &defNetInfo192, sizeof(netInfo));
                    printf("Closing sockets, restarting network driver\r\n\n");
                    mDelay(800);                            //allowtime for msg to xmit
                    wizMain();                              //test code for wiznet 5300
                    mDelay(100);                            //allow wiznet pll logic to stabalize
                    break;
                    }
                else if (!strcmp(tok, "NETSAV"))            //restoring network defaults
                    {
                    netInfo.Valid= VALID;                   //flag as valid
                    FRAM_WR_ENABLE();                       //WREN op-code must be issued prior to any Wr_Op
                    FRAM_WR(fPAGE_Net500, (uint8*)&netInfo, sizeof(netInfo)); //write 2 bytes
                    FRAM_WR_DISABLE();
                    break;
                    }
                else  {parErr=0xf;   break;}                
        case 'O':
                if (!strcmp(tok, "OVC"))                    //OVC Trip, BIAS Voltage Over Current
                    {
                    param1=arg_dec(&paramPtr,0);            //get 1st param
                    //The LT4256-1 latches off until the UV pin is cycled low
                    //Not intended to control LT4256-2 as it has auto retry
                    //Added 'OVC_TRIP_LATCH' since OVC_TRIP is only valid for 80mSec
                    if(param1)                              //clears on any input               
                        {
                        canOVCRstLO;                        //reset cycle low
                        uDelay(12);                         //needs min 10us
                        canOVCRstHI;                        //enable
                        genFlag &= ~OVC_TRIP;               //clear OVC flag
                        break;
                        }
                    if (genFlag & OVC_TRIP)
                        sprintf(tBuf,"1 (TRIP)\r\n\r");
                    else
                        sprintf(tBuf,"0 (OK)\r\n\r");
                    putBuf(prt, tBuf,0);                    
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'P':
                //tok[4]= 0;
                if (!strcmp(tok, "PACS"))               //PACKET SEND, (Used for initial testing)
                   {
                    int loop=0;
                    g_lcnt=0;
                    param1=arg_dec(&paramPtr,1);        //get 1st repeat param
                    param2=arg_dec(&paramPtr,8);        //get 2nd size param
                    while (loop++ < param1) 
                        {
                        create_packet(param2);          //byte count even and greater than 2
                        paKet.EMACTxBsy=1;
                        EMACTransmit(&hdkif_data[0], &pack[0]);
                        //some delay needed else false early done??? ~75uS on '1038 byte packet'
                        uDelay(20);
                        
                        //Set flag that send is pending/active, will be clear in 'EMACTxIntISR()'
                        //EMACTxIntISR(void) must set g_EMACTxBsy=0 for this to work 
                        while(paKet.EMACTxBsy);

                        g_lcnt++;
                       }
                    break;
                    }
                else if (!strcmp(tok, "PACR"))      //PACVIEW or PACREC, (Used for initial testing)
                   {
                  //phyHdr.enable=1; //done under "CASE T1ENA"
                    int key,retVal=0, ActPak;
                    param1=arg_dec(&paramPtr,100); //items count to display
                    //if (param1 > numbPack) param1=numbPack;
                    //sprintf(tBuf,"Ethernet PHY RecIntrPerSec= %d\r\n", paKet.RecPackCntsLat);
                    //putBuf(prt, tBuf,0);
                    sprintf(tBuf,"Enter 'CR' to exit loop\r\n");
                    putBuf(prt, tBuf,0);
                    //phyHdr.msgCnt=0;
                    phyHdr.PacEna=1;
                    for(int i=0;i<param1; i++)
                        {
                        while (paKet.EMACRxRdy==0)
                            {
                            if (USB_Rec.Cnt) {retVal=1; break; }
                            retVal= SockKeyWait(100, prt, &key);  //wait time in mS, returns FALSE on Timeout
                            if(retVal==1)
                                break;
                            }
                        paKet.EMACRxRdy=0;
                        ActPak= phyHdr.PacActive;
                        sprintf(tBuf,"Packet %d_of_%d      OrigSize=%d\r\n", i, param1, phyHdr.PacBytCnt[ActPak]);
                        putBuf(prt, tBuf,0);
                        HexDump((char*)phyHdr.PacPayLd1024w[ActPak], phyHdr.PacBytCnt[ActPak], prt);  //phyHdr.msgBytes[i]);
                        //data moved from emac local packet buffer 1ofnn see 'ver_ePHY.h'
                        //packet moved to local command line buffer, point to next packet in buffer
                        phyHdr.PacActive++;
                        if (phyHdr.PacActive>= PacBufs)         //ver_io.h
                            phyHdr.PacActive=0; 
                        putBuf(prt, "\r\n",2);
                        if(retVal==1)
                          break;
                        }
                    break;
                    }
                else if (!strcmp(tok, "PACE"))              //PACENA, (Used for initial testing) 
                   {
                    param1=arg_dec(&paramPtr,0);            //test EMAC port, enable cap bufs for data display
                    if (param1==1)
                        phyHdr.PacEna=1;
                    else
                        phyHdr.PacEna=0;
                    break;
                    }
                else if (!strcmp(tok, "PS"))                //MDIO... Phy link status
                   {
                    param1=arg_dec(&paramPtr,0);            //tesm EMAC port, enable cap bufs for data display
                    if(!Dp83640LinkStatusGet(hdkif_data->mdio_base, 1, 10))
                        sprintf(tBuf,"Link is up\r\n");
                    else
                        sprintf(tBuf,"Link is down\r\n");
                    putBuf(prt, tBuf,0);
                    break;
                    }
                else if (!strcmp(tok, "PR"))                //MDIO... Read Register
                   {
                    volatile uint16 linkStatus = 0U;
                    boolean retVal = TRUE;
                    
                    /* First read the BSR of the PHY */
                    retVal= MDIOPhyRegRead(hdkif_data->mdio_base, 1, (uint32)PHY_BSR, &linkStatus);
                    if(retVal != TRUE)
                        sprintf(tBuf,"read failed\r\n");
                    else
                        sprintf(tBuf,"data= %X\r\n",linkStatus);
                    putBuf(prt, tBuf,0);
                    break;
                    }
                else if (!strcmp(tok, "PWR"))               //POWER ENABLE
                   {
                    param1=arg_dec(&paramPtr,6);            
                    if(param1==0) //all off
                        {
                        CSR0= 3;                            //pwr dwn AFE Chips on this fpga, 3=off
                        CSR1= 3;                            //pwr dwn AFE Chips on this fpga
                        CSR2= 3;                            //pwr dwn AFE Chips on this fpga
                        CSR3= 3;                            //pwr dwn AFE Chips on this fpga
                        sprintf(tBuf,"Power Down All\r\n");
                        }
                    else if(param1==1) //1 on
                        {
                        CSR0 = 0x20;                        //Reset to AFE fifo, cnt, seq, power up
                        sprintf(tBuf,"Power Up GRP1\r\n");
                        }
                    else if(param1==2) //2 on
                        {
                        CSR1 = 0x20;                        //Reset to AFE fifo, cnt, seq, power up
                        sprintf(tBuf,"Power Up GRP2\r\n");
                        }
                    else if(param1==3) //3 on
                        {
                        CSR2 = 0x20;                        //Reset to AFE fifo, cnt, seq, power up
                        sprintf(tBuf,"Power Up GRP3\r\n");
                        }
                    else if(param1==4) //4 on
                        {
                        CSR3 = 0x20;                        //Reset to AFE fifo, cnt, seq, power up
                        sprintf(tBuf,"Power Up GRP4\r\n");
                        }
                    else if(param1==5) //all on
                        {
                        //General reset to AFE fifo, counter, sequencer and power up
                        CSR0 = 0x20;                        //pwr up AFE Chips on this fpga, 0=run
                        CSR1 = 0x20;                        //pwr up AFE Chips on this fpga
                        CSR2 = 0x20;                        //pwr up AFE Chips on this fpga
                        CSR3 = 0x20;                        //pwr up AFE Chips on this fpga
                        sprintf(tBuf,"Power Up All\r\n");
                        }
                    else  {parErr++;   break; }
                    putBuf(prt, tBuf,0);
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'Q':
                    {parErr=0xf;   break; }
        case 'R':
                if (!strcmp(tok, "RDX"))                    //Binary data out PHY port,(Used for initial testing)
                    {
                    //ePHY PORT DATA XMIT
                    //BUILD DATA HEADER 
                    //XMIT HEADER AND INITIAL DATA PACKET
                    //BACKGROUND WILL XMIT ADDITIONAL PACKETS AS NEEDED
                    uint32_t SpCntWrds; 
               //Test point toggle                    
               hLO_TP45;
               hHI_TP45;
               hLO_TP45;
                    param1= arg_hex(&paramPtr,-1);          //get 1st param1==word count
                    param2= arg_hex(&paramPtr,0);           //get 2nd param2==reset mode
                    //if (param1> (MAX_WORD_PER_FPGA*4));   //limit word count
                    //    param1= (MAX_WORD_PER_FPGA*4);
                                    
                    //0 will stop any active request
                    if (param1==0)
                        {
                        BinStRDX.gSndWrdCnt=0;              //stops any active background xfers
                        FPGA_Trig_Stat.OverMaxCounter=0;    //clr counter
                        break;
                        }
                    //already busy, exit
                    else if (BinStRDX.gSndWrdCnt)           //nonzero, rdb already active
                        break;

                    //ddr port read pointers not reset if param2 nonZero
                    if(param2==0)
                        { LNG_SDR_RD0=0;  LNG_SDR_RD1=0; LNG_SDR_RD2=0;  LNG_SDR_RD3=0; }

                    //Store trigger cnts of all fpga(s) for displayed diagnostics
                    //Also checks for overflows 
                    FPGA_Trig_Cnts_Update();        

                    //start with fpga0 (1of4)
                    FPGA_Trig_Stat.Fpga1of4= 0;    
                    
                    //Now on 1st FPGA, use Words Cnt for PHY xFERs , later FPGAs handled in sendRDX()
                    FPGA_Trig_Stat.WrdToSnd= FPGA_Trig_Stat.WrdCnt[FPGA_Trig_Stat.Fpga1of4]; //use word cnt ON ePHY
                    if (FPGA_Trig_Stat.Mask[FPGA_Trig_Stat.Fpga1of4] == 0)
                        FPGA_Trig_Stat.WrdToSnd=0;

                    SpCntWrds= FPGA_Trig_Stat.TotWrdCnt;            //FPGAs Reg 'Checked for OverMax'
                                    
                    //build Main 'Controller Header'
                    Cntrl_tdcHDR.tSpilTrgCntL= s0TRGCNT_HI;         //reverse now go big endian
                    Cntrl_tdcHDR.tSpilTrgCntH= s0TRGCNT_LO;         //reverse now go big endian
                    Cntrl_tdcHDR.tSpilCyc= s0SP_COUNT;              //16bit read
                    Cntrl_tdcHDR.tMask= FPGA_Trig_Stat.Mask[4];     //((sIN_MASK3<<12) + (sIN_MASK2<<8) + (sIN_MASK1<<4) + sIN_MASK0);
                    Cntrl_tdcHDR.tID= uC_FRAM.serNumb;
                    Cntrl_tdcHDR.tStatus= sSpillStatus;
                                    
                    //Note: full data count should be requested, else hdr repeats for each 'RDB'
                    //swap high/low 32 bit word count (endian little to big)
                    SpCntWrds += cCntlHdrSizWrd;                    //add cCntlHdrSizWrd for header only
                    uint16_t *hdr= (uint16_t*)&Cntrl_tdcHDR.tSpilWrdCntL;
                    *(hdr+1)= SpCntWrds;
                    *hdr= SpCntWrds>>16;
                    SpCntWrds -= cCntlHdrSizWrd;                    //remove cCntlHdrSizWrd to word cnt
                    //store last header at time of readout for display routine
                    memcpy(&tHDR_STORE, &Cntrl_tdcHDR, sizeof(tHDR_STORE)); //dest, src, bytes cnt

                    //1st send header
                    //send(prt,(uint8*)&Cntrl_tdcHDR,cCntlHdrSizByt);   //send AS Byte count
                    //creatpacket for PHY
                    fill_daq_packet(cCntlHdrSizByt, (u_8Bit*)&Cntrl_tdcHDR); //ByteCount, data buffer ptr
                    paKet.EMACTxBsy=1;      //'EMACTxIntISR()' g_EMACTxBsy=0 for done status

                    //send data to MAC
                    EMACTransmit(&hdkif_data[0], &PacLnkLst[0]);    //one or more packet chained
                    //some delay needed else false early done??? ~75uS on '1038 byte packet'
                    uDelay(25);
                    
                    //wait for eTX to finish
                    //set flag that send is pending/active, will be clear in 'EMACTxIntISR()'
                    while(paKet.EMACTxBsy); //tek
                    //todo add time out here          
                    
                    //check data count, no data then only header was sent
                    if (( SpCntWrds== 0) && ( param1== -1))         //no data or no req size
                        {
                        genFlag |= NO_ECHO;                         //no ASCII reply 
                        BinStRDX.gSndWrdCnt=0;
                        break;
                        }
                    
                    //after 1st packet, background loop will send remaining data using non-zero 'gSndWrdCnt'
                    if (param1 != -1)                               //user req size valid, use it
                        BinStRDX.gSndWrdCnt= param1;                //use user req size
                    else
                        BinStRDX.gSndWrdCnt= SpCntWrds;             //feb computed wrd cnt
                      
                    FPGA_Trig_Stat.RDBreqSiz= BinStRDX.gSndWrdCnt;  //store word cnt for diagnostics
                    BinStRDX.gSndSrc= (lPTR)&SDR_RD16SWP;           //uC DDR ram data reg
                  //mStime.gBusy_mSec=0;
                    BinStRDX.gSndPrt= prt;
                    BinStRDX.gSndMode=1;                    //0=reg mode, 1=daq mode
                    sendRDX();
                    break;
                    }
                else if (!strcmp(tok, "RDB"))               //read bin TDC data
                    {
                    sPTR saddr;
                    uint16_t free;
                    uint32_t SpCntWrds;
                    param1= arg_hex(&paramPtr,-1);          //get 1st param, word count
                    param2= arg_hex(&paramPtr,0);           //get 2nd param, reset mode
                                                            
                    //One Time New LIne Prompt Block on exit for DAQ Apps
                    genFlag |= NO_ECHO;                     //no ASCII reply     

                    //note, param1 is a signed 32 int 0x80000000 is negative number                    
                    if (param1==0)                          //0 will stop any active request
                        {BinSt.gSndBytCnt=0; break; }       //stops any active background xfers
                  
                    //ddr port read pointers not reset if param2 nonZero
                    if(param2==0)
                        { LNG_SDR_RD0=0;  LNG_SDR_RD1=0; LNG_SDR_RD2=0;  LNG_SDR_RD3=0; }
                    
                    //Store trigger cnts of all fpga(s) for SendBin() and displayed diagnostics
                    //Also checks for overflows 
                    FPGA_Trig_Cnts_Update();                    
                    SpCntWrds= FPGA_Trig_Stat.TotWrdCnt;            //32bit fpga registered count
                    
                    //start with fpga0 (1of4); do before next line code
                    FPGA_Trig_Stat.Fpga1of4= 0; 
                    
                    //Now on 1st FPGA, save Words to Byte CNT, later FPGAs handled in sendBin()
                    FPGA_Trig_Stat.BytToSnd= FPGA_Trig_Stat.WrdCnt[FPGA_Trig_Stat.Fpga1of4]<<1;
                    if (FPGA_Trig_Stat.Mask[FPGA_Trig_Stat.Fpga1of4] == 0)
                        FPGA_Trig_Stat.BytToSnd=0;
                                       
                    //build Main 'Controller Header'
                    Cntrl_tdcHDR.tSpilTrgCntL= s0TRGCNT_HI;         //reverse now go big endian
                    Cntrl_tdcHDR.tSpilTrgCntH= s0TRGCNT_LO;         //reverse now go big endian
                    Cntrl_tdcHDR.tSpilCyc= s0SP_COUNT;              //16bit read                    
                    Cntrl_tdcHDR.tMask= FPGA_Trig_Stat.Mask[4];     //updated regs done in FPGA_Trig_Cnts_Update() 
                    Cntrl_tdcHDR.tID= uC_FRAM.serNumb;
                    Cntrl_tdcHDR.tStatus= sSpillStatus;
                    
                    //Note: full data count should be requested, else hdr repeats for each 'RDB'
                    //swap high/low 32 bit word count (endian little to big)
                    SpCntWrds += cCntlHdrSizWrd;                    //add cCntlHdrSizWrd for header only
                    uint16_t *hdr= (uint16_t*)&Cntrl_tdcHDR.tSpilWrdCntL;
                    *(hdr+1)= SpCntWrds;
                    *hdr= SpCntWrds>>16;
                    SpCntWrds -= cCntlHdrSizWrd;                    //remove cCntlHdrSizWrd to word cnt
                    //store last header at time of readout for display routine
                    memcpy(&tHDR_STORE, &Cntrl_tdcHDR, sizeof(tHDR_STORE)); //dest, src, bytes cnt

                    //ethernet socket check
                    free =getSn_TX_FSR(prt);
                    if (free < 100)                                 //get freesize in (bytes)
                        uDelay(800);                                //should never happen, but allow some wait

                    //1st Send Header, let 'wiznet' do litte/big endian adjust on uC memory Header
                    setMR(getMR() & ~MR_FS);                        //enable byte swap when sending 16bit data
                    send(prt,(uint8*)&Cntrl_tdcHDR,cCntlHdrSizByt); //send AS Byte count                   
                    setMR(getMR() | MR_FS);                         //return to default
                    
                    //check data count, no data then only header was sent
                    if ( SpCntWrds == 0)                            //no data
                        {
                        BinSt.gSndBytCnt=0;
                        break;
                        }
                    
                    //after 1st packet, background loop will send remaining data using non-zero 'BinSt.gSndBytCnt'
                    if (param1 == -1)                       //user req size valid, use it
                        param1= SpCntWrds;             
                    
                    //finally, limit word count
                    if (param1> MAX_WORD_PER_FPGA*4)     
                        param1= MAX_WORD_PER_FPGA*4;

                    FPGA_Trig_Stat.RDBreqSiz= param1;       //store word cnt for diagnostics
                    BinSt.gSndBytCnt= param1<<1;            //words to byte count
                    BinSt.gSndSrc= (lPTR)&SDR_RD16SWP;      //uC DDR ram data reg                    
                    BinSt.gSndPrt= prt;
                    BinSt.gSndMode=1;                       //0=reg mode, 1=daq mode
                    sendBin(prt);
                    break;
                    }
                else if (!strcmp(tok, "RDBR"))              //read REGISTER data as bin
                    {
                    sPTR saddr;
                    uint16_t free;
                    param1= arg_hex(&paramPtr,-1);          //get 1st param, addr to read
                    param2= arg_hex(&paramPtr,-1);          //get 2nd param, word count
                    param3= arg_hex(&paramPtr,0);           //get 3nd param, reset mode
                    if (param2> 0x10000000)                 //limit word count
                        param2= 0x10000000;
                    
                    if(prt==ePhyIO)
                        {
                        putBuf(prt,"",0);      
                        break;
                        }
                    //One Time New LIne Prompt Block on exit for DAQ Apps
                    genFlag |= NO_ECHO;                     //no ASCII reply     

                    //addr out of range
                    if (param1> 0x1000)
                    { BinSt.gSndBytCnt=0;  break;}          //error
                    //0 will stop any active request
                    if (param1==0)
                        {BinSt.gSndBytCnt=0; break;}        //stops any active background xfers
                    
                    //ethernet socket check                   
                    free =getSn_TX_FSR(prt);
                    if (free < 100)                         //get freesize in (bytes), 'hdr xmit needs 9 bytes'
                        uDelay(800);                        //should never happen, but allow some wait
                    
                    //after 1st packet, background loop will send remaining data using non-zero 'gSndBytCnt'
                    BinSt.gSndBytCnt= param2<<1;            //use user req size
                    BinSt.gSndMode=0;                       //0=reg mode, 1=daq mode
                    BinSt.gSndSrcSptr= (sPTR)fpgaBase+param1; //uC DDR ram data reg
                    //mStime.gBusy_mSec=0;
                    BinSt.gSndPrt= prt;
                    sendBinNonDAQ(prt);
                    //One Time New LIne Prompt Block on exit for DAQ Apps
                    genFlag |= NO_ECHO;                     //no ASCII reply     
                    break;
                    }
                else if (!strcmp(tok, "RD"))                //16 bit EPI BUSS FPGA READ
                    {
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param
                        { parErr++;  break; }
                    //address only, no board number
                    param1 &= 0xFFFF;
                    sPTR saddr = (sPTR) fpgaBase;
                    saddr += param1;                        //adding to pointer, incr by ptr size
                    g_dat16= *saddr;

                    sprintf(Buf1500,"%04X\r\n",g_dat16);
                    putBuf(prt, Buf1500,0);                 //send to current active port
                    //One Time New LIne Prompt Block on exit for DAQ Apps
                    if(prt!=tty)
                        genFlag |= NO_ECHO;                     //no ASCII reply     
                    break;
                    }                
                //else if (!strcmp(tok, "RD0"))//'zero'       //system status registers
                //    {
                //    lPTR laddr = (lPTR) param1; 
                //    sprintf(Buf1500,"%08X %08X %08X %08X\r\n",systemREG2->DIEIDL_REG0,systemREG2->DIEIDH_REG1,systemREG2->DIEIDL_REG2,systemREG2->DIEIDH_REG3);
                //    putBuf(prt, Buf1500,0);                 //send to current active port
                //    break;
                //    }
                else if (!strcmp(tok, "RDI"))               //read data from fpga address and increment
                    {
                    //rdm mode is best for smaller xfers as it does NOT support
                    //background xmits and will tie up all ports until finished
                    sPTR saddr= f_Addr_CSR;
                    int i;
                   //get 1st param now
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param, rd addr hex
                        { param1=0;}                       //default start addr 0
                    //address only, no board number
                    param1 &= 0x7FFFF;
                    if ( (param2=arg_hex(&paramPtr,-1))==-1)//get read cnt hex
                        { param2= 16;}                      //cnt per line
                    if (param2 > 1024)                      //limit size
                        param2= 1024;                       

                    *Buf1500=0;
                    saddr +=param1;
                    for (i=1; i<param2+1; i++)              //limit size less than Buf1500;
                        {
                        sprintf (tBuf,"%4x ", *saddr++);
                        strcat(Buf1500, tBuf);
                        if (i%16==0)                        //16 words per line
                            {
                            strcat(Buf1500,"\r\n");
                            putBuf(prt, Buf1500,0);          
                            *Buf1500=0;                     //end of string
                            }
                        }
                    //if data in buffer, send it
                    if(*Buf1500)
                        {
                        strcat(Buf1500, "\r\n");
                        putBuf(prt, Buf1500,0);          
                        }
                    break;
                    }
                else if (!strcmp(tok, "RFI"))               //read data from FLASH address and increment
                    {
                    sPTR saddr= pFLASHbase;                 //flash base 0x64000000
                    int d16;
                    //get 1st param now
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param, rd addr hex
                        { param1= 0;}                       
                    param2= arg_hex(&paramPtr,256);         //get 2nd param, read cnt
                    
                    saddr +=param1;                         //max wrd addr 0x800000 for chip sel addr block
                    if((param1+param2)> 0x800000)           //addr out of range wrd cnt, will cause abort
                        {parErr=0xf; break;}
                    
                    *Buf1500=0;
                    for (int i=1; i<param2+1; i++)          //Note: Buf1500 is 1500 bytes
                      {
                       d16= *saddr++;
                       sprintf (tBuf,"%04x ", d16);
                       strcat(Buf1500, tBuf);
                       if (i%8==0)                          //8 words per line
                           {
                           strcat(Buf1500,"\r\n");
                           putBuf(prt, Buf1500,0);          //Note:putBuf to hBus will change fpga ptrs
                           *Buf1500=0;                      //set string size ==0
                           }
                      }
                    //if data in buffer, send it
                    if(*Buf1500)
                        {
                        strcat(Buf1500, "\r\n");
                        putBuf(prt, Buf1500,0);          
                        }
                    break;
                    }
           else if (!strcmp(tok, "RDM"))                    //read data from fpga address
                    {
                    //rdm mode is best for smaller xfers as it does NOT support
                    //background xmits and will tie up all ports until finished
                    int index;
                    //get 1st param now
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param, lword read addr hex
                        { parErr++;  break; }
                    //address only, no board number
                    param1 &= 0x7FFFF;
                    if ( (param2=arg_hex(&paramPtr,-1))==-1)//get 2nd param, lword read cnt hex
                        { parErr++;  break; }
                    param3= arg_dec(&paramPtr, 8);          //get 3rd param, line incr cnt
                    if ((param3<1) || (param3>16))          //limit sizes
                        param3=8;
                    sPTR saddr = (sPTR) fpgaBase;
                    saddr += param1;                        //adding to pointer, incr by ptr size
                    *Buf1500=0;
                    for (int i=1; i<param2+1; i++)          //Note: Buf1500 is 1500 bytes
                      {
                       saddr = (sPTR)fpgaBase+ param1;
                       index= *saddr;

                       sprintf (tBuf,"%04x ",index);
                       strcat(Buf1500, tBuf);
                       //read lower addr to advance pointer
                       if (i%param3==0)                     //words per line
                           {
                           strcat(Buf1500,"\r\n");
                           putBuf(prt, Buf1500,0);          //Note:putBuf to hBus will change fpga ptrs
                           *Buf1500=0;                      //set string size ==0
                           }
                      }
                    //if data in buffer, send it
                    if(*Buf1500)
                        {
                        strcat(Buf1500, "\r\n");
                        putBuf(prt, Buf1500,0);          
                        }
                    break;
                    }
           else if (!strcmp(tok, "RDS"))                    //read 32bit data from fpga address
                    {
                    //rdm mode is best for smaller xfers as it does NOT support
                    //background xmits and will tie up all ports until finished
                    int index;
                    //get 1st param now
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//get 1st param, lword read addr hex
                        { parErr++;  break; }
                    //address only, no board number
                    param1 &= 0x7FFFF;
                    if ( (param2=arg_hex(&paramPtr,-1))==-1)//get 2nd param, lword read cnt hex
                        { parErr++;  break; }
                    param3= arg_dec(&paramPtr, 8);          //get 3rd param, lword read cnt hex
                    if ((param3<0) || (param3>16))          //limit sizes
                        param3=8;
                    sPTR saddr = (sPTR) fpgaBase;
                    saddr += param1;                        //adding to pointer, incr by ptr size
                    *Buf1500=0;
                    for (int i=1; i<param2+1; i++)          //Note: Buf1500 is 1500 bytes
                      {
                       saddr = (sPTR)fpgaBase+ param1;
                       index= *saddr;

                       sprintf (tBuf,"%02x ",index);
                       strcat(Buf1500, tBuf);
                       //read lower addr to advance pointer
                       if (i%param3==0)                     //words per line
                           {
                           strcat(Buf1500,"\r\n");
                           putBuf(prt, Buf1500,0);          //Note:putBuf to hBus will change fpga ptrs
                           *Buf1500=0;                      //set string size ==0
                           }
                      }
                    //if data in buffer, send it
                    if(*Buf1500)
                        {
                        strcat(Buf1500, "\r\n");
                        putBuf(prt, Buf1500,0);          
                        }
                    break;
                    }
                  else if (!strcmp(tok, "RDEBLK"))            //ethernet block read/display 256 bytes
                    {
                    sPTR saddr = (sPTR) wizBase;            //now reading with ethernet offset
                    int short dat16;
                    param1=arg_hex(&paramPtr,0);            //get 1st param, def=0
                    saddr += param1/2;
                    *Buf1500=0;                             //set string size ==0
                    for (int i=0;i<32*4;i++)                    //Note: Buf1500 is 1500 bytes
                      {
                       if (i%8==0)
                           { 
                           if (i%32==0)
                               {
                                strcat(Buf1500,"\r\n");
                                putBuf(prt, Buf1500,0);     //send to current active port
                                *Buf1500=0;                 //set string size ==0
                               }
                           sprintf(tBuf,"\r\n%04X=",((int)saddr&0xffff) );
                           strcat(Buf1500, tBuf);
                           }
                       dat16 = *saddr++;
                       sprintf (tBuf," %04X",dat16&0xffff);
                       strcat(Buf1500, tBuf);
                      }
                    //cptr = (cPTR) WizReg0;
                    //*cptr=0x40;                           //enable memory test mode on wiznet
                    strcat(Buf1500,"\r\n");
                    putBuf(prt, Buf1500,0);                 //send to current active port
                    break;
                    }
                else if (!strcmp(tok, "RESET"))                    
                    {
                    //allow last usb chars to xmit
                     uDelay(1000);
                     _disable_interrupt_();
                     //resetEntry();
                     systemREG1->SYSECR = 0x80000; //BIT_n( 15U );
                     break;
                    }
                else  {parErr=0xf;   break; }
        case 'S':
                if (!strcmp(tok, "STAB"))       //read/return status block
                    {                           //"LSTAB" returns DATA POOL Reqs 'eCMD72_STAB' on LVDS 
                    int mode=0;
                    char hBuf[10];
                    param1= arg_dec(&paramPtr,4);  //get 1st param, fpga to process 1-4
                    if(param1==0)
                        {
                        putBuf(prt, (char *)HelpMenuSTAB, sizeof(HelpMenuSTAB)-1);        
                        break;
                        }
                    if((param1>4) || (param1<1))
                        {
                        mode=1;
                        param1=1;
                        }

                    stab(param1);               //update data buffer
                    *Buf1500=0;         
                    //read uC header
                    for (int ch=0,j=1; ch<sBLKSIZ22; ch++,j++)        //sSMALLSIZ==22     
                        {
                        sprintf (hBuf,"%4X ",statusBlock[0][ch]);
                        strcat(Buf1500, hBuf);
                        if (j%16==0)                //14 words per line, now 16
                            strcat(Buf1500,"\r\n");
                        }
                    strcat(Buf1500,"\r\n");
                    putBuf(prt, Buf1500,0);         //send to current active port
                    if (mode==1) 
                      break;
                    //loop displaying fpga data
                    for (int fpga=1; fpga<=param1; fpga++)
                        {
                        //reset output buffer for strcat use
                        *Buf1500=0;         
                        //read fpga(s) and HDMI port data
                        for (int ch=0,j=1; ch<sBLKSIZ38; ch++,j++)           
                            {
                            sprintf (hBuf,"%4X ",statusBlock[fpga][ch]);
                            strcat(Buf1500, hBuf);
                            if (j%16==0)                //14 words per line, now 16
                                strcat(Buf1500,"\r\n");
                            }
                        strcat(Buf1500,"\r\n");
                        putBuf(prt, Buf1500,0);         //send to current active port
                        }
                    break;
                    }
               else if (!strcmp(tok, "STAT"))
                    {
                    *Buf1500=0;                         //null buffer to be filled
                    header1TTY(prt);                    //show status registers
                    putBuf(prt, Buf1500,0);
                    putBuf(prt, "\r\n\r",2);
                    break;
                    }  
               else if (!strcmp(tok, "SOCK"))           //socket status reg 0x403,0x503,...
                    {
                    sockDisplay(prt, param1, paramPtr);
                    
    //extern uint8 getSn_KPALVTR(SOCKET s);
    uint8 kr;
    kr= getSn_KPALVTR(0);   //read it 
    sprintf(Buf1500,"keep alive= %d\r\n",kr);
    putBuf(prt, Buf1500,0);         //send to current active port
                    
                    
                    break;
                    }  
                else if (!strcmp(tok, "SET"))
                    {
                    int i;
                    if ( (param1=arg_dec(&paramPtr,-1)) ==-1)   //get 2nd param, option select
                        {
                        dispNet();                              //format network data to ascii
                        putBuf(prt, Buf1500,0);                 //send to current active port
                        break;
                        }
                    unsigned char *shtPtr;                      //load ram stucture with Atmel Flash data page IDINFO_PAGE
                    if      (param1==1) shtPtr = (uCHR*)&netInfo.ipAddr;
                    else if (param1==2) shtPtr = (uCHR*)&netInfo.gateWay;
                    else if (param1==3) shtPtr = (uCHR*)&netInfo.netMask;
                    else if (param1==4) shtPtr = (uCHR*)&netInfo.macAddr;
                    else if (param1==5) shtPtr = (uCHR*)&netInfo.sTelnet0;
                    else if (param1==6) shtPtr = (uCHR*)&netInfo.sTelnet1;
                    else if (param1==7) shtPtr = (uCHR*)&netInfo.sTelnet2;
                    //else if (param1==8) shtPtr = (uCHR*)&netInfo.sTelnet3;
                    else if (param1==8)  shtPtr = (uCHR*)&netInfo.sTcpTimeOut;
                    else if (param1==9)  shtPtr = (uCHR*)&netInfo.sTcpRetry;
                    else break;

                    if (param1<5)
                        {i=0;
                         char * p;
                         p= paramPtr;
                         while (*p)
                              {
                                if (*p == '.')
                                    *p=' ';
                                p++;                            //replace '.' with ' ' until 'cr'
                                i++;
                              }

                         for (i=0; i<netVars[param1-1]; i++)
                           {
                           if (param1==4)
                               param2= arg_hex(&paramPtr,-1);  //get macAddr hex
                           else
                               param2= arg_dec(&paramPtr,-1);  //get params dec
                           if (param2<0)
                               { parErr++; break; }
                           *shtPtr++ = (uCHR)param2;
                           }
                        }
                    else
                         {
                         param2 = arg_dec(&paramPtr,-1);        //get params dec
                         *(uLNG*)shtPtr++ = param2;
                         }
                    break;
                    }
                else if (!strcmp(tok, "SN"))                    //set serial number
                    {
                    param1=arg_dec(&paramPtr,-1);               //get 1st param, password
                    param2=arg_dec(&paramPtr,-1);               //get 2nd param, password
                    //note, struct 'uC_FRAM' filled from FRAM on power up
                    if ((param1==123) && (param2!=-1) && (param2<=9999)) //simple password followed by sn
                        {
                        uC_FRAM.serNumb= param2;
                        FRAM_WR_ENABLE();                      //WREN op-code issued prior to Wr_Op
                        //update serial number only
                        FRAM_WR(fPAGE_0400+idx_serNumb, (uint8*)&uC_FRAM.serNumb, 2); //addr,data,cnt
                        FRAM_WR_DISABLE();
                        sprintf(tBuf,"New SerNumb=%d\r\n\r",uC_FRAM.serNumb);
                        }
                    else
                        sprintf(tBuf,"SerNumb=%d\r\n\r",uC_FRAM.serNumb);
                    putBuf(prt, tBuf,0);                        //send to current active port
                    break;
                    }
                else if ((!strcmp(tok, "S4")) || (!strcmp(tok, "SD")) )//TEST code fpga read/writes
                    {                                          
                    //MT46H128M16LFDD-48 WT:C – 32 Meg x 16 x 4 Banks
                    //128 Megabytes = 134,217,728 Words
                    //testing 128Meg x 16 =  0x8000000 words
                    param1=arg_dec(&paramPtr,1);                //get 1st param, fpga 1-4                   
                    if ((param1<1) || (param1>4)) 
                        {parErr++;  break;}
                    
                    param2=arg_hex(&paramPtr,0x8000000);        //get 2nd param
                    sprintf(Buf1500, "Max Words=0x8000000  (134,217,728 WrdsDec),   MT46H128M16LF 128Meg x 16\r\n");
                    putBuf(prt, Buf1500,0);                     //send to current active port
                    sdRamTest(param1, param2, prt);             //chip(fpga) 1-4
                    break;
                    }
               if  (!strcmp(tok, "SB"))                         //TTYbaud set here
                    {
                    /** - set baudrate */
                    param1=arg_dec(&paramPtr,0);                //1st param
                    //if(prt!=tty)
                    //    {
                    //    putBuf(prt, "no change\r\n",0);         
                    //    break;
                    //    }
                    if(param1==1)
                    	{
                        uC_FRAM.TTYbaud= BAUD115200;
                        sprintf(tBuf,"Baud Rate Set and Stored in FRAM 115200\r\n");
                    	}
                    //save to Fram
                    else if(param1==2)
                    	{
                        uC_FRAM.TTYbaud= BAUD230400;
						sprintf(tBuf,"Baud Rate Set and Stored in FRAM 230400\r\n");
						}
                    else if(param1==3)
                    	{
                        uC_FRAM.TTYbaud= BAUD460800;
						sprintf(tBuf,"Baud Rate Set and Stored in FRAM 460800\r\n");
                    	}
                    else if(param1==4)
                    	{
                        uC_FRAM.TTYbaud= BAUD921600;
						sprintf(tBuf,"Baud Rate Set and Stored in FRAM 921600\r\n");
						}
                    else
                        {
                    	sprintf(tBuf,"No Change\r\n");
                        break;
                        }
                    putBuf(prt, tBuf,0);   
                    mDelay(100);                                //allow time to print last msg
                    sciSetBaudrate(UART, uC_FRAM.TTYbaud) ;                       
                    //sciREG->BRS = 14U/2;    //baudrate up from 450800 to 921600
                    
                    FRAM_WR_ENABLE();                        //WREN op-code issued prior to Wr_Op
                    //update baud number only
                    FRAM_WR(fPAGE_0400+idx_Baud, (uint8*)&uC_FRAM.TTYbaud, 4); //addr,data,cnt
                    FRAM_WR_DISABLE();
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'T':          
                if (!strcmp(tok, "TRIG"))                   //TRIG SOURCE
                    {   
                    uint16_t d16;  
                    param1=arg_dec(&paramPtr,-1);           //get 1st param
                    
                    if (param1==-1)
                        {
                        if (uC_FRAM.TrgSrcCh==1)
                            sprintf(tBuf,"%d= LEMO PULSE MODE\r\n", uC_FRAM.TrgSrcCh);
                        else
                            sprintf(tBuf,"%d= RJ45 FM MODE\r\n", uC_FRAM.TrgSrcCh);
                        putBuf(prt, tBuf,0);
                        break;
                        }
                    d16 = sTrigCntrl;
                    if(param1==0)      {SRCSEL_LO                   
                                        uC_FRAM.TrgSrcCh= param1; 
                                        hHI_ShtDwn; 
                                        sTrigCntrl=(d16|FMHI_PULLO);}  //rj45, set FPGA Reg
                    else if(param1==1) {SRCSEL_HI  
                                        uC_FRAM.TrgSrcCh= param1; 
                                        hLO_ShtDwn; 
                                        sTrigCntrl=(d16&~FMHI_PULLO);} //lemo, set FPGA Reg
                    break;
                    }
        case 'U':
                if ((!strcmp(tok, "UBUN"))||(!strcmp(tok, "UB")))//uBunch request status display
                   {                               
                    //added for testing tek may 2018
                    u_16Bit mask;
                    u_32Bit uB32REQ;
                    
                    _disable_interrupt_();                    
                    mask= (((sIN_MASK3&0xf)<<12) | ((sIN_MASK2&0xf)<<8) | ((sIN_MASK1&0xf)<<4) | ((sIN_MASK0&0xf)) );
                    Cntrl_tdcHDR.tMask= ((sIN_MASK3<<12) + (sIN_MASK2<<8) + (sIN_MASK1<<4) + sIN_MASK0);
                    
                    
                    sprintf(Buf1500,"uB Status Word       : Bits3-0, FPGA3-0 uBReq Err, BIT7-4 OvrFlow\r\n");
                    sprintf(tBuf,   "uB Status Word       : Bit8, FIFO not Empty after last uB req\r\n");
                    strcat (Buf1500,tBuf);                    
                    
                    sprintf(tBuf,   "uBunch Ch Mask 16-1  : %4X   (4Bits@Addr C21,821,421,021)\r\n",mask);
                    strcat (Buf1500,tBuf);                    
                    sprintf(tBuf,   "uBunch Counters      :\r\n");
                    strcat (Buf1500,tBuf);

                    sprintf(tBuf,   "   uB Req Total      : %-llu\r\n", paKet.uBunReqTot);
                    strcat (Buf1500,tBuf);
                    sprintf(tBuf,   "   uB Words Returned : %-llu\r\n", paKet.SndTotWrds);
                    strcat (Buf1500,tBuf);

                    
                    sprintf(tBuf,   "   ----------------- :\r\n");
                    strcat (Buf1500,tBuf);
                    
                    sprintf(tBuf,   "   Total Req Errors  : %-u\r\n", PaketStats.uBunReqErrs);
                    strcat (Buf1500,tBuf);
                    
                    sprintf(tBuf,   "   Over Flow BIT 15  : %-u\r\n", PaketStats.uBunOvrErrs);
                    strcat (Buf1500,tBuf);
                    
                    sprintf(tBuf,   "   Xmit Time Outs    : %-u\r\n", PaketStats.TOflag);
                    strcat (Buf1500,tBuf);

                    sprintf(tBuf,   "   Dropped Request   : %-u\r\n", PaketStats.eREC_BSY_DROP);
                    strcat (Buf1500,tBuf);
                    sprintf(tBuf,   "   ----------------- :\r\n");
                    strcat (Buf1500,tBuf);
                    
                    uB32REQ= ((pCtrl.uBunHI<<16)+(pCtrl.uBunLO));
                    sprintf(tBuf,   "   uB Req Last Rec'd : %08X\r\n", uB32REQ);
                    strcat (Buf1500,tBuf);                   
                    
                    PaketStats.uBunOvrErrs= 0;
                    PaketStats.TOflag= 0;
                    PaketStats.uBunReqErrs=0;
                    PaketStats.eREC_BSY_DROP=0;                                                 
                    paKet.uBunReqTot=0;
                    paKet.SndTotWrds=0;                     
                    putBuf(prt, Buf1500,0);
                    _enable_interrupt_();                    
                    break;
                   }
                else  {parErr=0xf;   break; }
        default:
                if (prt>=sock2)
                putBuf(prt,"\r\nsyntax error?\r\n",0);      //send to current active port
                break;
        }//end switch(*tok)

    if (parErr)
        {
        if(parErr==0xf)
            putBuf(prt,"\r\n?cmd\r\n",8);                   //cmd err, send to current active port
        else
            putBuf(prt,"\r\n?parm\r\n",9);                  //param err, send to current active port
        }
    if (genFlag & NO_ECHO)
        genFlag &= ~NO_ECHO;                                //one time turn off
    else
        { 
        //if controllers PhyLink_2_FEB, then no echo prompt
        //if sendBin() active, no prompt always
        if (prt!=ePhyIO)
            {
            if ((uC_FRAM.NewLinPrmpt==1) && (BinSt.gSndBytCnt==0))     //no '>' for TEST BEAM DAQ
                putBuf(prt,">",1);                          //new line prompt all telnet ports
            }
        }  
return 0; //end of process()
}




//-------------------- DMA CODE ------------------------------

/** void dmaConfigCtrlPacket(uint32 sadd,uint32 dadd,uint32 dsize)
*  configuring dma control packet stack
*       sadd  > source address
*       dadd  > destination  address
*       dsize > data size
*
*  note  after configuring the stack the control packet needs to be set by calling dmaSetCtrlPacket()
*  note  The frame/element count fields of the the  ITCOUNT register is only 13 bit wide, 
*  note  Hence a max of 8191 frames/elemets. bits 15:13 are ignored - so 8192 is same as 0.
*/
void dmaConfigCtrlPacket(uint32 sadd,uint32 dadd,uint32 dsize)
{
  g_dmaCTRLPKT.SADD      = sadd;			  /* source address             */
  g_dmaCTRLPKT.DADD      = dadd;			  /* destination  address       */
  g_dmaCTRLPKT.CHCTRL    = 0;                 /* channel control            */
  g_dmaCTRLPKT.FRCNT	 = 1;                 /* frame count                */
  g_dmaCTRLPKT.ELCNT     = dsize;             /* Element / Frame            */
  g_dmaCTRLPKT.ELDOFFSET = 0;                 /* element destination offset */
  g_dmaCTRLPKT.ELSOFFSET = 0;		          /* element destination offset */
  g_dmaCTRLPKT.FRDOFFSET = 0;		          /* frame destination offset   */
  g_dmaCTRLPKT.FRSOFFSET = 0;                 /* frame destination offset   */
  g_dmaCTRLPKT.PORTASGN  = 4;                 /* assign dma #               */
  g_dmaCTRLPKT.RDSIZE    = ACCESS_16_BIT;	  /* read size                  */
  g_dmaCTRLPKT.WRSIZE    = ACCESS_16_BIT; 	  /* write size                 */
  g_dmaCTRLPKT.TTYPE     = FRAME_TRANSFER ;   /* transfer type              */
  g_dmaCTRLPKT.ADDMODERD = ADDR_INC1;         /* address mode read          */
  g_dmaCTRLPKT.ADDMODEWR = ADDR_INC1;         /* address mode write         */
//g_dmaCTRLPKT.ADDMODEWR = ADDR_OFFSET;       /* address mode write         */
  g_dmaCTRLPKT.AUTOINIT  = AUTOINIT_ON;       /* autoinit                   */
}



void DMA_Initialization()
{
    //dest buffer is RDX0_dmaDATA or RDX1_dmaDATA;
    trigDMA0_fpga2mem1(8, &sTstCntLO);          //trig dma
    //DMA FROM FPGA 2 uC done?
    while (BinStRDX.gDMA_Fpga2Mem==1);
 
    trigDMA0_fpga2mem(8, &SDR_RD16SWP2);        //move from FPGA-to-MEMORY Buffer 			
    //DMA FROM FPGA 2 uC done?
    while (BinSt.gDMA_Fpga2Mem==1);
    fill_daq_packet(8, (u_8Bit*)&sTstCntLO);    //ByteCount, data buffer ptr    
   
}



//update nError counter 
//u_Store.nErrorCnt
void nError()
{
	FRAM_RD(fPAGE_0400+idx_nErrCnt, (uint8*)&uC_FRAM.nErrCnt, 2); //addr,data,cnt
    uC_FRAM.nErrCnt++;
    FRAM_WR_ENABLE();                      
    FRAM_WR(fPAGE_0400+idx_nErrCnt, (uint8*)&uC_FRAM.nErrCnt, 2); //addr,data,cnt
    sprintf(Buf1500,"**********************************************\r\n");
    sprintf(tBuf,   "**  nERROR ECC ReBoot.  FRAM ECC CNT= %04d  **\r\n", uC_FRAM.nErrCnt);
    strcat(Buf1500, tBuf);
    sprintf(tBuf,   "**********************************************\r\n");
    strcat(Buf1500, tBuf);
    putBuf(tty,Buf1500,0);                        
}




//Integer to ASCII Conversion
static char aBuf[32];
char* itoa(int val, int base)
{
   	int i = 30;
    if (val==0)
        {
        sprintf(aBuf,"00000000");
    		  return &aBuf[0];
        }

		  for(; val && i ; --i, val /= base)
		  aBuf[i] = "0123456789ABCDEF" [val % base];
		  return &aBuf[i+1];
}


void HexDump(char *addr, int len, int prt){
	char	*s=addr, *endPtr=(char *)((long)addr+len);
	int		i, remainder=len%16;
	
	//printf("\r\nOffset      Hex Value                                        Ascii value\r\n");
	
	// print out 16 byte blocks.
	while (s+16<=endPtr)
        {
		// 16 bytes
		for (i=0; i<16; i++)
            {
			sprintf(tBuf,"%02x ", s[i]);
            putBuf(prt, tBuf,0);
            }
		putBuf(prt, " ",0);
		for (i=0; i<16; i++)
            {
			if(s[i]>=32 && s[i]<=125)	
              sprintf(tBuf,"%c", s[i]);
			else
              sprintf(tBuf,".");
            putBuf(prt, tBuf,0);
            }
		s += 16;
		putBuf(prt,"\r\n",0);
        }
	
	// Print out remainder.
	if (remainder)
        {
		// 16 bytes
		for (i=0; i<remainder; i++)
            {
			sprintf(tBuf,"%02x ", s[i]);
            putBuf(prt, tBuf,0);
            }
		for (i=0; i<(16-remainder); i++)
            {
			putBuf(prt,"   ",0);
            }
		putBuf(prt," ",0);
		for (i=0; i<remainder; i++)
            {
			if		(s[i]>=32 && s[i]<=125)	
                {
                sprintf(tBuf,"%c", s[i]);
                putBuf(prt, tBuf,0);
                }
			else
              putBuf(prt, ".",0);
            }
		for (i=0; i<(16-remainder); i++)
            {
			putBuf(prt, " ",0);
            }
		putBuf(prt,"\r\n",0);
        }
	return;
}	// HexDump.





//string to upper case
char myupper(char ch)
{
    char val=0;
    if (ch>='a' && ch<='z')
      val = ch-' ';
    else
      val = ch;
 return val;
}


//delay in micro-seconds (~1.8uS overhead)
//uses RM48 (counter1 and compare1)
//32bit delay value
//waits for delay to finish, then returns
void uDelay(uint32 cnt)
{   
    int timeout=0;
    if (cnt==0) cnt=1;
    cnt= cnt*10;
    cnt -=9;
    rtiStopCounter(rtiCOUNTER_BLOCK1);
    while(!rtiResetCounter(rtiCOUNTER_BLOCK1))
      if (timeout++>1000) break;
    rtiSetPeriod(rtiCOMPARE1, cnt);
    rtiREG1->CMP[rtiCOMPARE1].UDCPx = cnt;  //Added to the compare value on each compare match
    rtiREG1->CMP[rtiCOMPARE1].COMPx = cnt;  //Compared with selected free running counter
    rtiStartCounter(rtiCOUNTER_BLOCK1);
    genFlag |= uTimeOut;                    //timeout flag 0==timeout
    rtiEnableNotification(rtiNOTIFICATION_COMPARE1);
    while (genFlag & uTimeOut);             //wait for timeout 
}


//delay in micro-seconds (~1.8uS overhead)
//uses RM48 (counter1 and compare2)
//32bit delay value
//tek, code initial delay is set to 10uS in interrupt handler
//inits timer that sends datablock on LVDS link (normally pooled data)
//returns to code without any delays
void LVDS_Intr_Time(uint32 cnt)   
{   
    int timeout=0;
    if (cnt==0) cnt=8;
    cnt= cnt*8;                             //8 cnts/uS
    rtiStopCounter(rtiCOUNTER_BLOCK1);
    while(!rtiResetCounter(rtiCOUNTER_BLOCK1))  //min to no wait here
      if (timeout++>1000) break;
    rtiSetPeriod(rtiCOMPARE2, cnt);
    rtiREG1->CMP[rtiCOMPARE2].UDCPx = cnt;  //Added to the compare value on each compare match
    rtiREG1->CMP[rtiCOMPARE2].COMPx = cnt;  //Compared with selected free running counter
    rtiStartCounter(rtiCOUNTER_BLOCK1);
    genFlag |= uTimeOut;                    //timeout flag 0==timeout
    rtiEnableNotification(rtiNOTIFICATION_COMPARE2);
    //LVDS_Intr_Time has now enabled interrupts,'rtiNotification()' does the rest
}


//delay in milli-seconds
//software delay, waits here until done
void mDelay(uint32 cnt)
{
    mStime.g_timeMs=0;
    if (cnt<1) cnt=2;                   //under 1mS can not be timed by sysTick
    while (mStime.g_timeMs < cnt);      //wait till done in sysTick
}



//ADS1259 is a high-linearity, low-drift, 24-bits, No Missing Codes
//The data rates are programmable up to 14kSPS 
//Data and control communication are handled over a 4MHz, SPI-compatible
//Start conversions, active high
//Reset/Power-Down; reset is active low; hold low for power-down
//USING RM48 SPI PORT 2, ADS1259 NEEDs SPI DATA ON FALLING EDGE OF CLOCK, MSB first

//Conversions are started when START is taken high and are stopped when START is taken low.
//If START is toggled during a conversion, the conversion is restarted.
//DRDY goes high when START is taken high.
//DRDY is an output that indicates when conversion data are available.

//The ADS1259 outputs 24 bits of conversion data in binary twos complement format, 
// MSB first. The data LSB has a weight of VREF/(223 – 1). 
//A positive full-scale code 7FFFFFh, negative full-scale of 800000h.
//Checksum = MSB data byte + Mid data byte + LSB data byte + 9Bh.
//Data rate setting= 10SPS (default)

//In Pulse Control mode, DRDY remains low until the next conversion is started.
//Gate Control mode (default) Start goes HIGH, Converts cont indefinitely til START pin LOW

#define  ref24  0.00000048828125
#define  refLp  0.000000154         //looping time in nS
float ADC_DATA=0;

//read ADS1259 24BIT ADC
//data takes 100 millisecond to update
//
//tek todo: When time permits background continous read and buffer this data in  
//          block of maybe 10 words for faster data display 
//
float ADC_ads1259(int cnt)
{
//Uses SPI_2 for adc data reads (8 bits per read)
//ADC_START         (G16, SPI5_SOMI3)
//gFlag Status  'ADC_REQ','ADC_DONE'
    uint16 txTestData[16] = { 0x00, 0x00, 0x00, 0x00 };
    uint16 rxTestData[8] = { 0 };
    int32_t d24;
    float  f24;
    int tim=0;

    //start read and wait till done
    ADCSTART_HI
    uDelay(5);  
    //wait until done 100mSec sample time
    while(GIO_ADC_RDY)          //154nS loop (wait for gio port sample done)
        {
        if(tim++>125)           //max wait plus for 100mS default sample time
            {
            return 0;           //return 0 on error
            }
        mDelay(1);  
        }

    ADCSTART_LO                 //ADC Inactive Low
    //Initiate SPI1 Transmit and Receive through Polling Mode
    spiTransmitAndReceiveData(spiREG2, &dataconfig_ADC, 3, txTestData, (uint16*)rxTestData);
    //try using intr verion of above
    //spiSendAndGetData()
       
    //3-8Bit reads for 24bit data
    d24= (rxTestData[0]<<16) + (rxTestData[1]<<8) +(rxTestData[2]);
    //sign extend 24bits to 32bits for floating point
    if(d24>>23)
        d24= d24 | 0xff000000;
    f24= (float)d24 * ref24;
    ADC_DATA= f24;
    return f24;
}


// Make sure you are using signed variables and then use 2 shifts.
// long value;         // 32 bit storage
// value=0xffff;       // 16 bit 2's complement -1, value is now 0x0000ffff
// value = ((value << 16) >> 16); // value is now 0xffffffff

//PGA280 is a universal high-voltage instrumentation amplifier.
//The input gain extends from ?V/V (attenuation) to 128V/V
//This interface allows clock rates up to 10MHz.

//Bit 7 is the most significant bit (MSB)
//Command Byte 01T0 aaaa dddd dddd: 
//Write 'dddd dddd' to internal PGA280 register at address aaaa

//This register can control the gain and address for an external MUX in one byte
//Selectable gains (in V/V) are : 128, 64, 32, 16, 8, 4, 2, 1, 1/2, 1/4, and 1/8
//The gain is set to 1/8V/V after device reset or power-on. 

//The SPI communicates to the internal registers, starting with a byte for 
//command and address. It is followed by a single data byte 
//(exception: 11tx 0ccc requires no data byte).

//On a read command, the device responds with the data byte and the checksum byte. 
//If the checksum is not desired, setting CS to high terminates the transmission.

//write command for GAINs is at address (Gain and optional MUX register)
//B7 B6 B5 B4 B3 B2   B1   B0
//G4 G3 G2 G1 G0 MUX2 MUX1 MUX0

//G3 G2 G1 G0 (Gain)
//0  0  0  0  1/8
//0  0  0  1  1/4
//0  0  1  0  1/2
//0  0  1  1  1
//0  1  0  0  2
//0  1  0  1  4
//0  1  1  0  8
//0  1  1  1  16
//1  0  0  0  32
//1  0  0  1  64
//1  0  1  0  128

//Document page 27
//CS is active low; data are sampled with the negative clock edge. It is insensitive
//to the starting condition of SCLK polarity (SPOL = 1 or 0). See Figure 52 and Figure 53.
//
//Gain is controlled by Register 0.
//Using SPI2_CS1 (D3) 
//PGA280 gain set
int pga280(uint16 gain, uint16 reset)
{
    uint16 addr[4] = { 0x6000, 0x00, 0x00, 0x00 };    

    if (reset==RST280)              //reset chip
        {
        spiTransmitData(spiREG2, &dataconfig_PGA, 1, &reset);   //spi 16bit mode
        uDelay(500);
        }
      
    //shift to gain bits
    gain <<= 3;
    addr[0] += gain;
    spiTransmitData(spiREG2, &dataconfig_PGA, 1, &addr[0]);     //spi 16bit mode
    return 1;
}



//Gain is controlled by Register 0.
//Using SPI2_CS1 (D3) 
//PGA280 gain readback
int pga280read(uint16 gain)
{
    uint16 addr[4] = { 0x8000, 0x00, 0x00, 0x00 };    
    spiTransmitAndReceiveData(spiREG2, &dataconfig_PGA, 1, &addr[0], &addr[2]);  //spi 16bit mode
    return (char)addr[2];
}



//Equalizer Chip GS3140 SPI Data Port Write 16Bit
//Using SPI3_CS2 (B2) 
//Serial data is clocked into the GS3140 SDIN pin on the rising edge of SCLK. 
//Serial data is clocked out of the device from the SDOUT pin on the falling 
//edge of SCLK (read operation). SCLK is ignored when CS is HIGH.
//The maximum interface clock rate is 32MHz.
//
//Write GS3140 setup
int EQW_GS3140(uint16 *data, int cnt)
{
    //force 'CS' to stay low for multiple cycles
    dataconfig_EQUAL.CS_HOLD = TRUE;    //Hold 'cs' low
    spiTransmitData(spiREG3, &dataconfig_EQUAL, cnt, data++);
    return 1;
}


//Read GS3140 setup
int EQR_GS3140(uint16 *addr, uint16 *Destdata, int cnt)
{
    //force 'CS' to stay low for multiple cycles
    dataconfig_EQUAL.CS_HOLD = TRUE;    //Hold 'cs' low
    spiTransmitData(spiREG3, &dataconfig_EQUAL, 1, addr);
    spiReceiveData(spiREG3, &dataconfig_EQUAL, cnt, Destdata);
    dataconfig_EQUAL.CS_HOLD = FALSE;   //release'cs'
    return 1;
}



//MT46H128M16LFDD-48 WT:C – 32 Meg x 16 x 4 Banks
//testing 128Meg x 16 ==  0x8000000 words
int sdRamTest(int chip, int count, int prt)
{ 
    u_32Bit err=0, i;
    u_32Bit addr32, offset=0;
    u_16Bit data16;
    u_32Bit tdat32=0, data32;
    sPTR SDramRdHIx;
    sPTR SDramRdLOx;      
    sPTR ucSDramLOx;
    sPTR CSRx;

    if(chip==1)
        {offset=0x000; }    //fpga 1of4
    else if(chip==2)
        {offset=0x400; }    //fpga 2of4
    else if(chip==3)
        {offset=0x800; }    //fpga 3of4
    else if(chip==4)
        {offset=0xC00; }    //fpga 4of4 

    //set write sdRAM Addr via special sequence
    SET_SDADDR_WRx(offset,0,0);
    ucSDramLOx= ((&SDR_RD16)+offset);
    
    sprintf(Buf1500, "Fpga Wr Addr Ptrs 0x%X, 0x%X\r\n", offset+2, offset+3 );
    putBuf(prt, Buf1500,0); 
    sprintf(Buf1500, "Fpga Rd Addr Ptrs 0x%X, 0x%X\r\nWriting Data... Write 32bits, Incr data...\r\n", offset+4, offset+5 );
    putBuf(prt, Buf1500,0); 
    
    //divide test count by 2, testing as long words vs words
    count /=2;          //this test writes 1 32 word per count
    
    //1st write sdram data 
    for (i=1; i<=count; i++ )
        {
        tdat32++;
        *ucSDramLOx=tdat32>>16;        //write data to fpga sdram
        *ucSDramLOx=tdat32;            //write data to fpga sdram
        //if(i%0x10000==0)             //mix data up a little, good for addressing logic
         // t16 <<=1;
        }
        
    sprintf(Buf1500, "Reading Data...\r\n");
    putBuf(prt, Buf1500,0); 
    //set read fpga addr once
    addr32=0;
    tdat32 =0;
        
    //set read sdRAM Addr via special sequence
    SET_SDADDR_RDx(offset,0,0);
        
    //2nd read sdram data, test
    for (i=1; i<=count; i++)
        {
        tdat32++;
        data32= *ucSDramLOx;
        data16= *ucSDramLOx;
        data32= ((data32<<16)+ data16); 
        if (data32 != tdat32)
            {
            sprintf(Buf1500, "A=%-7X  WR= %-8X  RD= %-8X\r\n", addr32, tdat32, data32);
            putBuf(prt, Buf1500,0); 
            if (err++>10)
                break;
            }
        addr32+=2;
       // if(i%0x10000==0)          //mix data up a little, good for addressing logic
       //   t16 <<=1;
        }
 
    if (err)
        sprintf(Buf1500, "Test Done ... Error Cnt=%d\r\n",err );
    else
        sprintf(Buf1500, "Tested Okay ...  EndAddr=%X     EndDataValue=%X\r\n",addr32, tdat32);
    putBuf(prt, Buf1500,0);                 //send to current active port
    return 0;
}




//nHet setup stuff, tek
//This pointer is used by the HET driver to access the NHET1 memory.
#define hetRAM1 ((hetRAMBASE_t *)0xFF460000U)
//** nHET variable needed to ref struture to nHet1 ram base address **
__no_init volatile HETPROGRAM0_UN e_HETPROGRAM0_UN @ 0xFF460000U; //tms470 addr=0x800000;
//RM48L nHet0 coded to capture temp sensor pulse train width, rise/fall
//any below ref works for getting the nHet capture data
//    Ris2Fall= hetRAM1->Instruction[0].Data;
//    Fall2Ris= hetRAM1->Instruction[1].Data;
//                   
//    Ris2Fall= e_HETPROGRAM0_UN.Memory0_PST[0].data_word;
//    Fall2Ris= e_HETPROGRAM0_UN.Memory0_PST[1].data_word;
//
//    Ris2Fall= HET_L00_0.memory.data_word;
//    Fall2Ris= HET_L01_0.memory.data_word;
#define N_Addr_LO5    (sPTR)(0xFF460000U+0x58)        //FPGA CSR reg
//nHET timer resolution is 100nS per bit (VLCK2=110MHZ, HR=10, LR=0, ACTUAL LR TIME=100nS)
#define MAXDELAY 26214              //MAX delay time in uS
//Max time loaded in counter <= 0x01ff ffxx, 0x000001xx == 100nS
//as seen in register the max value is 0x01ff-ffff (25bits)
//Does no use uC sysTick mSec timer
//Hardware delay using RM48L nHET timer, Min Delay 1uS
//
void hDelayuS(uint32 dly)
{
    if (dly> MAXDELAY)
        dly= MAXDELAY;
    if (dly>1)                                  //equate to 250nS per cnt
        {
        dly--;
        dly <<= 8;                              //times 256
        dly *=5;
        }
    else
        dly <<= 8;                              //times 256

    hetREG1->INTENAS= 1<<8;                     //nHet instr that gen irq 0,1,2,3...
    genFlag |= hDelay;                          //set het timer flag, intr clears it
    e_HETPROGRAM0_UN.Program0_ST.TIMERCOMP_0.memory.data_word= dly; //refer cnt loaded
    e_HETPROGRAM0_UN.Program0_ST.ENCOUNTER_0.memory.data_word=0;    //cnt up var, clr it
    e_HETPROGRAM0_UN.Program0_ST.START_0.memory.data_word=1<<7;     //enable counter bit
    //hetRAM1->Instruction[7].Data=0;           //clear het up counter reg7
    //hetRAM1->Instruction[8].Data=dly;         //load het refer count reg8
    //hetRAM1->Instruction[0].Data=1<<7;        //set counter start bit reg0
    while (genFlag & hDelay);                   //wait to finish, nonzero==busy
}


//Display Wiznet Network IC setup params
void dispNet()
{
    sprintf(Buf1500,"1)  LOCAL IP ADDR : %u.%u.%u.%u\r\n", *(netInfo.ipAddr),
                  *(netInfo.ipAddr+1),*(netInfo.ipAddr+2),*(netInfo.ipAddr+3));

    sprintf(tBuf,"2)  G/W IP ADDR   : %u.%u.%u.%u\r\n", *(netInfo.gateWay),
                *(netInfo.gateWay+1),*(netInfo.gateWay+2),*(netInfo.gateWay+3));
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"3)  SUBNET MASK   : %u.%u.%u.%u\r\n", *(netInfo.netMask),
                  *(netInfo.netMask+1),*(netInfo.netMask+2),*(netInfo.netMask+3));
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"4)  MAC ADDRESS   : %.2x.%.2x.%.2x.%.2x.%.2x.%.2x '6 Bytes Hex'\r\n",
                  *(netInfo.macAddr),*(netInfo.macAddr+1),*(netInfo.macAddr+2)
                  ,*(netInfo.macAddr+3),*(netInfo.macAddr+4),*(netInfo.macAddr+5));
    strcat(Buf1500, tBuf);

    sprintf(tBuf,"5)  Telnet Port 0 : %04d (d)          '(\\r) ending Cmds,(128 BytesMax)'\r\n", netInfo.sTelnet0);
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"6)  Telnet Port 1 : %04d (d)\r\n", netInfo.sTelnet1);
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"7)  Telnet Port 2 : %04d (d)\r\n", netInfo.sTelnet2);
    strcat(Buf1500, tBuf);
    //sprintf(tBuf,"8)  Telnet Port 3 : %04d (d)\r\n", netInfo.sTelnet3);
    //strcat(Buf1500, tBuf);

    sprintf(tBuf,"8)  TCP Timeout   : %04d (d)          'Word, (100uS/cnt), 10=1mS'\r\n", netInfo.sTcpTimeOut);
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"9)  TCP Retry     : %04d (d)          'Byte, TCP Max Re-Xmits after timeout'\r\n", netInfo.sTcpRetry);
    strcat(Buf1500, tBuf);

    sprintf(tBuf,"    Use cmd 'SET' as SET 3 255.255.255.0 or SET 4 1a 2b 3c 4d 5e 6f\r\n");
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"    Use cmd 'NETSAV' when finished.  The next RESET will use this saved setup.\r\n");
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"    To restore network defaults use command NETRST.\r\n");
    strcat(Buf1500, tBuf);
}



//read saved FRAM data and Load to FPGA Registers
//FRAM has 4 setup options as Page 1-4, Page=0 loads default setup
void dataRecall(int page, u_32Bit fADDR)
{
    //recall from FRAM
    static u_16Bit D16[8];
    sPTR saddr;
    sPTR frPTR= (uint16*)&uC_FRAM;      //FRAM data ptr 
    u_32Bit addr32;
    uint16 framAddr;                    //start of saved data
    uint16 cnt=0, data16, chip, param;
    uint16 ValidFlag;
    
    if (fADDR==fpgaBase)                //select fram addr per fpga address
      {framAddr= fPAGE_FPGA0+4; chip=0;}//fpga0
    else if (fADDR==fpgaBase1)
      {framAddr= fPAGE_FPGA1+4; chip=1;}//fpga1
    else if (fADDR==fpgaBase2)
      {framAddr= fPAGE_FPGA2+4; chip=2;}//fpga2
    else if (fADDR==fpgaBase3)
      {framAddr= fPAGE_FPGA3+4; chip=3;}//fpga3
    
    FRAM_RD_BUF16((framAddr-4), &ValidFlag, 1); //framAddr, fpgaAddr, read Cnt
    ValidFlag= ValidFlag&0xfff0;
    if (page==0)                        //param1=0=load constants
        ValidFlag=0;                    //force constant load
    
    if (ValidFlag == FP_REGS_VALID0)    //param1=0=load constants
        {
        //Stored Params Good, move FRAM to FEB Registers 'using constant structure for Adress Map'
        for(cnt=1; cnt<FP_REGS_MAX; cnt++)
            { 
            //read data block into temp storage D16[]array
            //FRAM Page 1-4 for different FRAM stored setup modes
            FRAM_RD_BUF16(framAddr+ ((page-1)*0x100), D16, f_RegConst[cnt].RegCnt);    //framAddr, fpgaAddr, writeCnt
            framAddr += f_RegConst[cnt].RegCnt*2;
            addr32 = (u_32Bit)fADDR+ (f_RegConst[cnt].Addr<<1);      //actual 32 bit address
            
            //load fpga reg, note special case if reg addr >= 0x100
            for (int i=0; i< f_RegConst[cnt].RegCnt; i++, addr32+=2)
                {
                if (f_RegConst[cnt].Addr < 0x100)
                    *(sPTR)addr32= D16[i];      //fpag range
                
                else if ((f_RegConst[cnt].Addr >= 0x300) && (fADDR==fpgaBase)) //broadcast range (only fpga0)
                    *(sPTR)addr32= D16[i];      //fpga range
                
                else if ((f_RegConst[cnt].Addr >= 0x100) && (f_RegConst[cnt].Addr < 0x300))
                  { 
                  //fpga afe range
                  wrAFE_Slow(chip, f_RegConst[cnt].Addr+ i, D16[i]); //fpga as (0-3), reg, data
                  }
                uDelay(100);    //serial DACs need delay between writes, so delay for all              
                }
            }
        //get uC range data from FRAM, {Mux, Gain, TrgSrc, LnkDir, nError}
        FRAM_RD(fPAGE_0400, (uint8*)&uC_FRAM, FRAM_400wsz); //addr,data, Bytecnt tek may2019 fix '<<1'
        }
    else
        {
        //LOADING DEFAULT DATA HERE, USING DEFALUT STORAGE struct f_RegConst
        for(cnt=1; cnt<FP_REGS_MAX; cnt++)
            { 
            addr32 = (u_32Bit)fADDR + (f_RegConst[cnt].Addr<<1);  //actual 32 bit address
            for (int i=0; i< f_RegConst[cnt].RegCnt; i++, addr32+=2)
                {
                //read constant setUp data
                *D16 = f_RegConst[cnt].Value[i];
                if (f_RegConst[cnt].Addr < 0x100)  //load fpga reg, note special case if reg addr >= 0x100
                    {
                    *(sPTR)addr32= *D16;
                    }
                else if ((f_RegConst[cnt].Addr >= 0x300)  && (fADDR==fpgaBase))   //broadcast range (only fpga0)
                    {
                    *(sPTR)addr32= *D16;
                    }
                else if ((f_RegConst[cnt].Addr >= 0x100) && (f_RegConst[cnt].Addr < 0x300))
                    {
                    wrAFE_Slow(chip, f_RegConst[cnt].Addr + i, f_RegConst[cnt].Value[i]); //fpga as (0-3), reg, data
                    }
                }
            }
        //get uC range data from FRAM, {Mux, Gain, TrgSrc, LnkDir, nError}
        FRAM_RD(fPAGE_0400, (uint8*)&uC_FRAM, FRAM_400wsz); //addr,data, Bytecnt tek may2019 fix
        //bootup constants update uC Range {SerNumb, MuxCh, Gain, Trig, LnkDir, nErr, xPhy, wDogTimOut}
        //param= sizeof(uC_Const);
        param= uC_FRAM.serNumb; //save ser numb
        memcpy(&uC_FRAM, &uC_Const,   sizeof(uC_Const));  //load uC struct with setup def
        uC_FRAM.serNumb= param; //restore ser numb
        }
    //uC restored values here
    //16 HDMI Trim Volt Chs per fpga, select 1of4 MuxChip(s) ADG1609
    if (uC_FRAM.pgaMuxCh==0)      {MUXA3_LO; MUXA2_LO;}
    else if (uC_FRAM.pgaMuxCh==1) {MUXA3_LO; MUXA2_HI;}
    else if (uC_FRAM.pgaMuxCh==2) {MUXA3_HI; MUXA2_LO;}
    else if (uC_FRAM.pgaMuxCh==3) {MUXA3_HI; MUXA2_HI;}

    //set gain on PGA280
    pga280(uC_FRAM.pga280Gain, RST280);  //Set Gain, 0x11 forces a reset first
    //trig
    data16= sTrigCntrl;
    if(uC_FRAM.TrgSrcCh==0) {SRCSEL_LO hHI_ShtDwn; sTrigCntrl=(data16|FMHI_PULLO);}  //rj45, set FPGA Reg
    else                    {SRCSEL_HI hLO_ShtDwn; sTrigCntrl=(data16&~FMHI_PULLO);} //lemo, set FPGA Reg
    //linkdir
    if (uC_FRAM.LinkDir==0) 
        {LINK_DIR_LO}           //hetREG1->DCLR=BIT19
    else
        {LINK_DIR_HI}           //hetREG1->DSET=BIT19
}



//Send binary data from any register, initially used for speed testing WIZNET 
//iniiiated by user cmd 'RDBR' 
int sendBinNonDAQ(int prt)
{
    unsigned int retVal=0, freeSize=0;              //Avail buf space while or after socket xmits data
    static uint16_t    dBytesMoved=0;
    //char msgBuf[80];
    hHI_TP45;
      
     if (BinSt.gSndBytCnt==0)
        return 0;
     
     
    //move 16bits data words
    if (dBytesMoved==0)                             //Note: datWrdsMoved should always an even number
        {
        //DMA's data,   FPGA==> uC_dmaBuf
        //DMA Stream enable
        BinSt.gDMA_Fpga2Mem=1;  //mark as active
        
        //max fill each time
        if (BinSt.gSndBytCnt >= D_SIZE_BYTE )
            dBytesMoved= D_SIZE_BYTE;
        else 
            dBytesMoved= BinSt.gSndBytCnt;

        //trap zero cout
        if (dBytesMoved > 0)  //addr to read from
           trigDMA0_fpga2mem(dBytesMoved/2, (sPTR)BinSt.gSndSrcSptr);       //move from FPGA-to-MEMORY Buffer 'RDB_dmaDATA[D_SIZE]'  
        }

    //possible no bytes were moved from 1of4 fpga(s), don't continue else dma flag will lock up code
    if (dBytesMoved== 0)
        return  BinSt.gSndBytCnt;
      
 
    freeSize= getSn_TX_FSR(prt);                    //get freesize in (bytes)
    if (freeSize < dBytesMoved)                     //wait for enough for bigger packet
        return  BinSt.gSndBytCnt;

    //now adjust stored byte count
    BinSt.gSndBytCnt -= dBytesMoved;                //freeSize is always <= MAX BUFFER

    if (freeSize==0)                                //zero, no room, start over, should never happen
        {
        sprintf(tBuf,"DiagSock_%1d, Free Buf Size=0\r\n",prt );
        putBuf(prt,tBuf,0);
        BinSt.gSndBytCnt= 0;                        //last packet, zero count
        dBytesMoved=0;
        return 0;
        }

    if (getSn_SSR(prt) != SOCK_ESTABLISHED)         //SOCKET CLOSED, STOP DATA XFER
        {
        sprintf(tBuf,"DiagSock_%1d, Socket Closed, Cancel 'RDB' WrdCnt=%0xX\r\n",prt, BinSt.gSndBytCnt);
        putBuf(prt,tBuf,0);
        BinSt.gSndBytCnt=0;
        dBytesMoved=0;
        return 0;
        }

    //DMA FROM FPGA 2 uC done?
    while (BinSt.gDMA_Fpga2Mem==1);
    //add timeout check here

    //Now move data (8Bits)dmaBuf==> ethernet socket
    //move to correct xmit Wiznet socket address (uint32_t)Sn_TX_FIFOR(prt)
    BinSt.gDMA_Mem2Wiz=1;
    trigDMA1_mem2wiznet(prt, dBytesMoved/2);
    
    //DMA FROM uC 2 Wiznet done?
    while (BinSt.gDMA_Mem2Wiz==1);
    //add timeout check here

    //packet buffer has been filled, now send it
    retVal= send_full(BinSt.gSndPrt,dBytesMoved);   //send to current active port, Byte count
    dBytesMoved= 0;                                 //ready for next pass
    if (retVal==0)                                  //in NULL, then socket error, stop xmits
        {
         //error, clear counts and exit
         sprintf(tBuf,"DiagSock_%1d, Wiznet Xmit Error , Cancel 'RDB' WrdCnt=%0xX\r\n",prt, BinSt.gSndBytCnt);
         putBuf(prt,tBuf,0);          
         BinSt.gSndBytCnt=0;
        }
    hLO_TP45;    
    return  BinSt.gSndBytCnt;
}



//Send Binary Data from FPGA DAQ Register, cmd 'RDB' call this sendBin()
//uC DMA moves data from all fpga(s), one fully at a time then uC DMA to Wiznet Socket
int sendBin(int prt)
{
    unsigned int retVal=0, freeSize=0;          //Available buf space while or after socket xmits data
    static uint16_t    dBytesMoved=0, fpgaEmp=0;
    hLO_TP45;                                   //for testing only
         
     if (BinSt.gSndBytCnt==0)
        return 0;

    //move 16bits data words
    if (dBytesMoved==0)                         //Note: datWrdsMoved should always an even number
        {
        //DMA's data,   FPGA==> uC_dmaBuf
        //DMA Stream enable
        BinSt.gDMA_Fpga2Mem=1;  //mark as active
        
        //max fill each time
        if (BinSt.gSndBytCnt >= D_SIZE_BYTE )
            dBytesMoved= D_SIZE_BYTE;
        else 
            dBytesMoved= BinSt.gSndBytCnt;

        //limit byte cnt to what remains in active fpga
        if (dBytesMoved > FPGA_Trig_Stat.BytToSnd)
            {
            dBytesMoved= FPGA_Trig_Stat.BytToSnd;
            fpgaEmp=1;
            }
        //readout sequence all, READ 4 FPGA as fpga0-3, if any data available
        //trap zero cout
        if (dBytesMoved > 0)
            {
            if (FPGA_Trig_Stat.Fpga1of4==0)
                trigDMA0_fpga2mem(dBytesMoved/2, &SDR_RD16SWP0);       //move from FPGA-to-MEMORY Buffer 'RDB_dmaDATA[D_SIZE]'  
            else if (FPGA_Trig_Stat.Fpga1of4==1)
                trigDMA0_fpga2mem(dBytesMoved/2, &SDR_RD16SWP1);       //move from FPGA-to-MEMORY Buffer 'RDB_dmaDATA[D_SIZE]'  
            else if (FPGA_Trig_Stat.Fpga1of4==2)
                trigDMA0_fpga2mem(dBytesMoved/2, &SDR_RD16SWP2);       //move from FPGA-to-MEMORY Buffer 'RDB_dmaDATA[D_SIZE]'  
            else if (FPGA_Trig_Stat.Fpga1of4==3)
                trigDMA0_fpga2mem(dBytesMoved/2, &SDR_RD16SWP3);       //move from FPGA-to-MEMORY Buffer 'RDB_dmaDATA[D_SIZE]'  
            //if req larger than data pool start over at fpga 0
            else if (FPGA_Trig_Stat.Fpga1of4>3)
                {
                trigDMA0_fpga2mem(dBytesMoved/2, &SDR_RD16SWP0);       //move from FPGA-to-MEMORY Buffer 'RDB_dmaDATA[D_SIZE]'  
                }
            }

        //End of fpga buffer, go to next fpga
        if (fpgaEmp==1)
            {
             do {
                if (FPGA_Trig_Stat.Fpga1of4 < 4)
                    FPGA_Trig_Stat.Fpga1of4++;
                if (FPGA_Trig_Stat.Fpga1of4==4)
                    {
                    //increment overrun counter, show in cmd 'HDR'
                    FPGA_Trig_Stat.OverRun++;
                    }
                fpgaEmp=0;
                FPGA_Trig_Stat.BytToSnd= FPGA_Trig_Stat.WrdCnt[FPGA_Trig_Stat.Fpga1of4]<<1;
                if (FPGA_Trig_Stat.Mask[FPGA_Trig_Stat.Fpga1of4] == 0)
                    FPGA_Trig_Stat.BytToSnd=0;
                } while (FPGA_Trig_Stat.BytToSnd==0);  //if fpag(n) has no data, then skip
            }
        else  
            {
            FPGA_Trig_Stat.BytToSnd -= dBytesMoved;
            }
        }
    //possible no bytes were moved from 1of4 fpga(s), don't continue else dma flag will lock up code
    if (dBytesMoved== 0)
        return BinSt.gSndBytCnt;     
 
    freeSize= getSn_TX_FSR(prt);                    //get freesize in (bytes)
    if (freeSize < dBytesMoved)                     //wait for enough for bigger packet
        return BinSt.gSndBytCnt;

    //DMA FROM FPGA 2 uC done?, if not exit, later code continues from here
    if (BinSt.gDMA_Fpga2Mem==1)
        return  BinSt.gSndBytCnt;
    
    //now adjust stored byte count
    BinSt.gSndBytCnt -= dBytesMoved;                //freeSize is always <= MAX BUFFER

    if (freeSize==0)                                //zero, no room, start over, should never happen
        {
        printf("DiagSocket, Free Buf Size=0\r\n");
        BinSt.gSndBytCnt= 0;                        //last packet, zero count
        dBytesMoved=0;
        return 0;
        }

    if (getSn_SSR(prt) != SOCK_ESTABLISHED)         //SOCKET CLOSED, STOP DATA XFER
        {
        printf("DiagSocket, Socket Closed\r\n");
        BinSt.gSndBytCnt=0;
        dBytesMoved=0;
        return 0;
        }

    //Now move data (8Bits)dmaBuf==> ethernet socket
    //move to correct xmit Wiznet socket address (uint32_t)Sn_TX_FIFOR(prt)
    BinSt.gDMA_Mem2Wiz=1;
    trigDMA1_mem2wiznet(prt, dBytesMoved/2);
    
    //DMA FROM uC 2 Wiznet done?
    while (BinSt.gDMA_Mem2Wiz==1);
    //add timeout check here

    //packet buffer has been filled, now send it
    retVal= send_full(BinSt.gSndPrt,dBytesMoved);   //send to current active port, Byte count
    dBytesMoved= 0;                                 //ready for next pass
    if (retVal==0)                                  //in NULL, then socket error, stop xmits
        {
         //error, clear counts and exit, should never happen
         //todo: add error counter this is just a msg to slow USB Port
         printf("DiagSocket, Wiznet Xmit Error\r\n");
         BinSt.gSndBytCnt=0;
         return 0;
        }
    hHI_TP45;    
    return  BinSt.gSndBytCnt;
}



//dma 0 used to copy data from FPGA to uC Memory buffer
//inits dma data type, addresses, xfer count
int trigDMA0_fpga2mem(uint16_t wordCnt, uint16_t *RD16SWP )  
{
	dmaDisable();
	//assigning dma request: channel-0 with request line - 1
	dmaReqAssign(0,1 );
    //prevent DMA lockup,count > zero
    if(wordCnt==0)
      wordCnt=8;
	//configuring dma control packets   (srcadd, destadd, datasize)
	dmaConfigCtrlPacket((uint32)RD16SWP, (uint32)&RDB_dmaDATA, wordCnt );
	g_dmaCTRLPKT.RDSIZE = ACCESS_16_BIT;    //change read size to 16 bits
    g_dmaCTRLPKT.ADDMODERD = ADDR_FIXED;    //address mode read fpga fixed addr
    g_dmaCTRLPKT.ADDMODEWR = ADDR_INC1;     //address mode write uC buf

	// setting dma control packets, upto 32 control packets are supported
	dmaSetCtrlPacket(DMA_CH0 ,g_dmaCTRLPKT);

	//setting the dma channel 0 to trigger on software request
	dmaSetChEnable(DMA_CH0, DMA_SW);
	dmaEnableInterrupt(DMA_CH0, FTC);
	//enabling dma module
	dmaEnable();
    return 0;
}
        

//dma 1 used to copy data from uC Memory to Wiznet xmit socket
int trigDMA1_mem2wiznet(uint16_t prt, uint16_t wordCnt)  
{
	dmaDisable();
	//assigning dma request: channel-0 with request line - 1
	dmaReqAssign(1,1 );
    //prevent DMA lockup,count > zero
    if(wordCnt==0)
      wordCnt=8;
	//configuring dma control packets   (srcadd, destadd, datasize)
	dmaConfigCtrlPacket((uint32)&RDB_dmaDATA, (uint32)Sn_TX_FIFOR(prt), wordCnt );
	g_dmaCTRLPKT.RDSIZE = ACCESS_16_BIT;    //change read size to 16 bits
    g_dmaCTRLPKT.ADDMODERD = ADDR_INC1;     //address mode read uC buffer
    g_dmaCTRLPKT.ADDMODEWR = ADDR_FIXED;    //address mode write to wiznet socket

	// setting dma control packets, upto 32 control packets are supported
	dmaSetCtrlPacket(DMA_CH1 ,g_dmaCTRLPKT);

	//setting the dma channel 1 to trigger on software request
	dmaSetChEnable(DMA_CH1, DMA_SW);
	dmaEnableInterrupt(DMA_CH1, FTC);
	//enabling dma module
	dmaEnable();
    return 0;
}


//send 'telent tcp packet' buffer is already filled
//this is a local replace for wiznets send()
//used by sendBin and sendBinNonDAQ to trig Wiznet Xmit
//has time delay var to show any delays on dest node not being ready
//
uint32 send_full(SOCKET s, uint32 len)              //len is byte count
{
    uint32 ret= len;
    static int ii;
    ii=0;
    if(!check_sendok_flag[s])                       // if first send, skip.
        {
        while (!(getSn_IR(s) & Sn_IR_SENDOK))       // wait previous SEND command completion.
            {
            if(ii++ > 20000)
                {
                //1 second time, then close socket and break
                close(s);                           //close the SOCKET
                //printf("WizSnd  : Close Socket\r\n");
                eTout[s]++;
                return 0;
                }
            uDelay(100/2);
            if (getSn_SSR(s) == SOCK_CLOSED)        //check timeout or abnormal closed.
                {
                //printf("WizSnd  : Close Socket\r\n");
                eTout[s]++;
                return 0;
                }
            }
       setSn_IR(s, Sn_IR_SENDOK);                   //clear Sn_IR_SENDOK
       }
   else
        check_sendok_flag[s] = 0;
    //keep track of delays
    if (ii>800)         //20ms
       BinSt.gDMA_TimDlys[0]++;
    else if (ii>400)    //10ms
       BinSt.gDMA_TimDlys[1]++;
    else if (ii>200)     //5ms
       BinSt.gDMA_TimDlys[2]++;
    else if (ii>40)     //1ms
       BinSt.gDMA_TimDlys[3]++;
    else if (ii>20)     //.1ms
       BinSt.gDMA_TimDlys[4]++;
   // send bytes amount='len' that is already loaded into sockets xmit buffer
   setSn_TX_WRSR(s,len);
   hHI_TP45;
   hLO_TP45;
   hHI_TP45;
   
   setSn_CR(s,Sn_CR_SEND);
   return ret;
}

//IAR
//To treat a floating-point constant as a float rather than as a double, add the 
//suffix f to it, for example:  double Test(float a) {return a + 1.0f;}

//read temperature chip tmp04 using het seq logic 'TMP05A'
//Temperature (°C) = 421 - (751 × (TH/TL))
//nominal conversion of TH/TL = 34ms/65ms at 25°C
//typical period of 99 ms at 25°C (CONV/IN pin is left floating)
float readTemperature()
{   
    float tempC;
    Ris2Fall= hetRAM1->Instruction[4].Data;
    Fall2Ris= hetRAM1->Instruction[5].Data;
  //tempF = 455-( (720*(float)Ris2Fall) /(float)Fall2Ris);
    tempC = 421-( (751* ((float)Ris2Fall) /(float)Fall2Ris));
    //lock out of range data
    if (Ris2Fall<1000)
        tempC=0;
    return tempC;
}


//uC adc only runs when trigger
//this keeps any possible sampling noise down
//
void adcRefresh()
{
    //check if new ADC Data req by user 'ADC_REFRESH'
    //this code updates uc ADC array, runs 'On Demand' by cmd 'ADC'
    int aData;
    if (genFlag & ADC_REFRESH) 
        {
        if (genFlag & ADC_TRIG)
            {
            adcStartConversion(adcREG1,adcGROUP1);
            genFlag &= ~(ADC_TRIG+ ADC_READY);  //clr req, mark adc not ready
            }
        else if ((genFlag & ADC_READY)==0) //if not ready, check it
            {
            //wait and read the conversion count, should always be ready
            if (adcIsConversionComplete(adcREG1,adcGROUP1))//short routine, (1 op)
                {
                genFlag |= ADC_READY;     //main loop check, data should be rdy
       //adcStopConversion(adcREG1,adcGROUP1); //tek Jun2019 mod
                }
            }
        if (genFlag & ADC_READY)
            {
            //Only update if new request for data reads
            adcGetData(adcREG1, adcGROUP1,&adc_data[0]);    //16 reads 
            
            //adjust array to match correctly to displays labeled groups
            aData= adc_data[8].value;       //temporary save
            adc_data[8].value= adc_data[7].value;
            adc_data[7].value= aData;
            genFlag &= ~ADC_REFRESH;
            genFlag |=  ADC_TRIG;           //preset trig for next request
            }
        }
}


//check fpga for runtime CheckSum Error
//return status of all 4 devices
int fpga_Check()
{
    int stat=0, retVal;
    if (InitB0_Read==0)    //check, low=error
       stat = 0x1;
    if (InitB1_Read==0)    //check, low=error
       stat +=0x2;
    if (InitB2_Read==0)    //check, low=error
       stat +=0x4;
    if (InitB3_Read==0)    //check, low=error
       stat +=0x8;
    if (stat==0)
      return 0;             //no problems, return now

    
    //Problems found, ReInit the FPGA
    //
    //load fpga from flash data
    retVal= flashXFER(4, tty, FLBASE);  //load all readout FPGA(s)
    //CONFIG FAILED, try FLASH backup
    if (retVal == -1)                   
        {
        uDelay(500);
        flashXFER(4, tty, FLBACKUP);    //load FPGA(s) with backup data
        }
    //needs setup time before loading fpga registers
    uDelay(500);
    
    //afe_A on each fpga
    wrAFE_Slow(0, 0x100, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    wrAFE_Slow(1, 0x100, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    wrAFE_Slow(2, 0x100, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    wrAFE_Slow(3, 0x100, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    //afe_B on each fpga
    wrAFE_Slow(0, 0x200, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    wrAFE_Slow(1, 0x200, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    wrAFE_Slow(2, 0x200, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    wrAFE_Slow(3, 0x200, 0x01);  //fpga(0-3),reg,data (afe soft reset)
    
    //General reset to AFE fifo, counter, sequencer
    CSR0= (0x20 | (CSR0));
    CSR1= (0x20 | (CSR1));
    CSR2= (0x20 | (CSR2));
    CSR3= (0x20 | (CSR3));

    //recall FPGA Setup Data from FRAM reloaded structure
    dataRecall(1, fpgaBase);       //fpga0
    dataRecall(1, fpgaBase1);      //fpga1
    dataRecall(1, fpgaBase2);      //fpga2
    dataRecall(1, fpgaBase3);      //fpga3
    return stat;
}




//Updates Trigger counts of all fpga(s) for display Diagnostics
//Also checks for overflows 
//
void FPGA_Trig_Cnts_Update()
{
    //Store trigger cnts of all fpga(s) for SendBin() and displayed diagnostics
    FPGA_Trig_Stat.WrdCnt[0]= ((s0WORD_CNTH<<16) + s0WORD_CNTL);    //32bit read
    FPGA_Trig_Stat.WrdCnt[1]= ((s1WORD_CNTH<<16) + s1WORD_CNTL);    //32bit read
    FPGA_Trig_Stat.WrdCnt[2]= ((s2WORD_CNTH<<16) + s2WORD_CNTL);    //32bit read
    FPGA_Trig_Stat.WrdCnt[3]= ((s3WORD_CNTH<<16) + s3WORD_CNTL);    //32bit read
    FPGA_Trig_Stat.WrdCnt[4]=  0x10001000;                          //32bits for req greater than fppa wordcnt
    FPGA_Trig_Stat.TotWrdCnt= 0;
    FPGA_Trig_Stat.OverMax=0;
    
    //total words of all fpgs
    if ((CSR0 & 3)==0)                                              //skip if in pwr down
        {
        if (FPGA_Trig_Stat.WrdCnt[0]> MAX_WORD_PER_FPGA)
            {
            FPGA_Trig_Stat.WrdCnt[0]= MAX_WORD_PER_FPGA;
            FPGA_Trig_Stat.OverMax+= 1<<0;
            }
        FPGA_Trig_Stat.TotWrdCnt += FPGA_Trig_Stat.WrdCnt[0];
        }
    if ((CSR1 & 3)==0)                                              //skip if in pwr down
        {
        if (FPGA_Trig_Stat.WrdCnt[1]> MAX_WORD_PER_FPGA)
            {
            FPGA_Trig_Stat.WrdCnt[1]= MAX_WORD_PER_FPGA;
            FPGA_Trig_Stat.OverMax+= 1<<1;
            }
        FPGA_Trig_Stat.TotWrdCnt += FPGA_Trig_Stat.WrdCnt[1];
        }        
    if ((CSR2 & 3)==0)                                              //skip if in pwr down
        {
        if (FPGA_Trig_Stat.WrdCnt[2]> MAX_WORD_PER_FPGA)
            {
            FPGA_Trig_Stat.WrdCnt[2]= MAX_WORD_PER_FPGA;
            FPGA_Trig_Stat.OverMax+= 1<<2;
            }
        FPGA_Trig_Stat.TotWrdCnt += FPGA_Trig_Stat.WrdCnt[2];
        }
    if ((CSR3 & 3)==0)                                              //skip if in pwr down
        {
        if (FPGA_Trig_Stat.WrdCnt[3]> MAX_WORD_PER_FPGA)
            {
            FPGA_Trig_Stat.WrdCnt[3]= MAX_WORD_PER_FPGA;
            FPGA_Trig_Stat.OverMax+= 1<<3;
            }
        FPGA_Trig_Stat.TotWrdCnt += FPGA_Trig_Stat.WrdCnt[3];
        }
    
    //increment counter just one on overflows
    if (FPGA_Trig_Stat.OverMax)
        FPGA_Trig_Stat.OverMaxCounter++;

    FPGA_Trig_Stat.Mask[0]=  sIN_MASK0; 
    FPGA_Trig_Stat.Mask[1]=  sIN_MASK1;
    FPGA_Trig_Stat.Mask[2]=  sIN_MASK2;
    FPGA_Trig_Stat.Mask[3]=  sIN_MASK3;
    //4 mask bits per register
    FPGA_Trig_Stat.Mask[4]= ((sIN_MASK3&0xf)<<12) | ((sIN_MASK2&0xf)<<8) | ((sIN_MASK1&0xf)<<4) | ((sIN_MASK0&0xf));
}



//	status block layout
//          1= serial number
//			1= spill cycle cnt
//			1= temperature
//		   16= adc channels (16)
//					
//          1= FPGA ADDR 303, trig cntrol reg        
//          1= FPGA ADDR 304, pipeline delay                      
//          1= FPGA ADDR 305, sample length       
//					
//		    +  //statusBlock is filled in bacground by 'OneWireTrigRead()'
//         24= afe DAC regs
//          4= afe Temperatures
//         10= afe ultraSound regs

//status block data fetch, see OneWireTrigRead()
//stab() takes about 450 uSec
void stab(int fcnt)
{
    uint16* tb= statusBlock[0];                 //use array[0] for board block data, ..array[1,2,3,4]hold fpga data
    uint16* fPtr;
    int ch;
    
  //FRAM_RD(SERIAL_NUMB_ADR, (uint16_t*)&param2,2); //read serial number
    *tb++= uC_FRAM.serNumb;                     //serial number
    *tb++= s0SP_COUNT;                          //spill cycle counter
                    
    //get adc readings
    //*tb++= (uint16)readTemperature()*10;        //int temp * 10, (this then includes 10th-of-a-degree)
    //get adc readings 800 nSec
    float ftemp;
    ftemp=(readTemperature()*100);              //floating value of pulse readings                       
    *tb++= (int)(ftemp);                        //includes 100th-of-a-degree)                        
    
    for (int i=0; i<16; i++)
        *tb++= adc_data[i].value;
                    
    //*tb++= readAFE_Slow(1, 0x103);        //read 16bit AFE reg trig cntrol reg        
    //*tb++= readAFE_Slow(1, 0x104);        //read 16bit AFE reg pipeline length                      
    //*tb++= readAFE_Slow(1, 0x105);        //read 16bit AFE reg sample length       

    *tb++= sTrigCntrl;                          //read 16bit AFE reg trig cntrol reg
    *tb++= sPipeLinDly;                         //pipeline delay (0==256) 12.56nS per cnt
    *tb= sSampleLen;                            //read 16bit AFE reg sample length       

    
    //loop reading on 1-4 fpga var fpga
    for (int fpga=1; fpga<=fcnt; fpga++)      
        {
        if (fpga==1)    fPtr= (uint16*)s0AFE_DACbias;
        else if(fpga==2)fPtr= (uint16*)s1AFE_DACbias;
        else if(fpga==3)fPtr= (uint16*)s2AFE_DACbias;
        else            fPtr= (uint16*)s3AFE_DACbias;
        //get dacs
        for (ch=0; ch<24; ch++)                 //read 24 dac regs (trim,led,bias,afe)
            statusBlock[fpga][ch]= *fPtr++;

        //statusBlock holds data, refreshed trigged by timer //on each call to 'STAB()' or 'CMB'
        statusBlock[fpga][ch++]= oneWireTemp[fpga-1][0];    //read stored ch 1 temp 
        statusBlock[fpga][ch++]= oneWireTemp[fpga-1][1];    //read stored ch 2 temp 
        statusBlock[fpga][ch++]= oneWireTemp[fpga-1][2];    //read stored ch 3 temp 
        statusBlock[fpga][ch++]= oneWireTemp[fpga-1][3];    //read stored ch 4 temp 
        
        //get afe data, note (10 reads@15uSec each)* 4fpgas
        statusBlock[fpga][ch++]= readAFE_Slow(fpga, 0x101);      //read 16bit AFE reg   'has fixed 10uS delay'
        statusBlock[fpga][ch++]= readAFE_Slow(fpga, 0x102);      //read 16bit AFE reg   'has fixed 10uS delay'
        statusBlock[fpga][ch++]= readAFE_Slow(fpga, 0x133);      //read 16bit AFE reg   'has fixed 10uS delay'
        statusBlock[fpga][ch++]= readAFE_Slow(fpga, 0x134);      //read 16bit AFE reg   'has fixed 10uS delay'
        statusBlock[fpga][ch++]= readAFE_Slow(fpga, 0x135);      //read 16bit AFE reg   'has fixed 10uS delay'
        statusBlock[fpga][ch++]= readAFE_Slow(fpga, 0x201);      //read 16bit AFE reg   'has fixed 10uS delay'
        statusBlock[fpga][ch++]= readAFE_Slow(fpga, 0x202);      //read 16bit AFE reg   'has fixed 10uS delay'
        statusBlock[fpga][ch++]= readAFE_Slow(fpga, 0x233);      //read 16bit AFE reg   'has fixed 10uS delay'
        statusBlock[fpga][ch++]= readAFE_Slow(fpga, 0x234);      //read 16bit AFE reg   'has fixed 10uS delay'
        statusBlock[fpga][ch++]= readAFE_Slow(fpga, 0x235);      //read 16bit AFE reg   'has fixed 10uS delay'      
        }
}


void USB_Rec_Init()
{
    USB_Rec.RdPtr=0;   //this code reset app, does not work with or without interrupts on/off
    USB_Rec.WrPtr=0;
}

							
							
//Keyboard wait for USB port entry with TIMEOUT
//Input wait time in seconds
int KeyBoardWait(int wait)
{
  wait *=1000;
  char inCh;
  mStime.g_wTicks=0;;
  if (wait==0)
    mStime.g_wTicks=0;
  while(1)
      {
      if(mStime.g_wTicks > wait)
        return 0;                       //timeout, return 'TRUE'
      else if (USB_Rec.Cnt)             //rec char count
        {
        inCh= getBufBin();          
        return inCh;                       //data avail, return 'TRUE'
        }
      }
}


//Waits (nn mSec) for received data from Socket, 
//Returns 1st Rec'd Char Cnt
//*key holds 1st recd char, valid if rtnCnt > 0
int SockKeyWait(int wait, int sock, int *key)
{
  int len, IPport;
  int *Ptr= (int *) &netInfo.sTelnet0;
  Ptr = Ptr+ (sock*2);
  IPport=  *Ptr;                              //read sockets port numb from structure
  
//wait *=1000;
  mStime.g_wTicks=0;
  while(1)
      {
      if(mStime.g_wTicks > wait)
        return 0;                           //timeout, return 'FALSE'
      else 
        {
        //socket, port, buffer, mode(Sn_MR for no delay)
        len= loopback_tcps(sock,IPport,(uint8*)eRecDatBuf[sock],0);  
        if (len)                            //rec char count
            {
            *key= eRecDatBuf[sock][0];
            //uTelnet downloads need echo else slow timeout response
            if (wait>200)
                putBuf(sock, "\0x08",1);            
            return len;                     //data avail, return 'TRUE'
            }
        }
      }
}



//Send data on LVDS FM PORT if word count non zero?
//Background handler to return data pool data
//Takes 16uS to xfer 64 words, see 'notifications.c()'
//Repeats as needed after a ~128uS delay
//
int lfdsFMsendBUF4096(char* sBuf, int lenWrds)
{
    HappyBus.Src= (snvPTR)BUF2048;          //done, init to src begin    
    HappyBus.SndWrds= lenWrds;              //snd word count
    *HappyBus.Src++= HappyBus.BrdNumb;      //send data on LVDS PORT
    *HappyBus.Src++= HappyBus.CmdType;      //send data on LVDS PORT
    *HappyBus.Src++= HappyBus.SndWrds;      //send data on LVDS PORT
    movStr16((snvPTR)sBuf, (snvPTR)HappyBus.Src, lenWrds);  //word to lwords
    HappyBus.Src= (snvPTR)BUF2048;          //reset to beg for backgrnd xmits 
    HappyBus.SndWrds +=3;
    //NOTE PROMPTS GETS SENT BACK ON LVDS ON EXIT HERE   
    genFlag |= NO_ECHO;                     //no ASCII reply 
    //send data on LVDS PORT via interrupts
    LVDS_Intr_Time(1);                      //Triggers interrupt xmit sequence         
    return 0;
}
  


#define PACs  2 
#pragma  pack(4)                                    //force 16bit boundary on all types
extern pbuf_t   pack[PACs]; 

#pragma pack(4)                                     //force 16bit boundary on all types
static  uint16   data[PACs][MAX_TRANSFER_UNIT];     //MAX_TRANSFER_UNIT= 1514

//fill EMAC Packet hdr buffer and xmitData array
// Pointer to the actual ethernet packet/packet fragment to be transmitted.
// The packet needs to be in the following format:
//   | Destination MAC Address (6 bytes)
//   | Source MAC Address (6 bytes)
//   | Length/Type (2 bytes)
//   | Data (46- 1500 bytes)
// The data can be split up over multiple pbufs which are linked as a linked list
// Note: Sending to mu2e non standard phy/fpga receiver logic (all data, no MACs or Len/Type)
void create_packet(int szBytCnt)                //byte count even and greater than 2
{     
    int i,j, pacCnt=PACs;     
    //error check
    if (szBytCnt < MAX_TRANSFER_UNIT)
      pacCnt=1;
    
    for(i=0; i<pacCnt; i++)     
        {     
        pack[i].tot_len= szBytCnt;              //len to of all buffers (bytes)
        pack[i].len= (szBytCnt/pacCnt);         //len to this buffer
        pack[i].payload = (uint8*)&data[i][0];  //load ptr to payload (==mac Addr) &TEXT1[0]
       
        //test data, split into two packets
        for(j=0; j< (szBytCnt/pacCnt); j++)
            data[i][j]=sTstCntLO;               //use fpga test counter
       
        //next emac header
        if(i!= (pacCnt-1))
            pack[i].next = &pack[i+1];          //ref for next packet, repeat loop
        }
    pack[pacCnt-1].next= NULL;                    //final emac packet marked as end
}


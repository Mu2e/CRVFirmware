
//********************************************************
// @file Mu2e_Cntrl.c 
//  Fermilab Terry Kiper 2016-2021
//  see revision list below
//
//  mu2e CRV Readout Controller (ROC) (also used by SBND)
//  RM48 Micro Controller ARM Cortex-R4F Floating-Point
//  RM48 CPU 220mhz
//  RM48 RAM 256KB
//  RM48 DATA FLASH 64KB
//  RM48 FLASH 3MB
//  RM48 Hercules device supports little-endian [LE] format
//
//  This function is called after file sys_main.c
//  Program Xilinx Spartan 6 fpga
//  Program and Read Spansion Inc S29JL064J 64 Megabit 
//                  (8M x 8-Bit/4M x 16-Bit) Flash Memory
//  Program and Read FRAM memory chip 'FM25CL64B'

//  The RM48L952 device integrates the ARM Cortex-R4F Floating-Point CPU which 
//  offers an efficient 1.66 DMIPS/MHz, and has configurations which can run 
//  up to 220 MHz, providing up to 365 DMIPS. The device supports the 
//  little-endian [LE] format. The RM48L952 device has 3MB of integrated flash and 
//  256KB of data RAM with single-bit error correction and double-bit error detection.

//********************************************************


/* Include Files */
#include "sys_common.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "nHetCap.h"
#include "reg_het.h"

#include "ver_io.h"
#include "ver_help.h"
#include "ver_ePHY.h"


#include "ZestETM1_SOCKET.h"
#include "ZestSock_TEK.h"       //Orange Tree ZestETMI sockets
#include "Mu2e_Ctrl_Misc.h"
#include "Mu2e_Ctrl.h"
#include "mu2e_Ctrl_i2c.h"      //mu2e i2c link functions
#include "ZestETM1_SPI.h"       //mu2e Orange Tree SPI Interface
#include "fram.h"               //u2e fram 

//Using V1 hardware
//tek 03-16-16: Ver101 Initial V1 code 
//tek 03-18-16: Ver102 Added in gigaBit Ethernet code 'ZestETMI_SOCKET.c'
//tek 05-11-16: Ver401 gigaBit Ethernet code 'ZestETMI_SOCKET.c' testing good
//tek 06-01-16: Ver402 Added LVDS link test code
//tek 07-21-16: Ver404 Added File download to fpga1 sdRam
//tek 07-21-16: Ver405 ZestETM1 logic changed added controls under cmd 'quit', commented out using intr via GIOA5 pi
//tek 08-12-16: Ver406 ZestETM1 FPGA Interface logic changed, now optional 8bit writes
//tek 08-12-16: Ver406 Fixed ZestETM1 sends Nulls on xmits of odd counts
//tek 08-12-16: Ver406 Note ZestETM1 has milli-second delay on updating 'IE_OUTGOING_EMPTY' status bit
//tek 08-15-16: Ver407 File loader cmd "RDF" load separate file to sdRam 2,3,4 (no sdRam on FPGA1), added cmd 'SOCK'
//tek 08-24-16: Ver408 Mods to OTree zestETM1 socket setup to recover from connection breaks
//tek 09-08-16: Ver409 Mods to OTree zestETM1 rec buf overrun, sio port Network setup, test functions see cmd'HT'
//tek 10-03-16: Ver410 Added PHY link test code
//tek 10-05-16: Ver411 POE power check functions mode to read 1 of 24 ch per call
//tek 10-11-16: Ver412 Updated i2c POE power monitor, func time 50uS every 10mSec
//tek 10-11-16: Ver412 Updated Network setup to handle OrangeTree Interface Board
//tek 10-11-16: Ver412 Code clean up in progress, added header files, rename some files
//tek 10-25-16: Ver413 Code clean up with more header files
//tek 12-20-16: Ver414 Modified cmds 'SET' to handle OrangeTree IP,GW,MASK changing
//tek 02-14-17: Ver415 nError code mod, (wait for new hardware ver to fully implement)

//tek 05-01-17: Ver416 increased OrangeTree chip sel 'WR' from 4 to 8 cnts becasue 8bit wr failed in Socket_SendFrame()
//tek 05-26-17: Ver417 mods to add phy data send/rec commands, cmd 'RDB' returns buffered FEB data from PHY
//tek 06-26-17: Ver418 rdb1, rdb2 mods, help menu mods
//tek 07-17-17: Ver419 added lvds fm port read functions, cmd 'PFM'
//tek 07-19-17: Ver420 added lvds fm port read functions, cmd 'HC'
//tek 07-25-17: Ver421 adding remote fpga file downloading via PHY Port
//tek 08-16-17: Ver422 added driver(ePHY) and receiver(FM-LVDS) code for controller--feb link
//tek 08-30-17: Ver430 good working ver of added driver(ePHY) and receiver(FM-LVDS) code 
//tek 09-26-17: Ver431 improved controller link to FEBs
//tek 10-05-17: Ver432 added ePHY link, sanity check, incremental link inits
//tek 11-15-17: Ver433 cleaned up i/o accesses to limit controllers from being busy more than 20uS
//tek 01-24-18: Ver433 added LVDS_SEQDATA_CHECK() to ready lvds ports for non controller req messages
//tek 01-24-18: Ver434 added mods for reading out febs at 'Proton Therapy RAD tesing on FEBs'
//tek 02-06-18: Ver435 removed some mods for reading out febs at 'Proton Therapy RAD tesing on FEBs'
//tek 02-14-18: Ver440 FPGA Reg Map ADDR change, new Addr for LVDS Rec Buf, stat, par, added Word Cnt
//tek 02-28-18: Ver441 optimized code in link check and data pooling.
//tek 03-07-18: Ver442 finished optimizing code in link check and data pooling, stable phy/lvds i/o
//tek 03-14-18: Ver443 added i2c data req interrupt mods, revisit rj45 frnt panel led blinking
//tek 03-14-18: Ver443 added PHY_LOADER_DAQ() to be used with Controller to FEB DAQ requests

//tek 03-20-18: Ver444 final checks ov ver443, ver445 will make pool data and LinkChk a global broadcasts
//tek 03-22-18: Ver445 pool data req global broadcasts, LinkChk now returns data a binary
//tek 03-22-18: Ver445 uBunch data req added a temp function for testing high speed reg
//tek 04-13-18: Ver446 uBunch fiber test continue
//tek 05-30-18: Ver447 uBunch data cntl-feb-cntrl initial testing stable
//tek 05-30-18: Ver447 uBunch data cntl-feb-cntrl initial testing stable
//tek 05-30-18: Ver447 Otree rec data function mods
//tek 06-05-18: Ver448 Otree 'SocketInit()' mods
//tek 06-08-18: Ver449 Pool data reqs mods 'lc_LSTAB()' and link_ID_Chk()
//tek 06-13-18: Ver450 uBun Testing Ok using CMDs 'UB0,''UB1', 'UB2' and PRECF
//tek 06-18-18: Ver451 added socket close option, socket connection counter as diag
//tek 07-15-18: Ver452 Otree boot delays up to 8 Sec on power up cycles for clean start
//tek 07-30-18: Ver453 DAQ return display using 'precf()'
//tek 08-08-18: Ver454 O'Tree initialing clean up from pwrUp to soft reset
//tek 09-14-18: Ver455 Added global addressing 0x301,0x302 for ePHY xmits
//tek 09-18-18: Ver456 Mods to UB1,2 testing, now sending 4 uBunch request per pack (was 10)
//tek 09-27-18: Ver457 Link Inits not required for cmds 'LC','LCA' with dedicated ports 1of24
//tek 10-02-18: Ver458 Mods to allow downloading test 'fake' data to FEB
//tek 10-17-18: Ver459 Link Init now only use FPGA 'Reg8,9' FM Status
//tek 10-19-18: Ver460 Pool update time now 120Sec, mods to 'fake data func'
//tek 10-26-18: Ver461 PLL Clock 'ADF4001' setup changed for a 100Khz Clock
//tek 03-25-19: Ver462 code clean up
//tek 08-23-19: Ver463 test rountines 'PSEND1' to send small non standard packets to FEB
//tek 12-03-19: Ver464 40Mhz Ref, 160Mhz feedback (using new vxo as of Sept2019)
//tek 12-11-19: Ver465 Added Data Pool test code, cmd POOLMODE
//tek 12-12-19: Ver466 lvds rec func mods using cmd 'LC' and HappyBusCheck()
//tek 12-18-19: Ver467 added testing 'TRIG' calls GTP1_Rec_Trigs()
//tek 01-15-20: Ver468 cmd 'UB2' cmd now sends uB Req as soon as 9word avail (same for cmd 'Trigger' Req mode)
//tek 02-07-20: Ver469 Pool data req every 30 sec, cleaned up PoolReq and ID timer code
//tek 02-10-20: Ver470 Fixed lvds link command 'LCA', renamed POOLMODE to POOLENA
//tek 02-27-20: Ver471 Mocs to lvds return data handler and data pool req logic
//tek 03-04-20: Ver472 code mods to lvds data receive port on handling 'LC' commands
//tek 03-06-20: Ver473 minor ubunch requesting code mods
//tek 03-10-20: Ver474 ub2 testing mods
//tek 03-12-20: Ver475 ub2 testing mods continued
//tek 03-25-20: Ver476 fix fpga downloader to feb cmd 'LDPGMFEB'
//tek 03-27-20  Ver477 mods to ubunch testing UB2-UB4 functions
//tek 03-31-20  Ver478 mods lvds command link routines
//tek 04-02-20  Ver479 mods to pooling routines
//tek 04-06-20  Ver480 mods to IOPs structure for pooling word count
//tek 04-07-20  Ver481 added direct flash loading using sockets 'FLSOCK1'
//tek 04-08-20  Ver482 uB testing routines modes
//tek 04-09-20  Ver483 trap nonascii returned data in 'LC' amd 'LCA'
//tek 04-10-20  Ver484 more mods 'LC' and 'LCA'
//tek 04-13-20  Ver485 lvds link data handler mods
//tek 04-15-20  Ver486 removed commands 'ST,STAB', use 'POOL'
//tek 04-22-20  Ver487 added ubReq 9word packet verify for out of order data, added cmd 'UB' for diag 
//tek 04-23-20  Ver488 PWRRST now does single port resets 
//tek 04-28-20  Ver489 mods to phy command headers
//tek 05-04-20  Ver490 mods ubunch testing for diagnostics 'uBunXmitBufLoad()'
//tek 05-05-20  Ver491 limit testing ub Req to 2 per packet, mods to 'PRECF,PREC,PFM'
//tek 05-15-20  Ver492 added help address map 
//tek 07-07-20  Ver493 wrrst for FEBs update and 'ZestETM1 O-Tree Power On/Off reset'





//tek 03-09-20  Ver493 'ZestETM1 Power Reset will not be supported on production boards, UNLESS Hardware Modified
//tek 03-25-21  Ver494 Updated comments and help page text
//tek 04-27-21  Ver495 added fifo clear before requesting FEB pooled data, uses lvds returns
//tek 05-18-21  Ver496 UB3 test mode retrigger fix.
//tek 11-04-21  Ver497 Modified command 'PRECALL' to select all POE ports
//tek 12-07-21  Ver498 Help menu cleanup cmd 'PRECALL' fix on socks 1,2
//tek 12-08-21  Ver499 Mods to 'TRIG' and 'TRIG_OLD' uB packet former
//tek 12-09-21  Ver500 Mods to 'GTP1_Rec_Trigs' PHY uB packets tranmitter 50uS backtoback xmit cap
//tek 11-21-22  Ver501 Added Testing mod Fake uB Req code since FPGA uBunch Sequencer not working, see 'GTP1_Rec_Trigs_Fake_uB_Request()'
//tek 11-21-22  Ver501 This mu2e code does not use new SBND ROC SPI code for Orange Tree Network interfacing (only needed if FPGA completely controls Otree)
//.................
//tek 12-09-22  Ver600 Modified code to support ROC Testing see 'GTP1_Rec_Trigs_Fake_uB_Request' 
//tek 02-16-23  Ver601 Modified command 'LCA' and 'LC' LVDS xmit/rec wait times, added ROC tester mode code under command 'TESTLVDS'
//tek 02-24-23  Ver602 Tester function 'TESTLVDS' and 'TESTPHY' now functional.
//tek 04-14-23  Ver603 CLKINITDATA Reg11 change from 8 to 9 to select IN2 buffer, new hardware layout
//tek 04-25-23  Ver604 FP LEDs on/off now by active FM Signal vs Current draw
//tek 05-04-23  Ver605 Fiber Test routine finished, GTP0 read of test data has extra data at times. GTP1 test data always good
//tek 08-18-23  Ver606 Fiber Test param fix
//tek 08-23-23  Ver607 Added Front Panel Leds to lvds testing, fixed bit ordering on FP Triple LED
//tek 09-05-23  Ver608 Added ChkSum to 'TESTPHY' function
//tek 09-06-23  Ver609 Updated 'TESTPHY' test
//tek 11-20-23  Ver610 Updated 'TESTLVDS' test
//tek 11-30-23  Ver611 Updated 'TESTLVDS' test
//tek 11-30-23  Ver612 Updated 'TESTLVDS' test
//tek 12-11-23  Ver613 Updated 'TESTLVDS' better display when getting random errors on all channels
//tek 07-15-24  Ver614 Added reg19 adf4001 enable with Reg19=0, back fans always on, use cmd 'FAN' to control after powerup
//tek 07-15-24  Ver615 Setting 20mhz LVDS bus clock on power, tester function 'TESTLVDS' uses 25Mhz then restores to 20Mhz
//tek 02-17-25  Ver615 InitFPGA_REGISTERS() 'WR 0xC00,0x29'
//tek 03-27-25  Ver616 Modified micro bunch testing commands UB3,UB4 
//tek 04-29-25  Ver617 Adding ROC to FEB FPGA file transfer to FEB FLASH
//tek 05-20-25  Ver618 New command 'FLDFEB' sends fpga binary file to FEB
//tek 05-27-25  Ver619 Cleanup on ROC to FEB downloads, see help 'HF' FLDFEB
//tek 06-24-25  Ver620 more Cleanup on ROC to FEB downloads
//tek 07-18-25  Ver621 mods for code downloads to feb all ports
//tek 08-14-25  Ver622 mods for code downloads to FEB pgms 8ch group for now
//tek 08-27-25  Ver623 mods for feb image code download
//tek 09-05-25  Ver624 mods for feb image code download again
//tek 09-09-25  Ver624 added ROC controller number to pool data req to FEB 'see PHY_LOADER_POOL_BCAST' file Mu2e_Cntrl_DAQ_PHY.c
//tek 09-12-25  Ver625 added FRAM vars for 3rd fpga image file storeage 'FEB File'
//tek 09-17-25  Ver626 clean up of fpga file downloads using Phy links in command 'febsend'
//tek 09-24-25  Ver627 UB2 test code enhanced 1or2 Request per packet and loop count option
//tek 01-22-26  Ver628 updated text messages related to FLSOCK3 binary file loading
//tek 01-23-26  Ver628 eraseFLASH_Sector() erase code cleanup, some mods functions 'fl1-fl3, flsock1-flsock3'

//Testing may at times disable i2c port when using jtag debugger see 'i2c_rec_intr_mode()'
//
//  Socket connections fail due to FNAL NETWORK PORT SCANS ON OTree Module
//  
//  done  TEK 09-2018, This is a OTree feature, contacted them, 'No Fix Available' 
//  done  TEK 09-2018, Should Enable Password on OTree Web Setup Page to 'u2e',
//                     prevents someone else from doing so and bricking the device
//  done  Use Netgear Switch GS108T to filter incoming packets to known good IPs


//RM48 Flash File Names Related to uC Hardware Specs 
//F05 is our 180nm process node, 
//F035 is our 130nm process node,
//F021 is our 65nm process node.

//*********************************************************************
//  Hercules? ARM® Safety MCUs - HALCoGen      
//  v4.05.02.00                
//  Build Date:  2016-03-02
//*********************************************************************
//---------------------------------------------------------------------
//New in This Version (4.05.02)
//---------------------------------------------------------------------
//Note1:
//----- 
//For all HALCoGen FreeRTOS based projects used with CCS, in the Compiler options under 
//Advanced Options -> Language Options -> ?Enable support for GCC Extension (--gcc)?.
//
//Note2:
//----- 
//For TMS570LC43x and RM57x Family of Devices Safety Functions are support only in 
//SafeTI Diagnostic Library version 2.1.0 which can be installed along with 
//HALCoGen 4.01.00 or latest. For using SafeTI Diagnostic Library with HALCoGen 
//please refer Examples --> example_SafetyLib.c in following Help file 
//C:\ti\Hercules\HALCoGen\v04.05.00\help\TMS570LC43x.chm (or) C:\ti\Hercules\HALCoGen\v04.05.00\help\RM57Lx.chm 

//POWER ON SELF TESTs
//The reality is, many (not to say all) customers are performing the CPU Self test 
//on power on only. From a safety standpoint this is enough. Doing this selftest 
//by interval at run time is really time consuming because of the context save/restore.
//These are the reason, to test on power on reset is the preferred solution. 
//The boot code will check if the reset condition is a power on reset 
//(in this case the CPU self test will be executed) or, if it is a warm reset,
//the test can be skipped. (PORST)


//define example for structure referenced into fixed memory location
//#define BUF   ((struct arp_hdr *) &uip_buf[0])
//#define IPBUF ((struct ethip_hdr *) &uip_buf[0])


// IAR STACK VIEWING
// To view the graphical stack bar:
// 1  Choose Tools>Options>Stack.
// 2  Select the option Enable graphical stack display and stack usage.
// You can open up to two Stack windows, each showing a different stack—if 
// several stacks are available—or the same stack with different display settings.


//---------------  after a 'HALCODEGEN' Must do Mods  --------------------------
//---------------  after a 'HALCODEGEN' Must do Mods  --------------------------
//after a 'HALCODEGEN' build the following stack fix may be needed 
//change stacks in sys_core.asm on each HAL-CODE-GEN --base- 0x08038000 for RM48L952

//new projects need this mode in *.icf file
//tek *** change linker file.icf from mem:__ICFEDIT_intvec_start__ { readonly section .intvec};
//tek *** change linker file.icf to   mem:__ICFEDIT_intvec_start__ { readonly section .intvecs};



//tek *** change project options linker from [.]Entry Symbol _main to below 
//Linker /Library   [.]Override default program entry
//					[.]Entry Symbol _c_int00


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
//

//---------------  after a 'HALCODEGEN' Must do Mods  --------------------------
//---------------  after a 'HALCODEGEN' Must do Mods  --------------------------
//in file 'i2c.c', change following structure 
//from 'static   struct g_i2cTransfer'
//to            'struct g_i2cTransfer'


//---------------  after a 'HALCODEGEN' Must do Mods  --------------------------
//---------------  after a 'HALCODEGEN' Must do Mods  --------------------------
//in file 'sci.c', change following structure 
//from 'static   struct g_sciTransfer'
//to            'struct g_sciTransfer'


//---------------  after a 'HALCODEGEN' Must do Mods  --------------------------
//---------------  after a 'HALCODEGEN' Must do Mods  --------------------------
/*
//this is needed in file 'sys_startup.c '
//after line add with '_coreInitStackPointer_()' add   _mpuInit_() + _mpuEnable_()
// Initialize Stack Pointers
    _coreInitStackPointer_();

//USER CODE BEGIN (7)
//TEK 09-2014, required if you want the EMIF port to actually work!!!
//The MPU setup was configured through Halcogen, the config function isn't actually 
//called by the generated code. Please place the following calls in sys_startup.c
    _mpuInit_();
    _mpuEnable_();
*/


//This pointer is used by the system driver to access the system frame 2 registers.
//#define systemREG2 ((systemBASE2_t *) 0xFFFFE100U)
//addr of DIEIDL_REG0= 0xFFFFE1F0


//---------------  after a 'HALCODEGEN' Must do Mods  --------------------------
//---------------  after a 'HALCODEGEN' Must do Mods  --------------------------
//change stacks in sys_core.asm on each HAL-CODE-GEN --base- 0x08038000 for RM48L952

//---------------  One time Linker File Must do Mods  --------------------------
//---------------  One time Linker File Must do Mods  --------------------------
//tek *** change linker file.icf from --place at address mem:__ICFEDIT_intvec_start__ { readonly section .intvec};
//tek *** change linker file.icf to   --place at address mem:__ICFEDIT_intvec_start__ { readonly section .intvecs};
// ****** use local copy of modified linker file 'RM48L952.icf'  *******

//The Stack Sizes in 'sys_core.asm'
//User Stack Base + Stack Size
//+Supervisor Stack Size
//+FIQ StackSize
//+IRQ stack Size
//+ Abort Stack Size
//+ Undefined StackBase
//total 0x3000 

//don't want this in file 'sys_core.asm'
//userSp  .word 0x08000000+0x00001700
//svcSp   .word 0x08000000+0x00001700+0x00000100
//fiqSp   .word 0x08000000+0x00001700+0x00000100+0x00000200
//irqSp   .word 0x08000000+0x00001700+0x00000100+0x00000200+0x00000600
//abortSp .word 0x08000000+0x00001700+0x00000100+0x00000200+0x00000600+0x00000100
//undefSp .word 0x08000000+0x00001700+0x00000100+0x00000200+0x00000600+0x00000100+0x00000100


//---------------  after a 'HALCODEGEN' Must do Mods  --------------------------
//---------------  after a 'HALCODEGEN' Must do Mods  --------------------------
//manually modified halcogen code as of july 2015  in file 'sys_core.asm' 
//fill down from top of RAM at 0x0803FFFF
/*
userSp  dcd 0x0803DE00+0x00001700
svcSp   dcd 0x0803DE00+0x00001700+0x00000100
fiqSp   dcd 0x0803DE00+0x00001700+0x00000100+0x00000200
irqSp   dcd 0x0803DE00+0x00001700+0x00000100+0x00000200+0x00000600
abortSp dcd 0x0803DE00+0x00001700+0x00000100+0x00000200+0x00000600+0x00000100
undefSp dcd 0x0803DE00+0x00001700+0x00000100+0x00000200+0x00000600+0x00000100+0x00000100
*/   
/*-Sizes
define symbol __ICFEDIT_size_cstack__   = 0x1700;
define symbol __ICFEDIT_size_svcstack__ = 0x100;
define symbol __ICFEDIT_size_irqstack__ = 0x600;
define symbol __ICFEDIT_size_fiqstack__ = 0x200;
define symbol __ICFEDIT_size_undstack__ = 0x100;
define symbol __ICFEDIT_size_abtstack__ = 0x100;
define symbol __ICFEDIT_size_heap__     = 0x4000;
End of ICF editor section. ###ICF###
*/


//nHet 
//nHET files, HalCoGen v4.03 now requires '_TMS470_BIG' for compile to work
//in file 'std_nhet.h' else missing def for ''HET_MEMORY' and more
//added HET_MEMORY in IAR project, compiler, preprocessor defined symbols


//nERROR is a push-pull output and is normally driven High. The Error Signaling Module (ESM) 
//drives this pin low to indicate any group2 and group3 error condition. Also, you can 
//configure the ESM to drive this pin low on any group1 error condition.
//When the nERROR pin gets driven low on an error condition the error flag must be cleared 
//and then 0x5 written to the ESMKEY register in order for the nERROR pin to go High after 
//the programmed duration has expired.
//nRST:   write 0x8000 to SYSECR register.
//nERROR: write 0xA to ESMEKR register.


//DMA SPEED INFO from TI Forums
//For each burst read (single 16 bit read can be considered as burst size of 1), 
//there is a 12 VCLK internal delay between burst. The same is true for reading 
//from the peripheral control registers. This 12 VCLK internal delay is also 
//true for writing if the peripheral/external memory space is configured as 
//strongly ordered by R4 MPU. This internal delay is reduced to 2 VCLK in writing
//when the memory is configured as device. When you calculate EMIF throughput, it 
//should be EMIF timing plus this internal delay.
//I would suggest using burst access to minimize the effect of internal delay. 
//The burst size of EMIF is 8x16 bits. You can consider using DMA for burst access.
// DMA SPEED FIX
// R4-MPU-PMU   
// REGION 5 0x60000000   STRONGLYORDERED_SHARABLE
// change to			 DEVICE_SHARABLE


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


//SDOCM00114735
//Safety Library fails to build with IAR compiler
//Release Note
 
//Safety Library fails to build with IAR compiler
//Version Info:
//IAR C/C++ Compiler for ARM
//  7.10.3.6832 (7.10.3.6832)
//Workaround 
//BEGIN COMMENT
//----------------------------
//For compiling the project in IAR, the following lines in the definition of SL_ESM_Init() under 
//sl_esm.c needs to be changed

//FROM
//      sl_vimRAM->ISR[1]  = &sl_esm_high_intr_handler;
//      sl_vimRAM->ISR[21] = &sl_esm_low_intr_handler;

//TO
//      sl_vimRAM->ISR[1]  = (sl_t_isrFuncPTR)&sl_esm_high_intr_handler;
//      sl_vimRAM->ISR[21] = (sl_t_isrFuncPTR)&sl_esm_low_intr_handler;
//----------------------------
//END COMMENT


//Network Mask Setup Info
//Class	Address	# of Hosts	Netmask (Binary)	Netmask (Decimal)
//B	    /16	65,534	11111111 11111111 00000000 00000000	255.255.0.0
//CIDR	/17	32,768	11111111 11111111 10000000 00000000	255.255.128.0
//CIDR	/18	16,384	11111111 11111111 11000000 00000000	255.255.192.0
//CIDR	/19	8,192	11111111 11111111 11100000 00000000	255.255.224.0
//CIDR	/20	4,096	11111111 11111111 11110000 00000000	255.255.240.0
//CIDR	/21	2,048	11111111 11111111 11111000 00000000	255.255.248.0
//CIDR	/22	1,024	11111111 11111111 11111100 00000000	255.255.252.0
//CIDR	/23	512	    11111111 11111111 11111110 00000000	255.255.254.0
//C	    /24	256	    11111111 11111111 11111111 00000000	255.255.255.0
//CIDR	/25	128	    11111111 11111111 11111111 10000000	255.255.255.128
//CIDR	/26	64	    11111111 11111111 11111111 11000000	255.255.255.192
//CIDR	/27	32	    11111111 11111111 11111111 11100000	255.255.255.224
//CIDR	/28	16	    11111111 11111111 11111111 11110000	255.255.255.240
//CIDR	/29	8	    11111111 11111111 11111111 11111000	255.255.255.248
//CIDR	/30	4	    11111111 11111111 11111111 11111100	255.255.255.252

//Applying a subnet mask to an IP address separates network address from host 
//address. The network bits are represented by the 1's in the mask, and the host 
//bits are represented by 0's. Performing a bitwise logical AND operation on 
//the IP address with the subnet mask produces the network address. For example, 
//applying the Class C subnet mask to our IP address 216.3.128.12 produces 
//the following network address:
//IP:   1101 1000 . 0000 0011 . 1000 0000 . 0000 1100  (216.003.128.012)
//Mask: 1111 1111 . 1111 1111 . 1111 1111 . 0000 0000  (255.255.255.000)
//      ---------------------------------------------
//      1101 1000 . 0000 0011 . 1000 0000 . 0000 0000  (216.003.128.000)

//Subnetting Network 
//Here is another scenario where subnetting is needed. Pretend that a web 
//host with a Class C network needs to divide the network so that parts of 
//the network can be leased to its customers. Let's assume that a host has 
//a network address of 216.3.128.0 (as shown in the example above). 
//Let's say that we're going to divide the network into 2 and dedicate the 
//first half to itself, and the other half to its customers.
//   216 .   3 . 128 . (0000 0000)  (1st half assigned to the web host)
//   216 .   3 . 128 . (1000 0000)  (2nd half assigned to the customers)
//---------------------------------------------------------------------------


extern char         cmdBuf20[];

extern const int    PoeAddr[];
extern uint16_t     POE_VLT_ALL[POECHsALL];
extern uint16_t     POE_CUR_ALL[POECHsALL];
extern int          resetEntry();       //file 'sys_intvecs.asm'
extern int          loopback_tcps(SOCKET s, uint16 port, uint8* buf, uint16 mode);
extern uint8        Op[];               //FRAM I/O Buffer 
extern volatile struct zSockets Sockets[];  //ZESTETM1 Board

//Remove 'static' definition from 'static struct g_sciTransfer' in func 'sci.c'
extern struct g_sciTransfer
{
    uint32   mode;          //Used to check for TX interrupt Enable
    uint32   tx_length;     //Transmit data length in number of Bytes
	uint32   rx_length;     //Receive data length in number of Bytes  
    uint8    * tx_data;     //Transmit data pointer  	
    uint8    * rx_data;     //Receive data pointer  
} g_sciTransfer_t[2U];


//i2c intr data buf ptrs
extern struct g_i2cTransfer
{
    uint32  mode;
    uint32  length;
    uint8   * data;
} g_i2cTransfer_t;

//ZestSock diagnostics structure
extern struct sockStat {
    unsigned int    badIP,
                    badIP_OVR,
                    conCnt;
    }soSt;        


u_16Bit     stab_Pool[24][eCMD72_MAX_WRDs174];  //data pool

g_dmaCTRL   g_dmaCTRLPKT;                       //dma control packet configuration stack
struct      blocksnd    BinSt;
struct      SetUpInfo_s Ser_Cntrl_Numb= {0,0};  //Init SetUp, serial# and DAQ_controller#

struct      SpillInfo_s Spill_Info= {200};      //Init SetUp

volatile    struct msTimers mStime; 
volatile    struct sLVDS    lvLnk;

struct      HappyBusReg HappyBus= {0,0,0,0, 0,0,1,0 ,0,0}; //Cmdlen, BrdNumb, Cmdtype, CntRec'd,  Active, ASCprt, Save, Err,  stat ...

struct ePHYregS ePHY_HDR= {
                    {0xD2D1, 0xD4D3, 0xD6D5},   //mac dest  ,One time init only used for diag pac viewing
                    {0xE2E1, 0xE4E3, 0xE6E5},   //mac src   ,One time init only used for diag pac viewing
                    0,0,0};                     //len,brd,cmd

struct      vB USB_Rec;
struct      ePHY_STRUCT ePHY_PORT;
struct      netinfo_s netInfo;          //Ethernet Network info struct
struct      uC_Store u_SRAM= {0,0};     //checksum and size info on command 'RDF' downloads to sdRAM


struct      uSums uPhySums;             //local storage of SOCKET download programming of FLASH
struct      uC_Store_FRAM u_FRAM= {0};  //nError FPGA status
struct      sLVDS_ePHY_REG IOPs[25];    //testing link assignment regs structure

//testing internal fpga logic uBunch xmits
struct ky k28;


//spi port control config
spiDAT1_t   dataconfig_ADC;
spiDAT1_t   dataconfig_PGA;
spiDAT1_t   dataconfig_FRAM;
spiDAT1_t   dataconfig_FRAM_HLD;    //hold 'cs' active low

spiDAT1_t   dataconfig_ZEST8;       //hold 'cs' active low 8 bit access
spiDAT1_t   dataconfig_ZEST16;      //hold 'cs' active low 16 bit access


#define FP_REGS_MAX     19     //(nn regs groups, plus one valid flag)
#define FP_REGS_VALID0 0xAFE0  //mask off lowest nibble
#define FP_REGS_VALID1 0xAFE1
#define FP_REGS_VALID2 0xAFE2
#define FP_REGS_VALID3 0xAFE3
#define rLOADED         00      //Number of actual register loade, Filled in by code

//fpga(4 ea) reg setup structure (defaults)
FPGA_RegS  f_RegConst [FP_REGS_MAX] = {  
//      {RegAdr,RegCnt,Data[0-8]}
        { 0x00, 0x02, FP_REGS_VALID0,rLOADED},//valid flag, cnt
        { 0x00, 0x01, 0x03},                  //RegAdr, RegCnt, Data[0-8]
        { 0x20, 0x01, 0x00},                  //RegAdr, RegCnt, Data[0-8]
        { 0x21, 0x01, 0x0F},
        { 0x22, 0x01, 0x00},
        { 0x23, 0x01, 0x00},
        { 0x6E, 0x01, 0x00},
        { 0x6F, 0x01, 0x00},
        { 0x70, 0x01, 0x00},
        { 0x71, 0x01, 0x00},
        { 0x74, 0x01, 0x00}, //todo, save space with strut for just d16 grps
        { 0x30, 0x08, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800},
        { 0x38, 0x08, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800},
        { 0x40, 0x04, 0x000, 0x000, 0x000, 0x000},
        { 0x44, 0x02, 0x000, 0x000},
        { 0x46, 0x02, 900, 900},
        {0x100, 0x01, 0x000},
        {0x101, 0x08, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
        {0x133, 0x02, 0x2004, 0x4140}
    };

//fpga(4ea) reg setup structure (flashed and restored)
FPGA_RegS  f_RegHold[4][FP_REGS_MAX];


//testing examples
uint8_t	HEADER_FF[8U]= {0x55U, 0x55U, 0x55U, 0x55U, 0x55U, 0x55U, 0x55U, 0xD5U};//MU2E CONTROLLER
uint8_t	ePHYAdd[12U] = { 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6,
                         0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6 }; //phy_afe_brd


//Buffers
//Buffers
#pragma pack(4)
uint16      lvdsBuf128[128];
#pragma pack(4)
char        phyBuf128[132];
uint16      phyBuf128i[128];
#pragma pack(4)
uint16      phyTest[16] ={0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80,
                          0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000, 0x8000 };
#pragma pack(4)
char        g_lineBuf[100];
char        tBuf[600];
char        Buf1500[1500];
char        cmdbuf[100];
char        ReplyBuf80[80];

//data pooling testing
int PoolMode=1;


//struct zSockets Sockets[];
uint8       eRecDatBuf[NUM_SOCKETS][TXRX_BytSiz1600]; //socket(s)  line buffer size
uint8       eCmdBuf[NUM_SOCKETS][eCmdBufBytSiz];

//fpga-uC-OrangTree DMA buffer
#define     DMA_DAQ_WSIZ   1024             //DMA Buffer Size 16bits
uint8       DMA_DAQ_BUF[(DMA_DAQ_WSIZ*2)+4];//DMA Buffer size

//DMA functions
int         DMA_FPGA_OUT(int xWords);
int         DMA_OTREE_IN(int xWords);

//DCS
struct FebDCSRply DCSrply = {0,0,0};

adcData_t   adc_data[16];                   //read buf size ***must always be 16***    
uint8_t     USB_inBuf[USB_inBufSz];

uint16_t    POE_PORTS_ACTIVE[POECHsALL+1];  //24 port plus 1 to hold 'act port cnt'

//misc vars
//misc vars
int         esm_Ch=0, esm_Cnt=0;
int         g_IntrCnt=0, adc_ms=0;
int         g_Sock=0, g_wait;
int         cmdIndex=0, d_nErr=0, d_nErrBoot=0;

uint32      volatile iFlag=0, genFlag=0, BootFlag=0, g_dat32;
uint16      g_i2cErr=0, g_dat16, global_pass=0, g_errorcnt, g_errNDR=0, g_PhyPrts;

char*       cbufptr= g_lineBuf;
char*       g_paramPtr;

div_t       divV;                       //struc for div() quot,rem  code ver ##
div_t       divR;                       //struc for div() quot,rem

unsigned int Ris2Fall, Fall2Ris;        //temperature stuff

struct uBuns uBReq;                     //uBunch related vars


//tester code inits
unsigned short revBits16(short int c);


//************* Ethernet Packet Preamble *****************
// The preamble of an Ethernet packet consists of a 56-bit (seven-byte) pattern of 
// alternating 1 and 0 bits, allowing devices on the network to easily synchronize 
// their receiver clocks, which is followed by the SFD to mark a new incoming frame. 
// For Ethernet variants transmitting single bits instead of larger symbols, the 
// on-the-wire bit pattern for the preamble together with the SFD portion of the 
// frame is 10101010 10101010 10101010 10101010 10101010 10101010 10101010 10101011;
// [4]:sections 4.2.5 and 3.2.2 since octets are transmitted least-significant 
// bit first, the corresponding hexadecimal representation 
// is 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0xD5.

//After an EMAC reset and before enabling the EMAC for send and receive, 
//all 16 head descriptor pointer registers must be initialized to 0.


#define MAX_TRANSFER_UNIT  200    //(sizeof(HelpMenu)/2
#define PAC_CNT     1 
#define DATA_CNT    MAX_TRANSFER_UNIT/PAC_CNT  //MAX_TRANSFER_UNIT_1514
#define DATA_LESS14 DATA_CNT-20


//FPGA (Base 0) write (addr,data16)
void wr16FPGA(uint16_t offset, uint16_t d16)
    {
    sPTR saddr = (sPTR) fpgaBase0;
    saddr += offset;                            //adding to pointer, incr by ptr size
    *saddr= d16;
    }


//FPGA  (Bsee 0) read(addr)
int rd16FPGA(uint16_t offset)
    {
    int ret16;
    sPTR saddr = (sPTR) fpgaBase0;
    saddr += offset;                            //adding to pointer, incr by ptr size
    ret16= *saddr;
    return(ret16);
    }


//Put the CPU in idle mode by executing the CPU idle instruction.
//asm(“ WFI”)
int IDLE()      //IDLE CPU
{
asm ("   WFI\n");
return 0;
}


//16bit moves, cnt is in word count
//Move 16bits per count, Incr Src Reg Only
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



void nError()
{
    FRAM_WR_ENABLE();                      
    //update nError counter
    //u_Store.nErrorCnt
	FRAM_RD(fram_NERROR_CNT, (uint8*)&u_FRAM.nErrorCnt, 2); //addr,data,cnt
    u_FRAM.nErrorCnt++;
    FRAM_WR(fram_NERROR_CNT, (uint8*)&u_FRAM.nErrorCnt, 2); //addr,data,cnt
    FRAM_WR_DISABLE();
    sprintf(tBuf,"nError extrn flip/flop bit= %d (0=Error)\r\n",d_nErr);
    putBuf(tty,tBuf,0);                 //send it        
}


void main_mu2e(void)
{
    divR = div(__VER__, 100);               //Compiler vers number is integer * 100
    divV = div(MU2Ever, 100);               //Code vers number is integer * 100
    u_16Bit * SocPort, otFlag;              //socket port number and status
    u_32Bit len, chCnt=0;
    u_8Bit i2cInit;
    
    //build pointer assignment regs structure (todo: can linker app do this)
    //this pointer structure can replace assignLinkPort()
    int FP= fpgaBase1;                          //1st rg45/fpga bank
    for (int rg=1, bit=0; rg<25; rg++)          //rj45 ports 1-24 as (8Chs per fpga)
        {
        IOPs[rg].FM40_STAp= FP+ (oFMStat40*2);  //lvds status
        IOPs[rg].FM41_PARp= FP+ (oFMPerr41*2);  //lvds parity error
        IOPs[rg].FM30_DATp= (uSHT*)FP+ ((oFM30_37_RXDAT+(bit)));//lvds data 0x30-37 pointer
        
        IOPs[rg].ePHY11_BCAST_FILLFIFOp=(uSHT*)FP+(oPHY11_BCAST_FILLFIFO); //ePHY Broadcast WR FIFO DATA
        IOPs[rg].ePHY0E_XMSKp=(uSHT*)FP+(oPHY0E_XMTMASK);//Broadcast Xmit Mask chans 0-7 enables
        IOPs[rg].ePHY12_XMITp=(uSHT*)FP+(oPHY12_XMIT);   //ePHY XMIT R/W (RD 1=empty), (WR 1 to xmit)
        IOPs[rg].ePHY16_STAp= (uSHT*)FP+(oPHY16_RXSTAT); //PHY 8-Port Rec data Status (bit7-0, 1=empty)

        IOPs[rg].ePHY18_RECCNTp= (uSHT*)FP+(oPHY18_1F_RXCNT+(bit));//ePHY rec word cnt 0x18
        
        IOPs[rg].ePHY20_RECBUFp= (uSHT*)FP+(oPHY20_27_RXDAT+(bit));//ePHY rec data fifo pointer 0x20
        IOPs[rg].ePHY_BIT= (1<<bit);                    //ePHY data avail status bit
        
        bit++;
        if (rg==8)  {FP+= (fOffset*2); bit=0;}      //2nd rg45/fpga bank            
        if (rg==16) {FP+= fOffset*2; bit=0;}        //3rd rg45/fpga bank
        }


    //may not be needed, power cycle may be needed to unhang i2c port when debugging intrs
    //may not be needed, power cycle may be needed to unhang i2c port when debugging intrs
    //may not be needed, power cycle may be needed to unhang i2c port when debugging intrs
    //set up for later, may get an initial interrupt
    g_i2cTransfer_t.data= &i2cInit;
    g_i2cTransfer_t.length=1;
    i2cEnableNotification(i2cREG1, I2C_RX_INT);
    i2cEnableNotification(i2cREG1, I2C_TX_INT);
    
    //main interrupt enable
    _enable_interrupt_();
    
    //Panel Leds Off for now, very bright
    LEDs_OFF    
        
    //esmTriggerErrorPinReset();
    //esmActivateNormalOperation();   //tek look at this
    //esmInit(); this is done by 'HalCodeGen' in sys_startup.c
    //esmEnableInterrupt(0);

    //The nError pin clears on a PORRST\. But with calling the esmInit() 
    //in the HalCoGen-Code, the nError pin will get set again.    
    //read external hardware flip/flop 'NC7SZ175' error latch bit
    d_nErr = hERR_Latch; //temp, for testing @ het1_4 pin
      
    //nError readout 'Hardware Mods'
    //Adding second power up timer chip 'STM6510' to prevent clocking the nError
    //latch chip 'NC7SZ175' during normal boot up tests.
    //Read external hardware flip/flop 'NC7SZ175' error latch bit
    //FEB mostly, comment out on controller until hardware modes done
    //if(d_nErr==0) 
        {
        //CLR_ERR, do a cycle as it clks a flip/flop,
        CLR_ERR_LO
        CLR_ERR_HI
        uDelay(1);
        //CLR_ERR, must keep low to allow nError to Reset Board
        CLR_ERR_LO
      //  nError();
        }
    
    //trigger now, cleans up registers
    //hDelayuS(1,1);                            //Hardware delay using RM48L nHET timer
    
    CSI_B0_HI
    CSI_B1_HI
    CSI_B2_HI
    CSI_B3_HI

    Suspend_LO                                  //Set suspend low, not used
    //reset fpga(s)
    PROGx_LO                                    //min 300nS
    uDelay(5);
    PROGx_HI                                
    uDelay(1000);                               //fpga reset takes about 800uS
    DSR_LO                                      //LOW enables data flow
      
    //set baudrate scilin, sysclock at 110Mhz
    //scilinREG->BRS = 14;    
    // baudrate 460800
    sciSetBaudrate(UART,460800);           //Set Baudrate Uart, have delay before 1st use of putBuf()
    sciSendByte(UART,0);                   // send char, may prevent "FLR status hangup on boot"
      
    //turn on otree network power
    oTreeOn
    
    //flash reset normal
    FlashRst_LO
    uDelay(10);                               
    FlashRst_HI
      
    //boot msg
    //sciSend(UART, mu_BOOT_MSGsz, mu_BOOT_MSG);
    putBuf(tty, (char*)mu_BOOT_MSG1, 0);

    if(d_nErrBoot)   //file 'sys_startup.c' ESM Group3 error check sets d_nErrBoot
        putBuf(tty, "ERROR ESM Group3, Group3 Only Drives the nERROR pin low\r\n\n", 0);
    
    //load fpga with individual calls with flash data
    for(int i=0,j; i<4; i++)
        {
        j=flashXFER(i, tty);
        if (j==-1)
            iFlag |= CONFIGFAIL;
        }
    mDelay(50);
    
    //ready flash chip
    flashStatus(tty);                      //show flash status
       
    //set fm lock to 20Mhz
    //modified SN14 for 'Yongyi at IERC room G291'
    //tek 7-16-24 now standard for all, 20Mhz
    sprintf(tBuf,"Reset ROC Transmit LVDS Clock\r\n");
    putBuf(tty, tBuf,0);                      
    wr16FPGA(0x000, 0x10);          //default is 25Mhz, set to normal 20Mhz

    //Flashed Startup Params for Network
    //copy flash data to Ram vars (network, baud, ser#)
    FRAM_RD(FRAM_AddrNet500, (uint8*)&netInfo, sizeof(netInfo)); //16bit writes (write 2 bytes)
    
    //now check if flash data is valid, if not load defs    **** Startup Params Network ****
    if ( netInfo.Valid != VALID)
        {
      //putBuf(tty, (char*)mu_BOOT_MSG2,0);                //Network Setup InValid
        memcpy(&netInfo, &defNetInfo,   sizeof(netInfo));  //load structure with defaults
        }

    //SPI5 setup, SPI5_CS0 'CS0' (H18)      //spiDAT1_t dataconfig_ZEST;
    dataconfig_ZEST16.CS_HOLD = FALSE;
    dataconfig_ZEST16.WDEL    = TRUE;
    dataconfig_ZEST16.DFSEL   = SPI_FMT_0;  //data format for 'spi5_cs0' 16bit
    dataconfig_ZEST16.CSNR    = SPI_CS_0; 
    
    //SPI5 setup, SPI5_CS0 'CS0' (H18)      //spiDAT1_t dataconfig_ZEST;
    dataconfig_ZEST8.CS_HOLD = FALSE;
    dataconfig_ZEST8.WDEL    = TRUE;
    dataconfig_ZEST8.DFSEL   = SPI_FMT_1;  //data format for 'spi5_cs0' 8bit
    dataconfig_ZEST8.CSNR    = SPI_CS_0;  
    
    //SPI3 setup, SPI3_CS0 'CS0' (V10)      //spiDAT1_t dataconfig_FRAM;
    dataconfig_FRAM.CS_HOLD = FALSE;        //keep 'cs' active low
    dataconfig_FRAM.WDEL    = TRUE;         //delay
    dataconfig_FRAM.DFSEL   = SPI_FMT_0;    //data format for 'spi3_cs0' 8bit
    dataconfig_FRAM.CSNR    = SPI_CS_0;     //ChipSelect (0-7 or )
    
    //start adc conversion,  1.206mV per bit
    adcStartConversion(adcREG1,adcGROUP1);
    /* ... wait and read the conversion count */
    //while((adcIsConversionComplete(adcREG1,adcGROUP1))==0);
    mDelay(1); //prevent lockup, just add delay
    adcGetData(adcREG1, adcGROUP1,&adc_data[0]);
    //id    = adc_data[0].id;  //example shows adc 
    //value = adc_data[0].value;

    //UART_SCI_LIN SetUp, use interrupt mode
    /* clear error flags */
    UART->FLR = ((uint32) SCI_FE_INT | (uint32) SCI_OE_INT | (uint32) SCI_PE_INT);
    g_sciTransfer_t[1].rx_length = 0;
    g_sciTransfer_t[1].rx_data   = USB_inBuf;

    
    //load structure from FRAM serNUMB  from FRAM
	FRAM_RD(fram_serNUMB, (uint8*)&Ser_Cntrl_Numb, Ser_Cntrl_ByteSz); //addr,data,cnt
    
    //Spill info stored here, adj as needed
	FRAM_RD(fram_Spill_REGs, (uint8*)&Spill_Info, SpillInfo_sByteSz); //addr,data,cnt
    mStime.SpillGateTimeout = Spill_Info.SpillGateTimeout; 
        
    /* Configure system response to error conditions signaled to the ESM group1 */
    /* This function can be configured from the ESM tab of HALCoGen */
    //esmInit(); //tek mod feb5,2016 to maybe stop ramdon nError reset on board
    
    //set POE RJ45 connector LEDs
    for(int i=0;i<15;i++)
        {
        if (i%2==1)
            RG45LEDS(0x5555);   //set rj45 leds
        else
            RG45LEDS(0xaaaa);   //set rj45 leds

        LED_BLU1;  //GRN
        mDelay(100);
        LED_BLU0
          
        LED_RED1;  //BLU
        mDelay(100);
        LED_RED0

        LED_GRN1;  //RED
        mDelay(100);
        LED_GRN0
        }
    RG45LEDS(0);                //set rj45 leds
    
    //Init CDCUN1208LPRHBR Clock Fanout Buffer via FPGA Configured SPI Port  
    InitFPGA_REGISTERS();       //setup PLL and clk driver 'ClkDrvInit()'
                                
    FAN1_HI;                    //turn 5v fan on as test
    FAN2_HI;                    //side fan, keep on all the time

    gioEnableNotification(gioPORTA, BIT5) ; //ETHERNET INTERRUPT ENABLE ETHERNET ZEST BRD 
    gioEnableNotification(gioPORTA, BIT4) ; //ETHERNET INTERRUPT ENABLE FPGA LOGIC
    uDelay(500);


    //ORANGE_TREE HTML PAGE, Browser Connect to IP:PORT
    //ORANGE_TREE Startup 'fpga logic requires an initial write to clear 'WR control signal'
    REG16(ZestETM1)= 0; 
    sprintf(tBuf,"\r\nZESTETM1 LNK: Orange-Tree Network Brd PwrUp Time Max 8 Sec\r\n");
    putBuf(tty,tBuf,0);                

    //FAN1_LO;                        //FAN test off
       
    //read sio port connection to otree to see when its ready.
    //test Zest Parallel port link via fpga, if fpga doesnt load, no Zest Ethernet
    //read addr Zest '0x214' WEB SERVERPORT normally 0x53 (80 dec) 
    //Data read on  bus and on sio port should match, if not
    uint16 sioDat, parDat, wlp=1;
    while(1)
        {
        sPTR zAdr = (sPTR) ZestETM1+ (0x214/2);     //get Phy Link Status
        sioDat= *zAdr;
        if((sioDat==0xffff) || (sioDat==0))
            sioDat= 0xf0f0;                         //dont allow 0 or ffff
        ZEST_RD(0x214, &parDat);                    //addr, data WEB SERVERPORT 'SPI PRT RD'
        if ((sioDat == parDat) || (iFlag & CONFIGFAIL))
            {
            break;
            }
        mDelay(500);
        if (wlp++>24)                       //Should require 16*500== 8 Seconds (max 24*500=12Sec)
            {
            //genFlag &= ~ZEST_ETM1_OK; 
            sprintf(Buf1500,"ZESTETM1 LNK: O'Tree Network Module Error, Check RJ45 Cable\r\n");
            sprintf(tBuf,"ZESTETM1 LNK: WebServerPortDualRead Par16_Port=%d(D), SIO_PORT=%d(D)\r\n", sioDat,parDat);
            strcat(Buf1500, tBuf);
            sprintf(tBuf,   "ZESTETM1 LNK: O'Tree will not fully init without network cable\r\n");
            strcat(Buf1500, tBuf);
            putBuf(tty,Buf1500,0);                
            break;
            }
        }
    
    //OTree MUST BE set up for NOT 'Auto-Open Settings via web page'
    //OTree Web Page Setup, 16bitReg , 2Bytes r/w, 'SPI slave',...
    //'TrigToUser', 'TrigTypeSingle', 'CLK DIR InputFromUser'
    //if nZESTETM1 acknowledged(wlp<16), enable sockets
    if (wlp<17) 
        {
        sprintf(tBuf,"ZESTETM1 LNK: Orange-Tree Network Brd PwrUp Time Actual %d Sec\r\n\n",wlp/2);  //time is .5Sec per Cnt
        putBuf(tty,tBuf,0); 
          
        sprintf(tBuf,"ZESTETM1 LNK: OK, O'Tree WebServerPortsRead Par16_Port=%d(D), SIO_PORT=%d(D)\r\n", sioDat,parDat);
        putBuf(tty,tBuf,0); 

        otFlag= REG16((ZestETM1+ 0x21E));   //read oTree status
        if ((otFlag & BIT4)==0)             //Is rg45 cable connected? 0=no
            {
            sprintf(tBuf,"ZESTETM1 LNK: ZESTETM1 LNK: O'Tree will not fully init without network cable\r\n");
            putBuf(tty,tBuf,0);                
            }
        //one more socket init if needed
        if (iFlag & OTREE_CONFIG)
            {
            SocketInit();  //OTree MUST BE set up for NOT 'Auto-Open Settings via web page'
            //sciUartSendText(UART,"SOCKET_ERROR: ReTry SocketInit Failed\r\n",39); 
            uDelay(500);
            }
        
        SocketInit();               //OTree MUST BE set up for NOT 'Auto-Open Settings via web page'       
        tcp_server_init(0,netInfo.sTelnet0, eRecDatBuf[0] );    //(sock,port) 
        tcp_server_init(1,netInfo.sTelnet1, eRecDatBuf[1] );    //(sock,port)                     
        tcp_server_init(2,netInfo.sTelnet2, eRecDatBuf[2] );    //(sock,port)                     
        tcp_server_init(3,netInfo.sTelnet3, eRecDatBuf[3] );    //(sock,port)                     
        uDelay(200);                //allow time for sockets to init
        connectStatus(0);
        connectStatus(1);
        connectStatus(2);
        connectStatus(3);
        //genFlag |= ZEST_ETM1_OK;
        Display_ZestETM1_NET(tty, tBuf);  //display ZestETM1 IP,GW,MASK registers
        }
    
    //dont stop network checking for now even if init failed
    genFlag |= ZEST_ETM1_OK;
    
    //enable DCS checks by default 
    genFlag |= Check_DCS;
        
    //putBuf(tty,"Compiler Optimizatiion must be set to 'NONE', else random nError resets\r\n\n",0);
    //LVDS FM Rec pError, BIT8 clear error flags
    //wr16FPGA(0x439, FMRstBit8);
    //wr16FPGA(0x839, FMRstBit8);
    //wr16FPGA(0xC39, FMRstBit8);
    
    //i/o link prt default else bus error
    HappyBus.SavePrt= 1;                    //default setup to port 1
    //assign default port
    assignLinkPort(1,1);
    
    //power up time outs before inits
    BinSt.gSndWordCnt= 0;
    
    //init default prompt to show 'LC' commands active port
    sprintf(USB_Rec.prompt,"\nP1>");
    lvLnk.IDPrt=1;
    putBuf(tty,"\n>",0); 
    
  //show message when i2c is disabled see 'i2c_rec_intr_mode()'
  //putBuf(tty,"I2C has been disable for 'Remote Login' testing/debugging with breakpoints\r\n>",0); 
    
    //Force CSR Reg0 to 20Mhz lvds fm clock, as testing code FPGA version used 26mhz clock
    wr16FPGA(0,0x10);               //Reg0 csr 20Mhz lvds fm clock

    //tek Sept2018, intial daq setup, may need to delete at some point
    REG16((fpgaBase0+ (0x27*2))) = 0x300;
    REG16((fpgaBase0+ (0x300*2)))= 0x8;
    REG16((fpgaBase0+ (0x300*2)))= 0xA8;
    
    //Link Init using stored FPGA link status of FEB Returned FM signals
    link_ID_Chk(tty);
    
    //Enable data pooling now
    lvLnk.PoolMode=1;                   //pool enable
    lvLnk.PoolChkmSec=ReqPoolTime30-5;  //pool update, normal 30Sed, 1st pass in 5 seconds
    
     
//******************************************************************************
//*******                  Main Loop Starts Here                       *********
//*******                  Main Loop Starts Here                       *********
//****************************************************************************** 
    
    //**************************************************************
    //******  Loop thru, checking usb, sock(s) for input    ********
    //**************************************************************        
    while(1)    
        {
        //**************************************************************
        //******       Check if char avail, UART (via USB)      ********
        //**************************************************************        
        if(USB_Rec.Cnt)
            {
            //process 1 input char
            cbufptr =  getline(cbufptr, cmdbufsiz, &cmdIndex);
            //ready UART/USB Receive
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
                if (iFlag & CONFIGFAIL)
                   sciUartSendText(UART,"FPGA_ERR>",9); //fpga err prompt
                else if (iFlag & OTREE_CONFIG)
                   sciUartSendText(UART,"Socket?>",11); 
                }
            }
        
        
        //**************************************************************
        //******                                                ********
        //******      DAQ uBunch data Request                   ********
        //******        1 extrn Trig  Mode 64 byte min packet   ********
        //******        1 extrn Trig1 Modes 8 byte min packet   ********
        //******        1 Software Req Mode, legacy testing     ********
        //******                                                ********
        //**************************************************************               

        //todo tek, 03-25-20 
        //Use NEW ver that sends short 3 Word Packets to FEB
        if(uBReq.Flag & DAQuB_Trig_NEW)         //daq uBunch decode active use short packet size reqs(min 6 bytes packet)
            {
            GTP1_Rec_Trigs();                   //check GTP Receive FIFO for uBun Requests
            }
        else if(uBReq.Flag & DAQuB_Trig_OLD)    //daq uBunch decode active, use standard pack size req (min 64+ bytes packet)
            {
            GTP1_Rec_Trigs();                   //check GTP Receive FIFO for uBun Requests
            }
        else if (uBReq.Flag & DAQuB_Test)       //test mode setup by command 'UB2'
            {
            GTP1_Rec_Trigs_Fake_uB_Request();   //fake ub req since FPGA uB Sequencer may not be working
            }

        
        //**************************************************************
        //******     Process DCS packages, takes ? uS           ********
        //************************************************************** 
        //
        if (genFlag & Check_DCS)
            {
            CheckAndProcessDCS();
            }
             

        //**************************************************************
        //******     DATA POOL UPDATE REQ, takes 15-40uS        ********
        //************************************************************** 
        //
        if (genFlag & PoolReqNow)   //PoolReqTimOt flag gets data   
            {
            PoolDataReq(tty);       //link data pool request
            }                    

        //note: Commands are sent to feb using command "LC" on ePHY link
        //only "LC" requests data from FEB, data returns here on FM-LVDS link 
        //Avg 2.0uS in Null per main loop pass to get back here
        //LVDSBusCheck() takes 200nS(no data), 2.5uS(if data,gets 1 word) 
        //**************************************************************
        //**  LVDS FM Data Available Check, FPGA FIFO Siz 512 WORDS   **
        //**************************************************************        
        //wait if data pooling busy 'PoolReqNow'
        else 
            HappyBusCheck();            //check for any lvds returned data
        
        //**************************************************************
        //******    Systick countdown, read POE ports power     ********
        //******     Sets leds, checks one ch every 10mSec      ********
        //******     takes 50uS                                 ********
        //**************************************************************    
        //
//may temporary disable while remote login testing
//any debug breakpoints tend to hang up i2c uC hardware block, needs pwr cycle        
        if (genFlag & POE_CHECK_DUE)       
            i2c_rec_intr_mode(); 
        
             
        //**************************************************************
        //****  Checks FPGA FEB Returned Clk, takes 2uS          *******
        //****  If returned Clk is valid assum FEB is active     *******
        //****  Requires lvLnk.LSTAB_Active==1                   *******        
        //**************************************************************                
        //
        if (genFlag & ID_ReqNow)        
            {
            link_ID_Chk(0);             //active boards give 1 wrd reply
            }
                               
        //**************************************************************
        //******       ZEST_ETM1 Check, contine if Okay         ********
        //******       Is network cable connedted               ********
        //**************************************************************        
        //if(!(genFlag & ZEST_ETM1_OK))
        otFlag= REG16((ZestETM1+ 0x21E));   //read oTree status
        if ((otFlag & BIT4)==0)             //Is rg45 cable connected?
            continue;                       //no cable, skip network stuff
       
        //**************************************************************
        //******         Check DAQ Block Data Move              ********
        //**************************************************************               
        //1st check for any active 'DAQ Block Data Move'
        if ((BinSt.gSndPrt==g_Sock) && (BinSt.gSndWordCnt!=0))
            sendBin(BinSt.gSndPrt);     //socket send until gBusy==0                
        
        //**************************************************************
        //******         Network Interrupt Check                ********
        //******    10 Sec check in case missed by Interrupt    ********
        //**************************************************************               
        if (genFlag & ZEST_ETM1_INTR)   
            {
            _disable_interrupt_();           //disable uC intr that calls 'SocketISRTEK()'
            SocketISRTEK(0);
            _enable_interrupt_();
            genFlag &= ~ZEST_ETM1_INTR;      //clr flag
            }
                
        
        //**************************************************************
        //******        Check sockets(4) for data               ********
        //**************************************************************               
        //cycle thru socket(s) checking for new rec'd data        
        SocPort= &netInfo.sTelnet0 + g_Sock; 
        len= loopback_tcps(g_Sock, *SocPort, eRecDatBuf[g_Sock],0 );                    
        
        //**************************************************************
        //******             telnet handler here                ********
        //**************************************************************               
        //if data avail, parse looking for cmd terminated by '\n' char
        if ( len )                                      //char based tty telnet port
            {                                           //each packet may contain only on char.
            char ch;
            uint8 *locPtr;
            unsigned int t=0;
            
            for (t=0; t<len; t++)
                {
                //tcp_server_rec_cnt_decrement(g_Sock);
                Sockets[g_Sock].RxFrameLenLast--;
                
                locPtr= eCmdBuf[g_Sock]+chCnt;          //dest buffer to build local cmd line a char at a time
                ch = myupper(eRecDatBuf[g_Sock][t]);    //upper case each char, data from sockets rec buffer
                *locPtr = ch;                           //store in socket(s) local Cmd Buffer
                if(ch == BACKSP)                        //backspace clean up
                   {
                   if (chCnt>0)                         //more than one char in buffer needed for backspace
                       {
                       *(--locPtr)= 0;                  //remove last char from buffer
                       chCnt--;
                       }
                   }
                else if(ch < ' ')
                   {
                    //skip newline char, assume it came along with a '\r'
                    if (ch != '\n')
                        {
                        *locPtr = 0;                    //mark end of buffer with null
                        process(g_Sock,(char*)eCmdBuf[g_Sock]); //respond to ascii cmd
                        //added tek aug2019 clear flag in case its set after reboot
                        iFlag &= ~OTREE_CONFIG;         //good data, assume connection ok now
                        chCnt=0;                        //init for next cmd line
                        }
                   }
                else
                    chCnt++;

                if ( chCnt >=eCmdBufBytSiz )
                     *(--locPtr) = '\r';                //end of buffer cleanUp
                }
            }
        //**************************************************************
        //******     Increment Socket Channel, Repeat loop      ********
        //**************************************************************                 
        if (++g_Sock > Sock3)                          //scan telnet sock0,sock1...
                  g_Sock=0;
              
        } //end main while loop
}

//******************************************************************************
//*******                  Main Loop Ends Here                         *********
//*******                  Main Loop Ends Here                         *********
//******************************************************************************        




//xMit USB (UART) data string
void sciUartSendText(sciBASE_t *sci, uint8 *text,uint32 length)
{
    //unsigned timer=0;
    while(length)
        {
        //warning, code has no breakout
        while ((UART->FLR & 0x4) == 4)        //wait until non busy
        {
         //if (timer++> 100000)
         //    {
              sciSendByte(UART,0);             // send NUll char, may prevent "FLR status hangup on boot"
              break;        //requires power cycle, give up
         //    }
        }        
        sciSendByte(UART,*text++);              /* send out text   */
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
        
        inCh = toupper(inCh);               //upper it
        *cptr= inCh;
        if (inCh == BACKSP)
            {
            if ( cptr > g_lineBuf )         //stop at beginning
                { putchar('\b'); putchar(' '); putchar('\b'); cptr--;}
            break;
            }
        if (inCh == '\r')                   //EOL
            { putchar(inCh);  break;}
        if (*cmdIndex >= (cmdbufLen-2))     //Sizeof(cmdbuf)) plus append Null
            { *cptr= '\r';    break;}
        *cmdIndex+= 1;                      //incr char counter
        cptr++;
        putchar(inCh);                      //echo rec'd character
        break;
        }
    return cptr ;                           //return byte count
}



#include <yfuns.h>      //I/O functions decl

//This function used by putch and printf
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



//Output char string to active i/o stream, USB, Socket(s)
//Send to Active socket or USB
//send  socket(ch,buffer,length,flags'oddcnt==send final as char')
void putBuf(int prt, char* sBuf,  int len)
{
    int g_TxCountOdd=0;
    char * p = sBuf;
    if (len==0)  
        {while (*p++) len++;}               //size of Null term data
    if(len==0)
        {
       len=0;                               //diagnotics, this should not happen
       return;
        }
    if(len&0x01==1)
        {
        g_TxCountOdd=1;                     //bytes will be sent as words, flag odd count
        }
    else
        g_TxCountOdd=0;
    
    //** Note On OrangeTree 16bit bus interface when sending odd number 
    //** of chars will xmit Null char as odd cnt.
    if (prt==Sock0)                                     //sock0 telnet user mode
        send(0, (const char*) sBuf, ((len)/2), g_TxCountOdd);//sock, src, len, flags ignored
    else if (prt==Sock1)                                //sock0 telnet user mode
        send(1, (const char*) sBuf, ((len)/2), g_TxCountOdd);//sock, src, len, flags ignored
    else if (prt==Sock2)                                //sock0 telnet user mode
        send(2, (const char*) sBuf, ((len)/2), g_TxCountOdd);//sock, src, len, flags ignored
    else if (prt==Sock3)                                //sock0 telnet user mode
        send(3, (const char*) sBuf, ((len)/2), g_TxCountOdd);//sock, src, len, flags ignored
    else if (prt==tty)
        sciUartSendText(UART,(uint8*)sBuf,len);         //send ascii 'tty'
    else if (prt==DCS)
        {
        REG16(fpgaBase0+(0x53*2)) = DCSrply.add;
        REG16(fpgaBase0+(0x53*2)) = (int)strtol(sBuf,NULL, 16L);
        }
  //TODO, setup code to have UART buffer xmits via interrupts
    return;
}


//------------------------------------------------------------------------------
//Read NULL terminted ascii string, Return 1st token begPtr and endPtr, else NULL
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
        if (isdigit(*tokPtr++)==0) 
          return defval;       //check for bad data type


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
//while( !(USARTx->SR & USART_FLAG_RXNE));      //polling rec not empty
//while( !(USART2->SR & USART_FLAG_TXE));       //polling xmit empty
//load uart transmit buffer with one character
//1st wait for active xmits to end
int putchar(int  c)
{
    sciUartSendText(UART,(uint8*)&c,1);         //send 1 byte
    return (c);
}


//#define uBunMax         10
//#define uBunIDsMax      8*uBunMax*2
//extern uSHT uBunJumboPac[];           //uBun Req Packet Concentrator
//extern uSHT uBunIDs[];                //store to compare to rtn data

#define CmdSiz20         20                     //limit repeat command line size
//static  char  tBufHold[CmdSiz20+1];             //store command for quick repeat option

//Check if rec'd command is valid and handle it.
int process(int prt, char *cmdPtr)
{
    sPTR p_sAddr;
    cPTR p_cAddr;
    char *tok = 0, *paramPtr= 0;
    int param1, param2, param3;
    unsigned int parErr=0;
    paramPtr = cmdPtr;                                  //ptr to data buffer
    
    //Save input string for possible repeat command
    //if (*cmdPtr)
    //    {
    //      if ((*cmdPtr!=']') && (*cmdPtr!='\n'))
    //        memcpy(tBufHold, cmdPtr, CmdSiz20);         //store cmd for repeat option (20 char limit)
    //    else
    //        memcpy(cmdPtr, tBufHold, CmdSiz20);         //limit command line size
    //    }    
    
    tok = mytok(paramPtr, &paramPtr);                   //find 1st token, should be acsii 'CMD'
    if (!tok)
        {
         newLinePrompt(prt);                            //Send newLine prompt with or without Port#
         return 1;
        }
    if (prt==tty)
        putBuf(prt,"\r\n",2);                           //start new line after recd command

    switch (*tok)                                       //command parameter search
        {           //strcmp, If all elements are equal, the function returns zero
        case 'A':
/*                
                if (!strcmp(tok, "AAA"))  //test code here, delete later
                    {
                    param1= arg_dec(&paramPtr,0);       
                    //POE_Volts_Check_One();                    
                    hetSIGNAL_t setup_pwm;
                    
                    //typedef struct hetSignal
                    //{
                    //	uint32 duty;        //< Duty cycle in % of the period  
                    //	float64   period;   ///< Period in us                 
                    //} hetSIGNAL_t;
                    
                    setup_pwm.period = 1e6/param1; //2 khz 
                    float Dutcycle = setup_pwm.period*0.075;
                    float Porcentduty= 0.075*100;
                    
                    pwmSetSignal(hetRAM1, pwm0, setup_pwm);                 // sets the PWM period
                    pwmSetDuty_full(hetRAM1, pwm0, Porcentduty, Dutcycle);  // set the pulse width
                    pwmStart(hetRAM1, pwm0);    
                    break;
                    }
*/    
                if (!strcmp(tok, "ADC"))
                   {
                    //conversion results :                                      
                    //adc_data[0] -> should have conversions for Group1 channel1
                    //adc_data[1] -> should have conversions for Group1 channel2
                    static float flt;
                    int i;
                    param2= arg_dec(&paramPtr,0);      //now get 1st param, 1==send data only, no text
                    
                    //wait and read the conversion count, should always be ready
                    if (adcIsConversionComplete(adcREG1,adcGROUP1))
                        adcGetData(adcREG1, adcGROUP1,&adc_data[0]);  //get last update data, requeted in 'notifications.c'
                    
                    *Buf1500=0;
                    for (i=0; i<ADC_CHS; i++)
                        {
                        flt= adc_data[i].value * adcScale[i];
                        if (param2==0)
                          //sprintf(tBuf,"%s %5.2f   Ch%1d_ADC=0x%03X  \r\n", adcName[i], flt, i, adc_data[i].value);
                            sprintf(tBuf,"%s %5.2f\r\n", adcName[i], flt);
                        else
                            sprintf(tBuf,"%5.2f ",flt);
                        strcat(Buf1500, tBuf);
                        }
                    //show temperature
                    flt= readTemperature();             //Temperature (°C) = 421 - (751 × (TH/TL))
                    if (param2==0)
                        sprintf(tBuf,"%s %5.2f\r\n", adcName[i], flt);
                    else
                        sprintf(tBuf,"%5.2f\r\n",flt);
                        strcat(Buf1500, tBuf);
                    
                    putBuf(prt, Buf1500,0);
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'D':          
                if (!strcmp(tok, "DSAV"))                   //read/store fpga regs to FRAM
                    {
                    uint16 framAddr;
                    uint16 cnt=0, d16, rcounter; 
                    u_32Bit addr32;
                    u_32Bit FPGA_BASE= fpgaBase0;
                    for(int chip=0; chip<4; chip++)
                        {
                        framAddr= fram_FPGA0_REG +4 +(0x100*chip);
                        rcounter=0;
                        for(cnt=1; cnt<FP_REGS_MAX; cnt++)
                            { 
                            addr32 = (u_32Bit)FPGA_BASE + (f_RegConst[cnt].Addr<<1);  //actual 32 bit address
                            for (int i=0; i< f_RegConst[cnt].RegCnt; i++, addr32+=2)
                                {
                                if (f_RegConst[cnt].Addr < 0x100)
                                    d16= *(sPTR)addr32;
                                f_RegHold[chip][cnt].Value[i]= d16;
                                //framAddr, fpgaAddr, write Cnt
                                FRAM_WR_BUF16(framAddr, &d16, 1); 
                                framAddr += 2;
                                rcounter++;
                                }
                            }
                        FPGA_BASE += 0x800;
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

                    FRAM_WR_BUF16(fram_FPGA0_REG, &f_RegHold[0][0].Value[0], 2);       //framAddr, fpgaAddr, write Cnt
                    FRAM_WR_BUF16(fram_FPGA1_REG, &f_RegHold[1][0].Value[0], 2);       //framAddr, fpgaAddr, write Cnt
                    FRAM_WR_BUF16(fram_FPGA2_REG, &f_RegHold[2][0].Value[0], 2);       //framAddr, fpgaAddr, write Cnt
                    FRAM_WR_BUF16(fram_FPGA3_REG, &f_RegHold[3][0].Value[0], 2);       //framAddr, fpgaAddr, write Cnt
                    putBuf(prt,"FRAM UPDATED\r\n",0);       //send to current active port
                    break;
                    }
                else if (!strcmp(tok, "DREC"))              //FLASH- data restore to registers
                    {
                    //recall from FRAM
                    param1= arg_dec(&paramPtr,1);           //now get 1st param, 0=load constant data
                    dataRecall(param1, fpgaBase0);
                    dataRecall(param1, fpgaBase1);
                    dataRecall(param1, fpgaBase2);
                    dataRecall(param1, fpgaBase3);
                    InitFPGA_REGISTERS();
                    putBuf(prt,"FPGA Regs Updated,  Broadcast Range 0x300 Up restored using fpga1 mem only\r\n",0); //send to current active port
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'C':
                if (!strcmp(tok, "CLOSE"))                  //close ethernet socket
                   {
                    param2= arg_dec(&paramPtr, g_Sock);     //get 1st param, flag for def value
                    if ((param2>=0) && (param2<4))          //valid sockets 0-3
                        {
                        //socket_close(param2);             //close connection
                        REG16(ZestETM1+(param2*CHANNEL_SPACING)+zIntrEna) = IE_STATE;   //state change intr enable'
                        REG16(ZestETM1+(param2*CHANNEL_SPACING)+zConnectCSR) = CONN_TCP;//Ch0 only,'Reset ch, Flush Bufs'
                          
                        //sockets reset and enable intrs
                        Sockets[param2].IE = 0;
                        Sockets[param2].State = 0;
                        REG16(ZestETM1+(param2*CHANNEL_SPACING)+zIntrEna) = Sockets[param2].IE;
                        REG16(ZestETM1+(param2*CHANNEL_SPACING)+zConnectCSR) = Sockets[param2].State;

                        if (Sockets[param2].Closing)
                          Sockets[param2].Used = 0;
                        mDelay(1);
                        u_16Bit *uPtr= &netInfo.sTelnet0 + param2; 
                        tcp_server_init(param2, *uPtr, eRecDatBuf[param2] );  //(sock,port)                     
                        }
                     break;
                    }
                else if ( !strcmp(tok, "CLR") || !strcmp(tok, "CLS"))  //Clear Screen
                    {
                    //char ClrScr[]= {0x1B,0x5B,0x32,0x4A,0};     //escape sequence for clear screen 
                    //putBuf(prt,ClrScr,0);                       //clear the the screen and scroll data both. 
                    for(int i=0;i<10;i++)
                        putBuf(prt,"\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n",0);//15 lines                 
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'W':
                if (!strcmp(tok, "WR"))                     //16bit EPI BUSS write
                    {
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//now get 1st param (addr)
                        { parErr++;  break; }
                    //address only, no board number
                    param1 &= 0x7FFFF;

                    if ( (param2=arg_hex(&paramPtr,-1))==-1)//now get 2nd param (data)
                        { parErr++;  break; }

                    sPTR saddr = (sPTR) fpgaBase0;
                    saddr += param1;                        //adding to pointer, incr by ptr size
                    *saddr= param2;
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'F':
          /*
                if (!strcmp(tok, "FD"))             //FPGA Load, direct from USB
                   {
                    if (prt!=tty)
                      break;
                  //param1=arg_dec(&paramPtr,1);    //now get 1st param U39B
                  //sprintf(tBuf,"FPGA Direct Load from USB, chip 1of4 =%d\r\n", param1);
                    //needs checking out, may not work 
                    loadSpartan6_FPGA(param1);
                    break;
                    }
          */
                if (!strcmp(tok, "FERASE"))    //FLASH erase all
                   {
                    sprintf(tBuf,"TOTAL FLASH Erase, Enter cmd as 'FEA 1'\r\n");
                    putBuf(prt, tBuf,0);
                    param1=arg_dec(&paramPtr,0);    //now get 1st param U39B
                    if(param1!= 1)
                     break;
                    eraseFLASH();                
                    break;
                    }
                else if (!strcmp(tok, "FL1"))       //FLASH Load of FPGA File via USB
                   {
                    if (prt!=tty)
                      break;                        //tty only
                    sprintf(tBuf,"S29JL064J: FPGA1 Flash Loader\r\n");
                    putBuf(prt, tBuf,0);
                    PROGx_LO                    
                    uDelay(5);                      //min 300nS
                    PROGx_HI                    
                    uDelay(100);                    //fpga reset takes about 800uS                    
                    flashStatus(prt);               //displays status
                    uDelay(100);  
                    
                    eraseFLASH_Sector(SectorCnt36, 0,prt);
                    sprintf(tBuf,"S29JL064J: Total sector erased=%d (%d Bytes)\r\n", SectorCnt36, ((SectorCnt36-7)*0x10000)); //Sec0-7(0x1000), then SecSiz(0x10000))
                    putBuf(prt, tBuf,0);
                    sprintf(tBuf,"S29JL064J: Select Binary file now (enable DSR/DTR)\r\n");
                    putBuf(prt, tBuf,0);
                    loadFLASH(S29JL064J_SECTOR0, prt);  //Actual S29JL064J address= 0x0 @Sector 0
                    sprintf(tBuf,"FL1_Load : Finished, reset board for complete update\r\n");
                    putBuf(prt, tBuf,0);
                    break;
                    }
                else if (!strcmp(tok, "FL2"))       //FLASH Load of FPGA File via USB
                   {
                    if (prt!=tty)
                      break;                        //tty only
                    sprintf(tBuf,"S29JL064J: FPGA234 Flash Loader\r\n");
                    putBuf(prt, tBuf,0);
                    PROGx_LO                    
                    uDelay(5);                      //min 300nS
                    PROGx_HI                    
                    uDelay(100);                    //fpga reset takes about 800uS                    
                    flashStatus(prt);               //displays status
                    uDelay(100);                   
                    
                    eraseFLASH_Sector(SectorCnt36, S29JL064J_SECTOR41, prt); //Actual S29JL064J address= 0x110000 @Sector 41                    
                    sprintf(tBuf,"S29JL064J: Total sector erased=%d (%d Bytes)\r\n", SectorCnt36, (SectorCnt36*0x10000) );
                    putBuf(prt, tBuf,0);
                    sprintf(tBuf,"S29JL064J: Select Binary file now (enable DSR/DTR)\r\n");
                    putBuf(prt, tBuf,0);
                    loadFLASH(S29JL064J_SECTOR41, prt); //Actual S29JL064J address= 0x110000 @Sector 41 'RFI 220000'
                    sprintf(tBuf,"FL2_Load : Finished, reset board for complete update\r\n");
                    putBuf(prt, tBuf,0);
                    break;
                    }                
                else if (!strcmp(tok, "FL3"))        //FLASH Load of FPGA File via USB
                   {
                    if (prt!=tty)
                        {
                        putBuf(prt,"Use USB port to load FLASH\r\n",0);
                        break;                        //tty only
                        }
                    sprintf(tBuf,"S29JL064J: FEB Image Flash Loader\r\n");
                    putBuf(prt, tBuf,0);
                    flashStatus(prt);                   //displays status
                    uDelay(100);                     
                    
                    //SectAddr71 Actual S29JL064J Word ADR=0x200000 (FEB BackUp image storeage) 
                    //"RFI 400000 to display"
                    eraseFLASH_Sector71(FileSectCntH, SectAddr71, prt);  //SectorsToErase, SectorStartAddr, DisplayPrt
                    sprintf(tBuf,"S29JL064J: Total sector erased=%d (%d Bytes)\r\n", FileSectCntH, (FileSectCntH*0x10000) );
                    putBuf(prt, tBuf,0);
                    
                    //FLASH erase done
                    sprintf(tBuf,"S29JL064J: Select Binary File Now (enable DSR/DTR)\r\n");
                    putBuf(prt, tBuf,0);
                    //Begin FLASH load using USB
                    loadFLASH(SectAddr71, prt);         //Actual S29JL064J Word ADR=0x200000 "RFI 400000 to display" (upper BackUp image)  
                    break;
                    }                
                else if (!strcmp(tok, "FLSOCK1"))       //FLASH Load via SOCKETs ROC FLASH Sector0=ROC Image1 (fpga1)
                   {
                    //disable watchdog timer, file notifications.c
                    if(prt==tty)
                        {
                        putBuf(prt,"Use Socket ports only\r\n",0);              //tty usb okay
                        break;
                        }          
                    while(FLASH_RDY==0)
                        {
                        putBuf(prt,"FL1_SOCK : Flash not ready\r\n",0); //send to current active port
                        break;
                        }
                    
                    sprintf(tBuf,"FL1_SOCK : Flash Loader FPGA1\r\n");
                    putBuf(prt, tBuf,0);
                    flashStatus(prt);                   //fills Buf1500 with ascii msgs
                    putBuf(prt, tBuf,0);
                    uDelay(100);                   
                    
                    sprintf(tBuf,"FL1_SOCK : FLASH Erase Now\r\n");
                    putBuf(prt, tBuf,0);
                    eraseFLASH_Sector(SectorCnt36, 0, prt);
                    sprintf(tBuf,"FL1_SOCK : Total Bytes erased = %u\r\n", ((SectorCnt36-7)*0x10000));
                    putBuf(prt, tBuf,0);
                    uPhySums.FL_SOCK_CHKSIZE= 0;
                    uPhySums.FL_SOCK_CHKSUM=0;
                    
                    loadFLASH_SOCK(prt,(char*)eRecDatBuf[g_Sock],1);    //ref fpga1
                    sprintf(tBuf,"FL1_SOCK : Finished, reset board for complete update\r\n");
                    putBuf(prt, tBuf,0);
                    uDelay(200);  
                    //flashXFER(4, prt);                //forces fpga to reload from updated Flash 
                    break;
                    }
                else if (!strcmp(tok, "FLSOCK2"))       //FLASH Load via SOCKETs ROC FLASH Sector41=ROC Image2 (fpga2-4)
                   {
                    //disable watchdog timer, file notifications.c
                    if(prt==tty)
                        {
                        putBuf(prt,"FL2_SOCK : Flash not ready\r\n",0); //send to current active port
                        break;
                        }          
                    while(FLASH_RDY==0)
                        {
                        putBuf(prt,"FL2_SOCK : Flash not ready\r\n",0); //send to current active port
                        break;
                        }
                    
                    sprintf(tBuf,  "FL2_SOCK : Flash Loader FPGA2\r\n");
                    putBuf(prt, tBuf,0);
                    flashStatus(prt);                           //fills Buf1500 with ascii msgs
                    putBuf(prt, tBuf,0);
                    uDelay(100);                   
                    
                    sprintf(tBuf,  "FL2_SOCK : FLASH Erase Now\r\n");
                    putBuf(prt, tBuf,0);
                    eraseFLASH_Sector(SectorCnt36, S29JL064J_SECTOR41, prt);  //RFI 220000'
                    sprintf(tBuf,  "FL2_SOCK : Total Bytes erased = %u\r\n", (SectorCnt36*0x10000));
                    putBuf(prt, tBuf,0);
                    uPhySums.FL_SOCK_CHKSIZE= 0;
                    uPhySums.FL_SOCK_CHKSUM=0;
                    
                    loadFLASH_SOCK(prt,(char*)eRecDatBuf[g_Sock],2);  //ref fpga2
                    sprintf(tBuf,"FL2_SOCK : Finished, reset board for complete update\r\n");
                    putBuf(prt, tBuf,0);
                    uDelay(200);                   
                    //flashXFER(4, prt);                //forces fpga to reload from updated Flash 
                    break;
                    }
                else if (!strcmp(tok, "FLSOCK3"))       //FLASH Load via SOCKETs ROC FLASH Sector71=FEB Image
                   {
                    //disable watchdog timer, file notifications.c
                    if(prt==tty)
                        {
                        putBuf(prt,"FL3_SOCK : Flash Socket command only\r\n",0); //send to current active port
                        break;
                        }          
                    while(FLASH_RDY==0)
                        {
                        putBuf(prt,"FL3_SOCK : Flash not ready\r\n",0); //send to current active port
                        break;
                        }
                    
                    sprintf(tBuf,  "FL3_SOCK : FEB Image Flash Loader\r\n");
                    putBuf(prt, tBuf,0);
                    flashStatus(prt);               //displays status
                    uDelay(100);        
                    
                    sprintf(tBuf,  "FL3_SOCK : FLASH Erase Now\r\n");
                    putBuf(prt, tBuf,0);
					
                    //SectAddr71 Actual S29JL064J Byte ADR=0x400000 (FEB BackUp image storeage) 
                    //"RFI 400000 to display"
                    eraseFLASH_Sector71(FileSectCntH, SectAddr71, prt);  //SectorsToErase, SectorStartAddr, DisplayPrt
					//erase done
                    sprintf(tBuf,"FL3_SOCK : Total Bytes erased = %u\r\n", FileSectCntH*0x10000);
                    putBuf(prt, tBuf,0);
                    uPhySums.FL_SOCK_CHKSIZE= 0;
                    uPhySums.FL_SOCK_CHKSUM=0;
                    //flash load begin
                    loadFLASH_SOCK(prt,(char*)eRecDatBuf[g_Sock],3);  //3==SectAddr71 (feb file)
                    //sprintf(tBuf,"FL3_SOCK : Finished, reset board for complete update\r\n");
                    //putBuf(prt, tBuf,0);
                    //uDelay(200);                   
                    //flashXFER(4, prt);                //forces fpga to reload from updated Flash 
                    break;
                    }                            
                
                else if (!strcmp(tok, "FEBSEND"))       //Send FPGA file via ePhy Link from ROC to FEB (expect FEB ready, see FEB cmd 'HF')
                   {
                    //disable watchdog timer, file notifications.c
                    int retval=0, poePort=0, PgmCount=0, actPorts;
                    lvLnk.PoolMode=0;                   //data pool must be off                    
                    lvLnk.PoolChkmSec=0;                //pool update req timer init
                    
                    if ( (poePort=arg_dec(&paramPtr,-1))==-1) //get 1st param, port 1of24h
                        { parErr++;  break;}            //no parameter input error                    
                    if ( (poePort<1) || (poePort>24))   //valid ports 1-24
                        { parErr++;  break;}

                    //Enter number of FEB to FLASH as 1(single) or 24(all)
                    if ( (PgmCount=arg_dec(&paramPtr,-1))==-1) //get 2nd param, pgm board count 1 or 24
                        { parErr++;  break;}            //no parameter input error                    
                    if (!((PgmCount==1) || (PgmCount==24)))
                        { parErr++;  break;}            //valid 1 or 24                     

                    actPorts= ( (ACT_PORTS_HI<<16)+ ACT_PORTS_LO);
                    if((actPorts&(1<<(poePort-1)))==0)
                        {
                        sprintf(tBuf,"ROC Port is inactive %d\r\n",poePort);
                        putBuf(prt, tBuf,0);  
                        break;
                        }
                                        
                    //verify valid image in FLASH
                    u_16Bit cksum;
                    u_32Bit imageSz;
                    //flashStatus(prt);                       //displays status
                    FRAM_RD(DWNLD_3_COUNT,(uint8*)&imageSz, 4); //read 4 bytes
                    FRAM_RD(DWNLD_3_CSUM, (uint8*)&cksum, 2);   //read 2 bytes
                                       
                  //imageSz &=0xfffffe;                     //even file size
                    if ((imageSz==0) || (imageSz>3000000))  //image size to small or large, normal around 2,192,012 bytes
                        return 1;
                    
                    //using main process() function to disable data pooling 
                    sprintf(tBuf,"FEBSEND: Disable data pool\r\n");
                    putBuf(prt, tBuf,0);  
                    sprintf(tBuf,"POOLENA 0");      //command to erase single active FEB
                    process(prt, tBuf);
                    
                    //set single FEB as control board to recieve 'ack checksum' returned on LVDS link'
                    HappyBus.SavePrt= poePort;
                    assignLinkPort(poePort,1);

                    //show ROC is busy for about 1 minute
                    //
                    sprintf(tBuf,"FEBSEND: ROC waiting on FEB(s) FLASH Erase, 50sec\r\n");
                    putBuf(prt, tBuf,0);  

                    //Use process() function, command to FEB(s), erase single of multiple erase flash high                    
                    //
                    if(PgmCount==24)
                        sprintf(tBuf,"LCA FL3ERA");     //command to erase all active FEB                      
                    else
                        sprintf(tBuf,"LC FL3ERA");      //command to erase single active FEB
                    process(prt, tBuf);
                    
                    //ROC has to 'wait' while FEBs erase, Then send fpga image, erase time varies, min ~45 seconds
                    mDelay(1000*55);                    //55 Second Delay
                    
                    //global reset clears all lvds rec fifos
                    *(uSHT*)IOPs[POE01].FM41_PARp= FMRstBit8;//FPGA2 lvds fifo buf and parErr clr              
                    *(uSHT*)IOPs[POE09].FM41_PARp= FMRstBit8;//FPGA3 lvds fifo buf and parErr clr               
                    *(uSHT*)IOPs[POE17].FM41_PARp= FMRstBit8;//FPGA4 lvds fifo buf and parErr clr                                   
                    

                    if(PgmCount==24)     
                        {
                        //enable all Phy to transmit (with active FEB connected)
                        *IOPs[1].ePHY0E_XMSKp= 0xFF;    //enable all port   
                        *IOPs[9].ePHY0E_XMSKp= 0xFF;    //enable all port   
                        *IOPs[17].ePHY0E_XMSKp=0xFF;    //enable all port   
                        }
                    else
                        {
                        //enable 1 Phy to transmit
                        *IOPs[1].ePHY0E_XMSKp= 0;        //disable all port   
                        *IOPs[9].ePHY0E_XMSKp= 0;        //disble all port   
                        *IOPs[17].ePHY0E_XMSKp=0;        //disble all port   
                        *IOPs[poePort].ePHY0E_XMSKp= IOPs[poePort].ePHY_BIT; //enable single phy port
                        }
                    
                    //send data file on ePHY to PMT
                    sprintf(tBuf,"FEBSEND: Sending Image File from local ROC FLASH to FEB\r\n");
                    putBuf(prt, tBuf,0);                                     
                    
                    //Limits background activity during 1mS interrupt routine
                    iFlag |= iPHY_BINMODE;              //set binary xfer flag, see 'notification.c', 
                    retval= SendFile_SrcSector71(prt, HappyBus.PoeBrdCh, PgmCount, cksum, imageSz);  // see 'Mu2e_Cntrl_Misc.c'
                    iFlag &= ~iPHY_BINMODE;             //clr binary xfer flag, see 'notification.c'
                    LED_GRN0;                           //GRN front panel tri-color LED OFF
                    
                    if(retval==0)                       //zero is good, no errors                      
                        sprintf(tBuf,"FEBSEND: Done\r\n\n");//show now error
                    else
                        sprintf(tBuf,"FEBSEND: Error\r\n\n");//show now error
                    putBuf(prt, tBuf,0);                                          
                    mDelay(2000);                       // allow for possible slower FEBs to reply
                              
                    //just in case of Phy Xmit Seq in FPGA, lets reset
                    sprintf(tBuf,"FT");                 //FPGA PHY xmit sequencer may get somewhat hung up at times 
                    process(prt, tBuf);                         
                    mDelay(2500);                       //allow fpga load time
                    
                    //Use process(), cmd FEBs to store last download size and checksum to FEB FRAM and read flash status
                    //
                    u_16Bit d16;
                    FRAM_RD(DWNLD_3_CSUM, (uint8*)&d16, 2); //read 2 bytes FL3 checksum
                    
                    if(PgmCount==24)
                        {
                        sprintf(tBuf,"LCA FL3EOF %X", d16);     //command store FL3 load size and chksum
                        process(prt, tBuf);                    
                        putBuf(prt,"\r\n",0);
                        sprintf(tBuf,"LCA FS");         //command for flash status                      
                        process(prt, tBuf);                    
                        }
                    else
                        {
                        sprintf(tBuf,"LC FL3EOF %X",d16);      //command store FL3 load size and chksum
                        process(prt, tBuf);                         
                        //wait and display returned data
                        while (HappyBus.WaitCnt)        //muti blocks, use timer
                            HappyBusCheck();            //check status/get data, takes ~200nS                        
                        putBuf(prt,"\r\n",0);
                        
                        sprintf(tBuf,"LC FS");          //command for  flash status
                        process(prt, tBuf);                         
                        //wait and display returned data
                        while (HappyBus.WaitCnt)        //muti blocks, use timer
                            HappyBusCheck();            //check status/get data, takes ~200nS                        
                        }
                    
                   
                    putBuf(prt,"\r\n",0);
                    //restore hBus link
                    mDelay(200);                        //wait for hBus reply
                    assignLinkPort(HappyBus.SavePrt,1);                    
                    break;
                    }               
                else if (!strcmp(tok, "FT"))            //Flash reLoad to FPGA
                   {
                    sprintf(tBuf,"Reset ROC after command 'FEBSEND' to fix any link problems\r\n");
                    putBuf(prt, tBuf,0);
                    for(int i=0,j; i<4; i++)
                        {
                        j=flashXFER(i, tty);
                        if (j==-1)
                            iFlag |= CONFIGFAIL;
                        }
                    InitFPGA_REGISTERS();
                    break;
                    }
                else if (!strcmp(tok, "FS"))            //FLASH Status
                   {
                    //int chip=1;
                    u_16Bit D16;
                    u_32Bit D32;
                    flashStatus(prt);                       //displays status
                    FRAM_RD(DWNLD_1_COUNT,(uint8*)&D32, 4); //read 4 bytes
                    FRAM_RD(DWNLD_1_CSUM, (uint8*)&D16, 2);
                    sprintf(Buf1500,"Flash1 BytCt: %d\r\n",D32);
                    sprintf(tBuf,   "Flash1 SumCk: %04X\r\n",D16);
                    strcat (Buf1500,tBuf);
                    FRAM_RD(DWNLD_2_COUNT,(uint8*)&D32, 4); //read 4 bytes
                    FRAM_RD(DWNLD_2_CSUM, (uint8*)&D16, 2); //read 2 bytes
                    sprintf(tBuf   ,"Flash2 BytCt: %d\r\n",D32);
                    strcat (Buf1500,tBuf);
                    sprintf(tBuf,   "Flash2 SumCk: %04X\r\n",D16);
                    strcat (Buf1500,tBuf);

                    //FLASH Sector71 reserved for downloadable FEB image file
                    FRAM_RD(DWNLD_3_COUNT,(uint8*)&D32, 4); //read 4 bytes
                    FRAM_RD(DWNLD_3_CSUM, (uint8*)&D16, 2);   //read 2 bytes
                    sprintf(tBuf   ,"FEBFL3 BytCt: %d\r\n",D32);
                    strcat (Buf1500,tBuf);
                    sprintf(tBuf,   "FEBFL3 SumCk: %04X\r\n",D16);
                    strcat (Buf1500,tBuf);
                    putBuf(prt, Buf1500,0);
                    
                    break;
                    }
                else if (!strcmp(tok, "FI"))        //fpga reset using init poin
                   {
                    //if (prt!=tty)
                    //  break;
                    sprintf(tBuf,"fpga reset to fpga2-4 using PROGRAM_B, not fpga1\r\n");
                    putBuf(prt, tBuf,0);
                    //reset fpga(s)
                    //PROGx_LO                      //not fpga0, hangs network
                    PROG1_LO                        //reset fpag1-3
                    PROG2_LO
                    PROG3_LO
                    uDelay(5);                      //min 300nS
                    //PROGx_HI                      //not fpga0, hangs network                  
                    PROG1_HI
                    PROG2_HI
                    PROG3_HI
                    break;
                    }                
                else if (!strcmp(tok, "FZ"))         //testing flash non 0xffff check
                   {
                    param1=arg_hex(&paramPtr,0x800); //now get 1st param (range to check)
                    sPTR saddr= pFLASHbase;
                    int d16,err=0,i;
                    sprintf(tBuf,"Check/Display 'Un-Erased' FLASH data (FlashBase=%X)\r\n",pFLASHbase);
                    putBuf(prt, tBuf,0);
                    for (i=0; i<param1+1; i++)      
                        {
                        d16= *saddr++;
                        if(d16!=0xffff)
                            {
                            sprintf(tBuf,"Addr= %X  Data==%4X\r\n", saddr, d16);
                            putBuf(prt, tBuf,0);
                            if (++err >= 20) break;
                            }
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
                    FRAM_WR(param1, (uint8*)&param2, 2);        //16bit writes (write 2 bytes)
                    break;
                    }
                else if ((!strcmp(tok,"FD")) || (!strcmp(tok,"FDUMP")))//'FDUMP' or 'FD' read flash status register
                    {
                    uint16_t Adr;
                    uint16_t D16BufIn[10];
                    int lp=0;
                    Adr=arg_hex(&paramPtr,0);              //now get 1st param (addr)
                    sprintf(Buf1500,"Read fRam block (16bit big endian per line)\r\n");
                    putBuf(prt, Buf1500,0);                     //send to current active port
                    while (lp++< 16)
                        {
                        FRAM_RD(Adr, (uint8*)D16BufIn, 8);      //FRAM size (8K x 8 ) display as 8 words per line
                        sprintf(Buf1500,"FRAM=%04X  %4x %4x %4x %4x  ",Adr&0x1fff, D16BufIn[0],D16BufIn[1],D16BufIn[2],D16BufIn[3]);
                        Adr += 8;       // 4 (16 bit reads)
                        FRAM_RD(Adr, (uint8*)D16BufIn, 8);
                        sprintf(tBuf,"%4x %4x %4x %4x\r\n", D16BufIn[0],D16BufIn[1],D16BufIn[2],D16BufIn[3]);
                        Adr += 8;       // 4 (16 bit reads)
                        strcat(Buf1500, tBuf);
                        putBuf(prt, Buf1500,0);             //send to current active port
                        }
                    break;
                    }
                else if  (!strcmp(tok, "FRERASE"))          //erase FRAM FM25CL64B, organized as 8K×8
                    {                                       //256 bytes per fdump of 128 words
                    uint16_t Adr=0, lp=0x400;
                    u_8Bit D8Buf=0;
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//now get 1st param (addr)
                        { parErr++;  break; }
                    if(param1==1)                           //erase dac slp,ped range A00-DFF
                        {
                        Adr=0xA00;
                        lp=0x400;
                        }
                    else if(param1==3)
                        lp=0x800;
                    FRAM_WR_ENABLE();                       //WREN op-code must be issued prior to any Wr_Op
                    dataconfig_FRAM.CS_HOLD = TRUE;         //Hold 'cs' low
                    sendFRAM(&Op[fWRITE]);                  //send fram OP CODE
                    sendFRAM((uint8_t*)&Adr+1);             //send fram ADDR HI
                    sendFRAM((uint8_t*)&Adr);               //send fram ADDR HI
                    //erase fram memory
                    while (lp--)
                        sendFRAM(&D8Buf);
                    //final byte, when done sets chip sel back high
                    dataconfig_FRAM.CS_HOLD = FALSE;        //Normal 'cs' high
                    sendFRAM(&D8Buf);
                    if (param1==1)
                        sprintf(Buf1500,"Zero FRAM, Addr 0xA00-0xDFF (DACs)\r\n");
                    else if (param1==2)
                      sprintf(Buf1500,  "Zero FRAM, Addr 0-0x400\r\n");
                    else if (param1==3)
                        sprintf(Buf1500,"Zero FRAM, Addr 0-0xA00\r\n");
                    putBuf(prt, Buf1500,0);                 //send to current active port
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
                else if  (!strcmp(tok, "FAN"))          //FAN ON/OFF CONTRL
                    {
                    param1=arg_hex(&paramPtr,-1);       //get 1st param FAN 
                    if (param1==1)                       //1==ON else OFF
                        FAN1_HI
                    else
                        FAN1_LO
                    break;
                    }                
                else  {parErr=0xf;   break; }
        case 'H':
                if ((!strcmp(tok, "H1")) || (!strcmp(tok, "HE")))
                    {
                    putBuf(prt, (char *)HelpMenu, sizeof(HelpMenu)-1);          //main help menu
                    break;
                    }
                else if (!strcmp(tok, "H2"))
                    {
                    putBuf(prt, (char *)HelpMenu2, sizeof(HelpMenu2)-1);        //main help menu partA
                    break;
                    }
                else if (!strcmp(tok, "HF"))
                    {
                    putBuf(prt, (char *)HelpMenuFLASH, sizeof(HelpMenuFLASH)-1); 
                    break;
                    }
                else if (!strcmp(tok, "HT"))
                    {
                    putBuf(prt, (char *)HelpMenuTest, sizeof(HelpMenuTest)-1); 
                    break;
                    }
                else if (!strcmp(tok, "HA"))
                    {
                    putBuf(prt, (char *)HelpMenuADRMAP, sizeof(HelpMenuADRMAP)-1); 
                    break;
                    }
                else if (!strcmp(tok, "HN"))
                    {
                    putBuf(prt, (char *)HelpMenu_ORG_TREE, sizeof(HelpMenu_ORG_TREE)-1); 
                    break;
                    }
                else if (!strcmp(tok, "HD"))                                    //hex dump, SRAM Memory
                    {
                    sprintf(tBuf,"Packet %d    \r\n", 1);
                    putBuf(prt, tBuf,0);
                   //HexDump((char*)phyHdr.msgStr[i], MsgBytMax, prt);  //phyHdr.msgBytes[i]);
                    HexDump((char*)Buf1500, 20, prt);  //phyHdr.msgBytes[i]);
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'I':
                if (!strcmp(tok, "ID"))
                   {
                    u_16Bit D16;
                    u_32Bit D32;
                    int i,j;
                    char  txtBuf40[40];
                    struct time ti;
                    sPTR zAdr = (sPTR) ZestETM1;    //get Phy Link Status
                    param1=arg_dec(&paramPtr,0);
                    sprintf(Buf1500,"Module Type : FEB Controller ROC Tester Version\r\n");
                    zAdr = (sPTR) ZestETM1;                 
                    i= *(zAdr+(0x200/2));
                    j= *(zAdr+(0x202/2));
                    sprintf(tBuf,"IPv4 Address: %u.%u.%u.%u\r\n", i>>8&0xff,i&0xff,j>>8&0xff,j&0xff);
                    strcat(Buf1500, tBuf);
                    
                    strncpy(txtBuf40,__VERSION__,31);  //shorten IAR response
                	sprintf(tBuf,"Compiler Ver: %s\r\n", txtBuf40 );
                    strcat(Buf1500, tBuf);
                    
                    sprintf(tBuf,"uC Compile  : %s\r\n", __DATE__);
                    strcat(Buf1500, tBuf);
                    sprintf(tBuf,"uC Code Ver : %d.%02d\r\n", divV.quot, divV.rem );
                    strcat(Buf1500, tBuf);
                    
                    FRAM_RD(DWNLD_1_COUNT,(uint8*)&D32, 4);     //read 4 bytes
                    FRAM_RD(DWNLD_1_CSUM, (uint8*)&D16, 2);
                    sprintf(tBuf   ,"Flash1 BytCt: %d\r\n",D32);
                    strcat (Buf1500,tBuf);
                    sprintf(tBuf,   "Flash1 SumCk: %04X\r\n",D16);
                    strcat (Buf1500,tBuf);
                    FRAM_RD(DWNLD_2_COUNT,(uint8*)&D32, 4);     //read 4 bytes
                    FRAM_RD(DWNLD_2_CSUM, (uint8*)&D16, 2);
                    sprintf(tBuf   ,"Flash2 BytCt: %d\r\n",D32);
                    strcat (Buf1500,tBuf);
                    sprintf(tBuf,   "Flash2 SumCk: %04X\r\n",D16);
                    strcat (Buf1500,tBuf);
                    
                    sprintf(tBuf,   "------------:\r\n");
                    strcat (Buf1500,tBuf);
                  //Add Uptime counter
                  //ti.totalSec= (86400 + (23*3600) + (59*60) + 59); //test numbers
                    ti.totalSec= (sUPTIMEHI<<16) + sUPTIMELO;
                    ti.sec = (ti.totalSec  %60);
                    ti.min = ((ti.totalSec /60) %60);
                    ti.hr  = ((ti.totalSec /3600) %24);
                    ti.day = ti.totalSec   /86400;
                    sprintf(tBuf,"DAY HH:MM:SS: %02d %02d:%02d:%02d\r\n", ti.day,ti.hr,ti.min,ti.sec);
                    strcat (Buf1500,tBuf);
                    
                    sprintf(tBuf,   "LVDS Rec Err: %d\r\n",HappyBus.FMRecvErr);
                    strcat (Buf1500,tBuf);
                    if(param1) HappyBus.FMRecvErr=0;        //clear counter
                    
                    //FRAM_RD(SERIAL_NUMB_ADR, (uint16_t*)&D16,2);    //read 2 bytes
                    sprintf(tBuf,"Contrl Numb : %d\r\n",Ser_Cntrl_Numb.CntrlNumb);
                    strcat (Buf1500,tBuf);
                    sprintf(tBuf,"Serial Numb : %d\r\n",Ser_Cntrl_Numb.serNumb);
                    strcat (Buf1500,tBuf);
                    
                    //FRAM_RD(fram_NERROR_CNT, (uint8*)&u_FRAM.nErrorCnt, 2); //addr,data,cnt
                    //sprintf(tBuf,   "RM48 ECC_ERR: %d\r\n",u_FRAM.nErrorCnt);
                    //strcat (Buf1500,tBuf);
                    
                    //link_ID_Chk() fills status array, see 'lvLnk.IDChkSec== ID_RegTime' 
                    sprintf(tBuf,   "Active FEBs : %d\r\n",POE_PORTS_ACTIVE[0]);
                    strcat (Buf1500,tBuf);
                    putBuf(prt, Buf1500,0);
                    break;
                    }
                 else if (!strcmp(tok, "IDS")) // ID - Serial
                    {
                    sprintf(Buf1500, "%X04\r\n", Ser_Cntrl_Numb.serNumb);
                    putBuf(prt, Buf1500,0);
                    break;
                    }
                else if (!strcmp(tok, "IDV1")) // ID - Version major
                    {
                    sprintf(Buf1500, "%04X\r\n", divV.quot);
                    putBuf(prt, Buf1500,0);
                    break;
                    }
                else if (!strcmp(tok, "IDV2")) // ID - Version minor
                    {
                    sprintf(Buf1500, "%04X\r\n", divV.rem);
                    putBuf(prt, Buf1500,0);
                    break;
                    }
                else if (!strcmp(tok, "IDA")) // ID - active FEBs
                    {
                    sprintf(Buf1500, "%04X\r\n", POE_PORTS_ACTIVE[0]);
                    putBuf(prt, Buf1500,0);
                    break;
                    }
                else if (!strcmp(tok, "IRD"))               //I2C1 Port READ testing 'POE'
                    {
                    int retVal;
                    static int datL, datH;
                    if ( (param1=arg_hex(&paramPtr,-1)) ==-1) //get POE Chip 1of6
                        { parErr++;  break; }
                    if ( (param2=arg_hex(&paramPtr,-1)) ==-1) //get reg addr to read
                        { parErr++;  break; }
                    //READ LSB 1ST
                     retVal= i2cRecvData(PoeAddr[param1],param2, 1, (uint8_t*) &datL, 0);
                    //READ MSB
                    retVal= i2cRecvData(PoeAddr[param1], param2, 1, (uint8_t*) &datH, 0 );
                    if(retVal)
                        sprintf(tBuf,"i2c error\r\n");
                    else
                        sprintf(tBuf,"data= %X\r\n", datL);
                    putBuf(prt, tBuf,0);
                    break;
                    }
                else if (!strcmp(tok, "IWR"))               //I2C1 Port WRITE testing 'POE'
                    {                                       //write( devAddr, regAddr, dataByte )
                    int retVal;
                    static int datL;//datH;
                    if ( (param1=arg_hex(&paramPtr,-1)) ==-1) //devAddr chip 1of6
                        { parErr++;  break; }
                    if ( (param2=arg_hex(&paramPtr,-1)) ==-1) //reg addr
                        { parErr++;  break; }
                    if ( (datL=arg_hex(&paramPtr,-1)) ==-1) //data param
                        { parErr++;  break; }
                    retVal= i2cSendData(PoeAddr[param1], param2, 1, (uint8_t*) &datL ); //i2cSendData (dev,reg,cnt,data[])
                    if(retVal)
                        {
                        sprintf(tBuf,"i2c error\r\n");
                        putBuf(prt, tBuf,0);
                        }
                    break;
                    }
                else if (!strcmp(tok, "IAR"))               //I2C1 Port 'ARA' intr check
                    {                                       //write( devAddr, regAddr, dataByte )
                    int retVal=0;
                    static uint8_t dat8;

                    //Read and store intr reg
                    for (int i=0;i<6;i++)
                        {
                        retVal= i2cRecvData(PoeAddr[i], 0x0, 1, &dat8, 0 ); //i2cSendData (dev,reg,cnt,data[])
                        if(retVal)
                            {
                            sprintf(tBuf,"i2c error %d\r\n",i);
                            putBuf(prt, tBuf,0);
                            }
                        else
                            {
                            sprintf(tBuf,"data= %2X\r\n",dat8);
                            putBuf(prt, tBuf,0);
                            }
                        }
                    break;
                    }
                else if (!strcmp(tok, "IRST"))              //I2C1 Port Intr resetting
                    {                                       //write( devAddr, regAddr, dataByte )
                    int retVal=0;
                    static uint8_t datL;

                    //set intr masks off
                    datL=0x00;
                    for (int i=0;i<6;i++)
                        retVal= i2cSendData(PoeAddr[i], 1, 1, &datL ); //i2cSendData (dev,reg,cnt,data[])
                    
                    //Clear-on-Read register 3 
                    for (int i=0;i<6;i++)
                        retVal= i2cRecvData(PoeAddr[i], 3, 1, &datL, 0 ); //i2cSendData (dev,reg,cnt,data[])
                    //Clear-on-Read register 5 
                    for (int i=0;i<6;i++)
                        retVal+= i2cRecvData(PoeAddr[i], 5, 1, &datL, 0 ); //i2cSendData (dev,reg,cnt,data[])
                    //Clear-on-Read register 7 
                    for (int i=0;i<6;i++)
                        retVal+= i2cRecvData(PoeAddr[i], 7, 1, &datL, 0 ); //i2cSendData (dev,reg,cnt,data[])
                    //Clear-on-Read register 9 
                    for (int i=0;i<6;i++)
                        retVal+= i2cRecvData(PoeAddr[i], 9, 1, &datL, 0); //i2cSendData (dev,reg,cnt,data[])
                    //Clear-on-Read register B 
                    for (int i=0;i<6;i++)
                        retVal+= i2cRecvData(PoeAddr[i], 0xb, 1, &datL, 0); //i2cSendData (dev,reg,cnt,data[])

                    //Clear PB Intr
                    datL=0xff;
                    for (int i=0;i<6;i++)
                        retVal+= i2cSendData(PoeAddr[i], 0x1A, 1, &datL ); //i2cSendData (dev,reg,cnt,data[])

                    //Clear Interrupt Enable
                    datL=0;
                    for (int i=0;i<6;i++)
                        retVal+= i2cSendData(PoeAddr[i], 0x17, 1, &datL ); //i2cSendData (dev,reg,cnt,data[])
                   
                    if(retVal)
                        {
                        sprintf(tBuf,"i2c error\r\n");
                        putBuf(prt, tBuf,0);
                        }
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'L':
                if (!strcmp(tok, "LC77"))            //Link Cmd, reqs data 1of24 connected FEBs
                   {
                    if (POE_PORTS_ACTIVE[HappyBus.PoeBrdCh]==0)
                        {
                        sprintf(tBuf,"Port %d may be InActive\r\n", HappyBus.PoeBrdCh);
                        putBuf(prt, tBuf,0);
                        genFlag &= ~ECHO_ACT_PORT;
                        break;
                        }        
                    
                    HappyBus.FM_PAR = (uINT)IOPs[HappyBus.PoeBrdCh].FM41_PARp;                    
                    REG16(HappyBus.FM_PAR)= FMRstBit8;  //BIT8, clear buffer, fpga                     
                    //buffer clear wait, its needed for some reason ??
                    if(HappyBus.PoeBrdCh>1)
                        uDelay(50);  
                    else 
                        uDelay(50);  

                    //lvLnk.PoolChkmSec=0;
                    genFlag |= ECHO_ACT_PORT;
                    HappyBus.CmdType= eCMD77_BINARY_DAT;                   
                    //make it slow else wont pick up super slow cmd 'A0' data 1 sec per reads
                    PHY_LOADER_eCMD77(paramPtr, prt,  HappyBus.PoeBrdCh, eECHO_ON,0); //cmd,poeprt,echoON,broadCast=1)                                            
                    HappyBus.WaitCnt= 10000;        //wait 5mS, ~500nS per HappyBusCheck() call
                    iFlag |= iNoPrompt;             //flag as no prompt on terminal 
                    break;                    
                   }                                                        
                
                else if (!strcmp(tok, "LC"))            //Link Cmd, reqs data 1of24 connected FEBs
                   {
                    if (POE_PORTS_ACTIVE[HappyBus.PoeBrdCh]==0)
                        {
                        sprintf(tBuf,"Port %d may be InActive\r\n", HappyBus.PoeBrdCh);
                        putBuf(prt, tBuf,0);
                        genFlag &= ~ECHO_ACT_PORT;
                        break;
                        }        
                    HappyBus.FM_PAR = (uINT)IOPs[HappyBus.PoeBrdCh].FM41_PARp;                    
                    REG16(HappyBus.FM_PAR)= FMRstBit8;  //BIT8, clear buffer, fpga                     
                    //buffer clear wait, its needed for some reason ??
                    //uDelay(50);  

                    genFlag |= ECHO_ACT_PORT;
                    HappyBus.CmdType= eCMD71_CONSOLE;                   
                    PHY_LOADER_CONSOLE(paramPtr, prt,  HappyBus.PoeBrdCh, eECHO_ON,0); //cmd,poeprt,echoON,broadCast=1)                     
                    iFlag |= iNoPrompt;             //flag as no prompt on terminal 
                    break;                    
                   }                                                        
                else if (!strcmp(tok, "LCA"))       //remote lvds link command to all boards
                    {                               //SEND CMD STRING ON ePHY port to FEB                      
                    int actPorts, wrMode=0;                    
                    actPorts= ( (ACT_PORTS_HI<<16)+ ACT_PORTS_LO);
                    
                    //example ROC write command "LCA WR 26 5678" 
                    //At initial entry          paramPtr-> "LCA WR 26 5678" 
                    //After strcmp(tok, "LCA" ) paramPtr-> "WR 26 5678"
                    if ((*paramPtr=='W') && (*(paramPtr+1)=='R'))//check "WR" ocmmand type
                        wrMode=1;                   //write command strin                    
                    
                    if (HappyBus.SavePrt==0)
                         HappyBus.SavePrt=1;        //just in case, force valid ptrs else 'boom'

                    //if write mode skip clearing read buffers
                    if (wrMode==0)
                        {
                        //clear all lvds receive fifos
                        *(uSHT*)IOPs[POE01].FM41_PARp= FMRstBit8;//FPGA2 lvds fifo buf and parErr clr              
                        *(uSHT*)IOPs[POE09].FM41_PARp= FMRstBit8;//FPGA3 lvds fifo buf and parErr clr               
                        *(uSHT*)IOPs[POE17].FM41_PARp= FMRstBit8;//FPGA4 lvds fifo buf and parErr clr               
                        }

                    for(int i=1,j=0; i<25; i++,j++)
                        {
                        if((actPorts&(1<<j))==0)        //does poe port have an active FEB
                            continue;
                        sprintf(ReplyBuf80,"-Port%02d\r\n",i); //prompt show port number
                        putBuf(prt,ReplyBuf80,0);
                        assignLinkPort(i,0);            //no fifo reset, changes all lvds and ephy pointers to new port 1of24  
                        HappyBus.PoeBrdCh= i;                        
                        HappyBus.CmdType= eCMD71_CONSOLE;

                        //make it slow else wont pick up super slow cmd 'A0' data 1 sec per reads
                        PHY_LOADER_CONSOLE(paramPtr, prt,  HappyBus.PoeBrdCh, eECHO_ON,0); //cmd,poeprt,echoON,broadCast=1)  
                        
                        //skip the wait/reading of return data on writes
                        if(wrMode)
                            continue;                   //skip return data check
                        
                        //return data word(s) have initial ASCII formatting delays, hard to know when its done
                        //commands return data sizes from a 2 words to 2K words 
                        HappyBus.WaitCnt= 10000;        //wait 500mS, ~2.5uS per background check on HappyBusCheck()
                        while (HappyBus.WaitCnt)
                            HappyBusCheck();            //check status/g4et data, takes ~200nS                        
                        }
                    HappyBus.WaitCnt=0;                 //done
                    assignLinkPort(HappyBus.SavePrt, 1);  
                    HappyBus.SlowReply=0;
                    break;
                    }
                else if (!strcmp(tok, "LCB"))      //remote lvds link command to all boards
                    {                               //SEND CMD STRING ON ePHY port to FEB                      
                    int actPorts;                    
                    actPorts= ( (ACT_PORTS_HI<<16)+ ACT_PORTS_LO);                    
                    if (HappyBus.SavePrt==0) 
                         HappyBus.SavePrt=1;        //just in case, force valid ptrs else 'boom'
                    
                    //if (wrMode==0)
                    //    {
                        //clear all lvds receive fifos
                        *(uSHT*)IOPs[POE01].FM41_PARp= FMRstBit8;//FPGA2 lvds fifo buf and parErr clr              
                        *(uSHT*)IOPs[POE09].FM41_PARp= FMRstBit8;//FPGA3 lvds fifo buf and parErr clr               
                        *(uSHT*)IOPs[POE17].FM41_PARp= FMRstBit8;//FPGA4 lvds fifo buf and parErr clr               
                    //    }
                    
                    HappyBus.CmdType= eCMD71_CONSOLE;
                    //make it slow else wont pick up super slow cmd 'A0' data 1 sec per reads
                    PHY_LOADER_CONSOLE(paramPtr, prt,  HappyBus.PoeBrdCh, eECHO_ON,1); //cmd,poeprt,echoON,broadCast=1)                        
                    
                    //example ROC write command "LCB WR 26 5678" 
                    //At initial entry          paramPtr-> "LCB WR 26 5678" 
                    //After strcmp(tok, "LCA" ) paramPtr-> "WR 26 5678"
                    if (!((*paramPtr=='W') && (*(paramPtr+1)=='R')))    //check "WR" ocmmand type
                        {
                        for(int i=1,j=0; i<25; i++,j++)
                            {
                            if((actPorts&(1<<j))==0)        //does poe port have an active FEB
                                continue;
                            //return data word(s) have initial ASCII formatting delays, hard to know when its done
                            //commands return data sizes from a 2 words to 2K words 
                            assignLinkPort(i,0); //no fifo reset, changes all lvds and ephy pointers to new port 1of24
                            HappyBus.PoeBrdCh= i;
                            HappyBus.WaitCnt= 10000;       //wait 500mS, ~2.5uS per background check on HappyBusCheck()
                            while (HappyBus.WaitCnt)        //muti blocks of slow data returns use timer
                                HappyBusCheck();            //check status/get data, takes ~200nS                        
                            }                       
                        }

                    HappyBus.WaitCnt=0;                 //done
                    assignLinkPort(HappyBus.SavePrt, 1); 
                    HappyBus.SlowReply=0;
                    break;
                    }
                
                else if (!strcmp(tok, "LP"))                //remote lvds link PORT 
                    {                                       //Set Bus Board Numb 1-24, 0=all
                    int i=1;
                    if(POE_PORTS_ACTIVE[0]==0)              //FEB attached total count 
                        {
                        sprintf(Buf1500,"No FEBs\r\n");
                        putBuf(prt, Buf1500,0);
                        break;
                        }
                    param1=arg_dec(&paramPtr,-1);           //1st param (board number)                    
                    if ((param1<1) || (param1>24))          //out of range as def, show status
                        {
                        genFlag &= ~ECHO_ACT_PORT;          //stop newLine prompt with Port#
                        sprintf(Buf1500,"PORTs=%02d  ",POE_PORTS_ACTIVE[0]);
                        //display line with active port
                        for (; i<25; i++)
                            {
                            if (POE_PORTS_ACTIVE[i]==0)
                                sprintf(tBuf,". ");
                            else
                                {
                                if(i==HappyBus.PoeBrdCh)    //is this our 'LP' assigned prt
                                    sprintf(tBuf,"(%d) ",i);
                                else
                                    sprintf(tBuf,"%d ",i);
                                }
                            strcat (Buf1500,tBuf);          //build complete reply string  
                            }
                        strcat (Buf1500,"\r\n\r");
                        putBuf(prt, Buf1500,0);
                        break; 
                        }  
                    //check if this is an active port
                    if (POE_PORTS_ACTIVE[param1]==0)
                        putBuf(prt, "InActive?\r\n", 0);    
                    //update screen prompt to show 'LC' commands active port
                    sprintf(USB_Rec.prompt,"\nP%d>",param1 );
                    genFlag |= ECHO_ACT_PORT;               //ena newLine prompt with Port#
                    //for the above 'New Prompt' adjust print lenght in newLinePrompt()                    
                    HappyBus.SavePrt= param1;
                    assignLinkPort(param1,1);
                    break;
                    }                       
                else if (!strcmp(tok, "LI"))            //lvds link attached FEB return clk check
                    {                                   //Set Bus Board Numb 1-24, 0=all
                    lvLnk.IDChkSec= ID_RegTime-1;       //use timer, does same as link_ID_Chk(0);                    
                    break;
                    } 
                else if (!strcmp(tok, "LPR"))
                    {
                    sprintf(Buf1500,"%04X\r\n",HappyBus.PoeBrdCh);
                    putBuf(prt, Buf1500,0);                 //send to current active port  
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'N':
                if (!strcmp(tok, "NETSAV"))             //restoring network defaults
                    {
                    //update OrangeTree if IP, GateWay, or Mask changed.
                    if(netInfo.UpDate==1)
                        {
                        sprintf(tBuf,"ZSU");            //update OrangeTree params
                        process(prt, tBuf);
                        }
                    netInfo.Valid= VALID;               //flag, mark as valid data
                    FRAM_WR_ENABLE();                   //WREN op-code must be issued prior to any Wr_Op
                    //FRAM_WR (destAddr, dataPtr, byte count )
                    FRAM_WR(FRAM_AddrNet500, (uint8*)&netInfo, sizeof(netInfo)); //16bit writes (write 2 bytes)
                    FRAM_WR_DISABLE();
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'P':
                if (!strcmp(tok, "POOL"))               //display 'datapool' data
                    {
                    int LF=1, idx, grp;
                    u_16Bit *poolPtr= &stab_Pool[0][0];
                    param1=arg_dec(&paramPtr,0);        //1st param (rd/clr pool)
                    for (int Brd1_24=1; Brd1_24<25; Brd1_24++)
                        {
                        if (POE_PORTS_ACTIVE[Brd1_24]==0)
                            continue;  
                        sprintf (Buf1500,"PORT(%d)\r\n", Brd1_24);
                        putBuf(prt, Buf1500,0);          
                        poolPtr= &stab_Pool[Brd1_24-1][0];                       
                        for (idx=0,LF=1; idx<sSMALLSIZ; idx++, LF++, poolPtr++)  
                            {
                            sprintf (Buf1500,"%4X ", *poolPtr);
                            //1st param (rd/clr pool data)
                            if (param1)
                                *poolPtr=0;
                            if (LF%16==0)
                                strcat(Buf1500, "\r\n");
                            putBuf(prt, Buf1500,0);          
                            }
                        putBuf(prt,"\r\n",0); 
                        
                        for (grp=1; grp<5; grp++)
                            {
                            for (LF=1, idx=0; idx<sBLOCKSIZ; idx++, LF++, poolPtr++)  
                                {
                                //1st param (rd/clr pool data)
                                sprintf (Buf1500,"%4X ", *poolPtr);
                                if (param1)
                                    *poolPtr=0;
                                if (LF%16==0)
                                    strcat(Buf1500, "\r\n");
                                putBuf(prt, Buf1500,0);              
                                }
                            putBuf(prt,"\r\n",0); 
                             }
                        }  
                    break;
                    }
                
                else if (!strcmp(tok, "POOLPAR"))               //display 'datapool' data
                    {
                    //int LF=1, idx, grp;
                    //u_16Bit *poolPtr= &stab_Pool[0][0];
                    param1=arg_dec(&paramPtr,0)-1;        //1st param (rd/clr pool)
                    param2=arg_dec(&paramPtr,1);
                    sprintf (Buf1500, "%4X", stab_Pool[param1][param2]);
                    putBuf(prt, Buf1500, 0);
                    if (prt != DCS)
                    {
                        putBuf(prt,"\r\n",0);
                    }
                     break;
                }
                    
  
                else if (!strcmp(tok, "POOLENA"))           //for testing stop data pool
                    {
                    param1=arg_dec(&paramPtr,1);            //default mode is enable
                    PoolMode= param1;
                    if(PoolMode==1)                         //enable=1
                        {
                        genFlag |= PoolReqNow;              //make new req now
                        lvLnk.PoolChkmSec= ReqPoolTime30-2;   //force update now
                        genFlag &= ~PoolReqNow;            
                        genFlag &= ~PoolReqGetData;            
                        sprintf(tBuf,"Data Pool ON\r\n");
                        lvLnk.PoolMode=1;
                        }
                    else if(PoolMode==0)                    //disable=0
                        {
                        //all genFlags cleared in 'notifications.c' when disabled
                        lvLnk.PoolMode=0;
                        sprintf(tBuf,"Data Pool OFF\r\n");
                        }  
                    putBuf(prt, tBuf,0);
                    break;
                    }
                else if (!strcmp(tok, "PWR"))               //LTC4266A POE+ Chips Readout
                    {
                    int checked=0;
                    //POE ACTIVE PORTS here means readback Volts > 1
                    sprintf(tBuf,"Port   Volts     Amps    Watts\r\n");
                    putBuf(prt, tBuf,0);

                    //display current and voltage for ports
                    float cur, vlt;
                    //Display POE register 'current and voltage'
                    for (int i=0,l=1; i<24; i++, l++)
                        {
                        cur= POE_CUR_ALL[i];
                      //Current, 1 LSB = 61.035µA when RSENSE= 0.5O or 122.07µA when RSENSE= 0.25O
                         cur= cur * .00012207;
                        vlt= POE_VLT_ALL[i];
                        vlt= vlt * .005760;  //.005835;

                        if ((vlt < 1) || (vlt>100))
                            continue;
                        checked++;
                        if (cur<2)
                            sprintf(tBuf,"%02d    %4.3f    %4.3f    %4.2f\r\n", l,vlt,cur,(cur*vlt));
                        else
                            sprintf(tBuf,"%02d    %4.3f    %4.3f    ???\r\n", l,vlt,cur);
                        putBuf(prt, tBuf,0);
                        }
                    if (checked==0)  //no connections
                        {
                        sprintf(tBuf,"  0      0        0        0\r\n");
                        putBuf(prt, tBuf,0);
                        }                      
                    break;
                    }
                else if (!strcmp(tok, "PWRV"))
                    {
                    param1=arg_dec(&paramPtr,0); // select port
                    if((param1<24))
                        {
                        sprintf(tBuf,"%04X\r\n",POE_VLT_ALL[param1]);
                        putBuf(prt, tBuf, 0);
                        break;
                        } else {
                        sprintf(tBuf,"ffff\r\n");
                        putBuf(prt, tBuf, 0);
                        break;
                        }
                    }
                 else if (!strcmp(tok, "PWRA"))
                    {
                    param1=arg_dec(&paramPtr,0); // select port
                    if((param1<24))
                        {
                        sprintf(tBuf,"%04X\r\n",POE_CUR_ALL[param1]);
                        putBuf(prt, tBuf, 0);
                        break;
                        } else {
                        sprintf(tBuf,"ffff\r\n");
                        putBuf(prt, tBuf, 0);
                        break;
                        }
                    }
                else if (!strcmp(tok, "PWROT"))             //Power cycle Orange Tree Dau Board 'ZestETM1'
                    {
                    //2020 Production Boards need hardware mods for this to work
                    //probably wont be done. SBND version board has hardware mods and code todo power reset
                    _disable_interrupt_();
                    oTreeOff                                //pwr cylce orange tree board
                    for(int w=0; w<5000; w++);              //delay some
                    oTreeOn                      
                    REG16(ZestETM1)= 0; 
                    //reset board for a clean start
                    resetEntry();
                    break;
                    }					
                else if (!strcmp(tok, "PWRRST"))                    //RESET LTC4266A POE+ Chips
                    {
                    uint8_t dat8= 0x10;                      
                    div_t pwrRst;
    
                    param1=arg_dec(&paramPtr,0);                   //now get 1st param, trig cnt
                    if((param1<1) || (param1>25))
                        {putBuf(prt, "Enter 1-24 or 25(All)\r\n",0); break; }
                    
                    //clear POE_PORTS_ACTIVE list
                    for(int i=1; i<25; i++)
                        POE_PORTS_ACTIVE[i]=0;
                    if(param1==25)
                        {  //now reset all, 6*4                      
                        for (int i=0;i<6;i++)       //6 LTC4266A IC's, 4Ch each
                            {
                            //dat8= 0x10 reset all 4 ports
                            poePortWr( PoeAddr[i], ltc_RSTPB, 1, &dat8);    //poePortWr(dev, reg, cnt, data[] )
                            mDelay(100);
                            }                   
                        }
                    else
                        {
                        //each LTC4266A IC 0x10--0x40 reset ch1-ch4
                        pwrRst= div(param1-1,4);                //-1 for div to work
                        dat8= (0x10 * (pwrRst.rem+1));          //need 1thru4 *10 here
                        dat8= i2cSendData( PoeAddr[pwrRst.quot], ltc_RSTPB, 1, &dat8);  //dat8= reset single port
                        if (dat8)
                            {
                            sprintf(tBuf,"i2c link error\r\n");
                            putBuf(prt, tBuf,0);
                            //sprintf(tBuf,"quot=%d   rem=%d   PoeAddr=%X    dat8=%X\r\n",pwrRst.quot, pwrRst.rem, PoeAddr[pwrRst.quot], dat8);
                            //putBuf(prt, tBuf,0);
                            }
                        }                      
                    break;
                    }                

                else if (!strcmp(tok, "PTRIG0"))        //used with tester code   
                    {
                    int cnt=0;
                    param1=arg_dec(&paramPtr,1);        //now get 1st param, trig cnt
                    if((param1<0) || (param1>1000))
                       param1=1;
                    param2=arg_dec(&paramPtr,0);        //display xmit data, 0=no
                        
                    //tekmod dec2022
                    uBReq.ReqCnt= param1*5;
                    
                    //trigger test commands
                    //Send data on GTP1
                    for(cnt=0; cnt<=param1; cnt++)
                      {
                      for(int i=0, j=0; j<6; j++)            //use 5 different payloads
                        {
                        GTP0_PREAM= ptrg0[i++];          //GTP1 PREAMBLE REG
                        GTP0_PAYLD= ptrg0[i++];          //GTP1 PAYLOAD REG
                        GTP0_PAYLD= ptrg0[i++];          //GTP1 PAYLOAD REG
                        GTP0_PAYLD= ptrg0[i++];          //GTP1 PAYLOAD REG
                        GTP0_PAYLD= ptrg0[i++];          //GTP1 PAYLOAD REG
                        GTP0_PAYLD= ptrg0[i++];          //GTP1 PAYLOAD REG
                        GTP0_PAYLD= ptrg0[i++];          //GTP1 PAYLOAD REG
                        GTP0_PAYLD= ptrg0[i++];          //GTP1 PAYLOAD REG
                        GTP0_PAYLD= ptrg0[i++];          //GTP1 PAYLOAD REG
                        GTP0_XMIT = 0;                   //GTP1 XMIT REG
                        i++;
                        }
                      }
                    
                    if(param2!=0)
                        {
                        int i=0, j=1;
                        putBuf(prt,"RDM 20 GTP0\r\n",0);
                        while(i < (5*10))  //6 lines of 10 words per line
                            {
                            sprintf(tBuf,"%4x ", ptrg0[i]); 
                            putBuf(prt,tBuf,0); 
                            if (j%10==0)
                                putBuf(prt,"\r\n",0); 
                            i++; j++;
                            }
                        //sprintf(tBuf,"Repeats %d times\r\n", param1); 
                        sprintf(tBuf,"\r\n"); 
                        putBuf(prt,tBuf,0); 
                        }
                    break;
                    }
                
                else if (!strcmp(tok, "PTRIG1"))        //used with tester code   
                    { 
                    int cnt=0;
                    param1=arg_dec(&paramPtr,1);        //now get 1st param, trig cnt
                    if((param1<0) || (param1>1000))
                       param1=1;
                    param2=arg_dec(&paramPtr,0);        //display xmit data, 0=no
                    
                    //tekmod dec2022
                    uBReq.ReqCnt= param1*5;
                                          
                    //trigger test commands
                    //Send data on GTP1
                    for(cnt=0; cnt<param1; cnt++)
                      {
                      for(int i=0,j=0; j<6; j++)        //use 5 different payloads
                        {
                        GTP1_PREAM= ptrg1[i++];          //GTP1 PREAMBLE REG
                        GTP1_PAYLD= ptrg1[i++];          //GTP1 PAYLOAD REG
                        GTP1_PAYLD= ptrg1[i++];          //GTP1 PAYLOAD REG
                        GTP1_PAYLD= ptrg1[i++];          //GTP1 PAYLOAD REG
                        GTP1_PAYLD= ptrg1[i++];          //GTP1 PAYLOAD REG
                        GTP1_PAYLD= ptrg1[i++];          //GTP1 PAYLOAD REG
                        GTP1_PAYLD= ptrg1[i++];          //GTP1 PAYLOAD REG
                        GTP1_PAYLD= ptrg1[i++];          //GTP1 PAYLOAD REG
                        GTP1_PAYLD= ptrg1[i++];          //GTP1 PAYLOAD REG
                        GTP1_XMIT = 0;                   //GTP1 XMIT REG
                        i++;
                        }
                      }
                    
                    //Note: WBL code in FPGAs has no buffer word count register so we loop on xmit count
                    if(param2!=0)
                        {
                        putBuf(prt,"RDM 21 GTP1\r\n",0);
                        int i=0, j=1;
                        while(i < (5*10))  //6 lines of 10 words per line
                            {
                            sprintf(tBuf,"%4x ", ptrg1[i]); 
                            putBuf(prt,tBuf,0); 
                            if (j%10==0)
                                putBuf(prt,"\r\n",0); 
                            i++; j++;
                            }
                        //sprintf(tBuf,"Repeats %d times\r\n", param1); 
                        sprintf(tBuf,"\r\n"); 
                        putBuf(prt,tBuf,0); 
                        }
                    break;
                    }                    
                else if (!strcmp(tok, "PINIT"))      
                    {
                    //trigger test commands
                    wr16FPGA(0x0, 0x40);                   
                    //wr16FPGA(0x8, 0xFF);
                    //wr16FPGA(0x9, 0xFFFF);
                    wr16FPGA(0x300, 0x88);
                    wr16FPGA(0x2, 0x1);
                    wr16FPGA(0x27, 0x00);
                    break;
                    }					
                else if (!strcmp(tok, "PTRIG"))         //used for DAQ uBunch Request  
                    {
                    int cnt=0;
                    param1=arg_dec(&paramPtr,1);        //now get 1st param, trig cnt
                    if((param1<0) || (param1>1000))
                       param1=1;
                    param2=arg_dec(&paramPtr,0);        //display xmit data, 0=no
                        
                    //tekmod dec2022
                    uBReq.ReqCnt= param1;
                    
                    //trigger test commands
                    //Send data on GTP1
                    for(cnt=0; cnt<=param1; cnt++)
                      {
                      for(int i=0, j=0; j<6; j++)        //use GTP1, 5 different payloads
                        {
                        GTP1_PREAM= ptrig[i++];          //GTP1 PREAMBLE REG
                        GTP1_PAYLD= ptrig[i++];          //GTP1 PAYLOAD REG
                        GTP1_PAYLD= ptrig[i++];          //GTP1 PAYLOAD REG
                        GTP1_PAYLD= ptrig[i++];          //GTP1 PAYLOAD REG
                        GTP1_PAYLD= ptrig[i++];          //GTP1 PAYLOAD REG
                        GTP1_PAYLD= ptrig[i++];          //GTP1 PAYLOAD REG
                        GTP1_PAYLD= ptrig[i++];          //GTP1 PAYLOAD REG
                        GTP1_PAYLD= ptrig[i++];          //GTP1 PAYLOAD REG
                        GTP1_PAYLD= ptrig[i++];          //GTP1 PAYLOAD REG
                        GTP1_XMIT = 0;                   //GTP1 XMIT REG
                        i++;
                        }
                      }
                    
                    if(param2!=0)
                        {
                        int i=0, j=1;
                        putBuf(prt,"RDM 21 GTP1\r\n",0);
                        while(i < (5*10))  //6 lines of 10 words per line
                            {
                            sprintf(tBuf,"%4x ", ptrig[i]); 
                            putBuf(prt,tBuf,0); 
                            if (j%10==0)
                                putBuf(prt,"\r\n",0); 
                            i++; j++;
                            }
                        //sprintf(tBuf,"Repeats %d times\r\n", param1); 
                        sprintf(tBuf,"\r\n"); 
                        putBuf(prt,tBuf,0); 
                        }
                    break;
                    }
                else if (!strcmp(tok, "PSEND"))                     
                    {
                    param1=arg_hex(&paramPtr,0x100);        //1st param (words to read)                    
                    //happybus command type for 'console display'
                    HappyBus.CmdType= eCMD71_CONSOLE;
                    //send command to feb on ePHY
                    sprintf(cmdBuf20,"RDX %X\r\n", param1); //use cmdBuf[20] note size
                    //PHY_LOADER_CONSOLE sets HappyBus.Active                  
                    PHY_LOADER_CONSOLE(cmdBuf20, prt, HappyBus.PoeBrdCh, eECHO_ON,0);  //send cmdBuf[20],poeprt,echoON,broadCast=1)
                    break;
                    }
                else if ((!strcmp(tok, "PRECALL")) || (!strcmp(tok, "PA"))) //PHY PORT Test, all ports data, format data
                    {
                    int i, pwr;

                    //ePHY Receive Buffers FPGA 2of4
                    for(i=1; i<9; i++)
                        {
                        pwr= POE_PORTS_ACTIVE[i];
                        if (pwr==0) continue;
                        sprintf(tBuf,"--PORT %d\r\n", i);    
                        putBuf(0, tBuf,0);
                        sprintf(tBuf,"PRECF %d", i);    
                        process(prt,tBuf); 
                        }

                    //ePHY Receive Buffers FPGA 3of4
                    for(i=9; i<17; i++)
                        {
                        pwr= POE_PORTS_ACTIVE[i];
                        if (pwr==0) continue;
                        sprintf(tBuf,"--PORT %d\r\n", i);    
                        putBuf(0, tBuf,0);
                        sprintf(tBuf,"PRECF %d", i);    
                        process(prt,tBuf); 
                        }

                    //ePHY Receive Buffers FPGA 4of4
                    for(i=17; i<25; i++)
                        {
                        pwr= POE_PORTS_ACTIVE[i];
                        if (pwr==0) continue;
                        sprintf(tBuf,"--PORT %d\r\n", i);    
                        putBuf(prt, tBuf,0);
                        sprintf(tBuf,"PRECF %d", i);            //format data out of buffer
                        process(prt,tBuf); 
                        }
                    break;
                    }
                else if (!strcmp(tok, "PREC"))                  //PHY PORT Testing, single ports daa
                    {
                    int d16, dat, cnt=0;
                    sPTR prtAddrS, prtAddrD;
                    param1=arg_dec(&paramPtr,HappyBus.PoeBrdCh);//get poe phy port number 1-24
                    param2=arg_hex(&paramPtr,4096);             //get count
                    param3=arg_dec(&paramPtr,8);                //words per line
                    if ((param1<1) || (param1>24))
                        { parErr++;  break; }
                    
                    prtAddrD= IOPs[param1].ePHY20_RECBUFp;
                    prtAddrS= IOPs[param1].ePHY16_STAp;
                                          
                    d16= *prtAddrS;
                    d16= d16 & PHYSTATUSBIT[param1];        //status bit for 1of8 ports
                    while(d16)
                        {
                        dat= *prtAddrD;                     //port addr for recd' data
                        sprintf(tBuf,"%04X ", dat); 
                        if (++cnt%param3==0)
                           strcat(tBuf,"\r\n");   
                        putBuf(prt,tBuf,0);                
                        if( --param2 ==0) break;
                        d16= *prtAddrS;
                        d16= (d16 & PHYSTATUSBIT[param1]);  //status bit for 1of8 ports
                        }
                    sprintf(tBuf,"WrdCnt=0x%X\r\n",cnt);
                    putBuf(prt,tBuf,0);                
                    break;
                    }
                else if (!strcmp(tok, "PRECTESTER"))            //PHY PORT Test Version for new ROCs loopback testing
                    {
                    int d16, dat, i=0, cnt=0, flag=0, ccr=0; 
                    sPTR prtAddrS, prtAddrD;
                    param1=arg_dec(&paramPtr,HappyBus.PoeBrdCh);//get poe phy port number 1-24
                    param2=arg_hex(&paramPtr,4096);             //get count
                    param3=arg_dec(&paramPtr,8);                //words per line
                    if ((param1<1) || (param1>24))
                        { parErr++;  break; }
                    
                    prtAddrD= IOPs[param1].ePHY20_RECBUFp;
                    prtAddrS= IOPs[param1].ePHY16_STAp;
                    //read port status for data avail.
                    d16= *prtAddrS;
                    d16= d16 & PHYSTATUSBIT[param1];        //status bit for 1of8 ports
                    
                    //zero out old data
                    for(i=0;i<128;i++)
                       phyBuf128i[i]=0;
                    i=0;

                    //wait for data in case for whatever reason data is late
                    if(d16==0)
                      uDelay(200);
                    //update status
                    d16= *prtAddrS;
                    d16= d16 & PHYSTATUSBIT[param1];        //status bit for 1of8 ports

                    if(d16==0) //no data
                        {
                        LED_RED1;  //RED ON
                        LED_GRN0;  //GRN OFF
                        }                            
                    
                    //Phy data here is raw from ROC fpga phy xmit, 
                    //phy rec'd data 1st byte taked by fpga seq logic, remaining words format reversed bits
                    while(d16)
                        {
                        if(i>128)                           //buffer overflow
                            i=128;
                        dat= *prtAddrD;                     //port addr for recd' data                      
                        phyBuf128i[i++]=dat&0xff;
                        phyBuf128i[i++]=dat>>8;
                        //dat= SwapBytes(dat);
                        //dat= revBits16(dat);
                        ccr += dat;                        
                        flag=1;                             //flag as some data was rec'd                         
                        if( --param2 ==0) break;                        
                        cnt++;                       
                          
                        d16= *prtAddrS;
                        d16= (d16 & PHYSTATUSBIT[param1]);  //status bit for 1of8 ports
                        }
                                       
                    ccr=  phyBuf128i[126]<<8;
                    ccr+= phyBuf128i[125];
                    if(global_pass<24)                       //clear initial connection errors
                        g_errorcnt=0;

                    if(flag==0)                             //no data recd
                        {
                        g_errNDR++;
                        g_PhyPrts--;                        //sub from total 24 ports being tested
                        LED_RED1;  //RED ON
                        LED_GRN0;  //GRN OFF
                        sprintf(tBuf,"\t\t\t\t\t\t\t\t\tPort#%02d, NoData\r\n",param1);
                        }
                    else
                        {
                        if (cnt < 128 )                    //data recd, check cnt
                          {
                          LED_RED1;  //RED ON
                          LED_GRN0;  //GRN OFF
                          g_errorcnt++;                          
                          }
                        else
                          {
                          LED_GRN1;  //GRN ON
                          LED_RED0;  //RED OFF
                          }
                          
                        sprintf(tBuf,"Prt_%02d  SN_%-2d   RecdWords=%d    Pass=%-4d  ChkSum=%04X    TotErrs=%d    NoDataPortsTotal=%d\r\n",param1, Ser_Cntrl_Numb.serNumb, cnt, global_pass, ccr, g_errorcnt, g_errNDR);
                        }
                    putBuf(prt,tBuf,0);                
                    break;
                    }
                else if (!strcmp(tok, "PRECF"))             //PHY PORT Testing
                    {
                    int d16, dat,uWCnt,uBHI,uBLO,uBSTA, cnt=0,cntD=0, cntTot=0,uB_Last=99, errs=0;
                    sPTR prtAddrS, prtAddrD;
                    param1=arg_dec(&paramPtr,HappyBus.PoeBrdCh);//get poe phy port number 1-24
                    param2=arg_hex(&paramPtr,4096);         //get count
                    param3=arg_dec(&paramPtr,8);            //words per line
                    
                    if ((param1<1) || (param1>24))
                        { parErr++;  break; }
                    
                    prtAddrD= IOPs[param1].ePHY20_RECBUFp;
                    prtAddrS= IOPs[param1].ePHY16_STAp;
                    d16= *prtAddrS;                   
                    d16= d16 & PHYSTATUSBIT[param1];        //status bit for 1of8 ports
                    while(d16)
                        { 
                        cntD= 0;
                        //read ubunch event header 4 words
                        uWCnt= *prtAddrD;           //word cnt
                        cntD++;                         
                        uBHI= *prtAddrD;            //ubun# high
                        cntD++;
                        uBLO= *prtAddrD;            //ubun# low
                        cntD++;                       
                        uBSTA= *prtAddrD;           //ubun status
                        cntD++;

                        if(uBSTA)
                          errs++;
                        
                        //using uB_Last as flag to not print with extra linefeed when ub hdr only
                        if((uB_Last==4) && (uWCnt<=4))
                            sprintf(Buf1500,"%04X %04X %04X %04X\r\n", uWCnt, uBHI, uBLO, uBSTA); 
                        else
                            sprintf(Buf1500,"\n%04X %04X %04X %04X\r\n", uWCnt, uBHI, uBLO, uBSTA); 

                        if (uWCnt>4)                //sub 4 word header if count valid
                            uWCnt-=4;
                        //read ubunch event data
                        while(cntD < uWCnt)
                            {
                            cntD++;
                            cnt++;
                            dat= *prtAddrD;                 //port addr for recd' data
                            sprintf(tBuf,"%04X ", dat); 
                            strcat(Buf1500, tBuf);   
                            if (cnt==param3)
                                {
                                strcat(Buf1500,"\r\n");   
                                cnt=0;
                                putBuf(prt,Buf1500,0);  
                                *Buf1500=0;
                                }
                            if( --param2 ==0) break;
                            d16= *prtAddrS;
                            d16= d16 &  PHYSTATUSBIT[param1];   //status bit for 1of8 ports
                            if(d16==0) break;
                            }
                        putBuf(prt,Buf1500,0); 
                        d16= *prtAddrS;
                        d16= d16 &  PHYSTATUSBIT[param1];       //status bit for 1of8 ports
                        uB_Last= cntD;
                        cntTot+= cntD;
                        if(cntTot>4095)
                            {
                            sprintf(tBuf,"\r\nPhyRx_OverFlow,  Cnt=%d\r\n", cntTot);
                            putBuf(prt,tBuf,0); 
                            break;
                            }
                        }
                    if(cntTot)
                        {
                        sprintf(Buf1500,"\r\nReg%3X=%X  RecTot=%d(D)\r\n",((int)prtAddrS&0xffff)/2, d16, cntTot);
                        //if (errs)
                        //    {
                        //    sprintf(tBuf,"FEB Status Word Bits: 8=Emp 7654=OverFlow 3210=Error\r\n");
                        //    strcat(Buf1500, tBuf);
                        //    }
                        putBuf(prt,Buf1500,0);                         
                        }
                    else
                        putBuf(prt," Empty\r\n",8); 
                    break;
                    } 
                else if (!strcmp(tok, "PFM"))               //LVDS PORT Testing
                    {
                    int d16, dat,i=0;
                    sPTR prtAddrS, prtAddrD;
                    param1=arg_dec(&paramPtr,1);            //get LVDS port DATA
                    param2=arg_dec(&paramPtr,4096);         //get count                    
                    if ((param1<1) || (param1>24))
                        { parErr++;  break; }

                    prtAddrD= IOPs[param1].ePHY20_RECBUFp;
                    prtAddrS= IOPs[param1].ePHY16_STAp;
                    
                    d16= *prtAddrS;
                    d16= d16 & PHYSTATUSBIT[param1];            //status bit for 1of8 ports
                    sprintf(tBuf,"RxSTAT=%x\r\n",d16);
                    putBuf(prt,tBuf,0);                
                    param3=0;
                    while(d16==0)
                        {
                        dat= *prtAddrD;                         //port addr for recd' data
                        sprintf(tBuf,"%4X ", dat); 
                        if (++i%8==0)
                           strcat(tBuf,"\r\n");   
                        putBuf(prt,tBuf,0);                
                        if( --param2 ==0) break;
                        if(param3++> (4096*8)) break;           //limit dump size
                        d16= *prtAddrS;
                        d16= d16 &  PHYSTATUSBIT[param1];       //status bit for 1of8 ports
                        }
                //h_TP48_LO
                    sprintf(tBuf,"\r\nRxSTAT=%x  RecWrdCnt=0x%X (%0d)\r\n", d16, i,i);
                    putBuf(prt,tBuf,0);                
                    break;
                    }                
                
                else  {parErr=0xf;   break; }
        case 'Q':
                if (!strcmp(tok, "Q"))                      //'QUIT' close this active ethernet socket
                   {
                    param2= g_Sock;                         //quit this socket
                    if ((param2>=0) && (param2<4))          //valid sockets 0-3
                        {
                        //socket_close(param2);             //close connection
                        REG16(ZestETM1+(param2*CHANNEL_SPACING)+zIntrEna) = IE_STATE;   //state change intr enable'
                        REG16(ZestETM1+(param2*CHANNEL_SPACING)+zConnectCSR) = CONN_TCP;//ch0 only, 'reset channel'
                          
                        //sockets reset and enable intrs
                        Sockets[param2].IE = 0;
                        Sockets[param2].State = 0;
                        REG16(ZestETM1+(param2*CHANNEL_SPACING)+zIntrEna) = Sockets[param2].IE;
                        REG16(ZestETM1+(param2*CHANNEL_SPACING)+zConnectCSR) = Sockets[param2].State;

                        if (Sockets[param2].Closing)
                          Sockets[param2].Used = 0;
                        mDelay(1);
                        u_16Bit *uPtr= &netInfo.sTelnet0 + param2; 
                        tcp_server_init(param2, *uPtr, eRecDatBuf[param2] );  //(sock,port)                     
                      }
                     break;
                    }
                else  {parErr=0xf;   break; }
        case 'R':          
                if (!strcmp(tok, "RD2"))                     //16 bit EPI BUSS FPGA READ
                    {
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//now get 1st param
                        { parErr++;  break; }
                    //address only, no board number
                    param1 &= 0xFFFF;
                    sPTR saddr = (sPTR) fpgaBase0;
                    saddr += param1;                        //adding to pointer, incr by ptr size
                    g_dat16= *saddr;

                    // send DCS package
                    
                    sPTR saddr2 = (sPTR) fpgaBase0;
                    saddr2 += 0x1A;                        //generate preamble
                    *saddr2 = 0x4;
                    saddr2 += 2;
                    *saddr2 = 0x10;
                    *saddr2 = 0x8040;
                    *saddr2 = 0x0000;
                    *saddr2 = param1;
                    *saddr2 = g_dat16;
                    *saddr2 = 0x0000;
                    *saddr2 = 0x0000;
                    *saddr2 = 0x0000;
                    saddr2 += 2;
                    *saddr2 = 0x1;
                    
                    sprintf(Buf1500,"%04X\r\n",g_dat16);
                    putBuf(prt, Buf1500,0);                 //send to current active port
                    break;
                    }
          
                else if (!strcmp(tok, "RD"))                     //16 bit EPI BUSS FPGA READ
                    {
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//now get 1st param
                        { parErr++;  break; }
                    //address only, no board number
                    param1 &= 0xFFFF;
                    sPTR saddr = (sPTR) fpgaBase0;
                    saddr += param1;                        //adding to pointer, incr by ptr size
                    g_dat16= *saddr;

                    sprintf(Buf1500,"%04X\r\n",g_dat16);
                    putBuf(prt, Buf1500,0);                 //send to current active port
                    break;
                    }                
                else if (!strcmp(tok, "RDBR"))              //read bin data, User Names Register
                    {
                    sPTR saddr = (sPTR) fpgaBase0;
                    //address
                    param1= arg_hex(&paramPtr,0);          
                    if (param1> 0x1000)                     //out of range
                        {BinSt.gSndWordCnt=0;  break;}      //addr error
                    if (param1==0)                          //stop any active xfer
                        {BinSt.gSndWordCnt=0; break;}       //stops any active background xfers
                  //BinSt.gSndSrc= (lPTR)&f1_RD16SWP;       //uC DDR ram data reg FPGA1
                  //BinSt.gSndSrc= (lPTR)&sTST_CNT_LO;      //32 bit test counter LOW 0x35
                    saddr += param1;
                    BinSt.gSndSrc= (lPTR)saddr;  //uC DDR ram data reg
                    
                    //count
                    param2= arg_hex(&paramPtr,10);          //now get 2nd param, word count
                    if (param1> 0x10000000)                 //limit word count
                        param1= 0x10000000;
                    BinSt.gSndWordCnt= param2;
                    
                    //test mode
                    param3= arg_dec(&paramPtr,0);          
                    if (param3==123)
                        BinSt.gTestMode=1;
                    else
                        BinSt.gTestMode=0;

                    //after 1st packet, background loop will send remaining data using non-zero 'gSndBytCnt'
                    mStime.gBusy_mSec=0;
                    BinSt.gSndPrt= prt;
                    g_wait=0;                               //testing delay counter
                    mStime.g_SockMs=0;
                    sendBin(prt);
                    break;
                    }
                else if (!strcmp(tok, "RDI"))               //read 32bit data from fpga address and increment
                    {
                    sPTR saddr= f_Addr_CSR;
                    int i;
                   //get 1st param now
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//now get 1st param, rd addr hex
                        { param1= 0;}                       //default start addr 0
                    //address only, no board number
                    param1 &= 0x7FFFF;
                    if ( (param2=arg_dec(&paramPtr,-1))==-1)//now get read cnt dec
                        { param2= 16;}                      //cnt per line
                    if (param2 > 256)                       //limit size less than Buf1500;
                        param2= 256;                        //5 chars per word, uses 1280 bytes (+crlf overhead)

                    *Buf1500=0;
                    saddr +=param1;
                    for (i=1; i<param2+1; i++)              //Note: Buf1500 is 1500 bytes
                        {
                        if (i%16==0)                        //14 words per line, now 16
                            {
                            sprintf (tBuf,"%4X\r\n", *saddr++);
                            strcat(Buf1500, tBuf);
                            putBuf(prt, Buf1500,0);          
                            *Buf1500=0;                     //set string size ==0
                            }
                        else
                            {
                            sprintf (tBuf,"%4X ", *saddr++);
                            strcat(Buf1500, tBuf);
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
                else if (!strcmp(tok, "RFI"))               //read 32bit data from parallel FLASH addr and incre
                    {
                    sPTR saddr= pFLASHbase;
                    int d16;
                    //get 1st param now
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//now get 1st param, rd addr hex
                        { param1= 0;}                       //default start addr 1
                    if ( (param2=arg_hex(&paramPtr,-1))==-1)//now get read cnt dec
                        { param2= 256;}                     
                    //address only, no board number
                    if (param1>=0x7FFFFF)                   //max addr else code abort
                        {
                        sprintf (tBuf,"End of FLASH");
                        putBuf(prt, tBuf,0);                 
                        break;
                        }

                    //User enters BYTE ADDR, change to word addr
                    param1 = param1/2;
                    
                    *Buf1500=0;
                    saddr +=param1;
                    for (int i=1; i<param2+1; i++)          //Note: Buf1500 is 1500 bytes
                      {
                       if (saddr>(pFLASHbase+0x3fffff))    //max buss addr else code abort
                        {
                        putBuf(prt, Buf1500,0);             //Note:putBuf to hBus will change fpga ptrs
                        sprintf (tBuf,"\r\nEnd of FLASH\r\n");
                        putBuf(prt, tBuf,0);                 
                        break;
                        }                        
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
                    putBuf(prt,"\r\n",2);
                    break;
                    }
                else if (!strcmp(tok, "RDM"))                    //read 32bit data from fpga address
                    {
                    //rdm mode is best for smaller xfers as it does NOT support
                    //background xmits and will tie up all ports until finished
                    int index;
                    //get 1st param now
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//now get 1st param, lword read addr hex
                        { parErr++;  break; }
                    //address only, no board number
                    param1 &= 0x7FFFF;

                    if ( (param2=arg_dec(&paramPtr,-1))==-1)//get 2nd param, lword read cnt hex
                        { parErr++;  break; }
                    param3= arg_dec(&paramPtr, 8);          //get 3rd param, lword read cnt hex
                    if ((param3<1) || (param3>16))          //limit sizes
                        param3=8;
                    sPTR saddr = (sPTR) fpgaBase0;
                    saddr += param1;                        //adding to pointer, incr by ptr size
                    *Buf1500=0;
                    for (int i=1; i<param2+1; i++)          //Note: Buf1500 is 1500 bytes
                      {
                       if (param1 >= 0x3ffff0)              //max bus addr else code abort
                        {
                        putBuf(prt, Buf1500,0);             //Note:putBuf to hBus will change fpga ptrs
                        sprintf (tBuf,"\r\nEnd of FLASH\r\n");
                        putBuf(prt, tBuf,0);                 
                        break;
                        }                        
                       saddr = (sPTR)fpgaBase0+ param1;
                       index= *saddr;

                       sprintf (tBuf,"%4x ",index);
                       strcat(Buf1500, tBuf);
                       //read lower addr to advance pointer
                       if ((i%param3==0) || (i==param2))    //print if 'eol limit' or 'max count'
                           {
                           strcat(Buf1500,"\r\n");
                           putBuf(prt, Buf1500,0);          //Note:putBuf to hBus will change fpga ptrs
                           *Buf1500=0;                      //set string size ==0
                           uDelay(1000);
                           }
                      }
                    break;
                    }
                else if (!strcmp(tok, "RDF"))               //rec data file, store in FPGA_1 SDRAM
                   {
                    uint8_t d8;
                    uint16 key, d16;
                    int len, count=0, sum=0;
                    extern int g_Done, g_Cnt;
                    char* eBufB= (char*)eRecDatBuf[g_Sock];

                    //use fpga sdRam 2,3 or 4
                    if ( (param1=arg_dec(&paramPtr,-1))==-1)//now get 1st param
                        { parErr++;  break; }
                    if ((param1>4) || (param1<2))
                        { parErr++;  break; }
                               
                    //fpga1 upper level fpga, has no sdRam
                    //set write address for sdRam(s) 1,2,3 in actual fpga(2,3,4) 
                    if (param1==2) 
                        {
                        REG16((&SDramWrHI)+fPtrOffset2)=0;
                        REG16((&SDramWrLO)+fPtrOffset2)=0;
                        }
                    else if (param1==3) 
                        {
                        REG16((&SDramWrHI)+fPtrOffset3)=0;
                        REG16((&SDramWrLO)+fPtrOffset3)=0;
                        }
                    else if (param1==4) 
                        {
                        REG16((&SDramWrHI)+fPtrOffset4)=0;
                        REG16((&SDramWrLO)+fPtrOffset4)=0;
                        }
                    if (prt==tty)
                        {
                        sprintf(tBuf,"SDram: waiting for data file (45seconds) ...\r\n");
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
                            if (param1==2) 
                                REG16((&SDR_RD16)+fPtrOffset2)= d16; //store to sdRam2
                            else if (param1==3) 
                                REG16((&SDR_RD16)+fPtrOffset3)= d16; //store to sdRam3
                            else if (param1==4) 
                                REG16((&SDR_RD16)+fPtrOffset4)= d16; //store to sdRam4
                           }
                        }
                    else
                        {
                        sprintf(tBuf,"SDram: waiting for data file (45seconds)...  (uTelnet use key '|')\r\n");
                        putBuf(prt, tBuf,0);
                        //wait for data
                        len= SockKeyWait(45000, prt, &key); //CharCnt=SockKeyWait(mSecWait, Port#, 1st Char in RecBuf)
                        if(len==0)
                            {
                            sprintf(tBuf,"uC_Dram: Len=0, 'RDF' wait on file 45 Second timeout\r\n");
                            putBuf(prt, tBuf,0);
                            break;
                            }
                        while(len>0)
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
                                eBufB= (char*)eRecDatBuf[g_Sock];
                                d8= *eBufB++;   //store 2nd of two words
                                sum += d16;
                                sum += d8;
                                d16= (d16<<8)+d8;
                                //write fpga2 
                                if (param1==2)      REG16((&SDR_RD16)+fPtrOffset2)= d16; //store to sdRam1
                                //write fpga3
                                else if (param1==3) REG16((&SDR_RD16)+fPtrOffset3)= d16; //store to sdRam2
                                //write fpga4
                                else if (param1==4) REG16((&SDR_RD16)+fPtrOffset4)= d16; //store to sdRam3
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
                                if (param1==2)      REG16((&SDR_RD16)+fPtrOffset2)= d16; //store to sdRam1
                                //write fpga3
                                else if (param1==3) REG16((&SDR_RD16)+fPtrOffset3)= d16; //store to sdRam2
                                //write fpga4
                                else if (param1==4) REG16((&SDR_RD16)+fPtrOffset4)= d16; //store to sdRam3
                                count+=2;
                                len -=2;
                                }
                            if(len==0)
                                {
                                len= SockKeyWait(500, prt, &key); //CharCnt=SockKeyWait(mSecWait, Port#, 1st Char in RecBuf)
                                //eBufW= (uSHT*)eRecDatBuf[g_Sock];
                                eBufB= (char*)eRecDatBuf[g_Sock];
                                }
                            }
                        u_SRAM.DwnLd_sCNT= count;
                        u_SRAM.DwnLd_sSUM= sum&0xffff;
                        }
                    //send extra bytes to force SDram Buffer Burst Write Cycle
                    for(int i=0; i<32; i++)
                        {
                        if (param1==2)  
                            {
                            REG16((&SDR_RD16)+fPtrOffset2)= 0x1234;
                            if ((REG16((sPTR)fpgaBase1+fPtrSDramCnt) & 0xff)==0) //check burst fifo
                                break;
                            }
                        if (param1==3)  
                            {
                            REG16((&SDR_RD16)+fPtrOffset3)= 0x1234;
                            if ((REG16((sPTR)fpgaBase2+fPtrSDramCnt) & 0xff)==0) //check burst fifo
                                break;
                            }
                        if (param1==4)  
                            {
                            REG16((&SDR_RD16)+fPtrOffset4)= 0x1234;
                            if ((REG16((sPTR)fpgaBase3+fPtrSDramCnt) & 0xff)==0) //check burst fifo
                                break;
                            }
                        }
                    //allow writes to finish
                    uDelay(1);

                    //set read addr to base, doesnt always change
                    if (param1==2)                           
                        {
                        REG16((&SDramRdHI)+fPtrOffset2)=0;
                        REG16((&SDramRdLO)+fPtrOffset2)=0;
                        }
                    else if (param1==3) 
                        {
                        REG16((&SDramRdHI)+fPtrOffset3)=0;
                        REG16((&SDramRdLO)+fPtrOffset3)=0;
                        }
                    else if (param1==4) 
                        {
                        REG16((&SDramRdHI)+fPtrOffset4)=0;
                        REG16((&SDramRdLO)+fPtrOffset4)=0;
                        }
                    sprintf(tBuf,"FEBCNTL: Recd %d Bytes, ChkSum=0x%4X\r\n", count,sum&0xffff);
                    putBuf(prt, tBuf,0);
                    break;
                    } 
                //else if (!strcmp(tok, "RD0"))               //system status registers
                //    {
                //    sprintf(Buf1500,"%08X %08X %08X %08X\r\n",systemREG2->DIEIDL_REG0,systemREG2->DIEIDH_REG1,systemREG2->DIEIDL_REG2,systemREG2->DIEIDH_REG3);
                //    putBuf(prt, Buf1500,0);                 //send to current active port
                //    break;
                //    }                
                else if (!strcmp(tok, "RESET"))                    
                    {
                    //allow last usb chars to xmit
                    uDelay(1000);
                    _disable_interrupt_();
                    resetEntry();
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'S':
               if  (!strcmp(tok, "SB"))                     //set usb/tty baud rate
                    {
                    /** - set baudrate */
                    sciSetBaudrate(UART, 921600) ;                                          
                     break;
                    }
               
               //controller 'stab' command not needed, use command 'pool' to see data
               else if ((!strcmp(tok, "STB")) || (!strcmp(tok, "STAB"))) //read returned status block
                    {
                    putBuf(prt, "Old command, use 'POOL'\r\n",0); //leave this for now...
                    break;
                    }
               else if (!strcmp(tok, "SOCK"))                   //socket status reg 0x403,0x503,...
                    {
                    int stat, sReg, sReg1,rAddr;
                    sPTR saddr;
                    sprintf (Buf1500,"Orange Tree ZestETM1 Status, %d Sockets Enabled\r\n",NUM_SOCKETS);
                    putBuf(prt, Buf1500,0);                     //send to current active port

                    for (int s=0; s<NUM_SOCKETS; s++)           //8 sockets, only show how many are setup (3 ftp, 1 udp)
                        {
                        sReg1= REG16(ZestETM1+(s*CHANNEL_SPACING)+zConnectCSR); //CHANNEL_SPACING==16words
                        sReg= sReg1 & 0xf;
                        stat= Sockets[s].State & 0xf;
                        rAddr= Sockets[s].RemoteAddr;
                        if (stat != sReg)
                          Sockets[s].State= sReg;
                        if (sReg == LISTEN)
                            sprintf (Buf1500,"Socket%-2d   StatRegC= %-2X   LISTEN\r\n",s, sReg1);
                        else if (sReg == ESTABLISHED)
                          sprintf (Buf1500,"Socket%-2d   StatRegC= %-2X   ESTABLISHED  %03u.%03u.%03u.%03u :%u\r\n", s, sReg1, 
                                ((rAddr>>24)&0xff),(rAddr>>16&0xff),(rAddr>>8&0xff),(rAddr&0xff), //IP addr breakdown
                                Sockets[s].RemotePort);                                           //port number
                        else if (sReg == CLOSED)
                            sprintf (Buf1500,"Socket%-2d   StatRegC= %-2X   CLOSED\r\n",s, sReg1);
                        else 
                            sprintf (Buf1500,"Socket%-2d   StatRegC= %-2X\r\n",s, sReg1);
                        putBuf(prt, Buf1500,0);                     //send to current active port
                        }
                    
                    sprintf (Buf1500,"Intr Count  = %d\r\n",g_IntrCnt);  //interrupt entry counter
                    putBuf(prt, Buf1500,0);                     //send to current active port
                    
                    //added for testing conditions of scanning apps hanging all otree sockets
                    sprintf (Buf1500,"Port Scans  = %u\r\n",soSt.badIP);    //bad ip counter
                    putBuf(prt, Buf1500,0);                                 //send to current active port
                    sprintf (Buf1500,"Pack OvrFlow= %u\r\n",soSt.badIP_OVR);//bad ip, packet data overflow
                    putBuf(prt, Buf1500,0);                                 //send to current active port
                    sprintf (Buf1500,"Sock ConnCnt= %u\r\n",soSt.conCnt);   //socket connection counter
                    putBuf(prt, Buf1500,0);                                 //send to current active port

                    if (g_IntrCnt>1000) g_IntrCnt=0;
                    break;
                    }
                else if (!strcmp(tok, "SET"))
                    {
                    int i;
                    if ( (param1=arg_dec(&paramPtr,-1)) ==-1)   //now get 2nd param, option select
                        {
                        dispNet(prt);                           //format network data to ascii
                        break;
                        }
                    unsigned char *shtPtr;                      //load ram stucture with Atmel Flash data page IDINFO_PAGE
                    if      (param1==1) shtPtr = (uCHR*)&netInfo.ipAddr;
                    else if (param1==2) shtPtr = (uCHR*)&netInfo.gateWay;
                    else if (param1==3) shtPtr = (uCHR*)&netInfo.netMask;
                    else if (param1==4) shtPtr = (uCHR*)&netInfo.sTelnet0;
                    else if (param1==5) shtPtr = (uCHR*)&netInfo.sTelnet1;
                    else if (param1==6) shtPtr = (uCHR*)&netInfo.sTelnet2;
                    else if (param1==7) shtPtr = (uCHR*)&netInfo.sTelnet3;
                    else if (param1==8) shtPtr = (uCHR*)&netInfo.Spare1;
                    else break;
                    if (param1<4)
                        {
                         i=0;
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
                           param2= arg_dec(&paramPtr,-1);  //now get params dec
                           if (param2<0)
                               { parErr++; break; }
                           *shtPtr++ = (uCHR)param2;
                           }
                         //flag as netInfo change for OrangeTree Update
                         netInfo.UpDate=1; 
                         putBuf(prt,"    'OTree will show new data after 'NETSAV' and cable connection\r\n",0);

                         }
                    else
                         {
                         param2 = arg_dec(&paramPtr,-1);        //now get params dec
                         *(uLNG*)shtPtr++ = param2;
                         putBuf(prt,"\r\n",2);                  //send to current active port
                         }
                    break;
                    }
                else if (!strcmp(tok, "SN"))                    //set serial number
                    {
                    param1=arg_dec(&paramPtr,-1);               //get 1st param, password
                    param2=arg_dec(&paramPtr,-1);               //get 2nd param serial #
                    param3=arg_dec(&paramPtr,1);                //get 3rd param controller #
                    //note, struct 'serNUMB_MISC' filled from FRAM on power up
                    if ( (param1==123) && (param2!=-1) )        //simple password followed by sn
                        {
                        Ser_Cntrl_Numb.serNumb= param2;
                        Ser_Cntrl_Numb.CntrlNumb= param3;
                        FRAM_WR_ENABLE();                      //WREN op-code issued prior to Wr_Op
                        //update serial number only
                        FRAM_WR(fram_serNUMB, (uint8*)&Ser_Cntrl_Numb.serNumb, 4); //addr,data,cnt (serNumb 1st in struct)
                        FRAM_WR_DISABLE();
                        }
                    else
                        {
                          sprintf(tBuf,"Serial Numb : %d\r\nControl Numb: %d\r\n",Ser_Cntrl_Numb.serNumb, Ser_Cntrl_Numb.CntrlNumb);
                        putBuf(prt, tBuf,0);                        //send to current active port
                        }
                    break;
                    }
                else if (!strcmp(tok, "SAVE"))                  //FLASH-INFO-SAVE user parameters
                    {
                    //reload defaults from FRAM
                    FRAM_WR_ENABLE();                           //WREN op-code, issued prior to Wr_Op
                    //store complete structure to fram
                    FRAM_WR(fram_serNUMB, (uint8*)&Ser_Cntrl_Numb, Ser_Cntrl_ByteSz); //addr,data,cnt
                    FRAM_WR_DISABLE();
                    putBuf(prt,"FRAM NOT UPDATED\r\n",0);       //send to current active port
                    break;
                    }
                else if ((!strcmp(tok, "S4")) || (!strcmp(tok, "SD")) )//TEST code fpga read/writes
                    {                                          
                    //MT46H64M16LF – 16 Meg x 16 x 4 Banks
                    //testing 16 Meg x 16 x 4 Banks ==  0x1ff ffff long words
                    param1=arg_dec(&paramPtr,1);                //get 1st param, FPGAs 2,3,4
                    
                    if (param1<2) param1=2;
                    if (param1>4) param1=2;
                    param2=arg_hex(&paramPtr,0x3fffff0);        //now get 2nd param
                    sprintf(Buf1500, "Max Words=0x3fffff0,   MT46H64M16LF 16Meg x 16 x 4 Banks\r\n");
                    putBuf(prt, Buf1500,0);                     //send to current active port
                    sdRamTest(param1, param2, prt);             //chip(fpga) 1-4
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'T': 
                if (!strcmp(tok, "TH"))
                    {   
                    float tempF;
                    tempF= readTemperature();
                    sprintf(tBuf,"TempF=%2.1f\r\n", tempF);
                    putBuf(prt,tBuf,0);                 //send it
                    break;
                    }
                if (!strcmp(tok, "TRIG_OLD"))           //Check fiber input uBunch reqs, mode 0 uses standard 64 byte Phy packet reqs to FEB
                    {   
                    param1=arg_dec(&paramPtr,4);        //get 1st param
                    if(param1==1)
                        {
                        uBReq.Flag |= DAQuB_Trig_OLD;   //aq uBunch decode On
                        uBReq.Flag &=~DAQuB_Trig_NEW;   //if 0 on, set mode 1 off
                        }
                    else if (param1==0)
                        {
                        uBReq.Flag &= ~DAQuB_Trig_OLD; //daq uBunch decode Off
                        }
                    
                    if (uBReq.Flag & DAQuB_Trig_OLD)   //set = on acive
                        sprintf(tBuf,"uB Trigs ON\r\n");
                    else
                        sprintf(tBuf,"uB Trigs OFF\r\n");
                    putBuf(prt, tBuf,0);
                    break;
                    }
                else if (!strcmp(tok, "TRIG"))         //Check fiber input uBunch reqs, mode 1 uses short Phy packet reqs to FEB
                    {   
                    param1=arg_dec(&paramPtr,1);        //get 1st param                    

                    if(param1==11)
                        {
                       param2=arg_dec(&paramPtr,1);     //get 2nd param all trigs via ptrig0,1 modes
                        //tek dec 2022 temp test mode
                        uBReq.ReqCnt= param2;
                        uBReq.SmPacMode=1;                    
                        uBReq.Flag |= DAQuB_Trig_NEW;   //aq uBunch decode On
                        uBReq.Flag &=~DAQuB_Trig_OLD;   //if 1 on, set mode 0 off
                        }
                    
                    else if(param1==1)                   
                  //if(param1==1)
                        {
                        uBReq.Flag |= DAQuB_Trig_NEW;   //aq uBunch decode On
                        uBReq.Flag &=~DAQuB_Trig_OLD;   //if 1 on, set mode 0 off
                        }
                    else if (param1==0)
                        {
                        uBReq.Flag &= ~DAQuB_Trig_NEW;  //daq uBunch decode Off
                        }
                    
                    if (uBReq.Flag & DAQuB_Trig_NEW)    //set = on acive
                        sprintf(tBuf,"uB Trigs (Short Pacs)ON\r\n");
                    else
                        sprintf(tBuf,"uB Trigs (Short Pacs)OFF\r\n");
                    putBuf(prt, tBuf,0);
                    break;
                    }
                
                else if (!strcmp(tok, "TESTINIT"))         //Tester Code, Init FEB/ROC in test mode
                    {                       
                    param1=arg_dec(&paramPtr,1);            //1 = INIT ROC, 2=INIT ROC AND FEB(s)  
                    //ROC INIT
                    //	2 SECOND GATES internal gate
                    wr16FPGA(0x00, 10);
                    wr16FPGA(0x30, 2);
                    wr16FPGA(0x31, 2);
                    wr16FPGA(0x3a, 0);

                    wr16FPGA(0x3b, 0);
                    wr16FPGA(0x43, 1);
                    wr16FPGA(0x00, 0x110);
                    wr16FPGA(0x400, 8);
                    wr16FPGA(0x800, 8);
                    wr16FPGA(0xC00, 8);

                    sprintf(tBuf,"ROC Initialized in WBL Test Mode\r\n");
                    putBuf(prt, tBuf,0);

                    if(param1==1)
                      break;
                    
                    //Init all active POE-FEB ports  
                    int pBits= ( (ACT_PORTS_HI<<16)+ ACT_PORTS_LO);
                    for(int i=1,j=0; i<25; i++,j++)
                    {
                    if((pBits&(1<<j))==0)
                            continue;
                    //FEB INIT 
                    //Link command version
                    sprintf(tBuf,"LCA TRIG 0");  process(prt, tBuf);
                    mDelay(10);
                    sprintf(tBuf,"LCA WR 306 0");  process(prt, tBuf);
                    mDelay(10);
                    sprintf(tBuf,"LCA WR 307 0");  process(prt, tBuf);
                    mDelay(10);
                    sprintf(tBuf,"LCA WR 304 e");  process(prt, tBuf);
                    mDelay(10);
                    sprintf(tBuf,"LCA WR 305 20"); process(prt, tBuf);
                    mDelay(10);
                    sprintf(tBuf,"LCA WR 317 3");  process(prt, tBuf);
                    mDelay(10);
                    sprintf(tBuf,"LCA WRI 030 800 8");  process(prt, tBuf);
                    mDelay(100);
                    sprintf(tBuf,"LCA WRI 430 800 8");  process(prt, tBuf);
                    mDelay(100);
                    sprintf(tBuf,"LCA WRI 830 800 8");  process(prt, tBuf);
                    mDelay(100);
                    sprintf(tBuf,"LCA WRI C30 800 8");  process(prt, tBuf);
                    mDelay(100);
                                      
                    sprintf(tBuf,"LCA AFEWR 1 102 A000");  process(prt, tBuf);
                    mDelay(10);
                    sprintf(tBuf,"LCA AFEWR 1 202 A000");  process(prt, tBuf);
                    mDelay(10);
                    sprintf(tBuf,"LCA AFEWR 2 102 A000");  process(prt, tBuf);
                    mDelay(10);
                    sprintf(tBuf,"LCA AFEWR 2 202 A000");  process(prt, tBuf);
                    mDelay(10);
                    sprintf(tBuf,"LCA AFEWR 3 102 A000");  process(prt, tBuf);
                    mDelay(10);
                    sprintf(tBuf,"LCA AFEWR 3 202 A000");  process(prt, tBuf);
                    mDelay(10);
                    sprintf(tBuf,"LCA AFEWR 4 102 A000");  process(prt, tBuf);
                    mDelay(10);
                    sprintf(tBuf,"LCA AFEWR 5 202 A000");  process(prt, tBuf);
                    mDelay(10);
                    }
                    lvLnk.PoolMode=0;                   //data pool off  
                    
                    sprintf(tBuf,"ROC-FEB Initialized in WBL Test Mode\r\n");
                    putBuf(prt, tBuf,0);
                    break;                     
                    }
                
                else if (!strcmp(tok, "TESTRAM"))      //Tester Code, Standard sdRAM Test 3 fpgas attached RAM
                    {
                    lvLnk.PoolMode=0;                   //data pool off  
                    iFlag |= iNoPrompt;                 //flag as no prompt on terminal 
                    sprintf(tBuf,"Busy ~45 Seconds Per FPGA\r\n");
                    putBuf(prt, tBuf,0);
                    putBuf(prt, "\r\nFPAG2 sdRam Test\r\n",0);
                    sprintf(tBuf,"SD 2");               //sdRAM2 Testing
                    process(prt, tBuf);                 //process command                                   
                    iFlag |= iNoPrompt;                 //flag as no prompt on terminal 
                    putBuf(prt, "\r\nFPAG3 sdRam Test\r\n",0);
                    sprintf(tBuf,"SD 3");               //sdRAM3 Testing
                    process(prt, tBuf);                 //process command                                   
                    iFlag |= iNoPrompt;                 //flag as no prompt on terminal 
                    putBuf(prt, "\r\nFPAG4 sdRam Test\r\n",0);
                    sprintf(tBuf,"SD 4");               //sdRAM4 Testing
                    process(prt, tBuf);                 //process command                                   
                    iFlag |= iNoPrompt;                 //flag as no prompt on terminal 
                    sprintf(tBuf,"SDRAM Finished\n");
                    putBuf(prt, tBuf,0);
                    break; 
                    }                    
                else if (!strcmp(tok, "TESTFIBER"))     //Tester Code, FIBER XMIT/REC
                    {
                    uint16_t len=0, key, cnt1=1,cnt2;
                    
                    sprintf(tBuf,"FT");                 //reset/reload FPGAs
                    process(prt, tBuf);                 //process command           
                    
                    sprintf(tBuf,"ROC FIBER LINK Testing\r\n");
                    putBuf(prt, tBuf,0);
                    lvLnk.PoolMode=0;                   //data pool off 

                    sprintf(tBuf,"\r\nReset FPGAs Done First\r\n");
                    putBuf(prt, tBuf,0);                    
                    sprintf(tBuf,"Option: TESTFIBER 1 c  (1=TransmitMode,  'c=Ch' 0=GTP0(Def), 1=GTP1, 3=Both(Def))\r\n");
                    putBuf(prt, tBuf,0);
                    sprintf(tBuf,"Option: TESTFIBER 2    (2=Receives Mode(Default), All Channels)\r\n\n");
                    putBuf(prt, tBuf,0);
                    param1=arg_dec(&paramPtr,2);            //now get 1st param, Test Mode, 1=Rec=ROC2(DUT,Default) or 2=Xmit=ROC1
                    param2=arg_dec(&paramPtr,3);            //now get 2nd param, xmit chan 0,1,3
 
                    if(param1==2)
                      sprintf(tBuf,"FIBER Receive Mode\r\n");
                    else if(param1==1)
                      sprintf(tBuf,"FIBER Xmit Mode\r\n");
                    else
                      {parErr++;  break; }
                    putBuf(prt, tBuf,0);
                    mDelay(1000);                           //show mode timeout before test data 
                    iFlag |= iNoPrompt;                     //flag as no prompt on terminal 
                  //wr16FPGA(0x02, BIT0);                   //GTP Rx fifo reset
                      
                    for (int l=0;l<4096; l++)   //empty fifos
                        {
                        rd16FPGA(0x20);         //EGPT0 FIFO
                        rd16FPGA(0x21);         //GPT1 FIFO
                        }
                    
                    while(1)
                    {                      
                    if (param1==1 && len==0)                //transmit mode
                        {
                        if(param2==0 || param2==3)
                        {
                        putBuf(prt,"Transmit GTP0\r\n",0);
                        iFlag |= iNoPrompt;                 //flag as no prompt on terminal 
                        sprintf(tBuf,"PTRIG0 %d 0",cnt1);   //XMIT on FIBER CH0 1 MULTI PACKET SEQ, Turn off local echo
                        process(prt, tBuf);                 //process command           
                        }
                        if(param2==1 || param2==3)
                        {
                        putBuf(prt,"Transmit GTP1\r\n",0);
                        iFlag |= iNoPrompt;                 //flag as no prompt on terminal 
                        sprintf(tBuf,"PTRIG1 %d 0",cnt1);   //XMIT on FIBER CH1 1 MULTI PACKET SEQ, Turn off local echo
                        process(prt, tBuf);                 //process command               
                        }
                        
                        len= SockKeyWait(0, prt, &key);     //CharCnt=SockKeyWait(mSecWait, Port#, Rtn 1st Ch RecBuf)
                        if(len)
                            {
                            iFlag |= iNoPrompt;             //flag as no prompt on terminal 
                            sprintf(tBuf,"FIBER Transmit Mode Exit\r\n");
                            putBuf(prt, tBuf,0);
                            break;
                            }                        
                        mDelay(1000);  
                        }
                    else if (param1==2 && len==0)   //receive mode
                        {
                        //Note: WBL code in FPGAs has no buffer word count register so we loop on xmit count 
                        iFlag |= iNoPrompt;                 //flag as no prompt on terminal
                        cnt2=0;
                        while(!(rd16FPGA(0x02) & BIT2))                 //GTP FIFO 0, 1==empty
                            {
                            iFlag |= iNoPrompt;                 //flag as no prompt on terminal
                            sprintf(tBuf,"GTP0  ");    
                            putBuf(prt,tBuf,0);
                            sprintf(tBuf,"RDM 20 %d 10", 10);   //Rec on FIBER CH0, 1 MULTI PACKET SEQ
                            process(prt, tBuf);              //process command           
                            if(cnt2++>800) break;
                            }
                        cnt2=0;
                        while(!(rd16FPGA(0x02) & BIT3))                  //GTP FIFO 1, 1==empty
                            {
                            iFlag |= iNoPrompt;                 //flag as no prompt on terminal
                            sprintf(tBuf,"GTP1  ");    
                            putBuf(prt,tBuf,0);
                            sprintf(tBuf,"RDM 21 %d 10", 10); //Rec on FIBER CH1, 1 MULTI PACKET SEQ
                            process(prt, tBuf);                 //process command            
                            if(cnt2++>800) break;
                            }
                        len= SockKeyWait(0, prt, &key);     //CharCnt=SockKeyWait(mSecWait, Port#, Rtn 1st Ch RecBuf)
                        if(len)
                            {
                            iFlag |= iNoPrompt;             //flag as no prompt on terminal 
                            sprintf(tBuf,"FIBER Receive Mode Exit\r\n");
                            putBuf(prt, tBuf,0);
                            break;
                            }          
                        //if (rdCnt!=0xC) putBuf(prt,"\r\n",0);//0xC==empty
                        mDelay(1000);                        
                        }                                             
                    }                    
                    break;                     
                    }
                
                else if (!strcmp(tok, "TESTLVDS"))         //Tester Code, LVDS sections
                    {
                    uint16_t len=0, key, dat, cnt, cntH, wcnt, xDat, xSum, Sum, d16, chan, Err=0, rcntErr=0, dBuf[dBufSz], prts, totprts=0;
                    uint32_t gcnt=0, h=0, loopLv=0;

                    sprintf(tBuf,"Setting ROC LVDS Clock to 25Mhz to match FEBs 25Mhz return data clock\r\n");
                    putBuf(prt, tBuf,0);
                    //Set ROC 25Mhz LVDS Xmit to match FEB LVDS return clock that always runs at 25Mhz                    
                    wr16FPGA(0x000, 0x00);                   //Set LVDS clock to 25Mhz

                    
                    //Use simple commands without wait option
                    sprintf(tBuf,"Option: 'TESTLVDS  1'       (Transmit Mode)\r\n\n");
                    putBuf(prt, tBuf,0);
                    sprintf(tBuf,"Option: 'TESTLVDS  2 0 s'   (Receive, Don't Display Data, Stop on 1st Error\r\n");
                    putBuf(prt, tBuf,0);
                    sprintf(tBuf,"Option: 'TESTLVDS  2 1 s'   (Receive, Display All Data,   Stop on 1st Error)\r\n\n");
                    putBuf(prt, tBuf,0);
                    sprintf(tBuf,"     ** Test each ROC in Transmit and Receive Modes **\r\n\n");
                    putBuf(prt, tBuf,0);
                    sprintf(tBuf,"     ** Start Transmit Mode ROC 1st, then Second ROC in Receive **\r\n\n");
                    putBuf(prt, tBuf,0);
                    
                    
                    sprintf(tBuf,"     **  Press any key to begin testing **\r\n\n");
                    putBuf(prt, tBuf,0);
                    while (SockKeyWait(0, prt, &key)==0);     
                    
                    //Turn Data Pool OFF, leave off for all testing
                    lvLnk.PoolMode=0;                   //data pool off  

                    param1=arg_dec(&paramPtr,0);           //1st xmit=1 (always recieves)
                    param2=arg_dec(&paramPtr,0);           //2nd display mode 0=counters only
                    param3=arg_dec(&paramPtr,0);           //3rd Exit on first error
                    
                    sPTR prtAddrS, prtAddrD, prtAddrC;                    
                    sPTR saddr = (sPTR) fpgaBase0;
                    saddr += 0x6E;                          //Added address 0x6e for testsep to xmit on all LVDS ports   
                    
                    EmptyAll_LVDS_FIFOs();                  //empty lvds fifos                      
                    //wr16FPGA(0x0, 0x0);                   //stop lvds trigger xmits
                    
                    if(param1==1)  //1=Xmit    
                        sprintf(tBuf,"ROC SN%-2d Xmit Mode Plus Receives\r\n",Ser_Cntrl_Numb.serNumb); 
                    else
                        sprintf(tBuf,"ROC SN%-2d Rec Mode, Each Port Receives 128 Words, (256 Words Ok)\r\n",Ser_Cntrl_Numb.serNumb); 
                    putBuf(prt,tBuf,0);   
                    sprintf(tBuf,"ROC SN%-2d Scanning all 24 POE LVDS PORTS for Received Data (Note Xmit SumCheck=FFFO)\r\n",Ser_Cntrl_Numb.serNumb); 
                    putBuf(prt,tBuf,0);   

              //tek code setup for trans ROC only mode every 100mSec
              //tek code setup for receive ROC only always checking for word count
              //and timeout/breakout in case word count fails

                    while(1) 
                        {  
                        prts=0;
                        totprts=0;

                      //Transmit Mode==1
                      //Transmit Mode==1 
                        if(param1==1)  
                           {
                           h=0;
                           uint16_t d, cnt;
                           prtAddrD = (u_16Bit*)IOPs[1].FM30_DATp;    //data Ch1 only
                           prtAddrC = (u_16Bit*)IOPs[1].FM30_DATp+8;  //word count chan 1 only
                           for (h=1; h<100000; h++)
                                {
                                cnt= *prtAddrC;
                                if (cnt)
                                  break;      //DUT returns 1 word when ready
                                uDelay(10);
                                }
                            d= *prtAddrD;
                            if(cnt)
                              {
                              sprintf(tBuf,"DUT Ready Loop=%d  ReplyCnt=%d  DataReply=%0x\r\n",loopLv, cnt, d); 
                              for (int j=0; j<256;j++)      //read and empty any extra fifo data
                                *prtAddrC;
                              }
                            else
                              sprintf(tBuf,"DUT Ready Loop=%d  ReplyCnt=%d\r\n",loopLv++, cnt); 
                            putBuf(prt,tBuf,0);   
                            loopLv++;  //loopLv transmit mode incr
                            EmptyAll_LVDS_FIFOs();      //empty lvds fifos                                                   
                            mDelay(100);  
                            //Transmit Test Data Pattern on LVDS 128 word loop
                            xDat=0x0001;
                            xSum=0;  
                            for (int i=1; i<=128; i++)  //24 Ports todo
                                {
                                xSum +=xDat;
                                *saddr= xDat;
                                xDat= xDat<<1;
                                if (i%16==0)
                                    xDat=1;
                                }
                            mDelay(100);  
                            }
                
                        //Receive Mode==0
                        //Receive Mode==0
                            if(param1==2)  
                            {
                              
                            prtAddrC = (u_16Bit*)IOPs[1].FM30_DATp+8;  //word count chan 1 only                              
                            for (h=0; h<100000; h++)
                                {
                                if (*prtAddrC>127)
                                  break;      //DUT returns 1 word when ready
                                uDelay(10);
                                }
                            sprintf(tBuf,"\r\nDUT Ready Loop=%d  Cnt=%d\r\n",loopLv, *prtAddrC); 
                            putBuf(prt,tBuf,0);   
                            loopLv++;  //loopLv rec mode incr

                            
                            for(chan=1; chan<=24; chan++)    //modified to find data on any port
                              {
                              //init channel pointers
                              prtAddrD = (u_16Bit*)IOPs[chan].FM30_DATp;    //data
                              prtAddrS = (u_16Bit*)IOPs[chan].FM40_STAp;    //status
                              prtAddrC = (u_16Bit*)IOPs[chan].FM30_DATp+8;  //word count                    
                             
                              if(chan==1)
                                {
                                 h=0;
                                 while (h++<1500)
                                    {
                                    if (*prtAddrC>128-10)  //hardcoded expect 128 words
                                      h=2000;
                                    uDelay(100);
                                    }
                                }
                                                                                                              
                              for(h=0; h<20000; h++)
                                  {
                                  if (*prtAddrC > 10)              //after 1 word, fifo fills quickly
                                      {
                                        h=25000;     //break out of loop once words recd
                                      }
                                  uDelay(10);
                                  }
                              if(h>25000)
                                h++;
                              if(chan==23) //tek test 7/16/24
                                wcnt=9999;
                              
                              wcnt= *prtAddrC;                          //port word count
                              d16= *prtAddrS;                           //check data avail status
                              d16 &= IOPs[chan].ePHY_BIT;               //0= data available  
                              cnt=0;
                              Sum=0;
                              prts++;                      
                              
                              //Expect 128 but may get twice
                              
                              if ( wcnt>0 )                             //words available only
                                  {
                                  cntH=wcnt;

                                  //Move all received FIFO data to buffer 
                                  //create checksum
                                  while(wcnt>0)
                                      {
                                      h_TP48_HI                   
                                      dat= *prtAddrD;                     //port addr for recd' data
                                      if(cnt++<256)
                                          dBuf[cnt]=dat;
                                      wcnt--;
                                      Sum +=dat;
                                      h_TP48_LO
                                      }
                                  
                                  gcnt += cnt;
                                  totprts++;                                

                                  //check chksum
                                  //now display data if bad
                                  int x=1;
                                  if (param2==1)
                                      {
                                      if(cnt>127) 
                                        cnt=128;  
                                      while(cnt-->0)
                                        {                                        
                                        if(param2==1)                   //Display all data if enabled
                                            {
                                            sprintf(tBuf,"%4X ", dBuf[x++]); 
                                            if (cnt%16==0)
                                               strcat(tBuf,"\r\n");  
                                            putBuf(prt,tBuf,0);  
                                            }
                                        else
                                            mDelay(1);                  //Allow time recd data FIFO loading
                                        }
                                      }
                                                                  
                                  if ((Sum == 0xfff8) || (Sum == 0xfff0) )  //sum for word counts 128 or 256
                                      {
                                      sprintf(tBuf,"Prt#%2d  Recd=%d   SumCk=%04X  TotErrDat=%d   ErrsRecCnt=%d\r\n", chan, cntH, Sum, Err, rcntErr); 
                                      putBuf(prt,tBuf,0);                                    
                                      LED_GRN1;  //GRN ON
                                      LED_RED0;  //RED OFF
                                      }
                                  else
                                      {
                                      ++Err;
                                      sprintf(tBuf,"Prt#%2d  Recd=%d   SumCk=%04X  TotErrDat=%d   ErrsRecCnt=%d *\r\n", chan, cntH, Sum, Err, rcntErr); 
                                      putBuf(prt,tBuf,0);                                    
                                      LED_RED1;  //RED ON
                                      LED_GRN0;  //GRN OFF
                                      if(param3)
                                          param3=999;  //force exit on first error
                                      }
                                  }
                              else  //no data timeout
                                  {
                                  rcntErr++;
                                  sprintf(tBuf,"Prt#%2d  Recd=%d   SumCk=%04X  TotErrDat=%d   Missing Port Data *****\r\n\n", chan, cntH, Sum, Err); 
                                  putBuf(prt,tBuf,0);                                    
                                  LED_RED1;    //RED ON
                                  }
                              
                              
                              
                              //exit loop if exit on error mode set
                              if(param3==999) 
                                    break;                            
                              }//End Receive Mode Single Channel   
                            
                            //send done word on lvds to xMIT Mode ROC
                            *saddr=0x1234;
                            } //end All 24 Channels
                        
                        //break on USB or socket data entry
                        len= SockKeyWait(0, prt, &key);     
                        if(param3==999) len=1;  //force exit on first error mode
                        if(len || (KeyBoardWait(0)==1))
                            {
                            sprintf(tBuf,"Test Exit\r\n");
                            chan=26;  //break out of loop
                            putBuf(prt, tBuf,0);
                            break;
                            }  
                        }
                    LED_RED0
                    LED_GRN0
                    sprintf(tBuf,"Reset ROC LVDS Clock from 25Mhz back to normal 20Mhz mode\r\n");
                    putBuf(prt, tBuf,0);                      
                    //restore ROC 20Mhz LVDS Xmit clock
                    wr16FPGA(0x000, 0x010);                   //Set LVDS clock to normal 20Mhz
                    break;  
                    }
                else if (!strcmp(tok, "TESTPHY"))       //Tester Code, PHY sections
                    {
                    uint16_t sndCnt=0, len, key,ccr=0;                     
                    iFlag |= iNoPrompt;                 //flag as no prompt on terminal 
                    lvLnk.PoolMode=0;                   //data pool off  
                    sprintf(tBuf,"ROC PHY Testing Mode, 1=Xmit Only, 2=Rec Only\r\n");
                    putBuf(prt, tBuf,0);
                    sprintf(tBuf,"Option: TESTPHY 1    (Xmits and Recs all ports, PacketDataSize=128Dec)\r\n");
                    putBuf(prt, tBuf,0);
                    sprintf(tBuf,"Option: TESTPHY 2    (Recs all Ports)\r\n");
                    putBuf(prt, tBuf,0);
                    sprintf(tBuf,"   ** Test each ROC in Transmit and Receive Modes **\r\n\n");
                    putBuf(prt, tBuf,0);

                    param1=arg_hex(&paramPtr,2);    //now get 1st param, Test Mode xmit/rec
                    global_pass=0;                  //pass counter
                    g_errorcnt=0;                   //error counter
                    g_errNDR=0; 
                    iFlag |= iNoPrompt;             //flag as no prompt on terminal 
                    wr16FPGA(0x400, 8);             //disable auto move data to fpga0 mode
                    wr16FPGA(0x800, 8);
                    wr16FPGA(0xc00, 8);
                    int rxStat4,rxStat8,rxStatC;                      
                    sPTR prtAddrS, prtAddrD;
                    
                    //Turn Data Pool OFF, leave off for all testing
                    lvLnk.PoolMode=0;                   //data pool off                       
                    
                    if(param1==2) //Mode=2, RECEIVE Data Loop, any key to exit
                        {
                        sprintf(tBuf,"ROC RECEIVE PHY Mode Only (SN#%d)\r\n", Ser_Cntrl_Numb.serNumb);
                        putBuf(prt, tBuf,0);

                        while(1)
                            {
                            //reset daq fpga Interlink FIFOs
                            //ePHY buffers must be empty before filling request
                            rxStat4 = ePHY_RX_STA_416&0xff;
                            rxStat8 = (ePHY_RX_STA_816&0xff);
                            rxStatC = (ePHY_RX_STA_C16&0xff);
                            g_PhyPrts=0;
                            
                            if (rxStat4+rxStat8+rxStatC)
                            //**** comment out for() loop to check single port only
                            for(param2=1; param2<=24; param2++)             //modified to find data on any port
                                {
                                g_PhyPrts++;
                                iFlag |= iNoPrompt;                     //flag as no prompt on terminal 
                                sprintf(tBuf,"PRECTESTER %d", param2);  //empty ePhy rec fifos
                                process(prt, tBuf);                            
                                }  
                            
                            len= SockKeyWait(0, prt, &key);         //CharCnt=SockKeyWait(mSecWait, Port#, Rtn 1st Ch RecBuf)
                            if(len)
                                {
                                iFlag |= iNoPrompt;                 //flag as no prompt on terminal 
                                sprintf(tBuf,"PHY Recieve Mode Exit\r\n");
                                putBuf(prt, tBuf,0);
                                break;
                                }    
                            if(KeyBoardWait(0))                     //returns non zero if USB char avail (wiats nn seconds)
                                break;  
                            if(g_PhyPrts)
                                {
                                LED_RED0;  //RED Off
                                LED_GRN1;  //GRN On
                                sprintf(tBuf,"\t\t\t\t\t\t\t\t\tActive PHY Ports=%d\r\n", g_PhyPrts);
                                putBuf(prt, tBuf,0);
                                }
                            }
                        global_pass++;                    
                        }
                    else  
                        //Mode=1, Transmit data loop
                        {
                        iFlag |= iNoPrompt;                 //flag as no prompt on terminal 
                    
                        //ROC1 Master Mode (Sends PHY Packets to DUT)
                        param2=arg_dec(&paramPtr,128);    //get 2nd param, xmit Byte cnt  
                        wr16FPGA(0x400, 8);                 //disable auto move data to fpga0 mode
                        wr16FPGA(0x800, 8);
                        wr16FPGA(0xc00, 8);
                        int j,k=0;
                        //Init test data       *** 128 '80H' words maximum in ROC Phy Xmit FIFOs   
                        ccr=0;
                        phyBuf128[0]=0;
                        phyBuf128[1]=0;
                        for(j=0; j<=128; j++)
                            {
                            phyBuf128[j]=k;
                            ccr+=k;
                            k++;
                            }
                        phyBuf128[126]=ccr&0xff;
                        phyBuf128[127]=ccr>>8;

                        sprintf(tBuf,"ROC TRANSMIT PHY Mode (SN#%d)  Xmit Data ChkSum=%04X\r\n",Ser_Cntrl_Numb.serNumb, ccr);
                        putBuf(prt, tBuf,0);

                        while(1)
                            {
                            sndCnt= param2;               //send packet size word cnt to byte count
                            //_disable_interrupt_();
                            //send standard min 6 byte header with with test data packet
                            PHY_LOAD_DAQ_K28SEND_BCAST_MINI_TESTER(0xaa55, HappyBus.PoeBrdCh, (sPTR)phyBuf128, sndCnt+3);     // bytes to send                                
                            //_enable_interrupt_();
                            
                            //user socket port keyboard breakout check
                            len= SockKeyWait(0, prt, &key); //CharCnt=SockKeyWait(mSecWait, Port#, Rtn 1st Ch RecBuf)
                            if(len)
                                {
                                iFlag |= iNoPrompt;                      //flag as no prompt on terminal 
                                sprintf(tBuf,"PHY Transmit Mode Exit\r\n");
                                putBuf(prt, tBuf,0);
                                break;
                                }     
                            //user USB port keyboard breakout check
                            if(KeyBoardWait(0))            //returns non zero if USB char avail (wiats nn seconds)
                                break;
                            
                            mDelay(800);                            
                            }                               
                        }
                    break;
                    }                
                else  {parErr=0xf;   break; }
        case 'U':        
                if ((!strcmp(tok, "UBUN"))||(!strcmp(tok, "UB")))//uBunch request status display
                    {                               
                    sprintf(Buf1500," GTP0 RECEIVE FIFO : uBunch 9 Word Data Req, Out of Order\r\n");
                    sprintf(tBuf,   " uB Packet Re Sync : %-7lu\r\n", uBReq.errRESYNC);
                    strcat(Buf1500,tBuf);
                    putBuf(prt, Buf1500,0);             //Note:putBuf to hBus will change fpga ptrs
                    
                    uBReq.errRESYNC= 0;
                    break;
                    }
                else if (!strcmp(tok, "UB0"))           //test code here, delete later
                    {
                    uBReq.SmPacMode=0;
                    uBReq.Mode=0;
                    REG16(fpgaBase0)= 0x0;              //stop test mode
                    uBReq.Flag &= ~DAQuB_Test;          //daq off
                    uBReq.Port= prt;                    //save port number for later status printout
                    break;
                    }
                else if (!strcmp(tok, "UB1"))           //test code here, delete later stuct k28
                    {
                    //data req pack via optical loopback
                    uBReq.uDly= arg_dec(&paramPtr,20);  //repeat after fixed delay
                    break;
                    }
                else if (!strcmp(tok, "UB2"))           //test code here
                    {
                    //int rxStat4,rxStat8,rxStatC;                      
                    //data req pack via optical loopback
                    param1= arg_dec(&paramPtr,1);       //Max 2 triggers per request
                    param2= arg_dec(&paramPtr,1);       //Loop to repeat in background
                    uBReq.ReqCnt= param1;
                    uBReq.LoopCnt= param2;              //32 bit
                    uBReq.Mode=2;                   
                    REG16(fpgaBase0+(0x27*2))= 0x300; //Reset SerDes and FIFOs, not equal 7
                    
                    h_TP47_LO                      
                    //tek small packets 03-13-20                  
                    uBReq.SmPacMode=1;                    
                    uBReq.Flag |= DAQuB_Test;           //Note if 'TRIG 1' active, packet will not be procees in 'DAQuB_Test test mode'
                    uBReq.Port= prt;                    //save port number for later status printout  
                    break;
                    }
                else if (!strcmp(tok, "UB3"))           //test code here, delete later stuct k28
                    {
                    //data req pack via optical loopback
                    uBReq.uDly= arg_dec(&paramPtr,25);  //repeat after fixed delay
                    k28.mode= eCMD_DAQ_DY2;
                    uBReq.Mode=3;
                    //dump remaining words uBun Req, no cnt reg at this time
                    //prevent optimizing out
                    int empty;
                    for(int j=0; j<4000; j++)
                        empty=GTP0_RQ_PAC0D;            //empty/dump fifo
                    
                    REG16(fpgaBase0+(0xf*2))= 10*10;
                    REG16(fpgaBase0+(0x32*2))= 1;       //bunch cnt high
                    REG16(fpgaBase0+(0x33*2))= 100;     //bunch cnt low
                    REG16(fpgaBase0)= 0x303;            //one loop test ubunch and heartbeat
                    mDelay(1);                          //allow time to ub reqs load
                    uBReq.Flag |= DAQuB_Test;           //daq on test, test mode
                    uBReq.Port= prt;                    //save port number for later status printout
                    break;
                    }
                else if (!strcmp(tok, "UB4"))           //test code here, delete later stuct k28
                    {
                    param2=arg_dec(&paramPtr,2);        //get param
                    uBReq.SmPacMode=1;
                    for (int i=0; i<param2; i++)
                      {
                      GTP1_Rec_Trigs_Fake_uB_Request();
                      uDelay(800);
                      }
                    break;
                    }
                else  {parErr=0xf;   break; }
        case 'Z':
                if (!strcmp(tok, "ZSOCK"))              //Mutiple 16bit EPI Bus OrangeTree ZestETM1 reads
                    {
                    int d16;
                    sPTR saddr;
                    *Buf1500=0;
                    //Cycle thru 16 channels reading 7 words base memory ++
                    for(int k=0; k<16; k++)
                        {
                        saddr= (sPTR) ZestETM1 + (16*k);    //CHANNEL_SPACING==16words
                        for (int i=1; i<8; i++)             //Note: Buf1500 is 1500 bytes
                          {
                           d16= *saddr++;
                           sprintf (tBuf,"%04x ", d16);
                           strcat(Buf1500, tBuf);
                          }
                        strcat(Buf1500,"\r\n");
                        putBuf(prt, Buf1500,0);             //Note:putBuf to hBus will change fpga ptrs
                        *Buf1500=0;                         //set string size ==0
                        }                        
                    putBuf(prt,"\r\n",2);
                    break;
                    }
                else if (!strcmp(tok, "ZINIT"))            //re-Init network 'ZestETM1' module socket
                    {
                    iFlag &= ~OTREE_CONFIG;                //clr flag
                    tcp_server_init(0,netInfo.sTelnet0, eRecDatBuf[0] );    //(sock,port)                     
                    tcp_server_init(1,netInfo.sTelnet1, eRecDatBuf[1] );    //(sock,port)                     
                    tcp_server_init(2,netInfo.sTelnet2, eRecDatBuf[2] );    //(sock,port)                     
                    tcp_server_init(3,netInfo.sTelnet3, eRecDatBuf[3] );    //(sock,port)                     
                    break;
                    }
                else if (!strcmp(tok, "ZRD"))               //16 bit EPI BUSS OrangeTree ZestETM1 read
                    {
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//now get 1st param
                        { parErr++;  break; }
                    //address only, no board number
                    param1 = ((param1 & 0xffff)/2);
                    sPTR saddr = (sPTR) ZestETM1;
                    saddr += param1;                        //adding to pointer, incr by ptr size
                    g_dat16= *saddr;

                    sprintf(Buf1500,"%04X\r\n",g_dat16);
                    putBuf(prt, Buf1500,0);                 //send to current active port
                    break;
                    }
                else if (!strcmp(tok, "ZWR"))                //16 bit EPI BUSS OrangeTree ZestETM1 Write
                    {
                    if ( (param1=arg_hex(&paramPtr,-1))==-1)//now get 1st param (addr)
                        { parErr++;  break; }
                    //address only, no board number
                    param1 = ((param1 & 0xffff)/2);

                    if ( (param2=arg_hex(&paramPtr,-1))==-1)//now get 2nd param (data)
                        { parErr++;  break; }

                    sPTR saddr = (sPTR) ZestETM1;
                    saddr += param1;                        //adding to pointer, incr by ptr size
                    *saddr= param2;
                    break;
                    }
                
                else if (!strcmp(tok, "ZSRDI"))             //ZEST ETM1 read using SIO port
                    {   
                    uint16 i=0, d16;
                    param1=arg_hex(&paramPtr,0x200);        //get 1st param, GP REG BASE ADDR
                    param2=arg_dec(&paramPtr,10);           //get 1st param, GP REG BASE ADDR
                    if(param2>128) param2=128;
                    param3= param2;
                    memset(Buf1500, 0, 64<<1);              //clear buf partially
                    while(param2--)
                        {
                        ZEST_RD(param1, &d16); //addr, data
                        lvdsBuf128[i++]= SwapBytes(d16);
                        param1+=2;                          //incr word address 
                        }
                    HexDump16((char*)lvdsBuf128,param3<<1), prt; //HexDump(buffer,len)  
                    break;
                    } 
                else if (!strcmp(tok, "ZSN"))               //ZEST ETM1 SIO PORT READs
                    {   
                    uint16 i=0, d16;
                    param1= 0x200;                          //start adr
                    param2=17;                              //count to read
                    memset(lvdsBuf128, 0, 20<<1);            //clear buf
                    while(param2--)
                        {
                        ZEST_RD(param1, &d16);              //addr, data
                        lvdsBuf128[i++]= d16;
                        param1+=2;
                        }
                    sprintf(tBuf,"Local IPv4 : %03u.%03u.%03u.%03u  Zest Addr 0x200-202\r\n", lvdsBuf128[0]>>8&0xff,lvdsBuf128[0]&0xff,lvdsBuf128[1]>>8&0xff,lvdsBuf128[1]&0xff);
                    putBuf(prt, tBuf,0);                    
                    sprintf(tBuf,"GATEWAY    : %03u.%03u.%03u.%03u  Zest Addr 0x20C-20E\r\n", lvdsBuf128[6]>>8&0xff,lvdsBuf128[6]&0xff,lvdsBuf128[7]>>8&0xff,lvdsBuf128[7]&0xff);
                    putBuf(prt, tBuf,0);                    //send to current active port
                    sprintf(tBuf,"SUBNET MASK: %03u.%03u.%03u.%03u  Zest Addr 0x204-206\r\n", lvdsBuf128[2]>>8&0xff,lvdsBuf128[2]&0xff,lvdsBuf128[3]>>8&0xff,lvdsBuf128[3]&0xff);
                    putBuf(prt, tBuf,0);                    //send to current active port

                    sprintf(tBuf,"DHCP/AutoIP: %05u            Zest Addr 0x208\r\n", lvdsBuf128[4]);
                    putBuf(prt, tBuf,0);                    
                    sprintf(tBuf,"Jumbo Frame: %05u            Zest Addr 0x210\r\n", lvdsBuf128[8]);
                    putBuf(prt, tBuf,0);                    
                    sprintf(tBuf,"LocCntrlPrt: %05u            Zest Addr 0x212\r\n", lvdsBuf128[9]);
                    putBuf(prt, tBuf,0);                    
                    sprintf(tBuf,"LocalWebPrt: %05u            Zest Addr 0x214\r\n", lvdsBuf128[10]);
                    putBuf(prt, tBuf,0);                    

                    sprintf(tBuf,"Update Reg : %02x (h)           Zest Addr 0x216 0=Ok, 2=Failed\r\n", lvdsBuf128[11]);
                    putBuf(prt, tBuf,0);                    
                    sprintf(tBuf,"Link Status: %02x (h)           Zest Addr 0x21E Bit4=Connected, Bit2=1Gbs\r\n", lvdsBuf128[15]);
                    putBuf(prt, tBuf,0);                    

                    if((lvdsBuf128[15] & 0x10)==0)
                        {
                        sprintf(tBuf,"ZEST_ETM1  : Regs are updated after a valid ethernet cable connection\r\n");
                        putBuf(prt, tBuf,0);                    
                        }
                    
                    break;
                    } 
                
                else if (!strcmp(tok, "ZSWR"))              //ZEST ETM1 read using SIO port
                    {   
                    param1=arg_hex(&paramPtr,0x204);        //get 1st param
                    param2=arg_hex(&paramPtr,0xFFFF);       //get 2nd param
                    ZEST_WR(param1, (uint16*)&param2);      //addr, data, CNT
                    break;
                    } 
                else if (!strcmp(tok, "ZSWRT"))             //ZEST ETM1 WR/RD register tesing
                    {   
                    uint16 d16,rd16;
                    param1=arg_hex(&paramPtr,0x204);        //get 1st reg
                    d16=arg_hex(&paramPtr,0);               //get 2nd data
                    ZEST_WR(param1, &d16);                  //SIO Operation
                    ZEST_RD(param1, &rd16);                 //SIO Operation
                    if(rd16!=d16)  
                        putBuf(tty,"OTree: data write failed\r\n",0); 
                    break;
                    } 
                else if (!strcmp(tok, "ZSU"))               //ZEST ETM1 read using SIO port
                    {   
                    uint16 d16;
                    uint32 i=0;
                    sPTR saddr;
                    //ETHERNET INTERRUPT DISABLE
                    gioDisableNotification(gioPORTA, BIT5) ;
                    
                    //Change local ip Hi
                    d16= ((netInfo.ipAddr[0]<<8) | (netInfo.ipAddr[1])) ;
                    ZEST_WR(0x200, &d16);
                    //Change local ip Lo
                    d16= ((netInfo.ipAddr[2]<<8) | (netInfo.ipAddr[3])) ;
                    ZEST_WR(0x202, &d16);

                    //Change local Mask Hi
                    d16= ((netInfo.netMask[0]<<8) | (netInfo.netMask[1])) ;
                    ZEST_WR(0x204, &d16);
                    //Change local Mask Lo
                    d16= ((netInfo.netMask[2]<<8) | (netInfo.netMask[3])) ;
                    ZEST_WR(0x206, &d16);
                    
                    //AUTO_IP, DHCP disabled
                    d16= 0;
                    ZEST_WR(0x208, &d16);
                   
                    //Change GateWay Hi
                    d16= ((netInfo.gateWay[0]<<8) | (netInfo.gateWay[1])) ;
                    ZEST_WR(0x20C, &d16);
                    //Change GateWay Lo
                    d16= ((netInfo.gateWay[2]<<8) | (netInfo.gateWay[3])) ;
                    ZEST_WR(0x20E, &d16);                    
                    
                    //Set jumbo frame limit default
                    d16= 1500;
                    ZEST_WR(0x210, &d16);
                    //network control port default
                    d16= 20480;
                    ZEST_WR(0x212, &d16);                  
                    //Set Web Server port default
                    d16= 80;
                    ZEST_WR(0x214, &d16);
                    
                    d16= 1;
                    ZEST_WR(0x216, &d16);   //rising edge trigs prog cycle
                    netInfo.UpDate=0;
                    
                    while(1)
                        {
                        //this oTree change may take over 700mS
                        ZEST_RD(0x216, &d16);              //addr, data
                        if(d16!=0)   //should go busy==1 or fail==2
                            {
                            i=0;
                            while(i++ < 10005)
                                {
                                uDelay(100);
                                ZEST_RD(0x216, &d16);       //addr, data
                                if((d16==0)||(d16==2)) {break;}
                                }
                            break;
                            }
                        uDelay(100);
                        if(i++ > 10005) break;
                        }
                    if ((d16==0)&&(i<10000)) 
                            {
                            sprintf(tBuf,"UpDateReg=%x  'Okay'\r\n",d16);
                            putBuf(tty,tBuf,0);                
                            }
                    else if(d16==2)
                            {
                            //socket close, use USB port for status
                            sprintf(tBuf,"UpDateReg=%x  'Failed'\r\n",d16);
                            putBuf(tty,tBuf,0); 
                            netInfo.UpDate=2;
                            break;
                            }
                    //Update network setting register
                    d16= 0;         
                    ZEST_WR(0x216, &d16);   //must be zero
                    //ETHERNET INTERRUPT ENABLE
                    gioEnableNotification(gioPORTA, BIT5) ;
                    break;
                    }                                 
                else if (!strcmp(tok, "ZINIT1"))            //re-Init network 'ZestETM1' module socket
                    {
                    iFlag &= ~OTREE_CONFIG;                //clr flag
                    tcp_server_init(0,netInfo.sTelnet0, eRecDatBuf[0] );    //(sock,port)                     
                    tcp_server_init(1,netInfo.sTelnet1, eRecDatBuf[1] );    //(sock,port)                     
                    tcp_server_init(2,netInfo.sTelnet2, eRecDatBuf[2] );    //(sock,port)                     
                    tcp_server_init(3,netInfo.sTelnet3, eRecDatBuf[3] );    //(sock,port)                     
                    break;
                    }
                else  {parErr=0xf;   break; }
        default:
                putBuf(prt,"\r\nsyntax error?\r\n",0);  //send to current active port
                break;
        }//end switch(*tok)

   if (parErr)
        {
        if(parErr==0xf)
            putBuf(prt,"\r\n?cmd\r\n",8);       //cmd err, send to current active port
        else
            putBuf(prt,"\r\n?parm \r\n",10);    //param err, send to current active port
        }
    else
        if ((iFlag & iNoPrompt))
            iFlag &= ~iNoPrompt;                //one time flag, clear it
        else
            if(prt != DCS)
                newLinePrompt(prt);                 //Send newLine prompt with or without Port#
    
return 0;
}


//Send newLine prompt with or without Port#
void newLinePrompt(int prt)
{
    if(genFlag&ECHO_ACT_PORT)
        {
        putBuf(prt,USB_Rec.prompt,0);           //link cmd active prompt, add poe port number 
        }
    else
        putBuf(prt,"\n>",2);                    //normal prompt
  
}




//-------------------- DMA CODE ------------------------------
//
/** void dmaConfigCtrlPacket(uint32 sadd,uint32 dadd,uint32 dsize)
*
*   configuring dma control packet stack
*       sadd  > source address
*       dadd  > destination  address
*       dsize > data size
*  @ note : after configuring the stack the control packet needs to be set by calling dmaSetCtrlPacket()
*  note  The frame/element count fields of the the  ITCOUNT register is only 13 bit wide, 
*  note  hence a max of 8191 frames/elemets. bits 15:13 are ignored - so 8192 is same as 0.
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




//Reverse bits of parameter in the register R0, return value is passed back
//to its caller in register R0.
#pragma optimize= none
unsigned short revBits16(short int c)
    {
    asm(" RBIT R0, R0 \n");                     //Asm lng rev bits in 32 register
    asm(" MOV R0, R0, LSR #16 \n");             //shift and return 16_bits in R0
  //asm(" BX lr\n");
    return c;
    }


//Reverse bits of parameter in the register R0, return value is passed back
//to its caller in register R0.
#pragma optimize= none
u_8Bit revBits8(u_8Bit c)                       //reverse 8 bits
    {
    asm(" RBIT R0, R0 \n");                     //Asm lng rev bits in 32 register
    asm(" MOV R0, R0, LSR #24 \n");             //shift and return 8_bits in R0
  //asm(" BX lr\n");
    return c;
    }


//byte swap
#pragma optimize= none
unsigned short SwapBytes(unsigned short c)
{
    __asm("    rev16     r0, r0\n"
          "    bx      lr\n");                  // need this to make sure r0 is returned
    return(c);                                  // return makes compiler happy - ignored
}

//Load-Multiple memory copy
//The previous example is modified to use LDM and STM instructions, 
//transferring 8 words per iteration. Due to the extra registers used, 
//these must be stored to the stack and later restored.

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




void MemCopy32(unsigned long *dst, unsigned long *src, int bytes)
{
  for (int i = 0; i < (bytes + 3) / 4; i++)
    *dst++ = *src++;
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
		// offset
		//printf("0x%04x  ", s-addr );

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
		// offset
		//printf("0x%08lx  ", (long)(s-addr));
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



void HexDump16(char *addr, int len){
	char	*s=addr, *endPtr=(char *)((long)addr+len);
	int		i, remainder=len%16;
	//print out 16 word
	while (s+16<=endPtr)
        {
		// 16 bytes
		for (i=0; i<16; i+=2)
			printf("%02x%02x ", s[i],s[i+1]);
        
		printf(" ");
		for (i=0; i<16; i++)
            {
			if(s[i]>=32 && s[i]<=125)	
              printf("%c", s[i]);
			else
              printf(".");
            }
		s += 16;
		printf("\r\n");
        }
	
	if (remainder)
        {
		for (i=0; i<remainder; i+=2){
			printf("%02x%02x ", s[i],s[i+1]);
            }
		for (i=0; i<(16-remainder); i++){
			printf("   ");
            }
		printf(" ");
		for (i=0; i<remainder; i++){
			if (s[i]>=32 && s[i]<=125)	printf("%c", s[i]);
			else printf(".");
            }
		for (i=0; i<(16-remainder); i++){
			printf(" ");
            }
		printf("\r\n");
        }
	return;
}




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


//delay in milli-seconds
void mDelay(uint32 cnt)
{
    mStime.g_timeMs=0;
    if (cnt<1) cnt=2;                   //under 1mS can not be timed by sysTick
    while (mStime.g_timeMs < cnt);      //wait till done in sysTick
}



//Set SDram Read Address and wait for status to show burst buffers ready
//Note FPGA SDram control logic, setting rd ptrs wait for ram burst to fill buffer
//FPGA BUS ADDR 1of4 0x0000, 0x0800, 0x1000, 0x1800
int SET_SDADDR_RDx(int chipOffset, int HI, int LO)
{
    int i=0;
    sPTR sdAddr;
    sdAddr= (sPTR)(fpgaBase0+(chipOffset*2)+ (0x04*2));
    *sdAddr++= HI;
    *sdAddr= LO;
    while(1)//while (MIGSTATUSREG& BIT9) //wait for 'MIG read data FIFO to go Non-Empty'
        {
        if(i++> 500)       //breakout, 'Potentially infinite loop'
            break;
        }
    return 0;
}


//Set SDram Write Address and wait for status to show burst buffers ready
//Note FPGA SDram control logic, setting write pointers will flush ram burst (16 words) 
//into memory at the new write address overwritting existing data.
//FPGA BUS ADDR 1of4 0x0000, 0x0800, 0x1000, 0x1800
int SET_SDADDR_WRx(int chipOffset, int HI, int LO)
{
	//set read addr to base, doesnt always change
    sPTR sdAddr= (sPTR)(fpgaBase0+(chipOffset*2)+ (0x02*2));
    *sdAddr++= HI;
    *sdAddr= LO;
    for (int i=0; i<500; i++);  //min delay may not be needed, call time may be enough
    return 0;
}
 


//MT46H64M16LF – 16 Meg x 16 x 4 Banks
//testing 16 Meg x 16 x 4 Banks ==  0x1ff ffff long words
int sdRamTest(int chip, int count, int prt)
{                                           //MT46H128M16LF – 32 Meg x 16 x 4 Banks
    u_32Bit err=0, i;
    u_32Bit addr32, offset=0;
    u_16Bit t16=0, data16;
    sPTR SDramRdHIx;
    sPTR SDramRdLOx;      
    sPTR ucSDramLOx;
    sPTR CSRx;

    if(chip==2)
        {offset=0x400; }     //chip offset
    else if(chip==3)
        {offset=0x800; }   
    else if(chip==4)
        {offset=0xC00; }   

    //set write sdRAM Addr via special sequence
    SET_SDADDR_WRx(offset,0,0);
    ucSDramLOx= ((&SDR_RD16)+offset);
    
    sprintf(Buf1500, "Fpga Wr Addr Ptrs 0x%X, 0x%X\r\n", offset+2, offset+3 );
    putBuf(prt, Buf1500,0); 
    sprintf(Buf1500, "Fpga Rd Addr Ptrs 0x%X, 0x%X\r\nWriting Data...\r\n", offset+4, offset+5 );
    putBuf(prt, Buf1500,0); 
        
    //1st write sdram data 
    for (i=1; i<=count; i++ )
        {
        t16++;
        *ucSDramLOx=t16;            //write data to fpga sdram
        if(i%1000==0)
          t16 <<=1;
        }
        
    sprintf(Buf1500, "Reading Data...\r\n");
    putBuf(prt, Buf1500,0); 
    //set read fpga addr once
    addr32=0;
    t16 =0;
        
    //set read sdRAM Addr via special sequence
    SET_SDADDR_RDx(offset,0,0);
        
    //2nd read sdram data, test
    for (i=1; i<=count; i++)
        {
        t16++;
        data16= *ucSDramLOx;
        if (data16 != t16)
            {
            sprintf(Buf1500, "A=%-7X  WR= %-4X  RD= %-4X\r\n", addr32, t16, data16);
            putBuf(prt, Buf1500,0); 
            if (err++>10)
                break;
            }
        addr32++;
        if(i%1000==0)
          t16 <<=1;
        }
 
    if (err)
        sprintf(Buf1500, "Test Done ... Error Cnt=%d\r\n",err );
    else
        sprintf(Buf1500, "Tested Okay... EndAddr=%X\r\n",addr32/2 );
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
//Hardware delay using RM48L nHET timer, Min Delay 1uS
void hDelayuS(uint32 dly, uint32 Wait)
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
    h_TP47_HI;

    e_HETPROGRAM0_UN.Program0_ST.TIMERCOMP_0.memory.data_word= dly; //refer cnt loaded
    e_HETPROGRAM0_UN.Program0_ST.ENCOUNTER_0.memory.data_word=0;    //cnt up var, clr it
    e_HETPROGRAM0_UN.Program0_ST.START_0.memory.data_word=1<<7;     //enable counter bit
    //hetRAM1->Instruction[7].Data=0;             //clear het up counter reg7
    //hetRAM1->Instruction[8].Data=dly;           //load het refer count reg8
    //hetRAM1->Instruction[0].Data=1<<7;          //set counter start bit reg0
    //wait for time out or check in background
    if (Wait)
        while (genFlag & hDelay);                   //wait to finish, nonzero==busy
    //h_TP47_LO;
}



//display network setup options
void dispNet(int prt)
{
    int i,j;
    sPTR zAdr;    
    //get real IP
    zAdr = (sPTR) ZestETM1;                 
    i= *(zAdr+(0x200/2));
    j= *(zAdr+(0x202/2));
    
    if(i==0)
        {
          sprintf(tBuf,"** Network cable must be connected to see IP Address**\r\n");
          putBuf(prt,tBuf,0);                
        }

    sprintf(Buf1500,"\nChange Options 1-7 for a new network setup\r\n");
    
    //real IP from ZestBoard Flash
    sprintf(tBuf," 1) IP     (ACTUAL)    :%u.%u.%u.%u\r\n", i>>8&0xff,i&0xff,j>>8&0xff,j&0xff);
    strcat(Buf1500, tBuf);
    
    //get real GW from ZestBoard Flash
    i= *(zAdr+(0x20C/2));
    j= *(zAdr+(0x20E/2));
    sprintf(tBuf," 2) G/W IP (ACTUAL)    :%u.%u.%u.%u\r\n", i>>8&0xff,i&0xff,j>>8&0xff,j&0xff);
    strcat(Buf1500, tBuf);

    //get real MASK from ZestBoard Flash
    i= *(zAdr+(0x204/2));
    j= *(zAdr+(0x206/2));
    sprintf(tBuf," 3) MASK   (ACTUAL)    :%u.%u.%u.%u\r\n", i>>8&0xff,i&0xff,j>>8&0xff,j&0xff);
    strcat(Buf1500, tBuf);
           
    sprintf(tBuf," 4) Telnet Sock0= Port :%04d (d)          \"use (\\r) ending Cmds,(128 BytesMax)\"\r\n", netInfo.sTelnet0);
    strcat(Buf1500, tBuf);
    sprintf(tBuf," 5) Telnet Sock1= Port :%04d (d)\r\n", netInfo.sTelnet1);
    strcat(Buf1500, tBuf);
    sprintf(tBuf," 6) Telnet Sock2= Port :%04d (d)\r\n", netInfo.sTelnet2);
    strcat(Buf1500, tBuf);
    sprintf(tBuf," 7) Telnet Sock3= Port :%04d (d)\r\n\n", netInfo.sTelnet3);
    strcat(Buf1500, tBuf);

    sprintf(tBuf,"  Example SET 3 255.255.255.0  ----  Example SET 4 5000\r\n");
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"  Use NETSAV when done.  NETSAV will close all sockets.\r\n");
    strcat(Buf1500, tBuf);
    
    sprintf(tBuf,"  IP, Gateway and MASK values may not change if invalid Subnet\r\n");
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"  Mask Examples \r\n     Mask              Subnetworks     Nodes/Subnet\r\n");
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"     255.255.255.000       1             254\r\n");    
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"     255.255.255.128       2             126\r\n");    
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"     255.255.255.192       4             62\r\n");    
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"     255.255.255.000   255.255.255=(Network Addr)   000=(Device Addr)\r\n");    
    strcat(Buf1500, tBuf);
    putBuf(prt, Buf1500,0);                 //send to current active port
    
    sprintf(Buf1500,"\n  ZestETM1 has logic to prevent illegal settings. This includes not allowing\r\n");
    sprintf(tBuf,"  zero or broadcast addr(e.g. 192.168.1.0 or 192.168.1.255), illegal\r\n"); 
    strcat(Buf1500, tBuf);
    sprintf(tBuf,"  addresses (e.g. 127.x.x.x) or addr not on the same subnet as the gateway\r\n"); 
    strcat(Buf1500, tBuf);    
    if (prt)
        {
          sprintf(tBuf,"  See ZestETM1 IpAddress:Port(80 HTML) settings (AdminPSW=u2e)\r\n"); 
        strcat(Buf1500, tBuf);        
        }
    putBuf(prt, Buf1500,0);                 //send to current active port

    
    //ZestETM1 
    //There is logic to prevent illegal settings.  This includes not allowing zero or broadcast 
    //addresses (e.g. 192.168.1.0 or 192.168.1.255), illegal addresses (e.g. 127.x.x.x) or 
    //addresses not on the same subnet as the gateway. 
    
    /*
    [Zero]=0 is reserved and represents all IP addresses.
    127 is a reserved address and is used for testing, like a loop back on an interface.
    255 is a reserved address and is used for broadcasting purposes.  
    
    Subnet mask is always used with IP address. Subnet mask has only one purpose, 
    to identify which part of an IP address is network address and which part is 
    host address. 
    For example how will we figure out network partition and host 
    partition from IP address 192.168.1.10 ? Here we need subnet mask to get 
    details about network address and host address. 
    
    In decimal notation subnet mask value 1 to 255 represent network address
    and value 0 [Zero] represent host address. 
    
    In binary notation subnet mask ON bit [ 1] represent network 
    address while OFF bit[0] represent host address.    
    */
}


void dataRecall(int mode, u_32Bit fADDR)
{
    //recall from FRAM
    static u_16Bit D16[8];
    sPTR saddr;
    u_32Bit addr32;
    uint16 framAddr;                    //start of saved data
    uint16 cnt=0;
    uint16 ValidFlag;
    
    if (fADDR==fpgaBase0)               //selec fram addr per fpga address
      framAddr= fram_FPGA0_REG+4;       //fpga0
    else if (fADDR==fpgaBase1)
      framAddr= fram_FPGA1_REG+4;       //fpga1
    else if (fADDR==fpgaBase2)
      framAddr= fram_FPGA2_REG+4;       //fpga2
    else if (fADDR==fpgaBase3)
      framAddr= fram_FPGA3_REG+4;       //fpga3
    
    FRAM_RD_BUF16((framAddr-4), &ValidFlag, 1); //framAddr, fpgaAddr, read Cnt
    if(mode==0)                         //param1=0=load constants
        ValidFlag=911;                  //force constant load
    ValidFlag= ValidFlag&0xfff0;
    if (ValidFlag == FP_REGS_VALID0)         //param1=0=load constants
        {
        //stored params good in struct f_RegHold
        for(cnt=1; cnt<FP_REGS_MAX; cnt++)
            { 
            //read data block into temp storage D16[]array
            FRAM_RD_BUF16(framAddr, D16, f_RegConst[cnt].RegCnt);    //framAddr, fpgaAddr, writeCnt
            framAddr += f_RegConst[cnt].RegCnt*2;
            addr32 = (u_32Bit)fADDR+ (f_RegConst[cnt].Addr<<1);      //actual 32 bit address
            
            //load fpga reg, note special case if reg addr >= 0x100
            for (int i=0; i< f_RegConst[cnt].RegCnt; i++, addr32+=2)
                {
                if (f_RegConst[cnt].Addr < 0x100)
                    *(sPTR)addr32= D16[i];
                }
            }
        }
    else
        {
        //NEED DEFALUT STORAGE struct f_RegConst
        for(cnt=1; cnt<FP_REGS_MAX; cnt++)
            { 
            addr32 = (u_32Bit)fADDR + (f_RegConst[cnt].Addr<<1);  //actual 32 bit address
            for (int i=0; i< f_RegConst[cnt].RegCnt; i++, addr32+=2)
                {
                *D16 = f_RegConst[cnt].Value[i];
                //load fpga reg, note special case if reg addr >= 0x100
                if (f_RegConst[cnt].Addr < 0x100)
                    *(sPTR)addr32= *D16;
                }
            }
        }
    
}

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
    tempC = 421-( (751*(float)Ris2Fall) /(float)Fall2Ris);
    //lock out of range data
    if (Ris2Fall<1000)
        tempC=0;
    return tempC;
}



//check fpga for runtime checksum error
//return status of all 4 devices
int fpga_Check(int dev)
{
    int stat=0;
    if (InitB0_Read==0)    //check, low=error
       stat = 0x1;
    if (InitB1_Read==0)    //check, low=error
       stat +=0x2;
    if (InitB2_Read==0)    //check, low=error
       stat +=0x4;
    if (InitB3_Read==0)    //check, low=error
       stat +=0x8;
    if (stat==0)
      return 0;

    //load fpga from flash data
    flashXFER(4, tty);
    mDelay(50);
    
    //recall FPGA Setup Data from FRAM reloaded structure
    dataRecall(1, fpgaBase0);      //fpga0
    dataRecall(1, fpgaBase1);      //fpga1
    dataRecall(1, fpgaBase2);      //fpga2
    dataRecall(1, fpgaBase3);      //fpga3
    return stat;
}



void USB_Rec_Init()
{
    USB_Rec.RdPtr=0;   //this code reset app, does not work with or without interrupts on/off
    USB_Rec.WrPtr=0;
}


	

//Keyboard wait for entry with TIMEOUT
//input wait time in seconds
int KeyBoardWait(int wait)
{
  wait *=1000;
  mStime.g_wTicks=0;;
  if (wait==0)
    mStime.g_wTicks=0;
  while(1)
      {
      if(mStime.g_wTicks > wait)
        return 0;                       //timeout, return 'TRUE'
      else if (USB_Rec.Cnt)             //rec char count
        return 1;                       //data avail, return 'TRUE'
      }
}



//Waits (nn mSec) for received data from socket, 
//Returns 1st Rec'd Char Cnt
//*key holds 1st recd char, valid if rtnCnt > 0
int SockKeyWait(int wait, uint16 sock, uint16 *key)
{
  int len, IPport;
  int *Ptr= (int *) &netInfo.sTelnet0;
  Ptr = Ptr+ (sock*2);
  IPport=  *Ptr;                              //read sockets port numb from structure
  mStime.g_wTicks=0;
  
  if(wait==0)                           //quick check then leave
    {
    len= loopback_tcps(sock,IPport,(uint8*)eRecDatBuf[sock],0);
    return len;                       //data avail, return 'TRUE'
    }
  while(1)
      {
      if(mStime.g_wTicks > wait)            //timer incr at 1mS rate
        return 0;                           //timeout, return 'TRUE'
      else 
        {
        //socket, port, buffer, mode(Sn_MR for no delay)
        len= loopback_tcps(sock,IPport,(uint8*)eRecDatBuf[sock],0);  
        if (len)                            //rec char count
            {
            *key= eRecDatBuf[sock][0];
            return len;                       //data avail, return 'TRUE'
            }
        }
      }
}


//-------------------- DMA CODE ------------------------------

//DMA TESTING ONLY
g_dmaCTRL   g_dmaCTRLPKTz;   // dma control packet configuration stack

/** void dmaConfigCtrlPacket(uint32 sadd,uint32 dadd,uint32 dsize)
*   configuring dma control packet stack
*       sadd  > source address
*       dadd  > destination  address
*       dsize > data size
*   @ note : after configuring the stack the control packet needs to be set by calling dmaSetCtrlPacket()
*
*  note  The frame/element count fields of the the  ITCOUNT register is only 13 bit wide, 
*  note  hence a max of 8191 frames/elemets. bits 15:13 are ignored - so 8192 is same as 0.
*/
void dmaConfigCtrlPacketZest(uint32 sadd,uint32 dadd,uint32 dsize)
{
  g_dmaCTRLPKTz.SADD      = sadd;			  /* source address             */
  g_dmaCTRLPKTz.DADD      = dadd;			  /* destination  address       */
  g_dmaCTRLPKTz.CHCTRL    = 0;                /* channel control            */
  g_dmaCTRLPKTz.FRCNT	  = 1;                /* frame count                */
  g_dmaCTRLPKTz.ELCNT     = dsize;            /* Element / Frame            */
  g_dmaCTRLPKTz.ELDOFFSET = 0;                /* element destination offset */
  g_dmaCTRLPKTz.ELSOFFSET = 0;		          /* element destination offset */
  g_dmaCTRLPKTz.FRDOFFSET = 0;		          /* frame destination offset   */
  g_dmaCTRLPKTz.FRSOFFSET = 0;                /* frame destination offset   */
  g_dmaCTRLPKTz.PORTASGN  = 4;                /* assign dma #               */
  g_dmaCTRLPKTz.RDSIZE    = ACCESS_16_BIT;	  /* read size                  */
  g_dmaCTRLPKTz.WRSIZE    = ACCESS_16_BIT; 	  /* write size                 */
  g_dmaCTRLPKTz.TTYPE     = FRAME_TRANSFER ;  /* transfer type              */
  g_dmaCTRLPKTz.ADDMODERD = ADDR_INC1;        /* address mode read          */
  g_dmaCTRLPKTz.ADDMODEWR = ADDR_FIXED;       /* address mode write         */
//g_dmaCTRLPKTz.ADDMODEWR = ADDR_OFFSET;      /* address mode write         */
  g_dmaCTRLPKTz.AUTOINIT  = AUTOINIT_ON;      /* autoinit                   */
}


//send Binary data, setup done under cmd 'RDB'
//called from cmd 'RDB' and main background loop
//fills uC buffer from FPGA via DMA
//moves uC buffer to OrangeTree via DMA 
//The uC is Little-endian, OrangeTree is Big-endian
//
int sendBin(int prt)
{
    uint16_t static dWordsMoved=0, sReg;
    //move 16bits data words
    if (dWordsMoved==0)                             //Note: datWrdsMoved should always an even number
        {
        if (BinSt.gSndWordCnt >= DMA_DAQ_WSIZ)      //DMA xfer max word size of buffer
            dWordsMoved= DMA_DAQ_WSIZ;              //max TX_BUF_SIZE
        else
            dWordsMoved= BinSt.gSndWordCnt;         //last of the data
        
        //now adjust stored byte count
        BinSt.gSndWordCnt -= dWordsMoved;           //freeSize is always <= MAX BUFFER

        //DMA's data,   FPGA==> uC_dmaBuf
        //if(BinSt.gTestMode)     //test mode feature for speed testing, load test data once
            {
            h_TP47_LO   
            DMA_FPGA_OUT(dWordsMoved);              
            h_TP47_HI
            BinSt.gTestMode=0;
            }
        }
    //wait for space, only avail flag shows that at least 64K bytes out of 3MByes still available
    sReg= (REG16(ZestETM1+(BinSt.gSndPrt*CHANNEL_SPACING)+zIntrEna) & IE_OUTGOING_NOT_FULL); //CHANNEL_SPACING==16words
    if (sReg == 0)
        {
        //do second check
        uDelay(10);
        sReg= (REG16(ZestETM1+(BinSt.gSndPrt*CHANNEL_SPACING)+zIntrEna) & IE_OUTGOING_NOT_FULL); //CHANNEL_SPACING==16words
        if (sReg == 0)
            {
            g_wait++;
            return  BinSt.gSndWordCnt;
            }
        }

    sReg= (REG16(ZestETM1+(BinSt.gSndPrt*CHANNEL_SPACING)+zConnectCSR) &0xf);
    if (sReg != ESTABLISHED)                        //SOCKET CLOSED, STOP DATA XFER
        {
        BinSt.gSndWordCnt=0;
        dWordsMoved=0;
        BinSt.gBusy=0;
        return 0;
        }

    h_TP48_LO
    //DMA's data, uC_dmaBuf ==> ZestETM1
    DMA_OTREE_IN(dWordsMoved);              
    dWordsMoved=0;
    h_TP48_HI  
    
    
    //out of data? done  (diagnostics)
    if(BinSt.gSndWordCnt==0)
        {
        //sprintf(tBuf,"No Data Total Waits = %2.3f Sec\r\n", (float) g_wait * .000025);
        //putBuf(tty, tBuf,0);
        BinSt.gBusy=0;
        return 0;
        }

  return 0;
}



//ch 0, Fpga to Mem
//-------------------- DMA CODE ------------------------------
//
int DMA_FPGA_OUT(int xWords)
{
    //disable DMAs
    dmaDisable();
    //assigning dma request: channel-0 with request line - 1
    dmaReqAssign(DMA_CH0,1 );

    //configuring dma control packets       (srcadd, destadd, datasize)
  //dmaConfigCtrlPacketZest((uint32)BinSt.gSndSrc, (uint32)eRecDatBuf[BinSt.gSndPrt], xWords ); //fpga==> uC_buffer
    dmaConfigCtrlPacketZest((uint32)BinSt.gSndSrc, (uint32)DMA_DAQ_BUF, xWords ); //fpga==> uC_buffer
    
    g_dmaCTRLPKTz.RDSIZE = ACCESS_16_BIT;	    //change read size to 16bits now
    g_dmaCTRLPKTz.ADDMODERD = ADDR_FIXED;       //address mode read 
    g_dmaCTRLPKTz.ADDMODEWR = ADDR_INC1;        //address mode write

    // setting dma control packets, upto 32 control packets are supported
    dmaSetCtrlPacket(DMA_CH0,g_dmaCTRLPKTz);

    //setting the dma channel to trigger on software request
    dmaSetChEnable(DMA_CH0, DMA_SW);
    dmaEnableInterrupt(DMA_CH0, FTC);

    //enabling dma module
    BinSt.gDMA_Fpga2Mem=1;
    dmaEnable();

    while(BinSt.gDMA_Fpga2Mem==1);   //DMA intr handler in 'notifications.c' change value
	//Sockets[sock].TxBufferPtr+= xWords<<1;
	//Sockets[sock].TxBufferLen-= xWords<<1;
    return 0;
}


//RM48L Delay between cycles, TI forumn expert
//What you observed is the expected behavior. For each burst read (single 16 bit read 
//can be considered as burst size of 1), there is a 12 VCLK internal delay between 
//burst. The same is true for reading from the peripheral control registers.
//
//This "12 VCLK" internal delay is also true for writing if the peripheral/external 
//memory space is configured as strongly ordered by R4 MPU. 
//
//This internal delay is reduced to "2 VCLK" in writing when the memory 
//is configured as device.
//
//When you calculate EMIF throughput, it  should be EMIF timing plus this 
//internal delay.  I would suggest using burst access to minimize the effect of 
//internal delay. The burst size of EMIF is 8x16 bits. You can consider using 
//DMA for burst access.


int DMA_OTREE_IN(int xWords)
{
    unsigned char* xSockFIFO = (unsigned char*) ZestETM1 + ((BinSt.gSndPrt *CHANNEL_SPACING+DATA_FIFO));
    //disable DMAs
    dmaDisable();
                    
    //assigning dma request: channel-1 with request line - 1
    dmaReqAssign(DMA_CH1,1 );

    //configuring dma control packets       (srcadd, destadd, datasize)    
    //dmaConfigCtrlPacketZest( (uint32)eRecDatBuf[BinSt.gSndPrt], (uint32)xSockFIFO, xWords ); //uC_buffer==> OT_ZestETM1
    dmaConfigCtrlPacketZest( (uint32)DMA_DAQ_BUF, (uint32)xSockFIFO, xWords ); //uC_buffer==> OT_ZestETM1
    g_dmaCTRLPKTz.RDSIZE    = ACCESS_16_BIT;    //change read size to 16bits now
    g_dmaCTRLPKTz.ADDMODERD = ADDR_INC1;        //address mode read
    g_dmaCTRLPKTz.ADDMODEWR = ADDR_FIXED;       //address mode write

    // setting dma control packets, upto 32 control packets are supported
    dmaSetCtrlPacket(DMA_CH1,g_dmaCTRLPKTz);

    //setting the dma channel to trigger on software request
    dmaSetChEnable(DMA_CH1, DMA_SW);
    dmaEnableInterrupt(DMA_CH1, FTC);

    //If paused, when bit is set back to 0 the GigExpedite will begin transmitting 
    //the buffered data in large packets rather than multiple small packets
	//Sockets[sock].State |= CONN_PAUSE;
	//GigExWriteReg16(sock*CHANNEL_SPACING+CONNECTION_STATE, Sockets[sock].State);
      
    //enabling dma module
    BinSt.gDMA_Mem2Wiz=1;
    dmaEnable();
    while(BinSt.gDMA_Mem2Wiz==1);   //DMA intr handler in 'notifications.c' change value

	//Sockets[sock].State &= ~CONN_PAUSE;
	//GigExWriteReg16(sock*CHANNEL_SPACING+CONNECTION_STATE, Sockets[sock].State);
    return 0;
}




//tek temp code for feb emulation tests

//apply scale factor to input data
void cFRAMrd(int fAddr, int short *fData, int short *peds)
{
    int offset= fAddr;
    //store constants, addr range 0x030-047
    if((fAddr>=addr030) && (fAddr<=addr047))
        fAddr = const_FPGADACs0; 
    //store constants, addr range 0x430-447
    else if((fAddr>=addr430) && (fAddr<=addr447))
        fAddr = const_FPGADACs1; 
    //store constants, addr range 0x830-847
    else if((fAddr>=addr830) && (fAddr<=addr847))
        fAddr = const_FPGADACs2; 
    //store constants, addr range 0xC30-C47
    else if((fAddr>=addrC30) && (fAddr<=addrC47))
        fAddr = const_FPGADACs3; 
    
    offset -=addr030;                       
    offset <<=2;
    fAddr+= offset;
    FRAM_RD(fAddr, (uint8*)fData, 4);    //reads 'cnt' bytes
    FRAM_RD(fAddr+0x60,(uint8*)peds,2);  //reads 'cnt' bytes
}



//DAC CONSTANTs correction table
//each addr 0,1,2...47 stores longword
//addr param must be valid
//Write 32 bit data to FRAM
void cFRAMwr(int fAddr, int short *fData)
{
    FRAM_WR_ENABLE();                       //WEL bit clr on 'CS' rising
    //store constants, addr range 0x030-047
    if((fAddr>=addr030) && (fAddr<=addr047))
        fAddr= const_FPGADACs0+ fAddr;      //form dest address was3000
    //store constants, addr range 0x430-447
    else if((fAddr>=addr430) && (fAddr<=addr447))
        fAddr= const_FPGADACs1+ fAddr;      //form dest address was3000
    //store constants, addr range 0x830-847
    else if((fAddr>=addr830) && (fAddr<=addr847))
        fAddr= const_FPGADACs2+ fAddr;      //form dest address was3000
    //store constants, addr range 0xC30-C47
    else if((fAddr>=addrC30) && (fAddr<=addrC47))
        fAddr= const_FPGADACs3+ fAddr;      //form dest address was3000
    FRAM_WR(fAddr, (uint8*)fData, 4);      //write 4 bytes
}


#define fDACcnt  24
//apply scale factor to input data
void cFRAMfill(int short *fData, int short *Ped)
{
    //store constants, addr range 0x030-047
    for(int i=0,k=const_FPGADACs0+(fDACcnt<<2),j=const_FPGADACs0; i< fDACcnt; i++, j+=4,k+=2)
        {
        //WREN op-code must be issued prior to any Wr_Op  
        FRAM_WR_ENABLE();
        FRAM_WR(j, (uint8*)fData, 4);     //16bit writes (wr 4 bytes) 
        FRAM_WR_ENABLE();
        FRAM_WR(k, (uint8*)Ped, 2);     //16bit writes (wr 4 bytes) 
        }

    //store constants, addr range 0x430-447
    for(int i=0,k=const_FPGADACs1+(fDACcnt<<2),j=const_FPGADACs1; i< fDACcnt; i++, j+=4,k+=2)
        {
        //WREN op-code must be issued prior to any Wr_Op  
        FRAM_WR_ENABLE();
        FRAM_WR(j, (uint8*)fData, 4);     //16bit writes (wr 4 bytes) 
        FRAM_WR_ENABLE();
        FRAM_WR(k, (uint8*)Ped, 2);     //16bit writes (wr 4 bytes) 
        }

    //store constants, addr range 0x830-847
    for(int i=0,k=const_FPGADACs2+(fDACcnt<<2),j=const_FPGADACs2; i< fDACcnt; i++, j+=4,k+=2)
        {
        //WREN op-code must be issued prior to any Wr_Op  
        //WREN op-code must be issued prior to any Wr_Op  
        FRAM_WR_ENABLE();
        FRAM_WR(j, (uint8*)fData, 4);     //16bit writes (wr 4 bytes) 
        FRAM_WR_ENABLE();
        FRAM_WR(k, (uint8*)Ped, 2);     //16bit writes (wr 4 bytes) 
        }

    //store constants, addr range 0xC30-C47
    for(int i=0,k=const_FPGADACs3+(fDACcnt<<2),j=const_FPGADACs3; i< fDACcnt; i++, j+=4,k+=2)
        {
        //WREN op-code must be issued prior to any Wr_Op  
        //WREN op-code must be issued prior to any Wr_Op  
        FRAM_WR_ENABLE();
        FRAM_WR(j, (uint8*)fData, 4);     //16bit writes (wr 4 bytes) 
        FRAM_WR_ENABLE();
        FRAM_WR(k, (uint8*)Ped, 2);     //16bit writes (wr 4 bytes) 
        }
}





float poePower(int poeprt)
{
    float cur, vlt;
    //Display POE register 'current and voltage'
    cur= POE_CUR_ALL[poeprt];
    //Current, 1 LSB = 61.035µA when RSENSE= 0.5O or 122.07µA when RSENSE= 0.25O
    cur= cur * .00012207;
    vlt= POE_VLT_ALL[poeprt];
    vlt= vlt * .005760;  //.005835;
	return (cur*vlt);
}



//see CDCUN1208LP datasheet for info on data array
#define CLKFANOUT    *(sPTR)(fpgaBase0+ (0x42*2))   //CDCUN1208LPRHBR read/write 16bit
uint16 CLKINITDATA[]= {0x0398,0x0398,0x0398,0x0398, 0x0398,0x0398,0x0398,0x0398,
                       0x0398,0,0,1,  0,0,0,2 };  // was 0x0398,0,0,9,  0,0,0,2 }; tek 7/10/24 from 9 to 1
//
//Init 'CDCUN1208LPRHBR' Clock(FM) Fanout Buffer via FPGA Configured SPI Port
//Init 3 'CDCUN1208LP' chips
int ClkDrvInit()
{
    int i;
    for (i=0; i<16; i++)
        REG16((&CLKFANOUT)+fPtrOffset2)= CLKINITDATA[i]; 

    for (i=0; i<16; i++)
        REG16((&CLKFANOUT)+fPtrOffset3)= CLKINITDATA[i]; 

    for (i=0; i<16; i++)
        REG16((&CLKFANOUT)+fPtrOffset4)= CLKINITDATA[i]; 
    
  return 0;
}


//Set power up FPGA Registers and Registered I/O, PLL Clock 'ADF4001'
//Init 'ADF4001' chip
//
void InitFPGA_REGISTERS()
{
    //setup PLL Chip
    //40Mhz Ref, 100Mhz feedback
    //wr16FPGA(0x17,0);
    //wr16FPGA(0x18,0x0050);    //was 0x164
    //wr16FPGA(0x18,0x3201);    //was 0xB401

    //40Mhz Ref, 160Mhz feedback (using new vxo as of Sept2019)
    wr16FPGA(0x17,0);
    wr16FPGA(0x18,0x0050);      // div by 20 addr 0
    wr16FPGA(0x18,0x5001);      // div by 80 addr 1

    wr16FPGA(0x17,0x12);
  //wr16FPGA(0x18,0x42);
  //pll locks now Feb 2024
    wr16FPGA(0x18,0x12);   
        
    //Rx Enable
    wr16FPGA(0x400,0x9);
    wr16FPGA(0x800,0x9);
    wr16FPGA(0xC00,0x9);
    
    //enable phy rec data auto move to sdRam via fpga code
    wr16FPGA(0x400,0x29);   //0x400 FPGA2 CSR
    wr16FPGA(0x800,0x29);   //0x800 FPGA3 CSR
    wr16FPGA(0xC00,0x29);   //0x800 FPGA3 CSR
    ClkDrvInit();           //init 'CDCUN1208LPRHBR' chip
    
    //adf4001 PLL power up 'tek added 7-15-24'
    wr16FPGA(0x19,0x01);        //data 0=enable,1=disable 
    wr16FPGA(0x19,0x00);        //data 0=enable,1=disable 
    
}                    





//empty all lvds receive fifos
//
uint16 ClrFmRecFIFO(void)
{
    u_16Bit rdat;
    u_16Bit* wrdUsed;                   //lvds fifo words used
	
     //Try global reset to clr old lvds fifo data, if this works us it 
    *(uSHT*)IOPs[POE01].FM41_PARp= FMRstBit8;//FPGA2 lvds fifo buf and parErr clr              
    *(uSHT*)IOPs[POE09].FM41_PARp= FMRstBit8;//FPGA3 lvds fifo buf and parErr clr               
    *(uSHT*)IOPs[POE17].FM41_PARp= FMRstBit8;//FPGA4 lvds fifo buf and parErr clr               
    
    for(int k=1; k<=24; k++)            //24 ports with up to 8 pmt boards per port                      
		{
        if (POE_PORTS_ACTIVE[k]==1)     //added to ignore ports that are not active
			{
            wrdUsed= (u_16Bit*)(IOPs[k].FM30_DATp)+(0x08); //lvds fifo words used              
            while (*wrdUsed) 
              rdat= *(u_16Bit*)IOPs[k].FM30_DATp; //read data in lvds fifo, expect 'ACK' from PMT   
			}
        }  
  return rdat;
}


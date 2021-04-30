//Help Info on RM48L compile and link
//tek 2016


// ---------  BEGIN COMMENT INFO FOR PROJECT BUILDING HELP --------
// ---------  BEGIN COMMENT INFO FOR PROJECT BUILDING HELP --------

//*********************************************************************
//  Hercules? ARM® Safety MCUs - HALCoGen      
//  v4.05.02.00                
//  Build Date:  2016-03-02
//*********************************************************************
//---------------------------------------------------------------------
//New in This Version (4.05.02)
//---------------------------------------------------------------------
//- Bug Fixes and few GUI Enhancements.
//- PLL supports floating point multiplier values.
//- Optimized EMAC driver to support inline swizzle function.
//- Added API?s to find the Configured Speed Status from EMAC phy.
//- Added HALCoGen WIKI Link to Start page.
//- Optimized MIBSPI driver code based on Transfer Group selection in GUI
//
//
//Note1:
//----- 
//For all HALCoGen FreeRTOS based projects used with CCS, in the Compiler options under 
//Advanced Options -> Language Options -> ?Enable support for GCC Extension (--gcc)?.
//
//Note2:
//----- 
//For TMS570LC43x and RM57x Family of Devices Safety Functions are support only in SafeTI 
//Diagnostic Library version 2.1.0 which can be installed along with HALCoGen 4.01.00 or latest.
//For using SafeTI Diagnostic Library with HALCoGen please refer Examples --> example_SafetyLib.cin 
//following Help file C:\ti\Hercules\HALCoGen\v04.05.00\help\TMS570LC43x.chm 
//(or) //C:\ti\Hercules\HALCoGen\v04.05.00\help\RM57Lx.chm 

//define example
//#define BUF   ((struct arp_hdr *)&uip_buf[0])
//#define IPBUF ((struct ethip_hdr *)&uip_buf[0])


// IAR STACK VIEWING
// To view the graphical stack bar:
// 1  Choose Tools>Options>Stack.
// 2  Select the option Enable graphical stack display and stack usage.
// You can open up to two Stack windows, each showing a different stack—if 
// several stacks are available—or the same stack with different display settings.

/*
SILABS SI5338C CLOCK GEN (uses I2C Programming interface)
The mask for each register number which when applied to registermap.h will 
automatically mask reserved bits and only allow the user to modify the "un-reserved" bits.

Clockbuilder
Desktop can also measure the current delivered by the EVB regulators to each supply 
voltage of the Si5338. A Si5338 configuration can be written to a text file to be
used by any system to configure the Si5338 via I2C. ClockBuilder Desktop can be downloaded 
from www.silabs.com/ClockBuilder and runs on Windows XP, Windows Vista, and Windows 7.
*/


//---------------------------------------------------------------------------
//after a 'HALCODEGEN' build the following stack fix may be needed 
//change stacks in sys_core.asm on each HAL-CODE-GEN --base- 0x08038000 for RM48L952

//new projects need this mode in *.icf file
//tek *** change linker file.icf from mem:__ICFEDIT_intvec_start__ { readonly section .intvec};
//tek *** change linker file.icf to   mem:__ICFEDIT_intvec_start__ { readonly section .intvecs};

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

//after a 'HALCODEGEN' build 'for UART/LIN usb data' 
//in file 'sci.c', change following structure 
//from 'static   struct g_sciTransfer'
//to            'struct g_sciTransfer'


//after a 'HALCODEGEN' build 'for PHY on RJ45 receive data' 
//add this code in function 'void EMACReceive(hdkif_t *hdkif)'
//function in 'emac.c' code mod to 'void EMACReceive(hdkif_t *hdkif)' 
//after this line --- rxch_int->free_head = curr_bd;
//add following lines
//function 'void EMACReceive(hdkif_t *hdkif)' 
//after this line --- rxch_int->free_head = curr_bd;
//add following lines
/*
      if (phyHdr.enable)
          {
          rxptr= rxch_int->free_head->bufptr;       //tek
          rxCnt= rxch_int->free_head->bufoff_len;   //tek
          //if(rxCnt>MsgBytMax)                     //MsgBytMax==64
          //  rxCnt=MsgBytMax;
          phyHdr.msgBytes[phyHdr.msgCnt]= rxCnt;
          //16bit moves, cnt is in word count Note: limit stored bytes to 'MsgBytMax'
          movStr16((sPTR)rxptr,(sPTR)&phyHdr.msgStr[phyHdr.msgCnt], MsgBytMax/2);  //rxCnt/2);
          phyHdr.msgCnt++;
          if (phyHdr.msgCnt>=40)
            phyHdr.msgCnt=0;
          }
      g_EMACRxRdy=1;                                //tek
*/


//file mode 'emac.c'
//tek comment out 'EMACLinkSetup()' to speed things up 02-27-15
//after 'Dp83640LinkStatusGet()' comment out 'EMACLinkSetup()' see below
  //if(EMACLinkSetup(hdkif) != EMAC_ERR_OK) {
  //  retVal = EMAC_ERR_CONNECT;
  //} else {
  //}


//file mode 'emac.c'
//ADD ONE LINE AFTER
  /*Initialize the EMAC, EMAC Control and MDIO modules. */
  //EMACInit(hdkif->emac_ctrl_base, hdkif->emac_base);
  //MDIOInit(hdkif->mdio_base, MDIO_FREQ_INPUT, MDIO_FREQ_OUTPUT);
//ADD  
  //EMACRxPromiscuousEnable(hdkif->emac_base, 0); //(base, ch)   'Enable Promiscuous Mode'




//The RXPROMCH bit in RXMBPENABLE selects the promiscuous channel to receive frames selected by
//the RXCMFEN, RXCSFEN, RXCEFEN, and RXCAFEN bits. These four bits allow reception of MAC
//control frames, short frames, error frames, and all frames (promiscuous), respectively.


//point type
//This pointer is used by the HET driver to access the het module registers.
//#define hetREG1 ((hetBASE_t *)0xFFF7B800U)

//ref material below from file 'reg_system.h'
/*
typedef volatile struct systemBase2
{
    uint32 PLLCTL3;        // 0x0000 
    uint32  rsvd1;         // 0x0004 
    uint32  STCCLKDIV;     // 0x0008 
    uint32  rsvd2[6U];     // 0x000C 
    uint32  ECPCNTRL0;     // 0x0024 
    uint32  rsvd3[5U];     // 0x0028 
    uint32  CLK2CNTL;      // 0x003C 	
    uint32  VCLKACON1;     // 0x0040 
    uint32  rsvd4[11U];    // 0x0044 
    uint32  CLKSLIP;       // 0x0070 
    uint32  rsvd5[30U];    // 0x0074 
    uint32  EFC_CTLEN;     // 0x00EC 
    uint32  DIEIDL_REG0;   // 0x00F0
    uint32  DIEIDH_REG1;   // 0x00F4  
    uint32  DIEIDL_REG2;   // 0x00F8
    uint32  DIEIDH_REG3;   // 0x00FC
} systemBASE2_t;  
*/


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


//*?**?**?**?**?**?**?**?* DON'T FORGET THIS  *?**?**?**?**?**?**?**?*
//*?**?**?**?**?**?**?**?* DON'T FORGET THIS  *?**?**?**?**?**?**?**?*
//change stacks in sys_core.asm on each HAL-CODE-GEN --base- 0x08038000 for RM48L952
//change stacks in sys_core.asm on each HAL-CODE-GEN --base- 0x08038000 for RM48L952

//tek *** change linker file.icf from --place at address mem:__ICFEDIT_intvec_start__ { readonly section .intvec};
//tek *** change linker file.icf to   --place at address mem:__ICFEDIT_intvec_start__ { readonly section .intvecs};
// ****** use local copy of modified linker file 'RM48L952.icf'  *******
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

/*this is good in file 'sys_core.asm'
userSp  dcd 0x08038000+0x00001000
svcSp   dcd 0x08038000+0x00001000+0x00000100
fiqSp   dcd 0x08038000+0x00001000+0x00000100+0x00000100
irqSp   dcd 0x08038000+0x00001000+0x00000100+0x00000100+0x00000100
abortSp dcd 0x08038000+0x00001000+0x00000100+0x00000100+0x00000100+0x00000100
undefSp dcd 0x08038000+0x00001000+0x00000100+0x00000100+0x00000100+0x00000100+0x00000100
*/


/*
//default halcogen build as of july 2015 in file 'sys_core.asm'
userSp  dcd 0x0803EB00+0x00001000
svcSp   dcd 0x0803EB00+0x00001000+0x00000100
fiqSp   dcd 0x0803EB00+0x00001000+0x00000100+0x00000100
irqSp   dcd 0x0803EB00+0x00001000+0x00000100+0x00000100+0x00000100
abortSp dcd 0x0803EB00+0x00001000+0x00000100+0x00000100+0x00000100+0x00000100
undefSp dcd 0x0803EB00+0x00001000+0x00000100+0x00000100+0x00000100+0x00000100+0x00000100

//manually modified halcogen code as of july 2015  in file 'sys_core.asm' 
//fill down from top of RAM at 0x0803FFFF
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
End of ICF editor section. ###ICF###*/


//nHet 
//nHET files, HalCoGen v4.03 now requires '_TMS470_BIG' for compile to work
//in file 'std_nhet.h' else missing def for ''HET_MEMORY' and more
//added _TMS470_BIG in IAR project, compiler, preprocessor defined symbols


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

// ---------  END COMMENT INFO FOR PROJECT BUILDING HELP --------
// ---------  END COMMENT INFO FOR PROJECT BUILDING HELP --------

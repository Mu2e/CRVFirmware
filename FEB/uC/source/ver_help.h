// Help mu2e RM48L 'ver_help.h'
// Fermilab Terry Kiper 2016-2020

#ifndef _VER_HELP
#define _VER_HELP

const   char HelpMenu[]={
               "HELP H1:     ------------------  Mu2E CRV FRONT END BOARD --------------------\r\n\n"  
               "Read ASCII\r\n"
               "  RDC         Read Spill Data Word Count(H) plus 8 word header\r\n"
               "  RD  adr     FPGA Addr, Adr16Hex, returns=Data16(H), 'ECHO Off'\r\n"
               "  RDI adr c   FPGA Read/Incr A16H,  c=WrdCnt(H)  Def=10H, Max=100H\r\n"
               "  RDM adr c i FPGA Read/NoIncr FPGA A16H,  c=WrdCnt(H), i=WrdsPerline\r\n"
               "  RDS adr c i FPGA Reads A16H, 2Chars+space,c=WrdCnt(H),i=WrdsPerline\r\n"
               "\nRead BINARY\r\n"
               "  RDB wCnt r  Read Spill Data (16bit BIN), WCnt(H)= # words to Read\r\n"
               "              if r=1 Data Read Ptrs Unmodified, Def=0=Reset Ptrs\r\n"
               "              1st Sends 8 word Control Header then Data\r\n"
               "              if no WrdCnt RDB returns WrdCnt= Last Spills Word Cnt\r\n"
               "              if WrdCnt=0, Stop any Active Req, WrdCntMax=0x2000000\r\n"
               "  RDBR a c    Read any A16H(no Incr), Rtns Bin 16bit Data,c=#Wrds(H) to Rd\r\n"
               "              if Adr a=0, stop any active RDB or RDBR cmd\r\n"
               "Write\r\n"
               "  WR  a d     FPGA Write Addr16H, Data16H, 'New Line ECHO Off'\r\n"
               "  WRI a d c   FPGA Write Addr16H with Incr, Data16H, c=Repeat WRI Cnt(D)\r\n"

               "AFE Chips\r\n"
               "  AFERESET f  Soft Reset on AFE chips per fpga, f=(1-4,5=all)\r\n"
               "  PWR n       ENA/DISABLE, 2 AFE5807s per FPGA, 0=ALL OFF, 1,2,3,4 (5=all on)\r\n"

               "\nFLASH         See Help Menu 'HF'\r\n"
               "FPGA\r\n"
               "  FT n        Re-load FPGA with Flash data, n=fpga 0-3 (4=ALL)\r\n"
               "  FI          Reset all FPGAs using ProgramB Input\r\n"
               "  FS          FPGA download Size, SumCheck, FLASH ID\r\n"

               "\nMisc\r\n"
               "  ADC n       Read uC ADC Chs and Temp, Read twice to update, n=1=noText\r\n"
               "  A0 n        Read ADC ADS1259(.1SecDly), Read n times(def=1,Max=10) Rtns(flt)\r\n"
               "  DACs        Use command 'H2' for DAC help\r\n"
               "  GAIN g r    Set PGA280 Gain= 1,2,4,8,16,32,64,128 (r=1,Reset PGA280)\r\n" 
               "  ID c        Display code versions and serial number, c=Clear ECC Count\r\n"
               "  LINK s      RJ45 LVDS Link direction, 0=REC, 1=XMIT\r\n"
               "  MUX f       Mux select on ADG1609 differential input, f=fpga(0-3)\r\n"
               "  TRIG s      External Input Source, 0=RJ45(CLK/TRIG), 1=LEM0(TRIG)\r\n"
               "  ECHO p      Newline Echo '>', p(0=OFF, 1=ON), Always off for Cmds 'RD','WR'\r\n"
               "  OVC  p      OverVoltCur Status as 0=OK, 1=TRIP, P=1=(Clears Trip)\r\n"

               "\nSave/Restore\r\n"
               "  DSAV        Device Setup Save to FLASH (one page available)\r\n"
               "  DREC        Device Setup Recall from FLASH,('DREC 0' Restores uC constants)\r\n"

               "Socket        See Help Menu 2, cmd H2\r\n"

               "Status\r\n"
               "  CMBENA c    Enable CMB data reads, c=1=ENA, 0=DISABLE\r\n"
               "  CMB t       Read 16 Chs TEMP+ROM, (Refresh=320mS), t=1=Show data only\r\n"
               "  HDR c       Spill Header, Updated by RDB Cmd, (c=1 clrs UserReqOvr cnt)\r\n"
               "  STAT        FPGA Setup Registers and CMB Bias Voltages\r\n"
               "  STAB f      DAQ Status Data(H),22_Wrds_uC+38_WrdsPerFPGA,f=1-4,def=4,0=HELP\r\n"
               "  UBUN        Display uBunch Requests Counters, Counters cleared\r\n\n"
               "HELP          HE(HELP1), H2(HELP2), HF(FLASH), HS(STATUS), HT(TEST), HA(ADRMAP)\r\n\n"
            };

const   char HelpMenu2[]={
               "HELP H2:     ------------------  Mu2E CRV FRONT END BOARD --------------------\r\n\n"
               "AFE Chips R/W\r\n"
               "  AFERD f a c Read AFE Reg,  f=fpga(1-4), A16H=(0x100...), c=WordCnt\r\n"
               "  AFEWR f a d Write AFE Reg, f=fpga(1-4), A16H=(0x100...), d=Data16\r\n\n"
               "DACs          ADDR   BiasTrim 30-3f, LED 40-43, BiasBus 44-45, VGA 46-47\r\n"
               "              SLOPE  16BIT, FFFF=+4096mV, 8000=0V, 0000=-4096mV\r\n"
               "              OFFSET 12BIT, 07FF=+2048mV, 0000=0V, 0FFF=(-1mV), 800(-2048mV)\r\n\n"
               "  DRD a       Reads DAC A16H, rtns=Data16H (Applys Inv Offset, Slope)\r\n"
               "  DWR a d     Write DAC A16H, d=0-FFF, (Applys Slope, Offset)\r\n"
               "  DSF a s o   Write Scale Factor A16H, s=Slope(Data16H), o=Offset(Sign16BitH)\r\n"
               "  DSI s o     Init all Slp,Off, s=Slope, o=Offset (if s=99 Clr S&O FRAM Data)\r\n"                 
               "  DSR a       Read Scale Factor A16H, rtns Slope(Data16H), Offset(Data16H)\r\n"
                 
               "\nMisc\r\n"
               "  EQR  a c    RD Equalizer Chip GS3140 Addr16H(BIT15 L=R,H=W), c=wrd count\r\n"
               "  EQW  a d    WR Equalizer Chip GS3140 Addr16H(BIT15 L=R,H=W), Data16H\r\n"
               "  EQRESET     RESET Equalizer Chip GS3140\r\n"
               "  RESET       uC Reset, if USB fails on restart Dis/Connect USB cable\r\n"
               "  SB b        Set Baud, 1=115K, 2=230K, 3=460K, 4=920K (saved in FRAM)\r\n"                 
               "  SN pwd n    Serial Number, pwd=123, n=0-9999(D), 'SN' reads Ser#(Dec)\r\n"
             //"  TH          Display on board temperature reading\r\n"
                
               "\nSocket Setup Save/Restore\r\n"
               "  NETSAV      Network Setup Save to flash\r\n"
               "  NETRST      Network Setup restore defaults\r\n"
               "\nSocket\r\n"
               "  CLOSE s     Close Network Socket, s=socket 0-3, Default=this connection\r\n"
               "  SET         Network Setup Registers\r\n"
               "  SOCK t      Display Network Socket Status, t==1 Extended timout status\r\n"
             //"  RDEBLK n    Wiznet Block Read 256 bytes, n=(Blk 0-3),1-3 Req Reset when done\r\n" //expert testing, requires reset after use
               "  WG          Wiznet ARP GateWay Init, (hangup testing on prototype)\r\n"
               "  WI          Wiznet Reset and re-initialize (hangup testing on prototype)\r\n"

               "\nFRAM         See Help Menu 'HF', (Stores saved power up register values)\r\n"
                   
               "\nnError\r\n"
               "  ER0         Clears ECC Reboot Counter\r\n"
               "  ER1         Trigger Single Bit ECC error, No reset, Clears nErr LED\r\n"
               "  ER2         Trigger Double Bit ECC error, uC Hardware Re-Boots Board\r\n\n"
               "HELP          HE(HELP1), H2(HELP2), HF(FLASH), HS(STATUS), HT(TEST), HA(ADRMAP)\r\n\n"
            };


const   char HelpMenu3[]={
               "HELP HT:     --------------  HT HELP TESTING MENU, Mu2E CRV FEB --------------\r\n\n"
               "SDram\r\n"
               "  SD f s     SDram Memory Test, f=1-4, s=sizeWords(H) (Default f=1, s=Max)\r\n"
               "             MT46H128M16LFDD  Size 128Meg x 16, Max=0x8000000\r\n"

               "\nPHY TEST\r\n"
               "  PACE e     DP83640 PHY, Enable Capture buffers (ena=1,dis=0)\r\n"
               "  PACR c     DP83640 PHY, Read Capture buffers (count=1-100)\r\n"
               "  PACS l p   DP83640 PHY, Send Pack, l=lpCnt(D), p=PayLdSz(Evn>2, Max=1500(D)\r\n"

               "\nFM LVDS XMIT\r\n"
               "  FMS c p    LVDS FM Send, c(H)=cnt to send, p(H)=initial data to send\r\n"

               "\nMDIO\r\n"
               "  PR         MDIO Physical Link Read Reg\r\n"
               "  PS         MDIO Physical Link Status\r\n"

               "\nTrig Setup Software\r\n"
               "  TRIG 1     LEMO SRC\r\n"
			   "  Check Reqs 305=80, 306=0, 307=237, 308=2, 309=4\r\n"
               "  WR 303 301 Single spill, TestPulse Ena, TrigSrc\r\n"
               "  RD 76      Spill Status, 2=done\r\n"

			   "\nTrig Setup External (50nS 50ohm 3 Vlts)\r\n"
			   "  TRIG 1     LEMO SRC\r\n"
			   "  Check Reqs 305=80, 306=0, 307=0, 308=2, 309=4\r\n"
			   "  WR 303 300 Single spill, TestPulse Ena, TrigSrc 'Pulse'\r\n"
			   "  Trigger    Extern Pulse\r\n"
               "  RD 76      Spill Status, 2=done\r\n"
               "\nDebug uBunch Errors\r\n"
               "  Debug p    Echo uB Errors to Port, 1=sock1, 2=sock2, 3=USB, else 'End Debug'\r\n"
               "             For testing only, will slow down packet handling if errors\r\n"
               "             If using USB best to set baud rate to max, (4=920K)\r\n\n"
               "HELP         HE(HELP1), H2(HELP2), HF(FLASH), HS(STATUS), HT(TEST), HA(ADRMAP)\r\n\n"

            };


const   char HelpMenuFLASH[]={
               "HELP HF:     ----------- FLASH MEMORY LOADING/PROGRAMMING FEB ----------------\r\n\n"                 
               "USB Port    - Programs FPGA FLASH\r\n"
               "  FL1         Flash Loader, Erase/Load Xilinx bin file, Resets FPGA(s)\r\n\n"
                 
               "SOCKET Port - Programs FPGA FLASH\r\n"
               "  FLSOCK      Flash Loader, Erase/Load Xilinx bin file, Resets FPGA(s)\r\n\n"
                                  
               "SOCKET Port - Programs FPGA FLASH, FPGAs must already be Configured\r\n"                 
               "  LDFILE s    1st, Load Binary file to SdRam in 1of4 FPGAs, (s=1,2,3 or 4)\r\n"
               "  LDFLASH     2nd, Program Flash using FPGA1 memory, Pgm cycles take ~20 Secs\r\n"                 
               "  LDSTAT      Display LD FLASH Status Registers\r\n\n"                                                   
                 
               "\n---------------- FLASH COMMANDS --------------------------------------------\r\n"
               "Flash Misc\r\n"
               "  FCOPY n     Copy FPGA Image to FLASH Backup, n==1=Copy else rtn Status\r\n"
               "  FCOMP       Compare FLASH Main FPGA data to Backup\r\n"
             //"  FERASE      Complete FLASH Erase, take 70 seconds\r\n"
               "  FZ c        Check/Display un-erased FLASH data, c=word cnt(H) to check\r\n"
               "  RFI a c     Read/Incr Flash at a=WordAddr(H), c=WrdCnt(H)\r\n"
               "  FS          Read FPGA download Size, SumCheck, FLASH ID\r\n\n"
                 
               "\n---------------- FRAM ADDRESS MAP for FPGA and NETWORK SAVED SETUP ---------\r\n"
               "FRAM\r\n"
               "  Address     0=FPGA1, 100=FPGA2, 200=FPGA3, 300=FPGA4\r\n"
               "              400=uC_Regs, 500=Network, 600=FPGA DownLoads Status\r\n"           
               "  FDUMP adr   FRAM, Display memory block (128 words), a=A16H(13bits,8KBytes)\r\n"
               "  FRS         FRAM, Status register read\r\n"
               "  FRD a       FRAM, Read  a=A16H(13bit) def=0, rtns data16  'FRS'=Chip Status\r\n"
               "  FWR a d     FRAM, Write a=A16H(13bit), d=Data16H\r\n\n"
                 
               "\n---------------- FAKE DATA LOADING, FLASH PGM OPTION -----------------------\r\n"
               "Fake Data\r\n"
               "  LDFE        Erase Fake Data Sectors in Flash (~65Secs)\r\n"
               "  LDFP        Programs Fake data from SDram Addr 0x007 to FLASH(~10Secs)\r\n"
               "  LDFC        Copy Fake Data from FLASH to sdRam, (~60 Seconds)\r\n"
               "  RFI 110000  Reads 1st 256 words Flash Fake data, BaseAdr=110000, RFI Adr Cnt\r\n\n"
               "HELP          HE(HELP1), H2(HELP2), HF(FLASH), HS(STATUS), HT(TEST), HA(ADRMAP)\r\n\n"
            };


uint8  mu_BOOT_MSG1[]= {"\r\nMU2E FRONT END READOUT BOARD, TI-RM48\r\n\n"};
const uint8  mu_BOOT_MSG2[]= {"Network Setup InValid, Loading Defaults\r\n"};
const uint8  mu_BOOT_MSG3[]= {"EMACHWInit-> Dp83640LinkStatusGet Failed, Data cable(rj45) connected?\r\n"};
const uint8  mu_BOOT_MSG4[]= {"EMACHWInit-> Dp83640LinkStatusGet Passed\r\n"};

const   char *adcName[]= {"1.2v_Pos", "1.8v_Pos", "5.0v_Pos",  "10v_Pos ",  "2.5v_Pos", "5.0v_Neg",  "15v_Pos ", "3.3v_Pos",  "Bias_0  ","Bias_1  ","Bias_2  ", "Bias_3  ", "Bias_4  ","Bias_5  ","Bias_6  ", "Bias_7  ", "Temp_C  "};
const   float adcScale[]={  .0010,      .0010,     .0020,       .0040,      .0010,      .0020,        .0060,       .0010,     .02104,     .02104,    .02104,     .02104,     .02104,     .02104,    .02104,      .02104 };
 
const   char *sHdrTxt[]= {"Trim Dac00","Trim Dac01","Trim Dac02","Trim Dac03","Trim Dac04","Trim Dac05","Trim Dac06","Trim Dac07",
                          "Trim Dac08","Trim Dac09","Trim Dac10","Trim Dac11","Trim Dac12","Trim Dac13","Trim Dac14","Trim Dac15",
                          "LED PULDac00","LED PULDac01","LED PULDac02","LED PULDac03","BiasBusDac00","BiasBusDac01",
                          "AFE0 GainDac","AFE1 GainDac",
                          "TEMP CMB 0","TEMP CMB 1","TEMP CMB 2","TEMP CMB 3",
                          "AFE0 REG01","AFE0 REG02","AFE0 REG33","AFE0 REG34","AFE0 REG35",
                          "AFE1 REG01","AFE1 REG02","AFE1 REG33","AFE1 REG34","AFE1 REG35"
                        };

//byte count for "GateWay","NetMask","MacAddr","ipAddr", sock0, sock1,sock2
const   int   netVars[]= {4, 4, 4, 6, 1,1,1};   
const   int   AFEadr[]= {0x101, 0x102, 0x133, 0x134, 0x135, 0x201, 0x202, 0x233, 0x234, 0x235};   



const   char HelpMenuSTAB[]={
               "STAB CMD, Data Block Format (16BIT words)\r\n"
               " 22 Word Block (times 1)\r\n"
               "    1 ----- Serial Number\r\n"
               "    1 ----- Spill Cycle Count\r\n"
               "    1 ----- Temperature\r\n"
               "   16 ----- ADC Channels (16)\r\n"
               "    1 ----- FPGA ADDR 303, Trig Control Reg\r\n"
               "    1 ----- FPGA ADDR 304, Pipeline Delay\r\n"
               "    1 ----- FPGA ADDR 305, Sample Length\r\n"
               "\r\n"
               " 38 Word Block (times 4)\r\n"
               "   24 ----- AFE DAC regs        30-3F BiasTrim, 40-43 LED, 44-45 BiasBus\r\n"
               "    4 ----- AFE Temperatures\r\n"
               "   10 ----- AFE UltraSound Regs\r\n\n"
               };
                 

//fpga address map as of 02-13-2020
const   char HelpMenuAdrMap[]={
"       FPGA Address Map as of 02-13-2020\r\n"  
"0x000: Control and status register\r\n"
" Bit0: Power Down AFE 0. 0: Run. 1: Power down.\r\n"
" Bit1: Power Down AFE 1. 0: Run. 1: Power down.\r\n"
" Bit2: Issue a reset to the AFE deserializer logic in the FPGA\r\n"
" Bit3: Issue a MIG DDR interface reset\r\n"
" Bit4: Reset readout sequencer 1: Reset, 0: No action\r\n"
" Bit5: Issue a general reset. \r\n"
" Bit6: Reset the serial controller in the AFE chips.\r\n"
" Bit7: Clear FM receive parity error.\r\n"
" Bit8: Trigger pedestal averager\r\n"
"\r\n"

"0x002: Set SDRam Write address upper bits\r\n"
"0x003: Set SDRam Write address lower bits\r\n"
"0x004: Set SDRam Read address upper bits\r\n"
"0x005: Set SDRam Read address lower bits\r\n"
"0x006: Byte swapped SDRam read data port\r\n"
"0x007: Un-swapped SDRam data port\r\n"
"0x008: MIG Status Register\r\n"
"0x009: MIG FIFO Count\r\n"
"0x00A: LVDS Transmit FIFO\r\n"
"0x00B: LVDS Transmit FIFO Empty Flag\r\n"
"0x00C: Read Event FIFO\r\n"
"0x00D: Read Event FIFO words used\r\n"
"0x010: Histogram control register\r\n"
"\r\n"

"0x011: Histogram accumulation interval\r\n"
"0x012: Histogram pedestal offset\r\n"
"0x014: Read/Write AFE 0 histogram memory read pointer\r\n"
"0x015: Read/Write AFE 1 histogram memory read pointer\r\n"
"0x016: Read AFE 0 histogram memory data port\r\n"
"0x017: Read AFE 1 histogram memory data port\r\n"
"0x020: I/V ADC Input Multiplexer control register\r\n"
"0x021: Channel Mask Register\r\n"
"0x022: Read/Write test Counter Bits 31...16\r\n"
"0x023: Read/Write test Counter Bits 15...0\r\n"
"0x024: One wire command register\r\n"
"0x025: One wire control register\r\n"
" Bit0..3: Selects which CMB read data is stored in the register file\r\n"
" Bit4: Request a write transaction\r\n"
" Bit5: Request a read transaction\r\n"
" Bit6: Request a reset transaction\r\n"
" Bit7: Transaction status. Returns a ‘1’ when transaction is in progress\r\n"
" Bit15..8: Transaction bitcnt(N-1) For WR set to 7, For 72bit RD set to 71\r\n"
"\r\n"

"0x026..0x2A: One wire returned data register file\r\n"
"0x02F: AFE Input FIFO empty flags\r\n"
"0x030..0x3F: Bias trim DAC voltage setting\r\n"
"0x040..0x43: LED flasher intensity DACs\r\n"
"0x044..0x45: Bias bus DACs\r\n"
"0x046..0x47: AFE VGA DACs\r\n"
"0x048..0x5F: DAC setup registers\r\n"
"0x06C: Uptime Counter bits 31...16\r\n"
"0x06D: Uptime Counter bits 15...0\r\n"
"0x080..0x8F: Pedestal registers\r\n"
"0x0FF: AFE setup data read register\r\n"
"0x100..0x13B: AFE 1 register file\r\n"
"0x200..0x23B: AFE 2 register file\r\n"
"0x300: Flash gate control register\r\n"
" Bit0: Enable the flash gate\r\n"
" Bit1: Select the CMB pulse routing. 1: Flash Gate, 0: LED flasher\r\n"
" Bit2: LED Flasher signal source. 0: Test pulser, 1: DCO frequency\r\n"
"\r\n"
"0x301: Flash Gate Turn on time\r\n"
"0x302: Flash Gate Turn off time\r\n"
"0x303: Trigger Control Register\r\n"
"0x304: Read/Write Hit Pipeline Delay Register\r\n"
"0x305: Read/Write Beam On Sample length register\r\n"
"0x306: Read/Write Beam Off Sample length register\r\n"
"0x307: Read/Write Gated Histogram Sample length register\r\n"
"0x30C: Read/Write ADC samples per hit\r\n"
"0x30E: Self Trigger Control Register\r\n"
"0x310: Broadcast of SDRam Read address lower bits\r\n"
"0x311: Broadcast of SDRam Read address upper bits\r\n"
"0x312: Broadcast of SDRam page number upper bits\r\n"
"0x313: Broadcast of SDRam page number lower bits\r\n"
"0x316: Broadcast write the four CSR registers\r\n"
"0x317: Event FIFO status\r\n"
};




//define example for casing struction into memory buffer
//#define BUF   ((struct arp_hdr *)&uip_buf[0])
//#define IPBUF ((struct ethip_hdr *)&uip_buf[0])


//get float numb using scanf expample
//    g_paramPtr= paramPtr;               //init g_paramPtr ptr before 'scanf()'
//    scanf("%f",&slp);                   //requires 26uS, reads float 'slope'
//    paramPtr= g_paramPtr;               //update param ptr after 'scanf()'


#endif

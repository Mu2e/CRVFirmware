//*******************************************************************************
// mu2e CRV Readout Controller 'HELP' + constants
// Fermilab Terry Kiper 2016-2021
//*******************************************************************************


#ifndef _VER_HELP
#define _VER_HELP

const   char HelpMenu[]={
               "HELP H1:  --------------- Mu2E CRV Readout Controller -------------------\r\n\n"
               "READ\r\n"
             //"  RDB1 & RDB2 Read Binary FEB DATA BLOCKS FPGA 1,2\r\n"
               //"  RDB WCnt    Read Spill Data(16bit BINARY), WCnt(H)= # Words to read\r\n"
               //"   ..         if WrdCnt=0 (Stops any Active Req)\r\n"
               //"   ..         no WrdCnt returns all data, use word cnt to limit data\r\n"
               "  RD  a       FPGA A16H, a=A16, returns=Data16\r\n"
               "  RDI adr c   FPGA Read/and Incr, A16H,  c=WrdCnt(D) Default=16\r\n"
               "  RDM adr c i FPGA Read/NoIncr FPGA A16H,c=WrdCnt(D), i=WrdsPerline(def=8)\r\n"
               "  RDBR a c    Read any A16H(no Incr), Rtns Bin 16bit Data,c=#Wrds(H) to Rd\r\n"
               "              if Adr a=0, stop any active RDB or RDBR cmd\r\n"
               "\nWRITE\r\n"
               "  WR  a d     FPGA Write Addr16(H), Data16(H)\r\n"                 
                 
               "\nLINK ACCESS TO FEBs (uses PHY and LVDS links)\r\n"
               "  LP p        Assign 1of24 Ports for 'LC' cmd, List Act Ports, Adds New Prompt\r\n"
               "  LC          Link Command, send cmd to Pre-Selected FEB port\r\n"                 
               "  LCA         Link Command All, send cmd to ALL FEB ports, may causes DAQ Errs\r\n"                 
               "  LI          Link Init, Detects connected FEBs on POE ports using FPGA logic\r\n"                 

               "\nMISC\r\n"
               "  ADC n       Read uC ADC Chs and Temperature, n=1=noText\r\n"
               "  ID          Display code versions and serial number\r\n"
               "  POOL        FEBs Pooled Data Display, see Help 'HT' for more Info\r\n" 
               "  RESET       uC Reset, if USB fails on restart Dis/Connect USB cable\r\n"
               "  SN pwd s c  Set s=Ser#, c=Cntrl#, pwd=123, n=0-999, c=1-19, no input reads\r\n"
               "  TRIG_OLD n  1=On,0=OFF, Fiber Data_Req_Buffer Xmits uBunch Req to FEB\r\n"
               "  TRIG n      1=On,0=OFF, New vers of TRIG, sends 4 word uBun Reqs, faster\r\n"

               "\nFLASH       See Help 'H2' or 'HF' for programming\r\n"
                 
               "\nPOE PORTS\r\n"
               "  PWR         Display POE Ports power readings\r\n"
               "  PWRRST n    Power Reset Port n=1of24, n=25 Power Cycles all POE ports\r\n"  
                 
               "\nSAVE/RESTORE\r\n"
               "  DSAV        Save Device Setup registers to Flash\r\n"
               "  DREC p      Recall/load Device Setup Regs from Flash, p=0(default setup)\r\n"
               "  FDUMP adr   Display FRAM (128 word block), fpga(1-4) adr=0,100,200,300\r\n"
                 
               "\nSOCKET\r\n"
               "  SET         Network Setup Registers\r\n"
               "  SOCK        Display Network Socket Status\r\n"
               "  NETSAV      Network PORT Setup Save to Flash, NETRST restores defaults\r\n"
               "  CLOSE s     Close any active Network Socket, s=Socket 0-3\r\n"
               "  QUIT        Closes this active socket connection\r\n\n"
               "HELP          HE(HELP1),H2(HELP2), HF(FLASH), HN(NETWORK), HT(TEST), HA(ADRMAP)\r\n\n"
                   
};

const   char HelpMenu2[]={
               "HELP H2:  ---------------- Mu2E CRV Readout Controller ------------------\r\n\n"
                                  
               "FPGA\r\n"
               "  FI          RESET all FPGAs using programB signal\r\n"
               "  FT          Flash Data Transfer, Reloads all FPGAs\r\n"
               "  FS          Read FPGA download Size, SumCheck, FLASH ID\r\n\n"

                 "FLASH MISC\r\n"
             //"  FERASE      Flash Erase All, takes 70 seconds\r\n"
               "  FZ c        Check/Display un-erased FLASH data, c=word cnt(H) to check\r\n"
               "  FES s       Flash Erase Sector, s=sector number\r\n\n"
               "  RF  a       Read flash address A16H, returns=Data16\r\n"
               "  RFI a c     Read/Incr Flash address A16H, c=WrdCnt(H)\r\n"
                 
                 
               "FRAM  (SETUP)\r\n"
               "  FDUMP adr   Display FRAM(8KB) (128 word block), fpga(1-4) adr=0,100,200,300\r\n"
               "  FRERASE 1   Erase All\r\n"
               "  FRD a       Read  a=A16H(13bit) def=0, rtns data16  'FRS'=Chip Status\r\n"
               "  FWR a d     Write a=A16H(13bit), d=16bit(hex),note FDUMP cmd under Save\r\n"
               "  FRS         Status register read\r\n\n"

               "MISC\r\n"
               "  CLR,CLS     Clear Screen\r\n"

     //          "\nnError TI-RM48\r\n"
     //          "  ER n        ESM Monitor, (n=default) reads nError Flip/Flop\r\n"
     //          "              ESM Monitor, (n=1) trig ESM single bit error\r\n"
     //          "              ESM Monitor, (n=2) trig ESM double bit error\r\n"
     //          "              ESM Monitor, (n=3) reset ESM error, F/F (1=Normal)\r\n"
               "\nHELP          HE(HELP1),H2(HELP2), HF(FLASH), HN(NETWORK), HT(TEST), HA(ADRMAP)\r\n\n"
};


const   char HelpMenuTest[]={
               "HELP HT: ------ SDRAM TESTING and I/O LINKS (FIBER, LVDS, ePHY) Tests ------\r\n\n"
               "SDram\r\n"
               "  SD f s     SDram Memory Test, f=(2,3,4)  s=sizeWords(H) (def f=2,s=all)\r\n"
               "             ..Max Size Words=0x3fffff0,   MT46H64M16LF (64Megx16)\r\n\n"
               "FIBER_LINK TEST\r\n"
               "  PINIT      1) Set up FPGA Regs to bypass FEBs Normal return data path\r\n"
               "  LDFILE f   2) Load Data File to FPGA SDram 2,3,4 (@Adr 4xx,8xx,Cxx)\r\n"
               "  PTRIG      3) 1 Packet Request, Load Preamble, Payload, ChkSum.. Triggers\r\n"
               "  PTRIG n    3) 6 Packet Req times n, Load Preamble, Payload, ChkSum.. Trigs\r\n\n"

               "LVDS_FM REC\r\n"
               "  PFM p c    Read LVDS(FM) data from Port p (1of24), c=WCnt(D)(def p=1,c=all)\r\n"
               "  POOL c     Display Pooled FEBs Data, 30Sec update,  c=1 Clears Pool Buffer\r\n" 
               "  POOLENA m  Data Pooling, Ena=1, Dis=0, Forces new update within 2 secs\r\n\n" 
                 
                 
                 
               "PHY_RECV     Reads PHY Port Data if available\r\n"
               "  PREC p c   PHY FIFO Read, p=1of24 ports(def=Active), c=wrdcnt(H),def=all\r\n\n"

               "LINK COMMANDS\r\n"
               "  LC cmd     LC cmd uses PHY Link to Send ASCII Cmd string to FEB\r\n"
               "             FEB Replys Sending ASCII data on 'LVDS LINK'\r\n"
               "             Controller will Auto Display this data, else cmd 'PFM p'\r\n\n"
                 
               "PHY PORT TESTING\r\n"
               "  PSEND      Sends cmd 'RDX 100'(100H) to FEB (send on Active Port, cmd 'LP')\r\n"
               "             Returns (last sampled) data on 'PHY LINK' to the ports PHY FIFO\r\n"
               "  PREC p c d Reads 'PHY LINK' REC FIFO data, p=1of24 ports(def=Active Port)\r\n"
               "             c=wrd cnt to read, d= display wrds per line\r\n\n"

               "EMPTY PHY_REC_FIFOs\r\n"
               "  PRECALL    Reads All 24 PHY FIFOs Buffers, or cmd 'PA'\r\n\n"                 
                 
               "READ PHY FIFOs uBunch Data\r\n"
               "  PRECF p c d  Reads PHY FIFO (Formats data), p=1of24 ports(def=Active Port)\r\n"
               "               c=wrd cnt to read, d=display wrds per line(def=12)\r\n\n"
               "uBunch Data Request Testing, Requires Fiber Loopback\r\n"
               "              On FEB see cmd 'HT' for help on debugging uBunch Request\r\n"
               "  UB0 d       Stops any active uBunch Requests\r\n"
               "  UB1 d       Set Delay between uB Requests, d=delay in uS, (Def=20uS)\r\n"
               "  UB2 cnt     Trigs cnt*10 uBun Requests, Loads Reg (F,32,33 Hex) (def cnt=1)\r\n\n"
               "  UB3 dly     Simular to UB2 but Re-triggers, d=delay in uS, (Def=20uS)\r\n"                                  
               "  uB_Rtn_Hdr  4 Word Hdr  1(WordCnt)  2(Status)  3(ubReq#HI)  4(ubReq#LO)\r\n"
               "  uB_Rtn_Hdr  Status Bits=  8(fifo non empty) 7654(ovrflow) 3210(uB Req Err)\r\n\n"
                 
               "HELP          HE(HELP1),H2(HELP2), HF(FLASH), HN(NETWORK), HT(TEST), HA(ADRMAP)\r\n\n"
};



const   char HelpMenu_ORG_TREE[]={
               "HELP HN: ------------- ORANGE TREE ZestETM1 NETWORK TESTING ----------------\r\n\n"               
               "ZEST 16Bit\r\n"
               "  ZSOCK      Read Socket Regs for Ch 1-16 Base Memory, Get 1st 7 words/ch\r\n"
               "  ZRD a      Read  Brd, a=A16\r\n"
               "  ZWR a d    Write Brd, a=A16, d=D16\r\n"
                 
               "\nZEST SPI\r\n"
               "  ZSRDI      SPI Read/Incr Memory, MailBox Memory @ 300(H)\r\n"
               "  ZSWR a d   SPI Write Memory, a=A16, d=D16\r\n"
               "  ZSN        SPI Read Zest Network Setup Info\r\n"
                 
               "\nNETWORK INITs\r\n"
             //"  PWROT     'PWR' Power cycle 'OT' Orange Tree ZestETM1 Network Module\r\n"
               "  ZINIT      Re-Init network 'ZestETM1' module interrupts\r\n"
               "  ZINIT1     Re-Init network 'ZestETM1' module sockets\r\n"
                 
               "\nNETWORK-Windows Cmd 'arp'\r\n"
               "             Use 'arp' to assigns IP number to MAC number\r\n"
               "             arp -s 157.55.85.212   00-aa-00-62-c6-09  Adds a static entry\r\n\n"                 
               "HELP         HE(HELP1),H2(HELP2), HF(FLASH), HN(NETWORK), HT(TEST), HA(ADRMAP)\r\n\n"
                 
};

const   char HelpMenuFLASH[]={
               "HELP HF: -------------- FLASH MEMORY LOADING/PROGRAMMING -------------------\r\n\n"

               "USB PORT Program FPGAs 1,2 FLASH\r\n"
               "   FL1         Program FPGA(1) FLASH with Xilinx binary file, no reboot\r\n"
               "   FL2         Program FPGAs(2,3,4) FLASH with Xilinx bin file, no reboot\r\n\n"

               "SOCKET PORT DIRECT, Programs ROC FLASH\r\n"
               "   FLSOCK1     Erase and Program FPGA(1) FLASH with Xilinx bin file\r\n"
               "   FLSOCK2     Erase and Program FPGA(2-4) FLASH with Xilinx bin file\r\n\n"
                
               "SOCKET PORT Programs ROC FLASH using SD_RAM, FPGAs must already be Configured\r\n" 
               "   LDFILE 2    Step1, Download Data to FPGA2 SD_RAM (DAQ Inactive)\r\n"
               "   LDFLASH     Step2, Programs FLASH with option for FPGA1 or FPGA2-4\r\n\n"

                 
               "FEB FLASH Programming using USB or SOCKET, '2 Steps'\r\n"
               "   LDFILE 2    Step1, Download Data to FPGA2 SD_RAM (DAQ Inactive)\r\n"
               "   LDPGMFEB n  Step2, Send file(2secs),PGMs if Xfer Good,(20sec),n=1,n=24(All)\r\n"
               "               Use cmd 'LC LDSTAT' or 'LCA LDSTAT' to see FEB Load/Pgm Status\r\n\n\n\n"
                 
               "-------------- FAKE TEST DATA LOADING TO CONTROLLER ------------------------\r\n"
               "Controller Only, USB or SOCKET\r\n"
               "   LDFILE n    Loads Data to 2=SD_RAM2_4xx, 3=SD_RAM3_8xx, 4=SD_RAM4_Cxx\r\n\n\n\n"
                 
               "-------------- FAKE TEST DATA LOADING TO FEB, FLASH PGM OPTION -------------\r\n"
               "Controller to FEB, USB or SOCKET, 4 Steps\r\n"
               "   1) LDFEB n  LOADs FILE\r\n"
               "               Loads Data File SD_RAM2 and Sends data to FEB (@Adr 4xx)\r\n"
               "               If n=1 sends data to default port, if n=24 use all ports\r\n"
               "               Busy time ~3 Seconds per Mbyte\r\n"
               "   2) LC LDFE  ERASE FLASH\r\n"
               "               FEB Erases Fake Data Sectors in Flash, uses Default Port\r\n"
               "               Busy time ~65 Secs, CMD 'LCA LDFE' for all ports\r\n"

               "   3) LC LDFP  PROGRAMS FLASH, UPPER MEMORY SECTORS\r\n"
               "               FEB Programs Fake data Flash Addr 200000(H)\r\n"
               "               Busy time ~10 Secs, CMD 'LCA LDFP' for all ports\r\n"

               "   4) LC LDFC  FEB Copys Fake Data from FLASH to SD_RAM1-4\r\n"
               "               Busy time ~60 Secs, CMD 'LCA LDFC' for all ports\r\n"
                 
               "\nHELP           HE(HELP1),H2(HELP2), HF(FLASH), HN(NETWORK), HT(TEST), HA(ADRMAP)\r\n\n"
};




const   char HelpMenuADRMAP[]={

"       Controller FPGA Address Map as of 06-19-2019\r\n"  
"0x000: Control and status register\r\n"
" Bit0: Internal spill generator Enable. This enables a sequencer that \r\n"
"       produces heartbeat packets with the Mu2e beam timing\r\n"
"       1: Enabled, 0: Disabled\r\n"
" Bit1: Internal spill generator continuous/burst select. When in burst\r\n"
"       mode, the value stored in the spill generator burst determines\r\n"
"       the number of microbunches transmitted each time the generator is\r\n"
"       enabled. 1: burst, 0: continuous\r\n"
" Bit2: Packet former holdoff. A diagnostic that prevents data coming from\r\n"
"       FPGA2,3 and 4 from being packetized and transmitted on the fiber\r\n"
"       transmitter. 1: Transmit Disabled, 0: Transmit enabled\r\n"
" Bit3: Issue a Gigabit Transceiver interface (GTP)reset. 1:Reset, 0:No action\r\n"
" Bit4: Timing Source Select. 1: Timing trans is connected to GPI,\r\n"
"       0: Timing source is the internal spill timer\r\n"
" Bit5: Marker Sync enable. 1: Heartbeat data is transmitted upon receipt\r\n"
"       of marker, 0: Heartbeat data is transmitted immediately upon receipt\r\n"
"       of heartbeat packet\r\n"
" Bit6: Trigger packet destination. 0: Packets are written to a FIFO readable\r\n"
"       from the processor bus. 1: Pac are sent via an FM link to FPGAs 2,3,4\r\n"
" Bit7: Reset packet former state machine, 1: Reset, 0: No action\r\n"
" Bit8: Internal trigger generator enable. 1: Enabled, 0: Disabled\r\n"
" Bit9: Internal trigger generator continuous/burst select. When in burst mode,\r\n"
"       the value stored in the trigger generator burst determines the\r\n"
"       number of triggers xmited each time gen is ena, 1: burst, 0: cont\r\n"
"0x002: GTP Rx FIFO CSR\r\n"
"0x003: Transmitted Event Wordcount,words sent on most recent event (diag)\r\n"
"0x004: Link Receive 0 Wordcount, words used on rec FIFO link from FPGA 2\r\n"
"0x005: Link Receive 0 Wordcount, words used on rec FIFO link from FPGA 3\r\n"
"0x006: Link Receive 0 Wordcount, words used on rec FIFO link from FPGA 4\r\n"
"0x007: Event builder FIFO status full and empty flags of event builder FIFO\r\n"
"0x008: Input port active bits for ports 23..16\r\n"
"0x009: Input port active bits for ports 15..0\r\n"
"       The expectation is that the uC will periodically monitor\r\n"
"       the activity on the ports attached to FPGA 4 and copy them\r\n"
"       to an eight bit register accessible at this location.	  \r\n"
"0x00A: Controller ID register, a writable 4 bit reg whose contents will be \r\n"
"       included in event data in the controller ID field\r\n"
"0x00B: Read the state of the debug header pins\r\n"
"0x00C: GTP elastic buffer status (see Xilinx UG386 p.144 for bit desc)\r\n"
"0x00D: Data Request Packet buffer FIFO that collects data request packets\r\n"
"       from the fiber channel is read at this address. Nine reads deliver\r\n"
"       the 8 payload words and the checksum of the packet\r\n"
"0x00E: Data Request Packet buffer words used\r\n"
"0x00F: Internal trigger burst count 16 bit register that counts down a burst\r\n"
"       of triggers if burst mode on the trigger generator is enabled\r\n"
"\r\n"
"0x010..0x15: Harmonica Jack LED control\r\n"
"       48 bi-color LEDs controlled with the 96 bits in this address range\r\n"
"       For each bit pair 00 or 11 is off, 01 is green, 10 is yellow\r\n"
"0x016: Harmonica Jack LED reset\r\n"
"       A write to this address will clear all the LED bits pairs to 00\r\n"
"0x017: ADF4001 control word upper byte. A write to this address will load\r\n"
"       the upper eight bits of the 24 bit word to be sent to the PLL chip\r\n"
"0x018: ADF4001 control word lower word\r\n"
"       A write to this address will load the lower 16 bits of the 24 bit word\r\n"
"       to be sent to the PLL chip along with the upper bits written to\r\n"
"       location 0x17 into xmit FIFO for serialization and xmit to PLL chip\r\n"
"\r\n"
"0x019: ADF4001 power down, write 1 to bit zero will power down the PLL chip\r\n"
"0x01A: 0x1B: GTP Preamble transmit register\r\n"
"0x01C: 0x1D: GTP payload transmit register\r\n"
"0x01E: 0x1F: GTP checksum transmit register\r\n"
"0x020: 0x21: GTP Receive FIFOs\r\n"
"0x022: State of the packet former state machine\r\n"
"0x024: 0x25, 0x26: Link Receive FIFOs\r\n"
"0x028: 0x29, 0x2A, 0x2B: Check sums\r\n"
"0x02C: Event wordcount buffer\r\n"
"0x02D: Event wordcount buffer status\r\n"
"0x032: Heartbeat burst length register upper bits\r\n"
"0x033: Heartbeat burst length register lower bits\r\n"
"0x034: Read/Write Test Counter upper word\r\n"
"0x035: Read/Write Test Counter lower word\r\n"
"0x036: Real Internal Microbunch counter upper bits\r\n"
"0x037: Real Internal Microbunch counter middle bits\r\n"
"0x038: Real Internal Microbunch counter lower bits\r\n"
"0x039: Trigger control register\r\n"
"0x03A: Read/Write Beam On Trigger Request Generator rate upper word\r\n"
"0x03B: Read/Write Beam On Trigger Request Generator rate lower word\r\n"
"0x06C: Read Uptime counter upper bits\r\n"
"0x06D: Read Uptime counter upper bits\r\n"
"0x300: Write to the CSR registers (0x400,0x800,0xC00)\r\n"
"0x301: Write to 24 Phy Tx data buffers (0x411,0x811,0xC11)\r\n"
"0x301: Write to 24 Phy Tx control register (0x412,0x812,0xC12)\r\n"
"\r\n"
"0x400: FPGA2 CSR\r\n"
" Bit0: Reset the Phy receive buffers, DDR read sequencer\r\n"
" Bit1: Reset LVDS serial link to FPGA 1. 1: Reset, 0: No action\r\n"
" Bit2: Ethernet Phy power down. 1: powered down, 0: normal operation\r\n"
" Bit3: Octal LVDS receiver enable. 1: Enabled, 0: Disabled\r\n"
" Bit4: Reset MIG DDR memory interface macro\r\n"
" Bit5: Enable the DRAM write sequencer. 1: Seq is Ena Xmit 0:Dis\r\n"
" Bit6: Ethernet Phy Data Source: 1 Data source if the FM Rec attached \r\n"
"       to FPGA 1, 0: Data source is the microcontroller bus\r\n"
" Bit7: Request for the DDR read seq to start. 1: Start Req, 0: no action\r\n"
" Bit8: Initialize DDR write sequencer. 1: Initialize, 0: no action\r\n"
"0x402: Set SDRam Write address upper bits\r\n"
"0x403: Set SDRam Write address lower bits\r\n"
"0x404: Set SDRam Read address upper bits\r\n"
"0x405: Set SDRam Read address lower bits\r\n"
"0x406: Byte swapped SDRam read data port\r\n"
"0x407: Un-swapped SDRam data port\r\n"
"0x408: MIG Status Register\r\n"
"0x409: MIG FIFO Count\r\n"
"0x40E: Phy Transmit Mask\r\n"
"0x410: Write to the 100MB link\r\n"
"0x411: Broadcast write to the eight Ethernet Phy chips\r\n"
"0x412: Phy Tx Transmit CSR bits\r\n"
"0x413: Phy Tx Transmit FIFO words used\r\n"
"0x414: Read Phy RxErr status bits\r\n"
"0x415: Read Phy RxCRS status bits\r\n"
"0x416: Read Phy Rx Buffer status bits\r\n"
"0x417: Phy Rx mask bits\r\n"
"0x418..0x41F: Read Phy Rx Data Buffer Words Used\r\n"
"\r\n"
"0x420..0x427: Read Phy Rx Data Buffers\r\n"
"0x42F: Read LVDS Rx Input Status\r\n"
"0x430..0x437: LVDS Rx Read Data Buffers\r\n"
"0x438..0x43F: LVDS Rx words used\r\n"
"0x440: Read LVDS Rx Buffer status bits\r\n"
"0x441: Read LVDS Rx Parity Error bits\r\n"
"0x442: LVDS Clock Fanout Serial data port\r\n"
"0x443: Read Test Counter upper bits\r\n"
"0x444: Read Test Counter lower bits\r\n"
"0x450..0x45F: Read Phy Rx CRCs\r\n"
"0x460: Read Phy Rx CRC Error bits\r\n"
"0x46C: Uptime Counter bits 31...16\r\n"
"0x46D: Uptime Counter bits 15...0\r\n"
"0x4FD..0x4FD: Phy SMI read data registers\r\n"
"0x4FF: Phy SMI Command port\r\n"
"0x500..0x57F: Phy SMI data registers\r\n"
};





const uint8  mu_BOOT_MSG1[]= {"\r\nMU2E CRV READOUT CONTROLLER\r\n"};
const uint8  mu_BOOT_MSG2[]= {"Network Setup InValid, Loading Defaults\r\n"};

// resolution 1.206mV per bit (for 5v Ref at 12Bits)
// resolution 1.000mV per bit (for 4.960v Ref at 12Bits)
const   char *adcName[]= {"1.2v_Pos=", "1.8v_Pos=", "2.5v_Pos=",  "3.3v_Pos=", "Temp_(C)=" };
const   float adcScale[]={  .0010,      .0010,     .0010,       .0010 };
const   int   netVars[]= {4, 4, 4, 6, 1,1,1};   //byte count for "GateWay","NetMask","MacAddr","ipAddr", sock0, sock1,sock2

//fpga data port lookup table
const   int    offPHYDATAPORT[] = {0x00, 0x20, 0x21, 0x22, 0x23,  0x24, 0x25, 0x26, 0x27 };
const   int    oFMData30[]      = {0x00, 0x30, 0x31, 0x32, 0x33,  0x34, 0x35, 0x36, 0x37 };
//const   int    oFMWrdCnt38[]    = {0x00, 0x38, 0x39, 0x3a, 0x3b,  0x3c, 0x3d, 0x3e, 0x3f };
const   int    PHYSTATUSBIT[]   = {0X00, BIT0, BIT1, BIT2, BIT3,  BIT4, BIT5, BIT6, BIT7 };


#endif

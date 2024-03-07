-- Firmware for apex FPGA

-- Sten Hansen Fermilab 10/15/2015

-- FPGA responsible for collecting data from three front end FPGAs
-- Microcontroller interface, GBT transceiver interface to the DAQ

-- 10/15/15 microcontoller interface
-- 12/22/15 serial data receivers for data coming from the PHY FPGAs
-- 03/15/16 serializers for the front panel LEDs, PLL chip
-- 03/15/16 serializers for the front panel LEDs, PLL chip
-- 05/16/16 minimal GTP loop back demonstrated
-- 04/02/18 Setup Beam On/Beam Off Microbunch generator
-- 08/10/18 Added event buffer FIFO flag outputs

----------------------------- Main Body of design -------------------------

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

Library UNISIM;
use UNISIM.vcomponents.all;

use work.Project_defs.all;

use work.git_hash_pkg.all;

entity ControllerFPGA_1 is port(

-- 100 MHz VXO clock, 50MHz Phy clock
	VXO_P,VXO_N,ClkB_P,ClkB_N,Clk50MHz,
	BnchClk_P,BnchClk_N,Marker_P,Marker_N : in std_logic;
-- clock return lines, not used
   -- TDAQRtn0_P, TDAQRtn0_N,TDAQRtn1_P,TDAQRtn1_N : out std_logic;
-- 156.25 MHz GTP Reference clock, Gigabit data lines
	GTPClk_P,GTPClk_N,GTPRx_P,GTPRx_N : in std_logic_vector(1 downto 0);
	GTPTx_P,GTPTx_N : out std_logic_vector(1 downto 0);
-- Optical transcever slow control lines
	TDisA,TDisB : buffer std_logic;
-- The partucular optical transceivers we bought don't come with I2C control
-- thses lines don't go anywhere for now.
	SD_A,SD_B : in std_logic;
-- microcontroller strobes
	CpldRst, CpldCS, uCRd, uCWr, EthCS : in std_logic;
-- microcontroller data, address buses
	uCA : in std_logic_vector(11 downto 0);
	uCD : inout std_logic_vector(15 downto 0);
-- Geographic address pins
	GA : in std_logic_vector(1 downto 0);
-- Serial inter-chip link clock, framing lines
	LINKClk_P,LINKClk_N,LinkFR_P,LinkFR_N  : in std_logic_vector(2 downto 0);
-- Serial inter-chip link Data lines
	LinkSDat_P,LinkSDat_N : in std_logic_vector(5 downto 0);
-- FM Transmitters for uBunch and Triggers
	TrigFM,uBunchLED,TrigLED,
-- Pll control lines
	PllSClk,PllSDat,PllLd,PllPDn : buffer std_logic;
   HeartBeatFM_P,HeartBeatFM_N : out  std_logic;
	PllStat : in std_logic;
-- Serial control lines for the RJ-45 LEDs
	LEDSClk,LEDSDat : out std_logic_vector(2 downto 0);
	LEDLd : out std_logic_vector(5 downto 0);
	LEDRst : buffer std_logic;
-- Orange Tree Ethernet daughter card lines
	DQ : inout std_logic_vector(15 downto 0);
	ZEthA : buffer std_logic_vector(8 downto 0);
	ZEthCS,ZEthWE,ZEthClk : buffer std_logic;
	ZEthBE : buffer std_logic_vector(1 downto 0);
	ZEthEOF : in std_logic_vector(1 downto 0);
	ZEthLen : in std_logic;
-- Back panel LEMOs
	GPO : buffer std_logic_vector(1 downto 0);
	GPI,NimTrig : in std_logic;
-- RJ45
   OGLEDA, OGLEDB, YLED : out std_logic;
--	TDAQRtn0_P, TDAQRtn0_N, TDAQRtn1_P, TDAQRtn1_N : buffer std_logic;
-- Debug port
	Debug : buffer std_logic_vector(10 downto 1)
);

end ControllerFPGA_1;

architecture behavioural of ControllerFPGA_1 is

---------------------- Signal declarations -----------------------

-- Name Arrays according to their size
Type Array_2x2 is Array(0 to 1) of std_logic_vector (1 downto 0);
Type Array_2x3 is Array(0 to 1) of std_logic_vector (2 downto 0);
Type Array_2x10 is Array(0 to 1) of std_logic_vector(9 downto 0);
Type Array_2x13 is Array(0 to 1) of std_logic_vector (12 downto 0);
Type Array_2x16 is Array(0 to 1) of std_logic_vector (15 downto 0);

Type Array_3x3 is Array(0 to 2) of std_logic_vector(2 downto 0);
Type Array_3x4 is Array(0 to 2) of std_logic_vector(3 downto 0);
Type Array_3x5 is Array(0 to 2) of std_logic_vector(4 downto 0);
Type Array_3x8 is Array(0 to 2) of std_logic_vector(7 downto 0);
Type Array_3x13 is Array(0 to 2) of std_logic_vector (12 downto 0);
Type Array_3x14 is Array (0 to 2) of std_logic_vector (13 downto 0);
Type Array_3x16 is Array(0 to 2) of std_logic_vector (15 downto 0);

Type Array_3x2x10 is Array (0 to 2) of Array_2x10;

-- these signals used to be single ended I/O
signal BnchClk,HeartBeatFM,BnchMarker : std_logic;

-- Synchronous edge detectors of uC read and write strobes
Signal RDDL,WRDL : std_logic_vector (1 downto 0);
signal EthWRDL,EthRDDL : std_logic_vector (4 downto 0);

-- Clock and reset signals
signal SysClk,Clk80MHz,FMGenClk,ResetHi,Pll_Locked,nEthClk,
		 EthClk,SerdesRst,LinkBuffRst,GTPRst, Seq_Rst : std_logic;

signal Clk80MHzAlign : std_logic;
signal Clk80MHzAlignCnt : std_logic_vector(7 downto 0);
signal LoopbackMode : std_logic_vector(2 downto 0);

-- Counter that determines the trig out pulse width
signal GPOCount : std_logic_vector(2 downto 0);

-- Signals for decoding duty cycle modulated microbunch marker
signal DDRBits : std_logic_vector(1 downto 0);
signal MarkerBits : std_logic_vector(15 downto 0);
signal MarkerDelay, MarkerDelayCounter : std_logic_vector(7 downto 0);
signal Even_Odd,MarkerDelayed,Marker,MarkerReq,MarkerSyncEn, MarkerDelayArm : std_logic;
signal MarkerLast : std_logic_vector(1 downto 0);
attribute ASYNC_REG : string; 
attribute ASYNC_REG of MarkerLast : signal is "TRUE"; 
signal MarkerCnt, MarkerCnt2, MarkerCnt3, MarkerCnt4, MarkerCnt5 : std_logic_vector(7 downto 0); -- counts decoded markers
signal BnchMarkerLast : std_logic_vector(7 downto 0);
signal MarkerDelayedCnt : std_logic_vector(7 downto 0);
signal HeartBeatCnt : std_logic_vector(7 downto 0); -- counts heartbeats sent out
signal HeartBtCnt : std_logic_vector(7 downto 0); -- counts heart beat packages from fibers
-- event window timers
signal WindowTimer : std_logic_vector(15 downto 0);
signal LastWindow : std_logic_vector(15 downto 0);
signal InjectionTs : std_logic_vector(15 downto 0);
signal NimTrigLast : std_logic; -- latch the input to detect input rising edge

-- foramt settings
signal uBinHeader : std_logic; 
signal uBwrt : std_logic;
signal uBdebug, uBdebug2 : std_logic;

-- Orange tree signals
signal iDQ : std_logic_vector (15 downto 0);
Signal DQWrtDly : Array_3x16;
signal DQEn : std_logic;

-- uC data bus
signal iCD : std_logic_vector(15 downto 0);
signal AddrReg : std_logic_vector(11 downto 0);
-- FM transmit enable
signal EnTx1,TrgSrc : std_logic;

-- Timing interval counters
signal Counter1us : std_logic_vector (7 downto 0);
signal Counter100us : std_logic_vector (13 downto 0);
signal Counter1ms : std_logic_vector (17 downto 0); 
signal Counter1s : std_logic_vector (27 downto 0);
signal GateCounter, TurnOnTime, TurnOffTime : std_logic_vector (8 downto 0);

signal TrigEn,TstTrigEn,TstTrigCE,Spill_Req,Beam_On : std_logic; 

signal TstPlsEn,TstPlsEnReq,SS_FR,IntTrig,IntTmgEn,TmgCntEn : std_logic;
signal SpillWidth,InterSpill,InterSpillCount : std_logic_vector (7 downto 0);

-- Signals for generating fake accelerator timing signals
signal SpillWidthCount : std_logic_vector (8 downto 0);
signal SuperCycleCount : std_logic_vector (13 downto 0);
signal IntuBunchCount,ExtuBunchCount : std_logic_vector (47 downto 0);
signal ExtuBunchOffset : std_logic_vector (15 downto 0);
-- Counter for counting down heartbeat bursts
signal HrtBtBrstCntReg,HrtBtBrstCounter  : std_logic_vector (23 downto 0);
signal uBunchLEDCnt : std_logic_vector (4 downto 0);
signal TrigType : std_logic_vector (3 downto 0);
signal DRFreq : std_logic_vector (31 downto 0);
signal DRCount : std_logic_vector (7 downto 0);
signal Int_uBunch : std_logic_vector (1 downto 0);

-- Count the number of triggers
signal DReq_Count : std_logic_vector (31 downto 0);
-- Make a test counter that increments with each read
signal TestCount : std_logic_vector (31 downto 0);
-- Uptime counter to check for un-anticipated resets
signal UpTimeCount,UpTimeStage : std_logic_vector (31 downto 0);
-- Number of data words per spill

-- Spill counter, event word cout, spill word count
signal SpillCount,EventWdCnt : std_logic_vector (15 downto 0);

signal GPIDL,TrigDL,iWrtDL  : Array_2x2;
signal GateWidth0,GateWidth1,PedWidth0,PedWidth1 : std_logic_vector (7 downto 0);
-- Test Pulse generator signals
-- DDS frequency registers
signal FreqReg,PhaseAcc : std_logic_vector (31 downto 0);
signal PhaseAccD : std_logic;

-- Link receive FIFO signals
signal LinkFIFOEn,LinkFIFOEnd,LinkFIFORdReq,LinkFIFOWrReq,
		 LinkFIFOEmpty,LinkFIFOFull : std_logic_vector (2 downto 0);
signal LinkFIFORdCnt : Array_3x13;
signal LinkRDDL : std_logic_vector (1 downto 0);
signal LinkFIFOOut : Array_3x16;

-- Event buffer signals
signal EventBuff_WrtEn,EventBuff_RdEn,
		 EventBuff_Full,EventBuff_Empty,EvBuffWrtGate : std_logic;
signal EventBuff_Dat,EventBuff_Out,EventSum, Event0, Event1, Event2 : std_logic_vector (15 downto 0);
signal FIFOCount : Array_3x16;

Type Event_Builder_Seq is (Idle,RdInWdCnt0,RdInWdCnt1,RdInWdCnt2,SumWdCnt,WrtWdCnt,WrtWdCnt0,WrtWdCnt1,WrtWdCnt2,RdStat0,
								   RdStat1,RdStat2,WrtStat,WrtUbLow,WrtUbHigh,WaitEvent,ReadFIFO,ReadFIFO0,ReadFIFO1,ReadFIFO2,
									RdUb0low,RdUb0high,RdUb1low,RdUb1high,RdUb2low,RdUb2high, RduB,
									VerifyUb0low,VerifyUb0high,VerifyUb1low,VerifyUb1high,VerifyUb2low,VerifyUb2high);
signal Event_Builder : Event_Builder_Seq;

-- Front panel LED Shifter signals
signal CMDwr_en,CMDrd_en,CMD_Full,CMD_Empty : std_logic;
signal ClkDiv : std_logic_vector (2 downto 0);
signal CMDBitCount : std_logic_vector (3 downto 0);
signal LEDShiftReg : std_logic_vector (15 downto 0);
signal CMD_Out : std_logic_vector (18 downto 0);
Type LEDSerializer_FSM is (Idle,Load,Shift,RdFIFO,SendRst,WaitRst,WaitPClk,SendPClk);
Signal LED_Shift : LEDSerializer_FSM;

-- Pll Chip Shifter signals
signal PLLBuffwr_en,PLLBuffrd_en,PLLBuff_full,PLLBuff_empty : std_logic;
signal PllStage : std_logic_vector (7 downto 0);
signal PLLBuff_Out,PllShiftReg : std_logic_vector (23 downto 0);
signal PllBitCount : std_logic_vector (4 downto 0);
Type PllSerializer_FSM is (Idle,Load,Shift,WaitLd,SendLd);
Signal Pll_Shift : PllSerializer_FSM;

-- Each channel produces two deserialized bit streams with ten bits total
Signal LinkPDat : Array_3x2x10;

signal WrtWdCount0,WrtWdCount1 : std_logic_vector(11 downto 0);
signal Buff_Wrt,Buff_Rd,Buff_Empty : std_logic_vector(2 downto 0);

signal SerDesInP,SerDesInN : Array_3x3;

-- Deserialize frame along with the 8 data lines. Use the deserialized 
-- frame signal as an input to the bitslip state machine
signal LinkFRDat : Array_3x5;
signal SlipReq : std_logic_vector(2 downto 0);
signal Slippause : Array_3x4;

-- Signal names used by SERDES see: XAP1024
signal rxioclkp : std_logic_vector(2 downto 0);
signal rxioclkn : std_logic_vector(2 downto 0);
signal rx_serdesstrobe	: std_logic_vector(2 downto 0);

signal RxOutClk : std_logic_vector (2 downto 0);

-- Signals used by GTP transceivers
Signal tile0_gtp0_refclk_i,tile0_gtp1_refclk_i,GTPRxRst : std_logic;
Signal PllLkDtct,GTPRstDn,RxUsrClk,BuffOut_DCMIn,
		 UsrClk2,UsrClk,TxDCMLock,Reframe : std_logic_vector (1 downto 0);
signal RxLOS,GTPTxClk,Rx_IsComma,InvalidChar,GTPDisp,GTPSysClk,
		 UsrWRDL,UsrRDDL,Rx_IsCtrl : Array_2x2;
signal TxCharIsK,TxCharErr,CommaDL,CommaDL1 : Array_2x2;
signal GTPTxStage,GTPTx,GTPRx,GTPRxReg,GTPRxBuff_Out : Array_2x16;

-- Signals used by GTP Tx and Rx FIFOs
signal DCM_Locked : std_logic_vector (1 downto 0);

signal GTPRxBuff_wr_en,GTPRxBuff_rd_en,GTPRxBuff_full,
		 GTPRxBuff_Emtpy,PRBSCntRst,PRBSErr : std_logic_vector (1 downto 0);
signal WdCountBuff_WrtEn,WdCountBuff_RdEn,WdCountBuff_Full,WdCountBuff_Empty : std_logic;
signal WdCountBuff_DatCnt : std_logic_vector (12 downto 0);
signal WdCountBuff_Out : std_logic_vector (15 downto 0);
signal EnPRBSTst,En_PRBS : Array_2x3;
signal GTPRxBuff_DatCnt : Array_2x13;
-- Signals used by the microbunch,trigger FM transmitters
signal HrtBtTxOuts,DreqTxOuts : TxOutRec;
signal HrtBtDone,HrtBtTxReq,HrtBtTxAck,HrtBtFMTxEn,TxEnReq,DReqTxEn,LinkBusy : std_logic;
signal HrtBtData : std_logic_vector (23 downto 0);
signal TrigFMDat : std_logic_vector (15 downto 0);

-- Trigger request packet buffer, FIFO status bits
signal DReqBuff_Out : std_logic_vector (15 downto 0);
signal DReqBuff_wr_en,DReqBuff_rd_en,DReqBuff_uCRd, DCSPktBuff_uCRd,
		 DReqBuff_Full,TrigTx_Sel,DReqBuff_Emtpy,
		 Dreq_Tx_Req,Dreq_Tx_ReqD,DReq_Tx_Ack,BmOnTrigReq,Stat_DReq : std_logic;
signal LinkFIFOStatReg,Lower_FM_Bits : std_logic_vector (2 downto 0);  

Type Trig_Tx_State is (Idle,SendTrigHdr,SendPad0,SendPktType,SenduBunch0,SenduBunch1,
								SenduBunch2,SendPad1,SendPad2,SendPad3,WaitCRC,SendCRC,SetPktType);
signal IntTrigSeq : Trig_Tx_State;
signal DReqBrstCntReg,DReqBrstCounter : std_logic_vector (15 downto 0);
signal DReqPrescale,PreScaleReg : std_logic_vector (8 downto 0);
signal Packet_Type : std_logic_vector (3 downto 0);

-- Heart beat FIFO
signal HrtBtWrtCnt,HrtBtRdCnt,TrigReqWdCnt,HrtBtWdCnt,DCSReqWdCnt : std_logic_vector (3 downto 0);
signal HrtBtBuffRdCnt,TrgPktRdCnt : std_logic_vector (10 downto 0);
signal HrtBtBuff_wr_en,HrtBtBuff_rd_en,HrtBtBuff_Full,HrtBtBuff_Emtpy,HrtBtFMReq : std_logic;
signal HrtBtBuff_Out : std_logic_vector (15 downto 0);
signal HrtBtMode : std_logic_vector (7 downto 0);
-- Time stamp FIFO
signal TStmpBuff_Out : std_logic_vector (15 downto 0);
signal TStmpBuff_wr_en,TStmpBuff_rd_en,TStmpBuff_Full,TStmpBuff_Emtpy : std_logic;
signal TStmpWds : std_logic_vector (8 downto 0);

-- DCS request FIFO
signal DCSPktBuff_wr_en,DCSPktBuff_rd_en,DCSPktBuff_Full,DCSPktBuff_Emtpy : std_logic;
signal DCSPktRdCnt : std_logic_vector (12 downto 0);
signal DCSPktBuff_Out : std_logic_vector (15 downto 0);
--signal DCSTxBuffWds : std_logic_vector (8 downto 0);


-- DCS answer FIFO
signal DCSBuff_In    : std_logic_vector (15 downto 0);
signal DCSBuff_wr_en : std_logic;
signal DCSBuff_rd_en : std_logic;
signal DCSBuff_Out   : std_logic_vector (15 downto 0);
signal DCSBuff_Full  : std_logic;
signal DCSBuff_Emtpy : std_logic;
signal DCSBuffRdCnt  : std_logic_vector (12 downto 0);
signal DCS_Header : std_logic_vector (15 downto 0);
signal DCS_Status : std_logic_vector (15 downto 0);
signal DCS_EvCnt  : std_logic_vector (15 downto 0);


-- FEB active register
signal ActiveReg : std_logic_vector (23 downto 0);
signal FPGA234_Active : Array_3x8;
signal ActiveCE : std_logic_vector(2 downto 0);
-- Controller ID register
signal IDReg : std_logic_vector (3 downto 0); 
-- FEB ID DPRam signals 
signal FEBID_addra,FEBID_addrb : std_logic_vector (4 downto 0); 
signal FEBID_doutb : std_logic_vector (15 downto 0); 
signal FEBID_wea : std_logic_vector (0 downto 0); 
-- Register that is the "OR" of all the status words from the FEBs
signal StatOr, Stat0 : std_logic_vector (7 downto 0); 
-- check uB consistency
signal uBcheck    : std_logic_vector (31 downto 0);
signal uBcheckRef : std_logic_vector (31 downto 0);
signal uBcheckFlag : std_logic;
-- CRC generator signals
Signal RdCRCEn,TxCRCEn,RxCRCRst,RxCRCRstD,TxCRCRst : std_logic_vector (1 downto 0);
signal TxCRC,RxCRC,TxCRCDat : Array_2x16;
-- Sequencers to read and send packet data
Type Packet_Parser_Seq is (Idle,Read_Type,Check_Seq_No,Wrt_uC_Queue,
									Wrt_FPGA_Queue,Check_CRC);
signal Packet_Parser : Packet_Parser_Seq;

Type Packet_Former_Seq is (Idle,WrtPktCnt,WrtHdrPkt,WrtCtrlHdrPkt,WrtDatPkt,WrtDCSPkt,Loopback,Loopback2);
signal Packet_Former : Packet_Former_Seq;
signal ChkCntr,FormStatReg,EmptyLatch : std_logic_vector (2 downto 0);
signal Pkt_Timer : std_logic_vector (3 downto 0);
signal TxPkCnt : std_logic_vector (10 downto 0);
signal EvTxWdCnt : std_logic_vector (13 downto 0);
signal FMTxBsy,FormHold,HrtBtTxInh,FormRst,ExtTmg,EvTxWdCntTC : std_logic;

signal TxSeqNo,RxSeqNo,WrtCount,GtpRxBuffStat,GtpRxBuffCnt : Array_2x3;
signal RxSeqNoErr : std_logic_vector (1 downto 0);

signal PunchBits : std_logic_vector (3 downto 0);

-- Link counters
signal LosCounter : std_logic_vector (3 downto 0);
signal CRCErrCnt  : std_logic_vector (7 downto 0);

-- Tx trace buffer
signal GTPTxBuff_In, GTPTxBuff_Out  : std_logic_vector(15 downto 0);
signal GTPTxBuff_wr_en, GTPTxBuff_rd_en : std_logic;
signal GTPTxBuff_DatCnt : std_logic_vector(12 downto 0);
-- Data request trace buffer
signal DReqBuffTrace_rd_en : std_logic;
signal DReqBuffTrace_Out : std_logic_vector (15 downto 0);
signal DReqBuffTrace_DatCnt : std_logic_vector (10 downto 0);
-- Input Link FIFO trace
signal LinkFIFOTraceRdReq : std_logic;
signal LinkFIFOTraceOut : std_logic_vector (15 downto 0);
signal LinkFIFOTraceRdCnt : std_logic_vector (12 downto 0);

signal GTPRstCnter : std_logic_vector(6 downto 0); -- used to be (11 downto 0);
signal GTPRstFromCnt : std_logic;
signal GTPTstFromCntEn : std_logic;
signal GTPRstArm : std_logic;

begin

Sys_Pll : SysPll2
  port map(
 -- Clock in ports
    CLK_IN1_P => ClkB_P,
    CLK_IN1_N => ClkB_N,
-- Clock out ports
    CLK_OUT1 => SysClk,   -- 100 MHz
    CLK_OUT2 => EthClk,   -- 160 MHz used for Orange Tree I/O
	 CLK_OUT3 => nEthClk,  -- 160 MHz 180 deg. phase fro DDR In
	 CLK_OUT4 => open, --Clk80MHz, -- 80 MHz for 20mbit FM transmitter
-- Status and control signals
    RESET  => ResetHi,
    LOCKED => Pll_Locked);

HrtBtFMOut : OBUFDS
generic map (
IOSTANDARD => "BLVDS_25")
port map (
O => HeartBeatFM_P, -- Diff_p output (connect directly to top-level port)
OB => HeartBeatFM_N, -- Diff_n output (connect directly to top-level port)
I => HeartBeatFM -- Buffer input
);

BunchClkDiffIn : IBUFDS
   generic map (
      DIFF_TERM => TRUE,
      IBUF_LOW_PWR => FALSE,
      IOSTANDARD => "LVDS_25")
   port map (
      O  => BnchClk,
      I  => BnchClk_P,
      IB => BnchClk_N
   );
BunchMarkerDiffIn : IBUFDS
   generic map (
      DIFF_TERM => TRUE,
      IBUF_LOW_PWR => FALSE,
      IOSTANDARD => "DEFAULT")
   port map (
      O  => BnchMarker,
      I  => Marker_P,
      IB => Marker_N
   );
	
Clk80MHzGenSync : Clk80MHzGen
  port map(
          clk160 => EthClk,
          rst => CpldRst,
          syncEnable => Clk80MHzAlign,
          MarkerBits => MarkerBits,
          clk80 => Clk80MHz,
			 shiftCnt => Clk80MHzAlignCnt);

BunchClkIn : IDDR2
   generic map(
      DDR_ALIGNMENT => "C0", -- Sets output alignment to "NONE", "C0", "C1" 
      INIT_Q0 => '0', -- Sets initial state of the Q0 output to '0' or '1'
      INIT_Q1 => '0', -- Sets initial state of the Q1 output to '0' or '1'
      SRTYPE => "SYNC") -- Specifies "SYNC" or "ASYNC" set/reset
   port map (
      Q0 => DDRBits(0), -- 1-bit output captured with C0 clock
      Q1 => DDRBits(1), -- 1-bit output captured with C1 clock
      C0 => EthClk,  -- 1-bit clock input
      C1 => nEthClk, -- 1-bit clock input
      CE => '1',  -- 1-bit clock enable input
      D => BnchClk,   -- 1-bit data input 
      R => ResetHi,    -- 1-bit reset input
      S => '0'     -- 1-bit set input
   );

TrigLED <= '0';

-- GTP System reset
GTPRst <= '1' when CpldRst = '0' 
  	            or (CpldCS = '0' and uCWR = '0' and uCA = CSRRegAddr and uCD(3) = '1') 
					or (GTPRstFromCnt = '1' and GTPTstFromCntEn = '1')
					else '0';

-- Reset of receive FIFOs, event counters
GTPRxRst <= '1' when CpldRst = '0' 
  	               or (CpldCS = '0' and uCWR = '0' and uCA = GTPFIFOAddr and uCD(0) = '1') else '0';

HrtBtData(19 downto 0) <= ExtuBunchCount(19 downto 0);
HrtBtData(20) <= Beam_On;
HrtBtData(21) <= '0' when ExtuBunchCount(31 downto 20) = 0 else '1';
HrtBtData(23 downto 22) <= "00";
-- FM transmitter for boadcasting microbunch numbers to the FEBs
HeartBeatTx : FM_Tx 
	generic map (Pwidth => 24)
		 port map(clock => Clk80MHz, 
					 reset => ResetHi,
					 Enable => HrtBtFMTxEn,
					 Data => HrtBtData, 
					 Tx_Out => HrtBtTxOuts);
HeartBeatFM <= HrtBtTxOuts.FM when ExtTmg = '0' else GPI;
Debug(1) <= HrtBtTxOuts.FM;
Debug(2) <= HrtBtFMTxEn;
Debug(3) <= MarkerDelayed;
Debug(4) <= HrtBtDone;
Debug(5) <= HrtBtFMReq;

OGLEDA <= HrtBtFMTxEn; -- TODO, indicate PLL lock 
OGLEDB <= Marker;
YLED   <= '0';

-- FM transmitter for data requests, used only when sending fake controller data to the GTP link
DReqTx : FM_Tx
	generic map (Pwidth => 16)
		 port map(clock => SysClk, 
					 reset => ResetHi,
					 Enable => DReqTxEn,
					 Data => DReqBuff_Out,
					 Tx_Out => DreqTxOuts);
TrigFM <= DreqTxOuts.FM when TrigTx_Sel = '1'
			 else LinkBusy when TrigTx_Sel = '0';

DReqTxEn <= '1' when TrigTx_Sel = '1' and DReqBuff_Emtpy = '0' and DreqTxOuts.Done = '0' 
					  else '0';
				  
-- FIFO for buffering broadcast trigger requests, 
-- crossing clock domains from UsrClk to Sysclk
DReqBuff : FIFO_DC_1kx16
  PORT MAP (rst => GTPRxRst,
    wr_clk => UsrClk2(0),
	 rd_clk => SysClk,
    din => GTPRxReg(0),
    wr_en => DReqBuff_wr_en,
    rd_en => DReqBuff_rd_en,
    dout => DReqBuff_Out,
    full => DReqBuff_Full,
    empty => DReqBuff_Emtpy,
	 rd_data_count => TrgPktRdCnt);

	 DReqBuff_rd_en <= DreqTxOuts.Done when TrigTx_Sel = '1' else DReqBuff_uCRd;

-- FIFO for buffering incoming heartbeats
-- crossing clock domains from UsrClk to Sysclk
HrtBtBuff : FIFO_DC_1kx16
  PORT MAP (rst => GTPRxRst,
    wr_clk => UsrClk2(0),
	 rd_clk => SysClk,
    din => GTPRxReg(0),
    wr_en => HrtBtBuff_wr_en,
    rd_en => HrtBtBuff_rd_en,
    dout => HrtBtBuff_Out,
    full => HrtBtBuff_Full,
    empty => HrtBtBuff_Emtpy,
	 rd_data_count => HrtBtBuffRdCnt);

-- FIFO for buffering status requests
DCSPktBuff : LinkFIFO
  PORT MAP (rst => GTPRxRst,
	 wr_clk => UsrClk2(0),
    rd_clk => SysClk,
    din => GTPRxReg(0),
    wr_en => DCSPktBuff_wr_en,
    rd_en => DCSPktBuff_rd_en,
    dout => DCSPktBuff_Out,
    full => DCSPktBuff_Full,
    empty => DCSPktBuff_Emtpy,
	 rd_data_count => DCSPktRdCnt);
	 
	 DCSPktBuff_rd_en <= DCSPktBuff_uCRd;

-- FIFO for DCS answers
DCSOutBuff : LinkFIFO
  PORT MAP (rst => GTPRxRst,
	 wr_clk => SysClk,
    rd_clk => UsrClk2(0),
    din => DCSBuff_In,
    wr_en => DCSBuff_wr_en,
    rd_en => DCSBuff_rd_en,
    dout => DCSBuff_Out,
    full => DCSBuff_Full,
    empty => DCSBuff_Emtpy,
	 rd_data_count => DCSBuffRdCnt);
	 
	 DCSPktBuff_rd_en <= DCSPktBuff_uCRd;


-- Queue up time stamps for later checking
TimeStampBuff : TrigPktBuff
  PORT MAP (rst => GTPRxRst,
    clk => UsrClk2(0),
    din => GTPRxReg(0),
    wr_en => TStmpBuff_wr_en,
    rd_en => TStmpBuff_rd_en,
    dout => TStmpBuff_Out,
    full => TStmpBuff_Full,
    empty => TStmpBuff_Emtpy,
	 data_count => TStmpWds);

-- DP Ram for storing FEB addresses
FEBIDList : FEBIDListRam
  PORT MAP (clka => SysClk,
    wea => FEBID_wea,
    addra => FEBID_addra,
    dina => uCD,
    clkb => SysClk,
    rstb => ResetHi,
    addrb => FEBID_addrb,
    doutb => FEBID_doutb);

EventBuff: FIFO_SC_4Kx16
  port map (clk => UsrClk2(0),
		rst => ResetHi,
		wr_en => EventBuff_WrtEn,
		rd_en => EventBuff_RdEn,
      din => EventBuff_Dat,
      dout => EventBuff_Out,
      full => EventBuff_Full,
	   empty => EventBuff_Empty);

WdCountBuff: GTPRxFIFO
  port map (clk => UsrClk2(0),
		rst => GTPRxRst,
		wr_en => WdCountBuff_WrtEn,
		rd_en => WdCountBuff_RdEn,
      din => EventBuff_Out,
      dout => WdCountBuff_Out,
      full => WdCountBuff_Full,
	   empty => WdCountBuff_Empty,
		data_count => WdCountBuff_DatCnt);

-- Generate two sets of logic for the two GTP sections
GenGTP_Pairs : for i in 0 to 1 generate

-- CRC generators for transmit data
TxCRCGen : crc 
 port map(data_in => TxCRCDat(i),
    crc_en => TxCRCEn(i), rst => TxCRCRst(i), clk => UsrClk2(i),
    crc_out => TxCRC(i));

-- CRC generators for receive data CRC checking
RxCRCGen : crc 
 port map( data_in => GTPRxReg(i),
    crc_en => RdCRCEn(i), rst => RxCRCRstD(i), clk => UsrClk2(i),
    crc_out => RxCRC(i));

-- GTP Receive data FIFOs
GTPRxBuffs : GTPRxFIFO
  PORT MAP (rst => GTPRxRst,
    clk => UsrClk2(i),
    din => GTPRxReg(i),
    wr_en => GTPRxBuff_wr_en(i),
    rd_en => GTPRxBuff_rd_en(i),
    dout => GTPRxBuff_Out(i),
    full => GTPRxBuff_Full(i),
    empty => GTPRxBuff_Emtpy(i),
	 data_count => GTPRxBuff_DatCnt(i));

-- To connect the GTPClkOut to the TxUSRClk and RxUSRClk you need a BUFIO2 
-- and a DCM, as an alternative to the IBUFF2 called for in the doc, since 
-- the IBUFF2 divide by 2 setting doesn't work. 
-- The DCM generated by the wizard has an input IBUFG as a default 
-- which must be deselected. In its place goes a separately instantiated BUFIO2
-- with the divide function bypassed

GTPClkBuffs : BUFIO2
generic map (DIVIDE => 1, DIVIDE_BYPASS => TRUE)
port map (DIVCLK => BuffOut_DCMIn(i), 
			 IOCLK => open, SERDESSTROBE => open, 
			 I => GTPSysClk(i)(0)); 

GTPTxDCMs : GTPClkDCM
  port map
   (-- Clock in ports
    CLK_IN1 => BuffOut_DCMIn(i),
    -- Clock out ports
    CLK_OUT1 => UsrClk(i),
    CLK_OUT2 => UsrClk2(i),
    -- Status and control signals
    RESET  => ResetHi,
    LOCKED => DCM_Locked(i));

end generate;

-- output trace buffer
-- remove or reduce if more ram is needed
-- could also use a GTPRxFIFO
GTPTxBuff : GTPRxFIFO
  PORT MAP (rst => GTPRxRst,
    clk => UsrClk2(0),
    din => GTPTxBuff_In,
    wr_en => GTPTxBuff_wr_en,           -- whenever GTPTx(0) != X"BC3C"
    rd_en => GTPTxBuff_rd_en,           -- uC or when almost full, see data_count
    dout => GTPTxBuff_Out,              -- to uC
    full => open,                       -- not used
    empty => open,                      -- not used 
	 data_count => GTPTxBuff_DatCnt);    -- needed for tace buffer behavior
	 
-- REMOVE ME!	 
-- trace buffer of data requests for debug 	 
DReqBuffTrace : FIFO_DC_1kx16
  PORT MAP (rst => GTPRxRst,
    wr_clk => UsrClk2(0),
	 rd_clk => SysClk,
    din => GTPRxReg(0),
    wr_en => DReqBuff_wr_en,
    rd_en => DReqBuffTrace_rd_en,
    dout => DReqBuffTrace_Out,
    full => open,
    empty => open,
	 rd_data_count => DReqBuffTrace_DatCnt);
	 
	 
-- trace buffer for link 0
LinkBuffTrace : LinkFIFO
  port map (rst => LinkBuffRst, wr_clk => RxOutClk(0), rd_clk => SysClk, 
    wr_en => LinkFIFOWrReq(0),rd_en => LinkFIFOTraceRdReq,
    din(15 downto 13) => LinkPDat(0)(1)(7 downto 5),
    din(12 downto 8) => LinkPDat(0)(0)(9 downto 5),
    din( 7 downto 5) => LinkPDat(0)(1)(2 downto 0),
    din( 4 downto 0) => LinkPDat(0)(0)(4 downto 0),
    dout => LinkFIFOTraceOut, empty => open,
	 full => open,
	 rd_data_count => LinkFIFOTraceRdCnt);


----------------------------- The GTP Wrapper -----------------------------
---------------------- Dedicated GTP Reference Clock Inputs ---------------

-- Each dedicated refclk will need its own IBUFDS instance

    tile0_refclk_ibufds_0 : IBUFDS
    port map
    (O => tile0_gtp0_refclk_i,
     I =>  GTPClk_P(0),   -- Connect to package pin A10
     IB => GTPClk_N(0));  -- Connect to package pin B10

    tile0_refclk_ibufds_1 : IBUFDS
    port map
    ( O => tile0_gtp1_refclk_i,
      I =>  GTPClk_P(1),  -- Connect to package pin C11
      IB => GTPClk_N(1)); -- Connect to package pin D11

    GTP_Xcvr_i : GTP_Xcvr
    generic map
    (
        WRAPPER_SIM_GTPRESET_SPEEDUP    =>      0,   -- Set this to 1 for simulation
        WRAPPER_SIMULATION              =>      0    -- Set this to 1 for simulation
    )
    port map
    (   --_____________________________________________________________________
        --_____________________________________________________________________
        --TILE0  (X0_Y0)
--------------------------------- PLL Ports --------------------------------
        TILE0_CLK00_IN                  =>      tile0_gtp0_refclk_i,
        TILE0_CLK01_IN                  =>      tile0_gtp1_refclk_i,
        TILE0_GTPRESET0_IN              =>      GTPRst,
        TILE0_GTPRESET1_IN              =>      GTPRst,
        TILE0_PLLLKDET0_OUT             =>      PllLkDtct(0),
        TILE0_PLLLKDET1_OUT             =>      PllLkDtct(1),
        TILE0_RESETDONE0_OUT            =>      GTPRstDn(0),
        TILE0_RESETDONE1_OUT            =>      GTPRstDn(1),
       ----------------------- Receive Ports - 8b10b Decoder ----------------------
        TILE0_RXCHARISCOMMA0_OUT        =>      Rx_IsComma(0),
        TILE0_RXCHARISCOMMA1_OUT        =>      Rx_IsComma(1),
        TILE0_RXCHARISK0_OUT            =>      Rx_IsCtrl(0),
        TILE0_RXCHARISK1_OUT            =>      Rx_IsCtrl(1),
        TILE0_RXDISPERR0_OUT            =>      GTPDisp(0),
        TILE0_RXDISPERR1_OUT            =>      GTPDisp(1),
        TILE0_RXNOTINTABLE0_OUT         =>      InvalidChar(0),
        TILE0_RXNOTINTABLE1_OUT         =>      InvalidChar(1),
       ---------------------- Receive Ports - Clock Correction --------------------
		  TILE0_RXCLKCORCNT0_OUT          =>      GtpRxBuffCnt(0),
		  TILE0_RXCLKCORCNT1_OUT          =>      GtpRxBuffCnt(1),
	    --------------- Receive Ports - Comma Detection and Alignment --------------
        TILE0_RXENMCOMMAALIGN0_IN       =>      Reframe(0), -- '0',
        TILE0_RXENMCOMMAALIGN1_IN       =>      Reframe(1), -- '0', 
        TILE0_RXENPCOMMAALIGN0_IN       =>      Reframe(0),
        TILE0_RXENPCOMMAALIGN1_IN       =>      Reframe(1),
        ----------------------- Receive Ports - PRBS Detection ---------------------
        TILE0_PRBSCNTRESET0_IN          =>      PRBSCntRst(0), 
        TILE0_PRBSCNTRESET1_IN          =>      PRBSCntRst(1), 
        TILE0_RXENPRBSTST0_IN           =>      EnPRBSTst(0), 
        TILE0_RXENPRBSTST1_IN           =>      EnPRBSTst(1),
        TILE0_RXPRBSERR0_OUT            =>      PRBSErr(0),
        TILE0_RXPRBSERR1_OUT            =>      PRBSErr(1),
       ------------------- Receive Ports - RX Data Path interface -----------------
        TILE0_RXDATA0_OUT               =>      GTPRx(0),
        TILE0_RXDATA1_OUT               =>      GTPRx(1),
        TILE0_RXRESET0_IN               =>      GTPRst,
        TILE0_RXRESET1_IN               =>      GTPRst,
        TILE0_RXUSRCLK0_IN              =>      UsrClk(0), 
        TILE0_RXUSRCLK1_IN              =>      UsrClk(1), 
        TILE0_RXUSRCLK20_IN             =>      UsrClk2(0), 
        TILE0_RXUSRCLK21_IN             =>      UsrClk2(1), 
      ------- Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR ------
        TILE0_RXN0_IN                   =>      GTPRx_N(0),
        TILE0_RXN1_IN                   =>      GTPRx_N(1),
        TILE0_RXP0_IN                   =>      GTPRx_P(0),
        TILE0_RXP1_IN                   =>      GTPRx_P(1),
      ----------- Receive Ports - RX Elastic Buffer and Phase Alignment ----------
        TILE0_RXBUFSTATUS0_OUT          =>      GtpRxBuffStat(0),
        TILE0_RXBUFSTATUS1_OUT          =>      GtpRxBuffStat(1),
      --------------- Receive Ports - RX Loss-of-sync State Machine --------------
        TILE0_RXLOSSOFSYNC0_OUT         =>      RxLOS(0),
        TILE0_RXLOSSOFSYNC1_OUT         =>      RxLOS(1),
       -------------------- Receive Ports - RX Polarity Control -------------------
		  TILE0_RXPOLARITY0_IN            =>      '1',
		  TILE0_RXPOLARITY1_IN            =>      '0',
       ---------------------------- TX/RX Datapath Ports --------------------------
        TILE0_GTPCLKOUT0_OUT            =>      GTPSysClk(0),
        TILE0_GTPCLKOUT1_OUT            =>      GTPSysClk(1),
       ------------------- Transmit Ports - 8b10b Encoder Control -----------------
        TILE0_TXCHARISK0_IN             =>      TxCharIsK(0),
        TILE0_TXCHARISK1_IN             =>      TxCharIsK(1),
        TILE0_TXKERR0_OUT               =>      TxCharErr(0),
        TILE0_TXKERR1_OUT               =>      TxCharErr(1),
       ------------------ Transmit Ports - TX Data Path interface -----------------
        TILE0_TXDATA0_IN                =>      GTPTx(0),
        TILE0_TXDATA1_IN                =>      GTPTx(1),
        TILE0_TXUSRCLK0_IN              =>      UsrClk(0),
        TILE0_TXUSRCLK1_IN              =>      UsrClk(1),
        TILE0_TXUSRCLK20_IN             =>      UsrClk2(0),
        TILE0_TXUSRCLK21_IN             =>      UsrClk2(1),
       --------------- Transmit Ports - TX Driver and OOB signalling --------------
        TILE0_TXN0_OUT                  =>      GTPTx_N(0),
        TILE0_TXN1_OUT                  =>      GTPTx_N(1),
        TILE0_TXP0_OUT                  =>      GTPTx_P(0),
        TILE0_TXP1_OUT                  =>      GTPTx_P(1),
        TILE0_TXENPRBSTST0_IN           =>      En_PRBS(0),
        TILE0_TXENPRBSTST1_IN           =>      En_PRBS(1)
);

-- GTP logic processes

-- GTP event handling process
TrigReqTx : process (UsrClk2(0), CpldRst)

begin

 if CpldRst = '0' then 

	CommaDL(0) <= "00"; GTPRxReg(0) <= X"0000";
	UsrWRDL(0) <= "00"; UsrRDDL(0) <= "00";
	Reframe(0) <= '1'; GTPTx(0) <= X"BC3C";
	TxCharIsK(0) <= "11"; GTPTxStage(0) <= X"BC3C";
   GTPTxBuff_In <= X"BC3C";
	TxSeqNo(0) <= "000"; TxCRCRst(0) <= '0'; HrtBtMode <= X"00";
   TxCRCEn(0) <= '0'; RdCRCEn(0) <= '0'; HrtBtBuff_wr_en <= '0';
 	RxCRCRst(0) <= '0';  RxCRCRstD(0) <= '0'; HrtBtWrtCnt <= X"0";
	TrigReqWdCnt <= X"0"; PRBSCntRst(0) <= '0'; DReqBuff_wr_en <= '0'; 
	DCSReqWdCnt  <= X"0"; DCSPktBuff_wr_en <= '0'; 
	DReq_Count <= (others =>'0');
	LinkRDDL <= "00"; Packet_Parser <= Idle; Event_Builder <= Idle;
	--RxSeqNoErr(0) <= '0'; 
	Packet_Former <= Idle; FormRst <= '0';
	LinkFIFORdReq <= (others =>'0'); StatOr <= X"00";  Stat0 <= X"00";
	uBcheck <= (others =>'0'); uBcheckRef  <= (others =>'0'); uBcheckFlag <= '0';
	EvTxWdCnt <= (others => '0'); EvTxWdCntTC <= '0'; EventBuff_RdEn <= '0'; 
	DCSBuff_rd_en <= '0';
	FIFOCount <= (others => (others => '0')); EventBuff_WrtEn <= '0';
	TStmpBuff_wr_en <= '0'; TStmpBuff_rd_en <= '0'; EvBuffWrtGate <= '0';
	TStmpBuff_Full <= '0'; TStmpBuff_Emtpy <= '0';
	TxPkCnt <= (others => '0'); Pkt_Timer <= X"0";
	EmptyLatch <= "000"; En_PRBS(0) <= "000";
	FormStatReg <= "000"; GTPRxBuff_wr_en(0) <= '0'; 
	ActiveReg <= X"000000"; LinkFIFOStatReg <= "000";
	Stat_DReq <= '0'; AddrReg <= (others =>'0');
	WdCountBuff_WrtEn <= '0'; WdCountBuff_RdEn <= '0';
	CRCErrCnt <= X"00"; 
	GTPTxBuff_wr_en <= '0';
	GTPRstCnter <= (others=>'0'); GTPRstFromCnt <= '0'; GTPTstFromCntEn <= '1';
	GTPRstArm <= '0';

elsif rising_edge (UsrClk2(0)) then

	if Pkt_Timer = 0 and 
		(Packet_Former = WrtHdrPkt or Packet_Former = WrtCtrlHdrPkt 
		  or Packet_Former = WrtDatPkt or (Packet_Former = WrtDCSPkt and FormStatReg = "111"))
	then GTPTx(0) <= TxCRC(0); GTPTxBuff_In <= TxCRC(0);
	else GTPTx(0) <= GTPTxStage(0); GTPTxBuff_In <= GTPTxStage(0);
	end if;

	if Rx_IsComma(0) = "00" and ReFrame(0) = '0' 
	then GTPRxBuff_wr_en(0) <= '1';
	else GTPRxBuff_wr_en(0) <= '0';
	end if;

-- If a packet header is being received then reset the Rx CRC generator
	  if Rx_IsCtrl(0) = "10" and RxLOS(0)(1) = '0' then RxCRCRst(0) <= '1';
	else RxCRCRst(0) <= '0';
	end if;

	RxCRCRstD(0) <= RxCRCRst(0);
	GTPRxReg(0) <= GTPRx(0);

	  if Rx_IsCtrl(0) = "00" and RxLOS(0)(1) = '0' then RdCRCEn(0) <= '1'; 
	else RdCRCEn(0) <= '0'; 
	end if;

	CommaDL(0)(0) <= Rx_IsComma(0)(0);
	CommaDL(0)(1) <= CommaDL(0)(0);

-- Hold reframe until a vaild pad character set is decoded
	if RxLOS(0)(1) = '0' and InvalidChar(0) = "00" and CommaDL(0) = 1 then Reframe(0) <= '0';
	elsif InvalidChar(0) /= "00" then Reframe(0) <= '1';
	else Reframe(0) <= Reframe(0);
	end if;

	UsrWRDL(0)(0) <= not uCWR and not CpldCS;
   UsrWRDL(0)(1) <= UsrWRDL(1)(0);

	UsrRDDL(0)(0) <= not uCRD and not CpldCS;
	UsrRDDL(0)(1) <= UsrRDDL(0)(0);

	if (uCWR = '0' or uCRD = '0') and CpldCS = '0' then AddrReg <= uCA;
	else AddrReg <= AddrReg;
	end if;

if (UsrRDDL(0) = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = GTPRdAddr0) 
   or (GTPRxBuff_DatCnt(0) >= '0' & X"FFE" and GTPRxBuff_wr_en(0) = '1') -- this line makes the fifo behave like a trace buffer
then GTPRxBuff_rd_en(0) <= '1';
else GTPRxBuff_rd_en(0) <= '0'; 
end if;

-- Use this address to append K28.0 to Dx.y where x is 5 bits of data and
-- y is the packet sequence number to five bits of microcontroller data
	if UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(0)
	 then GTPTx(0) <= X"1C" & TxSeqNo(0) & uCD(4 downto 0);
	      GTPTxBuff_In <= X"1C" & TxSeqNo(0) & uCD(4 downto 0);
			TxCRCDat(0) <= X"0000";
-- Use this address to send unmodified microcontroller data
	elsif UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(2)
	 then GTPTx(0) <= uCD; TxCRCDat(0) <= uCD;
	      GTPTxBuff_In <= uCD;
	-- Use this address to send the check sum
	elsif (UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(4))
	 then GTPTx(0) <= TxCRC(0); TxCRCDat(0) <= X"0000";
	      GTPTxBuff_In <= TxCRC(0);

-- Data header packet ID field is 5 bits wide 
-- The header packet ID is 5 
	elsif Packet_Former = WrtHdrPkt 
	 then
			Case Pkt_Timer is
			 When X"A" => GTPTxStage(0) <= X"1C" & TxSeqNo(0) & "00101"; TxCRCDat(0) <= X"0000";
			 When X"8" => GTPTxStage(0) <= X"A050"; TxCRCDat(0) <= X"A050";
			 When X"7" => GTPTxStage(0) <= "00000" & TxPkCnt; TxCRCDat(0) <= "00000" & TxPkCnt;  
			 When X"6" => GTPTxStage(0) <= TStmpBuff_Out; TxCRCDat(0) <= TStmpBuff_Out;
			 When X"5" => GTPTxStage(0) <= TStmpBuff_Out; TxCRCDat(0) <= TStmpBuff_Out;
			 When X"4" => GTPTxStage(0) <= TStmpBuff_Out; TxCRCDat(0) <= TStmpBuff_Out;
			 When X"0" => GTPTxStage(0) <= X"BC3C"; TxCRCDat(0) <= X"0000";
			 When others => GTPTxStage(0) <= X"0000"; TxCRCDat(0) <= X"0000";
	      end case;
	elsif Packet_Former = WrtCtrlHdrPkt
	 then
			Case Pkt_Timer is
		    When X"A" => GTPTxStage(0) <= X"1C" & TxSeqNo(0) & "00110"; TxCRCDat(0) <= X"0000";
			 When X"9" => GTPTxStage(0) <= X"00" & X"6" & IDReg;
							  TxCRCDat(0) <= X"00" & X"6" & IDReg; 
-- Add the words in the controller header packet to the total word count
			 When X"8" => GTPTxStage(0) <= EventBuff_Out + 8;
			 				  TxCRCDat(0) <= EventBuff_Out + 8;
			 When X"7" => GTPTxStage(0) <= X"00" & ActiveReg(23 downto 16);
							  TxCRCDat(0) <= X"00" & ActiveReg(23 downto 16);
			 When X"6" => GTPTxStage(0) <= ActiveReg(15 downto 0);
			 				  TxCRCDat(0) <= ActiveReg(15 downto 0);
			 When X"5" => GTPTxStage(0) <= DReq_Count(15 downto 0);
			 				  TxCRCDat(0) <= DReq_Count(15 downto 0);
			 When X"4" => GTPTxStage(0) <= EventBuff_Out; TxCRCDat(0) <= EventBuff_Out; -- EventBuff_Out; -- word count 0
			 --When X"3" => GTPTxStage(0) <= uBcheckRef(31 downto 16); TxCRCDat(0) <= uBcheckRef(31 downto 16); -- EventBuff_Out; -- word count 1
			 --When X"2" => GTPTxStage(0) <= uBcheckRef(15 downto  0); TxCRCDat(0) <= uBcheckRef(15 downto  0); -- EventBuff_Out; -- word count 2
			 When X"3" => GTPTxStage(0) <= EventBuff_Out; TxCRCDat(0) <= EventBuff_Out; -- EventBuff_Out; -- word count 1
			 When X"2" => GTPTxStage(0) <= EventBuff_Out; TxCRCDat(0) <= EventBuff_Out; -- EventBuff_Out; -- word count 2

			 When X"0" => GTPTxStage(0) <= X"BC3C"; TxCRCDat(0) <= X"0000";
			 When others => GTPTxStage(0) <= X"0000"; TxCRCDat(0) <= X"0000";
	      end case;
	elsif Packet_Former = WrtDatPkt 
	   then 
		 if    Pkt_Timer = 10 then GTPTxStage(0) <= X"1C" & TxSeqNo(0) & "00110"; TxCRCDat(0) <= X"0000";
		 elsif Pkt_Timer =  0 then GTPTxStage(0) <= X"BC3C"; TxCRCDat(0) <= X"0000";
		 elsif EvTxWdCnt > 0 or EvTxWdCntTC = '1' 
			then GTPTxStage(0) <= EventBuff_Out; TxCRCDat(0) <= EventBuff_Out;
		 else GTPTxStage(0) <= X"0000"; TxCRCDat(0) <= X"0000";
		 end if;
	elsif Packet_Former = WrtDCSPkt
	   then
			Case Pkt_Timer is
			 When X"A" => GTPTxStage(0) <= X"1C" & TxSeqNo(0) & "0" & DCS_Header(7 downto 4); TxCRCDat(0) <= X"0000"; 
			 When X"9" => GTPTxStage(0) <= DCS_EvCnt; TxCRCDat(0) <= DCS_EvCnt; 
			 When X"8" => GTPTxStage(0) <= DCS_Header; TxCRCDat(0) <= DCS_Header;
			 When X"7" => GTPTxStage(0) <= DCS_Status; TxCRCDat(0) <= DCS_Status;  
			 When X"6" => GTPTxStage(0) <= DCSBuff_Out; TxCRCDat(0) <= DCSBuff_Out;
			 When X"5" => GTPTxStage(0) <= DCSBuff_Out; TxCRCDat(0) <= DCSBuff_Out;
			 --When X"4" => GTPTxStage(0) <= X"0002"; TxCRCDat(0) <= X"0002";
			 --When X"3" => GTPTxStage(0) <= X"0003"; TxCRCDat(0) <= X"0003";
			 --When X"2" => GTPTxStage(0) <= X"0004"; TxCRCDat(0) <= X"0004";
			 When X"0" => GTPTxStage(0) <= X"BC3C"; TxCRCDat(0) <= X"0000";
			 When others => GTPTxStage(0) <= X"0000"; TxCRCDat(0) <= X"0000";
	      end case;
-- Pad is K28.5 K28.1 pair
    elsif Packet_Former = Loopback
	    then
		    GTPTxStage(0) <= X"1C90";
	 else GTPTxStage(0) <= X"BC3C"; TxCRCDat(0) <= X"0000";
	end if;

	-- Increment the sequence number and clear CRC when sending Packet ID
	if (UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(0))
		or ((Packet_Former = WrtHdrPkt or Packet_Former = WrtCtrlHdrPkt or Packet_Former = WrtDCSPkt
		or Packet_Former = WrtDatPkt) and Pkt_Timer = 10)
	 then TxSeqNo(0) <= TxSeqNo(0) + 1;
			TxCRCRst(0) <= '1';
	 else TxSeqNo(0) <= TxSeqNo(0);
			TxCRCRst(0) <= '0';
	end if;

-- Accumulate CRC while transmitting data
	if (UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(2))
	 or ((Packet_Former = WrtHdrPkt or Packet_Former = WrtCtrlHdrPkt
	 or (	Packet_Former = WrtDCSPkt and FormStatReg = "111")
	 or Packet_Former = WrtDatPkt) and Pkt_Timer /= 0 and Pkt_Timer /= 10)
	then TxCRCEn(0) <= '1';
	else
	 TxCRCEn(0) <= '0';
	end if;

-- One byte is control when sending the packet ID
	if (UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(0))
		or ((Packet_Former = WrtHdrPkt or Packet_Former = WrtCtrlHdrPkt or Packet_Former = WrtDCSPkt
		or Packet_Former = WrtDatPkt) and Pkt_Timer = 9)
	 then	TxCharIsK(0) <= "10";
-- Two bytes are data when sending the packet payload
	elsif (UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(2))
	 or  ((Packet_Former = WrtHdrPkt or Packet_Former = WrtCtrlHdrPkt 
	      or (	Packet_Former = WrtDCSPkt and FormStatReg = "111") -- neded because it starts with Pkt_Timer = 0, with BC3C
	      or Packet_Former = WrtDatPkt) and Pkt_Timer /= 10 and Pkt_Timer /= 9)
	 then TxCharIsK(0) <= "00";
-- Both bytes are K characters when sending pads
	else TxCharIsK(0) <= "11";
	end if;

	if UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	then FormRst <= uCD(7);
	else FormRst <= '0';
	end if;

-- Count down the nine words of the data request packet being received
	if GTPRxRst = '1' or RxLOS(0)(1) = '1' then TrigReqWdCnt <= X"0";
	elsif Rx_IsComma(0) = "00" and ReFrame(0) = '0' and Rx_IsCtrl(0) = "10" and GTPRx(0)(4 downto 0) = 2
	then TrigReqWdCnt <= X"9";
	elsif Rx_IsComma(0) = "00" and ReFrame(0) = '0' and Rx_IsCtrl(0) = "00" and TrigReqWdCnt /= 0  
	then TrigReqWdCnt <= TrigReqWdCnt - 1;
	else TrigReqWdCnt <= TrigReqWdCnt;
	end if;

	if Rx_IsComma(0) = "00" and RxLOS(0)(1) = '0' and ReFrame(0) = '0' and Rx_IsCtrl(0) = "00" and TrigReqWdCnt > 0
	then DReqBuff_wr_en <= '1';  Debug(10) <= '1';
	else DReqBuff_wr_en <= '0';  Debug(10) <= '0';
	end if;

	if TrigTx_Sel = '1' and DReqBuff_Emtpy = '0'
	then Stat_DReq <= '0';
	else Stat_DReq <= '1';
	end if;

-- Count down the nine words of the heart beat packet being received
	if GTPRxRst = '1' or RxLOS(0)(1) = '1' then HrtBtWrtCnt <= X"0";
	elsif Rx_IsComma(0) = "00" and ReFrame(0) = '0' and Rx_IsCtrl(0) = "10" and GTPRx(0)(4 downto 0) = 1
	then HrtBtWrtCnt <= X"9";
	elsif Rx_IsComma(0) = "00" and ReFrame(0) = '0' and Rx_IsCtrl(0) = "00" and HrtBtWrtCnt /= 0  
	then HrtBtWrtCnt <= HrtBtWrtCnt - 1;
	else HrtBtWrtCnt <= HrtBtWrtCnt;
	end if;

		if Rx_IsComma(0) = "00" and RxLOS(0)(1) = '0' and ReFrame(0) = '0' and Rx_IsCtrl(0) = "00" and HrtBtWrtCnt > 0
	then HrtBtBuff_wr_en <= '1'; --GPO(1) <= '1'; Debug(7) <= '1';
	else HrtBtBuff_wr_en <= '0'; --GPO(1) <= '0'; Debug(7) <= '0';
	end if;

-- Count down the nine words of the DCS packet being received
	if GTPRxRst = '1' or RxLOS(0)(1) = '1' then DCSReqWdCnt <= X"0";
	elsif Rx_IsComma(0) = "00" and ReFrame(0) = '0' and Rx_IsCtrl(0) = "10" and GTPRx(0)(4 downto 0) = 0
	then DCSReqWdCnt <= X"9";
	elsif Rx_IsComma(0) = "00" and ReFrame(0) = '0' and Rx_IsCtrl(0) = "00" and DCSReqWdCnt /= 0  
	then DCSReqWdCnt <= DCSReqWdCnt - 1;
	else DCSReqWdCnt <= DCSReqWdCnt;
	end if;

	if Rx_IsComma(0) = "00" and RxLOS(0)(1) = '0' and ReFrame(0) = '0' and Rx_IsCtrl(0) = "00" and DCSReqWdCnt > 0
	then DCSPktBuff_wr_en <= '1';
	else DCSPktBuff_wr_en <= '0';
	end if;

-- Check CRC
   if (((HrtBtWrtCnt = X"1") or (TrigReqWdCnt = X"1")) and 
	     GTPRxReg(0) /= GTPRxReg(0)) then
		 CRCErrCnt <= CRCErrCnt + 1; 
	else
	    CRCErrCnt <= CRCErrCnt;
	end if;

-- Update Loss Of Sync Counters
   if GTPRxRst = '1' then LosCounter <= (others =>'0');
	elsif RxLOS(0)(1) = '1' then LosCounter <= LosCounter + 1;
	else LosCounter <= LosCounter;
	end if;

-- Store the empty flag values when they make a transition, 
-- then try and send the updated value
	if DreqTxOuts.Done = '1'
	then LinkFIFOStatReg <= LinkFIFOEmpty;
	else LinkFIFOStatReg <= LinkFIFOStatReg;
	end if;

-- Store the time stamp subfield from the trigger request packet for later checking
	if Rx_IsComma(0) = "00" and RxLOS(0)(1) = '0' and ReFrame(0) = '0' and Rx_IsCtrl(0) = "00" 
	and TrigReqWdCnt >= 5 and TrigReqWdCnt <= 7 	
	then TStmpBuff_wr_en <= '1';
	else TStmpBuff_wr_en <= '0';
	end if;

	LinkRDDL(0) <= not CpldCS and not uCRD;
	LinkRDDL(1) <= LinkRDDL(0);

---------------------------------------------------------------------------
--	Idle,Read_Type,Check_Seq_No,Wrt_uC_Queue,Wrt_FPGA_Queue,SendHeartBeat,Check_CRC
---------------------------------------------------------------------------

Case Packet_Parser is
	when Idle =>
	 if GTPRxBuff_Emtpy(0) = '0' then Packet_Parser <= Check_Seq_No;
	 else Packet_Parser <= Idle;
	 end if;
	when Check_Seq_No => 
		if GTPRxBuff_Out(0)(4 downto 0) = 2 
		 then Packet_Parser <= Wrt_FPGA_Queue;
		else Packet_Parser <= Wrt_uC_Queue;
		end if;
	when Wrt_FPGA_Queue => 
		if WrtCount(0) = 0 then Packet_Parser <= Check_CRC;
		else Packet_Parser <= Wrt_FPGA_Queue;
		end if;
	when Wrt_uC_Queue => 
		if WrtCount(0) = 0 then Packet_Parser <= Check_CRC;
		else Packet_Parser <= Wrt_uC_Queue;
		end if;
	when Check_CRC => Packet_Parser <= Idle;
	when others => Packet_Parser <= Idle;
end Case;

if Packet_Parser = Check_Seq_No and GTPRxBuff_Out(0)(7 downto 5) /= RxSeqNo(0)
  then RxSeqNoErr(0) <= '1';
 elsif GTPRst = '1' then RxSeqNoErr(0) <= '0';
 end if;

---------------------------------------------------------------------------
-- Idle,RdInWdCnt0,RdInWdCnt1,RdInWdCnt2,SumWdCnt,WrtWdCnt,RdStat0,
-- RdStat1,RdStat2,WrtStat,WaitEvent,ReadFIFO0,ReadFIFO1,ReaddFIFO2
---------------------------------------------------------------------------
Case Event_Builder is
	when Idle => --Debug(10 downto 7) <= X"0";
		if LinkFIFOEmpty /= 7 and FormHold = '0' and TStmpWds >= 3 
		 then Event_Builder <= WaitEvent;
		else Event_Builder <= Idle;
		end if;
	when WaitEvent => --Debug(10 downto 7) <= X"1";
			-- Wait for a complete event to be in all link FIFOs from active ports
	    if ((LinkFIFOOut(0)(12 downto 0) <= LinkFIFORdCnt(0) and LinkFIFOEmpty(0) = '0') or ActiveReg(7 downto 0) = 0)
	   and ((LinkFIFOOut(1)(12 downto 0) <= LinkFIFORdCnt(1) and LinkFIFOEmpty(1) = '0') or ActiveReg(15 downto 8) = 0)
	   and ((LinkFIFOOut(2)(12 downto 0) <= LinkFIFORdCnt(2) and LinkFIFOEmpty(2) = '0') or ActiveReg(23 downto 16) = 0) 
	    then
		 if ActiveReg(15 downto 0) = 0 then Event_Builder <= RdInWdCnt2;
	  elsif ActiveReg(7 downto 0) = 0 then Event_Builder <= RdInWdCnt1;
	  else Event_Builder <= RdInWdCnt0;
	  end if;
	  elsif FormRst = '1' then Event_Builder <= Idle; 
	  else Event_Builder <= WaitEvent;
	end if;
 -- Read in three word counts in order to sum into a controller word count
	when RdInWdCnt0 => --Debug(10 downto 7) <= X"2"; 
		  if ActiveReg(23 downto 8) = 0 then Event_Builder <= SumWdCnt;
	  elsif ActiveReg(15 downto 8) = 0 then Event_Builder <= RdInWdCnt2;
	  else Event_Builder <= RdInWdCnt1;
	  end if;
	when RdInWdCnt1 => --Debug(10 downto 7) <= X"3";
		if ActiveReg(23 downto 16) = 0 then Event_Builder <= SumWdCnt;
			else Event_Builder <= RdInWdCnt2;
		end if;
	when RdInWdCnt2 => --Debug(10 downto 7) <= X"4";
			Event_Builder <= SumWdCnt;
-- Subtract 2 from each link word count FIFO to account for the word count and status words
	when SumWdCnt => --Debug(10 downto 7) <= X"5"; 
			Event_Builder <= WrtWdCnt;
-- Write the controller word count	
	when WrtWdCnt => --Debug(10 downto 7) <= X"6"; 
		if ActiveReg(15 downto 0) = 0 then Event_Builder <= RdStat2;
	elsif ActiveReg(7 downto 0) = 0 then Event_Builder <= RdStat1;
	  else Event_Builder <= RdStat0;
	end if;  
-- Read the status from the link FIFOs
	when RdStat0 => --Debug(10 downto 7) <= X"7";
		if ActiveReg(23 downto 8) = 0 then Event_Builder <= RduB;
	elsif ActiveReg(15 downto 8) = 0 then Event_Builder <= RdStat2;
	   else Event_Builder <= RdStat1;
		end if;
	when RdStat1 => --Debug(10 downto 7) <= X"8"; 
			if ActiveReg(23 downto 16) = 0 then Event_Builder <= RduB;
			else Event_Builder <= RdStat2;
			end if;
	when RdStat2 => --Debug(10 downto 7) <= X"9"; 
		Event_Builder <= RduB;
-- Write the "OR" of the status as the controller status word

   when RduB => -- this step could be jumped
	if uBinHeader = '0' then
	    Event_Builder <= WrtStat;
	else
	   if ActiveReg(15 downto 0) = 0 then Event_Builder <= RduB2low; -- only FPGA2 active 
	   elsif ActiveReg(7 downto 0) = 0 then Event_Builder <= RduB1low; -- FPGA1 active, FPGA0 not active, FPGA2 maybe active
	   else Event_Builder <= RduB0low; -- FPGA0 active
	   end if;  
   end if;

   when RduB2low =>
	    Event_Builder <= RduB2high;
	when RduB2high =>
	    Event_Builder <= WrtStat;
		 
	when RduB1low =>
		 Event_Builder <= RduB1high;
	when RduB1high =>
		 if ActiveReg(23 downto 16) = 0 then Event_Builder <= WrtStat; -- only FPGA1 active
		 else Event_Builder <= VerifyuB2low; -- verify against FPGA2, FPGA0 is not active
		 end if;
		 
	when VerifyuB2low =>
	    Event_Builder <= VerifyuB2high;
	when VerifyuB2high =>
	    Event_Builder <= WrtStat;
		 
	when RduB0low =>
	    Event_Builder <= RduB0high;
	when RduB0high =>
	    if ActiveReg(23 downto 8) = 0 then Event_Builder <= WrtStat; -- only FPGA0
	    elsif ActiveReg(15 downto 8) = 0 then Event_Builder <= VerifyuB2low; -- only FPGA2, check against it
	    else Event_Builder <= VerifyuB1low; -- verify against FPGA1 and FPGA2
		 end if;
		 
	when VerifyuB1low =>
	    Event_Builder <= VerifyuB1high;
	when VerifyuB1high =>
	    if ActiveReg(23 downto 16) = 0 then Event_Builder <= WrtStat; -- only FPGA2 not active, done
		 else Event_Builder <= VerifyuB2low; -- verify against FPGA2, FPGA0 is not active
		 end if;

	when WrtStat => 
	   if uBwrt = '1' then
		    Event_Builder <= WrtUbLow;
		else
		    Event_Builder <= ReadFIFO;
		end if;
	
	when WrtUbLow => 
	    Event_Builder <= WrtUbHigh;
	when WrtUbHigh =>
	    Event_Builder <= ReadFIFO;
		 
	--Debug(10 downto 7) <= X"A";
--	    Event_Builder <= WrtWdCnt0;
--   when WrtWdCnt0 =>
--	    Event_Builder <= WrtWdCnt1;
--	when WrtWdCnt1 => -- not really nded
--	    Event_Builder <= WrtWdCnt2;
--	when WrtWdCnt2 =>
-- Skip over any Link that has no data
    when ReadFIFO => -- this step doesn't do anything, we could jump it to speed up things
			if FIFOCount(0) /= 0 and ActiveReg(7 downto 0) /= 0 
				then Event_Builder <= ReadFIFO0;
	   elsif FIFOCount(0) = 0 and FIFOCount(1) /= 0 
				and ActiveReg(15 downto 8) /= 0 
				then Event_Builder <= ReadFIFO1; 
	   elsif FIFOCount(0) = 0 and FIFOCount(1) = 0 
				and FIFOCount(2) /= 0 and ActiveReg(23 downto 16) /= 0  
				then Event_Builder <= ReadFIFO2; 
	   else Event_Builder <= Idle;
		end if;
-- Read the data words from the three link FIFOs in succession
	 when ReadFIFO0 => --Debug(10 downto 7) <= X"B";
		if FIFOCount(0) = 1 or FIFOCount(0) = 0 then  
-- Skip over any Link that has no data
				if FIFOCount(1) /= 0 and ActiveReg(15 downto 8) /= 0  
				  then Event_Builder <= ReadFIFO1; 
				 elsif FIFOCount(1) = 0 and FIFOCount(2) /= 0 and ActiveReg(23 downto 16) /= 0 
				  then Event_Builder <= ReadFIFO2;
		       else Event_Builder <= Idle;
		      end if;
		  elsif FormRst = '1' then Event_Builder <= Idle;
		else Event_Builder <= ReadFIFO0;
		end if;
	 when ReadFIFO1 => --Debug(10 downto 7) <= X"C";
		if FIFOCount(1) = 1 or FIFOCount(1) = 0 then
-- Skip over any Link that has no data
			 if FIFOCount(2) /= 0  and ActiveReg(23 downto 16) /= 0  
			   then Event_Builder <= ReadFIFO2;
		     else Event_Builder <= Idle;
			 end if;
		 elsif FormRst = '1' then Event_Builder <= Idle; 
		else Event_Builder <= ReadFIFO1;
		end if;
	 when ReadFIFO2 => --Debug(10 downto 7) <= X"D";
		if FIFOCount(2) = 1 or FIFOCount(2) = 0 
			then Event_Builder <= Idle;
		 elsif FormRst = '1' then Event_Builder <= Idle; 
		else Event_Builder <= ReadFIFO2;
		end if;
	 when others => --Debug(10 downto 7) <= X"E";
	   Event_Builder <= Idle;
  end case;

-- Sum the word counts from the three Link FIFOs.
		if Event_Builder = Idle then EventSum <= (others => '0');
-- Account for removing the word count and status words from the data
	elsif Event_Builder = RdInWdCnt0 then 
	    if uBinHeader = '0' then EventSum <= LinkFIFOOut(0) - 2; else EventSum <= LinkFIFOOut(0) - 4; end if;
	elsif Event_Builder = RdInWdCnt1 then 
	    if uBinHeader = '0' then EventSum <= LinkFIFOOut(1) - 2; else EventSum <= LinkFIFOOut(1) - 4; end if;
	elsif Event_Builder = RdInWdCnt2 then 
	    if uBinHeader = '0' then EventSum <= LinkFIFOOut(2) - 2; else EventSum <= LinkFIFOOut(2) - 4; end if;
	else EventSum <= EventSum;
	end if;
	
-- storte the number of events from the first two FPGAs
      if Event_Builder = Idle then Event0 <= (others => '0');
	elsif Event_Builder = RdInWdCnt0 then Event0 <= LinkFIFOOut(0) - 2;
	else Event0 <= Event0;
	end if;
      if Event_Builder = Idle then Event1 <= (others => '0');
	elsif Event_Builder = RdInWdCnt1 then Event1 <= LinkFIFOOut(1) - 2;
	else Event1 <= Event1;
	end if;
      if Event_Builder = Idle then Event2 <= (others => '0');
	elsif Event_Builder = RdInWdCnt2 then Event2 <= LinkFIFOOut(2) - 2;
	else Event2 <= Event2;
	end if;

-- Select the data source for the event buffer FIFO
	   if Event_Builder = WrtWdCnt  then EventBuff_Dat <= EventSum;
	elsif Event_Builder = WrtStat	then EventBuff_Dat <= uBcheckFlag & Stat0(6 downto 0) & StatOR;
   elsif Event_Builder = WrtWdCnt0 then EventBuff_Dat <= Event0;
	elsif Event_Builder = WrtWdCnt1 then EventBuff_Dat <= Event1;
	elsif Event_Builder = WrtWdCnt2 then EventBuff_Dat <= Event2;
	elsif Event_Builder = WrtUbLow  then EventBuff_Dat <= uBcheck(15 downto 0);
	elsif Event_Builder = WrtUbHigh then EventBuff_Dat <= uBcheck(31 downto 16);
	elsif LinkFIFORdReq(0) = '1' then EventBuff_Dat <= LinkFIFOOut(0);
	elsif LinkFIFORdReq(1) = '1' then EventBuff_Dat <= LinkFIFOOut(1);
	elsif LinkFIFORdReq(2) = '1' then EventBuff_Dat <= LinkFIFOOut(2);
	else EventBuff_Dat <= EventBuff_Dat;
	end if;

-- Do an "or" of the FEB error words for the cotroller error word
   if Event_Builder = RdStat0 then 
				StatOr <= StatOr or LinkFIFOOut(0)(7 downto 0);
				Stat0  <=           LinkFIFOOut(0)(7 downto 0);
elsif Event_Builder = RdStat1 then 
				StatOr <= StatOr or LinkFIFOOut(1)(7 downto 0);
				Stat0 <= Stat0;
elsif Event_Builder = RdStat2 then 
				StatOr <= StatOr or LinkFIFOOut(2)(7 downto 0);
				Stat0 <= Stat0;
else StatOr <= StatOr; Stat0 <= Stat0;
end if;

-- latch uB status for comparison
if Packet_Former = Idle then
    uBcheckRef <= X"ffffffff";
elsif Packet_Former = WrtHdrPkt then
	 if    Pkt_Timer = X"6" then
        uBcheckRef(31 downto 16) <= TStmpBuff_Out;
		  uBcheckRef(15 downto  0) <= uBcheckRef(15 downto 0);
	 elsif Pkt_Timer = X"5" then
	     uBcheckRef(31 downto 16) <= uBcheckRef(31 downto 16);
		  uBcheckRef(15 downto  0) <= TStmpBuff_Out;
	 else
	     uBcheckRef <= uBcheckRef;
	 end if;
end if;



-- read uB numbers from inout buffers
   if Event_Builder = Idle then 
	    if uBdebug2 = '1' then
		    uBcheck <= uBcheck;
		 else
	        uBcheck <= (others =>'1');
		 end if;
	elsif Event_Builder = RdUb0low then 
	    uBcheck(31 downto 16) <= uBcheck(31 downto 16);
		 if uBdebug = '1' then
		     uBcheck(15 downto  0) <= X"aa00";
		 else
		     uBcheck(15 downto  0) <= LinkFIFOOut(0);
		 end if;
	elsif Event_Builder = RdUb0high then 
	    --uBcheck(31 downto 16) <= LinkFIFOOut(0);
		 if uBdebug = '1'  then
		     uBcheck(31 downto  16) <= X"aa01";
		 else
		     uBcheck(31 downto  16) <= LinkFIFOOut(0);
		 end if;
		 uBcheck(15 downto  0) <= uBcheck(15 downto 0);
	elsif Event_Builder = RdUb1low then 
	    uBcheck(31 downto 16) <= uBcheck(31 downto 16);
		 --uBcheck(15 downto  0) <= LinkFIFOOut(1);
		 if uBdebug = '1'  then
		     uBcheck(15 downto  0) <= X"aa02";
		 else
		     uBcheck(15 downto  0) <= LinkFIFOOut(1);
		 end if;
	elsif Event_Builder = RdUb1high then 
	    --uBcheck(31 downto 16) <= LinkFIFOOut(1);
		 if uBdebug = '1'  then
		     uBcheck(31 downto  16) <= X"aa03";
		 else
		     uBcheck(31 downto  16) <= LinkFIFOOut(1);
		 end if;
		 uBcheck(15 downto  0) <= uBcheck(15 downto 0);
	elsif Event_Builder = RdUb2low then 
	    uBcheck(31 downto 16) <= uBcheck(31 downto 16);
		 --uBcheck(15 downto  0) <= LinkFIFOOut(2);
		 if uBdebug = '1'  then
		     uBcheck(15 downto  0) <= X"aa04";
		 else
		     uBcheck(15 downto  0) <= LinkFIFOOut(2);
		 end if;
	elsif Event_Builder = RdUb2high then 
	    --uBcheck(31 downto 16) <= LinkFIFOOut(2);
		 if uBdebug = '1'  then
		     uBcheck(31 downto  16) <= X"aa05";
		 else
		     uBcheck(31 downto  16) <= LinkFIFOOut(2);
		 end if;
		 uBcheck(15 downto  0) <= uBcheck(15 downto 0);
	else uBcheck <= uBcheck;
	end if;
	
-- check if uB are consistent
   if Event_Builder = Idle then 
	    uBcheckFlag <= '0';
	elsif Event_Builder = VerifyUb0low then
	       if (uBcheck(15 downto  0) xor LinkFIFOOut(0)) = X"0000" then
			     uBcheckFlag <= uBcheckFlag;
			 else
				  uBcheckFlag <= '1';
			 end if;
	elsif Event_Builder = VerifyUb0high then
	       if (uBcheck(31 downto  16) xor LinkFIFOOut(0)) = X"0000" then
			     uBcheckFlag <= uBcheckFlag;
			 else
				  uBcheckFlag <= '1';
			 end if;
	elsif Event_Builder = VerifyUb1low then
	       if (uBcheck(15 downto  0) xor LinkFIFOOut(1)) = X"0000" then
			     uBcheckFlag <= uBcheckFlag;
			 else
				  uBcheckFlag <= '1';
			 end if;
	elsif Event_Builder = VerifyUb1high then
	       if (uBcheck(31 downto  16) xor LinkFIFOOut(1)) = X"0000" then
			     uBcheckFlag <= uBcheckFlag;
			 else
				  uBcheckFlag <= '1';
			 end if;
	elsif Event_Builder = VerifyUb2low then
	       if (uBcheck(15 downto  0) xor LinkFIFOOut(2)) = X"0000" then
			     uBcheckFlag <= uBcheckFlag;
			 else
				  uBcheckFlag <= '1';
			 end if;
	elsif Event_Builder = VerifyUb2high then
	       if (uBcheck(31 downto  16) xor LinkFIFOOut(2)) = X"0000" then
			     uBcheckFlag <= uBcheckFlag;
			 else
				  uBcheckFlag <= '1';
			 end if;
   else uBcheckFlag <= uBcheckFlag;
	end if;

--Copy port activity bits from the other FPGAs to this register
if TrigTx_Sel = '1' 
   then 
		if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ActvRegAddrHi 
		  then ActiveReg <= uCD(7 downto 0) & ActiveReg(15 downto 0);
	 elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ActvRegAddrLo 
		then ActiveReg <= ActiveReg(23 downto 16) & uCD;
	  else ActiveReg <= ActiveReg;
	 end if;
   else ActiveReg <= FPGA234_Active(2) & FPGA234_Active(1) & FPGA234_Active(0);
end if;

-- Count down the words read from each of the link FIFOs
	if Event_Builder = RdInWdCnt0 then 
	   if uBinHeader = '0' then FIFOCount(0) <= LinkFIFOOut(0) - 2; else FIFOCount(0) <= LinkFIFOOut(0) - 4; end if;   
	elsif Event_Builder = ReadFIFO0 and FIFOCount(0) /= 0 
						then FIFOCount(0) <= FIFOCount(0) - 1;
	else FIFOCount(0) <= FIFOCount(0);
	end if;

	if Event_Builder = RdInWdCnt1 then 
	    if uBinHeader = '0' then FIFOCount(1) <= LinkFIFOOut(1) - 2; else FIFOCount(1) <= LinkFIFOOut(1) - 4; end if;
	elsif Event_Builder = ReadFIFO1 and FIFOCount(1) /= 0 
						then FIFOCount(1) <= FIFOCount(1) - 1;
	else FIFOCount(1) <= FIFOCount(1);
	end if;

	if Event_Builder = RdInWdCnt2 then 
	    if uBinHeader = '0' then FIFOCount(2) <= LinkFIFOOut(2) - 2; else FIFOCount(2) <= LinkFIFOOut(2) - 4; end if;
	elsif Event_Builder = ReadFIFO2 and FIFOCount(2) /= 0 
						then FIFOCount(2) <= FIFOCount(2) - 1;
	else FIFOCount(2) <= FIFOCount(2);
	end if;

-- Link FIFO reads
-- Microcontroller read
   if (LinkRDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkRdAddr(0))
-- Read of header words, read of data words
   or Event_Builder = RdInWdCnt0 or Event_Builder = RdStat0 or Event_Builder = ReadFIFO0
	or (Event_Builder = RdUb and uBinHeader = '1') or Event_Builder = RdUb0low --or Event_Builder = RdUb0high
	or Event_Builder = VerifyUb0low --or Event_Builder = VerifyUb0high
 	then LinkFIFORdReq(0) <= '1'; 
	else LinkFIFORdReq(0) <= '0'; 
	end if;

 if (LinkRDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkRdAddr(1))
-- Read of header words, read of data words
   or Event_Builder = RdInWdCnt1 or Event_Builder = RdStat1 or Event_Builder = ReadFIFO1
	or (Event_Builder = RdUb and uBinHeader = '1') or Event_Builder = RdUb1low --or Event_Builder = RdUb1high
	or Event_Builder = VerifyUb1low --or Event_Builder = VerifyUb1high
	then LinkFIFORdReq(1) <= '1'; 
	else LinkFIFORdReq(1) <= '0'; 
	end if;

 if (LinkRDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkRdAddr(2))
-- Read of header words, read of data words
   or Event_Builder = RdInWdCnt2 or Event_Builder = RdStat2 or Event_Builder = ReadFIFO2
	or (Event_Builder = RdUb and uBinHeader = '1') or Event_Builder = RdUb2low --or Event_Builder = RdUb2high
	or Event_Builder = VerifyUb2low --or Event_Builder = VerifyUb2high
	then LinkFIFORdReq(2) <= '1'; 
	else LinkFIFORdReq(2) <= '0'; 
	end if;

 if Event_Builder = Idle then EvBuffWrtGate <= '0';
 elsif Event_Builder = WrtStat then EvBuffWrtGate <= '1';
 else EvBuffWrtGate <= EvBuffWrtGate;
 end if;

 if Event_Builder = WrtWdCnt or Event_Builder = WrtWdCnt0 or Event_Builder = WrtWdCnt1 or Event_Builder = WrtWdCnt2
   or Event_Builder = WrtStat or Event_Builder = WrtUbLow or Event_Builder = WrtUbHigh
	or (LinkFIFORdReq /= 0 and EvBuffWrtGate = '1')
   then EventBuff_WrtEn <= '1'; --Debug(6) <= '1';
  else EventBuff_WrtEn <= '0';  --Debug(6) <= '0';
 end if;

if (Packet_Former = WrtCtrlHdrPkt and (Pkt_Timer = 8 or Pkt_Timer = 5 
    or (uBwrt = '1' and (Pkt_Timer = 4 or Pkt_Timer = 3)) -- iff uB is written, also read it again
    ---or Pkt_Timer = 7 or Pkt_Timer = 4 or Pkt_Timer = 3 or Pkt_Timer = 2)
	 ))
 or (Packet_Former = WrtDatPkt and Pkt_Timer > 2 and EvTxWdCnt > 0)
then EventBuff_RdEn <= '1';  Debug(9) <= '1';
else EventBuff_RdEn <= '0';  Debug(9) <= '0';
end if;

if (Packet_Former = WrtDCSPkt and (Pkt_Timer = 7 or Pkt_Timer = 6)) --reads are one cycle ahead
then DCSBuff_rd_en <= '1';
else DCSBuff_rd_en <= '0';
end if;

--Debug(4) <= EventBuff_Empty;

--Debug(3 downto 1) <= LinkFIFOEmpty;

---------------------------------------------------------------------------
-- Idle,WrtPktCnt,WrtHdrPkt,WrtCtrlHdrPkt,WrtDatPkt,WrtDCSPkt
---------------------------------------------------------------------------

Case Packet_Former is 
	when Idle => FormStatReg <= "000"; 
		if EventBuff_Empty = '0' then Packet_Former <= WrtPktCnt;
		elsif DCSBuffRdCnt > 1 then  Packet_Former <= WrtDCSPkt;
		--elsif (LoopbackMode = "1" and Marker = '1') then Packet_Former <= Loopback;
		elsif (LoopbackMode = 1) then Packet_Former <= Loopback;
		elsif (LoopbackMode = 2 and Marker = '1') then Packet_Former <= Loopback;
		elsif (LoopbackMode = 3 and MarkerDelayed = '1') then Packet_Former <= Loopback;
		elsif (LoopbackMode = 4 and MarkerDelayed = '1') then Packet_Former <= Loopback; -- TODO, markerfromfiber
		else Packet_Former <= Idle; --Debug(5 downto 3) <= "000";
		end if;
-- Loopback
   when Loopback => Packet_Former <= Loopback2;
	when Loopback2 =>
	--if (LoopbackMode = "0" or Marker = '0') then Packet_Former <= Idle;
	if (LoopbackMode = 0 or 
	    (LoopbackMode = 2 and Marker = '0') or
		 (LoopbackMode = 3 and MarkerDelayed = '0') or 
		 (LoopbackMode = 4 and MarkerDelayed = '0') ) 
		 then Packet_Former <= Idle;
	else Packet_Former <= Packet_Former;
	end if;
		
-- Divide by eight to get the number of packets
	when WrtPktCnt => Packet_Former <= WrtHdrPkt;  FormStatReg <= "001";  
-- Send the packet header, packet type, packet count, time stamp and status
	when WrtHdrPkt => FormStatReg <= "010"; 
		if Pkt_Timer = 0 then Packet_Former <= WrtCtrlHdrPkt; --Debug(5 downto 3) <= "011";
	    elsif FormRst = '1' then Packet_Former <= Idle; --Debug(5 downto 3) <= "000";
		else Packet_Former <= WrtHdrPkt; --Debug(5 downto 3) <= "101";
		end if;
	when WrtCtrlHdrPkt =>  FormStatReg <= "011";  
		if Pkt_Timer = 0 then Packet_Former <= WrtDatPkt; --Debug(5 downto 3) <= "111";
	 elsif FormRst = '1' then Packet_Former <= Idle; --Debug(5 downto 3) <= "000";
	else Packet_Former <= WrtCtrlHdrPkt; --Debug(5 downto 3) <= "011";
		end if;
-- After Controller header is sent, the packets contain data for this FEB
-- The FEB header data is embedded in the sream coming from the front FPGAs
	when WrtDatPkt => FormStatReg <= "100";   
		if EvTxWdCnt = 0 and Pkt_Timer = 0
			then Packet_Former <= Idle; --Debug(5 downto 3) <= "000";
		 elsif FormRst = '1' then Packet_Former <= Idle; --Debug(5 downto 3) <= "000";
		else Packet_Former <= WrtDatPkt; --Debug(5 downto 3) <= "111";
		end if;
	when WrtDCSPkt => FormStatReg <= "111";
	   if FormStatReg = "111" and Pkt_Timer = 0
		   then Packet_Former <= Idle;
		elsif FormRst = '1' then Packet_Former <= Idle;
		else Packet_Former <= WrtDCSPkt;
		end if;
	when others => Packet_Former <= Idle; FormStatReg <= "101"; --Debug(5 downto 3) <= "000";
end Case;

if Packet_Former = WrtPktCnt then WdCountBuff_WrtEn <= '1';
else  WdCountBuff_WrtEn <= '0';
end if;

if UsrRDDL(0) = 2 and uCA(11 downto 10) = GA and uCA(9 downto 0) = EvWdCntBuffAd
then WdCountBuff_RdEn <= '1';
else WdCountBuff_RdEn <= '0';
end if;

-- Sum word counts, divide by eight and add 1 for the controller header packet
   if Packet_Former = WrtPktCnt and EventBuff_Out(2 downto 0)  = 0 then TxPkCnt <= EventBuff_Out(13 downto 3) + 1;
-- If not and even multiple of eight, account for final partially filled packet
elsif Packet_Former = WrtPktCnt and EventBuff_Out(2 downto 0) /= 0 then TxPkCnt <= EventBuff_Out(13 downto 3) + 2;
-- Decrement the packet count once for each packet ID write
elsif ((Packet_Former = WrtDatPkt or Packet_Former = WrtCtrlHdrPkt) and Pkt_Timer = 9) then TxPkCnt <= TxPkCnt - 1;
else TxPkCnt <= TxPkCnt;
end if;

-- Extract the word count from the event buffer FIFO 
   if Packet_Former = WrtPktCnt
		then EvTxWdCnt <= EventBuff_Out(13 downto 0);
-- Decrement the word count for each word sent within a data packet
	elsif EvTxWdCnt /= 0 and Packet_Former = WrtDatPkt and Pkt_Timer > 2
		then EvTxWdCnt <= EvTxWdCnt - 1;
	else EvTxWdCnt <= EvTxWdCnt;
	end if;

-- Use this word count terminal count to distinguish the last valid word read
-- from the event buffer FIFO
	if EvTxWdCnt = 1 and Packet_Former = WrtDatPkt and Pkt_Timer > 2
		then EvTxWdCntTC <= '1';
	elsif EvTxWdCnt /= 1 and Packet_Former = WrtDatPkt and Pkt_Timer > 2
		then EvTxWdCntTC <= '0';
	else EvTxWdCntTC <= EvTxWdCntTC;
	end if;

-- Read of timestamps for use in forming the header packet
 if (Packet_Former = WrtHdrPkt and Pkt_Timer <= 7 and Pkt_Timer >= 5)
	then TStmpBuff_rd_en <= '1';
	else TStmpBuff_rd_en <= '0';
	end if;

-- Increment the data request counter when forming the header packet.
if Packet_Former = WrtHdrPkt and Pkt_Timer = 9 then DReq_Count <= DReq_Count + 1;
elsif GTPRxRst = '1' or RxLOS(0)(1) = '1' then DReq_Count <= (others => '0');
else DReq_Count <= DReq_Count;
end if;

-- Counter for dividing data into packets
if Packet_Former = WrtPktCnt then Pkt_Timer <= X"A";
elsif Pkt_Timer /= 0 and (Packet_Former = WrtHdrPkt 
or Packet_Former = WrtCtrlHdrPkt or Packet_Former = WrtDatPkt or Packet_Former = WrtDCSPkt)
	then Pkt_Timer <= Pkt_Timer - 1;
elsif Pkt_Timer = 0 and (Packet_Former = WrtHdrPkt 
or Packet_Former = WrtCtrlHdrPkt or Packet_Former = WrtDatPkt or Packet_Former = WrtDCSPkt)
	then Pkt_Timer <= X"A";
elsif Packet_Former = Idle then Pkt_Timer <= X"0";
else Pkt_Timer <= Pkt_Timer;
end if;

-- output trace buffer
if GTPTxStage(0) /= X"BC3C"
then GTPTxBuff_wr_en <= '1';
else GTPTxBuff_wr_en <= '0';
end if;

if (UsrRDDL(0) = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = GTPTxRdAddr) 
   or (GTPTxBuff_DatCnt >= "0" & X"FFE" and GTPTxBuff_wr_en = '1') -- this line makes the fifo behave like a trace buffer
then GTPTxBuff_rd_en <= '1';
else GTPTxBuff_rd_en <= '0'; 
end if;

    if GTPRstArm = '1' then
	     GTPRstCnter <= (others=>'0');
	 else -- if not armed count CCs to arm the reset logic 
		 if (Rx_IsCtrl(0) & InvalidChar(0) & Rx_IsComma(0) & Reframe(0) & TDisA) = X"CC" 
		 then
			  GTPRstCnter <= GTPRstCnter + 1;
		 else
			  GTPRstCnter <= GTPRstCnter; -- (others=>'0') to count successes in a row
		 end if;	 
	 end if; 
	
	 if ((LosCounter > 8) and GTPRstArm = '1') then
	     GTPRstFromCnt <= '1';
		  GTPRstArm <= '0';
	 elsif GTPRstCnter(6) = '1' then -- better, count in a row, use as threshold of ~5?
	     GTPRstArm <= '1';
		  GTPRstFromCnt <= '0';
	 else 
	     GTPRstArm <= GTPRstArm;
		  GTPRstFromCnt <= '0';
	 end if;
	 --   if GTPRstCnter(10) = '1' then 
	--	      GTPRstCnter <= (others=>'0');
	--	      GTPRstFromCnt <= '1';
		-- else GTPRstCnter <= GTPRstCnter + 1;
	--	      GTPRstFromCnt <= '0';
	--	 end if;
	-- else GTPRstCnter <= (others=>'0');
	--      GTPRstFromCnt <= '0';
	-- end if;
	 
	--if LosCounter > 8 then
	--    GTPRstFromCnt <= '1';
	--else
	--    GTPRstFromCnt <= '0';
	--end if;
	 
	if UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPRstCntAd 
	then GTPTstFromCntEn <= uCD(0);
	else GTPTstFromCntEn <= GTPTstFromCntEn;
	end if;

end if; -- CpldRst

end process;

GTP1I_O : process (UsrClk2(1), CpldRst)

begin

 if CpldRst = '0' then 

	CommaDL(1) <= "00"; GTPRxReg(1) <= X"0000";
	UsrWRDL(1) <= "00"; UsrRDDL(1) <= "00"; Reframe(1) <= '1'; 
	TxCharIsK(1) <= "11"; GTPTx(1) <= X"BC3C";
	TxSeqNo(1) <= "000"; TxCRCRst(1) <= '0';
   TxCRCEn(1) <= '0'; RdCRCEn(1) <= '0'; 
	RxCRCRst(1) <= '0'; RxCRCRstD(1) <= '0'; TxCRCDat(1) <= X"0000";
	GTPRxBuff_wr_en(1) <= '0'; PRBSCntRst(1) <= '0';
	En_PRBS(1) <= "000"; DReq_Tx_Ack <= '0';
	IntTrigSeq <= Idle; Packet_Type <= X"1";
	HrtBtDone <= '0'; HrtBtTxAck <= '0';
	
elsif rising_edge (UsrClk2(1)) then

	UsrRDDL(1)(0) <= not uCRD and not CpldCS;
	UsrRDDL(1)(1) <= UsrRDDL(1)(0);

-- Use the data count to make the FIFO behave like a trace buffer.
if (UsrRDDL(1) = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = GTPRdAddr1)
	or (GTPRxBuff_DatCnt(1) >= '0' & X"FFE" and GTPRxBuff_wr_en(1) = '1')
then GTPRxBuff_rd_en(1) <= '1';
else GTPRxBuff_rd_en(1) <= '0'; 
end if;


	if Rx_IsComma(1) = "00" and ReFrame(1) = '0' 
	then GTPRxBuff_wr_en(1) <= '1'; 
	else GTPRxBuff_wr_en(1) <= '0'; 
	end if;

	if Rx_IsCtrl(1) = "10" then RxCRCRstD(1) <= '1';
	else RxCRCRstD(1) <= '0';
	end if;

	RxCRCRst(1) <= RxCRCRstD(1);
	GTPRxReg(1) <= GTPRx(1);

-- Enable the check sum for non-control words.
	if Rx_IsCtrl(1) = "00" then RdCRCEn(1) <= '1'; 
	else RdCRCEn(1) <= '0'; 
	end if;

	CommaDL(1)(0) <= Rx_IsComma(1)(0);
	CommaDL(1)(1) <= CommaDL(1)(0);

	if InvalidChar(1) = "00" and CommaDL(1) = 1 then Reframe(1) <= '0';
	elsif InvalidChar(1) /= "00" then Reframe(1) <= '1';
	else Reframe(1) <= Reframe(1);
	end if;

	UsrWRDL(1)(0) <= not uCWR and not CpldCS;
   UsrWRDL(1)(1) <= UsrWRDL(1)(0);

-- Request/Acknowledges to cross clock domains
	if DReq_Tx_Ack = '0' and Dreq_Tx_Req = '1' then 
	 DReq_Tx_Ack <= '1';
	elsif Packet_Type = X"2" then
	 DReq_Tx_Ack <= '0';
	end if;
	
	 HrtBtTxAck <= HrtBtTxReq;

-- State machine for sending trigger requests from internal trigger generator
-- Idle,SendTrigHdr,SendPktType,SendPad0,SenduBunch0,SenduBunch1,
--	SenduBunch2,SendPad1,SendPad2,SendPad3,SendCRC
Case IntTrigSeq is
	when Idle =>
	  if HrtBtTxAck = '1' then IntTrigSeq <= SendTrigHdr;
	   else IntTrigSeq <= Idle;
	  end if;
	when SendTrigHdr => IntTrigSeq <= SendPad0;
	when SendPad0 => IntTrigSeq <= SendPktType;
	when SendPktType =>  IntTrigSeq <= SenduBunch0;
	when SenduBunch0 => IntTrigSeq <= SenduBunch1;
	when SenduBunch1 => IntTrigSeq <= SenduBunch2;
	when SenduBunch2 => IntTrigSeq <= SendPad1;
	when SendPad1 => IntTrigSeq <= SendPad2;
	when SendPad2 => IntTrigSeq <= SendPad3;
	when SendPad3 => IntTrigSeq <= WaitCRC;
	when WaitCRC => IntTrigSeq <= SendCRC;
	when SendCRC => IntTrigSeq <= SetPktType;
	when SetPktType => 
		if Packet_Type = X"1" and DReq_Tx_Ack = '1' then
			Packet_Type <= X"2";
			IntTrigSeq <= SendTrigHdr;
		else
			Packet_Type <= X"1";
			IntTrigSeq <= Idle;
	   end if;
	when others => IntTrigSeq <= Idle;
end Case;

	if IntTrigSeq = SetPktType and Packet_Type = X"1" then 
		HrtBtDone <= '1';
	elsif
		HrtBtFMTxEn = '1'
	then 
		HrtBtDone <= '0';
	end if;

-- Use this address to append K28.0 to Dx.y where x is 5 bits of data and
-- y is the packet sequence number
	if (UsrWRDL(1) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(1))
	   or IntTrigSeq = SendTrigHdr  
	 then if IntTrigSeq = SendTrigHdr 
				then GTPTx(1) <= X"1C" & TxSeqNo(1) & '0' & Packet_Type;
				else GTPTx(1) <= X"1C" & TxSeqNo(1) & uCD(4 downto 0);
			 end if;
		   TxSeqNo(1) <= TxSeqNo(1) + 1;
			TxCharIsK(1) <= "10";
			TxCRCRst(1) <= '1';
			TxCRCEn(1) <= '0';
			TxCRCDat(1) <= (others => '0');
-- Use this address to send unmodified data
	elsif (UsrWRDL(1) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(3))
	   or IntTrigSeq = SendPad0
	 then if IntTrigSeq = SendPad0
				then GTPTx(1) <= (others => '0');
					  TxCRCDat(1) <= (others => '0');
				else GTPTx(1) <= uCD;
					  TxCRCDat(1) <= uCD;
			 end if;
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
	 elsif IntTrigSeq = SendPktType  
	  then GTPTx(1) <= X"00" & Packet_Type & X"0";
			TxCRCDat(1) <= X"00" & Packet_Type & X"0";
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
	 elsif IntTrigSeq = SenduBunch0  
	  then GTPTx(1) <= IntuBunchCount(15 downto 0);
	       TxCRCDat(1) <= IntuBunchCount(15 downto 0); 
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
	 elsif IntTrigSeq = SenduBunch1  
	  then GTPTx(1) <= IntuBunchCount(31 downto 16);
	      TxCRCDat(1) <= IntuBunchCount(31 downto 16); 
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
	 elsif IntTrigSeq = SenduBunch2  
	  then GTPTx(1) <= IntuBunchCount(47 downto 32);
	      TxCRCDat(1) <= IntuBunchCount(47 downto 32); 
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
	 elsif IntTrigSeq = SendPad1  
	  then GTPTx(1) <= (others => '0');
			 TxCRCDat(1) <= (others => '0');
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
	 elsif IntTrigSeq = SendPad2  
	  then GTPTx(1) <= (others => '0');
	  		TxCRCDat(1) <= (others => '0');
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
	 elsif IntTrigSeq = SendPad3  
	  then GTPTx(1) <= (others => '0');
			TxCRCDat(1) <= (others => '0');
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
	 elsif IntTrigSeq = WaitCRC  
	  then GTPTx(1) <= X"BC3C";
			TxCRCDat(1) <= (others => '0');
			TxCharIsK(1) <= "11";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '0';
-- Use this address to send the check sum
	elsif (UsrWRDL(1) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(5))
			or IntTrigSeq = SendCRC
	 then GTPTx(1) <= TxCRC(1);
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '0';
-- Pad is K28.5 K28.1 pair
	 else GTPTx(1) <= X"BC3C";
			TxCharIsK(1) <= "11";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '0';
	end if;

end if;

end process;

-- Fifo for buffering microcontoller parallel data prior to serializing
-- Used for controlling the 96 harmonica jack LEDs
CMDFifo : CMD_Fifo
  PORT MAP (clk => SysClk,
    rst => ResetHi,
    din(18 downto 16) => uCA(2 downto 0),
	 din(15 downto 0) => uCD,
    wr_en => CMDwr_en,
    rd_en => CMDrd_en,
    dout => CMD_Out,
    full => CMD_Full,empty => CMD_Empty);

-- Fifo for buffering microcontoller parallel data prior to serializing
-- Used for the stup registers on the ADF4001
PLLBuff: PLL_Buff
  PORT MAP (clk => SysClk,
    rst => ResetHi,
    din(23 downto 16) => PllStage,
	 din(15 downto 0) => uCD,
    wr_en => PLLBuffwr_en,
    rd_en => PLLBuffrd_en,
    dout => PLLBuff_Out,
    full => PLLBuff_full,
    empty => PLLBuff_empty);

---------------------------------------------------------------------------
-- Logic for the serial inputs from the three FPGAs attached to the FEBs --
---------------------------------------------------------------------------
GenSerdes : for i in 0 to 2 generate

-- Collect two data lanes and a frame signal into a three bit vector
-- Deserialize x 5
SerDesInP(i) <= (LinkFR_P(i) & LinkSDat_P(2*i+1) & LinkSDat_P(2*i));
SerDesInN(i) <= (LinkFR_N(i) & LinkSDat_N(2*i+1) & LinkSDat_N(2*i));

-- Deserializer macro refer to XAPP1064
LVDSInClk0 : serdes_1_to_n_clk_ddr_s8_diff generic map(
      	S			=> S) 		
port map (
	clkin_p   		=> LINKClk_P(i),
	clkin_n   		=> LINKClk_N(i),
	rxioclkp    	=> rxioclkp(i),
	rxioclkn   		=> rxioclkn(i),
	rx_serdesstrobe => rx_serdesstrobe(i),
	rx_bufg_x1		=> RxOutClk(i));

-- Data Inputs
LVDSInDat0 : serdes_1_to_n_data_ddr_s8_diff generic map(
      	S		=> S,			
      	D		=> D)
port map (                   
	use_phase_detector 	=> '1',	-- '1' enables the phase detector logic
	datain_p     	=> SerDesInP(i),
	datain_n     	=> SerDesInN(i),
	rxioclkp    	=> rxioclkp(i),
	rxioclkn   		=> rxioclkn(i),
	rxserdesstrobe => rx_serdesstrobe(i),
	gclk    		=> RxOutClk(i), -- this clock is assymmetric beacuse of the odd serialization factor
	bitslip   	=> SlipReq(i),
	reset   		=> SerdesRst,
  data_out(14 downto 10)  => LinkFRDat(i),
  data_out(9) => LinkPDat(i)(1)(0), -- the serial data goes out msb first, comes in lsb first
  data_out(8) => LinkPDat(i)(1)(1), -- so bit order needs to be reversed
  data_out(7) => LinkPDat(i)(1)(2),
  data_out(6) => LinkPDat(i)(1)(3),
  data_out(5) => LinkPDat(i)(1)(4),
  data_out(4) => LinkPDat(i)(0)(0),
  data_out(3) => LinkPDat(i)(0)(1),
  data_out(2) => LinkPDat(i)(0)(2),
  data_out(1) => LinkPDat(i)(0)(3),
  data_out(0) => LinkPDat(i)(0)(4),
  debug_in  	=> "00",
  debug    		=> open);

-- Extract the eight payload bits from the 10 bit parallel data fron the deserializer
-- Three lower bits from lane 1 and 5 bits from lane 0
LinkBuff : LinkFIFO
  port map (rst => LinkBuffRst, wr_clk => RxOutClk(i), rd_clk => UsrClk2(0), 
    wr_en => LinkFIFOWrReq(i),rd_en => LinkFIFORdReq(i),
    din(15 downto 13) => LinkPDat(i)(1)(7 downto 5),
    din(12 downto 8) => LinkPDat(i)(0)(9 downto 5),
    din( 7 downto 5) => LinkPDat(i)(1)(2 downto 0),
    din( 4 downto 0) => LinkPDat(i)(0)(4 downto 0),
    dout => LinkFIFOOut(i), empty => LinkFIFOEmpty(i),
	 full => LinkFIFOFull(i),
	 rd_data_count => LinkFIFORdCnt(i));

end generate;

--------------- Logic clocked with Serdes receive clocks ---------------

-- Three links from 3 FPGAs. Two lane serial with a 50MHz frame and 250MHz 
-- double data rate clock. V-Valid flag, D-high byte d-low byte

-- Clk0    -_-_-_-_-_-_-_-_-_-_
-- Frame0  -----_____-----_____
-- Lane 01 V1DDDV1dddV1DDDV1ddd
-- Lane 00 DDDDDdddddDDDDDddddd

-- Clk1    -_-_-_-_-_-_-_-_-_-_
-- Frame1  _____-----_____-----
-- Lane 11 V1DDDV1dddV1DDDV1ddd
-- Lane 10 DDDDDdddddDDDDDddddd

-- Clk2    -_-_-_-_-_-_-_-_-_-_
-- Frame2  -----_____-----_____
-- Lane 21 V1DDDV1dddV1DDDV1ddd
-- Lane 20 DDDDDdddddDDDDDddddd

GenLinkBuff : for i in 0 to 2 generate

LinkBuff : process (RxOutClk(i), CpldRst)

begin

 if CpldRst = '0' then 
 
	LinkPDat(i)(1)(9 downto 5) <= (others => '0'); 
	LinkPDat(i)(0)(9 downto 5) <= (others => '0'); 
	LinkFIFOWrReq(i) <= '0'; FPGA234_Active(i) <= (others => '0'); 
	SlipReq(i) <= '0'; Slippause(i) <= X"0";  ActiveCE(i) <= '0';

elsif rising_edge (RxOutClk(i)) then

-- Engage bit slip if shifted in framing signal isn't all 1's or all 0's
	if LinkFRDat(i) /= 0 and LinkFRDat(i) /= 31 and Slippause(i) = 0
	then Slippause(i) <= X"F";
	elsif Slippause(i) /= 0
	then  Slippause(i) <= Slippause(i) - 1;
	else Slippause(i) <= Slippause(i);
	end if;

-- Allow time between requests for bit slip to take effect
	if Slippause(i) = X"F" then SlipReq(i) <= '1';	
	else SlipReq(i) <= '0';
	end if;

-- Copy five bit shift result to five bit register to form a 10 bit result
	LinkPDat(i)(1)(9 downto 5) <= LinkPDat(i)(1)(4 downto 0);
	LinkPDat(i)(0)(9 downto 5) <= LinkPDat(i)(0)(4 downto 0);

-- Link Frame1 is reversed. Deal with that here.
if i = 1 then
-- Write to the Link FIFO when the frame signal indicates data is word aligned
	if LinkFRDat(i) = 0 and LinkPDat(i)(1)(4 downto 3) = "11"
			then LinkFIFOWrReq(i) <= '1';
	      else LinkFIFOWrReq(i) <= '0';
	end if;
-- Use the spare link receive bit to retrieve activity bits from the fron FPGAs
	if LinkFRDat(i) = 0 and LinkPDat(i)(1)(4 downto 3) = "10"
	  then ActiveCE(i) <= '1'; 
	 else  ActiveCE(i) <= '0';
	end if;
else
	if LinkFRDat(i) = 31 and LinkPDat(i)(1)(4 downto 3) = "11"
		then LinkFIFOWrReq(i) <= '1';
	   else LinkFIFOWrReq(i) <= '0';
	end if;
	if LinkFRDat(i) = 31 and LinkPDat(i)(1)(4 downto 3) = "10"
	  then ActiveCE(i) <= '1'; 
	 else  ActiveCE(i) <= '0';
	end if;
end if;

if ActiveCE(i) = '1' then 
     FPGA234_Active(i)(7 downto 5) <= LinkPDat(i)(1)(2 downto 0);
	  FPGA234_Active(i)(4 downto 0) <= LinkPDat(i)(0)(4 downto 0);
else 
	  FPGA234_Active(i) <= FPGA234_Active(i);
end if;
--    din( 7 downto 5) => LinkPDat(i)(1)(2 downto 0),
--    din( 4 downto 0) => LinkPDat(i)(0)(4 downto 0),

end if; -- CpldRst = '0'

end process;

end generate;

-- Reset for the input deserializer
SerdesRst <= '1' when CpldRst = '0' 
  	                or (CpldCS = '0' and uCWR = '0' and uCA = LinkCSRAddr and uCD(8) = '1') else '0';
LinkBuffRst <= '1' when CpldRst = '0' 
  	                or (CpldCS = '0' and uCWR = '0' and uCA = LinkCSRAddr and uCD(9) = '1') else '0';

ResetHi <= not CpldRst;  -- Generate and active high reset for the Xilinx macros

----------------------- Orange tree interface logic -----------------------------

 DQ <= DQWrtDly(2) when DQEn = '1' else (others => 'Z'); 
iDQ <= DQ when EthRDDL(4 downto 3) = 1 else iDQ;

EthProc : process(EthClk, CpldRst)

 begin 

-- asynchronous reset/preset
 if CpldRst = '0' then

	ZEthClk <= '0'; EthWRDL <= (others => '0');
	DQWrtDly <= (others => (others => '0'));
	ZEthA <= (others => '0'); DQEn <= '0';
	ZEthCS <= '1'; ZEthWE <= '1'; 
	ZEthBE <= "11"; EthRDDL <= (others => '0');
	MarkerBits <= X"0000"; Even_Odd <= '0'; 
	GPO(0) <= '0'; Marker <= '0'; 
	MarkerCnt <= (others => '0'); MarkerCnt2 <= (others => '0');
	MarkerCnt3 <= (others => '0'); MarkerCnt4 <= (others => '0'); MarkerCnt5 <= (others => '0');
	BnchMarkerLast <= (others => '0');
	

 elsif rising_edge (EthClk) then 

	MarkerBits <= MarkerBits(13 downto 0) & DDRBits;
	-- old style with punched clock
   ----if MarkerMode = '0' then
	--   if Marker = '0' and (MarkerBits = X"F0C0" or MarkerBits = X"F0FC") then  
	--	   GPO(0) <= '1'; Marker <= '1'; 
	--	   MarkerCnt <= MarkerCnt + 1;
	--   elsif Marker = '1' and (MarkerBits = X"FCF0" or MarkerBits = X"C0F0") then 
	--      GPO(0) <= '0'; Marker <= '0';
	--	   MarkerCnt <= MarkerCnt;
	--   else Marker <= Marker; GPO(0) <= GPO(0); MarkerCnt <= MarkerCnt;
	--   end if;
   --else 
	--end if;
	
	-- new style, punch or dedicated marker on separate line
	BnchMarkerLast(0) <= BnchMarker;
	BnchMarkerLast(7 downto 1) <= BnchMarkerLast(6 downto 0);
	
	-- full marker
	if BnchMarkerLast = "11101000" or
	   BnchMarkerLast = "10001110" then
	   MarkerCnt2 <= MarkerCnt2 + 1;
	else
	   MarkerCnt2 <= MarkerCnt2;
	end if;
	
	if Marker = '0' and (BnchMarkerLast = "11101000" or
	   BnchMarkerLast = "10001110") then
	   MarkerCnt <= MarkerCnt + 1;
		Marker <= '1'; 
	elsif Marker = '1' and BnchMarkerLast = "11001100" then
	   Marker <= '0';
		MarkerCnt <= MarkerCnt;
	else
	   MarkerCnt <= MarkerCnt;
		Marker <= Marker;
	end if;
	
	
	--if BnchMarker = '1' then
	--if (MarkerBits(7) = '1' and
	--    MarkerBits(5) = '0' and
	--	 MarkerBits(3) = '0' and
	--	 MarkerBits(1) = '0') or 
	--	(MarkerBits(7) = '1' and
	--    MarkerBits(5) = '1' and
	--	 MarkerBits(3) = '1' and
	--	 MarkerBits(1) = '0') then

	--if BnchMarkerLast = "00001111" then
	--if (MarkerBits(6) = '1' and
	--    MarkerBits(4) = '0' and
	--	 MarkerBits(2) = '0' and
	--	 MarkerBits(0) = '0') or 
	--	(MarkerBits(6) = '1' and
	--    MarkerBits(4) = '1' and
	--	 MarkerBits(2) = '1' and
	--	 MarkerBits(0) = '0') then
	if BnchMarkerLast = "00001111" then
	   MarkerCnt3 <= MarkerCnt3 + 1;
	else
	   MarkerCnt3 <= MarkerCnt3;
	end if;
	
	--if BnchMarkerLast(3 downto 0) = "0011" then
	--if (MarkerBits(14) = '1' and
	--    MarkerBits(12) = '1' and
	--	 MarkerBits(10) = '0' and
	--	 MarkerBits(8)  = '0' and
	--    MarkerBits(6)  = '1' and
	--    MarkerBits(4)  = '0' and
	--	 MarkerBits(2)  = '0' and
	--	 MarkerBits(0)  = '0'
	--	 ) or 
	--	(MarkerBits(14) = '1' and
	--    MarkerBits(12) = '1' and
	--	 MarkerBits(10) = '0' and
	--	 MarkerBits(8)  = '0' and
	--    MarkerBits(6)  = '1' and
	--    MarkerBits(4)  = '0' and
	--	 MarkerBits(2)  = '0' and
	--	 MarkerBits(0)  = '0') then
	if BnchMarkerLast(3 downto 0) = "0011"	then
	   MarkerCnt4 <= MarkerCnt4 + 1;
	else
	   MarkerCnt4 <= MarkerCnt4;
	end if;
	
	--if BnchMarkerLast(1 downto 0) = "01" then
	--if (MarkerBits(15) = '1' and
	--    MarkerBits(13) = '1' and
	--	 MarkerBits(11) = '0' and
	--	 MarkerBits(9)  = '0' and
	--    MarkerBits(7)  = '1' and
	--    MarkerBits(5)  = '0' and
	--	 MarkerBits(3)  = '0' and
	--	 MarkerBits(1)  = '0'
	--	 ) or 
	--	(MarkerBits(15) = '1' and
	--    MarkerBits(13) = '1' and
	--	 MarkerBits(11) = '0' and
	--	 MarkerBits(9)  = '0' and
	--    MarkerBits(7)  = '1' and
	--    MarkerBits(5)  = '0' and
	--	 MarkerBits(3)  = '0' and
	--	 MarkerBits(1)  = '0') then
	if BnchMarkerLast(1 downto 0) = "01" then
	   MarkerCnt5 <= MarkerCnt5 + 1;
	else
	   MarkerCnt5 <= MarkerCnt5;
	end if;
	

	

   GPO(1) <= BnchMarker;

--	if GPO(1) = '0' and MarkerBits = X"F0C0"
--	  then GPO(1) <= '1';
--	elsif GPO(1) = '1' and MarkerBits = X"F0FC"
--	  then GPO(1) <= '0';
--	 else GPO(1) <= GPO(1);
--	end if;

	if MarkerBits = X"C0F0" then Even_Odd <= '1';
	elsif MarkerBits = X"FCF0" then Even_Odd <= '0';  
	else Even_Odd <= Even_Odd; 
	end if;

	ZEthClk <= not ZEthClk;

-- Strobe timing delay chains
-- Write strobe timer
	if ZEthClk = '1' then
	 EthWRDL(0) <= not EthCS and not uCWR;
	 EthWRDL(1) <= EthWRDL(0);
	 EthWRDL(2) <= EthWRDL(1);
	 EthWRDL(3) <= EthWRDL(2);
	 EthWRDL(4) <= EthWRDL(3);
	else EthWRDL <= EthWRDL;
	end if; 

-- Read strobe timer
	if ZEthClk = '1' then
	 EthRDDL(0) <= not EthCS and not uCRD;
	 EthRDDL(1) <= EthRDDL(0);
	 EthRDDL(2) <= EthRDDL(1);
	 EthRDDL(3) <= EthRDDL(2);
	 EthRDDL(4) <= EthRDDL(3);
	else EthRDDL <= EthRDDL;
	end if; 

 -- Write data pipeline
if EthCS = '0' and uCWR = '0' then DQWrtDly(0) <= uCD;
 else DQWrtDly(0) <= DQWrtDly(0);
 end if;
 
 if ZEthClk = '1' then 
		DQWrtDly(1) <= DQWrtDly(0);
		DQWrtDly(2) <= DQWrtDly(1);
 else DQWrtDly(1) <= DQWrtDly(1);
		DQWrtDly(2) <= DQWrtDly(2);
  end if;

-- Tri state enable for read data
 if ZEthClk = '1' and DQEn = '0' and EthWRDL(2 downto 1) = 1 then DQEn <= '1'; 
  elsif ZEthClk = '1' and DQEn = '1' and EthWRDL(4 downto 3) = 1 then DQEn <= '0';
   else DQEn <= DQEn; 
 end if;

-- Chip enable and byte select
	if ZEthClk = '1' and ZEthCS = '1' and (EthWRDL(1 downto 0) = 1 or EthRDDL(1 downto 0) = 1) then 
		ZEthCS <= '0'; 
-- use a specific address to access any trailing bytes from a series of word accesses
		if uCA(8) = '0' and uCA(3 downto 0) = "1001" then 
				ZEthBE <= "01";
			else 
				ZEthBE <= "00";
			end if;
	elsif ZEthClk = '1' and ZEthCS = '0' and (EthWRDL(2 downto 1) = 1 or EthRDDL(2 downto 1) = 1) then
		ZEthCS <= '1';
		ZEthBE <= "11";
	else ZEthCS <= ZEthCS;
		 ZEthBE <= ZEthBE;
	end if;

-- Latch the address
	if EthCS = '0' and (uCWR = '0' or uCRd = '0') and uCA(8) = '0' and uCA(3 downto 0) = "1001"  
		then ZEthA <= uCA(8 downto 1) & '0'; 
	elsif EthCS = '0' and (uCWR = '0' or uCRd = '0') and (uCA(8) = '1' or uCA(3 downto 0) /= "1001")
		then ZEthA <= uCA(8 downto 0); 
	else ZEthA <= ZEthA; 
	end if;

-- Write strobe
		if ZEthClk = '1' and ZEthWE = '1' and EthWRDL(1 downto 0) = 1 
			then ZEthWE <= '0'; 
	elsif (ZEthClk = '1' and ZEthWE = '0' and EthWRDL(2 downto 1) = 1) or uCRd = '0'
			then ZEthWE <= '1'; 
	else ZEthWE <= ZEthWE;
	end if;

end if;

end process;

FMTxReq : process(Clk80MHz, CpldRst, GTPRxRst)

 begin 
-- asynchronous reset/preset
 if CpldRst = '0' or GTPRxRst = '1' then

	HrtBtFMTxEn <= '0'; FMTxBsy <= '0';
	MarkerDelayCounter <= (others => '0');
	HeartBeatCnt <= (others => '0');
	WindowTimer <= (others => '0');
	MarkerDelayedCnt <= (others => '0');
	LastWindow <= X"FFFF";
	InjectionTs <= X"FFFF";
	NimTrigLast <= '0';
	MarkerDelayArm <= '0';
	MarkerLast <= "00";

 elsif rising_edge(Clk80MHz) then
 
-- Counter to manage delay of marker receipt
   MarkerLast(0) <= Marker; 
	MarkerLast(1) <= MarkerLast(0);
  

  --if Marker = '1' and MarkerLast = '0' -- rising edge of the marker, marker is 90deg shifted, markerlast is sync
  if MarkerLast = "01"
  --if Marker = '1'
 		then 
		    MarkerDelayCounter <= MarkerDelay; 
			 MarkerDelayArm     <= '1';
			 MarkerDelayedCnt   <= MarkerDelayedCnt;
	elsif MarkerDelayCounter > 0
		then 
		    MarkerDelayCounter <= MarkerDelayCounter - 1; 
		    MarkerDelayArm     <= MarkerDelayArm;
			 MarkerDelayedCnt   <= MarkerDelayedCnt;
	elsif MarkerDelayCounter = 0 and MarkerDelayed = '0' and MarkerDelayArm = '1'
		then 
		    MarkerDelayed  <= '1'; 
			 MarkerDelayArm <= '0';
			 MarkerDelayedCnt <= MarkerDelayedCnt + 1;
	else
		MarkerDelayed  <= '0'; 
		MarkerDelayArm <= MarkerDelayArm;
		MarkerDelayedCnt   <= MarkerDelayedCnt;
  end if;

-- Send a heart beat without pause if there is no marker input expected
  if HrtBtFMReq = '1' or (MarkerSyncEn = '1' and MarkerDelayed = '1')
	  then 
	      HrtBtFMTxEn <= '1'; 
			HeartBeatCnt <= HeartBeatCnt + 1;
			WindowTimer <= (others => '0');
			LastWindow <= WindowTimer;
  else 
      HrtBtFMTxEn <= '0'; 
		HeartBeatCnt <= HeartBeatCnt;
		WindowTimer <= WindowTimer + 1;
		LastWindow <= LastWindow;
  end if;
  
  -- detect NimTrig transition to hight and store the current time stamp
  NimTrigLast <= NimTrig;
  if NimTrig = '1' and NimTrigLast = '0'
    then
      InjectionTs <= WindowTimer;
  else 
      InjectionTs <= InjectionTs;
  end if;

 if HrtBtBuff_rd_en = '1' 
     then FMTxBsy <= '1'; Debug(8) <= '1';
 elsif HrtBtTxOuts.Done = '1'
     then FMTxBsy <= '0'; Debug(8) <= '0';
 end if;
 
end if; -- rising_edge

end process;
	
----------------------- 100 Mhz clocked logic -----------------------------

main : process(SysClk, CpldRst)

 begin 

-- asynchronous reset/preset
 if CpldRst = '0' then

-- Synchronous edge detectors for various strobes
	RDDL <= "00"; WRDL <= "00"; 

-- Trigger and spill generator logic
	FreqReg <= X"00000237"; PhaseAcc <= (others => '0');
	--Buff_Rst <= '0'; 
	Seq_Rst <= '0'; 
	Beam_On <= '0'; TrigEn <= '1'; 
	TstPlsEn <= '0';  TstPlsEnReq <= '0'; SS_FR <= '0';  TstTrigEn <= '0';
	IntTrig <= '0'; TrigType <= X"0"; 
	SpillWidth <= X"02"; Spill_Req <= '0'; TstTrigCE <= '0';
	EventWdCnt <= (others => '0'); InterSpill <= X"04"; BmOnTrigReq <= '0';
	PhaseAccD <= '0';
	UpTimeStage <= (others => '0'); UpTimeCount <= (others => '0');
	Counter1us <= X"00"; Counter1ms <= (others => '0'); MarkerReq <= '0';
	SuperCycleCount <= (others => '0'); SpillWidthCount <= (others => '0');
	InterSpillCount <= (others => '0'); MarkerSyncEn <= '0';
	uBinHeader <= '1'; uBwrt <= '1'; uBdebug <= '0'; uBdebug2 <= '0';
	TxEnReq <= '0'; DRFreq <= (others => '0'); -- Delivery ring DDS
	Int_uBunch <= "00"; -- Rising edge of DDS terminal count
	DRCount <= (others => '0'); -- Delivery ring bunch counter
	Counter1s <= (others => '0');	TestCount <= (others => '0'); 
	Counter100us <= (others => '0');	SpillCount <= (others => '0'); 
	LEDRst <= '1'; LEDSDat <= "000"; LEDSClk <= "000"; LEDLd <= "000000";
	uBunchLED <= '0'; uBunchLEDCnt <= (others => '0'); IntTmgEn <= '0';
   HrtBtBrstCntReg <= (X"001000"); HrtBtBrstCounter <= (others => '0');
	ExtuBunchOffset <= (others => '0');
	ExtuBunchCount <= (others => '0'); IntuBunchCount <= (others => '0'); 
	HrtBtBuff_rd_en <= '0'; HrtBtRdCnt <= X"0"; HrtBtTxReq <= '0';
	HrtBtFMReq <= '0'; CMDwr_en <= '0'; CMDrd_en <= '0'; 
	TmgCntEn <= '0'; ClkDiv <= "000"; CMDBitCount <= (others => '0'); 
	LEDShiftReg <= (others => '0');	LED_Shift <= Idle;
	DReqBuff_uCRd <= '0'; LinkBusy <= '0'; HrtBtTxInh <= '0';
	DCSPktBuff_uCRd <= '0'; MarkerDelay <= (others => '0'); 
	Clk80MHzAlign <= '0';
	LoopbackMode <= (others => '0');
	DCSBuff_wr_en <= '0'; DCSBuff_In <= (others => '0');
	DCS_Header <= X"8040";
	DCS_Status <= X"0040"; -- cnt [:7], status[6:5], op[4:0] => cnt = 1
	DCS_EvCnt  <= X"0010"; -- 8 words, 16 bytes
	
-- Pll Chip Shifter signals
	PLLBuffwr_en <= '0'; PLLBuffrd_en <= '0'; PllPDn <= '1';
	PllStage <= X"00"; PllShiftReg <= (others => '0'); 
	PllBitCount <= (others => '0'); Pll_Shift <= Idle;
	PllSClk <= '0'; PllSDat <= '0'; PllLd <= '0';
	
-- TDAQ Receive link signals
	RxSeqNo(0) <= "000";  WrtCount(0) <= "000";
	TDisA <= '0'; TDisB <= '0';

	PunchBits <= X"0"; FormHold <= '0'; ExtTmg <= '0';
	IDReg <= X"1";

	DReqBrstCntReg <= X"0001"; DReqBrstCounter <= (others => '0');
	Dreq_Tx_Req <= '0'; Dreq_Tx_ReqD <= '0';
	DReqPrescale <= (others => '0'); PreScaleReg <= '0' & X"63";

	FEBID_wea <= "0"; 
	FEBID_addra <= (others => '0'); FEBID_addrb <= (others => '0');
	
	HeartBtCnt <= (others => '0');
	
 elsif rising_edge (SysClk) then 

-- Synchronous edge detectors for read and write strobes
RDDL(0) <= not uCRD and not CpldCS;
RDDL(1) <= RDDL(0);

WRDL(0) <= not uCWR and not CpldCS;
WRDL(1) <= WRDL(0);

LinkBusy <= LinkFIFOEmpty(0) and LinkFIFOEmpty(1) and LinkFIFOEmpty(2);

-- Select between LEMO and LVDS inputs for the triggers 
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TrigCtrlAddr 
then TstPlsEn <= uCD(0);
	  TrgSrc <= uCD(1);
else TstPlsEn <= TstPlsEn;
	  TrgSrc <= TrgSrc;
end if;

-- Internal beam on trigger generator
   if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = FreqRegAdHi 
	then FreqReg <= uCD & FreqReg(15 downto 0);
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = FreqRegAdLo 
	then FreqReg <= FreqReg(31 downto 16) & uCD;
 else FreqReg <= FreqReg;
 end if;

if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr
then FormHold <= uCD(2);
	  ExtTmg <= uCD(4);
	  MarkerSyncEn <= uCD(5);
-- Choose between internal and TDAQ supplied timing
	  TrigTx_Sel <= uCD(6);
	  HrtBtTxInh <= uCD(10);
else FormHold <= FormHold;
	  ExtTmg <= ExtTmg;
	  TrigTx_Sel <= TrigTx_Sel;
	  MarkerSyncEn <= MarkerSyncEn;
	  HrtBtTxInh <= HrtBtTxInh;
end if;

if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = FormatRegAddr
then uBinHeader <= uCD(0);
     uBwrt      <= uCD(1);
	  uBdebug    <= uCD(2);
	  uBdebug2   <= uCD(3);
     -- more format settings
else uBinHeader <= uBinHeader;
     uBwrt      <= uBwrt;
	  uBdebug    <= uBdebug;
	  uBdebug2   <= uBdebug2;
end if;

-- Enable the transmitting of heartbeats
if IntTmgEn = '0' 
	and WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr and uCD(0) = '1'
  then IntTmgEn <= '1';
-- If the finite length is enabled, stop after the count has expired
 elsif IntTmgEn = '1' 
   and ((WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr and uCD(0) = '0')
    or  (HrtBtBrstCounter = 1 and Int_uBunch = 1 and ((Beam_On = '1' and DRCount = 7) or DRCount = 143)))
  then IntTmgEn <= '0';
 else IntTmgEn <= IntTmgEn;
 end if;
 
-- Finite heartbeat transmit sequence length enable bit
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and TmgCntEn = '0' and uCD(0) = '1' and uCD(1) = '1'
  then TmgCntEn <= '1';
-- If the finite length is enabled, stop after the count has expired
 elsif TmgCntEn = '1' and ((WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and uCD(1) = '0')
    or (HrtBtBrstCounter = 1 and Int_uBunch = 1 
	 and ((Beam_On = '1' and DRCount = 7) or DRCount = 1)))
  then TmgCntEn <= '0';
 else TmgCntEn <= TmgCntEn;
 end if;

-- Finite timing transmission burst down counter;
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and TmgCntEn = '0' and uCD(0) = '1' and uCD(1) = '1'
  then HrtBtBrstCounter <= HrtBtBrstCntReg;
 elsif HrtBtBrstCounter /= 0 and Int_uBunch = 1 and ((Beam_On = '1' and DRCount = 7) or DRCount = 143)
  then HrtBtBrstCounter <= HrtBtBrstCounter - 1;
 else HrtBtBrstCounter <= HrtBtBrstCounter;
 end if;

-- Enable the transmitting of trigger request packet on GTPTx(1)
if TstTrigEn = '0' and WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and uCD(8) = '1'
  then TstTrigEn <= '1';
-- If the finite length is enabled, stop after the count has expired
 elsif TstTrigEn = '1' and ((WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and uCD(8) = '0')
  or (TstTrigCE = '1' and DReqBrstCounter = 1 and Dreq_Tx_Req = '1' and Dreq_Tx_ReqD = '0'))
  then TstTrigEn <= '0';
 else TstTrigEn <= TstTrigEn;
 end if;

-- Finite trigger burst length enable bit
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and TstTrigCE = '0' and uCD(8) = '1' and uCD(9) = '1'
  then TstTrigCE <= '1';
-- If the finite length is enabled, stop after the count has expired
 elsif TstTrigCE = '1' and ((WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and uCD(9) = '0')
    or (TstTrigCE = '1' and DReqBrstCounter = 1 and Dreq_Tx_Req = '1' and Dreq_Tx_ReqD = '0'))
  then TstTrigCE <= '0';
 else TstTrigCE <= TstTrigCE;
 end if;

-- Finite trigger burst down counter;
 if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and TstTrigEn = '0' and  uCD(8) = '1'
  then DReqBrstCounter <= DReqBrstCntReg;
 elsif DReqBrstCounter /= 0 and TstTrigEn = '1' and TstTrigCE = '1' 
								and Dreq_Tx_Req = '1' and Dreq_Tx_ReqD = '0'
  then DReqBrstCounter <= DReqBrstCounter - 1;
  else DReqBrstCounter <= DReqBrstCounter;
  end if;

-- Register used to prescale data requests during beam on
 if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = PreScaleRegAd then 
     PreScaleReg <= uCD(8 downto 0);
  else 
	  PreScaleReg <= PreScaleReg;
  end if;

-- Counter used to prescale data requests during beam on
	if Beam_On = '0'  
	 or (TstTrigEn = '1' and IntTmgEn = '1' and Int_uBunch = 1 
	and Beam_On = '1' and DRCount = 7 and BmOnTrigReq = '1' and DReqPrescale = PreScaleReg) then 
	  DReqPrescale <= (others => '0'); 
	elsif TstTrigEn = '1' and IntTmgEn = '1' and Int_uBunch = 1 
	and Beam_On = '1' and DRCount = 7 and BmOnTrigReq = '1' and DReqPrescale /= PreScaleReg then 
	  DReqPrescale <= DReqPrescale + 1;
	else
	  DReqPrescale <= DReqPrescale;
	end if;

-- Send a heart request to transmit a data request packet.
if TstTrigEn = '1' and IntTmgEn = '1' and Int_uBunch = 1 
	and ((Beam_On = '1' and DRCount = 7 and BmOnTrigReq = '1' and DReqPrescale = PreScaleReg) 
	or DRCount = 143)
	then Dreq_Tx_Req <= '1';
elsif DReq_Tx_Ack = '1'
	then Dreq_Tx_Req <= '0';
end if;
Dreq_Tx_ReqD <= Dreq_Tx_Req;

if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = IDregAddr 
then IDReg <= uCD(3 downto 0);
else IDReg <= IDReg;
end if;

-- Set delay for received marker
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = MarkerDelayAd
	then MarkerDelay <= uCD(7 downto 0);
	else MarkerDelay <= MarkerDelay;
end if;

-- Enable/Disable 80MHz alignment
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = Clk80MHzAdd
	then Clk80MHzAlign <= uCD(0);
	else Clk80MHzAlign <= Clk80MHzAlign;
end if;

-- Set Loopback mode
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = LoopbackAdd
	then LoopbackMode(2 downto 0) <= uCD(2 downto 0);
	else LoopbackMode <= LoopbackMode;
end if;

--	Read of the trigger request FIFO
	if RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = TRigReqBuffAd 
	then DReqBuff_uCRd <= '1';
	else DReqBuff_uCRd <= '0';
	end if;

--	Read of the dcs request FIFO
	if RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = DCSPktBuffAd 
	then DCSPktBuff_uCRd <= '1';
	else DCSPktBuff_uCRd <= '0';
	end if;
	
-- write to the DCS answer FIFO
	if WRDL = 1 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = DCSBuffAd 
	then 
	    DCSBuff_wr_en <= '1';
		 DCSBuff_In <= uCD(15 downto 0);
	else 
	    DCSBuff_wr_en <= '0';
		 DCSBuff_In <= DCSBuff_In;
	end if;

-- modify DCS headers (for debugging)
	if WRDL = 1 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = DCSHeaderAd 
	then 
	    DCS_Header <= uCD(15 downto 0);
	else 
	    DCS_Header <= DCS_Header;
	end if;
	
	if WRDL = 1 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = DCSEvCntAd 
	then 
	    DCS_EvCnt <= uCD(15 downto 0);
	else 
	    DCS_EvCnt <= DCS_EvCnt;
	end if;
	
	if WRDL = 1 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = DCSStatusAd 
	then 
	    DCS_Status <= uCD(15 downto 0);
	else 
	    DCS_Status <= DCS_EvCnt;
	end if;

-- 1us time base
if Counter1us /= Count1us then Counter1us <= Counter1us + 1;
else Counter1us <= X"00";
end if;

-- 100us time base
if Counter100us /= Count100us then Counter100us <= Counter100us + 1;
else Counter100us <= (others => '0');
end if;

-- 1ms time base
if Counter1ms = Count1ms then Counter1ms <= (others => '0');
else Counter1ms <= Counter1ms + 1;
end if;

-- 1 second time base
if	Counter1s /= Count1s then Counter1s <= Counter1s + 1;
else Counter1s <= (others => '0');
end if;

-- 1.4 second super cycle count in 100 us steps (14000)
if IntTmgEn = '1' and Counter100us = Count100us and SuperCycleCount /= SuperCycleLength 
 then SuperCycleCount <= SuperCycleCount + 1;
elsif (Counter100us = Count100us and SuperCycleCount = SuperCycleLength) or IntTmgEn = '0'
 then SuperCycleCount <= (others => '0');
else SuperCycleCount <= SuperCycleCount;
end if;

-- 4.72 MHz generator
if IntTmgEn = '1' then DRFreq <= DRFreq + X"0C154C98";
else DRFreq <= (others => '0');
end if;

-- Edge detector for the DDS MSB
if DRFreq(31) = '1' then Int_uBunch(0) <= '1';
else Int_uBunch(0) <= '0';
end if;
Int_uBunch(1) <= Int_uBunch(0);

-- For now define the on spill to be 8 4.7MHz ticks and the off spill 144 ticks
-- "DR" for delivery ring 
if IntTmgEn = '1' and 
   Int_uBunch = 1 and not((Beam_On = '1' and DRCount = 7) or DRCount = 143)
then DRCount <= DRCount + 1;
elsif Int_uBunch = 1 and ((Beam_On = '1' and DRCount = 7) or DRCount = 143)
then DRCount <= (others => '0');
else DRCount <= DRCount;
end if;

-- If timing is internal, increment the microbunch number
if IntTmgEn = '1' and 
	Int_uBunch = 1 and ((Beam_On = '1' and DRCount = 7) or DRCount = 143)
then IntuBunchCount <= IntuBunchCount + 1;
elsif WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = TrigCtrlAddr	and uCD(2) = '1'
then IntuBunchCount <= (others => '0');
else IntuBunchCount <= IntuBunchCount;
end if;


-- Send a heartbeat transmit request to GTPTx(1) 
if IntTmgEn = '1' and Int_uBunch = 1 and ((Beam_On = '1' and DRCount = 7) or DRCount = 143)
 then HrtBtTxReq <= '1'; 
	elsif HrtBtTxAck = '1' then HrtBtTxReq <= '0';
 else HrtBtTxReq <= HrtBtTxReq;
end if;

-- If there are at least eight words in the heartbeat receive FIFO
-- start reading the packet data from the buffer
	if HrtBtBuffRdCnt >= 9 and HrtBtRdCnt = 0 and FMTxBsy = '0' then 
		HrtBtRdCnt <= X"9";
		HeartBtCnt <= HeartBtCnt + 1;
	elsif HrtBtRdCnt /= 0 then 
		HrtBtRdCnt <= HrtBtRdCnt - 1;
		--if GTPRxRst = '0' then
		    HeartBtCnt <= HeartBtCnt;
	   --else
		--    HeartBtCnt <= (others => '0');
		--end if;
	else HrtBtRdCnt <= HrtBtRdCnt; HeartBtCnt <= HeartBtCnt;
	end if;
	
--Debug(5 downto 3) <= HrtBtBuffRdCnt(3 downto 1);

	if (HrtBtRdCnt /= 0 and HrtBtTxInh = '0') 
	or (HrtBtTxInh = '1' and RDDL = 2 and AddrReg(11 downto 10) = GA 
		 and AddrReg(9 downto 0) = HrtBtFIFORdAd) then	
	  HrtBtBuff_rd_en <= '1'; Debug(6) <= '1';
	else 
	  HrtBtBuff_rd_en <= '0'; Debug(6) <= '0';
	end if;

-- Update the microbunch count from the received heartbeat.
	 Case HrtBtRdCnt is
		when X"6" => ExtuBunchCount(15 downto 0) <= HrtBtBuff_Out;
		when X"5" => ExtuBunchCount(31 downto 16) <= HrtBtBuff_Out;
		when X"4" => ExtuBunchCount(47 downto 32) <= HrtBtBuff_Out;
		When X"3" => ExtuBunchCount <= (ExtuBunchCount + (X"0000" & ExtuBunchOffset));
	   when others => ExtuBunchCount <= ExtuBunchCount;
	 end case;


-- Use signal as a handshake to cross over to the 80MHz clock domain
	if MarkerSyncEn = '0' and HrtBtRdCnt = 2 then 
		HrtBtFMReq <= '1';
	elsif HrtBtFMTxEn = '1' then
	   HrtBtFMReq <= '0';
	end if;

if HrtBtRdCnt = 5 then HrtBtMode <= HrtBtBuff_Out(7 downto 0);
else HrtBtMode <= HrtBtMode;
end if;

if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = DReqBrstCntAd 
 then DReqBrstCntReg <= uCD;
else DReqBrstCntReg <= DReqBrstCntReg;
end if;

-- uB offset 
 if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = HrtBtOffsetAd
 then ExtuBunchOffset <= uCD(15 downto 0);
 else ExtuBunchOffset <= ExtuBunchOffset;
 end if;

-- Counter used to send a burst of mirobunches.
 if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = HrtBtBrstCntAdHi
 then HrtBtBrstCntReg(23 downto 16) <= uCD(7 downto 0);
 else HrtBtBrstCntReg(23 downto 16) <= HrtBtBrstCntReg(23 downto 16);
 end if;

 if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = HrtBtBrstCntAdLo
 then HrtBtBrstCntReg(15 downto 0) <= uCD(15 downto 0);
 else HrtBtBrstCntReg(15 downto 0) <= HrtBtBrstCntReg(15 downto 0);
 end if;

-- Overall gate to denote the 380 ms spill region of the super cycle
if Spill_Req = '0' and SuperCycleCount = SpillBegin and Counter100us = Count100us 
then Spill_Req <= '1'; 
elsif (Spill_Req = '1' and Counter100us = Count100us and SuperCycleCount = SpillEnd)
	   or IntTmgEn = '0'
then Spill_Req <= '0'; 
end if;

-- A flag to indicate the individual 53 ms spills
if Counter100us = Count100us and Beam_On = '0' 
 and ((Spill_Req = '0' and SuperCycleCount = SpillBegin)
	or (Spill_Req = '1' and InterSpillCount = InterSpillLength))
then Beam_On <= '1';
elsif (Beam_On = '1' and SpillWidthCount = SpillLength and Counter100us = Count100us)
	   or IntTmgEn = '0'
then Beam_On <= '0';
else Beam_On <= Beam_On;
end if;

-- Count 53.1 ms spill length
if Spill_Req = '1' and Beam_On = '1' and SpillWidthCount /= SpillLength
 and Counter100us = Count100us
then SpillWidthCount <= SpillWidthCount + 1;
elsif (Counter100us = Count100us and SpillWidthCount = SpillLength)
      or IntTmgEn = '0'
then SpillWidthCount <= (others => '0');
end if;

-- Count the 5 ms interspill length
if Spill_Req = '1' and Beam_On = '0' and InterSpillCount /= InterSpillLength 
	and Counter100us = Count100us
 then InterSpillCount <= InterSpillCount  + 1;
elsif IntTmgEn = '0' or Spill_Req = '0' or (InterSpillCount = InterSpillLength 
	and Counter100us = Count100us)
 then InterSpillCount <= (others => '0');
else InterSpillCount <= InterSpillCount;
end if;

if uBunchLED = '0' and Beam_On = '0' and Spill_Req = '1' 
	and InterSpillCount = 0 and Counter100us = Count100us
then uBunchLED <= '1';
elsif uBunchLED = '1' and uBunchLEDCnt = 1 then uBunchLED <= '0'; 
else uBunchLED <=  uBunchLED;
end if;

if uBunchLED = '0' and Beam_On = '0' and Spill_Req = '1' 
	and InterSpillCount = 0 and Counter100us = Count100us
then uBunchLEDCnt <= '1' & X"4";
elsif uBunchLEDCnt /= 0 and Counter1ms = Count1ms
then uBunchLEDCnt <= uBunchLEDCnt - 1;
else uBunchLEDCnt <= uBunchLEDCnt;
end if; 

-- Uptime in seconds since th last FPGA configure
if	Counter1s = Count1s then UpTimeCount <= UpTimeCount + 1;
else UpTimeCount <= UpTimeCount;
end if;

-- Register for staging uptime count.
if CpldCS = '1' then UpTimeStage <= UpTimeCount;
else UpTimeStage <= UpTimeStage;
end if;

-- Testcounter counter is writeable. For each read of the lower half, the entire
-- 32 bit counter increments
if    WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = TestCounterHiAd 
then TestCount <= (uCD & TestCount(15 downto 0));
elsif WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = TestCounterLoAd 
then TestCount <= (TestCount(31 downto 16) & uCD);
elsif RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = TestCounterLoAd 
then TestCount <= TestCount + 1;
else TestCount <= TestCount;
end if;

if WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPCSRAddr  
 then TDisA <= uCD(0); 
		TDisB <= uCD(8);
 else 
		TDisA <= TDisA;
		TDisB <= TDisB;
end if;

-- Serializer for front panel LEDs
ClkDiv <= ClkDiv + 1;

if    WRDL = 1 and  uCA(11 downto 10) = GA 
		and uCA(9 downto 0) >= LEDDatAddr(0) and  uCA(9 downto 0) <= LEDDatAddr(5)
then CMDwr_en <= '1';
else CMDwr_en <= '0';
end if;

-- Idle,Load,Shift,SendPClk,RdFIFO 
case LED_Shift is
	when Idle => 
		if CMD_Empty = '0' and ClkDiv = 7 then LED_Shift <= Load;
		elsif 
			WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = LEDRstAddr
			and uCD(0) = '1'  then LED_Shift <= WaitRst;
		else LED_Shift <= Idle;
		end if;
	when Load => 
		   if ClkDiv = 7 then LED_Shift <= Shift;
			else LED_Shift <= Load;
			end if;
	when Shift =>
		if CMDBitCount = 0 and ClkDiv = 7 then LED_Shift <= RdFIFO;
		else LED_Shift <= Shift;
		end if;
	when RdFIFO => LED_Shift <= SendPClk;
	when WaitRst =>
	    if ClkDiv = 7 then LED_Shift <= SendRst;
		 else LED_Shift <= WaitRst;
		 end if;
	when SendRst =>
	    if ClkDiv = 7 then LED_Shift <= WaitPClk;
		 else LED_Shift <= SendRst;
		 end if;
   when WaitPClk =>
	    if ClkDiv = 7 then LED_Shift <= SendPClk;
		 else LED_Shift <= WaitPClk;
		 end if;
	when SendPClk => 
		if ClkDiv = 7 then LED_Shift <= Idle;
		else LED_Shift <= SendPClk;
		end if;
end case;

if LED_Shift = SendRst then LEDRst <= '0';
else LEDRst <= '1';
end if;

if LED_Shift = Load and ClkDiv = 7 then CMDBitCount <= X"F";
elsif LED_Shift = Shift and ClkDiv = 7 then CMDBitCount <= CMDBitCount - 1;
else CMDBitCount <= CMDBitCount;
end if;

if LED_Shift = Load and ClkDiv = 7 then LEDShiftReg <= CMD_Out(15 downto 0);
elsif LED_Shift = Shift and ClkDiv = 7 then LEDShiftReg <= LEDShiftReg(14 downto 0) & '0';
else LEDShiftReg <= LEDShiftReg;
end if;

Case CMD_Out(18 downto 17) is
	when "00" => LEDSDat <= "00" & LEDShiftReg(15);
					 LEDSClk <= "00" & ClkDiv(2);
	when "01" => LEDSDat <= '0' & LEDShiftReg(15) & '0';
					 LEDSClk <= '0' & ClkDiv(2) & '0';
	when "10" => LEDSDat <= LEDShiftReg(15) & "00";
					 LEDSClk <= ClkDiv(2) & "00";
	when others => LEDSDat <= "000";
					   LEDSClk <= "000";
end case;

if LED_Shift = Shift then 
	Case CMD_Out(18 downto 17) is
		when "00" => LEDSClk <= "00" & ClkDiv(2);
		when "01" => LEDSClk <= '0' & ClkDiv(2) & '0';
		when "10" => LEDSClk <= ClkDiv(2) & "00";
		when others => LEDSClk <= "000";
	end case;
  else LEDSClk <= "000";
end if;

if LED_Shift = SendPClk then
Case CMD_Out(18 downto 16) is
	when "000" => LEDLd <= "000001";
	when "001" => LEDLd <= "000010";
	when "010" => LEDLd <= "000100";
	when "011" => LEDLd <= "001000";
	when "100" => LEDLd <= "010000";
	when "101" => LEDLd <= "100000";
	when others => LEDLd <= "000000";
end case;
else LEDLd <= "000000";
end if;

if LED_Shift = RdFIFO then CMDrd_en <= '1';
else CMDrd_en <= '0';
end if;

-- Serializer for the PLL chip
-- Pll data is 24 bits. Stage the upper order eight bits in a register
if WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = PLLHiAddr
 then PllStage <= uCD(7 downto 0);
else PllStage <= PllStage;
end if;
-- Apply the Staging register contents and the uC data bus to the FIFO input
if WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = PLLLoAddr
 then PLLBuffwr_en <= '1';
else PLLBuffwr_en <= '0';
end if;

-- Idle,Load,Shift,WaitLd,SendLd
Case Pll_Shift is when 
	Idle => 
	  if PLLBuff_empty = '0' and ClkDiv = 7 
	  then Pll_Shift <= Load;
	 else Pll_Shift <= Idle;
	 end if;
	When Load =>
	  if ClkDiv = 7 then Pll_Shift <= Shift;
	  else Pll_Shift <= Load;
	  end if;
	When Shift =>
	 if PllBitCount = 0 and ClkDiv = 7 
		then Pll_Shift <= WaitLd;
	 else Pll_Shift <= Shift;
	end if;
	When WaitLd =>
		if ClkDiv = 7 then Pll_Shift <= SendLd;
		else Pll_Shift <= WaitLd;
		end if;
	When SendLd =>
	 if ClkDiv = 7 then Pll_Shift <= Idle;
	 else Pll_Shift <= SendLd;
	 end if;
end Case;

-- Pll Shifter bit counter
if Pll_Shift = Load and ClkDiv = 7 then PllBitCount <= '1' & X"7";
elsif Pll_Shift = Shift and ClkDiv = 7 then PllBitCount <= PllBitCount - 1;
else PllBitCount <= PllBitCount;
end if;

-- Pll Shiftter shifter register
if Pll_Shift = Load and ClkDiv = 7 then PllShiftReg <= PLLBuff_Out;
elsif Pll_Shift = Shift and ClkDiv = 7 then PllShiftReg <= PllShiftReg(22 downto 0) & '0';
else PllShiftReg <= PllShiftReg;
end if;

-- Read the PLL fifo at the end of the shift sequence
if Pll_Shift = Load and ClkDiv = 7 then PLLBuffrd_en <= '1'; 
else PLLBuffrd_en <= '0'; 
end if;

-- PLL SPI port serial clock
if Pll_Shift = Shift then PllSClk <= ClkDiv(2);
else PllSClk <= '0';
end if;

PllSDat <= PllShiftReg(23); 

-- Assert a load pulse after the shift sequence is done
if Pll_Shift = SendLd then PllLd <= '1'; 
else PllLd <= '0'; 
end if;

if WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = PLLPDnAddr 
 then PllPDn <= uCD(0);
else PllPDn <= PllPDn;
end if;

------------------------------- Trigger Logic ----------------------------

PhaseAcc <= PhaseAcc + FreqReg;
PhaseAccD <= PhaseAcc(31);

if TstTrigEn = '1'
	and PhaseAcc(31) = '1' and PhaseAccD = '0' then 
   BmOnTrigReq <= '1';
elsif HrtBtDone = '1' then 
	BmOnTrigReq <= '0';
else BmOnTrigReq <= BmOnTrigReq;
end if;

-- Trig out width counter
if GPOCount = 0 and MarkerReq = '1' then GPOCount <= "111";
elsif GPOCount /= 0 then GPOCount <= GPOCount - 1;
else GPOCount <= GPOCount;
end if;

-- Flag bit indicating a software trigger 
if WrDL = 1 and uCA(9 downto 0) = TrigCtrlAddr and uCD(0) = '1' then IntTrig <= '1';
 elsif GPIDL(0) = 1 or TstTrigEn = '1'
 then  IntTrig <= '0';
 else  IntTrig <= IntTrig;
end if;

--	Read of the trigger request trace buffer
	if (RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = DReqBuffTraceAd ) or 
	    (DReqBuffTrace_DatCnt >= "011" & X"F0" ) -- this should make this buffer to a trace buffer.
	then DReqBuffTrace_rd_en <= '1';
	else DReqBuffTrace_rd_en <= '0';
	end if;

-- read link FIFO trace
	if (RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkFIFOTraceAd ) or 
	    (LinkFIFOTraceRdCnt >= '0' & X"FF0" ) -- this should make this buffer to a trace buffer.
	then LinkFIFOTraceRdReq <= '1';
	else LinkFIFOTraceRdReq <= '0';
	end if;


end if; --rising edge

end process;

------------------- mux for reading back registers -------------------------

with uCA(9 downto 0) select

iCD <= X"0" & '0' & HrtBtTxInh & TstTrigCE & TstTrigEn & '0' & TrigTx_Sel 
		 & MarkerSyncEn & ExtTmg & '0' & FormHold & TmgCntEn & IntTmgEn when CSRRegAddr,
		   Rx_IsCtrl(1) & InvalidChar(1) & Rx_IsComma(1) & Reframe(1) & TDisB 
		 & Rx_IsCtrl(0) & InvalidChar(0) & Rx_IsComma(0) & Reframe(0) & TDisA when GTPCSRAddr,
		 X"00" & "00" & GTPRxBuff_Full & GTPRxBuff_Emtpy & "00" when GTPFIFOAddr,
		 X"00" & "000" & PLLStat & "000" & PllPDn when PLLPDnAddr,
		 DReqBuff_Out(15 downto 0) when TRigReqBuffAd,
		 X"0" & '0' & TrgPktRdCnt when TRigReqWdUsedAd,
		 X"000" & "00" & TrgSrc & TstPlsEn when TrigCtrlAddr,
		 X"00" & ActiveReg(23 downto 16) when ActvRegAddrHi,
		 ActiveReg(15 downto 0) when ActvRegAddrLo,
		 X"000" & IDReg when IDregAddr,
		 X"0" & "00" & Debug when DebugPinAd,
		 X"000" & '0' & FormStatReg when GTPSeqStatAd,
		 X"000" & '0' & Beam_On & '0' & '0' when SpillStatAddr,
		 UpTimeStage(31 downto 16) when UpTimeRegAddrHi,
		 UpTimeStage(15 downto 0) when UpTimeRegAddrLo,
		 TestCount(31 downto 16) when TestCounterHiAd,
		 TestCount(15 downto 0) when TestCounterLoAd,
		 X"0" & "000" & PreScaleReg when PreScaleRegAd,
		 LinkFIFOOut(0) when LinkRdAddr(0),
		 LinkFIFOOut(1) when LinkRdAddr(1),
		 LinkFIFOOut(2) when LinkRdAddr(2),
		 X"00" & '0' & LinkFIFOFull & '0' & LinkFIFOEmpty when LinkCSRAddr,
		 GTPRxBuff_Out(0) when GTPRdAddr0,
		 GTPRxBuff_Out(1) when GTPRdAddr1,
		 WdCountBuff_Out when EvWdCntBuffAd,
		 WdCountBuff_Full & WdCountBuff_Empty & '0' & WdCountBuff_DatCnt when WdCntBuffStatAd,
		 TxCRC(0) when CRCRdAddr(0),
		 TxCRC(1) when CRCRdAddr(1),
		 RxCRC(0) when CRCRdAddr(2),
		 RxCRC(1) when CRCRdAddr(3),
		 "00" & EvTxWdCnt when EvTxWdCntAd,
		 "000" & LinkFIFORdCnt(0) when LinkWdCnt0Ad,
		 "000" & LinkFIFORdCnt(1) when LinkWdCnt1Ad,
		 "000" & LinkFIFORdCnt(2) when LinkWdCnt2Ad,
		 X"000" & "00" & EventBuff_Full & EventBuff_empty when EvBuffStatAd,
		 '0' & GtpRxBuffStat(1) & '0' & GtpRxBuffCnt(1) 
	  & '0' & GtpRxBuffStat(0) & '0' & GtpRxBuffCnt(0) when ElasticStatAd,
	    DReqBrstCntReg when DReqBrstCntAd,
	    X"00" & HrtBtBrstCntReg(23 downto 16) when HrtBtBrstCntAdHi,
	    HrtBtBrstCntReg(15 downto 0) when HrtBtBrstCntAdLo,
		 IntuBunchCount(47 downto 32) when MicroBunchAdHi,
		 IntuBunchCount(31 downto 16) when MicroBunchAdMid,
		 IntuBunchCount(15 downto 0) when MicroBunchAdLo,
		 FreqReg(31 downto 16) when FreqRegAdHi,
		 FreqReg(15 downto 0) when FreqRegAdLo,
		 MarkerBits when MarkerBitsAd,
		 DReqBuff_Emtpy & "0000" & TrgPktRdCnt when DreqBuffStatAd,
		 HrtBtBuff_Emtpy & "0000" & HrtBtBuffRdCnt when HrtBtBuffStatAd,
		 HrtBtBuff_Out when HrtBtFIFORdAd,
		 X"00" & MarkerDelay when MarkerDelayAd,
		 CRCErrCnt & X"0" & LosCounter when LinkErrAd,
		 "000" & DCSPktRdCnt when DCSPktWdUsedAd,
		 DCSPktBuff_Out(15 downto 0) when DCSPktBuffAd,
		 ExtuBunchOffset when HrtBtOffsetAd,
		 DReq_Count(15 downto 0) when DReqCountLowAd,
		 DReq_Count(31 downto 16) when DReqCountHiAd,
		 GTPTxBuff_Out when GTPTxRdAddr,
		 DReqBuffTrace_Out when DReqBuffTraceAd,
		 LinkFIFOTraceOut when LinkFIFOTraceAd,
		 DCSBuff_wr_en & DCSBuff_Full & DCSBuff_Emtpy & DCSBuffRdCnt when DCSBuffCntAd,	
       DCSBuff_In when DCSBuffAd,
		 DCS_Header when DCSHeaderAd,
		 DCS_EvCnt when DCSEvCntAd,
		 DCS_Status when DCSStatusAd,
		 GTPRstFromCnt & "0" & GTPTstFromCntEn & GTPRstArm & X"0" &"0" & GTPRstCnter when GTPRstCntAd,
		 X"000" & uBdebug2 & uBdebug & uBwrt & uBinHeader when FormatRegAddr,
		 uBcheck(31 downto 16) when uBLowRegAddr,
		 uBcheck(15 downto  0) when uBHighRegAddr,
		 HeartBeatCnt & HeartBtCnt when HeartBeatCntAddr,
		 MarkerDelayedCnt & MarkerCnt when MarkerCntAddr,
		 MarkerCnt3 & MarkerCnt2 when MarkerCnt2Addr,
		 MarkerCnt4 & MarkerCnt5 when MarkerCnt3Addr,
		 LastWindow when LastWindowLengthAddr,
		 InjectionTs when InjectionLengthAddr,
		 Clk80MHzAlignCnt & X"0" & "000" & Clk80MHzAlign when Clk80MHzAdd,
		 X"000" & "0" & LoopbackMode when LoopbackAdd,
		 X"002A" when DebugVersionAd,
		 GIT_HASH(31 downto 16) when GitHashHiAddr,
		 GIT_HASH(15 downto 0)  when GitHashLoAddr,
		 X"0000" when others;

-- Select between the Orange Tree port and the rest of the registers
uCD <= iCD when uCRd = '0' and CpldCS = '0' and uCA(11 downto 10) = GA 
		 else iDQ when uCRd = '0' and EthCS = '0'  
		 else (others => 'Z');

end behavioural;

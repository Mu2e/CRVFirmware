-- Firmware for Mu2e FEB FPGA A

-- Sten Hansen Fermilab 10/15/2014

-- FPGA responsible for collecting data from 16 AFE 480 MHz serial ports and 
-- storing it in DDR RAM.

-- Microcontroller interface, test pulse generator, TCLK decoder

-- 11/15/14 microcontoller interface, DRAM MIG, DACs, Flash Gate, LED Flasher
-- 02/17/15 input FIFOs for AFE data, AFE setup and readback
-- 01/22/15 one wire readout
-- 03/23/15 Controller for copying AFE data to DDR
-- 04/05/15 Setting up link to FTBF PWC controller
-- 04/10/15 Internal trigger generator
-- 04/14/15 Setting up broadcast uC Writes
-- 04/20/15 Set up sequencers to automate one wire tempereature and ID # readback
-- 05/15/16 Added gate qualified self trigger 
-- 07/15/16 Added self trigger
-- 12/02/16 Fixed readout bug for internally generated triggers, cleaned up source file
-- 12/21/16 Tested clocking based in an input clock of 100MHz and DDR clocking of
-- 			160MHz. See p.39 of UG388
-- 12/30/16 Upgraded histogramming logic using 512x32 block rams
-- 10/15/17 Begin work on zero suppressed data readout
-- 04/04/18 Removed internal trigger generator, test beam spill related logic
-- 05/23/18 changed arithmetic to IEEE.numeric.std

----------------------------- Main Body of design -------------------------

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use IEEE.numeric_std.all;
Library UNISIM;
use UNISIM.vcomponents.all;

use work.Proj_defs.all;

entity FEB_Fpga_A is port(

-- 100 MHz VXO clock
	VXO_P,VXO_N,ClkB_P,ClkB_N,
-- microcontroller strobes
	CpldRst, CpldCS, uCRd, uCWr : in std_logic;
-- microcontroller data, address buses
	uCA : in AddrPins;
	uCD : inout std_logic_vector(15 downto 0);
-- Geographic address pins
	GA : in std_logic_vector(1 downto 0);
-- SDRAM pins
	SDCKE,LDM,UDM,RAS,CAS,SDWE : out std_logic;
	SDClk_P,SDClk_N : out  std_logic;
	SDD : inout std_logic_vector(15 downto 0);
	UDQS,LDQS,SDRzq : inout std_logic;
	SDA : out std_logic_vector(13 downto 0);
	BA : out std_logic_vector(1 downto 0);
-- AFE serial control lines
	AFEPDn : buffer std_logic_vector(1 downto 0);
	AFECS : buffer std_logic_vector(1 downto 0);
	AFESClk,AFESDI,AFERst : buffer std_logic;
	AFESDO : in std_logic;
-- AFE clock, framing lines
	AFEDCO_P,AFEDCO_N,AFEFR_P,AFEFR_N  : in std_logic_vector(1 downto 0);
-- AFE Data lines
	AFEDat0_P,AFEDat0_N,AFEDat1_P,AFEDat1_N : in std_logic_vector(7 downto 0);
-- AFE Input clocks
	AFEClk_P,AFEClk_N : out std_logic_vector(1 downto 0);
-- Analog Mux address lines
	MuxEn : buffer std_logic_vector(3 downto 0);
	Muxad : buffer std_logic_vector(1 downto 0);
-- LED pulser/Flash Gate
	Pulse : out std_logic;
-- LED/Flash Gate select line
	PulseSel : buffer std_logic;
-- Serial DAC control lines
	DACCS : buffer std_logic_vector(2 downto 0);
	DACClk,DACDat,DACLd : buffer std_logic;
-- Temperature sensor lines
	Temp : inout std_logic_vector(3 downto 0);
-- Chip dependent I/O functions
	A7,LVDSTX : buffer std_logic;
	GPI0_N,GPI0_P,GPI1 : in std_logic;
-- Debug port
	Debug : buffer std_logic_vector(10 downto 1)
);

end FEB_Fpga_A;

architecture behavioural of FEB_Fpga_A is

---------------------- Signal declarations -----------------------

-- Define Arrays in terms of their size

-- Use this to deal with (0 downto 0)
Type Array_2x1 is Array(0 to 1) of std_logic_vector (0 downto 0);

Type Array_2x2 is Array(0 to 1) of std_logic_vector(1 downto 0);
Type Array_2x3 is Array(0 to 1) of std_logic_vector (2 downto 0);
Type Array_2x4 is Array(0 to 1) of std_logic_vector(3 downto 0);
Type Array_2x5 is Array(0 to 1) of std_logic_vector(4 downto 0);
Type Array_2x6 is Array(0 to 1) of std_logic_vector(5 downto 0);
Type Array_2x8 is Array(0 to 1) of std_logic_vector(7 downto 0);
Type Array_2x9 is Array(0 to 1) of std_logic_vector(8 downto 0);
Type Array_2x10 is Array(0 to 1) of std_logic_vector(9 downto 0); 
Type Array_2x11 is Array(0 to 1) of std_logic_vector(10 downto 0); 
Type Array_2x12 is Array(0 to 1) of std_logic_vector(11 downto 0);
Type Array_2x16 is Array(0 to 1) of std_logic_vector(15 downto 0); 
Type Array_2x32 is Array(0 to 1) of std_logic_vector(31 downto 0); 

Type Array_8x2 is Array(0 to 7) of std_logic_vector(1 downto 0); 
Type Array_8x4 is Array(0 to 7) of std_logic_vector(3 downto 0);
Type Array_8x8 is Array(0 to 7) of std_logic_vector(7 downto 0);
Type Array_8x10 is Array(0 to 7) of std_logic_vector(9 downto 0);
Type Array_8x12 is Array(0 to 7) of std_logic_vector(11 downto 0);
Type Array_8x16 is Array(0 to 7) of std_logic_vector(15 downto 0);

Type Array_2x8x2 is Array(0 to 1) of Array_8x2;
Type Array_2x8x4 is Array(0 to 1) of Array_8x4;
Type Array_2x8x8 is Array(0 to 1) of Array_8x8;
Type Array_2x8x10 is Array(0 to 1) of Array_8x10;
Type Array_2x8x12 is Array(0 to 1) of Array_8x12;
Type Array_2x8x16 is Array(0 to 1) of Array_8x16;

-- Signed arrays
Type Arrays_3x12 is Array(0 to 2) of signed(11 downto 0);
Type Arrays_8x12 is Array(0 to 7) of signed(11 downto 0);
Type Arrays_8x16 is Array(0 to 7) of signed(15 downto 0);
Type Arrays_2x3x12 is Array(0 to 1) of Arrays_3x12;
Type Arrays_8x2x12 is Array(0 to 1) of Arrays_8x12;
Type Arrays_8x2x16 is Array(0 to 1) of Arrays_8x16;

-- Chip dependent drive
signal RefIn,GPI0,GPO,GPOSel : std_logic;
-- Counter that determines the trig out pulse width
signal GPOCount : std_logic_vector(2 downto 0);
-- Clock and reset signals
signal SysClk,Clk100MHz,Buff_Rst,ResetHi,AsyncRst, 
		 Seq_Rst, AFEClk : std_logic;

-- Synchronous edge detectors of uC read and write strobes
signal IdleDL,MaskSelect,GateReq : std_logic_vector (1 downto 0);
signal RDDL,uRDDL,WRDL,uWRDL : std_logic_vector(1 downto 0);

-- uC data bus
signal iCD,CDStage,ICAP_O : std_logic_vector(15 downto 0);
signal AddrReg,uAddrReg : std_logic_vector(11 downto 0);
signal TrigType : std_logic_vector(11 downto 0);
signal ControllerNo,PortNo : std_logic_vector(4 downto 0);

-- Timing interval counters
signal Counter1us  : std_logic_vector (7 downto 0);
signal Counter10us : std_logic_vector (10 downto 0);
signal Counter1ms  : std_logic_vector (17 downto 0); 
signal Count100ms  : std_logic_vector (6 downto 0);
signal Counter1s   : std_logic_vector (27 downto 0);
signal GateCounter, TurnOnTime,TurnOffTime,LEDTime : std_logic_vector (8 downto 0);

-- Signals for aligning RxOutClks
Type AlignSeq_Type is (Idle,SendPDn,SendPEn,SendRst,WaitFR,CheckFR,RstCntr);
signal AlignSeq : AlignSeq_Type;
signal uSecCounter : std_logic_vector (8 downto 0);
signal AlignReq, AFESel : std_logic_vector (1 downto 0);
signal SlipCntReg : Array_2x3;
Type OD_WriteState is (Idle, SetAFESel0, WrtAFE0, SetAFESel1, WrtAFE1); 
signal OD_Write : OD_WriteState;

-- trigger logic signals
signal Strt_req, Seq_Busy,FlashEn,FlashGate, TrgSrc,
		 TrigReq,TrigReqD,SlfTrgEn,TmgSrcSel : std_logic; 

signal StatReg : std_logic_vector (3 downto 0);

signal LEDSrc : std_logic;
-- Make a test counter that increments with each read
signal TestCount : std_logic_vector (31 downto 0);
-- Uptime counter to check for un-anticipated resets
signal UpTimeCount : std_logic_vector (31 downto 0);
signal UpTimeStage : std_logic_vector (31 downto 0);
-- Number of data words per spill
-- 80 MHz counter ADC sample #
signal PipelineSet,WidthReg : std_logic_vector (7 downto 0);
signal BeamOnLength,BeamOffLength : std_logic_vector (11 downto 0);

-- Event word cout
signal EventWdCnt,NxtWdCount : std_logic_vector (15 downto 0);

signal GPIDL,iWrtDL : Array_2x2;
signal WrtPtrRst,ADCSmplGate,EvBuffStatFIFO_Full,
		 EvBuffStatFIFO_Empty,EvenWrtDone : std_logic_vector (1 downto 0);
signal RdDone,EvBuffStat_rden : std_logic;
signal EvBuffStatFIFO_Out,EvBuffStatFIFO_In : Array_2x1;
signal MaskReg,EvOvf : Array_2x8;
signal HistWidth : Array_2x8;
signal GateWidth,uBunchOffset : Array_2x12;
signal DoneDly : Array_2x3;
signal ADCSmplCntr : Array_2x8x4;
signal ADCSmplCntReg : std_logic_vector (3 downto 0);

-- Self trigger signals
signal Ped_Reg,Diff_Reg,IntTrgThresh : Arrays_8x2x12;
signal Ped_Avg : Arrays_8x2x16;
signal Avg_En : Array_2x8;
signal Pad_Avg_Count : Array_2x5;
signal Avg_Req : std_logic_vector(1 downto 0);
signal SlfTrgEdge : Array_2x8x2;
signal uBunchBuffOut,DDRAddrOut : std_logic_vector(31 downto 0);
signal uBunch : std_logic_vector(31 downto 0);
signal uBunchGuard : std_logic;
signal uBunchWrt,uBunchRd,uBunchBuffEmpty,uBunchBuffFull,
		 DDRAddrRd,DDRAddrFull,DDRAddrEmpty : std_logic;

Type Input_Seq_FSM is (Idle,Increment,WrtChanNo,WrtTimeStamp,WrtHits,
							  WrtHitWdCnt,LdNxtWrtAd);
Type In_Seq_Array is Array(0 to 7) of Input_Seq_FSM;
Type In_Seq_Array_2x8 is Array(0 to 1) of In_Seq_Array;
signal Input_Seqs : In_Seq_Array_2x8;
signal HitCntr : Array_2x8x8;
signal In_Seq_Stat : Array_2x8;

-- Octal DAC buffer FIFO control signals
signal ODFifoRdReq,ODFifoWrReq,ODFifoEmpty,Dev_Sel : std_logic;
-- Octal DAC shadow RAM signals
signal ShadowWrt : std_logic_vector (0 downto 0);
signal ShadowOut : std_logic_vector (15 downto 0);
signal SClkDL : std_logic_vector (2 downto 0); 
signal ODFifoData,ODFifoOut : std_logic_vector (27 downto 0);
-- Dac control signals
signal BiasActual,BiasTarget : Array_2x12;
signal RampGate : std_logic_vector (1 downto 0);
signal ClkDiv : std_logic_vector (2 downto 0);
signal BitCount : std_logic_vector (4 downto 0);
signal DACShift : std_logic_vector (23 downto 0);
signal AFERdReg : std_logic_vector (15 downto 0);
Type  Serializer_FSM is (Idle,Shift,ClearSync,SetLoad);
Signal Octal_Shift : Serializer_FSM;

-- Serdes divided clock output
signal RxOutClk : std_logic_vector (1 downto 0);
-- Input buffer memory control signals
signal Buff_In,Buff_Out : Array_2x8x16;
signal Buff_Rd_Ptr,NextEvAddr,Buff_Wrt_Ptr,WrtCrrntWdCntAd,WrtNxtWdCntAd,InWdCnt : Array_2x8x10;
signal NoHIts : Array_2x8;
Signal Ins,Outs : Array_2x8x12;
-- Deserialize frame along with the 8 data lines. Use the deserialized 
-- frame signal as an input to the bitslip state machine
signal AFE_Wrt,AFE_rd : Array_2x8;
signal SerDesInP,SerDesInN : Array_2x9;
signal FrDat : Array_2x6;
signal FrDat2 : Array_2x6;
signal SlipReq,SlipEn,SerdesRst,FR_OK,FRStat : std_logic_vector(1 downto 0);
signal Slippause : Array_2x4;
signal RstReqDL : Array_2x2;
-- Input pipeline delay signals
signal DPWrtAd,DPRdAd : Array_2x8;
signal PipeWrt, RstReq : std_logic_vector(1 downto 0);

-- MIG LPDDR controller signals 
signal AuxClk : std_logic;
signal SDRdDat,SDWrtDat : std_logic_vector(31 downto 0);
signal DDRRd_Mux : std_logic_vector(15 downto 0);
signal SDWrtAd,SDWrtAdStage,SDRdAD,SDRdPtr : std_logic_vector(29 downto 0);
signal SDwr_en,SDrd_en,SDCalDn,WrtCmdEn,SDRdCmdEn,SD_RstO,DDR_Reset,
		 SDwr_full,SDwr_empty,SDwr_error,SDrd_full,SDrd_empty,SDrd_overflow,
		 SDrd_error,SDwr_underrun,RdHi_LoSel,FifoRdD,WrtHi_LoSel,Even_Odd : std_logic;
signal ResetCount : std_logic_vector(3 downto 0);
constant RdBrstSiz : std_logic_vector(5 downto 0) := "000111";
constant WrtBrstSiz : std_logic_vector(5 downto 0) := "000111";
signal SDwr_count,DDR_Rd_Cnt : std_logic_vector(6 downto 0);
signal SDWrtCmd,SDRdCmd : std_logic_vector(2 downto 0);
signal SDcmd_empty,SDcmd_full : std_logic_vector(1 downto 0);

-- Signal names used by SERDES see: XAP1024
signal rxioclkp : std_logic_vector(1 downto 0);
signal rxioclkn : std_logic_vector(1 downto 0);
signal rx_serdesstrobe : std_logic_vector(1 downto 0);

-- Signals for DDR write sequencer
signal SampleCount,BuffRdCount : std_logic_vector(8 downto 0);

Type Event_Builder_FSM is (Idle,Check_Ovf,Add_Wd_Count,Incr_Chan0,
									Check_Mask0,WdCountWrt,WrtuBunchHi,WrtuBunchLo,
									Incr_Chan1,Check_Mask1,Wait1,Wait2,WrtData);
signal Event_Builder : Event_Builder_FSM;
signal Read_Seq_Stat : std_logic_vector(3 downto 0);
Type Write_Seq_FSM is (Idle,ChkWrtBuff,SndCmd,WtCmdMtpy,AddrRd,
							  SetWrtPtr,WrtDDR,Wait1,WritePad,WaitWrtDn);
signal DDR_Write_Seq : Write_Seq_FSM;
signal DDRWrtSeqStat : std_logic_vector(2 downto 0);
signal EvBuffWrt,EvBuffRd,EvBuffEmpty,EvBuffFull,DRAMRdBuffWrt,PageRdStat,
		 PageRdReq,DRAMRdBuffRd,DRAMRdBuffFull,DRAMRdBuffEmpty, rstDRAMBuffHi : std_logic;
signal EvBuffDat,EvBufffOut,DRAMRdBuffDat,DRAMRdBuffOut : std_logic_vector(15 downto 0);
signal DRAMRdBuffWdsUsed,EvBuffWdsUsed,PageWdCount : std_logic_vector(10 downto 0);
signal DDRWrtCount : std_logic_vector(10 downto 0);

signal AFE_Num  : Integer range 0 to 2; 
signal Chan_Num : integer range 0 to 7;

Type Read_Seq_FSM is (Idle,CheckEmpty,FirstCmd,CheckRdBuff0,
						 RdWdCount,CheckRdBuff1,RdDataHi,RdDataLo);
signal DDR_Read_Seq : Read_Seq_FSM;

-- Signals used by the current sense multiplexer
signal MuxSelReg : std_logic_vector(2 downto 0);
signal MuxadReg : std_logic_vector(1 downto 0);

-- Signal used by LVDS serial links
signal TxPDat : std_logic_vector(15 downto 0);
signal Rx1Dat,Rx1DatReg : std_logic_vector(23 downto 0);
signal RxOut : RxOutRec;
signal RxIn : RxInRec;
-- Signals used by the phase detector
signal TxEn,FMTxBuff_wreq,FMTxBuff_empty,FMTxBuff_full,PhDtct,SqWav,BeamOn : std_logic;
signal FBDiv : std_logic_vector(2 downto 0);
signal TxOuts : TxOutRec;

-- Histogrammer signals
signal HistGateCnt0,HistGateCnt1 : std_logic_vector(15 downto 0);
signal HistInterval : std_logic_vector(15 downto 0);
signal HistChan : std_logic_vector(2 downto 0);
signal HistMode : std_logic;
signal Triplet : Arrays_2x3x12;
signal Peak : Array_2x8;
signal HistEn,HistInit,HistEnReq : std_logic_vector(1 downto 0);
signal HistTimer,HistEnDl : Array_2x2;
Type Wen_Array is Array(0 to 1) of std_logic_vector(0 downto 0);
signal Hist_wena,Hist_wenb : Wen_Array;
signal HistAdaReg,HistAddra : Array_2x10;
signal HistAddrb : Array_2x11;
signal Hist_Outa : Array_2x32;
signal Hist_Data : Array_2x32;
signal Hist_Datb,Hist_Outb : Array_2x16;
signal Hist_Offset_Reg : signed(11 downto 0);

signal TempEn : std_logic;
signal TempCtrl : std_logic_vector(3 downto 0);
signal One_Wire_Out : std_logic_vector(15 downto 0);

-- Simon 

signal FRSimonSync : Array_2x3;

begin

-- IBUFDS: Differential Input Buffer
GPI0DiffIn : IBUFDS
generic map (
DIFF_TERM => TRUE, -- Differential Termination
IBUF_LOW_PWR => FALSE, -- Low power (TRUE) vs. performance (FALSE) setting for referenced I/O standards
IOSTANDARD => "DEFAULT")
port map (
I  => GPI0_P, -- Diff_p buffer input (connect directly to top-level port)
IB => GPI0_N, -- Diff_n buffer input (connect directly to top-level port)
O => GPI0);	  -- Buffer output

Sys_PLL : SysPLL
  port map
   (-- Clock in ports
    CLK_IN1_P => ClkB_P,
    CLK_IN1_N => ClkB_N,
    -- Clock out ports
    CLK_OUT1 => SysClk,
    CLK_OUT2 => Clk100MHz,
-- Status and control signals
    RESET  => ResetHi);

-- Clock out to AFE chips
-- OBUFDS: Differential Output Buffer
AFEClk0 : OBUFDS
generic map (
IOSTANDARD => "DEFAULT")
port map (
O =>  AFEClk_P(0), -- Diff_p output (connect directly to top-level port)
OB => AFEClk_N(0), -- Diff_n output (connect directly to top-level port)
I => AFEClk -- Buffer input
);

AFEClk1 : OBUFDS
generic map (IOSTANDARD => "DEFAULT")
port map (
O =>  AFEClk_P(1),  
OB => AFEClk_N(1), 
I => AFEClk 
);

-- MIG DDR controller
LPDDRCtrl : LPDDR_Ctrl
 generic map(
    C3_P0_MASK_SIZE => 4, C3_P0_DATA_PORT_SIZE => 32,
    C3_P1_MASK_SIZE => 4, C3_P1_DATA_PORT_SIZE => 32,
    C3_MEMCLK_PERIOD => 6250,  C3_RST_ACT_LOW => 0,
    C3_INPUT_CLK_TYPE => "DIFFERENTIAL",
    C3_CALIB_SOFT_IP => "TRUE",  C3_SIMULATION => "FALSE",
    DEBUG_EN => 0, C3_MEM_ADDR_ORDER => "ROW_BANK_COLUMN",
    C3_NUM_DQ_PINS => 16, C3_MEM_ADDR_WIDTH => 14,
    C3_MEM_BANKADDR_WIDTH => 2)
port map (
   mcb3_dram_dq => SDD, mcb3_dram_a => SDA,
   mcb3_dram_ba => BA,  mcb3_dram_cke => SDCKE,
   mcb3_dram_ras_n => RAS, mcb3_dram_cas_n => CAS,
   mcb3_dram_we_n => SDWE, mcb3_dram_dm => LDM,
   mcb3_dram_udqs => UDQS,	mcb3_rzq => SDRzq,
   mcb3_dram_udm =>  UDM,  mcb3_dram_dqs => LDQS,
   mcb3_dram_ck => SDClk_P, mcb3_dram_ck_n => SDClk_N,
   c3_sys_clk_p => VXO_P, c3_sys_clk_n => VXO_N,
   c3_sys_rst_i => DDR_Reset, c3_calib_done => SDCalDn,
	c3_clk0 => AuxClk,
   c3_rst0 => SD_RstO,
   c3_p2_cmd_clk => SysClk,     c3_p2_cmd_en => WrtCmdEn,
   c3_p2_cmd_instr => SDWrtCmd, c3_p2_cmd_bl => WrtBrstSiz,
   c3_p2_cmd_byte_addr => SDWrtAdStage,
   c3_p2_cmd_empty => SDcmd_empty(0), c3_p2_cmd_full => SDcmd_full(0),
   c3_p2_wr_clk => SysClk,  c3_p2_wr_en => SDwr_en,
   c3_p2_wr_mask => "0000",  c3_p2_wr_data => SDWrtDat,
   c3_p2_wr_full => SDwr_full,  c3_p2_wr_empty => SDwr_empty,
   c3_p2_wr_count => SDwr_count,
   c3_p2_wr_underrun => SDwr_underrun,
   c3_p2_wr_error => SDwr_error,  c3_p3_cmd_clk => SysClk,
   c3_p3_cmd_en => SDRdCmdEn, c3_p3_cmd_instr => SDRdCmd,
   c3_p3_cmd_bl => RdBrstSiz, c3_p3_cmd_byte_addr => SDRdAD,
   c3_p3_cmd_empty => SDcmd_empty(1), c3_p3_cmd_full =>  SDcmd_full(1),
   c3_p3_rd_clk => SysClk,  c3_p3_rd_en => SDrd_en,
   c3_p3_rd_data => SDRdDat,  c3_p3_rd_full => SDrd_full,
   c3_p3_rd_empty => SDrd_empty,
	c3_p3_rd_count => DDR_Rd_Cnt,
   c3_p3_rd_overflow => SDrd_overflow,  c3_p3_rd_error => SDrd_error
); 

-- Read the temperature/ID chip on the four connectoed CMBs
OneWire : One_Wire 
		 port map 
			(reset => CpldRst, clock => Clk100MHz,
			 CpldCS => CpldCS, uCWr => uCWr, GA => GA, 
			 uCA => uCA, Temp => Temp, TempEn => TempEn, 
			 TempCtrl => TempCtrl, uCD => uCD,
			 One_Wire_Out => One_Wire_Out);

FMTx : FM_Tx 
	generic map (Pwidth => 16)
  port map (clock => Clk100MHz,
				reset => ResetHi,
				Enable => TxEn,
				Data => TxPDat,
				Tx_Out => TxOuts);
LVDSTX <= TxOuts.FM;
TxEn <= not FMTxBuff_empty and not TxOuts.Done;

-- Buffer data written from the uC to the LVDS Tx port
FMTx_Buff : LVDSTxBuff
  PORT MAP (clk => Clk100MHz,
    rst => ResetHi, 
    din => uCD,
    wr_en => FMTxBuff_wreq,
    rd_en => TxOuts.Done,
    dout => TxPDat,
    full => FMTxBuff_full,
    empty => FMTxBuff_empty);

-- FM Receiver for microbunch number
FMRx1 : FM_Rx 
	generic map(Pwidth => 24)
	port map(SysClk => SysClk, RxClk => SysClk, 
				reset => ResetHi,
				Rx_In => RxIn,
				Data => Rx1Dat, 
				Rx_Out => RxOut);
RxIn.FM <= GPI0; 

-- FIFO for queueing serial data destined for AFEs and DACs
CmdFifo : Cmd_FIFO
  port map (clk => Clk100MHz, rst => ResetHi,
    wr_en => ODFifoWrReq,rd_en => ODFifoRdReq,
    din => ODFifoData,
    dout => ODFifoOut,
    empty => ODFifoEmpty);

-- Ram for storing DAC values for readback
ShadowRam : DAC_Ram
port map (clka => Clk100MHz,
    wea => ShadowWrt,
    addra => uCA(7 downto 0),
    dina => uCD,
    douta => ShadowOut);

uBunchBuff : SCFIFO_32x256
-- Fifo for buffering micro bunch numbers
  port map (rst => ResetHi,
				clk => SysClk,
				wr_en => uBunchWrt,
				rd_en => uBunchRd,
				din => uBunch,
    dout => uBunchBuffOut, 
    empty => uBunchBuffEmpty,
	 full => uBunchBuffFull);

DDRAddrBuff : SCFIFO_32x256
-- Fifo for buffering micro bunch numbers
  port map (rst => ResetHi,
				clk => SysClk,
				wr_en => uBunchWrt,
				rd_en => DDRAddrRd,
				din => uBunch,
    dout => DDRAddrOut, 
    empty => DDRAddrEmpty,
	 full => DDRAddrFull);

EventBuff : SCFIFO_1Kx16
-- Fifo for buffering one event
  port map (rst => ResetHi,
				clk => SysClk,
				wr_en => EvBuffWrt,
				rd_en => EvBuffRd,
				din => EvBuffDat,
    dout => EvBufffOut, 
    empty => EvBuffEmpty,
	 full => EvBuffFull,
	 data_count => EvBuffWdsUsed);

DRAMRdBuff : SCFIFO_1Kx16
-- Fifo for buffering one event
  port map (rst => ResetHi or rstDRAMBuffHi,
				clk => SysClk,
				wr_en => DRAMRdBuffWrt,
				rd_en => DRAMRdBuffRd,
				din => DRAMRdBuffDat,
    dout => DRAMRdBuffOut, 
    empty => DRAMRdBuffEmpty,
	 full => DRAMRdBuffFull,
	 data_count => DRAMRdBuffWdsUsed);

-- Map 8 bits + frame from the ultrasound chip onto a 9 bit vector
SerDesInP(0) <= (AFEFR_P(0) & AFEDat0_P);
SerDesInN(0) <= (AFEFR_N(0) & AFEDat0_N);
SerDesInP(1) <= (AFEFR_P(1) & AFEDat1_P);
SerDesInN(1) <= (AFEFR_N(1) & AFEDat1_N);
-- Simon
--SerDesInP(0) <= (AFEDat0_P);
--SerDesInN(0) <= (AFEDat0_N);
--SerDesInP(1) <= (AFEDat1_P);
--SerDesInN(1) <= (AFEDat1_N);

GenOnePerAFE : for i in 0 to 1 generate

-- Data buffer for event extracted from DRAM in response to a data request
EvBuffStatFIFO : FIFO_DC_32x1
  port map (rst => ResetHi,
		  wr_clk => RxOutClk(i),
		  rd_clk => SysClk,
		  wr_en => EvenWrtDone(i),
		  rd_en => EvBuffStat_rden, 
		  din => EvBuffStatFIFO_In(i),
		  dout => EvBuffStatFIFO_Out(i),
		  full => EvBuffStatFIFO_Full(i),
		  empty => EvBuffStatFIFO_Empty(i));

-- 1024 x 32 Ram - 1/2 of the usable ADC range
Hist : Hist_Ram
  port map (rsta => ResetHi,rstb => ResetHi,
	 clka => RxOutClk(i),clkb => Clk100MHz,
    wea => Hist_wena(i),web => Hist_wenb(i),
	 addra => HistAddra(i), addrb => HistAddrb(i),
    dina => Hist_Data(i), douta => Hist_Outa(i),
	 dinb => Hist_Datb(i), doutb => Hist_Outb(i));

-- Deserialize x6, since the maximum supported is x10. Concatenate two 6 bit
-- words to form a 12 bit word using an extra 6 bit register.
LVDSInClk : serdes_1_to_n_clk_ddr_s8_diff generic map(
      	S			=> S)
port map (
	clkin_p   		=> AFEDCO_P(i),
	clkin_n   		=> AFEDCO_N(i),
	rxioclkp    	=> rxioclkp(i),
	rxioclkn   		=> rxioclkn(i),
	rx_serdesstrobe => rx_serdesstrobe(i),
	rx_bufg_x1		=> RxOutClk(i));

-- Data Inputs
LVDSInDat : serdes_1_to_n_data_ddr_s8_diff generic map(
      	S		=> S,			
      	D		=> D)
port map (                   
	use_phase_detector 	=> '1',	-- '1' enables the phase detector logic
	datain_p     	=> SerDesInP(i),
	datain_n     	=> SerDesInN(i),
	rxioclkp    	=> rxioclkp(i),
	rxioclkn   		=> rxioclkn(i),
	rxserdesstrobe => rx_serdesstrobe(i),
	gclk    		=> RxOutClk(i),
	bitslip   	=> SlipReq(i),
	reset   		=> SerdesRst(i),
  data_out(53 downto 48)  => FrDat(i),
  data_out(47 downto 42)  => Ins(i)(7)(11 downto 6),
  data_out(41 downto 36)  => Ins(i)(6)(11 downto 6),
  data_out(35 downto 30)  => Ins(i)(5)(11 downto 6),
  data_out(29 downto 24)  => Ins(i)(4)(11 downto 6),
  data_out(23 downto 18)  => Ins(i)(3)(11 downto 6),
  data_out(17 downto 12)  => Ins(i)(2)(11 downto 6),
  data_out(11 downto 6)  => Ins(i)(1)(11 downto 6),
  data_out(5 downto 0)   => Ins(i)(0)(11 downto 6),
  debug_in  	=> "00",
  debug    		=> open);

--LVDSInDatFR : serdes_1_to_n_data_ddr_s8_diff generic map(
--      	S		=> S,			
--      	D		=> 1)
--port map (                   
--	use_phase_detector 	=> '1',	-- '1' enables the phase detector logic
--	datain_p     	=> SerDesInP(i)(8 downto 8),
--	datain_n     	=> SerDesInN(i)(8 downto 8),
--	rxioclkp    	=> rxioclkp(i),
--	rxioclkn   		=> rxioclkn(i),
--	rxserdesstrobe => rx_serdesstrobe(i),
--	gclk    		=> RxOutClk(i),
--	bitslip   	=> '0',
--	reset   		=> SerdesRst(i),
--  data_out(5 downto 0)   => FrDat(i),
--  debug_in  	=> "00",
--  debug    		=> open);

-- pipeline is 96 bits wide which does eight channels of ADC data
Pipeline : AFE_DP_Pipeline
  PORT MAP (
    clka => RxOutClk(i),
    wea => PipeWrt(i downto i),
    addra => DPWrtAd(i),
    addrb => DPRdAd(i),
    dina(95 downto 84) => Ins(i)(7), dina(83 downto 72) => Ins(i)(6),
    dina(71 downto 60) => Ins(i)(5), dina(59 downto 48) => Ins(i)(4),
    dina(47 downto 36) => Ins(i)(3), dina(35 downto 24) => Ins(i)(2),
    dina(23 downto 12) => Ins(i)(1), dina(11 downto 0) => Ins(i)(0),
    clkb => RxOutClk(i),
    doutb(95 downto 84) => Outs(i)(7), doutb(83 downto 72) => Outs(i)(6),
    doutb(71 downto 60) => Outs(i)(5), doutb(59 downto 48) => Outs(i)(4),
    doutb(47 downto 36) => Outs(i)(3), doutb(35 downto 24) => Outs(i)(2),
    doutb(23 downto 12) => Outs(i)(1), doutb(11 downto 0) => Outs(i)(0)
   );

end generate;

AsyncRst <= '1' when ResetHi = '1' or (uCWr = '0' and CpldCS = '0' and uCD(5) = '1' 
					 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
													  or uCA(9 downto 0) =  CSRBroadCastAd)) else '0';

-- DRAM input buffer FIFOs
-- 16 DP Rams 12 bits each. Four FPGAs do 64 channels in total
-- Eight DPRams per AFE, separate read and write clock domains
Gen_FIFOs_Per_AFE : for i in 0 to 1 generate
Gen_FIFOs_Per_Chan : for k in 0 to 7 generate

AFEBuff : DP_Ram_1kx16
  PORT MAP ( 
	 clka => RxOutClk(i), clkb => SysClk,
    wea => AFE_Wrt(i)(k downto k), 
    dina => Buff_In(i)(k),
	 doutb => Buff_Out(i)(k),
	 addra => Buff_Wrt_Ptr(i)(k),
	 addrb => Buff_Rd_Ptr(i)(k));
end generate;
end generate;

--Debug(9 downto 6) <= "0000";

-------------------------- Phase Detector logic ---------------------------

-- The external reference frequency is 25MHz so divide 160MHz
-- by 8 to compare two equal frequencies with the phase detector
IntClkDiv : process(SysClk,CpldRst)
begin
 if CpldRst = '0' then 
 
	FBDiv <= "000"; 
	SqWav <= '0';

elsif falling_edge(SysClk) then

 SqWav <= FBDiv(2);
 FBDiv <= FBDiv + 1;

end if; -- CpldRst
end process IntClkDiv;

----------------- Phase Detector combinatorial outputs --------------------

--UpFF : process(FBDiv(1),SqWav)
--begin
-- if PhDtct = '1' and SqWav = '1' then PhDtct <= '0';
--  elsif rising_edge(FBDiv(1)) then PhDtct <= '1';
-- end if;
--end process;

--DnFF : process(Ref25MHz,PhDtct)
--begin
-- if PhDtct = '1' and SqWav = '1' then SqWav <= '0';
--  elsif rising_edge(Ref25MHz) then SqWav <= '1';
-- end if;
--end process;

PhDtct <= TrgSrc and not(FBDiv(2) xor RefIn);
RefIn <= GPI0;

A7 <= SqWav when GA = 2 else TrgSrc and PhDtct when GA = 3 else GPO;

-- This must sit outside the gen loop
--Debugproc : process (RxOutClk(0), CpldRst)
--
--begin
--
-- if CpldRst = '0' then 
-- Debug(9 downto 2) <= (others => '0');
-- 
--elsif rising_edge (RxOutClk(0)) then
--
----Debug(1) <= ADCSmplGate(0);
--Debug(1) <= not Debug(1);
--if Input_Seqs(0)(1) = Idle then Debug(2) <= '0'; else Debug(2) <= '1'; end if;
--
--if SlfTrgEdge(0)(1) = 1 then Debug(3) <= '1'; else Debug(3) <= '0'; end if;
--if Diff_Reg(0)(1) > IntTrgThresh(0)(1) then Debug(4) <= '1'; else Debug(4) <= '0'; end if;
--
--if SlfTrgEdge(0)(2) = 1 then Debug(5) <= '1'; else Debug(5) <= '0'; end if;
--if Diff_Reg(0)(2) > IntTrgThresh(0)(2) then Debug(6) <= '1'; else Debug(6) <= '0'; end if;
--
--if SlfTrgEdge(0)(3) = 1 then Debug(7) <= '1'; else Debug(7) <= '0'; end if;
--if Diff_Reg(0)(3) > IntTrgThresh(0)(3) then Debug(8) <= '1'; else Debug(8) <= '0'; end if;
--
--if SlfTrgEdge(0)(4) = 1 then Debug(9) <= '1'; else Debug(9) <= '0'; end if;
----if Diff_Reg(0)(4) > IntTrgThresh(0)(4) then Debug(10) <= '1'; else Debug(10) <= '0'; end if;
--
--end if;
--end process;

----------------------- Logic using Serdes receive clocks -----------------------------

Gen_Two_AFEs : for i in 0 to 1 generate

Two_AFEs : process (RxOutClk(i), CpldRst)

begin

 if CpldRst = '0' then 

	DPWrtAd(i) <= X"08"; EvBuffStatFIFO_In(i) <= "0"; EvenWrtDone(i) <= '0';
	DPRdAd(i) <= (others => '0'); PipeWrt(i) <= '0'; ADCSmplGate(i) <= '0';
	iWrtDL(i) <= "00"; GateReq(i) <= '0'; GateWidth(i) <= (others => '0'); 
	SlipReq(i) <= '0'; Slippause(i) <= X"0"; RstReqDL(i) <= "00"; SlipEn(i) <= '1';
	SerdesRst(i) <= '1';
	DoneDly(i) <= "000";
	Triplet(i) <= (others => X"000"); HistTimer(i) <= "00"; Peak(i) <= X"00";
	HistEnDl(i) <= "00";  Hist_wena(i) <= "0"; uBunchOffset(i) <= (others => '0');
	HistAddra(i) <= (others => '0');	HistAdaReg(i) <= (others => '0');
   Hist_Data(i) <= (others => '0'); HistInit(i) <= '0';
	MaskReg(i) <= X"FF";	HistWidth(i) <= (others => '0'); 
	WrtPtrRst(i) <= '0'; Pad_Avg_Count(i) <= (others => '0'); Avg_Req(i) <= '0';

elsif rising_edge (RxOutClk(i)) then

	Debug(9+i) <= ADCSmplGate(i);
-- Synchronous edge detector for uC write strobe w.r.t. deserializer output clock
	iWrtDL(i)(0) <= not CpldCS and not uCWR;
	iWrtDL(i)(1) <= iWrtDL(i)(0);

	if iWrtDL(i) = 1 and uCD(8) = '1' and
		((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
										  or uCA(9 downto 0) = CSRBroadCastAd)
	  then Avg_Req(i) <= '1';
	elsif Pad_Avg_Count(i) /= 0 then Avg_Req(i) <= '0';
	end if;

-- Reset for the input deserializer
if CpldCS = '0' and uCWR = '0' and uCD(2) = '1' 
   and ((uCA(11 downto 10) = GA and uCA = CSRRegAddr)
	or uCA(9 downto 0) = CSRBroadCastAd)
 then SerdesRst(i) <= '1';
 else SerdesRst(i) <= '0';
end if;

	if Pad_Avg_Count(i) = 0 and FRDat(i) = 0 and Avg_Req(i) = '1'
	then Pad_Avg_Count(i) <= "10001";
	elsif Pad_Avg_Count(i) /= 0 and FRDat(i) = 0 
	then Pad_Avg_Count(i) <= Pad_Avg_Count(i) - 1;
	else Pad_Avg_Count(i) <= Pad_Avg_Count(i);
	end if;

-- Engage serdes bit slip if shifted out framing signal isn't all 1's or all 0's
	if (FRDat(i) /= 0 and FRDat(i) /= 63) and AlignReq(i) = '0' and Slippause(i) = 0
	then Slippause(i) <= X"F";
	elsif Slippause(i) /= 0
	then  Slippause(i) <= Slippause(i) - 1;
	else Slippause(i) <= Slippause(i);
	end if;

-- Allow time between requests for bit slip to take effect
	if Slippause(i) = X"F" then SlipReq(i) <= '1';	
	else SlipReq(i) <= '0';
	end if;

-- Read and write addresses for the pipeline delay
if PipeWrt(i) = '1' and DPRdAd(i) = X"FF" then 
	DPRdAd(i) <= (others => '0');
	DPWrtAd(i) <= PipelineSet;
  elsif PipeWrt(i) = '1' and DPRdAd(i) /= X"FF" then 
		  DPWrtAd(i) <= DPWrtAd(i) + 1;
		  DPRdAd(i)  <= DPRdAd(i) + 1;
	else DPWrtAd(i) <= DPWrtAd(i);
		  DPRdAd(i) <= DPRdAd(i);
 end if;

-- Increment the pipeline every other clock tick
	if FRDat(i) = 63 then PipeWrt(i) <= '1';
	else PipeWrt(i) <= '0';
	end if;

-- Channel mask register.
	if iWrtDL(i) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = InputMaskAddr
	 then MaskReg(i) <= uCD(8*i+7 downto 8*i); 
	else MaskReg(i) <= MaskReg(i);
	end if;

-- SlipEn
   if iWrtDL(i) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) =  AlignEnAd
	  then SlipEn(i) <= uCD(i);
	else
	  SlipEn(i) <= SlipEn(i);
	end if;

-- Use this counter to append time since microbunch start to the ADC data
	if TrigReq = '1' and GateWidth(i) = 0 then uBunchOffset(i) <= (others => '0');
	elsif GateWidth(i) /= 0 and FRDat(i) = 0 
	 then uBunchOffset(i) <= uBunchOffset(i) + 1;
	else uBunchOffset(i) <= uBunchOffset(i);
	end if;

-- Hold Gate request high until the next complete ADC sample is available
  if GateReq(i) = '0' and GateWidth(i) = 0 and TrigReq = '1'
	then GateReq(i) <= '1'; --Debug(i+2) <= '1';
	elsif GateReq(i) = '1' and GateWidth(i) /= 0
	then GateReq(i) <= '0'; --Debug(i+2) <= '0';
	else GateReq(i) <= GateReq(i);
  end if;

-- Synchronize the live gate counter with the frame signal
	if AsyncRst = '1' then GateWidth(i) <= (others => '0');
elsif GateWidth(i) = 0 and FRDat(i) = 0 and GateReq(i) = '1' and BeamOn = '1'
  then GateWidth(i) <= BeamOnLength;
elsif GateWidth(i) = 0 and FRDat(i) = 0 and GateReq(i) = '1' and BeamOn = '0'
  then GateWidth(i) <= BeamOffLength;
 elsif GateWidth(i) /= 0 and FRDat(i) = 0 
  then GateWidth(i) <= GateWidth(i) - 1;
 else GateWidth(i) <= GateWidth(i);
 end if;

if DoneDly(i) = 0 and GateWidth(i) = 1 and FRDat(i) = 0 
	then DoneDly(i) <= "101";
elsif DoneDly(i) /= 0 and FRDat(i) = 0 
	then DoneDly(i) <= DoneDly(i) - 1;
else DoneDly(i) <= DoneDly(i);
end if;

-- Allow time for the last ADC samples to be written before declaring the 
-- event complete
if DoneDly(i) = 1 then EvenWrtDone(i) <= '1';
else EvenWrtDone(i) <= '0';
end if;
 
 if GateWidth(i) = 0 and FRDat(i) = 0 and GateReq(i) = '1'
  then ADCSmplGate(i) <= '1';
 elsif AsyncRst = '1'
  or (GateWidth(i) = (X"000" & (ADCSmplCntReg + 2)) and FRDat(i) = 0)
 then ADCSmplGate(i) <= '0';
 else ADCSmplGate(i) <= ADCSmplGate(i);
 end if;

------------------------ Histogramming logic ----------------------------
-- First stage of the triplet register used for peak finding
	if FRDat(i) = 0 then 
	 case HistChan is
	   When "000" => Triplet(i)(0) <= Diff_Reg(i)(0) - Hist_Offset_Reg;
	   When "001" => Triplet(i)(0) <= Diff_Reg(i)(1) - Hist_Offset_Reg;
	   When "010" => Triplet(i)(0) <= Diff_Reg(i)(2) - Hist_Offset_Reg;
	   When "011" => Triplet(i)(0) <= Diff_Reg(i)(3) - Hist_Offset_Reg;
	   When "100" => Triplet(i)(0) <= Diff_Reg(i)(4) - Hist_Offset_Reg;
	   When "101" => Triplet(i)(0) <= Diff_Reg(i)(5) - Hist_Offset_Reg;
	   When "110" => Triplet(i)(0) <= Diff_Reg(i)(6) - Hist_Offset_Reg;
	   When "111" => Triplet(i)(0) <= Diff_Reg(i)(7) - Hist_Offset_Reg;
		When others =>  Triplet(i)(0) <= Triplet(i)(0);
	 end case;
	else Triplet(i)(0) <= Triplet(i)(0);
  end if;

-- Second and third triplet registers
	if FRDat(i) = 0 
		then Triplet(i)(1) <= Triplet(i)(0);
			  Triplet(i)(2) <= Triplet(i)(1);
		else Triplet(i)(1) <= Triplet(i)(1);
			  Triplet(i)(2) <= Triplet(i)(2);
   end if;

-- Counter used for enabling the histogrammer for a specific time associated 
-- with a trigger
	if GateWidth(i) = 0 and FRDat(i) = 0 and GateReq(i) = '1' 
	  and HistWidth(i) = 0 then HistWidth(i) <= WidthReg;
	elsif HistWidth(i) /= 0 and FRDat(i) = 0 then HistWidth(i) <= HistWidth(i) - 1;
	else HistWidth(i) <= HistWidth(i);
	end if;

-- Use these FFs to cross the clock boundary between sysclk and serdes clk
	HistEnDl(i)(0) <= HistEn(i);
	HistEnDl(i)(1) <= HistEnDl(i)(0);

-- Clear the histogram before beginning a new accumulation
	if HistInit(i) = '0' and HistEnDl(i) = 1 then HistInit(i) <= '1';
	elsif HistInit(i) = '1' and HistAddra(i) = "11" & X"FF" then HistInit(i) <= '0';
	else HistInit(i) <= HistInit(i);
	end if;

-- Histogram read, increment, write. Go through some contortions to make it 
-- appear big endian when reading out..
	if HistEnDl(i) = 1 or HistInit(i) = '1' then Hist_Data(i) <= (others => '0');
	else
	 if Hist_Outa(i)(31 downto 16) = X"FFFF" 
		then Hist_Data(i) <= Hist_Outa(i) + X"00010001";
	   else Hist_Data(i) <= Hist_Outa(i) + X"00010000";
	 end if;
	end if;

-- If a peak is found, update the count for the correspnding bin.
-- Histogram write timer
	if HistInit(i) = '0' and FRDat(i) = 0 and HistEnDl(i)(1) = '1' and HistTimer(i) = 0
						  and (HistWidth(i) /= 0 or HistMode = '0')
						  and Triplet(i)(1) > Triplet(i)(0) 
						  and Triplet(i)(1) > Triplet(i)(2)
	then HistTimer(i) <= "01";
	elsif HistTimer(i) /= 0
	then HistTimer(i) <=  HistTimer(i) + 1;
	end if;

-- Histogram memory address logic
	if HistEnDl(i) = 1 then HistAddra(i) <= (others => '0');
	elsif HistInit(i) = '1' then HistAddra(i) <= HistAddra(i) + 1;
	elsif HistInit(i) = '0' and FRDat(i) = 0 and HistEnDl(i)(1) = '1' and HistTimer(i) = 0
						  and (HistWidth(i) /= 0 or HistMode = '0')
						  and (Triplet(i)(1) > Triplet(i)(0)) 
						  and (Triplet(i)(1) > Triplet(i)(2))
	then
	   if Triplet(i)(1) > X"3FF" then HistAddra(i) <= "11" & X"FF";
	elsif Triplet(i)(1) < X"000" then HistAddra(i) <= "00" & X"00";
	 else HistAddra(i) <= std_logic_vector(Triplet(i)(1)(9 downto 0));
	end if;
	elsif HistTimer(i) = 2 then HistAddra(i) <= HistAdaReg(i);
	else HistAddra(i) <= HistAddra(i);
	end if;

	if HistTimer(i) = 1 then HistAdaReg(i) <= HistAddra(i);
	else HistAdaReg(i) <= HistAdaReg(i);
	end if;

-- Histogram memory write strobe
	if HistInit(i) = '1' or HistTimer(i) = 3 then Hist_wena(i) <= "1";
	else Hist_wena(i) <= "0";
	end if;

-----------------------------------------------------------------------------

	RstReqDL(i)(0) <= RstReq(i);
	RstReqDL(i)(1) <= RstReqDL(i)(0);

-- Engage serdes bit slip if shifted out framing signal isn't all 1's or all 0's
	if AlignReq(i) = '0' and Slippause(i) = 0 and (FRDat(i) /= 0 and FRDat(i) /= 63)
	then Slippause(i) <= X"F";
	elsif Slippause(i) /= 0
	then  Slippause(i) <= Slippause(i) - 1;
	else Slippause(i) <= Slippause(i);
	end if;

-- Allow time between requests for bit slip to take effect
	if Slippause(i) = X"8" then 
		SlipReq(i) <= '1';
		--Debug(7+i) <= '1';
	else 
		SlipReq(i) <= '0';
		--Debug(7+i) <= '0';
	end if;

-- Look at the deserilaized frame data. That tells you exactly how many
-- bit slips will be required to align the deserialized ADC data
Case SlipCntReg(i)(2 downto 1) is
	when "00" =>  
	if FRDat(i) = 0 or FRDat(i) = 63 then -- already aligned
		FRStat(i) <= '1';
	 else
		FRStat(i) <= '0';
	end if;
	when "01" => 
	if FRDat(i) = 15 or FRDat(i) = 48 then  -- two bit slips required 
		FRStat(i) <= '1';
	 else
		FRStat(i) <= '0';
	end if;
	when "10" =>
	if FRDat(i) = 3 or FRDat(i) = 60 then  -- four bit slips required  
		FRStat(i) <= '1';
	 else
		FRStat(i) <= '0';
	end if;
	when others => FRStat(i) <= '0';
end Case;

if FRDat(i) = 0 then 
	Debug(5+i) <= '1';
 else
	Debug(5+i) <= '0';
end if;	

end if;

end process;

-- FrDat2
FrDatLong : process (RxOutClk(i), CpldRst)
begin
  if CpldRst = '0' then 
    FrDat2(i) <= "000000";
  elsif rising_edge (RxOutClk(i)) then
    FrDat2(i) <= FrDat(i);
  end if;
end process;

Gen_Eight_Chans : for k in 0 to 7 generate

Eight_Chans : process (RxOutClk(i), CpldRst)

begin

 if CpldRst = '0' then 

	Ins(i)(k)(5 downto 0) <= (others => '0');
	Ped_Reg(i)(k) <= X"000"; SlfTrgEdge(i)(k) <= "00";
	IntTrgThresh(i)(k) <= X"00C"; Ped_Avg(i)(k) <= (others => '0');
	Diff_Reg(i)(k) <=  X"000"; Avg_En(i)(k) <= '0';
	Buff_Wrt_Ptr(i)(k) <= (others => '0'); 
	WrtCrrntWdCntAd(i)(k) <= (others => '0'); 
	WrtNxtWdCntAd(i)(k) <= (others => '0'); 
   InWdCnt(i)(k) <= (others => '0'); AFE_Wrt(i)(k) <= '0'; 
	Input_Seqs(i)(k) <= Idle;  In_Seq_Stat(i)(k) <= '0';
	ADCSmplCntr(i)(k) <= "0000";

elsif rising_edge (RxOutClk(i)) then

-- Copy six bit shift result to six bit register to form a 12 bit result
	Ins(i)(k)(5 downto 0) <= Ins(i)(k)(11 downto 6);

-- Pedestal registers, self trigger

-- Subtract off the pedestal before applying the threshold
	 if FRDat(i) = 0 
	  then Diff_Reg(i)(k) <= signed(Ins(i)(k)) - Ped_Reg(i)(k);
	 else Diff_Reg(i)(k) <= Diff_Reg(i)(k);
	 end if;

-- Pedestal registers
	if iWrtDL(i) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = PedRegAddr(i)(k)
	then Ped_Reg(i)(k) <= signed(uCD(11 downto 0));
	elsif FRDat(i) = 0 and Pad_Avg_Count(i) = 1 then Ped_Reg(i)(k) <= Ped_Avg(i)(k)(15 downto 4);
	else Ped_Reg(i)(k) <= Ped_Reg(i)(k);
	end if;

	if FRDat(i) = 0 and Pad_Avg_Count(i) /= 0 then Avg_En(i)(k) <= '1';
	 elsif Pad_Avg_Count(i) = 0 then Avg_En(i)(k) <= '0';
	 else Avg_En(i)(k) <= Avg_En(i)(k);
	end if;
 
-- Pedestal averaging 
	if Avg_En(i)(k) = '0' then Ped_Avg(i)(k) <= (others => '0');
	elsif FRDat(i) = 0 and Avg_En(i)(k) = '1'
	then Ped_Avg(i)(k) <= Ped_Avg(i)(k) + signed(Ins(i)(k));
	else Ped_Avg(i)(k) <= Ped_Avg(i)(k);
	end if;

-- Self trigger threshold registers
	if iWrtDL(i) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ThreshRegAddr(i)(k)
	 then IntTrgThresh(i)(k) <= signed(uCD(11 downto 0));
	else IntTrgThresh(i)(k) <= IntTrgThresh(i)(k);
	end if;

-- Self trigger synchronous edge detector
	 if FRDat(i) = 0 and ADCSmplGate(i) = '1' 
	   and Diff_Reg(i)(k) > IntTrgThresh(i)(k) 
		then SlfTrgEdge(i)(k)(0) <= '1';
	 elsif FRDat(i) = 0 and Diff_Reg(i)(k) <= IntTrgThresh(i)(k)
		then SlfTrgEdge(i)(k)(0) <= '0';
	 else SlfTrgEdge(i)(k)(0) <= SlfTrgEdge(i)(k)(0);
	 end if;

	 if FRDat(i) = 0 
		then SlfTrgEdge(i)(k)(1) <= SlfTrgEdge(i)(k)(0);
	 else SlfTrgEdge(i)(k)(1) <= SlfTrgEdge(i)(k)(1);
	 end if;

-- Input Sequencer (i: 0->1, k: 0->7)
-- (Idle,Increment,WrtChanNo,WrtTimeStamp,WrtHits,WrtHitWdCnt,LdNxtWrtAd);
case Input_Seqs(i)(k) is
	When Idle =>
	 if GateReq(i) = '1'
	  then Input_Seqs(i)(k) <= Increment;
	 else Input_Seqs(i)(k) <= Idle;
	  end if;
	When Increment => Input_Seqs(i)(k) <= WrtChanNo;
	When WrtChanNo =>
	 if FRDat(i) = 0 and SlfTrgEdge(i)(k) = 1 
	  then Input_Seqs(i)(k) <= WrtTimeStamp;
	 elsif GateWidth(i) = 0
	  then Input_Seqs(i)(k) <= WrtHitWdCnt;
	 else Input_Seqs(i)(k) <= WrtChanNo;
	 end if;
	When WrtTimeStamp => Input_Seqs(i)(k) <= WrtHits;
	When WrtHits =>
		if ADCSmplCntr(i)(k) = 0 
		 then Input_Seqs(i)(k) <= WrtChanNo;
		elsif GateWidth(i) = 0 then Input_Seqs(i)(k) <= WrtHitWdCnt;
		else Input_Seqs(i)(k) <= WrtHits;
	   end if;
	When WrtHitWdCnt => Input_Seqs(i)(k) <= LdNxtWrtAd;
	When LdNxtWrtAd => Input_Seqs(i)(k) <= Idle;
end case;

If Input_Seqs(i)(k) <= Idle then In_Seq_Stat(i)(k) <= '0';
else In_Seq_Stat(i)(k) <= '1';
end if;

-- ADC Sample counter
		if Input_Seqs(i)(k) = WrtChanNo then ADCSmplCntr(i)(k) <= ADCSmplCntReg;
	elsif Input_Seqs(i)(k) = WrtHits and FRDat(i) = 0 then ADCSmplCntr(i)(k) <= ADCSmplCntr(i)(k) - 1;
	else ADCSmplCntr(i)(k) <= ADCSmplCntr(i)(k);
	end if;

-- Event word counter
	   if Input_Seqs(i)(k) = Idle then InWdCnt(i)(k) <= (others => '0');
	elsif Input_Seqs(i)(k) = WrtTimeStamp  -- Input_Seqs(i)(k) = Increment or 
	   or (Input_Seqs(i)(k) = WrtChanNo and FRDat(i) = 0 and SlfTrgEdge(i)(k) = 1) -- Diff_Reg(i)(k) > IntTrgThresh(i)(k))
		or (Input_Seqs(i)(k) = WrtHits and FRDat(i) = 0)
	then InWdCnt(i)(k) <= InWdCnt(i)(k) + 1;
	end if;

-- Store present write pointer so a leading word count can be stored at the end of the microbunch
	if Input_Seqs(i)(k) = Increment then WrtCrrntWdCntAd(i)(k) <= Buff_Wrt_Ptr(i)(k);
	else WrtCrrntWdCntAd(i)(k) <= WrtCrrntWdCntAd(i)(k);
	end if;

-- After writing the leading word count the pointer needs to go to the end of the event
	if Input_Seqs(i)(k) = WrtHitWdCnt then WrtNxtWdCntAd(i)(k) <= Buff_Wrt_Ptr(i)(k) + 1;
	else WrtNxtWdCntAd(i)(k) <= WrtNxtWdCntAd(i)(k);
	end if;

-- DPRam write pointer
	if (Input_Seqs(i)(k) = WrtChanNo and FRDat(i) = 0 and SlfTrgEdge(i)(k) = 1) -- Diff_Reg(i)(k) > IntTrgThresh(i)(k))
	or Input_Seqs(i)(k) = WrtTimeStamp
	or (Input_Seqs(i)(k) = WrtHits and FRDat(i) = 0) 
	then Buff_Wrt_Ptr(i)(k) <= Buff_Wrt_Ptr(i)(k) + 1;
	elsif Input_Seqs(i)(k) = WrtHitWdCnt then Buff_Wrt_Ptr(i)(k) <= WrtCrrntWdCntAd(i)(k);
	elsif Input_Seqs(i)(k) = LdNxtWrtAd then Buff_Wrt_Ptr(i)(k) <= WrtNxtWdCntAd(i)(k);
	elsif WrtPtrRst(i) = '1' then Buff_Wrt_Ptr(i)(k) <= (others => '0');
	else Buff_Wrt_Ptr(i)(k) <= Buff_Wrt_Ptr(i)(k);
	end if;

-- Qualify writes with the mask register bits
	if ((Input_Seqs(i)(k) = WrtHits and FRDat(i) = 0) 
	 or (Input_Seqs(i)(k) = WrtChanNo and FRDat(i) = 0 and SlfTrgEdge(i)(k) = 1) -- Diff_Reg(i)(k) > IntTrgThresh(i)(k))
	 or Input_Seqs(i)(k) = WrtTimeStamp or Input_Seqs(i)(k) = WrtHitWdCnt)
	    and MaskReg(i)(k) = '1' then 
	AFE_Wrt(i)(k) <= '1';
	else 
	AFE_Wrt(i)(k) <= '0';
	end if;

-- Write the microbunch number, channel number, and timestamp followed by ADC data
	if Input_Seqs(i)(k) = WrtChanNo and FRDat(i) = 0 and SlfTrgEdge(i)(k) = 1 -- Diff_Reg(i)(k) > IntTrgThresh(i)(k)
	 then Buff_In(i)(k) <= ControllerNo & PortNo & GA & ChanArray(8*i+k);
  elsif Input_Seqs(i)(k) = WrtTimeStamp then Buff_In(i)(k) <= ADCSmplCntReg & uBunchOffset(i);
  elsif Input_Seqs(i)(k) = WrtHitWdCnt then Buff_In(i)(k) <= X"0" & "00" & InWdCnt(i)(k);
  else
	 Buff_In(i)(k) <= X"0" & Outs(i)(k);
 end if;

end if;

end process;

end generate;
end generate;

----------------------- 160 Mhz clocked logic -----------------------------

ResetHi <= not CpldRst;  -- Generate and active high reset for the Xilinx macros

main : process(SysClk, CpldRst)

 begin 

-- asynchronous reset/preset
 if CpldRst = '0' then
 
	AFE_Num <= 0; Chan_Num <= 0;
-- AFE input clock
	AFEClk <= '0';  AFERst <= '0'; 
-- Synchronous edge detectors for various strobes
	RDDL <= "00"; WRDL <= "00"; AddrReg <= (others => '0'); 
	PulseSel <= '0'; FlashEn <= '0'; LEDSrc <= '0'; 
	SlfTrgEn <= '0'; TurnOffTime <= '0' & X"70"; 
	LEDTime <= '0' & X"30"; TurnOnTime <= '0' & X"01"; 
	BeamOnLength <= X"050"; BeamOffLength <= X"700";
	TmgSrcSel <= '0'; 
-- Upper DRAM word staging register
	CDStage <= (others => '0'); 
-- Test pulse generator signals
	Buff_Rst <= '0'; Seq_Rst <= '0'; 
	Strt_req <= '0'; TrigType <= X"000"; 
	EventWdCnt <= (others => '0'); NxtWdCount <= (others => '0');
-- MIG related signals
	SDWrtAd <= (others => '0'); SDWrtAdStage <= (others => '0'); 
   SDWrtCmd <= "000"; WrtCmdEn <= '0'; SDrd_en <= '0';
	SDRdAD <= (others => '0'); SDRdPtr <= (others => '0'); 
	SDRdCmd <= "000"; SDRdCmdEn <= '0'; RdHi_LoSel <= '0'; Even_Odd <= '0';
   SDWrtDat <= (others => '0'); SDwr_en <= '0'; WrtHi_LoSel <= '0'; 
	rstDRAMBuffHi <= '0';

	Event_Builder <= Idle; NoHIts <= (others => X"00"); Read_Seq_Stat <= X"0";
	DDR_Write_Seq <= Idle;
	DDR_Read_Seq <= Idle; Seq_Busy <= '0'; DDRWrtSeqStat <= "000";
	Buff_Rd_Ptr(0) <= (others => (others => '0')); TrigReq <= '0';
	Buff_Rd_Ptr(1) <= (others => (others => '0')); TrigReqD <= '0';
	NextEvAddr(0) <= (others => (others => '0'));
	NextEvAddr(1) <= (others => (others => '0'));
	SampleCount <= (others => '0'); BuffRdCount <= (others => '0');
	uBunchWrt <= '0'; uBunchRd <= '0'; 	GateCounter <= '0' & X"00"; 
	BeamOn <= '0'; EvOvf <=(others => X"FF"); 	Rx1DatReg <= (others => '0');
	FlashGate <= '0'; FifoRdD <= '0'; 
	IdleDL <= "00"; GPIDL(1) <= "00"; uBunch <= (others => '0'); uBunchGuard <= '0';
	EvBuffWrt <= '0'; EvBuffRd <= '0'; EvBuffDat <= (others => '0'); PageRdReq <= '0';
	DRAMRdBuffDat <= (others => '0'); DDRWrtCount <= (others => '0'); PageRdStat <= '0';
	PageWdCount <= (others => '0'); DRAMRdBuffWrt <= '0'; DRAMRdBuffRd <= '0';
	EvBuffStat_rden <= '0'; RdDone <= '0'; GPOCount <= "000"; GPO <= '0'; 
	
elsif rising_edge (SysClk) then 

--if Input_Seqs(0)(0) /= Idle then
--	Debug(7) <= '1';
--else
--	Debug(7) <= '0';
--end if;

-- Synchronous edge detectors for read and write strobes
RDDL(0) <= not uCRD and not CpldCS;
RDDL(1) <= RDDL(0);

WRDL(0) <= not uCWR and not CpldCS;
WRDL(1) <= WRDL(0);

-- Latch the address for post increment during reads
if RDDL = 1 or WRDL = 1 then AddrReg <= uCA;
else AddrReg <= AddrReg;
end if;

if WRDL = 1 and uCA(9 downto 0) = FlashCtrlAddr 
-- Flash gate enable bit
then FlashEn <= uCD(0);
-- Choose between LED and flashgate
	  PulseSel <= uCD(1);
-- Select source for LED flasher pulse
	  LEDSrc <= uCD(2);
else FlashEn <= FlashEn;
	  PulseSel <= PulseSel;
	  LEDSrc <= LEDSrc;
end if;

-- Reset DRAM Read FIFO
if (WRDL = 1 and uCA(9 downto 0) = PageStatAddr and uCD(8) = '1') or
   (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = PageRstAddr and uCD(8) = '1')
then rstDRAMBuffHi <= '1';
else rstDRAMBuffHi <= '0';
end if;

-- Register for determining the turn on time 
if WRDL = 1 and uCA(9 downto 0) = OnTimeAddr
then TurnOnTime <= uCD(8 downto 0);
else TurnOnTime <= TurnOnTime;
end if;

-- Register for determining the turn off time  
if WRDL = 1 and uCA(9 downto 0) = OffTimeAddr
then TurnOffTime <= uCD(8 downto 0);
else TurnOffTime <= TurnOffTime;
end if;

if WRDL = 1 and uCA(9 downto 0) = LEDTimeAddr
then LEDTime <= uCD(8 downto 0);
else LEDTime <= LEDTime;
end if;

-- Register for determining the live gate lengths
   if WRDL = 1 and uCA(9 downto 0) = BeamOnLengthAd
then BeamOnLength <= uCD(11 downto 0);
else BeamOnLength <= BeamOnLength;
end if;

   if WRDL = 1 and uCA(9 downto 0) = BeamOffLengthAd
then BeamOffLength <= uCD(11 downto 0);
else BeamOffLength <= BeamOffLength;
end if;

-- Trigger control register bits
if uWRDL = 1 and uCA(9 downto 0) = IntTrgEnAddr 
then TmgSrcSel <= uCD(0); 
	  SlfTrgEn <= uCD(1);
else SlfTrgEn <= SlfTrgEn;
	  TmgSrcSel <= TmgSrcSel;
end if;

if Strt_req = '1' then AFEClk <= '0'; Debug(8) <= '0';
 else AFEClk <= not AFEClk; Debug(8) <= not Debug(8);
end if;

----------------------- Flash gate logic ----------------------

-- Counter for timing the flash gate. 270 counts at 159 MHz = 1.695 ns
if (GateCounter = 270 and TmgSrcSel = '1') 
or	(SlfTrgEn = '1' and RxOut.Done = '1' and Rx1Dat(20) = '1') 
or TmgSrcSel = '0' or FlashEn = '0'
	then GateCounter <= (others => '0');
else GateCounter <= GateCounter + 1;
end if;

--Debug(1) <= RxOut.Done;

if SlfTrgEn = '1' and RxOut.Done = '1' and Rx1Dat(20) = '1' 
	then BeamOn <= '1';
elsif SlfTrgEn = '1' and RxOut.Done = '1' and Rx1Dat(20) = '0'
	then BeamOn <= '0';
else BeamOn <= BeamOn;
end if;

-- "Turn on" in this case means reducing the SiPM voltage, "turn off"
-- means restoring to its nominal value
  if GateCounter = TurnOnTime and FlashEn = '1' and PulseSel = '1' then FlashGate <= '1';
elsif GateCounter = TurnOffTime then FlashGate <= '0';
else FlashGate <= FlashGate;
end if;

if FlashEn = '1' and
     ((PulseSel = '0' and (GateCounter = LEDTime or GateCounter = LEDTime + 1))
   or (PulseSel = '1' and FlashGate = '1'))
then Pulse <= '1';
else Pulse <= '0';
end if;

------------------------------- Trigger Logic ----------------------------

-- Global reset term
if WRDL = 1 and uCD(5) = '1' 
	and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
											or uCA(9 downto 0) = CSRBroadCastAd)
then Buff_Rst <= '1';
else Buff_Rst <= '0';
end if;

-- AFE specific reset
if uCWR = '0' and CpldCS = '0' and uCD(6) = '1'  
	and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
									      or uCA(9 downto 0) = CSRBroadCastAd)
then AFERst <= '1';
else AFERst <= '0';
end if;

-- Trig out width counter
if GPOCount = 0 and TrigReq = '1' then GPOCount <= "111";
elsif GPOCount /= 0 then GPOCount <= GPOCount - 1;
else GPOCount <= GPOCount;
end if;

-- Trig Out or SpillGate on the GPO LEMO.
   if TrigReq = '1' then GPO <= '1';
elsif GPOCount = 1 then GPO <= '0';
else GPO <= GPO;
end if;

-- Buff reset will clear the FM receiver parity error (if any)
if WRDL = 1 and uCD(7) = '1' 
	and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr) 
											or uCA(9 downto 0) = CSRBroadCastAd)
then RxIn.Clr_Err <= '1';
else RxIn.Clr_Err <= '0';
end if;

-- Latch the trigger type
-- If internally generated use these bits
-- if externally generated use the trigger type field from the trigger message
if RxOut.Done = '1' then TrigType <=  X"00" & Rx1Dat(23 downto 20);
else TrigType <= TrigType;
end if;

if RxOut.Done = '1' and Rx1Dat(21) = '1' and Rx1Dat(19 downto 0) = X"00000" and uBunchGuard = '0'
	then uBunch(31 downto 20) <= uBunch(31 downto 20) + 1;
		  uBunch(19 downto 0) <= (others => '0');
		  uBunchGuard <= '1';
 elsif RxOut.Done = '1' and Rx1Dat(21) = '1' and Rx1Dat(19 downto 0) /= 0 
	then uBunch <= uBunch(31 downto 20) & Rx1Dat(19 downto 0); uBunchGuard <= '0';
 elsif RxOut.Done = '1' and Rx1Dat(21) = '0'
	then uBunch <= X"000" & Rx1Dat(19 downto 0); uBunchGuard <= '0';
else uBunch <= uBunch; uBunchGuard <= uBunchGuard;
end if;

Debug(1) <= RxOut.Done;
Debug(2) <= Rx1Dat(21);
Debug(3) <= uBunchWrt;
Debug(4) <= uBunchGuard;

-- Write uBunch number at the uBunch beginning
if RxOut.Done = '1' and SlfTrgEn = '1' then uBunchWrt <= '1';
else uBunchWrt <= '0';
end if;

if RxOut.Done = '1' then Rx1DatReg <= Rx1Dat;
else Rx1DatReg <= Rx1DatReg;
end if;

-- Read the uBunch number after calculating and writing the event word count.
if Event_Builder = WrtuBunchLo or (SlfTrgEn = '0' and uBunchBuffEmpty = '0') 
	then uBunchRd <= '1';
else uBunchRd <= '0';
end if;

if SlfTrgEn = '1' and ((GateCounter = TurnOnTime and TmgSrcSel = '1')
   or (RxOut.Done = '1' and TmgSrcSel = '0'))
	then TrigReq <= '1';
elsif TrigReqD = '1' then TrigReq <= '0'; 
end if;
	TrigReqD <= TrigReq;

-- flag used to signal end of read out across clock domains
 if Event_Builder = Incr_Chan1
   and AFE_Num = 1 and Chan_Num = 7 
	then RdDone <= '1';
  else RdDone <= '0';
 end if;

 if Event_Builder = Add_Wd_Count or (SlfTrgEn = '0' and EvBuffStatFIFO_Empty = 0)
   then EvBuffStat_rden <= '1';
 else EvBuffStat_rden <= '0';
 end if; 

-------------------------- Event Builder -------------------------------

Case Event_Builder is
   When Idle => Read_Seq_Stat <= X"0";
	 	if EvBuffStatFIFO_Empty = 0 and SlfTrgEn = '1' and RdDone = '0'
		then Event_Builder <= Check_Mask0;
		else Event_Builder <= Idle;
		end if;
	When Check_Mask0 => Read_Seq_Stat <= X"1"; 
		if MaskReg(AFE_Num)(Chan_Num) = '1' 
		  then Event_Builder <= Check_Ovf;
		 else Event_Builder <= Incr_Chan0;
		end if;
	When Check_Ovf => Read_Seq_Stat <= X"3"; 
			if EvOvf(AFE_Num)(Chan_Num) = '0' 
				then Event_Builder <= Add_Wd_Count;
			else Event_Builder <= Incr_Chan0;
			end if;
	When Incr_Chan0 => Read_Seq_Stat <= X"2"; 
			if AFE_Num = 1 and Chan_Num = 7 
				then Event_Builder <= WdCountWrt;
			else Event_Builder <= Check_Mask0;
			end if;
	When Add_Wd_Count => Event_Builder <= Incr_Chan0; Read_Seq_Stat <= X"4"; 
	When WdCountWrt => Event_Builder <= WrtuBunchHi; Read_Seq_Stat <= X"5"; 
	When WrtuBunchHi => Event_Builder <= WrtuBunchLo; Read_Seq_Stat <= X"6"; 
	When WrtuBunchLo => Event_Builder <= Check_Mask1; Read_Seq_Stat <= X"7";  
	When Check_Mask1 =>  Read_Seq_Stat <= X"8";
		if MaskReg(AFE_Num)(Chan_Num) = '1' and EvOvf(AFE_Num)(Chan_Num) = '0'
		  then Event_Builder <= Wait1;
		 else Event_Builder <= Incr_Chan1;
		end if;
	When Incr_Chan1 => Read_Seq_Stat <= X"9";
		 if AFE_Num = 1 and Chan_Num = 7
		  then Event_Builder <= Idle;
			else Event_Builder <= Check_Mask1;
			end if;
	When Wait1 =>  Read_Seq_Stat <= X"A"; 
			if NoHIts(AFE_Num)(Chan_Num) = '0' 
			 then Event_Builder <= Wait2;
			else Event_Builder <= Incr_Chan1;
		   end if;
	When Wait2 => Read_Seq_Stat <= X"B"; 
		  Event_Builder <= WrtData;
	When WrtData =>
		 if SampleCount <= 1 then Event_Builder <= Incr_Chan1;
		else Event_Builder <= WrtData;
		end if;
End Case;

for i in 0 to 1 loop
for j in 0 to 7 loop
if	Buff_Out(i)(j) = 0 then NoHIts(i)(j) <= '1';
else NoHIts(i)(j) <= '0';
end if;
end loop;
end loop;
--EvOvf /= (X"00",X"00") & (14 downto 0);
 if Event_Builder = WdCountWrt
	then EvBuffDat <= EventWdCnt;
elsif Event_Builder = WrtuBunchHi
	then EvBuffDat <= uBunchBuffOut(31 downto 16);
elsif Event_Builder = WrtuBunchLo
	then EvBuffDat <= uBunchBuffOut(15 downto 0);
else EvBuffDat <= Buff_Out(AFE_Num)(Chan_Num);
end if;

if Event_Builder = WdCountWrt or Event_Builder = WrtData or Event_Builder = WrtuBunchHi
	or Event_Builder = WrtuBunchLo or Event_Builder = WrtData
 then EvBuffWrt <= '1'; 
 else EvBuffWrt <= '0'; 
 end if;

-- Increment AFE number after eight channels have been read out
if Event_Builder = Incr_Chan0 or Event_Builder = Incr_Chan1 
	then 
	 if Chan_Num /= 7 then Chan_Num <= Chan_Num + 1;
		else Chan_Num <= 0; 
	  end if;
	 if Chan_Num = 7 and AFE_Num = 0 then AFE_Num <= AFE_Num + 1;
	 elsif Chan_Num = 7 and AFE_Num = 1 then AFE_Num <= 0;
	 else AFE_Num <= AFE_Num;
    end if;
 end if;

NextEvAddr(AFE_Num)(Chan_Num) <= Buff_Rd_Ptr(AFE_Num)(Chan_Num) 
									    + Buff_Out(AFE_Num)(Chan_Num)(9 downto 0) + 1;

-- Assert reads when the read sequencer is requesting data
-- specify which FIFO to read with the channel number
 if SlfTrgEn = '0'
   then 	Buff_Rd_Ptr(0) <= (others => (others => '0'));
			Buff_Rd_Ptr(1) <= (others => (others => '0'));
-- If the channel has an overflow, skip ahead to the next channel
-- and advance the pointer on this channel to the next event
 elsif Event_Builder = Incr_Chan0 and MaskReg(AFE_Num)(Chan_Num) = '1' and EvOvf(AFE_Num)(Chan_Num) = '1'
		then Buff_Rd_Ptr(AFE_Num)(Chan_Num) <= NextEvAddr(AFE_Num)(Chan_Num);
--Buff_Rd_Ptr(AFE_Num)(Chan_Num) 
-- + Buff_Out(AFE_Num)(Chan_Num)(9 downto 0) + 1;
  elsif BuffRdCount /= 0 
    or (Event_Builder = Check_Mask1 and MaskReg(AFE_Num)(Chan_Num) = '1' 
	 and EvOvf(AFE_Num)(Chan_Num) = '0')
	then Buff_Rd_Ptr(AFE_Num)(Chan_Num) <= Buff_Rd_Ptr(AFE_Num)(Chan_Num) + 1;
	else Buff_Rd_Ptr(AFE_Num)(Chan_Num) <= Buff_Rd_Ptr(AFE_Num)(Chan_Num);
	end if;

   if Event_Builder = Idle then EventWdCnt <= X"0003";
elsif Event_Builder = Incr_Chan0 and MaskReg(AFE_Num)(Chan_Num) = '1' and EvOvf(AFE_Num)(Chan_Num) = '0'
	then EventWdCnt <= EventWdCnt + Buff_Out(AFE_Num)(Chan_Num); -- BuffOut_Mux; 
	else EventWdCnt <= EventWdCnt;
	end if;

NxtWdCount <= EventWdCnt + Buff_Out(AFE_Num)(Chan_Num);
--EventWdCnt + Buff_Out(AFE_Num)(Chan_Num)
	if Event_Builder = Idle then EvOvf <= (others => X"00");
elsif Event_Builder = Check_Ovf and MaskReg(AFE_Num)(Chan_Num) = '1' 
then 
   if NxtWdCount >= PageSize
	then EvOvf(AFE_Num)(Chan_Num) <= '1';
	else EvOvf <= EvOvf;
	end if;
else EvOvf <= EvOvf;
end if;

-- Count down the words stored in the uBunch event for this channel
if Event_Builder = Check_Mask1 and MaskReg(AFE_Num)(Chan_Num) = '1' and EvOvf(AFE_Num)(Chan_Num) = '0' 
	then SampleCount <= Buff_Out(AFE_Num)(Chan_Num)(8 downto 0); -- BuffOut_Mux(8 downto 0);
elsif Event_Builder = WrtData and SampleCount /= 0
then SampleCount <= SampleCount - 1;
else SampleCount <= SampleCount;
end if;
-- The same as above but with slightly different conditions
if Event_Builder = Check_Mask1 and MaskReg(AFE_Num)(Chan_Num) = '1' and EvOvf(AFE_Num)(Chan_Num) = '0' 
	then BuffRdCount <= Buff_Out(AFE_Num)(Chan_Num)(8 downto 0); -- BuffOut_Mux(8 downto 0);
elsif (Event_Builder = WrtData or Event_Builder = Wait1 or Event_Builder = Wait2) and BuffRdCount /= 0
then BuffRdCount <= BuffRdCount - 1;
else BuffRdCount <= BuffRdCount;
end if;

-------------------------------- DDR Macro Interface -------------------------------

-- Clear out any stale data from the read FIFO before starting a new block read
-- Read_Seq_FSM is (Idle,CheckEmpty,FirstCmd,CheckRdBuff0,RdWdCount,
-- 						CheckRdBuff1,RdDataHi,RdDataLo);
case DDR_Read_Seq is
	When Idle => --Debug(10 downto 8) <= "000";
 if WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrLoAd)
		 	 or uCA(9 downto 0) = BrdCstRdPtrLoAd or uCA(9 downto 0) = uBunchRdPtrLoAd)
		then DDR_Read_Seq <= CheckEmpty;
		else DDR_Read_Seq <= Idle;
		end if;
	When CheckEmpty => --Debug(10 downto 8) <= "001"; 
		if SDrd_empty = '1' then DDR_Read_Seq <= FirstCmd;
		else DDR_Read_Seq <= CheckEmpty;
		end if;
	When FirstCmd => --Debug(10 downto 8) <= "010";
		if PageRdReq = '1'
		then DDR_Read_Seq <= CheckRdBuff0;
		else DDR_Read_Seq <= Idle;
		end if;
	When CheckRdBuff0 =>  --Debug(10 downto 8) <= "011";
			if SDrd_empty = '0' then DDR_Read_Seq <= RdWdCount; 
			else DDR_Read_Seq <= CheckRdBuff0;
			end if;
	When RdWdCount => --Debug(10 downto 8) <= "100";
				  if RdHi_LoSel = '0'
				 then DDR_Read_Seq <= RdDataHi;
			    else DDR_Read_Seq <= RdDataLo;
				end if;
	When CheckRdBuff1 => --Debug(10 downto 8) <= "101";
		if SDrd_empty = '0' then DDR_Read_Seq <= RdDataHi;
		else DDR_Read_Seq <= CheckRdBuff1;
		end if;
	When RdDataHi => --Debug(10 downto 8) <= "110";
		if PageWdCount = 0 then DDR_Read_Seq <= Idle;
		else DDR_Read_Seq <= RdDataLo;
		end if;
	When RdDataLo => --Debug(10 downto 8) <= "111";
 		   if PageWdCount /= 0 and SDrd_en = '1' then DDR_Read_Seq <= CheckRdBuff1;
		elsif PageWdCount = 0 then DDR_Read_Seq <= Idle;
		else DDR_Read_Seq <= RdDataLo;
		end if;
	When others => DDR_Read_Seq <= Idle;
end case;

--if DDR_Read_Seq = Idle then
--	Debug(4) <= '0';
--else
--	Debug(4) <= '1';
--end if;

-- DDR Read address register
-- Microcontroller access upper
 if WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrHiAd)
				or uCA(9 downto 0) = BrdCstRdPtrHiAd)
then SDRdAD <= uCD(13 downto 0) & SDRdAD(15 downto 0);
-- Microcontroller access lower
elsif WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrLoAd)
				or uCA(9 downto 0) = BrdCstRdPtrLoAd)
then SDRdAD <= SDRdAD(29 downto 16) & uCD;
-- Convert the microcontroller data to a page number
elsif WRDL = 1 and uCA(9 downto 0) = uBunchRdPtrHiAd
 then SDRdAD <= uCD(3 downto 0) & SDRdAD(25 downto 0);
elsif WRDL = 1 and uCA(9 downto 0) = uBunchRdPtrLoAd
 then SDRdAD <= SDRdAD(29 downto 26) & uCD & "00" & X"00";
-- Increment by 8 long words for each burst read command
elsif SDRdCmdEn = '1' and DDR_Read_Seq /= CheckEmpty then SDRdAD <= SDRdAD + 32;
else SDRdAD <= SDRdAD;
end if;

-- DDR Read address pointer
-- Microcontroller access upper
 if WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrHiAd)
				or uCA(9 downto 0) = BrdCstRdPtrHiAd)
then SDRdPtr <= uCD(13 downto 0) & SDRdPtr(15 downto 0);
-- Microcontroller access lower
elsif WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrLoAd)
				or uCA(9 downto 0) = BrdCstRdPtrLoAd)
then SDRdPtr <= SDRdPtr(29 downto 16) & uCD;
elsif WRDL = 1 and uCA(9 downto 0) = uBunchRdPtrHiAd
 then SDRdPtr <= uCD(3 downto 0) & SDRdPtr(25 downto 0);
elsif WRDL = 1 and uCA(9 downto 0) = uBunchRdPtrLoAd
 then SDRdPtr <= SDRdPtr(29 downto 26) & uCD & "00" & X"00";
-- Increment by 1 long word for each read command
elsif SDrd_en = '1' and DDR_Read_Seq /= CheckEmpty then SDRdPtr <= SDRdPtr + 4;
else SDRdPtr <= SDRdPtr;  
end if;

-- DDR controller output FIFO is 32 bits. Clock once per two uC reads
  if (RDDL = 2 and AddrReg(11 downto 10) = GA 
		and (AddrReg(9 downto 0) = SDRamPortAd or AddrReg(9 downto 0) = SDRamSwapPort) 
		and RdHi_LoSel = '1')
	or (DDR_Read_Seq = CheckEmpty and SDrd_empty = '0')
	or (SDrd_en = '0' and PageWdCount /= 0 and DDR_Read_Seq = RdDataLo)
	then SDrd_en <= '1'; 
   else SDrd_en <= '0'; 
  end if;

-- Send a read command to fetch burst size number of long words
 if (SDrd_en = '1' and SDRdPtr(4 downto 0) = "01100" and DDR_Read_Seq /= CheckEmpty) 
  or DDR_Read_Seq = FirstCmd
then SDRdCmdEn <= '1';
	SDRdCmd <= ReadCmd; 
else SDRdCmdEn <= '0';
	SDRdCmd <= "000";  
end if;

-- Toggle between upper and lower words during reads from the DDR
 if WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrLoAd)
		 	 or uCA(9 downto 0) = BrdCstRdPtrLoAd or uCA(9 downto 0) = uBunchRdPtrLoAd)
then RdHi_LoSel <= '0'; 
 elsif (RDDL = 2 and AddrReg(11 downto 10) = GA 
 and (AddrReg(9 downto 0) = SDRamPortAd or AddrReg(9 downto 0) = SDRamSwapPort))
  or (PageWdCount /= 0 and (DDR_Read_Seq = RdDataHi
  or (DDR_Read_Seq = RdDataLo and PageWdCount /= 0 and SDrd_en = '0')))
then RdHi_LoSel <= not RdHi_LoSel;  
end if;

 if WRDL = 1 and uCA(9 downto 0) = uBunchRdPtrLoAd
then PageRdReq <= '1'; 
elsif DDR_Read_Seq = RdWdCount
then PageRdReq <= '0';
else PageRdReq <= PageRdReq;
end if;

-- DDR Page data FIFO write
if PageWdCount /= 0 and (DDR_Read_Seq = RdDataHi or (DDR_Read_Seq = RdDataLo and SDrd_en = '1'))
then DRAMRdBuffWrt <= '1'; 
else DRAMRdBuffWrt <= '0'; 
end if;

 if RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = PageFIFOAddr
  then DRAMRdBuffRd <= '1'; 
  else DRAMRdBuffRd <= '0';
 end if;

 if RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = PageFIFOAddr
	then PageRdStat <= '0'; 
 elsif (DDR_Read_Seq = RdDataHi or DDR_Read_Seq = RdDataLo) and PageWdCount = 0
 then PageRdStat <= '1'; 
 else PageRdStat <= PageRdStat; 
 end if;
 
 -- Page read word count. Assume size is already limited by the event builder
	if DDR_Read_Seq = RdWdCount and RdHi_LoSel = '0' 
			 then PageWdCount <= SDRdDat(26 downto 16);
elsif DDR_Read_Seq = RdWdCount and RdHi_LoSel = '1' 
			 then PageWdCount <= SDRdDat(10 downto 0);
 elsif PageWdCount /= 0 
	 and (DDR_Read_Seq = RdDataHi or (DDR_Read_Seq = RdDataLo and SDrd_en = '1'))
   then PageWdCount <= PageWdCount - 1;
	elsif DDR_Read_Seq = Idle then PageWdCount <= (others => '0');
else PageWdCount <= PageWdCount;
end if;

-- DDR page data
    if DDR_Read_Seq = RdDataHi then DRAMRdBuffDat <= SDRdDat(31 downto 16);
 elsif DDR_Read_Seq = RdDataLo then DRAMRdBuffDat <= SDRdDat(15 downto 0);
else DRAMRdBuffDat <= DRAMRdBuffDat;
end if;

-- DDR Write address register
-- Microcontroller access upper
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrHiAd 
then SDWrtAd <= uCD(13 downto 0) & SDWrtAd(15 downto 0);
-- Microcontroller access lower
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrLoAd
then SDWrtAd <= SDWrtAd(29 downto 16) & uCD;
-- Increment by 4 for each long word write
elsif SDwr_en = '1'
then SDWrtAd <= SDWrtAd + 4;
--Use the stored microbunch number shifted left by 9 places 
-- to advance to the next requested page
elsif DDR_Write_Seq = SetWrtPtr 
then SDWrtAd(9 downto 0) <= (others => '0');
	  SDWrtAd(29 downto 10) <= DDRAddrOut(19 downto 0);
else SDWrtAd <= SDWrtAd;
end if;

-- DDR Write address staging register
-- Microcontroller access upper
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrHiAd 
then SDWrtAdStage <= uCD(13 downto 0) & SDWrtAdStage(15 downto 0);
-- Microcontroller access lower
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrLoAd
then SDWrtAdStage <= SDWrtAdStage(29 downto 16) & uCD;
-- Reset the address at spill beginning
elsif Strt_req = '1' then SDWrtAdStage <= (others => '0');
-- Keep the address from the last update until the write command has been sent
elsif WrtCmdEn = '1' and DDR_Write_Seq /= SetWrtPtr then SDWrtAdStage <= SDWrtAd;
elsif DDR_Write_Seq = SetWrtPtr 
then SDWrtAdStage(9 downto 0) <= (others => '0');
	  SDWrtAdStage(29 downto 10) <= DDRAddrOut(19 downto 0);
else SDWrtAdStage <= SDWrtAdStage;
end if;

-- Writes to the MIG write FIFO
if (WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamPortAd and WrtHi_LoSel = '1')
  or (DDR_Write_Seq = WrtDDR and WrtHi_LoSel = '1')
or (DDR_Write_Seq = WritePad and (SDwr_en = '0' or SDWrtAd(4 downto 0) /= "11100")) 
then SDwr_en <= '1'; --Debug(7) <= '1';
else SDwr_en <= '0'; --Debug(7) <= '0';
end if;

-- When the number of writes = burst size, send a write command
	if (SDwr_en = '1' and SDWrtAd(4 downto 0) = "11100") 
or (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrLoAd)
or DDR_Write_Seq = SndCmd
then SDWrtCmd <= "010";
	  WrtCmdEn <= '1';
else SDWrtCmd <= "000";
	  WrtCmdEn <= '0';
	  end if;

-- Toggle between upper and lower words during writes to the DDR
    if (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrLoAd)
		 or Buff_Rst = '1' or Strt_req = '1' or DDR_Write_Seq = SetWrtPtr 
		 then WrtHi_LoSel <= '0'; 
 elsif (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamPortAd)
	     or DDR_Write_Seq = WrtDDR
 then WrtHi_LoSel <= not WrtHi_LoSel;
 end if;

-- Idle,ChkWrtBuff,SndCmd,WtCmdMtpy,SetWrtPtr,Wait1,WrtDDR,WritePad
Case DDR_Write_Seq is
   When Idle => DDRWrtSeqStat <= "000"; --Debug(10 downto 8) <= "000";
-- If the FIFO words used is > leading word count, 
-- then at least one event is ready for copying to DRAM
		if SlfTrgEn = '1' and EvBuffWdsUsed >= EvBufffOut(10 downto 0) 
		 and EvBuffEmpty = '0' 
		then DDR_Write_Seq <= ChkWrtBuff;
		else DDR_Write_Seq <= Idle;
		end if;
-- If there is stale data in the MIG transmit FIFO, force a burst write
	When ChkWrtBuff => DDRWrtSeqStat <= "001"; --Debug(10 downto 8) <= "001";
		if SDwr_empty = '0' then DDR_Write_Seq <= SndCmd;
		else DDR_Write_Seq <= SetWrtPtr;
		end if;
-- Send a MIG burst write command
	When SndCmd => DDR_Write_Seq <= WtCmdMtpy; DDRWrtSeqStat <= "010"; --Debug(10 downto 8) <= "010";
-- Wait for the MIG write FIFO to go empty
	When WtCmdMtpy => DDRWrtSeqStat <= "011";  --Debug(10 downto 8) <= "011";
		if SDwr_empty = '1' then DDR_Write_Seq <= SetWrtPtr;
		else DDR_Write_Seq <= WtCmdMtpy;
		end if;
-- When the input FIFOs have at least one event, copy the data to the DDR
	When SetWrtPtr =>  DDRWrtSeqStat <= "100"; --Debug(10 downto 8) <= "100";
			DDR_Write_Seq <= Wait1;
	When Wait1 => DDRWrtSeqStat <= "101"; --Debug(10 downto 8) <= "101";
		  DDR_Write_Seq <= WrtDDR;
	When WrtDDR =>  DDRWrtSeqStat <= "110"; --Debug(10 downto 8) <= "110";
		 if DDRWrtCount <= 1 then DDR_Write_Seq <= WritePad;
		else DDR_Write_Seq <= WrtDDR;
		end if;
-- At the end of each event flush the last data words from the write FIFO into the DDR
	When WritePad => DDRWrtSeqStat <= "111";  --Debug(10 downto 8) <= "111";
		if SDWrtAd(4 downto 0) = "11100" 
		then DDR_Write_Seq <= WaitWrtDn;
		else DDR_Write_Seq <= WritePad;
		end if;
	When WaitWrtDn =>
			if SDwr_empty = '1' 
	       then DDR_Write_Seq <= Idle;
			else DDR_Write_Seq <= WaitWrtDn;
			end if;
	When others => DDR_Write_Seq <= Idle;
end case;

if DDR_Write_Seq = Wait1 or (SlfTrgEn = '0' and DDRAddrEmpty = '0')
   then DDRAddrRd <= '1';
  else DDRAddrRd <= '0';
 end if;

--Debug(6) <= DDRAddrEmpty;
--Debug(5) <= DDRAddrRd;
--Debug(3) <= uBunchBuffEmpty;
--Debug(1) <= uBunchRd;

if DDR_Write_Seq = SetWrtPtr and EvBufffOut(10 downto 0) > 0 
						then DDRWrtCount <= EvBufffOut(10 downto 0);
elsif DDR_Write_Seq = WrtDDR and DDRWrtCount /= 0 
						then DDRWrtCount <= DDRWrtCount - 1;
else DDRWrtCount <= DDRWrtCount;
end if;

if DDR_Write_Seq =  Wait1 then EvBuffRd <= '1';  
elsif (DDR_Write_Seq = WrtDDR and DDRWrtCount <= 1) or DDR_Write_Seq = Idle
then EvBuffRd <= '0';
else EvBuffRd <= EvBuffRd;
end if;

-- Edge detector for DDR_Seq going to the Idle state
if DDR_Write_Seq = Idle then IdleDL(0) <= '1';
else IdleDL(0) <= '0';
end if;
IdleDL(1) <= IdleDL(0);

if DDR_Write_Seq = Idle then Seq_Busy <= '0';
else Seq_Busy <= '1';
end if;

 if Buff_Rst = '1' or Strt_req = '1' or DDR_Write_Seq = SetWrtPtr 
		 then Even_Odd <= '0';
elsif DDR_Write_Seq = WrtDDR 
		 then Even_Odd <= not Even_Odd;
 else Even_Odd <= Even_Odd;
end if;

-- Second staging register
if WRDL = 1 and  uCA(11 downto 10) = GA and 
 	uCA(9 downto 0) = SDRamPortAd and WrtHi_LoSel = '0'
	then CDStage <= uCD;
elsif (DDR_Write_Seq = WrtDDR and Even_Odd = '0')
	then CDStage <= EvBufffOut; -- BuffOut_Mux;
else CDStage <= CDStage;
end if;

-- Multiplexer to feed the appropriate data to the DRAM write FIFO
   if DDR_Write_Seq = WrtDDR and Even_Odd = '1' 
	 then SDWrtDat <= CDStage & EvBufffOut; -- BuffOut_Mux;
else SDWrtDat <= CDStage & uCD;
end if;

end if; -- CpldRst

end process main;

Clk100 : process(Clk100MHz, CpldRst)

 begin 

-- asynchronous reset/preset
 if CpldRst = '0' then

	uWRDL <= "00"; uRDDL <= "00";
	
-- Control bits written by the uC
	DACCS <= "111"; RampGate <= "00";
	BiasTarget <= (others => X"000"); BiasActual <= (others => X"000"); 
	DACLd <= '1'; DACClk <= '1'; DACDat <= '0'; Dev_Sel <= '0'; 
	DACShift <= (others => '0'); Octal_Shift <= Idle; ClkDiv <= (others => '0');
	BitCount <= "00000"; ODFifoWrReq <= '0'; ODFifoRdReq <= '0'; 
	ODFifoData <= (others => '0'); TrgSrc <= '0'; 
	PipelineSet <= X"04";  WidthReg <= X"10";
	
	TrigType <= X"000";	FMTxBuff_wreq <= '0'; 
	Hist_wenb <= (others => "0"); HistAddrb <= (others => (others => '0')); 
	Hist_Datb <= (others => (others => '0'));  Hist_Offset_Reg <= (X"FF6");
	Count100ms <= (others => '0'); ADCSmplCntReg <= X"8"; 
	ControllerNo <= "00000"; PortNo <= "00000"; 

	MuxEn <= X"0"; Muxad <= "00"; MuxSelReg <= "000"; MuxadReg <= "00";
	UpTimeStage <= (others => '0'); UpTimeCount <= (others => '0');
	Counter10us <= (others => '0'); Counter1ms <= (others => '0');
	Counter1s <= (others => '0');	TestCount <= (others => '0'); 
	Counter1us <= X"00"; -- 1us time base 
	ResetCount <= (others => '0'); DDR_Reset <= '0'; 
	AFEPDn <= "11"; AFECS <= "11"; AFESDI <= '0'; StatReg <= X"0";
	AFESClk <= '0'; SClkDL <= "000"; AFERdReg <= (others => '0');
	GPOSel <= '0'; HistEnReq <= "00"; uAddrReg <= (others => '0');
	HistInterval <= X"0800"; HistMode <= '0'; HistChan <= "000"; HistEn <= "00"; 
	HistGateCnt0 <= (others => '0');	HistGateCnt1 <= (others => '0'); 
	
-- Signals for aligning RxOutClks
	AlignReq <= "00"; RstReq <= "00";
	SlipCntReg <= ("000","000"); 
	uSecCounter <= (others => '0'); FR_OK <= "00";


elsif rising_edge (Clk100MHz) then 

-- Synchronous edge detectors for read and write strobes
uRDDL(0) <= not uCRD and not CpldCS;
uRDDL(1) <= uRDDL(0);

uWRDL(0) <= not uCWR and not CpldCS;
uWRDL(1) <= uWRDL(0);

-- Latch the address for post increment during reads
if uRDDL = 1 or uWRDL = 1 then uAddrReg <= uCA;
else uAddrReg <= uAddrReg;
end if;

------------------ Various prameter storage registers ---------------------

-- Specifiy the number of ADC samples per hit
if uWRDL = 1 and uCA(9 downto 0) = ADCSmplCntrAd 
	then ADCSmplCntReg <= uCD(3 downto 0);
 else ADCSmplCntReg <= ADCSmplCntReg;
end if;

-- Strobe to write data to L:VDS FM return link
if uWRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = LVDSTxFIFOAd 
 then FMTxBuff_wreq <= '1';
else FMTxBuff_wreq <= '0';
end if;

-- Register for defining the geographical address of the FEB
   if uWRDL = 1 and uCA(9 downto 0) =  FEBAddresRegAd then 
		ControllerNo <= uCD(12 downto 8); 
		PortNo <= uCD(4 downto 0);
	else
	   ControllerNo <= ControllerNo;
		PortNo <= PortNo;
	end if;

-------------------------- Histogram Control Registers ------------------

-- Specify the histogrammer accumulation interval
if uWRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = HistIntvalAd 
then HistInterval <= uCD(15 downto 0); 
else HistInterval <= HistInterval;
end if;

--Specify histogrammer offset
if uWRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = HistOfstAd
then Hist_Offset_Reg <= signed(uCD(11 downto 0));
else Hist_Offset_Reg <= Hist_Offset_Reg;
end if;

-- Specify which of the eight AFE channels to histogramm, whether or not to use an external gate
if uWRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = HistCtrlAd)
					  or uCA( 9 downto 0) = HistCtrlBroadCastAd)
 then HistChan <= uCD(2 downto 0);
		HistMode <= uCD(4);
else HistChan <= HistChan;
		HistMode <= HistMode;
end if; 

-- Histogrammer enable logic

-- latch the request from the uC until the next 1ms count
 if HistEnReq(0) = '0' and uWRDL = 1 and uCA(11 downto 10) = GA 
	and uCA(9 downto 0) = HistCtrlAd and uCD(5) = '1'  
then HistEnReq(0) <= '1';
elsif HistEnReq(0) = '1' and HistGateCnt0 = 1 and Counter1ms = Count1ms
then HistEnReq(0) <= '0';
else HistEnReq(0) <=  HistEnReq(0);
end if;

-- Histogrammer accumulation timers
 if HistEn(0) = '0' and HistGateCnt0 = 0 and HistEnReq(0) = '1' and Counter1ms = Count1ms
then HistGateCnt0 <= HistInterval;
elsif HistGateCnt0 /= 0 and Counter1ms = Count1ms 
then HistGateCnt0 <= HistGateCnt0 - 1;
else HistGateCnt0 <= HistGateCnt0;
end if;

-- Latch the histogram enable request from the uC until the next 1ms count
  if HistEn(0) = '0' and HistGateCnt0 = 0 and HistEnReq(0) = '1' and Counter1ms = Count1ms
then HistEn(0) <= '1';
elsif HistEn(0) = '1' and HistGateCnt0 = 1 and Counter1ms = Count1ms 
then  HistEn(0) <= '0';
else  HistEn(0) <=  HistEn(0);
end if;

-- Define the histogrammer accumulation time
 if HistEnReq(1) = '0' and uWRDL = 1 and uCA(11 downto 10) = GA 
	and uCA(9 downto 0) = HistCtrlAd and uCD(6) = '1'
then HistEnReq(1) <= '1';
elsif HistEnReq(1) = '1' and HistGateCnt1 = 1 and Counter1ms = Count1ms
then HistEnReq(1) <= '0';
else HistEnReq(1) <=  HistEnReq(1);
end if;

 if HistEn(1) = '0' and HistGateCnt1 = 0 and HistEnReq(1) = '1' and Counter1ms = Count1ms
then HistGateCnt1 <= HistInterval;
elsif HistGateCnt1 /= 0 and Counter1ms = Count1ms 
then HistGateCnt1 <= HistGateCnt1 - 1;
else HistGateCnt1 <= HistGateCnt1;
end if;

 if HistEn(1) = '0' and HistGateCnt1 = 0 and HistEnReq(1) = '1' and Counter1ms = Count1ms
then HistEn(1) <= '1';
elsif HistEn(1) = '1' and HistGateCnt1 = 1 and Counter1ms = Count1ms 
then  HistEn(1) <= '0';
else  HistEn(1) <=  HistEn(1);
end if;

-- Histogram memory pointers
 if uWRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = HistPtrAd0 
	then HistAddrb(0) <= uCD(10 downto 0);
elsif (uRDDL = 2 or uWRDL = 2) and uAddrReg(11 downto 10) = GA and uAddrReg(9 downto 0) = HistRd0Ad  
	then HistAddrb(0) <= HistAddrb(0) + 1;
else HistAddrb(0) <= HistAddrb(0);
end if;

-- Diagnostic writes to the Hist Ram 0
if uWRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = HistRd0Ad 
then Hist_wenb(0) <= "1";
else Hist_wenb(0) <= "0";
end if;
-- Allow a write to Data Port B to use the memory for other purposes
Hist_Datb(0) <= uCD;

 if uWRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = HistPtrAd1 
	then HistAddrb(1) <= uCD(10 downto 0);
elsif (uRDDL = 2 or uWRDL = 2) and uAddrReg(11 downto 10) = GA and uAddrReg(9 downto 0) = HistRd1Ad
	then HistAddrb(1) <= HistAddrb(1) + 1;
else HistAddrb(1) <= HistAddrb(1);
end if;

-- Diagnostic writes to the Hist Ram 1
if uWRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = HistRd1Ad 
then Hist_wenb(1) <= "1";
else Hist_wenb(1) <= "0";
end if;
-- Allow a write to Data Port B to use the memory for other purposes
Hist_Datb(0) <= uCD;

-- Make a 100ms counter by scaling the 1ms counter
   if Counter1ms = Count1ms and Count100ms /= 99 then Count100ms <= Count100ms + 1;
elsif Counter1ms = Count1ms and Count100ms  = 99 then Count100ms <= (others => '0');
else Count100ms <= Count100ms;
end if;

-- Timer to set width of DDR MIG reset
if uWRDL = 1 and uCD(3) = '1' and ResetCount = 0 
 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
    or uCA(9 downto 0) = CSRBroadCastAd)
then ResetCount <= X"F";
elsif ResetCount /= 0 then ResetCount <= ResetCount - 1;
end if;

-- MIG Reset
if ResetCount /= 0 then DDR_Reset <= '1';
else DDR_Reset <= '0';
end if;

-- AFE power down control bits
if uWRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
						or uCA(9 downto 0) = CSRBroadCastAd)
then AFEPDn <= uCD(1 downto 0);
else AFEPDn <= AFEPDn;
end if;

-- Select between LEMO and LVDS inputs for the triggers 
if uWRDL = 1 and uCA(9 downto 0) = TrigCtrlAddr 
then TrgSrc <= uCD(1);
else TrgSrc <= TrgSrc;
end if;

-- Analog multiplexer control lines
if uWRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = MuxCtrlAd then 
	Case uCD(4 downto 2) is
		When "100" => MuxEn <= "0001";
		When "101" => MuxEn <= "0010";
		When "110" => MuxEn <= "0100";
		When "111" => MuxEn <= "1000";
		When others => MuxEn <= "0000";
	end case;
-- Remap select lines since layout channel ordering is not monotonic
		Case uCD(1 downto 0) is
		When "00" => MuxAd <= "11";
		When "01" => MuxAd <= "00";
		When "10" => MuxAd <= "10";
		When "11" => MuxAd <= "01";
		When others => MuxAd <= "00";
	end case;
	MuxadReg <= uCD(1 downto 0);
	MuxSelReg <= uCD(4 downto 2);
 else 
	MuxEn <= MuxEn;
	Muxad <= Muxad;
	MuxadReg <= MuxadReg;
	MuxSelReg <= MuxSelReg;
end if;

-- 10us time base
if Counter10us /= Count10us then Counter10us <= Counter10us + 1;
else Counter10us <= (others => '0');
end if;

-- 1ms time base
if Counter1ms = Count1ms then Counter1ms <= (others => '0');
else Counter1ms <= Counter1ms + 1;
end if;

-- 1us time base
if Counter1us = Count1us then 
	Counter1us <= X"00";
else 
   Counter1us <= Counter1us + 1;
end if;

-- 1 second time base
if	Counter1s /= Count1s then Counter1s <= Counter1s + 1;
else Counter1s <= (others => '0');
end if;
-- Uptime in seconds since th last FPGA configure
if	Counter1s = Count1s then UpTimeCount <= UpTimeCount + 1;
else UpTimeCount <= UpTimeCount;
end if;

-- Register for staging uptime count.
if CpldCS = '1' then UpTimeStage <= UpTimeCount;
else UpTimeStage <= UpTimeStage;
end if;

-- Time base for alignment sequence
	if AlignSeq /= Idle and AlignSeq /= RstCntr
		and Counter1us = Count1us and uSecCounter /= 335
	 then uSecCounter <= uSecCounter + 1;
	elsif AlignSeq = Idle or AlignSeq = RstCntr
	   or (Counter1us = Count1us and uSecCounter = 335)
	 then uSecCounter <= (others => '0');
	else uSecCounter <= uSecCounter;
	end if;

-- Testcounter counter is writeable. For each read of the lower half, the entire
-- 32 bit counter increments
   if uWRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = TestCounterHiAd 
	 then TestCount <= uCD & TestCount(15 downto 0);
		elsif uWRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = TestCounterLoAd 
    then TestCount <= TestCount(31 downto 16) & uCD;
      elsif uRDDL = 2 and uAddrReg(11 downto 10) = GA and uAddrReg(9 downto 0) = TestCounterLoAd 
    then TestCount <= TestCount + 1;
      else TestCount <= TestCount;
end if;

-- Use this to cross clock domains
FR_OK <= FRStat;
--Debug(4) <= FRStat(0);
Debug(7) <= FRStat(1);

-- (Idle,SendPDn,SendPEn,SendRst,WaitFR,CheckFR,ChkSlipCnt,SendSlipReq);
case AlignSeq is
	when Idle => --Debug(3 downto 1) <= "000";
	 if AlignReq /= 0 and Counter1us = Count1us
		then AlignSeq <= SendPDn;
	 else AlignSeq <= Idle;
	 end if;
	when SendPDn => --Debug(3 downto 1) <= "001";
	  if Counter1us = Count1us and uSecCounter = 335
		 then AlignSeq <= SendPEn;
	  elsif AlignReq = 0 then
	    AlignSeq <= Idle;
	   else AlignSeq <= SendPDn;
	  end if;
	when SendPEn => --Debug(3 downto 1) <= "010";
	  if Counter1us = Count1us and uSecCounter = 335
		 then AlignSeq <= SendRst;
	  elsif AlignReq = 0 then
	    AlignSeq <= Idle;
	   else AlignSeq <= SendPEn;
	  end if;
	when SendRst => --Debug(3 downto 1) <= "011";
		if Counter1us = Count1us and uSecCounter = 335
		  then AlignSeq <= WaitFR;
	  elsif AlignReq = 0 then
	    AlignSeq <= Idle;
		else AlignSeq <= SendRst;
		end if;
	when WaitFR => --Debug(3 downto 1) <= "100";
	  --if Counter1us = Count1us and uSecCounter(3 downto 0) = X"F" then
     if Counter1us = Count1us and uSecCounter = 300 then	  
	    AlignSeq <= CheckFR;       
	   elsif AlignReq = 0 then
	    AlignSeq <= Idle;
	  else AlignSeq <= WaitFR;
	 end if;
	when CheckFR => --Debug(3 downto 1) <= "101";
	   if Counter1us = Count1us and uSecCounter = 300
	   --if Counter1us = Count1us and uSecCounter(3 downto 0) = X"F"
		  then
			  if (AlignReq(0) = '1' and FR_OK(0) = '1') 
		     or (AlignReq(1) = '1' and FR_OK(1) = '1')
			  or AlignReq = 0 then 
			   AlignSeq <= Idle; 
			  else 
			   AlignSeq <= RstCntr;
			 end if;
		 end if;
	 when RstCntr => --Debug(3 downto 1) <= "110";
			 if Counter1us = Count1us then
			  AlignSeq <= SendPDn;
	      else 
			  AlignSeq <= RstCntr;
		  end if;
	when others => --Debug(3 downto 1) <= "111"; AlignSeq <= Idle; 
 end Case;

-- Only 0,2 and 4 are allowed slip count values
  if uWRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SlipCntRegAd
	then 
		SlipCntReg(0) <= uCD(2)  & (not uCD(2)  and uCD(1)) & '0';
		SlipCntReg(1) <= uCD(10) & (not uCD(10) and uCD(9)) & '0';
     else
		SlipCntReg(0) <= SlipCntReg(0);
		SlipCntReg(1) <= SlipCntReg(1);
	end if;

-- Pass on a reset to AFE deserializer if requested
	if AlignSeq = SendRst 
		and Counter1us = Count1us and uSecCounter = 0
	  then RstReq <= AlignReq;
	 else RstReq <= "00";
	end if;
	
	-- Writing a 1 enables alignment
	if uWRDL = 1 and uCD(0) = '1' 
		and AFEPDn(0) = '0'
		and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = SlipCtrlAd)
												or uCA(9 downto 0) = BrdCstAlgnReqAd)
	 then AlignReq(0) <= '1'; 
-- Writing a 0 disables the alignment -> force quit alignment
-- Two conditions for stopping alignment: it successfully aligned, or it was told to stop
	elsif (AlignReq(0) = '1' and AlignSeq = CheckFR and FR_OK(0) = '1')
	  or (uWRDL = 1 and uCD(0) = '0' and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = SlipCtrlAd)
													or uCA(9 downto 0) = BrdCstAlgnReqAd))
	 then AlignReq(0) <= '0'; 
	else AlignReq(0) <= AlignReq(0);
	end if;

	if uWRDL = 1 and uCD(1) = '1' 
		and AFEPDn(1) = '0'
		and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = SlipCtrlAd)
													or uCA(9 downto 0) = BrdCstAlgnReqAd)
	 then AlignReq(1) <= '1'; 
	elsif (AlignReq(1) = '1' and AlignSeq = CheckFR and FR_OK(1) = '1')
  	   or (uWRDL = 1 and uCD(1) = '0' and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = SlipCtrlAd)
													or uCA(9 downto 0) = BrdCstAlgnReqAd))
	 then AlignReq(1) <= '0'; 
	else AlignReq(1) <= AlignReq(1);
	end if;
	
-- Pipeline delay setting 
if uWRDL = 1 and uCA(9 downto 0) = PipeLineAddr then PipelineSet <= uCD(7 downto 0);
else PipelineSet <= PipelineSet;
end if;

-- Gate width - determines the number of ADC sample per trigger and uCA(11 downto 10) = GA 
if uWRDL = 1 and uCA(9 downto 0) = GateAddr then WidthReg <= uCD(7 downto 0);
else WidthReg <= WidthReg;
end if;

-------------- Serializer for Bias Trim, Bias Bus, VGain DACs, AFE setup registers -------------

-- Clock runs at 159 MHz, serial data bit period is SysClk div 8
ClkDiv <= ClkDiv + 1;

-- Store DAC, AFE setup data in a buffer, so that multiple writes can be 
-- done without waiting for DAC data to be serialized

-- Bias voltage ramping logic
-- Write the bias target registers
if uWRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ("00" & X"44")
 then BiasTarget(0) <= uCD(11 downto 0);
else BiasTarget(0) <= BiasTarget(0);
end if;

if uWRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ("00" & X"45")
 then BiasTarget(1) <= uCD(11 downto 0);
else BiasTarget(1) <= BiasTarget(1);
end if;

-- If the present bias value and the target bias value differ, bring the
-- present bias value closer to the target by one count every millisecond.
-- Uee the rampgate signal to force one write when the actual = the target.

for i in 0 to 1 loop

if BiasTarget(i) /= BiasActual(i) then RampGate(i) <= '1';
elsif BiasTarget(i) = BiasActual(i) and ODFifoWrReq = '1'
then RampGate(i) <= '0';
else RampGate(i) <= RampGate(i);
end if;

if BiasTarget(i) /= BiasActual(i) and Counter1ms = 4
	then
	     if BiasTarget(i) < BiasActual(i) then BiasActual(i) <= BiasActual(i) - 1;
	  elsif BiasTarget(i) > BiasActual(i) then BiasActual(i) <= BiasActual(i) + 1;
		end if;
 end if;
 
end loop;

-- This state machine controls the order of AFE writes to the command FIFO
-- Timer requirement enables the OD write strobe for one clock cycle
--(Idle, SetAFESel0, WrtAFE0, SetAFESel1, WrtAFE1);
case OD_Write is
	when Idle =>
		if (AlignSeq = SendPdn or AlignSeq = SendPEn)
			and Counter1us = 1 and uSecCounter = 0 then
			if AlignReq(0) = '1' then
				OD_Write <= SetAFESel0; --Debug(4) <= '1';
			elsif AlignReq = 2 then
				OD_Write <= SetAFESel1; --Debug(4) <= '1';
			end if;
		else
			OD_Write <= Idle; --Debug(4) <= '0';
		end if;
	when SetAFESel0 => OD_Write <= WrtAFE0; --Debug(4) <= '1';
	when WrtAFE0 =>
		if AlignReq(1) = '1' then
			OD_Write <= SetAFESel1; --Debug(4) <= '1';
		else OD_Write <= Idle; --Debug(4) <= '0';
		end if;
	when SetAFESel1 => OD_Write <= WrtAFE1; --Debug(4) <= '1';
	when WrtAFE1 => OD_Write <= Idle; --Debug(4) <= '0';
end case;

-- Write the updated actual value to the bias DAC
if RampGate(0) = '1' and Counter1ms = Count1ms
	then ODFifoData <= X"0440" & BiasActual(0);
elsif RampGate(1) = '1' and Counter1ms = 1
	then ODFifoData <= X"0450" & BiasActual(1);
else ODFifoData <= ("00" & uCA(9 downto 0) & uCD);
end if;

if RampGate(0) = '1' and Counter1ms = Count1ms 
	then ODFifoData <= X"0440" & BiasActual(0); --Debug(7) <= '0';
elsif RampGate(1) = '1' and Counter1ms = 1
	then ODFifoData <= X"0450" & BiasActual(1); --Debug(7) <= '0';
-- Clock adjustment, enabling and disabling LVDS output for each ultrasound
elsif AlignSeq = SendPDn and (OD_Write = SetAFESel0 or OD_Write = WrtAFE0) then
	ODFifoData <= X"101" & X"0002"; --Debug(7) <= '1';
elsif AlignSeq = SendPDn and (OD_Write = SetAFESel1 or OD_Write = WrtAFE1) then
	ODFifoData <= X"201" & X"0002"; --Debug(7) <= '1';
elsif AlignSeq = SendPEn and (OD_Write = SetAFESel0 or OD_Write = WrtAFE0) then
	ODFifoData <= X"101" & X"0000"; --Debug(7) <= '0';
elsif AlignSeq = SendPEn and (OD_Write = SetAFESel1 or OD_Write = WrtAFE1) then
	ODFifoData <= X"201" & X"0000"; --Debug(7) <= '0';
else ODFifoData <= ("00" & uCA(9 downto 0) & uCD); --Debug(7) <= '0';
end if;

if (RampGate(0) = '1' and Counter1ms = Count1ms)
or (RampGate(1) = '1' and Counter1ms = 1)
or ((AlignSeq = SendPDn or AlignSeq = SendPEn) and 
     (OD_Write = SetAFESel0 or OD_Write = WrtAFE0 or OD_Write = SetAFESel1 or OD_Write = WrtAFE1))
or	(uWRDL = 1 and uCA(11 downto 10) = GA
           and ((uCA(9 downto 0) >= DatArray0Min and uCA(9 downto 0) <= CtrlArray2Max
				 and uCA(9 downto 0) /= ("00" & X"44") and uCA(9 downto 0) /= ("00" & X"45"))
				 or (uCA(9 downto 0) >= AFE0ArrayMin and uCA(9 downto 0) <= AFE0ArrayMax)
				 or (uCA(9 downto 0) >= AFE1ArrayMin and uCA(9 downto 0) <= AFE1ArrayMax)))
then ODFifoWrReq <= '1';
else ODFifoWrReq <= '0';
end if;

-- If data is going to the DACs, shadow it in ram
if uWRDL = 1 and uCA(11 downto 10) = GA 
  and uCA(9 downto 0) >= DatArray0Min and uCA(9 downto 0) <= CtrlArray2Max
then ShadowWrt <= "1";
else ShadowWrt <= "0";
end if;

-- Distinguish between DACs and AFEs
if ODFifoEmpty = '0' and ClkDiv = 0 and DacLd = '1' and Octal_Shift = Idle 
           and ODFifoOut(27 downto 16) >= DatArray0Min and ODFifoOut(27 downto 16) <= CtrlArray2Max
then Dev_Sel <= '1';
elsif ODFifoEmpty = '0' and ClkDiv = 0 and DacLd = '1' and Octal_Shift = Idle 
           and ((ODFifoOut(27 downto 16) >= AFE0ArrayMin and ODFifoOut(27 downto 16) <=AFE0ArrayMax)
				 or (ODFifoOut(27 downto 16) >= AFE1ArrayMin and ODFifoOut(27 downto 16) <=AFE1ArrayMax))
then Dev_Sel <= '0';
end if;

--(Idle,Shift,ClearSync,SetLoad);
Case Octal_Shift is
	   When Idle => 	
				if ODFifoEmpty = '0' and ClkDiv = 0 and DacLd = '1' then Octal_Shift <= Shift;
				else Octal_Shift <= Idle;
				end if;
		When Shift => if BitCount = 1 and ClkDiv = 0 then Octal_Shift <= ClearSync;
						 else Octal_Shift <= Shift;
						 end if;
	   When ClearSync => if DacCS = 7 and Dev_Sel = '1' and ClkDiv = 0 and Dev_Sel = '1' then Octal_Shift <= SetLoad;
						       elsif Dev_Sel = '0' and ClkDiv = 0 then Octal_Shift <= Idle;
						       else Octal_Shift <= ClearSync;
						      end if;
		When SetLoad => if DacLd = '0' and ClkDiv = 0 then Octal_Shift <= Idle;
					   else Octal_Shift <= SetLoad;
						end if;
		When others => Octal_Shift <= Idle;
 end Case;

-- DAC output shift registter
if ODFifoRdReq = '1' and Dev_Sel = '1' and ODFifoOut(27 downto 16) < DatArray2Min
	then DACShift <= X"00" & '0' & ODFifoOut(18 downto 16) & ODFifoOut(11 downto 0);
elsif ODFifoRdReq = '1' and Dev_Sel = '1' and ODFifoOut(27 downto 16) >= DatArray2Min 
														and ODFifoOut(27 downto 16) < CtrlArray0Min
	then 
-- Remap the LED intensity channels
	Case ODFifoOut(18 downto 16) is 
	 when "000" =>	DACShift <= X"001" & ODFifoOut(11 downto 0);
	 when "001" =>	DACShift <= X"003" & ODFifoOut(11 downto 0);
	 when "010" =>	DACShift <= X"000" & ODFifoOut(11 downto 0);
	 when "011" =>	DACShift <= X"002" & ODFifoOut(11 downto 0);
	 when others => DACShift <= X"00" & '0' & ODFifoOut(18 downto 16) & ODFifoOut(11 downto 0);
	end Case;
elsif ODFifoRdReq = '1' and Dev_Sel = '1' and ODFifoOut(27 downto 16) >= CtrlArray0Min
	then DACShift <= X"00" & '1' & ODFifoOut(14 downto 0);
elsif ODFifoRdReq = '1' and Dev_Sel = '0'
	then DACShift <= ODFifoOut(23 downto 0);
elsif BitCount /= 0 and ClkDiv = 0 and Octal_Shift = Shift then DACShift <= (DACShift(22 downto 0) & '0');
end if;

if Dev_Sel = '1' and Octal_Shift = Shift then DacDat <= DACShift(15);
else DacDat <= '0';
end if;

---- When one word has been serialized, read the buffer for the next word
if ODFifoEmpty = '0' and ClkDiv = 0 and DacLd = '1' and Octal_Shift = Idle
then ODFifoRdReq <= '1';
else ODFifoRdReq <= '0';
end if;

---- Serial bit counter
-- if destination is DAC, counter is 24
if ODFifoRdReq = '1' and BitCount = 0 and Dev_Sel = '0' 
then BitCount <= "11000";
-- if destination is AFE, counter is 16
elsif ODFifoRdReq = '1' and BitCount = 0 and Dev_Sel = '1' 
then BitCount <= "10000";
elsif BitCount /= 0 and Octal_Shift = Shift and ClkDiv = 0 then BitCount <= BitCount - 1;
end if;

---- DAC clock
if Dev_Sel = '1' and BitCount /= 0 and Octal_Shift = Shift then DacClk <= ClkDiv(2); 
else DacClk <= '0';
end if;

---- DAC load 
if Octal_Shift = SetLoad and ClkDiv = 0 and Dev_Sel = '1' then DacLd <= '0';
elsif Octal_Shift = Idle and ClkDiv = 0 then DacLd <= '1';
else DacLd <= DacLd;
end if;

---- Assert appropriate DAC chip select based on bits 22..19 of the FIFO output
if ODFifoRdReq = '1' and BitCount = 0 and Dev_Sel = '1'  
then 
 Case ODFifoOut(22 downto 19) is
	When X"6" => DacCS <= "110";
	When X"7" => DacCS <= "101";
	When X"8" => DacCS <= "011";
	When X"9" => DacCS <= "110";
	When X"A" => DacCS <= "101";
	When X"B" => DacCS <= "011";
	When others => DacCS <= "111";
 end case;
elsif ClkDiv = 0 and Octal_Shift = ClearSync 
then DacCS <= "111";
end if;

-- Assert appropriate AFE chip select based on bits 25, 24 of the FIFO output
if ODFifoRdReq = '1' and BitCount = 0 and Dev_Sel = '0'  
then 
 Case ODFifoOut(25 downto 24) is
	When "01" => AFECS <= "10";
	When "10" => AFECS <= "01";
	When others => AFECS <= "11";
 end case;
elsif ClkDiv = 0 and Octal_Shift = ClearSync 
then AFECS <= "11";
end if;

---- AFE serial clock
if Dev_Sel = '0' and BitCount /= 0 and Octal_Shift = Shift 
then AFESClk <= ClkDiv(2); 
else AFESClk <= '0';
end if;

---- AFE serial data
if Dev_Sel = '0' and Octal_Shift = Shift then AFESDI <= DACShift(23); 
else AFESDI <= '0';
end if;

-- Delayed copy of the serial clock for strobing in readback data
SClkDL(0) <= AFESClk;
SClkDL(1) <= SClkDL(0);
SClkDL(2) <= SClkDL(1);

-- Clock in any readback data
if Dev_Sel = '0' and SClkDL = 6 
then AFERdReg <= AFERdReg(14 downto 0) & AFESDO;
end if;

end if; -- CpldRst

end process Clk100;

Temp(0) <= '0' when TempEn = '1' and TempCtrl = "0001" else 'Z';
Temp(1) <= '0' when TempEn = '1' and TempCtrl = "0010" else 'Z';
Temp(2) <= '0' when TempEn = '1' and TempCtrl = "0100" else 'Z';
Temp(3) <= '0' when TempEn = '1' and TempCtrl = "1000" else 'Z';

------------------- mux for reading back registers -------------------------

DDRRd_Mux <= SDRdDat(31 downto 16) when RdHi_LoSel = '0' else SDRdDat(15 downto 0);

with uCA(9 downto 0) select

iCD <= X"000" & "00" & AFEPDn when CSRRegAddr,
		 X"000" & "00" & AlignReq when SlipCtrlAd,
		 X"0" & '0' & SlipCntReg(1)
			& X"0" & '0' & SlipCntReg(0) when SlipCntRegAd,
		 X"00" & WidthReg when GateAddr,
		 DRAMRdBuffOut when PageFIFOAddr,
		 X"0" & '0' & DRAMRdBuffWdsUsed when PageFIFOWdsAd,
		 X"0" & Read_Seq_Stat & X"0" & '0' &  DDRWrtSeqStat when WriteSeqStatAd,
		 X"00" & PipelineSet when PipeLineAddr,
		 X"00" & "000" & MuxSelReg & MuxadReg when MuxCtrlAd,
		 MaskReg(1) & MaskReg(0) when InputMaskAddr,
		 In_Seq_Stat(1)(7 downto 0) & In_Seq_Stat(0)(7 downto 0) when InseqStatAd,
		 X"000" & '0' & LEDSrc & PulseSel & FlashEn when FlashCtrlAddr,
		 UpTimeStage(31 downto 16) when UpTimeRegAddrHi,
		 UpTimeStage(15 downto 0) when UpTimeRegAddrLo,
		 TestCount(31 downto 16) when TestCounterHiAd,
		 TestCount(15 downto 0) when TestCounterLoAd,
		 X"000" & "00" & FMTxBuff_full & FMTxBuff_empty when LVDSTxFIFOStatAd,
		 X"0" & BeamOnLength when BeamOnLengthAd,
		 X"0" & BeamOffLength when BeamOffLengthAd,
		 "0000000" & TurnOnTime when OnTimeAddr,
		 "0000000" & TurnOffTime when OffTimeAddr,
 		 "0000000" & LEDTime when LEDTimeAddr,
		 "00" & SDWrtAd(29 downto 16) when SDRamWrtPtrHiAd,
		 SDWrtAd(15 downto 0) when SDRamWrtPtrLoAd,
		 "00" & SDRdAD(29 downto 16) when SDRamRdPtrHiAd,
		 SDRdAD(15 downto 0) when SDRamRdPtrLoAd,
		 DDRRd_Mux(7 downto 0) & DDRRd_Mux(15 downto 8) when SDRamSwapPort,
		 DDRRd_Mux when SDRamPortAd,
		 AFERdReg when AFERdDataAd,
		 '0' & DDR_Rd_Cnt & '0' & SDwr_count when DDRCountAddr,
		 X"0" & '0' & SDrd_empty & SDrd_full & SDcmd_empty(1) & SDcmd_full(1) 
		 & SDwr_underrun & SDwr_empty & SDwr_full & SDcmd_empty(0) & SDcmd_full(0) 
		 & SDCalDn & SD_RstO when DDRStatAddr,
		 HistInterval when HistIntvalAd,
		 X"00" & '0' & HistEnReq(1) & HistEnReq(0) & HistMode & '0' & HistChan when HistCtrlAd,
 		 X"0" & std_logic_vector(Hist_Offset_Reg) when HistOfstAd,
		 X"0" & '0' & HistAddrb(0) when HistPtrAd0,
		 X"0" & '0' & HistAddrb(1) when HistPtrAd1,
		 Hist_Outb(0) when HistRd0Ad,
		 Hist_Outb(1) when HistRd1Ad,
		 GA & "00" & X"001" when DebugVersionAd,
		 X"0" & std_logic_vector(Ped_Reg(0)(0)) when PedRegAddr(0)(0),
		 X"0" & std_logic_vector(Ped_Reg(0)(1)) when PedRegAddr(0)(1),
		 X"0" & std_logic_vector(Ped_Reg(0)(2)) when PedRegAddr(0)(2),
		 X"0" & std_logic_vector(Ped_Reg(0)(3)) when PedRegAddr(0)(3),
		 X"0" & std_logic_vector(Ped_Reg(0)(4)) when PedRegAddr(0)(4),
		 X"0" & std_logic_vector(Ped_Reg(0)(5)) when PedRegAddr(0)(5),
		 X"0" & std_logic_vector(Ped_Reg(0)(6)) when PedRegAddr(0)(6),
		 X"0" & std_logic_vector(Ped_Reg(0)(7)) when PedRegAddr(0)(7),
		 X"0" & std_logic_vector(Ped_Reg(1)(0)) when PedRegAddr(1)(0),
		 X"0" & std_logic_vector(Ped_Reg(1)(1)) when PedRegAddr(1)(1),
		 X"0" & std_logic_vector(Ped_Reg(1)(2)) when PedRegAddr(1)(2),
		 X"0" & std_logic_vector(Ped_Reg(1)(3)) when PedRegAddr(1)(3),
		 X"0" & std_logic_vector(Ped_Reg(1)(4)) when PedRegAddr(1)(4),
		 X"0" & std_logic_vector(Ped_Reg(1)(5)) when PedRegAddr(1)(5),
		 X"0" & std_logic_vector(Ped_Reg(1)(6)) when PedRegAddr(1)(6),
		 X"0" & std_logic_vector(Ped_Reg(1)(7)) when PedRegAddr(1)(7),
		 X"0" & std_logic_vector(IntTrgThresh(0)(0)) when ThreshRegAddr(0)(0),
		 X"0" & std_logic_vector(IntTrgThresh(0)(1)) when ThreshRegAddr(0)(1),
		 X"0" & std_logic_vector(IntTrgThresh(0)(2)) when ThreshRegAddr(0)(2),
		 X"0" & std_logic_vector(IntTrgThresh(0)(3)) when ThreshRegAddr(0)(3),
		 X"0" & std_logic_vector(IntTrgThresh(0)(4)) when ThreshRegAddr(0)(4),
		 X"0" & std_logic_vector(IntTrgThresh(0)(5)) when ThreshRegAddr(0)(5),
		 X"0" & std_logic_vector(IntTrgThresh(0)(6)) when ThreshRegAddr(0)(6),
		 X"0" & std_logic_vector(IntTrgThresh(0)(7)) when ThreshRegAddr(0)(7),
		 X"0" & std_logic_vector(IntTrgThresh(1)(0)) when ThreshRegAddr(1)(0),
		 X"0" & std_logic_vector(IntTrgThresh(1)(1)) when ThreshRegAddr(1)(1),
		 X"0" & std_logic_vector(IntTrgThresh(1)(2)) when ThreshRegAddr(1)(2),
		 X"0" & std_logic_vector(IntTrgThresh(1)(3)) when ThreshRegAddr(1)(3),
		 X"0" & std_logic_vector(IntTrgThresh(1)(4)) when ThreshRegAddr(1)(4),
		 X"0" & std_logic_vector(IntTrgThresh(1)(5)) when ThreshRegAddr(1)(5),
		 X"0" & std_logic_vector(IntTrgThresh(1)(6)) when ThreshRegAddr(1)(6),
		 X"0" & std_logic_vector(IntTrgThresh(1)(7)) when ThreshRegAddr(1)(7),
		 X"000" & "00" & TrgSrc & '0' when TrigCtrlAddr,
		 X"000" & ADCSmplCntReg when ADCSmplCntrAd,
		 X"000" &"00" & SlfTrgEn & TmgSrcSel when IntTrgEnAddr,
		 "000" & ControllerNo & "000" & PortNo when FEBAddresRegAd,
		 "00" & FRDat(0) & "00" & FRDat2(0) when FRDat0RegAd,
		 "00" & FRDat(1) & "00" & FRDat2(1) when FRDat1RegAd,
		 uBunch(15 downto  0)        when uBLoAd,
		 uBunch(31 downto 16)        when uBHiAd,
		 uBunchBuffOut(15 downto  0) when uBBuffLoAd,
		 uBunchBuffOut(31 downto 16) when uBBuffHiAd,
		 DDRAddrOut(15 downto  0)    when uBBuffAdLoAd,
		 DDRAddrOut(31 downto 16)    when uBBuffAdHiAd,
		 X"0000" when others;

-- Select between DAC readback and the rest of the registers
uCD <= iCD when uCRd = '0' and CpldCS = '0' and uCA(11 downto 10) = GA 
   and (uCA(9 downto 0) < DatArray0Min or uCA(9 downto 0) > CtrlArray2Max)
	and (uCA(9 downto 0) < OneWireCmdAd or uCA(9 downto 0) > TempDat4Ad)
	and uCA(9 downto 0) /= PageStatAddr
else ShadowOut when uCRd = '0' and CpldCS = '0' and uCA(11 downto 10) = GA 
   and uCA(9 downto 0) >= DatArray0Min and uCA(9 downto 0) <= CtrlArray2Max
else One_Wire_Out when uCRd = '0' and CpldCS = '0' and uCA(11 downto 10) = GA 
   and uCA(9 downto 0) >= OneWireCmdAd and uCA(9 downto 0) <= TempDat4Ad
-- Contrive to to show status bits from 4 FPGAs with a read from a single address.
else 'Z'&'Z'&'Z'&'Z'&'Z'&'Z'&'Z'&'Z'&'Z'&'Z'&'Z'& DRAMRdBuffEmpty &'Z'&'Z'&'Z'  
		& PageRdStat when uCRd = '0' and CpldCS = '0' and GA = "00" and uCA(9 downto 0) = PageStatAddr
else 'Z'&'Z'&'Z'&'Z'&'Z'&'Z'&'Z'&'Z'&'Z'&'Z'& DRAMRdBuffEmpty &'Z'&'Z'&'Z'
	   & PageRdStat &'Z' when uCRd = '0' and CpldCS = '0' and GA = "01" and uCA(9 downto 0) = PageStatAddr 
else 'Z'&'Z'&'Z'&'Z'&'Z'&'Z'&'Z'&'Z'&'Z'& DRAMRdBuffEmpty &'Z'&'Z'&'Z'
		& PageRdStat &'Z'&'Z' when uCRd = '0' and CpldCS = '0' and GA = "10" and uCA(9 downto 0) = PageStatAddr 
else 'Z'&'Z'&'Z'&'Z'&'Z'&'Z'&'Z'&'Z'& DRAMRdBuffEmpty &'Z'&'Z'&'Z'
		& PageRdStat &'Z'&'Z'&'Z' when uCRd = '0' and CpldCS = '0' and GA = "11" and uCA(9 downto 0) = PageStatAddr 
else (others => 'Z');

end behavioural;

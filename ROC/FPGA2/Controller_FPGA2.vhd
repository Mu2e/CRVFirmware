-- Firmware for Mu2e Controller FPGA 2

-- Sten Hansen Fermilab 10/26/2015

-- FPGA responsible for handling data from eight Ethernet PHY chips,
-- and eight LVDS I/Os
-- Collects data from eight phy chips and sends it to FPGA 1 via an 800 Mbit
-- LVDS Link. A 512Mb LPDDR RAM is available as a data buffer
-- Microcontroller interface

----------------------------- Main Body of design -------------------------

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;

Library UNISIM;
use UNISIM.vcomponents.all;

use work.Proj_Defs.all;

entity Controller_FPGA2 is port(

-- 160 MHz VXO clock, Phy clocks
	VXO_P,VXO_N,ClkB_P,ClkB_N,Clk50MHz,
-- microcontroller strobes
	CpldRst, CpldCS, uCRd, uCWr : in std_logic;
-- microcontroller data, address buses
	uCA : in std_logic_vector(11 downto 0);
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
-- Link serial transmitter signals
	LinkClk_P,LinkClk_N,LinkFR_P,LinkFR_N : out  std_logic;
	LinkD_P,LinkD_N : out std_logic_vector(1 downto 0);
-- Ethernet PHY Signals
	RxDA,RxDB,RxDC,RxDD,RxDE,RxDF,RxDG,RxDH : in std_logic_vector(3 downto 0);
	RxClk,RxDV,RxErr,CRS : in std_logic_vector(7 downto 0);
	TxDA,TxDB,TxDC,TxDD,TxDE,TxDF,TxDG,TxDH : out std_logic_vector(3 downto 0);
	TxEn : buffer std_logic_vector(7 downto 0);
	MDC : buffer std_logic_vector(1 downto 0);
	MDIO : inout std_logic_vector(1 downto 0);
	PhyPDn,PhyRst : buffer std_logic;
-- Two of eight TxClk chips from the PHYs are connected to the FPGA
	TxClk : in std_logic_vector(1 downto 0);
	Clk25MHz : buffer std_logic;
-- LVDS receivers
	FMRx : in std_logic_vector(7 downto 0);
-- Chip enable for octal LVDS receiver
	FMRxEn : buffer std_logic;
--- Heart beat data, asynchronous packets from top level FPGA
	HrtBtFM,DReqFM : in std_logic;
-- LVDS driver SPI port
	SPICS,SPISClk,SPIMOSI : buffer std_logic;
	SPIMISO : in std_logic;
-- Debug port
	Debug : buffer std_logic_vector(10 downto 1)
);

end Controller_FPGA2;

architecture behavioural of Controller_FPGA2 is

---------------------- Signal declarations -----------------------

Type Array_2x2 is Array(0 to 1) of std_logic_vector (1 downto 0);
Type Array_2x16 is Array(0 to 1) of std_logic_vector(15 downto 0);
Type Array_8x2 is Array(0 to 7) of std_logic_vector(1 downto 0);
Type Array_8x3 is Array(0 to 7) of std_logic_vector(2 downto 0);
Type Array_8x4 is Array (0 to 7) of std_logic_vector(3 downto 0);
Type Array_8x11 is Array(0 to 7) of std_logic_vector(10 downto 0);
Type Array_8x12 is Array(0 to 7) of std_logic_vector(11 downto 0);
Type Array_8x13 is Array(0 to 7) of std_logic_vector(12 downto 0);
Type Array_8x14 is Array(0 to 7) of std_logic_vector(13 downto 0);
Type Array_8x16 is Array(0 to 7) of std_logic_vector(15 downto 0);
Type Array_8x32 is Array(0 to 7) of std_logic_vector(31 downto 0);
Type Array_3x8x16 is Array(0 to 2) of Array_8x16;

-- Clock and reset signals
signal SysClk, i50MhzClk,ResetHi,GTPRxRst,LinkRst : std_logic;
-- Synchronous edge detectors of uC read and write strobes
Signal RDDL,WRDL : std_logic_vector (1 downto 0);
-- uC data bus
signal iCD,CDStage : std_logic_vector(15 downto 0);
signal AddrReg : std_logic_vector(11 downto 0);
-- TClk begin spill, end spill events
signal TrigCtrlReg : std_logic_vector(1 downto 0);

-- Timing interval counters
signal Counter1us : std_logic_vector (7 downto 0);
signal Counter10us : std_logic_vector (9 downto 0);
signal Counter1ms : std_logic_vector (16 downto 0);
signal Counter1s : std_logic_vector (27 downto 0);

signal Seq_Busy,FlashEn,TempEn : std_logic; 

signal StatReg : std_logic_vector (3 downto 0);

-- Count the number of triggers
-- Make a test counter that increments with each read
signal TestCount : std_logic_vector (31 downto 0);
-- Uptime counter to check for un-anticipated resets
signal UpTimeCount,UpTimeStage : std_logic_vector (31 downto 0);

-- Spill counter, event word cout, spill word count

signal GPIDL,TrigDL,iWrtDL  : Array_2x2;
signal GateReq : std_logic_vector (1 downto 0);
-- Test Pulse generator signals
signal Freq_Reg,PhaseAcc : std_logic_vector (31 downto 0);

-- MIG LPDDR controller signals 
signal AuxClk : std_logic;
signal SDRdDat,SDWrtDat : std_logic_vector(31 downto 0);
signal DDRRd_Mux : std_logic_vector(15 downto 0);
signal SDWrtAd,SDWrtAdStage,SDRdAD,SDRdPtr : std_logic_vector(29 downto 0);
signal WrtAddrReg,AddrBuff_Out : std_logic_vector(27 downto 0); 
signal AddrBuff_wren,AddrBuff_rden,AddrBuff_full,AddrBuff_empty : std_logic;

signal SDwr_en,SDrd_en,SDCalDn,WrtCmdEn,SDRdCmdEn,SD_RstO,DDR_Reset : std_logic;
signal SDwr_full,SDwr_empty,SDwr_error,SDrd_full,SDrd_empty,
		 SDrd_overflow,SDrd_error,SDwr_underrun,RdHi_LoSel,
		 FifoRdD, WrtHi_LoSel : std_logic;
signal ResetCount,DDRWrtStat : std_logic_vector(3 downto 0);
signal SDrd_enD : std_logic_vector(2 downto 0);
constant RdBrstSiz  : std_logic_vector(5 downto 0) := "000111";
constant WrtBrstSiz : std_logic_vector(5 downto 0) := "000111";
signal SDwr_count,DDR_Rd_Cnt : std_logic_vector(6 downto 0);
signal SDWrtCmd,SDRdCmd : std_logic_vector(2 downto 0);
signal SDcmd_empty,SDcmd_full : std_logic_vector(1 downto 0);

-- Signals used by DDR write sequencer
signal PortNo : Integer range 0 to 7; 
signal MaskReg : std_logic_vector(7 downto 0);
signal EventWdCnt,EventStat : std_logic_vector (15 downto 0);
signal uBunch : std_logic_vector(31 downto 0);
signal PortWdCounter : Array_8x16;
signal EventRdy,FirstActive : std_logic;
signal PhyActivityCounter: Array_8x16;

Type Write_Seq_FSM is (Idle,ChkWrtBuff,SndCmd,WtCmdMtpy,CheckActive0,IncrPort0,
							  Rd_WdCount,Rd_uBunchHi,Rd_uBunchLo,Rd_Stat,IncrPort1,
							  CheckActive1,ResetPortNo,Write_Wd_Count,Wrt_Stat,Wrt_uBunchHi,
							  Wrt_uBunchLo,WrtDDR,WaitBuff,WritePad,IncrBuffCnt);
signal DDR_Write_Seq : Write_Seq_FSM;

-- Signals used by SDRAM readout sequencer
Type Read_Seq_FSM is (Idle,Wait0,SetAddr,CheckEmpty,FirstCmd,CheckRdBuff0,
							  RdWdCount,CheckWdCount,PrepareWordCnt,CheckRdBuff1,RdDataHi,RdDataLo);
signal DDR_Read_Seq : Read_Seq_FSM;
signal WaitCount : std_logic_vector(5 downto 0);
signal EvWdCount : std_logic_vector(15 downto 0);
signal ReadCount,DDRRdStat,TxBlkCount : std_logic_vector(2 downto 0);
-- Count of event pending for readout

-- SMI signals
signal SMI_Full,SMI_Empty,SMI_wreq,SMI_rdreq,ClkDiv,MDIORd : std_logic;
signal SPI_Count : std_logic_vector (10 downto 0); 
signal Strt,TA,R_W,iMDIO,ChainSel : std_logic_vector (1 downto 0);
signal SMI_Out : std_logic_vector (23 downto 0);
signal PhyAd : std_logic_vector (4 downto 0);
signal BitCount : std_logic_vector (5 downto 0);
signal SMIShift : std_logic_vector (31 downto 0);
signal SMIRdReg0,SMIRdReg1 : std_logic_vector (15 downto 0);
Type  SMI_FSM is (Idle,Load,Shift,Done);
Signal SMI_Shift : SMI_FSM;

-- Clock fanout SPI signals
Signal SPI_Adddr,SPI_Out,SPI_Shift : std_logic_vector (15 downto 0);
Signal SPIDiv : std_logic_vector (2 downto 0);
Signal SPIBitCnt : std_logic_vector (3 downto 0);
Signal SPI_WrtReq,SPI_rdreq,SPI_Full,SPI_Empty : std_logic;
Type  SPI_FSM is (Idle,Load_Addr,Shift_Addr,Shift_Data,Done);
Signal SPI_State : SPI_FSM;

-- Ethernet PHY signals
-- PhyTx signals
signal PhyRstCnt : std_logic_vector (1 downto 0);
signal PhyTxBuff_Full,PhyTxBuff_Empty,PhyTxBuff_rdreq,PreambleTx,DDRRd_en,
		 PhyTxBuff_wreq,TxEnReq,TxEnAck,DDRWrt_En,DDRWrt_EnD,InitReq,PhyDatSel : std_logic;
signal PhyTxBuff_Count : std_logic_vector (10 downto 0);
signal PreambleCnt : std_logic_vector (2 downto 0);
signal TxReg : std_logic_vector (3 downto 0);
signal Preamble,CRCErr_Reg,TxEnMask : std_logic_vector (7 downto 0);
signal PhyTxBuff_Out,PhyTxBuff_Dat : std_logic_vector (15 downto 0);
signal TxNibbleCount : std_logic_vector (1 downto 0);

-- Phy Rx Signals
signal PhyRxBuff_wreq,PhyRxBuff_rdreq,PhyRxBuff_Empty,HitFlag,
		 PhyRxBuff_Full,iCRS,PhyRxBuff_RdStat : std_logic_vector (7 downto 0);
signal RxBuffRst,FMRxBuffRst : std_logic;
Signal PhyRxBuff_RdCnt : Array_8x12;
signal PhyRxBuff_Out : Array_8x16;
signal RxPipeline : Array_3x8x16;
signal RxNibbleCount,RxClkDL,iRxDV : Array_8x2;
signal StartCount : Array_8x3;
signal FMRxBuff_Count : Array_8x11;
signal PhyRx,PhyTx : Array_8x4;
signal Rx_CRC_Out : Array_8x32;
signal RdCRCEn,RxCRCRst : std_logic_vector(7 downto 0);

-- Signal used by timing/trigger LVDS FM receive links
signal RxDat : Array_2x16;
Type Array_OutRec_x2 is Array(0 to 1) of RxOutRec;
signal RxOut : Array_OutRec_x2;
Type Array_InRec_x2 is Array(0 to 1) of RxInRec;
signal RxIn : Array_InRec_x2;
signal i50MHz,RxFMClk,TrigWdCntRst,DRegSrc,LinkFIFOStat : std_logic;
signal TrigWdCount,DReqFMDL : std_logic_vector (3 downto 0);
signal TrigReqCount : std_logic_vector (7 downto 0);
signal DatReqBuff_rdreq,DatReqBuff_Full,DatReqBuff_Empty : std_logic; 
signal DatReqBuff_Count : std_logic_vector (9 downto 0); 
signal DatReqBuff_Out : std_logic_vector (15 downto 0);

-- Signal used by FEB LVDS FM receive links
signal FEBRxBuff_Dat,FEBRxBuff_Out : Array_8x16;
signal FEBRxBuff_wreq,FEBRxBuff_rdreq,FEBRxBuff_Empty,
		 FEBRxBuff_Full,PErrStat : std_logic_vector(7 downto 0);
Type Array_OutRec_x8 is Array(0 to 7) of RxOutRec;
signal FEBRxOut : Array_OutRec_x8;
Type Array_InRec_x8 is Array(0 to 7) of RxInRec;
signal FEBRxIn : Array_InRec_x8;

-- Signals used to determine which ports have FEBs plugged into them
signal RxDl : Array_8x2;
signal TransitionCount : Array_8x4;
signal Rx_active : std_logic_vector(7 downto 0);
signal LinkStatEn : std_logic;
signal LinkTxFullCnt : std_logic_vector(7 downto 0);

-- Serializer signals
signal FrameReg,ClockReg,LinkRegHi,LinkRegLo : std_logic_vector(4 downto 0);
signal TxFIFO_Out : std_logic_vector(8 downto 0);
signal LinkFIFO_Dat : std_logic_vector(17 downto 0);
signal BitClk,WdClk,PllLock,LockOut,Link_Stat_Req : std_logic;
signal LinkTxFull,LinkTxEmpty,LinkTxWrReq,LinkTxRDReq,TxValid,LinkTxTraceWrReq : std_logic;

-- Added 11/24: PHY to FEB signal for prefetch filling
-- Ready notification / edge-detect signals (per PHY port)
signal phy_empty_d    : Array_8x2 := (others => (others => '0'));  -- delayed copy for edge detect
signal ReadyStatus    : std_logic_vector(7 downto 0) := (others => '0');  -- sticky ready bits

-- debug trace buffer
signal LinkTxTraceRDReq : std_logic;
signal TxFIFOTrace_Out : std_logic_vector(8 downto 0);
signal LinkTxTrace_Cnt : std_logic_vector(12 downto 0);

-- SC: signals to handle overflows
signal tx_overflow : std_logic; -- flag indcating if we are sending an event with overflow
signal tx_overflow_cnt : std_logic_vector(15 downto 0); -- overflow counter for diagnostics
--signal tx_word_cnt : std_logic_vector(15 downto 0);
signal word_number : std_logic_vector(1 downto 0); -- shift register used to identify the first (word count) and second (status) words
signal EvWdCountTot : std_logic_vector(15 downto 0); -- latches the total EvWdCount used to send out (for the statemachine we start at +1)
-- TX MAX WORDS
-- we can at least try to cut at hit boundaries
-- the header is 4 words: count, status, uB-low, ub-high
-- each hit currently has 11 words
-- So we want to cut somewhere at 4 + N x 11 with integer N
-- N=1: 11*1 + 4 = 15 = 0x000f, for debugging, testing
-- N=744: 11*744 + 4 = 8188 = 0x1ffc
constant MAX_TX_WORDS : std_logic_vector(15 downto 0) := X"1ffc";-- X"2000";
-- constant UB_MISMATCH_STATUS_BIT : std_logic_vector(7 downto 0) := X"10";
constant OVERFLOW_STATUS_BIT : std_logic_vector(15 downto 0) := X"1000"; -- bit 12

-- added 11/24: Auto-TX FPGA automatic READY Packet sender, 2 words for now? Can change later. 
signal PhyTxDin_FPGA      : std_logic_vector(15 downto 0) := (others => '0'); -- FPGA-produced write data
signal PhyTxDin_mux       : std_logic_vector(15 downto 0) := (others => '0'); -- selected DIN to PhyTx_Buff (uC or FPGA)
signal PhyTxBuff_wr_en_mux: std_logic := '0';                                  -- combined wr_en into PhyTx_Buff
signal PhyTxWrReq_FPGA    : std_logic := '0';                                  -- one-cycle FPGA write request
signal AutoTx_State       : std_logic_vector(1 downto 0) := "00";              -- small FSM state
signal AutoTx_Port        : integer range 0 to 7 := 0;
signal AutoTx_WordIdx     : integer range 0 to 15 := 0; -- supports up to 16-word packets if needed
signal AutoTx_WordPending : std_logic := '0';  -- internal one-cycle writer strobe tracker
signal AutoTx_Claim : std_logic_vector(7 downto 0) := (others => '0'); -- one-hot claim for main to clear ReadyStatus

-- added 11/24: Helper function: ready packet word generator (adjust to your packet format)
function ready_packet_word(port_idx : integer; idx : integer) return std_logic_vector is
begin
  -- Placeholder packet (16-bit words):
  --  word 0 = total word count (including this length word) = 2
  --  word 1 = port id in low bits (0..7)
 if idx = 0 then
    return X"0002";  -- packet length = 2 words (16-bit)
  elsif idx = 1 then
    return std_logic_vector(to_unsigned(port_idx, 16));
  else
    return (others => '0');
  end if;
end function ready_packet_word;

constant READY_WORD_COUNT : integer := 2; -- number of words in the READY packet (update if you change helper)

-- added 11/24
-- helper: returns true if all bits of the std_logic_vector are '0'
function is_all_zero(vec : std_logic_vector) return boolean is
begin
  for i in vec'range loop
    if vec(i) = '1' then
      return false;
	 end if;
  end loop;
  return true;
end function is_all_zero;

begin

-- You can't use type defs in the pin list. Remap type def elements to 
-- separate std_logic_vectors
PhyRx(0) <= RxDA; TxDA <= PhyTx(0);
PhyRx(1) <= RxDB; TxDB <= PhyTx(1);
PhyRx(2) <= RxDC; TxDC <= PhyTx(2);
PhyRx(3) <= RxDD; TxDD <= PhyTx(3);
PhyRx(4) <= RxDE; TxDE <= PhyTx(4);
PhyRx(5) <= RxDF; TxDF <= PhyTx(5);
PhyRx(6) <= RxDG; TxDG <= PhyTx(6);
PhyRx(7) <= RxDH; TxDH <= PhyTx(7);

SysPLL : Sys_PLL
  port map
   (-- Clock in ports
    CLK_IN1_P => ClkB_P,
    CLK_IN1_N => ClkB_N,
    -- Clock out ports
    CLK_OUT1 => BitClk, -- 500 MHz serializer bit clock
    CLK_OUT2 => SysClk, -- 100 MHz system clock
    CLK_OUT3 => RxFMClk,-- 200 MHz FM Rx clock
	 CLK_OUT4 => i50MHz,
    -- Status and control signals
    RESET  => ResetHi,
	 LOCKED => PllLock);

-- Serializer to send data at 100MBytes/sec to the top level FPGA
-- The four bits are TX Clock, TX Frame and TxData(1 downto 0)
-- The serialization factor is five. Two five bit words are sent 
-- on two lanes to form a 10 bit result, one bit is used as data valid.

-- Clk    -_-_-_-_-_-_-_-_-_-_
-- Frame  -----_____-----_____
-- Lane 1 V1DDDV1dddV1DDDV1ddd
-- Lane 0 DDDDDdddddDDDDDddddd

FPGALinkTx : LinkTx
generic map ( sys_w => 4,   -- width of the data for the system
				  dev_w => 20)  -- width of the data for the device
port map ( -- From the device out to the system
  DATA_OUT_FROM_DEVICE(19) => ClockReg(0),
  DATA_OUT_FROM_DEVICE(18) => FrameReg(0),
  DATA_OUT_FROM_DEVICE(17) => LinkRegHi(0),
  DATA_OUT_FROM_DEVICE(16) => LinkRegLo(0),
  DATA_OUT_FROM_DEVICE(15) => ClockReg(1),
  DATA_OUT_FROM_DEVICE(14) => FrameReg(1),
  DATA_OUT_FROM_DEVICE(13) => LinkRegHi(1),
  DATA_OUT_FROM_DEVICE(12) => LinkRegLo(1),
  DATA_OUT_FROM_DEVICE(11) => ClockReg(2),
  DATA_OUT_FROM_DEVICE(10) => FrameReg(2),
  DATA_OUT_FROM_DEVICE(9) => LinkRegHi(2),
  DATA_OUT_FROM_DEVICE(8) => LinkRegLo(2),
  DATA_OUT_FROM_DEVICE(7) => ClockReg(3),
  DATA_OUT_FROM_DEVICE(6) => FrameReg(3),
  DATA_OUT_FROM_DEVICE(5) => LinkRegHi(3),
  DATA_OUT_FROM_DEVICE(4) => LinkRegLo(3),
  DATA_OUT_FROM_DEVICE(3) => ClockReg(4),
  DATA_OUT_FROM_DEVICE(2) => FrameReg(4),
  DATA_OUT_FROM_DEVICE(1) => LinkRegHi(4),
  DATA_OUT_FROM_DEVICE(0) => LinkRegLo(4),
  DATA_OUT_TO_PINS_P(3) => LinkClk_P,
  DATA_OUT_TO_PINS_P(2) => LinkFR_P,
  DATA_OUT_TO_PINS_P(1 downto 0) => LinkD_P,
  DATA_OUT_TO_PINS_N(3) => LinkClk_N,
  DATA_OUT_TO_PINS_N(2) => LinkFR_N,
  DATA_OUT_TO_PINS_N(1 downto 0) => LinkD_N,
-- Clock and reset signals
  CLK_IN  => BitClk,      -- Fast clock from PLL/MMCM 
  CLK_DIV_IN => SysClk,   -- Slow clock from PLL/MMCM
  LOCKED_IN => PllLock,
  LOCKED_OUT => LockOut,
  IO_RESET => LinkRst);  -- Reset signal for IO circuit

-- MIG DDR controller
LPDDRCtrl : LPDDR_Ctrl
 generic map(
    C3_P0_MASK_SIZE => 4, C3_P0_DATA_PORT_SIZE => 32,
    C3_P1_MASK_SIZE => 4, C3_P1_DATA_PORT_SIZE => 32,
    C3_MEMCLK_PERIOD => 6277,  C3_RST_ACT_LOW => 0,
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

-- Buffer for Link Tx data
-- Data is 18 bits, output is 9 bits
LinkBuff : LinkTxFIFO
  port map (rst => ResetHi,
	 wr_clk => SysClk, rd_clk => SysClk,
    wr_en => LinkTxWrReq, rd_en => LinkTxRDReq, 
	 din => LinkFIFO_Dat,
    dout => TxFIFO_Out,
    full => LinkTxFull, empty => LinkTxEmpty);
	 
-- copy of the same buffer as trace buffer for debug purpose
-- REMOVE ME
LinkBuffTrace : LinkTxFIFOTrace
  port map (rst => ResetHi,
	 wr_clk => SysClk, rd_clk => SysClk,
    wr_en => LinkTxTraceWrReq, rd_en => LinkTxTraceRDReq, 
	 din => LinkFIFO_Dat,
    dout => TxFIFOTrace_Out,
    full => open, empty => open,
	 rd_data_count => LinkTxTrace_Cnt);

LinkRegHi(4) <= TxValid and not LinkTxEmpty;
-- Use this as a Rx_Active data flag
LinkRegHi(3) <= TxFIFO_Out(8);
LinkRegHi(2 downto 0) <= TxFIFO_Out(7 downto 5) when LinkTxEmpty = '0' else "100";
LinkRegLo <= TxFIFO_Out(4 downto 0) when LinkTxEmpty = '0' else "00110";

-- Buffer for MDIO data
SMI_Buff : SMI_FIFO
  PORT MAP (wr_clk => SysClk, rst => ResetHi,
	 rd_clk => i50MHz,
	 din(23) => MDIORd,
	 din(22 downto 16) => uCA(6 downto 0),
    din(15 downto 0) => uCD,
    wr_en => SMI_wreq, rd_en => SMI_rdreq,
    dout => SMI_Out, full => SMI_Full,
    empty => SMI_Empty);

-- 1k deep buffer for PhyTx
-- 11/24 modify instantiation 
--    Find the existing block (search for "PhyTx_Buff : PhyTxBuff PORT MAP ( rst => ResetHi,   wr_clk => SysClk,")
--    Replace the PORT MAP arguments `din => uCD, wr_en => PhyTxBuff_wreq,` with the two lines below:
PhyTx_Buff : PhyTxBuff
  PORT MAP ( rst => ResetHi,   wr_clk => SysClk,
    rd_clk => i50MHz, din => PhyTxDin_mux,
    wr_en => PhyTxBuff_wr_en_mux, rd_en => PhyTxBuff_rdreq,
    dout => PhyTxBuff_Out, full => PhyTxBuff_Full,
    empty => PhyTxBuff_Empty,
	 wr_data_count => PhyTxBuff_Count);

-- Added 11/24/25
------------------combinational assignments (mux) and the AutoTx FSM process-------------
PhyTxDin_mux <= uCD when PhyTxBuff_wreq = '1' else PhyTxDin_FPGA;
PhyTxBuff_wr_en_mux <= PhyTxBuff_wreq or PhyTxWrReq_FPGA;


-- Buffer for SPI data
SPITx_Buff : PhyTxBuff
  PORT MAP ( rst => ResetHi, wr_clk => SysClk,
    rd_clk => i50MHz, din => uCD,
    wr_en => SPI_WrtReq, rd_en => SPI_rdreq,
    dout => SPI_Out, full => SPI_Full,
    empty => SPI_Empty,
	 wr_data_count => SPI_Count);

-- FM Receivers for getting data request packets and heartbeat data from
-- the top level FPGA
--DReqFMRx : FM_Rx
--	generic MAP(Pwidth => 16)
--	port map(SysClk => SysClk, RxClk => RxFMClk, 
--				reset => ResetHi,
--				Rx_In => RxIn(0),
--				Data => RxDat(0), 
--				Rx_Out => RxOut(0));
--RxIn(0).FM <= DReqFM when DRegSrc = '1' else '0';

-- Trigger request buffer
--DatReqBuff: SCFifo_512x16
--  PORT MAP ( rst => GTPRxRst, clk => SysClk,
--    din => RxDat(0),
--	 wr_en => RxOut(0).Done, rd_en => DatReqBuff_rdreq,
--    dout => DatReqBuff_Out, 
--	 full => DatReqBuff_Full, empty => DatReqBuff_Empty,
--	 data_count => DatReqBuff_Count);

GTPRxRst <= '1' when CpldRst = '0' 
  	or (CpldCS = '0' and uCWR = '0' and uCA(11 downto 10) = "00" and uCA(9 downto 0) = GTPFIFOAddr and uCD(0) = '1') else '0';

HrtBtRx : FM_Rx
	generic MAP(Pwidth => 16)
	port map(SysClk => SysClk, RxClk => RxFMClk, 
				reset => ResetHi,
				Rx_In => RxIn(1),
				Data => RxDat(1), 
				Rx_Out => RxOut(1));
RxIn(1).FM <= HrtBtFM;

FMRxBuffRst <= '1' when (uCWR = '0' and CpldCS = '0' and AddrReg(11 downto 10) = GA 
						 and AddrReg(9 downto 0) = FMRxErrAddr and uCD(8) = '1') or ResetHi = '1' else '0';

AddrBuff : SCFIFO_1Kx28
  PORT MAP(rst => ResetHi, clk => SysClk,
    din => WrtAddrReg,
    wr_en => AddrBuff_wren,
    rd_en => AddrBuff_rden, 
    dout => AddrBuff_Out,
    full => AddrBuff_full,
    empty => AddrBuff_empty);

----------------------------------------------------------------------------
---- Loop through eight Phy receive channels, eight FM receive channels ----
----------------------------------------------------------------------------

Debug(3) <= FEBRxBuff_Empty(0);
Debug(4) <= FEBRxOut(0).Done;
Debug(5) <= FEBRxBuff_rdreq(0);
Debug(6) <= FMRx(0);

-- 2 BRAM based
Gen_FEBRxBuffs : for i in 0 to 7 generate
-- 4k deep input buffers for eight Rx Phy channels
FEBRx_Buff : PhyRxBuff
  PORT MAP (rst => RxBuffRst, rd_clk => SysClk,
    wr_clk => RxFMClk, din => RxPipeline(2)(i),
    wr_en => PhyRxBuff_wreq(i), rd_en => PhyRxBuff_rdreq(i),
    dout => PhyRxBuff_Out(i), full => PhyRxBuff_Full(i),
    empty => PhyRxBuff_Empty(i),
	 rd_data_count => PhyRxBuff_RdCnt(i));
end generate;

-- 2 DRAM based
--Gen_FEBRxBuffs_test : for i in 5 to 7 generate

--FEBRx_Buff_test : FEBRx_test_Buff
  --PORT MAP (rst => RxBuffRst, rd_clk => SysClk,
    --wr_clk => RxFMClk, din => RxPipeline(2)(i),
--    wr_en => PhyRxBuff_wreq(i), rd_en => PhyRxBuff_rdreq(i),
--    dout => PhyRxBuff_Out(i), full => PhyRxBuff_Full(i),
--    empty => PhyRxBuff_Empty(i),
--	 rd_data_count => PhyRxBuff_RdCnt(i));
--	 
--end generate;


Gen_RxBuffs : for i in 0 to 7 generate

-- CRC generators for receive data CRC checking. 
RxCRC : CRC32_D4 
 port map(rst => RxCRCRst(i), clk => RxFMClk, crc_en => RdCRCEn(i), 
			 data_in => PhyRx(i),
			 crc_out => Rx_CRC_Out(i));





-- FM receivers for FEB FM links
FEBFMRx : FM_Rx
	generic MAP(Pwidth => 16)
	port map(SysClk => SysClk, RxClk => RxFMClk, 
				reset => ResetHi,
				Rx_In => FEBRxIn(i),
				Data => FEBRxBuff_Dat(i), 
				Rx_Out => FEBRxOut(i));

FEBRxIn(i).FM <= FMRx(i);
PErrStat(i) <= FEBRxOut(i).Parity_Err;

-- 1k deep input buffers for eight FM Rx links
FMRx_Buff : SCFIFO1Kx16
  PORT MAP ( rst => FMRxBuffRst, clk => SysClk,
    din => FEBRxBuff_Dat(i),
    wr_en => FEBRxOut(i).Done, rd_en => FEBRxBuff_rdreq(i),
    dout => FEBRxBuff_Out(i), full => FEBRxBuff_Full(i),
    empty => FEBRxBuff_Empty(i),
	 data_count => FMRxBuff_Count(i));

PhyRx_Proc : process(CpldRst, RxFMClk)

begin

 if CpldRst = '0' then

    PhyRxBuff_wreq(i) <= '0'; RxClkDL(i) <= "00";
	 RxNibbleCount(i) <= "00"; CRCErr_Reg(i) <= '0';
	 RxPipeline(0)(i) <= (others => '0');
	 RxPipeline(1)(i) <= (others => '0');
	 RxPipeline(2)(i) <= (others => '0');
	 iCRS(i) <= '0'; iRxDV(i) <= "00"; 
	 RdCRCEn(i) <= '0'; RxCRCRst(i) <= '1';
	 StartCount(i) <= "000";
	 PhyActivityCounter(i) <= (others => '0');

 elsif rising_edge(RxFMClk) then
 
-- PhyActivityCounter
   if PhyRxBuff_wreq(i) = '1' then 
	    PhyActivityCounter(i) <= PhyActivityCounter(i) + 1;
	else
		 PhyActivityCounter(i) <= PhyActivityCounter(i);
	end if;
	     

-- Synchronous edge detector for 25MHz PhyRx clock
	RxClkDL(i)(0) <= RxClk(i);
	RxClkDL(i)(1) <= RxClkDL(i)(0);

-- Registered copies of the carrier sense and data valid signals
	iRxDV(i)(0) <= RxDV(i); iCRS(i) <= CRS(i);
	iRxDV(i)(1) <= iRxDV(i)(0);

-- CRC Error register
	if iRxDV(i) = 2 and Rx_CRC_Out(i) /= X"C704DD7B" then
	CRCErr_Reg(i) <= '1';
-- Writing a '1' to the appropriate location will clear the error bit
	elsif uCWR = '0' and CpldCS = '0' and uCA(11 downto 10) = GA and uCA(9 downto 0) = CRCErrAddr
		and uCD(i) = '1' then CRCErr_Reg(i) <= '0';
	else CRCErr_Reg(i) <= CRCErr_Reg(i);
	end if;

-- Modulo four counter used to assemble nibbles into words
-- Increment nibble count while DAV is high
   if iRxDV(i)(0) = '1' and iCRS(i) = '1' and RxClkDL(i) = 2
   then RxNibbleCount(i) <= RxNibbleCount(i) + 1;  
   elsif RxBuffRst = '1' or RxDV(i) = '0' then RxNibbleCount(i) <= "00"; 
   else RxNibbleCount(i) <= RxNibbleCount(i); 
   end if;

-- Reset the CRC generator during the preamble
	if StartCount(i) = 1 and RxClkDL(i) = 2 and RxNibbleCount(i) = 2
	then RxCRCRst(i) <= '1';
	else RxCRCRst(i) <= '0';
	end if;

-- Enable the CRC generator for one clock tick per nibble
	if StartCount(i) > 3 and RxClkDL(i) = 2  then RdCRCEn(i) <= '1'; 
	else RdCRCEn(i) <= '0'; 
	end if;

-- Data valid stays true for eight nibbles during the CRC postamble. We want
-- only payload to go to the receive FIFO. Delay the data by two three word
-- periods with a pipeline. Load the first pipeline resigter one nibble at a time
  if RxClkDL(i) = 2 then
  Case RxNibbleCount(i) is
	When "00" => RxPipeline(0)(i)(3 downto 0) <= PhyRx(i);
	When "01" => RxPipeline(0)(i)(7 downto 4) <= PhyRx(i);
	When "10" => RxPipeline(0)(i)(11 downto 8) <= PhyRx(i);
	When "11" => RxPipeline(0)(i)(15 downto 12) <= PhyRx(i);
	When others => RxPipeline(0)(i) <= RxPipeline(0)(i);
	end Case;
	else RxPipeline(0)(i) <= RxPipeline(0)(i);
	end if;

-- After each group of four nibbles has arrived, advance the data 
-- through the pipeline 
 if iRxDV(i)(0) = '1' and iCRS(i) = '1' and RxClkDL(i) = 2
		and RxNibbleCount(i) = 0 
  then RxPipeline(1)(i) <= RxPipeline(0)(i); 
		 RxPipeline(2)(i) <= RxPipeline(1)(i); 
  else RxPipeline(1)(i) <= RxPipeline(1)(i);
		 RxPipeline(2)(i) <= RxPipeline(2)(i); 
  end if;

-- Use this counter to skip over the preamble at the beginning of the packet
   if iRxDV(i)(0) = '0' or iCRS(i) = '0' then StartCount(i) <= "000";
elsif StartCount(i) /= 6 and iRxDV(i)(0) = '1' and iCRS(i) = '1' and RxClkDL(i) = 2
	and RxNibbleCount(i) = 3 
	then StartCount(i) <= StartCount(i) + 1;
else StartCount(i) <= StartCount(i);
end if;

-- When the word has been assembled and pipeline delayed, write to the FIFO
 if StartCount(i) = 6 and iRxDV(i)(0) = '1' and iCRS(i) = '1' and RxClkDL(i) = 2
  and RxNibbleCount(i) = 3 and Rx_Active(i) = '1' and MaskReg(i) = '1'
 then PhyRxBuff_wreq(i) <= '1'; 
 else PhyRxBuff_wreq(i) <= '0'; 
  end if;

 end if; -- CpldRst

end process PhyRx_Proc;



end generate;
--Debug(2) <= PhyRxBuff_wreq(1);

-- Serializer for MDC links on the Phy chips, SPI ports on the LVDS Tx Chips --
-- Clock runs at 50 MHz, MDI bit period is 40ns, SPI bit perios is 80ns
SMI_Proc : process(CpldRst, i50MHz)

begin 

-- asynchronous reset/preset
 if CpldRst = '0' then

Clk25MHz <= '0'; SMI_rdreq <= '0'; MDC <= "00"; 
TxEn <= (others => '0'); Strt <= "01"; TA <= "10"; 
R_W <= "01"; PhyAd <= "00000"; BitCount <= (others => '0');
SMIShift <= (others => '0'); SMIRdReg0 <= (others => '0');
SMIRdReg1 <= (others => '0'); SMI_Shift <= Idle;
TxNibbleCount <= "00"; PhyTx <= (others => X"0");
PhyTxBuff_rdreq <= '0'; TxEnAck <= '0'; PreambleTx <= '0';
PreambleCnt <= "000"; Preamble <= X"00";

-- Clock fanout SPI signals
SPI_Adddr <= X"0800"; SPI_Shift <= (others => '0');
SPIDiv <= "000"; SPIBitCnt <= (others => '0');
SPI_State <= Idle; SPICS <= '1'; SPISClk <= '0'; 
SPI_rdreq <= '0';

elsif rising_edge (i50MHz) then 

Clk25MHz <= not Clk25MHz; 





-------------------- Logic for Phy MDIO serial control --------------------

--(Idle,Load,Shift,Done);
Case SMI_Shift is
	   When Idle => 	
				if SMI_Empty = '0' and MDC(0) = '0' then SMI_Shift <= Load;
				else SMI_Shift <= Idle;
				end if;
		When Load =>
				if MDC(0) = '0' then SMI_Shift <= Shift;
				else SMI_Shift <= Load;
				end if;
		When Shift => if BitCount = 0 and MDC(0) = '0' then SMI_Shift <= Done;
						 else SMI_Shift <= Shift;
						 end if;
		When Done =>
				if MDC(0) = '0' then SMI_Shift <= Idle;
				else SMI_Shift <= Done;
				end if;
		When others => SMI_Shift <= Idle;
 end Case;

-- Map logical onto physical adresses
Case SMI_Out(22 downto 21) is
	When "00" => PhyAd <= "00001";
	When "01" => PhyAd <= "00011";
	When "10" => PhyAd <= "00111";
	When "11" => PhyAd <= "01111";
	When others => PhyAd <= "00001";
end case;

if SMI_Out(23) = '0' then R_W <= "01";
else R_W <= "10";
end if;

-- SMI shift register
if SMI_Shift = Load and MDC(0) = '0'
then SMIShift <= Strt & R_W & PhyAd & SMI_Out(20 downto 16) & TA & SMI_Out(15 downto 0);
elsif BitCount /= 0 and MDC(0) = '0' and SMI_Shift = Shift 
	then SMIShift <= (SMIShift(30 downto 0) & '0');
end if;

-- After one word has been serialized, issue a FIFO read
if SMI_Shift = Done and MDC(0) = '0'
then SMI_rdreq <= '1';
else SMI_rdreq <= '0';
end if;

-- Serial bit counter
if SMI_Shift = Load and BitCount = 0 and MDC(0) = '0'
--then BitCount <= "11111";
then BitCount <= "100000";
elsif BitCount /= 0 and SMI_Shift = Shift and MDC(0) = '0' then BitCount <= BitCount - 1;
end if;

MDC <= not MDC;
--if SMI_Shift /= Idle or SMI_Empty = '0' then
--   MDC <= not MDC;
--else 
--   MDC <= "00"; 
--end if;

-- Clock in any readback data
if R_W = "10" and SMI_Shift = Shift and BitCount /= 0 and MDC(0) = '0'
then SMIRdReg0 <= SMIRdReg0(14 downto 0) & MDIO(0);
	  SMIRdReg1 <= SMIRdReg1(14 downto 0) & MDIO(1);
end if;

-- Choose which MDIO chain is active
if ChainSel(0) = '0' or SMI_Shift /= Shift or (R_W = "10" and BitCount <= 17)
	then MDIO(0) <= 'Z'; 
  else MDIO(0) <= SMIShift(31); 
end if;

if ChainSel(1) = '0' or SMI_Shift /= Shift or (R_W = "10" and BitCount <= 17)
	then MDIO(1) <= 'Z';
 else MDIO(1) <= SMIShift(31);
end if;

---------------------------- Logic for Phy transmit -----------------------

-- TxEn is used to hold off sending Phy data until a block of data has been 
-- loaded into the transmit FIFO
if Clk25MHz = '0' and TxEnAck = '0' and TxEnReq = '1' and PhyTxBuff_Empty = '0' then TxEnAck <= '1';
elsif PhyTxBuff_Empty = '1' and Clk25MHz = '0' then TxEnAck <= '0';
else TxEnAck <= TxEnAck;
end if;

-- Preamble: X"55",X"55",X"55",X"55",X"55",X"55",X"D5"
-- Use PreambleTx signal to distinguish between preamble and data. When seven 
-- bytes of preamble have been sent, start sending data
   if Clk25MHz = '0' and TxEnAck = '1' and TxEn = 0 and PreambleTx = '0' then PreambleTx <= '1';
elsif Clk25MHz = '0' and PreambleTx = '1' and PreambleCnt = 6 then PreambleTx <= '0';
else PreambleTx <= PreambleTx;
end if;

-- The modulo 4 nibble count needs to do a stutter step due to the odd count
-- of the preamble bytes
if Clk25MHz = '0' and TxEnAck = '1' and not(PreambleTx = '1' and PreambleCnt = 6)
	then TxNibbleCount <= TxNibbleCount + 1; 
elsif Clk25MHz = '0' and (TxEnAck = '0' or (PreambleTx = '1' and PreambleCnt = 6 ))
	then TxNibbleCount <= "00"; 
else TxNibbleCount <= TxNibbleCount; 
end if;

-- Counter used as a timer during preamble transmission
if Clk25MHz = '0' and PreambleTx = '1' and TxEn /= 0 
	and TxNibbleCount(0) = '0' and PreambleCnt /= 6 
	then PreambleCnt <= PreambleCnt + 1;
elsif Clk25MHz = '0' and TxEn /= 0 and TxNibbleCount(0) = '0' and PreambleCnt = 6 
	then PreambleCnt <= "000";
else PreambleCnt <= PreambleCnt;
end if; 

-- Affter sending six bytes of 0x55, change the preamble value to 0xD5
if Clk25MHz = '0' and PreambleCnt = 5 then Preamble <= X"D5";
elsif Clk25MHz = '0' and PreambleCnt /= 5 then Preamble <= X"55";
else Preamble <= Preamble;
end if;

-- Multiplexer to choose which preamble nibble or which nibble
-- of the transmit data word goes to the Tx Outs
Case TxNibbleCount is
	When "00" =>
	if PreambleTx = '1' then TxReg <= Preamble(3 downto 0);
	else TxReg <= PhyTxBuff_Out(3 downto 0);
	end if;
	When "01" =>
	if PreambleTx = '1' then TxReg <= Preamble(7 downto 4);
	else TxReg <= PhyTxBuff_Out(7 downto 4);
	end if;
	When "10" =>
	if PreambleTx = '1' then TxReg <= Preamble(3 downto 0);
	else TxReg <= PhyTxBuff_Out(11 downto 8);
	end if;
	When "11" =>
	if PreambleTx = '1' then TxReg <= Preamble(7 downto 4);
	else TxReg <= PhyTxBuff_Out(15 downto 12);
	end if;
	When others =>
		TxReg <= TxReg;
end Case;

-- Time the transmit data w.r.t the 25MHz Tx clocks
if Clk25MHz = '0' then PhyTx(0 to 3) <= (others => TxReg); 
else PhyTx(0 to 3) <= PhyTx(0 to 3);
end if;

if Clk25MHz = '0' then PhyTx(4 to 7) <= (others => TxReg);
else PhyTx(4 to 7) <= PhyTx(4 to 7);
end if;

-- When four nibbles have been sent, get the next word from the buffer
if TxNibbleCount = 2 and Clk25MHz = '0' and PreambleTx = '0' then PhyTxBuff_rdreq <= '1'; 
else PhyTxBuff_rdreq <= '0'; 
end if;

-- Set Tx Enables high until the Tx FIFO is empty
  if Clk25MHz = '0' and TxEnAck = '1' then TxEn <= TxEnMask;
  elsif Clk25MHz = '0' and PhyTxBuff_Empty = '1' then TxEn <= X"00"; 
  end if;

---------------- Logic used to set up the LVDS clock buffer ---------------

SPIDiv <= SPIDiv + 1;

-- Idle,Load_Addr,Shift_Addr,Shift_Data,Done
Case SPI_State is
 When Idle => 
	if SPI_Empty = '0' and SPIDiv = 7 then SPI_State <= Load_Addr;
	else SPI_State <= Idle;
	end if;
 When Load_Addr => 
	if SPIDiv = 7 then SPI_State <= Shift_Addr;
	else SPI_State <= Load_Addr;
	end if;
 When Shift_Addr => 
	if SPIDiv = 7 and SPIBitCnt = 0 then SPI_State <= Shift_Data;
	else SPI_State <= Shift_Addr;
	end if;
 When Shift_Data => 
	if SPIDiv = 7 and SPIBitCnt = 0 and SPI_Empty = '1' 
		then SPI_State <= Done;
	else SPI_State <= Shift_Data;
	end if;
 When Done => SPI_State <= Idle;

end Case;

if SPIDiv = 7 and SPI_State = Shift_Data and SPIBitCnt = X"E" then SPI_rdreq <= '1';
else SPI_rdreq <= '0';
end if; 

-- Shift register. This is set up for block data moves. 
-- Start by sending address of 0
if SPI_State = Load_Addr then SPI_Shift <= X"0800"; 
-- After that load the shifter with data from the FIFO buffer
elsif (SPI_State = Shift_Addr or SPI_State = Shift_Data) and SPIDiv = 7 and SPIBitCnt = 0 
   then SPI_Shift <= SPI_Out;
-- After loading, shift the data out
elsif (SPI_State = Shift_Addr or SPI_State = Shift_Data) and SPIBitCnt /= 0 and SPIDiv = 7 
then SPI_Shift <= SPI_Shift(14 downto 0) & '0';
end if;

-- Shift register bit counter
if (SPI_State = Load_Addr and SPIDiv = 7) 
or ((SPI_State = Shift_Addr or SPI_State = Shift_Data) and SPIBitCnt = 0 and SPIDiv = 7) then SPIBitCnt <= X"F";
elsif (SPI_State = Shift_Addr or SPI_State = Shift_Data) and SPIDiv = 7
then SPIBitCnt <= SPIBitCnt - 1;
else SPIBitCnt <= SPIBitCnt;
end if;

-- Hold chip selecr low for the duration of the block transfer
if SPICS = '1' and SPI_State = Load_Addr and SPIDiv = 7 then SPICS <= '0'; 
elsif SPI_State = Done then SPICS <= '1';
else SPICS <= SPICS;
end if;

-- Run the clock while data is being shifted out
if SPISClk = '0' and SPIDiv = 3 and (SPI_State = Shift_Addr or SPI_State = Shift_Data) then SPISClk <= '1';
elsif SPISClk = '1' and SPIDiv = 7 then SPISClk <= '0'; 
else SPISClk <= SPISClk;
end if;

end if; -- CpldRst

end process SMI_Proc;

SPIMOSI <= SPI_Shift(15);

AutoTx_Proc : process(SysClk, CpldRst)
  variable p : integer;
  variable found_port : integer range 0 to 7;
  variable have_port : boolean;
begin
  if CpldRst = '0' then
  PhyTxDin_FPGA      <= (others => '0');
  PhyTxWrReq_FPGA    <= '0';
  AutoTx_State       <= "00";
  AutoTx_Port        <= 0;
  AutoTx_WordIdx     <= 0;
  AutoTx_WordPending <= '0';
  AutoTx_Claim       <= (others => '0');  -- ensure claim is deasserted on reset
	elsif rising_edge(SysClk) then
  -- default: clear any claim every cycle; set one-hot when we actually claim
  AutoTx_Claim       <= X"00";

  -- Default single-cycle pulse behavior: ensure FPGA single-cycle writes
		if AutoTx_WordPending = '1' then
			PhyTxWrReq_FPGA <= '0';
			AutoTx_WordPending <= '0';
		else
			PhyTxWrReq_FPGA <= '0';
		end if;

		case AutoTx_State is
			when "00" =>  -- Idle: find a port with ReadyStatus set
			  found_port := 0;
			  have_port := false;
				for p in 0 to 7 loop
					if ReadyStatus(p) = '1' then
						found_port := p;
						have_port := true;
					exit;
					end if;
			end loop;
		if have_port and PhyTxBuff_Full = '0' then
				AutoTx_Port <= found_port;
				AutoTx_WordIdx <= 0;
				AutoTx_Claim(found_port) <= '1';  -- request main to clear ReadyStatus next cycle
				AutoTx_State <= "01";
		end if;
      when "01" =>  -- Sending: issue words one at a time
        if PhyTxBuff_Full = '0' and AutoTx_WordPending = '0' then
          -- prepare the next word to send
          PhyTxDin_FPGA <= ready_packet_word(AutoTx_Port, AutoTx_WordIdx);
          PhyTxWrReq_FPGA <= '1';         -- pulse for one cycle (cleared by top of proc next cycle)
          AutoTx_WordPending <= '1';      -- indicate we must clear the pulse next cycle
          -- increment index for next word (do not wrap yet)
          if AutoTx_WordIdx + 1 >= READY_WORD_COUNT then
            -- last word queued; return to idle on next cycle
            AutoTx_State <= "00";
            AutoTx_WordIdx <= 0;
          else
            AutoTx_WordIdx <= AutoTx_WordIdx + 1;
            AutoTx_State <= "01";
          end if;
        else
          -- wait until FIFO has room or previous pulse clears
          AutoTx_State <= "01";
        end if;

      when others =>
        AutoTx_State <= "00";
    end case;
  end if;
end process AutoTx_Proc;

----------------------- 100 Mhz clocked logic -----------------------------

ResetHi <= not CpldRst;  -- Generate and active high reset for the Xilinx macros

main : process(SysClk, CpldRst)

 begin 

-- asynchronous reset/preset
 if CpldRst = '0' then
-- Synchronous edge detectors for various strobes
	RDDL <= "00"; WRDL <= "00"; PortNo <= 0;
-- Upper DRAM word staging register
	CDStage <= (others => '0'); 
-- Control bits written by the uC
	EventWdCnt <= (others => '0'); EventStat <= (others => '0'); 
	PortWdCounter <= (others => (others => '0')); 
	SDWrtAd <= (others => '0'); SDWrtAdStage <= (others => '0'); 
	SDRdAD <= (others => '0'); SDRdPtr <= (others => '0'); 
   SDWrtCmd <= "000"; WrtCmdEn <= '0'; SDrd_en <= '0'; SDrd_enD <= "000";
	SDRdCmd <= "000";  SDRdCmdEn <= '0'; RdHi_LoSel <= '0'; EventRdy <= '0';
   SDWrtDat <= (others => '0'); SDwr_en <= '0'; WrtHi_LoSel <= '0';
	UpTimeStage <= (others => '0'); UpTimeCount <= (others => '0');
	Counter1us <= X"00"; Counter10us <= (others => '0'); Counter1ms <= (others => '0'); 
	Counter1s <= (others => '0');	TestCount <= (others => '0'); 
	ResetCount <= (others => '0'); TrigWdCntRst <= '0';
	DDR_Reset <= '0';	DDR_Write_Seq <= Idle; DDR_Read_Seq <= Idle; 
   AddrBuff_wren <= '0'; AddrBuff_rden <= '0'; WrtAddrReg <= (others => '0');
	Seq_Busy <= '0'; EvWdCount <= (others => '0');  TxBlkCount <= "000"; DRegSrc <= '0';
	ReadCount <= "000"; MaskReg <= X"FF"; FirstActive <= '0';
	DDRRd_en <= '0'; DDRWrt_En <= '0'; DDRWrt_EnD <= '0'; WaitCount <= (others => '0');
	LinkTxWrReq <= '0'; LinkTxTraceWrReq <= '0'; DatReqBuff_rdreq <= '0'; Rx_active <= X"00";
	SMI_wreq <= '0'; ChainSel <= "11"; PhyDatSel <= '0'; InitReq <= '0';
	PhyTxBuff_wreq <= '0'; TrigWdCount <= X"0"; MDIORd <= '0'; PhyPDn <= '1'; PhyRst <= '0';
	TxEnReq <= '0'; TxEnMask <= X"FF"; LinkTxRDReq <= '0'; TxValid <= '0'; 
	TrigReqCount <= X"00"; Link_Stat_Req <= '0'; HitFlag <= X"00";
	PhyRstCnt <= "11"; FMRxEn <= '0'; PhyRxBuff_RdStat <= X"00"; 
	PhyRxBuff_rdreq <= X"00"; RxBuffRst <= '0'; SPI_WrtReq <= '0'; DDRWrtStat <= X"0";
	ClockReg <= "10101"; -- initial clock pattern
	FrameReg <= "11111"; -- initial framing pattern
	RxDl <= (others => "00"); TransitionCount <= (others => X"0"); 
	DReqFMDL <= X"0"; LinkFIFOStat <= '0';
	LinkStatEn <= '1';
	LinkTxFullCnt <= X"00";
	tx_overflow <= '0';
	tx_overflow_cnt <= (others => '0');
	--tx_word_cnt <= (others => '0');
	word_number <= (others => '0');
	EvWdCountTot <= (others => '0');
--Debug(10 downto 8) <= (others => '0'); 

elsif rising_edge (SysClk) then 

    for p in 0 to 7 loop
      -- two-stage sample of FIFO empty flag for safe edge detection
      phy_empty_d(p)(1) <= phy_empty_d(p)(0);
      phy_empty_d(p)(0) <= PhyRxBuff_Empty(p);

      -- rising edge detected: PhyRxBuff became empty (0 -> 1)
      if phy_empty_d(p)(0) = '1' and phy_empty_d(p)(1) = '0' then
        if ReadyStatus(p) = '0' then
          ReadyStatus(p) <= '1';   -- latch that port is ready for new data
          --ReadyIRQ <= '1';         -- single-cycle pulse; clear below
        end if;
      end if;

		if not is_all_zero(PhyRxBuff_RdCnt(p)) then
			ReadyStatus(p) <= '0';
		end if;
    end loop;

   
-- Synchronous edge detectors for read and write strobes
RDDL(0) <= not uCRD and not CpldCS;
RDDL(1) <= RDDL(0); 

WRDL(0) <= not uCWR and not CpldCS;
WRDL(1) <= WRDL(0);

-- Latch the address for post increment during reads
if RDDL = 1 or WRDL = 1 then AddrReg <= uCA;
else AddrReg <= AddrReg;
end if;

-- clear ReadyStatus bits requested by AutoTx (AutoTx_Claim is single-writer from AutoTx_Proc)
--if AutoTx_Claim /= (others => '0') then
--  ReadyStatus <= ReadyStatus and (not AutoTx_Claim);
--else
--  ReadyStatus <= ReadyStatus;
--end if;
-- clear ReadyStatus bits requested by AutoTx (AutoTx_Claim is single-writer from AutoTx_Proc)
if AutoTx_Claim /= X"00" then
  ReadyStatus <= ReadyStatus and (not AutoTx_Claim);
end if;

-- clear ReadyStatus on microcontroller read of the ReadyStatus register
if RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = ReadyStatusAddr then
  ReadyStatus <= (others => '0');
end if;
       

-- 1us time base
if Counter1us /= Count1us then Counter1us <= Counter1us + 1;
else Counter1us <= X"00";
end if;

-- 10us time base
if Counter10us /= Count10us then Counter10us <= Counter10us + 1;
else Counter10us <= (others => '0');
end if;

-- 1ms time base
if Counter1ms /= Count1ms then Counter1ms <= Counter1ms + 1;
else Counter1ms <= (others => '0');
end if;

-- 1 second time base
if	Counter1s /= Count1s then Counter1s <= Counter1s + 1;
else Counter1s <= (others => '0');
end if;

-- Uptime in seconds since the last FPGA configure
if	Counter1s = Count1s then UpTimeCount <= UpTimeCount + 1;
else UpTimeCount <= UpTimeCount;
end if;

-- Register for staging uptime count.
if CpldCS = '1' then UpTimeStage <= UpTimeCount;
else UpTimeStage <= UpTimeStage;
end if;

-- Loop over eight LVDS receiver channels
for i in 0 to 7 loop

-- FM in edge detectors
RxDl(i)(0) <= FMRx(i);
RxDl(i)(1) <= RxDl(i)(0);

-- Increment this count with FEB LVDS Rx transistions, clear it periodically
if Counter10us(5 downto 0) = "00" & X"0" then TransitionCount(i) <= X"0";
elsif (RxDl(i)(0) = '1' xor RxDl(i)(1) = '1') and TransitionCount(i) /= 15
then TransitionCount(i) <= TransitionCount(i) + 1;
else TransitionCount(i) <= TransitionCount(i);
end if;

-- At the end of the timing interval check to see if there were FM transitions
if    Counter10us(5 downto 0) = "00" & X"0" and TransitionCount(i) = 15 and MaskReg(i) = '1' 
	then Rx_active(i) <= '1';
elsif (Counter10us(5 downto 0) = "00" & X"0" and TransitionCount(i) = 0) or  MaskReg(i) = '0' 
	then Rx_active(i) <= '0';
else Rx_active(i) <= Rx_active(i);
end if;

end loop;

-- Every sysclk reverses the clock and frame pattern
if LockOut = '1' then
	ClockReg <= not ClockReg; 
	FrameReg <= not FrameReg;
else
	ClockReg <= "10101"; 
	FrameReg <= "11111";
end if;
	
--- Channel mask register. One bit corresponds to one FEB
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = InputMaskAddr
then MaskReg <= uCD(7 downto 0);
else MaskReg <= MaskReg;
end if;

-- Data used to initialize the LVDS transmit fanout chip
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SPIWrtAddr
then SPI_WrtReq <= '1';
else SPI_WrtReq <= '0';
end if;

-- Buffer TDAQ trigger packets
if WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = PhyTxFIFOWrtAd)
				 or uCA(9 downto 0) = PhyTxBroadCastAd)
then PhyTxBuff_wreq <= '1'; 
else PhyTxBuff_wreq <= '0';
end if;

-- Clear ReadyStatus bits by writing 1s in the lower byte of uCD
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ReadyClearAddr then
  ReadyStatus <= ReadyStatus and (not uCD(7 downto 0));
end if;

-- When at least one packet has bee received, examine it. If it is a trigger request packet,
-- Start the data transfer from DRAM to the serial link transmitter
if DatReqBuff_Count > 8 and DDR_Read_Seq = Idle then DatReqBuff_rdreq <= '1';
elsif TrigWdCount = 8 or TrigWdCntRst = '1' then DatReqBuff_rdreq <= '0';
else DatReqBuff_rdreq <= DatReqBuff_rdreq;
end if;

-- Use this counter to extract one packet at a time
if DatReqBuff_rdreq = '1' and TrigWdCount /= 8 then TrigWdCount <= TrigWdCount + 1;
elsif TrigWdCount = 8 or TrigWdCntRst = '1' then TrigWdCount <= X"0";
else TrigWdCount <= TrigWdCount;
end if;

--if TrigWdCount = 1 and DatReqBuff_Out(7 downto 4) = 2
-- then TrigReqCount <= TrigReqCount + 1;
--elsif TrigReqCount /= 0 
--		and (DDR_Read_Seq = RdDataHi or DDR_Read_Seq = RdDataLo) and EvWdCount = 0 
-- then TrigReqCount <= TrigReqCount - 1;
--elsif TrigWdCntRst = '1' then TrigReqCount <= X"00";
--else TrigReqCount <= TrigReqCount;
--end if;

if WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
					or (uCA(9 downto 0) = CSRBroadCastAd))
then RxBuffRst <= uCD(0);
	  LinkRst <= uCD(1);
	  PhyPDn <= not uCD(2);
	  FMRxEn <= uCD(3);
	  DDRWrt_En <= uCD(5);
	  PhyDatSel <= uCD(6);
	  DDRRd_en <= uCD(7);
	  InitReq <= uCD(8);
	  TrigWdCntRst <= uCD(9);
else PhyPDn <= PhyPDn;
	  LinkRst <= '0';
	  FMRxEn <= FMRxEn;
     RxBuffRst <= '0';
	  DDRWrt_En <= DDRWrt_En;
	  PhyDatSel <= PhyDatSel;
	  DDRRd_en <= DDRRd_en;
	  InitReq <= '0';
	  TrigWdCntRst <= '0';
end if;

-- This is a copy of the data bit set in FPGA 1. 0 : the DReqFM bit becomes the "and" of the 
-- three link FIFO empty flags. 1 : The bit is FM data containig data request packets.
if WRDL = 1 and uCA = 0 then DRegSrc <= uCD(6);
else DRegSrc <= DRegSrc;
end if;
Debug(2) <= DRegSrc;
DDRWrt_EnD <= DDRWrt_En;

if WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
					or (uCA(9 downto 0) = CSRBroadCastAd))
	and uCD(0) = '1' and PhyRstCnt = 0
then PhyRst <= '0';
elsif PhyRstCnt = 1 and Counter1us = Count1us
then PhyRst <= '1';
else PhyRst <= PhyRst;
end if;

if WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
					or (uCA(9 downto 0) = CSRBroadCastAd))
 and uCD(0) = '1' and PhyRstCnt = 0
then PhyRstCnt <= "11";
elsif PhyRstCnt /= 0 and Counter1us = Count1us
then PhyRstCnt <= PhyRstCnt - 1;
else PhyRstCnt <= PhyRstCnt;
end if;

-- Timer to set width of DDR MIG reset
if WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
					or (uCA(9 downto 0) = CSRBroadCastAd))
and uCD(4) = '1' and ResetCount = 0 
then ResetCount <= X"F";
elsif ResetCount /= 0 then ResetCount <= ResetCount - 1;
end if;

-- MIG Reset
if ResetCount /= 0 then DDR_Reset <= '1';
else DDR_Reset <= '0';
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

-- Write to the SMI serial interface
if WRDL = 1 and uCA(9 downto 0) >= SMIArrayMin and uCA(9 downto 0) <= SMIArrayMax
then SMI_wreq <= '1';
else SMI_wreq <= '0';
end if;

if WRDL = 1 and uCA(9 downto 0) = SMICtrlAddr 
	then ChainSel <= uCD(1 downto 0);
		  MDIORd <= uCD(2);
   else ChainSel <= ChainSel;
		  MDIORd <= MDIORd;
end if;

for i in 0 to 7 loop

-- Mocrocontroller read of the FEB FM Rx FIFOs
if RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = FEBFMRdAddr(i) 
then FEBRxBuff_rdreq(i) <= '1';
else FEBRxBuff_rdreq(i) <= '0';
end if;

-- FEB FM receiver clear parity error
if ((WRDL = 1 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = FMRxErrAddr)
	and (uCD(i) = '1' or uCD(8) = '1')) or CpldRst = '0'
then FEBRxIn(i).Clr_Err <= '1';
else FEBRxIn(i).Clr_Err <= '0';
end if;

end loop;


if WRDL = 1 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = FMRxErrAddr
then RxIn(0).Clr_Err <= uCD(9);
else RxIn(0).Clr_Err <= '0';
end if;

if WRDL = 1 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = FMRxErrAddr
then RxIn(1).Clr_Err <= uCD(10);
else RxIn(1).Clr_Err <= '0';
end if;

-- If there is at least one complete packet in the buffer, enable phy transmit
if TxEnReq = '0' and TxEnAck = '0' and WRDL = 1 and  
   ((uCA(11 downto 10) = GA and uCA(9 downto 0) = PhyTxCSRAddr and uCD(0) = '1')
 or (uCA(9 downto 0) = PhyTxCSRBroadCastAd and uCD(0) = '1'))
 then TxEnReq <= '1';
elsif TxEnReq = '1' and TxEnAck = '1'
 then TxEnReq <= '0';
else TxEnReq <= TxEnReq;
end if;

 if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TxEnMaskAd
   then TxEnMask <= uCD(7 downto 0);
  else TxEnMask <= TxEnMask;
 end if;

-------------------------------- DDR Macro Interfaces -------------------------------

-- Read_Seq_FSM is (Idle,Wait0,SetAddr,CheckEmpty,FirstCmd,CheckRdBuff0,RdWdCount,CheckWdCount,PrepareWordCnt,
-- CheckRdBuff1,RdDataHi,RdDataLo
case DDR_Read_Seq is
	When Idle => DDRRdStat <= "000"; --Debug(10 downto 8) <= "000";
		if (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrLoAd)
		then DDR_Read_Seq <= CheckEmpty;
		elsif DDRRd_En = '1' and 
-- Copy data to the link FIFO when the reads lag the writes by at least one event 
			  ((AddrBuff_empty = '0' and TxBlkCount /= 0 and DDRWrt_En = '1') 
		   or (TrigReqCount /= 0 and DDRWrt_En = '0'))
		then DDR_Read_Seq <= Wait0;
		else DDR_Read_Seq <= Idle;
		end if;
	When Wait0 => 
			if WaitCount = 1 then DDR_Read_Seq <= SetAddr;
	        else DDR_Read_Seq <= Wait0;
			end if;
	When SetAddr => DDR_Read_Seq <= CheckEmpty;  DDRRdStat <= "001"; --Debug(10 downto 8) <= "001";
-- Clear out any stale data from the read FIFO before starting a new block read
	When CheckEmpty => DDRRdStat <= "010"; --Debug(10 downto 8) <= "010";
		if SDrd_empty = '1' then DDR_Read_Seq <= FirstCmd;
		else DDR_Read_Seq <= CheckEmpty;
		end if;
	When FirstCmd => DDRRdStat <= "011"; --Debug(10 downto 8) <= "011";
		if (AddrBuff_empty = '0' and DDRWrt_En = '1')
		or	(TrigReqCount /= 0 and DDRWrt_En = '0')
		then DDR_Read_Seq <= CheckRdBuff0;
		else DDR_Read_Seq <= Idle;
		end if;
	When CheckRdBuff0 => DDRRdStat <= "111";  --Debug(10 downto 8) <= "111";
			if SDrd_empty = '0' then DDR_Read_Seq <= RdWdCount; 
			else DDR_Read_Seq <= CheckRdBuff0;
			end if;
	When RdWdCount => DDRRdStat <= "100"; --Debug(10 downto 8) <= "100";
	    DDR_Read_Seq <= CheckWdCount;
	When CheckWdCount => DDRRdStat <= "100";
	    DDR_Read_Seq <= PrepareWordCnt;
	When PrepareWordCnt => DDRRdStat <= "100";
			if RdHi_LoSel = '0'
			then DDR_Read_Seq <= RdDataHi;
			else DDR_Read_Seq <= RdDataLo;
			end if;
	When CheckRdBuff1 => DDRRdStat <= "101"; --Debug(10 downto 8) <= "101";
		if SDrd_empty = '0' then DDR_Read_Seq <= RdDataHi;
		else DDR_Read_Seq <= CheckRdBuff1;
		end if;
	When RdDataHi => DDRRdStat <= "110"; --Debug(10 downto 8) <= "110";
		if EvWdCount = 0 then DDR_Read_Seq <= Idle;
		else DDR_Read_Seq <= RdDataLo;
		end if;
	When RdDataLo => DDRRdStat <= "111"; --Debug(10 downto 8) <= "111";
 		   if EvWdCount /= 0 and SDrd_en = '1' then DDR_Read_Seq <= CheckRdBuff1;
		elsif EvWdCount = 0 then DDR_Read_Seq <= Idle;
		else DDR_Read_Seq <= RdDataLo;
		end if;
	When others => DDR_Read_Seq <= Idle;
end case;

-- Wait for the DDR write data to get all the way to the inernal capacitor array..
 if DDR_Read_Seq = Wait0 and WaitCount = 0
	then WaitCount <= (others => '1');
 elsif WaitCount /= 0
   then WaitCount <= WaitCount - 1;
 else WaitCount <= WaitCount;
 end if;

-- Filter the "link FOFOs all empty" flag from FPGA 1. 
 DReqFMDL(3 downto 0) <= DReqFMDL(2 downto 0) & DReqFM; 
    if DReqFMDL = X"F" then LinkFIFOStat <= '1';
 elsif DReqFMDL = 0 then LinkFIFOStat <= '0';
 else LinkFIFOStat <= LinkFIFOStat;
 end if;
 
-- Wait for the link FIFOs to go empty before sending a block of events to FPGA 1
 if DRegSrc = '0' and TxBlkCount = 0 and LinkFIFOStat = '1' 
  then TxBlkCount <= "100";
 elsif TxBlkCount /= 0 and DDR_Read_Seq = FirstCmd
  then TxBlkCount <= TxBlkCount - 1;
 else TxBlkCount <= TxBlkCount;
 end if;
Debug(1) <= DReqFM;

	if DDR_Read_Seq = RdWdCount and RdHi_LoSel = '0' then EvWdCount <= SDRdDat(31 downto 16) + 1;
elsif DDR_Read_Seq = RdWdCount and RdHi_LoSel = '1' then EvWdCount <= SDRdDat(15 downto 0) + 1;
elsif DDR_Read_Seq = CheckWdCount and EvWdCount > MAX_TX_WORDS then EvWdCount <= MAX_TX_WORDS + 1;
elsif EvWdCount /= 0 and (DDR_Read_Seq = RdDataHi or (DDR_Read_Seq = RdDataLo and SDrd_en = '1'))
   then EvWdCount <= EvWdCount - 1;
	elsif DDRRd_en = '0' then EvWdCount <= (others => '0');
else EvWdCount <= EvWdCount;
end if;

if DDR_Read_Seq = PrepareWordCnt and EvWdCount > MAX_TX_WORDS then 
    tx_overflow <= '1';
	 tx_overflow_cnt <= tx_overflow_cnt + 1;
elsif DDR_Read_Seq = PrepareWordCnt and EvWdCount <= MAX_TX_WORDS then 
    tx_overflow <= '0';
	 tx_overflow_cnt <= tx_overflow_cnt;
else 
    tx_overflow <= tx_overflow;
	 tx_overflow_cnt <= tx_overflow_cnt;
end if;


-- DDR Read address register
-- Microcontroller access upper
 if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrHiAd 
then SDRdAD <= uCD(13 downto 0) & SDRdAD(15 downto 0);
-- Microcontroller access lower
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrLoAd 
then SDRdAD <= SDRdAD(29 downto 16) & uCD;
-- Increment by 8 long words for each burst read command
elsif SDRdCmdEn = '1' and DDR_Read_Seq /= CheckEmpty then SDRdAD <= SDRdAD + 32;
elsif InitReq = '1' then SDRdAD <= (others => '0');
elsif DDR_Read_Seq = SetAddr and DDRWrt_En = '1'
 then SDRdAD(29 downto 2) <= AddrBuff_Out;
	  SDRdAD(1 downto 0) <= "00";
elsif DDR_Read_Seq = SetAddr and DDRWrt_En = '0' and SDRdPtr(4 downto 0) /= 0 
then SDRdAD(29 downto 5) <= SDRdPtr(29 downto 5) + 1;
	  SDRdAD(4 downto 0) <= "00000";
elsif DDR_Read_Seq = SetAddr and DDRWrt_En = '0' and SDRdPtr(4 downto 0) = 0 
then SDRdAD <= SDRdPtr;
else SDRdAD <= SDRdAD;
end if;

-- Use this pointer to keep track of the read address for every read 
 if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrHiAd 
then SDRdPtr <= uCD(13 downto 0) & SDRdPtr(15 downto 0);
-- Microcontroller access lower
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrLoAd 
then SDRdPtr <= SDRdPtr(29 downto 16) & uCD;
elsif SDrd_en = '1' and DDR_Read_Seq /= CheckEmpty then SDRdPtr <= SDRdPtr + 4;
elsif DDR_Read_Seq = SetAddr and DDRWrt_En = '1'
then SDRdPtr(29 downto 2) <= AddrBuff_Out;
	  SDRdPtr(1 downto 0) <= "00";
elsif DDR_Read_Seq = SetAddr and DDRWrt_En = '0' and SDRdPtr(4 downto 0) /= 0 
then SDRdPtr(29 downto 5) <= SDRdPtr(29 downto 5) + 1;
	  SDRdPtr(4 downto 0) <= "00000";
else SDRdPtr <= SDRdPtr;
end if;

-- DDR controller output FIFO is 32 bits. Clock once per two uC reads
  if (RDDL = 2 and AddrReg(11 downto 10) = GA 
		and (AddrReg(9 downto 0) = SDRamPortAd or AddrReg(9 downto 0) = SDRamSwapPort) 
		and RdHi_LoSel = '1')
	or (DDR_Read_Seq = CheckEmpty and SDrd_empty = '0')
	or (SDrd_en = '0' and EvWdCount /= 0 and DDR_Read_Seq = RdDataLo)
	then SDrd_en <= '1'; 
   else SDrd_en <= '0';
  end if;

-- Send a read command to fetch burst size number of long words
 if (SDrd_en = '1' and SDRdPtr(4 downto 0) = "01100" and SDcmd_full(1) = '0' and DDR_Read_Seq /= CheckEmpty) 
  or DDR_Read_Seq = FirstCmd
then SDRdCmdEn <= '1'; 
	  SDRdCmd <= ReadCmd; 
else SDRdCmdEn <= '0';   
	  SDRdCmd <= "000";  
end if;

-- Use this counter to time the burst reads.
if DDR_Read_Seq = FirstCmd then ReadCount <= "111";
elsif SDrd_en = '1' then ReadCount <= ReadCount - 1;
else ReadCount <= ReadCount;
end if;

-- Toggle between upper and lower words during reads from the DDR
    if DDR_Read_Seq = SetAddr or InitReq = '1'
or (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrLoAd)
then RdHi_LoSel <= '0'; 
 elsif (RDDL = 2 and AddrReg(11 downto 10) = GA 
 and (AddrReg(9 downto 0) = SDRamPortAd or AddrReg(9 downto 0) = SDRamSwapPort))
  or (EvWdCount /= 0 and (DDR_Read_Seq = RdDataHi
  or (DDR_Read_Seq = RdDataLo and EvWdCount /= 0 and SDrd_en = '0')))
then RdHi_LoSel <= not RdHi_LoSel; 
end if;

--- Link FPGA2-FPG1 configuration
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = LinkCtrlAd
then LinkStatEn <= uCD(0);
else LinkStatEn <= LinkStatEn;
end if;

-- At 1Hz SEND A request to update the FM activity bits via the link to FPGA 1
-- Make this a lower priority than data transmission
-- DEBUG 
if Counter1s = Count1s and LinkTxEmpty = '1' and DDR_Read_Seq = Idle and LinkStatEn = '1'
  then Link_Stat_Req <= '1';
 elsif LinkTxWrReq = '1'
  then Link_Stat_Req <= '0';
end if;
--Link_Stat_Req <= '0';
-- DEBUG

-- Serial link write to the top level FPGA
if (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TxFIFOWrtAd)
  or (EvWdCount > 1 and (DDR_Read_Seq = RdDataHi or (DDR_Read_Seq = RdDataLo and SDrd_en = '1')))
  or (LinkTxWrReq = '0' and LinkTxEmpty = '1' and DDR_Read_Seq = Idle and Link_Stat_Req = '1')
then 
  LinkTxWrReq <= '1'; 
  if LinkTxFull = '1'
  then LinkTxFullCnt <= LinkTxFullCnt + 1;
  else LinkTxFullCnt <= LinkTxFullCnt;
  end if;  
else 
  LinkTxWrReq <= '0';
  LinkTxFullCnt <= LinkTxFullCnt;
end if;

-- DEBUG, the same for the buffer, don't buffer the status packages though
if (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TxFIFOWrtAd)
  or (EvWdCount > 1 and (DDR_Read_Seq = RdDataHi or (DDR_Read_Seq = RdDataLo and SDrd_en = '1')))
then LinkTxTraceWrReq <= '1';  
else LinkTxTraceWrReq <= '0'; 
end if;

--if DDR_Write_Seq = Idle then Debug(2) <= '1'; else  Debug(2) <= '0'; end if;

-- Track word position
if DDR_Read_Seq = PrepareWordCnt then 
    --tx_word_cnt <= (others => '0');  -- Reset at start of event
	 word_number <= "01";
elsif EvWdCount /= 0 and (DDR_Read_Seq = RdDataHi or (DDR_Read_Seq = RdDataLo and SDrd_en = '1')) 
    then 
	 -- tx_word_cnt <= tx_word_cnt + 1;
	 word_number(0) <= '0';
	 word_number(1 downto 1) <= word_number(0 downto 0);
else 
    --tx_word_cnt <= tx_word_cnt;
	 word_number <= word_number;
end if;

if DDR_Read_Seq = PrepareWordCnt then
    EvWdCountTot <= EvWdCount - 1; -- EvWdCountTot goes to the header, the +1 is needed for state machine
else
    EvWdCountTot <= EvWdCountTot;
end if;

-- Serial link transmit data
if Link_Stat_Req = '0' and DDR_Read_Seq = RdDataHi then
    --if tx_word_cnt = 1 and tx_overflow = '1' then
	 if word_number(0) = '1' then
	     LinkFIFO_Dat(17 downto 9) <= '1' & EvWdCountTot(15 downto 8);
        LinkFIFO_Dat(8 downto 0)  <= '1' & EvWdCountTot( 7 downto 0);	 
	 elsif word_number(1) = '1' and tx_overflow = '1' then
        LinkFIFO_Dat(17 downto 9) <= '1' & (SDRdDat(31 downto 24) or OVERFLOW_STATUS_BIT(15 downto 8));
        LinkFIFO_Dat(8 downto 0) <= '1' &  (SDRdDat(23 downto 16) or OVERFLOW_STATUS_BIT( 7 downto 0));
		  --LinkFIFO_Dat(8 downto 0) <= '1' & OVERFLOW_STATUS_BIT; 
	 else
        LinkFIFO_Dat(17 downto 9) <= '1' & SDRdDat(31 downto 24);
        LinkFIFO_Dat(8 downto 0) <= '1' & SDRdDat(23 downto 16);
	 end if;
elsif Link_Stat_Req = '0' and DDR_Read_Seq = RdDataLo then 
    --if tx_word_cnt = 1 and tx_overflow = '1' then
	 if word_number(0) = '1' then
	     LinkFIFO_Dat(17 downto 9) <= '1' & EvWdCountTot(15 downto 8);
        LinkFIFO_Dat(8 downto 0)  <= '1' & EvWdCountTot( 7 downto 0);
	 elsif word_number(1) = '1' and tx_overflow = '1' then
        LinkFIFO_Dat(17 downto 9) <= '1' & (SDRdDat(15 downto 8) or OVERFLOW_STATUS_BIT(15 downto 8));
		  LinkFIFO_Dat(8 downto 0) <= '1' &   (SDRdDat(7 downto 0) or OVERFLOW_STATUS_BIT( 7 downto 0));
		  --LinkFIFO_Dat(8 downto 0) <= '1' & OVERFLOW_STATUS_BIT;
    else
	     LinkFIFO_Dat(17 downto 9) <= '1' & SDRdDat(15 downto 8);
		  LinkFIFO_Dat(8 downto 0) <= '1' & SDRdDat(7 downto 0);
    end if;
elsif Link_Stat_Req = '1' then LinkFIFO_Dat <= '0' & X"00" & '0' & Rx_active;
else LinkFIFO_Dat(17 downto 9) <= '1' & uCD(15 downto 8);
	   LinkFIFO_Dat(8 downto 0) <= '1' & uCD(7 downto 0);
end if;

-- Send link data until the buffer is empty
if LinkTxEmpty = '0' and FrameReg = "00000" 
then LinkTxRDReq <= '1';
elsif LinkTxEmpty = '1' then LinkTxRDReq <= '0';
end if;

-- Append the valid bit to the data stream when transmitting
if LinkTxEmpty = '0' and FrameReg = "00000"
then TxValid <= '1';
elsif LinkTxEmpty = '1' then TxValid <= '0';
else  TxValid <= TxValid;
end if;

if DDR_Write_Seq = Rd_WdCount then WrtAddrReg <= SDWrtAd(29 downto 2);
else WrtAddrReg <= WrtAddrReg;
end if;

if DDR_Write_Seq = IncrBuffCnt and SDwr_empty = '1' 
then AddrBuff_wren <= '1';
else AddrBuff_wren <= '0';
end if;

if DDR_Read_Seq = FirstCmd or (DDRRd_En = '0' and AddrBuff_empty = '0') then AddrBuff_rden <= '1';
else AddrBuff_rden <= '0';
end if;

-- DDR Write address register
-- Microcontroller access upper
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrHiAd 
then SDWrtAd <= uCD(13 downto 0) & SDWrtAd(15 downto 0);
-- Microcontroller access lower
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrLoAd
then SDWrtAd <= SDWrtAd(29 downto 16) & uCD;
-- Reset the address when writes are enabled in preparation for incoming FEB data
elsif DDRWrt_En = '1' and DDRWrt_EnD = '0' then SDWrtAd <= (others => '0');
-- Increment by 4 for each long word write
elsif SDwr_en = '1'
then SDWrtAd <= SDWrtAd + 4;
else SDWrtAd <= SDWrtAd;
end if;

-- DDR Write address staging register
-- Microcontroller access upper
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrHiAd 
then SDWrtAdStage <= uCD(13 downto 0) & SDWrtAdStage(15 downto 0);
-- Microcontroller access lower
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrLoAd
then SDWrtAdStage <= SDWrtAdStage(29 downto 16) & uCD;
-- Reset the address when writes are enabled in preparation for incoming FEB data
elsif DDRWrt_En = '1' and DDRWrt_EnD = '0' then SDWrtAdStage <= (others => '0');
-- Keep the address from the last update until the write command has been sent
elsif WrtCmdEn = '1' then SDWrtAdStage <= SDWrtAd;
else SDWrtAdStage <= SDWrtAdStage;
end if;

-- DDR write data staging register
if WrtHi_LoSel = '0' then 
	if WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamPortAd
	 then CDStage <= uCD;
  elsif DDR_Write_Seq = Write_Wd_Count
		then CDStage <= EventWdCnt;
  elsif DDR_Write_Seq = Wrt_uBunchHi
		then  CDStage <= uBunch(31 downto 16);
  elsif DDR_Write_Seq = Wrt_uBunchLo 
		then CDStage <= uBunch(15 downto 0);
  elsif DDR_Write_Seq = Wrt_Stat
		then CDStage <= EventStat;
	elsif  DDR_Write_Seq = WrtDDR
	 then CDStage <= PhyRxBuff_Out(PortNo);
	end if;
  else CDStage <= CDStage;
 end if;

-- Multiplexer to feed the appropriate data to the DRAM write FIFO
	if WrtHi_LoSel = '1' then
	  SDWrtDat(31 downto 16) <= CDStage;
	else SDWrtDat(31 downto 16) <= SDWrtDat(31 downto 16);
	end if;

	 if WrtHi_LoSel = '1' then
	  if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamPortAd
	    then SDWrtDat(15 downto 0) <= uCD;
     elsif DDR_Write_Seq = WrtDDR 
	     then SDWrtDat(15 downto 0) <= PhyRxBuff_Out(PortNo);
	  elsif DDR_Write_Seq = Write_Wd_Count
		  then SDWrtDat(15 downto 0) <= EventWdCnt;
	  elsif DDR_Write_Seq = Wrt_uBunchHi 
	     then SDWrtDat(15 downto 0) <= uBunch(31 downto 16);
     elsif DDR_Write_Seq = Wrt_uBunchLo
		  then SDWrtDat(15 downto 0) <= uBunch(15 downto 0);
     elsif DDR_Write_Seq = Wrt_Stat
		   then SDWrtDat(15 downto 0) <= EventStat;
	  end if;
	  else SDWrtDat(15 downto 0) <= SDWrtDat(15 downto 0);
 end if;

-- Writes to the MIG write FIFO
if (WrtHi_LoSel = '1'
	and ((WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamPortAd) 
	  or DDR_Write_Seq = Write_Wd_Count or DDR_Write_Seq = Wrt_Stat or DDR_Write_Seq = WrtDDR
	  or DDR_Write_Seq = Wrt_uBunchHi or DDR_Write_Seq = Wrt_uBunchLo))
  or (DDR_Write_Seq = WritePad and (SDwr_en = '0' or SDWrtAd(4 downto 0) /= "11100")) 
then SDwr_en <= '1'; --Debug(1) <= '1';
else SDwr_en <= '0'; --Debug(1) <= '0';
end if;

-- When the number of writes = burst size, send a write command
	if (SDwr_en = '1' and SDWrtAd(4 downto 0) = "11100")
-- Issue a write when the write address is being set by the microcontroller so the write FIFO is empty
-- when the write data is loaded into the write FIFO
		or(WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrLoAd)
		or DDR_Write_Seq = SndCmd
			then SDWrtCmd <= "010";
				  WrtCmdEn <= '1';   
			else SDWrtCmd <= "000";
				  WrtCmdEn <= '0';
	  end if;

-- out trace buffer
--	Read of the trigger request trace buffer
	if (RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkTxTraceAd ) or 
	    (LinkTxTrace_Cnt >= "1" & X"F00" ) -- this should make this buffer to a trace buffer.
	then LinkTxTraceRDReq <= '1';
	else LinkTxTraceRDReq <= '0';
	end if;



-- Sum the word counts from the eight PHY receive FIFOs. The four header 
-- words from each FEB are stripped off, then the four header words for the 
-- concatenated data are appended
if DDR_Write_Seq = Idle 
	  then EventWdCnt <= (others => '0');
  elsif DDR_Write_Seq = Rd_WdCount and PhyRxBuff_Out(PortNo) >= 4
	  then EventWdCnt <= EventWdCnt + (PhyRxBuff_Out(PortNo) - 4);
  elsif DDR_Write_Seq = CheckActive0 and PortNo = 7 
	  then EventWdCnt <= EventWdCnt + 4;
 else EventWdCnt <= EventWdCnt;
end if;

-- Signal indicating the first active port
 if DDR_Write_Seq = Idle then FirstActive <= '1';
	elsif DDR_Write_Seq = Rd_Stat then FirstActive <= '0';
   else FirstActive <=  FirstActive;
 end if;	

-- Load the microbunch number from the first active port
	if FirstActive = '1' then
	  if DDR_Write_Seq = Rd_uBunchHi then uBunch <= PhyRxBuff_Out(PortNo) & uBunch(15 downto 0); 
	   elsif DDR_Write_Seq = Rd_uBunchLo then uBunch <= uBunch(31 downto 16) & PhyRxBuff_Out(PortNo);
     end if;
    else uBunch <= uBunch;
	end if;

-- "OR" the status bits from the FEBs and compare the first microbunch to any additional microbunches.
-- EventStat
-- bit 0 to 7: indicate error on port
-- bit 8 to 10: what error(s)
-- bit 11: uB mismatch between ports

if DDR_Write_Seq = Idle then EventStat <= (others => '0'); 
elsif	DDR_Write_Seq = Rd_Stat then
    -- OR each EventStat bit with the corresponding 4-bit group
	 EventStat(PortNo) <= PhyRxBuff_Out(PortNo)(0)  or PhyRxBuff_Out(PortNo)(1)  or PhyRxBuff_Out(PortNo)(2)  or PhyRxBuff_Out(PortNo)(3) or
	                      PhyRxBuff_Out(PortNo)(4)  or PhyRxBuff_Out(PortNo)(5)  or PhyRxBuff_Out(PortNo)(6)  or PhyRxBuff_Out(PortNo)(7) or
                         PhyRxBuff_Out(PortNo)(8)  or PhyRxBuff_Out(PortNo)(9)  or PhyRxBuff_Out(PortNo)(10) or PhyRxBuff_Out(PortNo)(11) or
                         PhyRxBuff_Out(PortNo)(12) or PhyRxBuff_Out(PortNo)(13) or PhyRxBuff_Out(PortNo)(14) or PhyRxBuff_Out(PortNo)(15);								 
	 EventStat(8)  <= EventStat(8) or PhyRxBuff_Out(PortNo)(3)  or PhyRxBuff_Out(PortNo)(2)  or PhyRxBuff_Out(PortNo)(1)  or PhyRxBuff_Out(PortNo)(0);
	 EventStat(9)  <= EventStat(9) or PhyRxBuff_Out(PortNo)(7)  or PhyRxBuff_Out(PortNo)(6)  or PhyRxBuff_Out(PortNo)(5)  or PhyRxBuff_Out(PortNo)(4)
	                              or PhyRxBuff_Out(PortNo)(11) or PhyRxBuff_Out(PortNo)(10) or PhyRxBuff_Out(PortNo)(9)  or PhyRxBuff_Out(PortNo)(8);
	 EventStat(10) <= EventStat(10) or PhyRxBuff_Out(PortNo)(15) or PhyRxBuff_Out(PortNo)(14) or PhyRxBuff_Out(PortNo)(13) or PhyRxBuff_Out(PortNo)(12);
    EventStat(15 downto 11) <= EventStat(15 downto 11); -- Keep upper bits unchanged
elsif FirstActive = '0' then
	 if (DDR_Write_Seq = Rd_uBunchHi and uBunch(31 downto 16) /= PhyRxBuff_Out(PortNo))
	 or (DDR_Write_Seq = Rd_uBunchLo and uBunch(15 downto 0) /= PhyRxBuff_Out(PortNo))
	  then  -- EventStat(7 downto 0) <= EventStat(7 downto 0) or UB_MISMATCH_STATUS_BIT; 
	      EventStat(11) <= EventStat(11) or '1';
	 end if;
else EventStat(15 downto 0) <= EventStat(15 downto 0);
end if;
--EventStat(15 downto 8) <= Rx_active;

-- Toggle between upper and lower words during writes to the DDR
    if (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrLoAd)
	 or (DDR_Write_Seq = Idle and DDRWrt_EnD = '1' and EventRdy = '1') then WrtHi_LoSel <= '0'; 
 elsif (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamPortAd)
	  or DDR_Write_Seq = Write_Wd_Count or DDR_Write_Seq = Wrt_Stat or DDR_Write_Seq = WrtDDR
	  or DDR_Write_Seq = Wrt_uBunchHi or DDR_Write_Seq = Wrt_uBunchLo
then WrtHi_LoSel <= not WrtHi_LoSel;
else WrtHi_LoSel <= WrtHi_LoSel;
end if;

-- loop through eight Phy Rx buffer read request and event status lines
for i in 0 to 7 loop

-- Sequencer read to microcontroller reads of the PhyRx FIFOs  
if (RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = PhyRxRdAddr(i))
or (PortNo = i and 
  ((DDR_Write_Seq = CheckActive0 and Rx_active(PortNo) = '1') 
or (DDR_Write_Seq = CheckActive1 and Rx_active(PortNo) = '1' and HitFlag(PortNo) = '1')
  or DDR_Write_Seq = Rd_WdCount or DDR_Write_Seq = Rd_uBunchHi 
  or DDR_Write_Seq = Rd_uBunchLo 
  or (DDR_Write_Seq = WrtDDR and PortWdCounter(PortNo) > 1 )))
then PhyRxBuff_rdreq(i) <= '1';
else PhyRxBuff_rdreq(i) <= '0';
end if;

-- Status indicating a port is active and a complete event is available for readout
 if Rx_active(i) = '1' and PhyRxBuff_Empty(i) = '0' and PhyRxBuff_RdCnt(i) >= PhyRxBuff_Out(i)
  then PhyRxBuff_RdStat(i) <= '1';
  else PhyRxBuff_RdStat(i) <= '0';
 end if;
 
 

end loop;

-- Flag to indicate an FEB had hits for this event.
-- This expression didn't work inside a loop statement. 
 if DDR_Write_Seq = Idle then HitFlag <= X"00";
 elsif DDR_Write_Seq = Rd_WdCount 
	 then 
		Case PortNo is
		 when 0 => if PhyRxBuff_Out(0) > 4 then HitFlag(0) <= '1';
						else HitFlag(0) <= '0';
						end if;
		 when 1 => if PhyRxBuff_Out(1) > 4 then HitFlag(1) <= '1';
						else HitFlag(1) <= '0';
						end if;
		 when 2 => if PhyRxBuff_Out(2) > 4 then HitFlag(2) <= '1';
						else HitFlag(2) <= '0';
						end if;
		 when 3 => if PhyRxBuff_Out(3) > 4 then HitFlag(3) <= '1';
						else HitFlag(3) <= '0';
						end if;
		 when 4 => if PhyRxBuff_Out(4) > 4 then HitFlag(4) <= '1';
						else HitFlag(4) <= '0';
						end if;
		 when 5 => if PhyRxBuff_Out(5) > 4 then HitFlag(5) <= '1';
						else HitFlag(5) <= '0';
						end if;
		 when 6 => if PhyRxBuff_Out(6) > 4 then HitFlag(6) <= '1';
						else HitFlag(6) <= '0';
						end if;
		 when 7 => if PhyRxBuff_Out(7) > 4 then HitFlag(7) <= '1';
						else HitFlag(7) <= '0';
						end if;
		end Case;
   else HitFlag <= HitFlag;
 end if;

-- Signal to indicate if all active ports have an event ready
  if Rx_active = PhyRxBuff_RdStat then EventRdy <= '1'; 
  else EventRdy <= '0'; 
  end if; 

-- Use this variable to cycle through the input ports during DDR writes
if PortNo /= 7 and (DDR_Write_Seq = IncrPort0 or DDR_Write_Seq = IncrPort1)
	      then PortNo <= PortNo + 1;
           elsif DDR_Write_Seq = ResetPortNo or (DDR_Write_Seq = Idle and EventRdy = '1') then PortNo <= 0; 
       else PortNo <= PortNo;
end if;

-- Load the words counts from the headers into eight counters, one for each port
	if DDR_Write_Seq = Idle then PortWdCounter <= (others => (others => '0'));
	  elsif DDR_Write_Seq = Rd_WdCount and PhyRxBuff_Out(PortNo) /= 0
		then PortWdCounter(PortNo) <= PhyRxBuff_Out(PortNo) - 1;
	  elsif DDR_Write_Seq = Rd_WdCount and PhyRxBuff_Out(PortNo) = 0
		then PortWdCounter(PortNo) <= (others => '0');
	  elsif PortWdCounter(PortNo) /= 0 and PhyRxBuff_rdreq(PortNo) = '1' and PhyRxBuff_Empty(PortNo) = '0' 
		then PortWdCounter(PortNo) <= PortWdCounter(PortNo) - 1;
	 else PortWdCounter(PortNo) <= PortWdCounter(PortNo);
	end if;

-- Idle,ChkWrtBuff,SndCmd,WtCmdMtpy,IncrPort0,CheckActive0,Rd_WdCount,Rd_uBunchHi,Rd_uBunchLo,
-- Rd_Stat,CheckActive1,ResetPortNo,Write_Wd_Count,Wrt_Stat,
-- Wrt_uBunchHi,Wrt_uBunchLo,WrtDDR,WritePad,IncrBuffCnt

Case DDR_Write_Seq is
   When Idle => DDRWrtStat <= X"0"; Debug(10 downto 7) <= X"0"; 
-- Make sure the DDR write FIFO is empty before taking the first trigger
		if DDRWrt_En = '1' and DDRWrt_EnD = '0' then DDR_Write_Seq <= ChkWrtBuff;
-- When the input FIFOs have at least one event, copy the data to the DDR
		elsif DDRWrt_EnD = '1' and EventRdy = '1' then DDR_Write_Seq <= CheckActive0;
		else DDR_Write_Seq <= Idle;
		end if;
-- If there is stale data in the MIG transmit FIFO, force a burst write
	When ChkWrtBuff => 
		if SDwr_count /= 0 then DDR_Write_Seq <= SndCmd;
		else DDR_Write_Seq <= Idle;
		end if;
-- Send a MIG burst write command
	When SndCmd => DDR_Write_Seq <= WtCmdMtpy; 
-- Wait for the MIG write FIFO to go empty
	When WtCmdMtpy =>  
		if SDwr_empty = '1' then DDR_Write_Seq <= Idle;
		else DDR_Write_Seq <= WtCmdMtpy;
		end if;
	When IncrPort0 => DDR_Write_Seq <= CheckActive0; DDRWrtStat <= X"1"; Debug(10 downto 7) <= X"1";
	When CheckActive0 => DDRWrtStat <= X"2"; Debug(10 downto 7) <= X"2"; 
			if Rx_active = 0 then DDR_Write_Seq <= Idle;
			elsif Rx_active(PortNo) = '1'
			 then DDR_Write_Seq <= Rd_WdCount;
			elsif Rx_active(PortNo) = '0' and PortNo /= 7 
			 then DDR_Write_Seq <= IncrPort0;
			else DDR_Write_Seq <= Write_Wd_Count;
			end if;
	When Rd_WdCount => DDR_Write_Seq <= Rd_uBunchHi;  DDRWrtStat <= X"3"; Debug(10 downto 7) <= X"3";
	When Rd_uBunchHi => DDR_Write_Seq <= Rd_uBunchLo; DDRWrtStat <= X"4"; Debug(10 downto 7) <= X"4"; 
	When Rd_uBunchLo => DDR_Write_Seq <= Rd_Stat; DDRWrtStat <= X"5"; Debug(10 downto 7) <= X"5";
	When Rd_Stat =>  DDRWrtStat <= X"6"; Debug(10 downto 7) <= X"6"; 
	      if PortNo = 7 then DDR_Write_Seq <= Write_Wd_Count;
		   else DDR_Write_Seq <= IncrPort0;
			end if;
	When Write_Wd_Count => DDR_Write_Seq <= Wrt_Stat; DDRWrtStat <= X"7"; Debug(10 downto 7) <= X"7";
	When Wrt_Stat => DDR_Write_Seq <= Wrt_uBunchHi;  DDRWrtStat <= X"A"; Debug(10 downto 7) <= X"8";
	When Wrt_uBunchHi => DDR_Write_Seq <= Wrt_uBunchLo;  DDRWrtStat <= X"8"; Debug(10 downto 7) <= X"9"; 
	When Wrt_uBunchLo => DDR_Write_Seq <= ResetPortNo; DDRWrtStat <= X"9"; Debug(10 downto 7) <= X"A"; 
	When ResetPortNo => DDR_Write_Seq <= CheckActive1; DDRWrtStat <= X"B"; Debug(10 downto 7) <= X"B";
	When IncrPort1 => DDR_Write_Seq <= CheckActive1; DDRWrtStat <= X"C"; Debug(10 downto 7) <= X"C"; 
	When CheckActive1 => DDRWrtStat <= X"D"; Debug(10 downto 7) <= X"D"; 
		if PortWdCounter(PortNo) /= 0
		then DDR_Write_Seq <= WrtDDR;
		elsif PortNo = 7 and PortWdCounter(PortNo) = 0
		then DDR_Write_Seq <= WritePad;
		else DDR_Write_Seq <= IncrPort1; 
		end if;
	When WrtDDR => DDRWrtStat <= X"E"; Debug(10 downto 7) <= X"E"; 
		if PortNo /= 7 and (PortWdCounter(PortNo) = 1 or PortWdCounter(PortNo) = 0)
			then DDR_Write_Seq <= IncrPort1;
		elsif (PortNo = 7 and (PortWdCounter(PortNo) = 1 or PortWdCounter(PortNo) = 0)) 
				or DDRWrt_En = '0'
		then DDR_Write_Seq <= WritePad;
		else DDR_Write_Seq <= WrtDDR;
		end if;
	When WritePad => DDRWrtStat <= X"F"; Debug(10 downto 7) <= X"F"; 
		if SDWrtAd(4 downto 0) = "11100" 
		then DDR_Write_Seq <= IncrBuffCnt;
		else DDR_Write_Seq <= WritePad;
		end if;
	When IncrBuffCnt => 
		if SDwr_empty = '1' then 
		 DDR_Write_Seq <= Idle;
		else DDR_Write_Seq <= IncrBuffCnt;
		end if;
	When others => DDR_Write_Seq <= Idle;  
	end case;

	if DDR_Write_Seq = Idle then Seq_Busy <= '0';
	 else Seq_Busy <= '1';
	end if;

--Debug(6) <= PhyRxBuff_Empty(2); 
--Debug(5) <= PhyRxBuff_Empty(1); 
--Debug(4) <= PhyRxBuff_Empty(0);
--if PhyRxBuff_rdreq = 0 then Debug(3) <= '0'; else Debug(3) <= '1'; end if;

end if; -- CpldRst

end process main;

------------------- mux for reading back registers -------------------------

DDRRd_Mux <= SDRdDat(31 downto 16) when RdHi_LoSel = '0' else SDRdDat(15 downto 0);


with uCA(9 downto 0) select

-- did I mess up these bits? Check what it used to be? *First worry about size of design
iCD <= "00000" & DatReqBuff_Empty & "00" & DDRRd_en & PhyDatSel & DDRWrt_En & "0" 
				& FMRxEn & (not PhyPDn) & "0" & RxBuffRst when CSRRegAddr,
		 X"00" & MaskReg when InputMaskAddr,
		 UpTimeStage(31 downto 16) when UpTimeRegAddrHi,
		 UpTimeStage(15 downto 0) when UpTimeRegAddrLo,
		 TestCount(31 downto 16) when TestCounterHiAd,
		 TestCount(15 downto 0) when TestCounterLoAd,
		 DDRWrtStat & X"00" & '0' & DDRRdStat when DDRRdStatAd,
		 "00" & SDWrtAd(29 downto 16) when SDRamWrtPtrHiAd,
		 SDWrtAd(15 downto 0) when SDRamWrtPtrLoAd,
		 "00" & SDRdAD(29 downto 16) when SDRamRdPtrHiAd,
		 SDRdAD(15 downto 0) when SDRamRdPtrLoAd,
		 DDRRd_Mux(7 downto 0) & DDRRd_Mux(15 downto 8) when SDRamSwapPort,
		 DDRRd_Mux when SDRamPortAd,
		 '0' & DDR_Rd_Cnt & '0' & SDwr_count when DDRCountAddr,
		 X"0" & "00" & SDrd_empty & SDrd_full & SDcmd_empty(1) & SDcmd_full(1) 
						 & SDwr_empty & SDwr_full & SDcmd_empty(0) & SDcmd_full(0) 
		 & SDCalDn & SD_RstO when DDRStatAddr,
	    SMIRdReg0 when SMIRdDataAd0,
		 SMIRdReg1 when SMIRdDataAd1,
		 "0000" & PhyRxBuff_RdCnt(0) when PhyRxWdUsedRdAddr(0),
		 "0000" & PhyRxBuff_RdCnt(1) when PhyRxWdUsedRdAddr(1),
		 "0000" & PhyRxBuff_RdCnt(2) when PhyRxWdUsedRdAddr(2),
		 "0000" & PhyRxBuff_RdCnt(3) when PhyRxWdUsedRdAddr(3),
		 "0000" & PhyRxBuff_RdCnt(4) when PhyRxWdUsedRdAddr(4),
		 "0000" & PhyRxBuff_RdCnt(5) when PhyRxWdUsedRdAddr(5),
		 "0000" & PhyRxBuff_RdCnt(6) when PhyRxWdUsedRdAddr(6),
		 "0000" & PhyRxBuff_RdCnt(7) when PhyRxWdUsedRdAddr(7),
		 PhyRxBuff_Out(0) when PhyRxRdAddr(0),
		 PhyRxBuff_Out(1) when PhyRxRdAddr(1),
		 PhyRxBuff_Out(2) when PhyRxRdAddr(2),
		 PhyRxBuff_Out(3) when PhyRxRdAddr(3),
		 PhyRxBuff_Out(4) when PhyRxRdAddr(4),
		 PhyRxBuff_Out(5) when PhyRxRdAddr(5),		 
		 PhyRxBuff_Out(6) when PhyRxRdAddr(6),
	    PhyRxBuff_Out(7) when PhyRxRdAddr(7),
		 X"00" & Rx_active when FEBFMActiveAD,
		 FEBRxBuff_Out(0) when FEBFMRdAddr(0),
		 FEBRxBuff_Out(1) when FEBFMRdAddr(1),
		 FEBRxBuff_Out(2) when FEBFMRdAddr(2),
		 FEBRxBuff_Out(3) when FEBFMRdAddr(3),
		 FEBRxBuff_Out(4) when FEBFMRdAddr(4),
		 FEBRxBuff_Out(5) when FEBFMRdAddr(5),
		 FEBRxBuff_Out(6) when FEBFMRdAddr(6),
		 FEBRxBuff_Out(7) when FEBFMRdAddr(7),
		 X"0" & '0' & FMRxBuff_Count(0) when FEBFMWdsUsedAddr(0),
		 X"0" & '0' & FMRxBuff_Count(1) when FEBFMWdsUsedAddr(1),
		 X"0" & '0' & FMRxBuff_Count(2) when FEBFMWdsUsedAddr(2),
		 X"0" & '0' & FMRxBuff_Count(3) when FEBFMWdsUsedAddr(3),
		 X"0" & '0' & FMRxBuff_Count(4) when FEBFMWdsUsedAddr(4),
		 X"0" & '0' & FMRxBuff_Count(5) when FEBFMWdsUsedAddr(5),
    	 X"0" & '0' & FMRxBuff_Count(6) when FEBFMWdsUsedAddr(6),
		 X"0" & '0' & FMRxBuff_Count(7) when FEBFMWdsUsedAddr(7),
		 FEBRxBuff_Full & FEBRxBuff_Empty when FMRxStatAddr,
		 Rx_CRC_Out(0)(31 downto 16) when RdCRCAddr(0),
		 Rx_CRC_Out(0)(15 downto 0)  when RdCRCAddr(1),
		 Rx_CRC_Out(1)(31 downto 16) when RdCRCAddr(2),
		 Rx_CRC_Out(1)(15 downto 0)  when RdCRCAddr(3),
		 Rx_CRC_Out(2)(31 downto 16) when RdCRCAddr(4),
    	 Rx_CRC_Out(2)(15 downto 0)  when RdCRCAddr(5),
		 Rx_CRC_Out(3)(31 downto 16) when RdCRCAddr(6),
		 Rx_CRC_Out(3)(15 downto 0)  when RdCRCAddr(7),
		 Rx_CRC_Out(4)(31 downto 16) when RdCRCAddr(8),
		 Rx_CRC_Out(4)(15 downto 0)  when RdCRCAddr(9),
		 Rx_CRC_Out(5)(31 downto 16) when RdCRCAddr(10),
		 Rx_CRC_Out(5)(15 downto 0)  when RdCRCAddr(11),
		 Rx_CRC_Out(6)(31 downto 16) when RdCRCAddr(12),
		 Rx_CRC_Out(6)(15 downto 0)  when RdCRCAddr(13),
		 Rx_CRC_Out(7)(31 downto 16) when RdCRCAddr(14),
		 Rx_CRC_Out(7)(15 downto 0)  when RdCRCAddr(15),
		 PhyActivityCounter(0) when PHYActivityCntAdd(0),
		 PhyActivityCounter(1) when PHYActivityCntAdd(1),
		 PhyActivityCounter(2) when PHYActivityCntAdd(2),
		 PhyActivityCounter(3) when PHYActivityCntAdd(3),
		 PhyActivityCounter(4) when PHYActivityCntAdd(4),
		 PhyActivityCounter(5) when PHYActivityCntAdd(5),
		 PhyActivityCounter(6) when PHYActivityCntAdd(6),
		 PhyActivityCounter(7) when PHYActivityCntAdd(7),
		 X"00" & CRCErr_Reg when CRCErrAddr,
		 X"0" & "00" & RxOut(1).Parity_Err & RxOut(0).Parity_Err & PErrStat when FMRxErrAddr,
		 X"000" & '0' & MDIORd & ChainSel when SMICtrlAddr,
		 X"00" & RxErr when RxErrAddr,
		 X"00" & CRS when RxCRSAddr,
		 X"00" & TxEnMask when TxEnMaskAd,
		 X"00" & not PhyRxBuff_Empty when RxDAVAddr,
		 X"000" & "00" & PhyTxBuff_Empty & TxEnAck when PhyTxCSRAddr,
       "00000" & PhyTxBuff_Count when PhyTxCntAddr,
		 TrigWdCount & DRegSrc & '0' & Debug when DebugAddr,
		 "00" & SDRdPtr(29 downto 16) when SDRdPtrAddrHi,
		 SDRdPtr(15 downto 0) when SDRdPtrAddrLo,
       "0000000" & TxFIFOTrace_Out when LinkTxTraceAd,	
		 LinkTxFullCnt & "00" & LinkTxFull & LinkTxEmpty & 
				           "000" & LinkStatEn when LinkCtrlAd,		
		 tx_overflow_cnt when OverflowCntAd,
       X"0011" when DebugVersion,							  
		 X"00" & ReadyStatus when ReadyStatusAddr,
		 X"0000" when others;



uCD <= iCD when uCRd = '0' and CpldCS = '0' and uCA(11 downto 10) = GA 
		 else (others => 'Z');

end behavioural;

-- Sten Hansen 	Fermilab   10/14/2014

-- Global Defs for CRV Controller FPGA 1
LIBRARY ieee;
use ieee.std_logic_1164.all;
use IEEE.std_logic_unsigned.all ;
library unisim ;
use unisim.vcomponents.all ;

package Project_Defs is

----------------------- Address list -----------------------------

Subtype AddrPtr is std_logic_vector(9 downto 0);

-- Control and status register
constant CSRRegAddr : AddrPtr  := "00" & X"00";
constant GTPCSRAddr : AddrPtr  := "00" & X"01";
constant GTPFIFOAddr : AddrPtr := "00" & X"02";

constant EvTxWdCntAd : AddrPtr := "00" & X"03";
constant LinkWdCnt0Ad : AddrPtr := "00" & X"04";
constant LinkWdCnt1Ad : AddrPtr := "00" & X"05";
constant LinkWdCnt2Ad : AddrPtr := "00" & X"06";
constant EvBuffStatAd : AddrPtr := "00" & X"07";

constant ActvRegAddrHi : AddrPtr := "00" & X"08";
constant ActvRegAddrLo : AddrPtr := "00" & X"09";
constant IDregAddr : AddrPtr := "00" & X"0A";
constant DebugPinAd : AddrPtr := "00" & X"0B";
constant ElasticStatAd : AddrPtr := "00" & X"0C";

constant TRigReqBuffAd : AddrPtr := "00" & X"0D"; 
constant TRigReqWdUsedAd : AddrPtr := "00" & X"0E"; 

constant DReqBrstCntAd : AddrPtr := "00" & X"0F"; 

Type LEDAddr_Array is Array(0 to 5) of AddrPtr;
constant LEDDatAddr : LEDAddr_Array  := ("00" & X"10","00" & X"11","00" & X"12",
													  "00" & X"13","00" & X"14","00" & X"15");
constant LEDRstAddr : AddrPtr := "00" & X"16";

constant PLLHiAddr : AddrPtr  := "00" & X"17";
constant PLLLoAddr : AddrPtr  := "00" & X"18";
constant PLLPDnAddr : AddrPtr := "00" & X"19";

Type GTPAddr_Array is Array(0 to 5) of AddrPtr;
constant GTPWrtAddr : GTPAddr_Array := ("00" & X"1A","00" & X"1B","00" & X"1C",
													 "00" & X"1D","00" & X"1E","00" & X"1F");
constant GTPRdAddr0 : AddrPtr := "00" & X"20";
constant GTPRdAddr1 : AddrPtr := "00" & X"21";

constant GTPSeqStatAd : AddrPtr := "00" & X"22";

constant HrtBtDatAd : AddrPtr := "00" & X"23";
-- Array of addresses for reading Link Receive FIFOs
Type RdAddrArrayType is Array(0 to 2) of AddrPtr;
constant LinkRdAddr : RdAddrArrayType := ("00" & X"24","00" & X"25","00" & X"26");
constant LinkCSRAddr : AddrPtr := "00" & X"27";

Type CRCArrayType is Array(0 to 3) of AddrPtr;
constant CRCRdAddr : CRCArrayType := ("00" & X"28","00" & X"29",
												  "00" & X"2A","00" & X"2B");
-- Event word count FIFO
constant EvWdCntBuffAd : AddrPtr := "00" & X"2C";
constant WdCntBuffStatAd : AddrPtr := "00" & X"2D";

constant GTPTxSnoopAd : AddrPtr := "00" & X"2E";
constant GTPTxSnoopStatAd : AddrPtr := "00" & X"2F";

constant SpillWidthRegAd : AddrPtr := "00" & X"30";
constant InterSpillRegAd : AddrPtr := "00" & X"31";

constant HrtBtBrstCntAdHi : AddrPtr := "00" & X"32"; 
constant HrtBtBrstCntAdLo : AddrPtr := "00" & X"33"; 

constant TestCounterHiAd : AddrPtr := "00" & X"34"; 
constant TestCounterLoAd : AddrPtr := "00" & X"35"; 

constant MicroBunchAdHi : AddrPtr  := "00" & X"36";
constant MicroBunchAdMid : AddrPtr := "00" & X"37";
constant MicroBunchAdLo : AddrPtr  := "00" & X"38";

-- Trigger control register
constant TrigCtrlAddr : AddrPtr := "00" & X"39";

constant FreqRegAdHi : AddrPtr := "00" & X"3A";
constant FreqRegAdLo : AddrPtr := "00" & X"3B";

-- Data request buffer
constant DreqBuffStatAd : AddrPtr := "00" & X"3C";
-- Heart Beat request buffer
constant HrtBtBuffStatAd : AddrPtr := "00" & X"3D";
constant HrtBtFIFORdAd : AddrPtr := "00" & X"3E";
-- Data request prescale during beam on
constant PreScaleRegAd : AddrPtr := "00" & X"40";

constant DCSPktBuffAd  : AddrPtr := "00" & X"50";
constant DCSPktWdUsedAd  : AddrPtr := "00" & X"51";

constant SpillTrigCntAdHi : AddrPtr := "00" & X"66";
constant SpillTrigCntAdLo : AddrPtr := "00" & X"67";

constant SpillCountAddr : AddrPtr := "00" & X"68";
constant EVWdCntAddr : AddrPtr := "00" & X"69";

constant SpillWdCntHiAd : AddrPtr := "00" & X"6A";
constant SpillWdCntLoAd : AddrPtr := "00" & X"6B";

constant UpTimeRegAddrHi : AddrPtr := "00" & X"6C";
constant UpTimeRegAddrLo : AddrPtr := "00" & X"6D";

constant TimeStampAdHi : AddrPtr := "00" & X"72";
constant TimeStampAdLo : AddrPtr := "00" & X"73";

constant SpillStatAddr : AddrPtr := "00" & X"76";

constant MarkerBitsAd : AddrPtr  := "00" & X"77";
constant MarkerDelayAd : AddrPtr := "00" & X"78";


constant LinkErrAd : AddrPtr           := "00" & X"80";
constant HrtBtOffsetAd : AddrPtr       := "00" & X"81";
constant DReqCountLowAd : AddrPtr      := "00" & X"82";
constant DReqCountHiAd : AddrPtr       := "00" & X"83";
--constant TxRxBufferSettingAd : AddrPtr := "00" & X"84";
constant GTPTxRdAddr : AddrPtr         := "00" & X"85";
constant DReqBuffTraceAd : AddrPtr     := "00" & X"86";
constant LinkFIFOTraceAd : AddrPtr     := "00" & X"87";

constant DebugVersionAd : AddrPtr      := "00" & X"99";

---------------------- Broadcast addresses ------------------------------

-- Phy transmit broadcast for all three front FPGAs
constant PhyTxBroadCastAd	: AddrPtr := "11" & X"00";

----------------------------------------------------------------------

Type PtrArrayType is Array(0 to 15) of std_logic_vector(3 downto 0);
constant ChanArray : PtrArrayType := (X"0",X"1",X"2",X"3",X"4",X"5",X"6",X"7",
												  X"8",X"9",X"A",X"B",X"C",X"D",X"E",X"F");
-- Timing constants assuming 100 MHz clock
-- 1us timer
constant Count1us : std_logic_vector (7 downto 0) := X"63"; -- 99 D
-- 10us timer
constant Count10us : std_logic_vector (10 downto 0) := "011" & X"E7"; -- 999 (10us)
-- 100us timer
constant Count100us : std_logic_vector (13 downto 0) := "10" & X"70F"; -- 9999 (100us) "00" & X"030"; --
-- 1msec timer
constant Count1ms : std_logic_vector (17 downto 0) := "01" & X"869F"; -- 99999 (1ms) "00" & X"0070"; --
-- 1Second timer
constant Count1s : std_logic_vector (27 downto 0) := X"5F5E0FF"; -- 99999999 -- Decimal
--  "000" & X"000037"; --  Value used for simulating test pulse generator

constant Count1s80MHz : std_logic_vector (27 downto 0) := X"4C4B3FF"; -- 79999999 -- Decimal
--constant Count1s80MHz : std_logic_vector (27 downto 0) := X"00003FF"; -- short value for simulation
constant RefreshCount : std_logic_vector (10 downto 0) := "101" & X"19"; -- 8.192 us

constant SuperCycleLength : std_logic_vector (13 downto 0) := "11" & X"6AF"; -- 13999

constant SpillBegin : std_logic_vector (13 downto 0) := "00" & X"61C"; -- 1566
constant SpillEnd : std_logic_vector (13 downto 0) := "01" & X"4F4"; -- 5364
constant SpillLength : std_logic_vector (8 downto 0) := '1' & X"AE"; -- 430 (43.1 ms)
constant InterSpillLength : std_logic_vector (7 downto 0) := X"31"; -- 49 (5 ms)

-- Use these shortened values for simulation 
--constant Count100us : std_logic_vector (13 downto 0) := "00" & X"01D"; -- 10620 (100us) "00" & X"030"; --
--constant Count1ms : std_logic_vector (17 downto 0) := "00" & X"002F"; -- 106207(1ms) "00" & X"0070"; --
--constant SpillBegin : std_logic_vector (11 downto 0) := X"00C"; -- 156
--constant SuperCycleLength : std_logic_vector (11 downto 0) := X"12A"; -- 1399 
--constant MicroBunchWidth : std_logic_vector (6 downto 0) := "001" & X"9"; -- 89
--constant SpillLength : std_logic_vector (8 downto 0) := '0' & X"20"; -- 480
--constant InterSpillLength : std_logic_vector (7 downto 0) := X"11"; -- 49

-- DDR macro command codes
constant RefreshCmd : std_logic_vector (2 downto 0) := "100";
constant ReadCmd : std_logic_vector (2 downto 0) := "001";
constant WriteCmd : std_logic_vector (2 downto 0) := "000";

-- Command codes from the controller
constant InitCode		: std_logic_vector(15 downto 0) := X"8000";
constant HitCntReset	: std_logic_vector(15 downto 0) := X"8001";
constant BeginSpill	: std_logic_vector(15 downto 0) := X"8002";
constant EndSpill 	: std_logic_vector(15 downto 0) := X"8003";
constant NxtEvReq 	: std_logic_vector(15 downto 0) := X"8004";
constant EventTrig   : std_logic_vector(3 downto 0) := X"9";
constant EventTrigD  : std_logic_vector(3 downto 0) := X"A";
constant PulserTrig  : std_logic_vector(3 downto 0) := X"B";
constant PulserTrigD : std_logic_vector(3 downto 0) := X"C";

----------------------------- Type Defs -------------------------------

Type LinkDat is Array(0 to 2) of std_logic_vector(1 downto 0);

-- Inter-module link FM serializer and deserializer type declarations

	Type TxOutRec is record
		FM,Done : std_logic;
		end record;

	Type RxInRec is record
		FM,Clr_Err : std_logic;
	end record;

	Type RxOutRec is record
		Done,Parity_Err : std_logic;
	end record;

-- TClk FM serializer and deserializer type declarations
Type TClkTxInRec is record
		En : std_logic;
		Data : std_logic_vector(7 downto 0);
end record;

Type TClkTxOutRec is record
		FM,Done : std_logic;
	end record;

Type TClkRxInRec is record
		FM,Clr_Err : std_logic;
end record;

Type  TClkRxOutRec is record
		Done,Parity_Err : std_logic;
		Data : std_logic_vector(7 downto 0);
end record;

------------------------ Xilinx Core gen Macros ------------------------

-- Clock synthesizer macro
component SysPll
port
 (-- Clock in ports
  CLK_IN1_P         : in     std_logic;
  CLK_IN1_N         : in     std_logic;
  -- Clock out ports
  CLK_OUT1          : out    std_logic;
  CLK_OUT2          : out    std_logic;
  CLK_OUT3          : out    std_logic;
  CLK_OUT4          : out    std_logic;
  -- Status and control signals
  RESET             : in     std_logic;
  LOCKED            : out    std_logic
 );
end component;

component GTPClkDCM
port
 (-- Clock in ports
  CLK_IN1           : in     std_logic;
  -- Clock out ports
  CLK_OUT1          : out    std_logic;
  CLK_OUT2          : out    std_logic;
  -- Status and control signals
  RESET             : in     std_logic;
  LOCKED            : out    std_logic
 );
end component;

component FIFO_DC_1kx16
  port ( rst,wr_clk,rd_clk,wr_en,rd_en : in std_logic;
    din : in std_logic_vector(15 downto 0);
    dout : out std_logic_vector(15 downto 0);
    full,empty : out std_logic;
    rd_data_count : out std_logic_vector(10 downto 0));
end component;

component GTPRxFIFO
  port ( rst,clk,wr_en,rd_en : in std_logic;
    din : in std_logic_vector(15 downto 0);
    dout : out std_logic_vector(15 downto 0);
    full,empty : out std_logic;
    data_count : out std_logic_vector(12 downto 0));
end component;

component GTPTxFIFO
  port ( rst,clk,wr_en,rd_en : in std_logic;
    din : in std_logic_vector(15 downto 0);
    dout : out std_logic_vector(15 downto 0);
    full,empty : out std_logic;
    data_count : out std_logic_vector(13 downto 0));
end component;

-- Fifo for queueing data from the front end FPGAs
component LinkFIFO
  port (rst,wr_clk,rd_clk,
		  wr_en,rd_en  : in std_logic;
    din : in std_logic_vector(15 downto 0);
    dout : out std_logic_vector(15 downto 0);
    full,empty : out STD_LOGIC;
    rd_data_count : out std_logic_vector(12 downto 0));
end component;

-- FIFO for queueing data form the microcontroller for setting the LEDs
component CMD_Fifo
  port (clk,rst,wr_en,rd_en : in std_logic;
    din : in std_logic_vector(18 downto 0);
    dout : out std_logic_vector(18 downto 0);
    full,empty : out std_logic);
end component;

-- FIFO for queueing data form the microcontroller for setting up the PLL chip
component PLL_Buff
  port (clk,rst,wr_en,rd_en : in std_logic;
    din : in std_logic_vector(23 downto 0);
    dout : out std_logic_vector(23 downto 0);
    full,empty : out std_logic);
end component;

-- FIFO associated with the optical links
component GTPRxBuff
  port (rst,wr_clk,rd_clk,wr_en,rd_en : in std_logic;
    din : in std_logic_vector(15 downto 0);
    dout : out std_logic_vector(15 downto 0);
    full,empty : out std_logic);
end component;

-- FIFO for storing crossing numbers received from TDAQ
component TrigPktBuff
  port (clk,rst,wr_en,rd_en : in std_logic;
    din : in std_logic_vector(15 downto 0);
    dout : out std_logic_vector(15 downto 0);
    full,empty : out std_logic;
    data_count : out std_logic_vector(8 downto 0));
end component;

component FIFO_SC_4Kx16
  port (clk,rst,wr_en,rd_en : in std_logic;
    din : in std_logic_vector(15 downto 0);
    dout : out std_logic_vector(15 downto 0);
    full,empty : out std_logic);
end component;

-- DPRam storing FEB indentifiers for this controller
component FEBIDListRam
  port (clka,clkb,rstb : IN STD_LOGIC;
     wea : in std_logic_vector(0 downto 0);
    dina : in std_logic_vector(15 downto 0);
    addra,addrb : in STD_LOGIC_VECTOR(4 downto 0);
    doutb : out std_logic_vector(15 downto 0));
end component;

-- constants for serdes factor and number of IO pins
constant S			: integer := 5 ;			-- Set the serdes factor to 5
constant D			: integer := 3 ;			-- Set the number of inputs and outputs
constant DS			: integer := (D*S)-1 ;	-- Used for bus widths = serdes factor * number of inputs - 1

-- Components from serdes example top level files re: XAPP1064
-- S:1 deserialization, D bits wide 
component serdes_1_to_n_clk_ddr_s8_diff is generic (
	S	: integer := 8) ;							-- Parameter to set the serdes factor 1..8
port 	(                                               			
	clkin_p			:  in std_logic ;			-- Input from LVDS receiver pin
	clkin_n			:  in std_logic ;			-- Input from LVDS receiver pin
	rxioclkp		: out std_logic ;				-- IO Clock network
	rxioclkn		: out std_logic ;				-- IO Clock network
	rx_serdesstrobe : out std_logic ;	   -- Parallel data capture strobe
	rx_bufg_x1		: out std_logic) ;		-- Global clock
end component ;

component serdes_1_to_n_data_ddr_s8_diff is generic (
	S			: integer := 8 ;		-- Parameter to set the serdes factor 1..8
	D 			: integer := 16) ;	-- Set the number of inputs and outputs
port 	(
	use_phase_detector	:  in std_logic ;				-- '1' enables the phase detector logic if USE_PD = TRUE
	datain_p		:  in std_logic_vector(D-1 downto 0) ;		-- Input from LVDS receiver pin
	datain_n		:  in std_logic_vector(D-1 downto 0) ;		-- Input from LVDS receiver pin
	rxioclkp		:  in std_logic ;				-- IO Clock network
	rxioclkn		:  in std_logic ;				-- IO Clock network
	rxserdesstrobe		:  in std_logic ;				-- Parallel data capture strobe
	reset			:  in std_logic ;				-- Reset line
	gclk			:  in std_logic ;				-- Global clock
	bitslip			:  in std_logic ;				-- Bitslip control line
	data_out		: out std_logic_vector((D*S)-1 downto 0) ;  	-- Output data
	debug_in		:  in std_logic_vector(1 downto 0) ;  		-- Debug Inputs, set to '0' if not required
	debug			: out std_logic_vector((2*D)+6 downto 0)) ; 	-- Debug output bus, 2D+6 = 2 lines per input (from mux and ce) + 7, 
																				-- leave nc if debug not required
end component ;

component GTP_Xcvr 
generic
(
    -- Simulation attributes  
    WRAPPER_SIM_GTPRESET_SPEEDUP    : integer   := 0; -- Set to 1 to speed up sim reset
    WRAPPER_SIMULATION              : integer   := 0  -- Set to 1 for simulation  
);
port
(   --_________________________________________________________________________
    --_________________________________________________________________________
    --TILE0  (X0_Y0)
    --------------------------------- PLL Ports --------------------------------
    TILE0_CLK00_IN                          : in   std_logic;
    TILE0_CLK01_IN                          : in   std_logic;
    TILE0_GTPRESET0_IN                      : in   std_logic;
    TILE0_GTPRESET1_IN                      : in   std_logic;
    TILE0_PLLLKDET0_OUT                     : out  std_logic;
    TILE0_PLLLKDET1_OUT                     : out  std_logic;
    TILE0_RESETDONE0_OUT                    : out  std_logic;
    TILE0_RESETDONE1_OUT                    : out  std_logic;
    ----------------------- Receive Ports - 8b10b Decoder ----------------------
    TILE0_RXCHARISCOMMA0_OUT                : out  std_logic_vector(1 downto 0);
    TILE0_RXCHARISCOMMA1_OUT                : out  std_logic_vector(1 downto 0);
    TILE0_RXCHARISK0_OUT                    : out  std_logic_vector(1 downto 0);
    TILE0_RXCHARISK1_OUT                    : out  std_logic_vector(1 downto 0);
    TILE0_RXDISPERR0_OUT                    : out  std_logic_vector(1 downto 0);
    TILE0_RXDISPERR1_OUT                    : out  std_logic_vector(1 downto 0);
    TILE0_RXNOTINTABLE0_OUT                 : out  std_logic_vector(1 downto 0);
    TILE0_RXNOTINTABLE1_OUT                 : out  std_logic_vector(1 downto 0);
    ---------------------- Receive Ports - Clock Correction --------------------
    TILE0_RXCLKCORCNT0_OUT                  : out  std_logic_vector(2 downto 0);
    TILE0_RXCLKCORCNT1_OUT                  : out  std_logic_vector(2 downto 0);
    --------------- Receive Ports - Comma Detection and Alignment --------------
    TILE0_RXENMCOMMAALIGN0_IN               : in   std_logic;
    TILE0_RXENMCOMMAALIGN1_IN               : in   std_logic;
    TILE0_RXENPCOMMAALIGN0_IN               : in   std_logic;
    TILE0_RXENPCOMMAALIGN1_IN               : in   std_logic;
    ----------------------- Receive Ports - PRBS Detection ---------------------
    TILE0_PRBSCNTRESET0_IN                  : in   std_logic;
    TILE0_PRBSCNTRESET1_IN                  : in   std_logic;
    TILE0_RXENPRBSTST0_IN                   : in   std_logic_vector(2 downto 0);
    TILE0_RXENPRBSTST1_IN                   : in   std_logic_vector(2 downto 0);
    TILE0_RXPRBSERR0_OUT                    : out  std_logic;
    TILE0_RXPRBSERR1_OUT                    : out  std_logic;
    ------------------- Receive Ports - RX Data Path interface -----------------
    TILE0_RXDATA0_OUT                       : out  std_logic_vector(15 downto 0);
    TILE0_RXDATA1_OUT                       : out  std_logic_vector(15 downto 0);
    TILE0_RXRESET0_IN                       : in   std_logic;
    TILE0_RXRESET1_IN                       : in   std_logic;
    TILE0_RXUSRCLK0_IN                      : in   std_logic;
    TILE0_RXUSRCLK1_IN                      : in   std_logic;
    TILE0_RXUSRCLK20_IN                     : in   std_logic;
    TILE0_RXUSRCLK21_IN                     : in   std_logic;
    ------- Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR ------
    TILE0_RXN0_IN                           : in   std_logic;
    TILE0_RXN1_IN                           : in   std_logic;
    TILE0_RXP0_IN                           : in   std_logic;
    TILE0_RXP1_IN                           : in   std_logic;
    ----------- Receive Ports - RX Elastic Buffer and Phase Alignment ----------
    TILE0_RXBUFSTATUS0_OUT                  : out  std_logic_vector(2 downto 0);
    TILE0_RXBUFSTATUS1_OUT                  : out  std_logic_vector(2 downto 0);
    --------------- Receive Ports - RX Loss-of-sync State Machine --------------
    TILE0_RXLOSSOFSYNC0_OUT                 : out  std_logic_vector(1 downto 0);
    TILE0_RXLOSSOFSYNC1_OUT                 : out  std_logic_vector(1 downto 0);
    -------------------- Receive Ports - RX Polarity Control -------------------
    TILE0_RXPOLARITY0_IN                    : in   std_logic;
    TILE0_RXPOLARITY1_IN                    : in   std_logic;
    ---------------------------- TX/RX Datapath Ports --------------------------
    TILE0_GTPCLKOUT0_OUT                    : out  std_logic_vector(1 downto 0);
    TILE0_GTPCLKOUT1_OUT                    : out  std_logic_vector(1 downto 0);
    ------------------- Transmit Ports - 8b10b Encoder Control -----------------
    TILE0_TXCHARISK0_IN                     : in   std_logic_vector(1 downto 0);
    TILE0_TXCHARISK1_IN                     : in   std_logic_vector(1 downto 0);
    TILE0_TXKERR0_OUT                       : out  std_logic_vector(1 downto 0);
    TILE0_TXKERR1_OUT                       : out  std_logic_vector(1 downto 0);
    ------------------ Transmit Ports - TX Data Path interface -----------------
    TILE0_TXDATA0_IN                        : in   std_logic_vector(15 downto 0);
    TILE0_TXDATA1_IN                        : in   std_logic_vector(15 downto 0);
    TILE0_TXUSRCLK0_IN                      : in   std_logic;
    TILE0_TXUSRCLK1_IN                      : in   std_logic;
    TILE0_TXUSRCLK20_IN                     : in   std_logic;
    TILE0_TXUSRCLK21_IN                     : in   std_logic;
    --------------- Transmit Ports - TX Driver and OOB signalling --------------
    TILE0_TXN0_OUT                          : out  std_logic;
    TILE0_TXN1_OUT                          : out  std_logic;
    TILE0_TXP0_OUT                          : out  std_logic;
    TILE0_TXP1_OUT                          : out  std_logic;
    --------------------- Transmit Ports - TX PRBS Generator -------------------
    TILE0_TXENPRBSTST0_IN                   : in   std_logic_vector(2 downto 0);
    TILE0_TXENPRBSTST1_IN                   : in   std_logic_vector(2 downto 0)
);

end component;

-------------------- component generated by web based tool ------------------

component crc is
  port ( data_in : in std_logic_vector (15 downto 0);
    crc_en , rst, clk : in std_logic;
    crc_out : out std_logic_vector (15 downto 0));
end component;

-------------------- user defined components ------------------

component FM_Tx is
   generic (Pwidth : positive);
	port(clock,reset,Enable : in std_logic;
		  Data : in std_logic_vector(Pwidth - 1 downto 0);
		  Tx_Out : buffer TxOutRec);
end component;

component FM_Rx is
   generic (Pwidth : positive);
   port (SysClk,RxClk,reset : in std_logic;
			Rx_In : in RxInRec;
	      Data : buffer std_logic_vector (Pwidth - 1 downto 0);
	      Rx_Out : buffer RxOutRec);
end component;

end Project_Defs;

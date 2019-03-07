-- Sten Hansen 	Fermilab   10/14/2014

-- Global Defs for AFE FEB FGPA A

LIBRARY ieee;
use ieee.std_logic_1164.all;
use IEEE.std_logic_unsigned.all ;
library unisim ;
use unisim.vcomponents.all ;

package Proj_Defs is

----------------------- Address list -----------------------------

Subtype AddrPtr is std_logic_vector(9 downto 0);

-- Control and status register
constant CSRRegAddr : AddrPtr  := "00" & X"00";
-- SDRAM related pointers are 30 bit byte address pointers
-- Given the 32 bit wide data I/O registers of the LPDDR interface
-- the lower order two bits should be zero.
-- LPDDR write address register
constant SDRamWrtPtrHiAd : AddrPtr := "00" & X"02";
constant SDRamWrtPtrLoAd : AddrPtr := "00" & X"03";

-- LPDDR read address register
constant SDRamRdPtrHiAd	: AddrPtr := "00" & X"04";
constant SDRamRdPtrLoAd	: AddrPtr := "00" & X"05";

-- Port for microcontroller read/write of SDRAM data
constant SDRamSwapPort : AddrPtr := "00" & X"06";
constant SDRamPortAd : AddrPtr := "00" & X"07";

-- DDR status bits, read and write counter
constant DDRStatAddr : AddrPtr := "00" & X"08";
constant DDRCountAddr : AddrPtr := "00" & X"09";

-- LVDS FEB to Controller link transmit address
constant LVDSTxFIFOAd : AddrPtr := "00" & X"0A";
constant LVDSTxFIFOStatAd : AddrPtr := "00" & X"0B";

-- DDR read event data FIFO, evant FIFO words used
constant PageFIFOAddr : AddrPtr := "00" & X"0C";
constant PageFIFOWdsAd : AddrPtr := "00" & X"0D";

-- Three bits showing the state of the DDR write sequencer
constant WriteSeqStatAd : AddrPtr := "00" & X"0E";
constant InseqStatAd : AddrPtr := "00" & X"0F";

-- Histogramming logic parameter adresses
constant HistCtrlAd   : AddrPtr := "00" & X"10";
constant HistIntvalAd : AddrPtr := "00" & X"11";
constant HistOfstAd0 : AddrPtr := "00" & X"12";
constant HistOfstAd1 : AddrPtr := "00" & X"13";
constant HistPtrAd0  : AddrPtr := "00" & X"14";
constant HistPtrAd1  : AddrPtr := "00" & X"15";
constant HistRd0Ad   : AddrPtr := "00" & X"16";
constant HistRd1Ad   : AddrPtr := "00" & X"17";

-- Address for controlling analog mux used for I/V curve data
constant MuxCtrlAd : AddrPtr  := "00" & X"20";
-- Specify which channels to read out
constant InputMaskAddr	: AddrPtr := "00" & X"21";
-- Counter used to produce sequential data as a diagnostic
constant TestCounterHiAd : AddrPtr := "00" & X"22";
constant TestCounterLoAd : AddrPtr := "00" & X"23";

-- Registers used for one wire link to the DS18B20 temperature sensor
constant OneWireCmdAd : AddrPtr := "00" & X"24";
constant TempCtrlAd : AddrPtr := "00" & X"25";

constant TempDat0Ad : AddrPtr := "00" & X"26";
constant TempDat1Ad : AddrPtr := "00" & X"27";
constant TempDat2Ad : AddrPtr := "00" & X"28";
constant TempDat3Ad : AddrPtr := "00" & X"29";
constant TempDat4Ad : AddrPtr := "00" & X"2A";

-- AFE Fifo emplty flags
constant AFEFifoStatAd : AddrPtr := "00" & X"2F";

-- SiPM Bias Trim DAC addresses
constant DatArray0Min : AddrPtr  := "00" & X"30";
constant DatArray0Max : AddrPtr  := "00" & X"37";
constant DatArray1Min : AddrPtr  := "00" & X"38";
constant DatArray1Max : AddrPtr  := "00" & X"3F";
constant DatArray2Min : AddrPtr  := "00" & X"40";
constant DatArray2Max : AddrPtr  := "00" & X"47";

constant CtrlArray0Min : AddrPtr := "00" & X"48";
constant CtrlArray0Max : AddrPtr := "00" & X"4F";
constant CtrlArray1Min : AddrPtr := "00" & X"50";
constant CtrlArray1Max : AddrPtr := "00" & X"57";
constant CtrlArray2Min : AddrPtr := "00" & X"58";
constant CtrlArray2Max : AddrPtr := "00" & X"5F";

constant uBunchFIFOStatAd : AddrPtr := "00" & X"60";

constant EvWrtCounterAdHi : AddrPtr := "00" & X"64";
constant EvWrtCounterAdLo : AddrPtr := "00" & X"65";
-- Spill word count
-- One second timer reset at FPGA config
constant UpTimeRegAddrHi : AddrPtr := "00" & X"6C";
constant UpTimeRegAddrLo : AddrPtr := "00" & X"6D";
-- Diagnostic access to timestamp register
constant TimeStampAdHi : AddrPtr := "00" & X"72";
constant TimeStampAdLo : AddrPtr := "00" & X"73";
-- Spill indicator

Type AddrArrayType is Array(0 to 7) of AddrPtr;
Type RegArrayType is Array(0 to 1) of AddrArrayType;
constant PedRegAddr : RegArrayType := (("00" & X"80","00" & X"81","00" & X"82","00" & X"83",
													 "00" & X"84","00" & X"85","00" & X"86","00" & X"87"),
													("00" & X"88","00" & X"89","00" & X"8A","00" & X"8B",
													 "00" & X"8C","00" & X"8D","00" & X"8E","00" & X"8F"));
constant ThreshRegAddr : RegArrayType := (("00" & X"90","00" & X"91","00" & X"92","00" & X"93",
														 "00" & X"94","00" & X"95","00" & X"96","00" & X"97"),
													   ("00" & X"98","00" & X"99","00" & X"9A","00" & X"9B",
														 "00" & X"9C","00" & X"9D","00" & X"9E","00" & X"9F"));
-- Register collecting data returned for the AFEs
constant AFERdDataAd : AddrPtr  := "00" & X"FF";
-- Map of the internal setup registers of the AFE chips
constant AFE0ArrayMin : AddrPtr  := "01" & X"00";
constant AFE0ArrayMax : AddrPtr  := "01" & X"66";
constant AFE1ArrayMin : AddrPtr  := "10" & X"00";
constant AFE1ArrayMax : AddrPtr  := "10" & X"66";

---------------------- Broadcast addresses ------------------------------
-- DDR macro command codes
constant ReadCmd : std_logic_vector (2 downto 0) := "001";
constant WriteCmd : std_logic_vector (2 downto 0) := "000";

-- Flash gate control register
constant FlashCtrlAddr : AddrPtr := "11" & X"00";
-- Flash gate on time and off time
constant OnTimeAddr  : AddrPtr := "11" & X"01";
constant OffTimeAddr : AddrPtr := "11" & X"02";

-- Trigger control register
constant TrigCtrlAddr : AddrPtr := "11" & X"03";

-- Adjutable pipeline attached to serdes outpus
constant PipeLineAddr	: AddrPtr := "11" & X"04";
-- Adjustable gate used for use in the test beam
constant BeamOnLengthAd  : AddrPtr := "11" & X"05";
constant BeamOffLengthAd : AddrPtr := "11" & X"06";

constant GateAddr	: AddrPtr := "11" & X"07";
-- Adjust such that controller and FEB time stamps match
constant CoarseInitRegAddr : AddrPtr  := "11" & X"0A";
-- Adjust such that pulser trigs from the controller are in time
constant PlsrTrgDlyRegAd : AddrPtr  := "11" & X"0B"; 
-- TCLK start spill event register
constant ADCSmplCntrAd : AddrPtr := "11" & X"0C";
constant uBunchWidthAd : AddrPtr := "11" & X"0D";

constant Inttrgenaddr : AddrPtr := "11" & X"0E";

constant BrdCstRdPtrHiAd : AddrPtr := "11" & X"10";
constant BrdCstRdPtrLoAd : AddrPtr := "11" & X"11";

constant uBunchRdPtrHiAd : AddrPtr := "11" & X"12";
constant uBunchRdPtrLoAd : AddrPtr := "11" & X"13";

constant FEBAddresRegAd      : AddrPtr := "11" & X"14";
constant HistCtrlBroadCastAd : AddrPtr := "11" & X"15";
constant CSRBroadCastAd : AddrPtr := "11" & X"16";
constant PageStatAddr : AddrPtr := "11" & X"17";
constant LEDTimeAddr : AddrPtr := "11" & X"18";

----------------------------------------------------------------------
Subtype AddrPins is std_logic_vector(11 downto 0);

Type PtrArrayType is Array(0 to 15) of std_logic_vector(3 downto 0);

constant ChanArray : PtrArrayType := (X"0",X"1",X"2",X"3",X"4",X"5",X"6",X"7",
												  X"8",X"9",X"A",X"B",X"C",X"D",X"E",X"F");

-- Timing constants assuming 160 MHz clock
-- 1us timer
constant Count1us : std_logic_vector (7 downto 0) := X"9F"; -- 159 D
-- 10us timer
constant Count10us : std_logic_vector (10 downto 0) := "110" & X"3F"; -- 1599 (10us)
-- 1msec timer
constant Count1ms : std_logic_vector (17 downto 0) := "10" & X"70FF"; -- 159999 (1ms)
-- use this for reasonable simulation times
--constant Count1ms : std_logic_vector (17 downto 0) := "00" & X"0040"; -- 159999 (1ms)
-- 1Second timer
constant Count1s : std_logic_vector (27 downto 0) := X"98967FF"; -- 159,999,999 Decimal
-- Use this for debugging purposes
-- constant Count1s : std_logic_vector (27 downto 0) := X"0000280"; -- 640 Decimal
--  "000" & X"000037"; --  Value used for simulating test pulse generator

----------------------------- Type Defs -------------------------------
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
component SysPLL
port
 (-- Clock in ports
  CLK_IN1_P         : in     std_logic;
  CLK_IN1_N         : in     std_logic;
  -- Clock out ports
  CLK_OUT1          : out    std_logic;
  CLK_OUT2          : out    std_logic;
  CLK_OUT3          : out    std_logic;
  -- Status and control signals
  RESET             : in     std_logic
 );
end component;

-- Fifo for queueing serial data. With this, the processor doesn't need to
-- check for serial transmits being done before sending the next data word 
component Cmd_FIFO 
  PORT (
    clk,rst,wr_en,rd_en : IN STD_LOGIC;
    din : IN STD_LOGIC_VECTOR(27 DOWNTO 0);
    dout : OUT STD_LOGIC_VECTOR(27 DOWNTO 0);
    full,empty : OUT STD_LOGIC);
end component;

-- Ram used to shadow DAC writes for later readback
component DAC_Ram
  port (
    clka : IN STD_LOGIC;
    wea : IN STD_LOGIC_VECTOR(0 DOWNTO 0);
    addra : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    dina : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
    douta : OUT STD_LOGIC_VECTOR(15 DOWNTO 0) );
end component;

-- LPDDR controller macro
component LPDDR_Ctrl
 generic(
    C3_P0_MASK_SIZE           : integer := 4;
    C3_P0_DATA_PORT_SIZE      : integer := 32;
    C3_P1_MASK_SIZE           : integer := 4;
    C3_P1_DATA_PORT_SIZE      : integer := 32;
    C3_MEMCLK_PERIOD          : integer := 6250;
    C3_RST_ACT_LOW            : integer := 0;
    C3_INPUT_CLK_TYPE         : string := "DIFFERENTIAL";
    C3_CALIB_SOFT_IP          : string := "TRUE";
    C3_SIMULATION             : string := "TRUE";
    DEBUG_EN                  : integer := 0;
    C3_MEM_ADDR_ORDER         : string := "BANK_ROW_COLUMN";
    C3_NUM_DQ_PINS            : integer := 16;
    C3_MEM_ADDR_WIDTH         : integer := 14;
    C3_MEM_BANKADDR_WIDTH     : integer := 2
);
    port (
   mcb3_dram_dq                            : inout  std_logic_vector(C3_NUM_DQ_PINS-1 downto 0);
   mcb3_dram_a                             : out std_logic_vector(C3_MEM_ADDR_WIDTH-1 downto 0);
   mcb3_dram_ba                            : out std_logic_vector(C3_MEM_BANKADDR_WIDTH-1 downto 0);
   mcb3_dram_cke                           : out std_logic;
   mcb3_dram_ras_n                         : out std_logic;
   mcb3_dram_cas_n                         : out std_logic;
   mcb3_dram_we_n                          : out std_logic;
   mcb3_dram_dm                            : out std_logic;
   mcb3_dram_udqs                          : inout  std_logic;
   mcb3_rzq                                : inout  std_logic;
   mcb3_dram_udm                           : out std_logic;
   c3_sys_clk_p                            : in  std_logic;
   c3_sys_clk_n                            : in  std_logic;
   c3_sys_rst_i                            : in  std_logic;
   c3_calib_done                           : out std_logic;
   c3_clk0                                 : out std_logic;
   c3_rst0                                 : out std_logic;
   mcb3_dram_dqs                           : inout  std_logic;
   mcb3_dram_ck                            : out std_logic;
   mcb3_dram_ck_n                          : out std_logic;
   c3_p2_cmd_clk                           : in std_logic;
   c3_p2_cmd_en                            : in std_logic;
   c3_p2_cmd_instr                         : in std_logic_vector(2 downto 0);
   c3_p2_cmd_bl                            : in std_logic_vector(5 downto 0);
   c3_p2_cmd_byte_addr                     : in std_logic_vector(29 downto 0);
   c3_p2_cmd_empty                         : out std_logic;
   c3_p2_cmd_full                          : out std_logic;
   c3_p2_wr_clk                            : in std_logic;
   c3_p2_wr_en                             : in std_logic;
   c3_p2_wr_mask                           : in std_logic_vector(3 downto 0);
   c3_p2_wr_data                           : in std_logic_vector(31 downto 0);
   c3_p2_wr_full                           : out std_logic;
   c3_p2_wr_empty                          : out std_logic;
   c3_p2_wr_count                          : out std_logic_vector(6 downto 0);
   c3_p2_wr_underrun                       : out std_logic;
   c3_p2_wr_error                          : out std_logic;
   c3_p3_cmd_clk                           : in std_logic;
   c3_p3_cmd_en                            : in std_logic;
   c3_p3_cmd_instr                         : in std_logic_vector(2 downto 0);
   c3_p3_cmd_bl                            : in std_logic_vector(5 downto 0);
   c3_p3_cmd_byte_addr                     : in std_logic_vector(29 downto 0);
   c3_p3_cmd_empty                         : out std_logic;
   c3_p3_cmd_full                          : out std_logic;
   c3_p3_rd_clk                            : in std_logic;
   c3_p3_rd_en                             : in std_logic;
   c3_p3_rd_data                           : out std_logic_vector(31 downto 0);
   c3_p3_rd_full                           : out std_logic;
   c3_p3_rd_empty                          : out std_logic;
   c3_p3_rd_count                          : out std_logic_vector(6 downto 0);
   c3_p3_rd_overflow                       : out std_logic;
   c3_p3_rd_error                          : out std_logic
);
end component;

-- Adjustable pipeline for AFE data, to allow time for the zero suppression
-- logic to make a decision and for compensating trigger latency
component AFE_DP_Pipeline
  port (
    clka,clkb : in std_logic;
	 wea : in std_logic_vector(0 downto 0);
    addra,addrb : in std_logic_vector(7 downto 0);
    dina : in std_logic_vector(95 downto 0);
    doutb : out std_logic_vector(95 downto 0));
end component;

-- FIFO for queueing AFE data while waiting for LPDDR writes 

-- Histogrammer memory 512x32
component Hist_Ram
  port (
    rsta,rstb,clka,clkb : in std_logic;
    wea,web : in STD_LOGIC_VECTOR(0 downto 0);
    addra : in STD_LOGIC_VECTOR(8 downto 0);
	 addrb : in STD_LOGIC_VECTOR(9 downto 0);
    dina : in STD_LOGIC_VECTOR(31 downto 0);
	 dinb : in STD_LOGIC_VECTOR(15 downto 0);
    douta : out STD_LOGIC_VECTOR(31 downto 0);
	 doutb : out STD_LOGIC_VECTOR(15 downto 0)
  );
end component;

-- Fifo for buffering parallel data while it gets serialized
component LVDSTxBuff
  port (rst,clk,wr_en,rd_en : in std_logic;
        din : in STD_LOGIC_VECTOR(15 downto 0);
		  dout : out STD_LOGIC_VECTOR(15 downto 0);
		  full,empty : out std_logic);
end component;

-- Fifo for buffering micro bunch numbers
component SCFIFO_32x256
  port (rst,clk,wr_en,rd_en : in std_logic;
    din : in std_logic_vector(31 downto 0);
    dout : out std_logic_vector(31 downto 0);
    empty,full : out std_logic);
end component;

component DP_Ram_1kx16
  port (clka,clkb : in std_logic;
    wea : in std_logic_vector(0 downto 0);
    dina : in std_logic_vector(15 downto 0);
    addra,addrb : in std_logic_vector(9 downto 0);
    doutb : out std_logic_vector(15 downto 0));
end component;

component SCFIFO_1kx16
  port (clk,rst, wr_en,rd_en : in std_logic;
    din : in std_logic_vector(15 downto 0);
    dout : out std_logic_vector(15 downto 0);
     full,empty : out std_logic;
    data_count : out std_logic_vector(10 downto 0));
end component;

component FIFO_DC_32x1
  port ( rst,wr_clk,rd_clk,wr_en,rd_en : in std_logic;
    din : in std_logic_vector(0 downto 0);
    dout : out std_logic_vector(0 downto 0);
    full,empty : out std_logic  );
end component;

-- constants for serdes factor and number of IO pins
constant S			: integer := 6 ;			-- Set the serdes factor 
constant D			: integer := 9 ;			-- Set the number of inputs and outputs
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

-------------------- user defined components ------------------

component TClk_Rx 
	port(clock,reset : std_logic;
		  TCRx_In : in TClkRxInRec;
	     TCRx_Out : buffer TClkRxOutRec);
end component;

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

component One_Wire is
		 port(clock,reset : in std_logic;
			GA,WRDL  : in std_logic_vector(1 downto 0);
			uCA : in std_logic_vector(11 downto 0);
			uCD : in std_logic_vector(15 downto 0);
			Counter1us : in std_logic_vector(7 downto 0);
			Temp : in  std_logic_vector(3 downto 0);
			TempEn : buffer std_logic;
			TempCtrl : buffer std_logic_vector(3 downto 0);
			One_Wire_Out : buffer std_logic_vector(15 downto 0));
end component;

end Proj_Defs;

-- Sten Hansen 	Fermilab   10/14/2014

-- Global Defs for CRV Controller FPGA 1
LIBRARY ieee;
use ieee.std_logic_1164.all;
use IEEE.std_logic_unsigned.all ;
library unisim ;
use unisim.vcomponents.all ;

use work.Project_defs.all;

package Project_Components is


------------------------ Xilinx Core gen Macros ------------------------

-- Clock synthesizer macro
component SysPll2
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

component Clk80MHzGen
    port(
         clk160 : in  std_logic;
         rst : in  std_logic;
         syncEnable : in  std_logic;
         MarkerBits : in  std_logic_vector(15 downto 0);
         clk80 : out  std_logic;
			shiftCnt : out std_logic_vector(7 downto 0)
        );
end component;

component EventBuilder is
    port (
        clk             : in  std_logic; 
        reset           : in  std_logic; -- active high
        FormRst         : in  std_logic;
        -- LinkFIFOs
        LinkFIFOOut     : in  Array_3x16;
        LinkFIFORdCnt   : in  Array_3x13;
        LinkFIFOEmpty   : in  std_logic_vector(2 downto 0);
        LinkFIFORdReq   : out std_logic_vector(2 downto 0);
        -- EventBuffer
        EventBuff_Dat    : out std_logic_vector (15 downto 0);
        EventBuff_WrtEn  : out std_logic;
        -- other signals
        TStmpWds        : in  std_logic_vector( 8 downto 0); -- used to sync with data requests, needed? 
        ActiveReg       : in  std_logic_vector(23 downto 0);
		  MarkerDelayed   : in  std_logic_vector( 3 downto 0);
        -- uC interface
        LinkRDDL        : in  std_logic_vector (1 downto 0);
        AddrReg         : in  std_logic_vector(11 downto 0);
        -- Settings
        FormHold        : in  std_logic; -- like and inverted enable, if hight -> hold  
        sendGR          : in  std_logic;
        uBinHeader      : in  std_logic;
		  uBwrt           : in  std_logic;
		  GA              : in  std_logic_vector (1 downto 0);
		  sendGrCnt       : in  std_logic_vector (7 downto 0);
		  -- GR data (fake events)
		  ExtuBunchCount  : in  std_logic_vector(47 downto 0);
		  HeartBtCnt      : in  std_logic_vector(15 downto 0);
		  HeartBeatCnt    : in  std_logic_vector(15 downto 0);
		  LastWindow      : in  std_logic_vector(15 downto 0);
		  Stats           : in  std_logic_vector(15 downto 0);
		  InjectionTs     : in  std_logic_vector(15 downto 0);
		  InjectionWindow : in  std_logic_vector(15 downto 0);
		  FakeNum         : out std_logic_vector( 7 downto 0)
    );
end component EventBuilder;

component PacketFormer is
    port (
        clk              : in  std_logic; 
        reset            : in  std_logic; -- active high
        pktFormerSend    : in  std_logic; -- trigers package
        pktFormerTimeout : in  std_logic; 
		  FormRst          : in  std_logic; 
        EventBuff_Out     : in  std_logic_vector(15 downto 0);
        EventBuff_RdEn    : out std_logic; 
        TStmpBuff_rd_en   : out std_logic;
        TStmpBuff_Out     : in  std_logic_vector(15 downto 0);
		  EventBuff_Empty   : in  std_logic;
        ActiveReg        : in  std_logic_vector(23 downto 0);
        DCSBuffRdCnt      : in  std_logic_vector(12 downto 0);
        DCSBuff_Out       : in  std_logic_vector(15 downto 0);
        DCSBuff_rd_en     : out std_logic;
        DRdone           : out std_logic; -- used to clear DR handler  
        TxCRCEn          : out std_logic;
        TxCRCRst         : out std_logic;
		  TxCRCDat         : out std_logic_vector(15 downto 0);
		  TxCRC            : in  std_logic_vector(15 downto 0);
        WdCountBuff_WrtEn : out std_logic;
        GTPTx            : out std_logic_vector(15 downto 0);
        TxCharIsK        : out std_logic_vector( 1 downto 0);
        GTPTxBuff_In      : out std_logic_vector(15 downto 0); -- why not just use GTPTx?
		  GTPTxBuff_wr_en   : out std_logic;
		  --Marker           : in  std_logic;
		  MarkerDelayed    : in  std_logic;
		  loopbackMarker   : in  std_logic;
        -- settings
        LoopbackMode     : in  std_logic_vector( 2 downto 0);
		  uBwrt            : in  std_logic;
        IDReg            : in  std_logic_vector( 3 downto 0)
    );
end component PacketFormer;

component debugMarkerkInputBuffer
   port(
	        rst      : in  std_logic; -- active low
           clk      : in  std_logic;
			  trig     : in  std_logic;
           data_in  : in  std_logic_vector (1 downto 0);
           rd_clk   : in  std_logic;
           rd_en    : in  std_logic;
           rd_data  : out std_logic_vector (15 downto 0);
			  rd_full  : out std_logic;
			  rd_empty : out std_logic
			  );
end component;

end Project_Components;

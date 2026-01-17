-- Sten Hansen 	Fermilab   10/24/2015

-- Global Defs for CRV controller FGPA 2

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

-- Control ROC FPGA2-FPGA1 link
-- bit 0: enable sending active FEBs
constant LinkCtrlAd : AddrPtr  := "00" & X"01";


-- SDRAM related pointers are 30 bit byte address pointers
-- Given the 32 bit wide data I/O registers of the LPDDR interface
-- the lower order two bits should be zero.

-- LPDDR write address register
constant SDRamWrtPtrHiAd : AddrPtr := "00" & X"02";
-- Use this when FGPA1 is resetting its GTP receive FIFOs
constant GTPFIFOAddr : AddrPtr := "00" & X"02";
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

constant BurstSizeAd : AddrPtr := "00" & X"0A";

constant HeaderBuffAd : AddrPtr := "00" & X"0B";
constant DDRRdStatAd : AddrPtr := "00" & X"0C";
constant TxEnMaskAd : AddrPtr := "00" & X"0E";
constant LinkSndReqAd : AddrPtr := "00" & X"0F";
constant TxFIFOWrtAd : AddrPtr := "00" & X"10";

constant PhyTxFIFOWrtAd : AddrPtr := "00" & X"11";
constant PhyTxCSRAddr : AddrPtr := "00" & X"12";
constant PhyTxCntAddr : AddrPtr := "00" & X"13";

constant RxErrAddr : AddrPtr := "00" & X"14"; 
constant RxCRSAddr : AddrPtr := "00" & X"15";
constant RxDAVAddr : AddrPtr := "00" & X"16";
constant InputMaskAddr : AddrPtr := "00" & X"17";

Type AddrArrayType is Array(0 to 7) of AddrPtr;
constant PhyRxWdUsedRdAddr : AddrArrayType := ("00" & X"18","00" & X"19","00" & X"1A","00" & X"1B",
													        "00" & X"1C","00" & X"1D","00" & X"1E","00" & X"1F");
constant PhyRxRdAddr : AddrArrayType := ("00" & X"20","00" & X"21","00" & X"22","00" & X"23",
													  "00" & X"24","00" & X"25","00" & X"26","00" & X"27");
constant FEBFMActiveAD : AddrPtr := "00" & X"2F";

constant FEBFMRdAddr : AddrArrayType
	  := ("00" & X"30","00" & X"31","00" & X"32","00" & X"33",
		   "00" & X"34","00" & X"35","00" & X"36","00" & X"37");

constant FEBFMWdsUsedAddr : AddrArrayType
	  := ("00" & X"38","00" & X"39","00" & X"3A","00" & X"3B",
		   "00" & X"3C","00" & X"3D","00" & X"3E","00" & X"3F");

constant FMRxStatAddr : AddrPtr := "00" & X"40";
constant FMRxErrAddr : AddrPtr := "00" & X"41";

constant SPIWrtAddr : AddrPtr := "00" & X"42";

-- Counter used to produce sequential data as a diagnostic
constant TestCounterHiAd : AddrPtr := "00" & X"43";
constant TestCounterLoAd : AddrPtr := "00" & X"44";
constant DDR_BuffCountAd : AddrPtr := "00" & X"45";

constant SDRdPtrAddrHi : AddrPtr := "00" & X"46";
constant SDRdPtrAddrLo : AddrPtr := "00" & X"47";

-- output trace buffer
constant LinkTxTraceAd : AddrPtr := "00" & X"48";

-- Array of addresses for reading Phy Rx CRCs 
Type CRCAdArrayType is Array(0 to 15) of AddrPtr;
constant RdCRCAddr : CRCAdArrayType := ("00" & X"50","00" & X"51","00" & X"52","00" & X"53",
													 "00" & X"54","00" & X"55","00" & X"56","00" & X"57",
													 "00" & X"58","00" & X"59","00" & X"5A","00" & X"5B",
													 "00" & X"5C","00" & X"5D","00" & X"5E","00" & X"5F");
constant CRCErrAddr : AddrPtr := "00" & X"60";

constant DebugAddr : AddrPtr := "00" & X"61";

constant SpillTrigCntAdHi : AddrPtr := "00" & X"66";
constant SpillTrigCntAdLo : AddrPtr := "00" & X"67";

constant SpillCountAddr : AddrPtr   := "00" & X"68";
constant EVWdCntAddr : AddrPtr  := "00" & X"69";

constant SpillWdCntHiAd : AddrPtr   := "00" & X"6A";
constant SpillWdCntLoAd : AddrPtr   := "00" & X"6B";

constant UpTimeRegAddrHi : AddrPtr  := "00" & X"6C";
constant UpTimeRegAddrLo : AddrPtr  := "00" & X"6D";

constant PHYActivityCntAdd : AddrArrayType := ("00" & X"70","00" & X"71","00" & X"72","00" & X"73",
											      		  "00" & X"74","00" & X"75","00" & X"76","00" & X"77");
															  
constant OverflowCntAd : AddrPtr  := "00" & X"80";

-- Register collecting SMI data returned from the PHY chips
constant SMIRdDataAd0 : AddrPtr  := "00" & X"FD";
constant SMIRdDataAd1 : AddrPtr  := "00" & X"FE";

constant DebugVersion : AddrPtr  := "00" & X"99";
-- Phy SMI control register
constant SMICtrlAddr : AddrPtr  := "00" & X"FF";
-- Map of the internal setup registers of the PHY chips
constant SMIArrayMin : AddrPtr  := "01" & X"00";
constant SMIArrayMax : AddrPtr  := "01" & X"7F";

---------------------- Broadcast addresses ------------------------------

-- Phy transmit broadcast for all three front FPGAs
constant CSRBroadCastAd : AddrPtr := "11" & X"00";
constant PhyTxBroadCastAd : AddrPtr := "11" & X"01";
constant PhyTxCSRBroadCastAd : AddrPtr := "11" & X"02";

-- Adjustable gate used for use in the test beam
--constant GateAddr	: AddrPtr := "11" & X"05";

----------------------------------------------------------------------

-- Timing constants assuming 100MHz clock
-- 1us timer
constant Count1us : std_logic_vector (7 downto 0) := X"63"; -- 99 D
-- 10us timer
constant Count10us : std_logic_vector (9 downto 0) := "11" & X"E7"; -- 999(10us)
-- 1msec timer
constant Count1ms : std_logic_vector (16 downto 0) := '1' & X"869F"; -- 99999 (1ms)
-- Debug --
--constant Count1ms : std_logic_vector (16 downto 0) := '0' & X"00fF";
-- 1Second timer
constant Count1s : std_logic_vector (27 downto 0) := X"5F5E0FF"; -- 99,999,999 Decimal
--  "000" & X"000037"; --  Value used for simulating test pulse generator

-- DDR macro command codes
constant RefreshCmd : std_logic_vector (2 downto 0) := "100";
constant ReadCmd : std_logic_vector (2 downto 0) := "001";
constant WriteCmd : std_logic_vector (2 downto 0) := "000";

-- Added 11/24 for assessing when it is "ready" to send data for prefetch from FEB to ROC
-- new register addresses (lower 10 bits)
-- Use the same defs file so both uC and FPGA see the constants.
--constant ReadyStatusAddr : std_logic_vector(9 downto 0) := X"1A0"; -- read ReadyStatus (low 8 bits)
--constant ReadyClearAddr  : std_logic_vector(9 downto 0) := X"1A1"; -- write bits=1 to clear corresponding ReadyStatus bits
-- Ready status / clear register addresses (10-bit)
-- 0x1A0 = 416 decimal  = "0110100000" (10 bits)
-- 0x1A1 = 417 decimal  = "0110100001" (10 bits)
constant ReadyStatusAddr : std_logic_vector(9 downto 0) := "0110100000";
constant ReadyClearAddr  : std_logic_vector(9 downto 0) := "0110100001";
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

-------------------- component generated by web based tool ------------------

component CRC32_D4 is
  port ( data_in : in std_logic_vector (3 downto 0);
    crc_en , rst, clk : in std_logic;
    crc_out : out std_logic_vector (31 downto 0));
end component;

------------------------ Xilinx Core gen Macros ------------------------

-- Clock synthesizer macro
component Sys_PLL
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

-- Data serializer macro
component LinkTx
generic
 (-- width of the data for the system
  sys_w       : integer := 4;
  -- width of the data for the device
  dev_w       : integer := 20);
port
 (
  -- From the device out to the system
  DATA_OUT_FROM_DEVICE    : in    std_logic_vector(dev_w-1 downto 0);
  DATA_OUT_TO_PINS_P      : out   std_logic_vector(sys_w-1 downto 0);
  DATA_OUT_TO_PINS_N      : out   std_logic_vector(sys_w-1 downto 0);

-- Clock and reset signals
  CLK_IN                  : in    std_logic;                    -- Fast clock from PLL/MMCM 
  CLK_DIV_IN              : in    std_logic;                    -- Slow clock from PLL/MMCM
  LOCKED_IN               : in    std_logic;
  LOCKED_OUT              : out   std_logic;
  IO_RESET                : in    std_logic);                   -- Reset signal for IO circuit
end component;

-- FIFO for queueing data to the Link Serializer
COMPONENT LinkTxFIFO
  PORT (
    rst : IN STD_LOGIC;
    wr_clk : IN STD_LOGIC;
    rd_clk : IN STD_LOGIC;
    din : IN STD_LOGIC_VECTOR(17 DOWNTO 0);
    wr_en : IN STD_LOGIC;
    rd_en : IN STD_LOGIC;
    dout : OUT STD_LOGIC_VECTOR(8 DOWNTO 0);
    full : OUT STD_LOGIC;
    empty : OUT STD_LOGIC
  );
END COMPONENT;

  component LinkTxFIFOTrace is
   PORT (
           wr_clk                    : IN  std_logic;
     	   rd_clk                    : IN  std_logic;
           rd_data_count             : OUT std_logic_vector(13-1 DOWNTO 0);
           rst                       : IN  std_logic;
           wr_en 		     : IN  std_logic;
           rd_en                     : IN  std_logic;
           din                       : IN  std_logic_vector(18-1 DOWNTO 0);
           dout                      : OUT std_logic_vector(9-1 DOWNTO 0);
           full                      : OUT std_logic;
           empty                     : OUT std_logic);

  end component;


-- FIFO for queueing SMI data the PHY chips.
COMPONENT SMI_FIFO
  PORT (
    rst : IN STD_LOGIC;
    wr_clk : IN STD_LOGIC;
    rd_clk : IN STD_LOGIC;
    din : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
    wr_en : IN STD_LOGIC;
    rd_en : IN STD_LOGIC;
    dout : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
    full : OUT STD_LOGIC;
    empty : OUT STD_LOGIC);
END COMPONENT;

-- FIFO for buffering Tx data the PHY chips.

COMPONENT PhyTxBuff
  PORT (
    rst : IN STD_LOGIC;
    wr_clk : IN STD_LOGIC;
    rd_clk : IN STD_LOGIC;
    din : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
    wr_en : IN STD_LOGIC;
    rd_en : IN STD_LOGIC;
    dout : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
    full : OUT STD_LOGIC;
    empty : OUT STD_LOGIC;
    wr_data_count : OUT STD_LOGIC_VECTOR(10 DOWNTO 0));
END COMPONENT;

COMPONENT PhyRxBuff
  PORT (
    rst : IN STD_LOGIC;
    wr_clk : IN STD_LOGIC;
    rd_clk : IN STD_LOGIC;
    din : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
    wr_en : IN STD_LOGIC;
    rd_en : IN STD_LOGIC;
    dout : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
    full : OUT STD_LOGIC;
    empty : OUT STD_LOGIC;
    rd_data_count : OUT STD_LOGIC_VECTOR(11 DOWNTO 0));
END COMPONENT;

COMPONENT FEBRx_test_Buff
  PORT (
    rst : IN STD_LOGIC;
    wr_clk : IN STD_LOGIC;
    rd_clk : IN STD_LOGIC;
    din : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
    wr_en : IN STD_LOGIC;
    rd_en : IN STD_LOGIC;
    dout : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
    full : OUT STD_LOGIC;
    empty : OUT STD_LOGIC;
    rd_data_count : OUT STD_LOGIC_VECTOR(13 DOWNTO 0));
END COMPONENT;

COMPONENT HeaderBuff
  PORT (
    clk : IN STD_LOGIC;
    rst : IN STD_LOGIC;
    din : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
    wr_en : IN STD_LOGIC;
    rd_en : IN STD_LOGIC;
    dout : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
    full : OUT STD_LOGIC;
    empty : OUT STD_LOGIC);
END COMPONENT;

COMPONENT SCFIFO1Kx16
  PORT (
    clk : IN STD_LOGIC;
    rst : IN STD_LOGIC;
    din : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
    wr_en : IN STD_LOGIC;
    rd_en : IN STD_LOGIC;
    dout : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
    full : OUT STD_LOGIC;
    empty : OUT STD_LOGIC;
    data_count : OUT STD_LOGIC_VECTOR(10 DOWNTO 0)
  );
END COMPONENT;

COMPONENT SCFifo_512x16
  PORT (
    clk : IN STD_LOGIC;
    rst : IN STD_LOGIC;
    din : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
    wr_en : IN STD_LOGIC;
    rd_en : IN STD_LOGIC;
    dout : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
    full : OUT STD_LOGIC;
    empty : OUT STD_LOGIC;
    data_count : OUT STD_LOGIC_VECTOR(9 DOWNTO 0)
  );
END COMPONENT;

COMPONENT SCFIFO_1Kx28
  PORT (
    clk : IN STD_LOGIC;
    rst : IN STD_LOGIC;
    din : IN STD_LOGIC_VECTOR(27 DOWNTO 0);
    wr_en : IN STD_LOGIC;
    rd_en : IN STD_LOGIC;
    dout : OUT STD_LOGIC_VECTOR(27 DOWNTO 0);
    full : OUT STD_LOGIC;
    empty : OUT STD_LOGIC
  );
END COMPONENT;

-- LPDDR controller macro
component LPDDR_Ctrl
 generic(
    C3_P0_MASK_SIZE           : integer := 4;
    C3_P0_DATA_PORT_SIZE      : integer := 32;
    C3_P1_MASK_SIZE           : integer := 4;
    C3_P1_DATA_PORT_SIZE      : integer := 32;
    C3_MEMCLK_PERIOD          : integer := 9400;
    C3_RST_ACT_LOW            : integer := 0;
    C3_INPUT_CLK_TYPE         : string := "DIFFERENTIAL";
    C3_CALIB_SOFT_IP          : string := "TRUE";
    C3_SIMULATION             : string := "FALSE";
    DEBUG_EN                  : integer := 0;
    C3_MEM_ADDR_ORDER         : string := "BANK_ROW_COLUMN";
    C3_NUM_DQ_PINS            : integer := 16;
    C3_MEM_ADDR_WIDTH         : integer := 14;
    C3_MEM_BANKADDR_WIDTH     : integer := 2);
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
   c3_p3_rd_error                          : out std_logic);
end component;

-------------------- user defined components ------------------

component TClk_Rx 
	port(clock,reset : std_logic;
		  TCRx_In : in TClkRxInRec;
	     TCRx_Out : buffer TClkRxOutRec);
end component;

component FM_Rx is
   generic (Pwidth : positive);
   port (SysClk,RxClk,reset : in std_logic;
			Rx_In : in RxInRec;
	      Data : buffer std_logic_vector (Pwidth - 1 downto 0);
	      Rx_Out : buffer RxOutRec);
end component;

component FM_Tx is
	generic (Pwidth : positive);
		 port(clock,reset,Enable : in std_logic;
				Data : in std_logic_vector(Pwidth - 1 downto 0);
				Tx_Out : buffer TxOutRec);
end component;

end Proj_Defs;

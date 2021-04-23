--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   10:22:03 03/21/2016
-- Design Name:   
-- Module Name:   C:/Experiments/mu2e/Readout_Controller/Controller_FPGA2/Controller_FPGA2_tb.vhd
-- Project Name:  Controller_FPGA2
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: Controller_FPGA2
-- 
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use work.Proj_Defs.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY Controller_FPGA2_tb IS
END Controller_FPGA2_tb;
 
ARCHITECTURE behavior OF Controller_FPGA2_tb IS 
 
    -- Component Declaration for the Unit Under Test (UUT)

 COMPONENT Controller_FPGA2
  PORT(
-- 100 MHz VXO clock, Phy clocks
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
  END COMPONENT;

--Inputs
   signal VXO_P,VXO_N,ClkB_P,ClkB_N : std_logic := '0';
   signal Clk50MHz : std_logic := '0';
   signal CpldRst,CpldCS,uCRd,uCWr : std_logic := '1';
   signal uCA : std_logic_vector(11 downto 0) := (others => '0');
   signal GA : std_logic_vector(1 downto 0) := (others => '0');
   signal RxDA,RxDB,RxDC,RxDD,RxDE,RxDF,RxDG,RxDH  : std_logic_vector(3 downto 0) := (others => '0');
   signal RxClk,RxDV,RxErr,CRS : std_logic_vector(7 downto 0) := (others => '0');
   signal TxClk : std_logic_vector(1 downto 0) := (others => '0');
   signal FMRx : std_logic_vector(7 downto 0) := (others => '0');
   signal HrtBtFM,DReqFM : std_logic := '0';
	signal SPICS,SPISClk,SPIMOSI : std_logic;
	signal SPIMISO : std_logic;

--BiDirs
   signal uCD,SDD : std_logic_vector(15 downto 0);
   signal UDQS,LDQS,SDRzq : std_logic;
   signal MDIO : std_logic_vector(1 downto 0);

--Outputs
   signal SDCKE,LDM,UDM,RAS,CAS,SDWE : std_logic;
   signal SDClk_P,SDClk_N : std_logic;
   signal SDA : std_logic_vector(13 downto 0);
   signal BA : std_logic_vector(1 downto 0);
   signal LinkClk_P,LinkClk_N : std_logic;
   signal LinkFR_P,LinkFR_N : std_logic;
   signal LinkD_P,LinkD_N : std_logic_vector(1 downto 0);
   signal TxDA,TxDB,TxDC,TxDD,TxDE,TxDF,TxDG,TxDH : std_logic_vector(3 downto 0);
   signal TxEn,iRxClk,iRxDV,iCRS : std_logic_vector(7 downto 0);
   signal MDC : std_logic_vector(1 downto 0);
   signal PhyPDn,PhyRst,Clk25MHz,Clk100MHz,FMRxEn : std_logic;
   signal Debug : std_logic_vector(10 downto 1);

	signal MDIO_Shift_Out0,MDIO_Shift_Out1,PhyTxDat : std_logic_vector(15 downto 0);
	signal MDIO_Shift_In0,MDIO_Shift_In1 : std_logic_vector(3 downto 0);
	signal MDIO_BitCount : std_logic_vector(4 downto 0);

	Type MDIO_State_Type is (Idle,Ld_RdBitCount,Ld_WrtBitCount,Wait_Write,Shift,Done);
	signal MDIO_State : MDIO_State_Type;

	Type RxInArray is Array(0 to 7) of std_logic_vector(3 downto 0);
	signal RxIns : RxInArray;

	Type PhyTstArray is Array(0 to 30) of std_logic_vector(15 downto 0);

	Type CountArray is Array(0 to 7) of std_logic_vector(1 downto 0);
	signal NibbleCount : CountArray;

	signal ResetHi,TrgTxEn,FMTxEn,TxDelay,RxDatEnable,RxDatEnableD,CRSEn : std_logic;
	signal TrigRxBuff_Out : std_logic_vector(15 downto 0);

	signal TrigOuts,LVDSOuts : TxOutRec;
	
--	constant RxSize : Integer := 94;
--	Type RxOutArray is Array(0 to RxSize - 1) of std_logic_vector(15 downto 0);
--	signal RxOuts : RxOutArray;
--	constant PhyRxVal : RxOutArray := (X"0030",X"5555",X"55D5",
--												  X"002B",X"1234",X"5678",
--												  X"0000",X"0004",X"0005",X"0006",
--												  X"0007",X"D9ED",X"C815",X"0000",
--												  X"8002",X"8002",X"957D",X"AEF0",
--                                    X"9210",X"9A98",X"A654",X"9654",
--												  X"9122",X"9344",X"9566",X"9788",
--                                    X"ABCC",X"ADEE",X"9F00",X"8003",
--												  X"1DED",X"8004",X"3AA9",X"8787",
--												  X"1122",X"0011",X"5566",X"7788",
--                                    X"09AA",X"0BCC",X"3DEE",X"FF00",
--												  X"1DED",X"8004",X"3AA9",X"8787",
--												  X"000D",X"1234",X"5679",X"0000",
--                                    X"09AA",X"3DEE",X"FF00",X"8004",
--												  X"1DED",X"8004",X"3AA9",X"8787",
--												  X"1122",X"0011",X"5566",X"7788",
-- 											  X"1122",X"0011",X"5566",X"7788",
--                                    X"09AA",X"0BCC",X"3DEE",X"FF00",
--												  X"1DED",X"8004",X"3AA9",X"8787",
--												  X"0003",X"0004",X"0005",X"0006",
--												  X"D9ED",X"C815",X"0000",X"000D",
--												  X"8002",X"8002",X"957D",X"AEF0",
--                                    X"9210",X"9A98",X"A654",X"9654",
--                                    X"09AA",X"0BCC",X"3DEE",X"FF00");

	constant RxSize0 : Integer := 385;
	Type RxOutArray0 is Array(0 to RxSize0 - 1) of std_logic_vector(15 downto 0);
	signal RxOuts0 : RxOutArray0;
	constant PhyRxVal0 : RxOutArray0 := (X"0030",X"5555",X"55D5",
												  X"0022",X"0000",X"0001",X"0000",X"0098",X"81AB",X"0FFD",X"0FFE",
												  X"0005",X"0033",X"0015",X"0FE9",X"0FE8",X"0FF7",X"009E",X"82AA",
												  X"0003",X"0004",X"001C",X"0027",X"0003",X"0FF0",X"0FF8",X"0000",
												  X"009F",X"82AA",X"000D",X"000C",X"001F",X"002B",X"000C",X"0FFC",
												  X"0003",X"000C",X"004A",X"0000",X"0002",X"0000",X"0098",X"8017",
												  X"0000",X"0FFE",X"0005",X"0031",X"0012",X"0FE9",X"0FE6",X"0FF6",
												  X"0098",X"880B",X"0000",X"0FFD",X"0007",X"0033",X"0010",X"0FE9",
												  X"0FE7",X"0FF6",X"009C",X"8116",X"0FF5",X"0FF6",X"000B",X"0014",
												  X"0FF7",X"0FE9",X"0FF0",X"0FF8",X"009E",X"8116",X"0002",X"0003",
												  X"001D",X"0026",X"0002",X"0FF1",X"0FF7",X"0000",X"009E",X"890A",
												  X"0004",X"0002",X"001F",X"0024",X"0FFF",X"0FEE",X"0FF9",X"0001",
												  X"009F",X"8116",X"000C",X"000C",X"0020",X"002A",X"000D",X"0FFE",
												  X"0002",X"000B",X"009F",X"890A",X"000D",X"000D",X"0025",X"002B",
												  X"0009",X"0FFC",X"0004",X"000A",X"0018",X"0000",X"0003",X"0000",
												  X"0098",X"8677",X"0FFF",X"0FFE",X"0009",X"0032",X"000D",X"0FE6",
												  X"0FE9",X"0FF8",X"009E",X"8775",X"0002",X"0003",X"0004",X"0020",
												  X"0023",X"0FFE",X"0FEF",X"0FF9",X"0022",X"0000",X"0004",X"0000",
												  X"0098",X"84E2",X"0FFD",X"0FFE",X"000E",X"0034",X"000B",X"0FE7",
												  X"0FE9",X"0FF8",X"009E",X"85E0",X"0002",X"0002",X"0003",X"0023",
												  X"0020",X"0FFB",X"0FF0",X"0FF9",X"009F",X"85E0",X"000C",X"000C",
												  X"000D",X"0029",X"0026",X"0007",X"0FFE",X"0005",X"0022",X"0000",
												  X"0005",X"0000",X"0098",X"834E",X"0FFE",X"0FFE",X"0010",X"0034",
												  X"0007",X"0FE5",X"0FEB",X"0FF8",X"009E",X"844C",X"0003",X"0003",
												  X"0003",X"0028",X"0022",X"0FFA",X"0FEF",X"0FFA",X"009F",X"844C",
												  X"000D",X"000E",X"000E",X"002B",X"0027",X"0005",X"0FFC",X"0008",
												  X"0022",X"0000",X"0006",X"0000",X"0098",X"81B9",X"0FFE",X"0FFE",
												  X"0011",X"0031",X"0007",X"0FE4",X"0FEB",X"0FF9",X"009E",X"82B7",
												  X"0002",X"0003",X"0005",X"0027",X"0020",X"0FFA",X"0FF0",X"0FFA",
												  X"009F",X"82B7",X"000D",X"000D",X"000E",X"002A",X"0026",X"0007",
												  X"0FFC",X"0005",X"0068",X"0000",X"0007",X"0000",X"0098",X"8025",
												  X"0FFD",X"0FFD",X"0016",X"0031",X"0003",X"0FE6",X"0FEC",X"0FFA",
												  X"0098",X"8819",X"0000",X"0FFD",X"0017",X"0030",X"0000",X"0FE4",
												  X"0FEE",X"0FFA",X"009C",X"8123",X"0FF8",X"0FF7",X"0FFA",X"0014",
												  X"0009",X"0FEE",X"0FEA",X"0FF0",X"009C",X"8917",X"0FF8",X"0FF8",
												  X"0FFA",X"0015",X"000A",X"0FEE",X"0FEA",X"0FF3",X"009D",X"8123",
												  X"0004",X"0003",X"0005",X"0021",X"0014",X"0FF8",X"0FF5",X"0FFD",
												  X"009D",X"8917",X"0001",X"0002",X"0004",X"0021",X"0014",X"0FF8",
												  X"0FF5",X"0FFF",X"009E",X"8123",X"0004",X"0002",X"0004",X"002A",
												  X"001C",X"0FF8",X"0FF1",X"0FFC",X"009E",X"8917",X"0003",X"0003",
												  X"0005",X"002B",X"001B",X"0FF4",X"0FEF",X"0FFC",X"009F",X"8123",
												  X"000E",X"000D",X"000D",X"002C",X"0020",X"0004",X"0FFE",X"0008",
												  X"009F",X"8917",X"000E",X"000E",X"000D",X"002C",X"0021",X"0003",
												  X"0FFD",X"0007",X"002C",X"0000",X"0008",X"0000",X"0098",X"8683",
												  X"0FFE",X"0FFD",X"0FFF",X"001C",X"002F",X"0FFF",X"0FE5",X"0FEE",
												  X"009C",X"8782",X"0FF8",X"0FF9",X"0FFD",X"0016",X"0007",X"0FEE",
												  X"0FE9",X"0FF2",X"009E",X"8782",X"0003",X"0003",X"0009",X"002A",
												  X"0016",X"0FF5",X"0FF2",X"0FFD",X"009F",X"8782",X"000C",X"000B",
												  X"000F",X"002B",X"001B",X"0002",X"0FFF",X"0008");

	constant RxSize1 : Integer := 51;
	Type RxOutArray1 is Array(0 to RxSize1 - 1) of std_logic_vector(15 downto 0);
	signal RxOuts1 : RxOutArray1;
	constant PhyRxVal1 : RxOutArray1 := (X"0030",X"5555",X"55D5",
												  X"0004",X"0000",X"0005",X"0000",X"0004",X"0000",X"0006",X"0000",
												  X"0004",X"0000",X"0007",X"0000",X"0004",X"0000",X"0008",X"0000",
												  X"0004",X"0000",X"0001",X"0000",X"0004",X"0000",X"0002",X"0000",
												  X"0004",X"0000",X"0003",X"0000",X"0004",X"0000",X"0004",X"0000",
												  X"0004",X"0000",X"0005",X"0000",X"0004",X"0000",X"0006",X"0000",
												  X"0004",X"0000",X"0007",X"0000",X"0004",X"0000",X"0008",X"0000");


	Type PacketArray is Array(0 to 8) of std_logic_vector(15 downto 0);
	constant TrigPacket : PacketArray := (X"0000",X"0020",X"3456",X"4567",
													  X"5678",X"0000",X"0000",X"0000",X"239D");
	constant MDIODat0 : std_logic_vector(15 downto 0) := X"4792";
	constant MDIODat1 : std_logic_vector(15 downto 0) := X"E31B";

   -- Clock period definitions
	Type Delay_Array is Array(0 to 7) of time;
	constant Delays : Delay_Array := (100 ns, 250 ns, 500 ns, 320 ns, 
												  80 ns, 700 ns, 175 ns, 640 ns);
   constant ClkB_P_period : time := 6.25 ns;
   constant ClkB_N_period : time := 6.25 ns;
   constant Clk50MHz_period : time := 20 ns;
   constant Clk25MHz_period : time := 40 ns;
   constant RxClk_period : time := 39.99 ns;
   constant TxClk_period : time := 40 ns;
   constant PhyClk_period : time := 40 ns;

BEGIN

-- Instantiate the Unit Under Test (UUT)
   uut: Controller_FPGA2 PORT MAP (
          VXO_P => VXO_P, VXO_N => VXO_N,
          ClkB_P => ClkB_P,ClkB_N => ClkB_N,
          Clk50MHz => Clk50MHz, Clk25MHz => Clk25MHz,
          CpldRst => CpldRst, CpldCS => CpldCS,
          uCRd => uCRd, uCWr => uCWr, uCA => uCA,
          uCD => uCD, GA => GA, SDCKE => SDCKE,
          LDM => LDM,UDM => UDM, RAS => RAS,
          CAS => CAS,SDWE => SDWE,
          SDClk_P => SDClk_P,SDClk_N => SDClk_N,
          SDD => SDD,UDQS => UDQS,LDQS => LDQS,
          SDRzq => SDRzq,SDA => SDA,BA => BA,
          LinkClk_P => LinkClk_P,LinkClk_N => LinkClk_N,
          LinkFR_P => LinkFR_P,LinkFR_N => LinkFR_N,
          LinkD_P => LinkD_P,LinkD_N => LinkD_N,
          RxDA => RxDA,RxDB => RxDB,RxDC => RxDC,RxDD => RxDD,
          RxDE => RxDE,RxDF => RxDF,RxDG => RxDG,RxDH => RxDH,
          RxClk => RxClk, RxDV => RxDV, RxErr => RxErr, CRS => CRS,
          TxDA => TxDA,TxDB => TxDB,TxDC => TxDC,TxDD => TxDD,
          TxDE => TxDE,TxDF => TxDF,TxDG => TxDG,TxDH => TxDH,
          TxEn => TxEn,MDC => MDC,MDIO => MDIO,PhyPDn => PhyPDn,
          PhyRst => PhyRst,TxClk => TxClk,FMRx => FMRx,
          FMRxEn => FMRxEn,HrtBtFM => HrtBtFM,DReqFM => DReqFM,
			 SPICS => SPICS,SPISClk => SPISClk, SPIMOSI => SPIMOSI,
			 SPIMISO => SPIMISO,Debug => Debug);

-- FM transmitter for requesting event data from the first level FPGAs
TrigTx : FM_Tx
	generic map (Pwidth => 16)
		 port map(clock => Clk100MHz, 
					 reset => ResetHi,
					 Enable => TrgTxEn,
					 Data => TrigRxBuff_Out,
					 Tx_Out => TrigOuts);
DReqFM <= TrigOuts.FM;
HrtBtFM <= TrigOuts.FM;
LVDSTx : FM_Tx
	generic map (Pwidth => 16)
		 port map(clock => Clk100MHz, 
					 reset => ResetHi,
					 Enable => TrgTxEn,
					 Data => TrigRxBuff_Out,
					 Tx_Out => LVDSOuts);
FMRx(0) <= LVDSOuts.FM;
FMRx(1) <= LVDSOuts.FM;
FMRx(2) <= LVDSOuts.FM;
FMRx(3) <= LVDSOuts.FM;
FMRx(4) <= LVDSOuts.FM;
FMRx(5) <= LVDSOuts.FM;
FMRx(6) <= LVDSOuts.FM;
FMRx(7) <= LVDSOuts.FM;


ResetHi <= not CpldRst;

TrgTxEn <= '1' when FMTxEn = '1' and TrigOuts.Done = '0' else '0';

-- Clock process definitions
   ClkB_process :process
   begin
		VXO_P <= '0';
		VXO_N <= '1';
		ClkB_P <= '0';
		ClkB_N <= '1';
		wait for ClkB_P_period/2;
		VXO_P <= '1';
		VXO_N <= '0';
		ClkB_P <= '1';
		ClkB_N <= '0';
		wait for ClkB_P_period/2;
   end process;
 
   Clk50MHz_process :process
   begin
		Clk50MHz <= '0';
		wait for Clk50MHz_period/2;
		Clk50MHz <= '1';
		wait for Clk50MHz_period/2;
   end process;

   Clk100MHz_process :process
   begin
		Clk100MHz <= '0';
		wait for 5 ns;
		Clk100MHz <= '1';
		wait for 5 ns;
   end process;


 
   RxClk_process : process
   begin
		iRxClk <= (others => '0');
		wait for RxClk_period/2;
		iRxClk <=  (others => '1');
		wait for RxClk_period/2;
   end process;
 
-- Stimulus process
   stim_proc: process
   begin
		CpldRst <= '0'; RxDatEnable <= '0';
		GA <= "00"; TxDelay <= '0'; CRSEn <= '0';
      -- hold reset state for 100 ns.
      wait for 100 ns;	
		CpldRst <= '1';
		wait for 1 us;
		RxDatEnable <= '1';
		CRSEn <= '1';
		wait for 1 us;
		TxDelay <= '1';
		wait for 65 us; --12600 ns; --8559 ns;
		CRSEn <= '0';
      wait;
   end process;

	TxClk <= (others => not Clk25MHz);

--	FMRx(0) <= Clk25MHz; 
--	FMRx(1) <= Clk25MHz; --'0';
--	FMRx(2) <= '0';
--	FMRx(3) <= '0';--Clk25MHz; 
--	FMRx(4) <= '0';--Clk25MHz; 
--	FMRx(5) <= '0';--Clk25MHz; 
--	FMRx(6) <= '0';--Clk25MHz; 
--	FMRx(7) <= '0';--Clk25MHz; 

FMTxProc : process(CpldRst,Clk100MHz)

	Variable Index : Integer;

	begin

	if CpldRst = '0' then

	Index := 0; FMTxEn <= '0';

 elsif rising_edge(Clk100MHz) then
	
		if Index = 0 and TxDelay = '1' then FMTxEn <= '1'; 
	 elsif TrigOuts.Done = '0' and Index = 8 then FMTxEn <= '0';
	 else FMTxEn <= FMTxEn;
	 end if;

	if Index < 8 then
	
		if TrigOuts.Done = '1' then Index := Index + 1;
		else Index := Index;
		end if;		
		
	end if;

	if  Index <= 8 then TrigRxBuff_Out <= TrigPacket(Index);
	else TrigRxBuff_Out <= TrigRxBuff_Out;
	end if;
	
	end if;

end process;


uCWrite : process 
	begin

	uCWr <= '1'; uCRd <= '1'; CpldCS <= '1';
	uCA <= (Others => 'Z');	uCD <= (others => 'Z');

	wait until CpldRst = '1';

  wait for 100 ns;	

	 PhyTxDat <= X"1234";

	for i in 0 to 7 loop
-- Phy Tx Writes
		  uCA <= X"301";
 		  wait for 5 ns;
		  CpldCS <= '0';
		  wait for 10 ns;
		  uCD <= PhyTxDat;
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA <= (Others => 'Z');
		  uCD <= (others => 'Z');
		  wait for 67 ns;

		 PhyTxDat <= PhyTxDat + X"1111";
	 
	end loop;

-- Phy Tx CSR Writes
		  uCA <= X"302";
 		  wait for 5 ns;
		  CpldCS <= '0';
		  wait for 10 ns;
		  uCD <= X"0001";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA <= (Others => 'Z');
		  uCD <= (others => 'Z');
		  wait for 67 ns;

  wait for 1 us;	

		  uCA <= X"000";
 		  wait for 5 ns;
		  CpldCS <= '0';
		  wait for 10 ns;
		  uCD <= X"00A8";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA <= (Others => 'Z');
		  uCD <= (others => 'Z');
		  wait for 67 ns;

  wait for 1 us;	

		  uCA <= X"010";
 		  wait for 5 ns;
		  CpldCS <= '0';
		  wait for 10 ns;
		  uCD <= X"1234";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA <= (Others => 'Z');
		  uCD <= (others => 'Z');
		  wait for 67 ns;

		  uCA <= X"010";
 		  wait for 5 ns;
		  CpldCS <= '0';
		  wait for 10 ns;
		  uCD <= X"5678";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA <= (Others => 'Z');
		  uCD <= (others => 'Z');
		  wait for 67 ns;

   wait for 8.5 us;	
--
--		  uCA <= X"000";
-- 		  wait for 5 ns;
--		  CpldCS <= '0';
--		  wait for 10 ns;
--		  uCD <= X"0100";
--		  uCWr <= '0';
--		  wait for 30 ns;
--		  uCWr <= '1';
--		  CpldCS <= '1';
--		  wait for 10 ns;
--		  uCA <= (Others => 'Z');
--		  uCD <= (others => 'Z');
--		  wait for 67 ns;
		  
  wait for 20 us;	

		  uCA <= X"002";
 		  wait for 5 ns;
		  CpldCS <= '0';
		  wait for 10 ns;
		  uCD <= X"0001";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA <= (Others => 'Z');
		  uCD <= (others => 'Z');
		  wait for 67 ns;

-- Set the write address
		  uCA <= X"003";
 		  wait for 5 ns;
		  CpldCS <= '0';
		  wait for 10 ns;
		  uCD <= X"1000";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA <= (Others => 'Z');
		  uCD <= (others => 'Z');

--		  uCA <= X"007";
-- 		  wait for 5 ns;
--		  CpldCS <= '0';
--		  wait for 10 ns;
--		  uCD <= X"B679";
--		  uCWr <= '0';
--		  wait for 30 ns;
--		  uCWr <= '1';
--		  CpldCS <= '1';
--		  wait for 10 ns;
--		  uCA <= (Others => 'Z');
--		  uCD <= (others => 'Z');
--
--		  uCA <= X"007";
-- 		  wait for 5 ns;
--		  CpldCS <= '0';
--		  wait for 10 ns;
--		  uCD <= X"4EDE";
--		  uCWr <= '0';
--		  wait for 30 ns;
--		  uCWr <= '1';
--		  CpldCS <= '1';
--		  wait for 10 ns;
--		  uCA <= (Others => 'Z');
--		  uCD <= (others => 'Z');
--		  wait for 67 ns;
--		  
--		  uCA <= X"007";
-- 		  wait for 5 ns;
--		  CpldCS <= '0';
--		  wait for 10 ns;
--		  uCD <= X"0001";
--		  uCWr <= '0';
--		  wait for 30 ns;
--		  uCWr <= '1';
--		  CpldCS <= '1';
--		  wait for 10 ns;
--		  uCA <= (Others => 'Z');
--		  uCD <= (others => 'Z');
--		  wait for 67 ns;

		  wait;

   end process;

MDIO(0) <= MDIO_Shift_Out0(15) when MDIO_BitCount < 16 and MDIO_State = Shift else 'Z';
MDIO(1) <= MDIO_Shift_Out1(15) when MDIO_BitCount < 16 and MDIO_State = Shift else 'Z';

MDIO_Rd : process(CpldRst,MDC(0))

begin
	if CpldRst = '0' then

	MDIO_Shift_Out0 <= (others => '0');
	MDIO_Shift_Out0 <= (others => '0');
	MDIO_BitCount <= (others => '0');
	MDIO_Shift_In0 <= X"0"; MDIO_Shift_In1 <= X"0";
	MDIO_State <= Idle;

 elsif falling_edge(MDC(0)) then

 MDIO_Shift_In0 <= MDIO_Shift_In0(2 downto 0) & MDIO(0);
 MDIO_Shift_In1 <= MDIO_Shift_In1(2 downto 0) & MDIO(1);

 if MDIO_BitCount = 16 and MDIO_State = Shift then
	MDIO_Shift_Out0 <= MDIODat0;
	MDIO_Shift_Out1 <= MDIODat1;
 elsif MDIO_BitCount < 16 and MDIO_State = Shift then
	MDIO_Shift_Out0 <= MDIO_Shift_Out0(14 downto 0) & '0';
	MDIO_Shift_Out1 <= MDIO_Shift_Out1(14 downto 0) & '0';
 else 
	MDIO_Shift_Out0 <= MDIO_Shift_Out0;
	MDIO_Shift_Out1 <= MDIO_Shift_Out1;
 end if;

-- Idle,Ld_RdBitCount,Ld_WrtBitCount,Wait_Write,Shift,Done
 Case MDIO_State is
	When Idle =>
	 if MDIO_Shift_In0 = "0110" or MDIO_Shift_In0 = "0110"
	 then MDIO_State <= Ld_RdBitCount;
	 elsif MDIO_Shift_In0 = "0101" or MDIO_Shift_In0 = "0101" 
	 then MDIO_State <= Ld_WrtBitCount;
	  else MDIO_State <= Idle;
	 end if;
	When Ld_RdBitCount => MDIO_State <= Shift;
	When Ld_WrtBitCount => MDIO_State <= Wait_Write;
	When Wait_Write =>
		if MDIO_BitCount = 0 then
		MDIO_State <= Done;
		else 
		MDIO_State <= Wait_Write;
		end if;
 	When Shift =>
		if MDIO_BitCount = 0 then
		MDIO_State <= Done;
		else 
		MDIO_State <= Shift;
		end if;
	When Done => MDIO_State <= Idle;
end case;

if MDIO_State = Ld_RdBitCount or MDIO_State = Ld_WrtBitCount then MDIO_BitCount <= "11001";
elsif MDIO_State = Shift or MDIO_State = Wait_Write then MDIO_BitCount <= MDIO_BitCount - 1;
else MDIO_BitCount <= MDIO_BitCount;
end if;

end if; -- CpldRst;

end process MDIO_Rd;

RxDA <= RxIns(0) after 15 ns; RxDB <= RxIns(1) after 15 ns; RxDC <= RxIns(2) after 15 ns; RxDD <= RxIns(3) after 15 ns;
RxDE <= RxIns(4) after 15 ns; RxDF <= RxIns(5) after 15 ns; RxDG <= RxIns(6) after 15 ns; RxDH <= RxIns(7) after 15 ns;

PhyRxGen : for i in 0 to 7 generate

RxClk(i) <= iRxClk(i);
RxDV(i) <= iRxDV(i) after 15 ns; 
CRS(i)  <= iCRS(i) after 15 ns;

PhyRxProc: process(CpldRst,iRxClk)

	Variable Index0 : Integer range 0 to RxSize0 - 1;
	Variable Index1 : Integer range 0 to RxSize1 - 1;

begin

	if CpldRst = '0' then

	NibbleCount(i) <= "00";  RxOuts0(i) <= PhyRxVal0(0); RxOuts1(i) <= PhyRxVal1(0);
	RxIns(i) <= X"0"; RxErr(i) <= '1'; iRxDV(i) <= '0';
	iCRS(i) <= '0'; Index0 := 0; Index1 := 0;

   elsif rising_edge(iRxClk(i)) then

	if RxDatEnable = '1' and Index0 < RxSize0 - 1 then

	if CRS(i) = '1' then NibbleCount(i) <= NibbleCount(i) + '1';
	else NibbleCount(i) <= "00";
	end if;

	RxErr(i) <= '0';

	if RxErr(i) = '0' and CRSEn = '1' then iCRS(i) <= '1'; else iCRS(i) <= '0'; end if;
	
	if iCRS(i) = '1' then iRxDV(i) <= '1'; else iRxDV(i) <= '0'; end if;
	
	 if NibbleCount(i) = 3 then 
		RxOuts0(i) <= PhyRxVal0(Index0);
		Index0 := Index0 + 1;
	 else RxOuts0(i) <= RxOuts0(i);
	 end if;
  if i = 0 or i = 2 or i = 4 or i = 6  
  then
  Case NibbleCount(i) is
	 when "00" => RxIns(i) <= RxOuts0(i)(3 downto 0);
	 when "01" => RxIns(i) <= RxOuts0(i)(7 downto 4);
	 when "10" => RxIns(i) <= RxOuts0(i)(11 downto 8);
	 when "11" => RxIns(i) <= RxOuts0(i)(15 downto 12);
	 when others => RxIns(i) <= RxOuts0(i)(3 downto 0);
  end case;
  end if; -- i = 0
 end if; -- RxDatEnable

 if RxDatEnable = '1' and Index1 < RxSize1 - 1 then

	 if NibbleCount(i) = 3 then 
		RxOuts1(i) <= PhyRxVal1(Index1);
		Index1 := Index1 + 1;
	 else RxOuts1(i) <= RxOuts1(i);
	 end if;

  if i = 1 or i = 3 or i = 5 or i = 7  
  then
   Case NibbleCount(i) is
	 when "00" => RxIns(i) <= RxOuts1(i)(3 downto 0);
	 when "01" => RxIns(i) <= RxOuts1(i)(7 downto 4);
	 when "10" => RxIns(i) <= RxOuts1(i)(11 downto 8);
	 when "11" => RxIns(i) <= RxOuts1(i)(15 downto 12);
	 when others => RxIns(i) <= RxOuts1(i)(3 downto 0);
 end Case;
 end if; -- i = 1 
 end if;
 
end if; -- CpldRst

end process PhyRxProc;

end generate;	

END;

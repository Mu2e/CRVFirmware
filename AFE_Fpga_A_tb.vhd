----------------------- FM Serializer  ------------------------

-- Transmits an FM serial stream at 1/4 the clock rate. 
-- A 100MHz clock encodes 25MHz FM serial data
-- In lieu of a start bit, a preamble of two 1.5 bit periods (like Mil-1553) 
-- is appended to the FM bit stream

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use work.Proj_Defs.all;

--entity FM_Tx is
--	generic (Pwidth : positive);
--		 port(clock,reset,Enable : in std_logic;
--				Data : in std_logic_vector(Pwidth - 1 downto 0);
--				Tx_Out : buffer TxOutRec);
--end FM_Tx;

architecture behavioural of FM_Tx is

-- Serializer state machine
Type FMTx is (TxIdle,TxStrtA,TxStrtB,ShftTx,ParityTx);
signal Tx_State : FMTx;

-- Shift register, bit width counter
signal TxShft : std_logic_vector (Pwidth-1 downto 0);
signal TxBitWdth : std_logic_vector (2 downto 0);
-- Transmitted FM data, running parity bit
signal Parity,Tx_Req : std_logic;
signal EnDL : std_logic_vector (1 downto 0);

begin

FM_Encode : process(clock, reset)
-- Frame bit counter
variable TxBtCnt : integer range 0 to Pwidth-1;

begin
 if reset = '1' then 

	Tx_State <= TxIdle; Tx_Out.FM <= '0';
	Tx_Out.Done <= '0'; Parity <= '0';
	TxShft <= (others => '0'); Tx_Req <= '0';
	TxBitWdth <= "000"; TxBtCnt := 0;
	EnDL <= "00";

elsif rising_edge(clock) then

	EnDL(0) <= Enable;
	EnDL(1) <= EnDL(0);

	    if Tx_Req = '0' and Tx_State = TxIdle and EnDL = 1 then Tx_Req <= '1';
	elsif  Tx_Req = '1' and Tx_State = TxStrtA then Tx_Req <= '0';
	else Tx_Req <= Tx_Req;
	end if;

   Case TxBitWdth is
	When "000" => TxBitWdth <= "001"; 
	When "001" => if Tx_Req = '1' then TxBitWdth <= "000"; else TxBitWdth <= "010"; end if;
	When "010" => TxBitWdth <= "011";
	When "011" => if Tx_State = TxStrtA or Tx_State = TxStrtB
			  then TxBitWdth <= "100";
			  else TxBitWdth <= "000";
			  end if;
	When "100" => if Tx_State = TxStrtA or Tx_State = TxStrtB
			  then TxBitWdth <= "101";
			  else TxBitWdth <= "000";
			  end if;
	When others => TxBitWdth <= "000";
  end Case;

-- FMTx TxIdle,TxStrtA,TxStrtB,ShftTx,ParityTx
Case Tx_State is
-- Send data on start
        When TxIdle => 
	 	 if Tx_Req = '1' and (TxBitWdth = "001" or TxBitWdth = "011")
		  then Tx_State <= TxStrtA;
			else Tx_State <= TxIdle;
			end if;
		When TxStrtA =>
		 if TxBitWdth = "101" then Tx_State <= TxStrtB;
		  else Tx_State <= TxStrtA;
		 end if;
 		When TxStrtB =>
		 if TxBitWdth = "101" then Tx_State <= ShftTx;
		  else Tx_State <= TxStrtB;
		 end if;
          When ShftTx =>
         if TxBitWdth = "011" and TxBtCnt = 0 then Tx_State <= ParityTx;
         else Tx_State <= ShftTx;
         end if;
           When ParityTx =>
         if TxBitWdth = "011" then Tx_State <= TxIdle;
         else Tx_State <= ParityTx;
         end if;
end case;

-- Two transitions per bit period is a 1, one transition a 0
 -- default state is a string of 1's
if ((TxBitWdth = "001" or TxBitWdth = "011") and Tx_State = TxIdle)
		  or TxBitWdth = "101" 	-- Start bit is defined 1 1/2 bit periods
					-- Number of data FM transitions is ShiftOut register data dependent
          or (Tx_State = ShftTx and ((TxShft(Pwidth-1) = '1' and TxBitWdth = "001") or TxBitWdth = "011"))
					-- Number of parity FM transitions is parity bit dependent
          or (Tx_State = ParityTx and ((Parity = '0' and TxBitWdth = "001") or TxBitWdth = "011"))
then Tx_Out.FM <= not Tx_Out.FM;
else Tx_Out.FM <= Tx_Out.FM;
end if;

-- data frames are "width" bits long 
if Tx_State = TxStrtB and TxBitWdth = "101"
  then TxBtCnt := (Pwidth-1);
elsif Tx_State = TxIdle then TxBtCnt := 0;
elsif Tx_State = ShftTx and TxBitWdth = "011" and TxBtCnt /= 0
	then TxBtCnt := TxBtCnt-1;
else TxBtCnt := TxBtCnt;
end if;
-- Load shift register with data byte at the beginning of the transmit sequence
-- load condition
if Tx_State = TxIdle and EnDL = 1
  then TxShft <= Data;
-- Shift one bit left (MSB first) during data portion of frame
-- shift condition
elsif Tx_State = ShftTx and TxBitWdth = "011" 
	then TxShft <= (TxShft(Pwidth-2 downto 0) & '0');
else TxShft <= TxShft;
end if;

  if (Parity = '1' and Tx_State = TxIdle) -- reset parity at start
  or (Tx_State = ShftTx and TxBitWdth = "011" and TxShft(Pwidth-1) = '0')
-- Toggle parity bit with each shifted out "0"
then Parity <= not Parity;
else Parity <= Parity;
end if;

-- Indicate when a frame has been shifted out
if TxBitWdth = "011" and Tx_State = ParityTx then Tx_Out.Done <= '1';
else Tx_Out.Done <= '0';
end if;

end if; -- reset

end process FM_Encode;

end behavioural; -- of Serial_Tx

--------------------------------------------------------------------------------
-- Company: Fermilab
-- Engineer: Sten Hansen
--
-- Create Date:   17:58:32 11/11/2014
-- Design Name:   AFE_Fpga_A
-- Module Name:   C:/Experiments/mu2e/SiPMFE/FPGA/AFE_Fpga_A/AFE_Fpga_A_tb.vhd
-- Project Name:  AFE_Fpga_A
-- Target Device: XC6LX25-3csg324 
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: FEB_Fpga_A
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
use work.Proj_defs.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
USE ieee.numeric_std.ALL;
 
ENTITY AFE_Fpga_A_tb IS
END AFE_Fpga_A_tb;
 
ARCHITECTURE behavior OF AFE_Fpga_A_tb IS 

-- Component Declaration for the Unit Under Test (UUT)

COMPONENT FEB_Fpga_A
 PORT(
-- 159.3 MHz VXO clock
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
-- AFE Input clock
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
	Debug : buffer std_logic_vector(10 downto 1));

END COMPONENT;

--Inputs
	signal ClkB_P,ClkB_N : std_logic := '0';
   signal VXO_P : std_logic := '0';
   signal VXO_N : std_logic := '0';
   signal CpldRst : std_logic := '0';
   signal CpldCS : std_logic := '0';
   signal uCRd : std_logic := '0';
   signal uCWr : std_logic := '0';
   signal uCA : std_logic_vector(11 downto 0) := (others => '0');
   signal GA : std_logic_vector(1 downto 0) := (others => '0');
   signal AFESDO : std_logic := '0';
   signal AFEDCO_P : std_logic_vector(1 downto 0) := (others => '0');
   signal AFEDCO_N : std_logic_vector(1 downto 0) := (others => '0');
   signal AFEFR_P : std_logic_vector(1 downto 0) := (others => '0');
   signal AFEFR_N : std_logic_vector(1 downto 0) := (others => '0');
   signal AFEDat0_P : std_logic_vector(7 downto 0) := (others => '0');
   signal AFEDat0_N : std_logic_vector(7 downto 0) := (others => '0');
   signal AFEDat1_P : std_logic_vector(7 downto 0) := (others => '0');
   signal AFEDat1_N : std_logic_vector(7 downto 0) := (others => '0');
   signal GPI1,GPI0_N,GPI0_P : std_logic;
   signal A7,R10, N8 : std_logic;

	--BiDirs
   signal uCD : std_logic_vector(15 downto 0);
   signal SDD : std_logic_vector(15 downto 0);
   signal UDQS,LDQS,SDRzq : std_logic;
   signal Temp : std_logic_vector(3 downto 0);
 
 	--Outputs
   signal SDCKE,LDM,UDM,RAS,CAS,SDWE,SDClk_P,SDClk_N : std_logic;
   signal SDA : std_logic_vector(13 downto 0);
   signal BA : std_logic_vector(1 downto 0);
   signal AFEPDn,AFECS,AFEClk_P,AFEClk_N : std_logic_vector(1 downto 0);
   signal AFESClk,AFESDI,AFERst : std_logic;
   signal MuxEn : std_logic_vector(3 downto 0);
   signal Muxad : std_logic_vector(1 downto 0);
   signal Pulse,PulseSel : std_logic;
   signal DACCS : std_logic_vector(2 downto 0);
   signal DACClk,DACDat,DACLd : std_logic;
   signal Debug : std_logic_vector(10 downto 1);

-- user defined types

Type Seed_Array is Array(0 to 7) of std_logic_vector(23 downto 0);
    constant Seed0 : Seed_Array := (X"47A95B",X"246865",X"F73B46",X"7EC0C4",
											   X"A7D4F3",X"CAF367",X"96A262",X"34B491");
    constant Seed1 : Seed_Array := (X"4B759A",X"F44C29",X"16CAF2",X"30761F",
											   X"D22371",X"599FDE",X"629A5E",X"E4911B");

Constant FM_size : Integer := 48;
Type Word_FM_array is Array(0 to FM_size - 1) of std_logic_vector(15 downto 0);
Constant FM_Words : Word_FM_array := (x"8002",x"8002",x"957D",x"AEF0",
                                      x"9210",x"9A98",x"A654",x"9654",
												  x"9122",x"9344",x"9566",x"9788",
                                      x"ABCC",x"ADEE",x"9F00",x"8003",
												  x"1DED",x"8004",x"3AA9",x"8787",
												  x"1122",x"3344",x"5566",x"7788",
                                      x"09AA",x"0BCC",x"3DEE",x"FF00",
												  x"1DED",x"8004",x"3AA9",x"8787",
												  x"1122",x"3344",x"5566",x"7788",
                                      x"09AA",x"3DEE",x"FF00",x"8004",
												  x"1DED",x"2BCB",x"3AA9",x"8787",
                                      x"6565",x"4343",x"2121",x"8004");

Type TxReg_Array is Array(0 to 7) of std_logic_vector(11 downto 0);	
signal TxReg0,TxReg1 : TxReg_Array;

Type DDR_Tap_Array is Array(0 to 7) of std_logic_vector(3 downto 0);
signal DDR_Tap0,DDR_Tap1 : DDR_Tap_Array;

Type DDR_LSFR_Array is Array(0 to 7) of std_logic_vector(23 downto 0);
signal DDR_LSFR0,DDR_LSFR1 : DDR_LSFR_Array;

   -- Clock period definitions
	constant VXO_period : time := 10 ns; -- 6.277 ns;
	constant DCO_period : time := 2.083333 ns;
	constant RF_Period : time := 18.831 ns;

signal iAFEFR : std_logic_vector(11 downto 0);
signal AFEDCODL : std_logic_vector(1 downto 0);

signal Gap_Count : std_logic_vector(3 downto 0);

signal IntDQSClk,iDQS : std_logic;
signal BurstCount : std_logic_vector(3 downto 0);
signal DDR_LSFR : std_logic_vector(23 downto 0);
signal DDR_Tap : std_logic_vector(3 downto 0);
Type DDR_Rx is (Idle,WaitCS0,WaitCS1,WaitCS2,WaitCS3,EnableDQS,DisableDQS);
signal DDR_Rx_State : DDR_Rx;

signal BitCount : std_logic_vector(3 downto 0);
signal RdReg : std_logic_vector(15 downto 0);
signal FMWord : std_logic_vector(23 downto 0);  
signal PresentAD : AddrPtr;
signal ResetHi,TxEn,Clk53,TxReq,TxAck : std_logic;
signal uBunchWidth : std_logic_vector(7 downto 0);  
signal Tx1_Out : TxOutRec;

Type FM_State is (Idle, SetEnable, WaitEn, ClrEn);
signal Tx_State : FM_State;

signal FM_Index : integer range 0 to FM_size;

component FM_Tx is
	generic (Pwidth : positive);
		 port(clock,reset,Enable : in std_logic;
				Data : in std_logic_vector(Pwidth - 1 downto 0);
				Tx_Out : buffer TxOutRec);
end component;

BEGIN

-- Instantiate the Unit Under Test (UUT)
   uut: FEB_Fpga_A PORT MAP (
   VXO_P => VXO_P, VXO_N => VXO_N, 
	ClkB_P => ClkB_P, ClkB_N => ClkB_N,
   CpldRst => CpldRst,CpldCS => CpldCS,
   uCRd => uCRd,uCWr => uCWr, uCA => uCA,
   uCD => uCD, GA => GA, SDCKE => SDCKE,
   LDM => LDM, UDM => UDM,
   RAS => RAS, CAS => CAS,
   SDWE => SDWE, SDClk_P => SDClk_P,
   SDClk_N => SDClk_N,  SDD => SDD,
   UDQS => UDQS, LDQS => LDQS,
   SDRzq => SDRzq, SDA => SDA,
   BA => BA, AFEPDn => AFEPDn,
   AFECS => AFECS, AFESClk => AFESClk,
   AFESDI => AFESDI, AFERst => AFERst, AFESDO => AFESDO,
   AFEDCO_P => AFEDCO_P, AFEDCO_N => AFEDCO_N,
   AFEFR_P => AFEFR_P, AFEFR_N => AFEFR_N,
   AFEDat0_P => AFEDat0_P, AFEDat0_N => AFEDat0_N,
   AFEDat1_P => AFEDat1_P, AFEDat1_N => AFEDat1_N,
   AFEClk_P => AFEClk_P, AFEClk_N => AFEClk_N,
   MuxEn => MuxEn, Muxad => Muxad,
   Pulse => Pulse, PulseSel => PulseSel,
   DACCS => DACCS, DACClk => DACClk,
   DACDat => DACDat, DACLd => DACLd,
   Temp => Temp, A7 => A7,Debug => Debug,
	GPI1 => GPI1,GPI0_N => GPI0_N ,GPI0_P => GPI0_P);

FMTx1 : FM_Tx 
	generic map (Pwidth => 24)
	 port map(clock => VXO_P,
				reset => ResetHi,
				Enable => TxEn,
				Data => FMWord,
				Tx_Out => Tx1_Out);
GPI0_P <= Tx1_Out.FM;
GPI0_N <= not Tx1_Out.FM;

Initproc : process
	begin
	 CpldRst <= '0'; Gap_Count <= X"0"; GPI1 <= '0';
	 ResetHi <= '1';
	wait for 100 ns;
	 CpldRst <= '1';
	 ResetHi <= '0';
	 wait;
end process;

ExtTrig : process
	begin
	 N8 <= '1'; 
	wait for 3 us;
	 N8 <= '0';
	wait for 50 ns;
	 N8 <= '1'; 
	wait for 13.2 us;
	 N8 <= '0';
	wait for 50 ns;
	 N8 <= '1'; 
	wait for 15.71 us;
	 N8 <= '0';
	wait for 50 ns;
	 N8 <= '1'; 
	wait for 13.39 us;
	 N8 <= '0';
	wait for 50 ns;
	 N8 <= '1'; 
	 wait;
end process;

ExtGate : process
	begin
	 R10 <= '0'; 
	wait for 1 us;
	 R10 <= '1';
	wait for 14 us;
	 R10 <= '0'; 
	wait for 1 us;
	 R10 <= '1';
	wait for 50 us;
	 R10 <= '0'; 
	 wait;
end process;

 AFEDCO_process : process
  begin
	AFEDCO_P <= "00";
	AFEDCO_N <= "11";
		wait for VXO_Period/9.6;
	AFEDCO_P <= "11";
	AFEDCO_N <= "00";
		wait for VXO_Period/9.6;
   end process;

VXO_process : process
 begin
	VXO_P <= '0';
	VXO_N <= '1';
	ClkB_P <= '0';
	ClkB_N <= '1';
 wait for VXO_period/2;
	VXO_P <= '1';
	VXO_N <= '0';
	ClkB_P <= '1';
	ClkB_N <= '0';
 wait for VXO_period/2;
end process;

--GPI0_process : process
-- begin
--	GPI0_P <= '0';
--	GPI0_N <= '1';
--	GPI0_P <= '0';
--	GPI0_N <= '1';
-- wait for 19.9 ns;
--	GPI0_P <= '1';
--	GPI0_N <= '0';
--	GPI0_P <= '1';
--	GPI0_N <= '0';
-- wait for 19.9 ns;
--end process;

RF_process : process
 begin
  Clk53 <= '0';
  wait for RF_Period/2;
  Clk53 <= '1';
  wait for  RF_Period/2;
end process;

uBunchWidthproc : process (Clk53,CpldRst)

	begin

		if CpldRst = '0' then 
			uBunchWidth <= (others => '0');
			TxReq <= '0';
		
		elsif rising_edge(Clk53)

		then
			
		if uBunchWidth /= 89 then uBunchWidth <= uBunchWidth + 1;
		else uBunchWidth <= (others => '0');
		end if;

		if uBunchWidth = 89 then TxReq <= '1';
		elsif TxAck = '1' then TxReq <= '0';
		else TxReq <= TxReq;
		end if;

	end if;
	
end process;


FMTx : Process(VXO_P,CpldRst)

 begin

if CpldRst = '0' then

FM_Index <= 0; TxAck <= '0';
TxEn <= '0'; 
FMWord <= X"112345"; -- Set bit 20 to iondicate beam on heartbeats
Tx_State <= Idle; 

elsif rising_edge(VXO_P )

then

if Tx_State = ClrEn then 
	FMWord(23 downto 20) <= X"3";
	FMWord(19 downto 0) <= FMWord(19 downto 0) + 1;
else FMWord <= FMWord;
end if;

-- Idle, SetEnables, WaitEn0, ClrEn0, WaitEn1, ClrEn1
TxAck <= TxReq;
	
Case Tx_State is
		When Idle =>
		  if TxAck = '1' 
			then Tx_State <= SetEnable;
		  else Tx_State <= Idle;
		  end if;
		 When SetEnable => Tx_State <= WaitEn;
		 When WaitEn =>
			If Tx1_Out.Done = '1' 
				then Tx_State <= ClrEn;
			else Tx_State <= WaitEn;
			end if;
		 when ClrEn => Tx_State <= Idle;
end case;

if TxEn = '0' and Tx_State = SetEnable
 then TxEn <= '1';
elsif Tx_State = ClrEn 
 then TxEn <= '0';
else TxEn <= TxEn;
end if;

end if; --CpldRst

end process FMTx;


gen8 : for i in 0 to 7 generate

-- Stimulus process
stim_proc: process(CpldRst,AFEClk_P(0))

begin		

if CpldRst = '0' then

 DDR_Tap0(i) <= "0000";  
 DDR_Tap1(i) <= "0000";  
 DDR_LSFR0(i) <= Seed0(i); 
 DDR_LSFR1(i) <= Seed1(i); 

elsif rising_edge(AFEClk_P(0))

then

	DDR_Tap0(i)(0) <= DDR_LSFR0(i)(23) xor DDR_LSFR0(i)(19);
	DDR_Tap0(i)(1) <= DDR_LSFR0(i)(13) xor DDR_LSFR0(i)(17);
	DDR_Tap0(i)(2) <= DDR_LSFR0(i)(3)  xor DDR_LSFR0(i)(12);
	DDR_Tap0(i)(3) <= DDR_LSFR0(i)(1)  xor DDR_LSFR0(i)(9);
	DDR_LSFR0(i) <= DDR_LSFR0(i)(22 downto 0) & DDR_Tap0(i)(3);

	DDR_Tap1(i)(0) <= DDR_LSFR1(i)(23) xor DDR_LSFR1(i)(19);
	DDR_Tap1(i)(1) <= DDR_LSFR1(i)(13) xor DDR_LSFR1(i)(17);
	DDR_Tap1(i)(2) <= DDR_LSFR1(i)(3)  xor DDR_LSFR1(i)(12);
	DDR_Tap1(i)(3) <= DDR_LSFR1(i)(1)  xor DDR_LSFR1(i)(9);
	DDR_LSFR1(i) <= DDR_LSFR1(i)(22 downto 0) & DDR_Tap1(i)(3);
 end if;

end process;

shift_proc: process(CpldRst,AFEDCO_P(0))

begin

if CpldRst = '0' then

	TxReg0(i) <= X"000";
	TxReg1(i) <= X"000";

	AFEDCODL <= "00";
	iAFEFR <= X"03F";

elsif (AFEDCO_P(0)' event)

then
	AFEDCODL(0) <= AFEClk_P(0);
	AFEDCODL(1) <= AFEDCODL(0);
	if AFEDCODL = "01" then 
-- The framing signal is high at the beginning of the serial word
   iAFEFR <= X"FC0";
-- Load a random value, then shift out
	TxReg0(i) <= DDR_LSFR0(i)(5) & DDR_LSFR0(i)(5) & DDR_LSFR0(i)(5) & DDR_LSFR0(i)(5) & DDR_LSFR0(i)(5) & DDR_LSFR0(i)(5) 
	           & DDR_LSFR0(i)(5) & DDR_LSFR0(i)(4 downto 0);
	TxReg1(i) <= DDR_LSFR1(i)(5) & DDR_LSFR1(i)(5) & DDR_LSFR1(i)(5) & DDR_LSFR1(i)(5) & DDR_LSFR1(i)(5) & DDR_LSFR1(i)(5)  
	           & DDR_LSFR1(i)(5) & DDR_LSFR1(i)(4 downto 0);
	else TxReg0(i) <= '0' & TxReg0(i)(11 downto 1);
	     TxReg1(i) <= '0' & TxReg1(i)(11 downto 1);
	iAFEFR <= iAFEFR(10 downto 0) & iAFEFR(11);
	end if;

end if;

AFEDat0_P(i) <=     TxReg0(i)(0) after 0.5 ns;
AFEDat0_N(i) <= not TxReg0(i)(0) after 0.5 ns;

AFEDat1_P(i) <=     TxReg1(i)(0) after 0.5 ns;
AFEDat1_N(i) <= not TxReg1(i)(0) after 0.5 ns;

AFEFR_P <=     (iAFEFR(11) & iAFEFR(11)) after 0.5 ns;
AFEFR_N <= not (iAFEFR(11) & iAFEFR(11)) after 0.5 ns;

end process;

end generate;

DDRCmd : process

 Variable Loop_Index : integer range 0 to 31;

 begin

	  CpldCS <= '1'; GA <= "00";
     uCRd <= '1'; uCWr <= '1'; 
	  uCA(9 downto 0) <= (others => 'Z');
	  uCA(11 downto 10) <= "00";
 	  uCD <= (others => 'Z');
--	  PresentAD <= FIFORdAddr0(0);
	  
	wait for 200 ns;

-- Issue a DDR reset
		  uCA(9 downto 0) <= CSRRegAddr;
 		  wait for 5 ns;
		  CpldCS <= '0';
		  wait for 10 ns;
        uCD <= X"006C";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');

		  wait for 50 ns;

		  uCA(9 downto 0) <= TrigCtrlAddr;
 		  wait for 5 ns;
		  CpldCS <= '0';
		  wait for 10 ns;
        uCD <= X"0002";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');

		  wait for 50 ns;

 	     uCA(9 downto 0) <= GateAddr;
 		  wait for 5 ns;
		  CpldCS <= '0';
		  uCD <= X"0020";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');

		  wait for 50 ns;

 	     uCA(9 downto 0) <= HistCtrlAd;
 		  wait for 5 ns;
		  CpldCS <= '0';
		  uCD <= X"0020";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');

		  wait for 50 ns;
 	     uCA(9 downto 0) <= HistIntvalAd;
 		  wait for 5 ns;
		  CpldCS <= '0';
		  uCD <= X"0080";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');

		  wait for 50 ns;

--  issue low word write
 	     uCA(9 downto 0) <= PipeLineAddr;
 		  wait for 5 ns;
		  CpldCS <= '0';
        uCD <= X"0010";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');

		  wait for 100 ns;

		  uCA(9 downto 0) <= CSRRegAddr;
 		  wait for 5 ns;
		  CpldCS <= '0';
		  wait for 10 ns;
        uCD <= X"0100";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');

		  wait for 100 ns;


		 uCA(9 downto 0) <= "00" & X"30";
			
--for Loop_Index in 0 to 31 loop

--  issue low word write
 	     uCA(9 downto 0) <= InputMaskAddr;
 		  wait for 5 ns;
		  CpldCS <= '0';
		  uCD <= X"FFFF";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');

		  wait for 100 ns;

--  Enable self trigger
 	    uCA(9 downto 0) <= IntTrgEnAddr; 
 		  wait for 5 ns;
		  CpldCS <= '0';
        uCD <= X"0002";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');

	wait for 200 ns;
-- 	     uCA(9 downto 0) <= IntTrgThreshAd; 
-- 		  wait for 5 ns;
--		  CpldCS <= '0';
--        uCD <= X"001E";
--		  uCWr <= '0';
--		  wait for 30 ns;
--		  uCWr <= '1';
--		  CpldCS <= '1';
--		  wait for 10 ns;
--		  uCA(9 downto 0) <= (Others => 'Z');
--		  uCD <= (others => 'Z');

	wait for 13071 ns;
	
	 	  uCA(9 downto 0) <= GateAddr;
 		  wait for 5 ns;
		  CpldCS <= '0';
		  uCD <= X"0020";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');

		  uCA(9 downto 0) <= CSRRegAddr;
 		  wait for 5 ns;
		  CpldCS <= '0';
		  wait for 10 ns;
        uCD <= X"0000";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');

--end loop;		

--  issue high word write
 	     uCA(9 downto 0) <= TempDat0Ad; --SDRamRdPtrHiAd;
 		  wait for 5 ns;
		  CpldCS <= '0';
        uCD <= X"0044";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');

		  wait for 50 ns;

--  issue low word write
 	     uCA(9 downto 0) <= TempCtrlAd; --SDRamRdPtrLoAd;
 		  wait for 5 ns;
		  CpldCS <= '0';
        if Loop_Index = 0 then uCD <= X"0711"; -- X"FED0"; 
		  else uCD <= X"FF10";
		  end if;
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');

--end loop;		
		  uCD <= (others => 'Z');

for Loop_Index in 0 to 31 loop

 	    uCA(9 downto 0) <= HistRd0Ad;
 	    wait for 5 ns;
		 CpldCS <= '0';
		 uCRd <= '0';
		 wait for 30 ns;
		 uCRd <= '1';
		 CpldCS <= '1';
		 wait for 10 ns;
		 uCA(9 downto 0) <= (Others => 'Z');
		 wait for 50 ns;

 	    uCA(9 downto 0) <= HistRd1Ad;
 	    wait for 5 ns;
		 CpldCS <= '0';
		 uCRd <= '0';
		 wait for 30 ns;
		 uCRd <= '1';
		 CpldCS <= '1';
		 wait for 10 ns;
		 uCA(9 downto 0) <= (Others => 'Z');
		 wait for 50 ns;

end loop;

--  issue low word write
 	     uCA(9 downto 0) <= "00" & X"0B"; --SDRamRdPtrLoAd;
 		  wait for 5 ns;
		  CpldCS <= '0';
		  uCD <= X"0050";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');
	

--  issue low word write
-- 	  uCA(9 downto 0) <= AFEFifoStatAd;
-- 	  wait for 5 ns;
--		  CpldCS <= '0';
--		  uCRd <= '0';
--		  wait for 30 ns;
--		  uCRd <= '1';
--		  CpldCS <= '1';
--		  wait for 10 ns;
--		  uCA(9 downto 0) <= (Others => 'Z');
--		  wait for 50 ns;
--
--		  PresentAD <= PresentAD + 1;

--end loop;		
		 wait for 15 us;

--  Enable self trigger
 	    uCA(9 downto 0) <= IntTrgEnAddr; 
 		  wait for 5 ns;
		  CpldCS <= '0';
        uCD <= X"0000";
		  uCWr <= '0';
		  wait for 30 ns;
		  uCWr <= '1';
		  CpldCS <= '1';
		  wait for 10 ns;
		  uCA(9 downto 0) <= (Others => 'Z');
		  uCD <= (others => 'Z');

		wait;
			
end process;

-- poor man's LPDDR model. Produces data in response to a read command.
DDR_Rd_Data : Process(IntDQSClk,CpldRst)

 begin

if CpldRst = '0' then

 BurstCount <= X"0"; iDQS <= '0';
 UDQS <= 'Z'; LDQS <= 'Z'; SDD <= (others => 'Z');
-- DDR_LSFR <= X"47A95B"; 
 DDR_Tap <= "0000";  
 DDR_LSFR <= (others => '0');

elsif falling_edge(IntDQSClk)

	then

	 DDR_Tap(0) <= DDR_LSFR(23) xor DDR_LSFR(19);
	 DDR_Tap(1) <= DDR_LSFR(13) xor DDR_LSFR(17);
	 DDR_Tap(2) <= DDR_LSFR(3) xor DDR_LSFR(12);
	 DDR_Tap(3) <= DDR_LSFR(1) xor DDR_LSFR(9);
--	 DDR_LSFR <= DDR_LSFR(22 downto 0) & DDR_Tap(3);

-- Idle,WaitCS0,WaitCS1,WaitCS2,WaitCS3,EnableDQS,DisableDQS
	Case DDR_Rx_State is
		When Idle =>
		  if SDCKE = '1' and RAS = '1' and CAS = '0' and SDWE = '1'
			then DDR_Rx_State <= WaitCS0;
		  else DDR_Rx_State <= Idle;
		  end if;
		 When WaitCS0 => 
		   if CAS = '1' then DDR_Rx_State <= WaitCS1;
			else DDR_Rx_State <= WaitCS0;
			end if;
		 When WaitCS1 => DDR_Rx_State <= WaitCS2;
		 When WaitCS2 => DDR_Rx_State <= WaitCS3; 
		 When WaitCS3 => DDR_Rx_State <= EnableDQS;  		 
		 when EnableDQS =>
			if BurstCount = 0
				then DDR_Rx_State <= DisableDQS;
			else DDR_Rx_State <= EnableDQS;
			end if;
		 When DisableDQS => DDR_Rx_State <= Idle;
	end case;

if DDR_Rx_State = WaitCS3 then BurstCount <= X"F";
 elsif DDR_Rx_State = EnableDQS and BurstCount > 0
 then BurstCount <= BurstCount - 1;
 else BurstCount <= BurstCount;
 end if;

  if DDR_Rx_State = WaitCS3 or DDR_Rx_State = EnableDQS then 
iDQS <= not iDQS;
else iDQS <= '0';
end if;

 if DDR_Rx_State = WaitCS3 or DDR_Rx_State = EnableDQS then 
	UDQS <= iDQS; 
	LDQS <= iDQS;
 else UDQS <= 'Z'; LDQS <= 'Z';
 end if;

 if DDR_Rx_State = EnableDQS
 then SDD <= DDR_LSFR(15 downto 0);
			 DDR_LSFR <= DDR_LSFR + 1;
 else SDD <= (others => 'Z');
 	   DDR_LSFR <= DDR_LSFR;
 end if;

end if; -- rising edge

end process	DDR_Rd_Data;


AFE_Rd_Data : Process(AFESClk,AFECS)

begin

if AFECS = 3 then

   BitCount <= "1001";
	RdReg <= DDR_LSFR0(3)(15 downto 0);
	
elsif rising_edge(AFESClk) then

 if BitCount /= 0 then BitCount <= BitCount - 1;
end if;

if BitCount = 0 then RdReg <= RdReg(14 downto 0) & '0'; 
end if;

end if; -- rising edge

end process AFE_Rd_Data;

AFESDO <= RdReg(15) when AFECS /= 3 and BitCount = 0 else 'Z' after 40ns;

-- Use this to generate DDR strobes
intDQSClkGen : process                                           
begin 
-- 200 MHz DDR Rx clock
    IntDQSClk <= '0';
    wait for VXO_period/6.4;
    IntDQSClk <= '1';
  wait for VXO_period/6.4;
end process intDQSClkGen;  

END;


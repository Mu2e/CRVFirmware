----------------------- FM Serializer  ------------------------

-- Transmits an FM serial stream at 1/4 the clock rate. 
-- A 100MHz clock encodes 25MHz FM serial data
-- In lieu of a start bit, a preamble of two 1.5 bit periods (like Mil-1553) 
-- is appended to the FM bit stream

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use work.Project_Defs.all;

entity FM_Tx is
	generic (Pwidth : positive);
		 port(clock,reset,Enable : in std_logic;
				Data : in std_logic_vector(Pwidth - 1 downto 0);
				Tx_Out : buffer TxOutRec);
end FM_Tx;

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

------------------------------ FM Deserializer ----------------------------
-- Receives an FM serial stream at 1/8 the clock rate. 
-- e.g. A 200MHz clock (rxclock) decodes 25MHz FM serial data
-- A preamble of two 1.5 bit periods (like Mil-1553) appended to the FM 
-- serial stream is expected. The done signal is synchronized to sysclk

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
USE work.Project_Defs.all;

entity FM_Rx is
	generic (Pwidth : positive);
	port(SysClk,RxClk,reset : in std_logic;
	Rx_In : in RxInRec;
	data : buffer std_logic_vector(Pwidth-1 downto 0 );
	Rx_Out : buffer RxOutRec);
end FM_Rx;

architecture behavioural of FM_Rx is

Type FMRx is (RxIdle,RxStrt,RxShift,ParityRx);
Signal Rx_State : FMRx;
-- Registers for FM decoder
-- Shift register, bit width counter
signal RxBitWdth : std_logic_vector (3 downto 0);
-- Edge detector for incoming FM data
signal RxDl : std_logic_vector (1 downto 0);
-- Transmitted FM data, running parity bit
signal RxParity,Rx_NRZ,Rx_Done_Req : std_logic;

begin

FM_Decode : process(RxClk, reset)

-- Frame bit counter
variable RxBtCnt : integer range 0 to Pwidth-1;

begin
 if reset = '1' then 

	Rx_State <= RxIdle; RxDl <= "00"; 
	Rx_Done_Req <= '0'; RxParity <= '0'; Rx_Out.Parity_Err <= '0';
	data <= (others => '0'); RxBtCnt := 0; 
	Rx_NRZ <= '0'; RxBitWdth <= "0000";

elsif rising_edge(RxClk) then

-- Synchronous edge detector for input
RxDl(0) <= Rx_In.FM;
RxDl(1) <= RxDl(0);

-- Reset sampling counter with every clock transition while decoder is idle.
-- Otherwise reset only once per bit period
if (RxDl(1) = '1' xor RxDl(0) = '1') and (RxBitWdth > "0100" or Rx_State = RxIdle)
  then RxBitWdth <= "0000";
elsif RxBitWdth /= "1111" and 
	not((RxDl(1) = '1' xor RxDl(0) = '1') and (RxBitWdth > "0100" or Rx_State = RxIdle))
  then RxBitWdth <= RxBitWdth + 1;
else RxBitWdth <= RxBitWdth;
end if;

-- RxIdle,RxStrt,RxShift,ParityRx 
Case Rx_State is
    When RxIdle =>
      if RxBitWdth = 8 then Rx_State <= RxStrt;
       else Rx_State <= RxIdle;
      end if;
    When RxStrt =>
     if RxBitWdth = 8 then Rx_State <= RxShift;
	  elsif ((RxDl(1) = '1' xor RxDl(0) = '1') and RxBitWdth < 8)
		  or RxBitWdth = 15 then Rx_State <= RxIdle;
      else Rx_State <= RxStrt;
     end if;
    When RxShift =>
      if RxBtCnt = 0 and RxBitWdth = 6 then Rx_State <= ParityRx;
	   elsif RxBitWdth = 15 then Rx_State <= RxIdle;
      else Rx_State <= RxShift;
      end if;
     When ParityRx =>
      if RxBitWdth = 6 or RxBitWdth = 15
		then Rx_State <= RxIdle;
     else Rx_State <= ParityRx;
      end if;
end case;

-- Serial data from FM is 1 if transition is in the middle of the bit period,
-- 0 if it is at the end 
if Rx_NRZ = '1' and (RxDl(1) = '1' xor RxDl(0) = '1') and RxBitWdth > 4
then Rx_NRZ <= '0';
elsif  Rx_NRZ = '0' and (RxDl(1) = '1' xor RxDl(0) = '1') and RxBitWdth <= 4
then Rx_NRZ <= '1';
else Rx_NRZ <= Rx_NRZ;
end if;

-- Serial data frame is "width" bits long
   if Rx_State = RxStrt and RxBitWdth = 8 then RxBtCnt := (Pwidth-1);
elsif Rx_State = RxIdle then RxBtCnt := 0;
elsif Rx_State = RxShift and RxBitWdth = 6 and RxBtCnt /= 0 
then RxBtCnt := RxBtCnt - 1;
else RxBtCnt := RxBtCnt;
end if;

-- Shift register
if Rx_State = RxShift and RxBitWdth = 6  
then data <= (data(Pwidth-2 downto 0) & Rx_NRZ);
else data <= data;
end if;

-- Parity bit toggles for each zero bit 
if  (Rx_State = RxShift and RxBitWdth = 6 and Rx_NRZ = '0')
 or (RxParity = '1' and Rx_State = RxStrt)
then RxParity <= not RxParity;
else RxParity <= RxParity;
end if;

-- If transmitted parity doesn't match the running parity, parity error 
if (Rx_Out.Parity_Err = '1' and Rx_In.Clr_Err = '1')
or (Rx_Out.Parity_Err = '0' and (Rx_NRZ = '1' xor RxParity = '0') and Rx_State = ParityRx
                 and (RxDl(1) = '1' xor RxDl(0) = '1') and RxBitWdth = "0110")
then Rx_Out.Parity_Err <= not Rx_Out.Parity_Err;
else Rx_Out.Parity_Err <= Rx_Out.Parity_Err;
end if;

-- Hold Rx done high for one sysclck period.
if Rx_State = ParityRx and RxBitWdth = 6 then Rx_Done_Req <= '1';
elsif Rx_Out.Done = '1' then Rx_Done_Req <= '0';
else Rx_Done_Req <= Rx_Done_Req;
end if;

end if; -- rising edge

end process FM_Decode;

-- SendRxDone for one sysclk period
Send_Rx_Done : process(SysClk, reset)
begin
if reset = '1' then Rx_Out.Done <= '0'; 
 elsif rising_edge(Sysclk) 
	then Rx_Out.Done <= Rx_Done_Req;
end if; -- reset
end process Send_Rx_Done;
end behavioural; -- of Serial_Rx

---------------------- TClk encoder section ----------------------------
-- Transmits a TClk serial stream at 1/10 the clock rate. 
-- A 100MHz clock encodes 10MHz FM serial data

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
USE work.Project_Defs.all;

entity TClk_Tx is
		 port(clock,reset : in std_logic;
				TCTx_In : in TClkTxInRec;
				TCTx_Out : buffer TClkTxOutRec);
end TClk_Tx;

architecture behavioural of TClk_Tx is

-- Serializer state machine
Type TClkTx_State is (TxIdle,TxStrt,ShftTx,ParityTx);
signal TCTx_State : TClkTx_State;

-- Shift register, bit width counter
signal TxShft : std_logic_vector (7 downto 0);
-- Frame bit counter, bit width timer
signal TCTxBtCnt : std_logic_vector (2 downto 0);
signal TCTxWidth : std_logic_vector (3 downto 0);
-- Transmitted TClk data, running parity bit
signal Parity,Tx_Req : std_logic;
signal EnDL : std_logic_vector (1 downto 0);

begin

TClk_Encode : process(clock, reset)

begin

 if reset = '1' then 

	TCTx_State <= TxIdle; TCTx_Out.FM <= '0';
	TCTx_Out.Done <= '0'; Parity <= '0';
	TxShft <= X"00"; Tx_Req <= '0'; EnDL <= "00";
	TCTxWidth <= "0000"; TCTxBtCnt <= "000";

elsif rising_edge(clock) then

	EnDL(0) <= TCTx_In.En;
	EnDL(1) <= EnDL(0);

	if Tx_Req = '0' and EnDL = 1 and TCTx_State = TxIdle then Tx_Req <= '1';
	elsif  Tx_Req = '1' and TCTx_State = TxStrt then Tx_Req <= '0';
	else Tx_Req <= Tx_Req;
	end if;

if TCTxWidth < 9 then TCTxWidth <= TCTxWidth + 1;
else TCTxWidth <= "0000";
end if;

-- TxIdle,TxStrt,ShftTx,ParityTx
Case TCTx_State is
-- Send data on uC write
        When TxIdle => 
	 	 if Tx_Req = '1' and TCTxWidth = 9
		  then TCTx_State <= TxStrt;
			else TCTx_State <= TxIdle;
			end if;
 		When TxStrt =>
		 if TCTxWidth = 9 then TCTx_State <= ShftTx;
		  else TCTx_State <= TxStrt;
		 end if;
          When ShftTx =>
         if TCTxWidth = 9 and TCTxBtCnt = 0 then TCTx_State <= ParityTx;
         else TCTx_State <= ShftTx;
         end if;
           When ParityTx =>
         if TCTxWidth = 9 then TCTx_State <= TxIdle;
         else TCTx_State <= ParityTx;
         end if;
end case;

-- default state is a string of 1's
-- Two transitions per bit period is a 1, one transition a 0
if TCTxWidth = 9 or (TCTxWidth = 4 
					and (TCTx_State = TxIdle or (TxShft(7) = '1' and TCTx_State = ShftTx) 
					 or (Parity = '1' and TCTx_State = ParityTx)))
	then TCTx_Out.FM <= not TCTx_Out.FM;
else TCTx_Out.FM <= TCTx_Out.FM;
end if;
 
-- data frames are 8 bits long 
if TCTx_State = TxStrt and TCTxWidth = 9
  then TCTxBtCnt <= "111";
elsif TCTx_State = TxIdle then TCTxBtCnt <= "000";
elsif TCTx_State = ShftTx and TCTxWidth = 9 and TCTxBtCnt > 0 
	then TCTxBtCnt <= TCTxBtCnt - 1;
else TCTxBtCnt <= TCTxBtCnt;
end if;

-- Load shift register with data byte at the beginning of the transmit sequence
-- load condition
if TCTx_State = TxIdle and Tx_Req = '1' and TCTxWidth = 9
  then TxShft <= TCTx_In.Data;
-- Shift one bit left (MSB first) during data portion of frame
-- shift condition
elsif TCTx_State = ShftTx and TCTxWidth = 9 
	then TxShft <= (TxShft(6 downto 0) & '0');
else TxShft <= TxShft;
end if;

if (Parity = '1' and TCTx_State = TxIdle) -- reset parity at start
  or (TCTx_State = ShftTx and TCTxWidth = 3 and TxShft(7) = '1')
-- Toggle parity bit with each shifted out "0"
then Parity <= not  Parity;
else Parity <= Parity;
end if;

-- Indicate when a frame has been shifted out
if TCTxWidth = 9 and TCTx_State = ParityTx then TCTx_Out.Done <= '1';
else TCTx_Out.Done <= '0';
end if;

end if; -- reset

end process TClk_Encode;

end behavioural; -- of TClk_Tx

------------------------------ TClk decoder section ------------------------------
-- Receives a TClk serial stream at 1/10 the clock rate. 
-- A 100 (or 106.2) MHz clock decodes 10MHz FM serial data

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
USE work.Project_Defs.all;

entity TClk_Rx is
	port( clock,reset : in std_logic;
			TCRx_In : in TClkRxInRec;
	      TCRx_Out : buffer TClkRxOutRec);
end TClk_Rx;

architecture behavioural of Tclk_Rx is

Type TclkFMRx is (RxIdle,RxStrt,RxShift,ParityRx);
Signal Rx_State : TClkFMRx;
-- Registers for FM decoder
-- Shift register, bit width counter
signal TCRxWidth,TCRxBtCnt : std_logic_vector (3 downto 0);
-- Edge detector for incoming FM data
signal RxDl : std_logic_vector (1 downto 0);
-- Transmitted FM data, running parity bit
signal RxParity,Rx_NRZ : std_logic;

begin

Tclk_Decode : process(clock, reset)

begin

 if reset = '1' then 

	Rx_State <= RxIdle; RxDl <= "00"; 
	TCRx_Out.Done <= '0'; RxParity <= '0'; TCRx_Out.Parity_Err <= '0';
	TCRx_Out.data <= (others => '0'); TCRxBtCnt <= "0000"; 
	Rx_NRZ <= '0'; TCRxWidth <= "0000";

elsif rising_edge(clock) then

-- Synchronous edge detector of FM input transitions
RxDl(0) <= TCRx_In.FM;
RxDl(1) <= RxDl(0);

-- Reset sampling counter with every Clock transition while decoder is in Idle,
-- otherwise reset only once per bit period
if (RxDl(1) = '1' xor RxDl(0) = '1') and (TCRxWidth > 6 or Rx_State = RxIdle)
  then TCRxWidth <= "0000";
elsif TCRxWidth /= 15 and 
	not((RxDl(1) = '1' xor RxDl(0) = '1') and (TCRxWidth > 6 or Rx_State = RxIdle))
  then TCRxWidth <= TCRxWidth + 1;
else TCRxWidth <= TCRxWidth;
end if;

-- TClk decoder state machine (RxIdle,RxStrt,RxShift,ParityRx)
Case Rx_State is
        When RxIdle =>
         if TCRxWidth > 6 then Rx_State <= RxStrt;
         else Rx_State <= RxIdle;
         end if;
         When RxStrt =>
         if RxDl(1) = '1' xor RxDl(0) = '1' then Rx_State <= RxShift;
         else Rx_State <= RxStrt;
         end if;
        When RxShift =>
         if TCRxBtCnt = 7 and (RxDl(1) = '1' xor RxDl(0) = '1') and TCRxWidth > 6
                  then Rx_State <= ParityRx;
         else Rx_State <= RxShift;
         end if;
        When ParityRx =>
         if (RxDl(1) = '1' xor RxDl(0) = '1') and TCRxWidth > 6 then Rx_State <= RxIdle;
         else Rx_State <= ParityRx;
         end if;
end case;

-- Serial TData is 1 if transition is in the middle of the bit period,
-- 0 if it is at the end
if  Rx_NRZ = '1' and (RxDl(1) = '1' xor RxDl(0) = '1') and TCRxWidth > 6 
   then Rx_NRZ <= '0';
elsif Rx_NRZ = '0' and (RxDl(1) = '1' xor RxDl(0) = '1') and TCRxWidth <= 6
   then Rx_NRZ <= '1';
else Rx_NRZ <= Rx_NRZ;
end if;

-- TClk events are 8 bits long 
if Rx_State = RxShift and (RxDl(1) = '1' xor RxDl(0) = '1') and TCRxWidth > 6 
then TCRxBtCnt <= TCRxBtCnt + 1;
elsif Rx_State = RxStrt then TCRxBtCnt <= "0000";
else TCRxBtCnt <= TCRxBtCnt;
end if;

-- Data input shift register
if Rx_State = RxShift and TCRxWidth = 7 then TCRx_Out.data <= (TCRx_Out.data(6 downto 0) & Rx_NRZ);
else TCRx_Out.data <= TCRx_Out.data;
end if;

-- Parity bit. Toggle if serial data is a "1"
 if RxParity = '1' and Rx_State = RxStrt
  then RxParity <= '0';
 elsif Rx_State = RxShift and TCRxWidth = 7 and Rx_NRZ = '1'
  then  RxParity <= not  RxParity;
 else  RxParity <= RxParity;
 end if;

-- If transmitted parity doesn't match the running parity, parity error 
if (TCRx_Out.Parity_Err = '1' and TCRx_In.Clr_Err = '1')
or (TCRx_Out.Parity_Err = '0' and (Rx_NRZ = '1' xor RxParity = '1') and Rx_State = ParityRx
                 and (RxDl(1) = '1' xor RxDl(0) = '1') and TCRxWidth > 6)
then TCRx_Out.Parity_Err <= not TCRx_Out.Parity_Err;
else TCRx_Out.Parity_Err <= TCRx_Out.Parity_Err;
end if;

-- Hold Rx done high for one clock period.
if Rx_State = ParityRx and TCRxWidth = 6 then TCRx_Out.Done <= '1';
else TCRx_Out.Done <= '0';
end if;

end if; -- reset

end process TClk_Decode;

end behavioural; -- of TClk_Rx

------------------------------ Beam Sync decoder section ------------------------------
-- Receives a TClk serial stream at 1/14 the clock rate. 
-- A 106 MHz clock decodes 7.59 MHz FM serial data

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
USE work.Project_Defs.all;

entity BS_Rx is
	port( clock,reset : in std_logic;
			Rx_In : in TClkRxInRec;
	      Rx_Out : buffer TClkRxOutRec);
end BS_Rx;

architecture behavioural of BS_Rx is

Type FMRx is (RxIdle,RxStrt,RxShift,ParityRx);
Signal Rx_State : FMRx;
-- Registers for FM decoder
-- Shift register, bit width counter
signal RxWidth,RxBtCnt : std_logic_vector (3 downto 0);
-- Edge detector for incoming FM data
signal RxDl : std_logic_vector (1 downto 0);
-- Transmitted FM data, running parity bit
signal RxParity,Rx_NRZ : std_logic;

begin

BS_Decode : process(clock, reset)

begin

 if reset = '1' then 

	Rx_State <= RxIdle; RxDl <= "00"; 
	Rx_Out.Done <= '0'; RxParity <= '0'; Rx_Out.Parity_Err <= '0';
	Rx_Out.data <= (others => '0'); RxBtCnt <= "0000"; 
	Rx_NRZ <= '0'; RxWidth <= "0000";

elsif rising_edge(clock) then

-- Synchronous edge detector of FM input transitions
RxDl(0) <= Rx_In.FM;
RxDl(1) <= RxDl(0);

-- Reset sampling counter with every Clock transition while decoder is in Idle,
-- otherwise reset only once per bit period
if (RxDl(1) = '1' xor RxDl(0) = '1') and (RxWidth > 8 or Rx_State = RxIdle)
  then RxWidth <= "0000";
elsif RxWidth /= 15 and 
	not((RxDl(1) = '1' xor RxDl(0) = '1') and (RxWidth > 8 or Rx_State = RxIdle))
  then RxWidth <= RxWidth + 1;
else RxWidth <= RxWidth;
end if;

-- TClk decoder state machine (RxIdle,RxStrt,RxShift,ParityRx)
Case Rx_State is
        When RxIdle =>
         if RxWidth > 8 then Rx_State <= RxStrt;
         else Rx_State <= RxIdle;
         end if;
         When RxStrt =>
         if RxDl(1) = '1' xor RxDl(0) = '1' then Rx_State <= RxShift;
         else Rx_State <= RxStrt;
         end if;
        When RxShift =>
         if RxBtCnt = 7 and (RxDl(1) = '1' xor RxDl(0) = '1') and RxWidth > 6
                  then Rx_State <= ParityRx;
         else Rx_State <= RxShift;
         end if;
        When ParityRx =>
         if (RxDl(1) = '1' xor RxDl(0) = '1') and RxWidth > 8 then Rx_State <= RxIdle;
         else Rx_State <= ParityRx;
         end if;
end case;

-- Serial TData is 1 if transition is in the middle of the bit period,
-- 0 if it is at the end
if  Rx_NRZ = '1' and (RxDl(1) = '1' xor RxDl(0) = '1') and RxWidth > 8 
   then Rx_NRZ <= '0';
elsif Rx_NRZ = '0' and (RxDl(1) = '1' xor RxDl(0) = '1') and RxWidth <= 8
   then Rx_NRZ <= '1';
else Rx_NRZ <= Rx_NRZ;
end if;

-- BS events are 8 bits long 
if Rx_State = RxShift and (RxDl(1) = '1' xor RxDl(0) = '1') and RxWidth > 8 
then RxBtCnt <= RxBtCnt + 1;
elsif Rx_State = RxStrt then RxBtCnt <= "0000";
else RxBtCnt <= RxBtCnt;
end if;

-- Data input shift register
if Rx_State = RxShift and RxWidth = 7 then Rx_Out.data <= (Rx_Out.data(6 downto 0) & Rx_NRZ);
else Rx_Out.data <= Rx_Out.data;
end if;

-- Parity bit. Toggle if serial data is a "1"
 if RxParity = '1' and Rx_State = RxStrt
  then RxParity <= '0';
 elsif Rx_State = RxShift and RxWidth = 9 and Rx_NRZ = '1'
  then  RxParity <= not  RxParity;
 else  RxParity <= RxParity;
 end if;

-- If transmitted parity doesn't match the running parity, parity error 
if (Rx_Out.Parity_Err = '1' and Rx_In.Clr_Err = '1')
or (Rx_Out.Parity_Err = '0' and (Rx_NRZ = '1' xor RxParity = '1') and Rx_State = ParityRx
                 and (RxDl(1) = '1' xor RxDl(0) = '1') and RxWidth > 8)
then Rx_Out.Parity_Err <= not Rx_Out.Parity_Err;
else Rx_Out.Parity_Err <= Rx_Out.Parity_Err;
end if;

-- Hold Rx done high for one clock period.
if Rx_State = ParityRx and RxWidth = 6 then Rx_Out.Done <= '1';
else Rx_Out.Done <= '0';
end if;

end if; -- reset

end process BS_Decode;

end behavioural; -- of BS_Rx


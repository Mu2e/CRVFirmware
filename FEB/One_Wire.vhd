LIBRARY ieee;
use ieee.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.Proj_Defs.all;

entity One_Wire is
	 port(clock,reset : in std_logic;
			GA,WRDL : in unsigned(1 downto 0);
			uCA : in unsigned(11 downto 0);
			uCD : in std_logic_vector(15 downto 0);
			Counter1us : in unsigned(7 downto 0);
			Temp : in  std_logic_vector(3 downto 0);
			TempCtrl : buffer std_logic_vector(3 downto 0);
			TempEn : buffer std_logic;
			One_Wire_Out : buffer std_logic_vector(15 downto 0));
end One_Wire;

architecture behavioural of One_Wire is

-- Signals used by 1 the wire interface
Type OneWireSeqType is (Idle,SendReset,ResetGap,WaitPresence,
						SendWrite,WriteGap,SendRead,WaitRead,ReadGap);
signal OneWireSeq : OneWireSeqType;

signal OneWWrtByte : std_logic_vector(7 downto 0);
signal OneWTimer : unsigned(8 downto 0);
signal OneWBitCount : unsigned(7 downto 0);
signal OneWireCmdReg : std_logic_vector(7 downto 0);
signal OneWStat,OneWRstReq,OneWWrtReq,OneWRdReq,ResponseBit,
		 TempStat,ROMStat: std_logic;
-- signal TempCtrl : std_logic_vector(3 downto 0);
signal TempDat : std_logic_vector(71 downto 0);

-- Sequencer for running a command list to retrieve tempereature data
Type OneWrRdTmpType is (Idle,Reset1Req,ChkReset1,ChkBsy1,LdSkpROMCmd1,
								SndSkROM1,ChkSndWrt1,ChkBsy2,LDTempCvtCmd,SndTmpCvt,
								ChkSndWrt2,ChkBsy3,Reset2Req,ChkReset2,ChkBsy4,
								LdSkpROMCmd2,SndSkROM2,ChkSndWrt3,ChkBsy5,
								LdRdScrtchPdCmd,SndRdScrtchPdCmd,ChkSndWrt4,
								ChkBsy6,RdScrtchPd,ChkRd,ChkBsy7);
signal OneWrRdTmp : OneWrRdTmpType;

-- Sequencer for running a command list to retrieve ID data
Type OneWrRdROMType is (Idle,ResetReq,ChkReset,ChkBsy1,LdRdROMCmd,
								SndRdROM,ChkSndWrt,ChkBsy2,RdROM,ChkRd,ChkBsy3);
signal OneWrRdROM : OneWrRdROMType;

begin

OneWire : process(clock, reset)

begin

 if reset = '1' then 

	OneWWrtByte <= X"00"; OneWTimer <= (others => '0'); OneWStat <= '0';
	OneWBitCount <= (others => '0'); TempCtrl <= (others => '0'); 
	OneWWrtReq <= '0'; OneWRdReq <= '0'; ResponseBit <= '0'; 
	TempDat <= (others => '0'); OneWireCmdReg <= X"00"; 
	OneWRstReq <= '0'; OneWrRdROM <= Idle;  OneWrRdTmp <= Idle;
	OneWireSeq <= Idle; TempStat <= '0';  ROMStat <= '0'; 

elsif rising_edge(clock) then

------------------ One wire temperature sensor logic ------------------

-- Idle,SendReset,ResetGap,WaitPresence,SendWrite,WriteGap,SendRead,ReadGap);
Case OneWireSeq is
	When Idle =>
		if OneWRstReq = '1' then OneWireSeq <= SendReset;
		elsif OneWRstReq = '0' and OneWWrtReq = '1' then OneWireSeq <= SendWrite; 
		elsif OneWRstReq = '0' and OneWRdReq = '1' then OneWireSeq <= SendRead; 
		else OneWireSeq <= Idle;
		end if;
	When SendReset => 
		if OneWTimer = 0 then OneWireSeq <= ResetGap;
		else OneWireSeq <= SendReset;
		end if;
	When ResetGap => 
		if OneWTimer = 0 then OneWireSeq <= WaitPresence;
		else OneWireSeq <= ResetGap;
		end if;
	When WaitPresence =>
		if OneWTimer = 0 then 
			   if OneWWrtReq = '1' then OneWireSeq <= SendWrite;
			elsif OneWRdReq = '1'  then OneWireSeq <= SendRead;
			else OneWireSeq <= Idle;
			end if;
		 else OneWireSeq <= WaitPresence;
		 end if;
	When SendWrite =>
		if OneWTimer = 1 then OneWireSeq <= WriteGap;
		else OneWireSeq <= SendWrite;
		end if;
	When WriteGap =>
		if    OneWTimer = 0 and OneWBitCount = 0 then OneWireSeq <= Idle;
		elsif OneWTimer = 0 and OneWBitCount /= 0 then OneWireSeq <= SendWrite;
		else OneWireSeq <= WriteGap;
		end if;
	When SendRead =>
		if OneWTimer = 1 then OneWireSeq <= ReadGap;
		else OneWireSeq <= SendRead;
		end if;
	When ReadGap =>
			if OneWTimer = 0 and OneWBitCount = 0 then OneWireSeq <= Idle;
			elsif OneWTimer = 0 and OneWBitCount /= 0 then OneWireSeq <= SendRead;
			else OneWireSeq <= ReadGap;
			end if;
	When others => OneWireSeq <= Idle;
end case;

-- Use this to indicate 1 wire transaction is busy
if OneWireSeq /= Idle or OneWTimer /= 0 then OneWStat <= '1';
else OneWStat <= '0'; 
end if;

-- Laatch the response bit from the appropriate CMB
 if OneWireSeq = WaitPresence and OneWTimer = 0 
	then 
		Case TempCtrl is 
		 When X"1" => ResponseBit <= Temp(0); 
		 When X"2" => ResponseBit <= Temp(1); 
		 When X"4" => ResponseBit <= Temp(2); 
		 When X"8" => ResponseBit <= Temp(3); 
		 When others => ResponseBit <= ResponseBit;
	 end case;
 else ResponseBit <= ResponseBit;
end if;

-- Open collector enable
if OneWireSeq = SendReset 
or (OneWireSeq = SendWrite and OneWWrtByte(0) = '0') 
or	(OneWireSeq = SendWrite and OneWWrtByte(0) = '1' and OneWTimer > 58) 
or (OneWireSeq = SendRead and OneWTimer > 58)
then TempEn <= '1';
else TempEn <= '0';
end if;

-- Shift in LSB first when doing a read
if OneWireSeq = SendRead and OneWTimer = 45 and Counter1us = Count1us
	then
		Case TempCtrl is 
		 When X"1" => TempDat <= Temp(0) & TempDat(71 downto 1); 
		 When X"2" => TempDat <= Temp(1) & TempDat(71 downto 1); 
		 When X"4" => TempDat <= Temp(2) & TempDat(71 downto 1); 
		 When X"8" => TempDat <= Temp(3) & TempDat(71 downto 1); 
		 When others => TempDat <= TempDat;
	 end case;
else TempDat <= TempDat;
end if; 

-- Shift out LSB first when sending data
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = OneWireCmdAd 
then OneWWrtByte <= uCD(7 downto 0);
elsif OneWrRdTmp = LdSkpROMCmd1 or OneWrRdTmp = LdSkpROMCmd2
then OneWWrtByte <= X"CC";
elsif OneWrRdTmp = LDTempCvtCmd then OneWWrtByte <= X"44";
elsif OneWrRdTmp = LdRdScrtchPdCmd then OneWWrtByte <= X"BE"; 
elsif OneWrRdROM = LdRdROMCmd then OneWWrtByte <= X"33";  
elsif OneWireSeq = SendWrite and OneWTimer = 1
then OneWWrtByte <= '0' & OneWWrtByte(7 downto 1);
else OneWWrtByte <= OneWWrtByte;
end if;

-- Command register
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = OneWireCmdAd
	 then OneWireCmdReg <= uCD(7 downto 0);
else OneWireCmdReg <= OneWireCmdReg;
end if;

-- Control and status register for the one wire temperature sensor
-- Bits 0..3 select which CMBs sensors to be read out
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TempCtrlAd 
then TempCtrl <= uCD(3 downto 0);
else TempCtrl <= TempCtrl;
end if;

-- Bit 4 indicates write - exclude simultaneous read and write
if (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TempCtrlAd 
	and uCD(4) = '1' and uCD(5) = '0')
 or OneWrRdTmp = LdSkpROMCmd1 or OneWrRdTmp = LdSkpROMCmd2 or OneWrRdTmp = LDTempCvtCmd
 or OneWrRdTmp = LdRdScrtchPdCmd or OneWrRdROM = LdRdROMCmd
	then OneWWrtReq <= '1';
elsif OneWireSeq = WriteGap and OneWTimer = 0 and OneWBitCount = 0 
	then OneWWrtReq <= '0';
else OneWWrtReq <= OneWWrtReq;
end if;

-- Bit 5 indicates read
if (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TempCtrlAd 
	and uCD(4) = '0' and uCD(5) = '1')
	or OneWrRdTmp = RdScrtchPd or OneWrRdROM = RdROM
then OneWRdReq <= '1';
elsif OneWireSeq = ReadGap and OneWTimer = 0 and OneWBitCount = 0 
then OneWRdReq <= '0';
else OneWRdReq <= OneWRdReq;
end if;

-- Bit 6 indicates a reset
if (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TempCtrlAd and uCD(6) = '1')
or OneWrRdTmp = Reset1Req or OneWrRdTmp = Reset2Req or OneWrRdROM = ResetReq
then OneWRstReq <= '1';
elsif OneWireSeq = SendReset
then OneWRstReq <= '0';
end if;

-- Serial bit counter
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TempCtrlAd 
then OneWBitCount <= unsigned(uCD(15 downto 8));
elsif OneWrRdTmp = LdSkpROMCmd1 or OneWrRdTmp = LdSkpROMCmd2
   or OneWrRdTmp = LDTempCvtCmd or OneWrRdTmp = LdRdScrtchPdCmd
	or OneWrRdROM = LdRdROMCmd
then OneWBitCount <= X"07";
elsif OneWrRdTmp = RdScrtchPd or OneWrRdROM = RdROM
then OneWBitCount <= X"47";
elsif (OneWireSeq = WriteGap or OneWireSeq = ReadGap) and OneWTimer = 0 
then OneWBitCount <= OneWBitCount - 1;
end if;

-- Timer. Different operations have different timing requirements
   if OneWWrtReq = '1' and OneWRstReq = '0' and OneWireSeq = Idle then OneWTimer <= '0' & X"3D"; 
elsif OneWRdReq = '1' and OneWRstReq = '0' and OneWireSeq = Idle then OneWTimer <= '0' & X"3D"; 
elsif OneWRstReq = '1' and OneWireSeq = Idle then OneWTimer <= '1' & X"E0";
elsif OneWireSeq = SendReset    and OneWTimer = 0 then OneWTimer <= '0' & X"3D"; 
elsif OneWireSeq = ResetGap     and OneWTimer = 0 then OneWTimer <= '1' & X"E0";
elsif OneWireSeq = WaitPresence and OneWTimer = 0 then OneWTimer <= '0' & X"3D"; 
elsif OneWireSeq = WriteGap     and OneWTimer = 0 then OneWTimer <= '0' & X"3D"; 
elsif OneWireSeq = ReadGap      and OneWTimer = 0 then OneWTimer <= '0' & X"3D";
elsif Counter1us = Count1us and OneWTimer /= 0 then OneWTimer <= OneWTimer - 1;
else OneWTimer <= OneWTimer;
end if;

-- Sequencers to do a complete 1 wire transaction within the FPGA

-- Temperature read sequencer
-- (Idle,Reset1Req,ChkBsy1,LdSkpROMCmd1,SndSkROM1,ChkBsy2,
--	LDTempCvtCmd,SndTmpCvt,ChkBsy3,Reset2Req,ChkBsy4,
--	LdSkpROMCmd2,SndSkROM2,ChkBsy5,LdRdScrtchPdCmd,
--	SndRdScrtchPdCmd,ChkBsy6,RdScrtchPd,ChkBsy7);
Case OneWrRdTmp is
	When Idle =>
		if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = OneWireCmdAd
		 and uCD(8) = '1' and uCD(9) = '0'
		then OneWrRdTmp <= Reset1Req;
		else OneWrRdTmp <= Idle;
	end if;
	When Reset1Req => OneWrRdTmp <= ChkReset1;
	When ChkReset1 =>
			if OneWireSeq = SendReset then OneWrRdTmp <= ChkBsy1;
		  else OneWrRdTmp <= ChkReset1;
		  end if;
	When ChkBsy1 =>
		if OneWStat = '0' then OneWrRdTmp <= LdSkpROMCmd1;
		 else OneWrRdTmp <= ChkBsy1;
		end if;
	When LdSkpROMCmd1 => OneWrRdTmp <= SndSkROM1;
	When SndSkROM1 => OneWrRdTmp <= ChkSndWrt1;
	When ChkSndWrt1 =>
		  if OneWireSeq = SendWrite then OneWrRdTmp <= ChkBsy2;
		  else OneWrRdTmp <= ChkSndWrt1;
		  end if;
	When ChkBsy2 =>
		if OneWStat = '0' then OneWrRdTmp <= LDTempCvtCmd;
      else OneWrRdTmp <= ChkBsy2;
		end if;
	When LDTempCvtCmd => OneWrRdTmp <= SndTmpCvt;
	When SndTmpCvt => OneWrRdTmp <= ChkSndWrt2;
	When ChkSndWrt2 =>
		  if OneWireSeq = SendWrite then OneWrRdTmp <= ChkBsy3;
		  else OneWrRdTmp <= ChkSndWrt2;
		  end if;
	When ChkBsy3 =>
		if OneWStat = '0' then OneWrRdTmp <= Reset2Req;
		else OneWrRdTmp <= ChkBsy3;
		end if;
	When Reset2Req => OneWrRdTmp <= ChkReset2;
	When ChkReset2 =>
			if OneWireSeq = SendReset then OneWrRdTmp <= ChkBsy4;
		  else OneWrRdTmp <= ChkReset2;
		  end if;
	When ChkBsy4 =>
			if OneWStat = '0' then OneWrRdTmp <= LdSkpROMCmd2;
			else OneWrRdTmp <= ChkBsy4;
			end if;
	When LdSkpROMCmd2 => OneWrRdTmp <= SndSkROM2;
	When SndSkROM2 => OneWrRdTmp <= ChkSndWrt3;
	When ChkSndWrt3 =>
		  if OneWireSeq = SendWrite then OneWrRdTmp <= ChkBsy5;
		  else OneWrRdTmp <= ChkSndWrt3;
		  end if;
	When ChkBsy5 =>
			if OneWStat = '0' then OneWrRdTmp <= LdRdScrtchPdCmd;
			else OneWrRdTmp <= ChkBsy5;
			end if; 
	When LdRdScrtchPdCmd => OneWrRdTmp <= SndRdScrtchPdCmd;
	When SndRdScrtchPdCmd => OneWrRdTmp <= ChkSndWrt4;
	When ChkSndWrt4 =>
		  if OneWireSeq = SendWrite then OneWrRdTmp <= ChkBsy6;
		  else OneWrRdTmp <= ChkSndWrt4;
		  end if;
	When ChkBsy6 =>
			if OneWStat = '0' then OneWrRdTmp <= RdScrtchPd;
			else OneWrRdTmp <= ChkBsy6;
			end if;
	When RdScrtchPd => OneWrRdTmp <= ChkRd;
	When ChkRd =>
			  if OneWireSeq = SendRead then OneWrRdTmp <= ChkBsy7;
		  else OneWrRdTmp <= ChkRd;
		  end if;
	When ChkBsy7 =>
			if OneWStat = '0' then OneWrRdTmp <= Idle;
			else OneWrRdTmp <= ChkBsy7;
			end if;
	When others => OneWrRdTmp <= Idle; 

end Case;

-- Temperature read sequencer status bit
 if OneWrRdTmp = Idle then TempStat <= '1';
 else TempStat <= '0';
 end if;

-- ROM read sequencer
-- (Idle,ResetReq,ChkReset,ChkBsy1,LdRdROMCmd,
--	SndRdROM,ChkSndWrt,ChkBsy2,RdROM,ChkRd,ChkBsy3);
Case OneWrRdROM is
	When Idle =>
		if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = OneWireCmdAd
		 and uCD(8) = '0' and uCD(9) = '1'
		then OneWrRdROM <= ResetReq;
		else OneWrRdROM <= Idle;
		end if;
	When ResetReq => OneWrRdROM <= ChkReset;
	When ChkReset => 
				if OneWireSeq = SendReset then OneWrRdROM <= ChkBsy1;
				else OneWrRdROM <= ChkReset;
				end if;
	When ChkBsy1 =>
			if OneWStat = '0' then OneWrRdROM <= LdRdROMCmd;
			else OneWrRdROM <= ChkBsy1;
			end if;
	When LdRdROMCmd => OneWrRdROM <= SndRdROM;
	When SndRdROM => OneWrRdROM <= ChkSndWrt;
	When ChkSndWrt =>  
				if OneWireSeq = SendWrite then OneWrRdROM <= ChkBsy2;
				else OneWrRdROM <= ChkSndWrt;
				end if;
	When ChkBsy2 => 
			if OneWStat = '0' then OneWrRdROM <= RdROM;
			else OneWrRdROM <= ChkBsy2;
			end if;
	When RdROM => OneWrRdROM <= ChkRd;
	When ChkRd =>
			if OneWireSeq = SendRead then OneWrRdROM <= ChkBsy3;
			else OneWrRdROM <= ChkRd;
			end if;
	When ChkBsy3 =>
			if OneWStat = '0' then OneWrRdROM <= Idle;
			else OneWrRdROM <= ChkBsy3;
			end if;
	When others => OneWrRdROM <= Idle; 
end Case;

end if;

-- ROM read sequencer status bit
if OneWrRdROM = Idle then ROMStat <= '1';
else ROMStat <= '0';
end if;

end process OneWire;

with uCA(9 downto 0) select

One_Wire_Out <= X"0" & "00" & ROMStat & TempStat & OneWireCmdReg when OneWireCmdAd,
				X"00" & OneWStat & "000" & TempCtrl when TempCtrlAd,
				X"00" & TempDat(71 downto 64) when TempDat0Ad,
				TempDat(63 downto 48) when TempDat1Ad,
				TempDat(47 downto 32) when TempDat2Ad,
				TempDat(31 downto 16) when TempDat3Ad,
				TempDat(15 downto 0) when TempDat4Ad,
				X"0000" when others;

end behavioural;
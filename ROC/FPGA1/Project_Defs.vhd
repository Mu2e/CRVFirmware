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

constant MarkerCntAddr : AddrPtr := "00" & X"41"; -- recieved markers
constant HeartBeatCntAddr : AddrPtr := "00" & X"42"; -- sent out heartbeats, recieved heartbeat packages
constant LastWindowLengthAddr : AddrPtr := "00" & X"43";
constant InjectionLengthAddr : AddrPtr := "00" & X"44";
constant Clk80MHzAdd : AddrPtr := "00" & X"45";
constant MarkerCnt2Addr : AddrPtr := "00" & X"46";
constant LoopbackAdd : AddrPtr := "00" & X"47";
constant MarkerCnt3Addr : AddrPtr := "00" & X"48";
constant DutyAdd : AddrPtr := "00" & X"49";
constant LoopbackMarkerCntAdd : AddrPtr := "00" & X"4A";
constant debugBuffAdd : AddrPtr := "00" & X"4B";
constant debugTrigAdd : AddrPtr := "00" & X"4C";
constant debugTrigMaskAdd : AddrPtr := "00" & X"4D";
constant debugTrigPatternAdd : AddrPtr := "00" & X"4E";
constant HeartBeatCnAddr  : AddrPtr := "00" & X"4F";



constant DCSPktBuffAd    : AddrPtr := "00" & X"50";
constant DCSPktWdUsedAd  : AddrPtr := "00" & X"51";
constant DCSBuffCntAd    : AddrPtr := "00" & X"52";
constant DCSBuffAd       : AddrPtr := "00" & X"53";
--constant DCSBuffRdAd     : AddrPtr := "00" & X"54";
constant InjectionLengtFirstAddr : AddrPtr := "00" & X"54";
constant DCSHeaderAd     : AddrPtr := "00" & X"55";
constant DCSEvCntAd      : AddrPtr := "00" & X"56";
constant DCSStatusAd     : AddrPtr := "00" & X"57";


constant sendGRAdd           : AddrPtr := "00" & X"58";
constant InjectionCntAdd     : AddrPtr := "00" & X"59";
constant InjectionWindowAdd  : AddrPtr := "00" & X"5A";
constant InjectionDutyAdd    : AddrPtr := "00" & X"5B";
constant EventBuffSpyAd       : AddrPtr := "00" & X"5C";
constant DRTimeoutAdd        : AddrPtr := "00" & X"5D";
constant DCSSpyAd            : AddrPtr := "00" & X"5E"; -- remove me again

constant SpillTrigCntAdHi : AddrPtr := "00" & X"66";
constant SpillTrigCntAdLo : AddrPtr := "00" & X"67";

constant SpillCountAddr : AddrPtr := "00" & X"68";
constant EVWdCntAddr : AddrPtr := "00" & X"69";

constant SpillWdCntHiAd : AddrPtr := "00" & X"6A";
constant SpillWdCntLoAd : AddrPtr := "00" & X"6B";

constant UpTimeRegAddrHi : AddrPtr := "00" & X"6C";
constant UpTimeRegAddrLo : AddrPtr := "00" & X"6D";


constant LastUbSentAddr : AddrPtr:= "00" & X"70";
constant TimeStampAdHi : AddrPtr := "00" & X"72";
constant TimeStampAdLo : AddrPtr := "00" & X"73";

constant SpillStatAddr : AddrPtr := "00" & X"76";

constant MarkerBitsAd : AddrPtr  := "00" & X"77";
constant MarkerDelayAd : AddrPtr := "00" & X"78";

-- event builder debugging
constant FormatRegAddr : AddrPtr := "00" & X"79";
constant uBLowRegAddr : AddrPtr := "00" & X"7A";
constant uBHighRegAddr : AddrPtr := "00" & X"7B";

constant LinkErrAd : AddrPtr           := "00" & X"80";
constant HrtBtOffsetAd : AddrPtr       := "00" & X"81";
constant DReqCountLowAd : AddrPtr      := "00" & X"82";
constant DReqCountHiAd : AddrPtr       := "00" & X"83";
--constant TxRxBufferSettingAd : AddrPtr := "00" & X"84";
constant GTPTxRdAddr : AddrPtr         := "00" & X"85";
constant DReqBuffTraceAd : AddrPtr     := "00" & X"86";
constant LinkFIFOTraceAd : AddrPtr     := "00" & X"87";


constant DRCntAdd : AddrPtr        := "00" & X"94";
constant DebugCntAdd : AddrPtr        := "00" & X"95";

constant GitHashHiAddr : AddrPtr      := "00" & X"96";
constant GitHashLoAddr : AddrPtr      := "00" & X"97";
constant GTPRstCntAd : AddrPtr        := "00" & X"98";
constant DebugVersionAd : AddrPtr     := "00" & X"99";

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
Type Array_3x13 is Array(0 to 2) of std_logic_vector (12 downto 0);
Type Array_3x14 is Array (0 to 2) of std_logic_vector (13 downto 0);
Type Array_3x16 is Array(0 to 2) of std_logic_vector (15 downto 0);

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


end Project_Defs;

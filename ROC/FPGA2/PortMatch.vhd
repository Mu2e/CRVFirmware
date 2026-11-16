-- Port match for the CRV ROC data FPGA (FPGA 2)
--
-- Decides which FEB answers go into the next event that is written to the DDR
-- and sent to FPGA 1. The aim: for every request sent to the FEBs exactly one
-- event part goes to FPGA 1, in the order of the requests.
--
-- List of open requests: the uC loads a request packet into the PHY transmit
-- buffer (first word: low byte X"82", high byte number of requests n, then per
-- request EWT bits 15..0 and EWT bits 31..16). Every request is pushed to a
-- list of four. If a fifth is pushed the oldest is dropped.
--
-- Header registers: the four header words of an answer (word count, EWT high,
-- EWT low, status) are read from the port buffer into registers, the write
-- sequencer in the top level takes them from here and reads only the data
-- words from the buffer.
--
-- Enable = '0': an event is built when all active ports hold a complete
-- answer, as before. Nothing is compared, nothing is emptied. The event
-- empties the list, so that the time in LastTime/MaxTime is the time from the
-- oldest request since the last event.
--
-- Enable = '1', E is the oldest open request. For every active port:
--   header with the EWT of E            the port is ready (when its data is complete)
--   header with the EWT of another      the port missed E: it is missing in this
--     open request                      event and keeps its answer
--   header with an EWT not in the list  the port buffer is emptied (read until
--                                       it stays empty)
--   no header                           wait. With Timeout /= 0 the port is
--                                       missing after Timeout x 10 us
-- The event for E is built when every active port is ready or missing. Then E
-- is removed from the list. If all ports are missing an event without data is
-- built.
--
-- The list is emptied by the reset of the port buffers, while the write to the
-- DDR is off and by the run start reset (GTPRxRst, write of bit 0 to bus
-- address 0x002), which also clears the counters. With no active port nothing
-- is pushed and nothing is built.
--
-- Write strobe: the write strobe of the uC is asynchronous to clk. It is taken
-- through two flip-flops here before its edge is detected, and these must not
-- be duplicated or merged with other flip-flops by the synthesis: two copies
-- of the first one can see the edge in different clocks, and logic that takes
-- the copies in the wrong combination sees two strobes for one write.

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

use work.Proj_Defs.all;

entity PortMatch is
    port (
        clk      : in  std_logic;
        rst_n    : in  std_logic; -- CpldRst
        -- uC bus
        uCWR     : in  std_logic; -- write strobe and chip select of the uC bus, active low,
        CpldCS   : in  std_logic; -- asynchronous
        uCA      : in  std_logic_vector(11 downto 0);
        GA       : in  std_logic_vector(1 downto 0);
        uCD      : in  std_logic_vector(15 downto 0);
        RdData   : out std_logic_vector(15 downto 0);
        Tick10us : in  std_logic;
        -- port buffers
        Active   : in  std_logic_vector(7 downto 0);   -- Rx_active
        WrtEn    : in  std_logic;                      -- DDRWrt_En
        BuffRst  : in  std_logic;                      -- RxBuffRst
        Empty    : in  std_logic_vector(7 downto 0);
        Dout     : in  std_logic_vector(127 downto 0); -- 8 x 16
        RdCnt    : in  std_logic_vector(103 downto 0); -- 8 x 13
        Rd       : out std_logic_vector(7 downto 0);   -- read enable, to be "or"ed with the one of the sequencer
        -- write sequencer
        SeqIdle  : in  std_logic;
        Go       : out std_logic;                      -- build an event now
        En       : out std_logic;
        Incl     : out std_logic_vector(7 downto 0);   -- ports in this event
        Miss     : out std_logic_vector(7 downto 0);   -- active ports that are missing in this event
        EvEwt    : out std_logic_vector(31 downto 0);  -- EWT of this event (Enable = '1')
        HdrWd    : out std_logic_vector(127 downto 0); -- header words of the eight ports
        HdrUbHi  : out std_logic_vector(127 downto 0);
        HdrUbLo  : out std_logic_vector(127 downto 0);
        HdrStat  : out std_logic_vector(127 downto 0)
    );
end entity PortMatch;

architecture rtl of PortMatch is

    type EwtArr  is array (0 to 3) of std_logic_vector(31 downto 0);
    type HdrArr  is array (0 to 7) of std_logic_vector(63 downto 0); -- word count, EWT high, EWT low, status
    type Cnt3Arr is array (0 to 7) of std_logic_vector(2 downto 0);
    type Cnt4Arr is array (0 to 7) of std_logic_vector(3 downto 0);
    type StType  is (S_Dec, S_Go, S_Busy);

    signal St        : StType;
    -- write strobe of the uC: WrS(0) samples the pins, WrS(1) is the clean copy
    signal WrS       : std_logic_vector(2 downto 0);
    signal WrStb     : std_logic;
    attribute register_duplication : string;
    attribute register_duplication of WrS : signal is "no";
    attribute equivalent_register_removal : string;
    attribute equivalent_register_removal of WrS : signal is "no";
    attribute shreg_extract : string;
    attribute shreg_extract of WrS : signal is "no";
    attribute keep : string;
    attribute keep of WrS : signal is "true";
    -- settings
    signal EnR, ClrStat : std_logic;
    signal TimeoutR  : std_logic_vector(15 downto 0);
    -- request packet
    signal First, IsReq : std_logic;
    signal Remain    : std_logic_vector(7 downto 0);
    signal LoLatch, LastW0 : std_logic_vector(15 downto 0);
    -- list, entry 0 is the oldest
    signal L         : EwtArr;
    signal LCnt      : integer range 0 to 4;
    signal TimeE, TimeGo : std_logic_vector(15 downto 0);
    -- ports
    signal Hdr       : HdrArr;
    signal HCnt      : Cnt3Arr;
    signal Drain     : std_logic_vector(7 downto 0);
    signal DrCnt     : Cnt4Arr;
    signal Fetch, Pop : std_logic_vector(7 downto 0);
    signal EqR, InR, CmpR, HVal : std_logic_vector(7 downto 0);
    signal Settle    : std_logic_vector(1 downto 0);
    -- event
    signal InclR, MissR : std_logic_vector(7 downto 0);
    signal EvEwtR    : std_logic_vector(31 downto 0);
    -- status
    signal LastTime, MaxTime : std_logic_vector(15 downto 0);
    signal EvCnt, ReqCnt, DrainCnt, MissEvCnt, TOCnt, ListDropCnt : std_logic_vector(15 downto 0);
    signal LastMiss, DrainedPorts : std_logic_vector(7 downto 0);
    signal BadEwt    : std_logic_vector(31 downto 0);
    signal BadPort   : std_logic_vector(2 downto 0);
    signal StCode    : std_logic_vector(1 downto 0);
    signal LCntV     : std_logic_vector(2 downto 0);

    function inc16(c : std_logic_vector(15 downto 0)) return std_logic_vector is
    begin
        if c = X"FFFF" then return c; else return c + 1; end if;
    end function;

begin

    -- one clock for every uC write, address and data are stable around it
    WrStb <= WrS(1) and not WrS(2);

    Go    <= '1' when St = S_Go else '0';
    En    <= EnR;
    Incl  <= InclR;
    Miss  <= MissR;
    EvEwt <= EvEwtR;

    Gen_Ports : for i in 0 to 7 generate
        HdrWd  (16*i+15 downto 16*i) <= Hdr(i)(63 downto 48);
        HdrUbHi(16*i+15 downto 16*i) <= Hdr(i)(47 downto 32);
        HdrUbLo(16*i+15 downto 16*i) <= Hdr(i)(31 downto 16);
        HdrStat(16*i+15 downto 16*i) <= Hdr(i)(15 downto 0);
        -- Headers are read only while no event is being built
        Fetch(i) <= '1' when St = S_Dec and WrtEn = '1' and BuffRst = '0' and Active(i) = '1'
                             and HCnt(i) /= 4 and Drain(i) = '0' else '0';
        Pop(i)   <= (Fetch(i) or Drain(i)) and not Empty(i);
        HVal(i)  <= '1' when HCnt(i) = 4 else '0';
    end generate;
    Rd <= Pop;

    StCode <= "00" when St = S_Dec else "01" when St = S_Go else "10";
    LCntV  <= "000" when LCnt = 0 else "001" when LCnt = 1 else "010" when LCnt = 2
              else "011" when LCnt = 3 else "100";

    with uCA(9 downto 0) select
    RdData <= HVal & '0' & LCntV & StCode & '0' & EnR when PMCsrAd,
              TimeoutR                       when PMTimeoutAd,
              LastTime                       when PMLastTimeAd,
              MaxTime                        when PMMaxTimeAd,
              EvCnt                          when PMEvCntAd,
              ReqCnt                         when PMReqCntAd,
              DrainCnt                       when PMDrainCntAd,
              MissEvCnt                      when PMMissCntAd,
              TOCnt                          when PMTimeoutCntAd,
              ListDropCnt                    when PMListDropCntAd,
              DrainedPorts & LastMiss        when PMPortsAd,
              BadEwt(31 downto 16)           when PMBadEwtHiAd,
              BadEwt(15 downto 0)            when PMBadEwtLoAd,
              "00000" & BadPort & Drain      when PMBadPortAd,
              L(0)(31 downto 16)             when PMOldestHiAd,
              L(0)(15 downto 0)              when PMOldestLoAd,
              LastW0                         when PMTxWord0Ad,
              X"0000"                        when others;

    process(clk, rst_n)
        variable vL       : EwtArr;
        variable vCnt     : integer range 0 to 4;
        variable vHeadChg : boolean;
        variable vListChg : boolean;
        variable vAllOk   : boolean;
        variable vTO      : boolean;
        variable vRdy, vMissN, vStart : std_logic_vector(7 downto 0);
        variable vWd      : std_logic_vector(15 downto 0);
        variable vEwt     : std_logic_vector(31 downto 0);
        variable vRunRst  : boolean;
    begin
        if rst_n = '0' then
            St <= S_Dec;
            WrS <= "000";
            EnR <= '1'; ClrStat <= '0'; TimeoutR <= X"012C"; -- on, 3 ms
            First <= '1'; IsReq <= '0'; Remain <= (others => '0');
            LoLatch <= (others => '0'); LastW0 <= (others => '0');
            L <= (others => (others => '0')); LCnt <= 0;
            TimeE <= (others => '0'); TimeGo <= (others => '0');
            Hdr <= (others => (others => '0'));
            HCnt <= (others => "000"); Drain <= X"00"; DrCnt <= (others => X"0");
            EqR <= X"00"; InR <= X"00"; CmpR <= X"00"; Settle <= "11";
            InclR <= X"00"; MissR <= X"00"; EvEwtR <= (others => '0');
            LastTime <= (others => '0'); MaxTime <= (others => '0');
            EvCnt <= (others => '0'); ReqCnt <= (others => '0'); DrainCnt <= (others => '0');
            MissEvCnt <= (others => '0'); TOCnt <= (others => '0'); ListDropCnt <= (others => '0');
            LastMiss <= X"00"; DrainedPorts <= X"00";
            BadEwt <= (others => '0'); BadPort <= "000";

        elsif rising_edge(clk) then

            vL := L; vCnt := LCnt; vHeadChg := false; vListChg := false;
            vStart := X"00";

            WrS <= WrS(1 downto 0) & (not uCWR and not CpldCS);

            -- run start: the reset the uC writes to FPGA 1 (GTPRxRst)
            vRunRst := WrStb = '1' and uCA(11 downto 10) = "00" and uCA(9 downto 0) = GTPFIFOAddr
                       and uCD(0) = '1';

            ---------------- settings ----------------
            ClrStat <= '0';
            if vRunRst then ClrStat <= '1'; end if;
            if WrStb = '1' and uCA(11 downto 10) = GA then
                if uCA(9 downto 0) = PMCsrAd then
                    EnR <= uCD(0); ClrStat <= uCD(15);
                end if;
                if uCA(9 downto 0) = PMTimeoutAd then
                    TimeoutR <= uCD;
                end if;
            end if;

            ---------------- requests going to the FEBs ----------------
            if WrStb = '1' and uCD(0) = '1'
               and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = PhyTxCSRAddr)
                    or uCA(9 downto 0) = PhyTxCSRBroadCastAd) then
                -- the packet is sent, the next word written is the first of a new packet
                First <= '1'; IsReq <= '0'; Remain <= (others => '0');
            elsif WrStb = '1'
               and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = PhyTxFIFOWrtAd)
                    or uCA(9 downto 0) = PhyTxBroadCastAd) then
                if First = '1' then
                    First <= '0'; LastW0 <= uCD;
                    if uCD(7 downto 0) = X"82" then
                        IsReq <= '1'; Remain <= uCD(14 downto 8) & '0';
                    else
                        IsReq <= '0'; Remain <= (others => '0');
                    end if;
                elsif IsReq = '1' and Remain /= 0 then
                    Remain <= Remain - 1;
                    if Remain(0) = '0' then
                        LoLatch <= uCD;
                    elsif Active /= 0 then
                        ReqCnt <= inc16(ReqCnt);
                        if vCnt = 4 then
                            vL(0) := vL(1); vL(1) := vL(2); vL(2) := vL(3);
                            vL(3) := uCD & LoLatch;
                            vHeadChg := true;
                            ListDropCnt <= inc16(ListDropCnt);
                        else
                            if vCnt = 0 then vHeadChg := true; end if;
                            vL(vCnt) := uCD & LoLatch;
                            vCnt := vCnt + 1;
                        end if;
                        vListChg := true;
                    else
                        -- no active port: counted, not put in the list
                        ReqCnt <= inc16(ReqCnt);
                    end if;
                end if;
            end if;

            ---------------- header registers ----------------
            for i in 0 to 7 loop
                if BuffRst = '1' or WrtEn = '0' then
                    HCnt(i) <= "000"; Drain(i) <= '0'; DrCnt(i) <= X"0";
                elsif Drain(i) = '1' then
                    -- empty the buffer: read until it has been empty for 15 clocks
                    if Empty(i) = '0' then
                        DrCnt(i) <= X"0";
                    elsif DrCnt(i) = X"F" then
                        Drain(i) <= '0'; HCnt(i) <= "000";
                    else
                        DrCnt(i) <= DrCnt(i) + 1;
                    end if;
                elsif Pop(i) = '1' then
                    case HCnt(i) is
                        when "000"  => Hdr(i)(63 downto 48) <= Dout(16*i+15 downto 16*i);
                        when "001"  => Hdr(i)(47 downto 32) <= Dout(16*i+15 downto 16*i);
                        when "010"  => Hdr(i)(31 downto 16) <= Dout(16*i+15 downto 16*i);
                        when others => Hdr(i)(15 downto 0)  <= Dout(16*i+15 downto 16*i);
                    end case;
                    HCnt(i) <= HCnt(i) + 1;
                end if;

                -- what the header of this port is, one clock later
                vEwt := Hdr(i)(47 downto 16);
                vWd  := Hdr(i)(63 downto 48);
                if HCnt(i) = 4 and LCnt /= 0 and vEwt = L(0) then EqR(i) <= '1';
                else EqR(i) <= '0';
                end if;
                if HCnt(i) = 4 and ((LCnt > 0 and vEwt = L(0)) or (LCnt > 1 and vEwt = L(1))
                                 or (LCnt > 2 and vEwt = L(2)) or (LCnt > 3 and vEwt = L(3))) then InR(i) <= '1';
                else InR(i) <= '0';
                end if;
                -- complete: all data words of the answer are in the buffer
                if HCnt(i) = 4 and (vWd <= 4 or ("000" & RdCnt(13*i+12 downto 13*i)) >= vWd - 4) then CmpR(i) <= '1';
                else CmpR(i) <= '0';
                end if;
            end loop;

            ---------------- decision ----------------
            case St is
            when S_Dec =>
                if Settle = 0 and WrtEn = '1' and BuffRst = '0' and SeqIdle = '1' then
                    if EnR = '0' then
                        -- as before: all active ports have a complete answer
                        if Active /= 0 and (Active and HVal and CmpR and not Drain) = Active then
                            InclR <= Active; MissR <= X"00";
                            TimeGo <= TimeE;
                            St <= S_Go;
                            -- the list is not used: start again with the next request
                            if vCnt /= 0 then
                                vCnt := 0; vHeadChg := true; vListChg := true;
                            end if;
                        end if;
                    else
                        vRdy   := Active and EqR and CmpR and not Drain;
                        vMissN := Active and InR and not EqR and not Drain;
                        -- headers that are not in the list
                        for i in 0 to 7 loop
                            if Active(i) = '1' and HCnt(i) = 4 and InR(i) = '0' and Drain(i) = '0' then
                                vStart(i) := '1';
                                BadEwt <= Hdr(i)(47 downto 16);
                                BadPort <= conv_std_logic_vector(i, 3);
                            end if;
                        end loop;
                        if vCnt /= 0 and LCnt /= 0 and Active /= 0 then
                            vAllOk := (vRdy or vMissN) = Active;
                            vTO    := TimeoutR /= 0 and TimeE >= TimeoutR;
                            if vAllOk or vTO then
                                InclR <= vRdy; MissR <= Active and not vRdy;
                                EvEwtR <= L(0);
                                TimeGo <= TimeE;
                                St <= S_Go;
                                if (Active and not vRdy) /= 0 then
                                    MissEvCnt <= inc16(MissEvCnt);
                                    LastMiss  <= Active and not vRdy;
                                end if;
                                if not vAllOk then
                                    TOCnt <= inc16(TOCnt);
                                    -- what is there of an answer that did not get complete is thrown away
                                    for i in 0 to 7 loop
                                        if Active(i) = '1' and vRdy(i) = '0' and vMissN(i) = '0'
                                           and HCnt(i) /= 0 and Drain(i) = '0' then
                                            vStart(i) := '1';
                                        end if;
                                    end loop;
                                end if;
                            end if;
                        end if;
                    end if;
                end if;

            when S_Go =>
                if SeqIdle = '0' then St <= S_Busy;
                elsif WrtEn = '0' then St <= S_Dec;
                end if;

            when S_Busy =>
                if SeqIdle = '1' then
                    St <= S_Dec;
                    for i in 0 to 7 loop
                        if InclR(i) = '1' then HCnt(i) <= "000"; end if;
                    end loop;
                    if vCnt /= 0 and EnR = '1' and vL(0) = EvEwtR then
                        vL(0) := vL(1); vL(1) := vL(2); vL(2) := vL(3);
                        vCnt := vCnt - 1;
                        vHeadChg := true; vListChg := true;
                    end if;
                    EvCnt <= inc16(EvCnt);
                    LastTime <= TimeGo;
                    if TimeGo > MaxTime then MaxTime <= TimeGo; end if;
                end if;
            end case;

            for i in 0 to 7 loop
                if vStart(i) = '1' and BuffRst = '0' and WrtEn = '1' then
                    Drain(i) <= '1'; DrCnt(i) <= X"0";
                end if;
            end loop;
            if vStart /= 0 then
                DrainCnt <= inc16(DrainCnt);
                DrainedPorts <= DrainedPorts or vStart;
            end if;

            ---------------- list, timer ----------------
            -- requests of an earlier run or from before the write to the DDR was on
            if BuffRst = '1' or WrtEn = '0' or vRunRst then
                if vCnt /= 0 then vListChg := true; end if;
                vCnt := 0;
            end if;
            L <= vL; LCnt <= vCnt;

            if vHeadChg or vCnt = 0 then TimeE <= (others => '0');
            elsif Tick10us = '1' and TimeE /= X"FFFF" then TimeE <= TimeE + 1;
            end if;

            -- EqR/InR/CmpR follow the headers and the list with one clock delay
            if Pop /= 0 or Drain /= 0 or vListChg or St /= S_Dec or BuffRst = '1' or WrtEn = '0' then Settle <= "11";
            elsif Settle /= 0 then Settle <= Settle - 1;
            end if;

            if ClrStat = '1' then
                LastTime <= (others => '0'); MaxTime <= (others => '0');
                EvCnt <= (others => '0'); ReqCnt <= (others => '0'); DrainCnt <= (others => '0');
                MissEvCnt <= (others => '0'); TOCnt <= (others => '0'); ListDropCnt <= (others => '0');
                LastMiss <= X"00"; DrainedPorts <= X"00";
            end if;

        end if;
    end process;

end architecture rtl;

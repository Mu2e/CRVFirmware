-- Simulation only: the EventBuilder / PacketFormer from before the EWT matching
-- change (renamed to *_orig by run_ghdl.sh) and the old data request handler,
-- behind the new port lists, so that EwtMatch_tb can run on the old firmware
-- for comparison.

LIBRARY ieee;
use ieee.std_logic_1164.all;
use work.Project_Defs.all;

entity EventBuilder is
    port (
        clk, reset, FormRst : in std_logic;
        LinkFIFOOut     : in  Array_3x16;
        LinkFIFORdCnt   : in  Array_3x14;
        LinkFIFOEmpty   : in  std_logic_vector(2 downto 0);
        LinkFIFORdReq   : out std_logic_vector(2 downto 0);
        LinkFIFORst     : out std_logic;
        FlushReq        : in  std_logic := '0';
        FlushAck        : out std_logic;
        FlushNow        : in  std_logic := '0';
        Truncated       : out std_logic;
        EventBuff_Dat   : out std_logic_vector (15 downto 0);
        EventBuff_WrtEn : out std_logic;
        TStmpWds        : in  std_logic_vector( 8 downto 0);
        ActiveReg       : in  std_logic_vector(23 downto 0);
        MarkerDelayed   : in  std_logic_vector( 3 downto 0);
        LinkRDDL        : in  std_logic_vector (1 downto 0);
        AddrReg         : in  std_logic_vector(11 downto 0);
        FormHold, sendGR, uBinHeader, uBwrt : in std_logic;
        GA              : in  std_logic_vector( 1 downto 0);
        sendGrCnt       : in  std_logic_vector( 7 downto 0);
        ExtuBunchCount  : in  std_logic_vector(47 downto 0);
        HeartBtCnt, HeartBeatCnt, LastWindow, Stats, InjectionTs, InjectionWindow : in std_logic_vector(15 downto 0);
        FakeNum         : out std_logic_vector( 7 downto 0);
        EvUb_Dat        : out std_logic_vector(15 downto 0);
        EvUb_WrEn       : out std_logic;
        EvBuildState    : out std_logic_vector( 3 downto 0);
        uBDebugOut      : out std_logic_vector (96 downto 0));
end entity;
architecture wrap of EventBuilder is
begin
    EvUb_Dat <= (others => '0'); EvUb_WrEn <= '0'; EvBuildState <= X"0";
    FlushAck <= '0'; Truncated <= '0';
    u : entity work.EventBuilder_orig
        port map (clk, reset, FormRst, LinkFIFOOut, LinkFIFORdCnt, LinkFIFOEmpty, LinkFIFORdReq,
                  LinkFIFORst, EventBuff_Dat, EventBuff_WrtEn, TStmpWds, ActiveReg, MarkerDelayed,
                  LinkRDDL, AddrReg, FormHold, sendGR, uBinHeader, uBwrt, GA, sendGrCnt,
                  ExtuBunchCount, HeartBtCnt, HeartBeatCnt, LastWindow, Stats, InjectionTs,
                  InjectionWindow, FakeNum, uBDebugOut);
end architecture;

LIBRARY ieee;
use ieee.std_logic_1164.all;
entity PacketFormer is
    port (
        clk, reset, pktFormerSend, pktFormerTimeout : in std_logic;
        pktFormerDiscard : in  std_logic := '0';
        DiscardDone      : out std_logic;
        pktFormerIdle    : out std_logic;
        FormRst          : in  std_logic;
        EventBuff_Out    : in  std_logic_vector(15 downto 0);
        EventBuff_RdEn   : out std_logic;
        TStmpBuff_rd_en  : out std_logic;
        TStmpBuff_Out    : in  std_logic_vector(15 downto 0);
        EventBuff_Empty  : in  std_logic;
        ActiveReg        : in  std_logic_vector(23 downto 0);
        DCSBuffRdCnt     : in  std_logic_vector(12 downto 0);
        DCSBuff_Out      : in  std_logic_vector(15 downto 0);
        DCSBuff_rd_en    : out std_logic;
        DRdone           : out std_logic;
        DReqCount        : out std_logic_vector(31 downto 0);
        FormStat         : out std_logic_vector( 2 downto 0);
        TxCRCEn, TxCRCRst : out std_logic;
        TxCRCDat         : out std_logic_vector(15 downto 0);
        TxCRC            : in  std_logic_vector(15 downto 0);
        WdCountBuff_WrtEn : out std_logic;
        GTPTx            : out std_logic_vector(15 downto 0);
        TxCharIsK        : out std_logic_vector( 1 downto 0);
        GTPTxBuff_In     : out std_logic_vector(15 downto 0);
        GTPTxBuff_wr_en  : out std_logic;
        MarkerDelayed, loopbackMarker : in std_logic;
        LoopbackMode     : in  std_logic_vector( 2 downto 0);
        uBwrt            : in  std_logic;
        IDReg            : in  std_logic_vector( 3 downto 0));
end entity;
architecture wrap of PacketFormer is
begin
    DiscardDone <= '0'; pktFormerIdle <= '1'; DReqCount <= (others => '0'); FormStat <= "000";
    u : entity work.PacketFormer_orig
        port map (clk, reset, pktFormerSend, pktFormerTimeout, FormRst, EventBuff_Out, EventBuff_RdEn,
                  TStmpBuff_rd_en, TStmpBuff_Out, EventBuff_Empty, ActiveReg, DCSBuffRdCnt, DCSBuff_Out,
                  DCSBuff_rd_en, DRdone, TxCRCEn, TxCRCRst, TxCRCDat, TxCRC, WdCountBuff_WrtEn,
                  GTPTx, TxCharIsK, GTPTxBuff_In, GTPTxBuff_wr_en, MarkerDelayed, loopbackMarker,
                  LoopbackMode, uBwrt, IDReg);
end architecture;

-- the data request handler as it was in Controller_FPGA1.vhd (TrigReqTx process)
LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
entity DRHandler is
    port (
        clk, reset       : in  std_logic;
        TStmpWds         : in  std_logic_vector( 8 downto 0);
        TStmpBuff_Out    : in  std_logic_vector(15 downto 0);
        TStmpBuff_rd_en  : out std_logic;
        EvUbBuff_Out     : in  std_logic_vector(15 downto 0);
        EvUbBuff_Empty   : in  std_logic;
        EvUbBuff_rd_en   : out std_logic;
        EventBuff_Empty  : in  std_logic;
        pktFormerIdle, DRdone, DiscardDone : in std_logic;
        pktFormerSend, pktFormerTimeout, pktFormerDiscard : out std_logic;
        DRHold           : in  std_logic;
        DRTimeout        : in  std_logic_vector(15 downto 0);
        MatchBypass, DropLateEn : in std_logic;
        TagGuardEn       : in  std_logic := '1';
        BadTag           : out std_logic;
        DRcnt, EwtDropCnt, EvDropCnt, LastDropEvUb, LastDropReqEwt : out std_logic_vector(15 downto 0);
        State            : out std_logic_vector( 3 downto 0));
end entity;
architecture legacy of DRHandler is
    Type DR_Handler_Seq is (Idle,StartCounter,Waiting,Send,Timeout);
    signal DR_Handler : DR_Handler_Seq;
    signal DRcnt_i : std_logic_vector(15 downto 0);
begin
    TStmpBuff_rd_en <= '0'; EvUbBuff_rd_en <= '0'; pktFormerDiscard <= '0';
    EwtDropCnt <= (others => '0'); EvDropCnt <= (others => '0');
    LastDropEvUb <= (others => '0'); LastDropReqEwt <= (others => '0');
    State <= X"0"; DRcnt <= DRcnt_i; BadTag <= '0';
    process(clk, reset)
    begin
        if reset = '1' then
            DRcnt_i <= (others =>'0');
            pktFormerTimeout <= '0';
            pktFormerSend <= '0';
            DR_Handler <= Idle;
        elsif rising_edge(clk) then
            Case DR_Handler is
                when Idle =>
                    if TStmpWds >= 3 and DRdone = '0' and DRHold = '0' then
                        DR_Handler <= StartCounter;
                    else
                        DR_Handler <= Idle;
                    end if;
                when StartCounter =>
                    DR_Handler <= Waiting;
                when Waiting =>
                    if EventBuff_Empty = '0' then DR_Handler <= Send; -- data avaiable
                    elsif DRcnt_i = 0        then DR_Handler <= Timeout;
                    else                          DR_Handler <= Waiting;
                    end if;
                when Send =>
                    if DRdone = '1' then DR_Handler <= Idle;
                    else                 DR_Handler <= Send;
                    end if;
                when Timeout =>
                    if DRdone = '1' then DR_Handler <= Idle;
                    else                 DR_Handler <= Timeout;
                    end if;
                when others =>
                    DR_Handler <= Idle;
            end Case;

            if DR_Handler = StartCounter then
                DRcnt_i <= DRTimeout;
            elsif DR_Handler = Waiting and DRTimeout /= X"FFFF" then
                DRcnt_i <= DRcnt_i - 1;
            else DRcnt_i <= DRcnt_i;
            end if;

            if DR_Handler = Send then
                pktFormerSend    <= '1';
                pktFormerTimeout <= '0';
            elsif DR_Handler = Timeout then
                pktFormerSend    <= '1';
                pktFormerTimeout <= '1';
            else
                pktFormerSend    <= '0';
                pktFormerTimeout <= '0';
            end if;
        end if;
    end process;
end architecture;

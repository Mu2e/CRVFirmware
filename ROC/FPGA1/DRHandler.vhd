-- Data request handler for the CRV ROC, FPGA 1
--
-- Decides when the PacketFormer may send the oldest event in the EventBuffer.
-- An event leaves FPGA1 only with the event window tag (EWT) it was requested
-- with. Before this module the oldest event was sent with the oldest EWT and
-- nothing compared the two.
--
--   req_EWT : oldest entry of the time stamp buffer (TStmpBuff, 3 words per
--             request, low word first, so TStmpBuff_Out shows EWT(15 downto 0)
--             whenever the buffer is at an entry boundary)
--   ev_uB   : EWT(15 downto 0) of the oldest event not yet handed to the
--             PacketFormer, from EvUbBuff (one entry per event, written by the
--             EventBuilder at the start of the event, so the header can go out
--             while the payload is still being copied)
--
--   ev_uB = req_EWT, no newer EWT queued  -> send
--   ev_uB = req_EWT, a newer EWT queued   -> the event is late, the DTC has
--                                            moved on: drop event and EWT
--                                            (only if DropLateEn = '1',
--                                             otherwise send)
--   ev_uB newer than req_EWT,             -> that request never produced an
--     a newer EWT queued                     event: drop the EWT, compare again
--   ev_uB newer than req_EWT,             -> no request can have produced this
--     no newer EWT queued                    event (an event exists only after
--                                            its request arrived): drop the
--                                            event, keep the EWT, report BadTag
--                                            (only if TagGuardEn = '1',
--                                             otherwise drop the EWT)
--   ev_uB older than req_EWT              -> late event of a request that was
--                                            dropped already: drop the event
--
--   no event yet, a newer EWT queued      -> the DTC has moved on: drop the
--                                            older EWT at once (only if
--                                            DropLateEn = '1'). The time stamp
--                                            buffer then never holds more than
--                                            the newest requests and cannot
--                                            fill up when no events come.
--
-- Nothing is sent for a dropped EWT or a dropped event.
--
-- The comparison is a wrap safe signed difference of the low 16 bits.
--
-- MatchBypass = '1' gives the previous order-only behaviour (used with sendGR,
-- where the events are not built from requests). EvUbBuff is drained in that
-- mode. Change MatchBypass only while no requests are pending.
--
-- A decision is taken only while the PacketFormer is idle and a complete EWT
-- is queued, and each EvUbBuff entry is used for exactly one event, so a header
-- can no longer be started for an event that does not exist.

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity DRHandler is
    port (
        clk              : in  std_logic;
        reset            : in  std_logic; -- active high, asynchronous (GTPRxRst)
        -- time stamp buffer (requested EWTs)
        TStmpWds         : in  std_logic_vector( 8 downto 0);
        TStmpBuff_Out    : in  std_logic_vector(15 downto 0);
        TStmpBuff_rd_en  : out std_logic; -- to be ORed with the PacketFormer read
        -- EWT of the built events
        EvUbBuff_Out     : in  std_logic_vector(15 downto 0);
        EvUbBuff_Empty   : in  std_logic;
        EvUbBuff_rd_en   : out std_logic;
        -- only used with MatchBypass = '1'
        EventBuff_Empty  : in  std_logic;
        -- PacketFormer
        pktFormerIdle    : in  std_logic;
        DRdone           : in  std_logic;
        DiscardDone      : in  std_logic;
        pktFormerSend    : out std_logic;
        pktFormerTimeout : out std_logic;
        pktFormerDiscard : out std_logic;
        -- settings
        DRHold           : in  std_logic;
        DRTimeout        : in  std_logic_vector(15 downto 0); -- X"FFFF": no timeout
        MatchBypass      : in  std_logic;
        DropLateEn       : in  std_logic;
        TagGuardEn       : in  std_logic := '1';
        -- one clock: event newer than the request and no newer request queued
        -- (also with TagGuardEn = '0')
        BadTag           : out std_logic;
        -- status
        DRcnt            : out std_logic_vector(15 downto 0);
        EwtDropCnt       : out std_logic_vector(15 downto 0); -- requests without event
        EvDropCnt        : out std_logic_vector(15 downto 0); -- events without request (late)
        LastDropEvUb     : out std_logic_vector(15 downto 0); -- ev_uB at the last drop
        LastDropReqEwt   : out std_logic_vector(15 downto 0); -- req_EWT at the last drop
        State            : out std_logic_vector( 3 downto 0)
    );
end entity DRHandler;

architecture rtl of DRHandler is

    Type DR_Handler_Seq is (Idle,StartCounter,Waiting,Compare,Send,Timeout,
                            DropEwt,DropEvent,Settle,DropOld);
    signal DR_Handler : DR_Handler_Seq;

    signal DRcnt_i       : std_logic_vector(15 downto 0);
    signal Diff          : std_logic_vector(15 downto 0);
    signal DropEvAfter   : std_logic; -- drop the event after its EWT (late event)
    signal PopCnt        : std_logic_vector(1 downto 0);
    signal SettleCnt     : std_logic_vector(2 downto 0);
    signal EwtDropCnt_i  : std_logic_vector(15 downto 0);
    signal EvDropCnt_i   : std_logic_vector(15 downto 0);
    signal Bypass, DropLate, Guard : std_logic;
    signal Later, CmpBadTag : std_logic;
    signal EvUb_rd_i     : std_logic;
    -- result of the comparison, valid in state Compare
    signal Equal, Late   : std_logic;
    signal CmpSend, CmpDropBoth, CmpDropEwt, CmpDropEv : std_logic;

begin

    Equal <= '1' when Diff = 0 else '0';
    -- a newer request is completely queued behind the one being compared
    Later <= '1' when TStmpWds >= 6 else '0';
    Late  <= Later and DropLate;

    CmpSend     <= Equal and not Late;
    CmpDropBoth <= Equal and Late;
    CmpDropEwt  <= not Equal and not Diff(15); -- event newer than request
    CmpDropEv   <= not Equal and Diff(15);     -- event older than request
    CmpBadTag   <= CmpDropEwt and not Later;   -- newer, but nothing was requested after req_EWT

    DRcnt          <= DRcnt_i;
    EwtDropCnt     <= EwtDropCnt_i;
    EvDropCnt      <= EvDropCnt_i;
    EvUbBuff_rd_en <= EvUb_rd_i;

    process(clk, reset)
    begin
        if reset = '1' then
            DR_Handler       <= Idle;
            DRcnt_i          <= (others => '0');
            Diff             <= (others => '0');
            DropEvAfter      <= '0';
            PopCnt           <= "00";
            SettleCnt        <= "000";
            EwtDropCnt_i     <= (others => '0');
            EvDropCnt_i      <= (others => '0');
            LastDropEvUb     <= (others => '0');
            LastDropReqEwt   <= (others => '0');
            Bypass           <= '0';
            DropLate         <= '0';
            Guard            <= '0';
            BadTag           <= '0';
            pktFormerSend    <= '0';
            pktFormerTimeout <= '0';
            pktFormerDiscard <= '0';
            TStmpBuff_rd_en  <= '0';
            EvUb_rd_i        <= '0';
            State            <= X"0";

        elsif rising_edge(clk) then

            -- settings come from the uC clock domain and are quasi static
            Bypass   <= MatchBypass;
            DropLate <= DropLateEn;
            Guard    <= TagGuardEn;

            if DR_Handler = Compare and CmpBadTag = '1' then BadTag <= '1';
            else                                             BadTag <= '0';
            end if;

            -- Both FIFO outputs are stable for at least one clock before the
            -- Compare state is entered, see Waiting.
            Diff <= EvUbBuff_Out - TStmpBuff_Out;

            Case DR_Handler is
                -- A complete EWT is queued and the previous hand shake with the
                -- PacketFormer is over (its reads of the time stamp buffer too).
                when Idle =>
                    if TStmpWds >= 3 and DRdone = '0' and DiscardDone = '0' and DRHold = '0'
                    then DR_Handler <= StartCounter;
                    else DR_Handler <= Idle;
                    end if;

                when StartCounter =>
                    DR_Handler <= Waiting;

                -- Wait for the next event. Never decide while the
                -- PacketFormer is busy.
                when Waiting =>
                    if pktFormerIdle = '1' and Bypass = '1' and EventBuff_Empty = '0' then
                        DR_Handler <= Send;
                    elsif pktFormerIdle = '1' and Bypass = '0' and EvUbBuff_Empty = '0' and EvUb_rd_i = '0' then
                        DR_Handler <= Compare;
                    elsif pktFormerIdle = '1' and Bypass = '0' and Late = '1' then
                        DR_Handler <= DropOld;  -- no event, and a newer request is queued
                    elsif pktFormerIdle = '1' and DRcnt_i = 0 then
                        DR_Handler <= Timeout;
                    else
                        DR_Handler <= Waiting;
                    end if;

                when Compare =>
                    if    CmpSend = '1'   then DR_Handler <= Send;
                    elsif CmpDropEv = '1' or (CmpBadTag = '1' and Guard = '1')
                                          then DR_Handler <= DropEvent; -- event without request
                    else                       DR_Handler <= DropEwt;   -- request without event, or late: both
                    end if;

                -- hold the request until the PacketFormer has started the header
                when Send =>
                    if DRdone = '1' then DR_Handler <= Settle;
                    else                 DR_Handler <= Send;
                    end if;

                when Timeout =>
                    if DRdone = '1' then DR_Handler <= Settle;
                    else                 DR_Handler <= Timeout;
                    end if;

                -- drop the oldest EWT without an event
                when DropOld =>
                    DR_Handler <= DropEwt;

                -- three reads of the time stamp buffer
                when DropEwt =>
                    if PopCnt = 0 then
                        if DropEvAfter = '1' then DR_Handler <= DropEvent;
                        else                      DR_Handler <= Settle;
                        end if;
                    else
                        DR_Handler <= DropEwt;
                    end if;

                when DropEvent =>
                    if DiscardDone = '1' then DR_Handler <= Settle;
                    else                      DR_Handler <= DropEvent;
                    end if;

                -- let the FIFO word counts and outputs settle
                when Settle =>
                    if SettleCnt = 0 then DR_Handler <= Idle;
                    else                  DR_Handler <= Settle;
                    end if;

                when others =>
                    DR_Handler <= Idle;
            end Case;

            -- time out counter, X"FFFF" disables it
            if DR_Handler = StartCounter then
                DRcnt_i <= DRTimeout;
            elsif DR_Handler = Waiting and DRTimeout /= X"FFFF" and DRcnt_i /= 0 then
                DRcnt_i <= DRcnt_i - 1;
            else
                DRcnt_i <= DRcnt_i;
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

            if DR_Handler = DropEvent then pktFormerDiscard <= '1';
            else                           pktFormerDiscard <= '0';
            end if;

            -- late event: remember to drop the event after its EWT
            if    DR_Handler = Compare then DropEvAfter <= CmpDropBoth;
            elsif DR_Handler = DropOld then DropEvAfter <= '0';
            else                            DropEvAfter <= DropEvAfter;
            end if;

            -- drop one EWT: TStmpBuff_rd_en is high for exactly three clocks
            if DR_Handler = Compare or DR_Handler = DropOld then
                PopCnt <= "11";
            elsif DR_Handler = DropEwt and PopCnt /= 0 then
                PopCnt <= PopCnt - 1;
            else
                PopCnt <= PopCnt;
            end if;

            if DR_Handler = DropEwt and PopCnt /= 0 then TStmpBuff_rd_en <= '1';
            else                                         TStmpBuff_rd_en <= '0';
            end if;

            -- One read of EvUbBuff for each event handed to the PacketFormer,
            -- to be sent or to be discarded. With MatchBypass the entries are
            -- not used and are drained.
            if DR_Handler = Compare and (CmpSend = '1' or CmpDropEv = '1'
                                         or (CmpBadTag = '1' and Guard = '1')) then
                EvUb_rd_i <= '1';                                   -- Compare -> Send or DropEvent
            elsif DR_Handler = DropEwt and PopCnt = 0 and DropEvAfter = '1' then
                EvUb_rd_i <= '1';                                   -- DropEwt -> DropEvent
            elsif Bypass = '1' and EvUbBuff_Empty = '0' and EvUb_rd_i = '0' then
                EvUb_rd_i <= '1';
            else
                EvUb_rd_i <= '0';
            end if;

            if DR_Handler = Settle and SettleCnt /= 0 then SettleCnt <= SettleCnt - 1;
            elsif DR_Handler = Settle                 then SettleCnt <= SettleCnt;
            else                                           SettleCnt <= "111";
            end if;

            -- counters, saturating
            if ((DR_Handler = Compare and ((CmpDropEwt = '1' and not (CmpBadTag = '1' and Guard = '1'))
                                           or CmpDropBoth = '1'))
                or DR_Handler = DropOld) and
               EwtDropCnt_i /= X"FFFF"
            then EwtDropCnt_i <= EwtDropCnt_i + 1;
            else EwtDropCnt_i <= EwtDropCnt_i;
            end if;

            if ((DR_Handler = Compare and (CmpDropEv = '1' or (CmpBadTag = '1' and Guard = '1'))) or
                (DR_Handler = DropEwt and PopCnt = 0 and DropEvAfter = '1')) and
               EvDropCnt_i /= X"FFFF"
            then EvDropCnt_i <= EvDropCnt_i + 1;
            else EvDropCnt_i <= EvDropCnt_i;
            end if;

            -- The pair that led to the last drop. A request dropped without an
            -- event (DropOld) only updates the request side.
            if DR_Handler = Compare and CmpSend = '0' then
                LastDropEvUb   <= EvUbBuff_Out;
                LastDropReqEwt <= TStmpBuff_Out;
            elsif DR_Handler = DropOld then
                LastDropReqEwt <= TStmpBuff_Out;
            end if;

            Case DR_Handler is
                when Idle         => State <= X"0";
                when StartCounter => State <= X"1";
                when Waiting      => State <= X"2";
                when Compare      => State <= X"3";
                when Send         => State <= X"4";
                when Timeout      => State <= X"5";
                when DropEwt      => State <= X"6";
                when DropEvent    => State <= X"7";
                when Settle       => State <= X"8";
                when DropOld      => State <= X"9";
                when others       => State <= X"F";
            end Case;

        end if;
    end process;

end architecture rtl;

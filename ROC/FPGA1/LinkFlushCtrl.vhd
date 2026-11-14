-- Link FIFO flush control for the CRV ROC, FPGA 1
--
-- The link FIFOs (data from the three data FPGAs) carry no framing: the event
-- builder takes the first word as a word count and the following words as a
-- header. If a FIFO ever starts in the middle of an event the builder builds
-- nonsense or waits for words that never come, the FIFOs are never empty again
-- and the data FPGAs stop sending. This module asks the event builder to
-- empty the three link FIFOs when what they hold cannot belong to a request
-- the DTC is still waiting for:
--
--   BadTag : the DRHandler found an event that is newer than the oldest
--            request while no newer request is queued. No request can have
--            produced it, the framing of the link FIFOs is lost.
--   stale  : a data request arrives from the DTC while the link FIFOs hold
--            data that the event builder is not building (it is idle or
--            waiting for an event to become complete). The DTC sends a
--            request only after the previous one was answered or timed out,
--            so what is there belongs to an event the DTC has given up on.
--            If the builder is in the middle of an event nothing is done: it
--            takes that event out of the FIFOs itself and the DRHandler drops
--            it as late.
--
-- The event builder never stops an event half way (the EventBuffer stays
-- framed):
--   FlushReq (bad tag) is a level, honoured when the builder is back in Idle
--            and answered with FlushAck. If the link FIFOs are empty by then
--            nothing is reset.
--   FlushNow (stale) is one clock, honoured only if the builder is idle or
--            waiting in that clock.
-- A flush can hit while a data FPGA is still sending; the rest of that event
-- then stays in the FIFO without header and is removed by the next request.
--
-- With FlushEn = '0' nothing is requested, the counters and the snapshot still
-- record what would have been done. Nothing is done with sendGR = '1'.
--
-- Counters stop at X"FFFF" and are cleared by reset (GTPRxRst, run start).

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

use work.Project_defs.all;

entity LinkFlushCtrl is
    port (
        clk           : in  std_logic;
        reset         : in  std_logic; -- active high, asynchronous (GTPRxRst)
        -- triggers
        DReqRx        : in  std_logic; -- one clock: data request received from the DTC
        BadTag        : in  std_logic; -- one clock: from the DRHandler
        -- settings
        FlushEn       : in  std_logic;
        sendGR        : in  std_logic;
        -- event builder
        EvBuildState  : in  std_logic_vector( 3 downto 0);
        FlushReq      : out std_logic;
        FlushAck      : in  std_logic;
        FlushNow      : out std_logic;
        Truncated     : in  std_logic; -- event cut at the size limit (link FIFOs are reset for that as before)
        LinkFIFORst   : in  std_logic; -- reset of the link FIFOs as it is applied
        -- link FIFOs, for the trigger and the snapshot
        LinkFIFOEmpty : in  std_logic_vector( 2 downto 0);
        LinkFIFOOut   : in  Array_3x16;
        LinkFIFORdCnt : in  Array_3x14;
        TStmpWds      : in  std_logic_vector( 8 downto 0);
        -- status
        LinkFlushCnt  : out std_logic_vector(15 downto 0); -- resets of the link FIFOs, any cause
        BadTagCnt     : out std_logic_vector(15 downto 0);
        StaleCnt      : out std_logic_vector(15 downto 0);
        TruncCnt      : out std_logic_vector(15 downto 0);
        -- at the last BadTag / stale trigger:
        -- [0] bad tag, [1] stale, [2] FlushEn, [7:4] event builder state,
        -- [10:8] link FIFO empty flags, [15:12] words in the time stamp buffer (stops at 15)
        SnapCause     : out std_logic_vector(15 downto 0);
        SnapHead      : out Array_3x16;                    -- first word of each link FIFO
        SnapWds       : out Array_3x14                    -- words in each link FIFO
    );
end entity LinkFlushCtrl;

architecture rtl of LinkFlushCtrl is

    signal En, GR          : std_logic;
    signal Stale           : std_logic;
    signal FlushReq_i      : std_logic;
    signal TruncD, RstD    : std_logic;
    signal LinkFlushCnt_i, BadTagCnt_i, StaleCnt_i, TruncCnt_i : std_logic_vector(15 downto 0);

begin

    FlushReq     <= FlushReq_i;
    LinkFlushCnt <= LinkFlushCnt_i;
    BadTagCnt    <= BadTagCnt_i;
    StaleCnt     <= StaleCnt_i;
    TruncCnt     <= TruncCnt_i;

    -- EvBuildState: 0 is Idle, 1 is WaitEvent
    Stale <= '1' when DReqRx = '1' and GR = '0' and LinkFIFOEmpty /= "111"
                      and (EvBuildState = X"0" or EvBuildState = X"1")
             else '0';

    process(clk, reset)
    begin
        if reset = '1' then
            En <= '0'; GR <= '0';
            FlushReq_i     <= '0';
            FlushNow       <= '0';
            TruncD <= '0'; RstD <= '0';
            LinkFlushCnt_i <= (others => '0');
            BadTagCnt_i    <= (others => '0');
            StaleCnt_i     <= (others => '0');
            TruncCnt_i     <= (others => '0');
            SnapCause      <= (others => '0');
            SnapHead       <= (others => (others => '0'));
            SnapWds        <= (others => (others => '0'));

        elsif rising_edge(clk) then

            -- settings come from the uC clock domain and are quasi static
            En <= FlushEn;
            GR <= sendGR;

            if FlushAck = '1' then
                FlushReq_i <= '0';
            elsif En = '1' and BadTag = '1' then
                FlushReq_i <= '1';
            else
                FlushReq_i <= FlushReq_i;
            end if;

            FlushNow <= En and Stale;

            if (BadTag = '1' or Stale = '1') then
                SnapCause(0)            <= BadTag;
                SnapCause(1)            <= Stale;
                SnapCause(2)            <= En;
                SnapCause(3)            <= '0';
                SnapCause(7 downto 4)   <= EvBuildState;
                SnapCause(10 downto 8)  <= LinkFIFOEmpty;
                SnapCause(11)           <= '0';
                if TStmpWds > 15 then SnapCause(15 downto 12) <= X"F";
                else                  SnapCause(15 downto 12) <= TStmpWds(3 downto 0);
                end if;
                SnapHead   <= LinkFIFOOut;
                SnapWds    <= LinkFIFORdCnt;
            end if;

            TruncD <= Truncated;
            RstD   <= LinkFIFORst;

            if BadTag = '1' and BadTagCnt_i /= X"FFFF" then BadTagCnt_i <= BadTagCnt_i + 1;
            else                                            BadTagCnt_i <= BadTagCnt_i;
            end if;

            if Stale = '1' and StaleCnt_i /= X"FFFF" then StaleCnt_i <= StaleCnt_i + 1;
            else                                          StaleCnt_i <= StaleCnt_i;
            end if;

            if Truncated = '1' and TruncD = '0' and TruncCnt_i /= X"FFFF" then TruncCnt_i <= TruncCnt_i + 1;
            else                                                               TruncCnt_i <= TruncCnt_i;
            end if;

            if LinkFIFORst = '1' and RstD = '0' and LinkFlushCnt_i /= X"FFFF" then LinkFlushCnt_i <= LinkFlushCnt_i + 1;
            else                                                                  LinkFlushCnt_i <= LinkFlushCnt_i;
            end if;

        end if;
    end process;

end architecture rtl;

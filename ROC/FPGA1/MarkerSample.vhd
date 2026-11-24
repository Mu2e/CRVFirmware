-- Sampling of the marker line for the CRV ROC, FPGA 1
--
-- The marker line carries a 40 MHz clock with stable rising edges; a marker is
-- a "punch", a long low followed by a long high. The line is sampled on both
-- edges of the 160 MHz clock in the input pad (IDDR2), which gives two sample
-- streams, 3.125 ns apart:
--   A : DDR(0), taken on the rising edge
--   B : DDR(1), taken on the falling edge before it
-- The marker decoder uses one sample per 160 MHz clock. The phase between the
-- marker line and the 160 MHz clock is fixed for a given cable, but unknown.
-- If the rising edge of the line sits on the sample point of the stream in
-- use, the edge is seen in one clock or in the next at random and the decoded
-- marker jumps by 6.25 ns.
--
-- This module chooses the stream once after reset and then keeps it:
--   * For every rising edge of the ordinary clock pulses it notes in which of
--     the four 160 MHz clocks of a 25 ns period the edge is seen, for both
--     streams. A stream that always sees it in the same clock is clean.
--   * The first window of 2**MEAS_BITS edges after reset is thrown away (the
--     clock may just have started). After the second window stream A is taken
--     if it was clean, otherwise stream B. If neither was clean the window is
--     repeated (Stat bit 4 remembers that) and stream A stays in use.
--   * After the choice ("locked") nothing is changed any more. Changes of the
--     clock in which an edge is seen are only counted, for both streams.
-- The rising edge that starts the long high of a punch is not used for this
-- (it follows 4 or more low samples). The decoded marker is checked instead:
-- markers are a whole number of 25 ns apart, so the chosen stream must show
-- the pattern 0001111 always in the same clock of the 25 ns period. Changes
-- are counted in Stat(15 downto 8).
--
-- Restart (from the uC clock domain, at least one 160 MHz period long) starts
-- the measurement again. Counters stop at X"FF".

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity MarkerSample is
    generic (
        MEAS_BITS : integer := 16
    );
    port (
        clk      : in  std_logic; -- 160 MHz
        rst_n    : in  std_logic; -- active low, asynchronous
        Restart  : in  std_logic;
        DDR      : in  std_logic_vector( 1 downto 0); -- (1) older sample, (0) newer sample
        Samp     : out std_logic;                     -- chosen stream, one sample per clock
        -- [0] stream in use (0 = A, 1 = B), [1] locked, [2] A not clean in the
        -- window, [3] B not clean in the window, [4] a window had to be repeated,
        -- [15:8] changes of the marker position since lock
        Stat     : out std_logic_vector(15 downto 0);
        -- changes of the edge position since lock: [7:0] stream A, [15:8] stream B
        EdgeJump : out std_logic_vector(15 downto 0)
    );
end entity MarkerSample;

architecture rtl of MarkerSample is

    type Hist_t is array(0 to 1) of std_logic_vector(5 downto 0);
    type Ph_t   is array(0 to 1) of std_logic_vector(1 downto 0);
    type Cnt_t  is array(0 to 1) of std_logic_vector(7 downto 0);

    signal RstS     : std_logic_vector(1 downto 0);
    signal Phase    : std_logic_vector(1 downto 0);
    signal Hist     : Hist_t;   -- earlier samples of each stream, (0) is the last one
    signal Ref      : Ph_t;
    signal HaveRef  : std_logic_vector(1 downto 0);
    signal Dirty    : std_logic_vector(1 downto 0);
    signal JumpCnt  : Cnt_t;
    signal MeasCnt  : std_logic_vector(MEAS_BITS-1 downto 0);
    signal Settled, Locked, Sel, Retry : std_logic;
    signal DirtyAtLock : std_logic_vector(1 downto 0);
    signal RefM     : std_logic_vector(1 downto 0);
    signal HaveRefM : std_logic;
    signal MJump    : std_logic_vector(7 downto 0);

    constant MeasEnd : std_logic_vector(MEAS_BITS-1 downto 0) := (others => '1');

begin

    Stat     <= MJump & "000" & Retry & DirtyAtLock(1) & DirtyAtLock(0) & Locked & Sel;
    EdgeJump <= JumpCnt(1) & JumpCnt(0);

    process(clk, rst_n)
        variable vEdge, vChg : std_logic_vector(1 downto 0);
        variable vDirty      : std_logic_vector(1 downto 0);
        variable vCur        : std_logic;
        variable vHist       : std_logic_vector(5 downto 0);
    begin
        if rst_n = '0' then
            RstS <= "00"; Phase <= "00";
            Hist <= (others => (others => '0'));
            Ref  <= (others => "00");
            HaveRef <= "00"; Dirty <= "00";
            JumpCnt <= (others => (others => '0'));
            MeasCnt <= (others => '0');
            Settled <= '0'; Locked <= '0'; Sel <= '0'; Retry <= '0';
            DirtyAtLock <= "00";
            RefM <= "00"; HaveRefM <= '0'; MJump <= (others => '0');
            Samp <= '0';

        elsif rising_edge(clk) then

            RstS  <= RstS(0) & Restart;
            Phase <= Phase + 1;

            -- rising edge of an ordinary clock pulse: high after 1 to 3 low samples
            for i in 0 to 1 loop
                Hist(i) <= Hist(i)(4 downto 0) & DDR(i);
                if DDR(i) = '1' and Hist(i)(0) = '0' and Hist(i)(3 downto 1) /= "000"
                then vEdge(i) := '1';
                else vEdge(i) := '0';
                end if;
                vChg(i) := '0';
                if vEdge(i) = '1' then
                    Ref(i)     <= Phase;
                    HaveRef(i) <= '1';
                    if HaveRef(i) = '1' and Phase /= Ref(i) then vChg(i) := '1'; end if;
                end if;
            end loop;

            vDirty := Dirty or vChg;

            if Locked = '0' then
                Dirty <= vDirty;
                if vEdge(0) = '1' then
                    if MeasCnt /= MeasEnd then
                        MeasCnt <= MeasCnt + 1;
                    else
                        MeasCnt <= (others => '0');
                        Dirty   <= "00";
                        if Settled = '0' then
                            Settled <= '1';
                        elsif vDirty = "11" then
                            Retry <= '1';
                        else
                            Locked      <= '1';
                            Sel         <= vDirty(0);
                            DirtyAtLock <= vDirty;
                        end if;
                    end if;
                end if;
            else
                for i in 0 to 1 loop
                    if vChg(i) = '1' and JumpCnt(i) /= X"FF" then JumpCnt(i) <= JumpCnt(i) + 1; end if;
                end loop;
            end if;

            -- chosen stream and check of the decoded marker (0001111)
            if Sel = '1' then vCur := DDR(1); vHist := Hist(1);
            else              vCur := DDR(0); vHist := Hist(0);
            end if;
            Samp <= vCur;

            if Locked = '1' and vCur = '1' and vHist = "000111" then
                RefM     <= Phase;
                HaveRefM <= '1';
                if HaveRefM = '1' and Phase /= RefM and MJump /= X"FF" then MJump <= MJump + 1; end if;
            end if;

            if RstS(1) = '1' then
                HaveRef <= "00"; Dirty <= "00";
                JumpCnt <= (others => (others => '0'));
                MeasCnt <= (others => '0');
                Settled <= '0'; Locked <= '0'; Sel <= '0'; Retry <= '0';
                DirtyAtLock <= "00";
                HaveRefM <= '0'; MJump <= (others => '0');
            end if;

        end if;
    end process;

end architecture rtl;

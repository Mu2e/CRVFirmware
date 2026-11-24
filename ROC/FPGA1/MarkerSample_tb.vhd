-- Test of MarkerSample: a marker line with stable rising edges, falling edges
-- that alternate (60:40 / 40:60), punches, and jitter on the rising edge, at
-- different phases against the 160 MHz clock.
--
-- The line is sampled as the IDDR2 in the pad does it (C0 alignment): at a
-- rising clock edge DDR(0) is the line at that edge, DDR(1) the line at the
-- falling edge before it. The marker decoder of Controller_FPGA1 (0001111 on
-- one sample per clock) is copied here to check when the marker comes out.

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.math_real.all;

entity MarkerSample_tb is
end entity MarkerSample_tb;

architecture sim of MarkerSample_tb is

    constant TCLK : time := 6.25 ns;

    signal clk     : std_logic := '0';
    signal rst_n   : std_logic := '0';
    signal Restart : std_logic := '0';
    signal Line    : std_logic := '0';
    signal LineF   : std_logic := '0';
    signal DDR     : std_logic_vector(1 downto 0) := "00";
    signal Samp    : std_logic;
    signal Stat, EdgeJump : std_logic_vector(15 downto 0);

    -- stimulus settings
    signal Offset  : time := 0 ns;     -- rising edge of the line after a rising clock edge
    signal Jitter  : time := 0 ns;     -- rising edge moves by up to +- this
    signal Run     : boolean := false;
    signal Clr     : boolean := false;

    -- decoder copy
    signal Last    : std_logic_vector(6 downto 0) := (others => '0');
    signal PunchT  : time := 0 ns;     -- nominal rising edge of the last punch high
    signal MkCnt   : integer := 0;
    signal MkLat, MkLatMin, MkLatMax : time := 0 ns;

    signal Fail    : boolean := false;
    signal Done    : boolean := false;

begin

    clk <= not clk after TCLK/2 when not Done else '0';

    dut : entity work.MarkerSample
        generic map (MEAS_BITS => 10)
        port map (clk => clk, rst_n => rst_n, Restart => Restart, DDR => DDR,
                  Samp => Samp, Stat => Stat, EdgeJump => EdgeJump);

    -- pad register
    process(clk)
    begin
        if falling_edge(clk) then LineF <= Line; end if;
        if rising_edge(clk)  then DDR <= LineF & Line; end if;
    end process;

    -- marker line: period 25 ns, high 15 ns and 10 ns in turn; every 40th
    -- period starts a punch: one period without pulse, then a high of 45 ns
    process
        variable s1 : positive := 11;
        variable s2 : positive := 7;
        variable r  : real;
        variable n  : integer := 0;
        variable t0, tr, hi : time;
        variable base : time;
    begin
        wait until Run;
        wait until rising_edge(clk);
        base := now + 4*TCLK;
        n := 0;
        while Run loop
            uniform(s1, s2, r);
            t0 := base + n * 25 ns + Offset;
            tr := t0 + (2.0*r - 1.0) * Jitter;
            if n mod 2 = 0 then hi := 15 ns; else hi := 10 ns; end if;
            if n mod 40 = 20 then
                null;                           -- no pulse
            elsif n mod 40 = 21 then
                wait for tr - now;
                Line <= '1'; PunchT <= t0;
                wait for 45 ns;
                Line <= '0';
            elsif n mod 40 = 22 then
                null;                           -- still in the long high
            else
                wait for tr - now;
                Line <= '1';
                wait for hi;
                Line <= '0';
            end if;
            n := n + 1;
        end loop;
        Line <= '0';
        wait for 100 ns;
    end process;

    -- decoder as in Controller_FPGA1
    process(clk)
    begin
        if rising_edge(clk) then
            Last <= Last(5 downto 0) & Samp;
            if Clr then
                MkCnt <= 0;
            elsif Last = "0001111" and Stat(1) = '1' then
                MkCnt <= MkCnt + 1;
                MkLat <= now - PunchT;
                if MkCnt = 0 then
                    MkLatMin <= now - PunchT; MkLatMax <= now - PunchT;
                else
                    if now - PunchT < MkLatMin then MkLatMin <= now - PunchT; end if;
                    if now - PunchT > MkLatMax then MkLatMax <= now - PunchT; end if;
                end if;
            end if;
        end if;
    end process;

    process
        variable selFirst : std_logic;

        procedure one(name : string; off : time; jit : time; expSel : character; expClean : boolean) is
            variable sp : time;
        begin
            Offset <= off; Jitter <= jit;
            Clr <= true;
            rst_n <= '0'; wait for 50 ns; rst_n <= '1';
            Clr <= false;
            Run <= true;
            wait until Stat(1) = '1' for 400 us;
            if Stat(1) /= '1' then
                report name & ": not locked" severity note; Fail <= true;
            end if;
            wait for 2 ms;
            sp := MkLatMax - MkLatMin;
            report name & ": stream " & std_logic'image(Stat(0)) &
                   "  window A/B not clean " & std_logic'image(Stat(2)) & std_logic'image(Stat(3)) &
                   "  repeated " & std_logic'image(Stat(4)) &
                   "  edge changes A " & integer'image(conv_integer(EdgeJump(7 downto 0))) &
                   " B " & integer'image(conv_integer(EdgeJump(15 downto 8))) &
                   "  marker changes " & integer'image(conv_integer(Stat(15 downto 8))) &
                   "  markers " & integer'image(MkCnt) &
                   "  marker after punch edge " & time'image(MkLatMin) & " .. " & time'image(MkLatMax)
                   severity note;
            if MkCnt < 100 then report name & ": too few markers" severity note; Fail <= true; end if;
            if sp > 1 ns then report name & ": the decoded marker jumps" severity note; Fail <= true; end if;
            if Stat(15 downto 8) /= 0 then report name & ": marker changes counted" severity note; Fail <= true; end if;
            if expSel = 'A' and Stat(0) /= '0' then report name & ": expected stream A" severity note; Fail <= true; end if;
            if expSel = 'B' and Stat(0) /= '1' then report name & ": expected stream B" severity note; Fail <= true; end if;
            if expClean and EdgeJump /= 0 then report name & ": edge changes counted" severity note; Fail <= true; end if;
            if expSel = 'B' and EdgeJump(7 downto 0) = 0 then
                report name & ": stream A should have shown changes" severity note; Fail <= true;
            end if;
            if expSel = 'B' and EdgeJump(15 downto 8) /= 0 then
                report name & ": stream B should be clean" severity note; Fail <= true;
            end if;
            Run <= false;
            wait for 500 ns;
        end procedure;
    begin
        -- sample points: stream A at 0 ns, stream B at 3.125 ns (modulo 6.25 ns)
        one("edge between A and B, no jitter      ", 1.5 ns,   0 ns,   'A', true);
        one("edge between B and A, no jitter      ", 4.7 ns,   0 ns,   'A', true);
        one("edge between A and B, jitter 0.5 ns  ", 1.5 ns,   0.5 ns, 'A', true);
        one("edge between B and A, jitter 0.5 ns  ", 4.7 ns,   0.5 ns, 'A', true);
        one("edge on the A sample, jitter 0.3 ns  ", 6.25 ns,  0.3 ns, 'B', false);
        one("edge on the A sample, jitter 1.0 ns  ", 6.25 ns,  1.0 ns, 'B', false);
        one("edge on the B sample, jitter 0.3 ns  ", 3.125 ns, 0.3 ns, 'A', false);
        one("edge on the B sample, jitter 1.0 ns  ", 3.125 ns, 1.0 ns, 'A', false);
        one("edge 0.2 ns before A, jitter 0.3 ns  ", 6.05 ns,  0.3 ns, 'B', false);
        one("edge 0.2 ns after A, jitter 0.3 ns   ", 0.2 ns,   0.3 ns, 'B', false);

        -- restart by the uC: same result again
        Offset <= 6.25 ns; Jitter <= 0.3 ns;
        rst_n <= '0'; wait for 50 ns; rst_n <= '1';
        Run <= true;
        wait until Stat(1) = '1' for 400 us;
        selFirst := Stat(0);
        Restart <= '1'; wait for 10 ns; Restart <= '0';
        wait for 100 ns;
        if Stat(1) /= '0' then report "restart: still locked" severity note; Fail <= true; end if;
        wait until Stat(1) = '1' for 400 us;
        if Stat(1) /= '1' or Stat(0) /= selFirst or Stat(0) /= '1' then
            report "restart: not the same stream again" severity note; Fail <= true;
        end if;
        Run <= false;
        wait for 500 ns;

        if Fail then report "MarkerSample_tb: FAILED" severity note;
        else         report "MarkerSample_tb: ALL CHECKS PASSED" severity note;
        end if;
        Done <= true;
        wait;
    end process;

end architecture sim;

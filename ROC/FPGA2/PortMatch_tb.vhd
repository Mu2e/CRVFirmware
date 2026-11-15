-- Test bench for the port match of FPGA 2 (PortMatch.vhd). Runs the complete
-- top level with behavioural models of the Xilinx cores (sim/core_models.vhd):
-- requests are loaded through the uC bus, FEB answers come in on the PHY
-- receive pins, the words written to the link FIFO (to FPGA 1) are checked.
--
--   MODE = "reg"   only complete, matching answers. The printed events must be
--                  the same for the firmware before the change and for the
--                  new one with the port match off (PMEN = 0) and on (1).
--   MODE = "reg2"  one FEB misses a request. Same events before the change
--                  and with the port match off.
--   MODE = "full"  the cases of the port match.
-- Run with sim/run_ghdl.sh.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

use work.sim_pkg.all;

entity PortMatch_tb is
    generic (MODE : string := "full"; PMEN : integer := 1);
end entity;

architecture sim of PortMatch_tb is

    constant ACTIVE : std_logic_vector(7 downto 0) := X"27"; -- ports 0, 1, 2, 5
    constant GA     : std_logic_vector(1 downto 0) := "01";

    type nib_arr is array (0 to 7) of std_logic_vector(3 downto 0);
    type int_arr is array (0 to 7) of integer;
    type w32_arr is array (0 to 7) of std_logic_vector(31 downto 0);
    type w16_arr is array (0 to 7) of std_logic_vector(15 downto 0);

    signal CpldRst : std_logic := '0';
    signal CpldCS, uCRd, uCWr : std_logic := '1';
    signal uCA  : std_logic_vector(11 downto 0) := (others => '0');
    signal uCD  : std_logic_vector(15 downto 0) := (others => 'Z');
    signal SDD  : std_logic_vector(15 downto 0);
    signal UDQS, LDQS, SDRzq : std_logic;
    signal MDIO : std_logic_vector(1 downto 0);
    signal rxd  : nib_arr := (others => X"0");
    signal RxClk : std_logic_vector(7 downto 0) := X"00";
    signal RxDV, CRS : std_logic_vector(7 downto 0) := X"00";
    signal FMRx : std_logic_vector(7 downto 0) := X"00";
    signal fmclk, rxc : std_logic := '0';

    -- FEB answers: one process per port, started by a change of ans_go(i)
    signal ans_go   : std_logic_vector(7 downto 0) := X"00";
    signal ans_ewt  : w32_arr := (others => (others => '0'));
    signal ans_stat : w16_arr := (others => (others => '0'));
    signal ans_wc   : int_arr := (others => 4); -- word count in the header
    signal ans_send : int_arr := (others => 4); -- words that are sent
    signal ans_busy : std_logic_vector(7 downto 0) := X"00";

    -- events seen on the link
    signal ev_cnt  : integer := 0;
    signal ev_ewt  : std_logic_vector(31 downto 0) := (others => '0');
    signal ev_stat : std_logic_vector(15 downto 0) := (others => '0');
    signal ev_n    : integer := 0;

    signal errors : integer := 0;

    function hex(v : std_logic_vector) return string is
        constant c : string(1 to 16) := "0123456789ABCDEF";
        variable r : string(1 to v'length/4);
        variable x : std_logic_vector(v'length-1 downto 0) := v;
    begin
        for i in r'range loop
            r(i) := c(to_integer(unsigned(x(x'left-4*(i-1) downto x'left-4*(i-1)-3))) + 1);
        end loop;
        return r;
    end function;

begin

    fmclk <= not fmclk after 20 ns; -- FM activity on the active ports
    rxc   <= not rxc   after 20 ns; -- 25 MHz PHY receive clock
    FMRx  <= ACTIVE when fmclk = '1' else X"00";
    RxClk <= (others => rxc);

    dut : entity work.Controller_FPGA2
      port map (
        VXO_P => '0', VXO_N => '1', ClkB_P => '0', ClkB_N => '1', Clk50MHz => '0',
        CpldRst => CpldRst, CpldCS => CpldCS, uCRd => uCRd, uCWr => uCWr,
        uCA => uCA, uCD => uCD, GA => GA,
        SDCKE => open, LDM => open, UDM => open, RAS => open, CAS => open, SDWE => open,
        SDClk_P => open, SDClk_N => open, SDD => SDD, UDQS => UDQS, LDQS => LDQS, SDRzq => SDRzq,
        SDA => open, BA => open,
        LinkClk_P => open, LinkClk_N => open, LinkFR_P => open, LinkFR_N => open,
        LinkD_P => open, LinkD_N => open,
        RxDA => rxd(0), RxDB => rxd(1), RxDC => rxd(2), RxDD => rxd(3),
        RxDE => rxd(4), RxDF => rxd(5), RxDG => rxd(6), RxDH => rxd(7),
        RxClk => RxClk, RxDV => RxDV, RxErr => X"00", CRS => CRS,
        TxDA => open, TxDB => open, TxDC => open, TxDD => open,
        TxDE => open, TxDF => open, TxDG => open, TxDH => open,
        TxEn => open, MDC => open, MDIO => MDIO, PhyPDn => open, PhyRst => open,
        TxClk => "00", Clk25MHz => open, FMRx => FMRx, FMRxEn => open,
        HrtBtFM => '0', DReqFM => '1',
        SPICS => open, SPISClk => open, SPIMOSI => open, SPIMISO => '0', Debug => open);

    -- FEB answer on one port: 4 words preamble, the answer, 2 words CRC (not checked here)
    gen_feb : for i in 0 to 7 generate
        process
            variable w : std_logic_vector(15 downto 0);
            procedure word(v : std_logic_vector(15 downto 0)) is
            begin
                for n in 0 to 3 loop
                    wait until falling_edge(rxc);
                    rxd(i) <= v(4*n+3 downto 4*n);
                    RxDV(i) <= '1'; CRS(i) <= '1';
                end loop;
            end procedure;
        begin
            wait on ans_go(i);
            ans_busy(i) <= '1';
            word(X"5555"); word(X"5555"); word(X"5555"); word(X"D555");
            for k in 0 to ans_send(i)-1 loop
                case k is
                    when 0 => w := std_logic_vector(to_unsigned(ans_wc(i), 16));
                    when 1 => w := ans_ewt(i)(31 downto 16);
                    when 2 => w := ans_ewt(i)(15 downto 0);
                    when 3 => w := ans_stat(i);
                    when others => w := std_logic_vector(to_unsigned(i, 4)) & ans_ewt(i)(3 downto 0)
                                        & std_logic_vector(to_unsigned(k mod 256, 8));
                end case;
                word(w);
            end loop;
            word(X"C0C0"); word(X"C1C1");
            wait until falling_edge(rxc);
            RxDV(i) <= '0'; CRS(i) <= '0'; rxd(i) <= X"0";
            ans_busy(i) <= '0';
        end process;
    end generate;

    -- events on the link: word count, status, EWT high, EWT low, data
    process
        variable l : line;
        variable n, k : integer := 0;
        variable w : std_logic_vector(15 downto 0);
        variable st : std_logic_vector(15 downto 0);
        variable ew : std_logic_vector(31 downto 0);
    begin
        wait until rising_edge(link_clk);
        if link_wr = '1' then
            w := link_dat(16 downto 9) & link_dat(7 downto 0);
            if link_dat(17) = '0' then
                null; -- status word of the link, not event data
            elsif k = 0 then
                n := to_integer(unsigned(w)); k := 1;
                write(l, string'("EV n=")); write(l, n);
            else
                if    k = 1 then st := w; write(l, string'(" stat=") & hex(w));
                elsif k = 2 then ew(31 downto 16) := w;
                elsif k = 3 then ew(15 downto 0) := w; write(l, string'(" ewt=") & hex(ew) & " :");
                else  write(l, string'(" ") & hex(w));
                end if;
                k := k + 1;
            end if;
            if k /= 0 and k >= n then
                writeline(output, l);
                ev_ewt <= ew; ev_stat <= st; ev_n <= n; ev_cnt <= ev_cnt + 1;
                k := 0;
            end if;
        end if;
    end process;

    stim : process
        variable rd : std_logic_vector(15 downto 0);
        variable seen : integer := 0;

        procedure fail(msg : string) is
        begin
            report "FAIL: " & msg severity note;
            errors <= errors + 1;
            wait for 0 ns;
        end procedure;

        procedure uc_write(a : std_logic_vector(11 downto 0); d : std_logic_vector(15 downto 0)) is
        begin
            uCA <= a; uCD <= d; CpldCS <= '0'; uCWr <= '0';
            wait for 60 ns;
            CpldCS <= '1'; uCWr <= '1';
            wait for 20 ns;
            uCD <= (others => 'Z');
            wait for 40 ns;
        end procedure;

        procedure uc_read(a : std_logic_vector(11 downto 0); d : out std_logic_vector(15 downto 0)) is
        begin
            uCA <= a; CpldCS <= '0'; uCRd <= '0';
            wait for 50 ns;
            d := uCD;
            CpldCS <= '1'; uCRd <= '1';
            wait for 50 ns;
        end procedure;

        procedure reg(a : integer; d : integer) is -- register of this data FPGA
        begin
            uc_write(GA & std_logic_vector(to_unsigned(a, 10)), std_logic_vector(to_unsigned(d, 16)));
        end procedure;

        procedure check_reg(a : integer; exp : integer; name : string) is
            variable v : std_logic_vector(15 downto 0);
        begin
            uc_read(GA & std_logic_vector(to_unsigned(a, 10)), v);
            if to_integer(unsigned(v)) /= exp then
                fail(name & " reads " & hex(v) & ", expected " & integer'image(exp));
            end if;
        end procedure;

        -- request packet as the uC loads it: broadcast data 0x301, send 0x302
        procedure request(e : integer) is
            variable v : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(e, 32));
        begin
            uc_write(X"301", X"0182");
            uc_write(X"301", v(15 downto 0));
            uc_write(X"301", v(31 downto 16));
            uc_write(X"302", X"0001");
        end procedure;

        procedure request2(e1, e2 : integer) is
            variable v1 : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(e1, 32));
            variable v2 : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(e2, 32));
        begin
            uc_write(X"301", X"0282");
            uc_write(X"301", v1(15 downto 0)); uc_write(X"301", v1(31 downto 16));
            uc_write(X"301", v2(15 downto 0)); uc_write(X"301", v2(31 downto 16));
            uc_write(X"302", X"0001");
        end procedure;

        -- answer of port p: nd data words; wc < 0: correct word count; send < 0: all words
        procedure answer(p : integer; e : integer; nd : integer; stat : integer := 0;
                         wc : integer := -1; send : integer := -1) is
        begin
            if ans_busy(p) = '1' then wait until ans_busy(p) = '0'; end if;
            ans_ewt(p)  <= std_logic_vector(to_unsigned(e, 32));
            ans_stat(p) <= std_logic_vector(to_unsigned(stat, 16));
            if wc < 0 then ans_wc(p) <= nd + 4; else ans_wc(p) <= wc; end if;
            if send < 0 then ans_send(p) <= nd + 4; else ans_send(p) <= send; end if;
            ans_go(p) <= not ans_go(p);
            wait for 1 ns;
        end procedure;

        procedure answer_all(e : integer; nd : integer) is
        begin
            for p in 0 to 7 loop
                if ACTIVE(p) = '1' then answer(p, e, nd + p); end if;
            end loop;
        end procedure;

        procedure expect(e : integer; stat : integer; n : integer; tmo : time := 400 us) is
        begin
            if ev_cnt = seen then wait until ev_cnt /= seen for tmo; end if;
            if ev_cnt = seen then
                fail("no event, expected EWT " & integer'image(e));
            else
                seen := seen + 1;
                if seen /= ev_cnt then fail("more events than expected"); seen := ev_cnt; end if;
                if to_integer(unsigned(ev_ewt)) /= e or to_integer(unsigned(ev_stat)) /= stat or ev_n /= n then
                    fail("event EWT " & hex(ev_ewt) & " status " & hex(ev_stat) & " words " & integer'image(ev_n)
                         & ", expected EWT " & integer'image(e) & " status " & integer'image(stat)
                         & " words " & integer'image(n));
                end if;
            end if;
        end procedure;

        procedure expect_none(t : time) is
        begin
            wait for t;
            if ev_cnt /= seen then fail("event that was not expected, EWT " & hex(ev_ewt)); seen := ev_cnt; end if;
        end procedure;

        procedure title(s : string) is
        begin
            report "---- " & s severity note;
        end procedure;

        -- data words of an event with all four active ports: nd+0, nd+1, nd+2, nd+5
        constant SUM4 : integer := 8;
    begin
        MDIO <= "ZZ"; SDD <= (others => 'Z'); UDQS <= 'Z'; LDQS <= 'Z'; SDRzq <= 'Z';
        wait for 200 ns;
        CpldRst <= '1';
        wait for 5 us;
        reg(16#00#, 16#0009#);            -- reset the port buffers, FM receivers on
        wait for 5 us;
        reg(16#00#, 16#01A8#);            -- init, DDR write and read on
        wait for 20 us;
        if PMEN = 1 then reg(16#81#, 1); end if;

        if MODE = "reg" then
            for k in 0 to 11 loop
                request(16#12340000# + k);
                wait for 3 us;
                for p in 0 to 7 loop
                    if ACTIVE(p) = '1' then
                        if k = 5 and p = 2 then
                            answer(p, 16#12340000# + k, 700);
                        else
                            answer(p, 16#12340000# + k, ((k * 7 + p * 3) mod 23) * (k mod 3), (k mod 4) * 16#0101# * (p mod 2));
                        end if;
                    end if;
                end loop;
                if k mod 4 = 3 then wait for 2 us; elsif k = 5 then wait for 400 us; else wait for 150 us; end if;
            end loop;
            wait for 600 us;

        elsif MODE = "reg2" then
            for k in 0 to 7 loop
                request(16#00050000# + k);
                wait for 3 us;
                for p in 0 to 7 loop
                    if ACTIVE(p) = '1' and not (k = 3 and p = 1) then
                        answer(p, 16#00050000# + k, k + p);
                    end if;
                end loop;
                wait for 150 us;
            end loop;
            wait for 300 us;

        else
            title("A  all ports answer");
            request(100); answer_all(100, 2);
            expect(100, 0, 4 + 4*2 + SUM4);
            check_reg(16#85#, 1, "events built");

            title("B  port 1 misses a request, no time out");
            request(101);
            answer(0, 101, 1); answer(2, 101, 1); answer(5, 101, 1);
            expect_none(300 us);
            request(102); answer_all(102, 3);
            expect(101, 16#2202#, 4 + 3);            -- port 1 missing: port bit 1, bits 9 and 13
            expect(102, 0, 4 + 4*3 + SUM4);
            check_reg(16#88#, 1, "events with missing ports");
            check_reg(16#87#, 0, "port buffers emptied");

            title("C  answer with an EWT that was not requested");
            request(103);
            answer(0, 103, 2); answer(1, 103, 2); answer(2, 16#7EAD0103#, 2); answer(5, 103, 2);
            expect_none(300 us);
            check_reg(16#87#, 1, "port buffers emptied");
            check_reg(16#8C#, 16#7EAD#, "EWT not in the list, high");
            check_reg(16#8D#, 16#0103#, "EWT not in the list, low");
            request(104); answer_all(104, 0);
            expect(103, 16#2204#, 4 + 3*2);
            expect(104, 0, 4 + SUM4);

            title("D  time out: port 0 does not answer, its answer comes late");
            reg(16#82#, 5);                          -- 50 us
            request(105);
            answer(1, 105, 1); answer(2, 105, 1); answer(5, 105, 1);
            expect(105, 16#2201#, 4 + 3*1, 200 us);
            check_reg(16#89#, 1, "events built by the time out");
            answer(0, 105, 6);
            expect_none(100 us);
            check_reg(16#87#, 2, "port buffers emptied");
            request(106); answer_all(106, 1);
            expect(106, 0, 4 + 4*1 + SUM4);

            title("E  nobody answers");
            request(107);
            expect(107, 16#2227#, 4, 200 us);
            request(108); answer_all(108, 1);
            expect(108, 0, 4 + 4*1 + SUM4);

            title("F  answer that does not get complete");
            request(109);
            answer(0, 109, 1); answer(1, 109, 1); answer(5, 109, 1);
            answer(2, 109, 30, 0, -1, 12);           -- header says 34 words, 12 are sent
            expect(109, 16#2204#, 4 + 3*1, 200 us);
            request(110); answer_all(110, 2);
            expect(110, 0, 4 + 4*2 + SUM4);

            title("G  data without a request");
            answer(1, 500, 3);
            expect_none(100 us);
            request(111); answer_all(111, 1);
            expect(111, 0, 4 + 4*1 + SUM4);

            title("H  two requests in one packet, answers in order");
            reg(16#82#, 0);
            request2(112, 113);
            answer_all(112, 1); answer_all(113, 2);
            expect(112, 0, 4 + 4*1 + SUM4);
            expect(113, 0, 4 + 4*2 + SUM4);

            title("I  two open requests, port 5 answers only the second");
            request2(114, 115);
            answer(0, 114, 1); answer(1, 114, 1); answer(2, 114, 1);
            answer(0, 115, 1); answer(1, 115, 1); answer(2, 115, 1); answer(5, 115, 1);
            expect(114, 16#2220#, 4 + 3*1);
            expect(115, 0, 4 + 4*1);

            title("J  five requests without answer: the oldest is dropped");
            for k in 0 to 4 loop request(120 + k); end loop;
            expect_none(50 us);
            check_reg(16#8A#, 1, "requests dropped from the list");
            answer_all(120, 1);                      -- not in the list any more
            expect_none(100 us);
            reg(16#82#, 5);                          -- the rest is released by the time out
            expect(121, 16#2227#, 4, 200 us);
            expect(122, 16#2227#, 4, 200 us);
            expect(123, 16#2227#, 4, 200 us);
            expect(124, 16#2227#, 4, 200 us);
            reg(16#82#, 0);

            title("K  port match off");
            reg(16#81#, 0);
            request(130); answer_all(130, 2);
            expect(130, 0, 4 + 4*2 + SUM4);
            expect_none(100 us);
        end if;

        wait for 1 ns;
        if MODE = "full" then
            if errors = 0 then report "ALL CHECKS PASSED" severity note;
            else report "CHECKS FAILED: " & integer'image(errors) severity note;
            end if;
        end if;
        assert false report "END OF SIMULATION" severity failure;
        wait;
    end process;

end architecture;

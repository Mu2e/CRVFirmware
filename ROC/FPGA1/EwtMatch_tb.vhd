--------------------------------------------------------------------------------
-- Test bench for the data request path of FPGA1:
--   link FIFOs -> EventBuilder -> EventBuff / EvUbBuff -> DRHandler -> PacketFormer
-- wired as in Controller_FPGA1.vhd.
--
-- Checks that an event is only sent with the event window tag (EWT) it was
-- requested with, and that nothing is sent for requests without event and
-- for late events. Self checking, ends with "EwtMatch_tb: ALL CHECKS PASSED"
-- or with a failure.
--
-- MODE = "full"    : all cases (new firmware only)
-- MODE = "regress" : normal events, DCS packets and GR events only. With
--                    STREAM_FILE set, everything that is transmitted is
--                    written to that file, to compare against the firmware
--                    before the change (see sim/run_ghdl.sh).
-- MODE = "skip"    : one request without event, no checks, log only. Shows
--                    what the firmware before the change does.
--
-- With ISim the real FIFO cores are used, with GHDL the models in
-- sim/fifo_models.vhd.
--------------------------------------------------------------------------------
LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

use work.Project_Defs.all;
use work.Project_Components.all;

ENTITY EwtMatch_tb IS
    generic (MODE        : string  := "full";
             LEGACY      : boolean := false; -- firmware before the change behind wrappers
             VERBOSE     : boolean := false; -- report the words written to / read from EventBuff per event
             STREAM_FILE : string  := "");
END EwtMatch_tb;

ARCHITECTURE behavior OF EwtMatch_tb IS

    constant clk_period : time := 6.4 ns;

    signal clk   : std_logic := '0';
    signal reset : std_logic := '1';
    signal done  : boolean := false;

    -- link FIFOs
    signal lf_din        : Array_3x16 := (others => (others => '0'));
    signal lf_wr         : std_logic_vector(2 downto 0) := "000";
    signal LinkFIFOOut   : Array_3x16;
    signal LinkFIFORdCnt : Array_3x14;
    signal LinkFIFOEmpty, LinkFIFORdReq : std_logic_vector(2 downto 0);
    signal LinkFIFORst, lf_rst : std_logic;

    -- flush of the link FIFOs
    signal DReqRx : std_logic := '0';
    signal LinkFlushEn, TagGuardEn : std_logic := '1';
    signal FlushReq, FlushAck, FlushNow, EvTruncated, BadTag : std_logic;
    signal LinkFlushCnt, BadTagCnt, StaleCnt, TruncCnt, SnapCause : std_logic_vector(15 downto 0);
    signal SnapHead : Array_3x16;
    signal SnapWds  : Array_3x14;

    -- event buffer and EWT side buffer
    signal EventBuff_Dat, EventBuff_Out : std_logic_vector(15 downto 0);
    signal EventBuff_WrtEn, EventBuff_RdEn, EventBuff_Empty : std_logic;
    signal EvUb_Dat, EvUbBuff_Out : std_logic_vector(15 downto 0);
    signal EvUb_WrEn, EvUbBuff_rd_en, EvUbBuff_Empty : std_logic;
    signal EvBuildState : std_logic_vector(3 downto 0);

    -- time stamp buffer
    signal ts_din : std_logic_vector(15 downto 0) := (others => '0');
    signal ts_wr  : std_logic := '0';
    signal TStmpBuff_Out : std_logic_vector(15 downto 0);
    signal TStmpBuff_rd_en, DR_TStmpBuff_rd_en, ts_rd : std_logic;
    signal TStmpWds : std_logic_vector(8 downto 0);

    -- DCS reply buffer
    signal dcs_din : std_logic_vector(15 downto 0) := (others => '0');
    signal dcs_wr  : std_logic := '0';
    signal DCSBuff_Out : std_logic_vector(15 downto 0);
    signal DCSBuff_rd_en : std_logic;
    signal dcs_cnt : std_logic_vector(8 downto 0);
    signal DCSBuffRdCnt : std_logic_vector(12 downto 0);

    -- handler <-> former
    signal pktFormerSend, pktFormerTimeout, pktFormerDiscard : std_logic;
    signal DRdone, DiscardDone, pktFormerIdle : std_logic;
    signal DReq_Count : std_logic_vector(31 downto 0);
    signal FormStat : std_logic_vector(2 downto 0);
    signal DRHandlerState : std_logic_vector(3 downto 0);
    signal EwtDropCnt, EvDropCnt, LastDropEvUb, LastDropReqEwt, DRcnt : std_logic_vector(15 downto 0);

    -- transmit side
    signal TxCRCEn, TxCRCRst : std_logic;
    signal TxCRCDat, TxCRC, GTPTx : std_logic_vector(15 downto 0);
    signal TxCharIsK : std_logic_vector(1 downto 0);

    -- settings
    signal sendGR, MatchBypassReg, MatchBypass, DRHold : std_logic := '0';
    signal DropLateEn : std_logic := '1';
    signal DRTimeout  : std_logic_vector(15 downto 0) := X"FFFF";
    signal MarkerDelayed : std_logic_vector(3 downto 0) := "0000";
    constant ActiveReg : std_logic_vector(23 downto 0) := X"010101"; -- one FEB on each data FPGA

    -- what the monitor saw on the transmit side, per data header
    constant NHIST : natural := 128;
    type slv48_arr is array (0 to NHIST-1) of std_logic_vector(47 downto 0);
    type slv32_arr is array (0 to NHIST-1) of std_logic_vector(31 downto 0);
    type int_arr   is array (0 to NHIST-1) of integer;
    signal h_ewt  : slv48_arr := (others => (others => '0')); -- EWT in the data header
    signal h_ub   : slv32_arr := (others => (others => '0')); -- uB in the controller header
    signal h_pk   : int_arr := (others => -1); -- packet count in the data header
    signal h_stat : int_arr := (others => -1); -- status word of the data header
    signal h_cnt  : int_arr := (others => -1); -- word count in the controller header
    signal h_dreq : int_arr := (others => -1); -- DReq_Count in the controller header
    signal h_dpk  : int_arr := (others => 0);  -- data packets seen after the header
    signal h_cst  : slv32_arr := (others => (others => '0')); -- status in the controller header
    signal n_hdr  : integer := 0;
    signal n_dcs  : integer := 0;

    function hex(v : std_logic_vector) return string is
        constant c : string(1 to 16) := "0123456789abcdef";
        variable x : std_logic_vector(v'length-1 downto 0) := v;
        variable s : string(1 to v'length/4);
    begin
        for i in s'range loop
            s(i) := c(to_integer(unsigned(x(x'length-4*(i-1)-1 downto x'length-4*i))) + 1);
        end loop;
        return s;
    end function;

BEGIN

    clk <= not clk after clk_period/2 when not done else '0';

    ---------------------------------------------------------------------------
    -- design, wired as in Controller_FPGA1.vhd
    ---------------------------------------------------------------------------
    lf_rst <= reset or LinkFIFORst;
    GenLinks : for i in 0 to 2 generate
        LinkBuff : LinkFIFO
            port map (rst => lf_rst, wr_clk => clk, rd_clk => clk,
                      wr_en => lf_wr(i), rd_en => LinkFIFORdReq(i),
                      din => lf_din(i), dout => LinkFIFOOut(i),
                      full => open, empty => LinkFIFOEmpty(i),
                      rd_data_count => LinkFIFORdCnt(i));
    end generate;

    EventBuilderInst : EventBuilder
        port map (clk => clk, reset => reset, FormRst => '0',
                  LinkFIFOOut => LinkFIFOOut, LinkFIFORdCnt => LinkFIFORdCnt,
                  LinkFIFOEmpty => LinkFIFOEmpty, LinkFIFORdReq => LinkFIFORdReq,
                  LinkFIFORst => LinkFIFORst,
                  FlushReq => FlushReq, FlushAck => FlushAck, FlushNow => FlushNow,
                  Truncated => EvTruncated,
                  EventBuff_Dat => EventBuff_Dat, EventBuff_WrtEn => EventBuff_WrtEn,
                  TStmpWds => TStmpWds, ActiveReg => ActiveReg,
                  MarkerDelayed => MarkerDelayed,
                  LinkRDDL => "00", AddrReg => X"000",
                  FormHold => '0', sendGR => sendGR, uBinHeader => '1', uBwrt => '1',
                  GA => "00", sendGrCnt => X"00",
                  ExtuBunchCount => X"000000000000", HeartBtCnt => X"1111",
                  HeartBeatCnt => X"2222", LastWindow => X"3333", Stats => X"4444",
                  InjectionTs => X"5555", InjectionWindow => X"6666",
                  FakeNum => open,
                  EvUb_Dat => EvUb_Dat, EvUb_WrEn => EvUb_WrEn, EvBuildState => EvBuildState,
                  uBDebugOut => open);

    EventBuff : FIFO_SC_4Kx16
        port map (clk => clk, rst => reset, wr_en => EventBuff_WrtEn, rd_en => EventBuff_RdEn,
                  din => EventBuff_Dat, dout => EventBuff_Out, full => open, empty => EventBuff_Empty);

    EvUbBuff : TrigPktBuff
        port map (rst => reset, clk => clk, din => EvUb_Dat, wr_en => EvUb_WrEn,
                  rd_en => EvUbBuff_rd_en, dout => EvUbBuff_Out, full => open,
                  empty => EvUbBuff_Empty, data_count => open);

    ts_rd <= TStmpBuff_rd_en or DR_TStmpBuff_rd_en;
    TimeStampBuff : TrigPktBuff
        port map (rst => reset, clk => clk, din => ts_din, wr_en => ts_wr,
                  rd_en => ts_rd, dout => TStmpBuff_Out, full => open,
                  empty => open, data_count => TStmpWds);

    DCSBuff : TrigPktBuff
        port map (rst => reset, clk => clk, din => dcs_din, wr_en => dcs_wr,
                  rd_en => DCSBuff_rd_en, dout => DCSBuff_Out, full => open,
                  empty => open, data_count => dcs_cnt);
    DCSBuffRdCnt <= "0000" & dcs_cnt;

    MatchBypass <= MatchBypassReg or sendGR;

    DRHandlerInst : DRHandler
        port map (clk => clk, reset => reset,
                  TStmpWds => TStmpWds, TStmpBuff_Out => TStmpBuff_Out,
                  TStmpBuff_rd_en => DR_TStmpBuff_rd_en,
                  EvUbBuff_Out => EvUbBuff_Out, EvUbBuff_Empty => EvUbBuff_Empty,
                  EvUbBuff_rd_en => EvUbBuff_rd_en,
                  EventBuff_Empty => EventBuff_Empty,
                  pktFormerIdle => pktFormerIdle, DRdone => DRdone, DiscardDone => DiscardDone,
                  pktFormerSend => pktFormerSend, pktFormerTimeout => pktFormerTimeout,
                  pktFormerDiscard => pktFormerDiscard,
                  DRHold => DRHold, DRTimeout => DRTimeout,
                  MatchBypass => MatchBypass, DropLateEn => DropLateEn,
                  TagGuardEn => TagGuardEn, BadTag => BadTag,
                  DRcnt => DRcnt, EwtDropCnt => EwtDropCnt, EvDropCnt => EvDropCnt,
                  LastDropEvUb => LastDropEvUb, LastDropReqEwt => LastDropReqEwt,
                  State => DRHandlerState);

    LinkFlushCtrlInst : LinkFlushCtrl
        port map (clk => clk, reset => reset,
                  DReqRx => DReqRx, BadTag => BadTag,
                  FlushEn => LinkFlushEn, sendGR => sendGR,
                  EvBuildState => EvBuildState,
                  FlushReq => FlushReq, FlushAck => FlushAck, FlushNow => FlushNow,
                  Truncated => EvTruncated, LinkFIFORst => LinkFIFORst,
                  LinkFIFOEmpty => LinkFIFOEmpty, LinkFIFOOut => LinkFIFOOut,
                  LinkFIFORdCnt => LinkFIFORdCnt,
                  TStmpWds => TStmpWds,
                  LinkFlushCnt => LinkFlushCnt, BadTagCnt => BadTagCnt,
                  StaleCnt => StaleCnt, TruncCnt => TruncCnt,
                  SnapCause => SnapCause, SnapHead => SnapHead, SnapWds => SnapWds);

    PacketFormerInst : PacketFormer
        port map (clk => clk, reset => reset,
                  pktFormerSend => pktFormerSend, pktFormerTimeout => pktFormerTimeout,
                  pktFormerDiscard => pktFormerDiscard, DiscardDone => DiscardDone,
                  pktFormerIdle => pktFormerIdle,
                  FormRst => '0',
                  EventBuff_Out => EventBuff_Out, EventBuff_RdEn => EventBuff_RdEn,
                  TStmpBuff_rd_en => TStmpBuff_rd_en, TStmpBuff_Out => TStmpBuff_Out,
                  EventBuff_Empty => EventBuff_Empty, ActiveReg => ActiveReg,
                  DCSBuffRdCnt => DCSBuffRdCnt, DCSBuff_Out => DCSBuff_Out,
                  DCSBuff_rd_en => DCSBuff_rd_en,
                  DRdone => DRdone, DReqCount => DReq_Count, FormStat => FormStat,
                  TxCRCEn => TxCRCEn, TxCRCRst => TxCRCRst, TxCRCDat => TxCRCDat, TxCRC => TxCRC,
                  WdCountBuff_WrtEn => open,
                  GTPTx => GTPTx, TxCharIsK => TxCharIsK,
                  GTPTxBuff_In => open, GTPTxBuff_wr_en => open,
                  MarkerDelayed => '0', loopbackMarker => '0',
                  LoopbackMode => "000", uBwrt => '1', IDReg => X"3");

    TxCRCGen : crc
        port map (data_in => TxCRCDat, crc_en => TxCRCEn, rst => TxCRCRst, clk => clk,
                  crc_out => TxCRC);

    ---------------------------------------------------------------------------
    -- monitor of the transmitted packets
    --   data header   (type 5): [marker, 0, 8x50, packet count, EWT lo, mid, hi, status, 0, CRC]
    --   first data pkt (type 6): [marker, count+8, DReq_Count, active hi, lo,
    --                             status hi, lo, uB word, uB word, CRC]
    ---------------------------------------------------------------------------
    monitor : process(clk)
        variable in_pk : boolean := false;
        variable idx, typ, cur : integer := 0;
        variable first_data : boolean := false;
        variable ewt : std_logic_vector(47 downto 0);
        variable ub  : std_logic_vector(31 downto 0);
        variable l : line;
        file f : text;
        variable f_open : boolean := false;
    begin
        if rising_edge(clk) and reset = '0' then
            if STREAM_FILE'length > 0 and not f_open then
                file_open(f, STREAM_FILE, write_mode); f_open := true;
            end if;
            if f_open and not (TxCharIsK = "11" and GTPTx = X"BC3C") then
                write(l, hex(GTPTx) & " " & hex("00" & TxCharIsK)); writeline(f, l);
            end if;

            if TxCharIsK = "10" and GTPTx(15 downto 8) = X"1C" then
                in_pk := true; idx := 0; typ := to_integer(unsigned(GTPTx(4 downto 0)));
                if typ = 5 then
                    cur := n_hdr; n_hdr <= n_hdr + 1; first_data := true;
                elsif typ = 6 then
                    if cur < NHIST then h_dpk(cur) <= h_dpk(cur) + 1; end if;
                elsif typ = 4 then
                    n_dcs <= n_dcs + 1;
                end if;
            elsif in_pk then
                idx := idx + 1;
                if typ = 5 and cur < NHIST then
                    case idx is
                        when 3 => h_pk(cur) <= to_integer(unsigned(GTPTx(10 downto 0)));
                        when 4 => ewt(15 downto 0)  := GTPTx;
                        when 5 => ewt(31 downto 16) := GTPTx;
                        when 6 => ewt(47 downto 32) := GTPTx; h_ewt(cur) <= ewt;
                        when 7 => h_stat(cur) <= to_integer(unsigned(GTPTx));
                                  report "TX data header #" & integer'image(cur) & "  EWT " & hex(ewt)
                                       & "  packets " & integer'image(h_pk(cur))
                                       & "  status " & hex(GTPTx) severity note;
                        when others => null;
                    end case;
                elsif typ = 6 and first_data and cur < NHIST then
                    case idx is
                        when 1 => h_cnt(cur)  <= to_integer(unsigned(GTPTx));
                        when 2 => h_dreq(cur) <= to_integer(unsigned(GTPTx));
                        when 5 => h_cst(cur)(31 downto 16) <= GTPTx;
                        when 6 => h_cst(cur)(15 downto 0)  <= GTPTx;
                        when 7 => ub(31 downto 16) := GTPTx;
                        when 8 => ub(15 downto 0)  := GTPTx; h_ub(cur) <= ub;
                                  report "TX   event of header #" & integer'image(cur) & "  uB " & hex(ub)
                                       & "  word count " & integer'image(h_cnt(cur))
                                       & "  DReq_Count " & integer'image(h_dreq(cur)) severity note;
                        when others => null;
                    end case;
                end if;
                if idx = 9 then
                    in_pk := false;
                    if typ = 6 then first_data := false; end if;
                end if;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- optional: words into and out of the event buffer, per event
    ---------------------------------------------------------------------------
    verb : process(clk)
        variable wr_n, rd_n, rd_empty_n : integer := 0;
        variable wr_first : std_logic_vector(15 downto 0);
        variable wr_d, rd_d : std_logic := '0';
        variable wr_gap, rd_gap : integer := 0;
        variable rst_d : std_logic := '0';
    begin
        if rising_edge(clk) and VERBOSE and reset = '0' then
            if EventBuff_WrtEn = '1' then
                if wr_n = 0 then wr_first := EventBuff_Dat; end if;
                wr_n := wr_n + 1; wr_gap := 0;
            elsif wr_n > 0 then
                wr_gap := wr_gap + 1;
                if wr_gap = 20 then
                    report "  EventBuff: " & integer'image(wr_n) & " words written, first word "
                         & integer'image(to_integer(unsigned(wr_first))) severity note;
                    wr_n := 0;
                end if;
            end if;
            if EventBuff_RdEn = '1' then
                rd_n := rd_n + 1; rd_gap := 0;
                if EventBuff_Empty = '1' then rd_empty_n := rd_empty_n + 1; end if;
            elsif rd_n > 0 then
                rd_gap := rd_gap + 1;
                if rd_gap = 40 then
                    report "  EventBuff: " & integer'image(rd_n) & " words read, "
                         & integer'image(rd_empty_n) & " of them while empty" severity note;
                    rd_n := 0; rd_empty_n := 0;
                end if;
            end if;
            if BadTag = '1' then
                report "  BadTag: event tag " & hex(LastDropEvUb) & " request " & hex(LastDropReqEwt) severity note;
            end if;
            if FlushNow = '1' then report "  FlushNow (stale data at a request)" severity note; end if;
            if FlushReq = '1' and FlushAck = '1' then report "  FlushReq acknowledged, builder " & hex(EvBuildState) severity note; end if;
            if LinkFIFORst = '1' and rst_d = '0' then
                report "  link FIFOs reset, heads " & hex(LinkFIFOOut(0)) & " " & hex(LinkFIFOOut(1)) & " " & hex(LinkFIFOOut(2))
                     & " words " & integer'image(to_integer(unsigned(LinkFIFORdCnt(0)))) & "/"
                     & integer'image(to_integer(unsigned(LinkFIFORdCnt(1)))) & "/"
                     & integer'image(to_integer(unsigned(LinkFIFORdCnt(2)))) severity note;
            end if;
            rst_d := LinkFIFORst;
            if MODE = "trace" and (EvBuildState /= X"0" or EventBuff_WrtEn = '1' or LinkFIFORdReq /= "000") then
                report "  builder " & hex(EvBuildState) & "  RdReq " & hex('0' & LinkFIFORdReq)
                     & "  link out " & hex(LinkFIFOOut(0)) & " " & hex(LinkFIFOOut(1)) & " " & hex(LinkFIFOOut(2))
                     & "  empty " & hex('0' & LinkFIFOEmpty)
                     & "  wr " & std_logic'image(EventBuff_WrtEn) & " " & hex(EventBuff_Dat) severity note;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- stimulus and checks
    ---------------------------------------------------------------------------
    stim : process
        variable errors  : integer := 0;
        variable exp_hdr : integer := 0;  -- data headers expected so far
        variable exp_dreq : integer := 0; -- DReq_Count expected in the next event
        variable exp_ewt_drop, exp_ev_drop : integer := 0;
        variable e : unsigned(47 downto 0);
        variable hdr0, stale0 : integer;

        procedure tick(n : natural := 1) is
        begin
            for i in 1 to n loop wait until rising_edge(clk); end loop;
        end procedure;

        procedure check(ok : boolean; msg : string) is
        begin
            if not ok then
                errors := errors + 1;
                report "CHECK FAILED: " & msg severity error;
            end if;
        end procedure;

        -- data request: three EWT words into the time stamp buffer, low word first
        procedure request(ewt : unsigned(47 downto 0)) is
        begin
            report "request  EWT " & hex(std_logic_vector(ewt)) severity note;
            -- the request is seen on the fibre a few clocks before its EWT is stored
            DReqRx <= '1'; tick; DReqRx <= '0'; tick(3);
            for i in 0 to 2 loop
                ts_din <= std_logic_vector(ewt(16*i+15 downto 16*i)); ts_wr <= '1';
                tick;
            end loop;
            ts_wr <= '0';
        end procedure;

        -- FEB data of one event on the three links, n payload words each:
        -- [n+4, status, EWT(31..16), EWT(15..0), payload ...]
        procedure feb_event(ewt : unsigned(47 downto 0); n0, n1, n2 : natural) is
            type nat3 is array (0 to 2) of natural;
            constant n : nat3 := (n0, n1, n2);
            variable nmax : natural := n0;
        begin
            report "FEB data EWT " & hex(std_logic_vector(ewt)) & "  payload words "
                 & integer'image(n0) & "/" & integer'image(n1) & "/" & integer'image(n2) severity note;
            if n1 > nmax then nmax := n1; end if;
            if n2 > nmax then nmax := n2; end if;
            for k in 0 to nmax + 3 loop
                for i in 0 to 2 loop
                    if k < n(i) + 4 then
                        lf_wr(i) <= '1';
                        case k is
                            when 0 => lf_din(i) <= std_logic_vector(to_unsigned(n(i) + 4, 16));
                            when 1 => lf_din(i) <= X"0000";
                            when 2 => lf_din(i) <= std_logic_vector(ewt(31 downto 16));
                            when 3 => lf_din(i) <= std_logic_vector(ewt(15 downto 0));
                            when others =>
                                lf_din(i) <= std_logic_vector(to_unsigned(i + 1, 3))
                                           & std_logic_vector(to_unsigned((k - 4) mod 8192, 13));
                        end case;
                    else
                        lf_wr(i) <= '0';
                    end if;
                end loop;
                tick;
            end loop;
            lf_wr <= "000";
        end procedure;

        -- wait until nothing moves any more
        procedure wait_quiet is
            variable quiet, total : natural := 0;
        begin
            if LEGACY then
                wait for 60 us; tick;
                return;
            end if;
            while quiet < 400 loop
                tick; total := total + 1;
                if pktFormerIdle = '1' and EvUbBuff_Empty = '1' and EvUb_WrEn = '0'
                   and (EvBuildState = X"0" or EvBuildState = X"1")
                   and (DRHandlerState = X"0" or DRHandlerState = X"2")
                then quiet := quiet + 1;
                else quiet := 0;
                end if;
                if total > 400000 then
                    check(false, "design does not become quiet");
                    return;
                end if;
            end loop;
        end procedure;

        -- the next data header must carry this EWT and this event
        procedure check_sent(ewt : unsigned(47 downto 0); payload : natural; stat_hdr : integer := 1) is
            variable i  : integer := exp_hdr;
            variable pk : integer := payload / 8 + 1;
        begin
            if payload mod 8 /= 0 then pk := pk + 1; end if;
            exp_hdr := exp_hdr + 1; exp_dreq := exp_dreq + 1;
            check(n_hdr >= exp_hdr, "event for EWT " & hex(std_logic_vector(ewt)) & " was not sent");
            if n_hdr < exp_hdr then return; end if;
            check(h_ewt(i) = std_logic_vector(ewt),
                  "header #" & integer'image(i) & " EWT " & hex(h_ewt(i)) & ", expected " & hex(std_logic_vector(ewt)));
            check(h_ub(i) = std_logic_vector(ewt(31 downto 0)),
                  "header #" & integer'image(i) & " event uB " & hex(h_ub(i)) & " does not belong to EWT " & hex(std_logic_vector(ewt)));
            check(h_cnt(i) = payload + 8,
                  "header #" & integer'image(i) & " word count " & integer'image(h_cnt(i)) & ", expected " & integer'image(payload + 8));
            check(h_pk(i) = pk,
                  "header #" & integer'image(i) & " packet count " & integer'image(h_pk(i)) & ", expected " & integer'image(pk));
            check(h_dpk(i) = pk,
                  "header #" & integer'image(i) & " announced " & integer'image(pk) & " packets, sent " & integer'image(h_dpk(i)));
            check(h_stat(i) = stat_hdr,
                  "header #" & integer'image(i) & " status " & integer'image(h_stat(i)));
            check(h_dreq(i) = exp_dreq mod 65536,
                  "header #" & integer'image(i) & " DReq_Count " & integer'image(h_dreq(i)) & ", expected " & integer'image(exp_dreq));
        end procedure;

        -- no header beyond the expected ones, drop counters as expected, buffers empty
        procedure check_state(what : string) is
        begin
            check(n_hdr = exp_hdr, what & ": " & integer'image(n_hdr) & " data headers sent, expected " & integer'image(exp_hdr));
            if not LEGACY then
                check(to_integer(unsigned(EwtDropCnt)) = exp_ewt_drop,
                      what & ": EWTs dropped " & integer'image(to_integer(unsigned(EwtDropCnt))) & ", expected " & integer'image(exp_ewt_drop));
                check(to_integer(unsigned(EvDropCnt)) = exp_ev_drop,
                      what & ": events dropped " & integer'image(to_integer(unsigned(EvDropCnt))) & ", expected " & integer'image(exp_ev_drop));
                check(EvUbBuff_Empty = '1', what & ": EvUbBuff not empty");
            end if;
            check(EventBuff_Empty = '1', what & ": EventBuff not empty");
        end procedure;

        procedure normal_event(ewt : unsigned(47 downto 0); n0, n1, n2 : natural) is
        begin
            request(ewt); tick(40);
            feb_event(ewt, n0, n1, n2);
            wait_quiet;
            check_sent(ewt, n0 + n1 + n2);
        end procedure;

        -- n payload words on one link, without header, starting at payload index first
        procedure raw_words(link : natural; first, n : natural) is
        begin
            for k in first to first + n - 1 loop
                lf_din(link) <= std_logic_vector(to_unsigned(link + 1, 3))
                              & std_logic_vector(to_unsigned(k mod 8192, 13));
                lf_wr(link) <= '1';
                tick;
            end loop;
            lf_wr(link) <= '0';
        end procedure;

        -- one complete event part on one link
        procedure feb_part(link : natural; ewt : unsigned(47 downto 0); n : natural) is
        begin
            lf_wr(link) <= '1';
            lf_din(link) <= std_logic_vector(to_unsigned(n + 4, 16)); tick;
            lf_din(link) <= X"0000"; tick;
            lf_din(link) <= std_logic_vector(ewt(31 downto 16)); tick;
            lf_din(link) <= std_logic_vector(ewt(15 downto 0)); tick;
            lf_wr(link) <= '0';
            raw_words(link, 0, n);
        end procedure;

        -- After a fault: normal requests with their events, one after the other.
        -- One of the first three must be sent correctly; then the counters are
        -- taken over and a further event must pass all checks.
        procedure recover(what : string; first : unsigned(47 downto 0)) is
            variable ok : boolean := false;
            variable ewt : unsigned(47 downto 0) := first;
            variable n0 : integer;
        begin
            for k in 1 to 3 loop
                n0 := n_hdr;
                request(ewt); tick(40);
                feb_event(ewt, 8, 8, 8);
                wait_quiet;
                if n_hdr = n0 + 1 and h_ewt(n0) = std_logic_vector(ewt)
                   and h_ub(n0) = std_logic_vector(ewt(31 downto 0)) and h_cnt(n0) = 32 then
                    ok := true;
                    report what & ": running again with request " & integer'image(k) & " after the fault, "
                         & integer'image(to_integer(unsigned(LinkFlushCnt))) & " link FIFO resets so far" severity note;
                    exit;
                end if;
                check(n_hdr = n0, what & ": a wrong event was sent");
                ewt := ewt + 1;
            end loop;
            check(ok, what & ": no event was sent correctly for three requests after the fault");
            exp_hdr := n_hdr; exp_dreq := h_dreq(n_hdr - 1);
            exp_ewt_drop := to_integer(unsigned(EwtDropCnt));
            exp_ev_drop := to_integer(unsigned(EvDropCnt));
            check(LinkFIFOEmpty = "111", what & ": link FIFOs not empty after recovery");
            check_state(what & ", recovered");
            normal_event(ewt + 1, 5, 6, 7);
            check_state(what & ", next event");
        end procedure;

        procedure dcs_reply is
        begin
            dcs_din <= X"1234"; dcs_wr <= '1'; tick;
            dcs_din <= X"5678"; tick;
            dcs_wr <= '0';
        end procedure;

    begin
        reset <= '1'; tick(20); reset <= '0'; tick(20);

        -----------------------------------------------------------------------
        if MODE = "skip" then
            report "=== one request without event (log only) ===" severity note;
            e := X"000000000100";
            request(e); tick(40); feb_event(e, 8, 8, 8); wait_quiet;
            request(e + 1); wait for 40 us; tick;            -- no FEB data
            request(e + 2); tick(40); feb_event(e + 2, 8, 8, 8); wait_quiet;
            request(e + 3); tick(40); feb_event(e + 3, 8, 8, 8); wait_quiet;
            wait for 300 us;
            report "=== end of log, " & integer'image(n_hdr) & " data headers ===" severity note;
            done <= true; wait;
        end if;

        if MODE = "trace" then -- cycle by cycle view of the event builder, use with VERBOSE
            e := X"000000000100";
            request(e); tick(40); feb_event(e, 4, 4, 4); wait_quiet;
            done <= true; wait;
        end if;

        -----------------------------------------------------------------------
        report "=== 1: normal events, EWT low word wraps ===" severity note;
        e := X"00000001FFFD";
        normal_event(e,     8, 3, 5);
        normal_event(e + 1, 0, 0, 0);   -- empty event
        normal_event(e + 2, 16, 1, 7);  -- EWT ...1FFFF
        normal_event(e + 3, 11, 22, 33);-- EWT ...20000
        normal_event(e + 4, 100, 0, 61);
        check_state("normal");

        report "=== 8: DCS packets ===" severity note;
        dcs_reply; wait_quiet;
        check(n_dcs = 1, "DCS packet not sent");
        -- DCS reply just before the event is complete
        request(e + 5); tick(40);
        dcs_reply;
        feb_event(e + 5, 4, 4, 4);
        wait_quiet;
        check_sent(e + 5, 12);
        check(n_dcs = 2, "second DCS packet not sent");
        check_state("DCS");

        if MODE = "regress" then
            report "=== 8: GR events (sendGR = 1) ===" severity note;
            sendGR <= '1'; tick(10);
            MarkerDelayed(0) <= '1'; tick; MarkerDelayed(0) <= '0'; tick(100);
            request(e + 6);
            wait for 60 us; tick;
            sendGR <= '0';
            report "=== regress run finished ===" severity note;
        else
            -------------------------------------------------------------------
            report "=== 2: request without event, across the wrap of the low word ===" severity note;
            e := X"00000002FFFE";
            normal_event(e, 8, 8, 8);
            request(e + 1);                 -- ...2FFFF, the FEBs deliver nothing
            wait for 40 us; tick;
            check_state("skip, before the next request");
            request(e + 2); tick(40);       -- ...30000
            feb_event(e + 2, 9, 9, 9);
            wait_quiet;
            exp_ewt_drop := exp_ewt_drop + 1;
            check_sent(e + 2, 27);
            check_state("skip");
            -- the request was dropped when the next one arrived, before any event: only the request side is recorded
            check(LastDropReqEwt = X"FFFF", "last drop register, request " & hex(LastDropReqEwt));
            normal_event(e + 3, 8, 8, 8);
            check_state("after skip");

            report "=== 5: two requests without event in a row ===" severity note;
            e := X"000000030010";
            request(e);     wait for 20 us; tick;
            request(e + 1); wait for 20 us; tick;
            request(e + 2); tick(40);
            feb_event(e + 2, 5, 6, 7);
            wait_quiet;
            exp_ewt_drop := exp_ewt_drop + 2;
            check_sent(e + 2, 18);
            check_state("two skips");

            report "=== 3: late event, the next request is already queued ===" severity note;
            e := X"000000030020";
            request(e);     wait for 20 us; tick;
            request(e + 1); tick(40);
            feb_event(e, 8, 8, 8);          -- late
            wait_quiet;
            exp_ewt_drop := exp_ewt_drop + 1; exp_ev_drop := exp_ev_drop + 1;
            check_state("late event dropped");
            feb_event(e + 1, 3, 3, 3);
            wait_quiet;
            check_sent(e + 1, 9);
            check_state("late event");

            report "=== 3b: same with late events and both events already in the buffer ===" severity note;
            e := X"000000030030";
            request(e); request(e + 1); tick(40);
            feb_event(e, 40, 40, 40); feb_event(e + 1, 2, 2, 2);
            wait_quiet;
            exp_ewt_drop := exp_ewt_drop + 1; exp_ev_drop := exp_ev_drop + 1;
            check_sent(e + 1, 6);
            check_state("late event, two in buffer");

            report "=== 3c: drop of late events switched off: send on equality ===" severity note;
            DropLateEn <= '0'; tick(10);
            e := X"000000030040";
            request(e); request(e + 1); tick(40);
            feb_event(e, 8, 8, 8); feb_event(e + 1, 2, 2, 2);
            wait_quiet;
            check_sent(e, 24);
            check_sent(e + 1, 6);
            check_state("drop late off");
            DropLateEn <= '1'; tick(10);

            report "=== row 4: event older than the oldest request ===" severity note;
            e := X"000000030050";
            request(e);     wait for 20 us; tick;      -- no data
            request(e + 1); tick(40);
            feb_event(e + 1, 8, 8, 8);
            wait_quiet;
            exp_ewt_drop := exp_ewt_drop + 1;
            check_sent(e + 1, 24);
            feb_event(e, 30, 30, 30);                  -- very late, nothing is pending
            wait for 20 us; tick;
            check_state("old event waits in the link FIFOs");
            hdr0 := to_integer(unsigned(LinkFlushCnt));
            request(e + 2); tick(40);
            wait_quiet;                                -- old event is flushed from the link FIFOs
            check(to_integer(unsigned(LinkFlushCnt)) = hdr0 + 1 and LinkFIFOEmpty = "111",
                  "old event was not flushed by the next request");
            check(SnapCause(2 downto 0) = "110" and SnapCause(10 downto 8) = "000"
                  and SnapHead(0) = X"0022" and SnapWds(1) = "00000000100010",
                  "snapshot of the flush: cause " & hex(SnapCause) & " head " & hex(SnapHead(0)));
            check_state("old event dropped");
            feb_event(e + 2, 4, 5, 6);
            wait_quiet;
            check_sent(e + 2, 15);
            check_state("old event");

            report "=== 4: request arrives while the previous event is being sent ===" severity note;
            e := X"000000030060";
            request(e); tick(40);
            feb_event(e, 400, 400, 400);
            hdr0 := exp_hdr;
            wait until n_hdr = hdr0 + 1 for 100 us;    -- header of the big event is out
            request(e + 1);
            while pktFormerIdle = '0' loop
                tick;
                check(n_hdr = hdr0 + 1, "a header was started while the previous event was being sent");
            end loop;
            wait for 30 us; tick;
            check(n_hdr = hdr0 + 1, "a header was sent for an event that does not exist");
            check_sent(e, 1200);
            check_state("request during read out, no event yet");
            feb_event(e + 1, 8, 8, 8);
            wait_quiet;
            check_sent(e + 1, 24);
            check_state("request during read out");

            report "=== 4b: request and its event arrive while the previous event is being sent ===" severity note;
            e := X"000000030070";
            request(e); tick(40);
            feb_event(e, 700, 700, 700);
            hdr0 := exp_hdr;
            wait until n_hdr = hdr0 + 1 for 100 us;
            request(e + 1); tick(40);
            feb_event(e + 1, 8, 8, 8);
            wait_quiet;
            check_sent(e, 2100);
            check_sent(e + 1, 24);
            check_state("event built during read out");

            report "=== 7: truncated event ===" severity note;
            e := X"000000030080";
            request(e); tick(40);
            feb_event(e, 6000, 6000, 6000);            -- 18000 > 16357 words
            wait_quiet;
            check_sent(e, 16357, 1);
            normal_event(e + 1, 8, 8, 8);
            check_state("truncated event");

            report "=== 7b: truncated event that is late ===" severity note;
            e := X"000000030090";
            request(e); request(e + 1); tick(40);
            feb_event(e, 6000, 6000, 6000);
            wait_quiet;
            exp_ewt_drop := exp_ewt_drop + 1; exp_ev_drop := exp_ev_drop + 1;
            check_state("late truncated event dropped");
            feb_event(e + 1, 8, 8, 8);
            wait_quiet;
            check_sent(e + 1, 24);
            check_state("late truncated event");

            report "=== time out path (disabled in operation) ===" severity note;
            DRTimeout <= X"0400"; tick(10);
            e := X"0000000300A0";
            request(e);
            wait for 30 us; tick;                      -- header with time out status, no event
            check(n_hdr = exp_hdr + 1, "time out header not sent");
            check(h_ewt(exp_hdr) = std_logic_vector(e), "time out header EWT");
            check(h_stat(exp_hdr) = 8 and h_pk(exp_hdr) = 0, "time out header status / packet count");
            exp_hdr := exp_hdr + 1; exp_dreq := exp_dreq + 1;
            DRTimeout <= X"FFFF"; tick(10);
            feb_event(e, 8, 8, 8);                     -- its event comes after the time out
            tick(100);
            request(e + 1); tick(40);
            wait_quiet;                                -- and is flushed by the next request
            check(LinkFIFOEmpty = "111", "event after time out was not flushed");
            check_state("event after time out dropped");
            feb_event(e + 1, 8, 8, 8);
            wait_quiet;
            check_sent(e + 1, 24);
            check_state("time out");

            ---------------------------------------------------------------
            -- a8: link FIFO flush
            ---------------------------------------------------------------
            report "=== F0: no link FIFO reset in all cases so far except the two truncated events and the two flushes ===" severity note;
            check(to_integer(unsigned(TruncCnt)) = 2, "truncated events counted: " & integer'image(to_integer(unsigned(TruncCnt))));
            check(to_integer(unsigned(LinkFlushCnt)) = 4, "link FIFO resets: " & integer'image(to_integer(unsigned(LinkFlushCnt))));
            check(to_integer(unsigned(BadTagCnt)) = 0, "bad tags: " & integer'image(to_integer(unsigned(BadTagCnt))));

            report "=== F1: link 0 starts in the middle of an event, nonsense tag newer than the request ===" severity note;
            e := X"000000031000";
            request(e); tick(40);
            raw_words(0, 10, 30);                      -- payload only: head 200a, tag word 200d
            feb_part(1, e, 8); feb_part(2, e, 8);
            wait_quiet;
            check(to_integer(unsigned(BadTagCnt)) = 1, "F1: bad tag not seen");
            check(n_hdr = exp_hdr, "F1: something was sent for the nonsense event");
            check(to_integer(unsigned(EwtDropCnt)) = exp_ewt_drop, "F1: a request was dropped for the nonsense event");
            recover("F1", e + 1);

            report "=== F2: same, nonsense tag older than the request ===" severity note;
            e := X"000000033000";
            request(e); tick(40);
            raw_words(0, 10, 30);
            feb_part(1, e, 8); feb_part(2, e, 8);
            wait_quiet;
            check(n_hdr = exp_hdr, "F2: something was sent for the nonsense event");
            recover("F2", e + 1);

            report "=== F3: a head word that never becomes complete, builder waits ===" severity note;
            e := X"000000034000";
            lf_din(0) <= X"0100"; lf_wr(0) <= '1'; tick; lf_wr(0) <= '0'; tick(10);
            hdr0 := to_integer(unsigned(LinkFlushCnt));
            request(e); tick(40);                      -- the stray word is flushed by this request
            check(to_integer(unsigned(LinkFlushCnt)) = hdr0 + 1, "F3: stray word not flushed by the request");
            feb_event(e, 8, 8, 8);
            wait_quiet;
            check_sent(e, 24);
            check_state("F3");
            hdr0 := to_integer(unsigned(LinkFlushCnt));
            request(e + 1); tick(40);
            lf_din(0) <= X"0100"; lf_wr(0) <= '1'; tick; lf_wr(0) <= '0';   -- stray word in front of the event
            feb_event(e + 1, 8, 8, 8);
            wait_quiet;
            check(n_hdr = exp_hdr and EvBuildState = X"1", "F3: builder is not waiting");
            check(to_integer(unsigned(LinkFlushCnt)) = hdr0, "F3: flush without a new request");
            recover("F3", e + 2);

            report "=== F4: the same with the flush switched off: counted, nothing reset ===" severity note;
            LinkFlushEn <= '0'; tick(10);
            e := X"000000035000";
            request(e); tick(40);
            lf_din(0) <= X"0100"; lf_wr(0) <= '1'; tick; lf_wr(0) <= '0';
            feb_event(e, 8, 8, 8);
            wait_quiet;
            hdr0 := to_integer(unsigned(LinkFlushCnt));
            stale0 := to_integer(unsigned(StaleCnt));
            request(e + 1); tick(40);
            wait_quiet;
            check(to_integer(unsigned(StaleCnt)) = stale0 + 1, "F4: stale data not counted");
            check(to_integer(unsigned(LinkFlushCnt)) = hdr0 and LinkFIFOEmpty /= "111", "F4: flushed although switched off");
            check(SnapCause(2 downto 0) = "010" and SnapHead(0) = X"0100", "F4: snapshot " & hex(SnapCause) & " " & hex(SnapHead(0)));
            LinkFlushEn <= '1'; tick(10);
            recover("F4", e + 2);

            report "=== F5: flush while the event is still arriving, the rest comes without header ===" severity note;
            e := X"000000036000";
            request(e); tick(40);
            for i in 0 to 2 loop                       -- first half of the event of e
                lf_din(i) <= std_logic_vector(to_unsigned(44, 16)); lf_wr(i) <= '1';
            end loop;
            tick; lf_wr <= "000";
            for i in 0 to 2 loop raw_words(i, 100, 20); end loop;
            request(e + 1); tick(40);                  -- the DTC gave up on e
            for i in 0 to 2 loop raw_words(i, 120, 23); end loop;   -- the rest of e
            tick(200);
            recover("F5", e + 2);

            report "=== F6: tag guard switched off: the request is dropped as before ===" severity note;
            TagGuardEn <= '0'; LinkFlushEn <= '0'; tick(10);
            e := X"000000037000";
            hdr0 := to_integer(unsigned(EwtDropCnt));
            request(e); tick(40);
            feb_event(e + 5, 8, 8, 8);                 -- newer than the only request
            wait for 20 us; tick;                      -- the event waits for a request
            check(to_integer(unsigned(EwtDropCnt)) = hdr0 + 1 and n_hdr = exp_hdr, "F6: request not dropped");
            TagGuardEn <= '1'; LinkFlushEn <= '1'; tick(10);
            request(e + 5); tick(40);                  -- the event is still there and is sent
            wait_quiet;
            exp_ewt_drop := to_integer(unsigned(EwtDropCnt)); exp_ev_drop := to_integer(unsigned(EvDropCnt));
            check_sent(e + 5, 24);
            check_state("F6");

            report "=== F7: link 1 one event behind: sent with the tag of link 0 and the mismatch flag ===" severity note;
            e := X"000000038000";
            request(e); tick(40);
            feb_part(0, e, 8); feb_part(1, e - 1, 8); feb_part(2, e, 8);
            wait_quiet;
            check_sent(e, 24);
            check(h_cst(exp_hdr - 1)(30) = '1', "F7: mismatch flag not set, status " & hex(h_cst(exp_hdr - 1)));
            check_state("F7");
            normal_event(e + 1, 8, 8, 8);
            check(h_cst(exp_hdr - 1)(30) = '0', "F7: mismatch flag set for a good event, status " & hex(h_cst(exp_hdr - 1)));

            report "=== F8: link 0 one event behind: tag of the previous request, dropped as late ===" severity note;
            e := X"000000039000";
            request(e); wait for 20 us; tick;          -- nothing complete for e
            request(e + 1); tick(40);
            feb_part(0, e, 8); feb_part(1, e + 1, 8); feb_part(2, e + 1, 8);
            wait_quiet;
            exp_ewt_drop := exp_ewt_drop + 1; exp_ev_drop := exp_ev_drop + 1;
            check_state("F8");
            feb_event(e + 1, 8, 8, 8);                 -- back in step
            wait_quiet;
            check_sent(e + 1, 24);
            check_state("F8, in step again");

            report "=== F9: 300 requests without any event, the time stamp buffer must not fill up ===" severity note;
            e := X"00000003A000";
            hdr0 := to_integer(unsigned(EwtDropCnt));
            stale0 := 0;
            for k in 0 to 299 loop
                request(e + k); tick(60);
                if to_integer(unsigned(TStmpWds)) > stale0 then stale0 := to_integer(unsigned(TStmpWds)); end if;
            end loop;
            wait_quiet;
            check(stale0 <= 6, "F9: " & integer'image(stale0) & " words in the time stamp buffer");
            check(to_integer(unsigned(TStmpWds)) = 3, "F9: the newest request is not the only one left");
            check(to_integer(unsigned(EwtDropCnt)) = hdr0 + 299 and n_hdr = exp_hdr, "F9: requests dropped "
                  & integer'image(to_integer(unsigned(EwtDropCnt)) - hdr0));
            exp_ewt_drop := exp_ewt_drop + 299;
            feb_event(e + 299, 8, 8, 8);               -- the event of the newest request is still sent
            wait_quiet;
            check_sent(e + 299, 24);
            check_state("F9");

            report "=== bypass: order only, as before the change ===" severity note;
            MatchBypassReg <= '1'; tick(10);
            e := X"0000000300B0";
            request(e); tick(40);
            feb_event(e + 5, 8, 8, 8);                 -- wrong event on purpose
            wait_quiet;
            check(n_hdr = exp_hdr + 1 and h_ewt(exp_hdr) = std_logic_vector(e)
                  and h_ub(exp_hdr) = std_logic_vector(e(31 downto 0) + 5), "bypass sends in order");
            exp_hdr := exp_hdr + 1; exp_dreq := exp_dreq + 1;
            check_state("bypass");
            MatchBypassReg <= '0'; tick(10);

            report "=== reset with a request pending (run start) ===" severity note;
            e := X"0000000300C0";
            request(e); wait for 10 us; tick;
            reset <= '1'; tick(5); reset <= '0'; tick(20);
            exp_dreq := 0; exp_ewt_drop := 0; exp_ev_drop := 0;
            normal_event(X"000000000000", 8, 8, 8);
            normal_event(X"000000000001", 8, 8, 8);
            check_state("after reset");
        end if;

        tick(100);
        if errors = 0 then
            report "EwtMatch_tb: ALL CHECKS PASSED (" & integer'image(n_hdr) & " data headers)" severity note;
        else
            report "EwtMatch_tb: " & integer'image(errors) & " CHECKS FAILED" severity failure;
        end if;
        done <= true;
        wait;
    end process;

END;

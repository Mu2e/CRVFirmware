-- Simulation only: behavioural stand-ins for the Xilinx cores of FPGA 2
-- (FIFOs, clock generator, serializer, memory controller). Good enough to run
-- the top level; they do not model the timing of the real cores.

library ieee;
use ieee.std_logic_1164.all;

package sim_pkg is
    -- words written to the link FIFO (what goes to FPGA 1), for the test bench
    signal link_clk : std_logic := '0';
    signal link_wr  : std_logic := '0';
    signal link_dat : std_logic_vector(17 downto 0) := (others => '0');
end package sim_pkg;

---------------------------------------------------------------------------
-- first word fall through FIFO with two clocks
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity sim_fifo is
    generic (W : positive := 16; AW : positive := 12);
    port (rst, wr_clk, rd_clk : in std_logic;
          din   : in  std_logic_vector(W-1 downto 0);
          wr_en, rd_en : in std_logic;
          dout  : out std_logic_vector(W-1 downto 0);
          full, empty : out std_logic;
          rd_count, wr_count : out std_logic_vector(AW downto 0));
end entity;

architecture sim of sim_fifo is
    type mem_t is array (0 to 2**AW-1) of std_logic_vector(W-1 downto 0);
    signal mem : mem_t := (others => (others => '0'));
    signal wp, rp, wp_s1, wp_s2, rp_s1, rp_s2 : unsigned(AW downto 0) := (others => '0');
    signal empty_i, full_i : std_logic;
begin
    empty_i <= '1' when wp_s2 = rp else '0';
    full_i  <= '1' when (wp - rp_s2) >= 2**AW else '0';
    empty <= empty_i; full <= full_i;
    dout  <= mem(to_integer(rp(AW-1 downto 0)));
    rd_count <= std_logic_vector(wp_s2 - rp);
    wr_count <= std_logic_vector(wp - rp_s2);

    process(wr_clk, rst) begin
        if rst = '1' then wp <= (others => '0'); rp_s1 <= (others => '0'); rp_s2 <= (others => '0');
        elsif rising_edge(wr_clk) then
            rp_s1 <= rp; rp_s2 <= rp_s1;
            if wr_en = '1' and full_i = '0' then
                mem(to_integer(wp(AW-1 downto 0))) <= din;
                wp <= wp + 1;
            end if;
        end if;
    end process;

    process(rd_clk, rst) begin
        if rst = '1' then rp <= (others => '0'); wp_s1 <= (others => '0'); wp_s2 <= (others => '0');
        elsif rising_edge(rd_clk) then
            wp_s1 <= wp; wp_s2 <= wp_s1;
            if rd_en = '1' and empty_i = '0' then rp <= rp + 1; end if;
        end if;
    end process;
end architecture;

---------------------------------------------------------------------------
library ieee; use ieee.std_logic_1164.all;
entity PhyRxBuff is
  port (rst, wr_clk, rd_clk : in std_logic; din : in std_logic_vector(15 downto 0);
        wr_en, rd_en : in std_logic; dout : out std_logic_vector(15 downto 0);
        full, empty : out std_logic; rd_data_count : out std_logic_vector(12 downto 0));
end entity;
architecture sim of PhyRxBuff is begin
  f : entity work.sim_fifo generic map (W => 16, AW => 12)
      port map (rst, wr_clk, rd_clk, din, wr_en, rd_en, dout, full, empty, rd_data_count, open);
end architecture;

library ieee; use ieee.std_logic_1164.all;
entity PhyTxBuff is
  port (rst, wr_clk, rd_clk : in std_logic; din : in std_logic_vector(15 downto 0);
        wr_en, rd_en : in std_logic; dout : out std_logic_vector(15 downto 0);
        full, empty : out std_logic; wr_data_count : out std_logic_vector(9 downto 0));
end entity;
architecture sim of PhyTxBuff is
  signal wc : std_logic_vector(10 downto 0);
begin
  f : entity work.sim_fifo generic map (W => 16, AW => 10)
      port map (rst, wr_clk, rd_clk, din, wr_en, rd_en, dout, full, empty, open, wc);
  wr_data_count <= wc(9 downto 0);
end architecture;

library ieee; use ieee.std_logic_1164.all;
entity SMI_FIFO is
  port (rst, wr_clk, rd_clk : in std_logic; din : in std_logic_vector(23 downto 0);
        wr_en, rd_en : in std_logic; dout : out std_logic_vector(23 downto 0);
        full, empty : out std_logic);
end entity;
architecture sim of SMI_FIFO is begin
  f : entity work.sim_fifo generic map (W => 24, AW => 4)
      port map (rst, wr_clk, rd_clk, din, wr_en, rd_en, dout, full, empty, open, open);
end architecture;

library ieee; use ieee.std_logic_1164.all;
entity SCFIFO1Kx16 is
  port (clk, rst : in std_logic; din : in std_logic_vector(15 downto 0);
        wr_en, rd_en : in std_logic; dout : out std_logic_vector(15 downto 0);
        full, empty : out std_logic; data_count : out std_logic_vector(10 downto 0));
end entity;
architecture sim of SCFIFO1Kx16 is begin
  f : entity work.sim_fifo generic map (W => 16, AW => 10)
      port map (rst, clk, clk, din, wr_en, rd_en, dout, full, empty, data_count, open);
end architecture;

library ieee; use ieee.std_logic_1164.all;
entity SCFIFO_1Kx28 is
  port (clk, rst : in std_logic; din : in std_logic_vector(27 downto 0);
        wr_en, rd_en : in std_logic; dout : out std_logic_vector(27 downto 0);
        full, empty : out std_logic);
end entity;
architecture sim of SCFIFO_1Kx28 is begin
  f : entity work.sim_fifo generic map (W => 28, AW => 10)
      port map (rst, clk, clk, din, wr_en, rd_en, dout, full, empty, open, open);
end architecture;

-- 18 bit in, 9 bit out, upper half first
library ieee; use ieee.std_logic_1164.all; use ieee.numeric_std.all;
use work.sim_pkg.all;
entity LinkTxFIFO is
  port (rst, wr_clk, rd_clk : in std_logic; din : in std_logic_vector(17 downto 0);
        wr_en, rd_en : in std_logic; dout : out std_logic_vector(8 downto 0);
        full, empty : out std_logic);
end entity;
architecture sim of LinkTxFIFO is
  signal d18 : std_logic_vector(17 downto 0);
  signal e18, half, pop : std_logic := '0';
begin
  f : entity work.sim_fifo generic map (W => 18, AW => 14)
      port map (rst, wr_clk, rd_clk, din, wr_en, pop, d18, full, e18, open, open);
  empty <= e18;
  dout  <= d18(17 downto 9) when half = '0' else d18(8 downto 0);
  pop   <= rd_en and half and not e18;
  process(rd_clk, rst) begin
    if rst = '1' then half <= '0';
    elsif rising_edge(rd_clk) then
      if rd_en = '1' and e18 = '0' then half <= not half; end if;
    end if;
  end process;
  link_clk <= wr_clk;
  link_wr  <= wr_en;
  link_dat <= din;
end architecture;

library ieee; use ieee.std_logic_1164.all;
entity LinkTxFIFOTrace is
  port (wr_clk, rd_clk : in std_logic; rd_data_count : out std_logic_vector(12 downto 0);
        rst, wr_en, rd_en : in std_logic; din : in std_logic_vector(17 downto 0);
        dout : out std_logic_vector(8 downto 0); full, empty : out std_logic);
end entity;
architecture sim of LinkTxFIFOTrace is begin
  rd_data_count <= (others => '0'); dout <= (others => '0'); full <= '0'; empty <= '1';
end architecture;

library ieee; use ieee.std_logic_1164.all;
entity Sys_PLL is
  port (CLK_IN1_P, CLK_IN1_N : in std_logic; CLK_OUT1, CLK_OUT2, CLK_OUT3, CLK_OUT4 : out std_logic;
        RESET : in std_logic; LOCKED : out std_logic);
end entity;
architecture sim of Sys_PLL is
  signal c200, c100, c50 : std_logic := '0';
begin
  c200 <= not c200 after 2.5 ns;
  process(c200) begin if rising_edge(c200) then c100 <= not c100; end if; end process;
  process(c100) begin if rising_edge(c100) then c50 <= not c50; end if; end process;
  CLK_OUT1 <= '0'; CLK_OUT2 <= c100; CLK_OUT3 <= c200; CLK_OUT4 <= c50; LOCKED <= '1';
end architecture;

library ieee; use ieee.std_logic_1164.all;
entity LinkTx is
  generic (sys_w : integer := 4; dev_w : integer := 20);
  port (DATA_OUT_FROM_DEVICE : in std_logic_vector(dev_w-1 downto 0);
        DATA_OUT_TO_PINS_P, DATA_OUT_TO_PINS_N : out std_logic_vector(sys_w-1 downto 0);
        CLK_IN, CLK_DIV_IN, LOCKED_IN : in std_logic; LOCKED_OUT : out std_logic; IO_RESET : in std_logic);
end entity;
architecture sim of LinkTx is begin
  DATA_OUT_TO_PINS_P <= (others => '0'); DATA_OUT_TO_PINS_N <= (others => '1'); LOCKED_OUT <= LOCKED_IN;
end architecture;

-- Memory controller: write port p2, read port p3, 2**16 long words
library ieee; use ieee.std_logic_1164.all; use ieee.numeric_std.all;
entity LPDDR_Ctrl is
 generic(C3_P0_MASK_SIZE : integer := 4; C3_P0_DATA_PORT_SIZE : integer := 32;
    C3_P1_MASK_SIZE : integer := 4; C3_P1_DATA_PORT_SIZE : integer := 32;
    C3_MEMCLK_PERIOD : integer := 9400; C3_RST_ACT_LOW : integer := 0;
    C3_INPUT_CLK_TYPE : string := "DIFFERENTIAL"; C3_CALIB_SOFT_IP : string := "TRUE";
    C3_SIMULATION : string := "FALSE"; DEBUG_EN : integer := 0;
    C3_MEM_ADDR_ORDER : string := "BANK_ROW_COLUMN"; C3_NUM_DQ_PINS : integer := 16;
    C3_MEM_ADDR_WIDTH : integer := 14; C3_MEM_BANKADDR_WIDTH : integer := 2);
 port (
   mcb3_dram_dq : inout std_logic_vector(C3_NUM_DQ_PINS-1 downto 0);
   mcb3_dram_a : out std_logic_vector(C3_MEM_ADDR_WIDTH-1 downto 0);
   mcb3_dram_ba : out std_logic_vector(C3_MEM_BANKADDR_WIDTH-1 downto 0);
   mcb3_dram_cke, mcb3_dram_ras_n, mcb3_dram_cas_n, mcb3_dram_we_n, mcb3_dram_dm : out std_logic;
   mcb3_dram_udqs, mcb3_rzq : inout std_logic;
   mcb3_dram_udm : out std_logic;
   c3_sys_clk_p, c3_sys_clk_n, c3_sys_rst_i : in std_logic;
   c3_calib_done, c3_clk0, c3_rst0 : out std_logic;
   mcb3_dram_dqs : inout std_logic;
   mcb3_dram_ck, mcb3_dram_ck_n : out std_logic;
   c3_p2_cmd_clk, c3_p2_cmd_en : in std_logic;
   c3_p2_cmd_instr : in std_logic_vector(2 downto 0);
   c3_p2_cmd_bl : in std_logic_vector(5 downto 0);
   c3_p2_cmd_byte_addr : in std_logic_vector(29 downto 0);
   c3_p2_cmd_empty, c3_p2_cmd_full : out std_logic;
   c3_p2_wr_clk, c3_p2_wr_en : in std_logic;
   c3_p2_wr_mask : in std_logic_vector(3 downto 0);
   c3_p2_wr_data : in std_logic_vector(31 downto 0);
   c3_p2_wr_full, c3_p2_wr_empty : out std_logic;
   c3_p2_wr_count : out std_logic_vector(6 downto 0);
   c3_p2_wr_underrun, c3_p2_wr_error : out std_logic;
   c3_p3_cmd_clk, c3_p3_cmd_en : in std_logic;
   c3_p3_cmd_instr : in std_logic_vector(2 downto 0);
   c3_p3_cmd_bl : in std_logic_vector(5 downto 0);
   c3_p3_cmd_byte_addr : in std_logic_vector(29 downto 0);
   c3_p3_cmd_empty, c3_p3_cmd_full : out std_logic;
   c3_p3_rd_clk, c3_p3_rd_en : in std_logic;
   c3_p3_rd_data : out std_logic_vector(31 downto 0);
   c3_p3_rd_full, c3_p3_rd_empty : out std_logic;
   c3_p3_rd_count : out std_logic_vector(6 downto 0);
   c3_p3_rd_overflow, c3_p3_rd_error : out std_logic);
end entity;
architecture sim of LPDDR_Ctrl is
  type q_t is array (0 to 127) of std_logic_vector(31 downto 0);
  signal rd_data_i : std_logic_vector(31 downto 0) := (others => '0');
  signal wcnt, rcnt : integer range 0 to 127 := 0;
begin
  mcb3_dram_dq <= (others => 'Z'); mcb3_dram_a <= (others => '0'); mcb3_dram_ba <= (others => '0');
  mcb3_dram_cke <= '0'; mcb3_dram_ras_n <= '1'; mcb3_dram_cas_n <= '1'; mcb3_dram_we_n <= '1';
  mcb3_dram_dm <= '0'; mcb3_dram_udm <= '0'; mcb3_dram_udqs <= 'Z'; mcb3_rzq <= 'Z'; mcb3_dram_dqs <= 'Z';
  mcb3_dram_ck <= '0'; mcb3_dram_ck_n <= '1';
  c3_calib_done <= '1'; c3_clk0 <= '0'; c3_rst0 <= '0';
  c3_p2_cmd_empty <= '1'; c3_p2_cmd_full <= '0'; c3_p3_cmd_empty <= '1'; c3_p3_cmd_full <= '0';
  c3_p2_wr_underrun <= '0'; c3_p2_wr_error <= '0'; c3_p3_rd_overflow <= '0'; c3_p3_rd_error <= '0';
  c3_p2_wr_full  <= '1' when wcnt >= 64 else '0';
  c3_p2_wr_empty <= '1' when wcnt = 0 else '0';
  c3_p2_wr_count <= std_logic_vector(to_unsigned(wcnt, 7));
  c3_p3_rd_full  <= '1' when rcnt >= 64 else '0';
  c3_p3_rd_empty <= '1' when rcnt = 0 else '0';
  c3_p3_rd_count <= std_logic_vector(to_unsigned(rcnt, 7));
  c3_p3_rd_data  <= rd_data_i;

  process(c3_p2_wr_clk)
    type mem_t is array (0 to 65535) of std_logic_vector(31 downto 0);
    variable mem : mem_t := (others => (others => '0'));
    variable wq, rq : q_t;
    variable wn, rn, a, n : integer := 0;
    variable rdelay : integer := 0;
    variable ra, rl : integer := 0;
  begin
    if rising_edge(c3_p2_wr_clk) then
      -- read FIFO
      if c3_p3_rd_en = '1' and rn > 0 then
        for k in 0 to rn-2 loop rq(k) := rq(k+1); end loop;
        rn := rn - 1;
      end if;
      -- a pending read command delivers its words after a few clocks
      if rdelay > 0 then
        rdelay := rdelay - 1;
        if rdelay = 0 then
          for k in 0 to rl-1 loop
            if rn < 128 then rq(rn) := mem((ra + k) mod 65536); rn := rn + 1; end if;
          end loop;
        end if;
      end if;
      if c3_p3_cmd_en = '1' then
        ra := to_integer(unsigned(c3_p3_cmd_byte_addr(17 downto 2)));
        rl := to_integer(unsigned(c3_p3_cmd_bl)) + 1;
        rdelay := 6;
      end if;
      -- write FIFO
      if c3_p2_wr_en = '1' and wn < 128 then wq(wn) := c3_p2_wr_data; wn := wn + 1; end if;
      if c3_p2_cmd_en = '1' then
        a := to_integer(unsigned(c3_p2_cmd_byte_addr(17 downto 2)));
        n := to_integer(unsigned(c3_p2_cmd_bl)) + 1;
        if n > wn then n := wn; end if;
        for k in 0 to n-1 loop mem((a + k) mod 65536) := wq(k); end loop;
        for k in 0 to wn-n-1 loop wq(k) := wq(k+n); end loop;
        wn := wn - n;
      end if;
      wcnt <= wn; rcnt <= rn;
      if rn > 0 then rd_data_i <= rq(0); end if;
    end if;
  end process;
end architecture;

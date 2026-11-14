-- Simulation only: behavioural models of the first-word-fall-through FIFO
-- cores used around the event builder / packet former, for simulators that
-- do not have XilinxCoreLib (GHDL). With ISim use the real cores instead.
--
-- A written word becomes visible at the output two clocks after the write.
-- data_count includes the words in the output stage ("extra logic" option).

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity fwft_fifo_model is
    generic (DEPTH : natural := 256; CNTW : natural := 9);
    port (clk,rst,wr_en,rd_en : in std_logic;
          din        : in  std_logic_vector(15 downto 0);
          dout       : out std_logic_vector(15 downto 0);
          full,empty : out std_logic;
          data_count : out std_logic_vector(CNTW-1 downto 0));
end entity;

architecture sim of fwft_fifo_model is
    type mem_t is array (0 to DEPTH-1) of std_logic_vector(15 downto 0);
    signal empty_i : std_logic := '1';
begin
    empty <= empty_i;
    process(clk, rst)
        variable mem : mem_t;
        variable wr, wr_d1, wr_d2, rd : natural := 0; -- running word counts
    begin
        if rst = '1' then
            wr := 0; wr_d1 := 0; wr_d2 := 0; rd := 0;
            empty_i <= '1'; full <= '0';
            dout <= (others => '0'); data_count <= (others => '0');
        elsif rising_edge(clk) then
            if rd_en = '1' and empty_i = '0' then rd := rd + 1; end if;
            wr_d2 := wr_d1; wr_d1 := wr;
            if wr_en = '1' and (wr - rd) < DEPTH then
                mem(wr mod DEPTH) := din; wr := wr + 1;
            end if;
            if rd = wr_d2 then empty_i <= '1'; else empty_i <= '0'; end if;
            if rd /= wr_d2 then dout <= mem(rd mod DEPTH); end if;
            if (wr - rd) >= DEPTH then full <= '1'; else full <= '0'; end if;
            data_count <= std_logic_vector(to_unsigned(wr - rd, CNTW));
        end if;
    end process;
end architecture;

LIBRARY ieee;
use ieee.std_logic_1164.all;
entity TrigPktBuff is
  port (clk,rst,wr_en,rd_en : in std_logic;
    din : in std_logic_vector(15 downto 0);
    dout : out std_logic_vector(15 downto 0);
    full,empty : out std_logic;
    data_count : out std_logic_vector(8 downto 0));
end entity;
architecture sim of TrigPktBuff is
begin
    m : entity work.fwft_fifo_model generic map (DEPTH => 256, CNTW => 9)
        port map (clk,rst,wr_en,rd_en,din,dout,full,empty,data_count);
end architecture;

LIBRARY ieee;
use ieee.std_logic_1164.all;
entity FIFO_SC_4Kx16 is
  port (clk,rst,wr_en,rd_en : in std_logic;
    din : in std_logic_vector(15 downto 0);
    dout : out std_logic_vector(15 downto 0);
    full,empty : out std_logic);
end entity;
architecture sim of FIFO_SC_4Kx16 is
begin
    m : entity work.fwft_fifo_model generic map (DEPTH => 16384, CNTW => 15)
        port map (clk,rst,wr_en,rd_en,din,dout,full,empty,open);
end architecture;

-- link FIFO (dual clock in the design, one clock is enough here)
LIBRARY ieee;
use ieee.std_logic_1164.all;
entity LinkFIFO is
  port (rst,wr_clk,rd_clk,
		  wr_en,rd_en  : in std_logic;
    din : in std_logic_vector(15 downto 0);
    dout : out std_logic_vector(15 downto 0);
    full,empty : out STD_LOGIC;
    rd_data_count : out std_logic_vector(13 downto 0));
end entity;
architecture sim of LinkFIFO is
    signal cnt : std_logic_vector(14 downto 0);
begin
    m : entity work.fwft_fifo_model generic map (DEPTH => 8192, CNTW => 15)
        port map (rd_clk,rst,wr_en,rd_en,din,dout,full,empty,cnt);
    rd_data_count <= cnt(13 downto 0);
end architecture;

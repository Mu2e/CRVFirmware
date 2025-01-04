----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    22:21:18 02/24/2024 
-- Design Name: 
-- Module Name:    Clk80MHzGen - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.all;

-- for BUFG
library UNISIM;
use UNISIM.VComponents.all;


entity Clk80MHzGen is
    Port ( clk160     : in  std_logic;
           rst        : in  std_logic; -- active low
           syncEnable : in  std_logic;
           MarkerBits : in  std_logic_vector(15 downto 0);
           clk80      : out std_logic;
           shiftCnt   : out std_logic_vector(7 downto 0));
end Clk80MHzGen;

architecture Behavioral of Clk80MHzGen is

signal outp      : std_logic;
signal outp_bufg : std_logic;  -- Buffered output clock
--signal cnt : std_logic_vector(0 downto 0);
signal align_cnt : unsigned(7 downto 0);
signal outp_next : std_logic;
signal sync_condition, marker_match : std_logic;
--signal sync_condition_reg, sync_condition_reg2 : std_logic;

begin

    sync_condition <= '1' when (syncEnable = '1') and MarkerBits = X"F0F0" else '0';

    BUFG_inst_out : BUFG
    port map (
        I => outp,
        O => outp_bufg
    );

    process(clk160, rst)
    begin 
        if rst = '0' then
         outp <= '0';
			outp_next <= '1';
         align_cnt <= (others => '0');
         --marker_match <= '0';
			--sync_condition_reg <= '0';
        elsif rising_edge(clk160) then
            outp <= outp_next;

				--sync_condition_reg <= sync_condition;
				--sync_condition_reg2 <= sync_condition_reg;

            if sync_condition = '1' then
                outp_next <= '1';
                if outp = '1' then -- phase change
                    align_cnt <= align_cnt + 1;
                else
                    align_cnt <= align_cnt;
                end if;
            else
                outp_next <= not outp_next; -- clock
            end if;
        end if;
    end process;

    clk80 <= outp_bufg;
    shiftCnt <= std_logic_vector(align_cnt);
end Behavioral;


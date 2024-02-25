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
use IEEE.std_logic_unsigned.all;


entity Clk80MHzGen is
    Port ( clk160     : in  std_logic;
	        rst        : in  std_logic; -- active low
           syncEnable : in  std_logic;
           MarkerBits : in  std_logic_vector(15 downto 0);
           clk80      : out std_logic;
			  shiftCnt   : out std_logic_vector(7 downto 0));
end Clk80MHzGen;

architecture Behavioral of Clk80MHzGen is

signal outp : std_logic;
--signal cnt : std_logic_vector(0 downto 0);
signal align_cnt : std_logic_vector(7 downto 0);

begin

clk80 <= outp;
shiftCnt <= align_cnt;

ClkGen : process(clk160, rst)
  begin
    if rst = '0'
	   then
		  --cnt <= "0";
		  outp <= '0'; -- or not?
		  align_cnt <= (others => '0');
	 elsif rising_edge(clk160) then
	     if syncEnable = '1' and MarkerBits = X"F0F0" then
			   --cnt <= "0";
				outp <= '1';
				if outp = '1' then -- not 0, aka a phase shift is needed
				    align_cnt <= align_cnt + 1;
			   else
				    align_cnt <= align_cnt;
				end if;
		  else
		      outp <= not outp;
		      --if cnt = "1" then
				--    outp <= not outp;
				--	 cnt <= "0";
				--else
				--    cnt <= cnt + 1;
				--	 outp <= outp;
				---end if;
				align_cnt <= align_cnt;
		  end if;
	 end if;
  
end process; -- ClkGen

end Behavioral;


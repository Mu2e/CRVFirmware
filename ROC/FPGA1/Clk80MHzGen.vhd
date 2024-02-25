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
           clk80      : out std_logic);
end Clk80MHzGen;

architecture Behavioral of Clk80MHzGen is

signal outp : std_logic;
--signal cnt : std_logic_vector(0 downto 0);

begin

clk80 <= outp;

ClkGen : process(clk160, rst)
  begin
    if rst = '0'
	   then
		  --cnt <= "0";
		  outp <= '0'; -- or not?
	 elsif rising_edge(clk160) then
	     if syncEnable = '1' and MarkerBits = X"F0F0" then
			   --cnt <= "0";
				outp <= '1';
		  else
		      outp <= not outp;
		      --if cnt = "1" then
				--    outp <= not outp;
				--	 cnt <= "0";
				--else
				--    cnt <= cnt + 1;
				--	 outp <= outp;
				---end if;
		  end if;
	 end if;
  
end process; -- ClkGen

end Behavioral;


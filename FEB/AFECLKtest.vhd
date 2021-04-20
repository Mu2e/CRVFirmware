--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   21:44:57 02/16/2021
-- Design Name:   
-- Module Name:   /home/newg2/Downloads/roc1_1/markertest.vhd
-- Project Name:  Controller_FPGA1
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: ControllerFPGA_1
-- 
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
use ieee.std_logic_unsigned.all;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY afeclktest IS
END afeclktest;
 
ARCHITECTURE behavior OF afeclktest IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
    

   --Inputs
   signal SysClk, CpldRst  : std_logic;

	--Internal
	signal Strt_req, sqWav, RefIn : std_logic;
	signal FBDiv : std_logic_vector(2 downto 0);

 	--Outputs
   signal AFEClk : std_logic;

   -- Clock period definitions
   constant SysClk_period : time := 6.25 ns;
 
BEGIN
 
RefIn <= SqWav;
 -- Instantiate the Unit Under Test (UUT)
main : process(SysClk, CpldRst)
  begin 
    -- asynchronous reset/preset
	if CpldRst = '0' then
		AFEClk <= '0';
		Strt_req <= '0';
		
	elsif rising_edge (SysClk) then 
	   if Strt_req = '1' then AFEClk <= '0';
      else AFEClk <= not AFEClk;
      end if;
	end if;
end process; -- EthProc

IntClkDiv : process(SysClk,CpldRst)
begin
 if CpldRst = '0' then 
 
	FBDiv <= "000"; 
	SqWav <= '0';

elsif falling_edge(SysClk) then

 SqWav <= FBDiv(2);
 FBDiv <= FBDiv + 1;

end if; -- CpldRst
end process IntClkDiv;

   -- Clock process definitions
   SysClk_process :process
   begin
		SysClk <= '1';
		wait for SysClk_period/2;
		SysClk <= '0';
		wait for SysClk_period/2;
   end process;
 

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
		CpldRst<='0';
		--MarkerBits<=X"0000";
      wait for 102 ns;	
		CpldRst<='1';
		
      wait;
   end process;

END;

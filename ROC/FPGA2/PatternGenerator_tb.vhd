--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   16:29:10 10/25/2024
-- Design Name:   
-- Module Name:   /home/scorrodi/Documents/CRVFirmwareProdRoc/ROC/FPGA2/PatternGenerator_tb.vhd
-- Project Name:  Controller_FPGA2
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: PatternGenerator
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
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use work.Proj_Defs.all;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY PatternGenerator_tb IS
END PatternGenerator_tb;
 
ARCHITECTURE behavior OF PatternGenerator_tb IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT PatternGenerator
    PORT(
         clk : IN  std_logic;
         rst : IN  std_logic;
         start : IN  std_logic;
         nwords : IN  std_logic_vector(7 downto 0);
         data : OUT  std_logic_vector(15 downto 0);
         valid : OUT  std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal clk : std_logic := '0';
   signal rst : std_logic := '0';
   signal start : std_logic := '0';
   signal nwords : std_logic_vector(7 downto 0) := (others => '0');

 	--Outputs
   signal data : std_logic_vector(15 downto 0);
   signal valid : std_logic;

   -- Clock period definitions
   constant clk_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: PatternGenerator PORT MAP (
          clk => clk,
          rst => rst,
          start => start,
          nwords => nwords,
          data => data,
          valid => valid
        );

   -- Clock process definitions
   clk_process :process
   begin
		clk <= '0';
		wait for clk_period/2;
		clk <= '1';
		wait for clk_period/2;
   end process;
 

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
		nwords <= X"0a";
		start  <= '0';
		rst <= '1';
      wait for 100 ns;	
		rst <= '0';
      wait for clk_period*4*32;
		start <= '1';
		wait for clk_period;
		start <= '0';
		wait for clk_period*15*32;
		start <= '1';
		wait for clk_period;
		start <= '0';
		wait for clk_period*15*32;
		start <= '1';
		wait for clk_period;
		nwords <= X"00";
		start <= '0';
		wait for clk_period*15*32;


      wait;
   end process;

END;

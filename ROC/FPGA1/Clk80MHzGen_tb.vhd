--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   22:42:57 02/24/2024
-- Design Name:   
-- Module Name:   /home/scorrodi/Documents/mu2e-daq-firmware-crv/ROC/FPGA1/Clk80MHzGen_tb.vhd
-- Project Name:  Controller_FPGA1
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: Clk80MHzGen
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
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY Clk80MHzGen_tb IS
END Clk80MHzGen_tb;
 
ARCHITECTURE behavior OF Clk80MHzGen_tb IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT Clk80MHzGen
    PORT(
         clk160 : IN  std_logic;
         rst : IN  std_logic;
         syncEnable : IN  std_logic;
         MarkerBits : IN  std_logic_vector(15 downto 0);
         clk80 : OUT  std_logic;
			shiftCnt : OUT std_logic_vector(7 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal clk160 : std_logic := '0';
   signal rst : std_logic := '0';
   signal syncEnable : std_logic := '0';
   signal MarkerBits : std_logic_vector(15 downto 0) := (others => '0');
	signal shiftCnt : std_logic_vector(7 downto 0);

 	--Outputs
   signal clk80 : std_logic;

   -- Clock period definitions
   constant clk160_period : time := 6.25 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: Clk80MHzGen PORT MAP (
          clk160 => clk160,
          rst => rst,
          syncEnable => syncEnable,
          MarkerBits => MarkerBits,
          clk80 => clk80,
			 shiftCnt => shiftCnt
        );

   -- Clock process definitions
   clk160_process :process
   begin
		clk160 <= '0';
		wait for clk160_period/2;
		clk160 <= '1';
		wait for clk160_period/2;
   end process;
 

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
      wait for 100 ns;	

      wait for clk160_period*7.5;

      -- insert stimulus here 
		rst <= '1';
		wait for clk160_period*6;
		MarkerBits <= X"C0C0";
		wait for clk160_period*6;
		MarkerBits <= X"F0F0";
		wait for clk160_period*6;
		syncEnable <= '1';
		MarkerBits <= X"C0C0";
		wait for clk160_period*6;
		MarkerBits <= X"F0F0";
		wait for clk160_period;
		MarkerBits <= X"F0C0";
		wait for clk160_period*1.5;
		MarkerBits <= X"F0F0";
		wait for clk160_period;
		MarkerBits <= X"F0C0";
		

      wait;
   end process;

END;

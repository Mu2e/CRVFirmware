--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   16:08:36 07/05/2024
-- Design Name:   
-- Module Name:   /home/scorrodi/Documents/CRVFirmwareProdRoc/ROC/FPGA1/EventBuilder_GR_tb.vhd
-- Project Name:  Controller_FPGA1
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: EventBuilder
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
 
use work.Project_defs.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY EventBuilder_GR_tb IS
END EventBuilder_GR_tb;
 
ARCHITECTURE behavior OF EventBuilder_GR_tb IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT EventBuilder
    PORT(
         clk : IN  std_logic;
         reset : IN  std_logic;
         FormRst : IN  std_logic;
         LinkFIFOOut : IN  Array_3x16;
         LinkFIFORdCnt : IN  Array_3x13;
         LinkFIFOEmpty : IN  std_logic_vector(2 downto 0);
         LinkFIFORdReq : OUT  std_logic_vector(2 downto 0);
         EventBuff_Dat : OUT  std_logic_vector(15 downto 0);
         EventBuff_WrtEn : OUT  std_logic;
         TStmpWds : IN  std_logic_vector(8 downto 0);
         ActiveReg : IN  std_logic_vector(23 downto 0);
         MarkerDelayed : IN  std_logic_vector(3 downto 0);
         LinkRDDL : IN  std_logic_vector(1 downto 0);
         AddrReg : IN  std_logic_vector(11 downto 0);
         FormHold : IN  std_logic;
         sendGR : IN  std_logic;
         uBinHeader : IN  std_logic;
         uBwrt : IN  std_logic;
         GA : IN  std_logic_vector(1 downto 0);
         ExtuBunchCount : IN  std_logic_vector(47 downto 0);
         HeartBtCnt : IN  std_logic_vector(15 downto 0);
         HeartBeatCnt : IN  std_logic_vector(15 downto 0);
         LastWindow : IN  std_logic_vector(15 downto 0);
         Stats : IN  std_logic_vector(15 downto 0);
         InjectionTs : IN  std_logic_vector(15 downto 0);
         InjectionWindow : IN  std_logic_vector(15 downto 0);
         FakeNum : OUT  std_logic_vector(7 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal clk : std_logic := '0';
   signal reset : std_logic := '1';
   signal FormRst : std_logic := '0';
   signal LinkFIFOOut : Array_3x16 := (others => (others => '0'));
   signal LinkFIFORdCnt : Array_3x13 := (others => (others => '0'));
   signal LinkFIFOEmpty : std_logic_vector(2 downto 0) := (others => '0');
   signal TStmpWds : std_logic_vector(8 downto 0) := (others => '0');
   signal ActiveReg : std_logic_vector(23 downto 0) := (others => '0');
   signal MarkerDelayed : std_logic_vector(3 downto 0) := (others => '0');
   signal LinkRDDL : std_logic_vector(1 downto 0) := (others => '0');
   signal AddrReg : std_logic_vector(11 downto 0) := (others => '0');
   signal FormHold : std_logic := '1';
   signal sendGR : std_logic := '0';
   signal uBinHeader : std_logic := '0';
   signal uBwrt : std_logic := '0';
   signal GA : std_logic_vector(1 downto 0) := (others => '0');
   signal ExtuBunchCount : std_logic_vector(47 downto 0) := (others => '0');
   signal HeartBtCnt : std_logic_vector(15 downto 0) := (others => '0');
   signal HeartBeatCnt : std_logic_vector(15 downto 0) := (others => '0');
   signal LastWindow : std_logic_vector(15 downto 0) := (others => '0');
   signal Stats : std_logic_vector(15 downto 0) := (others => '0');
   signal InjectionTs : std_logic_vector(15 downto 0) := (others => '0');
   signal InjectionWindow : std_logic_vector(15 downto 0) := (others => '0');

 	--Outputs
   signal LinkFIFORdReq : std_logic_vector(2 downto 0);
   signal EventBuff_Dat : std_logic_vector(15 downto 0);
   signal EventBuff_WrtEn : std_logic;
   signal FakeNum : std_logic_vector(7 downto 0);

   -- Clock period definitions
   constant clk_period : time := 6.4 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: EventBuilder PORT MAP (
          clk => clk,
          reset => reset,
          FormRst => FormRst,
          LinkFIFOOut => LinkFIFOOut,
          LinkFIFORdCnt => LinkFIFORdCnt,
          LinkFIFOEmpty => LinkFIFOEmpty,
          LinkFIFORdReq => LinkFIFORdReq,
          EventBuff_Dat => EventBuff_Dat,
          EventBuff_WrtEn => EventBuff_WrtEn,
          TStmpWds => TStmpWds,
          ActiveReg => ActiveReg,
          MarkerDelayed => MarkerDelayed,
          LinkRDDL => LinkRDDL,
          AddrReg => AddrReg,
          FormHold => FormHold,
          sendGR => sendGR,
          uBinHeader => uBinHeader,
          uBwrt => uBwrt,
          GA => GA,
          ExtuBunchCount => ExtuBunchCount,
          HeartBtCnt => HeartBtCnt,
          HeartBeatCnt => HeartBeatCnt,
          LastWindow => LastWindow,
          Stats => Stats,
          InjectionTs => InjectionTs,
          InjectionWindow => InjectionWindow,
          FakeNum => FakeNum
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
      wait for clk_period*1.75;	
		reset <= '0';
		wait for clk_period *1.25;
		sendGR <= '1';
		
		wait for clk_period  * 3.2;
		MarkerDelayed <= "0001";
		wait for 12.5ns;
		MarkerDelayed <= "0010";
		wait for 12.5ns;
		MarkerDelayed <= "0100";
		wait for 12.5ns;
		MarkerDelayed <= "1000";
		

      wait for clk_period*10.34;
		wait for clk_period  * 3.2;
		MarkerDelayed <= "0001";
		wait for 12.5ns;
		MarkerDelayed <= "0010";
		wait for 12.5ns;
		MarkerDelayed <= "0100";
		wait for 12.5ns;
		MarkerDelayed <= "1000";

      -- insert stimulus here 

      wait;
   end process;

END;

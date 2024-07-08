--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   21:02:39 07/07/2024
-- Design Name:   
-- Module Name:   /home/scorrodi/Documents/CRVFirmwareProdRoc/ROC/FPGA1/PacketFormer_Data_tb.vhd
-- Project Name:  Controller_FPGA1
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: PacketFormer
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
 
ENTITY PacketFormer_Data_tb IS
END PacketFormer_Data_tb;
 
ARCHITECTURE behavior OF PacketFormer_Data_tb IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT PacketFormer
    PORT(
         clk : IN  std_logic;
         reset : IN  std_logic;
         pktFormerSend : IN  std_logic;
         pktFormerTimeout : IN  std_logic;
         FormRst : IN  std_logic;
         EventBuff_Out : IN  std_logic_vector(15 downto 0);
         EventBuff_RdEn : OUT  std_logic;
         TStmpBuff_rd_en : OUT  std_logic;
         TStmpBuff_Out : IN  std_logic_vector(15 downto 0);
         EventBuff_Empty : IN  std_logic;
         ActiveReg : IN  std_logic_vector(23 downto 0);
         DCSBuffRdCnt : IN  std_logic_vector(12 downto 0);
         DCSBuff_Out : IN  std_logic_vector(15 downto 0);
         DCSBuff_rd_en : OUT  std_logic;
         DRdone : OUT  std_logic;
         TxCRCEn : OUT  std_logic;
         TxCRCRst : OUT  std_logic;
         TxCRCDat : OUT  std_logic_vector(15 downto 0);
         TxCRC : IN  std_logic_vector(15 downto 0);
         WdCountBuff_WrtEn : OUT  std_logic;
         GTPTx : OUT  std_logic_vector(15 downto 0);
         TxCharIsK : OUT  std_logic_vector(1 downto 0);
         GTPTxBuff_In : OUT  std_logic_vector(15 downto 0);
         LoopbackMode : IN  std_logic_vector(2 downto 0);
         uBwrt : IN  std_logic;
         IDReg : IN  std_logic_vector(3 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal clk : std_logic := '0';
   signal reset : std_logic := '1';
   signal pktFormerSend : std_logic := '0';
   signal pktFormerTimeout : std_logic := '0';
   signal FormRst : std_logic := '0';
   signal EventBuff_Out : std_logic_vector(15 downto 0) := (others => '0');
   signal TStmpBuff_Out : std_logic_vector(15 downto 0) := (others => '0');
   signal EventBuff_Empty : std_logic := '0';
   signal ActiveReg : std_logic_vector(23 downto 0) := (others => '0');
   signal DCSBuffRdCnt : std_logic_vector(12 downto 0) := (others => '0');
   signal DCSBuff_Out : std_logic_vector(15 downto 0) := (others => '0');
   signal TxCRC : std_logic_vector(15 downto 0) := (others => '0');
   signal LoopbackMode : std_logic_vector(2 downto 0) := (others => '0');
   signal uBwrt : std_logic := '0';
   signal IDReg : std_logic_vector(3 downto 0) := (others => '0');

 	--Outputs
   signal EventBuff_RdEn : std_logic;
   signal TStmpBuff_rd_en : std_logic;
   signal DCSBuff_rd_en : std_logic;
   signal DRdone : std_logic;
   signal TxCRCEn : std_logic;
   signal TxCRCRst : std_logic;
   signal TxCRCDat : std_logic_vector(15 downto 0);
   signal WdCountBuff_WrtEn : std_logic;
   signal GTPTx : std_logic_vector(15 downto 0);
   signal TxCharIsK : std_logic_vector(1 downto 0);
   signal GTPTxBuff_In : std_logic_vector(15 downto 0);

   -- Clock period definitions
   constant clk_period : time := 6.4 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: PacketFormer PORT MAP (
          clk => clk,
          reset => reset,
          pktFormerSend => pktFormerSend,
          pktFormerTimeout => pktFormerTimeout,
          FormRst => FormRst,
          EventBuff_Out => EventBuff_Out,
          EventBuff_RdEn => EventBuff_RdEn,
          TStmpBuff_rd_en => TStmpBuff_rd_en,
          TStmpBuff_Out => TStmpBuff_Out,
          EventBuff_Empty => EventBuff_Empty,
          ActiveReg => ActiveReg,
          DCSBuffRdCnt => DCSBuffRdCnt,
          DCSBuff_Out => DCSBuff_Out,
          DCSBuff_rd_en => DCSBuff_rd_en,
          DRdone => DRdone,
          TxCRCEn => TxCRCEn,
          TxCRCRst => TxCRCRst,
          TxCRCDat => TxCRCDat,
          TxCRC => TxCRC,
          WdCountBuff_WrtEn => WdCountBuff_WrtEn,
          GTPTx => GTPTx,
          TxCharIsK => TxCharIsK,
          GTPTxBuff_In => GTPTxBuff_In,
          LoopbackMode => LoopbackMode,
          uBwrt => uBwrt,
          IDReg => IDReg
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
      wait for 100 ns;	
		reset <= '0';

      wait for clk_period*10;
		pktFormerSend <= '1';
		pktFormerTimeout <= '1';
		
				
		wait until DRdone = '1';
		pktFormerSend <= '0';
		pktFormerTimeout <= '0';

      -- insert stimulus here 

      wait;
   end process;

END;

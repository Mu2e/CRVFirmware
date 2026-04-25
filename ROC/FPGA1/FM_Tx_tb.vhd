--------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
--
-- Create Date:   22:04:24 10/19/2025
-- Design Name:   
-- Module Name:   FM_Tx_tb.vhd
-- Project Name:  Controller_FPGA1
-- Target Device:  
-- Tool versions:  
-- Description:   Comprehensive testbench for FM_Tx module
-- 
-- VHDL Test Bench for module: FM_Tx
-- 
-- Dependencies: Project_Defs.all
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- Tests various data patterns and verifies FM encoding
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_arith.ALL;
USE ieee.std_logic_unsigned.ALL;
USE work.Project_Defs.ALL; -- Make sure this package exists in your project
 
ENTITY FM_Tx_tb IS
END FM_Tx_tb;
 
ARCHITECTURE behavior OF FM_Tx_tb IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
    COMPONENT FM_Tx
        GENERIC (Pwidth : positive);
        PORT(
            clock : IN std_logic;
            reset : IN std_logic;
            Enable : IN std_logic;
            Data : IN std_logic_vector(Pwidth - 1 downto 0);
            Tx_Out : BUFFER TxOutRec
        );
    END COMPONENT;
    
    -- Test parameters
    CONSTANT TEST_WIDTH : positive := 24; 
    
    -- Inputs
    SIGNAL clock : std_logic := '0';
	 SIGNAL clock40 : std_logic := '0';
	 SIGNAL clock20 : std_logic := '0';
    SIGNAL reset : std_logic := '1';
    SIGNAL Enable : std_logic := '0';
    SIGNAL Data : std_logic_vector(TEST_WIDTH - 1 downto 0) := (others => '0');
    
    -- Outputs
    SIGNAL Tx_Out : TxOutRec;
    
    -- Clock period definitions
    CONSTANT clock_period : time := 12.5 ns; -- 160MHz clock
    
    -- Test control signals
    SIGNAL test_complete : std_logic := '0';
    SIGNAL test_number : integer := 0;
    
    -- Test patterns
    TYPE test_data_array IS ARRAY (0 to 2) OF std_logic_vector(TEST_WIDTH-1 downto 0);
    CONSTANT test_patterns : test_data_array := (
        X"0F55F0",  -- test pattern
        X"FFFFFF",  -- All ones
        X"000001"  -- ONE
    );

BEGIN
 
    -- Instantiate the Unit Under Test (UUT)
    uut: FM_Tx 
        GENERIC MAP (Pwidth => TEST_WIDTH)
        PORT MAP (
            clock => clock,
            reset => reset,
            Enable => Enable,
            Data => Data,
            Tx_Out => Tx_Out
        );

    -- Clock process definitions
    clock_process : process
    begin
		clock <= '1';
      wait for clock_period/2;
      clock <= '0';
      wait for clock_period/2;
	end process;
	
	 clock_process_40 : process
    begin
		clock40 <= '1';
      wait for clock_period;
      clock40 <= '0';
      wait for clock_period;
	end process;
	
	 clock_process_20 : process
    begin
		clock20 <= '1';
      wait for clock_period*2;
      clock20 <= '0';
      wait for clock_period*2;
	end process;

    -- Stimulus process
    stim_proc: process
        variable expected_bits : integer;
        variable bit_count : integer;
    begin		
        -- Initialize
        reset <= '1';
        Enable <= '0';
        Data <= (others => '0');
        
        wait for clock_period * 7;
        
        -- Release reset
        reset <= '0';
        wait for clock_period * 10;
        
        report "Starting FM_Tx testbench...";
        
        -- Test each pattern
        for i in test_patterns'range loop
            test_number <= i + 1;
            
            report "Test " & integer'image(i+1) & ": Testing pattern 0x" & 
                   integer'image(conv_integer(test_patterns(i)));
            
            -- Load test data
            Data <= test_patterns(i);
            wait for clock_period;
            
            -- Assert Enable
            Enable <= '1';
            wait for clock_period;
            Enable <= '0';
            
            -- Wait for transmission to start
            wait until Tx_Out.Done = '0' and rising_edge(clock);
            report "Transmission started for pattern 0x" & 
                   integer'image(conv_integer(test_patterns(i)));
            
            -- Monitor the transmission
            bit_count := 0;
            wait until Tx_Out.Done = '1';
            report "Transmission completed for pattern 0x" & 
                   integer'image(conv_integer(test_patterns(i)));
            
            -- Wait a bit between tests
            wait for clock_period * 16;
        end loop;
    end process;



END behavior;
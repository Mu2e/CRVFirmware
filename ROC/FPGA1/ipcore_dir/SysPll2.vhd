-- file: SysPll2.vhd
-- 
-- (c) Copyright 2008 - 2011 Xilinx, Inc. All rights reserved.
-- 
-- This file contains confidential and proprietary information
-- of Xilinx, Inc. and is protected under U.S. and
-- international copyright and other intellectual property
-- laws.
-- 
-- DISCLAIMER
-- This disclaimer is not a license and does not grant any
-- rights to the materials distributed herewith. Except as
-- otherwise provided in a valid license issued to you by
-- Xilinx, and to the maximum extent permitted by applicable
-- law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
-- WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
-- AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
-- BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
-- INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
-- (2) Xilinx shall not be liable (whether in contract or tort,
-- including negligence, or under any other theory of
-- liability) for any loss or damage of any kind or nature
-- related to, arising under or in connection with these
-- materials, including for any direct, or any indirect,
-- special, incidental, or consequential loss or damage
-- (including loss of data, profits, goodwill, or any type of
-- loss or damage suffered as a result of any action brought
-- by a third party) even if such damage or loss was
-- reasonably foreseeable or Xilinx had been advised of the
-- possibility of the same.
-- 
-- CRITICAL APPLICATIONS
-- Xilinx products are not designed or intended to be fail-
-- safe, or for use in any application requiring fail-safe
-- performance, such as life-support or safety devices or
-- systems, Class III medical devices, nuclear facilities,
-- applications related to the deployment of airbags, or any
-- other applications that could lead to death, personal
-- injury, or severe property or environmental damage
-- (individually and collectively, "Critical
-- Applications"). Customer assumes the sole risk and
-- liability of any use of Xilinx products in Critical
-- Applications, subject only to applicable laws and
-- regulations governing limitations on product liability.
-- 
-- THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
-- PART OF THIS FILE AT ALL TIMES.
-- 
------------------------------------------------------------------------------
-- User entered comments
------------------------------------------------------------------------------
-- None
--
------------------------------------------------------------------------------
-- "Output    Output      Phase     Duty      Pk-to-Pk        Phase"
-- "Clock    Freq (MHz) (degrees) Cycle (%) Jitter (ps)  Error (ps)"
------------------------------------------------------------------------------
-- CLK_OUT1___100.000______0.000______50.0______400.000____150.000
-- CLK_OUT2___160.000_____90.000______50.0______300.000____150.000
-- CLK_OUT3___160.000____270.000______50.0______300.000____150.000
-- CLK_OUT4____80.000______0.000______50.0______300.000____150.000
--
------------------------------------------------------------------------------
-- "Input Clock   Freq (MHz)    Input Jitter (UI)"
------------------------------------------------------------------------------
-- __primary_____________160____________0.010

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.all;

entity SysPll2 is
port
 (-- Clock in ports
  CLK_IN1_P         : in     std_logic;
  CLK_IN1_N         : in     std_logic;
  -- Clock out ports
  CLK_OUT1          : out    std_logic;
  CLK_OUT2          : out    std_logic;
  CLK_OUT3          : out    std_logic;
  CLK_OUT4          : out    std_logic;
  -- Status and control signals
  RESET             : in     std_logic;
  LOCKED            : out    std_logic
 );
end SysPll2;

architecture xilinx of SysPll2 is
  attribute CORE_GENERATION_INFO : string;
  attribute CORE_GENERATION_INFO of xilinx : architecture is "SysPll2,clk_wiz_v3_6,{component_name=SysPll2,use_phase_alignment=true,use_min_o_jitter=false,use_max_i_jitter=false,use_dyn_phase_shift=false,use_inclk_switchover=false,use_dyn_reconfig=false,feedback_source=FDBK_AUTO,primtype_sel=DCM_SP,num_out_clk=4,clkin1_period=6.250,clkin2_period=6.250,use_power_down=false,use_reset=true,use_locked=true,use_inclk_stopped=false,use_status=false,use_freeze=false,use_clk_valid=false,feedback_type=SINGLE,clock_mgr_type=AUTO,manual_override=true}";
	  -- Input clock buffering / unused connectors
  signal clkin1            : std_logic;
  -- Output clock buffering
  signal clkfb             : std_logic;
  signal clk0              : std_logic;
  signal clk90             : std_logic;
  signal clk270            : std_logic;
  signal clkfx             : std_logic;
  signal clkdv             : std_logic;
  signal clkfbout          : std_logic;
  signal locked_internal   : std_logic;
  signal status_internal   : std_logic_vector(7 downto 0);
begin


  -- Input buffering
  --------------------------------------
  clkin1_buf : IBUFGDS
  port map
   (O  => clkin1,
    I  => CLK_IN1_P,
    IB => CLK_IN1_N);


  -- Clocking primitive
  --------------------------------------
  
  -- Instantiation of the DCM primitive
  --    * Unused inputs are tied off
  --    * Unused outputs are labeled unused
  dcm_sp_inst: DCM_SP
  generic map
   (CLKDV_DIVIDE          => 2.000,
    CLKFX_DIVIDE          => 8,
    CLKFX_MULTIPLY        => 5,
    CLKIN_DIVIDE_BY_2     => FALSE,
    CLKIN_PERIOD          => 6.250,
    CLKOUT_PHASE_SHIFT    => "NONE",
    CLK_FEEDBACK          => "1X",
    DESKEW_ADJUST         => "SOURCE_SYNCHRONOUS",
    PHASE_SHIFT           => 0,
    STARTUP_WAIT          => FALSE)
  port map
   -- Input clock
   (CLKIN                 => clkin1,
    CLKFB                 => clkfb,
    -- Output clocks
    CLK0                  => clk0,
    CLK90                 => clk90,
    CLK180                => open,
    CLK270                => clk270,
    CLK2X                 => open,
    CLK2X180              => open,
    CLKFX                 => clkfx,
    CLKFX180              => open,
    CLKDV                 => clkdv,
   -- Ports for dynamic phase shift
    PSCLK                 => '0',
    PSEN                  => '0',
    PSINCDEC              => '0',
    PSDONE                => open,
   -- Other control and status signals
    LOCKED                => locked_internal,
    STATUS                => status_internal,
    RST                   => RESET,
   -- Unused pin, tie low
    DSSEN                 => '0');

  LOCKED                <= locked_internal;



  -- Output buffering
  -------------------------------------
  clkf_buf : BUFG
  port map
   (O => clkfb,
    I => clk0);


  clkout1_buf : BUFG
  port map
   (O   => CLK_OUT1,
    I   => clkfx);



  clkout2_buf : BUFG
  port map
   (O   => CLK_OUT2,
    I   => clk90);

  clkout3_buf : BUFG
  port map
   (O   => CLK_OUT3,
    I   => clk270);

  clkout4_buf : BUFG
  port map
   (O   => CLK_OUT4,
    I   => clkdv);

end xilinx;

-- file: SerialTx_exdes.vhd
-- (c) Copyright 2009 - 2011 Xilinx, Inc. All rights reserved.
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

------------------------------------------------------------------------------
-- SelectIO wizard example design
------------------------------------------------------------------------------
-- This example design instantiates the IO circuitry
------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_misc.and_reduce;

library unisim;
use unisim.vcomponents.all;

entity SerialTx_exdes is
generic (
  -- width of the data for the system
  sys_w      : integer := 3;
  -- width of the data for the device
  dev_w      : integer := 12
);
port (
  PATTERN_COMPLETED_OUT     : out   std_logic_vector (1 downto 0);
  -- From the system into the device
  DATA_IN_FROM_PINS_P      : in    std_logic_vector(sys_w-1 downto 0);
  DATA_IN_FROM_PINS_N      : in    std_logic_vector(sys_w-1 downto 0);
  DATA_OUT_TO_PINS_P         : out   std_logic_vector(sys_w-1 downto 0);
  DATA_OUT_TO_PINS_N         : out   std_logic_vector(sys_w-1 downto 0);
  CLK_TO_PINS_FWD_P         : out std_logic;
  CLK_TO_PINS_FWD_N         : out std_logic;

  CLK_IN_P                 : in    std_logic;
  CLK_IN_N                 : in    std_logic;
  CLK_IN_FWD_P             : in    std_logic;
  CLK_IN_FWD_N             : in    std_logic;
  CLK_RESET                : in    std_logic;
  IO_RESET                 : in    std_logic);
end SerialTx_exdes;

architecture xilinx of SerialTx_exdes is

component SerialTx is
generic
 (-- width of the data for the system
  sys_w       : integer := 3;
  -- width of the data for the device
  dev_w       : integer := 12);
port
 (
  -- From the device out to the system
  DATA_OUT_FROM_DEVICE    : in    std_logic_vector(dev_w-1 downto 0);
  DATA_OUT_TO_PINS_P      : out   std_logic_vector(sys_w-1 downto 0);
  DATA_OUT_TO_PINS_N      : out   std_logic_vector(sys_w-1 downto 0);

-- Clock and reset signals
  CLK_IN_P                : in    std_logic;                    -- Differential fast clock from IOB
  CLK_IN_N                : in    std_logic;
  CLK_DIV_OUT             : out   std_logic;                    -- Slow clock output
  IO_RESET                : in    std_logic);                   -- Reset signal for IO circuit
end component;

   constant num_serial_bits  : integer := dev_w/sys_w;
   signal unused             : std_logic;
   signal clkin1             : std_logic;
   signal count_out          : std_logic_vector (num_serial_bits-1 downto 0);
   signal local_counter      : std_logic_vector(num_serial_bits-1 downto 0);
   signal count_out1         : std_logic_vector (num_serial_bits-1 downto 0);
   signal count_out2         : std_logic_vector (num_serial_bits-1 downto 0);
   signal pat_out            : std_logic_vector (num_serial_bits-1 downto 0);
   signal pattern_completed    : std_logic_vector (1 downto 0) := "00";
   signal clk_in_int_inv       : std_logic;
   signal clk_in_int_inv_c     : std_logic;
   signal clk_div_int               : std_logic;
   signal clk_in_int_buf            : std_logic;
            -- connection between ram and io circuit
   signal data_in_to_device         : std_logic_vector(dev_w-1 downto 0);
   signal data_in_to_device_int2    : std_logic_vector(dev_w-1 downto 0);
   signal data_in_to_device_int3    : std_logic_vector(dev_w-1 downto 0);

   signal data_out_from_device : std_logic_vector(dev_w-1 downto 0);

    type serdarr is array (0 to 7) of std_logic_vector(sys_w-1 downto 0);
   signal serdesstrobe             : std_logic;
   signal iserdes_q                : serdarr := (( others => (others => '0')));
   signal icascade                 : std_logic_vector(sys_w-1 downto 0);

   signal data_out_from_device_q    : std_logic_vector(dev_w-1 downto 0) ;
   signal data_in_from_pins_int     : std_logic_vector(sys_w-1 downto 0);
   signal data_in_to_device_int     : std_logic_vector(dev_w-1 downto 0);
   signal tristate_predelay         : std_logic_vector(sys_w-1 downto 0);
   signal data_out_to_pins_int      : std_logic_vector(sys_w-1 downto 0);
   signal data_out_to_pins_predelay : std_logic_vector(sys_w-1 downto 0);
   constant clock_enable            : std_logic := '1';

   signal clk_div_out          : std_logic;
   signal clkfbout             : std_logic;
   signal clkfbout_buf         : std_logic;
   signal clk_in_pll           : std_logic;
   signal clk_in_pll1          : std_logic;
   signal locked_in            : std_logic;
   signal locked_out           : std_logic;
   signal clk_div_in_int       : std_logic;
   signal clk_div_in           : std_logic;
   signal clk_fwd_out          : std_logic;
   signal clk_fwd_int          : std_logic;
   signal clk_fwd_int_buf      : std_logic;
   signal serdesstrobe1        : std_logic;
   signal clk_in_int_buf_fwd   : std_logic;
   signal clk_div_fwd_int      : std_logic;
   signal clk_div_fwd          : std_logic;
   signal rst_sync      : std_logic;
   signal rst_sync_int  : std_logic;
   signal rst_sync_int1 : std_logic;
   signal rst_sync_int2 : std_logic;
   signal rst_sync_int3 : std_logic;
   signal rst_sync_int4 : std_logic;
   signal rst_sync_int5 : std_logic;
   signal rst_sync_int6 : std_logic;
   signal rst_sync_d      : std_logic;
   signal rst_sync_int_d  : std_logic;
   signal rst_sync_int1_d : std_logic;
   signal rst_sync_int2_d : std_logic;
   signal rst_sync_int3_d : std_logic;
   signal rst_sync_int4_d : std_logic;
   signal rst_sync_int5_d : std_logic;
   signal rst_sync_int6_d : std_logic;
   signal bitslip       : std_logic := '0';
   signal bitslip_int   : std_logic := '0';
   signal equal         : std_logic := '0';
   signal equal1        : std_logic := '0';
   signal count_out3    : std_logic_vector(2 downto 0);
   signal start_count   : std_logic := '0';
   signal start_check   : std_logic := '0';
   signal bit_count     : std_logic_vector (2 downto 0);
   type delay_arr is array (0 to sys_w -1) of std_logic_vector(num_serial_bits-1 downto 0);
   signal data_delay_int1 : delay_arr;
   signal data_delay_int2 : delay_arr;
   signal data_delay     : delay_arr; 
   signal slave_shiftout          : std_logic_vector(sys_w-1 downto 0);

   attribute KEEP : string;
   attribute KEEP of clk_div_in_int : signal is "TRUE";
   attribute KEEP of clk_div_out : signal is "TRUE";



begin

   process (clk_div_out, IO_RESET) begin
     if (IO_RESET = '1') then
       rst_sync <= '1';
       rst_sync_int <= '1';
       rst_sync_int1 <= '1';
       rst_sync_int2 <= '1';
       rst_sync_int3 <= '1';
       rst_sync_int4 <= '1';
       rst_sync_int5 <= '1';
       rst_sync_int6 <= '1';
     elsif (clk_div_out = '1' and clk_div_out'event) then
       rst_sync <= '0';
       rst_sync_int <= rst_sync;
       rst_sync_int1 <= rst_sync_int;
       rst_sync_int2 <= rst_sync_int1;
       rst_sync_int3 <= rst_sync_int2;
       rst_sync_int4 <= rst_sync_int3;
       rst_sync_int5 <= rst_sync_int4;
       rst_sync_int6 <= rst_sync_int5;
     end if;
   end process;

   process (clk_div_in, IO_RESET) begin
     if (IO_RESET = '1') then
       rst_sync_d <= '1';
       rst_sync_int_d <= '1';
       rst_sync_int1_d <= '1';
       rst_sync_int2_d <= '1';
       rst_sync_int3_d <= '1';
       rst_sync_int4_d <= '1';
       rst_sync_int5_d <= '1';
       rst_sync_int6_d <= '1';
     elsif (clk_div_in = '1' and clk_div_in'event) then
       rst_sync_d <= '0';
       rst_sync_int_d <= rst_sync_d;
       rst_sync_int1_d <= rst_sync_int_d;
       rst_sync_int2_d <= rst_sync_int1_d;
       rst_sync_int3_d <= rst_sync_int2_d;
       rst_sync_int4_d <= rst_sync_int3_d;
       rst_sync_int5_d <= rst_sync_int4_d;
       rst_sync_int6_d <= rst_sync_int5_d;
     end if;
   end process;


   clkin_in_buf : IBUFGDS
   port map
     (O  => clkin1,
      I  => CLK_IN_P,
      IB => CLK_IN_N);

   -- set up the fabric PLL_BASE to drive the BUFPLL
   pll_base_inst : PLL_BASE
    generic map (
      BANDWIDTH             => "OPTIMIZED",
      CLK_FEEDBACK          => "CLKFBOUT",
      COMPENSATION          => "SYSTEM_SYNCHRONOUS",
      DIVCLK_DIVIDE         => 1,
      CLKFBOUT_MULT         => 8,
      CLKFBOUT_PHASE        => 0.000,
      CLKOUT0_DIVIDE        => 4,
      CLKOUT0_PHASE         => 0.000,
      CLKOUT0_DUTY_CYCLE    => 0.500,
      CLKOUT1_DIVIDE        => 4,
      CLKOUT1_PHASE         => 0.000,
      CLKOUT1_DUTY_CYCLE    => 0.500,
      CLKOUT2_DIVIDE        => 4*num_serial_bits,
      CLKOUT2_PHASE         => 0.000,
      CLKOUT2_DUTY_CYCLE    => 0.500,
      CLKOUT3_DIVIDE        => 8,
      CLKOUT3_PHASE         => 0.000,
      CLKOUT3_DUTY_CYCLE    => 0.500,
      CLKIN_PERIOD          => 10.0,
      REF_JITTER            => 0.010)
   port map (
     -- Output clocks
      CLKFBOUT              => clkfbout,
      CLKOUT0               => clk_in_pll1,
      CLKOUT1               => clk_fwd_int,
      CLKOUT2               => clk_div_in_int,
      CLKOUT3               => clk_div_fwd_int,
      CLKOUT4               => open,
      CLKOUT5               => open,
      -- Status and control signals
      LOCKED                => locked_in,
      RST                   => CLK_RESET,
      -- Input clock control
      CLKFBIN               => clkfbout_buf,
      CLKIN                 => clkin1);

   clkfb_buf : BUFG
    port map (
      O            => clkfbout_buf,
      I            => clkfbout);

   clkd_buf : BUFG
    port map (
      O            => clk_div_in,
      I            => clk_div_in_int);

   clk_fwd_buf : BUFG
    port map (
      O            => clk_div_fwd,
      I            => clk_div_fwd_int);

   clko_buf : BUFG
    port map (
      O            => clk_in_pll,
      I            => clk_in_pll1);



   process(clk_div_in) begin
   if (clk_div_in='1' and clk_div_in'event) then
     if (rst_sync_int6 = '1') then
       equal1 <= '0';
     else
       if (count_out3 = "100") then
          equal1 <= equal;
       else
          equal1 <= equal1;
       end if;
     end if;
    end if;
   end process;


   process(clk_div_in) begin
   if (clk_div_in='1' and clk_div_in'event) then
     if (rst_sync_int6 = '1') then
       count_out3 <= (others => '0');
     elsif (equal = '1' and count_out3 < "100" ) then
       count_out3 <= count_out3 + 1;
     else
       count_out3 <= (others => '0');
     end if;
    end if;
   end process;

   process(clk_div_in) begin
   if (clk_div_in='1' and clk_div_in'event) then
     if (rst_sync_int6 = '1') then
       count_out1 <= (others => '0');
       pat_out <= "1101";
       count_out1 <= (others => '0');
     elsif locked_in='1' then  
     if equal1='0' then
      pat_out <= "1101";
       count_out1 <= (others => '0');
    else
       count_out1 <= count_out1 + 1;
     end if;
    end if;
   end if;
  end process;

   process(clk_div_out) begin
   if (clk_div_out='1' and clk_div_out'event) then
     if (rst_sync_int6 = '1') then
       count_out2 <= (others => '0');
     elsif equal1='1' then
       count_out2 <= count_out1;
     else
       count_out2 <= pat_out;
     end if;
    end if;
   end process;   

   process(clk_div_out) begin
   if (clk_div_out='1' and clk_div_out'event) then
     if (rst_sync_int6 = '1') then
       count_out <= (others => '0');
     else
       count_out <= count_out2;
     end if;
    end if;
   end process; 
   


assign:for assg in 0 to num_serial_bits-1 generate begin
pinsss:for pinsss in 0 to sys_w-1 generate begin
   data_out_from_device(pinsss+sys_w*assg) <= count_out(assg);
end generate pinsss;
end generate assign;

   data_delay(0) <=                 data_in_to_device(9) &
                data_in_to_device(6) &
                data_in_to_device(3) &
   data_in_to_device(0);
   data_delay(1) <=                 data_in_to_device(10) &
                data_in_to_device(7) &
                data_in_to_device(4) &
   data_in_to_device(1);
   data_delay(2) <=                 data_in_to_device(11) &
                data_in_to_device(8) &
                data_in_to_device(5) &
   data_in_to_device(2);

   process (clk_div_in) begin
   if (clk_div_in='1' and clk_div_in'event) then
     if (rst_sync_int6_d = '1') then
       start_check <= '0';
     else
       if (data_delay(0) /= "0000") then
 
         start_check <= '1';
       end if;
     end if;
    end if;
   end process;

   process (clk_div_in) begin
   if (clk_div_in='1' and clk_div_in'event) then
     if (rst_sync_int6_d = '1') then
       start_count <= '0';
     else
       if (data_delay(0) = "0001" and equal = '1') then
 
         start_count <= '1';
       end if;
     end if;
    end if;
   end process;

   process (clk_div_in) begin
   if (clk_div_in='1' and clk_div_in'event) then    
     if (rst_sync_int6_d = '1') then
       local_counter <= (others =>'0');
     else
       if start_count = '1' then
         local_counter <= local_counter + 1;
       else
         local_counter <= (others =>'0');
       end if;
     end if;
    end if;
   end process;

   process (clk_div_in) begin
   if (clk_div_in='1' and clk_div_in'event) then
     if (rst_sync_int6_d = '1') then
       data_delay_int1(0) <= (others => '0');
       data_delay_int2(0) <= (others => '0');
       data_delay_int1(1) <= (others => '0');
       data_delay_int2(1) <= (others => '0');
       data_delay_int1(2) <= (others => '0');
       data_delay_int2(2) <= (others => '0');
     else
       data_delay_int1(0) <= data_delay(0);
       data_delay_int2(0) <= data_delay_int1(0);
       data_delay_int1(1) <= data_delay(1);
       data_delay_int2(1) <= data_delay_int1(1);
       data_delay_int1(2) <= data_delay(2);
       data_delay_int2(2) <= data_delay_int1(2);
     end if;
   end if;
   end process;

   process (clk_div_in) begin
   if (clk_div_in='1' and clk_div_in'event) then
     if (rst_sync_int6_d = '1') then
       bitslip_int <= '0';
       equal <= '0';
     else
      if (equal = '0' and locked_in = '1' and start_check = '1') then
        if (
      (data_delay(2) = pat_out) and
      (data_delay(1) = pat_out) and
      (data_delay(0) = pat_out)) then 
          bitslip_int <= '0';
          equal <= '1';
        else
          bitslip_int <= '1';
          equal <= '0';
        end if;
      else
        bitslip_int <= '0';
      end if;
     end if;
    end if;
   end process;

   process (clk_div_in) begin
   if (clk_div_in='1' and clk_div_in'event) then
     if (rst_sync_int6_d = '1') then
       bitslip <= '0';
       bit_count <= "000";
     else
       bit_count <= bit_count + '1';
         if bit_count = "111" then
           if bitslip_int='1' then
             bitslip <= not(bitslip);
           else
             bitslip <= '0';
           end if;
         else
           bitslip <= '0';
         end if;
      end if;
     end if;
   end process;

   process (clk_div_in) begin
   if (clk_div_in='1' and clk_div_in'event) then
     if equal = '1' then
      if (
        (data_delay_int2(1) = local_counter) and
        (data_delay_int2(2) = local_counter) and
        (data_delay_int2(0) = local_counter)) then
        if (local_counter = "1111") then
          pattern_completed <= "11";
        -- all over
        else
          pattern_completed <= "01";
          -- bitslip done, data checking in progress
        end if;
     else
          if (start_count = '1') then
             pattern_completed <= "10";
         -- incorrect data
          else
             pattern_completed <= pattern_completed;
          end if;
     end if;
   else
          pattern_completed <= "00";
         -- yet to get bitslip
   end if;
  end if;
 end process;



 
   PATTERN_COMPLETED_OUT <= pattern_completed;
  


   bufpll_inst : BUFPLL
    generic map (
      DIVIDE        => 4)
    port map (
      IOCLK        => clk_in_int_buf,
      LOCK         => locked_out,
      SERDESSTROBE => serdesstrobe,
      GCLK         => clk_div_in,  -- GCLK pin must be driven by BUFG
      LOCKED       => locked_in,
      PLLIN        => clk_in_pll1);




     clk_in_int_inv <= not(clk_in_pll);

  pins: for pin_count in 0 to sys_w-1 generate
    -- Instantiate the buffers
    ----------------------------------
     ibufds_inst : IBUFDS
       generic map (
         DIFF_TERM  => FALSE,             -- Differential termination
         IOSTANDARD => "LVDS_25")
       port map (
         I          => DATA_IN_FROM_PINS_P  (pin_count),
         IB         => DATA_IN_FROM_PINS_N  (pin_count),
         O          => data_in_from_pins_int(pin_count));

     -- Instantiate the serdes primitive
     ----------------------------------
     -- declare the iserdes
     iserdes2_master : ISERDES2
       generic map (
         BITSLIP_ENABLE =>  TRUE,
         DATA_RATE      => "SDR",
         DATA_WIDTH     => 4,
         INTERFACE_TYPE => "RETIMED",
         SERDES_MODE    => "NONE")
       port map (
         Q1         => iserdes_q(3)(pin_count),
         Q2         => iserdes_q(2)(pin_count),
         Q3         => iserdes_q(1)(pin_count),
         Q4         => iserdes_q(0)(pin_count),
         SHIFTOUT   => open,
         INCDEC     => open,
         VALID      => open,
         BITSLIP    => bitslip,
         CE0        => clock_enable,   -- 1-bit Clock enable input
         CLK0       => clk_in_int_buf, -- 1-bit IO Clock network input. Optionally Invertible. This is the primary clock
                                       -- input used when the clock doubler circuit is not engaged (see DATA_RATE
                                       -- attribute).
         CLK1       => '0',
         CLKDIV     => clk_div_in,
         D          => data_in_from_pins_int(pin_count), -- 1-bit Input signal from IOB.
         IOCE       => serdesstrobe,                       -- 1-bit Data strobe signal derived from BUFIO CE. Strobes data capture for
                                                          -- NETWORKING and NETWORKING_PIPELINES alignment modes.

         RST        => IO_RESET,        -- 1-bit Asynchronous reset only.
         SHIFTIN    => '0',


        -- unused connections
         FABRICOUT  => open,
         CFB0       => open,
         CFB1       => open,
         DFB        => open);





     -- Concatenate the serdes outputs together. Keep the timesliced
     --   bits together, and placing the earliest bits on the right
     --   ie, if data comes in 0, 1, 2, 3, 4, 5, 6, 7, ...
     --       the output will be 3210, 7654, ...
     -------------------------------------------------------------
     in_slices: for slice_count in 0 to num_serial_bits-1 generate begin
        -- This places the first data in time on the right
        data_in_to_device(slice_count*sys_w+sys_w-1 downto slice_count*sys_w) <=
          iserdes_q(num_serial_bits-slice_count-1);
        -- To place the first data in time on the left, use the
        --   following code, instead
        -- data_in_to_device2(slice_count*sys_w+sys_w-1 downto sys_w) <=
        --   iserdes_q(slice_count);
     end generate in_slices;
  end generate pins;

  bufpll_inst_fwd : BUFPLL
     generic map (
       DIVIDE        => 2)
     port map (
       IOCLK        => clk_in_int_buf_fwd,
       LOCK         => locked_out,
       SERDESSTROBE => serdesstrobe1,
       GCLK         => clk_div_fwd,
       LOCKED       => locked_in,
       PLLIN        => clk_fwd_int);

     oserdes2_fwd : OSERDES2
     generic map (
         DATA_RATE_OQ   => "SDR",
         DATA_RATE_OT   => "SDR",
	 TRAIN_PATTERN  => 0,
         DATA_WIDTH     => 4,	
         SERDES_MODE    => "NONE",
	 OUTPUT_MODE    => "SINGLE_ENDED")
     port map (
         D1             => '1',
         D2             => '0',
         D3             => '1',
         D4             => '0',
         T1             => '0',
         T2             => '0',
         T3             => '0',
         T4             => '0',
         SHIFTIN1       => '1',
         SHIFTIN2       => '1',
         SHIFTIN3       => '1',
         SHIFTIN4       => '1',
         SHIFTOUT1      => open,
         SHIFTOUT2      => open,
         SHIFTOUT3      => open,
         SHIFTOUT4      => open,
	 TRAIN          => '0',
	 OCE		=> locked_in,
	 CLK0		=> clk_in_int_buf_fwd,
	 CLK1		=> '0',
	 CLKDIV		=> clk_div_fwd,
 	 OQ		=> clk_fwd_out,
         TQ		=> open,
	 IOCE		=> serdesstrobe1,
	 TCE		=> clock_enable,
         RST            => IO_RESET);

          obufds_clk_inst : OBUFDS
           generic map (
             IOSTANDARD => "LVDS_25")
           port map (
             O          => CLK_TO_PINS_FWD_P,
             OB         => CLK_TO_PINS_FWD_N,
             I          => clk_fwd_out);

   -- Instantiate the IO design
   io_inst : SerialTx
   port map
   (
    -- From the drive out to the system
    DATA_OUT_FROM_DEVICE    => data_out_from_device,
    DATA_OUT_TO_PINS_P      => DATA_OUT_TO_PINS_P,
    DATA_OUT_TO_PINS_N      => DATA_OUT_TO_PINS_N,


    CLK_IN_P                => CLK_IN_FWD_P,
    CLK_IN_N                => CLK_IN_FWD_N,
    CLK_DIV_OUT             => clk_div_out,
    IO_RESET                => rst_sync_int);
end xilinx;

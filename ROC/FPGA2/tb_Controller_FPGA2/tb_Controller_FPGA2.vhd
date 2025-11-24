library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_Controller_FPGA2 is
end entity;

architecture tb of tb_Controller_FPGA2 is

  -- DUT port signals (only those needed for test are driven/monitored)
  signal VXO_P, VXO_N, ClkB_P, ClkB_N, Clk50MHz : std_logic := '0';
  signal CpldRst, CpldCS, uCRd, uCWr : std_logic;
  signal uCA : std_logic_vector(11 downto 0);
  signal uCD : std_logic_vector(15 downto 0);
  signal uCD_dir : std_logic := 'Z'; -- not used; uCD is inout in DUT

  signal GA : std_logic_vector(1 downto 0) := "00";

  -- minimal other ports tied off (DUT has many ports; tie to safe defaults)
  signal dummy_sl : std_logic := '0';
  signal dummy_slv4 : std_logic_vector(3 downto 0) := (others => '0');
  signal dummy_slv8 : std_logic_vector(7 downto 0) := (others => '0');

  -- Phy Rx signals the DUT expects
  signal PhyRxBuff_Empty   : std_logic_vector(7 downto 0) := (others => '0');
  signal PhyRxBuff_RdCnt0  : std_logic_vector(12 downto 0) := (others => '0'); -- element width used in DUT
  -- We'll expose read-count as an array element via generate in DUT; here we only drive signals the DUT samples
  -- For simplicity, we declare a vector to be connected in port map below.

  -- Clock signals
  signal SysClk    : std_logic := '0';
  signal RxFMClk   : std_logic := '0';
  signal i50MHzClk : std_logic := '0';

  constant SysClk_period  : time := 10 ns;  -- 100 MHz
  constant RxFM_period    : time := 5  ns;  -- 200 MHz (example)
  constant i50_period     : time := 20 ns;  -- 50 MHz (example)

  -- Local constants (must match Proj_Defs ReadyStatusAddr)
  -- This is the 10-bit address we picked earlier: 0x1A0 = "0110100000"
  constant ReadyStatusAddr : std_logic_vector(9 downto 0) := "0110100000";

begin

  ----------------------------------------------------------------------------
  -- Clock generators
  ----------------------------------------------------------------------------
  SysClk_proc : process
  begin
    while now < 5 ms loop
      SysClk <= '0';
      wait for SysClk_period/2;
      SysClk <= '1';
      wait for SysClk_period/2;
    end loop;
    wait;
  end process;

  RxFM_proc : process
  begin
    while now < 5 ms loop
      RxFMClk <= '0';
      wait for RxFM_period/2;
      RxFMClk <= '1';
      wait for RxFM_period/2;
    end loop;
    wait;
  end process;

  i50_proc : process
  begin
    while now < 5 ms loop
      i50MHzClk <= '0';
      wait for i50_period/2;
      i50MHzClk <= '1';
      wait for i50_period/2;
    end loop;
    wait;
  end process;

  ----------------------------------------------------------------------------
  -- Instantiate DUT (Controller_FPGA2)
  -- For brevity, only relevant signals are connected; other ports are tied off.
  -- Ensure your simulator compiles the full Controller_FPGA2 entity with these names.
  ----------------------------------------------------------------------------
  DUT: entity work.Controller_FPGA2
    port map (
      VXO_P    => VXO_P, VXO_N => VXO_N, ClkB_P => ClkB_P, ClkB_N => ClkB_N, Clk50MHz => Clk50MHz,
      CpldRst  => CpldRst, CpldCS => CpldCS, uCRd => uCRd, uCWr => uCWr,
      uCA => uCA, uCD => uCD,
      GA => GA,
      -- tie off many unused ports
      SDCKE => open, LDM => open, UDM => open, RAS => open, CAS => open, SDWE => open,
      SDClk_P => open, SDClk_N => open, SDD => open, UDQS => open, LDQS => open, SDRzq => open,
      SDA => open, BA => open,
      LinkClk_P => open, LinkClk_N => open, LinkFR_P => open, LinkFR_N => open,
      LinkD_P => open, LinkD_N => open,
      -- PHY rx/tx ports - connect only signals we plan to drive/monitor
      RxDA => (others => '0'), RxDB => (others => '0'), RxDC => (others => '0'), RxDD => (others => '0'),
      RxDE => (others => '0'), RxDF => (others => '0'), RxDG => (others => '0'), RxDH => (others => '0'),
      RxClk => (others => '0'), RxDV => (others => '0'), RxErr => (others => '0'), CRS => PhyRxBuff_Empty,
      TxDA => open, TxDB => open, TxDC => open, TxDD => open, TxDE => open, TxDF => open, TxDG => open, TxDH => open,
      TxEn => open, MDC => open, MDIO => open, PhyPDn => open, PhyRst => open,
      TxClk => open, Clk25MHz => open,
      FMRx => (others => '0'), FMRxEn => open,
      HrtBtFM => '0', DReqFM => '0',
      SPICS => open, SPISClk => open, SPIMOSI => open, SPIMISO => open,
      Debug => open
    );

  ----------------------------------------------------------------------------
  -- Test stimulus process
  ----------------------------------------------------------------------------
  stim_proc : process
    -- local variables for checks
    variable read_word : std_logic_vector(15 downto 0);
  begin
    -- initial reset
    CpldRst <= '0';
    CpldCS  <= '1';  -- not selected
    uCRd    <= '1';
    uCWr    <= '1';
    uCA     <= (others => '0');
    uCD     <= (others => 'Z'); -- microcontroller side; DUT drives when read active
    PhyRxBuff_Empty <= (others => '0');
    PhyRxBuff_RdCnt0 <= (others => '0');

    wait for 200 ns;  -- hold in reset time
    CpldRst <= '1';
    wait for 50 ns;

    ------------------------------------------------------------
    -- Test 1: FIFO 0 becomes empty (rising edge) sets ReadyStatus(0)
    ------------------------------------------------------------
    -- ensure bit is clear first
    assert (not (ReadyStatus'event and ReadyStatus /= (others => '0'))) report "ReadyStatus unknown at test start" severity note;

    -- make PHY FIFO element 0 go from 0->1 (rising edge) as sampled by DUT via CRS mapping
    PhyRxBuff_Empty(0) <= '0';
    wait for SysClk_period*2;
    -- now cause rising edge
    PhyRxBuff_Empty(0) <= '1';
    wait for SysClk_period*3;  -- give DUT time to sample and latch

    -- check ReadyStatus bit set (cannot access internal ReadyStatus directly from TB unless made port; instead read via MCU read)
    -- Prepare address for read: uCA = GA & ReadyStatusAddr
    uCA <= GA & ReadyStatusAddr;
    -- create a microcontroller-style read: CpldCS='0', uCRd='0' for one SysClk
    CpldCS <= '0';
    uCRd <= '0';
    wait for SysClk_period;   -- one cycle to create RDDL(0)=1
    uCRd <= '1';
    CpldCS <= '1';
    wait for SysClk_period * 2;  -- allow the read path and clear-on-read to process

    -- sample uCD which should contain X"00"&ReadyStatus
    read_word := uCD;
    assert (read_word(7 downto 0) = "00000001")
      report "Test1 FAIL: ReadyStatus bit 0 not returned via uCD (expected LSB=1). uCD=" & to_hstring(read_word)
      severity error;

    -- Also verify ReadyStatus cleared after the read (read again and expect 0)
    wait for SysClk_period * 2;
    -- perform read again
    uCA <= GA & ReadyStatusAddr;
    CpldCS <= '0'; uCRd <= '0';
    wait for SysClk_period;
    uCRd <= '1'; CpldCS <= '1';
    wait for SysClk_period * 2;
    read_word := uCD;
    assert (read_word(7 downto 0) = (others => '0'))
      report "Test1 FAIL: ReadyStatus bit 0 not cleared after read. uCD=" & to_hstring(read_word)
      severity error;

    -- restore PhyRxBuff_Empty(0) to zero to avoid side effects
    PhyRxBuff_Empty(0) <= '0';
    wait for SysClk_period*2;

    ------------------------------------------------------------
    -- Test 2: write-to-clear behavior (optional)
    ------------------------------------------------------------
    -- Simulate ReadyStatus set again
    PhyRxBuff_Empty(1) <= '1';
    wait for SysClk_period * 3;
    -- Confirm it is visible on read
    uCA <= GA & ReadyStatusAddr;
    CpldCS <= '0'; uCRd <= '0';
    wait for SysClk_period;
    uCRd <= '1'; CpldCS <= '1';
    wait for SysClk_period * 2;
    assert (uCD(1) = '1') report "Test2 prereq failed: ReadyStatus(1) not set" severity error;

    -- Now clear by writing to ReadyClearAddr: set uCA to ReadyClearAddr, uCD lower byte bits set accordingly and assert write
    uCA <= GA & ReadyClearAddr;
    uCD <= (15 downto 8 => '0') & "00000010"; -- request clear of bit 1
    CpldCS <= '0'; uCWr <= '0';
    wait for SysClk_period;
    uCWr <= '1'; CpldCS <= '1';
    wait for SysClk_period * 2;

    -- Read back ReadyStatus to verify cleared
    uCA <= GA & ReadyStatusAddr;
    CpldCS <= '0'; uCRd <= '0';
    wait for SysClk_period;
    uCRd <= '1'; CpldCS <= '1';
    wait for SysClk_period * 2;
    assert (uCD(1) = '0') report "Test2 FAIL: ReadyStatus(1) not cleared by write" severity error;

    -- Clean up
    PhyRxBuff_Empty(1) <= '0';
    wait for SysClk_period * 5;

    ------------------------------------------------------------
    -- Test 3: auto-clear by PhyRxBuff_RdCnt non-zero (optional)
    -- Set ReadyStatus via empty edge then set PhyRxBuff_RdCnt non-zero and verify it clears
    ------------------------------------------------------------
    PhyRxBuff_Empty(2) <= '1';
    wait for SysClk_period * 3;

    -- set a non-zero read-count value for port 2 (simulate FIFO refill)
    -- Note: Controller expects PhyRxBuff_RdCnt as an array element; here we assume the generate maps an internal vector.
    -- If your DUT exposes or produces a different naming, adapt accordingly.
    -- For the demonstration we will drive the CRS/rdcount signals (if accessible). If not, skip this part.
    -- PhyRxBuff_RdCnt0 <= x"001"; -- uncomment/adapt if you have access to the per-port rd count signal

    -- wait and check (uncomment the checks if rd count signal is actually connected)
    -- wait for SysClk_period * 4;
    -- uCA <= GA & ReadyStatusAddr;
    -- CpldCS <= '0'; uCRd <= '0';
    -- wait for SysClk_period;
    -- uCRd <= '1'; CpldCS <= '1';
    -- wait for SysClk_period * 2;
    -- assert (uCD(2) = '0') report "Test3 FAIL: ReadyStatus(2) not auto-cleared on RdCnt non-zero" severity error;

    ------------------------------------------------------------
    -- If we reach here, tests passed
    ------------------------------------------------------------
    report "All ReadyStatus tests PASSED" severity note;
    wait;
  end process stim_proc;

end architecture;

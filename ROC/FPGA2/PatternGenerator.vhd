LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;


entity PatternGenerator is
    Port ( clk    : in  std_logic; -- RxFMClk, 200MHz, a new word is needed every 32 cycles only
           rst    : in  std_logic; -- active high
           start  : in  std_logic;
           nwords : in  std_logic_vector(7 downto 0);
           data   : out std_logic_vector(15 downto 0);
           valid  : out std_logic);
end PatternGenerator;

architecture Behavioral of PatternGenerator is
    type state_type is (IDLE, WORD_COUNT, UB1, UB2, STAT, GENPATTERN);
    signal state, next_state : state_type;
    signal uB : std_logic_vector(31 downto 0);
    signal pattern : std_logic_vector(15 downto 0);
    signal data_out : std_logic_vector(15 downto 0);
    signal wdCnt : std_logic_vector(7 downto 0);
    signal wren : std_logic;
	 signal wren_fast  : std_logic_vector(31 downto 0);
	 signal start_long : std_logic_vector(31 downto 0);
	 
	 -- Clock division 
    signal slow_clk : std_logic;
    signal clk_divider : std_logic_vector(3 downto 0); -- 5 bits for divide by 32


begin
    data <= data_out;
    valid <= wren_fast(0);

    -- Clock divider
    process(clk, rst)
    begin
        if rst = '1' then
            clk_divider <= (others => '0');
            slow_clk <= '0';
        elsif rising_edge(clk) then
            if clk_divider = "1111" then
                clk_divider <= (others => '0');
                slow_clk <= not slow_clk;
            else
                clk_divider <= clk_divider + 1;
            end if;
        end if;
    end process;


    -- Combinatorial process
    process(state, start, wdCnt)
    begin
        -- Default assignments
        Case state is
            when IDLE =>
                if start_long /= 0 then next_state <= WORD_COUNT;
                else                    next_state <= IDLE;
                end if;

            when WORD_COUNT =>          next_state <= UB1;
            when UB1 =>                 next_state <= UB2;
            when UB2 =>                 next_state <= STAT;
            when STAT =>               
				    if wdCnt /= 0 then      next_state <= GENPATTERN;
					 else                    next_state <= IDLE;
					 end if;

            when GENPATTERN =>
                if wdCnt /= 0 then      next_state <= GENPATTERN;
                else                    next_state <= IDLE;
                end if;

            when others =>              next_state <= IDLE;
        end Case;
    end process;

    -- Sequential process, slow
    process(slow_clk, rst)
    begin
        if rst = '1' then
            state <= IDLE;
            uB <= (others => '0');
            wdCnt <= (others => '0');
            pattern <= (others => '0');
            data_out <= (others => '0');
            wren <= '0';
        elsif rising_edge(slow_clk) then
            state <= next_state;

            Case state is 
                when WORD_COUNT => data_out <= X"00" & nwords + 4;
                when UB1 =>        data_out <= uB(31 downto 16);
                when UB2 =>        data_out <= uB(15 downto  0);
                when STAT =>       data_out <= X"ff00";
                when GENPATTERN => data_out <= pattern;
                when others =>     data_out <= (others => '0');
            end Case;

            if state = WORD_COUNT or
               state = UB1 or
               state = UB2 or
               state = STAT or
               state = GENPATTERN then wren <= '1';
            else                       wren <= '0';
            end if;

            if state = STAT then uB <= uB + 1;
            else                 uB <= uB;
            end if;


            if state = WORD_COUNT then    wdCnt <= nwords;
				elsif state = GENPATTERN or
                  state = STAT then       wdCnt <= wdCnt - 1;
            else                          wdCnt <= wdCnt;
            end if;

            if state = GENPATTERN then pattern <= pattern + 1;
            else                       pattern <= pattern;
            end if;
        end if;
    end process;
	 
	 -- fast clock for shorter wren
	 process(clk, rst)
        begin
            if rst = '1' then
																		wren_fast  <= (others => '0');
																		start_long <= (others => '0');
				elsif rising_edge(clk) then
				   start_long(31 downto 0) <= start_long(30 downto 0) & start;
					wren_fast(31 downto 1) <= wren_fast(30 downto 0);
					if wren = '1' and wren_fast = 0 then   wren_fast(0) <= '1';
					else                                   wren_fast(0) <= '0';
					end if;
				end if;
	 end process; 
end Behavioral;

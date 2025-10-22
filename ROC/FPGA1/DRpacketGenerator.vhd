library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity DRpacketGenerator is
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;
        
        -- Trigger interface
        trigger     : in  std_logic;
        busy        : out std_logic;
        
        -- Input data
        id          : in  std_logic_vector(3 downto 0);
        UB_LOW      : in  std_logic_vector(15 downto 0);
        UB_MID      : in  std_logic_vector(15 downto 0);
        UB_HIGH     : in  std_logic_vector(15 downto 0);
        
        -- Output to FIFO
        wr_en       : out std_logic;
		  ts_wr_en    : out std_logic;
        wr_data     : out std_logic_vector(15 downto 0)
    );
end DRpacketGenerator;

architecture behavioral of DRpacketGenerator is
    type packet_state_t is (IDLE, WRITE_COUNT, WRITE_TYPE, WRITE_UB_LOW, WRITE_UB_MID, WRITE_UB_HIGH, WRITE_RES_1, WRITE_RES_2, WRITE_RES_3, WRITE_CRC);
    signal state : packet_state_t := IDLE;
    
begin
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                state <= IDLE;
                wr_en <= '0';
					 ts_wr_en <= '0';
                wr_data <= (others => '0');
                busy <= '0';
            else
                case state is
                    when IDLE =>
                        wr_en <= '0';
								ts_wr_en <= '0';
                        busy <= '0';
                        if trigger = '1' then
                            state <= WRITE_COUNT;
                            wr_en <= '1';
									 ts_wr_en <= '0';
                            wr_data <= x"0010";  -- 10 in hex
                            busy <= '1';
                        end if;
                        
                    when WRITE_COUNT =>
                        state <= WRITE_TYPE;
                        wr_data <= x"8" & id & x"20";  -- 8 + 0 + id + 20
								ts_wr_en <= '0';
                        
                    when WRITE_TYPE =>
                        state <= WRITE_UB_LOW;
                        wr_data <= UB_LOW;
								ts_wr_en <= '1';
                        
                    when WRITE_UB_LOW =>
                        state <= WRITE_UB_MID;
                        wr_data <= UB_MID;
								ts_wr_en <= '1';
                        
                    when WRITE_UB_MID =>
                        state <= WRITE_UB_HIGH;
                        wr_data <= UB_HIGH;
								ts_wr_en <= '1';
                        
                    when WRITE_UB_HIGH =>
                        state <= WRITE_RES_1;
                        wr_data <= (others => '0');
								ts_wr_en <= '0';
                        
                    when WRITE_RES_1 =>
                        state <= WRITE_RES_2;
                        wr_data <= (others => '0');
								ts_wr_en <= '0';
                        
                    when WRITE_RES_2 =>
                        state <= WRITE_RES_3;
                        wr_data <= (others => '0');
								ts_wr_en <= '0';
                        
                    when WRITE_RES_3 =>
                        state <= WRITE_CRC;
								wr_data <= (others => '1'); -- fake a CRC, its not used
								ts_wr_en <= '0';
								
						when WRITE_CRC =>
                        state <= IDLE;
                        wr_en <= '0';
								ts_wr_en <= '0';
                        busy <= '0';
                end case;
            end if;
        end if;
    end process;
    
end behavioral;
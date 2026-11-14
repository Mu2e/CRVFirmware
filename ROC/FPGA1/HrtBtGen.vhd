LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity HrtBtGen is 
    port(
        clk         : in  std_logic;
        rst_n       : in  std_logic; -- Active low reset
        enable      : in  std_logic;
        period      : in  std_logic_vector(31 downto 0);
        data_out    : out std_logic_vector(23 downto 0);
        tx_en_out   : out std_logic
    );
end HrtBtGen;

architecture behavioural of HrtBtGen is
    signal timer_cnt : std_logic_vector(31 downto 0);
    signal data_cnt  : std_logic_vector(23 downto 0);
begin

    process(clk, rst_n)
    begin
        if rst_n = '0' then
            timer_cnt <= (others => '0');
            data_cnt  <= (others => '0');
            tx_en_out <= '0';
        elsif rising_edge(clk) then
            if enable = '1' then
                if timer_cnt >= period then
                    timer_cnt <= (others => '0');
                    tx_en_out <= '1'; 
                    data_cnt  <= data_cnt + 1;
                else
                    timer_cnt <= timer_cnt + 1;
                    tx_en_out <= '0';
                end if;
            else
                -- Keep idle when disabled
                timer_cnt <= (others => '0');
                tx_en_out <= '0';
                -- data_cnt holds its last value
            end if;
        end if;
    end process;

    data_out <= data_cnt;

end behavioural;
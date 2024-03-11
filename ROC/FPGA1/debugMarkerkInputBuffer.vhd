library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.std_logic_unsigned.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity debugMarkerkInputBuffer is
    Port ( rst      : in  std_logic; -- active height
           clk      : in  std_logic;
			  trig     : in  std_logic;
           data_in  : in  std_logic_vector (1 downto 0);
           rd_clk   : in  std_logic;
           rd_en    : in  std_logic;
           rd_data  : out std_logic_vector (15 downto 0);
			  rd_full  : out std_logic;
			  rd_empty : out std_logic);
end debugMarkerkInputBuffer;


architecture Behavioral of debugMarkerkInputBuffer is

Type SM_States is (Idle,Writing,Holding);
signal SM_State : SM_States;

signal cnt : std_logic_vector(7 downto 0);
signal wr_en : std_logic;
signal full, empty : std_logic;
signal pipeline : std_logic_vector(31 downto 0);

COMPONENT debugMarkerBuffer
  PORT (
    rst    : in std_logic;
    wr_clk : in std_logic;
    rd_clk : in std_logic;
    din    : in std_logic_vector(1 DOWNTO 0);
    wr_en  : in std_logic;
    rd_en  : in std_logic;
    dout   : out std_logic_vector(15 DOWNTO 0);
    full   : out std_logic;
    empty  : out std_logic
  );
END COMPONENT;

begin

SM : process(clk, rst)
   begin
      if rst = '1' then
	      SM_State <= Idle;
			cnt <= (others => '0');
			wr_en <= '0';
			pipeline <= (others => '0');
	 
	   elsif rising_edge(clk) then
		    pipeline(31 downto 2) <= pipeline(29 downto 0);
			 pipeline( 1 downto 0) <= data_in;
		
		    case SM_State is 
		       when Idle =>
				     if trig = '1' and empty = '1' and full = '0' then
					      SM_State <= Writing;
							cnt <= X"FF";
					  else
					      SM_State <= SM_State;
					  end if;
				 when Writing =>
				     if cnt = 0 or full = '1' then
					       SM_State <= Holding;
							 --SM_State <= Idle;
					  else
					       SM_State <= SM_State; 
					  end if;
				 when Holding =>
				     if rst = '1' then SM_State <= Idle;
				 	  else SM_State <= SM_State;
                 end if;					  
			 end case;
			 
			 if (SM_State = Writing) and cnt /= 0 then
			    wr_en <= '1';
			 else
			    wr_en <= '0';
			 end if;
			 
		end if;
end process; 

debugMarkerBufferInst : debugMarkerBuffer
  PORT MAP (
    rst => rst,
    wr_clk => clk,
    rd_clk => rd_clk,
    din => pipeline(31 downto 30),
    wr_en => wr_en,
    rd_en => rd_en,
    dout => rd_data,
    full => full,
    empty => empty
  );
  
  rd_full <= full;
  rd_empty <= empty;

end Behavioral;


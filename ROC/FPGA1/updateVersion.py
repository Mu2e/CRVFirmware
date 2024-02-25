#\

import os
 
os.system('git rev-parse HEAD > git_hash.txt')

with open('git_hash.txt', 'r') as file:
    git_hash = file.read().strip()

with open('git_hash_pkg.vhd', 'w') as file:
    file.write(f"""library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

package git_hash_pkg is
    constant GIT_HASH : std_logic_vector(159 downto 0) := x"{git_hash}";
end package;
""")

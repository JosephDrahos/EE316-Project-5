----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 04/14/2021 11:58:38 PM
-- Design Name: 
-- Module Name: motor_pwm - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity motor_pwm is
    Port ( I_CLK : in STD_LOGIC;
           I_DATA : in STD_LOGIC_VECTOR (23 downto 0);
           O_DATA : out STD_LOGIC);
end motor_pwm;

architecture Behavioral of motor_pwm is

signal counter : std_logic_vector(23 downto 0) := (others => '0');

begin
	P0: process(I_CLK)
	begin
		if(rising_edge(I_CLK)) then
			if(counter <= I_DATA) then
				O_DATA <= '1';
			else
				O_DATA <= '0';
			end if;
			if(counter = "001001100010010110100000") then
				counter <= (others => '0');
			else
				counter <= counter + "00000001";
			end if;
		end if;
	end process;

end Behavioral;

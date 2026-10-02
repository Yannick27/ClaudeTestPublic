-------------------------------------------------------------------------------
-- PeriodNormLinPkg : types used by PeriodNormLin
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package PeriodNormLinPkg is
  type ChebyshevCoeff_t is array (natural range <>) of signed(31 downto 0);
end package PeriodNormLinPkg;

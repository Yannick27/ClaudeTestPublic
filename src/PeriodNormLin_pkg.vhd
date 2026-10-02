--------------------------------------------------------------------------------
-- Package : PeriodNormLin_pkg
-- Purpose : Types shared by PeriodNormLin and its users.
--           If the type ChebyshevCoeff_t already exists in another package of
--           your project, drop this file and change the "use" clause in
--           PeriodNormLin.vhd accordingly.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package PeriodNormLin_pkg is

  -- Chebyshev coefficients, Q8.24
  type ChebyshevCoeff_t is array (natural range <>) of signed(31 downto 0);

end package PeriodNormLin_pkg;

-------------------------------------------------------------------------------
-- ChevyPolyPkg : type used by the ChevyPoly entity port
--
-- If your project already declares ChebyshevCoeff_t in another package, do not
-- compile this file and `use` your own package in ChevyPoly.vhd instead
-- (declaring the same type in two visible packages makes the name ambiguous).
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package ChevyPolyPkg is

  type ChebyshevCoeff_t is array (natural range <>) of signed(31 downto 0); -- (Q.FRAC)

end package ChevyPolyPkg;

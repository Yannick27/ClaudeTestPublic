-- Type declarations used by the AngleCompute port list (as given in the spec).
-- If your project already declares these types in one of its own packages,
-- do not compile this file; just point the "use work.<pkg>.all" clauses of the
-- other files to your package.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package EcsTypes_pkg is
  type EcsPeriod_t          is array (natural range <>) of unsigned(23 downto 0);
  type EcsGain_t            is array (natural range <>) of signed(31 downto 0); -- format Q-4.36
  type EcsOffset_t          is array (natural range <>) of signed(31 downto 0); -- format Q8.24
  type ChebyshevCoeff_t     is array (natural range <>) of signed(31 downto 0);
  type ChebyshevCoeffArray_t is array (natural range <>) of ChebyshevCoeff_t;
end package EcsTypes_pkg;

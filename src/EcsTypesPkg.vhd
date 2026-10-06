-------------------------------------------------------------------------------
-- EcsTypesPkg
-- Array types used on the AngleCompute interface (copied verbatim from the
-- interface specification).  If these types already exist in your project's
-- own package, delete this file and add a "use" clause for your package to
-- AngleCompute.vhd and EcsLinearizer.vhd instead.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package EcsTypesPkg is
  type EcsPeriod_t           is array (natural range <>) of unsigned(23 downto 0);
  type EcsGain_t             is array (natural range <>) of signed(31 downto 0);  -- format Q-4.36
  type EcsOffset_t           is array (natural range <>) of signed(31 downto 0);  -- format Q8.24
  type ChebyshevCoeff_t      is array (natural range <>) of signed(31 downto 0);
  type ChebyshevCoeffArray_t is array (natural range <>) of ChebyshevCoeff_t;
end package EcsTypesPkg;

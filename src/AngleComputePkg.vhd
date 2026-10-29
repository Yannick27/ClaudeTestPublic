-------------------------------------------------------------------------------
-- AngleComputePkg : types shared by AngleCompute and its testbench
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package AngleComputePkg is

  constant ORDER : positive := 6;       --  Chebyshev polynomial order
  type     ChebyshevCoeff_t      is array (natural range <>) of signed(31 downto 0);
  type     EcsPeriod_t           is array (natural range <>) of unsigned(23 downto 0);
  type     EcsParam_t            is array (natural range <>) of signed(31 downto 0);
  type     ChebyshevCoeffArray_t is array (1 to 4) of ChebyshevCoeff_t(1 to ORDER);

end package AngleComputePkg;

-------------------------------------------------------------------------------
-- AngleComputePkg
--
--   * Types of the AngleCompute ports (VHDL-2008: array of unconstrained arrays).
--   * Constants / helper functions shared by the AngleCompute design units.
--
-- If the four port types already exist in another package of the project, delete
-- them here and add the corresponding "use" clause to the design units instead.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package AngleComputePkg is

  ---------------------------------------------------------------------------
  -- Port types
  ---------------------------------------------------------------------------
  type EcsPeriod_t           is array (natural range <>) of unsigned(23 downto 0);
  type EcsParam_t            is array (natural range <>) of signed(31 downto 0);
  type ChebyshevCoeff_t      is array (natural range <>) of signed(31 downto 0);
  type ChebyshevCoeffArray_t is array (natural range <>) of ChebyshevCoeff_t;

  ---------------------------------------------------------------------------
  -- Fixed-point format of the normalised period x and of the Chebyshev
  -- polynomials Tk(x): signed, T_W bits, T_FRAC fractional bits, so that
  -- +1.0 = 2**T_FRAC is representable.  35 bits = what four 18x18 RTG4 math
  -- blocks multiply (see MulSigned35).
  ---------------------------------------------------------------------------
  constant T_FRAC : natural := 33;
  constant T_W    : natural := T_FRAC + 2;

  -- ceil(log2(n))
  function clog2(n : positive) return natural;

  -- Width of PeriodLin = sum of ORDER products Tk * ck (T_FRAC fractional bits):
  --   sign + 31 coefficient bits + T_FRAC + clog2(ORDER) summation growth + 2 guard bits
  function LinWidth(order : positive) return positive;

end package AngleComputePkg;

package body AngleComputePkg is

  function clog2(n : positive) return natural is
    variable r : natural  := 0;
    variable v : positive := 1;
  begin
    while v < n loop
      v := v * 2;
      r := r + 1;
    end loop;
    return r;
  end function clog2;

  function LinWidth(order : positive) return positive is
  begin
    return 1 + 31 + T_FRAC + clog2(order) + 2;
  end function LinWidth;

end package body AngleComputePkg;

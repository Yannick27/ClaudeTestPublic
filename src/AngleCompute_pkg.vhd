-------------------------------------------------------------------------------
-- AngleCompute_pkg : types and helper functions shared by the AngleCompute
--                    design units.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package AngleCompute_pkg is

  -- Order of the Chebyshev polynomials. A VHDL type cannot depend on an entity
  -- generic, so the array size of ChebyshevCoeff_t is fixed here. The ORDER
  -- generic of AngleCompute must be equal to this constant (checked by an
  -- assertion at elaboration). To change the order, change this one constant.
  constant CHEBY_ORDER_C : positive := 6;

  type EcsPeriod_t      is array (natural range <>) of unsigned(23 downto 0);
  type EcsGain_t        is array (natural range <>) of signed(31 downto 0);  -- format Q-4.36
  type EcsOffset_t      is array (natural range <>) of signed(31 downto 0);  -- format Q8.24
  type ChebyshevCoeff_t is array (1 to CHEBY_ORDER_C) of signed(31 downto 0);  -- format Q.FRAC

  -- ceil(log2(n))
  function clog2(n : positive) return natural;

  -- Width of a linearized period (sum of ORDER terms coefficient*Tk, with
  -- |Tk| <= 1, expressed with GUARD more fractional bits than the coefficients)
  function LinWidth(order : positive; guard : natural) return positive;

end package AngleCompute_pkg;

package body AngleCompute_pkg is

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

  function LinWidth(order : positive; guard : natural) return positive is
  begin
    return 32 + guard + clog2(order);
  end function LinWidth;

end package body AngleCompute_pkg;

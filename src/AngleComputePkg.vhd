-------------------------------------------------------------------------------
-- AngleComputePkg
--   Types used on the AngleCompute interface + width helper functions.
--   (If these types already exist in your project package, keep yours and
--    only keep the helper functions.)
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package AngleComputePkg is

  type EcsPeriod_t           is array (natural range <>) of unsigned(23 downto 0);
  type EcsGain_t             is array (natural range <>) of signed(31 downto 0);  -- format Q-4.36
  type EcsOffset_t           is array (natural range <>) of signed(31 downto 0);  -- format Q8.24
  type ChebyshevCoeff_t      is array (natural range <>) of signed(31 downto 0);
  type ChebyshevCoeffArray_t is array (natural range <>) of ChebyshevCoeff_t;

  -- ceil(log2(n))
  function clog2(n : positive) return natural;

  -- Width of one linearized period  (|sum_k Tk*Ck| <= ORDER * 2**31)
  function LinWidth(ORDER : positive) return positive;

  -- Width of DeltaECS = LinP - LinN + Gamma  (|.| <= (2*ORDER+1) * 2**31)
  function DeltaWidth(ORDER : positive) return positive;

end package AngleComputePkg;

package body AngleComputePkg is

  function clog2(n : positive) return natural is
    variable r : natural := 0;
    variable v : natural := 1;
  begin
    while v < n loop
      v := v * 2;
      r := r + 1;
    end loop;
    return r;
  end function clog2;

  function LinWidth(ORDER : positive) return positive is
  begin
    return 32 + clog2(ORDER + 1);
  end function LinWidth;

  function DeltaWidth(ORDER : positive) return positive is
  begin
    return 32 + clog2(2 * ORDER + 2);
  end function DeltaWidth;

end package body AngleComputePkg;

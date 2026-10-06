-------------------------------------------------------------------------------
-- AngleComputePkg
-- Constants, width functions and the CORDIC angle table shared by the
-- AngleCompute design.  Every width that depends on ORDER is computed here from
-- worst-case bounds, so no intermediate value can overflow and no clamping or
-- saturation is needed anywhere.
--
-- Number formats
--   Coefficients / Gamma  : signed 32-bit integers in one common (unspecified)
--                           format.  PeriodLin and DeltaECS keep that format
--                           plus GUARD_BITS extra fractional bits.
--   Normalised period X   : signed Q2.30   (range [-2, 2), |X| <= 1 by spec)
--   Chebyshev term Tk     : signed Q2.30   (|Tk| <= 1)
--   Angle                 : 65536 = 2*pi, output is modulo 2*pi
-- The angle only depends on the ratio of the two rotated components, so the
-- absolute format of the coefficients and of the Cos/Sin inputs is irrelevant
-- as long as all coefficients/Gammas share one format and all four Cos/Sin
-- inputs share one format.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package AngleComputePkg is

  constant COEFF_W     : positive := 32;  -- width of coefficients, Gammas, Cos/Sin
  constant X_FRAC      : positive := 30;  -- fractional bits of X and Tk (Q2.30)
  constant GUARD_BITS  : natural  := 4;   -- extra fractional bits kept in PeriodLin
  constant ANGLE_W     : positive := 16;  -- output angle width (65536 = 2*pi)
  constant ANGLE_FRAC  : positive := 8;   -- CORDIC phase accumulator guard bits
  constant CORDIC_W    : positive := 32;  -- CORDIC x/y datapath width
  constant CORDIC_ITER : positive := 18;  -- CORDIC iterations (residual < 0.1 LSB)

  -- smallest k such that 2**k >= n
  function CeilLog2 (n : positive) return natural;

  function MinInt (a, b : integer) return integer;
  function MaxInt (a, b : integer) return integer;

  -- width of one linearised period:  |PeriodLin| <= ORDER * 2**31 * 2**GUARD_BITS
  function PeriodLinWidth (order : positive) return positive;
  -- width of DeltaECS = PeriodLinP - PeriodLinN + Gamma
  function DeltaWidth (order : positive) return positive;
  -- width of the rotated components  Cos*DeltaX -/+ Sin*DeltaZ
  function RotWidth (order : positive) return positive;

  -- a + b + cin as one carry chain (a and b must have the same length)
  function AddCin (a, b : signed; cin : std_logic) return signed;

  -- atan(2**-i) in turns * 2**(ANGLE_W + ANGLE_FRAC), i.e. 65536 = 2*pi
  type AtanTable_t is array (0 to CORDIC_ITER - 1) of unsigned(ANGLE_W + ANGLE_FRAC - 1 downto 0);
  constant ATAN_TABLE : AtanTable_t := (
    0  => to_unsigned(2097152, ANGLE_W + ANGLE_FRAC),
    1  => to_unsigned(1238021, ANGLE_W + ANGLE_FRAC),
    2  => to_unsigned(654136, ANGLE_W + ANGLE_FRAC),
    3  => to_unsigned(332050, ANGLE_W + ANGLE_FRAC),
    4  => to_unsigned(166669, ANGLE_W + ANGLE_FRAC),
    5  => to_unsigned(83416, ANGLE_W + ANGLE_FRAC),
    6  => to_unsigned(41718, ANGLE_W + ANGLE_FRAC),
    7  => to_unsigned(20860, ANGLE_W + ANGLE_FRAC),
    8  => to_unsigned(10430, ANGLE_W + ANGLE_FRAC),
    9  => to_unsigned(5215, ANGLE_W + ANGLE_FRAC),
    10 => to_unsigned(2608, ANGLE_W + ANGLE_FRAC),
    11 => to_unsigned(1304, ANGLE_W + ANGLE_FRAC),
    12 => to_unsigned(652, ANGLE_W + ANGLE_FRAC),
    13 => to_unsigned(326, ANGLE_W + ANGLE_FRAC),
    14 => to_unsigned(163, ANGLE_W + ANGLE_FRAC),
    15 => to_unsigned(81, ANGLE_W + ANGLE_FRAC),
    16 => to_unsigned(41, ANGLE_W + ANGLE_FRAC),
    17 => to_unsigned(20, ANGLE_W + ANGLE_FRAC));

end package AngleComputePkg;

package body AngleComputePkg is

  function CeilLog2 (n : positive) return natural is
    variable k : natural := 0;
  begin
    while 2 ** k < n loop
      k := k + 1;
    end loop;
    return k;
  end function CeilLog2;

  function MinInt (a, b : integer) return integer is
  begin
    if a < b then
      return a;
    end if;
    return b;
  end function MinInt;

  function MaxInt (a, b : integer) return integer is
  begin
    if a > b then
      return a;
    end if;
    return b;
  end function MaxInt;

  -- |c| <= 2**31 and |Tk| <= 1 (+ a few Q2.30 LSB of rounding), so
  -- |PL| < ORDER * 2**(31 + GUARD_BITS) * (1 + 2**-20) fits in
  -- 32 + CeilLog2(ORDER) + GUARD_BITS bits; one more bit is kept as margin.
  function PeriodLinWidth (order : positive) return positive is
  begin
    return 33 + CeilLog2(order) + GUARD_BITS;
  end function PeriodLinWidth;

  -- |PLP - PLN + Gamma| < (2*ORDER + 1) * 2**(31 + GUARD_BITS) < 2**(WPL) .
  function DeltaWidth (order : positive) return positive is
  begin
    return PeriodLinWidth(order) + 1;
  end function DeltaWidth;

  -- |Cos*DX -/+ Sin*DZ| <= 2 * 2**(WDL-1) * 2**31
  function RotWidth (order : positive) return positive is
  begin
    return DeltaWidth(order) + COEFF_W + 1;
  end function RotWidth;

  function AddCin (a, b : signed; cin : std_logic) return signed is
    variable s : signed(a'length downto 0);
  begin
    s := (a & '1') + (b & cin);
    return s(a'length downto 1);
  end function AddCin;

end package body AngleComputePkg;

-------------------------------------------------------------------------------
-- AngleComputePkg : types and fixed-point conventions for AngleCompute
--
-- Fixed-point conventions (all signed 32-bit values unless stated otherwise):
--   * Chebyshev coefficients, Gamma, Cos/Sin         : Q2.30  (1.0 = 2**30)
--   * OffsetNorm                                     : Q4.28  (1.0 = 2**28), so that
--     offsets such as -3.0 (period 1.5e6 * 2e-6) are representable
--   * Normalized period / Chebyshev polynomials Tk   : Q2.30
--   * Gain  : value of 1 period count, expressed with GAIN_FRAC fractional
--             bits (Gain = real_gain * 2**GAIN_FRAC).  With GAIN_FRAC = 48 a
--             period span of 1e6 counts (gain = 2e-6) gives ~1.1e9, which
--             fits comfortably in 32 bits signed.
--   * Linearized periods / DeltaECS                  : signed(LIN_W-1 downto 0), Q.30
--   * AnglexDO : 16-bit unsigned, 65536 = 360 degrees (2*pi), wraps modulo 2*pi
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package AngleComputePkg is

  type EcsPeriod_t      is array (natural range <>) of unsigned(23 downto 0);
  type EcsGain_t        is array (natural range <>) of signed(31 downto 0);
  type EcsOffset_t      is array (natural range <>) of signed(31 downto 0);
  type ChebyshevCoeff_t is array (0 to 5) of signed(31 downto 0);

  constant Q_FRAC    : natural := 30;  -- fractional bits of Offset/Coeff/Gamma/Sin/Cos/Tk
  constant OFFSET_FRAC : natural := 28;  -- fractional bits of OffsetNorm
  constant GAIN_FRAC : natural := 48;  -- fractional bits of GainNorm (must be >= Q_FRAC)
  constant LIN_W     : natural := 40;  -- width of linearized values (Q.30)

  -- CORDIC angle table: atan(2**-i) in units of 2*pi / 2**22
  constant CORDIC_ITER : natural := 17;
  constant ANGLE_W     : natural := 22;
  type AtanTable_t is array (0 to CORDIC_ITER-1) of unsigned(ANGLE_W-1 downto 0);
  constant ATAN_TABLE : AtanTable_t := (
    to_unsigned(524288, ANGLE_W), to_unsigned(309505, ANGLE_W),
    to_unsigned(163534, ANGLE_W), to_unsigned( 83012, ANGLE_W),
    to_unsigned( 41667, ANGLE_W), to_unsigned( 20854, ANGLE_W),
    to_unsigned( 10430, ANGLE_W), to_unsigned(  5215, ANGLE_W),
    to_unsigned(  2608, ANGLE_W), to_unsigned(  1304, ANGLE_W),
    to_unsigned(   652, ANGLE_W), to_unsigned(   326, ANGLE_W),
    to_unsigned(   163, ANGLE_W), to_unsigned(    81, ANGLE_W),
    to_unsigned(    41, ANGLE_W), to_unsigned(    20, ANGLE_W),
    to_unsigned(    10, ANGLE_W));

end package AngleComputePkg;

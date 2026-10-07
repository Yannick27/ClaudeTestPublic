-------------------------------------------------------------------------------
-- CordicAtan2
--
--   AnglexDO = atan2(YxDI, XxDI) over the full circle, 65536 = 2*pi
--              (0 = +X axis, 16384 = +Y axis, 32768 = -X axis, 49152 = -Y axis),
--   rounded to nearest, result in [0, 65535] (negative angles wrap by 2*pi).
--   If X = Y = 0 the angle is 0.
--
--   Algorithm : CORDIC in vectoring mode (shift and add only, no divider).
--     1. (X, Y) is scaled by PairNormalizer to 30 bits so that the precision does
--        not depend on the magnitude of the inputs.
--     2. If X < 0 the vector is rotated by pi first (both signs flipped, angle
--        accumulator preset to pi) so that the CORDIC (|angle| < 99.8 deg) covers
--        the whole circle.
--     3. NIT = 24 micro-rotations by atan(2**-i): X' = X -/+ Y/2**i,
--        Y' = Y +/- X/2**i, Z' = Z -/+ atan(2**-i), the sign being the sign of Y.
--     Error budget (worst case): residual angle 2**-23 rad, data path 2**-28 rad
--     per iteration, angle table 0.5 LSB of 2**-28 turn per iteration: well under
--     0.01 LSB of the 16-bit output.
--
--   Timing structure (RTG4, 100 MHz)
--     * X, Y and Z are only ever loaded by their adder/subtractor (one carry chain
--       from registers to the register, no multiplexer behind it).  The initial load
--       and the pre-rotation by pi are performed by the same adders as a first
--       "micro-rotation" 0 +/- (x0, y0) from cleared registers.
--     * There is no barrel shifter: X/2**i and Y/2**i are obtained by copying X and Y
--       and shifting the copies right by one bit per clock, i times.  (Latency is not
--       critical here and this keeps every path at one or two LUT levels.)
--     * The rounding constant (half an LSB of the 16-bit result) is preloaded into Z.
--     Per iteration: 1 clock set-up + i clocks shifting + 1 clock add.
--
--   Handshake : StartxSI = '1' for one clock with XxDI/YxDI stable during that
--   clock.  DonexSO = '1' for one clock when AnglexDO is valid (it then stays
--   valid until the next start).  Duration: up to WIN + NIT*(NIT+3)/2 + 8 clocks
--   (400 clocks for WIN = 68).
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity CordicAtan2 is
  generic(
    WIN : positive range 3 to 256 := 68    -- width of XxDI / YxDI
  );
  port(
    ClkxCI   : in  std_logic;
    StartxSI : in  std_logic;
    XxDI     : in  signed(WIN-1 downto 0);
    YxDI     : in  signed(WIN-1 downto 0);
    DonexSO  : out std_logic := '0';
    AnglexDO : out unsigned(15 downto 0) := (others => '0')
  );
end entity CordicAtan2;

architecture rtl of CordicAtan2 is

  constant NIT : positive := 24;   -- number of micro-rotations
  constant NW  : positive := 30;   -- width of the normalised input
  constant CW  : positive := 32;   -- width of the X/Y data path (2 bits of CORDIC gain headroom)
  constant AW  : positive := 28;   -- width of the angle accumulator, 2**AW = 2*pi

  constant HALF_LSB : natural := 2**(AW-17);   -- half an LSB of the 16-bit result
  constant PI_ANGLE : natural := 2**(AW-1);

  -- atan(2**-i) / (2*pi) * 2**AW, rounded to nearest (generated with 80-digit arithmetic)
  type AtanTab_t is array (0 to NIT-1) of natural;
  constant ATAN_TAB : AtanTab_t := (
    33554432, 19808338, 10466182,  5312797,  2666708,  1334654,
      667490,   333765,   166885,    83443,    41722,    20861,
       10430,     5215,     2608,     1304,      652,      326,
         163,       81,       41,       20,       10,        5);

  type State_t is (S_IDLE, S_NORM, S_SETUP, S_SHIFT, S_ADD, S_OUT);
  signal State : State_t := S_IDLE;

  signal NormDone : std_logic;
  signal NormZero : std_logic;
  signal NormX    : signed(NW-1 downto 0);
  signal NormY    : signed(NW-1 downto 0);

  signal X  : signed(CW-1 downto 0)   := (others => '0');
  signal Y  : signed(CW-1 downto 0)   := (others => '0');
  signal Z  : unsigned(AW-1 downto 0) := (others => '0');
  signal Xs : signed(CW-1 downto 0)   := (others => '0');  -- adder operand of Y  (X / 2**i)
  signal Ys : signed(CW-1 downto 0)   := (others => '0');  -- adder operand of X  (Y / 2**i)
  signal Ai : unsigned(AW-1 downto 0) := (others => '0');  -- adder operand of Z  (atan(2**-i))
  signal SubX, SubY, SubZ : std_logic := '0';              -- subtract instead of add
  signal Loading : std_logic := '0';                       -- the next S_ADD is the initial load
  signal SkipIt  : std_logic := '0';                       -- X = Y = 0: no iterations
  signal I  : natural range 0 to NIT-1 := 0;                -- iteration
  signal Cnt : natural range 0 to NIT-1 := 0;               -- 1-bit shifts still to do

  -- a + b or a - b through a single carry chain: (a,1) + (b xor sub, sub)
  function AddSub(a, b : signed; sub : std_logic) return signed is
    variable bx  : signed(b'length-1 downto 0);
    variable res : signed(a'length downto 0);
  begin
    bx := b;
    if sub = '1' then
      bx := not b;
    end if;
    res := (a & '1') + (bx & sub);
    return res(a'length downto 1);
  end function AddSub;

  function AddSub(a, b : unsigned; sub : std_logic) return unsigned is
    variable bx  : unsigned(b'length-1 downto 0);
    variable res : unsigned(a'length downto 0);
  begin
    bx := b;
    if sub = '1' then
      bx := not b;
    end if;
    res := (a & '1') + (bx & sub);
    return res(a'length downto 1);
  end function AddSub;

begin

  uNorm : entity work.PairNormalizer
    generic map(WIN => WIN, WOUT => NW)
    port map(
      ClkxCI   => ClkxCI,
      StartxSI => StartxSI,
      XxDI     => XxDI,
      YxDI     => YxDI,
      DonexSO  => NormDone,
      ZeroxSO  => NormZero,
      XxDO     => NormX,
      YxDO     => NormY);

  process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      DonexSO <= '0';

      -- X, Y, Z: cleared by the initial load, otherwise loaded by their adders only
      if State = S_NORM and NormDone = '1' then
        X <= (others => '0');
        Y <= (others => '0');
        Z <= (others => '0');
      elsif State = S_ADD then
        X <= AddSub(X, Ys, SubX);
        Y <= AddSub(Y, Xs, SubY);
        Z <= AddSub(Z, Ai, SubZ);
      end if;

      case State is

        when S_IDLE =>
          if StartxSI = '1' then
            State <= S_NORM;
          end if;

        -- initial load as the first micro-rotation: X' = 0 +/- x0, Y' = 0 +/- y0,
        -- Z' = half LSB (+ pi);  subtract when x0 < 0 (rotation by pi)
        when S_NORM =>
          if NormDone = '1' then
            Ys      <= resize(NormX, CW);
            Xs      <= resize(NormY, CW);
            SubX    <= NormX(NW-1);
            SubY    <= NormX(NW-1);
            SubZ    <= '0';
            if NormX(NW-1) = '1' then
              Ai <= to_unsigned(PI_ANGLE + HALF_LSB, AW);
            else
              Ai <= to_unsigned(HALF_LSB, AW);
            end if;
            Loading <= '1';
            SkipIt  <= NormZero;
            I       <= 0;
            State   <= S_ADD;
          end if;

        -- iteration set-up: copy X and Y (shifted by 0), table value, direction (sign of Y)
        when S_SETUP =>
          Xs   <= X;
          Ys   <= Y;
          Ai   <= to_unsigned(ATAN_TAB(I), AW);
          SubX <= Y(CW-1);               -- Y <  0: X' = X - Y/2**i, Y' = Y + X/2**i, Z' = Z - atan
          SubY <= not Y(CW-1);           -- Y >= 0: X' = X + Y/2**i, Y' = Y - X/2**i, Z' = Z + atan
          SubZ <= Y(CW-1);
          Cnt  <= I;
          if I = 0 then
            State <= S_ADD;
          else
            State <= S_SHIFT;
          end if;

        -- shift the copies right by one bit per clock: Xs = X / 2**i, Ys = Y / 2**i
        when S_SHIFT =>
          Xs  <= shift_right(Xs, 1);
          Ys  <= shift_right(Ys, 1);
          Cnt <= Cnt - 1;
          if Cnt = 1 then
            State <= S_ADD;
          end if;

        -- the adders above are loading X, Y, Z in this state
        when S_ADD =>
          if Loading = '1' then
            Loading <= '0';
            if SkipIt = '1' then
              State <= S_OUT;
            else
              State <= S_SETUP;
            end if;
          elsif I = NIT-1 then
            State <= S_OUT;
          else
            I     <= I + 1;
            State <= S_SETUP;
          end if;

        -- Z holds the angle plus half an LSB of the output: truncate to 16 bits
        when S_OUT =>
          AnglexDO <= Z(AW-1 downto AW-16);
          DonexSO  <= '1';
          State    <= S_IDLE;

      end case;
    end if;
  end process;

end architecture rtl;

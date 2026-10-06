-------------------------------------------------------------------------------
-- Atan2Cordic
-- Full-circle arctangent  Angle = atan2(Y, X)  with  65536 = 2*pi.
-- The result is the angle modulo 2*pi (negative angles wrap to 65536 - |a|),
-- so the output covers the whole circle.
--
-- X and Y are wide two's complement integers of arbitrary magnitude (only their
-- ratio matters).  They are first block-normalised: both are shifted left,
-- one bit per clock, until the larger one has used up the available range, so
-- small vectors keep full precision.  The upper CORDIC_W bits then feed an
-- iterative vectoring CORDIC:
--   * |x|,|y| < 2**(CORDIC_W-3) after normalisation leaves room for the CORDIC
--     gain (1.647) and the vector length (sqrt 2), so nothing can overflow;
--   * x < 0 is folded into x > 0 by negating x and y and starting the phase
--     at half a turn (no overflow possible: |x|,|y| < 2**(CORDIC_W-3));
--   * the phase accumulator is angle modulo 2*pi (ANGLE_FRAC guard bits); its
--     wrap-around is the intended modulo-2*pi behaviour, not an overflow.
-- CORDIC_ITER iterations leave a residual of atan(2**-(CORDIC_ITER-1)) < 0.1 LSB,
-- the result is rounded to 16 bits.  y = 0 counts as positive; X = Y = 0 gives
-- an arbitrary (but deterministic) angle.
--
-- Each CORDIC iteration takes three clocks (coarse shift, fine shift, then
-- add/subtract) to keep the logic depth low for 100 MHz in RTG4.
--
-- Handshake: pulse StartxSI for one clock (XxDI/YxDI are sampled on that
-- clock).  ValidxSO pulses for one clock together with the new AnglexDO,
-- which is held until the next result.
-- Latency: 58 + n clocks, n = number of normalisation shifts (0..WIN-1).
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.AngleComputePkg.all;

entity Atan2Cordic is
  generic (
    WIN : positive := 74  -- width of XxDI / YxDI
  );
  port (
    ClkxCI   : in  std_logic;
    StartxSI : in  std_logic;
    XxDI     : in  signed(WIN - 1 downto 0);  -- cosine-like component
    YxDI     : in  signed(WIN - 1 downto 0);  -- sine-like component
    ValidxSO : out std_logic := '0';
    AnglexDO : out unsigned(ANGLE_W - 1 downto 0) := (others => '0')
  );
end entity Atan2Cordic;

architecture rtl of Atan2Cordic is

  -- three extra sign bits so that any input can be normalised by shifting left
  constant WI       : positive := WIN + 3;
  constant ZW       : positive := ANGLE_W + ANGLE_FRAC;
  constant NORM_MAX : natural  := WI - 4;
  constant HALF_TURN : unsigned(ZW - 1 downto 0) := to_unsigned(2 ** (ZW - 1), ZW);
  constant HALF_LSB  : unsigned(ZW - 1 downto 0) := to_unsigned(2 ** (ANGLE_FRAC - 1), ZW);

  type state_t is (S_IDLE, S_NORM, S_FOLD, S_SHIFT_C, S_SHIFT_F, S_ITER, S_OUT);
  signal State : state_t := S_IDLE;

  signal Xw  : signed(WI - 1 downto 0) := (others => '0');
  signal Yw  : signed(WI - 1 downto 0) := (others => '0');
  signal Cnt : integer range 0 to NORM_MAX := 0;

  signal X    : signed(CORDIC_W - 1 downto 0) := (others => '0');
  signal Y    : signed(CORDIC_W - 1 downto 0) := (others => '0');
  signal Xc   : signed(CORDIC_W - 1 downto 0) := (others => '0');  -- shifted by 4*(Iter/4)
  signal Yc   : signed(CORDIC_W - 1 downto 0) := (others => '0');
  signal Xs   : signed(CORDIC_W - 1 downto 0) := (others => '0');  -- shifted by Iter
  signal Ys   : signed(CORDIC_W - 1 downto 0) := (others => '0');
  signal Z    : unsigned(ZW - 1 downto 0) := (others => '0');
  signal Az   : unsigned(ZW - 1 downto 0) := (others => '0');
  signal Iter : integer range 0 to CORDIC_ITER - 1 := 0;

  -- true if the four MSBs are equal, i.e. v is in [-2**(n-4), 2**(n-4))
  function TopEqual (v : signed) return boolean is
    constant n : natural := v'length;
  begin
    return v(n - 1) = v(n - 2) and v(n - 1) = v(n - 3) and v(n - 1) = v(n - 4);
  end function TopEqual;

begin

  assert WI >= CORDIC_W + 1
    report "Atan2Cordic: WIN too small for CORDIC_W" severity failure;

  process (ClkxCI)
    variable xv, yv : signed(CORDIC_W - 1 downto 0);
    variable zr     : unsigned(ZW - 1 downto 0);
  begin
    if rising_edge(ClkxCI) then
      ValidxSO <= '0';

      case State is

        when S_IDLE =>
          if StartxSI = '1' then
            Xw    <= resize(XxDI, WI);
            Yw    <= resize(YxDI, WI);
            Cnt   <= 0;
            State <= S_NORM;
          end if;

        -- shift both left while neither uses the top 4 bits
        when S_NORM =>
          if TopEqual(Xw) and TopEqual(Yw) and Cnt /= NORM_MAX then
            Xw  <= Xw(WI - 2 downto 0) & '0';
            Yw  <= Yw(WI - 2 downto 0) & '0';
            Cnt <= Cnt + 1;
          else
            State <= S_FOLD;
          end if;

        -- take the upper bits; fold the left half plane onto the right one
        when S_FOLD =>
          xv := Xw(WI - 1 downto WI - CORDIC_W);
          yv := Yw(WI - 1 downto WI - CORDIC_W);
          if xv(CORDIC_W - 1) = '1' then
            X <= -xv;
            Y <= -yv;
            Z <= HALF_TURN;
          else
            X <= xv;
            Y <= yv;
            Z <= (others => '0');
          end if;
          Iter  <= 0;
          State <= S_SHIFT_C;

        -- barrel shift by Iter in two clocks (coarse: 0/4/8/12/16, fine: 0..3)
        when S_SHIFT_C =>
          Xc    <= shift_right(X, (Iter / 4) * 4);
          Yc    <= shift_right(Y, (Iter / 4) * 4);
          Az    <= ATAN_TABLE(Iter);
          State <= S_SHIFT_F;

        when S_SHIFT_F =>
          Xs    <= shift_right(Xc, Iter mod 4);
          Ys    <= shift_right(Yc, Iter mod 4);
          State <= S_ITER;

        -- drive y towards 0, accumulating the angle that was rotated away
        when S_ITER =>
          if Y(CORDIC_W - 1) = '0' then
            X <= X + Ys;
            Y <= Y - Xs;
            Z <= Z + Az;
          else
            X <= X - Ys;
            Y <= Y + Xs;
            Z <= Z - Az;
          end if;
          if Iter = CORDIC_ITER - 1 then
            State <= S_OUT;
          else
            Iter  <= Iter + 1;
            State <= S_SHIFT_C;
          end if;

        when S_OUT =>
          zr       := Z + HALF_LSB;
          AnglexDO <= zr(ZW - 1 downto ANGLE_FRAC);
          ValidxSO <= '1';
          State    <= S_IDLE;

      end case;
    end if;
  end process;

end architecture rtl;

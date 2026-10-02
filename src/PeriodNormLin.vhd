--------------------------------------------------------------------------------
-- Entity  : PeriodNormLin
-- Target  : Microchip RTG4, 100 MHz
--
-- Function
--   1. wait for PeriodEcsValidxSI = '1'
--   2. x = PeriodEcsxDI * GainNormxDI + OffsetNormxDI            (-1 <= x <= 1)
--   3. y = sum(k = 1..6) Tk(x) * ChebyshevCoeffxDI(k)
--        T0 = 1, T1 = x, T(k+2) = 2*x*T(k+1) - T(k)
--   4. PeriodxDO <= y, ValidxSO <= '1'
--
-- Architecture
--   Area-optimised sequencer around ONE shared signed 34x34 multiplier
--   (maps to RTG4 math blocks). The work is split in 12 identical micro-ops:
--
--       Step  0       : x            = Period * Gain + Offset
--       Step  1 .. 5  : T(s+1)       = 2*x*T(s) - T(s-1)       (T2 .. T6)
--       Step  6 .. 11 : Acc         += T(s-5) * Coeff(s-5)     (k = 1 .. 6)
--
--   Every micro-op runs through the same 5 clock cycles:
--
--       Issue  : operand registers MulA/MulB are loaded
--       Mult1  : MulP0 <= MulA * MulB                (multiplier stage 1)
--       Mult2  : MulP  <= MulP0                      (multiplier stage 2)
--       Post1  : R     <= shift / round / offset-add of MulP
--       Post2  : destination register <= R (+/- another register)
--
--   so that no clock cycle contains more than one multiplier or one adder
--   (<= 44 bit). PeriodxDO/ValidxSO are updated 61 clock cycles after the
--   clock edge that samples PeriodEcsValidxSI = '1' (about 0.6 us at 100 MHz).
--
-- Number formats (all signed, QI.F = I integer bits incl. sign, F fraction)
--   PeriodEcsxDI     integer (unsigned, 24 bit)
--   GainNormxDI      Q-4.36  -> value = raw * 2**-36
--   OffsetNormxDI    Q8.24   -> value = raw * 2**-24
--   ChebyshevCoeff   Q8.24
--   x, Tk (internal) Q2.32   34 bit, range [-2, 2): the exact values are in
--                            [-1, 1], the extra bit holds rounding noise and
--                            the intermediate 2*x*T(k) which reaches +/-2.
--   PeriodxDO        Q8.24
--
-- Bit growth ("nothing overflows by design"; no clamping / saturation anywhere)
--   Period*Gain     25b * 32b signed          -> 57 bit, exact (68 bit register)
--   + Offset        |sum| < 2**56             -> evaluated at full 68 bit width
--   x               |x| < 2 (<= 1 by spec)    -> window NormSum(37 downto 4)
--   x*T(k)          34b * 34b                 -> 68 bit, exact
--   2*x*T(k)        |.| <= 2 + noise          -> R is 42 bit, window is 36 bit
--   T(k)*Coeff(k)   |T|<2, |C|<=128           -> |.| < 2**8 -> < 2**40 at Q.32
--   Acc             6 terms < 6 * 2**40       -> 44 bit
--   PeriodxDO       Acc(39 downto 8): the result is Q8.24 by specification
--   Wherever a wider vector is cut back to a narrower one (Post1/Post2/Finish)
--   the cut-off MSBs are pure sign extension for any input within the
--   specification. The synthesis tool therefore also removes the unused MSBs
--   of the adders above those windows.
--
-- Rounding
--   x is rounded to nearest at 2**-32, the Offset term carries the rounding
--   constant (free). Products are rounded to nearest at 2**-32. The final
--   Q8.24 value is rounded to nearest by pre-loading the accumulator with half
--   an output LSB (free). Worst-case error vs. an ideal evaluation is
--       0.5 LSB + 6 * 2**-9 LSB + sum(k) |C(k)| * (k**2 + k*(k-1)/2) * 2**-33
--   (LSB = 2**-24), checked by the testbench.
--
-- Handshake
--   * PeriodEcsValidxSI is sampled in state Idle only. Inputs have to be stable
--     from that moment until ValidxSO/PeriodxDO are updated (61 cycles).
--   * PeriodxDO and ValidxSO change in the same clock cycle; PeriodxDO only
--     changes together with ValidxSO going to '1', so it is always coherent.
--   * As long as PeriodEcsValidxSI stays '1' the computation is repeated back
--     to back and ValidxSO stays '1'.
--   * ValidxSO returns to '0' as soon as the module is idle and
--     PeriodEcsValidxSI is '0'. With a single-cycle PeriodEcsValidxSI pulse
--     ValidxSO is therefore a single-cycle pulse too.
--
-- Reset
--   The interface has no reset. All registers carry power-up values, ValidxSO
--   and PeriodxDO are '0' after configuration.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.PeriodNormLin_pkg.all;

entity PeriodNormLin is
  port(
    ClkxCI             : in  std_logic; -- clock
    -- Frequency measurements ----
    PeriodEcsxDI       : in  unsigned(23 downto 0); -- measured period (always between 1000000 and 2000000)
    PeriodEcsValidxSI  : in  std_logic; -- measured period is valid
    -- Parameters ------
    GainNormxDI        : in  signed(31 downto 0); -- gain to normalize period between -1 and 1 (Q-4.36)
    OffsetNormxDI      : in  signed(31 downto 0); -- offset to normalize period between -1 and 1 (Q8.24)
    ChebyshevCoeffxDI  : in  ChebyshevCoeff_t(1 to 6); -- Chebyshev coefficients for linearization (Q8.24)
    -- output result
    ValidxSO           : out std_logic           := '0'; -- normalized and linearized period is valid
    PeriodxDO          : out signed(31 downto 0) := (others => '0') -- normalized and linearized period (Q8.24)
  );
end entity PeriodNormLin;

architecture rtl of PeriodNormLin is

  ------------------------------------------------------------------------------
  -- Fixed-point constants
  ------------------------------------------------------------------------------
  constant GAIN_FRAC : natural := 36;                  -- Q-4.36
  constant OFFS_FRAC : natural := 24;                  -- Q8.24
  constant COEF_FRAC : natural := 24;                  -- Q8.24
  constant OUT_FRAC  : natural := 24;                  -- Q8.24
  constant FX_FRAC   : natural := 32;                  -- x and Tk: Q2.32
  constant FX_W      : natural := 34;                  -- 1 sign + 1 integer + 32 fraction
  constant PROD_W    : natural := 2 * FX_W;            -- multiplier output, 68 bit
  constant RES_W     : natural := 42;                  -- post-processed product, see header
  constant ACC_W     : natural := 44;                  -- accumulator, see header

  constant NORM_SHIFT : natural := GAIN_FRAC - FX_FRAC;   -- 4  : Q.36 -> Q.32
  constant OFFS_SHIFT : natural := GAIN_FRAC - OFFS_FRAC; -- 12 : Offset Q.24 -> Q.36
  constant T_SHIFT    : natural := FX_FRAC - 1;           -- 31 : x*T is Q.64, 2*x*T is wanted in Q.32
  constant C_SHIFT    : natural := COEF_FRAC;             -- 24 : T*C is Q.56 -> Q.32
  constant OUT_SHIFT  : natural := FX_FRAC - OUT_FRAC;    -- 8  : Q.32 -> Q.24

  subtype Fx_t   is signed(FX_W - 1 downto 0);
  subtype Prod_t is signed(PROD_W - 1 downto 0);
  subtype Res_t  is signed(RES_W - 1 downto 0);
  subtype Acc_t  is signed(ACC_W - 1 downto 0);
  type    FxArray_t is array (1 to 6) of Fx_t;

  constant ONE      : Fx_t  := (FX_FRAC => '1', others => '0');        -- T0 = 1.0
  constant ACC_INIT : Acc_t := to_signed(2 ** (OUT_SHIFT - 1), ACC_W); -- half output LSB = rounding

  ------------------------------------------------------------------------------
  -- floor(P / 2**S + 1/2), i.e. arithmetic shift right with round to nearest.
  -- The result is one bit wider than the shifted vector, so "+ 1" cannot wrap.
  ------------------------------------------------------------------------------
  function RoundShift(P : signed; S : positive) return signed is
    variable Pn : signed(P'length - 1 downto 0);
    variable Q  : signed(P'length - S downto 0);
  begin
    Pn := P;
    Q  := resize(Pn(Pn'left downto S), Q'length);
    if Pn(S - 1) = '1' then
      Q := Q + 1;
    end if;
    return Q;
  end function RoundShift;

  ------------------------------------------------------------------------------
  -- Sequencer
  ------------------------------------------------------------------------------
  type State_t is (Idle, Issue, Mult1, Mult2, Post1, Post2, Finish);
  signal State : State_t := Idle;
  signal Step  : natural range 0 to 11 := 0;

  -- shared multiplier
  signal MulA  : Fx_t   := (others => '0');
  signal MulB  : Fx_t   := (others => '0');
  signal MulP0 : Prod_t := (others => '0');
  signal MulP  : Prod_t := (others => '0');

  -- datapath
  signal OffsetTerm : Prod_t;                            -- Offset in Q.36 + rounding constant
  signal NormSum    : Prod_t;                            -- Period*Gain + Offset, Q.36
  signal R          : Res_t := (others => '0');          -- post-processed product
  signal Cheb       : FxArray_t := (others => (others => '0')); -- Cheb(k) = Tk(x), k = 1..6
  signal Acc        : Acc_t := (others => '0');          -- sum of Tk * Ck, Q.32

begin

  ------------------------------------------------------------------------------
  -- Offset aligned to the Q.36 product, plus half an LSB of the later cut to
  -- Q.32 (round to nearest). The 12 LSBs are zero, so this is wiring only.
  ------------------------------------------------------------------------------
  OffsetTerm <= resize(OffsetNormxDI, PROD_W - OFFS_SHIFT)
                & to_signed(2 ** (NORM_SHIFT - 1), OFFS_SHIFT);

  NormSum <= MulP + OffsetTerm;

  ------------------------------------------------------------------------------
  -- Shared multiplier, free running: operands (MulA/MulB) -> MulP0 -> MulP
  ------------------------------------------------------------------------------
  p_Mult : process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      MulP0 <= MulA * MulB;
      MulP  <= MulP0;
    end if;
  end process p_Mult;

  ------------------------------------------------------------------------------
  -- Sequencer and datapath control
  ------------------------------------------------------------------------------
  p_Seq : process(ClkxCI)
    variable Tm2 : Fx_t;
    variable Tn  : Res_t;
  begin
    if rising_edge(ClkxCI) then

      case State is

        -- wait for a valid measurement -----------------------------------------
        when Idle =>
          if PeriodEcsValidxSI = '1' then
            Step  <= 0;
            State <= Issue;
          else
            ValidxSO <= '0';
          end if;

        -- load multiplier operands ---------------------------------------------
        when Issue =>
          case Step is
            when 0 =>                                  -- Period * Gain
              MulA <= signed(resize(PeriodEcsxDI, FX_W));
              MulB <= resize(GainNormxDI, FX_W);
            when 1 to 5 =>                             -- x * T(Step)  -> T(Step+1)
              MulA <= Cheb(1);
              MulB <= Cheb(Step);
            when 6 to 11 =>                            -- T(k) * C(k), k = Step-5
              MulA <= Cheb(Step - 5);
              MulB <= resize(ChebyshevCoeffxDI(Step - 5), FX_W);
          end case;
          State <= Mult1;

        -- multiplier pipeline --------------------------------------------------
        when Mult1 =>
          State <= Mult2;

        when Mult2 =>
          State <= Post1;

        -- MulP is valid: shift / round / add offset ---------------------------
        when Post1 =>
          case Step is
            when 0 =>                                  -- (Period*Gain + Offset) Q.36 -> Q.32
              R <= resize(NormSum(NORM_SHIFT + FX_W - 1 downto NORM_SHIFT), RES_W);
            when 1 to 5 =>                             -- 2*x*T(k) in Q.32
              R <= resize(RoundShift(MulP, T_SHIFT), RES_W);
            when 6 to 11 =>                            -- T(k)*C(k) Q.56 -> Q.32
              R <= resize(RoundShift(MulP, C_SHIFT), RES_W);
          end case;
          State <= Post2;

        -- R is valid: store / recurse / accumulate ------------------------------
        when Post2 =>
          case Step is
            when 0 =>                                  -- T1 = x
              Cheb(1) <= R(FX_W - 1 downto 0);
            when 1 to 5 =>                             -- T(Step+1) = 2*x*T(Step) - T(Step-1)
              if Step = 1 then
                Tm2 := ONE;                            -- T0
              else
                Tm2 := Cheb(Step - 1);
              end if;
              Tn := R - resize(Tm2, RES_W);
              Cheb(Step + 1) <= Tn(FX_W - 1 downto 0); -- |T| < 2: fits Q2.32
            when 6 =>
              Acc <= ACC_INIT + resize(R, ACC_W);      -- first term, preloaded with rounding
            when 7 to 11 =>
              Acc <= Acc + resize(R, ACC_W);
          end case;

          if Step = 11 then
            State <= Finish;
          else
            Step  <= Step + 1;
            State <= Issue;
          end if;

        -- publish result ----------------------------------------------------------
        when Finish =>
          PeriodxDO <= Acc(OUT_SHIFT + 31 downto OUT_SHIFT);   -- Q.32 -> Q8.24
          ValidxSO  <= '1';
          State     <= Idle;

      end case;

    end if;
  end process p_Seq;

end architecture rtl;

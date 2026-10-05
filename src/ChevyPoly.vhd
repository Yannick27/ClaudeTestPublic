-------------------------------------------------------------------------------
-- ChevyPoly : period normalisation + Chebyshev polynomial linearisation
--
--   PeriodNorm = PeriodEcsxDI * GainNormxDI + OffsetNormxDI        (|PeriodNorm| <= 1)
--   PeriodxDO  = sum_{k=1..ORDER} Tk(PeriodNorm) * ChebyshevCoeffxDI(k)
--                T0 = 1, T1 = x, T(k+1) = 2*x*Tk - T(k-1)
--
-- Number formats
--   PeriodEcsxDI      unsigned integer (24 bit)
--   GainNormxDI       Q-4.36   value = raw * 2**-36
--   OffsetNormxDI     Q8.24    value = raw * 2**-24
--   ChebyshevCoeffxDI Q.FRAC   value = raw * 2**-FRAC
--   PeriodxDO         Q.FRAC   same format as the coefficients (the Tk are dimensionless,
--                              so FRAC cancels out of the arithmetic and is not needed
--                              by the datapath)
--   internal x, Tk    Q2.36    38 bit signed, value = raw * 2**-36
--
-- Architecture
--   One time-multiplexed 38x38 signed multiplier (3 register stages, intended to be
--   mapped onto the RTG4 MACC math blocks) driven by a small FSM. All operations are
--   executed back to back, a result is available ~55 clock cycles after
--   PeriodEcsValidxSI for ORDER = 6 (0.55 us at 100 MHz). A new computation is
--   accepted as soon as ValidxSO has been pulsed.
--
--   idle    : wait PeriodEcsValidxSI, issue P*G
--   norm    : x = P*G + O*2**12                         (exact, no rounding)
--   loop k = 1..ORDER
--     mac   : acc += c_k * T_k
--     rec   : T_k+1 = round(2*x*T_k) - T_k-1             (skipped for k = ORDER)
--   done    : PeriodxDO = round(acc), ValidxSO = '1' for one clock
--
-- Word growth (nothing is clamped or saturated, widths are sized so that nothing can
-- overflow as long as the stated input ranges hold):
--   P*G            |P*G| < 2**24 * 2**31 = 2**55                  -> 57 bit product
--   P*G + O*2**12  57 bit + 44 bit                                -> 58 bit sum, the
--                  result is within [-1,1] so its Q2.36 image is the low 38 bits
--   2*x*T_k        |.| <= 2 (Q2.36 range is [-2,2))               -> 41 bit before the
--                  subtraction of T_k-1, 38 bit afterwards (|T_k+1| <= 1 + few LSB)
--   c_k*T_k        |.| < 2**31 * 2                                -> kept with GUARD=12
--                  bits below the output LSB (the 24 lowest product bits are dropped,
--                  worst case error per term 2**-12 LSB of PeriodxDO)
--   acc            ORDER terms                                    -> 33+GUARD+clog2(ORDER)
--   Final result is assumed to fit the 32 bit output (as stated in the spec).
--
-- Rounding: x is exact; the product 2*x*T_k is rounded to nearest (Q2.36), the
-- c_k*T_k terms are truncated at 2**-(FRAC+12) and the final result is rounded to
-- nearest (the rounding constant is pre-loaded in the accumulator).
--
-- Inputs: PeriodEcsValidxSI should be a 1 clock strobe. A strobe arriving while a
-- computation is running is ignored. GainNormxDI, OffsetNormxDI and ChebyshevCoeffxDI
-- must be stable until ValidxSO (PeriodEcsxDI is sampled in the strobe cycle).
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.ChevyPolyPkg.all;

entity ChevyPoly is
  generic(ORDER : positive := 6;        -- order of Chebyshev polynomial
          FRAC  : natural  := 24        -- fractional bits of the coefficients ChebyshevCoeffxDI
         );
  port(
    ClkxCI            : in  std_logic;  -- clock
    -- Frequency measurements ----
    PeriodEcsxDI      : in  unsigned(23 downto 0); -- measured period (always between 1000000 and 2000000)
    PeriodEcsValidxSI : in  std_logic;  -- measured period is valid
    -- Parameters ------
    GainNormxDI       : in  signed(31 downto 0); -- gain to normalize period between -1 and 1 (Q-4.36)
    OffsetNormxDI     : in  signed(31 downto 0); -- offset to normalize period between -1 and 1 (Q8.24)
    ChebyshevCoeffxDI : in  ChebyshevCoeff_t(1 to ORDER); -- Chebyshev coefficients for linearization
    -- output result
    ValidxSO          : out std_logic           := '0'; -- normalized and linearized period is valid
    PeriodxDO         : out signed(31 downto 0) := (others => '0') -- normalized and linearized period
  );
end entity ChevyPoly;

architecture rtl of ChevyPoly is

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

  ---------------------------------------------------------------------------
  -- Fixed-point formats
  ---------------------------------------------------------------------------
  constant GAIN_FRAC : natural := 36;             -- GainNormxDI   : Q-4.36
  constant OFFS_FRAC : natural := 24;             -- OffsetNormxDI : Q8.24
  constant XFRAC     : natural := GAIN_FRAC;      -- x and Tk fractional bits (keeps x exact)
  constant XW        : natural := XFRAC + 2;      -- x and Tk : Q2.36, 38 bit
  constant PW        : natural := 2 * XW;         -- multiplier product (Q4.72), 76 bit

  -- normalisation: P (25 bit signed) * G (32 bit signed) + O * 2**12
  constant NPW       : natural := 25 + 32;        -- 57 bit product
  constant NSW       : natural := NPW + 1;        -- 58 bit sum

  -- recurrence: 2*x*Tk in Q2.36 = product >> (XFRAC-1)
  constant RSH       : natural := XFRAC - 1;      -- 35
  constant RW        : natural := PW - RSH;       -- 41 bit

  -- coefficient accumulation
  constant GUARD     : natural := 12;             -- accumulator bits below the output LSB
  constant MSH       : natural := XFRAC - GUARD;  -- product bits dropped before accumulation
  constant ACCW      : natural := 33 + GUARD + clog2(ORDER);

  -- multiplier: registered operands + MUL_PIPE product registers
  constant MUL_PIPE  : positive := 2;
  constant MUL_LAT   : positive := MUL_PIPE + 1;  -- clocks from operand load to result usable

  constant ONE       : signed(XW-1 downto 0)   := shift_left(to_signed(1, XW), XFRAC);       -- T0 = 1.0
  constant ROUND_ACC : signed(ACCW-1 downto 0) := shift_left(to_signed(1, ACCW), GUARD - 1); -- 0.5 LSB

  type State_t is (S_IDLE, S_NORM_WAIT, S_MAC_ISSUE, S_MAC_WAIT,
                   S_REC_ISSUE, S_REC_WAIT, S_REC_SUB, S_DONE);
  type MulPipe_t is array (1 to MUL_PIPE) of signed(PW-1 downto 0);

  signal StatexD   : State_t                    := S_IDLE;
  signal WaitCntxD : natural range 0 to MUL_LAT - 1 := 0;
  signal KxD       : natural range 1 to ORDER   := 1;

  signal MulAxD    : signed(XW-1 downto 0)      := (others => '0');
  signal MulBxD    : signed(XW-1 downto 0)      := (others => '0');
  signal MulPipexD : MulPipe_t                  := (others => (others => '0'));
  alias  MulPxD    : signed(PW-1 downto 0) is MulPipexD(MUL_PIPE);

  signal XxD       : signed(XW-1 downto 0)      := (others => '0'); -- normalised period
  signal TCurxD    : signed(XW-1 downto 0)      := (others => '0'); -- T_k
  signal TPrevxD   : signed(XW-1 downto 0)      := (others => '0'); -- T_k-1
  signal RxD       : signed(RW-1 downto 0)      := (others => '0'); -- round(2*x*T_k)
  signal AccxD     : signed(ACCW-1 downto 0)    := (others => '0');

begin

  assert GUARD <= XFRAC
    report "ChevyPoly: GUARD must not exceed the internal fractional bits" severity failure;

  ---------------------------------------------------------------------------
  -- Shared multiplier (operand regs in the FSM, then MUL_PIPE product regs).
  -- Written as plain A*B between registers so that the synthesis tool can
  -- infer MACC blocks and absorb the registers into them.
  ---------------------------------------------------------------------------
  p_Mul : process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      MulPipexD(1) <= MulAxD * MulBxD;
      for i in 2 to MUL_PIPE loop
        MulPipexD(i) <= MulPipexD(i - 1);
      end loop;
    end if;
  end process p_Mul;

  ---------------------------------------------------------------------------
  -- Sequencer / datapath
  ---------------------------------------------------------------------------
  p_Fsm : process(ClkxCI)
    variable v_NormSum : signed(NSW-1 downto 0);
    variable v_RndExt  : signed(RW downto 0);
    variable v_Next    : signed(RW-1 downto 0);
  begin
    if rising_edge(ClkxCI) then
      ValidxSO <= '0';

      case StatexD is

        -- wait for a valid period, issue P * G
        when S_IDLE =>
          if PeriodEcsValidxSI = '1' then
            MulAxD    <= signed(resize(PeriodEcsxDI, XW));   -- unsigned -> positive signed
            MulBxD    <= resize(GainNormxDI, XW);
            WaitCntxD <= MUL_LAT - 1;
            StatexD   <= S_NORM_WAIT;
          end if;

        -- x = P*G (Q.36) + O (Q.24) * 2**12 ; initialise T0, T1, acc, k
        when S_NORM_WAIT =>
          if WaitCntxD = 0 then
            v_NormSum := resize(MulPxD(NPW-1 downto 0), NSW)
                         + shift_left(resize(OffsetNormxDI, NSW), XFRAC - OFFS_FRAC);
            XxD     <= v_NormSum(XW-1 downto 0);   -- |x| <= 1 : upper bits are sign copies
            TCurxD  <= v_NormSum(XW-1 downto 0);   -- T1 = x
            TPrevxD <= ONE;                        -- T0 = 1
            AccxD   <= ROUND_ACC;
            KxD     <= 1;
            StatexD <= S_MAC_ISSUE;
          else
            WaitCntxD <= WaitCntxD - 1;
          end if;

        -- acc += c_k * T_k
        when S_MAC_ISSUE =>
          MulAxD    <= TCurxD;
          MulBxD    <= resize(ChebyshevCoeffxDI(KxD), XW);
          WaitCntxD <= MUL_LAT - 1;
          StatexD   <= S_MAC_WAIT;

        when S_MAC_WAIT =>
          if WaitCntxD = 0 then
            AccxD <= AccxD + resize(MulPxD(PW-1 downto MSH), ACCW);
            if KxD = ORDER then
              StatexD <= S_DONE;
            else
              StatexD <= S_REC_ISSUE;
            end if;
          else
            WaitCntxD <= WaitCntxD - 1;
          end if;

        -- T_k+1 = 2*x*T_k - T_k-1
        when S_REC_ISSUE =>
          MulAxD    <= XxD;
          MulBxD    <= TCurxD;
          WaitCntxD <= MUL_LAT - 1;
          StatexD   <= S_REC_WAIT;

        when S_REC_WAIT =>
          if WaitCntxD = 0 then
            -- product is Q4.72; 2*product in Q2.36 = product >> 35.
            -- Round to nearest: keep one extra bit, add 1 (= 0.5 LSB), drop it.
            v_RndExt := resize(MulPxD(PW-1 downto RSH-1), RW + 1) + 1;
            RxD      <= v_RndExt(RW downto 1);
            StatexD  <= S_REC_SUB;
          else
            WaitCntxD <= WaitCntxD - 1;
          end if;

        when S_REC_SUB =>
          v_Next  := RxD - resize(TPrevxD, RW);
          TPrevxD <= TCurxD;
          TCurxD  <= v_Next(XW-1 downto 0);        -- |T_k+1| <= 1 (+ rounding)
          KxD     <= KxD + 1;
          StatexD <= S_MAC_ISSUE;

        -- acc has GUARD extra fractional bits and the +0.5 LSB rounding constant
        when S_DONE =>
          PeriodxDO <= AccxD(GUARD+31 downto GUARD);
          ValidxSO  <= '1';
          StatexD   <= S_IDLE;

      end case;
    end if;
  end process p_Fsm;

end architecture rtl;

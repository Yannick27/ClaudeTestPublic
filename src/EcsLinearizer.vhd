-------------------------------------------------------------------------------
-- EcsLinearizer
-- Normalises and linearises the period of ONE ECS.  AngleCompute reuses a
-- single instance for all four ECSs.
--
--   X  = PeriodxDI * GainxDI + OffsetxDI                  (|X| <= 1 by spec)
--   PL = sum k=1..ORDER  CoeffxDI(k) * Tk(X)
--   T0 = 1, T1 = X, Tk+1 = 2*X*Tk - Tk-1
--
-- Arithmetic
--   * PeriodxDI*GainxDI is exact (Q-4.36 gain -> 36 fractional bits), the
--     Q8.24 offset is aligned to 36 fractional bits, the sum is rounded to
--     Q2.30.  The sum is evaluated modulo 2**38: the true value lies in
--     [-1, 1] so the wrap-around is harmless and no wide adder is needed.
--   * Tk are Q2.30 (|Tk| <= 1).  The recurrence product 2*X*Tk-1 may reach +/-2
--     for an instant; it is evaluated modulo 2**32 and the subtraction of Tk-1
--     brings it back inside [-1, 1], so the result is exact modulo 2**32.
--   * Each coefficient*Tk product (64 bit) is rounded to GUARD_BITS fractional
--     bits and accumulated in PeriodLinWidth(ORDER) bits (worst-case bound, no
--     overflow possible).
--   * PeriodLinxDO has the format of the coefficients with GUARD_BITS extra
--     fractional bits.
--
-- One SerialMulAcc (one multiplier) does all products one after the other:
-- 2*ORDER multiplications of ~11 clocks each, i.e. ~22*ORDER + 12 clocks.
--
-- Handshake: pulse StartxSI for one clock (inputs are sampled on that clock);
-- ValidxSO pulses for one clock when PeriodLinxDO holds the result, which then
-- stays valid until the next StartxSI.  PeriodLinxDO changes while computing.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.EcsTypesPkg.all;
use work.AngleComputePkg.all;

entity EcsLinearizer is
  generic (
    ORDER : positive := 6
  );
  port (
    ClkxCI       : in  std_logic;
    StartxSI     : in  std_logic;
    ValidxSO     : out std_logic := '0';
    PeriodxDI    : in  unsigned(23 downto 0);
    GainxDI      : in  signed(31 downto 0);   -- Q-4.36
    OffsetxDI    : in  signed(31 downto 0);   -- Q8.24
    CoeffxDI     : in  ChebyshevCoeff_t(1 to ORDER);
    PeriodLinxDO : out signed(PeriodLinWidth(ORDER) - 1 downto 0)
  );
end entity EcsLinearizer;

architecture rtl of EcsLinearizer is

  constant WPL : positive := PeriodLinWidth(ORDER);

  constant ONE_Q30  : signed(31 downto 0) := to_signed(2 ** X_FRAC, 32);
  -- offset (Q8.24) -> 36 fractional bits is a left shift by 12; adding 32
  -- (half an LSB of the Q2.30 result) only touches the 12 zero LSBs
  constant NORM_RND : signed(11 downto 0) := to_signed(32, 12);

  type state_t is (S_IDLE, S_NORM_WAIT, S_MAC_LOAD, S_MAC_WAIT, S_REC_LOAD, S_REC_WAIT);
  signal State : state_t := S_IDLE;

  signal OffCap   : signed(31 downto 0) := (others => '0');
  signal CoeffCap : ChebyshevCoeff_t(1 to ORDER) := (others => (others => '0'));
  signal CoeffK   : signed(31 downto 0) := (others => '0');
  signal K        : integer range 1 to ORDER := 1;

  signal X        : signed(31 downto 0) := (others => '0');  -- normalised period
  signal Ta       : signed(31 downto 0) := (others => '0');  -- multiplier operand A (= Tk)
  signal Bop      : signed(31 downto 0) := (others => '0');  -- multiplier operand B
  signal NegTprev : signed(31 downto 0) := (others => '0');  -- -Tk-1
  signal Acc      : signed(WPL - 1 downto 0) := (others => '0');

  signal MulStart : std_logic := '0';
  signal MulDone  : std_logic;
  signal MulP     : signed(63 downto 0);

begin

  PeriodLinxDO <= Acc;

  Mul : entity work.SerialMulAcc
    generic map (WA => 32, WB => 32, NT => 1)
    port map (
      ClkxCI   => ClkxCI,
      StartxSI => MulStart,
      AxDI     => Ta,
      BxDI     => Bop,
      SubxSI   => "0",
      DonexSO  => MulDone,
      PxDO     => MulP);

  process (ClkxCI)
    variable Sum38 : signed(37 downto 0);
    variable Tn    : signed(31 downto 0);
  begin
    if rising_edge(ClkxCI) then
      MulStart <= '0';
      ValidxSO <= '0';

      -- Coeff(K) through a free running register keeps the 6:1 mux out of
      -- the operand load path.  K is always updated several clocks before
      -- the coefficient is used.
      CoeffK <= CoeffCap(K);

      case State is

        when S_IDLE =>
          if StartxSI = '1' then
            -- sample this ECS' parameters and start  X = P*G + O
            Ta       <= signed(resize(PeriodxDI, 32));
            Bop      <= GainxDI;
            OffCap   <= OffsetxDI;
            CoeffCap <= CoeffxDI;
            K        <= 1;
            MulStart <= '1';
            State    <= S_NORM_WAIT;
          end if;

        when S_NORM_WAIT =>
          if MulDone = '1' then
            -- Q?.36 product + offset, rounded to Q2.30 (modulo 2**38)
            Sum38    := MulP(37 downto 0) + (OffCap(25 downto 0) & NORM_RND);
            X        <= Sum38(37 downto 6);
            Ta       <= Sum38(37 downto 6);   -- T1 = X
            NegTprev <= -ONE_Q30;             -- -T0
            Acc      <= (others => '0');
            State    <= S_MAC_LOAD;
          end if;

        -- Acc += Coeff(K) * Tk
        when S_MAC_LOAD =>
          Bop      <= CoeffK;
          MulStart <= '1';
          State    <= S_MAC_WAIT;

        when S_MAC_WAIT =>
          if MulDone = '1' then
            -- product has 30 fractional bits more than wanted: keep
            -- GUARD_BITS of them, round half up through the carry input
            Acc <= AddCin(Acc, resize(MulP(62 downto X_FRAC - GUARD_BITS), WPL),
                          MulP(X_FRAC - GUARD_BITS - 1));
            if K = ORDER then
              ValidxSO <= '1';
              State    <= S_IDLE;
            else
              K     <= K + 1;
              State <= S_REC_LOAD;
            end if;
          end if;

        -- Tk+1 = 2*X*Tk - Tk-1
        when S_REC_LOAD =>
          Bop      <= X;
          MulStart <= '1';
          State    <= S_REC_WAIT;

        when S_REC_WAIT =>
          if MulDone = '1' then
            -- (X*Tk) has 60 fractional bits; 2*X*Tk in Q2.30 is product >> 29
            Tn       := AddCin(MulP(60 downto 29), NegTprev, MulP(28));
            Ta       <= Tn;
            NegTprev <= -Ta;
            State    <= S_MAC_LOAD;
          end if;

      end case;
    end if;
  end process;

end architecture rtl;

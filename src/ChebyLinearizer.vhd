-------------------------------------------------------------------------------
-- ChebyLinearizer : normalization and Chebyshev linearization of one period.
--
--   PeriodNorm = Period * Gain + Offset                      (in [-1, 1])
--   PeriodLin  = sum(k = 1..ORDER) Tk(PeriodNorm) * Coeff(k)
--   T0 = 1, T1 = X, T(k+1) = 2*X*Tk - T(k-1)
--
-- Number formats
--   Period        : unsigned integer
--   Gain          : Q-4.36  (value = raw * 2^-36)
--   Offset        : Q8.24   (value = raw * 2^-24)
--   X, Tk         : Q2.30   (32 bits, value = raw * 2^-30), X is saturated to [-1, 1]
--   Coeff         : Q.FRAC  (value = raw * 2^-FRAC)
--   PeriodLin     : Q.(FRAC+GUARD), LinWidth(ORDER, GUARD) bits
--                   (the FRAC generic of AngleCompute does not change any
--                   arithmetic here, only the interpretation of the result)
--
-- All multiplications are made by one shared pipelined multiplier (MulPipe).
-- PeriodxDI, GainxDI, OffsetxDI and CoeffxDI must be stable from the clock
-- cycle where StartxSI = 1 until ValidxSO = 1.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.AngleCompute_pkg.all;

entity ChebyLinearizer is
  generic(
    ORDER : positive := CHEBY_ORDER_C;
    GUARD : natural  := 4                -- extra fractional bits kept in the result (<= 29)
  );
  port(
    ClkxCI    : in  std_logic;
    StartxSI  : in  std_logic;           -- start (1 clock pulse)
    ValidxSO  : out std_logic := '0';    -- result valid (1 clock pulse)
    PeriodxDI : in  unsigned(23 downto 0);
    GainxDI   : in  signed(31 downto 0);
    OffsetxDI : in  signed(31 downto 0);
    CoeffxDI  : in  ChebyshevCoeff_t;
    LinxDO    : out signed(LinWidth(ORDER, GUARD) - 1 downto 0) := (others => '0')
  );
end entity ChebyLinearizer;

architecture rtl of ChebyLinearizer is

  constant NF_C    : natural  := 30;                 -- fractional bits of X and Tk
  constant LIN_W_C : positive := LinWidth(ORDER, GUARD);
  constant ONE_C   : signed(31 downto 0) := to_signed(2 ** NF_C, 32);
  -- rounding constant for the product Coeff*Tk (FRAC+NF bits -> FRAC+GUARD bits)
  constant RND_ACC_C : signed(63 downto 0) := to_signed(2 ** (NF_C - GUARD - 1), 64);

  type State_t is (S_IDLE, S_NORM_ISSUE, S_NORM_WAIT,
                   S_COEF_ISSUE, S_REC_ISSUE, S_COEF_WAIT, S_REC_WAIT);
  signal StatexDP : State_t := S_IDLE;

  signal KxDP     : integer range 1 to ORDER := 1;
  signal XxDP     : signed(31 downto 0) := (others => '0');   -- normalized period
  signal TPrevxDP : signed(31 downto 0) := (others => '0');   -- T(k-1)
  signal TCurxDP  : signed(31 downto 0) := (others => '0');   -- T(k)
  signal AccxDP   : signed(LIN_W_C - 1 downto 0) := (others => '0');

  signal MulValidInxS  : std_logic;
  signal MulAxD        : signed(31 downto 0);
  signal MulBxD        : signed(31 downto 0);
  signal MulValidOutxS : std_logic;
  signal MulProdxD     : signed(63 downto 0);

begin

  assert ORDER = CHEBY_ORDER_C
    report "ChebyLinearizer: ORDER must be equal to CHEBY_ORDER_C of AngleCompute_pkg"
    severity failure;
  assert GUARD < NF_C
    report "ChebyLinearizer: GUARD must be < 30" severity failure;

  MulInst : entity work.MulPipe
    port map(
      ClkxCI   => ClkxCI,
      ValidxSI => MulValidInxS,
      AxDI     => MulAxD,
      BxDI     => MulBxD,
      ValidxSO => MulValidOutxS,
      ProdxDO  => MulProdxD);

  -- multiplier operand selection
  process(StatexDP, PeriodxDI, GainxDI, CoeffxDI, KxDP, XxDP, TCurxDP)
  begin
    MulValidInxS <= '0';
    MulAxD       <= (others => '0');
    MulBxD       <= (others => '0');
    case StatexDP is
      when S_NORM_ISSUE =>               -- Period * Gain
        MulValidInxS <= '1';
        MulAxD       <= signed(resize(PeriodxDI, 32));
        MulBxD       <= GainxDI;
      when S_COEF_ISSUE =>               -- Coeff(k) * Tk
        MulValidInxS <= '1';
        MulAxD       <= CoeffxDI(KxDP);
        MulBxD       <= TCurxDP;
      when S_REC_ISSUE =>                -- X * Tk (for T(k+1) = 2*X*Tk - T(k-1))
        MulValidInxS <= '1';
        MulAxD       <= XxDP;
        MulBxD       <= TCurxDP;
      when others =>
        null;
    end case;
  end process;

  process(ClkxCI)
    variable S_v    : signed(63 downto 0);
    variable X_v    : signed(31 downto 0);
    variable Prod_v : signed(LIN_W_C - 1 downto 0);
    variable M_v    : signed(33 downto 0);
  begin
    if rising_edge(ClkxCI) then
      ValidxSO <= '0';

      case StatexDP is

        when S_IDLE =>
          if StartxSI = '1' then
            StatexDP <= S_NORM_ISSUE;
          end if;

        when S_NORM_ISSUE =>
          StatexDP <= S_NORM_WAIT;

        when S_NORM_WAIT =>
          if MulValidOutxS = '1' then
            -- Period*Gain has 36 fractional bits, Offset has 24 -> align
            S_v := MulProdxD + shift_left(resize(OffsetxDI, 64), 12);
            -- 36 -> 30 fractional bits (rounded)
            S_v := shift_right(S_v + 2 ** 5, 6);
            -- saturate to [-1, 1] (the result is expected to be in this range)
            if S_v > resize(ONE_C, 64) then
              X_v := ONE_C;
            elsif S_v < -resize(ONE_C, 64) then
              X_v := -ONE_C;
            else
              X_v := resize(S_v, 32);
            end if;
            XxDP     <= X_v;
            TPrevxDP <= ONE_C;           -- T0
            TCurxDP  <= X_v;             -- T1
            KxDP     <= 1;
            AccxDP   <= (others => '0');
            StatexDP <= S_COEF_ISSUE;
          end if;

        when S_COEF_ISSUE =>
          if KxDP < ORDER then
            StatexDP <= S_REC_ISSUE;
          else
            StatexDP <= S_COEF_WAIT;
          end if;

        when S_REC_ISSUE =>
          StatexDP <= S_COEF_WAIT;

        when S_COEF_WAIT =>
          if MulValidOutxS = '1' then
            -- FRAC+30 -> FRAC+GUARD fractional bits (rounded)
            Prod_v := resize(shift_right(MulProdxD + RND_ACC_C, NF_C - GUARD), LIN_W_C);
            if KxDP = ORDER then
              LinxDO   <= AccxDP + Prod_v;
              ValidxSO <= '1';
              StatexDP <= S_IDLE;
            else
              AccxDP   <= AccxDP + Prod_v;
              StatexDP <= S_REC_WAIT;
            end if;
          end if;

        when S_REC_WAIT =>
          -- the X*Tk product arrives the clock cycle after the Coeff*Tk product
          if MulValidOutxS = '1' then
            -- 2*X*Tk : 60 -> 30 fractional bits (rounded); 34 bits, 2*X*Tk can reach 2.0
            M_v      := resize(shift_right(MulProdxD + 2 ** 28, 29), 34);
            TPrevxDP <= TCurxDP;
            TCurxDP  <= resize(M_v - resize(TPrevxDP, 34), 32);
            KxDP     <= KxDP + 1;
            StatexDP <= S_COEF_ISSUE;
          end if;

      end case;
    end if;
  end process;

end architecture rtl;

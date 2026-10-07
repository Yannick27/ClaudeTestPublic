-------------------------------------------------------------------------------
-- EcsLinearizer
--
--   Normalises one ECS period and linearises it with a Chebyshev polynomial:
--
--     x    = PeriodxDI * GainxDI + OffsetxDI              (|x| <= 1)
--     Lin  = T1(x)*c1 + T2(x)*c2 + ... + TORDER(x)*cORDER
--     T0 = 1, T1 = x, Tk+2 = 2*x*Tk+1 - Tk
--
--   Number formats
--     PeriodxDI  unsigned integer
--     GainxDI    Q-4.36  (raw value / 2**36)
--     OffsetxDI  Q8.24   (raw value / 2**24)
--     x, Tk      signed, 35 bits, T_FRAC = 33 fractional bits (+1.0 = 2**33)
--     CoeffxDI   signed 32 bit integers; only their common scale matters downstream
--     LinxDO     signed, LinWidth(ORDER) bits, T_FRAC fractional bits, i.e.
--                (sum of Tk*ck) * 2**33 with ck taken as plain integers: exact apart
--                from the truncation of Tk (error < 2**-28 relative to max|ck|).
--   P*G + O is evaluated modulo 2**38: the true result is within [-1, 1] and fits,
--   so intermediate wrap-around of P*G and O is harmless (nothing is clamped).
--
--   Sequence : P*G -> x ; then for k = 1..ORDER   sum += Tk * ck
--                                                 Tk+1 = x*Tk / 2**32 - Tk-1
--   One 35x35 multiplier and one 2-clock wide adder, used in turn.
--
--   Timing structure (RTG4, 100 MHz): the coefficients are loaded into a shift
--   register when the computation starts (no ORDER-way multiplexer), and x and Tk
--   are each loaded by a single adder/subtractor only (no multiplexer behind a
--   carry chain).
--
--   Handshake : StartxSI = '1' for one clock (when idle) with the period, gain and
--   offset stable during that clock.  DonexSO = '1' for one clock when LinxDO is
--   valid (it stays valid until the next start).
--   Duration  : 17*ORDER clocks (102 for ORDER = 6).
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.AngleComputePkg.all;

entity EcsLinearizer is
  generic(
    ORDER : positive := 6          -- order of the Chebyshev polynomial
  );
  port(
    ClkxCI    : in  std_logic;
    StartxSI  : in  std_logic;
    PeriodxDI : in  unsigned(23 downto 0);
    GainxDI   : in  signed(31 downto 0);
    OffsetxDI : in  signed(31 downto 0);
    CoeffxDI  : in  ChebyshevCoeff_t(1 to ORDER);
    DonexSO   : out std_logic := '0';
    LinxDO    : out signed(LinWidth(ORDER)-1 downto 0) := (others => '0')
  );
end entity EcsLinearizer;

architecture rtl of EcsLinearizer is

  constant LIN_W     : positive := LinWidth(ORDER);
  constant NORM_FRAC : natural  := 36;             -- fractional bits of Period*Gain (Gain is Q-4.36)
  constant OFF_FRAC  : natural  := 24;             -- fractional bits of Offset      (Offset is Q8.24)
  constant NORM_W    : natural  := NORM_FRAC + 2;  -- sign + 1 integer bit + 36 fractional bits

  type State_t is (S_IDLE, S_NORM,
                   S_DOT_ISSUE, S_DOT_WAIT, S_DOT_ADD,
                   S_REC_ISSUE, S_REC_WAIT);
  signal State : State_t := S_IDLE;

  -- multiplier
  signal MulStart : std_logic := '0';
  signal MulA     : signed(34 downto 0) := (others => '0');
  signal MulB     : signed(34 downto 0) := (others => '0');
  signal MulDone  : std_logic;
  signal MulP     : signed(69 downto 0);

  -- accumulator adder
  signal AddStart : std_logic := '0';
  signal AddA     : signed(LIN_W-1 downto 0) := (others => '0');
  signal AddB     : signed(LIN_W-1 downto 0) := (others => '0');
  signal AddDone  : std_logic;
  signal AddS     : signed(LIN_W-1 downto 0);

  signal X     : signed(T_W-1 downto 0) := (others => '0');   -- x = T1
  signal Tcur  : signed(T_W-1 downto 0) := (others => '0');   -- Tk, k >= 2
  signal Tprev : signed(T_W-1 downto 0) := (others => '0');   -- Tk-1
  signal K     : natural range 1 to ORDER := 1;

  -- coefficients c1..cORDER, c(k) at index 1 while Tk is being accumulated
  signal CoefSR : ChebyshevCoeff_t(1 to ORDER) := (others => (others => '0'));

  -- Offset aligned to the 36 fractional bits of P*G (modulo 2**38) + half an LSB of x (rounding)
  signal OffTerm : signed(NORM_W-1 downto 0) := (others => '0');

begin

  uMul : entity work.MulSigned35
    port map(
      ClkxCI   => ClkxCI,
      StartxSI => MulStart,
      AxDI     => MulA,
      BxDI     => MulB,
      DonexSO  => MulDone,
      PxDO     => MulP);

  uAdd : entity work.WideAddSub
    generic map(W => LIN_W)
    port map(
      ClkxCI   => ClkxCI,
      StartxSI => AddStart,
      SubxSI   => '0',
      AxDI     => AddA,
      BxDI     => AddB,
      DonexSO  => AddDone,
      SxDO     => AddS);

  process(ClkxCI)
    variable Sum : signed(NORM_W-1 downto 0);
  begin
    if rising_edge(ClkxCI) then
      DonexSO  <= '0';
      MulStart <= '0';
      AddStart <= '0';

      case State is

        when S_IDLE =>
          if StartxSI = '1' then
            MulA     <= signed(resize(PeriodxDI, 35));
            MulB     <= resize(GainxDI, 35);
            MulStart <= '1';
            OffTerm  <= OffsetxDI(NORM_W-(NORM_FRAC-OFF_FRAC)-1 downto 0) &
                        to_signed(2**(NORM_FRAC-T_FRAC-1), NORM_FRAC-OFF_FRAC);
            CoefSR   <= CoeffxDI;
            State    <= S_NORM;
          end if;

        -- x = P*G + O: product (36 fractional bits) + Offset aligned to 36 fractional
        -- bits + half an LSB of x for rounding, modulo 2**38; then drop 3 LSBs.
        when S_NORM =>
          if MulDone = '1' then
            Sum   := MulP(NORM_W-1 downto 0) + OffTerm;
            X     <= Sum(NORM_W-1 downto NORM_FRAC-T_FRAC);
            Tprev <= shift_left(to_signed(1, T_W), T_FRAC);            -- T0 = 1
            K     <= 1;
            State <= S_DOT_ISSUE;
          end if;

        -- sum += Tk * ck   (T1 = x, Tk = Tcur for k >= 2)
        when S_DOT_ISSUE =>
          if K = 1 then
            MulA <= X;
          else
            MulA <= Tcur;
          end if;
          MulB     <= resize(CoefSR(1), 35);
          MulStart <= '1';
          State    <= S_DOT_WAIT;

        -- the running sum is the output register of the adder (AddS), 0 for the first term
        when S_DOT_WAIT =>
          if MulDone = '1' then
            if K = 1 then
              AddA <= (others => '0');
            else
              AddA <= AddS;
            end if;
            AddB     <= resize(MulP, LIN_W);
            AddStart <= '1';
            for i in 1 to ORDER-1 loop
              CoefSR(i) <= CoefSR(i+1);
            end loop;
            State    <= S_DOT_ADD;
          end if;

        when S_DOT_ADD =>
          if AddDone = '1' then
            if K = ORDER then
              LinxDO  <= AddS;
              DonexSO <= '1';
              State   <= S_IDLE;
            else
              State <= S_REC_ISSUE;
            end if;
          end if;

        -- Tk+1 = 2*x*Tk - Tk-1 : x*Tk has 66 fractional bits, 2*x*Tk / 2**33 = x*Tk / 2**32.
        -- The 35-bit slice and subtraction wrap modulo 2**35; Tk+1 itself is within
        -- [-1, 1] (plus a few LSB of rounding error), so the result is exact.
        when S_REC_ISSUE =>
          MulA <= X;
          if K = 1 then
            MulB <= X;
          else
            MulB <= Tcur;
          end if;
          MulStart <= '1';
          State    <= S_REC_WAIT;

        when S_REC_WAIT =>
          if MulDone = '1' then
            Tcur <= MulP(2*T_FRAC downto T_FRAC-1) - Tprev;
            if K = 1 then
              Tprev <= X;
            else
              Tprev <= Tcur;
            end if;
            K     <= K + 1;
            State <= S_DOT_ISSUE;
          end if;

      end case;
    end if;
  end process;

end architecture rtl;

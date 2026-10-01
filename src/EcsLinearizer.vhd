-------------------------------------------------------------------------------
-- EcsLinearizer : one ECS channel
--   1. wait for PeriodValidxSI
--   2. PeriodNorm = Period * Gain + Offset            (saturated to [-1, 1])
--   3. PeriodLin  = sum_{k=1..6} Tk(PeriodNorm) * Coeff(k-1)
--      with T0 = 1, T1 = x, T(k+1) = 2*x*Tk - T(k-1)
-- A single pipelined multiplier is time-shared (12 multiplications).
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.AngleComputePkg.all;

entity EcsLinearizer is
  port(
    ClkxCI         : in  std_logic;
    StartxSI       : in  std_logic;     -- start: clears DonexSO, arms the channel
    PeriodxDI      : in  unsigned(23 downto 0);
    PeriodValidxSI : in  std_logic;
    GainxDI        : in  signed(31 downto 0);
    OffsetxDI      : in  signed(31 downto 0);
    CoeffxDI       : in  ChebyshevCoeff_t;
    DonexSO        : out std_logic := '0';  -- LinxDO valid (held until next start)
    LinxDO         : out signed(LIN_W-1 downto 0) := (others => '0')
  );
end entity EcsLinearizer;

architecture rtl of EcsLinearizer is

  type State_t is (S_IDLE, S_WAIT_VALID,
                   S_NORM_ISSUE, S_NORM_WAIT, S_NORM_ROUND, S_NORM_SHIFT, S_NORM_SAT, S_NORM_INIT,
                   S_TERM_ISSUE, S_TERM_WAIT, S_TERM_ACC,
                   S_REC_ISSUE, S_REC_WAIT, S_REC_ROUND, S_REC_UPDATE,
                   S_LIN_ROUND, S_LIN_SHIFT);
  signal State : State_t := S_IDLE;

  constant ONE : signed(31 downto 0) := to_signed(2**Q_FRAC, 32);   -- 1.0 in Q2.30

  -- multiplier
  signal MulStart : std_logic := '0';
  signal MulA     : signed(31 downto 0) := (others => '0');
  signal MulB     : signed(31 downto 0) := (others => '0');
  signal MulDone  : std_logic;
  signal MulP     : signed(63 downto 0);

  -- datapath registers
  signal Period : unsigned(23 downto 0) := (others => '0');
  signal Tmp    : signed(63 downto 0) := (others => '0');  -- rounded/shifted product
  signal X      : signed(31 downto 0) := (others => '0');  -- normalized period
  signal Tprev  : signed(31 downto 0) := (others => '0');  -- T(k-1)
  signal Tcur   : signed(31 downto 0) := (others => '0');  -- Tk
  signal Acc    : signed(65 downto 0) := (others => '0');
  signal K      : integer range 1 to 6 := 1;

begin

  uMul : entity work.Mul32
    port map(ClkxCI => ClkxCI, StartxSI => MulStart, AxDI => MulA, BxDI => MulB,
             DonexSO => MulDone, PxDO => MulP);

  process(ClkxCI)
    variable V34 : signed(33 downto 0);
    variable V40 : signed(LIN_W-1 downto 0);
  begin
    if rising_edge(ClkxCI) then
      MulStart <= '0';

      case State is

        when S_IDLE =>
          if StartxSI = '1' then
            DonexSO <= '0';
            State   <= S_WAIT_VALID;
          end if;

        when S_WAIT_VALID =>
          if PeriodValidxSI = '1' then
            Period <= PeriodxDI;
            State  <= S_NORM_ISSUE;
          end if;

        ---------------------------------------------------------------------
        -- normalization : X = sat((Period*Gain + Offset) >> (GAIN_FRAC-Q_FRAC))
        ---------------------------------------------------------------------
        when S_NORM_ISSUE =>
          MulA     <= signed(resize(Period, 32));
          MulB     <= GainxDI;
          MulStart <= '1';
          State    <= S_NORM_WAIT;

        when S_NORM_WAIT =>                 -- add offset in the product domain
          if MulDone = '1' then
            Tmp   <= MulP + shift_left(resize(OffsetxDI, 64), GAIN_FRAC-OFFSET_FRAC);
            State <= S_NORM_ROUND;
          end if;

        when S_NORM_ROUND =>
          Tmp   <= Tmp + to_signed(2**(GAIN_FRAC-Q_FRAC-1), 64);   -- rounding
          State <= S_NORM_SHIFT;

        when S_NORM_SHIFT =>
          Tmp   <= shift_right(Tmp, GAIN_FRAC-Q_FRAC);
          State <= S_NORM_SAT;

        when S_NORM_SAT =>                  -- saturate to [-1, 1]
          if Tmp > to_signed(2**Q_FRAC, 64) then
            X <= ONE;
          elsif Tmp < to_signed(-(2**Q_FRAC), 64) then
            X <= -ONE;
          else
            X <= resize(Tmp, 32);
          end if;
          State <= S_NORM_INIT;

        when S_NORM_INIT =>                 -- init recurrence: T0 = 1, T1 = x
          Tprev <= ONE;
          Tcur  <= X;
          K     <= 1;
          Acc   <= (others => '0');
          State <= S_TERM_ISSUE;

        ---------------------------------------------------------------------
        -- term : Acc += Tk * Coeff(k-1)
        ---------------------------------------------------------------------
        when S_TERM_ISSUE =>
          MulA     <= Tcur;
          MulB     <= CoeffxDI(K-1);
          MulStart <= '1';
          State    <= S_TERM_WAIT;

        when S_TERM_WAIT =>
          if MulDone = '1' then
            Tmp   <= MulP;
            State <= S_TERM_ACC;
          end if;

        when S_TERM_ACC =>
          Acc <= Acc + resize(Tmp, 66);
          if K = 6 then
            State <= S_LIN_ROUND;
          else
            State <= S_REC_ISSUE;
          end if;

        ---------------------------------------------------------------------
        -- recurrence : T(k+1) = 2*x*Tk - T(k-1)
        ---------------------------------------------------------------------
        when S_REC_ISSUE =>
          MulA     <= X;
          MulB     <= Tcur;
          MulStart <= '1';
          State    <= S_REC_WAIT;

        when S_REC_WAIT =>
          if MulDone = '1' then
            Tmp   <= MulP + to_signed(2**(Q_FRAC-2), 64);   -- rounding of >>(Q_FRAC-1)
            State <= S_REC_ROUND;
          end if;

        when S_REC_ROUND =>
          Tmp   <= shift_right(Tmp, Q_FRAC-1);              -- 2*x*Tk in Q2.30
          State <= S_REC_UPDATE;

        when S_REC_UPDATE =>
          V34   := resize(Tmp, 34) - resize(Tprev, 34);
          Tprev <= Tcur;
          Tcur  <= resize(V34, 32);
          K     <= K + 1;
          State <= S_TERM_ISSUE;

        ---------------------------------------------------------------------
        -- result : Lin = round(Acc / 2**Q_FRAC)
        ---------------------------------------------------------------------
        when S_LIN_ROUND =>
          Acc   <= Acc + to_signed(2**(Q_FRAC-1), 66);
          State <= S_LIN_SHIFT;

        when S_LIN_SHIFT =>
          V40     := resize(shift_right(Acc, Q_FRAC), LIN_W);
          LinxDO  <= V40;
          DonexSO <= '1';
          State   <= S_IDLE;

      end case;
    end if;
  end process;

end architecture rtl;

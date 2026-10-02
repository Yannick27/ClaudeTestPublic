-------------------------------------------------------------------------------
-- PeriodNormLin : normalizes a measured period to [-1, 1] and linearizes it
--                 with a 6th order Chebyshev polynomial.
--
--   PeriodNorm = PeriodEcsxDI * GainNormxDI + OffsetNormxDI
--   PeriodxDO  = sum(k = 1..6) Tk(PeriodNorm) * ChebyshevCoeffxDI(k)
--   with T0 = 1, T1 = X, Tk+2 = 2*X*Tk+1 - Tk
--
-- Fixed-point formats
--   PeriodEcsxDI      : unsigned integer
--   GainNormxDI       : Q-4.36, value = raw * 2**-36
--   OffsetNormxDI     : Q8.24,  value = raw * 2**-24
--   ChebyshevCoeffxDI : Q8.24
--   PeriodNorm, Tk    : Q2.30 (internal, 1.0 = 2**30)
--   PeriodxDO         : Q8.24
--
-- Architecture
--   One shared pipelined multiply-accumulate unit  P = A*B + C  (32x32 signed
--   -> 64 bit, modulo 2**64) and a small FSM that sequences the 12 operations:
--     step  0     : PeriodNorm = Period*Gain + Offset
--     steps 1..5  : T2..T6     = 2*X*Tk+1 - Tk
--     steps 6..11 : output     = sum Tk*Ck   (accumulated through the C input)
--   Rounding constants are folded into the C input, so every result is
--   rounded to nearest, not truncated.
--
--   The 32x32 multiplier is split in four 16x16 partial products (each maps
--   on one 18x18 RTG4 MACC) followed by small pipelined adders (<= 48 bit), so
--   100 MHz is not limited by a wide carry chain.  Latency of the MAC is 4
--   clock cycles, the whole computation takes 12*(1+4+1)+1 = 73 clock cycles.
--
-- No saturation: all arithmetic is modulo 2**64 and the true results always
-- fit in the extracted bit fields, so wrap-around of intermediate values is
-- harmless.
--
-- Handshake: the computation starts when PeriodEcsValidxSI = '1' while idle.
--   ValidxSO is cleared at the start and set together with the new PeriodxDO
--   at the end.  It stays set until the next computation starts.  A new
--   computation requires PeriodEcsValidxSI to go low and high again.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.PeriodNormLinPkg.all;

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

  constant ONE_Q30 : signed(31 downto 0) := to_signed(2**30, 32);

  -- FSM -----------------------------------------------------------------
  type State_t is (IDLE, ISSUE, WAITMUL, REARM);
  signal StatexD : State_t := IDLE;
  signal StepxD  : integer range 0 to 11 := 0;

  -- Chebyshev polynomials T1..T6 (Q2.30), T1 = normalized period
  type TArr_t is array (1 to 6) of signed(31 downto 0);
  signal TxD   : TArr_t := (others => (others => '0'));
  signal AccxD : signed(63 downto 0) := (others => '0');

  -- Multiply-accumulate unit  P = A*B + C -------------------------------
  signal MulAxD     : signed(31 downto 0) := (others => '0');
  signal MulBxD     : signed(31 downto 0) := (others => '0');
  signal MulCxD     : signed(63 downto 0) := (others => '0');
  signal MulStartxS : std_logic := '0';
  signal MulVldxS   : std_logic_vector(3 downto 0) := (others => '0');

  -- stage 1: partial products  A = Ah*2**16 + Al,  B = Bh*2**16 + Bl
  --          (Ah, Bh signed 16 bit; Al, Bl unsigned 16 bit)
  signal PpLLxD : unsigned(31 downto 0) := (others => '0'); -- Al*Bl
  signal PpLHxD : signed(32 downto 0)   := (others => '0'); -- Al*Bh
  signal PpHLxD : signed(32 downto 0)   := (others => '0'); -- Ah*Bl
  signal PpHHxD : signed(31 downto 0)   := (others => '0'); -- Ah*Bh
  signal C1xD   : signed(63 downto 0)   := (others => '0');
  -- stage 2
  signal MidxD    : signed(33 downto 0)   := (others => '0'); -- Ah*Bl + Al*Bh
  signal LowSumxD : unsigned(32 downto 0) := (others => '0'); -- Al*Bl + C(31:0)
  signal HiAxD    : signed(31 downto 0)   := (others => '0'); -- Ah*Bh + C(63:32)
  -- stage 3
  signal HiBxD    : signed(31 downto 0)   := (others => '0'); -- HiA + carry of LowSum
  signal LowBxD   : unsigned(31 downto 0) := (others => '0');
  signal MidBxD   : signed(33 downto 0)   := (others => '0');
  -- stage 4
  signal ProdxD   : signed(63 downto 0)   := (others => '0');

begin

  ---------------------------------------------------------------------------
  -- Pipelined multiply-accumulate
  --   P = A*B + C = Ah*Bh*2**32 + (Ah*Bl + Al*Bh)*2**16 + Al*Bl + C  (mod 2**64)
  ---------------------------------------------------------------------------
  p_mac : process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      MulVldxS <= MulVldxS(2 downto 0) & MulStartxS;

      -- stage 1 : 16x16 partial products
      PpLLxD <= unsigned(MulAxD(15 downto 0)) * unsigned(MulBxD(15 downto 0));
      PpLHxD <= signed(resize(unsigned(MulAxD(15 downto 0)), 17)) * MulBxD(31 downto 16);
      PpHLxD <= MulAxD(31 downto 16) * signed(resize(unsigned(MulBxD(15 downto 0)), 17));
      PpHHxD <= MulAxD(31 downto 16) * MulBxD(31 downto 16);
      C1xD   <= MulCxD;

      -- stage 2 : first additions
      MidxD    <= resize(PpHLxD, 34) + resize(PpLHxD, 34);
      LowSumxD <= resize(PpLLxD, 33) + resize(unsigned(C1xD(31 downto 0)), 33);
      HiAxD    <= PpHHxD + C1xD(63 downto 32);

      -- stage 3 : carry from low word into high word
      HiBxD  <= HiAxD + signed(resize(LowSumxD(32 downto 32), 32));
      LowBxD <= LowSumxD(31 downto 0);
      MidBxD <= MidxD;

      -- stage 4 : add middle term (weight 2**16)
      ProdxD(15 downto 0)  <= signed(LowBxD(15 downto 0));
      ProdxD(63 downto 16) <= (HiBxD & signed(LowBxD(31 downto 16))) + resize(MidBxD, 48);
    end if;
  end process p_mac;

  ---------------------------------------------------------------------------
  -- Sequencer
  ---------------------------------------------------------------------------
  p_fsm : process(ClkxCI)
    variable TPrevxV : signed(31 downto 0);
  begin
    if rising_edge(ClkxCI) then
      MulStartxS <= '0';

      case StatexD is

        when IDLE =>
          if PeriodEcsValidxSI = '1' then
            ValidxSO <= '0';
            StepxD   <= 0;
            StatexD  <= ISSUE;
          end if;

        -- load operands of the current step into the MAC
        when ISSUE =>
          if StepxD = 0 then
            -- Period*Gain (Q.36) + Offset<<12 (Q.36) + 0.5 LSB of Q2.30
            MulAxD <= signed(resize(PeriodEcsxDI, 32));
            MulBxD <= GainNormxDI;
            MulCxD <= shift_left(resize(OffsetNormxDI, 64), 12) or to_signed(2**5, 64);
          elsif StepxD <= 5 then
            -- X*Tk+1 (Q.60) - Tk*2**29 + 0.5 LSB  ->  bits 60:29 = 2*X*Tk+1 - Tk
            if StepxD = 1 then
              TPrevxV := ONE_Q30;                 -- T0
            else
              TPrevxV := TxD(StepxD - 1);
            end if;
            MulAxD <= TxD(1);
            MulBxD <= TxD(StepxD);
            MulCxD <= shift_left(-resize(TPrevxV, 64), 29) or to_signed(2**28, 64);
          else
            -- Tk (Q.30) * Ck (Q.24) = Q.54, accumulated
            MulAxD <= TxD(StepxD - 5);
            MulBxD <= ChebyshevCoeffxDI(StepxD - 5);
            MulCxD <= AccxD;
          end if;
          MulStartxS <= '1';
          StatexD    <= WAITMUL;

        -- wait for the MAC result and store it
        when WAITMUL =>
          if MulVldxS(3) = '1' then
            if StepxD = 0 then
              TxD(1)  <= ProdxD(37 downto 6);                -- Q.36 -> Q2.30
              AccxD   <= to_signed(2**29, 64);               -- 0.5 LSB of Q8.24 output
              StepxD  <= 1;
              StatexD <= ISSUE;
            elsif StepxD <= 5 then
              TxD(StepxD + 1) <= ProdxD(60 downto 29);
              StepxD  <= StepxD + 1;
              StatexD <= ISSUE;
            elsif StepxD <= 10 then
              AccxD   <= ProdxD;
              StepxD  <= StepxD + 1;
              StatexD <= ISSUE;
            else
              PeriodxDO <= ProdxD(61 downto 30);             -- Q.54 -> Q8.24
              ValidxSO  <= '1';
              StatexD   <= REARM;
            end if;
          end if;

        -- require the input valid flag to drop before the next computation
        when REARM =>
          if PeriodEcsValidxSI = '0' then
            StatexD <= IDLE;
          end if;

      end case;
    end if;
  end process p_fsm;

end architecture rtl;

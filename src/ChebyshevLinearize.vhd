-- Normalisation and Chebyshev linearisation of ONE period measurement.
--
--   X   = Period * Gain + Offset                     (|X| <= 1)
--   Lin = T1(X)*C(1) + T2(X)*C(2) + ... + TORDER(X)*C(ORDER)
--   T0 = 1, T1 = X, T(k+1) = 2*X*T(k) - T(k-1)
--
-- Number formats
--   Gain   : Q-4.36 (LSB = 2^-36)      Offset : Q8.24 (LSB = 2^-24)
--   Period : integer
--   X, Tk  : Q2.30  (signed 32 bit, LSB = 2^-30, +1.0 is representable)
--   Lin    : same scale as the Chebyshev coefficients (1 LSB of C = 1 LSB of Lin),
--            LINW = 32 + ceil(log2(ORDER+1)) bits. This is the worst case for
--            sum(Tk*Ck) with |Tk| <= 1, |Ck| <= 2^31, so it cannot overflow.
--            The coefficient scale itself is arbitrary: only Ck and Gamma
--            must share it (it cancels in the final arctan).
--
-- One 32x32 multiplier is shared by all steps (period*gain, X*Tk, Tk*Ck).
-- The result of a computation is a function of the inputs only: the inputs
-- (period, gain, offset, coefficients) are NOT latched and must stay stable
-- from the start request until DonexSO goes high.
--
-- Handshake:
--   StartxSI = '1' (one clock is enough) arms the block and clears DonexSO.
--   The block then waits for PeriodValidxSI = '1', computes, and sets
--   DonexSO = '1' with the result on LinxDO. DonexSO and LinxDO are held
--   until the next StartxSI.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.EcsTypes_pkg.all;

entity ChebyshevLinearize is
  generic(
    ORDER : positive := 6;
    LINW  : positive := 35          -- must be >= 32 + ceil(log2(ORDER+1))
  );
  port(
    ClkxCI         : in  std_logic;
    StartxSI       : in  std_logic;
    PeriodValidxSI : in  std_logic;
    PeriodxDI      : in  unsigned(23 downto 0);
    GainxDI        : in  signed(31 downto 0);                -- Q-4.36
    OffsetxDI      : in  signed(31 downto 0);                -- Q8.24
    CoeffxDI       : in  ChebyshevCoeff_t(1 to ORDER);
    DonexSO        : out std_logic := '0';
    LinxDO         : out signed(LINW-1 downto 0) := (others => '0')
  );
end entity ChebyshevLinearize;

architecture rtl of ChebyshevLinearize is

  function clog2(n : positive) return natural is
    variable r : natural := 0;
    variable v : positive := 1;
  begin
    while v < n loop
      v := v * 2;
      r := r + 1;
    end loop;
    return r;
  end function clog2;

  constant TFRAC  : natural := 30;                  -- fractional bits of X and Tk
  constant GUARD  : natural := 4;                   -- extra fractional bits kept in the accumulator
  constant TSHIFT : natural := TFRAC - GUARD;       -- product bits dropped before accumulation
  constant ACCW   : natural := LINW + GUARD;
  constant T_ONE  : signed(31 downto 0) := to_signed(2**TFRAC, 32);
  -- 0.5 LSB of Q2.30 expressed in the Q.36 domain (2^5), placed in the 12 LSBs
  -- that are zero after aligning the Q8.24 offset to Q.36 -> free rounding.
  constant RND_Q36 : signed(11 downto 0) := to_signed(2**5, 12);

  type state_t is (S_IDLE, S_WAIT_VALID,
                   S_NORM_ISSUE, S_NORM_WAIT,
                   S_COEF_ISSUE, S_COEF_WAIT,
                   S_REC_ISSUE,  S_REC_WAIT, S_REC_SUB,
                   S_FINISH);
  signal StateR : state_t := S_IDLE;

  signal KxD     : integer range 1 to ORDER := 1;   -- index of the current Chebyshev term
  signal XxD     : signed(31 downto 0);             -- normalised period (Q2.30)
  signal TCurxD  : signed(31 downto 0);             -- T(k)
  signal TPrevxD : signed(31 downto 0);             -- T(k-1)
  signal HalfxD  : signed(34 downto 0);             -- round(2*X*T(k)) in Q2.30
  signal AccxD   : signed(ACCW-1 downto 0);

  signal MulValidInxS  : std_logic;
  signal MulValidOutxS : std_logic;
  signal MulAxD        : signed(31 downto 0);
  signal MulBxD        : signed(31 downto 0);
  signal MulPxD        : signed(63 downto 0);

begin

  assert LINW >= 32 + clog2(ORDER + 1)
    report "ChebyshevLinearize: LINW too small, the sum of the Chebyshev terms could overflow"
    severity failure;

  ---------------------------------------------------------------------------
  -- shared multiplier and its operand selection
  ---------------------------------------------------------------------------
  u_mul : entity work.MulSigned
    generic map(WA => 32, WB => 32)
    port map(
      ClkxCI   => ClkxCI,
      ValidxSI => MulValidInxS,
      AxDI     => MulAxD,
      BxDI     => MulBxD,
      ValidxSO => MulValidOutxS,
      ProdxDO  => MulPxD
    );

  MulValidInxS <= '1' when (StateR = S_NORM_ISSUE) or (StateR = S_COEF_ISSUE) or
                           (StateR = S_REC_ISSUE) else '0';

  MulAxD <= signed(resize(PeriodxDI, 32)) when StateR = S_NORM_ISSUE else   -- Period * Gain
            TCurxD                        when StateR = S_COEF_ISSUE else   -- T(k) * C(k)
            XxD;                                                            -- X * T(k)

  MulBxD <= GainxDI                       when StateR = S_NORM_ISSUE else
            CoeffxDI(KxD)                 when StateR = S_COEF_ISSUE else
            TCurxD;

  ---------------------------------------------------------------------------
  -- control and data path
  ---------------------------------------------------------------------------
  process(ClkxCI)
    variable NormSumxV : signed(37 downto 0);
    variable AccSumxV  : signed(ACCW downto 0);
    variable RecExtxV  : signed(35 downto 0);
    variable RecDiffxV : signed(34 downto 0);
    variable FinSumxV  : signed(ACCW-1 downto 0);
  begin
    if rising_edge(ClkxCI) then
      case StateR is

        when S_IDLE =>
          if StartxSI = '1' then
            DonexSO <= '0';
            StateR  <= S_WAIT_VALID;
          end if;

        when S_WAIT_VALID =>
          if PeriodValidxSI = '1' then
            StateR <= S_NORM_ISSUE;
          end if;

        -- X = Period*Gain + Offset ------------------------------------------
        when S_NORM_ISSUE =>
          StateR <= S_NORM_WAIT;

        when S_NORM_WAIT =>
          if MulValidOutxS = '1' then
            -- Product (Q.36) + Offset aligned to Q.36 (<<12) + 0.5 LSB of Q2.30.
            -- |X| <= 1 by design, so X fits in 38 bits (sign, 1 integer bit, 36
            -- fractional bits): the bits above are sign copies and the 38-bit
            -- sum is exact. Keep the 32 MSBs -> Q2.30.
            NormSumxV := MulPxD(37 downto 0) + (OffsetxDI(25 downto 0) & RND_Q36);
            XxD     <= NormSumxV(37 downto 6);
            TCurxD  <= NormSumxV(37 downto 6);          -- T1 = X
            TPrevxD <= T_ONE;                           -- T0 = 1
            KxD     <= 1;
            AccxD   <= (others => '0');
            StateR  <= S_COEF_ISSUE;
          end if;

        -- Acc += T(k) * C(k) --------------------------------------------------
        when S_COEF_ISSUE =>
          StateR <= S_COEF_WAIT;

        when S_COEF_WAIT =>
          if MulValidOutxS = '1' then
            -- |T(k)*C(k)| <= 2^61: bits 63..62 are sign copies. Drop TSHIFT LSBs
            -- with round-to-nearest (the first dropped bit is added through the
            -- extra LSB column of the adder, i.e. as a carry-in).
            AccSumxV := (AccxD & '1') + (resize(MulPxD(62 downto TSHIFT), ACCW) & MulPxD(TSHIFT-1));
            AccxD <= AccSumxV(ACCW downto 1);
            if KxD = ORDER then
              StateR <= S_FINISH;
            else
              StateR <= S_REC_ISSUE;
            end if;
          end if;

        -- T(k+1) = 2*X*T(k) - T(k-1) --------------------------------------------
        when S_REC_ISSUE =>
          StateR <= S_REC_WAIT;

        when S_REC_WAIT =>
          if MulValidOutxS = '1' then
            -- X*T(k) is Q4.60; 2*X*T(k) in Q2.30 is the product >> 29, rounded.
            RecExtxV := MulPxD(63 downto 28) + 1;
            HalfxD   <= RecExtxV(35 downto 1);
            StateR   <= S_REC_SUB;
          end if;

        when S_REC_SUB =>
          -- |T(k+1)| <= 1 by definition of the Chebyshev polynomials, so the
          -- difference fits in 32 bits (only the sign copies are dropped).
          RecDiffxV := HalfxD - resize(TPrevxD, 35);
          TPrevxD <= TCurxD;
          TCurxD  <= RecDiffxV(31 downto 0);
          KxD     <= KxD + 1;
          StateR  <= S_COEF_ISSUE;

        -- Lin = round(Acc / 2^GUARD) ------------------------------------------
        when S_FINISH =>
          FinSumxV := AccxD + 2**(GUARD - 1);
          LinxDO  <= FinSumxV(ACCW-1 downto GUARD);
          DonexSO <= '1';
          StateR  <= S_IDLE;

      end case;
    end if;
  end process;

end architecture rtl;

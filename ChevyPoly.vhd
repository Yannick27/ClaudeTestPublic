-- ChevyPoly: period normalisation + Chebyshev linearisation (T1..TORDER)
--
-- Architecture : one shared, pipelined signed multiplier (maps to RTG4 MACC
--                blocks) driven by a small FSM. Latency is not critical, so
--                resources are minimised and timing at 100 MHz is easy.
--
-- Number formats
--   PeriodEcsxDI   : unsigned integer (24 bit)
--   GainNormxDI    : 32 bit, 36 fractional bits  (Q-4.36)
--   OffsetNormxDI  : 32 bit, 24 fractional bits  (Q8.24)
--   PeriodNorm     : Q2.36 (38 bit), computed modulo 2**38 (always in [-1,1])
--   X, Tk (internal): Q2.INTF, INTF = FRAC + 8 (capped at 36)
--   ChebyshevCoeff : 32 bit, FRAC fractional bits
--   PeriodxDO      : 32 bit, FRAC fractional bits (rounded to nearest)
--
-- No clamping / saturation anywhere; all wrap-around-free by design (the
-- 2-bit-integer intermediate values are modular, the final values fit).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package ChevyPolyPkg is
  type ChebyshevCoeff_t is array (natural range <>) of signed(31 downto 0); -- (Q.FRAC)
end package ChevyPolyPkg;

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

  function log2ceil(n : positive) return natural is
    variable r : natural := 0;
    variable v : natural := 1;
  begin
    while v < n loop
      v := v * 2;
      r := r + 1;
    end loop;
    return r;
  end function;

  constant GAIN_FRAC : natural := 36;                 -- fractional bits of GainNorm
  constant OFFS_FRAC : natural := 24;                 -- fractional bits of OffsetNorm
  constant NORM_W    : natural := GAIN_FRAC + 2;      -- Q2.36 PeriodNorm width
  constant INTF      : natural := minimum(FRAC + 8, GAIN_FRAC); -- internal fractional bits
  constant XW        : natural := INTF + 2;           -- Q2.INTF width
  constant MW        : natural := maximum(XW, 32);    -- multiplier operand width
  constant PW        : natural := 2 * MW;             -- product width
  constant AW        : natural := XW + 32 + log2ceil(ORDER) + 1; -- accumulator width

  type state_t is (S_IDLE, S_WAIT, S_NORM_USE, S_ACC_ISSUE, S_ACC_USE,
                   S_REC_ISSUE, S_REC_USE, S_DONE);

  signal StatexD : state_t := S_IDLE;
  signal NextxD  : state_t := S_IDLE;   -- state to enter after S_WAIT
  signal WCntxD  : natural range 0 to 1 := 0;
  signal KxD     : integer range 1 to ORDER := 1;

  -- shared multiplier: input regs -> product reg -> output reg
  signal AxD  : signed(MW - 1 downto 0) := (others => '0');
  signal BxD  : signed(MW - 1 downto 0) := (others => '0');
  signal P1xD : signed(PW - 1 downto 0) := (others => '0');
  signal P2xD : signed(PW - 1 downto 0) := (others => '0');

  signal XxD     : signed(XW - 1 downto 0) := (others => '0'); -- normalised period
  signal TcurxD  : signed(XW - 1 downto 0) := (others => '0'); -- Tk
  signal TprevxD : signed(XW - 1 downto 0) := (others => '0'); -- Tk-1
  signal AccxD   : signed(AW - 1 downto 0) := (others => '0');

begin

  process(ClkxCI)
    variable vSum  : signed(PW - 1 downto 0);
    variable vProd : signed(PW - 1 downto 0);
    variable vT    : signed(XW - 1 downto 0);
  begin
    if rising_edge(ClkxCI) then
      ValidxSO <= '0';

      -- free running multiplier pipeline
      P1xD <= AxD * BxD;
      P2xD <= P1xD;

      case StatexD is

        when S_IDLE =>
          if PeriodEcsValidxSI = '1' then
            AxD     <= resize(signed('0' & PeriodEcsxDI), MW);
            BxD     <= resize(GainNormxDI, MW);
            NextxD  <= S_NORM_USE;
            WCntxD  <= 0;
            StatexD <= S_WAIT;
          end if;

        -- P2 valid 2 cycles after the operands were registered
        when S_WAIT =>
          if WCntxD = 1 then
            StatexD <= NextxD;
          else
            WCntxD <= WCntxD + 1;
          end if;

        -- PeriodNorm = Period*Gain + Offset  (Q.36, modulo 2**38, result in [-1,1])
        when S_NORM_USE =>
          vSum := P2xD + shift_left(resize(OffsetNormxDI, PW), GAIN_FRAC - OFFS_FRAC);
          XxD     <= vSum(NORM_W - 1 downto GAIN_FRAC - INTF);   -- Q2.INTF
          TcurxD  <= vSum(NORM_W - 1 downto GAIN_FRAC - INTF);   -- T1 = X
          TprevxD <= shift_left(to_signed(1, XW), INTF);         -- T0 = 1
          AccxD   <= shift_left(to_signed(1, AW), INTF - 1);     -- rounding offset
          KxD     <= 1;
          StatexD <= S_ACC_ISSUE;

        -- acc += Tk * Ck
        when S_ACC_ISSUE =>
          AxD     <= resize(TcurxD, MW);
          BxD     <= resize(ChebyshevCoeffxDI(KxD), MW);
          NextxD  <= S_ACC_USE;
          WCntxD  <= 0;
          StatexD <= S_WAIT;

        when S_ACC_USE =>
          AccxD <= AccxD + resize(P2xD, AW);
          if KxD = ORDER then
            StatexD <= S_DONE;
          else
            StatexD <= S_REC_ISSUE;
          end if;

        -- Tk+1 = 2*X*Tk - Tk-1
        when S_REC_ISSUE =>
          AxD     <= resize(XxD, MW);
          BxD     <= resize(TcurxD, MW);
          NextxD  <= S_REC_USE;
          WCntxD  <= 0;
          StatexD <= S_WAIT;

        when S_REC_USE =>
          -- product has 2*INTF fractional bits; shift by INTF-1 gives 2*X*Tk (rounded)
          vProd   := P2xD + shift_left(to_signed(1, PW), INTF - 2);
          vT      := resize(shift_right(vProd, INTF - 1), XW);
          TprevxD <= TcurxD;
          TcurxD  <= vT - TprevxD;
          KxD     <= KxD + 1;
          StatexD <= S_ACC_ISSUE;

        when S_DONE =>
          PeriodxDO <= AccxD(INTF + 31 downto INTF);
          ValidxSO  <= '1';
          StatexD   <= S_IDLE;

      end case;
    end if;
  end process;

end architecture rtl;

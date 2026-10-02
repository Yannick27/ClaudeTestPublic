library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package PeriodNormLin_pkg is
  type ChebyshevCoeff_t is array (natural range <>) of signed(31 downto 0);
end package PeriodNormLin_pkg;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.PeriodNormLin_pkg.all;

-- Period normalization + 6th order Chebyshev linearization.
--
-- Single shared 32x32 signed multiplier, 4 clock cycles per multiplication
-- (operand register, product register, rounding stage, write-back stage) so that
-- every register-to-register path is short enough for 100 MHz on RTG4
-- (the 32x32 product is mapped by the synthesizer on the 18x18 MATHBLOCKs).
-- 12 multiplications in total -> 48 clock cycles + 1 start cycle.
--
-- Internal formats:
--   X (normalized period), Tk (Chebyshev polynomials) : Q2.30
--   Coefficients / output                           : Q8.24
entity PeriodNormLin is
  port(
    ClkxCI             : in  std_logic;
    PeriodEcsxDI       : in  unsigned(23 downto 0);
    PeriodEcsValidxSI  : in  std_logic;
    GainNormxDI        : in  signed(31 downto 0);          -- Q-4.36
    OffsetNormxDI      : in  signed(31 downto 0);          -- Q8.24
    ChebyshevCoeffxDI  : in  ChebyshevCoeff_t(1 to 6);     -- Q8.24
    ValidxSO           : out std_logic           := '0';   -- one clock cycle strobe
    PeriodxDO          : out signed(31 downto 0) := (others => '0')  -- Q8.24
  );
end entity PeriodNormLin;

architecture rtl of PeriodNormLin is

  type State_t is (IDLE, NORM, COEF, GEN);
  signal StatexD : State_t := IDLE;
  signal PhasexD : unsigned(1 downto 0) := (others => '0');
  signal IdxxD   : integer range 1 to 6 := 1;

  constant ONE_Q30 : signed(31 downto 0) := to_signed(2**30, 32);

  signal XxD       : signed(31 downto 0) := (others => '0');  -- normalized period, Q2.30
  signal TprevxD   : signed(31 downto 0) := (others => '0');  -- T(k-1), Q2.30
  signal TcurxD    : signed(31 downto 0) := (others => '0');  -- T(k),   Q2.30

  signal MulAxD    : signed(31 downto 0) := (others => '0');
  signal MulBxD    : signed(31 downto 0) := (others => '0');
  signal ProdxD    : signed(63 downto 0) := (others => '0');

  signal NormSumxD : signed(63 downto 0) := (others => '0');  -- Q.36 (period*gain+offset)
  signal GenRndxD  : signed(32 downto 0) := (others => '0');  -- 2*X*Tk, Q3.30
  signal AccxD     : signed(66 downto 0) := (others => '0');  -- sum Tk*Ck, 54 fractional bits

  -- v + 1 if b = '1' (round to nearest after dropping the bits below v)
  function RndAdd(v : signed; b : std_logic) return signed is
  begin
    if b = '1' then
      return v + 1;
    else
      return v;
    end if;
  end function;

begin

  -- Multiplier: operand registers (below) -> product register, no control logic
  -- so that the synthesizer can pack it into MATHBLOCKs.
  MulProc : process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      ProdxD <= MulAxD * MulBxD;
    end if;
  end process MulProc;

  CtrlProc : process(ClkxCI)
    variable XNew_v : signed(31 downto 0);
    variable TNew_v : signed(31 downto 0);
  begin
    if rising_edge(ClkxCI) then
      ValidxSO <= '0';

      case StatexD is

        when IDLE =>
          if PeriodEcsValidxSI = '1' then
            -- Period (<= 2e6, < 2^21) * gain, operands loaded here
            MulAxD  <= signed(resize(PeriodEcsxDI, 32));
            MulBxD  <= GainNormxDI;
            PhasexD <= to_unsigned(1, 2);
            StatexD <= NORM;
          end if;

        -- X = Period*Gain + Offset
        when NORM =>
          case to_integer(PhasexD) is
            when 1 =>                    -- product being registered
              PhasexD <= to_unsigned(2, 2);
            when 2 =>                    -- add offset (Q8.24 -> Q.36 : << 12)
              NormSumxD <= ProdxD + shift_left(resize(OffsetNormxDI, 64), 12);
              PhasexD   <= to_unsigned(3, 2);
            when others =>               -- Q.36 -> Q2.30 with rounding
              XNew_v   := RndAdd(NormSumxD(37 downto 6), NormSumxD(5));
              XxD      <= XNew_v;
              TcurxD   <= XNew_v;         -- T1 = X
              TprevxD  <= ONE_Q30;        -- T0 = 1
              AccxD    <= (others => '0');
              IdxxD    <= 1;
              PhasexD  <= (others => '0');
              StatexD  <= COEF;
          end case;

        -- Acc += Tk * Ck
        when COEF =>
          case to_integer(PhasexD) is
            when 0 =>
              MulAxD  <= TcurxD;
              MulBxD  <= ChebyshevCoeffxDI(IdxxD);
              PhasexD <= to_unsigned(1, 2);
            when 1 =>
              PhasexD <= to_unsigned(2, 2);
            when 2 =>
              AccxD   <= AccxD + resize(ProdxD, AccxD'length);
              PhasexD <= to_unsigned(3, 2);
            when others =>
              PhasexD <= (others => '0');
              if IdxxD = 6 then
                -- Q.54 -> Q8.24 with rounding
                PeriodxDO <= RndAdd(AccxD(61 downto 30), AccxD(29));
                ValidxSO  <= '1';
                StatexD   <= IDLE;
              else
                StatexD <= GEN;
              end if;
          end case;

        -- T(k+1) = 2*X*Tk - T(k-1)
        when GEN =>
          case to_integer(PhasexD) is
            when 0 =>
              MulAxD  <= XxD;
              MulBxD  <= TcurxD;
              PhasexD <= to_unsigned(1, 2);
            when 1 =>
              PhasexD <= to_unsigned(2, 2);
            when 2 =>                    -- 2*X*Tk : Q.60 * 2 -> Q.30, rounded
              GenRndxD <= RndAdd(ProdxD(61 downto 29), ProdxD(28));
              PhasexD  <= to_unsigned(3, 2);
            when others =>
              TNew_v  := resize(GenRndxD - resize(TprevxD, 33), 32);
              TprevxD <= TcurxD;
              TcurxD  <= TNew_v;
              IdxxD   <= IdxxD + 1;
              PhasexD <= (others => '0');
              StatexD <= COEF;
          end case;

      end case;
    end if;
  end process CtrlProc;

end architecture rtl;

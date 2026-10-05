-------------------------------------------------------------------------------
-- EcsLinearizer
--   Normalizes one ECS period and linearizes it with a Chebyshev series:
--
--     X   = Period * GainNorm + OffsetNorm            (|X| <= 1)
--     Lin = sum_{k=1..ORDER} Tk(X) * Coeff(k)
--
--   Tk are never built explicitly: the series is evaluated with Clenshaw's
--   recurrence (one multiplication per order, numerically stable):
--     b(k) = Coeff(k) + 2*X*b(k+1) - b(k+2)      k = ORDER .. 1
--     Lin  = X*b(1) - b(2)                       (Coeff(0) = 0)
--
--   Number formats
--     X          : signed 38 bit, 36 fractional bits (Q2.36, 1.0 = 2**36),
--                  i.e. Period*Gain is kept exact (no rounding)
--     b(k), Lin  : same scale as Coeff (the series is linear in Coeff), with
--                  enough guard bits to never overflow
--                  (|b(k)| <= ORDER*(ORDER+1)/2 * 2**31).
--                  Lin is LinWidth(ORDER) bits wide.
--   -> Coeff may have any fixed-point format; Lin has the same one.
--
--   One shared SerialMul does all products, ORDER+2 multiplications/call.
--   Start with a 1-cycle pulse, inputs must stay stable until DonexSO.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.AngleComputePkg.all;

entity EcsLinearizer is
  generic(ORDER : positive := 6);
  port(
    ClkxCI            : in  std_logic;
    StartxSI          : in  std_logic;
    DonexSO           : out std_logic := '0';
    PeriodxDI         : in  unsigned(23 downto 0);
    GainNormxDI       : in  signed(31 downto 0);  -- Q-4.36
    OffsetNormxDI     : in  signed(31 downto 0);  -- Q8.24
    ChebyshevCoeffxDI : in  ChebyshevCoeff_t(1 to ORDER);
    PeriodLinxDO      : out signed(LinWidth(ORDER) - 1 downto 0) := (others => '0')
    );
end entity EcsLinearizer;

architecture rtl of EcsLinearizer is

  constant X_W : positive := 38;                                   -- Q2.36
  constant WB  : positive := 32 + clog2(ORDER * (ORDER + 1) + 1);  -- Clenshaw word
  constant RW  : positive := X_W + WB;                             -- product width
  constant WL  : positive := LinWidth(ORDER);

  constant RND_2XB : signed(RW - 1 downto 0) := shift_left(to_signed(1, RW), 34);  -- >>35
  constant RND_XB  : signed(RW - 1 downto 0) := shift_left(to_signed(1, RW), 35);  -- >>36

  type state_t is (S_IDLE, S_NORM_WAIT, S_NORM_ADD, S_MUL, S_MUL_WAIT, S_ROUND, S_STEP);
  signal StatexS : state_t := S_IDLE;

  signal MulStartxS : std_logic := '0';
  signal MulDonexS  : std_logic;
  signal MulAxD     : signed(X_W - 1 downto 0) := (others => '0');
  signal MulBxD     : signed(WB - 1 downto 0)  := (others => '0');
  signal MulPxD     : signed(RW - 1 downto 0);

  signal XxD  : signed(X_W - 1 downto 0)  := (others => '0');
  signal B1xD : signed(WB - 1 downto 0)   := (others => '0');  -- b(k+1)
  signal B2xD : signed(WB - 1 downto 0)   := (others => '0');  -- b(k+2)
  signal TxD  : signed(WB - 1 downto 0)   := (others => '0');  -- 2*X*b(k+1) (or X*b(1))
  signal KxD  : integer range 0 to ORDER  := 0;

begin

  u_mul : entity work.SerialMul
    generic map(AW => X_W, BW => WB)
    port map(ClkxCI   => ClkxCI,
             StartxSI => MulStartxS,
             AxDI     => MulAxD,
             BxDI     => MulBxD,
             DonexSO  => MulDonexS,
             PxDO     => MulPxD);

  process(ClkxCI)
    variable SumV  : signed(57 downto 0);  -- Period*Gain (57 b) + Offset, 36 fractional bits
    variable BnewV : signed(WB - 1 downto 0);
  begin
    if rising_edge(ClkxCI) then
      DonexSO    <= '0';
      MulStartxS <= '0';

      case StatexS is

        when S_IDLE =>
          if StartxSI = '1' then
            MulAxD     <= signed(resize(PeriodxDI, X_W));
            MulBxD     <= resize(GainNormxDI, WB);
            MulStartxS <= '1';
            StatexS    <= S_NORM_WAIT;
          end if;

        when S_NORM_WAIT =>
          if MulDonexS = '1' then
            StatexS <= S_NORM_ADD;
          end if;

        when S_NORM_ADD =>
          -- Period*Gain has 36 fractional bits, Offset has 24 -> align, add
          SumV := resize(MulPxD, 58) + shift_left(resize(OffsetNormxDI, 58), 12);
          XxD  <= resize(SumV, X_W);
          KxD  <= ORDER;
          B1xD <= (others => '0');
          B2xD <= (others => '0');
          StatexS <= S_MUL;

        when S_MUL =>
          MulAxD     <= XxD;
          MulBxD     <= B1xD;
          MulStartxS <= '1';
          StatexS    <= S_MUL_WAIT;

        when S_MUL_WAIT =>
          if MulDonexS = '1' then
            StatexS <= S_ROUND;
          end if;

        when S_ROUND =>
          if KxD = 0 then
            TxD <= resize(shift_right(MulPxD + RND_XB, 36), WB);
          else
            TxD <= resize(shift_right(MulPxD + RND_2XB, 35), WB);
          end if;
          StatexS <= S_STEP;

        when S_STEP =>
          if KxD = 0 then
            PeriodLinxDO <= resize(TxD - B2xD, WL);
            DonexSO      <= '1';
            StatexS      <= S_IDLE;
          else
            BnewV := resize(ChebyshevCoeffxDI(KxD), WB) + TxD - B2xD;
            B2xD  <= B1xD;
            B1xD  <= BnewV;
            KxD   <= KxD - 1;
            StatexS <= S_MUL;
          end if;

      end case;
    end if;
  end process;

end architecture rtl;

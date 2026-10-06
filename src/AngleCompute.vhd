-- AngleCompute
--
--   ECS periods 1..4 --(normalise + Chebyshev linearise, 4 x ChebyshevLinearize)--> PeriodLin XP, XN, ZP, ZN
--   DeltaECSX = LinXP - LinXN + GammaX
--   DeltaECSZ = LinZP - LinZN + GammaZ
--   Num = CosThetaZ*DeltaECSX - SinThetaX*DeltaECSZ
--   Den = SinThetaZ*DeltaECSX + CosThetaX*DeltaECSZ
--   Angle = atan2(Num, Den)    in turns, 65536 = 2*pi      (CordicAtan2)
--
-- Channel assignment: 1 = XP, 2 = XN, 3 = ZP, 4 = ZN.
--
-- Architecture
--   * The four channels are independent: each ChebyshevLinearize instance waits
--     for its own PeriodEcsValidxSI(i), so the result does not depend on the order
--     in which the four periods become valid.
--   * Everything after the linearisation is serial and shares one multiplier:
--     4 products (36x32 bit) -> Num / Den -> CORDIC.
--   * The result is available 126..198 clock cycles (1.3..2 us at 100 MHz, measured
--     in simulation) after the last period is valid; the periods only arrive every
--     10..20 ms, so area (not speed) was the design goal. Every state does at most
--     one multiply or one add, to keep the paths short for 100 MHz on RTG4.
--   * The angle is atan2(Num, Den), i.e. the full circle 0..65535 (65536 = 2*pi),
--     not the +-pi/2 range of a plain arctan(Num/Den). Counter-clockwise from the
--     +Den axis towards the +Num axis.
--
-- Fixed-point bookkeeping (nothing is clamped or saturated, all widths are the
-- worst case for 32 bit coefficients):
--   LIN_W   = 32 + ceil(log2(ORDER+1))  |sum Tk*Ck|            <  2^(LIN_W-1)
--   DELTA_W = LIN_W + 1                 |LinP - LinN + Gamma|  <  2^(DELTA_W-1)
--   PROD_W  = DELTA_W + 32              Delta * sin/cos
--   VEC_W   = PROD_W                    Num / Den: |Delta| <= (2*ORDER+1)*2^31 and
--                                       |sin|,|cos| <= 2^31 give |Num|,|Den| <=
--                                       (2*ORDER+1)*2^63 < 2^(PROD_W-1), so the sum of
--                                       the two products needs no extra bit
-- The scale of the Chebyshev coefficients / Gamma and of the sin/cos inputs is
-- arbitrary (each pair only needs a common scale): both cancel in the arctangent.
--
-- Control: StartxSI is sampled while idle (level sensitive). ValidxSO is one
-- clock wide and coincident with the new AnglexDO. The inputs must be stable
-- from the start to ValidxSO (they are not latched).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.EcsTypes_pkg.all;

entity AngleCompute is
  generic(ORDER : positive := 6                                              -- order of Chevychev polynomial
  );
  port(
    ClkxCI : in std_logic;                                                   -- 100 MHz clock
    -- Internal control
    StartxSI : in  std_logic;                                                -- start computation
    ValidxSO : out std_logic := '0';                                         -- angle is valid
    AnglexDO : out unsigned(15 downto 0) := (others => '0');                 -- computed angle on full circle ( 65536 = 2π)
    -- Frequency measurements ----
    PeriodEcsxDI      : in EcsPeriod_t(1 to 4);                              -- Periods of all ECSs (always between 1000000 and 2000000)
    PeriodEcsValidxSI : in std_logic_vector(1 to 4);                         -- Valid signal for the periods of all ECS
    -- Parameters ------
    GainNormxDI       : in EcsGain_t(1 to 4);                                -- gain to normalize periods between -1 and 1 --format Q-4.36
    OffsetNormxDI     : in EcsOffset_t(1 to 4);                              -- offset to normalize periods between -1 and 1 --format Q8.24
    ChebyshevCoeffxDI : in ChebyshevCoeffArray_t(1 to 4)(1 to ORDER);        -- Chebyshev coefficients for linearization periods
    GammaXxDI         : in signed(31 downto 0);                              -- Chebyshev offset on X axis
    GammaZxDI         : in signed(31 downto 0);                              -- Chebyshev offset on Z axis
    CosThetaXxDI      : in signed(31 downto 0);                              -- Cos(ThetaX) (always between -1 and 1)
    SinThetaXxDI      : in signed(31 downto 0);                              -- Sin(ThetaX) (always between -1 and 1)
    CosThetaZxDI      : in signed(31 downto 0);                              -- Cos(ThetaZ) (always between -1 and 1)
    SinThetaZxDI      : in signed(31 downto 0)                               -- Sin(ThetaZ) (always between -1 and 1)
  );
end entity AngleCompute;

architecture rtl of AngleCompute is

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

  constant LIN_W   : positive := 32 + clog2(ORDER + 1);
  constant DELTA_W : positive := LIN_W + 1;
  constant PROD_W  : positive := DELTA_W + 32;
  constant VEC_W   : positive := PROD_W;

  type LinArray_t is array (1 to 4) of signed(LIN_W-1 downto 0);

  type state_t is (S_IDLE, S_START, S_WAIT_LIN, S_DELTA1, S_DELTA2,
                   S_MUL_ISSUE, S_MUL_WAIT, S_ANG_START, S_ANG_WAIT);
  signal StateR : state_t := S_IDLE;

  -- linearisation channels
  signal StartLinxS : std_logic;
  signal LinDonexS  : std_logic_vector(1 to 4);
  signal LinxD      : LinArray_t;

  -- linearised differences
  signal DeltaXxD : signed(DELTA_W-1 downto 0);
  signal DeltaZxD : signed(DELTA_W-1 downto 0);

  -- shared multiplier: 0: DX*CosZ  1: DZ*SinX  2: DX*SinZ  3: DZ*CosX
  signal OpxD          : integer range 0 to 3 := 0;
  signal MulValidInxS  : std_logic;
  signal MulValidOutxS : std_logic;
  signal MulAxD        : signed(DELTA_W-1 downto 0);
  signal MulBxD        : signed(31 downto 0);
  signal MulPxD        : signed(PROD_W-1 downto 0);

  -- vector handed to the CORDIC
  signal NumxD : signed(VEC_W-1 downto 0);
  signal DenxD : signed(VEC_W-1 downto 0);

  signal StartAngxS : std_logic;
  signal AngDonexS  : std_logic;
  signal AngxD      : unsigned(15 downto 0);

begin

  ---------------------------------------------------------------------------
  -- 4 independent channels: normalisation + Chebyshev linearisation
  ---------------------------------------------------------------------------
  StartLinxS <= '1' when StateR = S_START else '0';

  g_lin : for i in 1 to 4 generate
    u_lin : entity work.ChebyshevLinearize
      generic map(ORDER => ORDER, LINW => LIN_W)
      port map(
        ClkxCI         => ClkxCI,
        StartxSI       => StartLinxS,
        PeriodValidxSI => PeriodEcsValidxSI(i),
        PeriodxDI      => PeriodEcsxDI(i),
        GainxDI        => GainNormxDI(i),
        OffsetxDI      => OffsetNormxDI(i),
        CoeffxDI       => ChebyshevCoeffxDI(i),
        DonexSO        => LinDonexS(i),
        LinxDO         => LinxD(i)
      );
  end generate g_lin;

  ---------------------------------------------------------------------------
  -- multiplier shared by the four trigonometric products
  ---------------------------------------------------------------------------
  u_mul : entity work.MulSigned
    generic map(WA => DELTA_W, WB => 32)
    port map(
      ClkxCI   => ClkxCI,
      ValidxSI => MulValidInxS,
      AxDI     => MulAxD,
      BxDI     => MulBxD,
      ValidxSO => MulValidOutxS,
      ProdxDO  => MulPxD
    );

  MulValidInxS <= '1' when StateR = S_MUL_ISSUE else '0';

  MulAxD <= DeltaXxD when (OpxD = 0) or (OpxD = 2) else DeltaZxD;

  MulBxD <= CosThetaZxDI when OpxD = 0 else
            SinThetaXxDI when OpxD = 1 else
            SinThetaZxDI when OpxD = 2 else
            CosThetaXxDI;

  ---------------------------------------------------------------------------
  -- angle of the vector (Den, Num)
  ---------------------------------------------------------------------------
  StartAngxS <= '1' when StateR = S_ANG_START else '0';

  u_atan : entity work.CordicAtan2
    generic map(NW => VEC_W)
    port map(
      ClkxCI   => ClkxCI,
      StartxSI => StartAngxS,
      XxDI     => DenxD,
      YxDI     => NumxD,
      DonexSO  => AngDonexS,
      AnglexDO => AngxD
    );

  ---------------------------------------------------------------------------
  -- sequencing
  ---------------------------------------------------------------------------
  process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      ValidxSO <= '0';

      case StateR is

        when S_IDLE =>
          if StartxSI = '1' then
            StateR <= S_START;
          end if;

        -- one-clock start request to the four channels
        when S_START =>
          StateR <= S_WAIT_LIN;

        -- each channel waits for its own PeriodEcsValidxSI(i)
        when S_WAIT_LIN =>
          if LinDonexS = (1 to 4 => '1') then
            StateR <= S_DELTA1;
          end if;

        -- DeltaECS = LinP - LinN + Gamma
        when S_DELTA1 =>
          DeltaXxD <= resize(LinxD(1), DELTA_W) - resize(LinxD(2), DELTA_W);
          DeltaZxD <= resize(LinxD(3), DELTA_W) - resize(LinxD(4), DELTA_W);
          StateR   <= S_DELTA2;

        when S_DELTA2 =>
          DeltaXxD <= DeltaXxD + resize(GammaXxDI, DELTA_W);
          DeltaZxD <= DeltaZxD + resize(GammaZxDI, DELTA_W);
          OpxD     <= 0;
          StateR   <= S_MUL_ISSUE;

        -- Num = CosZ*DX - SinX*DZ ,  Den = SinZ*DX + CosX*DZ
        when S_MUL_ISSUE =>
          StateR <= S_MUL_WAIT;

        when S_MUL_WAIT =>
          if MulValidOutxS = '1' then
            case OpxD is
              when 0      => NumxD <= resize(MulPxD, VEC_W);
              when 1      => NumxD <= NumxD - resize(MulPxD, VEC_W);
              when 2      => DenxD <= resize(MulPxD, VEC_W);
              when others => DenxD <= DenxD + resize(MulPxD, VEC_W);
            end case;
            if OpxD = 3 then
              StateR <= S_ANG_START;
            else
              OpxD   <= OpxD + 1;
              StateR <= S_MUL_ISSUE;
            end if;
          end if;

        when S_ANG_START =>
          StateR <= S_ANG_WAIT;

        when S_ANG_WAIT =>
          if AngDonexS = '1' then
            AnglexDO <= AngxD;
            ValidxSO <= '1';                     -- one clock cycle
            StateR   <= S_IDLE;
          end if;

      end case;
    end if;
  end process;

end architecture rtl;

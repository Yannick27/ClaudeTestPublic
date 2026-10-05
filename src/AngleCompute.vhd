-------------------------------------------------------------------------------
-- AngleCompute
--
--   1. For each ECS 1..4 (in order): wait PeriodEcsValidxSI(n), normalize and
--      linearize the period (EcsLinearizer, Chebyshev / Clenshaw).
--   2. DeltaECSX = LinP(1) - LinN(2) + GammaX
--      DeltaECSZ = LinP(3) - LinN(4) + GammaZ
--   3. Num = CosThetaZ*DeltaECSX - SinThetaX*DeltaECSZ
--      Den = SinThetaZ*DeltaECSX + CosThetaX*DeltaECSZ
--   4. AnglexDO = atan2(Num, Den) on the full circle (65536 = 2*pi, wraps
--      modulo 2*pi), using a vectoring-mode CORDIC.
--
--   Fixed-point conventions
--     * The result only depends on the RATIO Num/Den, so the formats of the
--       Chebyshev coefficients / Gamma (shared by all) and of the Cos/Sin
--       terms (shared by all) do not matter. Nothing is clamped: all widths
--       are derived from ORDER so that no signal can overflow.
--     * Num/Den (WN bits) are first normalized (shifted left together until
--       the larger one uses the full range, no information is lost), then
--       the 28 MSBs feed the CORDIC.
--
--   Timing / resources (RTG4, 100 MHz)
--     * Two SerialMul instances (one in EcsLinearizer, one here), each using
--       a single 18x18 MACC. Every other operation is one add/shift per clock.
--     * Duration: about 650 clock cycles (~6.5 us) for ORDER = 6 (all valids
--       already high); mostly the 18x18 serial multiplications.
--
--   StartxSI is level sensitive in the idle state: it must be released
--   (pulse) before ValidxSO, otherwise a new computation is launched.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.AngleComputePkg.all;

entity AngleCompute is
  generic(ORDER : positive := 6        -- order of Chevychev polynomial
         );
  port(
    ClkxCI             : in  std_logic; -- 100 MHz clock
    -- Internal control
    StartxSI           : in  std_logic;  -- start computation
    ValidxSO           : out std_logic             := '0'; -- angle is valid
    AnglexDO           : out unsigned(15 downto 0) := (others => '0'); -- computed angle on full circle ( 65536 = 2π)
    -- Frequency measurements ----
    PeriodEcsxDI       : in  EcsPeriod_t(1 to 4); -- Periods of all ECSs (always between 1000000 and 2000000)
    PeriodEcsValidxSI  : in  std_logic_vector(1 to 4); -- Valid signal for the periods of all ECS
    -- Parameters ------
    GainNormxDI        : in  EcsGain_t(1 to 4); -- gain to normalize periods between -1 and 1 --format Q-4.36
    OffsetNormxDI      : in  EcsOffset_t(1 to 4); -- offset to normalize periods between -1 and 1 --format Q8.24
    ChebyshevCoeffxDI  : in  ChebyshevCoeffArray_t(1 to 4)(1 to ORDER); -- Chebyshev coefficients for linearization periods
    GammaXxDI          : in  signed(31 downto 0); -- Chebyshev offset on X axis
    GammaZxDI          : in  signed(31 downto 0); -- Chebyshev offset on Z axis
    CosThetaXxDI       : in  signed(31 downto 0); -- Cos(ThetaX) (always between -1 and 1)
    SinThetaXxDI       : in  signed(31 downto 0); -- Sin(ThetaX) (always between -1 and 1)
    CosThetaZxDI       : in  signed(31 downto 0); -- Cos(ThetaZ) (always between -1 and 1)
    SinThetaZxDI       : in  signed(31 downto 0)  -- Sin(ThetaZ) (always between -1 and 1)
  );
end entity AngleCompute;

architecture rtl of AngleCompute is

  ---------------------------------------------------------------------------
  -- Widths
  ---------------------------------------------------------------------------
  constant WL : positive := LinWidth(ORDER);     -- linearized period
  constant WD : positive := DeltaWidth(ORDER);   -- DeltaECS
  constant WP : positive := WD + 32;             -- DeltaECS * Cos/Sin
  constant WN : positive := WP + 1;              -- Num / Den

  -- CORDIC
  constant NIT    : positive := 18;              -- iterations
  constant CIN_W  : positive := 28;              -- MSBs of Num/Den fed to CORDIC
  constant CX_W   : positive := 34;              -- CORDIC x/y word (4 guard LSBs, 2 gain MSBs)
  constant ZW     : positive := 24;              -- CORDIC angle word, 2**24 = 2*pi

  type AtanTab_t is array (0 to NIT - 1) of unsigned(ZW - 1 downto 0);
  -- round(atan(2**-i) / (2*pi) * 2**24)
  constant ATAN_TAB : AtanTab_t := (
    to_unsigned(2097152, ZW), to_unsigned(1238021, ZW), to_unsigned(654136, ZW),
    to_unsigned(332050, ZW),  to_unsigned(166669, ZW),  to_unsigned(83416, ZW),
    to_unsigned(41718, ZW),   to_unsigned(20860, ZW),   to_unsigned(10430, ZW),
    to_unsigned(5215, ZW),    to_unsigned(2608, ZW),    to_unsigned(1304, ZW),
    to_unsigned(652, ZW),     to_unsigned(326, ZW),     to_unsigned(163, ZW),
    to_unsigned(81, ZW),      to_unsigned(41, ZW),      to_unsigned(20, ZW));

  constant Z_PI_2    : unsigned(ZW - 1 downto 0) := to_unsigned(2 ** (ZW - 2), ZW);
  constant Z_3PI_2   : unsigned(ZW - 1 downto 0) := to_unsigned(3 * 2 ** (ZW - 2), ZW);
  constant Z_HALFLSB : unsigned(ZW - 1 downto 0) := to_unsigned(2 ** (ZW - 17), ZW);

  ---------------------------------------------------------------------------
  -- FSM
  ---------------------------------------------------------------------------
  type state_t is (S_IDLE, S_WAIT_VALID, S_LIN_WAIT, S_DELTA,
                   S_MUL, S_MUL_WAIT, S_COMBINE, S_NORM,
                   S_PREROT, S_SHIFT, S_ITER, S_OUT);
  signal StatexS : state_t := S_IDLE;

  signal ChxD : integer range 1 to 4 := 1;

  -- linearizer
  signal LinStartxS : std_logic := '0';
  signal LinDonexS  : std_logic;
  signal LinResxD   : signed(WL - 1 downto 0);
  signal LinPerxD   : unsigned(23 downto 0);
  signal LinGainxD  : signed(31 downto 0);
  signal LinOffxD   : signed(31 downto 0);
  signal LinCoeffxD : ChebyshevCoeff_t(1 to ORDER);

  type LinArray_t is array (1 to 4) of signed(WL - 1 downto 0);
  signal LinxD : LinArray_t := (others => (others => '0'));

  signal DeltaXxD : signed(WD - 1 downto 0) := (others => '0');
  signal DeltaZxD : signed(WD - 1 downto 0) := (others => '0');

  -- rotation multiplier
  signal MulStartxS : std_logic := '0';
  signal MulDonexS  : std_logic;
  signal MulAxD     : signed(WD - 1 downto 0) := (others => '0');
  signal MulBxD     : signed(31 downto 0)     := (others => '0');
  signal MulPxD     : signed(WP - 1 downto 0);
  signal MxD        : integer range 0 to 3 := 0;

  type ProdArray_t is array (0 to 3) of signed(WP - 1 downto 0);
  signal ProdxD : ProdArray_t := (others => (others => '0'));

  signal NumxD : signed(WN - 1 downto 0) := (others => '0');
  signal DenxD : signed(WN - 1 downto 0) := (others => '0');
  signal NormCntxD : integer range 0 to WN := 0;

  -- CORDIC
  signal CxxD : signed(CX_W - 1 downto 0) := (others => '0');
  signal CyxD : signed(CX_W - 1 downto 0) := (others => '0');
  signal CzxD : unsigned(ZW - 1 downto 0) := (others => '0');
  signal XsxD : signed(CX_W - 1 downto 0) := (others => '0');
  signal YsxD : signed(CX_W - 1 downto 0) := (others => '0');
  signal ItxD : integer range 0 to NIT - 1 := 0;

begin

  ---------------------------------------------------------------------------
  -- Normalization + Chebyshev linearization of the selected ECS
  ---------------------------------------------------------------------------
  LinPerxD   <= PeriodEcsxDI(ChxD);
  LinGainxD  <= GainNormxDI(ChxD);
  LinOffxD   <= OffsetNormxDI(ChxD);
  LinCoeffxD <= ChebyshevCoeffxDI(ChxD);

  u_lin : entity work.EcsLinearizer
    generic map(ORDER => ORDER)
    port map(ClkxCI            => ClkxCI,
             StartxSI          => LinStartxS,
             DonexSO           => LinDonexS,
             PeriodxDI         => LinPerxD,
             GainNormxDI       => LinGainxD,
             OffsetNormxDI     => LinOffxD,
             ChebyshevCoeffxDI => LinCoeffxD,
             PeriodLinxDO      => LinResxD);

  ---------------------------------------------------------------------------
  -- Shared multiplier of the rotation (DeltaECS * Cos/Sin)
  ---------------------------------------------------------------------------
  u_mul : entity work.SerialMul
    generic map(AW => WD, BW => 32)
    port map(ClkxCI   => ClkxCI,
             StartxSI => MulStartxS,
             AxDI     => MulAxD,
             BxDI     => MulBxD,
             DonexSO  => MulDonexS,
             PxDO     => MulPxD);

  ---------------------------------------------------------------------------
  -- Main sequencer
  ---------------------------------------------------------------------------
  process(ClkxCI)
    variable X0V : signed(CX_W - 1 downto 0);
    variable Y0V : signed(CX_W - 1 downto 0);
    variable ZV  : unsigned(ZW - 1 downto 0);
  begin
    if rising_edge(ClkxCI) then
      ValidxSO   <= '0';
      LinStartxS <= '0';
      MulStartxS <= '0';

      case StatexS is

        when S_IDLE =>
          if StartxSI = '1' then
            ChxD    <= 1;
            StatexS <= S_WAIT_VALID;
          end if;

        -- wait valid period of ECS ChxD, then linearize it
        when S_WAIT_VALID =>
          if PeriodEcsValidxSI(ChxD) = '1' then
            LinStartxS <= '1';
            StatexS    <= S_LIN_WAIT;
          end if;

        when S_LIN_WAIT =>
          if LinDonexS = '1' then
            LinxD(ChxD) <= LinResxD;
            if ChxD = 4 then
              StatexS <= S_DELTA;
            else
              ChxD    <= ChxD + 1;
              StatexS <= S_WAIT_VALID;
            end if;
          end if;

        when S_DELTA =>
          DeltaXxD <= resize(LinxD(1), WD) - resize(LinxD(2), WD) + resize(GammaXxDI, WD);
          DeltaZxD <= resize(LinxD(3), WD) - resize(LinxD(4), WD) + resize(GammaZxDI, WD);
          MxD      <= 0;
          StatexS  <= S_MUL;

        -- four products: CosZ*DX, SinX*DZ, SinZ*DX, CosX*DZ
        when S_MUL =>
          case MxD is
            when 0 =>
              MulAxD <= DeltaXxD;
              MulBxD <= CosThetaZxDI;
            when 1 =>
              MulAxD <= DeltaZxD;
              MulBxD <= SinThetaXxDI;
            when 2 =>
              MulAxD <= DeltaXxD;
              MulBxD <= SinThetaZxDI;
            when 3 =>
              MulAxD <= DeltaZxD;
              MulBxD <= CosThetaXxDI;
          end case;
          MulStartxS <= '1';
          StatexS    <= S_MUL_WAIT;

        when S_MUL_WAIT =>
          if MulDonexS = '1' then
            ProdxD(MxD) <= MulPxD;
            if MxD = 3 then
              StatexS <= S_COMBINE;
            else
              MxD     <= MxD + 1;
              StatexS <= S_MUL;
            end if;
          end if;

        when S_COMBINE =>
          NumxD     <= resize(ProdxD(0), WN) - resize(ProdxD(1), WN);
          DenxD     <= resize(ProdxD(2), WN) + resize(ProdxD(3), WN);
          NormCntxD <= 0;
          StatexS   <= S_NORM;

        -- scale Num/Den together (x2 per clock) until one of them uses the
        -- full range: same ratio, better CORDIC resolution
        when S_NORM =>
          if NumxD(WN - 1) = NumxD(WN - 2) and DenxD(WN - 1) = DenxD(WN - 2)
             and NormCntxD /= WN - 2 then
            NumxD     <= shift_left(NumxD, 1);
            DenxD     <= shift_left(DenxD, 1);
            NormCntxD <= NormCntxD + 1;
          else
            StatexS <= S_PREROT;
          end if;

        -- CORDIC input: x = Den, y = Num; rotate by +-90 deg if x < 0
        when S_PREROT =>
          X0V := shift_left(resize(DenxD(WN - 1 downto WN - CIN_W), CX_W), CX_W - CIN_W - 2);
          Y0V := shift_left(resize(NumxD(WN - 1 downto WN - CIN_W), CX_W), CX_W - CIN_W - 2);
          if X0V(CX_W - 1) = '0' then
            CxxD <= X0V;
            CyxD <= Y0V;
            CzxD <= (others => '0');
          elsif Y0V(CX_W - 1) = '0' then      -- 2nd quadrant: -90 deg
            CxxD <= Y0V;
            CyxD <= -X0V;
            CzxD <= Z_PI_2;
          else                                -- 3rd quadrant: +90 deg
            CxxD <= -Y0V;
            CyxD <= X0V;
            CzxD <= Z_3PI_2;
          end if;
          ItxD    <= 0;
          StatexS <= S_SHIFT;

        when S_SHIFT =>
          XsxD    <= shift_right(CxxD, ItxD);
          YsxD    <= shift_right(CyxD, ItxD);
          StatexS <= S_ITER;

        -- vectoring mode: drive y to 0, accumulate the angle in z
        when S_ITER =>
          if CyxD(CX_W - 1) = '0' then
            CxxD <= CxxD + YsxD;
            CyxD <= CyxD - XsxD;
            CzxD <= CzxD + ATAN_TAB(ItxD);
          else
            CxxD <= CxxD - YsxD;
            CyxD <= CyxD + XsxD;
            CzxD <= CzxD - ATAN_TAB(ItxD);
          end if;
          if ItxD = NIT - 1 then
            StatexS <= S_OUT;
          else
            ItxD    <= ItxD + 1;
            StatexS <= S_SHIFT;
          end if;

        when S_OUT =>
          ZV       := CzxD + Z_HALFLSB;                -- round 24 -> 16 bit
          AnglexDO <= ZV(ZW - 1 downto ZW - 16);
          ValidxSO <= '1';
          StatexS  <= S_IDLE;

      end case;
    end if;
  end process;

end architecture rtl;

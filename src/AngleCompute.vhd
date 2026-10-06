-------------------------------------------------------------------------------
-- AngleCompute
-- Computes the sensor angle from the periods of four ECSs.
--
--   for ECS 1..4 (one after the other, as soon as PeriodEcsValidxSI(n) = '1'):
--     PeriodNorm = PeriodEcsxDI(n) * GainNormxDI(n) + OffsetNormxDI(n)
--     PeriodLin  = sum k=1..ORDER ChebyshevCoeffxDI(n)(k) * Tk(PeriodNorm)
--   DeltaECSX = PeriodLinXP(1) - PeriodLinXN(2) + GammaXxDI
--   DeltaECSZ = PeriodLinZP(3) - PeriodLinZN(4) + GammaZxDI
--   AnglexDO  = atan( (CosThetaZ*DeltaECSX - SinThetaX*DeltaECSZ)
--                    /(SinThetaZ*DeltaECSX + CosThetaX*DeltaECSZ) )
--
-- The arctangent is evaluated as a full-circle atan2(numerator, denominator),
-- i.e. the signs of numerator and denominator select the quadrant and the
-- result covers 0 .. 65535 (65536 = 2*pi, negative angles wrap modulo 2*pi).
--
-- Architecture (everything shared, fully sequential, no clamping/saturation):
--   EcsLinearizer  one instance, used 4 times (1 multiplier, 2*ORDER products)
--   SerialMulAcc   one instance computing both rotated components as exact
--                  two-term dot products (no wide adders)
--   Atan2Cordic    block-normalising 18 iteration CORDIC
-- Total: 2 x 17x17 multipliers (2 RTG4 math blocks) plus ~2.6k LUT4 / ~1.4k FF
-- (generic LUT4 mapping estimate for ORDER = 6).  Every register-to-register
-- path is a few LUT levels or one adder of at most 41 bits.
--
-- Formats: gain Q-4.36, offset Q8.24, coefficients/Gammas share one integer
-- format, the four Cos/Sin inputs share one format (see AngleComputePkg).
--
-- Timing: StartxSI is sampled while idle (a new start while busy is ignored).
-- The period/parameter inputs are used as they are (they must stay stable until
-- ValidxSO).  Latency from the last PeriodEcsValidxSI to ValidxSO (ORDER = 6,
-- simulated): 234..271 clocks, i.e. 2.3..2.7 us at 100 MHz; ~22 clocks more per
-- additional ORDER.  The spread comes from the CORDIC input normalisation.
--
-- There is no reset input: every register has an initial value, which the
-- device applies at power-up.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.EcsTypesPkg.all;
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
    SinThetaZxDI       : in  signed(31 downto 0) -- Sin(ThetaZ) (always between -1 and 1)
  );
end entity AngleCompute;

architecture rtl of AngleCompute is

  constant WPL  : positive := PeriodLinWidth(ORDER);
  constant WDL  : positive := DeltaWidth(ORDER);
  constant WROT : positive := RotWidth(ORDER);

  type state_t is (S_IDLE, S_WAIT_ECS, S_LIN_RUN,
                   S_ROT_N, S_ROT_N_WAIT, S_ROT_D, S_ROT_D_WAIT, S_ATAN_WAIT);
  signal State  : state_t := S_IDLE;
  signal EcsIdx : integer range 1 to 4 := 1;

  -- linearizer, fed with the parameters of ECS EcsIdx
  signal LinStart  : std_logic := '0';
  signal LinValid  : std_logic;
  signal LinPeriod : unsigned(23 downto 0);
  signal LinGain   : signed(31 downto 0);
  signal LinOffset : signed(31 downto 0);
  signal LinCoeff  : ChebyshevCoeff_t(1 to ORDER);
  signal PeriodLin : signed(WPL - 1 downto 0);

  -- DeltaECS in coefficient format with GUARD_BITS extra fractional bits
  signal DeltaX : signed(WDL - 1 downto 0) := (others => '0');
  signal DeltaZ : signed(WDL - 1 downto 0) := (others => '0');

  -- rotation: both rotated components as two-term dot products
  signal RotStart : std_logic := '0';
  signal RotA     : signed(2 * WDL - 1 downto 0);
  signal RotB     : signed(2 * COEFF_W - 1 downto 0) := (others => '0');
  signal RotSub   : std_logic_vector(1 downto 0) := "00";
  signal RotDone  : std_logic;
  signal RotP     : signed(WROT - 1 downto 0);
  signal NumReg   : signed(WROT - 1 downto 0) := (others => '0');

  -- arctangent
  signal AtanStart : std_logic := '0';
  signal AtanValid : std_logic;

begin

  ---------------------------------------------------------------------------
  -- ECS period linearization (one shared instance)
  ---------------------------------------------------------------------------
  LinPeriod <= PeriodEcsxDI(EcsIdx);
  LinGain   <= GainNormxDI(EcsIdx);
  LinOffset <= OffsetNormxDI(EcsIdx);
  LinCoeff  <= ChebyshevCoeffxDI(EcsIdx);

  Lin : entity work.EcsLinearizer
    generic map (ORDER => ORDER)
    port map (
      ClkxCI       => ClkxCI,
      StartxSI     => LinStart,
      ValidxSO     => LinValid,
      PeriodxDI    => LinPeriod,
      GainxDI      => LinGain,
      OffsetxDI    => LinOffset,
      CoeffxDI     => LinCoeff,
      PeriodLinxDO => PeriodLin);

  ---------------------------------------------------------------------------
  -- rotation:  N = CosThetaZ*DeltaX - SinThetaX*DeltaZ
  --            D = SinThetaZ*DeltaX + CosThetaX*DeltaZ
  ---------------------------------------------------------------------------
  RotA <= DeltaZ & DeltaX;

  Rot : entity work.SerialMulAcc
    generic map (WA => WDL, WB => COEFF_W, NT => 2)
    port map (
      ClkxCI   => ClkxCI,
      StartxSI => RotStart,
      AxDI     => RotA,
      BxDI     => RotB,
      SubxSI   => RotSub,
      DonexSO  => RotDone,
      PxDO     => RotP);

  ---------------------------------------------------------------------------
  -- atan2(N, D)
  ---------------------------------------------------------------------------
  Atan : entity work.Atan2Cordic
    generic map (WIN => WROT)
    port map (
      ClkxCI   => ClkxCI,
      StartxSI => AtanStart,
      XxDI     => RotP,      -- denominator D, valid while RotP is stable
      YxDI     => NumReg,    -- numerator N
      ValidxSO => AtanValid,
      AnglexDO => AnglexDO);

  ValidxSO <= AtanValid;

  ---------------------------------------------------------------------------
  -- sequencer
  ---------------------------------------------------------------------------
  process (ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      LinStart  <= '0';
      RotStart  <= '0';
      AtanStart <= '0';

      case State is

        when S_IDLE =>
          if StartxSI = '1' then
            EcsIdx <= 1;
            if PeriodEcsValidxSI(1) = '1' then
              -- period 1 is valid in the very clock of the start: do not miss it
              LinStart <= '1';
              State    <= S_LIN_RUN;
            else
              State <= S_WAIT_ECS;
            end if;
          end if;

        -- wait for the period of ECS EcsIdx, then linearize it
        when S_WAIT_ECS =>
          if PeriodEcsValidxSI(EcsIdx) = '1' then
            LinStart <= '1';
            State    <= S_LIN_RUN;
          end if;

        -- fold the linearized period into DeltaX / DeltaZ
        when S_LIN_RUN =>
          if LinValid = '1' then
            case EcsIdx is
              when 1 =>
                DeltaX <= resize(PeriodLin, WDL) +
                          shift_left(resize(GammaXxDI, WDL), GUARD_BITS);
              when 2 =>
                DeltaX <= DeltaX - resize(PeriodLin, WDL);
              when 3 =>
                DeltaZ <= resize(PeriodLin, WDL) +
                          shift_left(resize(GammaZxDI, WDL), GUARD_BITS);
              when 4 =>
                DeltaZ <= DeltaZ - resize(PeriodLin, WDL);
            end case;
            if EcsIdx = 4 then
              State <= S_ROT_N;
            else
              EcsIdx <= EcsIdx + 1;
              State  <= S_WAIT_ECS;
            end if;
          end if;

        -- N = CosThetaZ*DeltaX - SinThetaX*DeltaZ
        when S_ROT_N =>
          RotB     <= SinThetaXxDI & CosThetaZxDI;
          RotSub   <= "10";
          RotStart <= '1';
          State    <= S_ROT_N_WAIT;

        when S_ROT_N_WAIT =>
          if RotDone = '1' then
            NumReg <= RotP;
            State  <= S_ROT_D;
          end if;

        -- D = SinThetaZ*DeltaX + CosThetaX*DeltaZ
        when S_ROT_D =>
          RotB     <= CosThetaXxDI & SinThetaZxDI;
          RotSub   <= "00";
          RotStart <= '1';
          State    <= S_ROT_D_WAIT;

        when S_ROT_D_WAIT =>
          if RotDone = '1' then
            AtanStart <= '1';
            State     <= S_ATAN_WAIT;
          end if;

        when S_ATAN_WAIT =>
          if AtanValid = '1' then
            State <= S_IDLE;
          end if;

      end case;
    end if;
  end process;

end architecture rtl;

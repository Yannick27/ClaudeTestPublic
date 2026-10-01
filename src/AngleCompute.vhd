-------------------------------------------------------------------------------
-- AngleCompute
--   4 x EcsLinearizer (normalization + 6th-order Chebyshev linearization),
--   then
--     DeltaECSX = LinXP - LinXN + GammaX
--     DeltaECSZ = LinZP - LinZN + GammaZ
--     N = CosThetaZ*DeltaECSX - SinThetaX*DeltaECSZ
--     D = SinThetaZ*DeltaECSX + CosThetaX*DeltaECSZ
--     Angle = atan2(N, D)  (CORDIC, vectoring mode, no divider)
--   AnglexDO is 16-bit unsigned, 65536 = 2*pi (negative angles wrap to +2*pi).
--   See AngleComputePkg for fixed-point formats.
--
--   Handshake : StartxSI is level-sensitive in the idle state.  ValidxSO is
--   cleared when a computation starts and stays high after completion until
--   the next start.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.AngleComputePkg.all;

entity AngleCompute is
  port(
    ClkxCI              : in  std_logic;  -- 100 MHz clock
    -- Internal control
    StartxSI            : in  std_logic;  -- start computation
    ValidxSO            : out std_logic             := '0';  -- angle is valid
    AnglexDO            : out unsigned(15 downto 0) := (others => '0'); -- computed angle
    -- Frequency measurements ----
    PeriodEcsxDI        : in  EcsPeriod_t(1 to 4);  -- Periods of all ECSs (1000000..2000000)
    PeriodEcsValidxSI   : in  std_logic_vector(1 to 4);  -- Valid of the periods
    -- Parameters ------
    GainNormxDI         : in  EcsGain_t(1 to 4);
    OffsetNormxDI       : in  EcsOffset_t(1 to 4);
    ChebyshevCoeffXPxDI : in  ChebyshevCoeff_t;
    ChebyshevCoeffXNxDI : in  ChebyshevCoeff_t;
    ChebyshevCoeffZPxDI : in  ChebyshevCoeff_t;
    ChebyshevCoeffZNxDI : in  ChebyshevCoeff_t;
    GammaXxDI           : in  signed(31 downto 0);
    GammaZxDI           : in  signed(31 downto 0);
    CosThetaXxDI        : in  signed(31 downto 0);
    SinThetaXxDI        : in  signed(31 downto 0);
    CosThetaZxDI        : in  signed(31 downto 0);
    SinThetaZxDI        : in  signed(31 downto 0)
  );
end entity AngleCompute;

architecture rtl of AngleCompute is

  ---------------------------------------------------------------------------
  -- channels
  ---------------------------------------------------------------------------
  type Coeffs_t is array (1 to 4) of ChebyshevCoeff_t;
  type Lin_t    is array (1 to 4) of signed(LIN_W-1 downto 0);

  signal Coeffs   : Coeffs_t;
  signal ChStart  : std_logic := '0';
  signal ChDone   : std_logic_vector(1 to 4);
  signal ChLin    : Lin_t;

  ---------------------------------------------------------------------------
  -- sequencer
  ---------------------------------------------------------------------------
  type State_t is (S_IDLE, S_PULSE, S_WAIT_CH, S_DELTA,
                   S_NRM1, S_NRM1_END,
                   S_MUL_ISSUE, S_MUL_WAIT, S_MUL_ACC,
                   S_NRM2, S_NRM2_END,
                   S_CORDIC_INIT, S_CORDIC_SHIFT, S_CORDIC_STEP,
                   S_FINISH);
  signal State : State_t := S_IDLE;

  -- multiplier
  signal MulStart : std_logic := '0';
  signal MulA     : signed(31 downto 0) := (others => '0');
  signal MulB     : signed(31 downto 0) := (others => '0');
  signal MulDone  : std_logic;
  signal MulP     : signed(63 downto 0);
  signal PReg     : signed(63 downto 0) := (others => '0');

  signal DX, DZ   : signed(LIN_W-1 downto 0) := (others => '0');  -- DeltaECS
  signal DXs, DZs : signed(31 downto 0) := (others => '0');       -- normalized to 32 bits
  signal Nn, Dd   : signed(64 downto 0) := (others => '0');       -- numerator / denominator
  signal J        : integer range 0 to 3 := 0;

  -- CORDIC (26-bit datapath, 22-bit angle with 2**22 = 2*pi)
  constant CW : natural := 26;
  signal Cx, Cy   : signed(CW-1 downto 0) := (others => '0');
  signal Sx, Sy   : signed(CW-1 downto 0) := (others => '0');  -- shifted copies
  signal Cz       : unsigned(ANGLE_W-1 downto 0) := (others => '0');
  signal Iter     : integer range 0 to CORDIC_ITER-1 := 0;

begin

  Coeffs(1) <= ChebyshevCoeffXPxDI;
  Coeffs(2) <= ChebyshevCoeffXNxDI;
  Coeffs(3) <= ChebyshevCoeffZPxDI;
  Coeffs(4) <= ChebyshevCoeffZNxDI;

  gCh : for n in 1 to 4 generate
    uCh : entity work.EcsLinearizer
      port map(
        ClkxCI         => ClkxCI,
        StartxSI       => ChStart,
        PeriodxDI      => PeriodEcsxDI(n),
        PeriodValidxSI => PeriodEcsValidxSI(n),
        GainxDI        => GainNormxDI(n),
        OffsetxDI      => OffsetNormxDI(n),
        CoeffxDI       => Coeffs(n),
        DonexSO        => ChDone(n),
        LinxDO         => ChLin(n));
  end generate;

  uMul : entity work.Mul32
    port map(ClkxCI => ClkxCI, StartxSI => MulStart, AxDI => MulA, BxDI => MulB,
             DonexSO => MulDone, PxDO => MulP);

  process(ClkxCI)
    variable V21 : unsigned(ANGLE_W downto 0);
  begin
    if rising_edge(ClkxCI) then
      MulStart <= '0';
      ChStart  <= '0';

      case State is

        when S_IDLE =>
          if StartxSI = '1' then
            ValidxSO <= '0';
            ChStart  <= '1';
            State    <= S_PULSE;
          end if;

        when S_PULSE =>                     -- channels see ChStart, clear their Done
          State <= S_WAIT_CH;

        when S_WAIT_CH =>
          if ChDone = "1111" then
            State <= S_DELTA;
          end if;

        -----------------------------------------------------------------------
        -- DeltaECS
        -----------------------------------------------------------------------
        when S_DELTA =>
          DX    <= ChLin(1) - ChLin(2) + resize(GammaXxDI, LIN_W);
          DZ    <= ChLin(3) - ChLin(4) + resize(GammaZxDI, LIN_W);
          State <= S_NRM1;

        -----------------------------------------------------------------------
        -- joint normalization of (DX, DZ): shift left until one of them uses
        -- the full 40 bits, then keep the 32 MSBs (the ratio is scale-free).
        -----------------------------------------------------------------------
        when S_NRM1 =>
          if DX = 0 and DZ = 0 then
            Cz    <= (others => '0');           -- angle 0
            State <= S_FINISH;
          elsif DX(LIN_W-1) = DX(LIN_W-2) and DZ(LIN_W-1) = DZ(LIN_W-2) then
            DX <= shift_left(DX, 1);
            DZ <= shift_left(DZ, 1);
          else
            State <= S_NRM1_END;
          end if;

        when S_NRM1_END =>
          DXs   <= DX(LIN_W-1 downto LIN_W-32);
          DZs   <= DZ(LIN_W-1 downto LIN_W-32);
          J     <= 0;
          State <= S_MUL_ISSUE;

        -----------------------------------------------------------------------
        -- N = CosZ*DX - SinX*DZ ; D = SinZ*DX + CosX*DZ
        -----------------------------------------------------------------------
        when S_MUL_ISSUE =>
          case J is
            when 0      => MulA <= CosThetaZxDI; MulB <= DXs;
            when 1      => MulA <= SinThetaXxDI; MulB <= DZs;
            when 2      => MulA <= SinThetaZxDI; MulB <= DXs;
            when others => MulA <= CosThetaXxDI; MulB <= DZs;
          end case;
          MulStart <= '1';
          State    <= S_MUL_WAIT;

        when S_MUL_WAIT =>
          if MulDone = '1' then
            PReg  <= MulP;
            State <= S_MUL_ACC;
          end if;

        when S_MUL_ACC =>
          case J is
            when 0      => Nn <= resize(PReg, 65);
            when 1      => Nn <= Nn - resize(PReg, 65);
            when 2      => Dd <= resize(PReg, 65);
            when others => Dd <= Dd + resize(PReg, 65);
          end case;
          if J = 3 then
            State <= S_NRM2;
          else
            J     <= J + 1;
            State <= S_MUL_ISSUE;
          end if;

        -----------------------------------------------------------------------
        -- normalization of (N, D) before CORDIC
        -----------------------------------------------------------------------
        when S_NRM2 =>
          if Nn = 0 and Dd = 0 then
            Cz    <= (others => '0');           -- angle 0
            State <= S_FINISH;
          elsif Nn(64) = Nn(63) and Dd(64) = Dd(63) then
            Nn <= shift_left(Nn, 1);
            Dd <= shift_left(Dd, 1);
          else
            State <= S_NRM2_END;
          end if;

        when S_NRM2_END =>
          State <= S_CORDIC_INIT;

        -----------------------------------------------------------------------
        -- CORDIC vectoring: x = D, y = N ; pre-rotate by pi when x < 0
        -----------------------------------------------------------------------
        when S_CORDIC_INIT =>
          Iter <= 0;
          if Dd(64) = '1' then
            Cx <= -resize(Dd(64 downto 41), CW);
            Cy <= -resize(Nn(64 downto 41), CW);
            Cz <= to_unsigned(2**(ANGLE_W-1), ANGLE_W);   -- pi
          else
            Cx <= resize(Dd(64 downto 41), CW);
            Cy <= resize(Nn(64 downto 41), CW);
            Cz <= (others => '0');
          end if;
          State <= S_CORDIC_SHIFT;

        when S_CORDIC_SHIFT =>
          Sx    <= shift_right(Cx, Iter);
          Sy    <= shift_right(Cy, Iter);
          State <= S_CORDIC_STEP;

        when S_CORDIC_STEP =>
          if Cy(CW-1) = '0' then
            Cx <= Cx + Sy;
            Cy <= Cy - Sx;
            Cz <= Cz + ATAN_TABLE(Iter);
          else
            Cx <= Cx - Sy;
            Cy <= Cy + Sx;
            Cz <= Cz - ATAN_TABLE(Iter);
          end if;
          if Iter = CORDIC_ITER-1 then
            State <= S_FINISH;
          else
            Iter  <= Iter + 1;
            State <= S_CORDIC_SHIFT;
          end if;

        -----------------------------------------------------------------------
        -- round 22-bit angle to 16 bits (wraps modulo 2*pi)
        -----------------------------------------------------------------------
        when S_FINISH =>
          V21      := ('0' & Cz) + to_unsigned(2**(ANGLE_W-16-1), ANGLE_W+1);
          AnglexDO <= V21(ANGLE_W-1 downto ANGLE_W-16);
          ValidxSO <= '1';
          State    <= S_IDLE;

      end case;
    end if;
  end process;

end architecture rtl;

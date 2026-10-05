-------------------------------------------------------------------------------
-- AngleCompute : angle computation from 4 ECS periods (Microchip RTG4, 100 MHz)
--
-- Sequence
--   1. wait StartxSI = 1
--   2. for ECS 1..4 (XP, XN, ZP, ZN) : wait PeriodEcsValidxSI(n) = 1, sample the
--      period, normalize it and linearize it with a Chebyshev polynomial
--      (entity ChebyLinearizer, shared by the 4 channels)
--   3. DeltaECSX = PeriodLinXP - PeriodLinXN + GammaX
--      DeltaECSZ = PeriodLinZP - PeriodLinZN + GammaZ
--   4. Num = CosThetaZ*DeltaECSX - SinThetaX*DeltaECSZ
--      Den = SinThetaZ*DeltaECSX + CosThetaX*DeltaECSZ
--      AnglexDO = arctan(Num / Den)   (CORDIC, entity CordicAtan2)
--   5. ValidxSO = 1 for one clock cycle
--
-- Number formats / assumptions
--   * Angle      : turns, 2*pi <=> 65536 (AnglexDO is unsigned(15 downto 0)).
--   * The arctan is computed as atan2(Num, Den), i.e. over the full circle
--     [0, 2*pi) (the quadrant is given by the signs of Num and Den). Set
--     FULL_CIRCLE_C to false to get the plain arctan(Num/Den) in (-pi/2, pi/2)
--     (negative values wrap modulo 2*pi).
--   * Only the ratio Num/Den matters, so the scale of CosTheta*/SinTheta* is
--     free (Q1.31, Q2.30, ... all give the same angle), they only have to use
--     the same scale.
--   * The linearized values and the Gamma offsets use the same Q.FRAC format
--     (FRAC does not change the arithmetic, the ratio is scale independent).
--   * All parameter inputs (Gain, Offset, Chebyshev coefficients, Gamma, trig.
--     values) and the period inputs must be stable while the computation is
--     running (from StartxSI until ValidxSO). The periods are sampled on the
--     clock cycle where PeriodEcsValidxSI(n) = 1. The channels are processed
--     one after the other, so PeriodEcsValidxSI(n) is used as a level ("period
--     n is available") and must stay at '1' until channel n has been reached;
--     a 1-clock pulse that occurs while another channel is processed is missed.
--   * StartxSI is level sensitive in the idle state: if it is kept at '1', a
--     new computation starts as soon as the previous one is done.
--
-- Resources : 8 RTG4 math blocks (2 x MulPipe), no division, no RAM.
-- Latency after the last period : at most ~350 clock cycles (3.5 us).
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.AngleCompute_pkg.all;

entity AngleCompute is
  generic(ORDER : positive := 6;        -- order of Chevychev polynomial
          FRAC  : natural  := 24        -- fractional bits of the coefficients ChebyshevCoeff* and GammaXxDI and GammaXxDI
         );
  port(
    ClkxCI             : in  std_logic; -- 100 MHz clock
    -- Internal control
    StartxSI           : in  std_logic;  -- start computation
    ValidxSO           : out std_logic             := '0'; -- angle is valid
    AnglexDO           : out unsigned(15 downto 0) := (others => '0'); -- computed angle
    -- Frequency measurements ----
    PeriodEcsxDI       : in  EcsPeriod_t(1 to 4); -- Periods of all ECSs (always between 1000000 and 2000000)
    PeriodEcsValidxSI  : in  std_logic_vector(1 to 4); -- Valid signal for the periods of all ECS
    -- Parameters ------
    GainNormxDI        : in  EcsGain_t(1 to 4); -- gain to normalize periods between -1 and 1 --format Q-4.36
    OffsetNormxDI      : in  EcsOffset_t(1 to 4); -- offset to normalize periods between -1 and 1 --format Q8.24
    ChebyshevCoeffXPxDI : in  ChebyshevCoeff_t; -- Chebyshev coefficients for linearization of PeriodXP --format Q.FRAC
    ChebyshevCoeffXNxDI : in  ChebyshevCoeff_t; -- Chebyshev coefficients for linearization of PeriodXN --format Q.FRAC
    ChebyshevCoeffZPxDI : in  ChebyshevCoeff_t; -- Chebyshev coefficients for linearization of PeriodZP --format Q.FRAC
    ChebyshevCoeffZNxDI : in  ChebyshevCoeff_t; -- Chebyshev coefficients for linearization of PeriodZN --format Q.FRAC
    GammaXxDI          : in  signed(31 downto 0); -- Chebyshev offset on X axis --format Q.FRAC
    GammaZxDI          : in  signed(31 downto 0); -- Chebyshev offset on Z axis --format Q.FRAC
    CosThetaXxDI       : in  signed(31 downto 0); -- Cos(ThetaX) (always between -1 and 1)
    SinThetaXxDI       : in  signed(31 downto 0); -- Sin(ThetaX) (always between -1 and 1)
    CosThetaZxDI       : in  signed(31 downto 0); -- Cos(ThetaZ) (always between -1 and 1)
    SinThetaZxDI       : in  signed(31 downto 0) -- Sin(ThetaZ) (always between -1 and 1)
  );
end entity AngleCompute;

architecture rtl of AngleCompute is

  constant GUARD_C       : natural  := 4;    -- extra fractional bits of the linearized values
  constant LIN_W_C       : positive := LinWidth(ORDER, GUARD_C);
  constant DELTA_W_C     : positive := LIN_W_C + 2;   -- (XP - XN + Gamma) never overflows
  constant DATA_W_C      : positive := 32;   -- operand width of the multiplier and the CORDIC
  constant PROD_W_C      : positive := 65;   -- Num/Den : sum of two 64-bit products
  constant FULL_CIRCLE_C : boolean  := true; -- atan2 (true) or arctan (false)

  type State_t is (S_IDLE, S_WAIT_PERIOD, S_CHEB_START, S_CHEB_RUN,
                   S_DELTA1, S_DELTA2,
                   S_NORM1_START, S_NORM1_WAIT,
                   S_MUL_ISSUE, S_MUL_WAIT,
                   S_NORM2_START, S_NORM2_WAIT,
                   S_CORDIC_START, S_CORDIC_WAIT);
  signal StatexDP : State_t := S_IDLE;

  signal ChanxDP   : integer range 1 to 4 := 1;
  signal PeriodxDP : unsigned(23 downto 0) := (others => '0');

  -- Chebyshev linearizer
  signal ChebStartxS : std_logic;
  signal ChebValidxS : std_logic;
  signal ChebLinxD   : signed(LIN_W_C - 1 downto 0);
  signal ChebGainxD  : signed(31 downto 0);
  signal ChebOffsxD  : signed(31 downto 0);
  signal ChebCoeffxD : ChebyshevCoeff_t;

  signal LinXPxDP : signed(LIN_W_C - 1 downto 0) := (others => '0');
  signal LinXNxDP : signed(LIN_W_C - 1 downto 0) := (others => '0');
  signal LinZPxDP : signed(LIN_W_C - 1 downto 0) := (others => '0');
  signal LinZNxDP : signed(LIN_W_C - 1 downto 0) := (others => '0');

  signal DXxDP : signed(DELTA_W_C - 1 downto 0) := (others => '0');
  signal DZxDP : signed(DELTA_W_C - 1 downto 0) := (others => '0');

  -- normalization of DeltaECSX/Z to 32 bits
  signal Norm1StartxS : std_logic;
  signal Norm1DonexS  : std_logic;
  signal DXnxD        : signed(DATA_W_C - 1 downto 0);
  signal DZnxD        : signed(DATA_W_C - 1 downto 0);

  -- final multiplications
  signal MulValidInxS  : std_logic;
  signal MulAxD        : signed(31 downto 0);
  signal MulBxD        : signed(31 downto 0);
  signal MulValidOutxS : std_logic;
  signal MulProdxD     : signed(63 downto 0);
  signal MiDP          : integer range 0 to 3 := 0;   -- issue counter
  signal RiDP          : integer range 0 to 3 := 0;   -- result counter
  signal P1xDP         : signed(63 downto 0) := (others => '0');
  signal P3xDP         : signed(63 downto 0) := (others => '0');
  signal NumxDP        : signed(PROD_W_C - 1 downto 0) := (others => '0');
  signal DenxDP        : signed(PROD_W_C - 1 downto 0) := (others => '0');

  -- normalization of Num/Den to 32 bits
  signal Norm2StartxS : std_logic;
  signal Norm2DonexS  : std_logic;
  signal NumnxD       : signed(DATA_W_C - 1 downto 0);
  signal DennxD       : signed(DATA_W_C - 1 downto 0);

  -- CORDIC
  signal CordicStartxS : std_logic;
  signal CordicValidxS : std_logic;
  signal CordicAnglexD : unsigned(15 downto 0);

begin

  assert ORDER = CHEBY_ORDER_C
    report "AngleCompute: generic ORDER must be equal to CHEBY_ORDER_C of AngleCompute_pkg"
    severity failure;
  assert FRAC <= 31
    report "AngleCompute: FRAC must be <= 31" severity failure;

  ---------------------------------------------------------------------------
  -- channel selection for the (shared) linearizer
  ---------------------------------------------------------------------------
  ChebGainxD  <= GainNormxDI(ChanxDP);
  ChebOffsxD  <= OffsetNormxDI(ChanxDP);
  ChebCoeffxD <= ChebyshevCoeffXPxDI when ChanxDP = 1 else
                 ChebyshevCoeffXNxDI when ChanxDP = 2 else
                 ChebyshevCoeffZPxDI when ChanxDP = 3 else
                 ChebyshevCoeffZNxDI;

  ChebStartxS   <= '1' when StatexDP = S_CHEB_START  else '0';
  Norm1StartxS  <= '1' when StatexDP = S_NORM1_START else '0';
  Norm2StartxS  <= '1' when StatexDP = S_NORM2_START else '0';
  CordicStartxS <= '1' when StatexDP = S_CORDIC_START else '0';

  ---------------------------------------------------------------------------
  -- period normalization + Chebyshev linearization
  ---------------------------------------------------------------------------
  LinInst : entity work.ChebyLinearizer
    generic map(ORDER => ORDER, GUARD => GUARD_C)
    port map(
      ClkxCI    => ClkxCI,
      StartxSI  => ChebStartxS,
      ValidxSO  => ChebValidxS,
      PeriodxDI => PeriodxDP,
      GainxDI   => ChebGainxD,
      OffsetxDI => ChebOffsxD,
      CoeffxDI  => ChebCoeffxD,
      LinxDO    => ChebLinxD);

  ---------------------------------------------------------------------------
  -- block normalization of DeltaECSX/Z (value width -> 32 bits)
  ---------------------------------------------------------------------------
  Norm1Inst : entity work.BlockNormalizer
    generic map(IN_W => DELTA_W_C, OUT_W => DATA_W_C)
    port map(
      ClkxCI   => ClkxCI,
      StartxSI => Norm1StartxS,
      AxDI     => DXxDP,
      BxDI     => DZxDP,
      DonexSO  => Norm1DonexS,
      AxDO     => DXnxD,
      BxDO     => DZnxD);

  ---------------------------------------------------------------------------
  -- rotation matrix multiplications
  ---------------------------------------------------------------------------
  MulInst : entity work.MulPipe
    port map(
      ClkxCI   => ClkxCI,
      ValidxSI => MulValidInxS,
      AxDI     => MulAxD,
      BxDI     => MulBxD,
      ValidxSO => MulValidOutxS,
      ProdxDO  => MulProdxD);

  MulValidInxS <= '1' when StatexDP = S_MUL_ISSUE else '0';

  process(MiDP, CosThetaXxDI, SinThetaXxDI, CosThetaZxDI, SinThetaZxDI, DXnxD, DZnxD)
  begin
    case MiDP is
      when 0 =>      MulAxD <= CosThetaZxDI; MulBxD <= DXnxD;   -- CosZ*DX
      when 1 =>      MulAxD <= SinThetaXxDI; MulBxD <= DZnxD;   -- SinX*DZ
      when 2 =>      MulAxD <= SinThetaZxDI; MulBxD <= DXnxD;   -- SinZ*DX
      when others => MulAxD <= CosThetaXxDI; MulBxD <= DZnxD;   -- CosX*DZ
    end case;
  end process;

  ---------------------------------------------------------------------------
  -- block normalization of Num/Den (65 bits -> 32 bits)
  ---------------------------------------------------------------------------
  Norm2Inst : entity work.BlockNormalizer
    generic map(IN_W => PROD_W_C, OUT_W => DATA_W_C)
    port map(
      ClkxCI   => ClkxCI,
      StartxSI => Norm2StartxS,
      AxDI     => NumxDP,
      BxDI     => DenxDP,
      DonexSO  => Norm2DonexS,
      AxDO     => NumnxD,
      BxDO     => DennxD);

  ---------------------------------------------------------------------------
  -- arctan(Num/Den) = atan2(Y => Num, X => Den)
  ---------------------------------------------------------------------------
  CordicInst : entity work.CordicAtan2
    generic map(DATA_W => DATA_W_C, NIT => 18, OUT_W => 16, GUARD => 6,
                FULL_CIRCLE => FULL_CIRCLE_C)
    port map(
      ClkxCI   => ClkxCI,
      ValidxSI => CordicStartxS,
      XxDI     => DennxD,
      YxDI     => NumnxD,
      ValidxSO => CordicValidxS,
      AnglexDO => CordicAnglexD);

  ---------------------------------------------------------------------------
  -- sequencer
  ---------------------------------------------------------------------------
  process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      ValidxSO <= '0';

      case StatexDP is

        when S_IDLE =>
          if StartxSI = '1' then
            ChanxDP  <= 1;
            StatexDP <= S_WAIT_PERIOD;
          end if;

        when S_WAIT_PERIOD =>
          if PeriodEcsValidxSI(ChanxDP) = '1' then
            PeriodxDP <= PeriodEcsxDI(ChanxDP);
            StatexDP  <= S_CHEB_START;
          end if;

        when S_CHEB_START =>
          StatexDP <= S_CHEB_RUN;

        when S_CHEB_RUN =>
          if ChebValidxS = '1' then
            case ChanxDP is
              when 1      => LinXPxDP <= ChebLinxD;
              when 2      => LinXNxDP <= ChebLinxD;
              when 3      => LinZPxDP <= ChebLinxD;
              when others => LinZNxDP <= ChebLinxD;
            end case;
            if ChanxDP = 4 then
              StatexDP <= S_DELTA1;
            else
              ChanxDP  <= ChanxDP + 1;
              StatexDP <= S_WAIT_PERIOD;
            end if;
          end if;

        when S_DELTA1 =>
          DXxDP    <= resize(LinXPxDP, DELTA_W_C) - resize(LinXNxDP, DELTA_W_C);
          DZxDP    <= resize(LinZPxDP, DELTA_W_C) - resize(LinZNxDP, DELTA_W_C);
          StatexDP <= S_DELTA2;

        when S_DELTA2 =>
          -- Gamma (Q.FRAC) -> Q.(FRAC+GUARD)
          DXxDP    <= DXxDP + shift_left(resize(GammaXxDI, DELTA_W_C), GUARD_C);
          DZxDP    <= DZxDP + shift_left(resize(GammaZxDI, DELTA_W_C), GUARD_C);
          StatexDP <= S_NORM1_START;

        when S_NORM1_START =>
          StatexDP <= S_NORM1_WAIT;

        when S_NORM1_WAIT =>
          if Norm1DonexS = '1' then
            MiDP     <= 0;
            RiDP     <= 0;
            StatexDP <= S_MUL_ISSUE;
          end if;

        when S_MUL_ISSUE =>
          if MiDP = 3 then
            MiDP     <= 0;
            StatexDP <= S_MUL_WAIT;
          else
            MiDP <= MiDP + 1;
          end if;

        when S_MUL_WAIT =>
          -- products arrive in issue order, one per clock cycle
          if MulValidOutxS = '1' then
            case RiDP is
              when 0 => P1xDP  <= MulProdxD;
              when 1 => NumxDP <= resize(P1xDP, PROD_W_C) - resize(MulProdxD, PROD_W_C);
              when 2 => P3xDP  <= MulProdxD;
              when others =>
                DenxDP   <= resize(P3xDP, PROD_W_C) + resize(MulProdxD, PROD_W_C);
                StatexDP <= S_NORM2_START;
            end case;
            if RiDP = 3 then
              RiDP <= 0;
            else
              RiDP <= RiDP + 1;
            end if;
          end if;

        when S_NORM2_START =>
          StatexDP <= S_NORM2_WAIT;

        when S_NORM2_WAIT =>
          if Norm2DonexS = '1' then
            StatexDP <= S_CORDIC_START;
          end if;

        when S_CORDIC_START =>
          StatexDP <= S_CORDIC_WAIT;

        when S_CORDIC_WAIT =>
          if CordicValidxS = '1' then
            AnglexDO <= CordicAnglexD;
            ValidxSO <= '1';
            StatexDP <= S_IDLE;
          end if;

      end case;
    end if;
  end process;

end architecture rtl;

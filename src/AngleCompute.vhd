-------------------------------------------------------------------------------
-- AngleCompute
--
--   Computes the angle from the periods of four eddy-current sensors (ECS):
--
--     for n = 1..4 :  x_n   = Period_n * Gain_n + Offset_n          (|x_n| <= 1)
--                     Lin_n = sum_k=1..ORDER  Tk(x_n) * Coeff_n(k)    (Chebyshev)
--     DeltaECSX = Lin_1 - Lin_2 + GammaX            (XP - XN)
--     DeltaECSZ = Lin_3 - Lin_4 + GammaZ            (ZP - ZN)
--     Angle     = atan( (CosThetaZ*DeltaECSX - SinThetaX*DeltaECSZ)
--                     / (SinThetaZ*DeltaECSX + CosThetaX*DeltaECSZ) )
--
--   Sequence (everything runs on ClkxCI, no reset: all registers have initial values)
--     idle -> StartxSI = '1' -> wait PeriodEcsValidxSI(1) -> linearise ECS 1
--          -> wait PeriodEcsValidxSI(2) -> linearise ECS 2 -> ... ECS 4
--          -> rotate -> atan2 -> ValidxSO = '1' for one clock, AnglexDO valid.
--   One EcsLinearizer is reused for the four ECS.  The inputs are used directly (they are
--   stable during the computation), nothing is latched.
--
--   Angle : atan2 on the full circle, 65536 = 2*pi, rounded to nearest, wrapped into
--           [0, 65535] (the arctan of the formula above is evaluated with the signs of
--           numerator and denominator, i.e. over the full circle).  0 if the vector is 0.
--
--   Fixed point : see EcsLinearizer (Chebyshev, 33 fractional bits, exact accumulation of
--           Tk*ck in LinWidth(ORDER) bits), PairNormalizer (block scaling of the vectors
--           so the result does not depend on the scale of the coefficients and of the
--           sin/cos values: only their common scale matters) and CordicAtan2.
--           Coefficients, Gamma and sin/cos are used as plain integers: the position of
--           their binary point is irrelevant provided it is common to the four
--           coefficient sets and Gamma, and to the four sin/cos values.
--           No clamping or saturation anywhere: all word widths are sized for the worst
--           case, and modular arithmetic is only used where the true result is known to
--           fit (normalisation P*G+O, Chebyshev recurrence).
--
--   Latency after the last period valid: about 17*ORDER + 430 clocks (530 clocks = 5.3 us at
--   100 MHz for ORDER = 6): linearisation of ECS 4 (17*ORDER), two sums, scaling of
--   (DeltaECSX, DeltaECSZ) (up to 72), rotation (about 50) and atan2 (up to 400).  The periods
--   arrive about 1e6 clocks apart, so this is irrelevant for the application; the time is
--   spent on keeping every register-to-register path short (see the design units below).
--
--   Design units (compile in this order)
--     AngleComputePkg  port types, fixed-point constants, LinWidth()
--     MulSigned35      35x35 -> 70 bit multiplier, four 18x18 products = 4 RTG4 math blocks
--     WideAddSub       72-bit add/subtract in two clocks (two 36-bit carry chains)
--     PairNormalizer   common left shift of two words (block floating point)
--     CordicAtan2      full-circle atan2, CORDIC vectoring mode, 16-bit result
--     EcsLinearizer    P*G+O and the Chebyshev polynomial of one ECS
--     AngleCompute     this entity: sequencing, DeltaECS, rotation, atan2
--
--   Resources: 2 x MulSigned35 (8 math blocks), about 3000 registers.  No adder is wider than
--   38 bits and every multiplier is 18x18 per clock; all sub-blocks are registered at their
--   outputs.  Timing closure on RTG4 has to be confirmed with Libero (not available here).
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.AngleComputePkg.all;

entity AngleCompute is
  generic(ORDER : positive := 6        -- order of Chebyshev polynomial
         );
  port(
    ClkxCI             : in  std_logic; -- 100 MHz clock
    -- Internal control
    StartxSI           : in  std_logic;  -- start computation
    ValidxSO           : out std_logic             := '0'; -- angle is valid
    AnglexDO           : out unsigned(15 downto 0) := (others => '0'); -- computed angle on full circle ( 65536 = 2*pi)
    -- Frequency measurements ----
    PeriodEcsxDI       : in  EcsPeriod_t(1 to 4); -- Periods of all ECSs (always between 1000000 and 2000000)
    PeriodEcsValidxSI  : in  std_logic_vector(1 to 4); -- Valid signal for the periods of all ECS
    -- Parameters ------
    GainNormxDI        : in  EcsParam_t(1 to 4); -- gain to normalize periods between -1 and 1 --format Q-4.36
    OffsetNormxDI      : in  EcsParam_t(1 to 4); -- offset to normalize periods between -1 and 1 --format Q8.24
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

  constant LIN_W : positive := LinWidth(ORDER);
  constant DXW   : positive := LIN_W + 2;        -- DeltaECSX / DeltaECSZ: 2 more bits than one Lin
  constant NZ_W  : positive := T_W;              -- (DeltaECSX, DeltaECSZ) after scaling
  constant NUM_W : positive := NZ_W + 32 + 1;    -- numerator / denominator (35x32 products, 1 add)

  -- Every wide register is loaded in exactly one state (or one state and one registered
  -- flag): the results of the sub-blocks stay valid until their next start, so a "wait"
  -- state is followed by a "capture" state and clock enables stay one LUT deep.
  type State_t is (S_IDLE, S_WAIT_PERIOD, S_LIN, S_PREP, S_COMBINE, S_STORE, S_NORM,
                   S_ROT_ISSUE, S_ROT_WAIT, S_ROT_CAP, S_ROT_ADD, S_ROT_STORE, S_ATAN);
  signal State : State_t := S_IDLE;

  signal Ch : natural range 1 to 4 := 1;   -- ECS being processed (1 = XP, 2 = XN, 3 = ZP, 4 = ZN)
  signal J  : natural range 0 to 3 := 0;   -- rotation product being computed

  -- the ECS selected by Ch, seen by the (single) linearizer
  signal LinPeriod : unsigned(23 downto 0);
  signal LinGain   : signed(31 downto 0);
  signal LinOffset : signed(31 downto 0);
  signal LinCoeff  : ChebyshevCoeff_t(1 to ORDER);
  signal LinStart  : std_logic := '0';
  signal LinDone   : std_logic;
  signal LinResult : signed(LIN_W-1 downto 0);

  -- DeltaECSX / DeltaECSZ with 33 fractional bits
  signal DX : signed(DXW-1 downto 0) := (others => '0');
  signal DZ : signed(DXW-1 downto 0) := (others => '0');

  -- common scaling of (DX, DZ)
  signal NormStart : std_logic := '0';
  signal NormDone  : std_logic;
  signal DXn       : signed(NZ_W-1 downto 0);
  signal DZn       : signed(NZ_W-1 downto 0);

  -- rotation: 4 products, 2 sums
  signal MulStart : std_logic := '0';
  signal MulA     : signed(34 downto 0) := (others => '0');
  signal MulB     : signed(34 downto 0) := (others => '0');
  signal MulDone  : std_logic;
  signal MulP     : signed(69 downto 0);

  signal AddStart : std_logic := '0';
  signal AddSub   : std_logic := '0';
  signal AddA     : signed(DXW-1 downto 0) := (others => '0');
  signal AddB     : signed(DXW-1 downto 0) := (others => '0');
  signal AddDone  : std_logic;
  signal AddS     : signed(DXW-1 downto 0);

  signal PA  : signed(DXW-1 downto 0)  := (others => '0');   -- first product of a sum
  signal Num : signed(NUM_W-1 downto 0) := (others => '0');  -- CosThetaZ*DX - SinThetaX*DZ
  signal Den : signed(NUM_W-1 downto 0) := (others => '0');  -- SinThetaZ*DX + CosThetaX*DZ

  signal AtanStart : std_logic := '0';
  signal AtanDone  : std_logic;
  signal AtanAngle : unsigned(15 downto 0);

begin

  ---------------------------------------------------------------------------
  -- one linearizer, shared by the four ECS
  ---------------------------------------------------------------------------
  LinPeriod <= PeriodEcsxDI(Ch);
  LinGain   <= GainNormxDI(Ch);
  LinOffset <= OffsetNormxDI(Ch);
  LinCoeff  <= ChebyshevCoeffxDI(Ch);

  uLin : entity work.EcsLinearizer
    generic map(ORDER => ORDER)
    port map(
      ClkxCI    => ClkxCI,
      StartxSI  => LinStart,
      PeriodxDI => LinPeriod,
      GainxDI   => LinGain,
      OffsetxDI => LinOffset,
      CoeffxDI  => LinCoeff,
      DonexSO   => LinDone,
      LinxDO    => LinResult);

  uNorm : entity work.PairNormalizer
    generic map(WIN => DXW, WOUT => NZ_W)
    port map(
      ClkxCI   => ClkxCI,
      StartxSI => NormStart,
      XxDI     => DX,
      YxDI     => DZ,
      DonexSO  => NormDone,
      ZeroxSO  => open,
      XxDO     => DXn,
      YxDO     => DZn);

  uMul : entity work.MulSigned35
    port map(
      ClkxCI   => ClkxCI,
      StartxSI => MulStart,
      AxDI     => MulA,
      BxDI     => MulB,
      DonexSO  => MulDone,
      PxDO     => MulP);

  -- adds LinResult to DX / DZ, then forms numerator and denominator
  uAdd : entity work.WideAddSub
    generic map(W => DXW)
    port map(
      ClkxCI   => ClkxCI,
      StartxSI => AddStart,
      SubxSI   => AddSub,
      AxDI     => AddA,
      BxDI     => AddB,
      DonexSO  => AddDone,
      SxDO     => AddS);

  uAtan : entity work.CordicAtan2
    generic map(WIN => NUM_W)
    port map(
      ClkxCI   => ClkxCI,
      StartxSI => AtanStart,
      XxDI     => Den,
      YxDI     => Num,
      DonexSO  => AtanDone,
      AnglexDO => AtanAngle);

  ---------------------------------------------------------------------------
  -- sequencer
  ---------------------------------------------------------------------------
  process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      ValidxSO  <= '0';
      LinStart  <= '0';
      NormStart <= '0';
      MulStart  <= '0';
      AddStart  <= '0';
      AtanStart <= '0';

      case State is

        -- wait until StartxSI = 1.  DeltaECSX / DeltaECSZ start from the Gamma offsets
        -- (loaded on every idle clock: they are not used while idle).
        when S_IDLE =>
          DX <= shift_left(resize(GammaXxDI, DXW), T_FRAC);
          DZ <= shift_left(resize(GammaZxDI, DXW), T_FRAC);
          Ch <= 1;
          if StartxSI = '1' then
            if PeriodEcsValidxSI(1) = '1' then
              LinStart <= '1';
              State    <= S_LIN;
            else
              State <= S_WAIT_PERIOD;
            end if;
          end if;

        -- wait for PeriodEcsxDI(Ch) valid, then normalise + linearise that period
        when S_WAIT_PERIOD =>
          if PeriodEcsValidxSI(Ch) = '1' then
            LinStart <= '1';
            State    <= S_LIN;
          end if;

        when S_LIN =>
          if LinDone = '1' then
            State <= S_PREP;
          end if;

        -- DeltaECSX = Gamma + Lin1 - Lin2 ; DeltaECSZ = Gamma + Lin3 - Lin4
        when S_PREP =>
          if Ch <= 2 then
            AddA <= DX;
          else
            AddA <= DZ;
          end if;
          AddB <= resize(LinResult, DXW);
          if Ch = 2 or Ch = 4 then
            AddSub <= '1';
          else
            AddSub <= '0';
          end if;
          AddStart <= '1';
          State    <= S_COMBINE;

        when S_COMBINE =>
          if AddDone = '1' then
            State <= S_STORE;
          end if;

        when S_STORE =>
          if Ch <= 2 then
            DX <= AddS;
          else
            DZ <= AddS;
          end if;
          if Ch = 4 then
            NormStart <= '1';
            State     <= S_NORM;
          else
            Ch    <= Ch + 1;
            State <= S_WAIT_PERIOD;
          end if;

        -- scale (DX, DZ) to 35 bits (common shift)
        when S_NORM =>
          if NormDone = '1' then
            J     <= 0;
            State <= S_ROT_ISSUE;
          end if;

        -- J = 0: CosThetaZ*DX   J = 1: SinThetaX*DZ   (Num = P0 - P1)
        -- J = 2: SinThetaZ*DX   J = 3: CosThetaX*DZ   (Den = P2 + P3)
        when S_ROT_ISSUE =>
          case J is
            when 0 =>
              MulA <= resize(CosThetaZxDI, 35);
              MulB <= DXn;
            when 1 =>
              MulA <= resize(SinThetaXxDI, 35);
              MulB <= DZn;
            when 2 =>
              MulA <= resize(SinThetaZxDI, 35);
              MulB <= DXn;
            when 3 =>
              MulA <= resize(CosThetaXxDI, 35);
              MulB <= DZn;
          end case;
          MulStart <= '1';
          State    <= S_ROT_WAIT;

        when S_ROT_WAIT =>
          if MulDone = '1' then
            State <= S_ROT_CAP;
          end if;

        when S_ROT_CAP =>
          if J = 0 or J = 2 then
            PA    <= resize(MulP, DXW);
            J     <= J + 1;
            State <= S_ROT_ISSUE;
          else
            AddA <= PA;
            AddB <= resize(MulP, DXW);
            if J = 1 then
              AddSub <= '1';
            else
              AddSub <= '0';
            end if;
            AddStart <= '1';
            State    <= S_ROT_ADD;
          end if;

        when S_ROT_ADD =>
          if AddDone = '1' then
            State <= S_ROT_STORE;
          end if;

        when S_ROT_STORE =>
          if J = 1 then
            Num   <= AddS(NUM_W-1 downto 0);
            J     <= 2;
            State <= S_ROT_ISSUE;
          else
            Den       <= AddS(NUM_W-1 downto 0);
            AtanStart <= '1';
            State     <= S_ATAN;
          end if;

        -- angle = atan2(Num, Den); ValidxSO for one clock
        when S_ATAN =>
          if AtanDone = '1' then
            AnglexDO <= AtanAngle;
            ValidxSO <= '1';
            State    <= S_IDLE;
          end if;

      end case;
    end if;
  end process;

end architecture rtl;

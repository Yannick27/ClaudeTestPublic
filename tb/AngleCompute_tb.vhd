-------------------------------------------------------------------------------
-- AngleCompute_tb : self-checking VUnit testbench for AngleCompute
--
-- The DUT result is compared with a floating-point reference model computed
-- from the exact (quantized) values applied to the DUT ports :
--   PeriodNorm = PeriodEcs * GainNorm + OffsetNorm
--   PeriodLin  = sum(k = 1..ORDER) Tk(PeriodNorm) * ChebyshevCoeff(k)
--   DeltaECSX  = PeriodLinXP - PeriodLinXN + GammaX
--   DeltaECSZ  = PeriodLinZP - PeriodLinZN + GammaZ
--   Angle      = atan2(CosThetaZ*DeltaECSX - SinThetaX*DeltaECSZ,
--                      SinThetaZ*DeltaECSX + CosThetaX*DeltaECSZ)
--   AnglexDO   = Angle * 65536 / (2*pi), wrapped to [0, 65536)
--
-- Fixed-point formats :
--   GainNormxDI   : Q-4.36 (value = integer * 2**-36)
--   OffsetNormxDI : Q8.24  (value = integer * 2**-24)
--   ChebyshevCoeffxDI, GammaXxDI, GammaZxDI : COEFF_FRAC fractional bits
--   Cos/SinTheta  : TRIG_FRAC fractional bits
-- Only the ratio between the Chebyshev coefficient and Gamma scaling matters
-- for the angle (a common scale factor cancels in atan2), and the same holds
-- for the Cos/Sin scaling.
--
-- Protocol checks :
--   * ValidxSO is 0 when StartxSI is pulsed and stays 0 until all
--     PeriodEcsValidxSI bits are set
--   * ValidxSO rises within MAX_LATENCY clock cycles after PeriodEcsValidxSI(4)
--   * ValidxSO and AnglexDO are held stable until AckxSI is pulsed
--   * ValidxSO goes back to 0 within ACK_LATENCY cycles after AckxSI
--   * PeriodEcsxDI(k) carries random garbage until PeriodEcsValidxSI(k) = 1,
--     so a DUT reading a period before its valid is detected
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

library osvvm;
use osvvm.RandomPkg.all;

use work.AngleComputePkg.all;

entity AngleCompute_tb is
  generic(
    runner_cfg    : string;
    SEED          : natural  := 1;       -- OSVVM random seed
    N_RANDOM      : positive := 200;     -- number of computations in the random tests
    ANGLE_TOL_LSB : natural  := 2;       -- accepted angle error (1 LSB = 2*pi/65536)
    MAX_LATENCY   : positive := 10_000;  -- max cycles from PeriodEcsValidxSI(4) to ValidxSO
    ACK_LATENCY   : positive := 1;       -- max cycles from AckxSI sampled to ValidxSO = 0
    COEFF_FRAC    : natural  := 30;      -- fractional bits of Chebyshev coefficients and Gamma
    TRIG_FRAC     : natural  := 30       -- fractional bits of Cos/Sin(Theta)
  );
end entity AngleCompute_tb;

architecture tb of AngleCompute_tb is

  constant CLK_PERIOD  : time    := 10 ns;  -- 100 MHz
  constant GAIN_FRAC   : natural := 36;     -- Q-4.36
  constant OFFSET_FRAC : natural := 24;     -- Q8.24

  -----------------------------------------------------------------------------
  -- One full set of DUT inputs
  -----------------------------------------------------------------------------
  type TestVector_t is record
    Period   : EcsPeriod_t(1 to 4);
    Gain     : EcsParam_t(1 to 4);
    Offset   : EcsParam_t(1 to 4);
    Coeff    : ChebyshevCoeffArray_t;
    GammaX   : signed(31 downto 0);
    GammaZ   : signed(31 downto 0);
    CosX     : signed(31 downto 0);
    SinX     : signed(31 downto 0);
    CosZ     : signed(31 downto 0);
    SinZ     : signed(31 downto 0);
  end record;

  type RealArray4_t is array (1 to 4) of real;

  -----------------------------------------------------------------------------
  -- Fixed-point helpers
  -----------------------------------------------------------------------------
  function to_real(x : signed; frac : natural) return real is
  begin
    return real(to_integer(x)) * 2.0 ** (-real(frac));
  end function;

  function to_fixed(x : real; frac : natural) return signed is
  begin
    return to_signed(integer(round(x * 2.0 ** real(frac))), 32);
  end function;

  -----------------------------------------------------------------------------
  -- Reference model
  -----------------------------------------------------------------------------
  function norm_period(tv : TestVector_t; ch : positive) return real is
  begin
    return real(to_integer(tv.Period(ch))) * to_real(tv.Gain(ch), GAIN_FRAC)
           + to_real(tv.Offset(ch), OFFSET_FRAC);
  end function;

  function lin_period(tv : TestVector_t; ch : positive) return real is
    variable X              : real := norm_period(tv, ch);
    variable Tkm1, Tk, Tkp1 : real;
    variable Acc            : real := 0.0;
  begin
    Tkm1 := 1.0;                        -- T0
    Tk   := X;                          -- T1
    for k in 1 to ORDER loop
      Acc  := Acc + Tk * to_real(tv.Coeff(ch)(k), COEFF_FRAC);
      Tkp1 := 2.0 * X * Tk - Tkm1;
      Tkm1 := Tk;
      Tk   := Tkp1;
    end loop;
    return Acc;
  end function;

  -- Numerator and denominator of the arctangent
  procedure atan_args(tv : TestVector_t; Num, Den : out real) is
    variable DeltaX, DeltaZ : real;
  begin
    DeltaX := lin_period(tv, 1) - lin_period(tv, 2) + to_real(tv.GammaX, COEFF_FRAC);
    DeltaZ := lin_period(tv, 3) - lin_period(tv, 4) + to_real(tv.GammaZ, COEFF_FRAC);
    Num    := to_real(tv.CosZ, TRIG_FRAC) * DeltaX - to_real(tv.SinX, TRIG_FRAC) * DeltaZ;
    Den    := to_real(tv.SinZ, TRIG_FRAC) * DeltaX + to_real(tv.CosX, TRIG_FRAC) * DeltaZ;
  end procedure;

  -- Expected angle in LSB (65536 = 2*pi), as a real in [0, 65536)
  function expected_angle(tv : TestVector_t) return real is
    variable Num, Den, A : real;
  begin
    atan_args(tv, Num, Den);
    A := arctan(Num, Den) * 65536.0 / MATH_2_PI;
    if A < 0.0 then
      A := A + 65536.0;
    end if;
    return A;
  end function;

  -- Signed circular distance in LSB between the DUT angle and the expected one
  function angle_error(dut : unsigned(15 downto 0); expected : real) return real is
    variable E : real := real(to_integer(dut)) - expected;
  begin
    if E >= 32768.0 then
      E := E - 65536.0;
    elsif E < -32768.0 then
      E := E + 65536.0;
    end if;
    return E;
  end function;

  -----------------------------------------------------------------------------
  -- DUT signals
  -----------------------------------------------------------------------------
  signal ClkxC             : std_logic := '0';
  signal StartxS           : std_logic := '0';
  signal AnglexD           : unsigned(15 downto 0);
  signal ValidxS           : std_logic;
  signal AckxS             : std_logic := '0';
  signal PeriodEcsxD       : EcsPeriod_t(1 to 4)      := (others => (others => '0'));
  signal PeriodEcsValidxS  : std_logic_vector(1 to 4) := (others => '0');
  signal GainNormxD        : EcsParam_t(1 to 4)       := (others => (others => '0'));
  signal OffsetNormxD      : EcsParam_t(1 to 4)       := (others => (others => '0'));
  signal ChebyshevCoeffxD  : ChebyshevCoeffArray_t    := (others => (others => (others => '0')));
  signal GammaXxD          : signed(31 downto 0)      := (others => '0');
  signal GammaZxD          : signed(31 downto 0)      := (others => '0');
  signal CosThetaXxD       : signed(31 downto 0)      := (others => '0');
  signal SinThetaXxD       : signed(31 downto 0)      := (others => '0');
  signal CosThetaZxD       : signed(31 downto 0)      := (others => '0');
  signal SinThetaZxD       : signed(31 downto 0)      := (others => '0');

begin

  ClkxC <= not ClkxC after CLK_PERIOD / 2;

  dut : entity work.AngleCompute
    port map(
      ClkxCI            => ClkxC,
      StartxSI          => StartxS,
      AnglexDO          => AnglexD,
      ValidxSO          => ValidxS,
      AckxSI            => AckxS,
      PeriodEcsxDI      => PeriodEcsxD,
      PeriodEcsValidxSI => PeriodEcsValidxS,
      GainNormxDI       => GainNormxD,
      OffsetNormxDI     => OffsetNormxD,
      ChebyshevCoeffxDI => ChebyshevCoeffxD,
      GammaXxDI         => GammaXxD,
      GammaZxDI         => GammaZxD,
      CosThetaXxDI      => CosThetaXxD,
      SinThetaXxDI      => SinThetaXxD,
      CosThetaZxDI      => CosThetaZxD,
      SinThetaZxDI      => SinThetaZxD
    );

  test_runner_watchdog(runner, 100 ms);

  main : process
    variable Rnd      : RandomPType;
    variable MaxErr   : real := 0.0;
    variable TvNumber : natural := 0;

    ---------------------------------------------------------------------------
    -- Random generation of realistic parameters
    ---------------------------------------------------------------------------
    -- Random ECS channel : period range [Pmin, Pmax] inside [1e6, 2e6]
    -- normalized to [-1, 1] (Pmin -> 1, Pmax -> -1), random period inside it.
    procedure random_channel(tv : inout TestVector_t; ch : positive) is
      variable Span, Pmin, P : real;
    begin
      Span := Rnd.Uniform(200_000.0, 1_000_000.0);
      Pmin := Rnd.Uniform(1_000_000.0, 2_000_000.0 - Span);
      P    := Rnd.Uniform(Pmin + 0.001 * Span, Pmin + 0.999 * Span);
      tv.Gain(ch)   := to_fixed(-2.0 / Span, GAIN_FRAC);
      tv.Offset(ch) := to_fixed(1.0 + 2.0 * Pmin / Span, OFFSET_FRAC);
      tv.Period(ch) := to_unsigned(integer(round(P)), 24);
    end procedure;

    -- Random Chebyshev coefficients : dominant linear term + small corrections
    procedure random_coeffs(tv : inout TestVector_t; ch : positive) is
    begin
      tv.Coeff(ch)(1) := to_fixed(Rnd.Uniform(0.6, 1.0), COEFF_FRAC);
      for k in 2 to ORDER loop
        tv.Coeff(ch)(k) := to_fixed(Rnd.Uniform(-0.15, 0.15) / real(k - 1), COEFF_FRAC);
      end loop;
    end procedure;

    impure function random_vector return TestVector_t is
      variable tv       : TestVector_t;
      variable ThX, ThZ : real;
      variable Num, Den : real;
    begin
      loop
        for ch in 1 to 4 loop
          random_channel(tv, ch);
          random_coeffs(tv, ch);
        end loop;
        tv.GammaX := to_fixed(Rnd.Uniform(-0.25, 0.25), COEFF_FRAC);
        tv.GammaZ := to_fixed(Rnd.Uniform(-0.25, 0.25), COEFF_FRAC);
        ThX       := Rnd.Uniform(0.0, MATH_2_PI);
        ThZ       := Rnd.Uniform(0.0, MATH_2_PI);
        tv.CosX   := to_fixed(cos(ThX), TRIG_FRAC);
        tv.SinX   := to_fixed(sin(ThX), TRIG_FRAC);
        tv.CosZ   := to_fixed(cos(ThZ), TRIG_FRAC);
        tv.SinZ   := to_fixed(sin(ThZ), TRIG_FRAC);
        -- skip ill-conditioned vectors where the angle is not meaningful
        atan_args(tv, Num, Den);
        exit when sqrt(Num ** 2 + Den ** 2) > 0.05;
      end loop;
      return tv;
    end function;

    -- Vector producing the angle Phi : no Chebyshev contribution, the angle
    -- only comes from Gamma, ThetaX = ThetaZ = 0  =>  Angle = atan2(GammaX, GammaZ)
    impure function circle_vector(Phi : real) return TestVector_t is
      variable tv : TestVector_t := random_vector;
    begin
      tv.Coeff  := (others => (others => (others => '0')));
      tv.GammaX := to_fixed(0.5 * sin(Phi), COEFF_FRAC);
      tv.GammaZ := to_fixed(0.5 * cos(Phi), COEFF_FRAC);
      tv.CosX   := to_fixed(1.0, TRIG_FRAC);
      tv.SinX   := to_fixed(0.0, TRIG_FRAC);
      tv.CosZ   := to_fixed(1.0, TRIG_FRAC);
      tv.SinZ   := to_fixed(0.0, TRIG_FRAC);
      return tv;
    end function;

    ---------------------------------------------------------------------------
    -- Bus functional procedures
    ---------------------------------------------------------------------------
    procedure wait_cycles(n : natural) is
    begin
      for i in 1 to n loop
        wait until rising_edge(ClkxC);
      end loop;
    end procedure;

    -- Wait n cycles while checking that the DUT does not report a result
    procedure wait_no_valid(n : natural; msg : string) is
    begin
      for i in 1 to n loop
        wait until rising_edge(ClkxC);
        check_equal(ValidxS, '0', "ValidxSO asserted " & msg);
      end loop;
    end procedure;

    -- Run one computation : apply the parameters, pulse StartxSI, deliver
    -- the periods one after the other with random gaps in [GapMin, GapMax],
    -- wait for ValidxSO, check the angle, acknowledge after a random delay
    -- in [0, AckDelayMax] cycles.
    procedure run_computation(tv          : TestVector_t;
                              GapMin      : natural := 0;
                              GapMax      : natural := 50;
                              AckDelayMax : natural := 10) is
      variable Expected : real;
      variable Err      : real;
      variable Latency  : natural;
      variable Angle    : unsigned(15 downto 0);
    begin
      TvNumber := TvNumber + 1;
      Expected := expected_angle(tv);

      -- parameters, stable during the whole computation
      wait until rising_edge(ClkxC);
      check_equal(ValidxS, '0', "ValidxSO must be 0 before StartxSI");
      GainNormxD       <= tv.Gain;
      OffsetNormxD     <= tv.Offset;
      ChebyshevCoeffxD <= tv.Coeff;
      GammaXxD         <= tv.GammaX;
      GammaZxD         <= tv.GammaZ;
      CosThetaXxD      <= tv.CosX;
      SinThetaXxD      <= tv.SinX;
      CosThetaZxD      <= tv.CosZ;
      SinThetaZxD      <= tv.SinZ;
      PeriodEcsValidxS <= (others => '0');
      for ch in 1 to 4 loop
        -- garbage until the period is valid
        PeriodEcsxD(ch) <= to_unsigned(Rnd.RandInt(0, 2**24 - 1), 24);
      end loop;
      wait_no_valid(Rnd.RandInt(1, 5), "before StartxSI");

      -- start pulse
      StartxS <= '1';
      wait until rising_edge(ClkxC);
      StartxS <= '0';

      -- sequential delivery of the periods
      for ch in 1 to 4 loop
        wait_no_valid(Rnd.RandInt(GapMin, GapMax),
                      "before PeriodEcsValidxSI(" & to_string(ch) & ") = 1");
        PeriodEcsxD(ch)      <= tv.Period(ch);
        PeriodEcsValidxS(ch) <= '1';
      end loop;

      -- wait for the result
      Latency := 0;
      loop
        wait until rising_edge(ClkxC);
        exit when ValidxS = '1';
        Latency := Latency + 1;
        if Latency > MAX_LATENCY then
          check_failed("ValidxSO not asserted within " & to_string(MAX_LATENCY) &
                       " cycles after PeriodEcsValidxSI(4) (vector " & to_string(TvNumber) & ")");
          return;
        end if;
      end loop;

      -- check the angle
      Angle := AnglexD;
      Err   := angle_error(Angle, Expected);
      if abs(Err) > MaxErr then
        MaxErr := abs(Err);
      end if;
      check(abs(Err) <= real(ANGLE_TOL_LSB),
            "Vector " & to_string(TvNumber) & " : AnglexDO = " & to_string(to_integer(Angle)) &
            ", expected " & to_string(Expected, "%.3f") & " (error " & to_string(Err, "%.3f") & " LSB)");
      debug("Vector " & to_string(TvNumber) & " : latency " & to_string(Latency) &
            " cycles, AnglexDO = " & to_string(to_integer(Angle)) &
            ", expected " & to_string(Expected, "%.3f"));

      -- ValidxSO and AnglexDO held until acknowledge
      for i in 1 to Rnd.RandInt(0, AckDelayMax) loop
        wait until rising_edge(ClkxC);
        check_equal(ValidxS, '1', "ValidxSO must be held until AckxSI (vector " & to_string(TvNumber) & ")");
        check_equal(AnglexD, Angle, "AnglexDO must be stable while ValidxSO = 1 (vector " & to_string(TvNumber) & ")");
      end loop;

      -- acknowledge pulse
      AckxS <= '1';
      wait until rising_edge(ClkxC);
      AckxS <= '0';
      for i in 1 to ACK_LATENCY loop
        wait until rising_edge(ClkxC);
        exit when ValidxS = '0';
      end loop;
      check_equal(ValidxS, '0', "ValidxSO must go back to 0 after AckxSI (vector " & to_string(TvNumber) & ")");
    end procedure;

    variable tv       : TestVector_t;
    variable Num, Den : real;

  begin
    test_runner_setup(runner, runner_cfg);
    Rnd.InitSeed(SEED);

    while test_suite loop
      MaxErr   := 0.0;
      TvNumber := 0;
      wait_cycles(10);

      if run("test_nominal") then
        -- centered periods, pure linear Chebyshev, no rotation
        for ch in 1 to 4 loop
          tv.Gain(ch)   := to_fixed(-2.0 / 1.0e6, GAIN_FRAC);  -- [1e6, 2e6] -> [1, -1]
          tv.Offset(ch) := to_fixed(3.0, OFFSET_FRAC);
          tv.Coeff(ch)  := (1 => to_fixed(1.0, COEFF_FRAC), others => (others => '0'));
        end loop;
        tv.Period := (to_unsigned(1_300_000, 24), to_unsigned(1_700_000, 24),   -- X : 0.4 - (-0.4)
                      to_unsigned(1_600_000, 24), to_unsigned(1_300_000, 24));  -- Z : -0.2 - 0.4
        tv.GammaX := to_fixed(0.0, COEFF_FRAC);
        tv.GammaZ := to_fixed(0.0, COEFF_FRAC);
        tv.CosX   := to_fixed(1.0, TRIG_FRAC);
        tv.SinX   := to_fixed(0.0, TRIG_FRAC);
        tv.CosZ   := to_fixed(1.0, TRIG_FRAC);
        tv.SinZ   := to_fixed(0.0, TRIG_FRAC);
        -- atan2(0.8, -0.6) = 2.2143 rad -> 23095.96 LSB
        check(abs(expected_angle(tv) - 23095.96) < 0.5, "Reference model self-check");
        run_computation(tv);

      elsif run("test_full_circle") then
        -- 64 angles around the circle, including the axes (quadrant boundaries)
        for i in 0 to 63 loop
          run_computation(circle_vector(real(i) * MATH_2_PI / 64.0));
        end loop;
        -- angles close to the +/- pi wrap-around
        run_computation(circle_vector(MATH_PI - 1.0e-4));
        run_computation(circle_vector(MATH_PI + 1.0e-4));
        run_computation(circle_vector(-1.0e-4));

      elsif run("test_random") then
        for i in 1 to N_RANDOM loop
          run_computation(random_vector);
        end loop;

      elsif run("test_back_to_back_periods") then
        -- all periods valid on consecutive cycles, immediate acknowledge
        for i in 1 to N_RANDOM / 4 loop
          run_computation(random_vector, GapMin => 0, GapMax => 0, AckDelayMax => 0);
        end loop;

      elsif run("test_slow_periods") then
        -- long gaps between the periods
        for i in 1 to 10 loop
          run_computation(random_vector, GapMin => 200, GapMax => 2000);
        end loop;

      elsif run("test_late_acknowledge") then
        -- ValidxSO and AnglexDO must be held for a long time until AckxSI
        for i in 1 to 10 loop
          run_computation(random_vector, AckDelayMax => 1000);
        end loop;

      elsif run("test_extreme_periods") then
        -- periods at the edges of the [1e6, 2e6] range, normalized to ~ +/-1
        -- (1 count margin to absorb the gain quantization)
        for i in 1 to N_RANDOM / 4 loop
          loop
            tv := random_vector;
            for ch in 1 to 4 loop
              tv.Gain(ch)   := to_fixed(-2.0 / 1.0e6, GAIN_FRAC);
              tv.Offset(ch) := to_fixed(3.0, OFFSET_FRAC);
              if Rnd.RandInt(0, 1) = 0 then
                tv.Period(ch) := to_unsigned(1_000_001, 24);
              else
                tv.Period(ch) := to_unsigned(1_999_999, 24);
              end if;
            end loop;
            atan_args(tv, Num, Den);
            exit when sqrt(Num ** 2 + Den ** 2) > 0.05;
          end loop;
          run_computation(tv);
        end loop;
      end if;

      info("Maximum angle error : " & to_string(MaxErr, "%.3f") & " LSB over " &
           to_string(TvNumber) & " computation(s)");
    end loop;

    test_runner_cleanup(runner);
  end process main;

end architecture tb;

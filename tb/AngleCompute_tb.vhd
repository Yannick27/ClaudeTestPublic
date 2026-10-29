-------------------------------------------------------------------------------
-- AngleCompute_tb : VUnit testbench for AngleCompute
--
-- A real-number reference model computes, from the exact fixed-point values
-- applied to the DUT :
--   PeriodNorm(i) = PeriodEcs(i) * GainNorm(i) + OffsetNorm(i)
--   PeriodLin(i)  = sum(k = 1..ORDER) Tk(PeriodNorm(i)) * ChebyshevCoeff(i)(k)
--   DeltaECSX     = PeriodLin(1) - PeriodLin(2) + GammaX
--   DeltaECSZ     = PeriodLin(3) - PeriodLin(4) + GammaZ
--   Num           = CosThetaZ * DeltaECSX - SinThetaX * DeltaECSZ
--   Den           = SinThetaZ * DeltaECSX + CosThetaX * DeltaECSZ
--   Angle         = atan2(Num, Den) on the full circle (65536 = 2*pi)
-- and AnglexDO is compared to it with a tolerance of ANGLE_TOL_LSB (modulo
-- 65536).
--
-- Fixed-point formats of the DUT inputs :
--   GainNorm             : Q-4.36 (value = integer * 2**-36)
--   OffsetNorm           : Q8.24
--   Chebyshev coeffs     : Q8.24
--   GammaX / GammaZ      : Q8.24
--   Cos / Sin ThetaX / Z : Q2.30
--
-- Checked protocol (monitor process, active in every test) :
--   * ValidxSO never has a metavalue
--   * ValidxSO only rises after a StartxSI pulse and PeriodEcsValidxSI(4)
--   * ValidxSO stays at 1 and AnglexDO stays stable until AckxSI = 1
--   * ValidxSO is back to 0 the clock cycle following AckxSI = 1
-- Checked by the stimulus process for each computation :
--   * angle accuracy
--   * latency between rising edge of StartxSI and rising edge of ValidxSO
--     strictly below 1 ms
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
    runner_cfg      : string;
    SEED            : natural  := 12345;  -- random seed
    NUM_RANDOM      : positive := 100;    -- number of computations in random_computations
    SWEEP_POINTS    : positive := 64;     -- number of angles in full_circle_sweep
    ANGLE_TOL_LSB   : natural  := 2;      -- allowed angle error (1 LSB = 2*pi / 65536)
    MAX_VALID_DELAY : positive := 20000   -- max clock cycles between two PeriodEcsValidxSI rising edges
  );
end entity AngleCompute_tb;

architecture tb of AngleCompute_tb is

  constant CLK_PERIOD  : time := 10 ns;  -- 100 MHz
  constant MAX_LATENCY : time := 1 ms;

  constant GAIN_FRAC   : natural := 36;  -- Q-4.36
  constant OFFSET_FRAC : natural := 24;  -- Q8.24
  constant COEFF_FRAC  : natural := 24;  -- Q8.24
  constant GAMMA_FRAC  : natural := 24;  -- Q8.24
  constant TRIG_FRAC   : natural := 30;  -- Q2.30

  constant PERIOD_MIN  : natural := 1000000;
  constant PERIOD_MAX  : natural := 2000000;
  constant ANGLE_RANGE : real    := 65536.0;  -- 2*pi
  constant MIN_MAG     : real    := 0.1;      -- min |(Num, Den)| of random vectors (atan2 well defined)

  -----------------------------------------------------------------------------
  -- test vector and reference model
  -----------------------------------------------------------------------------
  type Vector_t is record
    Period : EcsPeriod_t(1 to 4);
    Gain   : EcsParam_t(1 to 4);
    Offset : EcsParam_t(1 to 4);
    Coeff  : ChebyshevCoeffArray_t;
    GammaX : signed(31 downto 0);
    GammaZ : signed(31 downto 0);
    CosX   : signed(31 downto 0);
    SinX   : signed(31 downto 0);
    CosZ   : signed(31 downto 0);
    SinZ   : signed(31 downto 0);
  end record;

  type Model_t is record
    Norm   : real_vector(1 to 4);
    Lin    : real_vector(1 to 4);
    DeltaX : real;
    DeltaZ : real;
    Num    : real;
    Den    : real;
    Mag    : real;
    Angle  : real;                      -- expected angle in LSB, [0, 65536)
  end record;

  function to_real(s : signed; frac : natural) return real is
  begin
    return real(to_integer(s)) / 2.0**frac;
  end function;

  function to_fixed(x : real; frac : natural) return signed is
    variable r : real := round(x * 2.0**frac);
  begin
    if r > 2147483647.0 then
      r := 2147483647.0;
    elsif r < -2147483647.0 then
      r := -2147483647.0;
    end if;
    return to_signed(integer(r), 32);
  end function;

  function norm_period(v : Vector_t; i : positive) return real is
  begin
    return real(to_integer(v.Period(i))) * to_real(v.Gain(i), GAIN_FRAC)
           + to_real(v.Offset(i), OFFSET_FRAC);
  end function;

  -- sum(k = 1..ORDER) Tk(x) * c(k), T0 = 1, T1 = x, Tk+2 = 2x*Tk+1 - Tk
  function chebyshev(x : real; c : ChebyshevCoeff_t) return real is
    variable Tkm1 : real := 1.0;
    variable Tk   : real := x;
    variable Tkp1 : real;
    variable Acc  : real := x * to_real(c(c'low), COEFF_FRAC);
  begin
    for k in 2 to ORDER loop
      Tkp1 := 2.0 * x * Tk - Tkm1;
      Acc  := Acc + Tkp1 * to_real(c(c'low + k - 1), COEFF_FRAC);
      Tkm1 := Tk;
      Tk   := Tkp1;
    end loop;
    return Acc;
  end function;

  function model(v : Vector_t) return Model_t is
    variable m : Model_t;
  begin
    for i in 1 to 4 loop
      m.Norm(i) := norm_period(v, i);
      m.Lin(i)  := chebyshev(m.Norm(i), v.Coeff(i));
    end loop;
    m.DeltaX := m.Lin(1) - m.Lin(2) + to_real(v.GammaX, GAMMA_FRAC);
    m.DeltaZ := m.Lin(3) - m.Lin(4) + to_real(v.GammaZ, GAMMA_FRAC);
    m.Num    := to_real(v.CosZ, TRIG_FRAC) * m.DeltaX - to_real(v.SinX, TRIG_FRAC) * m.DeltaZ;
    m.Den    := to_real(v.SinZ, TRIG_FRAC) * m.DeltaX + to_real(v.CosX, TRIG_FRAC) * m.DeltaZ;
    m.Mag    := sqrt(m.Num**2 + m.Den**2);
    if m.Mag = 0.0 then
      m.Angle := 0.0;
    else
      m.Angle := arctan(m.Num, m.Den) / MATH_2_PI * ANGLE_RANGE;
      if m.Angle < 0.0 then
        m.Angle := m.Angle + ANGLE_RANGE;
      end if;
    end if;
    return m;
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

  test_runner_watchdog(runner, 1 sec);

  -----------------------------------------------------------------------------
  -- stimulus and result checking
  -----------------------------------------------------------------------------
  main : process
    variable Rnd        : RandomPType;
    variable v          : Vector_t;
    variable NumChecked : natural := 0;
    variable MaxLatency : time    := 0 ns;
    variable MaxError   : real    := 0.0;

    procedure clk_wait(n : natural) is
    begin
      for i in 1 to n loop
        wait until rising_edge(ClkxC);
      end loop;
    end procedure;

    -- Gain / offset mapping [Tmin, Tmax] onto [1, -1], and a period located
    -- at Pos (0.0 = Tmin, 1.0 = Tmax) in that range
    procedure set_channel(variable v : inout Vector_t; i : positive; Tmin, Tmax, Pos : real) is
      variable Gain : real := -2.0 / (Tmax - Tmin);
      variable P    : integer;
      variable Norm : real;
    begin
      v.Gain(i)   := to_fixed(Gain, GAIN_FRAC);
      v.Offset(i) := to_fixed(1.0 - Gain * Tmin, OFFSET_FRAC);
      P           := integer(round(Tmin + Pos * (Tmax - Tmin)));
      -- keep the quantized normalized period in [-1, 1]
      loop
        v.Period(i) := to_unsigned(P, 24);
        Norm        := norm_period(v, i);
        exit when abs(Norm) <= 1.0;
        if Norm > 1.0 then
          P := P + 1;
        else
          P := P - 1;
        end if;
      end loop;
      assert P >= PERIOD_MIN and P <= PERIOD_MAX
        report "testbench error: period out of range" severity failure;
    end procedure;

    procedure set_rotation(variable v : inout Vector_t; ThetaX, ThetaZ : real) is
    begin
      v.CosX := to_fixed(cos(ThetaX), TRIG_FRAC);
      v.SinX := to_fixed(sin(ThetaX), TRIG_FRAC);
      v.CosZ := to_fixed(cos(ThetaZ), TRIG_FRAC);
      v.SinZ := to_fixed(sin(ThetaZ), TRIG_FRAC);
    end procedure;

    -- random parameters and periods, with a well defined angle
    procedure gen_random(variable v : inout Vector_t) is
      variable Sign : real;
    begin
      loop
        for i in 1 to 4 loop
          set_channel(v, i,
                      Rnd.RandReal(1.0e6, 1.2e6), Rnd.RandReal(1.8e6, 2.0e6),
                      Rnd.RandReal(0.0, 1.0));
          Sign := 1.0;
          if Rnd.RandInt(0, 1) = 1 then
            Sign := -1.0;
          end if;
          v.Coeff(i)(1) := to_fixed(Sign * Rnd.RandReal(0.5, 1.5), COEFF_FRAC);
          for k in 2 to ORDER loop
            v.Coeff(i)(k) := to_fixed(Rnd.RandReal(-0.2, 0.2) / real(k), COEFF_FRAC);
          end loop;
        end loop;
        v.GammaX := to_fixed(Rnd.RandReal(-0.5, 0.5), GAMMA_FRAC);
        v.GammaZ := to_fixed(Rnd.RandReal(-0.5, 0.5), GAMMA_FRAC);
        set_rotation(v, Rnd.RandReal(-MATH_PI / 6.0, MATH_PI / 6.0),
                     Rnd.RandReal(-MATH_PI / 6.0, MATH_PI / 6.0));
        exit when model(v).Mag >= MIN_MAG;
      end loop;
    end procedure;

    -- adjust GammaX / GammaZ so that atan2(Num, Den) = Angle (rad), |(Num, Den)| = Mag
    procedure set_target_angle(variable v : inout Vector_t; Angle, Mag : real) is
      variable m      : Model_t := model(v);
      variable cX, sX : real;
      variable cZ, sZ : real;
      variable Det    : real;
      variable N, D   : real;
    begin
      cX  := to_real(v.CosX, TRIG_FRAC);
      sX  := to_real(v.SinX, TRIG_FRAC);
      cZ  := to_real(v.CosZ, TRIG_FRAC);
      sZ  := to_real(v.SinZ, TRIG_FRAC);
      Det := cZ * cX + sX * sZ;
      N   := Mag * sin(Angle);
      D   := Mag * cos(Angle);
      v.GammaX := to_fixed((cX * N + sX * D) / Det - (m.Lin(1) - m.Lin(2)), GAMMA_FRAC);
      v.GammaZ := to_fixed((cZ * D - sZ * N) / Det - (m.Lin(3) - m.Lin(4)), GAMMA_FRAC);
    end procedure;

    -- one full computation : start, sequential period valids, wait result,
    -- check angle and latency, acknowledge
    procedure run_computation(v                            : Vector_t;
                              ValidDelayMin, ValidDelayMax : positive;
                              AckDelayMin, AckDelayMax     : natural) is
      constant m          : Model_t := model(v);
      variable StartTime  : time;
      variable LastValid  : time;
      variable Latency    : time;
      variable Diff       : real;
    begin
      -- parameters, stable for the whole computation
      wait until rising_edge(ClkxC);
      GainNormxD       <= v.Gain;
      OffsetNormxD     <= v.Offset;
      ChebyshevCoeffxD <= v.Coeff;
      GammaXxD         <= v.GammaX;
      GammaZxD         <= v.GammaZ;
      CosThetaXxD      <= v.CosX;
      SinThetaXxD      <= v.SinX;
      CosThetaZxD      <= v.CosZ;
      SinThetaZxD      <= v.SinZ;
      PeriodEcsValidxS <= (others => '0');
      -- periods are meaningless until their valid is set
      for i in 1 to 4 loop
        PeriodEcsxD(i) <= to_unsigned(Rnd.RandInt(PERIOD_MIN, PERIOD_MAX), 24);
      end loop;

      -- start pulse
      wait until rising_edge(ClkxC);
      StartxS   <= '1';
      StartTime := now;
      wait until rising_edge(ClkxC);
      StartxS   <= '0';

      -- periods valid, sequentially in order 1 to 4
      for i in 1 to 4 loop
        if i = 1 then
          clk_wait(Rnd.RandInt(ValidDelayMin, ValidDelayMax) - 1);
        else
          clk_wait(Rnd.RandInt(ValidDelayMin, ValidDelayMax));
        end if;
        PeriodEcsxD(i)      <= v.Period(i);
        PeriodEcsValidxS(i) <= '1';
      end loop;
      LastValid := now;

      -- result
      if ValidxS /= '1' then
        wait until ValidxS = '1' for StartTime + 2 * MAX_LATENCY - now;
      end if;
      Latency := now - StartTime;
      check_equal(ValidxS, '1', "ValidxSO not set within " & to_string(2 * MAX_LATENCY, 1 ns) & " after StartxSI");
      check(Latency < MAX_LATENCY,
            "Latency StartxSI -> ValidxSO = " & to_string(Latency, 1 ns) & ", must be < " & to_string(MAX_LATENCY, 1 ns));
      if Latency > MaxLatency then
        MaxLatency := Latency;
      end if;

      Diff := real(to_integer(AnglexD)) - m.Angle;
      Diff := Diff - ANGLE_RANGE * round(Diff / ANGLE_RANGE);  -- modulo 2*pi
      if abs(Diff) > MaxError then
        MaxError := abs(Diff);
      end if;
      check(abs(Diff) <= real(ANGLE_TOL_LSB),
            "AnglexDO = " & integer'image(to_integer(AnglexD)) &
            ", expected " & real'image(m.Angle) &
            " (error " & real'image(Diff) & " LSB, DeltaECSX = " & real'image(m.DeltaX) &
            ", DeltaECSZ = " & real'image(m.DeltaZ) & ")");
      debug("Angle " & integer'image(to_integer(AnglexD)) & " expected " & real'image(m.Angle) &
            ", latency from start " & to_string(Latency, 1 ns) &
            ", from last period valid " & to_string(now - LastValid, 1 ns));
      NumChecked := NumChecked + 1;

      -- acknowledge
      clk_wait(Rnd.RandInt(AckDelayMin, AckDelayMax));
      AckxS <= '1';
      wait until rising_edge(ClkxC);
      AckxS <= '0';
      wait until falling_edge(ClkxC);
      check_equal(ValidxS, '0', "ValidxSO must be 0 the clock cycle after AckxSI");
      clk_wait(Rnd.RandInt(0, 10));
    end procedure;

    variable Pos : real;

  begin
    test_runner_setup(runner, runner_cfg);
    Rnd.InitSeed(SEED);
    info("Seed = " & integer'image(SEED));

    while test_suite loop

      if run("valid_stays_low_without_start") then
        -- monitor checks that ValidxSO does not rise without StartxSI
        clk_wait(10000);
        check_equal(ValidxS, '0', "ValidxSO must stay 0 without StartxSI");

      elsif run("single_computation") then
        gen_random(v);
        run_computation(v, 10, 100, 2, 2);

      elsif run("full_circle_sweep") then
        -- angles k * 2*pi / SWEEP_POINTS, including 0, pi/2, pi and 3*pi/2
        for k in 0 to SWEEP_POINTS - 1 loop
          gen_random(v);
          set_target_angle(v, MATH_2_PI * real(k) / real(SWEEP_POINTS), Rnd.RandReal(0.2, 2.0));
          run_computation(v, 1, 200, 0, 5);
        end loop;

      elsif run("quadrant_boundaries") then
        -- just around 0, pi/2, pi, 3*pi/2 (wrap-around of the 16-bit angle at 0)
        for q in 0 to 3 loop
          for d in -3 to 3 loop
            gen_random(v);
            set_target_angle(v, MATH_PI_OVER_2 * real(q) + real(d) * MATH_2_PI / ANGLE_RANGE,
                             Rnd.RandReal(0.2, 2.0));
            run_computation(v, 1, 50, 0, 2);
          end loop;
        end loop;

      elsif run("random_computations") then
        for n in 1 to NUM_RANDOM loop
          gen_random(v);
          run_computation(v, 1, 2000, 0, 20);
        end loop;

      elsif run("extreme_periods") then
        -- full 1000000..2000000 period range, periods at both ends of the
        -- range (normalized period -> +1 / -1) and in the middle
        for n in 0 to 3**4 - 1 loop
          gen_random(v);
          for i in 1 to 4 loop
            Pos := 0.5 * real((n / 3**(i - 1)) mod 3);  -- 0.0, 0.5 or 1.0
            set_channel(v, i, real(PERIOD_MIN), real(PERIOD_MAX), Pos);
          end loop;
          set_target_angle(v, Rnd.RandReal(0.0, MATH_2_PI), Rnd.RandReal(0.2, 2.0));
          run_computation(v, 1, 20, 0, 2);
        end loop;

      elsif run("min_period_valid_delay") then
        -- PeriodEcsValidxSI rising on consecutive clock cycles, immediate ack
        for n in 1 to 20 loop
          gen_random(v);
          run_computation(v, 1, 1, 0, 0);
        end loop;

      elsif run("max_period_valid_delay") then
        -- last PeriodEcsValidxSI 4 * MAX_VALID_DELAY cycles after StartxSI
        for n in 1 to 3 loop
          gen_random(v);
          run_computation(v, MAX_VALID_DELAY, MAX_VALID_DELAY, 0, 1);
        end loop;

      elsif run("long_ack_delay") then
        -- ValidxSO and AnglexDO held during a long time before ack
        for n in 1 to 5 loop
          gen_random(v);
          run_computation(v, 1, 100, 1000, 5000);
        end loop;

      end if;

      info(integer'image(NumChecked) & " angles checked, max error " & real'image(MaxError) &
           " LSB, max latency " & to_string(MaxLatency, 1 ns));
    end loop;

    test_runner_cleanup(runner);
  end process;

  -----------------------------------------------------------------------------
  -- handshake monitor
  -----------------------------------------------------------------------------
  monitor : process
    variable ValidPrev       : std_logic := '0';
    variable AckPrev         : std_logic := '0';
    variable AnglePrev       : unsigned(15 downto 0);
    variable Pending         : boolean   := false;  -- StartxSI seen, no result yet
    variable AllPeriodsValid : boolean   := false;  -- PeriodEcsValidxSI(4) seen since StartxSI
  begin
    wait until rising_edge(ClkxC);
    -- signal values sampled here are the ones of the clock cycle just ended
    check(ValidxS = '0' or ValidxS = '1', "ValidxSO has a metavalue");

    if ValidPrev = '1' and AckPrev = '0' then
      check_equal(ValidxS, '1', "ValidxSO went back to 0 before AckxSI");
      check(AnglexD = AnglePrev, "AnglexDO changed while ValidxSO = 1");
    elsif ValidPrev = '1' and AckPrev = '1' then
      check_equal(ValidxS, '0', "ValidxSO must be 0 the clock cycle after AckxSI");
    end if;

    if ValidPrev = '0' and ValidxS = '1' then
      check(Pending and AllPeriodsValid,
            "ValidxSO rose without StartxSI and all PeriodEcsValidxSI");
      Pending := false;
    end if;

    if StartxS = '1' then
      Pending         := true;
      AllPeriodsValid := false;
    elsif Pending and PeriodEcsValidxS(4) = '1' then
      AllPeriodsValid := true;
    end if;

    ValidPrev := ValidxS;
    AckPrev   := AckxS;
    AnglePrev := AnglexD;
  end process;

end architecture tb;

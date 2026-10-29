-------------------------------------------------------------------------------
-- AngleCompute_tb : VUnit testbench for AngleCompute
--
-- Reference model (real arithmetic, from the quantized stimulus values):
--   Norm(i)   = PeriodEcs(i) * GainNorm(i) + OffsetNorm(i)          i = 1..4
--   Lin(i)    = sum_{k=1..ORDER} Tk(Norm(i)) * ChebyshevCoeff(i)(k)
--               T0 = 1, T1 = x, Tk+2 = 2*x*Tk+1 - Tk
--   DeltaECSX = Lin(1) - Lin(2) + GammaX
--   DeltaECSZ = Lin(3) - Lin(4) + GammaZ
--   Num       = CosThetaZ*DeltaECSX - SinThetaX*DeltaECSZ
--   Den       = SinThetaZ*DeltaECSX + CosThetaX*DeltaECSZ
--   Angle     = atan2(Num, Den) mapped on the full circle (65536 = 2*pi)
--
-- Fixed-point formats of the DUT inputs:
--   GainNorm        : Q-4.36  (36 fractional bits)
--   OffsetNorm      : Q8.24   (24 fractional bits)
--   ChebyshevCoeff  : Q8.24
--   GammaX/Z        : Q8.24
--   Cos/SinThetaX/Z : Q2.30   (30 fractional bits)
--
-- Checks performed on every transaction:
--   * AnglexDO matches the model within ANGLE_TOL_LSB (circular distance)
--   * ValidxSO rises less than 1 ms after the rising edge of PeriodEcsValidxSI(4)
--   * ValidxSO and AnglexDO are held stable until AckxSI, then ValidxSO
--     returns to 0 within ACK_TO_VALID_LOW_MAX clock cycles
--   * ValidxSO never rises before all 4 periods are valid (protocol monitor)
--   * the DUT waits for each PeriodEcsValidxSI(i): PeriodEcsxDI(i) carries
--     random garbage until its valid rises
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
    runner_cfg     : string;
    SEED           : natural  := 1;     -- seed of the random generator
    NB_RANDOM_RUNS : positive := 200;   -- number of fully random transactions
    ANGLE_TOL_LSB  : real     := 4.0    -- accepted angle error (1 LSB = 2*pi/65536)
  );
end entity AngleCompute_tb;

architecture tb of AngleCompute_tb is

  ---------------------------------------------------------------------------
  -- Constants
  ---------------------------------------------------------------------------
  constant CLK_PERIOD           : time    := 10 ns;  -- 100 MHz
  constant MAX_LATENCY          : time    := 1 ms;   -- PeriodEcsValidxSI(4) -> ValidxSO
  constant ACK_TO_VALID_LOW_MAX : positive := 2;     -- clock cycles

  constant GAIN_FRAC   : natural := 36;  -- Q-4.36
  constant OFFSET_FRAC : natural := 24;  -- Q8.24
  constant COEFF_FRAC  : natural := 24;  -- Q8.24
  constant GAMMA_FRAC  : natural := 24;  -- Q8.24
  constant TRIG_FRAC   : natural := 30;  -- Q2.30

  constant PERIOD_MIN : natural := 1_000_000;
  constant PERIOD_MAX : natural := 2_000_000;

  constant Q8_24_MAX : real := 128.0;   -- range of the Q8.24 intermediate values
  constant MIN_MAG   : real := 0.05;    -- minimum |(Num, Den)| so atan2 stays well conditioned

  constant ANGLE_FULL : real := 65536.0;

  ---------------------------------------------------------------------------
  -- Stimulus / expected values
  ---------------------------------------------------------------------------
  type Stimulus_t is record
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
  end record Stimulus_t;

  type Expected_t is record
    Norm   : real_vector(1 to 4);
    Lin    : real_vector(1 to 4);
    DeltaX : real;
    DeltaZ : real;
    Num    : real;
    Den    : real;
    Angle  : real;                      -- in LSB, [0, 65536)
  end record Expected_t;

  ---------------------------------------------------------------------------
  -- Fixed-point helpers
  ---------------------------------------------------------------------------
  function to_fix(x : real; frac : natural) return signed is
    constant V : real := round(x * 2.0 ** frac);
  begin
    assert abs(V) <= 2147483647.0
      report "TB: value " & real'image(x) & " does not fit in 32 bits with " &
      integer'image(frac) & " fractional bits"
      severity failure;
    return to_signed(integer(V), 32);
  end function;

  -- truncation towards zero (used for the gain so that |Norm| stays <= 1)
  function to_fix_trunc(x : real; frac : natural) return signed is
    constant V : real := trunc(x * 2.0 ** frac);
  begin
    assert abs(V) <= 2147483647.0
      report "TB: value " & real'image(x) & " does not fit in 32 bits" severity failure;
    return to_signed(integer(V), 32);
  end function;

  function fix_to_real(s : signed; frac : natural) return real is
  begin
    return real(to_integer(s)) / 2.0 ** frac;
  end function;

  ---------------------------------------------------------------------------
  -- Reference model
  ---------------------------------------------------------------------------
  function chebyshev_lin(x : real; c : ChebyshevCoeff_t) return real is
    variable TkM1 : real := 1.0;        -- T(k-1)
    variable Tk   : real := x;          -- T(k)
    variable TkP1 : real;
    variable Acc  : real := 0.0;
  begin
    for k in 1 to ORDER loop
      Acc  := Acc + Tk * fix_to_real(c(c'low + k - 1), COEFF_FRAC);
      TkP1 := 2.0 * x * Tk - TkM1;
      TkM1 := Tk;
      Tk   := TkP1;
    end loop;
    return Acc;
  end function;

  function model(S : Stimulus_t) return Expected_t is
    variable E : Expected_t;
    variable A : real;
  begin
    for i in 1 to 4 loop
      E.Norm(i) := real(to_integer(S.Period(i))) * fix_to_real(S.Gain(i), GAIN_FRAC) +
                   fix_to_real(S.Offset(i), OFFSET_FRAC);
      E.Lin(i)  := chebyshev_lin(E.Norm(i), S.Coeff(i));
    end loop;
    E.DeltaX := E.Lin(1) - E.Lin(2) + fix_to_real(S.GammaX, GAMMA_FRAC);
    E.DeltaZ := E.Lin(3) - E.Lin(4) + fix_to_real(S.GammaZ, GAMMA_FRAC);
    E.Num    := fix_to_real(S.CosZ, TRIG_FRAC) * E.DeltaX - fix_to_real(S.SinX, TRIG_FRAC) * E.DeltaZ;
    E.Den    := fix_to_real(S.SinZ, TRIG_FRAC) * E.DeltaX + fix_to_real(S.CosX, TRIG_FRAC) * E.DeltaZ;
    if E.Num = 0.0 and E.Den = 0.0 then
      A := 0.0;
    else
      A := arctan(E.Num, E.Den);        -- (-pi, pi]
    end if;
    A := A * ANGLE_FULL / MATH_2_PI;
    if A < 0.0 then
      A := A + ANGLE_FULL;
    end if;
    E.Angle := A;
    return E;
  end function;

  -- Stimulus respects the input specification and the Q8.24 ranges,
  -- and atan2 is well conditioned
  function is_legal(S : Stimulus_t) return boolean is
    constant E : Expected_t := model(S);
  begin
    for i in 1 to 4 loop
      if to_integer(S.Period(i)) < PERIOD_MIN or to_integer(S.Period(i)) > PERIOD_MAX or
         abs(E.Norm(i)) > 1.0 or abs(E.Lin(i)) >= Q8_24_MAX then
        return false;
      end if;
    end loop;
    return abs(E.DeltaX) < Q8_24_MAX and abs(E.DeltaZ) < Q8_24_MAX and
           abs(E.Num) < Q8_24_MAX and abs(E.Den) < Q8_24_MAX and
           sqrt(E.Num ** 2 + E.Den ** 2) >= MIN_MAG;
  end function;

  -- Circular distance between the DUT angle and the expected one, in LSB
  function angle_error(Got : unsigned(15 downto 0); Expected : real) return real is
    variable D : real := real(to_integer(Got)) - Expected;
  begin
    return D - ANGLE_FULL * round(D / ANGLE_FULL);
  end function;

  function to_str(E : Expected_t) return string is
  begin
    return "Norm=(" & real'image(E.Norm(1)) & ", " & real'image(E.Norm(2)) & ", " &
      real'image(E.Norm(3)) & ", " & real'image(E.Norm(4)) & ") Lin=(" &
      real'image(E.Lin(1)) & ", " & real'image(E.Lin(2)) & ", " &
      real'image(E.Lin(3)) & ", " & real'image(E.Lin(4)) & ") DeltaECSX=" &
      real'image(E.DeltaX) & " DeltaECSZ=" & real'image(E.DeltaZ) & " Num=" &
      real'image(E.Num) & " Den=" & real'image(E.Den);
  end function;

  ---------------------------------------------------------------------------
  -- DUT signals
  ---------------------------------------------------------------------------
  signal ClkxC              : std_logic := '0';
  signal StartxS            : std_logic := '0';
  signal AnglexD            : unsigned(15 downto 0);
  signal ValidxS            : std_logic;
  signal AckxS              : std_logic := '0';
  signal PeriodEcsxD        : EcsPeriod_t(1 to 4)       := (others => (others => '0'));
  signal PeriodEcsValidxS   : std_logic_vector(1 to 4)  := (others => '0');
  signal GainNormxD         : EcsParam_t(1 to 4)        := (others => (others => '0'));
  signal OffsetNormxD       : EcsParam_t(1 to 4)        := (others => (others => '0'));
  signal ChebyshevCoeffxD   : ChebyshevCoeffArray_t     := (others => (others => (others => '0')));
  signal GammaXxD           : signed(31 downto 0)       := (others => '0');
  signal GammaZxD           : signed(31 downto 0)       := (others => '0');
  signal CosThetaXxD        : signed(31 downto 0)       := (others => '0');
  signal SinThetaXxD        : signed(31 downto 0)       := (others => '0');
  signal CosThetaZxD        : signed(31 downto 0)       := (others => '0');
  signal SinThetaZxD        : signed(31 downto 0)       := (others => '0');

begin

  ClkxC <= not ClkxC after CLK_PERIOD / 2;

  test_runner_watchdog(runner, (NB_RANDOM_RUNS + 500) * 2 * MAX_LATENCY);

  ---------------------------------------------------------------------------
  -- DUT
  ---------------------------------------------------------------------------
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

  ---------------------------------------------------------------------------
  -- Stimulus and checks
  ---------------------------------------------------------------------------
  main : process
    variable RV            : RandomPType;
    variable Stim          : Stimulus_t;
    variable NbTransaction : natural;
    variable MaxLatency    : time;
    variable MaxError      : real;

    -- Gain / offset mapping the calibration range [PMin, PMax] on [1, -1]
    procedure set_normalization(variable S : inout Stimulus_t; i : positive; PMin, PMax : real) is
    begin
      S.Gain(i)   := to_fix_trunc(-2.0 / (PMax - PMin), GAIN_FRAC);
      S.Offset(i) := to_fix(-(PMin + PMax) / 2.0 * fix_to_real(S.Gain(i), GAIN_FRAC), OFFSET_FRAC);
    end procedure;

    procedure set_thetas(variable S : inout Stimulus_t; ThetaX, ThetaZ : real) is
    begin
      S.CosX := to_fix(cos(ThetaX), TRIG_FRAC);
      S.SinX := to_fix(sin(ThetaX), TRIG_FRAC);
      S.CosZ := to_fix(cos(ThetaZ), TRIG_FRAC);
      S.SinZ := to_fix(sin(ThetaZ), TRIG_FRAC);
    end procedure;

    impure function rand_period return unsigned is
      variable P : natural;
    begin
      case RV.RandInt(0, 9) is
        when 0      => P := PERIOD_MIN;
        when 1      => P := PERIOD_MAX;
        when others => P := RV.RandInt(PERIOD_MIN, PERIOD_MAX);
      end case;
      return to_unsigned(P, 24);
    end function;

    -- Nominal parameters: exact normalization, identity linearization
    -- (Lin = Norm), no gamma, ThetaX = ThetaZ = 0  =>  Angle = atan2(DeltaX, DeltaZ)
    procedure nominal(variable S : out Stimulus_t) is
    begin
      for i in 1 to 4 loop
        S.Period(i) := to_unsigned((PERIOD_MIN + PERIOD_MAX) / 2, 24);
        set_normalization(S, i, real(PERIOD_MIN), real(PERIOD_MAX));
        S.Coeff(i)    := (others => (others => '0'));
        S.Coeff(i)(1) := to_fix(1.0, COEFF_FRAC);
      end loop;
      S.GammaX := (others => '0');
      S.GammaZ := (others => '0');
      set_thetas(S, 0.0, 0.0);
    end procedure;

    -- Fully random legal stimulus
    procedure random_stimulus(variable S : out Stimulus_t) is
      variable Wide : boolean;
      variable Amp  : real;
    begin
      loop
        Wide := RV.RandInt(0, 3) = 0;   -- 25 %: large coefficients / gammas
        for i in 1 to 4 loop
          S.Period(i) := rand_period;
          set_normalization(S, i, RV.RandReal(0.95e6, 1.0e6), RV.RandReal(2.0e6, 2.05e6));
          if Wide then
            for k in 1 to ORDER loop
              S.Coeff(i)(k) := to_fix(RV.RandReal(-2.0, 2.0), COEFF_FRAC);
            end loop;
          else
            S.Coeff(i)(1) := to_fix(RV.RandReal(0.8, 1.2), COEFF_FRAC);
            for k in 2 to ORDER loop
              Amp           := 0.1 / real(k);
              S.Coeff(i)(k) := to_fix(RV.RandReal(-Amp, Amp), COEFF_FRAC);
            end loop;
          end if;
        end loop;
        Amp := 1.0;
        if Wide then
          Amp := 8.0;
        end if;
        S.GammaX := to_fix(RV.RandReal(-Amp, Amp), GAMMA_FRAC);
        S.GammaZ := to_fix(RV.RandReal(-Amp, Amp), GAMMA_FRAC);
        set_thetas(S, RV.RandReal(0.0, MATH_2_PI), RV.RandReal(0.0, MATH_2_PI));
        exit when is_legal(S);
      end loop;
    end procedure;

    -- Random garbage on the periods that are not valid yet
    procedure scramble_periods(First : positive) is
    begin
      for j in First to 4 loop
        PeriodEcsxD(j) <= to_unsigned(RV.RandInt(0, 2 ** 24 - 1), 24);
      end loop;
    end procedure;

    procedure wait_cycles(N : natural) is
    begin
      for c in 1 to N loop
        wait until rising_edge(ClkxC);
      end loop;
    end procedure;

    ---------------------------------------------------------------------------
    -- One complete computation:
    --   apply parameters, pulse StartxSI, deliver the 4 periods sequentially
    --   (GapMax = max idle cycles before each valid), wait for ValidxSO,
    --   check angle and latency, hold for up to AckDelayMax cycles, pulse
    --   AckxSI and check that ValidxSO returns to 0.
    ---------------------------------------------------------------------------
    procedure run_transaction(S : Stimulus_t; Tag : string;
                              GapMax : natural := 20; AckDelayMax : natural := 10) is
      constant E      : Expected_t := model(S);
      variable T4     : time;
      variable Lat    : time;
      variable Angle  : unsigned(15 downto 0);
      variable Err    : real;
    begin
      check(is_legal(S), "TB: illegal stimulus for " & Tag, failure);
      NbTransaction := NbTransaction + 1;

      -- parameters are stable during the whole computation
      GainNormxD       <= S.Gain;
      OffsetNormxD     <= S.Offset;
      ChebyshevCoeffxD <= S.Coeff;
      GammaXxD         <= S.GammaX;
      GammaZxD         <= S.GammaZ;
      CosThetaXxD      <= S.CosX;
      SinThetaXxD      <= S.SinX;
      CosThetaZxD      <= S.CosZ;
      SinThetaZxD      <= S.SinZ;
      PeriodEcsValidxS <= (others => '0');
      scramble_periods(1);
      wait until rising_edge(ClkxC);

      -- start pulse (1 clock cycle)
      StartxS <= '1';
      wait until rising_edge(ClkxC);
      StartxS <= '0';

      -- periods become valid sequentially
      for i in 1 to 4 loop
        for c in 1 to RV.RandInt(0, GapMax) loop
          scramble_periods(i);
          wait until rising_edge(ClkxC);
        end loop;
        PeriodEcsxD(i)      <= S.Period(i);
        PeriodEcsValidxS(i) <= '1';
        if i < 4 then
          scramble_periods(i + 1);
          wait until rising_edge(ClkxC);
        end if;
      end loop;
      T4 := now;

      -- result
      wait until ValidxS = '1' for MAX_LATENCY;
      Lat := now - T4;
      check_equal(ValidxS, '1', Tag & ": ValidxSO not asserted within " & time'image(MAX_LATENCY) &
                                " after PeriodEcsValidxSI(4)");
      check(Lat < MAX_LATENCY, Tag & ": latency " & time'image(Lat) & " >= " & time'image(MAX_LATENCY));
      if Lat > MaxLatency then
        MaxLatency := Lat;
      end if;

      wait until falling_edge(ClkxC);
      Angle := AnglexD;
      Err   := angle_error(Angle, E.Angle);
      if abs(Err) > MaxError then
        MaxError := abs(Err);
      end if;
      check(abs(Err) <= ANGLE_TOL_LSB,
            Tag & ": AnglexDO = " & integer'image(to_integer(Angle)) & ", expected " &
            real'image(E.Angle) & " (error " & real'image(Err) & " LSB). " & to_str(E));
      debug(Tag & ": AnglexDO = " & integer'image(to_integer(Angle)) & ", expected " &
            real'image(E.Angle) & ", latency " & time'image(Lat));

      -- ValidxSO and AnglexDO held until AckxSI
      for c in 0 to RV.RandInt(0, AckDelayMax) loop
        wait until rising_edge(ClkxC);
        check_equal(ValidxS, '1', Tag & ": ValidxSO deasserted before AckxSI");
        check_equal(AnglexD, Angle, Tag & ": AnglexDO changed while ValidxSO = 1");
      end loop;

      -- acknowledge (1 clock cycle)
      AckxS <= '1';
      wait until rising_edge(ClkxC);
      AckxS <= '0';
      for c in 1 to ACK_TO_VALID_LOW_MAX loop
        wait until falling_edge(ClkxC);
        exit when ValidxS = '0';
      end loop;
      check_equal(ValidxS, '0', Tag & ": ValidxSO still 1 " & integer'image(ACK_TO_VALID_LOW_MAX) &
                                " clock cycles after AckxSI");
    end procedure;

    constant RADII : real_vector(0 to 2) := (0.25, 1.0, 6.0);
    variable Ang   : real;
    variable Rad   : real;

  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop
      RV.InitSeed(running_test_case & integer'image(SEED));
      NbTransaction := 0;
      MaxLatency    := 0 ns;
      MaxError      := 0.0;
      wait_cycles(5);
      check_equal(ValidxS, '0', "ValidxSO must be 0 after power-up");

      -------------------------------------------------------------------------
      if run("atan2_full_circle") then
        -- Lin = 0 (all coefficients 0), thetas = 0 => Angle = atan2(GammaX, GammaZ)
        for r in RADII'range loop
          Rad := RADII(r);
          for a in 0 to 47 loop
            for d in -1 to 1 loop
              Ang := real(a) * MATH_2_PI / 48.0 + real(d) * 1.0e-4;
              nominal(Stim);
              for i in 1 to 4 loop
                Stim.Coeff(i) := (others => (others => '0'));
              end loop;
              Stim.GammaX := to_fix(Rad * sin(Ang), GAMMA_FRAC);
              Stim.GammaZ := to_fix(Rad * cos(Ang), GAMMA_FRAC);
              run_transaction(Stim, "atan2 R=" & real'image(Rad) & " A=" & real'image(Ang));
            end loop;
          end loop;
        end loop;

      -------------------------------------------------------------------------
      elsif run("rotation_thetas") then
        -- fixed deltas, thetas sweeping the full circle
        for t in 0 to 35 loop
          nominal(Stim);
          for i in 1 to 4 loop
            Stim.Coeff(i) := (others => (others => '0'));
          end loop;
          Stim.GammaX := to_fix(RV.RandReal(-2.0, 2.0), GAMMA_FRAC);
          Stim.GammaZ := to_fix(RV.RandReal(-2.0, 2.0), GAMMA_FRAC);
          set_thetas(Stim, real(t) * MATH_2_PI / 36.0, RV.RandReal(0.0, MATH_2_PI));
          if is_legal(Stim) then
            run_transaction(Stim, "rotation t=" & integer'image(t));
          end if;
          set_thetas(Stim, RV.RandReal(0.0, MATH_2_PI), real(t) * MATH_2_PI / 36.0);
          if is_legal(Stim) then
            run_transaction(Stim, "rotation t=" & integer'image(t));
          end if;
        end loop;

      -------------------------------------------------------------------------
      elsif run("normalization") then
        -- identity linearization on one channel, angle = atan2(+-Norm, 1) or atan2(1, +-Norm)
        for ch in 1 to 4 loop
          for p in 0 to 10 loop
            nominal(Stim);
            for i in 1 to 4 loop
              Stim.Coeff(i) := (others => (others => '0'));
            end loop;
            Stim.Coeff(ch)(1) := to_fix(1.0, COEFF_FRAC);
            if ch <= 2 then
              Stim.GammaZ := to_fix(1.0, GAMMA_FRAC);
            else
              Stim.GammaX := to_fix(1.0, GAMMA_FRAC);
            end if;
            if p <= 8 then              -- 1.0e6, 1.125e6, ... 2.0e6
              Stim.Period(ch) := to_unsigned(PERIOD_MIN + p * (PERIOD_MAX - PERIOD_MIN) / 8, 24);
            else
              Stim.Period(ch) := rand_period;
              set_normalization(Stim, ch, RV.RandReal(0.95e6, 1.0e6), RV.RandReal(2.0e6, 2.05e6));
            end if;
            run_transaction(Stim, "normalization ch=" & integer'image(ch) & " p=" &
                                  integer'image(to_integer(Stim.Period(ch))));
          end loop;
        end loop;

      -------------------------------------------------------------------------
      elsif run("chebyshev_polynomials") then
        -- one single coefficient Ck = 1 on one channel => Lin = Tk(Norm)
        for ch in 1 to 4 loop
          for k in 1 to ORDER loop
            for p in 0 to 6 loop
              nominal(Stim);
              for i in 1 to 4 loop
                Stim.Coeff(i) := (others => (others => '0'));
              end loop;
              Stim.Coeff(ch)(k) := to_fix(1.0, COEFF_FRAC);
              if ch <= 2 then
                Stim.GammaZ := to_fix(1.0, GAMMA_FRAC);
              else
                Stim.GammaX := to_fix(1.0, GAMMA_FRAC);
              end if;
              Stim.Period(ch) := to_unsigned(PERIOD_MIN + p * (PERIOD_MAX - PERIOD_MIN) / 6, 24);
              run_transaction(Stim, "chebyshev ch=" & integer'image(ch) & " k=" & integer'image(k) &
                                    " p=" & integer'image(to_integer(Stim.Period(ch))));
            end loop;
          end loop;
        end loop;

      -------------------------------------------------------------------------
      elsif run("chebyshev_coefficients_signs") then
        -- negative coefficients, sums of polynomials
        for n in 1 to 50 loop
          nominal(Stim);
          for i in 1 to 4 loop
            Stim.Period(i) := rand_period;
            for k in 1 to ORDER loop
              Stim.Coeff(i)(k) := to_fix(real(RV.RandInt(-8, 8)) / 4.0, COEFF_FRAC);
            end loop;
          end loop;
          if is_legal(Stim) then
            run_transaction(Stim, "coeff signs n=" & integer'image(n));
          end if;
        end loop;

      -------------------------------------------------------------------------
      elsif run("random_parameters") then
        for n in 1 to NB_RANDOM_RUNS loop
          random_stimulus(Stim);
          run_transaction(Stim, "random n=" & integer'image(n));
        end loop;

      -------------------------------------------------------------------------
      elsif run("slow_period_valids") then
        -- long and irregular delays between the 4 period valids: the DUT
        -- must wait for each of them (garbage on the inputs meanwhile)
        for n in 1 to 10 loop
          random_stimulus(Stim);
          run_transaction(Stim, "slow n=" & integer'image(n), GapMax => 20_000);
        end loop;

      -------------------------------------------------------------------------
      elsif run("back_to_back") then
        -- periods valid on consecutive cycles, immediate acknowledge, no idle time
        for n in 1 to 50 loop
          random_stimulus(Stim);
          run_transaction(Stim, "back-to-back n=" & integer'image(n), GapMax => 0, AckDelayMax => 0);
        end loop;

      -------------------------------------------------------------------------
      elsif run("valid_held_until_ack") then
        -- long acknowledge delays, ValidxSO must stay 1 and AnglexDO stable
        for n in 1 to 10 loop
          random_stimulus(Stim);
          run_transaction(Stim, "ack delay n=" & integer'image(n), AckDelayMax => 5_000);
          -- ValidxSO must stay 0 while idle after the acknowledge
          for c in 1 to RV.RandInt(1, 1_000) loop
            wait until rising_edge(ClkxC);
            check_equal(ValidxS, '0', "ValidxSO reasserted without a new computation");
          end loop;
        end loop;

      end if;

      info(running_test_case & ": " & integer'image(NbTransaction) & " transactions, max latency " &
           time'image(MaxLatency) & ", max angle error " & real'image(MaxError) & " LSB");
    end loop;

    test_runner_cleanup(runner);
  end process main;

  ---------------------------------------------------------------------------
  -- Protocol monitor (independent of the stimulus process)
  --   * ValidxSO rises only after PeriodEcsValidxSI(4) rose following a start
  --   * ValidxSO falls only after it has been acknowledged
  --   * ValidxSO is never a metavalue
  ---------------------------------------------------------------------------
  protocol_monitor : process
    variable Armed     : boolean   := false;  -- all periods valid, result pending
    variable Acked     : boolean   := false;  -- AckxSI seen while ValidxSO = 1
    variable ValidPrev : std_logic := '0';
    variable V4Prev    : std_logic := '0';
  begin
    wait until rising_edge(ClkxC);
    check(ValidxS = '0' or ValidxS = '1', "ValidxSO is a metavalue: " & std_logic'image(ValidxS));

    if ValidxS = '1' and ValidPrev = '0' then
      check(Armed, "ValidxSO asserted before all PeriodEcsValidxSI were 1");
      Armed := false;
      Acked := false;
    end if;
    if ValidxS = '0' and ValidPrev = '1' then
      check(Acked, "ValidxSO deasserted without AckxSI");
    end if;
    if ValidxS = '1' and AckxS = '1' then
      Acked := true;
    end if;

    if StartxS = '1' then
      Armed := false;
    end if;
    if PeriodEcsValidxS(4) = '1' and V4Prev = '0' then
      check(PeriodEcsValidxS(1 to 3) = "111", "TB: PeriodEcsValidxSI(4) before 1 to 3");
      Armed := true;
    end if;

    ValidPrev := ValidxS;
    V4Prev    := PeriodEcsValidxS(4);
  end process protocol_monitor;

end architecture tb;

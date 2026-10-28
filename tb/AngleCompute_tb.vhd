-------------------------------------------------------------------------------
--! @file      AngleCompute_tb.vhd
--! @brief     VUnit testbench of the plate angle computation (AngleCompute)
--! @author    Yannick Soulie
--! @date      2026-10-28
--! @copyright Copyright (c) 2026 Thales Alenia Space
--! @standard  VHDL'08
--! @license   This source code is proprietary information of Thales Alenia
--!            Space. It can only be used for the purposes specified in the
--!            respective contract.
-------------------------------------------------------------------------------

-------------------------------------------------------------------------------
-- Libraries
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

use work.BamPkg.all;

-------------------------------------------------------------------------------
-- Entity
-------------------------------------------------------------------------------
--! @brief     VUnit testbench of the plate angle computation (AngleCompute)
--! Description: self-checking testbench. Every result of the DUT is compared
--!              with a floating-point reference model of the specification:
--!   X(n)   = Period(n) * Gain(n) / 2**36 + Offset(n) / 2**24
--!   Lin(n) = sum_{k=1..6} Coeff(n)(k) * Tk(X(n))   (Tk built by recurrence,
--!            independent from the Clenshaw evaluation of the RTL)
--!   DX     = Lin(1) - Lin(2) + CoeffX(12),  DZ = Lin(3) - Lin(4) + CoeffZ(12)
--!   Num    = CosZ*DX - SinX*DZ,             Den = SinZ*DX + CosX*DZ
--!   Angle  = atan2(Num, Den) in [0, 2*pi), 65536 LSB = 2*pi
--!   The tolerance is 1 LSB (output rounding + CORDIC residual) plus the
--!   propagation of the Clenshaw rounding error (LIN_ERR) through the angle.
--!   Directed tests whose result is exactly representable are checked
--!   bit-exact.
--!
--!   Generics
--!     SEED     : seed of the random tests
--!     N_RANDOM : number of computations of the random test
-------------------------------------------------------------------------------
entity AngleCompute_tb is
  generic(
    runner_cfg : string;
    SEED       : positive := 1;         -- seed of the random tests
    N_RANDOM   : positive := 200        -- number of random computations
  );
end entity AngleCompute_tb;

-------------------------------------------------------------------------------
-- Architecture
-------------------------------------------------------------------------------
architecture tb of AngleCompute_tb is

  constant CLK_PERIOD  : time     := 10 ns; -- 100 MHz
  constant MAX_LATENCY : positive := 1000; -- clock cycles from StartxSI to ValidxSO (all periods valid)
  constant FULL_CIRCLE : real     := 65536.0; -- 2*pi in output LSB
  constant LIN_ERR     : real     := 16.0; -- bound of the Clenshaw rounding error on one Lin (coefficient LSB)

  constant S32_MAX : integer := 2147483647; -- 2**31 - 1
  constant S32_MIN : integer := -2147483647 - 1; -- -2**31
  constant ONE_Q30 : integer := 2 ** 30; -- Cos/Sin = 1.0

  -- nominal normalization: X = (Period - 1.5e6) * 2e-6, |X| <= 1
  constant NOM_GAIN_R : real    := 2.0e-6 * 2.0 ** 36;
  constant NOM_GAIN   : integer := integer(floor(NOM_GAIN_R));
  constant NOM_OFFSET : integer := integer(round(-1.5e6 * real(NOM_GAIN) / 2.0 ** 12));

  -- exact normalization: Period = 2**20 and Gain = 2**16 give Period*Gain = 1.0
  constant P_ONE : natural := 2 ** 20;
  constant G_ONE : integer := 2 ** 16;
  constant O_ONE : integer := 2 ** 24;  -- Offset = 1.0

  ---------------------------------------------------------------------------
  -- Stimulus / reference model
  ---------------------------------------------------------------------------
  type Params_t is record
    Period : EcsPeriod_t(1 to 4);
    Gain   : EcsParam_t(1 to 4);
    Offset : EcsParam_t(1 to 4);
    CoeffX : ChebyshevCoeff_t(0 to 12);
    CoeffZ : ChebyshevCoeff_t(0 to 12);
    CosX   : signed(31 downto 0);
    SinX   : signed(31 downto 0);
    CosZ   : signed(31 downto 0);
    SinZ   : signed(31 downto 0);
  end record Params_t;

  type Expect_t is record
    Defined : boolean;                  -- false when Num = Den = 0
    Angle   : real;                     -- expected angle in [0, 65536) LSB
    Tol     : real;                     -- tolerance in LSB
  end record Expect_t;

  -- saturated conversion of a real to a signed 32-bit word
  function to_s32(x : real) return signed is
  begin
    if x >= real(S32_MAX) then
      return to_signed(S32_MAX, 32);
    elsif x <= real(S32_MIN) then
      return to_signed(S32_MIN, 32);
    end if;
    return to_signed(integer(round(x)), 32);
  end function to_s32;

  function to_real(x : signed) return real is
  begin
    return real(to_integer(x));
  end function to_real;

  -- all Chebyshev coefficients null, X = 0, identity rotation
  function default_params return Params_t is
    variable p : Params_t;
  begin
    p.Period := (others => to_unsigned(1_500_000, 24));
    p.Gain   := (others => to_signed(NOM_GAIN, 32));
    p.Offset := (others => to_signed(NOM_OFFSET, 32));
    p.CoeffX := (others => (others => '0'));
    p.CoeffZ := (others => (others => '0'));
    p.CosX   := to_signed(ONE_Q30, 32);
    p.SinX   := (others => '0');
    p.CosZ   := to_signed(ONE_Q30, 32);
    p.SinZ   := (others => '0');
    return p;
  end function default_params;

  -- DX = dx, DZ = dz, identity rotation -> Angle = atan2(dx, dz)
  function vector_params(dx, dz : integer) return Params_t is
    variable p : Params_t := default_params;
  begin
    p.CoeffX(12) := to_signed(dx, 32);
    p.CoeffZ(12) := to_signed(dz, 32);
    return p;
  end function vector_params;

  -- normalization X = x of every ECS through the offset (Period*Gain = 1.0)
  function exact_norm_params(x : real_vector(1 to 4)) return Params_t is
    variable p : Params_t := default_params;
  begin
    for n in 1 to 4 loop
      p.Period(n) := to_unsigned(P_ONE, 24);
      p.Gain(n)   := to_signed(G_ONE, 32);
      p.Offset(n) := to_s32((x(n) - 1.0) * real(O_ONE));
    end loop;
    return p;
  end function exact_norm_params;

  -- linearized period: sum_{k=1..ORDER} c(k) * Tk(X)
  function lin_model(per : unsigned; gain, offset : signed; c : ChebyshevCoeff_t) return real is
    variable x, t0, t1, t2, acc : real;
  begin
    x   := real(to_integer(per)) * to_real(gain) / 2.0 ** 36 + to_real(offset) / 2.0 ** 24;
    t0  := 1.0;
    t1  := x;
    acc := to_real(c(c'low)) * t1;
    for k in 2 to c'length loop
      t2  := 2.0 * x * t1 - t0;
      acc := acc + to_real(c(c'low + k - 1)) * t2;
      t0  := t1;
      t1  := t2;
    end loop;
    return acc;
  end function lin_model;

  function model(p : Params_t) return Expect_t is
    variable lin              : real_vector(1 to 4);
    variable dx, dz, num, den : real;
    variable cx, sx, cz, sz   : real;
    variable mag, a           : real;
    variable e                : Expect_t;
  begin
    lin(1) := lin_model(p.Period(1), p.Gain(1), p.Offset(1), p.CoeffX(0 to 5));
    lin(2) := lin_model(p.Period(2), p.Gain(2), p.Offset(2), p.CoeffX(6 to 11));
    lin(3) := lin_model(p.Period(3), p.Gain(3), p.Offset(3), p.CoeffZ(0 to 5));
    lin(4) := lin_model(p.Period(4), p.Gain(4), p.Offset(4), p.CoeffZ(6 to 11));
    dx     := lin(1) - lin(2) + to_real(p.CoeffX(12));
    dz     := lin(3) - lin(4) + to_real(p.CoeffZ(12));
    cx     := to_real(p.CosX);
    sx     := to_real(p.SinX);
    cz     := to_real(p.CosZ);
    sz     := to_real(p.SinZ);
    num    := cz * dx - sx * dz;
    den    := sz * dx + cx * dz;
    mag    := sqrt(num * num + den * den);
    if mag = 0.0 then
      e := (Defined => false, Angle => 0.0, Tol => FULL_CIRCLE);
      return e;
    end if;
    a := arctan(num, den);
    if a < 0.0 then
      a := a + MATH_2_PI;
    end if;
    e.Defined := true;
    e.Angle   := a / MATH_2_PI * FULL_CIRCLE;
    if e.Angle >= FULL_CIRCLE then
      e.Angle := e.Angle - FULL_CIRCLE;
    end if;
    -- |error on (Num, Den)| <= (|CosZ|+|SinX|+|SinZ|+|CosX|) * 2*LIN_ERR
    e.Tol := 1.0 + (abs(cz) + abs(sx) + abs(sz) + abs(cx)) * 2.0 * LIN_ERR / mag * FULL_CIRCLE / MATH_2_PI;
    return e;
  end function model;

  -- signed circular distance actual - expected, in [-32768, 32768)
  function angle_err(actual : unsigned; expected : real) return real is
    variable err : real;
  begin
    err := real(to_integer(actual)) - expected;
    if err >= FULL_CIRCLE / 2.0 then
      err := err - FULL_CIRCLE;
    elsif err < -FULL_CIRCLE / 2.0 then
      err := err + FULL_CIRCLE;
    end if;
    return err;
  end function angle_err;

  -- expected angle of a direction given in degrees
  function deg(d : real) return unsigned is
  begin
    return to_unsigned(integer(round(d / 360.0 * FULL_CIRCLE)) mod 65536, 16);
  end function deg;

  ---------------------------------------------------------------------------
  -- DUT signals
  ---------------------------------------------------------------------------
  signal ClkxC              : std_logic                := '0';
  signal StartxS            : std_logic                := '0';
  signal AnglexD            : unsigned(15 downto 0);
  signal ValidxS            : std_logic;
  signal AckxS              : std_logic                := '0';
  signal PeriodEcsxD        : EcsPeriod_t(1 to 4)      := (others => (others => '0'));
  signal PeriodEcsValidxS   : std_logic_vector(1 to 4) := (others => '1');
  signal GainNormxD         : EcsParam_t(1 to 4)       := (others => (others => '0'));
  signal OffsetNormxD       : EcsParam_t(1 to 4)       := (others => (others => '0'));
  signal ChebyshevCoeffXxD  : ChebyshevCoeff_t(0 to 12) := (others => (others => '0'));
  signal ChebyshevCoeffZxD  : ChebyshevCoeff_t(0 to 12) := (others => (others => '0'));
  signal CosThetaXxD        : signed(31 downto 0)      := (others => '0');
  signal SinThetaXxD        : signed(31 downto 0)      := (others => '0');
  signal CosThetaZxD        : signed(31 downto 0)      := (others => '0');
  signal SinThetaZxD        : signed(31 downto 0)      := (others => '0');

begin

  ClkxC <= not ClkxC after CLK_PERIOD / 2;

  test_runner_watchdog(runner, 100 ms);

  ---------------------------------------------------------------------------
  -- DUT
  ---------------------------------------------------------------------------
  dut : entity work.AngleCompute
    port map(
      ClkxCI             => ClkxC,
      StartxSI           => StartxS,
      AnglexDO           => AnglexD,
      ValidxSO           => ValidxS,
      AckxSI             => AckxS,
      PeriodEcsxDI       => PeriodEcsxD,
      PeriodEcsValidxSI  => PeriodEcsValidxS,
      GainNormxDI        => GainNormxD,
      OffsetNormxDI      => OffsetNormxD,
      ChebyshevCoeffXxDI => ChebyshevCoeffXxD,
      ChebyshevCoeffZxDI => ChebyshevCoeffZxD,
      CosThetaXxDI       => CosThetaXxD,
      SinThetaXxDI       => SinThetaXxD,
      CosThetaZxDI       => CosThetaZxD,
      SinThetaZxDI       => SinThetaZxD
    );

  ---------------------------------------------------------------------------
  -- Protocol monitor: AnglexDO must not change while ValidxSO is high
  ---------------------------------------------------------------------------
  monitor : process(ClkxC)
    variable PrevValidV : std_logic := '0';
    variable PrevAngleV : unsigned(15 downto 0);
  begin
    if rising_edge(ClkxC) then
      if PrevValidV = '1' and ValidxS = '1' then
        check_equal(AnglexD, PrevAngleV, "AnglexDO changed while ValidxSO high");
      end if;
      PrevValidV := ValidxS;
      PrevAngleV := AnglexD;
    end if;
  end process monitor;

  ---------------------------------------------------------------------------
  -- Test sequencer
  ---------------------------------------------------------------------------
  main : process
    variable Seed1V    : positive := SEED;
    variable Seed2V    : positive := SEED + 7919;
    variable MaxErrV   : real     := 0.0;
    variable MaxLatV   : natural  := 0;
    variable AngleV    : unsigned(15 downto 0);
    variable LatencyV  : natural;
    variable ParamsV   : Params_t;
    variable ExpectV   : Expect_t;
    variable StartTV   : time;

    impure function rand return real is
      variable r : real;
    begin
      uniform(Seed1V, Seed2V, r);
      return r;
    end function rand;

    -- uniform in [-amp, amp]
    impure function rand_sym(amp : real) return real is
    begin
      return (2.0 * rand - 1.0) * amp;
    end function rand_sym;

    impure function rand_params return Params_t is
      variable p        : Params_t;
      variable g        : real;
      variable scale    : real;
      variable tx, tz   : real;
      variable amp      : real;
    begin
      for n in 1 to 4 loop
        p.Period(n) := to_unsigned(1_000_000 + integer(floor(rand * 1.0e6)), 24);
        -- |X| <= 1 for any period in [1e6, 2e6]
        g           := floor(NOM_GAIN_R * (0.9 + 0.099 * rand));
        p.Gain(n)   := to_s32(g);
        p.Offset(n) := to_s32(-1.5e6 * g / 2.0 ** 12);
      end loop;
      -- coefficient magnitude from 2**16 up to the full 32-bit range
      scale := 2.0 ** (16.0 + 15.0 * rand);
      for k in 0 to 12 loop
        p.CoeffX(k) := to_s32(rand_sym(scale));
        p.CoeffZ(k) := to_s32(rand_sym(scale));
      end loop;
      -- any rotation, any (common) format of the Cos/Sin terms
      tx     := MATH_2_PI * rand;
      tz     := MATH_2_PI * rand;
      amp    := 2.0 ** (16.0 + 15.0 * rand);
      p.CosX := to_s32(amp * cos(tx));
      p.SinX := to_s32(amp * sin(tx));
      p.CosZ := to_s32(amp * cos(tz));
      p.SinZ := to_s32(amp * sin(tz));
      return p;
    end function rand_params;

    procedure apply(p : Params_t) is
    begin
      PeriodEcsxD       <= p.Period;
      GainNormxD        <= p.Gain;
      OffsetNormxD      <= p.Offset;
      ChebyshevCoeffXxD <= p.CoeffX;
      ChebyshevCoeffZxD <= p.CoeffZ;
      CosThetaXxD       <= p.CosX;
      SinThetaXxD       <= p.SinX;
      CosThetaZxD       <= p.CosZ;
      SinThetaZxD       <= p.SinZ;
    end procedure apply;

    -- apply parameters and pulse StartxSI
    procedure start(p : Params_t) is
    begin
      wait until rising_edge(ClkxC);
      apply(p);
      StartxS <= '1';
      wait until rising_edge(ClkxC);
      StartxS <= '0';
      StartTV := now;
    end procedure start;

    -- wait ValidxSO (at most max_cycles), return the angle and the latency
    procedure wait_valid(angle : out unsigned; latency : out natural; max_cycles : positive := MAX_LATENCY) is
    begin
      if ValidxS /= '1' then
        wait until ValidxS = '1' for max_cycles * CLK_PERIOD;
      end if;
      check_equal(ValidxS, '1', "ValidxSO not asserted within " & to_string(max_cycles) & " cycles");
      angle   := AnglexD;
      latency := (now - StartTV) / CLK_PERIOD;
    end procedure wait_valid;

    -- pulse AckxSI and check that ValidxSO is released
    procedure acknowledge is
    begin
      wait until rising_edge(ClkxC);
      AckxS <= '1';
      wait until rising_edge(ClkxC);
      AckxS <= '0';
      if ValidxS /= '0' then
        wait until ValidxS = '0' for 4 * CLK_PERIOD;
      end if;
      check_equal(ValidxS, '0', "ValidxSO not released after AckxSI");
    end procedure acknowledge;

    -- full computation with handshake
    procedure compute(p : Params_t; angle : out unsigned) is
      variable a : unsigned(15 downto 0);
    begin
      start(p);
      wait_valid(a, LatencyV);
      if LatencyV > MaxLatV then
        MaxLatV := LatencyV;
      end if;
      acknowledge;
      angle := a;
    end procedure compute;

    -- check one result against the reference model
    procedure check_model(p : Params_t; angle : unsigned; msg : string) is
      variable e   : Expect_t;
      variable err : real;
    begin
      e := model(p);
      if not e.Defined then
        return;
      end if;
      err := angle_err(angle, e.Angle);
      if abs (err) > MaxErrV then
        MaxErrV := abs (err);
      end if;
      check(abs (err) <= e.Tol, msg & ": AnglexDO=" & to_string(to_integer(angle)) &
            " expected=" & to_string(e.Angle, 3) & " error=" & to_string(err, 3) &
            " tolerance=" & to_string(e.Tol, 3) & " LSB");
    end procedure check_model;

    procedure check_case(p : Params_t; msg : string) is
      variable a : unsigned(15 downto 0);
    begin
      compute(p, a);
      check_model(p, a, msg);
    end procedure check_case;

    procedure check_exact(p : Params_t; expected : unsigned; msg : string) is
      variable a : unsigned(15 downto 0);
    begin
      compute(p, a);
      check_equal(a, expected, msg);
    end procedure check_exact;

    constant R : integer := 2 ** 30;    -- magnitude of the directed vectors
    type     Vec_t is array (natural range <>) of integer_vector(0 to 1);
    constant DIRS : Vec_t := ((1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1),
                              (-1, 1), (-1, -1), (3, 4), (-5, 12), (7, -24), (-8, -15));
    constant SHIFTS : integer_vector := (0, 1, 2, 3, 5, 8, 13, 17, 21, 24, 26); -- 24 * 2**26 < 2**31

    variable xV    : real_vector(1 to 4);
    variable cV    : integer;
    variable thV   : real;
    variable mV    : integer;
    variable sV    : integer;
  begin
    test_runner_setup(runner, runner_cfg);
    show(get_logger(default_checker), display_handler, pass);

    while test_suite loop
      StartxS          <= '0';
      AckxS            <= '0';
      PeriodEcsValidxS <= (others => '1');
      apply(default_params);
      MaxErrV          := 0.0;
      MaxLatV          := 0;
      wait until rising_edge(ClkxC);

      -----------------------------------------------------------------------
      if run("no_output_without_start") then
        apply(vector_params(R, R));
        wait until ValidxS = '1' for 5 * MAX_LATENCY * CLK_PERIOD;
        check_equal(ValidxS, '0', "ValidxSO asserted without StartxSI");

      -----------------------------------------------------------------------
      -- the eight principal directions, bit-exact
      elsif run("cardinal_directions") then
        check_exact(vector_params(0, R), deg(0.0), "0 deg");
        check_exact(vector_params(R, R), deg(45.0), "45 deg");
        check_exact(vector_params(R, 0), deg(90.0), "90 deg");
        check_exact(vector_params(R, -R), deg(135.0), "135 deg");
        check_exact(vector_params(0, -R), deg(180.0), "180 deg");
        check_exact(vector_params(-R, -R), deg(225.0), "225 deg");
        check_exact(vector_params(-R, 0), deg(270.0), "270 deg");
        check_exact(vector_params(-R, R), deg(315.0), "315 deg");

      -----------------------------------------------------------------------
      -- full circle (all CORDIC quadrants / pre-rotations), large and small
      -- vectors (normalization of Num/Den)
      elsif run("full_circle_sweep") then
        for i in 0 to 511 loop
          thV := MATH_2_PI * (real(i) * 128.0 + 37.25) / FULL_CIRCLE;
          if i mod 2 = 0 then
            mV := R;
          else
            mV := 2 ** 12;
          end if;
          ParamsV := vector_params(integer(round(real(mV) * sin(thV))),
                                   integer(round(real(mV) * cos(thV))));
          check_case(ParamsV, "sweep point " & to_string(i));
        end loop;
        info("max error " & to_string(MaxErrV, 3) & " LSB, max latency " & to_string(MaxLatV) & " cycles");

      -----------------------------------------------------------------------
      -- same directions with magnitudes from 1 to 2**31: the result must not
      -- depend on the scale (normalization S_NORM)
      elsif run("magnitude_independence") then
        for d in DIRS'range loop
          for s in SHIFTS'range loop
            sV := 2 ** SHIFTS(s);
            check_case(vector_params(DIRS(d)(0) * sV, DIRS(d)(1) * sV),
                       "direction (" & to_string(DIRS(d)(0)) & "," & to_string(DIRS(d)(1)) &
                       ") * 2**" & to_string(SHIFTS(s)));
          end loop;
        end loop;
        -- smallest possible vectors are exact
        check_exact(vector_params(1, 0), deg(90.0), "DX = 1");
        check_exact(vector_params(0, 1), deg(0.0), "DZ = 1");
        check_exact(vector_params(-1, -1), deg(225.0), "DX = DZ = -1");

      -----------------------------------------------------------------------
      -- each rotation term alone, with its sign: Num/Den on an axis, bit-exact
      elsif run("rotation_terms") then
        ParamsV      := vector_params(R, R);
        ParamsV.CosX := (others => '0');
        ParamsV.SinX := (others => '0');
        ParamsV.CosZ := to_signed(ONE_Q30, 32);
        ParamsV.SinZ := (others => '0');
        check_exact(ParamsV, deg(90.0), "Num = CosZ*DX > 0, Den = 0");
        ParamsV.CosZ := (others => '0');
        ParamsV.SinX := to_signed(ONE_Q30, 32);
        check_exact(ParamsV, deg(270.0), "Num = -SinX*DZ < 0, Den = 0");
        ParamsV.SinX := (others => '0');
        ParamsV.SinZ := to_signed(ONE_Q30, 32);
        check_exact(ParamsV, deg(0.0), "Num = 0, Den = SinZ*DX > 0");
        ParamsV.SinZ := (others => '0');
        ParamsV.CosX := to_signed(-ONE_Q30, 32);
        check_exact(ParamsV, deg(180.0), "Num = 0, Den = CosX*DZ < 0");
        -- rotations of a fixed vector by known angles
        for i in 0 to 35 loop
          thV          := MATH_2_PI * real(i) / 36.0 + 0.01;
          ParamsV      := vector_params(R / 4, R / 3);
          ParamsV.CosX := to_s32(real(ONE_Q30) * cos(thV));
          ParamsV.SinX := to_s32(real(ONE_Q30) * sin(thV));
          ParamsV.CosZ := to_s32(real(ONE_Q30) * cos(thV + 0.3));
          ParamsV.SinZ := to_s32(real(ONE_Q30) * sin(thV + 0.3));
          check_case(ParamsV, "rotation " & to_string(i));
        end loop;

      -----------------------------------------------------------------------
      -- each Chebyshev coefficient of each ECS alone (X = 0.5, exact):
      -- checks the coefficient order, the ECS -> coefficient mapping and the
      -- sign of each Lin in DeltaECSX/Z
      elsif run("coefficient_mapping") then
        for n in 1 to 4 loop
          for k in 1 to 6 loop
            ParamsV := exact_norm_params((0.5, 0.5, 0.5, 0.5));
            cV      := ((n - 1) mod 2) * 6 + k - 1;
            if n <= 2 then
              ParamsV.CoeffX(cV) := to_signed(2 ** 28, 32);
              ParamsV.CoeffZ(12) := to_signed(2 ** 26, 32);
            else
              ParamsV.CoeffZ(cV) := to_signed(2 ** 28, 32);
              ParamsV.CoeffX(12) := to_signed(2 ** 26, 32);
            end if;
            check_case(ParamsV, "ECS" & to_string(n) & " coefficient " & to_string(k));
          end loop;
        end loop;
        -- T1 only: Lin = 2**27, bit-exact signs of each ECS
        for n in 1 to 4 loop
          ParamsV := exact_norm_params((0.5, 0.5, 0.5, 0.5));
          if n <= 2 then
            ParamsV.CoeffX((n - 1) * 6) := to_signed(2 ** 28, 32);
          else
            ParamsV.CoeffZ((n - 3) * 6) := to_signed(2 ** 28, 32);
          end if;
          case n is
            when 1      => check_exact(ParamsV, deg(90.0), "+Lin(1) in DeltaECSX");
            when 2      => check_exact(ParamsV, deg(270.0), "-Lin(2) in DeltaECSX");
            when 3      => check_exact(ParamsV, deg(0.0), "+Lin(3) in DeltaECSZ");
            when others => check_exact(ParamsV, deg(180.0), "-Lin(4) in DeltaECSZ");
          end case;
        end loop;

      -----------------------------------------------------------------------
      -- period, gain and offset of each ECS routed to the right linearizer
      elsif run("normalization_mapping") then
        xV := (0.5, -0.25, 0.125, 0.75);
        -- distinct offsets
        ParamsV           := exact_norm_params(xV);
        ParamsV.CoeffX(0) := to_signed(2 ** 28, 32);
        ParamsV.CoeffX(6) := to_signed(2 ** 28, 32);
        ParamsV.CoeffZ(0) := to_signed(2 ** 28, 32);
        ParamsV.CoeffZ(6) := to_signed(2 ** 28, 32);
        check_case(ParamsV, "distinct offsets");
        -- distinct gains
        for n in 1 to 4 loop
          ParamsV.Gain(n)   := to_s32(xV(n) * real(G_ONE));
          ParamsV.Offset(n) := (others => '0');
        end loop;
        check_case(ParamsV, "distinct gains");
        -- distinct periods: X = Period / 2**20 - 1.5
        xV := (0.25, -0.5, 0.125, 0.375);
        for n in 1 to 4 loop
          ParamsV.Period(n) := to_unsigned(integer((xV(n) + 1.5) * real(P_ONE)), 24);
          ParamsV.Gain(n)   := to_signed(G_ONE, 32);
          ParamsV.Offset(n) := to_s32(-1.5 * real(O_ONE));
        end loop;
        check_case(ParamsV, "distinct periods");
        -- periods at the limits of the range with the nominal normalization
        ParamsV           := default_params;
        ParamsV.Period    := (to_unsigned(1_000_000, 24), to_unsigned(2_000_000, 24),
                              to_unsigned(1_999_999, 24), to_unsigned(1_000_001, 24));
        ParamsV.CoeffX(0) := to_signed(2 ** 28, 32);
        ParamsV.CoeffX(7) := to_signed(-2 ** 27, 32);
        ParamsV.CoeffZ(2) := to_signed(2 ** 28, 32);
        ParamsV.CoeffZ(9) := to_signed(2 ** 27, 32);
        check_case(ParamsV, "period range limits");

      -----------------------------------------------------------------------
      -- all inputs at the limits of their range, |X| = 1: no internal overflow
      elsif run("extreme_values") then
        for q in 0 to 3 loop
          -- Lin(1) = 6*(2**31-1), Lin(2) = -6*2**31 (X = 1)
          -- Lin(3) = 6*2**31 - 3, Lin(4) = -(6*2**31 - 3) (X = -1)
          ParamsV := exact_norm_params((1.0, 1.0, -1.0, -1.0));
          for k in 0 to 5 loop
            ParamsV.CoeffX(k)     := to_signed(S32_MAX, 32);
            ParamsV.CoeffX(6 + k) := to_signed(S32_MIN, 32);
            if k mod 2 = 0 then     -- odd order: T(-1) = -1
              ParamsV.CoeffZ(k)     := to_signed(S32_MIN, 32);
              ParamsV.CoeffZ(6 + k) := to_signed(S32_MAX, 32);
            else
              ParamsV.CoeffZ(k)     := to_signed(S32_MAX, 32);
              ParamsV.CoeffZ(6 + k) := to_signed(S32_MIN, 32);
            end if;
          end loop;
          ParamsV.CoeffX(12) := to_signed(S32_MAX, 32);
          ParamsV.CoeffZ(12) := to_signed(S32_MAX, 32);
          -- |Num| and |Den| maximal, one quadrant per q
          if q mod 2 = 0 then
            ParamsV.CosZ := to_signed(S32_MAX, 32);
            ParamsV.SinX := to_signed(S32_MIN, 32);
          else
            ParamsV.CosZ := to_signed(S32_MIN, 32);
            ParamsV.SinX := to_signed(S32_MAX, 32);
          end if;
          if q / 2 = 0 then
            ParamsV.SinZ := to_signed(S32_MAX, 32);
            ParamsV.CosX := to_signed(S32_MAX, 32);
          else
            ParamsV.SinZ := to_signed(S32_MIN, 32);
            ParamsV.CosX := to_signed(S32_MIN, 32);
          end if;
          check_case(ParamsV, "extreme values, quadrant " & to_string(q));
          -- same with the opposite DeltaECS
          for k in 0 to 12 loop
            if ParamsV.CoeffX(k) = to_signed(S32_MAX, 32) then
              ParamsV.CoeffX(k) := to_signed(S32_MIN, 32);
            else
              ParamsV.CoeffX(k) := to_signed(S32_MAX, 32);
            end if;
          end loop;
          check_case(ParamsV, "extreme values, opposite DeltaECSX, quadrant " & to_string(q));
        end loop;

      -----------------------------------------------------------------------
      -- random parameters over the whole input space
      elsif run("random_parameters") then
        for i in 1 to N_RANDOM loop
          ParamsV := rand_params;
          check_case(ParamsV, "random case " & to_string(i));
        end loop;
        info("max error " & to_string(MaxErrV, 3) & " LSB, max latency " & to_string(MaxLatV) & " cycles");

      -----------------------------------------------------------------------
      -- each ECS is processed only once its period is valid
      elsif run("wait_for_period_valid") then
        ParamsV          := rand_params;
        PeriodEcsValidxS <= (others => '0');
        start(ParamsV);
        for n in 1 to 4 loop
          wait until ValidxS = '1' for 3 * MAX_LATENCY * CLK_PERIOD;
          check_equal(ValidxS, '0', "ValidxSO asserted before period of ECS" & to_string(n) & " valid");
          PeriodEcsValidxS(n) <= '1';
          StartTV             := now;
        end loop;
        wait_valid(AngleV, LatencyV);
        check_model(ParamsV, AngleV, "periods valid one after the other");
        acknowledge;
        -- a later valid of ECS1 alone must not be enough
        PeriodEcsValidxS <= "0111";
        ParamsV          := rand_params;
        start(ParamsV);
        wait until ValidxS = '1' for 3 * MAX_LATENCY * CLK_PERIOD;
        check_equal(ValidxS, '0', "ValidxSO asserted while period of ECS1 not valid");
        PeriodEcsValidxS <= "1111";
        StartTV          := now;
        wait_valid(AngleV, LatencyV);
        check_model(ParamsV, AngleV, "ECS1 valid last");
        acknowledge;

      -----------------------------------------------------------------------
      -- ValidxSO / AnglexDO held until AckxSI, single computation per start
      elsif run("valid_held_until_ack") then
        ParamsV := rand_params;
        start(ParamsV);
        wait_valid(AngleV, LatencyV);
        check_model(ParamsV, AngleV, "first result");
        -- new inputs must not affect the held result
        apply(rand_params);
        wait until ValidxS = '0' for 2000 * CLK_PERIOD;
        check_equal(ValidxS, '1', "ValidxSO released without AckxSI");
        check_equal(AnglexD, AngleV, "AnglexDO changed before AckxSI");
        acknowledge;
        wait until ValidxS = '1' for 3 * MAX_LATENCY * CLK_PERIOD;
        check_equal(ValidxS, '0', "new result without new StartxSI");

      -----------------------------------------------------------------------
      -- StartxSI is level sensitive in idle: held high, it relaunches a
      -- computation after the acknowledge
      elsif run("start_held_high") then
        ParamsV := vector_params(R, 0);
        apply(ParamsV);
        wait until rising_edge(ClkxC);
        StartxS <= '1';
        StartTV := now;
        wait_valid(AngleV, LatencyV);
        check_equal(AngleV, deg(90.0), "first computation");
        apply(vector_params(0, -R));
        acknowledge;
        StartTV := now;
        wait_valid(AngleV, LatencyV);
        check_equal(AngleV, deg(180.0), "relaunched computation");
        StartxS <= '0';
        acknowledge;
        wait until ValidxS = '1' for 3 * MAX_LATENCY * CLK_PERIOD;
        check_equal(ValidxS, '0', "new result after StartxSI released");

      -----------------------------------------------------------------------
      -- Num = Den = 0 has no defined angle but must not lock the module
      elsif run("null_vector") then
        compute(default_params, AngleV);
        info("angle of the null vector: " & to_string(to_integer(AngleV)));
        check_exact(vector_params(-R, R), deg(315.0), "computation after null vector");

      -----------------------------------------------------------------------
      -- computation time with all periods valid
      elsif run("latency") then
        for i in 1 to 20 loop
          check_case(rand_params, "latency case " & to_string(i));
        end loop;
        check_case(default_params, "null vector");          -- longest normalization
        check_case(vector_params(1, 1), "smallest vector");
        info("max latency " & to_string(MaxLatV) & " cycles (" & to_string(real(MaxLatV) * 0.01, 2) & " us at 100 MHz)");
        check(MaxLatV <= MAX_LATENCY, "latency " & to_string(MaxLatV) & " > " & to_string(MAX_LATENCY));
      end if;
    end loop;

    test_runner_cleanup(runner);
  end process main;

end architecture tb;

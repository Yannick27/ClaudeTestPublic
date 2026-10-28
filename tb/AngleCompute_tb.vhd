-------------------------------------------------------------------------------
--! @file      AngleCompute_tb.vhd
--! @brief     VUnit testbench of AngleCompute (plate angle computation)
--! @standard  VHDL'08
-------------------------------------------------------------------------------
--! Description:
--!   Self-checking VUnit testbench of AngleCompute. Random stimuli are built
--!   with osvvm.RandomPkg.
--!
--!   Reference model
--!     * Normalization / Chebyshev linearization: bit-exact model of the
--!       documented fixed-point scheme (X in Q2.36, Clenshaw recurrence with
--!       round-half-up of 2*X*b(k+1) and X*b(1)). Each fixed-point result is
--!       also compared with the Chebyshev series sum_k Ck*Tk(x) computed in
--!       real arithmetic (LIN_TOL), which validates the scheme itself.
--!     * DeltaECSX/Z, Num and Den: exact integer arithmetic (wide signed).
--!     * Angle: atan2(Num, Den) in real arithmetic, mapped to [0, 2*pi)
--!       (65536 = 2*pi). The DUT (28-bit CORDIC input, 18 iterations,
--!       rounding to 16 bits) must be within ANGLE_TOL_LSB on the circle.
--!
--!   Protocol checks (every transaction)
--!     * StartxSI / PeriodEcsValidxSI sequence: all at 0 at startup, 1-cycle
--!       StartxSI pulse, then PeriodEcsValidxSI(1), (2), (3), (4) rise one
--!       after the other after random delays. Periods of the ECSs whose
--!       valid is still low carry random garbage.
--!     * ValidxSO stays low until the computation is finished.
--!     * ValidxSO and AnglexDO are held until AckxSI, ValidxSO is cleared
--!       after AckxSI.
--!
--!   Normalization parameters: GainNormxDI < 0 and OffsetNormxDI > 0, the
--!   lowest period of an ECS range gives +1 and the highest one -1.
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

library osvvm;
use osvvm.RandomPkg.all;

use work.BamPkg.all;

entity AngleCompute_tb is
  generic(
    runner_cfg : string;
    G_SEED     : natural := 1           -- random seed (combined with test name)
  );
end entity AngleCompute_tb;

architecture tb of AngleCompute_tb is

  constant CLK_PERIOD : time := 10 ns;  -- 100 MHz

  ---------------------------------------------------------------------------
  -- DUT internal formats (from the AngleCompute / PeriodNormLin documentation)
  ---------------------------------------------------------------------------
  constant ORDER : positive := 6;
  constant X_W   : positive := 38;      -- normalized period, Q2.36
  constant WB    : positive := 32 + clog2(ORDER * (ORDER + 1) + 1); -- Clenshaw word
  constant LW    : positive := LinWidth(ORDER); -- linearized period
  constant WD    : positive := 32 + clog2(2 * ORDER + 2); -- DeltaECS
  constant WN    : positive := WD + 32 + 1; -- Num / Den

  ---------------------------------------------------------------------------
  -- Verification parameters
  ---------------------------------------------------------------------------
  constant PERIOD_MIN     : natural := 1_000_000; -- ECS period range
  constant PERIOD_MAX     : natural := 2_000_000;
  constant ANGLE_TOL_LSB  : real    := 1.0; -- angle tolerance (16-bit LSB)
  constant LIN_TOL        : real    := 8.0;  -- fixed-point vs real Chebyshev series (coeff LSB)
  constant X_TOL          : real    := 1.0e-4; -- |x| <= 1 + X_TOL after normalization
  constant TIMEOUT_CYCLES : natural := 10_000; -- last valid -> ValidxSO

  constant S32_MAX : signed(31 downto 0) := (31 => '0', others => '1');
  constant S32_MIN : signed(31 downto 0) := (31 => '1', others => '0');

  ---------------------------------------------------------------------------
  -- Types
  ---------------------------------------------------------------------------
  subtype Coeff13_t  is ChebyshevCoeff_t(0 to 12);
  subtype Coeff6_t   is ChebyshevCoeff_t(0 to ORDER - 1);
  subtype Period4_t  is EcsPeriod_t(1 to 4);
  subtype Param4_t   is EcsParam_t(1 to 4);
  type    LinArray_t is array (1 to 4) of signed(LW - 1 downto 0);

  -- one computation: all DUT inputs
  type Stim_t is record
    Period : Period4_t;
    Gain   : Param4_t;
    Offset : Param4_t;
    CoeffX : Coeff13_t;
    CoeffZ : Coeff13_t;
    CosX   : signed(31 downto 0);
    SinX   : signed(31 downto 0);
    CosZ   : signed(31 downto 0);
    SinZ   : signed(31 downto 0);
  end record Stim_t;

  -- handshake timing of one computation
  type Timing_t is record
    ValidDelay : integer_vector(1 to 4); -- clock cycles from StartxSI / previous valid
    AckDelay   : natural;               -- clock cycles from ValidxSO to AckxSI
  end record Timing_t;

  -- expected result
  type Ref_t is record
    X       : real_vector(1 to 4);      -- normalized periods
    LinErr  : real;                     -- max |fixed-point - real| linearization
    Num     : signed(WN - 1 downto 0);
    Den     : signed(WN - 1 downto 0);
    Defined : boolean;                  -- false if Num = Den = 0
    Angle   : real;                     -- expected angle in [0, 65536)
  end record Ref_t;

  ---------------------------------------------------------------------------
  -- Reference model
  ---------------------------------------------------------------------------
  -- signed -> real
  function ToReal(v : signed) return real is
    variable a : signed(v'length downto 0) := abs (resize(v, v'length + 1));
    variable r : real                      := 0.0;
  begin
    for i in a'high downto 0 loop
      r := 2.0 * r;
      if a(i) = '1' then
        r := r + 1.0;
      end if;
    end loop;
    if v(v'left) = '1' then
      return -r;
    end if;
    return r;
  end function ToReal;

  -- X = Period * GainNorm + OffsetNorm, Q2.36
  function NormX(P : unsigned(23 downto 0); G, O : signed(31 downto 0)) return signed is
    variable s : signed(57 downto 0);
  begin
    s := resize(signed('0' & P) * G, 58) + shift_left(resize(O, 58), 12);
    return resize(s, X_W);
  end function NormX;

  -- Chebyshev series, fixed-point Clenshaw recurrence
  --   b(k) = C(k) + 2*X*b(k+1) - b(k+2),  Lin = X*b(1) - b(2)
  function LinFx(X : signed(X_W - 1 downto 0); C : ChebyshevCoeff_t) return signed is
    constant RND_2XB : signed(X_W + WB - 1 downto 0) := shift_left(to_signed(1, X_W + WB), 34);
    constant RND_XB  : signed(X_W + WB - 1 downto 0) := shift_left(to_signed(1, X_W + WB), 35);
    variable b1, b2  : signed(WB - 1 downto 0)       := (others => '0');
    variable bn, t   : signed(WB - 1 downto 0);
  begin
    for k in ORDER downto 1 loop
      t  := resize(shift_right(X * b1 + RND_2XB, 35), WB);
      bn := resize(C(C'low + k - 1), WB) + t - b2;
      b2 := b1;
      b1 := bn;
    end loop;
    t := resize(shift_right(X * b1 + RND_XB, 36), WB);
    return resize(t - b2, LW);
  end function LinFx;

  -- Chebyshev series, real arithmetic: sum_{k=1..ORDER} C(k)*Tk(x)
  function LinReal(x : real; C : ChebyshevCoeff_t) return real is
    variable tkm1 : real := 1.0;        -- T(k-1)
    variable tk   : real := x;          -- T(k)
    variable tkp1 : real;
    variable s    : real := 0.0;
  begin
    for k in 1 to ORDER loop
      s    := s + tk * ToReal(C(C'low + k - 1));
      tkp1 := 2.0 * x * tk - tkm1;
      tkm1 := tk;
      tk   := tkp1;
    end loop;
    return s;
  end function LinReal;

  function ComputeRef(s : Stim_t) return Ref_t is
    variable r      : Ref_t;
    variable c      : Coeff6_t;
    variable xf     : signed(X_W - 1 downto 0);
    variable lin    : LinArray_t;
    variable dx, dz : signed(WD - 1 downto 0);
    variable a      : real;
  begin
    r.LinErr := 0.0;
    for n in 1 to 4 loop
      case n is
        when 1      => c := s.CoeffX(0 to 5);
        when 2      => c := s.CoeffX(6 to 11);
        when 3      => c := s.CoeffZ(0 to 5);
        when others => c := s.CoeffZ(6 to 11);
      end case;
      xf       := NormX(s.Period(n), s.Gain(n), s.Offset(n));
      r.X(n)   := ToReal(xf) / 2.0 ** 36;
      lin(n)   := LinFx(xf, c);
      r.LinErr := maximum(r.LinErr, abs (ToReal(lin(n)) - LinReal(r.X(n), c)));
    end loop;

    dx    := resize(lin(1), WD) - resize(lin(2), WD) + resize(s.CoeffX(12), WD);
    dz    := resize(lin(3), WD) - resize(lin(4), WD) + resize(s.CoeffZ(12), WD);
    r.Num := resize(dx * s.CosZ, WN) - resize(dz * s.SinX, WN);
    r.Den := resize(dx * s.SinZ, WN) + resize(dz * s.CosX, WN);

    r.Defined := r.Num /= 0 or r.Den /= 0;
    r.Angle   := 0.0;
    if r.Defined then
      a := arctan(ToReal(r.Num), ToReal(r.Den)) / MATH_2_PI * 65536.0;
      if a < 0.0 then
        a := a + 65536.0;
      end if;
      r.Angle := a;
    end if;
    return r;
  end function ComputeRef;

  -- signed distance on the circle between DUT angle and expected angle (LSB)
  function AngleErr(dut : unsigned(15 downto 0); ref : real) return real is
    variable d : real := real(to_integer(dut)) - ref;
  begin
    if d >= 32768.0 then
      d := d - 65536.0;
    elsif d < -32768.0 then
      d := d + 65536.0;
    end if;
    return d;
  end function AngleErr;

  ---------------------------------------------------------------------------
  -- DUT signals
  ---------------------------------------------------------------------------
  signal ClkxC              : std_logic                := '0';
  signal StartxS            : std_logic                := '0';
  signal AnglexD            : unsigned(15 downto 0);
  signal ValidxS            : std_logic;
  signal AckxS              : std_logic                := '0';
  signal PeriodEcsxD        : Period4_t                := (others => (others => '0'));
  signal PeriodEcsValidxS   : std_logic_vector(1 to 4) := (others => '0');
  signal GainNormxD         : Param4_t                 := (others => (others => '0'));
  signal OffsetNormxD       : Param4_t                 := (others => (others => '0'));
  signal ChebyshevCoeffXxD  : Coeff13_t                := (others => (others => '0'));
  signal ChebyshevCoeffZxD  : Coeff13_t                := (others => (others => '0'));
  signal CosThetaXxD        : signed(31 downto 0)      := (others => '0');
  signal SinThetaXxD        : signed(31 downto 0)      := (others => '0');
  signal CosThetaZxD        : signed(31 downto 0)      := (others => '0');
  signal SinThetaZxD        : signed(31 downto 0)      := (others => '0');

begin

  ClkxC <= not ClkxC after CLK_PERIOD / 2;

  test_runner_watchdog(runner, 500 ms);

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
  -- Stimuli and checks
  ---------------------------------------------------------------------------
  main : process
    variable RV : RandomPType;

    -- statistics
    variable NbTxn      : natural := 0;
    variable NbChecked  : natural := 0;
    variable MaxAngErr  : real    := 0.0;
    variable MaxLinErr  : real    := 0.0;
    variable MinLatency : natural := natural'high;
    variable MaxLatency : natural := 0;

    -------------------------------------------------------------------------
    -- Random helpers
    -------------------------------------------------------------------------
    -- uniform over the full signed 32-bit range
    impure function Rand32 return signed is
    begin
      return signed(RV.RandSlv(0, 2 ** 16 - 1, 16) & RV.RandSlv(0, 2 ** 16 - 1, 16));
    end function Rand32;

    -- |value| < 2**b, b random in MinBits..MaxBits (31 -> full range)
    impure function RandCoeff(MinBits, MaxBits : natural) return signed is
      constant b : natural := RV.RandInt(MinBits, MaxBits);
    begin
      if b >= 31 then
        return Rand32;
      end if;
      return to_signed(RV.RandInt(-(2 ** b - 1), 2 ** b - 1), 32);
    end function RandCoeff;

    impure function RandExtreme return signed is
    begin
      if RV.RandInt(0, 1) = 0 then
        return S32_MIN;
      end if;
      return S32_MAX;
    end function RandExtreme;

    -- normalization parameters of an ECS whose period lies in [PMin, PMax]:
    -- PMin -> +1, PMax -> -1 (Gain < 0, Offset > 0)
    procedure SetNorm(n : integer; PMin, PMax : natural; s : inout Stim_t) is
      constant d : real := real(PMax - PMin);
    begin
      s.Gain(n)   := to_signed(integer(round(-2.0 / d * 2.0 ** 36)), 32); -- Q-4.36
      s.Offset(n) := to_signed(integer(round(real(PMax + PMin) / d * 2.0 ** 24)), 32); -- Q8.24
    end procedure SetNorm;

    -- random ECS range, normalization and period of ECS n
    procedure RandEcs(n : integer; s : inout Stim_t) is
      variable PMin, PMax : natural;
    begin
      PMin := PERIOD_MIN;
      PMax := PERIOD_MAX;
      if RV.RandInt(0, 3) /= 0 then
        PMin := RV.RandInt(PERIOD_MIN, 1_400_000);
        PMax := RV.RandInt(1_600_000, PERIOD_MAX);
      end if;
      SetNorm(n, PMin, PMax, s);
      case RV.RandInt(0, 7) is
        when 0      => s.Period(n) := to_unsigned(PMin, 24);
        when 1      => s.Period(n) := to_unsigned(PMax, 24);
        when others => s.Period(n) := to_unsigned(RV.RandInt(PMin, PMax), 24);
      end case;
    end procedure RandEcs;

    -- Cos/Sin: true rotation (random angle and scale) or raw random words
    procedure RandRotation(s : inout Stim_t) is
      variable tx, tz, k : real;
    begin
      if RV.RandInt(0, 3) = 0 then
        s.CosX := Rand32;
        s.SinX := Rand32;
        s.CosZ := Rand32;
        s.SinZ := Rand32;
      else
        tx     := RV.RandReal(-MATH_PI, MATH_PI);
        tz     := RV.RandReal(-MATH_PI, MATH_PI);
        k      := RV.RandReal(2.0 ** 16, 2.0 ** 31 - 2.0);
        s.CosX := to_signed(integer(round(k * cos(tx))), 32);
        s.SinX := to_signed(integer(round(k * sin(tx))), 32);
        s.CosZ := to_signed(integer(round(k * cos(tz))), 32);
        s.SinZ := to_signed(integer(round(k * sin(tz))), 32);
      end if;
    end procedure RandRotation;

    -- no rotation: Num = K*DeltaECSX, Den = K*DeltaECSZ
    procedure NoRotation(s : inout Stim_t) is
    begin
      s.CosX := S32_MAX;
      s.SinX := (others => '0');
      s.CosZ := S32_MAX;
      s.SinZ := (others => '0');
    end procedure NoRotation;

    -- all Chebyshev coefficients 0 except the constants:
    -- DeltaECSX = Dx, DeltaECSZ = Dz
    impure function ConstStim(Dx, Dz : signed(31 downto 0)) return Stim_t is
      variable s : Stim_t;
    begin
      for n in 1 to 4 loop
        RandEcs(n, s);
      end loop;
      s.CoeffX     := (others => (others => '0'));
      s.CoeffZ     := (others => (others => '0'));
      s.CoeffX(12) := Dx;
      s.CoeffZ(12) := Dz;
      NoRotation(s);
      return s;
    end function ConstStim;

    -- fully random computation
    impure function RandStim(CoeffMinBits, CoeffMaxBits : natural) return Stim_t is
      variable s : Stim_t;
    begin
      for n in 1 to 4 loop
        RandEcs(n, s);
      end loop;
      for i in 0 to 12 loop
        s.CoeffX(i) := RandCoeff(CoeffMinBits, CoeffMaxBits);
        s.CoeffZ(i) := RandCoeff(CoeffMinBits, CoeffMaxBits);
      end loop;
      RandRotation(s);
      return s;
    end function RandStim;

    impure function RandTiming(MaxValidDelay, MaxAckDelay : positive) return Timing_t is
      variable t : Timing_t;
    begin
      for n in 1 to 4 loop
        if RV.RandInt(0, 3) = 0 then
          t.ValidDelay(n) := RV.RandInt(1, 3); -- valid while previous ECS is still linearized
        else
          t.ValidDelay(n) := RV.RandInt(1, MaxValidDelay);
        end if;
      end loop;
      t.AckDelay := RV.RandInt(0, MaxAckDelay);
      return t;
    end function RandTiming;

    -------------------------------------------------------------------------
    -- Bus functional procedures
    -------------------------------------------------------------------------
    -- wait n clock cycles, no result may be signalled meanwhile
    procedure WaitNoValid(n : natural) is
    begin
      for i in 1 to n loop
        wait until rising_edge(ClkxC);
        check(ValidxS = '0', "ValidxSO asserted while no computation is finished");
      end loop;
    end procedure WaitNoValid;

    -- one complete computation: start, valids 1..4, result check, ack
    procedure RunTxn(s : Stim_t; t : Timing_t) is
      variable r       : Ref_t;
      variable err     : real;
      variable angle   : unsigned(15 downto 0);
      variable tValid4 : time;
      variable latency : natural;
    begin
      r := ComputeRef(s);
      for n in 1 to 4 loop
        check(abs (r.X(n)) <= 1.0 + X_TOL,
              "Stimulus error: normalized period of ECS" & integer'image(n) & " = " & real'image(r.X(n)));
      end loop;
      check(r.LinErr <= LIN_TOL,
            "Fixed-point linearization differs from Chebyshev series by " & real'image(r.LinErr));
      MaxLinErr := maximum(MaxLinErr, r.LinErr);

      -- parameters; periods not yet valid carry garbage
      GainNormxD        <= s.Gain;
      OffsetNormxD      <= s.Offset;
      ChebyshevCoeffXxD <= s.CoeffX;
      ChebyshevCoeffZxD <= s.CoeffZ;
      CosThetaXxD       <= s.CosX;
      SinThetaXxD       <= s.SinX;
      CosThetaZxD       <= s.CosZ;
      SinThetaZxD       <= s.SinZ;
      for n in 1 to 4 loop
        PeriodEcsxD(n) <= RV.RandUnsigned(0, 2 ** 24 - 1, 24);
      end loop;
      PeriodEcsValidxS  <= (others => '0');
      WaitNoValid(RV.RandInt(2, 10));

      -- start pulse
      StartxS <= '1';
      wait until rising_edge(ClkxC);
      StartxS <= '0';

      -- periods become valid one after the other
      for n in 1 to 4 loop
        WaitNoValid(t.ValidDelay(n) - 1);
        PeriodEcsxD(n)      <= s.Period(n);
        PeriodEcsValidxS(n) <= '1';
        wait until rising_edge(ClkxC);  -- valid sampled by the DUT
        check(ValidxS = '0', "ValidxSO asserted while no computation is finished");
      end loop;
      tValid4 := now;

      -- result
      if ValidxS /= '1' then
        wait until ValidxS = '1' for TIMEOUT_CYCLES * CLK_PERIOD;
      end if;
      check_equal(ValidxS, '1', "Timeout waiting for ValidxSO");
      latency    := (now - tValid4) / CLK_PERIOD;
      MinLatency := minimum(MinLatency, latency);
      MaxLatency := maximum(MaxLatency, latency);

      angle := AnglexD;
      NbTxn := NbTxn + 1;
      if r.Defined then
        err       := AngleErr(angle, r.Angle);
        NbChecked := NbChecked + 1;
        MaxAngErr := maximum(MaxAngErr, abs err);
        check(abs err <= ANGLE_TOL_LSB,
              "Angle mismatch: AnglexDO = " & integer'image(to_integer(angle)) &
              ", expected = " & real'image(r.Angle) & ", error = " & real'image(err) & " LSB" &
              ", Num = " & to_hstring(r.Num) & ", Den = " & to_hstring(r.Den));
      else
        info("Num = Den = 0: angle undefined, not checked");
      end if;

      -- result is held until acknowledged
      for i in 1 to t.AckDelay loop
        wait until rising_edge(ClkxC);
        check(ValidxS = '1', "ValidxSO dropped before AckxSI");
        check(AnglexD = angle, "AnglexDO changed while ValidxSO is high");
      end loop;
      AckxS <= '1';
      wait until rising_edge(ClkxC);
      AckxS <= '0';
      wait until rising_edge(ClkxC);
      wait until rising_edge(ClkxC);
      check(ValidxS = '0', "ValidxSO not cleared after AckxSI");

      -- measurement cycle over: valids back to 0
      PeriodEcsValidxS <= (others => '0');
      WaitNoValid(RV.RandInt(1, 5));
    end procedure RunTxn;

    -------------------------------------------------------------------------
    variable s     : Stim_t;
    variable dx    : signed(31 downto 0);
    variable dz    : signed(31 downto 0);
    variable alpha : real;
    variable rad   : real;
    type IntArray_t is array (natural range <>) of integer;
    constant AXIS_DX : IntArray_t(0 to 7) := (0, 1, 1, 1, 0, -1, -1, -1);
    constant AXIS_DZ : IntArray_t(0 to 7) := (1, 1, 0, -1, -1, -1, 0, 1);
    constant RADIUS  : IntArray_t(0 to 5) := (1, 2, 1000, 2 ** 20, 2 ** 30, integer'high);

  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop
      RV.InitSeed(running_test_case & integer'image(G_SEED));

      -- all inputs at 0 at startup
      StartxS          <= '0';
      AckxS            <= '0';
      PeriodEcsValidxS <= (others => '0');
      WaitNoValid(10);

      if run("axis_and_diagonal_angles") then
        -- DeltaECS on the axes and diagonals (multiples of 45 deg), from
        -- tiny to full scale magnitudes (quadrant pre-rotation, Num/Den
        -- normalization)
        for r in RADIUS'range loop
          for d in AXIS_DX'range loop
            dx := to_signed(AXIS_DX(d) * RADIUS(r), 32);
            dz := to_signed(AXIS_DZ(d) * RADIUS(r), 32);
            RunTxn(ConstStim(dx, dz), RandTiming(200, 10));
          end loop;
        end loop;
        -- most negative values
        RunTxn(ConstStim(S32_MIN, (others => '0')), RandTiming(200, 10));
        RunTxn(ConstStim((others => '0'), S32_MIN), RandTiming(200, 10));
        RunTxn(ConstStim(S32_MIN, S32_MIN), RandTiming(200, 10));
        RunTxn(ConstStim(S32_MIN, S32_MAX), RandTiming(200, 10));
        RunTxn(ConstStim(S32_MAX, S32_MIN), RandTiming(200, 10));

      elsif run("full_circle_sweep") then
        -- DeltaECSX = R*sin(alpha), DeltaECSZ = R*cos(alpha), no rotation
        -- -> AnglexDO = alpha, for 1024 angles on the full circle
        for k in 0 to 1023 loop
          alpha := MATH_2_PI * (real(k) + RV.RandReal(-0.5, 0.5)) / 1024.0;
          rad   := 2.0 ** RV.RandReal(8.0, 31.0) - 1.0;
          dx    := to_signed(integer(round(rad * sin(alpha))), 32);
          dz    := to_signed(integer(round(rad * cos(alpha))), 32);
          RunTxn(ConstStim(dx, dz), RandTiming(50, 5));
        end loop;

      elsif run("rotation_terms") then
        -- linearization disabled, random DeltaECS and random Cos/Sin
        for i in 1 to 300 loop
          s := ConstStim(RandCoeff(0, 31), RandCoeff(0, 31));
          RandRotation(s);
          RunTxn(s, RandTiming(50, 5));
        end loop;

      elsif run("random_full_datapath") then
        -- random periods, normalization, Chebyshev coefficients, Cos/Sin
        for i in 1 to 500 loop
          RunTxn(RandStim(0, 31), RandTiming(300, 20));
        end loop;

      elsif run("realistic_coefficients") then
        -- large coefficients: linearization dominates the constants
        for i in 1 to 200 loop
          s := RandStim(24, 31);
          for n in 0 to 1 loop
            s.CoeffX(6 * n) := RandCoeff(29, 31); -- dominant first-order term
            s.CoeffZ(6 * n) := RandCoeff(29, 31);
          end loop;
          RunTxn(s, RandTiming(300, 20));
        end loop;

      elsif run("extreme_values") then
        -- full scale coefficients, periods at both ends of [1e6, 2e6]
        -- (x = +-1), full scale Cos/Sin: no internal overflow allowed
        for i in 1 to 200 loop
          for n in 1 to 4 loop
            SetNorm(n, PERIOD_MIN, PERIOD_MAX, s);
            if RV.RandInt(0, 1) = 0 then
              s.Period(n) := to_unsigned(PERIOD_MIN, 24);
            else
              s.Period(n) := to_unsigned(PERIOD_MAX, 24);
            end if;
          end loop;
          for k in 0 to 12 loop
            s.CoeffX(k) := RandExtreme;
            s.CoeffZ(k) := RandExtreme;
          end loop;
          s.CosX := RandExtreme;
          s.SinX := RandExtreme;
          s.CosZ := RandExtreme;
          s.SinZ := RandExtreme;
          RunTxn(s, RandTiming(50, 5));
        end loop;

      elsif run("slow_valids_and_ack") then
        -- long gaps between the period valids and long acknowledge delay
        for i in 1 to 20 loop
          RunTxn(RandStim(16, 31), RandTiming(5000, 500));
        end loop;
      end if;

      info("Computations: " & integer'image(NbTxn) & " (" & integer'image(NbChecked) & " angles checked)" &
           ", max angle error = " & real'image(MaxAngErr) & " LSB" &
           ", max linearization error = " & real'image(MaxLinErr) &
           ", latency after last valid = " & integer'image(MinLatency) & ".." & integer'image(MaxLatency) & " cycles");
    end loop;

    test_runner_cleanup(runner);
  end process main;

end architecture tb;

-------------------------------------------------------------------------------
-- VUnit testbench for AngleCompute (VHDL-2008, OSVVM RandomPkg)
--
--   Run with   python3 run.py            (see run.py at the repository root)
--
--   Golden model : double precision (math_real) normalization, Chebyshev
--                  series and atan2.
--   Pass criterion: circular angle error <= 1 LSB (2*pi/65536).
--
--   Stimulus rules (all tests):
--     * StartxSI and PeriodEcsValidxSI are all 0 at startup.
--     * One 1-clock-cycle pulse on StartxSI, then, after random delays,
--       PeriodEcsValidxSI(1), (2), (3), (4) go to 1 in this order.
--       The valid bits are cleared again before the next measurement.
--     * GainNormxDI < 0 and OffsetNormxDI > 0 : the lowest period (1000000)
--       is normalized to +1 and the highest (2000000) to -1.
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

entity tb_AngleCompute is
  generic(runner_cfg : string;
          ORDER      : positive := 6);
end entity tb_AngleCompute;

architecture sim of tb_AngleCompute is

  constant TCLK     : time    := 10 ns;
  constant P_MIN    : integer := 1000000;   -- period giving x = +1
  constant P_MAX    : integer := 2000000;   -- period giving x = -1
  constant MAX_GAP  : positive := 150;      -- max clock cycles between 2 valids
  constant TOL_LSB  : real    := 1.0;       -- max circular angle error

  signal Clk       : std_logic := '0';
  signal Start     : std_logic := '0';
  signal Valid     : std_logic;
  signal Angle     : unsigned(15 downto 0);
  signal Period    : EcsPeriod_t(1 to 4)                       := (others => to_unsigned(P_MIN, 24));
  signal PerValid  : std_logic_vector(1 to 4)                  := (others => '0');
  signal Gain      : EcsGain_t(1 to 4)                         := (others => (others => '0'));
  signal Offset    : EcsOffset_t(1 to 4)                       := (others => (others => '0'));
  signal Coeff     : ChebyshevCoeffArray_t(1 to 4)(1 to ORDER) := (others => (others => (others => '0')));
  signal GammaX    : signed(31 downto 0) := (others => '0');
  signal GammaZ    : signed(31 downto 0) := (others => '0');
  signal CosThX    : signed(31 downto 0) := to_signed(2 ** 30, 32);
  signal SinThX    : signed(31 downto 0) := (others => '0');
  signal CosThZ    : signed(31 downto 0) := to_signed(2 ** 30, 32);
  signal SinThZ    : signed(31 downto 0) := (others => '0');

  signal ValidCnt  : natural := 0;

  type RealArr_t is array (1 to 4) of real;

begin

  Clk <= not Clk after TCLK / 2;

  test_runner_watchdog(runner, 50 ms);

  dut : entity work.AngleCompute
    generic map(ORDER => ORDER)
    port map(ClkxCI            => Clk,
             StartxSI          => Start,
             ValidxSO          => Valid,
             AnglexDO          => Angle,
             PeriodEcsxDI      => Period,
             PeriodEcsValidxSI => PerValid,
             GainNormxDI       => Gain,
             OffsetNormxDI     => Offset,
             ChebyshevCoeffxDI => Coeff,
             GammaXxDI         => GammaX,
             GammaZxDI         => GammaZ,
             CosThetaXxDI      => CosThX,
             SinThetaXxDI      => SinThX,
             CosThetaZxDI      => CosThZ,
             SinThetaZxDI      => SinThZ);

  ---------------------------------------------------------------------------
  -- Protocol monitor on ValidxSO
  ---------------------------------------------------------------------------
  monitor : process(Clk)
    variable Prev : std_logic := '0';
  begin
    if rising_edge(Clk) then
      check_false(Valid = '1' and Prev = '1', "ValidxSO must be a 1-clock-cycle pulse");
      if Valid = '1' then
        check_equal(PerValid(4), '1', "ValidxSO raised before PeriodEcsValidxSI(4)");
        ValidCnt <= ValidCnt + 1;
      end if;
      Prev := Valid;
    end if;
  end process;

  ---------------------------------------------------------------------------
  -- Test sequencer
  ---------------------------------------------------------------------------
  main : process
    variable RV      : RandomPType;
    variable NMeas   : natural := 0;
    variable MaxErr  : real    := 0.0;

    ---------------------------------------------------------------------
    -- Golden model
    ---------------------------------------------------------------------
    -- Chebyshev series sum_{k=1..ORDER} Tk(x)*c(k)  (direct recurrence)
    function Cheb(x : real; c : ChebyshevCoeff_t) return real is
      variable Tkm1, Tk, Tkp1, Acc : real;
    begin
      Tkm1 := 1.0;
      Tk   := x;
      Acc  := 0.0;
      for k in 1 to ORDER loop
        Acc  := Acc + Tk * real(to_integer(c(k)));
        Tkp1 := 2.0 * x * Tk - Tkm1;
        Tkm1 := Tk;
        Tk   := Tkp1;
      end loop;
      return Acc;
    end function;

    function NormX(p : unsigned(23 downto 0); g, o : signed(31 downto 0)) return real is
    begin
      return real(to_integer(p)) * real(to_integer(g)) / 2.0 ** 36 + real(to_integer(o)) / 2.0 ** 24;
    end function;

    -- expected angle in LSB (0 <= angle < 65536), Defined = false if Num = Den = 0
    procedure ExpectedAngle(variable Expected : out real; variable Defined : out boolean) is
      variable L : RealArr_t;
      variable DX, DZ, Num, Den, Ang : real;
    begin
      for n in 1 to 4 loop
        L(n) := Cheb(NormX(Period(n), Gain(n), Offset(n)), Coeff(n));
        assert abs(NormX(Period(n), Gain(n), Offset(n))) <= 1.0
          report "testbench error: normalized period outside [-1, 1]" severity failure;
      end loop;
      DX  := L(1) - L(2) + real(to_integer(GammaX));
      DZ  := L(3) - L(4) + real(to_integer(GammaZ));
      Num := real(to_integer(CosThZ)) * DX - real(to_integer(SinThX)) * DZ;
      Den := real(to_integer(SinThZ)) * DX + real(to_integer(CosThX)) * DZ;
      Defined := not (Num = 0.0 and Den = 0.0);
      if Defined then
        Ang := arctan(Num, Den);
      else
        Ang := 0.0;
      end if;
      Expected := Ang / MATH_2_PI * 65536.0;
      if Expected < 0.0 then
        Expected := Expected + 65536.0;
      end if;
    end procedure;

    ---------------------------------------------------------------------
    -- Stimulus
    ---------------------------------------------------------------------
    procedure WaitClocks(n : natural) is
    begin
      for i in 1 to n loop
        wait until rising_edge(Clk);
      end loop;
    end procedure;

    -- Gain < 0 / Offset > 0 : P_MIN -> +1, P_MAX -> -1 (rounded inwards, so
    -- |x| never exceeds 1)
    procedure SetNormalization is
      variable GainAbs : integer;
      variable Off     : integer;
    begin
      GainAbs := integer(floor(2.0 * 2.0 ** 36 / real(P_MAX - P_MIN)));          -- Q-4.36
      Off     := integer(round(real(GainAbs) / 2.0 ** 36 * real(P_MIN + P_MAX) / 2.0 * 2.0 ** 24));  -- Q8.24
      for n in 1 to 4 loop
        Gain(n)   <= to_signed(-GainAbs, 32);
        Offset(n) <= to_signed(Off, 32);
      end loop;
    end procedure;

    procedure SetPeriods(a, b, c, d : integer) is
    begin
      Period <= (to_unsigned(a, 24), to_unsigned(b, 24), to_unsigned(c, 24), to_unsigned(d, 24));
    end procedure;

    procedure RandomPeriods is
    begin
      for n in 1 to 4 loop
        Period(n) <= to_unsigned(RV.RandInt(P_MIN, P_MAX), 24);
      end loop;
    end procedure;

    procedure RandomCoeffs is
    begin
      for n in 1 to 4 loop
        Coeff(n)(1) <= to_signed(2 ** 30 + RV.RandInt(-100000000, 100000000), 32);
        for k in 2 to ORDER loop
          Coeff(n)(k) <= to_signed(RV.RandInt(-50000000, 50000000), 32);
        end loop;
      end loop;
      GammaX <= to_signed(RV.RandInt(-100000000, 100000000), 32);
      GammaZ <= to_signed(RV.RandInt(-100000000, 100000000), 32);
    end procedure;

    -- Cos/Sin of two random angles; Scale = full scale of the 32-bit words
    procedure RandomTrig(Scale : real := 2.0 ** 30) is
      variable AX, AZ : real;
    begin
      AX := RV.RandReal(-MATH_PI, MATH_PI);
      AZ := RV.RandReal(-MATH_PI, MATH_PI);
      CosThX <= to_signed(integer(cos(AX) * Scale), 32);
      SinThX <= to_signed(integer(sin(AX) * Scale), 32);
      CosThZ <= to_signed(integer(cos(AZ) * Scale), 32);
      SinThZ <= to_signed(integer(sin(AZ) * Scale), 32);
    end procedure;

    -- Coefficients = 1.0 on T1 only, Gamma = 0, no rotation:
    --   Num = DeltaECSX, Den = DeltaECSZ
    procedure IdentityParams is
    begin
      for n in 1 to 4 loop
        Coeff(n) <= (1 => to_signed(2 ** 30, 32), others => (others => '0'));
      end loop;
      GammaX <= (others => '0');
      GammaZ <= (others => '0');
      CosThX <= to_signed(2 ** 30, 32);
      SinThX <= (others => '0');
      CosThZ <= to_signed(2 ** 30, 32);
      SinThZ <= (others => '0');
    end procedure;

    -- One measurement : Start pulse, then valid(1..4) with random delays,
    -- then wait for ValidxSO and compare the angle with the golden model.
    procedure RunMeasurement(Tag : string; CheckAngle : boolean := true) is
      variable Expected, Got, Err : real;
      variable Defined            : boolean;
      variable N0                 : natural;
      variable TLastValid         : time;
    begin
      wait for 1 ns;                       -- let the new parameters settle
      ExpectedAngle(Expected, Defined);
      N0 := ValidCnt;

      -- all control inputs at 0, then the Start pulse
      PerValid <= (others => '0');
      Start    <= '0';
      WaitClocks(RV.RandInt(2, 10));
      Start <= '1';
      wait until rising_edge(Clk);
      Start <= '0';

      -- valid of ECS 1, 2, 3, 4 one after the other
      for n in 1 to 4 loop
        WaitClocks(RV.RandInt(1, MAX_GAP));
        PerValid(n) <= '1';
      end loop;
      TLastValid := now;

      wait until Valid = '1' for 5000 * TCLK;
      check_equal(Valid, '1', Tag & ": timeout waiting for ValidxSO");
      check(now > TLastValid, Tag & ": ValidxSO raised before the last period valid");

      if CheckAngle and Defined then
        Got := real(to_integer(Angle));
        Err := abs(Got - Expected);
        if Err > 32768.0 then
          Err := 65536.0 - Err;          -- circular distance
        end if;
        if Err > MaxErr then
          MaxErr := Err;
        end if;
        check(Err <= TOL_LSB, Tag & ": angle " & real'image(Got) & ", expected "
              & real'image(Expected) & " (error " & real'image(Err) & " LSB)");
      end if;

      WaitClocks(20);
      check_equal(ValidCnt, N0 + 1, Tag & ": exactly one ValidxSO pulse per Start");
      NMeas := NMeas + 1;
    end procedure;

  begin
    test_runner_setup(runner, runner_cfg);
    RV.InitSeed(RV'instance_name);

    -- startup state: all control inputs at 0
    check_equal(Valid, '0', "ValidxSO at startup");
    SetNormalization;
    IdentityParams;
    WaitClocks(5);

    while test_suite loop
      if run("directed_quadrants") then
        -- identity parameters: Num = DeltaECSX = x1 - x2, Den = DeltaECSZ = x3 - x4
        -- (a larger period gives a smaller x)
        SetPeriods(1250000, 1750000, 1500000, 1500000);  RunMeasurement("DX>0 DZ=0");
        SetPeriods(1500000, 1500000, 1250000, 1750000);  RunMeasurement("DX=0 DZ>0");
        SetPeriods(1750000, 1250000, 1500000, 1500000);  RunMeasurement("DX<0 DZ=0");
        SetPeriods(1500000, 1500000, 1750000, 1250000);  RunMeasurement("DX=0 DZ<0");
        SetPeriods(1250000, 1750000, 1250000, 1750000);  RunMeasurement("DX>0 DZ>0");
        SetPeriods(1750000, 1250000, 1750000, 1250000);  RunMeasurement("DX<0 DZ<0");
        SetPeriods(1300000, 1700000, 1900000, 1100000);  RunMeasurement("DX>0 DZ<0");
        SetPeriods(1700000, 1300000, 1100000, 1900000);  RunMeasurement("DX<0 DZ>0");
        SetPeriods(P_MIN, P_MAX, P_MIN, P_MAX);          RunMeasurement("full range");

      elsif run("tiny_signal") then
        -- small DeltaECS (a few thousand counts) and small Cos/Sin words:
        -- Num/Den use only a small part of their range, which exercises the
        -- Num/Den normalization before the CORDIC
        SetPeriods(1500000, 1502000, 1500500, 1500000);  RunMeasurement("tiny periods 1");
        SetPeriods(1503000, 1500000, 1500000, 1501000);  RunMeasurement("tiny periods 2");
        for i in 1 to 20 loop
          RandomTrig(2.0 ** RV.RandInt(4, 12));
          SetPeriods(1500000, 1500000 + RV.RandInt(-3000, 3000),
                     1500000, 1500000 + RV.RandInt(-3000, 3000));
          RunMeasurement("tiny trig " & integer'image(i));
        end loop;

      elsif run("zero_signal") then
        -- angle undefined (Num = Den = 0): ValidxSO must still be generated
        SetPeriods(1500000, 1500000, 1500000, 1500000);  RunMeasurement("zero", false);

      elsif run("random_full_circle") then
        for i in 1 to 300 loop
          RandomPeriods;
          RandomCoeffs;
          RandomTrig;
          RunMeasurement("random " & integer'image(i));
        end loop;

      elsif run("extreme_values") then
        -- worst case magnitudes (overflow check): full-scale coefficients and
        -- gammas, x = +1 or -1
        for i in 1 to 100 loop
          RandomTrig;
          for n in 1 to 4 loop
            if RV.RandInt(0, 1) = 0 then
              Period(n) <= to_unsigned(P_MIN, 24);
            else
              Period(n) <= to_unsigned(P_MAX, 24);
            end if;
            for k in 1 to ORDER loop
              if RV.RandInt(0, 1) = 0 then
                Coeff(n)(k) <= signed'(x"80000000");
              else
                Coeff(n)(k) <= signed'(x"7FFFFFFF");
              end if;
            end loop;
          end loop;
          GammaX <= signed'(x"80000000");
          GammaZ <= signed'(x"7FFFFFFF");
          RunMeasurement("extreme " & integer'image(i));
        end loop;
      end if;
    end loop;

    info("measurements: " & integer'image(NMeas) & ", max angle error: " & real'image(MaxErr) & " LSB");
    test_runner_cleanup(runner);
    wait;
  end process;

end architecture sim;

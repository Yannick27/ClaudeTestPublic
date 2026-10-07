-------------------------------------------------------------------------------
-- tb_AngleCompute
--
--   Self-checking testbench of AngleCompute (VHDL-2008, no external files).
--
--   The expected angle is computed in double precision (ieee.math_real) directly from
--   the formulas of the specification: normalisation, Chebyshev recurrence,
--   DeltaECSX/Z, rotation by the sin/cos terms and a full-circle atan2, scaled to
--   65536 = 2*pi.  The DUT result must equal it to within TOL LSB (the DUT rounds to
--   nearest, so the error is 0.5 LSB at most plus a few thousandths of an LSB).
--
--   Tests
--     * directed: zero vector, the four axes and a diagonal (exact expected values),
--       x = +1 / 0 / -1 on every ECS, extreme coefficient / Gamma / sin-cos values;
--     * NRANDOM random sensor set-ups (windows of the period range, decaying Chebyshev
--       coefficients, Gamma, small misalignment angles);
--     * three handshake styles: 1-clock period-valid pulses, period-valid held high,
--       everything valid at once; a second StartxSI pulse while busy (must be
--       ignored); StartxSI held high (the computation re-arms);
--     * protocol: ValidxSO is exactly one clock wide, never early, angle stays stable.
--
--   Run (GHDL):  ghdl -a --std=08 <src>/*.vhd tb_AngleCompute.vhd
--                ghdl -e --std=08 tb_AngleCompute ; ghdl -r --std=08 tb_AngleCompute
--   Other orders: ghdl -r --std=08 tb_AngleCompute -gORDER=1  (any ORDER >= 1)
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.env.all;
use work.AngleComputePkg.all;

entity tb_AngleCompute is
  generic(
    ORDER   : positive := 6;     -- Chebyshev order of the DUT
    NRANDOM : natural  := 300    -- number of random cases
  );
end entity tb_AngleCompute;

architecture sim of tb_AngleCompute is

  constant TCLK : time := 10 ns;                    -- 100 MHz
  constant TOL  : real := 0.55;                     -- allowed error in LSB of the 16-bit angle
  -- one linearisation takes 17*ORDER clocks; the period-valid strobes are spaced further
  -- apart than that (in the application they are about 1e6 clocks apart)
  constant GAP  : natural := 17 * ORDER + 20;

  constant S32_MAX : signed(31 downto 0) := x"7FFFFFFF";   --  2**31 - 1
  constant S32_MIN : signed(31 downto 0) := x"80000000";   -- -2**31

  signal ClkxC             : std_logic := '0';
  signal StartxS           : std_logic := '0';
  signal ValidxS           : std_logic;
  signal AnglexD           : unsigned(15 downto 0);
  signal PeriodEcsxD       : EcsPeriod_t(1 to 4) := (others => (others => '0'));
  signal PeriodEcsValidxS  : std_logic_vector(1 to 4) := (others => '0');
  signal GainNormxD        : EcsParam_t(1 to 4) := (others => (others => '0'));
  signal OffsetNormxD      : EcsParam_t(1 to 4) := (others => (others => '0'));
  signal ChebyshevCoeffxD  : ChebyshevCoeffArray_t(1 to 4)(1 to ORDER) :=
                               (others => (others => (others => '0')));
  signal GammaXxD          : signed(31 downto 0) := (others => '0');
  signal GammaZxD          : signed(31 downto 0) := (others => '0');
  signal CosThetaXxD       : signed(31 downto 0) := (others => '0');
  signal SinThetaXxD       : signed(31 downto 0) := (others => '0');
  signal CosThetaZxD       : signed(31 downto 0) := (others => '0');
  signal SinThetaZxD       : signed(31 downto 0) := (others => '0');

  -- protocol monitor
  signal AllowValid     : boolean := false;
  signal ValidClocks    : natural := 0;
  signal ProtocolErrors : natural := 0;

  function ToReal(v : signed) return real is
  begin
    return real(to_integer(v));
  end function ToReal;

  -- Expected angle in LSB (65536 = 2*pi), in [0, 65536), from the current input signals.
  impure function RefAngle return real is
    type RealArr_t is array (1 to 4) of real;
    variable Lin : RealArr_t;
    variable x, t0, t1, tk, sum, dx, dz, num, den, a : real;
  begin
    for n in 1 to 4 loop
      x := real(to_integer(PeriodEcsxD(n))) * ToReal(GainNormxD(n)) / 2.0**36
           + ToReal(OffsetNormxD(n)) / 2.0**24;
      assert abs(x) <= 1.0
        report "testbench error: normalised period " & integer'image(n) & " out of [-1,1]: " & real'image(x)
        severity failure;
      t0  := 1.0;
      t1  := x;
      sum := x * ToReal(ChebyshevCoeffxD(n)(1));
      for k in 2 to ORDER loop
        tk  := 2.0 * x * t1 - t0;
        sum := sum + tk * ToReal(ChebyshevCoeffxD(n)(k));
        t0  := t1;
        t1  := tk;
      end loop;
      Lin(n) := sum;
    end loop;
    dx  := Lin(1) - Lin(2) + ToReal(GammaXxD);
    dz  := Lin(3) - Lin(4) + ToReal(GammaZxD);
    num := ToReal(CosThetaZxD) * dx - ToReal(SinThetaXxD) * dz;
    den := ToReal(SinThetaZxD) * dx + ToReal(CosThetaXxD) * dz;
    if num = 0.0 and den = 0.0 then
      return 0.0;
    end if;
    a := arctan(num, den) / MATH_2_PI * 65536.0;
    if a < 0.0 then
      a := a + 65536.0;
    end if;
    return a;
  end function RefAngle;

begin

  ClkxC <= not ClkxC after TCLK / 2;

  dut : entity work.AngleCompute
    generic map(ORDER => ORDER)
    port map(
      ClkxCI            => ClkxC,
      StartxSI          => StartxS,
      ValidxSO          => ValidxS,
      AnglexDO          => AnglexD,
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
      SinThetaZxDI      => SinThetaZxD);

  ---------------------------------------------------------------------------
  -- ValidxSO: one clock wide, only after the last period was presented
  ---------------------------------------------------------------------------
  monitor : process(ClkxC)
    variable Prev : std_logic := '0';
  begin
    if rising_edge(ClkxC) then
      if ValidxS = '1' then
        ValidClocks <= ValidClocks + 1;
        if not AllowValid then
          report "ValidxSO asserted before all periods were valid" severity error;
          ProtocolErrors <= ProtocolErrors + 1;
        end if;
        if Prev = '1' then
          report "ValidxSO is wider than one clock" severity error;
          ProtocolErrors <= ProtocolErrors + 1;
        end if;
      end if;
      Prev := ValidxS;
    end if;
  end process monitor;

  ---------------------------------------------------------------------------
  -- stimulus and checks
  ---------------------------------------------------------------------------
  stim : process

    variable Seed1 : positive := 20261007;
    variable Seed2 : positive := 31415926;

    variable NTests  : natural := 0;
    variable NFail   : natural := 0;
    variable MaxErr  : real    := 0.0;
    variable MaxLat  : natural := 0;
    variable Pulses0 : natural;

    impure function Rnd return real is
      variable r : real;
    begin
      uniform(Seed1, Seed2, r);
      return r;
    end function Rnd;

    impure function RndRange(lo, hi : real) return real is
    begin
      return lo + (hi - lo) * Rnd;
    end function RndRange;

    impure function RndInt32 return integer is
    begin
      return integer(floor(Rnd * 4294967296.0 - 2147483648.0));
    end function RndInt32;

    procedure Clocks(n : in natural) is
    begin
      for i in 1 to n loop
        wait until rising_edge(ClkxC);
      end loop;
    end procedure Clocks;

    -- sets Period/Gain/Offset of one ECS so that x = P*G + O is the requested value
    -- (P = 2**20 = 1048576, G = 2**16 -> P*G = 1.0 in Q-4.36)
    procedure SetX(n : in positive range 1 to 4; xv : in real) is
    begin
      PeriodEcsxD(n) <= to_unsigned(2**20, 24);
      GainNormxD(n)  <= to_signed(2**16, 32);
      OffsetNormxD(n) <= to_signed(integer(round((xv - 1.0) * 2.0**24)), 32);
    end procedure SetX;

    procedure SetTrig(thx, thz, scale : in real) is
    begin
      CosThetaXxD <= to_signed(integer(round(cos(thx) * scale)), 32);
      SinThetaXxD <= to_signed(integer(round(sin(thx) * scale)), 32);
      CosThetaZxD <= to_signed(integer(round(cos(thz) * scale)), 32);
      SinThetaZxD <= to_signed(integer(round(sin(thz) * scale)), 32);
    end procedure SetTrig;

    procedure ClearCoeff is
    begin
      ChebyshevCoeffxD <= (others => (others => (others => '0')));
      GammaXxD <= (others => '0');
      GammaZxD <= (others => '0');
    end procedure ClearCoeff;

    -- Runs one measurement.  mode 0: StartxSI pulse then 1-clock period-valid pulses
    --                                (plus a second StartxSI pulse while busy)
    --                        mode 1: StartxSI pulse with valid(1), valid levels stay high
    --                        mode 2: everything valid, StartxSI high for 3 clocks
    -- expect >= 0: the angle must be exactly this value as well.
    procedure RunCase(name : in string; mode : in natural; expect : in integer) is
      variable ref, err, d : real;
      variable lat : natural;
      variable ang : unsigned(15 downto 0);
    begin
      wait for 1 ns;                         -- the caller's signal assignments take effect
      ref     := RefAngle;
      Pulses0 := ValidClocks;
      AllowValid <= false;
      wait until rising_edge(ClkxC);
      if mode = 0 then
        StartxS <= '1';
        wait until rising_edge(ClkxC);
        StartxS <= '0';
        for ch in 1 to 4 loop
          Clocks(GAP);
          PeriodEcsValidxS(ch) <= '1';
          wait until rising_edge(ClkxC);
          PeriodEcsValidxS(ch) <= '0';
          if ch = 2 then                      -- a start while busy must be ignored
            StartxS <= '1';
            wait until rising_edge(ClkxC);
            StartxS <= '0';
          end if;
        end loop;
      elsif mode = 1 then
        PeriodEcsValidxS(1) <= '1';
        StartxS <= '1';
        wait until rising_edge(ClkxC);
        StartxS <= '0';
        for ch in 2 to 4 loop
          Clocks(GAP);
          PeriodEcsValidxS(ch) <= '1';
        end loop;
        wait until rising_edge(ClkxC);
      else
        AllowValid <= true;
        PeriodEcsValidxS <= (others => '1');
        StartxS <= '1';
        Clocks(3);
        StartxS <= '0';
      end if;
      AllowValid <= true;
      -- wait for ValidxSO (sampled like the DUT does: the value during the previous clock)
      lat := 0;
      loop
        wait until rising_edge(ClkxC);
        lat := lat + 1;
        exit when ValidxS = '1';
        assert lat < 20000 report name & ": timeout waiting for ValidxSO" severity failure;
      end loop;
      ang := AnglexD;
      PeriodEcsValidxS <= (others => '0');
      if mode < 2 and lat > MaxLat then      -- (mode 2 includes all four linearisations)
        MaxLat := lat;
      end if;
      -- error in LSB, wrapped to (-32768, 32768]
      d := real(to_integer(ang)) - ref;
      if d > 32768.0 then
        d := d - 65536.0;
      elsif d < -32768.0 then
        d := d + 65536.0;
      end if;
      err := abs(d);
      if err > MaxErr then
        MaxErr := err;
      end if;
      NTests := NTests + 1;
      if err > TOL or (expect >= 0 and to_integer(ang) /= expect) then
        NFail := NFail + 1;
        report name & ": angle " & integer'image(to_integer(ang)) & ", expected " & real'image(ref)
               & " (error " & real'image(d) & " LSB)" severity error;
      end if;
      -- angle stable, exactly one ValidxSO clock, nothing else happens afterwards
      Clocks(30);
      if AnglexD /= ang then
        report name & ": AnglexDO changed after ValidxSO" severity error;
        NFail := NFail + 1;
      end if;
      if ValidClocks /= Pulses0 + 1 then
        report name & ": expected exactly one ValidxSO clock, got " & integer'image(ValidClocks - Pulses0) severity error;
        NFail := NFail + 1;
      end if;
    end procedure RunCase;

    -- StartxSI held high: the computation re-arms after ValidxSO
    procedure RunHeldStart(name : in string) is
      variable ref, d : real;
      variable lat : natural;
      variable a1, a2 : unsigned(15 downto 0);
    begin
      wait for 1 ns;                         -- the caller's signal assignments take effect
      ref := RefAngle;
      Pulses0 := ValidClocks;
      AllowValid <= true;
      wait until rising_edge(ClkxC);
      PeriodEcsValidxS <= (others => '1');
      StartxS <= '1';
      for run in 1 to 2 loop
        lat := 0;
        loop
          wait until rising_edge(ClkxC);
          lat := lat + 1;
          exit when ValidxS = '1';
          assert lat < 20000 report name & ": timeout waiting for ValidxSO" severity failure;
        end loop;
        if run = 1 then
          a1 := AnglexD;
          StartxS <= '0';     -- one more run is already under way (Start was sampled in idle)
        else
          a2 := AnglexD;
        end if;
      end loop;
      PeriodEcsValidxS <= (others => '0');
      d := real(to_integer(a1)) - ref;
      if d > 32768.0 then d := d - 65536.0; elsif d < -32768.0 then d := d + 65536.0; end if;
      NTests := NTests + 1;
      if abs(d) > TOL or a1 /= a2 then
        NFail := NFail + 1;
        report name & ": held StartxSI: angles " & integer'image(to_integer(a1)) & " / "
               & integer'image(to_integer(a2)) & ", expected " & real'image(ref) severity error;
      end if;
      Clocks(30);
      if ValidClocks /= Pulses0 + 2 then
        report name & ": held StartxSI: expected two ValidxSO clocks, got " & integer'image(ValidClocks - Pulses0) severity error;
        NFail := NFail + 1;
      end if;
    end procedure RunHeldStart;

    variable w, plo, phi, scale, cs, thx, thz : real;
    variable xv : real;
    variable e : integer;

  begin
    wait for 10 * TCLK;

    -----------------------------------------------------------------------
    -- 1. zero vector, axes and diagonal.  The ECS 1 / ECS 3 signals are +/- A, the other
    --    ECS are 0; with cos = 1, sin = 0 :  num = DeltaECSX, den = DeltaECSZ.
    -----------------------------------------------------------------------
    SetTrig(0.0, 0.0, 2.0**30);
    for n in 1 to 4 loop
      SetX(n, 1.0);                        -- T1(1) = 1: Lin = c1 for c1 only
    end loop;
    ClearCoeff;
    RunCase("zero vector", 0, 0);

    for k in 0 to 7 loop
      ClearCoeff;
      case k is
        when 0 =>                          -- DX = 0,  DZ = +A  -> 0
          ChebyshevCoeffxD(3)(1) <= to_signed(1000000, 32);
          e := 0;
        when 1 =>                          -- DX = +A, DZ = +A  -> 45 deg
          ChebyshevCoeffxD(1)(1) <= to_signed(1000000, 32);
          ChebyshevCoeffxD(3)(1) <= to_signed(1000000, 32);
          e := 8192;
        when 2 =>                          -- DX = +A, DZ = 0   -> 90 deg
          ChebyshevCoeffxD(1)(1) <= to_signed(1000000, 32);
          e := 16384;
        when 3 =>                          -- DX = +A, DZ = -A  -> 135 deg
          ChebyshevCoeffxD(1)(1) <= to_signed(1000000, 32);
          ChebyshevCoeffxD(4)(1) <= to_signed(1000000, 32);
          e := 24576;
        when 4 =>                          -- DX = 0,  DZ = -A  -> 180 deg
          ChebyshevCoeffxD(4)(1) <= to_signed(1000000, 32);
          e := 32768;
        when 5 =>                          -- DX = -A, DZ = -A  -> 225 deg
          ChebyshevCoeffxD(2)(1) <= to_signed(1000000, 32);
          ChebyshevCoeffxD(4)(1) <= to_signed(1000000, 32);
          e := 40960;
        when 6 =>                          -- DX = -A, DZ = 0   -> 270 deg
          ChebyshevCoeffxD(2)(1) <= to_signed(1000000, 32);
          e := 49152;
        when others =>                     -- DX = -A, DZ = +A  -> 315 deg
          ChebyshevCoeffxD(2)(1) <= to_signed(1000000, 32);
          ChebyshevCoeffxD(3)(1) <= to_signed(1000000, 32);
          e := 57344;
      end case;
      RunCase("axis/diagonal " & integer'image(k), k mod 3, e);
    end loop;

    -- Gamma alone, any sign
    ClearCoeff;
    GammaXxD <= to_signed(-123456789, 32);
    GammaZxD <= to_signed(987654321, 32);
    RunCase("gamma only", 0, -1);

    -----------------------------------------------------------------------
    -- 2. x = +1, 0, -1 and +-0.5 on every ECS, random full-scale coefficients
    -----------------------------------------------------------------------
    SetTrig(0.1, -0.15, 2.0**30);
    for xi in 0 to 6 loop
      case xi is
        when 0 => xv := 1.0;
        when 1 => xv := -1.0;
        when 2 => xv := 0.0;
        when 3 => xv := 0.5;
        when 4 => xv := -0.5;
        when 5 => xv := 0.999999;
        when others => xv := -0.999999;
      end case;
      for n in 1 to 4 loop
        SetX(n, xv * (1.0 - 0.2 * real(n - 1)));
        for k in 1 to ORDER loop
          ChebyshevCoeffxD(n)(k) <= to_signed(RndInt32, 32);
        end loop;
      end loop;
      GammaXxD <= to_signed(RndInt32 / 4, 32);
      GammaZxD <= to_signed(RndInt32 / 4, 32);
      RunCase("x corner " & integer'image(xi), xi mod 3, -1);
    end loop;

    -----------------------------------------------------------------------
    -- 3. extreme coefficients, Gamma and sin/cos values
    -----------------------------------------------------------------------
    for pat in 0 to 5 loop
      for n in 1 to 4 loop
        SetX(n, real(2 * ((n + pat) mod 2) - 1) * 0.7 + 0.05 * real(n));
        for k in 1 to ORDER loop
          case pat is
            when 0 => ChebyshevCoeffxD(n)(k) <= S32_MAX;       -- +max
            when 1 => ChebyshevCoeffxD(n)(k) <= S32_MIN;          -- -2**31
            when 2 =>
              if (k + n) mod 2 = 0 then
                ChebyshevCoeffxD(n)(k) <= S32_MIN;
              else
                ChebyshevCoeffxD(n)(k) <= S32_MAX;
              end if;
            when others => ChebyshevCoeffxD(n)(k) <= to_signed(RndInt32, 32);
          end case;
        end loop;
      end loop;
      case pat is
        when 0 | 3 =>
          GammaXxD <= S32_MAX;
          GammaZxD <= S32_MIN;
        when 1 | 4 =>
          GammaXxD <= S32_MIN;
          GammaZxD <= S32_MAX;
        when others =>
          GammaXxD <= to_signed(RndInt32, 32);
          GammaZxD <= to_signed(RndInt32, 32);
      end case;
      if pat < 3 then
        SetTrig(0.3, -0.2, 2.0**31 - 1.0);
      else
        CosThetaXxD <= S32_MIN;                -- -2**31
        SinThetaXxD <= S32_MAX;
        CosThetaZxD <= to_signed(RndInt32, 32);
        SinThetaZxD <= S32_MIN;
      end if;
      RunCase("extreme " & integer'image(pat), pat mod 3, -1);
    end loop;

    -----------------------------------------------------------------------
    -- 4. StartxSI held high
    -----------------------------------------------------------------------
    SetTrig(0.2, 0.1, 2.0**30);
    for n in 1 to 4 loop
      SetX(n, 0.3 * real(n) - 0.5);
      for k in 1 to ORDER loop
        ChebyshevCoeffxD(n)(k) <= to_signed(integer(round(2.0**28 * RndRange(-1.0, 1.0))), 32);
      end loop;
    end loop;
    RunHeldStart("held start");

    -----------------------------------------------------------------------
    -- 5. random sensor set-ups: windows of the 1e6..2e6 period range mapped to [-1, 1],
    --    decaying Chebyshev coefficients, Gamma, small misalignment
    -----------------------------------------------------------------------
    for i in 1 to NRANDOM loop
      for n in 1 to 4 loop
        w   := RndRange(2.0e5, 1.0e6);
        plo := RndRange(1.0e6, 2.0e6 - w);
        phi := plo + w;
        GainNormxD(n)   <= to_signed(integer(round(2.0 / w * 2.0**36)), 32);
        OffsetNormxD(n) <= to_signed(integer(round(-(phi + plo) / w * 2.0**24)), 32);
        PeriodEcsxD(n)  <= to_unsigned(integer(floor(RndRange(plo + 2000.0, phi - 2000.0))), 24);
        for k in 1 to ORDER loop
          if k = 1 then
            cs := 2.0**30 * RndRange(0.4, 1.0) * real(2 * integer(floor(RndRange(0.0, 2.0))) - 1);
          else
            cs := 2.0**30 * RndRange(-1.0, 1.0) / 3.0**(k - 1);
          end if;
          ChebyshevCoeffxD(n)(k) <= to_signed(integer(round(cs)), 32);
        end loop;
      end loop;
      GammaXxD <= to_signed(integer(round(2.0**30 * RndRange(-0.5, 0.5))), 32);
      GammaZxD <= to_signed(integer(round(2.0**30 * RndRange(-0.5, 0.5))), 32);
      thx   := RndRange(-0.4, 0.4);
      thz   := RndRange(-0.4, 0.4);
      scale := 2.0**30;
      SetTrig(thx, thz, scale);
      RunCase("random " & integer'image(i), i mod 3, -1);
    end loop;

    -----------------------------------------------------------------------
    wait for 10 * TCLK;
    report "AngleCompute ORDER=" & integer'image(ORDER) & ": " & integer'image(NTests) & " measurements, "
           & integer'image(NFail) & " failures, " & integer'image(ProtocolErrors) & " protocol errors; "
           & "max |error| = " & real'image(MaxErr) & " LSB, max latency (valid -> ValidxSO) = "
           & integer'image(MaxLat) & " clocks" severity note;
    if NFail = 0 and ProtocolErrors = 0 then
      report "TEST PASSED" severity note;
    else
      report "TEST FAILED" severity failure;
    end if;
    finish;
    wait;
  end process stim;

end architecture sim;

-------------------------------------------------------------------------------
-- Self-checking system testbench for AngleCompute.
--
-- The expected angle is computed in real arithmetic straight from the
-- specification (normalisation, Chebyshev recurrence, DeltaECS, rotation,
-- atan2) and compared with the DUT, wrap-around aware.  Test groups:
--   1. random, realistic parameter sets (different valid-signal styles)
--   2. worst-case overflow corner: X = +/-1, all coefficients at +/-2**31,
--      Gamma and Cos/Sin at full scale
--   3. Gamma-only vectors (zero coefficients) from huge down to 1 LSB, which
--      exercise the block normalisation in front of the CORDIC
-- It also checks that ValidxSO is a single clock pulse, only appears after the
-- 4th period valid, and that the 4 periods are really processed in sequence.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.env.all;
use work.EcsTypesPkg.all;
use work.AngleComputePkg.all;

entity tb_AngleCompute is
  generic (
    ORDER   : positive := 6;
    NRANDOM : natural  := 200;
    NGAMMA  : natural  := 120;
    SEED    : positive := 20240601;
    GAP     : natural  := 400;    -- clocks between the valid of two ECSs
    MAX_ERR : real     := 0.75    -- allowed error in LSB of 65536 = 2*pi
  );
end entity tb_AngleCompute;

architecture sim of tb_AngleCompute is

  signal Clk    : std_logic := '0';
  signal Start  : std_logic := '0';
  signal Valid  : std_logic;
  signal Angle  : unsigned(15 downto 0);

  signal Per    : EcsPeriod_t(1 to 4) := (others => (others => '0'));
  signal PerVal : std_logic_vector(1 to 4) := (others => '0');
  signal Gn     : EcsGain_t(1 to 4) := (others => (others => '0'));
  signal Ofs    : EcsOffset_t(1 to 4) := (others => (others => '0'));
  signal Cf     : ChebyshevCoeffArray_t(1 to 4)(1 to ORDER) :=
                    (others => (others => (others => '0')));
  signal GamX   : signed(31 downto 0) := (others => '0');
  signal GamZ   : signed(31 downto 0) := (others => '0');
  signal CosX   : signed(31 downto 0) := (others => '0');
  signal SinX   : signed(31 downto 0) := (others => '0');
  signal CosZ   : signed(31 downto 0) := (others => '0');
  signal SinZ   : signed(31 downto 0) := (others => '0');

  -- valid pulse checker
  signal ValidAllowed : std_logic := '0';
  signal PulseCount   : natural := 0;
  signal AngleAtValid : unsigned(15 downto 0) := (others => '0');
  signal ProtocolErrs : natural := 0;

  function AngDiff (a : real; b : real) return real is
    variable d : real := a - b;
  begin
    while d >= 32768.0 loop d := d - 65536.0; end loop;
    while d < -32768.0 loop d := d + 65536.0; end loop;
    return d;
  end function AngDiff;

begin

  Clk <= not Clk after 5 ns;

  dut : entity work.AngleCompute
    generic map (ORDER => ORDER)
    port map (
      ClkxCI            => Clk,
      StartxSI          => Start,
      ValidxSO          => Valid,
      AnglexDO          => Angle,
      PeriodEcsxDI      => Per,
      PeriodEcsValidxSI => PerVal,
      GainNormxDI       => Gn,
      OffsetNormxDI     => Ofs,
      ChebyshevCoeffxDI => Cf,
      GammaXxDI         => GamX,
      GammaZxDI         => GamZ,
      CosThetaXxDI      => CosX,
      SinThetaXxDI      => SinX,
      CosThetaZxDI      => CosZ,
      SinThetaZxDI      => SinZ);

  ---------------------------------------------------------------------------
  -- ValidxSO protocol monitor
  ---------------------------------------------------------------------------
  monitor : process (Clk)
    variable prev : std_logic := '0';
  begin
    if rising_edge(Clk) then
      if Valid = '1' then
        PulseCount   <= PulseCount + 1;
        AngleAtValid <= Angle;
        if ValidAllowed = '0' then
          report "ValidxSO asserted before the 4th period was valid" severity error;
          ProtocolErrs <= ProtocolErrs + 1;
        end if;
        if prev = '1' then
          report "ValidxSO longer than one clock" severity error;
          ProtocolErrs <= ProtocolErrs + 1;
        end if;
      end if;
      prev := Valid;
    end if;
  end process monitor;

  ---------------------------------------------------------------------------
  -- stimulus and checking
  ---------------------------------------------------------------------------
  stim : process
    variable s1, s2   : positive := SEED;
    variable r        : real;
    variable errors   : natural := 0;
    variable ncases   : natural := 0;
    variable worst    : real := 0.0;
    variable minlat   : natural := 100000;
    variable maxlat   : natural := 0;
    variable refang   : real;
    variable refn, refd : real;
    variable refdx, refdz : real;
    variable lat      : natural;
    variable prevcnt  : natural;

    impure function Rnd return real is
    begin
      uniform(s1, s2, r);
      return r;
    end function Rnd;

    function Sgn (v : real) return real is
    begin
      if v < 0.5 then return -1.0; else return 1.0; end if;
    end function Sgn;

    function RoundInt (x : real) return integer is
    begin
      return integer(round(x));
    end function RoundInt;

    function Int (v : signed) return real is
    begin
      return real(to_integer(v));
    end function Int;

    -- sum k=1..ORDER c(k) * Tk(x), Tk by the three-term recurrence
    impure function Cheb (e : integer; x : real) return real is
      variable t0, t1, t2, acc : real;
    begin
      t0  := 1.0;
      t1  := x;
      acc := Int(Cf(e)(1)) * t1;
      for k in 2 to ORDER loop
        t2  := 2.0 * x * t1 - t0;
        acc := acc + Int(Cf(e)(k)) * t2;
        t0  := t1;
        t1  := t2;
      end loop;
      return acc;
    end function Cheb;

    impure function NormPeriod (e : integer) return real is
    begin
      return real(to_integer(Per(e))) * Int(Gn(e)) / 2.0 ** 36 + Int(Ofs(e)) / 2.0 ** 24;
    end function NormPeriod;

    -- reference model of the whole computation
    procedure Reference (variable ang, n, d, dx, dz : out real) is
      variable pl : real_vector(1 to 4);
      variable ddx, ddz, nn, dd, a : real;
    begin
      for e in 1 to 4 loop
        pl(e) := Cheb(e, NormPeriod(e));
      end loop;
      ddx := pl(1) - pl(2) + Int(GamX);
      ddz := pl(3) - pl(4) + Int(GamZ);
      nn  := Int(CosZ) * ddx - Int(SinX) * ddz;
      dd  := Int(SinZ) * ddx + Int(CosX) * ddz;
      a   := arctan(nn, dd) * 65536.0 / MATH_2_PI;
      ang := a; n := nn; d := dd; dx := ddx; dz := ddz;
    end procedure Reference;

    -- start a computation and feed the four valids
    --   style 0: valid levels set one by one, stay high
    --   style 1: valid is a single clock pulse
    --   style 2: all valids already high when start is given
    --   style 3: like 1, but ECS 1 pulse coincides with start
    procedure Run (style : natural; tag : string) is
    begin
      wait until falling_edge(Clk);
      PerVal       <= (others => '0');
      ValidAllowed <= '0';
      if style = 2 then
        PerVal <= (others => '1');
      end if;
      wait until falling_edge(Clk);
      prevcnt := PulseCount;
      Start   <= '1';
      if style = 3 then
        PerVal(1) <= '1';
      end if;
      wait until falling_edge(Clk);
      Start <= '0';
      if style = 3 then
        PerVal(1) <= '0';
      end if;

      for e in 1 to 4 loop
        if style < 2 or (style = 3 and e > 1) then
          for g in 1 to GAP loop
            wait until falling_edge(Clk);
          end loop;
          PerVal(e) <= '1';
          if e = 4 then
            ValidAllowed <= '1';
          end if;
          if style /= 0 then
            wait until falling_edge(Clk);
            PerVal(e) <= '0';
          end if;
        end if;
      end loop;
      if style = 2 then
        ValidAllowed <= '1';
      end if;

      lat := 1;
      while PulseCount = prevcnt and lat < 5000 loop
        wait until falling_edge(Clk);
        lat := lat + 1;
      end loop;
      if PulseCount = prevcnt then
        report tag & ": timeout waiting for ValidxSO" severity error;
        errors := errors + 1;
      else
        if style < 2 then
          if lat < minlat then minlat := lat; end if;
          if lat > maxlat then maxlat := lat; end if;
        end if;
        -- result registered at the valid pulse; let any extra pulse show up
        for g in 1 to 3 loop
          wait until falling_edge(Clk);
        end loop;
        if PulseCount /= prevcnt + 1 then
          report tag & ": more than one ValidxSO pulse" severity error;
          errors := errors + 1;
        end if;
      end if;
      ValidAllowed <= '0';
      PerVal       <= (others => '0');
    end procedure Run;

    procedure Check (tag : string) is
      variable err : real;
    begin
      Reference(refang, refn, refd, refdx, refdz);
      err := AngDiff(real(to_integer(AngleAtValid)), refang);
      ncases := ncases + 1;
      if abs(err) > worst then worst := abs(err); end if;
      if abs(err) > MAX_ERR then
        report tag & ": angle error " & real'image(err) & " LSB, got " &
               integer'image(to_integer(AngleAtValid)) & " expected " & real'image(refang) &
               " (N=" & real'image(refn) & " D=" & real'image(refd) & ")" severity error;
        errors := errors + 1;
      end if;
    end procedure Check;

    ---------------------------------------------------------------------
    -- stimulus generators
    ---------------------------------------------------------------------
    procedure SetTrig (thz : real; thx : real) is
    begin
      CosZ <= to_signed(RoundInt(cos(thz) * 2147483647.0), 32);
      SinZ <= to_signed(RoundInt(sin(thz) * 2147483647.0), 32);
      CosX <= to_signed(RoundInt(cos(thx) * 2147483647.0), 32);
      SinX <= to_signed(RoundInt(sin(thx) * 2147483647.0), 32);
    end procedure SetTrig;

    -- realistic sensor: periods inside the normalisation window, linear +
    -- small Chebyshev corrections, rotation close to a pure rotation
    procedure GenRandom is
      variable pmin, pmax, span, base, thz, thx : real;
      variable ex : integer;
    begin
      ex   := 18 + integer(floor(Rnd * 11.0));          -- coefficient scale 2**18 .. 2**28
      base := 2.0 ** ex;
      for e in 1 to 4 loop
        pmin := 1.0e6 + 0.35e6 * Rnd;
        pmax := 1.65e6 + 0.35e6 * Rnd;
        span := pmax - pmin;
        Gn(e)  <= to_signed(RoundInt(2.0 / span * 2.0 ** 36), 32);
        Ofs(e) <= to_signed(RoundInt(-(pmax + pmin) / span * 2.0 ** 24), 32);
        Per(e) <= to_unsigned(integer(pmin + span * (0.01 + 0.98 * Rnd)), 24);
        Cf(e)(1) <= to_signed(RoundInt(Sgn(Rnd) * base * (0.5 + Rnd)), 32);
        for k in 2 to ORDER loop
          Cf(e)(k) <= to_signed(RoundInt(base * (2.0 * Rnd - 1.0) * 0.35 ** (k - 1)), 32);
        end loop;
      end loop;
      GamX <= to_signed(RoundInt(base * (Rnd - 0.5)), 32);
      GamZ <= to_signed(RoundInt(base * (Rnd - 0.5)), 32);
      thz  := (2.0 * Rnd - 1.0) * MATH_PI;
      thx  := thz + (2.0 * Rnd - 1.0) * 0.4;
      SetTrig(thz, thx);
    end procedure GenRandom;

    -- X = +1 / -1 exactly (P*G + O with G = 2**17, O = -3*2**24)
    procedure GenStress (sel : natural) is
      variable xpos : boolean;
      variable cmax : signed(31 downto 0) := to_signed(2147483647, 32);
      variable cmin : signed(31 downto 0) := (31 => '1', others => '0');
    begin
      for e in 1 to 4 loop
        Gn(e)  <= to_signed(2 ** 17, 32);
        Ofs(e) <= to_signed(-3 * 2 ** 24, 32);
        -- ECS 1 and 3 give the most negative PL, ECS 2 and 4 the most positive
        xpos := ((e + sel) mod 3) /= 0;
        if xpos then
          Per(e) <= to_unsigned(2 ** 21, 24);        -- X = +1, Tk = 1
        else
          Per(e) <= to_unsigned(2 ** 20, 24);        -- X = -1, Tk = (-1)**k
        end if;
        for k in 1 to ORDER loop
          if xpos then
            if e mod 2 = 1 then Cf(e)(k) <= cmin; else Cf(e)(k) <= cmax; end if;
          else
            if (e mod 2 = 1) = (k mod 2 = 1) then Cf(e)(k) <= cmax; else Cf(e)(k) <= cmin; end if;
          end if;
        end loop;
      end loop;
      case sel mod 4 is
        when 0 =>
          GamX <= cmin; GamZ <= cmax;
          CosZ <= cmax; SinX <= cmax; SinZ <= cmin; CosX <= cmin;
        when 1 =>
          GamX <= cmax; GamZ <= cmin;
          CosZ <= cmin; SinX <= cmin; SinZ <= cmax; CosX <= cmax;
        when 2 =>
          GamX <= cmin; GamZ <= cmin;
          CosZ <= cmax; SinX <= cmin; SinZ <= cmax; CosX <= cmax;
        when others =>
          GamX <= cmax; GamZ <= cmax;
          CosZ <= cmin; SinX <= cmax; SinZ <= cmin; CosX <= cmin;
      end case;
    end procedure GenStress;

    -- zero coefficients: the angle is atan2 of a rotation of (Gamma_X, Gamma_Z)
    procedure GenGamma (n : natural) is
      variable mag, ang : real;
      variable m, gx, gz : integer;
    begin
      for e in 1 to 4 loop
        Gn(e)  <= to_signed(2 ** 17, 32);
        Ofs(e) <= to_signed(-3 * 2 ** 24, 32);
        Per(e) <= to_unsigned(2 ** 20 + RoundInt(Rnd * 2.0 ** 20), 24);
        for k in 1 to ORDER loop
          Cf(e)(k) <= (others => '0');
        end loop;
      end loop;
      m   := integer(floor(Rnd * 31.0));              -- magnitude 2**0 .. 2**30
      mag := 2.0 ** m * (0.5 + 0.5 * Rnd);
      ang := (2.0 * Rnd - 1.0) * MATH_PI;
      if n mod 10 = 0 then                            -- 1..3 LSB vectors
        gx := integer(floor(Rnd * 7.0)) - 3;
        gz := integer(floor(Rnd * 7.0)) - 3;
      else
        gx := RoundInt(mag * cos(ang));
        gz := RoundInt(mag * sin(ang));
      end if;
      if gx = 0 and gz = 0 then                       -- atan2(0, 0) is undefined
        gx := 1;
      end if;
      GamX <= to_signed(gx, 32);
      GamZ <= to_signed(gz, 32);
      SetTrig((2.0 * Rnd - 1.0) * MATH_PI, (2.0 * Rnd - 1.0) * MATH_PI);
    end procedure GenGamma;

    variable ok : boolean;
    variable tries : natural;
  begin
    wait for 100 ns;

    -- sanity: ATAN_TABLE and the two-argument arctan used as the reference
    assert abs(arctan(1.0, -1.0) - 3.0 * MATH_PI / 4.0) < 1.0e-6
      report "math_real arctan(y, x) is not atan2" severity failure;

    -- 1. random realistic cases ------------------------------------------
    for n in 0 to NRANDOM - 1 loop
      tries := 0;
      ok    := false;
      while not ok and tries < 50 loop
        GenRandom;
        wait for 1 ns;
        Reference(refang, refn, refd, refdx, refdz);
        -- the specification guarantees |X| <= 1; keep the vector well conditioned
        ok := abs(NormPeriod(1)) <= 1.0 and abs(NormPeriod(2)) <= 1.0 and
              abs(NormPeriod(3)) <= 1.0 and abs(NormPeriod(4)) <= 1.0 and
              sqrt(refn * refn + refd * refd) > 0.05 * 2.0 ** 18;
        tries := tries + 1;
      end loop;
      Run(n mod 4, "random " & integer'image(n));
      Check("random " & integer'image(n));
    end loop;

    -- 2. worst-case magnitudes ---------------------------------------------
    for n in 0 to 11 loop
      GenStress(n);
      wait for 1 ns;
      Run(n mod 2, "stress " & integer'image(n));
      Check("stress " & integer'image(n));
    end loop;

    -- 3. Gamma-only vectors from 2**30 down to 1 LSB -----------------------
    for n in 0 to NGAMMA - 1 loop
      GenGamma(n);
      wait for 1 ns;
      Run(n mod 4, "gamma " & integer'image(n));
      Check("gamma " & integer'image(n));
    end loop;

    errors := errors + ProtocolErrs;
    if errors = 0 then
      report "tb_AngleCompute PASSED: " & integer'image(ncases) & " cases, worst error " &
             real'image(worst) & " LSB, latency from last valid to ValidxSO " &
             integer'image(minlat) & ".." & integer'image(maxlat) & " clocks";
    else
      report "tb_AngleCompute FAILED, " & integer'image(errors) & " errors" severity failure;
    end if;
    finish;
  end process stim;

end architecture sim;

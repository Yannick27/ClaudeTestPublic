-------------------------------------------------------------------------------
-- Self-checking testbench for AngleCompute.
-- Compares the angle with a floating point reference computed from the same
-- (quantized) inputs. Run: see tb/run_ghdl.sh
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.AngleCompute_pkg.all;

entity AngleCompute_tb is
  generic(NTEST : positive := 300);
end entity AngleCompute_tb;

architecture sim of AngleCompute_tb is

  constant ORDER_C : positive := CHEBY_ORDER_C;
  constant FRAC_C  : natural  := 24;

  signal ClkxC      : std_logic := '0';
  signal StartxS    : std_logic := '0';
  signal ValidxS    : std_logic;
  signal AnglexD    : unsigned(15 downto 0);
  signal PeriodxD   : EcsPeriod_t(1 to 4) := (others => (others => '0'));
  signal PerValidxS : std_logic_vector(1 to 4) := (others => '0');
  signal GainxD     : EcsGain_t(1 to 4)   := (others => (others => '0'));
  signal OffsetxD   : EcsOffset_t(1 to 4) := (others => (others => '0'));
  signal CoeffXPxD, CoeffXNxD, CoeffZPxD, CoeffZNxD : ChebyshevCoeff_t := (others => (others => '0'));
  signal GammaXxD, GammaZxD : signed(31 downto 0) := (others => '0');
  signal CosXxD, SinXxD, CosZxD, SinZxD : signed(31 downto 0) := (others => '0');

  signal ValidCntxD : natural := 0;

  function ToReal(s : signed) return real is
  begin
    return real(to_integer(s));
  end function;

  -- Chebyshev linearization reference: sum Tk(x)*c(k), Tk(x) = cos(k*acos(x))
  function CheLin(x : real; c : ChebyshevCoeff_t) return real is
    variable r  : real := 0.0;
    variable xc : real := x;
  begin
    if xc > 1.0 then xc := 1.0; end if;
    if xc < -1.0 then xc := -1.0; end if;
    for k in 1 to ORDER_C loop
      r := r + cos(real(k) * arccos(xc)) * ToReal(c(k)) / 2.0 ** FRAC_C;
    end loop;
    return r;
  end function;

begin

  ClkxC <= not ClkxC after 5 ns;

  dut : entity work.AngleCompute
    generic map(ORDER => ORDER_C, FRAC => FRAC_C)
    port map(
      ClkxCI => ClkxC, StartxSI => StartxS, ValidxSO => ValidxS, AnglexDO => AnglexD,
      PeriodEcsxDI => PeriodxD, PeriodEcsValidxSI => PerValidxS,
      GainNormxDI => GainxD, OffsetNormxDI => OffsetxD,
      ChebyshevCoeffXPxDI => CoeffXPxD, ChebyshevCoeffXNxDI => CoeffXNxD,
      ChebyshevCoeffZPxDI => CoeffZPxD, ChebyshevCoeffZNxDI => CoeffZNxD,
      GammaXxDI => GammaXxD, GammaZxDI => GammaZxD,
      CosThetaXxDI => CosXxD, SinThetaXxDI => SinXxD,
      CosThetaZxDI => CosZxD, SinThetaZxDI => SinZxD);

  -- ValidxSO must be a single clock pulse
  process(ClkxC)
    variable prev : std_logic := '0';
  begin
    if rising_edge(ClkxC) then
      assert not (prev = '1' and ValidxS = '1') report "ValidxSO longer than 1 clock" severity failure;
      if ValidxS = '1' then ValidCntxD <= ValidCntxD + 1; end if;
      prev := ValidxS;
    end if;
  end process;

  stim : process
    variable s1 : positive := 11;
    variable s2 : positive := 29;
    type RealArr_t is array (1 to 4) of real;
    type CoeffArr_t is array (1 to 4) of ChebyshevCoeff_t;
    variable u       : real;
    variable cf      : CoeffArr_t;
    variable lin     : RealArr_t;
    variable per     : EcsPeriod_t(1 to 4);
    variable xn      : real;
    variable gx, gz  : real;
    variable thx, thz : real;
    variable dx, dz, num, den, aexp, adut, diff : real;
    variable maxerr  : real := 0.0;
    variable nbig    : natural := 0;
    variable cyc     : natural;
    variable skipped : natural := 0;

    procedure Rnd(variable r : out real) is
    begin
      uniform(s1, s2, r);
    end procedure;

    function Q31(v : real) return signed is
      variable r : real;
    begin
      r := round(v * (2.0 ** 31 - 1.0));
      return to_signed(integer(r), 32);
    end function;
  begin
    -- Gain = 2/1e6 (Q-4.36), Offset = -3 (Q8.24) : normalize [1e6, 2e6] to [-1, 1]
    for c in 1 to 4 loop
      GainxD(c)   <= to_signed(integer(round(2.0e-6 * 2.0 ** 36)), 32);
      OffsetxD(c) <= to_signed(-3 * 2 ** 24, 32);
    end loop;
    wait for 100 ns;

    for t in 1 to NTEST loop
      -- random parameters
      for c in 1 to 4 loop
        for k in 1 to ORDER_C loop
          Rnd(u);
          if k = 1 then
            cf(c)(k) := to_signed(integer(round((u * 4.0 - 2.0) * 2.0 ** FRAC_C)), 32);
          else
            cf(c)(k) := to_signed(integer(round((u * 0.6 - 0.3) * 2.0 ** FRAC_C)), 32);
          end if;
        end loop;
        Rnd(u);
        per(c) := to_unsigned(1000000 + integer(floor(u * 1000000.0)), 24);
      end loop;
      if t = 1 then per := (others => to_unsigned(1000000, 24)); end if;
      if t = 2 then per := (others => to_unsigned(2000000, 24)); end if;
      Rnd(u); gx := round((u - 0.5) * 2.0 ** FRAC_C);
      Rnd(u); gz := round((u - 0.5) * 2.0 ** FRAC_C);
      Rnd(u); thx := u * MATH_2_PI;
      Rnd(u); thz := u * MATH_2_PI;

      CoeffXPxD <= cf(1); CoeffXNxD <= cf(2); CoeffZPxD <= cf(3); CoeffZNxD <= cf(4);
      GammaXxD <= to_signed(integer(gx), 32);
      GammaZxD <= to_signed(integer(gz), 32);
      CosXxD <= Q31(cos(thx)); SinXxD <= Q31(sin(thx));
      CosZxD <= Q31(cos(thz)); SinZxD <= Q31(sin(thz));
      PeriodxD <= per;
      PerValidxS <= (others => '0');
      wait until rising_edge(ClkxC);

      -- reference
      for c in 1 to 4 loop
        xn := real(to_integer(per(c))) * real(to_integer(GainxD(c))) / 2.0 ** 36
              + real(to_integer(OffsetxD(c))) / 2.0 ** 24;
        lin(c) := CheLin(xn, cf(c));
      end loop;
      dx  := lin(1) - lin(2) + gx / 2.0 ** FRAC_C;
      dz  := lin(3) - lin(4) + gz / 2.0 ** FRAC_C;
      num := cos(thz) * dx - sin(thx) * dz;
      den := sin(thz) * dx + cos(thx) * dz;
      aexp := arctan(num, den) / MATH_2_PI * 65536.0;

      -- run : staggered period valid flags (held until the result)
      Rnd(u);
      for w in 1 to integer(u * 5.0) loop wait until rising_edge(ClkxC); end loop;
      StartxS <= '1';
      wait until rising_edge(ClkxC);
      StartxS <= '0';
      for c in 1 to 4 loop
        Rnd(u);
        if t mod 3 /= 0 then          -- some runs have all valid flags already set
          for w in 1 to integer(u * 40.0) loop wait until rising_edge(ClkxC); end loop;
        end if;
        PerValidxS(c) <= '1';
      end loop;
      if t mod 3 = 0 then PerValidxS <= (others => '1'); end if;

      cyc := 0;
      while ValidxS /= '1' loop
        wait until rising_edge(ClkxC);
        cyc := cyc + 1;
        assert cyc < 2000 report "timeout" severity failure;
      end loop;
      wait for 1 ns;
      PerValidxS <= (others => '0');
      adut := real(to_integer(AnglexD));
      diff := abs(adut - aexp);
      diff := diff - 65536.0 * round(diff / 65536.0);
      diff := abs(diff);
      if sqrt(num * num + den * den) < 1.0e-3 then
        skipped := skipped + 1;       -- vector too small : ill-conditioned reference
      else
        if diff > maxerr then maxerr := diff; end if;
        if diff > 1.5 then
          nbig := nbig + 1;
          report "test " & integer'image(t) & " dut=" & real'image(adut) & " exp=" & real'image(aexp)
                 & " |v|=" & real'image(sqrt(num * num + den * den)) severity error;
        end if;
      end if;
      wait until rising_edge(ClkxC);
    end loop;

    -- degenerate case : all coefficients and offsets zero -> Num = Den = 0, must still terminate
    CoeffXPxD <= (others => (others => '0')); CoeffXNxD <= (others => (others => '0'));
    CoeffZPxD <= (others => (others => '0')); CoeffZNxD <= (others => (others => '0'));
    GammaXxD <= (others => '0'); GammaZxD <= (others => '0');
    PerValidxS <= (others => '1');
    StartxS <= '1';
    wait until rising_edge(ClkxC);
    StartxS <= '0';
    cyc := 0;
    while ValidxS /= '1' loop
      wait until rising_edge(ClkxC);
      cyc := cyc + 1;
      assert cyc < 2000 report "timeout on zero vector" severity failure;
    end loop;
    report "zero vector done after " & integer'image(cyc) & " clocks";
    wait until rising_edge(ClkxC);

    report "DONE: " & integer'image(NTEST) & " tests, max error = " & real'image(maxerr)
           & " LSB, errors>1.5LSB = " & integer'image(nbig) & ", skipped = " & integer'image(skipped)
           & ", valid pulses = " & integer'image(ValidCntxD);
    assert nbig = 0 and ValidCntxD = NTEST + 1 report "TEST FAILED" severity failure;
    report "TEST PASSED";
    std.env.finish;
  end process;

end architecture sim;

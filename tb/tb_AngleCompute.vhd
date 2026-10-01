-- Self-checking testbench: compares AngleCompute with a floating-point model.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.AngleComputePkg.all;

entity tb_AngleCompute is
end entity;

architecture sim of tb_AngleCompute is
  signal Clk      : std_logic := '0';
  signal Start    : std_logic := '0';
  signal Valid    : std_logic;
  signal Angle    : unsigned(15 downto 0);
  signal Period   : EcsPeriod_t(1 to 4) := (others => (others => '0'));
  signal PValid   : std_logic_vector(1 to 4) := (others => '0');
  signal Gain     : EcsGain_t(1 to 4) := (others => (others => '0'));
  signal Offset   : EcsOffset_t(1 to 4) := (others => (others => '0'));
  signal CXP, CXN, CZP, CZN : ChebyshevCoeff_t := (others => (others => '0'));
  signal GX, GZ, CTX, STX, CTZ, STZ : signed(31 downto 0) := (others => '0');
  signal Done     : boolean := false;
begin

  Clk <= not Clk after 5 ns when not Done;

  dut : entity work.AngleCompute
    port map(ClkxCI => Clk, StartxSI => Start, ValidxSO => Valid, AnglexDO => Angle,
             PeriodEcsxDI => Period, PeriodEcsValidxSI => PValid,
             GainNormxDI => Gain, OffsetNormxDI => Offset,
             ChebyshevCoeffXPxDI => CXP, ChebyshevCoeffXNxDI => CXN,
             ChebyshevCoeffZPxDI => CZP, ChebyshevCoeffZNxDI => CZN,
             GammaXxDI => GX, GammaZxDI => GZ,
             CosThetaXxDI => CTX, SinThetaXxDI => STX,
             CosThetaZxDI => CTZ, SinThetaZxDI => STZ);

  process
    variable s1, s2 : positive := 7;
    variable seed1  : positive := 12345;
    variable seed2  : positive := 678;
    variable r      : real;
    variable cyc    : integer;
    variable errs   : integer := 0;
    variable maxerr : integer := 0;

    impure function rnd(lo, hi : real) return real is
      variable x : real;
    begin
      uniform(seed1, seed2, x);
      return lo + (hi - lo) * x;
    end function;

    function q(x : real; frac : natural) return signed is
    begin
      return to_signed(integer(round(x * 2.0**frac)), 32);
    end function;

    type real4 is array (1 to 4) of real;
    type real6 is array (0 to 5) of real;
    type real64 is array (1 to 4) of real6;
    variable cf   : real64;
    variable per  : real4;
    variable nrm  : real4;
    variable lin  : real4;
    variable g, o : real4;
    variable t0, t1, t2, xx : real;
    variable gx_r, gz_r, ax, az, cx_r, sx_r, cz_r, sz_r : real;
    variable dx, dz, nn, dd, ang, expv : real;
    variable e    : integer;
    variable expi : integer;
    variable tmpc : ChebyshevCoeff_t;
  begin
    wait for 100 ns;
    for trial in 1 to 40 loop
      -- random parameters
      for n in 1 to 4 loop
        per(n) := real(integer(rnd(1.0e6, 2.0e6)));
        g(n)   := 2.0e-6 * rnd(0.9, 1.1);
        o(n)   := -g(n) * 1.5e6;                       -- maps 1.5e6 to 0
        Period(n) <= to_unsigned(integer(per(n)), 24);
        Gain(n)   <= to_signed(integer(round(g(n) * 2.0**GAIN_FRAC)), 32);
        Offset(n) <= q(o(n), OFFSET_FRAC);
        -- use the quantized values for the model
        g(n) := real(to_integer(to_signed(integer(round(g(n) * 2.0**GAIN_FRAC)), 32))) / 2.0**GAIN_FRAC;
        o(n) := real(to_integer(q(o(n), OFFSET_FRAC))) / 2.0**OFFSET_FRAC;
        cf(n)(0) := rnd(0.6, 1.0);
        for k in 1 to 5 loop cf(n)(k) := rnd(-0.3, 0.3); end loop;
        for k in 0 to 5 loop
          tmpc(k) := q(cf(n)(k), Q_FRAC);
          cf(n)(k) := real(to_integer(tmpc(k))) / 2.0**Q_FRAC;
        end loop;
        case n is
          when 1 => CXP <= tmpc;
          when 2 => CXN <= tmpc;
          when 3 => CZP <= tmpc;
          when others => CZN <= tmpc;
        end case;
      end loop;
      gx_r := rnd(-0.1, 0.1);  gz_r := rnd(-0.1, 0.1);
      GX <= q(gx_r, Q_FRAC);   GZ <= q(gz_r, Q_FRAC);
      gx_r := real(to_integer(q(gx_r, Q_FRAC))) / 2.0**Q_FRAC;
      gz_r := real(to_integer(q(gz_r, Q_FRAC))) / 2.0**Q_FRAC;
      ax := rnd(-0.2, 0.2); az := rnd(-0.2, 0.2);
      if trial mod 5 = 0 then ax := 0.0; az := 0.0; end if;
      CTX <= q(cos(ax), Q_FRAC); STX <= q(sin(ax), Q_FRAC);
      CTZ <= q(cos(az), Q_FRAC); STZ <= q(sin(az), Q_FRAC);
      cx_r := real(to_integer(q(cos(ax), Q_FRAC))) / 2.0**Q_FRAC;
      sx_r := real(to_integer(q(sin(ax), Q_FRAC))) / 2.0**Q_FRAC;
      cz_r := real(to_integer(q(cos(az), Q_FRAC))) / 2.0**Q_FRAC;
      sz_r := real(to_integer(q(sin(az), Q_FRAC))) / 2.0**Q_FRAC;

      -- reference model
      for n in 1 to 4 loop
        xx := per(n) * g(n) + o(n);
        if xx > 1.0 then xx := 1.0; end if;
        if xx < -1.0 then xx := -1.0; end if;
        t0 := 1.0; t1 := xx; lin(n) := 0.0;
        for k in 1 to 6 loop
          lin(n) := lin(n) + t1 * cf(n)(k-1);
          t2 := 2.0 * xx * t1 - t0;
          t0 := t1; t1 := t2;
        end loop;
      end loop;
      dx := lin(1) - lin(2) + gx_r;
      dz := lin(3) - lin(4) + gz_r;
      nn := cz_r * dx - sx_r * dz;
      dd := sz_r * dx + cx_r * dz;
      ang := arctan(nn, dd);       -- (-pi, pi]
      if ang < 0.0 then ang := ang + MATH_2_PI; end if;
      expi := integer(round(ang / MATH_2_PI * 65536.0)) mod 65536;

      -- stagger the valids: each period becomes valid at a different time
      PValid <= (others => '0');
      wait until rising_edge(Clk);
      Start <= '1';
      wait until rising_edge(Clk);
      Start <= '0';
      for i in 1 to 4 loop wait until rising_edge(Clk); end loop;  -- channels armed
      for n in 1 to 4 loop
        for i in 1 to (trial mod 3) * n loop wait until rising_edge(Clk); end loop;
        PValid(n) <= '1';
        wait until rising_edge(Clk);
        PValid(n) <= '0';
      end loop;

      cyc := 0;
      while Valid = '0' and cyc < 5000 loop
        wait until rising_edge(Clk);
        cyc := cyc + 1;
      end loop;
      wait until rising_edge(Clk);

      e := abs(to_integer(Angle) - expi);
      if e > 32768 then e := 65536 - e; end if;
      if e > maxerr then maxerr := e; end if;
      report "trial " & integer'image(trial) & " cycles=" & integer'image(cyc) &
             " angle=" & integer'image(to_integer(Angle)) & " expected=" & integer'image(expi) &
             " err=" & integer'image(e);
      if cyc >= 5000 or e > 2 then
        errs := errs + 1;
        report "MISMATCH" severity error;
      end if;
    end loop;
    report "DONE errors=" & integer'image(errs) & " maxerr=" & integer'image(maxerr) & " LSB";
    Done <= true;
    wait;
  end process;
end architecture;

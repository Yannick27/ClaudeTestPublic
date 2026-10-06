-------------------------------------------------------------------------------
-- Self-checking testbench for Atan2Cordic.
-- Random vectors with magnitudes from 1 LSB up to the full input range (X and Y
-- scaled independently or together), plus the axes, the most negative input
-- value and the zero vector.  The result must be within MAX_ERR of the exact
-- atan2 (in LSB of 65536 = 2*pi, wrap-around aware).
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.env.all;
use work.AngleComputePkg.all;

entity tb_Atan2Cordic is
  generic (
    WIN     : positive := 74;
    NTESTS  : natural  := 4000;
    MAX_ERR : real     := 0.7
  );
end entity tb_Atan2Cordic;

architecture sim of tb_Atan2Cordic is

  signal Clk   : std_logic := '0';
  signal Start : std_logic := '0';
  signal X     : signed(WIN - 1 downto 0) := (others => '0');
  signal Y     : signed(WIN - 1 downto 0) := (others => '0');
  signal Valid : std_logic;
  signal Angle : unsigned(ANGLE_W - 1 downto 0);

  -- Horner scheme from the MSB: exact for small magnitudes (summing the weights
  -- of a negative number would round away the low bits)
  function ToReal (v : signed) return real is
    variable r : real := 0.0;
  begin
    if v(v'left) = '1' then
      r := -1.0;
    end if;
    for i in v'left - 1 downto v'right loop
      r := 2.0 * r;
      if v(i) = '1' then
        r := r + 1.0;
      end if;
    end loop;
    return r;
  end function ToReal;

  -- signed circular difference of two angles in LSB
  function AngDiff (a : real; b : real) return real is
    variable d : real := a - b;
  begin
    while d >= 32768.0 loop d := d - 65536.0; end loop;
    while d < -32768.0 loop d := d + 65536.0; end loop;
    return d;
  end function AngDiff;

begin

  Clk <= not Clk after 5 ns;

  dut : entity work.Atan2Cordic
    generic map (WIN => WIN)
    port map (ClkxCI => Clk, StartxSI => Start, XxDI => X, YxDI => Y,
              ValidxSO => Valid, AnglexDO => Angle);

  stim : process
    variable s1, s2 : positive := 3;
    variable r      : real;
    variable errors : natural := 0;
    variable worst  : real := 0.0;
    variable maxlat : natural := 0;
    variable minlat : natural := 1000;
    variable cycles : natural;
    variable xv, yv : signed(WIN - 1 downto 0);
    variable sx, sy : natural;
    variable ref, err : real;

    impure function RandBits return signed is
      variable v : signed(WIN - 1 downto 0);
    begin
      for i in 0 to WIN - 1 loop
        uniform(s1, s2, r);
        if r > 0.5 then v(i) := '1'; else v(i) := '0'; end if;
      end loop;
      return v;
    end function RandBits;

    procedure Check (xin : signed; yin : signed; zero : boolean) is
    begin
      X <= xin;
      Y <= yin;
      wait until falling_edge(Clk);
      Start <= '1';
      wait until falling_edge(Clk);
      Start <= '0';
      cycles := 1;
      while Valid /= '1' and cycles < 400 loop
        wait until falling_edge(Clk);
        cycles := cycles + 1;
      end loop;
      if Valid /= '1' then
        report "timeout" severity error;
        errors := errors + 1;
      else
        if cycles > maxlat then maxlat := cycles; end if;
        if cycles < minlat then minlat := cycles; end if;
        if not (zero or (ToReal(xin) = 0.0 and ToReal(yin) = 0.0)) then
          ref := arctan(ToReal(yin), ToReal(xin)) * 65536.0 / MATH_2_PI;
          err := AngDiff(real(to_integer(Angle)), ref);
          if abs(err) > worst then worst := abs(err); end if;
          if abs(err) > MAX_ERR then
            report "angle error " & real'image(err) & " LSB for x=" & real'image(ToReal(xin)) &
                   " y=" & real'image(ToReal(yin)) & " got " & integer'image(to_integer(Angle)) &
                   " exp " & real'image(ref) severity error;
            errors := errors + 1;
          end if;
        end if;
        wait until falling_edge(Clk);
        if Valid /= '0' then
          report "Valid longer than one clock" severity error;
          errors := errors + 1;
        end if;
      end if;
    end procedure Check;

    variable big, small, one, minv : signed(WIN - 1 downto 0);
  begin
    -- sanity check of the math_real two-argument arctan: atan2(1, -1) = 3*pi/4
    assert abs(arctan(1.0, -1.0) - 3.0 * MATH_PI / 4.0) < 1.0e-6
      report "math_real arctan(y, x) is not atan2" severity failure;
    -- the CORDIC table must match atan(2**-i) * 2**24 / (2*pi)
    for i in 0 to CORDIC_ITER - 1 loop
      assert abs(real(to_integer(ATAN_TABLE(i))) -
                 arctan(2.0 ** (-i)) / MATH_2_PI * 2.0 ** (ANGLE_W + ANGLE_FRAC)) <= 0.55
        report "ATAN_TABLE entry " & integer'image(i) & " is wrong" severity failure;
    end loop;

    wait until falling_edge(Clk);

    one  := to_signed(1, WIN);
    big  := (others => '0'); big(WIN - 2 downto 0) := (others => '1');   -- 2**(WIN-1) - 1
    minv := (others => '0'); minv(WIN - 1) := '1';                       -- -2**(WIN-1)
    small := to_signed(-1, WIN);

    -- axes and diagonals at several magnitudes
    for m in 0 to WIN - 2 loop
      xv := shift_left(one, m);
      yv := (others => '0');
      Check(xv, yv, false);
      Check(-xv, yv, false);
      Check(yv, xv, false);
      Check(yv, -xv, false);
      Check(xv, xv, false);
      Check(-xv, xv, false);
      Check(-xv, -xv, false);
      Check(xv, -xv, false);
    end loop;
    Check(big, big, false);
    Check(minv, minv, false);
    Check(minv, big, false);
    Check(big, minv, false);
    Check(small, small, false);
    Check(one, small, false);
    Check(minv, to_signed(0, WIN), false);
    Check(to_signed(0, WIN), minv, false);
    Check(to_signed(0, WIN), to_signed(0, WIN), true);   -- result unspecified, must finish

    -- random vectors, log-uniform magnitudes
    for n in 0 to NTESTS - 1 loop
      xv := RandBits;
      yv := RandBits;
      uniform(s1, s2, r);
      sx := integer(floor(r * real(WIN - 1)));
      uniform(s1, s2, r);
      if r > 0.5 then
        sy := sx;
      else
        sy := integer(floor(r * 2.0 * real(WIN - 1)));
        if sy > WIN - 2 then sy := WIN - 2; end if;
      end if;
      xv := shift_right(xv, sx);
      yv := shift_right(yv, sy);
      Check(xv, yv, false);
    end loop;

    if errors = 0 then
      report "tb_Atan2Cordic PASSED, worst error " & real'image(worst) & " LSB, latency " &
             integer'image(minlat) & ".." & integer'image(maxlat) & " clocks";
    else
      report "tb_Atan2Cordic FAILED, " & integer'image(errors) & " errors" severity failure;
    end if;
    finish;
  end process;

end architecture sim;

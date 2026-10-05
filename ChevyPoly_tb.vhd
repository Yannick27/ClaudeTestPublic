library ieee; use ieee.std_logic_1164.all; use ieee.numeric_std.all; use ieee.math_real.all;
use work.ChevyPolyPkg.all;
entity tb is end;
architecture sim of tb is
  constant ORDER : positive := 6; constant FRAC : natural := 24;
  signal clk : std_logic := '0';
  signal per : unsigned(23 downto 0) := (others=>'0');
  signal pv, v : std_logic := '0';
  signal gain, offs : signed(31 downto 0);
  signal coef : ChebyshevCoeff_t(1 to ORDER);
  signal res : signed(31 downto 0);
  constant cr : real_vector(1 to ORDER) := (0.5, -0.25, 0.1, 1.5, -2.0, 0.75);
begin
  clk <= not clk after 5 ns;
  gain <= to_signed(integer(floor(2.0e-6 * 2.0**36)), 32);
  offs <= to_signed(integer(round(-2.999985 * 2.0**24)), 32);
  g: for i in 1 to ORDER generate
    coef(i) <= to_signed(integer(round(cr(i)*2.0**FRAC)), 32);
  end generate;
  dut: entity work.ChevyPoly generic map(ORDER, FRAC)
    port map(clk, per, pv, gain, offs, coef, v, res);
  process
    variable p : integer; variable x, t0, t1, t2, e, maxerr : real := 0.0;
    variable cyc : integer;
  begin
    maxerr := 0.0;
    for n in 0 to 20 loop
      case n is when 0 => p := 1000000; when 1 => p := 2000000; when 2 => p := 1500000;
        when others => p := 1000000 + (n*49999) mod 1000001; end case;
      wait until rising_edge(clk);
      per <= to_unsigned(p,24); pv <= '1';
      wait until rising_edge(clk); pv <= '0'; cyc := 1;
      while v /= '1' loop wait until rising_edge(clk); cyc := cyc+1; end loop;
      wait for 1 ns;
      x := (real(p) * real(to_integer(gain))/2.0**36) + real(to_integer(offs))/2.0**24;
      t0 := 1.0; t1 := x; e := cr(1)*x;
      for k in 2 to ORDER loop t2 := 2.0*x*t1 - t0; e := e + cr(k)*t2; t0 := t1; t1 := t2; end loop;
      e := e - real(to_integer(res))/2.0**FRAC;
      if abs(e) > maxerr then maxerr := abs(e); end if;
      report "p=" & integer'image(p) & " x=" & real'image(x) & " out=" & real'image(real(to_integer(res))/2.0**FRAC) & " err(LSB)=" & real'image(e*2.0**FRAC) & " lat=" & integer'image(cyc);
      wait until rising_edge(clk); assert v='0' report "valid longer than 1 cycle" severity error;
    end loop;
    report "max err (LSB) = " & real'image(maxerr*2.0**FRAC);
    assert maxerr*2.0**FRAC < 4.0 report "FAIL" severity failure;
    report "PASS"; std.env.finish;
  end process;
end;

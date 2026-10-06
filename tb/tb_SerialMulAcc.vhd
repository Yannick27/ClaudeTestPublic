-------------------------------------------------------------------------------
-- Self-checking testbench for SerialMulAcc.
-- Compares the engine with numeric_std full-width multiplication for random and
-- extreme operands.  Run it for several (WA, WB, NT) combinations, e.g.
--   -gWA=32 -gWB=32 -gNT=1   (EcsLinearizer instance)
--   -gWA=41 -gWB=32 -gNT=2   (AngleCompute rotation instance, ORDER = 6)
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.env.all;
use work.AngleComputePkg.all;

entity tb_SerialMulAcc is
  generic (
    WA     : positive := 32;
    WB     : positive := 32;
    NT     : positive := 1;
    NTESTS : natural  := 3000
  );
end entity tb_SerialMulAcc;

architecture sim of tb_SerialMulAcc is

  constant WP   : positive := WA + WB + CeilLog2(NT);
  constant NA   : positive := (WA + 15) / 16;
  constant NB   : positive := (WB + 15) / 16;
  constant LAT  : positive := NT * NA * NB + 5;

  signal Clk   : std_logic := '0';
  signal Start : std_logic := '0';
  signal A     : signed(NT * WA - 1 downto 0) := (others => '0');
  signal B     : signed(NT * WB - 1 downto 0) := (others => '0');
  signal Sub   : std_logic_vector(NT - 1 downto 0) := (others => '0');
  signal Done  : std_logic;
  signal P     : signed(WP - 1 downto 0);

begin

  Clk <= not Clk after 5 ns;

  dut : entity work.SerialMulAcc
    generic map (WA => WA, WB => WB, NT => NT)
    port map (ClkxCI => Clk, StartxSI => Start, AxDI => A, BxDI => B,
              SubxSI => Sub, DonexSO => Done, PxDO => P);

  stim : process
    variable s1, s2 : positive := 7;
    variable r      : real;
    variable errors : natural := 0;
    variable cycles : natural;
    variable ref    : signed(WP - 1 downto 0);
    variable prod   : signed(WA + WB - 1 downto 0);
    variable av     : signed(WA - 1 downto 0);
    variable bv     : signed(WB - 1 downto 0);
    variable sv     : std_logic_vector(NT - 1 downto 0);

    -- mode 0: random, 1: most negative, 2: most positive, 3: zero, 4: -1
    impure function Rand (w : positive; mode : integer) return signed is
      variable v : signed(w - 1 downto 0);
    begin
      case mode is
        when 1 =>
          v := (others => '0');
          v(w - 1) := '1';
        when 2 =>
          v := (others => '1');
          v(w - 1) := '0';
        when 3 =>
          v := (others => '0');
        when 4 =>
          v := (others => '1');
        when others =>
          for i in 0 to w - 1 loop
            uniform(s1, s2, r);
            if r > 0.5 then
              v(i) := '1';
            else
              v(i) := '0';
            end if;
          end loop;
      end case;
      return v;
    end function Rand;

    impure function PickMode (n : integer) return integer is
    begin
      case n mod 12 is
        when 0      => return 1;
        when 1      => return 2;
        when 2      => return 3;
        when 3      => return 4;
        when others => return 0;
      end case;
    end function PickMode;

  begin
    wait until falling_edge(Clk);
    for n in 0 to NTESTS - 1 loop
      ref := (others => '0');
      for t in 0 to NT - 1 loop
        av := Rand(WA, PickMode(n + t));
        bv := Rand(WB, PickMode(n / 12 + t));
        uniform(s1, s2, r);
        if r > 0.5 then
          sv(t) := '1';
        else
          sv(t) := '0';
        end if;
        A((t + 1) * WA - 1 downto t * WA) <= av;
        B((t + 1) * WB - 1 downto t * WB) <= bv;
        prod := av * bv;
        if sv(t) = '1' then
          ref := ref - resize(prod, WP);
        else
          ref := ref + resize(prod, WP);
        end if;
      end loop;
      Sub <= sv;

      wait until falling_edge(Clk);
      Start <= '1';
      wait until falling_edge(Clk);
      Start <= '0';

      cycles := 1;
      while Done /= '1' and cycles < 200 loop
        wait until falling_edge(Clk);
        cycles := cycles + 1;
      end loop;

      if Done /= '1' then
        report "timeout in test " & integer'image(n) severity error;
        errors := errors + 1;
      else
        if cycles /= LAT then
          report "latency " & integer'image(cycles) & " /= " & integer'image(LAT) severity error;
          errors := errors + 1;
        end if;
        if P /= ref then
          report "mismatch in test " & integer'image(n) severity error;
          errors := errors + 1;
        end if;
        wait until falling_edge(Clk);
        if Done /= '0' then
          report "Done longer than one clock" severity error;
          errors := errors + 1;
        end if;
        if P /= ref then
          report "P not stable after Done" severity error;
          errors := errors + 1;
        end if;
      end if;
    end loop;

    if errors = 0 then
      report "tb_SerialMulAcc PASSED (WA=" & integer'image(WA) & " WB=" & integer'image(WB) &
             " NT=" & integer'image(NT) & ", " & integer'image(NTESTS) & " tests, latency " &
             integer'image(LAT) & " clocks)";
    else
      report "tb_SerialMulAcc FAILED, " & integer'image(errors) & " errors" severity failure;
    end if;
    finish;
  end process;

end architecture sim;

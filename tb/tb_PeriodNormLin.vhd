-------------------------------------------------------------------------------
-- Self-checking testbench for PeriodNormLin.
-- The reference is computed in floating point (real) from the same quantized
-- parameters and compared with the fixed-point result.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

use work.PeriodNormLinPkg.all;

entity tb_PeriodNormLin is
end entity tb_PeriodNormLin;

architecture sim of tb_PeriodNormLin is
  constant CLK_PERIOD : time := 10 ns;

  signal ClkxC   : std_logic := '0';
  signal PeriodxD : unsigned(23 downto 0) := to_unsigned(1500000, 24);
  signal ValidInxS : std_logic := '0';
  signal GainxD   : signed(31 downto 0) := to_signed(137438, 32);          -- ~2e-6 (Q-4.36)
  signal OffsetxD : signed(31 downto 0) := to_signed(-3 * 2**24, 32);      -- -3.0  (Q8.24)
  signal CoeffxD  : ChebyshevCoeff_t(1 to 6) := (
    to_signed(integer(  1.00 * 2.0**24), 32),
    to_signed(integer(  0.30 * 2.0**24), 32),
    to_signed(integer( -0.25 * 2.0**24), 32),
    to_signed(integer( 12.50 * 2.0**24), 32),
    to_signed(integer(-40.00 * 2.0**24), 32),
    to_signed(integer( 20.10 * 2.0**24), 32));
  signal ValidOutxS : std_logic;
  signal ResultxD   : signed(31 downto 0);
begin

  ClkxC <= not ClkxC after CLK_PERIOD / 2;

  dut : entity work.PeriodNormLin
    port map(
      ClkxCI            => ClkxC,
      PeriodEcsxDI      => PeriodxD,
      PeriodEcsValidxSI => ValidInxS,
      GainNormxDI       => GainxD,
      OffsetNormxDI     => OffsetxD,
      ChebyshevCoeffxDI => CoeffxD,
      ValidxSO          => ValidOutxS,
      PeriodxDO         => ResultxD);

  stim : process
    variable X, T0, T1, T2, Ref, Got, Err, MaxErr : real;
    variable Cnt : integer := 0;
    variable Cycles : integer;
    variable Seed1 : positive := 7;
    variable Seed2 : positive := 13;
    variable Rnd : real;
    variable Per : integer;

    procedure run_one(P : integer) is
    begin
      PeriodxD <= to_unsigned(P, 24);
      wait until rising_edge(ClkxC);
      ValidInxS <= '1';
      Cycles := 0;
      -- ValidxSO of the previous computation is cleared when the FSM starts
      wait until rising_edge(ClkxC);
      wait until rising_edge(ClkxC);
      assert ValidOutxS = '0' report "ValidxSO not cleared at start" severity error;
      while ValidOutxS /= '1' and Cycles < 200 loop
        wait until rising_edge(ClkxC);
        Cycles := Cycles + 1;
      end loop;
      assert Cycles < 200 report "timeout waiting for ValidxSO" severity failure;
      wait for 1 ns;
      -- reference
      X := real(P) * real(to_integer(GainxD)) / 2.0**36 + real(to_integer(OffsetxD)) / 2.0**24;
      T0 := 1.0; T1 := X;
      Ref := T1 * real(to_integer(CoeffxD(1))) / 2.0**24;
      for k in 2 to 6 loop
        T2 := 2.0 * X * T1 - T0;
        Ref := Ref + T2 * real(to_integer(CoeffxD(k))) / 2.0**24;
        T0 := T1; T1 := T2;
      end loop;
      Got := real(to_integer(ResultxD)) / 2.0**24;
      Err := abs(Got - Ref);
      if Err > MaxErr then MaxErr := Err; end if;
      report "P=" & integer'image(P) & " X=" & real'image(X) & " ref=" & real'image(Ref) &
             " got=" & real'image(Got) & " err=" & real'image(Err) &
             " cycles=" & integer'image(Cycles);
      -- tolerance: Q2.30 rounding of PeriodNorm/Tk (~5e-10) amplified by the
      -- polynomial slope and coefficient size, plus the Q8.24 output LSB
      assert Err < 1.0e-6 report "MISMATCH" severity error;
      ValidInxS <= '0';
      wait until rising_edge(ClkxC);
      wait until rising_edge(ClkxC);
      Cnt := Cnt + 1;
    end procedure;
  begin
    MaxErr := 0.0;
    wait for 5 * CLK_PERIOD;
    run_one(1000000);
    run_one(2000000);
    run_one(1500000);
    run_one(1250000);
    run_one(1750001);
    for i in 1 to 60 loop
      uniform(Seed1, Seed2, Rnd);
      Per := 1000000 + integer(Rnd * 1000000.0);
      run_one(Per);
    end loop;
    -- other coefficient set (negative / large values)
    CoeffxD <= (to_signed(integer(-50.0 * 2.0**24), 32), to_signed(integer(40.0 * 2.0**24), 32),
                to_signed(integer(-0.001 * 2.0**24), 32), to_signed(integer(5.5 * 2.0**24), 32),
                to_signed(integer(-7.7 * 2.0**24), 32), to_signed(integer(0.0), 32));
    wait for 3 * CLK_PERIOD;
    for i in 1 to 20 loop
      uniform(Seed1, Seed2, Rnd);
      Per := 1000000 + integer(Rnd * 1000000.0);
      run_one(Per);
    end loop;
    report "Done: " & integer'image(Cnt) & " computations, max error = " & real'image(MaxErr) severity note;
    std.env.finish;
  end process;

end architecture sim;

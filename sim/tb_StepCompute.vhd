-- Self-checking testbench for StepCompute: every possible angle delta (0..65535)
-- is applied from several measured angles and compared to an integer model.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_StepCompute is
end entity;

architecture sim of tb_StepCompute is
  constant ROT_PER_STEP : integer := integer(0.32 / 360.0 * 2.0 ** 16);
  constant ROT_ACCURACY : integer := integer(0.01 / 360.0 * 2.0 ** 16);

  signal Clk      : std_logic := '0';
  signal Valid    : std_logic := '0';
  signal Meas     : unsigned(15 downto 0) := (others => '0');
  signal Targ     : unsigned(15 downto 0) := (others => '0');
  signal VMax     : std_logic_vector(15 downto 0);
  signal Dir      : std_logic;
  signal Steps    : natural range 0 to 2000;
  signal OutValid : std_logic;
  signal Done     : boolean := false;
begin
  Clk <= not Clk after 5 ns when not Done;

  dut : entity work.StepCompute
    port map (ClkxCI => Clk, ValidAnglexSI => Valid, AngleMeasuredxDI => Meas,
              AngleTargetxDI => Targ, VMaxExtxDO => VMax, RotDirectionxSO => Dir,
              StepNumberxDO => Steps, OutputValidxSO => OutValid);

  process
    variable prev_dir : std_logic := '0';
    variable n_tests  : natural   := 0;
    variable delta, diff, exp_steps, exp_vmax : integer;
    variable exp_dir  : std_logic;
    variable cycles, max_cycles : natural := 0;

    procedure run(m, t : integer) is
    begin
      wait until rising_edge(Clk);
      Meas <= to_unsigned(m, 16); Targ <= to_unsigned(t, 16); Valid <= '1';
      wait until rising_edge(Clk);
      Valid <= '0';
      cycles := 1;
      while OutValid /= '1' loop
        wait until rising_edge(Clk);
        cycles := cycles + 1;
        assert cycles < 100 report "timeout" severity failure;
      end loop;
      wait for 1 ns;
      if cycles > max_cycles then max_cycles := cycles; end if;

      delta := (t - m) mod 65536;
      if delta <= 32768 then diff := delta; exp_dir := '1';
      else diff := 65536 - delta; exp_dir := '0'; end if;

      if diff > ROT_PER_STEP then
        exp_steps := diff / ROT_PER_STEP; exp_vmax := 65535; prev_dir := exp_dir;
      elsif diff > ROT_ACCURACY then
        exp_steps := 1; exp_vmax := 65535 * diff / ROT_PER_STEP; prev_dir := exp_dir;
      else
        exp_steps := 0; exp_vmax := 0;
      end if;

      assert Steps = exp_steps and to_integer(unsigned(VMax)) = exp_vmax and Dir = prev_dir
        report "MISMATCH meas=" & integer'image(m) & " targ=" & integer'image(t) &
               " steps=" & integer'image(Steps) & "/" & integer'image(exp_steps) &
               " vmax=" & integer'image(to_integer(unsigned(VMax))) & "/" & integer'image(exp_vmax) &
               " dir=" & std_logic'image(Dir) & "/" & std_logic'image(prev_dir)
        severity failure;

      wait until rising_edge(Clk);
      assert OutValid = '0' report "OutputValid wider than 1 cycle" severity failure;
      n_tests := n_tests + 1;
    end procedure;
  begin
    -- back-to-back pulses (one angle per clock): expect 3 consecutive results
    wait until rising_edge(Clk);
    Meas <= to_unsigned(0, 16); Targ <= to_unsigned(1000, 16); Valid <= '1';  -- 17 steps
    wait until rising_edge(Clk);
    Targ <= to_unsigned(20, 16);                                              -- 1 step, vmax 22598
    wait until rising_edge(Clk);
    Targ <= to_unsigned(1, 16);                                               -- reached
    wait until rising_edge(Clk);
    Valid <= '0';
    wait until OutValid = '1';
    wait for 1 ns;
    assert Steps = 17 and VMax = x"FFFF" and Dir = '1' report "burst 1 wrong" severity failure;
    wait until rising_edge(Clk); wait for 1 ns;
    assert OutValid = '1' and Steps = 1 and to_integer(unsigned(VMax)) = 22598 report "burst 2 wrong" severity failure;
    wait until rising_edge(Clk); wait for 1 ns;
    assert OutValid = '1' and Steps = 0 and VMax = x"0000" report "burst 3 wrong" severity failure;
    wait until rising_edge(Clk); wait for 1 ns;
    assert OutValid = '0' report "burst: extra valid" severity failure;
    prev_dir := '1';

    run(65535, 1);   -- wrap-around example: 2 LSB away, counter-clockwise
    run(1, 65535);   -- same, clockwise
    run(0, 0);
    for base in 0 to 3 loop
      for d in 0 to 65535 loop
        run((base * 21845 + 7) mod 65536, ((base * 21845 + 7) + d) mod 65536);
      end loop;
    end loop;
    report "PASS: " & integer'image(n_tests) & " tests, max latency " &
           integer'image(max_cycles) & " cycles" severity note;
    Done <= true;
    wait;
  end process;
end architecture;

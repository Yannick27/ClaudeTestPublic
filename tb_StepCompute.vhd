library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_StepCompute is
end entity;

architecture sim of tb_StepCompute is
  signal clk    : std_logic := '0';
  signal valid  : std_logic := '0';
  signal meas   : unsigned(15 downto 0) := (others => '0');
  signal targ   : unsigned(15 downto 0) := (others => '0');
  signal vmax   : std_logic_vector(15 downto 0);
  signal dir    : std_logic;
  signal steps  : natural range 0 to 2000;
  signal ovalid : std_logic;
  signal done   : boolean := false;
begin
  clk <= not clk after 5 ns when not done;

  dut : entity work.StepCompute
    port map (ClkxCI => clk, ValidAnglexSI => valid, AngleMeasuredxDI => meas,
              AngleTargetxDI => targ, VMaxExtxDO => vmax, RotDirectionxSO => dir,
              StepNumberxDO => steps, OutputValidxSO => ovalid);

  process
    procedure run(m, t : natural; exp_steps : natural; exp_dir : std_logic;
                  exp_vmax : natural; check_dir : boolean := true) is
      variable n : natural := 0;
    begin
      wait until falling_edge(clk);
      meas <= to_unsigned(m, 16); targ <= to_unsigned(t, 16); valid <= '1';
      wait until falling_edge(clk);
      valid <= '0';
      while ovalid /= '1' and n < 10 loop
        wait until falling_edge(clk); n := n + 1;
      end loop;
      assert ovalid = '1' report "no OutputValid" severity failure;
      assert n = 3 report "latency " & integer'image(n) severity failure;
      assert steps = exp_steps
        report "steps " & integer'image(steps) & " /= " & integer'image(exp_steps) &
               " for " & integer'image(m) & "->" & integer'image(t) severity failure;
      assert to_integer(unsigned(vmax)) = exp_vmax
        report "vmax " & integer'image(to_integer(unsigned(vmax))) & " /= " & integer'image(exp_vmax) severity failure;
      if check_dir then
        assert dir = exp_dir report "dir wrong for " & integer'image(m) & "->" & integer'image(t) severity failure;
      end if;
      wait until falling_edge(clk);
      assert ovalid = '0' report "OutputValid wider than 1 cycle" severity failure;
    end procedure;
  begin
    -- far, CCW: diff 1000 -> 1000*1125/65536 = 17
    run(0, 1000, 17, '1', 61166);
    -- far, CW
    run(1000, 0, 17, '0', 61166);
    -- wrap-around: 65535 -> 1 is 2 LSB CCW (close, not accurate: 2 <= ACCURACY=2 -> reached)
    run(65535, 1, 0, '0', 0, false);
    -- wrap-around: 1 -> 65535 : 2 LSB CW -> reached
    run(1, 65535, 0, '0', 0, false);
    -- close: diff 10 CCW across the wrap  (65530 -> 4)
    run(65530, 4, 1, '1', 10 * 1050);
    -- close: diff 58 CW  (ROT_PER_STEP = 58)
    run(100, 42, 1, '0', 58 * 1050);
    -- just above ROT_PER_STEP: 59 -> far, 59*1125/65536 = 1
    run(100, 159, 1, '1', 61166);
    -- diff 3 (> accuracy 2) -> partial
    run(500, 503, 1, '1', 3 * 1050);
    -- half circle: 32768*1125/65536 = 562, CW
    run(0, 32768, 562, '0', 61166);
    -- identical
    run(1234, 1234, 0, '0', 0, false);
    report "ALL TESTS PASSED";
    done <= true;
    wait;
  end process;
end architecture;

-- Self-checking testbench for StepCompute.
-- Compares the pipelined DUT against a behavioural reference (true division)
-- for every measured/target pair of a sweep covering the whole circle, with a
-- new angle on every clock cycle (checks the pipeline throughput as well).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity StepCompute_tb is
end entity StepCompute_tb;

architecture sim of StepCompute_tb is
  constant ROT_PER_STEP : integer := integer(0.32 / 360.0 * 2.0 ** 16);
  constant ROT_ACCURACY : integer := integer(0.01 / 360.0 * 2.0 ** 16);
  constant VMAX         : integer := 2 ** 16 * 140 / 150;
  constant LATENCY      : integer := 4;

  signal Clk      : std_logic := '0';
  signal Valid    : std_logic := '0';
  signal Meas     : unsigned(15 downto 0) := (others => '0');
  signal Targ     : unsigned(15 downto 0) := (others => '0');
  signal VMaxOut  : std_logic_vector(15 downto 0);
  signal Dir      : std_logic;
  signal Steps    : natural range 0 to 2000;
  signal OutValid : std_logic;
  signal Done     : boolean := false;

  -- expected results, queued by the stimulus process
  type exp_t is record
    steps : integer;
    dir   : std_logic;
    vmax  : integer;
  end record;
  type exp_arr is array (0 to 1023) of exp_t;
  signal Exp   : exp_arr;
  signal WrPtr : integer := 0;
  signal RdPtr : integer := 0;
  signal Errors : integer := 0;
  signal Checked : integer := 0;
begin
  Clk <= not Clk after 5 ns when not Done;

  dut : entity work.StepCompute
    port map (ClkxCI => Clk, ValidAnglexSI => Valid, AngleMeasuredxDI => Meas,
              AngleTargetxDI => Targ, VMaxExtxDO => VMaxOut, RotDirectionxSO => Dir,
              StepNumberxDO => Steps, OutputValidxSO => OutValid);

  stim : process
    variable m, t, d, diff : integer;
    variable e : exp_t;
    variable cnt : integer := 0;

    procedure apply(mm, tt : integer) is
    begin
      m := mm; t := tt;
      d := (t - m) mod 65536;
      if d < 32768 then diff := d; e.dir := '1'; else diff := 65536 - d; e.dir := '0'; end if;
      if diff > ROT_PER_STEP then
        e.steps := diff / ROT_PER_STEP; e.vmax := VMAX;
      elsif diff > ROT_ACCURACY then
        e.steps := 1; e.vmax := VMAX * diff / ROT_PER_STEP;  -- true division
      else
        e.steps := 0; e.vmax := -1;     -- VMax don't care
      end if;
      Exp(WrPtr mod 1024) <= e;
      Meas <= to_unsigned(mm, 16); Targ <= to_unsigned(tt, 16); Valid <= '1';
      WrPtr <= WrPtr + 1;
      wait until rising_edge(Clk);
      wait for 1 ns;
    end procedure;
  begin
    wait until rising_edge(Clk);
    wait for 1 ns;
    -- every difference 0..65535 from a few base angles (incl. wrap-around)
    for base in 0 to 3 loop
      for dd in 0 to 65535 loop
        case base is
          when 0 => apply(0, dd);
          when 1 => apply(65535, (65535 + dd) mod 65536);
          when 2 => apply(12345, (12345 + dd) mod 65536);
          when others => apply(40000, (40000 + dd) mod 65536);
        end case;
        -- sparse idle cycles to test non back-to-back pulses
        if dd mod 997 = 0 then
          Valid <= '0';
          wait until rising_edge(Clk); wait for 1 ns;
        end if;
      end loop;
    end loop;
    Valid <= '0';
    for i in 0 to 10 loop wait until rising_edge(Clk); end loop;
    report "Checked " & integer'image(Checked) & " results, errors = " & integer'image(Errors);
    assert Errors = 0 and Checked = 4 * 65536 report "TEST FAILED" severity failure;
    report "TEST PASSED";
    Done <= true;
    wait;
  end process;

  chk : process (Clk)
    variable e : exp_t;
    variable bad : boolean;
  begin
    if rising_edge(Clk) then
      if OutValid = '1' then
        e := Exp(RdPtr mod 1024);
        bad := false;
        if Steps /= e.steps then bad := true; end if;
        if e.steps /= 0 and Dir /= e.dir then bad := true; end if;
        if e.vmax >= 0 and abs(to_integer(unsigned(VMaxOut)) - e.vmax) > 1 then bad := true; end if;
        if to_integer(unsigned(VMaxOut)) > VMAX then bad := true; end if;
        if bad then
          Errors <= Errors + 1;
          report "Mismatch #" & integer'image(RdPtr) & ": steps=" & integer'image(Steps)
            & " exp " & integer'image(e.steps) & " vmax=" & integer'image(to_integer(unsigned(VMaxOut)))
            & " exp " & integer'image(e.vmax) & " dir=" & std_logic'image(Dir)
            & " exp " & std_logic'image(e.dir) severity error;
        end if;
        RdPtr <= RdPtr + 1;
        Checked <= Checked + 1;
      end if;
    end if;
  end process;
end architecture sim;

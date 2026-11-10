--------------------------------------------------------------------------------
-- Self-checking testbench for Deglitcher (160 MHz clock)
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

entity Deglitcher_tb is
  generic(
    PULSE_WIDTH    : natural := 10;
    RESYNCH_ENABLE : boolean := false
  );
end entity;

architecture sim of Deglitcher_tb is
  constant CLK_PERIOD : time := 6.25 ns;  -- 160 MHz

  function latency return natural is
  begin
    if RESYNCH_ENABLE then return PULSE_WIDTH + 3; else return PULSE_WIDTH + 1; end if;
  end function;

  signal Clk     : std_logic := '0';
  signal SigIn   : std_logic := '0';
  signal SigOut  : std_logic;
  signal Done    : boolean := false;
begin

  Clk <= not Clk after CLK_PERIOD / 2 when not Done;

  dut : entity work.Deglitcher
    generic map(
      DEFAULT_OUTPUT => '0',
      PULSE_WIDTH    => PULSE_WIDTH,
      RESYNCH_ENABLE => RESYNCH_ENABLE)
    port map(
      ClkxCI => Clk,
      SigxSI => SigIn,
      SigxSO => SigOut);

  stim : process
    -- drive a pulse of 'len' cycles of value 'v', then return to 'not v'
    -- and check whether the output ever reached 'v'
    procedure pulse(v : std_logic; len : natural; expect_pass : boolean) is
      variable seen : boolean := false;
    begin
      wait until falling_edge(Clk);
      SigIn <= v;
      for i in 1 to len loop
        wait until falling_edge(Clk);
      end loop;
      SigIn <= not v;
      for i in 1 to latency + 4 loop
        if SigOut = v then seen := true; end if;
        wait until falling_edge(Clk);
      end loop;
      assert seen = expect_pass
        report "Pulse '" & std_logic'image(v) & "' of " & natural'image(len) &
               " cycles: expected pass=" & boolean'image(expect_pass)
        severity error;
      assert SigOut = not v report "Output did not return" severity error;
    end procedure;

    variable cnt : natural;
  begin
    -- Initial state
    for i in 1 to latency + 2 loop wait until falling_edge(Clk); end loop;
    assert SigOut = '0' report "Wrong default output" severity error;

    -- High pulses on low background
    pulse('1', 1, PULSE_WIDTH = 0);
    pulse('1', PULSE_WIDTH, false);
    pulse('1', PULSE_WIDTH + 1, true);
    pulse('1', PULSE_WIDTH + 5, true);

    -- Move to high level, then low pulses on high background
    SigIn <= '1';
    for i in 1 to latency + 2 loop wait until falling_edge(Clk); end loop;
    assert SigOut = '1' report "Output did not go high" severity error;
    pulse('0', 1, PULSE_WIDTH = 0);
    pulse('0', PULSE_WIDTH, false);
    pulse('0', PULSE_WIDTH + 1, true);

    -- Latency check on a clean falling edge
    wait until falling_edge(Clk);
    SigIn <= '0';
    cnt := 0;
    while SigOut /= '0' loop
      wait until falling_edge(Clk);
      cnt := cnt + 1;
    end loop;
    assert cnt = latency
      report "Latency " & natural'image(cnt) & " expected " & natural'image(latency)
      severity error;

    report "Deglitcher_tb finished (PULSE_WIDTH=" & natural'image(PULSE_WIDTH) &
           ", RESYNCH_ENABLE=" & boolean'image(RESYNCH_ENABLE) & ")" severity note;
    Done <= true;
    wait;
  end process;

end architecture;

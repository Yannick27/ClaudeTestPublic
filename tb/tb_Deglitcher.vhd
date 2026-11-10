--------------------------------------------------------------------------------
-- Self-checking testbench for Deglitcher
--
-- For several PULSE_WIDTH values, with and without resynchronization, checks
-- that pulses of PULSE_WIDTH - 1 cycles are rejected and pulses of
-- PULSE_WIDTH cycles are passed, for both polarities and both default levels.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

entity tb_Deglitcher_Unit is
  generic(
    DEFAULT_OUTPUT : std_logic := '0';
    PULSE_WIDTH    : natural   := 10;
    RESYNCH_ENABLE : boolean   := false
  );
  port(
    DonexSO : out boolean := false
  );
end entity;

architecture sim of tb_Deglitcher_Unit is
  constant CLK_PERIOD : time := 6.25 ns;  -- 160 MHz

  signal ClkxC   : std_logic := '0';
  signal SigxS   : std_logic := DEFAULT_OUTPUT;
  signal OutxS   : std_logic;
  signal DonexS  : boolean := false;

  function EffWidth return positive is
  begin
    if PULSE_WIDTH = 0 then
      return 1;
    end if;
    return PULSE_WIDTH;
  end function;

  function SyncDelay return natural is
  begin
    if RESYNCH_ENABLE then
      return 2;
    end if;
    return 0;
  end function;
begin

  ClkxC <= not ClkxC after CLK_PERIOD / 2 when not DonexS;
  DonexSO <= DonexS;

  iDut : entity work.Deglitcher
    generic map(
      DEFAULT_OUTPUT => DEFAULT_OUTPUT,
      PULSE_WIDTH    => PULSE_WIDTH,
      RESYNCH_ENABLE => RESYNCH_ENABLE
    )
    port map(
      ClkxCI => ClkxC,
      SigxSI => SigxS,
      SigxSO => OutxS
    );

  pStim : process
    -- Drive the input to Level for N clock cycles (changes on falling edges)
    procedure Drive(Level : std_logic; N : natural) is
    begin
      SigxS <= Level;
      for i in 1 to N loop
        wait until falling_edge(ClkxC);
      end loop;
    end procedure;

    procedure Check(Expected : std_logic; Msg : string) is
    begin
      assert OutxS = Expected
        report "PULSE_WIDTH=" & natural'image(PULSE_WIDTH) & ": " & Msg
        severity failure;
    end procedure;

    variable Idle : std_logic := DEFAULT_OUTPUT;
  begin
    wait until falling_edge(ClkxC);
    Check(DEFAULT_OUTPUT, "wrong default output");

    for Pass in 1 to 2 loop
      -- Pulses too short must be filtered out, even several in a row
      for Len in 1 to EffWidth - 1 loop
        Drive(not Idle, Len);
        SigxS <= Idle;
        for i in 1 to EffWidth + SyncDelay + 2 loop
          wait until falling_edge(ClkxC);
          Check(Idle, "short pulse not filtered");
        end loop;
      end loop;

      for i in 1 to 3 loop
        Drive(not Idle, EffWidth - 1);
        Drive(Idle, 1);
      end loop;
      Drive(Idle, SyncDelay + 2);
      Check(Idle, "burst of short pulses not filtered");

      -- A change lasting PULSE_WIDTH cycles must go through, after exactly
      -- PULSE_WIDTH (+ synchronizer) cycles
      Drive(not Idle, EffWidth + SyncDelay - 1);
      Check(Idle, "output changed too early");
      Drive(not Idle, 1);
      Check(not Idle, "output did not change");

      -- A pulse of exactly PULSE_WIDTH cycles is passed with the same width
      -- (output is now at "not Idle"; the pulse goes back to Idle)
      for k in 1 to 2 * EffWidth + SyncDelay + 2 loop
        if k <= EffWidth then
          Drive(Idle, 1);
        else
          Drive(not Idle, 1);
        end if;
        if k >= EffWidth + SyncDelay and k < 2 * EffWidth + SyncDelay then
          Check(Idle, "minimum width pulse not passed");
        else
          Check(not Idle, "minimum width pulse has wrong timing");
        end if;
      end loop;
      Drive(not Idle, 5);

      -- Second pass: same checks with the opposite idle level
      Idle := not Idle;
    end loop;

    DonexS <= true;
    wait;
  end process;

end architecture;

--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

entity tb_Deglitcher is
end entity;

architecture sim of tb_Deglitcher is
  type NaturalArray is array (natural range <>) of natural;
  constant WIDTHS : NaturalArray := (0, 1, 2, 3, 4, 5, 8, 9, 10, 16, 17, 100);

  signal DonexS : boolean_vector(0 to 4 * WIDTHS'length - 1);
begin

  gWidth : for i in WIDTHS'range generate
    iA : entity work.tb_Deglitcher_Unit
      generic map('0', WIDTHS(i), false) port map(DonexS(4 * i));
    iB : entity work.tb_Deglitcher_Unit
      generic map('1', WIDTHS(i), false) port map(DonexS(4 * i + 1));
    iC : entity work.tb_Deglitcher_Unit
      generic map('0', WIDTHS(i), true) port map(DonexS(4 * i + 2));
    iD : entity work.tb_Deglitcher_Unit
      generic map('1', WIDTHS(i), true) port map(DonexS(4 * i + 3));
  end generate;

  pEnd : process
  begin
    wait until (and DonexS);
    report "tb_Deglitcher: all tests passed";
    wait;
  end process;

end architecture;

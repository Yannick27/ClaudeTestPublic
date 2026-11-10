--------------------------------------------------------------------------------
-- Deglitcher
--
-- Filters out pulses (glitches) on a single-bit signal. The output only
-- follows the input once the input has held a new value for more than
-- PULSE_WIDTH consecutive clock cycles:
--   * pulses of 1 .. PULSE_WIDTH clock cycles are suppressed
--   * a level held for PULSE_WIDTH+1 cycles is passed to the output
--
-- Latency (input edge -> output edge):
--   PULSE_WIDTH + 1 cycles            (RESYNCH_ENABLE = false)
--   PULSE_WIDTH + 3 cycles            (RESYNCH_ENABLE = true)
--
-- Target: Microchip RTG4 @ 160 MHz.
--   * The only logic in the critical path is a small counter (ceil(log2(
--     PULSE_WIDTH+1)) bits), an equality compare against a constant and a
--     2:1 mux, well within a 6.25 ns period.
--   * Output is registered (glitch free).
--   * RTG4 flip-flops are SET-hardened (built-in TMR), no extra mitigation
--     is required here.
--
-- Notes:
--   * The entity has no reset port. Power-up values are given through signal
--     initialisation. Check that your Libero/Synplify flow honours register
--     initial values on RTG4; if it does not, the output may start at an
--     undefined/'0' value and will settle to the input value after
--     PULSE_WIDTH+1 (+2) cycles of stable input.
--   * Set RESYNCH_ENABLE = true whenever SigxSI is asynchronous to ClkxCI
--     (e.g. an external pin). With RESYNCH_ENABLE = false the input must be
--     synchronous to ClkxCI.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

entity Deglitcher is
  generic(
    DEFAULT_OUTPUT : std_logic := '0';  -- default value of the output
    PULSE_WIDTH    : natural   := 10;   -- pulses equal to or shorter than PULSE_WIDTH clock cycles will be filtered out
    RESYNCH_ENABLE : boolean   := false -- synchronize input with a double flip flop before deglitching
  );
  port(
    ClkxCI : in  std_logic;                     -- Clock
    SigxSI : in  std_logic;                     -- Signal in
    SigxSO : out std_logic := DEFAULT_OUTPUT    -- signal out (glitch free)
  );
end entity;

architecture rtl of Deglitcher is

  -- Synchronised (or pass-through) version of the input
  signal SigSyncxS : std_logic;

  -- Registered output
  signal SigxDP : std_logic := DEFAULT_OUTPUT;

  -- Number of consecutive cycles the input has differed from the output
  signal CntxDP : natural range 0 to PULSE_WIDTH := 0;

begin

  ------------------------------------------------------------------------------
  -- Optional double flip-flop synchroniser
  ------------------------------------------------------------------------------
  gen_resynch : if RESYNCH_ENABLE generate
    signal Sync1xDP : std_logic := DEFAULT_OUTPUT;
    signal Sync2xDP : std_logic := DEFAULT_OUTPUT;

    -- Keep the synchroniser registers intact (no retiming / merging / SRL)
    attribute syn_preserve : boolean;
    attribute syn_preserve of Sync1xDP : signal is true;
    attribute syn_preserve of Sync2xDP : signal is true;
  begin
    p_sync : process(ClkxCI)
    begin
      if rising_edge(ClkxCI) then
        Sync1xDP <= SigxSI;
        Sync2xDP <= Sync1xDP;
      end if;
    end process;

    SigSyncxS <= Sync2xDP;
  end generate;

  gen_no_resynch : if not RESYNCH_ENABLE generate
    SigSyncxS <= SigxSI;
  end generate;

  ------------------------------------------------------------------------------
  -- Deglitch: output changes only after the input has differed from it for
  -- PULSE_WIDTH+1 consecutive clock cycles.
  ------------------------------------------------------------------------------
  p_deglitch : process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      if SigSyncxS = SigxDP then
        -- Input agrees with output: any pending glitch is discarded
        CntxDP <= 0;
      elsif CntxDP = PULSE_WIDTH then
        -- Input has been stable at the new value for PULSE_WIDTH+1 cycles
        SigxDP <= SigSyncxS;
        CntxDP <= 0;
      else
        CntxDP <= CntxDP + 1;
      end if;
    end if;
  end process;

  SigxSO <= SigxDP;

end architecture;

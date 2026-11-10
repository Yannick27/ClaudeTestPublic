--------------------------------------------------------------------------------
-- Deglitcher
--
-- Filters out pulses (of either polarity) equal to or shorter than
-- PULSE_WIDTH clock cycles. The output only changes once the input has been
-- seen at the new level for PULSE_WIDTH + 1 consecutive rising clock edges,
-- so the output is delayed by PULSE_WIDTH + 1 clock cycles (plus 2 when
-- RESYNCH_ENABLE is true). PULSE_WIDTH = 0 means no filtering (the signal is
-- only registered).
--
-- Target: Microchip RTG4, 160 MHz clock.
--   * There is no reset and RTG4 flip-flops do not load VHDL initial values
--     at power-up (the initial values below are for simulation only). The
--     logic is therefore self-initialising: from any register state, the
--     output is consistent with a stable input after at most
--     2 * PULSE_WIDTH + 2 clock cycles (plus 2 when RESYNCH_ENABLE is true).
--   * The end of the count is detected on the counter sign bit instead of a
--     wide comparator, so the critical path is only the carry chain of the
--     down counter, whatever the value of PULSE_WIDTH.
--   * With RESYNCH_ENABLE = false, SigxSI must already be synchronous to
--     ClkxCI.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity Deglitcher is
  generic(
    DEFAULT_OUTPUT : std_logic := '0';  -- default value of the output
    PULSE_WIDTH : natural := 10;  -- pulses equal to or shorter than PULSE_WIDTH clock cycles will be filtered out
    RESYNCH_ENABLE : boolean   := false -- synchronize input with a double flip flop before deglitching
  );
  port(
    ClkxCI        : in  std_logic;      -- Clock
    SigxSI        : in  std_logic;      -- Signal in
    SigxSO        : out std_logic := DEFAULT_OUTPUT      -- signal out (glitch free)
  );
end entity;

architecture rtl of Deglitcher is

  -- Number of bits needed to code the unsigned value V (at least 1)
  function NumBits(V : natural) return positive is
    variable Tmp : natural  := V;
    variable N   : positive := 1;
  begin
    while Tmp > 1 loop
      Tmp := Tmp / 2;
      N   := N + 1;
    end loop;
    return N;
  end function;

  function Max(A, B : integer) return integer is
  begin
    if A > B then
      return A;
    end if;
    return B;
  end function;

  -- The counter runs from CNT_RELOAD down to -1. It reaches -1 (sign bit set)
  -- when the input has differed from the output for PULSE_WIDTH samples; the
  -- (PULSE_WIDTH + 1)-th differing sample then updates the output.
  -- With PULSE_WIDTH = 0 the counter stays at -1 (no filtering).
  constant CNT_RELOAD : integer  := PULSE_WIDTH - 1;
  constant CNT_WIDTH  : positive := NumBits(Max(CNT_RELOAD, 0)) + 1;  -- + sign

  signal SyncxSP : std_logic_vector(1 downto 0) := (others => DEFAULT_OUTPUT);
  signal SigxS   : std_logic;
  signal CntxDP  : signed(CNT_WIDTH - 1 downto 0) := to_signed(CNT_RELOAD, CNT_WIDTH);
  signal OutxSP  : std_logic := DEFAULT_OUTPUT;

  -- Keep the synchronizer as two discrete flip-flops (no merging, no
  -- retiming, no replication)
  attribute syn_preserve : boolean;
  attribute syn_preserve of SyncxSP : signal is true;

begin

  ------------------------------------------------------------------------------
  -- Optional input resynchronization
  ------------------------------------------------------------------------------
  gResync : if RESYNCH_ENABLE generate
    pSync : process(ClkxCI)
    begin
      if rising_edge(ClkxCI) then
        SyncxSP <= SyncxSP(0) & SigxSI;
      end if;
    end process;

    SigxS <= SyncxSP(1);
  end generate;

  gNoResync : if not RESYNCH_ENABLE generate
    SigxS <= SigxSI;
  end generate;

  ------------------------------------------------------------------------------
  -- Deglitcher
  ------------------------------------------------------------------------------
  pDeglitch : process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      if SigxS = OutxSP then
        -- Input agrees with the output: (re)arm the counter
        CntxDP <= to_signed(CNT_RELOAD, CNT_WIDTH);
      elsif CntxDP(CntxDP'high) = '1' then
        -- Input stable at the new level for more than PULSE_WIDTH cycles:
        -- accept it
        OutxSP <= SigxS;
        CntxDP <= to_signed(CNT_RELOAD, CNT_WIDTH);
      else
        CntxDP <= CntxDP - 1;
      end if;
    end if;
  end process;

  SigxSO <= OutxSP;

end architecture;

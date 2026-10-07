-------------------------------------------------------------------------------
-- PairNormalizer
--
--   Block-floating-point scaling of a pair of signed words (X, Y): both are
--   shifted left by the same number of bits, one bit per clock, until at least one
--   of them has two different top bits (i.e. no further shift is possible without
--   overflow).  The WOUT most significant bits of each word are then output.
--
--   Only the ratio / angle of (X, Y) matters downstream, so the common scale factor
--   is irrelevant; what is gained is that the precision of everything that follows
--   no longer depends on the magnitude of the inputs.
--
--   Result : max(|X|,|Y|) is in [2**(WOUT-2), 2**(WOUT-1)) (the other word is
--            scaled identically and truncated, not rounded).
--   ZeroxSO: '1' when X = Y = 0 (outputs are then 0 as well).
--
--   Handshake : StartxSI = '1' for one clock with XxDI/YxDI stable during that
--   clock.  DonexSO = '1' for one clock when XxDO/YxDO/ZeroxSO are valid; they stay
--   valid until the next start.  Duration: (number of shifts, at most WIN-1) + 2
--   clocks.
--
--   Timing : the decision "shift once more" is registered (CanShift, MoreShifts) so
--   that the clock enable of the two WIN-bit shift registers is a 3-input function
--   of registers.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity PairNormalizer is
  generic(
    WIN  : positive range 3 to 256 := 72;  -- input word width
    WOUT : positive := 35                  -- output word width (<= WIN)
  );
  port(
    ClkxCI   : in  std_logic;
    StartxSI : in  std_logic;
    XxDI     : in  signed(WIN-1 downto 0);
    YxDI     : in  signed(WIN-1 downto 0);
    DonexSO  : out std_logic := '0';
    ZeroxSO  : out std_logic := '0';
    XxDO     : out signed(WOUT-1 downto 0);
    YxDO     : out signed(WOUT-1 downto 0)
  );
end entity PairNormalizer;

architecture rtl of PairNormalizer is

  signal X    : signed(WIN-1 downto 0) := (others => '0');
  signal Y    : signed(WIN-1 downto 0) := (others => '0');
  signal Cnt  : natural range 0 to WIN-1 := 0;   -- shifts done so far
  signal Busy : std_logic := '0';

  -- registered shift decision:
  signal CanShift    : std_logic := '0';  -- X and Y both have two equal top bits
  signal MoreShifts  : std_logic := '0';  -- Cnt /= WIN-1

begin

  assert WOUT <= WIN report "PairNormalizer: WOUT must be <= WIN" severity failure;

  process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      DonexSO <= '0';
      if StartxSI = '1' then
        X          <= XxDI;
        Y          <= YxDI;
        Cnt        <= 0;
        Busy       <= '1';
        MoreShifts <= '1';
        if XxDI(WIN-1) = XxDI(WIN-2) and YxDI(WIN-1) = YxDI(WIN-2) then
          CanShift <= '1';
        else
          CanShift <= '0';
        end if;
      elsif Busy = '1' then
        if CanShift = '1' and MoreShifts = '1' then
          X   <= shift_left(X, 1);
          Y   <= shift_left(Y, 1);
          Cnt <= Cnt + 1;
          if Cnt = WIN-2 then
            MoreShifts <= '0';
          end if;
          -- top bit pair after this shift = bits (WIN-2, WIN-3) of the current word
          if X(WIN-2) = X(WIN-3) and Y(WIN-2) = Y(WIN-3) then
            CanShift <= '1';
          else
            CanShift <= '0';
          end if;
        else
          -- Still shiftable after WIN-1 shifts: both words are zero.
          ZeroxSO <= CanShift;
          Busy    <= '0';
          DonexSO <= '1';
        end if;
      end if;
    end if;
  end process;

  XxDO <= X(WIN-1 downto WIN-WOUT);
  YxDO <= Y(WIN-1 downto WIN-WOUT);

end architecture rtl;

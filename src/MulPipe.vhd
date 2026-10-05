-------------------------------------------------------------------------------
-- MulPipe : 32x32 -> 64 bit signed multiplier, fully pipelined (1 product per
--           clock, latency 5 clocks).
--
-- The operands are split in 16-bit halves so that every partial product is a
-- (at most) 17x17 signed multiplication, i.e. fits exactly in one RTG4 math
-- block (18x18) with its input and output registers. The partial products are
-- summed over several stages with adders of at most 34 bits to meet 100 MHz.
--
--   A = AH*2^16 + AL  (AH signed, AL unsigned)     B = BH*2^16 + BL
--   A*B = AH*BH*2^32 + (AH*BL + AL*BH)*2^16 + AL*BL
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity MulPipe is
  port(
    ClkxCI   : in  std_logic;
    ValidxSI : in  std_logic;                       -- operands valid
    AxDI     : in  signed(31 downto 0);
    BxDI     : in  signed(31 downto 0);
    ValidxSO : out std_logic             := '0';    -- product valid (5 clocks later)
    ProdxDO  : out signed(63 downto 0)   := (others => '0')
  );
end entity MulPipe;

architecture rtl of MulPipe is

  signal ValidxDP : std_logic_vector(1 to 5) := (others => '0');

  -- stage 1 : registered operands
  signal AHxDP, BHxDP : signed(15 downto 0)   := (others => '0');
  signal ALxDP, BLxDP : unsigned(15 downto 0) := (others => '0');
  -- stage 2 : partial products
  signal HHxDP : signed(31 downto 0)   := (others => '0');
  signal HLxDP : signed(32 downto 0)   := (others => '0');
  signal LHxDP : signed(32 downto 0)   := (others => '0');
  signal LLxDP : unsigned(31 downto 0) := (others => '0');
  -- stage 3 : middle term
  signal MidxDP : signed(33 downto 0)   := (others => '0');
  signal HH2xDP : signed(31 downto 0)   := (others => '0');
  signal LL2xDP : unsigned(31 downto 0) := (others => '0');
  -- stage 4 : low word and carry
  signal LowxDP   : unsigned(32 downto 0) := (others => '0');
  signal MidHixDP : signed(17 downto 0)   := (others => '0');
  signal HH3xDP   : signed(31 downto 0)   := (others => '0');

begin

  process(ClkxCI)
    variable High_v : signed(31 downto 0);
  begin
    if rising_edge(ClkxCI) then
      -- stage 1
      AHxDP <= AxDI(31 downto 16);
      ALxDP <= unsigned(AxDI(15 downto 0));
      BHxDP <= BxDI(31 downto 16);
      BLxDP <= unsigned(BxDI(15 downto 0));
      -- stage 2
      HHxDP <= AHxDP * BHxDP;
      HLxDP <= AHxDP * signed('0' & BLxDP);
      LHxDP <= signed('0' & ALxDP) * BHxDP;
      LLxDP <= ALxDP * BLxDP;
      -- stage 3
      MidxDP <= resize(HLxDP, 34) + resize(LHxDP, 34);
      HH2xDP <= HHxDP;
      LL2xDP <= LLxDP;
      -- stage 4
      LowxDP   <= resize(LL2xDP, 33) +
                  shift_left(resize(unsigned(MidxDP(15 downto 0)), 33), 16);
      MidHixDP <= MidxDP(33 downto 16);
      HH3xDP   <= HH2xDP;
      -- stage 5
      High_v := HH3xDP + resize(MidHixDP, 32);
      if LowxDP(32) = '1' then
        High_v := High_v + 1;
      end if;
      ProdxDO <= High_v & signed(LowxDP(31 downto 0));

      ValidxDP <= ValidxSI & ValidxDP(1 to 4);
    end if;
  end process;

  ValidxSO <= ValidxDP(5);

end architecture rtl;

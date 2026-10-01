-------------------------------------------------------------------------------
-- Mul32 : pipelined 32x32 signed multiplier, built from four 17x17 signed
--         partial products so that each one maps onto a single RTG4 math
--         block (18x18).  Latency: DonexSO is high 4 clocks after StartxSI.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity Mul32 is
  port(
    ClkxCI   : in  std_logic;
    StartxSI : in  std_logic;           -- 1-clock pulse, operands sampled now
    AxDI     : in  signed(31 downto 0);
    BxDI     : in  signed(31 downto 0);
    DonexSO  : out std_logic := '0';    -- pulse, PxDO valid (held until next op)
    PxDO     : out signed(63 downto 0) := (others => '0')
  );
end entity Mul32;

architecture rtl of Mul32 is
  signal A_r, B_r     : signed(31 downto 0) := (others => '0');
  signal Hh, Hl, Lh, Ll : signed(33 downto 0) := (others => '0');
  signal X            : signed(63 downto 0) := (others => '0');
  signal Y            : signed(35 downto 0) := (others => '0');
  signal Valid        : std_logic_vector(3 downto 0) := (others => '0');
begin

  process(ClkxCI)
    variable Ah, Bh, Al, Bl : signed(16 downto 0);
  begin
    if rising_edge(ClkxCI) then
      -- stage 1 : operand registers
      A_r      <= AxDI;
      B_r      <= BxDI;
      Valid(0) <= StartxSI;

      -- stage 2 : partial products (signed high half, unsigned low half)
      Ah := resize(A_r(31 downto 16), 17);
      Bh := resize(B_r(31 downto 16), 17);
      Al := signed('0' & A_r(15 downto 0));
      Bl := signed('0' & B_r(15 downto 0));
      Hh       <= Ah * Bh;
      Hl       <= Ah * Bl;
      Lh       <= Al * Bh;
      Ll       <= Al * Bl;
      Valid(1) <= Valid(0);

      -- stage 3 : partial sums
      X        <= shift_left(resize(Hh, 64), 32) + resize(Ll, 64);
      Y        <= resize(Hl, 36) + resize(Lh, 36);
      Valid(2) <= Valid(1);

      -- stage 4 : final sum
      PxDO     <= X + shift_left(resize(Y, 64), 16);
      Valid(3) <= Valid(2);
    end if;
  end process;

  DonexSO <= Valid(3);

end architecture rtl;

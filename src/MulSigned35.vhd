-------------------------------------------------------------------------------
-- MulSigned35
--
--   Pipelined signed 35 x 35 -> 70 bit multiplier built from four 18 x 18
--   partial products, i.e. four RTG4 math blocks (MACC), with every adder
--   limited to 37 bits so that it closes timing at 100 MHz in the fabric.
--
--   Each operand is split as  A = A1 * 2**17 + A0
--     A0 = A(16 downto 0)   unsigned, kept non-negative in an 18-bit signed word
--     A1 = A(34 downto 17)  signed 18 bit
--   so that  A*B = A0*B0 + (A0*B1 + A1*B0) * 2**17 + A1*B1 * 2**34.
--
--   Handshake : StartxSI = '1' for one clock with AxDI/BxDI stable during that
--   clock.  DonexSO = '1' exactly LATENCY (= 5) clocks after the clock in which
--   StartxSI was high; PxDO is valid in that clock and stays valid until the next
--   operation (every pipeline stage only loads when it holds valid data, which maps to
--   the clock enables of the RTG4 flip-flops and math block registers).  A new
--   operation can be started every clock (fully pipelined).
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity MulSigned35 is
  port(
    ClkxCI   : in  std_logic;
    StartxSI : in  std_logic;
    AxDI     : in  signed(34 downto 0);
    BxDI     : in  signed(34 downto 0);
    DonexSO  : out std_logic;
    PxDO     : out signed(69 downto 0)
  );
end entity MulSigned35;

architecture rtl of MulSigned35 is

  -- stage 1: operand chunks
  signal A0, A1, B0, B1 : signed(17 downto 0) := (others => '0');
  -- stage 2: partial products
  signal PP00, PP01, PP10, PP11 : signed(35 downto 0) := (others => '0');
  -- stage 3
  signal Mid3 : signed(36 downto 0)   := (others => '0');  -- PP01 + PP10
  signal Lo3  : unsigned(33 downto 0) := (others => '0');  -- PP00 (always >= 0)
  signal Hi3  : signed(35 downto 0)   := (others => '0');  -- PP11
  -- stage 4
  signal Mid4 : signed(36 downto 0)   := (others => '0');  -- Mid3 + PP00 / 2**17
  signal Lo4  : unsigned(16 downto 0) := (others => '0');  -- PP00(16 downto 0)
  signal Hi4  : signed(35 downto 0)   := (others => '0');
  -- stage 5
  signal Hi5  : signed(35 downto 0)   := (others => '0');  -- PP11 + Mid4 / 2**17
  signal Mid5 : unsigned(16 downto 0) := (others => '0');  -- Mid4(16 downto 0)
  signal Lo5  : unsigned(16 downto 0) := (others => '0');

  signal Vld  : std_logic_vector(4 downto 0) := (others => '0');

begin

  -- Every stage only loads when the stage before it holds valid data (clock enable),
  -- so the result stays stable until the next operation.
  process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      -- 1: split operands
      if StartxSI = '1' then
        A0 <= signed('0' & AxDI(16 downto 0));
        A1 <= AxDI(34 downto 17);
        B0 <= signed('0' & BxDI(16 downto 0));
        B1 <= BxDI(34 downto 17);
      end if;

      -- 2: four 18x18 multiplications (one math block each)
      if Vld(0) = '1' then
        PP00 <= A0 * B0;
        PP01 <= A0 * B1;
        PP10 <= A1 * B0;
        PP11 <= A1 * B1;
      end if;

      -- 3: middle column
      if Vld(1) = '1' then
        Mid3 <= resize(PP01, 37) + resize(PP10, 37);
        Lo3  <= unsigned(PP00(33 downto 0));
        Hi3  <= PP11;
      end if;

      -- 4: carry of the low column into the middle column
      if Vld(2) = '1' then
        Mid4 <= Mid3 + signed(resize(Lo3(33 downto 17), 37));
        Lo4  <= Lo3(16 downto 0);
        Hi4  <= Hi3;
      end if;

      -- 5: carry of the middle column into the high column
      if Vld(3) = '1' then
        Hi5  <= Hi4 + resize(Mid4(36 downto 17), 36);
        Mid5 <= unsigned(Mid4(16 downto 0));
        Lo5  <= Lo4;
      end if;

      Vld <= Vld(3 downto 0) & StartxSI;
    end if;
  end process;

  PxDO    <= signed(std_logic_vector(Hi5) & std_logic_vector(Mid5) & std_logic_vector(Lo5));
  DonexSO <= Vld(4);

end architecture rtl;

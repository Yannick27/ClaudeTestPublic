-- Pipelined signed multiplier, 3 clock cycles of latency.
--
-- Operand registers -> product register -> output register. Written as plain
-- "a * b" between registers so that the synthesis tool can pack the operand /
-- product registers into the RTG4 math blocks (MACC, 18x18) and cascade
-- several of them for operands wider than 18 bits.
-- A valid flag travels alongside the data so users do not have to count cycles.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity MulSigned is
  generic(
    WA : positive := 32;
    WB : positive := 32
  );
  port(
    ClkxCI   : in  std_logic;
    ValidxSI : in  std_logic;                            -- operands valid in this cycle
    AxDI     : in  signed(WA-1 downto 0);
    BxDI     : in  signed(WB-1 downto 0);
    ValidxSO : out std_logic := '0';                     -- product valid, 3 cycles after ValidxSI
    ProdxDO  : out signed(WA+WB-1 downto 0)
  );
end entity MulSigned;

architecture rtl of MulSigned is
  signal AxD     : signed(WA-1 downto 0);
  signal BxD     : signed(WB-1 downto 0);
  signal Prod1xD : signed(WA+WB-1 downto 0);
  signal Prod2xD : signed(WA+WB-1 downto 0);
  signal VldxS   : std_logic_vector(1 to 3) := (others => '0');
begin

  process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      AxD     <= AxDI;
      BxD     <= BxDI;
      Prod1xD <= AxD * BxD;
      Prod2xD <= Prod1xD;
      VldxS   <= ValidxSI & VldxS(1 to 2);
    end if;
  end process;

  ValidxSO <= VldxS(3);
  ProdxDO  <= Prod2xD;

end architecture rtl;

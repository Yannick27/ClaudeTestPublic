-------------------------------------------------------------------------------
-- BlockNormalizer : block floating point normalization of two signed values.
--
-- Both values A and B are shifted left by the same number of bits, as long as
-- both keep their sign (i.e. as long as the two MSBs of both are identical),
-- then truncated to their OUT_W most significant bits. The ratio A/B is thus
-- preserved while the largest value uses the full OUT_W bits.
-- One shift per clock, so it takes at most IN_W+2 clocks.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity BlockNormalizer is
  generic(
    IN_W  : positive := 41;
    OUT_W : positive := 32               -- must be <= IN_W
  );
  port(
    ClkxCI   : in  std_logic;
    StartxSI : in  std_logic;            -- AxDI/BxDI are sampled when idle and StartxSI = 1
    AxDI     : in  signed(IN_W - 1 downto 0);
    BxDI     : in  signed(IN_W - 1 downto 0);
    DonexSO  : out std_logic                       := '0';  -- 1 clock pulse; outputs are then valid
    AxDO     : out signed(OUT_W - 1 downto 0)      := (others => '0');
    BxDO     : out signed(OUT_W - 1 downto 0)      := (others => '0')
  );
end entity BlockNormalizer;

architecture rtl of BlockNormalizer is
  type State_t is (S_IDLE, S_RUN);
  signal StatexDP : State_t := S_IDLE;
  signal AxDP     : signed(IN_W - 1 downto 0) := (others => '0');
  signal BxDP     : signed(IN_W - 1 downto 0) := (others => '0');
  signal CntxDP   : integer range 0 to IN_W   := 0;
begin

  assert OUT_W <= IN_W and IN_W >= 2
    report "BlockNormalizer: OUT_W must be <= IN_W" severity failure;

  process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      DonexSO <= '0';
      case StatexDP is
        when S_IDLE =>
          if StartxSI = '1' then
            AxDP     <= AxDI;
            BxDP     <= BxDI;
            CntxDP   <= 0;
            StatexDP <= S_RUN;
          end if;

        when S_RUN =>
          if AxDP(IN_W - 1) = AxDP(IN_W - 2) and
             BxDP(IN_W - 1) = BxDP(IN_W - 2) and CntxDP < IN_W - 1 then
            AxDP   <= shift_left(AxDP, 1);
            BxDP   <= shift_left(BxDP, 1);
            CntxDP <= CntxDP + 1;
          else
            AxDO     <= AxDP(IN_W - 1 downto IN_W - OUT_W);
            BxDO     <= BxDP(IN_W - 1 downto IN_W - OUT_W);
            DonexSO  <= '1';
            StatexDP <= S_IDLE;
          end if;
      end case;
    end if;
  end process;

end architecture rtl;

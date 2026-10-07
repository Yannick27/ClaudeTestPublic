-------------------------------------------------------------------------------
-- WideAddSub
--
--   S = A + B   (SubxSI = '0')   or   S = A - B   (SubxSI = '1'),   W bits.
--
--   The carry chain is cut in two halves of about W/2 bits that are processed in
--   two consecutive clocks, so no adder is wider than ~36 bits for the 72-bit
--   sums of AngleCompute (RTG4 fabric at 100 MHz).  No overflow handling: the
--   result is the exact W-bit two's complement sum.
--
--   Handshake : StartxSI = '1' for one clock with AxDI/BxDI/SubxSI stable during
--   that clock.  DonexSO = '1' two clocks later with SxDO valid.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity WideAddSub is
  generic(
    W : positive range 2 to 128 := 72
  );
  port(
    ClkxCI   : in  std_logic;
    StartxSI : in  std_logic;
    SubxSI   : in  std_logic;
    AxDI     : in  signed(W-1 downto 0);
    BxDI     : in  signed(W-1 downto 0);
    DonexSO  : out std_logic;
    SxDO     : out signed(W-1 downto 0)
  );
end entity WideAddSub;

architecture rtl of WideAddSub is

  constant LO_W : positive := (W + 1) / 2;
  constant HI_W : positive := W - LO_W;

  -- stage 1 results
  signal LoSum1 : unsigned(LO_W-1 downto 0) := (others => '0');
  signal Carry1 : std_logic                 := '0';
  signal AHi1   : signed(HI_W-1 downto 0)   := (others => '0');
  signal BHi1   : signed(HI_W-1 downto 0)   := (others => '0');  -- already inverted for A-B
  -- stage 2 results
  signal LoSum2 : unsigned(LO_W-1 downto 0) := (others => '0');
  signal HiSum2 : signed(HI_W-1 downto 0)   := (others => '0');

  signal Vld    : std_logic_vector(1 downto 0) := (others => '0');

begin

  -- Both stages only load when they hold valid data (clock enable): SxDO stays stable
  -- until the next operation.
  process(ClkxCI)
    variable BLo : unsigned(LO_W-1 downto 0);
    variable BHi : signed(HI_W-1 downto 0);
    variable Cin : std_logic;
    variable SLo : unsigned(LO_W+1 downto 0);   -- {carry, sum, 1}
    variable SHi : signed(HI_W downto 0);       -- {sum, 1}
  begin
    if rising_edge(ClkxCI) then
      -- 1: low half.  A - B = A + not(B) + 1 : the +1 enters as carry in.
      if StartxSI = '1' then
        BLo := unsigned(BxDI(LO_W-1 downto 0));
        BHi := BxDI(W-1 downto LO_W);
        Cin := '0';
        if SubxSI = '1' then
          BLo := not BLo;
          BHi := not BHi;
          Cin := '1';
        end if;
        -- A + B + Cin as one adder: (A,1) + (B,Cin) = 2*(A+B+Cin) + 1 (+1 if Cin)
        SLo    := ('0' & unsigned(AxDI(LO_W-1 downto 0)) & '1') + ('0' & BLo & Cin);
        LoSum1 <= SLo(LO_W downto 1);
        Carry1 <= SLo(LO_W+1);
        AHi1   <= AxDI(W-1 downto LO_W);
        BHi1   <= BHi;
      end if;

      -- 2: high half with the registered carry of the low half
      if Vld(0) = '1' then
        SHi    := (AHi1 & '1') + (BHi1 & Carry1);
        HiSum2 <= SHi(HI_W downto 1);
        LoSum2 <= LoSum1;
      end if;

      Vld <= Vld(0) & StartxSI;
    end if;
  end process;

  SxDO    <= signed(std_logic_vector(HiSum2) & std_logic_vector(LoSum2));
  DonexSO <= Vld(1);

end architecture rtl;

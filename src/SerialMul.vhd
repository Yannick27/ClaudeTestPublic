-------------------------------------------------------------------------------
-- SerialMul
--   Signed AW x BW multiplier built around ONE 18x18 signed multiplier
--   (maps on a single RTG4 MACC block). Operands are cut in 17-bit chunks
--   (lower chunks unsigned, top chunk signed), partial products are shifted
--   and accumulated one per clock cycle.
--
--   Latency : NA*NB + 4 clock cycles (NA = ceil(AW/17), NB = ceil(BW/17)).
--   Usage   : pulse StartxSI for 1 cycle, keep AxDI/BxDI stable until
--             DonexSO (1-cycle pulse). PxDO is valid from DonexSO and stays
--             valid until the next StartxSI.
--   Every register stage is a single small operation -> easy 100 MHz timing.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity SerialMul is
  generic(AW : positive := 34;
          BW : positive := 38);
  port(
    ClkxCI   : in  std_logic;
    StartxSI : in  std_logic;
    AxDI     : in  signed(AW - 1 downto 0);
    BxDI     : in  signed(BW - 1 downto 0);
    DonexSO  : out std_logic := '0';
    PxDO     : out signed(AW + BW - 1 downto 0)
    );
end entity SerialMul;

architecture rtl of SerialMul is

  constant CHUNK : positive := 17;
  constant NA    : positive := (AW + CHUNK - 1) / CHUNK;
  constant NB    : positive := (BW + CHUNK - 1) / CHUNK;
  constant RW    : positive := AW + BW;

  signal AExtxD : signed(NA * CHUNK - 1 downto 0);
  signal BExtxD : signed(NB * CHUNK - 1 downto 0);

  -- 17-bit chunk -> 18-bit signed (unsigned for lower chunks, signed for top)
  function GetChunk(v : signed; idx : natural; n : positive) return signed is
    variable u : signed(CHUNK - 1 downto 0);
  begin
    u := v(idx * CHUNK + CHUNK - 1 downto idx * CHUNK);
    if idx = n - 1 then
      return resize(u, CHUNK + 1);
    else
      return signed('0' & std_logic_vector(u));
    end if;
  end function GetChunk;

  signal IssuingxS : std_logic := '0';
  signal IxD       : integer range 0 to NA - 1 := 0;
  signal JxD       : integer range 0 to NB - 1 := 0;

  -- pipeline
  signal AqxD  : signed(CHUNK downto 0) := (others => '0');
  signal BqxD  : signed(CHUNK downto 0) := (others => '0');
  signal Sh1xD : integer range 0 to NA + NB - 2 := 0;
  signal V1xS  : std_logic := '0';
  signal L1xS  : std_logic := '0';

  signal PqxD  : signed(2 * CHUNK + 1 downto 0) := (others => '0');
  signal Sh2xD : integer range 0 to NA + NB - 2 := 0;
  signal V2xS  : std_logic := '0';
  signal L2xS  : std_logic := '0';

  signal SqxD  : signed(RW - 1 downto 0) := (others => '0');
  signal V3xS  : std_logic := '0';
  signal L3xS  : std_logic := '0';

  signal AccxD : signed(RW - 1 downto 0) := (others => '0');

begin

  AExtxD <= resize(AxDI, NA * CHUNK);
  BExtxD <= resize(BxDI, NB * CHUNK);
  PxDO   <= AccxD;

  process(ClkxCI)
  begin
    if rising_edge(ClkxCI) then
      DonexSO <= '0';

      -- stage 0 : chunk selection
      V1xS <= '0';
      L1xS <= '0';
      if StartxSI = '1' then
        IssuingxS <= '1';
        IxD       <= 0;
        JxD       <= 0;
        AccxD     <= (others => '0');
      elsif IssuingxS = '1' then
        AqxD  <= GetChunk(AExtxD, IxD, NA);
        BqxD  <= GetChunk(BExtxD, JxD, NB);
        Sh1xD <= IxD + JxD;
        V1xS  <= '1';
        if IxD = NA - 1 and JxD = NB - 1 then
          L1xS      <= '1';
          IssuingxS <= '0';
        end if;
        if JxD = NB - 1 then
          JxD <= 0;
          if IxD /= NA - 1 then
            IxD <= IxD + 1;
          end if;
        else
          JxD <= JxD + 1;
        end if;
      end if;

      -- stage 1 : 18x18 signed multiplication (MACC)
      PqxD  <= AqxD * BqxD;
      Sh2xD <= Sh1xD;
      V2xS  <= V1xS;
      L2xS  <= L1xS;

      -- stage 2 : weight of the partial product
      SqxD <= shift_left(resize(PqxD, RW), CHUNK * Sh2xD);
      V3xS <= V2xS;
      L3xS <= L2xS;

      -- stage 3 : accumulation
      if V3xS = '1' then
        AccxD <= AccxD + SqxD;
      end if;
      DonexSO <= V3xS and L3xS;
    end if;
  end process;

end architecture rtl;

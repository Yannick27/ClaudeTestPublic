-------------------------------------------------------------------------------
-- CordicAtan2 : fully pipelined CORDIC (vectoring mode) computing atan2(Y, X).
--
-- The angle is expressed in turns: 2*pi <=> 2^OUT_W. One result per clock,
-- latency NIT+2 clocks. Only the ratio Y/X matters, the scale of X and Y does
-- not (the value with the largest magnitude should preferably use most of the
-- DATA_W bits, see BlockNormalizer).
--
-- FULL_CIRCLE = true  : AnglexDO = atan2(Y, X)          in [0, 2*pi)
-- FULL_CIRCLE = false : AnglexDO = arctan(Y / X)        in (-pi/2, pi/2),
--                       negative angles wrap modulo 2*pi (two's complement)
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity CordicAtan2 is
  generic(
    DATA_W      : positive := 32;
    NIT         : positive := 18;        -- number of iterations (>= OUT_W+1 recommended)
    OUT_W       : positive := 16;        -- output angle width
    GUARD       : positive := 6;         -- extra internal angle bits
    FULL_CIRCLE : boolean  := true
  );
  port(
    ClkxCI   : in  std_logic;
    ValidxSI : in  std_logic;
    XxDI     : in  signed(DATA_W - 1 downto 0);
    YxDI     : in  signed(DATA_W - 1 downto 0);
    ValidxSO : out std_logic                  := '0';
    AnglexDO : out unsigned(OUT_W - 1 downto 0) := (others => '0')
  );
end entity CordicAtan2;

architecture rtl of CordicAtan2 is

  constant CW_C : positive := DATA_W + 2;      -- headroom for the CORDIC gain (1.647) and sqrt(2)
  constant AW_C : positive := OUT_W + GUARD;

  type XYArr_t   is array (0 to NIT) of signed(CW_C - 1 downto 0);
  type ZArr_t    is array (0 to NIT) of unsigned(AW_C - 1 downto 0);
  type AtanTab_t is array (0 to NIT - 1) of unsigned(AW_C - 1 downto 0);

  function AtanTable return AtanTab_t is
    variable t : AtanTab_t;
  begin
    for i in 0 to NIT - 1 loop
      t(i) := to_unsigned(integer(round(arctan(2.0 ** (-i)) / (2.0 * MATH_PI) * 2.0 ** AW_C)),
                          AW_C);
    end loop;
    return t;
  end function AtanTable;

  constant ATAN_C  : AtanTab_t := AtanTable;
  constant HALF_C  : unsigned(AW_C - 1 downto 0) := to_unsigned(2 ** (AW_C - 1), AW_C);
  constant ROUND_C : unsigned(AW_C downto 0) := to_unsigned(2 ** (GUARD - 1), AW_C + 1);

  signal XxDP : XYArr_t := (others => (others => '0'));
  signal YxDP : XYArr_t := (others => (others => '0'));
  signal ZxDP : ZArr_t  := (others => (others => '0'));
  signal VxDP : std_logic_vector(0 to NIT) := (others => '0');

begin

  -- stage 0 : move to the half plane X >= 0 (rotate by pi if X < 0)
  process(ClkxCI)
    variable X_v : signed(CW_C - 1 downto 0);
    variable Y_v : signed(CW_C - 1 downto 0);
    variable Z_v : unsigned(AW_C - 1 downto 0);
  begin
    if rising_edge(ClkxCI) then
      X_v := resize(XxDI, CW_C);
      Y_v := resize(YxDI, CW_C);
      Z_v := (others => '0');
      if XxDI(DATA_W - 1) = '1' then
        X_v := -X_v;
        Y_v := -Y_v;
        if FULL_CIRCLE then
          Z_v := HALF_C;
        end if;
      end if;
      XxDP(0) <= X_v;
      YxDP(0) <= Y_v;
      ZxDP(0) <= Z_v;
      VxDP(0) <= ValidxSI;
    end if;
  end process;

  -- stages 1..NIT : vectoring iterations, drive Y to 0
  Stages : for i in 0 to NIT - 1 generate
    process(ClkxCI)
    begin
      if rising_edge(ClkxCI) then
        if YxDP(i)(CW_C - 1) = '1' then          -- Y < 0 : rotate counter clockwise
          XxDP(i + 1) <= XxDP(i) - shift_right(YxDP(i), i);
          YxDP(i + 1) <= YxDP(i) + shift_right(XxDP(i), i);
          ZxDP(i + 1) <= ZxDP(i) - ATAN_C(i);
        else                                     -- Y >= 0 : rotate clockwise
          XxDP(i + 1) <= XxDP(i) + shift_right(YxDP(i), i);
          YxDP(i + 1) <= YxDP(i) - shift_right(XxDP(i), i);
          ZxDP(i + 1) <= ZxDP(i) + ATAN_C(i);
        end if;
        VxDP(i + 1) <= VxDP(i);
      end if;
    end process;
  end generate Stages;

  -- output stage : round the guard bits (wraps modulo 2*pi)
  process(ClkxCI)
    variable Sum_v : unsigned(AW_C downto 0);
  begin
    if rising_edge(ClkxCI) then
      Sum_v    := resize(ZxDP(NIT), AW_C + 1) + ROUND_C;
      AnglexDO <= Sum_v(AW_C - 1 downto GUARD);
      ValidxSO <= VxDP(NIT);
    end if;
  end process;

end architecture rtl;

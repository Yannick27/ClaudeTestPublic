-- Full-circle arctangent: AnglexDO = atan2(YxDI, XxDI) in turns, 65536 = 2*pi.
-- Result range 0 .. 65535 (negative angles wrap to 65536 - |angle|).
--
-- Only the DIRECTION of the vector (XxDI, YxDI) matters, so the vector is first
-- scaled by a common power of two so that max(|x|,|y|) lies in [2^28, 2^29)
-- (left shifts are lossless, right shifts only drop bits that are more than 28
-- bits below the largest component). It then goes through an iterative
-- vectoring CORDIC on 32 bit data:
--   * 2 bits of head-room: |x|,|y| <= sqrt(2)*2^29, CORDIC gain 1.647  -> < 2^31
--   * ITER = 20 micro-rotations, last residual angle atan(2^-19) = 0.02 LSB
--   * 24 bit angle accumulator (8 guard bits), rounded to 16 bit at the end.
-- Nothing is clamped: all widths are sized for the worst case.
--
-- Everything is serial / multi-cycle (a result takes at most ~125 clock cycles)
-- so that every register-to-register path is a single shift or add.
--
-- Handshake: StartxSI = '1' for one clock with the vector on XxDI/YxDI (the
-- vector is copied at that time). DonexSO is a one clock pulse, coincident with
-- the new AnglexDO. AnglexDO is held until the next result.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity CordicAtan2 is
  generic(
    NW : positive := 68                          -- width of the input components
  );
  port(
    ClkxCI   : in  std_logic;
    StartxSI : in  std_logic;
    XxDI     : in  signed(NW-1 downto 0);        -- "cos" component
    YxDI     : in  signed(NW-1 downto 0);        -- "sin" component
    DonexSO  : out std_logic := '0';
    AnglexDO : out unsigned(15 downto 0) := (others => '0')
  );
end entity CordicAtan2;

architecture rtl of CordicAtan2 is

  constant CW   : positive := 32;                -- CORDIC data width
  constant ZW   : positive := 24;                -- angle width, one turn = 2^ZW
  constant ITER : positive := 20;                -- number of micro-rotations
  constant NORM_TOP : natural := 29;             -- normalised: |component| < 2^NORM_TOP

  type AtanTable_t is array (0 to ITER-1) of unsigned(ZW-1 downto 0);
  -- round(atan(2^-i) / (2*pi) * 2^24)
  constant ATAN_TAB : AtanTable_t := (
    to_unsigned(2097152, ZW), to_unsigned(1238021, ZW), to_unsigned( 654136, ZW),
    to_unsigned( 332050, ZW), to_unsigned( 166669, ZW), to_unsigned(  83416, ZW),
    to_unsigned(  41718, ZW), to_unsigned(  20860, ZW), to_unsigned(  10430, ZW),
    to_unsigned(   5215, ZW), to_unsigned(   2608, ZW), to_unsigned(   1304, ZW),
    to_unsigned(    652, ZW), to_unsigned(    326, ZW), to_unsigned(    163, ZW),
    to_unsigned(     81, ZW), to_unsigned(     41, ZW), to_unsigned(     20, ZW),
    to_unsigned(     10, ZW), to_unsigned(      5, ZW));

  constant HALF_TURN : unsigned(ZW-1 downto 0) := to_unsigned(2**(ZW-1), ZW);
  constant RND_HALF_LSB : unsigned(ZW-1 downto 0) := to_unsigned(2**(ZW-17), ZW);

  -- True if bits v'high downto lo are all equal, i.e. v fits in lo+1 bits.
  function SignCopies(v : signed; lo : natural) return boolean is
  begin
    for i in lo to v'high loop
      if v(i) /= v(v'high) then
        return false;
      end if;
    end loop;
    return true;
  end function SignCopies;

  type state_t is (S_IDLE, S_N_EVAL, S_N_STEP, S_LOAD, S_SHIFT, S_ROT, S_FINISH);
  signal StateR : state_t := S_IDLE;

  signal NumxD   : signed(NW-1 downto 0);        -- y while normalising
  signal DenxD   : signed(NW-1 downto 0);        -- x while normalising
  signal FitHixS : boolean := false;             -- both components < 2^NORM_TOP
  signal FitLoxS : boolean := false;             -- both components < 2^(NORM_TOP-1)
  signal ZeroxS  : boolean := false;             -- vector is (0,0)

  signal XxD     : signed(CW-1 downto 0);
  signal YxD     : signed(CW-1 downto 0);
  signal ZxD     : unsigned(ZW-1 downto 0);
  signal XSxD    : signed(CW-1 downto 0);        -- x >> i
  signal YSxD    : signed(CW-1 downto 0);        -- y >> i
  signal ZAxD    : unsigned(ZW-1 downto 0);      -- atan(2^-i)
  signal IxD     : integer range 0 to ITER-1 := 0;

begin

  process(ClkxCI)
    variable AngSumxV : unsigned(ZW-1 downto 0);
  begin
    if rising_edge(ClkxCI) then
      DonexSO <= '0';

      case StateR is

        when S_IDLE =>
          if StartxSI = '1' then
            NumxD  <= YxDI;
            DenxD  <= XxDI;
            StateR <= S_N_EVAL;
          end if;

        -- scale the vector (one shift per two clocks) ---------------------------
        when S_N_EVAL =>
          FitHixS <= SignCopies(NumxD, NORM_TOP) and SignCopies(DenxD, NORM_TOP);
          FitLoxS <= SignCopies(NumxD, NORM_TOP - 1) and SignCopies(DenxD, NORM_TOP - 1);
          ZeroxS  <= (NumxD = 0) and (DenxD = 0);
          StateR  <= S_N_STEP;

        when S_N_STEP =>
          if not FitHixS then                     -- too big: shift right
            NumxD  <= shift_right(NumxD, 1);
            DenxD  <= shift_right(DenxD, 1);
            StateR <= S_N_EVAL;
          elsif FitLoxS and not ZeroxS then       -- too small: shift left
            NumxD  <= shift_left(NumxD, 1);
            DenxD  <= shift_left(DenxD, 1);
            StateR <= S_N_EVAL;
          else
            StateR <= S_LOAD;
          end if;

        -- x < 0: reflect the vector through the origin and add half a turn,
        -- so that the CORDIC always starts in the right half plane
        when S_LOAD =>
          if DenxD(NW-1) = '1' then
            XxD <= -DenxD(CW-1 downto 0);
            YxD <= -NumxD(CW-1 downto 0);
            ZxD <= HALF_TURN;
          else
            XxD <= DenxD(CW-1 downto 0);
            YxD <= NumxD(CW-1 downto 0);
            ZxD <= (others => '0');
          end if;
          IxD    <= 0;
          StateR <= S_SHIFT;

        -- CORDIC micro-rotation, split in two clocks: shift, then add ----------
        when S_SHIFT =>
          XSxD   <= shift_right(XxD, IxD);
          YSxD   <= shift_right(YxD, IxD);
          ZAxD   <= ATAN_TAB(IxD);
          StateR <= S_ROT;

        when S_ROT =>
          if YxD(CW-1) = '0' then                 -- y >= 0: rotate clockwise
            XxD <= XxD + YSxD;
            YxD <= YxD - XSxD;
            ZxD <= ZxD + ZAxD;
          else                                    -- y < 0: rotate counter-clockwise
            XxD <= XxD - YSxD;
            YxD <= YxD + XSxD;
            ZxD <= ZxD - ZAxD;
          end if;
          if IxD = ITER - 1 then
            StateR <= S_FINISH;
          else
            IxD    <= IxD + 1;
            StateR <= S_SHIFT;
          end if;

        -- angle modulo one turn (the unsigned wrap-around is intended) ---------
        when S_FINISH =>
          AngSumxV := ZxD + RND_HALF_LSB;
          AnglexDO <= AngSumxV(ZW-1 downto ZW-16);
          DonexSO  <= '1';
          StateR   <= S_IDLE;

      end case;
    end if;
  end process;

end architecture rtl;

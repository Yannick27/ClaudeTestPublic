-------------------------------------------------------------------------------
-- SerialMulAcc
-- Exact serial multiplier / dot-product engine built around one 17x17 signed
-- multiplier (maps to one RTG4 18x18 math block).
--
--   P = sum over t = 0..NT-1 of  (+/-) A(t) * B(t)        (signed, exact)
--
-- Operands are cut into 16-bit chunks (lower chunks unsigned, top chunk
-- signed).  The chunk products are accumulated column by column (Comba
-- scheme): after each column the low 16 bits of the accumulator are shifted
-- into the result and the accumulator is shifted right by 16.  The
-- accumulator therefore stays narrow (<= 37 bits for the instances used here)
-- no matter how wide the operands are, and no wide adder is ever needed.
--
-- Operand term t is   AxDI((t+1)*WA-1 downto t*WA)   and
--                     BxDI((t+1)*WB-1 downto t*WB).
-- SubxSI(t) = '1' subtracts term t instead of adding it.
--
-- Handshake: pulse StartxSI for one clock; AxDI/BxDI/SubxSI must stay stable
-- until DonexSO.  DonexSO is a one-clock pulse; PxDO is valid from that clock
-- until the next StartxSI.
-- Latency from StartxSI to DonexSO: NT*NA*NB + 5 clocks (NA = ceil(WA/16),
-- NB = ceil(WB/16)).
--
-- The order in which the chunk products are issued is fixed by the generics,
-- so it is a constant table (SCHED) built at elaboration; the control logic is
-- just a position counter and a ROM, with one-hot chunk selects.
--
-- Pipeline: schedule ROM -> chunk select -> multiplier (input regs) ->
--           product reg -> accumulate -> result shift register.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.AngleComputePkg.all;

entity SerialMulAcc is
  generic (
    WA : positive := 32;  -- width of every A operand (signed)
    WB : positive := 32;  -- width of every B operand (signed)
    NT : positive := 1    -- number of products summed
  );
  port (
    ClkxCI   : in  std_logic;
    StartxSI : in  std_logic;
    AxDI     : in  signed(NT * WA - 1 downto 0);
    BxDI     : in  signed(NT * WB - 1 downto 0);
    SubxSI   : in  std_logic_vector(NT - 1 downto 0);
    DonexSO  : out std_logic := '0';
    PxDO     : out signed(WA + WB + CeilLog2(NT) - 1 downto 0)
  );
end entity SerialMulAcc;

architecture rtl of SerialMulAcc is

  constant CHUNK : positive := 16;
  constant NA    : positive := (WA + CHUNK - 1) / CHUNK;  -- chunks of A
  constant NB    : positive := (WB + CHUNK - 1) / CHUNK;  -- chunks of B
  constant NC    : positive := NA + NB - 1;               -- columns
  -- |column sum| < (NT * min(NA,NB) + 1) * 2**32, plus one bit of margin
  constant ACCW  : positive := 34 + CeilLog2(NT * MinInt(NA, NB) + 1);
  constant WP    : positive := WA + WB + CeilLog2(NT);
  constant FULLW : positive := ACCW - CHUNK + CHUNK * NC;

  -- chunk idx of v: unsigned (zero extended) except the top one (sign extended)
  function ChunkOf (v : signed; idx : natural; n : positive) return signed is
    variable ext : signed(n * CHUNK - 1 downto 0);
    variable lo  : signed(CHUNK - 1 downto 0);
  begin
    ext := resize(v, n * CHUNK);
    lo  := ext(idx * CHUNK + CHUNK - 1 downto idx * CHUNK);
    if idx = n - 1 then
      return lo(CHUNK - 1) & lo;
    end if;
    return '0' & lo;
  end function ChunkOf;

  -- Issue schedule: the chunk products in column order.  Within a column the
  -- terms come first, then the A-chunks; the B-chunk index is column - A-chunk.
  constant NPAIR : positive := NT * NA * NB;

  type SchedEntry_t is record
    SelA   : std_logic_vector(NT * NA - 1 downto 0);  -- one-hot (term, A chunk)
    SelB   : std_logic_vector(NT * NB - 1 downto 0);  -- one-hot (term, B chunk)
    SelT   : std_logic_vector(NT - 1 downto 0);       -- one-hot term
    First  : std_logic;                               -- first product of a column
    ColEnd : std_logic;                               -- last product of a column
    Fin    : std_logic;                               -- last product of all
  end record;
  type Sched_t is array (0 to NPAIR - 1) of SchedEntry_t;

  function MakeSched return Sched_t is
    variable sch : Sched_t;
    variable n   : natural := 0;
    variable i0  : integer;
    variable i1  : integer;
  begin
    for c in 0 to NC - 1 loop
      i0 := MaxInt(0, c - (NB - 1));  -- A-chunk range of this column
      i1 := MinInt(NA - 1, c);
      for t in 0 to NT - 1 loop
        for i in i0 to i1 loop
          sch(n).SelA := (others => '0');
          sch(n).SelB := (others => '0');
          sch(n).SelT := (others => '0');
          sch(n).SelA(t * NA + i)       := '1';
          sch(n).SelB(t * NB + (c - i)) := '1';
          sch(n).SelT(t)                := '1';
          sch(n).First  := '0';
          sch(n).ColEnd := '0';
          sch(n).Fin    := '0';
          if t = 0 and i = i0 then
            sch(n).First := '1';
          end if;
          if t = NT - 1 and i = i1 then
            sch(n).ColEnd := '1';
            if c = NC - 1 then
              sch(n).Fin := '1';
            end if;
          end if;
          n := n + 1;
        end loop;
      end loop;
    end loop;
    return sch;
  end function MakeSched;

  constant SCHED : Sched_t := MakeSched;

  constant SCHED_NONE : SchedEntry_t := (
    SelA   => (others => '0'),
    SelB   => (others => '0'),
    SelT   => (others => '0'),
    First  => '0',
    ColEnd => '0',
    Fin    => '0');

  -- stage 0: schedule position and registered schedule entry
  signal Busy : std_logic := '0';
  signal Pos  : integer range 0 to NPAIR - 1 := 0;
  signal Cur  : SchedEntry_t := SCHED_NONE;
  signal CurV : std_logic := '0';

  -- stage 1: selected chunks
  signal ChA     : signed(CHUNK downto 0) := (others => '0');
  signal ChB     : signed(CHUNK downto 0) := (others => '0');
  signal V1      : std_logic := '0';
  signal First1  : std_logic := '0';
  signal ColEnd1 : std_logic := '0';
  signal Fin1    : std_logic := '0';
  signal Sub1    : std_logic := '0';

  -- stage 2: chunk product
  signal Prod    : signed(2 * CHUNK + 1 downto 0) := (others => '0');
  signal V2      : std_logic := '0';
  signal First2  : std_logic := '0';
  signal ColEnd2 : std_logic := '0';
  signal Fin2    : std_logic := '0';
  signal Sub2    : std_logic := '0';

  -- stage 3: column accumulator, stage 4: result shift register
  signal Acc     : signed(ACCW - 1 downto 0) := (others => '0');
  signal ColEnd3 : std_logic := '0';
  signal Fin3    : std_logic := '0';
  signal Res     : signed(CHUNK * NC - 1 downto 0) := (others => '0');

  signal Full    : signed(FULLW - 1 downto 0);

begin

  process (ClkxCI)
    variable base : signed(ACCW - 1 downto 0);
    variable va   : signed(CHUNK downto 0);
    variable vb   : signed(CHUNK downto 0);
    variable sb   : std_logic;
  begin
    if rising_edge(ClkxCI) then

      ---------------------------------------------------------------------
      -- stage 0: walk through the schedule
      ---------------------------------------------------------------------
      CurV <= '0';
      if StartxSI = '1' then
        Busy <= '1';
        Pos  <= 0;
        Acc  <= (others => '0');
      elsif Busy = '1' then
        Cur  <= SCHED(Pos);
        CurV <= '1';
        if SCHED(Pos).Fin = '1' then
          Busy <= '0';
        else
          Pos <= Pos + 1;
        end if;
      end if;

      ---------------------------------------------------------------------
      -- stage 1: fetch the chunks selected by the one-hot schedule entry
      ---------------------------------------------------------------------
      va := (others => '0');
      vb := (others => '0');
      sb := '0';
      for tt in 0 to NT - 1 loop
        for ii in 0 to NA - 1 loop
          if Cur.SelA(tt * NA + ii) = '1' then
            va := va or ChunkOf(AxDI((tt + 1) * WA - 1 downto tt * WA), ii, NA);
          end if;
        end loop;
        for jj in 0 to NB - 1 loop
          if Cur.SelB(tt * NB + jj) = '1' then
            vb := vb or ChunkOf(BxDI((tt + 1) * WB - 1 downto tt * WB), jj, NB);
          end if;
        end loop;
        if Cur.SelT(tt) = '1' and SubxSI(tt) = '1' then
          sb := '1';
        end if;
      end loop;
      ChA     <= va;
      ChB     <= vb;
      Sub1    <= sb;
      V1      <= CurV;
      First1  <= Cur.First;
      ColEnd1 <= Cur.ColEnd;
      Fin1    <= Cur.Fin;

      ---------------------------------------------------------------------
      -- stage 2: 17x17 signed multiply
      ---------------------------------------------------------------------
      Prod    <= ChA * ChB;
      V2      <= V1;
      First2  <= First1;
      ColEnd2 <= ColEnd1;
      Fin2    <= Fin1;
      Sub2    <= Sub1;

      ---------------------------------------------------------------------
      -- stage 3: column accumulation; at the first product of a column the
      -- previous column sum is first shifted right by 16 (carry)
      ---------------------------------------------------------------------
      ColEnd3 <= '0';
      Fin3    <= '0';
      if V2 = '1' then
        if First2 = '1' then
          base := shift_right(Acc, CHUNK);
        else
          base := Acc;
        end if;
        if Sub2 = '1' then
          Acc <= base - resize(Prod, ACCW);
        else
          Acc <= base + resize(Prod, ACCW);
        end if;
        ColEnd3 <= ColEnd2;
        Fin3    <= Fin2;
      end if;

      ---------------------------------------------------------------------
      -- stage 4: a finished column sum is in Acc; move its low 16 bits into
      -- the result shift register (first column ends up in the LSBs)
      ---------------------------------------------------------------------
      if ColEnd3 = '1' then
        Res <= Acc(CHUNK - 1 downto 0) & Res(Res'high downto CHUNK);
      end if;
      DonexSO <= Fin3;

    end if;
  end process;

  -- the remaining carry (Acc >> 16) forms the upper bits of the product
  Full <= Acc(ACCW - 1 downto CHUNK) & Res;
  PxDO <= Full(WP - 1 downto 0);

end architecture rtl;

--------------------------------------------------------------------------------
-- StepCompute
--
-- Computes, from a measured and a target angle (16 bit, full circle = 2^16),
-- the rotation direction, the number of full rotation steps and the amplitude
-- (VMax) of a last partial step.
--
-- Target : Microchip RTG4 FPGA. RTG4 has no hard divider, but the divisor is a
--          constant, so:
--            * AngleDiff / ROT_PER_STEP  = (AngleDiff * M) >> SHIFT
--              (one 16x17 unsigned multiply -> one MACC block, with input and
--              output registers)
--            * (2^16-1) * AngleDiff / ROT_PER_STEP for AngleDiff <= ROT_PER_STEP
--              is a small constant table (ROT_PER_STEP+1 entries).
--          Both are exact; this is checked at elaboration (asserts below).
--
-- Latency: OutputValidxSO is high 3 clock cycles after ValidAnglexSI.
--          The design is a pure pipeline: a new angle can be accepted on
--          every clock cycle.
--
-- Conventions:
--   * Increasing angle = counter-clockwise (RotDirectionxSO = '1')
--   * Angle difference is the shortest distance on the circle (max 2^15).
--     In the exact opposite case (2^15) counter-clockwise is chosen.
--   * Target reached (AngleDiff <= ROT_ACCURACY): StepNumberxDO = 0,
--     VMaxExtxDO = 0, RotDirectionxSO keeps its previous value.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity StepCompute is
  port (
    ClkxCI           : in  std_logic;   -- 100 MHz clock
    ValidAnglexSI    : in  std_logic;   -- angle is ready (1 clock cycle wide pulse)
    AngleMeasuredxDI : in  unsigned(15 downto 0);  -- measured angle on the full circle (2^16 <=> 2*pi)
    AngleTargetxDI   : in  unsigned(15 downto 0);  -- target angle on the full circle (2^16 <=> 2*pi)
    VMaxExtxDO       : out std_logic_vector(15 downto 0) := (others => '0');  -- amplitude of the (last) rotation step (2^16 - 1 <=> ROT_PER_STEP_DEG)
    RotDirectionxSO  : out std_logic                     := '0';  -- 0 = clockwise, 1 = counter clockwise
    StepNumberxDO    : out natural range 0 to 2000       := 0;    -- number of rotation steps of the plate
    OutputValidxSO   : out std_logic                     := '0'   -- outputs are valid (1 clock cycle wide pulse)
    );
end entity StepCompute;

architecture rtl of StepCompute is

  constant ROT_PER_STEP_DEG : real    := 0.32;  -- in degree
  constant ROT_ACCURACY_DEG : real    := 0.01;  -- in degree
  constant ROT_ACCURACY     : integer := integer(ROT_ACCURACY_DEG / 360.0 * 2.0 ** 16);  -- = 2
  constant ROT_PER_STEP     : integer := integer(ROT_PER_STEP_DEG / 360.0 * 2.0 ** 16);  -- = 58

  constant MAX_DIFF : integer := 2 ** 15;  -- largest shortest-way angle difference

  -- reciprocal multiplication: floor(x / ROT_PER_STEP) = (x * RECIP_M) >> RECIP_S
  constant RECIP_S : integer := 22;
  constant RECIP_M : integer := (2 ** RECIP_S + ROT_PER_STEP - 1) / ROT_PER_STEP;  -- ceil(2^S / P), 17 bit
  constant RECIP_E : integer := RECIP_M * ROT_PER_STEP - 2 ** RECIP_S;             -- rounding error of M

  -- partial step: VMax = (2^16 - 1) * i / ROT_PER_STEP, i = 0 .. ROT_PER_STEP
  type vmax_rom_t is array (0 to ROT_PER_STEP) of std_logic_vector(15 downto 0);
  function init_vmax_rom return vmax_rom_t is
    variable rom : vmax_rom_t;
  begin
    for i in 0 to ROT_PER_STEP loop
      rom(i) := std_logic_vector(to_unsigned((i * 65535) / ROT_PER_STEP, 16));
    end loop;
    return rom;
  end function;
  constant VMAX_ROM : vmax_rom_t := init_vmax_rom;

  -- pipeline stage 1: shortest angle difference
  signal V1xS    : std_logic             := '0';
  signal Diff1xD : unsigned(15 downto 0) := (others => '0');
  signal Dir1xS  : std_logic             := '0';

  -- pipeline stage 2: multiplication (MACC) / table / classification
  signal V2xS     : std_logic             := '0';
  signal Prod2xD  : unsigned(32 downto 0) := (others => '0');
  signal Vmax2xD  : std_logic_vector(15 downto 0) := (others => '0');
  signal Dir2xS   : std_logic             := '0';
  signal Far2xS   : std_logic             := '0';  -- far from target: full steps
  signal Close2xS : std_logic             := '0';  -- close to target: one partial step

begin

  -- elaboration checks: the reciprocal is exact for all x <= MAX_DIFF
  --   (x*M = x*2^S/P + x*E/P, exact if x*E < 2^S)
  assert ROT_PER_STEP > 0 and ROT_PER_STEP < 1024
    report "ROT_PER_STEP out of supported range" severity failure;
  assert RECIP_M < 2 ** 17
    report "reciprocal does not fit a single 18x18 MACC, lower RECIP_S" severity failure;
  assert MAX_DIFF * RECIP_E < 2 ** RECIP_S
    report "reciprocal division not exact, raise RECIP_S" severity failure;
  assert MAX_DIFF / ROT_PER_STEP <= 2000
    report "StepNumberxDO range too small" severity failure;

  process (ClkxCI)
    variable DeltaxD : unsigned(15 downto 0);
    variable IdxxD   : natural range 0 to ROT_PER_STEP;
  begin
    if rising_edge(ClkxCI) then

      ------------------------------------------------------------------------
      -- stage 1: shortest signed distance on the circle (wraps modulo 2^16)
      ------------------------------------------------------------------------
      V1xS    <= ValidAnglexSI;
      DeltaxD := AngleTargetxDI - AngleMeasuredxDI;
      if DeltaxD(15) = '1' and DeltaxD(14 downto 0) /= 0 then
        -- target is more than half a turn ahead: go clockwise
        Dir1xS  <= '0';
        Diff1xD <= 0 - DeltaxD;  -- 2^16 - Delta
      else
        Dir1xS  <= '1';
        Diff1xD <= DeltaxD;
      end if;

      ------------------------------------------------------------------------
      -- stage 2: steps = Diff / ROT_PER_STEP via MACC, partial VMax via table
      ------------------------------------------------------------------------
      V2xS    <= V1xS;
      Dir2xS  <= Dir1xS;
      Prod2xD <= Diff1xD * to_unsigned(RECIP_M, 17);

      if Diff1xD <= ROT_PER_STEP then
        IdxxD := to_integer(Diff1xD);
      else
        IdxxD := 0;
      end if;
      Vmax2xD <= VMAX_ROM(IdxxD);

      Far2xS   <= '0';
      Close2xS <= '0';
      if Diff1xD > ROT_PER_STEP then
        Far2xS <= '1';
      elsif Diff1xD > ROT_ACCURACY then
        Close2xS <= '1';
      end if;

      ------------------------------------------------------------------------
      -- stage 3: output registers
      ------------------------------------------------------------------------
      OutputValidxSO <= V2xS;
      if V2xS = '1' then
        if Far2xS = '1' then
          RotDirectionxSO <= Dir2xS;
          VMaxExtxDO      <= (others => '1');
          StepNumberxDO   <= to_integer(Prod2xD(RECIP_S + 10 downto RECIP_S));
        elsif Close2xS = '1' then
          RotDirectionxSO <= Dir2xS;
          VMaxExtxDO      <= Vmax2xD;
          StepNumberxDO   <= 1;
        else
          VMaxExtxDO      <= (others => '0');
          StepNumberxDO   <= 0;
        end if;
      end if;

    end if;
  end process;

end architecture rtl;

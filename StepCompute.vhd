-------------------------------------------------------------------------------
-- StepCompute
--
-- Computes the rotation command (direction, speed, number of steps) needed to
-- bring a plate from a measured angle to a target angle.
--
-- * Angles are on the full circle: 2^16 = 360 deg. Increasing angle is
--   counter-clockwise.
-- * Fully synchronous, single clock, no reset (outputs have power-up values),
--   no latches, no division, registered outputs. Multiplications are by
--   constants and map on the RTG4 math blocks (or on logic).
-- * Latency: 4 clock cycles from ValidAnglexSI to OutputValidxSO. The
--   outputs are held until the next result.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity StepCompute is
  port (
    ClkxCI           : in  std_logic;   -- 100 MHz clock
    ValidAnglexSI    : in  std_logic;   -- angle is ready (1 clock cycle wide pulse)
    AngleMeasuredxDI : in  unsigned(15 downto 0);  -- measured angle on the full circle (2^16 = 2*pi)
    AngleTargetxDI   : in  unsigned(15 downto 0);  -- target angle on the full circle (2^16 = 2*pi)
    VMaxExtxDO       : out std_logic_vector(15 downto 0) := (others => '0');  -- rotation of the plate in one rotation step (2^16 - 1 = ROT_PER_STEP_DEG)
    RotDirectionxSO  : out std_logic                     := '0';  -- direction of the rotation (0=clockwise, 1=counter clockwise)
    StepNumberxDO    : out natural range 0 to 2000       := 0;    -- number of rotation steps of the plate
    OutputValidxSO   : out std_logic                     := '0'   -- outputs are valid (1 clock cycle wide pulse)
    );
end entity StepCompute;

architecture rtl of StepCompute is

  constant ROT_PER_STEP_DEG : real                  := 0.32;  -- rotation step in degree
  constant ROT_ACCURACY_DEG : real                  := 0.01;  -- requested angle accuracy in degree
  constant VMAX_EXT         : positive              := 140;   -- V
  constant ROT_ACCURACY     : integer               := integer(ROT_ACCURACY_DEG / 360.0 * 2.0 ** 16);
  constant ROT_PER_STEP     : integer               := integer(ROT_PER_STEP_DEG / 360.0 * 2.0 ** 16);
  constant VMAX_EXT_SLV     : unsigned(15 downto 0) := to_unsigned(2 ** 16 * VMAX_EXT / 150, 16);
  constant ROT_PER_STEP_INV : integer               := integer(1.0 / ROT_PER_STEP_DEG * 360.0);
  -- VMAX_EXT_SLV / ROT_PER_STEP
  constant PARTIAL_RATIO    : integer               := integer(real(VMAX_EXT) / 150.0 / (ROT_PER_STEP_DEG / 360.0));

  -- Largest AngleDiff is 2^15 (half circle): the multiplier operands below
  -- are sized from the constants so the products can never overflow.
  constant COEF_W : positive := 11;     -- ceil(log2(max(ROT_PER_STEP_INV, PARTIAL_RATIO) + 1))

  -- pipeline stage 1: raw difference (target - measured) modulo 2^16
  signal DeltaxD   : unsigned(15 downto 0) := (others => '0');
  signal Valid1xS  : std_logic             := '0';

  -- pipeline stage 2: shortest absolute difference and direction
  signal AngleDiffxD  : unsigned(15 downto 0) := (others => '0');
  signal DirectionxS  : std_logic             := '0';
  signal Valid2xS     : std_logic             := '0';

  -- pipeline stage 3: products
  signal DiffxD       : unsigned(15 downto 0)          := (others => '0');
  signal Direction3xS : std_logic                      := '0';
  signal StepProdxD   : unsigned(15 + COEF_W downto 0) := (others => '0');
  signal PartProdxD   : unsigned(15 + COEF_W downto 0) := (others => '0');
  signal Valid3xS     : std_logic                      := '0';

begin

  -- PARTIAL_RATIO is only applied when AngleDiff <= ROT_PER_STEP, so the
  -- product stays within 16 bits
  assert ROT_PER_STEP * PARTIAL_RATIO < 2 ** 16
    report "PARTIAL_RATIO * ROT_PER_STEP overflows VMaxExtxDO"
    severity failure;
  assert (2 ** 15) * ROT_PER_STEP_INV / 2 ** 16 <= 2000
    report "StepNumberxDO may exceed its range"
    severity failure;
  assert ROT_PER_STEP_INV < 2 ** COEF_W and PARTIAL_RATIO < 2 ** COEF_W
    report "COEF_W too small"
    severity failure;

  process (ClkxCI)
  begin
    if rising_edge(ClkxCI) then

      ---------------------------------------------------------------------
      -- Stage 1: difference on the circle, modulo 2^16 (natural wrap-around)
      ---------------------------------------------------------------------
      DeltaxD  <= AngleTargetxDI - AngleMeasuredxDI;
      Valid1xS <= ValidAnglexSI;

      ---------------------------------------------------------------------
      -- Stage 2: shortest way.
      -- Delta < 2^15 : target is ahead in the increasing (counter-clockwise)
      --                direction  -> rotate CCW ('1'), diff = Delta
      -- Delta >= 2^15: target is behind -> rotate CW ('0'), diff = 2^16 - Delta
      -- (a half-circle difference, 2^15, rotates CW.)
      ---------------------------------------------------------------------
      if DeltaxD(15) = '0' then
        AngleDiffxD <= DeltaxD;
        DirectionxS <= '1';
      else
        AngleDiffxD <= (not DeltaxD) + 1;  -- two's complement, = 2^16 - Delta
        DirectionxS <= '0';
      end if;
      Valid2xS <= Valid1xS;

      ---------------------------------------------------------------------
      -- Stage 3: multiplications by constants (registered for timing)
      ---------------------------------------------------------------------
      DiffxD       <= AngleDiffxD;
      Direction3xS <= DirectionxS;
      StepProdxD   <= AngleDiffxD * to_unsigned(ROT_PER_STEP_INV, COEF_W);
      PartProdxD   <= AngleDiffxD * to_unsigned(PARTIAL_RATIO, COEF_W);
      Valid3xS     <= Valid2xS;

      ---------------------------------------------------------------------
      -- Stage 4: decision and registered outputs
      ---------------------------------------------------------------------
      OutputValidxSO <= Valid3xS;

      if Valid3xS = '1' then
        if DiffxD > ROT_PER_STEP then
          -- far from target: full speed, AngleDiff * ROT_PER_STEP_INV / 2^16
          RotDirectionxSO <= Direction3xS;
          VMaxExtxDO      <= std_logic_vector(VMAX_EXT_SLV);
          StepNumberxDO   <= to_integer(StepProdxD(StepProdxD'left downto 16));
        elsif DiffxD > ROT_ACCURACY then
          -- close to target: one partial step, AngleDiff * PARTIAL_RATIO
          RotDirectionxSO <= Direction3xS;
          VMaxExtxDO      <= std_logic_vector(PartProdxD(15 downto 0));
          StepNumberxDO   <= 1;
        else
          -- target reached: VMaxExtxDO and RotDirectionxSO are don't care
          -- (VMax forced to 0 so the plate cannot be driven by mistake)
          VMaxExtxDO    <= (others => '0');
          StepNumberxDO <= 0;
        end if;
      end if;

    end if;
  end process;

end architecture rtl;

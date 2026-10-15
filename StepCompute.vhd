--------------------------------------------------------------------------------
-- StepCompute
--
-- Computes, from a measured and a target angle (full circle = 2^16), the
-- direction, the number of rotation steps and the voltage setting (VMax) of
-- the next move of the plate.
--
-- Target device : Microchip RTG4 (no hardware divider, 18x18 math blocks)
--   * no division is implemented in logic: both divisions by ROT_PER_STEP are
--     replaced by multiplications with a pre-computed (elaboration time)
--     fixed-point reciprocal, which map onto the RTG4 math blocks.
--   * fully pipelined (a new angle may be presented on every clock cycle),
--     latency = 4 clock cycles between ValidAnglexSI and OutputValidxSO.
--   * no reset port: registers rely on their initial values (power-up value
--     of the RTG4 flip-flops).
--
-- Conventions:
--   * increasing angle = counter-clockwise rotation
--   * RotDirectionxSO = '0' : clockwise, '1' : counter-clockwise
--   * AngleDiff is the shortest distance on the circle (0 .. 2^15)
--     (exactly half a turn: clockwise is chosen)
--
-- Latency / pipeline:
--   stage 1 : delta    = Target - Measured   (modulo 2^16)
--   stage 2 : AngleDiff = |delta| on the circle + direction
--   stage 3 : multiplications (reciprocals) + classification
--   stage 4 : output selection / registers  (OutputValidxSO = '1')
--------------------------------------------------------------------------------
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
    VMaxExtxDO       : out std_logic_vector(15 downto 0) := (others => '0');  -- rotation of the plate in one rotation step (2^16 - 1 = ROT_PER_STEP_DEG°)
    RotDirectionxSO  : out std_logic                     := '0';  -- direction of the rotation of the plate (0=clockwise, 1=counter clockwise)
    StepNumberxDO    : out natural range 0 to 2000       := 0;  -- number of rotation steps of the plate
    OutputValidxSO   : out std_logic                     := '0'  -- outputs are valid
    );
end entity StepCompute;

architecture rtl of StepCompute is

  ------------------------------------------------------------------------------
  -- Constants
  ------------------------------------------------------------------------------
  constant ROT_PER_STEP_DEG : real                  := 0.32;  -- rotation step in degree
  constant ROT_ACCURACY_DEG : real                  := 0.01;  -- requested angle accuracy in degree
  constant ROT_ACCURACY     : integer               := integer(ROT_ACCURACY_DEG / 360.0 * 2.0 ** 16);
  constant ROT_PER_STEP     : integer               := integer(ROT_PER_STEP_DEG / 360.0 * 2.0 ** 16);
  constant VMAX_EXT         : positive              := 140;  --V
  constant VMAX_EXT_SLV     : unsigned(15 downto 0) := to_unsigned(2 ** 16 * VMAX_EXT / 150, 16);

  -- number of bits needed to represent the value n
  function bits_for (n : natural) return positive is
    variable v : natural  := n;
    variable r : positive := 1;
  begin
    while v > 1 loop
      v := v / 2;
      r := r + 1;
    end loop;
    return r;
  end function bits_for;

  constant ANGLE_W : positive := 16;
  constant HALF    : natural  := 2 ** (ANGLE_W - 1);  -- half a turn

  ------------------------------------------------------------------------------
  -- Division by ROT_PER_STEP replaced by a multiplication:
  --   q = floor(AngleDiff * RECIP_Q / 2**RECIP_SHIFT),  RECIP_Q = ceil(2**S / D)
  -- exact for every AngleDiff <= 2**15 as long as D * 2**15 <= 2**S
  -- (D <= 512 with S = 24, checked below).
  ------------------------------------------------------------------------------
  constant RECIP_SHIFT : natural  := 24;
  constant RECIP_Q     : natural  := (2 ** RECIP_SHIFT + ROT_PER_STEP - 1) / ROT_PER_STEP;
  constant RECIP_Q_W   : positive := bits_for(RECIP_Q);
  constant STEP_W      : positive := bits_for(2000);

  ------------------------------------------------------------------------------
  -- Partial step: VMax = VMAX_EXT * AngleDiff / ROT_PER_STEP
  --   = (AngleDiff * VMAX_SCALE) / 2**VMAX_FRAC       (AngleDiff <= ROT_PER_STEP)
  -- VMAX_SCALE is rounded down so that VMax never exceeds VMAX_EXT_SLV.
  ------------------------------------------------------------------------------
  constant VMAX_EXT_INT : natural  := 2 ** 16 * VMAX_EXT / 150;
  constant VMAX_FRAC    : natural  := 12;
  constant VMAX_SCALE   : natural  := VMAX_EXT_INT * 2 ** VMAX_FRAC / ROT_PER_STEP;
  constant VMAX_SCALE_W : positive := bits_for(VMAX_SCALE);
  constant SMALL_W      : positive := bits_for(ROT_PER_STEP);  -- width of AngleDiff when <= ROT_PER_STEP

  ------------------------------------------------------------------------------
  -- Signals
  ------------------------------------------------------------------------------
  type class_t is (REACHED, CLOSE, FAR);

  signal Valid1xS : std_logic := '0';
  signal Valid2xS : std_logic := '0';
  signal Valid3xS : std_logic := '0';

  -- stage 1
  signal DeltaxD : unsigned(ANGLE_W - 1 downto 0) := (others => '0');

  -- stage 2
  signal AngleDiffxD : unsigned(ANGLE_W - 1 downto 0) := (others => '0');
  signal DirxS       : std_logic                      := '0';

  -- stage 3
  signal StepProdxD  : unsigned(ANGLE_W + RECIP_Q_W - 1 downto 0)    := (others => '0');
  signal VMaxProdxD  : unsigned(SMALL_W + VMAX_SCALE_W - 1 downto 0) := (others => '0');
  signal ClassxS     : class_t                                       := REACHED;
  signal Dir3xS      : std_logic                                     := '0';

begin

  assert ROT_PER_STEP > 0 and ROT_PER_STEP * HALF <= 2 ** RECIP_SHIFT
    report "StepCompute: ROT_PER_STEP out of range for the reciprocal multiplication"
    severity failure;
  assert HALF / ROT_PER_STEP <= 2000
    report "StepCompute: StepNumberxDO range (0 to 2000) too small"
    severity failure;
  assert VMAX_EXT_INT < 2 ** 16
    report "StepCompute: VMAX_EXT does not fit in 16 bits"
    severity failure;

  process (ClkxCI)
    variable SmallV : unsigned(SMALL_W - 1 downto 0);
  begin
    if rising_edge(ClkxCI) then

      ------------------------------------------------------------------------
      -- Stage 1 : angular difference modulo 2**16 (target - measured)
      --   0 .. 2**15-1 : target is ahead (increasing angle -> counter-clockwise)
      --   2**15 .. 2**16-1 : target is behind (-> clockwise)
      ------------------------------------------------------------------------
      Valid1xS <= ValidAnglexSI;
      DeltaxD  <= AngleTargetxDI - AngleMeasuredxDI;

      ------------------------------------------------------------------------
      -- Stage 2 : shortest distance on the circle and rotation direction
      ------------------------------------------------------------------------
      Valid2xS <= Valid1xS;
      if DeltaxD(ANGLE_W - 1) = '0' then
        AngleDiffxD <= DeltaxD;
        DirxS       <= '1';             -- counter-clockwise
      else
        AngleDiffxD <= unsigned(-signed(DeltaxD));  -- 2**16 - delta (<= 2**15)
        DirxS       <= '0';             -- clockwise
      end if;

      ------------------------------------------------------------------------
      -- Stage 3 : reciprocal multiplications (RTG4 math blocks) + classification
      ------------------------------------------------------------------------
      Valid3xS <= Valid2xS;
      Dir3xS   <= DirxS;

      StepProdxD <= AngleDiffxD * to_unsigned(RECIP_Q, RECIP_Q_W);
      SmallV     := AngleDiffxD(SMALL_W - 1 downto 0);  -- only used when <= ROT_PER_STEP
      VMaxProdxD <= SmallV * to_unsigned(VMAX_SCALE, VMAX_SCALE_W);

      if AngleDiffxD > ROT_PER_STEP then
        ClassxS <= FAR;
      elsif AngleDiffxD > ROT_ACCURACY then
        ClassxS <= CLOSE;
      else
        ClassxS <= REACHED;
      end if;

      ------------------------------------------------------------------------
      -- Stage 4 : outputs
      ------------------------------------------------------------------------
      OutputValidxSO <= Valid3xS;

      if Valid3xS = '1' then            -- outputs only change with a new result
        RotDirectionxSO <= Dir3xS;
        case ClassxS is
          when FAR =>
            VMaxExtxDO    <= std_logic_vector(VMAX_EXT_SLV);
            StepNumberxDO <= to_integer(StepProdxD(RECIP_SHIFT + STEP_W - 1 downto RECIP_SHIFT));
          when CLOSE =>
            VMaxExtxDO    <= std_logic_vector(VMaxProdxD(VMAX_FRAC + 15 downto VMAX_FRAC));
            StepNumberxDO <= 1;
          when REACHED =>
            VMaxExtxDO    <= (others => '0');  -- don't care
            StepNumberxDO <= 0;
        end case;
      end if;

    end if;
  end process;

end architecture rtl;

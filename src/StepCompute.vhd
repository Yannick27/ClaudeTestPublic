--------------------------------------------------------------------------------
-- StepCompute
--
-- Computes, from a measured and a target angle (16 bit, full circle = 2^16),
-- the rotation direction, the number of full rotation steps and the amplitude
-- (VMax) of a last partial step.
--
-- Target : Microchip RTG4 FPGA (no DSP / multiplier needed, no hard divider
--          needed: the two divisions are done by a small shift/subtract
--          divider shared by both cases).
--
-- Latency: ValidAnglexSI -> OutputValidxSO is at most 36 clock cycles
--          (about 360 ns at 100 MHz). A ValidAnglexSI pulse that arrives while
--          a computation is running is ignored.
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

  constant MAX_STEPS : natural := 2000;  -- range of StepNumberxDO

  constant DIV_W : natural := 32;        -- dividend / quotient width
  constant DEN   : unsigned(16 downto 0) := to_unsigned(ROT_PER_STEP, 17);

  type state_t is (IDLE, CLASSIFY, DIVIDE, OUTPUT);
  signal State : state_t := IDLE;

  attribute syn_encoding : string;
  attribute syn_encoding of State : signal is "safe";  -- Synplify: SEU-safe FSM

  signal AngleDiffxD : unsigned(15 downto 0) := (others => '0');  -- |target - measured| on the circle
  signal DirxS       : std_logic             := '0';
  signal FarxS       : std_logic             := '0';  -- '1': far from target (full steps)
  signal ReachedxS   : std_logic             := '0';  -- '1': target reached

  -- shift / subtract divider
  signal NumQuotxD : unsigned(DIV_W - 1 downto 0) := (others => '0');  -- dividend shifted out, quotient shifted in
  signal RemxD     : unsigned(16 downto 0)        := (others => '0');
  signal CntxD     : natural range 0 to DIV_W - 1 := 0;

begin

  assert ROT_PER_STEP > 0 and ROT_PER_STEP < 2 ** 15
    report "ROT_PER_STEP out of supported range" severity failure;

  process (ClkxCI)
    variable DeltaxD   : unsigned(15 downto 0);
    variable ShiftedxD : unsigned(16 downto 0);
    variable StepsxD   : natural;
  begin
    if rising_edge(ClkxCI) then
      OutputValidxSO <= '0';

      case State is

        ----------------------------------------------------------------------
        -- wait for a new angle, compute shortest signed distance on the circle
        ----------------------------------------------------------------------
        when IDLE =>
          if ValidAnglexSI = '1' then
            DeltaxD := AngleTargetxDI - AngleMeasuredxDI;  -- wraps modulo 2^16
            if DeltaxD(15) = '1' and DeltaxD(14 downto 0) /= 0 then
              -- target is more than half a turn ahead: go clockwise
              DirxS       <= '0';
              AngleDiffxD <= 0 - DeltaxD;  -- 2^16 - Delta
            else
              DirxS       <= '1';
              AngleDiffxD <= DeltaxD;
            end if;
            State <= CLASSIFY;
          end if;

        ----------------------------------------------------------------------
        -- select the case and load the divider
        ----------------------------------------------------------------------
        when CLASSIFY =>
          CntxD <= 0;
          RemxD <= (others => '0');
          if AngleDiffxD > ROT_PER_STEP then
            -- far from target: steps = AngleDiff / ROT_PER_STEP
            FarxS     <= '1';
            ReachedxS <= '0';
            NumQuotxD <= resize(AngleDiffxD, DIV_W);
            State     <= DIVIDE;
          elsif AngleDiffxD > ROT_ACCURACY then
            -- close to target: VMax = (2^16 - 1) * AngleDiff / ROT_PER_STEP
            FarxS     <= '0';
            ReachedxS <= '0';
            NumQuotxD <= (AngleDiffxD & x"0000") - resize(AngleDiffxD, DIV_W);
            State     <= DIVIDE;
          else
            -- target reached, nothing to divide
            FarxS     <= '0';
            ReachedxS <= '1';
            NumQuotxD <= (others => '0');
            State     <= OUTPUT;
          end if;

        ----------------------------------------------------------------------
        -- restoring division by the constant ROT_PER_STEP, 1 bit per clock
        ----------------------------------------------------------------------
        when DIVIDE =>
          ShiftedxD := RemxD(15 downto 0) & NumQuotxD(DIV_W - 1);
          if ShiftedxD >= DEN then
            RemxD     <= ShiftedxD - DEN;
            NumQuotxD <= NumQuotxD(DIV_W - 2 downto 0) & '1';
          else
            RemxD     <= ShiftedxD;
            NumQuotxD <= NumQuotxD(DIV_W - 2 downto 0) & '0';
          end if;

          if CntxD = DIV_W - 1 then
            State <= OUTPUT;
          else
            CntxD <= CntxD + 1;
          end if;

        ----------------------------------------------------------------------
        -- register results, 1 cycle OutputValid pulse
        ----------------------------------------------------------------------
        when OUTPUT =>
          if ReachedxS = '1' then
            StepNumberxDO <= 0;
            VMaxExtxDO    <= (others => '0');
          elsif FarxS = '1' then
            RotDirectionxSO <= DirxS;
            VMaxExtxDO      <= (others => '1');
            if NumQuotxD > MAX_STEPS then  -- cannot happen (<= 2^15/ROT_PER_STEP), safety clamp
              StepNumberxDO <= MAX_STEPS;
            else
              StepNumberxDO <= to_integer(NumQuotxD(10 downto 0));
            end if;
          else
            RotDirectionxSO <= DirxS;
            VMaxExtxDO      <= std_logic_vector(NumQuotxD(15 downto 0));
            StepNumberxDO   <= 1;
          end if;
          OutputValidxSO <= '1';
          State          <= IDLE;

      end case;
    end if;
  end process;

end architecture rtl;

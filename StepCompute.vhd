-------------------------------------------------------------------------------
-- StepCompute
-- Computes rotation direction, number of full steps and partial-step amplitude
-- needed to go from AngleMeasured to AngleTarget (16-bit full-circle angles).
-- Increasing angle = counter-clockwise.
--
-- Target: Microchip RTG4 (no vendor primitives, no DSP/division operator needed;
-- the only "division" is a 32-cycle restoring divider by the constant
-- ROT_PER_STEP, which maps to a few LUTs and registers).
--
-- Latency: ValidAnglexSI -> OutputValidxSO = 34 clock cycles worst case
--          (2 cycles when target already reached).
-- If ValidAnglexSI arrives while a computation is running it is ignored.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity StepCompute is
  port (
    ClkxCI           : in  std_logic;   -- 100 MHz clock
    ValidAnglexSI    : in  std_logic;   -- angle is ready (1 clock cycle pulse)
    AngleMeasuredxDI : in  unsigned(15 downto 0);  -- measured angle (2^16 = 2*pi)
    AngleTargetxDI   : in  unsigned(15 downto 0);  -- target angle   (2^16 = 2*pi)
    VMaxExtxDO       : out std_logic_vector(15 downto 0) := (others => '0');
    RotDirectionxSO  : out std_logic                     := '0';  -- 0=CW, 1=CCW
    StepNumberxDO    : out natural range 0 to 2000       := 0;
    OutputValidxSO   : out std_logic                     := '0'   -- 1 clock pulse
    );
end entity StepCompute;

architecture rtl of StepCompute is

  constant ROT_PER_STEP_DEG : real    := 0.32;  -- in degree
  constant ROT_ACCURACY_DEG : real    := 0.01;  -- in degree
  constant ROT_ACCURACY     : integer := integer(ROT_ACCURACY_DEG / 360.0 * 2.0 ** 16);
  constant ROT_PER_STEP     : integer := integer(ROT_PER_STEP_DEG / 360.0 * 2.0 ** 16);

  constant DIVISOR : unsigned(16 downto 0) := to_unsigned(ROT_PER_STEP, 17);

  type state_t is (IDLE, DIVIDE, FINISH);
  signal StatexDP : state_t := IDLE;

  signal DirxDP      : std_logic := '0';
  signal PartialxSP  : std_logic := '0';  -- '1' = partial step (compute VMax)
  signal NumxDP      : unsigned(31 downto 0) := (others => '0');  -- numerator -> quotient
  signal RemxDP      : unsigned(16 downto 0) := (others => '0');
  signal CntxDP      : natural range 0 to 31 := 0;

begin

  process (ClkxCI)
    variable Diff_v    : unsigned(15 downto 0);
    variable Delta_v   : unsigned(15 downto 0);
    variable RemNext_v : unsigned(16 downto 0);
    variable Quot_v    : unsigned(31 downto 0);
  begin
    if rising_edge(ClkxCI) then
      OutputValidxSO <= '0';

      case StatexDP is

        when IDLE =>
          if ValidAnglexSI = '1' then
            -- signed circular difference (modulo 2^16): MSB set => target is
            -- more than half a turn ahead, i.e. shorter path is clockwise
            Delta_v := AngleTargetxDI - AngleMeasuredxDI;
            if Delta_v(15) = '0' then
              Diff_v := Delta_v;             -- counter-clockwise
              DirxDP <= '1';
            else
              Diff_v := 0 - Delta_v;         -- 2^16 - Delta (clockwise)
              DirxDP <= '0';
            end if;

            RemxDP <= (others => '0');
            CntxDP <= 0;

            if to_integer(Diff_v) > ROT_PER_STEP then
              -- far from target: steps = Diff / ROT_PER_STEP, full speed
              PartialxSP <= '0';
              NumxDP     <= x"0000" & Diff_v;
              StatexDP   <= DIVIDE;
            elsif to_integer(Diff_v) > ROT_ACCURACY then
              -- close: one step, VMax = (2^16-1) * Diff / ROT_PER_STEP
              PartialxSP <= '1';
              NumxDP     <= (Diff_v & x"0000") - resize(Diff_v, 32);  -- Diff*65535
              StatexDP   <= DIVIDE;
            else
              -- target reached
              VMaxExtxDO      <= (others => '0');
              StepNumberxDO   <= 0;
              OutputValidxSO  <= '1';
            end if;
          end if;

        when DIVIDE =>
          -- restoring division, one numerator bit per clock (MSB first)
          RemNext_v := RemxDP(15 downto 0) & NumxDP(31);
          if RemNext_v >= DIVISOR then
            RemxDP <= RemNext_v - DIVISOR;
            NumxDP <= NumxDP(30 downto 0) & '1';
          else
            RemxDP <= RemNext_v;
            NumxDP <= NumxDP(30 downto 0) & '0';
          end if;

          if CntxDP = 31 then
            StatexDP <= FINISH;
          else
            CntxDP <= CntxDP + 1;
          end if;

        when FINISH =>
          Quot_v := NumxDP;
          RotDirectionxSO <= DirxDP;
          if PartialxSP = '1' then
            StepNumberxDO <= 1;
            if Quot_v > 65535 then
              VMaxExtxDO <= (others => '1');
            else
              VMaxExtxDO <= std_logic_vector(Quot_v(15 downto 0));
            end if;
          else
            StepNumberxDO <= to_integer(Quot_v(10 downto 0));  -- <= 565 for 16-bit Diff
            VMaxExtxDO    <= (others => '1');                  -- 2^16 - 1
          end if;
          OutputValidxSO <= '1';
          StatexDP       <= IDLE;

      end case;
    end if;
  end process;

end architecture rtl;

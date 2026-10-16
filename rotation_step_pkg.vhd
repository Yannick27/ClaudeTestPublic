library ieee;
use ieee.math_real.all;

-- Testbench-only (not synthesizable)
package rotation_step_pkg is

  constant ROT_PER_STEP_DEG : real := 0.30;   -- rotation step in ° with VExt = VFULL_EXT
  constant ROT_ACCURACY_DEG : real := 0.01;   -- requested angle accuracy in °
  constant OVERSHOOT_MARGIN : real := 0.9;    -- margin applied to the number of rotation steps while far from target to prevent overshoot
  constant VMAX_EXT         : real := 150.0;  -- max extender voltage in V
  constant VFULL_EXT        : real := 140.0;  -- max recommended extender voltage in V
  constant ROT_PER_STEP_RAD : real := ROT_PER_STEP_DEG / 360.0 * MATH_2_PI;
  constant ROT_ACCURACY_RAD : real := ROT_ACCURACY_DEG / 360.0 * MATH_2_PI;

  -- Computes the next rotation command to bring AngleMeasured onto AngleTarget.
  -- Increasing angle is counter-clockwise.
  procedure compute_rotation_step(
    AngleTarget   : in  real;     -- target angle in rad, 0.0 .. MATH_2_PI
    AngleMeasured : in  real;     -- measured angle in rad, 0.0 .. MATH_2_PI
    VMaxExt       : out real;     -- extender voltage for one step (0.0 .. VFULL_EXT)
    CWrotation    : out boolean;  -- true = clockwise, false = counter-clockwise
    StepNumber    : out integer   -- number of rotation steps to apply
  );

end package rotation_step_pkg;

package body rotation_step_pkg is

  procedure compute_rotation_step(
    AngleTarget   : in  real;
    AngleMeasured : in  real;
    VMaxExt       : out real;
    CWrotation    : out boolean;
    StepNumber    : out integer
  ) is
    variable v_signed_diff : real;  -- target - measured, wrapped to [-pi, +pi]
    variable v_angle_diff  : real;  -- shortest angular distance, >= 0
    variable v_steps       : integer;
  begin
    -- Shortest signed distance, wrapped to [-pi, +pi]
    v_signed_diff := AngleTarget - AngleMeasured;
    if v_signed_diff > MATH_PI then
      v_signed_diff := v_signed_diff - MATH_2_PI;
    elsif v_signed_diff < -MATH_PI then
      v_signed_diff := v_signed_diff + MATH_2_PI;
    end if;
    v_angle_diff := abs(v_signed_diff);

    -- Angle must increase (diff > 0) => counter-clockwise; otherwise clockwise
    CWrotation := v_signed_diff < 0.0;

    if v_angle_diff > ROT_PER_STEP_RAD then
      -- Far from target: full-amplitude steps.
      -- floor() (not integer(), which rounds) so the margin really prevents overshoot.
      -- At least 1 step: a full step cannot overshoot here (diff > one step),
      -- whereas 0 steps would stall the loop when 1 < diff/step < 1/OVERSHOOT_MARGIN.
      VMaxExt    := VFULL_EXT;
      v_steps    := integer(floor(v_angle_diff / ROT_PER_STEP_RAD * OVERSHOOT_MARGIN));
      if v_steps < 1 then
        v_steps := 1;
      end if;
      StepNumber := v_steps;
    elsif v_angle_diff > ROT_ACCURACY_RAD then
      -- Close to target: one partial step
      VMaxExt    := v_angle_diff * VFULL_EXT / ROT_PER_STEP_RAD;
      StepNumber := 1;
    else
      -- Target reached (VMaxExt and CWrotation are don't care)
      VMaxExt    := 0.0;
      StepNumber := 0;
    end if;
  end procedure compute_rotation_step;

end package body rotation_step_pkg;

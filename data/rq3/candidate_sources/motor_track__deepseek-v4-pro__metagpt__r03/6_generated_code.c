/*
 * 6_generated_code.c
 *
 * Deterministic, heap-free implementation of the generated motor-track
 * planning and actuation API. This file intentionally avoids main(), tests,
 * and third-party dependencies.
 */

#include "6_generated_code.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*
 * PWM channel mapping.
 *
 * The fixed public header initializes four PWM channels. Channels are kept
 * explicit here to make the output phase relation clear:
 *   channel 0 -> left forward
 *   channel 1 -> left reverse
 *   channel 2 -> right forward
 *   channel 3 -> right reverse
 */
static const uint8_t kLeftForwardPwmChannel  = 0U;
static const uint8_t kLeftReversePwmChannel  = 1U;
static const uint8_t kRightForwardPwmChannel = 2U;
static const uint8_t kRightReversePwmChannel = 3U;

/* Planning and actuation limits. */
static const float kMaxDutyCycle    = 1.0F;
static const float kMaxPlannedSpeed = 100.0F;
static const float kMaxSteeringDiff = 0.8F;

/* Owned public state definitions. */
MotorChannel motor_l;
MotorChannel motor_r;
PIDState motor_pid_l;
PIDState motor_pid_r;
PIDState target_speed_pid;
PIDState posloop_pid;
float target_speed;
uint32_t clk;

/*
 * Static helper: clamp a float to an inclusive range.
 */
static float ClampFloat(float value, float lower, float upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

/*
 * Static helper: protect the steering angle by limiting the differential
 * between the two motor outputs while preserving the common-mode demand.
 *
 * This is the MT-R25 angle-protection step. The base saturation must already
 * have been applied before calling this helper.
 */
static void ApplyAngleProtection(float *left_output, float *right_output) {
  if (left_output == NULL || right_output == NULL) {
    return;
  }

  const float differential = *left_output - *right_output;
  const float excess = differential - kMaxSteeringDiff;

  if (excess > 0.0F) {
    *left_output -= excess * 0.5F;
    *right_output += excess * 0.5F;
  } else if (differential < -kMaxSteeringDiff) {
    const float negative_excess = -differential - kMaxSteeringDiff;
    *left_output += negative_excess * 0.5F;
    *right_output -= negative_excess * 0.5F;
  }

  *left_output = ClampFloat(*left_output, -kMaxDutyCycle, kMaxDutyCycle);
  *right_output = ClampFloat(*right_output, -kMaxDutyCycle, kMaxDutyCycle);
}

/*
 * Static helper: convert a signed actuation value into complementary
 * forward/reverse PWM duties and write them to the assigned channels.
 */
static void WriteComplementaryPwm(uint8_t forward_channel,
                                  uint8_t reverse_channel,
                                  float signed_output,
                                  float *forward_duty,
                                  float *reverse_duty) {
  signed_output = ClampFloat(signed_output, -kMaxDutyCycle, kMaxDutyCycle);

  if (signed_output >= 0.0F) {
    *forward_duty = signed_output;
    *reverse_duty = 0.0F;
  } else {
    *forward_duty = 0.0F;
    *reverse_duty = -signed_output;
  }

  pwm_duty(forward_channel, *forward_duty);
  pwm_duty(reverse_channel, *reverse_duty);
}

/*
 * Static helper: dispatch a single motor channel to the correct fixed PID
 * helper for MODE_NORMAL, MODE_BANGBANG, and MODE_SOFT. MODE_POSLOOP is
 * handled by the caller because it uses the shared position-loop PID.
 */
static float SolveSpeedPid(const MotorChannel *channel, PIDState *pid_state,
                           float error) {
  if (channel == NULL || pid_state == NULL) {
    return 0.0F;
  }

  switch (channel->mode) {
    case MODE_BANGBANG:
      return bangbang_pid_solve(pid_state, error);
    case MODE_SOFT:
      return changable_pid_solve(pid_state, error);
    case MODE_NORMAL:
    default:
      return pid_solve(pid_state, error);
  }
}

/*
 * Static helper: apply target-speed planning.
 *
 * MT-R01 through MT-R07 require the owned target-speed PID to participate in
 * speed_control before the final target is stored. The PID is used as an
 * incremental smoother around the current planned target. If no gains are
 * configured, the desired speed is passed through unchanged so that the
 * planner remains deterministic and usable with default zeroed PID state.
 */
static float ApplyTargetSpeedPlanning(float desired_speed,
                                      float current_speed) {
  const bool has_active_gains =
      (target_speed_pid.kp != 0.0F) ||
      (target_speed_pid.ki != 0.0F) ||
      (target_speed_pid.kd != 0.0F);

  if (!has_active_gains) {
    return desired_speed;
  }

  const float error = desired_speed - current_speed;
  const float control_output = pid_solve(&target_speed_pid, error);
  return current_speed + control_output;
}

/*
 * Static helper: resolve the MT-R16 / MT-R17 / MT-R18 mode policy.
 *
 * - MT-R17: safety-critical inputs force the normal mode.
 * - MT-R18: no newly detected scenario retains the previous mode.
 * - MT-R16: a newly detected scenario updates to the selected mode.
 *
 * previous_steering_diff is used as a consistency guard. If the stored
 * wheel targets are outside the planner speed range, the owned state is
 * invalid and the mode is reset to normal.
 */
static int ResolveStoredMode(int selected_mode, int previous_mode,
                             bool scenario_detected, bool safety_critical,
                             float previous_steering_diff) {
  if (safety_critical || fabsf(previous_steering_diff) > kMaxPlannedSpeed) {
    return MODE_NORMAL;
  }

  if (!scenario_detected) {
    return previous_mode;
  }

  return selected_mode;
}

/*
 * motor_init
 *
 * MT-R03: initializes four PWM channels at 17000 Hz with zero duty and
 * zeroes all owned planner/controller state.
 */
void motor_init(void) {
  uint8_t channel;

  memset(&motor_l, 0, sizeof(motor_l));
  memset(&motor_r, 0, sizeof(motor_r));
  memset(&motor_pid_l, 0, sizeof(motor_pid_l));
  memset(&motor_pid_r, 0, sizeof(motor_pid_r));
  memset(&target_speed_pid, 0, sizeof(target_speed_pid));
  memset(&posloop_pid, 0, sizeof(posloop_pid));
  target_speed = 0.0F;
  clk = 0U;

  /*
   * Do not rely on MODE_NORMAL being represented as zero. Set the mode
   * explicitly after zeroing so motor_init remains correct for any enum
   * values supplied by the fixed public header.
   */
  motor_l.mode = MODE_NORMAL;
  motor_r.mode = MODE_NORMAL;

  pwm_init(17000U, 4U);

  for (channel = 0U; channel < 4U; ++channel) {
    pwm_duty(channel, 0.0F);
  }
}

/*
 * speed_control
 *
 * Reads scenario inputs, computes planned targets and mode, executes the
 * MT-R08 through MT-R18 decision chain, applies the MT-R19 final safety
 * override, and stores only the planned values in the owned global state.
 * No PWM output is written here.
 */
void speed_control(void) {
  float raw_target_speed = 0.0F;
  float planned_target_speed = 0.0F;
  float planned_left_target = 0.0F;
  float planned_right_target = 0.0F;
  int selected_mode = MODE_NORMAL;

  const int previous_mode = motor_l.mode;
  const float previous_steering_diff =
      motor_l.target_speed - motor_r.target_speed;

  const bool animal_active = (animal != 0);
  const bool garage_active = (garage != 0);
  const bool tag_active = (tag != 0);
  const bool apriltag_active = (apriltag != 0);
  const bool ramp_active = (ramp != 0);
  const bool y_road_active = (y_road != 0);
  const bool circle_active = (circle != 0);
  const float lookahead_active = lookahead;
  const bool safety_critical = animal_active || garage_active;
  bool scenario_detected = false;

  /*
   * MT-R08 through MT-R18 decision chain.
   *
   * Safety-related situations have the highest priority. The remaining
   * branches implement the fixed scenario priority from the public contract.
   */
  if (animal_active || garage_active) {
    /* MT-R08 / MT-R09: stop immediately. */
    raw_target_speed = 0.0F;
    planned_left_target = 0.0F;
    planned_right_target = 0.0F;
    selected_mode = MODE_NORMAL;
    scenario_detected = true;
  } else if (tag_active) {
    /* MT-R10: generic tag following uses a conservative symmetric target. */
    raw_target_speed = 25.0F;
    planned_left_target = raw_target_speed;
    planned_right_target = raw_target_speed;
    selected_mode = MODE_NORMAL;
    scenario_detected = true;
  } else if (apriltag_active) {
    /* MT-R11: AprilTag engagement uses the position loop. */
    raw_target_speed = lookahead_active;
    planned_left_target = raw_target_speed;
    planned_right_target = raw_target_speed;
    selected_mode = MODE_POSLOOP;
    scenario_detected = true;
  } else if (ramp_active) {
    /* MT-R12: ramp traversal uses the soft/changeable PID mode. */
    raw_target_speed = 20.0F;
    planned_left_target = raw_target_speed;
    planned_right_target = raw_target_speed;
    selected_mode = MODE_SOFT;
    scenario_detected = true;
  } else if (y_road_active) {
    /* MT-R13: Y-road demands a stable differential steering target. */
    raw_target_speed = 22.0F;
    planned_left_target = 26.0F;
    planned_right_target = 18.0F;
    selected_mode = MODE_NORMAL;
    scenario_detected = true;
  } else if (circle_active) {
    /* MT-R14: circle following uses a fixed left/right speed offset. */
    raw_target_speed = 20.0F;
    planned_left_target = 24.0F;
    planned_right_target = 16.0F;
    selected_mode = MODE_NORMAL;
    scenario_detected = true;
  } else if (lookahead_active > 0.0F) {
    /* MT-R15: lookahead distance shapes the forward speed. */
    raw_target_speed = 15.0F + lookahead_active * 0.1F;
    planned_left_target = raw_target_speed;
    planned_right_target = raw_target_speed;
    selected_mode = MODE_NORMAL;
    scenario_detected = true;
  } else {
    /* MT-R18 fallback: no scenario active, retain safe zero planning. */
    raw_target_speed = 0.0F;
    planned_left_target = 0.0F;
    planned_right_target = 0.0F;
    selected_mode = MODE_NORMAL;
    scenario_detected = false;
  }

  /*
   * MT-R01 through MT-R07 target-speed planning update.
   */
  planned_target_speed =
      ApplyTargetSpeedPlanning(raw_target_speed, target_speed);

  /*
   * MT-R16 through MT-R18 mode retention/fallback/update policy.
   */
  selected_mode = ResolveStoredMode(selected_mode,
                                    previous_mode,
                                    scenario_detected,
                                    safety_critical,
                                    previous_steering_diff);

  /*
   * MT-R19 final safety override.
   *
   * The override clamps all planned speeds to a safe range and forces a stop
   * if a safety-critical situation is active.
   */
  if (safety_critical) {
    planned_target_speed = 0.0F;
    planned_left_target = 0.0F;
    planned_right_target = 0.0F;
    selected_mode = MODE_NORMAL;
  } else {
    planned_target_speed =
        ClampFloat(planned_target_speed, -kMaxPlannedSpeed, kMaxPlannedSpeed);
    planned_left_target =
        ClampFloat(planned_left_target, -kMaxPlannedSpeed, kMaxPlannedSpeed);
    planned_right_target =
        ClampFloat(planned_right_target, -kMaxPlannedSpeed, kMaxPlannedSpeed);
  }

  /*
   * Store only the planned targets and mode in the owned actuator state.
   * PWM outputs, motor PID states, and encoder values are intentionally not
   * touched here.
   */
  target_speed = planned_target_speed;
  motor_l.target_speed = planned_left_target;
  motor_r.target_speed = planned_right_target;
  motor_l.mode = selected_mode;
  motor_r.mode = selected_mode;
}

/*
 * motor_control
 *
 * Consumes stored planned targets and modes, dispatches the appropriate PID
 * helper, applies saturation and angle protection, and writes complementary
 * forward/reverse PWM duty values on every call.
 */
void motor_control(void) {
  float left_output = 0.0F;
  float right_output = 0.0F;

  /* Advance the owned time base by one control step. */
  ++clk;

  if (motor_l.mode == MODE_POSLOOP || motor_r.mode == MODE_POSLOOP) {
    /*
     * MT-R23: position-loop mode uses the shared position PID and the total
     * encoder feedback.
     */
    const float position_error = target_speed - get_total_encoder();
    const float position_output = pid_solve(&posloop_pid, position_error);
    left_output = position_output;
    right_output = position_output;
  } else {
    /*
     * MT-R20 / MT-R21 / MT-R22: speed-based modes use the per-motor PID
     * states. The fixed helpers are selected by the stored mode value.
     */
    const float left_error =
        motor_l.target_speed - (float)motor_l.encoder;
    const float right_error =
        motor_r.target_speed - (float)motor_r.encoder;

    left_output = SolveSpeedPid(&motor_l, &motor_pid_l, left_error);
    right_output = SolveSpeedPid(&motor_r, &motor_pid_r, right_error);
  }

  /*
   * MT-R24: base saturation.
   */
  left_output = ClampFloat(left_output, -kMaxDutyCycle, kMaxDutyCycle);
  right_output = ClampFloat(right_output, -kMaxDutyCycle, kMaxDutyCycle);

  /*
   * MT-R25: steering angle protection.
   */
  ApplyAngleProtection(&left_output, &right_output);

  /*
   * MT-R26: write complementary forward/reverse PWM duties.
   */
  WriteComplementaryPwm(kLeftForwardPwmChannel, kLeftReversePwmChannel,
                        left_output, &motor_l.forward_pwm,
                        &motor_l.reverse_pwm);
  WriteComplementaryPwm(kRightForwardPwmChannel, kRightReversePwmChannel,
                        right_output, &motor_r.forward_pwm,
                        &motor_r.reverse_pwm);
}

/*
 * get_total_encoder
 *
 * Pure utility: returns the sum of the two motor encoder values. This
 * function does not mutate planner, controller, or PWM state.
 */
float get_total_encoder(void) {
  return (float)motor_l.encoder + (float)motor_r.encoder;
}

/*
 * radius_3pts
 *
 * Pure utility: computes the radius of the circle passing through three
 * points. Returns 0.0F for degenerate/collinear input.
 */
float radius_3pts(float x1, float y1, float x2, float y2, float x3,
                  float y3) {
  const float a = hypotf(x2 - x1, y2 - y1);
  const float b = hypotf(x3 - x2, y3 - y2);
  const float c = hypotf(x1 - x3, y1 - y3);

  const float signed_area_twice =
      x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
  const float area = fabsf(signed_area_twice) * 0.5F;

  if (area <= 0.0F || a <= 0.0F || b <= 0.0F || c <= 0.0F) {
    return 0.0F;
  }

  return (a * b * c) / (4.0F * area);
}

/*
 * square_signal
 *
 * Pure utility: generates a deterministic square-wave signal with the given
 * period and duty cycle. The high level is +1.0F and the low level is -1.0F.
 * The function clamps duty to [0.0F, 1.0F] and returns 0.0F for non-positive
 * periods or non-finite inputs.
 */
float square_signal(float time, float period, float duty) {
  float cycle_time;

  if (!isfinite(time) || !isfinite(period) || !isfinite(duty)) {
    return 0.0F;
  }

  if (period <= 0.0F) {
    return 0.0F;
  }

  duty = ClampFloat(duty, 0.0F, 1.0F);

  cycle_time = fmodf(time, period);
  if (cycle_time < 0.0F) {
    cycle_time += period;
  }

  return (cycle_time < duty * period) ? 1.0F : -1.0F;
}

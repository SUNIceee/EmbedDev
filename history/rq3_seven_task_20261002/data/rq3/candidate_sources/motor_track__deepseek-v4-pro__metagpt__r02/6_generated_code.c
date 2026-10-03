/* 6_generated_code.c
 *
 * Implementation of the fixed generated motor API. This file intentionally
 * contains only application-owned state and the public entry points declared
 * in 6_generated_code.h. It does not include any third-party code.
 */
#include "6_generated_code.h"

#include <math.h>
#include <stdint.h>

/*
 * Fallback constants. The generated header normally supplies these symbols;
 * the guards below keep this translation unit self-contained if the header
 * only declares the related enums or omits them entirely.
 */
#ifndef ANIMAL
#define ANIMAL 1
#endif

#ifndef TAG_SEARCH
#define TAG_SEARCH 0
#endif

#ifndef TAG_STOP
#define TAG_STOP 1
#endif

#ifndef TAG_SHOOTING
#define TAG_SHOOTING 2
#endif

#ifndef APRILTAG_FOUND
#define APRILTAG_FOUND 3
#endif

#ifndef APRILTAG_MAYBE
#define APRILTAG_MAYBE 4
#endif

#ifndef GARAGE_STOP
#define GARAGE_STOP 0
#endif

#ifndef GARAGE_IN
#define GARAGE_IN 1
#endif

#ifndef GARAGE_OUT
#define GARAGE_OUT 2
#endif

#ifndef YROAD_NEAR
#define YROAD_NEAR 1
#endif

#ifndef YROAD_FOUND
#define YROAD_FOUND 2
#endif

/*
 * PWM channel mapping. The exact mapping is fixed by the generated header;
 * this implementation assumes the conventional four-channel layout:
 *   0 - left forward
 *   1 - left reverse
 *   2 - right forward
 *   3 - right reverse
 */
static const uint32_t kPwmLeftForward = 0U;
static const uint32_t kPwmLeftReverse = 1U;
static const uint32_t kPwmRightForward = 2U;
static const uint32_t kPwmRightReverse = 3U;

/* Application-owned globals. */
motor_environment_t motor_env = {0};
wheel_t motor_l = {0, MODE_NORMAL, 0.0f, 0.0f, 0.0f};
wheel_t motor_r = {0, MODE_NORMAL, 0.0f, 0.0f, 0.0f};
PIDController motor_pid_l = {7021.0f, 10.0f, 0.0f, 0.0f,
                             0.0f, 50000.0f, 50000.0f};
PIDController motor_pid_r = {7021.0f, 10.0f, 0.0f, 0.0f,
                             0.0f, 50000.0f, 50000.0f};
PIDController target_speed_pid = {5.0f, 0.0f, 30.0f, 0.0f,
                                  0.0f, 5.0f, 5.0f};
PIDController posloop_pid = {200.0f, 0.0f, 0.0f, 0.0f,
                             0.0f, 50000.0f, 50000.0f};

int32_t target_speed = 0;
uint32_t clk = 0U;
int64_t target_encoder = 0;

/*
 * The public target_speed and wheel_t.target_speed fields are int32_t in the
 * frozen header. Speed-control decisions are therefore planned in this
 * file-local float while the public fields hold the nearest integer
 * approximation. This keeps fractional logic such as 0.5 and the 0.02
 * per-cycle circle ramp representable without changing the public ABI.
 */
static float target_speed_planner = 0.0f;

static void InitializePidController(PIDController* controller, float kp,
                                    float ki, float kd, float max_output,
                                    float max_integral) {
  controller->kp = kp;
  controller->ki = ki;
  controller->kd = kd;
  controller->integral = 0.0f;
  controller->previous_error = 0.0f;
  controller->max_output = max_output;
  controller->max_integral = max_integral;
}

static void ResetWheel(wheel_t* wheel) {
  wheel->target_speed = 0;
  wheel->mode = MODE_NORMAL;
  wheel->previous_duty = 0.0f;
  wheel->brake_pid = 0.0f;
  wheel->pid = 0.0f;
}

static float ClampFloat(float value, float lower, float upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

static int32_t RoundToInt32(float value) {
  /* lroundf rounds half away from zero and is defined in C99/C11. */
  return (int32_t)lroundf(value);
}

static float ComputeDiffFromLaserAngle(float laser_angle) {
  const float kPi = 3.14159265358979323846f;
  return 15.8f * tanf(laser_angle * kPi * 2.4f / 180.0f) / 40.0f / 2.0f;
}

static int64_t TruncatedAverage(int64_t left, int64_t right) {
  if ((left >= 0 && right >= 0) || (left < 0 && right < 0)) {
    return left / 2 + right / 2 + (((left % 2) + (right % 2)) / 2);
  }

  /*
   * Mixed signs cannot overflow int64_t when summed, and this path preserves
   * C11 truncation-toward-zero semantics for the average.
   */
  return (left + right) / 2;
}

static float ComputeWheelDuty(wheel_t* wheel, PIDController* speed_pid,
                              PIDController* position_pid, float speed_error) {
  switch (wheel->mode) {
    case MODE_NORMAL:
      return pid_solve(speed_pid, speed_error);

    case MODE_BANGBANG:
      speed_pid->integral = 0.0f;
      return wheel->previous_duty +
             bangbang_pid_solve(&wheel->brake_pid, speed_error);

    case MODE_SOFT:
      speed_pid->integral = 0.0f;
      return wheel->previous_duty +
             changable_pid_solve(&wheel->pid, speed_error);

    case MODE_POSLOOP: {
      const int64_t total_encoder = get_total_encoder();
      const float position_error = (float)(target_encoder - total_encoder);
      speed_pid->integral = 0.0f;
      return pid_solve(position_pid, position_error);
    }

    default:
      speed_pid->integral = 0.0f;
      return 0.0f;
  }
}

static void WritePwmDuty(uint32_t forward_channel, uint32_t reverse_channel,
                         float duty) {
  if (duty >= 0.0f) {
    pwm_duty(forward_channel, (uint32_t)duty);
    pwm_duty(reverse_channel, 0U);
  } else {
    pwm_duty(forward_channel, 0U);
    pwm_duty(reverse_channel, (uint32_t)(-duty));
  }
}

void motor_init(void) {
  pwm_init(kPwmLeftForward, 17000U, 0U);
  pwm_init(kPwmLeftReverse, 17000U, 0U);
  pwm_init(kPwmRightForward, 17000U, 0U);
  pwm_init(kPwmRightReverse, 17000U, 0U);

  ResetWheel(&motor_l);
  ResetWheel(&motor_r);

  InitializePidController(&motor_pid_l, 7021.0f, 10.0f, 0.0f, 50000.0f,
                          50000.0f);
  InitializePidController(&motor_pid_r, 7021.0f, 10.0f, 0.0f, 50000.0f,
                          50000.0f);
  InitializePidController(&target_speed_pid, 5.0f, 0.0f, 30.0f, 5.0f, 5.0f);
  InitializePidController(&posloop_pid, 200.0f, 0.0f, 0.0f, 50000.0f,
                          50000.0f);

  target_speed = 0;
  target_speed_planner = 0.0f;
  clk = 0U;
  target_encoder = 0;
}

void speed_control(void) {
  /*
   * Planning stage only. Wheel modes and differential steering are computed
   * here, and only wheel target speeds are stored. No PWM writes are allowed.
   */
  motor_l.mode = MODE_NORMAL;
  motor_r.mode = MODE_NORMAL;

  float diff = ComputeDiffFromLaserAngle(motor_env.laser_angle);
  float planned_speed = target_speed_planner;

  /* R08: animal stop. */
  if (motor_env.fa_type == ANIMAL &&
      (uint32_t)(motor_env.now_ms - motor_env.animal_time_ms) < 2500U) {
    planned_speed = 0.0f;
    diff = 0.0f;
  } else if (motor_env.fruit_delta < 0.0f &&
             (motor_env.laser_angle < 5.0f ||
              motor_env.laser_angle > 175.0f)) {
    /* R09: fruit-laser reverse. */
    planned_speed = -1.0f;
    diff = 0.0f;
  } else if (motor_env.tag_state == TAG_SEARCH) {
    /* R10: tag search. */
    planned_speed = 1.0f;
  } else if (motor_env.tag_state == TAG_STOP ||
             motor_env.tag_state == TAG_SHOOTING) {
    /* R10: tag stop/shooting. */
    planned_speed = 0.0f;
  } else if (motor_env.tag_state == APRILTAG_FOUND) {
    /* R11: apriltag found. The integer public target becomes 1. */
    planned_speed = 0.5f;
    diff = 0.0f;
  } else if (motor_env.tag_state == APRILTAG_MAYBE) {
    /* R11: apriltag maybe. */
    planned_speed = 1.0f;
  } else if (motor_env.garage_direction == GARAGE_OUT) {
    /* R12: garage out direction. */
    planned_speed = 14.0f;
    motor_l.mode = MODE_SOFT;
    motor_r.mode = MODE_SOFT;
  } else if (motor_env.garage_direction == GARAGE_IN) {
    /* R12: garage in direction. */
    planned_speed = 10.0f;
    motor_l.mode = MODE_NORMAL;
    motor_r.mode = MODE_NORMAL;
  } else if (motor_env.enable_adc != 0) {
    /* R13: ADC bangbang mode. */
    planned_speed = 9.0f;
    motor_l.mode = MODE_BANGBANG;
    motor_r.mode = MODE_BANGBANG;
  } else if (motor_env.yroad_state == YROAD_NEAR ||
             motor_env.yroad_state == YROAD_FOUND) {
    /* R14: yroad detected. */
    planned_speed = 3.0f;
  } else if (motor_env.rptsn_num == 0) {
    /* R15: persistent circle ramp with no path points. */
    planned_speed = ClampFloat(planned_speed - 0.02f, 11.0f, 17.0f);
  } else if (motor_env.rptsn_num > 20) {
    /* R16: full lookahead. */
    int32_t id = motor_env.rptsn_num - 1;
    if (id >= 70) {
      id = 69;
    }
    const float x_diff = motor_env.rptsn[id][0] - motor_env.rptsn[0][0];
    const float y_diff = motor_env.rptsn[id][1] - motor_env.rptsn[0][1];
    const float safe_y_diff = (y_diff == 0.0f) ? 1.0e-6f : y_diff;
    const float lookahead_error = fabsf(x_diff / safe_y_diff);
    const float pid_result = pid_solve(&target_speed_pid, lookahead_error);
    planned_speed = ClampFloat(17.0f - pid_result, 9.0f, 17.0f);
  } else if (motor_env.rptsn_num > 5) {
    /* R17: degraded lookahead for 5 < rptsn_num <= 20. */
    planned_speed = 9.0f;
  } else {
    /*
     * R18: no matching decision. For 1 <= rptsn_num <= 5 retain the previous
     * scalar target speed. Diff and normal wheel modes were already restored
     * above.
     */
    motor_l.mode = MODE_NORMAL;
    motor_r.mode = MODE_NORMAL;
  }

  /* R19: final unconditional safety override. */
  if (motor_env.garage == GARAGE_STOP ||
      (motor_env.garage_direction != GARAGE_OUT &&
       motor_env.elec_data[0] + motor_env.elec_data[1] < 60.0f)) {
    planned_speed = 0.0f;
    diff = 0.0f;
    motor_l.mode = MODE_NORMAL;
    motor_r.mode = MODE_NORMAL;
  }

  target_speed_planner = planned_speed;
  target_speed = RoundToInt32(planned_speed);

  const float left_target = planned_speed - diff * planned_speed;
  const float right_target = planned_speed + diff * planned_speed;
  motor_l.target_speed = RoundToInt32(left_target);
  motor_r.target_speed = RoundToInt32(right_target);
}

void motor_control(void) {
  /*
   * Actuation stage only. Consume the stored wheel targets and modes, compute
   * control duties, apply saturation, and write four complementary PWM
   * channels.
   */
  const float left_error =
      (float)motor_l.target_speed - motor_env.encoder_speed_left;
  const float right_error =
      (float)motor_r.target_speed - motor_env.encoder_speed_right;

  float left_duty =
      ComputeWheelDuty(&motor_l, &motor_pid_l, &posloop_pid, left_error);
  float right_duty =
      ComputeWheelDuty(&motor_r, &motor_pid_r, &posloop_pid, right_error);

  left_duty = ClampFloat(left_duty, -50000.0f, 50000.0f);
  right_duty = ClampFloat(right_duty, -50000.0f, 50000.0f);

  const float laser_angle = motor_env.laser_angle;
  const float average_encoder_speed =
      (motor_env.encoder_speed_left + motor_env.encoder_speed_right) / 2.0f;

  if (fabsf(laser_angle) > 10.0f) {
    if (target_speed_planner - average_encoder_speed < 0.0f) {
      left_duty = ClampFloat(left_duty, -50000.0f, 40000.0f);
      right_duty = ClampFloat(right_duty, -50000.0f, 40000.0f);
    } else {
      left_duty = ClampFloat(left_duty, -40000.0f, 40000.0f);
      right_duty = ClampFloat(right_duty, -40000.0f, 40000.0f);
    }
  }

  motor_l.previous_duty = left_duty;
  motor_r.previous_duty = right_duty;

  WritePwmDuty(kPwmLeftForward, kPwmLeftReverse, left_duty);
  WritePwmDuty(kPwmRightForward, kPwmRightReverse, right_duty);
}

int64_t get_total_encoder(void) {
  return TruncatedAverage(motor_env.encoder_left, motor_env.encoder_right);
}

double radius_3pts(Point a, Point b, Point c) {
  const double ax = (double)a.x;
  const double ay = (double)a.y;
  const double bx = (double)b.x;
  const double by = (double)b.y;
  const double cx = (double)c.x;
  const double cy = (double)c.y;

  const double ab_x = bx - ax;
  const double ab_y = by - ay;
  const double ac_x = cx - ax;
  const double ac_y = cy - ay;

  const double side_ab = sqrt(ab_x * ab_x + ab_y * ab_y);
  const double side_bc = sqrt((cx - bx) * (cx - bx) + (cy - by) * (cy - by));
  const double side_ca = sqrt((ax - cx) * (ax - cx) + (ay - cy) * (ay - cy));

  const double cross = ab_x * ac_y - ab_y * ac_x;
  const double area = fabs(cross) * 0.5;

  if (area < 1.0e-12) {
    return 0.0;
  }

  return side_ab * side_bc * side_ca / (4.0 * area);
}

void square_signal(void) {
  clk = clk + 1U;
  if (clk > 10000U) {
    clk = 0U;
  }

  if (clk < 2000U) {
    motor_l.target_speed = 20;
    motor_r.target_speed = 20;
  } else if (clk < 4000U) {
    motor_l.target_speed = 0;
    motor_r.target_speed = 0;
  } else if (clk < 6000U) {
    motor_l.target_speed = 15;
    motor_r.target_speed = 15;
  } else if (clk < 8000U) {
    motor_l.target_speed = 28;
    motor_r.target_speed = 28;
  } else if (clk < 10000U) {
    motor_l.target_speed = 0;
    motor_r.target_speed = 0;
  } else {
    /* clk == 10000U: retain the previous wheel target speeds. */
  }
}

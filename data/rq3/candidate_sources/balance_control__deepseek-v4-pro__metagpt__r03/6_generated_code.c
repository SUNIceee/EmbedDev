/*
 * Copyright (c) 2025. All rights reserved.
 *
 * Balance-control implementation for 6_generated_code.h.
 *
 * This file contains the private state machines and public control functions.
 * It avoids third-party dependencies and uses only the fixed-width integer
 * types and the platform callbacks declared in the public header.
 */

#include "6_generated_code.h"

#include <stdint.h>

/*
 * Public globals are defined here and are zero-initialized as a safe default.
 * control_init() also clears them explicitly to make reset behavior obvious.
 */
float pitch = 0.0F;
float roll = 0.0F;
float yaw = 0.0F;

int16_t gyrox = 0;
int16_t gyroy = 0;
int16_t gyroz = 0;
int16_t state = 0;

int32_t speed = 0;
int32_t ac_pwm = 0;
int32_t vc_pwm = 0;
int32_t dc_pwm = 0;
int32_t left_pwm = 0;
int32_t right_pwm = 0;

/*
 * Private velocity PID state.
 *
 * e1 and e2 store the two previous error samples, accumulator stores the
 * integrated PID output, and old/new/count implement the 5-call decimation
 * and linear interpolation schedule.
 */
static float velocity_e1 = 0.0F;
static float velocity_e2 = 0.0F;
static int32_t velocity_accumulator = 0;
static int32_t velocity_old = 0;
static int32_t velocity_new = 0;
static int32_t velocity_count = 0;

/*
 * Private direction state.
 *
 * previous_filtered stores the last low-pass filtered gyroz value, and
 * old/new/count implement the independent 5-call direction schedule.
 */
static int16_t direction_previous_filtered = 0;
static int32_t direction_old = 0;
static int32_t direction_new = 0;
static int32_t direction_count = 0;

/* Private utility helpers. */

static int32_t ClampInt32(int32_t value, int32_t min_value,
                          int32_t max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static float ClampFloat(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static int32_t AbsInt32(int32_t value) {
  if (value < 0) {
    /* Cast to int64_t first to avoid overflow on INT32_MIN. */
    return (int32_t)(-(int64_t)value);
  }
  return value;
}

void control_init(void) {
  /* Clear all public sensor and output globals. */
  pitch = 0.0F;
  roll = 0.0F;
  yaw = 0.0F;

  gyrox = 0;
  gyroy = 0;
  gyroz = 0;
  state = 0;

  speed = 0;
  ac_pwm = 0;
  vc_pwm = 0;
  dc_pwm = 0;
  left_pwm = 0;
  right_pwm = 0;

  /* Clear private velocity PID state. */
  velocity_e1 = 0.0F;
  velocity_e2 = 0.0F;
  velocity_accumulator = 0;
  velocity_old = 0;
  velocity_new = 0;
  velocity_count = 0;

  /* Clear private direction state. */
  direction_previous_filtered = 0;
  direction_old = 0;
  direction_new = 0;
  direction_count = 0;
}

void get_mpu(void) {
  /*
   * The platform callback writes the six sensor measurements directly into
   * the public globals, so no additional copy is required.
   */
  board_read_imu(&pitch, &roll, &yaw, &gyrox, &gyroy, &gyroz);
}

int32_t get_speed(void) {
  const int32_t left_count = (int32_t)board_encoder_read(0);
  const int32_t right_count = (int32_t)board_encoder_read(1);

  board_encoder_clear(0);
  board_encoder_clear(1);

  return (AbsInt32(left_count) + AbsInt32(right_count)) / 2;
}

int32_t angle_proc(void) {
  /*
   * Angle calculation uses the current pitch and gyroy values. The explicit
   * cast to int32_t truncates toward zero, matching the required C semantics.
   */
  return (int32_t)(-(550.0F * (0.0F - pitch) - 2.0F * gyroy));
}

int32_t velocity_proc(int32_t measured_speed) {
  /*
   * Velocity PID runs on a 5-call decimation schedule. On each fifth entry,
   * update the PID accumulator. Between updates, linear interpolation is
   * returned by the old/new/count expression.
   */
  if (velocity_count >= 5) {
    velocity_count = 0;
    velocity_old = velocity_new;

    const float error = 0.0F - (float)measured_speed;
    const float p_term = 8.0F * (error - velocity_e1);
    float i_term = 0.1F * error;
    const float d_term =
        0.5F * (error - (2.0F * velocity_e1) + velocity_e2);

    /* Zero the integral term when it exceeds the allowed range. */
    if (i_term > 10.0F || i_term < -10.0F) {
      i_term = 0.0F;
    }

    velocity_accumulator =
        (int32_t)((float)velocity_accumulator + p_term + i_term + d_term);
    velocity_accumulator = ClampInt32(velocity_accumulator, -4500, 4500);

    velocity_e2 = velocity_e1;
    velocity_e1 = error;
    velocity_new = velocity_accumulator;
  }

  velocity_count++;
  return velocity_old + ((velocity_new - velocity_old) * velocity_count) / 5;
}

int32_t direction_proc(int32_t measured_speed) {
  /*
   * First filter the current gyroz value and immediately refresh the public
   * gyroz sample so later D-term calculation uses the filtered value.
   */
  const int16_t filtered_gyroz =
      (int16_t)(0.9F * (float)gyroz +
                0.1F * (float)direction_previous_filtered);
  gyroz = filtered_gyroz;
  direction_previous_filtered = filtered_gyroz;

  /*
   * Direction control also uses an independent 5-call decimation loop.
   */
  if (direction_count >= 5) {
    direction_count = 0;
    direction_old = direction_new;

    /* Read the platform direction error exactly once per update. */
    state = board_direction_error();

    const float p_gain =
        ClampFloat(0.6F * (float)measured_speed, 50.0F, 500.0F);
    const int32_t p_term =
        (int32_t)(p_gain * (float)state / 25.0F);
    const int32_t d_term = (int32_t)(1.5F * (float)gyroz / 100.0F);

    direction_new = ClampInt32(p_term + d_term, -3000, 3000);
  }

  direction_count++;
  return direction_old + ((direction_new - direction_old) * direction_count) / 5;
}

void get_pwm(void) {
  /*
   * get_pwm() is the single fusion/orchestration entry point. It refreshes
   * sensor inputs, derives measured speed, runs the three control paths, and
   * publishes the resulting PWM snapshots.
   */
  get_mpu();

  const int32_t measured_speed = get_speed();
  speed = measured_speed;

  const int32_t ac = angle_proc();
  const int32_t vc = velocity_proc(speed);
  const int32_t dc = direction_proc(speed);

  ac_pwm = ac;
  vc_pwm = vc;
  dc_pwm = dc;

  left_pwm = ac - vc + dc;
  right_pwm = ac - vc - dc;
}

void motor_proc(int32_t left, int32_t right) {
  /*
   * motor_proc() is intentionally isolated from the public PWM globals. It
   * receives explicit left and right drive values, clamps them to the
   * motor-controller duty range, and writes the corresponding direction and
   * duty commands through the platform callback.
   */
  const int32_t left_clamped = ClampInt32(left, -7200, 7200);
  const int32_t right_clamped = ClampInt32(right, -7200, 7200);

  if (left_clamped >= 0) {
    board_motor_write(0, 0, 1, left_clamped);
  } else {
    board_motor_write(0, 1, 0, -left_clamped);
  }

  if (right_clamped >= 0) {
    board_motor_write(1, 0, 1, right_clamped);
  } else {
    board_motor_write(1, 1, 0, -right_clamped);
  }
}

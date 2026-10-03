/* 6_generated_code.c */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "6_generated_code.h"

/* Public state globals. */
float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;
float gyrox = 0.0f;
float gyroy = 0.0f;
float gyroz = 0.0f;
int32_t speed = 0;
int32_t state = 0;
int32_t ac_pwm = 0;
int32_t vc_pwm = 0;
int32_t dc_pwm = 0;
int32_t left_pwm = 0;
int32_t right_pwm = 0;

/* Private velocity controller state. */
static float velocity_e1 = 0.0f;
static float velocity_e2 = 0.0f;
static int32_t velocity_accumulator = 0;
static int32_t velocity_old_endpoint = 0;
static int32_t velocity_new_endpoint = 0;
static int32_t velocity_counter = 0;

/* Private direction controller state. */
static int16_t direction_previous_filtered = 0;
static int32_t direction_old_endpoint = 0;
static int32_t direction_new_endpoint = 0;
static int32_t direction_counter = 0;

static int32_t clamp_int32(int32_t value, int32_t low, int32_t high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

static int64_t clamp_int64(int64_t value, int64_t low, int64_t high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

static float clamp_float(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

void control_init(void) {
  pitch = 0.0f;
  roll = 0.0f;
  yaw = 0.0f;
  gyrox = 0.0f;
  gyroy = 0.0f;
  gyroz = 0.0f;
  speed = 0;
  state = 0;
  ac_pwm = 0;
  vc_pwm = 0;
  dc_pwm = 0;
  left_pwm = 0;
  right_pwm = 0;

  velocity_e1 = 0.0f;
  velocity_e2 = 0.0f;
  velocity_accumulator = 0;
  velocity_old_endpoint = 0;
  velocity_new_endpoint = 0;
  velocity_counter = 0;

  direction_previous_filtered = 0;
  direction_old_endpoint = 0;
  direction_new_endpoint = 0;
  direction_counter = 0;
}

void get_mpu(void) {
  board_read_imu(&pitch, &roll, &yaw, &gyrox, &gyroy, &gyroz);
}

int32_t get_speed(void) {
  int32_t left_count;
  int32_t right_count;
  int64_t left_abs;
  int64_t right_abs;
  int64_t measured;

  left_count = board_read_encoder(0);
  right_count = board_read_encoder(1);

  board_clear_encoder(0);
  board_clear_encoder(1);

  left_abs = left_count < 0 ? -(int64_t)left_count : (int64_t)left_count;
  right_abs = right_count < 0 ? -(int64_t)right_count : (int64_t)right_count;
  measured = (left_abs + right_abs) / 2;

  speed = (int32_t)clamp_int64(measured, 0, INT32_MAX);

  return speed;
}

int32_t angle_proc(void) {
  ac_pwm = -(int32_t)(550.0f * (0.0f - pitch) - 2.0f * gyroy);
  return ac_pwm;
}

int32_t velocity_proc(int32_t measured_speed) {
  if (velocity_counter >= 5) {
    float error;
    float p_term;
    float i_term;
    float d_term;
    int64_t proposed;

    velocity_counter = 0;
    velocity_old_endpoint = velocity_new_endpoint;

    error = 0.0f - (float)measured_speed;
    p_term = 8.0f * (error - velocity_e1);
    i_term = 0.1f * error;
    d_term = 0.5f * (error - 2.0f * velocity_e1 + velocity_e2);

    if (i_term > 10.0f || i_term < -10.0f) {
      i_term = 0.0f;
    }

    proposed = (int64_t)((float)velocity_accumulator + p_term + i_term + d_term);
    velocity_accumulator = (int32_t)clamp_int64(proposed, -4500, 4500);

    velocity_new_endpoint = velocity_accumulator;
    velocity_e2 = velocity_e1;
    velocity_e1 = error;
  }

  velocity_counter++;
  vc_pwm = velocity_old_endpoint +
           ((velocity_new_endpoint - velocity_old_endpoint) * velocity_counter) / 5;

  return vc_pwm;
}

int32_t direction_proc(int32_t measured_speed) {
  float filtered_value;
  int16_t filtered;

  filtered_value = 0.9f * gyroz + 0.1f * (float)direction_previous_filtered;
  filtered = (int16_t)filtered_value;
  gyroz = (float)filtered;
  direction_previous_filtered = filtered;

  if (direction_counter >= 5) {
    float p_gain;
    int64_t p_term;
    int64_t d_term;
    int64_t proposed;

    direction_counter = 0;
    direction_old_endpoint = direction_new_endpoint;

    state = board_read_direction_error();

    p_gain = clamp_float(0.6f * (float)measured_speed, 50.0f, 500.0f);
    p_term = (int64_t)(p_gain * (float)state / 25.0f);
    d_term = (int64_t)(1.5f * gyroz / 100.0f);
    proposed = p_term + d_term;

    direction_new_endpoint = (int32_t)clamp_int64(proposed, -3000, 3000);
  }

  direction_counter++;
  dc_pwm = direction_old_endpoint +
           ((direction_new_endpoint - direction_old_endpoint) * direction_counter) / 5;

  return dc_pwm;
}

void get_pwm(void) {
  int32_t measured_speed;

  get_mpu();

  measured_speed = get_speed();

  angle_proc();
  velocity_proc(measured_speed);
  direction_proc(measured_speed);

  left_pwm = (int32_t)((int64_t)ac_pwm - (int64_t)vc_pwm + (int64_t)dc_pwm);
  right_pwm = (int32_t)((int64_t)ac_pwm - (int64_t)vc_pwm - (int64_t)dc_pwm);
}

void motor_proc(int32_t left, int32_t right) {
  int32_t clamped_left;
  int32_t clamped_right;

  clamped_left = clamp_int32(left, -7200, 7200);
  clamped_right = clamp_int32(right, -7200, 7200);

  if (clamped_left >= 0) {
    board_motor_write(0, false, true, clamped_left);
  } else {
    board_motor_write(0, true, false, -clamped_left);
  }

  if (clamped_right >= 0) {
    board_motor_write(1, false, true, clamped_right);
  } else {
    board_motor_write(1, true, false, -clamped_right);
  }
}

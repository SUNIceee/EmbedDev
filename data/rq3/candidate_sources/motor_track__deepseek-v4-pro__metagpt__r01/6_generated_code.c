/* 6_generated_code.c */
#include "6_generated_code.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define MOTOR_PWM_FREQUENCY_HZ 17000
#define MOTOR_PWM_INIT_DUTY 0
#define MOTOR_NUM_PWM_CHANNELS 4

#define MOTOR_LEFT_FORWARD_CHANNEL 0
#define MOTOR_LEFT_REVERSE_CHANNEL 1
#define MOTOR_RIGHT_FORWARD_CHANNEL 2
#define MOTOR_RIGHT_REVERSE_CHANNEL 3

#define MOTOR_DUTY_LIMIT 50000
#define MOTOR_DUTY_MIN (-MOTOR_DUTY_LIMIT)
#define MOTOR_DUTY_MAX MOTOR_DUTY_LIMIT

#define MOTOR_STOP_TARGET_SPEED 0
#define MOTOR_REVERSE_TARGET_SPEED (-1500)
#define MOTOR_TAG_SEARCH_SPEED 1500
#define MOTOR_TAG_STOP_SPEED 0
#define MOTOR_TAG_SHOOTING_SPEED 0
#define MOTOR_APRILTAG_READY_SPEED 0
#define MOTOR_GARAGE_OUT_SPEED 2500
#define MOTOR_GARAGE_IN_SPEED (-2500)
#define MOTOR_GARAGE_STOP_SPEED 0
#define MOTOR_RAMP_ADC_SPEED 1800
#define MOTOR_Y_ROAD_SPEED 2200
#define MOTOR_CIRCLE_RAMP_SPEED 1600
#define MOTOR_LOOKAHEAD_SPEED 2000
#define MOTOR_DEGRADED_LOOKAHEAD_SPEED 1200

#define MOTOR_DIFFERENTIAL_GAIN 10.0f
#define MOTOR_SAFE_SPEED_MIN (-3000)
#define MOTOR_SAFE_SPEED_MAX 3000
#define MOTOR_ANGLE_PROTECTION_LIMIT_DEG 30.0f

#define MOTOR_NORMAL_PID_KP 1.2f
#define MOTOR_NORMAL_PID_KI 0.05f
#define MOTOR_NORMAL_PID_KD 0.01f
#define MOTOR_POS_PID_KP 0.8f
#define MOTOR_POS_PID_KI 0.0f
#define MOTOR_POS_PID_KD 0.02f

WheelMotor motor_l;
WheelMotor motor_r;
PID motor_pid_l;
PID motor_pid_r;
PID target_speed_pid;
PID posloop_pid;
int32_t target_speed = 0;
uint32_t clk = 0U;

static int32_t ClampInt32(int32_t value, int32_t min_value, int32_t max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static void InitializePid(PID* pid, float kp, float ki, float kd, float out_min,
                          float out_max) {
  if (pid == NULL) {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->out_min = out_min;
  pid->out_max = out_max;
}

void motor_init(void) {
  memset(&motor_l, 0, sizeof(motor_l));
  memset(&motor_r, 0, sizeof(motor_r));
  memset(&motor_pid_l, 0, sizeof(motor_pid_l));
  memset(&motor_pid_r, 0, sizeof(motor_pid_r));
  memset(&target_speed_pid, 0, sizeof(target_speed_pid));
  memset(&posloop_pid, 0, sizeof(posloop_pid));

  target_speed = 0;
  clk = 0U;

  motor_l.mode = MODE_NORMAL;
  motor_r.mode = MODE_NORMAL;

  InitializePid(&motor_pid_l, MOTOR_NORMAL_PID_KP, MOTOR_NORMAL_PID_KI,
                MOTOR_NORMAL_PID_KD, (float)MOTOR_DUTY_MIN,
                (float)MOTOR_DUTY_MAX);
  InitializePid(&motor_pid_r, MOTOR_NORMAL_PID_KP, MOTOR_NORMAL_PID_KI,
                MOTOR_NORMAL_PID_KD, (float)MOTOR_DUTY_MIN,
                (float)MOTOR_DUTY_MAX);
  InitializePid(&target_speed_pid, MOTOR_NORMAL_PID_KP, MOTOR_NORMAL_PID_KI,
                MOTOR_NORMAL_PID_KD, (float)MOTOR_DUTY_MIN,
                (float)MOTOR_DUTY_MAX);
  InitializePid(&posloop_pid, MOTOR_POS_PID_KP, MOTOR_POS_PID_KI,
                MOTOR_POS_PID_KD, (float)MOTOR_DUTY_MIN,
                (float)MOTOR_DUTY_MAX);

  for (int channel = 0; channel < MOTOR_NUM_PWM_CHANNELS; ++channel) {
    pwm_init(channel, MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_INIT_DUTY);
  }
}

void speed_control(void) {
  motor_l.encoder = motor_env.left_encoder;
  motor_r.encoder = motor_env.right_encoder;
  motor_l.actual_speed = motor_env.left_speed;
  motor_r.actual_speed = motor_env.right_speed;

  motor_l.mode = MODE_NORMAL;
  motor_r.mode = MODE_NORMAL;

  if (motor_env.animal_stop != 0U) {
    target_speed = MOTOR_STOP_TARGET_SPEED;
    motor_l.mode = MODE_NORMAL;
    motor_r.mode = MODE_NORMAL;
    motor_l.target_speed = (float)target_speed;
    motor_r.target_speed = (float)target_speed;
  } else if (motor_env.fruit != 0U || motor_env.laser != 0U) {
    target_speed = MOTOR_REVERSE_TARGET_SPEED;
    motor_l.mode = MODE_NORMAL;
    motor_r.mode = MODE_NORMAL;
    motor_l.target_speed = (float)target_speed;
    motor_r.target_speed = (float)target_speed;
  } else if (motor_env.tag_search != 0U) {
    target_speed = MOTOR_TAG_SEARCH_SPEED;
    motor_l.mode = MODE_SOFT;
    motor_r.mode = MODE_SOFT;
    motor_l.target_speed = (float)target_speed;
    motor_r.target_speed = (float)target_speed;
  } else if (motor_env.tag_stop != 0U) {
    target_speed = MOTOR_TAG_STOP_SPEED;
    motor_l.mode = MODE_BANG_BANG;
    motor_r.mode = MODE_BANG_BANG;
    motor_l.target_speed = (float)target_speed;
    motor_r.target_speed = (float)target_speed;
  } else if (motor_env.tag_shooting != 0U) {
    target_speed = MOTOR_TAG_SHOOTING_SPEED;
    motor_l.mode = MODE_POSITION_LOOP;
    motor_r.mode = MODE_POSITION_LOOP;
    motor_l.target_speed = (float)motor_l.encoder;
    motor_r.target_speed = (float)motor_r.encoder;
  } else if (motor_env.apriltag_ready != 0U) {
    target_speed = MOTOR_APRILTAG_READY_SPEED;
    motor_l.mode = MODE_POSITION_LOOP;
    motor_r.mode = MODE_POSITION_LOOP;
    motor_l.target_speed = (float)motor_l.encoder;
    motor_r.target_speed = (float)motor_r.encoder;
  } else if (motor_env.garage_out != 0U) {
    target_speed = MOTOR_GARAGE_OUT_SPEED;
    motor_l.mode = MODE_NORMAL;
    motor_r.mode = MODE_NORMAL;
    motor_l.target_speed = (float)target_speed;
    motor_r.target_speed = (float)target_speed;
  } else if (motor_env.garage_in != 0U) {
    target_speed = MOTOR_GARAGE_IN_SPEED;
    motor_l.mode = MODE_NORMAL;
    motor_r.mode = MODE_NORMAL;
    motor_l.target_speed = (float)target_speed;
    motor_r.target_speed = (float)target_speed;
  } else if (motor_env.garage_stop != 0U) {
    target_speed = MOTOR_GARAGE_STOP_SPEED;
    motor_l.mode = MODE_BANG_BANG;
    motor_r.mode = MODE_BANG_BANG;
    motor_l.target_speed = (float)target_speed;
    motor_r.target_speed = (float)target_speed;
  } else if (motor_env.ramp_mode != 0U || motor_env.adc_mode != 0U) {
    target_speed = MOTOR_RAMP_ADC_SPEED;
    motor_l.mode = MODE_NORMAL;
    motor_r.mode = MODE_NORMAL;
    motor_l.target_speed = (float)target_speed;
    motor_r.target_speed = (float)target_speed;
  } else if (motor_env.y_road != 0U) {
    target_speed = MOTOR_Y_ROAD_SPEED;
    motor_l.mode = MODE_NORMAL;
    motor_r.mode = MODE_NORMAL;
    motor_l.target_speed = (float)target_speed;
    motor_r.target_speed = (float)target_speed;
  } else if (motor_env.circle_ramp != 0U) {
    target_speed = MOTOR_CIRCLE_RAMP_SPEED;
    motor_l.mode = MODE_SOFT;
    motor_r.mode = MODE_SOFT;
    motor_l.target_speed = (float)target_speed;
    motor_r.target_speed = (float)target_speed;
  } else if (motor_env.lookahead_curve != 0U) {
    target_speed = MOTOR_LOOKAHEAD_SPEED;
    motor_l.mode = MODE_SOFT;
    motor_r.mode = MODE_SOFT;
    motor_l.target_speed = (float)target_speed;
    motor_r.target_speed = (float)target_speed;
  } else if (motor_env.degraded_lookahead != 0U) {
    target_speed = MOTOR_DEGRADED_LOOKAHEAD_SPEED;
    motor_l.mode = MODE_BANG_BANG;
    motor_r.mode = MODE_BANG_BANG;
    motor_l.target_speed = (float)target_speed;
    motor_r.target_speed = (float)target_speed;
  } else {
    // Fallback retention: keep the previous target_speed and wheel targets.
  }

  // Final safety override.
  if (motor_env.inductance_low != 0U) {
    target_speed = MOTOR_STOP_TARGET_SPEED;
    motor_l.mode = MODE_NORMAL;
    motor_r.mode = MODE_NORMAL;
    motor_l.target_speed = (float)target_speed;
    motor_r.target_speed = (float)target_speed;
  }

  target_speed = ClampInt32(target_speed, MOTOR_SAFE_SPEED_MIN,
                            MOTOR_SAFE_SPEED_MAX);

  // Differential planning based on motor_env.angle. Position loops keep the
  // branch-assigned position targets.
  float diff = motor_env.angle;
  int32_t left_offset = 0;
  int32_t right_offset = 0;

  if (target_speed != 0) {
    left_offset = (int32_t)(diff * MOTOR_DIFFERENTIAL_GAIN);
    right_offset = -left_offset;
  }

  if (motor_l.mode != MODE_POSITION_LOOP) {
    motor_l.target_speed =
        (float)ClampInt32(target_speed + left_offset, MOTOR_SAFE_SPEED_MIN,
                          MOTOR_SAFE_SPEED_MAX);
  }

  if (motor_r.mode != MODE_POSITION_LOOP) {
    motor_r.target_speed =
        (float)ClampInt32(target_speed + right_offset, MOTOR_SAFE_SPEED_MIN,
                          MOTOR_SAFE_SPEED_MAX);
  }
}

void motor_control(void) {
  WheelMotor* wheels[2] = {&motor_l, &motor_r};
  PID* wheel_pids[2] = {&motor_pid_l, &motor_pid_r};
  int forward_channels[2] = {MOTOR_LEFT_FORWARD_CHANNEL,
                             MOTOR_RIGHT_FORWARD_CHANNEL};
  int reverse_channels[2] = {MOTOR_LEFT_REVERSE_CHANNEL,
                             MOTOR_RIGHT_REVERSE_CHANNEL};

  for (int i = 0; i < 2; ++i) {
    WheelMotor* wheel = wheels[i];
    PID* wheel_pid = wheel_pids[i];
    float output = 0.0f;

    switch (wheel->mode) {
      case MODE_NORMAL:
        output =
            pid_solve(wheel_pid, wheel->target_speed, (float)wheel->actual_speed);
        break;
      case MODE_BANG_BANG:
        wheel_pid->integral = 0.0f;
        wheel_pid->prev_error = 0.0f;
        output = bangbang_pid_solve(wheel_pid, wheel->target_speed,
                                    (float)wheel->actual_speed);
        break;
      case MODE_SOFT:
        wheel_pid->integral = 0.0f;
        wheel_pid->prev_error = 0.0f;
        output = changable_pid_solve(wheel_pid, wheel->target_speed,
                                     (float)wheel->actual_speed);
        break;
      case MODE_POSITION_LOOP:
        wheel_pid->integral = 0.0f;
        wheel_pid->prev_error = 0.0f;
        output = changable_pid_solve(&posloop_pid, wheel->target_speed,
                                     (float)wheel->encoder);
        break;
      default:
        wheel_pid->integral = 0.0f;
        wheel_pid->prev_error = 0.0f;
        output = 0.0f;
        break;
    }

    int32_t duty = (int32_t)output;
    duty = ClampInt32(duty, MOTOR_DUTY_MIN, MOTOR_DUTY_MAX);

    // Angle protection restriction.
    if (motor_env.angle > MOTOR_ANGLE_PROTECTION_LIMIT_DEG ||
        motor_env.angle < -MOTOR_ANGLE_PROTECTION_LIMIT_DEG) {
      duty = 0;
    }

    wheel->duty = duty;

    if (duty >= 0) {
      pwm_duty(forward_channels[i], duty);
      pwm_duty(reverse_channels[i], 0);
    } else {
      pwm_duty(forward_channels[i], 0);
      pwm_duty(reverse_channels[i], -duty);
    }
  }
}

int64_t get_total_encoder(void) {
  return (int64_t)motor_l.encoder + (int64_t)motor_r.encoder;
}

float radius_3pts(Point a, Point b, Point c) {
  float ab_x = b.x - a.x;
  float ab_y = b.y - a.y;
  float ac_x = c.x - a.x;
  float ac_y = c.y - a.y;
  float det = (ab_x * ac_y) - (ab_y * ac_x);

  if (fabsf(det) < 1e-6f) {
    return 0.0f;
  }

  float ab = sqrtf((ab_x * ab_x) + (ab_y * ab_y));
  float bc_x = c.x - b.x;
  float bc_y = c.y - b.y;
  float bc = sqrtf((bc_x * bc_x) + (bc_y * bc_y));
  float ca = sqrtf((a.x - c.x) * (a.x - c.x) +
                   (a.y - c.y) * (a.y - c.y));

  float radius = (ab * bc * ca) / (2.0f * fabsf(det));
  return radius;
}

int square_signal(uint32_t period, uint32_t high_time) {
  if (period == 0U) {
    return 0;
  }

  uint32_t phase = clk % period;
  ++clk;

  if (high_time > period) {
    high_time = period;
  }

  return (phase < high_time) ? 1 : 0;
}

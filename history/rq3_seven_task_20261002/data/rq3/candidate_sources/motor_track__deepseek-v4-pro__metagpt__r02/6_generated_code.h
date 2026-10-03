/* 6_generated_code.h
 *
 * Fixed public API for the generated motor controller.
 *
 * This header intentionally contains only standard C11 integer types, public
 * data structures, the externally owned environment instance, fixed helper
 * declarations, and the six public entry points.  Application-owned control
 * state is defined in 6_generated_code.c and is intentionally not part of
 * this public header.
 */
#ifndef MOTOR_6_GENERATED_CODE_H
#define MOTOR_6_GENERATED_CODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Point {
  float x;
  float y;
} Point;

typedef enum motor_mode_t {
  MODE_NORMAL = 0,
  MODE_SOFT = 1,
  MODE_BANGBANG = 2,
  MODE_POSLOOP = 3
} motor_mode_t;

typedef struct PIDController {
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
  float max_output;
  float max_integral;
} PIDController;

typedef struct wheel_t {
  int32_t target_speed;
  motor_mode_t mode;
  float previous_duty;
  float brake_pid;
  float pid;
} wheel_t;

typedef struct motor_environment_t {
  uint32_t now_ms;
  uint32_t animal_time_ms;
  int32_t fa_type;
  float fruit_delta;
  float laser_angle;
  int32_t tag_state;
  int32_t garage;
  int32_t garage_direction;
  int32_t enable_adc;
  int32_t yroad_state;
  int32_t rptsn_num;
  float rptsn[70][2];
  float elec_data[2];
  int64_t encoder_left;
  int64_t encoder_right;
  float encoder_speed_left;
  float encoder_speed_right;
} motor_environment_t;

/* Environment instance owned by the host. */
extern motor_environment_t motor_env;

/* Fixed external helper functions provided by the host platform. */
void pwm_init(uint32_t channel, uint32_t freq_hz, uint32_t duty);
void pwm_duty(uint32_t channel, uint32_t duty);
float pid_solve(PIDController* pid, float error);
float bangbang_pid_solve(float* state, float error);
float changable_pid_solve(float* state, float error);

/* Public motor controller entry points. */
void motor_init(void);
void speed_control(void);
void motor_control(void);
int64_t get_total_encoder(void);
double radius_3pts(Point a, Point b, Point c);
void square_signal(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_6_GENERATED_CODE_H */

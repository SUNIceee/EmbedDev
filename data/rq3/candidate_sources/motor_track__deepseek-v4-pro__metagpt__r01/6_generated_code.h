#ifndef MOTOR_CONTROL_6_GENERATED_CODE_H_
#define MOTOR_CONTROL_6_GENERATED_CODE_H_

/*
 * Public API for the generated motor-control library.
 *
 * This header contains only declarations. The corresponding translation unit
 * 6_generated_code.c defines all owned mutable state and implements the public
 * functions. The externally supplied motor_env object and fixed helper
 * functions are declared here as extern and must be provided by the host
 * integration at link time.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Motor operation modes.
 *
 * The numeric values are guaranteed so host code can store and compare them
 * across the generated library boundary without relying on compiler-specific
 * enum sizes.
 */
typedef enum MotorMode {
  MODE_NORMAL = 0,
  MODE_BANG_BANG = 1,
  MODE_SOFT = 2,
  MODE_POSITION_LOOP = 3
} MotorMode;

/*
 * Generic PID controller state.
 *
 * All fields are explicitly single-precision floating point. out_min/out_max
 * are enforced by the externally supplied pid helpers before they return.
 */
typedef struct PID {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float out_min;
  float out_max;
} PID;

/*
 * State of one motor channel.
 */
typedef struct WheelMotor {
  float target_speed;
  MotorMode mode;
  int32_t duty;
  int64_t encoder;
  int16_t actual_speed;
} WheelMotor;

/*
 * Environment information supplied by the host application.
 *
 * The flags are uint8_t to remain ABI-stable. A value of zero is reserved as
 * the inactive/default state.
 */
typedef struct MotorEnv {
  float angle;
  int64_t left_encoder;
  int64_t right_encoder;
  int16_t left_speed;
  int16_t right_speed;
  uint8_t animal_stop;
  uint8_t fruit;
  uint8_t laser;
  uint8_t tag_search;
  uint8_t tag_stop;
  uint8_t tag_shooting;
  uint8_t apriltag_ready;
  uint8_t garage_out;
  uint8_t garage_in;
  uint8_t garage_stop;
  uint8_t ramp_mode;
  uint8_t adc_mode;
  uint8_t y_road;
  uint8_t circle_ramp;
  uint8_t lookahead_curve;
  uint8_t degraded_lookahead;
  uint8_t inductance_low;
} MotorEnv;

/*
 * Simple 2D point used by radius_3pts().
 */
typedef struct Point {
  float x;
  float y;
} Point;

/*
 * Externally supplied fixed environment object.
 *
 * The generated library reads this object during speed_control()/motor_control()
 * but never defines or initializes it.
 */
extern MotorEnv motor_env;

/*
 * Owned global state.
 *
 * These objects are defined in 6_generated_code.c and are initialized
 * deterministically by motor_init().
 */
extern WheelMotor motor_l;
extern WheelMotor motor_r;
extern PID motor_pid_l;
extern PID motor_pid_r;
extern PID target_speed_pid;
extern PID posloop_pid;
extern int32_t target_speed;
extern uint32_t clk;

/*
 * Fixed helper functions supplied by the host integration.
 *
 * The generated library calls these helpers instead of reimplementing PWM or
 * PID behavior. The host must provide all five functions at link time.
 */
extern void pwm_init(int channel, int frequency, int duty);
extern void pwm_duty(int channel, int duty);
extern float pid_solve(PID* pid, float target, float actual);
extern float bangbang_pid_solve(PID* pid, float target, float actual);
extern float changable_pid_solve(PID* pid, float target, float actual);

/*
 * Public application functions.
 *
 * motor_init() performs deterministic PWM and state initialization.
 * speed_control() performs planning only and never touches PWM.
 * motor_control() performs actuation and writes only PWM.
 * get_total_encoder() returns the sum of both wheel encoders.
 * radius_3pts() returns the circumradius of three points.
 * square_signal() evaluates a deterministic periodic square wave.
 */
void motor_init(void);
void speed_control(void);
void motor_control(void);
int64_t get_total_encoder(void);
float radius_3pts(Point a, Point b, Point c);
int square_signal(uint32_t period, uint32_t high_time);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_6_GENERATED_CODE_H_ */

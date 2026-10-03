#ifndef MOTOR_TRACK_6_GENERATED_CODE_H_
#define MOTOR_TRACK_6_GENERATED_CODE_H_

#include <stdint.h>

/*
 * Public device state and API for the generated motor-track controller.
 *
 * The implementation file 6_generated_code.c owns the definitions of all
 * globals declared here and provides the function bodies.
 */

typedef struct MotorChannel {
  float target_speed;
  int mode;
  int encoder;
  float forward_pwm;
  float reverse_pwm;
} MotorChannel;

typedef struct PIDState {
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
  float output;
} PIDState;

/* Owned device state. */
extern MotorChannel motor_l;
extern MotorChannel motor_r;
extern PIDState motor_pid_l;
extern PIDState motor_pid_r;
extern PIDState target_speed_pid;
extern PIDState posloop_pid;
extern float target_speed;
extern uint32_t clk;

/* Public device API. */
void motor_init(void);
void speed_control(void);
void motor_control(void);
float get_total_encoder(void);

/*
 * Pure utility functions. These functions must not modify any owned device
 * state and must not write to PWM outputs.
 */
float radius_3pts(float x1, float y1, float x2, float y2, float x3, float y3);
float square_signal(float time, float period, float duty);

/*
 * Fixed motor-track contract helpers provided by the runtime environment.
 * These declarations keep the generated implementation free of implicit
 * function declarations.
 */
void pwm_init(uint32_t frequency_hz, uint8_t channel_count);
void pwm_duty(uint8_t channel, float duty);
float pid_solve(PIDState *pid, float error);
float bangbang_pid_solve(PIDState *pid, float error);
float changable_pid_solve(PIDState *pid, float error);

#endif  /* MOTOR_TRACK_6_GENERATED_CODE_H_ */

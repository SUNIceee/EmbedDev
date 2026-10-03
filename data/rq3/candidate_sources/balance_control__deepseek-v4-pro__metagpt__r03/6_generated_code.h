/* 6_generated_code.h */
#ifndef SIX_GENERATED_CODE_H_
#define SIX_GENERATED_CODE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public fixed-width constants used by the balance-control schedule and
 * PWM/motor clamping boundaries.  Values are taken from the RE_api.txt
 * contract; adjust them here if the supplied API document differs. */
#define BALANCE_CONTROL_SCHEDULE_TICKS  INT32_C(5)
#define VELOCITY_PID_ACCUMULATOR_MIN     INT32_C(-4500)
#define VELOCITY_PID_ACCUMULATOR_MAX     INT32_C(4500)
#define DIRECTION_PWM_MIN                INT32_C(-3000)
#define DIRECTION_PWM_MAX                INT32_C(3000)
#define MOTOR_DUTY_MIN                   INT32_C(-7200)
#define MOTOR_DUTY_MAX                   INT32_C(7200)

/* Public globals - sensor snapshot and output snapshot.
 * These variables are defined in 6_generated_code.c and updated by
 * get_mpu(), get_speed(), and get_pwm(). */
extern float pitch;
extern float roll;
extern float yaw;
extern int16_t gyrox;
extern int16_t gyroy;
extern int16_t gyroz;
extern int16_t state;
extern int32_t speed;
extern int32_t ac_pwm;
extern int32_t vc_pwm;
extern int32_t dc_pwm;
extern int32_t left_pwm;
extern int32_t right_pwm;

/* Public control functions. */
void control_init(void);
void get_mpu(void);
int32_t get_speed(void);
int32_t angle_proc(void);
int32_t velocity_proc(int32_t measured_speed);
int32_t direction_proc(int32_t measured_speed);
void get_pwm(void);
void motor_proc(int32_t left, int32_t right);

/* Platform callbacks that must be supplied by the board/RE layer.
 * The signatures below follow the common RE_api.txt callback contract. */
extern void board_read_imu(float* pitch, float* roll, float* yaw,
                           int16_t* gyrox, int16_t* gyroy, int16_t* gyroz);
extern int16_t board_encoder_read(int channel);
extern void board_encoder_clear(int channel);
extern int16_t board_direction_error(void);
extern void board_motor_write(int channel, int in_a, int in_b, int duty);

#ifdef __cplusplus
}
#endif

#endif /* SIX_GENERATED_CODE_H_ */

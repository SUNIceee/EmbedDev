/* 6_generated_code.h
 * Public frozen API for the bounded C11 balance-control library.
 *
 * This header exposes the public sensor/output globals, fixed gain/limit
 * constants, external board callback prototypes, and public control functions.
 * Only <stdint.h> and <stdbool.h> are required.
 */
#ifndef FSE_FROZEN_API_H
#define FSE_FROZEN_API_H

#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public sensor and output globals. */
extern float pitch;
extern float roll;
extern float yaw;
extern int16_t gyrox;
extern int16_t gyroy;
extern int16_t gyroz;
extern int32_t speed;
extern int16_t direction_error;
extern int32_t ac_pwm;
extern int32_t vc_pwm;
extern int32_t dc_pwm;
extern int32_t left_pwm;
extern int32_t right_pwm;

/* Board callback prototypes.
 * The application or board-support layer must provide these implementations.
 */
void board_read_imu(float *pitch, float *roll, float *yaw,
                    int16_t *gyrox, int16_t *gyroy, int16_t *gyroz);
int16_t board_encoder_read(int channel);
void board_encoder_clear(int channel);
int16_t board_direction_error(void);
void board_motor_write(int channel, bool in_a, bool in_b, uint16_t duty);

/* Public control API. */
void control_init(void);
void get_pwm(void);
void get_mpu(void);
int32_t get_speed(void);
int32_t angle_proc(void);
int32_t velocity_proc(int32_t measured_speed);
int32_t direction_proc(int32_t measured_speed);
void motor_proc(int32_t left, int32_t right);

/* Fixed control constants. */

/* Velocity control loop. */
#define VELOCITY_KP                   8.0f
#define VELOCITY_KI                   0.1f
#define VELOCITY_KD                   0.5f
#define VELOCITY_INTEGRAL_LIMIT      10.0f
#define VELOCITY_ACCUMULATOR_LIMIT 4500
#define VELOCITY_CONTROL_PERIOD        5

/* Direction control loop. */
#define DIRECTION_FILTER_NEW_COEF     0.9f
#define DIRECTION_FILTER_OLD_COEF     0.1f
#define DIRECTION_PGAIN_MIN          50.0f
#define DIRECTION_PGAIN_MAX         500.0f
#define DIRECTION_ERROR_SCALE        25.0f
#define DIRECTION_GYRO_SCALE        100.0f
#define DIRECTION_KD                  1.5f
#define DIRECTION_ENDPOINT_LIMIT   3000
#define DIRECTION_CONTROL_PERIOD       5

/* Motor output duty limit. */
#define MOTOR_PWM_LIMIT             7200

/* Angle control loop gains. */
#define ANGLE_KP                    300.0f
#define ANGLE_KD                      5.0f

#ifdef __cplusplus
}
#endif

#endif /* BALANCE_CONTROL_H */
#endif /* FSE_FROZEN_API_H */

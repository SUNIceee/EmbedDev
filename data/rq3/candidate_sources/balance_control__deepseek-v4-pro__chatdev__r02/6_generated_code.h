/* Fixed public API for the bounded three-loop balance control library. */

#ifndef FSE_FROZEN_API_H
#define FSE_FROZEN_API_H
#define BALANCE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#define BC_AC_SET 0.0f
#define BC_AC_KP 550.0f
#define BC_AC_KD 2.0f
#define BC_VC_SET 0
#define BC_VC_KP 8.0f
#define BC_VC_KI 0.1f
#define BC_VC_KD 0.5f
#define BC_VC_PERIOD 5u
#define BC_VC_LIMIT 4500
#define BC_DC_PERIOD 5u
#define BC_DC_COEF 0.6f
#define BC_DC_P_MIN 50.0f
#define BC_DC_P_MAX 500.0f
#define BC_DC_KD 1.5f
#define BC_DC_LIMIT 3000
#define BC_MOTOR_LIMIT 7200

extern float pitch, roll, yaw;
extern int16_t gyrox, gyroy, gyroz, state;
extern int32_t speed, ac_pwm, vc_pwm, dc_pwm, left_pwm, right_pwm;

void control_init(void);
void get_pwm(void);
void get_mpu(void);
int get_speed(void);
void motor_proc(int32_t left, int32_t right);
int32_t angle_proc(void);
int32_t velocity_proc(int32_t measured_speed);
int32_t direction_proc(int32_t measured_speed);

/* Platform provided, NOT implemented by this library. */
extern void board_read_imu(float *p, float *r, float *y, int16_t *gx, int16_t *gy, int16_t *gz);
extern int16_t board_encoder_read(uint8_t channel);
extern void board_encoder_clear(uint8_t channel);
extern int16_t board_direction_error(void);
extern void board_motor_write(uint8_t channel, bool in_a, bool in_b, uint16_t duty);

#endif

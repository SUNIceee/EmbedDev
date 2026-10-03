#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public sensor and output globals */
extern float pitch;
extern float roll;
extern float yaw;
extern int16_t gyrox;
extern int16_t gyroy;
extern int16_t gyroz;
extern int32_t speed;

extern int32_t ac_pwm;
extern int32_t vc_pwm;
extern int32_t dc_pwm;
extern int32_t left_pwm;
extern int32_t right_pwm;

extern int16_t state;

/* Controller API */
void control_init(void);
void get_mpu(void);
int32_t get_speed(void);
int32_t angle_proc(void);
int32_t velocity_proc(int32_t measured_speed);
int32_t direction_proc(int32_t measured_speed);
void get_pwm(void);
void motor_proc(int32_t left, int32_t right);

/* Platform callbacks supplied externally */
void board_read_imu(float *pitch, float *roll, float *yaw,
                    int16_t *gyrox, int16_t *gyroy, int16_t *gyroz);
int16_t board_encoder_read(int channel);
void board_encoder_clear(int channel);
int16_t board_direction_error(void);
void board_motor_write(int channel, int in_a, int in_b, int duty);

#ifdef __cplusplus
}
#endif

#endif

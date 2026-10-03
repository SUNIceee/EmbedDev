#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern float pitch;
extern float roll;
extern float yaw;
extern int16_t gyrox;
extern int16_t gyroy;
extern int16_t gyroz;
extern int16_t direction_error;
extern int32_t speed;
extern int32_t ac_pwm;
extern int32_t vc_pwm;
extern int32_t dc_pwm;
extern int32_t left_pwm;
extern int32_t right_pwm;

void control_init(void);
void get_mpu(void);
int32_t get_speed(void);
int32_t angle_proc(void);
int32_t velocity_proc(int32_t measured_speed);
int32_t direction_proc(int32_t measured_speed);
void get_pwm(void);
void motor_proc(int32_t left, int32_t right);

void board_read_imu(float *pitch, float *roll, float *yaw,
                    int16_t *gyrox, int16_t *gyroy, int16_t *gyroz);
int16_t board_encoder_read(uint8_t channel);
void board_encoder_clear(uint8_t channel);
int16_t board_direction_error(void);
void board_motor_write(uint8_t channel, bool in_a, bool in_b, int32_t duty);

#ifdef __cplusplus
}
#endif

#endif

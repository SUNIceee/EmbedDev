/* 6_generated_code.h
 * Public interface for the bounded deterministic balancing control library.
 *
 * This header declares the frozen public state, public API functions, and
 * board-level callbacks required by the generated implementation. It uses only
 * the C11 standard headers <stdint.h>, <stdbool.h>, and <stddef.h>.
 */
#ifndef SIX_GENERATED_CODE_H_
#define SIX_GENERATED_CODE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public IMU state. These values are updated only by get_mpu(). */
extern float pitch;
extern float roll;
extern float yaw;
extern float gyrox;
extern float gyroy;
extern float gyroz;

/* Public control state and output snapshots. */
extern int32_t speed;
extern int32_t state;
extern int32_t ac_pwm;
extern int32_t vc_pwm;
extern int32_t dc_pwm;
extern int32_t left_pwm;
extern int32_t right_pwm;

/* Public control API. */
void control_init(void);
void get_mpu(void);
int32_t get_speed(void);
int32_t angle_proc(void);
int32_t velocity_proc(int32_t measured_speed);
int32_t direction_proc(int32_t measured_speed);
void get_pwm(void);
void motor_proc(int32_t left, int32_t right);

/* Board-level callbacks provided by the target integration layer. */
void board_read_imu(float* pitch,
                    float* roll,
                    float* yaw,
                    float* gyrox,
                    float* gyroy,
                    float* gyroz);
int32_t board_read_encoder(uint8_t channel);
void board_clear_encoder(uint8_t channel);
int32_t board_read_direction_error(void);
void board_motor_write(uint8_t channel,
                       bool in_a,
                       bool in_b,
                       int32_t duty);

#ifdef __cplusplus
}
#endif

#endif /* SIX_GENERATED_CODE_H_ */

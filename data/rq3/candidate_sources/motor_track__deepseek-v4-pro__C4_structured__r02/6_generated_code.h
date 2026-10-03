#ifndef MOTOR_TRACK_CONTRACT_H
#define MOTOR_TRACK_CONTRACT_H
#include <stdint.h>
#define MOTOR_PWM_DUTY_MAX 50000.0f
#define MOTOR_TRACK_POINT_CAPACITY 128
#define MOTOR_PI 3.14159265358979323846f
typedef enum { MODE_NORMAL=0, MODE_BANGBANG=1, MODE_SOFT=2, MODE_POSLOOP=3 } motor_mode_t;
typedef enum { FA_NONE=0, ANIMAL=1 } fa_type_t;
typedef enum { TAG_NONE=0, TAG_SEARCH=1, TAG_STOP=2, TAG_SHOOTING=3 } tag_type_t;
typedef enum { APRILTAG_NONE=0, APRILTAG_FOUND=1, APRILTAG_MAYBE=2 } apriltag_type_t;
typedef enum { GARAGE_NONE=0, GARAGE_OUT_LEFT=1, GARAGE_OUT_RIGHT=2, GARAGE_IN_LEFT=3, GARAGE_IN_RIGHT=4, GARAGE_STOP=5 } garage_type_t;
typedef enum { YROAD_NONE=0, YROAD_FOUND=1, YROAD_NEAR=2 } yroad_type_t;
typedef enum { CIRCLE_NONE=0, CIRCLE_LEFT_BEGIN=1, CIRCLE_RIGHT_BEGIN=2 } circle_type_t;
typedef enum { MOTOR_LEFT_FORWARD=0, MOTOR_LEFT_REVERSE=1, MOTOR_RIGHT_FORWARD=2, MOTOR_RIGHT_REVERSE=3 } motor_pwm_channel_t;
typedef struct { float kp, ki, kd, out_i, last_error, max_output, max_integral; } motor_pid_t;
typedef struct {
    float target_speed, encoder_speed, duty;
    int64_t total_encoder, target_encoder;
    motor_mode_t motor_mode;
    motor_pid_t brake_pid, pid;
} motor_param_t;
typedef struct {
    uint32_t now_ms, animal_time_ms;
    fa_type_t fa_type;
    float fruit_delta, laser_angle;
    tag_type_t tag_type;
    apriltag_type_t apriltag_type;
    garage_type_t garage_type;
    int enable_adc;
    yroad_type_t yroad_type;
    circle_type_t circle_type;
    unsigned rptsn_num;
    float rptsn[MOTOR_TRACK_POINT_CAPACITY][2];
    float elec_data[2], angle;
} motor_environment_t;
extern motor_environment_t motor_env;
extern void pwm_init(motor_pwm_channel_t channel, uint32_t frequency_hz, uint32_t duty);
extern void pwm_duty(motor_pwm_channel_t channel, uint32_t duty);
extern float pid_solve(motor_pid_t *pid, float error);
extern float bangbang_pid_solve(motor_pid_t *pid, float error);
extern float changable_pid_solve(motor_pid_t *pid, float error);
extern motor_param_t motor_l, motor_r;
extern motor_pid_t motor_pid_l, motor_pid_r, target_speed_pid, posloop_pid;
extern float target_speed;
extern uint32_t clk;
void motor_init(void);
void speed_control(void);
void motor_control(void);
int64_t get_total_encoder(void);
float radius_3pts(float pt0[2], float pt1[2], float pt2[2]);
void square_signal(void);
#endif
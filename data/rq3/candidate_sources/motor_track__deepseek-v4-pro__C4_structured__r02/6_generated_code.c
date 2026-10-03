#include "6_generated_code.h"
#include <math.h>
#include <stdint.h>

#define MOTOR_NORMAL_SPEED 17.0f
#define MOTOR_NORMAL_MIN_SPEED (-8.0f)

static float cm_clamp(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static void cm_write_pwm_pair(motor_pwm_channel_t forward, motor_pwm_channel_t reverse, float duty)
{
    if (duty >= 0.0f) {
        pwm_duty(forward, (uint32_t)duty);
        pwm_duty(reverse, 0u);
    } else {
        pwm_duty(forward, 0u);
        pwm_duty(reverse, (uint32_t)(-duty));
    }
}

motor_param_t motor_l = {0};
motor_param_t motor_r = {0};
motor_pid_t motor_pid_l = {7021.0f, 10.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};
motor_pid_t motor_pid_r = {7021.0f, 10.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};
motor_pid_t target_speed_pid = {5.0f, 0.0f, 30.0f, 0.0f, 0.0f, 5.0f, 5.0f};
motor_pid_t posloop_pid = {200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};
float target_speed = 0.0f;
uint32_t clk = 0u;

void motor_init(void)
{
    pwm_init(MOTOR_LEFT_FORWARD, 17000u, 0u);
    pwm_init(MOTOR_LEFT_REVERSE, 17000u, 0u);
    pwm_init(MOTOR_RIGHT_FORWARD, 17000u, 0u);
    pwm_init(MOTOR_RIGHT_REVERSE, 17000u, 0u);
}

void speed_control(void)
{
    float diff = 0.0f;
    float solution = 0.0f;
    float error = 0.0f;
    unsigned id = 0u;

    motor_l.motor_mode = MODE_NORMAL;
    motor_r.motor_mode = MODE_NORMAL;
    diff = 15.8f * tanf(motor_env.angle * MOTOR_PI * 2.4f / 180.0f) / 40.0f / 2.0f;

    if (motor_env.fa_type == ANIMAL && (uint32_t)(motor_env.now_ms - motor_env.animal_time_ms) < 2500u) {
        target_speed = 0.0f;
        diff = 0.0f;
    } else if (motor_env.fruit_delta < 0.0f && (motor_env.laser_angle < 5.0f || motor_env.laser_angle > 175.0f)) {
        target_speed = -1.0f;
    } else if (motor_env.tag_type == TAG_SEARCH) {
        target_speed = 1.0f;
    } else if (motor_env.tag_type == TAG_STOP || motor_env.tag_type == TAG_SHOOTING) {
        target_speed = 0.0f;
    } else if (motor_env.apriltag_type == APRILTAG_FOUND) {
        target_speed = 0.5f;
        diff = 0.0f;
    } else if (motor_env.apriltag_type == APRILTAG_MAYBE) {
        target_speed = 1.0f;
    } else if (motor_env.garage_type == GARAGE_OUT_LEFT || motor_env.garage_type == GARAGE_OUT_RIGHT) {
        target_speed = 14.0f;
        motor_l.motor_mode = MODE_SOFT;
        motor_r.motor_mode = MODE_SOFT;
    } else if (motor_env.garage_type == GARAGE_IN_LEFT || motor_env.garage_type == GARAGE_IN_RIGHT) {
        target_speed = 10.0f;
    } else if (motor_env.enable_adc != 0) {
        target_speed = 9.0f;
        motor_l.motor_mode = MODE_BANGBANG;
        motor_r.motor_mode = MODE_BANGBANG;
    } else if (motor_env.yroad_type == YROAD_NEAR || motor_env.yroad_type == YROAD_FOUND) {
        target_speed = 3.0f;
    } else if (motor_env.circle_type == CIRCLE_LEFT_BEGIN || motor_env.circle_type == CIRCLE_RIGHT_BEGIN) {
        float previous_target_speed = target_speed;
        target_speed = previous_target_speed - 0.02f;
        if (target_speed < 11.0f) target_speed = 11.0f;
        if (target_speed > 17.0f) target_speed = 17.0f;
    } else if (motor_env.rptsn_num > 20u) {
        id = motor_env.rptsn_num - 1u;
        if (id > 70u) id = 70u;
        error = fabsf((motor_env.rptsn[id][0] - motor_env.rptsn[0][0]) / (motor_env.rptsn[id][1] - motor_env.rptsn[0][1]));
        solution = pid_solve(&target_speed_pid, error);
        target_speed = MOTOR_NORMAL_SPEED - solution;
        if (target_speed < 9.0f) target_speed = 9.0f;
        if (target_speed > MOTOR_NORMAL_SPEED) target_speed = MOTOR_NORMAL_SPEED;
    } else if (motor_env.rptsn_num > 5u) {
        target_speed = 9.0f;
    } else {
        /* rptsn_num <= 5 and no earlier decision matched: retain previous scalar target_speed. */
    }

    if (motor_env.garage_type == GARAGE_STOP ||
        ((motor_env.garage_type != GARAGE_OUT_LEFT && motor_env.garage_type != GARAGE_OUT_RIGHT) &&
         ((motor_env.elec_data[0] + motor_env.elec_data[1]) < 60.0f))) {
        target_speed = 0.0f;
        diff = 0.0f;
        motor_l.motor_mode = MODE_NORMAL;
        motor_r.motor_mode = MODE_NORMAL;
    }

    motor_l.target_speed = target_speed - diff * target_speed;
    motor_r.target_speed = target_speed + diff * target_speed;
}

void motor_control(void)
{
    float left_error = motor_l.target_speed - motor_l.encoder_speed;
    float right_error = motor_r.target_speed - motor_r.encoder_speed;
    float left_duty = 0.0f;
    float right_duty = 0.0f;

    switch (motor_l.motor_mode) {
    case MODE_NORMAL:
        left_duty = pid_solve(&motor_pid_l, left_error);
        break;
    case MODE_BANGBANG:
        motor_pid_l.out_i = 0.0f;
        left_duty = motor_l.duty + bangbang_pid_solve(&motor_l.brake_pid, left_error);
        break;
    case MODE_SOFT:
        motor_pid_l.out_i = 0.0f;
        left_duty = motor_l.duty + changable_pid_solve(&motor_l.pid, left_error);
        break;
    case MODE_POSLOOP:
        motor_pid_l.out_i = 0.0f;
        left_duty = pid_solve(&posloop_pid, (float)(motor_l.target_encoder - motor_l.total_encoder));
        break;
    default:
        left_duty = pid_solve(&motor_pid_l, left_error);
        break;
    }

    switch (motor_r.motor_mode) {
    case MODE_NORMAL:
        right_duty = pid_solve(&motor_pid_r, right_error);
        break;
    case MODE_BANGBANG:
        motor_pid_r.out_i = 0.0f;
        right_duty = motor_r.duty + bangbang_pid_solve(&motor_r.brake_pid, right_error);
        break;
    case MODE_SOFT:
        motor_pid_r.out_i = 0.0f;
        right_duty = motor_r.duty + changable_pid_solve(&motor_r.pid, right_error);
        break;
    case MODE_POSLOOP:
        motor_pid_r.out_i = 0.0f;
        right_duty = pid_solve(&posloop_pid, (float)(motor_r.target_encoder - motor_r.total_encoder));
        break;
    default:
        right_duty = pid_solve(&motor_pid_r, right_error);
        break;
    }

    left_duty = cm_clamp(left_duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
    right_duty = cm_clamp(right_duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);

    if (fabsf(motor_env.angle) > 10.0f) {
        float speed_error_summary = target_speed - 0.5f * (motor_l.encoder_speed + motor_r.encoder_speed);
        if (speed_error_summary < 0.0f) {
            left_duty = cm_clamp(left_duty, -MOTOR_PWM_DUTY_MAX, 40000.0f);
            right_duty = cm_clamp(right_duty, -MOTOR_PWM_DUTY_MAX, 40000.0f);
        } else {
            left_duty = cm_clamp(left_duty, -40000.0f, 40000.0f);
            right_duty = cm_clamp(right_duty, -40000.0f, 40000.0f);
        }
    }

    motor_l.duty = left_duty;
    motor_r.duty = right_duty;

    cm_write_pwm_pair(MOTOR_LEFT_FORWARD, MOTOR_LEFT_REVERSE, motor_l.duty);
    cm_write_pwm_pair(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_REVERSE, motor_r.duty);
}

int64_t get_total_encoder(void)
{
    return (motor_l.total_encoder + motor_r.total_encoder) / 2;
}

float radius_3pts(float pt0[2], float pt1[2], float pt2[2])
{
    float ux = pt1[0] - pt0[0];
    float uy = pt1[1] - pt0[1];
    float vx = pt2[0] - pt0[0];
    float vy = pt2[1] - pt0[1];
    float det = ux * vy - uy * vx;
    float a = sqrtf((pt2[0] - pt1[0]) * (pt2[0] - pt1[0]) +
                    (pt2[1] - pt1[1]) * (pt2[1] - pt1[1]));
    float b = sqrtf(vx * vx + vy * vy);
    float c = sqrtf(ux * ux + uy * uy);
    return (a * b * c) / (2.0f * fabsf(det));
}

void square_signal(void)
{
    ++clk;
    if (clk > 10000u) {
        clk = 0u;
    }

    if (clk < 2000u) {
        motor_l.target_speed = 20.0f;
        motor_r.target_speed = 20.0f;
    } else if (clk < 4000u) {
        motor_l.target_speed = 0.0f;
        motor_r.target_speed = 0.0f;
    } else if (clk < 6000u) {
        motor_l.target_speed = 15.0f;
        motor_r.target_speed = 15.0f;
    } else if (clk < 8000u) {
        motor_l.target_speed = 28.0f;
        motor_r.target_speed = 28.0f;
    } else if (clk < 10000u) {
        motor_l.target_speed = 0.0f;
        motor_r.target_speed = 0.0f;
    } else {
        /* clk == 10000: retain previous motor_l.target_speed and motor_r.target_speed. */
    }
}
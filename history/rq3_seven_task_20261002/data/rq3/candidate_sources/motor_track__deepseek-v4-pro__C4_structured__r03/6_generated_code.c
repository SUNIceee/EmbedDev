#include "6_generated_code.h"
#include <math.h>

static float motor_clamp(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

motor_param_t motor_l = {0};
motor_param_t motor_r = {0};
motor_pid_t motor_pid_l = {7021.0f, 10.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};
motor_pid_t motor_pid_r = {7021.0f, 10.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};
motor_pid_t target_speed_pid = {5.0f, 0.0f, 30.0f, 0.0f, 0.0f, 5.0f, 5.0f};
motor_pid_t posloop_pid = {200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};
float target_speed = 0.0f;
uint32_t clk = 0U;

void motor_init(void)
{
    pwm_init(MOTOR_LEFT_FORWARD, 17000U, 0U);
    pwm_init(MOTOR_LEFT_REVERSE, 17000U, 0U);
    pwm_init(MOTOR_RIGHT_FORWARD, 17000U, 0U);
    pwm_init(MOTOR_RIGHT_REVERSE, 17000U, 0U);
}

int64_t get_total_encoder(void)
{
    return (motor_l.total_encoder + motor_r.total_encoder) / 2;
}

float radius_3pts(float pt0[2], float pt1[2], float pt2[2])
{
    double ax = (double)pt1[0] - (double)pt0[0];
    double ay = (double)pt1[1] - (double)pt0[1];
    double bx = (double)pt2[0] - (double)pt0[0];
    double by = (double)pt2[1] - (double)pt0[1];
    double cross = ax * by - ay * bx;
    double area = 0.5 * fabs(cross);

    double side_a_x = (double)pt2[0] - (double)pt1[0];
    double side_a_y = (double)pt2[1] - (double)pt1[1];
    double side_a = sqrt(side_a_x * side_a_x + side_a_y * side_a_y);
    double side_b = sqrt(bx * bx + by * by);
    double side_c = sqrt(ax * ax + ay * ay);

    return (float)((side_a * side_b * side_c) / (4.0 * area));
}

void square_signal(void)
{
    clk += 1U;
    if (clk > 10000U) {
        clk = 0U;
    }

    if (clk < 2000U) {
        motor_l.target_speed = 20.0f;
        motor_r.target_speed = 20.0f;
    } else if (clk < 4000U) {
        motor_l.target_speed = 0.0f;
        motor_r.target_speed = 0.0f;
    } else if (clk < 6000U) {
        motor_l.target_speed = 15.0f;
        motor_r.target_speed = 15.0f;
    } else if (clk < 8000U) {
        motor_l.target_speed = 28.0f;
        motor_r.target_speed = 28.0f;
    } else if (clk < 10000U) {
        motor_l.target_speed = 0.0f;
        motor_r.target_speed = 0.0f;
    } else if (clk == 10000U) {
        /* Retain the previous wheel target speeds. */
    }
}

void speed_control(void)
{
    motor_l.motor_mode = MODE_NORMAL;
    motor_r.motor_mode = MODE_NORMAL;
    float diff = 15.8f * tanf(motor_env.angle * MOTOR_PI * 2.4f / 180.0f) / 40.0f / 2.0f;

    if (motor_env.fa_type == ANIMAL && (uint32_t)(motor_env.now_ms - motor_env.animal_time_ms) < 2500U) {
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
        motor_l.motor_mode = MODE_NORMAL;
        motor_r.motor_mode = MODE_NORMAL;
    } else if (motor_env.garage_type == GARAGE_OUT_LEFT || motor_env.garage_type == GARAGE_OUT_RIGHT) {
        target_speed = 14.0f;
        motor_l.motor_mode = MODE_SOFT;
        motor_r.motor_mode = MODE_SOFT;
    } else if (motor_env.garage_type == GARAGE_IN_LEFT || motor_env.garage_type == GARAGE_IN_RIGHT) {
        target_speed = 10.0f;
        motor_l.motor_mode = MODE_NORMAL;
        motor_r.motor_mode = MODE_NORMAL;
    } else if (motor_env.enable_adc != 0) {
        target_speed = 9.0f;
        motor_l.motor_mode = MODE_BANGBANG;
        motor_r.motor_mode = MODE_BANGBANG;
    } else if (motor_env.yroad_type == YROAD_NEAR || motor_env.yroad_type == YROAD_FOUND) {
        target_speed = 3.0f;
        motor_l.motor_mode = MODE_NORMAL;
        motor_r.motor_mode = MODE_NORMAL;
    } else if (motor_env.circle_type == CIRCLE_LEFT_BEGIN || motor_env.circle_type == CIRCLE_RIGHT_BEGIN) {
        target_speed = motor_clamp(target_speed - 0.02f, 11.0f, 17.0f);
        motor_l.motor_mode = MODE_NORMAL;
        motor_r.motor_mode = MODE_NORMAL;
    } else if (motor_env.rptsn_num > 20U) {
        unsigned id = motor_env.rptsn_num - 1U;
        if (id > 70U) {
            id = 70U;
        }
        float error = fabsf((motor_env.rptsn[id][0] - motor_env.rptsn[0][0]) /
                            (motor_env.rptsn[id][1] - motor_env.rptsn[0][1]));
        float pid_result = pid_solve(&target_speed_pid, error);
        target_speed = motor_clamp(17.0f - pid_result, 9.0f, 17.0f);
    } else if (motor_env.rptsn_num > 5U && motor_env.rptsn_num <= 20U) {
        target_speed = 9.0f;
    } else {
        motor_l.motor_mode = MODE_NORMAL;
        motor_r.motor_mode = MODE_NORMAL;
    }

    if (motor_env.garage_type == GARAGE_STOP ||
        ((motor_env.garage_type != GARAGE_OUT_LEFT && motor_env.garage_type != GARAGE_OUT_RIGHT) &&
         (motor_env.elec_data[0] + motor_env.elec_data[1] < 60.0f))) {
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

    switch (motor_l.motor_mode) {
    case MODE_NORMAL:
        motor_l.duty = pid_solve(&motor_pid_l, left_error);
        break;
    case MODE_BANGBANG:
        motor_pid_l.out_i = 0.0f;
        motor_l.duty += bangbang_pid_solve(&motor_l.brake_pid, left_error);
        break;
    case MODE_SOFT:
        motor_pid_l.out_i = 0.0f;
        motor_l.duty += changable_pid_solve(&motor_l.pid, left_error);
        break;
    case MODE_POSLOOP:
        motor_pid_l.out_i = 0.0f;
        motor_l.duty = pid_solve(&posloop_pid, (float)(motor_l.target_encoder - motor_l.total_encoder));
        break;
    }

    switch (motor_r.motor_mode) {
    case MODE_NORMAL:
        motor_r.duty = pid_solve(&motor_pid_r, right_error);
        break;
    case MODE_BANGBANG:
        motor_pid_r.out_i = 0.0f;
        motor_r.duty += bangbang_pid_solve(&motor_r.brake_pid, right_error);
        break;
    case MODE_SOFT:
        motor_pid_r.out_i = 0.0f;
        motor_r.duty += changable_pid_solve(&motor_r.pid, right_error);
        break;
    case MODE_POSLOOP:
        motor_pid_r.out_i = 0.0f;
        motor_r.duty = pid_solve(&posloop_pid, (float)(motor_r.target_encoder - motor_r.total_encoder));
        break;
    }

    motor_l.duty = motor_clamp(motor_l.duty, -50000.0f, 50000.0f);
    motor_r.duty = motor_clamp(motor_r.duty, -50000.0f, 50000.0f);

    float angle_abs = fabsf(motor_env.angle);
    if (angle_abs > 10.0f) {
        float encoder_avg = (motor_l.encoder_speed + motor_r.encoder_speed) / 2.0f;
        if ((target_speed - encoder_avg) < 0.0f) {
            motor_l.duty = motor_clamp(motor_l.duty, -50000.0f, 40000.0f);
            motor_r.duty = motor_clamp(motor_r.duty, -50000.0f, 40000.0f);
        } else {
            motor_l.duty = motor_clamp(motor_l.duty, -40000.0f, 40000.0f);
            motor_r.duty = motor_clamp(motor_r.duty, -40000.0f, 40000.0f);
        }
    }

    if (motor_l.duty >= 0.0f) {
        pwm_duty(MOTOR_LEFT_FORWARD, (uint32_t)motor_l.duty);
        pwm_duty(MOTOR_LEFT_REVERSE, 0U);
    } else {
        pwm_duty(MOTOR_LEFT_FORWARD, 0U);
        pwm_duty(MOTOR_LEFT_REVERSE, (uint32_t)(-motor_l.duty));
    }

    if (motor_r.duty >= 0.0f) {
        pwm_duty(MOTOR_RIGHT_FORWARD, (uint32_t)motor_r.duty);
        pwm_duty(MOTOR_RIGHT_REVERSE, 0U);
    } else {
        pwm_duty(MOTOR_RIGHT_FORWARD, 0U);
        pwm_duty(MOTOR_RIGHT_REVERSE, (uint32_t)(-motor_r.duty));
    }
}
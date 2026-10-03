#include "generated_code.h"
#include <math.h>

motor_param_t motor_l = {0};
motor_param_t motor_r = {0};
motor_pid_t motor_pid_l = {7021.0f, 10.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};
motor_pid_t motor_pid_r = {7021.0f, 10.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};
motor_pid_t target_speed_pid = {5.0f, 0.0f, 30.0f, 0.0f, 0.0f, 5.0f, 5.0f};
motor_pid_t posloop_pid = {200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};
float target_speed = 0.0f;
uint32_t clk = 0u;

static float clampf(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

void motor_init(void)
{
    pwm_init(MOTOR_LEFT_FORWARD, 17000u, 0u);
    pwm_init(MOTOR_LEFT_REVERSE, 17000u, 0u);
    pwm_init(MOTOR_RIGHT_FORWARD, 17000u, 0u);
    pwm_init(MOTOR_RIGHT_REVERSE, 17000u, 0u);
}

float radius_3pts(float points[3][2])
{
    double ax = points[0][0];
    double ay = points[0][1];
    double bx = points[1][0];
    double by = points[1][1];
    double cx = points[2][0];
    double cy = points[2][1];

    double abx = bx - ax;
    double aby = by - ay;
    double acx = cx - ax;
    double acy = cy - ay;

    double cross = abx * acy - aby * acx;
    double area2 = fabs(cross);

    double side_a = sqrt((cx - bx) * (cx - bx) + (cy - by) * (cy - by));
    double side_b = sqrt((cx - ax) * (cx - ax) + (cy - ay) * (cy - ay));
    double side_c = sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));

    double radius = (side_a * side_b * side_c) / (2.0 * area2);
    return (float)radius;
}

int64_t get_total_encoder(void)
{
    return (motor_l.total_encoder + motor_r.total_encoder) / 2;
}

void square_signal(void)
{
    clk = clk + 1u;
    if (clk > 10000u) {
        clk = 0u;
    }

    if (clk == 10000u) {
        return;
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
    } else {
        motor_l.target_speed = 0.0f;
        motor_r.target_speed = 0.0f;
    }
}

void speed_control(void)
{
    float diff = 15.8f * tanf(motor_env.angle * MOTOR_PI * 2.4f / 180.0f) / 40.0f / 2.0f;

    motor_l.motor_mode = MODE_NORMAL;
    motor_r.motor_mode = MODE_NORMAL;

    if (motor_env.fa_type == ANIMAL &&
        (uint32_t)(motor_env.now_ms - motor_env.animal_time_ms) < 2500u) {
        target_speed = 0.0f;
        diff = 0.0f;
    } else if (motor_env.fruit_delta < 0.0f &&
               (motor_env.laser_angle < 5.0f || motor_env.laser_angle > 175.0f)) {
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
    } else if (motor_env.garage_type == GARAGE_OUT_LEFT ||
               motor_env.garage_type == GARAGE_OUT_RIGHT) {
        target_speed = 14.0f;
        motor_l.motor_mode = MODE_SOFT;
        motor_r.motor_mode = MODE_SOFT;
    } else if (motor_env.garage_type == GARAGE_IN_LEFT ||
               motor_env.garage_type == GARAGE_IN_RIGHT) {
        target_speed = 10.0f;
    } else if (motor_env.enable_adc != 0) {
        target_speed = 9.0f;
        motor_l.motor_mode = MODE_BANGBANG;
        motor_r.motor_mode = MODE_BANGBANG;
    } else if (motor_env.yroad_type == YROAD_NEAR) {
        target_speed = 3.0f;
    } else if (motor_env.yroad_type == YROAD_FOUND) {
        target_speed = 3.0f;
    } else if (motor_env.circle_type == CIRCLE_LEFT_BEGIN ||
               motor_env.circle_type == CIRCLE_RIGHT_BEGIN) {
        target_speed = clampf(target_speed - 0.02f, 11.0f, 17.0f);
    } else if (motor_env.rptsn_num > 20u) {
        unsigned id = motor_env.rptsn_num - 1u;
        if (id > 70u) {
            id = 70u;
        }

        float error = fabsf((motor_env.rptsn[id][0] - motor_env.rptsn[0][0]) /
                            (motor_env.rptsn[id][1] - motor_env.rptsn[0][1]));
        float pid_return = pid_solve(&target_speed_pid, error);
        target_speed = clampf(17.0f - pid_return, 9.0f, 17.0f);
    } else if (motor_env.rptsn_num > 5u) {
        target_speed = 9.0f;
    } else {
        /* No matching decision: retain previous target_speed. */
    }

    if (motor_env.garage_type == GARAGE_STOP ||
        (motor_env.garage_type != GARAGE_OUT_LEFT &&
         motor_env.garage_type != GARAGE_OUT_RIGHT &&
         motor_env.elec_data[0] + motor_env.elec_data[1] < 60.0f)) {
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
            motor_l.duty = pid_solve(&posloop_pid,
                                     (float)(motor_l.target_encoder - motor_l.total_encoder));
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
            motor_r.duty = pid_solve(&posloop_pid,
                                     (float)(motor_r.target_encoder - motor_r.total_encoder));
            break;
    }

    motor_l.duty = clampf(motor_l.duty, -50000.0f, 50000.0f);
    motor_r.duty = clampf(motor_r.duty, -50000.0f, 50000.0f);

    float abs_angle = fabsf(motor_env.angle);
    if (abs_angle > 10.0f) {
        float encoder_avg = (motor_l.encoder_speed + motor_r.encoder_speed) * 0.5f;
        if (target_speed - encoder_avg < 0.0f) {
            motor_l.duty = clampf(motor_l.duty, -50000.0f, 40000.0f);
            motor_r.duty = clampf(motor_r.duty, -50000.0f, 40000.0f);
        } else {
            motor_l.duty = clampf(motor_l.duty, -40000.0f, 40000.0f);
            motor_r.duty = clampf(motor_r.duty, -40000.0f, 40000.0f);
        }
    }

    uint32_t left_forward = 0u;
    uint32_t left_reverse = 0u;
    uint32_t right_forward = 0u;
    uint32_t right_reverse = 0u;

    if (motor_l.duty >= 0.0f) {
        left_forward = (uint32_t)motor_l.duty;
    } else {
        left_reverse = (uint32_t)(-motor_l.duty);
    }

    if (motor_r.duty >= 0.0f) {
        right_forward = (uint32_t)motor_r.duty;
    } else {
        right_reverse = (uint32_t)(-motor_r.duty);
    }

    pwm_duty(MOTOR_LEFT_FORWARD, left_forward);
    pwm_duty(MOTOR_LEFT_REVERSE, left_reverse);
    pwm_duty(MOTOR_RIGHT_FORWARD, right_forward);
    pwm_duty(MOTOR_RIGHT_REVERSE, right_reverse);
}

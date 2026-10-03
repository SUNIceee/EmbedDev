#include "6_generated_code.h"
#include <math.h>

motor_param_t motor_l = {0};
motor_param_t motor_r = {0};
motor_pid_t motor_pid_l = {7021.0f, 10.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};
motor_pid_t motor_pid_r = {7021.0f, 10.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};
motor_pid_t target_speed_pid = {5.0f, 0.0f, 30.0f, 0.0f, 0.0f, 5.0f, 5.0f};
motor_pid_t posloop_pid = {200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};
float target_speed = 0.0f;
uint32_t clk = 0;

static float clamp_float(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void motor_init(void) {
    pwm_init(MOTOR_LEFT_FORWARD, 17000u, 0u);
    pwm_init(MOTOR_LEFT_REVERSE, 17000u, 0u);
    pwm_init(MOTOR_RIGHT_FORWARD, 17000u, 0u);
    pwm_init(MOTOR_RIGHT_REVERSE, 17000u, 0u);
}

int64_t get_total_encoder(void) {
    return (motor_l.total_encoder + motor_r.total_encoder) / 2;
}

float radius_3pts(float pt0[2], float pt1[2], float pt2[2]) {
    float dx1 = pt1[0] - pt0[0];
    float dy1 = pt1[1] - pt0[1];
    float dx2 = pt2[0] - pt0[0];
    float dy2 = pt2[1] - pt0[1];
    float cross = dx1 * dy2 - dy1 * dx2;
    float area_twice = fabsf(cross);

    float side_a = sqrtf((pt2[0] - pt1[0]) * (pt2[0] - pt1[0]) +
                         (pt2[1] - pt1[1]) * (pt2[1] - pt1[1]));
    float side_b = sqrtf((pt2[0] - pt0[0]) * (pt2[0] - pt0[0]) +
                         (pt2[1] - pt0[1]) * (pt2[1] - pt0[1]));
    float side_c = sqrtf((pt1[0] - pt0[0]) * (pt1[0] - pt0[0]) +
                         (pt1[1] - pt0[1]) * (pt1[1] - pt0[1]));

    return (side_a * side_b * side_c) / (2.0f * area_twice);
}

void square_signal(void) {
    clk++;
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

void speed_control(void) {
    motor_l.motor_mode = MODE_NORMAL;
    motor_r.motor_mode = MODE_NORMAL;

    float diff = 15.8f * tanf(motor_env.angle * MOTOR_PI * 2.4f / 180.0f) / 40.0f / 2.0f;

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
    } else if (motor_env.yroad_type == YROAD_NEAR ||
               motor_env.yroad_type == YROAD_FOUND) {
        target_speed = 3.0f;
    } else if (motor_env.circle_type == CIRCLE_LEFT_BEGIN ||
               motor_env.circle_type == CIRCLE_RIGHT_BEGIN) {
        target_speed = clamp_float(target_speed - 0.02f, 11.0f, 17.0f);
    } else if (motor_env.rptsn_num > 20u) {
        unsigned id = motor_env.rptsn_num - 1u;
        if (id > 70u) {
            id = 70u;
        }
        float error = fabsf((motor_env.rptsn[id][0] - motor_env.rptsn[0][0]) /
                            (motor_env.rptsn[id][1] - motor_env.rptsn[0][1]));
        float pid_result = pid_solve(&target_speed_pid, error);
        target_speed = clamp_float(17.0f - pid_result, 9.0f, 17.0f);
    } else if (motor_env.rptsn_num > 5u && motor_env.rptsn_num <= 20u) {
        target_speed = 9.0f;
    } else {
        /* rptsn_num <= 5 and no earlier decision: retain previous target_speed. */
    }

    if (motor_env.garage_type == GARAGE_STOP ||
        ((motor_env.garage_type != GARAGE_OUT_LEFT &&
          motor_env.garage_type != GARAGE_OUT_RIGHT) &&
         (motor_env.elec_data[0] + motor_env.elec_data[1] < 60.0f))) {
        target_speed = 0.0f;
        diff = 0.0f;
        motor_l.motor_mode = MODE_NORMAL;
        motor_r.motor_mode = MODE_NORMAL;
    }

    motor_l.target_speed = target_speed - diff * target_speed;
    motor_r.target_speed = target_speed + diff * target_speed;
}

void motor_control(void) {
    float error_l = motor_l.target_speed - motor_l.encoder_speed;
    float error_r = motor_r.target_speed - motor_r.encoder_speed;

    switch (motor_l.motor_mode) {
        case MODE_NORMAL:
            motor_l.duty = pid_solve(&motor_pid_l, error_l);
            break;
        case MODE_BANGBANG:
            motor_pid_l.out_i = 0.0f;
            motor_l.duty += bangbang_pid_solve(&motor_l.brake_pid, error_l);
            break;
        case MODE_SOFT:
            motor_pid_l.out_i = 0.0f;
            motor_l.duty += changable_pid_solve(&motor_l.pid, error_l);
            break;
        case MODE_POSLOOP:
            motor_pid_l.out_i = 0.0f;
            motor_l.duty = pid_solve(&posloop_pid,
                                     (float)(motor_l.target_encoder - motor_l.total_encoder));
            break;
    }

    switch (motor_r.motor_mode) {
        case MODE_NORMAL:
            motor_r.duty = pid_solve(&motor_pid_r, error_r);
            break;
        case MODE_BANGBANG:
            motor_pid_r.out_i = 0.0f;
            motor_r.duty += bangbang_pid_solve(&motor_r.brake_pid, error_r);
            break;
        case MODE_SOFT:
            motor_pid_r.out_i = 0.0f;
            motor_r.duty += changable_pid_solve(&motor_r.pid, error_r);
            break;
        case MODE_POSLOOP:
            motor_pid_r.out_i = 0.0f;
            motor_r.duty = pid_solve(&posloop_pid,
                                     (float)(motor_r.target_encoder - motor_r.total_encoder));
            break;
    }

    motor_l.duty = clamp_float(motor_l.duty, -50000.0f, 50000.0f);
    motor_r.duty = clamp_float(motor_r.duty, -50000.0f, 50000.0f);

    if (fabsf(motor_env.angle) > 10.0f) {
        float average_encoder_speed = (motor_l.encoder_speed + motor_r.encoder_speed) / 2.0f;
        if (target_speed - average_encoder_speed < 0.0f) {
            motor_l.duty = clamp_float(motor_l.duty, -50000.0f, 40000.0f);
            motor_r.duty = clamp_float(motor_r.duty, -50000.0f, 40000.0f);
        } else {
            motor_l.duty = clamp_float(motor_l.duty, -40000.0f, 40000.0f);
            motor_r.duty = clamp_float(motor_r.duty, -40000.0f, 40000.0f);
        }
    }

    if (motor_l.duty >= 0.0f) {
        pwm_duty(MOTOR_LEFT_FORWARD, (uint32_t)motor_l.duty);
        pwm_duty(MOTOR_LEFT_REVERSE, 0u);
    } else {
        pwm_duty(MOTOR_LEFT_FORWARD, 0u);
        pwm_duty(MOTOR_LEFT_REVERSE, (uint32_t)(-motor_l.duty));
    }

    if (motor_r.duty >= 0.0f) {
        pwm_duty(MOTOR_RIGHT_FORWARD, (uint32_t)motor_r.duty);
        pwm_duty(MOTOR_RIGHT_REVERSE, 0u);
    } else {
        pwm_duty(MOTOR_RIGHT_FORWARD, 0u);
        pwm_duty(MOTOR_RIGHT_REVERSE, (uint32_t)(-motor_r.duty));
    }
}

#include "generated_code.h"
#include <math.h>

motor_param_t motor_l = {0};
motor_param_t motor_r = {0};
float target_speed = 0.0f;
uint32_t clk = 0;

motor_pid_t motor_pid_l = {7021.0f, 10.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};
motor_pid_t motor_pid_r = {7021.0f, 10.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};
motor_pid_t target_speed_pid = {5.0f, 0.0f, 30.0f, 0.0f, 0.0f, 5.0f, 5.0f};
motor_pid_t posloop_pid = {200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 50000.0f};

static float clampf(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

void motor_init(void) {
    pwm_init(MOTOR_LEFT_FORWARD, 17000u, 0u);
    pwm_init(MOTOR_LEFT_REVERSE, 17000u, 0u);
    pwm_init(MOTOR_RIGHT_FORWARD, 17000u, 0u);
    pwm_init(MOTOR_RIGHT_REVERSE, 17000u, 0u);
}

float radius_3pts(float x1, float y1, float x2, float y2, float x3, float y3) {
    double dx12 = (double)x2 - (double)x1;
    double dy12 = (double)y2 - (double)y1;
    double dx13 = (double)x3 - (double)x1;
    double dy13 = (double)y3 - (double)y1;
    double dx23 = (double)x3 - (double)x2;
    double dy23 = (double)y3 - (double)y2;

    double a = sqrt(dx12 * dx12 + dy12 * dy12);
    double b = sqrt(dx13 * dx13 + dy13 * dy13);
    double c = sqrt(dx23 * dx23 + dy23 * dy23);

    double area = 0.5 * fabs(dx12 * dy13 - dx13 * dy12);
    return (float)(a * b * c / (4.0 * area));
}

int64_t get_total_encoder(void) {
    int64_t sum = motor_l.total_encoder + motor_r.total_encoder;
    if (sum >= 0) {
        return sum / 2;
    }
    return -((-sum) / 2);
}

void square_signal(void) {
    clk++;
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
        /* clk == 10000: retain previous two wheel target speeds */
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
        motor_l.motor_mode = MODE_NORMAL;
        motor_r.motor_mode = MODE_NORMAL;
    } else if (motor_env.garage_type == GARAGE_OUT_LEFT ||
               motor_env.garage_type == GARAGE_OUT_RIGHT) {
        target_speed = 14.0f;
        motor_l.motor_mode = MODE_SOFT;
        motor_r.motor_mode = MODE_SOFT;
    } else if (motor_env.garage_type == GARAGE_IN_LEFT ||
               motor_env.garage_type == GARAGE_IN_RIGHT) {
        target_speed = 10.0f;
        motor_l.motor_mode = MODE_NORMAL;
        motor_r.motor_mode = MODE_NORMAL;
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

        float dx = motor_env.rptsn[id][0] - motor_env.rptsn[0][0];
        float dy = motor_env.rptsn[id][1] - motor_env.rptsn[0][1];
        float error = fabsf(dx / dy);
        float returned = pid_solve(&target_speed_pid, error);
        target_speed = clampf(17.0f - returned, 9.0f, 17.0f);
    } else if (motor_env.rptsn_num > 5u && motor_env.rptsn_num <= 20u) {
        target_speed = 9.0f;
    } else if (motor_env.rptsn_num <= 5u) {
        /* No matching decision: retain previous scalar target_speed. */
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
    float left_error = motor_l.target_speed - motor_l.encoder_speed;
    float right_error = motor_r.target_speed - motor_r.encoder_speed;

    int64_t total_encoder = get_total_encoder();
    float left_pos_error = (float)(motor_l.target_encoder - total_encoder);
    float right_pos_error = (float)(motor_r.target_encoder - total_encoder);

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
            motor_l.duty = pid_solve(&posloop_pid, left_pos_error);
            break;
        default:
            motor_l.duty = 0.0f;
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
            motor_r.duty = pid_solve(&posloop_pid, right_pos_error);
            break;
        default:
            motor_r.duty = 0.0f;
            break;
    }

    motor_l.duty = clampf(motor_l.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
    motor_r.duty = clampf(motor_r.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);

    if (fabsf(motor_env.angle) > 10.0f) {
        float avg_encoder_speed = (motor_l.encoder_speed + motor_r.encoder_speed) / 2.0f;
        if (target_speed - avg_encoder_speed < 0.0f) {
            motor_l.duty = clampf(motor_l.duty, -50000.0f, 40000.0f);
            motor_r.duty = clampf(motor_r.duty, -50000.0f, 40000.0f);
        } else {
            motor_l.duty = clampf(motor_l.duty, -40000.0f, 40000.0f);
            motor_r.duty = clampf(motor_r.duty, -40000.0f, 40000.0f);
        }
    }

    int32_t left_duty_i = (int32_t)motor_l.duty;
    int32_t right_duty_i = (int32_t)motor_r.duty;

    uint32_t left_forward = 0u;
    uint32_t left_reverse = 0u;
    uint32_t right_forward = 0u;
    uint32_t right_reverse = 0u;

    if (left_duty_i >= 0) {
        left_forward = (uint32_t)left_duty_i;
    } else {
        left_reverse = (uint32_t)(-left_duty_i);
    }

    if (right_duty_i >= 0) {
        right_forward = (uint32_t)right_duty_i;
    } else {
        right_reverse = (uint32_t)(-right_duty_i);
    }

    pwm_duty(MOTOR_LEFT_FORWARD, left_forward);
    pwm_duty(MOTOR_LEFT_REVERSE, left_reverse);
    pwm_duty(MOTOR_RIGHT_FORWARD, right_forward);
    pwm_duty(MOTOR_RIGHT_REVERSE, right_reverse);
}

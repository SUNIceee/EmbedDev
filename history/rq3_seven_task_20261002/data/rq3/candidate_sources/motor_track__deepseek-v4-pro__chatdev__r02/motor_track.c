
/* motor_track.c - Implementation of the fixed motor_track application API. */
#include "6_generated_code.h"
#include <math.h>

#define NORMAL_SPEED 17.0f
#define NORMAL_MIN_SPEED -8.0f

static float clampf(float value, float low, float high)
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

motor_pid_t motor_pid_l = {
    .kp = 7021.0f,
    .ki = 10.0f,
    .kd = 0.0f,
    .out_i = 0.0f,
    .last_error = 0.0f,
    .max_output = 50000.0f,
    .max_integral = 50000.0f
};

motor_pid_t motor_pid_r = {
    .kp = 7021.0f,
    .ki = 10.0f,
    .kd = 0.0f,
    .out_i = 0.0f,
    .last_error = 0.0f,
    .max_output = 50000.0f,
    .max_integral = 50000.0f
};

motor_pid_t target_speed_pid = {
    .kp = 5.0f,
    .ki = 0.0f,
    .kd = 30.0f,
    .out_i = 0.0f,
    .last_error = 0.0f,
    .max_output = 5.0f,
    .max_integral = 5.0f
};

motor_pid_t posloop_pid = {
    .kp = 200.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .out_i = 0.0f,
    .last_error = 0.0f,
    .max_output = 50000.0f,
    .max_integral = 50000.0f
};

float target_speed = 0.0f;
uint32_t clk = 0u;

void motor_init(void)
{
    pwm_init(MOTOR_LEFT_FORWARD, 17000u, 0u);
    pwm_init(MOTOR_LEFT_REVERSE, 17000u, 0u);
    pwm_init(MOTOR_RIGHT_FORWARD, 17000u, 0u);
    pwm_init(MOTOR_RIGHT_REVERSE, 17000u, 0u);
}

int64_t get_total_encoder(void)
{
    int64_t sum = motor_l.total_encoder + motor_r.total_encoder;
    return sum / 2;
}

float radius_3pts(float pt0[2], float pt1[2], float pt2[2])
{
    double ax = (double)pt0[0] - (double)pt2[0];
    double ay = (double)pt0[1] - (double)pt2[1];
    double bx = (double)pt1[0] - (double)pt2[0];
    double by = (double)pt1[1] - (double)pt2[1];
    double cross = ax * by - ay * bx;
    double twice_area = fabs(cross);

    double dx01 = (double)pt0[0] - (double)pt1[0];
    double dy01 = (double)pt0[1] - (double)pt1[1];
    double dx12 = (double)pt1[0] - (double)pt2[0];
    double dy12 = (double)pt1[1] - (double)pt2[1];
    double dx20 = (double)pt2[0] - (double)pt0[0];
    double dy20 = (double)pt2[1] - (double)pt0[1];

    double side01 = sqrt(dx01 * dx01 + dy01 * dy01);
    double side12 = sqrt(dx12 * dx12 + dy12 * dy12);
    double side20 = sqrt(dx20 * dx20 + dy20 * dy20);

    return (float)((side01 * side12 * side20) / (2.0 * twice_area));
}

void square_signal(void)
{
    clk = clk + 1u;
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
    } else if (clk == 10000u) {
        /* Retain the previous two wheel target speeds at exactly 10000. */
    }
}

void speed_control(void)
{
    float diff;
    uint32_t elapsed;

    motor_l.motor_mode = MODE_NORMAL;
    motor_r.motor_mode = MODE_NORMAL;

    diff = 15.8f * tanf(motor_env.angle * MOTOR_PI * 2.4f / 180.0f) / 40.0f / 2.0f;

    elapsed = motor_env.now_ms - motor_env.animal_time_ms;

    if (motor_env.fa_type == ANIMAL && elapsed < 2500u) {
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
        float denominator;
        float numerator;
        float error;
        float pid_out;

        if (id > 70u) {
            id = 70u;
        }

        numerator = motor_env.rptsn[id][0] - motor_env.rptsn[0][0];
        denominator = motor_env.rptsn[id][1] - motor_env.rptsn[0][1];
        error = fabsf(numerator / denominator);
        pid_out = pid_solve(&target_speed_pid, error);
        target_speed = clampf(NORMAL_SPEED - pid_out, 9.0f, NORMAL_SPEED);
    } else if (motor_env.rptsn_num > 5u && motor_env.rptsn_num <= 20u) {
        target_speed = 9.0f;
    } else if (motor_env.rptsn_num <= 5u) {
        /* Retain previous scalar target_speed. Differential distribution is recomputed below. */
    }

    if (motor_env.garage_type == GARAGE_STOP ||
        (!(motor_env.garage_type == GARAGE_OUT_LEFT ||
           motor_env.garage_type == GARAGE_OUT_RIGHT) &&
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
    float left_duty;
    float right_duty;

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
            left_duty = pid_solve(&posloop_pid,
                                  (float)(motor_l.target_encoder - motor_l.total_encoder));
            break;
        default:
            motor_pid_l.out_i = 0.0f;
            left_duty = 0.0f;
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
            right_duty = pid_solve(&posloop_pid,
                                   (float)(motor_r.target_encoder - motor_r.total_encoder));
            break;
        default:
            motor_pid_r.out_i = 0.0f;
            right_duty = 0.0f;
            break;
    }

    left_duty = clampf(left_duty, -50000.0f, 50000.0f);
    right_duty = clampf(right_duty, -50000.0f, 50000.0f);

    if (fabsf(motor_env.angle) > 10.0f) {
        float average_encoder_speed = (motor_l.encoder_speed + motor_r.encoder_speed) / 2.0f;

        if (target_speed - average_encoder_speed < 0.0f) {
            left_duty = clampf(left_duty, -50000.0f, 40000.0f);
            right_duty = clampf(right_duty, -50000.0f, 40000.0f);
        } else {
            left_duty = clampf(left_duty, -40000.0f, 40000.0f);
            right_duty = clampf(right_duty, -40000.0f, 40000.0f);
        }
    }

    motor_l.duty = left_duty;
    motor_r.duty = right_duty;

    if (left_duty >= 0.0f) {
        pwm_duty(MOTOR_LEFT_FORWARD, (uint32_t)left_duty);
        pwm_duty(MOTOR_LEFT_REVERSE, 0u);
    } else {
        pwm_duty(MOTOR_LEFT_FORWARD, 0u);
        pwm_duty(MOTOR_LEFT_REVERSE, (uint32_t)(-left_duty));
    }

    if (right_duty >= 0.0f) {
        pwm_duty(MOTOR_RIGHT_FORWARD, (uint32_t)right_duty);
        pwm_duty(MOTOR_RIGHT_REVERSE, 0u);
    } else {
        pwm_duty(MOTOR_RIGHT_FORWARD, 0u);
        pwm_duty(MOTOR_RIGHT_REVERSE, (uint32_t)(-right_duty));
    }
}

#include "6_generated_code.h"
#include <math.h>
#include <string.h>

motor_param_t motor_l = {0};
motor_param_t motor_r = {0};
motor_pid_t motor_pid_l = {0};
motor_pid_t motor_pid_r = {0};
motor_pid_t target_speed_pid = {0};
motor_pid_t posloop_pid = {0};
float target_speed = 0.0f;
uint32_t clk = 0;

static float clamp_value(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void write_wheel_pwm(motor_pwm_channel_t forward, motor_pwm_channel_t reverse, float duty)
{
    if (duty >= 0.0f) {
        pwm_duty(forward, (uint32_t)duty);
        pwm_duty(reverse, 0u);
    } else {
        pwm_duty(forward, 0u);
        pwm_duty(reverse, (uint32_t)(-duty));
    }
}

void motor_init(void)
{
    pwm_init(MOTOR_LEFT_FORWARD, 17000u, 0u);
    pwm_init(MOTOR_LEFT_REVERSE, 17000u, 0u);
    pwm_init(MOTOR_RIGHT_FORWARD, 17000u, 0u);
    pwm_init(MOTOR_RIGHT_REVERSE, 17000u, 0u);

    motor_pid_l.kp = 7021.0f;
    motor_pid_l.ki = 10.0f;
    motor_pid_l.kd = 0.0f;
    motor_pid_l.out_i = 0.0f;
    motor_pid_l.last_error = 0.0f;
    motor_pid_l.max_output = 50000.0f;
    motor_pid_l.max_integral = 50000.0f;

    motor_pid_r.kp = 7021.0f;
    motor_pid_r.ki = 10.0f;
    motor_pid_r.kd = 0.0f;
    motor_pid_r.out_i = 0.0f;
    motor_pid_r.last_error = 0.0f;
    motor_pid_r.max_output = 50000.0f;
    motor_pid_r.max_integral = 50000.0f;

    target_speed_pid.kp = 5.0f;
    target_speed_pid.ki = 0.0f;
    target_speed_pid.kd = 30.0f;
    target_speed_pid.out_i = 0.0f;
    target_speed_pid.last_error = 0.0f;
    target_speed_pid.max_output = 5.0f;
    target_speed_pid.max_integral = 5.0f;

    posloop_pid.kp = 200.0f;
    posloop_pid.ki = 0.0f;
    posloop_pid.kd = 0.0f;
    posloop_pid.out_i = 0.0f;
    posloop_pid.last_error = 0.0f;
    posloop_pid.max_output = 50000.0f;
    posloop_pid.max_integral = 50000.0f;

    memset(&motor_l, 0, sizeof(motor_l));
    memset(&motor_r, 0, sizeof(motor_r));
    motor_l.motor_mode = MODE_NORMAL;
    motor_r.motor_mode = MODE_NORMAL;

    target_speed = 0.0f;
    clk = 0;
}

int64_t get_total_encoder(void)
{
    return (motor_l.total_encoder + motor_r.total_encoder) / 2;
}

float radius_3pts(float pt0[2], float pt1[2], float pt2[2])
{
    double ax = (double)pt2[0] - (double)pt1[0];
    double ay = (double)pt2[1] - (double)pt1[1];
    double a = sqrt(ax * ax + ay * ay);

    double bx = (double)pt2[0] - (double)pt0[0];
    double by = (double)pt2[1] - (double)pt0[1];
    double b = sqrt(bx * bx + by * by);

    double cx = (double)pt1[0] - (double)pt0[0];
    double cy = (double)pt1[1] - (double)pt0[1];
    double c = sqrt(cx * cx + cy * cy);

    double det = ((double)pt1[0] - (double)pt0[0]) * ((double)pt2[1] - (double)pt0[1])
               - ((double)pt1[1] - (double)pt0[1]) * ((double)pt2[0] - (double)pt0[0]);

    double area = fabs(det) / 2.0;
    double radius = (a * b * c) / (4.0 * area);

    return (float)radius;
}

void square_signal(void)
{
    clk++;
    if (clk > 10000u) {
        clk = 0;
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
        /* clk == 10000: retain previous wheel target speeds */
    }
}

void speed_control(void)
{
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
        target_speed = clamp_value(target_speed - 0.02f, 11.0f, 17.0f);
    } else if (motor_env.rptsn_num > 20u) {
        unsigned id = motor_env.rptsn_num - 1u;
        if (id > 70u) {
            id = 70u;
        }
        float error = fabsf((motor_env.rptsn[id][0] - motor_env.rptsn[0][0]) /
                            (motor_env.rptsn[id][1] - motor_env.rptsn[0][1]));
        float pid_out = pid_solve(&target_speed_pid, error);
        target_speed = clamp_value(17.0f - pid_out, 9.0f, 17.0f);
    } else if (motor_env.rptsn_num > 5u && motor_env.rptsn_num <= 20u) {
        target_speed = 9.0f;
    } else if (motor_env.rptsn_num <= 5u) {
        /* No matching decision: retain previous scalar target_speed. */
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
        default:
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
        default:
            break;
    }

    motor_l.duty = clamp_value(motor_l.duty, -50000.0f, 50000.0f);
    motor_r.duty = clamp_value(motor_r.duty, -50000.0f, 50000.0f);

    float angle_abs = fabsf(motor_env.angle);
    if (angle_abs > 10.0f) {
        float avg_encoder_speed = (motor_l.encoder_speed + motor_r.encoder_speed) * 0.5f;
        if (target_speed - avg_encoder_speed < 0.0f) {
            motor_l.duty = clamp_value(motor_l.duty, -50000.0f, 40000.0f);
            motor_r.duty = clamp_value(motor_r.duty, -50000.0f, 40000.0f);
        } else {
            motor_l.duty = clamp_value(motor_l.duty, -40000.0f, 40000.0f);
            motor_r.duty = clamp_value(motor_r.duty, -40000.0f, 40000.0f);
        }
    }

    write_wheel_pwm(MOTOR_LEFT_FORWARD, MOTOR_LEFT_REVERSE, motor_l.duty);
    write_wheel_pwm(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_REVERSE, motor_r.duty);
}

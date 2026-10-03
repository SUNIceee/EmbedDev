#include "6_generated_code.h"
#include <math.h>

motor_param_t motor_l = {
    .target_speed = 0.0f,
    .encoder_speed = 0.0f,
    .duty = 0.0f,
    .total_encoder = 0,
    .target_encoder = 0,
    .motor_mode = MODE_NORMAL,
    .brake_pid = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .pid = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};

motor_param_t motor_r = {
    .target_speed = 0.0f,
    .encoder_speed = 0.0f,
    .duty = 0.0f,
    .total_encoder = 0,
    .target_encoder = 0,
    .motor_mode = MODE_NORMAL,
    .brake_pid = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .pid = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};

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
uint32_t clk = 0U;

static float clamp_float(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static uint32_t min_u32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

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
    double dx_ab = (double)pt1[0] - (double)pt0[0];
    double dy_ab = (double)pt1[1] - (double)pt0[1];
    double dx_ac = (double)pt2[0] - (double)pt0[0];
    double dy_ac = (double)pt2[1] - (double)pt0[1];
    double dx_bc = (double)pt2[0] - (double)pt1[0];
    double dy_bc = (double)pt2[1] - (double)pt1[1];

    double det = dx_ab * dy_ac - dy_ab * dx_ac;
    double side_ab = sqrt(dx_ab * dx_ab + dy_ab * dy_ab);
    double side_ac = sqrt(dx_ac * dx_ac + dy_ac * dy_ac);
    double side_bc = sqrt(dx_bc * dx_bc + dy_bc * dy_bc);

    return (float)((side_ab * side_ac * side_bc) / (2.0 * fabs(det)));
}

void square_signal(void)
{
    clk = clk + 1U;
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
    }
}

void speed_control(void)
{
    motor_l.motor_mode = MODE_NORMAL;
    motor_r.motor_mode = MODE_NORMAL;

    float diff = 15.8f * tanf(motor_env.angle * MOTOR_PI * 2.4f / 180.0f) / 40.0f / 2.0f;

    if (motor_env.fa_type == ANIMAL &&
        (uint32_t)(motor_env.now_ms - motor_env.animal_time_ms) < 2500U) {
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
    } else if (motor_env.rptsn_num > 20U) {
        uint32_t id = min_u32(70U, motor_env.rptsn_num - 1U);
        float error = fabsf((motor_env.rptsn[id][0] - motor_env.rptsn[0][0]) /
                            (motor_env.rptsn[id][1] - motor_env.rptsn[0][1]));
        float returned_value = pid_solve(&target_speed_pid, error);
        target_speed = clamp_float(17.0f - returned_value, 9.0f, 17.0f);
    } else if (motor_env.rptsn_num > 5U && motor_env.rptsn_num <= 20U) {
        target_speed = 9.0f;
    } else if (motor_env.rptsn_num <= 5U) {
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
            motor_l.duty = motor_l.duty + bangbang_pid_solve(&motor_l.brake_pid, left_error);
            break;
        case MODE_SOFT:
            motor_pid_l.out_i = 0.0f;
            motor_l.duty = motor_l.duty + changable_pid_solve(&motor_l.pid, left_error);
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
            motor_r.duty = pid_solve(&motor_pid_r, right_error);
            break;
        case MODE_BANGBANG:
            motor_pid_r.out_i = 0.0f;
            motor_r.duty = motor_r.duty + bangbang_pid_solve(&motor_r.brake_pid, right_error);
            break;
        case MODE_SOFT:
            motor_pid_r.out_i = 0.0f;
            motor_r.duty = motor_r.duty + changable_pid_solve(&motor_r.pid, right_error);
            break;
        case MODE_POSLOOP:
            motor_pid_r.out_i = 0.0f;
            motor_r.duty = pid_solve(&posloop_pid,
                                     (float)(motor_r.target_encoder - motor_r.total_encoder));
            break;
        default:
            break;
    }

    motor_l.duty = clamp_float(motor_l.duty, -50000.0f, 50000.0f);
    motor_r.duty = clamp_float(motor_r.duty, -50000.0f, 50000.0f);

    if (fabsf(motor_env.angle) > 10.0f) {
        float speed_deficit = target_speed -
                              ((motor_l.encoder_speed + motor_r.encoder_speed) / 2.0f);
        if (speed_deficit < 0.0f) {
            motor_l.duty = clamp_float(motor_l.duty, -50000.0f, 40000.0f);
            motor_r.duty = clamp_float(motor_r.duty, -50000.0f, 40000.0f);
        } else {
            motor_l.duty = clamp_float(motor_l.duty, -40000.0f, 40000.0f);
            motor_r.duty = clamp_float(motor_r.duty, -40000.0f, 40000.0f);
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

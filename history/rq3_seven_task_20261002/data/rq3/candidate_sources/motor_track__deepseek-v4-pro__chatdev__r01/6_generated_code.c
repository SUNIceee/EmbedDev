/* Implementation of the track-aware differential motor control contract. */
#include "6_generated_code.h"
#include <math.h>

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
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
    return (motor_l.total_encoder + motor_r.total_encoder) / 2;
}

float radius_3pts(float pt0[2], float pt1[2], float pt2[2])
{
    double x0 = pt0[0];
    double y0 = pt0[1];
    double x1 = pt1[0];
    double y1 = pt1[1];
    double x2 = pt2[0];
    double y2 = pt2[1];

    double det = x0 * (y1 - y2) + x1 * (y2 - y0) + x2 * (y0 - y1);
    double a = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    double b = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    double c = sqrt((x2 - x0) * (x2 - x0) + (y2 - y0) * (y2 - y0));

    return (float)((a * b * c) / (2.0 * fabs(det)));
}

void square_signal(void)
{
    clk++;
    if (clk > 10000u)
    {
        clk = 0u;
    }

    if (clk < 2000u)
    {
        motor_l.target_speed = 20.0f;
        motor_r.target_speed = 20.0f;
    }
    else if (clk < 4000u)
    {
        motor_l.target_speed = 0.0f;
        motor_r.target_speed = 0.0f;
    }
    else if (clk < 6000u)
    {
        motor_l.target_speed = 15.0f;
        motor_r.target_speed = 15.0f;
    }
    else if (clk < 8000u)
    {
        motor_l.target_speed = 28.0f;
        motor_r.target_speed = 28.0f;
    }
    else if (clk < 10000u)
    {
        motor_l.target_speed = 0.0f;
        motor_r.target_speed = 0.0f;
    }
    else
    {
        /* At exactly clk == 10000, retain the previous wheel target speeds. */
    }
}

void speed_control(void)
{
    motor_l.motor_mode = MODE_NORMAL;
    motor_r.motor_mode = MODE_NORMAL;

    float diff = 15.8f * tanf(motor_env.angle * MOTOR_PI * 2.4f / 180.0f) / 40.0f / 2.0f;

    if (motor_env.fa_type == ANIMAL &&
        (uint32_t)(motor_env.now_ms - motor_env.animal_time_ms) < 2500u)
    {
        target_speed = 0.0f;
        diff = 0.0f;
    }
    else if (motor_env.fruit_delta < 0.0f &&
             (motor_env.laser_angle < 5.0f || motor_env.laser_angle > 175.0f))
    {
        target_speed = -1.0f;
    }
    else if (motor_env.tag_type == TAG_SEARCH)
    {
        target_speed = 1.0f;
    }
    else if (motor_env.tag_type == TAG_STOP || motor_env.tag_type == TAG_SHOOTING)
    {
        target_speed = 0.0f;
    }
    else if (motor_env.apriltag_type == APRILTAG_FOUND)
    {
        target_speed = 0.5f;
        diff = 0.0f;
    }
    else if (motor_env.apriltag_type == APRILTAG_MAYBE)
    {
        target_speed = 1.0f;
    }
    else if (motor_env.garage_type == GARAGE_OUT_LEFT ||
             motor_env.garage_type == GARAGE_OUT_RIGHT)
    {
        target_speed = 14.0f;
        motor_l.motor_mode = MODE_SOFT;
        motor_r.motor_mode = MODE_SOFT;
    }
    else if (motor_env.garage_type == GARAGE_IN_LEFT ||
             motor_env.garage_type == GARAGE_IN_RIGHT)
    {
        target_speed = 10.0f;
    }
    else if (motor_env.enable_adc != 0)
    {
        target_speed = 9.0f;
        motor_l.motor_mode = MODE_BANGBANG;
        motor_r.motor_mode = MODE_BANGBANG;
    }
    else if (motor_env.yroad_type == YROAD_NEAR)
    {
        target_speed = 3.0f;
    }
    else if (motor_env.yroad_type == YROAD_FOUND)
    {
        target_speed = 3.0f;
    }
    else if (motor_env.circle_type == CIRCLE_LEFT_BEGIN ||
             motor_env.circle_type == CIRCLE_RIGHT_BEGIN)
    {
        target_speed = clamp_float(target_speed - 0.02f, 11.0f, 17.0f);
    }
    else if (motor_env.rptsn_num > 20u)
    {
        unsigned id = motor_env.rptsn_num - 1u;
        if (id > 70u)
        {
            id = 70u;
        }

        float error = fabsf((motor_env.rptsn[id][0] - motor_env.rptsn[0][0]) /
                            (motor_env.rptsn[id][1] - motor_env.rptsn[0][1]));
        float returned = pid_solve(&target_speed_pid, error);
        target_speed = clamp_float(17.0f - returned, 9.0f, 17.0f);
    }
    else if (motor_env.rptsn_num > 5u && motor_env.rptsn_num <= 20u)
    {
        target_speed = 9.0f;
    }
    else if (motor_env.rptsn_num <= 5u)
    {
        /* Retain the previous scalar target_speed. */
    }

    if (motor_env.garage_type == GARAGE_STOP ||
        ((motor_env.garage_type != GARAGE_OUT_LEFT &&
          motor_env.garage_type != GARAGE_OUT_RIGHT) &&
         (motor_env.elec_data[0] + motor_env.elec_data[1] < 60.0f)))
    {
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
    float left_duty;
    float right_duty;

    float left_error = motor_l.target_speed - motor_l.encoder_speed;
    float right_error = motor_r.target_speed - motor_r.encoder_speed;

    switch (motor_l.motor_mode)
    {
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
        left_duty = 0.0f;
        break;
    }

    switch (motor_r.motor_mode)
    {
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
        right_duty = 0.0f;
        break;
    }

    left_duty = clamp_float(left_duty, -50000.0f, 50000.0f);
    right_duty = clamp_float(right_duty, -50000.0f, 50000.0f);

    if (fabsf(motor_env.angle) > 10.0f)
    {
        float encoder_average = (motor_l.encoder_speed + motor_r.encoder_speed) * 0.5f;

        if (target_speed - encoder_average < 0.0f)
        {
            left_duty = clamp_float(left_duty, -50000.0f, 40000.0f);
            right_duty = clamp_float(right_duty, -50000.0f, 40000.0f);
        }
        else
        {
            left_duty = clamp_float(left_duty, -40000.0f, 40000.0f);
            right_duty = clamp_float(right_duty, -40000.0f, 40000.0f);
        }
    }

    motor_l.duty = left_duty;
    motor_r.duty = right_duty;

    if (left_duty >= 0.0f)
    {
        pwm_duty(MOTOR_LEFT_FORWARD, (uint32_t)left_duty);
        pwm_duty(MOTOR_LEFT_REVERSE, 0u);
    }
    else
    {
        pwm_duty(MOTOR_LEFT_FORWARD, 0u);
        pwm_duty(MOTOR_LEFT_REVERSE, (uint32_t)(-left_duty));
    }

    if (right_duty >= 0.0f)
    {
        pwm_duty(MOTOR_RIGHT_FORWARD, (uint32_t)right_duty);
        pwm_duty(MOTOR_RIGHT_REVERSE, 0u);
    }
    else
    {
        pwm_duty(MOTOR_RIGHT_FORWARD, 0u);
        pwm_duty(MOTOR_RIGHT_REVERSE, (uint32_t)(-right_duty));
    }
}

/* Motor-track differential control application library implementation. */
#include "6_generated_code.h"
#include <math.h>

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

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
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

void speed_control(void)
{
    float diff;

    motor_l.motor_mode = MODE_NORMAL;
    motor_r.motor_mode = MODE_NORMAL;
    diff = 15.8f * tanf(motor_env.angle * MOTOR_PI * 2.4f / 180.0f) / 40.0f / 2.0f;

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
    } else if (motor_env.yroad_type == YROAD_NEAR) {
        target_speed = 3.0f;
    } else if (motor_env.yroad_type == YROAD_FOUND) {
        target_speed = 3.0f;
    } else if (motor_env.circle_type == CIRCLE_LEFT_BEGIN || motor_env.circle_type == CIRCLE_RIGHT_BEGIN) {
        target_speed = clamp_float(target_speed - 0.02f, 11.0f, 17.0f);
    } else if (motor_env.rptsn_num > 20u) {
        unsigned id = motor_env.rptsn_num - 1u;
        float error;
        float pid_result;

        if (id > 70u) {
            id = 70u;
        }

        error = fabsf((motor_env.rptsn[id][0] - motor_env.rptsn[0][0]) /
                      (motor_env.rptsn[id][1] - motor_env.rptsn[0][1]));
        pid_result = pid_solve(&target_speed_pid, error);
        target_speed = clamp_float(17.0f - pid_result, 9.0f, 17.0f);
    } else if (motor_env.rptsn_num > 5u && motor_env.rptsn_num <= 20u) {
        target_speed = 9.0f;
    } else {
        /* MT-R18: rptsn_num <= 5 and no earlier decision matches. */
    }

    if (motor_env.garage_type == GARAGE_STOP ||
        ((motor_env.garage_type != GARAGE_OUT_LEFT && motor_env.garage_type != GARAGE_OUT_RIGHT) &&
         (motor_env.elec_data[0] + motor_env.elec_data[1]) < 60.0f)) {
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
    float duty_l;
    float duty_r;

    if (motor_l.motor_mode == MODE_NORMAL) {
        duty_l = pid_solve(&motor_pid_l, error_l);
    } else if (motor_l.motor_mode == MODE_BANGBANG) {
        motor_pid_l.out_i = 0.0f;
        duty_l = motor_l.duty + bangbang_pid_solve(&motor_l.brake_pid, error_l);
    } else if (motor_l.motor_mode == MODE_SOFT) {
        motor_pid_l.out_i = 0.0f;
        duty_l = motor_l.duty + changable_pid_solve(&motor_l.pid, error_l);
    } else if (motor_l.motor_mode == MODE_POSLOOP) {
        motor_pid_l.out_i = 0.0f;
        duty_l = pid_solve(&posloop_pid, (float)(motor_l.target_encoder - motor_l.total_encoder));
    } else {
        duty_l = pid_solve(&motor_pid_l, error_l);
    }

    if (motor_r.motor_mode == MODE_NORMAL) {
        duty_r = pid_solve(&motor_pid_r, error_r);
    } else if (motor_r.motor_mode == MODE_BANGBANG) {
        motor_pid_r.out_i = 0.0f;
        duty_r = motor_r.duty + bangbang_pid_solve(&motor_r.brake_pid, error_r);
    } else if (motor_r.motor_mode == MODE_SOFT) {
        motor_pid_r.out_i = 0.0f;
        duty_r = motor_r.duty + changable_pid_solve(&motor_r.pid, error_r);
    } else if (motor_r.motor_mode == MODE_POSLOOP) {
        motor_pid_r.out_i = 0.0f;
        duty_r = pid_solve(&posloop_pid, (float)(motor_r.target_encoder - motor_r.total_encoder));
    } else {
        duty_r = pid_solve(&motor_pid_r, error_r);
    }

    duty_l = clamp_float(duty_l, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
    duty_r = clamp_float(duty_r, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);

    if (fabsf(motor_env.angle) > 10.0f) {
        float encoder_avg = (motor_l.encoder_speed + motor_r.encoder_speed) * 0.5f;
        if (target_speed - encoder_avg < 0.0f) {
            duty_l = clamp_float(duty_l, -MOTOR_PWM_DUTY_MAX, 40000.0f);
            duty_r = clamp_float(duty_r, -MOTOR_PWM_DUTY_MAX, 40000.0f);
        } else {
            duty_l = clamp_float(duty_l, -40000.0f, 40000.0f);
            duty_r = clamp_float(duty_r, -40000.0f, 40000.0f);
        }
    }

    motor_l.duty = duty_l;
    motor_r.duty = duty_r;

    if (duty_l >= 0.0f) {
        pwm_duty(MOTOR_LEFT_FORWARD, (uint32_t)duty_l);
        pwm_duty(MOTOR_LEFT_REVERSE, 0u);
    } else {
        pwm_duty(MOTOR_LEFT_FORWARD, 0u);
        pwm_duty(MOTOR_LEFT_REVERSE, (uint32_t)(-duty_l));
    }

    if (duty_r >= 0.0f) {
        pwm_duty(MOTOR_RIGHT_FORWARD, (uint32_t)duty_r);
        pwm_duty(MOTOR_RIGHT_REVERSE, 0u);
    } else {
        pwm_duty(MOTOR_RIGHT_FORWARD, 0u);
        pwm_duty(MOTOR_RIGHT_REVERSE, (uint32_t)(-duty_r));
    }
}

int64_t get_total_encoder(void)
{
    return (motor_l.total_encoder + motor_r.total_encoder) / 2;
}

float radius_3pts(float pt0[2], float pt1[2], float pt2[2])
{
    double dx10 = (double)pt1[0] - (double)pt0[0];
    double dy10 = (double)pt1[1] - (double)pt0[1];
    double dx20 = (double)pt2[0] - (double)pt0[0];
    double dy20 = (double)pt2[1] - (double)pt0[1];
    double dx21 = (double)pt2[0] - (double)pt1[0];
    double dy21 = (double)pt2[1] - (double)pt1[1];

    double a = sqrt(dx21 * dx21 + dy21 * dy21);
    double b = sqrt(dx20 * dx20 + dy20 * dy20);
    double c = sqrt(dx10 * dx10 + dy10 * dy10);

    double determinant = dx10 * dy20 - dy10 * dx20;
    if (determinant < 0.0) {
        determinant = -determinant;
    }
    double area = determinant / 2.0;

    return (float)((a * b * c) / (4.0 * area));
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
        /* clk == 10000: retain previous two wheel target speeds. */
    }
}

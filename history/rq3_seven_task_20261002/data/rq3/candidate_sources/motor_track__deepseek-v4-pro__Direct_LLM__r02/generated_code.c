#include "generated_code.h"
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
uint32_t clk = 0;

static float clamp_float(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void motor_init(void) {
    pwm_init(MOTOR_LEFT_FORWARD, 17000, 0);
    pwm_init(MOTOR_LEFT_REVERSE, 17000, 0);
    pwm_init(MOTOR_RIGHT_FORWARD, 17000, 0);
    pwm_init(MOTOR_RIGHT_REVERSE, 17000, 0);
}

float radius_3pts(float points[3][2]) {
    float x1 = points[0][0], y1 = points[0][1];
    float x2 = points[1][0], y2 = points[1][1];
    float x3 = points[2][0], y3 = points[2][1];

    float cross = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    float side_ab = sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    float side_bc = sqrtf((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    float side_ca = sqrtf((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3));

    return (side_ab * side_bc * side_ca) / (2.0f * fabsf(cross));
}

int64_t get_total_encoder(void) {
    int64_t sum = motor_l.total_encoder + motor_r.total_encoder;
    return sum / 2;
}

void square_signal(void) {
    clk++;
    if (clk > 10000) {
        clk = 0;
    }

    if (clk < 2000) {
        motor_l.target_speed = 20.0f;
        motor_r.target_speed = 20.0f;
    } else if (clk < 4000) {
        motor_l.target_speed = 0.0f;
        motor_r.target_speed = 0.0f;
    } else if (clk < 6000) {
        motor_l.target_speed = 15.0f;
        motor_r.target_speed = 15.0f;
    } else if (clk < 8000) {
        motor_l.target_speed = 28.0f;
        motor_r.target_speed = 28.0f;
    } else if (clk < 10000) {
        motor_l.target_speed = 0.0f;
        motor_r.target_speed = 0.0f;
    }
}

void speed_control(void) {
    motor_l.motor_mode = MODE_NORMAL;
    motor_r.motor_mode = MODE_NORMAL;

    float diff = 15.8f * tanf(motor_env.angle * MOTOR_PI * 2.4f / 180.0f) / 40.0f / 2.0f;

    uint32_t animal_elapsed = motor_env.now_ms - motor_env.animal_time_ms;

    if (motor_env.fa_type == ANIMAL && animal_elapsed < 2500u) {
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
        target_speed = clamp_float(target_speed - 0.02f, 11.0f, 17.0f);
    } else if (motor_env.rptsn_num > 20) {
        unsigned id = motor_env.rptsn_num - 1;
        if (id > 70) {
            id = 70;
        }
        float error = fabsf((motor_env.rptsn[id][0] - motor_env.rptsn[0][0]) /
                            (motor_env.rptsn[id][1] - motor_env.rptsn[0][1]));
        float returned = pid_solve(&target_speed_pid, error);
        target_speed = clamp_float(17.0f - returned, 9.0f, 17.0f);
    } else if (motor_env.rptsn_num > 5 && motor_env.rptsn_num <= 20) {
        target_speed = 9.0f;
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

static float compute_wheel_duty(motor_param_t *wheel, motor_pid_t *normal_pid,
                                float error, motor_pid_t *position_pid) {
    switch (wheel->motor_mode) {
        case MODE_NORMAL:
            return pid_solve(normal_pid, error);
        case MODE_BANGBANG:
            normal_pid->out_i = 0.0f;
            return wheel->duty + bangbang_pid_solve(&wheel->brake_pid, error);
        case MODE_SOFT:
            normal_pid->out_i = 0.0f;
            return wheel->duty + changable_pid_solve(&wheel->pid, error);
        case MODE_POSLOOP:
            normal_pid->out_i = 0.0f;
            return pid_solve(position_pid,
                             (float)(wheel->target_encoder - wheel->total_encoder));
        default:
            return 0.0f;
    }
}

void motor_control(void) {
    float left_error = motor_l.target_speed - motor_l.encoder_speed;
    float right_error = motor_r.target_speed - motor_r.encoder_speed;

    float left_duty = compute_wheel_duty(&motor_l, &motor_pid_l, left_error, &posloop_pid);
    float right_duty = compute_wheel_duty(&motor_r, &motor_pid_r, right_error, &posloop_pid);

    left_duty = clamp_float(left_duty, -50000.0f, 50000.0f);
    right_duty = clamp_float(right_duty, -50000.0f, 50000.0f);

    if (fabsf(motor_env.angle) > 10.0f) {
        float average_encoder_speed = (motor_l.encoder_speed + motor_r.encoder_speed) / 2.0f;
        if (target_speed - average_encoder_speed < 0.0f) {
            left_duty = clamp_float(left_duty, -50000.0f, 40000.0f);
            right_duty = clamp_float(right_duty, -50000.0f, 40000.0f);
        } else {
            left_duty = clamp_float(left_duty, -40000.0f, 40000.0f);
            right_duty = clamp_float(right_duty, -40000.0f, 40000.0f);
        }
    }

    motor_l.duty = left_duty;
    motor_r.duty = right_duty;

    uint32_t left_forward = 0;
    uint32_t left_reverse = 0;
    uint32_t right_forward = 0;
    uint32_t right_reverse = 0;

    if (left_duty >= 0.0f) {
        left_forward = (uint32_t)left_duty;
    } else {
        left_reverse = (uint32_t)(-left_duty);
    }

    if (right_duty >= 0.0f) {
        right_forward = (uint32_t)right_duty;
    } else {
        right_reverse = (uint32_t)(-right_duty);
    }

    pwm_duty(MOTOR_LEFT_FORWARD, left_forward);
    pwm_duty(MOTOR_LEFT_REVERSE, left_reverse);
    pwm_duty(MOTOR_RIGHT_FORWARD, right_forward);
    pwm_duty(MOTOR_RIGHT_REVERSE, right_reverse);
}

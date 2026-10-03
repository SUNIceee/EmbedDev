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

static float clamp_float(float value, float lo, float hi) {
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

int64_t get_total_encoder(void) {
    int64_t sum = motor_l.total_encoder + motor_r.total_encoder;
    return sum / 2;
}

float radius_3pts(float pt0[2], float pt1[2], float pt2[2]) {
    double x0 = (double)pt0[0], y0 = (double)pt0[1];
    double x1 = (double)pt1[0], y1 = (double)pt1[1];
    double x2 = (double)pt2[0], y2 = (double)pt2[1];

    double dx1 = x1 - x0;
    double dy1 = y1 - y0;
    double dx2 = x2 - x0;
    double dy2 = y2 - y0;

    double det = dx1 * dy2 - dy1 * dx2;
    double area = 0.5 * fabs(det);

    double a = sqrt(dx1 * dx1 + dy1 * dy1);
    double b = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    double c = sqrt(dx2 * dx2 + dy2 * dy2);

    if (area < 1e-12) return 0.0f;

    double radius = (a * b * c) / (4.0 * area);
    return (float)radius;
}

void square_signal(void) {
    ++clk;
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
        /* clk == 10000: retain previous wheel target speeds. */
    }
}

void speed_control(void) {
    motor_l.motor_mode = MODE_NORMAL;
    motor_r.motor_mode = MODE_NORMAL;

    float angle_rad = motor_env.angle * MOTOR_PI * 2.4f / 180.0f;
    float diff = 15.8f * tanf(angle_rad) / 40.0f / 2.0f;

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
    } else if (motor_env.rptsn_num > 20) {
        unsigned id = motor_env.rptsn_num - 1;
        if (id > 70) {
            id = 70;
        }

        float denominator = motor_env.rptsn[id][1] - motor_env.rptsn[0][1];
        float error = fabsf((motor_env.rptsn[id][0] - motor_env.rptsn[0][0]) / denominator);
        float val = pid_solve(&target_speed_pid, error);
        target_speed = clamp_float(17.0f - val, 9.0f, 17.0f);
    } else if (motor_env.rptsn_num > 5 && motor_env.rptsn_num <= 20) {
        target_speed = 9.0f;
    } else {
        /* rptsn_num <= 5 and no other decision matched: retain previous scalar target_speed. */
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

static void motor_control_one_wheel(motor_param_t *wheel, motor_pid_t *normal_pid) {
    float error = wheel->target_speed - wheel->encoder_speed;

    switch (wheel->motor_mode) {
    case MODE_NORMAL:
        wheel->duty = pid_solve(normal_pid, error);
        break;
    case MODE_BANGBANG:
        normal_pid->out_i = 0.0f;
        wheel->duty += bangbang_pid_solve(&wheel->brake_pid, error);
        break;
    case MODE_SOFT:
        normal_pid->out_i = 0.0f;
        wheel->duty += changable_pid_solve(&wheel->pid, error);
        break;
    case MODE_POSLOOP:
        normal_pid->out_i = 0.0f;
        wheel->duty = pid_solve(&posloop_pid,
                                (float)(wheel->target_encoder - wheel->total_encoder));
        break;
    default:
        wheel->duty = pid_solve(normal_pid, error);
        break;
    }

    wheel->duty = clamp_float(wheel->duty, -50000.0f, 50000.0f);
}

void motor_control(void) {
    motor_control_one_wheel(&motor_l, &motor_pid_l);
    motor_control_one_wheel(&motor_r, &motor_pid_r);

    if (fabsf(motor_env.angle) > 10.0f) {
        float average_encoder = (motor_l.encoder_speed + motor_r.encoder_speed) / 2.0f;

        if (target_speed - average_encoder < 0.0f) {
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

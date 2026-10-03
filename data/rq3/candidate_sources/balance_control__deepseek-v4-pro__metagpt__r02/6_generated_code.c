/* 6_generated_code.c - Bounded C11 balance control implementation.
 *
 * This file implements the public control API declared in
 * 6_generated_code.h. All hardware access is delegated to board_* callbacks
 * and all private loop state is kept in file-scope static variables.
 */

#include <stdint.h>
#include <stdbool.h>

#include "6_generated_code.h"

/* Public sensor and output globals. */
float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;
int16_t gyrox = 0;
int16_t gyroy = 0;
int16_t gyroz = 0;
int32_t speed = 0;
int16_t direction_error = 0;
int32_t ac_pwm = 0;
int32_t vc_pwm = 0;
int32_t dc_pwm = 0;
int32_t left_pwm = 0;
int32_t right_pwm = 0;

/*----------------------------------------------------------------------------
 * Private control state.
 *----------------------------------------------------------------------------*/
static struct {
    float e1;
    float e2;
    int32_t accumulator;
    int32_t old_endpoint;
    int32_t new_endpoint;
    int32_t counter;
} s_velocity_state;

static struct {
    int16_t previous_filtered;
    int32_t old_endpoint;
    int32_t new_endpoint;
    int32_t counter;
} s_direction_state;

/*----------------------------------------------------------------------------
 * Private helpers.
 *----------------------------------------------------------------------------*/
static int32_t prv_abs_i32(int32_t value) {
    return value < 0 ? -value : value;
}

static int32_t prv_clamp_i32(int32_t value, int32_t min, int32_t max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static float prv_clamp_f32(float value, float min, float max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static void prv_reset_velocity_state(void) {
    s_velocity_state.e1 = 0.0f;
    s_velocity_state.e2 = 0.0f;
    s_velocity_state.accumulator = 0;
    s_velocity_state.old_endpoint = 0;
    s_velocity_state.new_endpoint = 0;
    s_velocity_state.counter = 0;
}

static void prv_reset_direction_state(void) {
    s_direction_state.previous_filtered = 0;
    s_direction_state.old_endpoint = 0;
    s_direction_state.new_endpoint = 0;
    s_direction_state.counter = 0;
}

/*----------------------------------------------------------------------------
 * Public API implementation.
 *----------------------------------------------------------------------------*/
void control_init(void) {
    pitch = 0.0f;
    roll = 0.0f;
    yaw = 0.0f;
    gyrox = 0;
    gyroy = 0;
    gyroz = 0;
    speed = 0;
    direction_error = 0;
    ac_pwm = 0;
    vc_pwm = 0;
    dc_pwm = 0;
    left_pwm = 0;
    right_pwm = 0;

    prv_reset_velocity_state();
    prv_reset_direction_state();
}

void get_mpu(void) {
    board_read_imu(&pitch, &roll, &yaw, &gyrox, &gyroy, &gyroz);
}

int32_t get_speed(void) {
    return speed;
}

int32_t angle_proc(void) {
    /* Angle controller uses pitch angle and pitch rate only. */
    return (int32_t)((BALANCE_ANGLE_KP * pitch) +
                     (BALANCE_ANGLE_KD * gyroy));
}

void get_pwm(void) {
    int32_t left_count;
    int32_t right_count;

    board_read_imu(&pitch, &roll, &yaw, &gyrox, &gyroy, &gyroz);

    left_count = (int32_t)board_encoder_read(0);
    right_count = (int32_t)board_encoder_read(1);

    speed = (prv_abs_i32(left_count) + prv_abs_i32(right_count)) / 2;

    board_encoder_clear(0);
    board_encoder_clear(1);

    ac_pwm = angle_proc();
    vc_pwm = velocity_proc(speed);
    dc_pwm = direction_proc(speed);

    left_pwm = ac_pwm - vc_pwm + dc_pwm;
    right_pwm = ac_pwm - vc_pwm - dc_pwm;
}

int32_t velocity_proc(int32_t measured_speed) {
    if (s_velocity_state.counter >= 5) {
        const float e = 0.0f - (float)measured_speed;
        const float p = VELOCITY_KP * (e - s_velocity_state.e1);
        float i = VELOCITY_KI * e;
        const float d = VELOCITY_KD *
                        (e - (2.0f * s_velocity_state.e1) +
                         s_velocity_state.e2);

        s_velocity_state.counter = 0;
        s_velocity_state.old_endpoint = s_velocity_state.new_endpoint;

        if (i > VELOCITY_INTEGRAL_WINDUP ||
            i < -VELOCITY_INTEGRAL_WINDUP) {
            i = 0.0f;
        }

        /* Float-to-int32 cast truncates toward zero before clamping. */
        s_velocity_state.accumulator =
            (int32_t)((float)s_velocity_state.accumulator + p + i + d);
        s_velocity_state.accumulator =
            prv_clamp_i32(s_velocity_state.accumulator,
                          VELOCITY_ACCUMULATOR_MIN,
                          VELOCITY_ACCUMULATOR_MAX);
        s_velocity_state.new_endpoint = s_velocity_state.accumulator;

        s_velocity_state.e2 = s_velocity_state.e1;
        s_velocity_state.e1 = e;
    }

    s_velocity_state.counter += 1;

    return s_velocity_state.old_endpoint +
           ((s_velocity_state.new_endpoint - s_velocity_state.old_endpoint) *
            s_velocity_state.counter) /
               5;
}

int32_t direction_proc(int32_t measured_speed) {
    const float filtered = (0.9f * (float)gyroz) +
                           (0.1f * (float)s_direction_state.previous_filtered);

    gyroz = (int16_t)filtered;
    s_direction_state.previous_filtered = gyroz;

    if (s_direction_state.counter >= 5) {
        const int16_t state = board_direction_error();
        const float pgain_unclamped =
            DIRECTION_P_GAIN_SCALE * (float)measured_speed;
        const float pgain = prv_clamp_f32(pgain_unclamped,
                                          DIRECTION_P_GAIN_MIN,
                                          DIRECTION_P_GAIN_MAX);
        const int32_t pterm = (int32_t)((pgain * (float)state) / 25.0f);
        const int32_t dterm = (int32_t)((1.5f * (float)gyroz) / 100.0f);

        s_direction_state.counter = 0;
        s_direction_state.old_endpoint = s_direction_state.new_endpoint;
        direction_error = state;

        s_direction_state.new_endpoint = pterm + dterm;
        s_direction_state.new_endpoint =
            prv_clamp_i32(s_direction_state.new_endpoint,
                          DIRECTION_ENDPOINT_MIN,
                          DIRECTION_ENDPOINT_MAX);
    }

    s_direction_state.counter += 1;

    return s_direction_state.old_endpoint +
           ((s_direction_state.new_endpoint - s_direction_state.old_endpoint) *
            s_direction_state.counter) /
               5;
}

void motor_proc(int32_t left, int32_t right) {
    int32_t left_clamped = prv_clamp_i32(left, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    int32_t right_clamped =
        prv_clamp_i32(right, MOTOR_PWM_MIN, MOTOR_PWM_MAX);

    if (left_clamped >= 0) {
        board_motor_write(0, false, true, (uint16_t)left_clamped);
    } else {
        board_motor_write(0, true, false, (uint16_t)(-left_clamped));
    }

    if (right_clamped >= 0) {
        board_motor_write(1, false, true, (uint16_t)right_clamped);
    } else {
        board_motor_write(1, true, false, (uint16_t)(-right_clamped));
    }
}

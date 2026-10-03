/* Implementation of the three-loop balance-control library public API. */
#include "6_generated_code.h"

/* Private velocity-loop state. */
static float vc_e1;
static float vc_e2;
static int32_t vc_acc;
static int32_t vc_old;
static int32_t vc_new;
static uint32_t vc_count;

/* Private direction-loop state. */
static int16_t dc_prev_filtered;
static int32_t dc_old;
static int32_t dc_new;
static uint32_t dc_count;

/* Public global definitions. */
float pitch, roll, yaw;
int16_t gyrox, gyroy, gyroz, state;
int32_t speed, ac_pwm, vc_pwm, dc_pwm, left_pwm, right_pwm;

static int32_t clamp_i32(int32_t value, int32_t min_val, int32_t max_val) {
    if (value > max_val) {
        return max_val;
    }
    if (value < min_val) {
        return min_val;
    }
    return value;
}

static float clamp_f32(float value, float min_val, float max_val) {
    if (value > max_val) {
        return max_val;
    }
    if (value < min_val) {
        return min_val;
    }
    return value;
}

void control_init(void) {
    pitch = 0.0f;
    roll = 0.0f;
    yaw = 0.0f;
    gyrox = 0;
    gyroy = 0;
    gyroz = 0;
    state = 0;
    speed = 0;
    ac_pwm = 0;
    vc_pwm = 0;
    dc_pwm = 0;
    left_pwm = 0;
    right_pwm = 0;

    vc_e1 = 0.0f;
    vc_e2 = 0.0f;
    vc_acc = 0;
    vc_old = 0;
    vc_new = 0;
    vc_count = 0;

    dc_prev_filtered = 0;
    dc_old = 0;
    dc_new = 0;
    dc_count = 0;
}

void get_mpu(void) {
    board_read_imu(&pitch, &roll, &yaw, &gyrox, &gyroy, &gyroz);
}

int get_speed(void) {
    int32_t left_count = (int32_t)board_encoder_read(0);
    int32_t right_count = (int32_t)board_encoder_read(1);

    board_encoder_clear(0);
    board_encoder_clear(1);

    int32_t abs_left = (left_count < 0) ? -left_count : left_count;
    int32_t abs_right = (right_count < 0) ? -right_count : right_count;

    return (int)((abs_left + abs_right) / 2);
}

int32_t angle_proc(void) {
    return -(int32_t)(BC_AC_KP * (BC_AC_SET - pitch) - BC_AC_KD * (float)gyroy);
}

static void velocity_update(int32_t measured_speed) {
    float e = (float)BC_VC_SET - (float)measured_speed;
    float p = BC_VC_KP * (e - vc_e1);
    float i = BC_VC_KI * e;
    float d = BC_VC_KD * (e - 2.0f * vc_e1 + vc_e2);

    if (i > 10.0f || i < -10.0f) {
        i = 0.0f;
    }

    float next_acc_f = (float)vc_acc + p + i + d;
    int32_t next_acc = (int32_t)next_acc_f;
    next_acc = clamp_i32(next_acc, -BC_VC_LIMIT, BC_VC_LIMIT);

    vc_acc = next_acc;
    vc_e2 = vc_e1;
    vc_e1 = e;
    vc_new = vc_acc;
}

int32_t velocity_proc(int32_t measured_speed) {
    if (vc_count >= BC_VC_PERIOD) {
        vc_count = 0;
        vc_old = vc_new;
        velocity_update(measured_speed);
    }

    vc_count++;
    return vc_old + ((vc_new - vc_old) * (int32_t)vc_count / (int32_t)BC_VC_PERIOD);
}

static void direction_update(int32_t measured_speed) {
    state = board_direction_error();

    float pgain = BC_DC_COEF * (float)measured_speed;
    pgain = clamp_f32(pgain, BC_DC_P_MIN, BC_DC_P_MAX);

    int32_t pterm = (int32_t)(pgain * (float)state / 25.0f);
    int32_t dterm = (int32_t)(BC_DC_KD * (float)gyroz / 100.0f);
    int32_t sum = clamp_i32(pterm + dterm, -BC_DC_LIMIT, BC_DC_LIMIT);

    dc_new = sum;
}

int32_t direction_proc(int32_t measured_speed) {
    float filtered_f = 0.9f * (float)gyroz + 0.1f * (float)dc_prev_filtered;
    int16_t filtered = (int16_t)filtered_f;
    gyroz = filtered;
    dc_prev_filtered = filtered;

    if (dc_count >= BC_DC_PERIOD) {
        dc_count = 0;
        dc_old = dc_new;
        direction_update(measured_speed);
    }

    dc_count++;
    return dc_old + ((dc_new - dc_old) * (int32_t)dc_count / (int32_t)BC_DC_PERIOD);
}

void get_pwm(void) {
    get_mpu();

    speed = (int32_t)get_speed();

    ac_pwm = angle_proc();
    vc_pwm = velocity_proc(speed);
    dc_pwm = direction_proc(speed);

    left_pwm = ac_pwm - vc_pwm + dc_pwm;
    right_pwm = ac_pwm - vc_pwm - dc_pwm;
}

void motor_proc(int32_t left, int32_t right) {
    int32_t clamped_left = clamp_i32(left, -BC_MOTOR_LIMIT, BC_MOTOR_LIMIT);
    int32_t clamped_right = clamp_i32(right, -BC_MOTOR_LIMIT, BC_MOTOR_LIMIT);

    bool in_a;
    bool in_b;
    uint16_t duty;

    if (clamped_left >= 0) {
        in_a = false;
        in_b = true;
        duty = (uint16_t)clamped_left;
    } else {
        in_a = true;
        in_b = false;
        duty = (uint16_t)(-clamped_left);
    }
    board_motor_write(0, in_a, in_b, duty);

    if (clamped_right >= 0) {
        in_a = false;
        in_b = true;
        duty = (uint16_t)clamped_right;
    } else {
        in_a = true;
        in_b = false;
        duty = (uint16_t)(-clamped_right);
    }
    board_motor_write(1, in_a, in_b, duty);
}

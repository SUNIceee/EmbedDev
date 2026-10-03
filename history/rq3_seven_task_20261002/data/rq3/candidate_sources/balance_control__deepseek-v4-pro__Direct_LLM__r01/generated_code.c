#include "generated_code.h"

float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;
int16_t gyrox = 0;
int16_t gyroy = 0;
int16_t gyroz = 0;
int16_t direction_error = 0;
int32_t speed = 0;
int32_t ac_pwm = 0;
int32_t vc_pwm = 0;
int32_t dc_pwm = 0;
int32_t left_pwm = 0;
int32_t right_pwm = 0;

static float e1 = 0.0f;
static float e2 = 0.0f;
static int32_t velocity_accumulator = 0;
static int32_t velocity_old = 0;
static int32_t velocity_new = 0;
static int32_t velocity_count = 0;

static int16_t direction_prev_filtered = 0;
static int32_t direction_old = 0;
static int32_t direction_new = 0;
static int32_t direction_count = 0;

void control_init(void) {
    pitch = 0.0f;
    roll = 0.0f;
    yaw = 0.0f;
    gyrox = 0;
    gyroy = 0;
    gyroz = 0;
    direction_error = 0;
    speed = 0;
    ac_pwm = 0;
    vc_pwm = 0;
    dc_pwm = 0;
    left_pwm = 0;
    right_pwm = 0;

    e1 = 0.0f;
    e2 = 0.0f;
    velocity_accumulator = 0;
    velocity_old = 0;
    velocity_new = 0;
    velocity_count = 0;

    direction_prev_filtered = 0;
    direction_old = 0;
    direction_new = 0;
    direction_count = 0;
}

void get_mpu(void) {
    float p = 0.0f;
    float r = 0.0f;
    float y = 0.0f;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;

    board_read_imu(&p, &r, &y, &gx, &gy, &gz);

    pitch = p;
    roll = r;
    yaw = y;
    gyrox = gx;
    gyroy = gy;
    gyroz = gz;
}

int32_t get_speed(void) {
    int16_t left_count = board_encoder_read(0);
    int16_t right_count = board_encoder_read(1);
    int32_t left_abs;
    int32_t right_abs;

    board_encoder_clear(0);
    board_encoder_clear(1);

    left_abs = (left_count < 0) ? (-(int32_t)left_count) : (int32_t)left_count;
    right_abs = (right_count < 0) ? (-(int32_t)right_count) : (int32_t)right_count;

    return (left_abs + right_abs) / 2;
}

int32_t angle_proc(void) {
    return -(int32_t)(550.0f * (0.0f - pitch) - 2.0f * (float)gyroy);
}

int32_t velocity_proc(int32_t measured_speed) {
    float e = 0.0f - (float)measured_speed;
    int32_t result;

    if (velocity_count >= 5) {
        float p_term;
        float i_term;
        float d_term;
        float new_accumulator_float;
        int32_t new_accumulator;

        velocity_count = 0;
        velocity_old = velocity_new;

        p_term = 8.0f * (e - e1);
        i_term = 0.1f * e;
        d_term = 0.5f * (e - 2.0f * e1 + e2);

        if (i_term > 10.0f || i_term < -10.0f) {
            i_term = 0.0f;
        }

        new_accumulator_float = (float)velocity_accumulator + p_term + i_term + d_term;
        new_accumulator = (int32_t)new_accumulator_float;

        if (new_accumulator > 4500) {
            new_accumulator = 4500;
        } else if (new_accumulator < -4500) {
            new_accumulator = -4500;
        }

        velocity_accumulator = new_accumulator;
        velocity_new = new_accumulator;

        e2 = e1;
        e1 = e;
    }

    velocity_count++;
    result = velocity_old + (velocity_new - velocity_old) * velocity_count / 5;
    return result;
}

int32_t direction_proc(int32_t measured_speed) {
    int16_t filtered;
    int32_t result;

    filtered = (int16_t)(0.9f * (float)gyroz + 0.1f * (float)direction_prev_filtered);
    gyroz = filtered;
    direction_prev_filtered = filtered;

    if (direction_count >= 5) {
        int16_t state;
        float p_gain;
        int32_t p_term;
        int32_t d_term;
        int32_t endp;

        direction_count = 0;
        direction_old = direction_new;

        state = board_direction_error();
        direction_error = state;

        p_gain = 0.6f * (float)measured_speed;
        if (p_gain > 500.0f) {
            p_gain = 500.0f;
        } else if (p_gain < 50.0f) {
            p_gain = 50.0f;
        }

        p_term = (int32_t)(p_gain * (float)state / 25.0f);
        d_term = (int32_t)(1.5f * (float)gyroz / 100.0f);

        endp = p_term + d_term;
        if (endp > 3000) {
            endp = 3000;
        } else if (endp < -3000) {
            endp = -3000;
        }

        direction_new = endp;
    }

    direction_count++;
    result = direction_old + (direction_new - direction_old) * direction_count / 5;
    return result;
}

void get_pwm(void) {
    int32_t measured_speed;

    get_mpu();
    measured_speed = get_speed();
    speed = measured_speed;

    ac_pwm = angle_proc();
    vc_pwm = velocity_proc(measured_speed);
    dc_pwm = direction_proc(measured_speed);

    left_pwm = ac_pwm - vc_pwm + dc_pwm;
    right_pwm = ac_pwm - vc_pwm - dc_pwm;
}

void motor_proc(int32_t left, int32_t right) {
    int32_t left_clamped = left;
    int32_t right_clamped = right;

    if (left_clamped > 7200) {
        left_clamped = 7200;
    } else if (left_clamped < -7200) {
        left_clamped = -7200;
    }

    if (right_clamped > 7200) {
        right_clamped = 7200;
    } else if (right_clamped < -7200) {
        right_clamped = -7200;
    }

    if (left_clamped >= 0) {
        board_motor_write(0, false, true, left_clamped);
    } else {
        board_motor_write(0, true, false, -left_clamped);
    }

    if (right_clamped >= 0) {
        board_motor_write(1, false, true, right_clamped);
    } else {
        board_motor_write(1, true, false, -right_clamped);
    }
}

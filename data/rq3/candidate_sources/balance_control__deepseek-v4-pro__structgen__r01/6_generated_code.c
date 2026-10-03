#include "6_generated_code.h"

float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
int16_t gyrox = 0, gyroy = 0, gyroz = 0, state = 0;
int32_t speed = 0, ac_pwm = 0, vc_pwm = 0, dc_pwm = 0, left_pwm = 0, right_pwm = 0;

static float v_e1 = 0.0f;
static float v_e2 = 0.0f;
static int32_t v_acc = 0;
static int32_t v_old = 0;
static int32_t v_new = 0;
static int32_t v_count = 0;

static int16_t d_prev_filtered = 0;
static int32_t d_old = 0;
static int32_t d_new = 0;
static int32_t d_count = 0;

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float clamp_f(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
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

    v_e1 = 0.0f;
    v_e2 = 0.0f;
    v_acc = 0;
    v_old = 0;
    v_new = 0;
    v_count = 0;

    d_prev_filtered = 0;
    d_old = 0;
    d_new = 0;
    d_count = 0;
}

void get_mpu(void) {
    float p, r, y;
    int16_t gx, gy, gz;

    board_read_imu(&p, &r, &y, &gx, &gy, &gz);

    pitch = p;
    roll = r;
    yaw = y;
    gyrox = gx;
    gyroy = gy;
    gyroz = gz;
}

int get_speed(void) {
    int32_t l = (int32_t)board_encoder_read(0);
    int32_t r = (int32_t)board_encoder_read(1);

    board_encoder_clear(0);
    board_encoder_clear(1);

    if (l < 0) l = -l;
    if (r < 0) r = -r;

    return (int)((l + r) / 2);
}

int32_t angle_proc(void) {
    float val = (BC_AC_KP * (BC_AC_SET - pitch)) - (BC_AC_KD * (float)gyroy);
    return -(int32_t)val;
}

int32_t velocity_proc(int32_t measured_speed) {
    if (v_count >= (int32_t)BC_VC_PERIOD) {
        v_count = 0;
        v_old = v_new;

        float e = 0.0f - (float)measured_speed;
        float P = BC_VC_KP * (e - v_e1);
        float I = BC_VC_KI * e;
        float D = BC_VC_KD * (e - 2.0f * v_e1 + v_e2);

        if (I > 10.0f || I < -10.0f) {
            I = 0.0f;
        }

        float acc_f = (float)v_acc + P + I + D;
        int32_t raw = (int32_t)acc_f;
        raw = clamp_i32(raw, -BC_VC_LIMIT, BC_VC_LIMIT);

        v_acc = raw;
        v_new = v_acc;

        v_e2 = v_e1;
        v_e1 = e;
    }

    v_count++;
    return v_old + (v_new - v_old) * v_count / (int32_t)BC_VC_PERIOD;
}

int32_t direction_proc(int32_t measured_speed) {
    float filtered_f = 0.9f * (float)gyroz + 0.1f * (float)d_prev_filtered;
    int16_t filtered = (int16_t)filtered_f;

    gyroz = filtered;
    d_prev_filtered = filtered;

    if (d_count >= (int32_t)BC_DC_PERIOD) {
        d_count = 0;
        d_old = d_new;

        state = board_direction_error();

        float pgain = clamp_f(BC_DC_COEF * (float)measured_speed, BC_DC_P_MIN, BC_DC_P_MAX);
        int32_t pterm = (int32_t)(pgain * (float)state / 25.0f);
        int32_t dterm = (int32_t)(BC_DC_KD * (float)gyroz / 100.0f);
        int32_t sum = pterm + dterm;

        d_new = clamp_i32(sum, -BC_DC_LIMIT, BC_DC_LIMIT);
    }

    d_count++;
    return d_old + (d_new - d_old) * d_count / (int32_t)BC_DC_PERIOD;
}

void get_pwm(void) {
    get_mpu();

    int32_t measured = (int32_t)get_speed();
    speed = measured;

    ac_pwm = angle_proc();
    vc_pwm = velocity_proc(measured);
    dc_pwm = direction_proc(measured);

    left_pwm = ac_pwm - vc_pwm + dc_pwm;
    right_pwm = ac_pwm - vc_pwm - dc_pwm;
}

void motor_proc(int32_t left, int32_t right) {
    int32_t l = left;
    int32_t r = right;

    if (l > BC_MOTOR_LIMIT) {
        l = BC_MOTOR_LIMIT;
    } else if (l < -BC_MOTOR_LIMIT) {
        l = -BC_MOTOR_LIMIT;
    }

    if (r > BC_MOTOR_LIMIT) {
        r = BC_MOTOR_LIMIT;
    } else if (r < -BC_MOTOR_LIMIT) {
        r = -BC_MOTOR_LIMIT;
    }

    if (l >= 0) {
        board_motor_write(0, false, true, (uint16_t)l);
    } else {
        board_motor_write(0, true, false, (uint16_t)(-l));
    }

    if (r >= 0) {
        board_motor_write(1, false, true, (uint16_t)r);
    } else {
        board_motor_write(1, true, false, (uint16_t)(-r));
    }
}

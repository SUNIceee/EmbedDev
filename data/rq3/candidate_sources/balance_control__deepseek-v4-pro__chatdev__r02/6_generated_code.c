/* Implementation of the fixed three-loop balance control API. */

#include "6_generated_code.h"

/* Public sensor and output globals. */
float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;
int16_t gyrox = 0;
int16_t gyroy = 0;
int16_t gyroz = 0;
int16_t state = 0;
int32_t speed = 0;
int32_t ac_pwm = 0;
int32_t vc_pwm = 0;
int32_t dc_pwm = 0;
int32_t left_pwm = 0;
int32_t right_pwm = 0;

/* Private velocity-loop state. */
static float v_e1 = 0.0f;
static float v_e2 = 0.0f;
static int32_t v_acc = 0;
static int32_t v_old = 0;
static int32_t v_new = 0;
static uint8_t v_count = 0;

/* Private direction-loop state. */
static int16_t d_prev_filtered = 0;
static int32_t d_old = 0;
static int32_t d_new = 0;
static uint8_t d_count = 0;

static int32_t clamp_i32(int32_t value, int32_t lo, int32_t hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static float clamp_f32(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

void control_init(void)
{
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

void get_mpu(void)
{
    float p;
    float r;
    float y;
    int16_t gx;
    int16_t gy;
    int16_t gz;

    board_read_imu(&p, &r, &y, &gx, &gy, &gz);
    pitch = p;
    roll = r;
    yaw = y;
    gyrox = gx;
    gyroy = gy;
    gyroz = gz;
}

int get_speed(void)
{
    int32_t left = (int32_t)board_encoder_read(0);
    int32_t right = (int32_t)board_encoder_read(1);

    board_encoder_clear(0);
    board_encoder_clear(1);

    if (left < 0) {
        left = -left;
    }
    if (right < 0) {
        right = -right;
    }

    return (int)((left + right) / 2);
}

int32_t angle_proc(void)
{
    return -(int32_t)(BC_AC_KP * (BC_AC_SET - pitch) - BC_AC_KD * (float)gyroy);
}

int32_t velocity_proc(int32_t measured_speed)
{
    if ((int32_t)v_count >= (int32_t)BC_VC_PERIOD) {
        float e;
        float p;
        float i;
        float d;
        float acc_f;
        int32_t new_acc;

        v_count = 0;
        v_old = v_new;

        e = (float)BC_VC_SET - (float)measured_speed;
        p = BC_VC_KP * (e - v_e1);
        i = BC_VC_KI * e;
        d = BC_VC_KD * (e - 2.0f * v_e1 + v_e2);

        if (i > 10.0f || i < -10.0f) {
            i = 0.0f;
        }

        acc_f = (float)v_acc + p + i + d;
        new_acc = (int32_t)acc_f;
        v_acc = clamp_i32(new_acc, -BC_VC_LIMIT, BC_VC_LIMIT);
        v_new = v_acc;

        v_e2 = v_e1;
        v_e1 = e;
    }

    v_count++;
    return v_old + (v_new - v_old) * (int32_t)v_count / (int32_t)BC_VC_PERIOD;
}

int32_t direction_proc(int32_t measured_speed)
{
    float filt;
    int16_t filtered;

    filt = 0.9f * (float)gyroz + 0.1f * (float)d_prev_filtered;
    filtered = (int16_t)filt;
    gyroz = filtered;
    d_prev_filtered = filtered;

    if ((int32_t)d_count >= (int32_t)BC_DC_PERIOD) {
        float pgain;
        int32_t pterm;
        int32_t dterm;
        int32_t sum;

        d_count = 0;
        d_old = d_new;

        state = board_direction_error();

        pgain = clamp_f32(BC_DC_COEF * (float)measured_speed,
                          BC_DC_P_MIN, BC_DC_P_MAX);
        pterm = (int32_t)(pgain * (float)state / 25.0f);
        dterm = (int32_t)(BC_DC_KD * (float)gyroz / 100.0f);
        sum = pterm + dterm;
        d_new = clamp_i32(sum, -BC_DC_LIMIT, BC_DC_LIMIT);
    }

    d_count++;
    return d_old + (d_new - d_old) * (int32_t)d_count / (int32_t)BC_DC_PERIOD;
}

void get_pwm(void)
{
    get_mpu();

    speed = get_speed();

    ac_pwm = angle_proc();
    vc_pwm = velocity_proc(speed);
    dc_pwm = direction_proc(speed);

    left_pwm = ac_pwm - vc_pwm + dc_pwm;
    right_pwm = ac_pwm - vc_pwm - dc_pwm;
}

void motor_proc(int32_t left, int32_t right)
{
    int32_t cl = clamp_i32(left, -BC_MOTOR_LIMIT, BC_MOTOR_LIMIT);
    int32_t cr = clamp_i32(right, -BC_MOTOR_LIMIT, BC_MOTOR_LIMIT);

    bool la;
    bool lb;
    bool ra;
    bool rb;
    uint16_t lduty;
    uint16_t rduty;

    if (cl >= 0) {
        la = false;
        lb = true;
        lduty = (uint16_t)cl;
    } else {
        la = true;
        lb = false;
        lduty = (uint16_t)(-cl);
    }

    if (cr >= 0) {
        ra = false;
        rb = true;
        rduty = (uint16_t)cr;
    } else {
        ra = true;
        rb = false;
        rduty = (uint16_t)(-cr);
    }

    board_motor_write(0, la, lb, lduty);
    board_motor_write(1, ra, rb, rduty);
}

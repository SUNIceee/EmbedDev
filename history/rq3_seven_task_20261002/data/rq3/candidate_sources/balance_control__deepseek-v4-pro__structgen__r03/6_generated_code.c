#include "6_generated_code.h"

static int32_t clamp_i32(int32_t value, int32_t low, int32_t high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float clamp_f(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

/* Public global definitions */
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

/* Private velocity loop state */
static float v_e1 = 0.0f;
static float v_e2 = 0.0f;
static int32_t v_acc = 0;
static int32_t v_old = 0;
static int32_t v_new = 0;
static unsigned int v_count = 0;

/* Private direction loop state */
static int16_t d_prev_filtered = 0;
static int32_t d_old = 0;
static int32_t d_new = 0;
static unsigned int d_count = 0;

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
    board_read_imu(&pitch, &roll, &yaw, &gyrox, &gyroy, &gyroz);
}

int get_speed(void)
{
    int32_t left_count = (int32_t)board_encoder_read(0);
    int32_t right_count = (int32_t)board_encoder_read(1);

    board_encoder_clear(0);
    board_encoder_clear(1);

    if (left_count < 0)
        left_count = -left_count;
    if (right_count < 0)
        right_count = -right_count;

    return (int)((left_count + right_count) / 2);
}

int32_t angle_proc(void)
{
    return -(int32_t)(BC_AC_KP * (BC_AC_SET - pitch) -
                      BC_AC_KD * (float)gyroy);
}

int32_t velocity_proc(int32_t measured_speed)
{
    if (v_count >= BC_VC_PERIOD)
    {
        v_count = 0;
        v_old = v_new;

        float e = 0.0f - (float)measured_speed;
        float p = BC_VC_KP * (e - v_e1);
        float i = BC_VC_KI * e;
        float d = BC_VC_KD * (e - 2.0f * v_e1 + v_e2);

        if (i > 10.0f || i < -10.0f)
        {
            i = 0.0f;
        }

        float acc_f = (float)v_acc + p + i + d;
        int32_t raw_acc = (int32_t)acc_f;
        raw_acc = clamp_i32(raw_acc, -BC_VC_LIMIT, BC_VC_LIMIT);

        v_acc = raw_acc;
        v_e2 = v_e1;
        v_e1 = e;
        v_new = v_acc;
    }

    v_count++;
    return v_old + ((v_new - v_old) * (int32_t)v_count) /
                       (int32_t)BC_VC_PERIOD;
}

int32_t direction_proc(int32_t measured_speed)
{
    int16_t filtered = (int16_t)(0.9f * (float)gyroz +
                                 0.1f * (float)d_prev_filtered);
    gyroz = filtered;
    d_prev_filtered = filtered;

    if (d_count >= BC_DC_PERIOD)
    {
        d_count = 0;
        d_old = d_new;

        state = board_direction_error();

        float pgain = clamp_f(BC_DC_COEF * (float)measured_speed,
                              BC_DC_P_MIN, BC_DC_P_MAX);
        int32_t pterm = (int32_t)(pgain * (float)state / 25.0f);
        int32_t dterm = (int32_t)(BC_DC_KD * (float)gyroz / 100.0f);

        int32_t endpoint = clamp_i32(pterm + dterm,
                                     -BC_DC_LIMIT, BC_DC_LIMIT);
        d_new = endpoint;
    }

    d_count++;
    return d_old + ((d_new - d_old) * (int32_t)d_count) /
                       (int32_t)BC_DC_PERIOD;
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
    int32_t clamped_left = left;
    if (clamped_left > BC_MOTOR_LIMIT)
        clamped_left = BC_MOTOR_LIMIT;
    else if (clamped_left < -BC_MOTOR_LIMIT)
        clamped_left = -BC_MOTOR_LIMIT;

    int32_t left_duty = (clamped_left >= 0) ? clamped_left : -clamped_left;
    if (clamped_left >= 0)
        board_motor_write(0, false, true, (uint16_t)left_duty);
    else
        board_motor_write(0, true, false, (uint16_t)left_duty);

    int32_t clamped_right = right;
    if (clamped_right > BC_MOTOR_LIMIT)
        clamped_right = BC_MOTOR_LIMIT;
    else if (clamped_right < -BC_MOTOR_LIMIT)
        clamped_right = -BC_MOTOR_LIMIT;

    int32_t right_duty = (clamped_right >= 0) ? clamped_right : -clamped_right;
    if (clamped_right >= 0)
        board_motor_write(1, false, true, (uint16_t)right_duty);
    else
        board_motor_write(1, true, false, (uint16_t)right_duty);
}

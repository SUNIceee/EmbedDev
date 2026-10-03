/* balance_control.c
 * Three-loop balance control library implementation.
 * Implements the fixed public API declared in 6_generated_code.h.
 */
#include "6_generated_code.h"

/* Public state definitions. */
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

/* Velocity-loop private state. */
static float vc_e1 = 0.0f;
static float vc_e2 = 0.0f;
static int32_t vc_accumulator = 0;
static int32_t vc_old = 0;
static int32_t vc_new = 0;
static uint8_t vc_count = 0;

/* Direction-loop private state. */
static int16_t dc_prev_filtered = 0;
static int32_t dc_old = 0;
static int32_t dc_new = 0;
static uint8_t dc_count = 0;

static void velocity_pid_update(int32_t measured_speed)
{
    float e = 0.0f - (float)measured_speed;
    float p = BC_VC_KP * (e - vc_e1);
    float i = BC_VC_KI * e;

    if (i > 10.0f || i < -10.0f)
    {
        i = 0.0f;
    }

    float d = BC_VC_KD * (e - 2.0f * vc_e1 + vc_e2);
    float next = (float)vc_accumulator + p + i + d;
    int32_t acc = (int32_t)next;

    if (acc > BC_VC_LIMIT)
    {
        acc = BC_VC_LIMIT;
    }
    else if (acc < -BC_VC_LIMIT)
    {
        acc = -BC_VC_LIMIT;
    }

    vc_accumulator = acc;
    vc_e2 = vc_e1;
    vc_e1 = e;
}

static int32_t clamp_motor(int32_t value)
{
    if (value > BC_MOTOR_LIMIT)
    {
        return BC_MOTOR_LIMIT;
    }
    if (value < -BC_MOTOR_LIMIT)
    {
        return -BC_MOTOR_LIMIT;
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

    vc_e1 = 0.0f;
    vc_e2 = 0.0f;
    vc_accumulator = 0;
    vc_old = 0;
    vc_new = 0;
    vc_count = 0;

    dc_prev_filtered = 0;
    dc_old = 0;
    dc_new = 0;
    dc_count = 0;
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

    int32_t left_abs = left < 0 ? -left : left;
    int32_t right_abs = right < 0 ? -right : right;

    return (int)((left_abs + right_abs) / 2);
}

int32_t angle_proc(void)
{
    return -(int32_t)(BC_AC_KP * (BC_AC_SET - pitch) - BC_AC_KD * (float)gyroy);
}

int32_t velocity_proc(int32_t measured_speed)
{
    if (vc_count >= BC_VC_PERIOD)
    {
        vc_count = 0;
        vc_old = vc_new;
        velocity_pid_update(measured_speed);
        vc_new = vc_accumulator;
    }

    vc_count++;
    return vc_old + ((vc_new - vc_old) * (int32_t)vc_count) / (int32_t)BC_VC_PERIOD;
}

int32_t direction_proc(int32_t measured_speed)
{
    int16_t filtered = (int16_t)(0.9f * (float)gyroz + 0.1f * (float)dc_prev_filtered);
    gyroz = filtered;
    dc_prev_filtered = filtered;

    if (dc_count >= BC_DC_PERIOD)
    {
        dc_count = 0;
        dc_old = dc_new;

        state = board_direction_error();

        float pgain = BC_DC_COEF * (float)measured_speed;
        if (pgain < BC_DC_P_MIN)
        {
            pgain = BC_DC_P_MIN;
        }
        else if (pgain > BC_DC_P_MAX)
        {
            pgain = BC_DC_P_MAX;
        }

        int32_t pterm = (int32_t)(pgain * (float)state / 25.0f);
        int32_t dterm = (int32_t)(BC_DC_KD * (float)gyroz / 100.0f);
        int32_t endpoint = pterm + dterm;

        if (endpoint > BC_DC_LIMIT)
        {
            endpoint = BC_DC_LIMIT;
        }
        else if (endpoint < -BC_DC_LIMIT)
        {
            endpoint = -BC_DC_LIMIT;
        }

        dc_new = endpoint;
    }

    dc_count++;
    return dc_old + ((dc_new - dc_old) * (int32_t)dc_count) / (int32_t)BC_DC_PERIOD;
}

void get_pwm(void)
{
    get_mpu();

    int32_t measured = get_speed();
    speed = measured;

    ac_pwm = angle_proc();
    vc_pwm = velocity_proc(measured);
    dc_pwm = direction_proc(measured);

    left_pwm = ac_pwm - vc_pwm + dc_pwm;
    right_pwm = ac_pwm - vc_pwm - dc_pwm;
}

void motor_proc(int32_t left, int32_t right)
{
    int32_t l = clamp_motor(left);
    int32_t r = clamp_motor(right);

    bool l_in_a;
    bool l_in_b;
    uint16_t l_duty;

    if (l >= 0)
    {
        l_in_a = false;
        l_in_b = true;
        l_duty = (uint16_t)l;
    }
    else
    {
        l_in_a = true;
        l_in_b = false;
        l_duty = (uint16_t)(-l);
    }

    board_motor_write(0, l_in_a, l_in_b, l_duty);

    bool r_in_a;
    bool r_in_b;
    uint16_t r_duty;

    if (r >= 0)
    {
        r_in_a = false;
        r_in_b = true;
        r_duty = (uint16_t)r;
    }
    else
    {
        r_in_a = true;
        r_in_b = false;
        r_duty = (uint16_t)(-r);
    }

    board_motor_write(1, r_in_a, r_in_b, r_duty);
}

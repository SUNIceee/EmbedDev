#include "6_generated_code.h"

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

static float velocity_e1 = 0.0f;
static float velocity_e2 = 0.0f;
static int32_t velocity_accumulator = 0;
static int32_t velocity_old_endpoint = 0;
static int32_t velocity_new_endpoint = 0;
static uint8_t velocity_count = 0;

static int16_t direction_previous_filtered = 0;
static int32_t direction_old_endpoint = 0;
static int32_t direction_new_endpoint = 0;
static uint8_t direction_count = 0;

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

    velocity_e1 = 0.0f;
    velocity_e2 = 0.0f;
    velocity_accumulator = 0;
    velocity_old_endpoint = 0;
    velocity_new_endpoint = 0;
    velocity_count = 0;

    direction_previous_filtered = 0;
    direction_old_endpoint = 0;
    direction_new_endpoint = 0;
    direction_count = 0;
}

void get_mpu(void)
{
    board_read_imu(&pitch, &roll, &yaw, &gyrox, &gyroy, &gyroz);
}

int get_speed(void)
{
    int16_t left_count = board_encoder_read(0);
    int16_t right_count = board_encoder_read(1);

    board_encoder_clear(0);
    board_encoder_clear(1);

    int32_t abs_left = left_count < 0 ? -(int32_t)left_count : (int32_t)left_count;
    int32_t abs_right = right_count < 0 ? -(int32_t)right_count : (int32_t)right_count;

    return (int)((abs_left + abs_right) / 2);
}

int32_t angle_proc(void)
{
    return -(int32_t)(BC_AC_KP * (BC_AC_SET - pitch) - BC_AC_KD * (float)gyroy);
}

int32_t velocity_proc(int32_t measured_speed)
{
    if (velocity_count >= BC_VC_PERIOD) {
        velocity_count = 0;
        velocity_old_endpoint = velocity_new_endpoint;

        float e = 0.0f - (float)measured_speed;
        float p = BC_VC_KP * (e - velocity_e1);
        float i = BC_VC_KI * e;
        float d = BC_VC_KD * (e - 2.0f * velocity_e1 + velocity_e2);

        if (i > 10.0f || i < -10.0f) {
            i = 0.0f;
        }

        int32_t raw = (int32_t)((float)velocity_accumulator + p + i + d);
        if (raw > BC_VC_LIMIT) {
            velocity_accumulator = BC_VC_LIMIT;
        } else if (raw < -BC_VC_LIMIT) {
            velocity_accumulator = -BC_VC_LIMIT;
        } else {
            velocity_accumulator = raw;
        }

        velocity_e2 = velocity_e1;
        velocity_e1 = e;
        velocity_new_endpoint = velocity_accumulator;
    }

    velocity_count++;
    int32_t count = (int32_t)velocity_count;
    int32_t period = (int32_t)BC_VC_PERIOD;

    return velocity_old_endpoint + (velocity_new_endpoint - velocity_old_endpoint) * count / period;
}

int32_t direction_proc(int32_t measured_speed)
{
    int16_t filtered = (int16_t)(0.9f * (float)gyroz + 0.1f * (float)direction_previous_filtered);
    gyroz = filtered;
    direction_previous_filtered = filtered;

    if (direction_count >= BC_DC_PERIOD) {
        direction_count = 0;
        direction_old_endpoint = direction_new_endpoint;

        state = board_direction_error();

        float pgain = BC_DC_COEF * (float)measured_speed;
        if (pgain < BC_DC_P_MIN) {
            pgain = BC_DC_P_MIN;
        }
        if (pgain > BC_DC_P_MAX) {
            pgain = BC_DC_P_MAX;
        }

        int32_t pterm = (int32_t)(pgain * (float)state / 25.0f);
        int32_t dterm = (int32_t)(BC_DC_KD * (float)gyroz / 100.0f);
        int32_t sum = pterm + dterm;

        if (sum > BC_DC_LIMIT) {
            direction_new_endpoint = BC_DC_LIMIT;
        } else if (sum < -BC_DC_LIMIT) {
            direction_new_endpoint = -BC_DC_LIMIT;
        } else {
            direction_new_endpoint = sum;
        }
    }

    direction_count++;
    int32_t count = (int32_t)direction_count;
    int32_t period = (int32_t)BC_DC_PERIOD;

    return direction_old_endpoint + (direction_new_endpoint - direction_old_endpoint) * count / period;
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
    int32_t c0 = left;
    if (c0 < -BC_MOTOR_LIMIT) {
        c0 = -BC_MOTOR_LIMIT;
    }
    if (c0 > BC_MOTOR_LIMIT) {
        c0 = BC_MOTOR_LIMIT;
    }

    if (c0 >= 0) {
        board_motor_write(0, false, true, (uint16_t)c0);
    } else {
        board_motor_write(0, true, false, (uint16_t)(-c0));
    }

    int32_t c1 = right;
    if (c1 < -BC_MOTOR_LIMIT) {
        c1 = -BC_MOTOR_LIMIT;
    }
    if (c1 > BC_MOTOR_LIMIT) {
        c1 = BC_MOTOR_LIMIT;
    }

    if (c1 >= 0) {
        board_motor_write(1, false, true, (uint16_t)c1);
    } else {
        board_motor_write(1, true, false, (uint16_t)(-c1));
    }
}
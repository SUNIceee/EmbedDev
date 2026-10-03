#include "generated_code.h"

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

static float vel_e1 = 0.0f;
static float vel_e2 = 0.0f;
static int32_t vel_accumulator = 0;

static int32_t vel_old = 0;
static int32_t vel_new = 0;
static int32_t vel_count = 0;

static int16_t dir_previous_filtered = 0;

static int32_t dir_old = 0;
static int32_t dir_new = 0;
static int32_t dir_count = 0;

void control_init(void)
{
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

    vel_e1 = 0.0f;
    vel_e2 = 0.0f;
    vel_accumulator = 0;

    vel_old = 0;
    vel_new = 0;
    vel_count = 0;

    dir_previous_filtered = 0;

    dir_old = 0;
    dir_new = 0;
    dir_count = 0;
}

void get_mpu(void)
{
    board_read_imu(&pitch, &roll, &yaw,
                   &gyrox, &gyroy, &gyroz);
}

int32_t get_speed(void)
{
    int32_t left = (int32_t)board_encoder_read(0);
    int32_t right = (int32_t)board_encoder_read(1);

    board_encoder_clear(0);
    board_encoder_clear(1);

    if (left < 0)
    {
        left = -left;
    }

    if (right < 0)
    {
        right = -right;
    }

    return (left + right) / 2;
}

int32_t angle_proc(void)
{
    return -(int32_t)(550.0f * (0.0f - pitch) - 2.0f * (float)gyroy);
}

int32_t velocity_proc(int32_t measured_speed)
{
    if (vel_count >= 5)
    {
        vel_count = 0;
        vel_old = vel_new;

        float e = 0.0f - (float)measured_speed;
        float P = 8.0f * (e - vel_e1);
        float I = 0.1f * e;
        float D = 0.5f * (e - 2.0f * vel_e1 + vel_e2);

        if (I > 10.0f || I < -10.0f)
        {
            I = 0.0f;
        }

        float raw_acc = (float)vel_accumulator + P + I + D;
        int32_t acc = (int32_t)raw_acc;

        if (acc > 4500)
        {
            acc = 4500;
        }
        else if (acc < -4500)
        {
            acc = -4500;
        }

        vel_accumulator = acc;
        vel_new = acc;

        vel_e2 = vel_e1;
        vel_e1 = e;
    }

    vel_count++;

    return vel_old + ((vel_new - vel_old) * vel_count) / 5;
}

int32_t direction_proc(int32_t measured_speed)
{
    int16_t filtered = (int16_t)(0.9f * (float)gyroz +
                                 0.1f * (float)dir_previous_filtered);
    gyroz = filtered;
    dir_previous_filtered = filtered;

    if (dir_count >= 5)
    {
        dir_count = 0;
        dir_old = dir_new;

        direction_error = board_direction_error();

        float pgain = 0.6f * (float)measured_speed;
        if (pgain < 50.0f)
        {
            pgain = 50.0f;
        }
        else if (pgain > 500.0f)
        {
            pgain = 500.0f;
        }

        int32_t pterm = (int32_t)(pgain * (float)direction_error / 25.0f);
        int32_t dterm = (int32_t)(1.5f * (float)gyroz / 100.0f);

        int32_t endpoint = pterm + dterm;
        if (endpoint > 3000)
        {
            endpoint = 3000;
        }
        else if (endpoint < -3000)
        {
            endpoint = -3000;
        }

        dir_new = endpoint;
    }

    dir_count++;

    return dir_old + ((dir_new - dir_old) * dir_count) / 5;
}

void get_pwm(void)
{
    get_mpu();

    int32_t measured_speed = get_speed();
    speed = measured_speed;

    ac_pwm = angle_proc();
    vc_pwm = velocity_proc(measured_speed);
    dc_pwm = direction_proc(measured_speed);

    left_pwm = ac_pwm - vc_pwm + dc_pwm;
    right_pwm = ac_pwm - vc_pwm - dc_pwm;
}

void motor_proc(int32_t left, int32_t right)
{
    int32_t l = left;
    int32_t r = right;

    if (l > 7200)
    {
        l = 7200;
    }
    else if (l < -7200)
    {
        l = -7200;
    }

    if (r > 7200)
    {
        r = 7200;
    }
    else if (r < -7200)
    {
        r = -7200;
    }

    if (l >= 0)
    {
        board_motor_write(0, false, true, l);
    }
    else
    {
        board_motor_write(0, true, false, -l);
    }

    if (r >= 0)
    {
        board_motor_write(1, false, true, r);
    }
    else
    {
        board_motor_write(1, true, false, -r);
    }
}

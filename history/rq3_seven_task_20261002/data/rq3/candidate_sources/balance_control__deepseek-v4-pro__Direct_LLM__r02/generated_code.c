#include "generated_code.h"

/* Public sensor and output globals */
float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;
int16_t gyrox = 0;
int16_t gyroy = 0;
int16_t gyroz = 0;
int32_t speed = 0;

int32_t ac_pwm = 0;
int32_t vc_pwm = 0;
int32_t dc_pwm = 0;
int32_t left_pwm = 0;
int32_t right_pwm = 0;

int16_t state = 0;

/* Private velocity loop state */
static float e1 = 0.0f;
static float e2 = 0.0f;
static int32_t accumulator = 0;
static int32_t vel_old = 0;
static int32_t vel_new = 0;
static int32_t vel_count = 0;

/* Private direction loop state */
static int16_t previous_filtered = 0;
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

    ac_pwm = 0;
    vc_pwm = 0;
    dc_pwm = 0;
    left_pwm = 0;
    right_pwm = 0;
    state = 0;

    e1 = 0.0f;
    e2 = 0.0f;
    accumulator = 0;
    vel_old = 0;
    vel_new = 0;
    vel_count = 0;

    previous_filtered = 0;
    dir_old = 0;
    dir_new = 0;
    dir_count = 0;
}

void get_mpu(void)
{
    board_read_imu(&pitch, &roll, &yaw, &gyrox, &gyroy, &gyroz);
}

int32_t get_speed(void)
{
    int32_t left = (int32_t)board_encoder_read(0);
    int32_t right = (int32_t)board_encoder_read(1);

    board_encoder_clear(0);
    board_encoder_clear(1);

    int32_t abs_left = left < 0 ? -left : left;
    int32_t abs_right = right < 0 ? -right : right;

    return (abs_left + abs_right) / 2;
}

int32_t angle_proc(void)
{
    return -(int32_t)(550.0f * (0.0f - pitch) - 2.0f * (float)gyroy);
}

int32_t velocity_proc(int32_t measured_speed)
{
    if (vel_count >= 5) {
        vel_count = 0;
        vel_old = vel_new;

        float e = 0.0f - (float)measured_speed;
        float P = 8.0f * (e - e1);
        float I = 0.1f * e;
        float D = 0.5f * (e - 2.0f * e1 + e2);

        if (I > 10.0f || I < -10.0f) {
            I = 0.0f;
        }

        int32_t new_acc = (int32_t)((float)accumulator + P + I + D);
        if (new_acc < -4500) {
            new_acc = -4500;
        } else if (new_acc > 4500) {
            new_acc = 4500;
        }

        accumulator = new_acc;
        vel_new = accumulator;

        e2 = e1;
        e1 = e;
    }

    vel_count++;
    return vel_old + (vel_new - vel_old) * vel_count / 5;
}

int32_t direction_proc(int32_t measured_speed)
{
    float filtered = 0.9f * (float)gyroz + 0.1f * (float)previous_filtered;
    gyroz = (int16_t)filtered;
    previous_filtered = gyroz;

    if (dir_count >= 5) {
        dir_count = 0;
        dir_old = dir_new;

        state = board_direction_error();

        float Pgain = 0.6f * (float)measured_speed;
        if (Pgain < 50.0f) {
            Pgain = 50.0f;
        } else if (Pgain > 500.0f) {
            Pgain = 500.0f;
        }

        int32_t Pterm = (int32_t)(Pgain * (float)state / 25.0f);
        int32_t Dterm = (int32_t)(1.5f * (float)gyroz / 100.0f);
        int32_t endpoint = Pterm + Dterm;

        if (endpoint < -3000) {
            endpoint = -3000;
        } else if (endpoint > 3000) {
            endpoint = 3000;
        }

        dir_new = endpoint;
    }

    dir_count++;
    return dir_old + (dir_new - dir_old) * dir_count / 5;
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
    int32_t clamped_right = right;

    if (clamped_left > 7200) {
        clamped_left = 7200;
    } else if (clamped_left < -7200) {
        clamped_left = -7200;
    }

    if (clamped_right > 7200) {
        clamped_right = 7200;
    } else if (clamped_right < -7200) {
        clamped_right = -7200;
    }

    if (clamped_left >= 0) {
        board_motor_write(0, 0, 1, clamped_left);
    } else {
        board_motor_write(0, 1, 0, -clamped_left);
    }

    if (clamped_right >= 0) {
        board_motor_write(1, 0, 1, clamped_right);
    } else {
        board_motor_write(1, 1, 0, -clamped_right);
    }
}

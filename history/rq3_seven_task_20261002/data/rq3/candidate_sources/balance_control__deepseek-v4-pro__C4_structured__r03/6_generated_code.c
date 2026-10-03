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
static int32_t velocity_acc = 0;
static int32_t velocity_old = 0;
static int32_t velocity_new = 0;
static uint8_t velocity_count = 0;

static int16_t direction_previous_filtered = 0;
static int32_t direction_old = 0;
static int32_t direction_new = 0;
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
    velocity_acc = 0;
    velocity_old = 0;
    velocity_new = 0;
    velocity_count = 0;

    direction_previous_filtered = 0;
    direction_old = 0;
    direction_new = 0;
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
    int32_t left_abs;
    int32_t right_abs;

    board_encoder_clear(0);
    board_encoder_clear(1);

    left_abs = (left_count < 0) ? (-(int32_t)left_count) : ((int32_t)left_count);
    right_abs = (right_count < 0) ? (-(int32_t)right_count) : ((int32_t)right_count);

    return (int)((left_abs + right_abs) / 2);
}

int32_t angle_proc(void)
{
    return -(int32_t)(BC_AC_KP * (BC_AC_SET - pitch) - BC_AC_KD * (float)gyroy);
}

int32_t velocity_proc(int32_t measured_speed)
{
    if (velocity_count >= (uint8_t)BC_VC_PERIOD) {
        float e;
        float P;
        float I;
        float D;
        int32_t candidate;

        velocity_count = 0;
        velocity_old = velocity_new;

        e = 0.0f - (float)measured_speed;
        P = BC_VC_KP * (e - velocity_e1);
        I = BC_VC_KI * e;
        if (I > 10.0f || I < -10.0f) {
            I = 0.0f;
        }
        D = BC_VC_KD * (e - 2.0f * velocity_e1 + velocity_e2);

        candidate = (int32_t)((float)velocity_acc + P + I + D);
        if (candidate < -BC_VC_LIMIT) {
            candidate = -BC_VC_LIMIT;
        } else if (candidate > BC_VC_LIMIT) {
            candidate = BC_VC_LIMIT;
        }

        velocity_acc = candidate;
        velocity_new = candidate;
        velocity_e2 = velocity_e1;
        velocity_e1 = e;
    }

    velocity_count++;
    return velocity_old + ((velocity_new - velocity_old) * (int32_t)velocity_count) / (int32_t)BC_VC_PERIOD;
}

int32_t direction_proc(int32_t measured_speed)
{
    int16_t filtered = (int16_t)(0.9f * (float)gyroz + 0.1f * (float)direction_previous_filtered);
    gyroz = filtered;
    direction_previous_filtered = filtered;

    if (direction_count >= (uint8_t)BC_DC_PERIOD) {
        float Pgain;
        int32_t Pterm;
        int32_t Dterm;
        int32_t endpoint;

        direction_count = 0;
        direction_old = direction_new;

        state = board_direction_error();

        Pgain = BC_DC_COEF * (float)measured_speed;
        if (Pgain < BC_DC_P_MIN) {
            Pgain = BC_DC_P_MIN;
        } else if (Pgain > BC_DC_P_MAX) {
            Pgain = BC_DC_P_MAX;
        }

        Pterm = (int32_t)(Pgain * (float)state / 25.0f);
        Dterm = (int32_t)(BC_DC_KD * (float)gyroz / 100.0f);
        endpoint = Pterm + Dterm;
        if (endpoint < -BC_DC_LIMIT) {
            endpoint = -BC_DC_LIMIT;
        } else if (endpoint > BC_DC_LIMIT) {
            endpoint = BC_DC_LIMIT;
        }

        direction_new = endpoint;
    }

    direction_count++;
    return direction_old + ((direction_new - direction_old) * (int32_t)direction_count) / (int32_t)BC_DC_PERIOD;
}

void get_pwm(void)
{
    get_mpu();
    speed = (int32_t)get_speed();
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

    if (clamped_left < -BC_MOTOR_LIMIT) {
        clamped_left = -BC_MOTOR_LIMIT;
    } else if (clamped_left > BC_MOTOR_LIMIT) {
        clamped_left = BC_MOTOR_LIMIT;
    }

    if (clamped_left >= 0) {
        board_motor_write(0, false, true, (uint16_t)clamped_left);
    } else {
        board_motor_write(0, true, false, (uint16_t)(-clamped_left));
    }

    if (clamped_right < -BC_MOTOR_LIMIT) {
        clamped_right = -BC_MOTOR_LIMIT;
    } else if (clamped_right > BC_MOTOR_LIMIT) {
        clamped_right = BC_MOTOR_LIMIT;
    }

    if (clamped_right >= 0) {
        board_motor_write(1, false, true, (uint16_t)clamped_right);
    } else {
        board_motor_write(1, true, false, (uint16_t)(-clamped_right));
    }
}
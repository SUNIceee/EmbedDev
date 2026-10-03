#include "6_generated_code.h"
#include <stdlib.h>

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

typedef struct {
    float e1;
    float e2;
    int32_t accumulator;
    int32_t old_endpoint;
    int32_t new_endpoint;
    uint8_t count;
} VelocityLoopState;

typedef struct {
    int16_t previous_filtered;
    int32_t old_endpoint;
    int32_t new_endpoint;
    uint8_t count;
} DirectionLoopState;

static VelocityLoopState s_velocity;
static DirectionLoopState s_direction;

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

    s_velocity.e1 = 0.0f;
    s_velocity.e2 = 0.0f;
    s_velocity.accumulator = 0;
    s_velocity.old_endpoint = 0;
    s_velocity.new_endpoint = 0;
    s_velocity.count = 0;

    s_direction.previous_filtered = 0;
    s_direction.old_endpoint = 0;
    s_direction.new_endpoint = 0;
    s_direction.count = 0;
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

    return (abs((int)left_count) + abs((int)right_count)) / 2;
}

int32_t angle_proc(void)
{
    return -(int32_t)(BC_AC_KP * (BC_AC_SET - pitch) - BC_AC_KD * (float)gyroy);
}

int32_t velocity_proc(int32_t measured_speed)
{
    if (s_velocity.count >= (int)BC_VC_PERIOD) {
        s_velocity.count = 0;
        s_velocity.old_endpoint = s_velocity.new_endpoint;

        float e = 0.0f - (float)measured_speed;
        float P = BC_VC_KP * (e - s_velocity.e1);
        float I = BC_VC_KI * e;
        if (I > 10.0f || I < -10.0f) {
            I = 0.0f;
        }
        float D = BC_VC_KD * (e - 2.0f * s_velocity.e1 + s_velocity.e2);

        int32_t acc = (int32_t)((float)s_velocity.accumulator + P + I + D);
        if (acc < -BC_VC_LIMIT) {
            acc = -BC_VC_LIMIT;
        } else if (acc > BC_VC_LIMIT) {
            acc = BC_VC_LIMIT;
        }

        s_velocity.accumulator = acc;
        s_velocity.e2 = s_velocity.e1;
        s_velocity.e1 = e;
        s_velocity.new_endpoint = s_velocity.accumulator;
    }

    s_velocity.count++;
    return s_velocity.old_endpoint +
           (s_velocity.new_endpoint - s_velocity.old_endpoint) * (int32_t)s_velocity.count /
           (int32_t)BC_VC_PERIOD;
}

int32_t direction_proc(int32_t measured_speed)
{
    int16_t filtered = (int16_t)(0.9f * (float)gyroz +
                                 0.1f * (float)s_direction.previous_filtered);
    gyroz = filtered;
    s_direction.previous_filtered = filtered;

    if (s_direction.count >= (int)BC_DC_PERIOD) {
        s_direction.count = 0;
        s_direction.old_endpoint = s_direction.new_endpoint;

        state = board_direction_error();

        float pgain = BC_DC_COEF * (float)measured_speed;
        if (pgain < BC_DC_P_MIN) {
            pgain = BC_DC_P_MIN;
        } else if (pgain > BC_DC_P_MAX) {
            pgain = BC_DC_P_MAX;
        }

        int32_t pterm = (int32_t)(pgain * (float)state / 25.0f);
        int32_t dterm = (int32_t)(BC_DC_KD * (float)gyroz / 100.0f);
        int32_t sum = pterm + dterm;

        if (sum < -BC_DC_LIMIT) {
            sum = -BC_DC_LIMIT;
        } else if (sum > BC_DC_LIMIT) {
            sum = BC_DC_LIMIT;
        }

        s_direction.new_endpoint = sum;
    }

    s_direction.count++;
    return s_direction.old_endpoint +
           (s_direction.new_endpoint - s_direction.old_endpoint) * (int32_t)s_direction.count /
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
    int32_t l = left;
    if (l > BC_MOTOR_LIMIT) {
        l = BC_MOTOR_LIMIT;
    } else if (l < -BC_MOTOR_LIMIT) {
        l = -BC_MOTOR_LIMIT;
    }

    int32_t r = right;
    if (r > BC_MOTOR_LIMIT) {
        r = BC_MOTOR_LIMIT;
    } else if (r < -BC_MOTOR_LIMIT) {
        r = -BC_MOTOR_LIMIT;
    }

    bool l_in_a;
    bool l_in_b;
    uint16_t l_duty;
    if (l < 0) {
        l_in_a = true;
        l_in_b = false;
        l_duty = (uint16_t)(-l);
    } else {
        l_in_a = false;
        l_in_b = true;
        l_duty = (uint16_t)l;
    }
    board_motor_write(0, l_in_a, l_in_b, l_duty);

    bool r_in_a;
    bool r_in_b;
    uint16_t r_duty;
    if (r < 0) {
        r_in_a = true;
        r_in_b = false;
        r_duty = (uint16_t)(-r);
    } else {
        r_in_a = false;
        r_in_b = true;
        r_duty = (uint16_t)r;
    }
    board_motor_write(1, r_in_a, r_in_b, r_duty);
}
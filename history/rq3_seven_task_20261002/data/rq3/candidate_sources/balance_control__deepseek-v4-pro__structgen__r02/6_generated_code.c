#include "6_generated_code.h"

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float clamp_f32(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
int16_t gyrox = 0, gyroy = 0, gyroz = 0, state = 0;
int32_t speed = 0, ac_pwm = 0, vc_pwm = 0, dc_pwm = 0, left_pwm = 0, right_pwm = 0;

static float velocity_e1 = 0.0f;
static float velocity_e2 = 0.0f;
static int32_t velocity_accumulator = 0;
static int32_t velocity_old = 0;
static int32_t velocity_new = 0;
static uint32_t velocity_count = 0u;

static int16_t direction_prev_filtered = 0;
static int32_t direction_old = 0;
static int32_t direction_new = 0;
static uint32_t direction_count = 0u;

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

    velocity_e1 = 0.0f;
    velocity_e2 = 0.0f;
    velocity_accumulator = 0;
    velocity_old = 0;
    velocity_new = 0;
    velocity_count = 0u;

    direction_prev_filtered = 0;
    direction_old = 0;
    direction_new = 0;
    direction_count = 0u;
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
    int32_t left_count = (int32_t)board_encoder_read(0);
    int32_t right_count = (int32_t)board_encoder_read(1);

    board_encoder_clear(0);
    board_encoder_clear(1);

    if (left_count < 0) {
        left_count = -left_count;
    }
    if (right_count < 0) {
        right_count = -right_count;
    }

    return (int)((left_count + right_count) / 2);
}

int32_t angle_proc(void) {
    return -(int32_t)(BC_AC_KP * (BC_AC_SET - pitch) - BC_AC_KD * (float)gyroy);
}

int32_t velocity_proc(int32_t measured_speed) {
    if (velocity_count >= BC_VC_PERIOD) {
        velocity_count = 0u;
        velocity_old = velocity_new;

        float e = (float)BC_VC_SET - (float)measured_speed;
        float P = BC_VC_KP * (e - velocity_e1);
        float I = BC_VC_KI * e;
        float D = BC_VC_KD * (e - 2.0f * velocity_e1 + velocity_e2);

        if (I > 10.0f || I < -10.0f) {
            I = 0.0f;
        }

        float acc = (float)velocity_accumulator + P + I + D;
        velocity_accumulator = (int32_t)acc;
        velocity_accumulator = clamp_i32(velocity_accumulator, -BC_VC_LIMIT, BC_VC_LIMIT);

        velocity_new = velocity_accumulator;
        velocity_e2 = velocity_e1;
        velocity_e1 = e;
    }

    velocity_count++;
    int32_t count = (int32_t)velocity_count;

    return velocity_old + (velocity_new - velocity_old) * count / (int32_t)BC_VC_PERIOD;
}

int32_t direction_proc(int32_t measured_speed) {
    float filtered_f = 0.9f * (float)gyroz + 0.1f * (float)direction_prev_filtered;
    gyroz = (int16_t)filtered_f;
    direction_prev_filtered = gyroz;

    if (direction_count >= BC_DC_PERIOD) {
        direction_count = 0u;
        direction_old = direction_new;

        state = board_direction_error();

        float pgain = BC_DC_COEF * (float)measured_speed;
        pgain = clamp_f32(pgain, BC_DC_P_MIN, BC_DC_P_MAX);

        int32_t Pterm = (int32_t)(pgain * (float)state / 25.0f);
        int32_t Dterm = (int32_t)(BC_DC_KD * (float)gyroz / 100.0f);
        int32_t sum = Pterm + Dterm;

        direction_new = clamp_i32(sum, -BC_DC_LIMIT, BC_DC_LIMIT);
    }

    direction_count++;
    int32_t count = (int32_t)direction_count;

    return direction_old + (direction_new - direction_old) * count / (int32_t)BC_DC_PERIOD;
}

void get_pwm(void) {
    get_mpu();
    speed = get_speed();

    ac_pwm = angle_proc();
    vc_pwm = velocity_proc(speed);
    dc_pwm = direction_proc(speed);

    left_pwm = ac_pwm - vc_pwm + dc_pwm;
    right_pwm = ac_pwm - vc_pwm - dc_pwm;
}

void motor_proc(int32_t left, int32_t right) {
    int32_t cl = clamp_i32(left, -BC_MOTOR_LIMIT, BC_MOTOR_LIMIT);
    int32_t cr = clamp_i32(right, -BC_MOTOR_LIMIT, BC_MOTOR_LIMIT);

    if (cl >= 0) {
        board_motor_write(0, false, true, (uint16_t)cl);
    } else {
        board_motor_write(0, true, false, (uint16_t)(-cl));
    }

    if (cr >= 0) {
        board_motor_write(1, false, true, (uint16_t)cr);
    } else {
        board_motor_write(1, true, false, (uint16_t)(-cr));
    }
}

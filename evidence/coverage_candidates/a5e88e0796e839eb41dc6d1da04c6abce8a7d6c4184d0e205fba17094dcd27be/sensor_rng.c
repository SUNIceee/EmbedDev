/* Accelerometers, ADC temperature, hardware RNG, and assembly helper wrappers */
#include "6_generated_code.h"

static int16_t acc_raw[3] = {0, 0, 0};
static uint16_t adc_raw = 0;
static uint32_t rng_val = 0;
static bool rng_ready_flag = true;

int init_accelerometers(void) {
    return 0;
}

void read_accelerometers(float b[3]) {
    if (!b) return;
    b[0] = (float)acc_raw[0] / 1000.0f;
    b[1] = (float)acc_raw[1] / 1000.0f;
    b[2] = (float)acc_raw[2] / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    acc_raw[0] = x_mg;
    acc_raw[1] = y_mg;
    acc_raw[2] = z_mg;
}

void init_temperature_sensor(void) {
}

float read_temperature_sensor(void) {
    float temp = (float)adc_raw / 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    adc_raw = raw;
}

void init_rng(void) {
}

uint32_t get_random_number(void) {
    int max_retries = 1000;
    while (!rng_ready_flag && --max_retries > 0) {
        /* Host test safety non-deadlock wait */
    }
    return rng_val;
}

void discobot_set_rng(bool ready, uint32_t value) {
    rng_ready_flag = ready;
    rng_val = value;
}

int func2(int R0) {
    return (int)((uint32_t)R0 + 1U);
}

int func1(int R0) {
    return func2(R0);
}

/* Internal ADC1 temperature sensor processing with 12-bit raw conversion formula */

#include "6_generated_code.h"

static uint16_t current_adc_raw = 943;

void init_temperature_sensor(void) {
}

void discobot_set_adc_raw(uint16_t raw) {
    current_adc_raw = raw;
}

float read_temperature_sensor(void) {
    float temp = (float)current_adc_raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

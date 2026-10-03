/* Accelerometer interface for LIS3DSH reading and raw simulation injection */

#include "6_generated_code.h"

static int16_t raw_x_mg = 0;
static int16_t raw_y_mg = 0;
static int16_t raw_z_mg = 1000;

int init_accelerometers(void) {
    return 0;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    raw_x_mg = x_mg;
    raw_y_mg = y_mg;
    raw_z_mg = z_mg;
}

void read_accelerometers(float b[3]) {
    if (b != NULL) {
        b[0] = (float)raw_x_mg / 1000.0f;
        b[1] = (float)raw_y_mg / 1000.0f;
        b[2] = (float)raw_z_mg / 1000.0f;
    }
}

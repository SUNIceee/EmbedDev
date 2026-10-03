/* STM32 Hardware RNG module with ready polling and host injection */

#include "6_generated_code.h"

static bool rng_ready_flag = true;
static uint32_t rng_stored_value = 0x12345678;

void init_rng(void) {
    rng_ready_flag = true;
}

void discobot_set_rng(bool ready, uint32_t value) {
    rng_ready_flag = ready;
    rng_stored_value = value;
}

uint32_t get_random_number(void) {
    /* Blocking wait for DRDY as per spec, with safety counter */
    uint32_t wait_counter = 100000;
    while (!rng_ready_flag && wait_counter > 0) {
        wait_counter--;
    }
    return rng_stored_value;
}

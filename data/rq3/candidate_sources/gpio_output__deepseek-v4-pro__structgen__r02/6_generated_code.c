#include "6_generated_code.h"
#include <stddef.h>

static bool initialized = false;
static bool state[2] = { false, false };

void gpio_output_init(void) {
    state[0] = false;
    state[1] = false;
    initialized = true;

    board_gpio_write(0, false);
    board_gpio_write(1, false);
}

bool gpio_output_set(uint8_t channel, bool on) {
    if (channel != 0 && channel != 1) {
        return false;
    }

    if (!initialized) {
        return false;
    }

    state[channel] = on;
    board_gpio_write(channel, on);
    return true;
}

bool gpio_output_toggle(uint8_t channel) {
    if (channel != 0 && channel != 1) {
        return false;
    }

    if (!initialized) {
        return false;
    }

    state[channel] = !state[channel];
    board_gpio_write(channel, state[channel]);
    return true;
}

bool gpio_output_get(uint8_t channel, bool *out_on) {
    if (channel != 0 && channel != 1) {
        return false;
    }

    if (!initialized) {
        return false;
    }

    if (out_on == NULL) {
        return false;
    }

    *out_on = state[channel];
    return true;
}

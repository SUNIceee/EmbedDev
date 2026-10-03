#include "6_generated_code.h"

static bool initialized = false;
static bool state[2] = { false, false };

static bool is_valid_channel(uint8_t channel) {
    return channel == 0 || channel == 1;
}

void gpio_output_init(void) {
    initialized = true;
    state[0] = false;
    state[1] = false;

    board_gpio_write(0, false);
    board_gpio_write(1, false);
}

bool gpio_output_set(uint8_t channel, bool on) {
    if (!initialized) {
        return false;
    }

    if (!is_valid_channel(channel)) {
        return false;
    }

    state[channel] = on;
    board_gpio_write(channel, on);

    return true;
}

bool gpio_output_toggle(uint8_t channel) {
    if (!initialized) {
        return false;
    }

    if (!is_valid_channel(channel)) {
        return false;
    }

    state[channel] = !state[channel];
    board_gpio_write(channel, state[channel]);

    return true;
}

bool gpio_output_get(uint8_t channel, bool *out_on) {
    if (!initialized) {
        return false;
    }

    if (out_on == NULL) {
        return false;
    }

    if (!is_valid_channel(channel)) {
        return false;
    }

    *out_on = state[channel];

    return true;
}

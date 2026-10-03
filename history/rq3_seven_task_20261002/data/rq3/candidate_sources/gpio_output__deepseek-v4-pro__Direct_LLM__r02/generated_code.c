#include "generated_code.h"

static bool initialized = false;
static bool output_state[2] = { false, false };

static bool is_valid_channel(uint8_t channel) {
    return channel == 0 || channel == 1;
}

bool gpio_output_init(void) {
    output_state[0] = false;
    output_state[1] = false;

    board_gpio_write(0, false);
    board_gpio_write(1, false);

    initialized = true;
    return true;
}

bool gpio_output_set(uint8_t channel, bool on) {
    if (!initialized || !is_valid_channel(channel)) {
        return false;
    }

    output_state[channel] = on;
    board_gpio_write(channel, on);
    return true;
}

bool gpio_output_toggle(uint8_t channel) {
    if (!initialized || !is_valid_channel(channel)) {
        return false;
    }

    bool new_state = !output_state[channel];
    output_state[channel] = new_state;
    board_gpio_write(channel, new_state);
    return true;
}

bool gpio_output_get(uint8_t channel, bool *out_on) {
    if (!initialized || !is_valid_channel(channel) || out_on == NULL) {
        return false;
    }

    *out_on = output_state[channel];
    return true;
}

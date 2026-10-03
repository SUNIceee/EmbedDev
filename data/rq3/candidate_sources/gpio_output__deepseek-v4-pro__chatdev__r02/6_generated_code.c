/* GPIO Output Controller implementation for logical channels 0 and 1. */

#include "6_generated_code.h"
#include <stddef.h>

static bool initialized = false;
static bool output_state[2] = { false, false };

static bool is_valid_channel(uint8_t channel)
{
    return (channel == 0U) || (channel == 1U);
}

void gpio_output_init(void)
{
    initialized = true;
    output_state[0] = false;
    output_state[1] = false;

    board_gpio_write(0U, false);
    board_gpio_write(1U, false);
}

bool gpio_output_set(uint8_t channel, bool on)
{
    if (!initialized || !is_valid_channel(channel))
    {
        return false;
    }

    output_state[channel] = on;
    board_gpio_write(channel, on);

    return true;
}

bool gpio_output_toggle(uint8_t channel)
{
    if (!initialized || !is_valid_channel(channel))
    {
        return false;
    }

    output_state[channel] = !output_state[channel];
    board_gpio_write(channel, output_state[channel]);

    return true;
}

bool gpio_output_get(uint8_t channel, bool *out_on)
{
    if (!initialized || (out_on == NULL) || !is_valid_channel(channel))
    {
        return false;
    }

    *out_on = output_state[channel];

    return true;
}

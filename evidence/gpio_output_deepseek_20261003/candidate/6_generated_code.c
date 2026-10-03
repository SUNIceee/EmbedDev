#include "6_generated_code.h"

static bool initialized = false;
static bool logical_state[2] = { false, false };

void gpio_output_init(void)
{
    initialized = true;
    logical_state[0] = false;
    logical_state[1] = false;

    board_gpio_write(0, false);
    board_gpio_write(1, false);
}

bool gpio_output_set(uint8_t channel, bool on)
{
    if (!initialized)
    {
        return false;
    }

    if ((channel != 0u) && (channel != 1u))
    {
        return false;
    }

    logical_state[channel] = on;
    board_gpio_write(channel, on);
    return true;
}

bool gpio_output_toggle(uint8_t channel)
{
    if (!initialized)
    {
        return false;
    }

    if ((channel != 0u) && (channel != 1u))
    {
        return false;
    }

    logical_state[channel] = !logical_state[channel];
    board_gpio_write(channel, logical_state[channel]);
    return true;
}

bool gpio_output_get(uint8_t channel, bool *out_on)
{
    if (!initialized)
    {
        return false;
    }

    if ((channel != 0u) && (channel != 1u))
    {
        return false;
    }

    if (out_on == NULL)
    {
        return false;
    }

    *out_on = logical_state[channel];
    return true;
}
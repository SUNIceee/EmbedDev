/* GPIO Output Controller implementation.
 * Maintains initialization state and the last requested logical state for
 * channels 0 and 1. board_gpio_write is platform-supplied and must not be
 * implemented in this generated code.
 */
#include "6_generated_code.h"
#include <stddef.h>

static bool initialized = false;
static bool channel_state[2] = {false, false};

void gpio_output_init(void)
{
    initialized = true;

    channel_state[0] = false;
    channel_state[1] = false;

    board_gpio_write(0U, false);
    board_gpio_write(1U, false);
}

bool gpio_output_set(uint8_t channel, bool on)
{
    if (!initialized)
    {
        return false;
    }

    if ((channel != 0U) && (channel != 1U))
    {
        return false;
    }

    channel_state[channel] = on;
    board_gpio_write(channel, on);

    return true;
}

bool gpio_output_toggle(uint8_t channel)
{
    if (!initialized)
    {
        return false;
    }

    if ((channel != 0U) && (channel != 1U))
    {
        return false;
    }

    channel_state[channel] = !channel_state[channel];
    board_gpio_write(channel, channel_state[channel]);

    return true;
}

bool gpio_output_get(uint8_t channel, bool *out_on)
{
    if (!initialized)
    {
        return false;
    }

    if ((channel != 0U) && (channel != 1U))
    {
        return false;
    }

    if (out_on == NULL)
    {
        return false;
    }

    *out_on = channel_state[channel];

    return true;
}

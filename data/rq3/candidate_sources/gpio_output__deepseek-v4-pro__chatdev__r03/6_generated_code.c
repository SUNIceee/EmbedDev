/* GPIO Output Controller implementation.
 *
 * Maintains two independent logical output channels, 0 and 1.
 * Calls to set/toggle/get are rejected until gpio_output_init()
 * has completed once. The platform callback board_gpio_write is
 * supplied by the platform and must not be implemented here.
 */
#include "6_generated_code.h"
#include <stddef.h>

static bool initialized = false;
static bool channel_state[2] = { false, false };

void gpio_output_init(void)
{
    /* Reinitialization has the same effects: both channels are
       forced off and the platform is notified once per channel. */
    channel_state[0] = false;
    channel_state[1] = false;

    board_gpio_write(0, false);
    board_gpio_write(1, false);

    initialized = true;
}

bool gpio_output_set(uint8_t channel, bool on)
{
    if (!initialized)
    {
        return false;
    }

    if (channel > 1)
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

    if (channel > 1)
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

    if (channel > 1)
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

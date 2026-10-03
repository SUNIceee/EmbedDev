#include "generated_code.h"
#include <stddef.h>

static bool g_initialized = false;
static bool g_state[2] = { false, false };

static bool gpio_output_channel_valid(uint8_t channel)
{
    return (channel == 0u) || (channel == 1u);
}

void gpio_output_init(void)
{
    g_state[0] = false;
    g_state[1] = false;
    g_initialized = true;

    board_gpio_write(0u, false);
    board_gpio_write(1u, false);
}

bool gpio_output_set(uint8_t channel, bool on)
{
    if (!g_initialized)
    {
        return false;
    }

    if (!gpio_output_channel_valid(channel))
    {
        return false;
    }

    g_state[channel] = on;
    board_gpio_write(channel, on);

    return true;
}

bool gpio_output_toggle(uint8_t channel)
{
    if (!g_initialized)
    {
        return false;
    }

    if (!gpio_output_channel_valid(channel))
    {
        return false;
    }

    g_state[channel] = !g_state[channel];
    board_gpio_write(channel, g_state[channel]);

    return true;
}

bool gpio_output_get(uint8_t channel, bool *out_on)
{
    if (!g_initialized)
    {
        return false;
    }

    if (!gpio_output_channel_valid(channel))
    {
        return false;
    }

    if (out_on == NULL)
    {
        return false;
    }

    *out_on = g_state[channel];

    return true;
}

#include "generated_code.h"

static bool g_initialized = false;
static bool g_state[2] = { false, false };

void gpio_output_init(void)
{
    g_initialized = true;
    g_state[0] = false;
    g_state[1] = false;

    board_gpio_write(0, false);
    board_gpio_write(1, false);
}

bool gpio_output_set(uint8_t channel, bool on)
{
    if (!g_initialized)
    {
        return false;
    }
    if (channel > 1)
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
    if (channel > 1)
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
    if (channel > 1)
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

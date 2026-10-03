#include "6_generated_code.h"
#include <stddef.h>

static bool initialized = false;
static bool ch0_on = false;
static bool ch1_on = false;
static uint32_t ch0_write_count = 0u;
static uint32_t ch1_write_count = 0u;
static bool last_ch0_write_level = false;
static bool last_ch1_write_level = false;

void gpio_output_init(void)
{
    ch0_on = false;
    ch1_on = false;

    ch0_write_count += 1u;
    last_ch0_write_level = false;
    board_gpio_write(0, false);

    ch1_write_count += 1u;
    last_ch1_write_level = false;
    board_gpio_write(1, false);

    initialized = true;
}

bool gpio_output_set(uint8_t channel, bool on)
{
    if (!initialized)
    {
        return false;
    }

    if ((channel != 0) && (channel != 1))
    {
        return false;
    }

    if (channel == 0)
    {
        ch0_on = on;
        ch0_write_count += 1u;
        last_ch0_write_level = on;
        board_gpio_write(0, on);
        return true;
    }

    if (channel == 1)
    {
        ch1_on = on;
        ch1_write_count += 1u;
        last_ch1_write_level = on;
        board_gpio_write(1, on);
        return true;
    }

    return false;
}

bool gpio_output_toggle(uint8_t channel)
{
    if (!initialized)
    {
        return false;
    }

    if ((channel != 0) && (channel != 1))
    {
        return false;
    }

    if (channel == 0)
    {
        ch0_on = !ch0_on;
        ch0_write_count += 1u;
        last_ch0_write_level = ch0_on;
        board_gpio_write(0, ch0_on);
        return true;
    }

    if (channel == 1)
    {
        ch1_on = !ch1_on;
        ch1_write_count += 1u;
        last_ch1_write_level = ch1_on;
        board_gpio_write(1, ch1_on);
        return true;
    }

    return false;
}

bool gpio_output_get(uint8_t channel, bool *out_on)
{
    if (out_on == NULL)
    {
        return false;
    }

    if (!initialized)
    {
        return false;
    }

    if ((channel != 0) && (channel != 1))
    {
        return false;
    }

    if (channel == 0)
    {
        *out_on = ch0_on;
        return true;
    }

    if (channel == 1)
    {
        *out_on = ch1_on;
        return true;
    }

    return false;
}
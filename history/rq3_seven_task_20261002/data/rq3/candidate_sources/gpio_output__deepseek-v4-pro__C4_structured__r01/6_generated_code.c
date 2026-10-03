#include "6_generated_code.h"

static bool gpio_output_initialized = false; /* alias: initialized, init_done */
static bool gpio_output_ch0_state = false;   /* alias: ch0_state, logical_state[0] */
static bool gpio_output_ch1_state = false;   /* alias: ch1_state, logical_state[1] */

void gpio_output_init(void)
{
    gpio_output_ch0_state = false;
    gpio_output_ch1_state = false;

    board_gpio_write(0, false);
    board_gpio_write(1, false);

    gpio_output_initialized = true;
}

bool gpio_output_set(uint8_t channel, bool on)
{
    if (gpio_output_initialized && ((channel == 0) || (channel == 1)))
    {
        if (channel == 0)
        {
            gpio_output_ch0_state = on;
        }
        else
        {
            gpio_output_ch1_state = on;
        }

        board_gpio_write(channel, on);
        return true;
    }

    return false;
}

bool gpio_output_toggle(uint8_t channel)
{
    if (!gpio_output_initialized)
    {
        return false;
    }

    if (channel == 0)
    {
        gpio_output_ch0_state = !gpio_output_ch0_state;
        board_gpio_write(0, gpio_output_ch0_state);
        return true;
    }

    if (channel == 1)
    {
        gpio_output_ch1_state = !gpio_output_ch1_state;
        board_gpio_write(1, gpio_output_ch1_state);
        return true;
    }

    return false;
}

bool gpio_output_get(uint8_t channel, bool *out_on)
{
    if ((out_on == NULL) || (!gpio_output_initialized))
    {
        return false;
    }

    if (channel == 0)
    {
        *out_on = gpio_output_ch0_state;
        return true;
    }

    if (channel == 1)
    {
        *out_on = gpio_output_ch1_state;
        return true;
    }

    return false;
}
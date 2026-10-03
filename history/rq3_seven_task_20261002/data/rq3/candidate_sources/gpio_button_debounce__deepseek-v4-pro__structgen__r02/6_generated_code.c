#include "6_generated_code.h"

static bool initialized = false;
static bool led_on = false;
static bool accepted_pressed = false;
static bool candidate = false;
static uint32_t candidate_start_ms = 0U;

void button_led_init(uint32_t now_ms)
{
    led_on = false;
    accepted_pressed = false;
    candidate = false;
    candidate_start_ms = now_ms;
    board_led_write(false);
    initialized = true;
}

void button_led_poll(uint32_t now_ms)
{
    if (!initialized)
    {
        return;
    }

    bool raw = board_button_read();

    if (raw == candidate)
    {
        if (candidate == accepted_pressed)
        {
            return;
        }

        uint32_t elapsed = (uint32_t)(now_ms - candidate_start_ms);
        if (elapsed >= BUTTON_DEBOUNCE_MS)
        {
            if ((candidate == true) && (accepted_pressed == false))
            {
                led_on = !led_on;
                board_led_write(led_on);
                accepted_pressed = true;
            }
            else
            {
                accepted_pressed = candidate;
            }
        }
    }
    else
    {
        candidate = raw;
        candidate_start_ms = now_ms;
    }
}

bool button_led_is_on(void)
{
    return led_on;
}

bool button_led_is_pressed(void)
{
    return accepted_pressed;
}

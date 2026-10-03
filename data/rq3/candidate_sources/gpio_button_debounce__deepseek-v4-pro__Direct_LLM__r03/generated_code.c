#include "generated_code.h"

static bool initialized = false;
static bool led_on = false;
static bool accepted = false;
static bool candidate = false;
static uint32_t candidate_start = 0u;

void button_led_init(uint32_t now_ms)
{
    initialized = true;
    led_on = false;
    accepted = false;
    candidate = false;
    candidate_start = now_ms;
    board_led_write(false);
}

void button_led_poll(uint32_t now_ms)
{
    bool sample;
    uint32_t elapsed;
    bool previous_accepted;

    if (!initialized)
    {
        return;
    }

    sample = board_button_read();

    if (sample != candidate)
    {
        candidate = sample;
        candidate_start = now_ms;
        return;
    }

    if (sample != accepted)
    {
        elapsed = (uint32_t)(now_ms - candidate_start);
        if (elapsed >= BUTTON_DEBOUNCE_MS)
        {
            previous_accepted = accepted;
            accepted = sample;
            if (!previous_accepted && accepted)
            {
                led_on = !led_on;
                board_led_write(led_on);
            }
        }
    }
}

bool button_led_is_on(void)
{
    if (!initialized)
    {
        return false;
    }
    return led_on;
}

bool button_led_is_pressed(void)
{
    if (!initialized)
    {
        return false;
    }
    return accepted;
}

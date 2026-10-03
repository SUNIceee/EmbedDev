
/* Implementation of the debounced button and LED state machine. */

#include "6_generated_code.h"

static bool initialized = false;
static bool led_on = false;
static bool accepted_pressed = false;
static bool candidate_pressed = false;
static uint32_t candidate_start_ms = 0;

void button_led_init(uint32_t now_ms)
{
    initialized = true;
    led_on = false;
    accepted_pressed = false;
    candidate_pressed = false;
    candidate_start_ms = now_ms;

    board_led_write(false);
}

void button_led_poll(uint32_t now_ms)
{
    bool sample;
    uint32_t elapsed;

    if (!initialized)
    {
        return;
    }

    sample = board_button_read();

    if (sample != candidate_pressed)
    {
        candidate_pressed = sample;
        candidate_start_ms = now_ms;
        return;
    }

    if (candidate_pressed == accepted_pressed)
    {
        return;
    }

    elapsed = (uint32_t)(now_ms - candidate_start_ms);

    if (elapsed >= BUTTON_DEBOUNCE_MS)
    {
        if (!accepted_pressed && candidate_pressed)
        {
            led_on = !led_on;
            board_led_write(led_on);
        }

        accepted_pressed = candidate_pressed;
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


#include "6_generated_code.h"

static bool initialized;
static bool led_on;
static bool accepted_pressed;
static bool candidate_pressed;
static uint32_t candidate_start_ms;

void button_led_init(uint32_t now_ms)
{
    led_on = false;
    accepted_pressed = false;
    candidate_pressed = false;
    candidate_start_ms = now_ms;
    initialized = true;

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

    /* GB-R04: a raw sample that differs from the current candidate
       starts a new debounce interval and does not change accepted state. */
    if (sample != candidate_pressed)
    {
        candidate_pressed = sample;
        candidate_start_ms = now_ms;
    }

    /* GB-R09: rollover-safe unsigned elapsed time. */
    elapsed = (uint32_t)(now_ms - candidate_start_ms);

    /* GB-R05: accept only at an observed poll after the debounce threshold. */
    if ((candidate_pressed != accepted_pressed) &&
        (elapsed >= BUTTON_DEBOUNCE_MS))
    {
        accepted_pressed = candidate_pressed;

        /* GB-R06: only a new debounced press toggles and writes the LED. */
        if (accepted_pressed)
        {
            led_on = !led_on;
            board_led_write(led_on);
        }
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
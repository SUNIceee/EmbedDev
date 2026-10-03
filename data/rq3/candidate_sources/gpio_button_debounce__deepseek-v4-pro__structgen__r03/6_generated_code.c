#include "6_generated_code.h"

static bool initialized = false;
static bool led_state = false;
static bool accepted_button_state = false;
static bool candidate_value = false;
static uint32_t candidate_start_ms = 0U;

void button_led_init(uint32_t now_ms)
{
    initialized = true;
    led_state = false;
    accepted_button_state = false;
    candidate_value = false;
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

    if (sample != candidate_value)
    {
        candidate_value = sample;
        candidate_start_ms = now_ms;
    }

    if (candidate_value != accepted_button_state)
    {
        elapsed = (uint32_t)(now_ms - candidate_start_ms);

        if (elapsed >= BUTTON_DEBOUNCE_MS)
        {
            if (candidate_value)
            {
                accepted_button_state = true;
                led_state = !led_state;
                board_led_write(led_state);
            }
            else
            {
                accepted_button_state = false;
            }
        }
    }
}

bool button_led_is_on(void)
{
    return led_state;
}

bool button_led_is_pressed(void)
{
    return accepted_button_state;
}

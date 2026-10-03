#include "6_generated_code.h"

static bool initialized = false;
static bool led_state = false;
static bool accepted_pressed = false;
static bool candidate_pressed = false;
static uint32_t candidate_start_ms = 0U;

void button_led_init(uint32_t now_ms)
{
    initialized = true;
    led_state = false;
    accepted_pressed = false;
    candidate_pressed = false;
    candidate_start_ms = now_ms;

    board_led_write(false);
}

void button_led_poll(uint32_t now_ms)
{
    if (!initialized)
    {
        return;
    }

    bool sample = board_button_read();

    if (sample != candidate_pressed)
    {
        candidate_pressed = sample;
        candidate_start_ms = now_ms;
        return;
    }

    uint32_t elapsed = (uint32_t)(now_ms - candidate_start_ms);

    if (elapsed >= BUTTON_DEBOUNCE_MS && candidate_pressed != accepted_pressed)
    {
        if (!accepted_pressed && candidate_pressed)
        {
            led_state = !led_state;
            board_led_write(led_state);
            accepted_pressed = true;
        }
        else if (accepted_pressed && !candidate_pressed)
        {
            accepted_pressed = false;
        }
    }
}

bool button_led_is_on(void)
{
    return led_state;
}

bool button_led_is_pressed(void)
{
    return accepted_pressed;
}
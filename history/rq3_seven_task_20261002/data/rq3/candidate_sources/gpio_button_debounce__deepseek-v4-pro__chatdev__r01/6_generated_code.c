/* Implementation of the active-high debounced button/LED controller. */

#include "6_generated_code.h"

static bool     g_initialized = false;
static bool     g_led_on = false;
static bool     g_accepted_pressed = false;
static bool     g_candidate_pressed = false;
static uint32_t g_candidate_start_ms = 0u;

void button_led_init(uint32_t now_ms)
{
    g_led_on = false;
    g_accepted_pressed = false;
    g_candidate_pressed = false;
    g_candidate_start_ms = now_ms;
    g_initialized = true;

    board_led_write(false);
}

void button_led_poll(uint32_t now_ms)
{
    if (!g_initialized)
    {
        return;
    }

    const bool sample = board_button_read();

    if (sample != g_candidate_pressed)
    {
        g_candidate_pressed = sample;
        g_candidate_start_ms = now_ms;
        return;
    }

    if (g_candidate_pressed == g_accepted_pressed)
    {
        return;
    }

    const uint32_t elapsed_ms = (uint32_t)(now_ms - g_candidate_start_ms);

    if (elapsed_ms >= BUTTON_DEBOUNCE_MS)
    {
        const bool was_pressed = g_accepted_pressed;

        g_accepted_pressed = g_candidate_pressed;

        if (!was_pressed && g_accepted_pressed)
        {
            g_led_on = !g_led_on;
            board_led_write(g_led_on);
        }
    }
}

bool button_led_is_on(void)
{
    if (!g_initialized)
    {
        return false;
    }

    return g_led_on;
}

bool button_led_is_pressed(void)
{
    if (!g_initialized)
    {
        return false;
    }

    return g_accepted_pressed;
}

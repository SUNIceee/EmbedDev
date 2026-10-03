#include "generated_code.h"

static bool g_initialized;
static bool g_led_on;
static bool g_accepted_pressed;
static bool g_candidate_pressed;
static uint32_t g_candidate_start_ms;

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
    bool sample;

    if (!g_initialized) {
        return;
    }

    sample = board_button_read();

    if (sample != g_candidate_pressed) {
        g_candidate_pressed = sample;
        g_candidate_start_ms = now_ms;
        return;
    }

    if (sample != g_accepted_pressed) {
        uint32_t elapsed = (uint32_t)(now_ms - g_candidate_start_ms);

        if (elapsed >= BUTTON_DEBOUNCE_MS) {
            g_accepted_pressed = sample;

            if (sample) {
                g_led_on = !g_led_on;
                board_led_write(g_led_on);
            }
        }
    }
}

bool button_led_is_on(void)
{
    return g_led_on;
}

bool button_led_is_pressed(void)
{
    return g_accepted_pressed;
}

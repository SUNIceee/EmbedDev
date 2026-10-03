#include "6_generated_code.h"

static bool g_initialized;
static bool g_accepted;
static bool g_candidate;
static uint32_t g_candidate_start_ms;
static bool g_led;

void button_led_init(uint32_t now_ms)
{
    g_accepted = false;
    g_candidate = false;
    g_candidate_start_ms = now_ms;
    g_led = false;

    board_led_write(false);

    g_initialized = true;
}

void button_led_poll(uint32_t now_ms)
{
    if (!g_initialized)
    {
        return;
    }

    bool raw = board_button_read();

    if (raw != g_candidate)
    {
        g_candidate = raw;
        g_candidate_start_ms = now_ms;
    }

    if ((g_candidate != g_accepted) &&
        ((uint32_t)(now_ms - g_candidate_start_ms) >= BUTTON_DEBOUNCE_MS))
    {
        bool previous_accepted = g_accepted;
        g_accepted = g_candidate;

        if ((previous_accepted == false) && (g_accepted == true))
        {
            g_led = !g_led;
            board_led_write(g_led);
        }
    }
}

bool button_led_is_on(void)
{
    if (!g_initialized)
    {
        return false;
    }

    return g_led;
}

bool button_led_is_pressed(void)
{
    if (!g_initialized)
    {
        return false;
    }

    return g_accepted;
}

#include "6_generated_code.h"

static bool g_initialized = false;
static bool g_led_state = false;
static bool g_accepted_pressed = false;
static bool g_candidate_pressed = false;
static uint32_t g_candidate_start_ms = 0U;
static uint32_t g_board_led_write_count = 0U;
static uint32_t g_board_button_read_count = 0U;

void button_led_init(uint32_t now_ms)
{
    g_initialized = true;
    g_led_state = false;
    g_accepted_pressed = false;
    g_candidate_pressed = false;
    g_candidate_start_ms = now_ms;
    ++g_board_led_write_count;
    board_led_write(false);
}

void button_led_poll(uint32_t now_ms)
{
    if (!g_initialized)
    {
        return;
    }

    bool sample = board_button_read();
    ++g_board_button_read_count;

    if (sample != g_candidate_pressed)
    {
        g_candidate_pressed = sample;
        g_candidate_start_ms = now_ms;
    }

    if ((g_candidate_pressed != g_accepted_pressed) &&
        ((uint32_t)(now_ms - g_candidate_start_ms) >= BUTTON_DEBOUNCE_MS))
    {
        if (g_candidate_pressed)
        {
            g_accepted_pressed = true;
            g_led_state = !g_led_state;
            ++g_board_led_write_count;
            board_led_write(g_led_state);
        }
        else
        {
            g_accepted_pressed = false;
        }
    }
}

bool button_led_is_on(void)
{
    return g_led_state;
}

bool button_led_is_pressed(void)
{
    return g_accepted_pressed;
}
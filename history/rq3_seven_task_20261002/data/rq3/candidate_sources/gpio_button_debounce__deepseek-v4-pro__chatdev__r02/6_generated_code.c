/* Implementation of the GPIO debounced button and LED controller.
 *
 * The controller tracks a candidate input value and promotes it to the
 * accepted button state only after it remains stable for at least
 * BUTTON_DEBOUNCE_MS. LED toggling occurs only on an accepted press.
 */
#include "6_generated_code.h"

static bool initialized;
static bool led_on;
static bool accepted_pressed;
static bool candidate_pressed;
static uint32_t candidate_start_ms;

void button_led_init(uint32_t now_ms) {
    initialized = true;
    led_on = false;
    accepted_pressed = false;
    candidate_pressed = false;
    candidate_start_ms = now_ms;

    board_led_write(false);
}

void button_led_poll(uint32_t now_ms) {
    if (!initialized) {
        return;
    }

    bool sample = board_button_read();

    if (sample != candidate_pressed) {
        candidate_pressed = sample;
        candidate_start_ms = now_ms;
    } else if (candidate_pressed != accepted_pressed) {
        uint32_t elapsed = (uint32_t)(now_ms - candidate_start_ms);

        if (elapsed >= BUTTON_DEBOUNCE_MS) {
            if (!accepted_pressed && candidate_pressed) {
                accepted_pressed = true;
                led_on = !led_on;
                board_led_write(led_on);
            } else {
                accepted_pressed = candidate_pressed;
            }
        }
    }
}

bool button_led_is_on(void) {
    if (!initialized) {
        return false;
    }

    return led_on;
}

bool button_led_is_pressed(void) {
    if (!initialized) {
        return false;
    }

    return accepted_pressed;
}

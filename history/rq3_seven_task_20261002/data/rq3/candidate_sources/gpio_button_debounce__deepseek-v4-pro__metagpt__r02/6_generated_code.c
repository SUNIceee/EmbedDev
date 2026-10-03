/*
 * 6_generated_code.c
 *
 * Button LED debounce controller implementation.
 *
 * The controller owns all debounce state in a file-scope static structure.
 * It is deliberately independent of any particular board or MCU.
 */
#include <stdbool.h>
#include <stdint.h>

#include "6_generated_code.h"

/* File-scope controller state.
 *
 * Explicit defaults are provided for clarity. Static storage duration also
 * guarantees zero initialization before the first call in hosted/freestanding
 * environments that support it.
 */
typedef struct ButtonLedController {
  bool initialized;
  bool led_on;
  bool accepted_pressed;
  bool candidate_pressed;
  uint32_t candidate_start_ms;
} ButtonLedController;

static ButtonLedController g_controller = {
    .initialized = false,
    .led_on = false,
    .accepted_pressed = false,
    .candidate_pressed = false,
    .candidate_start_ms = 0U,
};

void button_led_init(uint32_t now_ms) {
  g_controller.initialized = true;
  g_controller.led_on = false;
  g_controller.accepted_pressed = false;
  g_controller.candidate_pressed = false;
  g_controller.candidate_start_ms = now_ms;

  board_led_write(false);
}

void button_led_poll(uint32_t now_ms) {
  if (!g_controller.initialized) {
    return;
  }

  const bool sampled_pressed = board_button_read();

  /*
   * A new candidate level starts a fresh debounce interval. A single
   * differing sample never changes the accepted button state.
   */
  if (sampled_pressed != g_controller.candidate_pressed) {
    g_controller.candidate_pressed = sampled_pressed;
    g_controller.candidate_start_ms = now_ms;
    return;
  }

  /*
   * Rollover-safe elapsed-time calculation. uint32_t subtraction wraps
   * modulo 2^32, which yields the correct interval across timer rollover.
   */
  const uint32_t elapsed_ms =
      (uint32_t)(now_ms - g_controller.candidate_start_ms);

  if (elapsed_ms < BUTTON_DEBOUNCE_MS) {
    return;
  }

  /*
   * Only an accepted released-to-pressed transition toggles the LED and
   * writes it exactly once.
   */
  if (sampled_pressed != g_controller.accepted_pressed) {
    g_controller.accepted_pressed = sampled_pressed;

    if (sampled_pressed) {
      g_controller.led_on = !g_controller.led_on;
      board_led_write(g_controller.led_on);
    }
  }
}

bool button_led_is_on(void) {
  return g_controller.led_on;
}

bool button_led_is_pressed(void) {
  return g_controller.accepted_pressed;
}

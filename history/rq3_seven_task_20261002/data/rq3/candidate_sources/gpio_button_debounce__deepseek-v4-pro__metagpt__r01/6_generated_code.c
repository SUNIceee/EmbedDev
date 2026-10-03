/* 6_generated_code.c
 *
 * Button LED debounce controller implementation.
 *
 * This module implements a debounced push button that toggles an LED when an
 * accepted release-to-press transition is detected. All state is file-scope
 * static and is never accessed directly by callers.
 */
#include "6_generated_code.h"

#ifndef BUTTON_DEBOUNCE_MS
#define BUTTON_DEBOUNCE_MS 20u
#endif

static bool initialized = false;
static bool led_on = false;
static bool accepted_button = false;
static bool candidate_button = false;
static uint32_t candidate_start_ms = 0;

void button_led_init(uint32_t now_ms) {
  initialized = true;
  led_on = false;
  accepted_button = false;
  candidate_button = false;
  candidate_start_ms = now_ms;

  board_led_write(false);
}

void button_led_poll(uint32_t now_ms) {
  if (!initialized) {
    return;
  }

  bool sample = board_button_read();

  if (sample != candidate_button) {
    candidate_button = sample;
    candidate_start_ms = now_ms;
    return;
  }

  if (sample == accepted_button) {
    return;
  }

  uint32_t elapsed_ms = (uint32_t)(now_ms - candidate_start_ms);
  if (elapsed_ms < BUTTON_DEBOUNCE_MS) {
    return;
  }

  accepted_button = sample;

  if (sample) {
    led_on = !led_on;
    board_led_write(led_on);
  }
}

bool button_led_is_on(void) {
  return initialized && led_on;
}

bool button_led_is_pressed(void) {
  return initialized && accepted_button;
}

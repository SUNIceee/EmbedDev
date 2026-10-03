/* 6_generated_code.c */
#include "6_generated_code.h"

#include <stdbool.h>
#include <stdint.h>

/* Board platform functions provided by the BSP. */
extern bool board_button_read(void);
extern void board_led_write(bool on);

static bool initialized = false;
static bool led_on = false;
static bool accepted_pressed = false;
static bool candidate_pressed = false;
static uint32_t candidate_start_ms = 0;

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

  const bool sample = board_button_read();

  if (sample != candidate_pressed) {
    candidate_pressed = sample;
    candidate_start_ms = now_ms;
  }

  if ((uint32_t)(now_ms - candidate_start_ms) >= BUTTON_DEBOUNCE_MS) {
    if (candidate_pressed != accepted_pressed) {
      accepted_pressed = candidate_pressed;

      if (accepted_pressed) {
        led_on = !led_on;
        board_led_write(led_on);
      }
    }
  }
}

bool button_led_is_on(void) {
  return led_on;
}

bool button_led_is_pressed(void) {
  return accepted_pressed;
}

/* 6_generated_code.c */
#include "6_generated_code.h"

/* External platform callback implemented by the integration environment. */
extern void board_gpio_write(uint8_t channel, bool on);

/* File-scoped controller state. */
static bool initialized = false;
static bool state[2] = {false, false};

void gpio_output_init(void) {
  state[0] = false;
  state[1] = false;
  board_gpio_write(0, false);
  board_gpio_write(1, false);
  initialized = true;
}

bool gpio_output_set(uint8_t channel, bool on) {
  if (!initialized) {
    return false;
  }
  if ((channel != 0U) && (channel != 1U)) {
    return false;
  }

  state[channel] = on;
  board_gpio_write(channel, on);
  return true;
}

bool gpio_output_toggle(uint8_t channel) {
  if (!initialized) {
    return false;
  }
  if ((channel != 0U) && (channel != 1U)) {
    return false;
  }

  state[channel] = !state[channel];
  board_gpio_write(channel, state[channel]);
  return true;
}

bool gpio_output_get(uint8_t channel, bool *out_on) {
  if (!initialized) {
    return false;
  }
  if ((channel != 0U) && (channel != 1U)) {
    return false;
  }
  if (out_on == NULL) {
    return false;
  }

  *out_on = state[channel];
  return true;
}

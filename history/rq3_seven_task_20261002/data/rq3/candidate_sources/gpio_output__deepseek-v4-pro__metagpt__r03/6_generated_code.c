/* 6_generated_code.c
 *
 * Implements a minimal GPIO output controller for two logical LED outputs.
 * The platform provides board_gpio_write(), which is synchronous and has no
 * error return value.
 */
#include "6_generated_code.h"

#include <stdbool.h>
#include <stdint.h>

extern void board_gpio_write(uint8_t channel, bool on);

static bool initialized = false;
static bool channel_state[2] = {false, false};

static bool gpio_output_is_valid_channel(uint8_t channel) {
  return channel < 2u;
}

void gpio_output_init(void) {
  initialized = true;
  channel_state[0] = false;
  channel_state[1] = false;

  /* Always write both initial levels exactly once per init call. */
  board_gpio_write(0u, false);
  board_gpio_write(1u, false);
}

bool gpio_output_set(uint8_t channel, bool on) {
  if (!initialized) {
    return false;
  }
  if (!gpio_output_is_valid_channel(channel)) {
    return false;
  }

  channel_state[channel] = on;
  board_gpio_write(channel, on);
  return true;
}

bool gpio_output_toggle(uint8_t channel) {
  if (!initialized) {
    return false;
  }
  if (!gpio_output_is_valid_channel(channel)) {
    return false;
  }

  channel_state[channel] = !channel_state[channel];
  board_gpio_write(channel, channel_state[channel]);
  return true;
}

bool gpio_output_get(uint8_t channel, bool *out_on) {
  if (!initialized) {
    return false;
  }
  if (!gpio_output_is_valid_channel(channel)) {
    return false;
  }
  if (out_on == NULL) {
    return false;
  }

  *out_on = channel_state[channel];
  return true;
}

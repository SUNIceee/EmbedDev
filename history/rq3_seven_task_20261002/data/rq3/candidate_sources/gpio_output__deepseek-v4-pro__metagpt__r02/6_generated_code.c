/* 6_generated_code.c */
#include "6_generated_code.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Number of valid GPIO channels. */
enum { kGpioChannelCount = 2 };

/* Module state. */
static bool initialized = false;
static bool channel_state[kGpioChannelCount] = { false, false };

static bool IsValidChannel(uint8_t channel) {
  return channel < kGpioChannelCount;
}

void gpio_output_init(void) {
  initialized = true;
  channel_state[0] = false;
  channel_state[1] = false;

  board_gpio_write(0, false);
  board_gpio_write(1, false);
}

bool gpio_output_set(uint8_t channel, bool on) {
  if (!initialized) {
    return false;
  }
  if (!IsValidChannel(channel)) {
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
  if (!IsValidChannel(channel)) {
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
  if (out_on == NULL) {
    return false;
  }
  if (!IsValidChannel(channel)) {
    return false;
  }

  *out_on = channel_state[channel];
  return true;
}

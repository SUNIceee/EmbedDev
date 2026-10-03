#ifndef FSE_FROZEN_API_H
#define FSE_FROZEN_API_H
#define GPIO_OUTPUT_H

#include <stdbool.h>
#include <stdint.h>

void gpio_output_init(void);
bool gpio_output_set(uint8_t channel, bool on);
bool gpio_output_toggle(uint8_t channel);
bool gpio_output_get(uint8_t channel, bool *out_on);

extern void board_gpio_write(uint8_t channel, bool on);

#endif

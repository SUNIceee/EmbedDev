#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdbool.h>
#include <stdint.h>

void board_gpio_write(uint8_t channel, bool on);

bool gpio_output_init(void);
bool gpio_output_set(uint8_t channel, bool on);
bool gpio_output_toggle(uint8_t channel);
bool gpio_output_get(uint8_t channel, bool *out_on);

#endif

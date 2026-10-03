#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdbool.h>
#include <stdint.h>

#define BUTTON_DEBOUNCE_MS 20u

void button_led_init(uint32_t now_ms);
void button_led_poll(uint32_t now_ms);
bool button_led_is_on(void);
bool button_led_is_pressed(void);

bool board_button_read(void);
void board_led_write(bool on);

#endif

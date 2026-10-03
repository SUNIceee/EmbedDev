#ifndef GPIO_BUTTON_DEBOUNCE_H
#define GPIO_BUTTON_DEBOUNCE_H

#include <stdbool.h>
#include <stdint.h>

#define BUTTON_DEBOUNCE_MS UINT32_C(20)

void button_led_init(uint32_t now_ms);
void button_led_poll(uint32_t now_ms);
bool button_led_is_on(void);
bool button_led_is_pressed(void);

/* Platform-supplied callbacks; do not implement in generated code. */
extern bool board_button_read(void);
extern void board_led_write(bool on);

#endif

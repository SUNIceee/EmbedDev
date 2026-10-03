#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdbool.h>
#include <stdint.h>

#define BUTTON_DEBOUNCE_MS 20

void button_led_init(uint32_t now_ms);
void button_led_poll(uint32_t now_ms);
bool button_led_is_on(void);
bool button_led_is_pressed(void);

/* Platform callbacks supplied by the environment. */
bool board_button_read(void);
void board_led_write(bool on);

#endif /* GENERATED_CODE_H */

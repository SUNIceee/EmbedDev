/* 6_generated_code.h */
#ifndef BUTTON_LED_GENERATED_H_
#define BUTTON_LED_GENERATED_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Debounce interval in milliseconds. Candidate changes must be stable
 * for at least this duration before being accepted. */
#define BUTTON_DEBOUNCE_MS ((uint32_t)20u)

/* Initializes the button LED controller state and writes the LED off once. */
void button_led_init(uint32_t now_ms);

/* Samples the platform button exactly once, debounces the input, and
 * toggles the LED only on an accepted release-to-press transition. */
void button_led_poll(uint32_t now_ms);

/* Returns the stored LED output state without accessing the platform. */
bool button_led_is_on(void);

/* Returns the stored accepted button state without accessing the platform. */
bool button_led_is_pressed(void);

/* Platform callbacks. These must be provided by the target system. */
extern bool board_button_read(void);
extern void board_led_write(bool on);

#ifdef __cplusplus
}
#endif

#endif  /* BUTTON_LED_GENERATED_H_ */

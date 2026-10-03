/* 6_generated_code.h
 *
 * Public interface for the debounced button/LED module.
 *
 * This header is intentionally compatible with the frozen fixed API.
 * It uses the same include guard as the original specification so it can be
 * included more than once safely and in any order with other headers.
 */
#ifndef GPIO_BUTTON_DEBOUNCE_H
#define GPIO_BUTTON_DEBOUNCE_H

#include <stdbool.h>
#include <stdint.h>

/* Minimum time in milliseconds that a button sample must remain stable
 * before it is accepted into the debounced state. */
#define BUTTON_DEBOUNCE_MS UINT32_C(20)

/* Initialize the button/LED module.
 *
 * Parameters:
 *   now_ms - current timestamp in milliseconds.
 *
 * This function must be called before button_led_poll().
 * It resets all internal state, writes the LED off exactly once, and never
 * reads the button. */
void button_led_init(uint32_t now_ms);

/* Poll the button and update the debounced state and LED accordingly.
 *
 * Parameters:
 *   now_ms - current timestamp in milliseconds.
 *
 * If initialization has not occurred, this function returns immediately
 * without accessing the board I/O. */
void button_led_poll(uint32_t now_ms);

/* Query the current stored LED state.
 *
 * Returns:
 *   true if the LED is on, false otherwise.
 *
 * This function is side-effect-free and performs no platform calls. */
bool button_led_is_on(void);

/* Query the debounced button press state.
 *
 * Returns:
 *   true if the button is currently considered pressed, false otherwise.
 *
 * This function is side-effect-free and performs no platform calls. */
bool button_led_is_pressed(void);

#endif /* GPIO_BUTTON_DEBOUNCE_H */

/* 6_generated_code.h
 *
 * Fixed public API header for the button/led controller.
 *
 * This header declares the platform callbacks that must be provided by the
 * board support code and the controller API used by the application.
 * The controller itself is implemented in 6_generated_code.c.
 */

#ifndef SIX_GENERATED_CODE_H_
#define SIX_GENERATED_CODE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Button debounce time in milliseconds.
 *
 * A candidate button level is accepted only after it has been observed for at
 * least this duration. The value is expressed as a uint32_t constant.
 */
#define BUTTON_DEBOUNCE_MS UINT32_C(20)

/**
 * Initializes the button/led controller.
 *
 * The function must be called once before any other controller function
 * (except button_led_is_on and button_led_is_pressed, which may be called at
 * any time). It sets all internal state to the released/off condition and
 * turns the LED off by calling board_led_write(false). It does not read the
 * button.
 *
 * @param now_ms Current monotonic time in milliseconds.
 */
void button_led_init(uint32_t now_ms);

/**
 * Polls the button and updates the LED if necessary.
 *
 * This non-blocking function reads the current button state using
 * board_button_read(), performs debouncing, and toggles the LED on an accepted
 * released-to-pressed transition. It must be called periodically.
 *
 * @param now_ms Current monotonic time in milliseconds. Must be monotonic.
 */
void button_led_poll(uint32_t now_ms);

/**
 * Returns the current LED state.
 *
 * @return true if the LED is on, false otherwise.
 */
bool button_led_is_on(void);

/**
 * Returns the accepted (debounced) button state.
 *
 * @return true if the button is currently considered pressed, false otherwise.
 */
bool button_led_is_pressed(void);

/**
 * Platform callback: read the raw button state.
 *
 * The implementation is platform-specific and must be provided externally.
 *
 * @return true if the button is physically pressed, false otherwise.
 */
extern bool board_button_read(void);

/**
 * Platform callback: set the LED state.
 *
 * The implementation is platform-specific and must be provided externally.
 *
 * @param on true to turn the LED on, false to turn it off.
 */
extern void board_led_write(bool on);

#ifdef __cplusplus
}
#endif

#endif /* SIX_GENERATED_CODE_H_ */

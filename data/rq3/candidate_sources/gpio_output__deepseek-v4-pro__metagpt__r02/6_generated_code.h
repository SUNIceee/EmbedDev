#ifndef SIX_GENERATED_CODE_H_
#define SIX_GENERATED_CODE_H_

/* Fixed public API for the GPIO output controller.
 *
 * The controller manages two GPIO output channels, channel 0 and channel 1.
 * All functions are side-effect free with respect to the internal state except
 * gpio_output_init, gpio_output_set, and gpio_output_toggle as described in
 * the implementation. board_gpio_write is supplied by the platform.
 */
#include <stdbool.h>
#include <stdint.h>

/* Initializes the GPIO output controller.
 *
 * Initialization is idempotent and always forces both channels off by calling
 * board_gpio_write(channel, false) for channels 0 and 1.
 */
void gpio_output_init(void);

/* Sets a GPIO channel to the requested state.
 *
 * Returns true on success, false if the controller has not been initialized,
 * the channel is invalid, or the board write cannot be performed. A valid
 * call writes the requested state to the board even if the state is unchanged.
 */
bool gpio_output_set(uint8_t channel, bool on);

/* Toggles a GPIO channel.
 *
 * Returns true on success, false if the controller has not been initialized
 * or the channel is invalid. A successful toggle updates the internal state
 * and writes the new state to the board.
 */
bool gpio_output_toggle(uint8_t channel);

/* Reads the current state of a GPIO channel.
 *
 * If out_on is non-NULL and the call succeeds, *out_on is set to the current
 * state. Returns true on success, false if the controller has not been
 * initialized, the channel is invalid, or out_on is NULL.
 */
bool gpio_output_get(uint8_t channel, bool *out_on);

/* Platform-supplied GPIO write function.
 *
 * This function must be provided by the platform integration layer. It is
 * declared here so the generated controller can call it.
 */
extern void board_gpio_write(uint8_t channel, bool on);

#endif  /* SIX_GENERATED_CODE_H_ */

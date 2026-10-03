/* 6_generated_code.h
 *
 * Public API for the GPIO output controller.
 * This header defines the interface and is intentionally free of
 * implementation details. The controller is designed to manage two
 * logical GPIO output channels (0 and 1) in a single-threaded
 * execution context.
 */
#ifndef SIX_GENERATED_CODE_H
#define SIX_GENERATED_CODE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the GPIO output controller.
 *
 * Sets both logical channel states to false and writes false to both
 * physical channels. Marks the controller as initialized.
 */
void gpio_output_init(void);

/**
 * @brief Sets a GPIO output channel to a defined state.
 *
 * @param channel GPIO output channel, must be 0 or 1.
 * @param on      Requested output state.
 * @return true if the operation succeeded, false if the channel is invalid
 *         or the controller has not been initialized.
 */
bool gpio_output_set(uint8_t channel, bool on);

/**
 * @brief Toggles a GPIO output channel.
 *
 * @param channel GPIO output channel, must be 0 or 1.
 * @return true if the operation succeeded, false if the channel is invalid
 *         or the controller has not been initialized.
 */
bool gpio_output_toggle(uint8_t channel);

/**
 * @brief Reads the current logical state of a GPIO output channel.
 *
 * @param channel GPIO output channel, must be 0 or 1.
 * @param out_on  Pointer to the variable that receives the current state.
 *                Must not be NULL.
 * @return true if the operation succeeded, false if the channel is invalid,
 *         the controller has not been initialized, or out_on is NULL.
 */
bool gpio_output_get(uint8_t channel, bool *out_on);

#ifdef __cplusplus
}
#endif

#endif /* SIX_GENERATED_CODE_H */

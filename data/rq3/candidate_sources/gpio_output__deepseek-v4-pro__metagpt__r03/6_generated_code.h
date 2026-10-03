/* 6_generated_code.h */
#ifndef SIX_GENERATED_CODE_H_
#define SIX_GENERATED_CODE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the GPIO output controller.
 *
 * Resets both logical LED states to off and writes board_gpio_write(0, false)
 * then board_gpio_write(1, false) exactly once per call.
 */
void gpio_output_init(void);

/**
 * @brief Sets a GPIO output channel to a given logical state.
 *
 * @param channel GPIO channel, must be 0 or 1.
 * @param on      Desired logical state.
 * @return true if the operation succeeded, false otherwise.
 */
bool gpio_output_set(uint8_t channel, bool on);

/**
 * @brief Toggles a GPIO output channel.
 *
 * @param channel GPIO channel, must be 0 or 1.
 * @return true if the operation succeeded, false otherwise.
 */
bool gpio_output_toggle(uint8_t channel);

/**
 * @brief Reads the current logical state of a GPIO output channel.
 *
 * @param channel GPIO channel, must be 0 or 1.
 * @param out_on  Non-null pointer where the current logical state is stored.
 * @return true if the query succeeded, false otherwise.
 */
bool gpio_output_get(uint8_t channel, bool *out_on);

#ifdef __cplusplus
}
#endif

#endif /* SIX_GENERATED_CODE_H_ */

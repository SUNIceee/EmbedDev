/* include/disco_hw_interface.h */
#ifndef DISCO_HW_INTERFACE_H_
#define DISCO_HW_INTERFACE_H_

#include <stdint.h>
#include <stdbool.h>
#include "disco_core.h"

/**
 * @brief Hardware abstraction layer for the DiscoBot system.
 * This structure decouples business logic from specific hardware peripherals.
 */

typedef struct {
    /**
     * @brief Reads accelerometer data from hardware.
     * @return 16-bit signed accelerometer value.
     */
    int16_t (*read_accel)(void);

    /**
     * @brief Generates or retrieves entropy/random data for system use.
     * @return 32-bit unsigned entropy value.
     */
    uint32_t (*get_entropy)(void);

    /**
     * @brief Sets the PWM duty cycle for motor control.
     * @param duty The duty cycle value (0-255).
     * @return true if operation was successful, false otherwise.
     */
    bool (*set_pwm_duty)(uint8_t duty);

    /**
     * @brief Writes a state to a generic GPIO pin (e.g., for status LEDs or debug).
     * @param pin_id The ID of the hardware pin.
     * @param state The state to write (true for HIGH, false for LOW).
     * @return true if successful.
     */
    bool (*write_gpio)(uint8_t pin_id, bool state);
} Hardware_Interface;

/**
 * @brief Global pointer to the current hardware implementation.
 * Injected during system initialization via dependency injection.
 */
extern const Hardware_Interface *const g_hw_api;

#endif /* DISCO_HW_INTERFACE_H_ */

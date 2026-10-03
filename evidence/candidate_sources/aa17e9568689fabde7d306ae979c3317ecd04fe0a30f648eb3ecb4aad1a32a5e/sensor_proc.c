/*
 * src/sensor_proc.c
 * Implementation of sensor processing logic decoupled from hardware.
 */

#include "disco_core.h"
#include "disco_hw_interface.h"
#include <stddef.h>

/**
 * @brief Internal reference to the injected hardware abstraction.
 * This is initialized during system startup to point to either
 * real HAL implementations or Mock implementations.
 */
static const Hardware_Interface* _hw_if = NULL;

/**
 * @brief Initialize the sensor processing module with a specific hardware interface.
 * 
 * @param hw_interface Pointer to the hardware abstraction layer (Target or Mock).
 */
void sensor_proc_init(const Hardware_Interface* hw_interface) {
    if (hw_interface != NULL) {
        _hw_if = hw_interface;
    }
}

/**
 * @brief Read accelerometer data through the abstracted interface.
 * 
 * @return int16_t The processed accelerometer value. 
 *         Returns 0 if the interface is not initialized.
 */
int16_t sensor_read_accel(void) {
    if (_hw_if == NULL || _hw_if->read_accel == NULL) {
        return 0;
    }
    return _hw_if->read_accel();
}

/**
 * @brief Retrieve entropy/randomness data from the hardware entropy source.
 * 
 * @return uint32_t The generated entropy value.
 *         Returns 0 if the interface is not initialized.
 */
uint32_t sensor_get_entropy(void) {
    if (_hw_if == NULL || _hw_if->get_entropy == NULL) {
        return 0;
    }
    return _hw_if->get_entropy();
}

/**
 * @brief Example of processed logic: checking sensor health status.
 * This function demonstrates how higher-level logic consumes the hardware interface.
 * 
 * @return bool True if sensors are responding within nominal expected values.
 */
bool sensor_is_healthy(void) {
    if (_hw_if == NULL) {
        return false;
    }
    
    /* Logic: Accelerometer should not return absolute zero in a dynamic environment */
    int16_t accel_val = sensor_read_accel();
    
    /* Simple health check logic */
    if (accel_val == -32768) {
        return false;
    }
    
    return true;
}

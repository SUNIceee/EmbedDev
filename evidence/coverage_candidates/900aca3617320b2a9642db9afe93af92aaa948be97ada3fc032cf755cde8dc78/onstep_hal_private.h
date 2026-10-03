/* Internal host-test adapter extensions. These are not part of the frozen public API. */

#ifndef ONSTEP_HAL_PRIVATE_H
#define ONSTEP_HAL_PRIVATE_H

#include "6_generated_code.h"

/* Adds signed microsteps to a virtual motor axis. Used by the domain loop to
   advance simulated Goto, manual motion, and tracking in deterministic tests. */
os_error_t os_hal_motor_advance(uint8_t axis, int32_t microsteps);

/* Test-only helper: erases the host NVM image. It must be called explicitly
   before os_init() when a blank NVM is required. Normal startup does not call it. */
os_error_t os_hal_nvm_erase_all(void);

#endif

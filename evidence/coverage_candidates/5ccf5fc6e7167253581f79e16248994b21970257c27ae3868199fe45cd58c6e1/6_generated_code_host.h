/* Non-frozen host/test instrumentation extension.
 * This header is separate from the frozen public API and allows a host test
 * harness to inject commands, read replies, set motor positions/limits/GPS/RTC,
 * and inject NVM faults without depending on real board hardware.
 */
#ifndef FSE_FROZEN_API_TEST_HOST_H
#define FSE_FROZEN_API_TEST_HOST_H

#include "6_generated_code.h"

os_error_t os_host_comm_rx_inject(uint8_t channel, const char *data, size_t length);
os_error_t os_host_comm_rx_clear(uint8_t channel);
os_error_t os_host_comm_tx_read(uint8_t channel, char *buffer, size_t buffer_size, size_t *length);
os_error_t os_host_comm_tx_clear(uint8_t channel);
os_error_t os_host_motor_set_position(uint8_t axis, int32_t position_steps);
os_error_t os_host_limit_set(uint8_t axis, bool triggered);
os_error_t os_host_gps_set(const os_site_info_t *site);
os_error_t os_host_gps_clear(void);
os_error_t os_host_nvm_set_fault(bool read_fault, bool write_fault);
os_error_t os_host_nvm_clear_fault(void);

#endif

/*
 * OnStep Default Fallback HAL Adapters Implementation
 */

#include "6_generated_code.h"
#include <string.h>

#if defined(__GNUC__) || defined(__clang__)
#define OS_WEAK __attribute__((weak))
#else
#define OS_WEAK
#endif

static int32_t s_mock_motor_pos[2] = {0, 0};
static uint32_t s_mock_rtc_seconds = 1700000000U;
static uint8_t s_mock_nvm_storage[512] = {0};

OS_WEAK os_error_t os_hal_motor_init(uint8_t axis) {
    return (axis > 1) ? OS_ERR_INVALID_ARGUMENT : OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    return (axis > 1) ? OS_ERR_INVALID_ARGUMENT : OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    return (axis > 1) ? OS_ERR_INVALID_ARGUMENT : OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    return (axis > 1) ? OS_ERR_INVALID_ARGUMENT : OS_ERR_NONE;
}

OS_WEAK int32_t os_hal_motor_get_position(uint8_t axis) {
    return (axis > 1) ? 0 : s_mock_motor_pos[axis];
}

OS_WEAK os_error_t os_hal_gps_init(void) {
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (!site) return OS_ERR_INVALID_ARGUMENT;
    site->valid = false;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_rtc_init(void) {
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (!utc_epoch_seconds) return OS_ERR_INVALID_ARGUMENT;
    *utc_epoch_seconds = s_mock_rtc_seconds;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    s_mock_rtc_seconds = utc_epoch_seconds;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_limit_init(void) {
    return OS_ERR_NONE;
}

OS_WEAK bool os_hal_limit_is_triggered(uint8_t axis) {
    return (axis > 1);
}

OS_WEAK os_error_t os_hal_nvm_init(void) {
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (!data || ((uint32_t)offset + length > sizeof(s_mock_nvm_storage))) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(data, &s_mock_nvm_storage[offset], length);
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (!data || ((uint32_t)offset + length > sizeof(s_mock_nvm_storage))) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(&s_mock_nvm_storage[offset], data, length);
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_comm_init(uint8_t channel) {
    return (channel > OS_CHANNEL_ETHERNET) ? OS_ERR_INVALID_ARGUMENT : OS_ERR_NONE;
}

OS_WEAK int16_t os_hal_comm_available(uint8_t channel) {
    return (channel > OS_CHANNEL_ETHERNET) ? 0 : 0;
}

OS_WEAK char os_hal_comm_read(uint8_t channel) {
    (void)channel;
    return 0;
}

OS_WEAK os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (!data || channel > OS_CHANNEL_ETHERNET) return OS_ERR_INVALID_ARGUMENT;
    (void)length;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) {
    (void)duration_ms;
    (void)count;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_timer_motor_init(void) {
    return OS_ERR_NONE;
}

/* Host-compatible non-blocking device adapter for OnStep domain logic.
   All state is observable and deterministic; no real GPIO/SPI/I2C/network is used. */

#include "onstep_hal_private.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define OS_HOST_NVM_SIZE              4096u
#define OS_HOST_COMM_RX_CAPACITY      256u
#define OS_HOST_COMM_TX_CAPACITY      512u

static bool     s_motor_initialized[2];
static bool     s_motor_enabled[2];
static bool     s_motor_direction[2];
static uint32_t s_motor_frequency[2];
static int32_t  s_motor_position[2];

static os_site_info_t s_gps_data;
static bool           s_gps_valid;

static uint32_t s_rtc_epoch;
static bool     s_rtc_initialized;

static bool s_limit_triggered[2];

static uint8_t s_nvm[OS_HOST_NVM_SIZE];
static bool    s_nvm_initialized;

static uint8_t  s_comm_rx[4][OS_HOST_COMM_RX_CAPACITY];
static uint16_t s_comm_rx_head[4];
static uint16_t s_comm_rx_tail[4];
static uint16_t s_comm_rx_count[4];

static uint8_t  s_comm_tx[4][OS_HOST_COMM_TX_CAPACITY];
static uint16_t s_comm_tx_len[4];
static bool     s_comm_initialized[4];

os_error_t os_hal_motor_init(uint8_t axis)
{
    if (axis > 1u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!s_motor_initialized[axis]) {
        s_motor_position[axis] = 0;
    }
    s_motor_initialized[axis] = true;
    s_motor_enabled[axis] = false;
    s_motor_direction[axis] = false;
    s_motor_frequency[axis] = 0u;

    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz)
{
    if (axis > 1u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_motor_frequency[axis] = frequency_hz;
    if (frequency_hz == 0u) {
        /* No new pulses in this implementation. */
    }

    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward)
{
    if (axis > 1u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_motor_direction[axis] = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable)
{
    if (axis > 1u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_motor_enabled[axis] = enable;
    if (!enable) {
        s_motor_frequency[axis] = 0u;
    }

    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis)
{
    if (axis > 1u) {
        return 0;
    }

    return s_motor_position[axis];
}

os_error_t os_hal_motor_advance(uint8_t axis, int32_t microsteps)
{
    if (axis > 1u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_motor_position[axis] += microsteps;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_init(void)
{
    memset(&s_gps_data, 0, sizeof(s_gps_data));
    s_gps_valid = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!s_gps_valid) {
        memset(site, 0, sizeof(*site));
        site->valid = false;
        return OS_ERR_TIMEOUT;
    }

    *site = s_gps_data;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void)
{
    s_rtc_initialized = true;
    s_rtc_epoch = 0u;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds)
{
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!s_rtc_initialized) {
        return OS_ERR_NOT_SUPPORTED;
    }

    *utc_epoch_seconds = s_rtc_epoch;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds)
{
    s_rtc_epoch = utc_epoch_seconds;
    s_rtc_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void)
{
    s_limit_triggered[0] = false;
    s_limit_triggered[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis)
{
    if (axis > 1u) {
        return true; /* fail-safe for invalid axes */
    }

    return s_limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void)
{
    /* Non-destructive: preserve previously written calibration, PEC, and config data. */
    s_nvm_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_erase_all(void)
{
    memset(s_nvm, 0, sizeof(s_nvm));
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length)
{
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!s_nvm_initialized) {
        return OS_ERR_NVM_FAULT;
    }

    if (((uint32_t)offset + (uint32_t)length) > OS_HOST_NVM_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    (void)memcpy(data, &s_nvm[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length)
{
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!s_nvm_initialized) {
        return OS_ERR_NVM_FAULT;
    }

    if (((uint32_t)offset + (uint32_t)length) > OS_HOST_NVM_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    (void)memcpy(&s_nvm[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel)
{
    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_comm_rx_head[channel] = 0u;
    s_comm_rx_tail[channel] = 0u;
    s_comm_rx_count[channel] = 0u;
    s_comm_tx_len[channel] = 0u;
    s_comm_initialized[channel] = true;

    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel)
{
    if (channel > OS_CHANNEL_ETHERNET || !s_comm_initialized[channel]) {
        return 0;
    }

    return (int16_t)s_comm_rx_count[channel];
}

char os_hal_comm_read(uint8_t channel)
{
    if (channel > OS_CHANNEL_ETHERNET || !s_comm_initialized[channel]) {
        return '\0';
    }

    if (s_comm_rx_count[channel] == 0u) {
        return '\0';
    }

    char value = (char)s_comm_rx[channel][s_comm_rx_tail[channel]];
    s_comm_rx_tail[channel] =
        (uint16_t)((s_comm_rx_tail[channel] + 1u) % OS_HOST_COMM_RX_CAPACITY);
    s_comm_rx_count[channel]--;

    return value;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length)
{
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (channel > OS_CHANNEL_ETHERNET || !s_comm_initialized[channel]) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (length > (OS_HOST_COMM_TX_CAPACITY - s_comm_tx_len[channel])) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (length > 0u) {
        (void)memcpy(&s_comm_tx[channel][s_comm_tx_len[channel]], data, length);
        s_comm_tx_len[channel] = (uint16_t)(s_comm_tx_len[channel] + length);
    }

    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count)
{
    (void)duration_ms;
    (void)count;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void)
{
    return OS_ERR_NONE;
}

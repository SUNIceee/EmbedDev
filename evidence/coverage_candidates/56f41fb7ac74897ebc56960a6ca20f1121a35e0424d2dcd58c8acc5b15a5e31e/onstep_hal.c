/* Deterministic host/board-neutral hardware adapter implementation. */

#include "6_generated_code.h"

#include <string.h>

#define HAL_AXIS_COUNT 2u
#define HAL_CHANNEL_COUNT 4u
#define HAL_NVM_SIZE 2048u
#define HAL_COMM_BUFFER_SIZE 256u
#define HAL_TICK_MILLISECONDS 10u

typedef struct {
    bool initialized;
    bool enabled;
    bool direction_forward;
    uint32_t frequency_hz;
    int32_t position_steps;
    double fractional_steps;
    os_error_t last_error;
} hal_motor_t;

typedef struct {
    char rx[HAL_COMM_BUFFER_SIZE];
    size_t rx_head;
    size_t rx_tail;
    char tx[HAL_COMM_BUFFER_SIZE];
    size_t tx_length;
    bool initialized;
} hal_channel_t;

static hal_motor_t motors[HAL_AXIS_COUNT];
static hal_channel_t channels[HAL_CHANNEL_COUNT];
static uint8_t nvm_image[HAL_NVM_SIZE];

static bool gps_initialized;
static bool rtc_initialized;
static bool limit_initialized;
static bool timer_initialized;

static bool limit_state[HAL_AXIS_COUNT];
static bool nvm_initialized;
static bool nvm_fail_reads;
static bool nvm_fail_writes;

static uint32_t rtc_epoch_seconds;
static os_site_info_t injected_gps_site;
static bool injected_gps_valid;

static void hal_advance_motor(hal_motor_t *motor)
{
    double steps;

    if (motor == NULL || !motor->enabled || motor->frequency_hz == 0u) {
        return;
    }

    steps = ((double)motor->frequency_hz *
             (double)HAL_TICK_MILLISECONDS) / 1000.0;
    steps += motor->fractional_steps;
    motor->fractional_steps = steps - (double)((int32_t)steps);

    if (steps >= 1.0) {
        int32_t whole = (int32_t)steps;
        motor->position_steps += motor->direction_forward ? whole : -whole;
    }
}

static bool hal_valid_axis(uint8_t axis)
{
    return axis < HAL_AXIS_COUNT;
}

static bool hal_valid_channel(uint8_t channel)
{
    return channel < HAL_CHANNEL_COUNT;
}

os_error_t os_hal_motor_init(uint8_t axis)
{
    if (!hal_valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    memset(&motors[axis], 0, sizeof(motors[axis]));
    motors[axis].initialized = true;
    motors[axis].last_error = OS_ERR_NONE;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz)
{
    if (!hal_valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!motors[axis].initialized) {
        return OS_ERR_INVALID_STATE;
    }

    motors[axis].frequency_hz = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward)
{
    if (!hal_valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!motors[axis].initialized) {
        return OS_ERR_INVALID_STATE;
    }

    motors[axis].direction_forward = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable)
{
    if (!hal_valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!motors[axis].initialized) {
        return OS_ERR_INVALID_STATE;
    }

    motors[axis].enabled = enable;

    if (!enable) {
        motors[axis].frequency_hz = 0u;
    }

    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis)
{
    if (!hal_valid_axis(axis)) {
        return 0;
    }

    hal_advance_motor(&motors[axis]);
    return motors[axis].position_steps;
}

os_error_t os_hal_gps_init(void)
{
    gps_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!gps_initialized) {
        site->valid = false;
        return OS_ERR_INVALID_STATE;
    }

    if (!injected_gps_valid) {
        site->valid = false;
        return OS_ERR_GPS_NO_SIGNAL;
    }

    *site = injected_gps_site;
    site->valid = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void)
{
    rtc_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds)
{
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!rtc_initialized) {
        return OS_ERR_INVALID_STATE;
    }

    *utc_epoch_seconds = rtc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds)
{
    if (!rtc_initialized) {
        return OS_ERR_INVALID_STATE;
    }

    rtc_epoch_seconds = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void)
{
    limit_initialized = true;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis)
{
    if (!hal_valid_axis(axis)) {
        return true;
    }

    if (!limit_initialized) {
        return true;
    }

    return limit_state[axis];
}

os_error_t os_hal_nvm_init(void)
{
    if (!nvm_initialized) {
        memset(nvm_image, 0, sizeof(nvm_image));
        nvm_initialized = true;
    }

    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset,
                           uint8_t *data,
                           uint16_t length)
{
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if ((uint32_t)offset + (uint32_t)length > HAL_NVM_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!nvm_initialized) {
        return OS_ERR_INVALID_STATE;
    }

    if (nvm_fail_reads) {
        return OS_ERR_NVM_FAULT;
    }

    memcpy(data, &nvm_image[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset,
                            const uint8_t *data,
                            uint16_t length)
{
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if ((uint32_t)offset + (uint32_t)length > HAL_NVM_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!nvm_initialized) {
        return OS_ERR_INVALID_STATE;
    }

    if (nvm_fail_writes) {
        return OS_ERR_NVM_FAULT;
    }

    memcpy(&nvm_image[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel)
{
    if (!hal_valid_channel(channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    memset(&channels[channel], 0, sizeof(channels[channel]));
    channels[channel].initialized = true;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel)
{
    size_t available;

    if (!hal_valid_channel(channel) || !channels[channel].initialized) {
        return 0;
    }

    if (channels[channel].rx_head >= channels[channel].rx_tail) {
        available = channels[channel].rx_head - channels[channel].rx_tail;
    } else {
        available = HAL_COMM_BUFFER_SIZE -
                    channels[channel].rx_tail +
                    channels[channel].rx_head;
    }

    if (available > 32767u) {
        return 32767;
    }

    return (int16_t)available;
}

char os_hal_comm_read(uint8_t channel)
{
    char value;

    if (!hal_valid_channel(channel) ||
        !channels[channel].initialized ||
        os_hal_comm_available(channel) <= 0) {
        return '\0';
    }

    value = channels[channel].rx[channels[channel].rx_tail];
    channels[channel].rx_tail =
        (channels[channel].rx_tail + 1u) % HAL_COMM_BUFFER_SIZE;
    return value;
}

os_error_t os_hal_comm_write(uint8_t channel,
                             const char *data,
                             size_t length)
{
    size_t room;

    if (!hal_valid_channel(channel) || data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!channels[channel].initialized) {
        return OS_ERR_INVALID_STATE;
    }

    room = HAL_COMM_BUFFER_SIZE - channels[channel].tx_length;

    if (length > room) {
        return OS_ERR_TIMEOUT;
    }

    memcpy(&channels[channel].tx[channels[channel].tx_length], data, length);
    channels[channel].tx_length += length;
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
    timer_initialized = true;
    (void)timer_initialized;
    return OS_ERR_NONE;
}

#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* Constants */
#define OS_AXIS_RA                   0
#define OS_AXIS_AZ                   0
#define OS_AXIS_DEC                  1
#define OS_AXIS_ALT                  1
#define OS_MAX_AXES                  2

#define OS_CHANNEL_USB               0
#define OS_CHANNEL_BLUETOOTH         1
#define OS_CHANNEL_WIFI              2
#define OS_CHANNEL_ETHERNET          3
#define OS_MAX_CHANNELS              4

#define OS_MAX_COMMAND_LENGTH        128
#define OS_MAX_REPLY_LENGTH          128
#define OS_NVM_CALIBRATION_SIZE_BYTES 256
#define OS_NVM_CONFIG_SIZE_BYTES      256

/* Enums */
typedef enum {
    OS_ERR_NONE = 0,
    OS_ERR_INVALID_ARGUMENT,
    OS_ERR_INVALID_STATE,
    OS_ERR_COMMAND_FORMAT,
    OS_ERR_NOT_SUPPORTED,
    OS_ERR_TIMEOUT,
    OS_ERR_HARDWARE
} os_error_t;

typedef enum {
    OS_STATE_INIT = 0,
    OS_STATE_IDLE_TRACKING,
    OS_STATE_GOTO,
    OS_STATE_CALIBRATION,
    OS_STATE_MANUAL_MOTION,
    OS_STATE_PARKED,
    OS_STATE_FAULT
} os_state_t;

typedef enum {
    OS_TRACK_SIDEREAL = 0,
    OS_TRACK_LUNAR,
    OS_TRACK_SOLAR,
    OS_TRACK_CUSTOM
} os_tracking_rate_t;

typedef enum {
    OS_GUIDE_EAST = 0,
    OS_GUIDE_WEST,
    OS_GUIDE_NORTH,
    OS_GUIDE_SOUTH
} os_guide_direction_t;

typedef enum {
    OS_ALIGN_1_STAR = 1,
    OS_ALIGN_2_STAR = 2,
    OS_ALIGN_3_STAR = 3,
    OS_ALIGN_N_STAR = 4
} os_align_mode_t;

typedef enum {
    OS_DIR_POSITIVE = 0,
    OS_DIR_NEGATIVE = 1
} os_direction_t;

typedef enum {
    OS_SPEED_GUIDE = 0,
    OS_SPEED_CENTER,
    OS_SPEED_FIND,
    OS_SPEED_MAX
} os_speed_rate_t;

/* Structures */
typedef struct {
    double latitude_degrees;
    double longitude_degrees;
    double elevation_metres;
    uint32_t utc_epoch_seconds;
    bool valid;
} os_site_info_t;

typedef struct {
    os_state_t state;
    bool moving;
    bool gps_locked;
    double current_ra_hours;
    double current_dec_degrees;
    int32_t motor_steps[OS_MAX_AXES];
} os_system_status_t;

/* HAL Hardware Abstraction Layer API */
void os_hal_motor_init(uint8_t axis);
void os_hal_motor_set_frequency(uint8_t axis, double frequency_hz);
void os_hal_motor_set_direction(uint8_t axis, bool forward);
void os_hal_motor_enable(uint8_t axis, bool enable);
int32_t os_hal_motor_get_position(uint8_t axis);
void os_hal_timer_motor_init(void);

void os_hal_gps_init(void);
bool os_hal_gps_poll(os_site_info_t *site);

void os_hal_rtc_init(void);
bool os_hal_rtc_read(uint32_t *utc_epoch_seconds);
bool os_hal_rtc_set(uint32_t utc_epoch_seconds);

void os_hal_limit_init(void);
bool os_hal_limit_is_triggered(uint8_t axis);

void os_hal_buzzer_beep(uint32_t duration_ms, uint32_t count);

void os_hal_comm_init(uint8_t channel);
size_t os_hal_comm_available(uint8_t channel);
int os_hal_comm_read(uint8_t channel);
void os_hal_comm_write(uint8_t channel, const char *data, size_t length);

void os_hal_nvm_init(void);
bool os_hal_nvm_read(uint32_t offset, void *data, size_t length);
bool os_hal_nvm_write(uint32_t offset, const void *data, size_t length);

/* Public Core Software API */
os_error_t os_init(void);
os_error_t os_loop_iteration(void);
os_error_t os_goto_equatorial(double ra_hours, double dec_degrees);
os_error_t os_goto_abort(void);
os_error_t os_set_tracking_rate(os_tracking_rate_t rate);
os_error_t os_set_custom_tracking_rate(double rate_hz);
os_error_t os_guide_pulse(os_guide_direction_t direction, uint32_t duration_ms);
os_error_t os_align_add_star(double ra_hours, double dec_degrees);
os_error_t os_align_compute(os_align_mode_t mode);
os_error_t os_park(void);
os_error_t os_unpark(void);
os_error_t os_manual_move(uint8_t axis, os_direction_t dir, os_speed_rate_t speed);
os_error_t os_manual_stop(uint8_t axis);
os_error_t os_query_is_moving(bool *is_moving);
os_error_t os_get_status(os_system_status_t *status);
os_error_t os_pec_set_correction(double worm_phase_deg, double arcsec_error);
os_error_t os_pec_enable(bool enable);
os_error_t os_process_command(uint8_t channel, const char *cmd, char *reply, size_t reply_max_len);

/* Mock Helpers for Host Test Suites */
void mock_set_motor_fault(uint8_t axis, bool fault);
void mock_set_limit_triggered(uint8_t axis, bool triggered);
void mock_inject_gps(const os_site_info_t *site);
void mock_inject_comm_rx(uint8_t channel, const char *data, size_t length);
size_t mock_get_comm_tx(uint8_t channel, char *buffer, size_t max_len);

#ifdef __cplusplus
}
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#endif /* GENERATED_CODE_H */

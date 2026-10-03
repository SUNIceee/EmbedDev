#ifndef FSE_FROZEN_API_H
#define FSE_FROZEN_API_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define OS_RA_MIN_HOURS         0.0f
#define OS_RA_MAX_HOURS        24.0f
#define OS_DEC_MIN_DEG        -90.0f
#define OS_DEC_MAX_DEG         90.0f
#define OS_SIDEREAL_RATE_ARCSEC_PER_SEC  15.041067f
#define OS_LUNAR_RATE_FACTOR      0.966f
#define OS_SOLAR_RATE_FACTOR      0.9973f
#define OS_GUIDE_RATE_MIN         0.1f
#define OS_GUIDE_RATE_MAX         1.0f
#define OS_GOTO_SPEED_MAX_DEG_PER_SEC  3.0f
#define OS_CALIBRATION_MIN_STARS     1
#define OS_CALIBRATION_RECOMMENDED   3
#define OS_CALIBRATION_MAX_STARS     9
#define OS_FIRMWARE_VERSION_MAJOR    1
#define OS_FIRMWARE_VERSION_MINOR    0
#define OS_FIRMWARE_VERSION_PATCH    0
#define OS_NVM_CALIBRATION_SIZE_BYTES   256
#define OS_NVM_CONFIG_SIZE_BYTES        256
#define OS_LX200_CMD_PREFIX    ':'
#define OS_LX200_CMD_SUFFIX    '#'
#define OS_MAX_COMMAND_LENGTH      64
#define OS_MAX_REPLY_LENGTH       128
#define OS_PEC_TABLE_SIZE          360
#define OS_CHANNEL_USB        0
#define OS_CHANNEL_BLUETOOTH  1
#define OS_CHANNEL_WIFI       2
#define OS_CHANNEL_ETHERNET   3
typedef enum {
    OS_STATE_INITIALIZING = 0,
    OS_STATE_IDLE_TRACKING,
    OS_STATE_GOTO,
    OS_STATE_ALIGNMENT,
    OS_STATE_MANUAL_MOTION,
    OS_STATE_PARKED,
    OS_STATE_FAULT
} os_state_t;
typedef enum {
    OS_ERR_NONE                 =  0,
    OS_ERR_INVALID_ARGUMENT     = -1,
    OS_ERR_INVALID_STATE        = -2,
    OS_ERR_TIMEOUT              = -3,
    OS_ERR_LIMIT_TRIGGERED      = -4,
    OS_ERR_MOTOR_DRIVER_FAULT   = -5,
    OS_ERR_GPS_NO_SIGNAL        = -6,
    OS_ERR_CALIBRATION_FAILED   = -7,
    OS_ERR_NVM_FAULT            = -8,
    OS_ERR_COMMAND_FORMAT       = -9,
    OS_ERR_NOT_SUPPORTED        = -10,
    OS_ERR_WATCHDOG_RESET       = -11
} os_error_t;
typedef enum {
    OS_TRACK_RATE_SIDEREAL = 0,
    OS_TRACK_RATE_LUNAR,
    OS_TRACK_RATE_SOLAR,
    OS_TRACK_RATE_CUSTOM
} os_track_rate_t;
typedef enum {
    OS_ALIGN_1STAR  = 1,
    OS_ALIGN_2STAR  = 2,
    OS_ALIGN_3STAR  = 3,
    OS_ALIGN_NSTAR  = 4
} os_align_mode_t;
typedef enum {
    OS_MOUNT_EQUATORIAL = 0,
    OS_MOUNT_ALTAZ      = 1
} os_mount_type_t;
typedef enum {
    OS_DIRECTION_NORTH = 0,
    OS_DIRECTION_SOUTH,
    OS_DIRECTION_EAST,
    OS_DIRECTION_WEST
} os_direction_t;
typedef enum {
    OS_SPEED_SLOW    = 0,
    OS_SPEED_MEDIUM  = 1,
    OS_SPEED_FAST    = 2,
    OS_SPEED_CUSTOM  = 3
} os_speed_level_t;
typedef struct {
    float ra_hours;
    float dec_degrees;
} os_equatorial_coord_t;
typedef struct {
    float azimuth_degrees;
    float altitude_degrees;
} os_horizontal_coord_t;
typedef struct {
    float    latitude_degrees;
    float    longitude_degrees;
    float    elevation_metres;
    uint32_t utc_epoch_seconds;
    bool     valid;
} os_site_info_t;
typedef struct {
    int32_t ra_steps;
    int32_t dec_steps;
} os_motor_position_t;
typedef struct {
    float matrix_ra_to_ra;
    float matrix_ra_to_dec;
    float matrix_dec_to_ra;
    float matrix_dec_to_dec;
    float offset_ra_arcsec;
    float offset_dec_arcsec;
    bool  valid;
} os_calibration_t;
typedef struct {
    bool     active;
    uint32_t duration_ms;
    float    rate_fraction;
    bool     direction_east;
    bool     direction_north;
    bool     dec_priority;
} os_guide_pulse_t;
typedef struct {
    int16_t corrections[360];
    bool    valid;
} os_pec_table_t;
os_error_t os_init(void);
void os_loop_iteration(void);
os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length);
os_error_t os_goto_equatorial(os_equatorial_coord_t target);
os_error_t os_goto_horizontal(os_horizontal_coord_t target);
os_error_t os_goto_abort(void);
os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor);
os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor);
os_error_t os_tracking_enable(void);
os_error_t os_tracking_disable(void);
os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms);
os_error_t os_guide_set_rate(float rate_fraction);
os_error_t os_guide_get_state(os_guide_pulse_t *pulse);
os_error_t os_align_begin(os_align_mode_t mode);
os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos);
os_error_t os_align_compute(void);
os_error_t os_align_get_residual(float *residual_arcsec);
os_error_t os_align_abort(void);
os_error_t os_park(void);
os_error_t os_unpark(void);
os_error_t os_park_set_position(os_equatorial_coord_t park_pos);
os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed);
os_error_t os_move_stop(void);
os_error_t os_move_set_custom_speed(float arcsec_per_sec);
os_error_t os_query_state(os_state_t *state);
os_error_t os_query_coordinates(os_equatorial_coord_t *coord);
os_error_t os_query_site(os_site_info_t *site);
os_error_t os_query_motor_position(os_motor_position_t *pos);
os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor, uint8_t *patch);
os_error_t os_query_is_moving(bool *moving);
os_error_t os_query_gps_locked(bool *locked);
os_error_t os_pec_enable(bool enable);
os_error_t os_pec_load_table(const os_pec_table_t *table);
os_error_t os_pec_get_table(os_pec_table_t *table);
os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec);
os_error_t os_calibration_get(os_calibration_t *calib);
os_error_t os_calibration_clear(void);
os_error_t os_hal_motor_init(uint8_t axis);
os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz);
os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward);
os_error_t os_hal_motor_enable(uint8_t axis, bool enable);
int32_t os_hal_motor_get_position(uint8_t axis);
os_error_t os_hal_gps_init(void);
os_error_t os_hal_gps_poll(os_site_info_t *site);
os_error_t os_hal_rtc_init(void);
os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds);
os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds);
os_error_t os_hal_limit_init(void);
bool os_hal_limit_is_triggered(uint8_t axis);
os_error_t os_hal_nvm_init(void);
os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length);
os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length);
os_error_t os_hal_comm_init(uint8_t channel);
int16_t os_hal_comm_available(uint8_t channel);
char os_hal_comm_read(uint8_t channel);
os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length);
os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count);
os_error_t os_hal_timer_motor_init(void);
#endif

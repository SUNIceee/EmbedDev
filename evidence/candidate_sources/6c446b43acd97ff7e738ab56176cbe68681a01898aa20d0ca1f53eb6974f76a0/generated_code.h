#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OS_AXIS_RA 0
#define OS_AXIS_DEC 1
#define OS_AXIS_COUNT 2

#define OS_CHANNEL_USB 0
#define OS_CHANNEL_BLUETOOTH 1
#define OS_CHANNEL_WIFI 2
#define OS_CHANNEL_ETHERNET 3
#define OS_CHANNEL_COUNT 4

#define OS_MAX_COMMAND_LENGTH 64
#define OS_MAX_REPLY_LENGTH 96
#define OS_MAX_ALIGN_STARS 8
#define OS_PEC_TABLE_SIZE 36
#define OS_NVM_CALIBRATION_OFFSET 0u
#define OS_NVM_CALIBRATION_SIZE_BYTES 512u
#define OS_NVM_CONFIG_OFFSET 512u
#define OS_NVM_CONFIG_SIZE_BYTES 512u

typedef enum {
    OS_ERR_NONE = 0,
    OS_ERR_INVALID_ARGUMENT = -1,
    OS_ERR_INVALID_STATE = -2,
    OS_ERR_COMMAND_FORMAT = -3,
    OS_ERR_NOT_SUPPORTED = -4,
    OS_ERR_TIMEOUT = -5,
    OS_ERR_HARDWARE = -6,
    OS_ERR_LIMIT_TRIGGERED = -7,
    OS_ERR_NO_MEMORY = -8,
    OS_ERR_CHECKSUM = -9
} os_error_t;

typedef enum {
    OS_STATE_INIT = 0,
    OS_STATE_IDLE_TRACKING,
    OS_STATE_GOTO,
    OS_STATE_ALIGNING,
    OS_STATE_MANUAL_MOTION,
    OS_STATE_PARKED,
    OS_STATE_FAULT
} os_state_t;

typedef enum {
    OS_MOUNT_EQUATORIAL = 0,
    OS_MOUNT_ALTAZ = 1
} os_mount_mode_t;

typedef enum {
    OS_TRACK_SIDEREAL = 0,
    OS_TRACK_LUNAR,
    OS_TRACK_SOLAR,
    OS_TRACK_CUSTOM
} os_tracking_rate_t;

typedef enum {
    OS_DIR_EAST = 0,
    OS_DIR_WEST,
    OS_DIR_NORTH,
    OS_DIR_SOUTH
} os_direction_t;

typedef enum {
    OS_ALIGN_ONE_STAR = 1,
    OS_ALIGN_TWO_STAR = 2,
    OS_ALIGN_THREE_STAR = 3,
    OS_ALIGN_MULTI_STAR = 4
} os_align_mode_t;

typedef enum {
    OS_RATE_SLOW = 0,
    OS_RATE_MEDIUM,
    OS_RATE_FAST,
    OS_RATE_CUSTOM
} os_motion_rate_t;

typedef struct {
    double ra_hours;
    double dec_degrees;
} os_equatorial_coord_t;

typedef struct {
    double az_degrees;
    double alt_degrees;
} os_horizontal_coord_t;

typedef struct {
    double latitude_degrees;
    double longitude_degrees;
    double elevation_metres;
    uint32_t utc_epoch_seconds;
    bool valid;
} os_site_info_t;

typedef struct {
    double motor_step_angle_degrees;
    double gear_ratio[OS_AXIS_COUNT];
    double steps_per_degree[OS_AXIS_COUNT];
    uint16_t microsteps[OS_AXIS_COUNT];
    double max_goto_rate_hz[OS_AXIS_COUNT];
    double tracking_rate_hz;
    double guide_rate_multiplier;
    os_mount_mode_t mount_mode;
    bool channel_enabled[OS_CHANNEL_COUNT];
    bool gps_enabled;
} os_config_t;

typedef struct {
    double m00;
    double m01;
    double m10;
    double m11;
    double b0;
    double b1;
    bool valid;
    double residual_arcsec;
} os_alignment_t;

typedef struct {
    os_equatorial_coord_t sky;
    int32_t motor_steps[OS_AXIS_COUNT];
} os_align_star_t;

typedef struct {
    os_state_t state;
    os_tracking_rate_t tracking_rate;
    os_site_info_t site;
    os_alignment_t alignment;
    int32_t target_steps[OS_AXIS_COUNT];
    int32_t current_steps[OS_AXIS_COUNT];
    bool initialized;
    bool gps_locked;
    bool parked;
    bool goto_active;
    bool manual_active;
    bool guide_active[4];
    uint32_t guide_remaining_ms[4];
    bool calibration_residual_computed;
    uint8_t align_star_count;
    os_error_t last_error;
} os_status_t;

os_error_t os_hal_comm_init(uint8_t channel);
int os_hal_comm_available(uint8_t channel);
int os_hal_comm_read(uint8_t channel);
os_error_t os_hal_comm_write(uint8_t channel, const uint8_t *data, size_t length);
os_error_t os_hal_motor_init(uint8_t axis);
os_error_t os_hal_motor_set_frequency(uint8_t axis, double frequency_hz);
os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward);
os_error_t os_hal_motor_enable(uint8_t axis, bool enable);
int32_t os_hal_motor_get_position(uint8_t axis);
os_error_t os_hal_timer_motor_init(void);
os_error_t os_hal_gps_init(void);
os_error_t os_hal_gps_poll(os_site_info_t *site);
os_error_t os_hal_rtc_init(void);
os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds);
os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds);
os_error_t os_hal_limit_init(void);
bool os_hal_limit_is_triggered(uint8_t axis);
os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count);
os_error_t os_hal_nvm_init(void);
os_error_t os_hal_nvm_read(uint32_t offset, void *data, size_t length);
os_error_t os_hal_nvm_write(uint32_t offset, const void *data, size_t length);

os_error_t os_init(const os_config_t *config);
os_error_t os_loop_iteration(void);
os_error_t os_process_command(uint8_t source_channel, const char *command, char *reply, size_t reply_size);
os_error_t os_goto_equatorial(double ra_hours, double dec_degrees);
os_error_t os_goto_abort(void);
bool os_query_is_moving(void);
os_error_t os_set_tracking_rate(os_tracking_rate_t rate, double custom_hz);
os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms);
os_error_t os_align_reset(os_align_mode_t mode);
os_error_t os_align_add_star(double ra_hours, double dec_degrees, int32_t axis0_steps, int32_t axis1_steps);
os_error_t os_align_compute(void);
os_error_t os_park(void);
os_error_t os_unpark(void);
os_error_t os_manual_move(os_direction_t direction, os_motion_rate_t rate, double custom_hz);
os_error_t os_manual_stop(void);
os_error_t os_pec_set_point(double worm_phase_deg, double correction_arcsec);
os_error_t os_pec_get_correction(double worm_phase_deg, double *correction_arcsec);
os_error_t os_get_status(os_status_t *status);

#ifdef __cplusplus
}
#endif

#endif

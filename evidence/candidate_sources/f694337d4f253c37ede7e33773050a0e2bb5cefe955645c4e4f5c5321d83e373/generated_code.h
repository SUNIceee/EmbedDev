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
#define OS_MAX_ALIGN_STARS 16
#define OS_PEC_TABLE_SIZE 32
#define OS_NVM_SIZE_BYTES 2048u
#define OS_NVM_CALIBRATION_OFFSET 0u
#define OS_NVM_CALIBRATION_SIZE_BYTES 256u
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
    OS_ERR_NO_MEMORY = -8
} os_error_t;

typedef enum {
    OS_STATE_UNINITIALIZED = 0,
    OS_STATE_INITIALIZING,
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
} os_tracking_mode_t;

typedef enum {
    OS_GUIDE_EAST = 0,
    OS_GUIDE_WEST,
    OS_GUIDE_NORTH,
    OS_GUIDE_SOUTH
} os_guide_direction_t;

typedef enum {
    OS_MOTION_EAST = 0,
    OS_MOTION_WEST,
    OS_MOTION_NORTH,
    OS_MOTION_SOUTH
} os_motion_direction_t;

typedef enum {
    OS_RATE_SLOW = 0,
    OS_RATE_MEDIUM,
    OS_RATE_FAST,
    OS_RATE_CUSTOM
} os_motion_rate_t;

typedef enum {
    OS_ALIGN_ONE_STAR = 1,
    OS_ALIGN_TWO_STAR = 2,
    OS_ALIGN_THREE_STAR = 3,
    OS_ALIGN_MULTI_STAR = 4
} os_align_mode_t;

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
    double microsteps;
    double steps_per_degree[OS_AXIS_COUNT];
    double max_goto_rate_deg_per_sec;
    double tracking_rate_arcsec_per_sec;
    double guide_rate_multiplier;
    os_mount_mode_t mount_mode;
    bool gps_enabled;
    bool channel_enabled[OS_CHANNEL_COUNT];
    int32_t park_position_steps[OS_AXIS_COUNT];
    double preset_latitude_degrees;
    double preset_longitude_degrees;
    double preset_elevation_metres;
} os_config_t;

typedef struct {
    double matrix[2][2];
    double offset[2];
    bool valid;
    double residual_arcsec;
} os_alignment_model_t;

typedef struct {
    double ra_hours;
    double dec_degrees;
    int32_t axis_steps[OS_AXIS_COUNT];
} os_alignment_star_t;

typedef struct {
    os_state_t state;
    bool initialized;
    bool gps_locked;
    bool rtc_valid;
    bool moving;
    bool guide_active;
    bool manual_active;
    bool goto_active;
    bool parked;
    bool fault;
    os_error_t last_error;
    double current_ra_hours;
    double current_dec_degrees;
    int32_t motor_position_steps[OS_AXIS_COUNT];
    double motor_frequency_hz[OS_AXIS_COUNT];
    bool motor_enabled[OS_AXIS_COUNT];
    bool limit_triggered[OS_AXIS_COUNT];
    os_tracking_mode_t tracking_mode;
    double custom_tracking_rate_arcsec_per_sec;
    os_alignment_model_t alignment;
    unsigned align_star_count;
} os_status_t;

typedef struct {
    bool initialized;
    bool enabled;
    bool direction_forward;
    double frequency_hz;
    int32_t position_steps;
    os_error_t last_error;
} os_hal_motor_observation_t;

os_error_t os_init(const os_config_t *config);
os_error_t os_loop_iteration(void);
os_error_t os_goto_equatorial(double ra_hours, double dec_degrees);
os_error_t os_goto_abort(void);
os_error_t os_query_is_moving(bool *is_moving);
os_error_t os_get_status(os_status_t *status);
os_error_t os_set_tracking_mode(os_tracking_mode_t mode, double custom_arcsec_per_sec);
os_error_t os_guide_pulse(os_guide_direction_t direction, uint32_t duration_ms);
os_error_t os_manual_move(os_motion_direction_t direction, os_motion_rate_t rate, double custom_hz);
os_error_t os_manual_stop(void);
os_error_t os_park(void);
os_error_t os_unpark(void);
os_error_t os_align_clear(void);
os_error_t os_align_add_star(double ra_hours, double dec_degrees, int32_t axis0_steps, int32_t axis1_steps);
os_error_t os_align_compute(os_align_mode_t mode);
os_error_t os_pec_set_point(unsigned index, double worm_phase_deg, double correction_arcsec);
os_error_t os_pec_get_correction(double worm_phase_deg, double *correction_arcsec);
os_error_t os_process_command(int source_channel, const char *command, char *reply, size_t reply_size);

os_error_t os_hal_comm_init(int channel);
int os_hal_comm_available(int channel);
int os_hal_comm_read(int channel);
os_error_t os_hal_comm_write(int channel, const uint8_t *data, size_t length);
os_error_t os_hal_motor_init(int axis);
os_error_t os_hal_motor_set_frequency(int axis, double frequency_hz);
os_error_t os_hal_motor_set_direction(int axis, bool forward);
os_error_t os_hal_motor_enable(int axis, bool enable);
int32_t os_hal_motor_get_position(int axis);
os_error_t os_hal_timer_motor_init(void);
os_error_t os_hal_gps_init(void);
os_error_t os_hal_gps_poll(os_site_info_t *site);
os_error_t os_hal_rtc_init(void);
os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds);
os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds);
os_error_t os_hal_limit_init(void);
bool os_hal_limit_is_triggered(int axis);
os_error_t os_hal_buzzer_beep(uint32_t duration_ms, unsigned count);
os_error_t os_hal_nvm_init(void);
os_error_t os_hal_nvm_read(size_t offset, void *data, size_t length);
os_error_t os_hal_nvm_write(size_t offset, const void *data, size_t length);

void os_hal_host_reset(void);
os_error_t os_hal_host_push_rx(int channel, const uint8_t *data, size_t length);
size_t os_hal_host_read_tx(int channel, uint8_t *data, size_t max_length);
void os_hal_host_set_limit(int axis, bool triggered);
void os_hal_host_advance_time_ms(uint32_t ms);
void os_hal_host_set_gps(const os_site_info_t *site);
void os_hal_host_set_rtc(uint32_t utc_epoch_seconds, bool valid);
os_error_t os_hal_host_get_motor(int axis, os_hal_motor_observation_t *out);

#ifdef __cplusplus
}
#endif

#endif

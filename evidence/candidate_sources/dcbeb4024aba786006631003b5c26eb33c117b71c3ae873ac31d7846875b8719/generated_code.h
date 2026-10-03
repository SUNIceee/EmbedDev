#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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
#define OS_MAX_ALIGNMENT_STARS 12
#define OS_PEC_TABLE_SIZE 32
#define OS_NVM_CAPACITY_BYTES 2048
#define OS_NVM_CALIBRATION_OFFSET 0
#define OS_NVM_CALIBRATION_SIZE_BYTES 512
#define OS_NVM_CONFIG_OFFSET 512
#define OS_NVM_CONFIG_SIZE_BYTES 512

#define OS_DEFAULT_STEPS_PER_DEGREE 1000.0
#define OS_DEFAULT_GOTO_RATE_HZ 1200.0
#define OS_DEFAULT_TRACKING_RATE_HZ 15.0
#define OS_DEFAULT_GUIDE_RATE_FACTOR 0.5
#define OS_LOOP_DT_SECONDS 0.05
#define OS_GOTO_DONE_TOLERANCE_STEPS 1.0
#define OS_ALIGNMENT_DEGENERATE_EPS 1.0e-9
#define OS_ALIGNMENT_RESIDUAL_LIMIT_ARCSEC 300.0

typedef enum {
    OS_ERR_NONE = 0,
    OS_ERR_INVALID_ARGUMENT = -1,
    OS_ERR_INVALID_STATE = -2,
    OS_ERR_COMMAND_FORMAT = -3,
    OS_ERR_NOT_SUPPORTED = -4,
    OS_ERR_TIMEOUT = -5,
    OS_ERR_DEVICE = -6,
    OS_ERR_LIMIT_TRIGGERED = -7,
    OS_ERR_STORAGE = -8,
    OS_ERR_DEGENERATE_ALIGNMENT = -9
} os_error_t;

typedef enum {
    OS_STATE_INITIALIZING = 0,
    OS_STATE_IDLE_TRACKING = 1,
    OS_STATE_GOTO = 2,
    OS_STATE_ALIGNING = 3,
    OS_STATE_MANUAL_MOTION = 4,
    OS_STATE_PARKED = 5,
    OS_STATE_FAULT = 6
} os_state_t;

typedef enum {
    OS_MOUNT_EQUATORIAL = 0,
    OS_MOUNT_ALTAZ = 1
} os_mount_mode_t;

typedef enum {
    OS_TRACK_SIDEREAL = 0,
    OS_TRACK_LUNAR = 1,
    OS_TRACK_SOLAR = 2,
    OS_TRACK_CUSTOM = 3
} os_tracking_rate_t;

typedef enum {
    OS_GUIDE_EAST = 0,
    OS_GUIDE_WEST = 1,
    OS_GUIDE_NORTH = 2,
    OS_GUIDE_SOUTH = 3
} os_guide_direction_t;

typedef enum {
    OS_MOVE_EAST = 0,
    OS_MOVE_WEST = 1,
    OS_MOVE_NORTH = 2,
    OS_MOVE_SOUTH = 3
} os_move_direction_t;

typedef enum {
    OS_RATE_SLOW = 0,
    OS_RATE_MEDIUM = 1,
    OS_RATE_FAST = 2,
    OS_RATE_CUSTOM = 3
} os_manual_rate_t;

typedef enum {
    OS_ALIGN_ONE_STAR = 1,
    OS_ALIGN_TWO_STAR = 2,
    OS_ALIGN_THREE_STAR = 3,
    OS_ALIGN_MULTI_STAR = 4
} os_alignment_mode_t;

typedef struct {
    double ra_hours;
    double dec_degrees;
} os_equatorial_coord_t;

typedef struct {
    double latitude_degrees;
    double longitude_degrees;
    double elevation_metres;
    uint32_t utc_epoch_seconds;
    bool valid;
} os_site_info_t;

typedef struct {
    double motor_step_angle_degrees;
    double gear_ratio_ra;
    double gear_ratio_dec;
    double steps_per_degree_ra;
    double steps_per_degree_dec;
    uint16_t microsteps;
    double max_goto_rate_hz;
    double tracking_rate_hz;
    double guide_rate_factor;
    os_mount_mode_t mount_mode;
    bool gps_enabled;
    bool rtc_enabled;
    bool channel_enabled[OS_CHANNEL_COUNT];
    int32_t park_position_steps[OS_AXIS_COUNT];
    os_site_info_t preset_site;
} os_config_t;

typedef struct {
    os_state_t state;
    bool initialized;
    bool gps_locked;
    bool rtc_valid;
    bool is_moving;
    bool parked;
    bool fault_active;
    os_error_t last_error;
    int32_t position_steps[OS_AXIS_COUNT];
    int32_t target_steps[OS_AXIS_COUNT];
    double current_ra_hours;
    double current_dec_degrees;
    double alignment_residual_arcsec;
    bool alignment_residual_valid;
    uint8_t alignment_star_count;
    bool guide_active;
    bool manual_motion_active;
    bool goto_active;
} os_status_t;

typedef struct {
    double matrix[2][2];
    double offset[2];
    double residual_arcsec;
    bool valid;
} os_alignment_solution_t;

typedef struct {
    double ra_hours;
    double dec_degrees;
    int32_t axis_steps[OS_AXIS_COUNT];
} os_alignment_star_t;

os_error_t os_init(void);
void os_loop_iteration(void);

os_error_t os_get_status(os_status_t *status);
os_state_t os_get_state(void);
bool os_query_is_moving(void);
os_error_t os_get_current_equatorial(os_equatorial_coord_t *coord);

os_error_t os_set_config(const os_config_t *config);
os_error_t os_get_config(os_config_t *config);

os_error_t os_goto_equatorial(double ra_hours, double dec_degrees);
os_error_t os_goto_abort(void);

os_error_t os_set_tracking_rate(os_tracking_rate_t rate, double custom_factor);
os_error_t os_guide_pulse(os_guide_direction_t direction, uint32_t duration_ms);

os_error_t os_align_begin(os_alignment_mode_t mode);
os_error_t os_align_add_star(double ra_hours,
                             double dec_degrees,
                             int32_t axis0_steps,
                             int32_t axis1_steps);
os_error_t os_align_compute(os_alignment_solution_t *solution);
os_error_t os_align_clear(void);

os_error_t os_park(void);
os_error_t os_unpark(void);

os_error_t os_manual_move(os_move_direction_t direction,
                          os_manual_rate_t rate,
                          double custom_frequency_hz);
os_error_t os_manual_stop(void);

os_error_t os_pec_set_point(double worm_phase_deg, double correction_arcsec);
os_error_t os_pec_get_correction(double worm_phase_deg, double *correction_arcsec);

os_error_t os_process_command(int channel,
                              const char *command,
                              char *reply,
                              size_t reply_size);

/* Hardware adaptation boundary. Board code may override the weak host defaults. */
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

os_error_t os_hal_buzzer_beep(uint32_t duration_ms, uint8_t count);

os_error_t os_hal_comm_init(int channel);
int os_hal_comm_available(int channel);
int os_hal_comm_read(int channel);
os_error_t os_hal_comm_write(int channel, const uint8_t *data, size_t length);

os_error_t os_hal_nvm_init(void);
os_error_t os_hal_nvm_read(size_t offset, void *data, size_t length);
os_error_t os_hal_nvm_write(size_t offset, const void *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif

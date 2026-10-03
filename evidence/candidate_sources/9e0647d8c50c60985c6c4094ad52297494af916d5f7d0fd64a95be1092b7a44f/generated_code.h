#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OS_MAX_COMMAND_LENGTH      64
#define OS_MAX_REPLY_LENGTH        128
#define OS_MAX_CHANNELS            4
#define OS_MAX_ALIGN_STARS         8
#define OS_PEC_MAX_POINTS          32

#define OS_NVM_CALIBRATION_SIZE_BYTES 128
#define OS_NVM_CONFIG_SIZE_BYTES       512

#define OS_STEPS_PER_DEGREE         100.0
#define OS_STEPS_PER_RA_HOUR        (OS_STEPS_PER_DEGREE * 15.0)
#define OS_DEFAULT_TRACK_FREQ_RA_HZ (15.0 * OS_STEPS_PER_DEGREE / 3600.0)
#define OS_MOTOR_MAX_FREQ_HZ        20000.0

typedef enum {
    OS_ERR_NONE = 0,
    OS_ERR_INVALID_ARGUMENT,
    OS_ERR_INVALID_STATE,
    OS_ERR_COMMAND_FORMAT,
    OS_ERR_NOT_SUPPORTED,
    OS_ERR_TIMEOUT,
    OS_ERR_LIMIT,
    OS_ERR_MOTOR,
    OS_ERR_DRIVER,
    OS_ERR_NVM,
    OS_ERR_RTC,
    OS_ERR_GPS,
    OS_ERR_FAULT
} os_error_t;

typedef enum {
    OS_STATE_INIT = 0,
    OS_STATE_IDLE_TRACKING,
    OS_STATE_GOTO,
    OS_STATE_ALIGN,
    OS_STATE_MANUAL,
    OS_STATE_PARKED,
    OS_STATE_FAULT
} os_state_t;

enum {
    OS_AXIS_RA = 0,
    OS_AXIS_DEC = 1,
    OS_AXIS_AZ = OS_AXIS_RA,
    OS_AXIS_ALT = OS_AXIS_DEC
};

enum {
    OS_CHANNEL_USB = 0,
    OS_CHANNEL_BLUETOOTH = 1,
    OS_CHANNEL_WIFI = 2,
    OS_CHANNEL_ETHERNET = 3
};

typedef enum {
    OS_GUIDE_EAST = 0,
    OS_GUIDE_WEST = 1,
    OS_GUIDE_NORTH = 2,
    OS_GUIDE_SOUTH = 3,
    OS_GUIDE_INVALID = 4
} os_guide_direction_t;

typedef enum {
    OS_ALIGN_1STAR = 1,
    OS_ALIGN_2STAR = 2,
    OS_ALIGN_3STAR = 3,
    OS_ALIGN_NSTAR = 4
} os_align_mode_t;

typedef struct {
    bool valid;
    double latitude_degrees;
    double longitude_degrees;
    double elevation_metres;
    uint64_t utc_epoch_seconds;
} os_site_info_t;

typedef struct {
    bool valid;
    double m00, m01, c0;
    double m10, m11, c1;
    uint8_t model;
    uint8_t star_count;
    double residual_arcsec;
} os_calibration_t;

/* Domain API */
os_error_t os_init(void);
void os_loop_iteration(void);

os_error_t os_goto_equatorial(double ra_hours, double dec_degrees);
os_error_t os_goto_abort(void);
bool os_query_is_moving(void);

os_error_t os_align_begin(os_align_mode_t mode);
os_error_t os_align_add_star(double ra_hours, double dec_degrees);
os_error_t os_align_compute(void);

os_error_t os_guide_pulse(os_guide_direction_t direction, uint32_t duration_ms);

os_error_t os_manual_move(int axis, bool forward, double rate_deg_per_sec);
os_error_t os_manual_stop(int axis);

os_error_t os_park(void);
os_error_t os_unpark(void);

os_state_t os_query_state(void);
os_error_t os_query_coordinates(double *ra_hours, double *dec_degrees);
os_error_t os_query_site(os_site_info_t *site);

os_error_t os_pec_set_table(const double *phase_deg, const double *error_arcsec, uint8_t count);
os_error_t os_pec_get_correction(double worm_phase_deg, double *correction_arcsec);

/* Platform adapter / HAL prototypes */
os_error_t os_hal_motor_init(int axis);
os_error_t os_hal_motor_set_frequency(int axis, double frequency_hz);
os_error_t os_hal_motor_set_direction(int axis, bool forward);
os_error_t os_hal_motor_enable(int axis, bool enable);
int64_t os_hal_motor_get_position(int axis);
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
os_error_t os_hal_comm_write(int channel, const void *data, size_t length);

os_error_t os_hal_nvm_init(void);
os_error_t os_hal_nvm_read(uint32_t offset, void *data, size_t length);
os_error_t os_hal_nvm_write(uint32_t offset, const void *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif

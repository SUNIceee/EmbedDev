#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OS_MAX_AXES 2
#define OS_MAX_CHANNELS 4
#define OS_MAX_COMMAND_LENGTH 64
#define OS_MAX_REPLY_LENGTH 128
#define OS_NVM_CALIBRATION_SIZE_BYTES 512
#define OS_NVM_CONFIG_SIZE_BYTES 512
#define OS_NVM_TOTAL_SIZE_BYTES (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)
#define OS_PEC_MAX_POINTS 64

typedef enum {
    OS_ERR_NONE = 0,
    OS_ERR_INVALID_ARGUMENT,
    OS_ERR_INVALID_STATE,
    OS_ERR_COMMAND_FORMAT,
    OS_ERR_TIMEOUT,
    OS_ERR_NOT_SUPPORTED,
    OS_ERR_DEVICE_FAULT,
    OS_ERR_LIMIT_TRIGGERED,
    OS_ERR_NVM,
    OS_ERR_RTC,
    OS_ERR_GPS
} os_error_t;

typedef enum {
    OS_STATE_INIT = 0,
    OS_STATE_IDLE_TRACKING,
    OS_STATE_GOTO,
    OS_STATE_PARKING,
    OS_STATE_PARKED,
    OS_STATE_MANUAL,
    OS_STATE_ALIGN,
    OS_STATE_FAULT
} os_state_t;

typedef enum {
    OS_CHANNEL_USB = 0,
    OS_CHANNEL_BLUETOOTH = 1,
    OS_CHANNEL_WIFI = 2,
    OS_CHANNEL_ETHERNET = 3
} os_channel_t;

typedef enum {
    OS_AXIS_RA = 0,
    OS_AXIS_DEC = 1,
    OS_AXIS_COUNT = 2
} os_axis_t;
#define OS_AXIS_AZ OS_AXIS_RA
#define OS_AXIS_ALT OS_AXIS_DEC

typedef enum {
    OS_GUIDE_RA_PLUS = 0,
    OS_GUIDE_RA_MINUS = 1,
    OS_GUIDE_DEC_PLUS = 2,
    OS_GUIDE_DEC_MINUS = 3
} os_guide_direction_t;
#define OS_GUIDE_EAST OS_GUIDE_RA_PLUS
#define OS_GUIDE_WEST OS_GUIDE_RA_MINUS
#define OS_GUIDE_NORTH OS_GUIDE_DEC_PLUS
#define OS_GUIDE_SOUTH OS_GUIDE_DEC_MINUS

#define OS_ALIGN_MODE_1STAR 1
#define OS_ALIGN_MODE_2STAR 2
#define OS_ALIGN_MODE_3STAR 3
#define OS_ALIGN_MODE_NSTAR 4

typedef struct {
    bool valid;
    double latitude_degrees;
    double longitude_degrees;
    double elevation_metres;
    uint32_t utc_epoch_seconds;
} os_site_info_t;

typedef struct {
    double worm_phase_deg;
    double correction_arcsec;
} os_pec_point_t;

/* Public OnStep domain API */
os_error_t os_init(void);
void os_loop_iteration(void);

os_error_t os_goto_equatorial(double ra_hours, double dec_degrees);
os_error_t os_goto_abort(void);
os_error_t os_query_is_moving(bool *moving);

os_error_t os_guide_pulse(os_guide_direction_t direction, uint32_t duration_ms);

os_error_t os_align_add_star(double ra_hours, double dec_degrees,
                             int32_t ra_steps, int32_t dec_steps);
os_error_t os_align_compute(uint8_t align_mode);
os_error_t os_align_clear(void);
os_error_t os_query_alignment_residual(double *residual_arcsec);

os_error_t os_park(void);
os_error_t os_unpark(void);

os_error_t os_set_site(const os_site_info_t *site);
os_error_t os_get_site(os_site_info_t *site);

os_error_t os_set_tracking_rate(double rate_factor);
os_error_t os_query_tracking_rate(double *rate_factor);
os_error_t os_query_state(os_state_t *state);
os_error_t os_query_axis_position(os_axis_t axis, int32_t *position);

os_error_t os_manual_move(os_axis_t axis, bool forward, double frequency_hz);
os_error_t os_manual_stop(os_axis_t axis);

os_error_t os_pec_set(const os_pec_point_t *points, uint16_t count);
os_error_t os_pec_clear(void);
os_error_t os_pec_apply(double worm_phase_deg, double *correction_arcsec);

/* Platform adapter / HAL API */
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

void os_hal_buzzer_beep(uint32_t duration_ms, uint8_t count);

os_error_t os_hal_comm_init(uint8_t channel);
size_t os_hal_comm_available(uint8_t channel);
int32_t os_hal_comm_read(uint8_t channel);
os_error_t os_hal_comm_write(uint8_t channel, const uint8_t *data, size_t length);

os_error_t os_hal_nvm_init(void);
os_error_t os_hal_nvm_read(uint32_t offset, uint8_t *data, size_t length);
os_error_t os_hal_nvm_write(uint32_t offset, const uint8_t *data, size_t length);

/* Host-test injection/observation helpers (deterministic host adapter support) */
void os_hal_test_comm_inject(uint8_t channel, const uint8_t *data, size_t length);
void os_hal_test_comm_clear(uint8_t channel);
void os_hal_test_gps_inject(const os_site_info_t *site);
void os_hal_test_rtc_inject(uint32_t utc_epoch_seconds);
void os_hal_test_limit_inject(uint8_t axis, bool triggered);
void os_hal_test_motor_inject_position(uint8_t axis, int32_t position);
void os_hal_test_nvm_clear(void);
void os_hal_test_nvm_inject_write_fault(bool fault);
void os_hal_test_nvm_inject_read_fault(bool fault);
void os_hal_test_buzzer_clear(void);
size_t os_hal_test_buzzer_event_count(void);

#ifdef __cplusplus
}
#endif

#endif /* GENERATED_CODE_H */

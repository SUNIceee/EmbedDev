#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OS_AXIS_RA                0
#define OS_AXIS_DEC               1
#define OS_AXIS_COUNT             2

#define OS_CHANNEL_USB            0
#define OS_CHANNEL_BLUETOOTH      1
#define OS_CHANNEL_WIFI           2
#define OS_CHANNEL_ETHERNET       3
#define OS_CHANNEL_COUNT          4

#define OS_MAX_COMMAND_LENGTH     64
#define OS_MAX_REPLY_LENGTH       128
#define OS_NVM_CALIBRATION_SIZE_BYTES 256
#define OS_NVM_CONFIG_SIZE_BYTES  256
#define OS_MAX_ALIGN_STARS        6

#define OS_MOTOR_STEPS_PER_DEGREE 800.0
#define OS_DEFAULT_GOTO_FREQ_HZ   200.0
#define OS_DEFAULT_GUIDE_RATE_MULTIPLIER 0.5
#define OS_ALIGN_DEGENERATE_DET   1e-9
#define OS_ALIGN_MAX_RESIDUAL_ARCSEC 300.0

typedef enum {
    OS_ERR_NONE = 0,
    OS_ERR_INVALID_ARGUMENT = 1,
    OS_ERR_INVALID_STATE = 2,
    OS_ERR_COMMAND_FORMAT = 3,
    OS_ERR_NOT_SUPPORTED = 4,
    OS_ERR_TIMEOUT = 5,
    OS_ERR_DEVICE_FAULT = 6,
    OS_ERR_NVM = 7,
    OS_ERR_GPS = 8,
    OS_ERR_RTC = 9,
    OS_ERR_LIMIT = 10,
    OS_ERR_DRIVER = 11
} os_error_t;

typedef enum {
    OS_STATE_UNINITIALIZED = 0,
    OS_STATE_INIT,
    OS_STATE_IDLE_TRACKING,
    OS_STATE_GOTO,
    OS_STATE_PARKING,
    OS_STATE_PARKED,
    OS_STATE_ALIGN,
    OS_STATE_MANUAL_MOVING,
    OS_STATE_FAULT
} os_state_t;

typedef enum {
    OS_ALIGN_1STAR = 1,
    OS_ALIGN_2STAR = 2,
    OS_ALIGN_3STAR = 3,
    OS_ALIGN_NSTAR = 4
} os_align_mode_t;

typedef enum {
    OS_GUIDE_DIR_EAST = 0,
    OS_GUIDE_DIR_WEST = 1,
    OS_GUIDE_DIR_NORTH = 2,
    OS_GUIDE_DIR_SOUTH = 3
} os_guide_direction_t;

typedef struct {
    bool valid;
    double latitude_degrees;
    double longitude_degrees;
    double elevation_metres;
    uint32_t utc_epoch_seconds;
} os_site_info_t;

typedef struct {
    double ra_hours;
    double dec_deg;
    int32_t pos[OS_AXIS_COUNT];
} os_align_star_t;

/* Public OnStep API */
os_error_t os_init(void);
void os_loop_iteration(void);

os_error_t os_goto_equatorial(double ra_hours, double dec_deg);
os_error_t os_goto_abort(void);
os_error_t os_query_is_moving(bool *is_moving);
os_error_t os_get_state(os_state_t *state);
os_error_t os_set_tracking_rate(double multiplier);

os_error_t os_guide_pulse(os_guide_direction_t direction, uint32_t duration_ms);
os_error_t os_align_add_star(double ra_hours, double dec_deg);
os_error_t os_align_compute(os_align_mode_t mode);
os_error_t os_align_clear(void);

os_error_t os_park(void);
os_error_t os_unpark(void);
os_error_t os_move_axis(uint8_t axis, bool positive, double frequency_hz);
os_error_t os_move_stop(uint8_t axis);

os_error_t os_pec_set_entry(double worm_phase_deg, double correction_arcsec);
os_error_t os_pec_clear(void);

os_error_t os_command_receive(uint8_t source_channel, const char *command,
                              char *reply, size_t reply_len);
os_error_t os_get_site(os_site_info_t *site);

/* Platform adapter API implemented by generated_code.c */
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

os_error_t os_hal_comm_init(uint8_t channel);
int16_t os_hal_comm_available(uint8_t channel);
int16_t os_hal_comm_read(uint8_t channel);
os_error_t os_hal_comm_write(uint8_t channel, const uint8_t *data, size_t length);

os_error_t os_hal_nvm_init(void);
os_error_t os_hal_nvm_read(uint32_t offset, void *data, size_t length);
os_error_t os_hal_nvm_write(uint32_t offset, const void *data, size_t length);

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count);

/* Host test hooks for deterministic tests */
void os_test_hal_set_gps_fix(bool valid, double lat, double lon,
                             double elev, uint32_t utc);
void os_test_hal_clear_gps(void);
void os_test_hal_set_rtc(uint32_t utc);
void os_test_hal_set_limit(uint8_t axis, bool triggered);
void os_test_hal_set_motor_position(uint8_t axis, int32_t steps);
void os_test_hal_set_driver_fault(uint8_t axis, bool fault);
void os_test_hal_reset_comm(uint8_t channel);
size_t os_test_hal_tx_available(uint8_t channel);
int os_test_hal_tx_read(uint8_t channel);
void os_test_hal_inject_byte(uint8_t channel, uint8_t byte);
void os_test_hal_inject_command(uint8_t channel, const char *cmd);
void os_test_hal_set_nvm_byte(uint32_t offset, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif

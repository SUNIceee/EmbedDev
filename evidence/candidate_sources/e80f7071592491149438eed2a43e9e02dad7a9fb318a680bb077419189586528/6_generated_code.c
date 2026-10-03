#include "6_generated_code.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#define OS_NUM_AXES                2
#define OS_NUM_CHANNELS            4
#define OS_NVM_TOTAL_SIZE          1536U
#define OS_NVM_CALIB_OFFSET        0U
#define OS_NVM_CONFIG_OFFSET       256U
#define OS_NVM_PEC_OFFSET          512U
#define OS_NVM_CONFIG_MAGIC        0x4F4E5354U
#define OS_NVM_CALIB_MAGIC         0x43414C42U
#define OS_NVM_PEC_MAGIC           0x50454331U
#define OS_LOOP_TICK_MS            10U
#define OS_TRACKING_BASE_FREQ_HZ   100U
#define OS_GOTO_SLEW_FREQ_HZ       1000U
#define OS_MANUAL_SLOW_FREQ_HZ     50U
#define OS_MANUAL_MEDIUM_FREQ_HZ   200U
#define OS_MANUAL_FAST_FREQ_HZ     800U
#define OS_MANUAL_TIMEOUT_MS       60000U
#define OS_MANUAL_ARC_SEC_PER_HZ_FACTOR 100.0f
#define OS_HAL_MOTOR_MAX_FREQ_HZ   200000U
#define OS_STEPS_PER_RA_HOUR       10000.0f
#define OS_STEPS_PER_DEC_DEGREE    10000.0f
#define OS_STEPS_PER_DEGREE_MOTOR  10000.0f
#define OS_CALIBRATION_DET_EPSILON 1e-9
#define OS_CALIBRATION_ARCSEC_PER_STEP 0.36
#define OS_CALIBRATION_MAX_RESIDUAL_ARCSEC 300.0
#define OS_MAX_CUSTOM_MANUAL_ARCSEC_PER_SEC 1000000.0f
#define OS_GOTO_MIN_STEP           100
#define OS_GOTO_MAX_STEP           1000
#define OS_GOTO_RAMP_TICKS         3
#define OS_GOTO_DECEL_TICKS        3
#define OS_WORM_PERIOD_SEC         600.0f
#define OS_PEC_HZ_PER_ARCSEC       0.01f

typedef struct {
    bool initialized;
    bool enabled;
    bool direction_forward;
    uint32_t frequency_hz;
    int32_t position_steps;
    bool drive_fault;
} hal_axis_t;

typedef struct {
    bool initialized;
    bool suspended;
    char rx[OS_MAX_COMMAND_LENGTH];
    uint16_t rx_count;
    uint16_t rx_head;
    char tx[OS_MAX_REPLY_LENGTH];
    uint16_t tx_count;
} hal_comm_t;

typedef struct {
    uint32_t magic;
    os_site_info_t site;
    os_equatorial_coord_t park;
} nvm_config_t;

typedef struct {
    uint32_t magic;
    uint32_t crc;
    os_calibration_t calibration;
} nvm_calibration_record_t;

typedef struct {
    uint32_t magic;
    uint32_t crc;
    os_pec_table_t table;
} nvm_pec_record_t;

typedef struct {
    os_equatorial_coord_t star_coord;
    os_motor_position_t motor_pos;
} alignment_sample_t;

static os_state_t system_state = OS_STATE_INITIALIZING;
static bool tracking_enabled;
static os_track_rate_t tracking_rate = OS_TRACK_RATE_SIDEREAL;
static float tracking_custom_factor = 1.0f;
static float guide_rate_fraction = 0.5f;
static os_guide_pulse_t guide_pulse;
static bool goto_motion_active;
static bool park_motion_active;
static bool manual_motion_active;
static uint32_t manual_remaining_ms;
static uint8_t manual_axis;
static os_motor_position_t goto_target_steps;
static os_motor_position_t park_target_steps;
static uint32_t goto_elapsed_ticks;
static uint32_t park_elapsed_ticks;
static os_equatorial_coord_t command_target_eq;
static os_equatorial_coord_t park_position_eq;
static bool custom_park_position_set;
static os_site_info_t site_info;
static os_calibration_t calibration;
static bool calibration_residual_computed;
static double calibration_residual_arcsec;
static os_align_mode_t alignment_mode;
static uint8_t alignment_star_count;
static alignment_sample_t alignment_samples[OS_CALIBRATION_MAX_STARS];
static bool pec_enabled;
static os_pec_table_t pec_table;
static float pec_phase_deg;
static float custom_manual_arcsec_per_sec;
static uint32_t loop_tick_ms;

static uint8_t active_guide_axis;

static hal_axis_t hal_axis[OS_NUM_AXES];
static hal_comm_t hal_comm[OS_NUM_CHANNELS];

static uint8_t nvm_image[OS_NVM_TOTAL_SIZE];
static bool nvm_initialized;

static bool gps_initialized;
static bool rtc_initialized;
static uint32_t rtc_epoch_seconds;
static bool limit_initialized;
static bool limit_triggered[OS_NUM_AXES];
static bool motor_timer_initialized;
static uint16_t last_buzzer_duration_ms;
static uint8_t last_buzzer_count;

static char loop_cmd[OS_NUM_CHANNELS][OS_MAX_COMMAND_LENGTH];
static uint8_t loop_cmd_len[OS_NUM_CHANNELS];

static bool axis_valid(uint8_t axis) {
    return axis < OS_NUM_AXES;
}

static bool channel_valid(uint8_t channel) {
    return channel < OS_NUM_CHANNELS;
}

static bool equatorial_coord_valid(float ra_hours, float dec_degrees) {
    return ra_hours >= OS_RA_MIN_HOURS && ra_hours <= OS_RA_MAX_HOURS &&
           dec_degrees >= OS_DEC_MIN_DEG && dec_degrees <= OS_DEC_MAX_DEG;
}

static bool direction_valid(os_direction_t direction) {
    switch (direction) {
        case OS_DIRECTION_NORTH:
        case OS_DIRECTION_SOUTH:
        case OS_DIRECTION_EAST:
        case OS_DIRECTION_WEST:
            return true;
        default:
            return false;
    }
}

static bool speed_valid(os_speed_level_t speed) {
    switch (speed) {
        case OS_SPEED_SLOW:
        case OS_SPEED_MEDIUM:
        case OS_SPEED_FAST:
        case OS_SPEED_CUSTOM:
            return true;
        default:
            return false;
    }
}

static bool align_mode_valid(os_align_mode_t mode) {
    return mode >= OS_ALIGN_1STAR && mode <= OS_ALIGN_NSTAR;
}

static double get_tracking_factor(void) {
    switch (tracking_rate) {
        case OS_TRACK_RATE_LUNAR:
            return (double)OS_LUNAR_RATE_FACTOR;
        case OS_TRACK_RATE_SOLAR:
            return (double)OS_SOLAR_RATE_FACTOR;
        case OS_TRACK_RATE_CUSTOM:
            return (double)tracking_custom_factor;
        default:
            return 1.0;
    }
}

static uint32_t compute_tracking_frequency_hz(void) {
    double freq = (double)OS_TRACKING_BASE_FREQ_HZ * get_tracking_factor();
    if (pec_enabled && pec_table.valid) {
        int idx = (int)pec_phase_deg % OS_PEC_TABLE_SIZE;
        if (idx < 0) {
            idx += OS_PEC_TABLE_SIZE;
        }
        freq += (double)pec_table.corrections[idx] * (double)OS_PEC_HZ_PER_ARCSEC;
    }
    if (freq < 0.0) {
        freq = 0.0;
    }
    return (uint32_t)freq;
}

static uint32_t manual_speed_frequency_hz(os_speed_level_t speed) {
    switch (speed) {
        case OS_SPEED_SLOW:
            return OS_MANUAL_SLOW_FREQ_HZ;
        case OS_SPEED_MEDIUM:
            return OS_MANUAL_MEDIUM_FREQ_HZ;
        case OS_SPEED_FAST:
            return OS_MANUAL_FAST_FREQ_HZ;
        case OS_SPEED_CUSTOM:
            return (uint32_t)((double)custom_manual_arcsec_per_sec *
                              (double)OS_MANUAL_ARC_SEC_PER_HZ_FACTOR);
        default:
            return 0U;
    }
}

static void reset_calibration_fields(void) {
    calibration.valid = false;
    calibration.matrix_ra_to_ra = 0.0f;
    calibration.matrix_ra_to_dec = 0.0f;
    calibration.matrix_dec_to_ra = 0.0f;
    calibration.matrix_dec_to_dec = 0.0f;
    calibration.offset_ra_arcsec = 0.0f;
    calibration.offset_dec_arcsec = 0.0f;
}

static void reset_runtime_flags(void) {
    system_state = OS_STATE_INITIALIZING;
    tracking_enabled = false;
    tracking_rate = OS_TRACK_RATE_SIDEREAL;
    tracking_custom_factor = 1.0f;
    guide_rate_fraction = 0.5f;
    goto_motion_active = false;
    park_motion_active = false;
    manual_motion_active = false;
    manual_remaining_ms = 0U;
    manual_axis = 0;
    goto_elapsed_ticks = 0U;
    park_elapsed_ticks = 0U;
    command_target_eq.ra_hours = 0.0f;
    command_target_eq.dec_degrees = 0.0f;
    reset_calibration_fields();
    calibration_residual_computed = false;
    calibration_residual_arcsec = 0.0;
    alignment_star_count = 0;
    alignment_mode = OS_ALIGN_1STAR;
    guide_pulse.active = false;
    guide_pulse.duration_ms = 0U;
    guide_pulse.rate_fraction = guide_rate_fraction;
    guide_pulse.direction_east = false;
    guide_pulse.direction_north = false;
    guide_pulse.dec_priority = false;
    active_guide_axis = 0;
    pec_enabled = false;
    pec_table.valid = false;
    memset(pec_table.corrections, 0, sizeof(pec_table.corrections));
    pec_phase_deg = 0.0f;
    custom_manual_arcsec_per_sec = 0.0f;
    loop_tick_ms = 0U;
    custom_park_position_set = false;
    memset(&site_info, 0, sizeof(site_info));
    park_position_eq.ra_hours = 0.0f;
    park_position_eq.dec_degrees = 90.0f;
}

static uint32_t nvm_crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
            crc = (crc >> 1U) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static uint32_t calibration_payload_crc(const os_calibration_t *cal) {
    return nvm_crc32((const uint8_t *)cal, sizeof(*cal));
}

static uint32_t pec_payload_crc(const os_pec_table_t *table) {
    return nvm_crc32((const uint8_t *)table, sizeof(*table));
}

static os_error_t load_nvm_config(void) {
    nvm_config_t cfg;
    os_error_t err = os_hal_nvm_read(OS_NVM_CONFIG_OFFSET, (uint8_t *)&cfg, (uint16_t)sizeof(cfg));
    if (err != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    if (cfg.magic != OS_NVM_CONFIG_MAGIC) {
        memset(&site_info, 0, sizeof(site_info));
        park_position_eq.ra_hours = 0.0f;
        park_position_eq.dec_degrees = 90.0f;
        custom_park_position_set = false;
        return OS_ERR_NONE;
    }
    site_info = cfg.site;
    site_info.valid = false;
    park_position_eq = cfg.park;
    custom_park_position_set = true;
    return OS_ERR_NONE;
}

static os_error_t save_nvm_config(void) {
    nvm_config_t cfg;
    cfg.magic = OS_NVM_CONFIG_MAGIC;
    cfg.site = site_info;
    cfg.site.valid = false;
    cfg.park = park_position_eq;
    return os_hal_nvm_write(OS_NVM_CONFIG_OFFSET, (const uint8_t *)&cfg, (uint16_t)sizeof(cfg));
}

static os_error_t load_nvm_calibration(void) {
    nvm_calibration_record_t rec;
    os_error_t err = os_hal_nvm_read(OS_NVM_CALIB_OFFSET, (uint8_t *)&rec, (uint16_t)sizeof(rec));
    if (err != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    if (rec.magic != OS_NVM_CALIB_MAGIC ||
        rec.crc != calibration_payload_crc(&rec.calibration)) {
        reset_calibration_fields();
        return OS_ERR_NONE;
    }
    calibration = rec.calibration;
    return OS_ERR_NONE;
}

static os_error_t save_nvm_calibration(void) {
    nvm_calibration_record_t rec;
    rec.magic = OS_NVM_CALIB_MAGIC;
    rec.calibration = calibration;
    rec.crc = calibration_payload_crc(&rec.calibration);
    return os_hal_nvm_write(OS_NVM_CALIB_OFFSET, (const uint8_t *)&rec, (uint16_t)sizeof(rec));
}

static os_error_t load_nvm_pec(void) {
    nvm_pec_record_t rec;
    os_error_t err = os_hal_nvm_read(OS_NVM_PEC_OFFSET, (uint8_t *)&rec, (uint16_t)sizeof(rec));
    if (err != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    if (rec.magic != OS_NVM_PEC_MAGIC ||
        rec.crc != pec_payload_crc(&rec.table)) {
        pec_table.valid = false;
        memset(pec_table.corrections, 0, sizeof(pec_table.corrections));
        return OS_ERR_NONE;
    }
    pec_table = rec.table;
    return OS_ERR_NONE;
}

static os_error_t save_nvm_pec(void) {
    nvm_pec_record_t rec;
    rec.magic = OS_NVM_PEC_MAGIC;
    rec.table = pec_table;
    rec.crc = pec_payload_crc(&rec.table);
    return os_hal_nvm_write(OS_NVM_PEC_OFFSET, (const uint8_t *)&rec, (uint16_t)sizeof(rec));
}

static void stop_all_pulses(void) {
    (void)os_hal_motor_set_frequency(0, 0U);
    (void)os_hal_motor_set_frequency(1, 0U);
}

static void disable_all_motors(void) {
    (void)os_hal_motor_enable(0, false);
    (void)os_hal_motor_enable(1, false);
}

static void suspend_all_comm_rx(void) {
    for (uint8_t channel = 0U; channel < OS_NUM_CHANNELS; channel++) {
        hal_comm[channel].suspended = true;
    }
}

static void resume_all_comm_rx(void) {
    for (uint8_t channel = 0U; channel < OS_NUM_CHANNELS; channel++) {
        hal_comm[channel].suspended = false;
    }
}

static void enter_fault(void) {
    system_state = OS_STATE_FAULT;
    goto_motion_active = false;
    park_motion_active = false;
    manual_motion_active = false;
    guide_pulse.active = false;
    guide_pulse.duration_ms = 0U;
    guide_pulse.direction_east = false;
    guide_pulse.direction_north = false;
    guide_pulse.dec_priority = false;
    stop_all_pulses();
    disable_all_motors();
    resume_all_comm_rx();
}

static os_error_t apply_tracking_motor_frequency(void) {
    if (!tracking_enabled || system_state == OS_STATE_PARKED || system_state == OS_STATE_FAULT) {
        (void)os_hal_motor_set_frequency(0, 0U);
        (void)os_hal_motor_set_frequency(1, 0U);
        return OS_ERR_INVALID_STATE;
    }
    (void)os_hal_motor_enable(0, true);
    (void)os_hal_motor_set_direction(0, true);
    (void)os_hal_motor_set_frequency(0, compute_tracking_frequency_hz());
    (void)os_hal_motor_set_frequency(1, 0U);
    return OS_ERR_NONE;
}

static void advance_axis_position_by_frequency(uint8_t axis, uint32_t elapsed_ms) {
    if (!axis_valid(axis)) {
        return;
    }
    uint32_t freq_hz = hal_axis[axis].frequency_hz;
    if (freq_hz == 0U) {
        return;
    }
    uint64_t increment = ((uint64_t)freq_hz * (uint64_t)elapsed_ms) / 1000U;
    if (increment > 0U) {
        hal_axis[axis].position_steps += (int32_t)increment;
    }
}

static void advance_pec_phase(void) {
    float degrees_per_tick = (float)((double)OS_LOOP_TICK_MS / 1000.0 * 360.0 / OS_WORM_PERIOD_SEC);
    pec_phase_deg += degrees_per_tick;
    if (pec_phase_deg >= 360.0f) {
        pec_phase_deg = fmodf(pec_phase_deg, 360.0f);
    }
}

static void equatorial_to_motor_steps(const os_equatorial_coord_t *coord, os_motor_position_t *steps) {
    if (calibration.valid) {
        double ra_arcsec = (double)coord->ra_hours * 54000.0;
        double dec_arcsec = (double)coord->dec_degrees * 3600.0;
        steps->ra_steps = (int32_t)((double)calibration.matrix_ra_to_ra * ra_arcsec +
                                     (double)calibration.matrix_ra_to_dec * dec_arcsec +
                                     (double)calibration.offset_ra_arcsec);
        steps->dec_steps = (int32_t)((double)calibration.matrix_dec_to_ra * ra_arcsec +
                                      (double)calibration.matrix_dec_to_dec * dec_arcsec +
                                      (double)calibration.offset_dec_arcsec);
    } else {
        steps->ra_steps = (int32_t)((double)coord->ra_hours * OS_STEPS_PER_RA_HOUR);
        steps->dec_steps = (int32_t)((double)coord->dec_degrees * OS_STEPS_PER_DEC_DEGREE);
    }
}

static void motor_steps_to_equatorial(const os_motor_position_t *pos, os_equatorial_coord_t *coord) {
    if (calibration.valid) {
        double m00 = (double)calibration.matrix_ra_to_ra;
        double m01 = (double)calibration.matrix_ra_to_dec;
        double m10 = (double)calibration.matrix_dec_to_ra;
        double m11 = (double)calibration.matrix_dec_to_dec;
        double det = m00 * m11 - m01 * m10;
        if (fabs(det) > 1e-12) {
            double b0 = (double)pos->ra_steps - (double)calibration.offset_ra_arcsec;
            double b1 = (double)pos->dec_steps - (double)calibration.offset_dec_arcsec;
            double ra_arcsec = (b0 * m11 - m01 * b1) / det;
            double dec_arcsec = (m00 * b1 - b0 * m10) / det;
            coord->ra_hours = (float)(ra_arcsec / 54000.0);
            coord->dec_degrees = (float)(dec_arcsec / 3600.0);
            return;
        }
    }
    coord->ra_hours = (float)((double)pos->ra_steps / OS_STEPS_PER_RA_HOUR);
    coord->dec_degrees = (float)((double)pos->dec_steps / OS_STEPS_PER_DEC_DEGREE);
}

static void horizontal_to_motor_steps(const os_horizontal_coord_t *coord, os_motor_position_t *steps) {
    steps->ra_steps = (int32_t)((double)coord->azimuth_degrees * OS_STEPS_PER_DEGREE_MOTOR);
    steps->dec_steps = (int32_t)((double)coord->altitude_degrees * OS_STEPS_PER_DEGREE_MOTOR);
}

static uint32_t compute_slew_step_inc(int64_t remaining_abs, uint32_t elapsed_ticks) {
    if (remaining_abs <= 0) {
        return 0U;
    }
    uint32_t step;
    if (elapsed_ticks < OS_GOTO_RAMP_TICKS) {
        step = OS_GOTO_MIN_STEP +
               ((OS_GOTO_MAX_STEP - OS_GOTO_MIN_STEP) * (elapsed_ticks + 1U)) / OS_GOTO_RAMP_TICKS;
    } else if (remaining_abs <= (int64_t)OS_GOTO_MAX_STEP * OS_GOTO_DECEL_TICKS) {
        if ((uint64_t)remaining_abs < OS_GOTO_MIN_STEP) {
            step = (uint32_t)remaining_abs;
        } else {
            step = OS_GOTO_MIN_STEP +
                   ((OS_GOTO_MAX_STEP - OS_GOTO_MIN_STEP) * (uint32_t)remaining_abs) /
                   (OS_GOTO_MAX_STEP * OS_GOTO_DECEL_TICKS);
        }
    } else {
        step = OS_GOTO_MAX_STEP;
    }
    if (step == 0U) {
        step = 1U;
    }
    if ((uint64_t)step > (uint64_t)remaining_abs) {
        step = (uint32_t)remaining_abs;
    }
    return step;
}

static void move_axis_toward(uint8_t axis, int32_t target, uint32_t max_step) {
    if (!axis_valid(axis)) {
        return;
    }
    int32_t current = hal_axis[axis].position_steps;
    int64_t delta = (int64_t)target - (int64_t)current;
    if (delta == 0) {
        return;
    }
    bool forward = delta > 0;
    int64_t abs_delta = forward ? delta : -delta;
    uint32_t step = (uint64_t)abs_delta < (uint64_t)max_step ? (uint32_t)abs_delta : max_step;
    if (step == 0U) {
        return;
    }
    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, step * 100U);
    hal_axis[axis].position_steps += forward ? (int32_t)step : -(int32_t)step;
}

static void goto_finalize(void) {
    goto_motion_active = false;
    goto_elapsed_ticks = 0U;
    stop_all_pulses();
    (void)os_hal_buzzer_beep(50U, 1U);
    system_state = OS_STATE_IDLE_TRACKING;
    if (tracking_enabled) {
        (void)apply_tracking_motor_frequency();
    }
}

static void park_finalize(void) {
    park_motion_active = false;
    park_elapsed_ticks = 0U;
    stop_all_pulses();
    tracking_enabled = false;
    disable_all_motors();
    suspend_all_comm_rx();
    system_state = OS_STATE_PARKED;
}

static void complete_guide(void) {
    guide_pulse.active = false;
    guide_pulse.duration_ms = 0U;
    guide_pulse.direction_east = false;
    guide_pulse.direction_north = false;
    guide_pulse.dec_priority = false;
    if (active_guide_axis == 0 && tracking_enabled && system_state == OS_STATE_IDLE_TRACKING) {
        (void)apply_tracking_motor_frequency();
    } else {
        (void)os_hal_motor_set_frequency(active_guide_axis, 0U);
    }
}

static os_error_t start_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    bool is_dec = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);
    active_guide_axis = is_dec ? 1U : 0U;
    uint32_t guide_bias_hz = (uint32_t)((double)OS_TRACKING_BASE_FREQ_HZ * (double)guide_rate_fraction);
    uint32_t total_hz = guide_bias_hz;
    if (!is_dec && tracking_enabled) {
        total_hz += compute_tracking_frequency_hz();
    }
    if (total_hz == 0U) {
        total_hz = guide_bias_hz;
    }
    bool forward = is_dec ? (direction == OS_DIRECTION_NORTH) : (direction == OS_DIRECTION_EAST);
    guide_pulse.active = true;
    guide_pulse.duration_ms = duration_ms;
    guide_pulse.rate_fraction = guide_rate_fraction;
    guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    guide_pulse.dec_priority = is_dec;
    (void)os_hal_motor_enable(active_guide_axis, true);
    (void)os_hal_motor_set_direction(active_guide_axis, forward);
    (void)os_hal_motor_set_frequency(active_guide_axis, total_hz);
    return OS_ERR_NONE;
}

static void stop_manual_motion(void) {
    manual_motion_active = false;
    manual_remaining_ms = 0U;
    stop_all_pulses();
    system_state = OS_STATE_IDLE_TRACKING;
    if (tracking_enabled) {
        (void)apply_tracking_motor_frequency();
    }
}

static void poll_commands(void) {
    for (uint8_t channel = 0; channel < OS_NUM_CHANNELS; channel++) {
        if (hal_comm[channel].suspended) {
            continue;
        }
        int16_t available = os_hal_comm_available(channel);
        while (available-- > 0 && loop_cmd_len[channel] < OS_MAX_COMMAND_LENGTH) {
            char c = os_hal_comm_read(channel);
            loop_cmd[channel][loop_cmd_len[channel]++] = c;
            if (c == OS_LX200_CMD_SUFFIX) {
                char reply[OS_MAX_REPLY_LENGTH];
                size_t reply_len = 0U;
                (void)os_command_parse(loop_cmd[channel], (size_t)loop_cmd_len[channel], channel,
                                       reply, sizeof(reply), &reply_len);
                if (reply_len > 0U) {
                    (void)os_hal_comm_write(channel, reply, reply_len);
                }
                loop_cmd_len[channel] = 0U;
            }
        }
    }
}

static void update_site_time(void) {
    os_site_info_t polled;
    os_error_t gps_err = os_hal_gps_poll(&polled);
    if (gps_err == OS_ERR_NONE && polled.valid) {
        site_info = polled;
        if (rtc_initialized) {
            (void)os_hal_rtc_set(site_info.utc_epoch_seconds);
        }
    } else {
        if (rtc_initialized) {
            uint32_t utc = 0U;
            if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
                site_info.utc_epoch_seconds = utc;
            }
        }
        site_info.valid = false;
    }
}

static void check_faults(void) {
    bool fault = false;
    for (uint8_t axis = 0; axis < OS_NUM_AXES; axis++) {
        if (os_hal_limit_is_triggered(axis)) {
            limit_triggered[axis] = true;
            fault = true;
        }
        if (hal_axis[axis].drive_fault) {
            fault = true;
        }
    }
    if (fault) {
        enter_fault();
    }
}

static void advance_goto(void) {
    if (!goto_motion_active || system_state != OS_STATE_GOTO) {
        return;
    }
    goto_elapsed_ticks++;
    int64_t ra_abs = (int64_t)goto_target_steps.ra_steps - (int64_t)hal_axis[0].position_steps;
    if (ra_abs < 0) {
        ra_abs = -ra_abs;
    }
    int64_t dec_abs = (int64_t)goto_target_steps.dec_steps - (int64_t)hal_axis[1].position_steps;
    if (dec_abs < 0) {
        dec_abs = -dec_abs;
    }
    uint32_t ra_step = compute_slew_step_inc(ra_abs, goto_elapsed_ticks);
    uint32_t dec_step = compute_slew_step_inc(dec_abs, goto_elapsed_ticks);
    move_axis_toward(0, goto_target_steps.ra_steps, ra_step);
    move_axis_toward(1, goto_target_steps.dec_steps, dec_step);
    if (hal_axis[0].position_steps == goto_target_steps.ra_steps &&
        hal_axis[1].position_steps == goto_target_steps.dec_steps) {
        goto_finalize();
    }
}

static void advance_park_motion(void) {
    if (!park_motion_active || system_state != OS_STATE_GOTO) {
        return;
    }
    park_elapsed_ticks++;
    int64_t ra_abs = (int64_t)park_target_steps.ra_steps - (int64_t)hal_axis[0].position_steps;
    if (ra_abs < 0) {
        ra_abs = -ra_abs;
    }
    int64_t dec_abs = (int64_t)park_target_steps.dec_steps - (int64_t)hal_axis[1].position_steps;
    if (dec_abs < 0) {
        dec_abs = -dec_abs;
    }
    uint32_t ra_step = compute_slew_step_inc(ra_abs, park_elapsed_ticks);
    uint32_t dec_step = compute_slew_step_inc(dec_abs, park_elapsed_ticks);
    move_axis_toward(0, park_target_steps.ra_steps, ra_step);
    move_axis_toward(1, park_target_steps.dec_steps, dec_step);
    if (hal_axis[0].position_steps == park_target_steps.ra_steps &&
        hal_axis[1].position_steps == park_target_steps.dec_steps) {
        park_finalize();
    }
}

static void advance_guide(void) {
    if (!guide_pulse.active) {
        return;
    }
    advance_axis_position_by_frequency(active_guide_axis, OS_LOOP_TICK_MS);
    if (guide_pulse.duration_ms > OS_LOOP_TICK_MS) {
        guide_pulse.duration_ms -= OS_LOOP_TICK_MS;
    } else {
        complete_guide();
    }
}

static void advance_manual_timeout(void) {
    if (!manual_motion_active) {
        return;
    }
    advance_axis_position_by_frequency(manual_axis, OS_LOOP_TICK_MS);
    if (manual_remaining_ms > OS_LOOP_TICK_MS) {
        manual_remaining_ms -= OS_LOOP_TICK_MS;
    } else {
        stop_manual_motion();
    }
}

static bool solve_linear_ls(const double *a, const double *b, int m, int n, double *x) {
    double q[OS_CALIBRATION_MAX_STARS][3];
    double r[3][3];
    double v[OS_CALIBRATION_MAX_STARS];
    double y[3];

    for (int col = 0; col < n; col++) {
        for (int row = 0; row < m; row++) {
            v[row] = a[row * n + col];
        }
        for (int k = 0; k < col; k++) {
            r[k][col] = 0.0;
            for (int row = 0; row < m; row++) {
                r[k][col] += q[row][k] * v[row];
            }
            for (int row = 0; row < m; row++) {
                v[row] -= r[k][col] * q[row][k];
            }
        }
        double norm = 0.0;
        for (int row = 0; row < m; row++) {
            norm += v[row] * v[row];
        }
        norm = sqrt(norm);
        if (norm < 1e-12) {
            return false;
        }
        r[col][col] = norm;
        for (int row = 0; row < m; row++) {
            q[row][col] = v[row] / norm;
        }
    }

    for (int col = 0; col < n; col++) {
        y[col] = 0.0;
        for (int row = 0; row < m; row++) {
            y[col] += q[row][col] * b[row];
        }
    }

    for (int col = n - 1; col >= 0; col--) {
        x[col] = y[col];
        for (int row = col + 1; row < n; row++) {
            x[col] -= r[col][row] * x[row];
        }
        x[col] /= r[col][col];
    }
    return true;
}

static int alignment_design_size(os_align_mode_t mode) {
    if (mode == OS_ALIGN_1STAR) {
        return 1;
    }
    if (mode == OS_ALIGN_2STAR) {
        return 2;
    }
    return 3;
}

static int alignment_min_stars(os_align_mode_t mode) {
    if (mode == OS_ALIGN_1STAR) {
        return 1;
    }
    if (mode == OS_ALIGN_2STAR) {
        return 2;
    }
    return 3;
}

static void build_design_matrix(const os_equatorial_coord_t *stars, double *a, int count, int design_size, bool for_dec) {
    for (int row = 0; row < count; row++) {
        double ra_arcsec = (double)stars[row].ra_hours * 54000.0;
        double dec_arcsec = (double)stars[row].dec_degrees * 3600.0;
        if (design_size == 1) {
            a[row * design_size] = 1.0;
        } else if (design_size == 2) {
            a[row * design_size + 0] = for_dec ? dec_arcsec : ra_arcsec;
            a[row * design_size + 1] = 1.0;
        } else {
            a[row * design_size + 0] = ra_arcsec;
            a[row * design_size + 1] = dec_arcsec;
            a[row * design_size + 2] = 1.0;
        }
    }
}

static bool check_3_star_degeneracy(const double *a) {
    double a0 = a[0 * 3 + 0];
    double a1 = a[0 * 3 + 1];
    double a2 = a[0 * 3 + 2];
    double a3 = a[1 * 3 + 0];
    double a4 = a[1 * 3 + 1];
    double a5 = a[1 * 3 + 2];
    double a6 = a[2 * 3 + 0];
    double a7 = a[2 * 3 + 1];
    double a8 = a[2 * 3 + 2];
    double det = a0 * (a4 * a8 - a5 * a7) - a1 * (a3 * a8 - a5 * a6) + a2 * (a3 * a7 - a4 * a6);
    return fabs(det) < OS_CALIBRATION_DET_EPSILON;
}

static double predict_step(const os_equatorial_coord_t *star, const double *x, os_align_mode_t mode, bool for_dec) {
    double ra_arcsec = (double)star->ra_hours * 54000.0;
    double dec_arcsec = (double)star->dec_degrees * 3600.0;
    if (mode == OS_ALIGN_1STAR) {
        return x[0];
    }
    if (mode == OS_ALIGN_2STAR) {
        double coord_x = for_dec ? dec_arcsec : ra_arcsec;
        return x[0] * coord_x + x[1];
    }
    (void)for_dec;
    return x[0] * ra_arcsec + x[1] * dec_arcsec + x[2];
}

static bool parse_ra_string(const char *s, size_t len, float *hours) {
    if (s == NULL || hours == NULL || len == 0U || len >= 32U) {
        return false;
    }
    char buf[32];
    memcpy(buf, s, len);
    buf[len] = '\0';
    if (strchr(buf, ':') != NULL) {
        int h = 0, m = 0;
        float sec = 0.0f;
        if (sscanf(buf, "%d:%d:%f", &h, &m, &sec) == 3) {
            *hours = (float)h + (float)m / 60.0f + sec / 3600.0f;
            return true;
        }
        return false;
    }
    float f = 0.0f;
    if (sscanf(buf, "%f", &f) == 1) {
        *hours = f;
        return true;
    }
    return false;
}

static bool parse_dec_string(const char *s, size_t len, float *degrees) {
    if (s == NULL || degrees == NULL || len == 0U || len >= 32U) {
        return false;
    }
    char buf[32];
    memcpy(buf, s, len);
    buf[len] = '\0';
    char *p = buf;
    int sign = 1;
    if (*p == '*') {
        p++;
    }
    if (*p == '+' || *p == '-') {
        if (*p == '-') {
            sign = -1;
        }
        p++;
    }
    if (strchr(p, ':') != NULL) {
        int d = 0, m = 0;
        float sec = 0.0f;
        if (sscanf(p, "%d:%d:%f", &d, &m, &sec) == 3) {
            int ad = d < 0 ? -d : d;
            *degrees = (float)sign * ((float)ad + (float)m / 60.0f + sec / 3600.0f);
            return true;
        }
        return false;
    }
    float f = 0.0f;
    if (sscanf(p, "%f", &f) == 1) {
        float af = f < 0.0f ? -f : f;
        *degrees = (float)sign * af;
        return true;
    }
    return false;
}

os_error_t os_init(void) {
    os_error_t err;
    reset_runtime_flags();
    system_state = OS_STATE_INITIALIZING;

    err = os_hal_nvm_init();
    if (err != OS_ERR_NONE) {
        enter_fault();
        return OS_ERR_NVM_FAULT;
    }

    if (load_nvm_config() != OS_ERR_NONE) {
        enter_fault();
        return OS_ERR_NVM_FAULT;
    }
    if (load_nvm_calibration() != OS_ERR_NONE) {
        enter_fault();
        return OS_ERR_NVM_FAULT;
    }
    if (load_nvm_pec() != OS_ERR_NONE) {
        enter_fault();
        return OS_ERR_NVM_FAULT;
    }

    for (uint8_t channel = 0; channel < OS_NUM_CHANNELS; channel++) {
        (void)os_hal_comm_init(channel);
    }

    for (uint8_t axis = 0; axis < OS_NUM_AXES; axis++) {
        err = os_hal_motor_init(axis);
        if (err != OS_ERR_NONE) {
            enter_fault();
            return err;
        }
    }

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    err = os_hal_timer_motor_init();
    if (err != OS_ERR_NONE) {
        enter_fault();
        return err;
    }

    (void)os_hal_gps_poll(&site_info);
    if (!site_info.valid) {
        uint32_t utc = 0U;
        if (rtc_initialized && os_hal_rtc_read(&utc) == OS_ERR_NONE) {
            site_info.utc_epoch_seconds = utc;
        }
        site_info.valid = false;
    }

    tracking_enabled = true;
    tracking_rate = OS_TRACK_RATE_SIDEREAL;
    tracking_custom_factor = 1.0f;
    system_state = OS_STATE_IDLE_TRACKING;
    (void)os_hal_motor_enable(0, true);
    (void)os_hal_motor_enable(1, true);
    (void)apply_tracking_motor_frequency();
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    loop_tick_ms += OS_LOOP_TICK_MS;
    poll_commands();
    update_site_time();
    check_faults();
    if (system_state == OS_STATE_FAULT) {
        return;
    }

    if (park_motion_active) {
        advance_park_motion();
    }
    if (goto_motion_active) {
        advance_goto();
    }
    if (guide_pulse.active) {
        advance_guide();
    }
    if (manual_motion_active) {
        advance_manual_timeout();
    }

    if (tracking_enabled && system_state == OS_STATE_IDLE_TRACKING &&
        !guide_pulse.active && !manual_motion_active && !goto_motion_active &&
        !park_motion_active) {
        (void)apply_tracking_motor_frequency();
        advance_pec_phase();
        advance_axis_position_by_frequency(0, OS_LOOP_TICK_MS);
    }
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!channel_valid(source_channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *reply_length = 0U;
    if (length == 0U || length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length < 2U || command[0] != OS_LX200_CMD_PREFIX || command[length - 1U] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    char body[OS_MAX_COMMAND_LENGTH];
    size_t body_len = length - 2U;
    if (body_len >= sizeof(body)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(body, command + 1, body_len);
    body[body_len] = '\0';

    if (strcmp(body, "GR") == 0) {
        os_equatorial_coord_t coord;
        os_error_t err = os_query_coordinates(&coord);
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "RA %g#", (double)coord.ra_hours);
            *reply_length = strlen(reply_buffer);
        }
        return err;
    }
    if (strcmp(body, "GD") == 0) {
        os_equatorial_coord_t coord;
        os_error_t err = os_query_coordinates(&coord);
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "Dec %g#", (double)coord.dec_degrees);
            *reply_length = strlen(reply_buffer);
        }
        return err;
    }
    if (strcmp(body, "GVP") == 0) {
        uint8_t major, minor, patch;
        os_error_t err = os_query_firmware_version(&major, &minor, &patch);
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "%u.%u.%u#",
                     (unsigned)major, (unsigned)minor, (unsigned)patch);
            *reply_length = strlen(reply_buffer);
        }
        return err;
    }
    if (strcmp(body, "MS") == 0) {
        os_error_t err = os_goto_equatorial(command_target_eq);
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1");
            *reply_length = strlen(reply_buffer);
        }
        return err;
    }
    if (strncmp(body, "Sr", 2) == 0) {
        float ra = 0.0f;
        if (!parse_ra_string(body + 2, body_len - 2U, &ra)) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        if (!equatorial_coord_valid(ra, command_target_eq.dec_degrees)) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        command_target_eq.ra_hours = ra;
        snprintf(reply_buffer, reply_buffer_size, "1");
        *reply_length = strlen(reply_buffer);
        return OS_ERR_NONE;
    }
    if (strncmp(body, "Sd", 2) == 0) {
        float dec = 0.0f;
        if (!parse_dec_string(body + 2, body_len - 2U, &dec)) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        if (!equatorial_coord_valid(command_target_eq.ra_hours, dec)) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        command_target_eq.dec_degrees = dec;
        snprintf(reply_buffer, reply_buffer_size, "1");
        *reply_length = strlen(reply_buffer);
        return OS_ERR_NONE;
    }
    if (strncmp(body, "Mg", 2) == 0) {
        if (body_len < 3U) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        os_direction_t dir;
        switch (body[2]) {
            case 'n': dir = OS_DIRECTION_NORTH; break;
            case 's': dir = OS_DIRECTION_SOUTH; break;
            case 'e': dir = OS_DIRECTION_EAST; break;
            case 'w': dir = OS_DIRECTION_WEST; break;
            default: return OS_ERR_INVALID_ARGUMENT;
        }
        uint32_t duration_ms = 0U;
        if (sscanf(body + 3, "%u", &duration_ms) != 1) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        os_error_t err = os_guide_pulse(dir, duration_ms);
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1");
            *reply_length = strlen(reply_buffer);
        }
        return err;
    }
    if (strcmp(body, "Me") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1");
            *reply_length = strlen(reply_buffer);
        }
        return err;
    }
    if (strcmp(body, "Mw") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1");
            *reply_length = strlen(reply_buffer);
        }
        return err;
    }
    if (strcmp(body, "Mn") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1");
            *reply_length = strlen(reply_buffer);
        }
        return err;
    }
    if (strcmp(body, "Ms") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1");
            *reply_length = strlen(reply_buffer);
        }
        return err;
    }
    if (strcmp(body, "Q") == 0) {
        os_error_t err = os_move_stop();
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1");
            *reply_length = strlen(reply_buffer);
        }
        return err;
    }
    if (strcmp(body, "hP") == 0) {
        os_error_t err = os_park();
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1");
            *reply_length = strlen(reply_buffer);
        }
        return err;
    }
    if (strcmp(body, "hO") == 0) {
        os_error_t err = os_unpark();
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1");
            *reply_length = strlen(reply_buffer);
        }
        return err;
    }
    if (strcmp(body, "Te") == 0) {
        os_error_t err = os_tracking_enable();
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1");
            *reply_length = strlen(reply_buffer);
        }
        return err;
    }
    if (strcmp(body, "Td") == 0) {
        os_error_t err = os_tracking_disable();
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1");
            *reply_length = strlen(reply_buffer);
        }
        return err;
    }

    return OS_ERR_COMMAND_FORMAT;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (!equatorial_coord_valid(target.ra_hours, target.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (system_state == OS_STATE_PARKED || system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    os_motor_position_t target_steps;
    equatorial_to_motor_steps(&target, &target_steps);
    os_motor_position_t current;
    current.ra_steps = os_hal_motor_get_position(0);
    current.dec_steps = os_hal_motor_get_position(1);

    if (target_steps.ra_steps == current.ra_steps && target_steps.dec_steps == current.dec_steps) {
        system_state = OS_STATE_IDLE_TRACKING;
        goto_motion_active = false;
        goto_elapsed_ticks = 0U;
        if (tracking_enabled) {
            (void)apply_tracking_motor_frequency();
        }
        return OS_ERR_NONE;
    }

    goto_target_steps = target_steps;
    goto_elapsed_ticks = 0U;
    goto_motion_active = true;
    park_motion_active = false;
    manual_motion_active = false;
    guide_pulse.active = false;
    guide_pulse.duration_ms = 0U;
    guide_pulse.direction_east = false;
    guide_pulse.direction_north = false;
    guide_pulse.dec_priority = false;
    system_state = OS_STATE_GOTO;

    (void)os_hal_motor_enable(0, true);
    (void)os_hal_motor_enable(1, true);
    (void)os_hal_motor_set_direction(0, target_steps.ra_steps >= current.ra_steps);
    (void)os_hal_motor_set_direction(1, target_steps.dec_steps >= current.dec_steps);
    (void)os_hal_motor_set_frequency(0, OS_GOTO_MIN_STEP * 100U);
    (void)os_hal_motor_set_frequency(1, OS_GOTO_MIN_STEP * 100U);
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < OS_DEC_MIN_DEG || target.altitude_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (system_state == OS_STATE_PARKED || system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    os_motor_position_t target_steps;
    horizontal_to_motor_steps(&target, &target_steps);
    os_motor_position_t current;
    current.ra_steps = os_hal_motor_get_position(0);
    current.dec_steps = os_hal_motor_get_position(1);

    if (target_steps.ra_steps == current.ra_steps && target_steps.dec_steps == current.dec_steps) {
        system_state = OS_STATE_IDLE_TRACKING;
        goto_motion_active = false;
        goto_elapsed_ticks = 0U;
        if (tracking_enabled) {
            (void)apply_tracking_motor_frequency();
        }
        return OS_ERR_NONE;
    }

    goto_target_steps = target_steps;
    goto_elapsed_ticks = 0U;
    goto_motion_active = true;
    park_motion_active = false;
    manual_motion_active = false;
    guide_pulse.active = false;
    guide_pulse.duration_ms = 0U;
    guide_pulse.direction_east = false;
    guide_pulse.direction_north = false;
    guide_pulse.dec_priority = false;
    system_state = OS_STATE_GOTO;

    (void)os_hal_motor_enable(0, true);
    (void)os_hal_motor_enable(1, true);
    (void)os_hal_motor_set_direction(0, target_steps.ra_steps >= current.ra_steps);
    (void)os_hal_motor_set_direction(1, target_steps.dec_steps >= current.dec_steps);
    (void)os_hal_motor_set_frequency(0, OS_GOTO_MIN_STEP * 100U);
    (void)os_hal_motor_set_frequency(1, OS_GOTO_MIN_STEP * 100U);
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    goto_motion_active = false;
    park_motion_active = false;
    manual_motion_active = false;
    guide_pulse.active = false;
    guide_pulse.duration_ms = 0U;
    guide_pulse.direction_east = false;
    guide_pulse.direction_north = false;
    guide_pulse.dec_priority = false;
    goto_elapsed_ticks = 0U;
    park_elapsed_ticks = 0U;
    stop_all_pulses();
    if (system_state == OS_STATE_GOTO || system_state == OS_STATE_MANUAL_MOTION) {
        system_state = OS_STATE_IDLE_TRACKING;
        if (tracking_enabled) {
            (void)apply_tracking_motor_frequency();
        }
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (rate < OS_TRACK_RATE_SIDEREAL || rate > OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM) {
        if (!(custom_factor > 0.0f)) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        tracking_custom_factor = custom_factor;
    } else {
        tracking_custom_factor = 1.0f;
    }
    tracking_rate = rate;
    if (tracking_enabled && system_state == OS_STATE_IDLE_TRACKING) {
        (void)apply_tracking_motor_frequency();
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = tracking_rate;
    *custom_factor = tracking_custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    if (system_state == OS_STATE_PARKED || system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    tracking_enabled = true;
    if (system_state == OS_STATE_IDLE_TRACKING) {
        (void)apply_tracking_motor_frequency();
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    tracking_enabled = false;
    stop_all_pulses();
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (!direction_valid(direction)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0U) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (guide_pulse.active) {
        bool new_dec = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);
        bool old_dec = guide_pulse.dec_priority;
        if (old_dec && !new_dec) {
            return OS_ERR_INVALID_STATE;
        }
        if (old_dec == new_dec) {
            return OS_ERR_INVALID_STATE;
        }
    }
    return start_guide_pulse(direction, duration_ms);
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (!(rate_fraction >= OS_GUIDE_RATE_MIN && rate_fraction <= OS_GUIDE_RATE_MAX)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    guide_rate_fraction = rate_fraction;
    if (guide_pulse.active) {
        guide_pulse.rate_fraction = guide_rate_fraction;
    }
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *pulse = guide_pulse;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (!align_mode_valid(mode)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (system_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    alignment_mode = mode;
    alignment_star_count = 0;
    calibration_residual_computed = false;
    calibration_residual_arcsec = 0.0;
    reset_calibration_fields();
    system_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (!equatorial_coord_valid(star_coord.ra_hours, star_coord.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (system_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (alignment_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }
    alignment_samples[alignment_star_count].star_coord = star_coord;
    alignment_samples[alignment_star_count].motor_pos = motor_pos;
    alignment_star_count++;
    calibration_residual_computed = false;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (system_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    int min_stars = alignment_min_stars(alignment_mode);
    if ((int)alignment_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    int design_size = alignment_design_size(alignment_mode);
    double design[OS_CALIBRATION_MAX_STARS * 3];
    os_equatorial_coord_t star_coords[OS_CALIBRATION_MAX_STARS];
    for (int i = 0; i < (int)alignment_star_count; i++) {
        star_coords[i] = alignment_samples[i].star_coord;
    }

    if ((alignment_mode == OS_ALIGN_3STAR || alignment_mode == OS_ALIGN_NSTAR) &&
        alignment_star_count == 3U && design_size == 3) {
        build_design_matrix(star_coords, design, 3, design_size, false);
        if (check_3_star_degeneracy(design)) {
            return OS_ERR_CALIBRATION_FAILED;
        }
    }

    build_design_matrix(star_coords, design, (int)alignment_star_count, design_size, false);
    double b_ra[OS_CALIBRATION_MAX_STARS];
    for (int i = 0; i < (int)alignment_star_count; i++) {
        b_ra[i] = (double)alignment_samples[i].motor_pos.ra_steps;
    }
    double coeff_ra[3] = {0.0, 0.0, 0.0};
    if (!solve_linear_ls(design, b_ra, (int)alignment_star_count, design_size, coeff_ra)) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    build_design_matrix(star_coords, design, (int)alignment_star_count, design_size, true);
    double b_dec[OS_CALIBRATION_MAX_STARS];
    for (int i = 0; i < (int)alignment_star_count; i++) {
        b_dec[i] = (double)alignment_samples[i].motor_pos.dec_steps;
    }
    double coeff_dec[3] = {0.0, 0.0, 0.0};
    if (!solve_linear_ls(design, b_dec, (int)alignment_star_count, design_size, coeff_dec)) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    double new_matrix_ra_to_ra = 0.0;
    double new_matrix_ra_to_dec = 0.0;
    double new_matrix_dec_to_ra = 0.0;
    double new_matrix_dec_to_dec = 0.0;
    double new_offset_ra = 0.0;
    double new_offset_dec = 0.0;

    if (alignment_mode == OS_ALIGN_1STAR) {
        double sum_ra_sky = 0.0;
        double sum_dec_sky = 0.0;
        double sum_ra_steps = 0.0;
        double sum_dec_steps = 0.0;
        for (int i = 0; i < (int)alignment_star_count; i++) {
            sum_ra_sky += (double)star_coords[i].ra_hours * 54000.0;
            sum_dec_sky += (double)star_coords[i].dec_degrees * 3600.0;
            sum_ra_steps += (double)alignment_samples[i].motor_pos.ra_steps;
            sum_dec_steps += (double)alignment_samples[i].motor_pos.dec_steps;
        }
        double mean_ra_sky = sum_ra_sky / (double)alignment_star_count;
        double mean_dec_sky = sum_dec_sky / (double)alignment_star_count;
        double mean_ra_steps = sum_ra_steps / (double)alignment_star_count;
        double mean_dec_steps = sum_dec_steps / (double)alignment_star_count;
        double scale_ra = (double)OS_STEPS_PER_RA_HOUR / 54000.0;
        double scale_dec = (double)OS_STEPS_PER_DEC_DEGREE / 3600.0;
        new_matrix_ra_to_ra = scale_ra;
        new_matrix_dec_to_dec = scale_dec;
        new_offset_ra = mean_ra_steps - scale_ra * mean_ra_sky;
        new_offset_dec = mean_dec_steps - scale_dec * mean_dec_sky;
    } else if (alignment_mode == OS_ALIGN_2STAR) {
        new_matrix_ra_to_ra = coeff_ra[0];
        new_offset_ra = coeff_ra[1];
        new_matrix_dec_to_dec = coeff_dec[0];
        new_offset_dec = coeff_dec[1];
    } else {
        new_matrix_ra_to_ra = coeff_ra[0];
        new_matrix_ra_to_dec = coeff_ra[1];
        new_offset_ra = coeff_ra[2];
        new_matrix_dec_to_ra = coeff_dec[0];
        new_matrix_dec_to_dec = coeff_dec[1];
        new_offset_dec = coeff_dec[2];
    }

    double residual = 0.0;
    if ((alignment_mode == OS_ALIGN_3STAR || alignment_mode == OS_ALIGN_NSTAR) &&
        alignment_star_count == 3U) {
        residual = 0.0;
    } else {
        double sum_sq = 0.0;
        for (int i = 0; i < (int)alignment_star_count; i++) {
            double pred_ra;
            double pred_dec;
            if (alignment_mode == OS_ALIGN_1STAR) {
                double ra_arcsec = (double)star_coords[i].ra_hours * 54000.0;
                double dec_arcsec = (double)star_coords[i].dec_degrees * 3600.0;
                pred_ra = new_matrix_ra_to_ra * ra_arcsec + new_offset_ra;
                pred_dec = new_matrix_dec_to_dec * dec_arcsec + new_offset_dec;
            } else {
                pred_ra = predict_step(&star_coords[i], coeff_ra, alignment_mode, false);
                pred_dec = predict_step(&star_coords[i], coeff_dec, alignment_mode, true);
            }
            double diff_ra = pred_ra - (double)alignment_samples[i].motor_pos.ra_steps;
            double diff_dec = pred_dec - (double)alignment_samples[i].motor_pos.dec_steps;
            sum_sq += diff_ra * diff_ra + diff_dec * diff_dec;
        }
        double rms_steps = sqrt(sum_sq / (2.0 * (double)alignment_star_count));
        residual = rms_steps * OS_CALIBRATION_ARCSEC_PER_STEP;
    }

    if ((int)alignment_star_count >= 4 && residual > OS_CALIBRATION_MAX_RESIDUAL_ARCSEC) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    calibration.matrix_ra_to_ra = (float)new_matrix_ra_to_ra;
    calibration.matrix_ra_to_dec = (float)new_matrix_ra_to_dec;
    calibration.matrix_dec_to_ra = (float)new_matrix_dec_to_ra;
    calibration.matrix_dec_to_dec = (float)new_matrix_dec_to_dec;
    calibration.offset_ra_arcsec = (float)new_offset_ra;
    calibration.offset_dec_arcsec = (float)new_offset_dec;
    calibration.valid = true;
    calibration_residual_arcsec = residual;
    calibration_residual_computed = true;

    if (save_nvm_calibration() != OS_ERR_NONE) {
        calibration.valid = false;
        return OS_ERR_NVM_FAULT;
    }

    system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!calibration_residual_computed) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = (float)calibration_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    if (system_state == OS_STATE_ALIGNMENT) {
        system_state = OS_STATE_IDLE_TRACKING;
        alignment_star_count = 0;
        calibration_residual_computed = false;
    }
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (system_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    if (system_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    os_motor_position_t target_steps;
    os_equatorial_coord_t park_coord;
    if (custom_park_position_set) {
        park_coord = park_position_eq;
    } else {
        park_coord.ra_hours = 0.0f;
        park_coord.dec_degrees = 90.0f;
    }
    equatorial_to_motor_steps(&park_coord, &target_steps);

    os_motor_position_t current;
    current.ra_steps = os_hal_motor_get_position(0);
    current.dec_steps = os_hal_motor_get_position(1);
    park_target_steps = target_steps;
    park_elapsed_ticks = 0U;
    park_motion_active = true;
    goto_motion_active = false;
    manual_motion_active = false;
    guide_pulse.active = false;
    guide_pulse.duration_ms = 0U;
    guide_pulse.direction_east = false;
    guide_pulse.direction_north = false;
    guide_pulse.dec_priority = false;
    system_state = OS_STATE_GOTO;

    (void)os_hal_motor_enable(0, true);
    (void)os_hal_motor_enable(1, true);
    (void)os_hal_motor_set_direction(0, target_steps.ra_steps >= current.ra_steps);
    (void)os_hal_motor_set_direction(1, target_steps.dec_steps >= current.dec_steps);
    (void)os_hal_motor_set_frequency(0, OS_GOTO_MIN_STEP * 100U);
    (void)os_hal_motor_set_frequency(1, OS_GOTO_MIN_STEP * 100U);
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (system_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    for (uint8_t channel = 0; channel < OS_NUM_CHANNELS; channel++) {
        (void)os_hal_comm_init(channel);
    }
    resume_all_comm_rx();
    (void)os_hal_motor_enable(0, true);
    (void)os_hal_motor_enable(1, true);
    if (site_info.valid && rtc_initialized) {
        (void)os_hal_rtc_set(site_info.utc_epoch_seconds);
    }
    tracking_enabled = true;
    park_motion_active = false;
    park_elapsed_ticks = 0U;
    system_state = OS_STATE_IDLE_TRACKING;
    (void)apply_tracking_motor_frequency();
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!equatorial_coord_valid(park_pos.ra_hours, park_pos.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    park_position_eq = park_pos;
    custom_park_position_set = true;
    if (save_nvm_config() != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (!direction_valid(direction)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!speed_valid(speed)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (system_state != OS_STATE_IDLE_TRACKING && system_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }

    bool is_dec = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);
    uint8_t axis = is_dec ? 1U : 0U;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    uint32_t freq = manual_speed_frequency_hz(speed);
    if (freq == 0U) {
        return OS_ERR_INVALID_STATE;
    }

    manual_axis = axis;
    manual_motion_active = true;
    manual_remaining_ms = OS_MANUAL_TIMEOUT_MS;
    system_state = OS_STATE_MANUAL_MOTION;
    bool forward = is_dec ? (direction == OS_DIRECTION_NORTH) : (direction == OS_DIRECTION_EAST);
    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, freq);
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (manual_motion_active || system_state == OS_STATE_MANUAL_MOTION) {
        stop_manual_motion();
    }
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (!(arcsec_per_sec > 0.0f) ||
        arcsec_per_sec > OS_MAX_CUSTOM_MANUAL_ARCSEC_PER_SEC) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    custom_manual_arcsec_per_sec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = system_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    os_motor_position_t pos;
    pos.ra_steps = os_hal_motor_get_position(0);
    pos.dec_steps = os_hal_motor_get_position(1);
    motor_steps_to_equatorial(&pos, coord);
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = site_info;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = os_hal_motor_get_position(0);
    pos->dec_steps = os_hal_motor_get_position(1);
    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor, uint8_t *patch) {
    if (major == NULL || minor == NULL || patch == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *major = OS_FIRMWARE_VERSION_MAJOR;
    *minor = OS_FIRMWARE_VERSION_MINOR;
    *patch = OS_FIRMWARE_VERSION_PATCH;
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *moving) {
    if (moving == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *moving = goto_motion_active || park_motion_active || manual_motion_active || guide_pulse.active ||
              (tracking_enabled && system_state == OS_STATE_IDLE_TRACKING);
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *locked = site_info.valid;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    pec_enabled = enable;
    if (tracking_enabled && system_state == OS_STATE_IDLE_TRACKING) {
        (void)apply_tracking_motor_frequency();
    }
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!table->valid) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pec_table = *table;
    if (save_nvm_pec() != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *table = pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (!(worm_phase_deg >= 0.0f && worm_phase_deg <= 360.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int index;
    if (worm_phase_deg >= 360.0f) {
        index = 359;
    } else {
        index = (int)worm_phase_deg;
    }
    if (index < 0 || index >= OS_PEC_TABLE_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pec_table.corrections[index] = error_arcsec;
    pec_table.valid = true;
    if (save_nvm_pec() != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *calib = calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    reset_calibration_fields();
    if (save_nvm_calibration() != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_init(uint8_t axis) {
    if (!axis_valid(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    hal_axis[axis].initialized = true;
    hal_axis[axis].enabled = false;
    hal_axis[axis].direction_forward = false;
    hal_axis[axis].frequency_hz = 0U;
    hal_axis[axis].drive_fault = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (!axis_valid(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!hal_axis[axis].initialized) {
        return OS_ERR_INVALID_STATE;
    }
    if (hal_axis[axis].drive_fault) {
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    if (frequency_hz > OS_HAL_MOTOR_MAX_FREQ_HZ) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    hal_axis[axis].frequency_hz = frequency_hz;
    if (frequency_hz == 0U) {
        hal_axis[axis].enabled = false;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (!axis_valid(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!hal_axis[axis].initialized) {
        return OS_ERR_INVALID_STATE;
    }
    hal_axis[axis].direction_forward = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (!axis_valid(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!hal_axis[axis].initialized) {
        return OS_ERR_INVALID_STATE;
    }
    if (enable && hal_axis[axis].drive_fault) {
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    hal_axis[axis].enabled = enable;
    if (!enable) {
        hal_axis[axis].frequency_hz = 0U;
        hal_axis[axis].direction_forward = false;
    }
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (!axis_valid(axis)) {
        return 0;
    }
    return hal_axis[axis].position_steps;
}

os_error_t os_hal_gps_init(void) {
    gps_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!gps_initialized) {
        return OS_ERR_INVALID_STATE;
    }
    site->valid = false;
    return OS_ERR_GPS_NO_SIGNAL;
}

os_error_t os_hal_rtc_init(void) {
    rtc_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!rtc_initialized) {
        return OS_ERR_INVALID_STATE;
    }
    *utc_epoch_seconds = rtc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    if (!rtc_initialized) {
        return OS_ERR_INVALID_STATE;
    }
    rtc_epoch_seconds = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    limit_initialized = true;
    limit_triggered[0] = false;
    limit_triggered[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (!axis_valid(axis)) {
        return true;
    }
    return limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void) {
    nvm_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!nvm_initialized) {
        return OS_ERR_NVM_FAULT;
    }
    uint32_t end = (uint32_t)offset + (uint32_t)length;
    if (end > OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(data, &nvm_image[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!nvm_initialized) {
        return OS_ERR_NVM_FAULT;
    }
    uint32_t end = (uint32_t)offset + (uint32_t)length;
    if (end > OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(&nvm_image[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (!channel_valid(channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    hal_comm[channel].initialized = true;
    hal_comm[channel].suspended = false;
    hal_comm[channel].rx_count = 0U;
    hal_comm[channel].rx_head = 0U;
    hal_comm[channel].tx_count = 0U;
    memset(hal_comm[channel].rx, 0, sizeof(hal_comm[channel].rx));
    memset(hal_comm[channel].tx, 0, sizeof(hal_comm[channel].tx));
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if (!channel_valid(channel)) {
        return 0;
    }
    if (!hal_comm[channel].initialized || hal_comm[channel].suspended) {
        return 0;
    }
    return (int16_t)hal_comm[channel].rx_count;
}

char os_hal_comm_read(uint8_t channel) {
    if (!channel_valid(channel)) {
        return '\0';
    }
    if (!hal_comm[channel].initialized || hal_comm[channel].suspended || hal_comm[channel].rx_count == 0U) {
        return '\0';
    }
    char c = hal_comm[channel].rx[hal_comm[channel].rx_head];
    hal_comm[channel].rx_head = (uint16_t)(hal_comm[channel].rx_head + 1U);
    hal_comm[channel].rx_count--;
    return c;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (!channel_valid(channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (data == NULL && length > 0U) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length > (size_t)OS_MAX_REPLY_LENGTH) {
        return OS_ERR_TIMEOUT;
    }
    for (size_t i = 0; i < length; i++) {
        hal_comm[channel].tx[hal_comm[channel].tx_count++ % OS_MAX_REPLY_LENGTH] = data[i];
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) {
    last_buzzer_duration_ms = duration_ms;
    last_buzzer_count = count;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void) {
    motor_timer_initialized = true;
    return OS_ERR_NONE;
}
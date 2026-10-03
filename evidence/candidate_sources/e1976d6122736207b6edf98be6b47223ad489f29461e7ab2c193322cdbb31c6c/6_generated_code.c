#include "6_generated_code.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define OS_PRIMARY_AXIS_COUNT       2U
#define OS_PRIMARY_AXIS_RA          0U
#define OS_PRIMARY_AXIS_DEC         1U
#define OS_STEPS_PER_DEGREE         1000.0
#define OS_LOOP_TICK_MS             1U
#define OS_MOTION_DEADBAND_STEPS    2
#define OS_GOTO_DEFAULT_HZ          3000U
#define OS_MANUAL_SLOW_HZ           10U
#define OS_MANUAL_MEDIUM_HZ         100U
#define OS_MANUAL_FAST_HZ           1000U
#define OS_MAX_CUSTOM_MANUAL_ASEC   10800.0f
#define OS_CALIBRATION_RESIDUAL_THRESHOLD_ARCSEC 300.0
#define OS_NVM_CALIBRATION_OFFSET   0
#define OS_NVM_CONFIG_OFFSET        OS_NVM_CALIBRATION_SIZE_BYTES
#define OS_WORM_PHASE_ADVANCE_LOOP_DEG 0.0041667f

typedef struct {
    os_equatorial_coord_t star;
    os_motor_position_t motor;
} align_sample_t;

static os_state_t s_state = OS_STATE_INITIALIZING;
static bool s_init_complete = false;
static bool s_config_loaded = false;
static bool s_nvm_ready = false;

static bool s_gps_locked = false;
static bool s_rtc_valid = false;
static uint32_t s_utc_epoch_seconds = 0U;
static double s_site_latitude_deg = 0.0;
static double s_site_longitude_deg = 0.0;
static double s_site_elevation_metres = 0.0;

static bool s_tracking_enabled = false;
static os_track_rate_t s_tracking_rate = OS_TRACK_RATE_SIDEREAL;
static float s_custom_tracking_factor = 1.0f;

static bool s_goto_in_progress = false;
static bool s_goto_abort_requested = false;
static os_equatorial_coord_t s_goto_equatorial_target = {0.0f, 0.0f};
static os_horizontal_coord_t s_goto_horizontal_target = {0.0f, 0.0f};
static int32_t s_motor_target_steps[OS_PRIMARY_AXIS_COUNT] = {0, 0};

static bool s_manual_motion_active = false;
static os_direction_t s_manual_direction = OS_DIRECTION_NORTH;
static os_speed_level_t s_manual_speed = OS_SPEED_SLOW;
static float s_custom_manual_speed_arcsec_per_sec = 15.0f;

static bool s_parking_active = false;
static bool s_parked = false;
static bool s_park_position_set = false;
static os_equatorial_coord_t s_park_position = {0.0f, 90.0f};

static os_guide_pulse_t s_guide_pulse = {0};
static uint32_t s_guide_pulse_remaining_ms = 0U;
static float s_guide_rate_fraction = 0.5f;

static os_align_mode_t s_align_mode = OS_ALIGN_1STAR;
static uint8_t s_align_count = 0U;
static align_sample_t s_align_samples[OS_CALIBRATION_MAX_STARS];
static bool s_calibration_residual_computed = false;
static double s_residual_arcsec = 0.0;
static bool s_calibration_degenerate = false;
static os_calibration_t s_calibration = {0};

static bool s_pec_enabled = false;
static os_pec_table_t s_pec_table = {0};
static float s_worm_phase_deg = 0.0f;

static char s_channel_rx[4][OS_MAX_COMMAND_LENGTH + 1];
static uint8_t s_channel_rx_len[4] = {0U, 0U, 0U, 0U};

static int32_t abs_i32(int32_t value) {
    return (value < 0) ? -value : value;
}

static bool axis_valid(uint8_t axis) {
    return axis < OS_PRIMARY_AXIS_COUNT;
}

static bool radec_valid(float ra_hours, float dec_degrees) {
    return isfinite((double)ra_hours) && isfinite((double)dec_degrees) &&
           ra_hours >= OS_RA_MIN_HOURS && ra_hours <= OS_RA_MAX_HOURS &&
           dec_degrees >= OS_DEC_MIN_DEG && dec_degrees <= OS_DEC_MAX_DEG;
}

static bool horizontal_valid(float az_deg, float alt_deg) {
    return isfinite((double)az_deg) && isfinite((double)alt_deg) &&
           az_deg >= 0.0f && az_deg < 360.0f &&
           alt_deg >= -90.0f && alt_deg <= 90.0f;
}

static int direction_axis(os_direction_t direction) {
    switch (direction) {
    case OS_DIRECTION_NORTH:
    case OS_DIRECTION_SOUTH:
        return 1;
    case OS_DIRECTION_EAST:
    case OS_DIRECTION_WEST:
        return 0;
    default:
        return -1;
    }
}

static bool direction_forward(os_direction_t direction) {
    return (direction == OS_DIRECTION_NORTH) || (direction == OS_DIRECTION_EAST);
}

static void stop_all_motor_pulses(void) {
    (void)os_hal_motor_set_frequency(0U, 0U);
    (void)os_hal_motor_set_frequency(1U, 0U);
}

static void disable_all_motors(void) {
    (void)os_hal_motor_enable(0U, false);
    (void)os_hal_motor_enable(1U, false);
}

static void enter_fault(void) {
    stop_all_motor_pulses();
    disable_all_motors();
    s_state = OS_STATE_FAULT;
}

static uint16_t pec_index_for_phase(float phase_deg) {
    if (phase_deg >= 360.0f) {
        return 0U;
    }
    if (phase_deg < 0.0f) {
        return 0U;
    }
    return (uint16_t)phase_deg;
}

static uint32_t tracking_frequency_hz(void) {
    if (!s_tracking_enabled) {
        return 0U;
    }
    float factor = 1.0f;
    switch (s_tracking_rate) {
    case OS_TRACK_RATE_LUNAR:
        factor = OS_LUNAR_RATE_FACTOR;
        break;
    case OS_TRACK_RATE_SOLAR:
        factor = OS_SOLAR_RATE_FACTOR;
        break;
    case OS_TRACK_RATE_CUSTOM:
        factor = s_custom_tracking_factor;
        break;
    case OS_TRACK_RATE_SIDEREAL:
    default:
        factor = 1.0f;
        break;
    }
    float freq = OS_SIDEREAL_RATE_ARCSEC_PER_SEC * factor;
    freq = freq / 3600.0f * (float)OS_STEPS_PER_DEGREE;
    if (s_pec_enabled && s_pec_table.valid) {
        int16_t correction = s_pec_table.corrections[pec_index_for_phase(s_worm_phase_deg)];
        freq += (float)correction * (float)OS_STEPS_PER_DEGREE / 3600.0f;
    }
    if (freq < 0.0f) {
        freq = 0.0f;
    }
    return (uint32_t)(freq + 0.5f);
}

static void apply_tracking_frequency(void) {
    uint32_t freq = tracking_frequency_hz();
    (void)os_hal_motor_set_frequency(0U, freq);
    (void)os_hal_motor_set_frequency(1U, 0U);
}

static uint32_t goto_frequency_hz(void) {
    return (uint32_t)(OS_GOTO_SPEED_MAX_DEG_PER_SEC * (float)OS_STEPS_PER_DEGREE);
}

static uint32_t manual_frequency_hz(void) {
    uint32_t freq = OS_MANUAL_SLOW_HZ;
    switch (s_manual_speed) {
    case OS_SPEED_SLOW:
        freq = OS_MANUAL_SLOW_HZ;
        break;
    case OS_SPEED_MEDIUM:
        freq = OS_MANUAL_MEDIUM_HZ;
        break;
    case OS_SPEED_FAST:
        freq = OS_MANUAL_FAST_HZ;
        break;
    case OS_SPEED_CUSTOM:
    default:
        freq = (uint32_t)(s_custom_manual_speed_arcsec_per_sec *
                          (float)OS_STEPS_PER_DEGREE / 3600.0f);
        if (freq > goto_frequency_hz()) {
            freq = goto_frequency_hz();
        }
        break;
    }
    return freq;
}

static void start_axis_motion(uint8_t axis, bool forward, uint32_t frequency_hz) {
    if (!axis_valid(axis)) {
        return;
    }
    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, frequency_hz);
}

static int32_t coordinate_to_ra_steps(float ra_hours, float dec_degrees) {
    if (s_calibration.valid) {
        return (int32_t)((double)s_calibration.matrix_ra_to_ra * (double)ra_hours +
                         (double)s_calibration.matrix_ra_to_dec * (double)dec_degrees +
                         (double)s_calibration.offset_ra_arcsec);
    }
    return (int32_t)((double)ra_hours * 15.0 * OS_STEPS_PER_DEGREE);
}

static int32_t coordinate_to_dec_steps(float ra_hours, float dec_degrees) {
    if (s_calibration.valid) {
        return (int32_t)((double)s_calibration.matrix_dec_to_ra * (double)ra_hours +
                         (double)s_calibration.matrix_dec_to_dec * (double)dec_degrees +
                         (double)s_calibration.offset_dec_arcsec);
    }
    return (int32_t)((double)dec_degrees * OS_STEPS_PER_DEGREE);
}

static double wrap_ra_hours(double ra_hours) {
    while (ra_hours < 0.0) {
        ra_hours += 24.0;
    }
    while (ra_hours >= 24.0) {
        ra_hours -= 24.0;
    }
    return ra_hours;
}

static void steps_to_equatorial(int32_t ra_steps, int32_t dec_steps,
                                os_equatorial_coord_t *coord) {
    if (coord == NULL) {
        return;
    }
    if (s_calibration.valid) {
        double m00 = (double)s_calibration.matrix_ra_to_ra;
        double m01 = (double)s_calibration.matrix_ra_to_dec;
        double m10 = (double)s_calibration.matrix_dec_to_ra;
        double m11 = (double)s_calibration.matrix_dec_to_dec;
        double c0 = (double)s_calibration.offset_ra_arcsec;
        double c1 = (double)s_calibration.offset_dec_arcsec;
        double denom = m00 * m11 - m01 * m10;
        if (fabs(denom) > 1e-9) {
            double ra = (m11 * ((double)ra_steps - c0) -
                         m01 * ((double)dec_steps - c1)) / denom;
            double dec = (-m10 * ((double)ra_steps - c0) +
                          m00 * ((double)dec_steps - c1)) / denom;
            coord->ra_hours = (float)wrap_ra_hours(ra);
            coord->dec_degrees = (float)dec;
            if (coord->dec_degrees > 90.0f) coord->dec_degrees = 90.0f;
            if (coord->dec_degrees < -90.0f) coord->dec_degrees = -90.0f;
            return;
        }
    }
    coord->ra_hours = (float)wrap_ra_hours((double)ra_steps /
                                           (15.0 * OS_STEPS_PER_DEGREE));
    coord->dec_degrees = (float)((double)dec_steps / OS_STEPS_PER_DEGREE);
    if (coord->dec_degrees > 90.0f) coord->dec_degrees = 90.0f;
    if (coord->dec_degrees < -90.0f) coord->dec_degrees = -90.0f;
}

static bool motion_target_reached(int32_t current0, int32_t current1,
                                  int32_t target0, int32_t target1) {
    return abs_i32(current0 - target0) <= OS_MOTION_DEADBAND_STEPS &&
           abs_i32(current1 - target1) <= OS_MOTION_DEADBAND_STEPS;
}

static void finish_goto(bool success) {
    stop_all_motor_pulses();
    s_goto_in_progress = false;
    s_goto_abort_requested = false;
    s_state = OS_STATE_IDLE_TRACKING;
    s_tracking_enabled = true;
    apply_tracking_frequency();
    if (success) {
        (void)os_hal_buzzer_beep(80U, 1U);
    }
}

static void finish_parking(void) {
    stop_all_motor_pulses();
    disable_all_motors();
    s_parking_active = false;
    s_parked = true;
    s_tracking_enabled = false;
    s_state = OS_STATE_PARKED;
}

static void apply_guide_pulse_output(void) {
    uint8_t axis = s_guide_pulse.dec_priority ? 1U : 0U;
    bool forward = s_guide_pulse.dec_priority ? s_guide_pulse.direction_north
                                              : s_guide_pulse.direction_east;
    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_direction(axis, forward);
    uint32_t base = tracking_frequency_hz();
    uint32_t bias = (uint32_t)((float)base * s_guide_pulse.rate_fraction);
    uint32_t axis_freq = base + bias;
    if (!s_tracking_enabled) {
        axis_freq = bias;
        (void)os_hal_motor_set_frequency(0U, 0U);
        (void)os_hal_motor_set_frequency(1U, 0U);
    } else if (axis == 0U) {
        (void)os_hal_motor_set_frequency(0U, axis_freq);
        (void)os_hal_motor_set_frequency(1U, 0U);
    } else {
        (void)os_hal_motor_set_frequency(0U, base);
        (void)os_hal_motor_set_frequency(1U, axis_freq);
    }
}

static void clear_guide_pulse(void) {
    s_guide_pulse.active = false;
    s_guide_pulse.duration_ms = 0U;
    s_guide_pulse.direction_east = false;
    s_guide_pulse.direction_north = false;
    s_guide_pulse.dec_priority = false;
    s_guide_pulse_remaining_ms = 0U;
    if (s_state == OS_STATE_IDLE_TRACKING && s_tracking_enabled) {
        apply_tracking_frequency();
    }
}

static uint8_t min_stars_for_mode(os_align_mode_t mode) {
    switch (mode) {
    case OS_ALIGN_1STAR:
        return 1U;
    case OS_ALIGN_2STAR:
        return 2U;
    case OS_ALIGN_3STAR:
    case OS_ALIGN_NSTAR:
    default:
        return 3U;
    }
}

static bool qr_solve_least_squares(const double *a_in, const double *b_in,
                                   int rows, int cols, double *x) {
    if (rows <= 0 || cols <= 0 || cols > rows || cols > 3) {
        return false;
    }
    double A[OS_CALIBRATION_MAX_STARS * 3];
    double B[OS_CALIBRATION_MAX_STARS];
    double v[OS_CALIBRATION_MAX_STARS];
    memcpy(A, a_in, sizeof(double) * (size_t)rows * (size_t)cols);
    memcpy(B, b_in, sizeof(double) * (size_t)rows);
    for (int j = 0; j < cols; ++j) {
        double norm = 0.0;
        for (int i = j; i < rows; ++i) {
            norm += A[i * cols + j] * A[i * cols + j];
        }
        norm = sqrt(norm);
        if (norm < 1e-12) {
            return false;
        }
        double alpha = (A[j * cols + j] > 0.0) ? -norm : norm;
        memset(v, 0, sizeof(v));
        v[j] = A[j * cols + j] - alpha;
        for (int i = j + 1; i < rows; ++i) {
            v[i] = A[i * cols + j];
        }
        double vnorm_sq = 0.0;
        for (int i = j; i < rows; ++i) {
            vnorm_sq += v[i] * v[i];
        }
        if (vnorm_sq < 1e-24) {
            return false;
        }
        for (int k = j; k < cols; ++k) {
            double dot = 0.0;
            for (int i = j; i < rows; ++i) {
                dot += v[i] * A[i * cols + k];
            }
            double factor = 2.0 * dot / vnorm_sq;
            for (int i = j; i < rows; ++i) {
                A[i * cols + k] -= factor * v[i];
            }
        }
        double dotb = 0.0;
        for (int i = j; i < rows; ++i) {
            dotb += v[i] * B[i];
        }
        double factorb = 2.0 * dotb / vnorm_sq;
        for (int i = j; i < rows; ++i) {
            B[i] -= factorb * v[i];
        }
    }
    for (int i = cols - 1; i >= 0; --i) {
        double sum = B[i];
        for (int j = i + 1; j < cols; ++j) {
            sum -= A[i * cols + j] * x[j];
        }
        if (fabs(A[i * cols + i]) < 1e-12) {
            return false;
        }
        x[i] = sum / A[i * cols + i];
    }
    return true;
}

static double calibration_residual_arcsec(os_equatorial_coord_t samples_radec[],
                                          os_motor_position_t samples_motor[],
                                          int count) {
    double sum_sq = 0.0;
    double ra_steps_per_arcsec = (15.0 * OS_STEPS_PER_DEGREE) / 3600.0;
    double dec_steps_per_arcsec = OS_STEPS_PER_DEGREE / 3600.0;
    for (int i = 0; i < count; ++i) {
        double pred0 = (double)s_calibration.matrix_ra_to_ra * samples_radec[i].ra_hours +
                       (double)s_calibration.matrix_ra_to_dec * samples_radec[i].dec_degrees +
                       (double)s_calibration.offset_ra_arcsec;
        double pred1 = (double)s_calibration.matrix_dec_to_ra * samples_radec[i].ra_hours +
                       (double)s_calibration.matrix_dec_to_dec * samples_radec[i].dec_degrees +
                       (double)s_calibration.offset_dec_arcsec;
        double err0 = ((double)samples_motor[i].ra_steps - pred0) / ra_steps_per_arcsec;
        double err1 = ((double)samples_motor[i].dec_steps - pred1) / dec_steps_per_arcsec;
        sum_sq += err0 * err0 + err1 * err1;
    }
    return sqrt(sum_sq / (double)count);
}

static bool calibration_solve(void) {
    os_equatorial_coord_t radec[OS_CALIBRATION_MAX_STARS];
    os_motor_position_t motor[OS_CALIBRATION_MAX_STARS];
    for (int i = 0; i < s_align_count; ++i) {
        radec[i] = s_align_samples[i].star;
        motor[i] = s_align_samples[i].motor;
    }
    os_calibration_t cand;
    memset(&cand, 0, sizeof(cand));
    if (s_align_mode == OS_ALIGN_1STAR) {
        cand.matrix_ra_to_ra = (float)(15.0 * OS_STEPS_PER_DEGREE);
        cand.matrix_ra_to_dec = 0.0f;
        cand.matrix_dec_to_ra = 0.0f;
        cand.matrix_dec_to_dec = (float)OS_STEPS_PER_DEGREE;
        cand.offset_ra_arcsec = (float)((double)motor[0].ra_steps -
                                        (15.0 * OS_STEPS_PER_DEGREE) * radec[0].ra_hours);
        cand.offset_dec_arcsec = (float)((double)motor[0].dec_steps -
                                        OS_STEPS_PER_DEGREE * radec[0].dec_degrees);
    } else if (s_align_mode == OS_ALIGN_2STAR) {
        if (fabs((double)radec[1].ra_hours - (double)radec[0].ra_hours) < 1e-12 ||
            fabs((double)radec[1].dec_degrees - (double)radec[0].dec_degrees) < 1e-12) {
            return false;
        }
        double m00 = ((double)motor[1].ra_steps - (double)motor[0].ra_steps) /
                     ((double)radec[1].ra_hours - (double)radec[0].ra_hours);
        double m11 = ((double)motor[1].dec_steps - (double)motor[0].dec_steps) /
                     ((double)radec[1].dec_degrees - (double)radec[0].dec_degrees);
        cand.matrix_ra_to_ra = (float)m00;
        cand.matrix_ra_to_dec = 0.0f;
        cand.matrix_dec_to_ra = 0.0f;
        cand.matrix_dec_to_dec = (float)m11;
        cand.offset_ra_arcsec = (float)((double)motor[0].ra_steps - m00 * radec[0].ra_hours);
        cand.offset_dec_arcsec = (float)((double)motor[0].dec_steps - m11 * radec[0].dec_degrees);
    } else {
        if (s_align_count == 3 && (s_align_mode == OS_ALIGN_3STAR ||
                                   s_align_mode == OS_ALIGN_NSTAR)) {
            double det = ((double)radec[1].ra_hours - radec[0].ra_hours) *
                         ((double)radec[2].dec_degrees - radec[0].dec_degrees) -
                         ((double)radec[2].ra_hours - radec[0].ra_hours) *
                         ((double)radec[1].dec_degrees - radec[0].dec_degrees);
            if (fabs(det) < 1e-4) {
                s_calibration_degenerate = true;
                return false;
            }
        }
        double A[OS_CALIBRATION_MAX_STARS * 3];
        double b0[OS_CALIBRATION_MAX_STARS];
        double b1[OS_CALIBRATION_MAX_STARS];
        double x0[3] = {0.0, 0.0, 0.0};
        double x1[3] = {0.0, 0.0, 0.0};
        for (int i = 0; i < s_align_count; ++i) {
            A[i * 3 + 0] = (double)radec[i].ra_hours;
            A[i * 3 + 1] = (double)radec[i].dec_degrees;
            A[i * 3 + 2] = 1.0;
            b0[i] = (double)motor[i].ra_steps;
            b1[i] = (double)motor[i].dec_steps;
        }
        if (!qr_solve_least_squares(A, b0, s_align_count, 3, x0) ||
            !qr_solve_least_squares(A, b1, s_align_count, 3, x1)) {
            s_calibration_degenerate = true;
            return false;
        }
        cand.matrix_ra_to_ra = (float)x0[0];
        cand.matrix_ra_to_dec = (float)x0[1];
        cand.matrix_dec_to_ra = (float)x1[0];
        cand.matrix_dec_to_dec = (float)x1[1];
        cand.offset_ra_arcsec = (float)x0[2];
        cand.offset_dec_arcsec = (float)x1[2];
    }
    cand.valid = true;
    s_calibration = cand;
    s_residual_arcsec = calibration_residual_arcsec(radec, motor, s_align_count);
    s_calibration_residual_computed = true;
    s_calibration_degenerate = false;
    if (s_align_count >= 4 &&
        s_residual_arcsec > OS_CALIBRATION_RESIDUAL_THRESHOLD_ARCSEC) {
        s_calibration_degenerate = true;
        s_calibration.valid = false;
        return false;
    }
    return true;
}

static os_error_t nvm_store_calibration(void) {
    if (!s_nvm_ready) {
        return OS_ERR_NVM_FAULT;
    }
    return os_hal_nvm_write(OS_NVM_CALIBRATION_OFFSET,
                            (const uint8_t *)&s_calibration,
                            sizeof(s_calibration));
}

static os_error_t nvm_store_park_position(void) {
    if (!s_nvm_ready) {
        return OS_ERR_NVM_FAULT;
    }
    return os_hal_nvm_write(OS_NVM_CONFIG_OFFSET,
                            (const uint8_t *)&s_park_position,
                            sizeof(s_park_position));
}

static void set_printf_reply(char *buf, size_t size, size_t *len,
                             const char *fmt, ...) {
    if (len == NULL) {
        return;
    }
    if (buf == NULL || size == 0U) {
        *len = 0U;
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, size, fmt, ap);
    va_end(ap);
    if (n < 0) {
        buf[0] = '\0';
        *len = 0U;
    } else if ((size_t)n < size) {
        *len = (size_t)n;
    } else {
        *len = size - 1U;
    }
}

static void set_reply_text(char *buf, size_t size, size_t *len,
                           const char *text) {
    set_printf_reply(buf, size, len, "%s", text);
}

static void reply_action_result(char *buf, size_t size, size_t *len,
                                os_error_t err) {
    if (err == OS_ERR_NONE) {
        set_reply_text(buf, size, len, "1#");
    } else {
        set_printf_reply(buf, size, len, "%d#", (int)err);
    }
}

static const char *state_text(os_state_t state) {
    switch (state) {
    case OS_STATE_INITIALIZING:
        return "INIT";
    case OS_STATE_IDLE_TRACKING:
        return "IDLE";
    case OS_STATE_GOTO:
        return "GOTO";
    case OS_STATE_ALIGNMENT:
        return "ALIGN";
    case OS_STATE_MANUAL_MOTION:
        return "MANUAL";
    case OS_STATE_PARKED:
        return "PARKED";
    case OS_STATE_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

static os_error_t dispatch_lx200(const char *body,
                                 char *reply, size_t reply_size,
                                 size_t *reply_len) {
    os_error_t rc = OS_ERR_NONE;
    if (strcmp(body, "GR") == 0) {
        os_equatorial_coord_t coord = {0.0f, 0.0f};
        os_query_coordinates(&coord);
        set_printf_reply(reply, reply_size, reply_len, "%.4f#", (double)coord.ra_hours);
    } else if (strcmp(body, "GD") == 0) {
        os_equatorial_coord_t coord = {0.0f, 0.0f};
        os_query_coordinates(&coord);
        set_printf_reply(reply, reply_size, reply_len, "%.4f#", (double)coord.dec_degrees);
    } else if (strcmp(body, "GVP") == 0) {
        set_printf_reply(reply, reply_size, reply_len, "%d.%d.%d#",
                         OS_FIRMWARE_VERSION_MAJOR, OS_FIRMWARE_VERSION_MINOR,
                         OS_FIRMWARE_VERSION_PATCH);
    } else if (strcmp(body, "GS") == 0 || strcmp(body, "GOS") == 0) {
        set_printf_reply(reply, reply_size, reply_len, "%s#", state_text(s_state));
    } else if (strcmp(body, "Me") == 0) {
        rc = os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
        reply_action_result(reply, reply_size, reply_len, rc);
    } else if (strcmp(body, "Mw") == 0) {
        rc = os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
        reply_action_result(reply, reply_size, reply_len, rc);
    } else if (strcmp(body, "Mn") == 0) {
        rc = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
        reply_action_result(reply, reply_size, reply_len, rc);
    } else if (strcmp(body, "Ms") == 0) {
        rc = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
        reply_action_result(reply, reply_size, reply_len, rc);
    } else if (strcmp(body, "Q") == 0) {
        if (s_goto_in_progress) {
            rc = os_goto_abort();
        } else if (s_manual_motion_active) {
            rc = os_move_stop();
        } else {
            rc = OS_ERR_INVALID_STATE;
        }
        reply_action_result(reply, reply_size, reply_len, rc);
    } else if (strcmp(body, "hP") == 0) {
        rc = os_park();
        reply_action_result(reply, reply_size, reply_len, rc);
    } else if (strcmp(body, "hO") == 0) {
        rc = os_unpark();
        reply_action_result(reply, reply_size, reply_len, rc);
    } else {
        rc = OS_ERR_COMMAND_FORMAT;
        set_reply_text(reply, reply_size, reply_len, "0 Unknown#");
    }
    return rc;
}

os_error_t os_init(void) {
    s_state = OS_STATE_INITIALIZING;
    s_init_complete = false;
    s_config_loaded = false;
    s_nvm_ready = false;
    s_gps_locked = false;
    s_rtc_valid = false;
    s_utc_epoch_seconds = 0U;
    s_site_latitude_deg = 0.0;
    s_site_longitude_deg = 0.0;
    s_site_elevation_metres = 0.0;
    s_tracking_enabled = false;
    s_tracking_rate = OS_TRACK_RATE_SIDEREAL;
    s_custom_tracking_factor = 1.0f;
    s_goto_in_progress = false;
    s_goto_abort_requested = false;
    s_manual_motion_active = false;
    s_parking_active = false;
    s_parked = false;
    s_park_position_set = false;
    s_park_position.ra_hours = 0.0f;
    s_park_position.dec_degrees = 90.0f;
    s_guide_pulse.active = false;
    s_guide_pulse.duration_ms = 0U;
    s_guide_pulse.rate_fraction = s_guide_rate_fraction;
    s_guide_pulse.direction_east = false;
    s_guide_pulse.direction_north = false;
    s_guide_pulse.dec_priority = false;
    s_guide_pulse_remaining_ms = 0U;
    s_guide_rate_fraction = 0.5f;
    s_custom_manual_speed_arcsec_per_sec = 15.0f;
    s_align_mode = OS_ALIGN_1STAR;
    s_align_count = 0U;
    memset(s_align_samples, 0, sizeof(s_align_samples));
    s_calibration_residual_computed = false;
    s_residual_arcsec = 0.0;
    s_calibration_degenerate = false;
    memset(&s_calibration, 0, sizeof(s_calibration));
    s_calibration.valid = false;
    s_pec_enabled = false;
    memset(&s_pec_table, 0, sizeof(s_pec_table));
    s_pec_table.valid = false;
    s_worm_phase_deg = 0.0f;
    memset(s_channel_rx, 0, sizeof(s_channel_rx));
    memset(s_channel_rx_len, 0, sizeof(s_channel_rx_len));

    if (os_hal_nvm_init() == OS_ERR_NONE) {
        s_nvm_ready = true;
        os_calibration_t cal_restore;
        memset(&cal_restore, 0, sizeof(cal_restore));
        if (os_hal_nvm_read(OS_NVM_CALIBRATION_OFFSET,
                            (uint8_t *)&cal_restore,
                            sizeof(cal_restore)) == OS_ERR_NONE) {
            if (cal_restore.valid) {
                s_calibration = cal_restore;
            }
        }
        os_equatorial_coord_t park_restore = {0.0f, 90.0f};
        if (os_hal_nvm_read(OS_NVM_CONFIG_OFFSET,
                            (uint8_t *)&park_restore,
                            sizeof(park_restore)) == OS_ERR_NONE) {
            if (radec_valid(park_restore.ra_hours, park_restore.dec_degrees)) {
                s_park_position = park_restore;
                s_park_position_set = true;
            }
        }
    }

    for (int ch = 0; ch < 4; ++ch) {
        (void)os_hal_comm_init((uint8_t)ch);
    }

    if (os_hal_motor_init(0U) != OS_ERR_NONE ||
        os_hal_motor_init(1U) != OS_ERR_NONE) {
        enter_fault();
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    (void)os_hal_limit_init();
    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    if (os_hal_timer_motor_init() != OS_ERR_NONE) {
        enter_fault();
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    os_site_info_t site;
    memset(&site, 0, sizeof(site));
    os_error_t gps_err = os_hal_gps_poll(&site);
    if (gps_err == OS_ERR_NONE && site.valid &&
        site.latitude_degrees >= -90.0f && site.latitude_degrees <= 90.0f &&
        site.longitude_degrees >= -180.0f && site.longitude_degrees <= 180.0f) {
        s_gps_locked = true;
        s_site_latitude_deg = (double)site.latitude_degrees;
        s_site_longitude_deg = (double)site.longitude_degrees;
        s_site_elevation_metres = (double)site.elevation_metres;
        s_utc_epoch_seconds = site.utc_epoch_seconds;
        (void)os_hal_rtc_set(site.utc_epoch_seconds);
    } else {
        s_gps_locked = false;
        uint32_t utc = 0U;
        if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
            s_rtc_valid = true;
            s_utc_epoch_seconds = utc;
        } else {
            s_rtc_valid = false;
            s_utc_epoch_seconds = 0U;
        }
    }

    s_init_complete = true;
    s_config_loaded = true;
    s_tracking_enabled = true;
    s_tracking_rate = OS_TRACK_RATE_SIDEREAL;
    s_custom_tracking_factor = 1.0f;
    s_state = OS_STATE_IDLE_TRACKING;
    apply_tracking_frequency();
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    for (int ch = 0; ch < 4; ++ch) {
        int16_t available = os_hal_comm_available((uint8_t)ch);
        while (available > 0) {
            char byte = os_hal_comm_read((uint8_t)ch);
            if (byte == '\n' || byte == '\r') {
                if (s_channel_rx_len[ch] > 0U) {
                    char reply[OS_MAX_REPLY_LENGTH];
                    size_t reply_len = 0U;
                    memset(reply, 0, sizeof(reply));
                    (void)os_command_parse(s_channel_rx[ch], s_channel_rx_len[ch],
                                           (uint8_t)ch, reply, sizeof(reply),
                                           &reply_len);
                    if (reply_len > 0U) {
                        (void)os_hal_comm_write((uint8_t)ch, reply, reply_len);
                    }
                }
                s_channel_rx_len[ch] = 0U;
                s_channel_rx[ch][0] = '\0';
                break;
            }
            if (s_channel_rx_len[ch] < OS_MAX_COMMAND_LENGTH) {
                s_channel_rx[ch][s_channel_rx_len[ch]++] = byte;
                s_channel_rx[ch][s_channel_rx_len[ch]] = '\0';
            } else {
                s_channel_rx_len[ch] = 0U;
                s_channel_rx[ch][0] = '\0';
                const char err[] = "0 Overflow#";
                (void)os_hal_comm_write((uint8_t)ch, err, sizeof(err) - 1U);
                break;
            }
            available = os_hal_comm_available((uint8_t)ch);
        }
    }

    os_site_info_t site;
    memset(&site, 0, sizeof(site));
    os_error_t gps_err = os_hal_gps_poll(&site);
    if (gps_err == OS_ERR_NONE && site.valid &&
        site.latitude_degrees >= -90.0f && site.latitude_degrees <= 90.0f &&
        site.longitude_degrees >= -180.0f && site.longitude_degrees <= 180.0f) {
        s_gps_locked = true;
        s_site_latitude_deg = (double)site.latitude_degrees;
        s_site_longitude_deg = (double)site.longitude_degrees;
        s_site_elevation_metres = (double)site.elevation_metres;
        s_utc_epoch_seconds = site.utc_epoch_seconds;
        (void)os_hal_rtc_set(site.utc_epoch_seconds);
    } else {
        s_gps_locked = false;
    }

    bool limit0 = os_hal_limit_is_triggered(0U);
    bool limit1 = os_hal_limit_is_triggered(1U);
    if (limit0 || limit1) {
        if (s_goto_in_progress || s_manual_motion_active ||
            s_parking_active || s_state == OS_STATE_IDLE_TRACKING) {
            enter_fault();
            s_goto_in_progress = false;
            s_manual_motion_active = false;
            s_parking_active = false;
            return;
        }
    }

    if (s_goto_in_progress || s_parking_active) {
        int32_t pos0 = os_hal_motor_get_position(0U);
        int32_t pos1 = os_hal_motor_get_position(1U);
        if (motion_target_reached(pos0, pos1,
                                  s_motor_target_steps[0],
                                  s_motor_target_steps[1])) {
            if (s_parking_active) {
                finish_parking();
            } else {
                finish_goto(true);
            }
            return;
        }
    }

    if (s_guide_pulse.active) {
        if (s_guide_pulse_remaining_ms > OS_LOOP_TICK_MS) {
            s_guide_pulse_remaining_ms -= OS_LOOP_TICK_MS;
        } else {
            clear_guide_pulse();
        }
    }

    if (s_state == OS_STATE_IDLE_TRACKING && s_tracking_enabled &&
        !s_goto_in_progress && !s_manual_motion_active &&
        !s_parking_active && !s_parked) {
        s_worm_phase_deg += OS_WORM_PHASE_ADVANCE_LOOP_DEG;
        if (s_worm_phase_deg >= 360.0f) {
            s_worm_phase_deg -= 360.0f;
        }
        apply_tracking_frequency();
    }
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *reply_length = 0U;
    set_reply_text(reply_buffer, reply_buffer_size, reply_length, "");
    if (length > OS_MAX_COMMAND_LENGTH) {
        set_reply_text(reply_buffer, reply_buffer_size, reply_length,
                       "0 OS_ERR_INVALID_ARGUMENT#");
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (source_channel > OS_CHANNEL_ETHERNET) {
        set_reply_text(reply_buffer, reply_buffer_size, reply_length,
                       "0 OS_ERR_INVALID_ARGUMENT#");
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length < 2U) {
        set_reply_text(reply_buffer, reply_buffer_size, reply_length,
                       "0 OS_ERR_COMMAND_FORMAT#");
        return OS_ERR_COMMAND_FORMAT;
    }
    char cmd[OS_MAX_COMMAND_LENGTH + 1];
    memset(cmd, 0, sizeof(cmd));
    memcpy(cmd, command, length);
    cmd[length] = '\0';
    size_t len = length;
    while (len > 0U && (cmd[len - 1U] == '\n' || cmd[len - 1U] == '\r')) {
        cmd[--len] = '\0';
    }
    if (len < 2U || cmd[0] != OS_LX200_CMD_PREFIX ||
        cmd[len - 1U] != OS_LX200_CMD_SUFFIX) {
        set_reply_text(reply_buffer, reply_buffer_size, reply_length,
                       "0 OS_ERR_COMMAND_FORMAT#");
        return OS_ERR_COMMAND_FORMAT;
    }
    size_t body_len = len - 2U;
    char body[OS_MAX_COMMAND_LENGTH];
    memset(body, 0, sizeof(body));
    if (body_len >= sizeof(body)) {
        body_len = sizeof(body) - 1U;
    }
    memcpy(body, cmd + 1, body_len);
    body[body_len] = '\0';
    if (body_len == 0U) {
        set_reply_text(reply_buffer, reply_buffer_size, reply_length,
                       "0 OS_ERR_COMMAND_FORMAT#");
        return OS_ERR_COMMAND_FORMAT;
    }
    return dispatch_lx200(body, reply_buffer, reply_buffer_size, reply_length);
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (!radec_valid(target.ra_hours, target.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0U) || os_hal_limit_is_triggered(1U)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    int32_t current0 = os_hal_motor_get_position(0U);
    int32_t current1 = os_hal_motor_get_position(1U);
    int32_t target0 = coordinate_to_ra_steps(target.ra_hours, target.dec_degrees);
    int32_t target1 = coordinate_to_dec_steps(target.ra_hours, target.dec_degrees);
    s_motor_target_steps[0] = target0;
    s_motor_target_steps[1] = target1;
    if (motion_target_reached(current0, current1, target0, target1)) {
        finish_goto(true);
        return OS_ERR_NONE;
    }
    s_goto_equatorial_target = target;
    s_goto_in_progress = true;
    s_goto_abort_requested = false;
    s_manual_motion_active = false;
    s_parking_active = false;
    s_parked = false;
    s_state = OS_STATE_GOTO;
    if (abs_i32(current0 - target0) > OS_MOTION_DEADBAND_STEPS) {
        start_axis_motion(0U, target0 > current0, goto_frequency_hz());
    } else {
        (void)os_hal_motor_set_frequency(0U, 0U);
    }
    if (abs_i32(current1 - target1) > OS_MOTION_DEADBAND_STEPS) {
        start_axis_motion(1U, target1 > current1, goto_frequency_hz());
    } else {
        (void)os_hal_motor_set_frequency(1U, 0U);
    }
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (!horizontal_valid(target.azimuth_degrees, target.altitude_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0U) || os_hal_limit_is_triggered(1U)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    int32_t current0 = os_hal_motor_get_position(0U);
    int32_t current1 = os_hal_motor_get_position(1U);
    int32_t target0 = (int32_t)((double)target.azimuth_degrees * OS_STEPS_PER_DEGREE);
    int32_t target1 = (int32_t)((double)target.altitude_degrees * OS_STEPS_PER_DEGREE);
    s_motor_target_steps[0] = target0;
    s_motor_target_steps[1] = target1;
    if (motion_target_reached(current0, current1, target0, target1)) {
        finish_goto(true);
        return OS_ERR_NONE;
    }
    s_goto_horizontal_target = target;
    s_goto_in_progress = true;
    s_goto_abort_requested = false;
    s_manual_motion_active = false;
    s_parking_active = false;
    s_parked = false;
    s_state = OS_STATE_GOTO;
    if (abs_i32(current0 - target0) > OS_MOTION_DEADBAND_STEPS) {
        start_axis_motion(0U, target0 > current0, goto_frequency_hz());
    } else {
        (void)os_hal_motor_set_frequency(0U, 0U);
    }
    if (abs_i32(current1 - target1) > OS_MOTION_DEADBAND_STEPS) {
        start_axis_motion(1U, target1 > current1, goto_frequency_hz());
    } else {
        (void)os_hal_motor_set_frequency(1U, 0U);
    }
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (!s_goto_in_progress) {
        return OS_ERR_INVALID_STATE;
    }
    stop_all_motor_pulses();
    s_goto_in_progress = false;
    s_goto_abort_requested = false;
    s_state = OS_STATE_IDLE_TRACKING;
    s_tracking_enabled = true;
    apply_tracking_frequency();
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (rate < OS_TRACK_RATE_SIDEREAL || rate > OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM &&
        (!isfinite((double)custom_factor) || custom_factor <= 0.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_tracking_rate = rate;
    if (rate == OS_TRACK_RATE_CUSTOM) {
        s_custom_tracking_factor = custom_factor;
    } else {
        s_custom_tracking_factor = 1.0f;
    }
    if (s_state == OS_STATE_IDLE_TRACKING && s_tracking_enabled) {
        apply_tracking_frequency();
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = s_tracking_rate;
    *custom_factor = s_custom_tracking_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    s_tracking_enabled = true;
    if (s_state == OS_STATE_IDLE_TRACKING &&
        !s_goto_in_progress && !s_manual_motion_active &&
        !s_parking_active) {
        apply_tracking_frequency();
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    s_tracking_enabled = false;
    if (!s_goto_in_progress && !s_manual_motion_active && !s_parking_active) {
        stop_all_motor_pulses();
    }
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (direction_axis(direction) < 0 || duration_ms == 0U) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    bool new_is_dec = (direction == OS_DIRECTION_NORTH ||
                       direction == OS_DIRECTION_SOUTH);
    if (s_guide_pulse.active) {
        if (s_guide_pulse.dec_priority && !new_is_dec) {
            return OS_ERR_NONE;
        }
    }
    s_guide_pulse.active = true;
    s_guide_pulse.duration_ms = duration_ms;
    s_guide_pulse.rate_fraction = s_guide_rate_fraction;
    s_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    s_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    s_guide_pulse.dec_priority = new_is_dec;
    s_guide_pulse_remaining_ms = duration_ms;
    apply_guide_pulse_output();
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (!isfinite((double)rate_fraction) ||
        rate_fraction < OS_GUIDE_RATE_MIN ||
        rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_guide_rate_fraction = rate_fraction;
    s_guide_pulse.rate_fraction = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *pulse = s_guide_pulse;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    s_align_mode = mode;
    s_align_count = 0U;
    memset(s_align_samples, 0, sizeof(s_align_samples));
    s_calibration.valid = false;
    s_calibration_residual_computed = false;
    s_residual_arcsec = 0.0;
    s_calibration_degenerate = false;
    s_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (!radec_valid(star_coord.ra_hours, star_coord.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_align_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }
    s_align_samples[s_align_count].star = star_coord;
    s_align_samples[s_align_count].motor = motor_pos;
    ++s_align_count;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_align_count < min_stars_for_mode(s_align_mode)) {
        return OS_ERR_INVALID_STATE;
    }
    if (!calibration_solve()) {
        s_calibration.valid = false;
        s_calibration_residual_computed = false;
        return OS_ERR_CALIBRATION_FAILED;
    }
    os_error_t nvm_err = nvm_store_calibration();
    if (nvm_err != OS_ERR_NONE) {
        s_calibration.valid = false;
        s_calibration_residual_computed = false;
        return OS_ERR_NVM_FAULT;
    }
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_calibration_residual_computed) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = (float)s_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    if (s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    s_align_count = 0U;
    memset(s_align_samples, 0, sizeof(s_align_samples));
    s_calibration.valid = false;
    s_calibration_residual_computed = false;
    s_residual_arcsec = 0.0;
    s_calibration_degenerate = false;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0U) || os_hal_limit_is_triggered(1U)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    int32_t current0 = os_hal_motor_get_position(0U);
    int32_t current1 = os_hal_motor_get_position(1U);
    int32_t target0 = coordinate_to_ra_steps(s_park_position.ra_hours,
                                              s_park_position.dec_degrees);
    int32_t target1 = coordinate_to_dec_steps(s_park_position.ra_hours,
                                              s_park_position.dec_degrees);
    s_motor_target_steps[0] = target0;
    s_motor_target_steps[1] = target1;
    if (motion_target_reached(current0, current1, target0, target1)) {
        finish_parking();
        return OS_ERR_NONE;
    }
    s_parking_active = true;
    s_parked = false;
    s_goto_in_progress = false;
    s_manual_motion_active = false;
    s_state = OS_STATE_PARKED;
    if (abs_i32(current0 - target0) > OS_MOTION_DEADBAND_STEPS) {
        start_axis_motion(0U, target0 > current0, goto_frequency_hz());
    } else {
        (void)os_hal_motor_set_frequency(0U, 0U);
    }
    if (abs_i32(current1 - target1) > OS_MOTION_DEADBAND_STEPS) {
        start_axis_motion(1U, target1 > current1, goto_frequency_hz());
    } else {
        (void)os_hal_motor_set_frequency(1U, 0U);
    }
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (s_state != OS_STATE_PARKED || s_parking_active) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_motor_enable(0U, true) != OS_ERR_NONE ||
        os_hal_motor_enable(1U, true) != OS_ERR_NONE) {
        enter_fault();
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    for (int ch = 0; ch < 4; ++ch) {
        (void)os_hal_comm_init((uint8_t)ch);
    }
    if (s_gps_locked && s_utc_epoch_seconds != 0U) {
        (void)os_hal_rtc_set(s_utc_epoch_seconds);
    }
    s_parked = false;
    s_tracking_enabled = true;
    s_state = OS_STATE_IDLE_TRACKING;
    apply_tracking_frequency();
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!radec_valid(park_pos.ra_hours, park_pos.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_park_position = park_pos;
    s_park_position_set = true;
    os_error_t nvm_err = nvm_store_park_position();
    if (nvm_err != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int axis = direction_axis(direction);
    if (axis < 0 || os_hal_limit_is_triggered((uint8_t)axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (s_state != OS_STATE_IDLE_TRACKING &&
        s_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }
    stop_all_motor_pulses();
    s_manual_motion_active = true;
    s_manual_direction = direction;
    s_manual_speed = speed;
    s_goto_in_progress = false;
    s_parking_active = false;
    s_parked = false;
    s_state = OS_STATE_MANUAL_MOTION;
    start_axis_motion((uint8_t)axis, direction_forward(direction),
                     manual_frequency_hz());
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (!s_manual_motion_active) {
        return OS_ERR_INVALID_STATE;
    }
    stop_all_motor_pulses();
    s_manual_motion_active = false;
    s_state = OS_STATE_IDLE_TRACKING;
    s_tracking_enabled = true;
    apply_tracking_frequency();
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (!isfinite((double)arcsec_per_sec) || arcsec_per_sec <= 0.0f ||
        arcsec_per_sec > OS_MAX_CUSTOM_MANUAL_ASEC) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_custom_manual_speed_arcsec_per_sec = arcsec_per_sec;
    if (s_manual_motion_active && s_manual_speed == OS_SPEED_CUSTOM) {
        int axis = direction_axis(s_manual_direction);
        if (axis >= 0) {
            (void)os_hal_motor_set_frequency((uint8_t)axis,
                                             manual_frequency_hz());
        }
    }
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = s_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int32_t ra_steps = os_hal_motor_get_position(0U);
    int32_t dec_steps = os_hal_motor_get_position(1U);
    steps_to_equatorial(ra_steps, dec_steps, coord);
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    site->latitude_degrees = (float)s_site_latitude_deg;
    site->longitude_degrees = (float)s_site_longitude_deg;
    site->elevation_metres = (float)s_site_elevation_metres;
    site->utc_epoch_seconds = s_utc_epoch_seconds;
    site->valid = (s_gps_locked || s_rtc_valid);
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = os_hal_motor_get_position(0U);
    pos->dec_steps = os_hal_motor_get_position(1U);
    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor,
                                     uint8_t *patch) {
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
    *moving = s_goto_in_progress || s_manual_motion_active ||
               s_parking_active;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *locked = s_gps_locked;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    if (enable && !s_pec_table.valid) {
        return OS_ERR_INVALID_STATE;
    }
    s_pec_enabled = enable;
    if (!enable && s_state == OS_STATE_IDLE_TRACKING && s_tracking_enabled) {
        apply_tracking_frequency();
    }
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL || !table->valid) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_pec_table = *table;
    s_pec_table.valid = true;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *table = s_pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (!isfinite((double)worm_phase_deg) ||
        worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    uint16_t index = pec_index_for_phase(worm_phase_deg);
    s_pec_table.corrections[index] = error_arcsec;
    s_pec_table.valid = true;
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *calib = s_calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    memset(&s_calibration, 0, sizeof(s_calibration));
    s_calibration.valid = false;
    s_calibration_residual_computed = false;
    s_residual_arcsec = 0.0;
    s_calibration_degenerate = false;
    os_error_t nvm_err = nvm_store_calibration();
    if (nvm_err != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    return OS_ERR_NONE;
}
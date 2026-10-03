/* OnStep domain logic implementing the frozen public API.
   The implementation is C11 and uses only the declared os_hal_* adapter surface. */

#include "6_generated_code.h"
#include "onstep_hal_private.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define AXIS_RA                 0u
#define AXIS_DEC                1u
#define AXIS_COUNT              2u

#define OS_STEPS_PER_DEGREE     3600.0
#define OS_STEPS_PER_ARCSEC     (OS_STEPS_PER_DEGREE / 3600.0)
#define OS_LOOP_PERIOD_MS       1u
#define OS_GOTO_TICK_STEPS      10
#define OS_MANUAL_TICK_SLOW     1
#define OS_MANUAL_TICK_MEDIUM   4
#define OS_MANUAL_TICK_FAST     10
#define OS_TRACKING_PULSE_HZ    15u
#define OS_GOTO_PULSE_HZ        10800u

#define OS_CALIBRATION_RESIDUAL_LIMIT_ARCSEC 300.0f

#define NVM_CAL_MAGIC           0x4F43414Cu
#define NVM_PEC_MAGIC           0x50454331u
#define NVM_CAL_OFFSET          0u
#define NVM_PEC_OFFSET          (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)

static os_state_t s_state = OS_STATE_INITIALIZING;
static bool       s_initialized;

static os_site_info_t s_site;
static bool           s_gps_locked;

static bool             s_tracking_enabled = true;
static os_track_rate_t  s_track_rate = OS_TRACK_RATE_SIDEREAL;
static float            s_custom_track_factor = 1.0f;

static os_equatorial_coord_t s_pending_target;
static bool                  s_pending_target_valid;

static os_equatorial_coord_t s_park_position;
static bool                  s_park_position_set;

static os_motor_position_t s_goto_target;
static bool                s_goto_active;
static bool                s_park_pending;

static bool            s_move_active;
static os_direction_t  s_move_direction;
static os_speed_level_t s_move_speed;
static float           s_move_custom_speed_arcsec_per_sec = 50.0f;

static float             s_guide_rate_fraction = 0.5f;
static os_guide_pulse_t  s_guide_pulse;

static bool             s_align_active;
static os_align_mode_t  s_align_mode = OS_ALIGN_3STAR;
static uint8_t          s_align_star_count;

static os_equatorial_coord_t s_align_stars[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t   s_align_motor_pos[OS_CALIBRATION_MAX_STARS];
static bool                  s_align_residual_valid;
static float                 s_align_residual_arcsec;

static os_calibration_t s_calibration;

static bool            s_pec_enabled;
static os_pec_table_t  s_pec_table;

static char   s_cmd_buf[4][OS_MAX_COMMAND_LENGTH];
static size_t s_cmd_len[4];

static float s_track_accum_ra = 0.0f;
static float s_track_accum_dec = 0.0f;

static bool valid_ra(float ra)
{
    return (ra >= OS_RA_MIN_HOURS && ra <= OS_RA_MAX_HOURS);
}

static bool valid_dec(float dec)
{
    return (dec >= OS_DEC_MIN_DEG && dec <= OS_DEC_MAX_DEG);
}

static bool valid_az(float az)
{
    return (az >= 0.0f && az <= 360.0f);
}

static bool valid_alt(float alt)
{
    return (alt >= -90.0f && alt <= 90.0f);
}

static bool valid_direction(os_direction_t direction)
{
    return (direction >= OS_DIRECTION_NORTH && direction <= OS_DIRECTION_WEST);
}

static bool valid_speed(os_speed_level_t speed)
{
    return (speed >= OS_SPEED_SLOW && speed <= OS_SPEED_CUSTOM);
}

static bool valid_track_rate(os_track_rate_t rate)
{
    return (rate >= OS_TRACK_RATE_SIDEREAL && rate <= OS_TRACK_RATE_CUSTOM);
}

static bool valid_align_mode(os_align_mode_t mode)
{
    return (mode == OS_ALIGN_1STAR || mode == OS_ALIGN_2STAR ||
            mode == OS_ALIGN_3STAR || mode == OS_ALIGN_NSTAR);
}

static void default_calibration(void)
{
    s_calibration.matrix_ra_to_ra = (float)OS_STEPS_PER_ARCSEC;
    s_calibration.matrix_ra_to_dec = 0.0f;
    s_calibration.matrix_dec_to_ra = 0.0f;
    s_calibration.matrix_dec_to_dec = (float)OS_STEPS_PER_ARCSEC;
    s_calibration.offset_ra_arcsec = 0.0f;
    s_calibration.offset_dec_arcsec = 0.0f;
    s_calibration.valid = false;
}

static void reset_runtime_state(void)
{
    s_initialized = false;
    s_state = OS_STATE_INITIALIZING;

    memset(&s_site, 0, sizeof(s_site));
    s_site.valid = false;
    s_gps_locked = false;

    s_tracking_enabled = true;
    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_custom_track_factor = 1.0f;

    s_pending_target.ra_hours = 0.0f;
    s_pending_target.dec_degrees = 0.0f;
    s_pending_target_valid = false;

    s_park_position.ra_hours = 0.0f;
    s_park_position.dec_degrees = 90.0f;
    s_park_position_set = false;

    s_goto_target.ra_steps = 0;
    s_goto_target.dec_steps = 0;
    s_goto_active = false;
    s_park_pending = false;

    s_move_active = false;
    s_move_direction = OS_DIRECTION_EAST;
    s_move_speed = OS_SPEED_SLOW;
    s_move_custom_speed_arcsec_per_sec = 50.0f;

    s_guide_rate_fraction = 0.5f;
    memset(&s_guide_pulse, 0, sizeof(s_guide_pulse));

    s_align_active = false;
    s_align_mode = OS_ALIGN_3STAR;
    s_align_star_count = 0u;
    s_align_residual_valid = false;
    s_align_residual_arcsec = 0.0f;
    memset(s_align_stars, 0, sizeof(s_align_stars));
    memset(s_align_motor_pos, 0, sizeof(s_align_motor_pos));

    default_calibration();

    s_pec_enabled = false;
    memset(&s_pec_table, 0, sizeof(s_pec_table));
    s_pec_table.valid = false;

    s_track_accum_ra = 0.0f;
    s_track_accum_dec = 0.0f;

    for (size_t i = 0; i < 4u; i++) {
        s_cmd_len[i] = 0u;
        memset(s_cmd_buf[i], 0, sizeof(s_cmd_buf[i]));
    }
}

static void eq_to_motor_steps(float ra_hours, float dec_deg, os_motor_position_t *out_steps)
{
    double x_arcsec = (double)ra_hours * 15.0 * 3600.0;
    double y_arcsec = (double)dec_deg * 3600.0;

    double ra_steps;
    double dec_steps;

    if (s_calibration.valid) {
        ra_steps = (double)s_calibration.matrix_ra_to_ra * x_arcsec +
                   (double)s_calibration.matrix_ra_to_dec * y_arcsec +
                   (double)s_calibration.offset_ra_arcsec;
        dec_steps = (double)s_calibration.matrix_dec_to_ra * x_arcsec +
                    (double)s_calibration.matrix_dec_to_dec * y_arcsec +
                    (double)s_calibration.offset_dec_arcsec;
    } else {
        ra_steps = OS_STEPS_PER_ARCSEC * x_arcsec;
        dec_steps = OS_STEPS_PER_ARCSEC * y_arcsec;
    }

    out_steps->ra_steps = (int32_t)(ra_steps >= 0.0 ? (ra_steps + 0.5) : (ra_steps - 0.5));
    out_steps->dec_steps = (int32_t)(dec_steps >= 0.0 ? (dec_steps + 0.5) : (dec_steps - 0.5));
}

static void motor_steps_to_eq(int32_t ra_steps, int32_t dec_steps,
                              float *ra_hours, float *dec_deg)
{
    double x_arcsec = 0.0;
    double y_arcsec = 0.0;
    bool converted = false;

    if (s_calibration.valid) {
        double rx = (double)ra_steps - (double)s_calibration.offset_ra_arcsec;
        double ry = (double)dec_steps - (double)s_calibration.offset_dec_arcsec;
        double a = s_calibration.matrix_ra_to_ra;
        double b = s_calibration.matrix_ra_to_dec;
        double c = s_calibration.matrix_dec_to_ra;
        double d = s_calibration.matrix_dec_to_dec;
        double det = a * d - b * c;

        if (fabs(det) > 1e-12) {
            x_arcsec = (d * rx - b * ry) / det;
            y_arcsec = (-c * rx + a * ry) / det;
            converted = true;
        }
    }

    if (!converted) {
        x_arcsec = (double)ra_steps / OS_STEPS_PER_ARCSEC;
        y_arcsec = (double)dec_steps / OS_STEPS_PER_ARCSEC;
    }

    double ra = x_arcsec / (15.0 * 3600.0);
    while (ra < 0.0) {
        ra += 24.0;
    }
    while (ra >= 24.0) {
        ra -= 24.0;
    }

    *ra_hours = (float)ra;
    *dec_deg = (float)(y_arcsec / 3600.0);
}

static uint32_t tracking_frequency_hz(void)
{
    float factor = 1.0f;

    switch (s_track_rate) {
    case OS_TRACK_RATE_SIDEREAL:
        factor = 1.0f;
        break;
    case OS_TRACK_RATE_LUNAR:
        factor = OS_LUNAR_RATE_FACTOR;
        break;
    case OS_TRACK_RATE_SOLAR:
        factor = OS_SOLAR_RATE_FACTOR;
        break;
    case OS_TRACK_RATE_CUSTOM:
        factor = s_custom_track_factor;
        break;
    default:
        factor = 1.0f;
        break;
    }

    float frequency = OS_SIDEREAL_RATE_ARCSEC_PER_SEC * factor * (float)OS_STEPS_PER_ARCSEC;
    if (frequency < 0.0f) {
        frequency = 0.0f;
    }

    return (uint32_t)(frequency + 0.5f);
}

static int32_t manual_steps_per_loop(void)
{
    switch (s_move_speed) {
    case OS_SPEED_SLOW:
        return OS_MANUAL_TICK_SLOW;
    case OS_SPEED_MEDIUM:
        return OS_MANUAL_TICK_MEDIUM;
    case OS_SPEED_FAST:
        return OS_MANUAL_TICK_FAST;
    case OS_SPEED_CUSTOM: {
        int32_t steps = (int32_t)(s_move_custom_speed_arcsec_per_sec *
                                  (float)OS_STEPS_PER_ARCSEC / 1000.0f);
        return steps > 0 ? steps : 1;
    }
    default:
        return OS_MANUAL_TICK_SLOW;
    }
}

static void move_axis_toward(uint8_t axis, int32_t delta)
{
    if (delta == 0) {
        return;
    }

    bool forward = delta > 0;
    int32_t step = delta;

    if (step > OS_GOTO_TICK_STEPS) {
        step = OS_GOTO_TICK_STEPS;
    } else if (step < -OS_GOTO_TICK_STEPS) {
        step = -OS_GOTO_TICK_STEPS;
    }

    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_frequency(axis, OS_GOTO_PULSE_HZ);
    (void)os_hal_motor_advance(axis, step);
}

static void finish_goto(void)
{
    s_goto_active = false;
    (void)os_hal_motor_set_frequency(AXIS_RA, 0u);
    (void)os_hal_motor_set_frequency(AXIS_DEC, 0u);

    if (s_park_pending) {
        s_park_pending = false;
        s_state = OS_STATE_PARKED;
        (void)os_hal_motor_enable(AXIS_RA, false);
        (void)os_hal_motor_enable(AXIS_DEC, false);
        (void)os_hal_buzzer_beep(100u, 1u);
    } else {
        s_state = OS_STATE_IDLE_TRACKING;
        (void)os_hal_buzzer_beep(80u, 1u);
    }
}

static void service_goto(void)
{
    os_motor_position_t current;
    current.ra_steps = os_hal_motor_get_position(AXIS_RA);
    current.dec_steps = os_hal_motor_get_position(AXIS_DEC);

    int32_t delta_ra = s_goto_target.ra_steps - current.ra_steps;
    int32_t delta_dec = s_goto_target.dec_steps - current.dec_steps;

    if (delta_ra == 0 && delta_dec == 0) {
        finish_goto();
        return;
    }

    move_axis_toward(AXIS_RA, delta_ra);
    move_axis_toward(AXIS_DEC, delta_dec);

    current.ra_steps = os_hal_motor_get_position(AXIS_RA);
    current.dec_steps = os_hal_motor_get_position(AXIS_DEC);

    if (current.ra_steps == s_goto_target.ra_steps &&
        current.dec_steps == s_goto_target.dec_steps) {
        finish_goto();
    }
}

static void service_manual(void)
{
    uint8_t axis = (s_move_direction == OS_DIRECTION_NORTH ||
                    s_move_direction == OS_DIRECTION_SOUTH) ? AXIS_DEC : AXIS_RA;
    bool forward = (s_move_direction == OS_DIRECTION_NORTH ||
                    s_move_direction == OS_DIRECTION_EAST);
    int32_t steps = manual_steps_per_loop();
    int32_t signed_steps = forward ? steps : -steps;
    uint32_t frequency_hz = (uint32_t)steps * 1000u;

    if (frequency_hz == 0u) {
        frequency_hz = 1000u;
    }

    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_frequency(axis, frequency_hz);
    (void)os_hal_motor_advance(axis, signed_steps);
}

static void apply_tracking(void)
{
    uint32_t base_hz = tracking_frequency_hz();

    if (s_tracking_enabled && s_state == OS_STATE_IDLE_TRACKING) {
        int32_t ra_freq = (int32_t)base_hz;
        int32_t dec_freq = 0;

        if (s_guide_pulse.active) {
            int32_t guide_delta = (int32_t)((float)base_hz * s_guide_pulse.rate_fraction);
            if (guide_delta == 0) {
                guide_delta = 1;
            }

            if (s_guide_pulse.dec_priority) {
                dec_freq = s_guide_pulse.direction_north ? guide_delta : -guide_delta;
            } else {
                ra_freq += s_guide_pulse.direction_east ? guide_delta : -guide_delta;
            }
        }

        (void)os_hal_motor_enable(AXIS_RA, true);
        (void)os_hal_motor_set_direction(AXIS_RA, ra_freq >= 0);
        (void)os_hal_motor_set_frequency(AXIS_RA, (uint32_t)(ra_freq >= 0 ? ra_freq : -ra_freq));

        if (dec_freq != 0) {
            (void)os_hal_motor_enable(AXIS_DEC, true);
            (void)os_hal_motor_set_direction(AXIS_DEC, dec_freq >= 0);
            (void)os_hal_motor_set_frequency(AXIS_DEC, (uint32_t)(dec_freq >= 0 ? dec_freq : -dec_freq));
        } else {
            (void)os_hal_motor_set_frequency(AXIS_DEC, 0u);
        }

        const float loop_seconds = (float)OS_LOOP_PERIOD_MS / 1000.0f;

        if (ra_freq != 0) {
            float ra_inc = (float)ra_freq * loop_seconds;
            s_track_accum_ra += ra_inc;
            int32_t ra_whole = (int32_t)s_track_accum_ra;
            if (ra_whole != 0) {
                s_track_accum_ra -= (float)ra_whole;
                (void)os_hal_motor_advance(AXIS_RA, ra_whole);
            }
        }

        if (dec_freq != 0) {
            float dec_inc = (float)dec_freq * loop_seconds;
            s_track_accum_dec += dec_inc;
            int32_t dec_whole = (int32_t)s_track_accum_dec;
            if (dec_whole != 0) {
                s_track_accum_dec -= (float)dec_whole;
                (void)os_hal_motor_advance(AXIS_DEC, dec_whole);
            }
        }
    } else {
        (void)os_hal_motor_set_frequency(AXIS_RA, 0u);
        (void)os_hal_motor_set_frequency(AXIS_DEC, 0u);
    }
}

static os_error_t write_reply(char *reply_buffer, size_t reply_buffer_size,
                              size_t *reply_length, const char *fmt, ...)
{
    if (reply_buffer == NULL || reply_length == NULL || fmt == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *reply_length = 0u;
    reply_buffer[0] = '\0';

    va_list ap;
    va_start(ap, fmt);
    int written = vsnprintf(reply_buffer, reply_buffer_size, fmt, ap);
    va_end(ap);

    if (written < 0) {
        return OS_ERR_COMMAND_FORMAT;
    }

    if ((size_t)written >= reply_buffer_size) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *reply_length = (size_t)written;
    return OS_ERR_NONE;
}

static bool parse_ra_string(const char *s, float *out)
{
    if (s == NULL || *s == '\0') {
        return false;
    }

    if (strchr(s, ':')) {
        int h = 0;
        int m = 0;
        float sec = 0.0f;
        if (sscanf(s, "%d:%d:%f", &h, &m, &sec) != 3) {
            return false;
        }

        float value = (float)h + (float)m / 60.0f + sec / 3600.0f;
        if (!valid_ra(value)) {
            return false;
        }

        *out = value;
        return true;
    }

    char *end = NULL;
    float value = strtof(s, &end);
    if (end == s) {
        return false;
    }

    if (!valid_ra(value)) {
        return false;
    }

    *out = value;
    return true;
}

static bool parse_dec_string(const char *s, float *out)
{
    if (s == NULL || *s == '\0') {
        return false;
    }

    char normalized[40];
    size_t j = 0u;
    for (size_t i = 0u; s[i] != '\0' && j < (sizeof(normalized) - 1u); i++) {
        if (s[i] == ':' || s[i] == '*') {
            normalized[j++] = ':';
        } else {
            normalized[j++] = s[i];
        }
    }
    normalized[j] = '\0';

    const char *p = normalized;
    char sign = '+';
    if (p[0] == '+' || p[0] == '-') {
        sign = p[0];
        p++;
    }

    int d = 0;
    int m = 0;
    float sec = 0.0f;
    if (sscanf(p, "%d:%d:%f", &d, &m, &sec) == 3) {
        float value = (float)d + (float)m / 60.0f + sec / 3600.0f;
        if (sign == '-') {
            value = -value;
        }

        if (!valid_dec(value)) {
            return false;
        }

        *out = value;
        return true;
    }

    char *end = NULL;
    float value = strtof(normalized, &end);
    if (end == normalized) {
        return false;
    }

    if (!valid_dec(value)) {
        return false;
    }

    *out = value;
    return true;
}

static bool solve_2x2(double a00, double a01, double a10, double a11,
                      double b0, double b1, double *x0, double *x1)
{
    double det = a00 * a11 - a01 * a10;
    if (fabs(det) < 1e-12) {
        return false;
    }

    *x0 = (b0 * a11 - a01 * b1) / det;
    *x1 = (a00 * b1 - b0 * a10) / det;
    return true;
}

static bool qr_solve_full(const double *A, const double *b, int n, double *x)
{
    double r[OS_CALIBRATION_MAX_STARS][3];
    double y[OS_CALIBRATION_MAX_STARS];
    double v[OS_CALIBRATION_MAX_STARS];

    for (int i = 0; i < n; i++) {
        r[i][0] = A[i * 3 + 0];
        r[i][1] = A[i * 3 + 1];
        r[i][2] = A[i * 3 + 2];
        y[i] = b[i];
    }

    for (int k = 0; k < 3; k++) {
        double norm = 0.0;
        for (int i = k; i < n; i++) {
            norm += r[i][k] * r[i][k];
        }
        norm = sqrt(norm);

        if (norm < 1e-12) {
            return false;
        }

        memset(v, 0, sizeof(v));
        for (int i = k; i < n; i++) {
            v[i] = r[i][k];
        }

        if (v[k] >= 0.0) {
            v[k] += norm;
        } else {
            v[k] -= norm;
        }

        double unorm = 0.0;
        for (int i = k; i < n; i++) {
            unorm += v[i] * v[i];
        }
        unorm = sqrt(unorm);

        if (unorm < 1e-12) {
            return false;
        }

        for (int i = k; i < n; i++) {
            v[i] /= unorm;
        }

        for (int col = k; col < 3; col++) {
            double dot = 0.0;
            for (int i = k; i < n; i++) {
                dot += v[i] * r[i][col];
            }

            for (int i = k; i < n; i++) {
                r[i][col] -= 2.0 * v[i] * dot;
            }
        }

        double dot_y = 0.0;
        for (int i = k; i < n; i++) {
            dot_y += v[i] * y[i];
        }

        for (int i = k; i < n; i++) {
            y[i] -= 2.0 * v[i] * dot_y;
        }
    }

    double m[3][3];
    for (int i = 0; i < 3; i++) {
        m[i][0] = r[i][0];
        m[i][1] = r[i][1];
        m[i][2] = r[i][2];
    }

    double b3[3] = { y[0], y[1], y[2] };

    if (fabs(m[2][2]) < 1e-12) {
        return false;
    }

    x[2] = b3[2] / m[2][2];

    if (fabs(m[1][1]) < 1e-12) {
        return false;
    }

    x[1] = (b3[1] - m[1][2] * x[2]) / m[1][1];

    if (fabs(m[0][0]) < 1e-12) {
        return false;
    }

    x[0] = (b3[0] - m[0][1] * x[1] - m[0][2] * x[2]) / m[0][0];

    return true;
}

static os_error_t nvm_save_calibration(void)
{
    uint8_t buffer[OS_NVM_CALIBRATION_SIZE_BYTES];
    memset(buffer, 0, sizeof(buffer));

    uint32_t magic = NVM_CAL_MAGIC;
    (void)memcpy(buffer, &magic, sizeof(magic));
    (void)memcpy(buffer + sizeof(magic), &s_calibration, sizeof(s_calibration));

    return os_hal_nvm_write((uint16_t)NVM_CAL_OFFSET, buffer, sizeof(buffer));
}

static void nvm_load_calibration(void)
{
    uint8_t buffer[OS_NVM_CALIBRATION_SIZE_BYTES];
    if (os_hal_nvm_read((uint16_t)NVM_CAL_OFFSET, buffer, sizeof(buffer)) != OS_ERR_NONE) {
        return;
    }

    uint32_t magic = 0u;
    (void)memcpy(&magic, buffer, sizeof(magic));
    if (magic != NVM_CAL_MAGIC) {
        return;
    }

    os_calibration_t loaded;
    (void)memcpy(&loaded, buffer + sizeof(magic), sizeof(loaded));

    if (!loaded.valid) {
        return;
    }

    if (!(loaded.matrix_ra_to_ra >= -1e9f && loaded.matrix_ra_to_ra <= 1e9f) ||
        !(loaded.matrix_ra_to_dec >= -1e9f && loaded.matrix_ra_to_dec <= 1e9f) ||
        !(loaded.matrix_dec_to_ra >= -1e9f && loaded.matrix_dec_to_ra <= 1e9f) ||
        !(loaded.matrix_dec_to_dec >= -1e9f && loaded.matrix_dec_to_dec <= 1e9f) ||
        !(loaded.offset_ra_arcsec >= -1e9f && loaded.offset_ra_arcsec <= 1e9f) ||
        !(loaded.offset_dec_arcsec >= -1e9f && loaded.offset_dec_arcsec <= 1e9f)) {
        return;
    }

    s_calibration = loaded;
}

static os_error_t nvm_clear_calibration(void)
{
    uint8_t buffer[OS_NVM_CALIBRATION_SIZE_BYTES];
    memset(buffer, 0, sizeof(buffer));
    return os_hal_nvm_write((uint16_t)NVM_CAL_OFFSET, buffer, sizeof(buffer));
}

static os_error_t nvm_save_pec(void)
{
    uint8_t buffer[sizeof(uint32_t) + sizeof(os_pec_table_t)];
    memset(buffer, 0, sizeof(buffer));

    uint32_t magic = NVM_PEC_MAGIC;
    (void)memcpy(buffer, &magic, sizeof(magic));
    (void)memcpy(buffer + sizeof(magic), &s_pec_table, sizeof(s_pec_table));

    return os_hal_nvm_write((uint16_t)NVM_PEC_OFFSET, buffer, sizeof(buffer));
}

static void nvm_load_pec(void)
{
    uint8_t buffer[sizeof(uint32_t) + sizeof(os_pec_table_t)];
    if (os_hal_nvm_read((uint16_t)NVM_PEC_OFFSET, buffer, sizeof(buffer)) != OS_ERR_NONE) {
        return;
    }

    uint32_t magic = 0u;
    (void)memcpy(&magic, buffer, sizeof(magic));
    if (magic != NVM_PEC_MAGIC) {
        return;
    }

    os_pec_table_t loaded;
    (void)memcpy(&loaded, buffer + sizeof(magic), sizeof(loaded));
    if (!loaded.valid) {
        return;
    }

    s_pec_table = loaded;
}

static os_error_t nvm_clear_pec(void)
{
    uint8_t buffer[sizeof(uint32_t) + sizeof(os_pec_table_t)];
    memset(buffer, 0, sizeof(buffer));
    return os_hal_nvm_write((uint16_t)NVM_PEC_OFFSET, buffer, sizeof(buffer));
}

static uint8_t align_min_stars(os_align_mode_t mode)
{
    switch (mode) {
    case OS_ALIGN_1STAR:
        return 1u;
    case OS_ALIGN_2STAR:
        return 2u;
    case OS_ALIGN_3STAR:
        return 3u;
    case OS_ALIGN_NSTAR:
        return 3u;
    default:
        return 0u;
    }
}

static bool align_check_collinearity_3star(void)
{
    double x[3];
    double y[3];

    for (int i = 0; i < 3; i++) {
        x[i] = (double)s_align_stars[i].ra_hours * 15.0 * 3600.0;
        y[i] = (double)s_align_stars[i].dec_degrees * 3600.0;
    }

    double det = x[0] * (y[1] - y[2]) -
                 y[0] * (x[1] - x[2]) +
                 (x[1] * y[2] - y[1] * x[2]);

    return fabs(det) < 1e-6;
}

static os_error_t compute_calibration(void)
{
    uint8_t min_stars = align_min_stars(s_align_mode);
    if (s_align_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    if (s_align_mode == OS_ALIGN_1STAR) {
        const os_equatorial_coord_t *star = &s_align_stars[0];
        const os_motor_position_t *motor = &s_align_motor_pos[0];

        double x = (double)star->ra_hours * 15.0 * 3600.0;
        double y = (double)star->dec_degrees * 3600.0;

        s_calibration.matrix_ra_to_ra = (float)OS_STEPS_PER_ARCSEC;
        s_calibration.matrix_ra_to_dec = 0.0f;
        s_calibration.matrix_dec_to_ra = 0.0f;
        s_calibration.matrix_dec_to_dec = (float)OS_STEPS_PER_ARCSEC;
        s_calibration.offset_ra_arcsec = (float)((double)motor->ra_steps - OS_STEPS_PER_ARCSEC * x);
        s_calibration.offset_dec_arcsec = (float)((double)motor->dec_steps - OS_STEPS_PER_ARCSEC * y);
        s_calibration.valid = true;

        s_align_residual_valid = false;
        s_align_residual_arcsec = 0.0f;

        return nvm_save_calibration();
    }

    if (s_align_mode == OS_ALIGN_2STAR) {
        double a = 0.0;
        double c = 0.0;
        double e = 0.0;
        double f = 0.0;

        const os_equatorial_coord_t *s1 = &s_align_stars[0];
        const os_motor_position_t *m1 = &s_align_motor_pos[0];
        const os_equatorial_coord_t *s2 = &s_align_stars[1];
        const os_motor_position_t *m2 = &s_align_motor_pos[1];

        double x1 = (double)s1->ra_hours * 15.0 * 3600.0;
        double x2 = (double)s2->ra_hours * 15.0 * 3600.0;
        double y1 = (double)s1->dec_degrees * 3600.0;
        double y2 = (double)s2->dec_degrees * 3600.0;

        if (!solve_2x2(x1, 1.0, x2, 1.0, (double)m1->ra_steps, (double)m2->ra_steps, &a, &c) ||
            !solve_2x2(y1, 1.0, y2, 1.0, (double)m1->dec_steps, (double)m2->dec_steps, &e, &f)) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        s_calibration.matrix_ra_to_ra = (float)a;
        s_calibration.matrix_ra_to_dec = 0.0f;
        s_calibration.matrix_dec_to_ra = 0.0f;
        s_calibration.matrix_dec_to_dec = (float)e;
        s_calibration.offset_ra_arcsec = (float)c;
        s_calibration.offset_dec_arcsec = (float)f;
        s_calibration.valid = true;

        s_align_residual_valid = false;
        s_align_residual_arcsec = 0.0f;

        return nvm_save_calibration();
    }

    double A[OS_CALIBRATION_MAX_STARS * 3];
    double b_ra[OS_CALIBRATION_MAX_STARS];
    double b_dec[OS_CALIBRATION_MAX_STARS];

    for (uint8_t i = 0; i < s_align_star_count; i++) {
        double x = (double)s_align_stars[i].ra_hours * 15.0 * 3600.0;
        double y = (double)s_align_stars[i].dec_degrees * 3600.0;

        A[i * 3 + 0] = x;
        A[i * 3 + 1] = y;
        A[i * 3 + 2] = 1.0;

        b_ra[i] = (double)s_align_motor_pos[i].ra_steps;
        b_dec[i] = (double)s_align_motor_pos[i].dec_steps;
    }

    double coef_ra[3] = {0.0, 0.0, 0.0};
    double coef_dec[3] = {0.0, 0.0, 0.0};

    if (!qr_solve_full(A, b_ra, s_align_star_count, coef_ra) ||
        !qr_solve_full(A, b_dec, s_align_star_count, coef_dec)) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    if (s_align_star_count == 3u && align_check_collinearity_3star()) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    if (s_align_star_count >= 4u) {
        double sum_sq = 0.0;

        for (uint8_t i = 0; i < s_align_star_count; i++) {
            double x = (double)s_align_stars[i].ra_hours * 15.0 * 3600.0;
            double y = (double)s_align_stars[i].dec_degrees * 3600.0;

            double pred_ra = coef_ra[0] * x + coef_ra[1] * y + coef_ra[2];
            double pred_dec = coef_dec[0] * x + coef_dec[1] * y + coef_dec[2];

            double diff_ra = pred_ra - (double)s_align_motor_pos[i].ra_steps;
            double diff_dec = pred_dec - (double)s_align_motor_pos[i].dec_steps;

            sum_sq += diff_ra * diff_ra + diff_dec * diff_dec;
        }

        double residual_steps = sqrt(sum_sq / (double)s_align_star_count);
        double residual_arcsec = residual_steps / OS_STEPS_PER_ARCSEC;

        if (residual_arcsec > (double)OS_CALIBRATION_RESIDUAL_LIMIT_ARCSEC) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        s_align_residual_arcsec = (float)residual_arcsec;
        s_align_residual_valid = true;
    } else {
        s_align_residual_arcsec = 0.0f;
        s_align_residual_valid = true;
    }

    s_calibration.matrix_ra_to_ra = (float)coef_ra[0];
    s_calibration.matrix_ra_to_dec = (float)coef_ra[1];
    s_calibration.matrix_dec_to_ra = (float)coef_dec[0];
    s_calibration.matrix_dec_to_dec = (float)coef_dec[1];
    s_calibration.offset_ra_arcsec = (float)coef_ra[2];
    s_calibration.offset_dec_arcsec = (float)coef_dec[2];
    s_calibration.valid = true;

    return nvm_save_calibration();
}

os_error_t os_init(void)
{
    reset_runtime_state();

    os_error_t first_error = OS_ERR_NONE;
    os_error_t err = OS_ERR_NONE;

    err = os_hal_nvm_init();
    if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) {
        first_error = err;
    }

    for (uint8_t ch = 0u; ch <= OS_CHANNEL_ETHERNET; ch++) {
        err = os_hal_comm_init(ch);
        if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) {
            first_error = err;
        }
    }

    for (uint8_t axis = 0u; axis < AXIS_COUNT; axis++) {
        err = os_hal_motor_init(axis);
        if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) {
            first_error = err;
        }
        (void)os_hal_motor_set_frequency(axis, 0u);
        (void)os_hal_motor_enable(axis, false);
    }

    err = os_hal_gps_init();
    if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) {
        first_error = err;
    }

    err = os_hal_rtc_init();
    if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) {
        first_error = err;
    }

    err = os_hal_limit_init();
    if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) {
        first_error = err;
    }

    err = os_hal_timer_motor_init();
    if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) {
        first_error = err;
    }

    os_site_info_t gps_site;
    err = os_hal_gps_poll(&gps_site);
    if (err == OS_ERR_NONE && gps_site.valid) {
        s_site = gps_site;
        s_gps_locked = true;
        (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
    } else {
        s_gps_locked = false;
        uint32_t rtc_time = 0u;
        if (os_hal_rtc_read(&rtc_time) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = rtc_time;
        }
    }

    nvm_load_calibration();
    nvm_load_pec();

    s_initialized = true;
    s_state = OS_STATE_IDLE_TRACKING;

    if (first_error != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
    }

    return first_error;
}

void os_loop_iteration(void)
{
    if (!s_initialized) {
        return;
    }

    for (uint8_t ch = 0u; ch <= OS_CHANNEL_ETHERNET; ch++) {
        while (os_hal_comm_available(ch) > 0) {
            char c = os_hal_comm_read(ch);

            if (s_cmd_len[ch] == 0u) {
                if (c == OS_LX200_CMD_PREFIX) {
                    s_cmd_buf[ch][s_cmd_len[ch]++] = c;
                }
                continue;
            }

            if (c == '\n' || c == '\r') {
                if (s_cmd_len[ch] > 0u &&
                    s_cmd_buf[ch][s_cmd_len[ch] - 1u] == OS_LX200_CMD_SUFFIX) {
                    s_cmd_buf[ch][s_cmd_len[ch]] = '\0';
                    char reply[OS_MAX_REPLY_LENGTH];
                    size_t reply_len = 0u;
                    (void)os_command_parse(s_cmd_buf[ch], s_cmd_len[ch], ch,
                                          reply, sizeof(reply), &reply_len);
                }
                s_cmd_len[ch] = 0u;
                continue;
            }

            if (s_cmd_len[ch] < (OS_MAX_COMMAND_LENGTH - 1u)) {
                s_cmd_buf[ch][s_cmd_len[ch]++] = c;
            } else {
                s_cmd_len[ch] = 0u;
                continue;
            }

            if (c == OS_LX200_CMD_SUFFIX) {
                s_cmd_buf[ch][s_cmd_len[ch]] = '\0';
                char reply[OS_MAX_REPLY_LENGTH];
                size_t reply_len = 0u;
                os_error_t parse_err = os_command_parse(
                    s_cmd_buf[ch], s_cmd_len[ch], ch,
                    reply, sizeof(reply), &reply_len);

                if (parse_err == OS_ERR_NONE && reply_len > 0u) {
                    (void)os_hal_comm_write(ch, reply, reply_len);
                }

                s_cmd_len[ch] = 0u;
            }
        }
    }

    os_site_info_t gps_site;
    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid) {
        s_site = gps_site;
        s_gps_locked = true;
        (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
    }

    if (!s_gps_locked) {
        uint32_t rtc_time = 0u;
        if (os_hal_rtc_read(&rtc_time) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = rtc_time;
        }
    }

    if (s_guide_pulse.active) {
        if (s_guide_pulse.duration_ms > 0u) {
            s_guide_pulse.duration_ms--;
        }
        if (s_guide_pulse.duration_ms == 0u) {
            s_guide_pulse.active = false;
        }
    }

    bool limit_ra = os_hal_limit_is_triggered(AXIS_RA);
    bool limit_dec = os_hal_limit_is_triggered(AXIS_DEC);

    if ((limit_ra || limit_dec) &&
        (s_state == OS_STATE_IDLE_TRACKING ||
         s_state == OS_STATE_GOTO ||
         s_state == OS_STATE_MANUAL_MOTION ||
         s_state == OS_STATE_ALIGNMENT)) {
        (void)os_hal_motor_set_frequency(AXIS_RA, 0u);
        (void)os_hal_motor_set_frequency(AXIS_DEC, 0u);
        s_goto_active = false;
        s_move_active = false;
        s_guide_pulse.active = false;
        s_park_pending = false;
        s_state = OS_STATE_FAULT;
    }

    if (s_state == OS_STATE_FAULT || s_state == OS_STATE_PARKED) {
        if (s_state == OS_STATE_FAULT) {
            (void)os_hal_motor_set_frequency(AXIS_RA, 0u);
            (void)os_hal_motor_set_frequency(AXIS_DEC, 0u);
        }
        return;
    }

    if (s_goto_active) {
        service_goto();
    } else if (s_move_active) {
        service_manual();
    } else if (s_state == OS_STATE_IDLE_TRACKING) {
        apply_tracking();
    } else if (s_state == OS_STATE_ALIGNMENT) {
        (void)os_hal_motor_set_frequency(AXIS_RA, 0u);
        (void)os_hal_motor_set_frequency(AXIS_DEC, 0u);
    }
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length)
{
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (source_channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (reply_buffer_size == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *reply_length = 0u;
    reply_buffer[0] = '\0';

    if (length < 2u || length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_COMMAND_FORMAT;
    }

    if (command[0] != OS_LX200_CMD_PREFIX ||
        command[length - 1u] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    char body[OS_MAX_COMMAND_LENGTH];
    size_t body_len = length - 2u;

    if (body_len >= sizeof(body)) {
        return OS_ERR_COMMAND_FORMAT;
    }

    (void)memcpy(body, command + 1u, body_len);
    body[body_len] = '\0';

    if (strcmp(body, "GR") == 0) {
        os_motor_position_t pos;
        float ra_hours = 0.0f;
        float dec_deg = 0.0f;

        pos.ra_steps = os_hal_motor_get_position(AXIS_RA);
        pos.dec_steps = os_hal_motor_get_position(AXIS_DEC);
        motor_steps_to_eq(pos.ra_steps, pos.dec_steps, &ra_hours, &dec_deg);

        return write_reply(reply_buffer, reply_buffer_size, reply_length,
                          "%07.4f#", (double)ra_hours);
    }

    if (strcmp(body, "GD") == 0) {
        os_motor_position_t pos;
        float ra_hours = 0.0f;
        float dec_deg = 0.0f;

        pos.ra_steps = os_hal_motor_get_position(AXIS_RA);
        pos.dec_steps = os_hal_motor_get_position(AXIS_DEC);
        motor_steps_to_eq(pos.ra_steps, pos.dec_steps, &ra_hours, &dec_deg);

        return write_reply(reply_buffer, reply_buffer_size, reply_length,
                          "%+08.4f#", (double)dec_deg);
    }

    if (strcmp(body, "GVP") == 0) {
        return write_reply(reply_buffer, reply_buffer_size, reply_length,
                          "%u.%u.%u#",
                          OS_FIRMWARE_VERSION_MAJOR,
                          OS_FIRMWARE_VERSION_MINOR,
                          OS_FIRMWARE_VERSION_PATCH);
    }

    if (strcmp(body, "MS") == 0) {
        if (!s_pending_target_valid) {
            os_query_coordinates(&s_pending_target);
            s_pending_target_valid = true;
        }

        os_error_t goto_err = os_goto_equatorial(s_pending_target);
        if (goto_err != OS_ERR_NONE) {
            return goto_err;
        }

        return write_reply(reply_buffer, reply_buffer_size, reply_length, "0#");
    }

    if (strncmp(body, "Sr", 2u) == 0) {
        float ra = 0.0f;
        if (!parse_ra_string(body + 2u, &ra)) {
            return OS_ERR_COMMAND_FORMAT;
        }

        s_pending_target.ra_hours = ra;
        s_pending_target_valid = true;

        return write_reply(reply_buffer, reply_buffer_size, reply_length, "0#");
    }

    if (strncmp(body, "Sd", 2u) == 0) {
        float dec = 0.0f;
        if (!parse_dec_string(body + 2u, &dec)) {
            return OS_ERR_COMMAND_FORMAT;
        }

        s_pending_target.dec_degrees = dec;
        s_pending_target_valid = true;

        return write_reply(reply_buffer, reply_buffer_size, reply_length, "0#");
    }

    if (strcmp(body, "Me") == 0) {
        os_error_t move_err = os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
        if (move_err != OS_ERR_NONE) {
            return move_err;
        }

        return write_reply(reply_buffer, reply_buffer_size, reply_length, "0#");
    }

    if (strcmp(body, "Mw") == 0) {
        os_error_t move_err = os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
        if (move_err != OS_ERR_NONE) {
            return move_err;
        }

        return write_reply(reply_buffer, reply_buffer_size, reply_length, "0#");
    }

    if (strcmp(body, "Mn") == 0) {
        os_error_t move_err = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
        if (move_err != OS_ERR_NONE) {
            return move_err;
        }

        return write_reply(reply_buffer, reply_buffer_size, reply_length, "0#");
    }

    if (strcmp(body, "Ms") == 0) {
        os_error_t move_err = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
        if (move_err != OS_ERR_NONE) {
            return move_err;
        }

        return write_reply(reply_buffer, reply_buffer_size, reply_length, "0#");
    }

    if (strcmp(body, "Q") == 0 || strcmp(body, "STOP") == 0) {
        if (s_goto_active) {
            return os_goto_abort();
        }
        if (s_move_active) {
            return os_move_stop();
        }

        return write_reply(reply_buffer, reply_buffer_size, reply_length, "0#");
    }

    if (strcmp(body, "hP") == 0) {
        os_error_t park_err = os_park();
        if (park_err != OS_ERR_NONE) {
            return park_err;
        }

        return write_reply(reply_buffer, reply_buffer_size, reply_length, "0#");
    }

    if (strcmp(body, "hO") == 0) {
        os_error_t unpark_err = os_unpark();
        if (unpark_err != OS_ERR_NONE) {
            return unpark_err;
        }

        return write_reply(reply_buffer, reply_buffer_size, reply_length, "0#");
    }

    return OS_ERR_COMMAND_FORMAT;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target)
{
    if (!valid_ra(target.ra_hours) || !valid_dec(target.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    if (os_hal_limit_is_triggered(AXIS_RA) || os_hal_limit_is_triggered(AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    os_motor_position_t target_steps;
    eq_to_motor_steps(target.ra_hours, target.dec_degrees, &target_steps);

    s_goto_target = target_steps;
    s_park_pending = false;
    s_goto_active = true;
    s_state = OS_STATE_GOTO;

    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target)
{
    if (!valid_az(target.azimuth_degrees) || !valid_alt(target.altitude_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    if (os_hal_limit_is_triggered(AXIS_RA) || os_hal_limit_is_triggered(AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_goto_target.ra_steps = (int32_t)(target.azimuth_degrees * OS_STEPS_PER_DEGREE);
    s_goto_target.dec_steps = (int32_t)(target.altitude_degrees * OS_STEPS_PER_DEGREE);
    s_park_pending = false;
    s_goto_active = true;
    s_state = OS_STATE_GOTO;

    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void)
{
    if (!s_goto_active) {
        return OS_ERR_INVALID_STATE;
    }

    s_goto_active = false;
    s_park_pending = false;
    s_state = OS_STATE_IDLE_TRACKING;

    (void)os_hal_motor_set_frequency(AXIS_RA, 0u);
    (void)os_hal_motor_set_frequency(AXIS_DEC, 0u);

    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor)
{
    if (!valid_track_rate(rate)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_track_rate = rate;
    if (rate == OS_TRACK_RATE_CUSTOM) {
        s_custom_track_factor = custom_factor;
    }

    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor)
{
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *rate = s_track_rate;
    *custom_factor = s_custom_track_factor;

    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void)
{
    s_tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void)
{
    s_tracking_enabled = false;
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms)
{
    if (!valid_direction(direction) || duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    bool is_dec = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);

    if (!s_guide_pulse.active) {
        s_guide_pulse.active = true;
        s_guide_pulse.duration_ms = duration_ms;
        s_guide_pulse.rate_fraction = s_guide_rate_fraction;
        s_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
        s_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
        s_guide_pulse.dec_priority = is_dec;
        return OS_ERR_NONE;
    }

    if (is_dec || !s_guide_pulse.dec_priority) {
        s_guide_pulse.active = true;
        s_guide_pulse.duration_ms = duration_ms;
        s_guide_pulse.rate_fraction = s_guide_rate_fraction;
        s_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
        s_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
        s_guide_pulse.dec_priority = is_dec;
    }

    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction)
{
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_guide_rate_fraction = rate_fraction;
    if (s_guide_pulse.active) {
        s_guide_pulse.rate_fraction = rate_fraction;
    }

    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse)
{
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *pulse = s_guide_pulse;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode)
{
    if (!valid_align_mode(mode)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    if (s_align_active) {
        return OS_ERR_INVALID_STATE;
    }

    s_align_active = true;
    s_align_mode = mode;
    s_align_star_count = 0u;
    s_align_residual_valid = false;
    s_align_residual_arcsec = 0.0f;
    s_state = OS_STATE_ALIGNMENT;

    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos)
{
    if (!valid_ra(star_coord.ra_hours) || !valid_dec(star_coord.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!s_align_active) {
        return OS_ERR_INVALID_STATE;
    }

    if (s_align_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    s_align_stars[s_align_star_count] = star_coord;
    s_align_motor_pos[s_align_star_count] = motor_pos;
    s_align_star_count++;

    return OS_ERR_NONE;
}

os_error_t os_align_compute(void)
{
    if (!s_align_active) {
        return OS_ERR_INVALID_STATE;
    }

    os_error_t compute_err = compute_calibration();

    if (compute_err == OS_ERR_NONE) {
        s_align_active = false;
        s_state = OS_STATE_IDLE_TRACKING;
    }

    return compute_err;
}

os_error_t os_align_get_residual(float *residual_arcsec)
{
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!s_align_residual_valid) {
        return OS_ERR_INVALID_STATE;
    }

    *residual_arcsec = s_align_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void)
{
    if (!s_align_active) {
        return OS_ERR_INVALID_STATE;
    }

    s_align_active = false;
    s_align_star_count = 0u;
    s_align_residual_valid = false;
    s_align_residual_arcsec = 0.0f;
    s_state = OS_STATE_IDLE_TRACKING;

    return OS_ERR_NONE;
}

os_error_t os_park(void)
{
    if (s_state == OS_STATE_PARKED) {
        return OS_ERR_NONE;
    }

    if (s_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    if (os_hal_limit_is_triggered(AXIS_RA) || os_hal_limit_is_triggered(AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    os_equatorial_coord_t park_pos = s_park_position;
    if (!s_park_position_set) {
        park_pos.ra_hours = 0.0f;
        park_pos.dec_degrees = 90.0f;
    }

    os_motor_position_t park_steps;
    eq_to_motor_steps(park_pos.ra_hours, park_pos.dec_degrees, &park_steps);

    s_goto_target = park_steps;
    s_park_pending = true;
    s_goto_active = true;
    s_state = OS_STATE_GOTO;

    return OS_ERR_NONE;
}

os_error_t os_unpark(void)
{
    if (s_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    for (uint8_t ch = 0u; ch <= OS_CHANNEL_ETHERNET; ch++) {
        (void)os_hal_comm_init(ch);
    }

    for (uint8_t axis = 0u; axis < AXIS_COUNT; axis++) {
        (void)os_hal_motor_enable(axis, true);
        (void)os_hal_motor_set_frequency(axis, 0u);
    }

    s_tracking_enabled = true;
    s_state = OS_STATE_IDLE_TRACKING;

    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos)
{
    if (!valid_ra(park_pos.ra_hours) || !valid_dec(park_pos.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_park_position = park_pos;
    s_park_position_set = true;

    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed)
{
    if (!valid_direction(direction) || !valid_speed(speed)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    if (s_goto_active) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t axis = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH)
                       ? AXIS_DEC
                       : AXIS_RA;

    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_move_active = true;
    s_move_direction = direction;
    s_move_speed = speed;
    s_state = OS_STATE_MANUAL_MOTION;

    return OS_ERR_NONE;
}

os_error_t os_move_stop(void)
{
    if (!s_move_active) {
        return OS_ERR_INVALID_STATE;
    }

    s_move_active = false;
    s_state = OS_STATE_IDLE_TRACKING;

    (void)os_hal_motor_set_frequency(AXIS_RA, 0u);
    (void)os_hal_motor_set_frequency(AXIS_DEC, 0u);

    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec)
{
    if (arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_move_custom_speed_arcsec_per_sec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state)
{
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *state = s_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord)
{
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    os_motor_position_t pos;
    pos.ra_steps = os_hal_motor_get_position(AXIS_RA);
    pos.dec_steps = os_hal_motor_get_position(AXIS_DEC);

    motor_steps_to_eq(pos.ra_steps, pos.dec_steps, &coord->ra_hours, &coord->dec_degrees);

    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *site = s_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos)
{
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    pos->ra_steps = os_hal_motor_get_position(AXIS_RA);
    pos->dec_steps = os_hal_motor_get_position(AXIS_DEC);

    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor, uint8_t *patch)
{
    if (major == NULL || minor == NULL || patch == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *major = OS_FIRMWARE_VERSION_MAJOR;
    *minor = OS_FIRMWARE_VERSION_MINOR;
    *patch = OS_FIRMWARE_VERSION_PATCH;

    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *moving)
{
    if (moving == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *moving = s_goto_active || s_move_active;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked)
{
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *locked = s_gps_locked;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable)
{
    s_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_pec_table = *table;
    s_pec_table.valid = true;

    return nvm_save_pec();
}

os_error_t os_pec_get_table(os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *table = s_pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec)
{
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    int index = (int)worm_phase_deg;
    if (index >= OS_PEC_TABLE_SIZE) {
        index = 0;
    }

    s_pec_table.corrections[index] = error_arcsec;
    s_pec_table.valid = true;

    return nvm_save_pec();
}

os_error_t os_calibration_get(os_calibration_t *calib)
{
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *calib = s_calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void)
{
    default_calibration();
    s_calibration.valid = false;

    return nvm_clear_calibration();
}

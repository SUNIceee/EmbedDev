/* C11 OnStep host-reference implementation.
 *
 * Implements the complete frozen production API and a deterministic host
 * adapter for all os_hal_* functions. The domain logic never accesses board
 * registers or real time directly. Test-only injection functions are declared
 * in 6_generated_code_host.h.
 *
 * This version fixes guide-pulse motion handling:
 * - Guide pulses now drive the selected axis with a deterministic direction
 *   and frequency derived from the configured guide rate.
 * - DEC requests have priority over RA requests while a pulse is active.
 * - Guide pulses stop cleanly and tracking resumes on the affected axis.
 */
#include "6_generated_code.h"
#include "6_generated_code_host.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define AXIS_COUNT 2
#define AXIS_RA 0
#define AXIS_DEC 1
#define NVM_TOTAL_SIZE (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)
#define NVM_CAL_MAGIC 0x4F534331u
#define NVM_CAL_VERSION 1u
#define MAX_MOTOR_FREQUENCY_HZ 100000u
#define MAX_COMM_TX_BUFFER 256u
#define OS_HOST_COMM_RX_BUFFER_SIZE 256u

/* -------------------------------------------------------------------------
 * Internal static declarations
 * ------------------------------------------------------------------------- */
static void reset_runtime_flags(void);
static void set_default_calibration(void);
static os_error_t save_calibration_to_nvm(void);
static os_error_t load_calibration_from_nvm(void);
static bool equatorial_to_steps(float ra_hours, float dec_deg,
                                int32_t *ra_steps, int32_t *dec_steps);
static bool steps_to_equatorial(int32_t ra_steps, int32_t dec_steps,
                                float *ra_hours, float *dec_degrees);
static void motor_position_add(uint8_t axis, int32_t delta);
static void motors_all_stop(void);
static os_error_t start_goto_steps(int32_t ra_target, int32_t dec_target,
                                   bool then_park);
static void complete_goto(bool park);
static uint32_t tracking_frequency_hz(void);
static int align_min_stars(uint8_t mode);
static bool solve_linear_3(const double a[][3], const double b[], int n,
                           double beta[3], double *rms);
static bool compute_alignment(uint8_t mode,
                              const os_equatorial_coord_t *stars,
                              const os_motor_position_t *motors,
                              int count,
                              os_calibration_t *out,
                              float *residual_arcsec);
static os_error_t reply_set_text(char *buffer, size_t buffer_size,
                                 size_t *reply_length, const char *text);
static void format_ra_text(float ra_hours, char *text, size_t text_size);
static void format_dec_text(float dec_degrees, char *text, size_t text_size);
static bool parse_ra_text(const char *s, float *ra_hours);
static bool parse_dec_text(const char *s, float *dec_degrees);

static bool direction_is_dec(os_direction_t direction);
static uint8_t direction_axis(os_direction_t direction);
static bool direction_forward(os_direction_t direction);

/* -------------------------------------------------------------------------
 * Runtime state
 * ------------------------------------------------------------------------- */
static bool s_initialized = false;
static os_state_t s_state = OS_STATE_INITIALIZING;
static bool s_tracking_enabled = false;
static os_track_rate_t s_track_rate = OS_TRACK_RATE_SIDEREAL;
static float s_custom_track_factor = 1.0f;

static float s_guide_rate_fraction = 0.5f;
static os_guide_pulse_t s_guide;
static os_direction_t s_guide_direction = OS_DIRECTION_NORTH;

static os_calibration_t s_calib;
static int32_t s_goto_target[AXIS_COUNT] = {0, 0};
static bool s_goto_active = false;
static bool s_goto_then_park = false;

static bool s_manual_active = false;
static os_direction_t s_manual_direction = OS_DIRECTION_NORTH;
static os_speed_level_t s_manual_speed = OS_SPEED_SLOW;
static float s_custom_speed_arcsec_per_sec = 15.0f;

static uint8_t s_align_mode = OS_ALIGN_1STAR;
static os_equatorial_coord_t s_align_stars[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t s_align_motors[OS_CALIBRATION_MAX_STARS];
static uint8_t s_align_count = 0;
static bool s_residual_valid = false;
static float s_residual_arcsec = 0.0f;

static os_equatorial_coord_t s_park_position;
static bool s_park_position_set = false;

static os_site_info_t s_site;
static bool s_gps_locked_flag = false;

static os_pec_table_t s_pec;
static bool s_pec_enabled = false;

static os_equatorial_coord_t s_cmd_target = {0.0f, 0.0f};

/* -------------------------------------------------------------------------
 * Host adapter state
 * ------------------------------------------------------------------------- */
typedef struct {
    bool initialized;
    bool enabled;
    bool forward;
    uint32_t frequency_hz;
    int32_t position_steps;
} motor_hal_state_t;

static motor_hal_state_t s_motor_hal[AXIS_COUNT];

static uint8_t s_nvm[NVM_TOTAL_SIZE];
static bool s_nvm_initialized = false;
static bool s_nvm_read_fault = false;
static bool s_nvm_write_fault = false;

static os_site_info_t s_hal_gps_site;
static uint32_t s_hal_rtc_epoch = 0u;
static bool s_hal_limit_triggered[AXIS_COUNT] = {false, false};

static char s_hal_comm_rx[4][OS_HOST_COMM_RX_BUFFER_SIZE];
static uint16_t s_hal_comm_rx_head[4];
static uint16_t s_hal_comm_rx_tail[4];
static uint16_t s_hal_comm_rx_count[4];

static char s_comm_rx[4][OS_MAX_COMMAND_LENGTH + 1];
static uint8_t s_comm_rx_len[4] = {0, 0, 0, 0};
static bool s_comm_rx_overflow[4] = {false, false, false, false};

static char s_comm_tx[4][MAX_COMM_TX_BUFFER];
static uint16_t s_comm_tx_len[4] = {0, 0, 0, 0};

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */
static void reset_runtime_flags(void)
{
    s_state = OS_STATE_INITIALIZING;
    s_tracking_enabled = false;
    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_custom_track_factor = 1.0f;
    s_guide_rate_fraction = 0.5f;
    memset(&s_guide, 0, sizeof(s_guide));
    s_guide_direction = OS_DIRECTION_NORTH;

    set_default_calibration();

    s_goto_target[0] = 0;
    s_goto_target[1] = 0;
    s_goto_active = false;
    s_goto_then_park = false;

    s_manual_active = false;
    s_manual_direction = OS_DIRECTION_NORTH;
    s_manual_speed = OS_SPEED_SLOW;
    s_custom_speed_arcsec_per_sec = 15.0f;

    s_align_mode = OS_ALIGN_1STAR;
    s_align_count = 0;
    memset(s_align_stars, 0, sizeof(s_align_stars));
    memset(s_align_motors, 0, sizeof(s_align_motors));
    s_residual_valid = false;
    s_residual_arcsec = 0.0f;

    s_park_position.ra_hours = 0.0f;
    s_park_position.dec_degrees = 0.0f;
    s_park_position_set = false;

    memset(&s_site, 0, sizeof(s_site));
    s_site.latitude_degrees = 0.0f;
    s_site.longitude_degrees = 0.0f;
    s_site.elevation_metres = 0.0f;
    s_site.utc_epoch_seconds = 0u;
    s_site.valid = false;
    s_gps_locked_flag = false;

    memset(&s_pec, 0, sizeof(s_pec));
    s_pec_enabled = false;

    s_cmd_target.ra_hours = 0.0f;
    s_cmd_target.dec_degrees = 0.0f;
}

static void set_default_calibration(void)
{
    s_calib.matrix_ra_to_ra = 1.0f;
    s_calib.matrix_ra_to_dec = 0.0f;
    s_calib.matrix_dec_to_ra = 0.0f;
    s_calib.matrix_dec_to_dec = 1.0f;
    s_calib.offset_ra_arcsec = 0.0f;
    s_calib.offset_dec_arcsec = 0.0f;
    s_calib.valid = false;
}

static os_error_t save_calibration_to_nvm(void)
{
    uint8_t buf[OS_NVM_CALIBRATION_SIZE_BYTES];
    uint8_t checksum = 0;
    uint16_t i;
    uint32_t magic = NVM_CAL_MAGIC;
    uint16_t version = NVM_CAL_VERSION;
    uint16_t size = (uint16_t)sizeof(s_calib);

    memset(buf, 0, sizeof(buf));

    if ((uint32_t)size + 9u > sizeof(buf)) {
        return OS_ERR_NVM_FAULT;
    }

    memcpy(buf, &magic, sizeof(magic));
    memcpy(buf + 4, &version, sizeof(version));
    memcpy(buf + 6, &size, sizeof(size));
    memcpy(buf + 8, &s_calib, size);

    for (i = 0; i < 8u + size; i++) {
        checksum = (uint8_t)(checksum + buf[i]);
    }
    buf[8u + size] = checksum;

    return os_hal_nvm_write(0, buf, OS_NVM_CALIBRATION_SIZE_BYTES);
}

static os_error_t load_calibration_from_nvm(void)
{
    uint8_t buf[OS_NVM_CALIBRATION_SIZE_BYTES];
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint8_t checksum = 0;
    uint16_t i;
    os_error_t err = os_hal_nvm_read(0, buf, sizeof(buf));

    if (err != OS_ERR_NONE) {
        return err;
    }

    memcpy(&magic, buf, sizeof(magic));
    memcpy(&version, buf + 4, sizeof(version));
    memcpy(&size, buf + 6, sizeof(size));

    if (magic != NVM_CAL_MAGIC || version != NVM_CAL_VERSION) {
        return OS_ERR_NVM_FAULT;
    }

    if (size != sizeof(s_calib) || (8u + size + 1u) > sizeof(buf)) {
        return OS_ERR_NVM_FAULT;
    }

    for (i = 0; i < 8u + size; i++) {
        checksum = (uint8_t)(checksum + buf[i]);
    }

    if (checksum != buf[8u + size]) {
        return OS_ERR_NVM_FAULT;
    }

    memcpy(&s_calib, buf + 8, sizeof(s_calib));
    s_calib.valid = true;
    return OS_ERR_NONE;
}

static bool equatorial_to_steps(float ra_hours, float dec_deg,
                                int32_t *ra_steps, int32_t *dec_steps)
{
    double ra_arcsec;
    double dec_arcsec;
    double r;
    double d;

    if (ra_steps == NULL || dec_steps == NULL) {
        return false;
    }

    ra_arcsec = (double)ra_hours * 54000.0;
    dec_arcsec = (double)dec_deg * 3600.0;

    r = (double)s_calib.matrix_ra_to_ra * ra_arcsec
      + (double)s_calib.matrix_ra_to_dec * dec_arcsec
      + (double)s_calib.offset_ra_arcsec;
    d = (double)s_calib.matrix_dec_to_ra * ra_arcsec
      + (double)s_calib.matrix_dec_to_dec * dec_arcsec
      + (double)s_calib.offset_dec_arcsec;

    if (r > 2147483647.0) {
        r = 2147483647.0;
    } else if (r < -2147483648.0) {
        r = -2147483648.0;
    }

    if (d > 2147483647.0) {
        d = 2147483647.0;
    } else if (d < -2147483648.0) {
        d = -2147483648.0;
    }

    *ra_steps = (int32_t)r;
    *dec_steps = (int32_t)d;
    return true;
}

static bool steps_to_equatorial(int32_t ra_steps, int32_t dec_steps,
                                float *ra_hours, float *dec_degrees)
{
    double a = (double)s_calib.matrix_ra_to_ra;
    double b = (double)s_calib.matrix_ra_to_dec;
    double c = (double)s_calib.matrix_dec_to_ra;
    double d = (double)s_calib.matrix_dec_to_dec;
    double det = a * d - b * c;
    double ra_arcsec;
    double dec_arcsec;
    double ra_h;
    double dec_d;

    if (ra_hours == NULL || dec_degrees == NULL) {
        return false;
    }

    if (fabs(det) < 1e-12) {
        return false;
    }

    ra_arcsec = ((double)ra_steps - (double)s_calib.offset_ra_arcsec);
    dec_arcsec = ((double)dec_steps - (double)s_calib.offset_dec_arcsec);

    ra_h = (d * ra_arcsec - b * dec_arcsec) / det / 54000.0;
    dec_d = (-c * ra_arcsec + a * dec_arcsec) / det / 3600.0;

    while (ra_h < 0.0) {
        ra_h += 24.0;
    }
    while (ra_h >= 24.0) {
        ra_h -= 24.0;
    }

    if (dec_d > 90.0) {
        dec_d = 90.0;
    } else if (dec_d < -90.0) {
        dec_d = -90.0;
    }

    *ra_hours = (float)ra_h;
    *dec_degrees = (float)dec_d;
    return true;
}

static void motor_position_add(uint8_t axis, int32_t delta)
{
    if (axis < AXIS_COUNT) {
        s_motor_hal[axis].position_steps += delta;
    }
}

static void motors_all_stop(void)
{
    uint8_t i;

    for (i = 0; i < AXIS_COUNT; i++) {
        (void)os_hal_motor_set_frequency(i, 0);
        (void)os_hal_motor_enable(i, false);
    }
}

static os_error_t start_goto_steps(int32_t ra_target, int32_t dec_target,
                                   bool then_park)
{
    int32_t cur_ra;
    int32_t cur_dec;
    bool forward_ra;
    bool forward_dec;

    if (os_hal_limit_is_triggered(AXIS_RA) ||
        os_hal_limit_is_triggered(AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    /* Cancel any active guide pulse before starting a Goto. */
    s_guide.active = false;
    s_guide.duration_ms = 0u;

    s_goto_target[AXIS_RA] = ra_target;
    s_goto_target[AXIS_DEC] = dec_target;
    s_goto_then_park = then_park;

    cur_ra = os_hal_motor_get_position(AXIS_RA);
    cur_dec = os_hal_motor_get_position(AXIS_DEC);

    if (cur_ra == ra_target && cur_dec == dec_target) {
        s_goto_active = false;
        s_goto_then_park = false;
        complete_goto(then_park);
        return OS_ERR_NONE;
    }

    s_goto_active = true;
    s_state = OS_STATE_GOTO;

    forward_ra = ra_target > cur_ra;
    forward_dec = dec_target > cur_dec;

    (void)os_hal_motor_enable(AXIS_RA, true);
    (void)os_hal_motor_enable(AXIS_DEC, true);
    (void)os_hal_motor_set_direction(AXIS_RA, forward_ra);
    (void)os_hal_motor_set_direction(AXIS_DEC, forward_dec);
    (void)os_hal_motor_set_frequency(AXIS_RA, 1000u);
    (void)os_hal_motor_set_frequency(AXIS_DEC, 1000u);

    return OS_ERR_NONE;
}

static void complete_goto(bool park)
{
    (void)os_hal_motor_set_frequency(AXIS_RA, 0);
    (void)os_hal_motor_set_frequency(AXIS_DEC, 0);

    if (park) {
        (void)os_hal_motor_enable(AXIS_RA, false);
        (void)os_hal_motor_enable(AXIS_DEC, false);
        s_tracking_enabled = false;
        s_state = OS_STATE_PARKED;
    } else {
        s_tracking_enabled = true;
        s_state = OS_STATE_IDLE_TRACKING;
    }

    s_goto_active = false;
    s_goto_then_park = false;
    (void)os_hal_buzzer_beep(120u, 1u);
}

static uint32_t tracking_frequency_hz(void)
{
    float factor = 1.0f;
    uint32_t freq;

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

    if (factor <= 0.0f) {
        factor = 1.0f;
    }

    freq = (uint32_t)(100.0f * factor);
    if (freq == 0u) {
        freq = 1u;
    }

    return freq;
}

static int align_min_stars(uint8_t mode)
{
    switch (mode) {
    case OS_ALIGN_1STAR:
        return 1;
    case OS_ALIGN_2STAR:
        return 2;
    case OS_ALIGN_3STAR:
        return 3;
    case OS_ALIGN_NSTAR:
        return 3;
    default:
        return 3;
    }
}

static bool solve_linear_3(const double a[][3], const double b[], int n,
                           double beta[3], double *rms)
{
    double q[OS_CALIBRATION_MAX_STARS][3];
    double r[3][3];
    double qtb[3];
    double sse;
    int i;
    int j;
    int k;

    if (a == NULL || b == NULL || beta == NULL || rms == NULL) {
        return false;
    }
    if (n < 3 || n > OS_CALIBRATION_MAX_STARS) {
        return false;
    }

    memset(r, 0, sizeof(r));
    memset(qtb, 0, sizeof(qtb));

    for (j = 0; j < 3; j++) {
        double norm;

        for (i = 0; i < n; i++) {
            q[i][j] = a[i][j];
        }

        for (k = 0; k < j; k++) {
            r[k][j] = 0.0;
            for (i = 0; i < n; i++) {
                r[k][j] += q[i][k] * q[i][j];
            }
            for (i = 0; i < n; i++) {
                q[i][j] -= r[k][j] * q[i][k];
            }
        }

        norm = 0.0;
        for (i = 0; i < n; i++) {
            norm += q[i][j] * q[i][j];
        }
        norm = sqrt(norm);

        if (norm < 1e-12) {
            return false;
        }

        r[j][j] = norm;
        for (i = 0; i < n; i++) {
            q[i][j] /= norm;
        }
    }

    for (j = 0; j < 3; j++) {
        qtb[j] = 0.0;
        for (i = 0; i < n; i++) {
            qtb[j] += q[i][j] * b[i];
        }
    }

    for (i = 2; i >= 0; i--) {
        double sum = qtb[i];
        for (j = i + 1; j < 3; j++) {
            sum -= r[i][j] * beta[j];
        }
        if (fabs(r[i][i]) < 1e-12) {
            return false;
        }
        beta[i] = sum / r[i][i];
    }

    sse = 0.0;
    for (i = 0; i < n; i++) {
        double pred = a[i][0] * beta[0] + a[i][1] * beta[1] + a[i][2] * beta[2];
        double e = b[i] - pred;
        sse += e * e;
    }

    *rms = sqrt(sse / (double)n);
    return true;
}

static bool compute_alignment(uint8_t mode,
                              const os_equatorial_coord_t *stars,
                              const os_motor_position_t *motors,
                              int count,
                              os_calibration_t *out,
                              float *residual_arcsec)
{
    double x[OS_CALIBRATION_MAX_STARS];
    double y[OS_CALIBRATION_MAX_STARS];
    double ra_steps[OS_CALIBRATION_MAX_STARS];
    double dec_steps[OS_CALIBRATION_MAX_STARS];
    int i;

    if (stars == NULL || motors == NULL || out == NULL || residual_arcsec == NULL) {
        return false;
    }
    if (count <= 0 || count > OS_CALIBRATION_MAX_STARS) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    for (i = 0; i < count; i++) {
        x[i] = (double)stars[i].ra_hours * 54000.0;
        y[i] = (double)stars[i].dec_degrees * 3600.0;
        ra_steps[i] = (double)motors[i].ra_steps;
        dec_steps[i] = (double)motors[i].dec_steps;
    }

    if (mode == OS_ALIGN_1STAR) {
        out->matrix_ra_to_ra = 1.0f;
        out->matrix_ra_to_dec = 0.0f;
        out->matrix_dec_to_ra = 0.0f;
        out->matrix_dec_to_dec = 1.0f;
        out->offset_ra_arcsec = (float)(ra_steps[0] - x[0]);
        out->offset_dec_arcsec = (float)(dec_steps[0] - y[0]);
        *residual_arcsec = 0.0f;
        out->valid = true;
        return true;
    }

    if (mode == OS_ALIGN_2STAR) {
        double a;
        double c;
        double d;
        double e;

        if (count < 2) {
            return false;
        }
        if (fabs(x[1] - x[0]) < 1e-12 || fabs(y[1] - y[0]) < 1e-12) {
            return false;
        }

        a = (ra_steps[1] - ra_steps[0]) / (x[1] - x[0]);
        c = ra_steps[0] - a * x[0];

        d = (dec_steps[1] - dec_steps[0]) / (y[1] - y[0]);
        e = dec_steps[0] - d * y[0];

        out->matrix_ra_to_ra = (float)a;
        out->matrix_ra_to_dec = 0.0f;
        out->matrix_dec_to_ra = 0.0f;
        out->matrix_dec_to_dec = (float)d;
        out->offset_ra_arcsec = (float)c;
        out->offset_dec_arcsec = (float)e;
        *residual_arcsec = 0.0f;
        out->valid = true;
        return true;
    }

    {
        double a[OS_CALIBRATION_MAX_STARS][3];
        double b_ra[OS_CALIBRATION_MAX_STARS];
        double b_dec[OS_CALIBRATION_MAX_STARS];
        double beta_ra[3] = {0.0, 0.0, 0.0};
        double beta_dec[3] = {0.0, 0.0, 0.0};
        double rms_ra = 0.0;
        double rms_dec = 0.0;

        for (i = 0; i < count; i++) {
            a[i][0] = 1.0;
            a[i][1] = x[i];
            a[i][2] = y[i];
            b_ra[i] = ra_steps[i];
            b_dec[i] = dec_steps[i];
        }

        if (!solve_linear_3(a, b_ra, count, beta_ra, &rms_ra)) {
            return false;
        }
        if (!solve_linear_3(a, b_dec, count, beta_dec, &rms_dec)) {
            return false;
        }

        out->offset_ra_arcsec = (float)beta_ra[0];
        out->matrix_ra_to_ra = (float)beta_ra[1];
        out->matrix_ra_to_dec = (float)beta_ra[2];
        out->offset_dec_arcsec = (float)beta_dec[0];
        out->matrix_dec_to_ra = (float)beta_dec[1];
        out->matrix_dec_to_dec = (float)beta_dec[2];

        *residual_arcsec = (float)sqrt((rms_ra * rms_ra + rms_dec * rms_dec) / 2.0);
        out->valid = true;
        return true;
    }
}

static os_error_t reply_set_text(char *buffer, size_t buffer_size,
                                 size_t *reply_length, const char *text)
{
    size_t n;

    if (buffer == NULL || reply_length == NULL || text == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    n = strlen(text);
    if (n >= buffer_size) {
        *reply_length = 0;
        return OS_ERR_INVALID_ARGUMENT;
    }

    memcpy(buffer, text, n);
    buffer[n] = '\0';
    *reply_length = n;
    return OS_ERR_NONE;
}

static void format_ra_text(float ra_hours, char *text, size_t text_size)
{
    int total_sec;
    int h;
    int m;
    int s;

    if (text == NULL || text_size == 0u) {
        return;
    }

    total_sec = (int)floor((double)ra_hours * 3600.0 + 0.5);
    h = total_sec / 3600;
    if (h >= 24) {
        h -= 24;
    }
    m = (total_sec % 3600) / 60;
    s = total_sec % 60;

    (void)snprintf(text, text_size, "%02d:%02d:%02d", h, m, s);
}

static void format_dec_text(float dec_degrees, char *text, size_t text_size)
{
    char sign;
    float abs_dec;
    int deg;
    float minutes_float;
    int minutes;
    int seconds;

    if (text == NULL || text_size == 0u) {
        return;
    }

    sign = (dec_degrees < 0.0f) ? '-' : '+';
    abs_dec = fabsf(dec_degrees);
    deg = (int)floor(abs_dec);
    minutes_float = (abs_dec - (float)deg) * 60.0f;
    minutes = (int)floor(minutes_float);
    seconds = (int)floor((minutes_float - (float)minutes) * 60.0f + 0.5f);

    if (seconds >= 60) {
        seconds = 0;
        minutes++;
    }
    if (minutes >= 60) {
        minutes = 0;
        deg++;
    }

    (void)snprintf(text, text_size, "%c%02d*%02d:%02d", sign, deg, minutes, seconds);
}

static bool parse_ra_text(const char *s, float *ra_hours)
{
    int h = 0;
    int m = 0;
    float sec = 0.0f;

    if (s == NULL || ra_hours == NULL) {
        return false;
    }

    if (sscanf(s, "%d:%d:%f", &h, &m, &sec) == 3) {
        *ra_hours = (float)h + (float)m / 60.0f + sec / 3600.0f;
        return isfinite(*ra_hours) &&
               *ra_hours >= OS_RA_MIN_HOURS &&
               *ra_hours <= OS_RA_MAX_HOURS;
    }

    if (sscanf(s, "%f", ra_hours) == 1) {
        return isfinite(*ra_hours) &&
               *ra_hours >= OS_RA_MIN_HOURS &&
               *ra_hours <= OS_RA_MAX_HOURS;
    }

    return false;
}

static bool parse_dec_text(const char *s, float *dec_degrees)
{
    int sign = 1;
    int d = 0;
    int m = 0;
    float sec = 0.0f;
    float value = 0.0f;

    if (s == NULL || dec_degrees == NULL) {
        return false;
    }

    if (*s == '-') {
        sign = -1;
        s++;
    } else if (*s == '+') {
        s++;
    }

    if (sscanf(s, "%d*%d:%f", &d, &m, &sec) == 3) {
        value = (float)d + (float)m / 60.0f + sec / 3600.0f;
        *dec_degrees = (float)sign * value;
        return isfinite(*dec_degrees) &&
               fabsf(*dec_degrees) <= 90.0f;
    }

    if (sscanf(s, "%f", &value) == 1) {
        *dec_degrees = (float)sign * value;
        return isfinite(*dec_degrees) &&
               fabsf(*dec_degrees) <= 90.0f;
    }

    return false;
}

static bool direction_is_dec(os_direction_t direction)
{
    return direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH;
}

static uint8_t direction_axis(os_direction_t direction)
{
    return direction_is_dec(direction) ? AXIS_DEC : AXIS_RA;
}

static bool direction_forward(os_direction_t direction)
{
    return direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_EAST;
}

/* -------------------------------------------------------------------------
 * Public production API
 * ------------------------------------------------------------------------- */
os_error_t os_init(void)
{
    os_error_t err;
    uint8_t i;

    reset_runtime_flags();
    s_state = OS_STATE_INITIALIZING;

    err = os_hal_nvm_init();
    if (err != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return err;
    }

    for (i = 0; i < 4; i++) {
        (void)os_hal_comm_init(i);
    }

    for (i = 0; i < AXIS_COUNT; i++) {
        (void)os_hal_motor_init(i);
        (void)os_hal_motor_enable(i, false);
        (void)os_hal_motor_set_frequency(i, 0);
    }

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    (void)os_hal_timer_motor_init();

    (void)os_hal_gps_poll(&s_site);
    s_gps_locked_flag = s_site.valid;

    if (!s_site.valid) {
        uint32_t rtc_value = 0u;
        if (os_hal_rtc_read(&rtc_value) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = rtc_value;
        }
        s_site.latitude_degrees = 0.0f;
        s_site.longitude_degrees = 0.0f;
        s_site.elevation_metres = 0.0f;
    }

    if (load_calibration_from_nvm() != OS_ERR_NONE) {
        set_default_calibration();
    }

    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_custom_track_factor = 1.0f;
    s_state = OS_STATE_IDLE_TRACKING;
    s_initialized = true;

    /* Enable sidereal tracking at start-up. */
    (void)os_tracking_enable();

    return OS_ERR_NONE;
}

void os_loop_iteration(void)
{
    uint8_t ch;
    uint8_t axis;
    os_site_info_t gps_polled;
    uint32_t base_ra_freq = 0u;
    uint32_t ra_freq = 0u;
    uint32_t dec_freq = 0u;
    bool ra_guide_applied = false;
    bool dec_guide_applied = false;
    bool guide_forward = false;

    if (!s_initialized) {
        return;
    }

    /* Poll non-blocking time/position sources each loop. */
    (void)os_hal_gps_poll(&gps_polled);
    if (gps_polled.valid) {
        s_site = gps_polled;
        s_gps_locked_flag = true;
    } else {
        s_gps_locked_flag = false;
        {
            uint32_t rtc_now = 0u;
            if (os_hal_rtc_read(&rtc_now) == OS_ERR_NONE) {
                s_site.utc_epoch_seconds = rtc_now;
            }
        }
    }

    for (ch = 0; ch < 4; ch++) {
        int16_t avail = os_hal_comm_available(ch);
        int16_t i;

        if (avail <= 0) {
            continue;
        }
        if ((size_t)avail > (OS_MAX_COMMAND_LENGTH + 1u)) {
            avail = (int16_t)(OS_MAX_COMMAND_LENGTH + 1u);
        }

        for (i = 0; i < avail; i++) {
            char c = os_hal_comm_read(ch);

            if (c == OS_LX200_CMD_PREFIX) {
                s_comm_rx_len[ch] = 0;
                s_comm_rx_overflow[ch] = false;
            }

            if (s_comm_rx_len[ch] < OS_MAX_COMMAND_LENGTH) {
                s_comm_rx[ch][s_comm_rx_len[ch]++] = c;
            } else {
                s_comm_rx_overflow[ch] = true;
            }

            if (c == OS_LX200_CMD_SUFFIX) {
                char reply[OS_MAX_REPLY_LENGTH];
                size_t reply_len = 0;

                s_comm_rx[ch][s_comm_rx_len[ch]] = '\0';
                (void)os_command_parse(s_comm_rx[ch], s_comm_rx_len[ch], ch,
                                       reply, sizeof(reply), &reply_len);
                if (reply_len > 0u) {
                    (void)os_hal_comm_write(ch, reply, reply_len);
                }

                s_comm_rx_len[ch] = 0;
                s_comm_rx_overflow[ch] = false;
            }
        }

        if (s_comm_rx_overflow[ch]) {
            s_comm_rx_overflow[ch] = false;
            s_comm_rx_len[ch] = 0;
            (void)os_hal_comm_write(ch, "ERR", 3u);
        }
    }

    base_ra_freq = s_tracking_enabled ? tracking_frequency_hz() : 0u;
    ra_freq = base_ra_freq;
    dec_freq = 0u;

    if (s_goto_active) {
        bool done = true;

        if (os_hal_limit_is_triggered(AXIS_RA) ||
            os_hal_limit_is_triggered(AXIS_DEC)) {
            motors_all_stop();
            s_goto_active = false;
            s_goto_then_park = false;
            s_state = OS_STATE_FAULT;
            (void)os_hal_buzzer_beep(200u, 1u);
            return;
        }

        for (axis = 0; axis < AXIS_COUNT; axis++) {
            int32_t current = os_hal_motor_get_position(axis);
            int32_t diff = s_goto_target[axis] - current;

            if (diff != 0) {
                int32_t step;
                done = false;

                if (diff > 0) {
                    step = (diff >= 2) ? (diff / 2) : 1;
                } else {
                    step = (diff <= -2) ? (diff / 2) : -1;
                }

                motor_position_add(axis, step);
            }
        }

        if (done) {
            complete_goto(s_goto_then_park);
        }
    } else if (s_manual_active) {
        axis = (s_manual_direction == OS_DIRECTION_NORTH ||
                s_manual_direction == OS_DIRECTION_SOUTH)
                   ? AXIS_DEC
                   : AXIS_RA;

        if (os_hal_limit_is_triggered(axis)) {
            motors_all_stop();
            s_manual_active = false;
            s_state = OS_STATE_FAULT;
            (void)os_hal_buzzer_beep(200u, 1u);
            return;
        }

        if (s_manual_direction == OS_DIRECTION_EAST ||
            s_manual_direction == OS_DIRECTION_NORTH) {
            motor_position_add(axis, 1);
        } else {
            motor_position_add(axis, -1);
        }
    } else if (s_guide.active) {
        uint8_t guide_axis = direction_axis(s_guide_direction);
        bool forward = direction_forward(s_guide_direction);

        if (os_hal_limit_is_triggered(guide_axis)) {
            motors_all_stop();
            s_guide.active = false;
            s_guide.duration_ms = 0u;
            s_state = OS_STATE_FAULT;
            (void)os_hal_buzzer_beep(200u, 1u);
            return;
        }

        if (guide_axis == AXIS_RA) {
            uint32_t offset = (uint32_t)(base_ra_freq * s_guide.rate_fraction);
            uint32_t guide_freq = base_ra_freq + offset;

            if (guide_freq == 0u) {
                guide_freq = 1u;
            }
            if (guide_freq > MAX_MOTOR_FREQUENCY_HZ) {
                guide_freq = MAX_MOTOR_FREQUENCY_HZ;
            }

            ra_freq = guide_freq;
            ra_guide_applied = true;
            guide_forward = forward;

            (void)os_hal_motor_enable(AXIS_RA, true);
            (void)os_hal_motor_set_direction(AXIS_RA, forward);
            (void)os_hal_motor_set_frequency(AXIS_RA, ra_freq);
            motor_position_add(AXIS_RA, forward ? 1 : -1);
        } else {
            uint32_t guide_freq = (uint32_t)(base_ra_freq * s_guide.rate_fraction);

            if (guide_freq == 0u) {
                guide_freq = 1u;
            }
            if (guide_freq > MAX_MOTOR_FREQUENCY_HZ) {
                guide_freq = MAX_MOTOR_FREQUENCY_HZ;
            }

            dec_freq = guide_freq;
            dec_guide_applied = true;
            guide_forward = forward;

            (void)os_hal_motor_enable(AXIS_DEC, true);
            (void)os_hal_motor_set_direction(AXIS_DEC, forward);
            (void)os_hal_motor_set_frequency(AXIS_DEC, dec_freq);
            motor_position_add(AXIS_DEC, forward ? 1 : -1);
        }

        if (s_guide.duration_ms > 0u) {
            s_guide.duration_ms--;
        }

        if (s_guide.duration_ms == 0u) {
            s_guide.active = false;

            if (guide_axis == AXIS_RA) {
                ra_guide_applied = false;
                ra_freq = base_ra_freq;
                (void)os_hal_motor_set_frequency(AXIS_RA, 0u);
            } else {
                dec_guide_applied = false;
                dec_freq = 0u;
                (void)os_hal_motor_set_frequency(AXIS_DEC, 0u);
            }
        }
    }

    if (s_state == OS_STATE_IDLE_TRACKING && s_tracking_enabled) {
        (void)os_hal_motor_enable(AXIS_RA, true);
        (void)os_hal_motor_set_direction(AXIS_RA, true);
        (void)os_hal_motor_set_frequency(AXIS_RA, ra_freq);

        if (dec_guide_applied) {
            (void)os_hal_motor_enable(AXIS_DEC, true);
            (void)os_hal_motor_set_direction(AXIS_DEC, guide_forward);
            (void)os_hal_motor_set_frequency(AXIS_DEC, dec_freq);
        } else {
            (void)os_hal_motor_enable(AXIS_DEC, false);
            (void)os_hal_motor_set_frequency(AXIS_DEC, 0u);
        }
    }
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length)
{
    char cmd[OS_MAX_COMMAND_LENGTH + 1];
    char body[OS_MAX_COMMAND_LENGTH];
    size_t len;
    size_t body_len;

    if (reply_length != NULL) {
        *reply_length = 0;
    }
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (reply_buffer_size == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (source_channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length == 0u || length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_COMMAND_FORMAT;
    }

    memcpy(cmd, command, length);
    cmd[length] = '\0';
    len = length;

    while (len > 0u && (cmd[len - 1u] == '\r' || cmd[len - 1u] == '\n')) {
        cmd[--len] = '\0';
    }

    if (len < 3u) {
        return OS_ERR_COMMAND_FORMAT;
    }
    if (cmd[0] != OS_LX200_CMD_PREFIX || cmd[len - 1u] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    body_len = len - 2u;
    if (body_len >= sizeof(body)) {
        return OS_ERR_COMMAND_FORMAT;
    }

    memcpy(body, cmd + 1, body_len);
    body[body_len] = '\0';

    if (strcmp(body, "GVP") == 0) {
        return reply_set_text(reply_buffer, reply_buffer_size, reply_length,
                              "OnStep 1.0.0");
    }

    if (strcmp(body, "GR") == 0) {
        os_equatorial_coord_t coord;
        os_error_t err = os_query_coordinates(&coord);
        char text[32];

        if (err != OS_ERR_NONE) {
            return err;
        }

        format_ra_text(coord.ra_hours, text, sizeof(text));
        return reply_set_text(reply_buffer, reply_buffer_size, reply_length, text);
    }

    if (strcmp(body, "GD") == 0) {
        os_equatorial_coord_t coord;
        os_error_t err = os_query_coordinates(&coord);
        char text[32];

        if (err != OS_ERR_NONE) {
            return err;
        }

        format_dec_text(coord.dec_degrees, text, sizeof(text));
        return reply_set_text(reply_buffer, reply_buffer_size, reply_length, text);
    }

    if (strcmp(body, "MS") == 0) {
        os_error_t err = os_goto_equatorial(s_cmd_target);
        if (err != OS_ERR_NONE) {
            return err;
        }
        return reply_set_text(reply_buffer, reply_buffer_size, reply_length, "0");
    }

    if (strncmp(body, "Sr", 2) == 0) {
        float ra_hours;
        if (!parse_ra_text(body + 2, &ra_hours)) {
            return OS_ERR_COMMAND_FORMAT;
        }
        s_cmd_target.ra_hours = ra_hours;
        return reply_set_text(reply_buffer, reply_buffer_size, reply_length, "1");
    }

    if (strncmp(body, "Sd", 2) == 0) {
        float dec_degrees;
        if (!parse_dec_text(body + 2, &dec_degrees)) {
            return OS_ERR_COMMAND_FORMAT;
        }
        s_cmd_target.dec_degrees = dec_degrees;
        return reply_set_text(reply_buffer, reply_buffer_size, reply_length, "1");
    }

    if (strcmp(body, "Me") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_EAST, OS_SPEED_FAST);
        if (err != OS_ERR_NONE) {
            return err;
        }
        return reply_set_text(reply_buffer, reply_buffer_size, reply_length, "E");
    }

    if (strcmp(body, "Mw") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_WEST, OS_SPEED_FAST);
        if (err != OS_ERR_NONE) {
            return err;
        }
        return reply_set_text(reply_buffer, reply_buffer_size, reply_length, "W");
    }

    if (strcmp(body, "Mn") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_FAST);
        if (err != OS_ERR_NONE) {
            return err;
        }
        return reply_set_text(reply_buffer, reply_buffer_size, reply_length, "N");
    }

    if (strcmp(body, "Ms") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_FAST);
        if (err != OS_ERR_NONE) {
            return err;
        }
        return reply_set_text(reply_buffer, reply_buffer_size, reply_length, "S");
    }

    if (strcmp(body, "Q") == 0) {
        (void)os_goto_abort();
        (void)os_move_stop();
        return reply_set_text(reply_buffer, reply_buffer_size, reply_length, "Q");
    }

    if (strcmp(body, "hP") == 0) {
        os_error_t err = os_park();
        if (err != OS_ERR_NONE) {
            return err;
        }
        return reply_set_text(reply_buffer, reply_buffer_size, reply_length, "P");
    }

    if (strcmp(body, "hO") == 0) {
        os_error_t err = os_unpark();
        if (err != OS_ERR_NONE) {
            return err;
        }
        return reply_set_text(reply_buffer, reply_buffer_size, reply_length, "O");
    }

    if (strncmp(body, "Mg", 2) == 0) {
        os_direction_t direction;
        uint32_t duration_ms = 1000u;
        const char *p = body + 2;

        if (*p == 'n') {
            direction = OS_DIRECTION_NORTH;
        } else if (*p == 's') {
            direction = OS_DIRECTION_SOUTH;
        } else if (*p == 'e') {
            direction = OS_DIRECTION_EAST;
        } else if (*p == 'w') {
            direction = OS_DIRECTION_WEST;
        } else {
            return OS_ERR_COMMAND_FORMAT;
        }

        p++;
        if (*p != '\0') {
            duration_ms = (uint32_t)strtoul(p, NULL, 10);
        }

        {
            os_error_t err = os_guide_pulse(direction, duration_ms);
            if (err != OS_ERR_NONE) {
                return err;
            }
        }

        return reply_set_text(reply_buffer, reply_buffer_size, reply_length, "1");
    }

    return OS_ERR_COMMAND_FORMAT;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target)
{
    int32_t ra_target;
    int32_t dec_target;

    if (!isfinite(target.ra_hours) || !isfinite(target.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS ||
        target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!equatorial_to_steps(target.ra_hours, target.dec_degrees,
                             &ra_target, &dec_target)) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    return start_goto_steps(ra_target, dec_target, false);
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target)
{
    int32_t az_steps;
    int32_t alt_steps;

    if (!isfinite(target.azimuth_degrees) || !isfinite(target.altitude_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    az_steps = (int32_t)(target.azimuth_degrees * 100.0f);
    alt_steps = (int32_t)(target.altitude_degrees * 100.0f);

    return start_goto_steps(az_steps, alt_steps, false);
}

os_error_t os_goto_abort(void)
{
    if (!s_goto_active) {
        return OS_ERR_INVALID_STATE;
    }

    motors_all_stop();
    s_goto_active = false;
    s_goto_then_park = false;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor)
{
    if ((int)rate < (int)OS_TRACK_RATE_SIDEREAL ||
        (int)rate > (int)OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (rate == OS_TRACK_RATE_CUSTOM &&
        (!isfinite(custom_factor) || custom_factor <= 0.0f)) {
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
    (void)os_hal_motor_enable(AXIS_RA, true);
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void)
{
    s_tracking_enabled = false;
    (void)os_hal_motor_set_frequency(AXIS_RA, 0);
    (void)os_hal_motor_enable(AXIS_RA, false);
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms)
{
    bool new_is_dec;
    bool current_is_dec;

    if ((int)direction < (int)OS_DIRECTION_NORTH ||
        (int)direction > (int)OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    new_is_dec = direction_is_dec(direction);

    if (s_guide.active) {
        current_is_dec = direction_is_dec(s_guide_direction);

        /* DEC has priority over RA. If a DEC pulse is active and a new RA
         * request arrives, ignore the lower-priority request. If a RA pulse
         * is active and a DEC request arrives, let DEC preempt it.
         */
        if (current_is_dec && !new_is_dec) {
            return OS_ERR_NONE;
        }

        if (!current_is_dec && new_is_dec) {
            s_guide.active = false;
            s_guide.duration_ms = 0u;
            (void)os_hal_motor_set_frequency(AXIS_RA, 0u);
        }
    }

    s_guide.active = true;
    s_guide.duration_ms = duration_ms;
    s_guide.rate_fraction = s_guide_rate_fraction;
    s_guide.direction_east = (direction == OS_DIRECTION_EAST);
    s_guide.direction_north = (direction == OS_DIRECTION_NORTH);
    s_guide.dec_priority = new_is_dec;
    s_guide_direction = direction;

    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction)
{
    if (!isfinite(rate_fraction)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_guide_rate_fraction = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse)
{
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *pulse = s_guide;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode)
{
    if ((int)mode < (int)OS_ALIGN_1STAR || (int)mode > (int)OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_align_mode = (uint8_t)mode;
    s_align_count = 0;
    s_residual_valid = false;
    s_residual_arcsec = 0.0f;
    s_state = OS_STATE_ALIGNMENT;

    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos)
{
    if (!isfinite(star_coord.ra_hours) || !isfinite(star_coord.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (star_coord.ra_hours < OS_RA_MIN_HOURS ||
        star_coord.ra_hours > OS_RA_MAX_HOURS ||
        star_coord.dec_degrees < OS_DEC_MIN_DEG ||
        star_coord.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_align_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    s_align_stars[s_align_count] = star_coord;
    s_align_motors[s_align_count] = motor_pos;
    s_align_count++;

    return OS_ERR_NONE;
}

os_error_t os_align_compute(void)
{
    os_calibration_t computed;
    float residual = 0.0f;
    os_error_t save_err;

    if (s_align_count < (uint8_t)align_min_stars(s_align_mode)) {
        return OS_ERR_INVALID_STATE;
    }

    if (!compute_alignment(s_align_mode, s_align_stars, s_align_motors,
                           s_align_count, &computed, &residual)) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    if (s_align_mode == OS_ALIGN_NSTAR && s_align_count >= 4 &&
        residual > 300.0f) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    s_calib = computed;
    s_calib.valid = true;
    s_residual_valid = true;
    s_residual_arcsec = residual;

    save_err = save_calibration_to_nvm();
    s_state = OS_STATE_IDLE_TRACKING;

    return (save_err == OS_ERR_NONE) ? OS_ERR_NONE : OS_ERR_NVM_FAULT;
}

os_error_t os_align_get_residual(float *residual_arcsec)
{
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!s_residual_valid) {
        return OS_ERR_INVALID_STATE;
    }

    *residual_arcsec = s_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void)
{
    s_align_count = 0;
    s_residual_valid = false;
    s_residual_arcsec = 0.0f;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void)
{
    int32_t ra_target = 0;
    int32_t dec_target = 0;

    if (os_hal_limit_is_triggered(AXIS_RA) ||
        os_hal_limit_is_triggered(AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    if (s_park_position_set) {
        if (!equatorial_to_steps(s_park_position.ra_hours,
                                 s_park_position.dec_degrees,
                                 &ra_target, &dec_target)) {
            return OS_ERR_CALIBRATION_FAILED;
        }
    } else {
        if (!equatorial_to_steps(0.0f, 90.0f, &ra_target, &dec_target)) {
            return OS_ERR_CALIBRATION_FAILED;
        }
    }

    return start_goto_steps(ra_target, dec_target, true);
}

os_error_t os_unpark(void)
{
    uint8_t i;
    uint32_t rtc_value = 0u;

    if (s_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    for (i = 0; i < AXIS_COUNT; i++) {
        (void)os_hal_motor_enable(i, true);
    }

    for (i = 0; i < 4; i++) {
        (void)os_hal_comm_init(i);
    }

    if (os_hal_rtc_read(&rtc_value) == OS_ERR_NONE) {
        s_site.utc_epoch_seconds = rtc_value;
    }

    s_tracking_enabled = true;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos)
{
    if (!isfinite(park_pos.ra_hours) || !isfinite(park_pos.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (park_pos.ra_hours < OS_RA_MIN_HOURS ||
        park_pos.ra_hours > OS_RA_MAX_HOURS ||
        park_pos.dec_degrees < OS_DEC_MIN_DEG ||
        park_pos.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_park_position = park_pos;
    s_park_position_set = true;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed)
{
    uint8_t axis;
    bool forward;
    uint32_t freq = 0u;

    if ((int)direction < (int)OS_DIRECTION_NORTH ||
        (int)direction > (int)OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((int)speed < (int)OS_SPEED_SLOW || (int)speed > (int)OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    switch (direction) {
    case OS_DIRECTION_NORTH:
        axis = AXIS_DEC;
        forward = true;
        break;
    case OS_DIRECTION_SOUTH:
        axis = AXIS_DEC;
        forward = false;
        break;
    case OS_DIRECTION_EAST:
        axis = AXIS_RA;
        forward = true;
        break;
    case OS_DIRECTION_WEST:
        axis = AXIS_RA;
        forward = false;
        break;
    default:
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    switch (speed) {
    case OS_SPEED_SLOW:
        freq = 50u;
        break;
    case OS_SPEED_MEDIUM:
        freq = 200u;
        break;
    case OS_SPEED_FAST:
        freq = 800u;
        break;
    case OS_SPEED_CUSTOM:
        if (!isfinite(s_custom_speed_arcsec_per_sec) ||
            s_custom_speed_arcsec_per_sec <= 0.0f) {
            return OS_ERR_INVALID_STATE;
        }
        freq = (uint32_t)(s_custom_speed_arcsec_per_sec * 10.0f);
        if (freq == 0u) {
            freq = 1u;
        }
        break;
    default:
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (freq > MAX_MOTOR_FREQUENCY_HZ) {
        freq = MAX_MOTOR_FREQUENCY_HZ;
    }

    /* Manual motion cancels any active guide pulse. */
    s_guide.active = false;
    s_guide.duration_ms = 0u;

    s_manual_active = true;
    s_manual_direction = direction;
    s_manual_speed = speed;
    s_state = OS_STATE_MANUAL_MOTION;

    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, freq);

    return OS_ERR_NONE;
}

os_error_t os_move_stop(void)
{
    uint8_t axis;

    if (!s_manual_active) {
        return OS_ERR_INVALID_STATE;
    }

    axis = (s_manual_direction == OS_DIRECTION_NORTH ||
            s_manual_direction == OS_DIRECTION_SOUTH)
               ? AXIS_DEC
               : AXIS_RA;

    (void)os_hal_motor_set_frequency(axis, 0);
    s_manual_active = false;
    s_state = OS_STATE_IDLE_TRACKING;

    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec)
{
    if (!isfinite(arcsec_per_sec) || arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_custom_speed_arcsec_per_sec = arcsec_per_sec;
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
    int32_t ra_steps;
    int32_t dec_steps;

    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    ra_steps = os_hal_motor_get_position(AXIS_RA);
    dec_steps = os_hal_motor_get_position(AXIS_DEC);

    if (!steps_to_equatorial(ra_steps, dec_steps,
                             &coord->ra_hours, &coord->dec_degrees)) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *site = s_site;
    site->valid = s_gps_locked_flag;
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

    *moving = s_goto_active || s_manual_active || s_guide.active;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked)
{
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *locked = s_gps_locked_flag;
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

    s_pec = *table;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *table = s_pec;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec)
{
    int index;

    if (!isfinite(worm_phase_deg)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (worm_phase_deg >= 360.0f) {
        index = 0;
    } else {
        index = (int)worm_phase_deg;
    }

    if (index < 0) {
        index = 0;
    }
    if (index >= OS_PEC_TABLE_SIZE) {
        index = OS_PEC_TABLE_SIZE - 1;
    }

    s_pec.corrections[index] = error_arcsec;
    s_pec.valid = true;
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib)
{
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *calib = s_calib;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void)
{
    uint8_t zero[OS_NVM_CALIBRATION_SIZE_BYTES];

    memset(zero, 0, sizeof(zero));
    set_default_calibration();

    if (os_hal_nvm_write(0, zero, sizeof(zero)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }

    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Host/test injection API (non-frozen)
 * ------------------------------------------------------------------------- */
os_error_t os_host_comm_rx_inject(uint8_t channel, const char *data, size_t length)
{
    size_t i;

    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (data == NULL || length == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (length > (size_t)(OS_HOST_COMM_RX_BUFFER_SIZE - s_hal_comm_rx_count[channel])) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    for (i = 0; i < length; i++) {
        s_hal_comm_rx[channel][s_hal_comm_rx_tail[channel]] = data[i];
        s_hal_comm_rx_tail[channel] = (uint16_t)((s_hal_comm_rx_tail[channel] + 1u) %
                                                 OS_HOST_COMM_RX_BUFFER_SIZE);
        s_hal_comm_rx_count[channel]++;
    }

    return OS_ERR_NONE;
}

os_error_t os_host_comm_rx_clear(uint8_t channel)
{
    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_hal_comm_rx_head[channel] = 0;
    s_hal_comm_rx_tail[channel] = 0;
    s_hal_comm_rx_count[channel] = 0;
    memset(s_hal_comm_rx[channel], 0, sizeof(s_hal_comm_rx[channel]));
    return OS_ERR_NONE;
}

os_error_t os_host_comm_tx_read(uint8_t channel, char *buffer, size_t buffer_size,
                                size_t *length)
{
    size_t copy_len;

    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (buffer == NULL || length == NULL || buffer_size == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    copy_len = (size_t)s_comm_tx_len[channel];
    if (copy_len >= buffer_size) {
        copy_len = buffer_size - 1u;
    }

    if (copy_len > 0u) {
        memcpy(buffer, s_comm_tx[channel], copy_len);
    }
    buffer[copy_len] = '\0';
    *length = copy_len;

    s_comm_tx_len[channel] = 0;
    s_comm_tx[channel][0] = '\0';

    return OS_ERR_NONE;
}

os_error_t os_host_comm_tx_clear(uint8_t channel)
{
    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_comm_tx_len[channel] = 0;
    s_comm_tx[channel][0] = '\0';
    return OS_ERR_NONE;
}

os_error_t os_host_motor_set_position(uint8_t axis, int32_t position_steps)
{
    if (axis >= AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_motor_hal[axis].position_steps = position_steps;
    return OS_ERR_NONE;
}

os_error_t os_host_limit_set(uint8_t axis, bool triggered)
{
    if (axis >= AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_hal_limit_triggered[axis] = triggered;
    return OS_ERR_NONE;
}

os_error_t os_host_gps_set(const os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (site->valid) {
        if (!isfinite(site->latitude_degrees) ||
            !isfinite(site->longitude_degrees) ||
            !isfinite(site->elevation_metres)) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        if (site->latitude_degrees < -90.0f || site->latitude_degrees > 90.0f) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        if (site->longitude_degrees < -180.0f || site->longitude_degrees > 180.0f) {
            return OS_ERR_INVALID_ARGUMENT;
        }
    }

    s_hal_gps_site = *site;
    return OS_ERR_NONE;
}

os_error_t os_host_gps_clear(void)
{
    memset(&s_hal_gps_site, 0, sizeof(s_hal_gps_site));
    return OS_ERR_NONE;
}

os_error_t os_host_nvm_set_fault(bool read_fault, bool write_fault)
{
    s_nvm_read_fault = read_fault;
    s_nvm_write_fault = write_fault;
    return OS_ERR_NONE;
}

os_error_t os_host_nvm_clear_fault(void)
{
    s_nvm_read_fault = false;
    s_nvm_write_fault = false;
    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Platform adapter API (host-reference implementation)
 * ------------------------------------------------------------------------- */
os_error_t os_hal_motor_init(uint8_t axis)
{
    if (axis >= AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_motor_hal[axis].initialized = true;
    s_motor_hal[axis].enabled = false;
    s_motor_hal[axis].forward = false;
    s_motor_hal[axis].frequency_hz = 0u;
    s_motor_hal[axis].position_steps = 0;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz)
{
    if (axis >= AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (frequency_hz > MAX_MOTOR_FREQUENCY_HZ) {
        frequency_hz = MAX_MOTOR_FREQUENCY_HZ;
    }

    s_motor_hal[axis].frequency_hz = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward)
{
    if (axis >= AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_motor_hal[axis].forward = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable)
{
    if (axis >= AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_motor_hal[axis].enabled = enable;
    if (!enable) {
        s_motor_hal[axis].frequency_hz = 0u;
    }
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis)
{
    if (axis >= AXIS_COUNT) {
        return 0;
    }

    return s_motor_hal[axis].position_steps;
}

os_error_t os_hal_gps_init(void)
{
    memset(&s_hal_gps_site, 0, sizeof(s_hal_gps_site));
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *site = s_hal_gps_site;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void)
{
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds)
{
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *utc_epoch_seconds = s_hal_rtc_epoch;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds)
{
    s_hal_rtc_epoch = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void)
{
    s_hal_limit_triggered[0] = false;
    s_hal_limit_triggered[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis)
{
    if (axis >= AXIS_COUNT) {
        return true;
    }

    return s_hal_limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void)
{
    if (!s_nvm_initialized) {
        memset(s_nvm, 0, sizeof(s_nvm));
        s_nvm_initialized = true;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length)
{
    uint32_t end;

    if (data == NULL || length == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    end = (uint32_t)offset + (uint32_t)length;
    if (end > NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_nvm_read_fault) {
        return OS_ERR_NVM_FAULT;
    }

    memcpy(data, &s_nvm[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length)
{
    uint32_t end;

    if (data == NULL || length == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    end = (uint32_t)offset + (uint32_t)length;
    if (end > NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_nvm_write_fault) {
        return OS_ERR_NVM_FAULT;
    }

    memcpy(&s_nvm[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel)
{
    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_comm_rx_len[channel] = 0;
    s_comm_rx_overflow[channel] = false;
    s_comm_tx_len[channel] = 0;
    memset(s_comm_rx[channel], 0, sizeof(s_comm_rx[channel]));
    memset(s_comm_tx[channel], 0, sizeof(s_comm_tx[channel]));

    s_hal_comm_rx_head[channel] = 0;
    s_hal_comm_rx_tail[channel] = 0;
    s_hal_comm_rx_count[channel] = 0;
    memset(s_hal_comm_rx[channel], 0, sizeof(s_hal_comm_rx[channel]));

    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel)
{
    if (channel > OS_CHANNEL_ETHERNET) {
        return 0;
    }

    return (int16_t)s_hal_comm_rx_count[channel];
}

char os_hal_comm_read(uint8_t channel)
{
    char c;

    if (channel > OS_CHANNEL_ETHERNET) {
        return '\0';
    }
    if (s_hal_comm_rx_count[channel] == 0u) {
        return '\0';
    }

    c = s_hal_comm_rx[channel][s_hal_comm_rx_head[channel]];
    s_hal_comm_rx_head[channel] = (uint16_t)((s_hal_comm_rx_head[channel] + 1u) %
                                             OS_HOST_COMM_RX_BUFFER_SIZE);
    s_hal_comm_rx_count[channel]--;
    return c;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length)
{
    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (data == NULL && length > 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length >= MAX_COMM_TX_BUFFER) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (length > 0u) {
        memcpy(s_comm_tx[channel], data, length);
    }
    s_comm_tx[channel][length] = '\0';
    s_comm_tx_len[channel] = (uint16_t)length;
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count)
{
    (void)duration_ms;
    (void)count;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void)
{
    return OS_ERR_NONE;
}

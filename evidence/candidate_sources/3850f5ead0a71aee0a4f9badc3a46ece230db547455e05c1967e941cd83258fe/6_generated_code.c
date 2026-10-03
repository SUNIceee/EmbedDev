#include "6_generated_code.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define OS_NVM_TOTAL_SIZE (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)
#define OS_LOOP_TICK_MS 1u
#define OS_GOTO_STEP_INCREMENT 1000
#define OS_MANUAL_SPEED_SLOW_HZ 100u
#define OS_MANUAL_SPEED_MEDIUM_HZ 1000u
#define OS_MANUAL_SPEED_FAST_HZ 10000u
#define OS_CALIBRATION_RESIDUAL_MAX_ARCSEC 300.0
#define OS_NVM_CAL_MAGIC 0x4F534E43u
#define OS_NVM_CFG_MAGIC 0x43464753u

typedef struct {
    double ra_arcsec;
    double dec_arcsec;
    double motor_ra;
    double motor_dec;
} align_sample_t;

typedef struct {
    uint32_t magic;
    uint32_t size;
    os_calibration_t calib;
    uint32_t checksum;
} nvm_cal_record_t;

typedef struct {
    uint32_t magic;
    uint32_t size;
    os_equatorial_coord_t park_pos;
    bool custom_park;
    uint32_t checksum;
} nvm_cfg_record_t;

static os_state_t s_telescope_state = OS_STATE_INITIALIZING;
static bool s_motion_active = false;
static bool s_goto_active = false;
static bool s_parking_active = false;
static bool s_manual_motion_active = false;

static bool s_tracking_enabled = false;
static os_track_rate_t s_track_rate = OS_TRACK_RATE_SIDEREAL;
static float s_custom_track_factor = 1.0f;

static os_guide_pulse_t s_guide;
static float s_guide_rate_fraction = 0.5f;

static os_align_mode_t s_align_mode = OS_ALIGN_1STAR;
static uint8_t s_align_count = 0;
static align_sample_t s_align_samples[OS_CALIBRATION_MAX_STARS];

static bool s_calibration_valid = false;
static os_calibration_t s_calibration;
static double s_cal_d[4] = {1.0, 0.0, 0.0, 1.0};
static double s_cal_off[2] = {0.0, 0.0};
static bool s_residual_computed = false;
static float s_residual_arcsec = 0.0f;

static bool s_park_custom = false;
static os_equatorial_coord_t s_park_pos = {0.0f, 90.0f};
static int32_t s_goto_target_ra_steps = 0;
static int32_t s_goto_target_dec_steps = 0;

static bool s_pec_enabled = false;
static os_pec_table_t s_pec_table;

static os_site_info_t s_site;
static bool s_gps_locked = false;
static bool s_gps_initialized = false;
static bool s_rtc_valid = false;
static bool s_rtc_initialized = false;
static uint32_t s_rtc_epoch_seconds = 1700000000u;

static bool s_motor_initialized[2] = {false, false};
static bool s_motor_enabled[2] = {false, false};
static uint32_t s_motor_freq[2] = {0u, 0u};
static bool s_motor_direction[2] = {false, false};
static int32_t s_motor_pos[2] = {0, 0};
static bool s_limit_triggered[2] = {false, false};
static bool s_motor_driver_fault[2] = {false, false};

static bool s_motor_timer_initialized = false;
static bool s_nvm_initialized = false;
static uint8_t s_nvm_memory[OS_NVM_TOTAL_SIZE];

static bool s_comm_initialized[4] = {false, false, false, false};
static char s_comm_tx[4][OS_MAX_REPLY_LENGTH];
static size_t s_comm_tx_len[4] = {0u, 0u, 0u, 0u};

static uint32_t s_loop_tick_ms = 0u;
static uint16_t s_buzzer_duration_ms = 0u;
static uint8_t s_buzzer_count = 0u;
static os_error_t s_last_error = OS_ERR_NONE;

static bool cmd_eq(const char *cmd, size_t len, const char *lit) {
    size_t n = strlen(lit);
    return len == n && memcmp(cmd, lit, n) == 0;
}

static bool valid_axis(uint8_t axis) {
    return axis == 0u || axis == 1u;
}

static bool valid_channel(uint8_t channel) {
    return channel == OS_CHANNEL_USB || channel == OS_CHANNEL_BLUETOOTH ||
           channel == OS_CHANNEL_WIFI || channel == OS_CHANNEL_ETHERNET;
}

static uint32_t nvm_checksum_bytes(const uint8_t *data, size_t len) {
    uint32_t sum = 0u;
    for (size_t i = 0u; i < len; i++) {
        sum += (uint32_t)data[i];
    }
    return sum;
}

static void set_reply(char *buf, size_t cap, size_t *len, const char *fmt, ...) {
    if (len != NULL) {
        *len = 0u;
    }
    if (buf == NULL || cap == 0u) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, cap, fmt, ap);
    va_end(ap);
    if (n < 0) {
        buf[0] = '\0';
        return;
    }
    size_t out = (size_t)n;
    if (out >= cap) {
        out = cap - 1u;
    }
    if (len != NULL) {
        *len = out;
    }
}

static void sync_cal_public_from_internal(void) {
    s_calibration.matrix_ra_to_ra = (float)s_cal_d[0];
    s_calibration.matrix_ra_to_dec = (float)s_cal_d[1];
    s_calibration.matrix_dec_to_ra = (float)s_cal_d[2];
    s_calibration.matrix_dec_to_dec = (float)s_cal_d[3];
    s_calibration.offset_ra_arcsec = (float)s_cal_off[0];
    s_calibration.offset_dec_arcsec = (float)s_cal_off[1];
    s_calibration.valid = s_calibration_valid;
}

static void sync_cal_internal_from_public(void) {
    s_cal_d[0] = (double)s_calibration.matrix_ra_to_ra;
    s_cal_d[1] = (double)s_calibration.matrix_ra_to_dec;
    s_cal_d[2] = (double)s_calibration.matrix_dec_to_ra;
    s_cal_d[3] = (double)s_calibration.matrix_dec_to_dec;
    s_cal_off[0] = (double)s_calibration.offset_ra_arcsec;
    s_cal_off[1] = (double)s_calibration.offset_dec_arcsec;
}

static void reset_runtime_flags(void) {
    s_motion_active = false;
    s_goto_active = false;
    s_parking_active = false;
    s_manual_motion_active = false;
    s_guide.active = false;
    s_guide.duration_ms = 0u;
    s_guide.rate_fraction = OS_GUIDE_RATE_MIN;
    s_guide.direction_east = false;
    s_guide.direction_north = false;
    s_guide.dec_priority = false;
    s_align_count = 0u;
    s_residual_computed = false;
    s_residual_arcsec = 0.0f;
    s_calibration_valid = false;
    memset(&s_calibration, 0, sizeof(s_calibration));
    memset(s_cal_d, 0, sizeof(s_cal_d));
    memset(s_cal_off, 0, sizeof(s_cal_off));
    s_cal_d[0] = 1.0;
    s_cal_d[3] = 1.0;
    s_pec_enabled = false;
    memset(&s_pec_table, 0, sizeof(s_pec_table));
    s_last_error = OS_ERR_NONE;
}

static bool nvm_read_calibration_record(void) {
    nvm_cal_record_t rec;
    memset(&rec, 0, sizeof(rec));
    os_error_t err = os_hal_nvm_read(0u, (uint8_t *)&rec, (uint16_t)sizeof(rec));
    if (err != OS_ERR_NONE) {
        return false;
    }
    uint32_t expected = rec.checksum;
    rec.checksum = 0u;
    uint32_t actual = nvm_checksum_bytes((const uint8_t *)&rec, sizeof(rec));
    if (rec.magic != OS_NVM_CAL_MAGIC || rec.size != sizeof(rec) || expected != actual) {
        return false;
    }
    s_calibration = rec.calib;
    s_calibration_valid = rec.calib.valid;
    sync_cal_internal_from_public();
    return true;
}

static bool nvm_read_config_record(void) {
    nvm_cfg_record_t rec;
    memset(&rec, 0, sizeof(rec));
    os_error_t err = os_hal_nvm_read(OS_NVM_CALIBRATION_SIZE_BYTES, (uint8_t *)&rec, (uint16_t)sizeof(rec));
    if (err != OS_ERR_NONE) {
        return false;
    }
    uint32_t expected = rec.checksum;
    rec.checksum = 0u;
    uint32_t actual = nvm_checksum_bytes((const uint8_t *)&rec, sizeof(rec));
    if (rec.magic != OS_NVM_CFG_MAGIC || rec.size != sizeof(rec) || expected != actual) {
        return false;
    }
    s_park_pos = rec.park_pos;
    s_park_custom = rec.custom_park;
    return true;
}

static void restore_persistent_state(void) {
    (void)nvm_read_calibration_record();
    (void)nvm_read_config_record();
}

static os_error_t persist_calibration_from(const os_calibration_t *calib) {
    nvm_cal_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = OS_NVM_CAL_MAGIC;
    rec.size = sizeof(rec);
    rec.calib = *calib;
    return os_hal_nvm_write(0u, (const uint8_t *)&rec, (uint16_t)sizeof(rec));
}

static os_error_t persist_park_config(void) {
    nvm_cfg_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = OS_NVM_CFG_MAGIC;
    rec.size = sizeof(rec);
    rec.park_pos = s_park_pos;
    rec.custom_park = s_park_custom;
    return os_hal_nvm_write(OS_NVM_CALIBRATION_SIZE_BYTES, (const uint8_t *)&rec, (uint16_t)sizeof(rec));
}

static void stop_all_axis_frequencies(void) {
    for (uint8_t ax = 0u; ax < 2u; ax++) {
        (void)os_hal_motor_set_frequency(ax, 0u);
    }
}

static void disable_all_motors(void) {
    for (uint8_t ax = 0u; ax < 2u; ax++) {
        (void)os_hal_motor_enable(ax, false);
    }
}

static void update_tracking_frequency(void) {
    if (!s_tracking_enabled || s_telescope_state != OS_STATE_IDLE_TRACKING) {
        return;
    }
    double rate = (double)OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
    if (s_track_rate == OS_TRACK_RATE_LUNAR) {
        rate *= (double)OS_LUNAR_RATE_FACTOR;
    } else if (s_track_rate == OS_TRACK_RATE_SOLAR) {
        rate *= (double)OS_SOLAR_RATE_FACTOR;
    } else if (s_track_rate == OS_TRACK_RATE_CUSTOM) {
        rate *= (double)s_custom_track_factor;
    }
    (void)os_hal_motor_set_frequency(0u, (uint32_t)rate);
    (void)os_hal_motor_enable(0u, true);
}

static void enter_fault(os_error_t err) {
    s_last_error = err;
    s_telescope_state = OS_STATE_FAULT;
    s_motion_active = false;
    s_goto_active = false;
    s_parking_active = false;
    s_manual_motion_active = false;
    s_guide.active = false;
    stop_all_axis_frequencies();
    disable_all_motors();
}

static void goto_set_motion_outputs(void) {
    int32_t p0 = os_hal_motor_get_position(0u);
    int32_t p1 = os_hal_motor_get_position(1u);
    bool fwd0 = s_goto_target_ra_steps >= p0;
    bool fwd1 = s_goto_target_dec_steps >= p1;
    (void)os_hal_motor_set_direction(0u, fwd0);
    (void)os_hal_motor_set_direction(1u, fwd1);
    uint32_t speed = (uint32_t)((double)OS_GOTO_SPEED_MAX_DEG_PER_SEC * 3600.0);
    (void)os_hal_motor_set_frequency(0u, speed);
    (void)os_hal_motor_set_frequency(1u, speed);
    (void)os_hal_motor_enable(0u, true);
    (void)os_hal_motor_enable(1u, true);
}

static void finish_goto(void) {
    s_goto_active = false;
    s_motion_active = false;
    s_telescope_state = OS_STATE_IDLE_TRACKING;
    s_tracking_enabled = true;
    stop_all_axis_frequencies();
    update_tracking_frequency();
    (void)os_hal_buzzer_beep(50u, 1u);
}

static void advance_goto(void) {
    if (!s_goto_active) {
        return;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        enter_fault(OS_ERR_LIMIT_TRIGGERED);
        return;
    }
    int32_t p0 = os_hal_motor_get_position(0u);
    int32_t p1 = os_hal_motor_get_position(1u);
    if (p0 == s_goto_target_ra_steps && p1 == s_goto_target_dec_steps) {
        finish_goto();
        return;
    }
    if (p0 < s_goto_target_ra_steps) {
        int32_t diff = s_goto_target_ra_steps - p0;
        p0 += diff > OS_GOTO_STEP_INCREMENT ? OS_GOTO_STEP_INCREMENT : diff;
    } else if (p0 > s_goto_target_ra_steps) {
        int32_t diff = p0 - s_goto_target_ra_steps;
        p0 -= diff > OS_GOTO_STEP_INCREMENT ? OS_GOTO_STEP_INCREMENT : diff;
    }
    if (p1 < s_goto_target_dec_steps) {
        int32_t diff = s_goto_target_dec_steps - p1;
        p1 += diff > OS_GOTO_STEP_INCREMENT ? OS_GOTO_STEP_INCREMENT : diff;
    } else if (p1 > s_goto_target_dec_steps) {
        int32_t diff = p1 - s_goto_target_dec_steps;
        p1 -= diff > OS_GOTO_STEP_INCREMENT ? OS_GOTO_STEP_INCREMENT : diff;
    }
    s_motor_pos[0] = p0;
    s_motor_pos[1] = p1;
}

static void finish_park(void) {
    s_parking_active = false;
    s_motion_active = false;
    s_telescope_state = OS_STATE_PARKED;
    s_tracking_enabled = false;
    stop_all_axis_frequencies();
    disable_all_motors();
    (void)os_hal_buzzer_beep(50u, 1u);
}

static int32_t s_park_target_ra_steps = 0;
static int32_t s_park_target_dec_steps = 0;

static void park_set_motion_outputs(void) {
    int32_t p0 = os_hal_motor_get_position(0u);
    int32_t p1 = os_hal_motor_get_position(1u);
    bool fwd0 = s_park_target_ra_steps >= p0;
    bool fwd1 = s_park_target_dec_steps >= p1;
    (void)os_hal_motor_set_direction(0u, fwd0);
    (void)os_hal_motor_set_direction(1u, fwd1);
    uint32_t speed = (uint32_t)((double)OS_GOTO_SPEED_MAX_DEG_PER_SEC * 3600.0);
    (void)os_hal_motor_set_frequency(0u, speed);
    (void)os_hal_motor_set_frequency(1u, speed);
    (void)os_hal_motor_enable(0u, true);
    (void)os_hal_motor_enable(1u, true);
}

static void advance_park(void) {
    if (!s_parking_active) {
        return;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        enter_fault(OS_ERR_LIMIT_TRIGGERED);
        return;
    }
    int32_t p0 = os_hal_motor_get_position(0u);
    int32_t p1 = os_hal_motor_get_position(1u);
    if (p0 == s_park_target_ra_steps && p1 == s_park_target_dec_steps) {
        finish_park();
        return;
    }
    if (p0 < s_park_target_ra_steps) {
        int32_t diff = s_park_target_ra_steps - p0;
        p0 += diff > OS_GOTO_STEP_INCREMENT ? OS_GOTO_STEP_INCREMENT : diff;
    } else if (p0 > s_park_target_ra_steps) {
        int32_t diff = p0 - s_park_target_ra_steps;
        p0 -= diff > OS_GOTO_STEP_INCREMENT ? OS_GOTO_STEP_INCREMENT : diff;
    }
    if (p1 < s_park_target_dec_steps) {
        int32_t diff = s_park_target_dec_steps - p1;
        p1 += diff > OS_GOTO_STEP_INCREMENT ? OS_GOTO_STEP_INCREMENT : diff;
    } else if (p1 > s_park_target_dec_steps) {
        int32_t diff = p1 - s_park_target_dec_steps;
        p1 -= diff > OS_GOTO_STEP_INCREMENT ? OS_GOTO_STEP_INCREMENT : diff;
    }
    s_motor_pos[0] = p0;
    s_motor_pos[1] = p1;
}

static void advance_guide(void) {
    if (!s_guide.active) {
        return;
    }
    if (s_guide.duration_ms > OS_LOOP_TICK_MS) {
        s_guide.duration_ms -= OS_LOOP_TICK_MS;
    } else {
        s_guide.duration_ms = 0u;
        s_guide.active = false;
        update_tracking_frequency();
    }
}

static double parse_ra_to_arcsec(float ra_hours) {
    return (double)ra_hours * 15.0 * 3600.0;
}

static double parse_dec_to_arcsec(float dec_degrees) {
    return (double)dec_degrees * 3600.0;
}

static void calib_to_motor_steps(double ra_arcsec, double dec_arcsec, int32_t *ra_steps, int32_t *dec_steps) {
    if (s_calibration_valid) {
        double m0 = s_cal_d[0] * ra_arcsec + s_cal_d[1] * dec_arcsec + s_cal_off[0];
        double m1 = s_cal_d[2] * ra_arcsec + s_cal_d[3] * dec_arcsec + s_cal_off[1];
        *ra_steps = (int32_t)m0;
        *dec_steps = (int32_t)m1;
    } else {
        *ra_steps = (int32_t)ra_arcsec;
        *dec_steps = (int32_t)dec_arcsec;
    }
}

static bool qr_solve_ols(const double *A, const double *b, double *x, int m, int n) {
    double q[OS_CALIBRATION_MAX_STARS][3] = {{0.0}};
    double r[3][3] = {{0.0}};
    double scale = 1.0;
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            double v = fabs(A[i * n + j]);
            if (v > scale) {
                scale = v;
            }
        }
    }
    for (int k = 0; k < n; k++) {
        for (int i = 0; i < m; i++) {
            q[i][k] = A[i * n + k];
        }
        for (int j = 0; j < k; j++) {
            double s = 0.0;
            for (int i = 0; i < m; i++) {
                s += q[i][j] * q[i][k];
            }
            r[j][k] = s;
            for (int i = 0; i < m; i++) {
                q[i][k] -= s * q[i][j];
            }
        }
        double norm = 0.0;
        for (int i = 0; i < m; i++) {
            norm += q[i][k] * q[i][k];
        }
        norm = sqrt(norm);
        if (norm <= 1.0e-9 * scale) {
            return false;
        }
        r[k][k] = norm;
        for (int i = 0; i < m; i++) {
            q[i][k] /= norm;
        }
    }
    double y[3] = {0.0, 0.0, 0.0};
    for (int j = 0; j < n; j++) {
        double s = 0.0;
        for (int i = 0; i < m; i++) {
            s += q[i][j] * b[i];
        }
        y[j] = s;
    }
    for (int i = n - 1; i >= 0; i--) {
        double s = y[i];
        for (int j = i + 1; j < n; j++) {
            s -= r[i][j] * x[j];
        }
        if (fabs(r[i][i]) <= 1.0e-12 * scale) {
            return false;
        }
        x[i] = s / r[i][i];
    }
    return true;
}

static bool fit_calibration(double m[4], double off[2]) {
    if (s_align_mode == OS_ALIGN_1STAR) {
        m[0] = 1.0; m[1] = 0.0; m[2] = 0.0; m[3] = 1.0;
        off[0] = s_align_samples[0].motor_ra - s_align_samples[0].ra_arcsec;
        off[1] = s_align_samples[0].motor_dec - s_align_samples[0].dec_arcsec;
        return true;
    }
    double A[OS_CALIBRATION_MAX_STARS * 3];
    double b[OS_CALIBRATION_MAX_STARS];
    double x[3] = {0.0, 0.0, 0.0};
    int n = s_align_count;
    if (s_align_mode == OS_ALIGN_2STAR) {
        memset(A, 0, sizeof(A));
        for (int i = 0; i < n; i++) {
            A[i * 2 + 0] = s_align_samples[i].ra_arcsec;
            A[i * 2 + 1] = 1.0;
            b[i] = s_align_samples[i].motor_ra;
        }
        if (!qr_solve_ols(A, b, x, n, 2)) {
            return false;
        }
        m[0] = x[0]; m[2] = 0.0; off[0] = x[1];
        memset(A, 0, sizeof(A));
        for (int i = 0; i < n; i++) {
            A[i * 2 + 0] = s_align_samples[i].dec_arcsec;
            A[i * 2 + 1] = 1.0;
            b[i] = s_align_samples[i].motor_dec;
        }
        memset(x, 0, sizeof(x));
        if (!qr_solve_ols(A, b, x, n, 2)) {
            return false;
        }
        m[3] = x[0]; m[1] = 0.0; off[1] = x[1];
        return true;
    }
    memset(A, 0, sizeof(A));
    for (int i = 0; i < n; i++) {
        A[i * 3 + 0] = s_align_samples[i].ra_arcsec;
        A[i * 3 + 1] = s_align_samples[i].dec_arcsec;
        A[i * 3 + 2] = 1.0;
        b[i] = s_align_samples[i].motor_ra;
    }
    if (!qr_solve_ols(A, b, x, n, 3)) {
        return false;
    }
    m[0] = x[0]; m[1] = x[1]; off[0] = x[2];
    memset(x, 0, sizeof(x));
    for (int i = 0; i < n; i++) {
        b[i] = s_align_samples[i].motor_dec;
    }
    if (!qr_solve_ols(A, b, x, n, 3)) {
        return false;
    }
    m[2] = x[0]; m[3] = x[1]; off[1] = x[2];
    return true;
}

static double compute_residual(const double m[4], const double off[2]) {
    double sum = 0.0;
    int n = s_align_count;
    for (int i = 0; i < n; i++) {
        double pra;
        double pdec;
        if (s_align_mode == OS_ALIGN_1STAR) {
            pra = s_align_samples[i].ra_arcsec + off[0];
            pdec = s_align_samples[i].dec_arcsec + off[1];
        } else if (s_align_mode == OS_ALIGN_2STAR) {
            pra = m[0] * s_align_samples[i].ra_arcsec + off[0];
            pdec = m[3] * s_align_samples[i].dec_arcsec + off[1];
        } else {
            pra = m[0] * s_align_samples[i].ra_arcsec + m[1] * s_align_samples[i].dec_arcsec + off[0];
            pdec = m[2] * s_align_samples[i].ra_arcsec + m[3] * s_align_samples[i].dec_arcsec + off[1];
        }
        double dra = pra - s_align_samples[i].motor_ra;
        double ddec = pdec - s_align_samples[i].motor_dec;
        sum += dra * dra + ddec * ddec;
    }
    return sqrt(sum / (double)n);
}

static int minimum_stars_for_mode(os_align_mode_t mode) {
    if (mode == OS_ALIGN_1STAR) {
        return OS_CALIBRATION_MIN_STARS;
    }
    if (mode == OS_ALIGN_2STAR) {
        return 2;
    }
    return OS_CALIBRATION_RECOMMENDED;
}

static void update_time_source(void) {
    os_site_info_t gps;
    memset(&gps, 0, sizeof(gps));
    os_error_t err = os_hal_gps_poll(&gps);
    if (err == OS_ERR_NONE && gps.valid) {
        s_site = gps;
        s_gps_locked = true;
        (void)os_hal_rtc_set(gps.utc_epoch_seconds);
    } else {
        s_gps_locked = false;
        uint32_t utc = 0u;
        if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
            s_rtc_valid = true;
            s_site.utc_epoch_seconds = utc;
        } else {
            s_rtc_valid = false;
        }
        s_site.valid = false;
    }
}

static void process_channel_command(uint8_t channel) {
    static char frame[OS_MAX_COMMAND_LENGTH + 1u];
    static size_t frame_len = 0u;
    (void)channel;
    frame_len = 0u;
}

static void poll_communication(void) {
    for (uint8_t ch = 0u; ch < 4u; ch++) {
        if (!s_comm_initialized[ch]) {
            continue;
        }
        (void)os_hal_comm_available(ch);
    }
}

static void format_ra(float ra_hours, char *buf, size_t cap) {
    double ra = (double)ra_hours;
    int h = (int)ra;
    double rem = (ra - (double)h) * 60.0;
    int m = (int)rem;
    int sec = (int)((rem - (double)m) * 60.0);
    (void)snprintf(buf, cap, "%02d:%02d:%02d#", h, m, sec);
}

static void format_dec(float dec_degrees, char *buf, size_t cap) {
    double dec = fabs((double)dec_degrees);
    int d = (int)dec;
    double rem = (dec - (double)d) * 60.0;
    int m = (int)rem;
    int sec = (int)((rem - (double)m) * 60.0);
    char sign = dec_degrees < 0.0f ? '-' : '+';
    (void)snprintf(buf, cap, "%c%02d*%02d:%02d#", sign, d, m, sec);
}

static os_error_t dispatch_command(const char *command, size_t length,
                                   char *reply_buffer, size_t reply_buffer_size,
                                   size_t *reply_length) {
    if (cmd_eq(command, length, ":GR#")) {
        os_equatorial_coord_t coord = {0.0f, 0.0f};
        os_error_t err = os_query_coordinates(&coord);
        if (err != OS_ERR_NONE) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
            return err;
        }
        char tmp[OS_MAX_REPLY_LENGTH];
        format_ra(coord.ra_hours, tmp, sizeof(tmp));
        set_reply(reply_buffer, reply_buffer_size, reply_length, "%s", tmp);
        return OS_ERR_NONE;
    }
    if (cmd_eq(command, length, ":GD#")) {
        os_equatorial_coord_t coord = {0.0f, 0.0f};
        os_error_t err = os_query_coordinates(&coord);
        if (err != OS_ERR_NONE) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
            return err;
        }
        char tmp[OS_MAX_REPLY_LENGTH];
        format_dec(coord.dec_degrees, tmp, sizeof(tmp));
        set_reply(reply_buffer, reply_buffer_size, reply_length, "%s", tmp);
        return OS_ERR_NONE;
    }
    if (cmd_eq(command, length, ":GVP#")) {
        set_reply(reply_buffer, reply_buffer_size, reply_length, "%u.%u.%u#",
                  OS_FIRMWARE_VERSION_MAJOR, OS_FIRMWARE_VERSION_MINOR, OS_FIRMWARE_VERSION_PATCH);
        return OS_ERR_NONE;
    }
    if (cmd_eq(command, length, ":Me#")) {
        return os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
    }
    if (cmd_eq(command, length, ":Mw#")) {
        return os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
    }
    if (cmd_eq(command, length, ":Mn#")) {
        return os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
    }
    if (cmd_eq(command, length, ":Ms#")) {
        return os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
    }
    if (cmd_eq(command, length, ":Q#") || cmd_eq(command, length, ":Qe#") ||
        cmd_eq(command, length, ":Qw#") || cmd_eq(command, length, ":Qn#") ||
        cmd_eq(command, length, ":Qs#")) {
        return os_move_stop();
    }
    if (cmd_eq(command, length, ":hP#")) {
        return os_park();
    }
    if (cmd_eq(command, length, ":hO#")) {
        return os_unpark();
    }
    if (cmd_eq(command, length, ":Te#")) {
        return os_tracking_enable();
    }
    if (cmd_eq(command, length, ":Td#")) {
        return os_tracking_disable();
    }
    set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
    return OS_ERR_NOT_SUPPORTED;
}

os_error_t os_init(void) {
    s_telescope_state = OS_STATE_INITIALIZING;
    reset_runtime_flags();
    s_guide_rate_fraction = 0.5f;
    s_custom_track_factor = 1.0f;
    s_park_custom = false;
    s_park_pos.ra_hours = 0.0f;
    s_park_pos.dec_degrees = 90.0f;
    memset(&s_site, 0, sizeof(s_site));

    os_error_t err = os_hal_nvm_init();
    if (err != OS_ERR_NONE) {
        s_telescope_state = OS_STATE_FAULT;
        return err;
    }
    restore_persistent_state();

    for (uint8_t ch = 0u; ch < 4u; ch++) {
        err = os_hal_comm_init(ch);
        if (err != OS_ERR_NONE) {
            s_telescope_state = OS_STATE_FAULT;
            return err;
        }
    }
    for (uint8_t ax = 0u; ax < 2u; ax++) {
        err = os_hal_motor_init(ax);
        if (err != OS_ERR_NONE) {
            s_telescope_state = OS_STATE_FAULT;
            return OS_ERR_MOTOR_DRIVER_FAULT;
        }
    }
    (void)os_hal_limit_init();
    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    err = os_hal_timer_motor_init();
    if (err != OS_ERR_NONE) {
        s_telescope_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    update_time_source();
    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_tracking_enabled = true;
    s_telescope_state = OS_STATE_IDLE_TRACKING;
    update_tracking_frequency();
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    s_loop_tick_ms += OS_LOOP_TICK_MS;
    update_time_source();
    poll_communication();
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        if (s_motion_active) {
            enter_fault(OS_ERR_LIMIT_TRIGGERED);
        }
    }
    advance_goto();
    advance_park();
    advance_guide();
    update_tracking_frequency();
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!valid_channel(source_channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    size_t len = length;
    while (len > 0u && (command[len - 1u] == '\r' || command[len - 1u] == '\n')) {
        len--;
    }
    if (len < 2u || command[0] != OS_LX200_CMD_PREFIX || command[len - 1u] != OS_LX200_CMD_SUFFIX) {
        set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
        return OS_ERR_COMMAND_FORMAT;
    }
    os_error_t err = dispatch_command(command, len, reply_buffer, reply_buffer_size, reply_length);
    if (err != OS_ERR_NONE && reply_length != NULL && *reply_length == 0u) {
        set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
    }
    return err;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (!isfinite((double)target.ra_hours) || !isfinite((double)target.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS ||
        target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (s_motor_driver_fault[0] || s_motor_driver_fault[1]) {
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    double ra_arcsec = parse_ra_to_arcsec(target.ra_hours);
    double dec_arcsec = parse_dec_to_arcsec(target.dec_degrees);
    int32_t ra_steps = 0;
    int32_t dec_steps = 0;
    calib_to_motor_steps(ra_arcsec, dec_arcsec, &ra_steps, &dec_steps);
    int32_t p0 = os_hal_motor_get_position(0u);
    int32_t p1 = os_hal_motor_get_position(1u);
    if (p0 == ra_steps && p1 == dec_steps) {
        s_telescope_state = OS_STATE_IDLE_TRACKING;
        s_motion_active = false;
        s_goto_active = false;
        s_tracking_enabled = true;
        update_tracking_frequency();
        return OS_ERR_NONE;
    }
    s_goto_target_ra_steps = ra_steps;
    s_goto_target_dec_steps = dec_steps;
    s_goto_active = true;
    s_motion_active = true;
    s_telescope_state = OS_STATE_GOTO;
    goto_set_motion_outputs();
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (!isfinite((double)target.altitude_degrees) || !isfinite((double)target.azimuth_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (target.altitude_degrees < OS_DEC_MIN_DEG || target.altitude_degrees > OS_DEC_MAX_DEG ||
        target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (s_motor_driver_fault[0] || s_motor_driver_fault[1]) {
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    double ra_arcsec = (double)target.azimuth_degrees * 3600.0;
    double dec_arcsec = (double)target.altitude_degrees * 3600.0;
    int32_t ra_steps = 0;
    int32_t dec_steps = 0;
    calib_to_motor_steps(ra_arcsec, dec_arcsec, &ra_steps, &dec_steps);
    int32_t p0 = os_hal_motor_get_position(0u);
    int32_t p1 = os_hal_motor_get_position(1u);
    if (p0 == ra_steps && p1 == dec_steps) {
        s_telescope_state = OS_STATE_IDLE_TRACKING;
        s_motion_active = false;
        s_goto_active = false;
        s_tracking_enabled = true;
        update_tracking_frequency();
        return OS_ERR_NONE;
    }
    s_goto_target_ra_steps = ra_steps;
    s_goto_target_dec_steps = dec_steps;
    s_goto_active = true;
    s_motion_active = true;
    s_telescope_state = OS_STATE_GOTO;
    goto_set_motion_outputs();
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (!s_goto_active || s_telescope_state != OS_STATE_GOTO) {
        return OS_ERR_INVALID_STATE;
    }
    s_goto_active = false;
    s_motion_active = false;
    s_telescope_state = OS_STATE_IDLE_TRACKING;
    s_tracking_enabled = true;
    stop_all_axis_frequencies();
    update_tracking_frequency();
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (rate != OS_TRACK_RATE_SIDEREAL && rate != OS_TRACK_RATE_LUNAR &&
        rate != OS_TRACK_RATE_SOLAR && rate != OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM) {
        if (!isfinite((double)custom_factor) || custom_factor <= 0.0f) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        s_custom_track_factor = custom_factor;
    } else {
        s_custom_track_factor = 1.0f;
    }
    s_track_rate = rate;
    update_tracking_frequency();
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = s_track_rate;
    *custom_factor = s_custom_track_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    s_tracking_enabled = true;
    update_tracking_frequency();
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    s_tracking_enabled = false;
    if (!s_motion_active) {
        (void)os_hal_motor_set_frequency(0u, 0u);
    }
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (direction != OS_DIRECTION_NORTH && direction != OS_DIRECTION_SOUTH &&
        direction != OS_DIRECTION_EAST && direction != OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    bool is_dec = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);
    if (s_guide.active && s_guide.dec_priority && !is_dec) {
        return OS_ERR_NONE;
    }
    s_guide.active = true;
    s_guide.duration_ms = duration_ms;
    s_guide.rate_fraction = s_guide_rate_fraction;
    s_guide.direction_east = (direction == OS_DIRECTION_EAST);
    s_guide.direction_north = (direction == OS_DIRECTION_NORTH);
    s_guide.dec_priority = is_dec;
    uint32_t freq = (uint32_t)((double)OS_SIDEREAL_RATE_ARCSEC_PER_SEC * (1.0 + (double)s_guide.rate_fraction));
    uint8_t axis = is_dec ? 1u : 0u;
    (void)os_hal_motor_set_direction(axis, (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_EAST));
    (void)os_hal_motor_set_frequency(axis, freq);
    (void)os_hal_motor_enable(axis, true);
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (!isfinite((double)rate_fraction) || rate_fraction < OS_GUIDE_RATE_MIN ||
        rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_guide_rate_fraction = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *pulse = s_guide;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode != OS_ALIGN_1STAR && mode != OS_ALIGN_2STAR && mode != OS_ALIGN_3STAR &&
        mode != OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_align_mode = mode;
    s_align_count = 0u;
    memset(s_align_samples, 0, sizeof(s_align_samples));
    s_residual_computed = false;
    s_residual_arcsec = 0.0f;
    s_telescope_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (!isfinite((double)star_coord.ra_hours) || !isfinite((double)star_coord.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (star_coord.ra_hours < OS_RA_MIN_HOURS || star_coord.ra_hours > OS_RA_MAX_HOURS ||
        star_coord.dec_degrees < OS_DEC_MIN_DEG || star_coord.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_telescope_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_align_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }
    align_sample_t *sample = &s_align_samples[s_align_count];
    sample->ra_arcsec = parse_ra_to_arcsec(star_coord.ra_hours);
    sample->dec_arcsec = parse_dec_to_arcsec(star_coord.dec_degrees);
    sample->motor_ra = (double)motor_pos.ra_steps;
    sample->motor_dec = (double)motor_pos.dec_steps;
    s_align_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (s_telescope_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    int needed = minimum_stars_for_mode(s_align_mode);
    if ((int)s_align_count < needed) {
        return OS_ERR_INVALID_STATE;
    }
    double m[4] = {0.0, 0.0, 0.0, 0.0};
    double off[2] = {0.0, 0.0};
    if (!fit_calibration(m, off)) {
        return OS_ERR_CALIBRATION_FAILED;
    }
    double residual = compute_residual(m, off);
    if (s_align_count >= 4 && residual > OS_CALIBRATION_RESIDUAL_MAX_ARCSEC) {
        return OS_ERR_CALIBRATION_FAILED;
    }
    os_calibration_t new_cal;
    memset(&new_cal, 0, sizeof(new_cal));
    new_cal.matrix_ra_to_ra = (float)m[0];
    new_cal.matrix_ra_to_dec = (float)m[1];
    new_cal.matrix_dec_to_ra = (float)m[2];
    new_cal.matrix_dec_to_dec = (float)m[3];
    new_cal.offset_ra_arcsec = (float)off[0];
    new_cal.offset_dec_arcsec = (float)off[1];
    new_cal.valid = true;
    os_error_t err = persist_calibration_from(&new_cal);
    if (err != OS_ERR_NONE) {
        return err;
    }
    s_calibration = new_cal;
    s_cal_d[0] = m[0]; s_cal_d[1] = m[1];
    s_cal_d[2] = m[2]; s_cal_d[3] = m[3];
    s_cal_off[0] = off[0]; s_cal_off[1] = off[1];
    s_calibration_valid = true;
    s_residual_computed = true;
    s_residual_arcsec = (float)residual;
    s_telescope_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_residual_computed) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = s_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    if (s_telescope_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    s_align_count = 0u;
    memset(s_align_samples, 0, sizeof(s_align_samples));
    s_residual_computed = false;
    s_residual_arcsec = 0.0f;
    s_telescope_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (s_telescope_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (s_motor_driver_fault[0] || s_motor_driver_fault[1]) {
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    os_equatorial_coord_t target = s_park_custom ? s_park_pos : (os_equatorial_coord_t){0.0f, 90.0f};
    double ra_arcsec = parse_ra_to_arcsec(target.ra_hours);
    double dec_arcsec = parse_dec_to_arcsec(target.dec_degrees);
    int32_t ra_steps = 0;
    int32_t dec_steps = 0;
    calib_to_motor_steps(ra_arcsec, dec_arcsec, &ra_steps, &dec_steps);
    if (os_hal_motor_get_position(0u) == ra_steps && os_hal_motor_get_position(1u) == dec_steps) {
        finish_park();
        return OS_ERR_NONE;
    }
    s_park_target_ra_steps = ra_steps;
    s_park_target_dec_steps = dec_steps;
    s_parking_active = true;
    s_motion_active = true;
    s_telescope_state = OS_STATE_GOTO;
    park_set_motion_outputs();
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (s_telescope_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    for (uint8_t ch = 0u; ch < 4u; ch++) {
        (void)os_hal_comm_init(ch);
    }
    if (s_gps_locked) {
        (void)os_hal_rtc_set(s_site.utc_epoch_seconds);
    }
    s_parking_active = false;
    s_motion_active = false;
    s_tracking_enabled = true;
    s_telescope_state = OS_STATE_IDLE_TRACKING;
    update_tracking_frequency();
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!isfinite((double)park_pos.ra_hours) || !isfinite((double)park_pos.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (park_pos.ra_hours < OS_RA_MIN_HOURS || park_pos.ra_hours > OS_RA_MAX_HOURS ||
        park_pos.dec_degrees < OS_DEC_MIN_DEG || park_pos.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_park_pos = park_pos;
    s_park_custom = true;
    os_error_t err = persist_park_config();
    return err;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction != OS_DIRECTION_NORTH && direction != OS_DIRECTION_SOUTH &&
        direction != OS_DIRECTION_EAST && direction != OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (speed != OS_SPEED_SLOW && speed != OS_SPEED_MEDIUM &&
        speed != OS_SPEED_FAST && speed != OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    uint8_t axis = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH) ? 1u : 0u;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (s_motor_driver_fault[0] || s_motor_driver_fault[1]) {
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    bool forward = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_EAST);
    uint32_t freq = OS_MANUAL_SPEED_MEDIUM_HZ;
    if (speed == OS_SPEED_SLOW) {
        freq = OS_MANUAL_SPEED_SLOW_HZ;
    } else if (speed == OS_SPEED_FAST) {
        freq = OS_MANUAL_SPEED_FAST_HZ;
    } else if (speed == OS_SPEED_CUSTOM) {
        freq = (uint32_t)s_manual_custom_speed_arcsec_per_sec;
    }
    s_manual_motion_active = true;
    s_motion_active = true;
    s_telescope_state = OS_STATE_MANUAL_MOTION;
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, freq);
    (void)os_hal_motor_enable(axis, true);
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (!s_manual_motion_active || s_telescope_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }
    s_manual_motion_active = false;
    s_motion_active = false;
    s_telescope_state = OS_STATE_IDLE_TRACKING;
    s_tracking_enabled = true;
    stop_all_axis_frequencies();
    update_tracking_frequency();
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (!isfinite((double)arcsec_per_sec) || arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_manual_custom_speed_arcsec_per_sec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = s_telescope_state;
    return OS_ERR_NONE;
}

static double normalize_ra_hours(double ra_hours) {
    double ra = ra_hours;
    while (ra < 0.0) {
        ra += 24.0;
    }
    while (ra >= 24.0) {
        ra -= 24.0;
    }
    return ra;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int32_t p0 = os_hal_motor_get_position(0u);
    int32_t p1 = os_hal_motor_get_position(1u);
    double ra_arcsec = (double)p0;
    double dec_arcsec = (double)p1;
    if (s_calibration_valid) {
        double det = s_cal_d[0] * s_cal_d[3] - s_cal_d[1] * s_cal_d[2];
        if (fabs(det) > 1.0e-12) {
            double a = (double)p0 - s_cal_off[0];
            double b = (double)p1 - s_cal_off[1];
            ra_arcsec = (s_cal_d[3] * a - s_cal_d[1] * b) / det;
            dec_arcsec = (-s_cal_d[2] * a + s_cal_d[0] * b) / det;
        }
    }
    double ra_hours = normalize_ra_hours(ra_arcsec / (15.0 * 3600.0));
    double dec_deg = dec_arcsec / 3600.0;
    if (dec_deg < OS_DEC_MIN_DEG) {
        dec_deg = OS_DEC_MIN_DEG;
    }
    if (dec_deg > OS_DEC_MAX_DEG) {
        dec_deg = OS_DEC_MAX_DEG;
    }
    coord->ra_hours = (float)ra_hours;
    coord->dec_degrees = (float)dec_deg;
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = s_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = os_hal_motor_get_position(0u);
    pos->dec_steps = os_hal_motor_get_position(1u);
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
    *moving = s_motion_active;
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
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(&s_pec_table, table, sizeof(s_pec_table));
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
    if (!isfinite((double)worm_phase_deg) || worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int index = 0;
    if (worm_phase_deg >= 360.0f) {
        index = 0;
    } else {
        index = (int)worm_phase_deg;
    }
    s_pec_table.corrections[index] = error_arcsec;
    s_pec_table.valid = true;
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *calib = s_calibration;
    calib->valid = s_calibration_valid;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    s_calibration_valid = false;
    memset(&s_calibration, 0, sizeof(s_calibration));
    s_cal_d[0] = 1.0; s_cal_d[1] = 0.0;
    s_cal_d[2] = 0.0; s_cal_d[3] = 1.0;
    s_cal_off[0] = 0.0; s_cal_off[1] = 0.0;
    s_calibration.valid = false;
    os_error_t err = persist_calibration_from(&s_calibration);
    return err;
}

os_error_t os_hal_motor_init(uint8_t axis) {
    if (!valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_motor_initialized[axis] = true;
    s_motor_enabled[axis] = false;
    s_motor_freq[axis] = 0u;
    s_motor_direction[axis] = false;
    s_motor_driver_fault[axis] = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (!valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_motor_driver_fault[axis]) {
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    s_motor_freq[axis] = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (!valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_motor_direction[axis] = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (!valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_motor_enabled[axis] = enable;
    if (!enable) {
        s_motor_freq[axis] = 0u;
    }
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (!valid_axis(axis)) {
        return 0;
    }
    return s_motor_pos[axis];
}

os_error_t os_hal_gps_init(void) {
    s_gps_initialized = true;
    s_gps_locked = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    site->valid = false;
    return OS_ERR_GPS_NO_SIGNAL;
}

os_error_t os_hal_rtc_init(void) {
    s_rtc_initialized = true;
    s_rtc_valid = true;
    s_rtc_epoch_seconds = 1700000000u;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_rtc_valid) {
        return OS_ERR_TIMEOUT;
    }
    *utc_epoch_seconds = s_rtc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    s_rtc_epoch_seconds = utc_epoch_seconds;
    s_rtc_valid = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    s_limit_triggered[0] = false;
    s_limit_triggered[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (!valid_axis(axis)) {
        return true;
    }
    return s_limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void) {
    if (!s_nvm_initialized) {
        memset(s_nvm_memory, 0, sizeof(s_nvm_memory));
        s_nvm_initialized = true;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (data == NULL && length > 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((uint32_t)offset + (uint32_t)length > OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length > 0u) {
        memcpy(data, &s_nvm_memory[offset], length);
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (data == NULL && length > 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((uint32_t)offset + (uint32_t)length > OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length > 0u) {
        memcpy(&s_nvm_memory[offset], data, length);
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (!valid_channel(channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_comm_initialized[channel] = true;
    s_comm_tx_len[channel] = 0u;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if (!valid_channel(channel)) {
        return -1;
    }
    return 0;
}

char os_hal_comm_read(uint8_t channel) {
    if (!valid_channel(channel) || os_hal_comm_available(channel) <= 0) {
        return '\0';
    }
    return '\0';
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (!valid_channel(channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (data == NULL && length > 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_comm_tx_len[channel] + length > sizeof(s_comm_tx[channel])) {
        return OS_ERR_TIMEOUT;
    }
    if (length > 0u) {
        memcpy(&s_comm_tx[channel][s_comm_tx_len[channel]], data, length);
        s_comm_tx_len[channel] += length;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) {
    s_buzzer_duration_ms = duration_ms;
    s_buzzer_count = count;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void) {
    s_motor_timer_initialized = true;
    return OS_ERR_NONE;
}
#include "6_generated_code.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

#define OS_NVM_CAL_MAGIC 0x4F4E4341u
#define OS_NVM_CONFIG_MAGIC 0x4F4E4346u
#define OS_NVM_PEC_MAGIC 0x4F4E5043u
#define OS_NVM_POSITION_MAGIC 0x4F4E504Fu
#define OS_NVM_CAL_OFFSET 0u
#define OS_NVM_CONFIG_OFFSET OS_NVM_CALIBRATION_SIZE_BYTES
#define OS_NVM_PEC_OFFSET (OS_NVM_CONFIG_OFFSET + OS_NVM_CONFIG_SIZE_BYTES)
#define OS_NVM_POSITION_OFFSET (OS_NVM_PEC_OFFSET + sizeof(os_nvm_pec_record_t))
#define OS_NVM_TOTAL_SIZE (OS_NVM_POSITION_OFFSET + sizeof(os_nvm_position_record_t))

#define OS_STEPS_PER_DEGREE_DEFAULT 1000.0f
#define OS_MAX_MOTOR_FREQ_HZ 5000u
#define OS_GOTO_START_FREQ_HZ 60u
#define OS_GOTO_ACCEL_FREQ_PER_LOOP 60u
#define OS_GOTO_ACCEL_LOOPS 12u
#define OS_GOTO_MAX_FREQ_HZ 800u
#define OS_GOTO_DECEL_STEPS 160
#define OS_GOTO_APPROACH_FREQ_HZ 40u
#define OS_ADVANCE_STEPS_PER_LOOP_PROFILE 16
#define OS_MANUAL_SLOW_HZ 100u
#define OS_MANUAL_MEDIUM_HZ 400u
#define OS_MANUAL_FAST_HZ 1000u
#define OS_MANUAL_CUSTOM_MAX_ARCSEC_PER_SEC 10000.0f
#define OS_CALIBRATION_MAX_RESIDUAL_ARCSEC 300.0
#define OS_COMM_RX_CAP 256
#define OS_COMM_TX_CAP 1024
#define OS_GPS_LOCK_TIMEOUT_LOOPS 5u

typedef struct {
    uint32_t magic;
    os_calibration_t calib;
    float residual_arcsec;
} os_nvm_cal_record_t;

typedef struct {
    uint32_t magic;
    uint8_t park_valid;
    os_equatorial_coord_t park_position;
    uint8_t site_valid;
    float site_latitude_degrees;
    float site_longitude_degrees;
    float site_elevation_metres;
} os_nvm_config_record_t;

typedef struct {
    uint32_t magic;
    uint8_t valid;
    int16_t corrections[OS_PEC_TABLE_SIZE];
} os_nvm_pec_record_t;

typedef struct {
    uint32_t magic;
    int32_t ra_steps;
    int32_t dec_steps;
} os_nvm_position_record_t;

typedef struct {
    char rx[OS_COMM_RX_CAP];
    uint16_t head;
    uint16_t tail;
    bool init;
    char tx[OS_COMM_TX_CAP];
    uint16_t tx_len;
} os_host_comm_t;

static os_state_t s_system_state = OS_STATE_INITIALIZING;
static bool s_goto_active = false;
static bool s_goto_abort_requested = false;
static bool s_manual_active = false;
static bool s_parking_active = false;
static bool s_parked = false;
static bool s_moving = false;
static bool s_tracking_enabled = false;
static os_track_rate_t s_track_rate = OS_TRACK_RATE_SIDEREAL;
static float s_track_custom_factor = 1.0f;
static os_guide_pulse_t s_guide_pulse;
static float s_guide_rate_fraction = 0.5f;
static float s_custom_manual_arcsec_per_sec = 100.0f;
static int32_t s_goto_target_steps[2] = {0, 0};
static int32_t s_park_target_steps[2] = {0, 0};
static os_equatorial_coord_t s_park_position;
static bool s_park_position_valid = false;
static uint8_t s_manual_axis = 0u;
static bool s_manual_forward = false;
static uint32_t s_motion_elapsed_loops = 0u;

static bool s_pending_ra_valid = false;
static bool s_pending_dec_valid = false;
static os_equatorial_coord_t s_pending_eq_target;

static os_align_mode_t s_align_mode = OS_ALIGN_1STAR;
static bool s_align_active = false;
static uint8_t s_align_star_count = 0;
static os_equatorial_coord_t s_align_stars[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t s_align_motors[OS_CALIBRATION_MAX_STARS];
static bool s_align_residual_valid = false;
static float s_align_residual_arcsec = 0.0f;
static os_calibration_t s_calibration;

static os_site_info_t s_site_info;
static bool s_site_info_valid = false;
static bool s_gps_locked = false;
static uint32_t s_gps_loss_ticks = 0u;
static bool s_rtc_valid = false;
static bool s_rtc_initialized = false;
static uint32_t s_rtc_epoch = 0u;
static bool s_gps_initialized = false;
static os_site_info_t s_preset_site;
static bool s_preset_site_valid = false;

static bool s_pec_enabled = false;
static os_pec_table_t s_pec_table;
static double s_pec_phase_deg = 0.0;

static bool s_motor_initialized[2] = {false, false};
static bool s_motor_enabled[2] = {false, false};
static bool s_motor_forward[2] = {false, false};
static uint32_t s_motor_freq[2] = {0u, 0u};
static int32_t s_motor_pos[2] = {0, 0};
static int32_t s_last_saved_motor_pos[2] = {0, 0};
static bool s_limit_triggered[2] = {false, false};
static bool s_limit_initialized = false;
static bool s_motor_timer_initialized = false;
static bool s_nvm_initialized = false;
static uint8_t s_nvm_memory[OS_NVM_TOTAL_SIZE] = {0};

static os_host_comm_t s_comm[4];
static char s_command_rx[4][OS_MAX_COMMAND_LENGTH + 1];
static size_t s_command_len[4] = {0u, 0u, 0u, 0u};
static uint16_t s_buzzer_last_duration_ms = 0u;
static uint8_t s_buzzer_last_count = 0u;

static void copy_reply_chars(char *dest, size_t cap, size_t *len, const char *src, size_t src_len) {
    if (!dest || !len || cap == 0u) {
        if (len) *len = 0u;
        return;
    }
    if (src_len >= cap) src_len = cap - 1u;
    for (size_t i = 0u; i < src_len; ++i) dest[i] = src[i];
    dest[src_len] = '\0';
    *len = src_len;
}

static void append_char(char *dest, size_t cap, size_t *len, char c) {
    if (!dest || !len || cap == 0u) return;
    if (*len + 1u >= cap) return;
    dest[*len] = c;
    (*len)++;
    dest[*len] = '\0';
}

static void append_two_digits(char *dest, size_t cap, size_t *len, int value) {
    append_char(dest, cap, len, (char)(48 + ((value / 10) % 10)));
    append_char(dest, cap, len, (char)(48 + (value % 10)));
}

static void append_dec_u32(char *dest, size_t cap, size_t *len, uint32_t value) {
    char tmp[10];
    int n = 0;
    if (value == 0u) {
        tmp[n++] = '0';
    } else {
        while (value != 0u && n < 10) {
            tmp[n++] = (char)((value % 10u) + (uint32_t)'0');
            value /= 10u;
        }
    }
    while (n > 0) append_char(dest, cap, len, tmp[--n]);
}

static void append_dec_i32(char *dest, size_t cap, size_t *len, int32_t value) {
    if (value < 0) {
        append_char(dest, cap, len, '-');
        uint32_t uv = (uint32_t)(-(int64_t)value);
        append_dec_u32(dest, cap, len, uv);
    } else {
        append_dec_u32(dest, cap, len, (uint32_t)value);
    }
}

static void append_ra_sexagesimal(char *dest, size_t cap, size_t *len, float ra_hours) {
    if (ra_hours < OS_RA_MIN_HOURS) ra_hours = OS_RA_MIN_HOURS;
    if (ra_hours >= OS_RA_MAX_HOURS) ra_hours = OS_RA_MAX_HOURS;
    long total_sec = (long)((double)ra_hours * 3600.0 + 0.5);
    int h = (int)(total_sec / 3600L);
    int m = (int)((total_sec % 3600L) / 60L);
    int s = (int)(total_sec % 60L);
    append_two_digits(dest, cap, len, h);
    append_char(dest, cap, len, ':');
    append_two_digits(dest, cap, len, m);
    append_char(dest, cap, len, ':');
    append_two_digits(dest, cap, len, s);
}

static void append_dec_sexagesimal(char *dest, size_t cap, size_t *len, float dec_degrees) {
    if (dec_degrees < OS_DEC_MIN_DEG) dec_degrees = OS_DEC_MIN_DEG;
    if (dec_degrees > OS_DEC_MAX_DEG) dec_degrees = OS_DEC_MAX_DEG;
    append_char(dest, cap, len, dec_degrees < 0.0f ? '-' : '+');
    double absolute = fabs((double)dec_degrees);
    if (absolute > 90.0) absolute = 90.0;
    long total_sec = (long)(absolute * 3600.0 + 0.5);
    int d = (int)(total_sec / 3600L);
    int m = (int)((total_sec % 3600L) / 60L);
    int s = (int)(total_sec % 60L);
    append_two_digits(dest, cap, len, d);
    append_char(dest, cap, len, 42);
    append_two_digits(dest, cap, len, m);
    append_char(dest, cap, len, 39);
    append_two_digits(dest, cap, len, s);
    append_char(dest, cap, len, 34);
}

static bool valid_eq_coord(float ra, float dec) {
    return (ra >= OS_RA_MIN_HOURS && ra <= OS_RA_MAX_HOURS &&
            dec >= OS_DEC_MIN_DEG && dec <= OS_DEC_MAX_DEG);
}

static uint8_t align_min_stars(os_align_mode_t mode) {
    if (mode == OS_ALIGN_1STAR) return 1;
    if (mode == OS_ALIGN_2STAR) return 2;
    return 3;
}

static bool internal_parse_angle_float(const char *text, size_t len, float *out) {
    if (!text || !out || len == 0u || len >= 31u) return false;
    char token[32];
    memcpy(token, text, len);
    token[len] = '\0';
    char *s = token;
    while (*s == ' ') s++;
    size_t slen = strlen(s);
    while (slen > 0u && (s[slen - 1u] == ' ' || s[slen - 1u] == '\t')) s[--slen] = '\0';
    if (slen == 0u) return false;
    char *colon = strchr(s, ':');
    if (colon != NULL) {
        double sign = 1.0;
        char *p = s;
        if (*p == '+' || *p == '-') {
            if (*p == '-') sign = -1.0;
            p++;
        }
        double total = 0.0;
        int comp = 0;
        while (*p) {
            char *endptr = NULL;
            double v = strtod(p, &endptr);
            if (endptr == p) return false;
            if (comp == 0) total += fabs(v);
            else total += fabs(v) / pow(60.0, (double)comp);
            comp++;
            if (comp > 3) return false;
            p = endptr;
            while (*p == ' ') p++;
            if (*p == ':') {
                p++;
                while (*p == ' ') p++;
                continue;
            }
            if (*p == '\0') break;
            return false;
        }
        *out = (float)(sign * total);
        return true;
    }
    char *endptr = NULL;
    float v = strtof(s, &endptr);
    if (endptr == s || *endptr != '\0') return false;
    *out = v;
    return true;
}

static bool ls_solve(const double *A, int m, int n, const double *b, double *x) {
    double a[OS_CALIBRATION_MAX_STARS * 3];
    double bb[OS_CALIBRATION_MAX_STARS];
    if (m > OS_CALIBRATION_MAX_STARS || n > 3 || m < n) return false;
    for (int i = 0; i < m; ++i) {
        bb[i] = b[i];
        for (int j = 0; j < n; ++j) a[i * n + j] = A[i * n + j];
    }
    for (int k = 0; k < n; ++k) {
        double norm = 0.0;
        for (int i = k; i < m; ++i) norm += a[i * n + k] * a[i * n + k];
        norm = sqrt(norm);
        if (norm < 1e-12) return false;
        double alpha = (a[k * n + k] >= 0.0) ? -norm : norm;
        double v[OS_CALIBRATION_MAX_STARS];
        for (int i = 0; i < k; ++i) v[i] = 0.0;
        v[k] = a[k * n + k] - alpha;
        for (int i = k + 1; i < m; ++i) v[i] = a[i * n + k];
        double denom = alpha * alpha - alpha * a[k * n + k];
        if (fabs(denom) < 1e-15) return false;
        double beta = 1.0 / denom;
        for (int j = k; j < n; ++j) {
            double dot = 0.0;
            for (int i = k; i < m; ++i) dot += v[i] * a[i * n + j];
            double tau = beta * dot;
            for (int i = k; i < m; ++i) a[i * n + j] -= tau * v[i];
        }
        double dotb = 0.0;
        for (int i = k; i < m; ++i) dotb += v[i] * bb[i];
        double taub = beta * dotb;
        for (int i = k; i < m; ++i) bb[i] -= taub * v[i];
    }
    for (int i = n - 1; i >= 0; --i) {
        double sum = bb[i];
        for (int j = i + 1; j < n; ++j) sum -= a[i * n + j] * x[j];
        if (fabs(a[i * n + i]) < 1e-12) return false;
        x[i] = sum / a[i * n + i];
    }
    return true;
}

static double internal_det3(const double *a) {
    return a[0] * (a[4] * a[8] - a[5] * a[7]) -
           a[1] * (a[3] * a[8] - a[5] * a[6]) +
           a[2] * (a[3] * a[7] - a[4] * a[6]);
}

static int32_t internal_double_to_i32_saturate(double v) {
    if (v >= (double)INT32_MAX) return INT32_MAX;
    if (v <= (double)INT32_MIN) return INT32_MIN;
    return (int32_t)v;
}

static bool internal_invert_calibration(double ra_arcsec, double dec_arcsec, double *ra_steps, double *dec_steps) {
    if (!s_calibration.valid) return false;
    double a = (double)s_calibration.matrix_ra_to_ra;
    double b = (double)s_calibration.matrix_dec_to_ra;
    double c = (double)s_calibration.matrix_ra_to_dec;
    double d = (double)s_calibration.matrix_dec_to_dec;
    double det = a * d - b * c;
    if (fabs(det) < 1e-9) return false;
    double rhs_ra = ra_arcsec - (double)s_calibration.offset_ra_arcsec;
    double rhs_dec = dec_arcsec - (double)s_calibration.offset_dec_arcsec;
    *ra_steps = (d * rhs_ra - b * rhs_dec) / det;
    *dec_steps = (a * rhs_dec - c * rhs_ra) / det;
    return true;
}

static void internal_eq_to_steps(os_equatorial_coord_t target, int32_t steps[2]) {
    double ra_as = (double)target.ra_hours * 15.0 * 3600.0;
    double dec_as = (double)target.dec_degrees * 3600.0;
    double rs = 0.0;
    double ds = 0.0;
    if (internal_invert_calibration(ra_as, dec_as, &rs, &ds)) {
        steps[0] = internal_double_to_i32_saturate(rs);
        steps[1] = internal_double_to_i32_saturate(ds);
    } else {
        steps[0] = internal_double_to_i32_saturate((double)target.ra_hours * 15.0 * (double)OS_STEPS_PER_DEGREE_DEFAULT);
        steps[1] = internal_double_to_i32_saturate((double)target.dec_degrees * (double)OS_STEPS_PER_DEGREE_DEFAULT);
    }
}

static void internal_horiz_to_steps(os_horizontal_coord_t target, int32_t steps[2]) {
    steps[0] = internal_double_to_i32_saturate((double)target.azimuth_degrees * (double)OS_STEPS_PER_DEGREE_DEFAULT);
    steps[1] = internal_double_to_i32_saturate((double)target.altitude_degrees * (double)OS_STEPS_PER_DEGREE_DEFAULT);
}

static void internal_load_calibration_defaults(void) {
    memset(&s_calibration, 0, sizeof(s_calibration));
    s_calibration.valid = false;
    s_align_residual_valid = false;
    s_align_residual_arcsec = 0.0f;
}

static os_error_t internal_load_calibration(void) {
    os_nvm_cal_record_t rec;
    memset(&rec, 0, sizeof(rec));
    os_error_t err = os_hal_nvm_read((uint16_t)OS_NVM_CAL_OFFSET, (uint8_t *)&rec, (uint16_t)sizeof(rec));
    if (err != OS_ERR_NONE) return OS_ERR_NVM_FAULT;
    if (rec.magic != OS_NVM_CAL_MAGIC) {
        internal_load_calibration_defaults();
        return OS_ERR_NONE;
    }
    s_calibration = rec.calib;
    s_align_residual_arcsec = rec.residual_arcsec;
    s_align_residual_valid = true;
    return OS_ERR_NONE;
}

static os_error_t internal_save_calibration(void) {
    os_nvm_cal_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = OS_NVM_CAL_MAGIC;
    rec.calib = s_calibration;
    rec.residual_arcsec = s_align_residual_arcsec;
    return os_hal_nvm_write((uint16_t)OS_NVM_CAL_OFFSET, (const uint8_t *)&rec, (uint16_t)sizeof(rec));
}

static void internal_clear_calibration_nvm(void) {
    os_nvm_cal_record_t rec;
    memset(&rec, 0, sizeof(rec));
    (void)os_hal_nvm_write((uint16_t)OS_NVM_CAL_OFFSET, (const uint8_t *)&rec, (uint16_t)sizeof(rec));
}

static os_error_t internal_load_config(void) {
    os_nvm_config_record_t rec;
    memset(&rec, 0, sizeof(rec));
    os_error_t err = os_hal_nvm_read((uint16_t)OS_NVM_CONFIG_OFFSET, (uint8_t *)&rec, (uint16_t)sizeof(rec));
    if (err != OS_ERR_NONE) return OS_ERR_NVM_FAULT;
    if (rec.magic != OS_NVM_CONFIG_MAGIC) {
        s_park_position_valid = false;
        memset(&s_park_position, 0, sizeof(s_park_position));
        s_preset_site_valid = false;
        memset(&s_preset_site, 0, sizeof(s_preset_site));
        return OS_ERR_NONE;
    }
    s_park_position_valid = (rec.park_valid != 0u);
    s_park_position = rec.park_position;
    s_preset_site_valid = (rec.site_valid != 0u);
    memset(&s_preset_site, 0, sizeof(s_preset_site));
    s_preset_site.latitude_degrees = rec.site_latitude_degrees;
    s_preset_site.longitude_degrees = rec.site_longitude_degrees;
    s_preset_site.elevation_metres = rec.site_elevation_metres;
    return OS_ERR_NONE;
}

static os_error_t internal_write_config(void) {
    os_nvm_config_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = OS_NVM_CONFIG_MAGIC;
    rec.park_valid = s_park_position_valid ? 1u : 0u;
    rec.park_position = s_park_position;
    rec.site_valid = s_preset_site_valid ? 1u : 0u;
    rec.site_latitude_degrees = s_preset_site.latitude_degrees;
    rec.site_longitude_degrees = s_preset_site.longitude_degrees;
    rec.site_elevation_metres = s_preset_site.elevation_metres;
    return os_hal_nvm_write((uint16_t)OS_NVM_CONFIG_OFFSET, (const uint8_t *)&rec, (uint16_t)sizeof(rec));
}

static os_error_t internal_save_pec(void) {
    os_nvm_pec_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = OS_NVM_PEC_MAGIC;
    rec.valid = s_pec_table.valid ? 1u : 0u;
    memcpy(rec.corrections, s_pec_table.corrections, sizeof(rec.corrections));
    return os_hal_nvm_write((uint16_t)OS_NVM_PEC_OFFSET, (const uint8_t *)&rec, (uint16_t)sizeof(rec));
}

static os_error_t internal_load_pec(void) {
    os_nvm_pec_record_t rec;
    memset(&rec, 0, sizeof(rec));
    os_error_t err = os_hal_nvm_read((uint16_t)OS_NVM_PEC_OFFSET, (uint8_t *)&rec, (uint16_t)sizeof(rec));
    if (err != OS_ERR_NONE) {
        memset(&s_pec_table, 0, sizeof(s_pec_table));
        return OS_ERR_NVM_FAULT;
    }
    if (rec.magic != OS_NVM_PEC_MAGIC || rec.valid == 0u) {
        memset(&s_pec_table, 0, sizeof(s_pec_table));
        return OS_ERR_NONE;
    }
    memset(&s_pec_table, 0, sizeof(s_pec_table));
    memcpy(s_pec_table.corrections, rec.corrections, sizeof(s_pec_table.corrections));
    s_pec_table.valid = true;
    return OS_ERR_NONE;
}

static os_error_t internal_save_motor_position(void) {
    os_nvm_position_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = OS_NVM_POSITION_MAGIC;
    rec.ra_steps = s_motor_pos[0];
    rec.dec_steps = s_motor_pos[1];
    os_error_t err = os_hal_nvm_write((uint16_t)OS_NVM_POSITION_OFFSET, (const uint8_t *)&rec, (uint16_t)sizeof(rec));
    if (err == OS_ERR_NONE) {
        s_last_saved_motor_pos[0] = s_motor_pos[0];
        s_last_saved_motor_pos[1] = s_motor_pos[1];
    }
    return err;
}

static os_error_t internal_load_motor_position(void) {
    os_nvm_position_record_t rec;
    memset(&rec, 0, sizeof(rec));
    os_error_t err = os_hal_nvm_read((uint16_t)OS_NVM_POSITION_OFFSET, (uint8_t *)&rec, (uint16_t)sizeof(rec));
    if (err != OS_ERR_NONE || rec.magic != OS_NVM_POSITION_MAGIC) {
        s_motor_pos[0] = 0;
        s_motor_pos[1] = 0;
        s_last_saved_motor_pos[0] = 0;
        s_last_saved_motor_pos[1] = 0;
        return OS_ERR_NONE;
    }
    s_motor_pos[0] = rec.ra_steps;
    s_motor_pos[1] = rec.dec_steps;
    s_last_saved_motor_pos[0] = rec.ra_steps;
    s_last_saved_motor_pos[1] = rec.dec_steps;
    return OS_ERR_NONE;
}

static void internal_stop_axis(uint8_t axis) {
    (void)os_hal_motor_set_frequency(axis, 0u);
}

static uint32_t internal_tracking_base_frequency_hz(void) {
    float factor = 1.0f;
    if (s_track_rate == OS_TRACK_RATE_LUNAR) factor = OS_LUNAR_RATE_FACTOR;
    else if (s_track_rate == OS_TRACK_RATE_SOLAR) factor = OS_SOLAR_RATE_FACTOR;
    else if (s_track_rate == OS_TRACK_RATE_CUSTOM) factor = s_track_custom_factor;
    double deg_per_sec = (double)(OS_SIDEREAL_RATE_ARCSEC_PER_SEC * factor) / 3600.0;
    double freq = deg_per_sec * (double)OS_STEPS_PER_DEGREE_DEFAULT;
    if (freq < 0.0) freq = 0.0;
    if (freq > (double)OS_MAX_MOTOR_FREQ_HZ) freq = (double)OS_MAX_MOTOR_FREQ_HZ;
    return (uint32_t)freq;
}

static uint32_t internal_tracking_frequency_hz(void) {
    uint32_t freq = internal_tracking_base_frequency_hz();
    if (s_pec_enabled && s_pec_table.valid) {
        int idx = (int)s_pec_phase_deg % 360;
        if (idx < 0) idx += 360;
        int16_t corr = s_pec_table.corrections[idx];
        double corr_hz = (double)corr / 3600.0 * (double)OS_STEPS_PER_DEGREE_DEFAULT;
        double f = (double)freq + corr_hz;
        if (f < 0.0) f = 0.0;
        if (f > (double)OS_MAX_MOTOR_FREQ_HZ) f = (double)OS_MAX_MOTOR_FREQ_HZ;
        freq = (uint32_t)f;
        s_pec_phase_deg += 1.0;
        if (s_pec_phase_deg >= 360.0) s_pec_phase_deg -= 360.0;
    }
    return freq;
}

static uint32_t internal_goto_frequency_hz(int32_t abs_remaining) {
    if (abs_remaining <= OS_GOTO_DECEL_STEPS) return OS_GOTO_APPROACH_FREQ_HZ;
    if (s_motion_elapsed_loops < OS_GOTO_ACCEL_LOOPS) {
        uint32_t f = OS_GOTO_START_FREQ_HZ + s_motion_elapsed_loops * OS_GOTO_ACCEL_FREQ_PER_LOOP;
        if (f > OS_GOTO_MAX_FREQ_HZ) f = OS_GOTO_MAX_FREQ_HZ;
        return f;
    }
    return OS_GOTO_MAX_FREQ_HZ;
}

static void internal_advance_axis_profile(uint8_t axis, int32_t target) {
    if (os_hal_limit_is_triggered(axis)) {
        internal_stop_axis(axis);
        return;
    }
    int32_t current = os_hal_motor_get_position(axis);
    int32_t remaining = target - current;
    if (remaining == 0) {
        internal_stop_axis(axis);
        return;
    }
    bool forward = remaining > 0;
    int32_t abs_rem = forward ? remaining : -remaining;
    uint32_t freq = internal_goto_frequency_hz(abs_rem);
    int32_t steps = OS_ADVANCE_STEPS_PER_LOOP_PROFILE;
    if (steps > abs_rem) steps = abs_rem;
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, freq);
    s_motor_pos[axis] = current + (forward ? steps : -steps);
}

static void internal_finish_goto(void) {
    internal_stop_axis(0);
    internal_stop_axis(1);
    s_goto_active = false;
    s_goto_abort_requested = false;
    s_moving = false;
    s_motion_elapsed_loops = 0u;
    s_system_state = OS_STATE_IDLE_TRACKING;
    s_tracking_enabled = true;
    (void)os_hal_buzzer_beep(200u, 1u);
}

static void internal_finish_park(void) {
    internal_stop_axis(0);
    internal_stop_axis(1);
    (void)os_hal_motor_enable(0, false);
    (void)os_hal_motor_enable(1, false);
    s_parking_active = false;
    s_moving = false;
    s_motion_elapsed_loops = 0u;
    s_parked = true;
    s_tracking_enabled = false;
    s_system_state = OS_STATE_PARKED;
}

static void internal_advance_motion(void) {
    if (!s_goto_active && !s_parking_active) return;
    int32_t target0 = s_goto_active ? s_goto_target_steps[0] : s_park_target_steps[0];
    int32_t target1 = s_goto_active ? s_goto_target_steps[1] : s_park_target_steps[1];
    internal_advance_axis_profile(0u, target0);
    internal_advance_axis_profile(1u, target1);
    s_motion_elapsed_loops++;
    bool done0 = os_hal_motor_get_position(0) == target0;
    bool done1 = os_hal_motor_get_position(1) == target1;
    if (done0 && done1) {
        if (s_goto_active) internal_finish_goto();
        else internal_finish_park();
    }
}

static void internal_guide_axis(uint8_t axis, bool forward, uint32_t duration_ms) {
    uint32_t base = internal_tracking_base_frequency_hz();
    uint32_t bias = (uint32_t)((double)base * (double)s_guide_rate_fraction);
    uint32_t freq = bias;
    bool direction_out = forward;
    if (axis == 0u) {
        direction_out = true;
        if (forward) {
            freq = base + bias;
        } else {
            freq = (base > bias) ? (base - bias) : 0u;
        }
    } else {
        direction_out = forward;
        freq = bias;
    }
    if (freq > OS_MAX_MOTOR_FREQ_HZ) freq = OS_MAX_MOTOR_FREQ_HZ;
    s_guide_pulse.active = true;
    s_guide_pulse.duration_ms = duration_ms;
    s_guide_pulse.rate_fraction = s_guide_rate_fraction;
    s_guide_pulse.direction_east = (axis == 0u && forward);
    s_guide_pulse.direction_north = (axis == 1u && forward);
    s_guide_pulse.dec_priority = (axis == 1u);
    (void)os_hal_motor_set_direction(axis, direction_out);
    (void)os_hal_motor_set_frequency(axis, freq);
}

static void internal_update_guide(void) {
    if (!s_guide_pulse.active) return;
    if (s_guide_pulse.duration_ms == 0u) {
        uint8_t axis = s_guide_pulse.dec_priority ? 1u : 0u;
        internal_stop_axis(axis);
        s_guide_pulse.active = false;
        s_guide_pulse.dec_priority = false;
        return;
    }
    s_guide_pulse.duration_ms--;
    uint8_t axis = s_guide_pulse.dec_priority ? 1u : 0u;
    bool forward = (axis == 1u) ? s_guide_pulse.direction_north : true;
    if (s_motor_freq[axis] > 0u) s_motor_pos[axis] += forward ? 1 : -1;
}

static void internal_update_manual(void) {
    if (!s_manual_active) return;
    if (s_motor_freq[s_manual_axis] > 0u) {
        s_motor_pos[s_manual_axis] += s_manual_forward ? 1 : -1;
    }
}

static void internal_update_tracking(void) {
    if (!s_tracking_enabled || s_parked || s_system_state != OS_STATE_IDLE_TRACKING) return;
    if (!s_site_info_valid) return;
    if (s_goto_active || s_parking_active || s_manual_active || s_guide_pulse.active) return;
    uint32_t freq = internal_tracking_frequency_hz();
    (void)os_hal_motor_set_direction(0, true);
    (void)os_hal_motor_set_frequency(0, freq);
    internal_stop_axis(1);
    if (freq > 0u) s_motor_pos[0] += 1;
}

static void internal_check_limits(void) {
    bool triggered = os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1);
    if (!triggered) return;
    internal_stop_axis(0);
    internal_stop_axis(1);
    if (os_hal_limit_is_triggered(0)) (void)os_hal_motor_enable(0, false);
    if (os_hal_limit_is_triggered(1)) (void)os_hal_motor_enable(1, false);
    s_goto_active = false;
    s_parking_active = false;
    s_manual_active = false;
    s_moving = false;
    s_system_state = OS_STATE_FAULT;
}

static void internal_update_time_source(void) {
    os_site_info_t site;
    memset(&site, 0, sizeof(site));
    os_error_t gps_err = os_hal_gps_poll(&site);
    bool gps_ok = (gps_err == OS_ERR_NONE && site.valid &&
                   site.latitude_degrees >= -90.0f && site.latitude_degrees <= 90.0f &&
                   site.longitude_degrees >= -180.0f && site.longitude_degrees <= 180.0f);
    if (gps_ok) {
        s_site_info = site;
        s_site_info_valid = true;
        s_gps_locked = true;
        s_gps_loss_ticks = 0u;
        if (s_rtc_initialized) {
            if (os_hal_rtc_set(site.utc_epoch_seconds) == OS_ERR_NONE) s_rtc_valid = true;
        }
        return;
    }
    s_gps_loss_ticks++;
    if (s_gps_loss_ticks > OS_GPS_LOCK_TIMEOUT_LOOPS) s_gps_locked = false;
    uint32_t utc = 0u;
    if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
        s_rtc_valid = true;
        s_rtc_epoch = utc;
        if (s_site_info_valid) {
            s_site_info.utc_epoch_seconds = utc;
        } else if (s_preset_site_valid) {
            s_site_info.latitude_degrees = s_preset_site.latitude_degrees;
            s_site_info.longitude_degrees = s_preset_site.longitude_degrees;
            s_site_info.elevation_metres = s_preset_site.elevation_metres;
            s_site_info.utc_epoch_seconds = utc;
            s_site_info.valid = true;
            s_site_info_valid = true;
        }
    } else {
        s_rtc_valid = false;
    }
}

static os_error_t command_error_reply(os_error_t code, char *reply_buffer, size_t reply_buffer_size, size_t *reply_length) {
    char c = '0';
    copy_reply_chars(reply_buffer, reply_buffer_size, reply_length, &c, 1u);
    return code;
}

static os_error_t dispatch_result(os_error_t e, char *reply_buffer, size_t reply_buffer_size, size_t *reply_length) {
    char c = (e == OS_ERR_NONE) ? '1' : '0';
    copy_reply_chars(reply_buffer, reply_buffer_size, reply_length, &c, 1u);
    return e;
}

os_error_t os_init(void) {
    s_system_state = OS_STATE_INITIALIZING;
    s_goto_active = false;
    s_goto_abort_requested = false;
    s_manual_active = false;
    s_parking_active = false;
    s_parked = false;
    s_moving = false;
    s_tracking_enabled = false;
    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_track_custom_factor = 1.0f;
    memset(&s_guide_pulse, 0, sizeof(s_guide_pulse));
    s_guide_rate_fraction = 0.5f;
    s_custom_manual_arcsec_per_sec = 100.0f;
    s_manual_axis = 0u;
    s_manual_forward = false;
    s_motion_elapsed_loops = 0u;
    s_pending_ra_valid = false;
    s_pending_dec_valid = false;
    memset(&s_pending_eq_target, 0, sizeof(s_pending_eq_target));
    s_align_active = false;
    s_align_star_count = 0;
    s_align_mode = OS_ALIGN_1STAR;
    s_align_residual_valid = false;
    s_align_residual_arcsec = 0.0f;
    internal_load_calibration_defaults();
    s_park_position_valid = false;
    memset(&s_park_position, 0, sizeof(s_park_position));
    s_preset_site_valid = false;
    memset(&s_preset_site, 0, sizeof(s_preset_site));
    s_gps_locked = false;
    s_gps_loss_ticks = 0u;
    s_site_info_valid = false;
    memset(&s_site_info, 0, sizeof(s_site_info));
    s_rtc_valid = false;
    s_rtc_initialized = false;
    s_rtc_epoch = 0u;
    s_gps_initialized = false;
    s_pec_enabled = false;
    memset(&s_pec_table, 0, sizeof(s_pec_table));
    s_pec_phase_deg = 0.0;
    for (int i = 0; i < 2; ++i) {
        s_motor_initialized[i] = false;
        s_motor_enabled[i] = false;
        s_motor_forward[i] = false;
        s_motor_freq[i] = 0u;
        s_motor_pos[i] = 0;
        s_last_saved_motor_pos[i] = 0;
        s_limit_triggered[i] = false;
    }
    s_limit_initialized = false;
    s_motor_timer_initialized = false;
    for (int i = 0; i < 4; ++i) {
        s_comm[i].init = false;
        s_comm[i].head = 0u;
        s_comm[i].tail = 0u;
        s_comm[i].tx_len = 0u;
        s_command_len[i] = 0u;
    }
    s_buzzer_last_duration_ms = 0u;
    s_buzzer_last_count = 0u;

    os_error_t critical = OS_ERR_NONE;
    os_error_t nvm_err = os_hal_nvm_init();
    if (nvm_err != OS_ERR_NONE) {
        critical = OS_ERR_NVM_FAULT;
    } else {
        (void)internal_load_calibration();
        (void)internal_load_config();
        (void)internal_load_pec();
        (void)internal_load_motor_position();
    }

    for (uint8_t ch = 0u; ch < 4u; ++ch) (void)os_hal_comm_init(ch);
    for (uint8_t axis = 0u; axis < 2u; ++axis) {
        if (os_hal_motor_init(axis) != OS_ERR_NONE) critical = OS_ERR_MOTOR_DRIVER_FAULT;
    }
    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    (void)os_hal_timer_motor_init();

    bool time_site_ready = false;
    os_site_info_t site;
    memset(&site, 0, sizeof(site));
    os_error_t gps_err = os_hal_gps_poll(&site);
    bool gps_ok = (gps_err == OS_ERR_NONE && site.valid &&
                   site.latitude_degrees >= -90.0f && site.latitude_degrees <= 90.0f &&
                   site.longitude_degrees >= -180.0f && site.longitude_degrees <= 180.0f);
    if (gps_ok) {
        s_site_info = site;
        s_site_info_valid = true;
        s_gps_locked = true;
        s_gps_loss_ticks = 0u;
        time_site_ready = true;
        if (s_rtc_initialized) (void)os_hal_rtc_set(site.utc_epoch_seconds);
    } else {
        s_gps_locked = false;
        uint32_t utc = 0u;
        if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
            s_rtc_valid = true;
            s_rtc_epoch = utc;
            if (s_preset_site_valid) {
                s_site_info.latitude_degrees = s_preset_site.latitude_degrees;
                s_site_info.longitude_degrees = s_preset_site.longitude_degrees;
                s_site_info.elevation_metres = s_preset_site.elevation_metres;
                s_site_info.utc_epoch_seconds = utc;
                s_site_info.valid = true;
                s_site_info_valid = true;
                time_site_ready = true;
            }
        } else {
            s_rtc_valid = false;
            if (critical == OS_ERR_NONE) critical = OS_ERR_TIMEOUT;
        }
    }

    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_track_custom_factor = 1.0f;
    if (critical == OS_ERR_NONE) {
        s_system_state = OS_STATE_IDLE_TRACKING;
        s_tracking_enabled = time_site_ready;
    } else {
        s_system_state = OS_STATE_FAULT;
        s_tracking_enabled = false;
    }
    return critical;
}

void os_loop_iteration(void) {
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        if (!s_comm[ch].init) continue;
        int avail = (int)os_hal_comm_available(ch);
        while (avail > 0 && s_command_len[ch] < OS_MAX_COMMAND_LENGTH) {
            char c = os_hal_comm_read(ch);
            if (c == '\n') {
                size_t len = s_command_len[ch];
                if (len > 0u && s_command_rx[ch][len - 1u] == '\r') len--;
                s_command_rx[ch][len] = '\0';
                char reply[OS_MAX_REPLY_LENGTH];
                size_t rlen = 0u;
                (void)os_command_parse(s_command_rx[ch], len, ch, reply, sizeof(reply), &rlen);
                if (rlen > 0u) (void)os_hal_comm_write(ch, reply, rlen);
                s_command_len[ch] = 0u;
                avail = (int)os_hal_comm_available(ch);
            } else {
                s_command_rx[ch][s_command_len[ch]++] = c;
                avail--;
            }
        }
        if (s_command_len[ch] >= OS_MAX_COMMAND_LENGTH) s_command_len[ch] = 0u;
    }

    internal_update_time_source();
    if (!s_gps_locked && !s_rtc_valid) {
        internal_stop_axis(0);
        internal_stop_axis(1);
        s_goto_active = false;
        s_parking_active = false;
        s_manual_active = false;
        s_moving = false;
        s_tracking_enabled = false;
        s_system_state = OS_STATE_FAULT;
        return;
    }
    if (!s_site_info_valid) s_tracking_enabled = false;

    internal_check_limits();
    if (s_system_state == OS_STATE_FAULT) return;
    internal_advance_motion();
    internal_update_guide();
    internal_update_manual();
    internal_update_tracking();
    if (s_motor_pos[0] != s_last_saved_motor_pos[0] || s_motor_pos[1] != s_last_saved_motor_pos[1]) {
        (void)internal_save_motor_position();
    }
}

os_error_t os_command_parse(const char *command, size_t length, uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (!command || !reply_buffer || !reply_length) return OS_ERR_INVALID_ARGUMENT;
    if (source_channel > OS_CHANNEL_ETHERNET || length == 0u || length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (reply_length) *reply_length = 0u;
    if (length < 2u) return command_error_reply(OS_ERR_COMMAND_FORMAT, reply_buffer, reply_buffer_size, reply_length);
    if (command[0] != OS_LX200_CMD_PREFIX || command[length - 1u] != OS_LX200_CMD_SUFFIX) {
        return command_error_reply(OS_ERR_COMMAND_FORMAT, reply_buffer, reply_buffer_size, reply_length);
    }
    size_t inner_len = length - 2u;
    if (inner_len >= OS_MAX_COMMAND_LENGTH) return command_error_reply(OS_ERR_COMMAND_FORMAT, reply_buffer, reply_buffer_size, reply_length);
    char inner[OS_MAX_COMMAND_LENGTH];
    for (size_t i = 0u; i < inner_len; ++i) inner[i] = command[i + 1u];
    inner[inner_len] = '\0';

    if (inner_len == 3u && inner[0] == 'G' && inner[1] == 'V' && inner[2] == 'P') {
        append_dec_u32(reply_buffer, reply_buffer_size, reply_length, OS_FIRMWARE_VERSION_MAJOR);
        append_char(reply_buffer, reply_buffer_size, reply_length, '.');
        append_dec_u32(reply_buffer, reply_buffer_size, reply_length, OS_FIRMWARE_VERSION_MINOR);
        append_char(reply_buffer, reply_buffer_size, reply_length, '.');
        append_dec_u32(reply_buffer, reply_buffer_size, reply_length, OS_FIRMWARE_VERSION_PATCH);
        return OS_ERR_NONE;
    }
    if (inner_len == 2u && inner[0] == 'G' && inner[1] == 'R') {
        os_equatorial_coord_t c;
        if (os_query_coordinates(&c) == OS_ERR_NONE) {
            append_ra_sexagesimal(reply_buffer, reply_buffer_size, reply_length, c.ra_hours);
        } else {
            append_char(reply_buffer, reply_buffer_size, reply_length, '0');
            append_char(reply_buffer, reply_buffer_size, reply_length, '0');
            append_char(reply_buffer, reply_buffer_size, reply_length, ':');
            append_char(reply_buffer, reply_buffer_size, reply_length, '0');
            append_char(reply_buffer, reply_buffer_size, reply_length, '0');
            append_char(reply_buffer, reply_buffer_size, reply_length, ':');
            append_char(reply_buffer, reply_buffer_size, reply_length, '0');
            append_char(reply_buffer, reply_buffer_size, reply_length, '0');
        }
        return OS_ERR_NONE;
    }
    if (inner_len == 2u && inner[0] == 'G' && inner[1] == 'D') {
        os_equatorial_coord_t c;
        if (os_query_coordinates(&c) == OS_ERR_NONE) {
            append_dec_sexagesimal(reply_buffer, reply_buffer_size, reply_length, c.dec_degrees);
        } else {
            append_char(reply_buffer, reply_buffer_size, reply_length, '+');
            append_char(reply_buffer, reply_buffer_size, reply_length, '0');
            append_char(reply_buffer, reply_buffer_size, reply_length, '0');
            append_char(reply_buffer, reply_buffer_size, reply_length, 42);
            append_char(reply_buffer, reply_buffer_size, reply_length, '0');
            append_char(reply_buffer, reply_buffer_size, reply_length, '0');
            append_char(reply_buffer, reply_buffer_size, reply_length, 39);
            append_char(reply_buffer, reply_buffer_size, reply_length, '0');
            append_char(reply_buffer, reply_buffer_size, reply_length, '0');
            append_char(reply_buffer, reply_buffer_size, reply_length, 34);
        }
        return OS_ERR_NONE;
    }
    if (inner_len == 2u && inner[0] == 'G' && inner[1] == 'g') {
        append_char(reply_buffer, reply_buffer_size, reply_length, s_gps_locked ? '1' : '0');
        return OS_ERR_NONE;
    }
    if (inner_len == 1u && inner[0] == 'Q') {
        append_dec_u32(reply_buffer, reply_buffer_size, reply_length, (uint32_t)s_system_state);
        return OS_ERR_NONE;
    }
    if (inner_len == 2u && inner[0] == 'M' && inner[1] == 'S') {
        if (!s_pending_ra_valid || !s_pending_dec_valid) {
            return command_error_reply(OS_ERR_INVALID_STATE, reply_buffer, reply_buffer_size, reply_length);
        }
        return dispatch_result(os_goto_equatorial(s_pending_eq_target), reply_buffer, reply_buffer_size, reply_length);
    }
    if (inner_len >= 2u && inner[0] == 'S' && inner[1] == 'r') {
        float value = 0.0f;
        if (!internal_parse_angle_float(inner + 2, inner_len - 2u, &value)) {
            return command_error_reply(OS_ERR_COMMAND_FORMAT, reply_buffer, reply_buffer_size, reply_length);
        }
        if (value < OS_RA_MIN_HOURS || value > OS_RA_MAX_HOURS) {
            return dispatch_result(OS_ERR_INVALID_ARGUMENT, reply_buffer, reply_buffer_size, reply_length);
        }
        s_pending_eq_target.ra_hours = value;
        s_pending_ra_valid = true;
        append_char(reply_buffer, reply_buffer_size, reply_length, '1');
        return OS_ERR_NONE;
    }
    if (inner_len >= 2u && inner[0] == 'S' && inner[1] == 'd') {
        float value = 0.0f;
        if (!internal_parse_angle_float(inner + 2, inner_len - 2u, &value)) {
            return command_error_reply(OS_ERR_COMMAND_FORMAT, reply_buffer, reply_buffer_size, reply_length);
        }
        if (value < OS_DEC_MIN_DEG || value > OS_DEC_MAX_DEG) {
            return dispatch_result(OS_ERR_INVALID_ARGUMENT, reply_buffer, reply_buffer_size, reply_length);
        }
        s_pending_eq_target.dec_degrees = value;
        s_pending_dec_valid = true;
        append_char(reply_buffer, reply_buffer_size, reply_length, '1');
        return OS_ERR_NONE;
    }
    if (inner_len == 2u && inner[0] == 'M' && inner[1] == 'e') return dispatch_result(os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM), reply_buffer, reply_buffer_size, reply_length);
    if (inner_len == 2u && inner[0] == 'M' && inner[1] == 'w') return dispatch_result(os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM), reply_buffer, reply_buffer_size, reply_length);
    if (inner_len == 2u && inner[0] == 'M' && inner[1] == 'n') return dispatch_result(os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM), reply_buffer, reply_buffer_size, reply_length);
    if (inner_len == 2u && inner[0] == 'M' && inner[1] == 's') return dispatch_result(os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM), reply_buffer, reply_buffer_size, reply_length);
    if (inner_len == 2u && inner[0] == 'T' && inner[1] == 'e') return dispatch_result(os_tracking_enable(), reply_buffer, reply_buffer_size, reply_length);
    if (inner_len == 2u && inner[0] == 'T' && inner[1] == 'd') return dispatch_result(os_tracking_disable(), reply_buffer, reply_buffer_size, reply_length);
    if (inner_len == 2u && inner[0] == 'h' && inner[1] == 'P') return dispatch_result(os_park(), reply_buffer, reply_buffer_size, reply_length);
    if (inner_len == 2u && inner[0] == 'h' && inner[1] == 'O') return dispatch_result(os_unpark(), reply_buffer, reply_buffer_size, reply_length);
    return command_error_reply(OS_ERR_COMMAND_FORMAT, reply_buffer, reply_buffer_size, reply_length);
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (!valid_eq_coord(target.ra_hours, target.dec_degrees)) return OS_ERR_INVALID_ARGUMENT;
    if (s_system_state != OS_STATE_IDLE_TRACKING) return OS_ERR_INVALID_STATE;
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) return OS_ERR_LIMIT_TRIGGERED;
    internal_eq_to_steps(target, s_goto_target_steps);
    int32_t delta0 = s_goto_target_steps[0] - os_hal_motor_get_position(0);
    int32_t delta1 = s_goto_target_steps[1] - os_hal_motor_get_position(1);
    if (delta0 < 0) delta0 = -delta0;
    if (delta1 < 0) delta1 = -delta1;
    if (delta0 <= 1 && delta1 <= 1) {
        internal_finish_goto();
        return OS_ERR_NONE;
    }
    s_system_state = OS_STATE_GOTO;
    s_goto_active = true;
    s_goto_abort_requested = false;
    s_moving = true;
    s_motion_elapsed_loops = 0u;
    if (s_goto_target_steps[0] != os_hal_motor_get_position(0)) {
        (void)os_hal_motor_set_direction(0, s_goto_target_steps[0] > os_hal_motor_get_position(0));
        (void)os_hal_motor_set_frequency(0, OS_GOTO_START_FREQ_HZ);
    }
    if (s_goto_target_steps[1] != os_hal_motor_get_position(1)) {
        (void)os_hal_motor_set_direction(1, s_goto_target_steps[1] > os_hal_motor_get_position(1));
        (void)os_hal_motor_set_frequency(1, OS_GOTO_START_FREQ_HZ);
    }
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) return OS_ERR_INVALID_ARGUMENT;
    if (s_system_state != OS_STATE_IDLE_TRACKING) return OS_ERR_INVALID_STATE;
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) return OS_ERR_LIMIT_TRIGGERED;
    internal_horiz_to_steps(target, s_goto_target_steps);
    int32_t delta0 = s_goto_target_steps[0] - os_hal_motor_get_position(0);
    int32_t delta1 = s_goto_target_steps[1] - os_hal_motor_get_position(1);
    if (delta0 < 0) delta0 = -delta0;
    if (delta1 < 0) delta1 = -delta1;
    if (delta0 <= 1 && delta1 <= 1) {
        internal_finish_goto();
        return OS_ERR_NONE;
    }
    s_system_state = OS_STATE_GOTO;
    s_goto_active = true;
    s_goto_abort_requested = false;
    s_moving = true;
    s_motion_elapsed_loops = 0u;
    if (s_goto_target_steps[0] != os_hal_motor_get_position(0)) {
        (void)os_hal_motor_set_direction(0, s_goto_target_steps[0] > os_hal_motor_get_position(0));
        (void)os_hal_motor_set_frequency(0, OS_GOTO_START_FREQ_HZ);
    }
    if (s_goto_target_steps[1] != os_hal_motor_get_position(1)) {
        (void)os_hal_motor_set_direction(1, s_goto_target_steps[1] > os_hal_motor_get_position(1));
        (void)os_hal_motor_set_frequency(1, OS_GOTO_START_FREQ_HZ);
    }
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (!s_goto_active) return OS_ERR_INVALID_STATE;
    s_goto_abort_requested = true;
    internal_stop_axis(0);
    internal_stop_axis(1);
    s_goto_active = false;
    s_moving = false;
    s_motion_elapsed_loops = 0u;
    s_system_state = OS_STATE_IDLE_TRACKING;
    s_tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if ((int)rate < 0 || rate > OS_TRACK_RATE_CUSTOM) return OS_ERR_INVALID_ARGUMENT;
    if (rate == OS_TRACK_RATE_CUSTOM && !(custom_factor > 0.0f)) return OS_ERR_INVALID_ARGUMENT;
    s_track_rate = rate;
    s_track_custom_factor = custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (!rate || !custom_factor) return OS_ERR_INVALID_ARGUMENT;
    *rate = s_track_rate;
    *custom_factor = s_track_custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    if (s_system_state == OS_STATE_PARKED || s_system_state == OS_STATE_FAULT) return OS_ERR_INVALID_STATE;
    if (!s_site_info_valid) return OS_ERR_INVALID_STATE;
    s_tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    s_tracking_enabled = false;
    internal_stop_axis(0);
    internal_stop_axis(1);
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) return OS_ERR_INVALID_ARGUMENT;
    if (duration_ms == 0u) return OS_ERR_INVALID_ARGUMENT;
    bool new_is_dec = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);
    uint8_t new_axis = new_is_dec ? 1u : 0u;
    if (s_guide_pulse.active && s_guide_pulse.dec_priority && !new_is_dec) {
        return OS_ERR_NONE;
    }
    if (s_guide_pulse.active) {
        uint8_t old_axis = s_guide_pulse.dec_priority ? 1u : 0u;
        if (old_axis != new_axis) internal_stop_axis(old_axis);
    }
    bool forward = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_NORTH);
    internal_guide_axis(new_axis, forward, duration_ms);
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) return OS_ERR_INVALID_ARGUMENT;
    s_guide_rate_fraction = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (!pulse) return OS_ERR_INVALID_ARGUMENT;
    *pulse = s_guide_pulse;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if ((int)mode < 1 || mode > OS_ALIGN_NSTAR) return OS_ERR_INVALID_ARGUMENT;
    if (s_system_state != OS_STATE_IDLE_TRACKING) return OS_ERR_INVALID_STATE;
    s_align_mode = mode;
    s_align_active = true;
    s_align_star_count = 0;
    s_align_residual_valid = false;
    s_align_residual_arcsec = 0.0f;
    s_system_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord, os_motor_position_t motor_pos) {
    if (!valid_eq_coord(star_coord.ra_hours, star_coord.dec_degrees)) return OS_ERR_INVALID_ARGUMENT;
    if (!s_align_active || s_system_state != OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE;
    if (s_align_star_count >= OS_CALIBRATION_MAX_STARS) return OS_ERR_INVALID_STATE;
    s_align_stars[s_align_star_count] = star_coord;
    s_align_motors[s_align_star_count] = motor_pos;
    s_align_star_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (!s_align_active || s_system_state != OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE;
    uint8_t mins = align_min_stars(s_align_mode);
    if (s_align_star_count < mins) return OS_ERR_INVALID_STATE;
    int m = (int)s_align_star_count;
    if (s_align_mode == OS_ALIGN_1STAR) {
        double as_per_step = 3600.0 / (double)OS_STEPS_PER_DEGREE_DEFAULT;
        double sum_ra_off = 0.0;
        double sum_dec_off = 0.0;
        for (int i = 0; i < m; ++i) {
            double target_ra = (double)s_align_stars[i].ra_hours * 15.0 * 3600.0;
            double target_dec = (double)s_align_stars[i].dec_degrees * 3600.0;
            sum_ra_off += target_ra - (double)s_align_motors[i].ra_steps * as_per_step;
            sum_dec_off += target_dec - (double)s_align_motors[i].dec_steps * as_per_step;
        }
        double off_ra = sum_ra_off / (double)m;
        double off_dec = sum_dec_off / (double)m;
        s_calibration.matrix_ra_to_ra = (float)as_per_step;
        s_calibration.matrix_ra_to_dec = 0.0f;
        s_calibration.matrix_dec_to_ra = 0.0f;
        s_calibration.matrix_dec_to_dec = (float)as_per_step;
        s_calibration.offset_ra_arcsec = (float)off_ra;
        s_calibration.offset_dec_arcsec = (float)off_dec;
        s_calibration.valid = true;
        double residual_sum = 0.0;
        for (int i = 0; i < m; ++i) {
            double pred_ra = (double)s_align_motors[i].ra_steps * as_per_step + off_ra;
            double pred_dec = (double)s_align_motors[i].dec_steps * as_per_step + off_dec;
            double target_ra = (double)s_align_stars[i].ra_hours * 15.0 * 3600.0;
            double target_dec = (double)s_align_stars[i].dec_degrees * 3600.0;
            double e_ra = pred_ra - target_ra;
            double e_dec = pred_dec - target_dec;
            residual_sum += e_ra * e_ra + e_dec * e_dec;
        }
        s_align_residual_arcsec = (float)sqrt(residual_sum / (2.0 * (double)m));
        s_align_residual_valid = true;
        s_align_active = false;
        s_system_state = OS_STATE_IDLE_TRACKING;
        return internal_save_calibration();
    }
    int n_ra = (s_align_mode == OS_ALIGN_2STAR) ? 2 : 3;
    int n_dec = n_ra;
    double A[OS_CALIBRATION_MAX_STARS * 3];
    double b[OS_CALIBRATION_MAX_STARS];
    double ra_coef[3] = {0.0, 0.0, 0.0};
    double dec_coef[3] = {0.0, 0.0, 0.0};
    memset(A, 0, sizeof(A));
    for (int i = 0; i < m; ++i) {
        A[i * n_ra + 0] = 1.0;
        if (n_ra >= 2) A[i * n_ra + 1] = (double)s_align_motors[i].ra_steps;
        if (n_ra >= 3) A[i * n_ra + 2] = (double)s_align_motors[i].dec_steps;
        b[i] = (double)s_align_stars[i].ra_hours * 15.0 * 3600.0;
    }
    double det_ra = 0.0;
    if (m == 3 && n_ra == 3) det_ra = internal_det3(A);
    if (!ls_solve(A, m, n_ra, b, ra_coef)) return OS_ERR_CALIBRATION_FAILED;
    memset(A, 0, sizeof(A));
    for (int i = 0; i < m; ++i) {
        A[i * n_dec + 0] = 1.0;
        if (n_dec >= 2) A[i * n_dec + 1] = (double)s_align_motors[i].dec_steps;
        if (n_dec >= 3) A[i * n_dec + 2] = (double)s_align_motors[i].ra_steps;
        b[i] = (double)s_align_stars[i].dec_degrees * 3600.0;
    }
    double det_dec = 0.0;
    if (m == 3 && n_dec == 3) det_dec = internal_det3(A);
    if (!ls_solve(A, m, n_dec, b, dec_coef)) return OS_ERR_CALIBRATION_FAILED;
    if (m == 3 && n_ra == 3) {
        if (fabs(det_ra) < 1e-9 || fabs(det_dec) < 1e-9) return OS_ERR_CALIBRATION_FAILED;
    }
    double residual = 0.0;
    if (m >= 4) {
        double sum = 0.0;
        for (int i = 0; i < m; ++i) {
            double mot_ra = (double)s_align_motors[i].ra_steps;
            double mot_dec = (double)s_align_motors[i].dec_steps;
            double pred_ra = ra_coef[0] + ra_coef[1] * mot_ra + ra_coef[2] * mot_dec;
            double pred_dec = dec_coef[0] + dec_coef[1] * mot_dec + dec_coef[2] * mot_ra;
            double target_ra = (double)s_align_stars[i].ra_hours * 15.0 * 3600.0;
            double target_dec = (double)s_align_stars[i].dec_degrees * 3600.0;
            double e_ra = pred_ra - target_ra;
            double e_dec = pred_dec - target_dec;
            sum += e_ra * e_ra + e_dec * e_dec;
        }
        residual = sqrt(sum / (2.0 * (double)m));
        if (residual > OS_CALIBRATION_MAX_RESIDUAL_ARCSEC) return OS_ERR_CALIBRATION_FAILED;
    }
    s_calibration.matrix_ra_to_ra = (float)ra_coef[1];
    s_calibration.matrix_dec_to_ra = (float)ra_coef[2];
    s_calibration.matrix_ra_to_dec = (float)dec_coef[2];
    s_calibration.matrix_dec_to_dec = (float)dec_coef[1];
    s_calibration.offset_ra_arcsec = (float)ra_coef[0];
    s_calibration.offset_dec_arcsec = (float)dec_coef[0];
    if (s_align_mode == OS_ALIGN_2STAR) {
        s_calibration.matrix_dec_to_ra = 0.0f;
        s_calibration.matrix_ra_to_dec = 0.0f;
    }
    s_calibration.valid = true;
    s_align_residual_valid = true;
    s_align_residual_arcsec = (float)residual;
    s_align_active = false;
    s_system_state = OS_STATE_IDLE_TRACKING;
    return internal_save_calibration();
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (!residual_arcsec) return OS_ERR_INVALID_ARGUMENT;
    if (!s_align_residual_valid) return OS_ERR_INVALID_STATE;
    *residual_arcsec = s_align_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    if (!s_align_active && s_system_state != OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE;
    s_align_active = false;
    s_align_star_count = 0;
    s_align_residual_valid = false;
    s_align_residual_arcsec = 0.0f;
    s_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (s_system_state != OS_STATE_IDLE_TRACKING) return OS_ERR_INVALID_STATE;
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) return OS_ERR_LIMIT_TRIGGERED;
    if (s_park_position_valid) {
        internal_eq_to_steps(s_park_position, s_park_target_steps);
    } else {
        s_park_target_steps[0] = 0;
        s_park_target_steps[1] = 0;
    }
    int32_t delta0 = s_park_target_steps[0] - os_hal_motor_get_position(0);
    int32_t delta1 = s_park_target_steps[1] - os_hal_motor_get_position(1);
    if (delta0 < 0) delta0 = -delta0;
    if (delta1 < 0) delta1 = -delta1;
    s_tracking_enabled = false;
    if (delta0 <= 1 && delta1 <= 1) {
        internal_finish_park();
        return OS_ERR_NONE;
    }
    s_parking_active = true;
    s_moving = true;
    s_motion_elapsed_loops = 0u;
    s_system_state = OS_STATE_GOTO;
    if (delta0 != 0) {
        (void)os_hal_motor_set_direction(0, s_park_target_steps[0] > os_hal_motor_get_position(0));
        (void)os_hal_motor_set_frequency(0, OS_GOTO_START_FREQ_HZ);
    }
    if (delta1 != 0) {
        (void)os_hal_motor_set_direction(1, s_park_target_steps[1] > os_hal_motor_get_position(1));
        (void)os_hal_motor_set_frequency(1, OS_GOTO_START_FREQ_HZ);
    }
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (s_system_state != OS_STATE_PARKED) return OS_ERR_INVALID_STATE;
    (void)os_hal_motor_enable(0, true);
    (void)os_hal_motor_enable(1, true);
    for (uint8_t ch = 0u; ch < 4u; ++ch) (void)os_hal_comm_init(ch);
    if (s_gps_locked && s_rtc_initialized) (void)os_hal_rtc_set(s_site_info.utc_epoch_seconds);
    s_parked = false;
    s_tracking_enabled = true;
    s_moving = false;
    s_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!valid_eq_coord(park_pos.ra_hours, park_pos.dec_degrees)) return OS_ERR_INVALID_ARGUMENT;
    s_park_position = park_pos;
    s_park_position_valid = true;
    return internal_write_config();
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) return OS_ERR_INVALID_ARGUMENT;
    if (speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) return OS_ERR_INVALID_ARGUMENT;
    if (s_system_state != OS_STATE_IDLE_TRACKING) return OS_ERR_INVALID_STATE;
    uint8_t axis = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) ? 0u : 1u;
    if (os_hal_limit_is_triggered(axis)) return OS_ERR_LIMIT_TRIGGERED;
    bool forward = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_NORTH);
    uint32_t freq = OS_MANUAL_MEDIUM_HZ;
    if (speed == OS_SPEED_SLOW) freq = OS_MANUAL_SLOW_HZ;
    else if (speed == OS_SPEED_MEDIUM) freq = OS_MANUAL_MEDIUM_HZ;
    else if (speed == OS_SPEED_FAST) freq = OS_MANUAL_FAST_HZ;
    else if (speed == OS_SPEED_CUSTOM) {
        double f = (double)s_custom_manual_arcsec_per_sec / 3600.0 * (double)OS_STEPS_PER_DEGREE_DEFAULT;
        if (f < 0.0) f = 0.0;
        if (f > (double)OS_MAX_MOTOR_FREQ_HZ) f = (double)OS_MAX_MOTOR_FREQ_HZ;
        freq = (uint32_t)f;
    }
    s_manual_active = true;
    s_manual_axis = axis;
    s_manual_forward = forward;
    s_moving = true;
    s_system_state = OS_STATE_MANUAL_MOTION;
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, freq);
    internal_stop_axis((uint8_t)(1u - axis));
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (s_system_state != OS_STATE_MANUAL_MOTION || !s_manual_active) return OS_ERR_INVALID_STATE;
    internal_stop_axis(0);
    internal_stop_axis(1);
    s_manual_active = false;
    s_manual_axis = 0u;
    s_manual_forward = false;
    s_moving = false;
    s_system_state = OS_STATE_IDLE_TRACKING;
    s_tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (!(arcsec_per_sec > 0.0f) || arcsec_per_sec > OS_MANUAL_CUSTOM_MAX_ARCSEC_PER_SEC) return OS_ERR_INVALID_ARGUMENT;
    s_custom_manual_arcsec_per_sec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (!state) return OS_ERR_INVALID_ARGUMENT;
    *state = s_system_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (!coord) return OS_ERR_INVALID_ARGUMENT;
    if (!s_calibration.valid) return OS_ERR_INVALID_STATE;
    int32_t ra_steps = os_hal_motor_get_position(0);
    int32_t dec_steps = os_hal_motor_get_position(1);
    double pra = (double)s_calibration.offset_ra_arcsec + (double)s_calibration.matrix_ra_to_ra * (double)ra_steps + (double)s_calibration.matrix_dec_to_ra * (double)dec_steps;
    double pdec = (double)s_calibration.offset_dec_arcsec + (double)s_calibration.matrix_ra_to_dec * (double)ra_steps + (double)s_calibration.matrix_dec_to_dec * (double)dec_steps;
    coord->ra_hours = (float)(pra / (15.0 * 3600.0));
    coord->dec_degrees = (float)(pdec / 3600.0);
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (!site) return OS_ERR_INVALID_ARGUMENT;
    if (!s_site_info_valid) return OS_ERR_INVALID_STATE;
    *site = s_site_info;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (!pos) return OS_ERR_INVALID_ARGUMENT;
    pos->ra_steps = os_hal_motor_get_position(0);
    pos->dec_steps = os_hal_motor_get_position(1);
    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor, uint8_t *patch) {
    if (!major || !minor || !patch) return OS_ERR_INVALID_ARGUMENT;
    *major = OS_FIRMWARE_VERSION_MAJOR;
    *minor = OS_FIRMWARE_VERSION_MINOR;
    *patch = OS_FIRMWARE_VERSION_PATCH;
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *moving) {
    if (!moving) return OS_ERR_INVALID_ARGUMENT;
    *moving = s_moving;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (!locked) return OS_ERR_INVALID_ARGUMENT;
    *locked = s_gps_locked;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    s_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (!table) return OS_ERR_INVALID_ARGUMENT;
    memset(&s_pec_table, 0, sizeof(s_pec_table));
    memcpy(s_pec_table.corrections, table->corrections, sizeof(s_pec_table.corrections));
    s_pec_table.valid = true;
    return internal_save_pec();
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (!table) return OS_ERR_INVALID_ARGUMENT;
    *table = s_pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) return OS_ERR_INVALID_ARGUMENT;
    int idx = (worm_phase_deg >= 360.0f) ? 0 : (int)worm_phase_deg;
    if (idx < 0 || idx >= OS_PEC_TABLE_SIZE) return OS_ERR_INVALID_ARGUMENT;
    s_pec_table.corrections[idx] = error_arcsec;
    s_pec_table.valid = true;
    return internal_save_pec();
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (!calib) return OS_ERR_INVALID_ARGUMENT;
    *calib = s_calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    internal_load_calibration_defaults();
    internal_clear_calibration_nvm();
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_init(uint8_t axis) {
    if (axis > 1u) return OS_ERR_INVALID_ARGUMENT;
    s_motor_initialized[axis] = true;
    s_motor_enabled[axis] = false;
    s_motor_freq[axis] = 0u;
    s_motor_forward[axis] = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (axis > 1u) return OS_ERR_INVALID_ARGUMENT;
    if (frequency_hz > OS_MAX_MOTOR_FREQ_HZ) frequency_hz = OS_MAX_MOTOR_FREQ_HZ;
    s_motor_freq[axis] = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (axis > 1u) return OS_ERR_INVALID_ARGUMENT;
    s_motor_forward[axis] = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (axis > 1u) return OS_ERR_INVALID_ARGUMENT;
    s_motor_enabled[axis] = enable;
    if (!enable) s_motor_freq[axis] = 0u;
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (axis > 1u) return 0;
    return s_motor_pos[axis];
}

os_error_t os_hal_gps_init(void) {
    s_gps_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (!site) return OS_ERR_INVALID_ARGUMENT;
    memset(site, 0, sizeof(*site));
    site->valid = false;
    if (!s_gps_initialized) return OS_ERR_GPS_NO_SIGNAL;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void) {
    s_rtc_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (!utc_epoch_seconds) return OS_ERR_INVALID_ARGUMENT;
    if (!s_rtc_initialized) {
        *utc_epoch_seconds = 0u;
        return OS_ERR_TIMEOUT;
    }
    *utc_epoch_seconds = s_rtc_epoch;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    s_rtc_epoch = utc_epoch_seconds;
    s_rtc_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    s_limit_initialized = true;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (axis > 1u) return true;
    return s_limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void) {
    s_nvm_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (!data) return OS_ERR_INVALID_ARGUMENT;
    if ((uint32_t)offset + (uint32_t)length > (uint32_t)OS_NVM_TOTAL_SIZE) return OS_ERR_INVALID_ARGUMENT;
    if (!s_nvm_initialized) return OS_ERR_NVM_FAULT;
    memcpy(data, &s_nvm_memory[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (!data) return OS_ERR_INVALID_ARGUMENT;
    if ((uint32_t)offset + (uint32_t)length > (uint32_t)OS_NVM_TOTAL_SIZE) return OS_ERR_INVALID_ARGUMENT;
    if (!s_nvm_initialized) return OS_ERR_NVM_FAULT;
    memcpy(&s_nvm_memory[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET) return OS_ERR_INVALID_ARGUMENT;
    memset(&s_comm[channel], 0, sizeof(s_comm[channel]));
    s_comm[channel].init = true;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET || !s_comm[channel].init) return 0;
    if (s_comm[channel].tail >= s_comm[channel].head) return (int16_t)(s_comm[channel].tail - s_comm[channel].head);
    return (int16_t)(OS_COMM_RX_CAP - s_comm[channel].head + s_comm[channel].tail);
}

char os_hal_comm_read(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET || !s_comm[channel].init) return '\0';
    if (os_hal_comm_available(channel) <= 0) return '\0';
    char c = s_comm[channel].rx[s_comm[channel].head];
    s_comm[channel].head = (uint16_t)((s_comm[channel].head + 1u) % OS_COMM_RX_CAP);
    return c;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (channel > OS_CHANNEL_ETHERNET) return OS_ERR_INVALID_ARGUMENT;
    if (!data) return OS_ERR_INVALID_ARGUMENT;
    if (!s_comm[channel].init) return OS_ERR_NOT_SUPPORTED;
    if ((size_t)s_comm[channel].tx_len + length > OS_COMM_TX_CAP) return OS_ERR_TIMEOUT;
    memcpy(&s_comm[channel].tx[s_comm[channel].tx_len], data, length);
    s_comm[channel].tx_len = (uint16_t)(s_comm[channel].tx_len + length);
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) {
    if (duration_ms == 0u || count == 0u) return OS_ERR_INVALID_ARGUMENT;
    s_buzzer_last_duration_ms = duration_ms;
    s_buzzer_last_count = count;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void) {
    s_motor_timer_initialized = true;
    return OS_ERR_NONE;
}
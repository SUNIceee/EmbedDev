#include "6_generated_code.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define OS_AXIS_RA                0u
#define OS_AXIS_DEC               1u
#define OS_DEFAULT_STEPS_PER_ARCSEC  16.0
#define OS_RA_ARCSEC_PER_HOUR     54000.0
#define OS_DEC_ARCSEC_PER_DEG     3600.0
#define OS_SIM_MS_PER_LOOP        10u
#define OS_ALIGN_MAX_RESIDUAL_ARCSEC 600.0f
#define OS_MOTOR_GOTO_FREQ_HZ     1000u
#define OS_GOTO_DEADBAND_STEPS    8L
#define OS_NVM_TOTAL_SIZE         (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)
#define OS_NVM_CAL_MAGIC          0x4F4E5354u
#define OS_NVM_CFG_MAGIC          0x4F4E4346u
#define OS_NVM_VERSION            1u

typedef struct {
    bool initialized;
    bool enabled;
    bool direction;
    uint32_t frequency_hz;
    int32_t position_steps;
    bool fault;
} motor_hal_state_t;

typedef struct {
    bool initialized;
    uint16_t rx_head;
    uint16_t rx_len;
    uint16_t tx_len;
    char rx[OS_MAX_COMMAND_LENGTH * 4u];
    char tx[OS_MAX_REPLY_LENGTH * 4u];
} comm_hal_state_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t checksum;
    uint8_t reserved;
    os_calibration_t calib;
    double solution[6];
} nvm_cal_record_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t checksum;
    uint8_t reserved;
    float park_ra_hours;
    float park_dec_degrees;
} nvm_cfg_record_t;

typedef struct {
    double ra_hours;
    double dec_degrees;
    int32_t ra_steps;
    int32_t dec_steps;
} align_entry_t;

static bool g_initialized = false;
static os_state_t g_system_state = OS_STATE_INITIALIZING;

static bool g_goto_active = false;
static os_equatorial_coord_t g_last_equ_target;
static int32_t g_goto_target_steps[2] = {0, 0};

static bool g_park_active = false;
static bool g_park_moving = false;
static int32_t g_park_target_steps[2] = {0, 0};
static os_equatorial_coord_t g_park_position;

static bool g_manual_active = false;
static os_direction_t g_manual_direction = OS_DIRECTION_NORTH;
static os_speed_level_t g_manual_speed = OS_SPEED_SLOW;
static float g_manual_custom_speed_arcsec_per_sec = 100.0f;

static os_guide_pulse_t g_guide_pulse;
static uint32_t g_guide_remaining_ms = 0u;
static float g_guide_rate_fraction = 0.5f;

static bool g_tracking_enabled = false;
static os_track_rate_t g_tracking_rate = OS_TRACK_RATE_SIDEREAL;
static float g_tracking_custom_factor = 1.0f;

static os_align_mode_t g_alignment_mode = OS_ALIGN_3STAR;
static uint8_t g_alignment_star_count = 0u;
static bool g_alignment_computed = false;
static float g_alignment_residual_arcsec = 0.0f;
static align_entry_t g_alignment_entries[OS_CALIBRATION_MAX_STARS];

static os_calibration_t g_calibration;
static double g_solution[6] = {0.0, OS_DEFAULT_STEPS_PER_ARCSEC, 0.0, 0.0, 0.0, OS_DEFAULT_STEPS_PER_ARCSEC};

static bool g_pec_enabled = false;
static os_pec_table_t g_pec_table;

static os_site_info_t g_site;
static bool g_gps_locked = false;

static motor_hal_state_t g_motor[2];
static bool g_gps_initialized = false;
static os_site_info_t g_gps_site;
static bool g_gps_has_valid = false;
static bool g_rtc_initialized = false;
static uint32_t g_rtc_time = 1700000000u;
static bool g_limits_initialized = false;
static bool g_limit_triggered[2] = {false, false};
static bool g_nvm_initialized = false;
static uint8_t g_nvm_image[OS_NVM_TOTAL_SIZE];
static comm_hal_state_t g_comm[4];
static bool g_buzzer_pending = false;
static uint16_t g_buzzer_duration_ms = 0u;
static uint8_t g_buzzer_count = 0u;
static bool g_timer_motor_initialized = false;

static char g_partial_cmd[4][OS_MAX_COMMAND_LENGTH + 2u];
static uint8_t g_partial_len[4] = {0u, 0u, 0u, 0u};

static bool axis_valid(uint8_t axis) {
    return axis == OS_AXIS_RA || axis == OS_AXIS_DEC;
}

static int32_t iabs32(int32_t v) {
    return v < 0 ? -v : v;
}

static void stop_axis(uint8_t axis) {
    if (!axis_valid(axis)) return;
    g_motor[axis].frequency_hz = 0u;
    g_motor[axis].enabled = false;
    g_motor[axis].direction = false;
}

static os_error_t reply_format(char *buffer, size_t buffer_size, size_t *reply_length,
                               const char *format, ...) {
    if (buffer == NULL || reply_length == NULL || buffer_size == 0u) {
        *reply_length = 0u;
        return OS_ERR_INVALID_ARGUMENT;
    }
    va_list ap;
    va_start(ap, format);
    int n = vsnprintf(buffer, buffer_size, format, ap);
    va_end(ap);
    if (n < 0) {
        *reply_length = 0u;
        return OS_ERR_COMMAND_FORMAT;
    }
    if ((size_t)n >= buffer_size) {
        n = (int)(buffer_size - 1u);
    }
    *reply_length = (size_t)n;
    return OS_ERR_NONE;
}

static double ra_hours_to_steps(double ra_hours) {
    return ra_hours * OS_RA_ARCSEC_PER_HOUR * OS_DEFAULT_STEPS_PER_ARCSEC;
}

static double dec_degrees_to_steps(double dec_degrees) {
    return dec_degrees * OS_DEC_ARCSEC_PER_DEG * OS_DEFAULT_STEPS_PER_ARCSEC;
}

static void steps_to_equatorial(int32_t ra_steps, int32_t dec_steps,
                                float *ra_hours, float *dec_degrees) {
    double p0 = g_solution[0];
    double p1 = g_solution[1];
    double p2 = g_solution[2];
    double p3 = g_solution[3];
    double p4 = g_solution[4];
    double p5 = g_solution[5];
    double s0 = (double)ra_steps;
    double s1 = (double)dec_steps;
    double x = 0.0;
    double y = 0.0;
    if (g_calibration.valid) {
        double det = (p1 * p5) - (p2 * p4);
        if (fabs(det) > 1e-9) {
            x = ((p5 * (s0 - p0)) - (p2 * (s1 - p3))) / det;
            y = ((-p4 * (s0 - p0)) + (p1 * (s1 - p3))) / det;
        } else {
            x = (s0 - p0) / p1;
            y = (s1 - p3) / p5;
        }
    } else {
        x = (s0 - p0) / p1;
        y = (s1 - p3) / p5;
    }

    double ra = x / OS_RA_ARCSEC_PER_HOUR;
    if (ra < 0.0) ra += 24.0;
    if (ra >= 24.0) ra = fmod(ra, 24.0);
    if (ra < 0.0) ra = 0.0;

    double dec = y / OS_DEC_ARCSEC_PER_DEG;
    if (dec < OS_DEC_MIN_DEG) dec = OS_DEC_MIN_DEG;
    if (dec > OS_DEC_MAX_DEG) dec = OS_DEC_MAX_DEG;

    *ra_hours = (float)ra;
    *dec_degrees = (float)dec;
}

static uint8_t record_checksum(const uint8_t *data, size_t length, size_t skip_offset) {
    uint8_t sum = 0u;
    for (size_t i = 0u; i < length; i++) {
        if (i == skip_offset) continue;
        sum = (uint8_t)(sum + data[i]);
    }
    return (uint8_t)(~sum);
}

static bool record_checksum_ok(const uint8_t *data, size_t length, size_t skip_offset,
                               uint8_t expected) {
    return record_checksum(data, length, skip_offset) == expected;
}

static os_error_t save_calibration_to_nvm(void) {
    nvm_cal_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = OS_NVM_CAL_MAGIC;
    rec.version = OS_NVM_VERSION;
    rec.reserved = 0u;
    rec.calib = g_calibration;
    memcpy(rec.solution, g_solution, sizeof(rec.solution));
    size_t skip = offsetof(nvm_cal_record_t, checksum);
    rec.checksum = record_checksum((const uint8_t *)&rec, sizeof(rec), skip);
    return os_hal_nvm_write(0u, (const uint8_t *)&rec, (uint16_t)sizeof(rec));
}

static os_error_t save_park_config_to_nvm(void) {
    nvm_cfg_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = OS_NVM_CFG_MAGIC;
    rec.version = OS_NVM_VERSION;
    rec.reserved = 0u;
    rec.park_ra_hours = g_park_position.ra_hours;
    rec.park_dec_degrees = g_park_position.dec_degrees;
    size_t skip = offsetof(nvm_cfg_record_t, checksum);
    rec.checksum = record_checksum((const uint8_t *)&rec, sizeof(rec), skip);
    return os_hal_nvm_write(OS_NVM_CONFIG_SIZE_BYTES, (const uint8_t *)&rec,
                            (uint16_t)sizeof(rec));
}

static uint32_t tracking_frequency_hz(void) {
    float factor = 1.0f;
    switch (g_tracking_rate) {
        case OS_TRACK_RATE_LUNAR:
            factor = OS_LUNAR_RATE_FACTOR;
            break;
        case OS_TRACK_RATE_SOLAR:
            factor = OS_SOLAR_RATE_FACTOR;
            break;
        case OS_TRACK_RATE_CUSTOM:
            factor = g_tracking_custom_factor;
            break;
        case OS_TRACK_RATE_SIDEREAL:
        default:
            factor = 1.0f;
            break;
    }

    double rate_arcsec_per_sec = (double)OS_SIDEREAL_RATE_ARCSEC_PER_SEC * (double)factor;
    uint32_t base_hz = (uint32_t)(rate_arcsec_per_sec * OS_DEFAULT_STEPS_PER_ARCSEC + 0.5);

    if (g_pec_enabled && g_pec_table.valid) {
        int32_t pos = g_motor[OS_AXIS_RA].position_steps;
        uint32_t rev = 100000u;
        uint32_t phase_counts = (uint32_t)(pos % (int32_t)rev);
        if (phase_counts >= rev) phase_counts = phase_counts % rev;
        uint32_t idx = (phase_counts * 360u) / rev;
        if (idx >= (uint32_t)OS_PEC_TABLE_SIZE) idx = (uint32_t)OS_PEC_TABLE_SIZE - 1u;
        int32_t correction = (int32_t)g_pec_table.corrections[idx];
        double corr_hz = ((double)base_hz * (double)correction * OS_DEFAULT_STEPS_PER_ARCSEC) /
                         rate_arcsec_per_sec;
        int32_t corr_i = (int32_t)corr_hz;
        if (corr_i > 0 && (uint32_t)corr_i > base_hz) corr_i = (int32_t)base_hz;
        if (corr_i < 0 && (uint32_t)(-corr_i) > base_hz) corr_i = -(int32_t)base_hz;
        base_hz = (uint32_t)((int32_t)base_hz + corr_i);
    }
    return base_hz;
}

static void set_tracking_motor_frequency(void) {
    if (!g_tracking_enabled) {
        stop_axis(OS_AXIS_RA);
        return;
    }
    uint32_t freq = tracking_frequency_hz();
    if (freq > 0u) {
        g_motor[OS_AXIS_RA].enabled = true;
        g_motor[OS_AXIS_RA].frequency_hz = freq;
    } else {
        stop_axis(OS_AXIS_RA);
    }
}

static void update_goto_axis_frequencies(const int32_t target_steps[2]) {
    int32_t pos0 = g_motor[OS_AXIS_RA].position_steps;
    int32_t pos1 = g_motor[OS_AXIS_DEC].position_steps;
    int32_t rem0 = target_steps[0] - pos0;
    int32_t rem1 = target_steps[1] - pos1;

    if (iabs32(rem0) > OS_GOTO_DEADBAND_STEPS) {
        g_motor[OS_AXIS_RA].enabled = true;
        g_motor[OS_AXIS_RA].direction = rem0 > 0;
        g_motor[OS_AXIS_RA].frequency_hz = OS_MOTOR_GOTO_FREQ_HZ;
    } else {
        stop_axis(OS_AXIS_RA);
    }

    if (iabs32(rem1) > OS_GOTO_DEADBAND_STEPS) {
        g_motor[OS_AXIS_DEC].enabled = true;
        g_motor[OS_AXIS_DEC].direction = rem1 > 0;
        g_motor[OS_AXIS_DEC].frequency_hz = OS_MOTOR_GOTO_FREQ_HZ;
    } else {
        stop_axis(OS_AXIS_DEC);
    }
}

static void update_active_frequencies(void) {
    if (g_guide_pulse.active) {
        return;
    }

    if (g_system_state == OS_STATE_GOTO) {
        const int32_t *targets = g_park_active ? g_park_target_steps : g_goto_target_steps;
        update_goto_axis_frequencies(targets);
    } else if (g_system_state == OS_STATE_IDLE_TRACKING) {
        set_tracking_motor_frequency();
    } else if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        stop_axis(OS_AXIS_RA);
        stop_axis(OS_AXIS_DEC);
    }
}

static void simulate_motor_pulses(void) {
    for (uint8_t axis = OS_AXIS_RA; axis <= OS_AXIS_DEC; axis++) {
        if (!g_motor[axis].enabled || g_motor[axis].frequency_hz == 0u) continue;
        int32_t delta = (int32_t)((g_motor[axis].frequency_hz * OS_SIM_MS_PER_LOOP) / 1000u);
        if (delta == 0) delta = 1;
        if (!g_motor[axis].direction) delta = -delta;
        g_motor[axis].position_steps += delta;
    }
}

static void check_limits_and_safety(void) {
    bool fault = false;
    for (uint8_t axis = OS_AXIS_RA; axis <= OS_AXIS_DEC; axis++) {
        if (g_motor[axis].enabled && g_motor[axis].frequency_hz > 0u &&
            os_hal_limit_is_triggered(axis)) {
            stop_axis(axis);
            fault = true;
        }
    }
    if (fault) {
        stop_axis(OS_AXIS_RA);
        stop_axis(OS_AXIS_DEC);
        g_goto_active = false;
        g_park_active = false;
        g_park_moving = false;
        g_manual_active = false;
        g_guide_pulse.active = false;
        g_tracking_enabled = false;
        g_system_state = OS_STATE_FAULT;
        os_hal_buzzer_beep(300u, 3u);
    }
}

static void advance_motion_state(void) {
    if (g_guide_pulse.active) {
        if (g_guide_remaining_ms > OS_SIM_MS_PER_LOOP) {
            g_guide_remaining_ms -= OS_SIM_MS_PER_LOOP;
        } else {
            g_guide_remaining_ms = 0u;
            g_guide_pulse.active = false;
            update_active_frequencies();
        }
    }

    if (g_system_state != OS_STATE_GOTO) return;

    int32_t pos0 = g_motor[OS_AXIS_RA].position_steps;
    int32_t pos1 = g_motor[OS_AXIS_DEC].position_steps;
    const int32_t *targets = g_park_active ? g_park_target_steps : g_goto_target_steps;
    int32_t rem0 = targets[0] - pos0;
    int32_t rem1 = targets[1] - pos1;

    if (iabs32(rem0) <= OS_GOTO_DEADBAND_STEPS &&
        iabs32(rem1) <= OS_GOTO_DEADBAND_STEPS) {
        stop_axis(OS_AXIS_RA);
        stop_axis(OS_AXIS_DEC);
        if (g_park_active) {
            g_park_active = false;
            g_park_moving = false;
            g_tracking_enabled = false;
            g_system_state = OS_STATE_PARKED;
        } else {
            g_goto_active = false;
            g_system_state = OS_STATE_IDLE_TRACKING;
            g_tracking_enabled = true;
            os_hal_buzzer_beep(200u, 2u);
            update_active_frequencies();
        }
    }
}

static void update_time_and_site(void) {
    os_site_info_t tmp;
    memset(&tmp, 0, sizeof(tmp));
    os_error_t err = os_hal_gps_poll(&tmp);
    if (err == OS_ERR_NONE && tmp.valid) {
        g_site = tmp;
        g_gps_locked = true;
        (void)os_hal_rtc_set(tmp.utc_epoch_seconds);
    } else {
        g_gps_locked = false;
        g_site.valid = false;
        uint32_t time_value = 0u;
        if (os_hal_rtc_read(&time_value) == OS_ERR_NONE) {
            g_site.utc_epoch_seconds = time_value;
        }
    }
}

static void process_commands(void) {
    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        if (!g_comm[ch].initialized) continue;
        while (os_hal_comm_available(ch) > 0) {
            char c = os_hal_comm_read(ch);
            if (c == '\r' || c == '\n') continue;
            if (g_partial_len[ch] >= OS_MAX_COMMAND_LENGTH) {
                g_partial_len[ch] = 0u;
                continue;
            }
            g_partial_cmd[ch][g_partial_len[ch]++] = c;
            if (c == OS_LX200_CMD_SUFFIX) {
                char reply[OS_MAX_REPLY_LENGTH];
                size_t reply_len = 0u;
                (void)os_command_parse(g_partial_cmd[ch], g_partial_len[ch], ch,
                                       reply, sizeof(reply), &reply_len);
                if (reply_len > 0u) {
                    (void)os_hal_comm_write(ch, reply, reply_len);
                }
                g_partial_len[ch] = 0u;
            }
        }
    }
}

static void default_solution(void) {
    g_solution[0] = 0.0;
    g_solution[1] = OS_DEFAULT_STEPS_PER_ARCSEC;
    g_solution[2] = 0.0;
    g_solution[3] = 0.0;
    g_solution[4] = 0.0;
    g_solution[5] = OS_DEFAULT_STEPS_PER_ARCSEC;
}

static int alignment_min_stars(os_align_mode_t mode) {
    switch (mode) {
        case OS_ALIGN_1STAR:
            return 1;
        case OS_ALIGN_2STAR:
            return 2;
        case OS_ALIGN_3STAR:
        case OS_ALIGN_NSTAR:
        default:
            return 3;
    }
}

static bool qr_solve_fixed(int n, int p, double A[][3], const double *b, double *beta) {
    double q[9][3];
    double r[3][3];
    memset(q, 0, sizeof(q));
    memset(r, 0, sizeof(r));

    for (int j = 0; j < p; j++) {
        for (int i = 0; i < n; i++) {
            q[i][j] = A[i][j];
        }
    }

    for (int k = 0; k < p; k++) {
        double norm = 0.0;
        for (int i = 0; i < n; i++) norm += q[i][k] * q[i][k];
        norm = sqrt(norm);
        if (norm < 1e-12) return false;
        r[k][k] = norm;
        for (int i = 0; i < n; i++) q[i][k] /= norm;

        for (int j = k + 1; j < p; j++) {
            double dot = 0.0;
            for (int i = 0; i < n; i++) dot += q[i][k] * q[i][j];
            r[k][j] = dot;
            for (int i = 0; i < n; i++) q[i][j] -= dot * q[i][k];
        }
    }

    double qty[3] = {0.0, 0.0, 0.0};
    for (int j = 0; j < p; j++) {
        double s = 0.0;
        for (int i = 0; i < n; i++) s += q[i][j] * b[i];
        qty[j] = s;
    }

    for (int i = p - 1; i >= 0; i--) {
        double s = qty[i];
        for (int j = i + 1; j < p; j++) s -= r[i][j] * beta[j];
        if (fabs(r[i][i]) < 1e-12) return false;
        beta[i] = s / r[i][i];
    }
    return true;
}

#define MODEL_OFFSET 1
#define MODEL_DIAG   2
#define MODEL_FULL   3

static bool solve_alignment_axis(int n, int model, int which_axis,
                                 const double *x, const double *y,
                                 const double *obs, double *beta) {
    double A[9][3];
    memset(A, 0, sizeof(A));
    int p = (model == MODEL_OFFSET) ? 1 : ((model == MODEL_DIAG) ? 2 : 3);

    for (int i = 0; i < n; i++) {
        if (model == MODEL_OFFSET) {
            A[i][0] = 1.0;
        } else if (model == MODEL_DIAG) {
            A[i][0] = 1.0;
            A[i][1] = (which_axis == OS_AXIS_RA) ? x[i] : y[i];
        } else {
            A[i][0] = 1.0;
            A[i][1] = x[i];
            A[i][2] = y[i];
        }
    }
    return qr_solve_fixed(n, p, A, obs, beta);
}

static bool alignment_3star_non_degenerate(const double *x, const double *y, int n) {
    if (n != 3) return true;
    double det = x[0] * (y[1] - y[2]) - x[1] * (y[0] - y[2]) + x[2] * (y[0] - y[1]);
    double scale = fabs(x[0]) + fabs(x[1]) + fabs(x[2]) +
                   fabs(y[0]) + fabs(y[1]) + fabs(y[2]) + 1.0;
    return fabs(det) > (1e-9 * scale);
}

static float compute_alignment_residual(void) {
    double sum = 0.0;
    int n = (int)g_alignment_star_count;
    for (int i = 0; i < n; i++) {
        double x = g_alignment_entries[i].ra_hours * OS_RA_ARCSEC_PER_HOUR;
        double y = g_alignment_entries[i].dec_degrees * OS_DEC_ARCSEC_PER_DEG;
        double pred0 = g_solution[0] + g_solution[1] * x + g_solution[2] * y;
        double pred1 = g_solution[3] + g_solution[4] * x + g_solution[5] * y;
        double dr = pred0 - (double)g_alignment_entries[i].ra_steps;
        double dd = pred1 - (double)g_alignment_entries[i].dec_steps;
        double arc0 = dr / OS_DEFAULT_STEPS_PER_ARCSEC;
        double arc1 = dd / OS_DEFAULT_STEPS_PER_ARCSEC;
        sum += (arc0 * arc0) + (arc1 * arc1);
    }
    return (float)sqrt(sum / (double)n);
}

static uint32_t manual_speed_frequency_hz(os_speed_level_t speed) {
    switch (speed) {
        case OS_SPEED_SLOW:
            return 500u;
        case OS_SPEED_MEDIUM:
            return 2000u;
        case OS_SPEED_FAST:
            return 8000u;
        case OS_SPEED_CUSTOM:
            return (uint32_t)(g_manual_custom_speed_arcsec_per_sec * OS_DEFAULT_STEPS_PER_ARCSEC);
        default:
            return 0u;
    }
}

/* ================= Public domain API ================= */

os_error_t os_init(void) {
    os_error_t first_error = OS_ERR_NONE;
    os_error_t err = OS_ERR_NONE;

    g_initialized = false;
    g_system_state = OS_STATE_INITIALIZING;

    g_goto_active = false;
    g_park_active = false;
    g_park_moving = false;
    g_manual_active = false;
    memset(&g_guide_pulse, 0, sizeof(g_guide_pulse));
    g_guide_remaining_ms = 0u;
    g_guide_rate_fraction = 0.5f;
    g_tracking_enabled = false;
    g_tracking_rate = OS_TRACK_RATE_SIDEREAL;
    g_tracking_custom_factor = 1.0f;
    g_alignment_star_count = 0u;
    g_alignment_computed = false;
    g_alignment_residual_arcsec = 0.0f;
    memset(g_alignment_entries, 0, sizeof(g_alignment_entries));
    memset(&g_calibration, 0, sizeof(g_calibration));
    memset(&g_site, 0, sizeof(g_site));
    g_gps_locked = false;
    g_pec_enabled = false;
    memset(&g_pec_table, 0, sizeof(g_pec_table));
    g_pec_table.valid = false;
    g_manual_custom_speed_arcsec_per_sec = 100.0f;

    g_park_position.ra_hours = 0.0f;
    g_park_position.dec_degrees = 90.0f;
    default_solution();

    err = os_hal_nvm_init();
    if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) first_error = err;

    nvm_cal_record_t cal_rec;
    memset(&cal_rec, 0, sizeof(cal_rec));
    err = os_hal_nvm_read(0u, (uint8_t *)&cal_rec, (uint16_t)sizeof(cal_rec));
    if (err == OS_ERR_NONE && cal_rec.magic == OS_NVM_CAL_MAGIC) {
        size_t skip = offsetof(nvm_cal_record_t, checksum);
        if (record_checksum_ok((const uint8_t *)&cal_rec, sizeof(cal_rec), skip,
                               cal_rec.checksum)) {
            g_calibration = cal_rec.calib;
            memcpy(g_solution, cal_rec.solution, sizeof(g_solution));
        } else if (first_error == OS_ERR_NONE) {
            first_error = OS_ERR_NVM_FAULT;
        }
    }

    nvm_cfg_record_t cfg_rec;
    memset(&cfg_rec, 0, sizeof(cfg_rec));
    err = os_hal_nvm_read(OS_NVM_CONFIG_SIZE_BYTES, (uint8_t *)&cfg_rec,
                          (uint16_t)sizeof(cfg_rec));
    if (err == OS_ERR_NONE && cfg_rec.magic == OS_NVM_CFG_MAGIC) {
        size_t skip = offsetof(nvm_cfg_record_t, checksum);
        if (record_checksum_ok((const uint8_t *)&cfg_rec, sizeof(cfg_rec), skip,
                               cfg_rec.checksum)) {
            g_park_position.ra_hours = cfg_rec.park_ra_hours;
            g_park_position.dec_degrees = cfg_rec.park_dec_degrees;
        } else if (first_error == OS_ERR_NONE) {
            first_error = OS_ERR_NVM_FAULT;
        }
    }

    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        err = os_hal_comm_init(ch);
        if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) first_error = err;
    }

    err = os_hal_motor_init(OS_AXIS_RA);
    if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) first_error = err;
    err = os_hal_motor_init(OS_AXIS_DEC);
    if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) first_error = err;
    (void)os_hal_motor_enable(OS_AXIS_RA, false);
    (void)os_hal_motor_enable(OS_AXIS_DEC, false);

    err = os_hal_gps_init();
    if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) first_error = err;
    err = os_hal_rtc_init();
    if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) first_error = err;
    err = os_hal_limit_init();
    if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) first_error = err;
    err = os_hal_timer_motor_init();
    if (err != OS_ERR_NONE && first_error == OS_ERR_NONE) first_error = err;

    os_site_info_t gps_tmp;
    memset(&gps_tmp, 0, sizeof(gps_tmp));
    err = os_hal_gps_poll(&gps_tmp);
    if (err == OS_ERR_NONE && gps_tmp.valid) {
        g_site = gps_tmp;
        g_gps_locked = true;
        (void)os_hal_rtc_set(gps_tmp.utc_epoch_seconds);
    } else {
        g_site.latitude_degrees = 0.0f;
        g_site.longitude_degrees = 0.0f;
        g_site.elevation_metres = 0.0f;
        g_site.valid = false;
        uint32_t rtc_time = 0u;
        if (os_hal_rtc_read(&rtc_time) == OS_ERR_NONE) {
            g_site.utc_epoch_seconds = rtc_time;
        }
        g_gps_locked = false;
    }

    g_tracking_rate = OS_TRACK_RATE_SIDEREAL;
    g_tracking_custom_factor = 1.0f;
    g_tracking_enabled = false;
    g_system_state = OS_STATE_IDLE_TRACKING;
    g_initialized = true;
    return first_error;
}

void os_loop_iteration(void) {
    if (!g_initialized) return;

    update_time_and_site();
    process_commands();
    update_active_frequencies();
    check_limits_and_safety();
    simulate_motor_pulses();
    advance_motion_state();
}

static os_error_t parse_ra_string(const char *body, float *ra_hours) {
    if (body == NULL || ra_hours == NULL || body[0] != 'r') return OS_ERR_COMMAND_FORMAT;
    float h = 0.0f, m = 0.0f, s = 0.0f;
    if (sscanf(body + 1, "%f:%f:%f", &h, &m, &s) != 3) return OS_ERR_COMMAND_FORMAT;
    if (h < 0.0f || h > 24.0f) return OS_ERR_INVALID_ARGUMENT;
    if (m < 0.0f || m >= 60.0f) return OS_ERR_INVALID_ARGUMENT;
    if (s < 0.0f || s >= 60.0f) return OS_ERR_INVALID_ARGUMENT;
    float ra = h + (m / 60.0f) + (s / 3600.0f);
    if (ra < OS_RA_MIN_HOURS || ra > OS_RA_MAX_HOURS) return OS_ERR_INVALID_ARGUMENT;
    *ra_hours = ra;
    return OS_ERR_NONE;
}

static os_error_t parse_dec_string(const char *body, float *dec_degrees) {
    if (body == NULL || dec_degrees == NULL || body[0] != 'd') return OS_ERR_COMMAND_FORMAT;
    float d = 0.0f, m = 0.0f, s = 0.0f;
    if (sscanf(body + 1, "%f*%f:%f", &d, &m, &s) != 3) return OS_ERR_COMMAND_FORMAT;
    if (d < OS_DEC_MIN_DEG || d > OS_DEC_MAX_DEG) return OS_ERR_INVALID_ARGUMENT;
    if (m < 0.0f || m >= 60.0f) return OS_ERR_INVALID_ARGUMENT;
    if (s < 0.0f || s >= 60.0f) return OS_ERR_INVALID_ARGUMENT;
    float sign = (d < 0.0f) ? -1.0f : 1.0f;
    float dec = sign * (fabsf(d) + (m / 60.0f) + (s / 3600.0f));
    if (dec < OS_DEC_MIN_DEG || dec > OS_DEC_MAX_DEG) return OS_ERR_INVALID_ARGUMENT;
    *dec_degrees = dec;
    return OS_ERR_NONE;
}

static void ra_string_from_coord(float ra_hours, char *output, size_t output_size) {
    double ra = (double)ra_hours;
    if (ra < 0.0) ra += 24.0;
    if (ra >= 24.0) ra = fmod(ra, 24.0);
    int hh = (int)ra;
    double m = (ra - (double)hh) * 60.0;
    int mm = (int)m;
    double s = (m - (double)mm) * 60.0;
    (void)snprintf(output, output_size, "%02d:%02d:%04.1f", hh, mm, s);
}

static void dec_string_from_coord(float dec_degrees, char *output, size_t output_size) {
    double d = (double)dec_degrees;
    char sign = d < 0.0 ? '-' : '+';
    d = fabs(d);
    int deg = (int)d;
    double m = (d - (double)deg) * 60.0;
    int mm = (int)m;
    double s = (m - (double)mm) * 60.0;
    (void)snprintf(output, output_size, "%c%02d*%02d:%04.1f", sign, deg, mm, s);
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        if (reply_length != NULL) *reply_length = 0u;
        return OS_ERR_INVALID_ARGUMENT;
    }
    *reply_length = 0u;
    if (length == 0u || source_channel > OS_CHANNEL_ETHERNET || reply_buffer_size == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length > OS_MAX_COMMAND_LENGTH) {
        (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "ERR#");
        return OS_ERR_COMMAND_FORMAT;
    }

    size_t cl = length;
    while (cl > 0u && (command[cl - 1u] == '\n' || command[cl - 1u] == '\r')) cl--;
    if (cl == 0u || command[0] != OS_LX200_CMD_PREFIX ||
        command[cl - 1u] != OS_LX200_CMD_SUFFIX) {
        (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "ERR#");
        return OS_ERR_COMMAND_FORMAT;
    }

    char cmd[OS_MAX_COMMAND_LENGTH + 1u];
    memcpy(cmd, command, cl);
    cmd[cl] = '\0';
    size_t body_len = cl - 2u;
    char *body = cmd + 1;
    body[body_len] = '\0';

    os_error_t result = OS_ERR_NONE;
    os_equatorial_coord_t eq;
    char temp[OS_MAX_REPLY_LENGTH];

    if (body_len == 0u) {
        result = OS_ERR_COMMAND_FORMAT;
        (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "ERR#");
    } else if (body[0] == 'G') {
        if (strcmp(body, "VP") == 0) {
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length,
                               "%u.%u.%u#", OS_FIRMWARE_VERSION_MAJOR,
                               OS_FIRMWARE_VERSION_MINOR, OS_FIRMWARE_VERSION_PATCH);
        } else if (strcmp(body, "R") == 0) {
            (void)os_query_coordinates(&eq);
            ra_string_from_coord(eq.ra_hours, temp, sizeof(temp));
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "%s#", temp);
        } else if (strcmp(body, "D") == 0) {
            (void)os_query_coordinates(&eq);
            dec_string_from_coord(eq.dec_degrees, temp, sizeof(temp));
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "%s#", temp);
        } else if (strcmp(body, "S") == 0) {
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "%d#",
                               (int)g_system_state);
        } else if (strcmp(body, "g") == 0) {
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length,
                               "%+.4f;%+.4f;%.1f;%lu;%u#",
                               (double)g_site.latitude_degrees,
                               (double)g_site.longitude_degrees,
                               (double)g_site.elevation_metres,
                               (unsigned long)g_site.utc_epoch_seconds,
                               (unsigned int)(g_site.valid ? 1u : 0u));
        } else if (strcmp(body, "G") == 0) {
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "%u#",
                               g_gps_locked ? 1u : 0u);
        } else if (strcmp(body, "MS") == 0) {
            bool moving = false;
            (void)os_query_is_moving(&moving);
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "%u#",
                               moving ? 1u : 0u);
        } else {
            result = OS_ERR_COMMAND_FORMAT;
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "ERR#");
        }
    } else if (body[0] == 'M') {
        if (strcmp(body, "S") == 0) {
            result = os_goto_equatorial(g_last_equ_target);
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length,
                               result == OS_ERR_NONE ? "OK#" : "ERR#");
        } else if (strcmp(body, "e") == 0) {
            result = os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length,
                               result == OS_ERR_NONE ? "OK#" : "ERR#");
        } else if (strcmp(body, "w") == 0) {
            result = os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length,
                               result == OS_ERR_NONE ? "OK#" : "ERR#");
        } else if (strcmp(body, "n") == 0) {
            result = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length,
                               result == OS_ERR_NONE ? "OK#" : "ERR#");
        } else if (strcmp(body, "s") == 0) {
            result = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length,
                               result == OS_ERR_NONE ? "OK#" : "ERR#");
        } else if (strcmp(body, "Q") == 0) {
            result = os_move_stop();
            if (result == OS_ERR_INVALID_STATE) {
                result = os_goto_abort();
            }
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length,
                               result == OS_ERR_NONE ? "OK#" : "ERR#");
        } else {
            result = OS_ERR_COMMAND_FORMAT;
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "ERR#");
        }
    } else if (body[0] == 'S') {
        if (body_len > 1u && body[1] == 'r') {
            float ra = 0.0f;
            result = parse_ra_string(body, &ra);
            if (result == OS_ERR_NONE) {
                g_last_equ_target.ra_hours = ra;
                (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "OK#");
            } else {
                (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "ERR#");
            }
        } else if (body_len > 1u && body[1] == 'd') {
            float dec = 0.0f;
            result = parse_dec_string(body, &dec);
            if (result == OS_ERR_NONE) {
                g_last_equ_target.dec_degrees = dec;
                (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "OK#");
            } else {
                (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "ERR#");
            }
        } else {
            result = OS_ERR_COMMAND_FORMAT;
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "ERR#");
        }
    } else if (body[0] == 'h') {
        if (strcmp(body, "P") == 0) {
            result = os_park();
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length,
                               result == OS_ERR_NONE ? "OK#" : "ERR#");
        } else if (strcmp(body, "O") == 0) {
            result = os_unpark();
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length,
                               result == OS_ERR_NONE ? "OK#" : "ERR#");
        } else {
            result = OS_ERR_COMMAND_FORMAT;
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "ERR#");
        }
    } else if (body[0] == 'A' && body_len >= 2u && body[1] == 'L') {
        os_align_mode_t mode = OS_ALIGN_3STAR;
        if (body[2] == '1') mode = OS_ALIGN_1STAR;
        else if (body[2] == '2') mode = OS_ALIGN_2STAR;
        else if (body[2] == '3') mode = OS_ALIGN_3STAR;
        else if (body[2] == 'N') mode = OS_ALIGN_NSTAR;
        else result = OS_ERR_COMMAND_FORMAT;
        if (result == OS_ERR_NONE) {
            result = os_align_begin(mode);
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length,
                               result == OS_ERR_NONE ? "OK#" : "ERR#");
        } else {
            (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "ERR#");
        }
    } else {
        result = OS_ERR_COMMAND_FORMAT;
        (void)reply_format(reply_buffer, reply_buffer_size, reply_length, "ERR#");
    }

    return result;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (!(target.ra_hours >= OS_RA_MIN_HOURS && target.ra_hours <= OS_RA_MAX_HOURS) ||
        !(target.dec_degrees >= OS_DEC_MIN_DEG && target.dec_degrees <= OS_DEC_MAX_DEG)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_last_equ_target = target;
    int32_t current0 = os_hal_motor_get_position(OS_AXIS_RA);
    int32_t current1 = os_hal_motor_get_position(OS_AXIS_DEC);
    int32_t target0 = (int32_t)ra_hours_to_steps((double)target.ra_hours);
    int32_t target1 = (int32_t)dec_degrees_to_steps((double)target.dec_degrees);
    g_goto_target_steps[0] = target0;
    g_goto_target_steps[1] = target1;

    int32_t rem0 = target0 - current0;
    int32_t rem1 = target1 - current1;
    if (iabs32(rem0) <= OS_GOTO_DEADBAND_STEPS && iabs32(rem1) <= OS_GOTO_DEADBAND_STEPS) {
        return OS_ERR_NONE;
    }

    g_goto_active = true;
    g_park_active = false;
    g_park_moving = false;
    g_system_state = OS_STATE_GOTO;
    update_goto_axis_frequencies(g_goto_target_steps);
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (!(target.azimuth_degrees >= 0.0f && target.azimuth_degrees <= 360.0f) ||
        !(target.altitude_degrees >= -90.0f && target.altitude_degrees <= 90.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    int32_t current0 = os_hal_motor_get_position(OS_AXIS_RA);
    int32_t current1 = os_hal_motor_get_position(OS_AXIS_DEC);
    int32_t target0 = (int32_t)((double)target.azimuth_degrees * OS_DEC_ARCSEC_PER_DEG *
                                OS_DEFAULT_STEPS_PER_ARCSEC);
    int32_t target1 = (int32_t)((double)target.altitude_degrees * OS_DEC_ARCSEC_PER_DEG *
                                OS_DEFAULT_STEPS_PER_ARCSEC);
    g_goto_target_steps[0] = target0;
    g_goto_target_steps[1] = target1;

    int32_t rem0 = target0 - current0;
    int32_t rem1 = target1 - current1;
    if (iabs32(rem0) <= OS_GOTO_DEADBAND_STEPS && iabs32(rem1) <= OS_GOTO_DEADBAND_STEPS) {
        return OS_ERR_NONE;
    }

    g_goto_active = true;
    g_park_active = false;
    g_park_moving = false;
    g_system_state = OS_STATE_GOTO;
    update_goto_axis_frequencies(g_goto_target_steps);
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (!g_goto_active) return OS_ERR_INVALID_STATE;
    g_goto_active = false;
    stop_axis(OS_AXIS_RA);
    stop_axis(OS_AXIS_DEC);
    g_system_state = OS_STATE_IDLE_TRACKING;
    g_tracking_enabled = true;
    update_active_frequencies();
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (rate < OS_TRACK_RATE_SIDEREAL || rate > OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM && !(custom_factor > 0.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_tracking_rate = rate;
    g_tracking_custom_factor = custom_factor;
    if (g_system_state == OS_STATE_IDLE_TRACKING) {
        set_tracking_motor_frequency();
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (rate == NULL || custom_factor == NULL) return OS_ERR_INVALID_ARGUMENT;
    *rate = g_tracking_rate;
    *custom_factor = g_tracking_custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    g_tracking_enabled = true;
    set_tracking_motor_frequency();
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    g_tracking_enabled = false;
    stop_axis(OS_AXIS_RA);
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST ||
        duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    bool new_is_dec = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);
    if (g_guide_pulse.active) {
        bool old_is_dec = g_guide_pulse.dec_priority;
        if (!new_is_dec && old_is_dec) {
            return OS_ERR_INVALID_STATE;
        }
    }

    uint8_t axis = new_is_dec ? OS_AXIS_DEC : OS_AXIS_RA;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    uint32_t base_hz = tracking_frequency_hz();
    if (base_hz == 0u) {
        base_hz = (uint32_t)(OS_SIDEREAL_RATE_ARCSEC_PER_SEC * OS_DEFAULT_STEPS_PER_ARCSEC);
    }
    uint32_t bias_hz = (uint32_t)((double)base_hz * (double)g_guide_rate_fraction + 0.5);
    if (bias_hz == 0u) bias_hz = 1u;

    memset(&g_guide_pulse, 0, sizeof(g_guide_pulse));
    g_guide_pulse.active = true;
    g_guide_pulse.duration_ms = duration_ms;
    g_guide_pulse.rate_fraction = g_guide_rate_fraction;
    g_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    g_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    g_guide_pulse.dec_priority = new_is_dec;
    g_guide_remaining_ms = duration_ms;

    g_motor[axis].enabled = true;
    g_motor[axis].direction = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_NORTH);
    g_motor[axis].frequency_hz = base_hz + bias_hz;
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (!(rate_fraction >= OS_GUIDE_RATE_MIN && rate_fraction <= OS_GUIDE_RATE_MAX)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_guide_rate_fraction = rate_fraction;
    if (g_guide_pulse.active) g_guide_pulse.rate_fraction = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (pulse == NULL) return OS_ERR_INVALID_ARGUMENT;
    *pulse = g_guide_pulse;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state == OS_STATE_GOTO || g_system_state == OS_STATE_MANUAL_MOTION ||
        g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    g_alignment_mode = mode;
    g_alignment_star_count = 0u;
    g_alignment_computed = false;
    g_alignment_residual_arcsec = 0.0f;
    memset(g_alignment_entries, 0, sizeof(g_alignment_entries));
    g_system_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (!(star_coord.ra_hours >= OS_RA_MIN_HOURS && star_coord.ra_hours <= OS_RA_MAX_HOURS) ||
        !(star_coord.dec_degrees >= OS_DEC_MIN_DEG && star_coord.dec_degrees <= OS_DEC_MAX_DEG)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_alignment_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }
    align_entry_t *entry = &g_alignment_entries[g_alignment_star_count++];
    entry->ra_hours = (double)star_coord.ra_hours;
    entry->dec_degrees = (double)star_coord.dec_degrees;
    entry->ra_steps = motor_pos.ra_steps;
    entry->dec_steps = motor_pos.dec_steps;
    g_alignment_computed = false;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    int min_stars = alignment_min_stars(g_alignment_mode);
    if ((int)g_alignment_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    double x[OS_CALIBRATION_MAX_STARS];
    double y[OS_CALIBRATION_MAX_STARS];
    double obs0[OS_CALIBRATION_MAX_STARS];
    double obs1[OS_CALIBRATION_MAX_STARS];
    memset(x, 0, sizeof(x));
    memset(y, 0, sizeof(y));
    memset(obs0, 0, sizeof(obs0));
    memset(obs1, 0, sizeof(obs1));

    int n = (int)g_alignment_star_count;
    for (int i = 0; i < n; i++) {
        x[i] = g_alignment_entries[i].ra_hours * OS_RA_ARCSEC_PER_HOUR;
        y[i] = g_alignment_entries[i].dec_degrees * OS_DEC_ARCSEC_PER_DEG;
        obs0[i] = (double)g_alignment_entries[i].ra_steps;
        obs1[i] = (double)g_alignment_entries[i].dec_steps;
    }

    double beta0[3] = {0.0, 0.0, 0.0};
    double beta1[3] = {0.0, 0.0, 0.0};
    bool ok = true;

    if (g_alignment_mode == OS_ALIGN_1STAR) {
        ok = solve_alignment_axis(n, MODEL_OFFSET, OS_AXIS_RA, x, y, obs0, beta0);
        if (ok) ok = solve_alignment_axis(n, MODEL_OFFSET, OS_AXIS_DEC, x, y, obs1, beta1);
        if (ok) {
            default_solution();
            g_solution[0] = beta0[0];
            g_solution[3] = beta1[0];
        }
    } else if (g_alignment_mode == OS_ALIGN_2STAR) {
        ok = solve_alignment_axis(n, MODEL_DIAG, OS_AXIS_RA, x, y, obs0, beta0);
        if (ok) ok = solve_alignment_axis(n, MODEL_DIAG, OS_AXIS_DEC, x, y, obs1, beta1);
        if (ok) {
            default_solution();
            g_solution[0] = beta0[0];
            g_solution[1] = beta0[1];
            g_solution[3] = beta1[0];
            g_solution[5] = beta1[1];
        }
    } else {
        if (n == 3 && !alignment_3star_non_degenerate(x, y, n)) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        ok = solve_alignment_axis(n, MODEL_FULL, OS_AXIS_RA, x, y, obs0, beta0);
        if (ok) ok = solve_alignment_axis(n, MODEL_FULL, OS_AXIS_DEC, x, y, obs1, beta1);
        if (ok) {
            g_solution[0] = beta0[0];
            g_solution[1] = beta0[1];
            g_solution[2] = beta0[2];
            g_solution[3] = beta1[0];
            g_solution[4] = beta1[1];
            g_solution[5] = beta1[2];
        }
    }

    if (!ok) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    float residual = compute_alignment_residual();
    if ((g_alignment_mode == OS_ALIGN_NSTAR && n > 3) ||
        (g_alignment_mode == OS_ALIGN_3STAR && n == 3)) {
        if ((g_alignment_mode == OS_ALIGN_NSTAR && n > 3) &&
            residual > OS_ALIGN_MAX_RESIDUAL_ARCSEC) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        if (g_alignment_mode == OS_ALIGN_3STAR && n == 3) {
            residual = 0.0f;
        }
    }
    g_alignment_residual_arcsec = residual;

    g_calibration.valid = true;
    g_calibration.matrix_ra_to_ra = (float)g_solution[1];
    g_calibration.matrix_ra_to_dec = (float)g_solution[2];
    g_calibration.matrix_dec_to_ra = (float)g_solution[4];
    g_calibration.matrix_dec_to_dec = (float)g_solution[5];
    g_calibration.offset_ra_arcsec = (float)g_solution[0];
    g_calibration.offset_dec_arcsec = (float)g_solution[3];

    os_error_t save_err = save_calibration_to_nvm();
    if (save_err != OS_ERR_NONE) {
        memset(&g_calibration, 0, sizeof(g_calibration));
        default_solution();
        g_alignment_computed = false;
        return OS_ERR_NVM_FAULT;
    }

    g_alignment_computed = true;
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (!g_alignment_computed) return OS_ERR_INVALID_STATE;
    *residual_arcsec = g_alignment_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    if (g_system_state != OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE;
    g_alignment_star_count = 0u;
    g_alignment_computed = false;
    g_alignment_residual_arcsec = 0.0f;
    memset(g_alignment_entries, 0, sizeof(g_alignment_entries));
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (g_system_state == OS_STATE_PARKED) return OS_ERR_NONE;
    if (g_system_state == OS_STATE_FAULT) return OS_ERR_INVALID_STATE;

    g_park_target_steps[0] = (int32_t)ra_hours_to_steps((double)g_park_position.ra_hours);
    g_park_target_steps[1] = (int32_t)dec_degrees_to_steps((double)g_park_position.dec_degrees);
    int32_t pos0 = os_hal_motor_get_position(OS_AXIS_RA);
    int32_t pos1 = os_hal_motor_get_position(OS_AXIS_DEC);
    g_goto_active = false;
    g_park_active = true;
    g_park_moving = true;
    g_system_state = OS_STATE_GOTO;

    if (iabs32(g_park_target_steps[0] - pos0) > OS_GOTO_DEADBAND_STEPS ||
        iabs32(g_park_target_steps[1] - pos1) > OS_GOTO_DEADBAND_STEPS) {
        update_goto_axis_frequencies(g_park_target_steps);
    }
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (g_system_state != OS_STATE_PARKED) return OS_ERR_INVALID_STATE;
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        (void)os_hal_comm_init(ch);
    }
    uint32_t time_value = 0u;
    if (g_gps_locked && g_site.valid) {
        (void)os_hal_rtc_set(g_site.utc_epoch_seconds);
    } else {
        (void)os_hal_rtc_read(&time_value);
    }
    g_motor[OS_AXIS_RA].enabled = true;
    g_motor[OS_AXIS_DEC].enabled = true;
    g_tracking_enabled = true;
    g_system_state = OS_STATE_IDLE_TRACKING;
    update_active_frequencies();
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!(park_pos.ra_hours >= OS_RA_MIN_HOURS && park_pos.ra_hours <= OS_RA_MAX_HOURS) ||
        !(park_pos.dec_degrees >= OS_DEC_MIN_DEG && park_pos.dec_degrees <= OS_DEC_MAX_DEG)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_park_position = park_pos;
    os_error_t err = save_park_config_to_nvm();
    if (err != OS_ERR_NONE) return OS_ERR_NVM_FAULT;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST ||
        speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t axis = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH)
                       ? OS_AXIS_DEC
                       : OS_AXIS_RA;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_manual_active = true;
    g_manual_direction = direction;
    g_manual_speed = speed;
    g_system_state = OS_STATE_MANUAL_MOTION;

    g_motor[axis].enabled = true;
    g_motor[axis].direction = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_EAST);
    g_motor[axis].frequency_hz = manual_speed_frequency_hz(speed);
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (g_system_state != OS_STATE_MANUAL_MOTION) return OS_ERR_INVALID_STATE;
    stop_axis(OS_AXIS_RA);
    stop_axis(OS_AXIS_DEC);
    g_manual_active = false;
    g_system_state = OS_STATE_IDLE_TRACKING;
    g_tracking_enabled = true;
    update_active_frequencies();
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (!(arcsec_per_sec > 0.0f)) return OS_ERR_INVALID_ARGUMENT;
    g_manual_custom_speed_arcsec_per_sec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (state == NULL) return OS_ERR_INVALID_ARGUMENT;
    *state = g_system_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (coord == NULL) return OS_ERR_INVALID_ARGUMENT;
    int32_t ra_steps = os_hal_motor_get_position(OS_AXIS_RA);
    int32_t dec_steps = os_hal_motor_get_position(OS_AXIS_DEC);
    steps_to_equatorial(ra_steps, dec_steps, &coord->ra_hours, &coord->dec_degrees);
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (site == NULL) return OS_ERR_INVALID_ARGUMENT;
    *site = g_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (pos == NULL) return OS_ERR_INVALID_ARGUMENT;
    pos->ra_steps = os_hal_motor_get_position(OS_AXIS_RA);
    pos->dec_steps = os_hal_motor_get_position(OS_AXIS_DEC);
    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor, uint8_t *patch) {
    if (major == NULL || minor == NULL || patch == NULL) return OS_ERR_INVALID_ARGUMENT;
    *major = OS_FIRMWARE_VERSION_MAJOR;
    *minor = OS_FIRMWARE_VERSION_MINOR;
    *patch = OS_FIRMWARE_VERSION_PATCH;
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *moving) {
    if (moving == NULL) return OS_ERR_INVALID_ARGUMENT;
    bool active = (g_system_state == OS_STATE_GOTO) ||
                  (g_system_state == OS_STATE_MANUAL_MOTION) ||
                  g_guide_pulse.active;
    if (!active) {
        for (uint8_t axis = OS_AXIS_RA; axis <= OS_AXIS_DEC; axis++) {
            if (g_motor[axis].enabled && g_motor[axis].frequency_hz > 0u) {
                active = true;
                break;
            }
        }
    }
    *moving = active;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (locked == NULL) return OS_ERR_INVALID_ARGUMENT;
    *locked = g_gps_locked;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    g_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL) return OS_ERR_INVALID_ARGUMENT;
    memcpy(&g_pec_table, table, sizeof(g_pec_table));
    g_pec_table.valid = true;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (table == NULL) return OS_ERR_INVALID_ARGUMENT;
    *table = g_pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (!(worm_phase_deg >= 0.0f && worm_phase_deg <= 360.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int idx;
    if (worm_phase_deg >= 360.0f) {
        idx = 0;
    } else {
        idx = (int)worm_phase_deg;
    }
    if (idx < 0 || idx >= OS_PEC_TABLE_SIZE) idx = 0;
    g_pec_table.corrections[idx] = error_arcsec;
    g_pec_table.valid = true;
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (calib == NULL) return OS_ERR_INVALID_ARGUMENT;
    *calib = g_calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    memset(&g_calibration, 0, sizeof(g_calibration));
    g_calibration.valid = false;
    default_solution();
    os_error_t err = save_calibration_to_nvm();
    if (err != OS_ERR_NONE) return OS_ERR_NVM_FAULT;
    return OS_ERR_NONE;
}

/* ================= HAL adapters ================= */

os_error_t os_hal_motor_init(uint8_t axis) {
    if (!axis_valid(axis)) return OS_ERR_INVALID_ARGUMENT;
    memset(&g_motor[axis], 0, sizeof(g_motor[axis]));
    g_motor[axis].initialized = true;
    g_motor[axis].enabled = false;
    g_motor[axis].frequency_hz = 0u;
    g_motor[axis].direction = false;
    g_motor[axis].fault = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (!axis_valid(axis)) return OS_ERR_INVALID_ARGUMENT;
    g_motor[axis].frequency_hz = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (!axis_valid(axis)) return OS_ERR_INVALID_ARGUMENT;
    g_motor[axis].direction = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (!axis_valid(axis)) return OS_ERR_INVALID_ARGUMENT;
    g_motor[axis].enabled = enable;
    if (!enable) g_motor[axis].frequency_hz = 0u;
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (!axis_valid(axis)) return 0;
    return g_motor[axis].position_steps;
}

os_error_t os_hal_gps_init(void) {
    g_gps_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (g_gps_has_valid) {
        *site = g_gps_site;
        site->valid = true;
        return OS_ERR_NONE;
    }
    memset(site, 0, sizeof(*site));
    site->valid = false;
    return OS_ERR_GPS_NO_SIGNAL;
}

os_error_t os_hal_rtc_init(void) {
    g_rtc_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (!g_rtc_initialized) return OS_ERR_TIMEOUT;
    *utc_epoch_seconds = g_rtc_time;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    if (!g_rtc_initialized) return OS_ERR_TIMEOUT;
    g_rtc_time = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    g_limits_initialized = true;
    g_limit_triggered[OS_AXIS_RA] = false;
    g_limit_triggered[OS_AXIS_DEC] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (!axis_valid(axis)) return true;
    return g_limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void) {
    g_nvm_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (data == NULL) return OS_ERR_INVALID_ARGUMENT;
    if ((uint32_t)offset + (uint32_t)length > OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    (void)g_nvm_initialized;
    memcpy(data, &g_nvm_image[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (data == NULL) return OS_ERR_INVALID_ARGUMENT;
    if ((uint32_t)offset + (uint32_t)length > OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(&g_nvm_image[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET) return OS_ERR_INVALID_ARGUMENT;
    memset(&g_comm[channel], 0, sizeof(g_comm[channel]));
    g_comm[channel].initialized = true;
    g_partial_len[channel] = 0u;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET || !g_comm[channel].initialized) return 0;
    return (int16_t)g_comm[channel].rx_len;
}

char os_hal_comm_read(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET || !g_comm[channel].initialized) return '\0';
    if (g_comm[channel].rx_len == 0u) return '\0';
    char c = g_comm[channel].rx[g_comm[channel].rx_head];
    g_comm[channel].rx_head++;
    if (g_comm[channel].rx_head >= (uint16_t)sizeof(g_comm[channel].rx)) {
        g_comm[channel].rx_head = 0u;
    }
    g_comm[channel].rx_len--;
    return c;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (channel > OS_CHANNEL_ETHERNET) return OS_ERR_INVALID_ARGUMENT;
    if (!g_comm[channel].initialized) return OS_ERR_NOT_SUPPORTED;
    if (data == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (length == 0u) return OS_ERR_NONE;
    if ((size_t)g_comm[channel].tx_len + length > sizeof(g_comm[channel].tx)) {
        return OS_ERR_TIMEOUT;
    }
    memcpy(&g_comm[channel].tx[g_comm[channel].tx_len], data, length);
    g_comm[channel].tx_len = (uint16_t)(g_comm[channel].tx_len + length);
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) {
    g_buzzer_pending = true;
    g_buzzer_duration_ms = duration_ms;
    g_buzzer_count = count;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void) {
    g_timer_motor_initialized = true;
    return OS_ERR_NONE;
}
#include "6_generated_code.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>

#define OS_NVM_TOTAL_SIZE 2048u
#define OS_NVM_OFFSET_CALIBRATION 0u
#define OS_NVM_OFFSET_CONFIG 256u
#define OS_NVM_OFFSET_PEC 512u
#define OS_LOOP_TICK_MS 10UL
#define OS_MOTOR_FREQ_LIMIT_HZ 1000000UL
#define OS_STEPS_PER_RA_HOUR 10.0f
#define OS_STEPS_PER_DEC_DEG 10.0f
#define OS_MANUAL_SLOW_HZ 10UL
#define OS_MANUAL_MEDIUM_HZ 50UL
#define OS_MANUAL_FAST_HZ 200UL

typedef struct {
    os_equatorial_coord_t coord;
    os_motor_position_t motor;
} align_point_t;

typedef struct {
    os_equatorial_coord_t pos;
    uint8_t custom_set;
} persist_park_t;

static os_state_t s_state = OS_STATE_INITIALIZING;
static bool s_goto_motion_active;
static bool s_manual_motion_active;
static bool s_park_motion_active;
static bool s_tracking_enabled;
static os_track_rate_t s_tracking_rate = OS_TRACK_RATE_SIDEREAL;
static float s_tracking_custom_factor = 1.0f;
static float s_residual_arcsec;
static bool s_calib_residual_computed;
static uint8_t s_align_star_count;
static os_align_mode_t s_align_mode = OS_ALIGN_1STAR;
static bool s_align_active;
static os_guide_pulse_t s_guide_pulse;
static uint32_t s_guide_elapsed_ms;
static float s_guide_rate_fraction = 0.5f;
static os_equatorial_coord_t s_park_pos = {0.0f, 90.0f};
static bool s_custom_park_set;
static os_calibration_t s_calibration;
static os_pec_table_t s_pec_table;
static bool s_pec_enabled;
static float s_worm_phase_deg;
static int32_t s_goto_target_steps[2];
static int32_t s_park_target_steps[2];
static uint8_t s_manual_axis;
static bool s_manual_forward;
static os_speed_level_t s_manual_speed = OS_SPEED_MEDIUM;
static float s_manual_custom_speed_arcsec_per_sec = 15.0f;
static os_site_info_t s_site = {0.0f, 0.0f, 0.0f, 0u, true};
static align_point_t s_align_points[OS_CALIBRATION_MAX_STARS];
static os_equatorial_coord_t s_last_equ_target = {0.0f, 90.0f};

static bool s_motor_initialized[2];
static bool s_motor_enabled[2];
static uint32_t s_motor_frequency_hz[2];
static bool s_motor_direction[2];
static int32_t s_motor_position_steps[2];
static bool s_motor_driver_fault[2];

static bool s_gps_initialized;
static bool s_gps_valid;
static os_site_info_t s_gps_site;
static bool s_rtc_initialized;
static bool s_rtc_valid;
static uint32_t s_rtc_utc;
static bool s_limit_initialized;
static bool s_limit_triggered[2];
static bool s_motor_timer_initialized;
static uint8_t s_nvm[OS_NVM_TOTAL_SIZE];

static bool s_comm_initialized[4];
static int16_t s_comm_rx_len[4];
static char s_comm_rx[4][OS_MAX_COMMAND_LENGTH + 1];
static char s_comm_tx[4][OS_MAX_REPLY_LENGTH * 2];
static uint16_t s_comm_tx_len[4];

static char s_cmd_buf[4][OS_MAX_COMMAND_LENGTH + 1];
static size_t s_cmd_len[4];

static bool s_buzzer_pending;
static uint16_t s_buzzer_duration_ms;
static uint8_t s_buzzer_count;

static uint32_t s_loop_tick_ms;

static void set_reply(char *buf, size_t size, size_t *len, const char *fmt, ...) {
    if (buf == NULL || len == NULL || size == 0u) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, size, fmt, ap);
    va_end(ap);
    if (n < 0) {
        n = 0;
    }
    if ((size_t)n >= size) {
        n = (int)(size - 1u);
    }
    buf[n] = '\0';
    *len = (size_t)n;
}

static int32_t ra_to_steps(float ra_hours) {
    return (int32_t)(ra_hours * OS_STEPS_PER_RA_HOUR);
}

static int32_t dec_to_steps(float dec_degrees) {
    return (int32_t)(dec_degrees * OS_STEPS_PER_DEC_DEG);
}

static int32_t az_to_steps(float az_degrees) {
    return (int32_t)(az_degrees * 10.0f);
}

static int32_t alt_to_steps(float alt_degrees) {
    return (int32_t)(alt_degrees * 10.0f);
}

static float parse_angle_string(const char *s) {
    if (s == NULL) {
        return 0.0f;
    }
    char tmp[OS_MAX_COMMAND_LENGTH];
    size_t n = 0u;
    while (*s != '\0' && n < sizeof(tmp) - 1u) {
        char c = *s++;
        if (c == ':' || c == '*' || c == ',' || c == ' ') {
            tmp[n++] = ' ';
        } else {
            tmp[n++] = c;
        }
    }
    tmp[n] = '\0';
    double values[3] = {0.0, 0.0, 0.0};
    int parts = 0;
    char *p = tmp;
    while (*p != '\0') {
        while (*p == ' ') {
            ++p;
        }
        if (*p == '\0') {
            break;
        }
        char *end = NULL;
        double v = strtod(p, &end);
        if (end == p) {
            break;
        }
        if (parts < 3) {
            values[parts++] = v;
        }
        p = end;
    }
    if (parts == 0) {
        return 0.0f;
    }
    double sign = 1.0;
    if (values[0] < 0.0) {
        sign = -1.0;
        values[0] = -values[0];
    }
    double result = values[0];
    if (parts > 1) {
        result += values[1] / 60.0;
    }
    if (parts > 2) {
        result += values[2] / 3600.0;
    }
    return (float)(sign * result);
}

static int32_t clamp_to_int32(double v) {
    if (v > 2147483647.0) {
        return 2147483647;
    }
    if (v < -2147483648.0) {
        return -2147483648;
    }
    return (int32_t)v;
}

static void compute_goto_target_steps(os_equatorial_coord_t target, int32_t steps[2]) {
    if (s_calibration.valid) {
        double ra_arcsec = (double)target.ra_hours * 3600.0;
        double dec_arcsec = (double)target.dec_degrees * 3600.0;
        double m_ra = (double)s_calibration.matrix_ra_to_ra * ra_arcsec +
                      (double)s_calibration.matrix_ra_to_dec * dec_arcsec +
                      (double)s_calibration.offset_ra_arcsec;
        double m_dec = (double)s_calibration.matrix_dec_to_ra * ra_arcsec +
                       (double)s_calibration.matrix_dec_to_dec * dec_arcsec +
                       (double)s_calibration.offset_dec_arcsec;
        steps[0] = clamp_to_int32(m_ra);
        steps[1] = clamp_to_int32(m_dec);
    } else {
        steps[0] = ra_to_steps(target.ra_hours);
        steps[1] = dec_to_steps(target.dec_degrees);
    }
}

static int32_t loop_step_increment(void) {
    uint32_t steps = (uint32_t)((OS_MANUAL_FAST_HZ * OS_LOOP_TICK_MS) / 1000u);
    if (steps == 0u) {
        steps = 1u;
    }
    return (int32_t)steps;
}

static int32_t step_toward(int32_t current, int32_t target, int32_t increment) {
    if (increment <= 0) {
        return current;
    }
    int64_t diff = (int64_t)target - (int64_t)current;
    if (diff == 0) {
        return current;
    }
    if (diff > 0) {
        return (diff < increment) ? current + (int32_t)diff : current + increment;
    }
    return (diff > -increment) ? current + (int32_t)diff : current - increment;
}

static bool starts_with(const char *s, const char *prefix) {
    if (s == NULL || prefix == NULL) {
        return false;
    }
    while (*prefix != '\0') {
        if (*s != *prefix) {
            return false;
        }
        ++s;
        ++prefix;
    }
    return true;
}

static int pec_index_from_phase(float phase_deg) {
    if (phase_deg >= 360.0f) {
        return 0;
    }
    if (phase_deg < 0.0f) {
        return 0;
    }
    int idx = (int)phase_deg;
    if (idx >= OS_PEC_TABLE_SIZE) {
        idx = OS_PEC_TABLE_SIZE - 1;
    }
    return idx;
}

static uint32_t effective_tracking_frequency_hz(void) {
    double base = (double)OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
    double factor = 1.0;
    switch (s_tracking_rate) {
        case OS_TRACK_RATE_SIDEREAL:
            factor = 1.0;
            break;
        case OS_TRACK_RATE_LUNAR:
            factor = (double)OS_LUNAR_RATE_FACTOR;
            break;
        case OS_TRACK_RATE_SOLAR:
            factor = (double)OS_SOLAR_RATE_FACTOR;
            break;
        case OS_TRACK_RATE_CUSTOM:
            factor = (s_tracking_custom_factor > 0.0f) ? (double)s_tracking_custom_factor : 1.0;
            break;
        default:
            factor = 1.0;
            break;
    }
    double freq = base * factor;
    if (s_pec_enabled && s_pec_table.valid) {
        int idx = pec_index_from_phase(s_worm_phase_deg);
        freq += (double)s_pec_table.corrections[idx];
    }
    if (freq < 0.0) {
        freq = 0.0;
    }
    if (freq > (double)OS_MOTOR_FREQ_LIMIT_HZ) {
        freq = (double)OS_MOTOR_FREQ_LIMIT_HZ;
    }
    return (uint32_t)freq;
}

static void apply_tracking_frequency(void) {
    if (!s_tracking_enabled) {
        (void)os_hal_motor_set_frequency(0u, 0u);
        (void)os_hal_motor_set_frequency(1u, 0u);
        return;
    }
    uint32_t freq = effective_tracking_frequency_hz();
    (void)os_hal_motor_set_frequency(0u, freq);
    (void)os_hal_motor_set_frequency(1u, 0u);
}

static uint32_t manual_frequency_hz(void) {
    switch (s_manual_speed) {
        case OS_SPEED_SLOW:
            return OS_MANUAL_SLOW_HZ;
        case OS_SPEED_MEDIUM:
            return OS_MANUAL_MEDIUM_HZ;
        case OS_SPEED_FAST:
            return OS_MANUAL_FAST_HZ;
        case OS_SPEED_CUSTOM:
        default:
            if (s_manual_custom_speed_arcsec_per_sec < 0.0f) {
                return 0u;
            }
            return (uint32_t)s_manual_custom_speed_arcsec_per_sec;
    }
}

static bool qr_solve_3(const double A[][3], const double b[], int n, double x[3]) {
    if (n < 3 || n > OS_CALIBRATION_MAX_STARS) {
        return false;
    }
    double q[9][3];
    double r[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    int i, j, k;
    for (j = 0; j < 3; ++j) {
        for (i = 0; i < n; ++i) {
            q[i][j] = A[i][j];
        }
        for (k = 0; k < j; ++k) {
            double dot = 0.0;
            for (i = 0; i < n; ++i) {
                dot += q[i][k] * q[i][j];
            }
            r[k][j] = dot;
            for (i = 0; i < n; ++i) {
                q[i][j] -= r[k][j] * q[i][k];
            }
        }
        double norm = 0.0;
        for (i = 0; i < n; ++i) {
            norm += q[i][j] * q[i][j];
        }
        norm = sqrt(norm);
        if (fabs(norm) < 1e-12) {
            return false;
        }
        r[j][j] = norm;
        for (i = 0; i < n; ++i) {
            q[i][j] /= norm;
        }
    }
    double c[3] = {0.0, 0.0, 0.0};
    for (k = 0; k < 3; ++k) {
        for (i = 0; i < n; ++i) {
            c[k] += q[i][k] * b[i];
        }
    }
    for (i = 2; i >= 0; --i) {
        double sum = c[i];
        for (j = i + 1; j < 3; ++j) {
            sum -= r[i][j] * x[j];
        }
        if (fabs(r[i][i]) < 1e-12) {
            return false;
        }
        x[i] = sum / r[i][i];
    }
    return true;
}

static float compute_residual_arcsec(const double *ra, const double *dec,
                                     const double *m_ra, const double *m_dec,
                                     int n, const double r_ra[3], const double r_dec[3]) {
    if (n <= 0) {
        return 0.0f;
    }
    double ss = 0.0;
    for (int i = 0; i < n; ++i) {
        double model_ra = r_ra[0] * ra[i] + r_ra[1] * dec[i] + r_ra[2];
        double model_dec = r_dec[0] * ra[i] + r_dec[1] * dec[i] + r_dec[2];
        double d_ra = m_ra[i] - model_ra;
        double d_dec = m_dec[i] - model_dec;
        ss += d_ra * d_ra + d_dec * d_dec;
    }
    return (float)sqrt(ss / (double)n);
}

os_error_t os_init(void) {
    s_state = OS_STATE_INITIALIZING;
    s_goto_motion_active = false;
    s_manual_motion_active = false;
    s_park_motion_active = false;
    s_tracking_enabled = false;
    s_calib_residual_computed = false;
    s_align_star_count = 0u;
    s_align_active = false;
    s_residual_arcsec = 0.0f;
    s_guide_pulse.active = false;
    s_guide_pulse.duration_ms = 0u;
    s_guide_pulse.rate_fraction = 0.5f;
    s_guide_pulse.direction_east = false;
    s_guide_pulse.direction_north = false;
    s_guide_pulse.dec_priority = false;
    s_guide_elapsed_ms = 0u;
    s_pec_enabled = false;
    s_tracking_rate = OS_TRACK_RATE_SIDEREAL;
    s_tracking_custom_factor = 1.0f;
    s_custom_park_set = false;
    s_park_pos.ra_hours = 0.0f;
    s_park_pos.dec_degrees = 90.0f;
    s_gps_valid = false;
    memset(&s_gps_site, 0, sizeof(s_gps_site));
    memset(s_align_points, 0, sizeof(s_align_points));
    memset(&s_calibration, 0, sizeof(s_calibration));
    memset(&s_pec_table, 0, sizeof(s_pec_table));

    if (os_hal_nvm_init() != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return OS_ERR_NVM_FAULT;
    }
    if (os_hal_nvm_read(OS_NVM_OFFSET_CALIBRATION, (uint8_t *)&s_calibration,
                        sizeof(s_calibration)) != OS_ERR_NONE) {
        memset(&s_calibration, 0, sizeof(s_calibration));
    }
    persist_park_t pp;
    memset(&pp, 0, sizeof(pp));
    if (os_hal_nvm_read(OS_NVM_OFFSET_CONFIG, (uint8_t *)&pp, sizeof(pp)) == OS_ERR_NONE) {
        if ((pp.custom_set & 0x01u) != 0u) {
            s_park_pos = pp.pos;
            s_custom_park_set = true;
        }
    }
    if (os_hal_nvm_read(OS_NVM_OFFSET_PEC, (uint8_t *)&s_pec_table,
                        sizeof(s_pec_table)) != OS_ERR_NONE) {
        memset(&s_pec_table, 0, sizeof(s_pec_table));
    }

    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        (void)os_hal_comm_init(ch);
    }
    for (uint8_t axis = 0u; axis < 2u; ++axis) {
        os_error_t err = os_hal_motor_init(axis);
        if (err != OS_ERR_NONE) {
            s_state = OS_STATE_FAULT;
            (void)os_hal_buzzer_beep(100u, 3u);
            return err;
        }
    }

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    if (os_hal_timer_motor_init() != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return OS_ERR_TIMEOUT;
    }

    os_site_info_t gps_site;
    memset(&gps_site, 0, sizeof(gps_site));
    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid) {
        s_gps_valid = true;
        s_gps_site = gps_site;
        s_site = gps_site;
        (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
    } else {
        s_gps_valid = false;
        uint32_t utc = 0u;
        if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = utc;
            s_site.valid = true;
        } else {
            s_site.valid = false;
        }
    }

    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    s_loop_tick_ms += OS_LOOP_TICK_MS;

    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        int16_t avail = os_hal_comm_available(ch);
        while (avail > 0) {
            char c = os_hal_comm_read(ch);
            --avail;
            if (s_cmd_len[ch] < OS_MAX_COMMAND_LENGTH) {
                s_cmd_buf[ch][s_cmd_len[ch]++] = c;
            }
            if (c == '\n' || c == OS_LX200_CMD_SUFFIX) {
                s_cmd_buf[ch][s_cmd_len[ch]] = '\0';
                char reply[OS_MAX_REPLY_LENGTH];
                size_t reply_len = 0u;
                (void)os_command_parse(s_cmd_buf[ch], s_cmd_len[ch], ch,
                                       reply, sizeof(reply), &reply_len);
                if (reply_len > 0u) {
                    (void)os_hal_comm_write(ch, reply, reply_len);
                }
                s_cmd_len[ch] = 0u;
                break;
            }
        }
    }

    os_site_info_t gps_site;
    memset(&gps_site, 0, sizeof(gps_site));
    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid) {
        s_gps_valid = true;
        s_gps_site = gps_site;
        s_site = gps_site;
        (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
    } else {
        s_gps_valid = false;
        uint32_t utc = 0u;
        if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = utc;
            s_site.valid = true;
        }
    }

    if (s_guide_pulse.active) {
        s_guide_elapsed_ms += OS_LOOP_TICK_MS;
        if (s_guide_elapsed_ms >= s_guide_pulse.duration_ms) {
            s_guide_pulse.active = false;
            s_guide_pulse.duration_ms = 0u;
            s_guide_elapsed_ms = 0u;
        }
    }

    if (s_goto_motion_active || s_park_motion_active) {
        bool limit0 = os_hal_limit_is_triggered(0u);
        bool limit1 = os_hal_limit_is_triggered(1u);
        if (limit0 || limit1) {
            s_goto_motion_active = false;
            s_park_motion_active = false;
            s_manual_motion_active = false;
            s_tracking_enabled = false;
            (void)os_hal_motor_set_frequency(0u, 0u);
            (void)os_hal_motor_set_frequency(1u, 0u);
            (void)os_hal_motor_enable(0u, false);
            (void)os_hal_motor_enable(1u, false);
            s_state = OS_STATE_FAULT;
            (void)os_hal_buzzer_beep(100u, 3u);
            return;
        }
        int32_t inc = loop_step_increment();
        if (s_goto_motion_active) {
            int32_t target_steps[2] = {s_goto_target_steps[0], s_goto_target_steps[1]};
            for (uint8_t axis = 0u; axis < 2u; ++axis) {
                s_motor_position_steps[axis] = step_toward(s_motor_position_steps[axis],
                                                           target_steps[axis], inc);
                bool axis_moving = (s_motor_position_steps[axis] != target_steps[axis]);
                (void)os_hal_motor_set_direction(axis, s_motor_position_steps[axis] < target_steps[axis]);
                (void)os_hal_motor_set_frequency(axis, axis_moving ? OS_MANUAL_FAST_HZ : 0u);
                (void)os_hal_motor_enable(axis, axis_moving);
            }
            bool arrived = (s_motor_position_steps[0] == s_goto_target_steps[0] &&
                            s_motor_position_steps[1] == s_goto_target_steps[1]);
            if (arrived) {
                s_goto_motion_active = false;
                s_tracking_enabled = true;
                s_state = OS_STATE_IDLE_TRACKING;
                (void)os_hal_motor_set_frequency(0u, 0u);
                (void)os_hal_motor_set_frequency(1u, 0u);
                (void)os_hal_buzzer_beep(80u, 1u);
                apply_tracking_frequency();
            }
        } else {
            int32_t target_steps[2] = {s_park_target_steps[0], s_park_target_steps[1]};
            for (uint8_t axis = 0u; axis < 2u; ++axis) {
                s_motor_position_steps[axis] = step_toward(s_motor_position_steps[axis],
                                                           target_steps[axis], inc);
                bool axis_moving = (s_motor_position_steps[axis] != target_steps[axis]);
                (void)os_hal_motor_set_direction(axis, s_motor_position_steps[axis] < target_steps[axis]);
                (void)os_hal_motor_set_frequency(axis, axis_moving ? OS_MANUAL_FAST_HZ : 0u);
                (void)os_hal_motor_enable(axis, axis_moving);
            }
            bool arrived = (s_motor_position_steps[0] == s_park_target_steps[0] &&
                            s_motor_position_steps[1] == s_park_target_steps[1]);
            if (arrived) {
                s_park_motion_active = false;
                s_tracking_enabled = false;
                s_state = OS_STATE_PARKED;
                (void)os_hal_motor_set_frequency(0u, 0u);
                (void)os_hal_motor_set_frequency(1u, 0u);
                (void)os_hal_motor_enable(0u, false);
                (void)os_hal_motor_enable(1u, false);
                (void)os_hal_buzzer_beep(80u, 1u);
            }
        }
    }

    if (s_manual_motion_active) {
        if (os_hal_limit_is_triggered(s_manual_axis)) {
            s_manual_motion_active = false;
            (void)os_hal_motor_set_frequency(s_manual_axis, 0u);
            (void)os_hal_motor_enable(s_manual_axis, false);
            s_state = OS_STATE_FAULT;
            (void)os_hal_buzzer_beep(100u, 3u);
        } else {
            (void)os_hal_motor_set_direction(s_manual_axis, s_manual_forward);
            (void)os_hal_motor_set_frequency(s_manual_axis, manual_frequency_hz());
        }
    }

    if (s_state == OS_STATE_IDLE_TRACKING && !s_tracking_enabled) {
        s_tracking_enabled = true;
    }

    if (s_tracking_enabled && s_state == OS_STATE_IDLE_TRACKING) {
        if (s_guide_pulse.active) {
            uint32_t base = effective_tracking_frequency_hz();
            uint32_t bias = (uint32_t)(s_guide_rate_fraction * OS_SIDEREAL_RATE_ARCSEC_PER_SEC);
            if (s_guide_pulse.dec_priority) {
                (void)os_hal_motor_set_direction(1u, s_guide_pulse.direction_north);
                (void)os_hal_motor_set_frequency(1u, base + bias);
                (void)os_hal_motor_set_frequency(0u, base);
            } else {
                (void)os_hal_motor_set_direction(0u, s_guide_pulse.direction_east);
                (void)os_hal_motor_set_frequency(0u, base + bias);
                (void)os_hal_motor_set_frequency(1u, 0u);
            }
        } else {
            apply_tracking_frequency();
        }
    } else {
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
    if (reply_buffer_size == 0u || length == 0u || length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (source_channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    reply_buffer[0] = '\0';
    *reply_length = 0u;

    size_t frame_len = length;
    while (frame_len > 0u && (command[frame_len - 1u] == '\n' || command[frame_len - 1u] == '\r')) {
        --frame_len;
    }

    os_error_t result = OS_ERR_NONE;
    bool is_lx200 = (frame_len >= 2u && command[0] == OS_LX200_CMD_PREFIX &&
                     command[frame_len - 1u] == OS_LX200_CMD_SUFFIX);
    if (!is_lx200) {
        set_reply(reply_buffer, reply_buffer_size, reply_length, "%d", (int)OS_ERR_COMMAND_FORMAT);
        result = OS_ERR_COMMAND_FORMAT;
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return result;
    }

    char body[OS_MAX_COMMAND_LENGTH];
    memset(body, 0, sizeof(body));
    size_t body_len = frame_len - 2u;
    if (body_len >= sizeof(body)) {
        body_len = sizeof(body) - 1u;
    }
    memcpy(body, command + 1u, body_len);

    if (body_len == 3u && body[0] == 'G' && body[1] == 'V' && body[2] == 'P') {
        uint8_t major = 0u, minor = 0u, patch = 0u;
        result = os_query_firmware_version(&major, &minor, &patch);
        if (result == OS_ERR_NONE) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "%u.%u.%u",
                      (unsigned)major, (unsigned)minor, (unsigned)patch);
        }
    } else if (body_len == 2u && body[0] == 'G' && body[1] == 'R') {
        os_equatorial_coord_t coord;
        result = os_query_coordinates(&coord);
        if (result == OS_ERR_NONE) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "%f", (double)coord.ra_hours);
        }
    } else if (body_len == 2u && body[0] == 'G' && body[1] == 'D') {
        os_equatorial_coord_t coord;
        result = os_query_coordinates(&coord);
        if (result == OS_ERR_NONE) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "%f", (double)coord.dec_degrees);
        }
    } else if (body_len == 2u && body[0] == 'G' && body[1] == 'S') {
        os_state_t st;
        result = os_query_state(&st);
        if (result == OS_ERR_NONE) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "%d", (int)st);
        }
    } else if (body_len == 2u && body[0] == 'G' && body[1] == 'Z') {
        bool locked = false;
        result = os_query_gps_locked(&locked);
        if (result == OS_ERR_NONE) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "%u", (unsigned)locked);
        }
    } else if (starts_with(body, "Sr") && body_len > 2u) {
        float ra = parse_angle_string(body + 2u);
        if (ra >= OS_RA_MIN_HOURS && ra <= OS_RA_MAX_HOURS) {
            s_last_equ_target.ra_hours = ra;
            set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        } else {
            result = OS_ERR_INVALID_ARGUMENT;
        }
    } else if (starts_with(body, "Sd") && body_len > 2u) {
        float dec = parse_angle_string(body + 2u);
        if (dec >= OS_DEC_MIN_DEG && dec <= OS_DEC_MAX_DEG) {
            s_last_equ_target.dec_degrees = dec;
            set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        } else {
            result = OS_ERR_INVALID_ARGUMENT;
        }
    } else if (body_len == 2u && body[0] == 'M' && body[1] == 'S') {
        s_last_equ_target.ra_hours = 0.0f;
        s_last_equ_target.dec_degrees = 90.0f;
        os_equatorial_coord_t current;
        if (s_goto_motion_active || s_park_motion_active) {
            result = OS_ERR_INVALID_STATE;
        } else if (os_query_coordinates(&current) == OS_ERR_NONE) {
            s_last_equ_target = current;
            result = os_goto_equatorial(s_last_equ_target);
            if (result == OS_ERR_NONE) {
                set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
            }
        } else {
            result = os_goto_equatorial(s_last_equ_target);
            if (result == OS_ERR_NONE) {
                set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
            }
        }
    } else if (starts_with(body, "Mg")) {
        const char *p = body + 2u;
        if (*p == '\0') {
            result = OS_ERR_NOT_SUPPORTED;
        } else {
            os_direction_t direction;
            if (*p == 'e') {
                direction = OS_DIRECTION_EAST;
            } else if (*p == 'w') {
                direction = OS_DIRECTION_WEST;
            } else if (*p == 'n') {
                direction = OS_DIRECTION_NORTH;
            } else if (*p == 's') {
                direction = OS_DIRECTION_SOUTH;
            } else {
                result = OS_ERR_NOT_SUPPORTED;
                set_reply(reply_buffer, reply_buffer_size, reply_length, "%d", (int)result);
                (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
                return result;
            }
            ++p;
            uint32_t duration_ms = (uint32_t)strtoul(p, NULL, 10);
            result = os_guide_pulse(direction, duration_ms);
            if (result == OS_ERR_NONE) {
                set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
            }
        }
    } else if (body_len == 2u && body[0] == 'M') {
        os_direction_t direction;
        if (body[1] == 'e') {
            direction = OS_DIRECTION_EAST;
        } else if (body[1] == 'w') {
            direction = OS_DIRECTION_WEST;
        } else if (body[1] == 'n') {
            direction = OS_DIRECTION_NORTH;
        } else if (body[1] == 's') {
            direction = OS_DIRECTION_SOUTH;
        } else {
            result = OS_ERR_NOT_SUPPORTED;
            set_reply(reply_buffer, reply_buffer_size, reply_length, "%d", (int)result);
            (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
            return result;
        }
        result = os_move_start(direction, OS_SPEED_MEDIUM);
        if (result == OS_ERR_NONE) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        }
    } else if (body_len == 1u && body[0] == 'Q') {
        if (s_manual_motion_active) {
            result = os_move_stop();
        } else {
            result = os_goto_abort();
        }
        if (result == OS_ERR_NONE) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        }
    } else if (body_len == 2u && body[0] == 'h' && body[1] == 'P') {
        result = os_park();
        if (result == OS_ERR_NONE) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        }
    } else if (body_len == 2u && body[0] == 'h' && body[1] == 'O') {
        result = os_unpark();
        if (result == OS_ERR_NONE) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        }
    } else {
        result = OS_ERR_NOT_SUPPORTED;
    }

    if (result != OS_ERR_NONE) {
        set_reply(reply_buffer, reply_buffer_size, reply_length, "%d", (int)result);
    }
    (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
    return result;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS ||
        target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_IDLE_TRACKING && s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    compute_goto_target_steps(target, s_goto_target_steps);
    s_goto_motion_active = true;
    s_park_motion_active = false;
    s_manual_motion_active = false;
    s_tracking_enabled = false;
    s_state = OS_STATE_GOTO;
    (void)os_hal_motor_set_direction(0u, s_goto_target_steps[0] > s_motor_position_steps[0]);
    (void)os_hal_motor_set_direction(1u, s_goto_target_steps[1] > s_motor_position_steps[1]);
    (void)os_hal_motor_set_frequency(0u, OS_MANUAL_FAST_HZ);
    (void)os_hal_motor_set_frequency(1u, OS_MANUAL_FAST_HZ);
    (void)os_hal_motor_enable(0u, true);
    (void)os_hal_motor_enable(1u, true);
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_IDLE_TRACKING && s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    s_goto_target_steps[0] = az_to_steps(target.azimuth_degrees);
    s_goto_target_steps[1] = alt_to_steps(target.altitude_degrees);
    s_goto_motion_active = true;
    s_park_motion_active = false;
    s_manual_motion_active = false;
    s_tracking_enabled = false;
    s_state = OS_STATE_GOTO;
    (void)os_hal_motor_set_direction(0u, s_goto_target_steps[0] > s_motor_position_steps[0]);
    (void)os_hal_motor_set_direction(1u, s_goto_target_steps[1] > s_motor_position_steps[1]);
    (void)os_hal_motor_set_frequency(0u, OS_MANUAL_FAST_HZ);
    (void)os_hal_motor_set_frequency(1u, OS_MANUAL_FAST_HZ);
    (void)os_hal_motor_enable(0u, true);
    (void)os_hal_motor_enable(1u, true);
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (!s_goto_motion_active && !s_park_motion_active && s_state != OS_STATE_GOTO) {
        return OS_ERR_INVALID_STATE;
    }
    s_goto_motion_active = false;
    s_park_motion_active = false;
    s_manual_motion_active = false;
    s_tracking_enabled = true;
    s_state = OS_STATE_IDLE_TRACKING;
    (void)os_hal_motor_set_frequency(0u, 0u);
    (void)os_hal_motor_set_frequency(1u, 0u);
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (rate < OS_TRACK_RATE_SIDEREAL || rate > OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_tracking_rate = rate;
    s_tracking_custom_factor = custom_factor;
    if (s_tracking_enabled && s_state == OS_STATE_IDLE_TRACKING) {
        apply_tracking_frequency();
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = s_tracking_rate;
    *custom_factor = s_tracking_custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    if (s_state == OS_STATE_FAULT || s_state == OS_STATE_PARKED ||
        s_state == OS_STATE_INITIALIZING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    s_tracking_enabled = true;
    apply_tracking_frequency();
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    s_tracking_enabled = false;
    (void)os_hal_motor_set_frequency(0u, 0u);
    (void)os_hal_motor_set_frequency(1u, 0u);
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state == OS_STATE_FAULT || s_state == OS_STATE_PARKED ||
        s_state == OS_STATE_INITIALIZING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    s_guide_pulse.active = true;
    s_guide_pulse.duration_ms = duration_ms;
    s_guide_pulse.rate_fraction = s_guide_rate_fraction;
    s_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    s_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    s_guide_pulse.dec_priority = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);
    s_guide_elapsed_ms = 0u;
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_guide_rate_fraction = rate_fraction;
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
    s_align_star_count = 0u;
    s_align_active = true;
    s_calib_residual_computed = false;
    s_residual_arcsec = 0.0f;
    memset(s_align_points, 0, sizeof(s_align_points));
    s_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (star_coord.ra_hours < OS_RA_MIN_HOURS || star_coord.ra_hours > OS_RA_MAX_HOURS ||
        star_coord.dec_degrees < OS_DEC_MIN_DEG || star_coord.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_align_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }
    s_align_points[s_align_star_count].coord = star_coord;
    s_align_points[s_align_star_count].motor = motor_pos;
    ++s_align_star_count;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    uint8_t n = s_align_star_count;
    uint8_t needed = (s_align_mode == OS_ALIGN_3STAR || s_align_mode == OS_ALIGN_NSTAR) ? 3u :
                     (s_align_mode == OS_ALIGN_2STAR ? 2u : 1u);
    if (n < needed) {
        return OS_ERR_INVALID_STATE;
    }

    double ra[OS_CALIBRATION_MAX_STARS];
    double dec[OS_CALIBRATION_MAX_STARS];
    double m_ra[OS_CALIBRATION_MAX_STARS];
    double m_dec[OS_CALIBRATION_MAX_STARS];
    for (uint8_t i = 0u; i < n; ++i) {
        ra[i] = (double)s_align_points[i].coord.ra_hours * 3600.0;
        dec[i] = (double)s_align_points[i].coord.dec_degrees * 3600.0;
        m_ra[i] = (double)s_align_points[i].motor.ra_steps;
        m_dec[i] = (double)s_align_points[i].motor.dec_steps;
    }

    double r_ra[3] = {0.0, 0.0, 0.0};
    double r_dec[3] = {0.0, 0.0, 0.0};

    if (s_align_mode == OS_ALIGN_1STAR) {
        r_ra[0] = 1.0;
        r_ra[1] = 0.0;
        r_ra[2] = m_ra[0] - ra[0];
        r_dec[0] = 0.0;
        r_dec[1] = 1.0;
        r_dec[2] = m_dec[0] - dec[0];
        s_residual_arcsec = 0.0f;
        s_calib_residual_computed = false;
    } else if (s_align_mode == OS_ALIGN_2STAR) {
        if (fabs(ra[1] - ra[0]) < 1e-12 || fabs(dec[1] - dec[0]) < 1e-12) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        double a = (m_ra[1] - m_ra[0]) / (ra[1] - ra[0]);
        double c = m_ra[0] - a * ra[0];
        double e = (m_dec[1] - m_dec[0]) / (dec[1] - dec[0]);
        double f = m_dec[0] - e * dec[0];
        r_ra[0] = a;
        r_ra[1] = 0.0;
        r_ra[2] = c;
        r_dec[0] = 0.0;
        r_dec[1] = e;
        r_dec[2] = f;
        s_residual_arcsec = 0.0f;
        s_calib_residual_computed = false;
    } else {
        if (s_align_mode == OS_ALIGN_3STAR) {
            double det = ra[0] * (dec[1] - dec[2]) -
                         dec[0] * (ra[1] - ra[2]) +
                         (ra[1] * dec[2] - ra[2] * dec[1]);
            if (fabs(det) < 1e-9) {
                return OS_ERR_CALIBRATION_FAILED;
            }
        }
        double A[OS_CALIBRATION_MAX_STARS][3];
        double b_ra[OS_CALIBRATION_MAX_STARS];
        double b_dec[OS_CALIBRATION_MAX_STARS];
        for (uint8_t i = 0u; i < n; ++i) {
            A[i][0] = ra[i];
            A[i][1] = dec[i];
            A[i][2] = 1.0;
            b_ra[i] = m_ra[i];
            b_dec[i] = m_dec[i];
        }
        if (!qr_solve_3(A, b_ra, (int)n, r_ra) ||
            !qr_solve_3(A, b_dec, (int)n, r_dec)) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        if (s_align_mode == OS_ALIGN_NSTAR && n >= 4u) {
            s_residual_arcsec = compute_residual_arcsec(ra, dec, m_ra, m_dec,
                                                       (int)n, r_ra, r_dec);
            s_calib_residual_computed = true;
        } else {
            s_residual_arcsec = 0.0f;
            s_calib_residual_computed = false;
        }
    }

    s_calibration.matrix_ra_to_ra = (float)r_ra[0];
    s_calibration.matrix_ra_to_dec = (float)r_ra[1];
    s_calibration.offset_ra_arcsec = (float)r_ra[2];
    s_calibration.matrix_dec_to_ra = (float)r_dec[0];
    s_calibration.matrix_dec_to_dec = (float)r_dec[1];
    s_calibration.offset_dec_arcsec = (float)r_dec[2];
    s_calibration.valid = true;

    if (os_hal_nvm_write(OS_NVM_OFFSET_CALIBRATION, (const uint8_t *)&s_calibration,
                         sizeof(s_calibration)) != OS_ERR_NONE) {
        s_calibration.valid = false;
        s_state = OS_STATE_FAULT;
        return OS_ERR_NVM_FAULT;
    }

    s_align_active = false;
    s_align_star_count = 0u;
    memset(s_align_points, 0, sizeof(s_align_points));
    s_state = OS_STATE_IDLE_TRACKING;
    s_tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_calib_residual_computed) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = s_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    if (s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    s_align_active = false;
    s_align_star_count = 0u;
    s_calib_residual_computed = false;
    s_residual_arcsec = 0.0f;
    memset(s_align_points, 0, sizeof(s_align_points));
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (s_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (s_custom_park_set) {
        s_park_target_steps[0] = ra_to_steps(s_park_pos.ra_hours);
        s_park_target_steps[1] = dec_to_steps(s_park_pos.dec_degrees);
    } else {
        s_park_pos.ra_hours = 0.0f;
        s_park_pos.dec_degrees = 90.0f;
        s_park_target_steps[0] = 0;
        s_park_target_steps[1] = dec_to_steps(90.0f);
    }
    s_park_motion_active = true;
    s_goto_motion_active = false;
    s_manual_motion_active = false;
    s_tracking_enabled = false;
    s_state = OS_STATE_GOTO;
    (void)os_hal_motor_set_direction(0u, s_park_target_steps[0] > s_motor_position_steps[0]);
    (void)os_hal_motor_set_direction(1u, s_park_target_steps[1] > s_motor_position_steps[1]);
    (void)os_hal_motor_set_frequency(0u, OS_MANUAL_FAST_HZ);
    (void)os_hal_motor_set_frequency(1u, OS_MANUAL_FAST_HZ);
    (void)os_hal_motor_enable(0u, true);
    (void)os_hal_motor_enable(1u, true);
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (s_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    for (uint8_t axis = 0u; axis < 2u; ++axis) {
        if (os_hal_motor_enable(axis, true) != OS_ERR_NONE) {
            s_state = OS_STATE_FAULT;
            return OS_ERR_MOTOR_DRIVER_FAULT;
        }
    }
    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        os_error_t err = os_hal_comm_init(ch);
        if (err != OS_ERR_NONE && err != OS_ERR_NOT_SUPPORTED) {
            s_state = OS_STATE_FAULT;
            return err;
        }
    }
    uint32_t utc = 0u;
    if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
        s_site.utc_epoch_seconds = utc;
    }
    s_tracking_enabled = true;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (park_pos.ra_hours < OS_RA_MIN_HOURS || park_pos.ra_hours > OS_RA_MAX_HOURS ||
        park_pos.dec_degrees < OS_DEC_MIN_DEG || park_pos.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    persist_park_t pp;
    memset(&pp, 0, sizeof(pp));
    pp.pos = park_pos;
    pp.custom_set = 1u;
    if (os_hal_nvm_write(OS_NVM_OFFSET_CONFIG, (const uint8_t *)&pp, sizeof(pp)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    s_park_pos = park_pos;
    s_custom_park_set = true;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    uint8_t axis = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH) ? 1u : 0u;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    s_manual_axis = axis;
    s_manual_forward = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_NORTH);
    s_manual_speed = speed;
    s_manual_motion_active = true;
    s_state = OS_STATE_MANUAL_MOTION;
    (void)os_hal_motor_set_direction(axis, s_manual_forward);
    (void)os_hal_motor_set_frequency(axis, manual_frequency_hz());
    (void)os_hal_motor_enable(axis, true);
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (s_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }
    s_manual_motion_active = false;
    (void)os_hal_motor_set_frequency(s_manual_axis, 0u);
    s_state = OS_STATE_IDLE_TRACKING;
    s_tracking_enabled = true;
    apply_tracking_frequency();
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (arcsec_per_sec <= 0.0f ||
        arcsec_per_sec > (OS_GOTO_SPEED_MAX_DEG_PER_SEC * 3600.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_manual_custom_speed_arcsec_per_sec = arcsec_per_sec;
    if (s_manual_motion_active && s_manual_speed == OS_SPEED_CUSTOM) {
        (void)os_hal_motor_set_frequency(s_manual_axis, manual_frequency_hz());
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
    double ra;
    double dec;
    if (s_calibration.valid) {
        int32_t mra = os_hal_motor_get_position(0u);
        int32_t mdec = os_hal_motor_get_position(1u);
        double a = (double)s_calibration.matrix_ra_to_ra;
        double b = (double)s_calibration.matrix_ra_to_dec;
        double c = (double)s_calibration.matrix_dec_to_ra;
        double d = (double)s_calibration.matrix_dec_to_dec;
        double e = (double)s_calibration.offset_ra_arcsec;
        double f = (double)s_calibration.offset_dec_arcsec;
        double mra_c = (double)mra - e;
        double mdec_c = (double)mdec - f;
        double det = a * d - b * c;
        if (fabs(det) < 1e-9) {
            return OS_ERR_INVALID_STATE;
        }
        double ra_arcsec = (d * mra_c - b * mdec_c) / det;
        double dec_arcsec = (-c * mra_c + a * mdec_c) / det;
        ra = ra_arcsec / 3600.0;
        dec = dec_arcsec / 3600.0;
    } else {
        ra = (double)os_hal_motor_get_position(0u) / (double)OS_STEPS_PER_RA_HOUR;
        dec = (double)os_hal_motor_get_position(1u) / (double)OS_STEPS_PER_DEC_DEG;
    }
    while (ra < 0.0) {
        ra += 24.0;
    }
    while (ra >= 24.0) {
        ra -= 24.0;
    }
    if (dec < -90.0) {
        dec = -90.0;
    }
    if (dec > 90.0) {
        dec = 90.0;
    }
    coord->ra_hours = (float)ra;
    coord->dec_degrees = (float)dec;
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
    *moving = (s_goto_motion_active || s_manual_motion_active || s_park_motion_active);
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *locked = s_gps_valid;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    if (enable && !s_pec_table.valid) {
        return OS_ERR_INVALID_STATE;
    }
    s_pec_enabled = enable;
    if (s_tracking_enabled && s_state == OS_STATE_IDLE_TRACKING) {
        apply_tracking_frequency();
    }
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_pec_table = *table;
    s_pec_table.valid = true;
    if (os_hal_nvm_write(OS_NVM_OFFSET_PEC, (const uint8_t *)&s_pec_table,
                         sizeof(s_pec_table)) != OS_ERR_NONE) {
        s_pec_table.valid = false;
        return OS_ERR_NVM_FAULT;
    }
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_pec_table.valid) {
        return OS_ERR_INVALID_STATE;
    }
    *table = s_pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (!(worm_phase_deg >= 0.0f && worm_phase_deg <= 360.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int index = (worm_phase_deg >= 360.0f) ? 0 : (int)worm_phase_deg;
    if (index < 0 || index >= OS_PEC_TABLE_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_worm_phase_deg = worm_phase_deg;
    s_pec_table.corrections[index] = error_arcsec;
    s_pec_table.valid = true;
    if (os_hal_nvm_write(OS_NVM_OFFSET_PEC, (const uint8_t *)&s_pec_table,
                         sizeof(s_pec_table)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
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
    if (os_hal_nvm_write(OS_NVM_OFFSET_CALIBRATION, (const uint8_t *)&s_calibration,
                         sizeof(s_calibration)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_init(uint8_t axis) {
    if (axis > 1u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_motor_initialized[axis] = true;
    s_motor_enabled[axis] = false;
    s_motor_frequency_hz[axis] = 0u;
    s_motor_direction[axis] = false;
    s_motor_driver_fault[axis] = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (axis > 1u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (frequency_hz > OS_MOTOR_FREQ_LIMIT_HZ) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_motor_frequency_hz[axis] = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (axis > 1u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_motor_direction[axis] = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (axis > 1u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_motor_enabled[axis] = enable;
    if (!enable) {
        s_motor_frequency_hz[axis] = 0u;
    }
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (axis > 1u) {
        return 0;
    }
    return s_motor_position_steps[axis];
}

os_error_t os_hal_gps_init(void) {
    s_gps_initialized = true;
    s_gps_valid = false;
    memset(&s_gps_site, 0, sizeof(s_gps_site));
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_gps_valid && s_gps_site.valid && s_gps_site.latitude_degrees >= -90.0f &&
        s_gps_site.latitude_degrees <= 90.0f && s_gps_site.longitude_degrees >= -180.0f &&
        s_gps_site.longitude_degrees <= 180.0f) {
        *site = s_gps_site;
        return OS_ERR_NONE;
    }
    memset(site, 0, sizeof(*site));
    site->valid = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void) {
    s_rtc_initialized = true;
    s_rtc_valid = true;
    s_rtc_utc = 0u;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_rtc_initialized || !s_rtc_valid) {
        return OS_ERR_TIMEOUT;
    }
    *utc_epoch_seconds = s_rtc_utc;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    if (!s_rtc_initialized || !s_rtc_valid) {
        return OS_ERR_TIMEOUT;
    }
    s_rtc_utc = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    s_limit_initialized = true;
    s_limit_triggered[0] = false;
    s_limit_triggered[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (axis > 1u) {
        return true;
    }
    return s_limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void) {
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((uint32_t)offset + (uint32_t)length > OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(data, &s_nvm[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((uint32_t)offset + (uint32_t)length > OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(&s_nvm[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_comm_initialized[channel] = true;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET || !s_comm_initialized[channel]) {
        return 0;
    }
    return s_comm_rx_len[channel];
}

char os_hal_comm_read(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET || !s_comm_initialized[channel] ||
        s_comm_rx_len[channel] <= 0) {
        return '\0';
    }
    char c = s_comm_rx[channel][0];
    --s_comm_rx_len[channel];
    if (s_comm_rx_len[channel] > 0) {
        memmove(&s_comm_rx[channel][0], &s_comm_rx[channel][1], (size_t)s_comm_rx_len[channel]);
    }
    return c;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length == 0u) {
        return OS_ERR_NONE;
    }
    if ((size_t)s_comm_tx_len[channel] + length > sizeof(s_comm_tx[channel])) {
        return OS_ERR_TIMEOUT;
    }
    memcpy(&s_comm_tx[channel][s_comm_tx_len[channel]], data, length);
    s_comm_tx_len[channel] = (uint16_t)(s_comm_tx_len[channel] + length);
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) {
    s_buzzer_pending = true;
    s_buzzer_duration_ms = duration_ms;
    s_buzzer_count = count;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void) {
    s_motor_timer_initialized = true;
    return OS_ERR_NONE;
}

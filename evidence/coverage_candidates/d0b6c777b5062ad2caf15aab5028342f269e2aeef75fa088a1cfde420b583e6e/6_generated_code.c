/* OnStep fixed API C11 library implementation with deterministic host HAL. */

#include "6_generated_code.h"

#include <stdio.h>
#include <string.h>

#define OS_AXIS_RA 0u
#define OS_AXIS_DEC 1u
#define OS_AXIS_COUNT 2u
#define OS_CHANNEL_COUNT 4u
#define OS_STEPS_PER_DEG 1000.0
#define OS_STEPS_PER_RA_HOUR 15000.0
#define OS_LOOP_DT_MS 100u
#define OS_TRACK_FREQ_BASE_HZ 4u
#define OS_GOTO_FREQ_HZ 3000u
#define OS_MANUAL_SLOW_HZ 50u
#define OS_MANUAL_MEDIUM_HZ 400u
#define OS_MANUAL_FAST_HZ 1200u
#define OS_NVM_BYTES 2048u
#define OS_NVM_CALIB_OFFSET 0u
#define OS_NVM_CALIB_MAGIC 0x43414C31u

typedef struct {
    uint32_t magic;
    os_calibration_t calibration;
} os_cal_record_t;

typedef struct {
    bool initialized;
    bool enabled;
    bool direction;
    uint32_t frequency_hz;
    int32_t position_steps;
    os_error_t fault;
} os_host_motor_t;

typedef struct {
    os_state_t state;
    os_track_rate_t track_rate;
    float custom_track_factor;
    bool tracking_enabled;
    bool gps_locked;
    os_site_info_t site;
    os_equatorial_coord_t current_coord;
    os_equatorial_coord_t target_coord;
    os_equatorial_coord_t pending_set_coord;
    os_equatorial_coord_t park_coord;
    bool park_custom;
    bool moving;
    bool parking_motion;
    int32_t goto_target[OS_AXIS_COUNT];
    int32_t goto_remaining[OS_AXIS_COUNT];
    os_direction_t manual_direction;
    os_speed_level_t manual_speed;
    float custom_manual_arcsec_per_sec;
    float guide_rate;
    os_guide_pulse_t guide;
    uint32_t guide_remaining_ms;
    os_align_mode_t align_mode;
    os_equatorial_coord_t align_star[OS_CALIBRATION_MAX_STARS];
    os_motor_position_t align_pos[OS_CALIBRATION_MAX_STARS];
    uint8_t align_count;
    os_calibration_t calibration;
    bool residual_valid;
    float residual_arcsec;
    os_pec_table_t pec;
    bool pec_enabled;
    char cmd_buf[OS_CHANNEL_COUNT][OS_MAX_COMMAND_LENGTH];
    size_t cmd_len[OS_CHANNEL_COUNT];
} os_context_t;

static os_context_t g_os;
static os_host_motor_t g_motor[OS_AXIS_COUNT];
static bool g_limit[OS_AXIS_COUNT];
static uint8_t g_nvm[OS_NVM_BYTES];
static bool g_nvm_initialized;
static uint32_t g_rtc_epoch = 1704067200u;
static os_site_info_t g_gps_sample;
static bool g_gps_has_sample;
static char g_rx[OS_CHANNEL_COUNT][256];
static size_t g_rx_head[OS_CHANNEL_COUNT], g_rx_tail[OS_CHANNEL_COUNT];
static char g_tx[OS_CHANNEL_COUNT][512];
static size_t g_tx_len[OS_CHANNEL_COUNT];
static uint16_t g_buzzer_duration;
static uint8_t g_buzzer_count;

static bool valid_ra(float ra) { return ra >= OS_RA_MIN_HOURS && ra <= OS_RA_MAX_HOURS; }
static bool valid_dec(float dec) { return dec >= OS_DEC_MIN_DEG && dec <= OS_DEC_MAX_DEG; }
static bool valid_eq(os_equatorial_coord_t c) { return valid_ra(c.ra_hours) && valid_dec(c.dec_degrees); }
static bool valid_direction(os_direction_t d) { return d >= OS_DIRECTION_NORTH && d <= OS_DIRECTION_WEST; }
static bool valid_speed(os_speed_level_t s) { return s >= OS_SPEED_SLOW && s <= OS_SPEED_CUSTOM; }
static bool valid_track(os_track_rate_t r) { return r >= OS_TRACK_RATE_SIDEREAL && r <= OS_TRACK_RATE_CUSTOM; }

static int32_t round_to_i32(double v) {
    return (int32_t)(v >= 0.0 ? v + 0.5 : v - 0.5);
}

static int32_t abs_i32(int32_t v) {
    return v < 0 ? -v : v;
}

static void stop_axis(uint8_t axis) {
    if (axis < OS_AXIS_COUNT) {
        (void)os_hal_motor_set_frequency(axis, 0u);
        (void)os_hal_motor_enable(axis, false);
    }
}

static void stop_all_motion(void) {
    stop_axis(OS_AXIS_RA);
    stop_axis(OS_AXIS_DEC);
    g_os.moving = false;
    g_os.parking_motion = false;
    g_os.goto_remaining[0] = 0;
    g_os.goto_remaining[1] = 0;
}

static float track_factor(void) {
    if (g_os.track_rate == OS_TRACK_RATE_LUNAR) return OS_LUNAR_RATE_FACTOR;
    if (g_os.track_rate == OS_TRACK_RATE_SOLAR) return OS_SOLAR_RATE_FACTOR;
    if (g_os.track_rate == OS_TRACK_RATE_CUSTOM) return g_os.custom_track_factor;
    return 1.0f;
}

static void apply_tracking(void) {
    if (g_os.state == OS_STATE_IDLE_TRACKING && g_os.tracking_enabled) {
        uint32_t freq = (uint32_t)(OS_TRACK_FREQ_BASE_HZ * track_factor());
        if (freq == 0u) freq = 1u;
        (void)os_hal_motor_set_direction(OS_AXIS_RA, true);
        (void)os_hal_motor_enable(OS_AXIS_RA, true);
        (void)os_hal_motor_set_frequency(OS_AXIS_RA, freq);
    }
}

static int32_t coord_to_ra_steps(os_equatorial_coord_t c) {
    double arcsec = (double)c.ra_hours * 15.0 * 3600.0;
    if (g_os.calibration.valid) {
        double dra = (double)g_os.calibration.matrix_ra_to_ra * arcsec +
                     (double)g_os.calibration.matrix_dec_to_ra * ((double)c.dec_degrees * 3600.0) +
                     (double)g_os.calibration.offset_ra_arcsec;
        return round_to_i32(dra / 3.6);
    }
    return round_to_i32((double)c.ra_hours * OS_STEPS_PER_RA_HOUR);
}

static int32_t coord_to_dec_steps(os_equatorial_coord_t c) {
    double arcsec = (double)c.dec_degrees * 3600.0;
    if (g_os.calibration.valid) {
        double ddec = (double)g_os.calibration.matrix_ra_to_dec * ((double)c.ra_hours * 15.0 * 3600.0) +
                      (double)g_os.calibration.matrix_dec_to_dec * arcsec +
                      (double)g_os.calibration.offset_dec_arcsec;
        return round_to_i32(ddec / 3.6);
    }
    return round_to_i32((double)c.dec_degrees * OS_STEPS_PER_DEG);
}

static os_equatorial_coord_t steps_to_coord(int32_t ra_steps, int32_t dec_steps) {
    os_equatorial_coord_t c;
    c.ra_hours = (float)((double)ra_steps / OS_STEPS_PER_RA_HOUR);
    c.dec_degrees = (float)((double)dec_steps / OS_STEPS_PER_DEG);
    if (c.ra_hours < 0.0f) c.ra_hours = 0.0f;
    if (c.ra_hours > 24.0f) c.ra_hours = 24.0f;
    if (c.dec_degrees < -90.0f) c.dec_degrees = -90.0f;
    if (c.dec_degrees > 90.0f) c.dec_degrees = 90.0f;
    return c;
}

static os_error_t start_goto_to(os_equatorial_coord_t target, bool parking) {
    if (!valid_eq(target)) return OS_ERR_INVALID_ARGUMENT;
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        stop_all_motion();
        g_os.state = OS_STATE_FAULT;
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_os.target_coord = target;
    g_os.goto_target[0] = coord_to_ra_steps(target);
    g_os.goto_target[1] = coord_to_dec_steps(target);
    g_os.goto_remaining[0] = g_os.goto_target[0] - os_hal_motor_get_position(OS_AXIS_RA);
    g_os.goto_remaining[1] = g_os.goto_target[1] - os_hal_motor_get_position(OS_AXIS_DEC);
    g_os.parking_motion = parking;

    if (abs_i32(g_os.goto_remaining[0]) <= 1 && abs_i32(g_os.goto_remaining[1]) <= 1) {
        g_os.current_coord = target;
        g_os.state = parking ? OS_STATE_PARKED : OS_STATE_IDLE_TRACKING;
        if (parking) {
            stop_all_motion();
            g_os.tracking_enabled = false;
        } else {
            apply_tracking();
        }
        return OS_ERR_NONE;
    }

    for (uint8_t axis = 0; axis < OS_AXIS_COUNT; axis++) {
        if (g_os.goto_remaining[axis] != 0) {
            (void)os_hal_motor_set_direction(axis, g_os.goto_remaining[axis] > 0);
            (void)os_hal_motor_enable(axis, true);
            (void)os_hal_motor_set_frequency(axis, OS_GOTO_FREQ_HZ);
        }
    }

    g_os.moving = true;
    g_os.state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

static void advance_host_motor(uint8_t axis, uint32_t ms) {
    if (axis >= OS_AXIS_COUNT || !g_motor[axis].enabled || g_motor[axis].frequency_hz == 0u) return;
    int32_t steps = (int32_t)((g_motor[axis].frequency_hz * ms) / 1000u);
    if (steps <= 0) steps = 1;
    g_motor[axis].position_steps += g_motor[axis].direction ? steps : -steps;
}

static void service_goto(void) {
    if (g_os.state != OS_STATE_GOTO || !g_os.moving) return;

    for (uint8_t axis = 0; axis < OS_AXIS_COUNT; axis++) {
        if (os_hal_limit_is_triggered(axis)) {
            stop_all_motion();
            g_os.state = OS_STATE_FAULT;
            return;
        }
    }

    bool any = false;
    for (uint8_t axis = 0; axis < OS_AXIS_COUNT; axis++) {
        int32_t rem = g_os.goto_target[axis] - os_hal_motor_get_position(axis);
        g_os.goto_remaining[axis] = rem;
        if (abs_i32(rem) > 1) {
            int32_t step = rem;
            int32_t max_step = (int32_t)(OS_GOTO_FREQ_HZ / (1000u / OS_LOOP_DT_MS));
            if (max_step < 1) max_step = 1;
            if (step > max_step) step = max_step;
            if (step < -max_step) step = -max_step;
            g_motor[axis].direction = step > 0;
            g_motor[axis].position_steps += step;
            any = true;
        } else {
            g_motor[axis].position_steps = g_os.goto_target[axis];
            stop_axis(axis);
        }
    }

    if (!any) {
        g_os.current_coord = g_os.target_coord;
        g_os.moving = false;
        (void)os_hal_buzzer_beep(100u, 1u);
        if (g_os.parking_motion) {
            g_os.tracking_enabled = false;
            stop_all_motion();
            g_os.state = OS_STATE_PARKED;
        } else {
            g_os.state = OS_STATE_IDLE_TRACKING;
            apply_tracking();
        }
    }
}

static void service_guide(void) {
    if (!g_os.guide.active) return;
    if (g_os.guide_remaining_ms <= OS_LOOP_DT_MS) {
        g_os.guide_remaining_ms = 0u;
        memset(&g_os.guide, 0, sizeof(g_os.guide));
        apply_tracking();
    } else {
        g_os.guide_remaining_ms -= OS_LOOP_DT_MS;
        g_os.guide.duration_ms = g_os.guide_remaining_ms;
    }
}

static void service_comm(void) {
    for (uint8_t ch = 0; ch < OS_CHANNEL_COUNT; ch++) {
        int16_t avail = os_hal_comm_available(ch);
        while (avail-- > 0) {
            char b = os_hal_comm_read(ch);
            if (g_os.cmd_len[ch] == 0u && b != OS_LX200_CMD_PREFIX) continue;
            if (g_os.cmd_len[ch] < OS_MAX_COMMAND_LENGTH) {
                g_os.cmd_buf[ch][g_os.cmd_len[ch]++] = b;
            } else {
                g_os.cmd_len[ch] = 0u;
                continue;
            }
            if (b == OS_LX200_CMD_SUFFIX) {
                char reply[OS_MAX_REPLY_LENGTH];
                size_t reply_len = 0u;
                (void)os_command_parse(g_os.cmd_buf[ch], g_os.cmd_len[ch], ch,
                                        reply, sizeof(reply), &reply_len);
                if (reply_len > 0u) (void)os_hal_comm_write(ch, reply, reply_len);
                g_os.cmd_len[ch] = 0u;
            }
        }
    }
}

static void save_calibration(void) {
    os_cal_record_t rec;
    rec.magic = OS_NVM_CALIB_MAGIC;
    rec.calibration = g_os.calibration;
    (void)os_hal_nvm_write(OS_NVM_CALIB_OFFSET, (const uint8_t *)&rec, (uint16_t)sizeof(rec));
}

static bool solve3(double a[3][3], double b[3], double x[3]) {
    double m[3][4];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) m[r][c] = a[r][c];
        m[r][3] = b[r];
    }
    for (int p = 0; p < 3; p++) {
        int best = p;
        double best_abs = m[p][p] < 0.0 ? -m[p][p] : m[p][p];
        for (int r = p + 1; r < 3; r++) {
            double v = m[r][p] < 0.0 ? -m[r][p] : m[r][p];
            if (v > best_abs) { best = r; best_abs = v; }
        }
        if (best_abs < 1e-9) return false;
        if (best != p) {
            for (int c = p; c < 4; c++) {
                double tmp = m[p][c]; m[p][c] = m[best][c]; m[best][c] = tmp;
            }
        }
        double div = m[p][p];
        for (int c = p; c < 4; c++) m[p][c] /= div;
        for (int r = 0; r < 3; r++) {
            if (r == p) continue;
            double f = m[r][p];
            for (int c = p; c < 4; c++) m[r][c] -= f * m[p][c];
        }
    }
    x[0] = m[0][3]; x[1] = m[1][3]; x[2] = m[2][3];
    return true;
}

static bool affine_fit(uint8_t n, double out_ra[3], double out_dec[3], double *resid) {
    double q[OS_CALIBRATION_MAX_STARS][3], r[3][3] = {{0}}, y0[OS_CALIBRATION_MAX_STARS], y1[OS_CALIBRATION_MAX_STARS];
    double v[OS_CALIBRATION_MAX_STARS][3];

    for (uint8_t i = 0; i < n; i++) {
        v[i][0] = (double)g_os.align_star[i].ra_hours * 15.0 * 3600.0;
        v[i][1] = (double)g_os.align_star[i].dec_degrees * 3600.0;
        v[i][2] = 1.0;
        y0[i] = (double)g_os.align_pos[i].ra_steps * 3.6;
        y1[i] = (double)g_os.align_pos[i].dec_steps * 3.6;
    }

    for (int j = 0; j < 3; j++) {
        for (uint8_t i = 0; i < n; i++) q[i][j] = v[i][j];
        for (int k = 0; k < j; k++) {
            double dot = 0.0;
            for (uint8_t i = 0; i < n; i++) dot += q[i][k] * v[i][j];
            r[k][j] = dot;
            for (uint8_t i = 0; i < n; i++) q[i][j] -= dot * q[i][k];
        }
        double norm2 = 0.0;
        for (uint8_t i = 0; i < n; i++) norm2 += q[i][j] * q[i][j];
        if (norm2 < 1e-12) return false;
        double norm = norm2;
        for (int iter = 0; iter < 8; iter++) norm = 0.5 * (norm + norm2 / norm);
        r[j][j] = norm;
        for (uint8_t i = 0; i < n; i++) q[i][j] /= norm;
    }

    double qty0[3] = {0}, qty1[3] = {0};
    for (int j = 0; j < 3; j++) {
        for (uint8_t i = 0; i < n; i++) {
            qty0[j] += q[i][j] * y0[i];
            qty1[j] += q[i][j] * y1[i];
        }
    }

    for (int i = 2; i >= 0; i--) {
        double s0 = qty0[i], s1 = qty1[i];
        for (int j = i + 1; j < 3; j++) {
            s0 -= r[i][j] * out_ra[j];
            s1 -= r[i][j] * out_dec[j];
        }
        if (r[i][i] < 1e-12 && r[i][i] > -1e-12) return false;
        out_ra[i] = s0 / r[i][i];
        out_dec[i] = s1 / r[i][i];
    }

    *resid = 0.0;
    if (n >= 4u) {
        double sum = 0.0;
        for (uint8_t i = 0; i < n; i++) {
            double x0 = (double)g_os.align_star[i].ra_hours * 15.0 * 3600.0;
            double x1 = (double)g_os.align_star[i].dec_degrees * 3600.0;
            double e0 = out_ra[0] * x0 + out_ra[1] * x1 + out_ra[2] - (double)g_os.align_pos[i].ra_steps * 3.6;
            double e1 = out_dec[0] * x0 + out_dec[1] * x1 + out_dec[2] - (double)g_os.align_pos[i].dec_steps * 3.6;
            sum += e0 * e0 + e1 * e1;
        }
        for (int iter = 0; iter < 8; iter++) {
            if (sum <= 0.0) break;
            double guess = *resid > 0.0 ? *resid : sum / (double)n;
            *resid = 0.5 * (guess + (sum / (double)n) / guess);
        }
    }
    return true;
}

static size_t put_reply(char *buf, size_t cap, const char *text) {
    size_t n = strlen(text);
    if (cap == 0u) return 0u;
    if (n >= cap) n = cap - 1u;
    memcpy(buf, text, n);
    buf[n] = '\0';
    return n;
}

os_error_t os_init(void) {
    memset(&g_os, 0, sizeof(g_os));
    g_os.state = OS_STATE_INITIALIZING;
    g_os.track_rate = OS_TRACK_RATE_SIDEREAL;
    g_os.custom_track_factor = 1.0f;
    g_os.tracking_enabled = true;
    g_os.guide_rate = 0.5f;
    g_os.park_coord.ra_hours = 0.0f;
    g_os.park_coord.dec_degrees = 90.0f;
    g_os.site.latitude_degrees = 0.0f;
    g_os.site.longitude_degrees = 0.0f;
    g_os.site.elevation_metres = 0.0f;
    g_os.site.utc_epoch_seconds = g_rtc_epoch;
    g_os.site.valid = true;

    os_error_t err = os_hal_nvm_init();
    if (err != OS_ERR_NONE) { g_os.state = OS_STATE_FAULT; return err; }

    os_cal_record_t rec;
    if (os_hal_nvm_read(OS_NVM_CALIB_OFFSET, (uint8_t *)&rec, (uint16_t)sizeof(rec)) == OS_ERR_NONE &&
        rec.magic == OS_NVM_CALIB_MAGIC && rec.calibration.valid) {
        g_os.calibration = rec.calibration;
    }

    for (uint8_t ch = 0; ch < OS_CHANNEL_COUNT; ch++) (void)os_hal_comm_init(ch);

    for (uint8_t axis = 0; axis < OS_AXIS_COUNT; axis++) {
        err = os_hal_motor_init(axis);
        if (err != OS_ERR_NONE) { g_os.state = OS_STATE_FAULT; return err; }
        (void)os_hal_motor_set_frequency(axis, 0u);
        (void)os_hal_motor_enable(axis, false);
    }

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    err = os_hal_timer_motor_init();
    if (err != OS_ERR_NONE) { g_os.state = OS_STATE_FAULT; return err; }

    os_site_info_t gps;
    if (os_hal_gps_poll(&gps) == OS_ERR_NONE && gps.valid) {
        g_os.site = gps;
        g_os.gps_locked = true;
        (void)os_hal_rtc_set(gps.utc_epoch_seconds);
    } else {
        uint32_t utc;
        if (os_hal_rtc_read(&utc) == OS_ERR_NONE) g_os.site.utc_epoch_seconds = utc;
        g_os.gps_locked = false;
    }

    g_os.state = OS_STATE_IDLE_TRACKING;
    apply_tracking();
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    service_comm();

    os_site_info_t gps;
    if (os_hal_gps_poll(&gps) == OS_ERR_NONE && gps.valid) {
        g_os.site = gps;
        g_os.gps_locked = true;
        (void)os_hal_rtc_set(gps.utc_epoch_seconds);
    }

    for (uint8_t axis = 0; axis < OS_AXIS_COUNT; axis++) {
        if (os_hal_limit_is_triggered(axis)) {
            stop_axis(axis);
            if (g_os.state == OS_STATE_GOTO || g_os.state == OS_STATE_MANUAL_MOTION) {
                g_os.state = OS_STATE_FAULT;
                g_os.moving = false;
            }
        }
    }

    service_goto();
    service_guide();

    if (g_os.state == OS_STATE_MANUAL_MOTION) {
        uint8_t axis = (g_os.manual_direction == OS_DIRECTION_EAST || g_os.manual_direction == OS_DIRECTION_WEST) ? OS_AXIS_RA : OS_AXIS_DEC;
        if (!os_hal_limit_is_triggered(axis)) advance_host_motor(axis, OS_LOOP_DT_MS);
    } else if (g_os.state == OS_STATE_IDLE_TRACKING) {
        apply_tracking();
        advance_host_motor(OS_AXIS_RA, OS_LOOP_DT_MS);
    }

    g_os.current_coord = steps_to_coord(os_hal_motor_get_position(OS_AXIS_RA),
                                        os_hal_motor_get_position(OS_AXIS_DEC));
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (command == NULL || reply_buffer == NULL || reply_length == NULL || reply_buffer_size == 0u) return OS_ERR_INVALID_ARGUMENT;
    *reply_length = 0u;
    if (source_channel >= OS_CHANNEL_COUNT || length < 3u || length > OS_MAX_COMMAND_LENGTH) return OS_ERR_INVALID_ARGUMENT;
    if (command[0] != OS_LX200_CMD_PREFIX || command[length - 1u] != OS_LX200_CMD_SUFFIX) return OS_ERR_COMMAND_FORMAT;

    os_error_t err = OS_ERR_NONE;
    if (strncmp(command, ":GVP#", 5u) == 0) {
        *reply_length = put_reply(reply_buffer, reply_buffer_size, "OnStep-FSE#");
    } else if (strncmp(command, ":GR#", 4u) == 0) {
        snprintf(reply_buffer, reply_buffer_size, "%02d:%02d:%02d#",
                 (int)g_os.current_coord.ra_hours,
                 (int)((g_os.current_coord.ra_hours - (int)g_os.current_coord.ra_hours) * 60.0f),
                 0);
        *reply_length = strlen(reply_buffer);
    } else if (strncmp(command, ":GD#", 4u) == 0) {
        snprintf(reply_buffer, reply_buffer_size, "%+03d*%02d:00#",
                 (int)g_os.current_coord.dec_degrees,
                 (int)((g_os.current_coord.dec_degrees - (int)g_os.current_coord.dec_degrees) * 60.0f));
        *reply_length = strlen(reply_buffer);
    } else if (strncmp(command, ":D#", 3u) == 0) {
        *reply_length = put_reply(reply_buffer, reply_buffer_size, g_os.moving ? "|" : "#");
    } else if (strncmp(command, ":MS#", 4u) == 0) {
        err = os_goto_equatorial(g_os.pending_set_coord);
        *reply_length = put_reply(reply_buffer, reply_buffer_size, err == OS_ERR_NONE ? "0#" : "1#");
    } else if (length >= 5u && command[1] == 'M') {
        os_direction_t d;
        if (command[2] == 'e') d = OS_DIRECTION_EAST;
        else if (command[2] == 'w') d = OS_DIRECTION_WEST;
        else if (command[2] == 'n') d = OS_DIRECTION_NORTH;
        else if (command[2] == 's') d = OS_DIRECTION_SOUTH;
        else return OS_ERR_COMMAND_FORMAT;
        err = os_move_start(d, OS_SPEED_MEDIUM);
        *reply_length = put_reply(reply_buffer, reply_buffer_size, err == OS_ERR_NONE ? "1#" : "0#");
    } else if (strncmp(command, ":Q#", 3u) == 0) {
        err = os_move_stop();
        *reply_length = put_reply(reply_buffer, reply_buffer_size, "1#");
    } else if (strncmp(command, ":hP#", 4u) == 0) {
        err = os_park();
        *reply_length = put_reply(reply_buffer, reply_buffer_size, err == OS_ERR_NONE ? "1#" : "0#");
    } else if (strncmp(command, ":hO#", 4u) == 0) {
        err = os_unpark();
        *reply_length = put_reply(reply_buffer, reply_buffer_size, err == OS_ERR_NONE ? "1#" : "0#");
    } else if (strncmp(command, ":Sr", 3u) == 0) {
        int h = 0, m = 0, s = 0;
        if (sscanf(command, ":Sr%d:%d:%d#", &h, &m, &s) != 3) return OS_ERR_COMMAND_FORMAT;
        g_os.pending_set_coord.ra_hours = (float)h + (float)m / 60.0f + (float)s / 3600.0f;
        err = valid_ra(g_os.pending_set_coord.ra_hours) ? OS_ERR_NONE : OS_ERR_INVALID_ARGUMENT;
        *reply_length = put_reply(reply_buffer, reply_buffer_size, err == OS_ERR_NONE ? "1#" : "0#");
    } else if (strncmp(command, ":Sd", 3u) == 0) {
        int d = 0, m = 0, sign = 1;
        char sg = '+';
        if (sscanf(command, ":Sd%c%d*%d", &sg, &d, &m) < 3) return OS_ERR_COMMAND_FORMAT;
        if (sg == '-') sign = -1;
        g_os.pending_set_coord.dec_degrees = (float)sign * ((float)d + (float)m / 60.0f);
        err = valid_dec(g_os.pending_set_coord.dec_degrees) ? OS_ERR_NONE : OS_ERR_INVALID_ARGUMENT;
        *reply_length = put_reply(reply_buffer, reply_buffer_size, err == OS_ERR_NONE ? "1#" : "0#");
    } else {
        err = OS_ERR_COMMAND_FORMAT;
    }
    return err;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) { return start_goto_to(target, false); }

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) return OS_ERR_INVALID_ARGUMENT;
    os_equatorial_coord_t eq;
    eq.ra_hours = target.azimuth_degrees / 15.0f;
    eq.dec_degrees = target.altitude_degrees;
    return start_goto_to(eq, false);
}

os_error_t os_goto_abort(void) {
    if (g_os.state != OS_STATE_GOTO) return OS_ERR_INVALID_STATE;
    stop_all_motion();
    g_os.state = OS_STATE_IDLE_TRACKING;
    apply_tracking();
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (!valid_track(rate)) return OS_ERR_INVALID_ARGUMENT;
    if (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f) return OS_ERR_INVALID_ARGUMENT;
    g_os.track_rate = rate;
    g_os.custom_track_factor = custom_factor;
    apply_tracking();
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (rate == NULL || custom_factor == NULL) return OS_ERR_INVALID_ARGUMENT;
    *rate = g_os.track_rate;
    *custom_factor = g_os.custom_track_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) { g_os.tracking_enabled = true; apply_tracking(); return OS_ERR_NONE; }

os_error_t os_tracking_disable(void) {
    g_os.tracking_enabled = false;
    stop_axis(OS_AXIS_RA);
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (!valid_direction(direction) || duration_ms == 0u) return OS_ERR_INVALID_ARGUMENT;
    g_os.guide.active = true;
    g_os.guide.duration_ms = duration_ms;
    g_os.guide.rate_fraction = g_os.guide_rate;
    g_os.guide.direction_east = direction == OS_DIRECTION_EAST;
    g_os.guide.direction_north = direction == OS_DIRECTION_NORTH;
    g_os.guide.dec_priority = direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH;
    g_os.guide_remaining_ms = duration_ms;
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) return OS_ERR_INVALID_ARGUMENT;
    g_os.guide_rate = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (pulse == NULL) return OS_ERR_INVALID_ARGUMENT;
    *pulse = g_os.guide;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (!(mode == OS_ALIGN_1STAR || mode == OS_ALIGN_2STAR || mode == OS_ALIGN_3STAR || mode == OS_ALIGN_NSTAR)) return OS_ERR_INVALID_ARGUMENT;
    g_os.align_mode = mode;
    g_os.align_count = 0u;
    g_os.residual_valid = false;
    memset(g_os.align_star, 0, sizeof(g_os.align_star));
    memset(g_os.align_pos, 0, sizeof(g_os.align_pos));
    g_os.state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord, os_motor_position_t motor_pos) {
    if (!valid_eq(star_coord)) return OS_ERR_INVALID_ARGUMENT;
    if (g_os.state != OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE;
    if (g_os.align_count >= OS_CALIBRATION_MAX_STARS) return OS_ERR_INVALID_STATE;
    g_os.align_star[g_os.align_count] = star_coord;
    g_os.align_pos[g_os.align_count] = motor_pos;
    g_os.align_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    uint8_t need = 3u;
    if (g_os.align_mode == OS_ALIGN_1STAR) need = 1u;
    else if (g_os.align_mode == OS_ALIGN_2STAR) need = 2u;
    if (g_os.align_count < need) return OS_ERR_INVALID_STATE;

    os_calibration_t c;
    memset(&c, 0, sizeof(c));

    if (g_os.align_mode == OS_ALIGN_1STAR) {
        c.matrix_ra_to_ra = 1.0f; c.matrix_dec_to_dec = 1.0f;
        c.offset_ra_arcsec = (float)((double)g_os.align_pos[0].ra_steps * 3.6 -
                           (double)g_os.align_star[0].ra_hours * 15.0 * 3600.0);
        c.offset_dec_arcsec = (float)((double)g_os.align_pos[0].dec_steps * 3.6 -
                            (double)g_os.align_star[0].dec_degrees * 3600.0);
        g_os.residual_arcsec = 0.0f;
    } else if (g_os.align_mode == OS_ALIGN_2STAR) {
        double x0 = (double)g_os.align_star[0].ra_hours * 15.0 * 3600.0;
        double x1 = (double)g_os.align_star[1].ra_hours * 15.0 * 3600.0;
        double y0 = (double)g_os.align_star[0].dec_degrees * 3600.0;
        double y1 = (double)g_os.align_star[1].dec_degrees * 3600.0;
        if ((x1 - x0 < 1e-9 && x1 - x0 > -1e-9) || (y1 - y0 < 1e-9 && y1 - y0 > -1e-9)) return OS_ERR_CALIBRATION_FAILED;
        c.matrix_ra_to_ra = (float)(((double)g_os.align_pos[1].ra_steps - g_os.align_pos[0].ra_steps) * 3.6 / (x1 - x0));
        c.matrix_dec_to_dec = (float)(((double)g_os.align_pos[1].dec_steps - g_os.align_pos[0].dec_steps) * 3.6 / (y1 - y0));
        c.offset_ra_arcsec = (float)((double)g_os.align_pos[0].ra_steps * 3.6 - c.matrix_ra_to_ra * x0);
        c.offset_dec_arcsec = (float)((double)g_os.align_pos[0].dec_steps * 3.6 - c.matrix_dec_to_dec * y0);
        g_os.residual_arcsec = 0.0f;
    } else {
        double ra_params[3] = {0}, dec_params[3] = {0}, resid = 0.0;
        if (!affine_fit(g_os.align_count, ra_params, dec_params, &resid)) return OS_ERR_CALIBRATION_FAILED;
        c.matrix_ra_to_ra = (float)ra_params[0];
        c.matrix_dec_to_ra = (float)ra_params[1];
        c.offset_ra_arcsec = (float)ra_params[2];
        c.matrix_ra_to_dec = (float)dec_params[0];
        c.matrix_dec_to_dec = (float)dec_params[1];
        c.offset_dec_arcsec = (float)dec_params[2];
        g_os.residual_arcsec = (float)resid;
    }

    c.valid = true;
    g_os.calibration = c;
    g_os.residual_valid = true;
    save_calibration();
    g_os.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (!g_os.residual_valid) return OS_ERR_INVALID_STATE;
    *residual_arcsec = g_os.residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    g_os.align_count = 0u;
    g_os.residual_valid = false;
    g_os.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void) { return start_goto_to(g_os.park_coord, true); }

os_error_t os_unpark(void) {
    if (g_os.state != OS_STATE_PARKED) return OS_ERR_INVALID_STATE;
    for (uint8_t axis = 0; axis < OS_AXIS_COUNT; axis++) (void)os_hal_motor_enable(axis, true);
    uint32_t utc;
    if (os_hal_rtc_read(&utc) == OS_ERR_NONE) g_os.site.utc_epoch_seconds = utc;
    g_os.tracking_enabled = true;
    g_os.state = OS_STATE_IDLE_TRACKING;
    apply_tracking();
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!valid_eq(park_pos)) return OS_ERR_INVALID_ARGUMENT;
    g_os.park_coord = park_pos;
    g_os.park_custom = true;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (!valid_direction(direction) || !valid_speed(speed)) return OS_ERR_INVALID_ARGUMENT;
    uint8_t axis = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) ? OS_AXIS_RA : OS_AXIS_DEC;
    if (os_hal_limit_is_triggered(axis)) {
        g_os.state = OS_STATE_FAULT;
        return OS_ERR_LIMIT_TRIGGERED;
    }
    uint32_t freq = OS_MANUAL_MEDIUM_HZ;
    if (speed == OS_SPEED_SLOW) freq = OS_MANUAL_SLOW_HZ;
    else if (speed == OS_SPEED_FAST) freq = OS_MANUAL_FAST_HZ;
    else if (speed == OS_SPEED_CUSTOM) freq = (uint32_t)(g_os.custom_manual_arcsec_per_sec > 0.0f ? g_os.custom_manual_arcsec_per_sec : OS_MANUAL_MEDIUM_HZ);

    g_os.manual_direction = direction;
    g_os.manual_speed = speed;
    g_os.moving = true;
    g_os.state = OS_STATE_MANUAL_MOTION;
    (void)os_hal_motor_set_direction(axis, direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_NORTH);
    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_frequency(axis, freq);
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (g_os.state == OS_STATE_MANUAL_MOTION) {
        stop_all_motion();
        g_os.state = OS_STATE_IDLE_TRACKING;
        apply_tracking();
    }
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (arcsec_per_sec <= 0.0f) return OS_ERR_INVALID_ARGUMENT;
    g_os.custom_manual_arcsec_per_sec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (state == NULL) return OS_ERR_INVALID_ARGUMENT;
    *state = g_os.state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (coord == NULL) return OS_ERR_INVALID_ARGUMENT;
    *coord = g_os.current_coord;
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (site == NULL) return OS_ERR_INVALID_ARGUMENT;
    *site = g_os.site;
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
    *moving = g_os.moving || g_os.state == OS_STATE_GOTO || g_os.state == OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (locked == NULL) return OS_ERR_INVALID_ARGUMENT;
    *locked = g_os.gps_locked;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    g_os.pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL) return OS_ERR_INVALID_ARGUMENT;
    g_os.pec = *table;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (table == NULL) return OS_ERR_INVALID_ARGUMENT;
    *table = g_os.pec;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) return OS_ERR_INVALID_ARGUMENT;
    int idx = (worm_phase_deg >= 360.0f) ? 0 : (int)worm_phase_deg;
    g_os.pec.corrections[idx] = error_arcsec;
    g_os.pec.valid = true;
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (calib == NULL) return OS_ERR_INVALID_ARGUMENT;
    *calib = g_os.calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    memset(&g_os.calibration, 0, sizeof(g_os.calibration));
    g_os.residual_valid = false;
    save_calibration();
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_init(uint8_t axis) {
    if (axis >= OS_AXIS_COUNT) return OS_ERR_INVALID_ARGUMENT;
    memset(&g_motor[axis], 0, sizeof(g_motor[axis]));
    g_motor[axis].initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (axis >= OS_AXIS_COUNT) return OS_ERR_INVALID_ARGUMENT;
    if (frequency_hz > 20000u) frequency_hz = 20000u;
    g_motor[axis].frequency_hz = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (axis >= OS_AXIS_COUNT) return OS_ERR_INVALID_ARGUMENT;
    g_motor[axis].direction = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (axis >= OS_AXIS_COUNT) return OS_ERR_INVALID_ARGUMENT;
    g_motor[axis].enabled = enable;
    if (!enable) g_motor[axis].frequency_hz = 0u;
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (axis >= OS_AXIS_COUNT) return 0;
    return g_motor[axis].position_steps;
}

os_error_t os_hal_gps_init(void) { return OS_ERR_NONE; }

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (!g_gps_has_sample) {
        site->valid = false;
        return OS_ERR_GPS_NO_SIGNAL;
    }
    *site = g_gps_sample;
    return site->valid ? OS_ERR_NONE : OS_ERR_GPS_NO_SIGNAL;
}

os_error_t os_hal_rtc_init(void) { return OS_ERR_NONE; }

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) return OS_ERR_INVALID_ARGUMENT;
    *utc_epoch_seconds = g_rtc_epoch;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    g_rtc_epoch = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    g_limit[0] = false;
    g_limit[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (axis >= OS_AXIS_COUNT) return true;
    return g_limit[axis];
}

os_error_t os_hal_nvm_init(void) {
    if (!g_nvm_initialized) {
        memset(g_nvm, 0, sizeof(g_nvm));
        g_nvm_initialized = true;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (data == NULL) return OS_ERR_INVALID_ARGUMENT;
    if ((uint32_t)offset + (uint32_t)length > OS_NVM_BYTES) return OS_ERR_INVALID_ARGUMENT;
    memcpy(data, &g_nvm[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (data == NULL) return OS_ERR_INVALID_ARGUMENT;
    if ((uint32_t)offset + (uint32_t)length > OS_NVM_BYTES) return OS_ERR_INVALID_ARGUMENT;
    memcpy(&g_nvm[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (channel >= OS_CHANNEL_COUNT) return OS_ERR_INVALID_ARGUMENT;
    g_rx_head[channel] = g_rx_tail[channel] = 0u;
    g_tx_len[channel] = 0u;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if (channel >= OS_CHANNEL_COUNT) return 0;
    if (g_rx_tail[channel] >= g_rx_head[channel]) return (int16_t)(g_rx_tail[channel] - g_rx_head[channel]);
    return (int16_t)(sizeof(g_rx[channel]) - g_rx_head[channel] + g_rx_tail[channel]);
}

char os_hal_comm_read(uint8_t channel) {
    if (channel >= OS_CHANNEL_COUNT || os_hal_comm_available(channel) <= 0) return '\0';
    char c = g_rx[channel][g_rx_head[channel]];
    g_rx_head[channel] = (g_rx_head[channel] + 1u) % sizeof(g_rx[channel]);
    return c;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (data == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (channel >= OS_CHANNEL_COUNT) return OS_ERR_INVALID_ARGUMENT;
    size_t room = sizeof(g_tx[channel]) - g_tx_len[channel];
    if (length > room) length = room;
    memcpy(&g_tx[channel][g_tx_len[channel]], data, length);
    g_tx_len[channel] += length;
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) {
    g_buzzer_duration = duration_ms;
    g_buzzer_count = count;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void) { return OS_ERR_NONE; }

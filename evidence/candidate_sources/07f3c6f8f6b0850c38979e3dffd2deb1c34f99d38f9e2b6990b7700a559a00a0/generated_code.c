#include "generated_code.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    os_config_t cfg;
    os_status_t st;
    os_align_mode_t align_mode;
    os_align_star_t stars[OS_MAX_ALIGN_STARS];
    char rx[OS_CHANNEL_COUNT][OS_MAX_COMMAND_LENGTH];
    uint8_t rx_len[OS_CHANNEL_COUNT];
    double custom_tracking_hz;
    double pec[OS_PEC_TABLE_SIZE + 1];
    bool pec_valid[OS_PEC_TABLE_SIZE + 1];
    int manual_axis;
    int manual_dir;
} os_context_t;

static os_context_t g;

static bool valid_axis(uint8_t axis) { return axis < OS_AXIS_COUNT; }
static bool valid_channel(uint8_t channel) { return channel < OS_CHANNEL_COUNT; }
static bool valid_ra_dec(double ra, double dec) { return ra >= 0.0 && ra <= 24.0 && dec >= -90.0 && dec <= 90.0; }

static os_error_t stop_axis(uint8_t axis) {
    os_error_t e;
    if (!valid_axis(axis)) return OS_ERR_INVALID_ARGUMENT;
    e = os_hal_motor_set_frequency(axis, 0.0);
    (void)os_hal_motor_enable(axis, false);
    return e;
}

static void stop_all(void) {
    (void)stop_axis(OS_AXIS_RA);
    (void)stop_axis(OS_AXIS_DEC);
}

static double cfg_steps_per_degree(uint8_t axis) {
    if (!valid_axis(axis) || g.cfg.steps_per_degree[axis] <= 0.0) return 1000.0;
    return g.cfg.steps_per_degree[axis];
}

static int32_t coord_to_axis0_steps(double ra_hours) {
    double deg = ra_hours * 15.0;
    if (g.st.alignment.valid) return (int32_t)llround(g.st.alignment.m00 * deg + g.st.alignment.m01 * g.st.site.latitude_degrees + g.st.alignment.b0);
    return (int32_t)llround(deg * cfg_steps_per_degree(OS_AXIS_RA));
}

static int32_t coord_to_axis1_steps(double dec_degrees) {
    if (g.st.alignment.valid) return (int32_t)llround(g.st.alignment.m10 * dec_degrees + g.st.alignment.m11 * g.st.site.longitude_degrees + g.st.alignment.b1);
    return (int32_t)llround(dec_degrees * cfg_steps_per_degree(OS_AXIS_DEC));
}

static bool limit_blocks_axis(uint8_t axis, int32_t delta) {
    (void)delta;
    if (!valid_axis(axis)) return true;
    return os_hal_limit_is_triggered(axis);
}

static os_error_t start_axis_toward(uint8_t axis, int32_t target, double hz) {
    int32_t cur;
    int32_t delta;
    if (!valid_axis(axis) || hz < 0.0) return OS_ERR_INVALID_ARGUMENT;
    cur = os_hal_motor_get_position(axis);
    delta = target - cur;
    if (delta == 0) {
        return stop_axis(axis);
    }
    if (limit_blocks_axis(axis, delta)) {
        (void)stop_axis(axis);
        g.st.last_error = OS_ERR_LIMIT_TRIGGERED;
        g.st.state = OS_STATE_FAULT;
        return OS_ERR_LIMIT_TRIGGERED;
    }
    (void)os_hal_motor_set_direction(axis, delta > 0);
    (void)os_hal_motor_enable(axis, true);
    return os_hal_motor_set_frequency(axis, hz);
}

static void update_positions(void) {
    g.st.current_steps[OS_AXIS_RA] = os_hal_motor_get_position(OS_AXIS_RA);
    g.st.current_steps[OS_AXIS_DEC] = os_hal_motor_get_position(OS_AXIS_DEC);
}

static void apply_tracking(void) {
    double hz = g.cfg.tracking_rate_hz > 0.0 ? g.cfg.tracking_rate_hz : 1.0;
    if (g.st.tracking_rate == OS_TRACK_LUNAR) hz *= 0.966;
    else if (g.st.tracking_rate == OS_TRACK_SOLAR) hz *= 0.9973;
    else if (g.st.tracking_rate == OS_TRACK_CUSTOM) hz = g.custom_tracking_hz;
    if (hz < 0.0) hz = 0.0;
    if (!os_hal_limit_is_triggered(OS_AXIS_RA)) {
        (void)os_hal_motor_set_direction(OS_AXIS_RA, true);
        (void)os_hal_motor_enable(OS_AXIS_RA, true);
        (void)os_hal_motor_set_frequency(OS_AXIS_RA, hz);
    } else {
        (void)stop_axis(OS_AXIS_RA);
        g.st.state = OS_STATE_FAULT;
        g.st.last_error = OS_ERR_LIMIT_TRIGGERED;
    }
}

static void poll_time_sources(void) {
    os_site_info_t site;
    uint32_t utc;
    memset(&site, 0, sizeof(site));
    if (g.cfg.gps_enabled && os_hal_gps_poll(&site) == OS_ERR_NONE && site.valid &&
        site.latitude_degrees >= -90.0 && site.latitude_degrees <= 90.0 &&
        site.longitude_degrees >= -180.0 && site.longitude_degrees <= 180.0) {
        g.st.site = site;
        g.st.gps_locked = true;
        (void)os_hal_rtc_set(site.utc_epoch_seconds);
    } else {
        g.st.gps_locked = false;
        if (os_hal_rtc_read(&utc) == OS_ERR_NONE) g.st.site.utc_epoch_seconds = utc;
    }
}

static os_error_t write_reply(uint8_t ch, const char *s) {
    if (!valid_channel(ch) || s == NULL) return OS_ERR_INVALID_ARGUMENT;
    return os_hal_comm_write(ch, (const uint8_t *)s, strlen(s));
}

os_error_t os_init(const os_config_t *config) {
    uint8_t i;
    os_alignment_t persisted;
    if (config == NULL) return OS_ERR_INVALID_ARGUMENT;

    memset(&g, 0, sizeof(g));
    g.cfg = *config;
    g.st.state = OS_STATE_INIT;
    g.st.tracking_rate = OS_TRACK_SIDEREAL;
    g.st.site.latitude_degrees = 0.0;
    g.st.site.longitude_degrees = 0.0;
    g.st.site.elevation_metres = 0.0;
    g.st.site.valid = false;
    g.st.calibration_residual_computed = false;
    g.st.align_star_count = 0;
    g.st.guide_active[0] = g.st.guide_active[1] = g.st.guide_active[2] = g.st.guide_active[3] = false;
    g.st.goto_active = false;
    g.st.manual_active = false;
    g.custom_tracking_hz = g.cfg.tracking_rate_hz;

    (void)os_hal_nvm_init();
    memset(&persisted, 0, sizeof(persisted));
    if (os_hal_nvm_read(OS_NVM_CALIBRATION_OFFSET, &persisted, sizeof(persisted)) == OS_ERR_NONE && persisted.valid) {
        g.st.alignment = persisted;
    }

    for (i = 0; i < OS_CHANNEL_COUNT; ++i) {
        if (g.cfg.channel_enabled[i]) (void)os_hal_comm_init(i);
    }

    for (i = 0; i < OS_AXIS_COUNT; ++i) {
        os_error_t e = os_hal_motor_init(i);
        (void)os_hal_motor_set_frequency(i, 0.0);
        (void)os_hal_motor_enable(i, false);
        if (e != OS_ERR_NONE) {
            g.st.last_error = e;
            g.st.state = OS_STATE_FAULT;
            return e;
        }
    }

    if (g.cfg.gps_enabled) (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    (void)os_hal_timer_motor_init();
    poll_time_sources();

    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        g.st.state = OS_STATE_FAULT;
        g.st.last_error = OS_ERR_LIMIT_TRIGGERED;
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g.st.initialized = true;
    g.st.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_loop_iteration(void) {
    uint8_t ch;
    if (!g.st.initialized) return OS_ERR_INVALID_STATE;

    poll_time_sources();
    update_positions();

    for (ch = 0; ch < OS_CHANNEL_COUNT; ++ch) {
        int avail;
        if (!g.cfg.channel_enabled[ch]) continue;
        avail = os_hal_comm_available(ch);
        while (avail-- > 0) {
            int c = os_hal_comm_read(ch);
            if (c < 0) break;
            if (c == ':') g.rx_len[ch] = 0;
            if (g.rx_len[ch] < OS_MAX_COMMAND_LENGTH - 1) g.rx[ch][g.rx_len[ch]++] = (char)c;
            else {
                g.rx_len[ch] = 0;
                (void)write_reply(ch, "0#");
                continue;
            }
            if (c == '#') {
                char reply[OS_MAX_REPLY_LENGTH];
                g.rx[ch][g.rx_len[ch]] = '\0';
                memset(reply, 0, sizeof(reply));
                (void)os_process_command(ch, g.rx[ch], reply, sizeof(reply));
                if (reply[0] != '\0') (void)write_reply(ch, reply);
                g.rx_len[ch] = 0;
            }
        }
    }

    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        stop_all();
        g.st.goto_active = false;
        g.st.manual_active = false;
        g.st.state = OS_STATE_FAULT;
        g.st.last_error = OS_ERR_LIMIT_TRIGGERED;
        return OS_ERR_LIMIT_TRIGGERED;
    }

    if (g.st.state == OS_STATE_GOTO) {
        bool done = true;
        uint8_t axis;
        for (axis = 0; axis < OS_AXIS_COUNT; ++axis) {
            int32_t cur = os_hal_motor_get_position(axis);
            int32_t delta = g.st.target_steps[axis] - cur;
            double hz = g.cfg.max_goto_rate_hz[axis] > 0.0 ? g.cfg.max_goto_rate_hz[axis] : 200.0;
            if (delta < 0) delta = -delta;
            if (delta > 0) {
                done = false;
                if (delta < 100) hz *= 0.25;
                (void)start_axis_toward(axis, g.st.target_steps[axis], hz);
            } else {
                (void)stop_axis(axis);
            }
        }
        if (done) {
            stop_all();
            g.st.goto_active = false;
            g.st.state = OS_STATE_IDLE_TRACKING;
            (void)os_hal_buzzer_beep(100, 2);
            apply_tracking();
        }
    } else if (g.st.state == OS_STATE_IDLE_TRACKING) {
        apply_tracking();
    }

    return OS_ERR_NONE;
}

os_error_t os_process_command(uint8_t source_channel, const char *command, char *reply, size_t reply_size) {
    double a, b;
    os_error_t e = OS_ERR_NONE;
    if (!valid_channel(source_channel) || command == NULL || reply == NULL || reply_size == 0) return OS_ERR_INVALID_ARGUMENT;
    reply[0] = '\0';
    if (command[0] != ':' || command[strlen(command) - 1u] != '#') {
        snprintf(reply, reply_size, "0#");
        return OS_ERR_COMMAND_FORMAT;
    }

    if (strncmp(command, ":GVP#", 5) == 0) snprintf(reply, reply_size, "OnStep-C11#");
    else if (strncmp(command, ":GU#", 4) == 0) snprintf(reply, reply_size, "%d#", (int)g.st.state);
    else if (strncmp(command, ":D#", 3) == 0) snprintf(reply, reply_size, "%c#", os_query_is_moving() ? '1' : '0');
    else if (strncmp(command, ":MS#", 4) == 0) {
        e = os_goto_equatorial((double)g.st.target_steps[0] / (15.0 * cfg_steps_per_degree(0)),
                               (double)g.st.target_steps[1] / cfg_steps_per_degree(1));
        snprintf(reply, reply_size, "%d#", e == OS_ERR_NONE ? 0 : 1);
    } else if (sscanf(command, ":Sr%lf#", &a) == 1) {
        if (a < 0.0 || a > 24.0) e = OS_ERR_INVALID_ARGUMENT;
        else {
            g.st.target_steps[0] = coord_to_axis0_steps(a);
            snprintf(reply, reply_size, "1#");
        }
    } else if (sscanf(command, ":Sd%lf#", &b) == 1) {
        if (b < -90.0 || b > 90.0) e = OS_ERR_INVALID_ARGUMENT;
        else {
            g.st.target_steps[1] = coord_to_axis1_steps(b);
            snprintf(reply, reply_size, "1#");
        }
    } else if (strncmp(command, ":Me#", 4) == 0) e = os_manual_move(OS_DIR_EAST, OS_RATE_MEDIUM, 0.0);
    else if (strncmp(command, ":Mw#", 4) == 0) e = os_manual_move(OS_DIR_WEST, OS_RATE_MEDIUM, 0.0);
    else if (strncmp(command, ":Mn#", 4) == 0) e = os_manual_move(OS_DIR_NORTH, OS_RATE_MEDIUM, 0.0);
    else if (strncmp(command, ":Ms#", 4) == 0) e = os_manual_move(OS_DIR_SOUTH, OS_RATE_MEDIUM, 0.0);
    else if (strncmp(command, ":Q#", 3) == 0) e = os_manual_stop();
    else if (strncmp(command, ":hP#", 4) == 0) e = os_park();
    else if (strncmp(command, ":hO#", 4) == 0) e = os_unpark();
    else e = OS_ERR_COMMAND_FORMAT;

    if (reply[0] == '\0') snprintf(reply, reply_size, "%d#", e == OS_ERR_NONE ? 1 : 0);
    return e;
}

os_error_t os_goto_equatorial(double ra_hours, double dec_degrees) {
    uint8_t axis;
    if (!valid_ra_dec(ra_hours, dec_degrees)) return OS_ERR_INVALID_ARGUMENT;
    if (!g.st.initialized || g.st.state == OS_STATE_PARKED || g.st.state == OS_STATE_FAULT) return OS_ERR_INVALID_STATE;

    g.st.target_steps[OS_AXIS_RA] = coord_to_axis0_steps(ra_hours);
    g.st.target_steps[OS_AXIS_DEC] = coord_to_axis1_steps(dec_degrees);

    for (axis = 0; axis < OS_AXIS_COUNT; ++axis) {
        int32_t delta = g.st.target_steps[axis] - os_hal_motor_get_position(axis);
        if (limit_blocks_axis(axis, delta)) return OS_ERR_LIMIT_TRIGGERED;
    }

    g.st.goto_active = true;
    g.st.manual_active = false;
    g.st.state = OS_STATE_GOTO;
    for (axis = 0; axis < OS_AXIS_COUNT; ++axis) {
        double hz = g.cfg.max_goto_rate_hz[axis] > 0.0 ? g.cfg.max_goto_rate_hz[axis] : 200.0;
        (void)start_axis_toward(axis, g.st.target_steps[axis], hz);
    }
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (!g.st.initialized) return OS_ERR_INVALID_STATE;
    stop_all();
    g.st.goto_active = false;
    g.st.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

bool os_query_is_moving(void) {
    return g.st.goto_active || g.st.manual_active || g.st.state == OS_STATE_GOTO || g.st.state == OS_STATE_MANUAL_MOTION;
}

os_error_t os_set_tracking_rate(os_tracking_rate_t rate, double custom_hz) {
    if (rate < OS_TRACK_SIDEREAL || rate > OS_TRACK_CUSTOM || (rate == OS_TRACK_CUSTOM && custom_hz < 0.0)) return OS_ERR_INVALID_ARGUMENT;
    g.st.tracking_rate = rate;
    g.custom_tracking_hz = custom_hz;
    if (g.st.state == OS_STATE_IDLE_TRACKING) apply_tracking();
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_DIR_EAST || direction > OS_DIR_SOUTH || duration_ms == 0u) return OS_ERR_INVALID_ARGUMENT;
    if (!g.st.initialized || g.st.state == OS_STATE_PARKED || g.st.state == OS_STATE_FAULT) return OS_ERR_INVALID_STATE;
    g.st.guide_active[direction] = true;
    g.st.guide_remaining_ms[direction] = duration_ms;
    return OS_ERR_NONE;
}

os_error_t os_align_reset(os_align_mode_t mode) {
    if (mode != OS_ALIGN_ONE_STAR && mode != OS_ALIGN_TWO_STAR && mode != OS_ALIGN_THREE_STAR && mode != OS_ALIGN_MULTI_STAR) return OS_ERR_INVALID_ARGUMENT;
    memset(g.stars, 0, sizeof(g.stars));
    g.align_mode = mode;
    g.st.align_star_count = 0;
    g.st.calibration_residual_computed = false;
    g.st.alignment.valid = false;
    g.st.state = OS_STATE_ALIGNING;
    return OS_ERR_NONE;
}

os_error_t os_align_add_star(double ra_hours, double dec_degrees, int32_t axis0_steps, int32_t axis1_steps) {
    os_align_star_t *s;
    if (!valid_ra_dec(ra_hours, dec_degrees)) return OS_ERR_INVALID_ARGUMENT;
    if (g.st.align_star_count >= OS_MAX_ALIGN_STARS) return OS_ERR_NO_MEMORY;
    s = &g.stars[g.st.align_star_count++];
    s->sky.ra_hours = ra_hours;
    s->sky.dec_degrees = dec_degrees;
    s->motor_steps[0] = axis0_steps;
    s->motor_steps[1] = axis1_steps;
    return OS_ERR_NONE;
}

static double det3(double a[3][3]) {
    return a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
         - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
         + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
}

static bool solve3(double a[3][3], double b[3], double x[3]) {
    int i, r, c, piv;
    double m[3][4];
    for (r = 0; r < 3; ++r) {
        for (c = 0; c < 3; ++c) m[r][c] = a[r][c];
        m[r][3] = b[r];
    }
    for (i = 0; i < 3; ++i) {
        double maxv = fabs(m[i][i]);
        piv = i;
        for (r = i + 1; r < 3; ++r) {
            if (fabs(m[r][i]) > maxv) {
                maxv = fabs(m[r][i]);
                piv = r;
            }
        }
        if (maxv < 1e-12) return false;
        if (piv != i) {
            for (c = i; c < 4; ++c) {
                double t = m[i][c];
                m[i][c] = m[piv][c];
                m[piv][c] = t;
            }
        }
        for (r = 0; r < 3; ++r) {
            double f;
            if (r == i) continue;
            f = m[r][i] / m[i][i];
            for (c = i; c < 4; ++c) m[r][c] -= f * m[i][c];
        }
    }
    for (i = 0; i < 3; ++i) x[i] = m[i][3] / m[i][i];
    return true;
}

os_error_t os_align_compute(void) {
    uint8_t i, min_stars;
    double n00[3][3] = {{0}}, n11[3][3] = {{0}}, b0[3] = {0}, b1[3] = {0}, x0[3], x1[3];
    if (g.align_mode == OS_ALIGN_ONE_STAR) min_stars = 1;
    else if (g.align_mode == OS_ALIGN_TWO_STAR) min_stars = 2;
    else min_stars = 3;
    if (g.st.align_star_count < min_stars) return OS_ERR_INVALID_STATE;

    if (g.align_mode == OS_ALIGN_ONE_STAR) {
        double ra_deg = g.stars[0].sky.ra_hours * 15.0;
        g.st.alignment.m00 = cfg_steps_per_degree(0);
        g.st.alignment.m01 = 0.0;
        g.st.alignment.m10 = 0.0;
        g.st.alignment.m11 = cfg_steps_per_degree(1);
        g.st.alignment.b0 = g.stars[0].motor_steps[0] - ra_deg * g.st.alignment.m00;
        g.st.alignment.b1 = g.stars[0].motor_steps[1] - g.stars[0].sky.dec_degrees * g.st.alignment.m11;
    } else if (g.align_mode == OS_ALIGN_TWO_STAR) {
        double xra0 = g.stars[0].sky.ra_hours * 15.0, xra1 = g.stars[1].sky.ra_hours * 15.0;
        double xd0 = g.stars[0].sky.dec_degrees, xd1 = g.stars[1].sky.dec_degrees;
        if (fabs(xra1 - xra0) < 1e-9 || fabs(xd1 - xd0) < 1e-9) return OS_ERR_INVALID_STATE;
        g.st.alignment.m00 = (g.stars[1].motor_steps[0] - g.stars[0].motor_steps[0]) / (xra1 - xra0);
        g.st.alignment.m01 = 0.0;
        g.st.alignment.m10 = 0.0;
        g.st.alignment.m11 = (g.stars[1].motor_steps[1] - g.stars[0].motor_steps[1]) / (xd1 - xd0);
        g.st.alignment.b0 = g.stars[0].motor_steps[0] - g.st.alignment.m00 * xra0;
        g.st.alignment.b1 = g.stars[0].motor_steps[1] - g.st.alignment.m11 * xd0;
    } else {
        for (i = 0; i < g.st.align_star_count; ++i) {
            double x[3] = {g.stars[i].sky.ra_hours * 15.0 * 3600.0, g.stars[i].sky.dec_degrees * 3600.0, 1.0};
            int r, c;
            for (r = 0; r < 3; ++r) {
                b0[r] += x[r] * (double)g.stars[i].motor_steps[0];
                b1[r] += x[r] * (double)g.stars[i].motor_steps[1];
                for (c = 0; c < 3; ++c) {
                    n00[r][c] += x[r] * x[c];
                    n11[r][c] = n00[r][c];
                }
            }
        }
        if (g.st.align_star_count == 3 && fabs(det3(n00)) < 1e-6) return OS_ERR_INVALID_STATE;
        if (!solve3(n00, b0, x0) || !solve3(n11, b1, x1)) return OS_ERR_INVALID_STATE;
        g.st.alignment.m00 = x0[0] * 3600.0;
        g.st.alignment.m01 = x0[1] * 3600.0;
        g.st.alignment.b0 = x0[2];
        g.st.alignment.m10 = x1[0] * 3600.0;
        g.st.alignment.m11 = x1[1] * 3600.0;
        g.st.alignment.b1 = x1[2];
    }

    g.st.alignment.valid = true;
    g.st.alignment.residual_arcsec = 0.0;
    if (g.st.align_star_count >= 4) {
        double sum = 0.0;
        for (i = 0; i < g.st.align_star_count; ++i) {
            double p0 = coord_to_axis0_steps(g.stars[i].sky.ra_hours);
            double p1 = coord_to_axis1_steps(g.stars[i].sky.dec_degrees);
            double e0 = p0 - g.stars[i].motor_steps[0];
            double e1 = p1 - g.stars[i].motor_steps[1];
            sum += e0 * e0 + e1 * e1;
        }
        g.st.alignment.residual_arcsec = sqrt(sum / (double)g.st.align_star_count);
    }
    g.st.calibration_residual_computed = true;
    (void)os_hal_nvm_write(OS_NVM_CALIBRATION_OFFSET, &g.st.alignment, sizeof(g.st.alignment));
    g.st.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    uint8_t axis;
    if (!g.st.initialized) return OS_ERR_INVALID_STATE;
    for (axis = 0; axis < OS_AXIS_COUNT; ++axis) {
        if (os_hal_limit_is_triggered(axis)) return OS_ERR_LIMIT_TRIGGERED;
    }
    g.st.target_steps[0] = 0;
    g.st.target_steps[1] = 0;
    g.st.goto_active = true;
    g.st.state = OS_STATE_GOTO;
    for (axis = 0; axis < OS_AXIS_COUNT; ++axis) (void)start_axis_toward(axis, 0, g.cfg.max_goto_rate_hz[axis] > 0.0 ? g.cfg.max_goto_rate_hz[axis] : 200.0);
    if (os_hal_motor_get_position(0) == 0 && os_hal_motor_get_position(1) == 0) {
        stop_all();
        g.st.parked = true;
        g.st.state = OS_STATE_PARKED;
    }
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    uint8_t ch;
    if (!g.st.initialized || !g.st.parked) return OS_ERR_INVALID_STATE;
    for (ch = 0; ch < OS_CHANNEL_COUNT; ++ch) {
        if (g.cfg.channel_enabled[ch]) (void)os_hal_comm_init(ch);
    }
    (void)os_hal_rtc_read(&g.st.site.utc_epoch_seconds);
    g.st.parked = false;
    g.st.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_manual_move(os_direction_t direction, os_motion_rate_t rate, double custom_hz) {
    int axis, sign;
    double hz;
    if (direction < OS_DIR_EAST || direction > OS_DIR_SOUTH || rate < OS_RATE_SLOW || rate > OS_RATE_CUSTOM || (rate == OS_RATE_CUSTOM && custom_hz <= 0.0)) return OS_ERR_INVALID_ARGUMENT;
    if (!g.st.initialized || g.st.state == OS_STATE_PARKED || g.st.state == OS_STATE_FAULT) return OS_ERR_INVALID_STATE;
    axis = (direction == OS_DIR_EAST || direction == OS_DIR_WEST) ? OS_AXIS_RA : OS_AXIS_DEC;
    sign = (direction == OS_DIR_EAST || direction == OS_DIR_NORTH) ? 1 : -1;
    if (limit_blocks_axis((uint8_t)axis, sign)) return OS_ERR_LIMIT_TRIGGERED;
    hz = rate == OS_RATE_SLOW ? 20.0 : (rate == OS_RATE_MEDIUM ? 100.0 : (rate == OS_RATE_FAST ? 400.0 : custom_hz));
    (void)os_hal_motor_set_direction((uint8_t)axis, sign > 0);
    (void)os_hal_motor_enable((uint8_t)axis, true);
    (void)os_hal_motor_set_frequency((uint8_t)axis, hz);
    g.manual_axis = axis;
    g.manual_dir = sign;
    g.st.manual_active = true;
    g.st.state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_manual_stop(void) {
    if (!g.st.initialized) return OS_ERR_INVALID_STATE;
    if (g.manual_axis >= 0 && g.manual_axis < OS_AXIS_COUNT) (void)stop_axis((uint8_t)g.manual_axis);
    g.st.manual_active = false;
    g.st.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_pec_set_point(double worm_phase_deg, double correction_arcsec) {
    int idx;
    if (worm_phase_deg < 0.0 || worm_phase_deg > 360.0 || !isfinite(correction_arcsec)) return OS_ERR_INVALID_ARGUMENT;
    idx = (int)llround((worm_phase_deg / 360.0) * OS_PEC_TABLE_SIZE);
    if (idx < 0) idx = 0;
    if (idx > OS_PEC_TABLE_SIZE) idx = OS_PEC_TABLE_SIZE;
    g.pec[idx] = correction_arcsec;
    g.pec_valid[idx] = true;
    (void)os_hal_nvm_write(OS_NVM_CONFIG_OFFSET, g.pec, sizeof(g.pec));
    return OS_ERR_NONE;
}

os_error_t os_pec_get_correction(double worm_phase_deg, double *correction_arcsec) {
    double pos, frac;
    int lo, hi;
    if (correction_arcsec == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (worm_phase_deg < 0.0 || worm_phase_deg > 360.0) return OS_ERR_INVALID_ARGUMENT;
    pos = (worm_phase_deg / 360.0) * OS_PEC_TABLE_SIZE;
    lo = (int)floor(pos);
    hi = lo + 1;
    if (hi > OS_PEC_TABLE_SIZE) hi = OS_PEC_TABLE_SIZE;
    frac = pos - (double)lo;
    if (!g.pec_valid[lo] || !g.pec_valid[hi]) {
        *correction_arcsec = 0.0;
        return OS_ERR_NONE;
    }
    *correction_arcsec = g.pec[lo] * (1.0 - frac) + g.pec[hi] * frac;
    return OS_ERR_NONE;
}

os_error_t os_get_status(os_status_t *status) {
    if (status == NULL) return OS_ERR_INVALID_ARGUMENT;
    update_positions();
    *status = g.st;
    return OS_ERR_NONE;
}

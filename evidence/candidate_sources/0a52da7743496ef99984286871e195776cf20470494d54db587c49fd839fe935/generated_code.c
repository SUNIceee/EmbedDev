#include "generated_code.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#if defined(__GNUC__) || defined(__clang__)
#define OS_WEAK __attribute__((weak))
#else
#define OS_WEAK
#endif

typedef struct {
    double start[OS_AXIS_COUNT];
    double target[OS_AXIS_COUNT];
    double position[OS_AXIS_COUNT];
    double frequency[OS_AXIS_COUNT];
    bool forward[OS_AXIS_COUNT];
    bool active;
    bool park_after_arrival;
} os_motion_t;

typedef struct {
    os_config_t config;
    os_status_t status;
    os_motion_t motion;
    os_alignment_mode_t align_mode;
    os_alignment_star_t stars[OS_MAX_ALIGNMENT_STARS];
    os_alignment_solution_t alignment;
    os_tracking_rate_t tracking_rate;
    double tracking_factor;
    uint32_t guide_remaining_ms[4];
    double pec_phase[OS_PEC_TABLE_SIZE];
    double pec_correction[OS_PEC_TABLE_SIZE];
    bool pec_valid[OS_PEC_TABLE_SIZE];
    char command_buffer[OS_CHANNEL_COUNT][OS_MAX_COMMAND_LENGTH];
    size_t command_length[OS_CHANNEL_COUNT];
} os_context_t;

static os_context_t g_os;

static bool os_axis_valid(int axis) {
    return axis == OS_AXIS_RA || axis == OS_AXIS_DEC;
}

static bool os_channel_valid(int channel) {
    return channel >= 0 && channel < OS_CHANNEL_COUNT;
}

static bool os_ra_valid(double ra_hours) {
    return ra_hours >= 0.0 && ra_hours <= 24.0;
}

static bool os_dec_valid(double dec_degrees) {
    return dec_degrees >= -90.0 && dec_degrees <= 90.0;
}

static void os_default_config(os_config_t *config) {
    size_t i;

    memset(config, 0, sizeof(*config));
    config->motor_step_angle_degrees = 1.8;
    config->gear_ratio_ra = 144.0;
    config->gear_ratio_dec = 144.0;
    config->steps_per_degree_ra = OS_DEFAULT_STEPS_PER_DEGREE;
    config->steps_per_degree_dec = OS_DEFAULT_STEPS_PER_DEGREE;
    config->microsteps = 16;
    config->max_goto_rate_hz = OS_DEFAULT_GOTO_RATE_HZ;
    config->tracking_rate_hz = OS_DEFAULT_TRACKING_RATE_HZ;
    config->guide_rate_factor = OS_DEFAULT_GUIDE_RATE_FACTOR;
    config->mount_mode = OS_MOUNT_EQUATORIAL;
    config->gps_enabled = true;
    config->rtc_enabled = true;
    for (i = 0; i < OS_CHANNEL_COUNT; ++i) {
        config->channel_enabled[i] = true;
    }
    config->park_position_steps[OS_AXIS_RA] = 0;
    config->park_position_steps[OS_AXIS_DEC] = 0;
    config->preset_site.latitude_degrees = 0.0;
    config->preset_site.longitude_degrees = 0.0;
    config->preset_site.elevation_metres = 0.0;
    config->preset_site.utc_epoch_seconds = 0u;
    config->preset_site.valid = true;
}

static void os_stop_axis(int axis) {
    if (!os_axis_valid(axis)) {
        return;
    }
    (void)os_hal_motor_set_frequency(axis, 0.0);
    (void)os_hal_motor_enable(axis, false);
    g_os.motion.frequency[axis] = 0.0;
}

static void os_stop_all_motion(void) {
    os_stop_axis(OS_AXIS_RA);
    os_stop_axis(OS_AXIS_DEC);
    g_os.motion.active = false;
    g_os.motion.park_after_arrival = false;
    g_os.status.is_moving = false;
    g_os.status.goto_active = false;
    g_os.status.manual_motion_active = false;
}

static void os_enter_fault(os_error_t error) {
    os_stop_all_motion();
    g_os.status.state = OS_STATE_FAULT;
    g_os.status.fault_active = true;
    g_os.status.last_error = error;
}

static double os_tracking_factor_for_rate(os_tracking_rate_t rate, double custom_factor) {
    if (rate == OS_TRACK_LUNAR) {
        return 0.966;
    }
    if (rate == OS_TRACK_SOLAR) {
        return 0.997269566;
    }
    if (rate == OS_TRACK_CUSTOM) {
        return custom_factor;
    }
    return 1.0;
}

static void os_apply_idle_tracking(void) {
    double freq;

    if (g_os.status.state != OS_STATE_IDLE_TRACKING) {
        return;
    }
    if (os_hal_limit_is_triggered(OS_AXIS_RA)) {
        os_enter_fault(OS_ERR_LIMIT_TRIGGERED);
        return;
    }

    freq = g_os.config.tracking_rate_hz * g_os.tracking_factor;
    if (freq < 0.0) {
        freq = -freq;
        (void)os_hal_motor_set_direction(OS_AXIS_RA, false);
    } else {
        (void)os_hal_motor_set_direction(OS_AXIS_RA, true);
    }
    (void)os_hal_motor_enable(OS_AXIS_RA, true);
    (void)os_hal_motor_set_frequency(OS_AXIS_RA, freq);
}

static void os_equatorial_to_steps(double ra_hours,
                                   double dec_degrees,
                                   double *axis0,
                                   double *axis1) {
    double x;
    double y;

    x = ra_hours * 15.0 * g_os.config.steps_per_degree_ra;
    y = dec_degrees * g_os.config.steps_per_degree_dec;

    if (g_os.alignment.valid) {
        double nx = g_os.alignment.matrix[0][0] * x +
                    g_os.alignment.matrix[0][1] * y +
                    g_os.alignment.offset[0];
        double ny = g_os.alignment.matrix[1][0] * x +
                    g_os.alignment.matrix[1][1] * y +
                    g_os.alignment.offset[1];
        x = nx;
        y = ny;
    }

    *axis0 = x;
    *axis1 = y;
}

static void os_steps_to_equatorial(double axis0,
                                   double axis1,
                                   double *ra_hours,
                                   double *dec_degrees) {
    *ra_hours = axis0 / g_os.config.steps_per_degree_ra / 15.0;
    *dec_degrees = axis1 / g_os.config.steps_per_degree_dec;
    while (*ra_hours < 0.0) {
        *ra_hours += 24.0;
    }
    while (*ra_hours > 24.0) {
        *ra_hours -= 24.0;
    }
}

static os_error_t os_start_motion_to(double axis0, double axis1, bool park_after) {
    int axis;

    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_os.motion.target[OS_AXIS_RA] = axis0;
    g_os.motion.target[OS_AXIS_DEC] = axis1;
    g_os.motion.park_after_arrival = park_after;
    g_os.motion.active = true;

    for (axis = 0; axis < OS_AXIS_COUNT; ++axis) {
        double delta = g_os.motion.target[axis] - g_os.motion.position[axis];
        bool forward = delta >= 0.0;
        double freq = g_os.config.max_goto_rate_hz;

        if (fabs(delta) <= OS_GOTO_DONE_TOLERANCE_STEPS) {
            (void)os_hal_motor_set_frequency(axis, 0.0);
            g_os.motion.frequency[axis] = 0.0;
            continue;
        }

        g_os.motion.forward[axis] = forward;
        g_os.motion.frequency[axis] = freq;
        (void)os_hal_motor_set_direction(axis, forward);
        (void)os_hal_motor_enable(axis, true);
        (void)os_hal_motor_set_frequency(axis, freq);
    }

    g_os.status.target_steps[OS_AXIS_RA] = (int32_t)lrint(axis0);
    g_os.status.target_steps[OS_AXIS_DEC] = (int32_t)lrint(axis1);
    g_os.status.is_moving = true;
    g_os.status.goto_active = !park_after;
    g_os.status.manual_motion_active = false;
    g_os.status.state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

static void os_update_motion(void) {
    bool any_active = false;
    int axis;

    if (!g_os.motion.active) {
        return;
    }

    for (axis = 0; axis < OS_AXIS_COUNT; ++axis) {
        double remaining;
        double step;

        if (os_hal_limit_is_triggered(axis)) {
            os_enter_fault(OS_ERR_LIMIT_TRIGGERED);
            return;
        }

        remaining = g_os.motion.target[axis] - g_os.motion.position[axis];
        if (fabs(remaining) <= OS_GOTO_DONE_TOLERANCE_STEPS) {
            g_os.motion.position[axis] = g_os.motion.target[axis];
            g_os.status.position_steps[axis] = (int32_t)lrint(g_os.motion.position[axis]);
            (void)os_hal_motor_set_frequency(axis, 0.0);
            g_os.motion.frequency[axis] = 0.0;
            continue;
        }

        any_active = true;
        step = g_os.motion.frequency[axis] * OS_LOOP_DT_SECONDS;
        if (step < 1.0) {
            step = 1.0;
        }
        if (fabs(remaining) < step) {
            step = fabs(remaining);
        }
        g_os.motion.position[axis] += (remaining >= 0.0) ? step : -step;
        g_os.status.position_steps[axis] = (int32_t)lrint(g_os.motion.position[axis]);
    }

    os_steps_to_equatorial(g_os.motion.position[OS_AXIS_RA],
                           g_os.motion.position[OS_AXIS_DEC],
                           &g_os.status.current_ra_hours,
                           &g_os.status.current_dec_degrees);

    if (!any_active) {
        os_stop_all_motion();
        if (g_os.motion.park_after_arrival) {
            g_os.status.state = OS_STATE_PARKED;
            g_os.status.parked = true;
        } else {
            g_os.status.state = OS_STATE_IDLE_TRACKING;
            (void)os_hal_buzzer_beep(100u, 1u);
            os_apply_idle_tracking();
        }
    }
}

static void os_update_guiding(void) {
    int i;

    for (i = 0; i < 4; ++i) {
        if (g_os.guide_remaining_ms[i] > 0u) {
            if (g_os.guide_remaining_ms[i] > (uint32_t)(OS_LOOP_DT_SECONDS * 1000.0)) {
                g_os.guide_remaining_ms[i] -= (uint32_t)(OS_LOOP_DT_SECONDS * 1000.0);
            } else {
                g_os.guide_remaining_ms[i] = 0u;
            }
        }
    }

    g_os.status.guide_active = g_os.guide_remaining_ms[0] || g_os.guide_remaining_ms[1] ||
                               g_os.guide_remaining_ms[2] || g_os.guide_remaining_ms[3];
}

static void os_write_reply(int channel, const char *reply) {
    if (reply != NULL && os_channel_valid(channel)) {
        (void)os_hal_comm_write(channel, (const uint8_t *)reply, strlen(reply));
    }
}

static int os_parse_ra(const char *text, double *ra_hours) {
    int h;
    int m;
    int s;

    if (sscanf(text, ":Sr%d:%d:%d#", &h, &m, &s) != 3) {
        return 0;
    }
    *ra_hours = (double)h + (double)m / 60.0 + (double)s / 3600.0;
    return 1;
}

static int os_parse_dec(const char *text, double *dec_degrees) {
    char sign;
    int d;
    int m;
    int s;
    double value;

    if (sscanf(text, ":Sd%c%d%*[:*]%d:%d#", &sign, &d, &m, &s) != 4) {
        return 0;
    }
    value = (double)d + (double)m / 60.0 + (double)s / 3600.0;
    *dec_degrees = (sign == '-') ? -value : value;
    return 1;
}

static void os_format_ra(double ra_hours, char *reply, size_t reply_size) {
    int h;
    int m;
    int s;
    double total;

    if (ra_hours < 0.0) {
        ra_hours = 0.0;
    }
    if (ra_hours > 24.0) {
        ra_hours = 24.0;
    }

    total = ra_hours * 3600.0;
    h = (int)(total / 3600.0);
    total -= (double)h * 3600.0;
    m = (int)(total / 60.0);
    s = (int)lrint(total - (double)m * 60.0);
    if (s >= 60) {
        s = 0;
        ++m;
    }
    if (m >= 60) {
        m = 0;
        ++h;
    }
    if (h >= 24) {
        h = 0;
    }
    (void)snprintf(reply, reply_size, "%02d:%02d:%02d#", h, m, s);
}

static void os_format_dec(double dec_degrees, char *reply, size_t reply_size) {
    char sign = '+';
    int d;
    int m;
    int s;
    double total;

    if (dec_degrees < 0.0) {
        sign = '-';
        dec_degrees = -dec_degrees;
    }

    total = dec_degrees * 3600.0;
    d = (int)(total / 3600.0);
    total -= (double)d * 3600.0;
    m = (int)(total / 60.0);
    s = (int)lrint(total - (double)m * 60.0);
    if (s >= 60) {
        s = 0;
        ++m;
    }
    if (m >= 60) {
        m = 0;
        ++d;
    }
    (void)snprintf(reply, reply_size, "%c%02d*%02d:%02d#", sign, d, m, s);
}

static void os_poll_commands(void) {
    int channel;

    for (channel = 0; channel < OS_CHANNEL_COUNT; ++channel) {
        int available;

        if (!g_os.config.channel_enabled[channel]) {
            continue;
        }

        available = os_hal_comm_available(channel);
        while (available > 0) {
            int byte_value = os_hal_comm_read(channel);
            char c = (char)byte_value;
            char reply[OS_MAX_REPLY_LENGTH];

            if (byte_value < 0) {
                break;
            }

            if (c == ':') {
                g_os.command_length[channel] = 0u;
            }

            if (g_os.command_length[channel] + 1u < OS_MAX_COMMAND_LENGTH) {
                g_os.command_buffer[channel][g_os.command_length[channel]++] = c;
                g_os.command_buffer[channel][g_os.command_length[channel]] = '\0';
            } else {
                g_os.command_length[channel] = 0u;
                os_write_reply(channel, "0#");
            }

            if (c == '#') {
                os_error_t err = os_process_command(channel,
                                                    g_os.command_buffer[channel],
                                                    reply,
                                                    sizeof(reply));
                if (err != OS_ERR_NONE && reply[0] == '\0') {
                    (void)snprintf(reply, sizeof(reply), "0#");
                }
                os_write_reply(channel, reply);
                g_os.command_length[channel] = 0u;
            }

            available = os_hal_comm_available(channel);
        }
    }
}

static int os_solve_linear(double a[6][7], int n, double x[6]) {
    int i;
    int j;
    int k;

    for (i = 0; i < n; ++i) {
        int pivot = i;
        double max_abs = fabs(a[i][i]);

        for (j = i + 1; j < n; ++j) {
            double v = fabs(a[j][i]);
            if (v > max_abs) {
                max_abs = v;
                pivot = j;
            }
        }

        if (max_abs < OS_ALIGNMENT_DEGENERATE_EPS) {
            return 0;
        }

        if (pivot != i) {
            for (k = i; k <= n; ++k) {
                double tmp = a[i][k];
                a[i][k] = a[pivot][k];
                a[pivot][k] = tmp;
            }
        }

        for (j = i + 1; j < n; ++j) {
            double factor = a[j][i] / a[i][i];
            for (k = i; k <= n; ++k) {
                a[j][k] -= factor * a[i][k];
            }
        }
    }

    for (i = n - 1; i >= 0; --i) {
        double sum = a[i][n];
        for (j = i + 1; j < n; ++j) {
            sum -= a[i][j] * x[j];
        }
        if (fabs(a[i][i]) < OS_ALIGNMENT_DEGENERATE_EPS) {
            return 0;
        }
        x[i] = sum / a[i][i];
    }

    return 1;
}

static os_error_t os_compute_affine(int parameters, os_alignment_solution_t *solution) {
    double normal[6][7];
    double x[6];
    uint8_t i;
    int p;
    int q;

    memset(normal, 0, sizeof(normal));
    memset(x, 0, sizeof(x));

    for (i = 0; i < g_os.status.alignment_star_count; ++i) {
        const os_alignment_star_t *star = &g_os.stars[i];
        double sx = star->ra_hours * 15.0 * 3600.0;
        double sy = star->dec_degrees * 3600.0;
        double row0[6] = {1.0, sx, sy, 0.0, 0.0, 0.0};
        double row1[6] = {0.0, 0.0, 0.0, 1.0, sx, sy};
        double b0 = (double)star->axis_steps[OS_AXIS_RA];
        double b1 = (double)star->axis_steps[OS_AXIS_DEC];

        if (parameters == 2) {
            row0[2] = 0.0;
            row1[4] = 0.0;
        } else if (parameters == 4) {
            row0[2] = 0.0;
            row1[4] = 0.0;
        }

        for (p = 0; p < parameters; ++p) {
            for (q = 0; q < parameters; ++q) {
                normal[p][q] += row0[p] * row0[q] + row1[p] * row1[q];
            }
            normal[p][parameters] += row0[p] * b0 + row1[p] * b1;
        }
    }

    if (!os_solve_linear(normal, parameters, x)) {
        return OS_ERR_DEGENERATE_ALIGNMENT;
    }

    if (parameters == 2) {
        solution->offset[0] = x[0];
        solution->offset[1] = x[1];
        solution->matrix[0][0] = g_os.config.steps_per_degree_ra / 3600.0;
        solution->matrix[0][1] = 0.0;
        solution->matrix[1][0] = 0.0;
        solution->matrix[1][1] = g_os.config.steps_per_degree_dec / 3600.0;
    } else if (parameters == 4) {
        solution->offset[0] = x[0];
        solution->matrix[0][0] = x[1];
        solution->matrix[0][1] = 0.0;
        solution->offset[1] = x[3];
        solution->matrix[1][0] = 0.0;
        solution->matrix[1][1] = x[5];
    } else {
        solution->offset[0] = x[0];
        solution->matrix[0][0] = x[1];
        solution->matrix[0][1] = x[2];
        solution->offset[1] = x[3];
        solution->matrix[1][0] = x[4];
        solution->matrix[1][1] = x[5];
    }

    solution->residual_arcsec = 0.0;
    if (g_os.status.alignment_star_count >= 4u) {
        double sum_sq = 0.0;
        for (i = 0; i < g_os.status.alignment_star_count; ++i) {
            const os_alignment_star_t *star = &g_os.stars[i];
            double sx = star->ra_hours * 15.0 * 3600.0;
            double sy = star->dec_degrees * 3600.0;
            double px = solution->offset[0] + solution->matrix[0][0] * sx + solution->matrix[0][1] * sy;
            double py = solution->offset[1] + solution->matrix[1][0] * sx + solution->matrix[1][1] * sy;
            double dx = px - (double)star->axis_steps[OS_AXIS_RA];
            double dy = py - (double)star->axis_steps[OS_AXIS_DEC];
            sum_sq += dx * dx + dy * dy;
        }
        solution->residual_arcsec = sqrt(sum_sq / (double)g_os.status.alignment_star_count);
    }

    solution->valid = true;
    return OS_ERR_NONE;
}

os_error_t os_init(void) {
    int axis;
    int channel;
    os_site_info_t site;
    uint32_t rtc_time = 0u;

    memset(&g_os, 0, sizeof(g_os));
    os_default_config(&g_os.config);

    g_os.status.state = OS_STATE_INITIALIZING;
    g_os.status.initialized = false;
    g_os.status.alignment_residual_valid = false;
    g_os.status.alignment_star_count = 0u;
    g_os.status.guide_active = false;
    g_os.status.manual_motion_active = false;
    g_os.status.goto_active = false;
    g_os.tracking_rate = OS_TRACK_SIDEREAL;
    g_os.tracking_factor = 1.0;

    (void)os_hal_nvm_init();

    for (channel = 0; channel < OS_CHANNEL_COUNT; ++channel) {
        if (g_os.config.channel_enabled[channel]) {
            (void)os_hal_comm_init(channel);
        }
    }

    for (axis = 0; axis < OS_AXIS_COUNT; ++axis) {
        os_error_t err = os_hal_motor_init(axis);
        if (err != OS_ERR_NONE) {
            os_enter_fault(err);
            return err;
        }
        (void)os_hal_motor_set_frequency(axis, 0.0);
        (void)os_hal_motor_enable(axis, false);
        g_os.motion.position[axis] = (double)os_hal_motor_get_position(axis);
        g_os.status.position_steps[axis] = (int32_t)lrint(g_os.motion.position[axis]);
    }

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();

    if (os_hal_timer_motor_init() != OS_ERR_NONE) {
        os_enter_fault(OS_ERR_DEVICE);
        return OS_ERR_DEVICE;
    }

    memset(&site, 0, sizeof(site));
    if (g_os.config.gps_enabled && os_hal_gps_poll(&site) == OS_ERR_NONE && site.valid &&
        site.latitude_degrees >= -90.0 && site.latitude_degrees <= 90.0 &&
        site.longitude_degrees >= -180.0 && site.longitude_degrees <= 180.0) {
        g_os.config.preset_site = site;
        g_os.status.gps_locked = true;
        (void)os_hal_rtc_set(site.utc_epoch_seconds);
    } else {
        g_os.status.gps_locked = false;
        if (g_os.config.rtc_enabled && os_hal_rtc_read(&rtc_time) == OS_ERR_NONE) {
            g_os.config.preset_site.utc_epoch_seconds = rtc_time;
            g_os.status.rtc_valid = true;
        }
    }

    g_os.status.initialized = true;
    g_os.status.state = OS_STATE_IDLE_TRACKING;
    g_os.status.last_error = OS_ERR_NONE;
    os_apply_idle_tracking();
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    if (!g_os.status.initialized) {
        return;
    }

    os_poll_commands();

    if (g_os.status.state == OS_STATE_FAULT || g_os.status.state == OS_STATE_PARKED) {
        return;
    }

    os_update_guiding();

    if (g_os.motion.active) {
        os_update_motion();
    } else {
        os_apply_idle_tracking();
    }
}

os_error_t os_get_status(os_status_t *status) {
    if (status == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *status = g_os.status;
    return OS_ERR_NONE;
}

os_state_t os_get_state(void) {
    return g_os.status.state;
}

bool os_query_is_moving(void) {
    return g_os.status.is_moving;
}

os_error_t os_get_current_equatorial(os_equatorial_coord_t *coord) {
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    coord->ra_hours = g_os.status.current_ra_hours;
    coord->dec_degrees = g_os.status.current_dec_degrees;
    return OS_ERR_NONE;
}

os_error_t os_set_config(const os_config_t *config) {
    if (config == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (config->steps_per_degree_ra <= 0.0 || config->steps_per_degree_dec <= 0.0 ||
        config->max_goto_rate_hz < 0.0 || config->tracking_rate_hz < 0.0 ||
        config->guide_rate_factor < 0.1 || config->guide_rate_factor > 1.0 ||
        (config->mount_mode != OS_MOUNT_EQUATORIAL && config->mount_mode != OS_MOUNT_ALTAZ)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_os.config = *config;
    return OS_ERR_NONE;
}

os_error_t os_get_config(os_config_t *config) {
    if (config == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *config = g_os.config;
    return OS_ERR_NONE;
}

os_error_t os_goto_equatorial(double ra_hours, double dec_degrees) {
    double axis0;
    double axis1;

    if (!os_ra_valid(ra_hours) || !os_dec_valid(dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_os.status.state == OS_STATE_PARKED || g_os.status.state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    os_equatorial_to_steps(ra_hours, dec_degrees, &axis0, &axis1);
    g_os.status.current_ra_hours = ra_hours;
    g_os.status.current_dec_degrees = dec_degrees;

    if (fabs(axis0 - g_os.motion.position[OS_AXIS_RA]) <= OS_GOTO_DONE_TOLERANCE_STEPS &&
        fabs(axis1 - g_os.motion.position[OS_AXIS_DEC]) <= OS_GOTO_DONE_TOLERANCE_STEPS) {
        g_os.status.state = OS_STATE_IDLE_TRACKING;
        g_os.status.is_moving = false;
        return OS_ERR_NONE;
    }

    return os_start_motion_to(axis0, axis1, false);
}

os_error_t os_goto_abort(void) {
    if (g_os.status.state != OS_STATE_GOTO && !g_os.motion.active) {
        return OS_ERR_INVALID_STATE;
    }
    os_stop_all_motion();
    g_os.status.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_set_tracking_rate(os_tracking_rate_t rate, double custom_factor) {
    if (rate < OS_TRACK_SIDEREAL || rate > OS_TRACK_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_CUSTOM && custom_factor <= 0.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_os.tracking_rate = rate;
    g_os.tracking_factor = os_tracking_factor_for_rate(rate, custom_factor);
    os_apply_idle_tracking();
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_guide_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_GUIDE_EAST || direction > OS_GUIDE_SOUTH || duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_os.status.state == OS_STATE_PARKED || g_os.status.state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    g_os.guide_remaining_ms[(int)direction] = duration_ms;
    g_os.status.guide_active = true;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_alignment_mode_t mode) {
    if (mode != OS_ALIGN_ONE_STAR && mode != OS_ALIGN_TWO_STAR &&
        mode != OS_ALIGN_THREE_STAR && mode != OS_ALIGN_MULTI_STAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_os.align_mode = mode;
    g_os.status.alignment_star_count = 0u;
    g_os.status.alignment_residual_valid = false;
    g_os.alignment.valid = false;
    g_os.status.state = OS_STATE_ALIGNING;
    return OS_ERR_NONE;
}

os_error_t os_align_add_star(double ra_hours,
                             double dec_degrees,
                             int32_t axis0_steps,
                             int32_t axis1_steps) {
    os_alignment_star_t *star;

    if (!os_ra_valid(ra_hours) || !os_dec_valid(dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_os.status.alignment_star_count >= OS_MAX_ALIGNMENT_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    star = &g_os.stars[g_os.status.alignment_star_count++];
    star->ra_hours = ra_hours;
    star->dec_degrees = dec_degrees;
    star->axis_steps[OS_AXIS_RA] = axis0_steps;
    star->axis_steps[OS_AXIS_DEC] = axis1_steps;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(os_alignment_solution_t *solution) {
    os_error_t err;
    uint8_t needed;
    int parameters;

    if (solution == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    needed = 3u;
    parameters = 6;
    if (g_os.align_mode == OS_ALIGN_ONE_STAR) {
        needed = 1u;
        parameters = 2;
    } else if (g_os.align_mode == OS_ALIGN_TWO_STAR) {
        needed = 2u;
        parameters = 4;
    } else if (g_os.align_mode == OS_ALIGN_THREE_STAR || g_os.align_mode == OS_ALIGN_MULTI_STAR) {
        needed = 3u;
        parameters = 6;
    }

    if (g_os.status.alignment_star_count < needed) {
        return OS_ERR_INVALID_STATE;
    }

    memset(solution, 0, sizeof(*solution));
    err = os_compute_affine(parameters, solution);
    if (err != OS_ERR_NONE) {
        return err;
    }

    if (g_os.status.alignment_star_count >= 4u &&
        solution->residual_arcsec > OS_ALIGNMENT_RESIDUAL_LIMIT_ARCSEC) {
        return OS_ERR_DEVICE;
    }

    g_os.alignment = *solution;
    g_os.status.alignment_residual_arcsec = solution->residual_arcsec;
    g_os.status.alignment_residual_valid = true;
    g_os.status.state = OS_STATE_IDLE_TRACKING;
    (void)os_hal_nvm_write(OS_NVM_CALIBRATION_OFFSET, solution, sizeof(*solution));
    return OS_ERR_NONE;
}

os_error_t os_align_clear(void) {
    memset(g_os.stars, 0, sizeof(g_os.stars));
    memset(&g_os.alignment, 0, sizeof(g_os.alignment));
    g_os.status.alignment_star_count = 0u;
    g_os.status.alignment_residual_valid = false;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (g_os.status.state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    return os_start_motion_to((double)g_os.config.park_position_steps[OS_AXIS_RA],
                              (double)g_os.config.park_position_steps[OS_AXIS_DEC],
                              true);
}

os_error_t os_unpark(void) {
    int axis;

    if (g_os.status.state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    for (axis = 0; axis < OS_AXIS_COUNT; ++axis) {
        (void)os_hal_motor_enable(axis, true);
    }
    for (axis = 0; axis < OS_CHANNEL_COUNT; ++axis) {
        if (g_os.config.channel_enabled[axis]) {
            (void)os_hal_comm_init(axis);
        }
    }
    if (g_os.config.rtc_enabled) {
        uint32_t utc = 0u;
        (void)os_hal_rtc_read(&utc);
    }

    g_os.status.parked = false;
    g_os.status.state = OS_STATE_IDLE_TRACKING;
    os_apply_idle_tracking();
    return OS_ERR_NONE;
}

os_error_t os_manual_move(os_move_direction_t direction,
                          os_manual_rate_t rate,
                          double custom_frequency_hz) {
    int axis;
    bool forward;
    double frequency;

    if (direction < OS_MOVE_EAST || direction > OS_MOVE_SOUTH ||
        rate < OS_RATE_SLOW || rate > OS_RATE_CUSTOM ||
        (rate == OS_RATE_CUSTOM && custom_frequency_hz <= 0.0)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_os.status.state == OS_STATE_PARKED || g_os.status.state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    axis = (direction == OS_MOVE_EAST || direction == OS_MOVE_WEST) ? OS_AXIS_RA : OS_AXIS_DEC;
    forward = (direction == OS_MOVE_EAST || direction == OS_MOVE_NORTH);

    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    frequency = 50.0;
    if (rate == OS_RATE_MEDIUM) {
        frequency = 250.0;
    } else if (rate == OS_RATE_FAST) {
        frequency = g_os.config.max_goto_rate_hz;
    } else if (rate == OS_RATE_CUSTOM) {
        frequency = custom_frequency_hz;
    }

    os_stop_all_motion();
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_frequency(axis, frequency);
    g_os.status.state = OS_STATE_MANUAL_MOTION;
    g_os.status.manual_motion_active = true;
    g_os.status.is_moving = true;
    return OS_ERR_NONE;
}

os_error_t os_manual_stop(void) {
    if (g_os.status.state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }
    os_stop_all_motion();
    g_os.status.state = OS_STATE_IDLE_TRACKING;
    os_apply_idle_tracking();
    return OS_ERR_NONE;
}

os_error_t os_pec_set_point(double worm_phase_deg, double correction_arcsec) {
    int index;

    if (worm_phase_deg < 0.0 || worm_phase_deg > 360.0 || !isfinite(correction_arcsec)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    index = (int)floor((worm_phase_deg / 360.0) * (double)OS_PEC_TABLE_SIZE);
    if (index >= OS_PEC_TABLE_SIZE) {
        index = OS_PEC_TABLE_SIZE - 1;
    }

    g_os.pec_phase[index] = worm_phase_deg;
    g_os.pec_correction[index] = correction_arcsec;
    g_os.pec_valid[index] = true;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_correction(double worm_phase_deg, double *correction_arcsec) {
    int index;

    if (correction_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (worm_phase_deg < 0.0 || worm_phase_deg > 360.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    index = (int)floor((worm_phase_deg / 360.0) * (double)OS_PEC_TABLE_SIZE);
    if (index >= OS_PEC_TABLE_SIZE) {
        index = OS_PEC_TABLE_SIZE - 1;
    }

    *correction_arcsec = g_os.pec_valid[index] ? g_os.pec_correction[index] : 0.0;
    return OS_ERR_NONE;
}

os_error_t os_process_command(int channel,
                              const char *command,
                              char *reply,
                              size_t reply_size) {
    os_error_t err;
    static double pending_ra = 0.0;
    static double pending_dec = 0.0;

    if (!os_channel_valid(channel) || command == NULL || reply == NULL || reply_size == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    reply[0] = '\0';

    if (command[0] != ':' || command[strlen(command) - 1u] != '#') {
        (void)snprintf(reply, reply_size, "0#");
        return OS_ERR_COMMAND_FORMAT;
    }

    if (strcmp(command, ":GR#") == 0) {
        os_format_ra(g_os.status.current_ra_hours, reply, reply_size);
        return OS_ERR_NONE;
    }
    if (strcmp(command, ":GD#") == 0) {
        os_format_dec(g_os.status.current_dec_degrees, reply, reply_size);
        return OS_ERR_NONE;
    }
    if (strcmp(command, ":GVP#") == 0) {
        (void)snprintf(reply, reply_size, "OnStep-Generated#");
        return OS_ERR_NONE;
    }
    if (strcmp(command, ":D#") == 0) {
        (void)snprintf(reply, reply_size, "%c#", os_query_is_moving() ? '1' : '0');
        return OS_ERR_NONE;
    }
    if (strncmp(command, ":Sr", 3u) == 0) {
        if (!os_parse_ra(command, &pending_ra) || !os_ra_valid(pending_ra)) {
            (void)snprintf(reply, reply_size, "0#");
            return OS_ERR_INVALID_ARGUMENT;
        }
        (void)snprintf(reply, reply_size, "1#");
        return OS_ERR_NONE;
    }
    if (strncmp(command, ":Sd", 3u) == 0) {
        if (!os_parse_dec(command, &pending_dec) || !os_dec_valid(pending_dec)) {
            (void)snprintf(reply, reply_size, "0#");
            return OS_ERR_INVALID_ARGUMENT;
        }
        (void)snprintf(reply, reply_size, "1#");
        return OS_ERR_NONE;
    }
    if (strcmp(command, ":MS#") == 0) {
        err = os_goto_equatorial(pending_ra, pending_dec);
        (void)snprintf(reply, reply_size, "%d#", err == OS_ERR_NONE ? 0 : 1);
        return err;
    }
    if (strcmp(command, ":Q#") == 0) {
        err = os_goto_abort();
        (void)snprintf(reply, reply_size, "1#");
        return err == OS_ERR_INVALID_STATE ? OS_ERR_NONE : err;
    }
    if (strcmp(command, ":hP#") == 0) {
        err = os_park();
        (void)snprintf(reply, reply_size, "%d#", err == OS_ERR_NONE ? 1 : 0);
        return err;
    }
    if (strcmp(command, ":hO#") == 0) {
        err = os_unpark();
        (void)snprintf(reply, reply_size, "%d#", err == OS_ERR_NONE ? 1 : 0);
        return err;
    }
    if (strcmp(command, ":Me#") == 0) {
        err = os_manual_move(OS_MOVE_EAST, OS_RATE_MEDIUM, 0.0);
        (void)snprintf(reply, reply_size, "%d#", err == OS_ERR_NONE ? 1 : 0);
        return err;
    }
    if (strcmp(command, ":Mw#") == 0) {
        err = os_manual_move(OS_MOVE_WEST, OS_RATE_MEDIUM, 0.0);
        (void)snprintf(reply, reply_size, "%d#", err == OS_ERR_NONE ? 1 : 0);
        return err;
    }
    if (strcmp(command, ":Mn#") == 0) {
        err = os_manual_move(OS_MOVE_NORTH, OS_RATE_MEDIUM, 0.0);
        (void)snprintf(reply, reply_size, "%d#", err == OS_ERR_NONE ? 1 : 0);
        return err;
    }
    if (strcmp(command, ":Ms#") == 0) {
        err = os_manual_move(OS_MOVE_SOUTH, OS_RATE_MEDIUM, 0.0);
        (void)snprintf(reply, reply_size, "%d#", err == OS_ERR_NONE ? 1 : 0);
        return err;
    }
    if (strcmp(command, ":Qe#") == 0 || strcmp(command, ":Qw#") == 0 ||
        strcmp(command, ":Qn#") == 0 || strcmp(command, ":Qs#") == 0) {
        err = os_manual_stop();
        (void)snprintf(reply, reply_size, "1#");
        return err == OS_ERR_INVALID_STATE ? OS_ERR_NONE : err;
    }

    (void)snprintf(reply, reply_size, "?#");
    return OS_ERR_COMMAND_FORMAT;
}

typedef struct {
    bool initialized[OS_AXIS_COUNT];
    bool enabled[OS_AXIS_COUNT];
    bool direction[OS_AXIS_COUNT];
    double frequency[OS_AXIS_COUNT];
    int32_t position[OS_AXIS_COUNT];
    bool limit[OS_AXIS_COUNT];
    uint8_t nvm[OS_NVM_CAPACITY_BYTES];
    uint8_t rx[OS_CHANNEL_COUNT][256];
    size_t rx_head[OS_CHANNEL_COUNT];
    size_t rx_tail[OS_CHANNEL_COUNT];
    uint8_t tx[OS_CHANNEL_COUNT][512];
    size_t tx_len[OS_CHANNEL_COUNT];
} os_host_hal_t;

static os_host_hal_t g_hal;

OS_WEAK os_error_t os_hal_motor_init(int axis) {
    if (!os_axis_valid(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_hal.initialized[axis] = true;
    g_hal.enabled[axis] = false;
    g_hal.frequency[axis] = 0.0;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_motor_set_frequency(int axis, double frequency_hz) {
    if (!os_axis_valid(axis) || frequency_hz < 0.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_hal.frequency[axis] = frequency_hz;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_motor_set_direction(int axis, bool forward) {
    if (!os_axis_valid(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_hal.direction[axis] = forward;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_motor_enable(int axis, bool enable) {
    if (!os_axis_valid(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_hal.enabled[axis] = enable;
    if (!enable) {
        g_hal.frequency[axis] = 0.0;
    }
    return OS_ERR_NONE;
}

OS_WEAK int32_t os_hal_motor_get_position(int axis) {
    if (!os_axis_valid(axis)) {
        return 0;
    }
    return g_hal.position[axis];
}

OS_WEAK os_error_t os_hal_timer_motor_init(void) {
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_gps_init(void) {
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    site->valid = false;
    return OS_ERR_TIMEOUT;
}

OS_WEAK os_error_t os_hal_rtc_init(void) {
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *utc_epoch_seconds = 0u;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    (void)utc_epoch_seconds;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_limit_init(void) {
    return OS_ERR_NONE;
}

OS_WEAK bool os_hal_limit_is_triggered(int axis) {
    if (!os_axis_valid(axis)) {
        return true;
    }
    return g_hal.limit[axis];
}

OS_WEAK os_error_t os_hal_buzzer_beep(uint32_t duration_ms, uint8_t count) {
    (void)duration_ms;
    (void)count;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_comm_init(int channel) {
    if (!os_channel_valid(channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    return OS_ERR_NONE;
}

OS_WEAK int os_hal_comm_available(int channel) {
    size_t head;
    size_t tail;

    if (!os_channel_valid(channel)) {
        return 0;
    }

    head = g_hal.rx_head[channel];
    tail = g_hal.rx_tail[channel];
    if (tail >= head) {
        return (int)(tail - head);
    }
    return (int)(sizeof(g_hal.rx[channel]) - head + tail);
}

OS_WEAK int os_hal_comm_read(int channel) {
    uint8_t value;

    if (!os_channel_valid(channel) || os_hal_comm_available(channel) <= 0) {
        return -1;
    }

    value = g_hal.rx[channel][g_hal.rx_head[channel]];
    g_hal.rx_head[channel] = (g_hal.rx_head[channel] + 1u) % sizeof(g_hal.rx[channel]);
    return (int)value;
}

OS_WEAK os_error_t os_hal_comm_write(int channel, const uint8_t *data, size_t length) {
    size_t available;

    if (!os_channel_valid(channel) || (data == NULL && length > 0u)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    available = sizeof(g_hal.tx[channel]) - g_hal.tx_len[channel];
    if (length > available) {
        length = available;
    }
    if (length > 0u) {
        memcpy(&g_hal.tx[channel][g_hal.tx_len[channel]], data, length);
        g_hal.tx_len[channel] += length;
    }
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_nvm_init(void) {
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_nvm_read(size_t offset, void *data, size_t length) {
    if (data == NULL && length > 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (offset > OS_NVM_CAPACITY_BYTES || length > OS_NVM_CAPACITY_BYTES - offset) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length > 0u) {
        memcpy(data, &g_hal.nvm[offset], length);
    }
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_nvm_write(size_t offset, const void *data, size_t length) {
    if (data == NULL && length > 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (offset > OS_NVM_CAPACITY_BYTES || length > OS_NVM_CAPACITY_BYTES - offset) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length > 0u) {
        memcpy(&g_hal.nvm[offset], data, length);
    }
    return OS_ERR_NONE;
}

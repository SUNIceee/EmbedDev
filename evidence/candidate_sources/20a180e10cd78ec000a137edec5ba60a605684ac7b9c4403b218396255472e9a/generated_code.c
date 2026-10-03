#include "generated_code.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define OS_DEFAULT_STEP_ANGLE 1.8
#define OS_DEFAULT_MICROSTEPS 16.0
#define OS_DEFAULT_GEAR_RATIO 144.0
#define OS_DEFAULT_GOTO_DEG_PER_SEC 3.0
#define OS_DEFAULT_TRACK_ARCSEC_PER_SEC 15.041067
#define OS_COMMAND_RX_SIZE 128
#define OS_COMMAND_TX_SIZE 512
#define OS_ALIGN_EPS 1.0e-9

typedef struct {
    bool active;
    int32_t target[OS_AXIS_COUNT];
    int direction[OS_AXIS_COUNT];
} os_motion_plan_t;

typedef struct {
    bool active;
    os_guide_direction_t direction;
    uint32_t remaining_ms;
} os_guide_state_t;

typedef struct {
    bool active;
    os_motion_direction_t direction;
    double frequency_hz;
} os_manual_state_t;

typedef struct {
    double phase_deg;
    double correction_arcsec;
    bool valid;
} os_pec_point_t;

typedef struct {
    os_config_t config;
    os_status_t status;
    os_alignment_star_t stars[OS_MAX_ALIGN_STARS];
    os_motion_plan_t goto_plan;
    os_motion_plan_t park_plan;
    os_manual_state_t manual;
    os_guide_state_t guide;
    os_pec_point_t pec[OS_PEC_TABLE_SIZE];
    char rx_command[OS_CHANNEL_COUNT][OS_MAX_COMMAND_LENGTH];
    size_t rx_len[OS_CHANNEL_COUNT];
    uint32_t loop_time_ms;
} os_context_t;

typedef struct {
    uint8_t rx[OS_COMMAND_RX_SIZE];
    size_t rx_head;
    size_t rx_tail;
    uint8_t tx[OS_COMMAND_TX_SIZE];
    size_t tx_len;
    os_hal_motor_observation_t motor[OS_AXIS_COUNT];
    bool comm_initialized[OS_CHANNEL_COUNT];
    bool limit[OS_AXIS_COUNT];
    uint8_t nvm[OS_NVM_SIZE_BYTES];
    os_site_info_t gps;
    bool rtc_valid;
    uint32_t rtc_epoch;
    uint32_t now_ms;
    uint32_t beep_duration_ms;
    unsigned beep_count;
} os_hal_host_t;

static os_context_t g_os;
static os_hal_host_t g_hal;

static bool os_valid_axis(int axis) { return axis == OS_AXIS_RA || axis == OS_AXIS_DEC; }
static bool os_valid_channel(int channel) { return channel >= 0 && channel < OS_CHANNEL_COUNT; }
static bool os_valid_ra_dec(double ra_hours, double dec_degrees) { return ra_hours >= 0.0 && ra_hours <= 24.0 && dec_degrees >= -90.0 && dec_degrees <= 90.0; }

static os_config_t os_default_config(void) {
    os_config_t c;
    memset(&c, 0, sizeof(c));
    c.motor_step_angle_degrees = OS_DEFAULT_STEP_ANGLE;
    c.microsteps = OS_DEFAULT_MICROSTEPS;
    c.gear_ratio[0] = OS_DEFAULT_GEAR_RATIO;
    c.gear_ratio[1] = OS_DEFAULT_GEAR_RATIO;
    c.steps_per_degree[0] = c.gear_ratio[0] * c.microsteps / c.motor_step_angle_degrees;
    c.steps_per_degree[1] = c.gear_ratio[1] * c.microsteps / c.motor_step_angle_degrees;
    c.max_goto_rate_deg_per_sec = OS_DEFAULT_GOTO_DEG_PER_SEC;
    c.tracking_rate_arcsec_per_sec = OS_DEFAULT_TRACK_ARCSEC_PER_SEC;
    c.guide_rate_multiplier = 0.5;
    c.mount_mode = OS_MOUNT_EQUATORIAL;
    c.gps_enabled = true;
    c.channel_enabled[OS_CHANNEL_USB] = true;
    c.channel_enabled[OS_CHANNEL_BLUETOOTH] = true;
    c.channel_enabled[OS_CHANNEL_WIFI] = true;
    c.channel_enabled[OS_CHANNEL_ETHERNET] = true;
    c.park_position_steps[0] = 0;
    c.park_position_steps[1] = 0;
    return c;
}

static bool os_config_valid(const os_config_t *c) {
    return c &&
           c->motor_step_angle_degrees > 0.0 &&
           c->microsteps > 0.0 &&
           c->gear_ratio[0] > 0.0 &&
           c->gear_ratio[1] > 0.0 &&
           c->steps_per_degree[0] > 0.0 &&
           c->steps_per_degree[1] > 0.0 &&
           c->max_goto_rate_deg_per_sec > 0.0 &&
           c->tracking_rate_arcsec_per_sec > 0.0 &&
           c->guide_rate_multiplier >= 0.1 &&
           c->guide_rate_multiplier <= 1.0 &&
           (c->mount_mode == OS_MOUNT_EQUATORIAL || c->mount_mode == OS_MOUNT_ALTAZ);
}

static int32_t os_target_steps_for_axis(int axis, double ra_hours, double dec_degrees) {
    double x = ra_hours * 15.0 * 3600.0;
    double y = dec_degrees * 3600.0;
    if (g_os.status.alignment.valid) {
        double v = g_os.status.alignment.matrix[axis][0] * x + g_os.status.alignment.matrix[axis][1] * y + g_os.status.alignment.offset[axis];
        return (int32_t)llround(v);
    }
    if (axis == OS_AXIS_RA) {
        return (int32_t)llround(ra_hours * 15.0 * g_os.config.steps_per_degree[axis]);
    }
    return (int32_t)llround(dec_degrees * g_os.config.steps_per_degree[axis]);
}

static void os_stop_axis(int axis) {
    if (os_valid_axis(axis)) {
        os_hal_motor_set_frequency(axis, 0.0);
        os_hal_motor_enable(axis, false);
        g_os.status.motor_frequency_hz[axis] = 0.0;
        g_os.status.motor_enabled[axis] = false;
    }
}

static void os_stop_all_motion(void) {
    os_stop_axis(OS_AXIS_RA);
    os_stop_axis(OS_AXIS_DEC);
    g_os.goto_plan.active = false;
    g_os.park_plan.active = false;
    g_os.manual.active = false;
    g_os.guide.active = false;
    g_os.status.moving = false;
    g_os.status.goto_active = false;
    g_os.status.manual_active = false;
    g_os.status.guide_active = false;
}

static double os_tracking_hz(void) {
    double arcsec = g_os.config.tracking_rate_arcsec_per_sec;
    if (g_os.status.tracking_mode == OS_TRACK_LUNAR) arcsec *= 0.966;
    if (g_os.status.tracking_mode == OS_TRACK_SOLAR) arcsec *= 0.9972696;
    if (g_os.status.tracking_mode == OS_TRACK_CUSTOM) arcsec = g_os.status.custom_tracking_rate_arcsec_per_sec;
    return (arcsec / 3600.0) * g_os.config.steps_per_degree[OS_AXIS_RA];
}

static void os_apply_tracking(void) {
    if (g_os.status.state == OS_STATE_IDLE_TRACKING && !g_os.status.fault) {
        if (!os_hal_limit_is_triggered(OS_AXIS_RA)) {
            os_hal_motor_enable(OS_AXIS_RA, true);
            os_hal_motor_set_direction(OS_AXIS_RA, true);
            os_hal_motor_set_frequency(OS_AXIS_RA, os_tracking_hz());
            g_os.status.motor_enabled[OS_AXIS_RA] = true;
            g_os.status.motor_frequency_hz[OS_AXIS_RA] = os_tracking_hz();
        }
    }
}

static os_error_t os_start_plan(os_motion_plan_t *plan, const int32_t target[OS_AXIS_COUNT], os_state_t state) {
    int i;
    if (!plan || !target) return OS_ERR_INVALID_ARGUMENT;
    for (i = 0; i < OS_AXIS_COUNT; ++i) {
        g_os.status.motor_position_steps[i] = os_hal_motor_get_position(i);
        if (os_hal_limit_is_triggered(i)) {
            g_os.status.limit_triggered[i] = true;
            g_os.status.last_error = OS_ERR_LIMIT_TRIGGERED;
            return OS_ERR_LIMIT_TRIGGERED;
        }
    }
    memset(plan, 0, sizeof(*plan));
    for (i = 0; i < OS_AXIS_COUNT; ++i) {
        plan->target[i] = target[i];
        plan->direction[i] = (target[i] >= g_os.status.motor_position_steps[i]) ? 1 : -1;
    }
    plan->active = true;
    g_os.status.state = state;
    g_os.status.moving = true;
    g_os.status.goto_active = (state == OS_STATE_GOTO);
    for (i = 0; i < OS_AXIS_COUNT; ++i) {
        if (plan->target[i] != g_os.status.motor_position_steps[i]) {
            os_hal_motor_enable(i, true);
            os_hal_motor_set_direction(i, plan->direction[i] > 0);
            os_hal_motor_set_frequency(i, g_os.config.max_goto_rate_deg_per_sec * g_os.config.steps_per_degree[i]);
            g_os.status.motor_enabled[i] = true;
            g_os.status.motor_frequency_hz[i] = g_os.config.max_goto_rate_deg_per_sec * g_os.config.steps_per_degree[i];
        }
    }
    return OS_ERR_NONE;
}

static void os_advance_plan(os_motion_plan_t *plan, bool park) {
    int done = 1;
    int i;
    if (!plan || !plan->active) return;
    for (i = 0; i < OS_AXIS_COUNT; ++i) {
        int32_t pos = os_hal_motor_get_position(i);
        g_os.status.motor_position_steps[i] = pos;
        if (os_hal_limit_is_triggered(i)) {
            g_os.status.limit_triggered[i] = true;
            os_stop_all_motion();
            g_os.status.state = OS_STATE_FAULT;
            g_os.status.fault = true;
            g_os.status.last_error = OS_ERR_LIMIT_TRIGGERED;
            return;
        }
        if ((plan->direction[i] > 0 && pos >= plan->target[i]) || (plan->direction[i] < 0 && pos <= plan->target[i]) || pos == plan->target[i]) {
            os_hal_motor_set_frequency(i, 0.0);
            g_os.status.motor_frequency_hz[i] = 0.0;
        } else {
            done = 0;
        }
    }
    if (done) {
        plan->active = false;
        g_os.status.moving = false;
        g_os.status.goto_active = false;
        os_hal_buzzer_beep(80, 1);
        if (park) {
            os_stop_all_motion();
            g_os.status.state = OS_STATE_PARKED;
            g_os.status.parked = true;
        } else {
            g_os.status.state = OS_STATE_IDLE_TRACKING;
            os_apply_tracking();
        }
    }
}

os_error_t os_init(const os_config_t *config) {
    int i;
    os_config_t defaults = os_default_config();
    os_error_t err;
    memset(&g_os, 0, sizeof(g_os));
    g_os.status.state = OS_STATE_INITIALIZING;
    g_os.config = config ? *config : defaults;
    if (!os_config_valid(&g_os.config)) return OS_ERR_INVALID_ARGUMENT;

    err = os_hal_nvm_init();
    if (err != OS_ERR_NONE) return err;
    for (i = 0; i < OS_CHANNEL_COUNT; ++i) {
        if (g_os.config.channel_enabled[i]) (void)os_hal_comm_init(i);
    }
    for (i = 0; i < OS_AXIS_COUNT; ++i) {
        err = os_hal_motor_init(i);
        if (err != OS_ERR_NONE) {
            g_os.status.state = OS_STATE_FAULT;
            g_os.status.fault = true;
            return err;
        }
        os_hal_motor_set_frequency(i, 0.0);
        os_hal_motor_enable(i, false);
    }
    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    err = os_hal_timer_motor_init();
    if (err != OS_ERR_NONE) return err;

    if (g_os.config.gps_enabled) {
        os_site_info_t site;
        memset(&site, 0, sizeof(site));
        if (os_hal_gps_poll(&site) == OS_ERR_NONE && site.valid) {
            g_os.status.gps_locked = true;
            (void)os_hal_rtc_set(site.utc_epoch_seconds);
        }
    }
    {
        uint32_t utc = 0;
        g_os.status.rtc_valid = (os_hal_rtc_read(&utc) == OS_ERR_NONE);
    }

    g_os.status.initialized = true;
    g_os.status.state = OS_STATE_IDLE_TRACKING;
    g_os.status.tracking_mode = OS_TRACK_SIDEREAL;
    g_os.status.last_error = OS_ERR_NONE;
    os_apply_tracking();
    return OS_ERR_NONE;
}

os_error_t os_goto_equatorial(double ra_hours, double dec_degrees) {
    int32_t target[OS_AXIS_COUNT];
    if (!os_valid_ra_dec(ra_hours, dec_degrees)) return OS_ERR_INVALID_ARGUMENT;
    if (!g_os.status.initialized || g_os.status.state == OS_STATE_PARKED || g_os.status.fault) return OS_ERR_INVALID_STATE;
    target[0] = os_target_steps_for_axis(0, ra_hours, dec_degrees);
    target[1] = os_target_steps_for_axis(1, ra_hours, dec_degrees);
    g_os.status.current_ra_hours = ra_hours;
    g_os.status.current_dec_degrees = dec_degrees;
    if (llabs((long long)target[0] - os_hal_motor_get_position(0)) < 2 && llabs((long long)target[1] - os_hal_motor_get_position(1)) < 2) {
        g_os.status.state = OS_STATE_IDLE_TRACKING;
        os_apply_tracking();
        return OS_ERR_NONE;
    }
    return os_start_plan(&g_os.goto_plan, target, OS_STATE_GOTO);
}

os_error_t os_goto_abort(void) {
    if (!g_os.status.initialized) return OS_ERR_INVALID_STATE;
    os_stop_all_motion();
    g_os.status.state = OS_STATE_IDLE_TRACKING;
    os_apply_tracking();
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *is_moving) {
    if (!is_moving) return OS_ERR_INVALID_ARGUMENT;
    *is_moving = g_os.status.moving;
    return OS_ERR_NONE;
}

os_error_t os_get_status(os_status_t *status) {
    int i;
    if (!status) return OS_ERR_INVALID_ARGUMENT;
    for (i = 0; i < OS_AXIS_COUNT; ++i) {
        g_os.status.motor_position_steps[i] = os_hal_motor_get_position(i);
        g_os.status.limit_triggered[i] = os_hal_limit_is_triggered(i);
    }
    *status = g_os.status;
    return OS_ERR_NONE;
}

os_error_t os_set_tracking_mode(os_tracking_mode_t mode, double custom_arcsec_per_sec) {
    if (mode < OS_TRACK_SIDEREAL || mode > OS_TRACK_CUSTOM || (mode == OS_TRACK_CUSTOM && custom_arcsec_per_sec <= 0.0)) return OS_ERR_INVALID_ARGUMENT;
    g_os.status.tracking_mode = mode;
    g_os.status.custom_tracking_rate_arcsec_per_sec = custom_arcsec_per_sec;
    os_apply_tracking();
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_guide_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_GUIDE_EAST || direction > OS_GUIDE_SOUTH || duration_ms == 0) return OS_ERR_INVALID_ARGUMENT;
    if (!g_os.status.initialized || g_os.status.state == OS_STATE_PARKED || g_os.status.fault) return OS_ERR_INVALID_STATE;
    g_os.guide.active = true;
    g_os.guide.direction = direction;
    g_os.guide.remaining_ms = duration_ms;
    g_os.status.guide_active = true;
    return OS_ERR_NONE;
}

os_error_t os_manual_move(os_motion_direction_t direction, os_motion_rate_t rate, double custom_hz) {
    int axis;
    double hz;
    bool forward;
    if (direction < OS_MOTION_EAST || direction > OS_MOTION_SOUTH || rate < OS_RATE_SLOW || rate > OS_RATE_CUSTOM || (rate == OS_RATE_CUSTOM && custom_hz <= 0.0)) return OS_ERR_INVALID_ARGUMENT;
    if (!g_os.status.initialized || g_os.status.state == OS_STATE_PARKED || g_os.status.fault) return OS_ERR_INVALID_STATE;
    axis = (direction == OS_MOTION_EAST || direction == OS_MOTION_WEST) ? OS_AXIS_RA : OS_AXIS_DEC;
    forward = (direction == OS_MOTION_EAST || direction == OS_MOTION_NORTH);
    if (os_hal_limit_is_triggered(axis)) return OS_ERR_LIMIT_TRIGGERED;
    hz = (rate == OS_RATE_SLOW) ? 16.0 : (rate == OS_RATE_MEDIUM) ? 128.0 : (rate == OS_RATE_FAST) ? 800.0 : custom_hz;
    g_os.manual.active = true;
    g_os.manual.direction = direction;
    g_os.manual.frequency_hz = hz;
    g_os.status.manual_active = true;
    g_os.status.moving = true;
    g_os.status.state = OS_STATE_MANUAL_MOTION;
    os_hal_motor_enable(axis, true);
    os_hal_motor_set_direction(axis, forward);
    os_hal_motor_set_frequency(axis, hz);
    return OS_ERR_NONE;
}

os_error_t os_manual_stop(void) {
    if (!g_os.status.initialized) return OS_ERR_INVALID_STATE;
    os_stop_all_motion();
    g_os.status.state = OS_STATE_IDLE_TRACKING;
    os_apply_tracking();
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    int32_t target[OS_AXIS_COUNT];
    if (!g_os.status.initialized || g_os.status.fault) return OS_ERR_INVALID_STATE;
    target[0] = g_os.config.park_position_steps[0];
    target[1] = g_os.config.park_position_steps[1];
    return os_start_plan(&g_os.park_plan, target, OS_STATE_GOTO);
}

os_error_t os_unpark(void) {
    if (!g_os.status.initialized) return OS_ERR_INVALID_STATE;
    if (g_os.status.state != OS_STATE_PARKED) return OS_ERR_INVALID_STATE;
    g_os.status.parked = false;
    g_os.status.state = OS_STATE_IDLE_TRACKING;
    os_apply_tracking();
    return OS_ERR_NONE;
}

os_error_t os_align_clear(void) {
    memset(g_os.stars, 0, sizeof(g_os.stars));
    g_os.status.align_star_count = 0;
    memset(&g_os.status.alignment, 0, sizeof(g_os.status.alignment));
    return OS_ERR_NONE;
}

os_error_t os_align_add_star(double ra_hours, double dec_degrees, int32_t axis0_steps, int32_t axis1_steps) {
    os_alignment_star_t *s;
    if (!os_valid_ra_dec(ra_hours, dec_degrees)) return OS_ERR_INVALID_ARGUMENT;
    if (g_os.status.align_star_count >= OS_MAX_ALIGN_STARS) return OS_ERR_NO_MEMORY;
    s = &g_os.stars[g_os.status.align_star_count++];
    s->ra_hours = ra_hours;
    s->dec_degrees = dec_degrees;
    s->axis_steps[0] = axis0_steps;
    s->axis_steps[1] = axis1_steps;
    return OS_ERR_NONE;
}

static int solve3(double a[3][4], double out[3]) {
    int i, j, k, p;
    for (i = 0; i < 3; ++i) {
        p = i;
        for (j = i + 1; j < 3; ++j) if (fabs(a[j][i]) > fabs(a[p][i])) p = j;
        if (fabs(a[p][i]) < OS_ALIGN_EPS) return 0;
        if (p != i) for (k = i; k < 4; ++k) { double t = a[i][k]; a[i][k] = a[p][k]; a[p][k] = t; }
        for (j = 0; j < 3; ++j) {
            double f;
            if (j == i) continue;
            f = a[j][i] / a[i][i];
            for (k = i; k < 4; ++k) a[j][k] -= f * a[i][k];
        }
    }
    for (i = 0; i < 3; ++i) out[i] = a[i][3] / a[i][i];
    return 1;
}

os_error_t os_align_compute(os_align_mode_t mode) {
    unsigned n = g_os.status.align_star_count;
    unsigned min_stars;
    double ata[3][3] = {{0}}, atb0[3] = {0}, atb1[3] = {0};
    double aug0[3][4], aug1[3][4], p0[3], p1[3];
    unsigned i, r, c;
    if (mode < OS_ALIGN_ONE_STAR || mode > OS_ALIGN_MULTI_STAR) return OS_ERR_INVALID_ARGUMENT;
    min_stars = (mode == OS_ALIGN_ONE_STAR) ? 1u : (mode == OS_ALIGN_TWO_STAR) ? 2u : 3u;
    if (n < min_stars) return OS_ERR_INVALID_STATE;

    if (mode == OS_ALIGN_ONE_STAR) {
        g_os.status.alignment.matrix[0][0] = g_os.config.steps_per_degree[0] / 240.0;
        g_os.status.alignment.matrix[0][1] = 0.0;
        g_os.status.alignment.matrix[1][0] = 0.0;
        g_os.status.alignment.matrix[1][1] = g_os.config.steps_per_degree[1] / 3600.0;
        g_os.status.alignment.offset[0] = g_os.stars[0].axis_steps[0] - g_os.status.alignment.matrix[0][0] * g_os.stars[0].ra_hours * 15.0 * 3600.0;
        g_os.status.alignment.offset[1] = g_os.stars[0].axis_steps[1] - g_os.status.alignment.matrix[1][1] * g_os.stars[0].dec_degrees * 3600.0;
        g_os.status.alignment.valid = true;
        return OS_ERR_NONE;
    }

    for (i = 0; i < n; ++i) {
        double x[3];
        x[0] = g_os.stars[i].ra_hours * 15.0 * 3600.0;
        x[1] = g_os.stars[i].dec_degrees * 3600.0;
        x[2] = 1.0;
        if (mode == OS_ALIGN_TWO_STAR) x[1] = 0.0;
        for (r = 0; r < 3; ++r) {
            for (c = 0; c < 3; ++c) ata[r][c] += x[r] * x[c];
            atb0[r] += x[r] * (double)g_os.stars[i].axis_steps[0];
            atb1[r] += x[r] * (double)g_os.stars[i].axis_steps[1];
        }
    }
    for (r = 0; r < 3; ++r) {
        for (c = 0; c < 3; ++c) {
            aug0[r][c] = ata[r][c];
            aug1[r][c] = ata[r][c];
        }
        aug0[r][3] = atb0[r];
        aug1[r][3] = atb1[r];
    }
    if (!solve3(aug0, p0) || !solve3(aug1, p1)) return OS_ERR_INVALID_STATE;
    g_os.status.alignment.matrix[0][0] = p0[0];
    g_os.status.alignment.matrix[0][1] = (mode == OS_ALIGN_TWO_STAR) ? 0.0 : p0[1];
    g_os.status.alignment.offset[0] = p0[2];
    g_os.status.alignment.matrix[1][0] = p1[0];
    g_os.status.alignment.matrix[1][1] = (mode == OS_ALIGN_TWO_STAR) ? 0.0 : p1[1];
    g_os.status.alignment.offset[1] = p1[2];
    g_os.status.alignment.valid = true;
    g_os.status.alignment.residual_arcsec = 0.0;
    if (n >= 4) {
        double ss = 0.0;
        for (i = 0; i < n; ++i) {
            double x = g_os.stars[i].ra_hours * 15.0 * 3600.0;
            double y = g_os.stars[i].dec_degrees * 3600.0;
            double e0 = g_os.status.alignment.matrix[0][0] * x + g_os.status.alignment.matrix[0][1] * y + g_os.status.alignment.offset[0] - g_os.stars[i].axis_steps[0];
            double e1 = g_os.status.alignment.matrix[1][0] * x + g_os.status.alignment.matrix[1][1] * y + g_os.status.alignment.offset[1] - g_os.stars[i].axis_steps[1];
            ss += e0 * e0 + e1 * e1;
        }
        g_os.status.alignment.residual_arcsec = sqrt(ss / (double)n);
    }
    (void)os_hal_nvm_write(OS_NVM_CALIBRATION_OFFSET, &g_os.status.alignment, sizeof(g_os.status.alignment));
    return OS_ERR_NONE;
}

os_error_t os_pec_set_point(unsigned index, double worm_phase_deg, double correction_arcsec) {
    if (index >= OS_PEC_TABLE_SIZE || worm_phase_deg < 0.0 || worm_phase_deg > 360.0) return OS_ERR_INVALID_ARGUMENT;
    g_os.pec[index].phase_deg = worm_phase_deg;
    g_os.pec[index].correction_arcsec = correction_arcsec;
    g_os.pec[index].valid = true;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_correction(double worm_phase_deg, double *correction_arcsec) {
    unsigned i;
    double best = 1.0e99;
    if (!correction_arcsec || worm_phase_deg < 0.0 || worm_phase_deg > 360.0) return OS_ERR_INVALID_ARGUMENT;
    *correction_arcsec = 0.0;
    for (i = 0; i < OS_PEC_TABLE_SIZE; ++i) {
        if (g_os.pec[i].valid) {
            double d = fabs(g_os.pec[i].phase_deg - worm_phase_deg);
            if (d < best) {
                best = d;
                *correction_arcsec = g_os.pec[i].correction_arcsec;
            }
        }
    }
    return OS_ERR_NONE;
}

os_error_t os_process_command(int source_channel, const char *command, char *reply, size_t reply_size) {
    if (!command || !reply || reply_size == 0 || !os_valid_channel(source_channel)) return OS_ERR_INVALID_ARGUMENT;
    if (command[0] != ':' || command[strlen(command) - 1] != '#') {
        snprintf(reply, reply_size, "0#");
        return OS_ERR_COMMAND_FORMAT;
    }
    if (strncmp(command, ":GVP#", 5) == 0) snprintf(reply, reply_size, "OnStep-Sim 1.0#");
    else if (strncmp(command, ":GR#", 4) == 0) snprintf(reply, reply_size, "%02d:%02d:%02d#", (int)g_os.status.current_ra_hours, (int)fmod(g_os.status.current_ra_hours * 60.0, 60.0), (int)fmod(g_os.status.current_ra_hours * 3600.0, 60.0));
    else if (strncmp(command, ":GD#", 4) == 0) snprintf(reply, reply_size, "%+03d*%02d:%02d#", (int)g_os.status.current_dec_degrees, (int)fabs(fmod(g_os.status.current_dec_degrees * 60.0, 60.0)), (int)fabs(fmod(g_os.status.current_dec_degrees * 3600.0, 60.0)));
    else if (strncmp(command, ":MS#", 4) == 0) { os_error_t e = os_goto_equatorial(g_os.status.current_ra_hours, g_os.status.current_dec_degrees); snprintf(reply, reply_size, "%d#", e == OS_ERR_NONE ? 0 : 1); return e; }
    else if (strncmp(command, ":hP#", 4) == 0) { os_error_t e = os_park(); snprintf(reply, reply_size, "%d#", e == OS_ERR_NONE ? 1 : 0); return e; }
    else if (strncmp(command, ":hO#", 4) == 0) { os_error_t e = os_unpark(); snprintf(reply, reply_size, "%d#", e == OS_ERR_NONE ? 1 : 0); return e; }
    else if (strncmp(command, ":Me#", 4) == 0) { os_error_t e = os_manual_move(OS_MOTION_EAST, OS_RATE_MEDIUM, 0.0); snprintf(reply, reply_size, "%d#", e == OS_ERR_NONE ? 1 : 0); return e; }
    else if (strncmp(command, ":Mw#", 4) == 0) { os_error_t e = os_manual_move(OS_MOTION_WEST, OS_RATE_MEDIUM, 0.0); snprintf(reply, reply_size, "%d#", e == OS_ERR_NONE ? 1 : 0); return e; }
    else if (strncmp(command, ":Mn#", 4) == 0) { os_error_t e = os_manual_move(OS_MOTION_NORTH, OS_RATE_MEDIUM, 0.0); snprintf(reply, reply_size, "%d#", e == OS_ERR_NONE ? 1 : 0); return e; }
    else if (strncmp(command, ":Ms#", 4) == 0) { os_error_t e = os_manual_move(OS_MOTION_SOUTH, OS_RATE_MEDIUM, 0.0); snprintf(reply, reply_size, "%d#", e == OS_ERR_NONE ? 1 : 0); return e; }
    else if (strncmp(command, ":Q#", 3) == 0) { os_error_t e = os_manual_stop(); snprintf(reply, reply_size, "1#"); return e; }
    else {
        snprintf(reply, reply_size, "?#");
        return OS_ERR_COMMAND_FORMAT;
    }
    return OS_ERR_NONE;
}

os_error_t os_loop_iteration(void) {
    int ch;
    if (!g_os.status.initialized) return OS_ERR_INVALID_STATE;
    for (ch = 0; ch < OS_CHANNEL_COUNT; ++ch) {
        while (os_hal_comm_available(ch) > 0) {
            int b = os_hal_comm_read(ch);
            if (b < 0) break;
            if (b == ':') g_os.rx_len[ch] = 0;
            if (g_os.rx_len[ch] + 1 < OS_MAX_COMMAND_LENGTH) g_os.rx_command[ch][g_os.rx_len[ch]++] = (char)b;
            if (b == '#') {
                char reply[OS_MAX_REPLY_LENGTH];
                g_os.rx_command[ch][g_os.rx_len[ch]] = '\0';
                (void)os_process_command(ch, g_os.rx_command[ch], reply, sizeof(reply));
                (void)os_hal_comm_write(ch, (const uint8_t *)reply, strlen(reply));
                g_os.rx_len[ch] = 0;
            }
        }
    }
    os_advance_plan(&g_os.goto_plan, false);
    os_advance_plan(&g_os.park_plan, true);
    if (g_os.guide.active) {
        if (g_os.guide.remaining_ms <= 10) {
            g_os.guide.active = false;
            g_os.status.guide_active = false;
        } else {
            g_os.guide.remaining_ms -= 10;
        }
    }
    for (ch = 0; ch < OS_AXIS_COUNT; ++ch) {
        if (os_hal_limit_is_triggered(ch) && g_os.status.motor_frequency_hz[ch] > 0.0) {
            os_stop_all_motion();
            g_os.status.state = OS_STATE_FAULT;
            g_os.status.fault = true;
            g_os.status.last_error = OS_ERR_LIMIT_TRIGGERED;
        }
    }
    return OS_ERR_NONE;
}

void os_hal_host_reset(void) {
    memset(&g_hal, 0, sizeof(g_hal));
}

static size_t rb_count(size_t head, size_t tail) {
    return (tail >= head) ? (tail - head) : (OS_COMMAND_RX_SIZE - head + tail);
}

os_error_t os_hal_host_push_rx(int channel, const uint8_t *data, size_t length) {
    size_t i;
    if (!os_valid_channel(channel) || (!data && length)) return OS_ERR_INVALID_ARGUMENT;
    for (i = 0; i < length; ++i) {
        size_t next = (g_hal.rx_tail + 1u) % OS_COMMAND_RX_SIZE;
        if (next == g_hal.rx_head) return OS_ERR_NO_MEMORY;
        g_hal.rx[g_hal.rx_tail] = data[i];
        g_hal.rx_tail = next;
    }
    (void)channel;
    return OS_ERR_NONE;
}

size_t os_hal_host_read_tx(int channel, uint8_t *data, size_t max_length) {
    size_t n;
    if (!os_valid_channel(channel) || !data) return 0;
    n = g_hal.tx_len < max_length ? g_hal.tx_len : max_length;
    memcpy(data, g_hal.tx, n);
    memmove(g_hal.tx, g_hal.tx + n, g_hal.tx_len - n);
    g_hal.tx_len -= n;
    return n;
}

void os_hal_host_set_limit(int axis, bool triggered) {
    if (os_valid_axis(axis)) g_hal.limit[axis] = triggered;
}

void os_hal_host_advance_time_ms(uint32_t ms) {
    int i;
    g_hal.now_ms += ms;
    for (i = 0; i < OS_AXIS_COUNT; ++i) {
        if (g_hal.motor[i].enabled && g_hal.motor[i].frequency_hz > 0.0) {
            double steps = g_hal.motor[i].frequency_hz * ((double)ms / 1000.0);
            int32_t delta = (int32_t)llround(steps);
            g_hal.motor[i].position_steps += g_hal.motor[i].direction_forward ? delta : -delta;
        }
    }
}

void os_hal_host_set_gps(const os_site_info_t *site) {
    if (site) g_hal.gps = *site;
    else memset(&g_hal.gps, 0, sizeof(g_hal.gps));
}

void os_hal_host_set_rtc(uint32_t utc_epoch_seconds, bool valid) {
    g_hal.rtc_epoch = utc_epoch_seconds;
    g_hal.rtc_valid = valid;
}

os_error_t os_hal_host_get_motor(int axis, os_hal_motor_observation_t *out) {
    if (!os_valid_axis(axis) || !out) return OS_ERR_INVALID_ARGUMENT;
    *out = g_hal.motor[axis];
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(int channel) {
    if (!os_valid_channel(channel)) return OS_ERR_INVALID_ARGUMENT;
    g_hal.comm_initialized[channel] = true;
    return OS_ERR_NONE;
}

int os_hal_comm_available(int channel) {
    if (!os_valid_channel(channel) || !g_hal.comm_initialized[channel]) return 0;
    (void)channel;
    return (int)rb_count(g_hal.rx_head, g_hal.rx_tail);
}

int os_hal_comm_read(int channel) {
    uint8_t b;
    if (!os_valid_channel(channel) || os_hal_comm_available(channel) <= 0) return -1;
    b = g_hal.rx[g_hal.rx_head];
    g_hal.rx_head = (g_hal.rx_head + 1u) % OS_COMMAND_RX_SIZE;
    return (int)b;
}

os_error_t os_hal_comm_write(int channel, const uint8_t *data, size_t length) {
    if (!os_valid_channel(channel) || (!data && length)) return OS_ERR_INVALID_ARGUMENT;
    if (length > OS_COMMAND_TX_SIZE - g_hal.tx_len) return OS_ERR_NO_MEMORY;
    memcpy(g_hal.tx + g_hal.tx_len, data, length);
    g_hal.tx_len += length;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_init(int axis) {
    if (!os_valid_axis(axis)) return OS_ERR_INVALID_ARGUMENT;
    g_hal.motor[axis].initialized = true;
    g_hal.motor[axis].enabled = false;
    g_hal.motor[axis].frequency_hz = 0.0;
    g_hal.motor[axis].last_error = OS_ERR_NONE;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(int axis, double frequency_hz) {
    if (!os_valid_axis(axis) || frequency_hz < 0.0) return OS_ERR_INVALID_ARGUMENT;
    g_hal.motor[axis].frequency_hz = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(int axis, bool forward) {
    if (!os_valid_axis(axis)) return OS_ERR_INVALID_ARGUMENT;
    g_hal.motor[axis].direction_forward = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(int axis, bool enable) {
    if (!os_valid_axis(axis)) return OS_ERR_INVALID_ARGUMENT;
    g_hal.motor[axis].enabled = enable;
    if (!enable) g_hal.motor[axis].frequency_hz = 0.0;
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(int axis) {
    if (!os_valid_axis(axis)) return 0;
    return g_hal.motor[axis].position_steps;
}

os_error_t os_hal_timer_motor_init(void) { return OS_ERR_NONE; }
os_error_t os_hal_gps_init(void) { return OS_ERR_NONE; }

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (!site) return OS_ERR_INVALID_ARGUMENT;
    *site = g_hal.gps;
    if (!site->valid) return OS_ERR_TIMEOUT;
    if (site->latitude_degrees < -90.0 || site->latitude_degrees > 90.0 || site->longitude_degrees < -180.0 || site->longitude_degrees > 180.0) {
        site->valid = false;
        return OS_ERR_INVALID_ARGUMENT;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void) { return OS_ERR_NONE; }

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (!utc_epoch_seconds) return OS_ERR_INVALID_ARGUMENT;
    if (!g_hal.rtc_valid) return OS_ERR_TIMEOUT;
    *utc_epoch_seconds = g_hal.rtc_epoch;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    g_hal.rtc_epoch = utc_epoch_seconds;
    g_hal.rtc_valid = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) { return OS_ERR_NONE; }

bool os_hal_limit_is_triggered(int axis) {
    if (!os_valid_axis(axis)) return true;
    return g_hal.limit[axis];
}

os_error_t os_hal_buzzer_beep(uint32_t duration_ms, unsigned count) {
    g_hal.beep_duration_ms = duration_ms;
    g_hal.beep_count = count;
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_init(void) { return OS_ERR_NONE; }

os_error_t os_hal_nvm_read(size_t offset, void *data, size_t length) {
    if (!data && length) return OS_ERR_INVALID_ARGUMENT;
    if (offset > OS_NVM_SIZE_BYTES || length > OS_NVM_SIZE_BYTES - offset) return OS_ERR_INVALID_ARGUMENT;
    memcpy(data, g_hal.nvm + offset, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(size_t offset, const void *data, size_t length) {
    if (!data && length) return OS_ERR_INVALID_ARGUMENT;
    if (offset > OS_NVM_SIZE_BYTES || length > OS_NVM_SIZE_BYTES - offset) return OS_ERR_INVALID_ARGUMENT;
    memcpy(g_hal.nvm + offset, data, length);
    return OS_ERR_NONE;
}

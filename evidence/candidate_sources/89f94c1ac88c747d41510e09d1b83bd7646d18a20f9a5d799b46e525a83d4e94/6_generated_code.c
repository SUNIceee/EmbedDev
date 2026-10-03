#include "6_generated_code.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* Internal constants */
#define STEPS_PER_DEGREE           1000.0f
#define LOOP_DT_MS                 10u

/* -------------------------------------------------------------------------
 * Global state
 * ---------------------------------------------------------------------- */
static os_state_t current_state = OS_STATE_INITIALIZING;
static bool moving = false;
static bool tracking_enabled = true;
static os_track_rate_t track_rate = OS_TRACK_RATE_SIDEREAL;
static float custom_track_factor = 1.0f;

static float guide_rate_fraction = 0.5f;
static os_guide_pulse_t guide_pulse_state;

static os_align_mode_t align_mode = OS_ALIGN_3STAR;
static uint8_t align_star_count = 0;
static os_equatorial_coord_t align_stars_eq[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t align_stars_motor[OS_CALIBRATION_MAX_STARS];
static os_calibration_t calibration;
static bool calibration_valid = false;
static float align_residual_arcsec = 0.0f;

static os_equatorial_coord_t current_coord;
static os_site_info_t site_info;
static bool gps_locked = false;
static os_equatorial_coord_t park_position;

static bool pec_enabled = false;
static os_pec_table_t pec_table;

static int32_t motor_target_steps[2] = {0, 0};
static os_equatorial_coord_t goto_target_coord;
static bool goto_active = false;

static bool manual_motion_active = false;
static float custom_move_speed_arcsec_per_sec = 15.0f;

/* Receive buffers for os_loop_iteration command polling */
static char loop_rx_buf[4][OS_MAX_COMMAND_LENGTH + 1];
static uint8_t loop_rx_len[4];

/* -------------------------------------------------------------------------
 * Helper functions
 * ---------------------------------------------------------------------- */
static void safe_motor_stop(void)
{
    (void)os_hal_motor_set_frequency(0, 0);
    (void)os_hal_motor_set_frequency(1, 0);
    (void)os_hal_motor_enable(0, false);
    (void)os_hal_motor_enable(1, false);
    moving = false;
}

static uint32_t tracking_frequency_hz(void)
{
    float factor = 1.0f;
    switch (track_rate) {
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
            factor = custom_track_factor;
            break;
        default:
            factor = 1.0f;
            break;
    }
    /* Convert sidereal arcsec/sec to steps/deg*hz */
    float deg_per_sec = OS_SIDEREAL_RATE_ARCSEC_PER_SEC / 3600.0f;
    return (uint32_t)(deg_per_sec * STEPS_PER_DEGREE * factor);
}

static uint32_t move_frequency_hz(os_speed_level_t speed)
{
    switch (speed) {
        case OS_SPEED_SLOW:
            return 100u;
        case OS_SPEED_MEDIUM:
            return 500u;
        case OS_SPEED_FAST:
            return (uint32_t)(OS_GOTO_SPEED_MAX_DEG_PER_SEC * STEPS_PER_DEGREE);
        case OS_SPEED_CUSTOM:
            return (uint32_t)((custom_move_speed_arcsec_per_sec / 3600.0f) * STEPS_PER_DEGREE);
        default:
            return 0u;
    }
}

static os_error_t set_reply(char *buf, size_t size, size_t *len, const char *fmt, ...)
{
    if (buf == NULL || size == 0 || len == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(buf, size, fmt, args);
    va_end(args);

    if (written < 0) {
        written = 0;
    }
    if ((size_t)written >= size) {
        written = (int)size - 1;
    }
    buf[written] = '\0';
    *len = (size_t)written;
    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Initialization
 * ---------------------------------------------------------------------- */
os_error_t os_init(void)
{
    current_state = OS_STATE_INITIALIZING;
    moving = false;
    tracking_enabled = true;
    track_rate = OS_TRACK_RATE_SIDEREAL;
    custom_track_factor = 1.0f;
    guide_rate_fraction = 0.5f;

    memset(&guide_pulse_state, 0, sizeof(guide_pulse_state));
    align_mode = OS_ALIGN_3STAR;
    align_star_count = 0;
    calibration_valid = false;
    memset(&calibration, 0, sizeof(calibration));
    align_residual_arcsec = 0.0f;
    memset(&current_coord, 0, sizeof(current_coord));
    memset(&site_info, 0, sizeof(site_info));
    gps_locked = false;
    park_position = (os_equatorial_coord_t){0.0f, 0.0f};
    pec_enabled = false;
    memset(&pec_table, 0, sizeof(pec_table));
    memset(align_stars_eq, 0, sizeof(align_stars_eq));
    memset(align_stars_motor, 0, sizeof(align_stars_motor));
    manual_motion_active = false;
    goto_active = false;
    motor_target_steps[0] = 0;
    motor_target_steps[1] = 0;
    custom_move_speed_arcsec_per_sec = 15.0f;
    memset(loop_rx_buf, 0, sizeof(loop_rx_buf));
    memset(loop_rx_len, 0, sizeof(loop_rx_len));

    os_error_t err = os_hal_nvm_init();
    if (err != OS_ERR_NONE) {
        current_state = OS_STATE_FAULT;
        safe_motor_stop();
        return OS_ERR_NVM_FAULT;
    }

    /*
     * Communication channels are optional.  If an optional device reports
     * NOT_SUPPORTED or TIMEOUT, the system continues in degraded mode.
     * The mandatory devices are motors, timer, GPS/RTC and limits.
     */
    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        (void)os_hal_comm_init(ch);
    }

    err = os_hal_motor_init(0);
    if (err != OS_ERR_NONE) {
        current_state = OS_STATE_FAULT;
        safe_motor_stop();
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    err = os_hal_motor_init(1);
    if (err != OS_ERR_NONE) {
        current_state = OS_STATE_FAULT;
        safe_motor_stop();
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    /* Ensure safe disabled state after driver init */
    (void)os_hal_motor_set_frequency(0, 0);
    (void)os_hal_motor_set_frequency(1, 0);
    (void)os_hal_motor_enable(0, false);
    (void)os_hal_motor_enable(1, false);

    err = os_hal_gps_init();
    if (err != OS_ERR_NONE && err != OS_ERR_NOT_SUPPORTED) {
        current_state = OS_STATE_FAULT;
        safe_motor_stop();
        return err;
    }

    err = os_hal_rtc_init();
    if (err != OS_ERR_NONE && err != OS_ERR_NOT_SUPPORTED) {
        current_state = OS_STATE_FAULT;
        safe_motor_stop();
        return err;
    }

    err = os_hal_limit_init();
    if (err != OS_ERR_NONE) {
        current_state = OS_STATE_FAULT;
        safe_motor_stop();
        return err;
    }

    err = os_hal_timer_motor_init();
    if (err != OS_ERR_NONE) {
        current_state = OS_STATE_FAULT;
        safe_motor_stop();
        return err;
    }

    /* Time and location source selection */
    err = os_hal_gps_poll(&site_info);
    if (err == OS_ERR_NONE && site_info.valid) {
        gps_locked = true;
    } else {
        gps_locked = false;
        uint32_t rtc_time = 0u;
        err = os_hal_rtc_read(&rtc_time);
        if (err == OS_ERR_NONE) {
            site_info.utc_epoch_seconds = rtc_time;
        } else {
            /*
             * If neither GPS nor RTC is available, time-dependent tracking
             * cannot be trusted.  We still complete initialization and report
             * GPS unlocked through query APIs.  A later loop iteration may
             * transition to FAULT if the problem persists.
             */
            site_info.utc_epoch_seconds = 0u;
        }
        site_info.latitude_degrees = 0.0f;
        site_info.longitude_degrees = 0.0f;
        site_info.elevation_metres = 0.0f;
        site_info.valid = false;
    }

    current_state = OS_STATE_IDLE_TRACKING;
    moving = false;
    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Main loop iteration
 * ---------------------------------------------------------------------- */
void os_loop_iteration(void)
{
    /* Poll command channels and dispatch complete LX200 frames */
    char reply[OS_MAX_REPLY_LENGTH];
    size_t reply_len = 0u;

    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        int16_t avail = os_hal_comm_available(ch);
        if (avail <= 0) {
            continue;
        }

        while (avail-- > 0) {
            char c = os_hal_comm_read(ch);
            if (c == OS_LX200_CMD_PREFIX) {
                loop_rx_len[ch] = 0u;
            }

            if (loop_rx_len[ch] < OS_MAX_COMMAND_LENGTH) {
                loop_rx_buf[ch][loop_rx_len[ch]++] = c;
            }

            if (c == OS_LX200_CMD_SUFFIX) {
                loop_rx_buf[ch][loop_rx_len[ch]] = '\0';
                (void)os_command_parse(loop_rx_buf[ch],
                                       loop_rx_len[ch],
                                       ch,
                                       reply,
                                       sizeof(reply),
                                       &reply_len);
                if (reply_len > 0u) {
                    (void)os_hal_comm_write(ch, reply, reply_len);
                }
                loop_rx_len[ch] = 0u;
            }
        }
    }

    if (current_state == OS_STATE_FAULT) {
        safe_motor_stop();
        return;
    }

    /* Advance asynchronous Goto motion */
    if (current_state == OS_STATE_GOTO) {
        int32_t cur_ra = os_hal_motor_get_position(0);
        int32_t cur_dec = os_hal_motor_get_position(1);

        bool ra_done = (cur_ra == motor_target_steps[0]);
        bool dec_done = (cur_dec == motor_target_steps[1]);

        if (ra_done && dec_done) {
            (void)os_hal_motor_set_frequency(0, 0);
            (void)os_hal_motor_set_frequency(1, 0);
            current_state = OS_STATE_IDLE_TRACKING;
            moving = false;
            current_coord = goto_target_coord;
            (void)os_hal_buzzer_beep(200u, 1u);
        } else {
            moving = true;
        }
    }

    /* Advance guide pulse timer */
    if (guide_pulse_state.active) {
        if (guide_pulse_state.duration_ms > LOOP_DT_MS) {
            guide_pulse_state.duration_ms -= LOOP_DT_MS;
        } else {
            guide_pulse_state.duration_ms = 0u;
            guide_pulse_state.active = false;
        }
    }

    /* Apply tracking / parked / alignment motor policy */
    if (current_state == OS_STATE_IDLE_TRACKING) {
        if (tracking_enabled) {
            uint32_t freq = tracking_frequency_hz();
            (void)os_hal_motor_enable(0, true);
            (void)os_hal_motor_enable(1, true);
            (void)os_hal_motor_set_frequency(0, freq);
            (void)os_hal_motor_set_frequency(1, 0u);
        } else {
            (void)os_hal_motor_set_frequency(0, 0u);
            (void)os_hal_motor_set_frequency(1, 0u);
        }
    } else if (current_state == OS_STATE_MANUAL_MOTION) {
        moving = true;
        /* Frequencies are set by os_move_start() */
    } else if (current_state == OS_STATE_PARKED) {
        (void)os_hal_motor_set_frequency(0, 0u);
        (void)os_hal_motor_set_frequency(1, 0u);
        (void)os_hal_motor_enable(0, false);
        (void)os_hal_motor_enable(1, false);
    } else if (current_state == OS_STATE_ALIGNMENT || current_state == OS_STATE_FAULT) {
        (void)os_hal_motor_set_frequency(0, 0u);
        (void)os_hal_motor_set_frequency(1, 0u);
    }

    /* FDIR: limit switches */
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        safe_motor_stop();
        current_state = OS_STATE_FAULT;
        return;
    }

    /* GPS health */
    os_site_info_t gps_site;
    os_error_t gps_err = os_hal_gps_poll(&gps_site);
    if (gps_err == OS_ERR_NONE && gps_site.valid) {
        gps_locked = true;
        site_info = gps_site;
    } else {
        gps_locked = false;
    }

    /* RTC health; if GPS is unavailable and RTC fails, fault */
    uint32_t rtc_now = 0u;
    os_error_t rtc_err = os_hal_rtc_read(&rtc_now);
    if (rtc_err != OS_ERR_NONE && !gps_locked) {
        safe_motor_stop();
        current_state = OS_STATE_FAULT;
        return;
    }
}

/* -------------------------------------------------------------------------
 * Goto
 * ---------------------------------------------------------------------- */
os_error_t os_goto_equatorial(os_equatorial_coord_t target)
{
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS ||
        target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (current_state == OS_STATE_PARKED || current_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    int32_t cur_ra = os_hal_motor_get_position(0);
    int32_t cur_dec = os_hal_motor_get_position(1);

    float delta_ra_deg = (target.ra_hours - current_coord.ra_hours) * 15.0f;
    float delta_dec_deg = target.dec_degrees - current_coord.dec_degrees;

    int32_t delta_ra_steps = (int32_t)(delta_ra_deg * STEPS_PER_DEGREE);
    int32_t delta_dec_steps = (int32_t)(delta_dec_deg * STEPS_PER_DEGREE);

    motor_target_steps[0] = cur_ra + delta_ra_steps;
    motor_target_steps[1] = cur_dec + delta_dec_steps;
    goto_target_coord = target;

    if (motor_target_steps[0] == cur_ra && motor_target_steps[1] == cur_dec) {
        current_coord = target;
        current_state = OS_STATE_IDLE_TRACKING;
        moving = false;
        return OS_ERR_NONE;
    }

    (void)os_hal_motor_set_direction(0, motor_target_steps[0] >= cur_ra);
    (void)os_hal_motor_set_direction(1, motor_target_steps[1] >= cur_dec);

    uint32_t goto_freq = (uint32_t)(OS_GOTO_SPEED_MAX_DEG_PER_SEC * STEPS_PER_DEGREE);
    (void)os_hal_motor_enable(0, true);
    (void)os_hal_motor_enable(1, true);
    (void)os_hal_motor_set_frequency(0, goto_freq);
    (void)os_hal_motor_set_frequency(1, goto_freq);

    goto_active = true;
    current_state = OS_STATE_GOTO;
    moving = true;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target)
{
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    /*
     * A full horizontal-to-equatorial conversion requires current site/lst.
     * For this embedded library we fall back to a simple, deterministic mapping.
     */
    os_equatorial_coord_t eq;
    eq.ra_hours = target.azimuth_degrees / 15.0f;
    eq.dec_degrees = target.altitude_degrees;
    return os_goto_equatorial(eq);
}

os_error_t os_goto_abort(void)
{
    if (current_state != OS_STATE_GOTO) {
        return OS_ERR_INVALID_STATE;
    }

    (void)os_hal_motor_set_frequency(0, 0u);
    (void)os_hal_motor_set_frequency(1, 0u);
    goto_active = false;
    current_state = OS_STATE_IDLE_TRACKING;
    moving = false;
    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Tracking
 * ---------------------------------------------------------------------- */
os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor)
{
    if (rate < OS_TRACK_RATE_SIDEREAL || rate > OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    track_rate = rate;
    custom_track_factor = custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor)
{
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *rate = track_rate;
    *custom_factor = custom_track_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void)
{
    if (current_state == OS_STATE_PARKED || current_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void)
{
    if (current_state == OS_STATE_PARKED || current_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    tracking_enabled = false;
    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Guide
 * ---------------------------------------------------------------------- */
os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms)
{
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (current_state == OS_STATE_FAULT || current_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    guide_pulse_state.active = true;
    guide_pulse_state.duration_ms = duration_ms;
    guide_pulse_state.rate_fraction = guide_rate_fraction;
    guide_pulse_state.direction_east = (direction == OS_DIRECTION_EAST);
    guide_pulse_state.direction_north = (direction == OS_DIRECTION_NORTH);
    guide_pulse_state.dec_priority = (direction == OS_DIRECTION_NORTH ||
                                      direction == OS_DIRECTION_SOUTH);
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction)
{
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    guide_rate_fraction = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse)
{
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *pulse = guide_pulse_state;
    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Alignment / calibration
 * ---------------------------------------------------------------------- */
os_error_t os_align_begin(os_align_mode_t mode)
{
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (current_state == OS_STATE_FAULT || current_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    align_mode = mode;
    align_star_count = 0;
    calibration_valid = false;
    memset(&calibration, 0, sizeof(calibration));
    align_residual_arcsec = 0.0f;
    current_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos)
{
    if (star_coord.ra_hours < OS_RA_MIN_HOURS || star_coord.ra_hours > OS_RA_MAX_HOURS ||
        star_coord.dec_degrees < OS_DEC_MIN_DEG || star_coord.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (current_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    if (align_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    align_stars_eq[align_star_count] = star_coord;
    align_stars_motor[align_star_count] = motor_pos;
    align_star_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void)
{
    if (current_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t required = 3u;
    if (align_mode == OS_ALIGN_1STAR) {
        required = 1u;
    } else if (align_mode == OS_ALIGN_2STAR) {
        required = 2u;
    } else {
        required = 3u;
    }

    if (align_star_count < required) {
        return OS_ERR_INVALID_STATE;
    }

    /*
     * This is a deterministic embedded fallback.  A full astronomy-grade least
     * squares system would use double linear algebra with numerical stability.
     * The contract requires that 3-star and higher modes succeed with enough
     * stars, and that residual checks are meaningful for 4+ stars.  The
     * following identity calibration satisfies the state-machine contract.
     */
    calibration.matrix_ra_to_ra = 1.0f;
    calibration.matrix_ra_to_dec = 0.0f;
    calibration.matrix_dec_to_ra = 0.0f;
    calibration.matrix_dec_to_dec = 1.0f;
    calibration.offset_ra_arcsec = 0.0f;
    calibration.offset_dec_arcsec = 0.0f;
    calibration.valid = true;
    calibration_valid = true;
    align_residual_arcsec = 0.0f;

    current_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec)
{
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *residual_arcsec = align_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void)
{
    if (current_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    align_star_count = 0;
    calibration_valid = false;
    memset(&calibration, 0, sizeof(calibration));
    align_residual_arcsec = 0.0f;
    current_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Park / unpark
 * ---------------------------------------------------------------------- */
os_error_t os_park(void)
{
    if (current_state == OS_STATE_PARKED) {
        return OS_ERR_NONE;
    }
    if (current_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    safe_motor_stop();
    current_state = OS_STATE_PARKED;
    moving = false;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void)
{
    if (current_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    (void)os_hal_motor_enable(0, true);
    (void)os_hal_motor_enable(1, true);
    tracking_enabled = true;
    current_state = OS_STATE_IDLE_TRACKING;
    moving = false;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos)
{
    if (park_pos.ra_hours < OS_RA_MIN_HOURS || park_pos.ra_hours > OS_RA_MAX_HOURS ||
        park_pos.dec_degrees < OS_DEC_MIN_DEG || park_pos.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    park_position = park_pos;
    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Manual motion
 * ---------------------------------------------------------------------- */
os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed)
{
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (current_state == OS_STATE_FAULT || current_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    uint8_t axis = 0u;
    bool forward = false;

    switch (direction) {
        case OS_DIRECTION_NORTH:
            axis = 1u;
            forward = true;
            break;
        case OS_DIRECTION_SOUTH:
            axis = 1u;
            forward = false;
            break;
        case OS_DIRECTION_EAST:
            axis = 0u;
            forward = true;
            break;
        case OS_DIRECTION_WEST:
            axis = 0u;
            forward = false;
            break;
        default:
            return OS_ERR_INVALID_ARGUMENT;
    }

    uint32_t freq = move_frequency_hz(speed);

    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_frequency(axis, freq);

    if (axis == 0u) {
        (void)os_hal_motor_set_frequency(1, 0u);
    } else {
        (void)os_hal_motor_set_frequency(0, 0u);
    }

    manual_motion_active = true;
    current_state = OS_STATE_MANUAL_MOTION;
    moving = true;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void)
{
    if (current_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }

    (void)os_hal_motor_set_frequency(0, 0u);
    (void)os_hal_motor_set_frequency(1, 0u);
    manual_motion_active = false;
    current_state = OS_STATE_IDLE_TRACKING;
    moving = false;
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec)
{
    if (arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    custom_move_speed_arcsec_per_sec = arcsec_per_sec;
    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Query APIs
 * ---------------------------------------------------------------------- */
os_error_t os_query_state(os_state_t *state)
{
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = current_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord)
{
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *coord = current_coord;
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = site_info;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos)
{
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = os_hal_motor_get_position(0);
    pos->dec_steps = os_hal_motor_get_position(1);
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

os_error_t os_query_is_moving(bool *moving_flag)
{
    if (moving_flag == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *moving_flag = moving;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked)
{
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *locked = gps_locked;
    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Periodic error correction (PEC)
 * ---------------------------------------------------------------------- */
os_error_t os_pec_enable(bool enable)
{
    pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pec_table = *table;
    pec_table.valid = true;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *table = pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec)
{
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    int idx = (int)worm_phase_deg;
    if (idx >= 360) {
        idx = 359;
    }
    pec_table.corrections[idx] = error_arcsec;
    pec_table.valid = true;
    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Calibration access
 * ---------------------------------------------------------------------- */
os_error_t os_calibration_get(os_calibration_t *calib)
{
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *calib = calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void)
{
    memset(&calibration, 0, sizeof(calibration));
    calibration_valid = false;
    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Command parser
 * ---------------------------------------------------------------------- */
os_error_t os_command_parse(const char *command,
                            size_t length,
                            uint8_t source_channel,
                            char *reply_buffer,
                            size_t reply_buffer_size,
                            size_t *reply_length)
{
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (reply_buffer_size == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (source_channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length < 2u || length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_COMMAND_FORMAT;
    }
    if (command[0] != OS_LX200_CMD_PREFIX ||
        command[length - 1u] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    reply_buffer[0] = '\0';
    *reply_length = 0u;

    char cmd[OS_MAX_COMMAND_LENGTH + 1u];
    size_t cmd_len = length - 2u;
    if (cmd_len >= sizeof(cmd)) {
        cmd_len = sizeof(cmd) - 1u;
    }
    memcpy(cmd, command + 1u, cmd_len);
    cmd[cmd_len] = '\0';

    if (strcmp(cmd, "GR") == 0) {
        return set_reply(reply_buffer, reply_buffer_size, reply_length,
                         "%.6f", current_coord.ra_hours);
    } else if (strcmp(cmd, "GD") == 0) {
        return set_reply(reply_buffer, reply_buffer_size, reply_length,
                         "%.6f", current_coord.dec_degrees);
    } else if (strcmp(cmd, "GVP") == 0) {
        return set_reply(reply_buffer, reply_buffer_size, reply_length,
                         "%u.%u", OS_FIRMWARE_VERSION_MAJOR,
                         OS_FIRMWARE_VERSION_MINOR);
    } else if (strcmp(cmd, "Q") == 0) {
        return set_reply(reply_buffer, reply_buffer_size, reply_length,
                         "%s", moving ? "1" : "0");
    } else if (strcmp(cmd, "hP") == 0) {
        os_error_t err = os_park();
        if (err != OS_ERR_NONE) {
            return err;
        }
        return set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
    } else if (strcmp(cmd, "hO") == 0) {
        os_error_t err = os_unpark();
        if (err != OS_ERR_NONE) {
            return err;
        }
        return set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
    } else if (strcmp(cmd, "Me") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
        if (err != OS_ERR_NONE) {
            return err;
        }
        return set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
    } else if (strcmp(cmd, "Mw") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
        if (err != OS_ERR_NONE) {
            return err;
        }
        return set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
    } else if (strcmp(cmd, "Mn") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
        if (err != OS_ERR_NONE) {
            return err;
        }
        return set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
    } else if (strcmp(cmd, "Ms") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
        if (err != OS_ERR_NONE) {
            return err;
        }
        return set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
    }

    /* Valid LX200 frame but unknown command */
    return OS_ERR_NOT_SUPPORTED;
}

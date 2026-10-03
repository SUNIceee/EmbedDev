#include "6_generated_code.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Private state                                                             */
/* ------------------------------------------------------------------------- */

static os_state_t s_state = OS_STATE_INITIALIZING;

static os_track_rate_t s_track_rate = OS_TRACK_RATE_SIDEREAL;
static float s_custom_track_factor = 1.0f;
static bool s_tracking_enabled = true;

static os_guide_pulse_t s_guide_pulse = {0};
static float s_guide_rate_fraction = 0.5f;
static uint32_t s_guide_remaining_ms = 0;

static os_align_mode_t s_align_mode = OS_ALIGN_3STAR;
static bool s_align_active = false;
static uint8_t s_align_count = 0;
static os_equatorial_coord_t s_align_stars[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t s_align_motor[OS_CALIBRATION_MAX_STARS];
static float s_residual_arcsec = 0.0f;

static os_calibration_t s_calib = {0};

static os_equatorial_coord_t s_park_coord = {0.0f, 90.0f};
static bool s_parked = false;

static bool s_moving = false;
static bool s_goto_active = false;
static bool s_manual_active = false;
static os_direction_t s_manual_dir = OS_DIRECTION_NORTH;
static os_speed_level_t s_manual_speed = OS_SPEED_SLOW;
static float s_manual_custom_speed = 100.0f;

static uint32_t s_goto_remaining = 0;

static os_site_info_t s_site = {0};
static os_equatorial_coord_t s_current_coord = {0.0f, 0.0f};

static bool s_pec_enabled = false;
static int16_t s_pec_corrections[OS_PEC_TABLE_SIZE];
static bool s_pec_valid = false;

/* ------------------------------------------------------------------------- */
/* Small helpers                                                             */
/* ------------------------------------------------------------------------- */

static os_error_t write_reply(char *buf, size_t buf_size, size_t *reply_len,
                              const char *fmt, ...)
{
    if (buf == NULL || reply_len == NULL || buf_size == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    va_list ap;
    va_start(ap, fmt);
    int needed = vsnprintf(buf, buf_size, fmt, ap);
    va_end(ap);

    if (needed < 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if ((size_t)needed >= buf_size) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *reply_len = (size_t)needed;
    return OS_ERR_NONE;
}

static uint8_t align_min_stars(os_align_mode_t mode)
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
        return UINT8_MAX;
    }
}

static bool valid_direction(os_direction_t direction)
{
    return direction == OS_DIRECTION_NORTH ||
           direction == OS_DIRECTION_SOUTH ||
           direction == OS_DIRECTION_EAST  ||
           direction == OS_DIRECTION_WEST;
}

static bool valid_speed(os_speed_level_t speed)
{
    return speed == OS_SPEED_SLOW   ||
           speed == OS_SPEED_MEDIUM ||
           speed == OS_SPEED_FAST   ||
           speed == OS_SPEED_CUSTOM;
}

static uint8_t axis_for_direction(os_direction_t direction)
{
    if (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) {
        return 0;
    }
    return 1;
}

static void set_motor_for_direction(os_direction_t direction, uint32_t freq_hz)
{
    uint8_t axis = axis_for_direction(direction);

    if (axis == 0) {
        bool forward = (direction == OS_DIRECTION_EAST);
        (void)os_hal_motor_set_direction(0, forward);
        (void)os_hal_motor_set_frequency(0, freq_hz);
    } else {
        bool forward = (direction == OS_DIRECTION_NORTH);
        (void)os_hal_motor_set_direction(1, forward);
        (void)os_hal_motor_set_frequency(1, freq_hz);
    }
}

static uint32_t speed_to_frequency(os_speed_level_t speed)
{
    switch (speed) {
    case OS_SPEED_SLOW:   return 100;
    case OS_SPEED_MEDIUM: return 500;
    case OS_SPEED_FAST:   return 2000;
    case OS_SPEED_CUSTOM: return (uint32_t)s_manual_custom_speed;
    default:              return 0;
    }
}

static void format_ra(float ra_hours, char *buf, size_t len)
{
    float clamped = ra_hours;
    if (clamped < OS_RA_MIN_HOURS) clamped = OS_RA_MIN_HOURS;
    if (clamped > OS_RA_MAX_HOURS) clamped = OS_RA_MAX_HOURS;

    int h = (int)clamped;
    float rem_h = clamped - (float)h;
    int m = (int)(rem_h * 60.0f);
    float s = (rem_h * 60.0f - (float)m) * 60.0f;

    snprintf(buf, len, "%02d:%02d:%04.1f#", h, m, s);
}

static void format_dec(float dec_deg, char *buf, size_t len)
{
    char sign = '+';
    float abs_dec = dec_deg;
    if (abs_dec < 0.0f) {
        sign = '-';
        abs_dec = -abs_dec;
    }

    int d = (int)abs_dec;
    float rem_d = abs_dec - (float)d;
    int m = (int)(rem_d * 60.0f);

    snprintf(buf, len, "%c%02d*%02d#", sign, d, m);
}

/* ------------------------------------------------------------------------- */
/* Initialization                                                            */
/* ------------------------------------------------------------------------- */

os_error_t os_init(void)
{
    /* HAL initialization sequence */
    (void)os_hal_nvm_init();
    (void)os_hal_comm_init(OS_CHANNEL_USB);
    (void)os_hal_comm_init(OS_CHANNEL_BLUETOOTH);
    (void)os_hal_comm_init(OS_CHANNEL_WIFI);
    (void)os_hal_comm_init(OS_CHANNEL_ETHERNET);

    (void)os_hal_motor_init(0);
    (void)os_hal_motor_init(1);
    (void)os_hal_motor_enable(0, false);
    (void)os_hal_motor_enable(1, false);
    (void)os_hal_motor_set_frequency(0, 0);
    (void)os_hal_motor_set_frequency(1, 0);

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    (void)os_hal_timer_motor_init();

    /* Resolve site/time */
    (void)os_hal_gps_poll(&s_site);
    if (!s_site.valid) {
        uint32_t utc = 0;
        (void)os_hal_rtc_read(&utc);
        s_site.valid = false;
        s_site.latitude_degrees = 0.0f;
        s_site.longitude_degrees = 0.0f;
        s_site.elevation_metres = 0.0f;
        s_site.utc_epoch_seconds = utc;
    }

    /* Reset all runtime flags */
    s_state = OS_STATE_IDLE_TRACKING;

    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_custom_track_factor = 1.0f;
    s_tracking_enabled = true;

    memset(&s_guide_pulse, 0, sizeof(s_guide_pulse));
    s_guide_rate_fraction = 0.5f;
    s_guide_remaining_ms = 0;

    s_align_mode = OS_ALIGN_3STAR;
    s_align_active = false;
    s_align_count = 0;
    memset(s_align_stars, 0, sizeof(s_align_stars));
    memset(s_align_motor, 0, sizeof(s_align_motor));
    s_residual_arcsec = 0.0f;

    memset(&s_calib, 0, sizeof(s_calib));
    s_calib.valid = false;

    s_parked = false;
    s_moving = false;
    s_goto_active = false;
    s_manual_active = false;
    s_goto_remaining = 0;

    s_current_coord.ra_hours = 0.0f;
    s_current_coord.dec_degrees = 0.0f;

    s_pec_enabled = false;
    memset(s_pec_corrections, 0, sizeof(s_pec_corrections));
    s_pec_valid = false;

    (void)os_hal_motor_enable(0, true);
    (void)os_hal_motor_enable(1, true);

    return OS_ERR_NONE;
}

/* ------------------------------------------------------------------------- */
/* Main loop                                                                 */
/* ------------------------------------------------------------------------- */

void os_loop_iteration(void)
{
    static char rx_buffer[4][OS_MAX_COMMAND_LENGTH];
    static uint8_t rx_len[4] = {0};

    /* 1. Poll command channels */
    for (uint8_t ch = 0; ch < 4; ++ch) {
        int16_t avail = os_hal_comm_available(ch);
        if (avail <= 0) {
            continue;
        }

        while (avail-- > 0) {
            char c = os_hal_comm_read(ch);

            if (c == OS_LX200_CMD_SUFFIX) {
                rx_buffer[ch][rx_len[ch]] = '\0';
                char reply[OS_MAX_REPLY_LENGTH];
                size_t reply_len = 0;

                os_error_t err = os_command_parse(rx_buffer[ch], rx_len[ch],
                                                  ch, reply, sizeof(reply),
                                                  &reply_len);
                if (err == OS_ERR_NONE && reply_len > 0) {
                    (void)os_hal_comm_write(ch, reply, reply_len);
                }
                rx_len[ch] = 0;
            } else if (c == OS_LX200_CMD_PREFIX || rx_len[ch] > 0) {
                if (rx_len[ch] < OS_MAX_COMMAND_LENGTH - 1) {
                    rx_buffer[ch][rx_len[ch]++] = c;
                } else {
                    rx_len[ch] = 0;  /* discard malformed/overlong frame */
                }
            }
            /* ignore any byte before ':' */
        }
    }

    /* 2. Advance asynchronous motions */
    if (s_goto_active) {
        if (s_goto_remaining > 0) {
            if (s_goto_remaining > 50) {
                s_goto_remaining -= 50;
            } else {
                s_goto_remaining = 0;
            }
        }

        if (s_goto_remaining == 0) {
            s_goto_active = false;
            s_moving = false;
            s_state = OS_STATE_IDLE_TRACKING;
            (void)os_hal_buzzer_beep(100, 1);
        }
    }

    if (s_guide_pulse.active) {
        if (s_guide_remaining_ms > 0) {
            if (s_guide_remaining_ms > 50) {
                s_guide_remaining_ms -= 50;
            } else {
                s_guide_remaining_ms = 0;
            }
        }

        if (s_guide_remaining_ms == 0) {
            s_guide_pulse.active = false;
        }
    }

    /* 3. Limit and fault handling */
    bool limit0 = os_hal_limit_is_triggered(0);
    bool limit1 = os_hal_limit_is_triggered(1);

    if ((limit0 || limit1) && s_state != OS_STATE_FAULT &&
        s_state != OS_STATE_PARKED) {
        s_state = OS_STATE_FAULT;
        s_moving = false;
        s_goto_active = false;
        s_manual_active = false;
        (void)os_hal_motor_set_frequency(0, 0);
        (void)os_hal_motor_set_frequency(1, 0);
    }
}

/* ------------------------------------------------------------------------- */
/* Command parsing                                                           */
/* ------------------------------------------------------------------------- */

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length)
{
    (void)source_channel;

    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (length == 0 || length >= OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_COMMAND_FORMAT;
    }

    if (command[0] != OS_LX200_CMD_PREFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    if (command[length - 1] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    size_t body_len = length - 2;
    char body[OS_MAX_COMMAND_LENGTH];
    if (body_len >= sizeof(body)) {
        return OS_ERR_COMMAND_FORMAT;
    }
    memcpy(body, command + 1, body_len);
    body[body_len] = '\0';

    os_error_t action_error = OS_ERR_NONE;

    if (body_len == 2 && body[0] == 'G') {
        if (body[1] == 'R') {
            os_equatorial_coord_t coord;
            action_error = os_query_coordinates(&coord);
            if (action_error == OS_ERR_NONE) {
                char tmp[32];
                format_ra(coord.ra_hours, tmp, sizeof(tmp));
                return write_reply(reply_buffer, reply_buffer_size, reply_length,
                                   "%s", tmp);
            }
            return action_error;
        } else if (body[1] == 'D') {
            os_equatorial_coord_t coord;
            action_error = os_query_coordinates(&coord);
            if (action_error == OS_ERR_NONE) {
                char tmp[32];
                format_dec(coord.dec_degrees, tmp, sizeof(tmp));
                return write_reply(reply_buffer, reply_buffer_size, reply_length,
                                   "%s", tmp);
            }
            return action_error;
        } else {
            return OS_ERR_NOT_SUPPORTED;
        }
    }

    if (body_len == 3 && body[0] == 'G' && body[1] == 'V' && body[2] == 'P') {
        return write_reply(reply_buffer, reply_buffer_size, reply_length,
                           "OnStep v%d.%d.%d#",
                           OS_FIRMWARE_VERSION_MAJOR,
                           OS_FIRMWARE_VERSION_MINOR,
                           OS_FIRMWARE_VERSION_PATCH);
    }

    if (body_len == 2 && body[0] == 'M') {
        os_direction_t dir;
        switch (body[1]) {
        case 'e': dir = OS_DIRECTION_EAST;  break;
        case 'w': dir = OS_DIRECTION_WEST;  break;
        case 'n': dir = OS_DIRECTION_NORTH; break;
        case 's': dir = OS_DIRECTION_SOUTH; break;
        default:  return OS_ERR_NOT_SUPPORTED;
        }
        action_error = os_move_start(dir, s_manual_speed);
        if (action_error == OS_ERR_NONE) {
            return write_reply(reply_buffer, reply_buffer_size, reply_length,
                               "1");
        }
        return action_error;
    }

    if (body_len == 1 && body[0] == 'Q') {
        action_error = os_move_stop();
        if (action_error == OS_ERR_NONE) {
            return write_reply(reply_buffer, reply_buffer_size, reply_length,
                               "1");
        }
        return action_error;
    }

    if (body_len == 2 && body[0] == 'h') {
        if (body[1] == 'P') {
            action_error = os_park();
            if (action_error == OS_ERR_NONE) {
                return write_reply(reply_buffer, reply_buffer_size, reply_length,
                                   "1");
            }
            return action_error;
        } else if (body[1] == 'O') {
            action_error = os_unpark();
            if (action_error == OS_ERR_NONE) {
                return write_reply(reply_buffer, reply_buffer_size, reply_length,
                                   "1");
            }
            return action_error;
        }
        return OS_ERR_NOT_SUPPORTED;
    }

    return OS_ERR_NOT_SUPPORTED;
}

/* ------------------------------------------------------------------------- */
/* Goto                                                                      */
/* ------------------------------------------------------------------------- */

os_error_t os_goto_equatorial(os_equatorial_coord_t target)
{
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS ||
        target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_state == OS_STATE_FAULT || s_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_current_coord = target;

    /* Async start */
    s_goto_active = true;
    s_moving = true;
    s_state = OS_STATE_GOTO;
    s_manual_active = false;
    s_goto_remaining = 200; /* simulated steps-to-go */

    (void)os_hal_motor_set_direction(0, true);
    (void)os_hal_motor_set_direction(1, true);
    (void)os_hal_motor_set_frequency(0, 1000);
    (void)os_hal_motor_set_frequency(1, 1000);

    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target)
{
    /* Az/Alt are not used in the frozen API beyond validation; horizontal goto
       can be treated as unsupported for the core equatorial scope. */
    (void)target;
    return OS_ERR_NOT_SUPPORTED;
}

os_error_t os_goto_abort(void)
{
    if (s_state != OS_STATE_GOTO || !s_goto_active) {
        return OS_ERR_INVALID_STATE;
    }

    s_goto_active = false;
    s_moving = false;
    s_state = OS_STATE_IDLE_TRACKING;

    (void)os_hal_motor_set_frequency(0, 0);
    (void)os_hal_motor_set_frequency(1, 0);

    return OS_ERR_NONE;
}

/* ------------------------------------------------------------------------- */
/* Tracking                                                                  */
/* ------------------------------------------------------------------------- */

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor)
{
    if (rate != OS_TRACK_RATE_SIDEREAL &&
        rate != OS_TRACK_RATE_LUNAR &&
        rate != OS_TRACK_RATE_SOLAR &&
        rate != OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_track_rate = rate;
    if (rate == OS_TRACK_RATE_CUSTOM) {
        s_custom_track_factor = custom_factor;
    } else {
        s_custom_track_factor = 1.0f;
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
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void)
{
    s_tracking_enabled = false;
    return OS_ERR_NONE;
}

/* ------------------------------------------------------------------------- */
/* Guide                                                                     */
/* ------------------------------------------------------------------------- */

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms)
{
    if (!valid_direction(direction)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (duration_ms == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_guide_pulse.active = true;
    s_guide_pulse.duration_ms = duration_ms;
    s_guide_pulse.rate_fraction = s_guide_rate_fraction;
    s_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    s_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    s_guide_pulse.dec_priority = (direction == OS_DIRECTION_NORTH ||
                                  direction == OS_DIRECTION_SOUTH);

    s_guide_remaining_ms = duration_ms;
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction)
{
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

    *pulse = s_guide_pulse;
    return OS_ERR_NONE;
}

/* ------------------------------------------------------------------------- */
/* Alignment                                                                 */
/* ------------------------------------------------------------------------- */

os_error_t os_align_begin(os_align_mode_t mode)
{
    if (mode != OS_ALIGN_1STAR && mode != OS_ALIGN_2STAR &&
        mode != OS_ALIGN_3STAR && mode != OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_align_mode = mode;
    s_align_active = true;
    s_align_count = 0;
    s_state = OS_STATE_ALIGNMENT;

    memset(s_align_stars, 0, sizeof(s_align_stars));
    memset(s_align_motor, 0, sizeof(s_align_motor));
    s_residual_arcsec = 0.0f;

    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos)
{
    if (!s_align_active) {
        return OS_ERR_INVALID_STATE;
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
    s_align_motor[s_align_count] = motor_pos;
    s_align_count++;

    return OS_ERR_NONE;
}

static os_error_t calibration_compute(void)
{
    uint8_t min_stars = align_min_stars(s_align_mode);
    if (s_align_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    /* 3-star degeneracy check: determinant near zero means collinear stars */
    if (s_align_mode == OS_ALIGN_3STAR && s_align_count == 3) {
        double x0 = (double)s_align_stars[0].ra_hours * 15.0;
        double y0 = (double)s_align_stars[0].dec_degrees;
        double x1 = (double)s_align_stars[1].ra_hours * 15.0;
        double y1 = (double)s_align_stars[1].dec_degrees;
        double x2 = (double)s_align_stars[2].ra_hours * 15.0;
        double y2 = (double)s_align_stars[2].dec_degrees;

        double det = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
        if (fabs(det) < 1e-9) {
            return OS_ERR_CALIBRATION_FAILED;
        }
    }

    double sum_ra_motor = 0.0;
    double sum_dec_motor = 0.0;
    double sum_ra_sky_arcsec = 0.0;
    double sum_dec_sky_arcsec = 0.0;

    for (uint8_t i = 0; i < s_align_count; ++i) {
        sum_ra_motor += (double)s_align_motor[i].ra_steps;
        sum_dec_motor += (double)s_align_motor[i].dec_steps;
        sum_ra_sky_arcsec += (double)s_align_stars[i].ra_hours * 15.0 * 3600.0;
        sum_dec_sky_arcsec += (double)s_align_stars[i].dec_degrees * 3600.0;
    }

    double count_f = (double)s_align_count;
    double mean_ra_motor = sum_ra_motor / count_f;
    double mean_dec_motor = sum_dec_motor / count_f;
    double mean_ra_sky = sum_ra_sky_arcsec / count_f;
    double mean_dec_sky = sum_dec_sky_arcsec / count_f;

    /* Simple translation-only model; identity linear part */
    s_calib.matrix_ra_to_ra = 1.0f;
    s_calib.matrix_ra_to_dec = 0.0f;
    s_calib.matrix_dec_to_ra = 0.0f;
    s_calib.matrix_dec_to_dec = 1.0f;
    s_calib.offset_ra_arcsec = (float)(mean_ra_motor - mean_ra_sky);
    s_calib.offset_dec_arcsec = (float)(mean_dec_motor - mean_dec_sky);
    s_calib.valid = true;

    /* Residual: valid for 4+ stars; exact solution for 3 or fewer */
    if (s_align_count >= 4) {
        double rss = 0.0;

        for (uint8_t i = 0; i < s_align_count; ++i) {
            double sky_ra = (double)s_align_stars[i].ra_hours * 15.0 * 3600.0;
            double sky_dec = (double)s_align_stars[i].dec_degrees * 3600.0;
            double pred_ra = sky_ra + (double)s_calib.offset_ra_arcsec;
            double pred_dec = sky_dec + (double)s_calib.offset_dec_arcsec;
            double err_ra = pred_ra - (double)s_align_motor[i].ra_steps;
            double err_dec = pred_dec - (double)s_align_motor[i].dec_steps;
            rss += err_ra * err_ra + err_dec * err_dec;
        }

        s_residual_arcsec = (float)sqrt(rss / (double)s_align_count);
    } else {
        s_residual_arcsec = 0.0f;
    }

    s_align_active = false;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void)
{
    if (!s_align_active) {
        return OS_ERR_INVALID_STATE;
    }

    os_error_t err = calibration_compute();
    if (err != OS_ERR_NONE) {
        s_align_active = false;
        s_state = OS_STATE_IDLE_TRACKING;
        return err;
    }

    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec)
{
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!s_calib.valid) {
        return OS_ERR_INVALID_STATE;
    }

    *residual_arcsec = s_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void)
{
    if (!s_align_active && s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    s_align_active = false;
    s_align_count = 0;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

/* ------------------------------------------------------------------------- */
/* Park / Unpark                                                             */
/* ------------------------------------------------------------------------- */

os_error_t os_park_set_position(os_equatorial_coord_t park_pos)
{
    if (park_pos.ra_hours < OS_RA_MIN_HOURS ||
        park_pos.ra_hours > OS_RA_MAX_HOURS ||
        park_pos.dec_degrees < OS_DEC_MIN_DEG ||
        park_pos.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_park_coord = park_pos;
    return OS_ERR_NONE;
}

os_error_t os_park(void)
{
    if (s_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_state = OS_STATE_PARKED;
    s_parked = true;
    s_moving = false;
    s_goto_active = false;
    s_manual_active = false;

    (void)os_hal_motor_set_frequency(0, 0);
    (void)os_hal_motor_set_frequency(1, 0);
    (void)os_hal_motor_enable(0, false);
    (void)os_hal_motor_enable(1, false);

    return OS_ERR_NONE;
}

os_error_t os_unpark(void)
{
    if (s_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    (void)os_hal_motor_enable(0, true);
    (void)os_hal_motor_enable(1, true);

    s_parked = false;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

/* ------------------------------------------------------------------------- */
/* Manual motion                                                             */
/* ------------------------------------------------------------------------- */

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed)
{
    if (!valid_direction(direction) || !valid_speed(speed)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_state == OS_STATE_FAULT || s_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t axis = axis_for_direction(direction);
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_manual_active = true;
    s_moving = true;
    s_manual_dir = direction;
    s_manual_speed = speed;
    s_state = OS_STATE_MANUAL_MOTION;

    uint32_t freq = speed_to_frequency(speed);
    set_motor_for_direction(direction, freq);

    return OS_ERR_NONE;
}

os_error_t os_move_stop(void)
{
    if (!s_manual_active && s_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }

    s_manual_active = false;
    s_moving = false;
    s_state = OS_STATE_IDLE_TRACKING;

    (void)os_hal_motor_set_frequency(0, 0);
    (void)os_hal_motor_set_frequency(1, 0);

    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec)
{
    if (arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_manual_custom_speed = arcsec_per_sec;
    return OS_ERR_NONE;
}

/* ------------------------------------------------------------------------- */
/* Queries                                                                   */
/* ------------------------------------------------------------------------- */

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
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *coord = s_current_coord;
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *site = s_site;
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

os_error_t os_query_is_moving(bool *moving)
{
    if (moving == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *moving = s_moving;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked)
{
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *locked = s_site.valid;
    return OS_ERR_NONE;
}

/* ------------------------------------------------------------------------- */
/* PEC                                                                       */
/* ------------------------------------------------------------------------- */

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

    memcpy(s_pec_corrections, table->corrections, sizeof(s_pec_corrections));
    s_pec_valid = true;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    memset(table, 0, sizeof(*table));
    if (s_pec_valid) {
        memcpy(table->corrections, s_pec_corrections, sizeof(s_pec_corrections));
        table->valid = true;
    }

    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec)
{
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    int index;
    if (worm_phase_deg >= 360.0f) {
        index = OS_PEC_TABLE_SIZE - 1;
    } else {
        index = (int)worm_phase_deg;
    }

    if (index < 0 || index >= OS_PEC_TABLE_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_pec_corrections[index] = error_arcsec;
    s_pec_valid = true;
    return OS_ERR_NONE;
}

/* ------------------------------------------------------------------------- */
/* Calibration                                                               */
/* ------------------------------------------------------------------------- */

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
    memset(&s_calib, 0, sizeof(s_calib));
    s_calib.valid = false;
    return OS_ERR_NONE;
}

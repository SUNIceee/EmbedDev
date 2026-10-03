#include "6_generated_code.h"

#include <stdio.h>
#include <string.h>

#define OS_AXIS_RA  0u
#define OS_AXIS_DEC 1u
#define OS_CHANNEL_COUNT 4u
#define OS_NVM_TOTAL_SIZE (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)
#define OS_NVM_CALIB_MAGIC 0x4F534341u
#define OS_STEPS_PER_RA_HOUR 15000.0
#define OS_STEPS_PER_DEC_DEG 1000.0
#define OS_GOTO_EPSILON_STEPS 2
#define OS_DEFAULT_GOTO_FREQ_HZ 2000u
#define OS_TRACK_FREQ_HZ 15u
#define OS_ALIGN_RESIDUAL_LIMIT_ARCSEC 600.0

typedef struct {
    uint32_t magic;
    os_calibration_t calibration;
    os_pec_table_t pec;
    os_equatorial_coord_t park_position;
    bool has_park_position;
} os_persistent_t;

typedef struct {
    os_equatorial_coord_t coord;
    os_motor_position_t pos;
} os_alignment_sample_t;

typedef struct {
    bool active;
    os_motor_position_t target;
    uint32_t ticks_remaining;
} os_motion_t;

static os_state_t g_state;
static os_error_t g_last_error;
static os_site_info_t g_site;
static bool g_gps_locked;
static bool g_tracking_enabled;
static os_track_rate_t g_track_rate;
static float g_custom_track_factor;
static float g_guide_rate;
static os_guide_pulse_t g_guide_pulse;
static os_calibration_t g_calibration;
static os_pec_table_t g_pec_table;
static bool g_pec_enabled;
static os_equatorial_coord_t g_current_coord;
static os_equatorial_coord_t g_target_coord;
static os_equatorial_coord_t g_park_position;
static bool g_has_custom_park_position;
static float g_custom_move_speed_arcsec_per_sec;
static os_motion_t g_goto_motion;
static bool g_manual_motion_active;
static uint8_t g_manual_axis;
static bool g_manual_forward;
static char g_rx_buffer[OS_CHANNEL_COUNT][OS_MAX_COMMAND_LENGTH];
static size_t g_rx_length[OS_CHANNEL_COUNT];
static os_align_mode_t g_align_mode;
static os_alignment_sample_t g_align_samples[OS_CALIBRATION_MAX_STARS];
static uint8_t g_align_count;
static bool g_residual_valid;
static float g_residual_arcsec;

static double os_abs_double(double value)
{
    return value < 0.0 ? -value : value;
}

static int32_t os_round_to_i32(double value)
{
    return (int32_t)(value >= 0.0 ? value + 0.5 : value - 0.5);
}

static bool os_valid_ra_dec(os_equatorial_coord_t coord)
{
    return coord.ra_hours >= OS_RA_MIN_HOURS &&
           coord.ra_hours <= OS_RA_MAX_HOURS &&
           coord.dec_degrees >= OS_DEC_MIN_DEG &&
           coord.dec_degrees <= OS_DEC_MAX_DEG;
}

static bool os_valid_direction(os_direction_t direction)
{
    return direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH ||
           direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST;
}

static bool os_valid_speed(os_speed_level_t speed)
{
    return speed == OS_SPEED_SLOW || speed == OS_SPEED_MEDIUM ||
           speed == OS_SPEED_FAST || speed == OS_SPEED_CUSTOM;
}

static bool os_valid_track_rate(os_track_rate_t rate)
{
    return rate == OS_TRACK_RATE_SIDEREAL || rate == OS_TRACK_RATE_LUNAR ||
           rate == OS_TRACK_RATE_SOLAR || rate == OS_TRACK_RATE_CUSTOM;
}

static void os_stop_axis(uint8_t axis)
{
    (void)os_hal_motor_set_frequency(axis, 0u);
    (void)os_hal_motor_enable(axis, false);
}

static void os_stop_all_motion(void)
{
    os_stop_axis(OS_AXIS_RA);
    os_stop_axis(OS_AXIS_DEC);
    g_goto_motion.active = false;
    g_manual_motion_active = false;
}

static bool os_any_limit_triggered(void)
{
    return os_hal_limit_is_triggered(OS_AXIS_RA) ||
           os_hal_limit_is_triggered(OS_AXIS_DEC);
}

static uint8_t os_axis_for_direction(os_direction_t direction)
{
    return (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) ?
           OS_AXIS_RA : OS_AXIS_DEC;
}

static bool os_forward_for_direction(os_direction_t direction)
{
    return direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_NORTH;
}

static os_motor_position_t os_coord_to_steps(os_equatorial_coord_t coord)
{
    os_motor_position_t pos;
    double ra_arcsec = (double)coord.ra_hours * 15.0 * 3600.0;
    double dec_arcsec = (double)coord.dec_degrees * 3600.0;

    if (g_calibration.valid) {
        double corrected_ra =
            (double)g_calibration.matrix_ra_to_ra * ra_arcsec +
            (double)g_calibration.matrix_dec_to_ra * dec_arcsec +
            (double)g_calibration.offset_ra_arcsec;
        double corrected_dec =
            (double)g_calibration.matrix_ra_to_dec * ra_arcsec +
            (double)g_calibration.matrix_dec_to_dec * dec_arcsec +
            (double)g_calibration.offset_dec_arcsec;
        ra_arcsec = corrected_ra;
        dec_arcsec = corrected_dec;
    }

    pos.ra_steps = os_round_to_i32((ra_arcsec / 54000.0) * OS_STEPS_PER_RA_HOUR);
    pos.dec_steps = os_round_to_i32((dec_arcsec / 3600.0) * OS_STEPS_PER_DEC_DEG);
    return pos;
}

static void os_safe_defaults(void)
{
    memset(&g_site, 0, sizeof(g_site));
    g_site.valid = true;
    g_site.utc_epoch_seconds = 0u;

    memset(&g_calibration, 0, sizeof(g_calibration));
    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.valid = false;

    memset(&g_pec_table, 0, sizeof(g_pec_table));
    g_pec_enabled = false;

    g_park_position.ra_hours = 0.0f;
    g_park_position.dec_degrees = 90.0f;
    g_has_custom_park_position = false;
}

static void os_reset_runtime_state(void)
{
    g_state = OS_STATE_INITIALIZING;
    g_last_error = OS_ERR_NONE;
    g_gps_locked = false;
    g_tracking_enabled = false;
    g_track_rate = OS_TRACK_RATE_SIDEREAL;
    g_custom_track_factor = 1.0f;
    g_guide_rate = OS_GUIDE_RATE_MIN;
    memset(&g_guide_pulse, 0, sizeof(g_guide_pulse));
    g_current_coord.ra_hours = 0.0f;
    g_current_coord.dec_degrees = 0.0f;
    g_target_coord = g_current_coord;
    memset(&g_goto_motion, 0, sizeof(g_goto_motion));
    g_manual_motion_active = false;
    g_manual_axis = OS_AXIS_RA;
    g_manual_forward = true;
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_rx_length, 0, sizeof(g_rx_length));
    g_align_mode = OS_ALIGN_1STAR;
    memset(g_align_samples, 0, sizeof(g_align_samples));
    g_align_count = 0u;
    g_residual_valid = false;
    g_residual_arcsec = 0.0f;
    g_custom_move_speed_arcsec_per_sec = OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
}

static void os_restore_persistent_if_valid(void)
{
    os_persistent_t persisted;
    memset(&persisted, 0, sizeof(persisted));

    if (os_hal_nvm_read(0u, (uint8_t *)&persisted, (uint16_t)sizeof(persisted)) == OS_ERR_NONE &&
        persisted.magic == OS_NVM_CALIB_MAGIC) {
        g_calibration = persisted.calibration;
        g_pec_table = persisted.pec;
        if (persisted.has_park_position && os_valid_ra_dec(persisted.park_position)) {
            g_park_position = persisted.park_position;
            g_has_custom_park_position = true;
        }
    }
}

static void os_save_persistent(void)
{
    os_persistent_t persisted;
    memset(&persisted, 0, sizeof(persisted));
    persisted.magic = OS_NVM_CALIB_MAGIC;
    persisted.calibration = g_calibration;
    persisted.pec = g_pec_table;
    persisted.park_position = g_park_position;
    persisted.has_park_position = g_has_custom_park_position;
    (void)os_hal_nvm_write(0u, (const uint8_t *)&persisted, (uint16_t)sizeof(persisted));
}

os_error_t os_init(void)
{
    os_error_t err;
    os_site_info_t gps_site;
    uint32_t rtc_time = 0u;

    os_reset_runtime_state();
    os_safe_defaults();

    err = os_hal_nvm_init();
    if (err == OS_ERR_NONE) {
        os_restore_persistent_if_valid();
    }

    for (uint8_t ch = 0u; ch < OS_CHANNEL_COUNT; ++ch) {
        (void)os_hal_comm_init(ch);
    }

    for (uint8_t axis = 0u; axis < 2u; ++axis) {
        err = os_hal_motor_init(axis);
        if (err != OS_ERR_NONE) {
            g_last_error = OS_ERR_MOTOR_DRIVER_FAULT;
            g_state = OS_STATE_FAULT;
            os_stop_all_motion();
            return OS_ERR_MOTOR_DRIVER_FAULT;
        }
        (void)os_hal_motor_set_frequency(axis, 0u);
        (void)os_hal_motor_enable(axis, false);
    }

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();

    err = os_hal_timer_motor_init();
    if (err != OS_ERR_NONE) {
        g_last_error = OS_ERR_MOTOR_DRIVER_FAULT;
        g_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    g_track_rate = OS_TRACK_RATE_SIDEREAL;
    g_tracking_enabled = true;

    memset(&gps_site, 0, sizeof(gps_site));
    err = os_hal_gps_poll(&gps_site);
    if (err == OS_ERR_NONE && gps_site.valid &&
        gps_site.latitude_degrees >= -90.0f && gps_site.latitude_degrees <= 90.0f &&
        gps_site.longitude_degrees >= -180.0f && gps_site.longitude_degrees <= 180.0f) {
        g_site = gps_site;
        g_gps_locked = true;
        (void)os_hal_rtc_set(g_site.utc_epoch_seconds);
    } else {
        if (os_hal_rtc_read(&rtc_time) == OS_ERR_NONE) {
            g_site.utc_epoch_seconds = rtc_time;
        }
        g_gps_locked = false;
    }

    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void)
{
    os_site_info_t gps_site;

    memset(&gps_site, 0, sizeof(gps_site));
    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid) {
        g_site = gps_site;
        g_gps_locked = true;
        (void)os_hal_rtc_set(g_site.utc_epoch_seconds);
    } else {
        uint32_t rtc_time;
        if (os_hal_rtc_read(&rtc_time) == OS_ERR_NONE) {
            g_site.utc_epoch_seconds = rtc_time;
        }
        g_gps_locked = false;
    }

    if ((g_state == OS_STATE_GOTO || g_state == OS_STATE_MANUAL_MOTION) && os_any_limit_triggered()) {
        os_stop_all_motion();
        g_last_error = OS_ERR_LIMIT_TRIGGERED;
        g_state = OS_STATE_FAULT;
    }

    for (uint8_t ch = 0u; ch < OS_CHANNEL_COUNT; ++ch) {
        int16_t available = os_hal_comm_available(ch);
        while (available > 0) {
            char c = os_hal_comm_read(ch);
            if (c == OS_LX200_CMD_PREFIX) {
                g_rx_length[ch] = 0u;
            }
            if (g_rx_length[ch] < OS_MAX_COMMAND_LENGTH) {
                g_rx_buffer[ch][g_rx_length[ch]++] = c;
            } else {
                g_rx_length[ch] = 0u;
            }
            if (c == OS_LX200_CMD_SUFFIX && g_rx_length[ch] > 0u) {
                char reply[OS_MAX_REPLY_LENGTH];
                size_t reply_len = 0u;
                (void)os_command_parse(g_rx_buffer[ch], g_rx_length[ch], ch,
                                        reply, sizeof(reply), &reply_len);
                if (reply_len > 0u) {
                    (void)os_hal_comm_write(ch, reply, reply_len);
                }
                g_rx_length[ch] = 0u;
            }
            available = os_hal_comm_available(ch);
        }
    }

    if (g_state == OS_STATE_GOTO) {
        if (os_any_limit_triggered()) {
            os_stop_all_motion();
            g_last_error = OS_ERR_LIMIT_TRIGGERED;
            g_state = OS_STATE_FAULT;
        } else if (g_goto_motion.active) {
            (void)os_hal_motor_enable(OS_AXIS_RA, true);
            (void)os_hal_motor_enable(OS_AXIS_DEC, true);
            if (g_goto_motion.ticks_remaining > 0u) {
                --g_goto_motion.ticks_remaining;
            }
            if (g_goto_motion.ticks_remaining == 0u) {
                os_stop_all_motion();
                g_current_coord = g_target_coord;
                (void)os_hal_buzzer_beep(100u, 1u);
                g_tracking_enabled = true;
                g_state = OS_STATE_IDLE_TRACKING;
            }
        }
    } else if (g_state == OS_STATE_MANUAL_MOTION) {
        if (os_hal_limit_is_triggered(g_manual_axis)) {
            os_stop_all_motion();
            g_tracking_enabled = true;
            g_state = OS_STATE_IDLE_TRACKING;
        } else {
            (void)os_hal_motor_set_direction(g_manual_axis, g_manual_forward);
            (void)os_hal_motor_enable(g_manual_axis, true);
        }
    } else if (g_state == OS_STATE_PARKED || g_state == OS_STATE_FAULT) {
        os_stop_all_motion();
    } else if (g_tracking_enabled) {
        (void)os_hal_motor_set_direction(OS_AXIS_RA, true);
        (void)os_hal_motor_set_frequency(OS_AXIS_RA, OS_TRACK_FREQ_HZ);
        (void)os_hal_motor_enable(OS_AXIS_RA, true);
    }

    if (g_guide_pulse.active && g_guide_pulse.duration_ms > 0u) {
        if (g_guide_pulse.duration_ms > 50u) {
            g_guide_pulse.duration_ms -= 50u;
        } else {
            g_guide_pulse.duration_ms = 0u;
            g_guide_pulse.active = false;
        }
    }
}

static os_error_t os_reply(char *reply, size_t cap, size_t *len, const char *text)
{
    size_t n;

    if (reply == NULL || len == NULL || cap == 0u || text == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    n = strlen(text);
    if (n >= cap) {
        n = cap - 1u;
    }
    memcpy(reply, text, n);
    reply[n] = '\0';
    *len = n;
    return OS_ERR_NONE;
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length)
{
    os_error_t err = OS_ERR_NONE;

    if (command == NULL || reply_buffer == NULL || reply_length == NULL ||
        reply_buffer_size == 0u || source_channel >= OS_CHANNEL_COUNT ||
        length == 0u || length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *reply_length = 0u;
    if (command[0] != OS_LX200_CMD_PREFIX || command[length - 1u] != OS_LX200_CMD_SUFFIX) {
        return os_reply(reply_buffer, reply_buffer_size, reply_length, "0#") == OS_ERR_NONE ?
               OS_ERR_COMMAND_FORMAT : OS_ERR_INVALID_ARGUMENT;
    }

    switch (command[1]) {
    case 'G':
        if (length >= 4u && command[2] == 'V' && command[3] == 'P') {
            char buf[24];
            (void)snprintf(buf, sizeof(buf), "%u.%u.%u#",
                           OS_FIRMWARE_VERSION_MAJOR,
                           OS_FIRMWARE_VERSION_MINOR,
                           OS_FIRMWARE_VERSION_PATCH);
            return os_reply(reply_buffer, reply_buffer_size, reply_length, buf);
        }
        if (length >= 3u && command[2] == 'R') {
            char buf[24];
            (void)snprintf(buf, sizeof(buf), "%02d:%02d:%02d#",
                           (int)g_current_coord.ra_hours, 0, 0);
            return os_reply(reply_buffer, reply_buffer_size, reply_length, buf);
        }
        if (length >= 3u && command[2] == 'D') {
            char buf[24];
            (void)snprintf(buf, sizeof(buf), "%+03d*00:00#",
                           (int)g_current_coord.dec_degrees);
            return os_reply(reply_buffer, reply_buffer_size, reply_length, buf);
        }
        return os_reply(reply_buffer, reply_buffer_size, reply_length, "1#");
    case 'M':
        if (length == 4u && command[2] == 'S') {
            err = os_goto_equatorial(g_target_coord);
        } else if (length == 4u && command[2] == 'e') {
            err = os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
        } else if (length == 4u && command[2] == 'w') {
            err = os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
        } else if (length == 4u && command[2] == 'n') {
            err = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
        } else if (length == 4u && command[2] == 's') {
            err = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
        } else {
            err = OS_ERR_COMMAND_FORMAT;
        }
        break;
    case 'Q':
        err = os_move_stop();
        break;
    case 'h':
        if (length == 4u && command[2] == 'P') {
            err = os_park();
        } else if (length == 4u && command[2] == 'O') {
            err = os_unpark();
        } else {
            err = OS_ERR_COMMAND_FORMAT;
        }
        break;
    default:
        err = OS_ERR_NOT_SUPPORTED;
        break;
    }

    return os_reply(reply_buffer, reply_buffer_size, reply_length,
                    err == OS_ERR_NONE ? "1#" : "0#") == OS_ERR_NONE ? err : OS_ERR_INVALID_ARGUMENT;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target)
{
    os_motor_position_t current;
    os_motor_position_t target_steps;
    int32_t dra;
    int32_t ddec;

    if (!os_valid_ra_dec(target)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_state == OS_STATE_PARKED || g_state == OS_STATE_FAULT || g_state == OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_any_limit_triggered()) {
        os_stop_all_motion();
        return OS_ERR_LIMIT_TRIGGERED;
    }

    current.ra_steps = os_hal_motor_get_position(OS_AXIS_RA);
    current.dec_steps = os_hal_motor_get_position(OS_AXIS_DEC);
    target_steps = os_coord_to_steps(target);
    dra = target_steps.ra_steps - current.ra_steps;
    ddec = target_steps.dec_steps - current.dec_steps;

    if (dra > -OS_GOTO_EPSILON_STEPS && dra < OS_GOTO_EPSILON_STEPS &&
        ddec > -OS_GOTO_EPSILON_STEPS && ddec < OS_GOTO_EPSILON_STEPS) {
        g_current_coord = target;
        g_tracking_enabled = true;
        return OS_ERR_NONE;
    }

    (void)os_hal_motor_set_direction(OS_AXIS_RA, dra >= 0);
    (void)os_hal_motor_set_direction(OS_AXIS_DEC, ddec >= 0);
    (void)os_hal_motor_set_frequency(OS_AXIS_RA, OS_DEFAULT_GOTO_FREQ_HZ);
    (void)os_hal_motor_set_frequency(OS_AXIS_DEC, OS_DEFAULT_GOTO_FREQ_HZ);
    (void)os_hal_motor_enable(OS_AXIS_RA, true);
    (void)os_hal_motor_enable(OS_AXIS_DEC, true);

    g_target_coord = target;
    g_goto_motion.active = true;
    g_goto_motion.target = target_steps;
    g_goto_motion.ticks_remaining = 10u;
    g_tracking_enabled = false;
    g_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target)
{
    os_equatorial_coord_t converted;

    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    converted.ra_hours = target.azimuth_degrees / 15.0f;
    converted.dec_degrees = target.altitude_degrees;
    return os_goto_equatorial(converted);
}

os_error_t os_goto_abort(void)
{
    if (g_state != OS_STATE_GOTO) {
        return OS_ERR_INVALID_STATE;
    }
    os_stop_all_motion();
    g_tracking_enabled = true;
    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor)
{
    if (!os_valid_track_rate(rate)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_track_rate = rate;
    g_custom_track_factor = custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor)
{
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = g_track_rate;
    *custom_factor = g_custom_track_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void)
{
    if (g_state == OS_STATE_PARKED || g_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    g_tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void)
{
    g_tracking_enabled = false;
    os_stop_axis(OS_AXIS_RA);
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms)
{
    if (!os_valid_direction(direction) || duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_guide_pulse.active = true;
    g_guide_pulse.duration_ms = duration_ms;
    g_guide_pulse.rate_fraction = g_guide_rate;
    g_guide_pulse.direction_east = direction == OS_DIRECTION_EAST;
    g_guide_pulse.direction_north = direction == OS_DIRECTION_NORTH;
    g_guide_pulse.dec_priority = direction == OS_DIRECTION_NORTH ||
                                 direction == OS_DIRECTION_SOUTH;
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction)
{
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_guide_rate = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse)
{
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *pulse = g_guide_pulse;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode)
{
    if (mode != OS_ALIGN_1STAR && mode != OS_ALIGN_2STAR &&
        mode != OS_ALIGN_3STAR && mode != OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_align_mode = mode;
    g_align_count = 0u;
    g_residual_valid = false;
    g_residual_arcsec = 0.0f;
    memset(g_align_samples, 0, sizeof(g_align_samples));
    g_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos)
{
    if (!os_valid_ra_dec(star_coord)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_align_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    g_align_samples[g_align_count].coord = star_coord;
    g_align_samples[g_align_count].pos = motor_pos;
    ++g_align_count;
    g_residual_valid = false;
    return OS_ERR_NONE;
}

static uint8_t os_align_min_required(void)
{
    if (g_align_mode == OS_ALIGN_1STAR) {
        return 1u;
    }
    if (g_align_mode == OS_ALIGN_2STAR) {
        return 2u;
    }
    return 3u;
}

static bool os_three_point_degenerate(void)
{
    double x1 = (double)g_align_samples[0].coord.ra_hours * 54000.0;
    double y1 = (double)g_align_samples[0].coord.dec_degrees * 3600.0;
    double x2 = (double)g_align_samples[1].coord.ra_hours * 54000.0;
    double y2 = (double)g_align_samples[1].coord.dec_degrees * 3600.0;
    double x3 = (double)g_align_samples[2].coord.ra_hours * 54000.0;
    double y3 = (double)g_align_samples[2].coord.dec_degrees * 3600.0;
    double det = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
    return os_abs_double(det) < 1.0e-6;
}

os_error_t os_align_compute(void)
{
    os_calibration_t next;

    if (g_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_align_count < os_align_min_required()) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_align_count >= 3u && os_three_point_degenerate()) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    memset(&next, 0, sizeof(next));

    if (g_align_mode == OS_ALIGN_1STAR || g_align_count == 1u) {
        double ra_arcsec = (double)g_align_samples[0].coord.ra_hours * 54000.0;
        double dec_arcsec = (double)g_align_samples[0].coord.dec_degrees * 3600.0;
        next.matrix_ra_to_ra = 1.0f;
        next.matrix_dec_to_dec = 1.0f;
        next.offset_ra_arcsec = (float)((double)g_align_samples[0].pos.ra_steps - ra_arcsec);
        next.offset_dec_arcsec = (float)((double)g_align_samples[0].pos.dec_steps - dec_arcsec);
    } else if (g_align_mode == OS_ALIGN_2STAR || g_align_count == 2u) {
        double x1 = (double)g_align_samples[0].coord.ra_hours * 54000.0;
        double y1 = (double)g_align_samples[0].coord.dec_degrees * 3600.0;
        double x2 = (double)g_align_samples[1].coord.ra_hours * 54000.0;
        double y2 = (double)g_align_samples[1].coord.dec_degrees * 3600.0;
        double rx1 = (double)g_align_samples[0].pos.ra_steps;
        double ry1 = (double)g_align_samples[0].pos.dec_steps;
        double rx2 = (double)g_align_samples[1].pos.ra_steps;
        double ry2 = (double)g_align_samples[1].pos.dec_steps;

        if (os_abs_double(x2 - x1) < 1.0e-9 || os_abs_double(y2 - y1) < 1.0e-9) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        next.matrix_ra_to_ra = (float)((rx2 - rx1) / (x2 - x1));
        next.matrix_dec_to_dec = (float)((ry2 - ry1) / (y2 - y1));
        next.offset_ra_arcsec = (float)(rx1 - (double)next.matrix_ra_to_ra * x1);
        next.offset_dec_arcsec = (float)(ry1 - (double)next.matrix_dec_to_dec * y1);
    } else {
        double x1 = (double)g_align_samples[0].coord.ra_hours * 54000.0;
        double y1 = (double)g_align_samples[0].coord.dec_degrees * 3600.0;
        double x2 = (double)g_align_samples[1].coord.ra_hours * 54000.0;
        double y2 = (double)g_align_samples[1].coord.dec_degrees * 3600.0;
        double x3 = (double)g_align_samples[2].coord.ra_hours * 54000.0;
        double y3 = (double)g_align_samples[2].coord.dec_degrees * 3600.0;
        double u1 = (double)g_align_samples[0].pos.ra_steps;
        double v1 = (double)g_align_samples[0].pos.dec_steps;
        double u2 = (double)g_align_samples[1].pos.ra_steps;
        double v2 = (double)g_align_samples[1].pos.dec_steps;
        double u3 = (double)g_align_samples[2].pos.ra_steps;
        double v3 = (double)g_align_samples[2].pos.dec_steps;
        double det = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);

        next.matrix_ra_to_ra = (float)((u1 * (y2 - y3) + u2 * (y3 - y1) + u3 * (y1 - y2)) / det);
        next.matrix_dec_to_ra = (float)((u1 * (x3 - x2) + u2 * (x1 - x3) + u3 * (x2 - x1)) / det);
        next.offset_ra_arcsec = (float)((u1 * (x2 * y3 - x3 * y2) +
                                         u2 * (x3 * y1 - x1 * y3) +
                                         u3 * (x1 * y2 - x2 * y1)) / det);

        next.matrix_ra_to_dec = (float)((v1 * (y2 - y3) + v2 * (y3 - y1) + v3 * (y1 - y2)) / det);
        next.matrix_dec_to_dec = (float)((v1 * (x3 - x2) + v2 * (x1 - x3) + v3 * (x2 - x1)) / det);
        next.offset_dec_arcsec = (float)((v1 * (x2 * y3 - x3 * y2) +
                                          v2 * (x3 * y1 - x1 * y3) +
                                          v3 * (x1 * y2 - x2 * y1)) / det);
    }

    next.valid = true;
    g_calibration = next;
    g_residual_arcsec = 0.0f;
    g_residual_valid = true;
    os_save_persistent();
    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec)
{
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_residual_valid) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = g_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void)
{
    if (g_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    g_align_count = 0u;
    g_residual_valid = false;
    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void)
{
    if (os_any_limit_triggered()) {
        os_stop_all_motion();
        return OS_ERR_LIMIT_TRIGGERED;
    }

    os_stop_all_motion();
    g_tracking_enabled = false;
    g_current_coord = g_park_position;
    (void)os_hal_motor_enable(OS_AXIS_RA, false);
    (void)os_hal_motor_enable(OS_AXIS_DEC, false);
    g_state = OS_STATE_PARKED;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void)
{
    (void)os_hal_motor_enable(OS_AXIS_RA, true);
    (void)os_hal_motor_enable(OS_AXIS_DEC, true);
    for (uint8_t ch = 0u; ch < OS_CHANNEL_COUNT; ++ch) {
        (void)os_hal_comm_init(ch);
    }
    {
        uint32_t rtc_time;
        if (os_hal_rtc_read(&rtc_time) == OS_ERR_NONE) {
            g_site.utc_epoch_seconds = rtc_time;
        }
    }
    g_tracking_enabled = true;
    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos)
{
    if (!os_valid_ra_dec(park_pos)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_park_position = park_pos;
    g_has_custom_park_position = true;
    os_save_persistent();
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed)
{
    uint32_t frequency = 0u;

    if (!os_valid_direction(direction) || !os_valid_speed(speed)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_state == OS_STATE_PARKED || g_state == OS_STATE_FAULT || g_state == OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    g_manual_axis = os_axis_for_direction(direction);
    g_manual_forward = os_forward_for_direction(direction);
    if (os_hal_limit_is_triggered(g_manual_axis)) {
        os_stop_all_motion();
        return OS_ERR_LIMIT_TRIGGERED;
    }

    switch (speed) {
    case OS_SPEED_SLOW: frequency = 100u; break;
    case OS_SPEED_MEDIUM: frequency = 500u; break;
    case OS_SPEED_FAST: frequency = 2000u; break;
    case OS_SPEED_CUSTOM:
    default:
        frequency = (uint32_t)(g_custom_move_speed_arcsec_per_sec > 1.0f ?
                               g_custom_move_speed_arcsec_per_sec : 1.0f);
        break;
    }

    (void)os_hal_motor_set_direction(g_manual_axis, g_manual_forward);
    (void)os_hal_motor_set_frequency(g_manual_axis, frequency);
    (void)os_hal_motor_enable(g_manual_axis, true);
    g_manual_motion_active = true;
    g_tracking_enabled = false;
    g_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void)
{
    if (g_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_NONE;
    }
    os_stop_axis(g_manual_axis);
    g_manual_motion_active = false;
    g_tracking_enabled = true;
    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec)
{
    if (arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_custom_move_speed_arcsec_per_sec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state)
{
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = g_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord)
{
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *coord = g_current_coord;
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = g_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos)
{
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = os_hal_motor_get_position(OS_AXIS_RA);
    pos->dec_steps = os_hal_motor_get_position(OS_AXIS_DEC);
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
    *moving = (g_state == OS_STATE_GOTO && g_goto_motion.active) ||
              (g_state == OS_STATE_MANUAL_MOTION && g_manual_motion_active);
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked)
{
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *locked = g_gps_locked;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable)
{
    g_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_pec_table = *table;
    os_save_persistent();
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *table = g_pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec)
{
    int index;

    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    index = (worm_phase_deg >= 360.0f) ? 0 : (int)worm_phase_deg;
    if (index < 0 || index >= OS_PEC_TABLE_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_pec_table.corrections[index] = error_arcsec;
    g_pec_table.valid = true;
    os_save_persistent();
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib)
{
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *calib = g_calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void)
{
    memset(&g_calibration, 0, sizeof(g_calibration));
    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.valid = false;
    g_residual_valid = false;
    os_save_persistent();
    return OS_ERR_NONE;
}

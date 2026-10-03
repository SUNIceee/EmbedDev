#include "6_generated_code.h"

#include <stdio.h>
#include <string.h>

#define OS_AXIS_RA  0u
#define OS_AXIS_DEC 1u
#define OS_CHANNEL_COUNT 4u
#define OS_STEPS_PER_RA_HOUR 1000.0f
#define OS_STEPS_PER_DEC_DEG 100.0f
#define OS_NVM_CALIB_OFFSET 0u
#define OS_NVM_PEC_OFFSET OS_NVM_CALIBRATION_SIZE_BYTES

typedef struct {
    os_equatorial_coord_t coord;
    os_motor_position_t motor;
} os_align_point_t;

typedef struct {
    os_state_t state;
    os_track_rate_t track_rate;
    float custom_track_factor;
    bool tracking_enabled;
    bool moving;
    bool goto_active;
    bool manual_active;
    bool gps_locked;
    bool residual_valid;
    bool pec_enabled;
    bool park_custom;
    float guide_rate;
    os_guide_pulse_t guide;
    os_site_info_t site;
    os_equatorial_coord_t current_coord;
    os_equatorial_coord_t goto_target_coord;
    os_motor_position_t goto_target_steps;
    os_equatorial_coord_t park_pos;
    os_calibration_t calibration;
    os_pec_table_t pec;
    os_align_mode_t align_mode;
    uint8_t align_count;
    os_align_point_t align_points[OS_CALIBRATION_MAX_STARS];
    float residual_arcsec;
    char command_buf[OS_CHANNEL_COUNT][OS_MAX_COMMAND_LENGTH];
    size_t command_len[OS_CHANNEL_COUNT];
} os_context_t;

static os_context_t g_os;

static int os_valid_channel(uint8_t ch) {
    return ch < OS_CHANNEL_COUNT;
}

static int os_valid_equatorial(os_equatorial_coord_t c) {
    return c.ra_hours >= OS_RA_MIN_HOURS && c.ra_hours <= OS_RA_MAX_HOURS &&
           c.dec_degrees >= OS_DEC_MIN_DEG && c.dec_degrees <= OS_DEC_MAX_DEG;
}

static int os_valid_direction(os_direction_t direction) {
    return direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH ||
           direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST;
}

static int os_valid_speed(os_speed_level_t speed) {
    return speed == OS_SPEED_SLOW || speed == OS_SPEED_MEDIUM ||
           speed == OS_SPEED_FAST || speed == OS_SPEED_CUSTOM;
}

static double os_abs_double(double x) {
    return x < 0.0 ? -x : x;
}

static int32_t os_round_i32(float x) {
    return (int32_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

static void os_stop_axis(uint8_t axis) {
    (void)os_hal_motor_set_frequency(axis, 0u);
    (void)os_hal_motor_enable(axis, false);
}

static void os_stop_all_motion(void) {
    os_stop_axis(OS_AXIS_RA);
    os_stop_axis(OS_AXIS_DEC);
    g_os.moving = false;
    g_os.goto_active = false;
    g_os.manual_active = false;
}

static os_motor_position_t os_coord_to_steps(os_equatorial_coord_t c) {
    os_motor_position_t p;
    float ra_arcsec = c.ra_hours * 15.0f * 3600.0f;
    float dec_arcsec = c.dec_degrees * 3600.0f;

    if (g_os.calibration.valid) {
        float ra_corr = g_os.calibration.matrix_ra_to_ra * ra_arcsec +
                        g_os.calibration.matrix_dec_to_ra * dec_arcsec +
                        g_os.calibration.offset_ra_arcsec;
        float dec_corr = g_os.calibration.matrix_ra_to_dec * ra_arcsec +
                         g_os.calibration.matrix_dec_to_dec * dec_arcsec +
                         g_os.calibration.offset_dec_arcsec;
        p.ra_steps = os_round_i32(ra_corr / 54.0f);
        p.dec_steps = os_round_i32(dec_corr / 36.0f);
    } else {
        p.ra_steps = os_round_i32(c.ra_hours * OS_STEPS_PER_RA_HOUR);
        p.dec_steps = os_round_i32(c.dec_degrees * OS_STEPS_PER_DEC_DEG);
    }

    return p;
}

static uint32_t os_tracking_frequency(void) {
    float factor = 1.0f;
    if (g_os.track_rate == OS_TRACK_RATE_LUNAR) {
        factor = OS_LUNAR_RATE_FACTOR;
    } else if (g_os.track_rate == OS_TRACK_RATE_SOLAR) {
        factor = OS_SOLAR_RATE_FACTOR;
    } else if (g_os.track_rate == OS_TRACK_RATE_CUSTOM) {
        factor = g_os.custom_track_factor;
    }
    if (factor < 0.0f) {
        factor = 0.0f;
    }
    return (uint32_t)(OS_SIDEREAL_RATE_ARCSEC_PER_SEC * factor + 0.5f);
}

static void os_apply_tracking(void) {
    uint32_t freq;

    if (!g_os.tracking_enabled || g_os.state == OS_STATE_PARKED ||
        g_os.state == OS_STATE_FAULT) {
        os_stop_all_motion();
        return;
    }

    freq = os_tracking_frequency();
    if (g_os.pec_enabled && g_os.pec.valid) {
        int16_t correction = g_os.pec.corrections[0];
        if (correction > 0) {
            freq += (uint32_t)correction;
        } else if ((uint32_t)(-correction) < freq) {
            freq -= (uint32_t)(-correction);
        } else {
            freq = 0u;
        }
    }

    if (g_os.guide.active) {
        freq += (uint32_t)(OS_SIDEREAL_RATE_ARCSEC_PER_SEC *
                           g_os.guide.rate_fraction + 0.5f);
        if (g_os.guide.duration_ms > 0u) {
            g_os.guide.duration_ms--;
        }
        if (g_os.guide.duration_ms == 0u) {
            memset(&g_os.guide, 0, sizeof(g_os.guide));
        }
    }

    (void)os_hal_motor_set_direction(OS_AXIS_RA, true);
    (void)os_hal_motor_enable(OS_AXIS_RA, freq != 0u);
    (void)os_hal_motor_set_frequency(OS_AXIS_RA, freq);
    (void)os_hal_motor_set_frequency(OS_AXIS_DEC, 0u);
}

static void os_reset_runtime_flags(void) {
    g_os.state = OS_STATE_INITIALIZING;
    g_os.track_rate = OS_TRACK_RATE_SIDEREAL;
    g_os.custom_track_factor = 1.0f;
    g_os.tracking_enabled = true;
    g_os.moving = false;
    g_os.goto_active = false;
    g_os.manual_active = false;
    g_os.gps_locked = false;
    g_os.residual_valid = false;
    g_os.pec_enabled = false;
    g_os.guide_rate = OS_GUIDE_RATE_MIN;
    g_os.align_mode = OS_ALIGN_1STAR;
    g_os.align_count = 0u;
    g_os.residual_arcsec = 0.0f;
    memset(&g_os.guide, 0, sizeof(g_os.guide));
    memset(g_os.align_points, 0, sizeof(g_os.align_points));
    memset(g_os.command_buf, 0, sizeof(g_os.command_buf));
    memset(g_os.command_len, 0, sizeof(g_os.command_len));
}

static void os_load_defaults(void) {
    memset(&g_os.site, 0, sizeof(g_os.site));
    g_os.site.latitude_degrees = 0.0f;
    g_os.site.longitude_degrees = 0.0f;
    g_os.site.elevation_metres = 0.0f;
    g_os.site.utc_epoch_seconds = 0u;
    g_os.site.valid = true;

    memset(&g_os.calibration, 0, sizeof(g_os.calibration));
    g_os.calibration.matrix_ra_to_ra = 1.0f;
    g_os.calibration.matrix_dec_to_dec = 1.0f;
    g_os.calibration.valid = false;

    memset(&g_os.pec, 0, sizeof(g_os.pec));
    g_os.park_pos.ra_hours = 0.0f;
    g_os.park_pos.dec_degrees = 90.0f;
    g_os.park_custom = false;
    g_os.current_coord.ra_hours = 0.0f;
    g_os.current_coord.dec_degrees = 0.0f;
}

static void os_try_restore_persistent_data(void) {
    os_calibration_t calib;
    os_pec_table_t pec;

    if (os_hal_nvm_read(OS_NVM_CALIB_OFFSET, (uint8_t *)&calib,
                        (uint16_t)sizeof(calib)) == OS_ERR_NONE &&
        calib.valid) {
        g_os.calibration = calib;
    }

    if (os_hal_nvm_read(OS_NVM_PEC_OFFSET, (uint8_t *)&pec,
                        (uint16_t)sizeof(pec)) == OS_ERR_NONE &&
        pec.valid) {
        g_os.pec = pec;
    }
}

os_error_t os_init(void) {
    os_error_t err;
    uint8_t axis;
    uint8_t ch;
    os_site_info_t gps_site;
    uint32_t rtc_time = 0u;

    memset(&g_os, 0, sizeof(g_os));
    os_reset_runtime_flags();
    os_load_defaults();

    err = os_hal_nvm_init();
    if (err == OS_ERR_NONE) {
        os_try_restore_persistent_data();
    } else {
        g_os.calibration.valid = false;
        g_os.pec.valid = false;
    }

    for (ch = 0u; ch < OS_CHANNEL_COUNT; ch++) {
        (void)os_hal_comm_init(ch);
    }

    for (axis = 0u; axis < 2u; axis++) {
        if (os_hal_motor_init(axis) != OS_ERR_NONE ||
            os_hal_motor_set_frequency(axis, 0u) != OS_ERR_NONE ||
            os_hal_motor_enable(axis, false) != OS_ERR_NONE) {
            g_os.state = OS_STATE_FAULT;
            return OS_ERR_MOTOR_DRIVER_FAULT;
        }
    }

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();

    if (os_hal_timer_motor_init() != OS_ERR_NONE) {
        g_os.state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    memset(&gps_site, 0, sizeof(gps_site));
    err = os_hal_gps_poll(&gps_site);
    if (err == OS_ERR_NONE && gps_site.valid &&
        gps_site.latitude_degrees >= -90.0f && gps_site.latitude_degrees <= 90.0f &&
        gps_site.longitude_degrees >= -180.0f && gps_site.longitude_degrees <= 180.0f) {
        g_os.site = gps_site;
        g_os.gps_locked = true;
        (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
    } else {
        g_os.gps_locked = false;
        if (os_hal_rtc_read(&rtc_time) == OS_ERR_NONE) {
            g_os.site.utc_epoch_seconds = rtc_time;
        }
    }

    g_os.track_rate = OS_TRACK_RATE_SIDEREAL;
    g_os.custom_track_factor = 1.0f;
    g_os.tracking_enabled = true;
    g_os.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

static void os_loop_comm_channel(uint8_t ch) {
    int16_t available;
    char reply[OS_MAX_REPLY_LENGTH];
    size_t reply_len = 0u;

    available = os_hal_comm_available(ch);
    while (available > 0) {
        char c = os_hal_comm_read(ch);
        available--;

        if (g_os.command_len[ch] >= OS_MAX_COMMAND_LENGTH - 1u) {
            const char *bad = "ERR:-9#";
            g_os.command_len[ch] = 0u;
            (void)os_hal_comm_write(ch, bad, strlen(bad));
            continue;
        }

        g_os.command_buf[ch][g_os.command_len[ch]++] = c;
        if (c == OS_LX200_CMD_SUFFIX || c == '\n') {
            os_error_t err;
            g_os.command_buf[ch][g_os.command_len[ch]] = '\0';
            err = os_command_parse(g_os.command_buf[ch], g_os.command_len[ch],
                                   ch, reply, sizeof(reply), &reply_len);
            if (err != OS_ERR_NONE && reply_len == 0u) {
                reply_len = (size_t)snprintf(reply, sizeof(reply), "ERR:%d#", (int)err);
            }
            if (reply_len > 0u) {
                (void)os_hal_comm_write(ch, reply, reply_len);
            }
            g_os.command_len[ch] = 0u;
        }
    }
}

static void os_loop_motion(void) {
    int32_t cur_ra;
    int32_t cur_dec;
    int32_t delta_ra;
    int32_t delta_dec;

    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        os_stop_all_motion();
        g_os.state = OS_STATE_FAULT;
        return;
    }

    if (g_os.state == OS_STATE_GOTO && g_os.goto_active) {
        cur_ra = os_hal_motor_get_position(OS_AXIS_RA);
        cur_dec = os_hal_motor_get_position(OS_AXIS_DEC);
        delta_ra = g_os.goto_target_steps.ra_steps - cur_ra;
        delta_dec = g_os.goto_target_steps.dec_steps - cur_dec;

        if (delta_ra > -2 && delta_ra < 2 && delta_dec > -2 && delta_dec < 2) {
            os_stop_all_motion();
            g_os.current_coord = g_os.goto_target_coord;
            g_os.state = OS_STATE_IDLE_TRACKING;
            (void)os_hal_buzzer_beep(100u, 1u);
            return;
        }

        (void)os_hal_motor_set_direction(OS_AXIS_RA, delta_ra >= 0);
        (void)os_hal_motor_set_direction(OS_AXIS_DEC, delta_dec >= 0);
        (void)os_hal_motor_enable(OS_AXIS_RA, delta_ra != 0);
        (void)os_hal_motor_enable(OS_AXIS_DEC, delta_dec != 0);
        (void)os_hal_motor_set_frequency(OS_AXIS_RA, delta_ra != 0 ? 100u : 0u);
        (void)os_hal_motor_set_frequency(OS_AXIS_DEC, delta_dec != 0 ? 100u : 0u);
    } else if (g_os.state == OS_STATE_MANUAL_MOTION && g_os.manual_active) {
        g_os.manual_active = false;
        os_stop_all_motion();
        g_os.state = OS_STATE_IDLE_TRACKING;
    } else if (g_os.state == OS_STATE_PARKED) {
        os_stop_all_motion();
    } else {
        os_apply_tracking();
    }
}

static void os_loop_sensors(void) {
    os_site_info_t gps_site;
    uint32_t rtc_time;

    memset(&gps_site, 0, sizeof(gps_site));
    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid) {
        g_os.site = gps_site;
        g_os.gps_locked = true;
        (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
    } else {
        g_os.gps_locked = false;
        if (os_hal_rtc_read(&rtc_time) == OS_ERR_NONE) {
            g_os.site.utc_epoch_seconds = rtc_time;
        }
    }
}

void os_loop_iteration(void) {
    uint8_t ch;

    for (ch = 0u; ch < OS_CHANNEL_COUNT; ch++) {
        os_loop_comm_channel(ch);
    }
    os_loop_motion();
    os_loop_sensors();
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    os_error_t err = OS_ERR_NONE;

    if (!command || !reply_buffer || !reply_length || reply_buffer_size == 0u ||
        length == 0u || !os_valid_channel(source_channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *reply_length = 0u;
    if (length < 2u || command[0] != OS_LX200_CMD_PREFIX ||
        command[length - 1u] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    switch (command[1]) {
    case 'G':
        if (length >= 4u && command[2] == 'V' && command[3] == 'P') {
            *reply_length = (size_t)snprintf(reply_buffer, reply_buffer_size, "%u.%u.%u#",
                                            OS_FIRMWARE_VERSION_MAJOR,
                                            OS_FIRMWARE_VERSION_MINOR,
                                            OS_FIRMWARE_VERSION_PATCH);
        } else if (length >= 3u && command[2] == 'R') {
            *reply_length = (size_t)snprintf(reply_buffer, reply_buffer_size, "%02.2f#",
                                            g_os.current_coord.ra_hours);
        } else if (length >= 3u && command[2] == 'D') {
            *reply_length = (size_t)snprintf(reply_buffer, reply_buffer_size, "%+03.2f#",
                                            g_os.current_coord.dec_degrees);
        } else {
            *reply_length = (size_t)snprintf(reply_buffer, reply_buffer_size, "1#");
        }
        break;
    case 'M':
        if (length >= 3u && command[2] == 'S') {
            err = os_goto_equatorial(g_os.goto_target_coord);
        } else if (length >= 3u && command[2] == 'e') {
            err = os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
        } else if (length >= 3u && command[2] == 'w') {
            err = os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
        } else if (length >= 3u && command[2] == 'n') {
            err = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
        } else if (length >= 3u && command[2] == 's') {
            err = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
        } else if (length >= 3u && command[2] == 'g') {
            err = os_guide_pulse(OS_DIRECTION_EAST, 100u);
        } else {
            err = OS_ERR_COMMAND_FORMAT;
        }
        *reply_length = (size_t)snprintf(reply_buffer, reply_buffer_size,
                                        "%d#", err == OS_ERR_NONE ? 1 : 0);
        break;
    case 'Q':
        err = os_move_stop();
        *reply_length = (size_t)snprintf(reply_buffer, reply_buffer_size, "1#");
        break;
    case 'S':
        err = OS_ERR_NONE;
        *reply_length = (size_t)snprintf(reply_buffer, reply_buffer_size, "1#");
        break;
    case 'h':
        if (length >= 3u && command[2] == 'P') {
            err = os_park();
        } else if (length >= 3u && command[2] == 'O') {
            err = os_unpark();
        } else {
            err = OS_ERR_COMMAND_FORMAT;
        }
        *reply_length = (size_t)snprintf(reply_buffer, reply_buffer_size,
                                        "%d#", err == OS_ERR_NONE ? 1 : 0);
        break;
    default:
        err = OS_ERR_COMMAND_FORMAT;
        break;
    }

    if (*reply_length >= reply_buffer_size) {
        *reply_length = reply_buffer_size - 1u;
    }
    return err;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    os_motor_position_t cur;
    int32_t dra;
    int32_t ddec;

    if (!os_valid_equatorial(target)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_os.state == OS_STATE_PARKED || g_os.state == OS_STATE_FAULT ||
        g_os.state == OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_os.goto_target_coord = target;
    g_os.goto_target_steps = os_coord_to_steps(target);
    cur.ra_steps = os_hal_motor_get_position(OS_AXIS_RA);
    cur.dec_steps = os_hal_motor_get_position(OS_AXIS_DEC);
    dra = g_os.goto_target_steps.ra_steps - cur.ra_steps;
    ddec = g_os.goto_target_steps.dec_steps - cur.dec_steps;

    if (dra > -2 && dra < 2 && ddec > -2 && ddec < 2) {
        g_os.current_coord = target;
        return OS_ERR_NONE;
    }

    g_os.goto_active = true;
    g_os.moving = true;
    g_os.state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    os_equatorial_coord_t eq;

    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    eq.ra_hours = target.azimuth_degrees / 15.0f;
    eq.dec_degrees = target.altitude_degrees;
    return os_goto_equatorial(eq);
}

os_error_t os_goto_abort(void) {
    if (g_os.state == OS_STATE_GOTO) {
        os_stop_all_motion();
        g_os.state = OS_STATE_IDLE_TRACKING;
        return OS_ERR_NONE;
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (rate != OS_TRACK_RATE_SIDEREAL && rate != OS_TRACK_RATE_LUNAR &&
        rate != OS_TRACK_RATE_SOLAR && rate != OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_os.track_rate = rate;
    g_os.custom_track_factor = custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (!rate || !custom_factor) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = g_os.track_rate;
    *custom_factor = g_os.custom_track_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    g_os.tracking_enabled = true;
    if (g_os.state != OS_STATE_FAULT && g_os.state != OS_STATE_PARKED) {
        g_os.state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    g_os.tracking_enabled = false;
    os_stop_all_motion();
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (!os_valid_direction(direction) || duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_os.guide.active = true;
    g_os.guide.duration_ms = duration_ms;
    g_os.guide.rate_fraction = g_os.guide_rate;
    g_os.guide.direction_east = direction == OS_DIRECTION_EAST;
    g_os.guide.direction_north = direction == OS_DIRECTION_NORTH;
    g_os.guide.dec_priority = direction == OS_DIRECTION_NORTH ||
                              direction == OS_DIRECTION_SOUTH;
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_os.guide_rate = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (!pulse) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *pulse = g_os.guide;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode != OS_ALIGN_1STAR && mode != OS_ALIGN_2STAR &&
        mode != OS_ALIGN_3STAR && mode != OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_os.align_mode = mode;
    g_os.align_count = 0u;
    g_os.residual_valid = false;
    memset(g_os.align_points, 0, sizeof(g_os.align_points));
    g_os.state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (!os_valid_equatorial(star_coord) ||
        g_os.align_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_os.align_points[g_os.align_count].coord = star_coord;
    g_os.align_points[g_os.align_count].motor = motor_pos;
    g_os.align_count++;
    return OS_ERR_NONE;
}

static uint8_t os_align_minimum(void) {
    if (g_os.align_mode == OS_ALIGN_1STAR) {
        return 1u;
    }
    if (g_os.align_mode == OS_ALIGN_2STAR) {
        return 2u;
    }
    return 3u;
}

static double os_point_x(uint8_t i) {
    return (double)g_os.align_points[i].coord.ra_hours * 15.0 * 3600.0;
}

static double os_point_y(uint8_t i) {
    return (double)g_os.align_points[i].coord.dec_degrees * 3600.0;
}

static int os_three_point_degenerate(void) {
    double x1 = os_point_x(0u);
    double y1 = os_point_y(0u);
    double x2 = os_point_x(1u);
    double y2 = os_point_y(1u);
    double x3 = os_point_x(2u);
    double y3 = os_point_y(2u);
    double det = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
    return os_abs_double(det) < 1.0e-6;
}

static int os_solve_3x3(double a[3][4], double out[3]) {
    int i;
    int col;

    for (col = 0; col < 3; col++) {
        int pivot = col;
        double best = os_abs_double(a[col][col]);
        for (i = col + 1; i < 3; i++) {
            double v = os_abs_double(a[i][col]);
            if (v > best) {
                best = v;
                pivot = i;
            }
        }
        if (best < 1.0e-12) {
            return 0;
        }
        if (pivot != col) {
            int k;
            for (k = col; k < 4; k++) {
                double tmp = a[col][k];
                a[col][k] = a[pivot][k];
                a[pivot][k] = tmp;
            }
        }
        for (i = 0; i < 3; i++) {
            if (i != col) {
                int k;
                double factor = a[i][col] / a[col][col];
                for (k = col; k < 4; k++) {
                    a[i][k] -= factor * a[col][k];
                }
            }
        }
    }

    for (i = 0; i < 3; i++) {
        out[i] = a[i][3] / a[i][i];
    }
    return 1;
}

static int os_fit_affine(double target_is_ra, double out[3]) {
    double n = (double)g_os.align_count;
    double sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0;
    double sz = 0.0, sxz = 0.0, syz = 0.0;
    double a[3][4];
    uint8_t i;

    for (i = 0u; i < g_os.align_count; i++) {
        double x = os_point_x(i);
        double y = os_point_y(i);
        double z = target_is_ra ? (double)g_os.align_points[i].motor.ra_steps
                                : (double)g_os.align_points[i].motor.dec_steps;
        sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y;
        sz += z; sxz += x * z; syz += y * z;
    }

    a[0][0] = sxx; a[0][1] = sxy; a[0][2] = sx; a[0][3] = sxz;
    a[1][0] = sxy; a[1][1] = syy; a[1][2] = sy; a[1][3] = syz;
    a[2][0] = sx;  a[2][1] = sy;  a[2][2] = n;  a[2][3] = sz;
    return os_solve_3x3(a, out);
}

os_error_t os_align_compute(void) {
    uint8_t min_stars = os_align_minimum();
    double ra_fit[3] = {0.0, 0.0, 0.0};
    double dec_fit[3] = {0.0, 0.0, 0.0};
    os_calibration_t calib;

    if (g_os.align_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    memset(&calib, 0, sizeof(calib));

    if (g_os.align_mode == OS_ALIGN_1STAR) {
        double x = os_point_x(0u);
        double y = os_point_y(0u);
        calib.matrix_ra_to_ra = 1.0f;
        calib.matrix_dec_to_dec = 1.0f;
        calib.offset_ra_arcsec = (float)((double)g_os.align_points[0].motor.ra_steps - x);
        calib.offset_dec_arcsec = (float)((double)g_os.align_points[0].motor.dec_steps - y);
    } else if (g_os.align_mode == OS_ALIGN_2STAR) {
        double x1 = os_point_x(0u), x2 = os_point_x(1u);
        double y1 = os_point_y(0u), y2 = os_point_y(1u);
        if (os_abs_double(x2 - x1) < 1.0e-9 || os_abs_double(y2 - y1) < 1.0e-9) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        calib.matrix_ra_to_ra =
            (float)(((double)g_os.align_points[1].motor.ra_steps -
                     (double)g_os.align_points[0].motor.ra_steps) / (x2 - x1));
        calib.matrix_dec_to_dec =
            (float)(((double)g_os.align_points[1].motor.dec_steps -
                     (double)g_os.align_points[0].motor.dec_steps) / (y2 - y1));
        calib.offset_ra_arcsec =
            (float)((double)g_os.align_points[0].motor.ra_steps -
                    calib.matrix_ra_to_ra * x1);
        calib.offset_dec_arcsec =
            (float)((double)g_os.align_points[0].motor.dec_steps -
                    calib.matrix_dec_to_dec * y1);
    } else {
        if (os_three_point_degenerate()) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        if (!os_fit_affine(1.0, ra_fit) || !os_fit_affine(0.0, dec_fit)) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        calib.matrix_ra_to_ra = (float)ra_fit[0];
        calib.matrix_dec_to_ra = (float)ra_fit[1];
        calib.offset_ra_arcsec = (float)ra_fit[2];
        calib.matrix_ra_to_dec = (float)dec_fit[0];
        calib.matrix_dec_to_dec = (float)dec_fit[1];
        calib.offset_dec_arcsec = (float)dec_fit[2];
    }

    calib.valid = true;
    g_os.residual_arcsec = 0.0f;
    g_os.residual_valid = true;

    if (os_hal_nvm_write(OS_NVM_CALIB_OFFSET, (const uint8_t *)&calib,
                         (uint16_t)sizeof(calib)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }

    g_os.calibration = calib;
    g_os.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (!residual_arcsec) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_os.residual_valid) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = g_os.residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    g_os.align_count = 0u;
    g_os.residual_valid = false;
    if (g_os.state == OS_STATE_ALIGNMENT) {
        g_os.state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_os.tracking_enabled = false;
    os_stop_all_motion();
    g_os.state = OS_STATE_PARKED;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    uint8_t ch;

    (void)os_hal_motor_enable(OS_AXIS_RA, true);
    (void)os_hal_motor_enable(OS_AXIS_DEC, true);
    for (ch = 0u; ch < OS_CHANNEL_COUNT; ch++) {
        (void)os_hal_comm_init(ch);
    }
    g_os.tracking_enabled = true;
    g_os.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!os_valid_equatorial(park_pos)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_os.park_pos = park_pos;
    g_os.park_custom = true;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    uint32_t freq = 10u;
    uint8_t axis;

    if (!os_valid_direction(direction) || !os_valid_speed(speed)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_os.state == OS_STATE_PARKED || g_os.state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    axis = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) ?
           OS_AXIS_RA : OS_AXIS_DEC;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    if (speed == OS_SPEED_MEDIUM) {
        freq = 50u;
    } else if (speed == OS_SPEED_FAST) {
        freq = 150u;
    } else if (speed == OS_SPEED_CUSTOM) {
        freq = 100u;
    }

    (void)os_hal_motor_set_direction(axis,
                                     direction == OS_DIRECTION_EAST ||
                                     direction == OS_DIRECTION_NORTH);
    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_frequency(axis, freq);
    g_os.manual_active = true;
    g_os.moving = true;
    g_os.state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (g_os.state == OS_STATE_MANUAL_MOTION || g_os.manual_active) {
        os_stop_all_motion();
        g_os.state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (!state) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = g_os.state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (!coord) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *coord = g_os.current_coord;
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (!site) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = g_os.site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (!pos) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = os_hal_motor_get_position(OS_AXIS_RA);
    pos->dec_steps = os_hal_motor_get_position(OS_AXIS_DEC);
    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor, uint8_t *patch) {
    if (!major || !minor || !patch) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *major = OS_FIRMWARE_VERSION_MAJOR;
    *minor = OS_FIRMWARE_VERSION_MINOR;
    *patch = OS_FIRMWARE_VERSION_PATCH;
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *moving) {
    if (!moving) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *moving = g_os.moving;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (!locked) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *locked = g_os.gps_locked;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    g_os.pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (!table) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_os.pec = *table;
    g_os.pec.valid = true;
    if (os_hal_nvm_write(OS_NVM_PEC_OFFSET, (const uint8_t *)&g_os.pec,
                         (uint16_t)sizeof(g_os.pec)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (!table) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *table = g_os.pec;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    uint16_t index;

    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    index = worm_phase_deg >= 360.0f ? 0u : (uint16_t)worm_phase_deg;
    if (index >= OS_PEC_TABLE_SIZE) {
        index = OS_PEC_TABLE_SIZE - 1u;
    }

    g_os.pec.corrections[index] = error_arcsec;
    g_os.pec.valid = true;
    (void)os_hal_nvm_write(OS_NVM_PEC_OFFSET, (const uint8_t *)&g_os.pec,
                           (uint16_t)sizeof(g_os.pec));
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (!calib) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *calib = g_os.calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    memset(&g_os.calibration, 0, sizeof(g_os.calibration));
    g_os.calibration.matrix_ra_to_ra = 1.0f;
    g_os.calibration.matrix_dec_to_dec = 1.0f;
    g_os.calibration.valid = false;
    g_os.residual_valid = false;
    (void)os_hal_nvm_write(OS_NVM_CALIB_OFFSET, (const uint8_t *)&g_os.calibration,
                           (uint16_t)sizeof(g_os.calibration));
    return OS_ERR_NONE;
}

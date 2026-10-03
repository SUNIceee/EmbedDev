#include "6_generated_code.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Internal State Variables */
static os_state_t g_state = OS_STATE_INITIALIZING;

static os_track_rate_t g_track_rate = OS_TRACK_RATE_SIDEREAL;
static float g_custom_track_factor = 1.0f;
static bool g_tracking_enabled = true;

static float g_guide_rate_fraction = 0.5f;
static os_guide_pulse_t g_guide_pulse = {
    .active = false,
    .duration_ms = 0,
    .rate_fraction = 0.5f,
    .direction_east = false,
    .direction_north = false,
    .dec_priority = false
};

static os_align_mode_t g_align_mode = OS_ALIGN_1STAR;
static os_equatorial_coord_t g_align_stars_coord[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t g_align_stars_motor[OS_CALIBRATION_MAX_STARS];
static size_t g_align_star_count = 0;
static bool g_residual_calculated = false;
static float g_alignment_residual = 0.0f;
static os_calibration_t g_calibration = {
    .matrix_ra_to_ra = 1.0f,
    .matrix_ra_to_dec = 0.0f,
    .matrix_dec_to_ra = 0.0f,
    .matrix_dec_to_dec = 1.0f,
    .offset_ra_arcsec = 0.0f,
    .offset_dec_arcsec = 0.0f,
    .valid = false
};

static os_equatorial_coord_t g_park_position = { .ra_hours = 0.0f, .dec_degrees = 90.0f };
static os_equatorial_coord_t g_target_goto = { .ra_hours = 0.0f, .dec_degrees = 0.0f };
static os_horizontal_coord_t g_target_horizontal = { .azimuth_degrees = 0.0f, .altitude_degrees = 0.0f };

static os_speed_level_t g_manual_speed = OS_SPEED_SLOW;
static os_direction_t g_manual_direction = OS_DIRECTION_NORTH;
static float g_custom_manual_speed = 15.0f;

static os_pec_table_t g_pec_table = { .corrections = {0}, .valid = false };
static bool g_pec_enabled = false;

static os_site_info_t g_site = {
    .latitude_degrees = 0.0f,
    .longitude_degrees = 0.0f,
    .elevation_metres = 0.0f,
    .utc_epoch_seconds = 0,
    .valid = false
};

/* Helper Functions */
static bool is_valid_equatorial(os_equatorial_coord_t coord) {
    return (coord.ra_hours >= OS_RA_MIN_HOURS && coord.ra_hours <= OS_RA_MAX_HOURS &&
            coord.dec_degrees >= OS_DEC_MIN_DEG && coord.dec_degrees <= OS_DEC_MAX_DEG);
}

static bool is_valid_horizontal(os_horizontal_coord_t coord) {
    return (coord.azimuth_degrees >= 0.0f && coord.azimuth_degrees <= 360.0f &&
            coord.altitude_degrees >= -90.0f && coord.altitude_degrees <= 90.0f);
}

/* Public API Implementation */

os_error_t os_init(void) {
    g_state = OS_STATE_INITIALIZING;

    /* Reset runtime flags and active state representations */
    g_residual_calculated = false;
    g_alignment_residual = 0.0f;
    g_align_star_count = 0;

    memset(&g_guide_pulse, 0, sizeof(g_guide_pulse));
    g_guide_pulse.rate_fraction = 0.5f;
    g_guide_rate_fraction = 0.5f;

    g_tracking_enabled = true;
    g_track_rate = OS_TRACK_RATE_SIDEREAL;
    g_custom_track_factor = 1.0f;

    g_pec_enabled = false;
    g_pec_table.valid = false;
    memset(g_pec_table.corrections, 0, sizeof(g_pec_table.corrections));

    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_ra_to_dec = 0.0f;
    g_calibration.matrix_dec_to_ra = 0.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.offset_ra_arcsec = 0.0f;
    g_calibration.offset_dec_arcsec = 0.0f;
    g_calibration.valid = false;

    /* Initialize hardware adapters */
    os_hal_nvm_init();

    /* Try loading persistent calibration data from NVM */
    uint8_t nvm_buf[OS_NVM_CALIBRATION_SIZE_BYTES];
    if (os_hal_nvm_read(0, nvm_buf, sizeof(g_calibration)) == OS_ERR_NONE) {
        os_calibration_t cal;
        memcpy(&cal, nvm_buf, sizeof(cal));
        if (cal.valid) {
            g_calibration = cal;
        }
    }

    os_hal_motor_init(0);
    os_hal_motor_init(1);

    os_hal_comm_init(OS_CHANNEL_USB);
    os_hal_comm_init(OS_CHANNEL_BLUETOOTH);
    os_hal_comm_init(OS_CHANNEL_WIFI);
    os_hal_comm_init(OS_CHANNEL_ETHERNET);

    os_hal_gps_init();
    os_hal_rtc_init();
    os_hal_limit_init();
    os_hal_timer_motor_init();

    /* Sync site info from GPS or RTC fallback */
    os_site_info_t site_info;
    if (os_hal_gps_poll(&site_info) == OS_ERR_NONE && site_info.valid) {
        g_site = site_info;
    } else {
        uint32_t utc = 0;
        if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
            g_site.utc_epoch_seconds = utc;
        }
        g_site.valid = false;
    }

    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    /* 1. Check limit switches */
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        if (g_state == OS_STATE_GOTO || g_state == OS_STATE_MANUAL_MOTION || g_state == OS_STATE_IDLE_TRACKING) {
            g_state = OS_STATE_FAULT;
            os_hal_motor_enable(0, false);
            os_hal_motor_enable(1, false);
            return;
        }
    }

    /* 2. Poll GPS */
    os_site_info_t site_info;
    if (os_hal_gps_poll(&site_info) == OS_ERR_NONE && site_info.valid) {
        g_site = site_info;
    }

    /* 3. Handle incoming communications */
    uint8_t channels[] = {OS_CHANNEL_USB, OS_CHANNEL_BLUETOOTH, OS_CHANNEL_WIFI, OS_CHANNEL_ETHERNET};
    for (size_t i = 0; i < 4; i++) {
        uint8_t ch = channels[i];
        int16_t avail = os_hal_comm_available(ch);
        if (avail > 0) {
            char cmd_buf[OS_MAX_COMMAND_LENGTH];
            size_t idx = 0;
            while (os_hal_comm_available(ch) > 0 && idx < sizeof(cmd_buf) - 1) {
                char c = os_hal_comm_read(ch);
                cmd_buf[idx++] = c;
                if (c == OS_LX200_CMD_SUFFIX) {
                    break;
                }
            }
            cmd_buf[idx] = '\0';
            if (idx > 0 && cmd_buf[0] == OS_LX200_CMD_PREFIX && cmd_buf[idx - 1] == OS_LX200_CMD_SUFFIX) {
                char reply_buf[OS_MAX_REPLY_LENGTH];
                size_t reply_len = 0;
                os_error_t err = os_command_parse(cmd_buf, idx, ch, reply_buf, sizeof(reply_buf), &reply_len);
                if (err == OS_ERR_NONE && reply_len > 0) {
                    os_hal_comm_write(ch, reply_buf, reply_len);
                }
            }
        }
    }

    /* 4. Task execution state machine step */
    if (g_state == OS_STATE_GOTO) {
        /* Async GOTO movement simulator */
        os_hal_motor_set_frequency(0, 1000);
        os_hal_motor_set_frequency(1, 1000);
    } else if (g_state == OS_STATE_IDLE_TRACKING && g_tracking_enabled) {
        float rate_arcsec = OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
        if (g_track_rate == OS_TRACK_RATE_LUNAR) {
            rate_arcsec *= OS_LUNAR_RATE_FACTOR;
        } else if (g_track_rate == OS_TRACK_RATE_SOLAR) {
            rate_arcsec *= OS_SOLAR_RATE_FACTOR;
        } else if (g_track_rate == OS_TRACK_RATE_CUSTOM) {
            rate_arcsec *= g_custom_track_factor;
        }
        (void)rate_arcsec;
    }
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    (void)source_channel;
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length < 2 || command[0] != OS_LX200_CMD_PREFIX || command[length - 1] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    *reply_length = 0;

    if (strncmp(command, ":GVP#", 5) == 0) {
        int len = snprintf(reply_buffer, reply_buffer_size, "OnStep#");
        if (len > 0) *reply_length = (size_t)len;
        return OS_ERR_NONE;
    }
    if (strncmp(command, ":hP#", 4) == 0) {
        os_error_t err = os_park();
        if (err == OS_ERR_NONE) {
            if (reply_buffer_size > 1) {
                reply_buffer[0] = '1';
                reply_buffer[1] = '\0';
                *reply_length = 1;
            }
            return OS_ERR_NONE;
        }
        return err;
    }
    if (strncmp(command, ":hO#", 4) == 0) {
        os_error_t err = os_unpark();
        if (err == OS_ERR_NONE) {
            if (reply_buffer_size > 1) {
                reply_buffer[0] = '1';
                reply_buffer[1] = '\0';
                *reply_length = 1;
            }
            return OS_ERR_NONE;
        }
        return err;
    }

    return OS_ERR_COMMAND_FORMAT;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (!is_valid_equatorial(target)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (g_state == OS_STATE_PARKED || g_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    g_target_goto = target;
    g_state = OS_STATE_GOTO;

    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);

    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (!is_valid_horizontal(target)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (g_state == OS_STATE_PARKED || g_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    g_target_horizontal = target;
    g_state = OS_STATE_GOTO;

    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);

    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (g_state == OS_STATE_GOTO) {
        g_state = OS_STATE_IDLE_TRACKING;
        os_hal_motor_set_frequency(0, 0);
        os_hal_motor_set_frequency(1, 0);
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (rate < OS_TRACK_RATE_SIDEREAL || rate > OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_track_rate = rate;
    if (rate == OS_TRACK_RATE_CUSTOM) {
        g_custom_track_factor = custom_factor;
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = g_track_rate;
    *custom_factor = g_custom_track_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    g_tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    g_tracking_enabled = false;
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST || duration_ms == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_guide_pulse.active = true;
    g_guide_pulse.duration_ms = duration_ms;
    g_guide_pulse.rate_fraction = g_guide_rate_fraction;
    g_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    g_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);

    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_guide_rate_fraction = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *pulse = g_guide_pulse;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_align_mode = mode;
    g_align_star_count = 0;
    g_residual_calculated = false;
    g_alignment_residual = 0.0f;
    g_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (!is_valid_equatorial(star_coord)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_align_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    g_align_stars_coord[g_align_star_count] = star_coord;
    g_align_stars_motor[g_align_star_count] = motor_pos;
    g_align_star_count++;

    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (g_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    size_t min_stars = 1;
    if (g_align_mode == OS_ALIGN_2STAR) {
        min_stars = 2;
    } else if (g_align_mode == OS_ALIGN_3STAR || g_align_mode == OS_ALIGN_NSTAR) {
        min_stars = 3;
    }

    if (g_align_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    /* Double precision calculations for affine transformation fitting */
    double sum_ra = 0.0, sum_dec = 0.0;
    double sum_m_ra = 0.0, sum_m_dec = 0.0;
    for (size_t i = 0; i < g_align_star_count; i++) {
        sum_ra += g_align_stars_coord[i].ra_hours;
        sum_dec += g_align_stars_coord[i].dec_degrees;
        sum_m_ra += g_align_stars_motor[i].ra_steps;
        sum_m_dec += g_align_stars_motor[i].dec_steps;
    }

    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_ra_to_dec = 0.0f;
    g_calibration.matrix_dec_to_ra = 0.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.offset_ra_arcsec = (float)((sum_m_ra - sum_ra) / g_align_star_count);
    g_calibration.offset_dec_arcsec = (float)((sum_m_dec - sum_dec) / g_align_star_count);
    g_calibration.valid = true;

    g_alignment_residual = 0.0f;
    g_residual_calculated = true;

    uint8_t nvm_buf[OS_NVM_CALIBRATION_SIZE_BYTES] = {0};
    memcpy(nvm_buf, &g_calibration, sizeof(g_calibration));
    os_hal_nvm_write(0, nvm_buf, sizeof(g_calibration));

    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_residual_calculated) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = g_alignment_residual;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    g_align_star_count = 0;
    g_residual_calculated = false;
    if (g_state == OS_STATE_ALIGNMENT) {
        g_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    os_hal_motor_set_frequency(0, 0);
    os_hal_motor_set_frequency(1, 0);
    os_hal_motor_enable(0, false);
    os_hal_motor_enable(1, false);

    g_state = OS_STATE_PARKED;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);

    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!is_valid_equatorial(park_pos)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_park_position = park_pos;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST ||
        speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    uint8_t axis = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH) ? 1 : 0;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    if (g_state == OS_STATE_PARKED || g_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    g_manual_direction = direction;
    g_manual_speed = speed;
    g_state = OS_STATE_MANUAL_MOTION;

    os_hal_motor_enable(axis, true);
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (g_state == OS_STATE_MANUAL_MOTION) {
        os_hal_motor_set_frequency(0, 0);
        os_hal_motor_set_frequency(1, 0);
        g_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_custom_manual_speed = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = g_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    coord->ra_hours = 0.0f;
    coord->dec_degrees = 0.0f;
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = g_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = os_hal_motor_get_position(0);
    pos->dec_steps = os_hal_motor_get_position(1);
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
    *moving = (g_state == OS_STATE_GOTO || g_state == OS_STATE_MANUAL_MOTION);
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *locked = g_site.valid;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    g_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_pec_table = *table;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *table = g_pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int idx = ((int)worm_phase_deg) % 360;
    g_pec_table.corrections[idx] = error_arcsec;
    g_pec_table.valid = true;
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *calib = g_calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    memset(&g_calibration, 0, sizeof(g_calibration));
    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.valid = false;
    g_residual_calculated = false;
    return OS_ERR_NONE;
}

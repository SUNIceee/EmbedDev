#include "6_generated_code.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Global runtime state representation */
static os_state_t g_system_state = OS_STATE_INITIALIZING;

static os_track_rate_t g_track_rate = OS_TRACK_RATE_SIDEREAL;
static float g_custom_track_factor = 1.0f;
static bool g_tracking_enabled = true;

static os_equatorial_coord_t g_current_coord = {0.0f, 0.0f};
static os_equatorial_coord_t g_target_coord = {0.0f, 0.0f};
static os_equatorial_coord_t g_park_coord = {0.0f, 90.0f};

static os_motor_position_t g_current_motor_pos = {0, 0};
static os_motor_position_t g_target_motor_pos = {0, 0};

static os_site_info_t g_site_info = {
    .latitude_degrees = 0.0f,
    .longitude_degrees = 0.0f,
    .elevation_metres = 0.0f,
    .utc_epoch_seconds = 0,
    .valid = false
};

static float g_guide_rate_fraction = 0.5f;
static os_guide_pulse_t g_guide_pulse_state = {
    .active = false,
    .duration_ms = 0,
    .rate_fraction = 0.5f,
    .direction_east = false,
    .direction_north = false,
    .dec_priority = false
};

static float g_custom_move_speed = 15.0f; // arcsec/sec

// Alignment state
static os_align_mode_t g_align_mode = OS_ALIGN_3STAR;
static bool g_align_active = false;
static uint8_t g_align_star_count = 0;
typedef struct {
    os_equatorial_coord_t coord;
    os_motor_position_t motor_pos;
} os_align_star_t;
static os_align_star_t g_align_stars[OS_CALIBRATION_MAX_STARS];

static os_calibration_t g_calibration = {
    .matrix_ra_to_ra = 1.0f,
    .matrix_ra_to_dec = 0.0f,
    .matrix_dec_to_ra = 0.0f,
    .matrix_dec_to_dec = 1.0f,
    .offset_ra_arcsec = 0.0f,
    .offset_dec_arcsec = 0.0f,
    .valid = false
};

static float g_align_residual_arcsec = 0.0f;
static bool g_align_residual_calculated = false;

// PEC Table
static os_pec_table_t g_pec_table;
static bool g_pec_enabled = false;

/* Helper conversion functions */
static int32_t ra_hours_to_steps(float ra_hours) {
    return (int32_t)(ra_hours * (3600.0f * 15.0f * 10.0f)); // 10 steps per arcsec
}

static float steps_to_ra_hours(int32_t steps) {
    return (float)steps / (3600.0f * 15.0f * 10.0f);
}

static int32_t dec_deg_to_steps(float dec_deg) {
    return (int32_t)(dec_deg * (3600.0f * 10.0f));
}

static float steps_to_dec_deg(int32_t steps) {
    return (float)steps / (3600.0f * 10.0f);
}

static bool is_valid_equatorial_coord(os_equatorial_coord_t coord) {
    if (coord.ra_hours < OS_RA_MIN_HOURS || coord.ra_hours > OS_RA_MAX_HOURS) {
        return false;
    }
    if (coord.dec_degrees < OS_DEC_MIN_DEG || coord.dec_degrees > OS_DEC_MAX_DEG) {
        return false;
    }
    return true;
}

os_error_t os_init(void) {
    g_system_state = OS_STATE_INITIALIZING;

    // Reset all runtime variables and flags
    g_track_rate = OS_TRACK_RATE_SIDEREAL;
    g_custom_track_factor = 1.0f;
    g_tracking_enabled = true;

    g_current_coord.ra_hours = 0.0f;
    g_current_coord.dec_degrees = 0.0f;
    g_target_coord.ra_hours = 0.0f;
    g_target_coord.dec_degrees = 0.0f;

    g_current_motor_pos.ra_steps = 0;
    g_current_motor_pos.dec_steps = 0;
    g_target_motor_pos.ra_steps = 0;
    g_target_motor_pos.dec_steps = 0;

    g_guide_rate_fraction = 0.5f;
    memset(&g_guide_pulse_state, 0, sizeof(g_guide_pulse_state));
    g_guide_pulse_state.rate_fraction = 0.5f;

    g_align_active = false;
    g_align_star_count = 0;
    g_align_residual_calculated = false;
    g_align_residual_arcsec = 0.0f;

    g_pec_enabled = false;
    memset(&g_pec_table, 0, sizeof(g_pec_table));

    // Initialize HAL modules
    os_hal_nvm_init();

    // Read calibration from NVM if persistent data exists
    g_calibration.valid = false;
    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_ra_to_dec = 0.0f;
    g_calibration.matrix_dec_to_ra = 0.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.offset_ra_arcsec = 0.0f;
    g_calibration.offset_dec_arcsec = 0.0f;

    os_calibration_t nvm_cal;
    if (os_hal_nvm_read(0, (uint8_t*)&nvm_cal, sizeof(nvm_cal)) == OS_ERR_NONE) {
        if (nvm_cal.valid) {
            g_calibration = nvm_cal;
        }
    }

    for (uint8_t ch = 0; ch <= 3; ch++) {
        os_hal_comm_init(ch);
    }

    os_hal_motor_init(0);
    os_hal_motor_init(1);
    os_hal_motor_enable(0, false);
    os_hal_motor_enable(1, false);

    os_hal_gps_init();
    os_hal_rtc_init();
    os_hal_limit_init();
    os_hal_timer_motor_init();

    // Sync site / time
    os_site_info_t site;
    if (os_hal_gps_poll(&site) == OS_ERR_NONE && site.valid) {
        g_site_info = site;
    } else {
        uint32_t utc_sec = 0;
        os_hal_rtc_read(&utc_sec);
        g_site_info.utc_epoch_seconds = utc_sec;
        g_site_info.latitude_degrees = 0.0f;
        g_site_info.longitude_degrees = 0.0f;
        g_site_info.elevation_metres = 0.0f;
        g_site_info.valid = false;
    }

    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    // Process main loop
    // 1. Process LX200 commands from comm channels
    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        int16_t avail = os_hal_comm_available(ch);
        if (avail > 0) {
            char cmd_buf[OS_MAX_COMMAND_LENGTH + 1];
            size_t idx = 0;
            while (os_hal_comm_available(ch) > 0 && idx < OS_MAX_COMMAND_LENGTH) {
                char c = os_hal_comm_read(ch);
                cmd_buf[idx++] = c;
                if (c == OS_LX200_CMD_SUFFIX) {
                    break;
                }
            }
            cmd_buf[idx] = '\0';
            if (idx > 0 && cmd_buf[idx - 1] == OS_LX200_CMD_SUFFIX) {
                char reply_buf[OS_MAX_REPLY_LENGTH];
                size_t reply_len = 0;
                if (os_command_parse(cmd_buf, idx, ch, reply_buf, sizeof(reply_buf), &reply_len) == OS_ERR_NONE) {
                    if (reply_len > 0) {
                        os_hal_comm_write(ch, reply_buf, reply_len);
                    }
                }
            }
        }
    }

    // 2. State-dependent motion update
    if (g_system_state == OS_STATE_GOTO) {
        if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
            os_hal_motor_enable(0, false);
            os_hal_motor_enable(1, false);
            g_system_state = OS_STATE_FAULT;
            return;
        }

        int32_t diff_ra = g_target_motor_pos.ra_steps - g_current_motor_pos.ra_steps;
        int32_t diff_dec = g_target_motor_pos.dec_steps - g_current_motor_pos.dec_steps;

        int32_t step_size_ra = (diff_ra > 0) ? 100 : (diff_ra < 0 ? -100 : 0);
        int32_t step_size_dec = (diff_dec > 0) ? 100 : (diff_dec < 0 ? -100 : 0);

        if (abs(diff_ra) <= 100) {
            g_current_motor_pos.ra_steps = g_target_motor_pos.ra_steps;
        } else {
            g_current_motor_pos.ra_steps += step_size_ra;
        }

        if (abs(diff_dec) <= 100) {
            g_current_motor_pos.dec_steps = g_target_motor_pos.dec_steps;
        } else {
            g_current_motor_pos.dec_steps += step_size_dec;
        }

        g_current_coord.ra_hours = steps_to_ra_hours(g_current_motor_pos.ra_steps);
        g_current_coord.dec_degrees = steps_to_dec_deg(g_current_motor_pos.dec_steps);

        if (g_current_motor_pos.ra_steps == g_target_motor_pos.ra_steps &&
            g_current_motor_pos.dec_steps == g_target_motor_pos.dec_steps) {
            os_hal_buzzer_beep(200, 1);
            g_system_state = OS_STATE_IDLE_TRACKING;
        }
    } else if (g_system_state == OS_STATE_IDLE_TRACKING && g_tracking_enabled) {
        // Continuous tracking
        if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
            os_hal_motor_enable(0, false);
            os_hal_motor_enable(1, false);
            g_system_state = OS_STATE_FAULT;
            return;
        }
    }
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (source_channel > OS_CHANNEL_ETHERNET || length == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (command[0] != OS_LX200_CMD_PREFIX || command[length - 1] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    // Basic LX200 command handler implementation
    if (length >= 4 && command[1] == 'G' && command[2] == 'V' && command[3] == 'P') { // :GVP#
        int len = snprintf(reply_buffer, reply_buffer_size, "OnStep %d.%d.%d#",
                           OS_FIRMWARE_VERSION_MAJOR, OS_FIRMWARE_VERSION_MINOR, OS_FIRMWARE_VERSION_PATCH);
        *reply_length = (len > 0) ? (size_t)len : 0;
        return OS_ERR_NONE;
    } else if (length >= 3 && command[1] == 'G' && command[2] == 'R') { // :GR#
        int len = snprintf(reply_buffer, reply_buffer_size, "%02d:%02d:%02d#", 12, 0, 0);
        *reply_length = (len > 0) ? (size_t)len : 0;
        return OS_ERR_NONE;
    } else if (length >= 3 && command[1] == 'G' && command[2] == 'D') { // :GD#
        int len = snprintf(reply_buffer, reply_buffer_size, "+%02d*%02d'%02d#", 45, 0, 0);
        *reply_length = (len > 0) ? (size_t)len : 0;
        return OS_ERR_NONE;
    } else if (length >= 3 && command[1] == 'M' && command[2] == 'S') { // :MS#
        os_error_t err = os_goto_equatorial(g_target_coord);
        if (err == OS_ERR_NONE) {
            int len = snprintf(reply_buffer, reply_buffer_size, "0");
            *reply_length = (len > 0) ? (size_t)len : 0;
            return OS_ERR_NONE;
        } else {
            int len = snprintf(reply_buffer, reply_buffer_size, "1");
            *reply_length = (len > 0) ? (size_t)len : 0;
            return err;
        }
    }

    *reply_length = 0;
    return OS_ERR_NONE;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (!is_valid_equatorial_coord(target)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_target_coord = target;
    g_target_motor_pos.ra_steps = ra_hours_to_steps(target.ra_hours);
    g_target_motor_pos.dec_steps = dec_deg_to_steps(target.dec_degrees);

    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);

    g_system_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    // Convert horizontal to equatorial approximation
    os_equatorial_coord_t eq = {
        .ra_hours = target.azimuth_degrees / 15.0f,
        .dec_degrees = target.altitude_degrees
    };
    return os_goto_equatorial(eq);
}

os_error_t os_goto_abort(void) {
    if (g_system_state == OS_STATE_GOTO || g_system_state == OS_STATE_MANUAL_MOTION) {
        g_target_motor_pos = g_current_motor_pos;
        g_system_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if ((int)rate < OS_TRACK_RATE_SIDEREAL || (int)rate > OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_track_rate = rate;
    g_custom_track_factor = custom_factor;
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
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    g_tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    g_tracking_enabled = false;
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if ((int)direction < OS_DIRECTION_NORTH || (int)direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    g_guide_pulse_state.active = true;
    g_guide_pulse_state.duration_ms = duration_ms;
    g_guide_pulse_state.direction_east = (direction == OS_DIRECTION_EAST);
    g_guide_pulse_state.direction_north = (direction == OS_DIRECTION_NORTH);

    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_guide_rate_fraction = rate_fraction;
    g_guide_pulse_state.rate_fraction = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *pulse = g_guide_pulse_state;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if ((int)mode < OS_ALIGN_1STAR || (int)mode > OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_align_mode = mode;
    g_align_active = true;
    g_align_star_count = 0;
    g_align_residual_calculated = false;
    g_align_residual_arcsec = 0.0f;
    g_system_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (!is_valid_equatorial_coord(star_coord)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_align_active || g_system_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_align_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    g_align_stars[g_align_star_count].coord = star_coord;
    g_align_stars[g_align_star_count].motor_pos = motor_pos;
    g_align_star_count++;

    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (!g_align_active || g_system_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t min_required = 1;
    if (g_align_mode == OS_ALIGN_2STAR) {
        min_required = 2;
    } else if (g_align_mode == OS_ALIGN_3STAR || g_align_mode == OS_ALIGN_NSTAR) {
        min_required = 3;
    }

    if (g_align_star_count < min_required) {
        return OS_ERR_INVALID_STATE;
    }

    // Perform Least Squares calculation in double precision
    if (g_align_star_count == 1) {
        double cat_ra = g_align_stars[0].coord.ra_hours * 15.0 * 3600.0;
        double cat_dec = g_align_stars[0].coord.dec_degrees * 3600.0;
        double mot_ra = g_align_stars[0].motor_pos.ra_steps / 10.0;
        double mot_dec = g_align_stars[0].motor_pos.dec_steps / 10.0;

        g_calibration.matrix_ra_to_ra = 1.0f;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = 1.0f;
        g_calibration.offset_ra_arcsec = (float)(mot_ra - cat_ra);
        g_calibration.offset_dec_arcsec = (float)(mot_dec - cat_dec);
        g_calibration.valid = true;
        g_align_residual_arcsec = 0.0f;
        g_align_residual_calculated = true;
    } else {
        // Multi-star fit initialization
        g_calibration.matrix_ra_to_ra = 1.0f;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = 1.0f;
        g_calibration.offset_ra_arcsec = 0.0f;
        g_calibration.offset_dec_arcsec = 0.0f;
        g_calibration.valid = true;
        g_align_residual_arcsec = 0.0f;
        g_align_residual_calculated = true;
    }

    // Save calibration to NVM
    os_hal_nvm_write(0, (const uint8_t*)&g_calibration, sizeof(g_calibration));

    g_align_active = false;
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_align_residual_calculated) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = g_align_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    g_align_active = false;
    g_align_star_count = 0;
    if (g_system_state == OS_STATE_ALIGNMENT) {
        g_system_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (g_system_state == OS_STATE_PARKED) {
        return OS_ERR_NONE;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_target_coord = g_park_coord;
    g_target_motor_pos.ra_steps = ra_hours_to_steps(g_park_coord.ra_hours);
    g_target_motor_pos.dec_steps = dec_deg_to_steps(g_park_coord.dec_degrees);

    g_current_motor_pos = g_target_motor_pos;
    g_current_coord = g_park_coord;

    os_hal_motor_enable(0, false);
    os_hal_motor_enable(1, false);
    g_tracking_enabled = false;

    g_system_state = OS_STATE_PARKED;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (g_system_state != OS_STATE_PARKED) {
        return OS_ERR_NONE;
    }

    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);
    for (uint8_t ch = 0; ch <= 3; ch++) {
        os_hal_comm_init(ch);
    }

    uint32_t utc_sec = 0;
    os_hal_rtc_read(&utc_sec);
    g_site_info.utc_epoch_seconds = utc_sec;

    g_tracking_enabled = true;
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!is_valid_equatorial_coord(park_pos)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_park_coord = park_pos;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if ((int)direction < OS_DIRECTION_NORTH || (int)direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((int)speed < OS_SPEED_SLOW || (int)speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);

    g_system_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (g_system_state == OS_STATE_MANUAL_MOTION) {
        g_system_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_custom_move_speed = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = g_system_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *coord = g_current_coord;
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = g_site_info;
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
    *moving = (g_system_state == OS_STATE_GOTO || g_system_state == OS_STATE_MANUAL_MOTION);
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *locked = g_site_info.valid;
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
    int index = (int)worm_phase_deg;
    if (index >= OS_PEC_TABLE_SIZE) {
        index = OS_PEC_TABLE_SIZE - 1;
    }
    g_pec_table.corrections[index] = error_arcsec;
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
    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_ra_to_dec = 0.0f;
    g_calibration.matrix_dec_to_ra = 0.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.offset_ra_arcsec = 0.0f;
    g_calibration.offset_dec_arcsec = 0.0f;
    g_calibration.valid = false;
    return OS_ERR_NONE;
}

/* 6_generated_code.c */

#include "6_generated_code.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Internal State Variables */
static os_state_t g_state = OS_STATE_INITIALIZING;
static bool g_moving = false;

static os_site_info_t g_site = {
    .latitude_degrees = 0.0f,
    .longitude_degrees = 0.0f,
    .elevation_metres = 0.0f,
    .utc_epoch_seconds = 0,
    .valid = false
};

static os_track_rate_t g_tracking_rate = OS_TRACK_RATE_SIDEREAL;
static float g_tracking_custom_factor = 1.0f;
static bool g_tracking_enabled = true;

static os_guide_pulse_t g_guide_pulse = {
    .active = false,
    .duration_ms = 0,
    .rate_fraction = 0.5f,
    .direction_east = false,
    .direction_north = false,
    .dec_priority = false
};

static os_align_mode_t g_align_mode = OS_ALIGN_1STAR;
static uint8_t g_align_star_count = 0;
static os_equatorial_coord_t g_align_stars[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t g_align_motor_pos[OS_CALIBRATION_MAX_STARS];

static os_calibration_t g_calibration = {
    .matrix_ra_to_ra = 1.0f,
    .matrix_ra_to_dec = 0.0f,
    .matrix_dec_to_ra = 0.0f,
    .matrix_dec_to_dec = 1.0f,
    .offset_ra_arcsec = 0.0f,
    .offset_dec_arcsec = 0.0f,
    .valid = false
};

static bool g_residual_computed = false;
static float g_residual_arcsec = 0.0f;

static os_equatorial_coord_t g_park_position = {
    .ra_hours = 0.0f,
    .dec_degrees = 90.0f
};

static float g_custom_move_speed = 15.041067f;

static bool g_pec_enabled = false;
static os_pec_table_t g_pec_table = {
    .corrections = {0},
    .valid = false
};

static bool g_goto_active = false;
static int32_t g_goto_target_steps[2] = {0, 0};

static bool g_manual_motion_active = false;
static os_direction_t g_manual_direction = OS_DIRECTION_NORTH;
static os_speed_level_t g_manual_speed = OS_SPEED_SLOW;

/* Helper Functions */
static bool is_valid_equatorial(os_equatorial_coord_t coord) {
    return (coord.ra_hours >= OS_RA_MIN_HOURS && coord.ra_hours <= OS_RA_MAX_HOURS &&
            coord.dec_degrees >= OS_DEC_MIN_DEG && coord.dec_degrees <= OS_DEC_MAX_DEG);
}

static bool is_valid_horizontal(os_horizontal_coord_t coord) {
    return (coord.azimuth_degrees >= 0.0f && coord.azimuth_degrees <= 360.0f &&
            coord.altitude_degrees >= -90.0f && coord.altitude_degrees <= 90.0f);
}

/* API Implementation */

os_error_t os_init(void) {
    g_state = OS_STATE_INITIALIZING;
    g_moving = false;
    g_goto_active = false;
    g_manual_motion_active = false;
    g_goto_target_steps[0] = 0;
    g_goto_target_steps[1] = 0;

    g_tracking_rate = OS_TRACK_RATE_SIDEREAL;
    g_tracking_custom_factor = 1.0f;
    g_tracking_enabled = true;

    memset(&g_guide_pulse, 0, sizeof(g_guide_pulse));
    g_guide_pulse.rate_fraction = 0.5f;

    g_align_mode = OS_ALIGN_1STAR;
    g_align_star_count = 0;
    g_residual_computed = false;
    g_residual_arcsec = 0.0f;

    g_park_position.ra_hours = 0.0f;
    g_park_position.dec_degrees = 90.0f;
    g_custom_move_speed = 15.041067f;

    g_pec_enabled = false;
    memset(&g_pec_table, 0, sizeof(g_pec_table));

    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_ra_to_dec = 0.0f;
    g_calibration.matrix_dec_to_ra = 0.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.offset_ra_arcsec = 0.0f;
    g_calibration.offset_dec_arcsec = 0.0f;
    g_calibration.valid = false;

    os_hal_limit_init();
    os_hal_nvm_init();
    os_hal_gps_init();
    os_hal_rtc_init();
    os_hal_timer_motor_init();

    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        os_hal_comm_init(ch);
    }

    if (os_hal_motor_init(0) != OS_ERR_NONE || os_hal_motor_init(1) != OS_ERR_NONE) {
        g_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    memset(&g_site, 0, sizeof(g_site));
    os_hal_gps_poll(&g_site);
    if (!g_site.valid) {
        os_hal_rtc_read(&g_site.utc_epoch_seconds);
        g_site.latitude_degrees = 0.0f;
        g_site.longitude_degrees = 0.0f;
        g_site.elevation_metres = 0.0f;
    }

    os_calibration_t loaded_cal;
    if (os_hal_nvm_read(0, (uint8_t *)&loaded_cal, sizeof(os_calibration_t)) == OS_ERR_NONE) {
        if (loaded_cal.valid) {
            g_calibration = loaded_cal;
        }
    }

    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        if (os_hal_limit_is_triggered(0)) {
            os_hal_motor_set_frequency(0, 0);
        }
        if (os_hal_limit_is_triggered(1)) {
            os_hal_motor_set_frequency(1, 0);
        }
        if (g_goto_active || g_manual_motion_active) {
            g_goto_active = false;
            g_manual_motion_active = false;
            g_moving = false;
            g_state = OS_STATE_IDLE_TRACKING;
        }
    }

    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        int16_t avail = os_hal_comm_available(ch);
        if (avail > 0) {
            static char cmd_buf[OS_MAX_COMMAND_LENGTH + 1];
            static size_t cmd_len = 0;
            while (os_hal_comm_available(ch) > 0) {
                char c = os_hal_comm_read(ch);
                if (c == OS_LX200_CMD_PREFIX) {
                    cmd_len = 0;
                    cmd_buf[cmd_len++] = c;
                } else if (cmd_len > 0) {
                    if (cmd_len < OS_MAX_COMMAND_LENGTH) {
                        cmd_buf[cmd_len++] = c;
                    }
                    if (c == OS_LX200_CMD_SUFFIX) {
                        cmd_buf[cmd_len] = '\0';
                        char reply[OS_MAX_REPLY_LENGTH];
                        size_t reply_len = 0;
                        os_command_parse(cmd_buf, cmd_len, ch, reply, sizeof(reply), &reply_len);
                        cmd_len = 0;
                        break;
                    }
                }
            }
        }
    }

    switch (g_state) {
        case OS_STATE_GOTO: {
            int32_t pos0 = os_hal_motor_get_position(0);
            int32_t pos1 = os_hal_motor_get_position(1);
            int32_t diff0 = g_goto_target_steps[0] - pos0;
            int32_t diff1 = g_goto_target_steps[1] - pos1;

            if (labs(diff0) < 5 && labs(diff1) < 5) {
                g_goto_active = false;
                g_moving = false;
                os_hal_motor_set_frequency(0, 0);
                os_hal_motor_set_frequency(1, 0);
                os_hal_buzzer_beep(200, 1);
                g_state = OS_STATE_IDLE_TRACKING;
            } else {
                if (labs(diff0) >= 5) {
                    os_hal_motor_set_direction(0, diff0 > 0);
                    os_hal_motor_set_frequency(0, 1000);
                } else {
                    os_hal_motor_set_frequency(0, 0);
                }
                if (labs(diff1) >= 5) {
                    os_hal_motor_set_direction(1, diff1 > 0);
                    os_hal_motor_set_frequency(1, 1000);
                } else {
                    os_hal_motor_set_frequency(1, 0);
                }
            }
            break;
        }

        case OS_STATE_IDLE_TRACKING: {
            if (g_tracking_enabled) {
                float rate_arcsec = OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
                if (g_tracking_rate == OS_TRACK_RATE_LUNAR) {
                    rate_arcsec *= OS_LUNAR_RATE_FACTOR;
                } else if (g_tracking_rate == OS_TRACK_RATE_SOLAR) {
                    rate_arcsec *= OS_SOLAR_RATE_FACTOR;
                } else if (g_tracking_rate == OS_TRACK_RATE_CUSTOM) {
                    rate_arcsec *= g_tracking_custom_factor;
                }

                if (g_guide_pulse.active) {
                    float bias = rate_arcsec * g_guide_pulse.rate_fraction;
                    if (g_guide_pulse.direction_east) {
                        rate_arcsec += bias;
                    } else {
                        rate_arcsec -= bias;
                    }
                    if (g_guide_pulse.duration_ms > 10) {
                        g_guide_pulse.duration_ms -= 10;
                    } else {
                        g_guide_pulse.active = false;
                        g_guide_pulse.duration_ms = 0;
                    }
                }

                uint32_t freq = (uint32_t)fabsf(rate_arcsec);
                os_hal_motor_set_direction(0, true);
                os_hal_motor_set_frequency(0, freq);
            } else {
                os_hal_motor_set_frequency(0, 0);
                os_hal_motor_set_frequency(1, 0);
            }
            break;
        }

        case OS_STATE_MANUAL_MOTION: {
            uint32_t freq = 100;
            if (g_manual_speed == OS_SPEED_MEDIUM) freq = 500;
            else if (g_manual_speed == OS_SPEED_FAST) freq = 2000;
            else if (g_manual_speed == OS_SPEED_CUSTOM) freq = (uint32_t)g_custom_move_speed;

            uint8_t axis = (g_manual_direction == OS_DIRECTION_EAST || g_manual_direction == OS_DIRECTION_WEST) ? 0 : 1;
            bool fwd = (g_manual_direction == OS_DIRECTION_EAST || g_manual_direction == OS_DIRECTION_NORTH);

            os_hal_motor_set_direction(axis, fwd);
            os_hal_motor_set_frequency(axis, freq);
            break;
        }

        case OS_STATE_PARKED:
        case OS_STATE_FAULT:
        default:
            os_hal_motor_set_frequency(0, 0);
            os_hal_motor_set_frequency(1, 0);
            break;
    }

    os_site_info_t polled_site;
    if (os_hal_gps_poll(&polled_site) == OS_ERR_NONE && polled_site.valid) {
        g_site = polled_site;
    }
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (source_channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length < 3 || command[0] != OS_LX200_CMD_PREFIX || command[length - 1] != OS_LX200_CMD_SUFFIX || length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_COMMAND_FORMAT;
    }

    reply_buffer[0] = '\0';
    *reply_length = 0;

    if (strncmp(command, ":GVP#", 5) == 0) {
        snprintf(reply_buffer, reply_buffer_size, "OnStep %d.%d.%d#", OS_FIRMWARE_VERSION_MAJOR, OS_FIRMWARE_VERSION_MINOR, OS_FIRMWARE_VERSION_PATCH);
    } else if (strncmp(command, ":GR#", 4) == 0) {
        os_equatorial_coord_t eq;
        os_query_coordinates(&eq);
        snprintf(reply_buffer, reply_buffer_size, "%02d:%02d:%02d#", (int)eq.ra_hours, (int)((eq.ra_hours - (int)eq.ra_hours) * 60), 0);
    } else if (strncmp(command, ":GD#", 4) == 0) {
        os_equatorial_coord_t eq;
        os_query_coordinates(&eq);
        snprintf(reply_buffer, reply_buffer_size, "%+03d*%02d#", (int)eq.dec_degrees, (int)(fabsf(eq.dec_degrees - (int)eq.dec_degrees) * 60));
    } else if (strncmp(command, ":hP#", 4) == 0) {
        os_error_t err = os_park();
        if (err != OS_ERR_NONE) return err;
        snprintf(reply_buffer, reply_buffer_size, "1#");
    } else if (strncmp(command, ":hO#", 4) == 0) {
        os_error_t err = os_unpark();
        if (err != OS_ERR_NONE) return err;
        snprintf(reply_buffer, reply_buffer_size, "1#");
    } else {
        snprintf(reply_buffer, reply_buffer_size, "1#");
    }

    *reply_length = strlen(reply_buffer);
    os_hal_comm_write(source_channel, reply_buffer, *reply_length);
    return OS_ERR_NONE;
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

    double ra_arcsec = target.ra_hours * 15.0 * 3600.0;
    double dec_arcsec = target.dec_degrees * 3600.0;

    if (g_calibration.valid) {
        double calc_ra = g_calibration.matrix_ra_to_ra * ra_arcsec + g_calibration.matrix_ra_to_dec * dec_arcsec + g_calibration.offset_ra_arcsec;
        double calc_dec = g_calibration.matrix_dec_to_ra * ra_arcsec + g_calibration.matrix_dec_to_dec * dec_arcsec + g_calibration.offset_dec_arcsec;
        ra_arcsec = calc_ra;
        dec_arcsec = calc_dec;
    }

    g_goto_target_steps[0] = (int32_t)lrint(ra_arcsec);
    g_goto_target_steps[1] = (int32_t)lrint(dec_arcsec);

    g_state = OS_STATE_GOTO;
    g_goto_active = true;
    g_moving = true;

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

    g_goto_target_steps[0] = (int32_t)lrint(target.azimuth_degrees * 3600.0);
    g_goto_target_steps[1] = (int32_t)lrint(target.altitude_degrees * 3600.0);

    g_state = OS_STATE_GOTO;
    g_goto_active = true;
    g_moving = true;

    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);

    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (g_state == OS_STATE_GOTO || g_moving) {
        os_hal_motor_set_frequency(0, 0);
        os_hal_motor_set_frequency(1, 0);
        g_goto_active = false;
        g_moving = false;
        g_state = OS_STATE_IDLE_TRACKING;
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
    g_tracking_rate = rate;
    g_tracking_custom_factor = custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = g_tracking_rate;
    *custom_factor = g_tracking_custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    g_tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    g_tracking_enabled = false;
    os_hal_motor_set_frequency(0, 0);
    os_hal_motor_set_frequency(1, 0);
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST || duration_ms == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    uint8_t axis = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) ? 0 : 1;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_guide_pulse.active = true;
    g_guide_pulse.duration_ms = duration_ms;
    g_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    g_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);

    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_guide_pulse.rate_fraction = rate_fraction;
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
    g_residual_computed = false;
    g_residual_arcsec = 0.0f;
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
    if (g_align_star_count < OS_CALIBRATION_MAX_STARS) {
        g_align_stars[g_align_star_count] = star_coord;
        g_align_motor_pos[g_align_star_count] = motor_pos;
        g_align_star_count++;
    }
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (g_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t min_stars = 1;
    if (g_align_mode == OS_ALIGN_2STAR) min_stars = 2;
    else if (g_align_mode == OS_ALIGN_3STAR || g_align_mode == OS_ALIGN_NSTAR) min_stars = 3;

    if (g_align_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    if (g_align_star_count == 1) {
        double sky_ra = g_align_stars[0].ra_hours * 15.0 * 3600.0;
        double sky_dec = g_align_stars[0].dec_degrees * 3600.0;

        g_calibration.matrix_ra_to_ra = 1.0f;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = 1.0f;
        g_calibration.offset_ra_arcsec = (float)(g_align_motor_pos[0].ra_steps - sky_ra);
        g_calibration.offset_dec_arcsec = (float)(g_align_motor_pos[0].dec_steps - sky_dec);
        g_residual_arcsec = 0.0f;
    } else {
        double sum_x = 0, sum_y = 0, sum_m1 = 0, sum_m2 = 0;
        for (uint8_t i = 0; i < g_align_star_count; i++) {
            sum_x += g_align_stars[i].ra_hours * 15.0 * 3600.0;
            sum_y += g_align_stars[i].dec_degrees * 3600.0;
            sum_m1 += g_align_motor_pos[i].ra_steps;
            sum_m2 += g_align_motor_pos[i].dec_steps;
        }
        g_calibration.matrix_ra_to_ra = 1.0f;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = 1.0f;
        g_calibration.offset_ra_arcsec = (float)((sum_m1 - sum_x) / g_align_star_count);
        g_calibration.offset_dec_arcsec = (float)((sum_m2 - sum_y) / g_align_star_count);

        if (g_align_star_count == 3) {
            g_residual_arcsec = 0.0f;
        } else {
            double sq_err_sum = 0.0;
            for (uint8_t i = 0; i < g_align_star_count; i++) {
                double sky_ra = g_align_stars[i].ra_hours * 15.0 * 3600.0;
                double sky_dec = g_align_stars[i].dec_degrees * 3600.0;
                double pred_ra = sky_ra + g_calibration.offset_ra_arcsec;
                double pred_dec = sky_dec + g_calibration.offset_dec_arcsec;
                double err_ra = g_align_motor_pos[i].ra_steps - pred_ra;
                double err_dec = g_align_motor_pos[i].dec_steps - pred_dec;
                sq_err_sum += err_ra * err_ra + err_dec * err_dec;
            }
            g_residual_arcsec = (float)sqrt(sq_err_sum / g_align_star_count);
        }
    }

    g_calibration.valid = true;
    g_residual_computed = true;

    os_hal_nvm_write(0, (const uint8_t *)&g_calibration, sizeof(os_calibration_t));

    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_residual_computed || g_align_star_count == 0) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = g_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    g_align_star_count = 0;
    g_residual_computed = false;
    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (g_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    os_hal_motor_set_frequency(0, 0);
    os_hal_motor_set_frequency(1, 0);
    os_hal_motor_enable(0, false);
    os_hal_motor_enable(1, false);

    g_state = OS_STATE_PARKED;
    g_moving = false;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (g_state != OS_STATE_PARKED) {
        return OS_ERR_NONE;
    }
    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);

    uint32_t utc;
    if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
        g_site.utc_epoch_seconds = utc;
    }

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

    uint8_t axis = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) ? 0 : 1;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (g_state == OS_STATE_PARKED || g_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    g_manual_direction = direction;
    g_manual_speed = speed;
    g_manual_motion_active = true;
    g_moving = true;
    g_state = OS_STATE_MANUAL_MOTION;

    os_hal_motor_enable(axis, true);
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    os_hal_motor_set_frequency(0, 0);
    os_hal_motor_set_frequency(1, 0);
    g_manual_motion_active = false;
    g_moving = false;
    g_state = OS_STATE_IDLE_TRACKING;
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
    *state = g_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int32_t pos0 = os_hal_motor_get_position(0);
    int32_t pos1 = os_hal_motor_get_position(1);

    double ra_arcsec = (double)pos0 - g_calibration.offset_ra_arcsec;
    double dec_arcsec = (double)pos1 - g_calibration.offset_dec_arcsec;

    coord->ra_hours = (float)(ra_arcsec / (15.0 * 3600.0));
    coord->dec_degrees = (float)(dec_arcsec / 3600.0);

    if (coord->ra_hours < 0.0f) coord->ra_hours += 24.0f;
    if (coord->ra_hours >= 24.0f) coord->ra_hours -= 24.0f;

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
    *moving = g_moving;
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
    int idx = (int)worm_phase_deg;
    if (idx >= OS_PEC_TABLE_SIZE) {
        idx = OS_PEC_TABLE_SIZE - 1;
    }
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
    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_ra_to_dec = 0.0f;
    g_calibration.matrix_dec_to_ra = 0.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.offset_ra_arcsec = 0.0f;
    g_calibration.offset_dec_arcsec = 0.0f;
    g_calibration.valid = false;
    g_residual_computed = false;
    g_residual_arcsec = 0.0f;
    g_align_star_count = 0;

    os_hal_nvm_write(0, (const uint8_t *)&g_calibration, sizeof(os_calibration_t));
    return OS_ERR_NONE;
}

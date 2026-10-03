/* Implementation of the OnStep library core functionality. */
#include "6_generated_code.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define STEPS_PER_DEGREE_RA   3600.0
#define STEPS_PER_DEGREE_DEC  3600.0
#define ARCSEC_PER_DEGREE     3600.0

typedef struct {
    os_equatorial_coord_t star;
    os_motor_position_t motor;
} os_align_star_t;

static os_state_t g_system_state = OS_STATE_INITIALIZING;
static os_track_rate_t g_track_rate = OS_TRACK_RATE_SIDEREAL;
static float g_custom_track_factor = 1.0f;
static bool g_tracking_enabled = true;

static os_guide_pulse_t g_guide_pulse_state = { false, 0, 0.5f, false, false, false };
static float g_guide_rate_fraction = 0.5f;

static os_align_mode_t g_align_mode = OS_ALIGN_1STAR;
static os_align_star_t g_align_stars[OS_CALIBRATION_MAX_STARS];
static uint8_t g_align_star_count = 0;
static bool g_align_in_progress = false;

static os_calibration_t g_calibration = { 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, false };
static float g_calibration_residual_arcsec = 0.0f;
static bool g_calibration_residual_calculated = false;

static os_equatorial_coord_t g_park_position = { 0.0f, 90.0f };
static bool g_goto_active = false;
static os_equatorial_coord_t g_target_equatorial = { 0.0f, 0.0f };
static int32_t g_target_steps_ra = 0;
static int32_t g_target_steps_dec = 0;

static bool g_manual_moving = false;
static os_direction_t g_manual_direction = OS_DIRECTION_NORTH;
static os_speed_level_t g_manual_speed = OS_SPEED_SLOW;
static float g_custom_manual_speed_arcsec = 15.0f;

static os_pec_table_t g_pec_table = { {0}, false };
static bool g_pec_enabled = false;

static os_site_info_t g_current_site = { 0.0f, 0.0f, 0.0f, 0, false };
static os_equatorial_coord_t g_target_set_coord = { 0.0f, 0.0f };

static os_error_t coord_to_steps(os_equatorial_coord_t eq, int32_t *ra_steps, int32_t *dec_steps) {
    if (!ra_steps || !dec_steps) return OS_ERR_INVALID_ARGUMENT;
    double ra_deg = (double)eq.ra_hours * 15.0;
    double dec_deg = (double)eq.dec_degrees;
    
    if (g_calibration.valid) {
        double ra_arcsec = ra_deg * 3600.0;
        double dec_arcsec = dec_deg * 3600.0;
        double corr_ra = (double)g_calibration.matrix_ra_to_ra * ra_arcsec +
                         (double)g_calibration.matrix_ra_to_dec * dec_arcsec +
                         (double)g_calibration.offset_ra_arcsec;
        double corr_dec = (double)g_calibration.matrix_dec_to_ra * ra_arcsec +
                          (double)g_calibration.matrix_dec_to_dec * dec_arcsec +
                          (double)g_calibration.offset_dec_arcsec;
        ra_deg = corr_ra / 3600.0;
        dec_deg = corr_dec / 3600.0;
    }

    *ra_steps = (int32_t)lround(ra_deg * STEPS_PER_DEGREE_RA);
    *dec_steps = (int32_t)lround(dec_deg * STEPS_PER_DEGREE_DEC);
    return OS_ERR_NONE;
}

static os_error_t steps_to_coord(int32_t ra_steps, int32_t dec_steps, os_equatorial_coord_t *eq) {
    if (!eq) return OS_ERR_INVALID_ARGUMENT;
    double ra_deg = (double)ra_steps / STEPS_PER_DEGREE_RA;
    double dec_deg = (double)dec_steps / STEPS_PER_DEGREE_DEC;
    
    if (g_calibration.valid) {
        double ra_arcsec = ra_deg * 3600.0 - (double)g_calibration.offset_ra_arcsec;
        double dec_arcsec = dec_deg * 3600.0 - (double)g_calibration.offset_dec_arcsec;
        double det = (double)g_calibration.matrix_ra_to_ra * (double)g_calibration.matrix_dec_to_dec -
                     (double)g_calibration.matrix_ra_to_dec * (double)g_calibration.matrix_dec_to_ra;
        if (fabs(det) > 1e-9) {
            double inv_ra = ((double)g_calibration.matrix_dec_to_dec * ra_arcsec - (double)g_calibration.matrix_ra_to_dec * dec_arcsec) / det;
            double inv_dec = (-(double)g_calibration.matrix_dec_to_ra * ra_arcsec + (double)g_calibration.matrix_ra_to_ra * dec_arcsec) / det;
            ra_deg = inv_ra / 3600.0;
            dec_deg = inv_dec / 3600.0;
        }
    }

    double ra_hours = ra_deg / 15.0;
    while (ra_hours < 0.0) ra_hours += 24.0;
    while (ra_hours >= 24.0) ra_hours -= 24.0;

    if (dec_deg > 90.0) dec_deg = 90.0;
    if (dec_deg < -90.0) dec_deg = -90.0;

    eq->ra_hours = (float)ra_hours;
    eq->dec_degrees = (float)dec_deg;
    return OS_ERR_NONE;
}

os_error_t os_init(void) {
    g_calibration_residual_calculated = false;
    g_calibration_residual_arcsec = 0.0f;
    g_align_star_count = 0;
    g_align_in_progress = false;
    g_guide_pulse_state.active = false;
    g_guide_pulse_state.duration_ms = 0;
    g_manual_moving = false;
    g_goto_active = false;
    g_tracking_enabled = true;
    g_track_rate = OS_TRACK_RATE_SIDEREAL;
    g_custom_track_factor = 1.0f;

    os_hal_nvm_init();
    os_hal_motor_init(0);
    os_hal_motor_init(1);
    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        os_hal_comm_init(ch);
    }
    os_hal_gps_init();
    os_hal_rtc_init();
    os_hal_limit_init();
    os_hal_timer_motor_init();

    g_calibration.valid = false;
    if (os_hal_nvm_read(0, (uint8_t*)&g_calibration, sizeof(os_calibration_t)) == OS_ERR_NONE) {
        if (!g_calibration.valid) {
            g_calibration.matrix_ra_to_ra = 1.0f;
            g_calibration.matrix_ra_to_dec = 0.0f;
            g_calibration.matrix_dec_to_ra = 0.0f;
            g_calibration.matrix_dec_to_dec = 1.0f;
            g_calibration.offset_ra_arcsec = 0.0f;
            g_calibration.offset_dec_arcsec = 0.0f;
        }
    } else {
        g_calibration.matrix_ra_to_ra = 1.0f;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = 1.0f;
        g_calibration.offset_ra_arcsec = 0.0f;
        g_calibration.offset_dec_arcsec = 0.0f;
        g_calibration.valid = false;
    }

    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        os_hal_motor_set_frequency(0, 0);
        os_hal_motor_set_frequency(1, 0);
        g_goto_active = false;
        g_manual_moving = false;
        g_system_state = OS_STATE_FAULT;
        return;
    }

    if (g_system_state == OS_STATE_FAULT) {
        return;
    }

    os_site_info_t site_poll;
    if (os_hal_gps_poll(&site_poll) == OS_ERR_NONE && site_poll.valid) {
        g_current_site = site_poll;
    } else {
        uint32_t rtc_sec = 0;
        if (os_hal_rtc_read(&rtc_sec) == OS_ERR_NONE) {
            g_current_site.utc_epoch_seconds = rtc_sec;
        }
    }

    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        int16_t avail = os_hal_comm_available(ch);
        if (avail > 0) {
            static char cmd_buf[OS_MAX_COMMAND_LENGTH];
            static size_t buf_pos = 0;
            while (os_hal_comm_available(ch) > 0 && buf_pos < OS_MAX_COMMAND_LENGTH - 1) {
                char c = os_hal_comm_read(ch);
                cmd_buf[buf_pos++] = c;
                if (c == OS_LX200_CMD_SUFFIX || c == '\n' || c == '\r') {
                    cmd_buf[buf_pos] = '\0';
                    char reply[OS_MAX_REPLY_LENGTH];
                    size_t reply_len = 0;
                    if (os_command_parse(cmd_buf, buf_pos, ch, reply, sizeof(reply), &reply_len) == OS_ERR_NONE && reply_len > 0) {
                        os_hal_comm_write(ch, reply, reply_len);
                    }
                    buf_pos = 0;
                    break;
                }
            }
        }
    }

    if (g_goto_active && g_system_state == OS_STATE_GOTO) {
        int32_t cur_ra = os_hal_motor_get_position(0);
        int32_t cur_dec = os_hal_motor_get_position(1);
        int32_t diff_ra = g_target_steps_ra - cur_ra;
        int32_t diff_dec = g_target_steps_dec - cur_dec;

        if (abs(diff_ra) < 10 && abs(diff_dec) < 10) {
            g_goto_active = false;
            os_hal_motor_set_frequency(0, 0);
            os_hal_motor_set_frequency(1, 0);
            os_hal_buzzer_beep(100, 1);
            g_system_state = OS_STATE_IDLE_TRACKING;
        } else {
            os_hal_motor_enable(0, true);
            os_hal_motor_enable(1, true);
            os_hal_motor_set_direction(0, diff_ra >= 0);
            os_hal_motor_set_direction(1, diff_dec >= 0);
            os_hal_motor_set_frequency(0, 1000);
            os_hal_motor_set_frequency(1, 1000);
        }
    }

    if (g_guide_pulse_state.active) {
        if (g_guide_pulse_state.duration_ms > 10) {
            g_guide_pulse_state.duration_ms -= 10;
        } else {
            g_guide_pulse_state.active = false;
            g_guide_pulse_state.duration_ms = 0;
        }
    }
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (!command || !reply_buffer || !reply_length) return OS_ERR_INVALID_ARGUMENT;
    if (source_channel > OS_CHANNEL_ETHERNET) return OS_ERR_INVALID_ARGUMENT;
    if (length == 0 || reply_buffer_size == 0) return OS_ERR_INVALID_ARGUMENT;

    if (command[0] != OS_LX200_CMD_PREFIX) return OS_ERR_COMMAND_FORMAT;

    const char *end = strchr(command, OS_LX200_CMD_SUFFIX);
    if (!end) return OS_ERR_COMMAND_FORMAT;

    reply_buffer[0] = '\0';
    *reply_length = 0;

    if (strncmp(command, ":GR#", 4) == 0) {
        os_equatorial_coord_t eq;
        os_query_coordinates(&eq);
        int hrs = (int)eq.ra_hours;
        int mins = (int)((eq.ra_hours - hrs) * 60.0f);
        int secs = (int)(((eq.ra_hours - hrs) * 60.0f - mins) * 60.0f);
        int len = snprintf(reply_buffer, reply_buffer_size, "%02d:%02d:%02d#", hrs, mins, secs);
        *reply_length = (len > 0) ? (size_t)len : 0;
        return OS_ERR_NONE;
    }
    
    if (strncmp(command, ":GD#", 4) == 0) {
        os_equatorial_coord_t eq;
        os_query_coordinates(&eq);
        char sign = (eq.dec_degrees >= 0.0f) ? '+' : '-';
        float abs_dec = fabsf(eq.dec_degrees);
        int degs = (int)abs_dec;
        int mins = (int)((abs_dec - degs) * 60.0f);
        int len = snprintf(reply_buffer, reply_buffer_size, "%c%02d*%02d#", sign, degs, mins);
        *reply_length = (len > 0) ? (size_t)len : 0;
        return OS_ERR_NONE;
    }

    if (strncmp(command, ":GVP#", 5) == 0) {
        int len = snprintf(reply_buffer, reply_buffer_size, "OnStep#");
        *reply_length = (len > 0) ? (size_t)len : 0;
        return OS_ERR_NONE;
    }

    if (strncmp(command, ":GVN#", 5) == 0) {
        int len = snprintf(reply_buffer, reply_buffer_size, "%d.%d.%d#",
                           OS_FIRMWARE_VERSION_MAJOR, OS_FIRMWARE_VERSION_MINOR, OS_FIRMWARE_VERSION_PATCH);
        *reply_length = (len > 0) ? (size_t)len : 0;
        return OS_ERR_NONE;
    }

    if (strncmp(command, ":MS#", 4) == 0) {
        os_error_t err = os_goto_equatorial(g_target_set_coord);
        if (err == OS_ERR_NONE) {
            int len = snprintf(reply_buffer, reply_buffer_size, "0");
            *reply_length = (len > 0) ? (size_t)len : 0;
        } else {
            int len = snprintf(reply_buffer, reply_buffer_size, "1");
            *reply_length = (len > 0) ? (size_t)len : 0;
        }
        return OS_ERR_NONE;
    }

    if (strncmp(command, ":Q#", 3) == 0) {
        os_goto_abort();
        os_move_stop();
        *reply_length = 0;
        return OS_ERR_NONE;
    }

    if (strncmp(command, ":hP#", 4) == 0) {
        os_park();
        int len = snprintf(reply_buffer, reply_buffer_size, "1");
        *reply_length = (len > 0) ? (size_t)len : 0;
        return OS_ERR_NONE;
    }

    if (strncmp(command, ":hO#", 4) == 0) {
        os_unpark();
        int len = snprintf(reply_buffer, reply_buffer_size, "1");
        *reply_length = (len > 0) ? (size_t)len : 0;
        return OS_ERR_NONE;
    }

    int len = snprintf(reply_buffer, reply_buffer_size, "0#");
    *reply_length = (len > 0) ? (size_t)len : 0;
    return OS_ERR_NONE;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS ||
        target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_target_equatorial = target;
    coord_to_steps(target, &g_target_steps_ra, &g_target_steps_dec);

    g_system_state = OS_STATE_GOTO;
    g_goto_active = true;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_system_state = OS_STATE_GOTO;
    g_goto_active = true;
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    g_goto_active = false;
    os_hal_motor_set_frequency(0, 0);
    os_hal_motor_set_frequency(1, 0);
    g_system_state = OS_STATE_IDLE_TRACKING;
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
    g_custom_track_factor = custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (!rate || !custom_factor) return OS_ERR_INVALID_ARGUMENT;
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
    if (duration_ms == 0) return OS_ERR_INVALID_ARGUMENT;
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_guide_pulse_state.active = true;
    g_guide_pulse_state.duration_ms = duration_ms;
    g_guide_pulse_state.rate_fraction = g_guide_rate_fraction;
    g_guide_pulse_state.direction_east = (direction == OS_DIRECTION_EAST);
    g_guide_pulse_state.direction_north = (direction == OS_DIRECTION_NORTH);
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
    if (!pulse) return OS_ERR_INVALID_ARGUMENT;
    *pulse = g_guide_pulse_state;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) return OS_ERR_INVALID_ARGUMENT;
    g_align_mode = mode;
    g_align_star_count = 0;
    g_align_in_progress = true;
    g_system_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord, os_motor_position_t motor_pos) {
    if (star_coord.ra_hours < OS_RA_MIN_HOURS || star_coord.ra_hours > OS_RA_MAX_HOURS ||
        star_coord.dec_degrees < OS_DEC_MIN_DEG || star_coord.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_align_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }
    g_align_stars[g_align_star_count].star = star_coord;
    g_align_stars[g_align_star_count].motor = motor_pos;
    g_align_star_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    uint8_t min_stars = 1;
    if (g_align_mode == OS_ALIGN_1STAR) min_stars = 1;
    else if (g_align_mode == OS_ALIGN_2STAR) min_stars = 2;
    else if (g_align_mode == OS_ALIGN_3STAR) min_stars = 3;
    else if (g_align_mode == OS_ALIGN_NSTAR) min_stars = 3;

    if (g_align_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    double sum_ra_star = 0.0, sum_dec_star = 0.0;
    double sum_ra_mot = 0.0, sum_dec_mot = 0.0;

    for (uint8_t i = 0; i < g_align_star_count; i++) {
        sum_ra_star += (double)g_align_stars[i].star.ra_hours * 15.0 * ARCSEC_PER_DEGREE;
        sum_dec_star += (double)g_align_stars[i].star.dec_degrees * ARCSEC_PER_DEGREE;
        sum_ra_mot += (double)g_align_stars[i].motor.ra_steps / STEPS_PER_DEGREE_RA * ARCSEC_PER_DEGREE;
        sum_dec_mot += (double)g_align_stars[i].motor.dec_steps / STEPS_PER_DEGREE_DEC * ARCSEC_PER_DEGREE;
    }

    double avg_ra_star = sum_ra_star / g_align_star_count;
    double avg_dec_star = sum_dec_star / g_align_star_count;
    double avg_ra_mot = sum_ra_mot / g_align_star_count;
    double avg_dec_mot = sum_dec_mot / g_align_star_count;

    if (g_align_mode == OS_ALIGN_1STAR) {
        g_calibration.matrix_ra_to_ra = 1.0f;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = 1.0f;
        g_calibration.offset_ra_arcsec = (float)(avg_ra_mot - avg_ra_star);
        g_calibration.offset_dec_arcsec = (float)(avg_dec_mot - avg_dec_star);
        g_calibration_residual_arcsec = 0.0f;
    } else if (g_align_mode == OS_ALIGN_2STAR) {
        double sxx = 0.0, syy = 0.0;
        double sx_rx = 0.0, sy_ry = 0.0;

        for (uint8_t i = 0; i < g_align_star_count; i++) {
            double x = (double)g_align_stars[i].star.ra_hours * 15.0 * ARCSEC_PER_DEGREE - avg_ra_star;
            double y = (double)g_align_stars[i].star.dec_degrees * ARCSEC_PER_DEGREE - avg_dec_star;
            double rx = (double)g_align_stars[i].motor.ra_steps / STEPS_PER_DEGREE_RA * ARCSEC_PER_DEGREE - avg_ra_mot;
            double ry = (double)g_align_stars[i].motor.dec_steps / STEPS_PER_DEGREE_DEC * ARCSEC_PER_DEGREE - avg_dec_mot;

            sxx += x * x;
            syy += y * y;
            sx_rx += x * rx;
            sy_ry += y * ry;
        }

        double m_ra_ra = (fabs(sxx) > 1e-9) ? (sx_rx / sxx) : 1.0;
        double m_dec_dec = (fabs(syy) > 1e-9) ? (sy_ry / syy) : 1.0;

        g_calibration.matrix_ra_to_ra = (float)m_ra_ra;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = (float)m_dec_dec;
        g_calibration.offset_ra_arcsec = (float)(avg_ra_mot - m_ra_ra * avg_ra_star);
        g_calibration.offset_dec_arcsec = (float)(avg_dec_mot - m_dec_dec * avg_dec_star);
        g_calibration_residual_arcsec = 0.0f;
    } else {
        double sxx = 0.0, sxy = 0.0, syy = 0.0;
        double sx_rx = 0.0, sx_ry = 0.0, sy_rx = 0.0, sy_ry = 0.0;

        for (uint8_t i = 0; i < g_align_star_count; i++) {
            double x = (double)g_align_stars[i].star.ra_hours * 15.0 * ARCSEC_PER_DEGREE - avg_ra_star;
            double y = (double)g_align_stars[i].star.dec_degrees * ARCSEC_PER_DEGREE - avg_dec_star;
            double rx = (double)g_align_stars[i].motor.ra_steps / STEPS_PER_DEGREE_RA * ARCSEC_PER_DEGREE - avg_ra_mot;
            double ry = (double)g_align_stars[i].motor.dec_steps / STEPS_PER_DEGREE_DEC * ARCSEC_PER_DEGREE - avg_dec_mot;

            sxx += x * x;
            sxy += x * y;
            syy += y * y;
            sx_rx += x * rx;
            sx_ry += x * ry;
            sy_rx += y * rx;
            sy_ry += y * ry;
        }

        double det = sxx * syy - sxy * sxy;
        if (fabs(det) < 1e-9) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        double m_ra_ra = (sx_rx * syy - sy_rx * sxy) / det;
        double m_ra_dec = (sy_rx * sxx - sx_rx * sxy) / det;
        double m_dec_ra = (sx_ry * syy - sy_ry * sxy) / det;
        double m_dec_dec = (sy_ry * sxx - sx_ry * sxy) / det;

        g_calibration.matrix_ra_to_ra = (float)m_ra_ra;
        g_calibration.matrix_ra_to_dec = (float)m_ra_dec;
        g_calibration.matrix_dec_to_ra = (float)m_dec_ra;
        g_calibration.matrix_dec_to_dec = (float)m_dec_dec;
        g_calibration.offset_ra_arcsec = (float)(avg_ra_mot - (m_ra_ra * avg_ra_star + m_ra_dec * avg_dec_star));
        g_calibration.offset_dec_arcsec = (float)(avg_dec_mot - (m_dec_ra * avg_ra_star + m_dec_dec * avg_dec_star));

        if (g_align_star_count == 3) {
            g_calibration_residual_arcsec = 0.0f;
        } else {
            double res_sum = 0.0;
            for (uint8_t i = 0; i < g_align_star_count; i++) {
                double star_ra = (double)g_align_stars[i].star.ra_hours * 15.0 * ARCSEC_PER_DEGREE;
                double star_dec = (double)g_align_stars[i].star.dec_degrees * ARCSEC_PER_DEGREE;
                double mot_ra = (double)g_align_stars[i].motor.ra_steps / STEPS_PER_DEGREE_RA * ARCSEC_PER_DEGREE;
                double mot_dec = (double)g_align_stars[i].motor.dec_steps / STEPS_PER_DEGREE_DEC * ARCSEC_PER_DEGREE;

                double pred_ra = m_ra_ra * star_ra + m_ra_dec * star_dec + g_calibration.offset_ra_arcsec;
                double pred_dec = m_dec_ra * star_ra + m_dec_dec * star_dec + g_calibration.offset_dec_arcsec;

                double err_ra = mot_ra - pred_ra;
                double err_dec = mot_dec - pred_dec;
                res_sum += sqrt(err_ra * err_ra + err_dec * err_dec);
            }
            g_calibration_residual_arcsec = (float)(res_sum / g_align_star_count);
        }
    }

    g_calibration.valid = true;
    g_calibration_residual_calculated = true;
    g_align_in_progress = false;

    os_hal_nvm_write(0, (const uint8_t*)&g_calibration, sizeof(os_calibration_t));
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (!residual_arcsec) return OS_ERR_INVALID_ARGUMENT;
    *residual_arcsec = g_calibration_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    g_align_in_progress = false;
    g_align_star_count = 0;
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    coord_to_steps(g_park_position, &g_target_steps_ra, &g_target_steps_dec);
    os_hal_motor_set_frequency(0, 0);
    os_hal_motor_set_frequency(1, 0);
    os_hal_motor_enable(0, false);
    os_hal_motor_enable(1, false);

    g_tracking_enabled = false;
    g_system_state = OS_STATE_PARKED;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);
    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        os_hal_comm_init(ch);
    }
    uint32_t rtc_sec = 0;
    if (os_hal_rtc_read(&rtc_sec) == OS_ERR_NONE) {
        g_current_site.utc_epoch_seconds = rtc_sec;
    }
    g_tracking_enabled = true;
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (park_pos.ra_hours < OS_RA_MIN_HOURS || park_pos.ra_hours > OS_RA_MAX_HOURS ||
        park_pos.dec_degrees < OS_DEC_MIN_DEG || park_pos.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_park_position = park_pos;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_manual_moving = true;
    g_manual_direction = direction;
    g_manual_speed = speed;

    uint8_t axis = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) ? 0 : 1;
    bool fwd = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_NORTH);
    os_hal_motor_set_direction(axis, fwd);

    uint32_t freq = 100;
    if (speed == OS_SPEED_MEDIUM) freq = 500;
    else if (speed == OS_SPEED_FAST) freq = 2000;
    else if (speed == OS_SPEED_CUSTOM) freq = (uint32_t)(g_custom_manual_speed_arcsec * 10.0f);

    os_hal_motor_enable(axis, true);
    os_hal_motor_set_frequency(axis, freq);

    g_system_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    g_manual_moving = false;
    os_hal_motor_set_frequency(0, 0);
    os_hal_motor_set_frequency(1, 0);
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (arcsec_per_sec <= 0.0f) return OS_ERR_INVALID_ARGUMENT;
    g_custom_manual_speed_arcsec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (!state) return OS_ERR_INVALID_ARGUMENT;
    *state = g_system_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (!coord) return OS_ERR_INVALID_ARGUMENT;
    int32_t ra_steps = os_hal_motor_get_position(0);
    int32_t dec_steps = os_hal_motor_get_position(1);
    return steps_to_coord(ra_steps, dec_steps, coord);
}

os_error_t os_query_site(os_site_info_t *site) {
    if (!site) return OS_ERR_INVALID_ARGUMENT;
    *site = g_current_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (!pos) return OS_ERR_INVALID_ARGUMENT;
    pos->ra_steps = os_hal_motor_get_position(0);
    pos->dec_steps = os_hal_motor_get_position(1);
    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor, uint8_t *patch) {
    if (!major || !minor || !patch) return OS_ERR_INVALID_ARGUMENT;
    *major = OS_FIRMWARE_VERSION_MAJOR;
    *minor = OS_FIRMWARE_VERSION_MINOR;
    *patch = OS_FIRMWARE_VERSION_PATCH;
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *moving) {
    if (!moving) return OS_ERR_INVALID_ARGUMENT;
    *moving = g_goto_active || g_manual_moving;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (!locked) return OS_ERR_INVALID_ARGUMENT;
    *locked = g_current_site.valid;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    g_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (!table) return OS_ERR_INVALID_ARGUMENT;
    g_pec_table = *table;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (!table) return OS_ERR_INVALID_ARGUMENT;
    *table = g_pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    uint16_t index = (uint16_t)lround(worm_phase_deg);
    if (index >= 360) index = 0;
    g_pec_table.corrections[index] = error_arcsec;
    g_pec_table.valid = true;
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (!calib) return OS_ERR_INVALID_ARGUMENT;
    *calib = g_calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    g_calibration.valid = false;
    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_ra_to_dec = 0.0f;
    g_calibration.matrix_dec_to_ra = 0.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.offset_ra_arcsec = 0.0f;
    g_calibration.offset_dec_arcsec = 0.0f;
    g_calibration_residual_calculated = false;
    g_calibration_residual_arcsec = 0.0f;
    return OS_ERR_NONE;
}

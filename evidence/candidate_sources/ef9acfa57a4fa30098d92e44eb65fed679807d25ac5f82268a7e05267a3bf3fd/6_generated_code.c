#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define STEPS_PER_ARCSEC 1.0
#define NVM_TOTAL_SIZE 1024
#define NVM_OFFSET_CALIBRATION 0
#define NVM_OFFSET_PEC 256
#define NVM_OFFSET_PARK 640

static os_state_t g_system_state = OS_STATE_INITIALIZING;
static int32_t g_ra_position = 0;
static int32_t g_dec_position = 0;
static int32_t g_target_ra_steps = 0;
static int32_t g_target_dec_steps = 0;
static bool g_tracking_enabled = true;
static os_track_rate_t g_tracking_rate = OS_TRACK_RATE_SIDEREAL;
static float g_custom_track_factor = 1.0f;
static bool g_is_moving = false;
static bool g_goto_active = false;
static os_site_info_t g_site_info = {0.0f, 0.0f, 0.0f, 0, false};
static os_calibration_t g_calibration = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, false};
static os_align_mode_t g_align_mode = OS_ALIGN_1STAR;
static uint8_t g_align_star_count = 0;
static os_equatorial_coord_t g_align_stars_eq[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t g_align_stars_motor[OS_CALIBRATION_MAX_STARS];
static float g_align_residual_arcsec = 0.0f;
static bool g_align_residual_valid = false;
static os_guide_pulse_t g_guide_pulse = {false, 0, 0.5f, false, false, false};
static bool g_manual_move_active = false;
static os_direction_t g_manual_dir = OS_DIRECTION_NORTH;
static os_speed_level_t g_manual_speed = OS_SPEED_MEDIUM;
static float g_custom_manual_speed = 15.0f;
static os_equatorial_coord_t g_park_pos = {0.0f, 90.0f};
static os_pec_table_t g_pec_table = {{0}, false};
static bool g_pec_enabled = false;
static float g_worm_phase_deg = 0.0f;
static os_equatorial_coord_t g_target_equatorial = {0.0f, 0.0f};

static uint32_t g_hal_motor_freq[2] = {0, 0};
static bool g_hal_motor_dir[2] = {true, true};
static bool g_hal_motor_enabled[2] = {false, false};
static bool g_hal_motor_init_done[2] = {false, false};
static uint32_t g_hal_rtc_epoch = 1700000000;
static bool g_hal_limit_triggered[2] = {false, false};
static uint8_t g_hal_nvm[NVM_TOTAL_SIZE];
static bool g_hal_nvm_init_done = false;
static char g_hal_comm_rx[4][128];
static size_t g_hal_comm_rx_head[4] = {0};
static size_t g_hal_comm_rx_tail[4] = {0};
static char g_hal_comm_tx[4][128];
static size_t g_hal_comm_tx_head[4] = {0};
static size_t g_hal_comm_tx_tail[4] = {0};
static bool g_hal_comm_init_done[4] = {false, false, false, false};

static void os_helper_celestial_to_steps(float ra_hours, float dec_degrees, int32_t *ra_steps, int32_t *dec_steps) {
    double ra_arcsec = (double)ra_hours * 54000.0;
    double dec_arcsec = (double)dec_degrees * 3600.0;
    double cal_ra = ra_arcsec;
    double cal_dec = dec_arcsec;
    if (g_calibration.valid) {
        cal_ra = (double)g_calibration.matrix_ra_to_ra * ra_arcsec +
                 (double)g_calibration.matrix_ra_to_dec * dec_arcsec +
                 (double)g_calibration.offset_ra_arcsec;
        cal_dec = (double)g_calibration.matrix_dec_to_ra * ra_arcsec +
                  (double)g_calibration.matrix_dec_to_dec * dec_arcsec +
                  (double)g_calibration.offset_dec_arcsec;
    }
    *ra_steps = (int32_t)(cal_ra * STEPS_PER_ARCSEC);
    *dec_steps = (int32_t)(cal_dec * STEPS_PER_ARCSEC);
}

static void os_helper_steps_to_celestial(int32_t ra_steps, int32_t dec_steps, float *ra_hours, float *dec_degrees) {
    double raw_ra = (double)ra_steps / STEPS_PER_ARCSEC;
    double raw_dec = (double)dec_steps / STEPS_PER_ARCSEC;
    double ra_arcsec = raw_ra;
    double dec_arcsec = raw_dec;
    if (g_calibration.valid) {
        double u = raw_ra - (double)g_calibration.offset_ra_arcsec;
        double v = raw_dec - (double)g_calibration.offset_dec_arcsec;
        double det = (double)g_calibration.matrix_ra_to_ra * (double)g_calibration.matrix_dec_to_dec -
                     (double)g_calibration.matrix_ra_to_dec * (double)g_calibration.matrix_dec_to_ra;
        if (fabs(det) > 1e-9) {
            ra_arcsec = (v * (-(double)g_calibration.matrix_ra_to_dec) + u * (double)g_calibration.matrix_dec_to_dec) / det;
            dec_arcsec = (u * (-(double)g_calibration.matrix_dec_to_ra) + v * (double)g_calibration.matrix_ra_to_ra) / det;
        }
    }
    double rah = ra_arcsec / 54000.0;
    double decd = dec_arcsec / 3600.0;
    while (rah < 0.0) rah += 24.0;
    while (rah >= 24.0) rah -= 24.0;
    if (decd < -90.0) decd = -90.0;
    if (decd > 90.0) decd = 90.0;
    *ra_hours = (float)rah;
    *dec_degrees = (float)decd;
}

os_error_t os_init(void) {
    g_system_state = OS_STATE_INITIALIZING;
    g_align_star_count = 0;
    g_align_residual_valid = false;
    g_align_residual_arcsec = 0.0f;
    g_guide_pulse.active = false;
    g_manual_move_active = false;
    g_goto_active = false;
    g_is_moving = false;
    g_tracking_enabled = true;
    g_tracking_rate = OS_TRACK_RATE_SIDEREAL;
    g_custom_track_factor = 1.0f;
    g_pec_enabled = false;
    g_worm_phase_deg = 0.0f;

    if (os_hal_nvm_init() != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    if (os_hal_motor_init(0) != OS_ERR_NONE || os_hal_motor_init(1) != OS_ERR_NONE) {
        g_system_state = OS_STATE_FAULT;
        os_hal_buzzer_beep(1000, 3);
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    os_hal_gps_init();
    os_hal_rtc_init();
    os_hal_limit_init();
    os_hal_timer_motor_init();
    for (uint8_t c = 0; c <= OS_CHANNEL_ETHERNET; c++) {
        os_hal_comm_init(c);
    }

    os_site_info_t site_temp;
    if (os_hal_gps_poll(&site_temp) == OS_ERR_NONE && site_temp.valid) {
        g_site_info = site_temp;
    } else {
        uint32_t rtc_sec = 0;
        if (os_hal_rtc_read(&rtc_sec) == OS_ERR_NONE) {
            g_site_info.utc_epoch_seconds = rtc_sec;
        }
        g_site_info.latitude_degrees = 0.0f;
        g_site_info.longitude_degrees = 0.0f;
        g_site_info.elevation_metres = 0.0f;
        g_site_info.valid = false;
    }

    os_calibration_t loaded_cal;
    if (os_hal_nvm_read(NVM_OFFSET_CALIBRATION, (uint8_t *)&loaded_cal, sizeof(loaded_cal)) == OS_ERR_NONE && loaded_cal.valid) {
        g_calibration = loaded_cal;
    } else {
        g_calibration.matrix_ra_to_ra = 1.0f;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = 1.0f;
        g_calibration.offset_ra_arcsec = 0.0f;
        g_calibration.offset_dec_arcsec = 0.0f;
        g_calibration.valid = false;
    }

    os_pec_table_t loaded_pec;
    if (os_hal_nvm_read(NVM_OFFSET_PEC, (uint8_t *)&loaded_pec, sizeof(loaded_pec)) == OS_ERR_NONE && loaded_pec.valid) {
        g_pec_table = loaded_pec;
    }

    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        g_system_state = OS_STATE_FAULT;
        g_is_moving = false;
        g_goto_active = false;
        g_manual_move_active = false;
        os_hal_motor_set_frequency(0, 0);
        os_hal_motor_set_frequency(1, 0);
        return;
    }

    if (g_guide_pulse.active) {
        if (g_guide_pulse.duration_ms > 10) {
            g_guide_pulse.duration_ms -= 10;
        } else {
            g_guide_pulse.active = false;
            g_guide_pulse.duration_ms = 0;
        }
    }

    if (g_system_state == OS_STATE_GOTO && g_goto_active) {
        int32_t diff_ra = g_target_ra_steps - g_ra_position;
        int32_t diff_dec = g_target_dec_steps - g_dec_position;
        int32_t step_ra = (diff_ra > 0) ? 10 : ((diff_ra < 0) ? -10 : 0);
        int32_t step_dec = (diff_dec > 0) ? 10 : ((diff_dec < 0) ? -10 : 0);
        if (abs(diff_ra) < 10) step_ra = diff_ra;
        if (abs(diff_dec) < 10) step_dec = diff_dec;

        g_ra_position += step_ra;
        g_dec_position += step_dec;

        if (g_ra_position == g_target_ra_steps && g_dec_position == g_target_dec_steps) {
            g_goto_active = false;
            g_is_moving = false;
            g_system_state = OS_STATE_IDLE_TRACKING;
            os_hal_buzzer_beep(200, 1);
        }
    } else if (g_system_state == OS_STATE_MANUAL_MOTION && g_manual_move_active) {
        int32_t speed_steps = 5;
        if (g_manual_speed == OS_SPEED_MEDIUM) speed_steps = 15;
        else if (g_manual_speed == OS_SPEED_FAST) speed_steps = 50;
        else if (g_manual_speed == OS_SPEED_CUSTOM) speed_steps = (int32_t)(g_custom_manual_speed * STEPS_PER_ARCSEC / 10.0f);

        if (g_manual_dir == OS_DIRECTION_EAST) g_ra_position += speed_steps;
        else if (g_manual_dir == OS_DIRECTION_WEST) g_ra_position -= speed_steps;
        else if (g_manual_dir == OS_DIRECTION_NORTH) g_dec_position += speed_steps;
        else if (g_manual_dir == OS_DIRECTION_SOUTH) g_dec_position -= speed_steps;
    } else if (g_system_state == OS_STATE_IDLE_TRACKING && g_tracking_enabled) {
        g_ra_position += 1;
        g_worm_phase_deg += 0.1f;
        if (g_worm_phase_deg >= 360.0f) g_worm_phase_deg -= 360.0f;
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
    if (length < 3 || command[0] != OS_LX200_CMD_PREFIX || command[length - 1] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    char cmd_body[64];
    size_t body_len = length - 2;
    if (body_len >= sizeof(cmd_body)) body_len = sizeof(cmd_body) - 1;
    memcpy(cmd_body, command + 1, body_len);
    cmd_body[body_len] = '\0';

    reply_buffer[0] = '\0';

    if (strcmp(cmd_body, "GR") == 0) {
        float ra = 0.0f, dec = 0.0f;
        os_helper_steps_to_celestial(g_ra_position, g_dec_position, &ra, &dec);
        int hrs = (int)ra;
        int mins = (int)((ra - hrs) * 60.0f);
        int secs = (int)(((ra - hrs) * 60.0f - mins) * 60.0f);
        snprintf(reply_buffer, reply_buffer_size, "%02d:%02d:%02d#", hrs, mins, secs);
    } else if (strcmp(cmd_body, "GD") == 0) {
        float ra = 0.0f, dec = 0.0f;
        os_helper_steps_to_celestial(g_ra_position, g_dec_position, &ra, &dec);
        char sign = (dec >= 0) ? '+' : '-';
        float adec = fabsf(dec);
        int degs = (int)adec;
        int mins = (int)((adec - degs) * 60.0f);
        int secs = (int)(((adec - degs) * 60.0f - mins) * 60.0f);
        snprintf(reply_buffer, reply_buffer_size, "%c%02d*%02d'%02d#", sign, degs, mins, secs);
    } else if (strcmp(cmd_body, "GVP") == 0) {
        snprintf(reply_buffer, reply_buffer_size, "OnStep %d.%d.%d#",
                 OS_FIRMWARE_VERSION_MAJOR, OS_FIRMWARE_VERSION_MINOR, OS_FIRMWARE_VERSION_PATCH);
    } else if (strcmp(cmd_body, "MS") == 0) {
        os_error_t err = os_goto_equatorial(g_target_equatorial);
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "0#");
        } else {
            snprintf(reply_buffer, reply_buffer_size, "1Error#");
        }
    } else if (strncmp(cmd_body, "Sr", 2) == 0) {
        int h = 0, m = 0, s = 0;
        if (sscanf(cmd_body + 2, "%d:%d:%d", &h, &m, &s) >= 2) {
            g_target_equatorial.ra_hours = (float)h + (float)m / 60.0f + (float)s / 3600.0f;
            snprintf(reply_buffer, reply_buffer_size, "1#");
        } else {
            snprintf(reply_buffer, reply_buffer_size, "0#");
        }
    } else if (strncmp(cmd_body, "Sd", 2) == 0) {
        int d = 0, m = 0, s = 0;
        if (sscanf(cmd_body + 2, "%d*%d:%d", &d, &m, &s) >= 2) {
            float sign = (cmd_body[2] == '-') ? -1.0f : 1.0f;
            g_target_equatorial.dec_degrees = sign * (fabsf((float)d) + (float)m / 60.0f + (float)s / 3600.0f);
            snprintf(reply_buffer, reply_buffer_size, "1#");
        } else {
            snprintf(reply_buffer, reply_buffer_size, "0#");
        }
    } else if (strcmp(cmd_body, "hP") == 0) {
        os_error_t err = os_park();
        snprintf(reply_buffer, reply_buffer_size, (err == OS_ERR_NONE) ? "1#" : "0#");
    } else if (strcmp(cmd_body, "hO") == 0) {
        os_error_t err = os_unpark();
        snprintf(reply_buffer, reply_buffer_size, (err == OS_ERR_NONE) ? "1#" : "0#");
    } else if (strcmp(cmd_body, "Q") == 0 || strncmp(cmd_body, "Q", 1) == 0) {
        if (g_system_state == OS_STATE_GOTO) os_goto_abort();
        else if (g_system_state == OS_STATE_MANUAL_MOTION) os_move_stop();
        snprintf(reply_buffer, reply_buffer_size, "#");
    } else {
        snprintf(reply_buffer, reply_buffer_size, "OK#");
    }

    *reply_length = strlen(reply_buffer);
    os_hal_comm_write(source_channel, reply_buffer, *reply_length);
    return OS_ERR_NONE;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS ||
        target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    os_helper_celestial_to_steps(target.ra_hours, target.dec_degrees, &g_target_ra_steps, &g_target_dec_steps);
    g_system_state = OS_STATE_GOTO;
    g_goto_active = true;
    g_is_moving = true;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < 0.0f || target.altitude_degrees > 90.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    float ra_est = (target.azimuth_degrees / 360.0f) * 24.0f;
    float dec_est = target.altitude_degrees - 45.0f;
    os_equatorial_coord_t eq = {ra_est, dec_est};
    return os_goto_equatorial(eq);
}

os_error_t os_goto_abort(void) {
    if (g_system_state != OS_STATE_GOTO && g_system_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }
    g_goto_active = false;
    g_manual_move_active = false;
    g_is_moving = false;
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
    g_tracking_rate = rate;
    g_custom_track_factor = custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = g_tracking_rate;
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
    if (duration_ms == 0 || direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    g_guide_pulse.active = true;
    g_guide_pulse.duration_ms = duration_ms;
    g_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    g_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    g_guide_pulse.dec_priority = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);
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
    if (g_system_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    g_align_mode = mode;
    g_align_star_count = 0;
    g_system_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (star_coord.ra_hours < OS_RA_MIN_HOURS || star_coord.ra_hours > OS_RA_MAX_HOURS ||
        star_coord.dec_degrees < OS_DEC_MIN_DEG || star_coord.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_align_star_count < OS_CALIBRATION_MAX_STARS) {
        g_align_stars_eq[g_align_star_count] = star_coord;
        g_align_stars_motor[g_align_star_count] = motor_pos;
        g_align_star_count++;
    }
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (g_system_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    uint8_t min_req = 1;
    if (g_align_mode == OS_ALIGN_2STAR) min_req = 2;
    else if (g_align_mode == OS_ALIGN_3STAR || g_align_mode == OS_ALIGN_NSTAR) min_req = 3;

    if (g_align_star_count < min_req) {
        return OS_ERR_INVALID_STATE;
    }

    if (g_align_mode == OS_ALIGN_1STAR) {
        double s_ra = (double)g_align_stars_eq[0].ra_hours * 54000.0;
        double s_dec = (double)g_align_stars_eq[0].dec_degrees * 3600.0;
        double m_ra = (double)g_align_stars_motor[0].ra_steps;
        double m_dec = (double)g_align_stars_motor[0].dec_steps;
        g_calibration.matrix_ra_to_ra = 1.0f;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = 1.0f;
        g_calibration.offset_ra_arcsec = (float)(m_ra - s_ra);
        g_calibration.offset_dec_arcsec = (float)(m_dec - s_dec);
        g_calibration.valid = true;
        g_align_residual_arcsec = 0.0f;
        g_align_residual_valid = true;
    } else if (g_align_mode == OS_ALIGN_2STAR) {
        double s_ra1 = (double)g_align_stars_eq[0].ra_hours * 54000.0;
        double s_dec1 = (double)g_align_stars_eq[0].dec_degrees * 3600.0;
        double m_ra1 = (double)g_align_stars_motor[0].ra_steps;
        double m_dec1 = (double)g_align_stars_motor[0].dec_steps;
        double s_ra2 = (double)g_align_stars_eq[1].ra_hours * 54000.0;
        double s_dec2 = (double)g_align_stars_eq[1].dec_degrees * 3600.0;
        double m_ra2 = (double)g_align_stars_motor[1].ra_steps;
        double m_dec2 = (double)g_align_stars_motor[1].dec_steps;
        double scale_ra = (fabs(s_ra2 - s_ra1) > 1e-3) ? (m_ra2 - m_ra1) / (s_ra2 - s_ra1) : 1.0;
        double scale_dec = (fabs(s_dec2 - s_dec1) > 1e-3) ? (m_dec2 - m_dec1) / (s_dec2 - s_dec1) : 1.0;
        g_calibration.matrix_ra_to_ra = (float)scale_ra;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = (float)scale_dec;
        g_calibration.offset_ra_arcsec = (float)(m_ra1 - scale_ra * s_ra1);
        g_calibration.offset_dec_arcsec = (float)(m_dec1 - scale_dec * s_dec1);
        g_calibration.valid = true;
        g_align_residual_arcsec = 0.0f;
        g_align_residual_valid = true;
    } else if (g_align_mode == OS_ALIGN_3STAR) {
        double x1 = (double)g_align_stars_eq[0].ra_hours * 54000.0, y1 = (double)g_align_stars_eq[0].dec_degrees * 3600.0;
        double u1 = (double)g_align_stars_motor[0].ra_steps, v1 = (double)g_align_stars_motor[0].dec_steps;
        double x2 = (double)g_align_stars_eq[1].ra_hours * 54000.0, y2 = (double)g_align_stars_eq[1].dec_degrees * 3600.0;
        double u2 = (double)g_align_stars_motor[1].ra_steps, v2 = (double)g_align_stars_motor[1].dec_steps;
        double x3 = (double)g_align_stars_eq[2].ra_hours * 54000.0, y3 = (double)g_align_stars_eq[2].dec_degrees * 3600.0;
        double u3 = (double)g_align_stars_motor[2].ra_steps, v3 = (double)g_align_stars_motor[2].dec_steps;

        double det = x1 * (y2 - y3) - y1 * (x2 - x3) + (x2 * y3 - x3 * y2);
        if (fabs(det) < 1e-6) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        double detA = u1 * (y2 - y3) - y1 * (u2 - u3) + (u2 * y3 - u3 * y2);
        double detB = x1 * (u2 - u3) - u1 * (x2 - x3) + (x2 * u3 - x3 * u2);
        double detC = x1 * (y2 * u3 - y3 * u2) - y1 * (x2 * u3 - x3 * u2) + u1 * (x2 * y3 - x3 * y2);
        double detD = v1 * (y2 - y3) - y1 * (v2 - v3) + (v2 * y3 - v3 * y2);
        double detE = x1 * (v2 - v3) - v1 * (x2 - x3) + (x2 * v3 - x3 * v2);
        double detF = x1 * (y2 * v3 - y3 * v2) - y1 * (x2 * v3 - x3 * v2) + v1 * (x2 * y3 - x3 * y2);

        g_calibration.matrix_ra_to_ra = (float)(detA / det);
        g_calibration.matrix_ra_to_dec = (float)(detB / det);
        g_calibration.offset_ra_arcsec = (float)(detC / det);
        g_calibration.matrix_dec_to_ra = (float)(detD / det);
        g_calibration.matrix_dec_to_dec = (float)(detE / det);
        g_calibration.offset_dec_arcsec = (float)(detF / det);
        g_calibration.valid = true;
        g_align_residual_arcsec = 0.0f;
        g_align_residual_valid = true;
    } else {
        double sum_x = 0, sum_y = 0, sum_xx = 0, sum_yy = 0, sum_xy = 0;
        double sum_u = 0, sum_xu = 0, sum_yu = 0, sum_v = 0, sum_xv = 0, sum_yv = 0;
        int n = g_align_star_count;
        for (int i = 0; i < n; i++) {
            double x = (double)g_align_stars_eq[i].ra_hours * 54000.0;
            double y = (double)g_align_stars_eq[i].dec_degrees * 3600.0;
            double u = (double)g_align_stars_motor[i].ra_steps;
            double v = (double)g_align_stars_motor[i].dec_steps;
            sum_x += x; sum_y += y; sum_xx += x * x; sum_yy += y * y; sum_xy += x * y;
            sum_u += u; sum_xu += x * u; sum_yu += y * u;
            sum_v += v; sum_xv += x * v; sum_yv += y * v;
        }
        double det = n * (sum_xx * sum_yy - sum_xy * sum_xy) - sum_x * (sum_x * sum_yy - sum_y * sum_xy) + sum_y * (sum_x * sum_xy - sum_y * sum_xx);
        if (fabs(det) < 1e-6) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        g_calibration.matrix_ra_to_ra = (float)((sum_xu * sum_yy - sum_yu * sum_xy) / (sum_xx * sum_yy - sum_xy * sum_xy));
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.offset_ra_arcsec = (float)((sum_u - g_calibration.matrix_ra_to_ra * sum_x) / n);
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = (float)((sum_yv * sum_xx - sum_xv * sum_xy) / (sum_xx * sum_yy - sum_xy * sum_xy));
        g_calibration.offset_dec_arcsec = (float)((sum_v - g_calibration.matrix_dec_to_dec * sum_y) / n);
        g_calibration.valid = true;

        double sum_sq_res = 0.0;
        for (int i = 0; i < n; i++) {
            double x = (double)g_align_stars_eq[i].ra_hours * 54000.0;
            double y = (double)g_align_stars_eq[i].dec_degrees * 3600.0;
            double u = (double)g_align_stars_motor[i].ra_steps;
            double v = (double)g_align_stars_motor[i].dec_steps;
            double cu = g_calibration.matrix_ra_to_ra * x + g_calibration.offset_ra_arcsec;
            double cv = g_calibration.matrix_dec_to_dec * y + g_calibration.offset_dec_arcsec;
            sum_sq_res += (cu - u) * (cu - u) + (cv - v) * (cv - v);
        }
        g_align_residual_arcsec = (float)sqrt(sum_sq_res / (2.0 * n));
        g_align_residual_valid = true;
    }

    os_hal_nvm_write(NVM_OFFSET_CALIBRATION, (const uint8_t *)&g_calibration, sizeof(g_calibration));
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *residual_arcsec = g_align_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    if (g_system_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    g_align_star_count = 0;
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (g_system_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    os_helper_celestial_to_steps(g_park_pos.ra_hours, g_park_pos.dec_degrees, &g_target_ra_steps, &g_target_dec_steps);
    g_ra_position = g_target_ra_steps;
    g_dec_position = g_target_dec_steps;
    g_tracking_enabled = false;
    g_is_moving = false;
    os_hal_motor_enable(0, false);
    os_hal_motor_enable(1, false);
    g_system_state = OS_STATE_PARKED;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (g_system_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);
    for (uint8_t c = 0; c <= OS_CHANNEL_ETHERNET; c++) {
        os_hal_comm_init(c);
    }
    uint32_t rtc_sec = 0;
    if (os_hal_rtc_read(&rtc_sec) == OS_ERR_NONE) {
        g_site_info.utc_epoch_seconds = rtc_sec;
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
    g_park_pos = park_pos;
    os_hal_nvm_write(NVM_OFFSET_PARK, (const uint8_t *)&g_park_pos, sizeof(g_park_pos));
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST ||
        speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (g_system_state != OS_STATE_IDLE_TRACKING && g_system_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }
    g_manual_dir = direction;
    g_manual_speed = speed;
    g_manual_move_active = true;
    g_is_moving = true;
    g_system_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (g_system_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }
    g_manual_move_active = false;
    g_is_moving = false;
    g_system_state = OS_STATE_IDLE_TRACKING;
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
    *state = g_system_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    os_helper_steps_to_celestial(g_ra_position, g_dec_position, &coord->ra_hours, &coord->dec_degrees);
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
    pos->ra_steps = g_ra_position;
    pos->dec_steps = g_dec_position;
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
    *moving = g_is_moving;
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
    if (enable && !g_pec_table.valid) {
        return OS_ERR_INVALID_STATE;
    }
    g_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_pec_table = *table;
    g_pec_table.valid = true;
    if (os_hal_nvm_write(NVM_OFFSET_PEC, (const uint8_t *)&g_pec_table, sizeof(g_pec_table)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
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
    if (idx >= 360) idx = 359;
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
    os_hal_nvm_write(NVM_OFFSET_CALIBRATION, (const uint8_t *)&g_calibration, sizeof(g_calibration));
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_init(uint8_t axis) {
    if (axis != 0 && axis != 1) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_hal_motor_init_done[axis] = true;
    g_hal_motor_enabled[axis] = true;
    g_hal_motor_freq[axis] = 0;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (axis != 0 && axis != 1) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_hal_motor_freq[axis] = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (axis != 0 && axis != 1) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_hal_motor_dir[axis] = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (axis != 0 && axis != 1) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_hal_motor_enabled[axis] = enable;
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (axis == 0) return g_ra_position;
    if (axis == 1) return g_dec_position;
    return 0;
}

os_error_t os_hal_gps_init(void) {
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = g_site_info;
    if (!site->valid) {
        return OS_ERR_GPS_NO_SIGNAL;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void) {
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *utc_epoch_seconds = g_hal_rtc_epoch;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    g_hal_rtc_epoch = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    g_hal_limit_triggered[0] = false;
    g_hal_limit_triggered[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (axis != 0 && axis != 1) {
        return true;
    }
    return g_hal_limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void) {
    if (!g_hal_nvm_init_done) {
        memset(g_hal_nvm, 0, sizeof(g_hal_nvm));
        g_hal_nvm_init_done = true;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((uint32_t)offset + (uint32_t)length > NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(data, g_hal_nvm + offset, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((uint32_t)offset + (uint32_t)length > NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(g_hal_nvm + offset, data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_hal_comm_init_done[channel] = true;
    g_hal_comm_rx_head[channel] = 0;
    g_hal_comm_rx_tail[channel] = 0;
    g_hal_comm_tx_head[channel] = 0;
    g_hal_comm_tx_tail[channel] = 0;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET || !g_hal_comm_init_done[channel]) {
        return 0;
    }
    return (int16_t)(g_hal_comm_rx_head[channel] - g_hal_comm_rx_tail[channel]);
}

char os_hal_comm_read(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET || !g_hal_comm_init_done[channel]) {
        return 0;
    }
    if (g_hal_comm_rx_head[channel] == g_hal_comm_rx_tail[channel]) {
        return 0;
    }
    char c = g_hal_comm_rx[channel][g_hal_comm_rx_tail[channel] % 128];
    g_hal_comm_rx_tail[channel]++;
    return c;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (data == NULL || channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    for (size_t i = 0; i < length; i++) {
        g_hal_comm_tx[channel][g_hal_comm_tx_head[channel] % 128] = data[i];
        g_hal_comm_tx_head[channel]++;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) {
    if (duration_ms == 0 || count == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void) {
    return OS_ERR_NONE;
}
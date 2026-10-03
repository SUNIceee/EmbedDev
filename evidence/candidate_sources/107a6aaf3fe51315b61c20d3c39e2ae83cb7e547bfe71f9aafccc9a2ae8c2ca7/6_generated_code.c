#include "6_generated_code.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define STEPS_PER_DEGREE 1000.0f
#define STEPS_PER_HOUR   15000.0f
#define ARCSEC_PER_STEP  3.6

static os_state_t g_system_state = OS_STATE_INITIALIZING;
static bool g_tracking_enabled = false;
static os_track_rate_t g_track_rate = OS_TRACK_RATE_SIDEREAL;
static float g_custom_track_factor = 1.0f;
static os_site_info_t g_site_info = {0.0f, 0.0f, 0.0f, 0, false};
static int32_t g_motor_position_steps[2] = {0, 0};
static int32_t g_goto_target_steps[2] = {0, 0};
static os_calibration_t g_calibration = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, false};
static float g_residual_arcsec = 0.0f;
static bool g_residual_computed = false;
static os_align_mode_t g_align_mode = OS_ALIGN_3STAR;
static uint8_t g_align_star_count = 0;
static os_equatorial_coord_t g_align_stars_sky[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t g_align_stars_motor[OS_CALIBRATION_MAX_STARS];
static os_guide_pulse_t g_guide_pulse = {false, 0, 0.5f, true, true, false};
static os_pec_table_t g_pec_table = {{0}, false};
static bool g_pec_enabled = false;
static float g_pec_phase_deg = 0.0f;
static os_equatorial_coord_t g_park_pos = {0.0f, 90.0f};
static bool g_is_parked = false;
static bool g_manual_motion_active = false;
static os_direction_t g_manual_direction = OS_DIRECTION_EAST;
static os_speed_level_t g_manual_speed = OS_SPEED_MEDIUM;
static float g_custom_slew_rate = 3600.0f;

static bool g_hal_motor_initialized[2] = {false, false};
static bool g_hal_motor_enabled[2] = {false, false};
static uint32_t g_hal_motor_freq[2] = {0, 0};
static bool g_hal_motor_dir[2] = {true, true};
static bool g_hal_limit_triggered[2] = {false, false};
static uint8_t g_nvm_buffer[512] = {0};
static char g_comm_rx[4][64] = {{0}};
static uint8_t g_comm_rx_head[4] = {0};
static uint8_t g_comm_rx_tail[4] = {0};
static bool g_comm_enabled[4] = {false, false, false, false};
static uint32_t g_rtc_seconds = 1700000000;

os_error_t os_init(void) {
    g_system_state = OS_STATE_INITIALIZING;
    g_residual_computed = false;
    g_align_star_count = 0;
    g_guide_pulse.active = false;
    g_guide_pulse.duration_ms = 0;
    g_manual_motion_active = false;
    g_is_parked = false;
    g_pec_enabled = false;
    g_pec_phase_deg = 0.0f;

    os_hal_nvm_init();
    os_hal_nvm_read(0, (uint8_t *)&g_calibration, sizeof(os_calibration_t));
    if (g_calibration.matrix_ra_to_ra == 0.0f && g_calibration.matrix_dec_to_dec == 0.0f) {
        g_calibration.matrix_ra_to_ra = 1.0f;
        g_calibration.matrix_dec_to_dec = 1.0f;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.offset_ra_arcsec = 0.0f;
        g_calibration.offset_dec_arcsec = 0.0f;
        g_calibration.valid = false;
    }

    for (uint8_t c = 0; c <= OS_CHANNEL_ETHERNET; c++) {
        os_hal_comm_init(c);
    }

    if (os_hal_motor_init(0) != OS_ERR_NONE || os_hal_motor_init(1) != OS_ERR_NONE) {
        g_system_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    os_hal_gps_init();
    if (os_hal_gps_poll(&g_site_info) == OS_ERR_NONE && g_site_info.valid) {
        os_hal_rtc_set(g_site_info.utc_epoch_seconds);
    } else {
        os_hal_rtc_init();
        os_hal_rtc_read(&g_site_info.utc_epoch_seconds);
    }

    os_hal_limit_init();
    os_hal_timer_motor_init();

    g_tracking_enabled = true;
    g_track_rate = OS_TRACK_RATE_SIDEREAL;
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        os_hal_motor_set_frequency(0, 0);
        os_hal_motor_set_frequency(1, 0);
        g_system_state = OS_STATE_FAULT;
        return;
    }

    for (uint8_t ch = 0; ch <= 3; ch++) {
        if (os_hal_comm_available(ch) > 0) {
            char cmd_buf[64] = {0};
            size_t idx = 0;
            while (os_hal_comm_available(ch) > 0 && idx < sizeof(cmd_buf) - 1) {
                char c = os_hal_comm_read(ch);
                cmd_buf[idx++] = c;
                if (c == '#') break;
            }
            if (idx > 0 && cmd_buf[idx - 1] == '#') {
                char reply[128] = {0};
                size_t reply_len = 0;
                if (os_command_parse(cmd_buf, idx, ch, reply, sizeof(reply), &reply_len) == OS_ERR_NONE && reply_len > 0) {
                    os_hal_comm_write(ch, reply, reply_len);
                }
            }
        }
    }

    if (g_system_state == OS_STATE_GOTO) {
        bool ra_reached = false;
        bool dec_reached = false;

        if (g_motor_position_steps[0] < g_goto_target_steps[0]) {
            g_motor_position_steps[0] += 50;
            if (g_motor_position_steps[0] >= g_goto_target_steps[0]) {
                g_motor_position_steps[0] = g_goto_target_steps[0];
                ra_reached = true;
            }
        } else if (g_motor_position_steps[0] > g_goto_target_steps[0]) {
            g_motor_position_steps[0] -= 50;
            if (g_motor_position_steps[0] <= g_goto_target_steps[0]) {
                g_motor_position_steps[0] = g_goto_target_steps[0];
                ra_reached = true;
            }
        } else {
            ra_reached = true;
        }

        if (g_motor_position_steps[1] < g_goto_target_steps[1]) {
            g_motor_position_steps[1] += 50;
            if (g_motor_position_steps[1] >= g_goto_target_steps[1]) {
                g_motor_position_steps[1] = g_goto_target_steps[1];
                dec_reached = true;
            }
        } else if (g_motor_position_steps[1] > g_goto_target_steps[1]) {
            g_motor_position_steps[1] -= 50;
            if (g_motor_position_steps[1] <= g_goto_target_steps[1]) {
                g_motor_position_steps[1] = g_goto_target_steps[1];
                dec_reached = true;
            }
        } else {
            dec_reached = true;
        }

        if (ra_reached && dec_reached) {
            os_hal_buzzer_beep(200, 1);
            g_system_state = OS_STATE_IDLE_TRACKING;
        }
    } else if (g_system_state == OS_STATE_MANUAL_MOTION) {
        int32_t step_delta = 10;
        if (g_manual_direction == OS_DIRECTION_EAST) g_motor_position_steps[0] += step_delta;
        else if (g_manual_direction == OS_DIRECTION_WEST) g_motor_position_steps[0] -= step_delta;
        else if (g_manual_direction == OS_DIRECTION_NORTH) g_motor_position_steps[1] += step_delta;
        else if (g_manual_direction == OS_DIRECTION_SOUTH) g_motor_position_steps[1] -= step_delta;
    } else if (g_system_state == OS_STATE_IDLE_TRACKING && g_tracking_enabled) {
        g_motor_position_steps[0] += 1;
    }

    if (g_guide_pulse.active) {
        if (g_guide_pulse.duration_ms > 10) {
            g_guide_pulse.duration_ms -= 10;
        } else {
            g_guide_pulse.duration_ms = 0;
            g_guide_pulse.active = false;
            if (g_system_state == OS_STATE_GOTO || g_system_state == OS_STATE_MANUAL_MOTION) {
            } else {
                g_system_state = OS_STATE_IDLE_TRACKING;
            }
        }
    }

    if (g_pec_enabled) {
        g_pec_phase_deg += 0.1f;
        if (g_pec_phase_deg >= 360.0f) g_pec_phase_deg -= 360.0f;
    }
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    (void)source_channel;
    if (!command || !reply_buffer || !reply_length) return OS_ERR_INVALID_ARGUMENT;
    if (length < 2 || command[0] != OS_LX200_CMD_PREFIX || command[length - 1] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    if (strncmp(command, ":GR#", 4) == 0) {
        os_equatorial_coord_t eq;
        os_query_coordinates(&eq);
        int hrs = (int)eq.ra_hours;
        int mins = (int)((eq.ra_hours - hrs) * 60.0f);
        int secs = (int)((eq.ra_hours - hrs - mins / 60.0f) * 3600.0f);
        snprintf(reply_buffer, reply_buffer_size, "%02d:%02d:%02d#", hrs, abs(mins), abs(secs));
        *reply_length = strlen(reply_buffer);
        return OS_ERR_NONE;
    } else if (strncmp(command, ":GD#", 4) == 0) {
        os_equatorial_coord_t eq;
        os_query_coordinates(&eq);
        int degs = (int)eq.dec_degrees;
        int mins = (int)(fabsf(eq.dec_degrees - degs) * 60.0f);
        snprintf(reply_buffer, reply_buffer_size, "%+03d*%02d#", degs, mins);
        *reply_length = strlen(reply_buffer);
        return OS_ERR_NONE;
    } else if (strncmp(command, ":GVP#", 5) == 0) {
        snprintf(reply_buffer, reply_buffer_size, "OnStep 1.0.0#");
        *reply_length = strlen(reply_buffer);
        return OS_ERR_NONE;
    } else if (strncmp(command, ":hP#", 4) == 0) {
        os_error_t err = os_park();
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1");
        } else {
            snprintf(reply_buffer, reply_buffer_size, "0");
        }
        *reply_length = strlen(reply_buffer);
        return err;
    } else if (strncmp(command, ":hO#", 4) == 0) {
        os_error_t err = os_unpark();
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1");
        } else {
            snprintf(reply_buffer, reply_buffer_size, "0");
        }
        *reply_length = strlen(reply_buffer);
        return err;
    }

    snprintf(reply_buffer, reply_buffer_size, "0#");
    *reply_length = strlen(reply_buffer);
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
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    double sky_ra_arcsec = (double)target.ra_hours * 15.0 * 3600.0;
    double sky_dec_arcsec = (double)target.dec_degrees * 3600.0;
    double motor_ra_arcsec = sky_ra_arcsec;
    double motor_dec_arcsec = sky_dec_arcsec;

    if (g_calibration.valid) {
        motor_ra_arcsec = (double)g_calibration.matrix_ra_to_ra * sky_ra_arcsec +
                          (double)g_calibration.matrix_ra_to_dec * sky_dec_arcsec +
                          (double)g_calibration.offset_ra_arcsec;
        motor_dec_arcsec = (double)g_calibration.matrix_dec_to_ra * sky_ra_arcsec +
                           (double)g_calibration.matrix_dec_to_dec * sky_dec_arcsec +
                           (double)g_calibration.offset_dec_arcsec;
    }

    g_goto_target_steps[0] = (int32_t)(motor_ra_arcsec / ARCSEC_PER_STEP);
    g_goto_target_steps[1] = (int32_t)(motor_dec_arcsec / ARCSEC_PER_STEP);
    g_system_state = OS_STATE_GOTO;
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
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    os_equatorial_coord_t eq;
    eq.ra_hours = target.azimuth_degrees / 15.0f;
    eq.dec_degrees = target.altitude_degrees;
    return os_goto_equatorial(eq);
}

os_error_t os_goto_abort(void) {
    if (g_system_state != OS_STATE_GOTO) {
        return OS_ERR_INVALID_STATE;
    }
    g_goto_target_steps[0] = g_motor_position_steps[0];
    g_goto_target_steps[1] = g_motor_position_steps[1];
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
    if (g_system_state == OS_STATE_FAULT || g_system_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    g_tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    if (g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    g_tracking_enabled = false;
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (duration_ms == 0 || direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state == OS_STATE_FAULT || g_system_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
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
    if (!pulse) return OS_ERR_INVALID_ARGUMENT;
    *pulse = g_guide_pulse;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state == OS_STATE_FAULT || g_system_state == OS_STATE_GOTO) {
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
        g_align_stars_sky[g_align_star_count] = star_coord;
        g_align_stars_motor[g_align_star_count] = motor_pos;
        g_align_star_count++;
    }
    return OS_ERR_NONE;
}

static bool solve_3x3_double(double M[3][3], double B[3], double X[3]) {
    double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
                 M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                 M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    if (fabs(det) < 1e-9) return false;
    double invdet = 1.0 / det;
    X[0] = invdet * (B[0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
                     M[0][1] * (B[1] * M[2][2] - M[1][2] * B[2]) +
                     M[0][2] * (B[1] * M[2][1] - M[1][1] * B[2]));
    X[1] = invdet * (M[0][0] * (B[1] * M[2][2] - M[1][2] * B[2]) -
                     B[0] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                     M[0][2] * (M[1][0] * B[2] - B[1] * M[2][0]));
    X[2] = invdet * (M[0][0] * (M[1][1] * B[2] - B[1] * M[2][1]) -
                     M[0][1] * (M[1][0] * B[2] - B[1] * M[2][0]) +
                     B[0] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]));
    return true;
}

os_error_t os_align_compute(void) {
    uint8_t min_stars = 1;
    if (g_align_mode == OS_ALIGN_2STAR) min_stars = 2;
    else if (g_align_mode == OS_ALIGN_3STAR || g_align_mode == OS_ALIGN_NSTAR) min_stars = 3;

    if (g_align_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    double S_ra[OS_CALIBRATION_MAX_STARS];
    double S_dec[OS_CALIBRATION_MAX_STARS];
    double M_ra[OS_CALIBRATION_MAX_STARS];
    double M_dec[OS_CALIBRATION_MAX_STARS];

    for (uint8_t i = 0; i < g_align_star_count; i++) {
        S_ra[i] = (double)g_align_stars_sky[i].ra_hours * 15.0 * 3600.0;
        S_dec[i] = (double)g_align_stars_sky[i].dec_degrees * 3600.0;
        M_ra[i] = (double)g_align_stars_motor[i].ra_steps * ARCSEC_PER_STEP;
        M_dec[i] = (double)g_align_stars_motor[i].dec_steps * ARCSEC_PER_STEP;
    }

    double A = 1.0, B = 0.0, O_ra = 0.0;
    double C = 0.0, D = 1.0, O_dec = 0.0;
    double rms_residual = 0.0;

    if (g_align_mode == OS_ALIGN_1STAR) {
        O_ra = M_ra[0] - S_ra[0];
        O_dec = M_dec[0] - S_dec[0];
        rms_residual = 0.0;
    } else if (g_align_mode == OS_ALIGN_2STAR) {
        double dS_ra = S_ra[1] - S_ra[0];
        double dM_ra = M_ra[1] - M_ra[0];
        if (fabs(dS_ra) > 1e-6) {
            A = dM_ra / dS_ra;
            O_ra = M_ra[0] - A * S_ra[0];
        } else {
            O_ra = M_ra[0] - S_ra[0];
        }
        double dS_dec = S_dec[1] - S_dec[0];
        double dM_dec = M_dec[1] - M_dec[0];
        if (fabs(dS_dec) > 1e-6) {
            D = dM_dec / dS_dec;
            O_dec = M_dec[0] - D * S_dec[0];
        } else {
            O_dec = M_dec[0] - S_dec[0];
        }
        rms_residual = 0.0;
    } else {
        double M_mat[3][3] = {{0}};
        double B_ra[3] = {0};
        double B_dec[3] = {0};
        uint8_t n = g_align_star_count;

        for (uint8_t i = 0; i < n; i++) {
            M_mat[0][0] += S_ra[i] * S_ra[i];
            M_mat[0][1] += S_ra[i] * S_dec[i];
            M_mat[0][2] += S_ra[i];
            M_mat[1][1] += S_dec[i] * S_dec[i];
            M_mat[1][2] += S_dec[i];
            M_mat[2][2] += 1.0;

            B_ra[0] += S_ra[i] * M_ra[i];
            B_ra[1] += S_dec[i] * M_ra[i];
            B_ra[2] += M_ra[i];

            B_dec[0] += S_ra[i] * M_dec[i];
            B_dec[1] += S_dec[i] * M_dec[i];
            B_dec[2] += M_dec[i];
        }
        M_mat[1][0] = M_mat[0][1];
        M_mat[2][0] = M_mat[0][2];
        M_mat[2][1] = M_mat[1][2];

        double X_ra[3], X_dec[3];
        if (!solve_3x3_double(M_mat, B_ra, X_ra) || !solve_3x3_double(M_mat, B_dec, X_dec)) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        A = X_ra[0]; B = X_ra[1]; O_ra = X_ra[2];
        C = X_dec[0]; D = X_dec[1]; O_dec = X_dec[2];

        double sum_sq = 0.0;
        for (uint8_t i = 0; i < n; i++) {
            double err_ra = (A * S_ra[i] + B * S_dec[i] + O_ra) - M_ra[i];
            double err_dec = (C * S_ra[i] + D * S_dec[i] + O_dec) - M_dec[i];
            sum_sq += err_ra * err_ra + err_dec * err_dec;
        }
        rms_residual = sqrt(sum_sq / (double)n);

        if (n >= 4 && rms_residual > 600.0) {
            return OS_ERR_CALIBRATION_FAILED;
        }
    }

    g_calibration.matrix_ra_to_ra = (float)A;
    g_calibration.matrix_ra_to_dec = (float)B;
    g_calibration.matrix_dec_to_ra = (float)C;
    g_calibration.matrix_dec_to_dec = (float)D;
    g_calibration.offset_ra_arcsec = (float)O_ra;
    g_calibration.offset_dec_arcsec = (float)O_dec;
    g_calibration.valid = true;
    g_residual_arcsec = (float)rms_residual;
    g_residual_computed = true;

    os_hal_nvm_write(0, (const uint8_t *)&g_calibration, sizeof(os_calibration_t));
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (!residual_arcsec) return OS_ERR_INVALID_ARGUMENT;
    if (!g_residual_computed) return OS_ERR_INVALID_STATE;
    *residual_arcsec = g_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    if (g_system_state == OS_STATE_FAULT) return OS_ERR_INVALID_STATE;
    g_align_star_count = 0;
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (g_system_state == OS_STATE_FAULT) return OS_ERR_INVALID_STATE;
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    g_goto_target_steps[0] = (int32_t)(g_park_pos.ra_hours * STEPS_PER_HOUR);
    g_goto_target_steps[1] = (int32_t)(g_park_pos.dec_degrees * STEPS_PER_DEGREE);
    g_motor_position_steps[0] = g_goto_target_steps[0];
    g_motor_position_steps[1] = g_goto_target_steps[1];
    os_hal_motor_set_frequency(0, 0);
    os_hal_motor_set_frequency(1, 0);
    os_hal_motor_enable(0, false);
    os_hal_motor_enable(1, false);
    g_tracking_enabled = false;
    g_is_parked = true;
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
    os_hal_rtc_read(&g_site_info.utc_epoch_seconds);
    g_tracking_enabled = true;
    g_is_parked = false;
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (park_pos.ra_hours < OS_RA_MIN_HOURS || park_pos.ra_hours > OS_RA_MAX_HOURS ||
        park_pos.dec_degrees < OS_DEC_MIN_DEG || park_pos.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_park_pos = park_pos;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST ||
        speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    uint8_t axis = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) ? 0 : 1;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    g_manual_direction = direction;
    g_manual_speed = speed;
    g_manual_motion_active = true;
    g_system_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (g_system_state == OS_STATE_FAULT) return OS_ERR_INVALID_STATE;
    g_manual_motion_active = false;
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (arcsec_per_sec <= 0.0f) return OS_ERR_INVALID_ARGUMENT;
    g_custom_slew_rate = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (!state) return OS_ERR_INVALID_ARGUMENT;
    *state = g_system_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (!coord) return OS_ERR_INVALID_ARGUMENT;
    double motor_ra_arcsec = (double)g_motor_position_steps[0] * ARCSEC_PER_STEP;
    double motor_dec_arcsec = (double)g_motor_position_steps[1] * ARCSEC_PER_STEP;
    double sky_ra_arcsec = motor_ra_arcsec;
    double sky_dec_arcsec = motor_dec_arcsec;

    if (g_calibration.valid) {
        double A = (double)g_calibration.matrix_ra_to_ra;
        double B = (double)g_calibration.matrix_ra_to_dec;
        double C = (double)g_calibration.matrix_dec_to_ra;
        double D = (double)g_calibration.matrix_dec_to_dec;
        double O_ra = (double)g_calibration.offset_ra_arcsec;
        double O_dec = (double)g_calibration.offset_dec_arcsec;
        double det = A * D - B * C;
        if (fabs(det) > 1e-9) {
            double dM_ra = motor_ra_arcsec - O_ra;
            double dM_dec = motor_dec_arcsec - O_dec;
            sky_ra_arcsec = (D * dM_ra - B * dM_dec) / det;
            sky_dec_arcsec = (-C * dM_ra + A * dM_dec) / det;
        } else {
            sky_ra_arcsec = motor_ra_arcsec - O_ra;
            sky_dec_arcsec = motor_dec_arcsec - O_dec;
        }
    }

    double ra_hours = sky_ra_arcsec / 54000.0;
    while (ra_hours < 0.0) ra_hours += 24.0;
    while (ra_hours >= 24.0) ra_hours -= 24.0;
    double dec_degrees = sky_dec_arcsec / 3600.0;
    if (dec_degrees < -90.0) dec_degrees = -90.0;
    if (dec_degrees > 90.0) dec_degrees = 90.0;

    coord->ra_hours = (float)ra_hours;
    coord->dec_degrees = (float)dec_degrees;
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (!site) return OS_ERR_INVALID_ARGUMENT;
    *site = g_site_info;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (!pos) return OS_ERR_INVALID_ARGUMENT;
    pos->ra_steps = g_motor_position_steps[0];
    pos->dec_steps = g_motor_position_steps[1];
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
    *moving = (g_system_state == OS_STATE_GOTO || g_system_state == OS_STATE_MANUAL_MOTION);
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (!locked) return OS_ERR_INVALID_ARGUMENT;
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
    if (!table || !table->valid) return OS_ERR_INVALID_ARGUMENT;
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
    int idx = (int)worm_phase_deg;
    if (idx >= 360) idx = 359;
    g_pec_table.corrections[idx] = error_arcsec;
    g_pec_table.valid = true;
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (!calib) return OS_ERR_INVALID_ARGUMENT;
    *calib = g_calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    if (g_system_state == OS_STATE_FAULT) return OS_ERR_INVALID_STATE;
    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.matrix_ra_to_dec = 0.0f;
    g_calibration.matrix_dec_to_ra = 0.0f;
    g_calibration.offset_ra_arcsec = 0.0f;
    g_calibration.offset_dec_arcsec = 0.0f;
    g_calibration.valid = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_init(uint8_t axis) {
    if (axis >= 2) return OS_ERR_INVALID_ARGUMENT;
    g_hal_motor_initialized[axis] = true;
    g_hal_motor_enabled[axis] = false;
    g_hal_motor_freq[axis] = 0;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (axis >= 2) return OS_ERR_INVALID_ARGUMENT;
    g_hal_motor_freq[axis] = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (axis >= 2) return OS_ERR_INVALID_ARGUMENT;
    g_hal_motor_dir[axis] = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (axis >= 2) return OS_ERR_INVALID_ARGUMENT;
    g_hal_motor_enabled[axis] = enable;
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (axis >= 2) return 0;
    return g_motor_position_steps[axis];
}

os_error_t os_hal_gps_init(void) {
    g_site_info.latitude_degrees = 0.0f;
    g_site_info.longitude_degrees = 0.0f;
    g_site_info.elevation_metres = 0.0f;
    g_site_info.valid = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (!site) return OS_ERR_INVALID_ARGUMENT;
    *site = g_site_info;
    if (!g_site_info.valid) return OS_ERR_GPS_NO_SIGNAL;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void) {
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (!utc_epoch_seconds) return OS_ERR_INVALID_ARGUMENT;
    *utc_epoch_seconds = g_rtc_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    if (utc_epoch_seconds == 0) return OS_ERR_INVALID_ARGUMENT;
    g_rtc_seconds = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    g_hal_limit_triggered[0] = false;
    g_hal_limit_triggered[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (axis >= 2) return true;
    return g_hal_limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void) {
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (!data) return OS_ERR_INVALID_ARGUMENT;
    if ((uint32_t)offset + length > sizeof(g_nvm_buffer)) return OS_ERR_INVALID_ARGUMENT;
    memcpy(data, g_nvm_buffer + offset, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (!data) return OS_ERR_INVALID_ARGUMENT;
    if ((uint32_t)offset + length > sizeof(g_nvm_buffer)) return OS_ERR_INVALID_ARGUMENT;
    memcpy(g_nvm_buffer + offset, data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (channel > 3) return OS_ERR_INVALID_ARGUMENT;
    g_comm_enabled[channel] = true;
    g_comm_rx_head[channel] = 0;
    g_comm_rx_tail[channel] = 0;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if (channel > 3 || !g_comm_enabled[channel]) return 0;
    return (int16_t)((g_comm_rx_head[channel] - g_comm_rx_tail[channel]) & 63);
}

char os_hal_comm_read(uint8_t channel) {
    if (channel > 3 || os_hal_comm_available(channel) == 0) return '\0';
    char c = g_comm_rx[channel][g_comm_rx_tail[channel] & 63];
    g_comm_rx_tail[channel]++;
    return c;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (!data || channel > 3) return OS_ERR_INVALID_ARGUMENT;
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) {
    (void)duration_ms;
    if (count == 0) return OS_ERR_INVALID_ARGUMENT;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void) {
    return OS_ERR_NONE;
}
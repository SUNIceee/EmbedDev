#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define STEPS_PER_DEGREE 3600.0f
#define STEPS_PER_HOUR   54000.0f

typedef struct {
    os_equatorial_coord_t star;
    os_motor_position_t motor;
} os_align_point_t;

static os_state_t g_state = OS_STATE_IDLE_TRACKING;
static os_track_rate_t g_track_rate = OS_TRACK_RATE_SIDEREAL;
static float g_custom_track_factor = 1.0f;
static bool g_tracking_enabled = true;

static os_guide_pulse_t g_guide_pulse = {0};
static float g_guide_rate_fraction = 0.5f;

static os_align_mode_t g_align_mode = OS_ALIGN_1STAR;
static os_align_point_t g_align_stars[OS_CALIBRATION_MAX_STARS];
static uint8_t g_align_star_count = 0;
static bool g_align_calculated = false;
static float g_align_residual_arcsec = 0.0f;
static os_calibration_t g_calibration = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, false};

static os_equatorial_coord_t g_park_pos = {0.0f, 90.0f};
static os_motor_position_t g_target_motor_pos = {0, 0};
static os_motor_position_t g_current_motor_pos = {0, 0};
static os_equatorial_coord_t g_target_equatorial = {0.0f, 0.0f};

static bool g_pec_enabled = false;
static os_pec_table_t g_pec_table = {{0}, false};

static float g_custom_move_speed = 15.0f;
static os_site_info_t g_cached_site = {0.0f, 0.0f, 0.0f, 0, false};

static uint8_t g_nvm_storage[512] = {0};
static bool g_motor_initialized[2] = {false, false};
static bool g_motor_enabled[2] = {false, false};
static bool g_limit_triggered[2] = {false, false};
static uint32_t g_rtc_seconds = 1704067200;

static char g_comm_rx[4][128];
static size_t g_comm_rx_len[4] = {0, 0, 0, 0};

static bool validate_equatorial(os_equatorial_coord_t target) {
    if (isnan(target.ra_hours) || isinf(target.ra_hours) ||
        isnan(target.dec_degrees) || isinf(target.dec_degrees)) {
        return false;
    }
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS) {
        return false;
    }
    if (target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return false;
    }
    return true;
}

static void coords_to_steps(os_equatorial_coord_t eq, int32_t *ra_steps, int32_t *dec_steps) {
    double ra_sec = (double)eq.ra_hours * 54000.0;
    double dec_sec = (double)eq.dec_degrees * 3600.0;
    if (g_calibration.valid) {
        double r = (double)g_calibration.matrix_ra_to_ra * ra_sec + (double)g_calibration.matrix_dec_to_ra * dec_sec + (double)g_calibration.offset_ra_arcsec;
        double d = (double)g_calibration.matrix_ra_to_dec * ra_sec + (double)g_calibration.matrix_dec_to_dec * dec_sec + (double)g_calibration.offset_dec_arcsec;
        *ra_steps = (int32_t)lround(r);
        *dec_steps = (int32_t)lround(d);
    } else {
        *ra_steps = (int32_t)lround(ra_sec);
        *dec_steps = (int32_t)lround(dec_sec);
    }
}

static void steps_to_coords(int32_t ra_steps, int32_t dec_steps, os_equatorial_coord_t *eq) {
    double r_sec = (double)ra_steps;
    double d_sec = (double)dec_steps;
    double ra_sec = r_sec;
    double dec_sec = d_sec;
    if (g_calibration.valid) {
        r_sec -= (double)g_calibration.offset_ra_arcsec;
        d_sec -= (double)g_calibration.offset_dec_arcsec;
        double det = (double)g_calibration.matrix_ra_to_ra * (double)g_calibration.matrix_dec_to_dec - (double)g_calibration.matrix_ra_to_dec * (double)g_calibration.matrix_dec_to_ra;
        if (fabs(det) > 1e-9) {
            ra_sec = ((double)g_calibration.matrix_dec_to_dec * r_sec - (double)g_calibration.matrix_dec_to_ra * d_sec) / det;
            dec_sec = (-(double)g_calibration.matrix_ra_to_dec * r_sec + (double)g_calibration.matrix_ra_to_ra * d_sec) / det;
        }
    }
    eq->ra_hours = (float)(ra_sec / 54000.0);
    eq->dec_degrees = (float)(dec_sec / 3600.0);
    if (eq->ra_hours < 0.0f) eq->ra_hours += 24.0f;
    if (eq->ra_hours >= 24.0f) eq->ra_hours -= 24.0f;
    if (eq->dec_degrees < -90.0f) eq->dec_degrees = -90.0f;
    if (eq->dec_degrees > 90.0f) eq->dec_degrees = 90.0f;
}

os_error_t os_init(void) {
    g_state = OS_STATE_INITIALIZING;
    g_track_rate = OS_TRACK_RATE_SIDEREAL;
    g_custom_track_factor = 1.0f;
    g_tracking_enabled = true;

    memset(&g_guide_pulse, 0, sizeof(g_guide_pulse));
    g_guide_rate_fraction = 0.5f;

    g_align_mode = OS_ALIGN_1STAR;
    g_align_star_count = 0;
    g_align_calculated = false;
    g_align_residual_arcsec = 0.0f;
    memset(g_align_stars, 0, sizeof(g_align_stars));

    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.matrix_ra_to_dec = 0.0f;
    g_calibration.matrix_dec_to_ra = 0.0f;
    g_calibration.offset_ra_arcsec = 0.0f;
    g_calibration.offset_dec_arcsec = 0.0f;
    g_calibration.valid = false;

    g_target_motor_pos.ra_steps = 0;
    g_target_motor_pos.dec_steps = 0;
    g_current_motor_pos.ra_steps = 0;
    g_current_motor_pos.dec_steps = 0;

    g_pec_enabled = false;
    memset(&g_pec_table, 0, sizeof(g_pec_table));

    os_hal_nvm_init();
    for (uint8_t c = 0; c < 4; c++) {
        os_hal_comm_init(c);
    }
    os_hal_motor_init(0);
    os_hal_motor_init(1);
    os_hal_timer_motor_init();
    os_hal_limit_init();
    os_hal_gps_init();
    os_hal_rtc_init();

    os_calibration_t loaded_cal;
    if (os_hal_nvm_read(0, (uint8_t *)&loaded_cal, sizeof(loaded_cal)) == OS_ERR_NONE) {
        if (loaded_cal.valid) {
            g_calibration = loaded_cal;
            g_align_calculated = true;
        }
    }

    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);

    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    for (uint8_t ch = 0; ch < 4; ch++) {
        while (os_hal_comm_available(ch) > 0) {
            char c = os_hal_comm_read(ch);
            if (c == OS_LX200_CMD_PREFIX) {
                g_comm_rx_len[ch] = 0;
                g_comm_rx[ch][g_comm_rx_len[ch]++] = c;
            } else if (g_comm_rx_len[ch] > 0) {
                if (g_comm_rx_len[ch] < sizeof(g_comm_rx[ch]) - 1) {
                    g_comm_rx[ch][g_comm_rx_len[ch]++] = c;
                }
                if (c == OS_LX200_CMD_SUFFIX) {
                    g_comm_rx[ch][g_comm_rx_len[ch]] = '\0';
                    char reply[OS_MAX_REPLY_LENGTH];
                    size_t reply_len = 0;
                    os_command_parse(g_comm_rx[ch], g_comm_rx_len[ch], ch, reply, sizeof(reply), &reply_len);
                    if (reply_len > 0) {
                        os_hal_comm_write(ch, reply, reply_len);
                    }
                    g_comm_rx_len[ch] = 0;
                }
            }
        }
    }

    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        if (g_state == OS_STATE_GOTO || g_state == OS_STATE_MANUAL_MOTION) {
            g_state = OS_STATE_FAULT;
            os_hal_motor_set_frequency(0, 0);
            os_hal_motor_set_frequency(1, 0);
            return;
        }
    }

    if (g_state == OS_STATE_GOTO) {
        int32_t diff_ra = g_target_motor_pos.ra_steps - g_current_motor_pos.ra_steps;
        int32_t diff_dec = g_target_motor_pos.dec_steps - g_current_motor_pos.dec_steps;
        int32_t step_ra = (diff_ra > 0) ? 100 : ((diff_ra < 0) ? -100 : 0);
        int32_t step_dec = (diff_dec > 0) ? 100 : ((diff_dec < 0) ? -100 : 0);

        if (labs(diff_ra) <= 100) g_current_motor_pos.ra_steps = g_target_motor_pos.ra_steps;
        else g_current_motor_pos.ra_steps += step_ra;

        if (labs(diff_dec) <= 100) g_current_motor_pos.dec_steps = g_target_motor_pos.dec_steps;
        else g_current_motor_pos.dec_steps += step_dec;

        if (g_current_motor_pos.ra_steps == g_target_motor_pos.ra_steps &&
            g_current_motor_pos.dec_steps == g_target_motor_pos.dec_steps) {
            g_state = OS_STATE_IDLE_TRACKING;
            os_hal_buzzer_beep(100, 1);
        }
    }

    if (g_guide_pulse.active) {
        if (g_guide_pulse.duration_ms > 10) {
            g_guide_pulse.duration_ms -= 10;
        } else {
            g_guide_pulse.duration_ms = 0;
            g_guide_pulse.active = false;
        }
    }
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (!command || !reply_buffer || !reply_length) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (source_channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length == 0 || command[0] != OS_LX200_CMD_PREFIX || command[length - 1] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    reply_buffer[0] = '\0';
    *reply_length = 0;

    if (strncmp(command, ":GR#", 4) == 0) {
        os_equatorial_coord_t eq;
        steps_to_coords(g_current_motor_pos.ra_steps, g_current_motor_pos.dec_steps, &eq);
        int hrs = (int)eq.ra_hours;
        int mins = (int)((eq.ra_hours - hrs) * 60.0f);
        int secs = (int)((((eq.ra_hours - hrs) * 60.0f) - mins) * 60.0f);
        snprintf(reply_buffer, reply_buffer_size, "%02d:%02d:%02d#", hrs, mins, secs);
    } else if (strncmp(command, ":GD#", 4) == 0) {
        os_equatorial_coord_t eq;
        steps_to_coords(g_current_motor_pos.ra_steps, g_current_motor_pos.dec_steps, &eq);
        char sign = (eq.dec_degrees >= 0.0f) ? '+' : '-';
        float abs_deg = fabsf(eq.dec_degrees);
        int degs = (int)abs_deg;
        int mins = (int)((abs_deg - degs) * 60.0f);
        int secs = (int)((((abs_deg - degs) * 60.0f) - mins) * 60.0f);
        snprintf(reply_buffer, reply_buffer_size, "%c%02d*%02d'%02d#", sign, degs, mins, secs);
    } else if (strncmp(command, ":GVP#", 5) == 0) {
        snprintf(reply_buffer, reply_buffer_size, "OnStep %d.%d.%d#", OS_FIRMWARE_VERSION_MAJOR, OS_FIRMWARE_VERSION_MINOR, OS_FIRMWARE_VERSION_PATCH);
    } else if (strncmp(command, ":MS#", 4) == 0) {
        os_error_t err = os_goto_equatorial(g_target_equatorial);
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "0");
        } else {
            snprintf(reply_buffer, reply_buffer_size, "1");
        }
    } else if (strncmp(command, ":hP#", 4) == 0) {
        os_error_t err = os_park();
        snprintf(reply_buffer, reply_buffer_size, "%d", (err == OS_ERR_NONE) ? 1 : 0);
    } else if (strncmp(command, ":hO#", 4) == 0) {
        os_error_t err = os_unpark();
        snprintf(reply_buffer, reply_buffer_size, "%d", (err == OS_ERR_NONE) ? 1 : 0);
    } else {
        snprintf(reply_buffer, reply_buffer_size, "0");
    }

    *reply_length = strlen(reply_buffer);
    return OS_ERR_NONE;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (!validate_equatorial(target)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (g_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    g_target_equatorial = target;
    coords_to_steps(target, &g_target_motor_pos.ra_steps, &g_target_motor_pos.dec_steps);

    if (g_target_motor_pos.ra_steps == g_current_motor_pos.ra_steps &&
        g_target_motor_pos.dec_steps == g_current_motor_pos.dec_steps) {
        return OS_ERR_NONE;
    }

    g_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (isnan(target.azimuth_degrees) || target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        isnan(target.altitude_degrees) || target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (g_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    g_target_motor_pos.ra_steps = (int32_t)lround(target.azimuth_degrees * STEPS_PER_DEGREE);
    g_target_motor_pos.dec_steps = (int32_t)lround(target.altitude_degrees * STEPS_PER_DEGREE);
    g_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (g_state == OS_STATE_GOTO) {
        g_target_motor_pos = g_current_motor_pos;
        g_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (rate < OS_TRACK_RATE_SIDEREAL || rate > OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM && (isnan(custom_factor) || custom_factor <= 0.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_track_rate = rate;
    g_custom_track_factor = custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (!rate || !custom_factor) {
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
    g_guide_pulse.dec_priority = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (isnan(rate_fraction) || rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_guide_rate_fraction = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (!pulse) {
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
    g_align_calculated = false;
    g_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (!validate_equatorial(star_coord)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_align_star_count < OS_CALIBRATION_MAX_STARS) {
        g_align_stars[g_align_star_count].star = star_coord;
        g_align_stars[g_align_star_count].motor = motor_pos;
        g_align_star_count++;
    }
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    uint8_t min_stars = 1;
    if (g_align_mode == OS_ALIGN_2STAR) min_stars = 2;
    else if (g_align_mode >= OS_ALIGN_3STAR) min_stars = 3;

    if (g_align_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    if (g_align_star_count == 1) {
        double ra_sec = (double)g_align_stars[0].star.ra_hours * 54000.0;
        double dec_sec = (double)g_align_stars[0].star.dec_degrees * 3600.0;
        g_calibration.matrix_ra_to_ra = 1.0f;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = 1.0f;
        g_calibration.offset_ra_arcsec = (float)((double)g_align_stars[0].motor.ra_steps - ra_sec);
        g_calibration.offset_dec_arcsec = (float)((double)g_align_stars[0].motor.dec_steps - dec_sec);
        g_calibration.valid = true;
        g_align_residual_arcsec = 0.0f;
    } else if (g_align_star_count == 2) {
        double x0 = (double)g_align_stars[0].star.ra_hours * 54000.0;
        double y0 = (double)g_align_stars[0].star.dec_degrees * 3600.0;
        double x1 = (double)g_align_stars[1].star.ra_hours * 54000.0;
        double y1 = (double)g_align_stars[1].star.dec_degrees * 3600.0;
        double u0 = (double)g_align_stars[0].motor.ra_steps;
        double v0 = (double)g_align_stars[0].motor.dec_steps;
        double u1 = (double)g_align_stars[1].motor.ra_steps;
        double v1 = (double)g_align_stars[1].motor.dec_steps;

        double m_ra = (fabs(x1 - x0) > 1e-6) ? (u1 - u0) / (x1 - x0) : 1.0;
        double m_dec = (fabs(y1 - y0) > 1e-6) ? (v1 - v0) / (y1 - y0) : 1.0;

        g_calibration.matrix_ra_to_ra = (float)m_ra;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = (float)m_dec;
        g_calibration.offset_ra_arcsec = (float)(u0 - m_ra * x0);
        g_calibration.offset_dec_arcsec = (float)(v0 - m_dec * y0);
        g_calibration.valid = true;
        g_align_residual_arcsec = 0.0f;
    } else {
        double sX = 0, sY = 0, sX2 = 0, sY2 = 0, sXY = 0;
        double sU = 0, sV = 0, sXU = 0, sYU = 0, sXV = 0, sYV = 0;
        double N = (double)g_align_star_count;

        for (uint8_t i = 0; i < g_align_star_count; i++) {
            double x = (double)g_align_stars[i].star.ra_hours * 54000.0;
            double y = (double)g_align_stars[i].star.dec_degrees * 3600.0;
            double u = (double)g_align_stars[i].motor.ra_steps;
            double v = (double)g_align_stars[i].motor.dec_steps;

            sX += x; sY += y; sX2 += x*x; sY2 += y*y; sXY += x*y;
            sU += u; sV += v; sXU += x*u; sYU += y*u; sXV += x*v; sYV += y*v;
        }

        double det = sX2 * (sY2 * N - sY * sY) - sXY * (sXY * N - sY * sX) + sX * (sXY * sY - sY2 * sX);
        if (fabs(det) < 1e-6) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        double inv00 = (sY2 * N - sY * sY) / det;
        double inv01 = -(sXY * N - sY * sX) / det;
        double inv02 = (sXY * sY - sY2 * sX) / det;
        double inv10 = inv01;
        double inv11 = (sX2 * N - sX * sX) / det;
        double inv12 = -(sX2 * sY - sXY * sX) / det;
        double inv20 = inv02;
        double inv21 = inv12;
        double inv22 = (sX2 * sY2 - sXY * sXY) / det;

        double a = inv00 * sXU + inv01 * sYU + inv02 * sU;
        double b = inv10 * sXU + inv11 * sYU + inv12 * sU;
        double c = inv20 * sXU + inv21 * sYU + inv22 * sU;

        double d = inv00 * sXV + inv01 * sYV + inv02 * sV;
        double e = inv10 * sXV + inv11 * sYV + inv12 * sV;
        double f = inv20 * sXV + inv21 * sYV + inv22 * sV;

        g_calibration.matrix_ra_to_ra = (float)a;
        g_calibration.matrix_dec_to_ra = (float)b;
        g_calibration.offset_ra_arcsec = (float)c;
        g_calibration.matrix_ra_to_dec = (float)d;
        g_calibration.matrix_dec_to_dec = (float)e;
        g_calibration.offset_dec_arcsec = (float)f;
        g_calibration.valid = true;

        if (g_align_star_count == 3) {
            g_align_residual_arcsec = 0.0f;
        } else {
            double sum_sq = 0.0;
            for (uint8_t i = 0; i < g_align_star_count; i++) {
                double x = (double)g_align_stars[i].star.ra_hours * 54000.0;
                double y = (double)g_align_stars[i].star.dec_degrees * 3600.0;
                double u = (double)g_align_stars[i].motor.ra_steps;
                double v = (double)g_align_stars[i].motor.dec_steps;
                double res_u = a * x + b * y + c - u;
                double res_v = d * x + e * y + f - v;
                sum_sq += res_u * res_u + res_v * res_v;
            }
            g_align_residual_arcsec = (float)sqrt(sum_sq / (double)g_align_star_count);
            if (g_align_residual_arcsec > 300.0f) {
                g_calibration.valid = false;
                return OS_ERR_CALIBRATION_FAILED;
            }
        }
    }

    g_align_calculated = true;
    g_state = OS_STATE_IDLE_TRACKING;
    os_hal_nvm_write(0, (const uint8_t *)&g_calibration, sizeof(g_calibration));
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (!residual_arcsec) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_align_calculated || !g_calibration.valid) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = g_align_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    g_align_star_count = 0;
    if (g_state == OS_STATE_ALIGNMENT) {
        g_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    coords_to_steps(g_park_pos, &g_target_motor_pos.ra_steps, &g_target_motor_pos.dec_steps);
    g_current_motor_pos = g_target_motor_pos;
    g_state = OS_STATE_PARKED;
    g_tracking_enabled = false;
    os_hal_motor_enable(0, false);
    os_hal_motor_enable(1, false);
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);
    for (uint8_t c = 0; c < 4; c++) {
        os_hal_comm_init(c);
    }
    os_hal_rtc_read(&g_rtc_seconds);
    g_tracking_enabled = true;
    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!validate_equatorial(park_pos)) {
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
    uint8_t axis = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) ? 0 : 1;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    g_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (g_state == OS_STATE_MANUAL_MOTION) {
        g_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (isnan(arcsec_per_sec) || arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_custom_move_speed = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (!state) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = g_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (!coord) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    steps_to_coords(g_current_motor_pos.ra_steps, g_current_motor_pos.dec_steps, coord);
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (!site) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    os_site_info_t poll_site;
    if (os_hal_gps_poll(&poll_site) == OS_ERR_NONE && poll_site.valid) {
        g_cached_site = poll_site;
    } else {
        g_cached_site.utc_epoch_seconds = g_rtc_seconds;
    }
    *site = g_cached_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (!pos) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = os_hal_motor_get_position(0);
    pos->dec_steps = os_hal_motor_get_position(1);
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
    *moving = (g_state == OS_STATE_GOTO || g_state == OS_STATE_MANUAL_MOTION);
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (!locked) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    os_site_info_t poll_site;
    if (os_hal_gps_poll(&poll_site) == OS_ERR_NONE) {
        *locked = poll_site.valid;
    } else {
        *locked = false;
    }
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    g_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (!table) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_pec_table = *table;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (!table) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *table = g_pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (isnan(worm_phase_deg) || worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int idx = (int)floorf(worm_phase_deg);
    if (idx >= 360) idx = 0;
    g_pec_table.corrections[idx] = error_arcsec;
    g_pec_table.valid = true;
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (!calib) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *calib = g_calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.matrix_ra_to_dec = 0.0f;
    g_calibration.matrix_dec_to_ra = 0.0f;
    g_calibration.offset_ra_arcsec = 0.0f;
    g_calibration.offset_dec_arcsec = 0.0f;
    g_calibration.valid = false;
    g_align_calculated = false;
    os_hal_nvm_write(0, (const uint8_t *)&g_calibration, sizeof(g_calibration));
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_init(uint8_t axis) {
    if (axis > 1) return OS_ERR_INVALID_ARGUMENT;
    g_motor_initialized[axis] = true;
    g_motor_enabled[axis] = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (axis > 1) return OS_ERR_INVALID_ARGUMENT;
    (void)frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (axis > 1) return OS_ERR_INVALID_ARGUMENT;
    (void)forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (axis > 1) return OS_ERR_INVALID_ARGUMENT;
    g_motor_enabled[axis] = enable;
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (axis == 0) return g_current_motor_pos.ra_steps;
    if (axis == 1) return g_current_motor_pos.dec_steps;
    return 0;
}

os_error_t os_hal_gps_init(void) {
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (!site) return OS_ERR_INVALID_ARGUMENT;
    site->valid = false;
    return OS_ERR_GPS_NO_SIGNAL;
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
    g_rtc_seconds = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    g_limit_triggered[0] = false;
    g_limit_triggered[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (axis > 1) return true;
    return g_limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void) {
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (!data || (uint32_t)offset + length > sizeof(g_nvm_storage)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(data, &g_nvm_storage[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (!data || (uint32_t)offset + length > sizeof(g_nvm_storage)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(&g_nvm_storage[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (channel > 3) return OS_ERR_INVALID_ARGUMENT;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if (channel > 3) return OS_ERR_NOT_SUPPORTED;
    return 0;
}

char os_hal_comm_read(uint8_t channel) {
    (void)channel;
    return '\0';
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (!data || channel > 3) return OS_ERR_INVALID_ARGUMENT;
    (void)length;
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) {
    (void)duration_ms;
    (void)count;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void) {
    return OS_ERR_NONE;
}
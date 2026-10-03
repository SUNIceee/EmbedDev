#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define STEPS_PER_DEGREE 3600
#define NVM_OFFSET_CALIBRATION 0
#define NVM_OFFSET_PARK 256
#define NVM_OFFSET_PEC 512

static os_state_t g_system_state = OS_STATE_INITIALIZING;
static os_mount_type_t g_mount_type = OS_MOUNT_EQUATORIAL;
static bool g_tracking_enabled = true;
static os_track_rate_t g_tracking_rate = OS_TRACK_RATE_SIDEREAL;
static float g_custom_rate_factor = 1.0f;
static float g_custom_slew_speed = 3600.0f;
static os_site_info_t g_site_info = {0.0f, 0.0f, 0.0f, 0, false};
static os_calibration_t g_calibration = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, false};
static os_align_mode_t g_align_mode = OS_ALIGN_1STAR;
static uint8_t g_align_star_count = 0;
static os_equatorial_coord_t g_align_star_coords[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t g_align_motor_positions[OS_CALIBRATION_MAX_STARS];
static float g_calibration_residual = 0.0f;
static os_equatorial_coord_t g_park_pos = {0.0f, 90.0f};
static bool g_park_state = false;
static bool g_is_parking = false;
static os_guide_pulse_t g_guide_pulse = {false, 0, 0.5f, false, false, false};
static uint32_t g_guide_end_tick = 0;
static os_pec_table_t g_pec_table = {{0}, false};
static bool g_pec_enabled = false;
static double g_worm_phase_deg = 0.0;
static int32_t g_motor_pos[2] = {0, 0};
static int32_t g_target_motor_pos[2] = {0, 0};
static bool g_motor_enabled[2] = {false, false};
static bool g_motor_dir[2] = {true, true};
static uint32_t g_motor_freq[2] = {0, 0};
static uint32_t g_system_tick = 0;
static char g_comm_rx_buf[4][OS_MAX_COMMAND_LENGTH];
static size_t g_comm_rx_len[4] = {0, 0, 0, 0};
static uint8_t g_hal_nvm_sim[1024];
static bool g_hal_nvm_inited = false;

static void coords_to_steps(float ra_hours, float dec_deg, int32_t *ra_steps, int32_t *dec_steps) {
    double ra_deg = (double)ra_hours * 15.0;
    double dec = (double)dec_deg;
    if (g_calibration.valid) {
        double ra_corr = (double)g_calibration.matrix_ra_to_ra * ra_deg + (double)g_calibration.matrix_dec_to_ra * dec + (double)g_calibration.offset_ra_arcsec / 3600.0;
        double dec_corr = (double)g_calibration.matrix_ra_to_dec * ra_deg + (double)g_calibration.matrix_dec_to_dec * dec + (double)g_calibration.offset_dec_arcsec / 3600.0;
        ra_deg = ra_corr;
        dec = dec_corr;
    }
    *ra_steps = (int32_t)lround(ra_deg * STEPS_PER_DEGREE);
    *dec_steps = (int32_t)lround(dec * STEPS_PER_DEGREE);
}

static void steps_to_coords(int32_t ra_steps, int32_t dec_steps, float *ra_hours, float *dec_deg) {
    double u = (double)ra_steps / (double)STEPS_PER_DEGREE;
    double v = (double)dec_steps / (double)STEPS_PER_DEGREE;
    double ra_deg = u;
    double dec = v;
    if (g_calibration.valid) {
        double u_off = u - (double)g_calibration.offset_ra_arcsec / 3600.0;
        double v_off = v - (double)g_calibration.offset_dec_arcsec / 3600.0;
        double m_r_r = (double)g_calibration.matrix_ra_to_ra;
        double m_d_r = (double)g_calibration.matrix_dec_to_ra;
        double m_r_d = (double)g_calibration.matrix_ra_to_dec;
        double m_d_d = (double)g_calibration.matrix_dec_to_dec;
        double det = m_r_r * m_d_d - m_d_r * m_r_d;
        if (fabs(det) > 1e-9) {
            ra_deg = (m_d_d * u_off - m_d_r * v_off) / det;
            dec = (-m_r_d * u_off + m_r_r * v_off) / det;
        } else {
            ra_deg = u_off;
            dec = v_off;
        }
    }
    double ra_h = ra_deg / 15.0;
    while (ra_h < 0.0) ra_h += 24.0;
    while (ra_h >= 24.0) ra_h -= 24.0;
    if (dec > 90.0) dec = 90.0;
    if (dec < -90.0) dec = -90.0;
    *ra_hours = (float)ra_h;
    *dec_deg = (float)dec;
}

os_error_t os_init(void) {
    g_system_state = OS_STATE_INITIALIZING;
    g_mount_type = OS_MOUNT_EQUATORIAL;
    g_tracking_enabled = true;
    g_tracking_rate = OS_TRACK_RATE_SIDEREAL;
    g_custom_rate_factor = 1.0f;
    g_custom_slew_speed = 3600.0f;
    g_align_star_count = 0;
    g_calibration_residual = 0.0f;
    g_calibration.valid = false;
    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_ra_to_dec = 0.0f;
    g_calibration.matrix_dec_to_ra = 0.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.offset_ra_arcsec = 0.0f;
    g_calibration.offset_dec_arcsec = 0.0f;
    g_guide_pulse.active = false;
    g_guide_pulse.duration_ms = 0;
    g_park_state = false;
    g_is_parking = false;
    g_pec_enabled = false;
    g_pec_table.valid = false;
    g_worm_phase_deg = 0.0;
    g_system_tick = 0;
    for (int i = 0; i < 4; i++) {
        g_comm_rx_len[i] = 0;
    }

    if (os_hal_nvm_init() != OS_ERR_NONE) {
        g_system_state = OS_STATE_FAULT;
        return OS_ERR_NVM_FAULT;
    }

    os_calibration_t cal_nvm;
    if (os_hal_nvm_read(NVM_OFFSET_CALIBRATION, (uint8_t*)&cal_nvm, sizeof(cal_nvm)) == OS_ERR_NONE) {
        if (cal_nvm.valid) {
            g_calibration = cal_nvm;
        }
    }

    for (uint8_t ch = 0; ch <= OS_CHANNEL_ETHERNET; ch++) {
        os_hal_comm_init(ch);
    }

    if (os_hal_motor_init(0) != OS_ERR_NONE || os_hal_motor_init(1) != OS_ERR_NONE) {
        g_system_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    os_hal_limit_init();
    os_hal_gps_init();
    os_hal_rtc_init();
    os_hal_timer_motor_init();

    os_site_info_t site;
    if (os_hal_gps_poll(&site) == OS_ERR_NONE && site.valid) {
        g_site_info = site;
    } else {
        uint32_t utc_sec = 0;
        if (os_hal_rtc_read(&utc_sec) == OS_ERR_NONE) {
            g_site_info.utc_epoch_seconds = utc_sec;
        }
    }

    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);

    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    g_system_tick++;

    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        os_hal_motor_set_frequency(0, 0);
        os_hal_motor_set_frequency(1, 0);
        g_system_state = OS_STATE_FAULT;
    }

    for (uint8_t ch = 0; ch <= OS_CHANNEL_ETHERNET; ch++) {
        int16_t avail = os_hal_comm_available(ch);
        while (avail > 0) {
            char c = os_hal_comm_read(ch);
            avail--;
            if (c == OS_LX200_CMD_PREFIX) {
                g_comm_rx_len[ch] = 0;
                g_comm_rx_buf[ch][g_comm_rx_len[ch]++] = c;
            } else if (g_comm_rx_len[ch] > 0) {
                if (g_comm_rx_len[ch] < OS_MAX_COMMAND_LENGTH - 1) {
                    g_comm_rx_buf[ch][g_comm_rx_len[ch]++] = c;
                }
                if (c == OS_LX200_CMD_SUFFIX) {
                    g_comm_rx_buf[ch][g_comm_rx_len[ch]] = '\0';
                    char reply[OS_MAX_REPLY_LENGTH];
                    size_t reply_len = 0;
                    os_command_parse(g_comm_rx_buf[ch], g_comm_rx_len[ch], ch, reply, sizeof(reply), &reply_len);
                    if (reply_len > 0) {
                        os_hal_comm_write(ch, reply, reply_len);
                    }
                    g_comm_rx_len[ch] = 0;
                }
            }
        }
    }

    if (g_system_state == OS_STATE_GOTO) {
        bool ra_arrived = false;
        bool dec_arrived = false;

        for (uint8_t axis = 0; axis < 2; axis++) {
            int32_t current_pos = os_hal_motor_get_position(axis);
            int32_t diff = g_target_motor_pos[axis] - current_pos;
            if (abs(diff) <= 5) {
                os_hal_motor_set_frequency(axis, 0);
                if (axis == 0) ra_arrived = true;
                else dec_arrived = true;
            } else {
                os_hal_motor_set_direction(axis, diff > 0);
                uint32_t speed = 3600;
                if (abs(diff) < 3600) {
                    speed = (uint32_t)abs(diff);
                    if (speed < 100) speed = 100;
                }
                os_hal_motor_set_frequency(axis, speed);
                if (diff > 0) g_motor_pos[axis] += (int32_t)(speed / 100 + 1);
                else g_motor_pos[axis] -= (int32_t)(speed / 100 + 1);
            }
        }
        if (ra_arrived && dec_arrived) {
            if (g_is_parking) {
                os_hal_motor_enable(0, false);
                os_hal_motor_enable(1, false);
                g_tracking_enabled = false;
                g_system_state = OS_STATE_PARKED;
                g_park_state = true;
                g_is_parking = false;
            } else {
                os_hal_buzzer_beep(200, 1);
                g_system_state = OS_STATE_IDLE_TRACKING;
            }
        }
    }

    if (g_guide_pulse.active) {
        if (g_system_tick >= g_guide_end_tick) {
            g_guide_pulse.active = false;
        }
    }

    os_site_info_t site;
    if (os_hal_gps_poll(&site) == OS_ERR_NONE && site.valid) {
        g_site_info = site;
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

    reply_buffer[0] = '\0';
    *reply_length = 0;

    if (strncmp(command, ":GR#", 4) == 0) {
        os_equatorial_coord_t coord;
        os_query_coordinates(&coord);
        int hrs = (int)coord.ra_hours;
        int mins = (int)((coord.ra_hours - hrs) * 60.0f);
        int secs = (int)(((coord.ra_hours - hrs) * 60.0f - mins) * 60.0f);
        snprintf(reply_buffer, reply_buffer_size, "%02d:%02d:%02d#", hrs, mins, secs);
        *reply_length = strlen(reply_buffer);
        return OS_ERR_NONE;
    } else if (strncmp(command, ":GD#", 4) == 0) {
        os_equatorial_coord_t coord;
        os_query_coordinates(&coord);
        int deg = (int)coord.dec_degrees;
        int mins = (int)(fabsf(coord.dec_degrees - deg) * 60.0f);
        snprintf(reply_buffer, reply_buffer_size, "%+03d*%02d#", deg, mins);
        *reply_length = strlen(reply_buffer);
        return OS_ERR_NONE;
    } else if (strncmp(command, ":GVP#", 5) == 0) {
        snprintf(reply_buffer, reply_buffer_size, "OnStep %d.%d.%d#", OS_FIRMWARE_VERSION_MAJOR, OS_FIRMWARE_VERSION_MINOR, OS_FIRMWARE_VERSION_PATCH);
        *reply_length = strlen(reply_buffer);
        return OS_ERR_NONE;
    } else if (strncmp(command, ":hP#", 4) == 0) {
        os_error_t err = os_park();
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1#");
        } else {
            snprintf(reply_buffer, reply_buffer_size, "0#");
        }
        *reply_length = strlen(reply_buffer);
        return err;
    } else if (strncmp(command, ":hO#", 4) == 0) {
        os_error_t err = os_unpark();
        if (err == OS_ERR_NONE) {
            snprintf(reply_buffer, reply_buffer_size, "1#");
        } else {
            snprintf(reply_buffer, reply_buffer_size, "0#");
        }
        *reply_length = strlen(reply_buffer);
        return err;
    } else if (strncmp(command, ":Q#", 3) == 0) {
        os_move_stop();
        snprintf(reply_buffer, reply_buffer_size, "1#");
        *reply_length = strlen(reply_buffer);
        return OS_ERR_NONE;
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
    if (g_system_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    coords_to_steps(target.ra_hours, target.dec_degrees, &g_target_motor_pos[0], &g_target_motor_pos[1]);
    g_is_parking = false;
    g_system_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < OS_DEC_MIN_DEG || target.altitude_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
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
    g_is_parking = false;
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
    g_tracking_rate = rate;
    g_custom_rate_factor = custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = g_tracking_rate;
    *custom_factor = g_custom_rate_factor;
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
    os_hal_motor_set_frequency(0, 0);
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST || duration_ms == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    g_guide_pulse.active = true;
    g_guide_pulse.duration_ms = duration_ms;
    g_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    g_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    g_guide_end_tick = g_system_tick + duration_ms;
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
        star_coord.dec_degrees < OS_DEC_MIN_DEG || star_coord.dec_degrees > OS_DEC_MAX_DEG ||
        g_align_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    g_align_star_coords[g_align_star_count] = star_coord;
    g_align_motor_positions[g_align_star_count] = motor_pos;
    g_align_star_count++;
    return OS_ERR_NONE;
}

static bool solve_2d_affine_axis(int n, const double *x, const double *y, const double *z, double *out_a, double *out_b, double *out_c) {
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    for (int i = 0; i < n; i++) {
        sum_x += x[i];
        sum_y += y[i];
        sum_z += z[i];
    }
    double mean_x = sum_x / (double)n;
    double mean_y = sum_y / (double)n;
    double mean_z = sum_z / (double)n;

    double s_xx = 0.0, s_yy = 0.0, s_xy = 0.0, s_xz = 0.0, s_yz = 0.0;
    for (int i = 0; i < n; i++) {
        double dx = x[i] - mean_x;
        double dy = y[i] - mean_y;
        double dz = z[i] - mean_z;
        s_xx += dx * dx;
        s_yy += dy * dy;
        s_xy += dx * dy;
        s_xz += dx * dz;
        s_yz += dy * dz;
    }

    double det = s_xx * s_yy - s_xy * s_xy;
    if (fabs(det) < 1e-9) {
        return false;
    }

    double a = (s_xz * s_yy - s_yz * s_xy) / det;
    double b = (s_yz * s_xx - s_xz * s_xy) / det;
    double c = mean_z - a * mean_x - b * mean_y;

    *out_a = a;
    *out_b = b;
    *out_c = c;
    return true;
}

os_error_t os_align_compute(void) {
    if (g_system_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    uint8_t min_stars = 1;
    if (g_align_mode == OS_ALIGN_2STAR) min_stars = 2;
    else if (g_align_mode == OS_ALIGN_3STAR || g_align_mode == OS_ALIGN_NSTAR) min_stars = 3;

    if (g_align_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    int n = g_align_star_count;
    if (n == 1) {
        double ra_sky = (double)g_align_star_coords[0].ra_hours * 15.0;
        double dec_sky = (double)g_align_star_coords[0].dec_degrees;
        double ra_mot = (double)g_align_motor_positions[0].ra_steps / (double)STEPS_PER_DEGREE;
        double dec_mot = (double)g_align_motor_positions[0].dec_steps / (double)STEPS_PER_DEGREE;
        g_calibration.matrix_ra_to_ra = 1.0f;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = 1.0f;
        g_calibration.offset_ra_arcsec = (float)((ra_mot - ra_sky) * 3600.0);
        g_calibration.offset_dec_arcsec = (float)((dec_mot - dec_sky) * 3600.0);
        g_calibration_residual = 0.0f;
    } else if (n == 2) {
        double sum_x = 0, sum_y = 0, sum_u = 0, sum_v = 0;
        double sum_xx = 0, sum_yy = 0, sum_xu = 0, sum_yv = 0;
        for (int i = 0; i < n; i++) {
            double x = (double)g_align_star_coords[i].ra_hours * 15.0;
            double y = (double)g_align_star_coords[i].dec_degrees;
            double u = (double)g_align_motor_positions[i].ra_steps / (double)STEPS_PER_DEGREE;
            double v = (double)g_align_motor_positions[i].dec_steps / (double)STEPS_PER_DEGREE;
            sum_x += x; sum_y += y; sum_u += u; sum_v += v;
            sum_xx += x * x; sum_yy += y * y; sum_xu += x * u; sum_yv += y * v;
        }
        double mean_x = sum_x / (double)n, mean_y = sum_y / (double)n;
        double mean_u = sum_u / (double)n, mean_v = sum_v / (double)n;
        double s_xx = sum_xx - (double)n * mean_x * mean_x;
        double s_yy = sum_yy - (double)n * mean_y * mean_y;
        double s_xu = sum_xu - (double)n * mean_x * mean_u;
        double s_yv = sum_yv - (double)n * mean_y * mean_v;
        if (fabs(s_xx) < 1e-12 || fabs(s_yy) < 1e-12) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        double m_r_r = s_xu / s_xx;
        double m_d_d = s_yv / s_yy;
        double o_r = mean_u - m_r_r * mean_x;
        double o_d = mean_v - m_d_d * mean_y;

        g_calibration.matrix_ra_to_ra = (float)m_r_r;
        g_calibration.matrix_ra_to_dec = 0.0f;
        g_calibration.matrix_dec_to_ra = 0.0f;
        g_calibration.matrix_dec_to_dec = (float)m_d_d;
        g_calibration.offset_ra_arcsec = (float)(o_r * 3600.0);
        g_calibration.offset_dec_arcsec = (float)(o_d * 3600.0);
        g_calibration_residual = 0.0f;
    } else {
        double x[OS_CALIBRATION_MAX_STARS], y[OS_CALIBRATION_MAX_STARS];
        double u[OS_CALIBRATION_MAX_STARS], v[OS_CALIBRATION_MAX_STARS];
        for (int i = 0; i < n; i++) {
            x[i] = (double)g_align_star_coords[i].ra_hours * 15.0;
            y[i] = (double)g_align_star_coords[i].dec_degrees;
            u[i] = (double)g_align_motor_positions[i].ra_steps / (double)STEPS_PER_DEGREE;
            v[i] = (double)g_align_motor_positions[i].dec_steps / (double)STEPS_PER_DEGREE;
        }
        double m_r_r = 0.0, m_d_r = 0.0, o_r = 0.0;
        double m_r_d = 0.0, m_d_d = 0.0, o_d = 0.0;
        if (!solve_2d_affine_axis(n, x, y, u, &m_r_r, &m_d_r, &o_r) ||
            !solve_2d_affine_axis(n, x, y, v, &m_r_d, &m_d_d, &o_d)) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        g_calibration.matrix_ra_to_ra = (float)m_r_r;
        g_calibration.matrix_dec_to_ra = (float)m_d_r;
        g_calibration.matrix_ra_to_dec = (float)m_r_d;
        g_calibration.matrix_dec_to_dec = (float)m_d_d;
        g_calibration.offset_ra_arcsec = (float)(o_r * 3600.0);
        g_calibration.offset_dec_arcsec = (float)(o_d * 3600.0);

        if (n == 3) {
            g_calibration_residual = 0.0f;
        } else {
            double sum_sq_err = 0.0;
            for (int i = 0; i < n; i++) {
                double u_pred = m_r_r * x[i] + m_d_r * y[i] + o_r;
                double v_pred = m_r_d * x[i] + m_d_d * y[i] + o_d;
                double du = u[i] - u_pred;
                double dv = v[i] - v_pred;
                sum_sq_err += (du * du + dv * dv);
            }
            double rms_deg = sqrt(sum_sq_err / (double)n);
            g_calibration_residual = (float)(rms_deg * 3600.0);
        }
    }

    g_calibration.valid = true;
    os_hal_nvm_write(NVM_OFFSET_CALIBRATION, (const uint8_t*)&g_calibration, sizeof(g_calibration));
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_calibration.valid) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = g_calibration_residual;
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
    if (g_system_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    coords_to_steps(g_park_pos.ra_hours, g_park_pos.dec_degrees, &g_target_motor_pos[0], &g_target_motor_pos[1]);
    int32_t diff0 = g_target_motor_pos[0] - os_hal_motor_get_position(0);
    int32_t diff1 = g_target_motor_pos[1] - os_hal_motor_get_position(1);
    if (abs(diff0) <= 5 && abs(diff1) <= 5) {
        os_hal_motor_enable(0, false);
        os_hal_motor_enable(1, false);
        g_tracking_enabled = false;
        g_system_state = OS_STATE_PARKED;
        g_park_state = true;
        g_is_parking = false;
    } else {
        g_is_parking = true;
        g_system_state = OS_STATE_GOTO;
    }
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (g_system_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);
    for (uint8_t ch = 0; ch <= OS_CHANNEL_ETHERNET; ch++) {
        os_hal_comm_init(ch);
    }
    uint32_t utc_sec = 0;
    if (os_hal_rtc_read(&utc_sec) == OS_ERR_NONE) {
        g_site_info.utc_epoch_seconds = utc_sec;
    }
    g_tracking_enabled = true;
    g_system_state = OS_STATE_IDLE_TRACKING;
    g_park_state = false;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (park_pos.ra_hours < OS_RA_MIN_HOURS || park_pos.ra_hours > OS_RA_MAX_HOURS ||
        park_pos.dec_degrees < OS_DEC_MIN_DEG || park_pos.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_park_pos = park_pos;
    return os_hal_nvm_write(NVM_OFFSET_PARK, (const uint8_t*)&g_park_pos, sizeof(g_park_pos));
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST ||
        speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state != OS_STATE_IDLE_TRACKING && g_system_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }
    uint8_t axis = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) ? 0 : 1;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    bool forward = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_EAST);
    os_hal_motor_set_direction(axis, forward);
    uint32_t freq = 500;
    if (speed == OS_SPEED_MEDIUM) freq = 1000;
    else if (speed == OS_SPEED_FAST) freq = 3600;
    else if (speed == OS_SPEED_CUSTOM) freq = (uint32_t)g_custom_slew_speed;
    os_hal_motor_set_frequency(axis, freq);
    g_system_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    os_hal_motor_set_frequency(0, 0);
    os_hal_motor_set_frequency(1, 0);
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (arcsec_per_sec <= 0.0f || arcsec_per_sec > (OS_GOTO_SPEED_MAX_DEG_PER_SEC * 3600.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_custom_slew_speed = arcsec_per_sec;
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
    steps_to_coords(g_motor_pos[0], g_motor_pos[1], &coord->ra_hours, &coord->dec_degrees);
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
    if (enable && !g_pec_table.valid) {
        return OS_ERR_INVALID_STATE;
    }
    g_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL || !table->valid) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_pec_table = *table;
    return os_hal_nvm_write(NVM_OFFSET_PEC, (const uint8_t*)&g_pec_table, sizeof(g_pec_table));
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
    return os_hal_nvm_write(NVM_OFFSET_CALIBRATION, (const uint8_t*)&g_calibration, sizeof(g_calibration));
}

os_error_t os_hal_motor_init(uint8_t axis) {
    if (axis >= 2) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_motor_enabled[axis] = false;
    g_motor_freq[axis] = 0;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (axis >= 2) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_motor_freq[axis] = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (axis >= 2) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_motor_dir[axis] = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (axis >= 2) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_motor_enabled[axis] = enable;
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (axis >= 2) {
        return 0;
    }
    return g_motor_pos[axis];
}

os_error_t os_hal_gps_init(void) {
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = g_site_info;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void) {
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *utc_epoch_seconds = g_site_info.utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    if (utc_epoch_seconds == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_site_info.utc_epoch_seconds = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (axis >= 2) {
        return true;
    }
    return false;
}

os_error_t os_hal_nvm_init(void) {
    if (!g_hal_nvm_inited) {
        memset(g_hal_nvm_sim, 0, sizeof(g_hal_nvm_sim));
        g_hal_nvm_inited = true;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (data == NULL || (uint32_t)offset + length > sizeof(g_hal_nvm_sim)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(data, &g_hal_nvm_sim[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (data == NULL || (uint32_t)offset + length > sizeof(g_hal_nvm_sim)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(&g_hal_nvm_sim[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET) {
        return 0;
    }
    return 0;
}

char os_hal_comm_read(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET) {
        return 0;
    }
    return 0;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (data == NULL || channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    (void)length;
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
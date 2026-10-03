#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OS_STEPS_PER_RA_HOUR        54000
#define OS_STEPS_PER_DEC_DEGREE      3600
#define OS_STEPS_PER_ARCSEC_RA         1.0
#define OS_STEPS_PER_ARCSEC_DEC        1.0
#define OS_LOOP_TICK_MS                10
#define OS_GOTO_ACCEL_STEPS_PER_LOOP2   8.0
#define OS_MOVE_SLOW_HZ                20u
#define OS_MOVE_MEDIUM_HZ             100u
#define OS_MOVE_FAST_HZ               500u
#define OS_CALIBRATION_RESIDUAL_LIMIT_ARCSEC 300.0
#define OS_NVM_CALIBRATION_OFFSET         0
#define OS_NVM_CONFIG_OFFSET             OS_NVM_CALIBRATION_SIZE_BYTES
#define OS_NVM_PEC_OFFSET                (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)
#define OS_NVM_CALIBRATION_MAGIC          0x4F534E53u
#define OS_NVM_CONFIG_MAGIC               0x4F534E43u
#define OS_NVM_PEC_MAGIC                  0x50454331u
#define OS_PI 3.14159265358979323846

typedef struct {
    char buffer[OS_MAX_COMMAND_LENGTH + 1];
    uint8_t length;
    bool in_cmd;
} comm_rx_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    int32_t ra_steps;
    int32_t dec_steps;
    float park_ra_hours;
    float park_dec_degrees;
    bool park_position_valid;
    float fallback_lat;
    float fallback_lon;
    float fallback_elevation;
    uint32_t fallback_utc;
    bool fallback_valid;
} os_nvm_config_t;

static os_state_t s_state = OS_STATE_INITIALIZING;
static bool s_initialized = false;

static bool s_tracking_enabled = true;
static os_track_rate_t s_track_rate = OS_TRACK_RATE_SIDEREAL;
static float s_custom_track_factor = 1.0f;

static float s_guide_rate_fraction = 0.5f;
static os_guide_pulse_t s_guide_pulse;

static bool s_move_active = false;
static os_direction_t s_move_dir = OS_DIRECTION_NORTH;
static os_speed_level_t s_move_speed = OS_SPEED_SLOW;
static float s_custom_move_speed = 15.0f;

static bool s_goto_active = false;
static bool s_park_active = false;
static bool s_parked = false;
static int32_t s_goto_target_ra_steps = 0;
static int32_t s_goto_target_dec_steps = 0;
static int32_t s_park_ra_steps = 0;
static int32_t s_park_dec_steps = 0;
static double s_goto_ra_velocity = 0.0;
static double s_goto_dec_velocity = 0.0;

static bool s_park_position_valid = false;
static os_equatorial_coord_t s_park_position;

static int32_t s_position_ra_steps = 0;
static int32_t s_position_dec_steps = 0;
static double s_axis_fraction[2] = {0.0, 0.0};

static os_calibration_t s_calib;
static bool s_align_residual_computed = false;
static float s_align_residual_arcsec = 0.0f;
static uint8_t s_align_star_count = 0;
static os_align_mode_t s_align_mode = OS_ALIGN_1STAR;
static double s_align_ra_arcsec[OS_CALIBRATION_MAX_STARS];
static double s_align_dec_arcsec[OS_CALIBRATION_MAX_STARS];
static int32_t s_align_motor_ra[OS_CALIBRATION_MAX_STARS];
static int32_t s_align_motor_dec[OS_CALIBRATION_MAX_STARS];

static bool s_pec_enabled = false;
static os_pec_table_t s_pec_table;
static double s_worm_phase_deg = 0.0;

static os_site_info_t s_site;
static bool s_fallback_site_valid = false;
static os_site_info_t s_fallback_site;
static uint32_t s_last_rtc_epoch = 0;
static bool s_rtc_valid = false;

static bool s_config_loaded_from_nvm = false;

static bool s_lx200_ra_valid = false;
static bool s_lx200_dec_valid = false;
static float s_lx200_target_ra_hours = 0.0f;
static float s_lx200_target_dec_degrees = 0.0f;

static comm_rx_t s_comm_rx[4];

static void stop_all_motors(void) {
    (void)os_hal_motor_set_frequency(0, 0);
    (void)os_hal_motor_set_frequency(1, 0);
    (void)os_hal_motor_enable(0, false);
    (void)os_hal_motor_enable(1, false);
}

static int32_t axis_position(uint8_t axis) {
    return axis == 0 ? s_position_ra_steps : s_position_dec_steps;
}

static void account_steps(uint8_t axis, double steps) {
    if (axis > 1) {
        return;
    }
    double total = s_axis_fraction[axis] + steps;
    double whole = trunc(total);
    s_axis_fraction[axis] = total - whole;
    int32_t iwhole = (int32_t)whole;
    if (axis == 0) {
        s_position_ra_steps += iwhole;
    } else {
        s_position_dec_steps += iwhole;
    }
}

static double rate_factor_for_current_tracking(void) {
    switch (s_track_rate) {
        case OS_TRACK_RATE_LUNAR:
            return OS_LUNAR_RATE_FACTOR;
        case OS_TRACK_RATE_SOLAR:
            return OS_SOLAR_RATE_FACTOR;
        case OS_TRACK_RATE_CUSTOM:
            return (double)s_custom_track_factor;
        case OS_TRACK_RATE_SIDEREAL:
        default:
            return 1.0;
    }
}

static double tracking_frequency_hz_double(void) {
    double rate = OS_SIDEREAL_RATE_ARCSEC_PER_SEC * rate_factor_for_current_tracking();

    if (s_pec_enabled && s_pec_table.valid) {
        uint16_t index = (uint16_t)s_worm_phase_deg % OS_PEC_TABLE_SIZE;
        rate += (double)s_pec_table.corrections[index] * 0.001;
    }

    return rate * OS_STEPS_PER_ARCSEC_RA;
}

static uint32_t tracking_frequency_hz(void) {
    double freq = tracking_frequency_hz_double();
    if (freq < 0.0) {
        freq = 0.0;
    }
    return (uint32_t)(freq + 0.5);
}

static double manual_frequency_hz_double(void) {
    switch (s_move_speed) {
        case OS_SPEED_SLOW:
            return (double)OS_MOVE_SLOW_HZ;
        case OS_SPEED_MEDIUM:
            return (double)OS_MOVE_MEDIUM_HZ;
        case OS_SPEED_FAST:
            return (double)OS_MOVE_FAST_HZ;
        case OS_SPEED_CUSTOM:
            return (double)s_custom_move_speed * OS_STEPS_PER_ARCSEC_RA;
        default:
            return (double)OS_MOVE_MEDIUM_HZ;
    }
}

static uint32_t manual_frequency_hz(void) {
    double freq = manual_frequency_hz_double();
    if (freq < 0.0) {
        freq = 0.0;
    }
    return (uint32_t)(freq + 0.5);
}

static double goto_max_steps_per_loop(uint8_t axis) {
    double steps_per_deg = (axis == 0)
        ? ((double)OS_STEPS_PER_RA_HOUR / 15.0)
        : (double)OS_STEPS_PER_DEC_DEGREE;
    double max_steps_per_sec = steps_per_deg * (double)OS_GOTO_SPEED_MAX_DEG_PER_SEC;
    return max_steps_per_sec * (double)OS_LOOP_TICK_MS / 1000.0;
}

static void advance_axis_towards(uint8_t axis, int32_t target, double *velocity, double max_step) {
    int32_t current = axis_position(axis);
    int32_t delta = target - current;
    if (delta == 0) {
        (void)os_hal_motor_set_frequency(axis, 0);
        *velocity = 0.0;
        return;
    }

    bool forward = delta > 0;
    int32_t distance = abs(delta);
    double accel = OS_GOTO_ACCEL_STEPS_PER_LOOP2;

    if ((double)distance <= ((*velocity) * (*velocity)) / (2.0 * accel)) {
        *velocity -= accel;
        if (*velocity < 1.0) {
            *velocity = 1.0;
        }
    } else {
        *velocity += accel;
        if (*velocity > max_step) {
            *velocity = max_step;
        }
    }

    int32_t step = (int32_t)(*velocity);
    if (step > distance) {
        step = distance;
    }
    if (step < 1) {
        step = 1;
    }
    if (!forward) {
        step = -step;
    }

    if (axis == 0) {
        s_position_ra_steps += step;
    } else {
        s_position_dec_steps += step;
    }

    (void)os_hal_motor_set_direction(axis, forward);
    uint32_t freq = (uint32_t)((*velocity) * (1000.0 / (double)OS_LOOP_TICK_MS));
    (void)os_hal_motor_set_frequency(axis, freq);
    (void)os_hal_motor_enable(axis, true);
}

static void advance_goto_or_park(void) {
    int32_t target_ra = s_goto_active ? s_goto_target_ra_steps : s_park_ra_steps;
    int32_t target_dec = s_goto_active ? s_goto_target_dec_steps : s_park_dec_steps;

    advance_axis_towards(0, target_ra, &s_goto_ra_velocity, goto_max_steps_per_loop(0));
    advance_axis_towards(1, target_dec, &s_goto_dec_velocity, goto_max_steps_per_loop(1));

    if (s_position_ra_steps == target_ra && s_position_dec_steps == target_dec) {
        (void)os_hal_motor_set_frequency(0, 0);
        (void)os_hal_motor_set_frequency(1, 0);

        if (s_goto_active) {
            s_goto_active = false;
            s_state = OS_STATE_IDLE_TRACKING;
            (void)os_hal_buzzer_beep(200, 1);
        } else if (s_park_active) {
            s_park_active = false;
            s_parked = true;
            s_tracking_enabled = false;
            s_state = OS_STATE_PARKED;
            stop_all_motors();
        }
    }
}

static void advance_manual_move(void) {
    uint8_t axis = 0;
    bool forward = false;

    switch (s_move_dir) {
        case OS_DIRECTION_NORTH: axis = 1; forward = true;  break;
        case OS_DIRECTION_SOUTH: axis = 1; forward = false; break;
        case OS_DIRECTION_EAST:  axis = 0; forward = true;  break;
        case OS_DIRECTION_WEST:  axis = 0; forward = false; break;
        default: return;
    }

    double freq = manual_frequency_hz_double();
    uint32_t motor_freq = (uint32_t)(freq + 0.5);
    if (motor_freq == 0) {
        (void)os_hal_motor_set_frequency(axis, 0);
        return;
    }

    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, motor_freq);
    (void)os_hal_motor_enable(axis, true);

    double signed_steps = forward ? freq : -freq;
    signed_steps *= (double)OS_LOOP_TICK_MS / 1000.0;
    account_steps(axis, signed_steps);

    uint8_t other = axis == 0 ? 1 : 0;
    (void)os_hal_motor_set_frequency(other, 0);
}

static void advance_guide_pulse(uint32_t interval_ms) {
    uint8_t axis = 0;
    bool requested_forward = false;

    if (s_guide_pulse.dec_priority) {
        axis = 1;
        requested_forward = s_guide_pulse.direction_north;
    } else {
        axis = 0;
        requested_forward = s_guide_pulse.direction_east;
    }

    double base_freq = (axis == 0 && s_tracking_enabled) ? tracking_frequency_hz_double() : 0.0;
    double bias = base_freq * (double)s_guide_pulse.rate_fraction;
    double net = base_freq + (requested_forward ? bias : -bias);

    bool motor_forward = net >= 0.0;
    double motor_freq = fabs(net);
    uint32_t freq = (uint32_t)(motor_freq + 0.5);

    (void)os_hal_motor_set_direction(axis, motor_forward);
    (void)os_hal_motor_set_frequency(axis, freq);
    (void)os_hal_motor_enable(axis, true);

    double signed_steps = net * ((double)interval_ms / 1000.0);
    account_steps(axis, signed_steps);

    if (axis == 0) {
        (void)os_hal_motor_set_frequency(1, 0);
    } else {
        if (s_tracking_enabled) {
            double ra_freq = tracking_frequency_hz_double();
            (void)os_hal_motor_set_direction(0, true);
            (void)os_hal_motor_set_frequency(0, (uint32_t)(ra_freq + 0.5));
            (void)os_hal_motor_enable(0, true);
            account_steps(0, ra_freq * ((double)interval_ms / 1000.0));
        } else {
            (void)os_hal_motor_set_frequency(0, 0);
        }
    }
}

static void load_calibration_from_nvm(void) {
    uint8_t buf[4 + sizeof(os_calibration_t)];
    if (os_hal_nvm_read(OS_NVM_CALIBRATION_OFFSET, buf, sizeof(buf)) == OS_ERR_NONE) {
        uint32_t magic = 0;
        memcpy(&magic, buf, 4);
        if (magic == OS_NVM_CALIBRATION_MAGIC) {
            os_calibration_t calib;
            memcpy(&calib, buf + 4, sizeof(calib));
            if (calib.valid) {
                s_calib = calib;
            }
        }
    }
}

static os_error_t save_calibration_blob(const os_calibration_t *calib) {
    uint8_t buf[4 + sizeof(os_calibration_t)];
    uint32_t magic = OS_NVM_CALIBRATION_MAGIC;
    memcpy(buf, &magic, 4);
    memcpy(buf + 4, calib, sizeof(*calib));
    if (os_hal_nvm_write(OS_NVM_CALIBRATION_OFFSET, buf, sizeof(buf)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    return OS_ERR_NONE;
}

static void load_config_from_nvm(void) {
    os_nvm_config_t cfg;
    if (os_hal_nvm_read(OS_NVM_CONFIG_OFFSET, (uint8_t *)&cfg, sizeof(cfg)) != OS_ERR_NONE) {
        return;
    }
    if (cfg.magic != OS_NVM_CONFIG_MAGIC) {
        return;
    }

    s_position_ra_steps = cfg.ra_steps;
    s_position_dec_steps = cfg.dec_steps;
    s_park_position.ra_hours = cfg.park_ra_hours;
    s_park_position.dec_degrees = cfg.park_dec_degrees;
    s_park_position_valid = cfg.park_position_valid;

    if (cfg.fallback_valid) {
        s_fallback_site.latitude_degrees = cfg.fallback_lat;
        s_fallback_site.longitude_degrees = cfg.fallback_lon;
        s_fallback_site.elevation_metres = cfg.fallback_elevation;
        s_fallback_site.utc_epoch_seconds = cfg.fallback_utc;
        s_fallback_site.valid = false;
        s_fallback_site_valid = true;
    }

    s_config_loaded_from_nvm = true;
}

static os_error_t save_config_to_nvm(void) {
    os_nvm_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    cfg.magic = OS_NVM_CONFIG_MAGIC;
    cfg.version = 1u;
    cfg.ra_steps = s_position_ra_steps;
    cfg.dec_steps = s_position_dec_steps;
    cfg.park_ra_hours = s_park_position.ra_hours;
    cfg.park_dec_degrees = s_park_position.dec_degrees;
    cfg.park_position_valid = s_park_position_valid;

    if (s_fallback_site_valid) {
        cfg.fallback_lat = s_fallback_site.latitude_degrees;
        cfg.fallback_lon = s_fallback_site.longitude_degrees;
        cfg.fallback_elevation = s_fallback_site.elevation_metres;
        cfg.fallback_utc = s_fallback_site.utc_epoch_seconds;
        cfg.fallback_valid = true;
    }

    if (os_hal_nvm_write(OS_NVM_CONFIG_OFFSET, (const uint8_t *)&cfg, sizeof(cfg)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    return OS_ERR_NONE;
}

static void load_pec_from_nvm(void) {
    uint8_t buf[4 + sizeof(os_pec_table_t)];
    if (os_hal_nvm_read(OS_NVM_PEC_OFFSET, buf, sizeof(buf)) == OS_ERR_NONE) {
        uint32_t magic = 0;
        memcpy(&magic, buf, 4);
        if (magic == OS_NVM_PEC_MAGIC) {
            os_pec_table_t table;
            memcpy(&table, buf + 4, sizeof(table));
            if (table.valid) {
                s_pec_table = table;
            }
        }
    }
}

static os_error_t save_pec_blob(const os_pec_table_t *table) {
    uint8_t buf[4 + sizeof(os_pec_table_t)];
    uint32_t magic = OS_NVM_PEC_MAGIC;
    memcpy(buf, &magic, 4);
    memcpy(buf + 4, table, sizeof(*table));
    if (os_hal_nvm_write(OS_NVM_PEC_OFFSET, buf, sizeof(buf)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    return OS_ERR_NONE;
}

static bool qr_solve(double a[][6], double b[], int m, int n, double x[6]) {
    double v[18];
    for (int k = 0; k < n; ++k) {
        double norm = 0.0;
        for (int i = k; i < m; ++i) {
            norm += a[i][k] * a[i][k];
        }
        norm = sqrt(norm);
        if (norm < 1e-12) {
            return false;
        }

        double beta = (a[k][k] > 0.0) ? -norm : norm;
        for (int i = 0; i < k; ++i) {
            v[i] = 0.0;
        }
        v[k] = a[k][k] - beta;
        for (int i = k + 1; i < m; ++i) {
            v[i] = a[i][k];
        }

        double vnorm = 0.0;
        for (int i = k; i < m; ++i) {
            vnorm += v[i] * v[i];
        }
        if (vnorm < 1e-24) {
            return false;
        }

        for (int j = k; j < n; ++j) {
            double dot = 0.0;
            for (int i = k; i < m; ++i) {
                dot += v[i] * a[i][j];
            }
            dot = 2.0 * dot / vnorm;
            for (int i = k; i < m; ++i) {
                a[i][j] -= dot * v[i];
            }
        }

        double dotb = 0.0;
        for (int i = k; i < m; ++i) {
            dotb += v[i] * b[i];
        }
        dotb = 2.0 * dotb / vnorm;
        for (int i = k; i < m; ++i) {
            b[i] -= dotb * v[i];
        }
    }

    for (int i = n - 1; i >= 0; --i) {
        if (fabs(a[i][i]) < 1e-12) {
            return false;
        }
        double sum = b[i];
        for (int j = i + 1; j < n; ++j) {
            sum -= a[i][j] * x[j];
        }
        x[i] = sum / a[i][i];
    }
    return true;
}

static os_error_t compute_calibration(uint8_t mode, uint8_t count,
                                      const double ra_arcsec[],
                                      const double dec_arcsec[],
                                      const int32_t ra_steps[],
                                      const int32_t dec_steps[],
                                      os_calibration_t *out,
                                      float *residual_out) {
    if (count < 1 || count > OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }
    memset(out, 0, sizeof(*out));
    *residual_out = 0.0f;

    double desired_ra_corr[OS_CALIBRATION_MAX_STARS];
    double desired_dec_corr[OS_CALIBRATION_MAX_STARS];
    for (uint8_t i = 0; i < count; ++i) {
        desired_ra_corr[i] = (double)ra_steps[i] / OS_STEPS_PER_ARCSEC_RA;
        desired_dec_corr[i] = (double)dec_steps[i] / OS_STEPS_PER_ARCSEC_DEC;
    }

    if (mode == OS_ALIGN_1STAR) {
        out->matrix_ra_to_ra = 1.0f;
        out->matrix_dec_to_dec = 1.0f;
        out->offset_ra_arcsec = (float)(desired_ra_corr[0] - ra_arcsec[0]);
        out->offset_dec_arcsec = (float)(desired_dec_corr[0] - dec_arcsec[0]);
        out->valid = true;
        *residual_out = 0.0f;
        return OS_ERR_NONE;
    }

    if (mode == OS_ALIGN_2STAR) {
        if (count < 2) {
            return OS_ERR_INVALID_STATE;
        }
        double x1 = ra_arcsec[0];
        double x2 = ra_arcsec[1];
        if (fabs(x2 - x1) < 1e-12) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        double m00 = (desired_ra_corr[1] - desired_ra_corr[0]) / (x2 - x1);
        double b0 = desired_ra_corr[0] - m00 * x1;

        double y1 = dec_arcsec[0];
        double y2 = dec_arcsec[1];
        if (fabs(y2 - y1) < 1e-12) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        double m11 = (desired_dec_corr[1] - desired_dec_corr[0]) / (y2 - y1);
        double b1 = desired_dec_corr[0] - m11 * y1;

        out->matrix_ra_to_ra = (float)m00;
        out->matrix_dec_to_dec = (float)m11;
        out->offset_ra_arcsec = (float)b0;
        out->offset_dec_arcsec = (float)b1;
        out->valid = true;

        double se = 0.0;
        for (uint8_t i = 0; i < count; ++i) {
            double pred_ra = m00 * ra_arcsec[i] + b0;
            double pred_dec = m11 * dec_arcsec[i] + b1;
            double er = pred_ra - desired_ra_corr[i];
            double ed = pred_dec - desired_dec_corr[i];
            se += er * er + ed * ed;
        }
        *residual_out = (float)sqrt(se / count);
        return OS_ERR_NONE;
    }

    if (mode == OS_ALIGN_3STAR) {
        if (count < 3) {
            return OS_ERR_INVALID_STATE;
        }
        double x1 = ra_arcsec[0], y1 = dec_arcsec[0];
        double x2 = ra_arcsec[1], y2 = dec_arcsec[1];
        double x3 = ra_arcsec[2], y3 = dec_arcsec[2];
        double det = x1 * (y2 - y3) - y1 * (x2 - x3) + (x2 * y3 - y2 * x3);
        if (fabs(det) < 1e-6) {
            return OS_ERR_CALIBRATION_FAILED;
        }
    }

    if (mode == OS_ALIGN_NSTAR && count < 3) {
        return OS_ERR_INVALID_STATE;
    }

    double A[18][6] = {{0.0}};
    double y[18] = {0.0};
    int m = 2 * (int)count;

    for (uint8_t i = 0; i < count; ++i) {
        int row = 2 * i;
        A[row][0] = ra_arcsec[i];
        A[row][1] = dec_arcsec[i];
        A[row][4] = 1.0;
        y[row] = desired_ra_corr[i];

        A[row + 1][2] = ra_arcsec[i];
        A[row + 1][3] = dec_arcsec[i];
        A[row + 1][5] = 1.0;
        y[row + 1] = desired_dec_corr[i];
    }

    double p[6] = {0.0};
    if (!qr_solve(A, y, m, 6, p)) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    out->matrix_ra_to_ra  = (float)p[0];
    out->matrix_ra_to_dec = (float)p[1];
    out->matrix_dec_to_ra = (float)p[2];
    out->matrix_dec_to_dec = (float)p[3];
    out->offset_ra_arcsec  = (float)p[4];
    out->offset_dec_arcsec = (float)p[5];
    out->valid = true;

    double se = 0.0;
    for (uint8_t i = 0; i < count; ++i) {
        double pred_ra = p[0] * ra_arcsec[i] + p[1] * dec_arcsec[i] + p[4];
        double pred_dec = p[2] * ra_arcsec[i] + p[3] * dec_arcsec[i] + p[5];
        double er = pred_ra - desired_ra_corr[i];
        double ed = pred_dec - desired_dec_corr[i];
        se += er * er + ed * ed;
    }
    double residual = sqrt(se / count);
    *residual_out = (float)residual;

    if (count >= 4 && residual > OS_CALIBRATION_RESIDUAL_LIMIT_ARCSEC) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    return OS_ERR_NONE;
}

static void apply_calibration_to_steps(float ra_hours, float dec_degrees,
                                       int32_t *ra_steps, int32_t *dec_steps) {
    double raw_ra_arcsec = (double)ra_hours * 54000.0;
    double raw_dec_arcsec = (double)dec_degrees * 3600.0;

    if (s_calib.valid) {
        double corr_ra = (double)s_calib.matrix_ra_to_ra * raw_ra_arcsec +
                         (double)s_calib.matrix_ra_to_dec * raw_dec_arcsec +
                         (double)s_calib.offset_ra_arcsec;
        double corr_dec = (double)s_calib.matrix_dec_to_ra * raw_ra_arcsec +
                          (double)s_calib.matrix_dec_to_dec * raw_dec_arcsec +
                          (double)s_calib.offset_dec_arcsec;
        *ra_steps = (int32_t)lround(corr_ra * OS_STEPS_PER_ARCSEC_RA);
        *dec_steps = (int32_t)lround(corr_dec * OS_STEPS_PER_ARCSEC_DEC);
        return;
    }

    *ra_steps = (int32_t)lround((double)ra_hours * OS_STEPS_PER_RA_HOUR);
    *dec_steps = (int32_t)lround((double)dec_degrees * OS_STEPS_PER_DEC_DEGREE);
}

static float clamp_ra_hours(float ra_hours) {
    float ra = fmodf(ra_hours, 24.0f);
    if (ra < 0.0f) {
        ra += 24.0f;
    }
    return ra;
}

static float clamp_dec_degrees(float dec_degrees) {
    if (dec_degrees > OS_DEC_MAX_DEG) {
        return OS_DEC_MAX_DEG;
    }
    if (dec_degrees < OS_DEC_MIN_DEG) {
        return OS_DEC_MIN_DEG;
    }
    return dec_degrees;
}

static float gmst_degrees_from_utc(uint32_t utc_epoch) {
    double jd = 2440587.5 + ((double)utc_epoch / 86400.0);
    double T = (jd - 2451545.0) / 36525.0;
    double gmst = 280.46061837 +
                  360.98564736629 * (jd - 2451545.0) +
                  0.000387933 * T * T -
                  T * T * T / 38710000.0;
    gmst = fmod(gmst, 360.0);
    if (gmst < 0.0) {
        gmst += 360.0;
    }
    return (float)gmst;
}

static os_equatorial_coord_t horizontal_to_equatorial(os_horizontal_coord_t target) {
    os_equatorial_coord_t eq;
    float latitude = s_site.valid ? s_site.latitude_degrees :
                     (s_fallback_site_valid ? s_fallback_site.latitude_degrees : 0.0f);
    float longitude = s_site.valid ? s_site.longitude_degrees :
                      (s_fallback_site_valid ? s_fallback_site.longitude_degrees : 0.0f);
    uint32_t utc = s_rtc_valid ? s_last_rtc_epoch :
                   (s_site.valid ? s_site.utc_epoch_seconds :
                   (s_fallback_site_valid ? s_fallback_site.utc_epoch_seconds : 0u));

    double alt = (double)target.altitude_degrees * OS_PI / 180.0;
    double az = (double)target.azimuth_degrees * OS_PI / 180.0;
    double lat = (double)latitude * OS_PI / 180.0;
    double lon = (double)longitude;

    double sin_dec = sin(lat) * sin(alt) + cos(lat) * cos(alt) * cos(az);
    if (sin_dec > 1.0) sin_dec = 1.0;
    if (sin_dec < -1.0) sin_dec = -1.0;
    double dec = asin(sin_dec);

    double cos_dec = cos(dec);
    double sin_ha = 0.0;
    double cos_ha = 1.0;
    if (fabs(cos_dec) > 1e-9 && fabs(cos(lat)) > 1e-9) {
        sin_ha = -cos(alt) * sin(az) / cos_dec;
        cos_ha = (sin(alt) - sin(lat) * sin_dec) / (cos(lat) * cos_dec);
    }
    double ha = atan2(sin_ha, cos_ha);

    double lst_deg = (double)gmst_degrees_from_utc(utc) + lon;
    double ra_deg = lst_deg - (ha * 180.0 / OS_PI);
    while (ra_deg < 0.0) ra_deg += 360.0;
    while (ra_deg >= 360.0) ra_deg -= 360.0;

    eq.ra_hours = clamp_ra_hours((float)(ra_deg / 15.0));
    eq.dec_degrees = clamp_dec_degrees((float)(dec * 180.0 / OS_PI));
    return eq;
}

static size_t format_ra_reply(float ra_hours, char *buf, size_t size) {
    if (size == 0) {
        return 0;
    }
    ra_hours = clamp_ra_hours(ra_hours);

    int h = (int)ra_hours;
    float mf = (ra_hours - (float)h) * 60.0f;
    int m = (int)mf;
    float sf = (mf - (float)m) * 60.0f;
    int s = (int)(sf + 0.5f);
    if (s >= 60) { s -= 60; m++; }
    if (m >= 60) { m -= 60; h++; }
    if (h >= 24) { h -= 24; }

    int len = snprintf(buf, size, "%02d:%02d:%02d#", h, m, s);
    if (len < 0) {
        return 0;
    }
    if ((size_t)len >= size) {
        len = (int)(size - 1);
    }
    return (size_t)len;
}

static size_t format_dec_reply(float dec_deg, char *buf, size_t size) {
    if (size == 0) {
        return 0;
    }
    char sign = (dec_deg < 0.0f) ? '-' : '+';
    float ad = fabsf(dec_deg);
    if (ad > 90.0f) {
        ad = 90.0f;
    }

    int d = (int)ad;
    float mf = (ad - (float)d) * 60.0f;
    int m = (int)mf;
    float sf = (mf - (float)m) * 60.0f;
    int s = (int)(sf + 0.5f);
    if (s >= 60) { s -= 60; m++; }
    if (m >= 60) { m -= 60; d++; }
    if (d > 90) { d = 90; }

    int len = snprintf(buf, size, "%c%02d:%02d:%02d#", sign, d, m, s);
    if (len < 0) {
        return 0;
    }
    if ((size_t)len >= size) {
        len = (int)(size - 1);
    }
    return (size_t)len;
}

static bool parse_ra_lx200(const char *s, float *hours) {
    if (!s || !hours) {
        return false;
    }
    int h = 0, m = 0;
    float sec = 0.0f;
    if (sscanf(s, "%d:%d:%f", &h, &m, &sec) < 3) {
        return false;
    }
    if (h < 0 || h > 24 || m < 0 || m >= 60 || sec < 0.0f || sec >= 60.0f) {
        return false;
    }
    *hours = (float)h + ((float)m / 60.0f) + (sec / 3600.0f);
    return true;
}

static bool parse_dec_lx200(const char *s, float *degrees) {
    if (!s || !degrees) {
        return false;
    }
    char sign = '\0';
    int d = 0, m = 0;
    float sec = 0.0f;
    char sep1 = '\0', sep2 = '\0';
    if (sscanf(s, "%c%d%c%d%c%f", &sign, &d, &sep1, &m, &sep2, &sec) != 6) {
        return false;
    }
    if ((sign != '+' && sign != '-') || d < 0 || d > 90 || m < 0 || m >= 60 ||
        sec < 0.0f || sec >= 60.0f) {
        return false;
    }
    if ((sep1 != ':' && sep1 != '*') || sep2 != ':') {
        return false;
    }
    float value = (float)d + ((float)m / 60.0f) + (sec / 3600.0f);
    *degrees = (sign == '-') ? -value : value;
    return true;
}

static void set_reply_text(char *reply_buffer, size_t reply_buffer_size,
                           size_t *reply_length, const char *text) {
    if (!reply_buffer || reply_buffer_size == 0 || !reply_length) {
        return;
    }
    size_t len = strlen(text);
    if (len >= reply_buffer_size) {
        len = reply_buffer_size - 1;
    }
    memcpy(reply_buffer, text, len);
    reply_buffer[len] = '\0';
    *reply_length = len;
}

static void reset_runtime_state(void) {
    s_state = OS_STATE_INITIALIZING;
    s_initialized = false;

    s_tracking_enabled = true;
    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_custom_track_factor = 1.0f;
    s_guide_rate_fraction = 0.5f;
    memset(&s_guide_pulse, 0, sizeof(s_guide_pulse));

    s_move_active = false;
    s_move_dir = OS_DIRECTION_NORTH;
    s_move_speed = OS_SPEED_SLOW;
    s_custom_move_speed = 15.0f;

    s_goto_active = false;
    s_park_active = false;
    s_parked = false;
    s_goto_target_ra_steps = 0;
    s_goto_target_dec_steps = 0;
    s_park_ra_steps = 0;
    s_park_dec_steps = 0;
    s_goto_ra_velocity = 0.0;
    s_goto_dec_velocity = 0.0;

    s_park_position_valid = false;
    memset(&s_park_position, 0, sizeof(s_park_position));

    s_position_ra_steps = 0;
    s_position_dec_steps = 0;
    s_axis_fraction[0] = 0.0;
    s_axis_fraction[1] = 0.0;

    s_align_star_count = 0;
    s_align_residual_computed = false;
    s_align_residual_arcsec = 0.0f;
    s_align_mode = OS_ALIGN_1STAR;
    memset(s_align_ra_arcsec, 0, sizeof(s_align_ra_arcsec));
    memset(s_align_dec_arcsec, 0, sizeof(s_align_dec_arcsec));
    memset(s_align_motor_ra, 0, sizeof(s_align_motor_ra));
    memset(s_align_motor_dec, 0, sizeof(s_align_motor_dec));

    s_pec_enabled = false;
    memset(&s_pec_table, 0, sizeof(s_pec_table));
    s_worm_phase_deg = 0.0;

    memset(&s_calib, 0, sizeof(s_calib));
    memset(&s_site, 0, sizeof(s_site));
    s_site.valid = false;
    s_fallback_site_valid = false;
    memset(&s_fallback_site, 0, sizeof(s_fallback_site));
    s_last_rtc_epoch = 0;
    s_rtc_valid = false;

    s_config_loaded_from_nvm = false;

    s_lx200_ra_valid = false;
    s_lx200_dec_valid = false;
    s_lx200_target_ra_hours = 0.0f;
    s_lx200_target_dec_degrees = 0.0f;

    memset(s_comm_rx, 0, sizeof(s_comm_rx));
}

os_error_t os_init(void) {
    reset_runtime_state();

    (void)os_hal_nvm_init();
    load_calibration_from_nvm();
    load_config_from_nvm();
    load_pec_from_nvm();

    (void)os_hal_comm_init(OS_CHANNEL_USB);
    (void)os_hal_comm_init(OS_CHANNEL_BLUETOOTH);
    (void)os_hal_comm_init(OS_CHANNEL_WIFI);
    (void)os_hal_comm_init(OS_CHANNEL_ETHERNET);

    os_error_t motor_err0 = os_hal_motor_init(0);
    os_error_t motor_err1 = os_hal_motor_init(1);
    if (motor_err0 != OS_ERR_NONE || motor_err1 != OS_ERR_NONE) {
        (void)os_hal_motor_enable(0, false);
        (void)os_hal_motor_enable(1, false);
        (void)os_hal_motor_set_frequency(0, 0);
        (void)os_hal_motor_set_frequency(1, 0);
        s_initialized = true;
        s_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    (void)os_hal_motor_enable(0, false);
    (void)os_hal_motor_enable(1, false);
    (void)os_hal_motor_set_frequency(0, 0);
    (void)os_hal_motor_set_frequency(1, 0);

    if (!s_config_loaded_from_nvm) {
        s_position_ra_steps = os_hal_motor_get_position(0);
        s_position_dec_steps = os_hal_motor_get_position(1);
    }

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();

    os_error_t timer_err = os_hal_timer_motor_init();
    if (timer_err != OS_ERR_NONE) {
        (void)os_hal_motor_enable(0, false);
        (void)os_hal_motor_enable(1, false);
        (void)os_hal_motor_set_frequency(0, 0);
        (void)os_hal_motor_set_frequency(1, 0);
        s_initialized = true;
        s_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    (void)os_hal_gps_poll(&s_site);
    if (s_site.valid) {
        s_rtc_valid = true;
        s_last_rtc_epoch = s_site.utc_epoch_seconds;
        (void)os_hal_rtc_set(s_last_rtc_epoch);
    } else if (os_hal_rtc_read(&s_last_rtc_epoch) == OS_ERR_NONE) {
        s_rtc_valid = true;
    }

    if (!s_rtc_valid && s_fallback_site_valid && s_fallback_site.utc_epoch_seconds != 0u) {
        s_last_rtc_epoch = s_fallback_site.utc_epoch_seconds;
        s_rtc_valid = true;
    }

    s_initialized = true;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    if (!s_initialized) {
        return;
    }

    for (uint8_t ch = 0; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        int16_t avail = os_hal_comm_available(ch);
        if (avail <= 0) {
            continue;
        }

        while (avail-- > 0) {
            char c = os_hal_comm_read(ch);

            if (!s_comm_rx[ch].in_cmd) {
                if (c == OS_LX200_CMD_PREFIX) {
                    s_comm_rx[ch].in_cmd = true;
                    s_comm_rx[ch].buffer[0] = c;
                    s_comm_rx[ch].length = 1;
                    s_comm_rx[ch].buffer[1] = '\0';
                }
                continue;
            }

            if (c == OS_LX200_CMD_PREFIX) {
                s_comm_rx[ch].buffer[0] = c;
                s_comm_rx[ch].length = 1;
                s_comm_rx[ch].buffer[1] = '\0';
                continue;
            }

            if (s_comm_rx[ch].length >= OS_MAX_COMMAND_LENGTH) {
                s_comm_rx[ch].in_cmd = false;
                s_comm_rx[ch].length = 0;
                s_comm_rx[ch].buffer[0] = '\0';
                continue;
            }

            s_comm_rx[ch].buffer[s_comm_rx[ch].length++] = c;
            s_comm_rx[ch].buffer[s_comm_rx[ch].length] = '\0';

            if (c == OS_LX200_CMD_SUFFIX) {
                char reply[OS_MAX_REPLY_LENGTH];
                size_t reply_len = 0;
                os_error_t err = os_command_parse(s_comm_rx[ch].buffer,
                                                  s_comm_rx[ch].length,
                                                  ch,
                                                  reply,
                                                  sizeof(reply),
                                                  &reply_len);
                if (err == OS_ERR_NONE && reply_len > 0) {
                    (void)os_hal_comm_write(ch, reply, reply_len);
                }
                s_comm_rx[ch].in_cmd = false;
                s_comm_rx[ch].length = 0;
                s_comm_rx[ch].buffer[0] = '\0';
            }
        }
    }

    (void)os_hal_gps_poll(&s_site);
    if (s_site.valid) {
        s_rtc_valid = true;
        s_last_rtc_epoch = s_site.utc_epoch_seconds;
    } else if (os_hal_rtc_read(&s_last_rtc_epoch) == OS_ERR_NONE) {
        s_rtc_valid = true;
    }

    bool limit0 = os_hal_limit_is_triggered(0);
    bool limit1 = os_hal_limit_is_triggered(1);
    if (limit0 || limit1) {
        stop_all_motors();
        s_goto_active = false;
        s_park_active = false;
        s_move_active = false;
        s_guide_pulse.active = false;
        s_state = OS_STATE_FAULT;
        return;
    }

    if (s_state == OS_STATE_FAULT) {
        stop_all_motors();
        return;
    }

    if (s_guide_pulse.active) {
        uint32_t todo_ms = s_guide_pulse.duration_ms > OS_LOOP_TICK_MS
                               ? (uint32_t)OS_LOOP_TICK_MS
                               : s_guide_pulse.duration_ms;
        advance_guide_pulse(todo_ms);
        s_guide_pulse.duration_ms -= todo_ms;
        if (s_guide_pulse.duration_ms == 0) {
            s_guide_pulse.active = false;
        }
    } else if (s_goto_active || s_park_active) {
        advance_goto_or_park();
    } else if (s_move_active) {
        advance_manual_move();
    } else if (s_state == OS_STATE_IDLE_TRACKING && s_tracking_enabled) {
        double freq = tracking_frequency_hz_double();
        uint32_t motor_freq = (uint32_t)(freq + 0.5);
        (void)os_hal_motor_set_direction(0, true);
        (void)os_hal_motor_set_frequency(0, motor_freq);
        (void)os_hal_motor_enable(0, true);
        (void)os_hal_motor_set_frequency(1, 0);

        double ra_steps = freq * ((double)OS_LOOP_TICK_MS / 1000.0);
        account_steps(0, ra_steps);
        double full_ra_steps = (double)OS_STEPS_PER_RA_HOUR * 24.0;
        s_worm_phase_deg = fmod(s_worm_phase_deg + (ra_steps / full_ra_steps) * 360.0, 360.0);
        if (s_worm_phase_deg < 0.0) {
            s_worm_phase_deg += 360.0;
        }
    } else {
        (void)os_hal_motor_set_frequency(0, 0);
        (void)os_hal_motor_set_frequency(1, 0);
    }

    (void)save_config_to_nvm();
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
    if (length == 0 || length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_COMMAND_FORMAT;
    }
    if (reply_buffer_size == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (command[0] != OS_LX200_CMD_PREFIX || command[length - 1] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    size_t body_len = length - 2;
    if (body_len == 0 || body_len >= OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_COMMAND_FORMAT;
    }

    char body[OS_MAX_COMMAND_LENGTH];
    memcpy(body, command + 1, body_len);
    body[body_len] = '\0';

    *reply_length = 0;
    reply_buffer[0] = '\0';

    char c1 = body[0];
    char c2 = body_len > 1 ? body[1] : '\0';

    if (c1 == 'G' && c2 == 'R') {
        os_equatorial_coord_t coord;
        os_error_t err = os_query_coordinates(&coord);
        if (err != OS_ERR_NONE) {
            return err;
        }
        *reply_length = format_ra_reply(coord.ra_hours, reply_buffer, reply_buffer_size);
        return OS_ERR_NONE;
    }

    if (c1 == 'G' && c2 == 'D') {
        os_equatorial_coord_t coord;
        os_error_t err = os_query_coordinates(&coord);
        if (err != OS_ERR_NONE) {
            return err;
        }
        *reply_length = format_dec_reply(coord.dec_degrees, reply_buffer, reply_buffer_size);
        return OS_ERR_NONE;
    }

    if (c1 == 'G' && c2 == 'V' && body_len >= 3 && body[2] == 'P') {
        set_reply_text(reply_buffer, reply_buffer_size, reply_length, "1.0.0#");
        return OS_ERR_NONE;
    }

    if (c1 == 'Q') {
        (void)os_goto_abort();
        (void)os_move_stop();
        set_reply_text(reply_buffer, reply_buffer_size, reply_length, "1#");
        return OS_ERR_NONE;
    }

    if (c1 == 'S' && c2 == 'r') {
        float ra = 0.0f;
        if (!parse_ra_lx200(body + 2, &ra)) {
            return OS_ERR_COMMAND_FORMAT;
        }
        s_lx200_target_ra_hours = ra;
        s_lx200_ra_valid = true;
        set_reply_text(reply_buffer, reply_buffer_size, reply_length, "1#");
        return OS_ERR_NONE;
    }

    if (c1 == 'S' && c2 == 'd') {
        float dec = 0.0f;
        if (!parse_dec_lx200(body + 2, &dec)) {
            return OS_ERR_COMMAND_FORMAT;
        }
        s_lx200_target_dec_degrees = dec;
        s_lx200_dec_valid = true;
        set_reply_text(reply_buffer, reply_buffer_size, reply_length, "1#");
        return OS_ERR_NONE;
    }

    if (c1 == 'M' && c2 == 'S') {
        if (!s_lx200_ra_valid || !s_lx200_dec_valid) {
            return OS_ERR_INVALID_STATE;
        }
        os_equatorial_coord_t target;
        target.ra_hours = s_lx200_target_ra_hours;
        target.dec_degrees = s_lx200_target_dec_degrees;
        os_error_t err = os_goto_equatorial(target);
        if (err != OS_ERR_NONE) {
            return err;
        }
        set_reply_text(reply_buffer, reply_buffer_size, reply_length, "1#");
        return OS_ERR_NONE;
    }

    if (c1 == 'M') {
        os_direction_t dir;
        if (c2 == 'e') {
            dir = OS_DIRECTION_EAST;
        } else if (c2 == 'w') {
            dir = OS_DIRECTION_WEST;
        } else if (c2 == 'n') {
            dir = OS_DIRECTION_NORTH;
        } else if (c2 == 's') {
            dir = OS_DIRECTION_SOUTH;
        } else {
            return OS_ERR_COMMAND_FORMAT;
        }

        os_error_t err = os_move_start(dir, OS_SPEED_MEDIUM);
        if (err != OS_ERR_NONE) {
            return err;
        }
        set_reply_text(reply_buffer, reply_buffer_size, reply_length, "1#");
        return OS_ERR_NONE;
    }

    if (c1 == 'h') {
        if (c2 == 'P') {
            os_error_t err = os_park();
            if (err != OS_ERR_NONE) {
                return err;
            }
            set_reply_text(reply_buffer, reply_buffer_size, reply_length, "1#");
            return OS_ERR_NONE;
        }
        if (c2 == 'O') {
            os_error_t err = os_unpark();
            if (err != OS_ERR_NONE) {
                return err;
            }
            set_reply_text(reply_buffer, reply_buffer_size, reply_length, "1#");
            return OS_ERR_NONE;
        }
    }

    return OS_ERR_COMMAND_FORMAT;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    int32_t ra_steps = 0;
    int32_t dec_steps = 0;
    apply_calibration_to_steps(target.ra_hours, target.dec_degrees, &ra_steps, &dec_steps);

    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_goto_target_ra_steps = ra_steps;
    s_goto_target_dec_steps = dec_steps;
    s_goto_active = true;
    s_park_active = false;
    s_parked = false;
    s_goto_ra_velocity = 0.0;
    s_goto_dec_velocity = 0.0;
    s_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (target.altitude_degrees < OS_DEC_MIN_DEG || target.altitude_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    os_equatorial_coord_t eq = horizontal_to_equatorial(target);
    return os_goto_equatorial(eq);
}

os_error_t os_goto_abort(void) {
    stop_all_motors();
    s_goto_active = false;
    s_park_active = false;
    s_move_active = false;
    s_guide_pulse.active = false;
    s_goto_ra_velocity = 0.0;
    s_goto_dec_velocity = 0.0;
    if (s_state == OS_STATE_GOTO || s_state == OS_STATE_MANUAL_MOTION) {
        s_state = OS_STATE_IDLE_TRACKING;
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
    s_track_rate = rate;
    if (rate == OS_TRACK_RATE_CUSTOM) {
        s_custom_track_factor = custom_factor;
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (!rate || !custom_factor) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = s_track_rate;
    *custom_factor = s_custom_track_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    s_tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    s_tracking_enabled = false;
    (void)os_hal_motor_set_frequency(0, 0);
    (void)os_hal_motor_set_frequency(1, 0);
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    uint8_t axis = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH) ? 1 : 0;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    memset(&s_guide_pulse, 0, sizeof(s_guide_pulse));
    s_guide_pulse.active = true;
    s_guide_pulse.duration_ms = duration_ms;
    s_guide_pulse.rate_fraction = s_guide_rate_fraction;
    s_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    s_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    s_guide_pulse.dec_priority = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_guide_rate_fraction = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (!pulse) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *pulse = s_guide_pulse;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_align_mode = mode;
    s_align_star_count = 0;
    s_align_residual_computed = false;
    s_align_residual_arcsec = 0.0f;
    s_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (star_coord.ra_hours < OS_RA_MIN_HOURS || star_coord.ra_hours > OS_RA_MAX_HOURS) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (star_coord.dec_degrees < OS_DEC_MIN_DEG || star_coord.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_align_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    s_align_ra_arcsec[s_align_star_count] = (double)star_coord.ra_hours * 54000.0;
    s_align_dec_arcsec[s_align_star_count] = (double)star_coord.dec_degrees * 3600.0;
    s_align_motor_ra[s_align_star_count] = motor_pos.ra_steps;
    s_align_motor_dec[s_align_star_count] = motor_pos.dec_steps;
    s_align_star_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t min_stars = 1;
    if (s_align_mode == OS_ALIGN_2STAR) {
        min_stars = 2;
    } else if (s_align_mode == OS_ALIGN_3STAR || s_align_mode == OS_ALIGN_NSTAR) {
        min_stars = 3;
    }

    if (s_align_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    os_calibration_t local_calib;
    float residual = 0.0f;
    os_error_t err = compute_calibration(s_align_mode,
                                         s_align_star_count,
                                         s_align_ra_arcsec,
                                         s_align_dec_arcsec,
                                         s_align_motor_ra,
                                         s_align_motor_dec,
                                         &local_calib,
                                         &residual);
    if (err != OS_ERR_NONE) {
        return err;
    }

    err = save_calibration_blob(&local_calib);
    if (err != OS_ERR_NONE) {
        return err;
    }

    s_calib = local_calib;
    s_calib.valid = true;
    s_align_residual_computed = true;
    s_align_residual_arcsec = residual;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (!residual_arcsec) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_align_residual_computed) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = s_align_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    s_align_star_count = 0;
    s_align_residual_computed = false;
    s_align_residual_arcsec = 0.0f;
    if (s_state == OS_STATE_ALIGNMENT) {
        s_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (!s_park_position_valid) {
        s_park_position.ra_hours = 0.0f;
        s_park_position.dec_degrees = 90.0f;
    }

    int32_t ra_steps = 0;
    int32_t dec_steps = 0;
    apply_calibration_to_steps(s_park_position.ra_hours, s_park_position.dec_degrees,
                               &ra_steps, &dec_steps);

    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_park_ra_steps = ra_steps;
    s_park_dec_steps = dec_steps;
    s_park_active = true;
    s_goto_active = false;
    s_parked = false;
    s_goto_ra_velocity = 0.0;
    s_goto_dec_velocity = 0.0;
    s_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    (void)os_hal_comm_init(OS_CHANNEL_USB);
    (void)os_hal_comm_init(OS_CHANNEL_BLUETOOTH);
    (void)os_hal_comm_init(OS_CHANNEL_WIFI);
    (void)os_hal_comm_init(OS_CHANNEL_ETHERNET);

    (void)os_hal_gps_poll(&s_site);
    if (s_site.valid) {
        s_rtc_valid = true;
        s_last_rtc_epoch = s_site.utc_epoch_seconds;
        (void)os_hal_rtc_set(s_last_rtc_epoch);
    }

    (void)os_hal_motor_enable(0, true);
    (void)os_hal_motor_enable(1, true);
    s_parked = false;
    s_park_active = false;
    s_goto_active = false;
    s_tracking_enabled = true;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (park_pos.ra_hours < OS_RA_MIN_HOURS || park_pos.ra_hours > OS_RA_MAX_HOURS) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (park_pos.dec_degrees < OS_DEC_MIN_DEG || park_pos.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_park_position = park_pos;
    s_park_position_valid = true;
    return save_config_to_nvm();
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    uint8_t axis = 0;
    bool forward = false;
    switch (direction) {
        case OS_DIRECTION_NORTH: axis = 1; forward = true; break;
        case OS_DIRECTION_SOUTH: axis = 1; forward = false; break;
        case OS_DIRECTION_EAST:  axis = 0; forward = true; break;
        case OS_DIRECTION_WEST:  axis = 0; forward = false; break;
        default: return OS_ERR_INVALID_ARGUMENT;
    }

    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_move_dir = direction;
    s_move_speed = speed;
    s_move_active = true;
    s_goto_active = false;
    s_park_active = false;
    s_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    s_move_active = false;
    stop_all_motors();
    if (s_state == OS_STATE_MANUAL_MOTION) {
        s_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_custom_move_speed = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (!state) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = s_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (!coord) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    float ra_hours;
    float dec_degrees;

    if (s_calib.valid) {
        double corr_ra_arcsec = (double)s_position_ra_steps / OS_STEPS_PER_ARCSEC_RA;
        double corr_dec_arcsec = (double)s_position_dec_steps / OS_STEPS_PER_ARCSEC_DEC;

        double m00 = (double)s_calib.matrix_ra_to_ra;
        double m01 = (double)s_calib.matrix_ra_to_dec;
        double m10 = (double)s_calib.matrix_dec_to_ra;
        double m11 = (double)s_calib.matrix_dec_to_dec;
        double rhs_ra = corr_ra_arcsec - (double)s_calib.offset_ra_arcsec;
        double rhs_dec = corr_dec_arcsec - (double)s_calib.offset_dec_arcsec;
        double det = m00 * m11 - m01 * m10;

        if (fabs(det) > 1e-12) {
            double raw_ra = (m11 * rhs_ra - m01 * rhs_dec) / det;
            double raw_dec = (-m10 * rhs_ra + m00 * rhs_dec) / det;
            ra_hours = (float)(raw_ra / 54000.0);
            dec_degrees = (float)(raw_dec / 3600.0);
        } else {
            ra_hours = (float)(corr_ra_arcsec / 54000.0);
            dec_degrees = (float)(corr_dec_arcsec / 3600.0);
        }
    } else {
        ra_hours = (float)s_position_ra_steps / (float)OS_STEPS_PER_RA_HOUR;
        dec_degrees = (float)s_position_dec_steps / (float)OS_STEPS_PER_DEC_DEGREE;
    }

    coord->ra_hours = clamp_ra_hours(ra_hours);
    coord->dec_degrees = clamp_dec_degrees(dec_degrees);
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (!site) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = s_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (!pos) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = s_position_ra_steps;
    pos->dec_steps = s_position_dec_steps;
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
    *moving = s_goto_active || s_park_active || s_move_active || s_guide_pulse.active;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (!locked) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *locked = s_site.valid;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    s_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (!table) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    os_pec_table_t local = *table;
    local.valid = true;
    os_error_t err = save_pec_blob(&local);
    if (err != OS_ERR_NONE) {
        return err;
    }
    s_pec_table = local;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (!table) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *table = s_pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    uint16_t index = (worm_phase_deg >= 360.0f) ? 0 : (uint16_t)worm_phase_deg;
    if (index >= OS_PEC_TABLE_SIZE) {
        index = OS_PEC_TABLE_SIZE - 1;
    }

    os_pec_table_t local = s_pec_table;
    local.corrections[index] = error_arcsec;
    local.valid = true;
    os_error_t err = save_pec_blob(&local);
    if (err != OS_ERR_NONE) {
        return err;
    }
    s_pec_table = local;
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (!calib) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *calib = s_calib;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    os_calibration_t cleared;
    memset(&cleared, 0, sizeof(cleared));
    os_error_t err = save_calibration_blob(&cleared);
    if (err != OS_ERR_NONE) {
        return err;
    }

    s_calib = cleared;
    s_align_residual_computed = false;
    s_align_residual_arcsec = 0.0f;
    return OS_ERR_NONE;
}
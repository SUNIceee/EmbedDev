#include "6_generated_code.h"

#if defined(__has_include)
# if __has_include("Config.h")
#  include "Config.h"
#  define OS_CONFIG_H_PRESENT 1
# endif
#else
# include "Config.h"
# define OS_CONFIG_H_PRESENT 1
#endif

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef OS_STEPS_PER_ARCSEC
#define OS_STEPS_PER_ARCSEC 1.0
#endif

#ifndef OS_MOUNT_TYPE
#define OS_MOUNT_TYPE OS_MOUNT_EQUATORIAL
#endif

#ifndef OS_CHANNEL_USB_ENABLED
#define OS_CHANNEL_USB_ENABLED 1u
#endif

#ifndef OS_CHANNEL_BLUETOOTH_ENABLED
#define OS_CHANNEL_BLUETOOTH_ENABLED 0u
#endif

#ifndef OS_CHANNEL_WIFI_ENABLED
#define OS_CHANNEL_WIFI_ENABLED 0u
#endif

#ifndef OS_CHANNEL_ETHERNET_ENABLED
#define OS_CHANNEL_ETHERNET_ENABLED 0u
#endif

#ifndef OS_CONFIG_MANUAL_SLOW_ARCSEC_PER_SEC
#define OS_CONFIG_MANUAL_SLOW_ARCSEC_PER_SEC 100.0
#endif

#ifndef OS_CONFIG_MANUAL_MEDIUM_ARCSEC_PER_SEC
#define OS_CONFIG_MANUAL_MEDIUM_ARCSEC_PER_SEC 1000.0
#endif

#ifndef OS_CONFIG_MANUAL_FAST_ARCSEC_PER_SEC
#define OS_CONFIG_MANUAL_FAST_ARCSEC_PER_SEC 10000.0
#endif

#define AXIS_RA_AZ                0u
#define AXIS_DEC_ALT              1u
#define AXIS_COUNT                2u

#define DEFAULT_DEADBAND_STEPS    2
#define DEFAULT_SLOW_FREQ_HZ      100u
#define DEFAULT_FAST_FREQ_HZ      ((uint32_t)(OS_GOTO_SPEED_MAX_DEG_PER_SEC * 3600.0 * OS_STEPS_PER_ARCSEC))
#define GOTO_ACCEL_FREQ_HZ        500u
#define GOTO_DECEL_STEP_THRESHOLD 400

#define MAX_ALIGN_PARAMS           3
#define NVM_CAL_MAGIC              0x4F534341u
#define NVM_CONFIG_MAGIC           0x4F534346u
#define NVM_PEC_BASE               (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)

#define DEFAULT_PARK_RA_HOURS      0.0f
#define DEFAULT_PARK_DEC_DEG       90.0f
#define CALIBRATION_MAX_RESIDUAL_ARCSEC 300.0f
#define PEC_WORM_STEPS_PER_REV     1296000u
#define DEFAULT_GUIDE_PULSE_MS     1000u
#define OS_LOOP_MS                1u

typedef struct {
    double m00;
    double m01;
    double m10;
    double m11;
    double off_ra;
    double off_dec;
} calib_double_t;

typedef struct {
    double m00;
    double m01;
    double m10;
    double m11;
    double off_ra;
    double off_dec;
    uint32_t magic;
} nvm_cal_t;

typedef struct {
    uint32_t magic;
    float    park_ra_hours;
    float    park_dec_degrees;
} nvm_config_t;

static os_state_t s_state = OS_STATE_INITIALIZING;
static os_error_t s_last_error = OS_ERR_NONE;
static bool s_nvm_ok = false;
static bool s_gps_locked = false;
static os_site_info_t s_site;
static bool s_tracking_enabled = false;
static os_track_rate_t s_track_rate = OS_TRACK_RATE_SIDEREAL;
static float s_custom_track_factor = 1.0f;
static calib_double_t s_calib;
static bool s_calib_valid = false;
static float s_residual_arcsec = 0.0f;

static os_align_mode_t s_align_mode = OS_ALIGN_3STAR;
static uint8_t s_align_count = 0;
static os_equatorial_coord_t s_align_star_coord[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t s_align_motor_pos[OS_CALIBRATION_MAX_STARS];

static os_guide_pulse_t s_guide_pulse_ra;
static os_guide_pulse_t s_guide_pulse_dec;
static float s_guide_rate_fraction = 0.5f;
static uint32_t s_guide_pulse_ra_start_ms = 0u;
static uint32_t s_guide_pulse_dec_start_ms = 0u;
static uint32_t s_virtual_ms = 0u;

static bool s_manual_active = false;
static uint8_t s_manual_axis = 0u;
static bool s_manual_forward = true;
static double s_manual_speed_arcsec = 500.0;
static double s_custom_manual_speed_arcsec = 500.0;

static bool s_goto_active = false;
static bool s_doing_park = false;
static int32_t s_goto_target_ra = 0;
static int32_t s_goto_target_dec = 0;
static uint32_t s_goto_ra_freq = 0u;
static uint32_t s_goto_dec_freq = 0u;

static bool s_low_power = false;
static os_mount_type_t s_mount_type = OS_MOUNT_EQUATORIAL;
static bool s_tracking_target_valid = false;
static double s_tracking_ra_hours = 0.0;
static double s_tracking_dec_degrees = 0.0;

static os_equatorial_coord_t s_park_coord = { DEFAULT_PARK_RA_HOURS, DEFAULT_PARK_DEC_DEG };

static bool s_pec_enabled = false;
static os_pec_table_t s_pec_table;
static float s_worm_phase_deg = 0.0f;

static bool s_channel_enabled[4] = { 0u, 0u, 0u, 0u };
static bool s_last_lx200_target_valid = false;
static os_equatorial_coord_t s_last_lx200_target = { 0.0f, 0.0f };

static char s_rx_buf[4][OS_MAX_COMMAND_LENGTH];
static uint8_t s_rx_len[4] = { 0u, 0u, 0u, 0u };

static void stop_axis(uint8_t axis);
static void set_motor(uint8_t axis, bool enable, bool forward, uint32_t frequency_hz);
static void update_outputs(void);
static void update_goto_motion(void);
static void update_manual_motion(void);
static double get_tracking_rate_arcsec(void);
static double get_local_sidereal_hours(void);
static void calculate_tracking_frequency(uint8_t axis, uint32_t *frequency_hz, bool *forward);
static void nvm_cal_load(void);
static os_error_t nvm_cal_save(void);
static void nvm_config_load(void);
static os_error_t nvm_config_save(void);
static void load_pec_table_from_nvm(void);
static os_error_t persist_pec_table(void);
static os_error_t fail_init(os_error_t err);
static int32_t clamp_to_int32(double value);
static void forward_calibration(double x_arcsec, double y_arcsec, double *ra_steps, double *dec_steps);
static bool invert_calibration(double ra_steps, double dec_steps, double *x_arcsec, double *y_arcsec);
static bool qr_least_squares(const double *a, const double *b, int n, int m, double *x);
static os_error_t solve_alignment(void);
static float compute_residual_arcsec(void);
static bool alignment_degenerate(void);
static bool command_is(const char *command, size_t length, const char *expected);
static bool parse_guide_command(const char *command, size_t length, uint32_t *duration_ms, os_direction_t *direction);
static void apply_guide_bias_to_axis(uint8_t axis, uint32_t base_freq, bool base_forward, const os_guide_pulse_t *pulse);

static int32_t clamp_to_int32(double value) {
    if (value >= 2147483647.0) {
        return INT32_MAX;
    }
    if (value <= -2147483648.0) {
        return INT32_MIN;
    }
    return (int32_t)value;
}

static void stop_axis(uint8_t axis) {
    (void)os_hal_motor_set_frequency(axis, 0u);
    (void)os_hal_motor_enable(axis, false);
}

static void set_motor(uint8_t axis, bool enable, bool forward, uint32_t frequency_hz) {
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, frequency_hz);
    (void)os_hal_motor_enable(axis, enable);
}

static double get_tracking_rate_arcsec(void) {
    switch (s_track_rate) {
        case OS_TRACK_RATE_LUNAR:
            return OS_SIDEREAL_RATE_ARCSEC_PER_SEC * OS_LUNAR_RATE_FACTOR;
        case OS_TRACK_RATE_SOLAR:
            return OS_SIDEREAL_RATE_ARCSEC_PER_SEC * OS_SOLAR_RATE_FACTOR;
        case OS_TRACK_RATE_CUSTOM:
            return OS_SIDEREAL_RATE_ARCSEC_PER_SEC * (double)s_custom_track_factor;
        case OS_TRACK_RATE_SIDEREAL:
        default:
            return OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
    }
}

static double get_local_sidereal_hours(void) {
    if (!s_site.valid) {
        return 0.0;
    }
    double jd = (double)s_site.utc_epoch_seconds / 86400.0 + 2440587.5;
    double gmst_deg = fmod(280.46061837 + 360.98564736629 * (jd - 2451545.0), 360.0);
    if (gmst_deg < 0.0) {
        gmst_deg += 360.0;
    }
    double lst_deg = gmst_deg + (double)s_site.longitude_degrees;
    double lst_hours = fmod(lst_deg / 15.0, 24.0);
    if (lst_hours < 0.0) {
        lst_hours += 24.0;
    }
    return lst_hours;
}

static void calculate_tracking_frequency(uint8_t axis, uint32_t *frequency_hz, bool *forward) {
    double base_hz = get_tracking_rate_arcsec() * OS_STEPS_PER_ARCSEC;
    *frequency_hz = 0u;
    *forward = true;

    if (s_mount_type == OS_MOUNT_EQUATORIAL) {
        if (axis == AXIS_RA_AZ) {
            *frequency_hz = (uint32_t)base_hz;
            *forward = true;
        } else {
            *frequency_hz = 0u;
            *forward = true;
        }
        return;
    }

    double lat_rad = (double)s_site.latitude_degrees * M_PI / 180.0;
    double dec_rad = (s_tracking_target_valid ? s_tracking_dec_degrees : 0.0) * M_PI / 180.0;
    double lst_hours = get_local_sidereal_hours();
    double ra_hours = s_tracking_target_valid ? s_tracking_ra_hours : 0.0;
    double h_deg = (lst_hours - ra_hours) * 15.0;
    double h_rad = h_deg * M_PI / 180.0;

    double sin_dec = sin(dec_rad);
    double cos_dec = cos(dec_rad);
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_h = sin(h_rad);
    double cos_h = cos(h_rad);

    double sin_alt = sin_dec * sin_lat + cos_dec * cos_lat * cos_h;
    if (sin_alt > 1.0) {
        sin_alt = 1.0;
    } else if (sin_alt < -1.0) {
        sin_alt = -1.0;
    }
    double alt_rad = asin(sin_alt);
    double denom = cos_h * sin_lat * cos_dec - sin_dec * cos_lat;
    double az_rad = atan2(sin_h * cos_dec, denom);
    double cos_alt = cos(alt_rad);
    if (fabs(cos_alt) < 0.05) {
        cos_alt = 0.05;
    }
    double dAlt = cos_lat * sin(az_rad);
    double dAz = cos_lat * cos(az_rad) / cos_alt;

    if (axis == AXIS_RA_AZ) {
        double rate_hz = base_hz * fabs(dAz);
        *frequency_hz = (uint32_t)rate_hz;
        *forward = dAz >= 0.0;
    } else {
        double rate_hz = base_hz * fabs(dAlt);
        *frequency_hz = (uint32_t)rate_hz;
        *forward = dAlt >= 0.0;
    }
}

static void forward_calibration(double x_arcsec, double y_arcsec, double *ra_steps, double *dec_steps) {
    *ra_steps = s_calib.m00 * x_arcsec + s_calib.m01 * y_arcsec + s_calib.off_ra;
    *dec_steps = s_calib.m10 * x_arcsec + s_calib.m11 * y_arcsec + s_calib.off_dec;
}

static bool invert_calibration(double ra_steps, double dec_steps, double *x_arcsec, double *y_arcsec) {
    double b0 = ra_steps - s_calib.off_ra;
    double b1 = dec_steps - s_calib.off_dec;
    double det = s_calib.m00 * s_calib.m11 - s_calib.m01 * s_calib.m10;
    if (fabs(det) < 1e-12) {
        return false;
    }
    double x = (b0 * s_calib.m11 - s_calib.m01 * b1) / det;
    double y = (s_calib.m00 * b1 - b0 * s_calib.m10) / det;
    if (!isfinite(x) || !isfinite(y)) {
        return false;
    }
    *x_arcsec = x;
    *y_arcsec = y;
    return true;
}

static bool qr_least_squares(const double *a, const double *b, int n, int m, double *x) {
    double q[OS_CALIBRATION_MAX_STARS][MAX_ALIGN_PARAMS];
    double r[MAX_ALIGN_PARAMS][MAX_ALIGN_PARAMS];
    double z[MAX_ALIGN_PARAMS];

    memset(r, 0, sizeof(r));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            q[i][j] = a[i * m + j];
        }
    }

    for (int k = 0; k < m; k++) {
        double norm = 0.0;
        for (int i = 0; i < n; i++) {
            norm += q[i][k] * q[i][k];
        }
        norm = sqrt(norm);
        if (!isfinite(norm) || norm < 1e-9) {
            return false;
        }

        r[k][k] = norm;
        for (int i = 0; i < n; i++) {
            q[i][k] /= norm;
        }

        for (int j = k + 1; j < m; j++) {
            double dot = 0.0;
            for (int i = 0; i < n; i++) {
                dot += q[i][k] * q[i][j];
            }
            r[k][j] = dot;
            for (int i = 0; i < n; i++) {
                q[i][j] -= dot * q[i][k];
            }
        }
    }

    for (int k = 0; k < m; k++) {
        z[k] = 0.0;
        for (int i = 0; i < n; i++) {
            z[k] += q[i][k] * b[i];
        }
    }

    for (int k = m - 1; k >= 0; k--) {
        double v = z[k];
        for (int j = k + 1; j < m; j++) {
            v -= r[k][j] * x[j];
        }
        x[k] = v / r[k][k];
        if (!isfinite(x[k])) {
            return false;
        }
    }

    return true;
}

static bool alignment_degenerate(void) {
    int n = (int)s_align_count;
    bool full_matrix_three_star = ((s_align_mode == OS_ALIGN_3STAR) || (s_align_mode == OS_ALIGN_NSTAR)) && (n == 3);
    if (!full_matrix_three_star) {
        return false;
    }

    double x1 = (double)s_align_star_coord[0].ra_hours * 54000.0;
    double y1 = (double)s_align_star_coord[0].dec_degrees * 3600.0;
    double x2 = (double)s_align_star_coord[1].ra_hours * 54000.0;
    double y2 = (double)s_align_star_coord[1].dec_degrees * 3600.0;
    double x3 = (double)s_align_star_coord[2].ra_hours * 54000.0;
    double y3 = (double)s_align_star_coord[2].dec_degrees * 3600.0;
    double det = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
    return fabs(det) < 1e-9;
}

static float compute_residual_arcsec(void) {
    int n = (int)s_align_count;
    double det = s_calib.m00 * s_calib.m11 - s_calib.m01 * s_calib.m10;
    if (fabs(det) < 1e-12) {
        return (float)INFINITY;
    }

    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        double x = (double)s_align_star_coord[i].ra_hours * 54000.0;
        double y = (double)s_align_star_coord[i].dec_degrees * 3600.0;
        double pred_ra = 0.0;
        double pred_dec = 0.0;
        forward_calibration(x, y, &pred_ra, &pred_dec);

        double dr = pred_ra - (double)s_align_motor_pos[i].ra_steps;
        double dd = pred_dec - (double)s_align_motor_pos[i].dec_steps;

        double x_err = (dr * s_calib.m11 - s_calib.m01 * dd) / det;
        double y_err = (s_calib.m00 * dd - dr * s_calib.m10) / det;
        sum += x_err * x_err + y_err * y_err;
    }

    return (float)sqrt(sum / (double)n);
}

static os_error_t solve_alignment(void) {
    int n = (int)s_align_count;

    s_calib_valid = false;
    s_residual_arcsec = 0.0f;

    if (alignment_degenerate()) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    if (s_align_mode == OS_ALIGN_1STAR) {
        double sum_ra = 0.0;
        double sum_dec = 0.0;
        for (int i = 0; i < n; i++) {
            double x = (double)s_align_star_coord[i].ra_hours * 54000.0;
            double y = (double)s_align_star_coord[i].dec_degrees * 3600.0;
            sum_ra += (double)s_align_motor_pos[i].ra_steps - x;
            sum_dec += (double)s_align_motor_pos[i].dec_steps - y;
        }
        s_calib.m00 = 1.0;
        s_calib.m01 = 0.0;
        s_calib.m10 = 0.0;
        s_calib.m11 = 1.0;
        s_calib.off_ra = sum_ra / (double)n;
        s_calib.off_dec = sum_dec / (double)n;
    } else if (s_align_mode == OS_ALIGN_2STAR) {
        double a[OS_CALIBRATION_MAX_STARS * 2];
        double b[OS_CALIBRATION_MAX_STARS];
        double pr[2];
        double pd[2];

        for (int i = 0; i < n; i++) {
            double x = (double)s_align_star_coord[i].ra_hours * 54000.0;
            a[i * 2 + 0] = x;
            a[i * 2 + 1] = 1.0;
            b[i] = (double)s_align_motor_pos[i].ra_steps;
        }
        if (!qr_least_squares(a, b, n, 2, pr)) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        for (int i = 0; i < n; i++) {
            double y = (double)s_align_star_coord[i].dec_degrees * 3600.0;
            a[i * 2 + 0] = y;
            a[i * 2 + 1] = 1.0;
            b[i] = (double)s_align_motor_pos[i].dec_steps;
        }
        if (!qr_least_squares(a, b, n, 2, pd)) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        s_calib.m00 = pr[0];
        s_calib.m01 = 0.0;
        s_calib.m10 = 0.0;
        s_calib.m11 = pd[0];
        s_calib.off_ra = pr[1];
        s_calib.off_dec = pd[1];
    } else {
        double a[OS_CALIBRATION_MAX_STARS * 3];
        double b[OS_CALIBRATION_MAX_STARS];
        double pr[3];
        double pd[3];

        for (int i = 0; i < n; i++) {
            double x = (double)s_align_star_coord[i].ra_hours * 54000.0;
            double y = (double)s_align_star_coord[i].dec_degrees * 3600.0;
            a[i * 3 + 0] = x;
            a[i * 3 + 1] = y;
            a[i * 3 + 2] = 1.0;
            b[i] = (double)s_align_motor_pos[i].ra_steps;
        }
        if (!qr_least_squares(a, b, n, 3, pr)) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        for (int i = 0; i < n; i++) {
            b[i] = (double)s_align_motor_pos[i].dec_steps;
        }
        if (!qr_least_squares(a, b, n, 3, pd)) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        s_calib.m00 = pr[0];
        s_calib.m01 = pr[1];
        s_calib.m10 = pd[0];
        s_calib.m11 = pd[1];
        s_calib.off_ra = pr[2];
        s_calib.off_dec = pd[2];
    }

    s_residual_arcsec = compute_residual_arcsec();
    if (!isfinite(s_residual_arcsec)) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    s_calib_valid = true;
    return OS_ERR_NONE;
}

static void nvm_cal_load(void) {
    nvm_cal_t temp;
    memset(&temp, 0, sizeof(temp));
    if (os_hal_nvm_read(0, (uint8_t *)&temp, (uint16_t)sizeof(temp)) != OS_ERR_NONE) {
        return;
    }
    if (temp.magic != NVM_CAL_MAGIC) {
        return;
    }
    if (!isfinite(temp.m00) || !isfinite(temp.m01) ||
        !isfinite(temp.m10) || !isfinite(temp.m11) ||
        !isfinite(temp.off_ra) || !isfinite(temp.off_dec)) {
        return;
    }

    s_calib.m00 = temp.m00;
    s_calib.m01 = temp.m01;
    s_calib.m10 = temp.m10;
    s_calib.m11 = temp.m11;
    s_calib.off_ra = temp.off_ra;
    s_calib.off_dec = temp.off_dec;
    s_calib_valid = true;
}

static os_error_t nvm_cal_save(void) {
    nvm_cal_t temp;
    memset(&temp, 0, sizeof(temp));
    temp.m00 = s_calib.m00;
    temp.m01 = s_calib.m01;
    temp.m10 = s_calib.m10;
    temp.m11 = s_calib.m11;
    temp.off_ra = s_calib.off_ra;
    temp.off_dec = s_calib.off_dec;
    temp.magic = s_calib_valid ? NVM_CAL_MAGIC : 0u;

    return os_hal_nvm_write(0, (const uint8_t *)&temp, (uint16_t)sizeof(temp));
}

static void nvm_config_load(void) {
    nvm_config_t temp;
    memset(&temp, 0, sizeof(temp));
    if (os_hal_nvm_read((uint16_t)OS_NVM_CALIBRATION_SIZE_BYTES, (uint8_t *)&temp, (uint16_t)sizeof(temp)) != OS_ERR_NONE) {
        return;
    }
    if (temp.magic != NVM_CONFIG_MAGIC) {
        return;
    }
    if (!isfinite(temp.park_ra_hours) || !isfinite(temp.park_dec_degrees)) {
        return;
    }
    if (temp.park_ra_hours < OS_RA_MIN_HOURS || temp.park_ra_hours > OS_RA_MAX_HOURS ||
        temp.park_dec_degrees < OS_DEC_MIN_DEG || temp.park_dec_degrees > OS_DEC_MAX_DEG) {
        return;
    }

    s_park_coord.ra_hours = temp.park_ra_hours;
    s_park_coord.dec_degrees = temp.park_dec_degrees;
}

static os_error_t nvm_config_save(void) {
    nvm_config_t temp;
    memset(&temp, 0, sizeof(temp));
    temp.magic = NVM_CONFIG_MAGIC;
    temp.park_ra_hours = s_park_coord.ra_hours;
    temp.park_dec_degrees = s_park_coord.dec_degrees;

    return os_hal_nvm_write((uint16_t)OS_NVM_CALIBRATION_SIZE_BYTES, (const uint8_t *)&temp, (uint16_t)sizeof(temp));
}

static void load_pec_table_from_nvm(void) {
    os_pec_table_t temp;
    memset(&temp, 0, sizeof(temp));
    if (os_hal_nvm_read((uint16_t)NVM_PEC_BASE, (uint8_t *)&temp, (uint16_t)sizeof(temp)) != OS_ERR_NONE) {
        return;
    }
    memcpy(&s_pec_table, &temp, sizeof(s_pec_table));
}

static os_error_t persist_pec_table(void) {
    return os_hal_nvm_write((uint16_t)NVM_PEC_BASE, (const uint8_t *)&s_pec_table, (uint16_t)sizeof(s_pec_table));
}

static void update_goto_motion(void) {
    if (os_hal_limit_is_triggered(AXIS_RA_AZ) || os_hal_limit_is_triggered(AXIS_DEC_ALT)) {
        stop_axis(AXIS_RA_AZ);
        stop_axis(AXIS_DEC_ALT);
        s_goto_active = false;
        s_doing_park = false;
        s_goto_ra_freq = 0u;
        s_goto_dec_freq = 0u;
        s_state = OS_STATE_FAULT;
        s_last_error = OS_ERR_LIMIT_TRIGGERED;
        return;
    }

    int32_t p0 = os_hal_motor_get_position(AXIS_RA_AZ);
    int32_t p1 = os_hal_motor_get_position(AXIS_DEC_ALT);
    int64_t delta0 = (int64_t)s_goto_target_ra - (int64_t)p0;
    int64_t delta1 = (int64_t)s_goto_target_dec - (int64_t)p1;

    int64_t abs0 = (delta0 < 0) ? -delta0 : delta0;
    int64_t abs1 = (delta1 < 0) ? -delta1 : delta1;

    if (abs0 <= DEFAULT_DEADBAND_STEPS && abs1 <= DEFAULT_DEADBAND_STEPS) {
        stop_axis(AXIS_RA_AZ);
        stop_axis(AXIS_DEC_ALT);
        s_goto_active = false;
        s_goto_ra_freq = 0u;
        s_goto_dec_freq = 0u;
        if (s_doing_park) {
            s_doing_park = false;
            s_tracking_enabled = false;
            s_low_power = true;
            s_state = OS_STATE_PARKED;
        } else {
            s_state = OS_STATE_IDLE_TRACKING;
            s_tracking_enabled = true;
            (void)os_hal_buzzer_beep(200, 1);
        }
        return;
    }

    if (abs0 > DEFAULT_DEADBAND_STEPS) {
        bool forward = delta0 > 0;
        uint32_t freq;
        if (abs0 > GOTO_DECEL_STEP_THRESHOLD) {
            freq = s_goto_ra_freq + GOTO_ACCEL_FREQ_HZ;
            if (freq > DEFAULT_FAST_FREQ_HZ) {
                freq = DEFAULT_FAST_FREQ_HZ;
            }
        } else {
            if (s_goto_ra_freq > DEFAULT_SLOW_FREQ_HZ) {
                freq = (s_goto_ra_freq > GOTO_ACCEL_FREQ_HZ) ? (s_goto_ra_freq - GOTO_ACCEL_FREQ_HZ) : DEFAULT_SLOW_FREQ_HZ;
            } else {
                freq = DEFAULT_SLOW_FREQ_HZ;
            }
        }
        s_goto_ra_freq = freq;
        set_motor(AXIS_RA_AZ, true, forward, freq);
    } else {
        stop_axis(AXIS_RA_AZ);
        s_goto_ra_freq = 0u;
    }

    if (abs1 > DEFAULT_DEADBAND_STEPS) {
        bool forward = delta1 > 0;
        uint32_t freq;
        if (abs1 > GOTO_DECEL_STEP_THRESHOLD) {
            freq = s_goto_dec_freq + GOTO_ACCEL_FREQ_HZ;
            if (freq > DEFAULT_FAST_FREQ_HZ) {
                freq = DEFAULT_FAST_FREQ_HZ;
            }
        } else {
            if (s_goto_dec_freq > DEFAULT_SLOW_FREQ_HZ) {
                freq = (s_goto_dec_freq > GOTO_ACCEL_FREQ_HZ) ? (s_goto_dec_freq - GOTO_ACCEL_FREQ_HZ) : DEFAULT_SLOW_FREQ_HZ;
            } else {
                freq = DEFAULT_SLOW_FREQ_HZ;
            }
        }
        s_goto_dec_freq = freq;
        set_motor(AXIS_DEC_ALT, true, forward, freq);
    } else {
        stop_axis(AXIS_DEC_ALT);
        s_goto_dec_freq = 0u;
    }
}

static void update_manual_motion(void) {
    if (os_hal_limit_is_triggered(s_manual_axis)) {
        stop_axis(s_manual_axis);
        s_manual_active = false;
        s_state = OS_STATE_FAULT;
        s_last_error = OS_ERR_LIMIT_TRIGGERED;
    }
}

static void apply_guide_bias_to_axis(uint8_t axis, uint32_t base_freq, bool base_forward, const os_guide_pulse_t *pulse) {
    if (pulse == NULL || !pulse->active) {
        return;
    }

    bool forward = pulse->direction_east || pulse->direction_north;
    double bias_hz = (double)pulse->rate_fraction * OS_SIDEREAL_RATE_ARCSEC_PER_SEC * OS_STEPS_PER_ARCSEC;
    double signed_base = (double)base_freq * (base_forward ? 1.0 : -1.0);
    double desired = signed_base + (forward ? bias_hz : -bias_hz);
    bool out_forward = desired >= 0.0;
    double out_mag = desired >= 0.0 ? desired : -desired;

    set_motor(axis, true, out_forward, (uint32_t)out_mag);
}

static void update_outputs(void) {
    if (s_state == OS_STATE_GOTO || s_state == OS_STATE_MANUAL_MOTION) {
        return;
    }

    if (s_state != OS_STATE_IDLE_TRACKING) {
        stop_axis(AXIS_RA_AZ);
        stop_axis(AXIS_DEC_ALT);
        return;
    }

    if (!s_tracking_enabled) {
        stop_axis(AXIS_RA_AZ);
        stop_axis(AXIS_DEC_ALT);
        return;
    }

    uint32_t ra_freq = 0u;
    uint32_t dec_freq = 0u;
    bool ra_forward = true;
    bool dec_forward = true;

    calculate_tracking_frequency(AXIS_RA_AZ, &ra_freq, &ra_forward);
    calculate_tracking_frequency(AXIS_DEC_ALT, &dec_freq, &dec_forward);

    if (s_pec_enabled && s_pec_table.valid && s_mount_type == OS_MOUNT_EQUATORIAL) {
        int idx = (int)s_worm_phase_deg;
        if (idx >= OS_PEC_TABLE_SIZE) {
            idx -= OS_PEC_TABLE_SIZE;
        }
        if (idx < 0) {
            idx += OS_PEC_TABLE_SIZE;
        }
        int16_t corr = s_pec_table.corrections[idx];
        int64_t f = (int64_t)ra_freq + corr;
        if (f < 0) {
            f = 0;
        }
        ra_freq = (uint32_t)f;
    }

    (void)os_hal_motor_set_direction(AXIS_RA_AZ, ra_forward);
    (void)os_hal_motor_set_frequency(AXIS_RA_AZ, ra_freq);
    (void)os_hal_motor_set_direction(AXIS_DEC_ALT, dec_forward);
    (void)os_hal_motor_set_frequency(AXIS_DEC_ALT, dec_freq);
    (void)os_hal_motor_enable(AXIS_RA_AZ, true);
    (void)os_hal_motor_enable(AXIS_DEC_ALT, true);

    if (s_state == OS_STATE_IDLE_TRACKING) {
        apply_guide_bias_to_axis(AXIS_RA_AZ, ra_freq, ra_forward, &s_guide_pulse_ra);
        apply_guide_bias_to_axis(AXIS_DEC_ALT, dec_freq, dec_forward, &s_guide_pulse_dec);
    }
}

static void reset_runtime_state(void) {
    s_state = OS_STATE_INITIALIZING;
    s_last_error = OS_ERR_NONE;
    s_nvm_ok = false;
    s_gps_locked = false;
    s_tracking_enabled = false;
    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_custom_track_factor = 1.0f;

    s_calib.m00 = 1.0;
    s_calib.m01 = 0.0;
    s_calib.m10 = 0.0;
    s_calib.m11 = 1.0;
    s_calib.off_ra = 0.0;
    s_calib.off_dec = 0.0;
    s_calib_valid = false;
    s_residual_arcsec = 0.0f;

    s_align_mode = OS_ALIGN_3STAR;
    s_align_count = 0;
    memset(s_align_star_coord, 0, sizeof(s_align_star_coord));
    memset(s_align_motor_pos, 0, sizeof(s_align_motor_pos));

    memset(&s_guide_pulse_ra, 0, sizeof(s_guide_pulse_ra));
    memset(&s_guide_pulse_dec, 0, sizeof(s_guide_pulse_dec));
    s_guide_rate_fraction = 0.5f;
    s_guide_pulse_ra_start_ms = 0u;
    s_guide_pulse_dec_start_ms = 0u;

    s_manual_active = false;
    s_manual_axis = AXIS_RA_AZ;
    s_manual_forward = true;
    s_manual_speed_arcsec = 500.0;
    s_custom_manual_speed_arcsec = 500.0;

    s_goto_active = false;
    s_doing_park = false;
    s_goto_target_ra = 0;
    s_goto_target_dec = 0;
    s_goto_ra_freq = 0u;
    s_goto_dec_freq = 0u;

    s_low_power = false;
    s_mount_type = (os_mount_type_t)OS_MOUNT_TYPE;
    s_tracking_target_valid = false;
    s_tracking_ra_hours = 0.0;
    s_tracking_dec_degrees = 0.0;

    s_park_coord.ra_hours = DEFAULT_PARK_RA_HOURS;
    s_park_coord.dec_degrees = DEFAULT_PARK_DEC_DEG;

    s_pec_enabled = false;
    s_worm_phase_deg = 0.0f;
    memset(&s_pec_table, 0, sizeof(s_pec_table));
    s_pec_table.valid = false;

    s_site.latitude_degrees = 0.0f;
    s_site.longitude_degrees = 0.0f;
    s_site.elevation_metres = 0.0f;
    s_site.utc_epoch_seconds = 0u;
    s_site.valid = true;

    s_last_lx200_target_valid = false;
    s_last_lx200_target.ra_hours = 0.0f;
    s_last_lx200_target.dec_degrees = 0.0f;

    for (int i = 0; i < 4; i++) {
        s_channel_enabled[i] = 0u;
        s_rx_len[i] = 0u;
    }
}

static os_error_t fail_init(os_error_t err) {
    s_last_error = err;
    s_state = OS_STATE_FAULT;
    stop_axis(AXIS_RA_AZ);
    stop_axis(AXIS_DEC_ALT);
    return err;
}

os_error_t os_init(void) {
    reset_runtime_state();

    s_channel_enabled[OS_CHANNEL_USB] = (OS_CHANNEL_USB_ENABLED != 0u);
    s_channel_enabled[OS_CHANNEL_BLUETOOTH] = (OS_CHANNEL_BLUETOOTH_ENABLED != 0u);
    s_channel_enabled[OS_CHANNEL_WIFI] = (OS_CHANNEL_WIFI_ENABLED != 0u);
    s_channel_enabled[OS_CHANNEL_ETHERNET] = (OS_CHANNEL_ETHERNET_ENABLED != 0u);

    if (os_hal_nvm_init() == OS_ERR_NONE) {
        s_nvm_ok = true;
        nvm_cal_load();
        nvm_config_load();
        load_pec_table_from_nvm();
    } else {
        s_last_error = OS_ERR_NVM_FAULT;
    }

    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        if (s_channel_enabled[ch]) {
            (void)os_hal_comm_init(ch);
        }
    }

    os_error_t err = os_hal_motor_init(AXIS_RA_AZ);
    if (err != OS_ERR_NONE) {
        return fail_init(err);
    }
    err = os_hal_motor_init(AXIS_DEC_ALT);
    if (err != OS_ERR_NONE) {
        return fail_init(err);
    }

    stop_axis(AXIS_RA_AZ);
    stop_axis(AXIS_DEC_ALT);

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();

    err = os_hal_timer_motor_init();
    if (err != OS_ERR_NONE) {
        return fail_init(err);
    }

    uint32_t rtc_time = 0u;
    bool rtc_ok = (os_hal_rtc_read(&rtc_time) == OS_ERR_NONE);

    os_site_info_t gps_site;
    memset(&gps_site, 0, sizeof(gps_site));
    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid) {
        bool coords_valid = isfinite(gps_site.latitude_degrees) &&
                            isfinite(gps_site.longitude_degrees) &&
                            isfinite(gps_site.elevation_metres) &&
                            gps_site.latitude_degrees >= -90.0f &&
                            gps_site.latitude_degrees <= 90.0f &&
                            gps_site.longitude_degrees >= -180.0f &&
                            gps_site.longitude_degrees <= 180.0f;
        if (coords_valid) {
            s_site = gps_site;
            s_gps_locked = true;
            if (gps_site.utc_epoch_seconds != 0u) {
                (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
            }
        }
    }

    if (!s_gps_locked && rtc_ok) {
        s_site.utc_epoch_seconds = rtc_time;
        s_site.valid = true;
    }

    if (!s_gps_locked && !rtc_ok) {
        s_site.valid = false;
        return fail_init(OS_ERR_GPS_NO_SIGNAL);
    }

    s_state = OS_STATE_IDLE_TRACKING;
    s_tracking_enabled = true;
    update_outputs();
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    s_virtual_ms += OS_LOOP_MS;

    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        if (!s_channel_enabled[ch]) {
            continue;
        }

        int16_t available = os_hal_comm_available(ch);
        if (available <= 0) {
            continue;
        }

        for (int16_t i = 0; i < available; i++) {
            char c = os_hal_comm_read(ch);
            if (s_rx_len[ch] >= (OS_MAX_COMMAND_LENGTH - 1u)) {
                s_rx_len[ch] = 0u;
                continue;
            }
            s_rx_buf[ch][s_rx_len[ch]++] = c;

            if (c == OS_LX200_CMD_SUFFIX) {
                char reply[OS_MAX_REPLY_LENGTH];
                size_t reply_length = 0u;
                os_error_t err = os_command_parse(s_rx_buf[ch], s_rx_len[ch], ch,
                                                  reply, sizeof(reply), &reply_length);
                if (err != OS_ERR_NONE && reply_length == 0u) {
                    int n = snprintf(reply, sizeof(reply), "%d#", (int)err);
                    if (n > 0) {
                        reply_length = ((size_t)n >= sizeof(reply)) ? (sizeof(reply) - 1u) : (size_t)n;
                    }
                }
                if (reply_length > 0u) {
                    (void)os_hal_comm_write(ch, reply, reply_length);
                }
                s_rx_len[ch] = 0u;
            }
        }
    }

    os_site_info_t gps_site;
    memset(&gps_site, 0, sizeof(gps_site));
    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid) {
        bool coords_valid = isfinite(gps_site.latitude_degrees) &&
                            isfinite(gps_site.longitude_degrees) &&
                            isfinite(gps_site.elevation_metres) &&
                            gps_site.latitude_degrees >= -90.0f &&
                            gps_site.latitude_degrees <= 90.0f &&
                            gps_site.longitude_degrees >= -180.0f &&
                            gps_site.longitude_degrees <= 180.0f;
        if (coords_valid) {
            s_site = gps_site;
            s_gps_locked = true;
            if (gps_site.utc_epoch_seconds != 0u) {
                (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
            }
        }
    } else {
        s_gps_locked = false;
        uint32_t t = 0u;
        if (os_hal_rtc_read(&t) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = t;
            s_site.valid = true;
        } else if (!s_site.valid) {
            s_site.valid = false;
        }
    }

    if (s_pec_enabled && s_pec_table.valid &&
        s_state == OS_STATE_IDLE_TRACKING && s_tracking_enabled &&
        s_mount_type == OS_MOUNT_EQUATORIAL) {
        double steps_per_sec = get_tracking_rate_arcsec() * OS_STEPS_PER_ARCSEC;
        s_worm_phase_deg += (float)(steps_per_sec * 360.0 / (double)PEC_WORM_STEPS_PER_REV);
        if (s_worm_phase_deg >= 360.0f) {
            s_worm_phase_deg -= 360.0f;
        }
    }

    if (s_guide_pulse_ra.active &&
        (s_virtual_ms - s_guide_pulse_ra_start_ms) >= s_guide_pulse_ra.duration_ms) {
        s_guide_pulse_ra.active = false;
    }
    if (s_guide_pulse_dec.active &&
        (s_virtual_ms - s_guide_pulse_dec_start_ms) >= s_guide_pulse_dec.duration_ms) {
        s_guide_pulse_dec.active = false;
    }

    if (s_state == OS_STATE_GOTO && s_goto_active) {
        update_goto_motion();
    } else if (s_state == OS_STATE_MANUAL_MOTION && s_manual_active) {
        update_manual_motion();
    }

    update_outputs();
}

static bool command_is(const char *command, size_t length, const char *expected) {
    size_t expected_length = strlen(expected);
    if (length != expected_length) {
        return false;
    }
    return memcmp(command, expected, length) == 0;
}

static bool parse_guide_command(const char *command, size_t length, uint32_t *duration_ms, os_direction_t *direction) {
    if (command == NULL || duration_ms == NULL || direction == NULL) {
        return false;
    }
    if (length < 5u || memcmp(command, ":Mg", 3u) != 0) {
        return false;
    }

    char d = command[3];
    switch (d) {
        case 'e':
            *direction = OS_DIRECTION_EAST;
            break;
        case 'w':
            *direction = OS_DIRECTION_WEST;
            break;
        case 'n':
            *direction = OS_DIRECTION_NORTH;
            break;
        case 's':
            *direction = OS_DIRECTION_SOUTH;
            break;
        default:
            return false;
    }

    if (length == 5u && command[4] == OS_LX200_CMD_SUFFIX) {
        *duration_ms = DEFAULT_GUIDE_PULSE_MS;
        return true;
    }

    if (length > 5u) {
        char num[32];
        size_t num_len = length - 5u;
        if (num_len >= sizeof(num)) {
            return false;
        }
        memcpy(num, command + 4, num_len);
        num[num_len] = '\0';

        char *endptr = NULL;
        unsigned long value = strtoul(num, &endptr, 10);
        if (endptr == num || *endptr != '\0' || value > UINT32_MAX) {
            return false;
        }
        *duration_ms = (uint32_t)value;
        return true;
    }

    return false;
}

static void write_int_reply(char *reply_buffer, size_t reply_buffer_size, size_t *reply_length, int value) {
    int n = snprintf(reply_buffer, reply_buffer_size, "%d#", value);
    if (n < 0) {
        *reply_length = 0u;
        return;
    }
    *reply_length = ((size_t)n >= reply_buffer_size) ? (reply_buffer_size - 1u) : (size_t)n;
}

static void write_float_reply(char *reply_buffer, size_t reply_buffer_size, size_t *reply_length, double value) {
    int n = snprintf(reply_buffer, reply_buffer_size, "%.4f#", value);
    if (n < 0) {
        *reply_length = 0u;
        return;
    }
    *reply_length = ((size_t)n >= reply_buffer_size) ? (reply_buffer_size - 1u) : (size_t)n;
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
    if (length == 0u || length > OS_MAX_COMMAND_LENGTH || reply_buffer_size == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    reply_buffer[0] = '\0';
    *reply_length = 0u;

    if (length < 2u || command[0] != OS_LX200_CMD_PREFIX || command[length - 1u] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    if (command_is(command, length, ":GR#")) {
        os_equatorial_coord_t coord;
        os_error_t err = os_query_coordinates(&coord);
        if (err != OS_ERR_NONE) {
            return err;
        }
        write_float_reply(reply_buffer, reply_buffer_size, reply_length, (double)coord.ra_hours);
        return OS_ERR_NONE;
    }

    if (command_is(command, length, ":GD#")) {
        os_equatorial_coord_t coord;
        os_error_t err = os_query_coordinates(&coord);
        if (err != OS_ERR_NONE) {
            return err;
        }
        write_float_reply(reply_buffer, reply_buffer_size, reply_length, (double)coord.dec_degrees);
        return OS_ERR_NONE;
    }

    if (command_is(command, length, ":GVP#")) {
        uint8_t major = 0u;
        uint8_t minor = 0u;
        uint8_t patch = 0u;
        os_error_t err = os_query_firmware_version(&major, &minor, &patch);
        if (err != OS_ERR_NONE) {
            return err;
        }
        int n = snprintf(reply_buffer, reply_buffer_size, "%u.%u.%u#",
                         (unsigned)major, (unsigned)minor, (unsigned)patch);
        if (n < 0) {
            return OS_ERR_COMMAND_FORMAT;
        }
        *reply_length = ((size_t)n >= reply_buffer_size) ? (reply_buffer_size - 1u) : (size_t)n;
        return OS_ERR_NONE;
    }

    if (command_is(command, length, ":GL#")) {
        write_float_reply(reply_buffer, reply_buffer_size, reply_length, get_local_sidereal_hours());
        return OS_ERR_NONE;
    }

    if (command_is(command, length, ":GG#")) {
        bool locked = false;
        os_error_t err = os_query_gps_locked(&locked);
        if (err != OS_ERR_NONE) {
            return err;
        }
        write_int_reply(reply_buffer, reply_buffer_size, reply_length, locked ? 1 : 0);
        return OS_ERR_NONE;
    }

    if (command_is(command, length, ":Gt#")) {
        write_float_reply(reply_buffer, reply_buffer_size, reply_length, (double)s_site.latitude_degrees);
        return OS_ERR_NONE;
    }

    if (command_is(command, length, ":Gg#")) {
        write_float_reply(reply_buffer, reply_buffer_size, reply_length, (double)s_site.longitude_degrees);
        return OS_ERR_NONE;
    }

    if (command_is(command, length, ":Ga#")) {
        write_float_reply(reply_buffer, reply_buffer_size, reply_length, (double)s_site.elevation_metres);
        return OS_ERR_NONE;
    }

    if (command_is(command, length, ":Gstat#")) {
        const char *state_name = "unknown";
        switch (s_state) {
            case OS_STATE_INITIALIZING:
                state_name = "initializing";
                break;
            case OS_STATE_IDLE_TRACKING:
                state_name = "idle_tracking";
                break;
            case OS_STATE_GOTO:
                state_name = "goto";
                break;
            case OS_STATE_ALIGNMENT:
                state_name = "alignment";
                break;
            case OS_STATE_MANUAL_MOTION:
                state_name = "manual_motion";
                break;
            case OS_STATE_PARKED:
                state_name = "parked";
                break;
            case OS_STATE_FAULT:
                state_name = "fault";
                break;
            default:
                break;
        }
        int n = snprintf(reply_buffer, reply_buffer_size, "%s#", state_name);
        if (n < 0) {
            return OS_ERR_COMMAND_FORMAT;
        }
        *reply_length = ((size_t)n >= reply_buffer_size) ? (reply_buffer_size - 1u) : (size_t)n;
        return OS_ERR_NONE;
    }

    if (command_is(command, length, ":GVD#")) {
        uint8_t major = 0u;
        uint8_t minor = 0u;
        uint8_t patch = 0u;
        os_error_t err = os_query_firmware_version(&major, &minor, &patch);
        if (err != OS_ERR_NONE) {
            return err;
        }
        int n = snprintf(reply_buffer, reply_buffer_size, "%u.%u.%u#",
                         (unsigned)major, (unsigned)minor, (unsigned)patch);
        if (n < 0) {
            return OS_ERR_COMMAND_FORMAT;
        }
        *reply_length = ((size_t)n >= reply_buffer_size) ? (reply_buffer_size - 1u) : (size_t)n;
        return OS_ERR_NONE;
    }

    if (command_is(command, length, ":Me#")) {
        return os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
    }
    if (command_is(command, length, ":Mw#")) {
        return os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
    }
    if (command_is(command, length, ":Mn#")) {
        return os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
    }
    if (command_is(command, length, ":Ms#")) {
        return os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
    }

    if (command_is(command, length, ":hP#")) {
        return os_park();
    }
    if (command_is(command, length, ":hO#")) {
        return os_unpark();
    }

    if (command_is(command, length, ":MS#")) {
        if (!s_last_lx200_target_valid) {
            return OS_ERR_INVALID_STATE;
        }
        os_error_t err = os_goto_equatorial(s_last_lx200_target);
        if (err != OS_ERR_NONE) {
            return err;
        }
        write_int_reply(reply_buffer, reply_buffer_size, reply_length, 1);
        return OS_ERR_NONE;
    }

    uint32_t guide_duration_ms = 0u;
    os_direction_t guide_direction = OS_DIRECTION_NORTH;
    if (parse_guide_command(command, length, &guide_duration_ms, &guide_direction)) {
        os_error_t err = os_guide_pulse(guide_direction, guide_duration_ms);
        if (err != OS_ERR_NONE) {
            return err;
        }
        write_int_reply(reply_buffer, reply_buffer_size, reply_length, 1);
        return OS_ERR_NONE;
    }

    if (command_is(command, length, ":Q#")) {
        stop_axis(AXIS_RA_AZ);
        stop_axis(AXIS_DEC_ALT);
        s_manual_active = false;
        s_goto_active = false;
        s_doing_park = false;
        s_goto_ra_freq = 0u;
        s_goto_dec_freq = 0u;
        s_state = OS_STATE_IDLE_TRACKING;
        return OS_ERR_NONE;
    }

    if (length > 3u && memcmp(command, ":Sr", 3u) == 0) {
        char inner[OS_MAX_COMMAND_LENGTH + 1];
        size_t inner_len = length - 2u;
        if (inner_len >= sizeof(inner)) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        memcpy(inner, command + 1, inner_len);
        inner[inner_len] = '\0';

        float ra_hours = 0.0f;
        float dec_degrees = 0.0f;
        if (sscanf(inner, "Sr%f:%f", &ra_hours, &dec_degrees) != 2) {
            return OS_ERR_COMMAND_FORMAT;
        }

        os_equatorial_coord_t target;
        target.ra_hours = ra_hours;
        target.dec_degrees = dec_degrees;
        os_error_t err = os_goto_equatorial(target);
        if (err != OS_ERR_NONE) {
            return err;
        }

        s_last_lx200_target_valid = true;
        s_last_lx200_target = target;

        write_int_reply(reply_buffer, reply_buffer_size, reply_length, 1);
        return OS_ERR_NONE;
    }

    return OS_ERR_NOT_SUPPORTED;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (!isfinite(target.ra_hours) || !isfinite(target.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(AXIS_RA_AZ) || os_hal_limit_is_triggered(AXIS_DEC_ALT)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    double x = (double)target.ra_hours * 54000.0;
    double y = (double)target.dec_degrees * 3600.0;
    double ra_steps = 0.0;
    double dec_steps = 0.0;
    forward_calibration(x, y, &ra_steps, &dec_steps);

    int32_t target_ra = clamp_to_int32(ra_steps);
    int32_t target_dec = clamp_to_int32(dec_steps);
    int32_t cur0 = os_hal_motor_get_position(AXIS_RA_AZ);
    int32_t cur1 = os_hal_motor_get_position(AXIS_DEC_ALT);
    int64_t d0 = (int64_t)target_ra - (int64_t)cur0;
    int64_t d1 = (int64_t)target_dec - (int64_t)cur1;
    int64_t a0 = (d0 < 0) ? -d0 : d0;
    int64_t a1 = (d1 < 0) ? -d1 : d1;

    if (a0 <= DEFAULT_DEADBAND_STEPS && a1 <= DEFAULT_DEADBAND_STEPS) {
        s_goto_active = false;
        s_goto_ra_freq = 0u;
        s_goto_dec_freq = 0u;
        s_state = OS_STATE_IDLE_TRACKING;
        return OS_ERR_NONE;
    }

    s_goto_target_ra = target_ra;
    s_goto_target_dec = target_dec;
    s_goto_active = true;
    s_doing_park = false;
    s_goto_ra_freq = 0u;
    s_goto_dec_freq = 0u;
    s_tracking_target_valid = true;
    s_tracking_ra_hours = (double)target.ra_hours;
    s_tracking_dec_degrees = (double)target.dec_degrees;
    s_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (!isfinite(target.azimuth_degrees) || !isfinite(target.altitude_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(AXIS_RA_AZ) || os_hal_limit_is_triggered(AXIS_DEC_ALT)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    double x_arcsec = 0.0;
    double y_arcsec = 0.0;
    bool target_ra_dec_valid = false;

    if (s_mount_type == OS_MOUNT_EQUATORIAL) {
        if (!s_site.valid) {
            return OS_ERR_GPS_NO_SIGNAL;
        }

        double alt_rad = (double)target.altitude_degrees * M_PI / 180.0;
        double az_rad = (double)target.azimuth_degrees * M_PI / 180.0;
        double lat_rad = (double)s_site.latitude_degrees * M_PI / 180.0;

        double sin_alt = sin(alt_rad);
        double cos_alt = cos(alt_rad);
        double sin_lat = sin(lat_rad);
        double cos_lat = cos(lat_rad);
        double cos_az = cos(az_rad);
        double sin_az = sin(az_rad);

        double sin_dec = sin_alt * sin_lat + cos_alt * cos_lat * cos_az;
        if (sin_dec > 1.0) {
            sin_dec = 1.0;
        } else if (sin_dec < -1.0) {
            sin_dec = -1.0;
        }
        double dec_rad = asin(sin_dec);
        double cos_dec = cos(dec_rad);

        double h_rad = 0.0;
        double denom = cos_lat * cos_dec;
        if (fabs(denom) < 1e-9) {
            h_rad = 0.0;
        } else {
            double cos_h = (sin_alt - sin_lat * sin_dec) / denom;
            double sin_h = -sin_az * cos_alt / cos_dec;
            h_rad = atan2(sin_h, cos_h);
        }

        double lst_hours = get_local_sidereal_hours();
        double ra_hours = lst_hours - (h_rad * 12.0 / M_PI);
        ra_hours = fmod(ra_hours, 24.0);
        if (ra_hours < 0.0) {
            ra_hours += 24.0;
        }
        double dec_degrees = dec_rad * 180.0 / M_PI;

        x_arcsec = ra_hours * 54000.0;
        y_arcsec = dec_degrees * 3600.0;

        s_tracking_target_valid = true;
        s_tracking_ra_hours = ra_hours;
        s_tracking_dec_degrees = dec_degrees;
        target_ra_dec_valid = true;
    } else {
        x_arcsec = (double)target.azimuth_degrees * 3600.0;
        y_arcsec = (double)target.altitude_degrees * 3600.0;
        s_tracking_target_valid = false;
    }

    double ra_steps = 0.0;
    double dec_steps = 0.0;
    if (s_calib_valid) {
        forward_calibration(x_arcsec, y_arcsec, &ra_steps, &dec_steps);
    } else {
        ra_steps = x_arcsec;
        dec_steps = y_arcsec;
        if (!target_ra_dec_valid && s_mount_type == OS_MOUNT_ALTAZ && OS_STEPS_PER_ARCSEC != 1.0) {
            ra_steps = x_arcsec * OS_STEPS_PER_ARCSEC;
            dec_steps = y_arcsec * OS_STEPS_PER_ARCSEC;
        }
    }

    int32_t target_ra = clamp_to_int32(ra_steps);
    int32_t target_dec = clamp_to_int32(dec_steps);
    int32_t cur0 = os_hal_motor_get_position(AXIS_RA_AZ);
    int32_t cur1 = os_hal_motor_get_position(AXIS_DEC_ALT);
    int64_t d0 = (int64_t)target_ra - (int64_t)cur0;
    int64_t d1 = (int64_t)target_dec - (int64_t)cur1;
    int64_t a0 = (d0 < 0) ? -d0 : d0;
    int64_t a1 = (d1 < 0) ? -d1 : d1;

    if (a0 <= DEFAULT_DEADBAND_STEPS && a1 <= DEFAULT_DEADBAND_STEPS) {
        s_goto_active = false;
        s_goto_ra_freq = 0u;
        s_goto_dec_freq = 0u;
        s_state = OS_STATE_IDLE_TRACKING;
        return OS_ERR_NONE;
    }

    s_goto_target_ra = target_ra;
    s_goto_target_dec = target_dec;
    s_goto_active = true;
    s_doing_park = false;
    s_goto_ra_freq = 0u;
    s_goto_dec_freq = 0u;
    s_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (!s_goto_active && s_state != OS_STATE_GOTO) {
        return OS_ERR_INVALID_STATE;
    }

    s_goto_active = false;
    s_doing_park = false;
    s_goto_ra_freq = 0u;
    s_goto_dec_freq = 0u;
    stop_axis(AXIS_RA_AZ);
    stop_axis(AXIS_DEC_ALT);
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if ((int)rate < (int)OS_TRACK_RATE_SIDEREAL || (int)rate > (int)OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM) {
        if (!isfinite(custom_factor) || custom_factor <= 0.0f) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        s_custom_track_factor = custom_factor;
    }

    s_track_rate = rate;
    update_outputs();
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = s_track_rate;
    *custom_factor = s_custom_track_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    s_tracking_enabled = true;
    update_outputs();
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    s_tracking_enabled = false;
    update_outputs();
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (!isfinite(rate_fraction) ||
        rate_fraction < OS_GUIDE_RATE_MIN ||
        rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_guide_rate_fraction = rate_fraction;
    if (s_guide_pulse_ra.active) {
        s_guide_pulse_ra.rate_fraction = rate_fraction;
    }
    if (s_guide_pulse_dec.active) {
        s_guide_pulse_dec.rate_fraction = rate_fraction;
    }
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if ((int)direction < (int)OS_DIRECTION_NORTH || (int)direction > (int)OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }

    bool is_dec = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);
    os_guide_pulse_t *pulse = is_dec ? &s_guide_pulse_dec : &s_guide_pulse_ra;
    uint32_t *start_ms = is_dec ? &s_guide_pulse_dec_start_ms : &s_guide_pulse_ra_start_ms;

    pulse->active = true;
    pulse->duration_ms = duration_ms;
    pulse->rate_fraction = s_guide_rate_fraction;
    pulse->direction_east = (direction == OS_DIRECTION_EAST);
    pulse->direction_north = (direction == OS_DIRECTION_NORTH);
    pulse->dec_priority = is_dec;
    *start_ms = s_virtual_ms;

    update_outputs();
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_guide_pulse_dec.active) {
        *pulse = s_guide_pulse_dec;
    } else if (s_guide_pulse_ra.active) {
        *pulse = s_guide_pulse_ra;
    } else {
        memset(pulse, 0, sizeof(*pulse));
    }
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if ((int)mode < (int)OS_ALIGN_1STAR || (int)mode > (int)OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }

    s_align_mode = mode;
    s_align_count = 0u;
    s_residual_arcsec = 0.0f;
    s_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (!isfinite(star_coord.ra_hours) || !isfinite(star_coord.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (star_coord.ra_hours < OS_RA_MIN_HOURS || star_coord.ra_hours > OS_RA_MAX_HOURS) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (star_coord.dec_degrees < OS_DEC_MIN_DEG || star_coord.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_align_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    s_align_star_coord[s_align_count] = star_coord;
    s_align_motor_pos[s_align_count] = motor_pos;
    s_align_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t min_stars = 1u;
    if (s_align_mode == OS_ALIGN_2STAR) {
        min_stars = 2u;
    } else if (s_align_mode == OS_ALIGN_3STAR || s_align_mode == OS_ALIGN_NSTAR) {
        min_stars = 3u;
    }

    if (s_align_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    os_error_t err = solve_alignment();
    if (err != OS_ERR_NONE) {
        return err;
    }

    if (s_align_count >= 4u && s_residual_arcsec > CALIBRATION_MAX_RESIDUAL_ARCSEC) {
        s_calib_valid = false;
        return OS_ERR_CALIBRATION_FAILED;
    }

    err = nvm_cal_save();
    if (err != OS_ERR_NONE) {
        s_calib_valid = false;
        s_last_error = OS_ERR_NVM_FAULT;
        s_state = OS_STATE_FAULT;
        return OS_ERR_NVM_FAULT;
    }

    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_calib_valid) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = s_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    if (s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    s_align_count = 0u;
    s_residual_arcsec = 0.0f;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!isfinite(park_pos.ra_hours) || !isfinite(park_pos.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (park_pos.ra_hours < OS_RA_MIN_HOURS || park_pos.ra_hours > OS_RA_MAX_HOURS) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (park_pos.dec_degrees < OS_DEC_MIN_DEG || park_pos.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_park_coord = park_pos;
    os_error_t err = nvm_config_save();
    if (err != OS_ERR_NONE) {
        s_last_error = err;
        return err;
    }
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(AXIS_RA_AZ) || os_hal_limit_is_triggered(AXIS_DEC_ALT)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    double x = (double)s_park_coord.ra_hours * 54000.0;
    double y = (double)s_park_coord.dec_degrees * 3600.0;
    double ra_steps = 0.0;
    double dec_steps = 0.0;
    forward_calibration(x, y, &ra_steps, &dec_steps);

    s_goto_target_ra = clamp_to_int32(ra_steps);
    s_goto_target_dec = clamp_to_int32(dec_steps);
    s_goto_active = true;
    s_doing_park = true;
    s_goto_ra_freq = 0u;
    s_goto_dec_freq = 0u;
    s_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (s_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    os_error_t err = os_hal_motor_init(AXIS_RA_AZ);
    if (err != OS_ERR_NONE) {
        s_last_error = err;
        s_state = OS_STATE_FAULT;
        return err;
    }
    err = os_hal_motor_init(AXIS_DEC_ALT);
    if (err != OS_ERR_NONE) {
        s_last_error = err;
        s_state = OS_STATE_FAULT;
        return err;
    }
    err = os_hal_motor_enable(AXIS_RA_AZ, true);
    if (err != OS_ERR_NONE) {
        s_last_error = err;
        s_state = OS_STATE_FAULT;
        return err;
    }
    err = os_hal_motor_enable(AXIS_DEC_ALT, true);
    if (err != OS_ERR_NONE) {
        s_last_error = err;
        s_state = OS_STATE_FAULT;
        return err;
    }

    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        if (s_channel_enabled[ch]) {
            (void)os_hal_comm_init(ch);
        }
    }

    if (s_gps_locked && s_site.valid && s_site.utc_epoch_seconds != 0u) {
        (void)os_hal_rtc_set(s_site.utc_epoch_seconds);
    }

    s_low_power = false;
    s_state = OS_STATE_IDLE_TRACKING;
    s_tracking_enabled = true;
    update_outputs();
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (!isfinite(arcsec_per_sec) || arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_custom_manual_speed_arcsec = (double)arcsec_per_sec;

    if (s_manual_active && s_state == OS_STATE_MANUAL_MOTION) {
        uint32_t freq = (uint32_t)(s_custom_manual_speed_arcsec * OS_STEPS_PER_ARCSEC);
        set_motor(s_manual_axis, true, s_manual_forward, freq);
    }
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if ((int)direction < (int)OS_DIRECTION_NORTH || (int)direction > (int)OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((int)speed < (int)OS_SPEED_SLOW || (int)speed > (int)OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (speed == OS_SPEED_CUSTOM && !(s_custom_manual_speed_arcsec > 0.0)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t axis = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST)
                       ? AXIS_RA_AZ
                       : AXIS_DEC_ALT;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    double speed_arcsec = 0.0;
    switch (speed) {
        case OS_SPEED_SLOW:
            speed_arcsec = OS_CONFIG_MANUAL_SLOW_ARCSEC_PER_SEC;
            break;
        case OS_SPEED_MEDIUM:
            speed_arcsec = OS_CONFIG_MANUAL_MEDIUM_ARCSEC_PER_SEC;
            break;
        case OS_SPEED_FAST:
            speed_arcsec = OS_CONFIG_MANUAL_FAST_ARCSEC_PER_SEC;
            break;
        case OS_SPEED_CUSTOM:
        default:
            speed_arcsec = s_custom_manual_speed_arcsec;
            break;
    }

    s_manual_axis = axis;
    s_manual_forward = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_NORTH);
    s_manual_speed_arcsec = speed_arcsec;
    s_manual_active = true;
    s_state = OS_STATE_MANUAL_MOTION;

    uint32_t freq = (uint32_t)(speed_arcsec * OS_STEPS_PER_ARCSEC);
    set_motor(axis, true, s_manual_forward, freq);
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (s_state != OS_STATE_MANUAL_MOTION || !s_manual_active) {
        return OS_ERR_INVALID_STATE;
    }

    stop_axis(s_manual_axis);
    s_manual_active = false;
    s_state = OS_STATE_IDLE_TRACKING;
    update_outputs();
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = s_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    double ra_steps = (double)os_hal_motor_get_position(AXIS_RA_AZ);
    double dec_steps = (double)os_hal_motor_get_position(AXIS_DEC_ALT);
    double x_arcsec = 0.0;
    double y_arcsec = 0.0;

    if (!invert_calibration(ra_steps, dec_steps, &x_arcsec, &y_arcsec)) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    coord->ra_hours = (float)(x_arcsec / 54000.0);
    coord->dec_degrees = (float)(y_arcsec / 3600.0);
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = s_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = os_hal_motor_get_position(AXIS_RA_AZ);
    pos->dec_steps = os_hal_motor_get_position(AXIS_DEC_ALT);
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
    *moving = (s_goto_active || s_manual_active || s_guide_pulse_ra.active || s_guide_pulse_dec.active ||
               s_state == OS_STATE_GOTO || s_state == OS_STATE_MANUAL_MOTION);
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *locked = s_gps_locked;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    s_pec_enabled = enable;
    update_outputs();
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(&s_pec_table, table, sizeof(os_pec_table_t));
    s_pec_table.valid = true;
    os_error_t err = persist_pec_table();
    if (err != OS_ERR_NONE) {
        s_last_error = err;
        return err;
    }
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(table, &s_pec_table, sizeof(os_pec_table_t));
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (!isfinite(worm_phase_deg) ||
        worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    int index = (int)worm_phase_deg;
    if (index >= OS_PEC_TABLE_SIZE) {
        index -= OS_PEC_TABLE_SIZE;
    }
    if (index < 0) {
        index += OS_PEC_TABLE_SIZE;
    }

    s_pec_table.corrections[index] = error_arcsec;
    s_pec_table.valid = true;
    return persist_pec_table();
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    calib->matrix_ra_to_ra = (float)s_calib.m00;
    calib->matrix_ra_to_dec = (float)s_calib.m01;
    calib->matrix_dec_to_ra = (float)s_calib.m10;
    calib->matrix_dec_to_dec = (float)s_calib.m11;
    calib->offset_ra_arcsec = (float)s_calib.off_ra;
    calib->offset_dec_arcsec = (float)s_calib.off_dec;
    calib->valid = s_calib_valid;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    s_calib.m00 = 1.0;
    s_calib.m01 = 0.0;
    s_calib.m10 = 0.0;
    s_calib.m11 = 1.0;
    s_calib.off_ra = 0.0;
    s_calib.off_dec = 0.0;
    s_calib_valid = false;
    s_residual_arcsec = 0.0f;

    os_error_t err = nvm_cal_save();
    if (err != OS_ERR_NONE) {
        s_last_error = err;
        return err;
    }
    return OS_ERR_NONE;
}
#include "6_generated_code.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef OS_CONFIG_MOUNT_TYPE
#define OS_CONFIG_MOUNT_TYPE OS_MOUNT_EQUATORIAL
#endif

#define STEPS_PER_DEGREE 3600.0
#define DEFAULT_PARK_RA_HOURS  0.0f
#define DEFAULT_PARK_DEC_DEG   90.0f
#define MAX_MOTOR_FREQ_HZ      20000U
#define GOTO_FREQ_HZ           8000U
#define GOTO_START_FREQ_HZ     100U
#define GOTO_ACCEL_STEP_HZ     400U
#define GOTO_DECEL_DIST_STEPS  800
#define MANUAL_FREQ_SLOW       100U
#define MANUAL_FREQ_MEDIUM     500U
#define MANUAL_FREQ_FAST       1500U
#define CALIBRATION_RESIDUAL_LIMIT_ARCSEC 300.0f
#define RX_BUFFER_SIZE         (OS_MAX_COMMAND_LENGTH + 2U)
#define SIDEREAL_DAY_SECONDS   86164.0905
#define NVM_CALIB_OFFSET       0U
#define NVM_CONFIG_OFFSET      OS_NVM_CALIBRATION_SIZE_BYTES
#define NVM_PEC_OFFSET         (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)

typedef struct {
    uint8_t pec_enabled;
} nvm_config_t;

static void update_current_from_steps(void);
static double local_sidereal_hours(uint32_t epoch_seconds, float longitude_degrees);

static os_state_t s_state = OS_STATE_INITIALIZING;
static os_mount_type_t s_mount_type = OS_CONFIG_MOUNT_TYPE;

static bool s_tracking_enabled = true;
static os_track_rate_t s_track_rate = OS_TRACK_RATE_SIDEREAL;
static float s_custom_track_factor = 1.0f;

static os_site_info_t s_site = {0.0f, 0.0f, 0.0f, 0U, false};
static bool s_gps_locked = false;

static os_calibration_t s_calib;
static bool s_alignment_active = false;
static bool s_alignment_residual_computed = false;
static float s_residual_arcsec = 0.0f;
static os_align_mode_t s_align_mode = OS_ALIGN_1STAR;
static int s_align_count = 0;
static double s_align_x[OS_CALIBRATION_MAX_STARS];
static double s_align_y[OS_CALIBRATION_MAX_STARS];
static double s_align_ra_steps[OS_CALIBRATION_MAX_STARS];
static double s_align_dec_steps[OS_CALIBRATION_MAX_STARS];

static bool s_goto_active = false;
static bool s_park_active = false;
static bool s_manual_moving = false;
static bool s_moving = false;
static int32_t s_goto_target_ra = 0;
static int32_t s_goto_target_dec = 0;
static uint32_t s_goto_axis_freq[2] = {0U, 0U};

static os_direction_t s_move_direction = OS_DIRECTION_NORTH;
static os_speed_level_t s_move_speed = OS_SPEED_SLOW;
static float s_custom_manual_arcsec_per_sec = 100.0f;

static float s_guide_rate = 0.5f;
static os_guide_pulse_t s_guide_pulse;
static uint32_t s_guide_start_rtc_seconds = 0U;
static bool s_guide_start_valid = false;
static os_direction_t s_guide_direction = OS_DIRECTION_NORTH;

static bool s_pec_enabled = false;
static os_pec_table_t s_pec_table;
static float s_worm_phase_deg = 0.0f;
static uint32_t s_last_rtc_seconds = 0U;
static bool s_last_rtc_valid = false;

static os_equatorial_coord_t s_current_coord = {0.0f, 0.0f};
static os_equatorial_coord_t s_park_coord = {DEFAULT_PARK_RA_HOURS, DEFAULT_PARK_DEC_DEG};

static bool s_pending_ra_set = false;
static bool s_pending_dec_set = false;
static float s_pending_ra_hours = 0.0f;
static float s_pending_dec_degrees = 0.0f;

static char s_rx[4][RX_BUFFER_SIZE];
static uint8_t s_rx_len[4];

static int32_t round_to_i32(double v) {
    if (v >= 0.0) {
        return (int32_t)(v + 0.5);
    }
    return (int32_t)(v - 0.5);
}

static float wrap_ra_hours(float hours) {
    while (hours < 0.0f) {
        hours += 24.0f;
    }
    while (hours >= 24.0f) {
        hours -= 24.0f;
    }
    return hours;
}

static float clamp_dec_degrees(float degrees) {
    if (degrees > OS_DEC_MAX_DEG) {
        return OS_DEC_MAX_DEG;
    }
    if (degrees < OS_DEC_MIN_DEG) {
        return OS_DEC_MIN_DEG;
    }
    return degrees;
}

static uint32_t clamp_freq(double freq) {
    if (freq < 0.0) {
        return 0U;
    }
    if (freq > (double)MAX_MOTOR_FREQ_HZ) {
        return MAX_MOTOR_FREQ_HZ;
    }
    return (uint32_t)freq;
}

static bool valid_channel(uint8_t channel) {
    return channel <= OS_CHANNEL_ETHERNET;
}

static bool valid_rate(os_track_rate_t rate) {
    return rate >= OS_TRACK_RATE_SIDEREAL && rate <= OS_TRACK_RATE_CUSTOM;
}

static bool valid_direction(os_direction_t direction) {
    return direction >= OS_DIRECTION_NORTH && direction <= OS_DIRECTION_WEST;
}

static bool valid_speed(os_speed_level_t speed) {
    return speed >= OS_SPEED_SLOW && speed <= OS_SPEED_CUSTOM;
}

static bool valid_equatorial_coord(os_equatorial_coord_t coord) {
    return coord.ra_hours >= OS_RA_MIN_HOURS &&
           coord.ra_hours <= OS_RA_MAX_HOURS &&
           coord.dec_degrees >= OS_DEC_MIN_DEG &&
           coord.dec_degrees <= OS_DEC_MAX_DEG;
}

static bool valid_horizontal_coord(os_horizontal_coord_t coord) {
    return coord.azimuth_degrees >= 0.0f &&
           coord.azimuth_degrees <= 360.0f &&
           coord.altitude_degrees >= OS_DEC_MIN_DEG &&
           coord.altitude_degrees <= OS_DEC_MAX_DEG;
}

static void stop_motors(void) {
    os_hal_motor_set_frequency(0, 0U);
    os_hal_motor_set_frequency(1, 0U);
}

static void coord_to_steps(os_equatorial_coord_t coord, int32_t *ra_steps, int32_t *dec_steps) {
    double ra_deg = (double)coord.ra_hours * 15.0;
    double dec_deg = (double)coord.dec_degrees;

    if (s_calib.valid) {
        *ra_steps = round_to_i32((double)s_calib.matrix_ra_to_ra * ra_deg +
                                 (double)s_calib.matrix_ra_to_dec * dec_deg +
                                 (double)s_calib.offset_ra_arcsec);
        *dec_steps = round_to_i32((double)s_calib.matrix_dec_to_ra * ra_deg +
                                  (double)s_calib.matrix_dec_to_dec * dec_deg +
                                  (double)s_calib.offset_dec_arcsec);
    } else {
        *ra_steps = round_to_i32(ra_deg * STEPS_PER_DEGREE);
        *dec_steps = round_to_i32(dec_deg * STEPS_PER_DEGREE);
    }
}

static void steps_to_coord(int32_t ra_steps, int32_t dec_steps, os_equatorial_coord_t *coord) {
    if (s_calib.valid) {
        double det = (double)s_calib.matrix_ra_to_ra * (double)s_calib.matrix_dec_to_dec -
                     (double)s_calib.matrix_ra_to_dec * (double)s_calib.matrix_dec_to_ra;
        if (fabs(det) > 1e-12) {
            double rx = (double)ra_steps - (double)s_calib.offset_ra_arcsec;
            double ry = (double)dec_steps - (double)s_calib.offset_dec_arcsec;
            double ra_deg = ((double)s_calib.matrix_dec_to_dec * rx -
                             (double)s_calib.matrix_ra_to_dec * ry) / det;
            double dec_deg = (-(double)s_calib.matrix_dec_to_ra * rx +
                              (double)s_calib.matrix_ra_to_ra * ry) / det;
            coord->ra_hours = wrap_ra_hours((float)(ra_deg / 15.0));
            coord->dec_degrees = clamp_dec_degrees((float)dec_deg);
            return;
        }
    }

    coord->ra_hours = wrap_ra_hours((float)((double)ra_steps / STEPS_PER_DEGREE / 15.0));
    coord->dec_degrees = clamp_dec_degrees((float)((double)dec_steps / STEPS_PER_DEGREE));
}

static float current_pec_correction_rate_arcsec_per_sec(void) {
    if (!s_pec_enabled || !s_pec_table.valid) {
        return 0.0f;
    }
    int index = (int)s_worm_phase_deg % 360;
    if (index < 0) {
        index += 360;
    }
    int next = (index + 1) % 360;
    double err_current = (double)s_pec_table.corrections[index];
    double err_next = (double)s_pec_table.corrections[next];
    double derivative_arcsec_per_deg = err_next - err_current;
    double phase_rate_deg_per_sec = (double)OS_SIDEREAL_RATE_ARCSEC_PER_SEC / 3600.0;
    return (float)(derivative_arcsec_per_deg * phase_rate_deg_per_sec);
}

static float static_tracking_frequency_raw(void) {
    float factor = 1.0f;
    switch (s_track_rate) {
        case OS_TRACK_RATE_SIDEREAL:
            factor = 1.0f;
            break;
        case OS_TRACK_RATE_LUNAR:
            factor = OS_LUNAR_RATE_FACTOR;
            break;
        case OS_TRACK_RATE_SOLAR:
            factor = OS_SOLAR_RATE_FACTOR;
            break;
        case OS_TRACK_RATE_CUSTOM:
            factor = s_custom_track_factor;
            break;
        default:
            factor = 1.0f;
            break;
    }
    return OS_SIDEREAL_RATE_ARCSEC_PER_SEC * factor;
}

static uint32_t tracking_frequency_hz(void) {
    double base = (double)static_tracking_frequency_raw();
    double corr = (double)current_pec_correction_rate_arcsec_per_sec();
    return clamp_freq(base + corr);
}

static uint32_t manual_frequency_hz(os_speed_level_t speed) {
    uint32_t freq = MANUAL_FREQ_SLOW;
    switch (speed) {
        case OS_SPEED_SLOW:
            freq = MANUAL_FREQ_SLOW;
            break;
        case OS_SPEED_MEDIUM:
            freq = MANUAL_FREQ_MEDIUM;
            break;
        case OS_SPEED_FAST:
            freq = MANUAL_FREQ_FAST;
            break;
        case OS_SPEED_CUSTOM:
            freq = (uint32_t)s_custom_manual_arcsec_per_sec;
            break;
        default:
            freq = MANUAL_FREQ_SLOW;
            break;
    }
    if (freq > MAX_MOTOR_FREQ_HZ) {
        freq = MAX_MOTOR_FREQ_HZ;
    }
    return freq;
}

static bool equatorial_to_horizontal(os_equatorial_coord_t eq,
                                     const os_site_info_t *site,
                                     uint32_t epoch_seconds,
                                     os_horizontal_coord_t *horiz) {
    double lst = local_sidereal_hours(epoch_seconds, site->longitude_degrees);
    double ha_deg = (lst - (double)eq.ra_hours) * 15.0;
    double ha = ha_deg * M_PI / 180.0;
    double dec = (double)eq.dec_degrees * M_PI / 180.0;
    double lat = (double)site->latitude_degrees * M_PI / 180.0;

    double sin_alt = sin(dec) * sin(lat) + cos(dec) * cos(lat) * cos(ha);
    if (sin_alt > 1.0) {
        sin_alt = 1.0;
    }
    if (sin_alt < -1.0) {
        sin_alt = -1.0;
    }
    double alt = asin(sin_alt);
    double cos_alt = cos(alt);
    if (fabs(cos_alt) < 1e-12) {
        return false;
    }

    double az = atan2(-sin(ha) * cos(dec),
                      cos(dec) * sin(lat) - sin(dec) * cos(lat) * cos(ha));
    az = fmod(az + 2.0 * M_PI, 2.0 * M_PI);

    horiz->azimuth_degrees = (float)(az * 180.0 / M_PI);
    horiz->altitude_degrees = (float)(alt * 180.0 / M_PI);
    return true;
}

static void apply_tracking_altaz(void) {
    update_current_from_steps();
    os_horizontal_coord_t h0, h1;
    if (!s_site.valid ||
        !equatorial_to_horizontal(s_current_coord, &s_site, s_site.utc_epoch_seconds, &h0) ||
        !equatorial_to_horizontal(s_current_coord, &s_site, s_site.utc_epoch_seconds + 1U, &h1)) {
        stop_motors();
        os_hal_motor_enable(0, false);
        os_hal_motor_enable(1, false);
        return;
    }

    double az0 = (double)h0.azimuth_degrees;
    double alt0 = (double)h0.altitude_degrees;
    double az1 = (double)h1.azimuth_degrees;
    double alt1 = (double)h1.altitude_degrees;

    double delta_az = az1 - az0;
    while (delta_az > 180.0) {
        delta_az -= 360.0;
    }
    while (delta_az < -180.0) {
        delta_az += 360.0;
    }
    double delta_alt = alt1 - alt0;

    double freq_az = fabs(delta_az) * STEPS_PER_DEGREE;
    double freq_alt = fabs(delta_alt) * STEPS_PER_DEGREE;
    uint32_t f0 = clamp_freq(freq_az);
    uint32_t f1 = clamp_freq(freq_alt);

    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);
    os_hal_motor_set_direction(0, delta_az >= 0.0);
    os_hal_motor_set_direction(1, delta_alt >= 0.0);
    os_hal_motor_set_frequency(0, f0);
    os_hal_motor_set_frequency(1, f1);
}

static void apply_tracking(void) {
    if (!s_tracking_enabled) {
        stop_motors();
        return;
    }

    if (s_mount_type == OS_MOUNT_ALTAZ) {
        apply_tracking_altaz();
        return;
    }

    uint32_t freq = tracking_frequency_hz();
    os_hal_motor_set_direction(0, true);
    os_hal_motor_set_frequency(0, freq);
    os_hal_motor_set_frequency(1, 0U);
    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);
}

static void apply_manual_motion(void) {
    uint8_t axis = (s_move_direction == OS_DIRECTION_NORTH ||
                    s_move_direction == OS_DIRECTION_SOUTH) ? 1U : 0U;
    bool forward = (s_move_direction == OS_DIRECTION_NORTH ||
                    s_move_direction == OS_DIRECTION_EAST);
    uint32_t freq = manual_frequency_hz(s_move_speed);

    os_hal_motor_set_direction(axis, forward);
    os_hal_motor_set_frequency(axis, freq);
    os_hal_motor_set_frequency(axis == 0U ? 1U : 0U, 0U);
    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);
}

static void apply_guide_motion(void) {
    bool guide_is_dec = (s_guide_direction == OS_DIRECTION_NORTH ||
                         s_guide_direction == OS_DIRECTION_SOUTH);
    uint32_t base_freq = s_tracking_enabled ? tracking_frequency_hz() : 0U;
    uint32_t bias = clamp_freq((double)static_tracking_frequency_raw() *
                               (double)s_guide_pulse.rate_fraction);

    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);

    if (guide_is_dec) {
        os_hal_motor_set_direction(0, true);
        os_hal_motor_set_frequency(0, base_freq);
        os_hal_motor_set_direction(1, (s_guide_direction == OS_DIRECTION_NORTH));
        os_hal_motor_set_frequency(1, bias);
    } else {
        bool east = (s_guide_direction == OS_DIRECTION_EAST);
        uint32_t ra_freq;
        os_hal_motor_set_direction(0, true);
        if (east) {
            ra_freq = clamp_freq((double)base_freq + (double)bias);
        } else {
            if (base_freq > bias) {
                ra_freq = base_freq - bias;
            } else {
                ra_freq = 0U;
            }
        }
        os_hal_motor_set_frequency(0, ra_freq);
        os_hal_motor_set_frequency(1, 0U);
    }
}

static bool linear_fit_2param(const double x[], const double z[],
                              int n, double coeff[2]) {
    if (n < 2 || n > OS_CALIBRATION_MAX_STARS) {
        return false;
    }

    double A[OS_CALIBRATION_MAX_STARS][2];
    double Q[OS_CALIBRATION_MAX_STARS][2];
    double R[2][2];
    double b[OS_CALIBRATION_MAX_STARS];
    memset(R, 0, sizeof(R));

    for (int i = 0; i < n; ++i) {
        A[i][0] = x[i];
        A[i][1] = 1.0;
        b[i] = z[i];
        Q[i][0] = A[i][0];
        Q[i][1] = A[i][1];
    }

    for (int k = 0; k < 2; ++k) {
        double norm = 0.0;
        for (int i = 0; i < n; ++i) {
            norm += Q[i][k] * Q[i][k];
        }
        norm = sqrt(norm);
        if (norm < 1e-12) {
            return false;
        }

        R[k][k] = norm;
        for (int i = 0; i < n; ++i) {
            Q[i][k] /= norm;
        }

        for (int j = k + 1; j < 2; ++j) {
            R[k][j] = 0.0;
            for (int i = 0; i < n; ++i) {
                R[k][j] += Q[i][k] * A[i][j];
            }
            for (int i = 0; i < n; ++i) {
                A[i][j] -= R[k][j] * Q[i][k];
            }
        }
    }

    double c[2] = {0.0, 0.0};
    for (int k = 0; k < 2; ++k) {
        for (int i = 0; i < n; ++i) {
            c[k] += Q[i][k] * b[i];
        }
    }

    coeff[1] = c[1] / R[1][1];
    coeff[0] = (c[0] - R[0][1] * coeff[1]) / R[0][0];
    return true;
}

static bool linear_fit_3param(const double x[], const double y[], const double z[],
                              int n, double coeff[3]) {
    if (n < 3 || n > OS_CALIBRATION_MAX_STARS) {
        return false;
    }

    double A[OS_CALIBRATION_MAX_STARS][3];
    double Q[OS_CALIBRATION_MAX_STARS][3];
    double R[3][3];
    double b[OS_CALIBRATION_MAX_STARS];
    memset(R, 0, sizeof(R));

    for (int i = 0; i < n; ++i) {
        A[i][0] = x[i];
        A[i][1] = y[i];
        A[i][2] = 1.0;
        b[i] = z[i];
        Q[i][0] = A[i][0];
        Q[i][1] = A[i][1];
        Q[i][2] = A[i][2];
    }

    for (int k = 0; k < 3; ++k) {
        double norm = 0.0;
        for (int i = 0; i < n; ++i) {
            norm += Q[i][k] * Q[i][k];
        }
        norm = sqrt(norm);
        if (norm < 1e-12) {
            return false;
        }

        R[k][k] = norm;
        for (int i = 0; i < n; ++i) {
            Q[i][k] /= norm;
        }

        for (int j = k + 1; j < 3; ++j) {
            R[k][j] = 0.0;
            for (int i = 0; i < n; ++i) {
                R[k][j] += Q[i][k] * A[i][j];
            }
            for (int i = 0; i < n; ++i) {
                A[i][j] -= R[k][j] * Q[i][k];
            }
        }
    }

    double c[3] = {0.0, 0.0, 0.0};
    for (int k = 0; k < 3; ++k) {
        for (int i = 0; i < n; ++i) {
            c[k] += Q[i][k] * b[i];
        }
    }

    coeff[2] = c[2] / R[2][2];
    coeff[1] = (c[1] - R[1][2] * coeff[2]) / R[1][1];
    coeff[0] = (c[0] - R[0][1] * coeff[1] - R[0][2] * coeff[2]) / R[0][0];
    return true;
}

static bool compute_alignment(void) {
    double m00 = 0.0;
    double m01 = 0.0;
    double m10 = 0.0;
    double m11 = 0.0;
    double b0 = 0.0;
    double b1 = 0.0;
    double residual = 0.0;
    int n = s_align_count;

    if (s_align_mode == OS_ALIGN_1STAR) {
        m00 = STEPS_PER_DEGREE;
        m11 = STEPS_PER_DEGREE;
        m01 = 0.0;
        m10 = 0.0;

        for (int i = 0; i < n; ++i) {
            b0 += s_align_ra_steps[i] - m00 * s_align_x[i];
            b1 += s_align_dec_steps[i] - m11 * s_align_y[i];
        }
        b0 /= (double)n;
        b1 /= (double)n;

        double sum = 0.0;
        for (int i = 0; i < n; ++i) {
            double ra_pred = m00 * s_align_x[i] + b0;
            double dec_pred = m11 * s_align_y[i] + b1;
            double ra_err = s_align_ra_steps[i] - ra_pred;
            double dec_err = s_align_dec_steps[i] - dec_pred;
            sum += ra_err * ra_err + dec_err * dec_err;
        }
        residual = sqrt(sum / (double)n);
    } else if (s_align_mode == OS_ALIGN_2STAR) {
        double ra_coeff[2] = {0.0, 0.0};
        double dec_coeff[2] = {0.0, 0.0};

        if (!linear_fit_2param(s_align_x, s_align_ra_steps, n, ra_coeff) ||
            !linear_fit_2param(s_align_y, s_align_dec_steps, n, dec_coeff)) {
            return false;
        }

        m00 = ra_coeff[0];
        b0 = ra_coeff[1];
        m11 = dec_coeff[0];
        b1 = dec_coeff[1];
        m01 = 0.0;
        m10 = 0.0;

        double sum = 0.0;
        for (int i = 0; i < n; ++i) {
            double ra_pred = m00 * s_align_x[i] + b0;
            double dec_pred = m11 * s_align_y[i] + b1;
            double ra_err = s_align_ra_steps[i] - ra_pred;
            double dec_err = s_align_dec_steps[i] - dec_pred;
            sum += ra_err * ra_err + dec_err * dec_err;
        }
        residual = sqrt(sum / (double)n);
    } else {
        if (n == 3) {
            double det = (s_align_x[1] - s_align_x[0]) * (s_align_y[2] - s_align_y[0]) -
                         (s_align_x[2] - s_align_x[0]) * (s_align_y[1] - s_align_y[0]);
            if (fabs(det) < 1e-9) {
                return false;
            }
        }

        double ra_coeff[3] = {0.0, 0.0, 0.0};
        double dec_coeff[3] = {0.0, 0.0, 0.0};
        if (!linear_fit_3param(s_align_x, s_align_y, s_align_ra_steps, n, ra_coeff) ||
            !linear_fit_3param(s_align_x, s_align_y, s_align_dec_steps, n, dec_coeff)) {
            return false;
        }

        m00 = ra_coeff[0];
        m01 = ra_coeff[1];
        b0 = ra_coeff[2];
        m10 = dec_coeff[0];
        m11 = dec_coeff[1];
        b1 = dec_coeff[2];

        if (n == 3) {
            residual = 0.0;
        } else {
            double sum = 0.0;
            for (int i = 0; i < n; ++i) {
                double ra_pred = m00 * s_align_x[i] + m01 * s_align_y[i] + b0;
                double dec_pred = m10 * s_align_x[i] + m11 * s_align_y[i] + b1;
                double ra_err = s_align_ra_steps[i] - ra_pred;
                double dec_err = s_align_dec_steps[i] - dec_pred;
                sum += ra_err * ra_err + dec_err * dec_err;
            }
            residual = sqrt(sum / (double)n);
        }
    }

    s_calib.matrix_ra_to_ra = (float)m00;
    s_calib.matrix_ra_to_dec = (float)m01;
    s_calib.matrix_dec_to_ra = (float)m10;
    s_calib.matrix_dec_to_dec = (float)m11;
    s_calib.offset_ra_arcsec = (float)b0;
    s_calib.offset_dec_arcsec = (float)b1;
    s_calib.valid = true;
    s_residual_arcsec = (float)residual;
    s_alignment_residual_computed = true;

    return true;
}

static void update_current_from_steps(void) {
    int32_t ra_steps = os_hal_motor_get_position(0);
    int32_t dec_steps = os_hal_motor_get_position(1);
    steps_to_coord(ra_steps, dec_steps, &s_current_coord);
}

static void begin_async_move(int32_t target_ra, int32_t target_dec) {
    s_goto_target_ra = target_ra;
    s_goto_target_dec = target_dec;
    s_goto_active = true;
    s_park_active = false;
    s_manual_moving = false;
    s_moving = true;
    s_goto_axis_freq[0] = 0U;
    s_goto_axis_freq[1] = 0U;
    s_guide_pulse.active = false;
    s_state = OS_STATE_GOTO;
}

static void begin_park_async(int32_t target_ra, int32_t target_dec) {
    s_goto_target_ra = target_ra;
    s_goto_target_dec = target_dec;
    s_goto_active = false;
    s_park_active = true;
    s_manual_moving = false;
    s_moving = true;
    s_goto_axis_freq[0] = 0U;
    s_goto_axis_freq[1] = 0U;
    s_guide_pulse.active = false;
    s_state = OS_STATE_GOTO;
}

static void advance_goto_or_park(void) {
    int32_t cur_ra = os_hal_motor_get_position(0);
    int32_t cur_dec = os_hal_motor_get_position(1);
    int32_t diff_ra = s_goto_target_ra - cur_ra;
    int32_t diff_dec = s_goto_target_dec - cur_dec;

    if (diff_ra == 0 && diff_dec == 0) {
        stop_motors();
        s_moving = false;
        if (s_park_active) {
            s_park_active = false;
            s_goto_active = false;
            s_tracking_enabled = false;
            s_state = OS_STATE_PARKED;
            os_hal_motor_enable(0, false);
            os_hal_motor_enable(1, false);
        } else {
            s_goto_active = false;
            s_park_active = false;
            s_tracking_enabled = true;
            s_state = OS_STATE_IDLE_TRACKING;
            os_hal_buzzer_beep(200U, 1U);
        }
        update_current_from_steps();
        return;
    }

    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);

    for (uint8_t axis = 0U; axis < 2U; ++axis) {
        int32_t diff = (axis == 0U) ? diff_ra : diff_dec;
        if (diff == 0) {
            os_hal_motor_set_frequency(axis, 0U);
            s_goto_axis_freq[axis] = 0U;
            continue;
        }

        uint32_t dist = (diff > 0) ? (uint32_t)diff : (uint32_t)(-diff);
        if (s_goto_axis_freq[axis] == 0U) {
            s_goto_axis_freq[axis] = GOTO_START_FREQ_HZ;
        }

        uint32_t freq;
        if (dist <= GOTO_DECEL_DIST_STEPS) {
            uint32_t desired = (uint32_t)(((uint64_t)GOTO_FREQ_HZ * dist) / GOTO_DECEL_DIST_STEPS);
            if (desired < GOTO_START_FREQ_HZ) {
                desired = GOTO_START_FREQ_HZ;
            }
            if (s_goto_axis_freq[axis] > desired) {
                s_goto_axis_freq[axis] = desired;
            }
            freq = s_goto_axis_freq[axis];
        } else if (s_goto_axis_freq[axis] < GOTO_FREQ_HZ) {
            uint32_t next = s_goto_axis_freq[axis] + GOTO_ACCEL_STEP_HZ;
            if (next > GOTO_FREQ_HZ) {
                next = GOTO_FREQ_HZ;
            }
            s_goto_axis_freq[axis] = next;
            freq = next;
        } else {
            freq = GOTO_FREQ_HZ;
        }

        os_hal_motor_set_direction(axis, diff > 0);
        os_hal_motor_set_frequency(axis, freq);
    }
}

static void write_reply_copy(const char *src, char *reply_buffer,
                             size_t reply_buffer_size, size_t *reply_length) {
    size_t n = strlen(src);
    if (reply_buffer_size == 0U) {
        *reply_length = 0U;
        return;
    }

    if (n >= reply_buffer_size) {
        n = reply_buffer_size - 1U;
    }

    memcpy(reply_buffer, src, n);
    reply_buffer[n] = 0;
    *reply_length = n;
}

static void format_ra_reply(float ra_hours, char *out, size_t out_size) {
    double total_seconds = (double)ra_hours * 3600.0;
    while (total_seconds < 0.0) {
        total_seconds += 86400.0;
    }
    while (total_seconds >= 86400.0) {
        total_seconds -= 86400.0;
    }

    int hours = (int)(total_seconds / 3600.0);
    int minutes = (int)((total_seconds - (double)hours * 3600.0) / 60.0);
    double seconds = total_seconds - (double)hours * 3600.0 - (double)minutes * 60.0;
    snprintf(out, out_size, "%02d:%02d:%04.1f#", hours, minutes, seconds);
}

static void format_dec_reply(float dec_degrees, char *out, size_t out_size) {
    char sign = '+';
    if (dec_degrees < 0.0f) {
        sign = '-';
        dec_degrees = -dec_degrees;
    }

    int degrees = (int)dec_degrees;
    double minutes_full = ((double)dec_degrees - (double)degrees) * 60.0;
    int minutes = (int)minutes_full;
    double seconds = (minutes_full - (double)minutes) * 60.0;
    snprintf(out, out_size, "%c%02d*%02d:%04.1f#", sign, degrees, minutes, seconds);
}

static bool parse_ra_hours(const char *s, float *hours) {
    if (s == NULL || *s == 0) {
        return false;
    }
    int h = 0;
    int m = 0;
    double sec = 0.0;
    if (sscanf(s, "%d:%d:%lf", &h, &m, &sec) != 3) {
        return false;
    }
    if (h < 0 || h >= 24 || m < 0 || m >= 60 || sec < 0.0 || sec >= 60.0) {
        return false;
    }
    *hours = (float)((double)h + (double)m / 60.0 + sec / 3600.0);
    return true;
}

static bool parse_dec_degrees(const char *s, float *degrees) {
    if (s == NULL || *s == 0) {
        return false;
    }
    char sign = 0;
    int d = 0;
    int m = 0;
    double sec = 0.0;
    if (sscanf(s, "%c%d%*c%d:%lf", &sign, &d, &m, &sec) != 4) {
        return false;
    }
    if (sign != '+' && sign != '-') {
        return false;
    }
    if (d < 0 || d > 90 || m < 0 || m >= 60 || sec < 0.0 || sec >= 60.0) {
        return false;
    }
    double value = (double)d + (double)m / 60.0 + sec / 3600.0;
    if (sign == '-') {
        value = -value;
    }
    if (value < OS_DEC_MIN_DEG || value > OS_DEC_MAX_DEG) {
        return false;
    }
    *degrees = (float)value;
    return true;
}

static uint32_t parse_guide_duration_ms(const char *s) {
    if (s == NULL || *s == 0) {
        return 100U;
    }
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s || *end != 0 || v == 0U) {
        return 100U;
    }
    return (uint32_t)v;
}

static void handle_received_command(uint8_t channel) {
    if (s_rx_len[channel] == 0U) {
        return;
    }

    s_rx[channel][s_rx_len[channel]] = 0;
    char reply[OS_MAX_REPLY_LENGTH + 1];
    size_t reply_len = 0U;

    os_error_t err = os_command_parse(s_rx[channel], s_rx_len[channel], channel,
                                      reply, sizeof(reply), &reply_len);
    if (err == OS_ERR_NONE && reply_len > 0U) {
        os_hal_comm_write(channel, reply, reply_len);
    }

    s_rx_len[channel] = 0U;
}

static void poll_channels(void) {
    for (uint8_t ch = 0U; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        int16_t avail = os_hal_comm_available(ch);
        if (avail <= 0) {
            continue;
        }

        while (avail-- > 0) {
            char c = os_hal_comm_read(ch);
            if (c == 10 || c == 13) {
                handle_received_command(ch);
                if (c == 10) {
                    break;
                }
            } else {
                if (s_rx_len[ch] >= (uint8_t)(RX_BUFFER_SIZE - 1U)) {
                    s_rx_len[ch] = 0U;
                } else {
                    s_rx[ch][s_rx_len[ch]++] = c;
                }
            }
        }
    }
}

static void update_site_and_lock_state(void) {
    os_site_info_t gps_site;
    memset(&gps_site, 0, sizeof(gps_site));

    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid) {
        s_gps_locked = true;
        s_site = gps_site;
        if (gps_site.utc_epoch_seconds != 0U) {
            os_hal_rtc_set(gps_site.utc_epoch_seconds);
        }
    } else {
        s_gps_locked = false;
        uint32_t rtc = 0U;
        if (os_hal_rtc_read(&rtc) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = rtc;
            s_site.valid = true;
        } else {
            s_site.valid = false;
        }
    }
}

static void check_limit_safety(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        stop_motors();
        os_hal_motor_enable(0, false);
        os_hal_motor_enable(1, false);
        s_moving = false;
        s_goto_active = false;
        s_park_active = false;
        s_manual_moving = false;
        s_guide_pulse.active = false;
        s_state = OS_STATE_FAULT;
    }
}

static double julian_days_since_j2000(uint32_t epoch_seconds) {
    return ((double)epoch_seconds / 86400.0) - 10957.5;
}

static double gmst_hours(uint32_t epoch_seconds) {
    double days = julian_days_since_j2000(epoch_seconds);
    double deg = fmod(280.46061837 + 360.98564736629 * days, 360.0);
    if (deg < 0.0) {
        deg += 360.0;
    }
    return deg / 15.0;
}

static double local_sidereal_hours(uint32_t epoch_seconds, float longitude_degrees) {
    double lst = gmst_hours(epoch_seconds) + (double)longitude_degrees / 15.0;
    return wrap_ra_hours((float)lst);
}

static bool horizontal_to_equatorial(os_horizontal_coord_t horizontal,
                                     os_equatorial_coord_t *equatorial) {
    if (!s_site.valid || s_site.utc_epoch_seconds == 0U) {
        return false;
    }

    double lat = (double)s_site.latitude_degrees * M_PI / 180.0;
    double alt = (double)horizontal.altitude_degrees * M_PI / 180.0;
    double az = (double)horizontal.azimuth_degrees * M_PI / 180.0;

    double sin_dec = sin(alt) * sin(lat) + cos(alt) * cos(lat) * cos(az);
    if (sin_dec > 1.0) {
        sin_dec = 1.0;
    }
    if (sin_dec < -1.0) {
        sin_dec = -1.0;
    }
    double dec = asin(sin_dec);
    double cos_dec = cos(dec);
    if (fabs(cos_dec) < 1e-12) {
        return false;
    }

    double ha = atan2(-sin(az) * cos(alt),
                      cos(alt) * sin(lat) - sin(alt) * cos(lat) * cos(az));
    double lst = local_sidereal_hours(s_site.utc_epoch_seconds, s_site.longitude_degrees);
    double ra_hours = lst - (ha * 12.0 / M_PI);

    equatorial->ra_hours = wrap_ra_hours((float)ra_hours);
    equatorial->dec_degrees = (float)(dec * 180.0 / M_PI);
    return valid_equatorial_coord(*equatorial);
}

os_error_t os_init(void) {
    s_state = OS_STATE_INITIALIZING;
    s_mount_type = OS_CONFIG_MOUNT_TYPE;
    s_tracking_enabled = true;
    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_custom_track_factor = 1.0f;
    s_gps_locked = false;
    s_alignment_active = false;
    s_alignment_residual_computed = false;
    s_residual_arcsec = 0.0f;
    s_align_count = 0;
    s_goto_active = false;
    s_park_active = false;
    s_manual_moving = false;
    s_moving = false;
    s_goto_axis_freq[0] = 0U;
    s_goto_axis_freq[1] = 0U;
    s_guide_pulse.active = false;
    s_guide_pulse.duration_ms = 0U;
    s_guide_pulse.rate_fraction = 0.5f;
    s_guide_pulse.direction_east = false;
    s_guide_pulse.direction_north = false;
    s_guide_pulse.dec_priority = false;
    s_guide_start_rtc_seconds = 0U;
    s_guide_start_valid = false;
    s_pec_enabled = false;
    s_worm_phase_deg = 0.0f;
    s_last_rtc_seconds = 0U;
    s_last_rtc_valid = false;
    memset(&s_pec_table, 0, sizeof(s_pec_table));
    memset(&s_calib, 0, sizeof(s_calib));
    s_pending_ra_set = false;
    s_pending_dec_set = false;
    s_pending_ra_hours = 0.0f;
    s_pending_dec_degrees = 0.0f;

    memset(s_rx, 0, sizeof(s_rx));
    memset(s_rx_len, 0, sizeof(s_rx_len));

    bool nvm_ok = (os_hal_nvm_init() == OS_ERR_NONE);
    if (nvm_ok) {
        uint8_t calib_buf[sizeof(os_calibration_t)];
        memset(calib_buf, 0, sizeof(calib_buf));
        if (os_hal_nvm_read(NVM_CALIB_OFFSET, calib_buf, (uint16_t)sizeof(calib_buf)) == OS_ERR_NONE) {
            os_calibration_t loaded;
            memcpy(&loaded, calib_buf, sizeof(loaded));
            if (loaded.valid) {
                s_calib = loaded;
            }
        }

        uint8_t cfg_byte = 0U;
        if (os_hal_nvm_read(NVM_CONFIG_OFFSET, &cfg_byte, 1U) == OS_ERR_NONE) {
            if (cfg_byte == 0U || cfg_byte == 1U) {
                s_pec_enabled = (cfg_byte == 1U);
            }
        }

        os_pec_table_t loaded_pec;
        memset(&loaded_pec, 0, sizeof(loaded_pec));
        if (os_hal_nvm_read(NVM_PEC_OFFSET, (uint8_t *)&loaded_pec, (uint16_t)sizeof(loaded_pec)) == OS_ERR_NONE) {
            if (loaded_pec.valid) {
                s_pec_table = loaded_pec;
            }
        }
    }

    os_hal_comm_init(OS_CHANNEL_USB);
    os_hal_comm_init(OS_CHANNEL_BLUETOOTH);
    os_hal_comm_init(OS_CHANNEL_WIFI);
    os_hal_comm_init(OS_CHANNEL_ETHERNET);

    if (os_hal_motor_init(0) != OS_ERR_NONE ||
        os_hal_motor_init(1) != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    os_hal_motor_enable(0, false);
    os_hal_motor_enable(1, false);
    stop_motors();

    os_hal_gps_init();
    os_hal_rtc_init();
    os_hal_limit_init();
    if (os_hal_timer_motor_init() != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    s_site.valid = false;
    os_site_info_t gps_site;
    memset(&gps_site, 0, sizeof(gps_site));
    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid) {
        s_site = gps_site;
        s_gps_locked = true;
        if (gps_site.utc_epoch_seconds != 0U) {
            os_hal_rtc_set(gps_site.utc_epoch_seconds);
        }
    } else {
        uint32_t rtc = 0U;
        if (os_hal_rtc_read(&rtc) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = rtc;
            s_site.latitude_degrees = 0.0f;
            s_site.longitude_degrees = 0.0f;
            s_site.elevation_metres = 0.0f;
            s_site.valid = true;
        } else {
            s_state = OS_STATE_FAULT;
            stop_motors();
            os_hal_motor_enable(0, false);
            os_hal_motor_enable(1, false);
            return OS_ERR_GPS_NO_SIGNAL;
        }
    }

    uint32_t now_sec = 0U;
    if (os_hal_rtc_read(&now_sec) == OS_ERR_NONE) {
        s_last_rtc_seconds = now_sec;
        s_last_rtc_valid = true;
    } else if (s_site.valid) {
        s_last_rtc_seconds = s_site.utc_epoch_seconds;
        s_last_rtc_valid = true;
    }

    update_current_from_steps();
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    poll_channels();
    update_site_and_lock_state();
    check_limit_safety();

    if (s_state == OS_STATE_FAULT) {
        stop_motors();
        os_hal_motor_enable(0, false);
        os_hal_motor_enable(1, false);
        update_current_from_steps();
        return;
    }

    uint32_t now_sec = 0U;
    bool now_valid = false;
    if (os_hal_rtc_read(&now_sec) == OS_ERR_NONE) {
        now_valid = true;
    } else if (s_site.valid) {
        now_sec = s_site.utc_epoch_seconds;
        now_valid = true;
    }

    if (now_valid && s_last_rtc_valid) {
        uint32_t elapsed = (now_sec >= s_last_rtc_seconds) ? (now_sec - s_last_rtc_seconds) : 0U;
        if (elapsed > 0U && elapsed < 3600U) {
            s_worm_phase_deg += (float)(360.0 * (double)elapsed / SIDEREAL_DAY_SECONDS);
            if (s_worm_phase_deg >= 360.0f) {
                s_worm_phase_deg -= 360.0f;
            }
        }
    }
    if (now_valid) {
        s_last_rtc_seconds = now_sec;
        s_last_rtc_valid = true;
    }

    if (s_guide_pulse.active) {
        if (s_guide_start_valid && now_valid) {
            uint32_t elapsed_ms = (now_sec >= s_guide_start_rtc_seconds) ? (now_sec - s_guide_start_rtc_seconds) * 1000U : 0U;
            if (elapsed_ms >= s_guide_pulse.duration_ms) {
                s_guide_pulse.active = false;
            }
        }
    }

    update_current_from_steps();

    if (s_state == OS_STATE_GOTO) {
        advance_goto_or_park();
    } else if (s_state == OS_STATE_MANUAL_MOTION) {
        apply_manual_motion();
    } else if (s_state == OS_STATE_PARKED) {
        stop_motors();
        os_hal_motor_enable(0, false);
        os_hal_motor_enable(1, false);
    } else if (s_state == OS_STATE_IDLE_TRACKING || s_state == OS_STATE_ALIGNMENT) {
        if (s_guide_pulse.active) {
            apply_guide_motion();
        } else {
            apply_tracking();
        }
    }

    update_current_from_steps();
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (reply_buffer_size == 0U) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!valid_channel(source_channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    reply_buffer[0] = 0;
    *reply_length = 0U;

    if (length == 0U || length > OS_MAX_COMMAND_LENGTH + 2U) {
        return OS_ERR_COMMAND_FORMAT;
    }

    char cmd[OS_MAX_COMMAND_LENGTH + 2U];
    memset(cmd, 0, sizeof(cmd));
    memcpy(cmd, command, length);
    cmd[length] = 0;

    if (length < 3U || cmd[0] != OS_LX200_CMD_PREFIX || cmd[length - 1U] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    cmd[length - 1U] = 0;
    const char *op = cmd + 1;
    char tmp[OS_MAX_REPLY_LENGTH + 1];
    os_error_t err = OS_ERR_NONE;

    if (strcmp(op, "GR") == 0) {
        os_equatorial_coord_t coord;
        err = os_query_coordinates(&coord);
        if (err != OS_ERR_NONE) {
            return err;
        }
        format_ra_reply(coord.ra_hours, tmp, sizeof(tmp));
        write_reply_copy(tmp, reply_buffer, reply_buffer_size, reply_length);
        return OS_ERR_NONE;
    } else if (strcmp(op, "GD") == 0) {
        os_equatorial_coord_t coord;
        err = os_query_coordinates(&coord);
        if (err != OS_ERR_NONE) {
            return err;
        }
        format_dec_reply(coord.dec_degrees, tmp, sizeof(tmp));
        write_reply_copy(tmp, reply_buffer, reply_buffer_size, reply_length);
        return OS_ERR_NONE;
    } else if (strcmp(op, "GVP") == 0 || strcmp(op, "GVN") == 0) {
        snprintf(tmp, sizeof(tmp), "%d.%d.%d#",
                 OS_FIRMWARE_VERSION_MAJOR,
                 OS_FIRMWARE_VERSION_MINOR,
                 OS_FIRMWARE_VERSION_PATCH);
        write_reply_copy(tmp, reply_buffer, reply_buffer_size, reply_length);
        return OS_ERR_NONE;
    } else if (strcmp(op, "Me") == 0) {
        err = os_move_start(OS_DIRECTION_EAST, OS_SPEED_SLOW);
    } else if (strcmp(op, "Mw") == 0) {
        err = os_move_start(OS_DIRECTION_WEST, OS_SPEED_SLOW);
    } else if (strcmp(op, "Mn") == 0) {
        err = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_SLOW);
    } else if (strcmp(op, "Ms") == 0) {
        err = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_SLOW);
    } else if (strcmp(op, "Q") == 0) {
        err = os_move_stop();
    } else if (strcmp(op, "hP") == 0) {
        err = os_park();
    } else if (strcmp(op, "hO") == 0) {
        err = os_unpark();
    } else if (strcmp(op, "MS") == 0) {
        if (!s_pending_ra_set || !s_pending_dec_set) {
            return OS_ERR_INVALID_STATE;
        }
        os_equatorial_coord_t target = {s_pending_ra_hours, s_pending_dec_degrees};
        err = os_goto_equatorial(target);
        if (err == OS_ERR_NONE) {
            s_pending_ra_set = false;
            s_pending_dec_set = false;
        }
    } else if (strncmp(op, "Sr", 2) == 0) {
        float ra_hours = 0.0f;
        if (!parse_ra_hours(op + 2, &ra_hours)) {
            return OS_ERR_COMMAND_FORMAT;
        }
        s_pending_ra_hours = ra_hours;
        s_pending_ra_set = true;
        if (s_pending_dec_set) {
            os_equatorial_coord_t target = {s_pending_ra_hours, s_pending_dec_degrees};
            err = os_goto_equatorial(target);
            if (err == OS_ERR_NONE) {
                s_pending_ra_set = false;
                s_pending_dec_set = false;
            }
        }
    } else if (strncmp(op, "Sd", 2) == 0) {
        float dec_degrees = 0.0f;
        if (!parse_dec_degrees(op + 2, &dec_degrees)) {
            return OS_ERR_COMMAND_FORMAT;
        }
        s_pending_dec_degrees = dec_degrees;
        s_pending_dec_set = true;
        if (s_pending_ra_set) {
            os_equatorial_coord_t target = {s_pending_ra_hours, s_pending_dec_degrees};
            err = os_goto_equatorial(target);
            if (err == OS_ERR_NONE) {
                s_pending_ra_set = false;
                s_pending_dec_set = false;
            }
        }
    } else if (strncmp(op, "MgE", 3) == 0 || strncmp(op, "MgW", 3) == 0 ||
               strncmp(op, "MgN", 3) == 0 || strncmp(op, "MgS", 3) == 0) {
        os_direction_t direction = OS_DIRECTION_NORTH;
        if (strncmp(op, "MgE", 3) == 0) {
            direction = OS_DIRECTION_EAST;
        } else if (strncmp(op, "MgW", 3) == 0) {
            direction = OS_DIRECTION_WEST;
        } else if (strncmp(op, "MgN", 3) == 0) {
            direction = OS_DIRECTION_NORTH;
        } else if (strncmp(op, "MgS", 3) == 0) {
            direction = OS_DIRECTION_SOUTH;
        }
        uint32_t duration_ms = parse_guide_duration_ms(op + 3);
        err = os_guide_pulse(direction, duration_ms);
    } else if (strcmp(op, "TS") == 0) {
        err = os_tracking_set_rate(OS_TRACK_RATE_SIDEREAL, 1.0f);
    } else if (strcmp(op, "TL") == 0) {
        err = os_tracking_set_rate(OS_TRACK_RATE_LUNAR, 1.0f);
    } else if (strcmp(op, "TO") == 0) {
        err = os_tracking_set_rate(OS_TRACK_RATE_SOLAR, 1.0f);
    } else if (strncmp(op, "TC", 2) == 0) {
        char *end = NULL;
        float factor = strtof(op + 2, &end);
        if (end == op + 2 || *end != 0) {
            return OS_ERR_COMMAND_FORMAT;
        }
        err = os_tracking_set_rate(OS_TRACK_RATE_CUSTOM, factor);
    } else if (strcmp(op, "Te") == 0) {
        err = os_tracking_enable();
    } else if (strcmp(op, "Td") == 0) {
        err = os_tracking_disable();
    } else if (strcmp(op, "AA") == 0) {
        err = os_align_begin(OS_ALIGN_3STAR);
    } else if (strcmp(op, "AS") == 0) {
        os_equatorial_coord_t coord;
        os_motor_position_t pos;
        err = os_query_coordinates(&coord);
        if (err == OS_ERR_NONE) {
            err = os_query_motor_position(&pos);
        }
        if (err == OS_ERR_NONE) {
            err = os_align_accept_star(coord, pos);
        }
    } else if (strcmp(op, "AC") == 0) {
        err = os_align_compute();
    } else if (strcmp(op, "AB") == 0) {
        err = os_align_abort();
    } else {
        return OS_ERR_NOT_SUPPORTED;
    }

    if (err != OS_ERR_NONE) {
        return err;
    }
    write_reply_copy("1#", reply_buffer, reply_buffer_size, reply_length);
    return OS_ERR_NONE;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (!valid_equatorial_coord(target)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    int32_t target_ra;
    int32_t target_dec;
    coord_to_steps(target, &target_ra, &target_dec);

    int32_t cur_ra = os_hal_motor_get_position(0);
    int32_t cur_dec = os_hal_motor_get_position(1);

    if (os_hal_limit_is_triggered(0) && target_ra != cur_ra) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (os_hal_limit_is_triggered(1) && target_dec != cur_dec) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    if (target_ra == cur_ra && target_dec == cur_dec) {
        os_hal_buzzer_beep(200U, 1U);
        return OS_ERR_NONE;
    }

    begin_async_move(target_ra, target_dec);
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (!valid_horizontal_coord(target)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    int32_t target_axis0;
    int32_t target_axis1;

    if (s_mount_type == OS_MOUNT_ALTAZ) {
        target_axis0 = round_to_i32((double)target.azimuth_degrees * STEPS_PER_DEGREE);
        target_axis1 = round_to_i32((double)target.altitude_degrees * STEPS_PER_DEGREE);
    } else {
        os_equatorial_coord_t eq;
        if (!horizontal_to_equatorial(target, &eq)) {
            return OS_ERR_INVALID_STATE;
        }
        coord_to_steps(eq, &target_axis0, &target_axis1);
    }

    int32_t cur_axis0 = os_hal_motor_get_position(0);
    int32_t cur_axis1 = os_hal_motor_get_position(1);

    if (os_hal_limit_is_triggered(0) && target_axis0 != cur_axis0) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (os_hal_limit_is_triggered(1) && target_axis1 != cur_axis1) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    if (target_axis0 == cur_axis0 && target_axis1 == cur_axis1) {
        os_hal_buzzer_beep(200U, 1U);
        return OS_ERR_NONE;
    }

    begin_async_move(target_axis0, target_axis1);
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    stop_motors();
    s_goto_active = false;
    s_park_active = false;
    s_manual_moving = false;
    s_moving = false;
    s_goto_axis_freq[0] = 0U;
    s_goto_axis_freq[1] = 0U;
    if (s_state == OS_STATE_GOTO) {
        s_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (!valid_rate(rate)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (rate == OS_TRACK_RATE_CUSTOM) {
        if (custom_factor <= 0.0f) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        float max_factor = (float)MAX_MOTOR_FREQ_HZ / OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
        if (custom_factor > max_factor) {
            custom_factor = max_factor;
        }
    }

    s_track_rate = rate;
    if (rate == OS_TRACK_RATE_CUSTOM) {
        s_custom_track_factor = custom_factor;
    }
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
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    s_tracking_enabled = false;
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (!valid_direction(direction)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0U) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_guide_direction = direction;
    s_guide_pulse.active = true;
    s_guide_pulse.duration_ms = duration_ms;
    s_guide_pulse.rate_fraction = s_guide_rate;
    s_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    s_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    s_guide_pulse.dec_priority = (direction == OS_DIRECTION_NORTH ||
                                  direction == OS_DIRECTION_SOUTH);

    uint32_t rtc_sec = 0U;
    if (os_hal_rtc_read(&rtc_sec) == OS_ERR_NONE) {
        s_guide_start_rtc_seconds = rtc_sec;
        s_guide_start_valid = true;
    } else if (s_site.valid) {
        s_guide_start_rtc_seconds = s_site.utc_epoch_seconds;
        s_guide_start_valid = true;
    } else {
        s_guide_start_valid = false;
    }

    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_guide_rate = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (pulse == NULL) {
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
    s_align_count = 0;
    s_alignment_active = true;
    s_alignment_residual_computed = false;
    s_residual_arcsec = 0.0f;
    s_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (!valid_equatorial_coord(star_coord)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_alignment_active || s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_align_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    s_align_x[s_align_count] = (double)star_coord.ra_hours * 15.0;
    s_align_y[s_align_count] = (double)star_coord.dec_degrees;
    s_align_ra_steps[s_align_count] = (double)motor_pos.ra_steps;
    s_align_dec_steps[s_align_count] = (double)motor_pos.dec_steps;
    ++s_align_count;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (!s_alignment_active || s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    if (s_align_mode == OS_ALIGN_1STAR && s_align_count < 1) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_align_mode == OS_ALIGN_2STAR && s_align_count < 2) {
        return OS_ERR_INVALID_STATE;
    }
    if ((s_align_mode == OS_ALIGN_3STAR || s_align_mode == OS_ALIGN_NSTAR) &&
        s_align_count < 3) {
        return OS_ERR_INVALID_STATE;
    }

    if (!compute_alignment()) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    if (!(s_align_count == 3) &&
        s_align_count >= 4 &&
        s_residual_arcsec > CALIBRATION_RESIDUAL_LIMIT_ARCSEC) {
        memset(&s_calib, 0, sizeof(s_calib));
        s_calib.valid = false;
        s_alignment_residual_computed = false;
        s_residual_arcsec = 0.0f;
        return OS_ERR_CALIBRATION_FAILED;
    }

    s_alignment_active = false;
    s_state = OS_STATE_IDLE_TRACKING;

    if (os_hal_nvm_write(NVM_CALIB_OFFSET, (const uint8_t *)&s_calib,
                         (uint16_t)sizeof(s_calib)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }

    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_alignment_residual_computed) {
        return OS_ERR_INVALID_STATE;
    }

    *residual_arcsec = s_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    s_alignment_active = false;
    s_align_count = 0;
    s_alignment_residual_computed = false;
    s_residual_arcsec = 0.0f;
    if (s_state == OS_STATE_ALIGNMENT) {
        s_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    int32_t target_ra;
    int32_t target_dec;
    coord_to_steps(s_park_coord, &target_ra, &target_dec);

    int32_t cur_ra = os_hal_motor_get_position(0);
    int32_t cur_dec = os_hal_motor_get_position(1);

    if (os_hal_limit_is_triggered(0) && target_ra != cur_ra) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (os_hal_limit_is_triggered(1) && target_dec != cur_dec) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    if (target_ra == cur_ra && target_dec == cur_dec) {
        stop_motors();
        os_hal_motor_enable(0, false);
        os_hal_motor_enable(1, false);
        s_tracking_enabled = false;
        s_moving = false;
        s_goto_active = false;
        s_park_active = false;
        s_state = OS_STATE_PARKED;
        return OS_ERR_NONE;
    }

    begin_park_async(target_ra, target_dec);
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);
    os_hal_comm_init(OS_CHANNEL_USB);
    os_hal_comm_init(OS_CHANNEL_BLUETOOTH);
    os_hal_comm_init(OS_CHANNEL_WIFI);
    os_hal_comm_init(OS_CHANNEL_ETHERNET);

    os_site_info_t gps_site;
    memset(&gps_site, 0, sizeof(gps_site));
    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid) {
        s_site = gps_site;
        s_gps_locked = true;
        if (gps_site.utc_epoch_seconds != 0U) {
            os_hal_rtc_set(gps_site.utc_epoch_seconds);
        }
    } else {
        s_gps_locked = false;
        uint32_t rtc = 0U;
        if (os_hal_rtc_read(&rtc) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = rtc;
            s_site.valid = true;
        } else {
            s_site.valid = false;
        }
    }

    s_tracking_enabled = true;
    s_moving = false;
    s_goto_active = false;
    s_park_active = false;
    s_manual_moving = false;
    s_goto_axis_freq[0] = 0U;
    s_goto_axis_freq[1] = 0U;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!valid_equatorial_coord(park_pos)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_park_coord = park_pos;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (!valid_direction(direction)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!valid_speed(speed)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    uint8_t axis = (direction == OS_DIRECTION_NORTH ||
                    direction == OS_DIRECTION_SOUTH) ? 1U : 0U;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_move_direction = direction;
    s_move_speed = speed;
    s_manual_moving = true;
    s_moving = true;
    s_goto_active = false;
    s_park_active = false;
    s_guide_pulse.active = false;
    s_goto_axis_freq[0] = 0U;
    s_goto_axis_freq[1] = 0U;
    s_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    stop_motors();
    s_manual_moving = false;
    s_moving = false;
    if (s_state == OS_STATE_MANUAL_MOTION) {
        s_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_custom_manual_arcsec_per_sec = arcsec_per_sec;
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
    update_current_from_steps();
    coord->ra_hours = wrap_ra_hours(s_current_coord.ra_hours);
    coord->dec_degrees = clamp_dec_degrees(s_current_coord.dec_degrees);
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
    bool tracking_output_active = s_tracking_enabled &&
                                  (s_state == OS_STATE_IDLE_TRACKING ||
                                   s_state == OS_STATE_ALIGNMENT);
    *moving = s_moving || s_guide_pulse.active || tracking_output_active;
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

    uint8_t cfg_byte = (enable ? 1U : 0U);
    if (os_hal_nvm_write(NVM_CONFIG_OFFSET, &cfg_byte, 1U) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }

    if (enable && s_pec_table.valid) {
        if (os_hal_nvm_write(NVM_PEC_OFFSET, (const uint8_t *)&s_pec_table,
                             (uint16_t)sizeof(s_pec_table)) != OS_ERR_NONE) {
            return OS_ERR_NVM_FAULT;
        }
    }

    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!table->valid) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_pec_table = *table;
    s_pec_table.valid = true;

    if (os_hal_nvm_write(NVM_PEC_OFFSET, (const uint8_t *)&s_pec_table,
                         (uint16_t)sizeof(s_pec_table)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }

    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *table = s_pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    int index = (int)worm_phase_deg;
    if (index >= 360) {
        index = 0;
    }

    s_pec_table.corrections[index] = error_arcsec;
    s_pec_table.valid = true;

    if (os_hal_nvm_write(NVM_PEC_OFFSET, (const uint8_t *)&s_pec_table,
                         (uint16_t)sizeof(s_pec_table)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }

    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *calib = s_calib;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    memset(&s_calib, 0, sizeof(s_calib));
    s_calib.valid = false;
    s_alignment_residual_computed = false;
    s_residual_arcsec = 0.0f;

    if (os_hal_nvm_write(NVM_CALIB_OFFSET, (const uint8_t *)&s_calib,
                         (uint16_t)sizeof(s_calib)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }

    return OS_ERR_NONE;
}
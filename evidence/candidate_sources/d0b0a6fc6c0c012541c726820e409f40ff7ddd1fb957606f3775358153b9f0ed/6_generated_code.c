#include "6_generated_code.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define OS_AXIS_RA                 0u
#define OS_AXIS_DEC                1u
#define OS_MOTOR_AXIS_COUNT        2u
#define OS_NVM_PEC_OFFSET          (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)
#define OS_NVM_TOTAL_SIZE          (OS_NVM_PEC_OFFSET + 4u + sizeof(os_pec_table_t))
#define OS_CALIB_MAGIC             0x4F534341u
#define OS_PEC_MAGIC               0x4F535045u
#define OS_MOTOR_MAX_FREQUENCY_HZ  100000u
#define OS_LOOP_PERIOD_MS          10u
#define OS_MOTION_STEPS_PER_LOOP   50000u
#define OS_MOTION_ACCEL_STEPS_PER_LOOP 5000u
#define OS_MOTION_DECEL_STEPS_PER_LOOP 8000u
#define OS_MOTION_MIN_STEPS_PER_LOOP 1u
#define OS_MANUAL_TIMEOUT_ITERATIONS 10000u
#define OS_MOTOR_STEPS_PER_ARCSEC  1.0
#define OS_MOTOR_STEPS_PER_DEGREE  3600.0
#define OS_DEFAULT_PARK_RA_HOURS   0.0f
#define OS_DEFAULT_PARK_DEC_DEG    90.0f
#define OS_CALIBRATION_MAX_RESIDUAL_ARCSEC 600.0
#define OS_PEC_WORM_PERIOD_SECONDS 600.0
#define OS_PI                      3.14159265358979323846
#define OS_PEC_WORM_STEPS          10000.0

typedef struct {
    double ra_hours;
    double dec_degrees;
    int32_t ra_steps;
    int32_t dec_steps;
} align_sample_t;

static os_state_t s_state = OS_STATE_INITIALIZING;
static os_mount_type_t s_mount_type = OS_MOUNT_EQUATORIAL;

static os_track_rate_t s_track_rate = OS_TRACK_RATE_SIDEREAL;
static float s_track_custom_factor = 1.0f;
static bool s_tracking_enabled = false;

static os_calibration_t s_calib = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, false};
static bool s_calib_residual_computed = false;
static float s_calib_residual_arcsec = 0.0f;

static bool s_align_active = false;
static os_align_mode_t s_align_mode = OS_ALIGN_1STAR;
static uint8_t s_align_star_count = 0;
static align_sample_t s_align_samples[OS_CALIBRATION_MAX_STARS];

static float s_guide_rate_fraction = 0.5f;
static os_guide_pulse_t s_guide_pulse = { false, 0u, 0.5f, false, false, false };
static uint32_t s_guide_remaining_ms = 0u;
static bool s_guide_pending_ra = false;
static uint32_t s_guide_pending_ra_duration_ms = 0u;
static bool s_guide_pending_ra_direction_east = true;

static bool s_goto_active = false;
static int32_t s_goto_target_steps[2] = { 0, 0 };
static uint32_t s_goto_velocity[2] = { 0u, 0u };

static bool s_park_active = false;
static int32_t s_park_target_steps[2] = { 0, 0 };
static os_equatorial_coord_t s_park_position = { OS_DEFAULT_PARK_RA_HOURS, OS_DEFAULT_PARK_DEC_DEG };
static bool s_park_position_set = false;

static bool s_manual_active = false;
static os_direction_t s_manual_direction = OS_DIRECTION_NORTH;
static os_speed_level_t s_manual_speed = OS_SPEED_SLOW;
static float s_manual_custom_speed = 100.0f;
static uint32_t s_manual_remaining_iterations = 0u;

static bool s_pec_enabled = false;
static os_pec_table_t s_pec_table = { {0}, false };
static float s_worm_phase_deg = 0.0f;

static os_site_info_t s_site = {0};
static bool s_gps_locked = false;
static bool s_site_valid = false;

static bool s_lx200_ra_set = false;
static float s_lx200_ra_hours = 0.0f;
static bool s_lx200_dec_set = false;
static float s_lx200_dec_deg = 0.0f;

static bool s_motor_initialized[2] = { false, false };
static bool s_motor_enabled[2] = { false, false };
static bool s_motor_direction[2] = { false, false };
static uint32_t s_motor_freq[2] = { 0u, 0u };
static int32_t s_motor_pos[2] = { 0, 0 };
static int32_t s_motor_phase_remainder[2] = { 0, 0 };

static bool s_gps_init = false;
static os_site_info_t s_gps_site = {0};
static bool s_rtc_init = false;
static uint32_t s_rtc_epoch = 0u;
static bool s_limit_init = false;
static bool s_limit_triggered[2] = { false, false };
static bool s_nvm_init = false;
static uint8_t s_nvm[OS_NVM_TOTAL_SIZE];

static bool s_comm_init[4] = { false, false, false, false };
#define OS_COMM_RX_BUF_SIZE OS_MAX_COMMAND_LENGTH
static char s_comm_rx[4][OS_COMM_RX_BUF_SIZE];
static uint16_t s_comm_rx_avail[4] = { 0u, 0u, 0u, 0u };
static uint16_t s_comm_rx_head[4] = { 0u, 0u, 0u, 0u };
static char s_comm_tx[4][OS_MAX_REPLY_LENGTH];
static size_t s_comm_tx_len[4] = { 0u, 0u, 0u, 0u };
static char s_cmd_buf[4][OS_MAX_COMMAND_LENGTH];
static uint16_t s_cmd_len[4] = { 0u, 0u, 0u, 0u };

static bool s_buzzer_pending = false;
static uint16_t s_buzzer_duration = 0u;
static uint8_t s_buzzer_count = 0u;
static bool s_timer_init = false;

static bool valid_ra(float ra_hours) {
    return isfinite(ra_hours) && (ra_hours >= OS_RA_MIN_HOURS) && (ra_hours <= OS_RA_MAX_HOURS);
}
static bool valid_dec(float dec_degrees) {
    return isfinite(dec_degrees) && (dec_degrees >= OS_DEC_MIN_DEG) && (dec_degrees <= OS_DEC_MAX_DEG);
}
static bool valid_az(float az_degrees) {
    return isfinite(az_degrees) && (az_degrees >= 0.0f) && (az_degrees <= 360.0f);
}
static bool valid_alt(float alt_degrees) {
    return isfinite(alt_degrees) && (alt_degrees >= -90.0f) && (alt_degrees <= 90.0f);
}
static bool valid_direction(os_direction_t direction) {
    return (direction >= OS_DIRECTION_NORTH) && (direction <= OS_DIRECTION_WEST);
}
static bool valid_speed(os_speed_level_t speed) {
    return (speed >= OS_SPEED_SLOW) && (speed <= OS_SPEED_CUSTOM);
}
static float clamp_ra(float v) {
    if (v < OS_RA_MIN_HOURS) return OS_RA_MIN_HOURS;
    if (v > OS_RA_MAX_HOURS) return OS_RA_MAX_HOURS;
    return v;
}
static float clamp_dec(float v) {
    if (v < OS_DEC_MIN_DEG) return OS_DEC_MIN_DEG;
    if (v > OS_DEC_MAX_DEG) return OS_DEC_MAX_DEG;
    return v;
}

static uint8_t direction_axis(os_direction_t direction) {
    return ((direction == OS_DIRECTION_NORTH) || (direction == OS_DIRECTION_SOUTH)) ? OS_AXIS_DEC : OS_AXIS_RA;
}
static bool direction_forward(os_direction_t direction) {
    return ((direction == OS_DIRECTION_NORTH) || (direction == OS_DIRECTION_EAST));
}

static void equ_to_motor_steps(os_equatorial_coord_t eq, int32_t *ra_steps, int32_t *dec_steps) {
    if (!s_calib.valid) {
        *ra_steps = (int32_t)lround((double)eq.ra_hours * 15.0 * OS_MOTOR_STEPS_PER_DEGREE);
        *dec_steps = (int32_t)lround((double)eq.dec_degrees * OS_MOTOR_STEPS_PER_DEGREE);
        return;
    }
    double ra_arcsec = (double)eq.ra_hours * 15.0 * 3600.0;
    double dec_arcsec = (double)eq.dec_degrees * 3600.0;
    double mra = (double)s_calib.matrix_ra_to_ra * ra_arcsec
               + (double)s_calib.matrix_ra_to_dec * dec_arcsec
               + (double)s_calib.offset_ra_arcsec;
    double mdec = (double)s_calib.matrix_dec_to_ra * ra_arcsec
                + (double)s_calib.matrix_dec_to_dec * dec_arcsec
                + (double)s_calib.offset_dec_arcsec;
    *ra_steps = (int32_t)lround(mra);
    *dec_steps = (int32_t)lround(mdec);
}
static void horiz_to_motor_steps(os_horizontal_coord_t h, int32_t *ra_steps, int32_t *dec_steps) {
    *ra_steps = (int32_t)lround((double)h.azimuth_degrees * OS_MOTOR_STEPS_PER_DEGREE);
    *dec_steps = (int32_t)lround((double)h.altitude_degrees * OS_MOTOR_STEPS_PER_DEGREE);
}
static void motor_steps_to_equ(int32_t ra_steps, int32_t dec_steps, os_equatorial_coord_t *out) {
    double ra_arcsec = (double)ra_steps;
    double dec_arcsec = (double)dec_steps;
    if (s_calib.valid) {
        double a = (double)s_calib.matrix_ra_to_ra;
        double b = (double)s_calib.matrix_ra_to_dec;
        double c = (double)s_calib.offset_ra_arcsec;
        double d = (double)s_calib.matrix_dec_to_ra;
        double e = (double)s_calib.matrix_dec_to_dec;
        double f = (double)s_calib.offset_dec_arcsec;
        double det = a * e - b * d;
        if (fabs(det) > 1e-12) {
            double mr = (double)ra_steps - c;
            double md = (double)dec_steps - f;
            ra_arcsec = (e * mr - b * md) / det;
            dec_arcsec = (-d * mr + a * md) / det;
        }
    }
    out->ra_hours = clamp_ra((float)(ra_arcsec / (15.0 * 3600.0)));
    out->dec_degrees = clamp_dec((float)(dec_arcsec / 3600.0));
}

static void nvm_reset_calibration(void) {
    s_calib.matrix_ra_to_ra = 1.0f;
    s_calib.matrix_ra_to_dec = 0.0f;
    s_calib.matrix_dec_to_ra = 0.0f;
    s_calib.matrix_dec_to_dec = 1.0f;
    s_calib.offset_ra_arcsec = 0.0f;
    s_calib.offset_dec_arcsec = 0.0f;
    s_calib.valid = false;
}
static void nvm_load_calibration(void) {
    uint8_t magic_bytes[4] = {0};
    if (os_hal_nvm_read(0u, magic_bytes, 4u) != OS_ERR_NONE) {
        nvm_reset_calibration();
        return;
    }
    uint32_t magic = 0u;
    memcpy(&magic, magic_bytes, sizeof(magic));
    if (magic != OS_CALIB_MAGIC) {
        nvm_reset_calibration();
        return;
    }
    os_calibration_t loaded;
    if (os_hal_nvm_read(4u, (uint8_t *)&loaded, sizeof(loaded)) != OS_ERR_NONE) {
        nvm_reset_calibration();
        return;
    }
    if (!loaded.valid ||
        !isfinite(loaded.matrix_ra_to_ra) ||
        !isfinite(loaded.matrix_ra_to_dec) ||
        !isfinite(loaded.matrix_dec_to_ra) ||
        !isfinite(loaded.matrix_dec_to_dec) ||
        !isfinite(loaded.offset_ra_arcsec) ||
        !isfinite(loaded.offset_dec_arcsec)) {
        nvm_reset_calibration();
        return;
    }
    s_calib = loaded;
}
static os_error_t nvm_save_calibration(void) {
    uint8_t buf[4u + sizeof(os_calibration_t)];
    uint32_t magic = OS_CALIB_MAGIC;
    memcpy(buf, &magic, sizeof(magic));
    memcpy(buf + 4u, &s_calib, sizeof(s_calib));
    return os_hal_nvm_write(0u, buf, (uint16_t)sizeof(buf));
}

static void nvm_invalidate_pec(void) {
    s_pec_table.valid = false;
    memset(s_pec_table.corrections, 0, sizeof(s_pec_table.corrections));
}
static void nvm_load_pec(void) {
    uint8_t header[4] = {0};
    if (os_hal_nvm_read(OS_NVM_PEC_OFFSET, header, 4u) != OS_ERR_NONE) {
        nvm_invalidate_pec();
        return;
    }
    uint32_t magic = 0u;
    memcpy(&magic, header, sizeof(magic));
    if (magic != OS_PEC_MAGIC) {
        nvm_invalidate_pec();
        return;
    }
    os_pec_table_t loaded;
    if (os_hal_nvm_read((uint16_t)(OS_NVM_PEC_OFFSET + 4u), (uint8_t *)&loaded, sizeof(loaded)) != OS_ERR_NONE) {
        nvm_invalidate_pec();
        return;
    }
    if (!loaded.valid) {
        nvm_invalidate_pec();
        return;
    }
    memcpy(&s_pec_table, &loaded, sizeof(s_pec_table));
}
static os_error_t nvm_save_pec(void) {
    uint8_t buf[4u + sizeof(os_pec_table_t)];
    uint32_t magic = OS_PEC_MAGIC;
    memcpy(buf, &magic, sizeof(magic));
    memcpy(buf + 4u, &s_pec_table, sizeof(s_pec_table));
    return os_hal_nvm_write(OS_NVM_PEC_OFFSET, buf, (uint16_t)sizeof(buf));
}

static bool parse_ra_hms(const char *s, size_t len, float *ra_hours) {
    if ((s == NULL) || (len == 0u)) {
        return false;
    }
    char buf[32];
    if (len >= sizeof(buf)) return false;
    memcpy(buf, s, len);
    buf[len] = '\0';
    char *end = NULL;
    float h = strtof(buf, &end);
    if (end == buf) return false;
    float m = 0.0f;
    float sec = 0.0f;
    if (*end == ':') {
        char *p = end + 1;
        m = strtof(p, &end);
        if (end == p) return false;
        if (*end == ':') {
            p = end + 1;
            sec = strtof(p, &end);
            if (end == p) return false;
        }
        if (*end != '\0') return false;
    } else if (*end != '\0') {
        return false;
    }
    if ((h < 0.0f) || (h > 24.0f)) return false;
    if ((m < 0.0f) || (m >= 60.0f) || (sec < 0.0f) || (sec >= 60.0f)) return false;
    if ((h == 24.0f) && ((m != 0.0f) || (sec != 0.0f))) return false;
    *ra_hours = h + m / 60.0f + sec / 3600.0f;
    return valid_ra(*ra_hours);
}
static bool parse_dec_dms(const char *s, size_t len, float *dec_deg) {
    if ((s == NULL) || (len == 0u)) return false;
    char buf[32];
    if (len >= sizeof(buf)) return false;
    memcpy(buf, s, len);
    buf[len] = '\0';
    char *p = buf;
    int sign = 1;
    if ((*p == '+') || (*p == '-')) {
        sign = (*p == '-') ? -1 : 1;
        p++;
    }
    char *end = NULL;
    float deg = strtof(p, &end);
    if (end == p) return false;
    float m = 0.0f;
    float sec = 0.0f;
    if (*end == '*') {
        p = end + 1;
        m = strtof(p, &end);
        if (end == p) return false;
        if (*end == ':') {
            p = end + 1;
            sec = strtof(p, &end);
            if (end == p) return false;
        }
        if (*end != '\0') return false;
    } else if (*end == ':') {
        p = end + 1;
        m = strtof(p, &end);
        if (end == p) return false;
        if (*end == ':') {
            p = end + 1;
            sec = strtof(p, &end);
            if (end == p) return false;
        }
        if (*end != '\0') return false;
    } else if (*end != '\0') {
        return false;
    }
    double abs_dec = fabs((double)deg) + ((double)m / 60.0) + ((double)sec / 3600.0);
    if (abs_dec > 90.0) return false;
    if ((m < 0.0f) || (m >= 60.0f) || (sec < 0.0f) || (sec >= 60.0f)) return false;
    float value = (float)(sign * abs_dec);
    if (!valid_dec(value)) return false;
    *dec_deg = value;
    return true;
}

static double deg2rad(double d) { return d * OS_PI / 180.0; }
static double rad2deg(double r) { return r * 180.0 / OS_PI; }
static double wrap_pi(double r) {
    while (r > OS_PI) r -= 2.0 * OS_PI;
    while (r < -OS_PI) r += 2.0 * OS_PI;
    return r;
}
static void horizontal_from_ha(double ha_rad, double lat_rad, double dec_rad,
                               double *alt_rad, double *az_rad) {
    double sin_alt = sin(lat_rad) * sin(dec_rad) + cos(lat_rad) * cos(dec_rad) * cos(ha_rad);
    if (sin_alt > 1.0) sin_alt = 1.0;
    if (sin_alt < -1.0) sin_alt = -1.0;
    *alt_rad = asin(sin_alt);
    double x = -sin(ha_rad) * cos(dec_rad);
    double y = sin(dec_rad) * cos(lat_rad) - cos(dec_rad) * sin(lat_rad) * cos(ha_rad);
    *az_rad = atan2(x, y);
}
static void compute_altaz_rates(double dec_deg, double ha_hours, double dha_dt_deg_s,
                                double lat_deg, double *az_rate_deg_s, double *alt_rate_deg_s) {
    double lat_rad = deg2rad(lat_deg);
    double dec_rad = deg2rad(dec_deg);
    double ha_rad = deg2rad(ha_hours * 15.0);
    double h_rad = 1e-4;
    double alt0, az0, alt1, az1;
    horizontal_from_ha(ha_rad, lat_rad, dec_rad, &alt0, &az0);
    horizontal_from_ha(ha_rad + h_rad, lat_rad, dec_rad, &alt1, &az1);
    double dAlt = (alt1 - alt0) / h_rad;
    double dAz = wrap_pi(az1 - az0) / h_rad;
    double dha_rad_s = dha_dt_deg_s * OS_PI / 180.0;
    *alt_rate_deg_s = dAlt * dha_rad_s * 180.0 / OS_PI;
    *az_rate_deg_s = dAz * dha_rad_s * 180.0 / OS_PI;
}
static double compute_local_sidereal_time_hours(void) {
    double utc_sec = (double)s_site.utc_epoch_seconds;
    double jd = utc_sec / 86400.0 + 2440587.5;
    double d = jd - 2451545.0;
    double T = d / 36525.0;
    double gmst_deg = 280.46061837 + 360.98564736629 * d
                    + 0.000387933 * T * T
                    - (T * T * T) / 38710000.0;
    double gmst_hours = fmod(gmst_deg / 15.0, 24.0);
    if (gmst_hours < 0.0) gmst_hours += 24.0;
    double lst_hours = gmst_hours + (double)s_site.longitude_degrees / 15.0;
    lst_hours = fmod(lst_hours, 24.0);
    if (lst_hours < 0.0) lst_hours += 24.0;
    return lst_hours;
}
static double pec_correction_for_phase(void) {
    if (!(s_pec_enabled && s_pec_table.valid)) {
        return 0.0;
    }
    if (s_worm_phase_deg >= 360.0f) {
        return (double)s_pec_table.corrections[OS_PEC_TABLE_SIZE - 1];
    }
    if (s_worm_phase_deg < 0.0f) {
        return (double)s_pec_table.corrections[0];
    }
    double phase = (double)s_worm_phase_deg;
    int idx = (int)floor(phase);
    if (idx < 0) idx = 0;
    if (idx >= (OS_PEC_TABLE_SIZE - 1)) {
        return (double)s_pec_table.corrections[OS_PEC_TABLE_SIZE - 1];
    }
    double frac = phase - floor(phase);
    double v0 = (double)s_pec_table.corrections[idx];
    double v1 = (double)s_pec_table.corrections[idx + 1];
    return v0 * (1.0 - frac) + v1 * frac;
}
static double compute_tracking_sky_rate_arcsec(void) {
    double rate = (double)OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
    switch (s_track_rate) {
        case OS_TRACK_RATE_LUNAR:
            rate *= (double)OS_LUNAR_RATE_FACTOR;
            break;
        case OS_TRACK_RATE_SOLAR:
            rate *= (double)OS_SOLAR_RATE_FACTOR;
            break;
        case OS_TRACK_RATE_CUSTOM:
            rate *= (double)s_track_custom_factor;
            break;
        case OS_TRACK_RATE_SIDEREAL:
        default:
            break;
    }
    if (s_pec_enabled && s_pec_table.valid) {
        double pec_error_arcsec = pec_correction_for_phase();
        rate += pec_error_arcsec / OS_PEC_WORM_PERIOD_SECONDS;
    }
    if (rate < 0.0) rate = 0.0;
    return rate;
}
static uint32_t cap_motor_frequency(uint32_t freq) {
    if (freq > OS_MOTOR_MAX_FREQUENCY_HZ) freq = OS_MOTOR_MAX_FREQUENCY_HZ;
    return freq;
}
static void compute_tracking_freqs_internal(uint32_t *freq0, uint32_t *freq1,
                                            bool *fwd0, bool *fwd1) {
    *freq0 = 0u;
    *freq1 = 0u;
    *fwd0 = true;
    *fwd1 = false;
    if (!s_tracking_enabled) {
        return;
    }
    double sky_rate_arcsec = compute_tracking_sky_rate_arcsec();
    if (s_mount_type == OS_MOUNT_ALTAZ) {
        int32_t ra_steps = os_hal_motor_get_position(OS_AXIS_RA);
        int32_t dec_steps = os_hal_motor_get_position(OS_AXIS_DEC);
        os_equatorial_coord_t eq;
        motor_steps_to_equ(ra_steps, dec_steps, &eq);
        double lst_hours = compute_local_sidereal_time_hours();
        double ha_hours = lst_hours - (double)eq.ra_hours;
        double dha_dt_deg_s = sky_rate_arcsec / 3600.0;
        double az_rate = 0.0;
        double alt_rate = 0.0;
        compute_altaz_rates((double)eq.dec_degrees, ha_hours, dha_dt_deg_s,
                            (double)s_site.latitude_degrees, &az_rate, &alt_rate);
        *fwd0 = (az_rate >= 0.0);
        *fwd1 = (alt_rate >= 0.0);
        *freq0 = cap_motor_frequency((uint32_t)lround(fabs(az_rate) * OS_MOTOR_STEPS_PER_DEGREE));
        *freq1 = cap_motor_frequency((uint32_t)lround(fabs(alt_rate) * OS_MOTOR_STEPS_PER_DEGREE));
        return;
    }
    double steps_per_arcsec = OS_MOTOR_STEPS_PER_ARCSEC;
    if (s_calib.valid) {
        double scale = fabs((double)s_calib.matrix_ra_to_ra);
        if (scale > 0.0) steps_per_arcsec = scale;
    }
    *freq0 = cap_motor_frequency((uint32_t)lround(sky_rate_arcsec * steps_per_arcsec));
    *freq1 = 0u;
    *fwd0 = true;
    *fwd1 = false;
}
static uint32_t compute_tracking_frequency_hz(void) {
    uint32_t f0 = 0u;
    uint32_t f1 = 0u;
    bool fwd0 = false;
    bool fwd1 = false;
    compute_tracking_freqs_internal(&f0, &f1, &fwd0, &fwd1);
    return f0;
}
static void apply_tracking_frequency(void) {
    uint32_t f0 = 0u;
    uint32_t f1 = 0u;
    bool fwd0 = true;
    bool fwd1 = false;
    compute_tracking_freqs_internal(&f0, &f1, &fwd0, &fwd1);
    if (!s_tracking_enabled) {
        (void)os_hal_motor_set_frequency(OS_AXIS_RA, 0u);
        (void)os_hal_motor_set_frequency(OS_AXIS_DEC, 0u);
        return;
    }
    if (s_mount_type == OS_MOUNT_ALTAZ) {
        (void)os_hal_motor_set_direction(OS_AXIS_RA, fwd0);
        (void)os_hal_motor_set_direction(OS_AXIS_DEC, fwd1);
        (void)os_hal_motor_set_frequency(OS_AXIS_RA, f0);
        (void)os_hal_motor_set_frequency(OS_AXIS_DEC, f1);
        return;
    }
    (void)os_hal_motor_set_direction(OS_AXIS_RA, true);
    (void)os_hal_motor_set_frequency(OS_AXIS_RA, f0);
    (void)os_hal_motor_set_frequency(OS_AXIS_DEC, 0u);
}

static void motor_stop_all(void) {
    (void)os_hal_motor_set_frequency(OS_AXIS_RA, 0u);
    (void)os_hal_motor_set_frequency(OS_AXIS_DEC, 0u);
}
static void clear_active_motions(void) {
    s_goto_active = false;
    s_park_active = false;
    s_manual_active = false;
    s_manual_remaining_iterations = 0u;
    s_guide_pulse.active = false;
    s_guide_pulse.duration_ms = 0u;
    s_guide_pulse.dec_priority = false;
    s_guide_pulse.direction_east = false;
    s_guide_pulse.direction_north = false;
    s_guide_remaining_ms = 0u;
    s_guide_pending_ra = false;
    s_guide_pending_ra_duration_ms = 0u;
    s_guide_pending_ra_direction_east = true;
    s_goto_velocity[0] = 0u;
    s_goto_velocity[1] = 0u;
    motor_stop_all();
}

static void start_goto_from_steps(int32_t ra_target, int32_t dec_target) {
    s_manual_active = false;
    s_manual_remaining_iterations = 0u;
    s_guide_pulse.active = false;
    s_guide_pulse.duration_ms = 0u;
    s_guide_pulse.dec_priority = false;
    s_guide_pulse.direction_east = false;
    s_guide_pulse.direction_north = false;
    s_guide_remaining_ms = 0u;
    s_guide_pending_ra = false;
    s_guide_pending_ra_duration_ms = 0u;
    s_guide_pending_ra_direction_east = true;

    s_goto_target_steps[OS_AXIS_RA] = ra_target;
    s_goto_target_steps[OS_AXIS_DEC] = dec_target;
    s_goto_velocity[0] = 0u;
    s_goto_velocity[1] = 0u;
    s_goto_active = true;
    s_park_active = false;
    motor_stop_all();
    (void)os_hal_motor_enable(OS_AXIS_RA, true);
    (void)os_hal_motor_enable(OS_AXIS_DEC, true);
    s_state = OS_STATE_GOTO;
}

static void step_towards(uint8_t axis, int32_t target) {
    int32_t current = os_hal_motor_get_position(axis);
    int64_t delta64 = (int64_t)target - (int64_t)current;
    if (delta64 == 0) {
        s_goto_velocity[axis] = 0u;
        (void)os_hal_motor_set_frequency(axis, 0u);
        return;
    }
    int32_t direction = (delta64 > 0) ? 1 : -1;
    uint64_t remaining64 = (delta64 > 0) ? (uint64_t)delta64 : (uint64_t)(-delta64);
    uint32_t velocity = s_goto_velocity[axis];
    uint32_t braking = (velocity * velocity) / (2u * OS_MOTION_DECEL_STEPS_PER_LOOP);
    uint64_t decision = (uint64_t)braking + (uint64_t)velocity;
    if (remaining64 <= decision) {
        if (velocity > OS_MOTION_MIN_STEPS_PER_LOOP) {
            velocity = (velocity > OS_MOTION_DECEL_STEPS_PER_LOOP)
                       ? (velocity - OS_MOTION_DECEL_STEPS_PER_LOOP)
                       : OS_MOTION_MIN_STEPS_PER_LOOP;
        }
    } else {
        if (velocity < OS_MOTION_STEPS_PER_LOOP) {
            velocity += OS_MOTION_ACCEL_STEPS_PER_LOOP;
            if (velocity > OS_MOTION_STEPS_PER_LOOP) {
                velocity = OS_MOTION_STEPS_PER_LOOP;
            }
        }
    }
    if (velocity < OS_MOTION_MIN_STEPS_PER_LOOP) {
        velocity = OS_MOTION_MIN_STEPS_PER_LOOP;
    }
    uint32_t step = (remaining64 < (uint64_t)velocity) ? (uint32_t)remaining64 : velocity;
    s_goto_velocity[axis] = velocity;
    int32_t applied = (int32_t)step * direction;
    s_motor_pos[axis] += applied;
    (void)os_hal_motor_set_direction(axis, direction > 0);
    uint32_t freq = velocity * 10u;
    (void)os_hal_motor_set_frequency(axis, cap_motor_frequency(freq));
}
static bool axis_at_target(uint8_t axis, const int32_t target[2]) {
    return os_hal_motor_get_position(axis) == target[axis];
}
static void advance_goto(void) {
    step_towards(OS_AXIS_RA, s_goto_target_steps[OS_AXIS_RA]);
    step_towards(OS_AXIS_DEC, s_goto_target_steps[OS_AXIS_DEC]);
    if (axis_at_target(OS_AXIS_RA, s_goto_target_steps) &&
        axis_at_target(OS_AXIS_DEC, s_goto_target_steps)) {
        s_goto_active = false;
        s_goto_velocity[0] = 0u;
        s_goto_velocity[1] = 0u;
        motor_stop_all();
        s_tracking_enabled = true;
        s_state = OS_STATE_IDLE_TRACKING;
        (void)os_hal_buzzer_beep(100u, 1u);
    }
}
static void advance_park(void) {
    step_towards(OS_AXIS_RA, s_park_target_steps[OS_AXIS_RA]);
    step_towards(OS_AXIS_DEC, s_park_target_steps[OS_AXIS_DEC]);
    if (axis_at_target(OS_AXIS_RA, s_park_target_steps) &&
        axis_at_target(OS_AXIS_DEC, s_park_target_steps)) {
        s_park_active = false;
        s_goto_velocity[0] = 0u;
        s_goto_velocity[1] = 0u;
        motor_stop_all();
        s_tracking_enabled = false;
        (void)os_hal_motor_enable(OS_AXIS_RA, false);
        (void)os_hal_motor_enable(OS_AXIS_DEC, false);
        s_comm_init[OS_CHANNEL_WIFI] = false;
        s_comm_init[OS_CHANNEL_ETHERNET] = false;
        s_state = OS_STATE_PARKED;
    }
}
static uint32_t manual_frequency(void) {
    uint32_t freq = 0u;
    switch (s_manual_speed) {
        case OS_SPEED_SLOW:
            freq = 100u;
            break;
        case OS_SPEED_MEDIUM:
            freq = 500u;
            break;
        case OS_SPEED_FAST:
            freq = 1000u;
            break;
        case OS_SPEED_CUSTOM:
        default:
            freq = (uint32_t)lround((double)s_manual_custom_speed);
            break;
    }
    return cap_motor_frequency(freq);
}
static void stop_manual(void) {
    s_manual_active = false;
    s_manual_remaining_iterations = 0u;
    motor_stop_all();
    s_state = OS_STATE_IDLE_TRACKING;
}
static void accumulate_motor_pulse(uint8_t axis) {
    if (axis >= OS_MOTOR_AXIS_COUNT) return;
    if (!s_motor_enabled[axis]) return;
    if (s_motor_freq[axis] == 0u) return;
    uint64_t total = (uint64_t)s_motor_freq[axis] * OS_LOOP_PERIOD_MS
                   + (uint64_t)(uint32_t)s_motor_phase_remainder[axis];
    uint32_t steps = (uint32_t)(total / 1000u);
    s_motor_phase_remainder[axis] = (int32_t)(total % 1000u);
    int32_t delta = s_motor_direction[axis] ? (int32_t)steps : -(int32_t)steps;
    s_motor_pos[axis] += delta;
}
static void advance_manual(void) {
    uint8_t axis = direction_axis(s_manual_direction);
    bool forward = direction_forward(s_manual_direction);
    if (s_manual_remaining_iterations == 0u) {
        stop_manual();
        return;
    }
    s_manual_remaining_iterations--;
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, manual_frequency());
    accumulate_motor_pulse(axis);
}
static void advance_guide(void) {
    if (!s_guide_pulse.active) {
        return;
    }
    if (s_guide_remaining_ms <= OS_LOOP_PERIOD_MS) {
        s_guide_pulse.active = false;
        s_guide_pulse.dec_priority = false;
        s_guide_pulse.duration_ms = 0u;
        s_guide_pulse.direction_east = false;
        s_guide_pulse.direction_north = false;

        if (s_guide_pending_ra) {
            s_guide_pending_ra = false;
            s_guide_pulse.active = true;
            s_guide_pulse.duration_ms = s_guide_pending_ra_duration_ms;
            s_guide_pulse.rate_fraction = s_guide_rate_fraction;
            s_guide_pulse.dec_priority = false;
            s_guide_pulse.direction_north = false;
            s_guide_pulse.direction_east = s_guide_pending_ra_direction_east;
            s_guide_remaining_ms = s_guide_pending_ra_duration_ms;
            s_guide_pending_ra_duration_ms = 0u;
            (void)os_hal_motor_set_direction(OS_AXIS_RA, s_guide_pulse.direction_east);
            uint32_t base = compute_tracking_frequency_hz();
            uint32_t freq = cap_motor_frequency((uint32_t)lround((double)base * (1.0 + (double)s_guide_rate_fraction)));
            (void)os_hal_motor_set_frequency(OS_AXIS_RA, freq);
            return;
        }

        motor_stop_all();
        return;
    }

    s_guide_remaining_ms -= OS_LOOP_PERIOD_MS;
    bool is_dec = s_guide_pulse.dec_priority;
    bool forward = is_dec ? s_guide_pulse.direction_north : s_guide_pulse.direction_east;
    uint8_t axis = is_dec ? OS_AXIS_DEC : OS_AXIS_RA;
    uint32_t base = compute_tracking_frequency_hz();
    uint32_t freq = (uint32_t)lround((double)base * (1.0 + (double)s_guide_rate_fraction));
    freq = cap_motor_frequency(freq);
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, freq);
}

static bool alignment_min_stars(void) {
    switch (s_align_mode) {
        case OS_ALIGN_1STAR:
            return s_align_star_count >= 1u;
        case OS_ALIGN_2STAR:
            return s_align_star_count >= 2u;
        case OS_ALIGN_3STAR:
        case OS_ALIGN_NSTAR:
        default:
            return s_align_star_count >= 3u;
    }
}
static double det3(const double a[3][3]) {
    return a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
         - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
         + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
}
static bool solve3(double a[3][4], double x[3]) {
    for (int col = 0; col < 3; col++) {
        int pivot = col;
        double best = fabs(a[col][col]);
        for (int row = col + 1; row < 3; row++) {
            double v = fabs(a[row][col]);
            if (v > best) {
                best = v;
                pivot = row;
            }
        }
        if (best < 1e-12) return false;
        if (pivot != col) {
            for (int j = 0; j < 4; j++) {
                double tmp = a[col][j];
                a[col][j] = a[pivot][j];
                a[pivot][j] = tmp;
            }
        }
        for (int row = col + 1; row < 3; row++) {
            double factor = a[row][col] / a[col][col];
            for (int j = col; j < 4; j++) {
                a[row][j] -= factor * a[col][j];
            }
        }
    }
    for (int row = 2; row >= 0; row--) {
        double sum = a[row][3];
        for (int col = row + 1; col < 3; col++) {
            sum -= a[row][col] * x[col];
        }
        if (fabs(a[row][row]) < 1e-12) return false;
        x[row] = sum / a[row][row];
    }
    return true;
}
static bool householder_qr_solve(double a[OS_CALIBRATION_MAX_STARS][3],
                                 const double *b, int m, double x[3]) {
    double r[OS_CALIBRATION_MAX_STARS][3];
    double c[OS_CALIBRATION_MAX_STARS];
    double v[OS_CALIBRATION_MAX_STARS];
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < 3; j++) r[i][j] = a[i][j];
        c[i] = b[i];
    }
    for (int k = 0; k < 3; k++) {
        double norm = 0.0;
        for (int i = k; i < m; i++) norm += r[i][k] * r[i][k];
        norm = sqrt(norm);
        if (norm < 1e-12) return false;
        double sign = (r[k][k] >= 0.0) ? 1.0 : -1.0;
        double vk = r[k][k] + sign * norm;
        v[k] = vk;
        for (int i = k + 1; i < m; i++) v[i] = r[i][k];
        double vnorm2 = 0.0;
        for (int i = k; i < m; i++) vnorm2 += v[i] * v[i];
        if (vnorm2 < 1e-30) return false;
        double beta = 2.0 / vnorm2;
        for (int j = k; j < 3; j++) {
            double dot = 0.0;
            for (int i = k; i < m; i++) dot += v[i] * r[i][j];
            for (int i = k; i < m; i++) r[i][j] -= beta * v[i] * dot;
        }
        double dotc = 0.0;
        for (int i = k; i < m; i++) dotc += v[i] * c[i];
        for (int i = k; i < m; i++) c[i] -= beta * v[i] * dotc;
    }
    for (int i = 2; i >= 0; i--) {
        double sum = c[i];
        for (int j = i + 1; j < 3; j++) sum -= r[i][j] * x[j];
        if (fabs(r[i][i]) < 1e-12) return false;
        x[i] = sum / r[i][i];
    }
    return true;
}
static bool compute_alignment_1star(void) {
    const align_sample_t *s = &s_align_samples[0];
    double ra_arcsec = s->ra_hours * 15.0 * 3600.0;
    double dec_arcsec = s->dec_degrees * 3600.0;
    s_calib.matrix_ra_to_ra = 1.0f;
    s_calib.matrix_ra_to_dec = 0.0f;
    s_calib.matrix_dec_to_ra = 0.0f;
    s_calib.matrix_dec_to_dec = 1.0f;
    s_calib.offset_ra_arcsec = (float)((double)s->ra_steps - ra_arcsec);
    s_calib.offset_dec_arcsec = (float)((double)s->dec_steps - dec_arcsec);
    s_calib.valid = true;
    s_calib_residual_arcsec = 0.0f;
    return true;
}
static bool compute_alignment_2star(void) {
    const align_sample_t *a = &s_align_samples[0];
    const align_sample_t *b = &s_align_samples[1];
    double ra1 = a->ra_hours * 15.0 * 3600.0;
    double ra2 = b->ra_hours * 15.0 * 3600.0;
    double dec1 = a->dec_degrees * 3600.0;
    double dec2 = b->dec_degrees * 3600.0;
    double den_ra = ra2 - ra1;
    double den_dec = dec2 - dec1;
    if (fabs(den_ra) < 1e-12 || fabs(den_dec) < 1e-12) return false;
    double scale_ra = ((double)b->ra_steps - (double)a->ra_steps) / den_ra;
    double scale_dec = ((double)b->dec_steps - (double)a->dec_steps) / den_dec;
    s_calib.matrix_ra_to_ra = (float)scale_ra;
    s_calib.matrix_ra_to_dec = 0.0f;
    s_calib.matrix_dec_to_ra = 0.0f;
    s_calib.matrix_dec_to_dec = (float)scale_dec;
    s_calib.offset_ra_arcsec = (float)((double)a->ra_steps - scale_ra * ra1);
    s_calib.offset_dec_arcsec = (float)((double)a->dec_steps - scale_dec * dec1);
    s_calib.valid = true;
    s_calib_residual_arcsec = 0.0f;
    return true;
}
static bool compute_alignment_3star(void) {
    double A[3][3];
    double b_ra[3];
    double b_dec[3];
    for (int i = 0; i < 3; i++) {
        double ra_arcsec = s_align_samples[i].ra_hours * 15.0 * 3600.0;
        double dec_arcsec = s_align_samples[i].dec_degrees * 3600.0;
        A[i][0] = ra_arcsec;
        A[i][1] = dec_arcsec;
        A[i][2] = 1.0;
        b_ra[i] = (double)s_align_samples[i].ra_steps;
        b_dec[i] = (double)s_align_samples[i].dec_steps;
    }
    if (fabs(det3(A)) < 1e-9) return false;
    double a_ra[3][4];
    double a_dec[3][4];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            a_ra[i][j] = A[i][j];
            a_dec[i][j] = A[i][j];
        }
        a_ra[i][3] = b_ra[i];
        a_dec[i][3] = b_dec[i];
    }
    double x_ra[3] = {0.0, 0.0, 0.0};
    double x_dec[3] = {0.0, 0.0, 0.0};
    if (!solve3(a_ra, x_ra) || !solve3(a_dec, x_dec)) return false;
    s_calib.matrix_ra_to_ra = (float)x_ra[0];
    s_calib.matrix_ra_to_dec = (float)x_ra[1];
    s_calib.offset_ra_arcsec = (float)x_ra[2];
    s_calib.matrix_dec_to_ra = (float)x_dec[0];
    s_calib.matrix_dec_to_dec = (float)x_dec[1];
    s_calib.offset_dec_arcsec = (float)x_dec[2];
    s_calib.valid = true;
    s_calib_residual_arcsec = 0.0f;
    return true;
}
static bool compute_alignment_nstar(void) {
    int m = (int)s_align_star_count;
    if (m == 3) {
        if (!compute_alignment_3star()) return false;
        s_calib_residual_arcsec = 0.0f;
        return true;
    }

    double A[OS_CALIBRATION_MAX_STARS][3];
    double b_ra[OS_CALIBRATION_MAX_STARS];
    double b_dec[OS_CALIBRATION_MAX_STARS];
    for (int i = 0; i < m; i++) {
        double ra_arcsec = s_align_samples[i].ra_hours * 15.0 * 3600.0;
        double dec_arcsec = s_align_samples[i].dec_degrees * 3600.0;
        A[i][0] = ra_arcsec;
        A[i][1] = dec_arcsec;
        A[i][2] = 1.0;
        b_ra[i] = (double)s_align_samples[i].ra_steps;
        b_dec[i] = (double)s_align_samples[i].dec_steps;
    }
    double x_ra[3] = {0.0, 0.0, 0.0};
    double x_dec[3] = {0.0, 0.0, 0.0};
    if (!householder_qr_solve(A, b_ra, m, x_ra) ||
        !householder_qr_solve(A, b_dec, m, x_dec)) return false;
    double sum = 0.0;
    for (int i = 0; i < m; i++) {
        double pred_ra = x_ra[0] * A[i][0] + x_ra[1] * A[i][1] + x_ra[2];
        double pred_dec = x_dec[0] * A[i][0] + x_dec[1] * A[i][1] + x_dec[2];
        double dr = pred_ra - (double)s_align_samples[i].ra_steps;
        double dd = pred_dec - (double)s_align_samples[i].dec_steps;
        sum += dr * dr + dd * dd;
    }
    double rms = sqrt(sum / (double)m);
    if (rms > OS_CALIBRATION_MAX_RESIDUAL_ARCSEC) return false;
    s_calib.matrix_ra_to_ra = (float)x_ra[0];
    s_calib.matrix_ra_to_dec = (float)x_ra[1];
    s_calib.offset_ra_arcsec = (float)x_ra[2];
    s_calib.matrix_dec_to_ra = (float)x_dec[0];
    s_calib.matrix_dec_to_dec = (float)x_dec[1];
    s_calib.offset_dec_arcsec = (float)x_dec[2];
    s_calib.valid = true;
    s_calib_residual_arcsec = (float)rms;
    return true;
}

static void set_reply(char *reply_buffer, size_t reply_buffer_size,
                      size_t *reply_length, const char *msg) {
    if ((reply_buffer == NULL) || (reply_length == NULL) || (reply_buffer_size == 0u)) {
        if (reply_length != NULL) *reply_length = 0u;
        return;
    }
    int n = snprintf(reply_buffer, reply_buffer_size, "%s", msg);
    if (n < 0) {
        *reply_length = 0u;
        return;
    }
    if ((size_t)n >= reply_buffer_size) {
        *reply_length = reply_buffer_size - 1u;
    } else {
        *reply_length = (size_t)n;
    }
}

static void maybe_dispatch_command_channel(uint8_t channel) {
    uint16_t len = s_cmd_len[channel];
    if (len < 2u) return;
    if ((s_cmd_buf[channel][0] != OS_LX200_CMD_PREFIX) ||
        (s_cmd_buf[channel][len - 1u] != OS_LX200_CMD_SUFFIX)) {
        return;
    }
    char reply[OS_MAX_REPLY_LENGTH];
    size_t reply_len = 0u;
    (void)os_command_parse(s_cmd_buf[channel], (size_t)len, channel,
                           reply, sizeof(reply), &reply_len);
}
static void handle_loop_command_byte(uint8_t channel, char c) {
    if ((c == '\n') || (c == '\r')) {
        if (s_cmd_len[channel] > 0u) {
            maybe_dispatch_command_channel(channel);
        }
        s_cmd_len[channel] = 0u;
        return;
    }
    if (s_cmd_len[channel] >= OS_MAX_COMMAND_LENGTH) {
        s_cmd_len[channel] = 0u;
    }
    s_cmd_buf[channel][s_cmd_len[channel]++] = c;
    if ((c == OS_LX200_CMD_SUFFIX) &&
        (s_cmd_len[channel] > 1u) &&
        (s_cmd_buf[channel][0] == OS_LX200_CMD_PREFIX)) {
        maybe_dispatch_command_channel(channel);
        s_cmd_len[channel] = 0u;
    }
}

static bool refresh_site_time(void) {
    os_site_info_t gps_site;
    if ((os_hal_gps_poll(&gps_site) == OS_ERR_NONE) && gps_site.valid) {
        s_site = gps_site;
        s_gps_locked = true;
        s_site_valid = true;
        (void)os_hal_rtc_set(s_site.utc_epoch_seconds);
    } else {
        s_gps_locked = false;
        uint32_t utc = 0u;
        if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = utc;
            s_site_valid = true;
        } else {
            s_site_valid = false;
        }
    }
    s_site.valid = s_site_valid;
    return s_site_valid;
}

os_error_t os_init(void) {
    s_state = OS_STATE_INITIALIZING;

    clear_active_motions();
    s_manual_direction = OS_DIRECTION_NORTH;
    s_manual_speed = OS_SPEED_SLOW;
    s_manual_custom_speed = 100.0f;
    s_guide_rate_fraction = 0.5f;
    s_guide_pulse.rate_fraction = s_guide_rate_fraction;
    s_tracking_enabled = false;
    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_track_custom_factor = 1.0f;
    s_align_active = false;
    s_align_mode = OS_ALIGN_1STAR;
    s_align_star_count = 0u;
    memset(s_align_samples, 0, sizeof(s_align_samples));
    nvm_reset_calibration();
    s_calib_residual_computed = false;
    s_calib_residual_arcsec = 0.0f;
    s_pec_enabled = false;
    nvm_invalidate_pec();
    s_worm_phase_deg = 0.0f;
    s_gps_locked = false;
    s_site_valid = false;
    memset(&s_site, 0, sizeof(s_site));
    s_park_position.ra_hours = OS_DEFAULT_PARK_RA_HOURS;
    s_park_position.dec_degrees = OS_DEFAULT_PARK_DEC_DEG;
    s_park_position_set = false;
    s_lx200_ra_set = false;
    s_lx200_ra_hours = 0.0f;
    s_lx200_dec_set = false;
    s_lx200_dec_deg = 0.0f;
    s_mount_type = OS_MOUNT_EQUATORIAL;
    s_goto_velocity[0] = 0u;
    s_goto_velocity[1] = 0u;
    s_motor_phase_remainder[0] = 0;
    s_motor_phase_remainder[1] = 0;
    for (uint8_t i = 0u; i < 4u; i++) {
        s_cmd_len[i] = 0u;
    }

    os_error_t err = os_hal_nvm_init();
    if (err != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return err;
    }

    nvm_load_calibration();
    nvm_load_pec();

    (void)os_hal_comm_init(OS_CHANNEL_USB);
    (void)os_hal_comm_init(OS_CHANNEL_BLUETOOTH);
    (void)os_hal_comm_init(OS_CHANNEL_WIFI);
    (void)os_hal_comm_init(OS_CHANNEL_ETHERNET);

    err = os_hal_motor_init(OS_AXIS_RA);
    if (err != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return err;
    }
    err = os_hal_motor_init(OS_AXIS_DEC);
    if (err != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return err;
    }

    err = os_hal_limit_init();
    if ((err != OS_ERR_NONE) && (err != OS_ERR_NOT_SUPPORTED)) {
        s_state = OS_STATE_FAULT;
        return err;
    }

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();

    err = os_hal_timer_motor_init();
    if (err != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return err;
    }

    (void)refresh_site_time();
    if (!s_site_valid) {
        s_tracking_enabled = false;
        s_state = OS_STATE_FAULT;
        return OS_ERR_TIMEOUT;
    }

    s_tracking_enabled = true;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    for (uint8_t ch = 0u; ch < 4u; ch++) {
        if (!s_comm_init[ch]) continue;
        int16_t avail = os_hal_comm_available(ch);
        if (avail < 0) avail = 0;
        while (avail > 0) {
            char c = os_hal_comm_read(ch);
            handle_loop_command_byte(ch, c);
            avail--;
        }
    }

    (void)refresh_site_time();
    if (!s_site_valid && s_tracking_enabled && (s_state == OS_STATE_IDLE_TRACKING)) {
        s_tracking_enabled = false;
        motor_stop_all();
        s_state = OS_STATE_FAULT;
        return;
    }

    int32_t ra_pos = os_hal_motor_get_position(OS_AXIS_RA);
    double raw_phase = fmod((double)ra_pos, OS_PEC_WORM_STEPS);
    if (raw_phase < 0.0) raw_phase += OS_PEC_WORM_STEPS;
    s_worm_phase_deg = (float)(raw_phase / OS_PEC_WORM_STEPS * 360.0);

    bool ra_limit = os_hal_limit_is_triggered(OS_AXIS_RA);
    bool dec_limit = os_hal_limit_is_triggered(OS_AXIS_DEC);
    if (ra_limit || dec_limit) {
        motor_stop_all();
        s_goto_active = false;
        s_park_active = false;
        s_manual_active = false;
        s_guide_pulse.active = false;
        s_guide_remaining_ms = 0u;
        s_state = OS_STATE_FAULT;
        return;
    }

    if (s_goto_active) {
        advance_goto();
        return;
    }
    if (s_park_active) {
        advance_park();
        return;
    }
    if (s_manual_active) {
        advance_manual();
        return;
    }
    if (s_guide_pulse.active) {
        apply_tracking_frequency();
        advance_guide();
        accumulate_motor_pulse(OS_AXIS_RA);
        accumulate_motor_pulse(OS_AXIS_DEC);
        return;
    }
    if (s_tracking_enabled && (s_state == OS_STATE_IDLE_TRACKING)) {
        apply_tracking_frequency();
        accumulate_motor_pulse(OS_AXIS_RA);
        if (s_mount_type == OS_MOUNT_ALTAZ) {
            accumulate_motor_pulse(OS_AXIS_DEC);
        }
    } else {
        motor_stop_all();
    }
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if ((command == NULL) || (reply_buffer == NULL) || (reply_length == NULL)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((length < 2u) || (length > OS_MAX_COMMAND_LENGTH)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (source_channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (reply_buffer_size == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *reply_length = 0u;

    if ((command[0] != OS_LX200_CMD_PREFIX) || (command[length - 1u] != OS_LX200_CMD_SUFFIX)) {
        set_reply(reply_buffer, reply_buffer_size, reply_length, "ERR_COMMAND_FORMAT");
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return OS_ERR_COMMAND_FORMAT;
    }

    const char *body = command + 1;
    size_t body_len = length - 2u;

    if ((body_len == 2u) && (memcmp(body, "MS", 2u) == 0)) {
        os_equatorial_coord_t target;
        os_error_t err = OS_ERR_NONE;
        if (s_lx200_ra_set && s_lx200_dec_set) {
            target.ra_hours = s_lx200_ra_hours;
            target.dec_degrees = s_lx200_dec_deg;
        } else {
            err = os_query_coordinates(&target);
        }
        if (err == OS_ERR_NONE) {
            err = os_goto_equatorial(target);
        }
        set_reply(reply_buffer, reply_buffer_size, reply_length, (err == OS_ERR_NONE) ? "0" : "ERR_GOTO");
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return err;
    }

    if ((body_len >= 2u) && (body[0] == 'S') && (body[1] == 'r')) {
        float ra_hours = 0.0f;
        if (parse_ra_hms(body + 2u, body_len - 2u, &ra_hours)) {
            s_lx200_ra_hours = ra_hours;
            s_lx200_ra_set = true;
            set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
            (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
            return OS_ERR_NONE;
        }
        set_reply(reply_buffer, reply_buffer_size, reply_length, "ERR_INVALID_ARGUMENT");
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return OS_ERR_INVALID_ARGUMENT;
    }

    if ((body_len >= 2u) && (body[0] == 'S') && (body[1] == 'd')) {
        float dec_deg = 0.0f;
        if (parse_dec_dms(body + 2u, body_len - 2u, &dec_deg)) {
            s_lx200_dec_deg = dec_deg;
            s_lx200_dec_set = true;
            set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
            (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
            return OS_ERR_NONE;
        }
        set_reply(reply_buffer, reply_buffer_size, reply_length, "ERR_INVALID_ARGUMENT");
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return OS_ERR_INVALID_ARGUMENT;
    }

    if ((body_len == 2u) && (memcmp(body, "GR", 2u) == 0)) {
        os_equatorial_coord_t coord;
        os_error_t err = os_query_coordinates(&coord);
        if (err != OS_ERR_NONE) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "ERR_QUERY");
        } else {
            char tmp[OS_MAX_REPLY_LENGTH];
            (void)snprintf(tmp, sizeof(tmp), "%.6f", (double)coord.ra_hours);
            set_reply(reply_buffer, reply_buffer_size, reply_length, tmp);
        }
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return err;
    }

    if ((body_len == 2u) && (memcmp(body, "GD", 2u) == 0)) {
        os_equatorial_coord_t coord;
        os_error_t err = os_query_coordinates(&coord);
        if (err != OS_ERR_NONE) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "ERR_QUERY");
        } else {
            char tmp[OS_MAX_REPLY_LENGTH];
            (void)snprintf(tmp, sizeof(tmp), "%.6f", (double)coord.dec_degrees);
            set_reply(reply_buffer, reply_buffer_size, reply_length, tmp);
        }
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return err;
    }

    if ((body_len == 3u) && (memcmp(body, "GVP", 3u) == 0)) {
        uint8_t major = 0u;
        uint8_t minor = 0u;
        uint8_t patch = 0u;
        os_error_t err = os_query_firmware_version(&major, &minor, &patch);
        char tmp[OS_MAX_REPLY_LENGTH];
        (void)snprintf(tmp, sizeof(tmp), "%u.%u.%u", (unsigned)major, (unsigned)minor, (unsigned)patch);
        set_reply(reply_buffer, reply_buffer_size, reply_length, tmp);
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return err;
    }

    if ((body_len == 2u) && (memcmp(body, "Me", 2u) == 0)) {
        os_error_t err = os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
        set_reply(reply_buffer, reply_buffer_size, reply_length, (err == OS_ERR_NONE) ? "0" : "ERR_MOTION");
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return err;
    }
    if ((body_len == 2u) && (memcmp(body, "Mw", 2u) == 0)) {
        os_error_t err = os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
        set_reply(reply_buffer, reply_buffer_size, reply_length, (err == OS_ERR_NONE) ? "0" : "ERR_MOTION");
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return err;
    }
    if ((body_len == 2u) && (memcmp(body, "Mn", 2u) == 0)) {
        os_error_t err = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
        set_reply(reply_buffer, reply_buffer_size, reply_length, (err == OS_ERR_NONE) ? "0" : "ERR_MOTION");
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return err;
    }
    if ((body_len == 2u) && (memcmp(body, "Ms", 2u) == 0)) {
        os_error_t err = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
        set_reply(reply_buffer, reply_buffer_size, reply_length, (err == OS_ERR_NONE) ? "0" : "ERR_MOTION");
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return err;
    }
    if ((body_len == 1u) && (body[0] == 'Q')) {
        os_error_t err = os_move_stop();
        set_reply(reply_buffer, reply_buffer_size, reply_length, (err == OS_ERR_NONE) ? "0" : "ERR_STOP");
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return err;
    }
    if ((body_len == 2u) && (memcmp(body, "hP", 2u) == 0)) {
        os_error_t err = os_park();
        set_reply(reply_buffer, reply_buffer_size, reply_length, (err == OS_ERR_NONE) ? "0" : "ERR_PARK");
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return err;
    }
    if ((body_len == 2u) && (memcmp(body, "hO", 2u) == 0)) {
        os_error_t err = os_unpark();
        set_reply(reply_buffer, reply_buffer_size, reply_length, (err == OS_ERR_NONE) ? "0" : "ERR_UNPARK");
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return err;
    }

    set_reply(reply_buffer, reply_buffer_size, reply_length, "ERR_NOT_SUPPORTED");
    (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
    return OS_ERR_NOT_SUPPORTED;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (!valid_ra(target.ra_hours) || !valid_dec(target.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    int32_t ra_steps = 0;
    int32_t dec_steps = 0;
    equ_to_motor_steps(target, &ra_steps, &dec_steps);
    start_goto_from_steps(ra_steps, dec_steps);
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (!valid_az(target.azimuth_degrees) || !valid_alt(target.altitude_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    int32_t ra_steps = 0;
    int32_t dec_steps = 0;
    horiz_to_motor_steps(target, &ra_steps, &dec_steps);
    start_goto_from_steps(ra_steps, dec_steps);
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (!s_goto_active) {
        return OS_ERR_INVALID_STATE;
    }
    s_goto_active = false;
    s_goto_velocity[0] = 0u;
    s_goto_velocity[1] = 0u;
    motor_stop_all();
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if ((rate < OS_TRACK_RATE_SIDEREAL) || (rate > OS_TRACK_RATE_CUSTOM)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((rate == OS_TRACK_RATE_CUSTOM) && ((!isfinite(custom_factor)) || (custom_factor <= 0.0f))) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_track_rate = rate;
    s_track_custom_factor = (rate == OS_TRACK_RATE_CUSTOM) ? custom_factor : 1.0f;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if ((rate == NULL) || (custom_factor == NULL)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = s_track_rate;
    *custom_factor = s_track_custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    if ((s_state == OS_STATE_PARKED) || (s_state == OS_STATE_FAULT)) {
        return OS_ERR_INVALID_STATE;
    }
    s_tracking_enabled = true;
    if ((s_state == OS_STATE_IDLE_TRACKING) && !s_goto_active && !s_park_active && !s_manual_active && !s_guide_pulse.active) {
        apply_tracking_frequency();
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    s_tracking_enabled = false;
    if (!s_goto_active && !s_park_active && !s_manual_active && !s_guide_pulse.active) {
        motor_stop_all();
    }
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (!valid_direction(direction)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    uint8_t axis = direction_axis(direction);
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    bool new_dec = (axis == OS_AXIS_DEC);
    if (s_guide_pulse.active && s_guide_pulse.dec_priority && !new_dec) {
        s_guide_pending_ra = true;
        s_guide_pending_ra_duration_ms = duration_ms;
        s_guide_pending_ra_direction_east = (direction == OS_DIRECTION_EAST);
        return OS_ERR_NONE;
    }

    s_guide_pending_ra = false;
    s_guide_pending_ra_duration_ms = 0u;

    s_guide_pulse.active = true;
    s_guide_pulse.duration_ms = duration_ms;
    s_guide_pulse.rate_fraction = s_guide_rate_fraction;
    s_guide_pulse.dec_priority = new_dec;
    s_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    s_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    s_guide_remaining_ms = duration_ms;
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if ((!isfinite(rate_fraction)) ||
        (rate_fraction < OS_GUIDE_RATE_MIN) ||
        (rate_fraction > OS_GUIDE_RATE_MAX)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_guide_rate_fraction = rate_fraction;
    if (!s_guide_pulse.active) {
        s_guide_pulse.rate_fraction = rate_fraction;
    }
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
    if ((mode < OS_ALIGN_1STAR) || (mode > OS_ALIGN_NSTAR)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_align_active = true;
    s_align_mode = mode;
    s_align_star_count = 0u;
    s_calib_residual_computed = false;
    s_calib_residual_arcsec = 0.0f;
    memset(s_align_samples, 0, sizeof(s_align_samples));
    s_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (!valid_ra(star_coord.ra_hours) || !valid_dec(star_coord.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_align_active) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_align_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }
    align_sample_t *sample = &s_align_samples[s_align_star_count];
    sample->ra_hours = (double)star_coord.ra_hours;
    sample->dec_degrees = (double)star_coord.dec_degrees;
    sample->ra_steps = motor_pos.ra_steps;
    sample->dec_steps = motor_pos.dec_steps;
    s_align_star_count++;
    s_calib_residual_computed = false;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (!s_align_active) {
        return OS_ERR_INVALID_STATE;
    }
    if (!alignment_min_stars()) {
        return OS_ERR_INVALID_STATE;
    }
    bool ok = false;
    switch (s_align_mode) {
        case OS_ALIGN_1STAR:
            ok = compute_alignment_1star();
            break;
        case OS_ALIGN_2STAR:
            ok = compute_alignment_2star();
            break;
        case OS_ALIGN_3STAR:
            ok = compute_alignment_3star();
            break;
        case OS_ALIGN_NSTAR:
            ok = compute_alignment_nstar();
            break;
        default:
            return OS_ERR_INVALID_ARGUMENT;
    }
    if (!ok) {
        return OS_ERR_CALIBRATION_FAILED;
    }
    s_calib_residual_computed = true;
    os_error_t err = nvm_save_calibration();
    if (err != OS_ERR_NONE) {
        return err;
    }
    s_align_active = false;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_calib_residual_computed) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = s_calib_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    if (!s_align_active) {
        return OS_ERR_INVALID_STATE;
    }
    s_align_active = false;
    s_align_star_count = 0u;
    s_calib_residual_computed = false;
    s_calib_residual_arcsec = 0.0f;
    memset(s_align_samples, 0, sizeof(s_align_samples));
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (s_state == OS_STATE_PARKED) {
        return OS_ERR_NONE;
    }

    clear_active_motions();
    s_tracking_enabled = false;

    int32_t ra_steps = 0;
    int32_t dec_steps = 0;
    equ_to_motor_steps(s_park_position, &ra_steps, &dec_steps);
    s_park_target_steps[OS_AXIS_RA] = ra_steps;
    s_park_target_steps[OS_AXIS_DEC] = dec_steps;
    s_park_active = true;
    (void)os_hal_motor_enable(OS_AXIS_RA, true);
    (void)os_hal_motor_enable(OS_AXIS_DEC, true);
    s_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (s_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    clear_active_motions();
    (void)os_hal_motor_enable(OS_AXIS_RA, true);
    (void)os_hal_motor_enable(OS_AXIS_DEC, true);
    (void)os_hal_comm_init(OS_CHANNEL_USB);
    (void)os_hal_comm_init(OS_CHANNEL_BLUETOOTH);
    (void)os_hal_comm_init(OS_CHANNEL_WIFI);
    (void)os_hal_comm_init(OS_CHANNEL_ETHERNET);

    (void)refresh_site_time();
    if (!s_site_valid) {
        s_tracking_enabled = false;
        motor_stop_all();
        s_state = OS_STATE_FAULT;
        return OS_ERR_TIMEOUT;
    }

    s_tracking_enabled = true;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!valid_ra(park_pos.ra_hours) || !valid_dec(park_pos.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_park_position = park_pos;
    s_park_position_set = true;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (!valid_direction(direction)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!valid_speed(speed)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t axis = direction_axis(direction);
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_manual_active = true;
    s_manual_direction = direction;
    s_manual_speed = speed;
    s_manual_remaining_iterations = OS_MANUAL_TIMEOUT_ITERATIONS;
    motor_stop_all();
    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_direction(axis, direction_forward(direction));
    (void)os_hal_motor_set_frequency(axis, manual_frequency());
    s_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (!s_manual_active) {
        return OS_ERR_INVALID_STATE;
    }
    stop_manual();
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if ((!isfinite(arcsec_per_sec)) || (arcsec_per_sec <= 0.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_manual_custom_speed = arcsec_per_sec;
    if (s_manual_active && (s_manual_speed == OS_SPEED_CUSTOM)) {
        uint8_t axis = direction_axis(s_manual_direction);
        (void)os_hal_motor_set_frequency(axis, manual_frequency());
    }
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
    int32_t ra_steps = os_hal_motor_get_position(OS_AXIS_RA);
    int32_t dec_steps = os_hal_motor_get_position(OS_AXIS_DEC);
    motor_steps_to_equ(ra_steps, dec_steps, coord);
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    (void)refresh_site_time();
    *site = s_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = os_hal_motor_get_position(OS_AXIS_RA);
    pos->dec_steps = os_hal_motor_get_position(OS_AXIS_DEC);
    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor, uint8_t *patch) {
    if ((major == NULL) || (minor == NULL) || (patch == NULL)) {
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
    *moving = s_goto_active || s_park_active || s_manual_active || s_guide_pulse.active;
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
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(&s_pec_table, table, sizeof(s_pec_table));
    return nvm_save_pec();
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(table, &s_pec_table, sizeof(s_pec_table));
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if ((!isfinite(worm_phase_deg)) || (worm_phase_deg < 0.0f) || (worm_phase_deg > 360.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int index = 0;
    if (worm_phase_deg >= 360.0f) {
        index = OS_PEC_TABLE_SIZE - 1;
    } else {
        index = (int)worm_phase_deg;
    }
    if (index < 0) index = 0;
    if (index >= OS_PEC_TABLE_SIZE) index = OS_PEC_TABLE_SIZE - 1;
    s_pec_table.corrections[index] = error_arcsec;
    s_pec_table.valid = true;
    return nvm_save_pec();
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *calib = s_calib;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    nvm_reset_calibration();
    uint8_t zero[4] = {0u, 0u, 0u, 0u};
    return os_hal_nvm_write(0u, zero, 4u);
}

os_error_t os_hal_motor_init(uint8_t axis) {
    if (axis >= OS_MOTOR_AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_motor_initialized[axis] = true;
    s_motor_enabled[axis] = false;
    s_motor_freq[axis] = 0u;
    s_motor_direction[axis] = false;
    s_motor_phase_remainder[axis] = 0;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (axis >= OS_MOTOR_AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (frequency_hz > OS_MOTOR_MAX_FREQUENCY_HZ) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_motor_freq[axis] = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (axis >= OS_MOTOR_AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_motor_direction[axis] = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (axis >= OS_MOTOR_AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_motor_enabled[axis] = enable;
    if (!enable) {
        s_motor_freq[axis] = 0u;
    }
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (axis >= OS_MOTOR_AXIS_COUNT) {
        return 0;
    }
    return s_motor_pos[axis];
}

os_error_t os_hal_gps_init(void) {
    s_gps_init = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_gps_init) {
        site->valid = false;
        return OS_ERR_NOT_SUPPORTED;
    }
    if (!s_gps_site.valid) {
        site->valid = false;
        return OS_ERR_GPS_NO_SIGNAL;
    }
    *site = s_gps_site;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void) {
    s_rtc_init = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_rtc_init) {
        return OS_ERR_NOT_SUPPORTED;
    }
    *utc_epoch_seconds = s_rtc_epoch;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    if (!s_rtc_init) {
        return OS_ERR_NOT_SUPPORTED;
    }
    s_rtc_epoch = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    s_limit_init = true;
    s_limit_triggered[OS_AXIS_RA] = false;
    s_limit_triggered[OS_AXIS_DEC] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (axis >= OS_MOTOR_AXIS_COUNT) {
        return true;
    }
    return s_limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void) {
    if (!s_nvm_init) {
        memset(s_nvm, 0, sizeof(s_nvm));
        s_nvm_init = true;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_nvm_init) {
        return OS_ERR_INVALID_STATE;
    }
    uint32_t end = (uint32_t)offset + (uint32_t)length;
    if (end > OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(data, &s_nvm[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_nvm_init) {
        return OS_ERR_INVALID_STATE;
    }
    uint32_t end = (uint32_t)offset + (uint32_t)length;
    if (end > OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(&s_nvm[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_comm_init[channel] = true;
    memset(s_comm_rx[channel], 0, sizeof(s_comm_rx[channel]));
    memset(s_comm_tx[channel], 0, sizeof(s_comm_tx[channel]));
    s_comm_rx_avail[channel] = 0u;
    s_comm_rx_head[channel] = 0u;
    s_comm_tx_len[channel] = 0u;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if ((channel > OS_CHANNEL_ETHERNET) || !s_comm_init[channel]) {
        return 0;
    }
    return (int16_t)s_comm_rx_avail[channel];
}

char os_hal_comm_read(uint8_t channel) {
    if ((channel > OS_CHANNEL_ETHERNET) || !s_comm_init[channel]) {
        return '\0';
    }
    if (s_comm_rx_avail[channel] == 0u) {
        return '\0';
    }
    char byte = s_comm_rx[channel][s_comm_rx_head[channel]];
    s_comm_rx_head[channel]++;
    if (s_comm_rx_head[channel] >= OS_COMM_RX_BUF_SIZE) {
        s_comm_rx_head[channel] = 0u;
    }
    s_comm_rx_avail[channel]--;
    return byte;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_comm_init[channel]) {
        return OS_ERR_NOT_SUPPORTED;
    }
    if ((data == NULL) && (length > 0u)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length > OS_MAX_REPLY_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(s_comm_tx[channel], data, length);
    s_comm_tx_len[channel] = length;
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) {
    s_buzzer_pending = true;
    s_buzzer_duration = duration_ms;
    s_buzzer_count = count;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void) {
    s_timer_init = true;
    return OS_ERR_NONE;
}
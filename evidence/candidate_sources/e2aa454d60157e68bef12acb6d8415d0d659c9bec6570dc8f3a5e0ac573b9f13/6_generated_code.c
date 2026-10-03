#include "6_generated_code.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define OS_NVM_TOTAL_CAPACITY 2048u
#define OS_NVM_MAX_RECORD_TOTAL_BYTES 396u
#define OS_NVM_MAGIC_CALIBRATION 0x4F43414Cu
#define OS_NVM_MAGIC_PARK        0x4F504152u
#define OS_NVM_MAGIC_SITE        0x4F534954u
#define OS_NVM_MAGIC_MOTOR       0x4F4D4F54u
#define OS_NVM_MAGIC_PEC         0x4F504543u
#define OS_NVM_CAL_OFFSET    0u
#define OS_NVM_PEC_OFFSET    256u
#define OS_NVM_SITE_OFFSET   640u
#define OS_NVM_PARK_OFFSET   704u
#define OS_NVM_MOTOR_OFFSET  768u

#define OS_COMM_RX_CAPACITY 160u
#define OS_COMM_TX_CAPACITY 256u
#define OS_MOTOR_MAX_FREQ_HZ 100000u
#define OS_MANUAL_STEPS_PER_ARCSEC 10.0
#define OS_TRACKING_STEPS_PER_ARCSEC 10.0
#define OS_TRACKING_FREQ_SIDEREAL_HZ 150u

#define OS_TRAJ_MAX_SPEED_STEPS_PER_LOOP 120
#define OS_TRAJ_MIN_SPEED_STEPS_PER_LOOP 2
#define OS_TRAJ_LOW_SPEED_STEPS_PER_LOOP 2
#define OS_TRAJ_ACCEL_STEPS_PER_LOOP 10
#define OS_TRAJ_DECEL_STEPS_PER_LOOP 10
#define OS_TRAJ_LOW_DISTANCE_STEPS 25
#define OS_TRAJ_HZ_PER_STEP 20u

typedef enum {
    TRAJ_PHASE_IDLE = 0,
    TRAJ_PHASE_ACCEL,
    TRAJ_PHASE_CRUISE,
    TRAJ_PHASE_DECEL,
    TRAJ_PHASE_LOW
} traj_phase_t;

static os_state_t s_state = OS_STATE_INITIALIZING;
static os_mount_type_t s_mount_type = OS_MOUNT_EQUATORIAL;

static bool s_tracking_enabled = false;
static os_track_rate_t s_tracking_rate = OS_TRACK_RATE_SIDEREAL;
static float s_tracking_custom_factor = 1.0f;
static float s_tracking_rate_factor = 1.0f;
static os_equatorial_coord_t s_tracking_ref_coord = {0.0f, 0.0f};

static os_calibration_t s_calibration;
static bool s_residual_valid = false;
static float s_residual_arcsec = 0.0f;

static bool s_goto_active = false;
static bool s_goto_resume_tracking = true;
static int32_t s_goto_target_steps[2] = {0, 0};
static traj_phase_t s_traj_phase[2] = {TRAJ_PHASE_IDLE, TRAJ_PHASE_IDLE};
static int32_t s_traj_speed_steps[2] = {0, 0};

static bool s_park_active = false;
static bool s_park_resume_tracking = true;
static os_equatorial_coord_t s_park_position;
static bool s_park_position_valid = false;

static bool s_manual_active = false;
static os_direction_t s_manual_direction = OS_DIRECTION_NORTH;
static os_speed_level_t s_manual_speed = OS_SPEED_SLOW;
static float s_manual_custom_speed_arcsec_per_sec = 100.0f;

static os_guide_pulse_t s_guide_pulse;
static uint32_t s_guide_start_ms = 0u;

static bool s_alignment_active = false;
static os_align_mode_t s_alignment_mode = OS_ALIGN_1STAR;
static uint8_t s_alignment_star_count = 0u;
static os_equatorial_coord_t s_alignment_stars[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t s_alignment_motor_pos[OS_CALIBRATION_MAX_STARS];

static os_pec_table_t s_pec_table;
static bool s_pec_enabled = false;

static os_site_info_t s_site;
static bool s_gps_locked = false;
static uint32_t s_system_tick_ms = 0u;

static bool s_hal_motor_initialized[2] = {false, false};
static bool s_hal_motor_enabled[2] = {false, false};
static bool s_hal_motor_direction[2] = {false, false};
static uint32_t s_hal_motor_frequency[2] = {0u, 0u};
static int32_t s_motor_steps[2] = {0, 0};
static uint32_t s_motor_substep[2] = {0u, 0u};

static bool s_hal_gps_initialized = false;
static bool s_hal_gps_valid = false;
static os_site_info_t s_hal_gps_site;

static bool s_hal_rtc_initialized = false;
static uint32_t s_hal_rtc_time = 0u;

static bool s_hal_limit_initialized = false;
static bool s_hal_limit_triggered[2] = {false, false};

static bool s_hal_nvm_initialized = false;
static uint8_t s_hal_nvm[OS_NVM_TOTAL_CAPACITY];

static bool s_hal_comm_initialized[4] = {false, false, false, false};
static char s_hal_comm_rx[4][OS_COMM_RX_CAPACITY];
static uint16_t s_hal_comm_rx_len[4] = {0u, 0u, 0u, 0u};
static uint16_t s_hal_comm_rx_head[4] = {0u, 0u, 0u, 0u};
static char s_hal_comm_tx[4][OS_COMM_TX_CAPACITY];
static size_t s_hal_comm_tx_len[4] = {0u, 0u, 0u, 0u};

static bool s_hal_motor_timer_initialized = false;
static os_equatorial_coord_t s_goto_command_target = {0.0f, 0.0f};
static char s_loop_command_buffer[4][OS_MAX_COMMAND_LENGTH + 1u];
static size_t s_loop_command_len[4] = {0u, 0u, 0u, 0u};

static bool cmd_eq(const char *buf, size_t len, const char *cmd) {
    size_t n = strlen(cmd);
    return (len == n) && (strncmp(buf, cmd, n) == 0);
}

static bool cmd_starts_with(const char *buf, size_t len, const char *prefix) {
    size_t n = strlen(prefix);
    return (len >= n) && (strncmp(buf, prefix, n) == 0);
}

static void set_reply(char *reply, size_t size, size_t *reply_len, const char *text) {
    if ((reply == NULL) || (reply_len == NULL)) {
        if (reply_len != NULL) {
            *reply_len = 0u;
        }
        return;
    }
    if (size == 0u) {
        *reply_len = 0u;
        return;
    }
    if (text == NULL) {
        text = "";
    }
    int written = snprintf(reply, size, "%s", text);
    if (written < 0) {
        written = 0;
    }
    size_t w = (size_t)written;
    if (w >= size) {
        w = size - 1u;
    }
    reply[w] = '\0';
    *reply_len = w;
}

static bool valid_equatorial_coord(const os_equatorial_coord_t *c) {
    if (c == NULL) {
        return false;
    }
    if (!isfinite((double)c->ra_hours) || !isfinite((double)c->dec_degrees)) {
        return false;
    }
    return (c->ra_hours >= OS_RA_MIN_HOURS) && (c->ra_hours <= OS_RA_MAX_HOURS)
        && (c->dec_degrees >= OS_DEC_MIN_DEG) && (c->dec_degrees <= OS_DEC_MAX_DEG);
}

static uint32_t nvm_fnv1a(const uint8_t *data, uint16_t length) {
    uint32_t hash = 2166136261u;
    uint16_t i;
    for (i = 0u; i < length; ++i) {
        hash ^= (uint32_t)data[i];
        hash *= 16777619u;
    }
    return hash;
}

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_len;
    uint32_t crc;
} os_nvm_record_header_t;

static uint8_t s_nvm_write_buf[OS_NVM_MAX_RECORD_TOTAL_BYTES];
static uint8_t s_nvm_read_buf[OS_NVM_MAX_RECORD_TOTAL_BYTES];

static os_error_t nvm_write_record(uint16_t offset, uint32_t magic,
                                   const void *payload, uint16_t payload_len) {
    if (payload == NULL || payload_len > (OS_NVM_MAX_RECORD_TOTAL_BYTES - sizeof(os_nvm_record_header_t))) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    uint16_t total = (uint16_t)(sizeof(os_nvm_record_header_t) + payload_len);
    if (((uint32_t)offset + (uint32_t)total) > (uint32_t)OS_NVM_TOTAL_CAPACITY) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    os_nvm_record_header_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.magic = magic;
    hdr.version = 1u;
    hdr.payload_len = payload_len;
    hdr.crc = 0u;
    memcpy(s_nvm_write_buf, &hdr, sizeof(hdr));
    if (payload_len > 0u) {
        memcpy(s_nvm_write_buf + sizeof(hdr), payload, payload_len);
    }
    uint32_t crc = nvm_fnv1a(s_nvm_write_buf, total);
    memcpy(s_nvm_write_buf + offsetof(os_nvm_record_header_t, crc), &crc, sizeof(crc));
    return os_hal_nvm_write(offset, s_nvm_write_buf, total);
}

static os_error_t nvm_read_record(uint16_t offset, uint32_t expected_magic,
                                  void *payload, uint16_t max_payload_len,
                                  uint16_t *actual_payload_len) {
    if (payload == NULL || max_payload_len == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    os_error_t err = os_hal_nvm_read(offset, s_nvm_read_buf, (uint16_t)sizeof(os_nvm_record_header_t));
    if (err != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    os_nvm_record_header_t hdr;
    memcpy(&hdr, s_nvm_read_buf, sizeof(hdr));
    if (hdr.magic != expected_magic || hdr.version != 1u || hdr.payload_len > max_payload_len) {
        return OS_ERR_NVM_FAULT;
    }
    uint16_t total = (uint16_t)(sizeof(os_nvm_record_header_t) + hdr.payload_len);
    if (total > OS_NVM_MAX_RECORD_TOTAL_BYTES
        || ((uint32_t)offset + (uint32_t)total) > (uint32_t)OS_NVM_TOTAL_CAPACITY) {
        return OS_ERR_NVM_FAULT;
    }
    err = os_hal_nvm_read(offset, s_nvm_read_buf, total);
    if (err != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    uint32_t stored_crc;
    memcpy(&stored_crc, s_nvm_read_buf + offsetof(os_nvm_record_header_t, crc), sizeof(stored_crc));
    uint32_t zero_crc = 0u;
    memcpy(s_nvm_read_buf + offsetof(os_nvm_record_header_t, crc), &zero_crc, sizeof(zero_crc));
    uint32_t computed_crc = nvm_fnv1a(s_nvm_read_buf, total);
    if (computed_crc != stored_crc) {
        return OS_ERR_NVM_FAULT;
    }
    memcpy(payload, s_nvm_read_buf + sizeof(os_nvm_record_header_t), hdr.payload_len);
    if (actual_payload_len != NULL) {
        *actual_payload_len = hdr.payload_len;
    }
    return OS_ERR_NONE;
}

static void load_persisted_state(void) {
    os_calibration_t cal;
    uint16_t actual = 0u;
    memset(&cal, 0, sizeof(cal));
    if (nvm_read_record(OS_NVM_CAL_OFFSET, OS_NVM_MAGIC_CALIBRATION,
                        &cal, (uint16_t)sizeof(cal), &actual) == OS_ERR_NONE
        && actual == sizeof(cal)
        && cal.valid
        && isfinite((double)cal.matrix_ra_to_ra)
        && isfinite((double)cal.matrix_ra_to_dec)
        && isfinite((double)cal.matrix_dec_to_ra)
        && isfinite((double)cal.matrix_dec_to_dec)
        && isfinite((double)cal.offset_ra_arcsec)
        && isfinite((double)cal.offset_dec_arcsec)) {
        s_calibration = cal;
    }

    os_pec_table_t pec;
    memset(&pec, 0, sizeof(pec));
    if (nvm_read_record(OS_NVM_PEC_OFFSET, OS_NVM_MAGIC_PEC,
                        &pec, (uint16_t)sizeof(pec), &actual) == OS_ERR_NONE
        && actual == sizeof(pec)) {
        s_pec_table = pec;
        if (!pec.valid) {
            s_pec_enabled = false;
        }
    }

    os_site_info_t site;
    memset(&site, 0, sizeof(site));
    if (nvm_read_record(OS_NVM_SITE_OFFSET, OS_NVM_MAGIC_SITE,
                        &site, (uint16_t)sizeof(site), &actual) == OS_ERR_NONE
        && actual == sizeof(site)) {
        s_site = site;
    }

    os_equatorial_coord_t park;
    memset(&park, 0, sizeof(park));
    if (nvm_read_record(OS_NVM_PARK_OFFSET, OS_NVM_MAGIC_PARK,
                        &park, (uint16_t)sizeof(park), &actual) == OS_ERR_NONE
        && actual == sizeof(park)
        && valid_equatorial_coord(&park)) {
        s_park_position = park;
        s_park_position_valid = true;
    }

    os_motor_position_t motor;
    memset(&motor, 0, sizeof(motor));
    if (nvm_read_record(OS_NVM_MOTOR_OFFSET, OS_NVM_MAGIC_MOTOR,
                        &motor, (uint16_t)sizeof(motor), &actual) == OS_ERR_NONE
        && actual == sizeof(motor)) {
        s_motor_steps[0] = motor.ra_steps;
        s_motor_steps[1] = motor.dec_steps;
    }
}

static void persist_motor_position(void) {
    os_motor_position_t pos;
    pos.ra_steps = s_motor_steps[0];
    pos.dec_steps = s_motor_steps[1];
    (void)nvm_write_record(OS_NVM_MOTOR_OFFSET, OS_NVM_MAGIC_MOTOR,
                           &pos, (uint16_t)sizeof(pos));
}

static void stop_all_motor_frequencies(void) {
    (void)os_hal_motor_set_frequency(0u, 0u);
    (void)os_hal_motor_set_frequency(1u, 0u);
}

static double compute_lst_hours(uint32_t utc_epoch, double longitude_deg) {
    double jd = (double)utc_epoch / 86400.0 + 2440587.5;
    double t = (jd - 2451545.0) / 36525.0;
    double gmst_deg = 280.46061837
                      + 360.98564736629 * (jd - 2451545.0)
                      + 0.000387933 * t * t
                      - t * t * t / 38710000.0;
    gmst_deg = fmod(gmst_deg, 360.0);
    if (gmst_deg < 0.0) {
        gmst_deg += 360.0;
    }
    double lst_hours = fmod(gmst_deg + longitude_deg, 360.0) / 15.0;
    if (lst_hours < 0.0) {
        lst_hours += 24.0;
    }
    return lst_hours;
}

static void compute_altaz_rates_arcsec_per_sec(const os_site_info_t *site,
                                                const os_equatorial_coord_t *ref,
                                                double *az_rate,
                                                double *alt_rate) {
    if ((site == NULL) || (ref == NULL) || (az_rate == NULL) || (alt_rate == NULL)) {
        *az_rate = 0.0;
        *alt_rate = 0.0;
        return;
    }
    double lst_hours = compute_lst_hours(site->utc_epoch_seconds, (double)site->longitude_degrees);
    double ha_hours = fmod(lst_hours - (double)ref->ra_hours + 24.0, 24.0);
    double ha_deg = ha_hours * 15.0;
    if (ha_deg > 180.0) {
        ha_deg -= 360.0;
    }
    double lat_rad = (double)site->latitude_degrees * M_PI / 180.0;
    double dec_rad = (double)ref->dec_degrees * M_PI / 180.0;
    double ha_rad = ha_deg * M_PI / 180.0;
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_dec = sin(dec_rad);
    double cos_dec = cos(dec_rad);
    double sin_ha = sin(ha_rad);
    double cos_ha = cos(ha_rad);
    double sin_alt = sin_lat * sin_dec + cos_lat * cos_dec * cos_ha;
    if (sin_alt > 1.0) sin_alt = 1.0;
    if (sin_alt < -1.0) sin_alt = -1.0;
    double alt_rad = asin(sin_alt);
    double cos_alt = cos(alt_rad);
    if (fabs(cos_alt) < 1e-9) {
        cos_alt = 1e-9;
    }
    *alt_rate = -15.0 * cos_lat * cos_dec * sin_ha / cos_alt;
    double tan_dec = tan(dec_rad);
    double az_rad = atan2(sin_ha, cos_ha * sin_lat - tan_dec * cos_lat);
    *az_rate = 15.0 * (sin_lat + cos_lat * tan(alt_rad) * cos(az_rad));
}

static uint32_t tracking_frequency_hz_for_axis(uint8_t axis, bool *forward) {
    if (forward != NULL) {
        *forward = true;
    }
    if (!s_tracking_enabled) {
        return 0u;
    }
    double factor = (double)s_tracking_rate_factor;
    if (!isfinite(factor) || factor < 0.0) {
        factor = 1.0;
    }
    double base_arcsec = OS_SIDEREAL_RATE_ARCSEC_PER_SEC * factor;
    double hz = 0.0;

    if (s_mount_type == OS_MOUNT_EQUATORIAL) {
        if (axis == 0u) {
            hz = base_arcsec * OS_TRACKING_STEPS_PER_ARCSEC;
            if (forward != NULL) {
                *forward = true;
            }
        } else {
            hz = 0.0;
        }
    } else {
        double az_rate = 0.0;
        double alt_rate = 0.0;
        compute_altaz_rates_arcsec_per_sec(&s_site, &s_tracking_ref_coord, &az_rate, &alt_rate);
        double rate = (axis == 0u) ? az_rate : alt_rate;
        hz = rate * OS_TRACKING_STEPS_PER_ARCSEC;
        if (forward != NULL) {
            *forward = (rate >= 0.0);
        }
    }

    if (hz < 0.0) {
        hz = -hz;
    }
    if (hz > (double)OS_MOTOR_MAX_FREQ_HZ) {
        hz = (double)OS_MOTOR_MAX_FREQ_HZ;
    }
    return (uint32_t)hz;
}

static void apply_axis_rate(uint8_t axis, uint32_t hz, bool forward) {
    if (hz > 0u) {
        if (!s_hal_motor_enabled[axis]) {
            (void)os_hal_motor_enable(axis, true);
        }
        (void)os_hal_motor_set_direction(axis, forward);
    }
    (void)os_hal_motor_set_frequency(axis, hz);
}

static void apply_tracking_motor_rate(void) {
    if ((s_state == OS_STATE_PARKED) || (s_state == OS_STATE_FAULT)) {
        stop_all_motor_frequencies();
        return;
    }
    if (s_goto_active || s_park_active || s_manual_active || s_guide_pulse.active) {
        return;
    }

    bool fwd0 = true;
    bool fwd1 = true;
    uint32_t hz0 = s_tracking_enabled ? tracking_frequency_hz_for_axis(0u, &fwd0) : 0u;
    uint32_t hz1 = 0u;
    if ((s_mount_type == OS_MOUNT_ALTAZ) && s_tracking_enabled) {
        hz1 = tracking_frequency_hz_for_axis(1u, &fwd1);
    }
    apply_axis_rate(0u, hz0, fwd0);
    apply_axis_rate(1u, hz1, fwd1);
}

static uint32_t guide_reference_hz_for_axis(uint8_t axis) {
    bool forward = true;
    uint32_t hz = s_tracking_enabled ? tracking_frequency_hz_for_axis(axis, &forward) : 0u;
    if (hz == 0u) {
        double factor = (double)s_tracking_rate_factor;
        if (!isfinite(factor) || factor < 0.0) {
            factor = 1.0;
        }
        double ref_hz = OS_SIDEREAL_RATE_ARCSEC_PER_SEC * factor * OS_TRACKING_STEPS_PER_ARCSEC;
        if (ref_hz > (double)OS_MOTOR_MAX_FREQ_HZ) {
            ref_hz = (double)OS_MOTOR_MAX_FREQ_HZ;
        }
        hz = (uint32_t)ref_hz;
    }
    return hz;
}

static void apply_guide_motor_rate(void) {
    if ((s_state == OS_STATE_PARKED) || (s_state == OS_STATE_FAULT)) {
        stop_all_motor_frequencies();
        return;
    }
    if (!s_guide_pulse.active) {
        apply_tracking_motor_rate();
        return;
    }

    uint8_t axis = s_guide_pulse.dec_priority ? 1u : 0u;
    bool forward = s_guide_pulse.dec_priority
                       ? s_guide_pulse.direction_north
                       : s_guide_pulse.direction_east;
    bool base_forward = true;
    uint32_t base_hz = s_tracking_enabled ? tracking_frequency_hz_for_axis(axis, &base_forward) : 0u;
    uint32_t reference_hz = guide_reference_hz_for_axis(axis);
    float frac = s_guide_pulse.rate_fraction;
    if (!isfinite((double)frac) || frac < OS_GUIDE_RATE_MIN || frac > OS_GUIDE_RATE_MAX) {
        frac = 0.5f;
    }
    uint32_t bias_hz = (uint32_t)((double)reference_hz * (double)frac);
    uint32_t hz = base_hz + bias_hz;
    if (hz == 0u) {
        hz = (uint32_t)((double)reference_hz * (double)OS_GUIDE_RATE_MIN);
    }
    if (hz > OS_MOTOR_MAX_FREQ_HZ) {
        hz = OS_MOTOR_MAX_FREQ_HZ;
    }
    if (!s_hal_motor_enabled[axis]) {
        (void)os_hal_motor_enable(axis, true);
    }
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, hz);
}

static bool calibration_to_steps(const os_equatorial_coord_t *coord, int32_t steps[2]) {
    if ((coord == NULL) || (steps == NULL) || !s_calibration.valid) {
        return false;
    }
    double ra_arcsec = (double)coord->ra_hours * 15.0 * 3600.0;
    double dec_arcsec = (double)coord->dec_degrees * 3600.0;
    double m_rr = (double)s_calibration.matrix_ra_to_ra;
    double m_rd = (double)s_calibration.matrix_ra_to_dec;
    double m_dr = (double)s_calibration.matrix_dec_to_ra;
    double m_dd = (double)s_calibration.matrix_dec_to_dec;
    double off_ra = (double)s_calibration.offset_ra_arcsec;
    double off_dec = (double)s_calibration.offset_dec_arcsec;
    double ra_steps = off_ra + m_rr * ra_arcsec + m_rd * dec_arcsec;
    double dec_steps = off_dec + m_dr * ra_arcsec + m_dd * dec_arcsec;
    if (!isfinite(ra_steps) || !isfinite(dec_steps)) {
        return false;
    }
    if (ra_steps > 2147483647.0) ra_steps = 2147483647.0;
    if (ra_steps < -2147483648.0) ra_steps = -2147483648.0;
    if (dec_steps > 2147483647.0) dec_steps = 2147483647.0;
    if (dec_steps < -2147483648.0) dec_steps = -2147483648.0;
    steps[0] = (int32_t)ra_steps;
    steps[1] = (int32_t)dec_steps;
    return true;
}

static bool steps_to_coordinates(int32_t ra_steps, int32_t dec_steps, os_equatorial_coord_t *coord) {
    if ((coord == NULL) || !s_calibration.valid) {
        return false;
    }
    double m_rr = (double)s_calibration.matrix_ra_to_ra;
    double m_rd = (double)s_calibration.matrix_ra_to_dec;
    double m_dr = (double)s_calibration.matrix_dec_to_ra;
    double m_dd = (double)s_calibration.matrix_dec_to_dec;
    double det = m_rr * m_dd - m_rd * m_dr;
    if (fabs(det) < 1e-12) {
        return false;
    }
    double ra_c = (double)ra_steps - (double)s_calibration.offset_ra_arcsec;
    double dec_c = (double)dec_steps - (double)s_calibration.offset_dec_arcsec;
    double ra_arcsec = ( m_dd * ra_c - m_rd * dec_c) / det;
    double dec_arcsec = (-m_dr * ra_c + m_rr * dec_c) / det;
    coord->ra_hours = (float)(ra_arcsec / (15.0 * 3600.0));
    coord->dec_degrees = (float)(dec_arcsec / 3600.0);
    return true;
}

static bool solve_two(double x0, double x1, double y0, double y1,
                      double *slope, double *offset) {
    if ((slope == NULL) || (offset == NULL) || (fabs(x1 - x0) < 1e-12)) {
        return false;
    }
    *slope = (y1 - y0) / (x1 - x0);
    *offset = y0 - (*slope) * x0;
    return true;
}

static bool solve_full_affine(const double *x, const double *z, const double *y, int n,
                              double coef[3], double *det_out) {
    if ((x == NULL) || (z == NULL) || (y == NULL) || (coef == NULL) || (n < 3)) {
        return false;
    }
    double A[OS_CALIBRATION_MAX_STARS][3];
    double Q[OS_CALIBRATION_MAX_STARS][3];
    double R[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double c[3] = {0.0, 0.0, 0.0};
    int row, k, j;

    for (row = 0; row < n; ++row) {
        A[row][0] = 1.0;
        A[row][1] = x[row];
        A[row][2] = z[row];
    }

    if (n == 3) {
        double det = (x[1] - x[0]) * (z[2] - z[0]) - (x[2] - x[0]) * (z[1] - z[0]);
        double max_abs = 0.0;
        for (row = 0; row < n; ++row) {
            double ax = fabs(x[row]);
            double az = fabs(z[row]);
            if (ax > max_abs) max_abs = ax;
            if (az > max_abs) max_abs = az;
        }
        if (max_abs < 1e-12) {
            return false;
        }
        double normalized_det = det / (max_abs * max_abs);
        if (det_out != NULL) {
            *det_out = normalized_det;
        }
        if (fabs(normalized_det) < 1e-9) {
            return false;
        }
    }

    for (k = 0; k < 3; ++k) {
        double norm = 0.0;
        for (row = 0; row < n; ++row) {
            norm += A[row][k] * A[row][k];
        }
        norm = sqrt(norm);
        if (norm < 1e-12) {
            return false;
        }
        R[k][k] = norm;
        for (row = 0; row < n; ++row) {
            Q[row][k] = A[row][k] / norm;
        }
        for (j = k + 1; j < 3; ++j) {
            double sum = 0.0;
            for (row = 0; row < n; ++row) {
                sum += Q[row][k] * A[row][j];
            }
            R[k][j] = sum;
            for (row = 0; row < n; ++row) {
                A[row][j] -= R[k][j] * Q[row][k];
            }
        }
    }

    for (k = 0; k < 3; ++k) {
        for (row = 0; row < n; ++row) {
            c[k] += Q[row][k] * y[row];
        }
    }

    coef[2] = c[2] / R[2][2];
    coef[1] = (c[1] - R[1][2] * coef[2]) / R[1][1];
    coef[0] = (c[0] - R[0][1] * coef[1] - R[0][2] * coef[2]) / R[0][0];
    return true;
}

static int64_t traj_decel_distance_steps(int64_t speed) {
    return (speed * speed) / (2 * (int64_t)OS_TRAJ_DECEL_STEPS_PER_LOOP);
}

static void initiate_async_motion(void) {
    for (int axis = 0; axis < 2; ++axis) {
        if (s_goto_target_steps[axis] == s_motor_steps[axis]) {
            s_traj_phase[axis] = TRAJ_PHASE_IDLE;
            s_traj_speed_steps[axis] = 0;
        } else {
            s_traj_phase[axis] = TRAJ_PHASE_ACCEL;
            s_traj_speed_steps[axis] = OS_TRAJ_MIN_SPEED_STEPS_PER_LOOP;
        }
    }
    (void)os_hal_motor_enable(0u, true);
    (void)os_hal_motor_enable(1u, true);
    stop_all_motor_frequencies();
}

static void advance_async_motion(void) {
    if (!s_goto_active && !s_park_active) {
        return;
    }

    bool all_reached = true;
    int axis;
    for (axis = 0; axis < 2; ++axis) {
        int32_t current = s_motor_steps[axis];
        int32_t target = s_goto_target_steps[axis];
        int64_t delta = (int64_t)target - (int64_t)current;
        int64_t remaining = (delta < 0) ? -delta : delta;
        if (remaining == 0) {
            s_traj_phase[axis] = TRAJ_PHASE_IDLE;
            s_traj_speed_steps[axis] = 0;
            (void)os_hal_motor_set_frequency((uint8_t)axis, 0u);
            continue;
        }
        all_reached = false;

        int64_t speed = s_traj_speed_steps[axis];
        if (speed < OS_TRAJ_MIN_SPEED_STEPS_PER_LOOP) {
            speed = OS_TRAJ_MIN_SPEED_STEPS_PER_LOOP;
        }

        if (remaining <= OS_TRAJ_LOW_DISTANCE_STEPS) {
            s_traj_phase[axis] = TRAJ_PHASE_LOW;
            speed = OS_TRAJ_LOW_SPEED_STEPS_PER_LOOP;
        } else {
            switch (s_traj_phase[axis]) {
                case TRAJ_PHASE_ACCEL:
                    speed += OS_TRAJ_ACCEL_STEPS_PER_LOOP;
                    if (speed >= OS_TRAJ_MAX_SPEED_STEPS_PER_LOOP) {
                        speed = OS_TRAJ_MAX_SPEED_STEPS_PER_LOOP;
                        s_traj_phase[axis] = TRAJ_PHASE_CRUISE;
                    }
                    break;
                case TRAJ_PHASE_CRUISE:
                    speed = OS_TRAJ_MAX_SPEED_STEPS_PER_LOOP;
                    if (remaining <= (traj_decel_distance_steps(speed) + OS_TRAJ_LOW_DISTANCE_STEPS)) {
                        s_traj_phase[axis] = TRAJ_PHASE_DECEL;
                    }
                    break;
                case TRAJ_PHASE_DECEL:
                    speed -= OS_TRAJ_DECEL_STEPS_PER_LOOP;
                    if (speed <= OS_TRAJ_LOW_SPEED_STEPS_PER_LOOP) {
                        speed = OS_TRAJ_LOW_SPEED_STEPS_PER_LOOP;
                        s_traj_phase[axis] = TRAJ_PHASE_LOW;
                    }
                    break;
                case TRAJ_PHASE_LOW:
                    speed = OS_TRAJ_LOW_SPEED_STEPS_PER_LOOP;
                    break;
                default:
                    s_traj_phase[axis] = TRAJ_PHASE_IDLE;
                    speed = remaining;
                    break;
            }
        }

        if (speed > remaining) {
            speed = remaining;
        }
        if (speed < 1) {
            speed = 1;
        }
        s_traj_speed_steps[axis] = (int32_t)speed;

        bool forward = (delta > 0);
        if (!s_hal_motor_enabled[axis]) {
            (void)os_hal_motor_enable((uint8_t)axis, true);
        }
        (void)os_hal_motor_set_direction((uint8_t)axis, forward);
        (void)os_hal_motor_set_frequency((uint8_t)axis, (uint32_t)(speed * (int64_t)OS_TRAJ_HZ_PER_STEP));

        if (forward) {
            s_motor_steps[axis] += (int32_t)speed;
        } else {
            s_motor_steps[axis] -= (int32_t)speed;
        }
    }

    if (all_reached) {
        stop_all_motor_frequencies();
        for (axis = 0; axis < 2; ++axis) {
            s_traj_phase[axis] = TRAJ_PHASE_IDLE;
            s_traj_speed_steps[axis] = 0;
        }
        if (s_park_active) {
            s_park_active = false;
            s_state = OS_STATE_PARKED;
            (void)os_tracking_disable();
            (void)os_hal_motor_enable(0u, false);
            (void)os_hal_motor_enable(1u, false);
        } else {
            s_goto_active = false;
            s_state = OS_STATE_IDLE_TRACKING;
            if (s_goto_resume_tracking) {
                (void)os_tracking_enable();
            }
        }
        persist_motor_position();
        (void)os_hal_buzzer_beep(120u, 1u);
    }
}

static void advance_motor_positions_from_frequency(void) {
    if (s_goto_active || s_park_active) {
        return;
    }
    int axis;
    for (axis = 0; axis < 2; ++axis) {
        uint32_t hz = s_hal_motor_frequency[axis];
        if (!s_hal_motor_enabled[axis] || (hz == 0u)) {
            continue;
        }
        uint32_t whole = hz / 1000u;
        uint32_t frac = hz % 1000u;
        s_motor_substep[axis] += frac;
        int32_t delta = (int32_t)whole;
        if (s_motor_substep[axis] >= 1000u) {
            delta += 1;
            s_motor_substep[axis] -= 1000u;
        }
        if (delta > 0) {
            if (s_hal_motor_direction[axis]) {
                s_motor_steps[axis] += delta;
            } else {
                s_motor_steps[axis] -= delta;
            }
        }
    }
}

static bool parse_ra_body(const char *command, size_t length, float *ra_hours) {
    if ((command == NULL) || (ra_hours == NULL) || (length < 4u)) {
        return false;
    }
    size_t start = 3u;
    size_t end = length - 1u;
    if (end <= start) {
        return false;
    }
    size_t n = end - start;
    char tmp[24];
    if (n >= sizeof(tmp)) {
        return false;
    }
    memcpy(tmp, command + start, n);
    tmp[n] = '\0';
    int colons = 0;
    size_t i;
    for (i = 0; i < n; ++i) {
        if (tmp[i] == ':') {
            colons++;
        }
    }
    double hh = 0.0, mm = 0.0, ss = 0.0;
    int used = 0;
    if (colons == 2) {
        used = sscanf(tmp, "%lf:%lf:%lf", &hh, &mm, &ss);
        if (used == 3) {
            *ra_hours = (float)(hh + mm / 60.0 + ss / 3600.0);
        }
    } else if (colons == 1) {
        used = sscanf(tmp, "%lf:%lf", &hh, &mm);
        if (used == 2) {
            *ra_hours = (float)(hh + mm / 60.0);
        }
    } else {
        char *endptr = NULL;
        double value = strtod(tmp, &endptr);
        if ((endptr != tmp) && (*endptr == '\0')) {
            used = 1;
            *ra_hours = (float)value;
        }
    }
    if (used == 0) {
        return false;
    }
    return isfinite((double)*ra_hours) && (*ra_hours >= OS_RA_MIN_HOURS) && (*ra_hours <= OS_RA_MAX_HOURS);
}

static bool parse_dec_body(const char *command, size_t length, float *dec_degrees) {
    if ((command == NULL) || (dec_degrees == NULL) || (length < 4u)) {
        return false;
    }
    size_t start = 3u;
    size_t end = length - 1u;
    if (end <= start) {
        return false;
    }
    size_t n = end - start;
    char tmp[24];
    if (n >= sizeof(tmp)) {
        return false;
    }
    memcpy(tmp, command + start, n);
    tmp[n] = '\0';
    size_t i;
    for (i = 0; i < n; ++i) {
        if (tmp[i] == '*') {
            tmp[i] = ':';
        }
    }
    int colons = 0;
    for (i = 0; i < n; ++i) {
        if (tmp[i] == ':') {
            colons++;
        }
    }
    double dd = 0.0, mm = 0.0, ss = 0.0;
    int used = 0;
    if (colons == 2) {
        used = sscanf(tmp, "%lf:%lf:%lf", &dd, &mm, &ss);
        if (used == 3) {
            int sign = (dd < 0) ? -1 : 1;
            double abs_dd = fabs(dd);
            *dec_degrees = (float)(sign * (abs_dd + mm / 60.0 + ss / 3600.0));
        }
    } else if (colons == 1) {
        used = sscanf(tmp, "%lf:%lf", &dd, &mm);
        if (used == 2) {
            int sign = (dd < 0) ? -1 : 1;
            double abs_dd = fabs(dd);
            *dec_degrees = (float)(sign * (abs_dd + mm / 60.0));
        }
    } else {
        char *endptr = NULL;
        double value = strtod(tmp, &endptr);
        if ((endptr != tmp) && (*endptr == '\0')) {
            used = 1;
            *dec_degrees = (float)value;
        }
    }
    if (used == 0) {
        return false;
    }
    return isfinite((double)*dec_degrees) && (*dec_degrees >= OS_DEC_MIN_DEG) && (*dec_degrees <= OS_DEC_MAX_DEG);
}

os_error_t os_init(void) {
    s_state = OS_STATE_INITIALIZING;
    s_tracking_enabled = false;
    s_tracking_rate = OS_TRACK_RATE_SIDEREAL;
    s_tracking_custom_factor = 1.0f;
    s_tracking_rate_factor = 1.0f;
    s_tracking_ref_coord.ra_hours = 0.0f;
    s_tracking_ref_coord.dec_degrees = 0.0f;

    memset(&s_calibration, 0, sizeof(s_calibration));
    s_residual_valid = false;
    s_residual_arcsec = 0.0f;

    s_goto_active = false;
    s_goto_resume_tracking = true;
    s_goto_target_steps[0] = 0;
    s_goto_target_steps[1] = 0;
    s_traj_phase[0] = TRAJ_PHASE_IDLE;
    s_traj_phase[1] = TRAJ_PHASE_IDLE;
    s_traj_speed_steps[0] = 0;
    s_traj_speed_steps[1] = 0;

    s_park_active = false;
    s_park_resume_tracking = true;
    memset(&s_park_position, 0, sizeof(s_park_position));
    s_park_position_valid = false;

    s_manual_active = false;
    s_manual_direction = OS_DIRECTION_NORTH;
    s_manual_speed = OS_SPEED_SLOW;
    s_manual_custom_speed_arcsec_per_sec = 100.0f;

    memset(&s_guide_pulse, 0, sizeof(s_guide_pulse));
    s_guide_pulse.rate_fraction = 0.5f;
    s_guide_start_ms = 0u;

    s_alignment_active = false;
    s_alignment_mode = OS_ALIGN_1STAR;
    s_alignment_star_count = 0u;
    memset(s_alignment_stars, 0, sizeof(s_alignment_stars));
    memset(s_alignment_motor_pos, 0, sizeof(s_alignment_motor_pos));

    memset(&s_pec_table, 0, sizeof(s_pec_table));
    s_pec_enabled = false;

    memset(&s_site, 0, sizeof(s_site));
    s_gps_locked = false;
    s_system_tick_ms = 0u;

    s_motor_steps[0] = 0;
    s_motor_steps[1] = 0;
    s_motor_substep[0] = 0u;
    s_motor_substep[1] = 0u;

    os_error_t err = os_hal_nvm_init();
    if (err != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return OS_ERR_NVM_FAULT;
    }

    (void)os_hal_comm_init(OS_CHANNEL_USB);
    (void)os_hal_comm_init(OS_CHANNEL_BLUETOOTH);
    (void)os_hal_comm_init(OS_CHANNEL_WIFI);
    (void)os_hal_comm_init(OS_CHANNEL_ETHERNET);

    err = os_hal_motor_init(0u);
    if (err != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    err = os_hal_motor_init(1u);
    if (err != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    err = os_hal_timer_motor_init();
    if (err != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    (void)os_hal_motor_enable(0u, true);
    (void)os_hal_motor_enable(1u, true);

    load_persisted_state();

    if (s_calibration.valid) {
        os_equatorial_coord_t recovered;
        memset(&recovered, 0, sizeof(recovered));
        if (steps_to_coordinates(s_motor_steps[0], s_motor_steps[1], &recovered)) {
            s_tracking_ref_coord = recovered;
        }
    }

    os_site_info_t gps_site;
    memset(&gps_site, 0, sizeof(gps_site));
    err = os_hal_gps_poll(&gps_site);
    if ((err == OS_ERR_NONE) && gps_site.valid) {
        s_site = gps_site;
        s_gps_locked = true;
        (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
    } else {
        s_gps_locked = false;
        uint32_t rtc_time = 0u;
        if (os_hal_rtc_read(&rtc_time) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = rtc_time;
        }
    }

    s_tracking_rate = OS_TRACK_RATE_SIDEREAL;
    s_tracking_custom_factor = 1.0f;
    s_tracking_rate_factor = 1.0f;
    s_tracking_enabled = true;
    s_state = OS_STATE_IDLE_TRACKING;
    apply_tracking_motor_rate();
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    ++s_system_tick_ms;

    uint8_t ch;
    for (ch = 0u; ch < 4u; ++ch) {
        int16_t avail = os_hal_comm_available(ch);
        if (avail < 0) {
            avail = 0;
        }
        while (avail-- > 0) {
            char byte = os_hal_comm_read(ch);
            if (byte == '\r') {
                continue;
            }
            if ((byte == '\n') || (byte == OS_LX200_CMD_SUFFIX)) {
                if ((s_loop_command_len[ch] > 0u) && (s_loop_command_len[ch] < OS_MAX_COMMAND_LENGTH)) {
                    s_loop_command_buffer[ch][s_loop_command_len[ch]] = byte;
                    s_loop_command_len[ch] += 1u;
                    s_loop_command_buffer[ch][s_loop_command_len[ch]] = '\0';

                    char reply[OS_MAX_REPLY_LENGTH];
                    size_t reply_len = 0u;
                    (void)os_command_parse(s_loop_command_buffer[ch], s_loop_command_len[ch],
                                           ch, reply, sizeof(reply), &reply_len);
                    if (reply_len > 0u) {
                        (void)os_hal_comm_write(ch, reply, reply_len);
                    }
                }
                s_loop_command_len[ch] = 0u;
                continue;
            }

            if (s_loop_command_len[ch] < OS_MAX_COMMAND_LENGTH) {
                s_loop_command_buffer[ch][s_loop_command_len[ch]] = byte;
                s_loop_command_len[ch] += 1u;
            } else {
                s_loop_command_len[ch] = 0u;
            }
        }
    }

    os_site_info_t gps_site;
    memset(&gps_site, 0, sizeof(gps_site));
    if ((os_hal_gps_poll(&gps_site) == OS_ERR_NONE) && gps_site.valid) {
        s_site = gps_site;
        s_gps_locked = true;
        (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
    } else {
        s_gps_locked = false;
        uint32_t rtc_now = 0u;
        if (os_hal_rtc_read(&rtc_now) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = rtc_now;
        }
    }

    bool limit0 = os_hal_limit_is_triggered(0u);
    bool limit1 = os_hal_limit_is_triggered(1u);
    if (limit0 || limit1) {
        stop_all_motor_frequencies();
        s_goto_active = false;
        s_park_active = false;
        s_manual_active = false;
        s_guide_pulse.active = false;
        s_state = OS_STATE_FAULT;
        (void)os_hal_buzzer_beep(200u, 2u);
        persist_motor_position();
        return;
    }

    advance_async_motion();

    if (s_manual_active && (s_state == OS_STATE_MANUAL_MOTION)) {
        uint8_t axis = ((s_manual_direction == OS_DIRECTION_NORTH) || (s_manual_direction == OS_DIRECTION_SOUTH)) ? 1u : 0u;
        bool forward = (s_manual_direction == OS_DIRECTION_NORTH) || (s_manual_direction == OS_DIRECTION_EAST);
        if (!s_hal_motor_enabled[axis]) {
            (void)os_hal_motor_enable(axis, true);
        }
        (void)os_hal_motor_set_direction(axis, forward);
        (void)os_hal_motor_set_frequency(axis, s_manual_speed == OS_SPEED_SLOW ? 80u
                                                       : s_manual_speed == OS_SPEED_MEDIUM ? 400u
                                                       : s_manual_speed == OS_SPEED_FAST ? 1200u
                                                       : (uint32_t)(s_manual_custom_speed_arcsec_per_sec * OS_MANUAL_STEPS_PER_ARCSEC));
    }

    if (s_guide_pulse.active) {
        apply_guide_motor_rate();
    }

    advance_motor_positions_from_frequency();
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (reply_length != NULL) {
        *reply_length = 0u;
    }
    if ((command == NULL) || (reply_buffer == NULL) || (reply_length == NULL)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (source_channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((length < 2u) || (command[0] != OS_LX200_CMD_PREFIX)
        || (command[length - 1u] != OS_LX200_CMD_SUFFIX)) {
        set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
        return OS_ERR_COMMAND_FORMAT;
    }

    if (cmd_eq(command, length, ":GVP#")) {
        uint8_t major = OS_FIRMWARE_VERSION_MAJOR;
        uint8_t minor = OS_FIRMWARE_VERSION_MINOR;
        uint8_t patch = OS_FIRMWARE_VERSION_PATCH;
        if (reply_buffer_size > 0u) {
            int written = snprintf(reply_buffer, reply_buffer_size, "%u.%u.%u",
                                   (unsigned)major, (unsigned)minor, (unsigned)patch);
            if (written < 0) written = 0;
            size_t w = (size_t)written;
            if (w >= reply_buffer_size) w = reply_buffer_size - 1u;
            reply_buffer[w] = '\0';
            *reply_length = w;
        }
        return OS_ERR_NONE;
    }

    if (cmd_eq(command, length, ":GR#")) {
        os_equatorial_coord_t coord;
        memset(&coord, 0, sizeof(coord));
        if (os_query_coordinates(&coord) == OS_ERR_NONE) {
            if (reply_buffer_size > 0u) {
                int written = snprintf(reply_buffer, reply_buffer_size, "%+.6f", (double)coord.ra_hours);
                if (written < 0) written = 0;
                size_t w = (size_t)written;
                if (w >= reply_buffer_size) w = reply_buffer_size - 1u;
                reply_buffer[w] = '\0';
                *reply_length = w;
            }
        } else {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
        }
        return OS_ERR_NONE;
    }

    if (cmd_eq(command, length, ":GD#")) {
        os_equatorial_coord_t coord;
        memset(&coord, 0, sizeof(coord));
        if (os_query_coordinates(&coord) == OS_ERR_NONE) {
            if (reply_buffer_size > 0u) {
                int written = snprintf(reply_buffer, reply_buffer_size, "%+.6f", (double)coord.dec_degrees);
                if (written < 0) written = 0;
                size_t w = (size_t)written;
                if (w >= reply_buffer_size) w = reply_buffer_size - 1u;
                reply_buffer[w] = '\0';
                *reply_length = w;
            }
        } else {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
        }
        return OS_ERR_NONE;
    }

    if (cmd_eq(command, length, ":Me#")) {
        os_error_t e = os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
        set_reply(reply_buffer, reply_buffer_size, reply_length, (e == OS_ERR_NONE) ? "1" : "0");
        return e;
    }
    if (cmd_eq(command, length, ":Mw#")) {
        os_error_t e = os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
        set_reply(reply_buffer, reply_buffer_size, reply_length, (e == OS_ERR_NONE) ? "1" : "0");
        return e;
    }
    if (cmd_eq(command, length, ":Mn#")) {
        os_error_t e = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
        set_reply(reply_buffer, reply_buffer_size, reply_length, (e == OS_ERR_NONE) ? "1" : "0");
        return e;
    }
    if (cmd_eq(command, length, ":Ms#")) {
        os_error_t e = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
        set_reply(reply_buffer, reply_buffer_size, reply_length, (e == OS_ERR_NONE) ? "1" : "0");
        return e;
    }
    if (cmd_eq(command, length, ":Q#")) {
        os_error_t e = os_move_stop();
        set_reply(reply_buffer, reply_buffer_size, reply_length, (e == OS_ERR_NONE) ? "1" : "0");
        return e;
    }
    if (cmd_eq(command, length, ":hP#")) {
        os_error_t e = os_park();
        set_reply(reply_buffer, reply_buffer_size, reply_length, (e == OS_ERR_NONE) ? "1" : "0");
        return e;
    }
    if (cmd_eq(command, length, ":hO#")) {
        os_error_t e = os_unpark();
        set_reply(reply_buffer, reply_buffer_size, reply_length, (e == OS_ERR_NONE) ? "1" : "0");
        return e;
    }

    if (cmd_eq(command, length, ":MS#")) {
        os_error_t e = os_goto_equatorial(s_goto_command_target);
        set_reply(reply_buffer, reply_buffer_size, reply_length, (e == OS_ERR_NONE) ? "1" : "0");
        return e;
    }

    if (cmd_starts_with(command, length, ":Sr")) {
        float ra = 0.0f;
        if (!parse_ra_body(command, length, &ra)) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
            return OS_ERR_COMMAND_FORMAT;
        }
        s_goto_command_target.ra_hours = ra;
        os_error_t e = os_goto_equatorial(s_goto_command_target);
        set_reply(reply_buffer, reply_buffer_size, reply_length, (e == OS_ERR_NONE) ? "1" : "0");
        return e;
    }

    if (cmd_starts_with(command, length, ":Sd")) {
        float dec = 0.0f;
        if (!parse_dec_body(command, length, &dec)) {
            set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
            return OS_ERR_COMMAND_FORMAT;
        }
        s_goto_command_target.dec_degrees = dec;
        set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        return OS_ERR_NONE;
    }

    set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
    return OS_ERR_NOT_SUPPORTED;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (!valid_equatorial_coord(&target)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((s_state == OS_STATE_PARKED) || (s_state == OS_STATE_FAULT)) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_goto_active || s_park_active || s_manual_active) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    if (!calibration_to_steps(&target, s_goto_target_steps)) {
        s_goto_target_steps[0] = (int32_t)(target.ra_hours * 100.0f);
        s_goto_target_steps[1] = (int32_t)(target.dec_degrees * 100.0f);
    }
    s_tracking_ref_coord = target;
    s_goto_active = true;
    s_goto_resume_tracking = s_tracking_enabled;
    s_park_active = false;
    s_manual_active = false;
    s_guide_pulse.active = false;
    s_state = OS_STATE_GOTO;
    initiate_async_motion();
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (!isfinite((double)target.azimuth_degrees) || !isfinite((double)target.altitude_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((target.azimuth_degrees < 0.0f) || (target.azimuth_degrees > 360.0f)
        || (target.altitude_degrees < -90.0f) || (target.altitude_degrees > 90.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((s_state == OS_STATE_PARKED) || (s_state == OS_STATE_FAULT)) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_goto_active || s_park_active || s_manual_active) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_goto_target_steps[0] = (int32_t)(target.azimuth_degrees * 100.0f);
    s_goto_target_steps[1] = (int32_t)(target.altitude_degrees * 100.0f);
    s_goto_active = true;
    s_goto_resume_tracking = s_tracking_enabled;
    s_park_active = false;
    s_manual_active = false;
    s_guide_pulse.active = false;
    s_state = OS_STATE_GOTO;
    initiate_async_motion();
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (s_goto_active || (s_state == OS_STATE_GOTO)) {
        s_goto_active = false;
        s_park_active = false;
        for (int axis = 0; axis < 2; ++axis) {
            s_traj_phase[axis] = TRAJ_PHASE_IDLE;
            s_traj_speed_steps[axis] = 0;
        }
        stop_all_motor_frequencies();
        if ((s_state != OS_STATE_PARKED) && (s_state != OS_STATE_FAULT)) {
            s_state = OS_STATE_IDLE_TRACKING;
            if (s_goto_resume_tracking || s_park_resume_tracking || s_tracking_enabled) {
                (void)os_tracking_enable();
            }
        }
        persist_motor_position();
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if ((rate < OS_TRACK_RATE_SIDEREAL) || (rate > OS_TRACK_RATE_CUSTOM)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM) {
        if (!isfinite((double)custom_factor) || (custom_factor <= 0.0f)) {
            return OS_ERR_INVALID_ARGUMENT;
        }
    }
    s_tracking_rate = rate;
    s_tracking_custom_factor = custom_factor;
    switch (rate) {
        case OS_TRACK_RATE_SIDEREAL:
            s_tracking_rate_factor = 1.0f;
            break;
        case OS_TRACK_RATE_LUNAR:
            s_tracking_rate_factor = OS_LUNAR_RATE_FACTOR;
            break;
        case OS_TRACK_RATE_SOLAR:
            s_tracking_rate_factor = OS_SOLAR_RATE_FACTOR;
            break;
        case OS_TRACK_RATE_CUSTOM:
            s_tracking_rate_factor = custom_factor;
            break;
        default:
            s_tracking_rate_factor = 1.0f;
            break;
    }
    apply_tracking_motor_rate();
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if ((rate == NULL) || (custom_factor == NULL)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = s_tracking_rate;
    *custom_factor = s_tracking_custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    if ((s_state == OS_STATE_PARKED) || (s_state == OS_STATE_FAULT)) {
        return OS_ERR_INVALID_STATE;
    }
    s_tracking_enabled = true;
    if ((s_state != OS_STATE_GOTO) && (s_state != OS_STATE_ALIGNMENT)
        && (s_state != OS_STATE_MANUAL_MOTION)) {
        s_state = OS_STATE_IDLE_TRACKING;
    }
    apply_tracking_motor_rate();
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    s_tracking_enabled = false;
    if (!s_goto_active && !s_park_active && !s_manual_active && !s_guide_pulse.active) {
        stop_all_motor_frequencies();
    }
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if ((direction < OS_DIRECTION_NORTH) || (direction > OS_DIRECTION_WEST)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((s_state == OS_STATE_PARKED) || (s_state == OS_STATE_FAULT) || (s_state == OS_STATE_INITIALIZING)) {
        return OS_ERR_INVALID_STATE;
    }

    bool new_is_dec = (direction == OS_DIRECTION_NORTH) || (direction == OS_DIRECTION_SOUTH);
    if (s_guide_pulse.active && s_guide_pulse.dec_priority && !new_is_dec) {
        return OS_ERR_NONE;
    }

    s_guide_pulse.active = true;
    s_guide_pulse.duration_ms = duration_ms;
    s_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    s_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    s_guide_pulse.dec_priority = new_is_dec;
    if (!isfinite((double)s_guide_pulse.rate_fraction)
        || s_guide_pulse.rate_fraction < OS_GUIDE_RATE_MIN
        || s_guide_pulse.rate_fraction > OS_GUIDE_RATE_MAX) {
        s_guide_pulse.rate_fraction = 0.5f;
    }
    s_guide_start_ms = s_system_tick_ms;
    apply_guide_motor_rate();
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (!isfinite((double)rate_fraction)
        || (rate_fraction < OS_GUIDE_RATE_MIN)
        || (rate_fraction > OS_GUIDE_RATE_MAX)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_guide_pulse.rate_fraction = rate_fraction;
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
    if ((s_state == OS_STATE_PARKED) || (s_state == OS_STATE_FAULT)) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_goto_active || s_park_active || s_manual_active || s_guide_pulse.active) {
        return OS_ERR_INVALID_STATE;
    }
    s_alignment_mode = mode;
    s_alignment_star_count = 0u;
    s_alignment_active = true;
    s_residual_valid = false;
    s_residual_arcsec = 0.0f;
    memset(s_alignment_stars, 0, sizeof(s_alignment_stars));
    memset(s_alignment_motor_pos, 0, sizeof(s_alignment_motor_pos));
    s_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (!valid_equatorial_coord(&star_coord)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_alignment_active) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_alignment_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }
    s_alignment_stars[s_alignment_star_count] = star_coord;
    s_alignment_motor_pos[s_alignment_star_count] = motor_pos;
    s_alignment_star_count += 1u;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (!s_alignment_active) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t min_stars = 0u;
    switch (s_alignment_mode) {
        case OS_ALIGN_1STAR:
            min_stars = 1u;
            break;
        case OS_ALIGN_2STAR:
            min_stars = 2u;
            break;
        case OS_ALIGN_3STAR:
        case OS_ALIGN_NSTAR:
            min_stars = 3u;
            break;
        default:
            return OS_ERR_INVALID_STATE;
    }
    if (s_alignment_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    double ra_arcsec[OS_CALIBRATION_MAX_STARS];
    double dec_arcsec[OS_CALIBRATION_MAX_STARS];
    double motor0[OS_CALIBRATION_MAX_STARS];
    double motor1[OS_CALIBRATION_MAX_STARS];
    int i;
    for (i = 0; i < s_alignment_star_count; ++i) {
        ra_arcsec[i] = (double)s_alignment_stars[i].ra_hours * 15.0 * 3600.0;
        dec_arcsec[i] = (double)s_alignment_stars[i].dec_degrees * 3600.0;
        motor0[i] = (double)s_alignment_motor_pos[i].ra_steps;
        motor1[i] = (double)s_alignment_motor_pos[i].dec_steps;
    }

    double m_rr = 0.0, m_rd = 0.0, m_dr = 0.0, m_dd = 0.0;
    double off_ra = 0.0, off_dec = 0.0;
    double residual_steps = 0.0;

    if (s_alignment_mode == OS_ALIGN_1STAR) {
        m_rr = 1.0;
        m_rd = 0.0;
        m_dr = 0.0;
        m_dd = 1.0;
        off_ra = motor0[0] - ra_arcsec[0];
        off_dec = motor1[0] - dec_arcsec[0];
    } else if (s_alignment_mode == OS_ALIGN_2STAR) {
        if (!solve_two(ra_arcsec[0], ra_arcsec[1], motor0[0], motor0[1], &m_rr, &off_ra)
            || !solve_two(dec_arcsec[0], dec_arcsec[1], motor1[0], motor1[1], &m_dd, &off_dec)) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        m_rd = 0.0;
        m_dr = 0.0;
    } else {
        double coef_ra[3] = {0.0, 0.0, 0.0};
        double coef_dec[3] = {0.0, 0.0, 0.0};
        double det = 0.0;
        if (!solve_full_affine(ra_arcsec, dec_arcsec, motor0, s_alignment_star_count, coef_ra, &det)
            || !solve_full_affine(ra_arcsec, dec_arcsec, motor1, s_alignment_star_count, coef_dec, &det)) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        off_ra = coef_ra[0];
        m_rr = coef_ra[1];
        m_rd = coef_ra[2];
        off_dec = coef_dec[0];
        m_dr = coef_dec[1];
        m_dd = coef_dec[2];
    }

    int n = (int)s_alignment_star_count;
    if (n >= 4) {
        double sum = 0.0;
        for (i = 0; i < n; ++i) {
            double fit0 = off_ra + m_rr * ra_arcsec[i] + m_rd * dec_arcsec[i];
            double fit1 = off_dec + m_dr * ra_arcsec[i] + m_dd * dec_arcsec[i];
            sum += (motor0[i] - fit0) * (motor0[i] - fit0);
            sum += (motor1[i] - fit1) * (motor1[i] - fit1);
        }
        residual_steps = sqrt(sum / (double)(n * 2));
        double scale_a = fabs(m_rr) + fabs(m_rd);
        double scale_d = fabs(m_dr) + fabs(m_dd);
        double scale = (scale_a > scale_d) ? scale_a : scale_d;
        if (scale < 1e-6) {
            scale = 1e-6;
        }
        residual_steps /= scale;
    }

    s_calibration.matrix_ra_to_ra = (float)m_rr;
    s_calibration.matrix_ra_to_dec = (float)m_rd;
    s_calibration.matrix_dec_to_ra = (float)m_dr;
    s_calibration.matrix_dec_to_dec = (float)m_dd;
    s_calibration.offset_ra_arcsec = (float)off_ra;
    s_calibration.offset_dec_arcsec = (float)off_dec;
    s_calibration.valid = true;

    os_error_t nvm_err = nvm_write_record(OS_NVM_CAL_OFFSET, OS_NVM_MAGIC_CALIBRATION,
                                           &s_calibration, (uint16_t)sizeof(s_calibration));
    if (nvm_err != OS_ERR_NONE) {
        s_calibration.valid = false;
        return OS_ERR_NVM_FAULT;
    }

    s_residual_arcsec = (float)residual_steps;
    s_residual_valid = true;
    s_alignment_active = false;
    s_alignment_star_count = 0u;
    if (s_state != OS_STATE_FAULT) {
        s_state = OS_STATE_IDLE_TRACKING;
    }
    apply_tracking_motor_rate();
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_residual_valid) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = s_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    s_alignment_active = false;
    s_alignment_star_count = 0u;
    memset(s_alignment_stars, 0, sizeof(s_alignment_stars));
    memset(s_alignment_motor_pos, 0, sizeof(s_alignment_motor_pos));
    if (s_state == OS_STATE_ALIGNMENT) {
        s_state = OS_STATE_IDLE_TRACKING;
        apply_tracking_motor_rate();
    }
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (s_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_goto_active || s_park_active || s_manual_active) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    os_equatorial_coord_t park_pos;
    if (s_park_position_valid) {
        park_pos = s_park_position;
    } else {
        park_pos.ra_hours = 0.0f;
        park_pos.dec_degrees = 90.0f;
    }
    if (!calibration_to_steps(&park_pos, s_goto_target_steps)) {
        s_goto_target_steps[0] = (int32_t)(park_pos.ra_hours * 100.0f);
        s_goto_target_steps[1] = (int32_t)(park_pos.dec_degrees * 100.0f);
    }
    s_park_active = true;
    s_goto_active = false;
    s_park_resume_tracking = s_tracking_enabled;
    s_state = OS_STATE_GOTO;
    initiate_async_motion();
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (s_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    (void)os_hal_motor_enable(0u, true);
    (void)os_hal_motor_enable(1u, true);
    (void)os_hal_comm_init(OS_CHANNEL_USB);
    (void)os_hal_comm_init(OS_CHANNEL_BLUETOOTH);
    (void)os_hal_comm_init(OS_CHANNEL_WIFI);
    (void)os_hal_comm_init(OS_CHANNEL_ETHERNET);

    uint32_t rtc_now = 0u;
    if (os_hal_rtc_read(&rtc_now) == OS_ERR_NONE) {
        s_site.utc_epoch_seconds = rtc_now;
    }
    s_tracking_enabled = true;
    s_state = OS_STATE_IDLE_TRACKING;
    apply_tracking_motor_rate();
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!valid_equatorial_coord(&park_pos)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    os_error_t err = nvm_write_record(OS_NVM_PARK_OFFSET, OS_NVM_MAGIC_PARK,
                                       &park_pos, (uint16_t)sizeof(park_pos));
    if (err != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    s_park_position = park_pos;
    s_park_position_valid = true;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if ((direction < OS_DIRECTION_NORTH) || (direction > OS_DIRECTION_WEST)
        || (speed < OS_SPEED_SLOW) || (speed > OS_SPEED_CUSTOM)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    uint8_t axis = ((direction == OS_DIRECTION_NORTH) || (direction == OS_DIRECTION_SOUTH)) ? 1u : 0u;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if ((s_state == OS_STATE_PARKED) || (s_state == OS_STATE_FAULT)) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_goto_active || s_park_active || s_manual_active) {
        return OS_ERR_INVALID_STATE;
    }

    uint32_t freq = 0u;
    switch (speed) {
        case OS_SPEED_SLOW:
            freq = 80u;
            break;
        case OS_SPEED_MEDIUM:
            freq = 400u;
            break;
        case OS_SPEED_FAST:
            freq = 1200u;
            break;
        case OS_SPEED_CUSTOM: {
            double freq_d = (double)s_manual_custom_speed_arcsec_per_sec * OS_MANUAL_STEPS_PER_ARCSEC;
            if (freq_d > (double)OS_MOTOR_MAX_FREQ_HZ) {
                freq_d = (double)OS_MOTOR_MAX_FREQ_HZ;
            }
            if (freq_d < 1.0) {
                freq_d = 1.0;
            }
            freq = (uint32_t)freq_d;
            break;
        }
        default:
            freq = 400u;
            break;
    }

    bool forward = (direction == OS_DIRECTION_NORTH) || (direction == OS_DIRECTION_EAST);
    if (!s_hal_motor_enabled[axis]) {
        (void)os_hal_motor_enable(axis, true);
    }
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, freq);
    s_manual_active = true;
    s_manual_direction = direction;
    s_manual_speed = speed;
    s_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    s_manual_active = false;
    if (!s_goto_active && !s_park_active && !s_guide_pulse.active) {
        if ((s_state != OS_STATE_PARKED) && (s_state != OS_STATE_FAULT)) {
            s_state = OS_STATE_IDLE_TRACKING;
            (void)os_tracking_enable();
        } else {
            stop_all_motor_frequencies();
        }
    }
    persist_motor_position();
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (!isfinite((double)arcsec_per_sec) || (arcsec_per_sec <= 0.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_manual_custom_speed_arcsec_per_sec = arcsec_per_sec;
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
    if (!steps_to_coordinates(s_motor_steps[0], s_motor_steps[1], coord)) {
        return OS_ERR_CALIBRATION_FAILED;
    }
    if (coord->ra_hours < OS_RA_MIN_HOURS) coord->ra_hours = OS_RA_MIN_HOURS;
    if (coord->ra_hours > OS_RA_MAX_HOURS) coord->ra_hours = OS_RA_MAX_HOURS;
    if (coord->dec_degrees < OS_DEC_MIN_DEG) coord->dec_degrees = OS_DEC_MIN_DEG;
    if (coord->dec_degrees > OS_DEC_MAX_DEG) coord->dec_degrees = OS_DEC_MAX_DEG;
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    os_site_info_t gps_site;
    memset(&gps_site, 0, sizeof(gps_site));
    if ((os_hal_gps_poll(&gps_site) == OS_ERR_NONE) && gps_site.valid) {
        s_site = gps_site;
        s_gps_locked = true;
    } else {
        s_gps_locked = false;
        uint32_t rtc_now = 0u;
        if (os_hal_rtc_read(&rtc_now) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = rtc_now;
        }
    }
    *site = s_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = s_motor_steps[0];
    pos->dec_steps = s_motor_steps[1];
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
    if (enable && !s_pec_table.valid) {
        return OS_ERR_INVALID_STATE;
    }
    s_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(&s_pec_table, table, sizeof(s_pec_table));
    if (!s_pec_table.valid) {
        s_pec_enabled = false;
    } else {
        os_error_t err = nvm_write_record(OS_NVM_PEC_OFFSET, OS_NVM_MAGIC_PEC,
                                           &s_pec_table, (uint16_t)sizeof(s_pec_table));
        if (err != OS_ERR_NONE) {
            return OS_ERR_NVM_FAULT;
        }
    }
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(table, &s_pec_table, sizeof(s_pec_table));
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (!isfinite((double)worm_phase_deg)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((worm_phase_deg < 0.0f) || (worm_phase_deg > 360.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int idx = (int)floorf(worm_phase_deg + 0.5f);
    if (idx >= OS_PEC_TABLE_SIZE) {
        idx = 0;
    }
    if (idx < 0) {
        idx = 0;
    }
    s_pec_table.corrections[idx] = error_arcsec;
    s_pec_table.valid = true;
    os_error_t err = nvm_write_record(OS_NVM_PEC_OFFSET, OS_NVM_MAGIC_PEC,
                                       &s_pec_table, (uint16_t)sizeof(s_pec_table));
    if (err != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_calibration.valid) {
        return OS_ERR_INVALID_STATE;
    }
    *calib = s_calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    memset(&s_calibration, 0, sizeof(s_calibration));
    s_calibration.valid = false;

    os_calibration_t cleared;
    memset(&cleared, 0, sizeof(cleared));
    os_error_t err = nvm_write_record(OS_NVM_CAL_OFFSET, OS_NVM_MAGIC_CALIBRATION,
                                       &cleared, (uint16_t)sizeof(cleared));
    if (err != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_init(uint8_t axis) {
    if (axis > 1u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_hal_motor_initialized[axis] = true;
    s_hal_motor_enabled[axis] = false;
    s_hal_motor_direction[axis] = false;
    s_hal_motor_frequency[axis] = 0u;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (axis > 1u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_hal_motor_enabled[axis]) {
        if (frequency_hz != 0u) {
            return OS_ERR_INVALID_STATE;
        }
        s_hal_motor_frequency[axis] = 0u;
        return OS_ERR_NONE;
    }
    if (frequency_hz > OS_MOTOR_MAX_FREQ_HZ) {
        frequency_hz = OS_MOTOR_MAX_FREQ_HZ;
    }
    s_hal_motor_frequency[axis] = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (axis > 1u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_hal_motor_direction[axis] = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (axis > 1u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_hal_motor_enabled[axis] = enable;
    if (!enable) {
        s_hal_motor_frequency[axis] = 0u;
    }
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (axis > 1u) {
        return 0;
    }
    return s_motor_steps[axis];
}

os_error_t os_hal_gps_init(void) {
    s_hal_gps_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memset(site, 0, sizeof(*site));
    if (!s_hal_gps_valid) {
        return OS_ERR_GPS_NO_SIGNAL;
    }
    *site = s_hal_gps_site;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void) {
    s_hal_rtc_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *utc_epoch_seconds = s_hal_rtc_time;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    s_hal_rtc_time = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    s_hal_limit_initialized = true;
    s_hal_limit_triggered[0] = false;
    s_hal_limit_triggered[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (axis > 1u) {
        return true;
    }
    return s_hal_limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void) {
    s_hal_nvm_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (((uint32_t)offset + (uint32_t)length) > (uint32_t)OS_NVM_TOTAL_CAPACITY) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(data, &s_hal_nvm[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (((uint32_t)offset + (uint32_t)length) > (uint32_t)OS_NVM_TOTAL_CAPACITY) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(&s_hal_nvm[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_hal_comm_initialized[channel] = true;
    s_hal_comm_rx_len[channel] = 0u;
    s_hal_comm_rx_head[channel] = 0u;
    s_hal_comm_tx_len[channel] = 0u;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if ((channel > OS_CHANNEL_ETHERNET) || !s_hal_comm_initialized[channel]) {
        return 0;
    }
    uint16_t avail = s_hal_comm_rx_len[channel] - s_hal_comm_rx_head[channel];
    return (int16_t)avail;
}

char os_hal_comm_read(uint8_t channel) {
    if ((channel > OS_CHANNEL_ETHERNET) || !s_hal_comm_initialized[channel]) {
        return '\0';
    }
    if (s_hal_comm_rx_head[channel] >= s_hal_comm_rx_len[channel]) {
        return '\0';
    }
    char byte = s_hal_comm_rx[channel][s_hal_comm_rx_head[channel]];
    s_hal_comm_rx_head[channel] += 1u;
    return byte;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length > OS_COMM_TX_CAPACITY) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((s_hal_comm_tx_len[channel] + length) > OS_COMM_TX_CAPACITY) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(&s_hal_comm_tx[channel][s_hal_comm_tx_len[channel]], data, length);
    s_hal_comm_tx_len[channel] += length;
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) {
    (void)duration_ms;
    (void)count;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void) {
    s_hal_motor_timer_initialized = true;
    return OS_ERR_NONE;
}
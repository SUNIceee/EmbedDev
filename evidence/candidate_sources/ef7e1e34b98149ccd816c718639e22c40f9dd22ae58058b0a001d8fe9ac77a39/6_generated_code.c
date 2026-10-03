#include "6_generated_code.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#define ME_PI 3.14159265358979323846
#define AXIS_COUNT 2
#define OS_LOOP_ASSUMED_MS 10u
#define STEPS_PER_DEGREE 1000.0
#define STEPS_PER_ARCSEC (STEPS_PER_DEGREE / 3600.0)
#define RA_STEPS_PER_HOUR (15.0 * STEPS_PER_DEGREE)
#define OS_ALIGN_MAX_RESIDUAL_ARCSEC 300.0f
#define NVM_PEC_MAGIC_SIZE 4u
#define NVM_PEC_OFFSET (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)
#define NVM_CONFIG_OFFSET (OS_NVM_CALIBRATION_SIZE_BYTES)
#define NVM_PEC_SIZE (sizeof(os_pec_table_t) + NVM_PEC_MAGIC_SIZE)
#define NVM_TOTAL_SIZE (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES + NVM_PEC_SIZE)
#define COMM_RX_BUF_SIZE (OS_MAX_COMMAND_LENGTH + 4)
#define COMM_TX_BUF_SIZE OS_MAX_REPLY_LENGTH

static const uint8_t s_nvm_calib_magic[4] = { 'O','S','C','1' };
static const uint8_t s_nvm_config_magic[4] = { 'O','S','C','2' };
static const uint8_t s_nvm_pec_magic[4] = { 'O','S','P','1' };

static os_state_t s_state = OS_STATE_INITIALIZING;
static bool s_init_complete = false;
static os_mount_type_t s_mount = OS_MOUNT_EQUATORIAL;
static os_site_info_t s_site;
static uint32_t s_rtc_epoch = 0;
static bool s_rtc_valid = false;
static bool s_gps_valid = false;
static os_track_rate_t s_track_rate = OS_TRACK_RATE_SIDEREAL;
static float s_custom_track_factor = 1.0f;
static bool s_tracking_enabled = true;
static bool s_parked = false;
static bool s_park_pos_set = false;
static os_equatorial_coord_t s_park_pos;
static float s_custom_manual_speed_arcsec_per_sec = 100.0f;

static bool s_goto_active = false;
static bool s_auto_active = false;
static os_state_t s_auto_dest_state = OS_STATE_IDLE_TRACKING;
static int32_t s_target_steps[AXIS_COUNT] = { 0, 0 };
static os_equatorial_coord_t s_goto_target;
static bool s_goto_target_set = false;
static bool s_manual_active = false;
static os_direction_t s_manual_direction = OS_DIRECTION_NORTH;
static os_speed_level_t s_manual_speed = OS_SPEED_SLOW;
static uint8_t s_manual_axis = 0;
static bool s_manual_forward = true;

static bool s_guide_active = false;
static os_guide_pulse_t s_guide_pulse;
static os_guide_pulse_t s_guide_queue[2];
static uint8_t s_guide_queue_count = 0;
static float s_guide_rate = 0.5f;

static os_calibration_t s_calib;
static bool s_calib_valid = false;
static os_align_mode_t s_align_mode = OS_ALIGN_3STAR;
static int s_align_count = 0;
static os_equatorial_coord_t s_align_stars[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t s_align_pos[OS_CALIBRATION_MAX_STARS];
static bool s_align_residual_valid = false;
static float s_align_residual = 0.0f;

static bool s_pec_enabled = false;
static bool s_pec_loaded = false;
static os_pec_table_t s_pec_table;

static bool s_motor_fault[AXIS_COUNT] = { false, false };
static bool s_motion_limit_fault = false;

static bool s_hal_motor_initialized[AXIS_COUNT] = { false, false };
static bool s_hal_motor_enabled[AXIS_COUNT] = { false, false };
static bool s_hal_motor_direction[AXIS_COUNT] = { true, true };
static uint32_t s_hal_motor_freq[AXIS_COUNT] = { 0, 0 };
static int32_t s_hal_motor_pos[AXIS_COUNT] = { 0, 0 };
static double s_hal_motor_fraction[AXIS_COUNT] = { 0.0, 0.0 };
static bool s_hal_limit_triggered[AXIS_COUNT] = { false, false };
static uint32_t s_hal_rtc_epoch = 0;
static bool s_hal_rtc_initialized = false;
static uint8_t s_nvm[NVM_TOTAL_SIZE];
static bool s_nvm_initialized = false;
static bool s_hal_comm_initialized[4] = { false, false, false, false };
static uint8_t s_comm_phys_rx[4][COMM_RX_BUF_SIZE];
static uint8_t s_comm_phys_rx_len[4] = { 0, 0, 0, 0 };
static char s_comm_tx[4][COMM_TX_BUF_SIZE];
static size_t s_comm_tx_len[4] = { 0, 0, 0, 0 };
static char s_comm_frame_buf[4][OS_MAX_COMMAND_LENGTH + 2];
static uint8_t s_comm_frame_len[4] = { 0, 0, 0, 0 };
static bool s_comm_frame_active[4] = { false, false, false, false };

static os_site_info_t s_hal_gps_latest;
static bool s_hal_gps_latest_valid = false;

static bool valid_axis(uint8_t axis) {
    return axis < AXIS_COUNT;
}

static bool valid_channel(uint8_t channel) {
    return channel <= OS_CHANNEL_ETHERNET;
}

static bool valid_ra_hours(float ra) {
    return ra >= OS_RA_MIN_HOURS && ra <= OS_RA_MAX_HOURS;
}

static bool valid_dec_degrees(float dec) {
    return dec >= OS_DEC_MIN_DEG && dec <= OS_DEC_MAX_DEG;
}

static bool valid_azimuth_degrees(float az) {
    return az >= 0.0f && az <= 360.0f;
}

static bool valid_altitude_degrees(float alt) {
    return alt >= -90.0f && alt <= 90.0f;
}

static float wrap_ra_hours(double hours) {
    while (hours < 0.0) hours += 24.0;
    while (hours >= 24.0) hours -= 24.0;
    return (float)hours;
}

static double wrap_two_pi(double a) {
    a = fmod(a, 2.0 * ME_PI);
    if (a < 0.0) a += 2.0 * ME_PI;
    return a;
}

static double clamp_sin(double x) {
    if (x < -1.0) x = -1.0;
    if (x > 1.0) x = 1.0;
    return x;
}

static double deg_to_rad(double d) {
    return d * ME_PI / 180.0;
}

static double rad_to_deg(double r) {
    return r * 180.0 / ME_PI;
}

static bool has_time_source(void) {
    return s_site.valid || s_rtc_valid || s_gps_valid;
}

static uint32_t current_utc(void) {
    if (s_site.valid && s_site.utc_epoch_seconds != 0) return s_site.utc_epoch_seconds;
    if (s_rtc_valid) return s_rtc_epoch;
    return 0;
}

static double unix_to_j2000_days(uint32_t unix) {
    return ((double)unix / 86400.0) - 10957.5;
}

static double gmst_rad(uint32_t unix) {
    double d = unix_to_j2000_days(unix);
    double gm = 280.46061837 + 360.98564736629 * d;
    gm = fmod(gm, 360.0);
    if (gm < 0.0) gm += 360.0;
    return deg_to_rad(gm);
}

static double local_sidereal_rad(double longitude_deg, uint32_t unix) {
    return wrap_two_pi(gmst_rad(unix) + deg_to_rad(longitude_deg));
}

static void horizontal_to_equatorial_rad(double alt, double az, double lat, double lst,
                                         double *ra_out, double *dec_out) {
    double sin_alt = sin(alt);
    double cos_alt = cos(alt);
    double sin_az = sin(az);
    double cos_az = cos(az);
    double sin_lat = sin(lat);
    double cos_lat = cos(lat);
    double sin_dec = sin_lat * sin_alt + cos_lat * cos_alt * cos_az;
    *dec_out = asin(clamp_sin(sin_dec));
    double h = atan2(-cos_alt * sin_az,
                     cos_lat * sin_alt - sin_lat * cos_alt * cos_az);
    *ra_out = wrap_two_pi(lst - h);
}

static void equatorial_to_horizontal_rad(double ra, double dec, double lat, double lst,
                                         double *alt_out, double *az_out) {
    double h = lst - ra;
    double sin_alt = sin(lat) * sin(dec) + cos(lat) * cos(dec) * cos(h);
    *alt_out = asin(clamp_sin(sin_alt));
    double az = atan2(-cos(dec) * sin(h),
                      cos(lat) * sin(dec) - sin(lat) * cos(dec) * cos(h));
    *az_out = wrap_two_pi(az);
}

static double clip_speed_to_goto_max(double freq) {
    double max_steps_per_sec = OS_GOTO_SPEED_MAX_DEG_PER_SEC * STEPS_PER_DEGREE;
    if (freq > max_steps_per_sec) freq = max_steps_per_sec;
    return freq;
}

static os_motor_position_t current_motor_pos(void) {
    os_motor_position_t p;
    p.ra_steps = os_hal_motor_get_position(0);
    p.dec_steps = os_hal_motor_get_position(1);
    return p;
}

static int32_t double_to_int32_round(double v) {
    if (v >= 0.0) return (int32_t)(v + 0.5);
    return (int32_t)(v - 0.5);
}

static os_equatorial_coord_t motor_pos_to_coord_equatorial(os_motor_position_t pos) {
    os_equatorial_coord_t c;
    double ra_steps = (double)pos.ra_steps;
    double dec_steps = (double)pos.dec_steps;
    if (s_calib_valid) {
        double a = (double)s_calib.matrix_ra_to_ra;
        double b = (double)s_calib.matrix_ra_to_dec;
        double d = (double)s_calib.matrix_dec_to_ra;
        double e = (double)s_calib.matrix_dec_to_dec;
        double det = a * e - b * d;
        if (fabs(det) > 1e-12) {
            double ox = (double)s_calib.offset_ra_arcsec;
            double oy = (double)s_calib.offset_dec_arcsec;
            double rm = ra_steps - ox;
            double dm = dec_steps - oy;
            double ra_arcsec = ( e * rm - b * dm) / det;
            double dec_arcsec = (-d * rm + a * dm) / det;
            c.ra_hours = wrap_ra_hours(ra_arcsec / (15.0 * 3600.0));
            c.dec_degrees = (float)(dec_arcsec / 3600.0);
            return c;
        }
    }
    c.ra_hours = wrap_ra_hours(ra_steps / RA_STEPS_PER_HOUR);
    c.dec_degrees = (float)(dec_steps / STEPS_PER_DEGREE);
    return c;
}

static os_equatorial_coord_t altaz_motor_to_equatorial(os_motor_position_t pos) {
    os_equatorial_coord_t c;
    double alt = (double)pos.dec_steps / STEPS_PER_DEGREE;
    double az = (double)pos.ra_steps / STEPS_PER_DEGREE;
    az = fmod(az, 360.0);
    if (az < 0.0) az += 360.0;
    if (!has_time_source()) {
        c.ra_hours = 0.0f;
        c.dec_degrees = (float)alt;
        return c;
    }
    double lat = s_site.latitude_degrees;
    double lon = s_site.longitude_degrees;
    uint32_t utc = current_utc();
    double lst = local_sidereal_rad(lon, utc);
    double ra_rad = 0.0, dec_rad = 0.0;
    horizontal_to_equatorial_rad(deg_to_rad(alt), deg_to_rad(az), deg_to_rad(lat), lst,
                                 &ra_rad, &dec_rad);
    c.ra_hours = wrap_ra_hours(rad_to_deg(ra_rad) / 15.0);
    c.dec_degrees = (float)rad_to_deg(dec_rad);
    return c;
}

static os_equatorial_coord_t motor_pos_to_coord(os_motor_position_t pos) {
    if (s_mount == OS_MOUNT_ALTAZ) return altaz_motor_to_equatorial(pos);
    return motor_pos_to_coord_equatorial(pos);
}

static os_motor_position_t coord_to_motor_pos_equatorial(os_equatorial_coord_t coord) {
    os_motor_position_t p;
    double ra_arcsec = (double)coord.ra_hours * 15.0 * 3600.0;
    double dec_arcsec = (double)coord.dec_degrees * 3600.0;
    if (s_calib_valid) {
        double a = (double)s_calib.matrix_ra_to_ra;
        double b = (double)s_calib.matrix_ra_to_dec;
        double d = (double)s_calib.matrix_dec_to_ra;
        double e = (double)s_calib.matrix_dec_to_dec;
        double ox = (double)s_calib.offset_ra_arcsec;
        double oy = (double)s_calib.offset_dec_arcsec;
        p.ra_steps = double_to_int32_round(a * ra_arcsec + b * dec_arcsec + ox);
        p.dec_steps = double_to_int32_round(d * ra_arcsec + e * dec_arcsec + oy);
    } else {
        p.ra_steps = double_to_int32_round(ra_arcsec * STEPS_PER_ARCSEC);
        p.dec_steps = double_to_int32_round(dec_arcsec * STEPS_PER_ARCSEC);
    }
    return p;
}

static os_motor_position_t coord_to_motor_pos_altaz(os_equatorial_coord_t coord) {
    os_motor_position_t p;
    if (!has_time_source()) {
        p.ra_steps = 0;
        p.dec_steps = 0;
        return p;
    }
    double lat = s_site.latitude_degrees;
    double lon = s_site.longitude_degrees;
    uint32_t utc = current_utc();
    double lst = local_sidereal_rad(lon, utc);
    double ra = deg_to_rad((double)coord.ra_hours * 15.0);
    double dec = deg_to_rad((double)coord.dec_degrees);
    double alt = 0.0, az = 0.0;
    equatorial_to_horizontal_rad(ra, dec, deg_to_rad(lat), lst, &alt, &az);
    p.ra_steps = double_to_int32_round(rad_to_deg(az) * STEPS_PER_DEGREE);
    p.dec_steps = double_to_int32_round(rad_to_deg(alt) * STEPS_PER_DEGREE);
    return p;
}

static os_motor_position_t coord_to_motor_pos(os_equatorial_coord_t coord) {
    if (s_mount == OS_MOUNT_ALTAZ) return coord_to_motor_pos_altaz(coord);
    return coord_to_motor_pos_equatorial(coord);
}

static os_motor_position_t horizontal_to_motor_pos_equatorial(os_horizontal_coord_t target) {
    os_motor_position_t p;
    if (!has_time_source()) {
        p.ra_steps = 0;
        p.dec_steps = 0;
        return p;
    }
    double lat = s_site.latitude_degrees;
    double lon = s_site.longitude_degrees;
    uint32_t utc = current_utc();
    double lst = local_sidereal_rad(lon, utc);
    double ra_rad = 0.0, dec_rad = 0.0;
    horizontal_to_equatorial_rad(deg_to_rad(target.altitude_degrees),
                                 deg_to_rad(target.azimuth_degrees),
                                 deg_to_rad(lat), lst, &ra_rad, &dec_rad);
    os_equatorial_coord_t eq;
    eq.ra_hours = wrap_ra_hours(rad_to_deg(ra_rad) / 15.0);
    eq.dec_degrees = (float)rad_to_deg(dec_rad);
    return coord_to_motor_pos(eq);
}

static os_motor_position_t horizontal_to_motor_pos(os_horizontal_coord_t target) {
    if (s_mount == OS_MOUNT_ALTAZ) {
        os_motor_position_t p;
        p.ra_steps = double_to_int32_round((double)target.azimuth_degrees * STEPS_PER_DEGREE);
        p.dec_steps = double_to_int32_round((double)target.altitude_degrees * STEPS_PER_DEGREE);
        return p;
    }
    return horizontal_to_motor_pos_equatorial(target);
}

static bool motion_blocked_by_limits(os_motor_position_t target) {
    os_motor_position_t cur = current_motor_pos();
    if (os_hal_limit_is_triggered(0) && target.ra_steps != cur.ra_steps) return true;
    if (os_hal_limit_is_triggered(1) && target.dec_steps != cur.dec_steps) return true;
    return false;
}

static void halt_axes(void) {
    os_hal_motor_set_frequency(0, 0);
    os_hal_motor_set_frequency(1, 0);
}

static void enter_fault_state(void) {
    halt_axes();
    s_auto_active = false;
    s_goto_active = false;
    s_manual_active = false;
    s_guide_active = false;
    s_guide_queue_count = 0;
    s_state = OS_STATE_FAULT;
    s_motion_limit_fault = true;
}

static os_error_t enable_axis_motor(uint8_t axis) {
    os_error_t err = os_hal_motor_init(axis);
    if (err != OS_ERR_NONE) {
        s_motor_fault[axis] = true;
        enter_fault_state();
        return err;
    }
    os_hal_motor_enable(axis, true);
    return OS_ERR_NONE;
}

static os_error_t start_auto_move(os_motor_position_t target, os_state_t dest_state) {
    if (motion_blocked_by_limits(target)) return OS_ERR_LIMIT_TRIGGERED;
    os_error_t e0 = enable_axis_motor(0);
    if (e0 != OS_ERR_NONE) return e0;
    os_error_t e1 = enable_axis_motor(1);
    if (e1 != OS_ERR_NONE) return e1;
    s_target_steps[0] = target.ra_steps;
    s_target_steps[1] = target.dec_steps;
    s_auto_active = true;
    s_auto_dest_state = dest_state;
    s_goto_active = true;
    s_manual_active = false;
    s_guide_active = false;
    s_guide_queue_count = 0;
    s_state = OS_STATE_GOTO;
    os_motor_position_t cur = current_motor_pos();
    int32_t diff0 = s_target_steps[0] - cur.ra_steps;
    int32_t diff1 = s_target_steps[1] - cur.dec_steps;
    os_hal_motor_set_direction(0, diff0 >= 0);
    os_hal_motor_set_direction(1, diff1 >= 0);
    os_hal_motor_set_frequency(0, diff0 != 0 ? (uint32_t)clip_speed_to_goto_max(3000.0) : 0);
    os_hal_motor_set_frequency(1, diff1 != 0 ? (uint32_t)clip_speed_to_goto_max(3000.0) : 0);
    return OS_ERR_NONE;
}

static uint32_t manual_speed_frequency(os_speed_level_t speed) {
    switch (speed) {
        case OS_SPEED_SLOW: return 500;
        case OS_SPEED_MEDIUM: return 1000;
        case OS_SPEED_FAST: return 2000;
        case OS_SPEED_CUSTOM:
        default: {
            double steps = (double)fabs(s_custom_manual_speed_arcsec_per_sec) * STEPS_PER_ARCSEC;
            if (steps > 10000.0) steps = 10000.0;
            return (uint32_t)steps;
        }
    }
}

static bool map_direction_to_axis_forward(os_direction_t dir, uint8_t *axis, bool *forward) {
    *axis = 0;
    *forward = true;
    if (s_mount == OS_MOUNT_EQUATORIAL) {
        switch (dir) {
            case OS_DIRECTION_EAST:  *axis = 0; *forward = false; return true;
            case OS_DIRECTION_WEST:  *axis = 0; *forward = true;  return true;
            case OS_DIRECTION_NORTH: *axis = 1; *forward = true;  return true;
            case OS_DIRECTION_SOUTH: *axis = 1; *forward = false; return true;
            default: return false;
        }
    } else {
        switch (dir) {
            case OS_DIRECTION_EAST:  *axis = 0; *forward = true;  return true;
            case OS_DIRECTION_WEST:  *axis = 0; *forward = false; return true;
            case OS_DIRECTION_NORTH: *axis = 1; *forward = true;  return true;
            case OS_DIRECTION_SOUTH: *axis = 1; *forward = false; return true;
            default: return false;
        }
    }
}

static void simulate_step_for_axis(uint8_t axis, int32_t clamp_target, bool has_clamp_target) {
    if (!valid_axis(axis) || !s_hal_motor_initialized[axis]) return;
    if (!s_hal_motor_enabled[axis] || s_hal_motor_freq[axis] == 0) return;
    s_hal_motor_fraction[axis] += (double)s_hal_motor_freq[axis] * OS_LOOP_ASSUMED_MS / 1000.0;
    int32_t whole = (int32_t)s_hal_motor_fraction[axis];
    if (whole == 0) return;
    s_hal_motor_fraction[axis] -= (double)whole;
    int32_t dir = s_hal_motor_direction[axis] ? 1 : -1;
    int32_t new_pos = s_hal_motor_pos[axis] + dir * whole;
    if (has_clamp_target) {
        if (dir > 0 && new_pos > clamp_target) new_pos = clamp_target;
        if (dir < 0 && new_pos < clamp_target) new_pos = clamp_target;
    }
    s_hal_motor_pos[axis] = new_pos;
}

static double tracking_rate_factor(void) {
    switch (s_track_rate) {
        case OS_TRACK_RATE_SIDEREAL: return 1.0;
        case OS_TRACK_RATE_LUNAR: return OS_LUNAR_RATE_FACTOR;
        case OS_TRACK_RATE_SOLAR: return OS_SOLAR_RATE_FACTOR;
        case OS_TRACK_RATE_CUSTOM: return (double)s_custom_track_factor;
        default: return 1.0;
    }
}

static double pec_correction_steps_per_sec(void) {
    if (!s_pec_enabled || !s_pec_table.valid) return 0.0;
    int32_t ra = s_hal_motor_pos[0];
    uint32_t idx = (ra < 0) ? (uint32_t)(-(ra + 1)) + 1u : (uint32_t)ra;
    idx %= OS_PEC_TABLE_SIZE;
    return (double)s_pec_table.corrections[idx] * STEPS_PER_ARCSEC;
}

static bool guide_queue_push(os_guide_pulse_t pulse) {
    if (s_guide_queue_count >= 2) return false;
    s_guide_queue[s_guide_queue_count++] = pulse;
    return true;
}

static bool guide_queue_pop(os_guide_pulse_t *pulse) {
    if (s_guide_queue_count == 0) return false;
    int idx = 0;
    if (s_guide_queue_count == 2 &&
        !s_guide_queue[0].dec_priority && s_guide_queue[1].dec_priority) {
        idx = 1;
    }
    *pulse = s_guide_queue[idx];
    for (int i = idx; i < (int)s_guide_queue_count - 1; ++i) {
        s_guide_queue[i] = s_guide_queue[i + 1];
    }
    --s_guide_queue_count;
    return true;
}

static void update_tracking_and_guide(void) {
    if (s_guide_active) {
        if (s_guide_pulse.duration_ms > OS_LOOP_ASSUMED_MS) {
            s_guide_pulse.duration_ms -= OS_LOOP_ASSUMED_MS;
        } else {
            s_guide_pulse.active = false;
            s_guide_active = false;
            if (guide_queue_pop(&s_guide_pulse)) {
                s_guide_pulse.active = true;
                s_guide_active = true;
            }
        }
    } else if (guide_queue_pop(&s_guide_pulse)) {
        s_guide_pulse.active = true;
        s_guide_active = true;
    }

    uint32_t base_ra = 0;
    if (s_tracking_enabled &&
        (s_state == OS_STATE_IDLE_TRACKING || s_state == OS_STATE_ALIGNMENT || s_state == OS_STATE_GOTO)) {
        double sidereal_steps_per_sec = OS_SIDEREAL_RATE_ARCSEC_PER_SEC * STEPS_PER_ARCSEC;
        base_ra = (uint32_t)(sidereal_steps_per_sec * tracking_rate_factor());
    }

    double ra_freq_double = (double)base_ra + pec_correction_steps_per_sec();
    if (ra_freq_double < 0.0) ra_freq_double = 0.0;
    uint32_t ra_freq = (uint32_t)ra_freq_double;
    uint32_t dec_freq = 0;
    bool ra_forward = true;
    bool dec_forward = true;

    if (s_guide_active) {
        uint32_t bias = (uint32_t)(s_guide_rate * OS_SIDEREAL_RATE_ARCSEC_PER_SEC * STEPS_PER_ARCSEC);
        if (s_guide_pulse.dec_priority) {
            dec_freq = bias;
            dec_forward = s_guide_pulse.direction_north ? true : false;
        } else {
            ra_freq += bias;
            ra_forward = s_guide_pulse.direction_east ? true : false;
        }
    }

    os_hal_motor_set_direction(0, ra_forward);
    os_hal_motor_set_frequency(0, ra_freq);
    os_hal_motor_set_direction(1, dec_forward);
    os_hal_motor_set_frequency(1, dec_freq);
    simulate_step_for_axis(0, 0, false);
    simulate_step_for_axis(1, 0, false);
}

static void poll_time_location(void) {
    os_site_info_t site;
    if (os_hal_gps_poll(&site) == OS_ERR_NONE && site.valid) {
        s_site = site;
        s_gps_valid = true;
        s_rtc_valid = true;
        return;
    }

    s_gps_valid = false;
    uint32_t t = 0;
    if (os_hal_rtc_read(&t) == OS_ERR_NONE) {
        s_rtc_epoch = t;
        s_site.utc_epoch_seconds = t;
        s_site.valid = false;
        s_rtc_valid = true;
    } else {
        s_rtc_valid = false;
        s_site.valid = false;
        s_site.utc_epoch_seconds = 0;
    }
}

static void process_rx_frame(uint8_t channel, const char *cmd, size_t len) {
    char reply[OS_MAX_REPLY_LENGTH];
    size_t reply_len = 0;
    os_error_t err = os_command_parse(cmd, len, channel, reply, sizeof(reply), &reply_len);
    (void)err;
    if (reply_len > 0) os_hal_comm_write(channel, reply, reply_len);
}

static void poll_commands(void) {
    for (uint8_t ch = 0; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        if (!s_hal_comm_initialized[ch]) continue;
        while (os_hal_comm_available(ch) > 0) {
            char c = os_hal_comm_read(ch);
            if (s_comm_frame_active[ch]) {
                if (c == OS_LX200_CMD_SUFFIX) {
                    if (s_comm_frame_len[ch] < OS_MAX_COMMAND_LENGTH) {
                        s_comm_frame_buf[ch][s_comm_frame_len[ch]++] = c;
                    }
                    s_comm_frame_buf[ch][s_comm_frame_len[ch]] = '\0';
                    process_rx_frame(ch, s_comm_frame_buf[ch], s_comm_frame_len[ch]);
                    s_comm_frame_active[ch] = false;
                    s_comm_frame_len[ch] = 0;
                } else if (c == OS_LX200_CMD_PREFIX) {
                    s_comm_frame_buf[ch][0] = c;
                    s_comm_frame_len[ch] = 1;
                    s_comm_frame_active[ch] = true;
                } else if (s_comm_frame_len[ch] < OS_MAX_COMMAND_LENGTH) {
                    s_comm_frame_buf[ch][s_comm_frame_len[ch]++] = c;
                } else {
                    s_comm_frame_active[ch] = false;
                    s_comm_frame_len[ch] = 0;
                }
            } else if (c == OS_LX200_CMD_PREFIX) {
                s_comm_frame_buf[ch][0] = c;
                s_comm_frame_len[ch] = 1;
                s_comm_frame_active[ch] = true;
            }
        }
    }
}

static bool solve_3param_centered(const double *x, const double *y, const double *target,
                                  int n, double coeff[3]) {
    if (n < 3 || n > OS_CALIBRATION_MAX_STARS) return false;
    double mean_x = 0.0, mean_y = 0.0, mean_t = 0.0;
    for (int i = 0; i < n; ++i) {
        mean_x += x[i];
        mean_y += y[i];
        mean_t += target[i];
    }
    mean_x /= (double)n;
    mean_y /= (double)n;
    mean_t /= (double)n;

    double X[OS_CALIBRATION_MAX_STARS];
    double Y[OS_CALIBRATION_MAX_STARS];
    double B[OS_CALIBRATION_MAX_STARS];
    double A[OS_CALIBRATION_MAX_STARS][3];
    for (int i = 0; i < n; ++i) {
        X[i] = x[i] - mean_x;
        Y[i] = y[i] - mean_y;
        B[i] = target[i] - mean_t;
        A[i][0] = X[i];
        A[i][1] = Y[i];
        A[i][2] = 1.0;
    }

    double Q[OS_CALIBRATION_MAX_STARS][3];
    double R[3][3] = { {0.0} };
    for (int k = 0; k < 3; ++k) {
        for (int i = 0; i < n; ++i) Q[i][k] = A[i][k];
        for (int j = 0; j < k; ++j) {
            double rjk = 0.0;
            for (int i = 0; i < n; ++i) rjk += Q[i][j] * A[i][k];
            R[j][k] = rjk;
            for (int i = 0; i < n; ++i) Q[i][k] -= rjk * Q[i][j];
        }
        double norm = 0.0;
        for (int i = 0; i < n; ++i) norm += Q[i][k] * Q[i][k];
        norm = sqrt(norm);
        if (norm < 1e-12) return false;
        R[k][k] = norm;
        for (int i = 0; i < n; ++i) Q[i][k] /= norm;
    }

    double qtb[3] = { 0.0, 0.0, 0.0 };
    for (int k = 0; k < 3; ++k) {
        for (int i = 0; i < n; ++i) qtb[k] += Q[i][k] * B[i];
    }

    double raw[3] = { 0.0, 0.0, 0.0 };
    for (int i = 2; i >= 0; --i) {
        double sum = qtb[i];
        for (int j = i + 1; j < 3; ++j) sum -= R[i][j] * raw[j];
        if (fabs(R[i][i]) < 1e-12) return false;
        raw[i] = sum / R[i][i];
    }

    coeff[0] = raw[0];
    coeff[1] = raw[1];
    coeff[2] = mean_t + raw[2] - raw[0] * mean_x - raw[1] * mean_y;
    return true;
}

static bool fit_line_ls(const double *x, const double *y, int n, double *a, double *c) {
    if (n < 2) return false;
    double mean_x = 0.0, mean_y = 0.0;
    for (int i = 0; i < n; ++i) {
        mean_x += x[i];
        mean_y += y[i];
    }
    mean_x /= (double)n;
    mean_y /= (double)n;
    double num = 0.0, den = 0.0;
    for (int i = 0; i < n; ++i) {
        double dx = x[i] - mean_x;
        num += dx * (y[i] - mean_y);
        den += dx * dx;
    }
    if (fabs(den) < 1e-15) return false;
    *a = num / den;
    *c = mean_y - (*a) * mean_x;
    return true;
}

static os_error_t compute_alignment(void) {
    double ra_arcsec[OS_CALIBRATION_MAX_STARS];
    double dec_arcsec[OS_CALIBRATION_MAX_STARS];
    double motor_ra[OS_CALIBRATION_MAX_STARS];
    double motor_dec[OS_CALIBRATION_MAX_STARS];

    for (int i = 0; i < s_align_count; ++i) {
        ra_arcsec[i] = (double)s_align_stars[i].ra_hours * 15.0 * 3600.0;
        dec_arcsec[i] = (double)s_align_stars[i].dec_degrees * 3600.0;
        motor_ra[i] = (double)s_align_pos[i].ra_steps;
        motor_dec[i] = (double)s_align_pos[i].dec_steps;
    }

    double a_ra = 0.0, b_ra = 0.0, c_ra = 0.0;
    double d_dec = 0.0, e_dec = 0.0, f_dec = 0.0;

    if (s_align_mode == OS_ALIGN_1STAR) {
        if (s_align_count < 1) return OS_ERR_INVALID_STATE;
        double sps = STEPS_PER_ARCSEC;
        a_ra = sps;
        b_ra = 0.0;
        c_ra = motor_ra[0] - sps * ra_arcsec[0];
        d_dec = 0.0;
        e_dec = sps;
        f_dec = motor_dec[0] - sps * dec_arcsec[0];
    } else if (s_align_mode == OS_ALIGN_2STAR) {
        if (s_align_count < 2) return OS_ERR_INVALID_STATE;
        if (!fit_line_ls(ra_arcsec, motor_ra, s_align_count, &a_ra, &c_ra)) return OS_ERR_CALIBRATION_FAILED;
        if (!fit_line_ls(dec_arcsec, motor_dec, s_align_count, &e_dec, &f_dec)) return OS_ERR_CALIBRATION_FAILED;
        b_ra = 0.0;
        d_dec = 0.0;
    } else {
        if (s_align_count < 3) return OS_ERR_INVALID_STATE;
        double coeff_ra[3] = { 0.0, 0.0, 0.0 };
        double coeff_dec[3] = { 0.0, 0.0, 0.0 };
        if (!solve_3param_centered(ra_arcsec, dec_arcsec, motor_ra, s_align_count, coeff_ra)) return OS_ERR_CALIBRATION_FAILED;
        if (!solve_3param_centered(ra_arcsec, dec_arcsec, motor_dec, s_align_count, coeff_dec)) return OS_ERR_CALIBRATION_FAILED;
        a_ra = coeff_ra[0];
        b_ra = coeff_ra[1];
        c_ra = coeff_ra[2];
        d_dec = coeff_dec[0];
        e_dec = coeff_dec[1];
        f_dec = coeff_dec[2];
    }

    double sum_sq = 0.0;
    for (int i = 0; i < s_align_count; ++i) {
        double pred_ra = a_ra * ra_arcsec[i] + b_ra * dec_arcsec[i] + c_ra;
        double pred_dec = d_dec * ra_arcsec[i] + e_dec * dec_arcsec[i] + f_dec;
        double err_ra = pred_ra - motor_ra[i];
        double err_dec = pred_dec - motor_dec[i];
        sum_sq += err_ra * err_ra + err_dec * err_dec;
    }
    double rms_steps = sqrt(sum_sq / (double)s_align_count);
    double residual_arcsec = rms_steps / STEPS_PER_ARCSEC;

    if (s_align_count >= 4 && residual_arcsec > (double)OS_ALIGN_MAX_RESIDUAL_ARCSEC) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    s_calib.matrix_ra_to_ra = (float)a_ra;
    s_calib.matrix_ra_to_dec = (float)b_ra;
    s_calib.matrix_dec_to_ra = (float)d_dec;
    s_calib.matrix_dec_to_dec = (float)e_dec;
    s_calib.offset_ra_arcsec = (float)c_ra;
    s_calib.offset_dec_arcsec = (float)f_dec;
    s_calib.valid = true;
    s_calib_valid = true;
    s_align_residual = (float)residual_arcsec;
    s_align_residual_valid = (s_align_count >= 4);
    return OS_ERR_NONE;
}

static os_error_t save_calibration_to_nvm(void) {
    uint8_t buf[OS_NVM_CALIBRATION_SIZE_BYTES];
    memset(buf, 0, sizeof(buf));
    memcpy(buf, s_nvm_calib_magic, sizeof(s_nvm_calib_magic));
    memcpy(buf + 4, &s_calib, sizeof(s_calib));
    return os_hal_nvm_write(0, buf, sizeof(buf));
}

static os_error_t save_park_to_nvm(void) {
    uint8_t buf[OS_NVM_CONFIG_SIZE_BYTES];
    memset(buf, 0, sizeof(buf));
    memcpy(buf, s_nvm_config_magic, sizeof(s_nvm_config_magic));
    buf[4] = s_park_pos_set ? 1 : 0;
    memcpy(buf + 8, &s_park_pos, sizeof(s_park_pos));
    return os_hal_nvm_write(NVM_CONFIG_OFFSET, buf, sizeof(buf));
}

static os_error_t save_pec_to_nvm(void) {
    uint8_t buf[sizeof(os_pec_table_t) + NVM_PEC_MAGIC_SIZE];
    memset(buf, 0, sizeof(buf));
    memcpy(buf, s_nvm_pec_magic, sizeof(s_nvm_pec_magic));
    memcpy(buf + NVM_PEC_MAGIC_SIZE, &s_pec_table, sizeof(s_pec_table));
    return os_hal_nvm_write((uint16_t)NVM_PEC_OFFSET, buf, (uint16_t)sizeof(buf));
}

static void load_calibration_from_nvm(void) {
    uint8_t buf[OS_NVM_CALIBRATION_SIZE_BYTES];
    memset(buf, 0, sizeof(buf));
    if (os_hal_nvm_read(0, buf, sizeof(buf)) != OS_ERR_NONE) return;
    if (memcmp(buf, s_nvm_calib_magic, sizeof(s_nvm_calib_magic)) != 0) return;
    os_calibration_t temp;
    memcpy(&temp, buf + 4, sizeof(temp));
    if (temp.valid) {
        s_calib = temp;
        s_calib_valid = true;
    }
}

static void load_park_from_nvm(void) {
    uint8_t buf[OS_NVM_CONFIG_SIZE_BYTES];
    memset(buf, 0, sizeof(buf));
    if (os_hal_nvm_read(NVM_CONFIG_OFFSET, buf, sizeof(buf)) != OS_ERR_NONE) return;
    if (memcmp(buf, s_nvm_config_magic, sizeof(s_nvm_config_magic)) != 0) return;
    s_park_pos_set = (buf[4] == 1);
    memcpy(&s_park_pos, buf + 8, sizeof(s_park_pos));
}

static void load_pec_from_nvm(void) {
    uint8_t buf[sizeof(os_pec_table_t) + NVM_PEC_MAGIC_SIZE];
    memset(buf, 0, sizeof(buf));
    if (os_hal_nvm_read((uint16_t)NVM_PEC_OFFSET, buf, (uint16_t)sizeof(buf)) != OS_ERR_NONE) return;
    if (memcmp(buf, s_nvm_pec_magic, sizeof(s_nvm_pec_magic)) != 0) return;
    os_pec_table_t temp;
    memcpy(&temp, buf + NVM_PEC_MAGIC_SIZE, sizeof(temp));
    if (temp.valid) {
        s_pec_table = temp;
        s_pec_loaded = true;
    }
}

static void clear_runtime_flags(void) {
    s_state = OS_STATE_INITIALIZING;
    s_init_complete = false;
    s_parked = false;
    s_goto_active = false;
    s_auto_active = false;
    s_manual_active = false;
    s_guide_active = false;
    s_guide_queue_count = 0;
    s_align_count = 0;
    s_align_residual_valid = false;
    s_align_residual = 0.0f;
    s_calib_valid = false;
    s_park_pos_set = false;
    s_tracking_enabled = true;
    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_custom_track_factor = 1.0f;
    s_pec_enabled = false;
    s_pec_loaded = false;
    s_guide_rate = 0.5f;
    s_custom_manual_speed_arcsec_per_sec = 100.0f;
    s_motor_fault[0] = false;
    s_motor_fault[1] = false;
    s_motion_limit_fault = false;
    s_goto_target.ra_hours = 0.0f;
    s_goto_target.dec_degrees = 0.0f;
    s_goto_target_set = false;
    memset(&s_calib, 0, sizeof(s_calib));
    memset(&s_guide_pulse, 0, sizeof(s_guide_pulse));
    memset(&s_pec_table, 0, sizeof(s_pec_table));
    memset(&s_park_pos, 0, sizeof(s_park_pos));
    s_site.latitude_degrees = 0.0f;
    s_site.longitude_degrees = 0.0f;
    s_site.elevation_metres = 0.0f;
    s_site.utc_epoch_seconds = 0;
    s_site.valid = false;
    s_gps_valid = false;
    s_rtc_valid = false;
    s_rtc_epoch = 0;
    s_hal_motor_initialized[0] = false;
    s_hal_motor_initialized[1] = false;
    s_hal_motor_enabled[0] = false;
    s_hal_motor_enabled[1] = false;
    s_hal_motor_direction[0] = true;
    s_hal_motor_direction[1] = true;
    s_hal_motor_freq[0] = 0;
    s_hal_motor_freq[1] = 0;
    s_hal_motor_pos[0] = 0;
    s_hal_motor_pos[1] = 0;
    s_hal_motor_fraction[0] = 0.0;
    s_hal_motor_fraction[1] = 0.0;
    s_hal_limit_triggered[0] = false;
    s_hal_limit_triggered[1] = false;
    s_hal_rtc_epoch = 0;
    s_hal_rtc_initialized = false;
    memset(&s_hal_gps_latest, 0, sizeof(s_hal_gps_latest));
    s_hal_gps_latest_valid = false;
    for (int i = 0; i < 4; ++i) {
        s_hal_comm_initialized[i] = false;
        s_comm_phys_rx_len[i] = 0;
        s_comm_tx_len[i] = 0;
        s_comm_frame_len[i] = 0;
        s_comm_frame_active[i] = false;
    }
}

static void init_one_comm(uint8_t channel) {
    os_error_t err = os_hal_comm_init(channel);
    if (err != OS_ERR_NONE) {
        s_hal_comm_initialized[channel] = false;
        s_comm_phys_rx_len[channel] = 0;
        s_comm_tx_len[channel] = 0;
        s_comm_frame_len[channel] = 0;
        s_comm_frame_active[channel] = false;
    } else {
        s_hal_comm_initialized[channel] = true;
    }
}

os_error_t os_init(void) {
    clear_runtime_flags();
    s_state = OS_STATE_INITIALIZING;

    os_error_t err = os_hal_nvm_init();
    if (err != OS_ERR_NONE) return OS_ERR_NVM_FAULT;
    load_calibration_from_nvm();
    load_park_from_nvm();
    load_pec_from_nvm();

    init_one_comm(OS_CHANNEL_USB);
    init_one_comm(OS_CHANNEL_BLUETOOTH);
    init_one_comm(OS_CHANNEL_WIFI);
    init_one_comm(OS_CHANNEL_ETHERNET);

    err = os_hal_motor_init(0);
    if (err != OS_ERR_NONE) { s_motor_fault[0] = true; s_state = OS_STATE_FAULT; return OS_ERR_MOTOR_DRIVER_FAULT; }
    err = os_hal_motor_init(1);
    if (err != OS_ERR_NONE) { s_motor_fault[1] = true; s_state = OS_STATE_FAULT; return OS_ERR_MOTOR_DRIVER_FAULT; }

    os_hal_gps_init();
    os_hal_rtc_init();
    os_hal_limit_init();
    err = os_hal_timer_motor_init();
    if (err != OS_ERR_NONE) { s_state = OS_STATE_FAULT; return err; }

    uint32_t rtc = 0;
    if (os_hal_rtc_read(&rtc) == OS_ERR_NONE) {
        s_rtc_epoch = rtc;
        s_rtc_valid = true;
        s_site.utc_epoch_seconds = rtc;
    }

    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);
    s_state = OS_STATE_IDLE_TRACKING;
    s_init_complete = true;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    if (!s_init_complete) return;
    poll_time_location();

    bool limit0 = os_hal_limit_is_triggered(0);
    bool limit1 = os_hal_limit_is_triggered(1);
    if (limit0 || limit1) {
        if (s_state != OS_STATE_FAULT) {
            halt_axes();
            s_auto_active = false;
            s_goto_active = false;
            s_manual_active = false;
            s_guide_active = false;
            s_guide_queue_count = 0;
            s_state = OS_STATE_FAULT;
            s_motion_limit_fault = true;
        }
        poll_commands();
        return;
    }

    poll_commands();

    if (s_auto_active) {
        os_motor_position_t cur = current_motor_pos();
        bool arrived = true;
        for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
            int32_t cur_step = (axis == 0) ? cur.ra_steps : cur.dec_steps;
            int32_t diff = s_target_steps[axis] - cur_step;
            if (diff != 0) {
                arrived = false;
                uint32_t freq = (abs(diff) < 200) ? 500 : 3000;
                freq = (uint32_t)clip_speed_to_goto_max((double)freq);
                os_hal_motor_set_direction(axis, diff > 0);
                os_hal_motor_set_frequency(axis, freq);
                simulate_step_for_axis(axis, s_target_steps[axis], true);
            } else {
                os_hal_motor_set_frequency(axis, 0);
            }
        }
        if (arrived) {
            halt_axes();
            s_auto_active = false;
            s_goto_active = false;
            if (s_auto_dest_state == OS_STATE_PARKED) {
                s_parked = true;
                s_tracking_enabled = false;
                os_hal_motor_enable(0, false);
                os_hal_motor_enable(1, false);
                s_state = OS_STATE_PARKED;
            } else {
                s_state = OS_STATE_IDLE_TRACKING;
                s_tracking_enabled = true;
                os_hal_buzzer_beep(100, 1);
            }
        }
    } else if (s_manual_active) {
        if (os_hal_limit_is_triggered(s_manual_axis)) {
            enter_fault_state();
            return;
        }
        os_hal_motor_set_direction(s_manual_axis, s_manual_forward);
        os_hal_motor_set_frequency(s_manual_axis, manual_speed_frequency(s_manual_speed));
        simulate_step_for_axis(s_manual_axis, 0, false);
    } else {
        update_tracking_and_guide();
    }
}

static void set_reply(char *reply_buffer, size_t reply_buffer_size, size_t *reply_length, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(reply_buffer, reply_buffer_size, fmt, ap);
    va_end(ap);
    if (n < 0) n = 0;
    if ((size_t)n >= reply_buffer_size) n = (int)(reply_buffer_size > 0 ? reply_buffer_size - 1 : 0);
    *reply_length = (size_t)n;
}

static bool parse_angular(const char *s, float *value) {
    if (s == NULL || *s == '\0') return false;
    int d = 0, m = 0;
    float sec = 0.0f;
    int matched = sscanf(s, "%d%*1[:*]%d%*1[:*]%f", &d, &m, &sec);
    if (matched == 3) {
        *value = (float)((double)d + (double)m / 60.0 + (double)sec / 3600.0);
        return true;
    }
    matched = sscanf(s, "%d%*1[:*]%d", &d, &m);
    if (matched == 2) {
        *value = (float)((double)d + (double)m / 60.0);
        return true;
    }
    float raw = 0.0f;
    if (sscanf(s, "%f", &raw) == 1) {
        *value = raw;
        return true;
    }
    return false;
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (length == 0 || length > OS_MAX_COMMAND_LENGTH) return OS_ERR_INVALID_ARGUMENT;
    if (source_channel > OS_CHANNEL_ETHERNET) return OS_ERR_INVALID_ARGUMENT;
    if (reply_buffer_size == 0) { *reply_length = 0; return OS_ERR_INVALID_ARGUMENT; }
    if (length < 2 || command[0] != OS_LX200_CMD_PREFIX || command[length - 1] != OS_LX200_CMD_SUFFIX) {
        *reply_length = 0;
        return OS_ERR_COMMAND_FORMAT;
    }

    char body[OS_MAX_COMMAND_LENGTH];
    size_t body_len = length - 2;
    memcpy(body, command + 1, body_len);
    body[body_len] = '\0';
    set_reply(reply_buffer, reply_buffer_size, reply_length, "");

    if (strcmp(body, "GR") == 0) {
        os_equatorial_coord_t coord;
        os_error_t e = os_query_coordinates(&coord);
        if (e != OS_ERR_NONE) return e;
        set_reply(reply_buffer, reply_buffer_size, reply_length, "%02.6f", coord.ra_hours);
        return OS_ERR_NONE;
    } else if (strcmp(body, "GD") == 0) {
        os_equatorial_coord_t coord;
        os_error_t e = os_query_coordinates(&coord);
        if (e != OS_ERR_NONE) return e;
        set_reply(reply_buffer, reply_buffer_size, reply_length, "%+02.6f", coord.dec_degrees);
        return OS_ERR_NONE;
    } else if (strcmp(body, "GVP") == 0) {
        uint8_t major = 0, minor = 0, patch = 0;
        os_error_t e = os_query_firmware_version(&major, &minor, &patch);
        if (e != OS_ERR_NONE) return e;
        set_reply(reply_buffer, reply_buffer_size, reply_length, "%u.%u.%u", major, minor, patch);
        return OS_ERR_NONE;
    } else if (strcmp(body, "GG") == 0) {
        bool locked = false;
        os_error_t e = os_query_gps_locked(&locked);
        if (e != OS_ERR_NONE) return e;
        set_reply(reply_buffer, reply_buffer_size, reply_length, locked ? "1" : "0");
        return OS_ERR_NONE;
    } else if (strcmp(body, "GM") == 0) {
        bool moving = false;
        os_error_t e = os_query_is_moving(&moving);
        if (e != OS_ERR_NONE) return e;
        set_reply(reply_buffer, reply_buffer_size, reply_length, moving ? "1" : "0");
        return OS_ERR_NONE;
    } else if (strcmp(body, "GS") == 0 || strcmp(body, "GSTAT") == 0) {
        os_state_t state = OS_STATE_INITIALIZING;
        os_error_t e = os_query_state(&state);
        if (e != OS_ERR_NONE) return e;
        const char *name = "?";
        switch (state) {
            case OS_STATE_INITIALIZING: name = "INIT"; break;
            case OS_STATE_IDLE_TRACKING: name = "IDLE"; break;
            case OS_STATE_GOTO: name = "GOTO"; break;
            case OS_STATE_ALIGNMENT: name = "ALIGN"; break;
            case OS_STATE_MANUAL_MOTION: name = "MOVE"; break;
            case OS_STATE_PARKED: name = "PARKED"; break;
            case OS_STATE_FAULT: name = "FAULT"; break;
        }
        set_reply(reply_buffer, reply_buffer_size, reply_length, "%s", name);
        return OS_ERR_NONE;
    } else if (strncmp(body, "Sr", 2) == 0) {
        float ra = 0.0f;
        if (!parse_angular(body + 2, &ra)) return OS_ERR_COMMAND_FORMAT;
        if (!valid_ra_hours(ra)) return OS_ERR_INVALID_ARGUMENT;
        s_goto_target.ra_hours = ra;
        s_goto_target_set = true;
        set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        return OS_ERR_NONE;
    } else if (strncmp(body, "Sd", 2) == 0) {
        float dec = 0.0f;
        if (!parse_angular(body + 2, &dec)) return OS_ERR_COMMAND_FORMAT;
        if (!valid_dec_degrees(dec)) return OS_ERR_INVALID_ARGUMENT;
        s_goto_target.dec_degrees = dec;
        s_goto_target_set = true;
        set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        return OS_ERR_NONE;
    } else if (strcmp(body, "MS") == 0) {
        os_error_t e = os_goto_equatorial(s_goto_target);
        if (e == OS_ERR_NONE) set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        else set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
        return e;
    } else if (strcmp(body, "hP") == 0) {
        os_error_t e = os_park();
        if (e == OS_ERR_NONE) set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        else set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
        return e;
    } else if (strcmp(body, "hO") == 0) {
        os_error_t e = os_unpark();
        if (e == OS_ERR_NONE) set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        else set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
        return e;
    } else if (strcmp(body, "Me") == 0) {
        os_error_t e = os_move_start(OS_DIRECTION_EAST, OS_SPEED_FAST);
        if (e == OS_ERR_NONE) set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        else set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
        return e;
    } else if (strcmp(body, "Mw") == 0) {
        os_error_t e = os_move_start(OS_DIRECTION_WEST, OS_SPEED_FAST);
        if (e == OS_ERR_NONE) set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        else set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
        return e;
    } else if (strcmp(body, "Mn") == 0) {
        os_error_t e = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_FAST);
        if (e == OS_ERR_NONE) set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        else set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
        return e;
    } else if (strcmp(body, "Ms") == 0) {
        os_error_t e = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_FAST);
        if (e == OS_ERR_NONE) set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        else set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
        return e;
    } else if (strcmp(body, "Q") == 0) {
        os_error_t e = os_move_stop();
        if (e == OS_ERR_NONE) set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
        else set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
        return e;
    }

    *reply_length = 0;
    return OS_ERR_NOT_SUPPORTED;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (!valid_ra_hours(target.ra_hours) || !valid_dec_degrees(target.dec_degrees)) return OS_ERR_INVALID_ARGUMENT;
    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT || s_state == OS_STATE_INITIALIZING) return OS_ERR_INVALID_STATE;
    if (s_mount == OS_MOUNT_ALTAZ && !has_time_source()) return OS_ERR_INVALID_STATE;
    os_motor_position_t steps = coord_to_motor_pos(target);
    return start_auto_move(steps, OS_STATE_IDLE_TRACKING);
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (!valid_azimuth_degrees(target.azimuth_degrees) || !valid_altitude_degrees(target.altitude_degrees)) return OS_ERR_INVALID_ARGUMENT;
    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT || s_state == OS_STATE_INITIALIZING) return OS_ERR_INVALID_STATE;
    if (s_mount == OS_MOUNT_EQUATORIAL && !has_time_source()) return OS_ERR_INVALID_STATE;
    os_motor_position_t steps = horizontal_to_motor_pos(target);
    return start_auto_move(steps, OS_STATE_IDLE_TRACKING);
}

os_error_t os_goto_abort(void) {
    if (!s_auto_active && !s_goto_active) return OS_ERR_INVALID_STATE;
    halt_axes();
    s_auto_active = false;
    s_goto_active = false;
    s_manual_active = false;
    s_state = OS_STATE_IDLE_TRACKING;
    s_tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (rate < OS_TRACK_RATE_SIDEREAL || rate > OS_TRACK_RATE_CUSTOM) return OS_ERR_INVALID_ARGUMENT;
    if (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f) return OS_ERR_INVALID_ARGUMENT;
    if (rate != OS_TRACK_RATE_CUSTOM) custom_factor = 1.0f;
    s_track_rate = rate;
    s_custom_track_factor = custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (rate == NULL || custom_factor == NULL) return OS_ERR_INVALID_ARGUMENT;
    *rate = s_track_rate;
    *custom_factor = s_custom_track_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT || s_state == OS_STATE_INITIALIZING) return OS_ERR_INVALID_STATE;
    s_tracking_enabled = true;
    if (s_state == OS_STATE_IDLE_TRACKING) {
        os_error_t e = enable_axis_motor(0);
        if (e != OS_ERR_NONE) return e;
        e = enable_axis_motor(1);
        if (e != OS_ERR_NONE) return e;
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    s_tracking_enabled = false;
    if (!s_auto_active && !s_manual_active) halt_axes();
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) return OS_ERR_INVALID_ARGUMENT;
    if (duration_ms == 0) return OS_ERR_INVALID_ARGUMENT;
    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT || s_state == OS_STATE_INITIALIZING) return OS_ERR_INVALID_STATE;

    os_guide_pulse_t pulse;
    memset(&pulse, 0, sizeof(pulse));
    pulse.active = true;
    pulse.duration_ms = duration_ms;
    pulse.rate_fraction = s_guide_rate;
    pulse.direction_east = (direction == OS_DIRECTION_EAST);
    pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    pulse.dec_priority = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);

    if (!s_guide_active) {
        s_guide_pulse = pulse;
        s_guide_active = true;
        return OS_ERR_NONE;
    }
    if (!guide_queue_push(pulse)) return OS_ERR_INVALID_STATE;
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) return OS_ERR_INVALID_ARGUMENT;
    s_guide_rate = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (pulse == NULL) return OS_ERR_INVALID_ARGUMENT;
    *pulse = s_guide_pulse;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) return OS_ERR_INVALID_ARGUMENT;
    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT || s_state == OS_STATE_INITIALIZING) return OS_ERR_INVALID_STATE;
    s_align_mode = mode;
    s_align_count = 0;
    s_align_residual_valid = false;
    s_align_residual = 0.0f;
    s_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (!valid_ra_hours(star_coord.ra_hours) || !valid_dec_degrees(star_coord.dec_degrees)) return OS_ERR_INVALID_ARGUMENT;
    if (s_state != OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE;
    if (s_align_count >= OS_CALIBRATION_MAX_STARS) return OS_ERR_INVALID_STATE;
    s_align_stars[s_align_count] = star_coord;
    s_align_pos[s_align_count] = motor_pos;
    ++s_align_count;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (s_state != OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE;
    unsigned int min_stars = (s_align_mode == OS_ALIGN_1STAR) ? 1 :
                             (s_align_mode == OS_ALIGN_2STAR) ? 2 : 3;
    if ((unsigned int)s_align_count < min_stars) return OS_ERR_INVALID_STATE;

    os_error_t err = compute_alignment();
    if (err != OS_ERR_NONE) return err;
    err = save_calibration_to_nvm();
    if (err != OS_ERR_NONE) return OS_ERR_NVM_FAULT;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (!s_align_residual_valid) return OS_ERR_INVALID_STATE;
    *residual_arcsec = s_align_residual;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    s_align_count = 0;
    s_align_residual_valid = false;
    s_align_residual = 0.0f;
    if (s_state == OS_STATE_ALIGNMENT) s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (s_state == OS_STATE_PARKED) return OS_ERR_INVALID_STATE;
    if (s_state == OS_STATE_FAULT || s_state == OS_STATE_INITIALIZING) return OS_ERR_INVALID_STATE;
    os_equatorial_coord_t p;
    if (s_park_pos_set) p = s_park_pos;
    else { p.ra_hours = 0.0f; p.dec_degrees = 90.0f; }
    os_motor_position_t steps = coord_to_motor_pos(p);
    return start_auto_move(steps, OS_STATE_PARKED);
}

os_error_t os_unpark(void) {
    if (s_state != OS_STATE_PARKED) return OS_ERR_INVALID_STATE;
    os_error_t e0 = enable_axis_motor(0);
    if (e0 != OS_ERR_NONE) return e0;
    os_error_t e1 = enable_axis_motor(1);
    if (e1 != OS_ERR_NONE) return e1;
    for (uint8_t ch = 0; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        init_one_comm(ch);
    }
    poll_time_location();
    if (s_gps_valid) os_hal_rtc_set(s_site.utc_epoch_seconds);
    else if (s_rtc_valid) os_hal_rtc_set(s_rtc_epoch);
    s_parked = false;
    s_tracking_enabled = true;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (!valid_ra_hours(park_pos.ra_hours) || !valid_dec_degrees(park_pos.dec_degrees)) return OS_ERR_INVALID_ARGUMENT;
    s_park_pos = park_pos;
    s_park_pos_set = true;
    return save_park_to_nvm();
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) return OS_ERR_INVALID_ARGUMENT;
    if (speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) return OS_ERR_INVALID_ARGUMENT;
    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT || s_state == OS_STATE_INITIALIZING) return OS_ERR_INVALID_STATE;
    uint8_t axis = 0;
    bool forward = true;
    if (!map_direction_to_axis_forward(direction, &axis, &forward)) return OS_ERR_INVALID_STATE;
    if (os_hal_limit_is_triggered(axis)) return OS_ERR_LIMIT_TRIGGERED;

    s_auto_active = false;
    s_goto_active = false;
    s_guide_active = false;
    s_guide_queue_count = 0;
    s_manual_active = true;
    s_manual_direction = direction;
    s_manual_speed = speed;
    s_manual_axis = axis;
    s_manual_forward = forward;
    s_state = OS_STATE_MANUAL_MOTION;
    os_error_t e = enable_axis_motor(axis);
    if (e != OS_ERR_NONE) return e;
    os_hal_motor_set_direction(axis, forward);
    os_hal_motor_set_frequency(axis, manual_speed_frequency(speed));
    os_hal_motor_set_frequency((uint8_t)(1 - axis), 0);
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    halt_axes();
    s_manual_active = false;
    s_auto_active = false;
    s_goto_active = false;
    if (s_state != OS_STATE_FAULT && s_state != OS_STATE_PARKED && s_state != OS_STATE_INITIALIZING) {
        s_state = OS_STATE_IDLE_TRACKING;
        s_tracking_enabled = true;
    }
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (arcsec_per_sec <= 0.0f) return OS_ERR_INVALID_ARGUMENT;
    s_custom_manual_speed_arcsec_per_sec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (state == NULL) return OS_ERR_INVALID_ARGUMENT;
    *state = s_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (coord == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (s_mount == OS_MOUNT_ALTAZ && !has_time_source()) return OS_ERR_INVALID_STATE;
    *coord = motor_pos_to_coord(current_motor_pos());
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (site == NULL) return OS_ERR_INVALID_ARGUMENT;
    *site = s_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (pos == NULL) return OS_ERR_INVALID_ARGUMENT;
    *pos = current_motor_pos();
    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor, uint8_t *patch) {
    if (major == NULL || minor == NULL || patch == NULL) return OS_ERR_INVALID_ARGUMENT;
    *major = OS_FIRMWARE_VERSION_MAJOR;
    *minor = OS_FIRMWARE_VERSION_MINOR;
    *patch = OS_FIRMWARE_VERSION_PATCH;
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *moving) {
    if (moving == NULL) return OS_ERR_INVALID_ARGUMENT;
    *moving = s_auto_active || s_manual_active || s_guide_active;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (locked == NULL) return OS_ERR_INVALID_ARGUMENT;
    *locked = s_gps_valid;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    s_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL) return OS_ERR_INVALID_ARGUMENT;
    memcpy(&s_pec_table, table, sizeof(s_pec_table));
    s_pec_loaded = true;
    return save_pec_to_nvm();
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (table == NULL) return OS_ERR_INVALID_ARGUMENT;
    memcpy(table, &s_pec_table, sizeof(s_pec_table));
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) return OS_ERR_INVALID_ARGUMENT;
    int idx = (worm_phase_deg >= 360.0f) ? 0 : (int)worm_phase_deg;
    if (idx < 0) idx = 0;
    if (idx >= OS_PEC_TABLE_SIZE) idx = OS_PEC_TABLE_SIZE - 1;
    s_pec_table.corrections[idx] = error_arcsec;
    s_pec_table.valid = true;
    s_pec_loaded = true;
    return save_pec_to_nvm();
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (calib == NULL) return OS_ERR_INVALID_ARGUMENT;
    *calib = s_calib;
    calib->valid = s_calib_valid;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    memset(&s_calib, 0, sizeof(s_calib));
    s_calib_valid = false;
    return save_calibration_to_nvm();
}

os_error_t os_hal_motor_init(uint8_t axis) {
    if (!valid_axis(axis)) return OS_ERR_INVALID_ARGUMENT;
    s_hal_motor_initialized[axis] = true;
    s_hal_motor_enabled[axis] = false;
    s_hal_motor_freq[axis] = 0;
    s_hal_motor_direction[axis] = true;
    s_motor_fault[axis] = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (!valid_axis(axis)) return OS_ERR_INVALID_ARGUMENT;
    if (!s_hal_motor_initialized[axis]) return OS_ERR_MOTOR_DRIVER_FAULT;
    s_hal_motor_freq[axis] = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (!valid_axis(axis)) return OS_ERR_INVALID_ARGUMENT;
    if (!s_hal_motor_initialized[axis]) return OS_ERR_MOTOR_DRIVER_FAULT;
    s_hal_motor_direction[axis] = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (!valid_axis(axis)) return OS_ERR_INVALID_ARGUMENT;
    if (!s_hal_motor_initialized[axis]) return OS_ERR_MOTOR_DRIVER_FAULT;
    s_hal_motor_enabled[axis] = enable;
    if (!enable) s_hal_motor_freq[axis] = 0;
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (!valid_axis(axis)) return 0;
    return s_hal_motor_pos[axis];
}

os_error_t os_hal_gps_init(void) {
    s_hal_gps_latest_valid = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (s_hal_gps_latest_valid) {
        *site = s_hal_gps_latest;
        return OS_ERR_NONE;
    }
    memset(site, 0, sizeof(*site));
    site->valid = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void) {
    s_hal_rtc_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (!s_hal_rtc_initialized) return OS_ERR_INVALID_STATE;
    *utc_epoch_seconds = s_hal_rtc_epoch;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    if (!s_hal_rtc_initialized) return OS_ERR_INVALID_STATE;
    s_hal_rtc_epoch = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    s_hal_limit_triggered[0] = false;
    s_hal_limit_triggered[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (!valid_axis(axis)) return true;
    return s_hal_limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void) {
    if (!s_nvm_initialized) {
        memset(s_nvm, 0, sizeof(s_nvm));
        s_nvm_initialized = true;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (data == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (!s_nvm_initialized) return OS_ERR_NVM_FAULT;
    uint32_t end = (uint32_t)offset + (uint32_t)length;
    if (end > (uint32_t)NVM_TOTAL_SIZE || length == 0) return OS_ERR_INVALID_ARGUMENT;
    memcpy(data, s_nvm + offset, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (data == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (!s_nvm_initialized) return OS_ERR_NVM_FAULT;
    uint32_t end = (uint32_t)offset + (uint32_t)length;
    if (end > (uint32_t)NVM_TOTAL_SIZE || length == 0) return OS_ERR_INVALID_ARGUMENT;
    memcpy(s_nvm + offset, data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (!valid_channel(channel)) return OS_ERR_INVALID_ARGUMENT;
    s_hal_comm_initialized[channel] = true;
    s_comm_phys_rx_len[channel] = 0;
    s_comm_tx_len[channel] = 0;
    s_comm_frame_len[channel] = 0;
    s_comm_frame_active[channel] = false;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if (!valid_channel(channel) || !s_hal_comm_initialized[channel]) return 0;
    return (int16_t)s_comm_phys_rx_len[channel];
}

char os_hal_comm_read(uint8_t channel) {
    if (!valid_channel(channel) || !s_hal_comm_initialized[channel]) return '\0';
    if (s_comm_phys_rx_len[channel] == 0) return '\0';
    char c = (char)s_comm_phys_rx[channel][0];
    --s_comm_phys_rx_len[channel];
    memmove(s_comm_phys_rx[channel], s_comm_phys_rx[channel] + 1, s_comm_phys_rx_len[channel]);
    return c;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (data == NULL && length > 0) return OS_ERR_INVALID_ARGUMENT;
    if (!valid_channel(channel)) return OS_ERR_INVALID_ARGUMENT;
    if (length > COMM_TX_BUF_SIZE) length = COMM_TX_BUF_SIZE;
    if (length > 0) memcpy(s_comm_tx[channel], data, length);
    s_comm_tx_len[channel] = length;
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
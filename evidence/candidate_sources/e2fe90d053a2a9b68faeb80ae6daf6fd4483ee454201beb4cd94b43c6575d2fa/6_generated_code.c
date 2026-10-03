#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OS_AXIS_COUNT                2u
#define OS_CHANNEL_COUNT             4u
#define OS_NVM_TOTAL_SIZE            2048u
#define OS_NVM_CONFIG_OFFSET         OS_NVM_CALIBRATION_SIZE_BYTES
#define OS_NVM_PEC_OFFSET            (OS_NVM_CONFIG_OFFSET + OS_NVM_CONFIG_SIZE_BYTES)
#define OS_NVM_PEC_SIZE_BYTES        768u
#define OS_STEPS_PER_ARCSEC          1.0
#define OS_LOOP_TICK_MS              1u
#define OS_GOTO_EPSILON_STEPS        5
#define OS_GOTO_MIN_STEPS_PER_LOOP   1
#define OS_GOTO_ACCEL_STEPS_PER_LOOP 2
#define OS_GOTO_DECEL_STEPS_PER_LOOP 2
#define OS_GOTO_MAX_FREQ_HZ          ((uint32_t)(OS_GOTO_SPEED_MAX_DEG_PER_SEC * 3600.0f * OS_STEPS_PER_ARCSEC))
#define OS_GOTO_MAX_STEPS_PER_LOOP_F (OS_GOTO_SPEED_MAX_DEG_PER_SEC * 3600.0f * OS_STEPS_PER_ARCSEC * ((float)OS_LOOP_TICK_MS / 1000.0f))
#define OS_GOTO_MAX_STEPS_PER_LOOP   ((uint32_t)(OS_GOTO_MAX_STEPS_PER_LOOP_F + 0.999999f))
#define OS_PARK_STEPS_PER_LOOP       10
#define OS_PARK_FREQ_HZ              10800u
#define OS_MANUAL_TIMEOUT_LOOPS      1000u
#define OS_MOTOR_MAX_FREQ_HZ         1000000u
#define OS_CALIBRATION_MAGIC         0x43414C00u
#define OS_PARK_MAGIC                0x5041524Bu
#define OS_PEC_MAGIC                 0x50454300u
#define OS_CALIBRATION_RESIDUAL_LIMIT_ARCSEC 300.0
#define OS_WORM_PHASE_ADVANCE_DEG    1.0
#define OS_PI 3.14159265358979323846
#define OS_RX_BUFFER_SIZE            256u

typedef enum {
    OS_TIME_SOURCE_NONE = 0,
    OS_TIME_SOURCE_GPS = 1,
    OS_TIME_SOURCE_RTC_PRESET = 2
} os_time_source_t;

typedef struct {
    double ra_as;
    double dec_as;
    double motor_ra;
    double motor_dec;
} align_sample_t;

typedef struct {
    uint32_t magic;
    uint8_t  enabled;
    uint8_t  reserved[3];
    os_pec_table_t table;
} nvm_pec_record_t;

static os_state_t g_system_state = OS_STATE_INITIALIZING;
static os_mount_type_t g_mount_type = OS_MOUNT_EQUATORIAL;

static bool g_motor_initialized[OS_AXIS_COUNT];
static bool g_motor_enabled[OS_AXIS_COUNT];
static bool g_motor_direction_forward[OS_AXIS_COUNT];
static uint32_t g_motor_frequency_hz[OS_AXIS_COUNT];
static int32_t g_motor_position_steps[OS_AXIS_COUNT];
static bool g_motor_init_fault[OS_AXIS_COUNT];

static bool g_limit_initialized;
static bool g_limit_triggered[OS_AXIS_COUNT];
static bool g_limit_inject_active[OS_AXIS_COUNT];
static bool g_limit_inject_value[OS_AXIS_COUNT];

static bool g_gps_initialized;
static bool g_rtc_initialized;
static bool g_nvm_initialized;
static bool g_motor_timer_initialized;
static bool g_comm_initialized[OS_CHANNEL_COUNT];
static bool g_comm_park_suspended[OS_CHANNEL_COUNT];

static os_site_info_t g_gps_site;
static bool g_gps_inject_active;
static os_site_info_t g_gps_inject_site;
static uint32_t g_rtc_epoch = 1700000000u;
static os_site_info_t g_site;
static bool g_gps_locked;
static os_time_source_t g_time_source = OS_TIME_SOURCE_NONE;

static bool g_nvm_read_fault;
static bool g_nvm_write_fault;

static bool g_tracking_enabled;
static os_track_rate_t g_track_rate = OS_TRACK_RATE_SIDEREAL;
static float g_track_custom_factor = 1.0f;

static bool g_goto_active;
static int32_t g_goto_target[OS_AXIS_COUNT];
static bool g_goto_direction_forward[OS_AXIS_COUNT];
static double g_goto_step_inc[OS_AXIS_COUNT];
static double g_goto_accum[OS_AXIS_COUNT];
static int32_t g_goto_decel_distance[OS_AXIS_COUNT];
static bool g_has_equatorial_target;
static os_equatorial_coord_t g_tracking_equ_target;
static bool g_has_horizontal_target;
static os_horizontal_coord_t g_tracking_hz_target;

static bool g_parking_active;
static int32_t g_park_target_steps[OS_AXIS_COUNT];
static bool g_park_direction_forward[OS_AXIS_COUNT];

static bool g_manual_active;
static os_direction_t g_manual_direction = OS_DIRECTION_NORTH;
static os_speed_level_t g_manual_speed = OS_SPEED_SLOW;
static float g_custom_manual_speed = 60.0f;
static uint32_t g_manual_ticks_remaining;
static double g_manual_accum;

static os_guide_pulse_t g_guide_pulse;
static uint32_t g_guide_remaining_ms;
static float g_guide_rate_fraction = 0.5f;

static bool g_align_active;
static os_align_mode_t g_align_mode = OS_ALIGN_1STAR;
static uint8_t g_align_star_count;
static bool g_align_residual_computed;
static double g_align_residual_arcsec;
static align_sample_t g_align_samples[OS_CALIBRATION_MAX_STARS];

static os_calibration_t g_calibration = {
    .matrix_ra_to_ra = 1.0f,
    .matrix_ra_to_dec = 0.0f,
    .matrix_dec_to_ra = 0.0f,
    .matrix_dec_to_dec = 1.0f,
    .offset_ra_arcsec = 0.0f,
    .offset_dec_arcsec = 0.0f,
    .valid = false
};
static double g_calib_matrix_d[6] = {1.0, 0.0, 0.0, 1.0, 0.0, 0.0};

static bool g_park_custom_valid;
static os_equatorial_coord_t g_park_position;

static bool g_pec_enabled;
static os_pec_table_t g_pec_table;
static double g_worm_phase_deg;

static bool g_pending_ra_valid;
static bool g_pending_dec_valid;
static bool g_pending_az_valid;
static bool g_pending_alt_valid;
static float g_pending_ra_hours;
static float g_pending_dec_degrees;
static float g_pending_az_degrees;
static float g_pending_alt_degrees;

static uint8_t g_nvm[OS_NVM_TOTAL_SIZE];

static char g_loop_cmd_buf[OS_CHANNEL_COUNT][OS_MAX_COMMAND_LENGTH + 1u];
static uint8_t g_loop_cmd_len[OS_CHANNEL_COUNT];

static char g_comm_rx[OS_CHANNEL_COUNT][OS_RX_BUFFER_SIZE];
static uint16_t g_comm_rx_head[OS_CHANNEL_COUNT];
static uint16_t g_comm_rx_tail[OS_CHANNEL_COUNT];
static uint16_t g_comm_rx_count[OS_CHANNEL_COUNT];

static char g_comm_tx[OS_CHANNEL_COUNT][OS_MAX_REPLY_LENGTH];
static size_t g_comm_tx_len[OS_CHANNEL_COUNT];

static uint32_t g_loop_tick_ms;
static uint16_t g_buzzer_duration_ms;
static uint8_t g_buzzer_count;

os_error_t os_test_gps_set_site(const os_site_info_t *site);
os_error_t os_test_gps_set_no_signal(void);
os_error_t os_test_limit_set(uint8_t axis, bool triggered);
os_error_t os_test_motor_set_fault(uint8_t axis, bool fault);
os_error_t os_test_nvm_set_fault(bool read_fault, bool write_fault);
os_error_t os_test_comm_rx_push(uint8_t channel, const char *data, size_t length);

static bool valid_axis(uint8_t axis)
{
    return axis < OS_AXIS_COUNT;
}

static bool valid_channel(uint8_t channel)
{
    return channel < OS_CHANNEL_COUNT;
}

static bool valid_direction(os_direction_t direction)
{
    return (direction >= OS_DIRECTION_NORTH) && (direction <= OS_DIRECTION_WEST);
}

static bool valid_speed(os_speed_level_t speed)
{
    return (speed >= OS_SPEED_SLOW) && (speed <= OS_SPEED_CUSTOM);
}

static bool valid_track_rate(os_track_rate_t rate)
{
    return (rate >= OS_TRACK_RATE_SIDEREAL) && (rate <= OS_TRACK_RATE_CUSTOM);
}

static bool valid_align_mode(os_align_mode_t mode)
{
    return (mode >= OS_ALIGN_1STAR) && (mode <= OS_ALIGN_NSTAR);
}

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? (-value) : value;
}

static double wrap_two_pi_deg(double x)
{
    x = fmod(x, 360.0);
    if (x < 0.0) {
        x += 360.0;
    }
    return x;
}

static double wrap_pi_deg(double x)
{
    x = fmod(x, 360.0);
    if (x > 180.0) {
        x -= 360.0;
    } else if (x <= -180.0) {
        x += 360.0;
    }
    return x;
}

static double deg_to_rad(double deg)
{
    return deg * OS_PI / 180.0;
}

static double clamp_sin(double v)
{
    if (v > 1.0) {
        return 1.0;
    }
    if (v < -1.0) {
        return -1.0;
    }
    return v;
}

static void horizontal_to_equatorial_rad(double az_deg, double alt_deg,
                                         double lat_deg, double lon_deg,
                                         double utc,
                                         double *ra_rad, double *dec_rad)
{
    const double omega = 7.2921158553e-5;
    double az = deg_to_rad(az_deg);
    double alt = deg_to_rad(alt_deg);
    double lat = deg_to_rad(lat_deg);
    double lst = fmod(utc * omega + deg_to_rad(lon_deg), 2.0 * OS_PI);
    if (lst < 0.0) {
        lst += 2.0 * OS_PI;
    }

    double sin_dec = sin(alt) * sin(lat) + cos(alt) * cos(lat) * cos(az);
    sin_dec = clamp_sin(sin_dec);
    double dec = asin(sin_dec);

    double y = -cos(alt) * sin(az);
    double x = sin(alt) * cos(lat) - cos(alt) * sin(lat) * cos(az);
    double h = atan2(y, x);

    double ra = lst - h;
    ra = fmod(ra, 2.0 * OS_PI);
    if (ra < 0.0) {
        ra += 2.0 * OS_PI;
    }
    *ra_rad = ra;
    *dec_rad = dec;
}

static void altaz_rates_for_equ(double ra_rad, double dec_rad,
                                double lat_deg, double lon_deg, double utc,
                                double *az_arcsec_per_sec,
                                double *alt_arcsec_per_sec)
{
    const double omega = 7.2921158553e-5;
    double lat = deg_to_rad(lat_deg);
    double lst0 = fmod(utc * omega + deg_to_rad(lon_deg), 2.0 * OS_PI);
    if (lst0 < 0.0) {
        lst0 += 2.0 * OS_PI;
    }

    double h0 = lst0 - ra_rad;
    double sin_alt0 = clamp_sin(sin(lat) * sin(dec_rad) +
                                cos(lat) * cos(dec_rad) * cos(h0));
    double alt0 = asin(sin_alt0);
    double y0 = -cos(dec_rad) * sin(h0);
    double x0 = sin(dec_rad) * cos(lat) - cos(dec_rad) * sin(lat) * cos(h0);
    double az0_deg = wrap_two_pi_deg(atan2(y0, x0) * 180.0 / OS_PI);

    double lst1 = lst0 + omega;
    double h1 = lst1 - ra_rad;
    double sin_alt1 = clamp_sin(sin(lat) * sin(dec_rad) +
                                cos(lat) * cos(dec_rad) * cos(h1));
    double alt1 = asin(sin_alt1);
    double y1 = -cos(dec_rad) * sin(h1);
    double x1 = sin(dec_rad) * cos(lat) - cos(dec_rad) * sin(lat) * cos(h1);
    double az1_deg = wrap_two_pi_deg(atan2(y1, x1) * 180.0 / OS_PI);

    double daz_deg = wrap_pi_deg(az1_deg - az0_deg);
    double dalt_deg = (alt1 - alt0) * 180.0 / OS_PI;

    *az_arcsec_per_sec = daz_deg * 3600.0;
    *alt_arcsec_per_sec = dalt_deg * 3600.0;
}

static void compute_altaz_tracking_rates(double *az_arcsec_per_sec,
                                         double *alt_arcsec_per_sec)
{
    double ra_rad = 0.0;
    double dec_rad = 0.0;

    if (g_has_horizontal_target) {
        horizontal_to_equatorial_rad(
            g_tracking_hz_target.azimuth_degrees,
            g_tracking_hz_target.altitude_degrees,
            (double)g_site.latitude_degrees,
            (double)g_site.longitude_degrees,
            (double)g_site.utc_epoch_seconds,
            &ra_rad, &dec_rad);
    } else {
        if (!g_has_equatorial_target) {
            os_equatorial_coord_t coord;
            steps_to_coord(g_motor_position_steps[0], g_motor_position_steps[1],
                           &coord.ra_hours, &coord.dec_degrees);
            g_tracking_equ_target = coord;
            g_has_equatorial_target = true;
        }
        ra_rad = (double)g_tracking_equ_target.ra_hours * OS_PI / 12.0;
        dec_rad = deg_to_rad((double)g_tracking_equ_target.dec_degrees);
    }

    altaz_rates_for_equ(ra_rad, dec_rad,
                        (double)g_site.latitude_degrees,
                        (double)g_site.longitude_degrees,
                        (double)g_site.utc_epoch_seconds,
                        az_arcsec_per_sec, alt_arcsec_per_sec);
}

static void calibration_set_identity_double(void)
{
    g_calib_matrix_d[0] = 1.0;
    g_calib_matrix_d[1] = 0.0;
    g_calib_matrix_d[2] = 0.0;
    g_calib_matrix_d[3] = 1.0;
    g_calib_matrix_d[4] = 0.0;
    g_calib_matrix_d[5] = 0.0;
    g_calibration.matrix_ra_to_ra = 1.0f;
    g_calibration.matrix_ra_to_dec = 0.0f;
    g_calibration.matrix_dec_to_ra = 0.0f;
    g_calibration.matrix_dec_to_dec = 1.0f;
    g_calibration.offset_ra_arcsec = 0.0f;
    g_calibration.offset_dec_arcsec = 0.0f;
    g_calibration.valid = false;
}

static void calibration_sync_public_from_double(void)
{
    g_calibration.matrix_ra_to_ra = (float)g_calib_matrix_d[0];
    g_calibration.matrix_ra_to_dec = (float)g_calib_matrix_d[1];
    g_calibration.matrix_dec_to_ra = (float)g_calib_matrix_d[2];
    g_calibration.matrix_dec_to_dec = (float)g_calib_matrix_d[3];
    g_calibration.offset_ra_arcsec = (float)g_calib_matrix_d[4];
    g_calibration.offset_dec_arcsec = (float)g_calib_matrix_d[5];
    g_calibration.valid = true;
}

static void calibration_sync_double_from_public(void)
{
    g_calib_matrix_d[0] = (double)g_calibration.matrix_ra_to_ra;
    g_calib_matrix_d[1] = (double)g_calibration.matrix_ra_to_dec;
    g_calib_matrix_d[2] = (double)g_calibration.matrix_dec_to_ra;
    g_calib_matrix_d[3] = (double)g_calibration.matrix_dec_to_dec;
    g_calib_matrix_d[4] = (double)g_calibration.offset_ra_arcsec;
    g_calib_matrix_d[5] = (double)g_calibration.offset_dec_arcsec;
}

static os_error_t nvm_save_calibration(void)
{
    uint8_t buf[OS_NVM_CALIBRATION_SIZE_BYTES];
    uint32_t magic = OS_CALIBRATION_MAGIC;
    memset(buf, 0, sizeof(buf));
    memcpy(buf, &magic, sizeof(magic));
    memcpy(buf + sizeof(magic), &g_calibration, sizeof(g_calibration));
    return os_hal_nvm_write(0, buf, sizeof(buf));
}

static void nvm_load_calibration(void)
{
    uint8_t buf[OS_NVM_CALIBRATION_SIZE_BYTES];
    uint32_t magic = 0;
    os_calibration_t cal;

    memset(buf, 0, sizeof(buf));
    memset(&cal, 0, sizeof(cal));
    if (os_hal_nvm_read(0, buf, sizeof(buf)) != OS_ERR_NONE) {
        calibration_set_identity_double();
        return;
    }
    memcpy(&magic, buf, sizeof(magic));
    if (magic != OS_CALIBRATION_MAGIC) {
        calibration_set_identity_double();
        return;
    }
    memcpy(&cal, buf + sizeof(magic), sizeof(cal));
    if (!cal.valid) {
        calibration_set_identity_double();
        return;
    }
    g_calibration = cal;
    calibration_sync_double_from_public();
}

static os_error_t nvm_save_park_position(void)
{
    uint8_t buf[OS_NVM_CONFIG_SIZE_BYTES];
    uint32_t magic = OS_PARK_MAGIC;
    memset(buf, 0, sizeof(buf));
    memcpy(buf, &magic, sizeof(magic));
    memcpy(buf + sizeof(magic), &g_park_custom_valid, sizeof(g_park_custom_valid));
    memcpy(buf + sizeof(magic) + sizeof(g_park_custom_valid), &g_park_position, sizeof(g_park_position));
    return os_hal_nvm_write(OS_NVM_CONFIG_OFFSET, buf, sizeof(buf));
}

static void nvm_load_park_position(void)
{
    uint8_t buf[OS_NVM_CONFIG_SIZE_BYTES];
    uint32_t magic = 0;
    bool custom_valid = false;
    os_equatorial_coord_t park_pos;

    memset(buf, 0, sizeof(buf));
    memset(&park_pos, 0, sizeof(park_pos));
    if (os_hal_nvm_read(OS_NVM_CONFIG_OFFSET, buf, sizeof(buf)) != OS_ERR_NONE) {
        g_park_custom_valid = false;
        return;
    }
    memcpy(&magic, buf, sizeof(magic));
    if (magic != OS_PARK_MAGIC) {
        g_park_custom_valid = false;
        return;
    }
    memcpy(&custom_valid, buf + sizeof(magic), sizeof(custom_valid));
    memcpy(&park_pos, buf + sizeof(magic) + sizeof(custom_valid), sizeof(park_pos));
    g_park_custom_valid = custom_valid;
    g_park_position = park_pos;
}

static os_error_t nvm_save_pec(void)
{
    nvm_pec_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = OS_PEC_MAGIC;
    rec.enabled = g_pec_enabled ? 1u : 0u;
    rec.table = g_pec_table;
    return os_hal_nvm_write(OS_NVM_PEC_OFFSET, (const uint8_t *)&rec, sizeof(rec));
}

static void nvm_load_pec(void)
{
    nvm_pec_record_t rec;
    memset(&rec, 0, sizeof(rec));
    if (os_hal_nvm_read(OS_NVM_PEC_OFFSET, (uint8_t *)&rec, sizeof(rec)) != OS_ERR_NONE) {
        return;
    }
    if (rec.magic != OS_PEC_MAGIC) {
        return;
    }
    g_pec_enabled = (rec.enabled != 0u);
    g_pec_table = rec.table;
}

static void reset_runtime_flags(void)
{
    uint8_t i;

    g_align_residual_computed = false;
    g_align_active = false;
    g_align_star_count = 0;
    g_align_residual_arcsec = 0.0;
    memset(g_align_samples, 0, sizeof(g_align_samples));

    g_guide_pulse.active = false;
    g_guide_pulse.duration_ms = 0u;
    g_guide_pulse.rate_fraction = 0.5f;
    g_guide_pulse.direction_east = false;
    g_guide_pulse.direction_north = false;
    g_guide_pulse.dec_priority = false;
    g_guide_remaining_ms = 0u;
    g_guide_rate_fraction = 0.5f;

    g_manual_active = false;
    g_manual_ticks_remaining = 0u;
    g_manual_accum = 0.0;

    g_goto_active = false;
    g_parking_active = false;
    g_tracking_enabled = false;
    g_mount_type = OS_MOUNT_EQUATORIAL;
    g_has_equatorial_target = false;
    g_has_horizontal_target = false;

    g_pending_ra_valid = false;
    g_pending_dec_valid = false;
    g_pending_az_valid = false;
    g_pending_alt_valid = false;

    for (i = 0; i < OS_AXIS_COUNT; i++) {
        g_goto_step_inc[i] = 0.0;
        g_goto_accum[i] = 0.0;
        g_goto_decel_distance[i] = 0;
        g_goto_target[i] = 0;
        g_park_target_steps[i] = 0;
        g_motor_initialized[i] = false;
        g_motor_enabled[i] = false;
        g_motor_direction_forward[i] = false;
        g_motor_frequency_hz[i] = 0u;
        g_motor_position_steps[i] = 0;
    }

    g_pec_enabled = false;
    memset(&g_pec_table, 0, sizeof(g_pec_table));
    g_pec_table.valid = false;
    g_worm_phase_deg = 0.0;

    g_gps_locked = false;
    g_time_source = OS_TIME_SOURCE_NONE;
    memset(&g_site, 0, sizeof(g_site));
    memset(&g_gps_site, 0, sizeof(g_gps_site));
}

static double current_tracking_arcsec_per_sec(void)
{
    switch (g_track_rate) {
    case OS_TRACK_RATE_SIDEREAL:
        return OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
    case OS_TRACK_RATE_LUNAR:
        return (double)OS_SIDEREAL_RATE_ARCSEC_PER_SEC * (double)OS_LUNAR_RATE_FACTOR;
    case OS_TRACK_RATE_SOLAR:
        return (double)OS_SIDEREAL_RATE_ARCSEC_PER_SEC * (double)OS_SOLAR_RATE_FACTOR;
    case OS_TRACK_RATE_CUSTOM:
        return (double)OS_SIDEREAL_RATE_ARCSEC_PER_SEC * (double)g_track_custom_factor;
    default:
        return 0.0;
    }
}

static double pec_correction_arcsec_per_sec(void)
{
    int index;
    if (!g_pec_enabled || !g_pec_table.valid) {
        return 0.0;
    }
    index = (int)g_worm_phase_deg;
    if (index < 0) {
        index = 0;
    }
    if (index >= OS_PEC_TABLE_SIZE) {
        index = index % OS_PEC_TABLE_SIZE;
    }
    return (double)g_pec_table.corrections[index];
}

static void pec_advance_phase(void)
{
    g_worm_phase_deg += OS_WORM_PHASE_ADVANCE_DEG;
    if (g_worm_phase_deg >= 360.0) {
        g_worm_phase_deg -= 360.0;
    }
}

static void coord_to_steps(os_equatorial_coord_t target, int32_t *ra_steps, int32_t *dec_steps)
{
    double ra_as = (double)target.ra_hours * 15.0 * 3600.0;
    double dec_as = (double)target.dec_degrees * 3600.0;

    if (g_calibration.valid) {
        *ra_steps = (int32_t)lround(g_calib_matrix_d[0] * ra_as + g_calib_matrix_d[1] * dec_as + g_calib_matrix_d[4]);
        *dec_steps = (int32_t)lround(g_calib_matrix_d[2] * ra_as + g_calib_matrix_d[3] * dec_as + g_calib_matrix_d[5]);
    } else {
        *ra_steps = (int32_t)lround(ra_as * OS_STEPS_PER_ARCSEC);
        *dec_steps = (int32_t)lround(dec_as * OS_STEPS_PER_ARCSEC);
    }
}

static void steps_to_coord(int32_t ra_steps, int32_t dec_steps, float *ra_hours, float *dec_degrees)
{
    double ra_as;
    double dec_as;
    double ra_h;
    double dec_d;

    if (g_calibration.valid) {
        double a = g_calib_matrix_d[0];
        double b = g_calib_matrix_d[1];
        double c = g_calib_matrix_d[2];
        double d = g_calib_matrix_d[3];
        double det = (a * d) - (b * c);
        double dx = (double)ra_steps - g_calib_matrix_d[4];
        double dy = (double)dec_steps - g_calib_matrix_d[5];
        if (fabs(det) > 1e-12) {
            ra_as = ((d * dx) - (b * dy)) / det;
            dec_as = ((-c * dx) + (a * dy)) / det;
        } else {
            ra_as = dx;
            dec_as = dy;
        }
    } else {
        ra_as = (double)ra_steps / OS_STEPS_PER_ARCSEC;
        dec_as = (double)dec_steps / OS_STEPS_PER_ARCSEC;
    }

    ra_h = ra_as / (15.0 * 3600.0);
    ra_h = fmod(ra_h, 24.0);
    if (ra_h < 0.0) {
        ra_h += 24.0;
    }
    dec_d = dec_as / 3600.0;
    if (dec_d > 90.0) {
        dec_d = 90.0;
    }
    if (dec_d < -90.0) {
        dec_d = -90.0;
    }
    *ra_hours = (float)ra_h;
    *dec_degrees = (float)dec_d;
}

static bool site_gps_ranges_valid(const os_site_info_t *site)
{
    if (!site->valid) {
        return false;
    }
    if (site->latitude_degrees < -90.0f || site->latitude_degrees > 90.0f) {
        return false;
    }
    if (site->longitude_degrees < -180.0f || site->longitude_degrees > 180.0f) {
        return false;
    }
    return isfinite((double)site->elevation_metres);
}

static uint8_t manual_axis_for_direction(os_direction_t direction)
{
    return ((direction == OS_DIRECTION_NORTH) || (direction == OS_DIRECTION_SOUTH)) ? 1u : 0u;
}

static bool manual_forward_for_direction(os_direction_t direction)
{
    return (direction == OS_DIRECTION_NORTH) || (direction == OS_DIRECTION_EAST);
}

static double manual_speed_arcsec_per_sec(void)
{
    switch (g_manual_speed) {
    case OS_SPEED_SLOW:
        return 100.0;
    case OS_SPEED_MEDIUM:
        return 1000.0;
    case OS_SPEED_FAST:
        return 5000.0;
    case OS_SPEED_CUSTOM:
    default:
        return (double)g_custom_manual_speed;
    }
}

static uint8_t guide_axis(void)
{
    if (g_guide_pulse.dec_priority) {
        return 1u;
    }
    return 0u;
}

static bool guide_forward(void)
{
    if (g_guide_pulse.dec_priority) {
        return g_guide_pulse.direction_north;
    }
    return g_guide_pulse.direction_east;
}

static void motion_apply_tracking(void)
{
    if (!g_tracking_enabled) {
        os_hal_motor_set_frequency(0u, 0u);
        os_hal_motor_set_frequency(1u, 0u);
        return;
    }

    if (g_mount_type == OS_MOUNT_ALTAZ) {
        double az_rate_arcsec = 0.0;
        double alt_rate_arcsec = 0.0;
        double rate_factor = current_tracking_arcsec_per_sec() / (double)OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
        uint32_t az_freq;
        uint32_t alt_freq;

        compute_altaz_tracking_rates(&az_rate_arcsec, &alt_rate_arcsec);
        az_rate_arcsec *= rate_factor;
        alt_rate_arcsec *= rate_factor;

        az_freq = (uint32_t)lround(fabs(az_rate_arcsec) * OS_STEPS_PER_ARCSEC);
        alt_freq = (uint32_t)lround(fabs(alt_rate_arcsec) * OS_STEPS_PER_ARCSEC);
        if (az_freq > OS_MOTOR_MAX_FREQ_HZ) {
            az_freq = OS_MOTOR_MAX_FREQ_HZ;
        }
        if (alt_freq > OS_MOTOR_MAX_FREQ_HZ) {
            alt_freq = OS_MOTOR_MAX_FREQ_HZ;
        }

        os_hal_motor_enable(0u, true);
        os_hal_motor_enable(1u, true);
        os_hal_motor_set_direction(0u, az_rate_arcsec >= 0.0);
        os_hal_motor_set_direction(1u, alt_rate_arcsec >= 0.0);
        os_hal_motor_set_frequency(0u, az_freq);
        os_hal_motor_set_frequency(1u, alt_freq);
    } else {
        double rate_arcsec_per_sec = current_tracking_arcsec_per_sec() + pec_correction_arcsec_per_sec();
        uint32_t base_frequency_hz;

        if (rate_arcsec_per_sec < 0.0) {
            rate_arcsec_per_sec = 0.0;
        }
        base_frequency_hz = (uint32_t)lround(rate_arcsec_per_sec * OS_STEPS_PER_ARCSEC);

        os_hal_motor_enable(0u, true);
        os_hal_motor_set_direction(0u, true);
        os_hal_motor_set_frequency(0u, base_frequency_hz);
        os_hal_motor_set_frequency(1u, 0u);
    }
}

static void motion_stop_slew(void)
{
    os_hal_motor_set_frequency(0u, 0u);
    os_hal_motor_set_frequency(1u, 0u);
}

static bool motion_check_faults(void)
{
    uint8_t i;
    for (i = 0; i < OS_AXIS_COUNT; i++) {
        if (os_hal_limit_is_triggered(i)) {
            g_goto_active = false;
            g_parking_active = false;
            g_manual_active = false;
            g_guide_pulse.active = false;
            g_guide_remaining_ms = 0u;
            g_system_state = OS_STATE_FAULT;
            os_hal_motor_set_frequency(i, 0u);
            os_hal_motor_enable(i, false);
            return true;
        }
    }
    return false;
}

static int32_t goto_profile_compute_step(uint8_t axis, int32_t remaining)
{
    int32_t abs_rem = abs_i32(remaining);
    double velocity = g_goto_step_inc[axis];

    if ((double)abs_rem > (double)g_goto_decel_distance[axis]) {
        velocity += OS_GOTO_ACCEL_STEPS_PER_LOOP;
    } else {
        velocity -= OS_GOTO_DECEL_STEPS_PER_LOOP;
    }

    if (velocity < (double)OS_GOTO_MIN_STEPS_PER_LOOP) {
        velocity = (double)OS_GOTO_MIN_STEPS_PER_LOOP;
    }
    if (velocity > OS_GOTO_MAX_STEPS_PER_LOOP_F) {
        velocity = OS_GOTO_MAX_STEPS_PER_LOOP_F;
    }
    if (velocity > (double)abs_rem) {
        velocity = (double)abs_rem;
    }

    g_goto_step_inc[axis] = velocity;
    g_goto_accum[axis] += velocity;
    {
        int32_t magnitude = (int32_t)g_goto_accum[axis];
        g_goto_accum[axis] -= (double)magnitude;
        if (magnitude == 0) {
            magnitude = 1;
            g_goto_accum[axis] = 0.0;
        }
        if (magnitude > abs_rem) {
            magnitude = abs_rem;
        }
        return (remaining > 0) ? magnitude : -magnitude;
    }
}

static void goto_init_profiles(const int32_t target_steps[OS_AXIS_COUNT])
{
    uint8_t i;
    for (i = 0; i < OS_AXIS_COUNT; i++) {
        int32_t distance = abs_i32(target_steps[i] - g_motor_position_steps[i]);
        double decel = (OS_GOTO_MAX_STEPS_PER_LOOP_F * OS_GOTO_MAX_STEPS_PER_LOOP_F) /
                       (2.0 * (double)OS_GOTO_DECEL_STEPS_PER_LOOP);
        g_goto_step_inc[i] = (double)OS_GOTO_MIN_STEPS_PER_LOOP;
        g_goto_accum[i] = 0.0;
        if (decel > ((double)distance / 2.0)) {
            decel = (double)distance / 2.0;
        }
        if (decel < (double)OS_GOTO_MIN_STEPS_PER_LOOP) {
            decel = (double)OS_GOTO_MIN_STEPS_PER_LOOP;
        }
        g_goto_decel_distance[i] = (int32_t)decel;
    }
}

static void motion_update(void)
{
    uint8_t i;

    if (g_guide_pulse.active) {
        if (g_guide_remaining_ms > OS_LOOP_TICK_MS) {
            g_guide_remaining_ms -= OS_LOOP_TICK_MS;
        } else {
            g_guide_remaining_ms = 0u;
        }

        if (g_guide_remaining_ms > 0u) {
            uint8_t axis = guide_axis();
            bool forward = guide_forward();
            double base = current_tracking_arcsec_per_sec();
            double offset = base * (double)g_guide_rate_fraction;
            double applied = forward ? (base + offset) : (base - offset);
            if (applied < 0.0) {
                applied = 0.0;
            }
            os_hal_motor_set_direction(axis, forward);
            os_hal_motor_set_frequency(axis, (uint32_t)lround(applied));
        } else {
            g_guide_pulse.active = false;
            g_guide_pulse.duration_ms = 0u;
            if (g_tracking_enabled) {
                motion_apply_tracking();
            } else {
                motion_stop_slew();
            }
        }
    }

    if (g_goto_active) {
        bool done = true;
        for (i = 0; i < OS_AXIS_COUNT; i++) {
            int32_t remaining = g_goto_target[i] - g_motor_position_steps[i];
            if (abs_i32(remaining) <= OS_GOTO_EPSILON_STEPS) {
                os_hal_motor_set_frequency(i, 0u);
                continue;
            }
            done = false;
            {
                int32_t step = goto_profile_compute_step(i, remaining);
                g_motor_position_steps[i] += step;
                os_hal_motor_set_direction(i, step > 0);
                if (step != 0) {
                    uint32_t frequency_hz = (uint32_t)lround(g_goto_step_inc[i] * 1000.0);
                    if (frequency_hz > OS_GOTO_MAX_FREQ_HZ) {
                        frequency_hz = OS_GOTO_MAX_FREQ_HZ;
                    }
                    if (frequency_hz > OS_MOTOR_MAX_FREQ_HZ) {
                        frequency_hz = OS_MOTOR_MAX_FREQ_HZ;
                    }
                    os_hal_motor_set_frequency(i, frequency_hz);
                }
            }
        }
        if (done) {
            g_goto_active = false;
            g_system_state = OS_STATE_IDLE_TRACKING;
            os_hal_buzzer_beep(50u, 1u);
            motion_stop_slew();
            if (g_tracking_enabled) {
                motion_apply_tracking();
            }
        }
    }

    if (g_parking_active) {
        bool done = true;
        for (i = 0; i < OS_AXIS_COUNT; i++) {
            int32_t remaining = g_park_target_steps[i] - g_motor_position_steps[i];
            if (abs_i32(remaining) <= OS_GOTO_EPSILON_STEPS) {
                os_hal_motor_set_frequency(i, 0u);
                continue;
            }
            done = false;
            {
                int32_t step = (remaining > 0) ? OS_PARK_STEPS_PER_LOOP : -OS_PARK_STEPS_PER_LOOP;
                if (abs_i32(remaining) < OS_PARK_STEPS_PER_LOOP) {
                    step = remaining;
                }
                g_motor_position_steps[i] += step;
                os_hal_motor_set_direction(i, step > 0);
                os_hal_motor_set_frequency(i, OS_PARK_FREQ_HZ);
            }
        }
        if (done) {
            g_parking_active = false;
            g_tracking_enabled = false;
            g_system_state = OS_STATE_PARKED;
            os_hal_buzzer_beep(50u, 1u);
            for (i = 0; i < OS_AXIS_COUNT; i++) {
                os_hal_motor_set_frequency(i, 0u);
                os_hal_motor_enable(i, false);
            }
            for (i = 1u; i < OS_CHANNEL_COUNT; i++) {
                g_comm_initialized[i] = false;
                g_comm_park_suspended[i] = true;
                g_comm_rx_count[i] = 0u;
                g_comm_rx_head[i] = 0u;
                g_comm_rx_tail[i] = 0u;
                g_comm_tx_len[i] = 0u;
            }
        }
    }

    if (g_manual_active) {
        uint8_t axis = manual_axis_for_direction(g_manual_direction);
        bool forward = manual_forward_for_direction(g_manual_direction);
        double speed = manual_speed_arcsec_per_sec();

        os_hal_motor_set_direction(axis, forward);
        os_hal_motor_set_frequency(axis, (uint32_t)lround(speed));

        g_manual_accum += speed * OS_STEPS_PER_ARCSEC * ((double)OS_LOOP_TICK_MS / 1000.0);
        {
            int32_t increment = (int32_t)g_manual_accum;
            if (increment != 0) {
                g_manual_accum -= (double)increment;
                int32_t delta = forward ? increment : -increment;
                g_motor_position_steps[axis] += delta;
            }
        }

        if (g_manual_ticks_remaining > 0u) {
            g_manual_ticks_remaining--;
        }
        if (g_manual_ticks_remaining == 0u) {
            g_manual_active = false;
            g_system_state = OS_STATE_IDLE_TRACKING;
            if (g_tracking_enabled) {
                motion_apply_tracking();
            } else {
                motion_stop_slew();
            }
        }
    }

    if (!g_goto_active && !g_parking_active && !g_manual_active && !g_guide_pulse.active) {
        if (g_system_state == OS_STATE_IDLE_TRACKING) {
            motion_apply_tracking();
        }
    }
}

static void poll_time_site(void)
{
    os_site_info_t site;
    memset(&site, 0, sizeof(site));
    if (os_hal_gps_poll(&site) == OS_ERR_NONE && site.valid && site_gps_ranges_valid(&site)) {
        g_gps_site = site;
        g_site = site;
        g_site.valid = true;
        g_gps_locked = true;
        g_time_source = OS_TIME_SOURCE_GPS;
        os_hal_rtc_set(site.utc_epoch_seconds);
    } else if (g_time_source == OS_TIME_SOURCE_GPS) {
        g_site.utc_epoch_seconds = g_rtc_epoch;
        g_site.valid = false;
        g_gps_locked = false;
    } else {
        g_site.utc_epoch_seconds = g_rtc_epoch;
    }
}

static void poll_commands(void)
{
    uint8_t channel;
    for (channel = 0u; channel < OS_CHANNEL_COUNT; channel++) {
        if (!g_comm_initialized[channel]) {
            continue;
        }
        while (os_hal_comm_available(channel) > 0) {
            char byte = os_hal_comm_read(channel);

            if (g_loop_cmd_len[channel] == 0u) {
                if (byte != OS_LX200_CMD_PREFIX) {
                    continue;
                }
                g_loop_cmd_buf[channel][0] = byte;
                g_loop_cmd_len[channel] = 1u;
                continue;
            }

            if (byte == OS_LX200_CMD_PREFIX) {
                g_loop_cmd_buf[channel][0] = byte;
                g_loop_cmd_len[channel] = 1u;
                continue;
            }

            if (g_loop_cmd_len[channel] >= OS_MAX_COMMAND_LENGTH) {
                g_loop_cmd_len[channel] = 0u;
                continue;
            }

            g_loop_cmd_buf[channel][g_loop_cmd_len[channel]++] = byte;
            if (byte == OS_LX200_CMD_SUFFIX) {
                char reply[OS_MAX_REPLY_LENGTH];
                size_t reply_len = 0u;
                memset(reply, 0, sizeof(reply));
                (void)os_command_parse(g_loop_cmd_buf[channel], g_loop_cmd_len[channel], channel,
                                       reply, sizeof(reply), &reply_len);
                g_loop_cmd_len[channel] = 0u;
            }
        }
    }
}

static bool alignment_first_three_collinear(const align_sample_t *samples, uint8_t count)
{
    double dx1;
    double dy1;
    double dx2;
    double dy2;
    double cross;
    double len1;
    double len2;
    double sin_angle;

    if (count < 3u) {
        return false;
    }
    dx1 = samples[1].ra_as - samples[0].ra_as;
    dy1 = samples[1].dec_as - samples[0].dec_as;
    dx2 = samples[2].ra_as - samples[0].ra_as;
    dy2 = samples[2].dec_as - samples[0].dec_as;
    cross = (dx1 * dy2) - (dy1 * dx2);
    len1 = hypot(dx1, dy1);
    len2 = hypot(dx2, dy2);
    if (len1 < 1e-9 || len2 < 1e-9) {
        return true;
    }
    sin_angle = fabs(cross) / (len1 * len2);
    return sin_angle < 1e-6;
}

static bool qr_solve_3(const double a[][3], const double *b, double *x, uint8_t n)
{
    double q[OS_CALIBRATION_MAX_STARS][3];
    double r[3][3] = {{0}};
    double qtb[3] = {0};
    uint8_t i;
    uint8_t j;
    uint8_t row;

    for (row = 0; row < n; row++) {
        for (j = 0; j < 3; j++) {
            q[row][j] = a[row][j];
        }
    }

    for (i = 0; i < 3; i++) {
        double norm = 0.0;
        for (row = 0; row < n; row++) {
            norm += q[row][i] * q[row][i];
        }
        norm = sqrt(norm);
        if (norm < 1e-12) {
            return false;
        }
        r[i][i] = norm;
        for (row = 0; row < n; row++) {
            q[row][i] /= norm;
        }
        for (j = (uint8_t)(i + 1u); j < 3; j++) {
            double dot = 0.0;
            for (row = 0; row < n; row++) {
                dot += q[row][i] * q[row][j];
            }
            r[i][j] = dot;
            for (row = 0; row < n; row++) {
                q[row][j] -= dot * q[row][i];
            }
        }
    }

    for (i = 0; i < 3; i++) {
        double dot = 0.0;
        for (row = 0; row < n; row++) {
            dot += q[row][i] * b[row];
        }
        qtb[i] = dot;
    }

    for (i = 3u; i > 0u; i--) {
        uint8_t idx = (uint8_t)(i - 1u);
        double sum = qtb[idx];
        for (j = (uint8_t)(idx + 1u); j < 3; j++) {
            sum -= r[idx][j] * x[j];
        }
        x[idx] = sum / r[idx][idx];
    }
    return true;
}

static os_error_t align_compute_internal(void)
{
    uint8_t n = g_align_star_count;
    uint8_t minimum;
    uint8_t i;

    if (g_align_mode == OS_ALIGN_1STAR) {
        minimum = 1u;
    } else if (g_align_mode == OS_ALIGN_2STAR) {
        minimum = 2u;
    } else {
        minimum = 3u;
    }
    if (n < minimum) {
        return OS_ERR_INVALID_STATE;
    }

    if (g_align_mode == OS_ALIGN_1STAR) {
        double off_ra = g_align_samples[0].motor_ra - g_align_samples[0].ra_as;
        double off_dec = g_align_samples[0].motor_dec - g_align_samples[0].dec_as;
        g_calib_matrix_d[0] = 1.0;
        g_calib_matrix_d[1] = 0.0;
        g_calib_matrix_d[2] = 0.0;
        g_calib_matrix_d[3] = 1.0;
        g_calib_matrix_d[4] = off_ra;
        g_calib_matrix_d[5] = off_dec;
        calibration_sync_public_from_double();
        g_align_residual_arcsec = 0.0;
        g_align_residual_computed = true;
        (void)nvm_save_calibration();
        return OS_ERR_NONE;
    }

    if (g_align_mode == OS_ALIGN_2STAR) {
        double den_ra = g_align_samples[1].ra_as - g_align_samples[0].ra_as;
        double den_dec = g_align_samples[1].dec_as - g_align_samples[0].dec_as;
        double m0;
        double b0;
        double m1;
        double b1;
        if (fabs(den_ra) < 1e-12 || fabs(den_dec) < 1e-12) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        m0 = (g_align_samples[1].motor_ra - g_align_samples[0].motor_ra) / den_ra;
        b0 = g_align_samples[0].motor_ra - (m0 * g_align_samples[0].ra_as);
        m1 = (g_align_samples[1].motor_dec - g_align_samples[0].motor_dec) / den_dec;
        b1 = g_align_samples[0].motor_dec - (m1 * g_align_samples[0].dec_as);
        g_calib_matrix_d[0] = m0;
        g_calib_matrix_d[1] = 0.0;
        g_calib_matrix_d[2] = 0.0;
        g_calib_matrix_d[3] = m1;
        g_calib_matrix_d[4] = b0;
        g_calib_matrix_d[5] = b1;
        calibration_sync_public_from_double();
        g_align_residual_arcsec = 0.0;
        g_align_residual_computed = true;
        (void)nvm_save_calibration();
        return OS_ERR_NONE;
    }

    if (alignment_first_three_collinear(g_align_samples, n)) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    {
        double a[OS_CALIBRATION_MAX_STARS][3];
        double b_ra[OS_CALIBRATION_MAX_STARS];
        double b_dec[OS_CALIBRATION_MAX_STARS];
        double x_ra[3] = {0};
        double x_dec[3] = {0};
        double residual_avg;

        for (i = 0; i < n; i++) {
            a[i][0] = g_align_samples[i].ra_as;
            a[i][1] = g_align_samples[i].dec_as;
            a[i][2] = 1.0;
            b_ra[i] = g_align_samples[i].motor_ra;
            b_dec[i] = g_align_samples[i].motor_dec;
        }

        if (!qr_solve_3(a, b_ra, x_ra, n) || !qr_solve_3(a, b_dec, x_dec, n)) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        g_calib_matrix_d[0] = x_ra[0];
        g_calib_matrix_d[1] = x_ra[1];
        g_calib_matrix_d[2] = x_dec[0];
        g_calib_matrix_d[3] = x_dec[1];
        g_calib_matrix_d[4] = x_ra[2];
        g_calib_matrix_d[5] = x_dec[2];
        calibration_sync_public_from_double();

        if (n == 3u) {
            residual_avg = 0.0;
        } else {
            double residual_sum = 0.0;
            for (i = 0; i < n; i++) {
                double pred_ra = (x_ra[0] * g_align_samples[i].ra_as) +
                                 (x_ra[1] * g_align_samples[i].dec_as) + x_ra[2];
                double pred_dec = (x_dec[0] * g_align_samples[i].ra_as) +
                                  (x_dec[1] * g_align_samples[i].dec_as) + x_dec[2];
                double err_ra = pred_ra - g_align_samples[i].motor_ra;
                double err_dec = pred_dec - g_align_samples[i].motor_dec;
                residual_sum += hypot(err_ra, err_dec);
            }
            residual_avg = residual_sum / (double)n;
            if (residual_avg > OS_CALIBRATION_RESIDUAL_LIMIT_ARCSEC) {
                return OS_ERR_CALIBRATION_FAILED;
            }
        }

        g_align_residual_arcsec = residual_avg;
        g_align_residual_computed = true;
        (void)nvm_save_calibration();
        return OS_ERR_NONE;
    }
}

os_error_t os_hal_motor_init(uint8_t axis)
{
    if (!valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_motor_init_fault[axis]) {
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    g_motor_initialized[axis] = true;
    g_motor_enabled[axis] = false;
    g_motor_frequency_hz[axis] = 0u;
    g_motor_direction_forward[axis] = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz)
{
    if (!valid_axis(axis) || frequency_hz > OS_MOTOR_MAX_FREQ_HZ) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_motor_frequency_hz[axis] = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward)
{
    if (!valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_motor_direction_forward[axis] = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable)
{
    if (!valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_motor_enabled[axis] = enable;
    if (!enable) {
        g_motor_frequency_hz[axis] = 0u;
    }
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis)
{
    if (!valid_axis(axis)) {
        return 0;
    }
    return g_motor_position_steps[axis];
}

os_error_t os_hal_gps_init(void)
{
    g_gps_initialized = true;
    memset(&g_gps_site, 0, sizeof(g_gps_site));
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_gps_inject_active) {
        *site = g_gps_inject_site;
    } else {
        *site = g_gps_site;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void)
{
    g_rtc_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds)
{
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_rtc_initialized) {
        return OS_ERR_TIMEOUT;
    }
    *utc_epoch_seconds = g_rtc_epoch;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds)
{
    if (!g_rtc_initialized) {
        return OS_ERR_TIMEOUT;
    }
    g_rtc_epoch = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void)
{
    g_limit_initialized = true;
    g_limit_triggered[0] = false;
    g_limit_triggered[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis)
{
    if (axis >= OS_AXIS_COUNT) {
        return true;
    }
    if (g_limit_inject_active[axis]) {
        return g_limit_inject_value[axis];
    }
    return g_limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void)
{
    g_nvm_initialized = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length)
{
    uint32_t end;
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    end = (uint32_t)offset + (uint32_t)length;
    if (end > OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_nvm_read_fault) {
        return OS_ERR_NVM_FAULT;
    }
    memcpy(data, &g_nvm[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length)
{
    uint32_t end;
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    end = (uint32_t)offset + (uint32_t)length;
    if (end > OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_nvm_write_fault) {
        return OS_ERR_NVM_FAULT;
    }
    memcpy(&g_nvm[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel)
{
    if (!valid_channel(channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_comm_initialized[channel] = true;
    g_comm_park_suspended[channel] = false;
    g_comm_rx_head[channel] = 0u;
    g_comm_rx_tail[channel] = 0u;
    g_comm_rx_count[channel] = 0u;
    g_comm_tx_len[channel] = 0u;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel)
{
    if (!valid_channel(channel) || !g_comm_initialized[channel]) {
        return 0;
    }
    return (int16_t)g_comm_rx_count[channel];
}

char os_hal_comm_read(uint8_t channel)
{
    char byte;
    if (!valid_channel(channel) || !g_comm_initialized[channel] || g_comm_rx_count[channel] == 0u) {
        return '\0';
    }
    byte = g_comm_rx[channel][g_comm_rx_head[channel]];
    g_comm_rx_head[channel] = (uint16_t)((g_comm_rx_head[channel] + 1u) % OS_RX_BUFFER_SIZE);
    g_comm_rx_count[channel]--;
    return byte;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length)
{
    if (!valid_channel(channel) || data == NULL || length > OS_MAX_REPLY_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_comm_initialized[channel]) {
        return OS_ERR_NOT_SUPPORTED;
    }
    memcpy(g_comm_tx[channel], data, length);
    g_comm_tx_len[channel] = length;
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count)
{
    g_buzzer_duration_ms = duration_ms;
    g_buzzer_count = count;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void)
{
    g_motor_timer_initialized = true;
    return OS_ERR_NONE;
}

static void copy_reply(const char *reply,
                       char *reply_buffer, size_t reply_buffer_size,
                       size_t *reply_length)
{
    size_t source_len = strlen(reply);
    size_t written = 0u;

    if (reply_buffer_size > 0u) {
        written = (source_len < reply_buffer_size) ? source_len : (reply_buffer_size - 1u);
        if (written > 0u) {
            memcpy(reply_buffer, reply, written);
        }
        reply_buffer[written] = '\0';
    }
    *reply_length = written;
}

static bool parse_ra_substr(const char *s, size_t len, float *value)
{
    char num[32];
    int h = 0;
    int m = 0;
    float sm = 0.0f;
    float v = 0.0f;

    if (len == 0u || len >= sizeof(num)) {
        return false;
    }
    memcpy(num, s, len);
    num[len] = '\0';

    if (sscanf(num, "%d:%d:%f", &h, &m, &sm) == 3) {
        v = (float)h + (float)m / 60.0f + sm / 3600.0f;
    } else if (sscanf(num, "%d:%f", &h, &sm) == 2) {
        v = (float)h + sm / 60.0f;
    } else if (sscanf(num, "%f", &v) != 1) {
        return false;
    }
    if (v < OS_RA_MIN_HOURS || v > OS_RA_MAX_HOURS) {
        return false;
    }
    *value = v;
    return true;
}

static bool parse_dec_substr(const char *s, size_t len, float *value)
{
    char num[32];
    int d = 0;
    int m = 0;
    float sm = 0.0f;
    float v = 0.0f;

    if (len == 0u || len >= sizeof(num)) {
        return false;
    }
    memcpy(num, s, len);
    num[len] = '\0';

    if (sscanf(num, "%d:%d:%f", &d, &m, &sm) == 3) {
        int sign = (d < 0) ? -1 : 1;
        v = (float)abs(d) + (float)m / 60.0f + sm / 3600.0f;
        v *= (float)sign;
    } else if (sscanf(num, "%d:%f", &d, &sm) == 2) {
        int sign = (d < 0) ? -1 : 1;
        v = (float)abs(d) + sm / 60.0f;
        v *= (float)sign;
    } else if (sscanf(num, "%f", &v) != 1) {
        return false;
    }
    if (v < OS_DEC_MIN_DEG || v > OS_DEC_MAX_DEG) {
        return false;
    }
    *value = v;
    return true;
}

static bool parse_az_substr(const char *s, size_t len, float *value)
{
    char num[32];
    float v = 0.0f;
    if (len == 0u || len >= sizeof(num)) {
        return false;
    }
    memcpy(num, s, len);
    num[len] = '\0';
    if (sscanf(num, "%f", &v) != 1) {
        return false;
    }
    if (v < 0.0f || v > 360.0f) {
        return false;
    }
    *value = v;
    return true;
}

static bool parse_alt_substr(const char *s, size_t len, float *value)
{
    return parse_dec_substr(s, len, value);
}

static bool parse_guide_substr(const char *s, size_t len,
                               os_direction_t *direction, uint32_t *duration_ms)
{
    char num[32];
    char d;
    unsigned long parsed;
    char *endp = NULL;

    if (len < 2u || (len - 1u) >= sizeof(num)) {
        return false;
    }
    d = s[0];
    switch (d) {
    case 'e':
    case 'E':
        *direction = OS_DIRECTION_EAST;
        break;
    case 'w':
    case 'W':
        *direction = OS_DIRECTION_WEST;
        break;
    case 'n':
    case 'N':
        *direction = OS_DIRECTION_NORTH;
        break;
    case 's':
    case 'S':
        *direction = OS_DIRECTION_SOUTH;
        break;
    default:
        return false;
    }

    memcpy(num, s + 1, len - 1u);
    num[len - 1u] = '\0';
    parsed = strtoul(num, &endp, 10);
    if (endp == num || parsed == 0u || parsed > UINT32_MAX) {
        return false;
    }
    *duration_ms = (uint32_t)parsed;
    return true;
}

os_error_t os_init(void)
{
    uint8_t i;

    g_system_state = OS_STATE_INITIALIZING;
    reset_runtime_flags();

    if (os_hal_nvm_init() != OS_ERR_NONE) {
        g_system_state = OS_STATE_FAULT;
        return OS_ERR_NVM_FAULT;
    }
    nvm_load_calibration();
    nvm_load_park_position();
    nvm_load_pec();

    for (i = 0u; i < OS_CHANNEL_COUNT; i++) {
        (void)os_hal_comm_init(i);
    }

    for (i = 0u; i < OS_AXIS_COUNT; i++) {
        if (os_hal_motor_init(i) != OS_ERR_NONE) {
            g_system_state = OS_STATE_FAULT;
            return OS_ERR_MOTOR_DRIVER_FAULT;
        }
    }

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();

    {
        os_site_info_t gps_polled;
        memset(&gps_polled, 0, sizeof(gps_polled));
        if (os_hal_gps_poll(&gps_polled) == OS_ERR_NONE && site_gps_ranges_valid(&gps_polled)) {
            g_gps_site = gps_polled;
            g_site = gps_polled;
            g_site.valid = true;
            g_gps_locked = true;
            g_time_source = OS_TIME_SOURCE_GPS;
            (void)os_hal_rtc_set(gps_polled.utc_epoch_seconds);
        } else {
            uint32_t rtc_epoch = 0u;
            if (os_hal_rtc_read(&rtc_epoch) != OS_ERR_NONE) {
                g_system_state = OS_STATE_FAULT;
                return OS_ERR_GPS_NO_SIGNAL;
            }
            g_site.latitude_degrees = 0.0f;
            g_site.longitude_degrees = 0.0f;
            g_site.elevation_metres = 0.0f;
            g_site.utc_epoch_seconds = rtc_epoch;
            g_site.valid = false;
            g_gps_locked = false;
            g_time_source = OS_TIME_SOURCE_RTC_PRESET;
        }
    }

    if (os_hal_timer_motor_init() != OS_ERR_NONE) {
        g_system_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    g_track_rate = OS_TRACK_RATE_SIDEREAL;
    g_track_custom_factor = 1.0f;
    g_tracking_enabled = true;
    g_mount_type = OS_MOUNT_EQUATORIAL;
    g_system_state = OS_STATE_IDLE_TRACKING;

    return OS_ERR_NONE;
}

void os_loop_iteration(void)
{
    if (g_system_state == OS_STATE_FAULT || g_system_state == OS_STATE_INITIALIZING) {
        return;
    }

    g_loop_tick_ms += OS_LOOP_TICK_MS;
    poll_time_site();
    poll_commands();
    if (motion_check_faults()) {
        return;
    }
    pec_advance_phase();
    motion_update();
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length)
{
    char reply[OS_MAX_REPLY_LENGTH];
    char frame[OS_MAX_COMMAND_LENGTH + 1u];
    size_t reply_len = 0u;
    os_error_t result = OS_ERR_NONE;

    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length > OS_MAX_COMMAND_LENGTH || source_channel >= OS_CHANNEL_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (length < 2u || command[0] != OS_LX200_CMD_PREFIX || command[length - 1u] != OS_LX200_CMD_SUFFIX) {
        snprintf(reply, sizeof(reply), "format error");
        copy_reply(reply, reply_buffer, reply_buffer_size, reply_length);
        if (*reply_length > 0u) {
            (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        }
        return OS_ERR_COMMAND_FORMAT;
    }

    memcpy(frame, command, length);
    frame[length] = '\0';
    memset(reply, 0, sizeof(reply));
    reply_len = 0u;

    if (strcmp(frame, ":GVP#") == 0) {
        snprintf(reply, sizeof(reply), "%u.%u.%u",
                 OS_FIRMWARE_VERSION_MAJOR, OS_FIRMWARE_VERSION_MINOR, OS_FIRMWARE_VERSION_PATCH);
        result = OS_ERR_NONE;
    } else if (strcmp(frame, ":GR#") == 0) {
        os_equatorial_coord_t coord;
        (void)os_query_coordinates(&coord);
        snprintf(reply, sizeof(reply), "%.3f", (double)coord.ra_hours);
        result = OS_ERR_NONE;
    } else if (strcmp(frame, ":GD#") == 0) {
        os_equatorial_coord_t coord;
        (void)os_query_coordinates(&coord);
        snprintf(reply, sizeof(reply), "%.3f", (double)coord.dec_degrees);
        result = OS_ERR_NONE;
    } else if (strcmp(frame, ":Me#") == 0) {
        result = os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
    } else if (strcmp(frame, ":Mw#") == 0) {
        result = os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
    } else if (strcmp(frame, ":Mn#") == 0) {
        result = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
    } else if (strcmp(frame, ":Ms#") == 0) {
        result = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
    } else if (strcmp(frame, ":Q#") == 0) {
        (void)os_goto_abort();
        (void)os_move_stop();
        result = OS_ERR_NONE;
    } else if (strcmp(frame, ":hP#") == 0) {
        result = os_park();
    } else if (strcmp(frame, ":hO#") == 0) {
        result = os_unpark();
    } else if (frame[0] == ':' && frame[1] == 'S' && frame[2] == 'r' && length > 4u) {
        size_t param_len = length - 4u;
        float ra;
        if (!parse_ra_substr(frame + 3, param_len, &ra)) {
            snprintf(reply, sizeof(reply), "format error");
            result = OS_ERR_COMMAND_FORMAT;
        } else {
            g_pending_ra_hours = ra;
            g_pending_ra_valid = true;
            result = OS_ERR_NONE;
        }
    } else if (frame[0] == ':' && frame[1] == 'S' && frame[2] == 'd' && length > 4u) {
        size_t param_len = length - 4u;
        float dec;
        if (!parse_dec_substr(frame + 3, param_len, &dec)) {
            snprintf(reply, sizeof(reply), "format error");
            result = OS_ERR_COMMAND_FORMAT;
        } else {
            g_pending_dec_degrees = dec;
            g_pending_dec_valid = true;
            result = OS_ERR_NONE;
        }
    } else if (frame[0] == ':' && frame[1] == 'S' && frame[2] == 'z' && length > 4u) {
        size_t param_len = length - 4u;
        float az;
        if (!parse_az_substr(frame + 3, param_len, &az)) {
            snprintf(reply, sizeof(reply), "format error");
            result = OS_ERR_COMMAND_FORMAT;
        } else {
            g_pending_az_degrees = az;
            g_pending_az_valid = true;
            result = OS_ERR_NONE;
        }
    } else if (frame[0] == ':' && frame[1] == 'S' && frame[2] == 'a' && length > 4u) {
        size_t param_len = length - 4u;
        float alt;
        if (!parse_alt_substr(frame + 3, param_len, &alt)) {
            snprintf(reply, sizeof(reply), "format error");
            result = OS_ERR_COMMAND_FORMAT;
        } else {
            g_pending_alt_degrees = alt;
            g_pending_alt_valid = true;
            result = OS_ERR_NONE;
        }
    } else if (strcmp(frame, ":MS#") == 0) {
        if (!g_pending_ra_valid || !g_pending_dec_valid) {
            result = OS_ERR_INVALID_STATE;
        } else {
            os_equatorial_coord_t target;
            target.ra_hours = g_pending_ra_hours;
            target.dec_degrees = g_pending_dec_degrees;
            result = os_goto_equatorial(target);
        }
    } else if (strcmp(frame, ":MA#") == 0) {
        if (!g_pending_az_valid || !g_pending_alt_valid) {
            result = OS_ERR_INVALID_STATE;
        } else {
            os_horizontal_coord_t target;
            target.azimuth_degrees = g_pending_az_degrees;
            target.altitude_degrees = g_pending_alt_degrees;
            result = os_goto_horizontal(target);
        }
    } else if (frame[0] == ':' && frame[1] == 'M' && frame[2] == 'g' && length > 4u) {
        size_t guide_len = length - 4u;
        os_direction_t direction;
        uint32_t duration_ms;
        if (!parse_guide_substr(frame + 3, guide_len, &direction, &duration_ms)) {
            snprintf(reply, sizeof(reply), "format error");
            result = OS_ERR_COMMAND_FORMAT;
        } else {
            result = os_guide_pulse(direction, duration_ms);
        }
    } else {
        snprintf(reply, sizeof(reply), "unknown command");
        result = OS_ERR_COMMAND_FORMAT;
    }

    if (reply[0] == '\0') {
        if (result == OS_ERR_NONE) {
            snprintf(reply, sizeof(reply), "ok");
        } else {
            snprintf(reply, sizeof(reply), "err %d", (int)result);
        }
    }

    reply_len = strlen(reply);
    copy_reply(reply, reply_buffer, reply_buffer_size, reply_length);
    if (*reply_length > 0u) {
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
    }
    return result;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target)
{
    int32_t target_steps[2];
    int32_t diff_ra;
    int32_t diff_dec;

    if (!isfinite((double)target.ra_hours) || !isfinite((double)target.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state != OS_STATE_IDLE_TRACKING && g_system_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_parking_active) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_mount_type = OS_MOUNT_EQUATORIAL;
    coord_to_steps(target, &target_steps[0], &target_steps[1]);
    diff_ra = target_steps[0] - g_motor_position_steps[0];
    diff_dec = target_steps[1] - g_motor_position_steps[1];

    g_tracking_equ_target = target;
    g_has_equatorial_target = true;
    g_has_horizontal_target = false;

    if (abs_i32(diff_ra) <= OS_GOTO_EPSILON_STEPS && abs_i32(diff_dec) <= OS_GOTO_EPSILON_STEPS) {
        motion_stop_slew();
        os_hal_buzzer_beep(50u, 1u);
        g_system_state = OS_STATE_IDLE_TRACKING;
        if (g_tracking_enabled) {
            motion_apply_tracking();
        }
        return OS_ERR_NONE;
    }

    g_goto_target[0] = target_steps[0];
    g_goto_target[1] = target_steps[1];
    g_goto_direction_forward[0] = diff_ra > 0;
    g_goto_direction_forward[1] = diff_dec > 0;
    goto_init_profiles(g_goto_target);
    g_goto_active = true;
    g_system_state = OS_STATE_GOTO;
    os_hal_motor_set_direction(0u, g_goto_direction_forward[0]);
    os_hal_motor_set_direction(1u, g_goto_direction_forward[1]);
    os_hal_motor_set_frequency(0u, (uint32_t)OS_GOTO_MIN_STEPS_PER_LOOP * 1000u);
    os_hal_motor_set_frequency(1u, (uint32_t)OS_GOTO_MIN_STEPS_PER_LOOP * 1000u);
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target)
{
    int32_t target_steps[2];
    int32_t diff0;
    int32_t diff1;

    if (!isfinite((double)target.azimuth_degrees) || !isfinite((double)target.altitude_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state != OS_STATE_IDLE_TRACKING && g_system_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_parking_active) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_mount_type = OS_MOUNT_ALTAZ;
    target_steps[0] = (int32_t)lround((double)target.azimuth_degrees * 3600.0 * OS_STEPS_PER_ARCSEC);
    target_steps[1] = (int32_t)lround((double)target.altitude_degrees * 3600.0 * OS_STEPS_PER_ARCSEC);
    diff0 = target_steps[0] - g_motor_position_steps[0];
    diff1 = target_steps[1] - g_motor_position_steps[1];

    g_tracking_hz_target = target;
    g_has_horizontal_target = true;
    g_has_equatorial_target = false;

    if (abs_i32(diff0) <= OS_GOTO_EPSILON_STEPS && abs_i32(diff1) <= OS_GOTO_EPSILON_STEPS) {
        motion_stop_slew();
        os_hal_buzzer_beep(50u, 1u);
        g_system_state = OS_STATE_IDLE_TRACKING;
        if (g_tracking_enabled) {
            motion_apply_tracking();
        }
        return OS_ERR_NONE;
    }

    g_goto_target[0] = target_steps[0];
    g_goto_target[1] = target_steps[1];
    g_goto_direction_forward[0] = diff0 > 0;
    g_goto_direction_forward[1] = diff1 > 0;
    goto_init_profiles(g_goto_target);
    g_goto_active = true;
    g_system_state = OS_STATE_GOTO;
    os_hal_motor_set_direction(0u, g_goto_direction_forward[0]);
    os_hal_motor_set_direction(1u, g_goto_direction_forward[1]);
    os_hal_motor_set_frequency(0u, (uint32_t)OS_GOTO_MIN_STEPS_PER_LOOP * 1000u);
    os_hal_motor_set_frequency(1u, (uint32_t)OS_GOTO_MIN_STEPS_PER_LOOP * 1000u);
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void)
{
    g_goto_active = false;
    motion_stop_slew();
    g_system_state = OS_STATE_IDLE_TRACKING;
    if (g_tracking_enabled) {
        motion_apply_tracking();
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor)
{
    if (!valid_track_rate(rate)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM) {
        if (!isfinite((double)custom_factor) || custom_factor <= 0.0f) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        g_track_custom_factor = custom_factor;
    } else {
        g_track_custom_factor = 1.0f;
    }
    g_track_rate = rate;
    if (g_tracking_enabled && g_system_state == OS_STATE_IDLE_TRACKING) {
        motion_apply_tracking();
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor)
{
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = g_track_rate;
    *custom_factor = g_track_custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void)
{
    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT ||
        g_system_state == OS_STATE_INITIALIZING) {
        return OS_ERR_INVALID_STATE;
    }
    g_tracking_enabled = true;
    if (g_system_state == OS_STATE_IDLE_TRACKING) {
        motion_apply_tracking();
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void)
{
    g_tracking_enabled = false;
    if (!g_goto_active && !g_parking_active && !g_manual_active && !g_guide_pulse.active) {
        motion_stop_slew();
    }
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms)
{
    if (!valid_direction(direction) || duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state == OS_STATE_FAULT || g_system_state == OS_STATE_PARKED ||
        g_system_state == OS_STATE_INITIALIZING) {
        return OS_ERR_INVALID_STATE;
    }

    if (g_guide_pulse.active) {
        bool new_is_dec = (direction == OS_DIRECTION_NORTH) || (direction == OS_DIRECTION_SOUTH);
        if (g_guide_pulse.dec_priority && !new_is_dec) {
            return OS_ERR_INVALID_STATE;
        }
    }

    g_guide_pulse.active = true;
    g_guide_pulse.duration_ms = duration_ms;
    g_guide_pulse.rate_fraction = g_guide_rate_fraction;
    g_guide_pulse.dec_priority = (direction == OS_DIRECTION_NORTH) || (direction == OS_DIRECTION_SOUTH);
    g_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    g_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    g_guide_remaining_ms = duration_ms;
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction)
{
    if (!isfinite((double)rate_fraction) || rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_guide_rate_fraction = rate_fraction;
    if (g_guide_pulse.active) {
        g_guide_pulse.rate_fraction = rate_fraction;
    }
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse)
{
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *pulse = g_guide_pulse;
    if (g_guide_pulse.active) {
        pulse->duration_ms = g_guide_remaining_ms;
    }
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode)
{
    if (!valid_align_mode(mode)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_system_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_parking_active || g_goto_active || g_manual_active) {
        return OS_ERR_INVALID_STATE;
    }
    g_align_active = true;
    g_align_mode = mode;
    g_align_star_count = 0u;
    g_align_residual_computed = false;
    g_align_residual_arcsec = 0.0;
    g_system_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos)
{
    if (!isfinite((double)star_coord.ra_hours) || !isfinite((double)star_coord.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (star_coord.ra_hours < OS_RA_MIN_HOURS || star_coord.ra_hours > OS_RA_MAX_HOURS) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (star_coord.dec_degrees < OS_DEC_MIN_DEG || star_coord.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_align_active || g_align_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    g_align_samples[g_align_star_count].ra_as = (double)star_coord.ra_hours * 15.0 * 3600.0;
    g_align_samples[g_align_star_count].dec_as = (double)star_coord.dec_degrees * 3600.0;
    g_align_samples[g_align_star_count].motor_ra = (double)motor_pos.ra_steps;
    g_align_samples[g_align_star_count].motor_dec = (double)motor_pos.dec_steps;
    g_align_star_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void)
{
    os_error_t err;
    if (!g_align_active) {
        return OS_ERR_INVALID_STATE;
    }
    err = align_compute_internal();
    if (err != OS_ERR_NONE) {
        g_align_residual_computed = false;
        return err;
    }
    g_align_active = false;
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec)
{
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_align_residual_computed) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = (float)g_align_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void)
{
    g_align_active = false;
    g_align_star_count = 0u;
    g_align_residual_computed = false;
    g_align_residual_arcsec = 0.0;
    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park(void)
{
    int32_t target_steps[2];

    if (g_system_state == OS_STATE_PARKED && !g_parking_active) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_system_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    if (g_park_custom_valid) {
        coord_to_steps(g_park_position, &target_steps[0], &target_steps[1]);
    } else {
        target_steps[0] = 0;
        target_steps[1] = (int32_t)lround(90.0 * 3600.0 * OS_STEPS_PER_ARCSEC);
    }

    g_park_target_steps[0] = target_steps[0];
    g_park_target_steps[1] = target_steps[1];
    g_park_direction_forward[0] = (target_steps[0] - g_motor_position_steps[0]) > 0;
    g_park_direction_forward[1] = (target_steps[1] - g_motor_position_steps[1]) > 0;
    g_parking_active = true;
    g_system_state = OS_STATE_IDLE_TRACKING;
    os_hal_motor_set_direction(0u, g_park_direction_forward[0]);
    os_hal_motor_set_direction(1u, g_park_direction_forward[1]);
    os_hal_motor_set_frequency(0u, OS_PARK_FREQ_HZ);
    os_hal_motor_set_frequency(1u, OS_PARK_FREQ_HZ);
    return OS_ERR_NONE;
}

os_error_t os_unpark(void)
{
    uint8_t i;
    if (g_system_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    for (i = 0u; i < OS_CHANNEL_COUNT; i++) {
        (void)os_hal_comm_init(i);
    }
    for (i = 0u; i < OS_AXIS_COUNT; i++) {
        (void)os_hal_motor_enable(i, true);
    }
    (void)os_hal_rtc_set(g_site.utc_epoch_seconds);
    g_tracking_enabled = true;
    g_system_state = OS_STATE_IDLE_TRACKING;
    motion_apply_tracking();
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos)
{
    if (!isfinite((double)park_pos.ra_hours) || !isfinite((double)park_pos.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (park_pos.ra_hours < OS_RA_MIN_HOURS || park_pos.ra_hours > OS_RA_MAX_HOURS) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (park_pos.dec_degrees < OS_DEC_MIN_DEG || park_pos.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_park_position = park_pos;
    g_park_custom_valid = true;
    (void)nvm_save_park_position();
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed)
{
    uint8_t axis;

    if (!valid_direction(direction) || !valid_speed(speed)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    axis = manual_axis_for_direction(direction);
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (g_system_state != OS_STATE_IDLE_TRACKING && g_system_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_parking_active || g_goto_active) {
        return OS_ERR_INVALID_STATE;
    }

    g_manual_active = true;
    g_manual_direction = direction;
    g_manual_speed = speed;
    g_manual_ticks_remaining = OS_MANUAL_TIMEOUT_LOOPS;
    g_manual_accum = 0.0;
    g_system_state = OS_STATE_MANUAL_MOTION;
    os_hal_motor_enable(axis, true);
    os_hal_motor_set_direction(axis, manual_forward_for_direction(direction));
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void)
{
    g_manual_active = false;
    g_manual_ticks_remaining = 0u;
    g_system_state = OS_STATE_IDLE_TRACKING;
    if (g_tracking_enabled) {
        motion_apply_tracking();
    } else {
        motion_stop_slew();
    }
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec)
{
    if (!isfinite((double)arcsec_per_sec) || arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_custom_manual_speed = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state)
{
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = g_system_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord)
{
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    steps_to_coord(g_motor_position_steps[0], g_motor_position_steps[1],
                   &coord->ra_hours, &coord->dec_degrees);
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = g_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos)
{
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = g_motor_position_steps[0];
    pos->dec_steps = g_motor_position_steps[1];
    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor, uint8_t *patch)
{
    if (major == NULL || minor == NULL || patch == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *major = OS_FIRMWARE_VERSION_MAJOR;
    *minor = OS_FIRMWARE_VERSION_MINOR;
    *patch = OS_FIRMWARE_VERSION_PATCH;
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *moving)
{
    bool any_motor_frequency = false;
    if (moving == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    any_motor_frequency = (g_motor_frequency_hz[0] != 0u) || (g_motor_frequency_hz[1] != 0u);
    *moving = g_goto_active || g_parking_active || g_manual_active || g_guide_pulse.active ||
              (g_tracking_enabled && any_motor_frequency);
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked)
{
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *locked = g_gps_locked;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable)
{
    g_pec_enabled = enable;
    (void)nvm_save_pec();
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(&g_pec_table, table, sizeof(g_pec_table));
    g_pec_table.valid = true;
    (void)nvm_save_pec();
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *table = g_pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec)
{
    int index;

    if (!isfinite((double)worm_phase_deg)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    index = (int)worm_phase_deg;
    if (index >= 360) {
        index = 0;
    }
    if (index < 0) {
        index = 0;
    }
    g_pec_table.corrections[index] = error_arcsec;
    g_pec_table.valid = true;
    (void)nvm_save_pec();
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib)
{
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *calib = g_calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void)
{
    calibration_set_identity_double();
    g_align_residual_computed = false;
    g_align_residual_arcsec = 0.0;
    (void)nvm_save_calibration();
    return OS_ERR_NONE;
}

os_error_t os_test_gps_set_site(const os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_gps_inject_site = *site;
    g_gps_inject_active = true;
    return OS_ERR_NONE;
}

os_error_t os_test_gps_set_no_signal(void)
{
    memset(&g_gps_inject_site, 0, sizeof(g_gps_inject_site));
    g_gps_inject_site.valid = false;
    g_gps_inject_active = true;
    return OS_ERR_NONE;
}

os_error_t os_test_limit_set(uint8_t axis, bool triggered)
{
    if (!valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_limit_inject_active[axis] = true;
    g_limit_inject_value[axis] = triggered;
    return OS_ERR_NONE;
}

os_error_t os_test_motor_set_fault(uint8_t axis, bool fault)
{
    if (!valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_motor_init_fault[axis] = fault;
    return OS_ERR_NONE;
}

os_error_t os_test_nvm_set_fault(bool read_fault, bool write_fault)
{
    g_nvm_read_fault = read_fault;
    g_nvm_write_fault = write_fault;
    return OS_ERR_NONE;
}

os_error_t os_test_comm_rx_push(uint8_t channel, const char *data, size_t length)
{
    if (!valid_channel(channel) || (data == NULL && length > 0u)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    while (length > 0u && g_comm_rx_count[channel] < OS_RX_BUFFER_SIZE) {
        uint16_t next = (uint16_t)((g_comm_rx_tail[channel] + 1u) % OS_RX_BUFFER_SIZE);
        g_comm_rx[channel][g_comm_rx_tail[channel]] = *data;
        g_comm_rx_tail[channel] = next;
        g_comm_rx_count[channel]++;
        data++;
        length--;
    }
    return OS_ERR_NONE;
}
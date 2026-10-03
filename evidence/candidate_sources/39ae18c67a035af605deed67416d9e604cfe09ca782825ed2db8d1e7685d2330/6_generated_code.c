#include "6_generated_code.h"

#include <stdio.h>
#include <string.h>

#define OS_AXIS_RA  0u
#define OS_AXIS_DEC 1u
#define OS_AXIS_COUNT 2u
#define OS_CHANNEL_COUNT 4u
#define OS_STEPS_PER_RA_HOUR 15000.0f
#define OS_STEPS_PER_DEC_DEG 1000.0f
#define OS_GOTO_FREQ_HZ 500u
#define OS_TRACK_FREQ_HZ 15u
#define OS_ALIGN_EPS 1.0e-9
#define OS_NVM_CALIB_OFFSET 0u
#define OS_NVM_PEC_OFFSET ((uint16_t)OS_NVM_CALIBRATION_SIZE_BYTES)

typedef struct {
    os_equatorial_coord_t coord;
    os_motor_position_t pos;
} align_star_t;

static os_state_t g_state = OS_STATE_INITIALIZING;
static os_site_info_t g_site;
static bool g_gps_locked;
static bool g_tracking_enabled;
static os_track_rate_t g_track_rate;
static float g_custom_track_factor;
static float g_guide_rate;
static os_guide_pulse_t g_guide;
static os_calibration_t g_calib;
static bool g_residual_valid;
static float g_residual_arcsec;
static os_pec_table_t g_pec;
static bool g_pec_enabled;
static os_align_mode_t g_align_mode;
static align_star_t g_align_stars[OS_CALIBRATION_MAX_STARS];
static uint8_t g_align_count;
static os_equatorial_coord_t g_current_coord;
static os_equatorial_coord_t g_target_coord;
static os_motor_position_t g_target_steps;
static bool g_goto_active;
static bool g_manual_active;
static os_direction_t g_manual_direction;
static float g_custom_move_speed;
static os_equatorial_coord_t g_park_pos;
static bool g_custom_park;
static char g_rx_buf[OS_CHANNEL_COUNT][OS_MAX_COMMAND_LENGTH];
static size_t g_rx_len[OS_CHANNEL_COUNT];

static bool valid_coord(os_equatorial_coord_t c)
{
    return c.ra_hours >= OS_RA_MIN_HOURS && c.ra_hours <= OS_RA_MAX_HOURS &&
           c.dec_degrees >= OS_DEC_MIN_DEG && c.dec_degrees <= OS_DEC_MAX_DEG;
}

static bool valid_direction(os_direction_t d)
{
    return d == OS_DIRECTION_NORTH || d == OS_DIRECTION_SOUTH ||
           d == OS_DIRECTION_EAST || d == OS_DIRECTION_WEST;
}

static bool valid_channel(uint8_t channel)
{
    return channel < OS_CHANNEL_COUNT;
}

static int32_t ra_to_steps(float ra_hours)
{
    return (int32_t)(ra_hours * OS_STEPS_PER_RA_HOUR);
}

static int32_t dec_to_steps(float dec_degrees)
{
    return (int32_t)((dec_degrees + 90.0f) * OS_STEPS_PER_DEC_DEG);
}

static float steps_to_ra(int32_t steps)
{
    float v = (float)steps / OS_STEPS_PER_RA_HOUR;
    while (v < 0.0f) {
        v += 24.0f;
    }
    while (v > 24.0f) {
        v -= 24.0f;
    }
    return v;
}

static float steps_to_dec(int32_t steps)
{
    float v = ((float)steps / OS_STEPS_PER_DEC_DEG) - 90.0f;
    if (v < -90.0f) {
        v = -90.0f;
    }
    if (v > 90.0f) {
        v = 90.0f;
    }
    return v;
}

static void stop_axis(uint8_t axis)
{
    (void)os_hal_motor_set_frequency(axis, 0u);
    (void)os_hal_motor_enable(axis, false);
}

static void stop_motion(void)
{
    stop_axis(OS_AXIS_RA);
    stop_axis(OS_AXIS_DEC);
    g_goto_active = false;
    g_manual_active = false;
}

static bool any_main_limit_triggered(void)
{
    return os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC);
}

static bool limit_for_direction(os_direction_t direction)
{
    if (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) {
        return os_hal_limit_is_triggered(OS_AXIS_RA);
    }
    if (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH) {
        return os_hal_limit_is_triggered(OS_AXIS_DEC);
    }
    return true;
}

static void apply_tracking(void)
{
    uint32_t freq = OS_TRACK_FREQ_HZ;

    if (!g_tracking_enabled || g_state != OS_STATE_IDLE_TRACKING) {
        return;
    }

    if (g_track_rate == OS_TRACK_RATE_LUNAR) {
        freq = (uint32_t)((float)freq * OS_LUNAR_RATE_FACTOR);
    } else if (g_track_rate == OS_TRACK_RATE_SOLAR) {
        freq = (uint32_t)((float)freq * OS_SOLAR_RATE_FACTOR);
    } else if (g_track_rate == OS_TRACK_RATE_CUSTOM) {
        freq = (uint32_t)((float)freq * g_custom_track_factor);
    }

    if (g_pec_enabled && g_pec.valid) {
        freq += 1u;
    }

    (void)os_hal_motor_set_direction(OS_AXIS_RA, true);
    (void)os_hal_motor_enable(OS_AXIS_RA, true);
    (void)os_hal_motor_set_frequency(OS_AXIS_RA, freq);
}

static os_error_t start_goto_steps(os_motor_position_t target)
{
    int32_t ra_pos = os_hal_motor_get_position(OS_AXIS_RA);
    int32_t dec_pos = os_hal_motor_get_position(OS_AXIS_DEC);
    int32_t dra = target.ra_steps - ra_pos;
    int32_t ddec = target.dec_steps - dec_pos;

    if (any_main_limit_triggered()) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_target_steps = target;

    if (dra > -2 && dra < 2 && ddec > -2 && ddec < 2) {
        stop_motion();
        g_state = OS_STATE_IDLE_TRACKING;
        g_tracking_enabled = true;
        return OS_ERR_NONE;
    }

    (void)os_hal_motor_set_direction(OS_AXIS_RA, dra >= 0);
    (void)os_hal_motor_set_direction(OS_AXIS_DEC, ddec >= 0);
    (void)os_hal_motor_enable(OS_AXIS_RA, true);
    (void)os_hal_motor_enable(OS_AXIS_DEC, true);
    (void)os_hal_motor_set_frequency(OS_AXIS_RA, dra == 0 ? 0u : OS_GOTO_FREQ_HZ);
    (void)os_hal_motor_set_frequency(OS_AXIS_DEC, ddec == 0 ? 0u : OS_GOTO_FREQ_HZ);

    g_goto_active = true;
    g_manual_active = false;
    g_tracking_enabled = false;
    g_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

static int min_stars_for_mode(os_align_mode_t mode)
{
    if (mode == OS_ALIGN_1STAR) {
        return 1;
    }
    if (mode == OS_ALIGN_2STAR) {
        return 2;
    }
    if (mode == OS_ALIGN_3STAR || mode == OS_ALIGN_NSTAR) {
        return 3;
    }
    return 0;
}

static double arcsec_ra(os_equatorial_coord_t c)
{
    return (double)c.ra_hours * 15.0 * 3600.0;
}

static double arcsec_dec(os_equatorial_coord_t c)
{
    return (double)c.dec_degrees * 3600.0;
}

static double dabs_local(double x)
{
    return x < 0.0 ? -x : x;
}

static bool solve3(double a[3][4], double out[3])
{
    for (int col = 0; col < 3; col++) {
        int pivot = col;
        double best = dabs_local(a[col][col]);

        for (int row = col + 1; row < 3; row++) {
            double v = dabs_local(a[row][col]);
            if (v > best) {
                best = v;
                pivot = row;
            }
        }

        if (best < OS_ALIGN_EPS) {
            return false;
        }

        if (pivot != col) {
            for (int k = col; k < 4; k++) {
                double t = a[col][k];
                a[col][k] = a[pivot][k];
                a[pivot][k] = t;
            }
        }

        for (int row = col + 1; row < 3; row++) {
            double f = a[row][col] / a[col][col];
            for (int k = col; k < 4; k++) {
                a[row][k] -= f * a[col][k];
            }
        }
    }

    for (int row = 2; row >= 0; row--) {
        double sum = a[row][3];
        for (int k = row + 1; k < 3; k++) {
            sum -= a[row][k] * out[k];
        }
        out[row] = sum / a[row][row];
    }

    return true;
}

static bool fit_affine(uint8_t n, bool dec_output, double out[3])
{
    double ata[3][4];
    memset(ata, 0, sizeof(ata));

    for (uint8_t i = 0; i < n; i++) {
        double x[3];
        double y;

        x[0] = arcsec_ra(g_align_stars[i].coord);
        x[1] = arcsec_dec(g_align_stars[i].coord);
        x[2] = 1.0;
        y = dec_output ? (double)g_align_stars[i].pos.dec_steps
                       : (double)g_align_stars[i].pos.ra_steps;

        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                ata[r][c] += x[r] * x[c];
            }
            ata[r][3] += x[r] * y;
        }
    }

    return solve3(ata, out);
}

static bool align_degenerate(void)
{
    double x1 = arcsec_ra(g_align_stars[0].coord);
    double y1 = arcsec_dec(g_align_stars[0].coord);
    double x2 = arcsec_ra(g_align_stars[1].coord);
    double y2 = arcsec_dec(g_align_stars[1].coord);
    double x3 = arcsec_ra(g_align_stars[2].coord);
    double y3 = arcsec_dec(g_align_stars[2].coord);
    double det = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

    return dabs_local(det) < 1.0e-3;
}

os_error_t os_init(void)
{
    os_error_t err;
    os_site_info_t gps_site;
    uint32_t rtc_time = 0u;
    uint8_t nvm_buf[OS_NVM_CALIBRATION_SIZE_BYTES];

    g_state = OS_STATE_INITIALIZING;
    memset(&g_site, 0, sizeof(g_site));
    memset(&g_guide, 0, sizeof(g_guide));
    memset(&g_calib, 0, sizeof(g_calib));
    memset(&g_pec, 0, sizeof(g_pec));
    memset(g_rx_buf, 0, sizeof(g_rx_buf));
    memset(g_rx_len, 0, sizeof(g_rx_len));
    g_gps_locked = false;
    g_tracking_enabled = true;
    g_track_rate = OS_TRACK_RATE_SIDEREAL;
    g_custom_track_factor = 1.0f;
    g_guide_rate = OS_GUIDE_RATE_MIN;
    g_residual_valid = false;
    g_residual_arcsec = 0.0f;
    g_pec_enabled = false;
    g_align_count = 0u;
    g_goto_active = false;
    g_manual_active = false;
    g_custom_move_speed = OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
    g_current_coord.ra_hours = 0.0f;
    g_current_coord.dec_degrees = 0.0f;
    g_target_coord = g_current_coord;
    g_park_pos.ra_hours = 0.0f;
    g_park_pos.dec_degrees = 90.0f;
    g_custom_park = false;

    err = os_hal_nvm_init();
    if (err != OS_ERR_NONE) {
        g_state = OS_STATE_FAULT;
        return OS_ERR_NVM_FAULT;
    }

    if (os_hal_nvm_read(OS_NVM_CALIB_OFFSET, nvm_buf, sizeof(nvm_buf)) == OS_ERR_NONE) {
        os_calibration_t tmp;
        memset(&tmp, 0, sizeof(tmp));
        memcpy(&tmp, nvm_buf, sizeof(tmp));
        if (tmp.valid) {
            g_calib = tmp;
        }
    }

    for (uint8_t ch = 0; ch < OS_CHANNEL_COUNT; ch++) {
        (void)os_hal_comm_init(ch);
    }

    for (uint8_t axis = 0; axis < OS_AXIS_COUNT; axis++) {
        err = os_hal_motor_init(axis);
        if (err != OS_ERR_NONE) {
            err = os_hal_motor_init(axis);
            if (err != OS_ERR_NONE) {
                g_state = OS_STATE_FAULT;
                return OS_ERR_MOTOR_DRIVER_FAULT;
            }
        }
        (void)os_hal_motor_set_frequency(axis, 0u);
        (void)os_hal_motor_enable(axis, false);
    }

    err = os_hal_gps_init();
    if (err != OS_ERR_NONE && err != OS_ERR_NOT_SUPPORTED) {
        g_state = OS_STATE_FAULT;
        return err;
    }

    err = os_hal_rtc_init();
    if (err != OS_ERR_NONE && err != OS_ERR_NOT_SUPPORTED) {
        g_state = OS_STATE_FAULT;
        return err;
    }

    err = os_hal_limit_init();
    if (err != OS_ERR_NONE) {
        g_state = OS_STATE_FAULT;
        return err;
    }

    err = os_hal_timer_motor_init();
    if (err != OS_ERR_NONE) {
        g_state = OS_STATE_FAULT;
        return err;
    }

    memset(&gps_site, 0, sizeof(gps_site));
    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid &&
        gps_site.latitude_degrees >= -90.0f && gps_site.latitude_degrees <= 90.0f &&
        gps_site.longitude_degrees >= -180.0f && gps_site.longitude_degrees <= 180.0f) {
        g_site = gps_site;
        g_gps_locked = true;
        (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
    } else {
        g_site.latitude_degrees = 0.0f;
        g_site.longitude_degrees = 0.0f;
        g_site.elevation_metres = 0.0f;
        if (os_hal_rtc_read(&rtc_time) == OS_ERR_NONE) {
            g_site.utc_epoch_seconds = rtc_time;
        }
        g_site.valid = true;
        g_gps_locked = false;
    }

    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void)
{
    os_site_info_t polled_site;
    uint32_t rtc_time;
    char reply[OS_MAX_REPLY_LENGTH];
    size_t reply_len;

    for (uint8_t ch = 0; ch < OS_CHANNEL_COUNT; ch++) {
        int16_t available = os_hal_comm_available(ch);

        while (available > 0) {
            char c = os_hal_comm_read(ch);

            if (g_rx_len[ch] < OS_MAX_COMMAND_LENGTH) {
                g_rx_buf[ch][g_rx_len[ch]++] = c;
            } else {
                g_rx_len[ch] = 0u;
            }

            if (c == OS_LX200_CMD_SUFFIX || c == '\n') {
                reply_len = 0u;
                (void)os_command_parse(g_rx_buf[ch], g_rx_len[ch], ch,
                                       reply, sizeof(reply), &reply_len);
                if (reply_len > 0u) {
                    (void)os_hal_comm_write(ch, reply, reply_len);
                }
                g_rx_len[ch] = 0u;
            }

            available--;
        }
    }

    memset(&polled_site, 0, sizeof(polled_site));
    if (os_hal_gps_poll(&polled_site) == OS_ERR_NONE && polled_site.valid) {
        g_site = polled_site;
        g_gps_locked = true;
        (void)os_hal_rtc_set(polled_site.utc_epoch_seconds);
    } else if (os_hal_rtc_read(&rtc_time) == OS_ERR_NONE) {
        g_site.utc_epoch_seconds = rtc_time;
        g_gps_locked = false;
    }

    g_current_coord.ra_hours = steps_to_ra(os_hal_motor_get_position(OS_AXIS_RA));
    g_current_coord.dec_degrees = steps_to_dec(os_hal_motor_get_position(OS_AXIS_DEC));

    if (any_main_limit_triggered() &&
        (g_state == OS_STATE_GOTO || g_state == OS_STATE_MANUAL_MOTION)) {
        stop_motion();
        g_state = OS_STATE_FAULT;
        (void)os_hal_buzzer_beep(100u, 2u);
        return;
    }

    if (g_state == OS_STATE_GOTO && g_goto_active) {
        int32_t ra_pos = os_hal_motor_get_position(OS_AXIS_RA);
        int32_t dec_pos = os_hal_motor_get_position(OS_AXIS_DEC);
        int32_t dra = g_target_steps.ra_steps - ra_pos;
        int32_t ddec = g_target_steps.dec_steps - dec_pos;

        if (dra > -2 && dra < 2) {
            stop_axis(OS_AXIS_RA);
        }
        if (ddec > -2 && ddec < 2) {
            stop_axis(OS_AXIS_DEC);
        }
        if (dra > -2 && dra < 2 && ddec > -2 && ddec < 2) {
            g_goto_active = false;
            g_state = OS_STATE_IDLE_TRACKING;
            g_tracking_enabled = true;
            (void)os_hal_buzzer_beep(80u, 1u);
        }
    }

    if (g_guide.active) {
        g_guide.active = false;
        g_guide.duration_ms = 0u;
    }

    apply_tracking();
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target)
{
    os_motor_position_t target_steps;

    if (!valid_coord(target)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (g_state == OS_STATE_PARKED || g_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    target_steps.ra_steps = ra_to_steps(target.ra_hours);
    target_steps.dec_steps = dec_to_steps(target.dec_degrees);
    g_target_coord = target;

    return start_goto_steps(target_steps);
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target)
{
    os_equatorial_coord_t eq;

    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    eq.ra_hours = target.azimuth_degrees / 15.0f;
    eq.dec_degrees = target.altitude_degrees;
    return os_goto_equatorial(eq);
}

os_error_t os_goto_abort(void)
{
    if (g_state != OS_STATE_GOTO && !g_goto_active) {
        return OS_ERR_INVALID_STATE;
    }

    stop_motion();
    g_tracking_enabled = true;
    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor)
{
    if (rate != OS_TRACK_RATE_SIDEREAL && rate != OS_TRACK_RATE_LUNAR &&
        rate != OS_TRACK_RATE_SOLAR && rate != OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_track_rate = rate;
    g_custom_track_factor = custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor)
{
    if (!rate || !custom_factor) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *rate = g_track_rate;
    *custom_factor = g_custom_track_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void)
{
    if (g_state == OS_STATE_PARKED || g_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    g_tracking_enabled = true;
    g_state = OS_STATE_IDLE_TRACKING;
    apply_tracking();
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void)
{
    g_tracking_enabled = false;
    stop_axis(OS_AXIS_RA);
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction)
{
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_guide_rate = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms)
{
    if (!valid_direction(direction) || duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_guide.active = true;
    g_guide.duration_ms = duration_ms;
    g_guide.rate_fraction = g_guide_rate;
    g_guide.direction_east = (direction == OS_DIRECTION_EAST);
    g_guide.direction_north = (direction == OS_DIRECTION_NORTH);
    g_guide.dec_priority = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse)
{
    if (!pulse) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *pulse = g_guide;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode)
{
    if (mode != OS_ALIGN_1STAR && mode != OS_ALIGN_2STAR &&
        mode != OS_ALIGN_3STAR && mode != OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_align_mode = mode;
    g_align_count = 0u;
    g_residual_valid = false;
    g_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos)
{
    if (!valid_coord(star_coord)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_align_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    g_align_stars[g_align_count].coord = star_coord;
    g_align_stars[g_align_count].pos = motor_pos;
    g_align_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void)
{
    int min_count = min_stars_for_mode(g_align_mode);
    double ra_fit[3] = {0.0, 0.0, 0.0};
    double dec_fit[3] = {0.0, 0.0, 0.0};
    uint8_t nvm_buf[OS_NVM_CALIBRATION_SIZE_BYTES];

    if (g_state != OS_STATE_ALIGNMENT || min_count == 0) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_align_count < (uint8_t)min_count) {
        return OS_ERR_INVALID_STATE;
    }

    if (g_align_mode == OS_ALIGN_1STAR) {
        g_calib.matrix_ra_to_ra = 1.0f;
        g_calib.matrix_ra_to_dec = 0.0f;
        g_calib.matrix_dec_to_ra = 0.0f;
        g_calib.matrix_dec_to_dec = 1.0f;
        g_calib.offset_ra_arcsec = (float)g_align_stars[0].pos.ra_steps;
        g_calib.offset_dec_arcsec = (float)g_align_stars[0].pos.dec_steps;
    } else if (g_align_mode == OS_ALIGN_2STAR) {
        double x1 = arcsec_ra(g_align_stars[0].coord);
        double x2 = arcsec_ra(g_align_stars[1].coord);
        double y1 = arcsec_dec(g_align_stars[0].coord);
        double y2 = arcsec_dec(g_align_stars[1].coord);

        if (dabs_local(x2 - x1) < OS_ALIGN_EPS || dabs_local(y2 - y1) < OS_ALIGN_EPS) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        g_calib.matrix_ra_to_ra = (float)(((double)g_align_stars[1].pos.ra_steps -
                                           (double)g_align_stars[0].pos.ra_steps) / (x2 - x1));
        g_calib.matrix_ra_to_dec = 0.0f;
        g_calib.matrix_dec_to_ra = 0.0f;
        g_calib.matrix_dec_to_dec = (float)(((double)g_align_stars[1].pos.dec_steps -
                                             (double)g_align_stars[0].pos.dec_steps) / (y2 - y1));
        g_calib.offset_ra_arcsec = (float)((double)g_align_stars[0].pos.ra_steps -
                                           (double)g_calib.matrix_ra_to_ra * x1);
        g_calib.offset_dec_arcsec = (float)((double)g_align_stars[0].pos.dec_steps -
                                            (double)g_calib.matrix_dec_to_dec * y1);
    } else {
        if (align_degenerate()) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        if (!fit_affine(g_align_count, false, ra_fit) ||
            !fit_affine(g_align_count, true, dec_fit)) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        g_calib.matrix_ra_to_ra = (float)ra_fit[0];
        g_calib.matrix_dec_to_ra = (float)ra_fit[1];
        g_calib.offset_ra_arcsec = (float)ra_fit[2];
        g_calib.matrix_ra_to_dec = (float)dec_fit[0];
        g_calib.matrix_dec_to_dec = (float)dec_fit[1];
        g_calib.offset_dec_arcsec = (float)dec_fit[2];
    }

    g_calib.valid = true;
    g_residual_arcsec = 0.0f;
    g_residual_valid = true;

    memset(nvm_buf, 0, sizeof(nvm_buf));
    memcpy(nvm_buf, &g_calib, sizeof(g_calib));
    (void)os_hal_nvm_write(OS_NVM_CALIB_OFFSET, nvm_buf, sizeof(nvm_buf));

    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec)
{
    if (!residual_arcsec) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_residual_valid) {
        return OS_ERR_INVALID_STATE;
    }

    *residual_arcsec = g_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void)
{
    g_align_count = 0u;
    g_residual_valid = false;
    if (g_state == OS_STATE_ALIGNMENT) {
        g_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_park(void)
{
    os_motor_position_t target;

    if (any_main_limit_triggered()) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    target.ra_steps = ra_to_steps(g_custom_park ? g_park_pos.ra_hours : 0.0f);
    target.dec_steps = dec_to_steps(g_custom_park ? g_park_pos.dec_degrees : 90.0f);

    if (start_goto_steps(target) != OS_ERR_NONE) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    stop_motion();
    g_tracking_enabled = false;
    g_state = OS_STATE_PARKED;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void)
{
    if (g_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    for (uint8_t ch = 0; ch < OS_CHANNEL_COUNT; ch++) {
        (void)os_hal_comm_init(ch);
    }

    (void)os_hal_motor_enable(OS_AXIS_RA, true);
    (void)os_hal_motor_enable(OS_AXIS_DEC, true);
    g_tracking_enabled = true;
    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos)
{
    if (!valid_coord(park_pos)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_park_pos = park_pos;
    g_custom_park = true;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed)
{
    uint8_t axis;
    uint32_t freq;

    if (!valid_direction(direction) ||
        (speed != OS_SPEED_SLOW && speed != OS_SPEED_MEDIUM &&
         speed != OS_SPEED_FAST && speed != OS_SPEED_CUSTOM)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (limit_for_direction(direction)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    if (g_state == OS_STATE_PARKED || g_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    axis = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) ? OS_AXIS_RA : OS_AXIS_DEC;
    if (speed == OS_SPEED_SLOW) {
        freq = 50u;
    } else if (speed == OS_SPEED_MEDIUM) {
        freq = 200u;
    } else if (speed == OS_SPEED_FAST) {
        freq = 800u;
    } else {
        freq = (uint32_t)(g_custom_move_speed > 1.0f ? g_custom_move_speed : 1.0f);
    }

    stop_motion();
    (void)os_hal_motor_set_direction(axis, direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_NORTH);
    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_frequency(axis, freq);

    g_manual_active = true;
    g_manual_direction = direction;
    g_tracking_enabled = false;
    g_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void)
{
    if (!g_manual_active && g_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }

    stop_motion();
    g_tracking_enabled = true;
    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec)
{
    if (arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_custom_move_speed = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state)
{
    if (!state) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *state = g_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord)
{
    if (!coord) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    coord->ra_hours = steps_to_ra(os_hal_motor_get_position(OS_AXIS_RA));
    coord->dec_degrees = steps_to_dec(os_hal_motor_get_position(OS_AXIS_DEC));
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site)
{
    if (!site) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *site = g_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos)
{
    if (!pos) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    pos->ra_steps = os_hal_motor_get_position(OS_AXIS_RA);
    pos->dec_steps = os_hal_motor_get_position(OS_AXIS_DEC);
    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor, uint8_t *patch)
{
    if (!major || !minor || !patch) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *major = OS_FIRMWARE_VERSION_MAJOR;
    *minor = OS_FIRMWARE_VERSION_MINOR;
    *patch = OS_FIRMWARE_VERSION_PATCH;
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *moving)
{
    if (!moving) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *moving = g_goto_active || g_manual_active ||
              g_state == OS_STATE_GOTO || g_state == OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked)
{
    if (!locked) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *locked = g_gps_locked;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable)
{
    g_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table)
{
    if (!table) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_pec = *table;
    g_pec.valid = table->valid;
    (void)os_hal_nvm_write(OS_NVM_PEC_OFFSET, (const uint8_t *)&g_pec, sizeof(g_pec));
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table)
{
    if (!table) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *table = g_pec;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec)
{
    int index;

    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    index = (worm_phase_deg >= 360.0f) ? 0 : (int)worm_phase_deg;
    if (index < 0 || index >= OS_PEC_TABLE_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_pec.corrections[index] = error_arcsec;
    g_pec.valid = true;
    (void)os_hal_nvm_write(OS_NVM_PEC_OFFSET, (const uint8_t *)&g_pec, sizeof(g_pec));
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib)
{
    if (!calib) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *calib = g_calib;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void)
{
    memset(&g_calib, 0, sizeof(g_calib));
    g_residual_valid = false;
    return OS_ERR_NONE;
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length)
{
    os_error_t err = OS_ERR_NONE;

    if (!command || !reply_buffer || !reply_length) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!valid_channel(source_channel) || reply_buffer_size == 0u ||
        length < 3u || length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *reply_length = 0u;
    reply_buffer[0] = '\0';

    if (command[0] != OS_LX200_CMD_PREFIX || command[length - 1u] != OS_LX200_CMD_SUFFIX) {
        err = OS_ERR_COMMAND_FORMAT;
    } else if (length >= 5u && strncmp(command, ":GVP#", 5u) == 0) {
        int n = snprintf(reply_buffer, reply_buffer_size, "%u.%u.%u#",
                         OS_FIRMWARE_VERSION_MAJOR,
                         OS_FIRMWARE_VERSION_MINOR,
                         OS_FIRMWARE_VERSION_PATCH);
        *reply_length = n > 0 ? (size_t)n : 0u;
    } else if (length >= 4u && strncmp(command, ":GR#", 4u) == 0) {
        os_equatorial_coord_t c;
        (void)os_query_coordinates(&c);
        {
            int n = snprintf(reply_buffer, reply_buffer_size, "%02.2f#",
                             (double)c.ra_hours);
            *reply_length = n > 0 ? (size_t)n : 0u;
        }
    } else if (length >= 4u && strncmp(command, ":GD#", 4u) == 0) {
        os_equatorial_coord_t c;
        (void)os_query_coordinates(&c);
        {
            int n = snprintf(reply_buffer, reply_buffer_size, "%+03.2f#",
                             (double)c.dec_degrees);
            *reply_length = n > 0 ? (size_t)n : 0u;
        }
    } else if (length >= 4u && strncmp(command, ":MS#", 4u) == 0) {
        err = os_goto_equatorial(g_target_coord);
        if (err == OS_ERR_NONE) {
            if (reply_buffer_size >= 3u) {
                reply_buffer[0] = '0';
                reply_buffer[1] = '#';
                reply_buffer[2] = '\0';
                *reply_length = 2u;
            }
        }
    } else if (length >= 4u && strncmp(command, ":Q#", 3u) == 0) {
        if (g_state == OS_STATE_GOTO) {
            err = os_goto_abort();
        } else if (g_state == OS_STATE_MANUAL_MOTION) {
            err = os_move_stop();
        }
        if (reply_buffer_size >= 2u) {
            reply_buffer[0] = '#';
            reply_buffer[1] = '\0';
            *reply_length = 1u;
        }
    } else if (length >= 4u && strncmp(command, ":Me#", 4u) == 0) {
        err = os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
    } else if (length >= 4u && strncmp(command, ":Mw#", 4u) == 0) {
        err = os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
    } else if (length >= 4u && strncmp(command, ":Mn#", 4u) == 0) {
        err = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
    } else if (length >= 4u && strncmp(command, ":Ms#", 4u) == 0) {
        err = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
    } else if (length >= 4u && strncmp(command, ":hP#", 4u) == 0) {
        err = os_park();
    } else if (length >= 4u && strncmp(command, ":hO#", 4u) == 0) {
        err = os_unpark();
    } else {
        err = OS_ERR_NOT_SUPPORTED;
    }

    if (err != OS_ERR_NONE && *reply_length == 0u) {
        int n = snprintf(reply_buffer, reply_buffer_size, "%d#", (int)err);
        *reply_length = n > 0 ? (size_t)n : 0u;
    }

    if (*reply_length >= reply_buffer_size) {
        *reply_length = reply_buffer_size - 1u;
        reply_buffer[*reply_length] = '\0';
    }

    return err;
}

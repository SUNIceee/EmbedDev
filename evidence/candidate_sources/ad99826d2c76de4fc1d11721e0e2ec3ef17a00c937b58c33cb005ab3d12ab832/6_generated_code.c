#include "6_generated_code.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* -------------------------------------------------------------------------
 * Internal state
 * ---------------------------------------------------------------------- */
static os_state_t s_state = OS_STATE_INITIALIZING;

static bool s_tracking_enabled = true;
static os_track_rate_t s_track_rate = OS_TRACK_RATE_SIDEREAL;
static float s_custom_track_factor = 1.0f;

static os_site_info_t s_site;
static uint32_t s_rtc_epoch = 0;

static os_equatorial_coord_t s_current_coord = {0.0f, 0.0f};
static os_equatorial_coord_t s_park_position = {0.0f, 90.0f};

static bool s_goto_active = false;
static uint32_t s_goto_steps_remaining = 0;

static bool s_move_active = false;
static os_direction_t s_move_direction = OS_DIRECTION_NORTH;
static os_speed_level_t s_move_speed = OS_SPEED_SLOW;
static float s_custom_move_speed_arcsec_per_sec = 100.0f;

static os_guide_pulse_t s_guide_pulse = {0};
static float s_guide_rate_fraction = 0.5f;

static bool s_align_active = false;
static os_align_mode_t s_align_mode = OS_ALIGN_1STAR;
static int s_align_star_count = 0;
static bool s_align_computed = false;
static float s_align_residual_arcsec = 0.0f;

static double s_align_ra_hours[OS_CALIBRATION_MAX_STARS];
static double s_align_dec_deg[OS_CALIBRATION_MAX_STARS];
static double s_align_motor_ra[OS_CALIBRATION_MAX_STARS];
static double s_align_motor_dec[OS_CALIBRATION_MAX_STARS];

static os_calibration_t s_calibration = {0};
static bool s_calibration_valid = false;

static bool s_pec_enabled = false;
static bool s_pec_table_valid = false;
static int16_t s_pec_corrections[OS_PEC_TABLE_SIZE] = {0};

static char s_rx_buffer[4][OS_MAX_COMMAND_LENGTH + 2];
static uint8_t s_rx_length[4] = {0};

/* -------------------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------------- */
static bool valid_ra(float ra)
{
    return ra >= OS_RA_MIN_HOURS && ra <= OS_RA_MAX_HOURS;
}

static bool valid_dec(float dec)
{
    return dec >= OS_DEC_MIN_DEG && dec <= OS_DEC_MAX_DEG;
}

static void stop_motors(void)
{
    (void)os_hal_motor_set_frequency(0, 0);
    (void)os_hal_motor_set_frequency(1, 0);
}

static void reset_runtime_flags(void)
{
    s_state = OS_STATE_INITIALIZING;
    s_tracking_enabled = true;
    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_custom_track_factor = 1.0f;

    s_goto_active = false;
    s_goto_steps_remaining = 0;

    s_move_active = false;
    s_move_direction = OS_DIRECTION_NORTH;
    s_move_speed = OS_SPEED_SLOW;
    s_custom_move_speed_arcsec_per_sec = 100.0f;

    s_guide_pulse.active = false;
    s_guide_pulse.duration_ms = 0;
    s_guide_pulse.rate_fraction = 0.0f;
    s_guide_pulse.direction_east = false;
    s_guide_pulse.direction_north = false;
    s_guide_pulse.dec_priority = false;
    s_guide_rate_fraction = 0.5f;

    s_align_active = false;
    s_align_mode = OS_ALIGN_1STAR;
    s_align_star_count = 0;
    s_align_computed = false;
    s_align_residual_arcsec = 0.0f;

    s_calibration_valid = false;
    (void)memset(&s_calibration, 0, sizeof(s_calibration));

    s_pec_enabled = false;
    s_pec_table_valid = false;
    (void)memset(s_pec_corrections, 0, sizeof(s_pec_corrections));

    s_current_coord.ra_hours = 0.0f;
    s_current_coord.dec_degrees = 0.0f;
}

static void solve_least_squares_3(const double A[][3], const double b[], int n,
                                  double x[3])
{
    double Q[OS_CALIBRATION_MAX_STARS][3];
    double R[3][3] = {{0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0}};
    double y[3] = {0.0, 0.0, 0.0};
    int i, j, k;

    for (i = 0; i < n; ++i) {
        for (j = 0; j < 3; ++j) {
            Q[i][j] = A[i][j];
        }
    }

    /* Modified Gram-Schmidt QR */
    for (j = 0; j < 3; ++j) {
        double norm = 0.0;
        for (i = 0; i < n; ++i) norm += Q[i][j] * Q[i][j];
        norm = sqrt(norm);
        if (norm < 1e-12) {
            R[j][j] = 0.0;
            continue;
        }
        R[j][j] = norm;
        for (i = 0; i < n; ++i) Q[i][j] /= norm;
        for (k = j + 1; k < 3; ++k) {
            double dot = 0.0;
            for (i = 0; i < n; ++i) dot += Q[i][j] * Q[i][k];
            R[j][k] = dot;
            for (i = 0; i < n; ++i) Q[i][k] -= Q[i][j] * dot;
        }
    }

    for (j = 0; j < 3; ++j) {
        y[j] = 0.0;
        for (i = 0; i < n; ++i) y[j] += Q[i][j] * b[i];
    }

    for (j = 2; j >= 0; --j) {
        if (fabs(R[j][j]) < 1e-12) {
            x[j] = 0.0;
            continue;
        }
        double sum = y[j];
        for (k = j + 1; k < 3; ++k) sum -= R[j][k] * x[k];
        x[j] = sum / R[j][j];
    }
}

/* -------------------------------------------------------------------------
 * Production API
 * ---------------------------------------------------------------------- */
os_error_t os_init(void)
{
    os_error_t err;

    s_state = OS_STATE_INITIALIZING;
    reset_runtime_flags();

    /* NVM must be initialised first */
    err = os_hal_nvm_init();
    if (err != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return OS_ERR_NVM_FAULT;
    }

    /* Communication channels (0..3) */
    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        err = os_hal_comm_init(ch);
        if (err == OS_ERR_NOT_SUPPORTED) {
            continue;
        }
        if (err != OS_ERR_NONE) {
            s_state = OS_STATE_FAULT;
            return OS_ERR_COMMAND_FORMAT;
        }
    }

    /* Motor drivers for axis 0 and axis 1 */
    err = os_hal_motor_init(0);
    if (err != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    err = os_hal_motor_init(1);
    if (err != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    /* Sensors and timer */
    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();

    err = os_hal_timer_motor_init();
    if (err != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    /* Choose GPS or fallback to RTC/preset site */
    s_site = (os_site_info_t){0};
    err = os_hal_gps_poll(&s_site);
    if (err == OS_ERR_NONE && s_site.valid) {
        if (s_site.utc_epoch_seconds != 0U) {
            (void)os_hal_rtc_set(s_site.utc_epoch_seconds);
        }
    } else {
        s_site.valid = false;
        err = os_hal_rtc_read(&s_rtc_epoch);
        if (err == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = s_rtc_epoch;
        }
    }

    /* Default tracking */
    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_custom_track_factor = 1.0f;
    s_tracking_enabled = true;
    s_state = OS_STATE_IDLE_TRACKING;

    stop_motors();
    return OS_ERR_NONE;
}

void os_loop_iteration(void)
{
    /* Poll communication channels */
    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        int16_t avail = os_hal_comm_available(ch);
        if (avail <= 0) {
            continue;
        }

        while (avail-- > 0) {
            char c = os_hal_comm_read(ch);

            if (c == OS_LX200_CMD_SUFFIX || c == '\n' || c == '\r') {
                if (s_rx_length[ch] > 0U) {
                    char reply[OS_MAX_REPLY_LENGTH];
                    size_t reply_len = 0U;

                    s_rx_buffer[ch][s_rx_length[ch]] = '\0';
                    os_error_t perr = os_command_parse(s_rx_buffer[ch],
                                                      s_rx_length[ch],
                                                      ch,
                                                      reply,
                                                      sizeof(reply),
                                                      &reply_len);
                    if (perr == OS_ERR_NONE && reply_len > 0U) {
                        (void)os_hal_comm_write(ch, reply, reply_len);
                    }
                    s_rx_length[ch] = 0;
                }
            } else if (c == OS_LX200_CMD_PREFIX) {
                s_rx_length[ch] = 0;
                s_rx_buffer[ch][s_rx_length[ch]++] = c;
            } else if (s_rx_length[ch] > 0U &&
                       s_rx_length[ch] < OS_MAX_COMMAND_LENGTH) {
                s_rx_buffer[ch][s_rx_length[ch]++] = c;
            } else {
                /* Non-LX200 bytes outside a frame are ignored */
            }
        }
    }

    /* Advance asynchronous Goto */
    if (s_goto_active) {
        if (s_goto_steps_remaining > 0U) {
            s_goto_steps_remaining--;
            (void)os_hal_motor_set_frequency(0, 1000U);
            (void)os_hal_motor_set_frequency(1, 1000U);
        }
        if (s_goto_steps_remaining == 0U) {
            s_goto_active = false;
            stop_motors();
            (void)os_hal_buzzer_beep(100, 1);
            s_state = OS_STATE_IDLE_TRACKING;
        }
    }

    /* Manual motion continues until stop command or limit */
    if (s_move_active) {
        uint8_t axis = (s_move_direction == OS_DIRECTION_NORTH ||
                       s_move_direction == OS_DIRECTION_SOUTH) ? 1U : 0U;
        if (os_hal_limit_is_triggered(axis)) {
            (void)os_hal_motor_set_frequency(axis, 0);
            s_move_active = false;
            s_state = OS_STATE_FAULT;
            (void)os_hal_buzzer_beep(200, 3);
        }
    }

    /* Global limit / fault detection */
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        stop_motors();
        s_goto_active = false;
        s_move_active = false;
        s_state = OS_STATE_FAULT;
        (void)os_hal_buzzer_beep(200, 2);
    }

    /* Simple tracking placeholder: if idle and tracking enabled,
     * no frequency change is enforced here. Host/timer handles real pulses.
     */
    if (s_tracking_enabled && s_state == OS_STATE_IDLE_TRACKING) {
        /* Intentionally left blank; actual tracking rate is handled by
         * HAL timer and configuration. */
    }
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length)
{
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (source_channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (reply_buffer_size == 0U) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length == 0U || length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_COMMAND_FORMAT;
    }

    *reply_length = 0U;
    reply_buffer[0] = '\0';

    const char *start = command;
    size_t len = length;

    if (start[0] != OS_LX200_CMD_PREFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }
    start++;
    len--;
    if (len == 0U || start[len - 1U] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }
    len--;

    char body[OS_MAX_COMMAND_LENGTH] = {0};
    if (len >= sizeof(body)) {
        return OS_ERR_COMMAND_FORMAT;
    }
    (void)memcpy(body, start, len);
    body[len] = '\0';

    os_error_t err = OS_ERR_NONE;

    if (strcmp(body, "GR") == 0) {
        int ra_h = (int)s_current_coord.ra_hours;
        float ra_mf = (s_current_coord.ra_hours - (float)ra_h) * 60.0f;
        int ra_m = (int)ra_mf;
        float ra_sf = (ra_mf - (float)ra_m) * 60.0f;
        int ra_s = (int)ra_sf;
        (void)snprintf(reply_buffer, reply_buffer_size,
                       "%02d:%02d:%02d#", ra_h, ra_m, ra_s);
    } else if (strcmp(body, "GD") == 0) {
        float dec = s_current_coord.dec_degrees;
        char sign = (dec < 0.0f) ? '-' : '+';
        dec = fabsf(dec);
        int d = (int)dec;
        int m = (int)((dec - (float)d) * 60.0f);
        int sec = (int)((((dec - (float)d) * 60.0f) - (float)m) * 60.0f);
        (void)snprintf(reply_buffer, reply_buffer_size,
                       "%c%02d:%02d:%02d#", sign, d, m, sec);
    } else if (strcmp(body, "GVP") == 0) {
        (void)snprintf(reply_buffer, reply_buffer_size,
                       "%d.%d.%d#",
                       OS_FIRMWARE_VERSION_MAJOR,
                       OS_FIRMWARE_VERSION_MINOR,
                       OS_FIRMWARE_VERSION_PATCH);
    } else if (strcmp(body, "hP") == 0) {
        err = os_park();
        if (err == OS_ERR_NONE) {
            (void)snprintf(reply_buffer, reply_buffer_size, "PARKING#");
        }
    } else if (strcmp(body, "hO") == 0) {
        err = os_unpark();
        if (err == OS_ERR_NONE) {
            (void)snprintf(reply_buffer, reply_buffer_size, "UNPARKED#");
        }
    } else if (strcmp(body, "Me") == 0) {
        err = os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
    } else if (strcmp(body, "Mw") == 0) {
        err = os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
    } else if (strcmp(body, "Mn") == 0) {
        err = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
    } else if (strcmp(body, "Ms") == 0) {
        err = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
    } else if (strcmp(body, "Q") == 0) {
        (void)snprintf(reply_buffer, reply_buffer_size,
                       "STATE %d#", (int)s_state);
    } else {
        err = OS_ERR_NOT_SUPPORTED;
    }

    if (err == OS_ERR_NONE && reply_buffer[0] != '\0') {
        *reply_length = strlen(reply_buffer);
    }

    return err;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target)
{
    if (!valid_ra(target.ra_hours) || !valid_dec(target.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_goto_active = true;
    s_goto_steps_remaining = 100U;
    s_state = OS_STATE_GOTO;
    s_current_coord = target;

    (void)os_hal_motor_set_direction(0, true);
    (void)os_hal_motor_set_direction(1, true);
    (void)os_hal_motor_set_frequency(0, 1000U);
    (void)os_hal_motor_set_frequency(1, 1000U);

    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target)
{
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < OS_DEC_MIN_DEG ||
        target.altitude_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    os_equatorial_coord_t eq = {
        .ra_hours = target.azimuth_degrees / 15.0f,
        .dec_degrees = target.altitude_degrees
    };
    return os_goto_equatorial(eq);
}

os_error_t os_goto_abort(void)
{
    if (s_goto_active) {
        s_goto_active = false;
        s_goto_steps_remaining = 0U;
        stop_motors();
        s_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor)
{
    if (rate < OS_TRACK_RATE_SIDEREAL || rate > OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_track_rate = rate;
    s_custom_track_factor = (rate == OS_TRACK_RATE_CUSTOM) ? custom_factor : 1.0f;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor)
{
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = s_track_rate;
    *custom_factor = s_custom_track_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void)
{
    s_tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void)
{
    s_tracking_enabled = false;
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms)
{
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0U) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_guide_pulse.active = true;
    s_guide_pulse.duration_ms = duration_ms;
    s_guide_pulse.rate_fraction = s_guide_rate_fraction;
    s_guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    s_guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    s_guide_pulse.dec_priority = (direction == OS_DIRECTION_NORTH ||
                                  direction == OS_DIRECTION_SOUTH);
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction)
{
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_guide_rate_fraction = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse)
{
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *pulse = s_guide_pulse;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode)
{
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_align_active = true;
    s_align_mode = mode;
    s_align_star_count = 0;
    s_align_computed = false;
    s_align_residual_arcsec = 0.0f;
    s_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos)
{
    if (!valid_ra(star_coord.ra_hours) || !valid_dec(star_coord.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_align_active) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_align_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    s_align_ra_hours[s_align_star_count] = (double)star_coord.ra_hours;
    s_align_dec_deg[s_align_star_count] = (double)star_coord.dec_degrees;
    s_align_motor_ra[s_align_star_count] = (double)motor_pos.ra_steps;
    s_align_motor_dec[s_align_star_count] = (double)motor_pos.dec_steps;
    s_align_star_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void)
{
    if (!s_align_active) {
        return OS_ERR_INVALID_STATE;
    }

    int required = 0;
    switch (s_align_mode) {
        case OS_ALIGN_1STAR:
            required = 1;
            break;
        case OS_ALIGN_2STAR:
            required = 2;
            break;
        case OS_ALIGN_3STAR:
        case OS_ALIGN_NSTAR:
            required = 3;
            break;
        default:
            return OS_ERR_INVALID_STATE;
    }

    if (s_align_star_count < required) {
        return OS_ERR_INVALID_STATE;
    }

    double A[OS_CALIBRATION_MAX_STARS][3];
    double b_ra[OS_CALIBRATION_MAX_STARS];
    double b_dec[OS_CALIBRATION_MAX_STARS];

    for (int i = 0; i < s_align_star_count; ++i) {
        A[i][0] = s_align_motor_ra[i];
        A[i][1] = s_align_motor_dec[i];
        A[i][2] = 1.0;
        b_ra[i] = s_align_ra_hours[i] * 15.0;    /* RA hours -> degrees */
        b_dec[i] = s_align_dec_deg[i];            /* Dec degrees */
    }

    double x_ra[3] = {0.0, 0.0, 0.0};
    double x_dec[3] = {0.0, 0.0, 0.0};

    solve_least_squares_3((const double (*)[3])A, b_ra, s_align_star_count, x_ra);
    solve_least_squares_3((const double (*)[3])A, b_dec, s_align_star_count, x_dec);

    double det = x_ra[0] * x_dec[1] - x_ra[1] * x_dec[0];

    /* 3-star mode gives exact fit; check matrix degeneracy instead of residual */
    if (s_align_mode == OS_ALIGN_3STAR && s_align_star_count == 3) {
        if (fabs(det) < 1e-9) {
            return OS_ERR_CALIBRATION_FAILED;
        }
    }

    s_calibration.matrix_ra_to_ra = (float)x_ra[0];
    s_calibration.matrix_ra_to_dec = (float)x_ra[1];
    s_calibration.matrix_dec_to_ra = (float)x_dec[0];
    s_calibration.matrix_dec_to_dec = (float)x_dec[1];
    /* Offsets are stored in arcseconds */
    s_calibration.offset_ra_arcsec = (float)(x_ra[2] * 3600.0);
    s_calibration.offset_dec_arcsec = (float)(x_dec[2] * 3600.0);
    s_calibration.valid = true;

    /* Residual (only meaningful for 4+ calibration stars) */
    double residual_sum = 0.0;
    for (int i = 0; i < s_align_star_count; ++i) {
        double pred_ra = x_ra[0] * A[i][0] + x_ra[1] * A[i][1] + x_ra[2];
        double pred_dec = x_dec[0] * A[i][0] + x_dec[1] * A[i][1] + x_dec[2];
        double err_ra_arcsec = (pred_ra - b_ra[i]) * 3600.0;
        double err_dec_arcsec = (pred_dec - b_dec[i]) * 3600.0;
        residual_sum += sqrt(err_ra_arcsec * err_ra_arcsec +
                             err_dec_arcsec * err_dec_arcsec);
    }
    s_align_residual_arcsec = (float)(residual_sum / (double)s_align_star_count);
    s_align_computed = true;

    /* Persist calibration to NVM */
    os_error_t nvm_err = os_hal_nvm_write(0,
                                          (const uint8_t *)&s_calibration,
                                          sizeof(s_calibration));
    if (nvm_err != OS_ERR_NONE) {
        s_calibration.valid = false;
        return OS_ERR_NVM_FAULT;
    }

    s_calibration_valid = true;
    s_align_active = false;
    s_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec)
{
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_align_computed) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = s_align_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void)
{
    if (s_align_active) {
        s_align_active = false;
        s_align_star_count = 0;
        s_align_computed = false;
        s_align_residual_arcsec = 0.0f;
        s_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_park(void)
{
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    stop_motors();
    s_tracking_enabled = false;
    s_state = OS_STATE_PARKED;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void)
{
    s_tracking_enabled = true;
    s_state = OS_STATE_IDLE_TRACKING;

    (void)os_hal_motor_enable(0, true);
    (void)os_hal_motor_enable(1, true);
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos)
{
    if (!valid_ra(park_pos.ra_hours) || !valid_dec(park_pos.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_park_position = park_pos;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed)
{
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    uint8_t axis = (direction == OS_DIRECTION_NORTH ||
                    direction == OS_DIRECTION_SOUTH) ? 1U : 0U;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    bool forward = false;
    if (axis == 0U) {
        forward = (direction == OS_DIRECTION_EAST);
    } else {
        forward = (direction == OS_DIRECTION_NORTH);
    }

    uint32_t freq = 100U;
    switch (speed) {
        case OS_SPEED_SLOW:
            freq = 10U;
            break;
        case OS_SPEED_MEDIUM:
            freq = 100U;
            break;
        case OS_SPEED_FAST:
            freq = 500U;
            break;
        case OS_SPEED_CUSTOM:
            freq = (uint32_t)(s_custom_move_speed_arcsec_per_sec * 10.0f);
            break;
        default:
            freq = 100U;
            break;
    }

    s_move_active = true;
    s_move_direction = direction;
    s_move_speed = speed;
    s_state = OS_STATE_MANUAL_MOTION;

    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, freq);
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void)
{
    if (s_move_active) {
        s_move_active = false;
        stop_motors();
        s_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec)
{
    if (arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_custom_move_speed_arcsec_per_sec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state)
{
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = s_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord)
{
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *coord = s_current_coord;
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = s_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos)
{
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = os_hal_motor_get_position(0);
    pos->dec_steps = os_hal_motor_get_position(1);
    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor,
                                     uint8_t *patch)
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
    if (moving == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *moving = s_goto_active || s_move_active;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked)
{
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *locked = s_site.valid;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable)
{
    s_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    (void)memcpy(s_pec_corrections, table->corrections,
                 sizeof(s_pec_corrections));
    s_pec_table_valid = table->valid;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    (void)memcpy(table->corrections, s_pec_corrections,
                 sizeof(s_pec_corrections));
    table->valid = s_pec_table_valid;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec)
{
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    int index = (int)worm_phase_deg;
    if (index == 360) {
        index = 359;
    }
    s_pec_corrections[index] = error_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib)
{
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_calibration_valid) {
        return OS_ERR_INVALID_STATE;
    }
    *calib = s_calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void)
{
    s_calibration_valid = false;
    s_calibration.valid = false;
    (void)memset(&s_calibration, 0, sizeof(s_calibration));
    return OS_ERR_NONE;
}

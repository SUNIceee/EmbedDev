/* OnStep frozen API implementation (C11 host/server library). */
#include "6_generated_code.h"

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define OS_AXIS_RA              0u
#define OS_AXIS_DEC             1u
#define OS_NUM_AXES             2u
#define OS_NUM_CHANNELS         4u
#define OS_STEPS_PER_DEGREE     1000.0f
#define OS_STEPS_PER_HOUR       (OS_STEPS_PER_DEGREE * 15.0f)
#define OS_STEPS_PER_ARCSEC     ((double)OS_STEPS_PER_DEGREE / 3600.0)
#define OS_LOOP_DT_SEC          0.25f
#define OS_FAST_SPEED_DEG_PER_SEC        3.0f
#define OS_MEDIUM_SPEED_DEG_PER_SEC      0.5f
#define OS_SLOW_SPEED_DEG_PER_SEC        0.1f
#define OS_NVM_TOTAL_SIZE                 (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)
#define OS_ALIGN_MAX_RESIDUAL_ARCSEC      300.0

#define OS_HAL_RX_FIFO_SIZE               512u
#define OS_HAL_TX_FIFO_SIZE               2048u

typedef enum {
    OS_ACTION_NONE = 0,
    OS_ACTION_GOTO,
    OS_ACTION_PARK
} os_internal_action_t;

typedef struct {
    char data[OS_HAL_RX_FIFO_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} os_comm_rx_fifo_t;

typedef struct {
    char data[OS_HAL_TX_FIFO_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} os_comm_tx_fifo_t;

/* Host-test aids; not part of the frozen public API. */
int16_t os_hal_comm_inject_rx(uint8_t channel, const char *data, size_t length);
int16_t os_hal_comm_tx_available(uint8_t channel);
char os_hal_comm_tx_read(uint8_t channel);
void os_hal_comm_tx_clear(uint8_t channel);

static bool g_initialized = false;
static os_state_t g_state = OS_STATE_INITIALIZING;

static os_calibration_t g_calibration = { .valid = false };
static bool g_residual_valid = false;
static float g_residual_arcsec = 0.0f;

static os_align_mode_t g_align_mode = OS_ALIGN_1STAR;
static uint8_t g_align_star_count = 0;
static os_equatorial_coord_t g_align_stars[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t g_align_motors[OS_CALIBRATION_MAX_STARS];

static bool g_tracking_enabled = true;
static os_track_rate_t g_track_rate = OS_TRACK_RATE_SIDEREAL;
static float g_track_custom_factor = 1.0f;

static bool g_pending_ra_set = false;
static bool g_pending_dec_set = false;
static float g_pending_ra = 0.0f;
static float g_pending_dec = 0.0f;

static bool g_goto_active = false;
static bool g_park_pending = false;
static os_internal_action_t g_current_action = OS_ACTION_NONE;
static int32_t g_goto_target[OS_NUM_AXES] = {0, 0};
static int32_t g_park_ra_steps = 0;
static int32_t g_park_dec_steps = (int32_t)(90.0f * OS_STEPS_PER_DEGREE);
static bool g_park_position_set = false;

static bool g_manual_active = false;
static uint8_t g_manual_axis = OS_AXIS_RA;
static bool g_manual_forward = true;
static float g_custom_speed_arcsec_per_sec = 15.0f;

static os_guide_pulse_t g_guide_pulse = { .active = false };
static uint32_t g_guide_remaining_ms = 0;
static uint8_t g_guide_axis = OS_AXIS_RA;
static bool g_guide_forward = true;
static float g_guide_rate_fraction = 0.5f;

static bool g_pec_enabled = false;
static os_pec_table_t g_pec_table = { .valid = false };

static os_site_info_t g_site = {45.0f, -75.0f, 100.0f, 0, true};
static bool g_gps_locked = false;

static char g_comm_rx[OS_NUM_CHANNELS][OS_MAX_COMMAND_LENGTH];
static size_t g_comm_rx_len[OS_NUM_CHANNELS] = {0, 0, 0, 0};

static os_comm_rx_fifo_t g_comm_rx_fifo[OS_NUM_CHANNELS];
static os_comm_tx_fifo_t g_comm_tx_fifo[OS_NUM_CHANNELS];

static bool g_motor_initialized[OS_NUM_AXES] = {false, false};
static bool g_motor_enabled[OS_NUM_AXES] = {false, false};
static bool g_motor_direction[OS_NUM_AXES] = {true, true};
static uint32_t g_motor_frequency[OS_NUM_AXES] = {0, 0};
static int32_t g_motor_position[OS_NUM_AXES] = {0, 0};
static double g_motor_fraction[OS_NUM_AXES] = {0.0, 0.0};
static bool g_limit_triggered[OS_NUM_AXES] = {false, false};
static bool g_rtc_valid = false;
static uint32_t g_rtc_epoch = 0;
static bool g_nvm_initialized = false;
static uint8_t g_nvm[OS_NVM_TOTAL_SIZE] = {0};

static bool eq_valid(float ra_hours, float dec_degrees)
{
    return (ra_hours >= OS_RA_MIN_HOURS && ra_hours <= OS_RA_MAX_HOURS &&
            dec_degrees >= OS_DEC_MIN_DEG && dec_degrees <= OS_DEC_MAX_DEG);
}

static bool horizontal_valid(float az_degrees, float alt_degrees)
{
    return (az_degrees >= 0.0f && az_degrees <= 360.0f &&
            alt_degrees >= OS_DEC_MIN_DEG && alt_degrees <= OS_DEC_MAX_DEG);
}

static bool valid_axis(uint8_t axis)
{
    return axis < OS_NUM_AXES;
}

static bool valid_channel(uint8_t channel)
{
    return channel < OS_NUM_CHANNELS;
}

static void set_reply_fmt(char *buf, size_t cap, size_t *len, const char *fmt, ...)
{
    if (buf == NULL || len == NULL || cap == 0) {
        if (len != NULL) {
            *len = 0;
        }
        return;
    }

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, cap, fmt, ap);
    va_end(ap);

    if (n < 0) {
        buf[0] = '\0';
        *len = 0;
        return;
    }

    size_t written = (size_t)n;
    if (written >= cap) {
        written = cap - 1;
    }
    *len = written;
}

static float speed_to_deg_per_sec(os_speed_level_t speed)
{
    switch (speed) {
        case OS_SPEED_SLOW:
            return OS_SLOW_SPEED_DEG_PER_SEC;
        case OS_SPEED_MEDIUM:
            return OS_MEDIUM_SPEED_DEG_PER_SEC;
        case OS_SPEED_FAST:
            return OS_FAST_SPEED_DEG_PER_SEC;
        case OS_SPEED_CUSTOM:
            return g_custom_speed_arcsec_per_sec / 3600.0f;
        default:
            return 0.0f;
    }
}

static bool direction_to_axis_forward(os_direction_t direction,
                                      uint8_t *axis,
                                      bool *forward)
{
    switch (direction) {
        case OS_DIRECTION_NORTH:
            *axis = OS_AXIS_DEC;
            *forward = true;
            return true;
        case OS_DIRECTION_SOUTH:
            *axis = OS_AXIS_DEC;
            *forward = false;
            return true;
        case OS_DIRECTION_EAST:
            *axis = OS_AXIS_RA;
            *forward = true;
            return true;
        case OS_DIRECTION_WEST:
            *axis = OS_AXIS_RA;
            *forward = false;
            return true;
        default:
            return false;
    }
}

static float tracking_frequency_hz(void)
{
    float rate_arcsec = OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
    switch (g_track_rate) {
        case OS_TRACK_RATE_LUNAR:
            rate_arcsec *= OS_LUNAR_RATE_FACTOR;
            break;
        case OS_TRACK_RATE_SOLAR:
            rate_arcsec *= OS_SOLAR_RATE_FACTOR;
            break;
        case OS_TRACK_RATE_CUSTOM:
            rate_arcsec *= g_track_custom_factor;
            break;
        case OS_TRACK_RATE_SIDEREAL:
        default:
            break;
    }
    return (rate_arcsec / 3600.0f) * OS_STEPS_PER_DEGREE;
}

static void target_to_steps(const os_equatorial_coord_t *target,
                            int32_t *ra_steps_out,
                            int32_t *dec_steps_out)
{
    if (target == NULL || ra_steps_out == NULL || dec_steps_out == NULL) {
        return;
    }

    if (g_calibration.valid) {
        double ra_arcsec  = (double)target->ra_hours * 54000.0;
        double dec_arcsec = (double)target->dec_degrees * 3600.0;

        double ra_steps =
            (double)g_calibration.matrix_ra_to_ra * ra_arcsec +
            (double)g_calibration.matrix_ra_to_dec * dec_arcsec +
            (double)g_calibration.offset_ra_arcsec * OS_STEPS_PER_ARCSEC;

        double dec_steps =
            (double)g_calibration.matrix_dec_to_ra * ra_arcsec +
            (double)g_calibration.matrix_dec_to_dec * dec_arcsec +
            (double)g_calibration.offset_dec_arcsec * OS_STEPS_PER_ARCSEC;

        *ra_steps_out  = (int32_t)lround(ra_steps);
        *dec_steps_out = (int32_t)lround(dec_steps);
    } else {
        *ra_steps_out  = (int32_t)(target->ra_hours * OS_STEPS_PER_HOUR);
        *dec_steps_out = (int32_t)(target->dec_degrees * OS_STEPS_PER_DEGREE);
    }
}

static void steps_to_coord(int32_t ra_steps,
                           int32_t dec_steps,
                           os_equatorial_coord_t *coord)
{
    if (coord == NULL) {
        return;
    }

    if (g_calibration.valid) {
        double a = (double)g_calibration.matrix_ra_to_ra;
        double b = (double)g_calibration.matrix_ra_to_dec;
        double d = (double)g_calibration.matrix_dec_to_ra;
        double e = (double)g_calibration.matrix_dec_to_dec;

        double det = a * e - b * d;
        if (fabs(det) >= 1e-12) {
            double ra_offset_steps =
                (double)g_calibration.offset_ra_arcsec * OS_STEPS_PER_ARCSEC;
            double dec_offset_steps =
                (double)g_calibration.offset_dec_arcsec * OS_STEPS_PER_ARCSEC;

            double ra_diff  = (double)ra_steps - ra_offset_steps;
            double dec_diff = (double)dec_steps - dec_offset_steps;

            double ra_arcsec  = ( e * ra_diff - b * dec_diff) / det;
            double dec_arcsec = (-d * ra_diff + a * dec_diff) / det;

            coord->ra_hours    = (float)(ra_arcsec / 54000.0);
            coord->dec_degrees = (float)(dec_arcsec / 3600.0);
            return;
        }
    }

    coord->ra_hours    = (float)ra_steps / OS_STEPS_PER_HOUR;
    coord->dec_degrees = (float)dec_steps / OS_STEPS_PER_DEGREE;
}

static void update_motor_axis(uint8_t axis, float dt)
{
    if (!valid_axis(axis) || g_motor_frequency[axis] == 0) {
        return;
    }

    double step_f = ((double)g_motor_frequency[axis] * (double)dt) + g_motor_fraction[axis];
    int32_t step = (int32_t)step_f;
    g_motor_fraction[axis] = step_f - (double)step;

    if (step <= 0) {
        return;
    }

    if (g_motor_direction[axis]) {
        g_motor_position[axis] += step;
    } else {
        g_motor_position[axis] -= step;
    }
}

static void stop_axis(uint8_t axis)
{
    if (!valid_axis(axis)) {
        return;
    }
    g_motor_frequency[axis] = 0;
    g_motor_fraction[axis] = 0.0;
    (void)os_hal_motor_set_frequency(axis, 0);
}

static void stop_all_axes(void)
{
    stop_axis(OS_AXIS_RA);
    stop_axis(OS_AXIS_DEC);
}

static void enable_all_axes(bool enable)
{
    for (uint8_t axis = 0; axis < OS_NUM_AXES; axis++) {
        (void)os_hal_motor_enable(axis, enable);
    }
}

static void complete_motion(void)
{
    stop_all_axes();

    if (g_current_action == OS_ACTION_PARK) {
        g_state = OS_STATE_PARKED;
        g_tracking_enabled = false;
        enable_all_axes(false);
        (void)os_hal_buzzer_beep(100, 1);
    } else {
        g_state = OS_STATE_IDLE_TRACKING;
        g_tracking_enabled = true;
        (void)os_hal_buzzer_beep(50, 2);
    }

    g_goto_active = false;
    g_park_pending = false;
    g_current_action = OS_ACTION_NONE;
}

static void update_goto_park(float dt)
{
    if (!g_goto_active) {
        return;
    }

    bool limits_ok = true;
    for (uint8_t axis = 0; axis < OS_NUM_AXES; axis++) {
        if (os_hal_limit_is_triggered(axis)) {
            limits_ok = false;
            break;
        }
    }

    if (!limits_ok) {
        stop_all_axes();
        g_goto_active = false;
        g_park_pending = false;
        g_current_action = OS_ACTION_NONE;
        g_state = OS_STATE_FAULT;
        return;
    }

    for (uint8_t axis = 0; axis < OS_NUM_AXES; axis++) {
        int32_t diff = g_goto_target[axis] - g_motor_position[axis];

        if (diff == 0) {
            (void)os_hal_motor_set_frequency(axis, 0);
            g_motor_frequency[axis] = 0;
            continue;
        }

        bool forward = (diff > 0);
        float abs_diff = (float)fabs((double)diff);
        float max_freq = OS_FAST_SPEED_DEG_PER_SEC * OS_STEPS_PER_DEGREE;
        float max_step = max_freq * dt;
        float freq = (abs_diff <= max_step) ? (abs_diff / dt) : max_freq;

        (void)os_hal_motor_set_direction(axis, forward);
        (void)os_hal_motor_set_frequency(axis, (uint32_t)freq);
        update_motor_axis(axis, dt);

        if (forward && g_motor_position[axis] > g_goto_target[axis]) {
            g_motor_position[axis] = g_goto_target[axis];
        } else if (!forward && g_motor_position[axis] < g_goto_target[axis]) {
            g_motor_position[axis] = g_goto_target[axis];
        }
    }

    bool done = (g_motor_position[OS_AXIS_RA] == g_goto_target[OS_AXIS_RA] &&
                 g_motor_position[OS_AXIS_DEC] == g_goto_target[OS_AXIS_DEC]);

    if (done) {
        complete_motion();
    }
}

static void update_manual_motion(float dt)
{
    if (!g_manual_active) {
        return;
    }

    for (uint8_t axis = 0; axis < OS_NUM_AXES; axis++) {
        update_motor_axis(axis, dt);
    }
}

static void update_tracking(float dt)
{
    if (!g_tracking_enabled) {
        return;
    }

    bool ra_guide_active = (g_guide_pulse.active && g_guide_axis == OS_AXIS_RA);
    if (!ra_guide_active) {
        uint32_t freq = (uint32_t)tracking_frequency_hz();
        (void)os_hal_motor_set_direction(OS_AXIS_RA, true);
        (void)os_hal_motor_set_frequency(OS_AXIS_RA, freq);
    }

    (void)os_hal_motor_set_frequency(OS_AXIS_DEC, 0);
    update_motor_axis(OS_AXIS_RA, dt);
    update_motor_axis(OS_AXIS_DEC, dt);
}

static void update_guide_pulse(float dt)
{
    if (!g_guide_pulse.active) {
        return;
    }

    if (g_guide_remaining_ms == 0) {
        g_guide_pulse.active = false;
        return;
    }

    float base = tracking_frequency_hz();
    float extra = base * g_guide_rate_fraction;
    float freq = g_guide_forward ? (base + extra) : (base - extra);
    if (freq < 0.0f) {
        freq = 0.0f;
    }

    (void)os_hal_motor_set_direction(g_guide_axis, g_guide_forward);
    (void)os_hal_motor_set_frequency(g_guide_axis, (uint32_t)freq);
    update_motor_axis(g_guide_axis, dt);

    uint32_t elapsed = (uint32_t)(dt * 1000.0f);
    if (elapsed == 0) {
        elapsed = 1;
    }
    if (g_guide_remaining_ms > elapsed) {
        g_guide_remaining_ms -= elapsed;
    } else {
        g_guide_remaining_ms = 0;
        g_guide_pulse.active = false;
    }
}

static bool solve_2x2(const double A[2][2], const double b[2], double x[2])
{
    double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    if (fabs(det) < 1e-12) {
        return false;
    }

    x[0] = (b[0] * A[1][1] - A[0][1] * b[1]) / det;
    x[1] = (A[0][0] * b[1] - b[0] * A[1][0]) / det;
    return true;
}

static bool solve_linear_6(double A[6][6], double b[6], double x[6])
{
    for (int i = 0; i < 6; i++) {
        x[i] = 0.0;
    }

    for (int col = 0; col < 6; col++) {
        int pivot = col;
        double max_abs = fabs(A[col][col]);
        for (int row = col + 1; row < 6; row++) {
            double v = fabs(A[row][col]);
            if (v > max_abs) {
                max_abs = v;
                pivot = row;
            }
        }

        if (max_abs < 1e-12) {
            return false;
        }

        if (pivot != col) {
            for (int j = col; j < 6; j++) {
                double tmp = A[pivot][j];
                A[pivot][j] = A[col][j];
                A[col][j] = tmp;
            }
            double tmp = b[pivot];
            b[pivot] = b[col];
            b[col] = tmp;
        }

        double diag = A[col][col];
        for (int j = col; j < 6; j++) {
            A[col][j] /= diag;
        }
        b[col] /= diag;

        for (int row = col + 1; row < 6; row++) {
            double factor = A[row][col];
            if (fabs(factor) < 1e-9) {
                continue;
            }
            for (int j = col; j < 6; j++) {
                A[row][j] -= factor * A[col][j];
            }
            b[row] -= factor * b[col];
        }
    }

    for (int i = 5; i >= 0; i--) {
        double sum = b[i];
        for (int j = i + 1; j < 6; j++) {
            sum -= A[i][j] * x[j];
        }
        x[i] = sum / A[i][i];
    }
    return true;
}

static double compute_residual_arcsec(const double x[6])
{
    if (g_align_star_count == 0) {
        return 0.0;
    }

    double sum_sq = 0.0;
    for (uint8_t i = 0; i < g_align_star_count; i++) {
        double ra_arcsec = (double)g_align_stars[i].ra_hours * 54000.0;
        double dec_arcsec = (double)g_align_stars[i].dec_degrees * 3600.0;
        double ra_steps = (double)g_align_motors[i].ra_steps;
        double dec_steps = (double)g_align_motors[i].dec_steps;

        double pred_ra = x[0] * ra_arcsec + x[1] * dec_arcsec + x[2];
        double pred_dec = x[3] * ra_arcsec + x[4] * dec_arcsec + x[5];

        double diff_ra = pred_ra - ra_steps;
        double diff_dec = pred_dec - dec_steps;
        sum_sq += (diff_ra * diff_ra + diff_dec * diff_dec);
    }

    double rms_steps = sqrt(sum_sq / (2.0 * (double)g_align_star_count));
    double residual_arcsec = rms_steps / OS_STEPS_PER_ARCSEC;
    return residual_arcsec;
}

static void calibration_save(void)
{
    (void)os_hal_nvm_write(0, (const uint8_t *)&g_calibration, (uint16_t)sizeof(g_calibration));
}

static void calibration_load(void)
{
    os_calibration_t cal = {0};
    if (os_hal_nvm_read(0, (uint8_t *)&cal, (uint16_t)sizeof(cal)) == OS_ERR_NONE) {
        if (cal.valid) {
            g_calibration = cal;
        }
    }
}

os_error_t os_init(void)
{
    g_initialized = false;
    g_state = OS_STATE_INITIALIZING;

    g_tracking_enabled = true;
    g_track_rate = OS_TRACK_RATE_SIDEREAL;
    g_track_custom_factor = 1.0f;

    g_pending_ra_set = false;
    g_pending_dec_set = false;
    g_pending_ra = 0.0f;
    g_pending_dec = 0.0f;

    g_goto_active = false;
    g_park_pending = false;
    g_current_action = OS_ACTION_NONE;
    g_goto_target[OS_AXIS_RA] = 0;
    g_goto_target[OS_AXIS_DEC] = 0;

    g_manual_active = false;
    g_manual_axis = OS_AXIS_RA;
    g_manual_forward = true;
    g_custom_speed_arcsec_per_sec = 15.0f;

    g_guide_pulse.active = false;
    g_guide_remaining_ms = 0;
    g_guide_axis = OS_AXIS_RA;
    g_guide_forward = true;
    g_guide_rate_fraction = 0.5f;

    g_align_mode = OS_ALIGN_1STAR;
    g_align_star_count = 0;
    g_residual_valid = false;
    g_residual_arcsec = 0.0f;

    g_pec_enabled = false;
    memset(&g_pec_table, 0, sizeof(g_pec_table));
    memset(&g_calibration, 0, sizeof(g_calibration));
    g_calibration.valid = false;

    g_site = (os_site_info_t){45.0f, -75.0f, 100.0f, 0, true};
    g_gps_locked = false;

    for (size_t i = 0; i < OS_NUM_CHANNELS; i++) {
        g_comm_rx_len[i] = 0;
    }

    for (uint8_t axis = 0; axis < OS_NUM_AXES; axis++) {
        g_motor_fraction[axis] = 0.0;
    }

    (void)os_hal_nvm_init();
    calibration_load();

    for (uint8_t channel = 0; channel < OS_NUM_CHANNELS; channel++) {
        (void)os_hal_comm_init(channel);
    }

    if (os_hal_motor_init(OS_AXIS_RA) != OS_ERR_NONE ||
        os_hal_motor_init(OS_AXIS_DEC) != OS_ERR_NONE) {
        g_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    (void)os_hal_timer_motor_init();

    enable_all_axes(true);
    g_state = OS_STATE_IDLE_TRACKING;
    g_initialized = true;
    return OS_ERR_NONE;
}

void os_loop_iteration(void)
{
    if (!g_initialized) {
        return;
    }

    os_site_info_t site;
    if (os_hal_gps_poll(&site) == OS_ERR_NONE && site.valid) {
        g_site = site;
        g_gps_locked = true;
    } else {
        g_gps_locked = false;
    }

    for (uint8_t channel = 0; channel < OS_NUM_CHANNELS; channel++) {
        int16_t available = os_hal_comm_available(channel);
        if (available <= 0) {
            continue;
        }

        for (int16_t i = 0; i < available; i++) {
            char c = os_hal_comm_read(channel);
            if (c == '\0') {
                continue;
            }

            if (g_comm_rx_len[channel] < (OS_MAX_COMMAND_LENGTH - 1)) {
                g_comm_rx[channel][g_comm_rx_len[channel]++] = c;
            } else {
                g_comm_rx_len[channel] = 0;
                continue;
            }

            if (c == OS_LX200_CMD_SUFFIX) {
                char reply[OS_MAX_REPLY_LENGTH];
                size_t reply_len = 0;
                (void)os_command_parse(g_comm_rx[channel], g_comm_rx_len[channel],
                                       channel, reply, sizeof(reply), &reply_len);
                (void)os_hal_comm_write(channel, reply, reply_len);
                g_comm_rx_len[channel] = 0;
            }
        }
    }

    bool fault = false;
    for (uint8_t axis = 0; axis < OS_NUM_AXES; axis++) {
        if (os_hal_limit_is_triggered(axis)) {
            stop_axis(axis);
            fault = true;
        }
    }

    if (fault) {
        g_state = OS_STATE_FAULT;
        g_goto_active = false;
        g_park_pending = false;
        g_manual_active = false;
        g_guide_pulse.active = false;
        g_current_action = OS_ACTION_NONE;
        return;
    }

    float dt = OS_LOOP_DT_SEC;

    update_guide_pulse(dt);

    if (g_goto_active) {
        update_goto_park(dt);
    } else if (g_manual_active) {
        update_manual_motion(dt);
    } else if (g_state == OS_STATE_IDLE_TRACKING && !g_park_pending) {
        update_tracking(dt);
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
    if (length == 0 || length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_COMMAND_FORMAT;
    }
    if (reply_buffer_size == 0 || reply_buffer_size > OS_MAX_REPLY_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *reply_length = 0;
    reply_buffer[0] = '\0';

    size_t start = 0;
    size_t end = length;
    if (start < end && command[start] == OS_LX200_CMD_PREFIX) {
        start++;
    }
    if (end > start && command[end - 1] == OS_LX200_CMD_SUFFIX) {
        end--;
    }

    while (start < end &&
           (command[start] == ' ' || command[start] == '\r' || command[start] == '\n')) {
        start++;
    }
    while (end > start &&
           (command[end - 1] == ' ' || command[end - 1] == '\r' || command[end - 1] == '\n')) {
        end--;
    }

    size_t clen = end - start;
    if (clen == 0 || clen >= OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_COMMAND_FORMAT;
    }

    char cmd[OS_MAX_COMMAND_LENGTH];
    memcpy(cmd, command + start, clen);
    cmd[clen] = '\0';

    if (strcmp(cmd, "GR") == 0) {
        os_equatorial_coord_t coord;
        os_error_t err = os_query_coordinates(&coord);
        if (err != OS_ERR_NONE) {
            set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "0");
            return err;
        }
        int hours = (int)coord.ra_hours;
        float fractional_minutes = (coord.ra_hours - (float)hours) * 60.0f;
        int minutes = (int)fractional_minutes;
        int seconds = (int)((fractional_minutes - (float)minutes) * 60.0f);
        set_reply_fmt(reply_buffer, reply_buffer_size, reply_length,
                      "%02d:%02d:%02d", hours, minutes, seconds);
        return OS_ERR_NONE;
    }

    if (strcmp(cmd, "GD") == 0) {
        os_equatorial_coord_t coord;
        os_error_t err = os_query_coordinates(&coord);
        if (err != OS_ERR_NONE) {
            set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "0");
            return err;
        }
        float abs_dec = fabsf(coord.dec_degrees);
        int sign = (coord.dec_degrees < 0.0f) ? -1 : 1;
        int degrees = (int)abs_dec;
        float fractional_minutes = (abs_dec - (float)degrees) * 60.0f;
        int minutes = (int)fractional_minutes;
        int seconds = (int)((fractional_minutes - (float)minutes) * 60.0f);
        int signed_degrees = sign * degrees;
        set_reply_fmt(reply_buffer, reply_buffer_size, reply_length,
                      "%+03d*%02d:%02d", signed_degrees, minutes, seconds);
        return OS_ERR_NONE;
    }

    if (strcmp(cmd, "GVP") == 0) {
        set_reply_fmt(reply_buffer, reply_buffer_size, reply_length,
                      "OnStep %d.%d.%d",
                      OS_FIRMWARE_VERSION_MAJOR,
                      OS_FIRMWARE_VERSION_MINOR,
                      OS_FIRMWARE_VERSION_PATCH);
        return OS_ERR_NONE;
    }

    if (strcmp(cmd, "GSTATE") == 0) {
        set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "%d", (int)g_state);
        return OS_ERR_NONE;
    }

    if (strcmp(cmd, "GMOVING") == 0) {
        bool moving = false;
        os_error_t err = os_query_is_moving(&moving);
        if (err != OS_ERR_NONE) {
            set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "0");
            return err;
        }
        set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "%u", moving ? 1u : 0u);
        return OS_ERR_NONE;
    }

    if (strcmp(cmd, "GGPS") == 0) {
        bool locked = false;
        os_error_t err = os_query_gps_locked(&locked);
        if (err != OS_ERR_NONE) {
            set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "0");
            return err;
        }
        set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "%u", locked ? 1u : 0u);
        return OS_ERR_NONE;
    }

    if (strcmp(cmd, "PARK") == 0 || strcmp(cmd, "hP") == 0) {
        os_error_t err = os_park();
        if (err != OS_ERR_NONE) {
            set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "0");
            return err;
        }
        set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "1");
        return OS_ERR_NONE;
    }

    if (strcmp(cmd, "UNPARK") == 0 || strcmp(cmd, "hO") == 0) {
        os_error_t err = os_unpark();
        if (err != OS_ERR_NONE) {
            set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "0");
            return err;
        }
        set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "1");
        return OS_ERR_NONE;
    }

    if (strcmp(cmd, "Me") == 0 || strcmp(cmd, "ME") == 0 ||
        strcmp(cmd, "Mw") == 0 || strcmp(cmd, "MW") == 0 ||
        strcmp(cmd, "Mn") == 0 || strcmp(cmd, "MN") == 0 ||
        strcmp(cmd, "Ms") == 0 || strcmp(cmd, "MS") == 0) {
        os_direction_t dir;
        if (strcmp(cmd, "Me") == 0 || strcmp(cmd, "ME") == 0) {
            dir = OS_DIRECTION_EAST;
        } else if (strcmp(cmd, "Mw") == 0 || strcmp(cmd, "MW") == 0) {
            dir = OS_DIRECTION_WEST;
        } else if (strcmp(cmd, "Mn") == 0 || strcmp(cmd, "MN") == 0) {
            dir = OS_DIRECTION_NORTH;
        } else {
            dir = OS_DIRECTION_SOUTH;
        }

        os_error_t err = os_move_start(dir, OS_SPEED_MEDIUM);
        if (err != OS_ERR_NONE) {
            set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "0");
            return err;
        }
        set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "1");
        return OS_ERR_NONE;
    }

    if (strcmp(cmd, "QSTOP") == 0 || strcmp(cmd, "Q") == 0) {
        os_error_t err = os_move_stop();
        if (err != OS_ERR_NONE) {
            set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "0");
            return err;
        }
        set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "1");
        return OS_ERR_NONE;
    }

    if (strncmp(cmd, "Sr", 2) == 0) {
        float ra_val = 0.0f;
        if (sscanf(cmd + 2, "%f", &ra_val) != 1) {
            return OS_ERR_COMMAND_FORMAT;
        }

        g_pending_ra = ra_val;
        g_pending_ra_set = true;

        if (g_pending_ra_set && g_pending_dec_set) {
            os_equatorial_coord_t target;
            target.ra_hours = g_pending_ra;
            target.dec_degrees = g_pending_dec;
            g_pending_ra_set = false;
            g_pending_dec_set = false;

            os_error_t err = os_goto_equatorial(target);
            if (err != OS_ERR_NONE) {
                set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "0");
                return err;
            }
        }

        set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "1");
        return OS_ERR_NONE;
    }

    if (strncmp(cmd, "Sd", 2) == 0) {
        float dec_val = 0.0f;
        if (sscanf(cmd + 2, "%f", &dec_val) != 1) {
            return OS_ERR_COMMAND_FORMAT;
        }

        g_pending_dec = dec_val;
        g_pending_dec_set = true;

        if (g_pending_ra_set && g_pending_dec_set) {
            os_equatorial_coord_t target;
            target.ra_hours = g_pending_ra;
            target.dec_degrees = g_pending_dec;
            g_pending_ra_set = false;
            g_pending_dec_set = false;

            os_error_t err = os_goto_equatorial(target);
            if (err != OS_ERR_NONE) {
                set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "0");
                return err;
            }
        }

        set_reply_fmt(reply_buffer, reply_buffer_size, reply_length, "1");
        return OS_ERR_NONE;
    }

    return OS_ERR_NOT_SUPPORTED;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target)
{
    if (!eq_valid(target.ra_hours, target.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    for (uint8_t axis = 0; axis < OS_NUM_AXES; axis++) {
        if (os_hal_limit_is_triggered(axis)) {
            return OS_ERR_LIMIT_TRIGGERED;
        }
    }

    int32_t ra_steps = 0;
    int32_t dec_steps = 0;
    target_to_steps(&target, &ra_steps, &dec_steps);

    g_goto_target[OS_AXIS_RA] = ra_steps;
    g_goto_target[OS_AXIS_DEC] = dec_steps;

    enable_all_axes(true);
    g_goto_active = true;
    g_park_pending = false;
    g_current_action = OS_ACTION_GOTO;
    g_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target)
{
    if (!horizontal_valid(target.azimuth_degrees, target.altitude_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    for (uint8_t axis = 0; axis < OS_NUM_AXES; axis++) {
        if (os_hal_limit_is_triggered(axis)) {
            return OS_ERR_LIMIT_TRIGGERED;
        }
    }

    g_goto_target[OS_AXIS_RA] = (int32_t)(target.azimuth_degrees * OS_STEPS_PER_DEGREE);
    g_goto_target[OS_AXIS_DEC] = (int32_t)(target.altitude_degrees * OS_STEPS_PER_DEGREE);

    enable_all_axes(true);
    g_goto_active = true;
    g_park_pending = false;
    g_current_action = OS_ACTION_GOTO;
    g_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void)
{
    stop_all_axes();
    g_goto_active = false;
    g_park_pending = false;
    g_current_action = OS_ACTION_NONE;
    if (g_state == OS_STATE_GOTO) {
        g_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor)
{
    if ((int)rate < OS_TRACK_RATE_SIDEREAL || (int)rate > OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (rate == OS_TRACK_RATE_CUSTOM) {
        if (!isfinite(custom_factor) || custom_factor <= 0.0f) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        g_track_custom_factor = custom_factor;
    }

    g_track_rate = rate;
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
    g_tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void)
{
    g_tracking_enabled = false;
    if (g_state == OS_STATE_IDLE_TRACKING) {
        stop_all_axes();
    }
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms)
{
    if (duration_ms == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    uint8_t axis = OS_AXIS_RA;
    bool forward = true;
    if (!direction_to_axis_forward(direction, &axis, &forward)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_guide_pulse.active = true;
    g_guide_pulse.duration_ms = duration_ms;
    g_guide_pulse.rate_fraction = g_guide_rate_fraction;
    g_guide_pulse.direction_east = (axis == OS_AXIS_RA && forward);
    g_guide_pulse.direction_north = (axis == OS_AXIS_DEC && forward);
    g_guide_pulse.dec_priority = (axis == OS_AXIS_DEC);

    g_guide_remaining_ms = duration_ms;
    g_guide_axis = axis;
    g_guide_forward = forward;
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction)
{
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
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
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode)
{
    if ((int)mode < OS_ALIGN_1STAR || (int)mode > OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_align_mode = mode;
    g_align_star_count = 0;
    g_residual_valid = false;
    g_residual_arcsec = 0.0f;
    g_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos)
{
    if (!eq_valid(star_coord.ra_hours, star_coord.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (g_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    if (g_align_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    g_align_stars[g_align_star_count] = star_coord;
    g_align_motors[g_align_star_count] = motor_pos;
    g_align_star_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void)
{
    uint8_t min_stars = OS_CALIBRATION_MIN_STARS;
    if (g_align_mode == OS_ALIGN_2STAR) {
        min_stars = 2;
    } else if (g_align_mode == OS_ALIGN_3STAR || g_align_mode == OS_ALIGN_NSTAR) {
        min_stars = 3;
    }

    if (g_align_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    double x[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool ok = true;

    if (g_align_mode == OS_ALIGN_1STAR) {
        x[0] = 0.0;
        x[1] = 0.0;
        x[2] = (double)g_align_motors[0].ra_steps;
        x[3] = 0.0;
        x[4] = 0.0;
        x[5] = (double)g_align_motors[0].dec_steps;
    } else if (g_align_mode == OS_ALIGN_2STAR) {
        double A_ra[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
        double b_ra[2] = {0.0, 0.0};
        double A_dec[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
        double b_dec[2] = {0.0, 0.0};

        for (uint8_t i = 0; i < g_align_star_count; i++) {
            double ra_arcsec = (double)g_align_stars[i].ra_hours * 54000.0;
            double dec_arcsec = (double)g_align_stars[i].dec_degrees * 3600.0;
            double ra_steps = (double)g_align_motors[i].ra_steps;
            double dec_steps = (double)g_align_motors[i].dec_steps;

            A_ra[0][0] += ra_arcsec * ra_arcsec;
            A_ra[0][1] += ra_arcsec;
            A_ra[1][0] += ra_arcsec;
            A_ra[1][1] += 1.0;
            b_ra[0] += ra_arcsec * ra_steps;
            b_ra[1] += ra_steps;

            A_dec[0][0] += dec_arcsec * dec_arcsec;
            A_dec[0][1] += dec_arcsec;
            A_dec[1][0] += dec_arcsec;
            A_dec[1][1] += 1.0;
            b_dec[0] += dec_arcsec * dec_steps;
            b_dec[1] += dec_steps;
        }

        double x_ra[2] = {0.0, 0.0};
        double x_dec[2] = {0.0, 0.0};
        ok = solve_2x2(A_ra, b_ra, x_ra) && solve_2x2(A_dec, b_dec, x_dec);
        if (ok) {
            x[0] = x_ra[0];
            x[1] = 0.0;
            x[2] = x_ra[1];
            x[3] = 0.0;
            x[4] = x_dec[0];
            x[5] = x_dec[1];
        }
    } else {
        double A[6][6] = {{0.0}};
        double b[6] = {0.0};

        for (uint8_t i = 0; i < g_align_star_count; i++) {
            double ra_arcsec = (double)g_align_stars[i].ra_hours * 54000.0;
            double dec_arcsec = (double)g_align_stars[i].dec_degrees * 3600.0;
            double ra_steps = (double)g_align_motors[i].ra_steps;
            double dec_steps = (double)g_align_motors[i].dec_steps;

            A[0][0] += ra_arcsec * ra_arcsec;
            A[0][1] += ra_arcsec * dec_arcsec;
            A[0][2] += ra_arcsec;
            A[1][0] += ra_arcsec * dec_arcsec;
            A[1][1] += dec_arcsec * dec_arcsec;
            A[1][2] += dec_arcsec;
            A[2][0] += ra_arcsec;
            A[2][1] += dec_arcsec;
            A[2][2] += 1.0;

            A[3][3] += ra_arcsec * ra_arcsec;
            A[3][4] += ra_arcsec * dec_arcsec;
            A[3][5] += ra_arcsec;
            A[4][3] += ra_arcsec * dec_arcsec;
            A[4][4] += dec_arcsec * dec_arcsec;
            A[4][5] += dec_arcsec;
            A[5][3] += ra_arcsec;
            A[5][4] += dec_arcsec;
            A[5][5] += 1.0;

            b[0] += ra_arcsec * ra_steps;
            b[1] += dec_arcsec * ra_steps;
            b[2] += ra_steps;
            b[3] += ra_arcsec * dec_steps;
            b[4] += dec_arcsec * dec_steps;
            b[5] += dec_steps;
        }

        ok = solve_linear_6(A, b, x);
    }

    if (!ok) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    double residual = compute_residual_arcsec(x);
    if (g_align_mode == OS_ALIGN_NSTAR && residual > OS_ALIGN_MAX_RESIDUAL_ARCSEC) {
        return OS_ERR_CALIBRATION_FAILED;
    }

    g_calibration.matrix_ra_to_ra = (float)x[0];
    g_calibration.matrix_ra_to_dec = (float)x[1];
    g_calibration.matrix_dec_to_ra = (float)x[3];
    g_calibration.matrix_dec_to_dec = (float)x[4];
    g_calibration.offset_ra_arcsec = (float)(x[2] / OS_STEPS_PER_ARCSEC);
    g_calibration.offset_dec_arcsec = (float)(x[5] / OS_STEPS_PER_ARCSEC);
    g_calibration.valid = true;

    calibration_save();
    g_residual_valid = true;
    g_residual_arcsec = (float)residual;
    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec)
{
    if (residual_arcsec == NULL) {
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
    g_align_star_count = 0;
    g_residual_valid = false;
    g_residual_arcsec = 0.0f;
    if (g_state == OS_STATE_ALIGNMENT) {
        g_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_park(void)
{
    if (g_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    for (uint8_t axis = 0; axis < OS_NUM_AXES; axis++) {
        if (os_hal_limit_is_triggered(axis)) {
            return OS_ERR_LIMIT_TRIGGERED;
        }
    }

    g_goto_target[OS_AXIS_RA] = g_park_ra_steps;
    g_goto_target[OS_AXIS_DEC] = g_park_dec_steps;

    enable_all_axes(true);
    g_goto_active = true;
    g_park_pending = true;
    g_current_action = OS_ACTION_PARK;
    g_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void)
{
    if (g_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    for (uint8_t channel = 0; channel < OS_NUM_CHANNELS; channel++) {
        (void)os_hal_comm_init(channel);
    }

    (void)os_hal_motor_init(OS_AXIS_RA);
    (void)os_hal_motor_init(OS_AXIS_DEC);
    enable_all_axes(true);
    (void)os_hal_timer_motor_init();

    g_tracking_enabled = true;
    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos)
{
    if (!eq_valid(park_pos.ra_hours, park_pos.dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_park_ra_steps = (int32_t)(park_pos.ra_hours * OS_STEPS_PER_HOUR);
    g_park_dec_steps = (int32_t)(park_pos.dec_degrees * OS_STEPS_PER_DEGREE);
    g_park_position_set = true;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed)
{
    uint8_t axis = OS_AXIS_RA;
    bool forward = true;

    if (!direction_to_axis_forward(direction, &axis, &forward)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if ((int)speed < OS_SPEED_SLOW || (int)speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    float deg_per_sec = speed_to_deg_per_sec(speed);
    if (deg_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    uint32_t freq = (uint32_t)(deg_per_sec * OS_STEPS_PER_DEGREE);
    enable_all_axes(true);
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, freq);

    g_manual_active = true;
    g_manual_axis = axis;
    g_manual_forward = forward;
    g_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void)
{
    stop_all_axes();
    g_manual_active = false;
    if (g_state == OS_STATE_MANUAL_MOTION) {
        g_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec)
{
    if (!isfinite(arcsec_per_sec) || arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_custom_speed_arcsec_per_sec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state)
{
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *state = g_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord)
{
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    int32_t ra_steps = os_hal_motor_get_position(OS_AXIS_RA);
    int32_t dec_steps = os_hal_motor_get_position(OS_AXIS_DEC);

    os_equatorial_coord_t result;
    steps_to_coord(ra_steps, dec_steps, &result);

    result.ra_hours = fmodf(result.ra_hours, 24.0f);
    if (result.ra_hours < 0.0f) {
        result.ra_hours += 24.0f;
    }

    if (result.dec_degrees > OS_DEC_MAX_DEG) {
        result.dec_degrees = OS_DEC_MAX_DEG;
    } else if (result.dec_degrees < OS_DEC_MIN_DEG) {
        result.dec_degrees = OS_DEC_MIN_DEG;
    }

    *coord = result;
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

    pos->ra_steps = os_hal_motor_get_position(OS_AXIS_RA);
    pos->dec_steps = os_hal_motor_get_position(OS_AXIS_DEC);
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
    if (moving == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *moving = (g_goto_active || g_manual_active || g_guide_pulse.active);
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
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    memcpy(&g_pec_table, table, sizeof(g_pec_table));
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    memcpy(table, &g_pec_table, sizeof(g_pec_table));
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec)
{
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    int index = (int)floorf(worm_phase_deg);
    if (index >= OS_PEC_TABLE_SIZE) {
        index = OS_PEC_TABLE_SIZE - 1;
    }
    g_pec_table.corrections[index] = error_arcsec;
    g_pec_table.valid = true;
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
    memset(&g_calibration, 0, sizeof(g_calibration));
    g_calibration.valid = false;
    calibration_save();
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_init(uint8_t axis)
{
    if (!valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!g_motor_initialized[axis]) {
        g_motor_position[axis] = 0;
        g_motor_initialized[axis] = true;
    }

    g_motor_enabled[axis] = false;
    g_motor_frequency[axis] = 0;
    g_motor_fraction[axis] = 0.0;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz)
{
    if (!valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (frequency_hz > 1000000u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_motor_frequency[axis] = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward)
{
    if (!valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_motor_direction[axis] = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable)
{
    if (!valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_motor_enabled[axis] = enable;
    if (!enable) {
        g_motor_frequency[axis] = 0;
        g_motor_fraction[axis] = 0.0;
    }
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis)
{
    if (!valid_axis(axis)) {
        return 0;
    }

    return g_motor_position[axis];
}

os_error_t os_hal_gps_init(void)
{
    g_site.valid = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    site->latitude_degrees = 0.0f;
    site->longitude_degrees = 0.0f;
    site->elevation_metres = 0.0f;
    site->utc_epoch_seconds = 0;
    site->valid = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void)
{
    g_rtc_valid = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds)
{
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!g_rtc_valid) {
        return OS_ERR_TIMEOUT;
    }

    *utc_epoch_seconds = g_rtc_epoch;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds)
{
    g_rtc_epoch = utc_epoch_seconds;
    g_rtc_valid = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void)
{
    for (uint8_t axis = 0; axis < OS_NUM_AXES; axis++) {
        g_limit_triggered[axis] = false;
    }
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis)
{
    if (!valid_axis(axis)) {
        return true;
    }

    return g_limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void)
{
    if (!g_nvm_initialized) {
        memset(g_nvm, 0, sizeof(g_nvm));
        g_nvm_initialized = true;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length)
{
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    uint32_t end = (uint32_t)offset + (uint32_t)length;
    if (end > OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    memcpy(data, &g_nvm[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length)
{
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    uint32_t end = (uint32_t)offset + (uint32_t)length;
    if (end > OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    memcpy(&g_nvm[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel)
{
    if (!valid_channel(channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    memset(&g_comm_rx_fifo[channel], 0, sizeof(g_comm_rx_fifo[channel]));
    memset(&g_comm_tx_fifo[channel], 0, sizeof(g_comm_tx_fifo[channel]));
    g_comm_rx_len[channel] = 0;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel)
{
    if (!valid_channel(channel)) {
        return 0;
    }

    return (int16_t)g_comm_rx_fifo[channel].count;
}

char os_hal_comm_read(uint8_t channel)
{
    if (!valid_channel(channel)) {
        return '\0';
    }

    os_comm_rx_fifo_t *f = &g_comm_rx_fifo[channel];
    if (f->count == 0) {
        return '\0';
    }

    char c = f->data[f->head];
    f->head = (uint16_t)((f->head + 1u) % OS_HAL_RX_FIFO_SIZE);
    f->count--;
    return c;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length)
{
    if (!valid_channel(channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (data == NULL && length != 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (length > OS_HAL_TX_FIFO_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    os_comm_tx_fifo_t *f = &g_comm_tx_fifo[channel];
    if (length > (size_t)(OS_HAL_TX_FIFO_SIZE - f->count)) {
        return OS_ERR_TIMEOUT;
    }

    for (size_t i = 0; i < length; i++) {
        f->data[f->tail] = data[i];
        f->tail = (uint16_t)((f->tail + 1u) % OS_HAL_TX_FIFO_SIZE);
    }

    f->count = (uint16_t)(f->count + length);
    return OS_ERR_NONE;
}

int16_t os_hal_comm_inject_rx(uint8_t channel, const char *data, size_t length)
{
    if (!valid_channel(channel)) {
        return -1;
    }

    if (data == NULL && length != 0) {
        return -1;
    }

    if (length > OS_HAL_RX_FIFO_SIZE) {
        return -1;
    }

    os_comm_rx_fifo_t *f = &g_comm_rx_fifo[channel];
    if (length > (size_t)(OS_HAL_RX_FIFO_SIZE - f->count)) {
        return -1;
    }

    for (size_t i = 0; i < length; i++) {
        f->data[f->tail] = data[i];
        f->tail = (uint16_t)((f->tail + 1u) % OS_HAL_RX_FIFO_SIZE);
    }

    f->count = (uint16_t)(f->count + length);
    return (int16_t)length;
}

int16_t os_hal_comm_tx_available(uint8_t channel)
{
    if (!valid_channel(channel)) {
        return -1;
    }

    return (int16_t)g_comm_tx_fifo[channel].count;
}

char os_hal_comm_tx_read(uint8_t channel)
{
    if (!valid_channel(channel)) {
        return '\0';
    }

    os_comm_tx_fifo_t *f = &g_comm_tx_fifo[channel];
    if (f->count == 0) {
        return '\0';
    }

    char c = f->data[f->head];
    f->head = (uint16_t)((f->head + 1u) % OS_HAL_TX_FIFO_SIZE);
    f->count--;
    return c;
}

void os_hal_comm_tx_clear(uint8_t channel)
{
    if (!valid_channel(channel)) {
        return;
    }

    memset(&g_comm_tx_fifo[channel], 0, sizeof(g_comm_tx_fifo[channel]));
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count)
{
    (void)duration_ms;
    (void)count;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void)
{
    return OS_ERR_NONE;
}

#include "generated_code.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if defined(__GNUC__) || defined(__clang__)
#define OS_WEAK __attribute__((weak))
#else
#define OS_WEAK
#endif

/* -------------------------------------------------------------------------
 * Internal constants and tiny helpers
 * ---------------------------------------------------------------------- */
#define HAL_RX_BUF_SIZE            256
#define HAL_TX_BUF_SIZE            512
#define HAL_NVM_TOTAL              (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)

#define OS_MOTOR_GOTO_HZ           800.0
#define OS_MOTOR_PARK_HZ           300.0
#define OS_MOTOR_ACCEL_HZ_PER_LOOP 25.0
#define OS_MOTOR_MIN_HZ            5.0

#define OS_ALIGN_MAX_RESIDUAL_ARCSEC 300.0

#define NVM_MAGIC 0x4F4E5354u
#define NVM_MAGIC_OFFSET 0u
#define NVM_SITE_OFFSET   16u
#define NVM_CAL_OFFSET    64u

static bool is_valid_axis(int axis) {
    return axis == OS_AXIS_RA || axis == OS_AXIS_DEC;
}

static bool is_valid_channel(int channel) {
    return channel >= 0 && channel < OS_MAX_CHANNELS;
}

static bool is_valid_ra(double ra_hours) {
    return ra_hours >= 0.0 && ra_hours <= 24.0;
}

static bool is_valid_dec(double dec_degrees) {
    return dec_degrees >= -90.0 && dec_degrees <= 90.0;
}

static int64_t round_i64(double x) {
    return (int64_t)(x >= 0.0 ? x + 0.5 : x - 0.5);
}

/* -------------------------------------------------------------------------
 * Weak host HAL implementation
 * ---------------------------------------------------------------------- */
typedef struct {
    uint8_t rx[HAL_RX_BUF_SIZE];
    size_t rx_head;
    size_t rx_tail;
    uint8_t tx[HAL_TX_BUF_SIZE];
    size_t tx_head;
    size_t tx_tail;
    bool initialized;
} hal_channel_t;

static hal_channel_t s_comm[OS_MAX_CHANNELS];

static uint8_t s_nvm[HAL_NVM_TOTAL];

static bool s_limit[2];
static bool s_gps_locked;
static os_site_info_t s_gps_site;
static uint32_t s_rtc_epoch = 1609459200u;
static bool s_rtc_initialized;

static uint64_t s_motor_time_ms[2];
static double s_motor_pos[2];
static bool s_motor_initialized[2];
static bool s_motor_enabled[2];
static bool s_motor_dir[2];
static double s_motor_freq[2];

static void motor_tick(int axis) {
    if (!is_valid_axis(axis) || !s_motor_initialized[axis]) {
        return;
    }

    uint64_t now = s_motor_time_ms[axis] + 1u;
    uint64_t dt = now - s_motor_time_ms[axis];
    s_motor_time_ms[axis] = now;

    if (s_motor_enabled[axis] && s_motor_freq[axis] > 0.0 && dt > 0u) {
        double steps = s_motor_freq[axis] * ((double)dt / 1000.0);
        if (!s_motor_dir[axis]) {
            steps = -steps;
        }
        s_motor_pos[axis] += steps;
    }
}

OS_WEAK os_error_t os_hal_motor_init(int axis) {
    if (!is_valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_motor_initialized[axis] = true;
    s_motor_enabled[axis] = false;
    s_motor_dir[axis] = false;
    s_motor_freq[axis] = 0.0;
    s_motor_pos[axis] = 0.0;
    s_motor_time_ms[axis] = 0u;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_motor_set_frequency(int axis, double frequency_hz) {
    if (!is_valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (frequency_hz < 0.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    motor_tick(axis);

    if (frequency_hz > OS_MOTOR_MAX_FREQ_HZ) {
        frequency_hz = OS_MOTOR_MAX_FREQ_HZ;
    }

    s_motor_freq[axis] = frequency_hz;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_motor_set_direction(int axis, bool forward) {
    if (!is_valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    motor_tick(axis);
    s_motor_dir[axis] = forward;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_motor_enable(int axis, bool enable) {
    if (!is_valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    motor_tick(axis);
    s_motor_enabled[axis] = enable;
    if (!enable) {
        s_motor_freq[axis] = 0.0;
    }
    return OS_ERR_NONE;
}

OS_WEAK int64_t os_hal_motor_get_position(int axis) {
    if (!is_valid_axis(axis)) {
        return 0;
    }

    motor_tick(axis);
    return round_i64(s_motor_pos[axis]);
}

OS_WEAK os_error_t os_hal_timer_motor_init(void) {
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_gps_init(void) {
    s_gps_locked = false;
    memset(&s_gps_site, 0, sizeof(s_gps_site));
    s_gps_site.valid = false;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_gps_locked && s_gps_site.valid) {
        *site = s_gps_site;
        return OS_ERR_NONE;
    }

    memset(site, 0, sizeof(*site));
    site->valid = false;
    return OS_ERR_TIMEOUT;
}

OS_WEAK os_error_t os_hal_rtc_init(void) {
    s_rtc_initialized = true;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!s_rtc_initialized) {
        return OS_ERR_INVALID_STATE;
    }

    *utc_epoch_seconds = s_rtc_epoch;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    s_rtc_epoch = utc_epoch_seconds;
    s_rtc_initialized = true;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_limit_init(void) {
    s_limit[OS_AXIS_RA] = false;
    s_limit[OS_AXIS_DEC] = false;
    return OS_ERR_NONE;
}

OS_WEAK bool os_hal_limit_is_triggered(int axis) {
    if (!is_valid_axis(axis)) {
        return true;
    }

    return s_limit[axis];
}

OS_WEAK os_error_t os_hal_buzzer_beep(uint32_t duration_ms, uint8_t count) {
    (void)duration_ms;
    (void)count;
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_comm_init(int channel) {
    if (!is_valid_channel(channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    memset(&s_comm[channel], 0, sizeof(s_comm[channel]));
    s_comm[channel].initialized = true;
    return OS_ERR_NONE;
}

OS_WEAK int os_hal_comm_available(int channel) {
    if (!is_valid_channel(channel) || !s_comm[channel].initialized) {
        return 0;
    }

    hal_channel_t *c = &s_comm[channel];
    return (int)((c->rx_tail - c->rx_head + HAL_RX_BUF_SIZE) % HAL_RX_BUF_SIZE);
}

OS_WEAK int os_hal_comm_read(int channel) {
    if (!is_valid_channel(channel) || !s_comm[channel].initialized) {
        return -1;
    }

    hal_channel_t *c = &s_comm[channel];
    size_t avail = (c->rx_tail - c->rx_head + HAL_RX_BUF_SIZE) % HAL_RX_BUF_SIZE;
    if (avail == 0u) {
        return -1;
    }

    uint8_t b = c->rx[c->rx_head];
    c->rx_head = (c->rx_head + 1u) % HAL_RX_BUF_SIZE;
    return (int)b;
}

OS_WEAK os_error_t os_hal_comm_write(int channel, const void *data, size_t length) {
    if (!is_valid_channel(channel) || !s_comm[channel].initialized) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (data == NULL && length > 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    hal_channel_t *c = &s_comm[channel];
    const uint8_t *p = (const uint8_t *)data;

    for (size_t i = 0u; i < length; ++i) {
        size_t next = (c->tx_tail + 1u) % HAL_TX_BUF_SIZE;
        if (next == c->tx_head) {
            return OS_ERR_FAULT;
        }
        c->tx[c->tx_tail] = p[i];
        c->tx_tail = next;
    }

    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_nvm_init(void) {
    memset(s_nvm, 0, sizeof(s_nvm));
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_nvm_read(uint32_t offset, void *data, size_t length) {
    if (data == NULL || length == 0u || (uint64_t)offset + (uint64_t)length > HAL_NVM_TOTAL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    memcpy(data, &s_nvm[offset], length);
    return OS_ERR_NONE;
}

OS_WEAK os_error_t os_hal_nvm_write(uint32_t offset, const void *data, size_t length) {
    if (data == NULL || length == 0u || (uint64_t)offset + (uint64_t)length > HAL_NVM_TOTAL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    memcpy(&s_nvm[offset], data, length);
    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Domain state
 * ---------------------------------------------------------------------- */
typedef enum {
    AXIS_IDLE = 0,
    AXIS_GOTO = 1,
    AXIS_MANUAL = 2,
    AXIS_PARK = 3
} axis_kind_t;

typedef struct {
    bool initialized;
    os_state_t state;

    bool goto_active;
    bool park_in_progress;
    bool manual_active[2];
    bool guide_pulse_active;
    bool limit_fault;

    bool busy[2];
    axis_kind_t kind[2];
    double target[2];
    double axis_freq[2];
    double manual_hz[2];
    bool manual_dir[2];

    bool align_active;
    os_align_mode_t align_mode;
    uint8_t align_star_count;
    double align_ra[OS_MAX_ALIGN_STARS];
    double align_dec[OS_MAX_ALIGN_STARS];
    double align_motor0[OS_MAX_ALIGN_STARS];
    double align_motor1[OS_MAX_ALIGN_STARS];
    bool align_residual_computed;
    double align_residual_arcsec;

    bool nvm_valid;
    os_site_info_t site;
    bool gps_locked;
    os_calibration_t cal;

    double guide_bias_hz[2];
    int guide_axis;
    uint32_t guide_remaining_ms;

    double pec_phase[OS_PEC_MAX_POINTS];
    double pec_error[OS_PEC_MAX_POINTS];
    uint8_t pec_count;

    double last_goto_ra_hours;
    double last_goto_dec_degrees;
    bool has_last_goto_target;

    char cmd_buf[OS_MAX_CHANNELS][OS_MAX_COMMAND_LENGTH + 1];
    uint8_t cmd_len[OS_MAX_CHANNELS];
    bool cmd_active[OS_MAX_CHANNELS];
} os_t;

static os_t g_os;

/* -------------------------------------------------------------------------
 * Calibration helpers
 * ---------------------------------------------------------------------- */
static void calibration_identity(os_calibration_t *cal) {
    if (cal == NULL) {
        return;
    }

    cal->valid = false;
    cal->m00 = OS_STEPS_PER_RA_HOUR;
    cal->m01 = 0.0;
    cal->c0 = 0.0;
    cal->m10 = 0.0;
    cal->m11 = OS_STEPS_PER_DEGREE;
    cal->c1 = 0.0;
    cal->model = 0u;
    cal->star_count = 0u;
    cal->residual_arcsec = 0.0;
}

static void apply_calibration(double ra_hours, double dec_degrees, double *motor0, double *motor1) {
    if (motor0 == NULL || motor1 == NULL) {
        return;
    }

    *motor0 = g_os.cal.m00 * ra_hours + g_os.cal.m01 * dec_degrees + g_os.cal.c0;
    *motor1 = g_os.cal.m10 * ra_hours + g_os.cal.m11 * dec_degrees + g_os.cal.c1;
}

static void motor_to_equatorial(double motor0, double motor1, double *ra_hours, double *dec_degrees) {
    if (ra_hours == NULL || dec_degrees == NULL) {
        return;
    }

    double det = g_os.cal.m00 * g_os.cal.m11 - g_os.cal.m01 * g_os.cal.m10;

    if (fabs(det) < 1e-12) {
        *ra_hours = motor0 / OS_STEPS_PER_RA_HOUR;
        *dec_degrees = motor1 / OS_STEPS_PER_DEGREE;
    } else {
        double rx = motor0 - g_os.cal.c0;
        double ry = motor1 - g_os.cal.c1;
        *ra_hours = (g_os.cal.m11 * rx - g_os.cal.m01 * ry) / det;
        *dec_degrees = (-g_os.cal.m10 * rx + g_os.cal.m00 * ry) / det;
    }

    *ra_hours = fmod(*ra_hours, 24.0);
    if (*ra_hours < 0.0) {
        *ra_hours += 24.0;
    }
}

static double det3(const double m[3][3]) {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

static bool solve2(double a, double b, double c, double d, double e, double f, double *x, double *y) {
    if (x == NULL || y == NULL) {
        return false;
    }

    double det = a * d - b * c;
    if (fabs(det) < 1e-15) {
        return false;
    }

    *x = (e * d - b * f) / det;
    *y = (a * f - e * c) / det;
    return true;
}

static bool lsq3(const double A_in[][3], const double b_in[], int n, double x[3], double *residual_out) {
    if (A_in == NULL || b_in == NULL || x == NULL || residual_out == NULL || n < 3) {
        return false;
    }

    double Q[OS_MAX_ALIGN_STARS][3];
    double R[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double b[OS_MAX_ALIGN_STARS];

    for (int i = 0; i < n; ++i) {
        Q[i][0] = A_in[i][0];
        Q[i][1] = A_in[i][1];
        Q[i][2] = A_in[i][2];
        b[i] = b_in[i];
    }

    /* Modified Gram-Schmidt QR */
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
            double dot = 0.0;
            for (int i = 0; i < n; ++i) {
                dot += Q[i][k] * Q[i][j];
            }
            R[k][j] = dot;
            for (int i = 0; i < n; ++i) {
                Q[i][j] -= dot * Q[i][k];
            }
        }
    }

    double y[3] = {0.0, 0.0, 0.0};
    for (int k = 0; k < 3; ++k) {
        double s = 0.0;
        for (int i = 0; i < n; ++i) {
            s += Q[i][k] * b[i];
        }
        y[k] = s;
    }

    for (int k = 2; k >= 0; --k) {
        double s = y[k];
        for (int j = k + 1; j < 3; ++j) {
            s -= R[k][j] * x[j];
        }
        x[k] = s / R[k][k];
    }

    double res_sum = 0.0;
    for (int i = 0; i < n; ++i) {
        double pred = A_in[i][0] * x[0] + A_in[i][1] * x[1] + A_in[i][2] * x[2];
        double r = b_in[i] - pred;
        res_sum += r * r;
    }

    *residual_out = sqrt(res_sum / (double)n);
    return true;
}

static os_error_t compute_alignment(void) {
    if (g_os.align_star_count == 0u) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t min_stars = 3u;

    switch (g_os.align_mode) {
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
            return OS_ERR_INVALID_ARGUMENT;
    }

    if (g_os.align_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t n = g_os.align_star_count;

    if (g_os.align_mode == OS_ALIGN_1STAR) {
        g_os.cal.m00 = OS_STEPS_PER_RA_HOUR;
        g_os.cal.m01 = 0.0;
        g_os.cal.m10 = 0.0;
        g_os.cal.m11 = OS_STEPS_PER_DEGREE;
        g_os.cal.c0 = g_os.align_motor0[0] - g_os.cal.m00 * g_os.align_ra[0];
        g_os.cal.c1 = g_os.align_motor1[0] - g_os.cal.m11 * g_os.align_dec[0];
        g_os.cal.model = 1u;
        g_os.cal.star_count = n;
        g_os.cal.valid = true;
        g_os.cal.residual_arcsec = 0.0;
        return OS_ERR_NONE;
    }

    if (g_os.align_mode == OS_ALIGN_2STAR) {
        double a0 = g_os.align_ra[0];
        double a1 = 1.0;
        double b0 = g_os.align_ra[1];
        double b1 = 1.0;

        double m00 = 0.0;
        double c0 = 0.0;
        if (!solve2(a0, a1, b0, b1, g_os.align_motor0[0], g_os.align_motor0[1], &m00, &c0)) {
            return OS_ERR_INVALID_STATE;
        }

        double a2 = g_os.align_dec[0];
        double b2 = g_os.align_dec[1];
        double m11 = 0.0;
        double c1 = 0.0;
        if (!solve2(a2, a1, b2, b1, g_os.align_motor1[0], g_os.align_motor1[1], &m11, &c1)) {
            return OS_ERR_INVALID_STATE;
        }

        g_os.cal.m00 = m00;
        g_os.cal.m01 = 0.0;
        g_os.cal.c0 = c0;
        g_os.cal.m10 = 0.0;
        g_os.cal.m11 = m11;
        g_os.cal.c1 = c1;
        g_os.cal.model = 2u;
        g_os.cal.star_count = n;
        g_os.cal.valid = true;
        g_os.cal.residual_arcsec = 0.0;
        return OS_ERR_NONE;
    }

    if (g_os.align_mode == OS_ALIGN_3STAR && n == 3) {
        double A3[3][3] = {
            {g_os.align_ra[0], g_os.align_dec[0], 1.0},
            {g_os.align_ra[1], g_os.align_dec[1], 1.0},
            {g_os.align_ra[2], g_os.align_dec[2], 1.0}
        };

        double scale = 0.0;
        for (int r = 0; r < 3; ++r) {
            double s = fabs(A3[r][0]) + fabs(A3[r][1]) + 1.0;
            if (s > scale) {
                scale = s;
            }
        }

        double tol = 1e-12 * scale * scale * scale;
        if (fabs(det3(A3)) < tol) {
            return OS_ERR_INVALID_STATE;
        }

        double A[3][3];
        double b0[3];
        double b1[3];

        for (int i = 0; i < 3; ++i) {
            A[i][0] = g_os.align_ra[i];
            A[i][1] = g_os.align_dec[i];
            A[i][2] = 1.0;
            b0[i] = g_os.align_motor0[i];
            b1[i] = g_os.align_motor1[i];
        }

        double x0[3] = {0.0, 0.0, 0.0};
        double x1[3] = {0.0, 0.0, 0.0};
        double res = 0.0;

        if (!lsq3(A, b0, 3, x0, &res) || !lsq3(A, b1, 3, x1, &res)) {
            return OS_ERR_INVALID_STATE;
        }

        g_os.cal.m00 = x0[0];
        g_os.cal.m01 = x0[1];
        g_os.cal.c0 = x0[2];
        g_os.cal.m10 = x1[0];
        g_os.cal.m11 = x1[1];
        g_os.cal.c1 = x1[2];
        g_os.cal.model = 3u;
        g_os.cal.star_count = n;
        g_os.cal.valid = true;
        g_os.cal.residual_arcsec = 0.0;
        return OS_ERR_NONE;
    }

    double A[OS_MAX_ALIGN_STARS][3];
    double b0[OS_MAX_ALIGN_STARS];
    double b1[OS_MAX_ALIGN_STARS];

    for (int i = 0; i < n; ++i) {
        A[i][0] = g_os.align_ra[i];
        A[i][1] = g_os.align_dec[i];
        A[i][2] = 1.0;
        b0[i] = g_os.align_motor0[i];
        b1[i] = g_os.align_motor1[i];
    }

    double x0[3] = {0.0, 0.0, 0.0};
    double x1[3] = {0.0, 0.0, 0.0};
    double res0 = 0.0;
    double res1 = 0.0;

    if (!lsq3(A, b0, n, x0, &res0) || !lsq3(A, b1, n, x1, &res1)) {
        return OS_ERR_INVALID_STATE;
    }

    double residual_steps = (res0 + res1) / 2.0;
    double residual_arcsec = residual_steps * 3600.0 / OS_STEPS_PER_DEGREE;

    if (residual_arcsec > OS_ALIGN_MAX_RESIDUAL_ARCSEC) {
        return OS_ERR_INVALID_STATE;
    }

    g_os.cal.m00 = x0[0];
    g_os.cal.m01 = x0[1];
    g_os.cal.c0 = x0[2];
    g_os.cal.m10 = x1[0];
    g_os.cal.m11 = x1[1];
    g_os.cal.c1 = x1[2];
    g_os.cal.model = g_os.align_mode;
    g_os.cal.star_count = n;
    g_os.cal.valid = true;
    g_os.cal.residual_arcsec = residual_arcsec;
    g_os.align_residual_computed = true;
    g_os.align_residual_arcsec = residual_arcsec;

    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * NVM helpers
 * ---------------------------------------------------------------------- */
static os_error_t nvm_save_calibration(void) {
    uint32_t magic = NVM_MAGIC;
    os_error_t err = os_hal_nvm_write(NVM_MAGIC_OFFSET, &magic, sizeof(magic));
    if (err != OS_ERR_NONE) {
        return err;
    }

    return os_hal_nvm_write(NVM_CAL_OFFSET, &g_os.cal, sizeof(g_os.cal));
}

static void nvm_load_state(void) {
    uint32_t magic = 0u;
    if (os_hal_nvm_read(NVM_MAGIC_OFFSET, &magic, sizeof(magic)) != OS_ERR_NONE || magic != NVM_MAGIC) {
        g_os.nvm_valid = false;
        calibration_identity(&g_os.cal);
        return;
    }

    g_os.nvm_valid = true;

    os_calibration_t cal;
    memset(&cal, 0, sizeof(cal));
    if (os_hal_nvm_read(NVM_CAL_OFFSET, &cal, sizeof(cal)) == OS_ERR_NONE && cal.valid) {
        g_os.cal = cal;
    } else {
        calibration_identity(&g_os.cal);
    }

    os_site_info_t site;
    memset(&site, 0, sizeof(site));
    if (os_hal_nvm_read(NVM_SITE_OFFSET, &site, sizeof(site)) == OS_ERR_NONE && site.valid) {
        g_os.site = site;
    }
}

/* -------------------------------------------------------------------------
 * Command handling
 * ---------------------------------------------------------------------- */
static void command_reply(int channel, const char *fmt, ...) {
    char buf[OS_MAX_REPLY_LENGTH + 1];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    os_hal_comm_write(channel, buf, strlen(buf));
}

static void command_execute(int channel, const char *cmd) {
    if (cmd == NULL || cmd[0] != ':') {
        command_reply(channel, "0");
        return;
    }

    char clean[OS_MAX_COMMAND_LENGTH + 1];
    strncpy(clean, cmd, sizeof(clean) - 1u);
    clean[sizeof(clean) - 1u] = '\0';

    size_t len = strlen(clean);
    while (len > 0u && (clean[len - 1u] == '#' || clean[len - 1u] == '\n' || clean[len - 1u] == '\r')) {
        clean[--len] = '\0';
    }

    if (strcmp(clean, ":GR") == 0) {
        double ra = 0.0;
        double dec = 0.0;
        (void)os_query_coordinates(&ra, &dec);
        command_reply(channel, "%.6f#", ra);
        return;
    }

    if (strcmp(clean, ":GD") == 0) {
        double ra = 0.0;
        double dec = 0.0;
        (void)os_query_coordinates(&ra, &dec);
        command_reply(channel, "%.6f#", dec);
        return;
    }

    if (strcmp(clean, ":GVP") == 0) {
        command_reply(channel, "OnStep-1.0#");
        return;
    }

    if (strcmp(clean, ":Ginfo") == 0) {
        command_reply(channel, "%d#", (int)g_os.state);
        return;
    }

    if (strcmp(clean, ":Q") == 0) {
        (void)os_goto_abort();
        (void)os_manual_stop(OS_AXIS_RA);
        (void)os_manual_stop(OS_AXIS_DEC);
        command_reply(channel, "1");
        return;
    }

    if (strcmp(clean, ":hP") == 0) {
        os_error_t err = os_park();
        command_reply(channel, "%d", (int)err);
        return;
    }

    if (strcmp(clean, ":hO") == 0) {
        os_error_t err = os_unpark();
        command_reply(channel, "%d", (int)err);
        return;
    }

    if (strcmp(clean, ":Me") == 0) {
        os_error_t err = os_manual_move(OS_AXIS_RA, true, 1.0);
        command_reply(channel, "%d", (int)err);
        return;
    }

    if (strcmp(clean, ":Mw") == 0) {
        os_error_t err = os_manual_move(OS_AXIS_RA, false, 1.0);
        command_reply(channel, "%d", (int)err);
        return;
    }

    if (strcmp(clean, ":Mn") == 0) {
        os_error_t err = os_manual_move(OS_AXIS_DEC, true, 1.0);
        command_reply(channel, "%d", (int)err);
        return;
    }

    if (strcmp(clean, ":Ms") == 0) {
        os_error_t err = os_manual_move(OS_AXIS_DEC, false, 1.0);
        command_reply(channel, "%d", (int)err);
        return;
    }

    if (strcmp(clean, ":MS") == 0) {
        os_error_t err = OS_ERR_INVALID_STATE;
        if (g_os.has_last_goto_target) {
            err = os_goto_equatorial(g_os.last_goto_ra_hours, g_os.last_goto_dec_degrees);
        }
        command_reply(channel, "%d", (int)err);
        return;
    }

    command_reply(channel, "0");
}

static void process_channel(int channel) {
    if (!is_valid_channel(channel)) {
        return;
    }

    while (os_hal_comm_available(channel) > 0) {
        int value = os_hal_comm_read(channel);
        if (value < 0) {
            break;
        }

        char byte = (char)value;

        if (!g_os.cmd_active[channel]) {
            if (byte == ':') {
                memset(g_os.cmd_buf[channel], 0, sizeof(g_os.cmd_buf[channel]));
                g_os.cmd_len[channel] = 0u;
                g_os.cmd_buf[channel][g_os.cmd_len[channel]++] = byte;
                g_os.cmd_active[channel] = true;
            }
            continue;
        }

        if (g_os.cmd_len[channel] >= OS_MAX_COMMAND_LENGTH) {
            g_os.cmd_active[channel] = false;
            command_reply(channel, "0");
            continue;
        }

        g_os.cmd_buf[channel][g_os.cmd_len[channel]++] = byte;

        if (byte == '#' || byte == '\n' || byte == '\r') {
            g_os.cmd_buf[channel][g_os.cmd_len[channel]] = '\0';
            command_execute(channel, g_os.cmd_buf[channel]);
            g_os.cmd_active[channel] = false;
        }
    }
}

/* -------------------------------------------------------------------------
 * Motion / loop helpers
 * ---------------------------------------------------------------------- */
static void stop_axis(int axis) {
    if (!is_valid_axis(axis)) {
        return;
    }

    g_os.busy[axis] = false;
    g_os.kind[axis] = AXIS_IDLE;
    g_os.target[axis] = 0.0;
    g_os.axis_freq[axis] = 0.0;
    g_os.manual_hz[axis] = 0.0;
    g_os.manual_active[axis] = false;
    (void)os_hal_motor_set_frequency(axis, 0.0);
}

static void stop_all_axes(void) {
    stop_axis(OS_AXIS_RA);
    stop_axis(OS_AXIS_DEC);
    g_os.park_in_progress = false;
    g_os.goto_active = false;
}

static void start_axis_motion(int axis, double target_steps, axis_kind_t kind) {
    if (!is_valid_axis(axis)) {
        return;
    }

    g_os.busy[axis] = true;
    g_os.kind[axis] = kind;
    g_os.target[axis] = target_steps;
    g_os.axis_freq[axis] = 0.0;
    (void)os_hal_motor_enable(axis, true);
}

static void update_motion(void) {
    for (int axis = 0; axis < 2; ++axis) {
        if (!g_os.busy[axis]) {
            continue;
        }

        double pos = (double)os_hal_motor_get_position(axis);
        double dist = g_os.target[axis] - pos;

        if (fabs(dist) <= 0.5) {
            (void)os_hal_motor_set_frequency(axis, 0.0);
            g_os.busy[axis] = false;
            g_os.kind[axis] = AXIS_IDLE;
            g_os.axis_freq[axis] = 0.0;
            if (g_os.kind[axis] == AXIS_MANUAL) {
                g_os.manual_active[axis] = false;
            }
            continue;
        }

        bool dir = dist > 0.0;
        (void)os_hal_motor_set_direction(axis, dir);
        (void)os_hal_motor_enable(axis, true);

        if (g_os.kind[axis] == AXIS_MANUAL) {
            g_os.axis_freq[axis] = g_os.manual_hz[axis];
        } else {
            double max_freq = (g_os.kind[axis] == AXIS_PARK) ? OS_MOTOR_PARK_HZ : OS_MOTOR_GOTO_HZ;
            double decel_distance = (g_os.axis_freq[axis] * g_os.axis_freq[axis]) / (2.0 * OS_MOTOR_ACCEL_HZ_PER_LOOP * 100.0);

            if (fabs(dist) > decel_distance) {
                g_os.axis_freq[axis] = fmin(max_freq, g_os.axis_freq[axis] + OS_MOTOR_ACCEL_HZ_PER_LOOP);
            } else {
                g_os.axis_freq[axis] = fmax(OS_MOTOR_MIN_HZ, g_os.axis_freq[axis] - OS_MOTOR_ACCEL_HZ_PER_LOOP);
            }
        }

        (void)os_hal_motor_set_frequency(axis, g_os.axis_freq[axis]);
    }

    if (!g_os.busy[0] && !g_os.busy[1]) {
        if (g_os.state == OS_STATE_GOTO && g_os.park_in_progress) {
            g_os.park_in_progress = false;
            g_os.state = OS_STATE_PARKED;
            (void)os_hal_motor_enable(OS_AXIS_RA, false);
            (void)os_hal_motor_enable(OS_AXIS_DEC, false);
        } else if (g_os.state == OS_STATE_GOTO && g_os.goto_active) {
            g_os.goto_active = false;
            g_os.state = OS_STATE_IDLE_TRACKING;
            (void)os_hal_buzzer_beep(100u, 1u);
        } else if (g_os.state == OS_STATE_MANUAL &&
                   !g_os.manual_active[OS_AXIS_RA] &&
                   !g_os.manual_active[OS_AXIS_DEC]) {
            g_os.state = OS_STATE_IDLE_TRACKING;
        }
    }
}

static void update_tracking(void) {
    if (g_os.state != OS_STATE_IDLE_TRACKING) {
        return;
    }

    if (!g_os.busy[OS_AXIS_RA]) {
        double freq = OS_DEFAULT_TRACK_FREQ_RA_HZ;
        if (g_os.guide_pulse_active && g_os.guide_axis == OS_AXIS_RA) {
            freq += g_os.guide_bias_hz[OS_AXIS_RA];
        }
        (void)os_hal_motor_enable(OS_AXIS_RA, true);
        (void)os_hal_motor_set_direction(OS_AXIS_RA, true);
        (void)os_hal_motor_set_frequency(OS_AXIS_RA, freq);
    }

    if (!g_os.busy[OS_AXIS_DEC]) {
        double freq = 0.0;
        if (g_os.guide_pulse_active && g_os.guide_axis == OS_AXIS_DEC) {
            freq += g_os.guide_bias_hz[OS_AXIS_DEC];
        }
        if (freq > 0.0) {
            (void)os_hal_motor_enable(OS_AXIS_DEC, true);
            (void)os_hal_motor_set_direction(OS_AXIS_DEC, true);
        }
        (void)os_hal_motor_set_frequency(OS_AXIS_DEC, freq);
    }
}

static void update_guide(void) {
    if (!g_os.guide_pulse_active) {
        return;
    }

    if (g_os.guide_remaining_ms > 0u) {
        --g_os.guide_remaining_ms;
        if (g_os.guide_remaining_ms == 0u) {
            g_os.guide_pulse_active = false;
        }
    }
}

static void update_time_site(void) {
    os_site_info_t site;
    memset(&site, 0, sizeof(site));

    if (os_hal_gps_poll(&site) == OS_ERR_NONE && site.valid) {
        g_os.site = site;
        g_os.gps_locked = true;
        (void)os_hal_rtc_set((uint32_t)site.utc_epoch_seconds);
        return;
    }

    uint32_t rtc = 0u;
    if (os_hal_rtc_read(&rtc) == OS_ERR_NONE) {
        g_os.site.utc_epoch_seconds = rtc;
    }
}

static void update_limits(void) {
    bool ra_triggered = os_hal_limit_is_triggered(OS_AXIS_RA);
    bool dec_triggered = os_hal_limit_is_triggered(OS_AXIS_DEC);

    if ((ra_triggered || dec_triggered) &&
        (g_os.state == OS_STATE_GOTO || g_os.state == OS_STATE_MANUAL || g_os.park_in_progress)) {
        stop_all_axes();
        g_os.limit_fault = true;
        g_os.state = OS_STATE_FAULT;
        (void)os_hal_buzzer_beep(200u, 3u);
    }
}

/* -------------------------------------------------------------------------
 * Public domain API
 * ---------------------------------------------------------------------- */
os_error_t os_init(void) {
    memset(&g_os, 0, sizeof(g_os));

    g_os.state = OS_STATE_INIT;
    g_os.align_mode = OS_ALIGN_3STAR;

    /* Explicit runtime-flag reset as required by REQ-FUNC-001 */
    g_os.goto_active = false;
    g_os.park_in_progress = false;
    g_os.manual_active[OS_AXIS_RA] = false;
    g_os.manual_active[OS_AXIS_DEC] = false;
    g_os.guide_pulse_active = false;
    g_os.limit_fault = false;
    g_os.align_active = false;
    g_os.align_star_count = 0u;
    g_os.align_residual_computed = false;
    g_os.align_residual_arcsec = 0.0;
    g_os.has_last_goto_target = false;

    calibration_identity(&g_os.cal);

    (void)os_hal_nvm_init();
    nvm_load_state();

    for (int ch = 0; ch < OS_MAX_CHANNELS; ++ch) {
        (void)os_hal_comm_init(ch);
    }

    (void)os_hal_motor_init(OS_AXIS_RA);
    (void)os_hal_motor_init(OS_AXIS_DEC);
    (void)os_hal_motor_enable(OS_AXIS_RA, false);
    (void)os_hal_motor_enable(OS_AXIS_DEC, false);

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    (void)os_hal_timer_motor_init();

    os_site_info_t site;
    memset(&site, 0, sizeof(site));

    if (os_hal_gps_poll(&site) == OS_ERR_NONE && site.valid) {
        g_os.site = site;
        g_os.gps_locked = true;
        (void)os_hal_rtc_set((uint32_t)site.utc_epoch_seconds);
    } else {
        uint32_t rtc = 0u;
        if (os_hal_rtc_read(&rtc) == OS_ERR_NONE) {
            g_os.site.utc_epoch_seconds = rtc;
        } else {
            g_os.site.utc_epoch_seconds = 1609459200u;
        }

        g_os.site.latitude_degrees = 0.0;
        g_os.site.longitude_degrees = 0.0;
        g_os.site.elevation_metres = 0.0;
        g_os.site.valid = false;
        g_os.gps_locked = false;
    }

    g_os.initialized = true;
    g_os.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    if (!g_os.initialized) {
        return;
    }

    for (int ch = 0; ch < OS_MAX_CHANNELS; ++ch) {
        process_channel(ch);
    }

    update_time_site();
    update_limits();

    if (g_os.state == OS_STATE_FAULT) {
        return;
    }

    update_guide();
    update_motion();
    update_tracking();
}

os_error_t os_goto_equatorial(double ra_hours, double dec_degrees) {
    if (!is_valid_ra(ra_hours) || !is_valid_dec(dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT;
    }

    double motor_ra = 0.0;
    double motor_dec = 0.0;
    apply_calibration(ra_hours, dec_degrees, &motor_ra, &motor_dec);

    start_axis_motion(OS_AXIS_RA, motor_ra, AXIS_GOTO);
    start_axis_motion(OS_AXIS_DEC, motor_dec, AXIS_GOTO);

    g_os.goto_active = true;
    g_os.park_in_progress = false;
    g_os.state = OS_STATE_GOTO;

    g_os.last_goto_ra_hours = ra_hours;
    g_os.last_goto_dec_degrees = dec_degrees;
    g_os.has_last_goto_target = true;

    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    stop_all_axes();
    g_os.goto_active = false;
    g_os.park_in_progress = false;

    if (g_os.state != OS_STATE_FAULT) {
        g_os.state = OS_STATE_IDLE_TRACKING;
    }

    return OS_ERR_NONE;
}

bool os_query_is_moving(void) {
    return g_os.busy[OS_AXIS_RA] || g_os.busy[OS_AXIS_DEC];
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_os.align_active = true;
    g_os.align_mode = mode;
    g_os.align_star_count = 0u;
    g_os.align_residual_computed = false;
    g_os.align_residual_arcsec = 0.0;
    g_os.state = OS_STATE_ALIGN;

    return OS_ERR_NONE;
}

os_error_t os_align_add_star(double ra_hours, double dec_degrees) {
    if (!is_valid_ra(ra_hours) || !is_valid_dec(dec_degrees)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!g_os.align_active) {
        return OS_ERR_INVALID_STATE;
    }

    if (g_os.align_star_count >= OS_MAX_ALIGN_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t idx = g_os.align_star_count;
    g_os.align_ra[idx] = ra_hours;
    g_os.align_dec[idx] = dec_degrees;
    g_os.align_motor0[idx] = (double)os_hal_motor_get_position(OS_AXIS_RA);
    g_os.align_motor1[idx] = (double)os_hal_motor_get_position(OS_AXIS_DEC);
    ++g_os.align_star_count;

    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (!g_os.align_active) {
        return OS_ERR_INVALID_STATE;
    }

    os_error_t err = compute_alignment();
    if (err != OS_ERR_NONE) {
        return err;
    }

    g_os.align_active = false;
    g_os.align_residual_computed = true;
    g_os.state = OS_STATE_IDLE_TRACKING;

    (void)nvm_save_calibration();
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_guide_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_GUIDE_EAST || direction >= OS_GUIDE_INVALID) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    int axis = OS_AXIS_RA;
    int priority = 1;
    double sign = 1.0;
    double bias_hz = OS_DEFAULT_TRACK_FREQ_RA_HZ * 0.5;

    switch (direction) {
        case OS_GUIDE_EAST:
            axis = OS_AXIS_RA;
            sign = 1.0;
            priority = 1;
            break;
        case OS_GUIDE_WEST:
            axis = OS_AXIS_RA;
            sign = -1.0;
            priority = 1;
            break;
        case OS_GUIDE_NORTH:
            axis = OS_AXIS_DEC;
            sign = 1.0;
            priority = 2;
            break;
        case OS_GUIDE_SOUTH:
            axis = OS_AXIS_DEC;
            sign = -1.0;
            priority = 2;
            break;
        default:
            return OS_ERR_INVALID_ARGUMENT;
    }

    /* DEC priority over RA when simultaneous signals arrive. */
    if (g_os.guide_pulse_active) {
        int old_priority = (g_os.guide_axis == OS_AXIS_DEC) ? 2 : 1;
        if (priority < old_priority) {
            return OS_ERR_INVALID_STATE;
        }
    }

    g_os.guide_axis = axis;
    g_os.guide_bias_hz[0] = 0.0;
    g_os.guide_bias_hz[1] = 0.0;
    g_os.guide_bias_hz[axis] = sign * bias_hz;
    g_os.guide_remaining_ms = duration_ms;
    g_os.guide_pulse_active = true;

    return OS_ERR_NONE;
}

os_error_t os_manual_move(int axis, bool forward, double rate_deg_per_sec) {
    if (!is_valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (rate_deg_per_sec <= 0.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT;
    }

    double freq = rate_deg_per_sec * OS_STEPS_PER_DEGREE;
    if (freq > OS_MOTOR_MAX_FREQ_HZ) {
        freq = OS_MOTOR_MAX_FREQ_HZ;
    }

    double pos = (double)os_hal_motor_get_position(axis);
    g_os.manual_active[axis] = true;
    g_os.manual_hz[axis] = freq;
    g_os.manual_dir[axis] = forward;
    g_os.kind[axis] = AXIS_MANUAL;
    g_os.target[axis] = pos + (forward ? 1e12 : -1e12);
    g_os.axis_freq[axis] = 0.0;
    g_os.busy[axis] = true;
    g_os.state = OS_STATE_MANUAL;

    return OS_ERR_NONE;
}

os_error_t os_manual_stop(int axis) {
    if (!is_valid_axis(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_os.manual_active[axis] = false;
    g_os.busy[axis] = false;
    g_os.kind[axis] = AXIS_IDLE;
    g_os.axis_freq[axis] = 0.0;
    g_os.manual_hz[axis] = 0.0;
    (void)os_hal_motor_set_frequency(axis, 0.0);

    if (!g_os.manual_active[OS_AXIS_RA] &&
        !g_os.manual_active[OS_AXIS_DEC] &&
        g_os.state == OS_STATE_MANUAL) {
        g_os.state = OS_STATE_IDLE_TRACKING;
    }

    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT;
    }

    double park_ra_steps = 0.0;
    double park_dec_steps = 90.0 * OS_STEPS_PER_DEGREE;

    start_axis_motion(OS_AXIS_RA, park_ra_steps, AXIS_PARK);
    start_axis_motion(OS_AXIS_DEC, park_dec_steps, AXIS_PARK);

    g_os.park_in_progress = true;
    g_os.state = OS_STATE_GOTO;
    g_os.goto_active = false;

    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (g_os.state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    for (int ch = 0; ch < OS_MAX_CHANNELS; ++ch) {
        (void)os_hal_comm_init(ch);
    }

    (void)os_hal_motor_enable(OS_AXIS_RA, true);
    (void)os_hal_motor_enable(OS_AXIS_DEC, true);
    (void)os_hal_motor_set_frequency(OS_AXIS_RA, 0.0);
    (void)os_hal_motor_set_frequency(OS_AXIS_DEC, 0.0);
    g_os.state = OS_STATE_IDLE_TRACKING;

    return OS_ERR_NONE;
}

os_state_t os_query_state(void) {
    return g_os.state;
}

os_error_t os_query_coordinates(double *ra_hours, double *dec_degrees) {
    if (ra_hours == NULL || dec_degrees == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    double m0 = (double)os_hal_motor_get_position(OS_AXIS_RA);
    double m1 = (double)os_hal_motor_get_position(OS_AXIS_DEC);
    motor_to_equatorial(m0, m1, ra_hours, dec_degrees);

    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *site = g_os.site;
    return OS_ERR_NONE;
}

os_error_t os_pec_set_table(const double *phase_deg, const double *error_arcsec, uint8_t count) {
    if (phase_deg == NULL || error_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (count > OS_PEC_MAX_POINTS) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    for (uint8_t i = 0u; i < count; ++i) {
        if (phase_deg[i] < 0.0 || phase_deg[i] > 360.0) {
            return OS_ERR_INVALID_ARGUMENT;
        }
    }

    for (uint8_t i = 0u; i < count; ++i) {
        g_os.pec_phase[i] = phase_deg[i];
        g_os.pec_error[i] = error_arcsec[i];
    }
    g_os.pec_count = count;

    /* Insertion sort by phase */
    for (uint8_t i = 1u; i < count; ++i) {
        double ph = g_os.pec_phase[i];
        double er = g_os.pec_error[i];
        int j = (int)i - 1;
        while (j >= 0 && g_os.pec_phase[j] > ph) {
            g_os.pec_phase[j + 1] = g_os.pec_phase[j];
            g_os.pec_error[j + 1] = g_os.pec_error[j];
            --j;
        }
        g_os.pec_phase[j + 1] = ph;
        g_os.pec_error[j + 1] = er;
    }

    return OS_ERR_NONE;
}

os_error_t os_pec_get_correction(double worm_phase_deg, double *correction_arcsec) {
    if (correction_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (worm_phase_deg < 0.0 || worm_phase_deg > 360.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (g_os.pec_count == 0u) {
        *correction_arcsec = 0.0;
        return OS_ERR_NONE;
    }

    if (g_os.pec_count == 1u) {
        *correction_arcsec = g_os.pec_error[0];
        return OS_ERR_NONE;
    }

    double p0 = g_os.pec_phase[0];
    double p1 = g_os.pec_phase[g_os.pec_count - 1u];
    double e0 = g_os.pec_error[0];
    double e1 = g_os.pec_error[g_os.pec_count - 1u];

    if (worm_phase_deg < p0) {
        double angle_span = p0 + (360.0 - p1);
        if (angle_span < 1e-12) {
            *correction_arcsec = e0;
        } else {
            double frac = (worm_phase_deg + (360.0 - p1)) / angle_span;
            *correction_arcsec = e1 + frac * (e0 - e1);
        }
        return OS_ERR_NONE;
    }

    if (worm_phase_deg >= p1) {
        double angle_span = (p0 + 360.0) - p1;
        if (angle_span < 1e-12) {
            *correction_arcsec = e1;
        } else {
            double frac = (worm_phase_deg - p1) / angle_span;
            *correction_arcsec = e1 + frac * (e0 - e1);
        }
        return OS_ERR_NONE;
    }

    for (uint8_t i = 0u; i < g_os.pec_count - 1u; ++i) {
        if (worm_phase_deg >= g_os.pec_phase[i] && worm_phase_deg <= g_os.pec_phase[i + 1u]) {
            double span = g_os.pec_phase[i + 1u] - g_os.pec_phase[i];
            if (span < 1e-12) {
                *correction_arcsec = g_os.pec_error[i];
            } else {
                double frac = (worm_phase_deg - g_os.pec_phase[i]) / span;
                *correction_arcsec = g_os.pec_error[i] +
                                     frac * (g_os.pec_error[i + 1u] - g_os.pec_error[i]);
            }
            return OS_ERR_NONE;
        }
    }

    *correction_arcsec = e1;
    return OS_ERR_NONE;
}

#include "generated_code.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Default domain configuration                                               */
/* ------------------------------------------------------------------------- */
#define NVM_SIGNATURE                        0x4F4E5354u
#define NVM_VERSION                          1u

#define DEFAULT_AXIS0_STEPS_PER_RA_HOUR      15000.0
#define DEFAULT_AXIS1_STEPS_PER_DEGREE       1000.0
#define DEFAULT_TRACKING_FREQUENCY_HZ        10.0
#define DEFAULT_GUIDE_RATE                   0.5
#define DEFAULT_MAX_GOTO_FREQUENCY_HZ        2000.0
#define DEFAULT_MANUAL_FREQUENCY_HZ          500.0
#define DEFAULT_LOOP_DT_SECONDS              0.01
#define DEFAULT_WORM_STEPS_PER_REV           100000.0

#define OS_ALIGN_MAX_STARS                   16
#define OS_COMM_RX_SIZE                      256
#define OS_COMM_TX_SIZE                      512

/* ------------------------------------------------------------------------- */
/* Types                                                                      */
/* ------------------------------------------------------------------------- */
typedef struct {
    uint32_t signature;
    uint32_t version;
    double tracking_rate_factor;
    double site_latitude_degrees;
    double site_longitude_degrees;
    double site_elevation_metres;
    uint32_t site_utc_epoch_seconds;
    uint8_t site_valid;
    double transform[6];
    uint8_t align_valid;
    uint8_t align_mode;
    uint8_t align_star_count;
    double align_residual_arcsec;
    double park_axis0_steps;
    double park_axis1_steps;
    uint8_t reserved[64];
} persistent_t;

typedef struct {
    double ra_hours;
    double dec_degrees;
    int32_t ra_steps;
    int32_t dec_steps;
} align_star_t;

typedef struct {
    bool initialized;
    bool enabled;
    bool direction_forward;
    double frequency_hz;
    int32_t position_steps;
    os_error_t fault;
} motor_state_t;

typedef struct {
    bool initialized;
    bool enabled;
    uint8_t rx[OS_COMM_RX_SIZE];
    size_t rx_head;
    size_t rx_tail;
    size_t rx_count;
    uint8_t tx[OS_COMM_TX_SIZE];
    size_t tx_head;
    size_t tx_tail;
    size_t tx_count;
    os_error_t fault;
} comm_channel_t;

/* ------------------------------------------------------------------------- */
/* HAL state                                                                  */
/* ------------------------------------------------------------------------- */
static motor_state_t s_motors[OS_MAX_AXES];
static comm_channel_t s_comm[OS_MAX_CHANNELS];

static os_site_info_t s_gps_site;
static bool s_gps_has_data = false;
static uint32_t s_rtc_value = 0;
static bool s_rtc_valid = false;
static bool s_rtc_fault = false;

static bool s_limit_triggered[OS_MAX_AXES] = { false, false };

static uint8_t s_nvm[OS_NVM_TOTAL_SIZE_BYTES];
static bool s_nvm_initialized = false;
static bool s_nvm_write_fault = false;
static bool s_nvm_read_fault = false;

#define MAX_BUZZER_EVENTS 16
static uint32_t s_buzzer_duration_ms[MAX_BUZZER_EVENTS];
static uint8_t s_buzzer_count[MAX_BUZZER_EVENTS];
static size_t s_buzzer_events = 0;

/* ------------------------------------------------------------------------- */
/* Domain state                                                               */
/* ------------------------------------------------------------------------- */
typedef struct {
    os_state_t state;

    double current_steps[OS_MAX_AXES];
    double target_steps[OS_MAX_AXES];
    double axis_frequency_hz[OS_MAX_AXES];
    bool axis_direction[OS_MAX_AXES];

    bool axis_manual_active[OS_MAX_AXES];
    double axis_manual_frequency_hz[OS_MAX_AXES];

    double tracking_rate_factor;

    bool target_ra_valid;
    double target_ra_hours;
    double target_dec_degrees;

    bool align_valid;
    uint8_t align_mode;
    uint8_t align_star_count;
    double align_matrix[6];
    double align_residual_arcsec;
    bool align_residual_computed;
    align_star_t align_stars[OS_ALIGN_MAX_STARS];

    os_site_info_t site;
    double worm_phase_deg;

    bool guide_active[OS_MAX_AXES];
    double guide_remaining_s[OS_MAX_AXES];
    double guide_frequency_offset_hz[OS_MAX_AXES];

    bool pec_valid;
    uint16_t pec_count;
    os_pec_point_t pec_table[OS_PEC_MAX_POINTS];
} runtime_t;

static runtime_t s_runtime;
static persistent_t s_persistent;

/* ------------------------------------------------------------------------- */
/* Small helpers                                                              */
/* ------------------------------------------------------------------------- */
static bool in_range(double v, double lo, double hi) {
    return isfinite(v) && (v >= lo) && (v <= hi);
}

static double clamp_frequency(double hz) {
    if (!isfinite(hz) || hz < 0.0) return 0.0;
    if (hz > 100000.0) return 100000.0;
    return hz;
}

static int32_t double_to_steps(double v) {
    if (v >= 0.0) return (int32_t)(v + 0.5);
    return (int32_t)(v - 0.5);
}

static void hal_update_motor_position(uint8_t axis, double position) {
    if (axis < OS_MAX_AXES) {
        s_motors[axis].position_steps = double_to_steps(position);
    }
}

static void domain_default_matrix(double m[6]) {
    m[0] = 1.0; m[1] = 0.0; m[2] = 0.0;
    m[3] = 0.0; m[4] = 1.0; m[5] = 0.0;
}

/* ------------------------------------------------------------------------- */
/* HAL implementation                                                         */
/* ------------------------------------------------------------------------- */
os_error_t os_hal_motor_init(uint8_t axis) {
    if (axis >= OS_MAX_AXES) return OS_ERR_INVALID_ARGUMENT;
    s_motors[axis].initialized = true;
    s_motors[axis].enabled = false;
    s_motors[axis].direction_forward = true;
    s_motors[axis].frequency_hz = 0.0;
    s_motors[axis].position_steps = 0;
    s_motors[axis].fault = OS_ERR_NONE;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, double frequency_hz) {
    if (axis >= OS_MAX_AXES) return OS_ERR_INVALID_ARGUMENT;
    if (!isfinite(frequency_hz) || frequency_hz < 0.0) return OS_ERR_INVALID_ARGUMENT;
    s_motors[axis].frequency_hz = clamp_frequency(frequency_hz);
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (axis >= OS_MAX_AXES) return OS_ERR_INVALID_ARGUMENT;
    s_motors[axis].direction_forward = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (axis >= OS_MAX_AXES) return OS_ERR_INVALID_ARGUMENT;
    s_motors[axis].enabled = enable;
    if (!enable) s_motors[axis].frequency_hz = 0.0;
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (axis >= OS_MAX_AXES) return 0;
    return s_motors[axis].position_steps;
}

os_error_t os_hal_timer_motor_init(void) {
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_init(void) {
    s_gps_site.valid = false;
    s_gps_has_data = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (!s_gps_has_data || !s_gps_site.valid) {
        site->valid = false;
        return OS_ERR_TIMEOUT;
    }
    *site = s_gps_site;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void) {
    s_rtc_valid = true;
    s_rtc_value = 0;
    s_rtc_fault = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (s_rtc_fault) return OS_ERR_DEVICE_FAULT;
    if (!s_rtc_valid) return OS_ERR_INVALID_STATE;
    *utc_epoch_seconds = s_rtc_value;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    s_rtc_value = utc_epoch_seconds;
    s_rtc_valid = true;
    s_rtc_fault = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    s_limit_triggered[OS_AXIS_RA] = false;
    s_limit_triggered[OS_AXIS_DEC] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (axis >= OS_MAX_AXES) return true;
    return s_limit_triggered[axis];
}

void os_hal_buzzer_beep(uint32_t duration_ms, uint8_t count) {
    if (s_buzzer_events >= MAX_BUZZER_EVENTS) return;
    s_buzzer_duration_ms[s_buzzer_events] = duration_ms;
    s_buzzer_count[s_buzzer_events] = count;
    ++s_buzzer_events;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (channel >= OS_MAX_CHANNELS) return OS_ERR_INVALID_ARGUMENT;
    memset(&s_comm[channel], 0, sizeof(s_comm[channel]));
    s_comm[channel].initialized = true;
    s_comm[channel].enabled = true;
    s_comm[channel].fault = OS_ERR_NONE;
    return OS_ERR_NONE;
}

size_t os_hal_comm_available(uint8_t channel) {
    if (channel >= OS_MAX_CHANNELS || !s_comm[channel].initialized) return 0;
    return s_comm[channel].rx_count;
}

int32_t os_hal_comm_read(uint8_t channel) {
    if (channel >= OS_MAX_CHANNELS || !s_comm[channel].initialized) return -1;
    if (s_comm[channel].rx_count == 0) return -1;
    uint8_t byte = s_comm[channel].rx[s_comm[channel].rx_tail];
    s_comm[channel].rx_tail = (s_comm[channel].rx_tail + 1) % OS_COMM_RX_SIZE;
    --s_comm[channel].rx_count;
    return (int32_t)byte;
}

os_error_t os_hal_comm_write(uint8_t channel, const uint8_t *data, size_t length) {
    if (channel >= OS_MAX_CHANNELS) return OS_ERR_INVALID_ARGUMENT;
    if (!s_comm[channel].initialized) return OS_ERR_INVALID_STATE;
    if (data == NULL && length > 0) return OS_ERR_INVALID_ARGUMENT;
    if (length == 0) return OS_ERR_NONE;
    if (length > (OS_COMM_TX_SIZE - s_comm[channel].tx_count)) return OS_ERR_TIMEOUT;

    for (size_t i = 0; i < length; ++i) {
        s_comm[channel].tx[s_comm[channel].tx_tail] = data[i];
        s_comm[channel].tx_tail = (s_comm[channel].tx_tail + 1) % OS_COMM_TX_SIZE;
        ++s_comm[channel].tx_count;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_init(void) {
    if (!s_nvm_initialized) {
        memset(s_nvm, 0, sizeof(s_nvm));
        s_nvm_initialized = true;
    }
    s_nvm_read_fault = false;
    s_nvm_write_fault = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint32_t offset, uint8_t *data, size_t length) {
    if (data == NULL && length > 0) return OS_ERR_INVALID_ARGUMENT;
    if (!s_nvm_initialized) return OS_ERR_INVALID_STATE;
    if (s_nvm_read_fault) return OS_ERR_DEVICE_FAULT;
    if ((size_t)offset + length > OS_NVM_TOTAL_SIZE_BYTES) return OS_ERR_INVALID_ARGUMENT;
    memcpy(data, s_nvm + offset, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint32_t offset, const uint8_t *data, size_t length) {
    if (data == NULL && length > 0) return OS_ERR_INVALID_ARGUMENT;
    if (!s_nvm_initialized) return OS_ERR_INVALID_STATE;
    if (s_nvm_write_fault) return OS_ERR_DEVICE_FAULT;
    if ((size_t)offset + length > OS_NVM_TOTAL_SIZE_BYTES) return OS_ERR_INVALID_ARGUMENT;
    memcpy(s_nvm + offset, data, length);
    return OS_ERR_NONE;
}

/* ------------------------------------------------------------------------- */
/* Host injection helpers                                                     */
/* ------------------------------------------------------------------------- */
void os_hal_test_comm_inject(uint8_t channel, const uint8_t *data, size_t length) {
    if (channel >= OS_MAX_CHANNELS || data == NULL) return;
    for (size_t i = 0; i < length; ++i) {
        if (s_comm[channel].rx_count >= OS_COMM_RX_SIZE) return;
        s_comm[channel].rx[s_comm[channel].rx_tail] = data[i];
        s_comm[channel].rx_tail = (s_comm[channel].rx_tail + 1) % OS_COMM_RX_SIZE;
        ++s_comm[channel].rx_count;
    }
}

void os_hal_test_comm_clear(uint8_t channel) {
    if (channel >= OS_MAX_CHANNELS) return;
    s_comm[channel].rx_head = s_comm[channel].rx_tail = 0;
    s_comm[channel].rx_count = 0;
    s_comm[channel].tx_head = s_comm[channel].tx_tail = 0;
    s_comm[channel].tx_count = 0;
}

void os_hal_test_gps_inject(const os_site_info_t *site) {
    if (site == NULL) return;
    s_gps_site = *site;
    s_gps_site.valid = true;
    s_gps_has_data = true;
}

void os_hal_test_rtc_inject(uint32_t utc_epoch_seconds) {
    s_rtc_value = utc_epoch_seconds;
    s_rtc_valid = true;
    s_rtc_fault = false;
}

void os_hal_test_limit_inject(uint8_t axis, bool triggered) {
    if (axis < OS_MAX_AXES) s_limit_triggered[axis] = triggered;
}

void os_hal_test_motor_inject_position(uint8_t axis, int32_t position) {
    if (axis < OS_MAX_AXES) s_motors[axis].position_steps = position;
}

void os_hal_test_nvm_clear(void) {
    memset(s_nvm, 0, sizeof(s_nvm));
    s_nvm_initialized = true;
}

void os_hal_test_nvm_inject_write_fault(bool fault) {
    s_nvm_write_fault = fault;
}

void os_hal_test_nvm_inject_read_fault(bool fault) {
    s_nvm_read_fault = fault;
}

void os_hal_test_buzzer_clear(void) {
    s_buzzer_events = 0;
}

size_t os_hal_test_buzzer_event_count(void) {
    return s_buzzer_events;
}

/* ------------------------------------------------------------------------- */
/* NVM persistence                                                            */
/* ------------------------------------------------------------------------- */
static void persistent_defaults(persistent_t *p) {
    memset(p, 0, sizeof(*p));
    p->signature = NVM_SIGNATURE;
    p->version = NVM_VERSION;
    p->tracking_rate_factor = 1.0;
    p->transform[0] = 1.0;
    p->transform[4] = 1.0;
    p->park_axis0_steps = 0.0;
    p->park_axis1_steps = 0.0;
}

static void apply_persistent_to_runtime(void) {
    s_runtime.tracking_rate_factor = s_persistent.tracking_rate_factor;

    s_runtime.align_valid = (s_persistent.align_valid != 0);
    s_runtime.align_mode = s_persistent.align_mode;
    s_runtime.align_star_count = s_persistent.align_star_count;
    s_runtime.align_residual_arcsec = s_persistent.align_residual_arcsec;
    memcpy(s_runtime.align_matrix, s_persistent.transform, sizeof(s_runtime.align_matrix));

    s_runtime.site.valid = (s_persistent.site_valid != 0);
    s_runtime.site.latitude_degrees = s_persistent.site_latitude_degrees;
    s_runtime.site.longitude_degrees = s_persistent.site_longitude_degrees;
    s_runtime.site.elevation_metres = s_persistent.site_elevation_metres;
    s_runtime.site.utc_epoch_seconds = s_persistent.site_utc_epoch_seconds;
}

static void write_runtime_to_persistent(void) {
    s_persistent.tracking_rate_factor = s_runtime.tracking_rate_factor;

    s_persistent.site_valid = (uint8_t)(s_runtime.site.valid ? 1 : 0);
    s_persistent.site_latitude_degrees = s_runtime.site.latitude_degrees;
    s_persistent.site_longitude_degrees = s_runtime.site.longitude_degrees;
    s_persistent.site_elevation_metres = s_runtime.site.elevation_metres;
    s_persistent.site_utc_epoch_seconds = s_runtime.site.utc_epoch_seconds;

    s_persistent.align_valid = (uint8_t)(s_runtime.align_valid ? 1 : 0);
    s_persistent.align_mode = s_runtime.align_mode;
    s_persistent.align_star_count = s_runtime.align_star_count;
    s_persistent.align_residual_arcsec = s_runtime.align_residual_arcsec;
    memcpy(s_persistent.transform, s_runtime.align_matrix, sizeof(s_persistent.transform));
}

static os_error_t nvm_load(void) {
    os_error_t err = os_hal_nvm_read(0, (uint8_t *)&s_persistent, sizeof(s_persistent));
    if (err != OS_ERR_NONE) return err;
    if (s_persistent.signature != NVM_SIGNATURE || s_persistent.version != NVM_VERSION) {
        persistent_defaults(&s_persistent);
        (void)os_hal_nvm_write(0, (const uint8_t *)&s_persistent, sizeof(s_persistent));
        return OS_ERR_NONE;
    }
    return OS_ERR_NONE;
}

static void nvm_save(void) {
    write_runtime_to_persistent();
    s_persistent.signature = NVM_SIGNATURE;
    s_persistent.version = NVM_VERSION;
    (void)os_hal_nvm_write(0, (const uint8_t *)&s_persistent, sizeof(s_persistent));
}

/* ------------------------------------------------------------------------- */
/* Coordinate transforms                                                      */
/* ------------------------------------------------------------------------- */
static void motor_steps_from_celestial(double ra_hours, double dec_degrees,
                                       double *sx, double *sy) {
    double x = ra_hours * DEFAULT_AXIS0_STEPS_PER_RA_HOUR;
    double y = dec_degrees * DEFAULT_AXIS1_STEPS_PER_DEGREE;
    const double *m = s_runtime.align_matrix;
    if (!s_runtime.align_valid) {
        *sx = x;
        *sy = y;
        return;
    }
    *sx = m[0] * x + m[1] * y + m[2];
    *sy = m[3] * x + m[4] * y + m[5];
}

static void celestial_from_motor_steps(double sx, double sy,
                                      double *ra_hours, double *dec_degrees) {
    const double *m = s_runtime.align_matrix;
    double a = m[0], b = m[1], c = m[2];
    double d = m[3], e = m[4], f = m[5];
    double nx = sx - c;
    double ny = sy - f;
    double det = a * e - b * d;

    double x, y;
    if (fabs(det) < 1e-12) {
        x = nx;
        y = ny;
    } else {
        x = (e * nx - b * ny) / det;
        y = (-d * nx + a * ny) / det;
    }
    *ra_hours = x / DEFAULT_AXIS0_STEPS_PER_RA_HOUR;
    *dec_degrees = y / DEFAULT_AXIS1_STEPS_PER_DEGREE;
}

static void normalize_ra_hours(double *ra) {
    if (!isfinite(*ra)) *ra = 0.0;
    while (*ra < 0.0) *ra += 24.0;
    while (*ra >= 24.0) *ra -= 24.0;
}

/* ------------------------------------------------------------------------- */
/* Alignment solver                                                           */
/* ------------------------------------------------------------------------- */
static int qr_solve(const double *A, const double *b, int n, int m, double *x) {
    double Q[32 * 6];
    double R[6][6];
    double v[32];
    double z[6];
    const double eps = 1e-12;

    memset(Q, 0, sizeof(Q));
    memset(R, 0, sizeof(R));
    memset(z, 0, sizeof(z));
    memset(x, 0, (size_t)m * sizeof(double));

    if (n > 32 || m > 6 || n < m) return 0;

    for (int j = 0; j < m; ++j) {
        for (int i = 0; i < n; ++i) v[i] = A[i * m + j];
        for (int k = 0; k < j; ++k) {
            double dot = 0.0;
            for (int i = 0; i < n; ++i) dot += Q[i * m + k] * v[i];
            R[k][j] = dot;
            for (int i = 0; i < n; ++i) v[i] -= dot * Q[i * m + k];
        }
        double norm = 0.0;
        for (int i = 0; i < n; ++i) norm += v[i] * v[i];
        norm = sqrt(norm);
        if (norm < eps) return 0;
        R[j][j] = norm;
        for (int i = 0; i < n; ++i) Q[i * m + j] = v[i] / norm;
    }

    for (int k = 0; k < m; ++k) {
        double dot = 0.0;
        for (int i = 0; i < n; ++i) dot += Q[i * m + k] * b[i];
        z[k] = dot;
    }

    for (int i = m - 1; i >= 0; --i) {
        double sum = z[i];
        for (int j = i + 1; j < m; ++j) sum -= R[i][j] * x[j];
        if (fabs(R[i][i]) < eps) return 0;
        x[i] = sum / R[i][i];
    }
    return 1;
}

/* ------------------------------------------------------------------------- */
/* Tracking/motion helpers                                                    */
/* ------------------------------------------------------------------------- */
static void motor_apply_axis(uint8_t axis, double frequency_hz, bool forward) {
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, frequency_hz);
    if (frequency_hz > 0.0) {
        (void)os_hal_motor_enable(axis, true);
    }
    s_runtime.axis_frequency_hz[axis] = frequency_hz;
    s_runtime.axis_direction[axis] = forward;
}

static void motor_stop_axis(uint8_t axis) {
    (void)os_hal_motor_set_frequency(axis, 0.0);
    s_runtime.axis_frequency_hz[axis] = 0.0;
}

static void apply_tracking_frequencies(void) {
    if (s_runtime.state == OS_STATE_PARKED || s_runtime.state == OS_STATE_FAULT) {
        motor_stop_axis(OS_AXIS_RA);
        motor_stop_axis(OS_AXIS_DEC);
        return;
    }

    double ra_freq = DEFAULT_TRACKING_FREQUENCY_HZ * s_runtime.tracking_rate_factor +
                     s_runtime.guide_frequency_offset_hz[OS_AXIS_RA];
    if (ra_freq < 0.0) ra_freq = 0.0;

    double dec_freq = s_runtime.guide_frequency_offset_hz[OS_AXIS_DEC];
    if (dec_freq < 0.0) dec_freq = 0.0;

    motor_apply_axis(OS_AXIS_RA, ra_freq, true);
    motor_apply_axis(OS_AXIS_DEC, dec_freq, true);
}

static void advance_axis_by_frequency(uint8_t axis, bool forward) {
    double freq = s_runtime.axis_frequency_hz[axis];
    if (freq <= 0.0) return;
    double step = freq * DEFAULT_LOOP_DT_SECONDS;
    if (!forward) step = -step;
    s_runtime.current_steps[axis] += step;
    hal_update_motor_position(axis, s_runtime.current_steps[axis]);
}

static void advance_tracking(void) {
    for (uint8_t axis = 0; axis < OS_MAX_AXES; ++axis) {
        if (s_runtime.guide_active[axis]) {
            s_runtime.guide_remaining_s[axis] -= DEFAULT_LOOP_DT_SECONDS;
            if (s_runtime.guide_remaining_s[axis] <= 0.0) {
                s_runtime.guide_active[axis] = false;
                s_runtime.guide_remaining_s[axis] = 0.0;
                s_runtime.guide_frequency_offset_hz[axis] = 0.0;
            }
        }
    }

    /* Add PEC correction to RA while idle tracking if a table is loaded. */
    if (s_runtime.pec_valid && s_runtime.pec_count > 0) {
        double corr = 0.0;
        (void)os_pec_apply(s_runtime.worm_phase_deg, &corr);
        s_runtime.guide_frequency_offset_hz[OS_AXIS_RA] += corr;
    }

    apply_tracking_frequencies();
    advance_axis_by_frequency(OS_AXIS_RA, true);
    advance_axis_by_frequency(OS_AXIS_DEC, true);

    s_runtime.worm_phase_deg =
        fmod(fabs(s_runtime.current_steps[OS_AXIS_RA]) * 360.0 / DEFAULT_WORM_STEPS_PER_REV,
             360.0);
}

static void advance_goto(void) {
    bool done = true;

    for (uint8_t axis = 0; axis < OS_MAX_AXES; ++axis) {
        double delta = s_runtime.target_steps[axis] - s_runtime.current_steps[axis];
        if (fabs(delta) < 0.5) {
            s_runtime.current_steps[axis] = s_runtime.target_steps[axis];
            hal_update_motor_position(axis, s_runtime.current_steps[axis]);
            motor_stop_axis(axis);
            continue;
        }

        done = false;
        if (os_hal_limit_is_triggered(axis)) {
            motor_stop_axis(axis);
            s_runtime.state = OS_STATE_FAULT;
            os_hal_buzzer_beep(200, 2);
            return;
        }

        bool forward = (delta > 0.0);
        double freq = DEFAULT_MAX_GOTO_FREQUENCY_HZ;
        double step = freq * DEFAULT_LOOP_DT_SECONDS;
        if (!forward) step = -step;

        if (fabs(step) >= fabs(delta)) {
            s_runtime.current_steps[axis] = s_runtime.target_steps[axis];
            hal_update_motor_position(axis, s_runtime.current_steps[axis]);
            motor_stop_axis(axis);
        } else {
            s_runtime.current_steps[axis] += step;
            hal_update_motor_position(axis, s_runtime.current_steps[axis]);
            motor_apply_axis(axis, freq, forward);
        }
    }

    if (!done) return;

    if (s_runtime.state == OS_STATE_GOTO) {
        s_runtime.state = OS_STATE_IDLE_TRACKING;
        os_hal_buzzer_beep(100, 2);
    } else if (s_runtime.state == OS_STATE_PARKING) {
        s_runtime.state = OS_STATE_PARKED;
        (void)os_hal_motor_enable(OS_AXIS_RA, false);
        (void)os_hal_motor_enable(OS_AXIS_DEC, false);
        motor_stop_axis(OS_AXIS_RA);
        motor_stop_axis(OS_AXIS_DEC);
        os_hal_buzzer_beep(200, 1);
    }
}

static void advance_manual(void) {
    for (uint8_t axis = 0; axis < OS_MAX_AXES; ++axis) {
        if (!s_runtime.axis_manual_active[axis]) continue;
        if (os_hal_limit_is_triggered(axis)) {
            motor_stop_axis(axis);
            s_runtime.axis_manual_active[axis] = false;
            s_runtime.state = OS_STATE_FAULT;
            os_hal_buzzer_beep(200, 2);
            return;
        }
        motor_apply_axis(axis, s_runtime.axis_manual_frequency_hz[axis],
                         s_runtime.axis_direction[axis]);
        advance_axis_by_frequency(axis, s_runtime.axis_direction[axis]);
    }

    bool any_manual = false;
    for (uint8_t axis = 0; axis < OS_MAX_AXES; ++axis) {
        if (s_runtime.axis_manual_active[axis]) any_manual = true;
    }
    if (!any_manual && s_runtime.state == OS_STATE_MANUAL) {
        s_runtime.state = OS_STATE_IDLE_TRACKING;
        apply_tracking_frequencies();
    }
}

/* ------------------------------------------------------------------------- */
/* Command processing                                                         */
/* ------------------------------------------------------------------------- */
static void format_ra_reply(double ra, char *out, size_t out_size) {
    normalize_ra_hours(&ra);
    int h = (int)ra;
    double m = (ra - h) * 60.0;
    int mi = (int)m;
    int sec = (int)((m - mi) * 60.0 + 0.5);
    if (sec >= 60) { sec = 0; ++mi; }
    if (mi >= 60) { mi = 0; ++h; }
    if (h >= 24) h = 0;
    (void)snprintf(out, out_size, "%02d:%02d:%02d#", h, mi, sec);
}

static void format_dec_reply(double dec, char *out, size_t out_size) {
    char sign = '+';
    if (dec < 0.0) { sign = '-'; dec = -dec; }
    int d = (int)dec;
    double m = (dec - d) * 60.0;
    int mi = (int)m;
    int sec = (int)((m - mi) * 60.0 + 0.5);
    if (sec >= 60) { sec = 0; ++mi; }
    if (mi >= 60) { mi = 0; ++d; }
    (void)snprintf(out, out_size, "%c%02d:%02d:%02d#", sign, d, mi, sec);
}

static double parse_hms(const char *s) {
    int h = 0, m = 0, sec = 0;
    if (sscanf(s, "%d:%d:%d", &h, &m, &sec) < 2) return -1.0;
    if (h < 0 || h > 24 || m < 0 || m >= 60 || sec < 0 || sec >= 60) return -1.0;
    return (double)h + (double)m / 60.0 + (double)sec / 3600.0;
}

static double parse_dms(const char *s) {
    int d = 0, m = 0, sec = 0;
    char sign = '+';
    if (sscanf(s, "%c%d:%d:%d", &sign, &d, &m, &sec) < 2) return -999.0;
    if (d < 0 || d > 90 || m < 0 || m >= 60 || sec < 0 || sec >= 60) return -999.0;
    double v = (double)d + (double)m / 60.0 + (double)sec / 3600.0;
    return (sign == '-') ? -v : v;
}

static void handle_lx200_command(uint8_t channel, const char *cmd) {
    char reply[OS_MAX_REPLY_LENGTH];
    memset(reply, 0, sizeof(reply));

    if (strcmp(cmd, ":GR#") == 0) {
        double ra, dec;
        celestial_from_motor_steps(s_runtime.current_steps[OS_AXIS_RA],
                                  s_runtime.current_steps[OS_AXIS_DEC],
                                  &ra, &dec);
        format_ra_reply(ra, reply, sizeof(reply));
    } else if (strcmp(cmd, ":GD#") == 0) {
        double ra, dec;
        celestial_from_motor_steps(s_runtime.current_steps[OS_AXIS_RA],
                                  s_runtime.current_steps[OS_AXIS_DEC],
                                  &ra, &dec);
        format_dec_reply(dec, reply, sizeof(reply));
    } else if (strcmp(cmd, ":GVP#") == 0) {
        (void)snprintf(reply, sizeof(reply), "OnStep 1.0#");
    } else if (strcmp(cmd, ":MS#") == 0) {
        os_error_t err = OS_ERR_INVALID_STATE;
        if (s_runtime.target_ra_valid) {
            err = os_goto_equatorial(s_runtime.target_ra_hours,
                                     s_runtime.target_dec_degrees);
        }
        (void)snprintf(reply, sizeof(reply), "%d", (err == OS_ERR_NONE) ? 1 : 0);
    } else if (strncmp(cmd, ":Sr", 3) == 0) {
        char tmp[16] = {0};
        size_t len = strlen(cmd);
        if (len >= 9 && cmd[len-1] == '#') {
            size_t num = len - 4; /* remove :Sr and # */
            if (num < sizeof(tmp)) {
                memcpy(tmp, cmd + 3, num);
                double ra = parse_hms(tmp);
                if (ra >= 0.0 && ra < 24.0) {
                    s_runtime.target_ra_valid = true;
                    s_runtime.target_ra_hours = ra;
                    (void)snprintf(reply, sizeof(reply), "1");
                } else {
                    (void)snprintf(reply, sizeof(reply), "0");
                }
            }
        }
    } else if (strncmp(cmd, ":Sd", 3) == 0) {
        char tmp[16] = {0};
        size_t len = strlen(cmd);
        if (len >= 7 && cmd[len-1] == '#') {
            size_t num = len - 4;
            if (num < sizeof(tmp)) {
                memcpy(tmp, cmd + 3, num);
                double dec = parse_dms(tmp);
                if (dec >= -90.0 && dec <= 90.0) {
                    s_runtime.target_dec_degrees = dec;
                    (void)snprintf(reply, sizeof(reply), "1");
                } else {
                    (void)snprintf(reply, sizeof(reply), "0");
                }
            }
        }
    } else if (strcmp(cmd, ":Me#") == 0) {
        os_error_t err = os_manual_move(OS_AXIS_RA, true, DEFAULT_MANUAL_FREQUENCY_HZ);
        (void)snprintf(reply, sizeof(reply), "%d", (err == OS_ERR_NONE) ? 1 : 0);
    } else if (strcmp(cmd, ":Mw#") == 0) {
        os_error_t err = os_manual_move(OS_AXIS_RA, false, DEFAULT_MANUAL_FREQUENCY_HZ);
        (void)snprintf(reply, sizeof(reply), "%d", (err == OS_ERR_NONE) ? 1 : 0);
    } else if (strcmp(cmd, ":Mn#") == 0) {
        os_error_t err = os_manual_move(OS_AXIS_DEC, true, DEFAULT_MANUAL_FREQUENCY_HZ);
        (void)snprintf(reply, sizeof(reply), "%d", (err == OS_ERR_NONE) ? 1 : 0);
    } else if (strcmp(cmd, ":Ms#") == 0) {
        os_error_t err = os_manual_move(OS_AXIS_DEC, false, DEFAULT_MANUAL_FREQUENCY_HZ);
        (void)snprintf(reply, sizeof(reply), "%d", (err == OS_ERR_NONE) ? 1 : 0);
    } else if (strcmp(cmd, ":Qe#") == 0 || strcmp(cmd, ":Qw#") == 0) {
        os_error_t err = os_manual_stop(OS_AXIS_RA);
        (void)snprintf(reply, sizeof(reply), "%d", (err == OS_ERR_NONE) ? 1 : 0);
    } else if (strcmp(cmd, ":Qn#") == 0 || strcmp(cmd, ":Qs#") == 0) {
        os_error_t err = os_manual_stop(OS_AXIS_DEC);
        (void)snprintf(reply, sizeof(reply), "%d", (err == OS_ERR_NONE) ? 1 : 0);
    } else if (strcmp(cmd, ":hP#") == 0) {
        os_error_t err = os_park();
        (void)snprintf(reply, sizeof(reply), "%d", (err == OS_ERR_NONE) ? 1 : 0);
    } else if (strcmp(cmd, ":hO#") == 0) {
        os_error_t err = os_unpark();
        (void)snprintf(reply, sizeof(reply), "%d", (err == OS_ERR_NONE) ? 1 : 0);
    } else {
        (void)snprintf(reply, sizeof(reply), "0");
    }

    if (reply[0] != '\0') {
        (void)os_hal_comm_write(channel, (const uint8_t *)reply, strlen(reply));
    }
}

static void poll_commands(void) {
    char cmd[OS_MAX_COMMAND_LENGTH];
    size_t cmd_len = 0;
    bool in_cmd = false;

    for (uint8_t channel = 0; channel < OS_MAX_CHANNELS; ++channel) {
        cmd_len = 0;
        in_cmd = false;
        while (os_hal_comm_available(channel) > 0) {
            int32_t byte = os_hal_comm_read(channel);
            if (byte < 0) break;

            char c = (char)byte;
            if (!in_cmd) {
                if (c == ':') {
                    in_cmd = true;
                    cmd_len = 0;
                    cmd[cmd_len++] = c;
                }
                continue;
            }

            if (cmd_len < (sizeof(cmd) - 1)) {
                cmd[cmd_len++] = c;
            }

            if (c == '#') {
                cmd[cmd_len] = '\0';
                handle_lx200_command(channel, cmd);
                in_cmd = false;
                cmd_len = 0;
            } else if (cmd_len >= (sizeof(cmd) - 1)) {
                in_cmd = false;
                cmd_len = 0;
            }
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Public domain API                                                          */
/* ------------------------------------------------------------------------- */
os_error_t os_init(void) {
    /* Reset every runtime module flag explicitly. */
    memset(&s_runtime, 0, sizeof(s_runtime));
    domain_default_matrix(s_runtime.align_matrix);
    s_runtime.state = OS_STATE_INIT;
    s_runtime.tracking_rate_factor = 1.0;
    s_runtime.align_residual_computed = false;
    s_runtime.align_star_count = 0;
    s_runtime.target_ra_valid = false;
    s_runtime.pec_valid = false;
    s_runtime.pec_count = 0;

    memset(&s_persistent, 0, sizeof(s_persistent));
    persistent_defaults(&s_persistent);

    (void)os_hal_nvm_init();
    (void)nvm_load();
    apply_persistent_to_runtime();

    for (uint8_t ch = 0; ch < OS_MAX_CHANNELS; ++ch) {
        (void)os_hal_comm_init(ch);
    }

    for (uint8_t axis = 0; axis < OS_MAX_AXES; ++axis) {
        (void)os_hal_motor_init(axis);
        s_runtime.current_steps[axis] = 0.0;
        s_runtime.target_steps[axis] = 0.0;
        s_runtime.axis_frequency_hz[axis] = 0.0;
        s_runtime.axis_direction[axis] = true;
        s_runtime.axis_manual_active[axis] = false;
        s_runtime.guide_active[axis] = false;
        s_runtime.guide_remaining_s[axis] = 0.0;
        s_runtime.guide_frequency_offset_hz[axis] = 0.0;
    }

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    (void)os_hal_timer_motor_init();

    bool have_time = false;
    os_site_info_t gps_site;
    os_error_t gps_err = os_hal_gps_poll(&gps_site);
    if (gps_err == OS_ERR_NONE && gps_site.valid) {
        s_runtime.site = gps_site;
        (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
        have_time = true;
    } else {
        uint32_t rtc_time = 0;
        os_error_t rtc_err = os_hal_rtc_read(&rtc_time);
        if (rtc_err == OS_ERR_NONE) {
            s_runtime.site.utc_epoch_seconds = rtc_time;
            have_time = true;
        }
    }

    if (have_time) {
        s_runtime.state = OS_STATE_IDLE_TRACKING;
    } else {
        s_runtime.state = OS_STATE_FAULT;
    }

    (void)os_hal_buzzer_beep(50, 1);
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    if (s_runtime.state == OS_STATE_INIT) return;

    /* Non-blocking GPS update. */
    os_site_info_t gps_site;
    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid) {
        s_runtime.site = gps_site;
        (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
    } else if (s_runtime.site.valid) {
        uint32_t rtc_time = 0;
        if (os_hal_rtc_read(&rtc_time) == OS_ERR_NONE) {
            s_runtime.site.utc_epoch_seconds = rtc_time;
        }
    }

    for (uint8_t axis = 0; axis < OS_MAX_AXES; ++axis) {
        if (os_hal_limit_is_triggered(axis)) {
            if (s_runtime.state == OS_STATE_GOTO || s_runtime.state == OS_STATE_PARKING) {
                motor_stop_axis(axis);
                s_runtime.state = OS_STATE_FAULT;
                os_hal_buzzer_beep(200, 2);
                return;
            }
        }
    }

    switch (s_runtime.state) {
    case OS_STATE_GOTO:
    case OS_STATE_PARKING:
        advance_goto();
        break;
    case OS_STATE_MANUAL:
        advance_manual();
        break;
    case OS_STATE_IDLE_TRACKING:
        advance_tracking();
        break;
    default:
        break;
    }

    poll_commands();
}

os_error_t os_goto_equatorial(double ra_hours, double dec_degrees) {
    if (!in_range(ra_hours, 0.0, 23.999999) ||
        !in_range(dec_degrees, -90.0, 90.0)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_runtime.state == OS_STATE_PARKED ||
        s_runtime.state == OS_STATE_PARKING ||
        s_runtime.state == OS_STATE_FAULT ||
        s_runtime.state == OS_STATE_GOTO) {
        return OS_ERR_INVALID_STATE;
    }

    if (os_hal_limit_is_triggered(OS_AXIS_RA) ||
        os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    double sx, sy;
    motor_steps_from_celestial(ra_hours, dec_degrees, &sx, &sy);

    double d0 = sx - s_runtime.current_steps[OS_AXIS_RA];
    double d1 = sy - s_runtime.current_steps[OS_AXIS_DEC];
    if (fabs(d0) < 0.5 && fabs(d1) < 0.5) {
        s_runtime.current_steps[OS_AXIS_RA] = sx;
        s_runtime.current_steps[OS_AXIS_DEC] = sy;
        hal_update_motor_position(OS_AXIS_RA, s_runtime.current_steps[OS_AXIS_RA]);
        hal_update_motor_position(OS_AXIS_DEC, s_runtime.current_steps[OS_AXIS_DEC]);
        s_runtime.state = OS_STATE_IDLE_TRACKING;
        os_hal_buzzer_beep(100, 2);
        return OS_ERR_NONE;
    }

    s_runtime.target_steps[OS_AXIS_RA] = sx;
    s_runtime.target_steps[OS_AXIS_DEC] = sy;
    s_runtime.state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (s_runtime.state != OS_STATE_GOTO && s_runtime.state != OS_STATE_PARKING) {
        return OS_ERR_INVALID_STATE;
    }
    motor_stop_axis(OS_AXIS_RA);
    motor_stop_axis(OS_AXIS_DEC);
    s_runtime.state = OS_STATE_IDLE_TRACKING;
    apply_tracking_frequencies();
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *moving) {
    if (moving == NULL) return OS_ERR_INVALID_ARGUMENT;
    *moving = (s_runtime.state == OS_STATE_GOTO ||
               s_runtime.state == OS_STATE_PARKING ||
               s_runtime.state == OS_STATE_MANUAL);
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_guide_direction_t direction, uint32_t duration_ms) {
    if (duration_ms == 0) return OS_ERR_INVALID_ARGUMENT;

    uint8_t axis;
    double sign;
    switch (direction) {
    case OS_GUIDE_RA_PLUS:  axis = OS_AXIS_RA; sign =  1.0; break;
    case OS_GUIDE_RA_MINUS: axis = OS_AXIS_RA; sign = -1.0; break;
    case OS_GUIDE_DEC_PLUS: axis = OS_AXIS_DEC; sign =  1.0; break;
    case OS_GUIDE_DEC_MINUS:axis = OS_AXIS_DEC; sign = -1.0; break;
    default: return OS_ERR_INVALID_ARGUMENT;
    }

    /* Deterministic conflict handling for the same axis. */
    if (s_runtime.guide_active[axis]) {
        double existing_sign = (s_runtime.guide_frequency_offset_hz[axis] >= 0.0) ? 1.0 : -1.0;
        if (existing_sign != sign) {
            return OS_ERR_INVALID_STATE;
        }
        s_runtime.guide_remaining_s[axis] = duration_ms / 1000.0;
        return OS_ERR_NONE;
    }

    s_runtime.guide_active[axis] = true;
    s_runtime.guide_remaining_s[axis] = duration_ms / 1000.0;
    s_runtime.guide_frequency_offset_hz[axis] =
        sign * DEFAULT_TRACKING_FREQUENCY_HZ * DEFAULT_GUIDE_RATE;
    return OS_ERR_NONE;
}

os_error_t os_align_add_star(double ra_hours, double dec_degrees,
                             int32_t ra_steps, int32_t dec_steps) {
    if (!in_range(ra_hours, 0.0, 23.999999) ||
        !in_range(dec_degrees, -90.0, 90.0)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_runtime.align_star_count >= OS_ALIGN_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    align_star_t *star = &s_runtime.align_stars[s_runtime.align_star_count++];
    star->ra_hours = ra_hours;
    star->dec_degrees = dec_degrees;
    star->ra_steps = ra_steps;
    star->dec_steps = dec_steps;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(uint8_t align_mode) {
    if (align_mode != OS_ALIGN_MODE_1STAR &&
        align_mode != OS_ALIGN_MODE_2STAR &&
        align_mode != OS_ALIGN_MODE_3STAR &&
        align_mode != OS_ALIGN_MODE_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    uint8_t min_stars = 0;
    switch (align_mode) {
    case OS_ALIGN_MODE_1STAR: min_stars = 1; break;
    case OS_ALIGN_MODE_2STAR: min_stars = 2; break;
    case OS_ALIGN_MODE_3STAR: min_stars = 3; break;
    case OS_ALIGN_MODE_NSTAR: min_stars = 3; break;
    default: return OS_ERR_INVALID_ARGUMENT;
    }

    if (s_runtime.align_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    double m[6] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    double residual = 0.0;
    bool ok = true;

    const double ra_scale = DEFAULT_AXIS0_STEPS_PER_RA_HOUR;
    const double dec_scale = DEFAULT_AXIS1_STEPS_PER_DEGREE;

    if (align_mode == OS_ALIGN_MODE_1STAR) {
        const align_star_t *s0 = &s_runtime.align_stars[0];
        double x0 = s0->ra_hours * ra_scale;
        double y0 = s0->dec_degrees * dec_scale;
        m[0] = 1.0; m[1] = 0.0; m[2] = (double)s0->ra_steps - x0;
        m[3] = 0.0; m[4] = 1.0; m[5] = (double)s0->dec_steps - y0;
        ok = true;
    } else if (align_mode == OS_ALIGN_MODE_2STAR) {
        const align_star_t *s0 = &s_runtime.align_stars[0];
        const align_star_t *s1 = &s_runtime.align_stars[1];
        double x0 = s0->ra_hours * ra_scale;
        double x1 = s1->ra_hours * ra_scale;
        double y0 = s0->dec_degrees * dec_scale;
        double y1 = s1->dec_degrees * dec_scale;

        if (fabs(x1 - x0) < 1e-12 || fabs(y1 - y0) < 1e-12) {
            return OS_ERR_INVALID_STATE;
        }

        double a = ((double)s1->ra_steps - (double)s0->ra_steps) / (x1 - x0);
        double c = (double)s0->ra_steps - a * x0;
        double e = ((double)s1->dec_steps - (double)s0->dec_steps) / (y1 - y0);
        double f = (double)s0->dec_steps - e * y0;

        m[0] = a; m[1] = 0.0; m[2] = c;
        m[3] = 0.0; m[4] = e; m[5] = f;
        ok = true;
    } else {
        if (align_mode == OS_ALIGN_MODE_3STAR && s_runtime.align_star_count != 3) {
            return OS_ERR_INVALID_STATE;
        }

        int n = 2 * (int)s_runtime.align_star_count;
        double A[32 * 6];
        double b[32];
        memset(A, 0, sizeof(A));
        memset(b, 0, sizeof(b));

        for (uint8_t i = 0; i < s_runtime.align_star_count; ++i) {
            const align_star_t *s = &s_runtime.align_stars[i];
            double x = s->ra_hours * ra_scale;
            double y = s->dec_degrees * dec_scale;

            A[(2 * i) * 6 + 0] = x;
            A[(2 * i) * 6 + 1] = y;
            A[(2 * i) * 6 + 2] = 1.0;
            b[2 * i] = (double)s->ra_steps;

            A[(2 * i + 1) * 6 + 3] = x;
            A[(2 * i + 1) * 6 + 4] = y;
            A[(2 * i + 1) * 6 + 5] = 1.0;
            b[2 * i + 1] = (double)s->dec_steps;
        }

        ok = qr_solve(A, b, n, 6, m) != 0;
        if (!ok) return OS_ERR_INVALID_STATE;
    }

    /* Residual calculation uses double precision. */
    double sum_sq = 0.0;
    for (uint8_t i = 0; i < s_runtime.align_star_count; ++i) {
        const align_star_t *s = &s_runtime.align_stars[i];
        double x = s->ra_hours * ra_scale;
        double y = s->dec_degrees * dec_scale;
        double sx = m[0] * x + m[1] * y + m[2];
        double sy = m[3] * x + m[4] * y + m[5];
        double dr = sx - (double)s->ra_steps;
        double dd = sy - (double)s->dec_steps;
        sum_sq += (dr * dr + dd * dd);
    }

    if (align_mode == OS_ALIGN_MODE_3STAR) {
        residual = 0.0;
    } else if (align_mode == OS_ALIGN_MODE_NSTAR || align_mode == OS_ALIGN_MODE_2STAR) {
        residual = sqrt(sum_sq / (double)(2 * s_runtime.align_star_count));
    } else {
        residual = sqrt(sum_sq / (double)(2 * s_runtime.align_star_count));
    }

    memcpy(s_runtime.align_matrix, m, sizeof(m));
    s_runtime.align_valid = true;
    s_runtime.align_mode = align_mode;
    s_runtime.align_residual_arcsec = residual;
    s_runtime.align_residual_computed = true;
    nvm_save();
    return OS_ERR_NONE;
}

os_error_t os_align_clear(void) {
    s_runtime.align_star_count = 0;
    s_runtime.align_valid = false;
    s_runtime.align_residual_computed = false;
    s_runtime.align_residual_arcsec = 0.0;
    domain_default_matrix(s_runtime.align_matrix);
    nvm_save();
    return OS_ERR_NONE;
}

os_error_t os_query_alignment_residual(double *residual_arcsec) {
    if (residual_arcsec == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (!s_runtime.align_residual_computed) return OS_ERR_INVALID_STATE;
    *residual_arcsec = s_runtime.align_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (s_runtime.state == OS_STATE_PARKED) return OS_ERR_NONE;
    if (s_runtime.state != OS_STATE_IDLE_TRACKING) return OS_ERR_INVALID_STATE;
    if (os_hal_limit_is_triggered(OS_AXIS_RA) ||
        os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_runtime.target_steps[OS_AXIS_RA] = s_persistent.park_axis0_steps;
    s_runtime.target_steps[OS_AXIS_DEC] = s_persistent.park_axis1_steps;
    s_runtime.state = OS_STATE_PARKING;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (s_runtime.state != OS_STATE_PARKED) return OS_ERR_INVALID_STATE;

    for (uint8_t axis = 0; axis < OS_MAX_AXES; ++axis) {
        (void)os_hal_motor_init(axis);
    }
    for (uint8_t ch = 0; ch < OS_MAX_CHANNELS; ++ch) {
        (void)os_hal_comm_init(ch);
    }
    (void)os_hal_rtc_init();

    s_runtime.state = OS_STATE_IDLE_TRACKING;
    apply_tracking_frequencies();
    return OS_ERR_NONE;
}

os_error_t os_set_site(const os_site_info_t *site) {
    if (site == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (!in_range(site->latitude_degrees, -90.0, 90.0) ||
        !in_range(site->longitude_degrees, -180.0, 180.0) ||
        !isfinite(site->elevation_metres)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    s_runtime.site = *site;
    s_runtime.site.valid = true;
    nvm_save();
    return OS_ERR_NONE;
}

os_error_t os_get_site(os_site_info_t *site) {
    if (site == NULL) return OS_ERR_INVALID_ARGUMENT;
    *site = s_runtime.site;
    return OS_ERR_NONE;
}

os_error_t os_set_tracking_rate(double rate_factor) {
    if (!isfinite(rate_factor) || rate_factor < 0.0 || rate_factor > 10.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_runtime.tracking_rate_factor = rate_factor;
    if (s_runtime.state == OS_STATE_IDLE_TRACKING) {
        apply_tracking_frequencies();
    }
    nvm_save();
    return OS_ERR_NONE;
}

os_error_t os_query_tracking_rate(double *rate_factor) {
    if (rate_factor == NULL) return OS_ERR_INVALID_ARGUMENT;
    *rate_factor = s_runtime.tracking_rate_factor;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (state == NULL) return OS_ERR_INVALID_ARGUMENT;
    *state = s_runtime.state;
    return OS_ERR_NONE;
}

os_error_t os_query_axis_position(os_axis_t axis, int32_t *position) {
    if (position == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (axis != OS_AXIS_RA && axis != OS_AXIS_DEC) return OS_ERR_INVALID_ARGUMENT;
    *position = os_hal_motor_get_position((uint8_t)axis);
    return OS_ERR_NONE;
}

os_error_t os_manual_move(os_axis_t axis, bool forward, double frequency_hz) {
    if (axis != OS_AXIS_RA && axis != OS_AXIS_DEC) return OS_ERR_INVALID_ARGUMENT;
    if (!isfinite(frequency_hz) || frequency_hz < 0.0) return OS_ERR_INVALID_ARGUMENT;
    if (os_hal_limit_is_triggered((uint8_t)axis)) return OS_ERR_LIMIT_TRIGGERED;

    uint8_t a = (uint8_t)axis;
    s_runtime.axis_manual_active[a] = true;
    s_runtime.axis_manual_frequency_hz[a] = clamp_frequency(frequency_hz);
    s_runtime.axis_direction[a] = forward;
    s_runtime.state = OS_STATE_MANUAL;

    motor_apply_axis(a, s_runtime.axis_manual_frequency_hz[a], forward);
    return OS_ERR_NONE;
}

os_error_t os_manual_stop(os_axis_t axis) {
    if (axis != OS_AXIS_RA && axis != OS_AXIS_DEC) return OS_ERR_INVALID_ARGUMENT;

    uint8_t a = (uint8_t)axis;
    s_runtime.axis_manual_active[a] = false;
    motor_stop_axis(a);

    bool any_active = false;
    for (uint8_t i = 0; i < OS_MAX_AXES; ++i) {
        if (s_runtime.axis_manual_active[i]) any_active = true;
    }

    if (!any_active && s_runtime.state == OS_STATE_MANUAL) {
        s_runtime.state = OS_STATE_IDLE_TRACKING;
        apply_tracking_frequencies();
    }
    return OS_ERR_NONE;
}

os_error_t os_pec_set(const os_pec_point_t *points, uint16_t count) {
    if (points == NULL) {
        if (count == 0) {
            s_runtime.pec_valid = false;
            s_runtime.pec_count = 0;
            return OS_ERR_NONE;
        }
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (count > OS_PEC_MAX_POINTS) return OS_ERR_INVALID_ARGUMENT;

    for (uint16_t i = 0; i < count; ++i) {
        if (!in_range(points[i].worm_phase_deg, 0.0, 360.0) ||
            !isfinite(points[i].correction_arcsec)) {
            return OS_ERR_INVALID_ARGUMENT;
        }
    }

    memcpy(s_runtime.pec_table, points, count * sizeof(os_pec_point_t));
    for (uint16_t i = 0; i < count; ++i) {
        for (uint16_t j = i + 1; j < count; ++j) {
            if (s_runtime.pec_table[j].worm_phase_deg <
                s_runtime.pec_table[i].worm_phase_deg) {
                os_pec_point_t tmp = s_runtime.pec_table[i];
                s_runtime.pec_table[i] = s_runtime.pec_table[j];
                s_runtime.pec_table[j] = tmp;
            }
        }
    }

    s_runtime.pec_count = count;
    s_runtime.pec_valid = (count > 0);
    return OS_ERR_NONE;
}

os_error_t os_pec_clear(void) {
    s_runtime.pec_valid = false;
    s_runtime.pec_count = 0;
    return OS_ERR_NONE;
}

os_error_t os_pec_apply(double worm_phase_deg, double *correction_arcsec) {
    if (correction_arcsec == NULL) return OS_ERR_INVALID_ARGUMENT;
    if (!in_range(worm_phase_deg, 0.0, 360.0)) return OS_ERR_INVALID_ARGUMENT;
    if (worm_phase_deg > 360.0) return OS_ERR_INVALID_ARGUMENT;
    if (worm_phase_deg == 360.0) worm_phase_deg = 0.0;

    if (!s_runtime.pec_valid || s_runtime.pec_count == 0) {
        *correction_arcsec = 0.0;
        return OS_ERR_NONE;
    }

    const os_pec_point_t *table = s_runtime.pec_table;
    uint16_t n = s_runtime.pec_count;

    if (worm_phase_deg <= table[0].worm_phase_deg) {
        *correction_arcsec = table[0].correction_arcsec;
        return OS_ERR_NONE;
    }
    if (worm_phase_deg >= table[n - 1].worm_phase_deg) {
        if (worm_phase_deg >= 360.0) {
            *correction_arcsec = table[0].correction_arcsec;
        } else {
            *correction_arcsec = table[n - 1].correction_arcsec;
        }
        return OS_ERR_NONE;
    }

    for (uint16_t i = 0; i < (uint16_t)(n - 1); ++i) {
        if (worm_phase_deg >= table[i].worm_phase_deg &&
            worm_phase_deg <= table[i + 1].worm_phase_deg) {
            double x0 = table[i].worm_phase_deg;
            double x1 = table[i + 1].worm_phase_deg;
            double y0 = table[i].correction_arcsec;
            double y1 = table[i + 1].correction_arcsec;
            double denom = x1 - x0;
            if (fabs(denom) < 1e-12) {
                *correction_arcsec = y0;
            } else {
                double t = (worm_phase_deg - x0) / denom;
                *correction_arcsec = y0 + t * (y1 - y0);
            }
            return OS_ERR_NONE;
        }
    }

    *correction_arcsec = table[n - 1].correction_arcsec;
    return OS_ERR_NONE;
}

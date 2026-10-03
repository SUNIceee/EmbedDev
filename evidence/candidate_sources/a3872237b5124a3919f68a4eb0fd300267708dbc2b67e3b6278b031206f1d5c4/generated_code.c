#include "generated_code.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* -------------------------------------------------------------------------
 * Internal Data Types & Persistent Storage Structures
 * ------------------------------------------------------------------------- */
#define OS_NVM_MAGIC 0x4F4E5354U

typedef struct {
    uint32_t magic;
    double transform[2][2];
    double offset[2];
    bool valid;
    double park_ra;
    double park_dec;
} os_nvm_calibration_t;

typedef struct {
    double ra_hours;
    double dec_degrees;
    int32_t steps[OS_MAX_AXES];
} os_align_star_t;

typedef struct {
    uint8_t buffer[256];
    size_t head;
    size_t tail;
    size_t count;
} os_comm_ring_t;

/* -------------------------------------------------------------------------
 * Global / Static State Variables
 * ------------------------------------------------------------------------- */
/* System Status */
static os_state_t g_system_state = OS_STATE_INIT;
static os_tracking_rate_t g_tracking_rate = OS_TRACK_SIDEREAL;
static double g_custom_tracking_hz = 4.1666667;

/* Motor Control State */
static bool g_motor_enabled[OS_MAX_AXES] = {false, false};
static bool g_motor_dir_forward[OS_MAX_AXES] = {true, true};
static double g_motor_freq[OS_MAX_AXES] = {0.0, 0.0};
static int32_t g_motor_pos[OS_MAX_AXES] = {0, 0};
static bool g_motor_fault[OS_MAX_AXES] = {false, false};

/* Limits */
static bool g_limit_triggered[OS_MAX_AXES] = {false, false};

/* Sensors / Location / Time */
static os_site_info_t g_site_info = {0.0, 0.0, 0.0, 0, false};
static uint32_t g_rtc_seconds = 1600000000U;

/* Goto Target & Motion Planning */
static bool g_goto_active = false;
static int32_t g_goto_target_steps[OS_MAX_AXES] = {0, 0};
static double g_target_ra_hours = 0.0;
static double g_target_dec_degrees = 0.0;

/* Manual Motion State */
static bool g_manual_active[OS_MAX_AXES] = {false, false};

/* Park State */
static bool g_park_active = false;

/* Alignment Data */
static os_align_star_t g_align_stars[OS_MAX_ALIGN_STARS];
static uint8_t g_align_star_count = 0;
static bool g_align_residual_calculated = false;
static double g_align_residual_arcsec = 0.0;

/* Calibration Matrix: maps (RA*15, Dec) degrees to (Motor 0, Motor 1) steps */
static double g_cal_matrix[2][2] = {{1000.0, 0.0}, {0.0, 1000.0}};
static double g_cal_offset[2] = {0.0, 0.0};
static bool g_cal_valid = false;

/* Guide Pulse State */
static bool g_guide_active = false;
static uint32_t g_guide_ra_end_time = 0;
static uint32_t g_guide_dec_end_time = 0;
static os_guide_direction_t g_guide_ra_dir = OS_GUIDE_EAST;
static os_guide_direction_t g_guide_dec_dir = OS_GUIDE_NORTH;

/* PEC State */
static bool g_pec_enabled = false;
static double g_pec_phase_deg = 0.0;
static double g_pec_correction_arcsec = 0.0;

/* Communication Buffers */
static os_comm_ring_t g_comm_rx[OS_MAX_CHANNELS];
static char g_comm_tx[OS_MAX_CHANNELS][256];
static size_t g_comm_tx_len[OS_MAX_CHANNELS];
static char g_cmd_accum[OS_MAX_CHANNELS][OS_MAX_COMMAND_LENGTH];
static size_t g_cmd_accum_len[OS_MAX_CHANNELS];

/* NVM Mock Memory */
static uint8_t g_nvm_storage[512];

/* Timer counter simulation */
static uint32_t g_system_tick_ms = 0;

/* -------------------------------------------------------------------------
 * Internal Utility Functions
 * ------------------------------------------------------------------------- */
static double get_baseline_tracking_hz(void) {
    switch (g_tracking_rate) {
        case OS_TRACK_SIDEREAL: return 15.0 * OS_STEPS_PER_DEGREE / 3600.0; /* ~4.1667 Hz */
        case OS_TRACK_LUNAR:    return 14.51 * OS_STEPS_PER_DEGREE / 3600.0;
        case OS_TRACK_SOLAR:    return 14.9589 * OS_STEPS_PER_DEGREE / 3600.0;
        case OS_TRACK_CUSTOM:   return g_custom_tracking_hz;
        default:                return 4.1666667;
    }
}

static void celestial_to_steps(double ra_hours, double dec_degrees, int32_t *steps_0, int32_t *steps_1) {
    double x_deg = ra_hours * 15.0;
    double y_deg = dec_degrees;
    if (g_cal_valid) {
        *steps_0 = (int32_t)lround(g_cal_matrix[0][0] * x_deg + g_cal_matrix[0][1] * y_deg + g_cal_offset[0]);
        *steps_1 = (int32_t)lround(g_cal_matrix[1][0] * x_deg + g_cal_matrix[1][1] * y_deg + g_cal_offset[1]);
    } else {
        *steps_0 = (int32_t)lround(x_deg * OS_STEPS_PER_DEGREE);
        *steps_1 = (int32_t)lround(y_deg * OS_STEPS_PER_DEGREE);
    }
}

static void steps_to_celestial(int32_t steps_0, int32_t steps_1, double *ra_hours, double *dec_degrees) {
    if (g_cal_valid) {
        double s0 = (double)steps_0 - g_cal_offset[0];
        double s1 = (double)steps_1 - g_cal_offset[1];
        double det = g_cal_matrix[0][0] * g_cal_matrix[1][1] - g_cal_matrix[0][1] * g_cal_matrix[1][0];
        if (fabs(det) > 1e-9) {
            double x_deg = (g_cal_matrix[1][1] * s0 - g_cal_matrix[0][1] * s1) / det;
            double y_deg = (-g_cal_matrix[1][0] * s0 + g_cal_matrix[0][0] * s1) / det;
            *ra_hours = fmod(x_deg / 15.0, 24.0);
            if (*ra_hours < 0.0) *ra_hours += 24.0;
            *dec_degrees = y_deg;
            return;
        }
    }
    *ra_hours = fmod(((double)steps_0 / OS_STEPS_PER_DEGREE) / 15.0, 24.0);
    if (*ra_hours < 0.0) *ra_hours += 24.0;
    *dec_degrees = (double)steps_1 / OS_STEPS_PER_DEGREE;
}

/* -------------------------------------------------------------------------
 * HAL Implementations
 * ------------------------------------------------------------------------- */
void os_hal_motor_init(uint8_t axis) {
    if (axis < OS_MAX_AXES) {
        g_motor_enabled[axis] = false;
        g_motor_dir_forward[axis] = true;
        g_motor_freq[axis] = 0.0;
    }
}

void os_hal_motor_set_frequency(uint8_t axis, double frequency_hz) {
    if (axis < OS_MAX_AXES) {
        g_motor_freq[axis] = frequency_hz;
    }
}

void os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (axis < OS_MAX_AXES) {
        g_motor_dir_forward[axis] = forward;
    }
}

void os_hal_motor_enable(uint8_t axis, bool enable) {
    if (axis < OS_MAX_AXES) {
        g_motor_enabled[axis] = enable;
    }
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (axis < OS_MAX_AXES) {
        return g_motor_pos[axis];
    }
    return 0;
}

void os_hal_timer_motor_init(void) {
    /* Timer initialization */
}

void os_hal_gps_init(void) {
}

bool os_hal_gps_poll(os_site_info_t *site) {
    if (!site) return false;
    *site = g_site_info;
    return g_site_info.valid;
}

void os_hal_rtc_init(void) {
}

bool os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (!utc_epoch_seconds) return false;
    *utc_epoch_seconds = g_rtc_seconds;
    return true;
}

bool os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    g_rtc_seconds = utc_epoch_seconds;
    return true;
}

void os_hal_limit_init(void) {
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    /* REQ-FDIR-001: Invalid axis numbers (not 0 or 1) must return true (triggered) */
    if (axis >= OS_MAX_AXES) {
        return true;
    }
    return g_limit_triggered[axis];
}

void os_hal_buzzer_beep(uint32_t duration_ms, uint32_t count) {
    (void)duration_ms;
    (void)count;
}

void os_hal_comm_init(uint8_t channel) {
    if (channel < OS_MAX_CHANNELS) {
        g_comm_rx[channel].head = 0;
        g_comm_rx[channel].tail = 0;
        g_comm_rx[channel].count = 0;
        g_comm_tx_len[channel] = 0;
        g_cmd_accum_len[channel] = 0;
    }
}

size_t os_hal_comm_available(uint8_t channel) {
    if (channel < OS_MAX_CHANNELS) {
        return g_comm_rx[channel].count;
    }
    return 0;
}

int os_hal_comm_read(uint8_t channel) {
    if (channel >= OS_MAX_CHANNELS || g_comm_rx[channel].count == 0) {
        return -1;
    }
    uint8_t b = g_comm_rx[channel].buffer[g_comm_rx[channel].tail];
    g_comm_rx[channel].tail = (g_comm_rx[channel].tail + 1) % sizeof(g_comm_rx[channel].buffer);
    g_comm_rx[channel].count--;
    return b;
}

void os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (channel >= OS_MAX_CHANNELS || !data) return;
    for (size_t i = 0; i < length; i++) {
        if (g_comm_tx_len[channel] < sizeof(g_comm_tx[channel]) - 1) {
            g_comm_tx[channel][g_comm_tx_len[channel]++] = data[i];
            g_comm_tx[channel][g_comm_tx_len[channel]] = '\0';
        }
    }
}

void os_hal_nvm_init(void) {
}

bool os_hal_nvm_read(uint32_t offset, void *data, size_t length) {
    if (!data || offset + length > sizeof(g_nvm_storage)) return false;
    memcpy(data, g_nvm_storage + offset, length);
    return true;
}

bool os_hal_nvm_write(uint32_t offset, const void *data, size_t length) {
    if (!data || offset + length > sizeof(g_nvm_storage)) return false;
    memcpy(g_nvm_storage + offset, data, length);
    return true;
}

/* Host Mock Helpers */
void mock_set_motor_fault(uint8_t axis, bool fault) {
    if (axis < OS_MAX_AXES) {
        g_motor_fault[axis] = fault;
    }
}

void mock_set_limit_triggered(uint8_t axis, bool triggered) {
    if (axis < OS_MAX_AXES) {
        g_limit_triggered[axis] = triggered;
    }
}

void mock_inject_gps(const os_site_info_t *site) {
    if (site) {
        g_site_info = *site;
    }
}

void mock_inject_comm_rx(uint8_t channel, const char *data, size_t length) {
    if (channel >= OS_MAX_CHANNELS || !data) return;
    for (size_t i = 0; i < length; i++) {
        if (g_comm_rx[channel].count < sizeof(g_comm_rx[channel].buffer)) {
            g_comm_rx[channel].buffer[g_comm_rx[channel].head] = (uint8_t)data[i];
            g_comm_rx[channel].head = (g_comm_rx[channel].head + 1) % sizeof(g_comm_rx[channel].buffer);
            g_comm_rx[channel].count++;
        }
    }
}

size_t mock_get_comm_tx(uint8_t channel, char *buffer, size_t max_len) {
    if (channel >= OS_MAX_CHANNELS || !buffer || max_len == 0) return 0;
    size_t copy_len = g_comm_tx_len[channel];
    if (copy_len >= max_len) copy_len = max_len - 1;
    memcpy(buffer, g_comm_tx[channel], copy_len);
    buffer[copy_len] = '\0';
    g_comm_tx_len[channel] = 0;
    g_comm_tx[channel][0] = '\0';
    return copy_len;
}

/* -------------------------------------------------------------------------
 * Core Application Logic Implementations
 * ------------------------------------------------------------------------- */
os_error_t os_init(void) {
    /* 1. Reset all runtime state flags explicitly */
    g_align_star_count = 0;
    g_align_residual_calculated = false;
    g_align_residual_arcsec = 0.0;
    g_guide_active = false;
    g_guide_ra_end_time = 0;
    g_guide_dec_end_time = 0;
    g_manual_active[0] = false;
    g_manual_active[1] = false;
    g_goto_active = false;
    g_park_active = false;
    g_pec_enabled = false;

    /* 2. Restore persistent data if available in NVM */
    os_hal_nvm_init();
    os_nvm_calibration_t nvm_cal;
    if (os_hal_nvm_read(0, &nvm_cal, sizeof(nvm_cal)) && nvm_cal.magic == OS_NVM_MAGIC) {
        memcpy(g_cal_matrix, nvm_cal.transform, sizeof(g_cal_matrix));
        memcpy(g_cal_offset, nvm_cal.offset, sizeof(g_cal_offset));
        g_cal_valid = nvm_cal.valid;
    } else {
        g_cal_matrix[0][0] = 1000.0; g_cal_matrix[0][1] = 0.0;
        g_cal_matrix[1][0] = 0.0;    g_cal_matrix[1][1] = 1000.0;
        g_cal_offset[0] = 0.0;       g_cal_offset[1] = 0.0;
        g_cal_valid = false;
    }

    /* 3. Initialize comm channels */
    for (uint8_t c = 0; c < OS_MAX_CHANNELS; c++) {
        os_hal_comm_init(c);
    }

    /* 4. Initialize motors */
    for (uint8_t a = 0; a < OS_MAX_AXES; a++) {
        os_hal_motor_init(a);
        if (g_motor_fault[a]) {
            g_system_state = OS_STATE_FAULT;
            return OS_ERR_HARDWARE;
        }
        os_hal_motor_enable(a, true);
    }

    /* 5. Initialize sensors */
    os_hal_limit_init();
    os_hal_gps_init();
    os_hal_rtc_init();
    os_hal_timer_motor_init();

    /* 6. Default tracking rate */
    g_tracking_rate = OS_TRACK_SIDEREAL;
    os_hal_motor_set_direction(OS_AXIS_RA, true);
    os_hal_motor_set_frequency(OS_AXIS_RA, get_baseline_tracking_hz());
    os_hal_motor_set_frequency(OS_AXIS_DEC, 0.0);

    g_system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *is_moving) {
    if (!is_moving) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *is_moving = (g_system_state == OS_STATE_GOTO ||
                  g_system_state == OS_STATE_MANUAL_MOTION ||
                  g_goto_active || g_park_active ||
                  g_manual_active[0] || g_manual_active[1]);
    return OS_ERR_NONE;
}

os_error_t os_get_status(os_system_status_t *status) {
    if (!status) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    status->state = g_system_state;
    os_query_is_moving(&status->moving);
    status->gps_locked = g_site_info.valid;
    status->motor_steps[0] = os_hal_motor_get_position(0);
    status->motor_steps[1] = os_hal_motor_get_position(1);
    steps_to_celestial(status->motor_steps[0], status->motor_steps[1],
                       &status->current_ra_hours, &status->current_dec_degrees);
    return OS_ERR_NONE;
}

os_error_t os_goto_equatorial(double ra_hours, double dec_degrees) {
    /* Parameter check FIRST */
    if (ra_hours < 0.0 || ra_hours > 24.0 || dec_degrees < -90.0 || dec_degrees > 90.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (g_system_state == OS_STATE_PARKED || g_system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    /* Check limit switches */
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_INVALID_STATE;
    }

    g_target_ra_hours = ra_hours;
    g_target_dec_degrees = dec_degrees;
    celestial_to_steps(ra_hours, dec_degrees, &g_goto_target_steps[0], &g_goto_target_steps[1]);

    g_goto_active = true;
    g_system_state = OS_STATE_GOTO;

    /* Speed set to max Goto speed (~3 deg/sec = 3000 Hz) */
    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);

    for (uint8_t a = 0; a < OS_MAX_AXES; a++) {
        bool fwd = (g_goto_target_steps[a] >= g_motor_pos[a]);
        os_hal_motor_set_direction(a, fwd);
        os_hal_motor_set_frequency(a, (g_goto_target_steps[a] != g_motor_pos[a]) ? 3000.0 : 0.0);
    }

    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (g_goto_active || g_system_state == OS_STATE_GOTO) {
        g_goto_active = false;
        for (uint8_t a = 0; a < OS_MAX_AXES; a++) {
            os_hal_motor_set_frequency(a, 0.0);
        }
        g_system_state = OS_STATE_IDLE_TRACKING;
        os_hal_motor_set_frequency(OS_AXIS_RA, get_baseline_tracking_hz());
    }
    return OS_ERR_NONE;
}

os_error_t os_set_tracking_rate(os_tracking_rate_t rate) {
    if (rate < OS_TRACK_SIDEREAL || rate > OS_TRACK_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_tracking_rate = rate;
    if (g_system_state == OS_STATE_IDLE_TRACKING) {
        os_hal_motor_set_frequency(OS_AXIS_RA, get_baseline_tracking_hz());
    }
    return OS_ERR_NONE;
}

os_error_t os_set_custom_tracking_rate(double rate_hz) {
    if (rate_hz < 0.0 || rate_hz > 1000.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_custom_tracking_hz = rate_hz;
    if (g_tracking_rate == OS_TRACK_CUSTOM && g_system_state == OS_STATE_IDLE_TRACKING) {
        os_hal_motor_set_frequency(OS_AXIS_RA, get_baseline_tracking_hz());
    }
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_guide_direction_t direction, uint32_t duration_ms) {
    if (duration_ms == 0 || direction < OS_GUIDE_EAST || direction > OS_GUIDE_SOUTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    uint32_t now = g_system_tick_ms;
    g_guide_active = true;

    if (direction == OS_GUIDE_EAST || direction == OS_GUIDE_WEST) {
        g_guide_ra_dir = direction;
        g_guide_ra_end_time = now + duration_ms;
    } else {
        g_guide_dec_dir = direction;
        g_guide_dec_end_time = now + duration_ms;
    }

    return OS_ERR_NONE;
}

os_error_t os_align_add_star(double ra_hours, double dec_degrees) {
    if (ra_hours < 0.0 || ra_hours > 24.0 || dec_degrees < -90.0 || dec_degrees > 90.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_align_star_count >= OS_MAX_ALIGN_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    g_align_stars[g_align_star_count].ra_hours = ra_hours;
    g_align_stars[g_align_star_count].dec_degrees = dec_degrees;
    g_align_stars[g_align_star_count].steps[0] = os_hal_motor_get_position(0);
    g_align_stars[g_align_star_count].steps[1] = os_hal_motor_get_position(1);
    g_align_star_count++;

    return OS_ERR_NONE;
}

os_error_t os_align_compute(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1_STAR || mode > OS_ALIGN_N_STAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    /* Check star count requirements */
    if ((mode == OS_ALIGN_1_STAR && g_align_star_count < 1) ||
        (mode == OS_ALIGN_2_STAR && g_align_star_count < 2) ||
        (mode == OS_ALIGN_3_STAR && g_align_star_count < 3) ||
        (mode == OS_ALIGN_N_STAR && g_align_star_count < 3)) {
        return OS_ERR_INVALID_STATE;
    }

    if (mode == OS_ALIGN_1_STAR) {
        double x_deg = g_align_stars[0].ra_hours * 15.0;
        double y_deg = g_align_stars[0].dec_degrees;
        g_cal_matrix[0][0] = 1000.0; g_cal_matrix[0][1] = 0.0;
        g_cal_matrix[1][0] = 0.0;    g_cal_matrix[1][1] = 1000.0;
        g_cal_offset[0] = (double)g_align_stars[0].steps[0] - x_deg * 1000.0;
        g_cal_offset[1] = (double)g_align_stars[0].steps[1] - y_deg * 1000.0;
        g_align_residual_arcsec = 0.0;
    } else if (mode == OS_ALIGN_2_STAR) {
        double x0 = g_align_stars[0].ra_hours * 15.0;
        double y0 = g_align_stars[0].dec_degrees;
        double x1 = g_align_stars[1].ra_hours * 15.0;
        double y1 = g_align_stars[1].dec_degrees;

        if (fabs(x1 - x0) < 1e-5 || fabs(y1 - y0) < 1e-5) {
            return OS_ERR_INVALID_STATE;
        }
        double m00 = ((double)g_align_stars[1].steps[0] - (double)g_align_stars[0].steps[0]) / (x1 - x0);
        double m11 = ((double)g_align_stars[1].steps[1] - (double)g_align_stars[0].steps[1]) / (y1 - y0);

        g_cal_matrix[0][0] = m00; g_cal_matrix[0][1] = 0.0;
        g_cal_matrix[1][0] = 0.0; g_cal_matrix[1][1] = m11;
        g_cal_offset[0] = (double)g_align_stars[0].steps[0] - m00 * x0;
        g_cal_offset[1] = (double)g_align_stars[0].steps[1] - m11 * y0;
        g_align_residual_arcsec = 0.0;
    } else {
        /* 3-star / N-star mode using Cramer's rule / least squares with double precision */
        double X[OS_MAX_ALIGN_STARS], Y[OS_MAX_ALIGN_STARS];
        double S0[OS_MAX_ALIGN_STARS], S1[OS_MAX_ALIGN_STARS];

        for (uint8_t i = 0; i < g_align_star_count; i++) {
            X[i] = g_align_stars[i].ra_hours * 15.0;
            Y[i] = g_align_stars[i].dec_degrees;
            S0[i] = (double)g_align_stars[i].steps[0];
            S1[i] = (double)g_align_stars[i].steps[1];
        }

        if (g_align_star_count == 3) {
            double det = X[0]*(Y[1] - Y[2]) - Y[0]*(X[1] - X[2]) + (X[1]*Y[2] - X[2]*Y[1]);
            if (fabs(det) < 1e-6) {
                return OS_ERR_INVALID_STATE; /* Collinear points */
            }

            /* Axis 0 affine parameters */
            double det_m00 = S0[0]*(Y[1] - Y[2]) - Y[0]*(S0[1] - S0[2]) + (S0[1]*Y[2] - S0[2]*Y[1]);
            double det_m01 = X[0]*(S0[1] - S0[2]) - S0[0]*(X[1] - X[2]) + (X[1]*S0[2] - X[2]*S0[1]);
            double det_c0  = X[0]*(Y[1]*S0[2] - S0[1]*Y[2]) - Y[0]*(X[1]*S0[2] - S0[1]*X[2]) + S0[0]*(X[1]*Y[2] - X[2]*Y[1]);

            /* Axis 1 affine parameters */
            double det_m10 = S1[0]*(Y[1] - Y[2]) - Y[0]*(S1[1] - S1[2]) + (S1[1]*Y[2] - S1[2]*Y[1]);
            double det_m11 = X[0]*(S1[1] - S1[2]) - S1[0]*(X[1] - X[2]) + (X[1]*S1[2] - X[2]*S1[1]);
            double det_c1  = X[0]*(Y[1]*S1[2] - S1[1]*Y[2]) - Y[0]*(X[1]*S1[2] - S1[1]*X[2]) + S1[0]*(X[1]*Y[2] - X[2]*Y[1]);

            g_cal_matrix[0][0] = det_m00 / det;
            g_cal_matrix[0][1] = det_m01 / det;
            g_cal_offset[0]    = det_c0  / det;

            g_cal_matrix[1][0] = det_m10 / det;
            g_cal_matrix[1][1] = det_m11 / det;
            g_cal_offset[1]    = det_c1  / det;

            g_align_residual_arcsec = 0.0;
        } else {
            /* N-star (> 3): Simple linear regression / normal equations */
            double sum_x = 0, sum_y = 0, sum_x2 = 0, sum_y2 = 0, sum_xy = 0;
            double sum_s0 = 0, sum_xs0 = 0, sum_ys0 = 0;
            double sum_s1 = 0, sum_xs1 = 0, sum_ys1 = 0;
            double N = (double)g_align_star_count;

            for (uint8_t i = 0; i < g_align_star_count; i++) {
                sum_x += X[i]; sum_y += Y[i];
                sum_x2 += X[i]*X[i]; sum_y2 += Y[i]*Y[i]; sum_xy += X[i]*Y[i];
                sum_s0 += S0[i]; sum_xs0 += X[i]*S0[i]; sum_ys0 += Y[i]*S0[i];
                sum_s1 += S1[i]; sum_xs1 += X[i]*S1[i]; sum_ys1 += Y[i]*S1[i];
            }

            g_cal_matrix[0][0] = (N*sum_xs0 - sum_x*sum_s0) / (N*sum_x2 - sum_x*sum_x + 1e-9);
            g_cal_matrix[0][1] = 0.0;
            g_cal_offset[0] = (sum_s0 - g_cal_matrix[0][0]*sum_x) / N;

            g_cal_matrix[1][0] = 0.0;
            g_cal_matrix[1][1] = (N*sum_ys1 - sum_y*sum_s1) / (N*sum_y2 - sum_y*sum_y + 1e-9);
            g_cal_offset[1] = (sum_s1 - g_cal_matrix[1][1]*sum_y) / N;

            /* Calculate residual RMS in arcsec */
            double res_sq_sum = 0.0;
            for (uint8_t i = 0; i < g_align_star_count; i++) {
                double pred0 = g_cal_matrix[0][0]*X[i] + g_cal_offset[0];
                double pred1 = g_cal_matrix[1][1]*Y[i] + g_cal_offset[1];
                double err0_deg = (S0[i] - pred0) / OS_STEPS_PER_DEGREE;
                double err1_deg = (S1[i] - pred1) / OS_STEPS_PER_DEGREE;
                res_sq_sum += (err0_deg*err0_deg + err1_deg*err1_deg) * 3600.0 * 3600.0;
            }
            g_align_residual_arcsec = sqrt(res_sq_sum / N);
        }
    }

    g_cal_valid = true;
    g_align_residual_calculated = true;

    /* Write to NVM */
    os_nvm_calibration_t nvm_cal;
    nvm_cal.magic = OS_NVM_MAGIC;
    memcpy(nvm_cal.transform, g_cal_matrix, sizeof(g_cal_matrix));
    memcpy(nvm_cal.offset, g_cal_offset, sizeof(g_cal_offset));
    nvm_cal.valid = true;
    nvm_cal.park_ra = 0.0;
    nvm_cal.park_dec = 90.0;
    os_hal_nvm_write(0, &nvm_cal, sizeof(nvm_cal));

    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_INVALID_STATE;
    }

    g_park_active = true;
    g_system_state = OS_STATE_GOTO;

    /* Move to park position: RA=0, Dec=+90 */
    celestial_to_steps(0.0, 90.0, &g_goto_target_steps[0], &g_goto_target_steps[1]);
    for (uint8_t a = 0; a < OS_MAX_AXES; a++) {
        bool fwd = (g_goto_target_steps[a] >= g_motor_pos[a]);
        os_hal_motor_set_direction(a, fwd);
        os_hal_motor_set_frequency(a, (g_goto_target_steps[a] != g_motor_pos[a]) ? 2000.0 : 0.0);
    }

    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    for (uint8_t a = 0; a < OS_MAX_AXES; a++) {
        os_hal_motor_enable(a, true);
    }
    for (uint8_t c = 0; c < OS_MAX_CHANNELS; c++) {
        os_hal_comm_init(c);
    }
    uint32_t now;
    os_hal_rtc_read(&now);

    g_park_active = false;
    g_system_state = OS_STATE_IDLE_TRACKING;
    os_hal_motor_set_frequency(OS_AXIS_RA, get_baseline_tracking_hz());
    os_hal_motor_set_frequency(OS_AXIS_DEC, 0.0);
    return OS_ERR_NONE;
}

os_error_t os_manual_move(uint8_t axis, os_direction_t dir, os_speed_rate_t speed) {
    if (axis >= OS_MAX_AXES || (dir != OS_DIR_POSITIVE && dir != OS_DIR_NEGATIVE) ||
        (speed < OS_SPEED_GUIDE || speed > OS_SPEED_MAX)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_INVALID_STATE;
    }

    double freq = 10.0;
    switch (speed) {
        case OS_SPEED_GUIDE:  freq = 15.0; break;
        case OS_SPEED_CENTER: freq = 150.0; break;
        case OS_SPEED_FIND:   freq = 600.0; break;
        case OS_SPEED_MAX:    freq = 2000.0; break;
    }

    g_manual_active[axis] = true;
    g_system_state = OS_STATE_MANUAL_MOTION;

    os_hal_motor_set_direction(axis, dir == OS_DIR_POSITIVE);
    os_hal_motor_set_frequency(axis, freq);
    return OS_ERR_NONE;
}

os_error_t os_manual_stop(uint8_t axis) {
    if (axis >= OS_MAX_AXES) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g_manual_active[axis] = false;
    os_hal_motor_set_frequency(axis, 0.0);

    if (!g_manual_active[0] && !g_manual_active[1]) {
        g_system_state = OS_STATE_IDLE_TRACKING;
        os_hal_motor_set_frequency(OS_AXIS_RA, get_baseline_tracking_hz());
    }

    return OS_ERR_NONE;
}

os_error_t os_pec_set_correction(double worm_phase_deg, double arcsec_error) {
    if (worm_phase_deg < 0.0 || worm_phase_deg > 360.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_pec_phase_deg = worm_phase_deg;
    g_pec_correction_arcsec = arcsec_error;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    g_pec_enabled = enable;
    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * LX200 Command Parser
 * ------------------------------------------------------------------------- */
os_error_t os_process_command(uint8_t channel, const char *cmd, char *reply, size_t reply_max_len) {
    if (channel >= OS_MAX_CHANNELS || !cmd || !reply || reply_max_len == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    reply[0] = '\0';
    size_t len = strlen(cmd);
    if (len < 3 || cmd[0] != ':' || cmd[len - 1] != '#') {
        return OS_ERR_COMMAND_FORMAT;
    }

    /* Query commands: :G...# */
    if (cmd[1] == 'G') {
        if (strcmp(cmd, ":GR#") == 0) {
            double ra_h, dec_d;
            steps_to_celestial(g_motor_pos[0], g_motor_pos[1], &ra_h, &dec_d);
            int h = (int)ra_h;
            int m = (int)((ra_h - h) * 60.0);
            int s = (int)((ra_h - h - m / 60.0) * 3600.0);
            snprintf(reply, reply_max_len, "%02d:%02d:%02d#", h, m, s);
        } else if (strcmp(cmd, ":GD#") == 0) {
            double ra_h, dec_d;
            steps_to_celestial(g_motor_pos[0], g_motor_pos[1], &ra_h, &dec_d);
            char sign = (dec_d >= 0) ? '+' : '-';
            double abs_d = fabs(dec_d);
            int d = (int)abs_d;
            int m = (int)((abs_d - d) * 60.0);
            int s = (int)((abs_d - d - m / 60.0) * 3600.0);
            snprintf(reply, reply_max_len, "%c%02d*%02d'%02d#", sign, d, m, s);
        } else if (strcmp(cmd, ":GVP#") == 0) {
            snprintf(reply, reply_max_len, "OnStep#");
        } else if (strcmp(cmd, ":GS#") == 0) {
            snprintf(reply, reply_max_len, "12:00:00#");
        } else {
            snprintf(reply, reply_max_len, "0#");
        }
    }
    /* Set commands: :S...# */
    else if (cmd[1] == 'S') {
        if (cmd[2] == 'r') {
            int h = 0, m = 0, s = 0;
            if (sscanf(cmd + 3, "%d:%d:%d", &h, &m, &s) >= 2) {
                g_target_ra_hours = h + m / 60.0 + s / 3600.0;
                snprintf(reply, reply_max_len, "1");
            } else {
                return OS_ERR_COMMAND_FORMAT;
            }
        } else if (cmd[2] == 'd') {
            int d = 0, m = 0, s = 0;
            char sign = '+';
            if (sscanf(cmd + 3, "%c%d*%d:%d", &sign, &d, &m, &s) >= 2 ||
                sscanf(cmd + 3, "%d*%d:%d", &d, &m, &s) >= 2) {
                double val = d + m / 60.0 + s / 3600.0;
                if (cmd[3] == '-') val = -val;
                g_target_dec_degrees = val;
                snprintf(reply, reply_max_len, "1");
            } else {
                return OS_ERR_COMMAND_FORMAT;
            }
        }
    }
    /* Motion / Goto commands: :M...# */
    else if (cmd[1] == 'M') {
        if (strcmp(cmd, ":MS#") == 0) {
            os_error_t err = os_goto_equatorial(g_target_ra_hours, g_target_dec_degrees);
            if (err == OS_ERR_NONE) {
                snprintf(reply, reply_max_len, "0");
            } else {
                snprintf(reply, reply_max_len, "1Target Out of Bounds#");
            }
        } else if (strcmp(cmd, ":Me#") == 0) {
            os_manual_move(OS_AXIS_RA, OS_DIR_POSITIVE, OS_SPEED_FIND);
        } else if (strcmp(cmd, ":Mw#") == 0) {
            os_manual_move(OS_AXIS_RA, OS_DIR_NEGATIVE, OS_SPEED_FIND);
        } else if (strcmp(cmd, ":Mn#") == 0) {
            os_manual_move(OS_AXIS_DEC, OS_DIR_POSITIVE, OS_SPEED_FIND);
        } else if (strcmp(cmd, ":Ms#") == 0) {
            os_manual_move(OS_AXIS_DEC, OS_DIR_NEGATIVE, OS_SPEED_FIND);
        } else if (strncmp(cmd, ":Mg", 3) == 0) {
            /* Guide pulse: :MgdFFFF# e.g., :Mge0500# */
            char d = cmd[3];
            uint32_t dur = (uint32_t)atoi(cmd + 4);
            os_guide_direction_t gdir = OS_GUIDE_EAST;
            if (d == 'e') gdir = OS_GUIDE_EAST;
            else if (d == 'w') gdir = OS_GUIDE_WEST;
            else if (d == 'n') gdir = OS_GUIDE_NORTH;
            else if (d == 's') gdir = OS_GUIDE_SOUTH;
            os_guide_pulse(gdir, dur);
        }
    }
    /* Stop commands: :Q...# */
    else if (cmd[1] == 'Q') {
        if (strcmp(cmd, ":Qe#") == 0 || strcmp(cmd, ":Qw#") == 0) {
            os_manual_stop(OS_AXIS_RA);
        } else if (strcmp(cmd, ":Qn#") == 0 || strcmp(cmd, ":Qs#") == 0) {
            os_manual_stop(OS_AXIS_DEC);
        } else if (strcmp(cmd, ":Q#") == 0) {
            os_manual_stop(OS_AXIS_RA);
            os_manual_stop(OS_AXIS_DEC);
        }
    }
    /* Park & Wake commands: :hP# / :hO# */
    else if (cmd[1] == 'h') {
        if (strcmp(cmd, ":hP#") == 0) {
            os_park();
            snprintf(reply, reply_max_len, "0");
        } else if (strcmp(cmd, ":hO#") == 0) {
            os_unpark();
            snprintf(reply, reply_max_len, "0");
        }
    }

    return OS_ERR_NONE;
}

/* -------------------------------------------------------------------------
 * Main Loop Iteration
 * ------------------------------------------------------------------------- */
os_error_t os_loop_iteration(void) {
    g_system_tick_ms += 10;

    /* 1. Process incoming communications across all channels */
    for (uint8_t c = 0; c < OS_MAX_CHANNELS; c++) {
        while (os_hal_comm_available(c) > 0) {
            int ch = os_hal_comm_read(c);
            if (ch < 0) break;
            if (g_cmd_accum_len[c] < sizeof(g_cmd_accum[c]) - 1) {
                g_cmd_accum[c][g_cmd_accum_len[c]++] = (char)ch;
                g_cmd_accum[c][g_cmd_accum_len[c]] = '\0';
            }
            if (ch == '#') {
                char reply_buf[OS_MAX_REPLY_LENGTH];
                reply_buf[0] = '\0';
                os_process_command(c, g_cmd_accum[c], reply_buf, sizeof(reply_buf));
                if (reply_buf[0] != '\0') {
                    os_hal_comm_write(c, reply_buf, strlen(reply_buf));
                }
                g_cmd_accum_len[c] = 0;
                g_cmd_accum[c][0] = '\0';
            }
        }
    }

    /* 2. Check FDIR: Limit Switches and Motor Faults */
    for (uint8_t a = 0; a < OS_MAX_AXES; a++) {
        if (g_motor_fault[a]) {
            os_hal_motor_set_frequency(a, 0.0);
            g_system_state = OS_STATE_FAULT;
            return OS_ERR_HARDWARE;
        }

        if (os_hal_limit_is_triggered(a)) {
            os_hal_motor_set_frequency(a, 0.0);
            os_hal_buzzer_beep(500, 3);
            if (g_system_state == OS_STATE_GOTO || g_system_state == OS_STATE_MANUAL_MOTION) {
                g_goto_active = false;
                g_manual_active[a] = false;
                g_system_state = OS_STATE_FAULT;
            }
        }
    }

    /* 3. Advance Motor Positions based on Frequency & Direction */
    for (uint8_t a = 0; a < OS_MAX_AXES; a++) {
        if (g_motor_enabled[a] && g_motor_freq[a] > 0.0) {
            int32_t step_delta = (int32_t)lround(g_motor_freq[a] * 0.010);
            if (step_delta < 1) step_delta = 1;
            if (g_motor_dir_forward[a]) {
                g_motor_pos[a] += step_delta;
            } else {
                g_motor_pos[a] -= step_delta;
            }
        }
    }

    /* 4. Advance Goto Motion State Machine */
    if (g_system_state == OS_STATE_GOTO && g_goto_active) {
        bool ra_reached = false;
        bool dec_reached = false;

        for (uint8_t a = 0; a < OS_MAX_AXES; a++) {
            int32_t diff = g_goto_target_steps[a] - g_motor_pos[a];
            if (labs(diff) <= 15) {
                g_motor_pos[a] = g_goto_target_steps[a];
                os_hal_motor_set_frequency(a, 0.0);
                if (a == OS_AXIS_RA) ra_reached = true;
                if (a == OS_AXIS_DEC) dec_reached = true;
            } else {
                os_hal_motor_set_direction(a, diff > 0);
            }
        }

        if (ra_reached && dec_reached) {
            g_goto_active = false;
            os_hal_buzzer_beep(200, 1);
            if (g_park_active) {
                g_system_state = OS_STATE_PARKED;
                os_hal_motor_enable(0, false);
                os_hal_motor_enable(1, false);
            } else {
                g_system_state = OS_STATE_IDLE_TRACKING;
                os_hal_motor_set_direction(OS_AXIS_RA, true);
                os_hal_motor_set_frequency(OS_AXIS_RA, get_baseline_tracking_hz());
            }
        }
    }

    /* 5. Handle Guide Pulses */
    if (g_guide_active) {
        /* DEC guide has priority over RA */
        if (g_system_tick_ms < g_guide_dec_end_time) {
            double bias = (g_guide_dec_dir == OS_GUIDE_NORTH) ? 15.0 : -15.0;
            os_hal_motor_set_direction(OS_AXIS_DEC, bias > 0);
            os_hal_motor_set_frequency(OS_AXIS_DEC, fabs(bias));
        } else if (g_system_tick_ms < g_guide_ra_end_time) {
            double base = get_baseline_tracking_hz();
            double bias = (g_guide_ra_dir == OS_GUIDE_EAST) ? base * 0.5 : base * -0.5;
            os_hal_motor_set_frequency(OS_AXIS_RA, base + bias);
        } else {
            g_guide_active = false;
            if (g_system_state == OS_STATE_IDLE_TRACKING) {
                os_hal_motor_set_frequency(OS_AXIS_RA, get_baseline_tracking_hz());
                os_hal_motor_set_frequency(OS_AXIS_DEC, 0.0);
            }
        }
    }

    return OS_ERR_NONE;
}

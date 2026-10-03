/* SOURCE */
#include "generated_code.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_ALIGN_STARS 16
#define PEC_TABLE_SIZE  361

/* Steps conversion constants (1000 steps per degree) */
#define STEPS_PER_DEGREE 1000.0
#define SIDEREAL_DEG_PER_SEC (15.0 / 3600.0) /* 15 arcsec/sec */

typedef struct {
    double target_ra;
    double target_dec;
    int32_t motor_step_0;
    int32_t motor_step_1;
} os_align_star_t;

typedef struct {
    uint32_t magic;
    double matrix[2][2];
    double offset[2];
    bool valid;
    double park_ra;
    double park_dec;
} os_nvm_record_t;

/* System Internal State Variables */
static os_state_t g_state = OS_STATE_INIT;
static os_tracking_rate_t g_tracking_rate = OS_TRACK_SIDEREAL;
static double g_custom_tracking_hz = SIDEREAL_DEG_PER_SEC * STEPS_PER_DEGREE;

/* Motor HAL Mock State */
static bool g_motor_initialized[OS_MAX_AXES] = {false, false};
static bool g_motor_enabled[OS_MAX_AXES] = {false, false};
static bool g_motor_direction[OS_MAX_AXES] = {true, true};
static double g_motor_frequency[OS_MAX_AXES] = {0.0, 0.0};
static int32_t g_motor_position[OS_MAX_AXES] = {0, 0};
static bool g_motor_fault[OS_MAX_AXES] = {false, false};

/* Limits Mock State */
static bool g_limit_triggered[OS_MAX_AXES] = {false, false};

/* Sensors Mock State */
static os_site_info_t g_gps_site = {0.0, 0.0, 0.0, 0, false};
static uint32_t g_rtc_epoch = 1704067200; /* Default 2024-01-01 UTC */

/* Comm Channels Ring Buffers */
#define COMM_BUF_SIZE 256
static char g_comm_rx[OS_MAX_CHANNELS][COMM_BUF_SIZE];
static size_t g_comm_rx_head[OS_MAX_CHANNELS] = {0};
static size_t g_comm_rx_tail[OS_MAX_CHANNELS] = {0};

static char g_comm_tx[OS_MAX_CHANNELS][COMM_BUF_SIZE];
static size_t g_comm_tx_len[OS_MAX_CHANNELS] = {0};
static bool g_comm_enabled[OS_MAX_CHANNELS] = {false, false, false, false};

static char g_channel_cmd_buf[OS_MAX_CHANNELS][OS_MAX_COMMAND_LENGTH];
static size_t g_channel_cmd_len[OS_MAX_CHANNELS] = {0};

/* NVM Buffer */
static uint8_t g_nvm_storage[512] = {0};

/* Goto Motion State */
static bool g_goto_active = false;
static int32_t g_goto_target_steps[OS_MAX_AXES] = {0, 0};
static double g_target_ra_hours = 0.0;
static double g_target_dec_degrees = 0.0;

/* Manual Motion State */
static bool g_manual_active[OS_MAX_AXES] = {false, false};
static os_direction_t g_manual_dir[OS_MAX_AXES] = {OS_DIR_POSITIVE, OS_DIR_POSITIVE};
static os_speed_rate_t g_manual_speed[OS_MAX_AXES] = {OS_SPEED_GUIDE, OS_SPEED_GUIDE};

/* Parking State */
static bool g_park_active = false;
static int32_t g_park_target_steps[OS_MAX_AXES] = {0, 0};

/* Guide Pulse State */
static uint32_t g_guide_dec_remaining_ms = 0;
static os_guide_direction_t g_guide_dec_dir = OS_GUIDE_NORTH;
static uint32_t g_guide_ra_remaining_ms = 0;
static os_guide_direction_t g_guide_ra_dir = OS_GUIDE_EAST;

/* Calibration State */
static os_align_star_t g_align_stars[MAX_ALIGN_STARS];
static uint8_t g_align_star_count = 0;
static bool g_align_residual_computed = false;
static double g_align_residual_arcsec = 0.0;
static double g_cal_matrix[2][2] = {{1.0, 0.0}, {0.0, 1.0}};
static double g_cal_offset[2] = {0.0, 0.0};
static bool g_cal_valid = false;

/* PEC State */
static bool g_pec_enabled = false;
static double g_pec_table[PEC_TABLE_SIZE] = {0.0};

/* ========================================================================= */
/* HAL Implementation                                                        */
/* ========================================================================= */

void os_hal_motor_init(uint8_t axis) {
    if (axis < OS_MAX_AXES) {
        g_motor_initialized[axis] = true;
        g_motor_enabled[axis] = false;
        g_motor_frequency[axis] = 0.0;
        g_motor_direction[axis] = true;
    }
}

void os_hal_motor_set_frequency(uint8_t axis, double frequency_hz) {
    if (axis < OS_MAX_AXES) {
        g_motor_frequency[axis] = (frequency_hz < 0.0) ? 0.0 : frequency_hz;
    }
}

void os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (axis < OS_MAX_AXES) {
        g_motor_direction[axis] = forward;
    }
}

void os_hal_motor_enable(uint8_t axis, bool enable) {
    if (axis < OS_MAX_AXES) {
        g_motor_enabled[axis] = enable;
        if (!enable) {
            g_motor_frequency[axis] = 0.0;
        }
    }
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (axis < OS_MAX_AXES) {
        return g_motor_position[axis];
    }
    return 0;
}

void os_hal_timer_motor_init(void) {}

void os_hal_gps_init(void) {}

bool os_hal_gps_poll(os_site_info_t *site) {
    if (!site) return false;
    *site = g_gps_site;
    return g_gps_site.valid;
}

void os_hal_rtc_init(void) {}

bool os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (!utc_epoch_seconds) return false;
    *utc_epoch_seconds = g_rtc_epoch;
    return true;
}

bool os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    g_rtc_epoch = utc_epoch_seconds;
    return true;
}

void os_hal_limit_init(void) {}

bool os_hal_limit_is_triggered(uint8_t axis) {
    /* SRS REQ-FDIR-001: os_hal_limit_is_triggered for invalid axis numbers MUST return true */
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
        g_comm_enabled[channel] = true;
    }
}

size_t os_hal_comm_available(uint8_t channel) {
    if (channel >= OS_MAX_CHANNELS || !g_comm_enabled[channel]) {
        return 0;
    }
    if (g_comm_rx_head[channel] >= g_comm_rx_tail[channel]) {
        return g_comm_rx_head[channel] - g_comm_rx_tail[channel];
    }
    return COMM_BUF_SIZE - g_comm_rx_tail[channel] + g_comm_rx_head[channel];
}

int os_hal_comm_read(uint8_t channel) {
    if (channel >= OS_MAX_CHANNELS || !g_comm_enabled[channel]) return -1;
    if (os_hal_comm_available(channel) == 0) return -1;
    char c = g_comm_rx[channel][g_comm_rx_tail[channel]];
    g_comm_rx_tail[channel] = (g_comm_rx_tail[channel] + 1) % COMM_BUF_SIZE;
    return (unsigned char)c;
}

void os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (channel >= OS_MAX_CHANNELS || !g_comm_enabled[channel] || !data) return;
    for (size_t i = 0; i < length; i++) {
        if (g_comm_tx_len[channel] < COMM_BUF_SIZE) {
            g_comm_tx[channel][g_comm_tx_len[channel]++] = data[i];
        }
    }
}

void os_hal_nvm_init(void) {}

bool os_hal_nvm_read(uint32_t offset, void *data, size_t length) {
    if (!data || (offset + length > sizeof(g_nvm_storage))) return false;
    memcpy(data, g_nvm_storage + offset, length);
    return true;
}

bool os_hal_nvm_write(uint32_t offset, const void *data, size_t length) {
    if (!data || (offset + length > sizeof(g_nvm_storage))) return false;
    memcpy(g_nvm_storage + offset, data, length);
    return true;
}

/* Mock Helpers */
void mock_set_motor_fault(uint8_t axis, bool fault) {
    if (axis < OS_MAX_AXES) g_motor_fault[axis] = fault;
}

void mock_set_limit_triggered(uint8_t axis, bool triggered) {
    if (axis < OS_MAX_AXES) g_limit_triggered[axis] = triggered;
}

void mock_inject_gps(const os_site_info_t *site) {
    if (site) g_gps_site = *site;
}

void mock_inject_comm_rx(uint8_t channel, const char *data, size_t length) {
    if (channel >= OS_MAX_CHANNELS || !data) return;
    for (size_t i = 0; i < length; i++) {
        size_t next_head = (g_comm_rx_head[channel] + 1) % COMM_BUF_SIZE;
        if (next_head != g_comm_rx_tail[channel]) {
            g_comm_rx[channel][g_comm_rx_head[channel]] = data[i];
            g_comm_rx_head[channel] = next_head;
        }
    }
}

size_t mock_get_comm_tx(uint8_t channel, char *buffer, size_t max_len) {
    if (channel >= OS_MAX_CHANNELS || !buffer || max_len == 0) return 0;
    size_t copy_len = (g_comm_tx_len[channel] < max_len) ? g_comm_tx_len[channel] : max_len - 1;
    memcpy(buffer, g_comm_tx[channel], copy_len);
    buffer[copy_len] = '\0';
    g_comm_tx_len[channel] = 0;
    return copy_len;
}

/* ========================================================================= */
/* Core Logic Implementation                                                 */
/* ========================================================================= */

static void get_current_celestial(double *ra_hours, double *dec_degrees) {
    double deg0 = (double)g_motor_position[OS_AXIS_RA] / STEPS_PER_DEGREE;
    double deg1 = (double)g_motor_position[OS_AXIS_DEC] / STEPS_PER_DEGREE;
    if (ra_hours) {
        double ra = deg0 / 15.0;
        while (ra < 0.0) ra += 24.0;
        while (ra >= 24.0) ra -= 24.0;
        *ra_hours = ra;
    }
    if (dec_degrees) {
        if (deg1 > 90.0) deg1 = 90.0;
        if (deg1 < -90.0) deg1 = -90.0;
        *dec_degrees = deg1;
    }
}

static void cel_to_steps(double ra_hours, double dec_degrees, int32_t *step0, int32_t *step1) {
    double deg0 = ra_hours * 15.0;
    double deg1 = dec_degrees;
    if (g_cal_valid) {
        double s0 = g_cal_matrix[0][0] * (deg0 * STEPS_PER_DEGREE) + g_cal_matrix[0][1] * (deg1 * STEPS_PER_DEGREE) + g_cal_offset[0];
        double s1 = g_cal_matrix[1][0] * (deg0 * STEPS_PER_DEGREE) + g_cal_matrix[1][1] * (deg1 * STEPS_PER_DEGREE) + g_cal_offset[1];
        if (step0) *step0 = (int32_t)lround(s0);
        if (step1) *step1 = (int32_t)lround(s1);
    } else {
        if (step0) *step0 = (int32_t)lround(deg0 * STEPS_PER_DEGREE);
        if (step1) *step1 = (int32_t)lround(deg1 * STEPS_PER_DEGREE);
    }
}

os_error_t os_init(void) {
    g_state = OS_STATE_INIT;

    /* Reset ALL runtime boolean / counter state flags explicitly */
    g_align_residual_computed = false;
    g_align_residual_arcsec = 0.0;
    g_align_star_count = 0;
    g_guide_dec_remaining_ms = 0;
    g_guide_ra_remaining_ms = 0;
    g_manual_active[0] = false;
    g_manual_active[1] = false;
    g_goto_active = false;
    g_park_active = false;

    /* Init hardware channels */
    for (uint8_t c = 0; c < OS_MAX_CHANNELS; c++) {
        os_hal_comm_init(c);
        g_comm_rx_head[c] = 0;
        g_comm_rx_tail[c] = 0;
        g_comm_tx_len[c] = 0;
        g_channel_cmd_len[c] = 0;
    }

    os_hal_limit_init();
    os_hal_gps_init();
    os_hal_rtc_init();
    os_hal_nvm_init();
    os_hal_timer_motor_init();

    /* Load NVM persisted settings if valid */
    os_nvm_record_t record;
    if (os_hal_nvm_read(0, &record, sizeof(record)) && record.magic == 0x4F4E5354) {
        g_cal_matrix[0][0] = record.matrix[0][0];
        g_cal_matrix[0][1] = record.matrix[0][1];
        g_cal_matrix[1][0] = record.matrix[1][0];
        g_cal_matrix[1][1] = record.matrix[1][1];
        g_cal_offset[0] = record.offset[0];
        g_cal_offset[1] = record.offset[1];
        g_cal_valid = record.valid;
    } else {
        g_cal_matrix[0][0] = 1.0; g_cal_matrix[0][1] = 0.0;
        g_cal_matrix[1][0] = 0.0; g_cal_matrix[1][1] = 1.0;
        g_cal_offset[0] = 0.0;   g_cal_offset[1] = 0.0;
        g_cal_valid = false;
    }

    /* Initialize Motor Drivers */
    for (uint8_t a = 0; a < OS_MAX_AXES; a++) {
        os_hal_motor_init(a);
        if (g_motor_fault[a]) {
            g_state = OS_STATE_FAULT;
            return OS_ERR_HARDWARE;
        }
        os_hal_motor_enable(a, true);
    }

    g_tracking_rate = OS_TRACK_SIDEREAL;
    g_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_goto_equatorial(double ra_hours, double dec_degrees) {
    /* Parameter range validation MUST precede state checks */
    if (ra_hours < 0.0 || ra_hours > 24.0 || dec_degrees < -90.0 || dec_degrees > 90.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (g_state == OS_STATE_PARKED || g_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    int32_t target0 = 0, target1 = 0;
    cel_to_steps(ra_hours, dec_degrees, &target0, &target1);

    /* Check limit switches before starting motion */
    if (target0 > g_motor_position[OS_AXIS_RA] && os_hal_limit_is_triggered(OS_AXIS_RA)) return OS_ERR_INVALID_STATE;
    if (target1 > g_motor_position[OS_AXIS_DEC] && os_hal_limit_is_triggered(OS_AXIS_DEC)) return OS_ERR_INVALID_STATE;

    g_target_ra_hours = ra_hours;
    g_target_dec_degrees = dec_degrees;
    g_goto_target_steps[OS_AXIS_RA] = target0;
    g_goto_target_steps[OS_AXIS_DEC] = target1;

    g_goto_active = true;
    g_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (g_goto_active || g_park_active || g_state == OS_STATE_GOTO) {
        g_goto_active = false;
        g_park_active = false;
        os_hal_motor_set_frequency(OS_AXIS_RA, 0.0);
        os_hal_motor_set_frequency(OS_AXIS_DEC, 0.0);
        g_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_set_tracking_rate(os_tracking_rate_t rate) {
    if (rate < OS_TRACK_SIDEREAL || rate > OS_TRACK_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_tracking_rate = rate;
    return OS_ERR_NONE;
}

os_error_t os_set_custom_tracking_rate(double rate_hz) {
    if (rate_hz < 0.0 || rate_hz > 1000.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_custom_tracking_hz = rate_hz;
    g_tracking_rate = OS_TRACK_CUSTOM;
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_guide_direction_t direction, uint32_t duration_ms) {
    if (duration_ms == 0 || direction < OS_GUIDE_EAST || direction > OS_GUIDE_SOUTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (direction == OS_GUIDE_NORTH || direction == OS_GUIDE_SOUTH) {
        g_guide_dec_dir = direction;
        g_guide_dec_remaining_ms = duration_ms;
    } else {
        g_guide_ra_dir = direction;
        g_guide_ra_remaining_ms = duration_ms;
    }
    return OS_ERR_NONE;
}

os_error_t os_align_add_star(double ra_hours, double dec_degrees) {
    if (ra_hours < 0.0 || ra_hours > 24.0 || dec_degrees < -90.0 || dec_degrees > 90.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_align_star_count >= MAX_ALIGN_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    g_align_stars[g_align_star_count].target_ra = ra_hours;
    g_align_stars[g_align_star_count].target_dec = dec_degrees;
    g_align_stars[g_align_star_count].motor_step_0 = g_motor_position[OS_AXIS_RA];
    g_align_stars[g_align_star_count].motor_step_1 = g_motor_position[OS_AXIS_DEC];
    g_align_star_count++;

    return OS_ERR_NONE;
}

os_error_t os_align_compute(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1_STAR || mode > OS_ALIGN_N_STAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    /* Minimum star count requirement check */
    if ((mode == OS_ALIGN_1_STAR && g_align_star_count < 1) ||
        (mode == OS_ALIGN_2_STAR && g_align_star_count < 2) ||
        (mode == OS_ALIGN_3_STAR && g_align_star_count < 3) ||
        (mode == OS_ALIGN_N_STAR && g_align_star_count < 3)) {
        return OS_ERR_INVALID_STATE;
    }

    if (mode == OS_ALIGN_1_STAR) {
        double expected0 = g_align_stars[0].target_ra * 15.0 * STEPS_PER_DEGREE;
        double expected1 = g_align_stars[0].target_dec * STEPS_PER_DEGREE;
        g_cal_matrix[0][0] = 1.0; g_cal_matrix[0][1] = 0.0;
        g_cal_matrix[1][0] = 0.0; g_cal_matrix[1][1] = 1.0;
        g_cal_offset[0] = (double)g_align_stars[0].motor_step_0 - expected0;
        g_cal_offset[1] = (double)g_align_stars[0].motor_step_1 - expected1;
        g_align_residual_arcsec = 0.0;
    } else if (mode == OS_ALIGN_2_STAR) {
        double x0 = g_align_stars[0].target_ra * 15.0 * STEPS_PER_DEGREE;
        double x1 = g_align_stars[1].target_ra * 15.0 * STEPS_PER_DEGREE;
        double y0 = (double)g_align_stars[0].motor_step_0;
        double y1 = (double)g_align_stars[1].motor_step_0;
        double dx = x1 - x0;
        if (fabs(dx) < 1e-6) return OS_ERR_INVALID_STATE;
        g_cal_matrix[0][0] = (y1 - y0) / dx; g_cal_matrix[0][1] = 0.0;
        g_cal_matrix[1][0] = 0.0;            g_cal_matrix[1][1] = 1.0;
        g_cal_offset[0] = y0 - g_cal_matrix[0][0] * x0;
        g_cal_offset[1] = 0.0;
        g_align_residual_arcsec = 0.0;
    } else {
        /* 3-star or N-star affine fitting using double precision */
        double X0 = g_align_stars[0].target_ra * 15.0 * STEPS_PER_DEGREE;
        double Y0 = g_align_stars[0].target_dec * STEPS_PER_DEGREE;
        double X1 = g_align_stars[1].target_ra * 15.0 * STEPS_PER_DEGREE;
        double Y1 = g_align_stars[1].target_dec * STEPS_PER_DEGREE;
        double X2 = g_align_stars[2].target_ra * 15.0 * STEPS_PER_DEGREE;
        double Y2 = g_align_stars[2].target_dec * STEPS_PER_DEGREE;

        double detA = X0 * (Y1 - Y2) - Y0 * (X1 - X2) + (X1 * Y2 - X2 * Y1);
        if (fabs(detA) < 1e-6) {
            return OS_ERR_INVALID_STATE; /* Collinear data degenerate */
        }

        double invA[3][3];
        invA[0][0] = (Y1 - Y2) / detA; invA[0][1] = (Y2 - Y0) / detA; invA[0][2] = (Y0 - Y1) / detA;
        invA[1][0] = (X2 - X1) / detA; invA[1][1] = (X0 - X2) / detA; invA[1][2] = (X1 - X0) / detA;
        invA[2][0] = (X1*Y2 - X2*Y1)/detA; invA[2][1] = (X2*Y0 - X0*Y2)/detA; invA[2][2] = (X0*Y1 - X1*Y0)/detA;

        double S0[3] = {(double)g_align_stars[0].motor_step_0, (double)g_align_stars[1].motor_step_0, (double)g_align_stars[2].motor_step_0};
        double S1[3] = {(double)g_align_stars[0].motor_step_1, (double)g_align_stars[1].motor_step_1, (double)g_align_stars[2].motor_step_1};

        g_cal_matrix[0][0] = invA[0][0]*S0[0] + invA[0][1]*S0[1] + invA[0][2]*S0[2];
        g_cal_matrix[0][1] = invA[1][0]*S0[0] + invA[1][1]*S0[1] + invA[1][2]*S0[2];
        g_cal_offset[0]    = invA[2][0]*S0[0] + invA[2][1]*S0[1] + invA[2][2]*S0[2];

        g_cal_matrix[1][0] = invA[0][0]*S1[0] + invA[0][1]*S1[1] + invA[0][2]*S1[2];
        g_cal_matrix[1][1] = invA[1][0]*S1[0] + invA[1][1]*S1[1] + invA[1][2]*S1[2];
        g_cal_offset[1]    = invA[2][0]*S1[0] + invA[2][1]*S1[1] + invA[2][2]*S1[2];

        if (mode == OS_ALIGN_3_STAR || g_align_star_count == 3) {
            g_align_residual_arcsec = 0.0;
        } else {
            double sum_sq_err = 0.0;
            for (uint8_t i = 0; i < g_align_star_count; i++) {
                double tx = g_align_stars[i].target_ra * 15.0 * STEPS_PER_DEGREE;
                double ty = g_align_stars[i].target_dec * STEPS_PER_DEGREE;
                double pred0 = g_cal_matrix[0][0]*tx + g_cal_matrix[0][1]*ty + g_cal_offset[0];
                double pred1 = g_cal_matrix[1][0]*tx + g_cal_matrix[1][1]*ty + g_cal_offset[1];
                double err0 = (double)g_align_stars[i].motor_step_0 - pred0;
                double err1 = (double)g_align_stars[i].motor_step_1 - pred1;
                sum_sq_err += (err0*err0 + err1*err1);
            }
            g_align_residual_arcsec = sqrt(sum_sq_err / g_align_star_count) * (3600.0 / STEPS_PER_DEGREE);
        }
    }

    g_cal_valid = true;
    g_align_residual_computed = true;

    /* Save to NVM */
    os_nvm_record_t record;
    record.magic = 0x4F4E5354;
    record.matrix[0][0] = g_cal_matrix[0][0]; record.matrix[0][1] = g_cal_matrix[0][1];
    record.matrix[1][0] = g_cal_matrix[1][0]; record.matrix[1][1] = g_cal_matrix[1][1];
    record.offset[0] = g_cal_offset[0]; record.offset[1] = g_cal_offset[1];
    record.valid = true;
    record.park_ra = 0.0; record.park_dec = 90.0;
    os_hal_nvm_write(0, &record, sizeof(record));

    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_INVALID_STATE;
    }
    g_park_target_steps[OS_AXIS_RA] = 0;
    g_park_target_steps[OS_AXIS_DEC] = (int32_t)(90.0 * STEPS_PER_DEGREE);
    g_park_active = true;
    g_goto_active = true;
    g_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    for (uint8_t a = 0; a < OS_MAX_AXES; a++) {
        os_hal_motor_enable(a, true);
    }
    for (uint8_t c = 0; c < OS_MAX_CHANNELS; c++) {
        os_hal_comm_init(c);
    }
    g_park_active = false;
    g_goto_active = false;
    g_state = OS_STATE_IDLE_TRACKING;
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

    g_manual_active[axis] = true;
    g_manual_dir[axis] = dir;
    g_manual_speed[axis] = speed;
    g_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_manual_stop(uint8_t axis) {
    if (axis >= OS_MAX_AXES) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_manual_active[axis] = false;
    os_hal_motor_set_frequency(axis, 0.0);
    if (!g_manual_active[0] && !g_manual_active[1]) {
        g_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *is_moving) {
    if (!is_moving) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *is_moving = (g_state == OS_STATE_GOTO || g_state == OS_STATE_MANUAL_MOTION || g_goto_active);
    return OS_ERR_NONE;
}

os_error_t os_get_status(os_system_status_t *status) {
    if (!status) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    status->state = g_state;
    status->moving = (g_state == OS_STATE_GOTO || g_state == OS_STATE_MANUAL_MOTION || g_goto_active);
    status->gps_locked = g_gps_site.valid;
    get_current_celestial(&status->current_ra_hours, &status->current_dec_degrees);
    status->motor_steps[OS_AXIS_RA] = g_motor_position[OS_AXIS_RA];
    status->motor_steps[OS_AXIS_DEC] = g_motor_position[OS_AXIS_DEC];
    status->align_residual_computed = g_align_residual_computed;
    status->align_residual_arcsec = g_align_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_pec_set_correction(double worm_phase_deg, double arcsec_error) {
    if (worm_phase_deg < 0.0 || worm_phase_deg > 360.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int idx = (int)lround(worm_phase_deg);
    if (idx >= 0 && idx < PEC_TABLE_SIZE) {
        g_pec_table[idx] = arcsec_error;
    }
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    g_pec_enabled = enable;
    return OS_ERR_NONE;
}

/* LX200 Command Parser */
os_error_t os_process_command(uint8_t channel, const char *cmd, char *reply, size_t reply_max_len) {
    if (channel >= OS_MAX_CHANNELS || !cmd || !reply || reply_max_len == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    reply[0] = '\0';
    size_t len = strlen(cmd);
    if (len < 3 || cmd[0] != ':' || cmd[len - 1] != '#') {
        return OS_ERR_COMMAND_FORMAT;
    }

    if (cmd[1] == 'G') {
        /* Query commands */
        if (strcmp(cmd, ":GR#") == 0) {
            double ra = 0.0;
            get_current_celestial(&ra, NULL);
            int h = (int)ra;
            int m = (int)((ra - h) * 60.0);
            int s = (int)((ra - h - m/60.0) * 3600.0);
            snprintf(reply, reply_max_len, "%02d:%02d:%02d#", h, m, s);
        } else if (strcmp(cmd, ":GD#") == 0) {
            double dec = 0.0;
            get_current_celestial(NULL, &dec);
            char sign = (dec >= 0) ? '+' : '-';
            double adec = fabs(dec);
            int d = (int)adec;
            int m = (int)((adec - d) * 60.0);
            snprintf(reply, reply_max_len, "%c%02d*%02d#", sign, d, m);
        } else if (strcmp(cmd, ":GVP#") == 0) {
            snprintf(reply, reply_max_len, "OnStep#");
        } else if (strcmp(cmd, ":GS#") == 0) {
            snprintf(reply, reply_max_len, "12:00:00#");
        }
    } else if (cmd[1] == 'S') {
        /* Set commands */
        if (strncmp(cmd, ":Sr", 3) == 0) {
            int h = 0, m = 0, s = 0;
            if (sscanf(cmd + 3, "%d:%d:%d", &h, &m, &s) >= 2) {
                g_target_ra_hours = h + m/60.0 + s/3600.0;
                snprintf(reply, reply_max_len, "1");
            } else {
                snprintf(reply, reply_max_len, "0");
            }
        } else if (strncmp(cmd, ":Sd", 3) == 0) {
            int d = 0, m = 0, s = 0;
            char sign = '+';
            if (sscanf(cmd + 3, "%c%d*%d:%d", &sign, &d, &m, &s) >= 2 ||
                sscanf(cmd + 3, "%d*%d", &d, &m) >= 2) {
                double dec = d + m/60.0 + s/3600.0;
                if (cmd[3] == '-') dec = -dec;
                g_target_dec_degrees = dec;
                snprintf(reply, reply_max_len, "1");
            } else {
                snprintf(reply, reply_max_len, "0");
            }
        }
    } else if (cmd[1] == 'M') {
        /* Motion commands */
        if (strcmp(cmd, ":MS#") == 0) {
            os_error_t err = os_goto_equatorial(g_target_ra_hours, g_target_dec_degrees);
            if (err == OS_ERR_NONE) {
                snprintf(reply, reply_max_len, "0");
            } else {
                snprintf(reply, reply_max_len, "1");
            }
        } else if (strncmp(cmd, ":Mg", 3) == 0) {
            char dir_c = cmd[3];
            int duration = 0;
            sscanf(cmd + 4, "%d", &duration);
            os_guide_direction_t gdir = OS_GUIDE_EAST;
            if (dir_c == 'e') gdir = OS_GUIDE_EAST;
            else if (dir_c == 'w') gdir = OS_GUIDE_WEST;
            else if (dir_c == 'n') gdir = OS_GUIDE_NORTH;
            else if (dir_c == 's') gdir = OS_GUIDE_SOUTH;
            os_guide_pulse(gdir, duration);
        } else if (strcmp(cmd, ":Me#") == 0) { os_manual_move(OS_AXIS_RA, OS_DIR_POSITIVE, OS_SPEED_FIND); }
        else if (strcmp(cmd, ":Mw#") == 0) { os_manual_move(OS_AXIS_RA, OS_DIR_NEGATIVE, OS_SPEED_FIND); }
        else if (strcmp(cmd, ":Mn#") == 0) { os_manual_move(OS_AXIS_DEC, OS_DIR_POSITIVE, OS_SPEED_FIND); }
        else if (strcmp(cmd, ":Ms#") == 0) { os_manual_move(OS_AXIS_DEC, OS_DIR_NEGATIVE, OS_SPEED_FIND); }
    } else if (cmd[1] == 'Q') {
        /* Stop commands */
        if (strcmp(cmd, ":Qe#") == 0 || strcmp(cmd, ":Qw#") == 0) { os_manual_stop(OS_AXIS_RA); }
        else if (strcmp(cmd, ":Qn#") == 0 || strcmp(cmd, ":Qs#") == 0) { os_manual_stop(OS_AXIS_DEC); }
        else if (strcmp(cmd, ":Q#") == 0) { os_goto_abort(); }
    } else if (cmd[1] == 'h') {
        /* Park commands */
        if (strcmp(cmd, ":hP#") == 0) { os_park(); }
        else if (strcmp(cmd, ":hO#") == 0) { os_unpark(); }
    }

    return OS_ERR_NONE;
}

os_error_t os_loop_iteration(void) {
    if (g_state == OS_STATE_FAULT) {
        return OS_ERR_HARDWARE;
    }

    /* 1. Poll Communication Channels */
    for (uint8_t c = 0; c < OS_MAX_CHANNELS; c++) {
        while (os_hal_comm_available(c) > 0) {
            int ch = os_hal_comm_read(c);
            if (ch < 0) break;
            char byte_in = (char)ch;
            if (g_channel_cmd_len[c] < OS_MAX_COMMAND_LENGTH - 1) {
                g_channel_cmd_buf[c][g_channel_cmd_len[c]++] = byte_in;
                g_channel_cmd_buf[c][g_channel_cmd_len[c]] = '\0';
            }
            if (byte_in == '#') {
                char reply_buf[OS_MAX_REPLY_LENGTH] = {0};
                os_process_command(c, g_channel_cmd_buf[c], reply_buf, sizeof(reply_buf));
                if (reply_buf[0] != '\0') {
                    os_hal_comm_write(c, reply_buf, strlen(reply_buf));
                }
                g_channel_cmd_len[c] = 0;
            }
        }
    }

    /* 2. Check FDIR Limit Switch Status */
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        if (g_goto_active || g_state == OS_STATE_GOTO || g_state == OS_STATE_MANUAL_MOTION) {
            os_hal_motor_set_frequency(OS_AXIS_RA, 0.0);
            os_hal_motor_set_frequency(OS_AXIS_DEC, 0.0);
            os_hal_buzzer_beep(500, 3);
            g_goto_active = false;
            g_park_active = false;
            g_state = OS_STATE_FAULT;
            return OS_ERR_HARDWARE;
        }
    }

    /* 3. Handle Motion Processing */
    if (g_goto_active || g_state == OS_STATE_GOTO) {
        int32_t diff0 = g_goto_target_steps[OS_AXIS_RA] - g_motor_position[OS_AXIS_RA];
        int32_t diff1 = g_goto_target_steps[OS_AXIS_DEC] - g_motor_position[OS_AXIS_DEC];
        int32_t step_speed = 100; /* Simulated motion step per loop tick */

        if (labs(diff0) <= step_speed) {
            g_motor_position[OS_AXIS_RA] = g_goto_target_steps[OS_AXIS_RA];
        } else {
            g_motor_position[OS_AXIS_RA] += (diff0 > 0) ? step_speed : -step_speed;
        }

        if (labs(diff1) <= step_speed) {
            g_motor_position[OS_AXIS_DEC] = g_goto_target_steps[OS_AXIS_DEC];
        } else {
            g_motor_position[OS_AXIS_DEC] += (diff1 > 0) ? step_speed : -step_speed;
        }

        if (g_motor_position[OS_AXIS_RA] == g_goto_target_steps[OS_AXIS_RA] &&
            g_motor_position[OS_AXIS_DEC] == g_goto_target_steps[OS_AXIS_DEC]) {
            g_goto_active = false;
            os_hal_buzzer_beep(200, 1);
            if (g_park_active) {
                g_park_active = false;
                g_state = OS_STATE_PARKED;
                os_hal_motor_enable(OS_AXIS_RA, false);
                os_hal_motor_enable(OS_AXIS_DEC, false);
            } else {
                g_state = OS_STATE_IDLE_TRACKING;
            }
        }
    } else if (g_state == OS_STATE_MANUAL_MOTION) {
        int32_t manual_step = (g_manual_speed[OS_AXIS_RA] + 1) * 20;
        if (g_manual_active[OS_AXIS_RA]) {
            g_motor_position[OS_AXIS_RA] += (g_manual_dir[OS_AXIS_RA] == OS_DIR_POSITIVE) ? manual_step : -manual_step;
        }
        if (g_manual_active[OS_AXIS_DEC]) {
            g_motor_position[OS_AXIS_DEC] += (g_manual_dir[OS_AXIS_DEC] == OS_DIR_POSITIVE) ? manual_step : -manual_step;
        }
    } else if (g_state == OS_STATE_IDLE_TRACKING) {
        /* Baseline Sidereal / Custom Celestial Tracking */
        double track_rate = SIDEREAL_DEG_PER_SEC * STEPS_PER_DEGREE;
        if (g_tracking_rate == OS_TRACK_LUNAR) track_rate *= 0.966;
        else if (g_tracking_rate == OS_TRACK_SOLAR) track_rate *= 0.997;
        else if (g_tracking_rate == OS_TRACK_CUSTOM) track_rate = g_custom_tracking_hz;

        /* Apply PEC correction if enabled */
        if (g_pec_enabled) {
            track_rate += g_pec_table[0] * (STEPS_PER_DEGREE / 3600.0);
        }

        os_hal_motor_set_frequency(OS_AXIS_RA, track_rate);
        g_motor_position[OS_AXIS_RA] += (int32_t)lround(track_rate * 0.01);
    }

    /* 4. Process Priority Guide Pulses (DEC before RA) */
    if (g_guide_dec_remaining_ms > 0) {
        int32_t pulse_step = (g_guide_dec_dir == OS_GUIDE_NORTH) ? 1 : -1;
        g_motor_position[OS_AXIS_DEC] += pulse_step;
        g_guide_dec_remaining_ms = (g_guide_dec_remaining_ms > 10) ? g_guide_dec_remaining_ms - 10 : 0;
    } else if (g_guide_ra_remaining_ms > 0) {
        int32_t pulse_step = (g_guide_ra_dir == OS_GUIDE_EAST) ? 1 : -1;
        g_motor_position[OS_AXIS_RA] += pulse_step;
        g_guide_ra_remaining_ms = (g_guide_ra_remaining_ms > 10) ? g_guide_ra_remaining_ms - 10 : 0;
    }

    return OS_ERR_NONE;
}

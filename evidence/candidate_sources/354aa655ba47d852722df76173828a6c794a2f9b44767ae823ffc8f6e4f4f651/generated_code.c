#include "generated_code.h"

/* Helper Constants */
#define STEPS_PER_DEGREE 3600.0

/* Internal Mock HAL State */
typedef struct {
    bool initialized;
    bool enabled;
    bool direction_forward;
    double frequency_hz;
    int32_t position_steps;
    bool fault;
} mock_motor_t;

typedef struct {
    char rx_buf[256];
    size_t rx_head;
    size_t rx_tail;
    char tx_buf[256];
    size_t tx_len;
    bool enabled;
} mock_comm_channel_t;

static mock_motor_t g_mock_motors[OS_MAX_AXES];
static mock_comm_channel_t g_mock_comm[OS_MAX_CHANNELS];
static bool g_mock_limit_triggered[OS_MAX_AXES];
static os_site_info_t g_mock_gps_site;
static uint32_t g_mock_rtc_epoch;
static uint8_t g_nvm_calibration[OS_NVM_CALIBRATION_SIZE_BYTES];
static uint8_t g_nvm_config[OS_NVM_CONFIG_SIZE_BYTES];

/* Internal System Context */
typedef struct {
    os_state_t state;
    os_tracking_rate_t tracking_rate;
    double custom_tracking_rate_hz;
    bool pec_enabled;
    double pec_table[361];
    
    /* Target Coordinates & Steps */
    double target_ra_hours;
    double target_dec_degrees;
    int32_t target_steps[OS_MAX_AXES];
    
    /* Alignment Data */
    uint32_t align_star_count;
    double align_ra_hours[16];
    double align_dec_degrees[16];
    int32_t align_steps_ra[16];
    int32_t align_steps_dec[16];
    bool align_residual_calculated;
    double align_matrix[2][2];
    double align_offset[2];
    
    /* Motion states */
    bool is_manual_moving[OS_MAX_AXES];
    os_direction_t manual_dir[OS_MAX_AXES];
    os_speed_rate_t manual_speed[OS_MAX_AXES];
    
    /* Guide pulse */
    uint32_t guide_duration_ms;
    os_guide_direction_t guide_direction;
    
    /* Site info */
    os_site_info_t current_site;
} onstep_ctx_t;

static onstep_ctx_t g_ctx;

/* HAL Implementation (Host Mock) */
void os_hal_motor_init(uint8_t axis) {
    if (axis < OS_MAX_AXES) {
        g_mock_motors[axis].initialized = true;
        g_mock_motors[axis].enabled = false;
        g_mock_motors[axis].direction_forward = true;
        g_mock_motors[axis].frequency_hz = 0.0;
        g_mock_motors[axis].position_steps = 0;
        g_mock_motors[axis].fault = false;
    }
}

void os_hal_motor_set_frequency(uint8_t axis, double frequency_hz) {
    if (axis < OS_MAX_AXES) {
        g_mock_motors[axis].frequency_hz = frequency_hz;
    }
}

void os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (axis < OS_MAX_AXES) {
        g_mock_motors[axis].direction_forward = forward;
    }
}

void os_hal_motor_enable(uint8_t axis, bool enable) {
    if (axis < OS_MAX_AXES) {
        g_mock_motors[axis].enabled = enable;
    }
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (axis < OS_MAX_AXES) {
        return g_mock_motors[axis].position_steps;
    }
    return 0;
}

void os_hal_timer_motor_init(void) {
    /* Timer initialized for motor pulses */
}

void os_hal_gps_init(void) {
    g_mock_gps_site.valid = false;
    g_mock_gps_site.latitude_degrees = 0.0;
    g_mock_gps_site.longitude_degrees = 0.0;
    g_mock_gps_site.elevation_metres = 0.0;
    g_mock_gps_site.utc_epoch_seconds = 0;
}

bool os_hal_gps_poll(os_site_info_t *site) {
    if (!site) return false;
    *site = g_mock_gps_site;
    return g_mock_gps_site.valid;
}

void os_hal_rtc_init(void) {
    g_mock_rtc_epoch = 1609459200; /* Default 2021-01-01 */
}

bool os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (!utc_epoch_seconds) return false;
    *utc_epoch_seconds = g_mock_rtc_epoch;
    return true;
}

bool os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    g_mock_rtc_epoch = utc_epoch_seconds;
    return true;
}

void os_hal_limit_init(void) {
    for (int i = 0; i < OS_MAX_AXES; i++) {
        g_mock_limit_triggered[i] = false;
    }
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (axis >= OS_MAX_AXES) {
        return true; /* Invalid axis fails safe to true */
    }
    return g_mock_limit_triggered[axis];
}

void os_hal_buzzer_beep(uint32_t duration_ms, uint32_t count) {
    (void)duration_ms;
    (void)count;
}

void os_hal_comm_init(uint8_t channel) {
    if (channel < OS_MAX_CHANNELS) {
        g_mock_comm[channel].enabled = true;
        g_mock_comm[channel].rx_head = 0;
        g_mock_comm[channel].rx_tail = 0;
        g_mock_comm[channel].tx_len = 0;
    }
}

size_t os_hal_comm_available(uint8_t channel) {
    if (channel >= OS_MAX_CHANNELS || !g_mock_comm[channel].enabled) return 0;
    if (g_mock_comm[channel].rx_head >= g_mock_comm[channel].rx_tail) {
        return g_mock_comm[channel].rx_head - g_mock_comm[channel].rx_tail;
    }
    return sizeof(g_mock_comm[channel].rx_buf) - g_mock_comm[channel].rx_tail + g_mock_comm[channel].rx_head;
}

int os_hal_comm_read(uint8_t channel) {
    if (channel >= OS_MAX_CHANNELS || os_hal_comm_available(channel) == 0) return -1;
    char c = g_mock_comm[channel].rx_buf[g_mock_comm[channel].rx_tail];
    g_mock_comm[channel].rx_tail = (g_mock_comm[channel].rx_tail + 1) % sizeof(g_mock_comm[channel].rx_buf);
    return (unsigned char)c;
}

void os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (channel >= OS_MAX_CHANNELS || !data) return;
    size_t available_space = sizeof(g_mock_comm[channel].tx_buf) - g_mock_comm[channel].tx_len;
    size_t to_write = length < available_space ? length : available_space;
    memcpy(g_mock_comm[channel].tx_buf + g_mock_comm[channel].tx_len, data, to_write);
    g_mock_comm[channel].tx_len += to_write;
}

void os_hal_nvm_init(void) {
    /* NVM initialized */
}

bool os_hal_nvm_read(uint32_t offset, void *data, size_t length) {
    if (!data) return false;
    if (offset + length <= OS_NVM_CALIBRATION_SIZE_BYTES) {
        memcpy(data, g_nvm_calibration + offset, length);
        return true;
    }
    return false;
}

bool os_hal_nvm_write(uint32_t offset, const void *data, size_t length) {
    if (!data) return false;
    if (offset + length <= OS_NVM_CALIBRATION_SIZE_BYTES) {
        memcpy(g_nvm_calibration + offset, data, length);
        return true;
    }
    return false;
}

/* Mock Helpers */
void mock_set_motor_fault(uint8_t axis, bool fault) {
    if (axis < OS_MAX_AXES) {
        g_mock_motors[axis].fault = fault;
    }
}

void mock_set_limit_triggered(uint8_t axis, bool triggered) {
    if (axis < OS_MAX_AXES) {
        g_mock_limit_triggered[axis] = triggered;
    }
}

void mock_inject_gps(const os_site_info_t *site) {
    if (site) {
        g_mock_gps_site = *site;
    }
}

void mock_inject_comm_rx(uint8_t channel, const char *data, size_t length) {
    if (channel >= OS_MAX_CHANNELS || !data) return;
    for (size_t i = 0; i < length; i++) {
        size_t next_head = (g_mock_comm[channel].rx_head + 1) % sizeof(g_mock_comm[channel].rx_buf);
        if (next_head != g_mock_comm[channel].rx_tail) {
            g_mock_comm[channel].rx_buf[g_mock_comm[channel].rx_head] = data[i];
            g_mock_comm[channel].rx_head = next_head;
        }
    }
}

size_t mock_get_comm_tx(uint8_t channel, char *buffer, size_t max_len) {
    if (channel >= OS_MAX_CHANNELS || !buffer) return 0;
    size_t to_copy = g_mock_comm[channel].tx_len < max_len ? g_mock_comm[channel].tx_len : max_len;
    memcpy(buffer, g_mock_comm[channel].tx_buf, to_copy);
    memmove(g_mock_comm[channel].tx_buf, g_mock_comm[channel].tx_buf + to_copy, g_mock_comm[channel].tx_len - to_copy);
    g_mock_comm[channel].tx_len -= to_copy;
    return to_copy;
}

/* Core Logic Helper */
static void reset_runtime_state(void) {
    g_ctx.state = OS_STATE_IDLE_TRACKING;
    g_ctx.tracking_rate = OS_TRACK_SIDEREAL;
    g_ctx.custom_tracking_rate_hz = 0.0;
    g_ctx.pec_enabled = false;
    memset(g_ctx.pec_table, 0, sizeof(g_ctx.pec_table));
    
    g_ctx.target_ra_hours = 0.0;
    g_ctx.target_dec_degrees = 0.0;
    g_ctx.target_steps[0] = 0;
    g_ctx.target_steps[1] = 0;
    
    g_ctx.align_star_count = 0;
    g_ctx.align_residual_calculated = false;
    g_ctx.align_matrix[0][0] = 1.0;
    g_ctx.align_matrix[0][1] = 0.0;
    g_ctx.align_matrix[1][0] = 0.0;
    g_ctx.align_matrix[1][1] = 1.0;
    g_ctx.align_offset[0] = 0.0;
    g_ctx.align_offset[1] = 0.0;
    
    for (int i = 0; i < OS_MAX_AXES; i++) {
        g_ctx.is_manual_moving[i] = false;
        g_ctx.manual_dir[i] = OS_DIR_POSITIVE;
        g_ctx.manual_speed[i] = OS_SPEED_GUIDE;
    }
    
    g_ctx.guide_duration_ms = 0;
    g_ctx.guide_direction = OS_GUIDE_EAST;
}

/* API Implementation */
os_error_t os_init(void) {
    reset_runtime_state();
    
    /* Hardware Initialization Sequence */
    os_hal_nvm_init();
    for (uint8_t i = 0; i < OS_MAX_CHANNELS; i++) {
        os_hal_comm_init(i);
    }
    
    for (uint8_t a = 0; a < OS_MAX_AXES; a++) {
        os_hal_motor_init(a);
        os_hal_motor_enable(a, true);
        if (g_mock_motors[a].fault) {
            g_ctx.state = OS_STATE_FAULT;
            return OS_ERR_HARDWARE;
        }
    }
    
    os_hal_gps_init();
    os_hal_rtc_init();
    os_hal_limit_init();
    os_hal_timer_motor_init();
    
    if (!os_hal_gps_poll(&g_ctx.current_site)) {
        g_ctx.current_site.valid = false;
        uint32_t rtc_time = 0;
        os_hal_rtc_read(&rtc_time);
        g_ctx.current_site.utc_epoch_seconds = rtc_time;
        g_ctx.current_site.latitude_degrees = 0.0;
        g_ctx.current_site.longitude_degrees = 0.0;
        g_ctx.current_site.elevation_metres = 0.0;
    }
    
    g_ctx.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_set_tracking_rate(os_tracking_rate_t rate) {
    if (rate < OS_TRACK_SIDEREAL || rate > OS_TRACK_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_ctx.tracking_rate = rate;
    return OS_ERR_NONE;
}

os_error_t os_set_custom_tracking_rate(double rate_hz) {
    if (rate_hz < 0.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_ctx.custom_tracking_rate_hz = rate_hz;
    g_ctx.tracking_rate = OS_TRACK_CUSTOM;
    return OS_ERR_NONE;
}

os_error_t os_goto_equatorial(double ra_hours, double dec_degrees) {
    /* Rule: Param validation priority over state check */
    if (ra_hours < 0.0 || ra_hours > 24.0 || dec_degrees < -90.0 || dec_degrees > 90.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    
    if (g_ctx.state == OS_STATE_PARKED || g_ctx.state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    
    /* Limit check before starting motion */
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_INVALID_STATE;
    }
    
    g_ctx.target_ra_hours = ra_hours;
    g_ctx.target_dec_degrees = dec_degrees;
    
    g_ctx.target_steps[OS_AXIS_RA] = (int32_t)(ra_hours * 15.0 * STEPS_PER_DEGREE);
    g_ctx.target_steps[OS_AXIS_DEC] = (int32_t)(dec_degrees * STEPS_PER_DEGREE);
    
    g_ctx.state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (g_ctx.state == OS_STATE_GOTO || g_ctx.state == OS_STATE_MANUAL_MOTION) {
        os_hal_motor_set_frequency(OS_AXIS_RA, 0.0);
        os_hal_motor_set_frequency(OS_AXIS_DEC, 0.0);
        g_ctx.state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_guide_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_GUIDE_EAST || direction > OS_GUIDE_SOUTH || duration_ms == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_ctx.guide_direction = direction;
    g_ctx.guide_duration_ms = duration_ms;
    return OS_ERR_NONE;
}

os_error_t os_align_add_star(double ra_hours, double dec_degrees) {
    if (ra_hours < 0.0 || ra_hours > 24.0 || dec_degrees < -90.0 || dec_degrees > 90.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g_ctx.align_star_count >= 16) {
        return OS_ERR_INVALID_STATE;
    }
    uint32_t idx = g_ctx.align_star_count;
    g_ctx.align_ra_hours[idx] = ra_hours;
    g_ctx.align_dec_degrees[idx] = dec_degrees;
    g_ctx.align_steps_ra[idx] = os_hal_motor_get_position(OS_AXIS_RA);
    g_ctx.align_steps_dec[idx] = os_hal_motor_get_position(OS_AXIS_DEC);
    g_ctx.align_star_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1_STAR || mode > OS_ALIGN_N_STAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    
    uint32_t min_stars = 1;
    if (mode == OS_ALIGN_2_STAR) min_stars = 2;
    else if (mode == OS_ALIGN_3_STAR || mode == OS_ALIGN_N_STAR) min_stars = 3;
    
    if (g_ctx.align_star_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }
    
    if (mode == OS_ALIGN_3_STAR) {
        /* Check collinearity using double precision determinant */
        double x0 = g_ctx.align_ra_hours[0] * 15.0;
        double y0 = g_ctx.align_dec_degrees[0];
        double x1 = g_ctx.align_ra_hours[1] * 15.0;
        double y1 = g_ctx.align_dec_degrees[1];
        double x2 = g_ctx.align_ra_hours[2] * 15.0;
        double y2 = g_ctx.align_dec_degrees[2];
        
        double det = x0 * (y1 - y2) - y0 * (x1 - x2) + (x1 * y2 - x2 * y1);
        if (fabs(det) < 1e-6) {
            return OS_ERR_INVALID_STATE; /* Collinear data */
        }
    }
    
    g_ctx.align_residual_calculated = true;
    os_hal_nvm_write(0, g_ctx.align_matrix, sizeof(g_ctx.align_matrix));
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (os_hal_limit_is_triggered(OS_AXIS_RA) || os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_INVALID_STATE;
    }
    g_ctx.target_steps[OS_AXIS_RA] = 0;
    g_ctx.target_steps[OS_AXIS_DEC] = 0;
    g_ctx.state = OS_STATE_PARKED;
    os_hal_motor_enable(OS_AXIS_RA, false);
    os_hal_motor_enable(OS_AXIS_DEC, false);
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (g_ctx.state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    os_hal_motor_enable(OS_AXIS_RA, true);
    os_hal_motor_enable(OS_AXIS_DEC, true);
    for (uint8_t ch = 0; ch < OS_MAX_CHANNELS; ch++) {
        os_hal_comm_init(ch);
    }
    uint32_t rtc_sec = 0;
    os_hal_rtc_read(&rtc_sec);
    g_ctx.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_manual_move(uint8_t axis, os_direction_t dir, os_speed_rate_t speed) {
    if (axis >= OS_MAX_AXES || dir > OS_DIR_NEGATIVE || speed > OS_SPEED_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_INVALID_STATE;
    }
    if (g_ctx.state == OS_STATE_PARKED || g_ctx.state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    g_ctx.is_manual_moving[axis] = true;
    g_ctx.manual_dir[axis] = dir;
    g_ctx.manual_speed[axis] = speed;
    g_ctx.state = OS_STATE_MANUAL_MOTION;
    
    os_hal_motor_set_direction(axis, dir == OS_DIR_POSITIVE);
    double freq = 100.0 * (speed + 1);
    os_hal_motor_set_frequency(axis, freq);
    return OS_ERR_NONE;
}

os_error_t os_manual_stop(uint8_t axis) {
    if (axis >= OS_MAX_AXES) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_ctx.is_manual_moving[axis] = false;
    os_hal_motor_set_frequency(axis, 0.0);
    
    bool any_moving = false;
    for (int a = 0; a < OS_MAX_AXES; a++) {
        if (g_ctx.is_manual_moving[a]) any_moving = true;
    }
    if (!any_moving && g_ctx.state == OS_STATE_MANUAL_MOTION) {
        g_ctx.state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *is_moving) {
    if (!is_moving) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *is_moving = (g_ctx.state == OS_STATE_GOTO || g_ctx.state == OS_STATE_MANUAL_MOTION);
    return OS_ERR_NONE;
}

os_error_t os_get_status(os_system_status_t *status) {
    if (!status) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    status->state = g_ctx.state;
    status->moving = (g_ctx.state == OS_STATE_GOTO || g_ctx.state == OS_STATE_MANUAL_MOTION);
    status->gps_locked = g_ctx.current_site.valid;
    status->motor_steps[OS_AXIS_RA] = os_hal_motor_get_position(OS_AXIS_RA);
    status->motor_steps[OS_AXIS_DEC] = os_hal_motor_get_position(OS_AXIS_DEC);
    status->current_ra_hours = (double)status->motor_steps[OS_AXIS_RA] / (15.0 * STEPS_PER_DEGREE);
    status->current_dec_degrees = (double)status->motor_steps[OS_AXIS_DEC] / STEPS_PER_DEGREE;
    return OS_ERR_NONE;
}

os_error_t os_pec_set_correction(double worm_phase_deg, double arcsec_error) {
    if (worm_phase_deg < 0.0 || worm_phase_deg > 360.0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int idx = (int)worm_phase_deg;
    if (idx >= 0 && idx <= 360) {
        g_ctx.pec_table[idx] = arcsec_error;
    }
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    g_ctx.pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_process_command(uint8_t channel, const char *cmd, char *reply, size_t reply_max_len) {
    if (!cmd || !reply) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (reply_max_len == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    
    reply[0] = '\0';
    
    /* Format LX200 command parsing */
    if (strcmp(cmd, ":GR#") == 0) {
        snprintf(reply, reply_max_len, "%02d:%02d:%02d#", (int)g_ctx.target_ra_hours, 0, 0);
    } else if (strcmp(cmd, ":GD#") == 0) {
        snprintf(reply, reply_max_len, "%+03d*00'00#", (int)g_ctx.target_dec_degrees);
    } else if (strcmp(cmd, ":GVP#") == 0) {
        snprintf(reply, reply_max_len, "OnStep#");
    } else if (strcmp(cmd, ":MS#") == 0) {
        os_error_t err = os_goto_equatorial(g_ctx.target_ra_hours, g_ctx.target_dec_degrees);
        if (err == OS_ERR_NONE) {
            snprintf(reply, reply_max_len, "0");
        } else {
            snprintf(reply, reply_max_len, "1Target Error#");
        }
    } else if (strncmp(cmd, ":Sr", 3) == 0) {
        int h = 0, m = 0, s = 0;
        if (sscanf(cmd + 3, "%d:%d:%d", &h, &m, &s) >= 1) {
            g_ctx.target_ra_hours = h + m / 60.0 + s / 3600.0;
            snprintf(reply, reply_max_len, "1");
        } else {
            return OS_ERR_COMMAND_FORMAT;
        }
    } else if (strncmp(cmd, ":Sd", 3) == 0) {
        int d = 0, m = 0, s = 0;
        if (sscanf(cmd + 3, "%d*%d:%d", &d, &m, &s) >= 1 || sscanf(cmd + 3, "%d*%d'%d", &d, &m, &s) >= 1) {
            g_ctx.target_dec_degrees = (d >= 0 ? 1 : -1) * (abs(d) + m / 60.0 + s / 3600.0);
            snprintf(reply, reply_max_len, "1");
        } else {
            return OS_ERR_COMMAND_FORMAT;
        }
    } else if (strcmp(cmd, ":hP#") == 0) {
        os_park();
        snprintf(reply, reply_max_len, "0");
    } else if (strcmp(cmd, ":hO#") == 0) {
        os_unpark();
        snprintf(reply, reply_max_len, "0");
    } else if (strcmp(cmd, ":Q#") == 0) {
        os_goto_abort();
        snprintf(reply, reply_max_len, "0");
    } else {
        snprintf(reply, reply_max_len, "0");
    }
    
    return OS_ERR_NONE;
}

os_error_t os_loop_iteration(void) {
    /* 1. Check Comm Channels */
    for (uint8_t ch = 0; ch < OS_MAX_CHANNELS; ch++) {
        if (os_hal_comm_available(ch) > 0) {
            char cmdbuf[OS_MAX_COMMAND_LENGTH];
            size_t idx = 0;
            while (os_hal_comm_available(ch) > 0 && idx < OS_MAX_COMMAND_LENGTH - 1) {
                int c = os_hal_comm_read(ch);
                if (c < 0) break;
                cmdbuf[idx++] = (char)c;
                if (c == '#') break;
            }
            cmdbuf[idx] = '\0';
            if (idx > 0 && cmdbuf[idx - 1] == '#') {
                char reply[OS_MAX_REPLY_LENGTH];
                if (os_process_command(ch, cmdbuf, reply, sizeof(reply)) == OS_ERR_NONE) {
                    os_hal_comm_write(ch, reply, strlen(reply));
                }
            }
        }
    }
    
    /* 2. Monitor Limit Switches & Faults */
    for (uint8_t a = 0; a < OS_MAX_AXES; a++) {
        if (os_hal_limit_is_triggered(a) || g_mock_motors[a].fault) {
            os_hal_motor_set_frequency(a, 0.0);
            if (g_ctx.state == OS_STATE_GOTO || g_ctx.state == OS_STATE_MANUAL_MOTION) {
                g_ctx.state = OS_STATE_FAULT;
                os_hal_buzzer_beep(500, 2);
            }
        }
    }
    
    /* 3. Advance Goto Motion Step-by-Step */
    if (g_ctx.state == OS_STATE_GOTO) {
        bool ra_reached = false;
        bool dec_reached = false;
        
        int32_t cur_ra = g_mock_motors[OS_AXIS_RA].position_steps;
        int32_t target_ra = g_ctx.target_steps[OS_AXIS_RA];
        if (abs(cur_ra - target_ra) <= 100) {
            g_mock_motors[OS_AXIS_RA].position_steps = target_ra;
            ra_reached = true;
        } else {
            g_mock_motors[OS_AXIS_RA].position_steps += (target_ra > cur_ra ? 100 : -100);
        }
        
        int32_t cur_dec = g_mock_motors[OS_AXIS_DEC].position_steps;
        int32_t target_dec = g_ctx.target_steps[OS_AXIS_DEC];
        if (abs(cur_dec - target_dec) <= 100) {
            g_mock_motors[OS_AXIS_DEC].position_steps = target_dec;
            dec_reached = true;
        } else {
            g_mock_motors[OS_AXIS_DEC].position_steps += (target_dec > cur_dec ? 100 : -100);
        }
        
        if (ra_reached && dec_reached) {
            g_ctx.state = OS_STATE_IDLE_TRACKING;
            os_hal_buzzer_beep(200, 1);
        }
    }
    
    return OS_ERR_NONE;
}

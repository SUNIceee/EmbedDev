#include "6_generated_code.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define OS_NVM_TOTAL_BYTES (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)
#define OS_COMM_BUFFER_SIZE 256u
#define OS_MOTOR_FREQ_MAX_HZ 200000u
#define OS_MOVE_STEP_HZ 1000u
#define OS_GOTO_SMALL_DELTA_STEPS 2
#define OS_ALIGN_RESIDUAL_LIMIT_ARCSEC 300.0
#define OS_MANUAL_TIMEOUT_TICKS 10000u
#define OS_CAL_NVM_MAGIC0 0x4Fu
#define OS_CAL_NVM_MAGIC1 0x4Eu
#define OS_CAL_NVM_MAGIC2 0x43u
#define OS_CAL_NVM_MAGIC3 0x41u
#define OS_CAL_NVM_RECORD_SIZE 53u

typedef struct {
    char rx[OS_COMM_BUFFER_SIZE];
    uint16_t rx_head;
    uint16_t rx_tail;
    uint16_t rx_count;
    char tx[OS_COMM_BUFFER_SIZE];
    uint16_t tx_head;
    uint16_t tx_tail;
    uint16_t tx_count;
    bool initialized;
} os_comm_channel_t;

static os_state_t s_state = OS_STATE_INITIALIZING;
static bool s_init_complete;
static bool s_motor_initialized[2];
static bool s_motor_enabled[2];
static uint32_t s_motor_freq[2];
static bool s_motor_dir[2];
static int32_t s_motor_pos[2];

static bool s_gps_initialized;
static bool s_gps_has_fix;
static os_site_info_t s_gps_site;
static bool s_rtc_initialized;
static uint32_t s_rtc_epoch;
static bool s_limit_initialized;
static bool s_limit[2];
static bool s_buzzer_scheduled;
static bool s_timer_initialized;

static os_comm_channel_t s_comm[4];
static char s_loop_cmd[4][OS_MAX_COMMAND_LENGTH + 2u];
static uint16_t s_loop_len[4];

static bool s_nvm_initialized;
static uint8_t s_nvm[OS_NVM_TOTAL_BYTES];

static os_track_rate_t s_track_rate = OS_TRACK_RATE_SIDEREAL;
static float s_custom_track_factor = 1.0f;
static bool s_tracking_active;
static os_mount_type_t s_mount_type = OS_MOUNT_EQUATORIAL;
static os_site_info_t s_site;
static bool s_gps_locked;
static bool s_rtc_valid;

static bool s_goto_active;
static int32_t s_goto_target[2];
static bool s_park_active;
static int32_t s_park_target[2];
static bool s_park_position_set;
static os_equatorial_coord_t s_park_position;
static bool s_manual_active;
static os_direction_t s_manual_direction;
static os_speed_level_t s_manual_speed;
static float s_manual_custom_speed_arcsec = 100.0f;
static uint16_t s_manual_remaining;

static os_guide_pulse_t s_guide;
static uint32_t s_guide_elapsed;
static uint8_t s_guide_axis;
static bool s_guide_forward;
static float s_guide_rate_fraction = 0.5f;

static bool s_pec_enabled;
static os_pec_table_t s_pec_table;

static os_align_mode_t s_align_mode = OS_ALIGN_3STAR;
static uint8_t s_align_count;
static double s_align_ra[OS_CALIBRATION_MAX_STARS];
static double s_align_dec[OS_CALIBRATION_MAX_STARS];
static int32_t s_align_mra[OS_CALIBRATION_MAX_STARS];
static int32_t s_align_mdec[OS_CALIBRATION_MAX_STARS];
static bool s_residual_computed;
static float s_residual_arcsec;
static bool s_calibration_valid;
static double s_calib[2][3];
static bool s_align_prior_calib_valid;

static bool axis_valid(uint8_t axis) {
    return axis <= 1u;
}

static bool channel_valid(uint8_t channel) {
    return channel <= OS_CHANNEL_ETHERNET;
}

static void reply_text(char *buf, size_t cap, size_t *len, const char *txt) {
    if (buf == NULL || cap == 0u) {
        if (len != NULL) {
            *len = 0u;
        }
        return;
    }
    size_t n = strlen(txt);
    if (n >= cap) {
        n = cap - 1u;
    }
    memcpy(buf, txt, n);
    buf[n] = '\0';
    if (len != NULL) {
        *len = n;
    }
}

static void reply_float(char *buf, size_t cap, size_t *len, float value) {
    char tmp[32];
    int n = snprintf(tmp, sizeof(tmp), "%.4f", (double)value);
    if (n < 0) {
        n = 0;
    }
    if ((size_t)n >= cap) {
        n = (int)cap - 1;
    }
    if (n < 0) {
        n = 0;
    }
    memcpy(buf, tmp, (size_t)n);
    buf[n] = '\0';
    if (len != NULL) {
        *len = (size_t)n;
    }
}

static uint32_t tracking_frequency_hz(void) {
    double base = (double)OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
    if (s_track_rate == OS_TRACK_RATE_LUNAR) {
        base *= (double)OS_LUNAR_RATE_FACTOR;
    } else if (s_track_rate == OS_TRACK_RATE_SOLAR) {
        base *= (double)OS_SOLAR_RATE_FACTOR;
    } else if (s_track_rate == OS_TRACK_RATE_CUSTOM) {
        base *= (double)s_custom_track_factor;
    }
    if (base < 0.0) {
        base = 0.0;
    }
    if (base > (double)OS_MOTOR_FREQ_MAX_HZ) {
        base = (double)OS_MOTOR_FREQ_MAX_HZ;
    }
    return (uint32_t)base;
}

static void tracking_apply(void) {
    if (s_tracking_active && (s_state == OS_STATE_IDLE_TRACKING || s_state == OS_STATE_GOTO)) {
        os_hal_motor_set_direction(0u, true);
        os_hal_motor_set_frequency(0u, tracking_frequency_hz());
        if (s_mount_type == OS_MOUNT_ALTAZ) {
            os_hal_motor_set_direction(1u, true);
            os_hal_motor_set_frequency(1u, tracking_frequency_hz());
        } else {
            os_hal_motor_set_frequency(1u, 0u);
        }
    } else {
        os_hal_motor_set_frequency(0u, 0u);
        os_hal_motor_set_frequency(1u, 0u);
    }
}

static uint8_t manual_axis_for_direction(os_direction_t direction) {
    if (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH) {
        return 1u;
    }
    return 0u;
}

static bool manual_forward_for_direction(os_direction_t direction) {
    return (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_EAST);
}

static uint32_t manual_frequency_hz(void) {
    uint32_t freq = 100u;
    if (s_manual_speed == OS_SPEED_SLOW) {
        freq = 50u;
    } else if (s_manual_speed == OS_SPEED_MEDIUM) {
        freq = 200u;
    } else if (s_manual_speed == OS_SPEED_FAST) {
        freq = OS_MOVE_STEP_HZ;
    } else if (s_manual_speed == OS_SPEED_CUSTOM) {
        double d = (double)s_manual_custom_speed_arcsec;
        if (d < 0.0) {
            d = 0.0;
        }
        if (d > (double)OS_MOTOR_FREQ_MAX_HZ) {
            d = (double)OS_MOTOR_FREQ_MAX_HZ;
        }
        freq = (uint32_t)d;
    }
    if (freq > OS_MOTOR_FREQ_MAX_HZ) {
        freq = OS_MOTOR_FREQ_MAX_HZ;
    }
    return freq;
}

static void stop_motion_outputs(void) {
    os_hal_motor_set_frequency(0u, 0u);
    os_hal_motor_set_frequency(1u, 0u);
}

static void guide_restore_base_rate(void) {
    if (s_guide.active) {
        os_hal_motor_set_direction(s_guide_axis, false);
        os_hal_motor_set_frequency(s_guide_axis, 0u);
        s_guide.active = false;
        s_guide.duration_ms = 0u;
        s_guide_elapsed = 0u;
    }
    tracking_apply();
}

static void update_guide(void) {
    if (!s_guide.active) {
        return;
    }
    s_guide_elapsed++;
    if (s_guide_elapsed >= s_guide.duration_ms) {
        guide_restore_base_rate();
    }
}

static void motion_tick(void) {
    if (s_goto_active || s_park_active) {
        if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
            s_goto_active = false;
            s_park_active = false;
            stop_motion_outputs();
            os_hal_motor_enable(0u, false);
            os_hal_motor_enable(1u, false);
            s_state = OS_STATE_FAULT;
            return;
        }
        bool arrived = true;
        for (uint8_t axis = 0u; axis <= 1u; axis++) {
            int32_t target = s_goto_active ? s_goto_target[axis] : s_park_target[axis];
            int32_t pos = s_motor_pos[axis];
            if (pos < target) {
                os_hal_motor_set_direction(axis, true);
                os_hal_motor_set_frequency(axis, OS_MOVE_STEP_HZ);
                s_motor_pos[axis]++;
                arrived = false;
            } else if (pos > target) {
                os_hal_motor_set_direction(axis, false);
                os_hal_motor_set_frequency(axis, OS_MOVE_STEP_HZ);
                s_motor_pos[axis]--;
                arrived = false;
            } else {
                os_hal_motor_set_frequency(axis, 0u);
            }
        }
        if (arrived) {
            if (s_goto_active) {
                s_goto_active = false;
                s_state = OS_STATE_IDLE_TRACKING;
                s_tracking_active = true;
                tracking_apply();
                os_hal_buzzer_beep(100u, 1u);
            } else if (s_park_active) {
                s_park_active = false;
                stop_motion_outputs();
                os_hal_motor_enable(0u, false);
                os_hal_motor_enable(1u, false);
                s_tracking_active = false;
                s_state = OS_STATE_PARKED;
                os_hal_buzzer_beep(100u, 1u);
            }
        }
        return;
    }

    if (s_manual_active) {
        uint8_t axis = manual_axis_for_direction(s_manual_direction);
        if (os_hal_limit_is_triggered(axis)) {
            s_manual_active = false;
            stop_motion_outputs();
            s_state = OS_STATE_FAULT;
            return;
        }
        bool forward = manual_forward_for_direction(s_manual_direction);
        os_hal_motor_set_direction(axis, forward);
        os_hal_motor_set_frequency(axis, manual_frequency_hz());
        if (forward) {
            s_motor_pos[axis]++;
        } else {
            s_motor_pos[axis]--;
        }
        if (s_manual_remaining > 0u) {
            s_manual_remaining--;
        }
        if (s_manual_remaining == 0u) {
            s_manual_active = false;
            s_state = OS_STATE_IDLE_TRACKING;
            s_tracking_active = true;
            tracking_apply();
        }
        return;
    }

    for (uint8_t axis = 0u; axis <= 1u; axis++) {
        if (s_motor_enabled[axis] && s_motor_freq[axis] > 0u) {
            if (s_motor_dir[axis]) {
                s_motor_pos[axis]++;
            } else {
                s_motor_pos[axis]--;
            }
        }
    }
}

static void loop_commands(void) {
    char reply[OS_MAX_REPLY_LENGTH];
    size_t reply_len = 0u;
    for (uint8_t ch = 0u; ch <= OS_CHANNEL_ETHERNET; ch++) {
        while (os_hal_comm_available(ch) > 0) {
            char c = os_hal_comm_read(ch);
            if (s_loop_len[ch] < OS_MAX_COMMAND_LENGTH) {
                s_loop_cmd[ch][s_loop_len[ch]++] = c;
            }
            if (c == OS_LX200_CMD_SUFFIX) {
                s_loop_cmd[ch][s_loop_len[ch]] = '\0';
                reply_len = 0u;
                (void)os_command_parse(s_loop_cmd[ch], (size_t)s_loop_len[ch], ch,
                                       reply, sizeof(reply), &reply_len);
                s_loop_len[ch] = 0u;
            } else if (s_loop_len[ch] >= OS_MAX_COMMAND_LENGTH) {
                s_loop_len[ch] = 0u;
            }
        }
    }
}

static bool solve_linear2(double a[2][2], double b[2], double x[2]) {
    double det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if (fabs(det) < 1e-12) {
        return false;
    }
    x[0] = (b[0] * a[1][1] - b[1] * a[0][1]) / det;
    x[1] = (a[0][0] * b[1] - a[1][0] * b[0]) / det;
    return true;
}

static bool solve_linear3(double a[3][3], double b[3], double x[3]) {
    double m[3][4];
    for (uint8_t r = 0u; r < 3u; r++) {
        m[r][0] = a[r][0];
        m[r][1] = a[r][1];
        m[r][2] = a[r][2];
        m[r][3] = b[r];
    }
    for (uint8_t col = 0u; col < 3u; col++) {
        uint8_t pivot = col;
        for (uint8_t r = (uint8_t)(col + 1u); r < 3u; r++) {
            if (fabs(m[r][col]) > fabs(m[pivot][col])) {
                pivot = r;
            }
        }
        if (fabs(m[pivot][col]) < 1e-12) {
            return false;
        }
        if (pivot != col) {
            for (uint8_t c = 0u; c < 4u; c++) {
                double tmp = m[col][c];
                m[col][c] = m[pivot][c];
                m[pivot][c] = tmp;
            }
        }
        double div = m[col][col];
        for (uint8_t c = 0u; c < 4u; c++) {
            m[col][c] /= div;
        }
        for (uint8_t r = 0u; r < 3u; r++) {
            if (r == col) {
                continue;
            }
            double factor = m[r][col];
            for (uint8_t c = 0u; c < 4u; c++) {
                m[r][c] -= factor * m[col][c];
            }
        }
    }
    x[0] = m[0][3];
    x[1] = m[1][3];
    x[2] = m[2][3];
    return true;
}

static double det3(double a[3][3]) {
    return a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
         - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
         + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
}

static void calibration_to_nvm(uint8_t *buf, uint16_t len) {
    if (buf == NULL || len < OS_CAL_NVM_RECORD_SIZE) {
        return;
    }
    buf[0] = OS_CAL_NVM_MAGIC0;
    buf[1] = OS_CAL_NVM_MAGIC1;
    buf[2] = OS_CAL_NVM_MAGIC2;
    buf[3] = OS_CAL_NVM_MAGIC3;
    memcpy(&buf[4], &s_calib[0][0], sizeof(double));
    memcpy(&buf[12], &s_calib[0][1], sizeof(double));
    memcpy(&buf[20], &s_calib[0][2], sizeof(double));
    memcpy(&buf[28], &s_calib[1][0], sizeof(double));
    memcpy(&buf[36], &s_calib[1][1], sizeof(double));
    memcpy(&buf[44], &s_calib[1][2], sizeof(double));
    buf[52] = s_calibration_valid ? 1u : 0u;
}

static bool calibration_from_nvm(const uint8_t *buf, uint16_t len) {
    if (buf == NULL || len < OS_CAL_NVM_RECORD_SIZE) {
        return false;
    }
    if (buf[0] != OS_CAL_NVM_MAGIC0 || buf[1] != OS_CAL_NVM_MAGIC1 ||
        buf[2] != OS_CAL_NVM_MAGIC2 || buf[3] != OS_CAL_NVM_MAGIC3) {
        return false;
    }
    memcpy(&s_calib[0][0], &buf[4], sizeof(double));
    memcpy(&s_calib[0][1], &buf[12], sizeof(double));
    memcpy(&s_calib[0][2], &buf[20], sizeof(double));
    memcpy(&s_calib[1][0], &buf[28], sizeof(double));
    memcpy(&s_calib[1][1], &buf[36], sizeof(double));
    memcpy(&s_calib[1][2], &buf[44], sizeof(double));
    s_calibration_valid = (buf[52] != 0u);
    return true;
}

os_error_t os_init(void) {
    s_init_complete = false;
    s_goto_active = false;
    s_park_active = false;
    s_manual_active = false;
    s_align_count = 0u;
    s_residual_computed = false;
    s_calibration_valid = false;
    s_residual_arcsec = 0.0f;
    s_align_prior_calib_valid = false;
    s_guide.active = false;
    s_guide.duration_ms = 0u;
    s_guide_elapsed = 0u;
    s_pec_enabled = false;
    s_gps_locked = false;
    s_rtc_valid = false;
    s_tracking_active = false;
    s_park_position_set = false;
    s_track_rate = OS_TRACK_RATE_SIDEREAL;
    s_custom_track_factor = 1.0f;
    s_manual_custom_speed_arcsec = 100.0f;
    s_state = OS_STATE_INITIALIZING;

    memset(&s_site, 0, sizeof(s_site));
    s_site.latitude_degrees = 0.0f;
    s_site.longitude_degrees = 0.0f;
    s_site.elevation_metres = 0.0f;
    s_site.utc_epoch_seconds = 0u;
    s_site.valid = false;

    (void)os_hal_nvm_init();
    (void)os_hal_motor_init(0u);
    (void)os_hal_motor_init(1u);
    (void)os_hal_comm_init(OS_CHANNEL_USB);
    (void)os_hal_comm_init(OS_CHANNEL_BLUETOOTH);
    (void)os_hal_comm_init(OS_CHANNEL_WIFI);
    (void)os_hal_comm_init(OS_CHANNEL_ETHERNET);
    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    (void)os_hal_timer_motor_init();

    os_site_info_t poll_site;
    memset(&poll_site, 0, sizeof(poll_site));
    os_error_t gps_ret = os_hal_gps_poll(&poll_site);
    if (gps_ret == OS_ERR_NONE && poll_site.valid) {
        s_gps_site = poll_site;
        s_site = poll_site;
        s_gps_locked = true;
        (void)os_hal_rtc_set(poll_site.utc_epoch_seconds);
    } else {
        s_gps_locked = false;
        uint32_t utc = 0u;
        if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = utc;
            s_rtc_valid = true;
        } else {
            s_rtc_valid = false;
        }
    }

    uint8_t nvm_rec[OS_CAL_NVM_RECORD_SIZE];
    memset(nvm_rec, 0, sizeof(nvm_rec));
    if (os_hal_nvm_read(0u, nvm_rec, OS_CAL_NVM_RECORD_SIZE) == OS_ERR_NONE) {
        (void)calibration_from_nvm(nvm_rec, OS_CAL_NVM_RECORD_SIZE);
    } else {
        s_calibration_valid = false;
    }

    s_tracking_active = true;
    s_state = OS_STATE_IDLE_TRACKING;
    s_init_complete = true;
    tracking_apply();
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    if (!s_init_complete) {
        s_state = OS_STATE_FAULT;
        return;
    }

    os_site_info_t poll_site;
    memset(&poll_site, 0, sizeof(poll_site));
    if (os_hal_gps_poll(&poll_site) == OS_ERR_NONE && poll_site.valid) {
        s_gps_locked = true;
        s_site = poll_site;
        (void)os_hal_rtc_set(poll_site.utc_epoch_seconds);
    } else {
        s_gps_locked = false;
        uint32_t utc = 0u;
        if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
            s_site.utc_epoch_seconds = utc;
            s_rtc_valid = true;
        } else {
            s_rtc_valid = false;
        }
    }

    update_guide();
    motion_tick();
    loop_commands();
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length == 0u || length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!channel_valid(source_channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (reply_buffer_size == 0u || reply_buffer_size > OS_MAX_REPLY_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *reply_length = 0u;
    reply_buffer[0] = '\0';

    if (length < 2u || command[0] != OS_LX200_CMD_PREFIX ||
        command[length - 1u] != OS_LX200_CMD_SUFFIX) {
        reply_text(reply_buffer, reply_buffer_size, reply_length, "0");
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return OS_ERR_COMMAND_FORMAT;
    }

    char inner[OS_MAX_COMMAND_LENGTH];
    size_t inner_len = length - 2u;
    if (inner_len >= sizeof(inner)) {
        inner_len = sizeof(inner) - 1u;
    }
    memcpy(inner, command + 1u, inner_len);
    inner[inner_len] = '\0';

    os_error_t err = OS_ERR_COMMAND_FORMAT;
    bool handled = false;

    if (inner_len >= 2u && strncmp(inner, "GD", 2u) == 0) {
        handled = true;
        os_equatorial_coord_t coord;
        err = os_query_coordinates(&coord);
        if (err == OS_ERR_NONE) {
            reply_float(reply_buffer, reply_buffer_size, reply_length, coord.dec_degrees);
        } else {
            reply_text(reply_buffer, reply_buffer_size, reply_length, "0");
        }
    } else if (inner_len >= 2u && strncmp(inner, "GR", 2u) == 0) {
        handled = true;
        os_equatorial_coord_t coord;
        err = os_query_coordinates(&coord);
        if (err == OS_ERR_NONE) {
            reply_float(reply_buffer, reply_buffer_size, reply_length, coord.ra_hours);
        } else {
            reply_text(reply_buffer, reply_buffer_size, reply_length, "0");
        }
    } else if (inner_len >= 3u && strncmp(inner, "GVP", 3u) == 0) {
        handled = true;
        reply_text(reply_buffer, reply_buffer_size, reply_length, "1.0.0");
        err = OS_ERR_NONE;
    } else if (inner_len >= 2u && strncmp(inner, "Me", 2u) == 0) {
        handled = true;
        err = os_move_start(OS_DIRECTION_EAST, OS_SPEED_SLOW);
    } else if (inner_len >= 2u && strncmp(inner, "Mw", 2u) == 0) {
        handled = true;
        err = os_move_start(OS_DIRECTION_WEST, OS_SPEED_SLOW);
    } else if (inner_len >= 2u && strncmp(inner, "Mn", 2u) == 0) {
        handled = true;
        err = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_SLOW);
    } else if (inner_len >= 2u && strncmp(inner, "Ms", 2u) == 0) {
        handled = true;
        err = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_SLOW);
    } else if (inner_len >= 2u && strncmp(inner, "MS", 2u) == 0) {
        handled = true;
        os_equatorial_coord_t target;
        target.ra_hours = 0.0f;
        target.dec_degrees = 0.0f;
        err = os_goto_equatorial(target);
    } else if (inner_len >= 2u && strncmp(inner, "hP", 2u) == 0) {
        handled = true;
        err = os_park();
    } else if (inner_len >= 2u && strncmp(inner, "hO", 2u) == 0) {
        handled = true;
        err = os_unpark();
    } else if (inner_len >= 2u && strncmp(inner, "Te", 2u) == 0) {
        handled = true;
        err = os_tracking_enable();
    } else if (inner_len >= 2u && strncmp(inner, "Td", 2u) == 0) {
        handled = true;
        err = os_tracking_disable();
    }

    if (!handled) {
        reply_text(reply_buffer, reply_buffer_size, reply_length, "0");
        (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
        return OS_ERR_COMMAND_FORMAT;
    }

    if (err == OS_ERR_NONE) {
        if (*reply_length == 0u) {
            reply_text(reply_buffer, reply_buffer_size, reply_length, "1");
        }
    } else {
        reply_text(reply_buffer, reply_buffer_size, reply_length, "0");
    }

    (void)os_hal_comm_write(source_channel, reply_buffer, *reply_length);
    return err;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS ||
        target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    int32_t target_ra = (int32_t)((double)target.ra_hours * 3600.0 * 10.0);
    int32_t target_dec = (int32_t)((double)target.dec_degrees * 3600.0 * 10.0);
    int32_t ra_err = target_ra - s_motor_pos[0];
    int32_t dec_err = target_dec - s_motor_pos[1];
    if (abs(ra_err) <= OS_GOTO_SMALL_DELTA_STEPS &&
        abs(dec_err) <= OS_GOTO_SMALL_DELTA_STEPS) {
        stop_motion_outputs();
        s_state = OS_STATE_IDLE_TRACKING;
        s_tracking_active = true;
        tracking_apply();
        return OS_ERR_NONE;
    }

    s_goto_target[0] = target_ra;
    s_goto_target[1] = target_dec;
    s_goto_active = true;
    s_park_active = false;
    s_manual_active = false;
    s_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (target.altitude_degrees < OS_DEC_MIN_DEG || target.altitude_degrees > OS_DEC_MAX_DEG ||
        target.azimuth_degrees < 0.0f || target.azimuth_degrees >= 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    int32_t target_ra = (int32_t)((double)target.azimuth_degrees * 1000.0);
    int32_t target_dec = (int32_t)((double)target.altitude_degrees * 1000.0);
    int32_t ra_err = target_ra - s_motor_pos[0];
    int32_t dec_err = target_dec - s_motor_pos[1];
    if (abs(ra_err) <= OS_GOTO_SMALL_DELTA_STEPS &&
        abs(dec_err) <= OS_GOTO_SMALL_DELTA_STEPS) {
        stop_motion_outputs();
        s_state = OS_STATE_IDLE_TRACKING;
        s_tracking_active = true;
        tracking_apply();
        return OS_ERR_NONE;
    }

    s_goto_target[0] = target_ra;
    s_goto_target[1] = target_dec;
    s_goto_active = true;
    s_park_active = false;
    s_manual_active = false;
    s_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (!s_goto_active && s_state != OS_STATE_GOTO) {
        return OS_ERR_INVALID_STATE;
    }
    s_goto_active = false;
    s_park_active = false;
    s_manual_active = false;
    stop_motion_outputs();
    s_state = OS_STATE_IDLE_TRACKING;
    s_tracking_active = true;
    tracking_apply();
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (rate < OS_TRACK_RATE_SIDEREAL || rate > OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM &&
        (!isfinite(custom_factor) || custom_factor <= 0.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    s_track_rate = rate;
    if (rate == OS_TRACK_RATE_CUSTOM) {
        s_custom_track_factor = custom_factor;
    } else {
        s_custom_track_factor = 1.0f;
    }
    tracking_apply();
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = s_track_rate;
    *custom_factor = s_custom_track_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    s_tracking_active = true;
    tracking_apply();
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    s_tracking_active = false;
    stop_motion_outputs();
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state == OS_STATE_PARKED || s_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t axis = manual_axis_for_direction(direction);
    bool forward = manual_forward_for_direction(direction);
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_guide_axis = axis;
    s_guide_forward = forward;
    s_guide.active = true;
    s_guide.duration_ms = duration_ms;
    s_guide.rate_fraction = s_guide_rate_fraction;
    s_guide_elapsed = 0u;
    s_guide.direction_east = (direction == OS_DIRECTION_EAST);
    s_guide.direction_north = (direction == OS_DIRECTION_NORTH);
    s_guide.dec_priority = (axis == 1u);

    os_hal_motor_set_direction(axis, forward);
    uint32_t base = tracking_frequency_hz();
    uint32_t p = (uint32_t)((double)base * (double)s_guide.rate_fraction);
    uint32_t freq = base + p;
    if (freq > OS_MOTOR_FREQ_MAX_HZ) {
        freq = OS_MOTOR_FREQ_MAX_HZ;
    }
    os_hal_motor_set_frequency(axis, freq);
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (!isfinite(rate_fraction) ||
        rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_guide_rate_fraction = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    update_guide();
    *pulse = s_guide;
    pulse->rate_fraction = s_guide_rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    s_align_prior_calib_valid = s_calibration_valid;
    s_align_mode = mode;
    s_align_count = 0u;
    s_residual_computed = false;
    s_residual_arcsec = 0.0f;
    s_calibration_valid = false;
    s_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (star_coord.ra_hours < OS_RA_MIN_HOURS || star_coord.ra_hours > OS_RA_MAX_HOURS ||
        star_coord.dec_degrees < OS_DEC_MIN_DEG || star_coord.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (s_align_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }
    uint8_t idx = s_align_count;
    s_align_ra[idx] = (double)star_coord.ra_hours;
    s_align_dec[idx] = (double)star_coord.dec_degrees;
    s_align_mra[idx] = motor_pos.ra_steps;
    s_align_mdec[idx] = motor_pos.dec_steps;
    s_align_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    uint8_t min_stars = 0u;
    if (s_align_mode == OS_ALIGN_1STAR) {
        min_stars = 1u;
    } else if (s_align_mode == OS_ALIGN_2STAR) {
        min_stars = 2u;
    } else {
        min_stars = 3u;
    }
    if (s_align_count < min_stars) {
        s_residual_computed = false;
        return OS_ERR_INVALID_STATE;
    }

    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double d = 0.0;
    double e = 0.0;
    double f = 0.0;
    bool solved = true;
    uint8_t n = s_align_count;

    if (s_align_mode == OS_ALIGN_1STAR) {
        a = 1.0;
        c = (double)s_align_mra[0] - s_align_ra[0];
        e = 1.0;
        f = (double)s_align_mdec[0] - s_align_dec[0];
    } else if (s_align_mode == OS_ALIGN_2STAR) {
        double xa[2][2];
        double ya[2];
        double xv[2];
        xa[0][0] = s_align_ra[0];
        xa[0][1] = 1.0;
        xa[1][0] = s_align_ra[1];
        xa[1][1] = 1.0;
        ya[0] = (double)s_align_mra[0];
        ya[1] = (double)s_align_mra[1];
        if (!solve_linear2(xa, ya, xv)) {
            solved = false;
        } else {
            a = xv[0];
            c = xv[1];
        }
        double xd[2][2];
        double yd[2];
        double xdv[2];
        xd[0][0] = s_align_dec[0];
        xd[0][1] = 1.0;
        xd[1][0] = s_align_dec[1];
        xd[1][1] = 1.0;
        yd[0] = (double)s_align_mdec[0];
        yd[1] = (double)s_align_mdec[1];
        if (!solve_linear2(xd, yd, xdv)) {
            solved = false;
        } else {
            e = xdv[0];
            f = xdv[1];
        }
    } else {
        double xtx[3][3];
        double xty_ra[3];
        double xty_dec[3];
        memset(xtx, 0, sizeof(xtx));
        memset(xty_ra, 0, sizeof(xty_ra));
        memset(xty_dec, 0, sizeof(xty_dec));
        for (uint8_t i = 0u; i < n; i++) {
            double row[3];
            row[0] = s_align_ra[i];
            row[1] = s_align_dec[i];
            row[2] = 1.0;
            for (uint8_t j = 0u; j < 3u; j++) {
                for (uint8_t k = 0u; k < 3u; k++) {
                    xtx[j][k] += row[j] * row[k];
                }
            }
            xty_ra[0] += row[0] * (double)s_align_mra[i];
            xty_ra[1] += row[1] * (double)s_align_mra[i];
            xty_ra[2] += row[2] * (double)s_align_mra[i];
            xty_dec[0] += row[0] * (double)s_align_mdec[i];
            xty_dec[1] += row[1] * (double)s_align_mdec[i];
            xty_dec[2] += row[2] * (double)s_align_mdec[i];
        }
        if (fabs(det3(xtx)) < 1e-12) {
            solved = false;
        }
        double beta_ra[3];
        double beta_dec[3];
        if (solved && !solve_linear3(xtx, xty_ra, beta_ra)) {
            solved = false;
        }
        if (solved && !solve_linear3(xtx, xty_dec, beta_dec)) {
            solved = false;
        }
        if (solved) {
            a = beta_ra[0];
            b = beta_ra[1];
            c = beta_ra[2];
            d = beta_dec[0];
            e = beta_dec[1];
            f = beta_dec[2];
        }
    }

    s_residual_computed = true;
    if (!solved) {
        s_residual_arcsec = 0.0f;
        s_calibration_valid = false;
        return OS_ERR_CALIBRATION_FAILED;
    }

    double sum = 0.0;
    for (uint8_t i = 0u; i < n; i++) {
        double pred_ra = a * s_align_ra[i] + b * s_align_dec[i] + c;
        double pred_dec = d * s_align_ra[i] + e * s_align_dec[i] + f;
        double erra = pred_ra - (double)s_align_mra[i];
        double errd = pred_dec - (double)s_align_mdec[i];
        sum += erra * erra + errd * errd;
    }
    double rms = sqrt(sum / (double)n);
    s_residual_arcsec = (float)rms;

    if (s_align_mode == OS_ALIGN_NSTAR && n >= 4u) {
        if (!isfinite(s_residual_arcsec) ||
            s_residual_arcsec > (float)OS_ALIGN_RESIDUAL_LIMIT_ARCSEC) {
            s_calibration_valid = false;
            return OS_ERR_CALIBRATION_FAILED;
        }
    }

    s_calib[0][0] = a;
    s_calib[0][1] = b;
    s_calib[0][2] = c;
    s_calib[1][0] = d;
    s_calib[1][1] = e;
    s_calib[1][2] = f;
    s_calibration_valid = true;

    uint8_t nvm_rec[OS_CAL_NVM_RECORD_SIZE];
    memset(nvm_rec, 0, sizeof(nvm_rec));
    calibration_to_nvm(nvm_rec, OS_CAL_NVM_RECORD_SIZE);
    if (os_hal_nvm_write(0u, nvm_rec, OS_CAL_NVM_RECORD_SIZE) != OS_ERR_NONE) {
        s_state = OS_STATE_FAULT;
        return OS_ERR_NVM_FAULT;
    }

    s_state = OS_STATE_IDLE_TRACKING;
    s_tracking_active = true;
    tracking_apply();
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_residual_computed) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = s_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    if (s_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    s_align_count = 0u;
    s_residual_computed = false;
    s_residual_arcsec = 0.0f;
    s_calibration_valid = s_align_prior_calib_valid;
    s_state = OS_STATE_IDLE_TRACKING;
    s_tracking_active = true;
    tracking_apply();
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    if (os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    if (s_park_position_set) {
        s_park_target[0] = (int32_t)((double)s_park_position.ra_hours * 3600.0 * 10.0);
        s_park_target[1] = (int32_t)((double)s_park_position.dec_degrees * 3600.0 * 10.0);
    } else {
        s_park_target[0] = 0;
        s_park_target[1] = 0;
    }

    s_goto_active = false;
    s_manual_active = false;
    s_park_active = true;
    s_tracking_active = false;
    s_state = OS_STATE_PARKED;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (s_state != OS_STATE_PARKED && !s_park_active) {
        return OS_ERR_INVALID_STATE;
    }
    s_park_active = false;
    (void)os_hal_motor_init(0u);
    (void)os_hal_motor_init(1u);
    (void)os_hal_motor_enable(0u, true);
    (void)os_hal_motor_enable(1u, true);
    (void)os_hal_comm_init(OS_CHANNEL_USB);
    (void)os_hal_comm_init(OS_CHANNEL_BLUETOOTH);
    (void)os_hal_comm_init(OS_CHANNEL_WIFI);
    (void)os_hal_comm_init(OS_CHANNEL_ETHERNET);
    if (s_gps_locked) {
        (void)os_hal_rtc_set(s_site.utc_epoch_seconds);
    }
    s_tracking_active = true;
    s_state = OS_STATE_IDLE_TRACKING;
    tracking_apply();
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (park_pos.ra_hours < OS_RA_MIN_HOURS || park_pos.ra_hours > OS_RA_MAX_HOURS ||
        park_pos.dec_degrees < OS_DEC_MIN_DEG || park_pos.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_park_position = park_pos;
    s_park_position_set = true;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (s_state != OS_STATE_IDLE_TRACKING) {
        return OS_ERR_INVALID_STATE;
    }
    uint8_t axis = manual_axis_for_direction(direction);
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    s_manual_active = true;
    s_manual_direction = direction;
    s_manual_speed = speed;
    s_manual_remaining = OS_MANUAL_TIMEOUT_TICKS;
    s_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (!s_manual_active || s_state != OS_STATE_MANUAL_MOTION) {
        return OS_ERR_INVALID_STATE;
    }
    s_manual_active = false;
    s_manual_remaining = 0u;
    stop_motion_outputs();
    s_state = OS_STATE_IDLE_TRACKING;
    s_tracking_active = true;
    tracking_apply();
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (!isfinite(arcsec_per_sec) || arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_manual_custom_speed_arcsec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = s_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_calibration_valid) {
        return OS_ERR_CALIBRATION_FAILED;
    }
    double a = s_calib[0][0];
    double b = s_calib[0][1];
    double c = s_calib[0][2];
    double d = s_calib[1][0];
    double e = s_calib[1][1];
    double f = s_calib[1][2];
    double det = a * e - b * d;
    if (fabs(det) < 1e-12) {
        return OS_ERR_CALIBRATION_FAILED;
    }
    double rhs_ra = (double)s_motor_pos[0] - c;
    double rhs_dec = (double)s_motor_pos[1] - f;
    double ra = (e * rhs_ra - b * rhs_dec) / det;
    double dec = (-d * rhs_ra + a * rhs_dec) / det;
    if (ra < OS_RA_MIN_HOURS) {
        ra = OS_RA_MIN_HOURS;
    }
    if (ra > OS_RA_MAX_HOURS) {
        ra = OS_RA_MAX_HOURS;
    }
    if (dec < OS_DEC_MIN_DEG) {
        dec = OS_DEC_MIN_DEG;
    }
    if (dec > OS_DEC_MAX_DEG) {
        dec = OS_DEC_MAX_DEG;
    }
    coord->ra_hours = (float)ra;
    coord->dec_degrees = (float)dec;
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = s_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = s_motor_pos[0];
    pos->dec_steps = s_motor_pos[1];
    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor, uint8_t *patch) {
    if (major == NULL || minor == NULL || patch == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *major = OS_FIRMWARE_VERSION_MAJOR;
    *minor = OS_FIRMWARE_VERSION_MINOR;
    *patch = OS_FIRMWARE_VERSION_PATCH;
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *moving) {
    if (moving == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *moving = s_goto_active || s_manual_active || s_park_active;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *locked = s_gps_locked;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    if (enable && !s_pec_table.valid) {
        return OS_ERR_INVALID_STATE;
    }
    s_pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_pec_table = *table;
    s_pec_table.valid = true;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *table = s_pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (!isfinite(worm_phase_deg) || worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int idx = (int)worm_phase_deg;
    if (idx >= 360) {
        idx = 359;
    }
    if (idx < 0) {
        idx = 0;
    }
    s_pec_table.corrections[idx] = error_arcsec;
    s_pec_table.valid = true;
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    calib->matrix_ra_to_ra = (float)s_calib[0][0];
    calib->matrix_ra_to_dec = (float)s_calib[0][1];
    calib->matrix_dec_to_ra = (float)s_calib[1][0];
    calib->matrix_dec_to_dec = (float)s_calib[1][1];
    calib->offset_ra_arcsec = (float)s_calib[0][2];
    calib->offset_dec_arcsec = (float)s_calib[1][2];
    calib->valid = s_calibration_valid;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    memset(s_calib, 0, sizeof(s_calib));
    s_calibration_valid = false;
    s_residual_computed = false;
    s_residual_arcsec = 0.0f;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_init(uint8_t axis) {
    if (!axis_valid(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_motor_initialized[axis] = true;
    s_motor_enabled[axis] = false;
    s_motor_freq[axis] = 0u;
    s_motor_dir[axis] = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (!axis_valid(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (frequency_hz > OS_MOTOR_FREQ_MAX_HZ) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_motor_freq[axis] = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (!axis_valid(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    s_motor_dir[axis] = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (!axis_valid(axis)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!enable) {
        s_motor_freq[axis] = 0u;
    }
    s_motor_enabled[axis] = enable;
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (!axis_valid(axis)) {
        return 0;
    }
    return s_motor_pos[axis];
}

os_error_t os_hal_gps_init(void) {
    s_gps_initialized = true;
    s_gps_has_fix = false;
    memset(&s_gps_site, 0, sizeof(s_gps_site));
    s_gps_site.valid = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_gps_initialized) {
        site->valid = false;
        return OS_ERR_NOT_SUPPORTED;
    }
    if (!s_gps_has_fix) {
        site->valid = false;
        return OS_ERR_GPS_NO_SIGNAL;
    }
    *site = s_gps_site;
    site->valid = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void) {
    s_rtc_initialized = true;
    s_rtc_epoch = 0u;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_rtc_initialized) {
        return OS_ERR_NOT_SUPPORTED;
    }
    *utc_epoch_seconds = s_rtc_epoch;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    if (!s_rtc_initialized) {
        return OS_ERR_NOT_SUPPORTED;
    }
    s_rtc_epoch = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    s_limit_initialized = true;
    s_limit[0] = false;
    s_limit[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (!axis_valid(axis)) {
        return true;
    }
    if (!s_limit_initialized) {
        return false;
    }
    return s_limit[axis];
}

os_error_t os_hal_nvm_init(void) {
    s_nvm_initialized = true;
    memset(s_nvm, 0, sizeof(s_nvm));
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((uint32_t)offset + (uint32_t)length > (uint32_t)OS_NVM_TOTAL_BYTES) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_nvm_initialized) {
        return OS_ERR_NOT_SUPPORTED;
    }
    memcpy(data, &s_nvm[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((uint32_t)offset + (uint32_t)length > (uint32_t)OS_NVM_TOTAL_BYTES) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_nvm_initialized) {
        return OS_ERR_NOT_SUPPORTED;
    }
    memcpy(&s_nvm[offset], data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (!channel_valid(channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memset(&s_comm[channel], 0, sizeof(s_comm[channel]));
    s_comm[channel].initialized = true;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if (!channel_valid(channel) || !s_comm[channel].initialized) {
        return 0;
    }
    return (int16_t)s_comm[channel].rx_count;
}

char os_hal_comm_read(uint8_t channel) {
    if (!channel_valid(channel) || !s_comm[channel].initialized ||
        s_comm[channel].rx_count == 0u) {
        return '\0';
    }
    char c = s_comm[channel].rx[s_comm[channel].rx_head];
    s_comm[channel].rx_head = (uint16_t)((s_comm[channel].rx_head + 1u) & 255u);
    s_comm[channel].rx_count--;
    return c;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (!channel_valid(channel)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (data == NULL && length > 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!s_comm[channel].initialized) {
        return OS_ERR_NOT_SUPPORTED;
    }
    if (length == 0u) {
        return OS_ERR_NONE;
    }
    if ((size_t)s_comm[channel].tx_count + length > OS_COMM_BUFFER_SIZE) {
        return OS_ERR_TIMEOUT;
    }
    for (size_t i = 0u; i < length; i++) {
        s_comm[channel].tx[s_comm[channel].tx_tail] = data[i];
        s_comm[channel].tx_tail = (uint16_t)((s_comm[channel].tx_tail + 1u) & 255u);
        s_comm[channel].tx_count++;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) {
    (void)duration_ms;
    (void)count;
    s_buzzer_scheduled = true;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void) {
    s_timer_initialized = true;
    return OS_ERR_NONE;
}

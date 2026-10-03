#include "6_generated_code.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

#define ONSTEP_AXIS_COUNT           2
#define ONSTEP_AXIS_RA              0
#define ONSTEP_AXIS_DEC             1
#define ONSTEP_STEPS_PER_DEGREE     1000.0
#define ONSTEP_DEFAULT_ARC_SEC_PER_STEP (3600.0 / ONSTEP_STEPS_PER_DEGREE)
#define ONSTEP_GOTO_MAX_FREQ_HZ     ((uint32_t)(OS_GOTO_SPEED_MAX_DEG_PER_SEC * ONSTEP_STEPS_PER_DEGREE))
#define ONSTEP_GOTO_LOW_FREQ_HZ     200u
#define ONSTEP_ARRIVE_EPSILON_STEPS 2
#define ONSTEP_SPEED_SLOW_ARCSEC    15.0f
#define ONSTEP_SPEED_MEDIUM_ARCSEC  60.0f
#define ONSTEP_SPEED_FAST_ARCSEC    180.0f
#define ONSTEP_ALIGN_MAX_RESIDUAL_ARCSEC 300.0
#define ONSTEP_PLATFORM_MAX_FREQ_HZ 200000UL
#define ONSTEP_PI 3.14159265358979323846
#define COMM_RX_CAP                 256
#define COMM_TX_CAP                 256

#define NVM_CAL_MAGIC   0x4F4C4343UL
#define NVM_CFG_MAGIC   0x4F434647UL
#define NVM_PEC_MAGIC   0x4F504543UL
#define NVM_MOTOR_POS_MAGIC 0x4F4D5053UL
#define NVM_CAL_VERSION 1u
#define NVM_CFG_VERSION 1u
#define NVM_PEC_VERSION 1u
#define NVM_MOTOR_POS_VERSION 1u
#define OS_NVM_TOTAL_SIZE 2048
#define OS_NVM_PEC_OFFSET (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)

/*------------------------------------------------------------------------*/
typedef struct {
    bool initialized;
    bool enabled;
    bool forward;
    uint32_t frequency_hz;
    int32_t position_steps;
    double step_accum;
    os_error_t last_fault;
} motor_hal_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t checksum;
    os_calibration_t calib;
} cal_nvm_record_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t checksum;
    os_mount_type_t mount_type;
    bool site_valid;
    bool park_valid;
    os_site_info_t site;
    os_equatorial_coord_t park_pos;
} cfg_nvm_record_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t checksum;
    bool valid;
    int16_t corrections[OS_PEC_TABLE_SIZE];
} pec_nvm_record_t;

#define OS_NVM_MOTOR_POS_OFFSET ((uint16_t)(OS_NVM_PEC_OFFSET + (uint16_t)sizeof(pec_nvm_record_t)))

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t checksum;
    bool valid;
    int32_t ra_steps;
    int32_t dec_steps;
} motor_pos_nvm_record_t;

typedef struct {
    char rx[COMM_RX_CAP];
    uint16_t rx_head;
    uint16_t rx_tail;
    char tx[COMM_TX_CAP];
    uint16_t tx_head;
    uint16_t tx_tail;
    bool enabled;
} comm_channel_t;

/*------------------------------------------------------------------------*/
static motor_hal_t motor_hal[ONSTEP_AXIS_COUNT];
static os_site_info_t hal_gps;
static uint32_t hal_rtc_utc;
static bool limit_triggered[ONSTEP_AXIS_COUNT];
static uint8_t hal_nvm[OS_NVM_TOTAL_SIZE];
static bool nvm_initialized = false;
static comm_channel_t comm_channels[4];
static uint16_t buzzer_last_duration;
static uint8_t buzzer_last_count;
static bool timer_initialized;

/* Domain state */
static os_state_t current_state = OS_STATE_INITIALIZING;
static os_mount_type_t mount_type = OS_MOUNT_EQUATORIAL;
static bool tracking_enabled = false;
static os_track_rate_t tracking_rate = OS_TRACK_RATE_SIDEREAL;
static float tracking_custom_factor = 1.0f;

static os_guide_pulse_t guide_pulse;
static float guide_rate_fraction = 0.5f;

static os_equatorial_coord_t command_target;
static bool command_target_valid = false;

static bool goto_active = false;
static bool park_active = false;
static int32_t goto_target_ra_steps = 0;
static int32_t goto_target_dec_steps = 0;

static bool manual_active = false;
static os_direction_t manual_direction = OS_DIRECTION_NORTH;
static os_speed_level_t manual_speed = OS_SPEED_SLOW;
static float manual_custom_arcsec_per_sec = ONSTEP_SPEED_SLOW_ARCSEC;

static os_align_mode_t align_mode = OS_ALIGN_1STAR;
static uint8_t align_count = 0;
static os_equatorial_coord_t align_stars[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t align_motor_positions[OS_CALIBRATION_MAX_STARS];
static bool align_residual_computed = false;
static float align_residual_arcsec = 0.0f;

static bool calib_valid = false;
static double calib_m00 = ONSTEP_DEFAULT_ARC_SEC_PER_STEP;
static double calib_m01 = 0.0;
static double calib_m10 = 0.0;
static double calib_m11 = ONSTEP_DEFAULT_ARC_SEC_PER_STEP;
static double calib_b0 = 0.0;
static double calib_b1 = 0.0;

static bool pec_enabled = false;
static bool pec_valid = false;
static os_pec_table_t pec_table;

static os_site_info_t current_site;
static os_site_info_t config_site;
static bool gps_locked = false;

static bool park_position_valid = false;
static os_equatorial_coord_t park_position = {0.0f, 90.0f};
static bool limit_direction_known[ONSTEP_AXIS_COUNT] = {false, false};
static bool limit_unsafe_forward[ONSTEP_AXIS_COUNT] = {false, false};

static char rx_line[4][OS_MAX_COMMAND_LENGTH + 1];
static uint8_t rx_line_len[4];

static int32_t last_saved_motor_ra = INT32_MIN;
static int32_t last_saved_motor_dec = INT32_MIN;

/*------------------------------------------------------------------------*/
static uint16_t calc_checksum(const uint8_t *data, size_t len) {
    uint32_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return (uint16_t)(sum & 0xFFFFu);
}

static void stop_axis(uint8_t axis) {
    (void)os_hal_motor_set_frequency(axis, 0);
    (void)os_hal_motor_enable(axis, false);
}

static void stop_all_axes(void) {
    stop_axis(0);
    stop_axis(1);
}

static int32_t abs_i32(int32_t v) {
    if (v < 0) {
        return (int32_t)(-(int64_t)v);
    }
    return v;
}

static double normalize_deg_360(double a) {
    a = fmod(a, 360.0);
    if (a < 0.0) {
        a += 360.0;
    }
    return a;
}

static double normalize_deg_180(double a) {
    a = normalize_deg_360(a);
    if (a > 180.0) {
        a -= 360.0;
    }
    return a;
}

static double jd_from_utc(uint32_t utc_epoch_seconds) {
    return (double)utc_epoch_seconds / 86400.0 + 2440587.5;
}

static double gmst_deg(double jd) {
    double t = (jd - 2451545.0) / 36525.0;
    double gmst = 280.46061837 + 360.98564736629 * (jd - 2451545.0);
    gmst += 0.000387933 * t * t - t * t * t / 38710000.0;
    return normalize_deg_360(gmst);
}

static double lst_deg(double jd, double longitude_deg) {
    return normalize_deg_360(gmst_deg(jd) + longitude_deg);
}

static void equatorial_to_horizontal_deg(double ra_deg, double dec_deg,
                                         double lat_deg, double lon_deg,
                                         uint32_t utc_epoch_seconds,
                                         double *az_deg, double *alt_deg) {
    double jd = jd_from_utc(utc_epoch_seconds);
    double lst = lst_deg(jd, lon_deg);
    double ha_deg = normalize_deg_180(lst - ra_deg);

    double lat = lat_deg * ONSTEP_PI / 180.0;
    double dec = dec_deg * ONSTEP_PI / 180.0;
    double ha = ha_deg * ONSTEP_PI / 180.0;

    double sin_alt = sin(lat) * sin(dec) + cos(lat) * cos(dec) * cos(ha);
    double alt = asin(fmax(-1.0, fmin(1.0, sin_alt)));
    double az = atan2(-sin(ha) * cos(dec),
                      sin(dec) * cos(lat) - cos(dec) * sin(lat) * cos(ha));

    *alt_deg = alt * 180.0 / ONSTEP_PI;
    *az_deg = normalize_deg_360(az * 180.0 / ONSTEP_PI);
}

static void horizontal_to_equatorial_deg(double az_deg, double alt_deg,
                                         double lat_deg, double lon_deg,
                                         uint32_t utc_epoch_seconds,
                                         double *ra_deg, double *dec_deg) {
    double jd = jd_from_utc(utc_epoch_seconds);
    double lst = lst_deg(jd, lon_deg);

    double lat = lat_deg * ONSTEP_PI / 180.0;
    double az = az_deg * ONSTEP_PI / 180.0;
    double alt = alt_deg * ONSTEP_PI / 180.0;

    double sin_dec = sin(alt) * sin(lat) + cos(alt) * cos(lat) * cos(az);
    double dec = asin(fmax(-1.0, fmin(1.0, sin_dec)));
    double ha = atan2(-sin(az) * cos(alt),
                      sin(alt) * cos(lat) - cos(alt) * sin(lat) * cos(az));

    *dec_deg = dec * 180.0 / ONSTEP_PI;
    *ra_deg = normalize_deg_360(lst - ha * 180.0 / ONSTEP_PI);
}

static double current_site_lat_deg(void) {
    return current_site.valid ? (double)current_site.latitude_degrees : 0.0;
}

static double current_site_lon_deg(void) {
    return current_site.valid ? (double)current_site.longitude_degrees : 0.0;
}

static uint32_t current_site_utc(void) {
    return current_site.utc_epoch_seconds;
}

static bool motion_axis_limit_ok(uint8_t axis, int32_t current, int32_t target) {
    int32_t error = target - current;
    if (error == 0) {
        return true;
    }
    if (!os_hal_limit_is_triggered(axis)) {
        return true;
    }
    if (!limit_direction_known[axis]) {
        limit_direction_known[axis] = true;
        limit_unsafe_forward[axis] = true;
    }
    bool want_forward = error > 0;
    return want_forward != limit_unsafe_forward[axis];
}

static bool manual_direction_to_axis_sign(os_direction_t direction,
                                          uint8_t *axis,
                                          bool *forward) {
    switch (direction) {
        case OS_DIRECTION_NORTH:
            *axis = 1; *forward = true; return true;
        case OS_DIRECTION_SOUTH:
            *axis = 1; *forward = false; return true;
        case OS_DIRECTION_EAST:
            *axis = 0; *forward = true; return true;
        case OS_DIRECTION_WEST:
            *axis = 0; *forward = false; return true;
        default:
            return false;
    }
}

static void motor_tick_all(void) {
    for (uint8_t axis = 0; axis < ONSTEP_AXIS_COUNT; ++axis) {
        motor_hal_t *m = &motor_hal[axis];
        if (!m->enabled || m->frequency_hz == 0) {
            m->step_accum = 0.0;
            continue;
        }
        double inc = (double)m->frequency_hz / 1000.0;
        m->step_accum += inc;
        int32_t steps = (int32_t)m->step_accum;
        if (steps != 0) {
            if (m->forward) {
                m->position_steps += steps;
            } else {
                m->position_steps -= steps;
            }
            m->step_accum -= steps;
        }
    }
}

/*------------------------------------------------------------------------*/
/* HAL implementation */
/*------------------------------------------------------------------------*/
os_error_t os_hal_motor_init(uint8_t axis) {
    if (axis >= ONSTEP_AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    motor_hal[axis].initialized = true;
    motor_hal[axis].enabled = false;
    motor_hal[axis].forward = true;
    motor_hal[axis].frequency_hz = 0;
    motor_hal[axis].position_steps = 0;
    motor_hal[axis].step_accum = 0.0;
    motor_hal[axis].last_fault = OS_ERR_NONE;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) {
    if (axis >= ONSTEP_AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!motor_hal[axis].initialized) {
        return OS_ERR_INVALID_STATE;
    }
    if (frequency_hz > ONSTEP_PLATFORM_MAX_FREQ_HZ) {
        frequency_hz = ONSTEP_PLATFORM_MAX_FREQ_HZ;
    }
    motor_hal[axis].frequency_hz = frequency_hz;
    if (frequency_hz == 0) {
        motor_hal[axis].step_accum = 0.0;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) {
    if (axis >= ONSTEP_AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!motor_hal[axis].initialized) {
        return OS_ERR_INVALID_STATE;
    }
    motor_hal[axis].forward = forward;
    motor_hal[axis].step_accum = 0.0;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable) {
    if (axis >= ONSTEP_AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!motor_hal[axis].initialized) {
        return OS_ERR_INVALID_STATE;
    }
    motor_hal[axis].enabled = enable;
    if (!enable) {
        motor_hal[axis].frequency_hz = 0;
        motor_hal[axis].step_accum = 0.0;
    }
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis) {
    if (axis >= ONSTEP_AXIS_COUNT) {
        return 0;
    }
    return motor_hal[axis].position_steps;
}

os_error_t os_hal_gps_init(void) {
    memset(&hal_gps, 0, sizeof(hal_gps));
    hal_gps.valid = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = hal_gps;
    if (!hal_gps.valid) {
        return OS_ERR_GPS_NO_SIGNAL;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void) {
    hal_rtc_utc = 0;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) {
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *utc_epoch_seconds = hal_rtc_utc;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) {
    hal_rtc_utc = utc_epoch_seconds;
    return OS_ERR_NONE;
}

os_error_t os_hal_limit_init(void) {
    limit_triggered[0] = false;
    limit_triggered[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis) {
    if (axis >= ONSTEP_AXIS_COUNT) {
        return true;
    }
    return limit_triggered[axis];
}

os_error_t os_hal_nvm_init(void) {
    if (!nvm_initialized) {
        memset(hal_nvm, 0, sizeof(hal_nvm));
        nvm_initialized = true;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) {
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((uint32_t)offset + length > (uint32_t)OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(data, hal_nvm + offset, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) {
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if ((uint32_t)offset + length > (uint32_t)OS_NVM_TOTAL_SIZE) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(hal_nvm + offset, data, length);
    return OS_ERR_NONE;
}

os_error_t os_hal_comm_init(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memset(&comm_channels[channel], 0, sizeof(comm_channels[channel]));
    comm_channels[channel].enabled = true;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET || !comm_channels[channel].enabled) {
        return 0;
    }
    comm_channel_t *c = &comm_channels[channel];
    uint16_t n = (uint16_t)((c->rx_tail - c->rx_head + COMM_RX_CAP) % COMM_RX_CAP);
    return (int16_t)n;
}

char os_hal_comm_read(uint8_t channel) {
    if (channel > OS_CHANNEL_ETHERNET || !comm_channels[channel].enabled) {
        return '\0';
    }
    comm_channel_t *c = &comm_channels[channel];
    if (c->rx_head == c->rx_tail) {
        return '\0';
    }
    char ch = c->rx[c->rx_head];
    c->rx_head = (uint16_t)((c->rx_head + 1) % COMM_RX_CAP);
    return ch;
}

os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) {
    if (channel > OS_CHANNEL_ETHERNET || data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!comm_channels[channel].enabled) {
        return OS_ERR_NOT_SUPPORTED;
    }
    comm_channel_t *c = &comm_channels[channel];
    for (size_t i = 0; i < length; ++i) {
        uint16_t next_tail = (uint16_t)((c->tx_tail + 1) % COMM_TX_CAP);
        if (next_tail == c->tx_head) {
            return OS_ERR_INVALID_ARGUMENT;
        }
        c->tx[c->tx_tail] = data[i];
        c->tx_tail = next_tail;
    }
    return OS_ERR_NONE;
}

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) {
    buzzer_last_duration = duration_ms;
    buzzer_last_count = count;
    return OS_ERR_NONE;
}

os_error_t os_hal_timer_motor_init(void) {
    timer_initialized = true;
    return OS_ERR_NONE;
}

/*------------------------------------------------------------------------*/
/* NVM records */
/*------------------------------------------------------------------------*/
static void calibration_from_public(const os_calibration_t *cal) {
    calib_m00 = cal->matrix_ra_to_ra;
    calib_m01 = cal->matrix_ra_to_dec;
    calib_m10 = cal->matrix_dec_to_ra;
    calib_m11 = cal->matrix_dec_to_dec;
    calib_b0 = cal->offset_ra_arcsec;
    calib_b1 = cal->offset_dec_arcsec;
    calib_valid = cal->valid;
}

static os_calibration_t calibration_to_public(void) {
    os_calibration_t cal;
    memset(&cal, 0, sizeof(cal));
    cal.matrix_ra_to_ra = (float)calib_m00;
    cal.matrix_ra_to_dec = (float)calib_m01;
    cal.matrix_dec_to_ra = (float)calib_m10;
    cal.matrix_dec_to_dec = (float)calib_m11;
    cal.offset_ra_arcsec = (float)calib_b0;
    cal.offset_dec_arcsec = (float)calib_b1;
    cal.valid = calib_valid;
    return cal;
}

static os_error_t save_calibration_to_nvm(void) {
    cal_nvm_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = NVM_CAL_MAGIC;
    rec.version = NVM_CAL_VERSION;
    rec.calib = calibration_to_public();
    rec.checksum = calc_checksum((const uint8_t *)&rec, sizeof(rec));
    return os_hal_nvm_write(0, (const uint8_t *)&rec, sizeof(rec));
}

static void load_calibration_from_nvm(void) {
    cal_nvm_record_t rec;
    memset(&rec, 0, sizeof(rec));
    if (os_hal_nvm_read(0, (uint8_t *)&rec, sizeof(rec)) != OS_ERR_NONE) {
        calib_valid = false;
        return;
    }
    uint16_t saved = rec.checksum;
    rec.checksum = 0;
    if (rec.magic != NVM_CAL_MAGIC || rec.version != NVM_CAL_VERSION ||
        saved != calc_checksum((const uint8_t *)&rec, sizeof(rec))) {
        calib_valid = false;
        return;
    }
    calibration_from_public(&rec.calib);
}

static os_error_t save_config_to_nvm(void) {
    cfg_nvm_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = NVM_CFG_MAGIC;
    rec.version = NVM_CFG_VERSION;
    rec.mount_type = mount_type;
    rec.site_valid = config_site.valid;
    rec.park_valid = park_position_valid;
    rec.site = config_site;
    rec.park_pos = park_position;
    rec.checksum = calc_checksum((const uint8_t *)&rec, sizeof(rec));
    return os_hal_nvm_write(OS_NVM_CALIBRATION_SIZE_BYTES, (const uint8_t *)&rec, sizeof(rec));
}

static void load_config_from_nvm(void) {
    cfg_nvm_record_t rec;
    memset(&rec, 0, sizeof(rec));
    if (os_hal_nvm_read(OS_NVM_CALIBRATION_SIZE_BYTES, (uint8_t *)&rec, sizeof(rec)) != OS_ERR_NONE) {
        config_site.valid = false;
        park_position_valid = false;
        mount_type = OS_MOUNT_EQUATORIAL;
        return;
    }
    uint16_t saved = rec.checksum;
    rec.checksum = 0;
    if (rec.magic != NVM_CFG_MAGIC || rec.version != NVM_CFG_VERSION ||
        saved != calc_checksum((const uint8_t *)&rec, sizeof(rec))) {
        config_site.valid = false;
        park_position_valid = false;
        mount_type = OS_MOUNT_EQUATORIAL;
        return;
    }
    mount_type = rec.mount_type;
    config_site = rec.site;
    park_position = rec.park_pos;
    park_position_valid = rec.park_valid;
}

static os_error_t save_pec_to_nvm(void) {
    pec_nvm_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = NVM_PEC_MAGIC;
    rec.version = NVM_PEC_VERSION;
    rec.valid = pec_valid;
    memcpy(rec.corrections, pec_table.corrections, sizeof(pec_table.corrections));
    rec.checksum = calc_checksum((const uint8_t *)&rec, sizeof(rec));
    return os_hal_nvm_write(OS_NVM_PEC_OFFSET, (const uint8_t *)&rec, sizeof(rec));
}

static void load_pec_from_nvm(void) {
    pec_nvm_record_t rec;
    memset(&rec, 0, sizeof(rec));
    if (os_hal_nvm_read(OS_NVM_PEC_OFFSET, (uint8_t *)&rec, sizeof(rec)) != OS_ERR_NONE) {
        pec_valid = false;
        return;
    }
    uint16_t saved = rec.checksum;
    rec.checksum = 0;
    if (rec.magic != NVM_PEC_MAGIC || rec.version != NVM_PEC_VERSION ||
        saved != calc_checksum((const uint8_t *)&rec, sizeof(rec))) {
        pec_valid = false;
        return;
    }
    pec_valid = rec.valid;
    memcpy(pec_table.corrections, rec.corrections, sizeof(pec_table.corrections));
    pec_table.valid = rec.valid;
}

static os_error_t save_motor_position_to_nvm(void) {
    motor_pos_nvm_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = NVM_MOTOR_POS_MAGIC;
    rec.version = NVM_MOTOR_POS_VERSION;
    rec.valid = true;
    rec.ra_steps = os_hal_motor_get_position(0);
    rec.dec_steps = os_hal_motor_get_position(1);
    rec.checksum = calc_checksum((const uint8_t *)&rec, sizeof(rec));
    return os_hal_nvm_write(OS_NVM_MOTOR_POS_OFFSET, (const uint8_t *)&rec, sizeof(rec));
}

static void load_motor_position_from_nvm(void) {
    motor_pos_nvm_record_t rec;
    memset(&rec, 0, sizeof(rec));
    if (os_hal_nvm_read(OS_NVM_MOTOR_POS_OFFSET, (uint8_t *)&rec, sizeof(rec)) != OS_ERR_NONE) {
        return;
    }
    uint16_t saved = rec.checksum;
    rec.checksum = 0;
    if (rec.magic != NVM_MOTOR_POS_MAGIC || rec.version != NVM_MOTOR_POS_VERSION ||
        saved != calc_checksum((const uint8_t *)&rec, sizeof(rec)) || !rec.valid) {
        return;
    }
    motor_hal[0].position_steps = rec.ra_steps;
    motor_hal[1].position_steps = rec.dec_steps;
}

/*------------------------------------------------------------------------*/
/* Coordinate helpers */
/*------------------------------------------------------------------------*/
static void coord_to_steps(const os_equatorial_coord_t *coord,
                           int32_t *ra_steps,
                           int32_t *dec_steps) {
    if (mount_type == OS_MOUNT_ALTAZ) {
        double az_deg, alt_deg;
        equatorial_to_horizontal_deg((double)coord->ra_hours * 15.0,
                                     (double)coord->dec_degrees,
                                     current_site_lat_deg(),
                                     current_site_lon_deg(),
                                     current_site_utc(),
                                     &az_deg, &alt_deg);
        *ra_steps = (int32_t)llround(az_deg * ONSTEP_STEPS_PER_DEGREE);
        *dec_steps = (int32_t)llround(alt_deg * ONSTEP_STEPS_PER_DEGREE);
        return;
    }

    double ra_as = (double)coord->ra_hours * 15.0 * 3600.0;
    double dec_as = (double)coord->dec_degrees * 3600.0;
    if (calib_valid) {
        double det = calib_m00 * calib_m11 - calib_m01 * calib_m10;
        if (fabs(det) > 1e-12) {
            double dra = ra_as - calib_b0;
            double ddec = dec_as - calib_b1;
            double x = ( calib_m11 * dra - calib_m01 * ddec) / det;
            double y = (-calib_m10 * dra + calib_m00 * ddec) / det;
            *ra_steps = (int32_t)llround(x);
            *dec_steps = (int32_t)llround(y);
            return;
        }
    }
    *ra_steps = (int32_t)llround(ra_as / ONSTEP_DEFAULT_ARC_SEC_PER_STEP);
    *dec_steps = (int32_t)llround(dec_as / ONSTEP_DEFAULT_ARC_SEC_PER_STEP);
}

static void steps_to_coord(int32_t ra_steps, int32_t dec_steps,
                           os_equatorial_coord_t *coord) {
    if (mount_type == OS_MOUNT_ALTAZ) {
        double az_deg = (double)ra_steps / ONSTEP_STEPS_PER_DEGREE;
        double alt_deg = (double)dec_steps / ONSTEP_STEPS_PER_DEGREE;
        double ra_deg, dec_deg;
        horizontal_to_equatorial_deg(az_deg, alt_deg,
                                     current_site_lat_deg(),
                                     current_site_lon_deg(),
                                     current_site_utc(),
                                     &ra_deg, &dec_deg);
        coord->ra_hours = (float)(normalize_deg_360(ra_deg) / 15.0);
        coord->dec_degrees = (float)fmin(OS_DEC_MAX_DEG, fmax(OS_DEC_MIN_DEG, dec_deg));
        return;
    }

    double ra_as;
    double dec_as;
    if (calib_valid) {
        ra_as = calib_m00 * ra_steps + calib_m01 * dec_steps + calib_b0;
        dec_as = calib_m10 * ra_steps + calib_m11 * dec_steps + calib_b1;
    } else {
        ra_as = (double)ra_steps * ONSTEP_DEFAULT_ARC_SEC_PER_STEP;
        dec_as = (double)dec_steps * ONSTEP_DEFAULT_ARC_SEC_PER_STEP;
    }
    double ra_deg = ra_as / 3600.0;
    double dec_deg = dec_as / 3600.0;
    while (ra_deg < 0.0) {
        ra_deg += 360.0;
    }
    while (ra_deg >= 360.0) {
        ra_deg -= 360.0;
    }
    coord->ra_hours = (float)(ra_deg / 15.0);
    coord->dec_degrees = (float)(fmin(OS_DEC_MAX_DEG, fmax(OS_DEC_MIN_DEG, dec_deg)));
}

/*------------------------------------------------------------------------*/
/* Tracking and guide helpers */
/*------------------------------------------------------------------------*/
static double get_tracking_rate_arcsec(void) {
    switch (tracking_rate) {
        case OS_TRACK_RATE_LUNAR:
            return OS_SIDEREAL_RATE_ARCSEC_PER_SEC * OS_LUNAR_RATE_FACTOR;
        case OS_TRACK_RATE_SOLAR:
            return OS_SIDEREAL_RATE_ARCSEC_PER_SEC * OS_SOLAR_RATE_FACTOR;
        case OS_TRACK_RATE_CUSTOM:
            return OS_SIDEREAL_RATE_ARCSEC_PER_SEC * tracking_custom_factor;
        case OS_TRACK_RATE_SIDEREAL:
        default:
            return OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
    }
}

static uint32_t compute_tracking_frequency(void) {
    double as = get_tracking_rate_arcsec();
    return (uint32_t)llround((as / 3600.0) * ONSTEP_STEPS_PER_DEGREE);
}

static void compute_altaz_tracking_frequencies_signed(int64_t *f0_signed,
                                                      int64_t *f1_signed) {
    int32_t ra_steps = os_hal_motor_get_position(0);
    int32_t dec_steps = os_hal_motor_get_position(1);
    double az = (double)ra_steps / ONSTEP_STEPS_PER_DEGREE;
    double alt = (double)dec_steps / ONSTEP_STEPS_PER_DEGREE;

    double ra_deg, dec_deg;
    horizontal_to_equatorial_deg(az, alt,
                                 current_site_lat_deg(),
                                 current_site_lon_deg(),
                                 current_site_utc(),
                                 &ra_deg, &dec_deg);

    double az1, alt1, az2, alt2;
    equatorial_to_horizontal_deg(ra_deg, dec_deg,
                                 current_site_lat_deg(),
                                 current_site_lon_deg(),
                                 current_site_utc(),
                                 &az1, &alt1);
    equatorial_to_horizontal_deg(ra_deg, dec_deg,
                                 current_site_lat_deg(),
                                 current_site_lon_deg(),
                                 current_site_utc() + 1u,
                                 &az2, &alt2);

    double daz = normalize_deg_180(az2 - az1);
    double dalt = alt2 - alt1;
    *f0_signed = (int64_t)llround(daz * ONSTEP_STEPS_PER_DEGREE);
    *f1_signed = (int64_t)llround(dalt * ONSTEP_STEPS_PER_DEGREE);
}

static uint32_t manual_frequency(void) {
    float arcsec = ONSTEP_SPEED_SLOW_ARCSEC;
    switch (manual_speed) {
        case OS_SPEED_SLOW:
            arcsec = ONSTEP_SPEED_SLOW_ARCSEC;
            break;
        case OS_SPEED_MEDIUM:
            arcsec = ONSTEP_SPEED_MEDIUM_ARCSEC;
            break;
        case OS_SPEED_FAST:
            arcsec = ONSTEP_SPEED_FAST_ARCSEC;
            break;
        case OS_SPEED_CUSTOM:
            if (manual_custom_arcsec_per_sec > 0.0f) {
                arcsec = manual_custom_arcsec_per_sec;
            }
            break;
        default:
            break;
    }
    double deg_per_sec = (double)arcsec / 3600.0;
    return (uint32_t)llround(deg_per_sec * ONSTEP_STEPS_PER_DEGREE);
}

static void guide_direction_to_axis_sign(const os_guide_pulse_t *pulse,
                                         uint8_t *axis,
                                         bool *forward) {
    if (pulse->dec_priority) {
        *axis = 1;
        *forward = pulse->direction_north;
    } else {
        *axis = 0;
        *forward = pulse->direction_east;
    }
}

/*------------------------------------------------------------------------*/
/* Alignment numeric helpers */
/*------------------------------------------------------------------------*/
static bool solve_ls_3param(int m,
                            const double *x,
                            const double *y,
                            const double *b,
                            double p[3],
                            double *res_ssq) {
    if (m < 3) {
        return false;
    }

    double q[OS_CALIBRATION_MAX_STARS][3];
    double r[3][3] = {{0.0}};

    for (int j = 0; j < 3; ++j) {
        double v[OS_CALIBRATION_MAX_STARS];
        for (int i = 0; i < m; ++i) {
            if (j == 0) {
                v[i] = 1.0;
            } else if (j == 1) {
                v[i] = x[i];
            } else {
                v[i] = y[i];
            }
        }

        for (int k = 0; k < j; ++k) {
            double dot = 0.0;
            for (int i = 0; i < m; ++i) {
                dot += q[i][k] * v[i];
            }
            r[k][j] = dot;
            for (int i = 0; i < m; ++i) {
                v[i] -= dot * q[i][k];
            }
        }

        double norm = 0.0;
        for (int i = 0; i < m; ++i) {
            norm += v[i] * v[i];
        }
        norm = sqrt(norm);
        if (norm < 1e-12) {
            return false;
        }
        for (int i = 0; i < m; ++i) {
            q[i][j] = v[i] / norm;
        }
        r[j][j] = norm;
    }

    double qb[3] = {0.0, 0.0, 0.0};
    for (int j = 0; j < 3; ++j) {
        for (int i = 0; i < m; ++i) {
            qb[j] += q[i][j] * b[i];
        }
    }

    for (int j = 2; j >= 0; --j) {
        double sum = qb[j];
        for (int k = j + 1; k < 3; ++k) {
            sum -= r[j][k] * p[k];
        }
        p[j] = sum / r[j][j];
    }

    double residual = 0.0;
    for (int i = 0; i < m; ++i) {
        double fit = p[0] + p[1] * x[i] + p[2] * y[i];
        double diff = b[i] - fit;
        residual += diff * diff;
    }
    *res_ssq = residual;
    return true;
}

static bool compute_alignment(void) {
    uint8_t min_stars = (align_mode == OS_ALIGN_1STAR) ? 1u :
                       (align_mode == OS_ALIGN_2STAR) ? 2u : 3u;
    if (align_count < min_stars) {
        return false;
    }

    double x[OS_CALIBRATION_MAX_STARS];
    double y[OS_CALIBRATION_MAX_STARS];
    double ra_as[OS_CALIBRATION_MAX_STARS];
    double dec_as[OS_CALIBRATION_MAX_STARS];

    for (uint8_t i = 0; i < align_count; ++i) {
        x[i] = (double)align_motor_positions[i].ra_steps;
        y[i] = (double)align_motor_positions[i].dec_steps;
        ra_as[i] = (double)align_stars[i].ra_hours * 15.0 * 3600.0;
        dec_as[i] = (double)align_stars[i].dec_degrees * 3600.0;
    }

    if (align_mode == OS_ALIGN_1STAR) {
        calib_m00 = ONSTEP_DEFAULT_ARC_SEC_PER_STEP;
        calib_m01 = 0.0;
        calib_m10 = 0.0;
        calib_m11 = ONSTEP_DEFAULT_ARC_SEC_PER_STEP;
        calib_b0 = ra_as[0] - calib_m00 * x[0] - calib_m01 * y[0];
        calib_b1 = dec_as[0] - calib_m10 * x[0] - calib_m11 * y[0];
        align_residual_arcsec = 0.0f;
        align_residual_computed = true;
        return true;
    }

    if (align_mode == OS_ALIGN_2STAR) {
        double dx = x[1] - x[0];
        double dy = y[1] - y[0];
        if (fabs(dx) < 1e-12 || fabs(dy) < 1e-12) {
            return false;
        }
        calib_m00 = (ra_as[1] - ra_as[0]) / dx;
        calib_m01 = 0.0;
        calib_m10 = 0.0;
        calib_m11 = (dec_as[1] - dec_as[0]) / dy;
        calib_b0 = ra_as[0] - calib_m00 * x[0];
        calib_b1 = dec_as[0] - calib_m11 * y[0];

        double res = 0.0;
        for (int i = 0; i < 2; ++i) {
            double fit_ra = calib_m00 * x[i] + calib_b0;
            double fit_dec = calib_m11 * y[i] + calib_b1;
            double dr = ra_as[i] - fit_ra;
            double dd = dec_as[i] - fit_dec;
            res += dr * dr + dd * dd;
        }
        align_residual_arcsec = (float)sqrt(res / 2.0);
        align_residual_computed = true;
        return true;
    }

    double p_ra[3] = {0.0, 0.0, 0.0};
    double p_dec[3] = {0.0, 0.0, 0.0};
    double res_ra = 0.0;
    double res_dec = 0.0;

    if (!solve_ls_3param((int)align_count, x, y, ra_as, p_ra, &res_ra) ||
        !solve_ls_3param((int)align_count, x, y, dec_as, p_dec, &res_dec)) {
        align_residual_computed = true;
        align_residual_arcsec = 1.0e9f;
        return false;
    }

    calib_b0 = p_ra[0];
    calib_m00 = p_ra[1];
    calib_m01 = p_ra[2];
    calib_b1 = p_dec[0];
    calib_m10 = p_dec[1];
    calib_m11 = p_dec[2];

    bool three_star = (align_count == 3);
    double det = p_ra[1] * p_dec[2] - p_ra[2] * p_dec[1];

    if (three_star) {
        if (fabs(det) < 1e-9) {
            align_residual_computed = true;
            align_residual_arcsec = 0.0f;
            return false;
        }
        align_residual_arcsec = 0.0f;
        align_residual_computed = true;
        return true;
    }

    double total = res_ra + res_dec;
    align_residual_arcsec = (float)sqrt(total / (double)align_count);
    align_residual_computed = true;

    if (fabs(det) < 1e-9) {
        return false;
    }
    if (align_residual_arcsec > (float)ONSTEP_ALIGN_MAX_RESIDUAL_ARCSEC) {
        return false;
    }
    return true;
}

/*------------------------------------------------------------------------*/
/* Motion updates */
/*------------------------------------------------------------------------*/
static void update_goto_or_park_motion(void) {
    int32_t cur0 = os_hal_motor_get_position(0);
    int32_t cur1 = os_hal_motor_get_position(1);
    int32_t err0 = goto_target_ra_steps - cur0;
    int32_t err1 = goto_target_dec_steps - cur1;
    bool arrived0 = (abs_i32(err0) <= ONSTEP_ARRIVE_EPSILON_STEPS);
    bool arrived1 = (abs_i32(err1) <= ONSTEP_ARRIVE_EPSILON_STEPS);

    if (arrived0 && arrived1) {
        os_hal_motor_set_frequency(0, 0);
        os_hal_motor_set_frequency(1, 0);
        bool was_park = park_active;
        goto_active = false;
        park_active = false;
        (void)os_hal_buzzer_beep(200, 1);
        if (was_park) {
            tracking_enabled = false;
            os_hal_motor_enable(0, false);
            os_hal_motor_enable(1, false);
            current_state = OS_STATE_PARKED;
            comm_channels[OS_CHANNEL_BLUETOOTH].enabled = false;
            comm_channels[OS_CHANNEL_WIFI].enabled = false;
            comm_channels[OS_CHANNEL_ETHERNET].enabled = false;
        } else {
            current_state = OS_STATE_IDLE_TRACKING;
            if (!tracking_enabled) {
                tracking_enabled = true;
            }
        }
        return;
    }

    for (uint8_t axis = 0; axis < 2; ++axis) {
        int32_t target = (axis == 0) ? goto_target_ra_steps : goto_target_dec_steps;
        int32_t current = (axis == 0) ? cur0 : cur1;
        int32_t error = target - current;

        if (abs_i32(error) <= ONSTEP_ARRIVE_EPSILON_STEPS) {
            (void)os_hal_motor_set_frequency(axis, 0);
            continue;
        }

        bool forward = error > 0;
        if (os_hal_limit_is_triggered(axis)) {
            if (!limit_direction_known[axis] || forward == limit_unsafe_forward[axis]) {
                limit_direction_known[axis] = true;
                limit_unsafe_forward[axis] = forward;
                stop_all_axes();
                goto_active = false;
                park_active = false;
                current_state = OS_STATE_FAULT;
                return;
            }
        }

        (void)os_hal_motor_enable(axis, true);
        (void)os_hal_motor_set_direction(axis, forward);
        uint32_t freq = (abs_i32(error) > 200) ? ONSTEP_GOTO_MAX_FREQ_HZ : ONSTEP_GOTO_LOW_FREQ_HZ;
        (void)os_hal_motor_set_frequency(axis, freq);
    }
}

static void update_manual_motion(void) {
    uint8_t axis = 0;
    bool forward = true;
    if (!manual_direction_to_axis_sign(manual_direction, &axis, &forward)) {
        stop_all_axes();
        manual_active = false;
        current_state = OS_STATE_IDLE_TRACKING;
        return;
    }

    if (os_hal_limit_is_triggered(axis)) {
        if (!limit_direction_known[axis] || forward == limit_unsafe_forward[axis]) {
            limit_direction_known[axis] = true;
            limit_unsafe_forward[axis] = forward;
            stop_all_axes();
            manual_active = false;
            current_state = OS_STATE_FAULT;
            return;
        }
    }

    uint32_t freq = manual_frequency();
    uint8_t other = (axis == 0) ? 1 : 0;
    (void)os_hal_motor_set_frequency(other, 0);
    (void)os_hal_motor_enable(axis, true);
    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, freq);
}

static void update_tracking_and_guide_rates(void) {
    if (current_state == OS_STATE_PARKED) {
        stop_all_axes();
        return;
    }

    if (current_state == OS_STATE_FAULT) {
        stop_all_axes();
        return;
    }

    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        stop_all_axes();
        current_state = OS_STATE_FAULT;
        return;
    }

    int64_t base_axis0 = 0;
    int64_t base_axis1 = 0;

    if (tracking_enabled && current_state == OS_STATE_IDLE_TRACKING) {
        if (mount_type == OS_MOUNT_ALTAZ) {
            int64_t f0_signed = 0;
            int64_t f1_signed = 0;
            compute_altaz_tracking_frequencies_signed(&f0_signed, &f1_signed);
            base_axis0 = f0_signed;
            base_axis1 = f1_signed;
        } else {
            base_axis0 = (int64_t)compute_tracking_frequency();
            base_axis1 = 0;
        }

        if (pec_enabled && pec_valid && mount_type == OS_MOUNT_EQUATORIAL) {
            int32_t pos = os_hal_motor_get_position(0);
            int idx = abs_i32(pos) % OS_PEC_TABLE_SIZE;
            int64_t corr_hz = (int64_t)llround(((double)pec_table.corrections[idx] / 3600.0) * ONSTEP_STEPS_PER_DEGREE);
            int64_t adjusted = base_axis0 + corr_hz;
            if (adjusted < 0) {
                adjusted = 0;
            }
            base_axis0 = adjusted;
        }
    }

    int64_t f0 = base_axis0;
    int64_t f1 = base_axis1;

    if (guide_pulse.active) {
        uint8_t gaxis = 0;
        bool gforward = true;
        guide_direction_to_axis_sign(&guide_pulse, &gaxis, &gforward);
        int64_t bias = (int64_t)llround(
            ((double)guide_rate_fraction * OS_SIDEREAL_RATE_ARCSEC_PER_SEC / 3600.0) *
            ONSTEP_STEPS_PER_DEGREE);
        if (gaxis == 0) {
            f0 += gforward ? bias : -bias;
        } else {
            f1 += gforward ? bias : -bias;
        }
    }

    bool f0_forward = f0 >= 0;
    bool f1_forward = f1 >= 0;
    uint32_t f0_abs = (uint32_t)(f0 >= 0 ? f0 : -f0);
    uint32_t f1_abs = (uint32_t)(f1 >= 0 ? f1 : -f1);

    if (f0_abs == 0) {
        (void)os_hal_motor_set_frequency(0, 0);
        (void)os_hal_motor_enable(0, false);
    } else {
        (void)os_hal_motor_enable(0, true);
        (void)os_hal_motor_set_direction(0, f0_forward);
        (void)os_hal_motor_set_frequency(0, f0_abs);
    }

    if (f1_abs == 0) {
        (void)os_hal_motor_set_frequency(1, 0);
        (void)os_hal_motor_enable(1, false);
    } else {
        (void)os_hal_motor_enable(1, true);
        (void)os_hal_motor_set_direction(1, f1_forward);
        (void)os_hal_motor_set_frequency(1, f1_abs);
    }
}

static void update_time_site(void) {
    os_site_info_t gps;
    if (os_hal_gps_poll(&gps) == OS_ERR_NONE && gps.valid) {
        current_site = gps;
        gps_locked = true;
        return;
    }

    uint32_t utc = 0;
    if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
        current_site.utc_epoch_seconds = utc;
        if (config_site.valid) {
            current_site.latitude_degrees = config_site.latitude_degrees;
            current_site.longitude_degrees = config_site.longitude_degrees;
            current_site.elevation_metres = config_site.elevation_metres;
            current_site.valid = true;
        } else {
            current_site.valid = false;
        }
    } else {
        current_site.valid = false;
    }
    gps_locked = false;
}

/*------------------------------------------------------------------------*/
/* Command parsing */
/*------------------------------------------------------------------------*/
static int reply_text(char *buf, size_t cap, size_t *len, const char *text) {
    size_t n = strlen(text);
    if (n >= cap) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    memcpy(buf, text, n);
    if (n < cap) {
        buf[n] = '\0';
    }
    *len = n;
    return OS_ERR_NONE;
}

static void format_ra_hms(const os_equatorial_coord_t *coord, char *buf, size_t cap) {
    double ra = coord->ra_hours;
    while (ra < 0.0) {
        ra += 24.0;
    }
    while (ra >= 24.0) {
        ra -= 24.0;
    }
    int h = (int)ra;
    double minf = (ra - h) * 60.0;
    int m = (int)minf;
    double sec = (minf - m) * 60.0;
    (void)snprintf(buf, cap, "%02d:%02d:%04.1f#", h, m, sec);
}

static void format_dec_dms(const os_equatorial_coord_t *coord, char *buf, size_t cap) {
    double dec = coord->dec_degrees;
    char sign = '+';
    if (dec < 0.0) {
        sign = '-';
        dec = -dec;
    }
    int d = (int)dec;
    double minf = (dec - d) * 60.0;
    int m = (int)minf;
    double sec = (minf - m) * 60.0;
    (void)snprintf(buf, cap, "%c%02d*%02d:%04.1f#", sign, d, m, sec);
}

static bool parse_ra_hours_text(const char *s, size_t len, float *value) {
    char buf[40];
    if (len >= sizeof(buf)) {
        return false;
    }
    memcpy(buf, s, len);
    buf[len] = '\0';

    int h = -1;
    int m = -1;
    float sec = -1.0f;
    int consumed = 0;
    if (sscanf(buf, "%d:%d:%f%n", &h, &m, &sec, &consumed) != 3) {
        return false;
    }
    if (consumed != (int)len) {
        return false;
    }
    if (m < 0 || m >= 60 || sec < 0.0f || sec >= 60.0f) {
        return false;
    }
    if (h == 24) {
        if (m != 0 || sec != 0.0f) {
            return false;
        }
        *value = 24.0f;
        return true;
    }
    if (h < 0 || h > 23) {
        return false;
    }
    *value = (float)h + ((float)m / 60.0f) + (sec / 3600.0f);
    return true;
}

static bool parse_dec_degrees_text(const char *s, size_t len, float *value) {
    char buf[40];
    if (len >= sizeof(buf)) {
        return false;
    }
    memcpy(buf, s, len);
    buf[len] = '\0';

    char *p = buf;
    int sign = 1;
    if (*p == '-') {
        sign = -1;
        ++p;
    } else if (*p == '+') {
        ++p;
    }

    if (*p == '\0') {
        return false;
    }

    int d = 0;
    int m = 0;
    float sec = 0.0f;
    int consumed = 0;
    if (sscanf(p, "%d*%d:%f%n", &d, &m, &sec, &consumed) != 3) {
        consumed = 0;
        if (sscanf(p, "%d:%d:%f%n", &d, &m, &sec, &consumed) != 3) {
            return false;
        }
    }

    size_t remaining = len - (size_t)(p - buf);
    if (consumed != (int)remaining) {
        return false;
    }
    if (d < 0 || d > 90 || m < 0 || m >= 60 || sec < 0.0f || sec >= 60.0f) {
        return false;
    }
    if (d == 90 && (m != 0 || sec != 0.0f)) {
        return false;
    }
    *value = (float)sign * ((float)d + ((float)m / 60.0f) + (sec / 3600.0f));
    return true;
}

static void poll_commands(void) {
    for (uint8_t ch = 0; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        while (os_hal_comm_available(ch) > 0) {
            char c = os_hal_comm_read(ch);
            if (rx_line_len[ch] == 0 && c != OS_LX200_CMD_PREFIX) {
                continue;
            }
            if (rx_line_len[ch] < OS_MAX_COMMAND_LENGTH) {
                rx_line[ch][rx_line_len[ch]++] = c;
            } else {
                rx_line_len[ch] = 0;
                continue;
            }
            if (c == OS_LX200_CMD_SUFFIX) {
                if (rx_line_len[ch] >= 2 &&
                    rx_line[ch][0] == OS_LX200_CMD_PREFIX) {
                    char reply[OS_MAX_REPLY_LENGTH];
                    size_t reply_len = 0;
                    os_error_t parse_error = os_command_parse(rx_line[ch], rx_line_len[ch], ch,
                                                              reply, sizeof(reply), &reply_len);
                    if (parse_error == OS_ERR_NONE) {
                        if (reply_len > 0) {
                            (void)os_hal_comm_write(ch, reply, reply_len);
                        }
                    } else {
                        static const char error_reply[] = "0";
                        (void)os_hal_comm_write(ch, error_reply, 1);
                    }
                }
                rx_line_len[ch] = 0;
            }
        }
    }
}

/*------------------------------------------------------------------------*/
/* Public API implementation */
/*------------------------------------------------------------------------*/
static void reset_runtime_state(void) {
    current_state = OS_STATE_INITIALIZING;
    mount_type = OS_MOUNT_EQUATORIAL;
    tracking_enabled = false;
    tracking_rate = OS_TRACK_RATE_SIDEREAL;
    tracking_custom_factor = 1.0f;
    memset(&guide_pulse, 0, sizeof(guide_pulse));
    guide_rate_fraction = 0.5f;
    command_target_valid = false;
    goto_active = false;
    park_active = false;
    manual_active = false;
    align_count = 0;
    align_mode = OS_ALIGN_1STAR;
    align_residual_computed = false;
    align_residual_arcsec = 0.0f;
    calib_valid = false;
    calib_m00 = ONSTEP_DEFAULT_ARC_SEC_PER_STEP;
    calib_m01 = 0.0;
    calib_m10 = 0.0;
    calib_m11 = ONSTEP_DEFAULT_ARC_SEC_PER_STEP;
    calib_b0 = 0.0;
    calib_b1 = 0.0;
    pec_enabled = false;
    pec_valid = false;
    memset(&pec_table, 0, sizeof(pec_table));
    memset(&current_site, 0, sizeof(current_site));
    memset(&config_site, 0, sizeof(config_site));
    gps_locked = false;
    park_position_valid = false;
    park_position.ra_hours = 0.0f;
    park_position.dec_degrees = 90.0f;
    limit_direction_known[0] = false;
    limit_direction_known[1] = false;
    limit_unsafe_forward[0] = false;
    limit_unsafe_forward[1] = false;
    memset(rx_line, 0, sizeof(rx_line));
    memset(rx_line_len, 0, sizeof(rx_line_len));
    last_saved_motor_ra = INT32_MIN;
    last_saved_motor_dec = INT32_MIN;
}

static void cancel_manual_motion(void) {
    if (manual_active) {
        stop_all_axes();
        manual_active = false;
    }
}

static void cancel_goto_park_motion(void) {
    if (goto_active || park_active) {
        stop_all_axes();
        goto_active = false;
        park_active = false;
    }
}

static void comm_enable_all_channels(void) {
    for (uint8_t ch = 0; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        comm_channels[ch].enabled = true;
    }
}

os_error_t os_init(void) {
    reset_runtime_state();

    (void)os_hal_nvm_init();
    load_calibration_from_nvm();
    load_config_from_nvm();
    load_pec_from_nvm();

    for (uint8_t ch = 0; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        (void)os_hal_comm_init(ch);
    }

    os_error_t err = os_hal_motor_init(0);
    if (err != OS_ERR_NONE) {
        motor_hal[0].last_fault = err;
        current_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    err = os_hal_motor_init(1);
    if (err != OS_ERR_NONE) {
        motor_hal[1].last_fault = err;
        current_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    if (os_hal_motor_enable(0, false) != OS_ERR_NONE ||
        os_hal_motor_enable(1, false) != OS_ERR_NONE) {
        current_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    load_motor_position_from_nvm();

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    if (os_hal_timer_motor_init() != OS_ERR_NONE) {
        current_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }

    update_time_site();

    tracking_enabled = true;
    tracking_rate = OS_TRACK_RATE_SIDEREAL;
    tracking_custom_factor = 1.0f;
    current_state = OS_STATE_IDLE_TRACKING;

    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    motor_tick_all();
    update_time_site();
    poll_commands();

    if (current_state == OS_STATE_FAULT) {
        stop_all_axes();
        goto_active = false;
        park_active = false;
        manual_active = false;
        return;
    }

    if (goto_active) {
        update_goto_or_park_motion();
    }
    if (manual_active) {
        update_manual_motion();
    }

    if (!goto_active && !manual_active && !park_active) {
        update_tracking_and_guide_rates();
    }

    if (guide_pulse.active) {
        if (guide_pulse.duration_ms <= 1u) {
            guide_pulse.active = false;
        } else {
            --guide_pulse.duration_ms;
        }
    }

    int32_t ra_pos = os_hal_motor_get_position(0);
    int32_t dec_pos = os_hal_motor_get_position(1);
    if (ra_pos != last_saved_motor_ra || dec_pos != last_saved_motor_dec) {
        if (save_motor_position_to_nvm() == OS_ERR_NONE) {
            last_saved_motor_ra = ra_pos;
            last_saved_motor_dec = dec_pos;
        }
    }
}

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *reply_length = 0;
    if (reply_buffer_size == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (source_channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length == 0 || length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length < 2 ||
        command[0] != OS_LX200_CMD_PREFIX ||
        command[length - 1] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    char cmd[OS_MAX_COMMAND_LENGTH + 1];
    memcpy(cmd, command, length);
    cmd[length] = '\0';

    char tmp[OS_MAX_REPLY_LENGTH];
    os_equatorial_coord_t coord;

    if (strcmp(cmd, ":GR#") == 0) {
        (void)os_query_coordinates(&coord);
        format_ra_hms(&coord, tmp, sizeof(tmp));
        return reply_text(reply_buffer, reply_buffer_size, reply_length, tmp);
    }
    if (strcmp(cmd, ":GD#") == 0) {
        (void)os_query_coordinates(&coord);
        format_dec_dms(&coord, tmp, sizeof(tmp));
        return reply_text(reply_buffer, reply_buffer_size, reply_length, tmp);
    }
    if (strcmp(cmd, ":GVP#") == 0) {
        (void)snprintf(tmp, sizeof(tmp), "%d.%d.%d#",
                       OS_FIRMWARE_VERSION_MAJOR,
                       OS_FIRMWARE_VERSION_MINOR,
                       OS_FIRMWARE_VERSION_PATCH);
        return reply_text(reply_buffer, reply_buffer_size, reply_length, tmp);
    }
    if (strcmp(cmd, ":Me#") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_EAST, OS_SPEED_FAST);
        if (err == OS_ERR_NONE) {
            return reply_text(reply_buffer, reply_buffer_size, reply_length, "1");
        }
        return err;
    }
    if (strcmp(cmd, ":Mw#") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_WEST, OS_SPEED_FAST);
        if (err == OS_ERR_NONE) {
            return reply_text(reply_buffer, reply_buffer_size, reply_length, "1");
        }
        return err;
    }
    if (strcmp(cmd, ":Mn#") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_FAST);
        if (err == OS_ERR_NONE) {
            return reply_text(reply_buffer, reply_buffer_size, reply_length, "1");
        }
        return err;
    }
    if (strcmp(cmd, ":Ms#") == 0) {
        os_error_t err = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_FAST);
        if (err == OS_ERR_NONE) {
            return reply_text(reply_buffer, reply_buffer_size, reply_length, "1");
        }
        return err;
    }
    if (strcmp(cmd, ":Q#") == 0) {
        (void)os_move_stop();
        (void)os_goto_abort();
        return reply_text(reply_buffer, reply_buffer_size, reply_length, "1");
    }
    if (strcmp(cmd, ":hP#") == 0) {
        os_error_t err = os_park();
        if (err == OS_ERR_NONE) {
            return reply_text(reply_buffer, reply_buffer_size, reply_length, "1");
        }
        return err;
    }
    if (strcmp(cmd, ":hO#") == 0) {
        os_error_t err = os_unpark();
        if (err == OS_ERR_NONE) {
            return reply_text(reply_buffer, reply_buffer_size, reply_length, "1");
        }
        return err;
    }
    if (strcmp(cmd, ":MS#") == 0) {
        if (!command_target_valid) {
            return OS_ERR_INVALID_STATE;
        }
        os_error_t err = os_goto_equatorial(command_target);
        if (err == OS_ERR_NONE) {
            return reply_text(reply_buffer, reply_buffer_size, reply_length, "1");
        }
        return err;
    }
    if (strncmp(cmd, ":Sr", 3) == 0) {
        float ra = 0.0f;
        if (parse_ra_hours_text(cmd + 3, length - 3 - 1, &ra)) {
            command_target.ra_hours = ra;
            command_target_valid = true;
            return reply_text(reply_buffer, reply_buffer_size, reply_length, "1");
        }
        return OS_ERR_COMMAND_FORMAT;
    }
    if (strncmp(cmd, ":Sd", 3) == 0) {
        float dec = 0.0f;
        if (parse_dec_degrees_text(cmd + 3, length - 3 - 1, &dec)) {
            command_target.dec_degrees = dec;
            command_target_valid = true;
            return reply_text(reply_buffer, reply_buffer_size, reply_length, "1");
        }
        return OS_ERR_COMMAND_FORMAT;
    }

    return OS_ERR_NOT_SUPPORTED;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS ||
        target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (current_state == OS_STATE_FAULT || current_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    int32_t target0 = 0;
    int32_t target1 = 0;
    coord_to_steps(&target, &target0, &target1);

    int32_t cur0 = os_hal_motor_get_position(0);
    int32_t cur1 = os_hal_motor_get_position(1);
    if (!motion_axis_limit_ok(0, cur0, target0) ||
        !motion_axis_limit_ok(1, cur1, target1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    cancel_manual_motion();
    if (goto_active || park_active) {
        cancel_goto_park_motion();
    }

    goto_target_ra_steps = target0;
    goto_target_dec_steps = target1;
    goto_active = true;
    park_active = false;
    current_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (current_state == OS_STATE_FAULT || current_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    int32_t target0 = (int32_t)llround((double)target.azimuth_degrees * ONSTEP_STEPS_PER_DEGREE);
    int32_t target1 = (int32_t)llround((double)target.altitude_degrees * ONSTEP_STEPS_PER_DEGREE);

    int32_t cur0 = os_hal_motor_get_position(0);
    int32_t cur1 = os_hal_motor_get_position(1);
    if (!motion_axis_limit_ok(0, cur0, target0) ||
        !motion_axis_limit_ok(1, cur1, target1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    cancel_manual_motion();
    if (goto_active || park_active) {
        cancel_goto_park_motion();
    }

    goto_target_ra_steps = target0;
    goto_target_dec_steps = target1;
    goto_active = true;
    park_active = false;
    current_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (!goto_active && !park_active) {
        return OS_ERR_NONE;
    }
    stop_all_axes();
    goto_active = false;
    park_active = false;
    current_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (rate < OS_TRACK_RATE_SIDEREAL || rate > OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    tracking_rate = rate;
    if (rate == OS_TRACK_RATE_CUSTOM) {
        tracking_custom_factor = custom_factor;
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *rate = tracking_rate;
    *custom_factor = tracking_custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    tracking_enabled = false;
    (void)os_hal_motor_set_frequency(0, 0);
    (void)os_hal_motor_set_frequency(1, 0);
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (current_state == OS_STATE_FAULT || current_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    bool new_dec_priority = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);

    if (guide_pulse.active) {
        if (guide_pulse.dec_priority && !new_dec_priority) {
            return OS_ERR_INVALID_STATE;
        }
        if (new_dec_priority == guide_pulse.dec_priority) {
            bool new_forward = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_EAST);
            bool old_forward = guide_pulse.dec_priority ? guide_pulse.direction_north : guide_pulse.direction_east;
            if (new_forward != old_forward) {
                return OS_ERR_INVALID_STATE;
            }
        }
    }

    memset(&guide_pulse, 0, sizeof(guide_pulse));
    guide_pulse.active = true;
    guide_pulse.duration_ms = duration_ms;
    guide_pulse.rate_fraction = guide_rate_fraction;
    guide_pulse.dec_priority = new_dec_priority;
    guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    guide_rate_fraction = rate_fraction;
    if (guide_pulse.active) {
        guide_pulse.rate_fraction = rate_fraction;
    }
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *pulse = guide_pulse;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    align_mode = mode;
    align_count = 0;
    align_residual_computed = false;
    align_residual_arcsec = 0.0f;
    current_state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos) {
    if (star_coord.ra_hours < OS_RA_MIN_HOURS || star_coord.ra_hours > OS_RA_MAX_HOURS ||
        star_coord.dec_degrees < OS_DEC_MIN_DEG || star_coord.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (current_state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }
    if (align_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }
    align_stars[align_count] = star_coord;
    align_motor_positions[align_count] = motor_pos;
    ++align_count;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    uint8_t min_stars = (align_mode == OS_ALIGN_1STAR) ? 1u :
                       (align_mode == OS_ALIGN_2STAR) ? 2u : 3u;
    if (align_count < min_stars) {
        return OS_ERR_INVALID_STATE;
    }

    if (!compute_alignment()) {
        calib_valid = false;
        return OS_ERR_CALIBRATION_FAILED;
    }

    calib_valid = true;
    os_error_t save_err = save_calibration_to_nvm();
    if (save_err != OS_ERR_NONE) {
        calib_valid = false;
        return OS_ERR_NVM_FAULT;
    }

    current_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!align_residual_computed) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = align_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    align_count = 0;
    align_residual_computed = false;
    align_residual_arcsec = 0.0f;
    if (current_state == OS_STATE_ALIGNMENT) {
        current_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (current_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    if (current_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    os_equatorial_coord_t target = park_position_valid ? park_position : (os_equatorial_coord_t){0.0f, 90.0f};
    int32_t target0 = 0;
    int32_t target1 = 0;
    coord_to_steps(&target, &target0, &target1);

    int32_t cur0 = os_hal_motor_get_position(0);
    int32_t cur1 = os_hal_motor_get_position(1);
    if (!motion_axis_limit_ok(0, cur0, target0) ||
        !motion_axis_limit_ok(1, cur1, target1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    cancel_manual_motion();
    if (goto_active || park_active) {
        cancel_goto_park_motion();
    }

    goto_target_ra_steps = target0;
    goto_target_dec_steps = target1;
    goto_active = true;
    park_active = true;
    current_state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (current_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    (void)os_hal_motor_enable(0, true);
    (void)os_hal_motor_enable(1, true);
    comm_enable_all_channels();

    update_time_site();
    if (gps_locked && current_site.valid) {
        (void)os_hal_rtc_set(current_site.utc_epoch_seconds);
    }

    tracking_enabled = true;
    current_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (park_pos.ra_hours < OS_RA_MIN_HOURS || park_pos.ra_hours > OS_RA_MAX_HOURS ||
        park_pos.dec_degrees < OS_DEC_MIN_DEG || park_pos.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    park_position = park_pos;
    park_position_valid = true;
    os_error_t save_err = save_config_to_nvm();
    if (save_err != OS_ERR_NONE) {
        return save_err;
    }
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (current_state == OS_STATE_FAULT || current_state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t axis = 0;
    bool forward = true;
    (void)manual_direction_to_axis_sign(direction, &axis, &forward);

    int32_t cur = os_hal_motor_get_position(axis);
    int32_t target = cur + (forward ? 1 : -1);
    if (!motion_axis_limit_ok(axis, cur, target)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    cancel_goto_park_motion();

    manual_direction = direction;
    manual_speed = speed;
    manual_active = true;
    current_state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (!manual_active) {
        return OS_ERR_NONE;
    }
    stop_all_axes();
    manual_active = false;
    current_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    manual_custom_arcsec_per_sec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = current_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int32_t ra = os_hal_motor_get_position(0);
    int32_t dec = os_hal_motor_get_position(1);
    steps_to_coord(ra, dec, coord);
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = current_site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pos->ra_steps = os_hal_motor_get_position(0);
    pos->dec_steps = os_hal_motor_get_position(1);
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
    bool tracking_moving = tracking_enabled && current_state == OS_STATE_IDLE_TRACKING;
    *moving = goto_active || park_active || manual_active || guide_pulse.active || tracking_moving;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *locked = gps_locked;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    pec_table = *table;
    pec_valid = table->valid;
    os_error_t save_err = save_pec_to_nvm();
    if (save_err != OS_ERR_NONE) {
        return save_err;
    }
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *table = pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int idx = (int)worm_phase_deg;
    if (idx >= OS_PEC_TABLE_SIZE) {
        idx = 0;
    }
    pec_table.corrections[idx] = error_arcsec;
    pec_table.valid = true;
    pec_valid = true;
    os_error_t save_err = save_pec_to_nvm();
    if (save_err != OS_ERR_NONE) {
        return save_err;
    }
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *calib = calibration_to_public();
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    calib_valid = false;
    calib_m00 = ONSTEP_DEFAULT_ARC_SEC_PER_STEP;
    calib_m01 = 0.0;
    calib_m10 = 0.0;
    calib_m11 = ONSTEP_DEFAULT_ARC_SEC_PER_STEP;
    calib_b0 = 0.0;
    calib_b1 = 0.0;
    os_error_t save_err = save_calibration_to_nvm();
    if (save_err != OS_ERR_NONE) {
        return save_err;
    }
    return OS_ERR_NONE;
}
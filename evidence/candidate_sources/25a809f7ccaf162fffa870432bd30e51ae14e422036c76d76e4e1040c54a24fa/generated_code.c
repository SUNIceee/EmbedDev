#include "generated_code.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OS_NVM_TOTAL_SIZE (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)
#define OS_HAL_COMM_BUF_SIZE 128u
#define OS_LOOP_TICK_MS 10u
#define OS_PEC_MAX_ENTRIES 24u
#define OS_ALIGN_NVM_MAGIC 0x4F4E5354u

typedef struct {
    bool initialized;
    bool enabled;
    bool direction_forward;
    double frequency_hz;
    int32_t position_steps;
    bool fault;
} hal_motor_t;

typedef struct {
    uint8_t rx[OS_HAL_COMM_BUF_SIZE];
    uint8_t tx[OS_HAL_COMM_BUF_SIZE];
    uint8_t rx_head;
    uint8_t rx_tail;
    uint8_t tx_head;
    uint8_t tx_tail;
} hal_comm_t;

typedef struct {
    uint32_t magic;
    bool valid;
    double matrix[2][3];
} align_nvm_t;

typedef struct {
    os_state_t state;
    bool init_done;
    bool tracking_enabled;
    bool moving;
    bool goto_active;
    bool parking_active;
    double tracker_multiplier;
    double guide_rate_multiplier;
    double ra_hours_last;
    double dec_deg_last;
    int32_t goto_target[OS_AXIS_COUNT];
    int32_t park_target[OS_AXIS_COUNT];
    bool manual_active[OS_AXIS_COUNT];
    bool manual_positive[OS_AXIS_COUNT];
    double manual_freq[OS_AXIS_COUNT];
    bool guide_active[OS_AXIS_COUNT];
    bool guide_positive[OS_AXIS_COUNT];
    uint32_t guide_ticks[OS_AXIS_COUNT];
    bool pec_enabled;
    uint8_t pec_count;
    struct {
        double phase_deg;
        double correction_arcsec;
    } pec[OS_PEC_MAX_ENTRIES];
    os_align_star_t align_stars[OS_MAX_ALIGN_STARS];
    uint8_t align_star_count;
    bool align_valid;
    double align_matrix[2][3];
    bool align_matrix_valid;
} onstep_t;

static hal_motor_t g_motor[OS_AXIS_COUNT];
static hal_comm_t g_comm[OS_CHANNEL_COUNT];
static uint8_t g_nvm[OS_NVM_TOTAL_SIZE];

static os_site_info_t g_gps_site;
static uint32_t g_rtc_epoch;
static bool g_gps_valid;
static bool g_rtc_valid;
static bool g_limit[OS_AXIS_COUNT];
static onstep_t g;

static char g_cmd_buf[OS_CHANNEL_COUNT][OS_MAX_COMMAND_LENGTH];
static uint8_t g_cmd_len[OS_CHANNEL_COUNT];

static void reset_runtime_flags(void);
static os_error_t load_nvm_state(void);
static os_error_t save_nvm_state(void);
static void start_tracking(void);
static void stop_motors(void);
static void set_axis_motion(uint8_t axis, double signed_freq_hz);
static void update_planned_move(bool parking);
static void update_axes(void);
static void poll_commands(void);
static double tracking_frequency(void);
static bool qr_solve(int n, int p, const double A[OS_MAX_ALIGN_STARS][3],
                     const double b[OS_MAX_ALIGN_STARS], double x[3]);
static bool solve_mapping(const os_align_star_t *stars, int count,
                          os_align_mode_t mode, double matrix[2][3]);

/* ---------------- HAL: motor ---------------- */

os_error_t os_hal_motor_init(uint8_t axis)
{
    if (axis >= OS_AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_motor[axis].initialized = true;
    g_motor[axis].enabled = false;
    g_motor[axis].direction_forward = true;
    g_motor[axis].frequency_hz = 0.0;
    g_motor[axis].position_steps = 0;
    g_motor[axis].fault = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_frequency(uint8_t axis, double frequency_hz)
{
    if (axis >= OS_AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!(frequency_hz >= 0.0)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_motor[axis].frequency_hz = frequency_hz;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward)
{
    if (axis >= OS_AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_motor[axis].direction_forward = forward;
    return OS_ERR_NONE;
}

os_error_t os_hal_motor_enable(uint8_t axis, bool enable)
{
    if (axis >= OS_AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_motor[axis].enabled = enable;
    if (!enable) {
        g_motor[axis].frequency_hz = 0.0;
    }
    return OS_ERR_NONE;
}

int32_t os_hal_motor_get_position(uint8_t axis)
{
    if (axis >= OS_AXIS_COUNT) {
        return 0;
    }
    return g_motor[axis].position_steps;
}

os_error_t os_hal_timer_motor_init(void)
{
    return OS_ERR_NONE;
}

/* ---------------- HAL: GPS / RTC ---------------- */

os_error_t os_hal_gps_init(void)
{
    g_gps_site.valid = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_gps_poll(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_gps_valid) {
        site->valid = false;
        return OS_ERR_TIMEOUT;
    }
    *site = g_gps_site;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_init(void)
{
    g_rtc_epoch = 0;
    g_rtc_valid = false;
    return OS_ERR_NONE;
}

os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds)
{
    if (utc_epoch_seconds == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_rtc_valid) {
        *utc_epoch_seconds = 0;
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

/* ---------------- HAL: limit ---------------- */

os_error_t os_hal_limit_init(void)
{
    g_limit[0] = false;
    g_limit[1] = false;
    return OS_ERR_NONE;
}

bool os_hal_limit_is_triggered(uint8_t axis)
{
    if (axis >= OS_AXIS_COUNT) {
        return true;
    }
    return g_limit[axis];
}

/* ---------------- HAL: comm ---------------- */

static int ring_count(uint8_t head, uint8_t tail)
{
    return (int)((head - tail + OS_HAL_COMM_BUF_SIZE) % OS_HAL_COMM_BUF_SIZE);
}

static bool ring_push(uint8_t *buf, uint8_t *head, uint8_t *tail, uint8_t byte)
{
    uint8_t next = (uint8_t)((*head + 1u) % OS_HAL_COMM_BUF_SIZE);
    if (next == *tail) {
        return false;
    }
    buf[*head] = byte;
    *head = next;
    return true;
}

static int ring_pop(uint8_t *buf, uint8_t *head, uint8_t *tail)
{
    if (*head == *tail) {
        return -1;
    }
    int byte = buf[*tail];
    *tail = (uint8_t)((*tail + 1u) % OS_HAL_COMM_BUF_SIZE);
    return byte;
}

os_error_t os_hal_comm_init(uint8_t channel)
{
    if (channel >= OS_CHANNEL_COUNT) {
        return OS_ERR_NOT_SUPPORTED;
    }
    g_comm[channel].rx_head = 0;
    g_comm[channel].rx_tail = 0;
    g_comm[channel].tx_head = 0;
    g_comm[channel].tx_tail = 0;
    return OS_ERR_NONE;
}

int16_t os_hal_comm_available(uint8_t channel)
{
    if (channel >= OS_CHANNEL_COUNT) {
        return -1;
    }
    return (int16_t)ring_count(g_comm[channel].rx_head,
                               g_comm[channel].rx_tail);
}

int16_t os_hal_comm_read(uint8_t channel)
{
    if (channel >= OS_CHANNEL_COUNT) {
        return -1;
    }
    int byte = ring_pop(g_comm[channel].rx, &g_comm[channel].rx_head,
                        &g_comm[channel].rx_tail);
    return (int16_t)byte;
}

os_error_t os_hal_comm_write(uint8_t channel, const uint8_t *data, size_t length)
{
    if (channel >= OS_CHANNEL_COUNT) {
        return OS_ERR_NOT_SUPPORTED;
    }
    if (data == NULL && length > 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    for (size_t i = 0; i < length; i++) {
        if (!ring_push(g_comm[channel].tx, &g_comm[channel].tx_head,
                       &g_comm[channel].tx_tail, data[i])) {
            return OS_ERR_TIMEOUT;
        }
    }
    return OS_ERR_NONE;
}

/* ---------------- HAL: NVM ---------------- */

os_error_t os_hal_nvm_init(void)
{
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_read(uint32_t offset, void *data, size_t length)
{
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (offset > OS_NVM_TOTAL_SIZE || length > OS_NVM_TOTAL_SIZE - offset) {
        return OS_ERR_NVM;
    }
    memcpy(data, &g_nvm[offset], length);
    return OS_ERR_NONE;
}

os_error_t os_hal_nvm_write(uint32_t offset, const void *data, size_t length)
{
    if (data == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (offset > OS_NVM_TOTAL_SIZE || length > OS_NVM_TOTAL_SIZE - offset) {
        return OS_ERR_NVM;
    }
    memcpy(&g_nvm[offset], data, length);
    return OS_ERR_NONE;
}

/* ---------------- HAL: buzzer ---------------- */

os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count)
{
    (void)duration_ms;
    (void)count;
    return OS_ERR_NONE;
}

/* ---------------- host test hooks ---------------- */

void os_test_hal_set_gps_fix(bool valid, double lat, double lon,
                             double elev, uint32_t utc)
{
    g_gps_site.valid = valid;
    g_gps_site.latitude_degrees = lat;
    g_gps_site.longitude_degrees = lon;
    g_gps_site.elevation_metres = elev;
    g_gps_site.utc_epoch_seconds = utc;
    g_gps_valid = valid;
    if (valid) {
        g_rtc_epoch = utc;
        g_rtc_valid = true;
    }
}

void os_test_hal_clear_gps(void)
{
    g_gps_valid = false;
    g_gps_site.valid = false;
}

void os_test_hal_set_rtc(uint32_t utc)
{
    g_rtc_epoch = utc;
    g_rtc_valid = true;
}

void os_test_hal_set_limit(uint8_t axis, bool triggered)
{
    if (axis < OS_AXIS_COUNT) {
        g_limit[axis] = triggered;
    }
}

void os_test_hal_set_motor_position(uint8_t axis, int32_t steps)
{
    if (axis < OS_AXIS_COUNT) {
        g_motor[axis].position_steps = steps;
    }
}

void os_test_hal_set_driver_fault(uint8_t axis, bool fault)
{
    if (axis < OS_AXIS_COUNT) {
        g_motor[axis].fault = fault;
    }
}

void os_test_hal_reset_comm(uint8_t channel)
{
    if (channel < OS_CHANNEL_COUNT) {
        g_comm[channel].rx_head = 0;
        g_comm[channel].rx_tail = 0;
        g_comm[channel].tx_head = 0;
        g_comm[channel].tx_tail = 0;
    }
}

size_t os_test_hal_tx_available(uint8_t channel)
{
    if (channel >= OS_CHANNEL_COUNT) {
        return 0;
    }
    return (size_t)ring_count(g_comm[channel].tx_head, g_comm[channel].tx_tail);
}

int os_test_hal_tx_read(uint8_t channel)
{
    if (channel >= OS_CHANNEL_COUNT) {
        return -1;
    }
    return ring_pop(g_comm[channel].tx, &g_comm[channel].tx_head,
                    &g_comm[channel].tx_tail);
}

void os_test_hal_inject_byte(uint8_t channel, uint8_t byte)
{
    if (channel >= OS_CHANNEL_COUNT) {
        return;
    }
    (void)ring_push(g_comm[channel].rx, &g_comm[channel].rx_head,
                    &g_comm[channel].rx_tail, byte);
}

void os_test_hal_inject_command(uint8_t channel, const char *cmd)
{
    if (cmd == NULL) {
        return;
    }
    while (*cmd != '\0') {
        os_test_hal_inject_byte(channel, (uint8_t)*cmd);
        cmd++;
    }
}

void os_test_hal_set_nvm_byte(uint32_t offset, uint8_t value)
{
    if (offset < OS_NVM_TOTAL_SIZE) {
        g_nvm[offset] = value;
    }
}

/* ---------------- internal utilities ---------------- */

static void reset_runtime_flags(void)
{
    memset(&g, 0, sizeof(g));
    g.state = OS_STATE_UNINITIALIZED;
    g.tracker_multiplier = 1.0;
    g.guide_rate_multiplier = OS_DEFAULT_GUIDE_RATE_MULTIPLIER;
    g.pec_enabled = false;
}

static double tracking_frequency(void)
{
    return OS_MOTOR_STEPS_PER_DEGREE * (15.0 / 3600.0) * g.tracker_multiplier;
}

static void stop_motors(void)
{
    for (uint8_t axis = 0; axis < OS_AXIS_COUNT; axis++) {
        (void)os_hal_motor_set_frequency(axis, 0.0);
        (void)os_hal_motor_enable(axis, false);
    }
}

static void start_tracking(void)
{
    g.tracking_enabled = true;
    for (uint8_t axis = 0; axis < OS_AXIS_COUNT; axis++) {
        (void)os_hal_motor_enable(axis, true);
    }
}

static os_error_t load_nvm_state(void)
{
    align_nvm_t rec;
    memset(&rec, 0, sizeof(rec));
    os_error_t err = os_hal_nvm_read(0, &rec, sizeof(rec));
    if (err != OS_ERR_NONE) {
        return OS_ERR_NVM;
    }
    if (rec.magic == OS_ALIGN_NVM_MAGIC && rec.valid) {
        memcpy(g.align_matrix, rec.matrix, sizeof(g.align_matrix));
        g.align_valid = true;
        g.align_matrix_valid = true;
    }
    return OS_ERR_NONE;
}

static os_error_t save_nvm_state(void)
{
    align_nvm_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = OS_ALIGN_NVM_MAGIC;
    rec.valid = g.align_valid;
    memcpy(rec.matrix, g.align_matrix, sizeof(g.align_matrix));
    return os_hal_nvm_write(0, &rec, sizeof(rec));
}

static void set_axis_motion(uint8_t axis, double signed_freq_hz)
{
    if (axis >= OS_AXIS_COUNT) {
        return;
    }

    if (os_hal_limit_is_triggered(axis)) {
        (void)os_hal_motor_set_frequency(axis, 0.0);
        (void)os_hal_motor_enable(axis, false);
        g.manual_active[axis] = false;
        g.guide_active[axis] = false;
        g.goto_active = false;
        g.parking_active = false;
        g.moving = false;
        g.state = OS_STATE_FAULT;
        return;
    }

    if (g_motor[axis].fault) {
        (void)os_hal_motor_set_frequency(axis, 0.0);
        (void)os_hal_motor_enable(axis, false);
        g.state = OS_STATE_FAULT;
        return;
    }

    bool forward = signed_freq_hz >= 0.0;
    double magnitude = fabs(signed_freq_hz);

    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_enable(axis, magnitude > 0.0);
    (void)os_hal_motor_set_frequency(axis, magnitude);

    if (magnitude > 0.0) {
        if (forward) {
            g_motor[axis].position_steps++;
        } else {
            g_motor[axis].position_steps--;
        }
    }
}

static void update_planned_move(bool parking)
{
    int32_t target[OS_AXIS_COUNT];
    if (parking) {
        target[0] = g.park_target[0];
        target[1] = g.park_target[1];
    } else {
        target[0] = g.goto_target[0];
        target[1] = g.goto_target[1];
    }

    bool done = true;
    for (uint8_t axis = 0; axis < OS_AXIS_COUNT; axis++) {
        int32_t current = os_hal_motor_get_position(axis);
        int32_t diff = target[axis] - current;
        if (diff == 0) {
            set_axis_motion(axis, 0.0);
            continue;
        }
        done = false;
        if (os_hal_limit_is_triggered(axis)) {
            set_axis_motion(axis, 0.0);
            g.goto_active = false;
            g.parking_active = false;
            g.moving = false;
            g.state = OS_STATE_FAULT;
            return;
        }
        double signed_freq = (diff > 0) ? OS_DEFAULT_GOTO_FREQ_HZ
                                        : -OS_DEFAULT_GOTO_FREQ_HZ;
        set_axis_motion(axis, signed_freq);
    }

    if (done) {
        if (parking) {
            g.parking_active = false;
            g.moving = false;
            g.state = OS_STATE_PARKED;
            stop_motors();
            (void)os_hal_buzzer_beep(200, 1);
        } else {
            g.goto_active = false;
            g.moving = false;
            g.state = OS_STATE_IDLE_TRACKING;
            start_tracking();
            (void)os_hal_buzzer_beep(100, 2);
        }
    }
}

static void update_axes(void)
{
    if (g.state == OS_STATE_PARKED) {
        return;
    }

    for (uint8_t axis = 0; axis < OS_AXIS_COUNT; axis++) {
        double signed_freq = 0.0;

        if (g.manual_active[axis]) {
            signed_freq = g.manual_positive[axis] ? g.manual_freq[axis]
                                                   : -g.manual_freq[axis];
        } else if (g.guide_active[axis]) {
            double base = (axis == OS_AXIS_RA && g.tracking_enabled)
                              ? tracking_frequency()
                              : 0.0;
            double offset = (g.guide_positive[axis] ? 1.0 : -1.0) *
                            tracking_frequency() * g.guide_rate_multiplier;
            signed_freq = base + offset;
            if (g.guide_ticks[axis] > 0) {
                g.guide_ticks[axis]--;
                if (g.guide_ticks[axis] == 0) {
                    g.guide_active[axis] = false;
                }
            }
        } else if (g.state == OS_STATE_IDLE_TRACKING && g.tracking_enabled &&
                   axis == OS_AXIS_RA) {
            signed_freq = tracking_frequency();
        }

        set_axis_motion(axis, signed_freq);
    }
}

static void poll_commands(void)
{
    for (uint8_t ch = 0; ch < OS_CHANNEL_COUNT; ch++) {
        while (os_hal_comm_available(ch) > 0) {
            int16_t byte = os_hal_comm_read(ch);
            if (byte < 0) {
                break;
            }
            uint8_t c = (uint8_t)byte;

            if (c == '\r' || c == '\n') {
                if (g_cmd_len[ch] > 0) {
                    if (g_cmd_buf[ch][0] == ':' &&
                        g_cmd_buf[ch][g_cmd_len[ch] - 1] != '#') {
                        g_cmd_buf[ch][g_cmd_len[ch]++] = '#';
                    }
                    g_cmd_buf[ch][g_cmd_len[ch]] = '\0';

                    char reply[OS_MAX_REPLY_LENGTH];
                    (void)os_command_receive(ch, g_cmd_buf[ch], reply,
                                             sizeof(reply));
                    (void)os_hal_comm_write(ch, (const uint8_t *)reply,
                                            strlen(reply));
                    g_cmd_len[ch] = 0;
                }
                continue;
            }

            if (g_cmd_len[ch] >= OS_MAX_COMMAND_LENGTH - 1) {
                char reply[OS_MAX_REPLY_LENGTH];
                (void)os_command_receive(ch, "", reply, sizeof(reply));
                (void)os_hal_comm_write(ch, (const uint8_t *)reply,
                                        strlen(reply));
                g_cmd_len[ch] = 0;
                continue;
            }

            g_cmd_buf[ch][g_cmd_len[ch]++] = c;

            if (c == '#') {
                g_cmd_buf[ch][g_cmd_len[ch]] = '\0';
                char reply[OS_MAX_REPLY_LENGTH];
                (void)os_command_receive(ch, g_cmd_buf[ch], reply,
                                         sizeof(reply));
                (void)os_hal_comm_write(ch, (const uint8_t *)reply,
                                        strlen(reply));
                g_cmd_len[ch] = 0;
            }
        }
    }
}

static bool qr_solve(int n, int p, const double A[OS_MAX_ALIGN_STARS][3],
                     const double b[OS_MAX_ALIGN_STARS], double x[3])
{
    double q[OS_MAX_ALIGN_STARS][3];
    double r[3][3];
    memset(r, 0, sizeof(r));

    for (int j = 0; j < p; j++) {
        double v[OS_MAX_ALIGN_STARS];
        for (int i = 0; i < n; i++) {
            v[i] = A[i][j];
        }

        for (int k = 0; k < j; k++) {
            double dot = 0.0;
            for (int i = 0; i < n; i++) {
                dot += q[i][k] * v[i];
            }
            r[k][j] = dot;
            for (int i = 0; i < n; i++) {
                v[i] -= dot * q[i][k];
            }
        }

        double norm = 0.0;
        for (int i = 0; i < n; i++) {
            norm += v[i] * v[i];
        }
        norm = sqrt(norm);
        if (norm < OS_ALIGN_DEGENERATE_DET) {
            return false;
        }

        r[j][j] = norm;
        for (int i = 0; i < n; i++) {
            q[i][j] = v[i] / norm;
        }
    }

    double y[3] = {0.0, 0.0, 0.0};
    for (int k = 0; k < p; k++) {
        double dot = 0.0;
        for (int i = 0; i < n; i++) {
            dot += q[i][k] * b[i];
        }
        y[k] = dot;
    }

    for (int i = p - 1; i >= 0; i--) {
        double sum = y[i];
        for (int j = i + 1; j < p; j++) {
            sum -= r[i][j] * x[j];
        }
        x[i] = sum / r[i][i];
    }

    return true;
}

static bool solve_mapping(const os_align_star_t *stars, int count,
                          os_align_mode_t mode, double matrix[2][3])
{
    if (stars == NULL) {
        return false;
    }

    int p = (mode == OS_ALIGN_1STAR) ? 1 : (mode == OS_ALIGN_2STAR) ? 2 : 3;

    for (int axis = 0; axis < OS_AXIS_COUNT; axis++) {
        double A[OS_MAX_ALIGN_STARS][3];
        double b[OS_MAX_ALIGN_STARS];
        memset(A, 0, sizeof(A));
        memset(b, 0, sizeof(b));

        for (int i = 0; i < count; i++) {
            double ra = stars[i].ra_hours;
            double dec = stars[i].dec_deg;

            if (mode == OS_ALIGN_1STAR) {
                A[i][0] = 1.0;
            } else if (mode == OS_ALIGN_2STAR) {
                if (axis == 0) {
                    A[i][0] = ra;
                    A[i][1] = 1.0;
                } else {
                    A[i][0] = dec;
                    A[i][1] = 1.0;
                }
            } else {
                A[i][0] = ra;
                A[i][1] = dec;
                A[i][2] = 1.0;
            }

            b[i] = (axis == 0) ? stars[i].pos[0] : stars[i].pos[1];
        }

        double x[3] = {0.0, 0.0, 0.0};
        if (!qr_solve(count, p, A, b, x)) {
            return false;
        }

        if (mode == OS_ALIGN_1STAR) {
            matrix[axis][0] = 0.0;
            matrix[axis][1] = 0.0;
            matrix[axis][2] = x[0];
        } else if (mode == OS_ALIGN_2STAR) {
            if (axis == 0) {
                matrix[0][0] = x[0];
                matrix[0][1] = 0.0;
                matrix[0][2] = x[1];
            } else {
                matrix[1][0] = 0.0;
                matrix[1][1] = x[0];
                matrix[1][2] = x[1];
            }
        } else {
            matrix[axis][0] = x[0];
            matrix[axis][1] = x[1];
            matrix[axis][2] = x[2];
        }
    }

    return true;
}

/* ---------------- Public API ---------------- */

os_error_t os_init(void)
{
    reset_runtime_flags();
    g.state = OS_STATE_INIT;

    (void)os_hal_nvm_init();
    (void)load_nvm_state();

    for (uint8_t ch = 0; ch < OS_CHANNEL_COUNT; ch++) {
        (void)os_hal_comm_init(ch);
    }

    for (uint8_t axis = 0; axis < OS_AXIS_COUNT; axis++) {
        os_error_t err = os_hal_motor_init(axis);
        if (err != OS_ERR_NONE) {
            g.state = OS_STATE_FAULT;
            return OS_ERR_DRIVER;
        }
        (void)os_hal_motor_enable(axis, false);
        (void)os_hal_motor_set_frequency(axis, 0.0);
    }

    (void)os_hal_gps_init();
    (void)os_hal_rtc_init();
    (void)os_hal_limit_init();
    (void)os_hal_timer_motor_init();

    bool time_available = false;

    os_site_info_t site;
    os_error_t gps_err = os_hal_gps_poll(&site);
    if (gps_err == OS_ERR_NONE && site.valid) {
        g_gps_site = site;
        g_gps_valid = true;
        (void)os_hal_rtc_set(site.utc_epoch_seconds);
        time_available = true;
    } else {
        uint32_t utc = 0;
        if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
            g_rtc_epoch = utc;
            g_rtc_valid = true;
            time_available = true;
        }
    }

    if (!time_available) {
        g.state = OS_STATE_FAULT;
        stop_motors();
        return OS_ERR_RTC;
    }

    g.state = OS_STATE_IDLE_TRACKING;
    g.init_done = true;
    start_tracking();
    return OS_ERR_NONE;
}

void os_loop_iteration(void)
{
    if (!g.init_done) {
        return;
    }

    poll_commands();

    os_site_info_t site;
    if (os_hal_gps_poll(&site) == OS_ERR_NONE && site.valid) {
        g_gps_site = site;
        g_gps_valid = true;
        (void)os_hal_rtc_set(site.utc_epoch_seconds);
    }

    if (g.state == OS_STATE_PARKED) {
        return;
    }

    if (g.goto_active) {
        update_planned_move(false);
    } else if (g.parking_active) {
        update_planned_move(true);
    } else {
        update_axes();
    }
}

os_error_t os_goto_equatorial(double ra_hours, double dec_deg)
{
    if (!(ra_hours >= 0.0 && ra_hours <= 24.0)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!(dec_deg >= -90.0 && dec_deg <= 90.0)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (os_hal_limit_is_triggered(OS_AXIS_RA) ||
        os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT;
    }

    int32_t target[OS_AXIS_COUNT];
    if (g.align_valid && g.align_matrix_valid) {
        target[0] = (int32_t)lround(g.align_matrix[0][0] * ra_hours +
                                    g.align_matrix[0][1] * dec_deg +
                                    g.align_matrix[0][2]);
        target[1] = (int32_t)lround(g.align_matrix[1][0] * ra_hours +
                                    g.align_matrix[1][1] * dec_deg +
                                    g.align_matrix[1][2]);
    } else {
        target[0] = (int32_t)lround(ra_hours * 15.0 * OS_MOTOR_STEPS_PER_DEGREE);
        target[1] = (int32_t)lround(dec_deg * OS_MOTOR_STEPS_PER_DEGREE);
    }

    if (target[0] == os_hal_motor_get_position(OS_AXIS_RA) &&
        target[1] == os_hal_motor_get_position(OS_AXIS_DEC)) {
        g.ra_hours_last = ra_hours;
        g.dec_deg_last = dec_deg;
        g.goto_active = false;
        g.parking_active = false;
        g.moving = false;
        g.state = OS_STATE_IDLE_TRACKING;
        (void)os_hal_buzzer_beep(100, 1);
        return OS_ERR_NONE;
    }

    g.ra_hours_last = ra_hours;
    g.dec_deg_last = dec_deg;
    g.goto_target[0] = target[0];
    g.goto_target[1] = target[1];
    g.goto_active = true;
    g.parking_active = false;
    g.moving = true;
    g.state = OS_STATE_GOTO;
    start_tracking();
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void)
{
    g.goto_active = false;
    g.parking_active = false;
    g.moving = false;
    g.state = OS_STATE_IDLE_TRACKING;
    (void)os_hal_motor_set_frequency(0, 0.0);
    (void)os_hal_motor_set_frequency(1, 0.0);
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *is_moving)
{
    if (is_moving == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *is_moving = g.moving;
    return OS_ERR_NONE;
}

os_error_t os_get_state(os_state_t *state)
{
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *state = g.state;
    return OS_ERR_NONE;
}

os_error_t os_set_tracking_rate(double multiplier)
{
    if (!(multiplier > 0.0)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g.tracker_multiplier = multiplier;
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_guide_direction_t direction, uint32_t duration_ms)
{
    if (direction != OS_GUIDE_DIR_EAST &&
        direction != OS_GUIDE_DIR_WEST &&
        direction != OS_GUIDE_DIR_NORTH &&
        direction != OS_GUIDE_DIR_SOUTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (duration_ms == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    uint8_t axis;
    bool positive;
    if (direction == OS_GUIDE_DIR_EAST) {
        axis = OS_AXIS_RA;
        positive = true;
    } else if (direction == OS_GUIDE_DIR_WEST) {
        axis = OS_AXIS_RA;
        positive = false;
    } else if (direction == OS_GUIDE_DIR_NORTH) {
        axis = OS_AXIS_DEC;
        positive = true;
    } else {
        axis = OS_AXIS_DEC;
        positive = false;
    }

    g.guide_active[axis] = true;
    g.guide_positive[axis] = positive;
    g.guide_ticks[axis] = duration_ms / OS_LOOP_TICK_MS;
    if (g.guide_ticks[axis] == 0) {
        g.guide_ticks[axis] = 1;
    }
    return OS_ERR_NONE;
}

os_error_t os_align_add_star(double ra_hours, double dec_deg)
{
    if (!(ra_hours >= 0.0 && ra_hours <= 24.0)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!(dec_deg >= -90.0 && dec_deg <= 90.0)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g.align_star_count >= OS_MAX_ALIGN_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    os_align_star_t *star = &g.align_stars[g.align_star_count];
    star->ra_hours = ra_hours;
    star->dec_deg = dec_deg;
    star->pos[0] = os_hal_motor_get_position(OS_AXIS_RA);
    star->pos[1] = os_hal_motor_get_position(OS_AXIS_DEC);
    g.align_star_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(os_align_mode_t mode)
{
    int required = 0;
    if (mode == OS_ALIGN_1STAR) {
        required = 1;
    } else if (mode == OS_ALIGN_2STAR) {
        required = 2;
    } else if (mode == OS_ALIGN_3STAR || mode == OS_ALIGN_NSTAR) {
        required = 3;
    } else {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (g.align_star_count < required) {
        return OS_ERR_INVALID_STATE;
    }

    double matrix[2][3];
    memset(matrix, 0, sizeof(matrix));
    if (!solve_mapping(g.align_stars, g.align_star_count, mode, matrix)) {
        return OS_ERR_INVALID_STATE;
    }

    if (g.align_star_count >= 4) {
        double max_resid_arcsec = 0.0;
        for (uint8_t i = 0; i < g.align_star_count; i++) {
            double ra = g.align_stars[i].ra_hours;
            double dec = g.align_stars[i].dec_deg;
            double pred0 = matrix[0][0] * ra + matrix[0][1] * dec + matrix[0][2];
            double pred1 = matrix[1][0] * ra + matrix[1][1] * dec + matrix[1][2];
            double resid_steps =
                sqrt((pred0 - g.align_stars[i].pos[0]) *
                         (pred0 - g.align_stars[i].pos[0]) +
                     (pred1 - g.align_stars[i].pos[1]) *
                         (pred1 - g.align_stars[i].pos[1]));
            double resid_arcsec =
                resid_steps / OS_MOTOR_STEPS_PER_DEGREE * 3600.0;
            if (resid_arcsec > max_resid_arcsec) {
                max_resid_arcsec = resid_arcsec;
            }
        }
        if (max_resid_arcsec > OS_ALIGN_MAX_RESIDUAL_ARCSEC) {
            return OS_ERR_INVALID_STATE;
        }
    }

    memcpy(g.align_matrix, matrix, sizeof(matrix));
    g.align_valid = true;
    g.align_matrix_valid = true;
    (void)save_nvm_state();
    return OS_ERR_NONE;
}

os_error_t os_align_clear(void)
{
    g.align_star_count = 0;
    g.align_valid = false;
    g.align_matrix_valid = false;
    memset(g.align_matrix, 0, sizeof(g.align_matrix));
    (void)save_nvm_state();
    return OS_ERR_NONE;
}

os_error_t os_park(void)
{
    if (os_hal_limit_is_triggered(OS_AXIS_RA) ||
        os_hal_limit_is_triggered(OS_AXIS_DEC)) {
        return OS_ERR_LIMIT;
    }

    g.park_target[0] = 0;
    g.park_target[1] = 0;
    g.goto_active = false;
    g.parking_active = true;
    g.moving = true;
    g.state = OS_STATE_PARKING;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void)
{
    for (uint8_t ch = 0; ch < OS_CHANNEL_COUNT; ch++) {
        (void)os_hal_comm_init(ch);
    }

    os_site_info_t site;
    if (os_hal_gps_poll(&site) == OS_ERR_NONE && site.valid) {
        g_gps_site = site;
        g_gps_valid = true;
        (void)os_hal_rtc_set(site.utc_epoch_seconds);
    }

    for (uint8_t axis = 0; axis < OS_AXIS_COUNT; axis++) {
        (void)os_hal_motor_enable(axis, true);
        (void)os_hal_motor_set_frequency(axis, 0.0);
    }

    g.parking_active = false;
    g.goto_active = false;
    g.moving = false;
    g.state = OS_STATE_IDLE_TRACKING;
    start_tracking();
    return OS_ERR_NONE;
}

os_error_t os_move_axis(uint8_t axis, bool positive, double frequency_hz)
{
    if (axis >= OS_AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!(frequency_hz >= 0.0)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT;
    }

    g.manual_active[axis] = true;
    g.manual_positive[axis] = positive;
    g.manual_freq[axis] = frequency_hz;
    g.moving = true;
    g.state = OS_STATE_MANUAL_MOVING;
    (void)os_hal_motor_enable(axis, true);
    return OS_ERR_NONE;
}

os_error_t os_move_stop(uint8_t axis)
{
    if (axis >= OS_AXIS_COUNT) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    g.manual_active[axis] = false;
    g.manual_freq[axis] = 0.0;
    (void)os_hal_motor_set_frequency(axis, 0.0);

    bool any_manual = false;
    for (uint8_t a = 0; a < OS_AXIS_COUNT; a++) {
        if (g.manual_active[a]) {
            any_manual = true;
            break;
        }
    }
    if (!any_manual && !g.goto_active && !g.parking_active) {
        g.moving = false;
        g.state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_pec_set_entry(double worm_phase_deg, double correction_arcsec)
{
    if (!(worm_phase_deg >= 0.0 && worm_phase_deg <= 360.0)) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (g.pec_count >= OS_PEC_MAX_ENTRIES) {
        return OS_ERR_INVALID_STATE;
    }
    g.pec[g.pec_count].phase_deg = worm_phase_deg;
    g.pec[g.pec_count].correction_arcsec = correction_arcsec;
    g.pec_count++;
    g.pec_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_pec_clear(void)
{
    g.pec_count = 0;
    g.pec_enabled = false;
    return OS_ERR_NONE;
}

static void current_ra_dec(double *ra_hours, double *dec_deg)
{
    int32_t p0 = os_hal_motor_get_position(OS_AXIS_RA);
    int32_t p1 = os_hal_motor_get_position(OS_AXIS_DEC);

    if (g.align_matrix_valid) {
        double det = g.align_matrix[0][0] * g.align_matrix[1][1] -
                     g.align_matrix[0][1] * g.align_matrix[1][0];
        if (fabs(det) > 1e-12) {
            double s0 = (double)p0 - g.align_matrix[0][2];
            double s1 = (double)p1 - g.align_matrix[1][2];
            double ra = (g.align_matrix[1][1] * s0 -
                         g.align_matrix[0][1] * s1) / det;
            double dec = (-g.align_matrix[1][0] * s0 +
                          g.align_matrix[0][0] * s1) / det;
            ra = fmod(ra, 24.0);
            if (ra < 0.0) {
                ra += 24.0;
            }
            *ra_hours = ra;
            *dec_deg = dec;
            return;
        }
    }

    double ra = (double)p0 / (15.0 * OS_MOTOR_STEPS_PER_DEGREE);
    double dec = (double)p1 / OS_MOTOR_STEPS_PER_DEGREE;
    ra = fmod(ra, 24.0);
    if (ra < 0.0) {
        ra += 24.0;
    }
    *ra_hours = ra;
    *dec_deg = dec;
}

os_error_t os_command_receive(uint8_t source_channel, const char *command,
                              char *reply, size_t reply_len)
{
    (void)source_channel;
    if (command == NULL || reply == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (reply_len == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    reply[0] = '\0';

    size_t len = strlen(command);
    if (len < 3) {
        return OS_ERR_COMMAND_FORMAT;
    }
    if (command[0] != ':' || command[len - 1] != '#') {
        return OS_ERR_COMMAND_FORMAT;
    }

    const char *cmd = command + 1;
    size_t clen = len - 2;

    if (clen == 2 && strncmp(cmd, "GR", 2) == 0) {
        double ra = 0.0, dec = 0.0;
        current_ra_dec(&ra, &dec);
        snprintf(reply, reply_len, "%.6f", ra);
        return OS_ERR_NONE;
    }

    if (clen == 2 && strncmp(cmd, "GD", 2) == 0) {
        double ra = 0.0, dec = 0.0;
        current_ra_dec(&ra, &dec);
        snprintf(reply, reply_len, "%.6f", dec);
        return OS_ERR_NONE;
    }

    if (clen == 3 && strncmp(cmd, "GVP", 3) == 0) {
        snprintf(reply, reply_len, "OnStep generated_code 1.0.0");
        return OS_ERR_NONE;
    }

    if (clen == 2 && strncmp(cmd, "MS", 2) == 0) {
        os_error_t err = os_goto_equatorial(g.ra_hours_last, g.dec_deg_last);
        snprintf(reply, reply_len, "%d", err == OS_ERR_NONE ? 1 : 0);
        return err;
    }

    if (clen == 2 && strncmp(cmd, "Me", 2) == 0) {
        os_error_t err = os_move_axis(OS_AXIS_RA, true, 100.0);
        snprintf(reply, reply_len, "%d", err == OS_ERR_NONE ? 1 : 0);
        return err;
    }
    if (clen == 2 && strncmp(cmd, "Mw", 2) == 0) {
        os_error_t err = os_move_axis(OS_AXIS_RA, false, 100.0);
        snprintf(reply, reply_len, "%d", err == OS_ERR_NONE ? 1 : 0);
        return err;
    }
    if (clen == 2 && strncmp(cmd, "Mn", 2) == 0) {
        os_error_t err = os_move_axis(OS_AXIS_DEC, true, 100.0);
        snprintf(reply, reply_len, "%d", err == OS_ERR_NONE ? 1 : 0);
        return err;
    }
    if (clen == 2 && strncmp(cmd, "Ms", 2) == 0) {
        os_error_t err = os_move_axis(OS_AXIS_DEC, false, 100.0);
        snprintf(reply, reply_len, "%d", err == OS_ERR_NONE ? 1 : 0);
        return err;
    }

    if (clen == 1 && cmd[0] == 'Q') {
        (void)os_move_stop(OS_AXIS_RA);
        (void)os_move_stop(OS_AXIS_DEC);
        snprintf(reply, reply_len, "1");
        return OS_ERR_NONE;
    }

    if (clen == 2 && strncmp(cmd, "hP", 2) == 0) {
        os_error_t err = os_park();
        snprintf(reply, reply_len, "%d", err == OS_ERR_NONE ? 1 : 0);
        return err;
    }
    if (clen == 2 && strncmp(cmd, "hO", 2) == 0) {
        os_error_t err = os_unpark();
        snprintf(reply, reply_len, "%d", err == OS_ERR_NONE ? 1 : 0);
        return err;
    }

    if (clen == 3 && strncmp(cmd, "Mg", 2) == 0) {
        os_guide_direction_t dir;
        if (cmd[2] == 'E') {
            dir = OS_GUIDE_DIR_EAST;
        } else if (cmd[2] == 'W') {
            dir = OS_GUIDE_DIR_WEST;
        } else if (cmd[2] == 'N') {
            dir = OS_GUIDE_DIR_NORTH;
        } else if (cmd[2] == 'S') {
            dir = OS_GUIDE_DIR_SOUTH;
        } else {
            return OS_ERR_COMMAND_FORMAT;
        }
        os_error_t err = os_guide_pulse(dir, 500);
        snprintf(reply, reply_len, "%d", err == OS_ERR_NONE ? 1 : 0);
        return err;
    }

    if (clen > 2 && strncmp(cmd, "Sr", 2) == 0) {
        double ra = strtod(cmd + 2, NULL);
        if (ra >= 0.0 && ra <= 24.0) {
            g.ra_hours_last = ra;
            snprintf(reply, reply_len, "1");
            return OS_ERR_NONE;
        }
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (clen > 2 && strncmp(cmd, "Sd", 2) == 0) {
        double dec = strtod(cmd + 2, NULL);
        if (dec >= -90.0 && dec <= 90.0) {
            g.dec_deg_last = dec;
            snprintf(reply, reply_len, "1");
            return OS_ERR_NONE;
        }
        return OS_ERR_INVALID_ARGUMENT;
    }

    snprintf(reply, reply_len, "unsupported");
    return OS_ERR_NOT_SUPPORTED;
}

os_error_t os_get_site(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    *site = g_gps_site;
    if (!g_gps_valid) {
        site->valid = false;
    }
    return OS_ERR_NONE;
}

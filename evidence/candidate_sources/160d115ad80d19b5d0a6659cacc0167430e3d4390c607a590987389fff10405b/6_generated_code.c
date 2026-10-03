#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define AXES 2u
#define CHANNELS 4u
#define ARCSEC_PER_STEP 3.6
#define NVM_BYTES 1024u
#define CAL_MAGIC 0x4f534341u
#define CFG_MAGIC 0x4f534346u
#define PEC_MAGIC 0x4f535045u
#define LOOP_MS 10u

#if defined(__GNUC__)
#define OS_WEAK __attribute__((weak))
#else
#define OS_WEAK
#endif

typedef struct { char data[OS_MAX_COMMAND_LENGTH]; size_t length; bool receiving; } command_buffer_t;
typedef struct { uint32_t magic; os_calibration_t calibration; uint32_t checksum; } calibration_record_t;
typedef struct { uint32_t magic; os_equatorial_coord_t park; uint32_t checksum; } config_record_t;
typedef struct { uint32_t magic; os_pec_table_t table; bool enabled; uint32_t checksum; } pec_record_t;

typedef struct {
    os_state_t state;
    bool initialized;
    bool tracking;
    bool goto_active;
    bool park_active;
    bool manual_active;
    bool gps_locked;
    bool residual_valid;
    bool rtc_valid;
    os_track_rate_t track_rate;
    float custom_factor;
    float guide_rate;
    float manual_speed;
    float residual_arcsec;
    uint32_t guide_remaining_ms;
    os_guide_pulse_t guide;
    os_site_info_t site;
    os_calibration_t calibration;
    os_equatorial_coord_t park_position;
    os_equatorial_coord_t pending_target;
    bool pending_ra;
    bool pending_dec;
    os_pec_table_t pec;
    bool pec_enabled;
    os_align_mode_t align_mode;
    uint8_t star_count;
    os_equatorial_coord_t stars[OS_CALIBRATION_MAX_STARS];
    os_motor_position_t star_positions[OS_CALIBRATION_MAX_STARS];
    int32_t target[AXES];
    bool direction[AXES];
    command_buffer_t commands[CHANNELS];
} onstep_context_t;

static onstep_context_t g;

static int32_t hal_position[AXES];
static uint32_t hal_frequency[AXES];
static bool hal_direction[AXES];
static bool hal_enabled[AXES];
static bool hal_initialized[AXES];
static bool hal_limits[AXES];
static uint8_t hal_rx[CHANNELS][256];
static size_t hal_rx_head[CHANNELS], hal_rx_tail[CHANNELS];
static char hal_tx[CHANNELS][512];
static size_t hal_tx_length[CHANNELS];
static uint8_t hal_nvm[NVM_BYTES];
static bool hal_nvm_ready;
static uint32_t hal_rtc;

static bool finite_float(float x) { return x == x && x < INFINITY && x > -INFINITY; }
static bool valid_equatorial(os_equatorial_coord_t c) {
    return finite_float(c.ra_hours) && finite_float(c.dec_degrees) && c.ra_hours >= OS_RA_MIN_HOURS && c.ra_hours <= OS_RA_MAX_HOURS && c.dec_degrees >= OS_DEC_MIN_DEG && c.dec_degrees <= OS_DEC_MAX_DEG;
}
static bool valid_horizontal(os_horizontal_coord_t c) {
    return finite_float(c.azimuth_degrees) && finite_float(c.altitude_degrees) && c.azimuth_degrees >= 0.0f && c.azimuth_degrees <= 360.0f && c.altitude_degrees >= -90.0f && c.altitude_degrees <= 90.0f;
}
static bool valid_direction(os_direction_t d) { return d >= OS_DIRECTION_NORTH && d <= OS_DIRECTION_WEST; }
static uint32_t checksum(const void *data, size_t length) {
    const uint8_t *p = (const uint8_t *)data;
    uint32_t s = 2166136261u;
    while (length--) s = (s ^ *p++) * 16777619u;
    return s;
}
static void calibration_default(void) {
    memset(&g.calibration, 0, sizeof g.calibration);
    g.calibration.matrix_ra_to_ra = 1.0f;
    g.calibration.matrix_dec_to_dec = 1.0f;
}
static void stop_axis(uint8_t axis) {
    if (axis < AXES) {
        (void)os_hal_motor_set_frequency(axis, 0);
        (void)os_hal_motor_enable(axis, false);
    }
}
static void stop_all(void) { stop_axis(0); stop_axis(1); }
static os_error_t check_limit(uint8_t axis, bool forward) {
    (void)forward;
    if (axis >= AXES || os_hal_limit_is_triggered(axis)) return OS_ERR_LIMIT_TRIGGERED;
    return OS_ERR_NONE;
}
static void persist_state(void) {
    calibration_record_t cr;
    config_record_t cf;
    pec_record_t pr;
    memset(&cr, 0, sizeof cr); cr.magic = CAL_MAGIC; cr.calibration = g.calibration; cr.checksum = checksum(&cr, sizeof cr - sizeof cr.checksum);
    memset(&cf, 0, sizeof cf); cf.magic = CFG_MAGIC; cf.park = g.park_position; cf.checksum = checksum(&cf, sizeof cf - sizeof cf.checksum);
    memset(&pr, 0, sizeof pr); pr.magic = PEC_MAGIC; pr.table = g.pec; pr.enabled = g.pec_enabled; pr.checksum = checksum(&pr, sizeof pr - sizeof pr.checksum);
    (void)os_hal_nvm_write(0, (const uint8_t *)&cr, (uint16_t)sizeof cr);
    (void)os_hal_nvm_write(OS_NVM_CALIBRATION_SIZE_BYTES, (const uint8_t *)&cf, (uint16_t)sizeof cf);
    (void)os_hal_nvm_write(512, (const uint8_t *)&pr, (uint16_t)sizeof pr);
}
static void restore_state(void) {
    calibration_record_t cr;
    config_record_t cf;
    pec_record_t pr;
    if (os_hal_nvm_read(0, (uint8_t *)&cr, (uint16_t)sizeof cr) == OS_ERR_NONE && cr.magic == CAL_MAGIC && cr.checksum == checksum(&cr, sizeof cr - sizeof cr.checksum) && cr.calibration.valid) g.calibration = cr.calibration;
    if (os_hal_nvm_read(OS_NVM_CALIBRATION_SIZE_BYTES, (uint8_t *)&cf, (uint16_t)sizeof cf) == OS_ERR_NONE && cf.magic == CFG_MAGIC && cf.checksum == checksum(&cf, sizeof cf - sizeof cf.checksum) && valid_equatorial(cf.park)) g.park_position = cf.park;
    if (os_hal_nvm_read(512, (uint8_t *)&pr, (uint16_t)sizeof pr) == OS_ERR_NONE && pr.magic == PEC_MAGIC && pr.checksum == checksum(&pr, sizeof pr - sizeof pr.checksum) && pr.table.valid) { g.pec = pr.table; g.pec_enabled = pr.enabled; }
}
static void reset_runtime(void) {
    memset(&g, 0, sizeof g);
    g.state = OS_STATE_INITIALIZING;
    g.tracking = true;
    g.track_rate = OS_TRACK_RATE_SIDEREAL;
    g.custom_factor = 1.0f;
    g.guide_rate = 0.5f;
    g.manual_speed = 360.0f;
    g.park_position.ra_hours = 0.0f;
    g.park_position.dec_degrees = 90.0f;
    g.site.valid = false;
    calibration_default();
}
static void site_update(void) {
    os_site_info_t s;
    uint32_t epoch;
    if (os_hal_gps_poll(&s) == OS_ERR_NONE && s.valid && s.latitude_degrees >= -90.0f && s.latitude_degrees <= 90.0f && s.longitude_degrees >= -180.0f && s.longitude_degrees <= 180.0f) {
        g.site = s; g.gps_locked = true; g.rtc_valid = os_hal_rtc_set(s.utc_epoch_seconds) == OS_ERR_NONE;
    } else {
        g.gps_locked = false;
        if (os_hal_rtc_read(&epoch) == OS_ERR_NONE) { g.site.utc_epoch_seconds = epoch; g.site.valid = true; g.rtc_valid = true; }
        else g.rtc_valid = false;
    }
}
static void coordinates_to_steps(os_equatorial_coord_t c, int32_t *ra, int32_t *dec) {
    double x = (double)c.ra_hours * 54000.0;
    double y = (double)c.dec_degrees * 3600.0;
    double a = g.calibration.matrix_ra_to_ra * x + g.calibration.matrix_dec_to_ra * y + g.calibration.offset_ra_arcsec;
    double b = g.calibration.matrix_ra_to_dec * x + g.calibration.matrix_dec_to_dec * y + g.calibration.offset_dec_arcsec;
    *ra = (int32_t)(a / ARCSEC_PER_STEP + (a >= 0.0 ? 0.5 : -0.5));
    *dec = (int32_t)(b / ARCSEC_PER_STEP + (b >= 0.0 ? 0.5 : -0.5));
}
static bool steps_to_coordinates(os_motor_position_t p, os_equatorial_coord_t *c) {
    double a = g.calibration.matrix_ra_to_ra, b = g.calibration.matrix_dec_to_ra;
    double d = g.calibration.matrix_dec_to_dec, e = g.calibration.matrix_ra_to_dec;
    double determinant = a * d - b * e;
    double x, y, ra, dec;
    if (fabs(determinant) < 1.0e-12) return false;
    x = p.ra_steps * ARCSEC_PER_STEP - g.calibration.offset_ra_arcsec;
    y = p.dec_steps * ARCSEC_PER_STEP - g.calibration.offset_dec_arcsec;
    ra = (d * x - b * y) / determinant / 54000.0;
    dec = (-e * x + a * y) / determinant / 3600.0;
    while (ra < 0.0) ra += 24.0;
    while (ra >= 24.0) ra -= 24.0;
    c->ra_hours = (float)ra;
    c->dec_degrees = (float)dec;
    if (c->dec_degrees < -90.0f) c->dec_degrees = -90.0f;
    if (c->dec_degrees > 90.0f) c->dec_degrees = 90.0f;
    return true;
}
static void begin_motion(int32_t ra, int32_t dec, bool parking) {
    uint8_t axis;
    g.target[0] = ra; g.target[1] = dec; g.goto_active = !parking; g.park_active = parking; g.manual_active = false; g.state = OS_STATE_GOTO;
    for (axis = 0; axis < AXES; ++axis) {
        int32_t current = os_hal_motor_get_position(axis);
        g.direction[axis] = (axis == 0 ? ra : dec) >= current;
        if (current == (axis == 0 ? ra : dec)) stop_axis(axis);
        else { (void)os_hal_motor_set_direction(axis, g.direction[axis]); (void)os_hal_motor_enable(axis, true); (void)os_hal_motor_set_frequency(axis, 3000); }
    }
}
static void complete_motion(void) {
    bool complete = true;
    uint8_t axis;
    for (axis = 0; axis < AXES; ++axis) {
        int32_t p = os_hal_motor_get_position(axis);
        if (os_hal_limit_is_triggered(axis)) { stop_all(); g.goto_active = false; g.park_active = false; g.manual_active = false; g.state = OS_STATE_FAULT; return; }
        if ((g.direction[axis] && p >= g.target[axis]) || (!g.direction[axis] && p <= g.target[axis])) stop_axis(axis);
        else complete = false;
    }
    if (complete) {
        stop_all();
        if (g.park_active) { g.park_active = false; g.goto_active = false; g.tracking = false; g.state = OS_STATE_PARKED; }
        else if (g.goto_active) { g.goto_active = false; g.state = OS_STATE_IDLE_TRACKING; (void)os_hal_buzzer_beep(80, 1); }
    }
}
static uint32_t tracking_frequency(void) {
    double factor = 1.0;
    if (g.track_rate == OS_TRACK_RATE_LUNAR) factor = OS_LUNAR_RATE_FACTOR;
    else if (g.track_rate == OS_TRACK_RATE_SOLAR) factor = OS_SOLAR_RATE_FACTOR;
    else if (g.track_rate == OS_TRACK_RATE_CUSTOM) factor = g.custom_factor;
    if (g.pec_enabled && g.pec.valid) factor += (double)g.pec.corrections[0] / 3600.0;
    if (factor < 0.0) factor = 0.0;
    return (uint32_t)(OS_SIDEREAL_RATE_ARCSEC_PER_SEC * factor / ARCSEC_PER_STEP + 0.5);
}
static int reply_format(char *buffer, size_t size, size_t *length, const char *format, ...) {
    va_list ap;
    int n;
    va_start(ap, format); n = vsnprintf(buffer, size, format, ap); va_end(ap);
    if (n < 0 || (size_t)n >= size) return 0;
    *length = (size_t)n;
    return 1;
}
static bool parse_ra(const char *s, size_t n, float *out) {
    int h, m, sec = 0;
    char extra;
    if (n < 4 || sscanf(s, "%d:%d:%d%c", &h, &m, &sec, &extra) < 2) return false;
    if (h < 0 || h > 24 || m < 0 || m >= 60 || sec < 0 || sec >= 60) return false;
    *out = h + m / 60.0f + sec / 3600.0f;
    return *out >= 0.0f && *out <= 24.0f;
}
static bool parse_dec(const char *s, size_t n, float *out) {
    char sign;
    int d, m, sec = 0;
    if (n < 4 || sscanf(s, "%c%d*%d:%d", &sign, &d, &m, &sec) < 3) return false;
    if ((sign != '+' && sign != '-') || d < 0 || d > 90 || m < 0 || m >= 60 || sec < 0 || sec >= 60) return false;
    *out = d + m / 60.0f + sec / 3600.0f;
    if (sign == '-') *out = -*out;
    return *out >= -90.0f && *out <= 90.0f;
}

os_error_t os_init(void) {
    os_error_t result = OS_ERR_NONE;
    uint8_t i;
    reset_runtime();
    if (os_hal_nvm_init() == OS_ERR_NONE) restore_state(); else result = OS_ERR_NVM_FAULT;
    for (i = 0; i < CHANNELS; ++i) if (os_hal_comm_init(i) != OS_ERR_NONE) result = OS_ERR_NOT_SUPPORTED;
    for (i = 0; i < AXES; ++i) { if (os_hal_motor_init(i) != OS_ERR_NONE) result = OS_ERR_MOTOR_DRIVER_FAULT; stop_axis(i); }
    if (os_hal_gps_init() != OS_ERR_NONE) g.gps_locked = false;
    if (os_hal_rtc_init() != OS_ERR_NONE) g.rtc_valid = false;
    if (os_hal_limit_init() != OS_ERR_NONE) result = OS_ERR_MOTOR_DRIVER_FAULT;
    if (os_hal_timer_motor_init() != OS_ERR_NONE) result = OS_ERR_MOTOR_DRIVER_FAULT;
    site_update();
    g.initialized = result != OS_ERR_MOTOR_DRIVER_FAULT;
    g.state = result == OS_ERR_MOTOR_DRIVER_FAULT ? OS_STATE_FAULT : OS_STATE_IDLE_TRACKING;
    return result;
}

void os_loop_iteration(void) {
    uint8_t channel, axis;
    site_update();
    for (axis = 0; axis < AXES; ++axis) {
        if (os_hal_limit_is_triggered(axis) && (g.goto_active || g.park_active || g.manual_active || g.tracking)) {
            stop_axis(axis);
            if (g.goto_active || g.park_active || g.manual_active) { g.goto_active = false; g.park_active = false; g.manual_active = false; g.state = OS_STATE_FAULT; }
        }
    }
    if (g.goto_active || g.park_active) complete_motion();
    if (g.guide.active) {
        if (g.guide_remaining_ms <= LOOP_MS) { g.guide_remaining_ms = 0; g.guide.active = false; }
        else g.guide_remaining_ms -= LOOP_MS;
    }
    if (g.state == OS_STATE_IDLE_TRACKING && g.tracking && !os_hal_limit_is_triggered(0)) {
        (void)os_hal_motor_set_direction(0, true); (void)os_hal_motor_enable(0, true); (void)os_hal_motor_set_frequency(0, tracking_frequency());
    }
    for (channel = 0; channel < CHANNELS; ++channel) {
        while (os_hal_comm_available(channel) > 0) {
            command_buffer_t *b = &g.commands[channel];
            char ch = os_hal_comm_read(channel);
            if (ch == OS_LX200_CMD_PREFIX) { b->receiving = true; b->length = 0; }
            if (!b->receiving) continue;
            if (b->length >= OS_MAX_COMMAND_LENGTH) { b->receiving = false; b->length = 0; continue; }
            b->data[b->length++] = ch;
            if (ch == OS_LX200_CMD_SUFFIX) {
                char reply[OS_MAX_REPLY_LENGTH]; size_t length = 0;
                (void)os_command_parse(b->data, b->length, channel, reply, sizeof reply, &length);
                if (length != 0) (void)os_hal_comm_write(channel, reply, length);
                b->receiving = false; b->length = 0;
            }
        }
    }
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    int32_t ra, dec, p0, p1;
    if (!valid_equatorial(target)) return OS_ERR_INVALID_ARGUMENT;
    if (g.state == OS_STATE_PARKED || g.state == OS_STATE_FAULT || g.state == OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE;
    p0 = os_hal_motor_get_position(0); p1 = os_hal_motor_get_position(1); coordinates_to_steps(target, &ra, &dec);
    if ((ra != p0 && check_limit(0, ra > p0) != OS_ERR_NONE) || (dec != p1 && check_limit(1, dec > p1) != OS_ERR_NONE)) return OS_ERR_LIMIT_TRIGGERED;
    if (ra == p0 && dec == p1) { g.state = OS_STATE_IDLE_TRACKING; return OS_ERR_NONE; }
    begin_motion(ra, dec, false); return OS_ERR_NONE;
}
os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    os_equatorial_coord_t c;
    if (!valid_horizontal(target)) return OS_ERR_INVALID_ARGUMENT;
    c.ra_hours = target.azimuth_degrees / 15.0f; c.dec_degrees = target.altitude_degrees;
    return os_goto_equatorial(c);
}
os_error_t os_goto_abort(void) { if (!g.goto_active && !g.park_active) return OS_ERR_INVALID_STATE; stop_all(); g.goto_active = false; g.park_active = false; g.state = OS_STATE_IDLE_TRACKING; return OS_ERR_NONE; }
os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) { if (rate < OS_TRACK_RATE_SIDEREAL || rate > OS_TRACK_RATE_CUSTOM || !finite_float(custom_factor) || (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f)) return OS_ERR_INVALID_ARGUMENT; g.track_rate = rate; g.custom_factor = custom_factor; return OS_ERR_NONE; }
os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) { if (!rate || !custom_factor) return OS_ERR_INVALID_ARGUMENT; *rate = g.track_rate; *custom_factor = g.custom_factor; return OS_ERR_NONE; }
os_error_t os_tracking_enable(void) { if (g.state == OS_STATE_PARKED || g.state == OS_STATE_FAULT) return OS_ERR_INVALID_STATE; g.tracking = true; return OS_ERR_NONE; }
os_error_t os_tracking_disable(void) { g.tracking = false; if (!g.goto_active && !g.park_active && !g.manual_active) stop_axis(0); return OS_ERR_NONE; }
os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) { if (!valid_direction(direction) || duration_ms == 0) return OS_ERR_INVALID_ARGUMENT; if (g.state == OS_STATE_PARKED || g.state == OS_STATE_FAULT) return OS_ERR_INVALID_STATE; g.guide.active = true; g.guide_remaining_ms = duration_ms; g.guide.duration_ms = duration_ms; g.guide.rate_fraction = g.guide_rate; g.guide.direction_east = direction == OS_DIRECTION_EAST; g.guide.direction_north = direction == OS_DIRECTION_NORTH; g.guide.dec_priority = direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH; return OS_ERR_NONE; }
os_error_t os_guide_set_rate(float rate_fraction) { if (!finite_float(rate_fraction) || rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) return OS_ERR_INVALID_ARGUMENT; g.guide_rate = rate_fraction; return OS_ERR_NONE; }
os_error_t os_guide_get_state(os_guide_pulse_t *pulse) { if (!pulse) return OS_ERR_INVALID_ARGUMENT; *pulse = g.guide; return OS_ERR_NONE; }
os_error_t os_align_begin(os_align_mode_t mode) { if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) return OS_ERR_INVALID_ARGUMENT; if (g.state == OS_STATE_PARKED || g.state == OS_STATE_FAULT) return OS_ERR_INVALID_STATE; g.align_mode = mode; g.star_count = 0; g.residual_valid = false; g.state = OS_STATE_ALIGNMENT; return OS_ERR_NONE; }
os_error_t os_align_accept_star(os_equatorial_coord_t star_coord, os_motor_position_t motor_pos) { if (!valid_equatorial(star_coord)) return OS_ERR_INVALID_ARGUMENT; if (g.state != OS_STATE_ALIGNMENT || g.star_count >= OS_CALIBRATION_MAX_STARS) return OS_ERR_INVALID_STATE; g.stars[g.star_count] = star_coord; g.star_positions[g.star_count] = motor_pos; ++g.star_count; return OS_ERR_NONE; }
os_error_t os_align_compute(void) {
    uint8_t i, n = g.align_mode == OS_ALIGN_NSTAR ? 3u : (uint8_t)g.align_mode;
    double A[3][3], q[3][3], r[3][3], bz[3], by[3], z[3], y[3], norm, dot, predicted_ra, predicted_dec, sumsq = 0.0;
    if (g.state != OS_STATE_ALIGNMENT || g.star_count < n) return OS_ERR_INVALID_STATE;
    calibration_default();
    if (n == 1) {
        g.calibration.offset_ra_arcsec = (float)(g.star_positions[0].ra_steps * ARCSEC_PER_STEP - g.stars[0].ra_hours * 54000.0);
        g.calibration.offset_dec_arcsec = (float)(g.star_positions[0].dec_steps * ARCSEC_PER_STEP - g.stars[0].dec_degrees * 3600.0);
    } else if (n == 2) {
        double x0 = g.stars[0].ra_hours * 54000.0, x1 = g.stars[1].ra_hours * 54000.0;
        double y0 = g.stars[0].dec_degrees * 3600.0, y1 = g.stars[1].dec_degrees * 3600.0;
        if (fabs(x1-x0) < 1e-9 || fabs(y1-y0) < 1e-9) return OS_ERR_CALIBRATION_FAILED;
        g.calibration.matrix_ra_to_ra = (float)((g.star_positions[1].ra_steps-g.star_positions[0].ra_steps)*ARCSEC_PER_STEP/(x1-x0));
        g.calibration.matrix_dec_to_dec = (float)((g.star_positions[1].dec_steps-g.star_positions[0].dec_steps)*ARCSEC_PER_STEP/(y1-y0));
        g.calibration.offset_ra_arcsec = (float)(g.star_positions[0].ra_steps*ARCSEC_PER_STEP-g.calibration.matrix_ra_to_ra*x0);
        g.calibration.offset_dec_arcsec = (float)(g.star_positions[0].dec_steps*ARCSEC_PER_STEP-g.calibration.matrix_dec_to_dec*y0);
    } else {
        memset(A, 0, sizeof A);
        for (i = 0; i < n; ++i) { double x=g.stars[i].ra_hours*54000.0, yv=g.stars[i].dec_degrees*3600.0; A[0][0]=x; A[0][1]=yv; A[0][2]=1.0; }
        for (i = 0; i < 3; ++i) { norm=0.0; for (uint8_t j=0;j<n;++j) { double x=g.stars[j].ra_hours*54000.0, yv=g.stars[j].dec_degrees*3600.0; double v=i==0?x:(i==1?yv:1.0); norm+=v*v; } if (norm < 1e-18) return OS_ERR_CALIBRATION_FAILED; }
        for (uint8_t col=0; col<3; ++col) { for (uint8_t row=0; row<n; ++row) { double x=g.stars[row].ra_hours*54000.0, yv=g.stars[row].dec_degrees*3600.0; q[row][col]=col==0?x:(col==1?yv:1.0); } for (uint8_t k=0;k<col;++k) { dot=0.0; for (uint8_t row=0;row<n;++row) dot+=q[row][k]*q[row][col]; r[k][col]=dot; for (uint8_t row=0;row<n;++row) q[row][col]-=dot*q[row][k]; } norm=0.0; for (uint8_t row=0;row<n;++row) norm+=q[row][col]*q[row][col]; norm=sqrt(norm); if (norm<1e-12) return OS_ERR_CALIBRATION_FAILED; r[col][col]=norm; for (uint8_t row=0;row<n;++row) q[row][col]/=norm; }
        for (uint8_t col=0;col<3;++col) { bz[col]=by[col]=0.0; for (uint8_t row=0;row<n;++row) { bz[col]+=q[row][col]*g.star_positions[row].ra_steps*ARCSEC_PER_STEP; by[col]+=q[row][col]*g.star_positions[row].dec_steps*ARCSEC_PER_STEP; } }
        for (int k=2;k>=0;--k) { z[k]=bz[k]; y[k]=by[k]; for (uint8_t j=(uint8_t)k+1;j<3;++j) { z[k]-=r[k][j]*z[j]; y[k]-=r[k][j]*y[j]; } z[k]/=r[k][k]; y[k]/=r[k][k]; }
        g.calibration.matrix_ra_to_ra=(float)z[0]; g.calibration.matrix_dec_to_ra=(float)z[1]; g.calibration.offset_ra_arcsec=(float)z[2]; g.calibration.matrix_ra_to_dec=(float)y[0]; g.calibration.matrix_dec_to_dec=(float)y[1]; g.calibration.offset_dec_arcsec=(float)y[2];
    }
    g.calibration.valid = true;
    for (i=0;i<g.star_count;++i) { double x=g.stars[i].ra_hours*54000.0,yv=g.stars[i].dec_degrees*3600.0; predicted_ra=g.calibration.matrix_ra_to_ra*x+g.calibration.matrix_dec_to_ra*yv+g.calibration.offset_ra_arcsec; predicted_dec=g.calibration.matrix_ra_to_dec*x+g.calibration.matrix_dec_to_dec*yv+g.calibration.offset_dec_arcsec; sumsq+=(predicted_ra-g.star_positions[i].ra_steps*ARCSEC_PER_STEP)*(predicted_ra-g.star_positions[i].ra_steps*ARCSEC_PER_STEP)+(predicted_dec-g.star_positions[i].dec_steps*ARCSEC_PER_STEP)*(predicted_dec-g.star_positions[i].dec_steps*ARCSEC_PER_STEP); }
    g.residual_arcsec=(float)sqrt(sumsq/g.star_count); if (g.star_count >= 4 && g.residual_arcsec > 600.0f) { g.calibration.valid=false; return OS_ERR_CALIBRATION_FAILED; } g.residual_valid=true; persist_state(); g.state=OS_STATE_IDLE_TRACKING; return OS_ERR_NONE;
}
os_error_t os_align_get_residual(float *residual_arcsec) { if (!residual_arcsec) return OS_ERR_INVALID_ARGUMENT; if (!g.residual_valid) return OS_ERR_INVALID_STATE; *residual_arcsec=g.residual_arcsec; return OS_ERR_NONE; }
os_error_t os_align_abort(void) { if (g.state != OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE; g.star_count=0; g.residual_valid=false; g.state=OS_STATE_IDLE_TRACKING; return OS_ERR_NONE; }
os_error_t os_park(void) { int32_t ra,dec; if (g.state==OS_STATE_PARKED || g.state==OS_STATE_FAULT || g.state==OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE; if (os_hal_limit_is_triggered(0)||os_hal_limit_is_triggered(1)) return OS_ERR_LIMIT_TRIGGERED; coordinates_to_steps(g.park_position,&ra,&dec); begin_motion(ra,dec,true); return OS_ERR_NONE; }
os_error_t os_unpark(void) { uint8_t i; if (g.state != OS_STATE_PARKED) return OS_ERR_INVALID_STATE; for(i=0;i<AXES;++i) (void)os_hal_motor_enable(i,true); for(i=0;i<CHANNELS;++i) (void)os_hal_comm_init(i); site_update(); g.tracking=true; g.state=OS_STATE_IDLE_TRACKING; return OS_ERR_NONE; }
os_error_t os_park_set_position(os_equatorial_coord_t park_pos) { if (!valid_equatorial(park_pos)) return OS_ERR_INVALID_ARGUMENT; g.park_position=park_pos; persist_state(); return OS_ERR_NONE; }
os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) { uint8_t axis; bool forward; uint32_t hz; if (!valid_direction(direction)||speed<OS_SPEED_SLOW||speed>OS_SPEED_CUSTOM) return OS_ERR_INVALID_ARGUMENT; if (g.state==OS_STATE_PARKED||g.state==OS_STATE_FAULT||g.state==OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE; axis=(direction==OS_DIRECTION_NORTH||direction==OS_DIRECTION_SOUTH)?1:0; forward=direction==OS_DIRECTION_NORTH||direction==OS_DIRECTION_EAST; if (check_limit(axis,forward)!=OS_ERR_NONE) return OS_ERR_LIMIT_TRIGGERED; hz=speed==OS_SPEED_SLOW?50u:speed==OS_SPEED_MEDIUM?500u:speed==OS_SPEED_FAST?3000u:(uint32_t)(g.manual_speed/ARCSEC_PER_STEP+0.5); (void)os_hal_motor_set_direction(axis,forward); (void)os_hal_motor_enable(axis,true); (void)os_hal_motor_set_frequency(axis,hz); g.manual_active=true; g.tracking=false; g.state=OS_STATE_MANUAL_MOTION; return OS_ERR_NONE; }
os_error_t os_move_stop(void) { if (!g.manual_active) return OS_ERR_INVALID_STATE; stop_all(); g.manual_active=false; g.tracking=true; g.state=OS_STATE_IDLE_TRACKING; return OS_ERR_NONE; }
os_error_t os_move_set_custom_speed(float arcsec_per_sec) { if (!finite_float(arcsec_per_sec)||arcsec_per_sec<=0.0f) return OS_ERR_INVALID_ARGUMENT; g.manual_speed=arcsec_per_sec; return OS_ERR_NONE; }
os_error_t os_query_state(os_state_t *state) { if (!state) return OS_ERR_INVALID_ARGUMENT; *state=g.state; return OS_ERR_NONE; }
os_error_t os_query_coordinates(os_equatorial_coord_t *coord) { os_motor_position_t p; if (!coord) return OS_ERR_INVALID_ARGUMENT; p.ra_steps=os_hal_motor_get_position(0); p.dec_steps=os_hal_motor_get_position(1); return steps_to_coordinates(p,coord)?OS_ERR_NONE:OS_ERR_CALIBRATION_FAILED; }
os_error_t os_query_site(os_site_info_t *site) { if (!site) return OS_ERR_INVALID_ARGUMENT; *site=g.site; return OS_ERR_NONE; }
os_error_t os_query_motor_position(os_motor_position_t *pos) { if (!pos) return OS_ERR_INVALID_ARGUMENT; pos->ra_steps=os_hal_motor_get_position(0); pos->dec_steps=os_hal_motor_get_position(1); return OS_ERR_NONE; }
os_error_t os_query_firmware_version(uint8_t *major,uint8_t *minor,uint8_t *patch) { if (!major||!minor||!patch) return OS_ERR_INVALID_ARGUMENT; *major=OS_FIRMWARE_VERSION_MAJOR; *minor=OS_FIRMWARE_VERSION_MINOR; *patch=OS_FIRMWARE_VERSION_PATCH; return OS_ERR_NONE; }
os_error_t os_query_is_moving(bool *moving) { if (!moving) return OS_ERR_INVALID_ARGUMENT; *moving=g.goto_active||g.park_active||g.manual_active||(g.tracking&&g.state==OS_STATE_IDLE_TRACKING); return OS_ERR_NONE; }
os_error_t os_query_gps_locked(bool *locked) { if (!locked) return OS_ERR_INVALID_ARGUMENT; *locked=g.gps_locked; return OS_ERR_NONE; }
os_error_t os_pec_enable(bool enable) { if (enable&&!g.pec.valid) return OS_ERR_INVALID_STATE; g.pec_enabled=enable; persist_state(); return OS_ERR_NONE; }
os_error_t os_pec_load_table(const os_pec_table_t *table) { if (!table) return OS_ERR_INVALID_ARGUMENT; g.pec=*table; g.pec_enabled=false; persist_state(); return OS_ERR_NONE; }
os_error_t os_pec_get_table(os_pec_table_t *table) { if (!table) return OS_ERR_INVALID_ARGUMENT; *table=g.pec; return OS_ERR_NONE; }
os_error_t os_pec_record_phase(float worm_phase_deg,int16_t error_arcsec) { uint16_t index; if (!finite_float(worm_phase_deg)||worm_phase_deg<0.0f||worm_phase_deg>360.0f) return OS_ERR_INVALID_ARGUMENT; index=worm_phase_deg==360.0f?0u:(uint16_t)worm_phase_deg; g.pec.corrections[index]=error_arcsec; g.pec.valid=true; persist_state(); return OS_ERR_NONE; }
os_error_t os_calibration_get(os_calibration_t *calib) { if (!calib) return OS_ERR_INVALID_ARGUMENT; *calib=g.calibration; return OS_ERR_NONE; }
os_error_t os_calibration_clear(void) { calibration_default(); g.residual_valid=false; persist_state(); return OS_ERR_NONE; }

os_error_t os_command_parse(const char *command,size_t length,uint8_t source_channel,char *reply_buffer,size_t reply_buffer_size,size_t *reply_length) {
    os_error_t e=OS_ERR_COMMAND_FORMAT;
    if (!command||!reply_buffer||!reply_length) return OS_ERR_INVALID_ARGUMENT;
    *reply_length=0;
    if (source_channel>OS_CHANNEL_ETHERNET||reply_buffer_size<2||length<3||length>OS_MAX_COMMAND_LENGTH) return OS_ERR_INVALID_ARGUMENT;
    if (command[0]!=OS_LX200_CMD_PREFIX||command[length-1]!=OS_LX200_CMD_SUFFIX) return OS_ERR_COMMAND_FORMAT;
    if (length==5&&memcmp(command,":GVP#",5)==0) { if (!reply_format(reply_buffer,reply_buffer_size,reply_length,"OnStep %u.%u.%u#",1u,0u,0u)) return OS_ERR_INVALID_ARGUMENT; return OS_ERR_NONE; }
    if (length==4&&memcmp(command,":GR#",4)==0) { os_equatorial_coord_t c; e=os_query_coordinates(&c); if(e==OS_ERR_NONE&&!reply_format(reply_buffer,reply_buffer_size,reply_length,"%06.3f#",c.ra_hours)) e=OS_ERR_INVALID_ARGUMENT; }
    else if (length==4&&memcmp(command,":GD#",4)==0) { os_equatorial_coord_t c; e=os_query_coordinates(&c); if(e==OS_ERR_NONE&&!reply_format(reply_buffer,reply_buffer_size,reply_length,"%+07.3f#",c.dec_degrees)) e=OS_ERR_INVALID_ARGUMENT; }
    else if (length>=5&&command[1]=='S'&&command[2]=='r') { float v; if(!parse_ra(command+3,length-4,&v)) e=OS_ERR_INVALID_ARGUMENT; else { g.pending_target.ra_hours=v; g.pending_ra=true; e=OS_ERR_NONE; } }
    else if (length>=5&&command[1]=='S'&&command[2]=='d') { float v; if(!parse_dec(command+3,length-4,&v)) e=OS_ERR_INVALID_ARGUMENT; else { g.pending_target.dec_degrees=v; g.pending_dec=true; e=OS_ERR_NONE; } }
    else if (length==4&&memcmp(command,":MS#",4)==0) e=(!g.pending_ra||!g.pending_dec)?OS_ERR_INVALID_STATE:os_goto_equatorial(g.pending_target);
    else if (length==3&&memcmp(command,":Q#",3)==0) e=g.goto_active||g.park_active?os_goto_abort():g.manual_active?os_move_stop():OS_ERR_NONE;
    else if (length==4&&memcmp(command,":hP#",4)==0) e=os_park();
    else if (length==4&&memcmp(command,":hO#",4)==0) e=os_unpark();
    else if (length==4&&memcmp(command,":Me#",4)==0) e=os_move_start(OS_DIRECTION_EAST,OS_SPEED_MEDIUM);
    else if (length==4&&memcmp(command,":Mw#",4)==0) e=os_move_start(OS_DIRECTION_WEST,OS_SPEED_MEDIUM);
    else if (length==4&&memcmp(command,":Mn#",4)==0) e=os_move_start(OS_DIRECTION_NORTH,OS_SPEED_MEDIUM);
    else if (length==4&&memcmp(command,":Ms#",4)==0) e=os_move_start(OS_DIRECTION_SOUTH,OS_SPEED_MEDIUM);
    else e=OS_ERR_COMMAND_FORMAT;
    if (*reply_length==0) { reply_buffer[0]=e==OS_ERR_NONE?'1':'0'; reply_buffer[1]='#'; *reply_length=2; }
    return e;
}

OS_WEAK os_error_t os_hal_motor_init(uint8_t axis) { if(axis>=AXES)return OS_ERR_INVALID_ARGUMENT; hal_initialized[axis]=true; hal_enabled[axis]=false; hal_frequency[axis]=0; return OS_ERR_NONE; }
OS_WEAK os_error_t os_hal_motor_set_frequency(uint8_t axis,uint32_t frequency_hz) { if(axis>=AXES)return OS_ERR_INVALID_ARGUMENT; hal_frequency[axis]=frequency_hz; return OS_ERR_NONE; }
OS_WEAK os_error_t os_hal_motor_set_direction(uint8_t axis,bool forward) { if(axis>=AXES)return OS_ERR_INVALID_ARGUMENT; hal_direction[axis]=forward; return OS_ERR_NONE; }
OS_WEAK os_error_t os_hal_motor_enable(uint8_t axis,bool enable) { if(axis>=AXES)return OS_ERR_INVALID_ARGUMENT; hal_enabled[axis]=enable; if(!enable)hal_frequency[axis]=0; return OS_ERR_NONE; }
OS_WEAK int32_t os_hal_motor_get_position(uint8_t axis) { return axis<AXES?hal_position[axis]:0; }
OS_WEAK os_error_t os_hal_gps_init(void) { return OS_ERR_NONE; }
OS_WEAK os_error_t os_hal_gps_poll(os_site_info_t *site) { if(!site)return OS_ERR_INVALID_ARGUMENT; site->valid=false; return OS_ERR_GPS_NO_SIGNAL; }
OS_WEAK os_error_t os_hal_rtc_init(void) { return OS_ERR_NONE; }
OS_WEAK os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) { if(!utc_epoch_seconds)return OS_ERR_INVALID_ARGUMENT; *utc_epoch_seconds=hal_rtc; return OS_ERR_NONE; }
OS_WEAK os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) { hal_rtc=utc_epoch_seconds; return OS_ERR_NONE; }
OS_WEAK os_error_t os_hal_limit_init(void) { return OS_ERR_NONE; }
OS_WEAK bool os_hal_limit_is_triggered(uint8_t axis) { return axis>=AXES?true:hal_limits[axis]; }
OS_WEAK os_error_t os_hal_nvm_init(void) { if(!hal_nvm_ready){memset(hal_nvm,0,sizeof hal_nvm);hal_nvm_ready=true;} return OS_ERR_NONE; }
OS_WEAK os_error_t os_hal_nvm_read(uint16_t offset,uint8_t *data,uint16_t length) { if(!data)return OS_ERR_INVALID_ARGUMENT; if((uint32_t)offset+length>NVM_BYTES)return OS_ERR_NVM_FAULT; memcpy(data,hal_nvm+offset,length); return OS_ERR_NONE; }
OS_WEAK os_error_t os_hal_nvm_write(uint16_t offset,const uint8_t *data,uint16_t length) { if(!data)return OS_ERR_INVALID_ARGUMENT; if((uint32_t)offset+length>NVM_BYTES)return OS_ERR_NVM_FAULT; memcpy(hal_nvm+offset,data,length); return OS_ERR_NONE; }
OS_WEAK os_error_t os_hal_comm_init(uint8_t channel) { if(channel>=CHANNELS)return OS_ERR_INVALID_ARGUMENT; hal_rx_head[channel]=hal_rx_tail[channel]=hal_tx_length[channel]=0; return OS_ERR_NONE; }
OS_WEAK int16_t os_hal_comm_available(uint8_t channel) { if(channel>=CHANNELS)return 0; return (int16_t)(hal_rx_tail[channel]-hal_rx_head[channel]); }
OS_WEAK char os_hal_comm_read(uint8_t channel) { if(channel>=CHANNELS||hal_rx_head[channel]>=hal_rx_tail[channel])return 0; return (char)hal_rx[channel][hal_rx_head[channel]++%sizeof hal_rx[channel]]; }
OS_WEAK os_error_t os_hal_comm_write(uint8_t channel,const char *data,size_t length) { if(channel>=CHANNELS||!data)return OS_ERR_INVALID_ARGUMENT; if(length>sizeof hal_tx[channel]-hal_tx_length[channel])return OS_ERR_TIMEOUT; memcpy(hal_tx[channel]+hal_tx_length[channel],data,length);hal_tx_length[channel]+=length;return OS_ERR_NONE; }
OS_WEAK os_error_t os_hal_buzzer_beep(uint16_t duration_ms,uint8_t count) { (void)duration_ms;(void)count;return OS_ERR_NONE; }
OS_WEAK os_error_t os_hal_timer_motor_init(void) { return OS_ERR_NONE; }
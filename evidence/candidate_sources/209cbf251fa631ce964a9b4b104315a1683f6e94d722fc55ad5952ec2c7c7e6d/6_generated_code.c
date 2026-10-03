#include "6_generated_code.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#define OS_PI 3.14159265358979323846
#define OS_LOOP_TICK_MS 10u
#define OS_GOTO_EPSILON_STEPS 4
#define OS_GOTO_ARRIVE_FREQ_HZ 50u
#define OS_GOTO_ACCEL_HZ_PER_SEC 40000.0
#define OS_MAX_MOTOR_FREQ_HZ 200000u
#define OS_MAX_RESIDUAL_ARCSEC 300.0f
#define OS_PARK_RECORD_OFFSET       (OS_NVM_CALIBRATION_SIZE_BYTES)
#define OS_CONFIG_RECORD_OFFSET     (OS_NVM_CALIBRATION_SIZE_BYTES + 32u)
#define OS_SITE_RECORD_OFFSET       (OS_NVM_CALIBRATION_SIZE_BYTES + 96u)
#define OS_PEC_RECORD_OFFSET        (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES)
#define OS_PEC_WORM_CYCLE_ARCSEC 360.0f

static os_state_t st = OS_STATE_INITIALIZING;
static os_track_rate_t tr = OS_TRACK_RATE_SIDEREAL;
static float cf = 1.0f;
static bool te = false;
static float guide_rate = 0.5f;
static os_guide_pulse_t gp;
static uint32_t gr_remaining = 0;
static uint8_t g_axis = 0;
static bool g_forward = true;
static bool gps_locked = false;
static os_site_info_t site;
static os_calibration_t cal;
static os_pec_table_t pec;
static bool pec_en = false;
static bool residual_ok = false;
static float residual_arcsec = 0.0f;
static bool align_active = false;
static os_align_mode_t align_mode = OS_ALIGN_1STAR;
static uint8_t align_n = 0;
static os_equatorial_coord_t align_star[OS_CALIBRATION_MAX_STARS];
static os_motor_position_t align_motor[OS_CALIBRATION_MAX_STARS];
static os_equatorial_coord_t park_pos = {0.0f, 90.0f};
static bool park_set = false;
static bool nvm_ok = false;
static struct { bool active; bool parking; int32_t t[2]; bool done[2]; uint32_t freq[2]; } motion;
static bool man_active = false;
static uint8_t man_axis = 0;
static bool man_forward = true;
static os_speed_level_t man_speed = OS_SPEED_MEDIUM;
static float custom_speed_arcsec = 15.0f;
static char cmdbuf[4][OS_MAX_COMMAND_LENGTH];
static uint8_t cmdlen[4];
static bool cmdin[4];
static os_equatorial_coord_t goto_target = {0.0f, 0.0f};

static float cfg_steps_per_arcsec = 1.0f;
static os_mount_type_t cfg_mount_type = OS_MOUNT_EQUATORIAL;
static float current_worm_phase_deg = 0.0f;
static bool last_cmd_valid[2] = {false, false};
static bool last_cmd_fw[2] = {false, false};
static bool limit_safe_valid[2] = {false, false};
static bool limit_safe_fw[2] = {false, false};

static int32_t abs32(int32_t x) { return x < 0 ? -x : x; }
static bool valid_ra(float v) { return isfinite(v) && v >= OS_RA_MIN_HOURS && v <= OS_RA_MAX_HOURS; }
static bool valid_dec(float v) { return isfinite(v) && v >= OS_DEC_MIN_DEG && v <= OS_DEC_MAX_DEG; }
static bool valid_dir(os_direction_t d) { return d >= OS_DIRECTION_NORTH && d <= OS_DIRECTION_WEST; }
static bool valid_speed(os_speed_level_t s) { return s >= OS_SPEED_SLOW && s <= OS_SPEED_CUSTOM; }

static bool map_direction(os_direction_t d, uint8_t *axis, bool *fw) {
    switch (d) {
        case OS_DIRECTION_NORTH: *axis = 1; *fw = true; return true;
        case OS_DIRECTION_SOUTH: *axis = 1; *fw = false; return true;
        case OS_DIRECTION_EAST:  *axis = 0; *fw = true; return true;
        case OS_DIRECTION_WEST:  *axis = 0; *fw = false; return true;
        default: return false;
    }
}

static void set_motor_direction(uint8_t axis, bool forward) {
    if (axis > 1) return;
    last_cmd_valid[axis] = true;
    last_cmd_fw[axis] = forward;
    os_hal_motor_set_direction(axis, forward);
}

static void stop_axis(uint8_t axis) {
    if (axis <= 1) os_hal_motor_set_frequency(axis, 0);
}

static void enter_fault(void) {
    st = OS_STATE_FAULT;
    motion.active = false;
    motion.parking = false;
    man_active = false;
    gp.active = false;
    gr_remaining = 0;
    stop_axis(0);
    stop_axis(1);
    os_hal_motor_enable(0, false);
    os_hal_motor_enable(1, false);
}

static void set_reply(char *b, size_t sz, size_t *out, const char *fmt, ...) {
    if (!b || !sz || !out) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sz, fmt, ap);
    va_end(ap);
    if (n < 0) { b[0] = 0; *out = 0; return; }
    *out = (size_t)n < sz ? (size_t)n : sz - 1;
}

static void put_f32(uint8_t *b, size_t *o, float v) { memcpy(b + *o, &v, sizeof(v)); *o += sizeof(v); }
static float get_f32(const uint8_t *b, size_t *o) { float v; memcpy(&v, b + *o, sizeof(v)); *o += sizeof(v); return v; }
static void put_u32(uint8_t *b, size_t *o, uint32_t v) { memcpy(b + *o, &v, sizeof(v)); *o += sizeof(v); }
static uint32_t get_u32(const uint8_t *b, size_t *o) { uint32_t v; memcpy(&v, b + *o, sizeof(v)); *o += sizeof(v); return v; }

static os_error_t read_calibration_record(os_calibration_t *c);
static os_error_t write_calibration_record(const os_calibration_t *c);
static os_error_t read_park_record(os_equatorial_coord_t *p, bool *set);
static os_error_t write_park_record(const os_equatorial_coord_t *p, bool set);
static os_error_t read_config_record(float *steps, uint8_t *mount);
static os_error_t read_site_record(os_site_info_t *s);
static os_error_t write_site_record(const os_site_info_t *s);
static os_error_t read_pec_table(os_pec_table_t *t);
static os_error_t write_pec_table(const os_pec_table_t *t);
static void load_persistent_state(void);
static void load_pec_table(void);
static bool horizontal_to_equatorial(const os_horizontal_coord_t *h, const os_site_info_t *st_site, os_equatorial_coord_t *eq);
static os_error_t get_current_eq(os_equatorial_coord_t *e);
static uint32_t clamp_freq(double f);
static uint32_t goto_max_freq(void);
static int32_t goto_decel_steps(uint32_t freq);
static uint32_t speed_hz(os_speed_level_t s);
static uint32_t tracking_hz_for_axis(uint8_t axis, bool include_pec);
static uint32_t normal_output_freq_for_axis(uint8_t axis);
static float base_tracking_rate_arcsec_for_axis(uint8_t axis);
static void update_worm_phase(void);
static void update_guide_pulse(void);
static void update_motion_and_limits(void);
static void poll_gps_and_time(void);
static void poll_commands(void);
static bool qr_solve(int m, int n, double A[OS_CALIBRATION_MAX_STARS][3], double *B, double *X);
static os_error_t motion_start(int32_t ra_target, int32_t dec_target, bool parking);
static bool parse_u32(const char *s, uint32_t *v);
static bool read_unsigned_max(const char **pp, int max, int *out);
static bool read_float_seconds_max(const char **pp, float max, float *out);
static bool parse_ra_string(const char *s, float *out);
static bool parse_dec_string(const char *s, float *out);
static void format_ra_reply(char *b, size_t sz, size_t *out);
static void format_dec_reply(char *b, size_t sz, size_t *out);

static os_error_t read_calibration_record(os_calibration_t *c) {
    if (!c) return OS_ERR_INVALID_ARGUMENT;
    uint8_t b[64];
    size_t off = 0;
    os_error_t e = os_hal_nvm_read(0, b, sizeof(b));
    if (e) return e;
    if (!(b[0] == 'O' && b[1] == 'S' && b[2] == 'C' && b[3] == 'L')) return OS_ERR_NVM_FAULT;
    off = 4;
    if (b[off++] != 1) return OS_ERR_NVM_FAULT;
    bool v = b[off++] != 0;
    c->matrix_ra_to_ra = get_f32(b, &off);
    c->matrix_ra_to_dec = get_f32(b, &off);
    c->matrix_dec_to_ra = get_f32(b, &off);
    c->matrix_dec_to_dec = get_f32(b, &off);
    c->offset_ra_arcsec = get_f32(b, &off);
    c->offset_dec_arcsec = get_f32(b, &off);
    size_t data_off = off;
    uint32_t chk = get_u32(b, &off);
    uint32_t sum = 0;
    for (size_t i = 0; i < data_off; ++i) sum += b[i];
    if (sum != chk) return OS_ERR_NVM_FAULT;
    c->valid = v;
    return OS_ERR_NONE;
}

static os_error_t write_calibration_record(const os_calibration_t *c) {
    if (!c) return OS_ERR_INVALID_ARGUMENT;
    uint8_t b[64];
    size_t off = 0;
    b[off++] = 'O'; b[off++] = 'S'; b[off++] = 'C'; b[off++] = 'L';
    b[off++] = 1;
    b[off++] = (uint8_t)(c->valid ? 1 : 0);
    put_f32(b, &off, c->matrix_ra_to_ra);
    put_f32(b, &off, c->matrix_ra_to_dec);
    put_f32(b, &off, c->matrix_dec_to_ra);
    put_f32(b, &off, c->matrix_dec_to_dec);
    put_f32(b, &off, c->offset_ra_arcsec);
    put_f32(b, &off, c->offset_dec_arcsec);
    size_t data_off = off;
    uint32_t sum = 0;
    for (size_t i = 0; i < data_off; ++i) sum += b[i];
    put_u32(b, &off, sum);
    return os_hal_nvm_write(0, b, (uint16_t)off);
}

static os_error_t read_park_record(os_equatorial_coord_t *p, bool *set) {
    if (!p || !set) return OS_ERR_INVALID_ARGUMENT;
    uint8_t b[32];
    size_t off = 0;
    os_error_t e = os_hal_nvm_read(OS_PARK_RECORD_OFFSET, b, sizeof(b));
    if (e) return e;
    if (!(b[0] == 'O' && b[1] == 'S' && b[2] == 'P' && b[3] == 'K')) return OS_ERR_NVM_FAULT;
    off = 4;
    if (b[off++] != 1) return OS_ERR_NVM_FAULT;
    bool s = b[off++] != 0;
    p->ra_hours = get_f32(b, &off);
    p->dec_degrees = get_f32(b, &off);
    size_t data_off = off;
    uint32_t chk = get_u32(b, &off);
    uint32_t sum = 0;
    for (size_t i = 0; i < data_off; ++i) sum += b[i];
    if (sum != chk) return OS_ERR_NVM_FAULT;
    *set = s;
    return OS_ERR_NONE;
}

static os_error_t write_park_record(const os_equatorial_coord_t *p, bool set) {
    if (!p) return OS_ERR_INVALID_ARGUMENT;
    uint8_t b[32];
    size_t off = 0;
    b[off++] = 'O'; b[off++] = 'S'; b[off++] = 'P'; b[off++] = 'K';
    b[off++] = 1;
    b[off++] = (uint8_t)(set ? 1 : 0);
    put_f32(b, &off, p->ra_hours);
    put_f32(b, &off, p->dec_degrees);
    size_t data_off = off;
    uint32_t sum = 0;
    for (size_t i = 0; i < data_off; ++i) sum += b[i];
    put_u32(b, &off, sum);
    return os_hal_nvm_write(OS_PARK_RECORD_OFFSET, b, (uint16_t)off);
}

static os_error_t read_config_record(float *steps, uint8_t *mount) {
    if (!steps || !mount) return OS_ERR_INVALID_ARGUMENT;
    uint8_t b[32];
    size_t off = 0;
    os_error_t e = os_hal_nvm_read(OS_CONFIG_RECORD_OFFSET, b, sizeof(b));
    if (e) return e;
    if (!(b[0] == 'O' && b[1] == 'S' && b[2] == 'C' && b[3] == 'F')) return OS_ERR_NVM_FAULT;
    off = 4;
    if (b[off++] != 1) return OS_ERR_NVM_FAULT;
    float s = get_f32(b, &off);
    uint8_t m = b[off++];
    size_t data_off = off;
    uint32_t chk = get_u32(b, &off);
    uint32_t sum = 0;
    for (size_t i = 0; i < data_off; ++i) sum += b[i];
    if (sum != chk) return OS_ERR_NVM_FAULT;
    if (!(s > 0.0f) || !isfinite(s)) return OS_ERR_NVM_FAULT;
    if (m > (uint8_t)OS_MOUNT_ALTAZ) return OS_ERR_NVM_FAULT;
    *steps = s;
    *mount = m;
    return OS_ERR_NONE;
}

static os_error_t read_site_record(os_site_info_t *s) {
    if (!s) return OS_ERR_INVALID_ARGUMENT;
    uint8_t b[32];
    size_t off = 0;
    os_error_t e = os_hal_nvm_read(OS_SITE_RECORD_OFFSET, b, sizeof(b));
    if (e) return e;
    if (!(b[0] == 'O' && b[1] == 'S' && b[2] == 'S' && b[3] == 'T')) return OS_ERR_NVM_FAULT;
    off = 4;
    if (b[off++] != 1) return OS_ERR_NVM_FAULT;
    bool v = b[off++] != 0;
    s->latitude_degrees = get_f32(b, &off);
    s->longitude_degrees = get_f32(b, &off);
    s->elevation_metres = get_f32(b, &off);
    s->utc_epoch_seconds = get_u32(b, &off);
    size_t data_off = off;
    uint32_t chk = get_u32(b, &off);
    uint32_t sum = 0;
    for (size_t i = 0; i < data_off; ++i) sum += b[i];
    if (sum != chk) return OS_ERR_NVM_FAULT;
    if (!(s->latitude_degrees >= -90.0f && s->latitude_degrees <= 90.0f)) return OS_ERR_NVM_FAULT;
    if (!(s->longitude_degrees >= -180.0f && s->longitude_degrees <= 180.0f)) return OS_ERR_NVM_FAULT;
    if (!isfinite(s->elevation_metres)) return OS_ERR_NVM_FAULT;
    s->valid = v;
    return OS_ERR_NONE;
}

static os_error_t write_site_record(const os_site_info_t *s) {
    if (!s) return OS_ERR_INVALID_ARGUMENT;
    uint8_t b[32];
    size_t off = 0;
    b[off++] = 'O'; b[off++] = 'S'; b[off++] = 'S'; b[off++] = 'T';
    b[off++] = 1;
    b[off++] = (uint8_t)(s->valid ? 1 : 0);
    put_f32(b, &off, s->latitude_degrees);
    put_f32(b, &off, s->longitude_degrees);
    put_f32(b, &off, s->elevation_metres);
    put_u32(b, &off, s->utc_epoch_seconds);
    size_t data_off = off;
    uint32_t sum = 0;
    for (size_t i = 0; i < data_off; ++i) sum += b[i];
    put_u32(b, &off, sum);
    return os_hal_nvm_write(OS_SITE_RECORD_OFFSET, b, (uint16_t)off);
}

static os_error_t read_pec_table(os_pec_table_t *t) {
    if (!t) return OS_ERR_INVALID_ARGUMENT;
    uint8_t raw[OS_PEC_TABLE_SIZE * 2 + 16];
    memset(raw, 0, sizeof(raw));
    os_error_t e = os_hal_nvm_read(OS_PEC_RECORD_OFFSET, raw, sizeof(raw));
    if (e) return e;
    if (!(raw[0] == 'O' && raw[1] == 'S' && raw[2] == 'P' && raw[3] == 'C')) return OS_ERR_NVM_FAULT;
    size_t off = 4;
    if (raw[off++] != 1) return OS_ERR_NVM_FAULT;
    bool v = raw[off++] != 0;
    if (off + sizeof(t->corrections) + 4u > sizeof(raw)) return OS_ERR_NVM_FAULT;
    memcpy(t->corrections, raw + off, sizeof(t->corrections));
    off += sizeof(t->corrections);
    size_t data_off = off;
    uint32_t chk = get_u32(raw, &off);
    uint32_t sum = 0;
    for (size_t i = 0; i < data_off; ++i) sum += raw[i];
    if (sum != chk) return OS_ERR_NVM_FAULT;
    t->valid = v;
    return OS_ERR_NONE;
}

static os_error_t write_pec_table(const os_pec_table_t *t) {
    if (!t) return OS_ERR_INVALID_ARGUMENT;
    uint8_t raw[OS_PEC_TABLE_SIZE * 2 + 16];
    memset(raw, 0, sizeof(raw));
    size_t off = 0;
    raw[off++] = 'O'; raw[off++] = 'S'; raw[off++] = 'P'; raw[off++] = 'C';
    raw[off++] = 1;
    raw[off++] = (uint8_t)(t->valid ? 1 : 0);
    memcpy(raw + off, t->corrections, sizeof(t->corrections));
    off += sizeof(t->corrections);
    size_t data_off = off;
    uint32_t sum = 0;
    for (size_t i = 0; i < data_off; ++i) sum += raw[i];
    put_u32(raw, &off, sum);
    return os_hal_nvm_write(OS_PEC_RECORD_OFFSET, raw, (uint16_t)off);
}

static void load_persistent_state(void) {
    os_calibration_t c;
    if (read_calibration_record(&c) == OS_ERR_NONE) {
        cal = c;
    } else {
        cal.valid = false;
        memset(&cal, 0, sizeof(cal));
    }
    os_equatorial_coord_t p;
    bool s = false;
    if (read_park_record(&p, &s) == OS_ERR_NONE) {
        park_pos = p;
        park_set = s;
    }
    float steps = 1.0f;
    uint8_t mount = (uint8_t)OS_MOUNT_EQUATORIAL;
    if (read_config_record(&steps, &mount) == OS_ERR_NONE) {
        cfg_steps_per_arcsec = steps;
        cfg_mount_type = (os_mount_type_t)mount;
    }
}

static void load_pec_table(void) {
    if (!nvm_ok) return;
    os_pec_table_t loaded;
    memset(&loaded, 0, sizeof(loaded));
    if (read_pec_table(&loaded) == OS_ERR_NONE) {
        pec = loaded;
    }
}

static bool horizontal_to_equatorial(const os_horizontal_coord_t *h, const os_site_info_t *st_site, os_equatorial_coord_t *eq) {
    if (!h || !st_site || !eq) return false;
    double alt = (double)h->altitude_degrees * OS_PI / 180.0;
    double az = (double)h->azimuth_degrees * OS_PI / 180.0;
    double lat = (double)st_site->latitude_degrees * OS_PI / 180.0;
    double sin_dec = sin(alt) * sin(lat) + cos(alt) * cos(lat) * cos(az);
    if (sin_dec > 1.0) sin_dec = 1.0;
    else if (sin_dec < -1.0) sin_dec = -1.0;
    double dec = asin(sin_dec);
    double H = atan2(-cos(alt) * sin(az), sin(alt) * cos(lat) - cos(alt) * sin(lat) * cos(az));
    double days = ((double)st_site->utc_epoch_seconds - 946728000.0) / 86400.0;
    double gmst_deg = 280.46061837 + 360.98564736629 * days;
    gmst_deg = fmod(gmst_deg, 360.0);
    if (gmst_deg < 0) gmst_deg += 360.0;
    double lst = (gmst_deg + st_site->longitude_degrees) * OS_PI / 180.0;
    double ra_rad = lst - H;
    ra_rad = fmod(ra_rad, 2.0 * OS_PI);
    if (ra_rad < 0) ra_rad += 2.0 * OS_PI;
    eq->ra_hours = (float)(ra_rad * 12.0 / OS_PI);
    eq->dec_degrees = (float)(dec * 180.0 / OS_PI);
    return valid_ra(eq->ra_hours) && valid_dec(eq->dec_degrees);
}

static os_error_t get_current_eq(os_equatorial_coord_t *e) {
    if (!e) return OS_ERR_INVALID_ARGUMENT;
    if (cfg_mount_type == OS_MOUNT_ALTAZ) {
        if (!site.valid) return OS_ERR_GPS_NO_SIGNAL;
        double az_deg = (double)os_hal_motor_get_position(0) / (3600.0 * cfg_steps_per_arcsec);
        double alt_deg = (double)os_hal_motor_get_position(1) / (3600.0 * cfg_steps_per_arcsec);
        az_deg = fmod(az_deg, 360.0);
        if (az_deg < 0) az_deg += 360.0;
        if (alt_deg < -90.0) alt_deg = -90.0;
        if (alt_deg > 90.0) alt_deg = 90.0;
        os_horizontal_coord_t h;
        h.azimuth_degrees = (float)az_deg;
        h.altitude_degrees = (float)alt_deg;
        if (!horizontal_to_equatorial(&h, &site, e)) return OS_ERR_INVALID_STATE;
        return OS_ERR_NONE;
    }

    int32_t rs = os_hal_motor_get_position(0);
    int32_t ds = os_hal_motor_get_position(1);
    double ra = 0.0, de = 0.0;
    if (cal.valid) {
        double m00 = cal.matrix_ra_to_ra;
        double m01 = cal.matrix_ra_to_dec;
        double m10 = cal.matrix_dec_to_ra;
        double m11 = cal.matrix_dec_to_dec;
        double det = m00 * m11 - m01 * m10;
        if (fabs(det) > 1e-12) {
            double rao = cal.offset_ra_arcsec;
            double deo = cal.offset_dec_arcsec;
            ra = (m11 * ((double)rs - rao) - m01 * ((double)ds - deo)) / det;
            de = (-m10 * ((double)rs - rao) + m00 * ((double)ds - deo)) / det;
        } else {
            ra = (double)rs / cfg_steps_per_arcsec;
            de = (double)ds / cfg_steps_per_arcsec;
        }
    } else {
        ra = (double)rs / cfg_steps_per_arcsec;
        de = (double)ds / cfg_steps_per_arcsec;
    }
    e->ra_hours = (float)(ra / 54000.0);
    e->dec_degrees = (float)(de / 3600.0);
    if (e->ra_hours < 0.0f) e->ra_hours = 0.0f;
    if (e->ra_hours > 24.0f) e->ra_hours = 24.0f;
    if (e->dec_degrees < -90.0f) e->dec_degrees = -90.0f;
    if (e->dec_degrees > 90.0f) e->dec_degrees = 90.0f;
    return OS_ERR_NONE;
}

static void format_ra_reply(char *b, size_t sz, size_t *out) {
    os_equatorial_coord_t e;
    if (get_current_eq(&e) != OS_ERR_NONE) { set_reply(b, sz, out, "0"); return; }
    double ra = e.ra_hours;
    if (ra >= 24.0) ra -= 24.0;
    int h = (int)ra;
    double x = (ra - h) * 60.0;
    int m = (int)x;
    double s = (x - m) * 60.0;
    int si = (int)floor(s + 0.5);
    if (si >= 60) { si -= 60; m++; }
    if (m >= 60) { m -= 60; h = (h + 1) % 24; }
    set_reply(b, sz, out, "%02d:%02d:%02d#", h, m, si);
}

static void format_dec_reply(char *b, size_t sz, size_t *out) {
    os_equatorial_coord_t e;
    if (get_current_eq(&e) != OS_ERR_NONE) { set_reply(b, sz, out, "0"); return; }
    float v = e.dec_degrees;
    char sign = v < 0 ? '-' : '+';
    v = fabsf(v);
    int d = (int)v;
    double x = (v - d) * 60.0;
    int m = (int)x;
    double s = (x - m) * 60.0;
    int si = (int)floor(s + 0.5);
    if (si >= 60) { si -= 60; m++; }
    if (m >= 60) { m -= 60; d++; }
    set_reply(b, sz, out, "%c%02d*%02d:%02d#", sign, d, m, si);
}

static uint32_t clamp_freq(double f) {
    if (!isfinite(f) || f < 0.0) return 0;
    if (f > (double)OS_MAX_MOTOR_FREQ_HZ) return OS_MAX_MOTOR_FREQ_HZ;
    return (uint32_t)(f + 0.5);
}

static uint32_t speed_hz(os_speed_level_t s) {
    switch (s) {
        case OS_SPEED_SLOW: return 50;
        case OS_SPEED_MEDIUM: return 200;
        case OS_SPEED_FAST: return 1000;
        case OS_SPEED_CUSTOM:
            return clamp_freq((double)custom_speed_arcsec * cfg_steps_per_arcsec);
        default:
            return 0;
    }
}

static float base_tracking_rate_arcsec_for_axis(uint8_t axis) {
    if (axis > 1) return 0.0f;
    float base = OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
    if (tr == OS_TRACK_RATE_LUNAR) base *= OS_LUNAR_RATE_FACTOR;
    else if (tr == OS_TRACK_RATE_SOLAR) base *= OS_SOLAR_RATE_FACTOR;
    else if (tr == OS_TRACK_RATE_CUSTOM) base *= cf;

    if (cfg_mount_type == OS_MOUNT_EQUATORIAL) {
        return axis == 0 ? base : 0.0f;
    }

    double lat_deg = site.valid ? (double)site.latitude_degrees : 0.0;
    double lat_rad = lat_deg * OS_PI / 180.0;
    if (axis == 0) return base * (float)cos(lat_rad);
    return base * (float)sin(lat_rad);
}

static int16_t current_pec_correction(void) {
    if (!pec_en || !pec.valid) return 0;
    int idx = (int)floorf(current_worm_phase_deg);
    if (idx < 0) idx = 0;
    if (idx >= OS_PEC_TABLE_SIZE) idx = OS_PEC_TABLE_SIZE - 1;
    return pec.corrections[idx];
}

static uint32_t tracking_hz_for_axis(uint8_t axis, bool include_pec) {
    if (axis > 1) return 0;
    if (!te || st != OS_STATE_IDLE_TRACKING) return 0;
    float rate = base_tracking_rate_arcsec_for_axis(axis);
    if (include_pec && axis == 0 && pec_en && pec.valid) {
        rate += (float)current_pec_correction();
    }
    if (rate < 0.0f) rate = 0.0f;
    return clamp_freq((double)rate * cfg_steps_per_arcsec);
}

static uint32_t normal_output_freq_for_axis(uint8_t axis) {
    return tracking_hz_for_axis(axis, true);
}

static uint32_t goto_max_freq(void) {
    return clamp_freq((double)OS_GOTO_SPEED_MAX_DEG_PER_SEC * 3600.0 * cfg_steps_per_arcsec);
}

static int32_t goto_decel_steps(uint32_t freq) {
    double f = (double)freq;
    double f0 = (double)OS_GOTO_ARRIVE_FREQ_HZ;
    if (f <= f0) return 0;
    double d = (f * f - f0 * f0) / (2.0 * OS_GOTO_ACCEL_HZ_PER_SEC);
    if (d > 2147483647.0) return 2147483647;
    return (int32_t)ceil(d);
}

static void update_worm_phase(void) {
    if (!pec_en || !pec.valid) return;
    if (!te || st != OS_STATE_IDLE_TRACKING) return;
    float rate_arcsec = base_tracking_rate_arcsec_for_axis(0);
    float delta_arcsec = rate_arcsec * (OS_LOOP_TICK_MS / 1000.0f);
    current_worm_phase_deg += (delta_arcsec / OS_PEC_WORM_CYCLE_ARCSEC) * 360.0f;
    current_worm_phase_deg = fmodf(current_worm_phase_deg, 360.0f);
    if (current_worm_phase_deg < 0.0f) current_worm_phase_deg += 360.0f;
}

static void update_guide_pulse(void) {
    if (!gp.active) return;
    if (gr_remaining <= OS_LOOP_TICK_MS) {
        gr_remaining = 0;
        gp.duration_ms = 0;
        gp.active = false;
        uint32_t f = normal_output_freq_for_axis(g_axis);
        os_hal_motor_set_frequency(g_axis, f);
        return;
    }
    gr_remaining -= OS_LOOP_TICK_MS;
    gp.duration_ms = gr_remaining;
    set_motor_direction(g_axis, g_forward);
    os_hal_motor_enable(g_axis, true);

    double base = (double)normal_output_freq_for_axis(g_axis);
    double off = (double)guide_rate * (double)OS_SIDEREAL_RATE_ARCSEC_PER_SEC * (double)cfg_steps_per_arcsec;
    double f = base;
    if (g_forward) f += off;
    else f -= off;
    if (f < 0.0) f = 0.0;
    os_error_t e = os_hal_motor_set_frequency(g_axis, clamp_freq(f));
    if (e == OS_ERR_MOTOR_DRIVER_FAULT) enter_fault();
}

static os_error_t motion_start(int32_t ra_target, int32_t dec_target, bool parking) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) return OS_ERR_LIMIT_TRIGGERED;

    int32_t rpos = os_hal_motor_get_position(0);
    int32_t dpos = os_hal_motor_get_position(1);
    bool rd = abs32(ra_target - rpos) <= OS_GOTO_EPSILON_STEPS;
    bool dd = abs32(dec_target - dpos) <= OS_GOTO_EPSILON_STEPS;

    motion.active = true;
    motion.parking = parking;
    motion.t[0] = ra_target;
    motion.t[1] = dec_target;
    motion.done[0] = rd;
    motion.done[1] = dd;
    motion.freq[0] = OS_GOTO_ARRIVE_FREQ_HZ;
    motion.freq[1] = OS_GOTO_ARRIVE_FREQ_HZ;

    if (rd) {
        stop_axis(0);
    } else {
        os_hal_motor_enable(0, true);
        set_motor_direction(0, ra_target > rpos);
        os_error_t me = os_hal_motor_set_frequency(0, OS_GOTO_ARRIVE_FREQ_HZ);
        if (me == OS_ERR_MOTOR_DRIVER_FAULT) { enter_fault(); return me; }
    }
    if (dd) {
        stop_axis(1);
    } else {
        os_hal_motor_enable(1, true);
        set_motor_direction(1, dec_target > dpos);
        os_error_t me = os_hal_motor_set_frequency(1, OS_GOTO_ARRIVE_FREQ_HZ);
        if (me == OS_ERR_MOTOR_DRIVER_FAULT) { enter_fault(); return me; }
    }

    if (rd && dd) {
        stop_axis(0);
        stop_axis(1);
        os_hal_motor_enable(0, false);
        os_hal_motor_enable(1, false);
        if (parking) {
            st = OS_STATE_PARKED;
            te = false;
            os_hal_buzzer_beep(110, 1);
        } else {
            st = OS_STATE_IDLE_TRACKING;
            te = true;
            os_hal_buzzer_beep(80, 1);
        }
        motion.active = false;
        return OS_ERR_NONE;
    }

    man_active = false;
    st = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

static void update_motion_and_limits(void) {
    if (gp.active) {
        if (os_hal_limit_is_triggered(g_axis)) {
            stop_axis(g_axis);
            enter_fault();
        }
        return;
    }

    if (motion.active) {
        for (uint8_t ax = 0; ax < 2; ++ax) {
            if (motion.done[ax]) continue;
            if (os_hal_limit_is_triggered(ax)) {
                if (last_cmd_valid[ax]) {
                    limit_safe_valid[ax] = true;
                    limit_safe_fw[ax] = !last_cmd_fw[ax];
                }
                stop_axis(ax);
                enter_fault();
                return;
            }
            int32_t pos = os_hal_motor_get_position(ax);
            int32_t diff = motion.t[ax] - pos;
            int32_t rem = abs32(diff);
            if (rem <= OS_GOTO_EPSILON_STEPS) {
                os_hal_motor_set_frequency(ax, 0);
                motion.done[ax] = true;
                continue;
            }

            uint32_t cur = motion.freq[ax];
            uint32_t inc = (uint32_t)floor((OS_GOTO_ACCEL_HZ_PER_SEC * (double)OS_LOOP_TICK_MS) / 1000.0);
            if (inc < 1) inc = 1;
            if (rem <= (int32_t)goto_decel_steps(cur)) {
                if (cur > OS_GOTO_ARRIVE_FREQ_HZ) {
                    if (cur > inc) cur -= inc;
                    else cur = OS_GOTO_ARRIVE_FREQ_HZ;
                }
            } else {
                uint32_t maxf = goto_max_freq();
                if (cur < maxf) {
                    uint32_t nf = cur + inc;
                    cur = (nf > maxf || nf < cur) ? maxf : nf;
                }
            }

            set_motor_direction(ax, diff > 0);
            os_hal_motor_enable(ax, true);
            os_error_t e = os_hal_motor_set_frequency(ax, cur);
            if (e == OS_ERR_MOTOR_DRIVER_FAULT) { enter_fault(); return; }
            motion.freq[ax] = cur;
        }

        if (motion.done[0] && motion.done[1]) {
            bool parking = motion.parking;
            motion.active = false;
            stop_axis(0);
            stop_axis(1);
            os_hal_motor_enable(0, false);
            os_hal_motor_enable(1, false);
            if (parking) {
                st = OS_STATE_PARKED;
                te = false;
                os_hal_buzzer_beep(110, 1);
            } else {
                st = OS_STATE_IDLE_TRACKING;
                te = true;
                os_hal_buzzer_beep(80, 1);
            }
        }
        return;
    }

    if (man_active) {
        uint8_t ax = man_axis;
        if (os_hal_limit_is_triggered(ax)) {
            if (last_cmd_valid[ax]) {
                limit_safe_valid[ax] = true;
                limit_safe_fw[ax] = !last_cmd_fw[ax];
            }
            stop_axis(ax);
            enter_fault();
            return;
        }
        set_motor_direction(ax, man_forward);
        os_hal_motor_enable(ax, true);
        os_error_t e = os_hal_motor_set_frequency(ax, speed_hz(man_speed));
        if (e == OS_ERR_MOTOR_DRIVER_FAULT) { enter_fault(); return; }
        return;
    }

    if (st == OS_STATE_IDLE_TRACKING && te) {
        for (uint8_t ax = 0; ax < 2; ++ax) {
            if (os_hal_limit_is_triggered(ax)) {
                if (last_cmd_valid[ax]) {
                    limit_safe_valid[ax] = true;
                    limit_safe_fw[ax] = !last_cmd_fw[ax];
                }
                stop_axis(ax);
                enter_fault();
                return;
            }
        }
    }
}

static void poll_gps_and_time(void) {
    os_site_info_t tmp;
    os_error_t e = os_hal_gps_poll(&tmp);
    if (e == OS_ERR_NONE && tmp.valid) {
        site = tmp;
        gps_locked = true;
    } else {
        gps_locked = false;
        uint32_t t = 0;
        if (os_hal_rtc_read(&t) == OS_ERR_NONE) site.utc_epoch_seconds = t;
        else site.utc_epoch_seconds = 0;
    }
}

static bool parse_u32(const char *s, uint32_t *v) {
    if (!s || !*s) return false;
    uint32_t val = 0;
    while (*s) {
        if (*s < '0' || *s > '9') return false;
        if (val > (UINT32_MAX - (uint32_t)(*s - '0')) / 10u) return false;
        val = val * 10u + (uint32_t)(*s - '0');
        ++s;
    }
    *v = val;
    return true;
}

static bool read_unsigned_max(const char **pp, int max, int *out) {
    const char *p = *pp;
    if (!p || !isdigit((unsigned char)*p)) return false;
    int v = 0;
    while (isdigit((unsigned char)*p)) {
        int d = *p - '0';
        if (v > (max - d) / 10) return false;
        v = v * 10 + d;
        ++p;
    }
    *pp = p;
    *out = v;
    return true;
}

static bool read_float_seconds_max(const char **pp, float max, float *out) {
    const char *p = *pp;
    if (!p || !isdigit((unsigned char)*p)) return false;
    float v = 0.0f;
    while (isdigit((unsigned char)*p)) {
        v = v * 10.0f + (float)(*p - '0');
        if (v > max) return false;
        ++p;
    }
    if (*p == '.') {
        ++p;
        float scale = 0.1f;
        while (isdigit((unsigned char)*p)) {
            v += (*p - '0') * scale;
            scale *= 0.1f;
            if (v > max) return false;
            ++p;
        }
    }
    *pp = p;
    *out = v;
    return true;
}

static bool parse_ra_string(const char *s, float *out) {
    if (!s || !out) return false;
    int h = 0, m = 0;
    float sec = 0.0f;
    const char *p = s;
    if (!read_unsigned_max(&p, 24, &h)) return false;
    if (*p == ':') {
        ++p;
        if (!read_unsigned_max(&p, 59, &m)) return false;
        if (*p == ':') {
            ++p;
            if (!read_float_seconds_max(&p, 59.999f, &sec)) return false;
        } else if (*p != 0) {
            return false;
        }
    } else if (*p == '.') {
        ++p;
        if (!isdigit((unsigned char)*p)) return false;
        float frac = 0.0f;
        float scale = 0.1f;
        while (isdigit((unsigned char)*p)) {
            frac += (*p - '0') * scale;
            scale *= 0.1f;
            ++p;
        }
        sec = frac * 3600.0f;
    } else if (*p != 0) {
        return false;
    }
    if (*p != 0) return false;
    float val = (float)h + (float)m / 60.0f + sec / 3600.0f;
    if (!valid_ra(val)) return false;
    *out = val;
    return true;
}

static bool parse_dec_string(const char *s, float *out) {
    if (!s || !out) return false;
    const char *p = s;
    int sign = 1;
    if (*p == '+') { sign = 1; ++p; }
    else if (*p == '-') { sign = -1; ++p; }
    else return false;
    int d = 0, m = 0;
    float sec = 0.0f;
    if (!read_unsigned_max(&p, 90, &d)) return false;
    if (*p == '*' || *p == ':') {
        ++p;
        if (!read_unsigned_max(&p, 59, &m)) return false;
        if (*p == ':') {
            ++p;
            if (!read_float_seconds_max(&p, 59.999f, &sec)) return false;
        } else if (*p != 0) {
            return false;
        }
    } else if (*p != 0) {
        return false;
    }
    if (*p != 0) return false;
    float val = (float)sign * ((float)d + (float)m / 60.0f + sec / 3600.0f);
    if (!valid_dec(val)) return false;
    *out = val;
    return true;
}

static void poll_commands(void) {
    for (uint8_t ch = 0; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        int16_t avail = os_hal_comm_available(ch);
        if (avail <= 0) continue;
        while (avail-- > 0) {
            char c = os_hal_comm_read(ch);
            if (!cmdin[ch]) {
                if (c == OS_LX200_CMD_PREFIX) {
                    cmdin[ch] = true;
                    cmdlen[ch] = 0;
                    cmdbuf[ch][cmdlen[ch]++] = c;
                }
                continue;
            }
            if (cmdlen[ch] < OS_MAX_COMMAND_LENGTH) cmdbuf[ch][cmdlen[ch]++] = c;
            if (c == OS_LX200_CMD_SUFFIX || cmdlen[ch] >= OS_MAX_COMMAND_LENGTH) {
                cmdin[ch] = false;
                if (cmdlen[ch] >= 2) {
                    char reply[OS_MAX_REPLY_LENGTH];
                    size_t rlen = 0;
                    os_error_t e = os_command_parse(cmdbuf[ch], cmdlen[ch], ch, reply, sizeof(reply), &rlen);
                    if (e == OS_ERR_NONE && rlen > 0) {
                        if (rlen > sizeof(reply)) rlen = sizeof(reply);
                        os_hal_comm_write(ch, reply, rlen);
                    } else if (e != OS_ERR_NONE) {
                        os_hal_comm_write(ch, "0", 1);
                    }
                }
            }
        }
    }
}

static bool qr_solve(int m, int n, double A[OS_CALIBRATION_MAX_STARS][3], double *B, double *X) {
    if (m < n || n <= 0 || n > 3) return false;
    for (int k = 0; k < n; ++k) {
        double v[OS_CALIBRATION_MAX_STARS];
        for (int i = 0; i < m; ++i) v[i] = (i < k) ? 0.0 : A[i][k];
        double norm = 0.0;
        for (int i = k; i < m; ++i) norm += v[i] * v[i];
        norm = sqrt(norm);
        if (norm < 1e-12) return false;
        double alpha = -copysign(norm, v[k]);
        if (fabs(alpha - v[k]) < 1e-12) alpha = -alpha;
        v[k] -= alpha;
        double beta = 0.0;
        for (int i = k; i < m; ++i) beta += v[i] * v[i];
        if (beta < 1e-30) return false;
        for (int j = k; j < n; ++j) {
            double dot = 0.0;
            for (int i = k; i < m; ++i) dot += v[i] * A[i][j];
            double tau = dot / beta;
            for (int i = k; i < m; ++i) A[i][j] -= tau * v[i];
        }
        for (int i = 0; i < m; ++i) {
            if (i == k) A[i][k] = alpha;
            else if (i > k) A[i][k] = 0.0;
        }
        double dotb = 0.0;
        for (int i = k; i < m; ++i) dotb += v[i] * B[i];
        double taub = dotb / beta;
        for (int i = k; i < m; ++i) B[i] -= taub * v[i];
    }
    for (int i = n - 1; i >= 0; --i) {
        double sum = B[i];
        for (int j = i + 1; j < n; ++j) sum -= A[i][j] * X[j];
        if (fabs(A[i][i]) < 1e-12) return false;
        X[i] = sum / A[i][i];
    }
    return true;
}

os_error_t os_init(void) {
    st = OS_STATE_INITIALIZING;
    memset(&gp, 0, sizeof(gp));
    gr_remaining = 0;
    memset(&motion, 0, sizeof(motion));
    motion.active = false;
    motion.parking = false;
    memset(align_star, 0, sizeof(align_star));
    memset(align_motor, 0, sizeof(align_motor));
    align_n = 0;
    align_active = false;
    residual_ok = false;
    residual_arcsec = 0.0f;
    man_active = false;
    te = false;
    pec_en = false;
    cal.valid = false;
    memset(&cal, 0, sizeof(cal));
    memset(&pec, 0, sizeof(pec));
    park_pos.ra_hours = 0.0f;
    park_pos.dec_degrees = 90.0f;
    park_set = false;
    nvm_ok = false;
    cfg_steps_per_arcsec = 1.0f;
    cfg_mount_type = OS_MOUNT_EQUATORIAL;
    current_worm_phase_deg = 0.0f;
    goto_target.ra_hours = 0.0f;
    goto_target.dec_degrees = 0.0f;
    for (int i = 0; i < 2; ++i) {
        last_cmd_valid[i] = false;
        last_cmd_fw[i] = false;
        limit_safe_valid[i] = false;
        limit_safe_fw[i] = false;
    }
    for (int i = 0; i < 4; ++i) {
        cmdin[i] = false;
        cmdlen[i] = 0;
        memset(cmdbuf[i], 0, OS_MAX_COMMAND_LENGTH);
    }

    if (os_hal_nvm_init() == OS_ERR_NONE) {
        nvm_ok = true;
        load_persistent_state();
        load_pec_table();
    }

    for (int i = 0; i <= OS_CHANNEL_ETHERNET; ++i) os_hal_comm_init((uint8_t)i);

    os_error_t e = os_hal_motor_init(0);
    if (e) { st = OS_STATE_FAULT; return e; }
    e = os_hal_motor_init(1);
    if (e) { st = OS_STATE_FAULT; return e; }
    os_hal_motor_enable(0, false);
    os_hal_motor_enable(1, false);

    os_hal_gps_init();
    os_hal_rtc_init();
    os_hal_limit_init();

    e = os_hal_timer_motor_init();
    if (e) { st = OS_STATE_FAULT; return e; }

    os_site_info_t tmp;
    e = os_hal_gps_poll(&tmp);
    if (e == OS_ERR_NONE && tmp.valid) {
        site = tmp;
        gps_locked = true;
        if (nvm_ok) write_site_record(&site);
        os_hal_rtc_set(tmp.utc_epoch_seconds);
    } else {
        gps_locked = false;
        uint32_t t = 0;
        if (nvm_ok) {
            os_site_info_t preset;
            if (read_site_record(&preset) == OS_ERR_NONE && preset.valid) {
                site = preset;
                site.valid = true;
            } else {
                site.latitude_degrees = 0.0f;
                site.longitude_degrees = 0.0f;
                site.elevation_metres = 0.0f;
                site.valid = true;
            }
        } else {
            site.latitude_degrees = 0.0f;
            site.longitude_degrees = 0.0f;
            site.elevation_metres = 0.0f;
            site.valid = true;
        }
        if (os_hal_rtc_read(&t) == OS_ERR_NONE) site.utc_epoch_seconds = t;
        else site.utc_epoch_seconds = 0;
    }

    te = true;
    tr = OS_TRACK_RATE_SIDEREAL;
    st = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    poll_gps_and_time();
    poll_commands();
    update_worm_phase();
    update_guide_pulse();
    update_motion_and_limits();

    if (!motion.active && !man_active && !gp.active && st == OS_STATE_IDLE_TRACKING && te) {
        for (uint8_t ax = 0; ax < 2; ++ax) {
            uint32_t f = tracking_hz_for_axis(ax, true);
            if (f > 0) {
                set_motor_direction(ax, true);
                os_hal_motor_enable(ax, true);
                os_hal_motor_set_frequency(ax, f);
            } else {
                os_hal_motor_set_frequency(ax, 0);
            }
        }
    }
}

os_error_t os_command_parse(const char *command, size_t length, uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size, size_t *reply_length) {
    if (!command || !reply_buffer || !reply_length || reply_buffer_size == 0) return OS_ERR_INVALID_ARGUMENT;
    if (source_channel > OS_CHANNEL_ETHERNET) return OS_ERR_INVALID_ARGUMENT;
    reply_buffer[0] = 0;
    *reply_length = 0;
    if (length < 2 || length > OS_MAX_COMMAND_LENGTH) return OS_ERR_INVALID_ARGUMENT;
    if (command[0] != OS_LX200_CMD_PREFIX || command[length - 1] != OS_LX200_CMD_SUFFIX) return OS_ERR_COMMAND_FORMAT;

    size_t body_len = length - 2;
    char tmp[OS_MAX_COMMAND_LENGTH];
    memcpy(tmp, command + 1, body_len);
    tmp[body_len] = 0;
    char *body = tmp;
    os_error_t e = OS_ERR_NONE;

    if (strcmp(body, "GR") == 0) { format_ra_reply(reply_buffer, reply_buffer_size, reply_length); return OS_ERR_NONE; }
    if (strcmp(body, "GD") == 0) { format_dec_reply(reply_buffer, reply_buffer_size, reply_length); return OS_ERR_NONE; }
    if (strcmp(body, "GVP") == 0) { set_reply(reply_buffer, reply_buffer_size, reply_length, "OnStep %u.%u.%u", OS_FIRMWARE_VERSION_MAJOR, OS_FIRMWARE_VERSION_MINOR, OS_FIRMWARE_VERSION_PATCH); return OS_ERR_NONE; }
    if (strcmp(body, "GS") == 0) { set_reply(reply_buffer, reply_buffer_size, reply_length, "%d", (int)st); return OS_ERR_NONE; }
    if (strcmp(body, "Gg") == 0) { bool l = false; os_query_gps_locked(&l); set_reply(reply_buffer, reply_buffer_size, reply_length, "%d", l ? 1 : 0); return OS_ERR_NONE; }
    if (strcmp(body, "Gm") == 0) { bool m = false; os_query_is_moving(&m); set_reply(reply_buffer, reply_buffer_size, reply_length, "%d", m ? 1 : 0); return OS_ERR_NONE; }

    if (strcmp(body, "MS") == 0) {
        e = os_goto_equatorial(goto_target);
    } else if (body_len > 2 && body[0] == 'S' && body[1] == 'r') {
        float ra = 0.0f;
        if (!parse_ra_string(body + 2, &ra)) return OS_ERR_COMMAND_FORMAT;
        goto_target.ra_hours = ra;
        e = os_goto_equatorial(goto_target);
    } else if (body_len > 2 && body[0] == 'S' && body[1] == 'd') {
        float dec = 0.0f;
        if (!parse_dec_string(body + 2, &dec)) return OS_ERR_COMMAND_FORMAT;
        goto_target.dec_degrees = dec;
        e = os_goto_equatorial(goto_target);
    } else if (strcmp(body, "Me") == 0) {
        e = os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
    } else if (strcmp(body, "Mw") == 0) {
        e = os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
    } else if (strcmp(body, "Mn") == 0) {
        e = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
    } else if (strcmp(body, "Ms") == 0) {
        e = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
    } else if (strcmp(body, "hP") == 0) {
        e = os_park();
    } else if (strcmp(body, "hO") == 0) {
        e = os_unpark();
    } else if (body[0] == 'M' && body[1] == 'g' && body_len >= 4) {
        os_direction_t d;
        uint32_t dur = 0;
        switch (body[2]) {
            case 'E': d = OS_DIRECTION_EAST; break;
            case 'W': d = OS_DIRECTION_WEST; break;
            case 'N': d = OS_DIRECTION_NORTH; break;
            case 'S': d = OS_DIRECTION_SOUTH; break;
            default: return OS_ERR_COMMAND_FORMAT;
        }
        if (!parse_u32(body + 3, &dur) || dur == 0) return OS_ERR_INVALID_ARGUMENT;
        e = os_guide_pulse(d, dur);
    } else {
        return OS_ERR_NOT_SUPPORTED;
    }

    if (e == OS_ERR_NONE) set_reply(reply_buffer, reply_buffer_size, reply_length, "1");
    else set_reply(reply_buffer, reply_buffer_size, reply_length, "0");
    return e;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (!valid_ra(target.ra_hours) || !valid_dec(target.dec_degrees)) return OS_ERR_INVALID_ARGUMENT;
    if (st == OS_STATE_INITIALIZING || st == OS_STATE_PARKED || st == OS_STATE_FAULT || st == OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE;
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) return OS_ERR_LIMIT_TRIGGERED;

    double ra_arcsec = (double)target.ra_hours * 54000.0;
    double de_arcsec = (double)target.dec_degrees * 3600.0;
    double rs, ds;
    if (cal.valid) {
        rs = (double)cal.matrix_ra_to_ra * ra_arcsec + (double)cal.matrix_ra_to_dec * de_arcsec + cal.offset_ra_arcsec;
        ds = (double)cal.matrix_dec_to_ra * ra_arcsec + (double)cal.matrix_dec_to_dec * de_arcsec + cal.offset_dec_arcsec;
    } else {
        rs = ra_arcsec * cfg_steps_per_arcsec;
        ds = de_arcsec * cfg_steps_per_arcsec;
    }
    return motion_start((int32_t)lround(rs), (int32_t)lround(ds), false);
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (!isfinite(target.altitude_degrees) || !isfinite(target.azimuth_degrees)) return OS_ERR_INVALID_ARGUMENT;
    if (target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f || target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f) return OS_ERR_INVALID_ARGUMENT;
    if (st == OS_STATE_INITIALIZING || st == OS_STATE_PARKED || st == OS_STATE_FAULT || st == OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE;
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) return OS_ERR_LIMIT_TRIGGERED;

    if (cfg_mount_type == OS_MOUNT_ALTAZ) {
        double az_steps = (double)target.azimuth_degrees * 3600.0 * cfg_steps_per_arcsec;
        double alt_steps = (double)target.altitude_degrees * 3600.0 * cfg_steps_per_arcsec;
        return motion_start((int32_t)lround(az_steps), (int32_t)lround(alt_steps), false);
    }

    if (!site.valid) {
        os_site_info_t tmp;
        os_error_t e = os_hal_gps_poll(&tmp);
        if (e == OS_ERR_NONE && tmp.valid) {
            site = tmp;
            gps_locked = true;
        } else {
            return OS_ERR_GPS_NO_SIGNAL;
        }
    }
    os_equatorial_coord_t eq;
    if (!horizontal_to_equatorial(&target, &site, &eq)) return OS_ERR_INVALID_ARGUMENT;
    return os_goto_equatorial(eq);
}

os_error_t os_goto_abort(void) {
    if (!motion.active) return OS_ERR_INVALID_STATE;
    stop_axis(0);
    stop_axis(1);
    motion.active = false;
    motion.parking = false;
    st = OS_STATE_IDLE_TRACKING;
    te = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (rate < OS_TRACK_RATE_SIDEREAL || rate > OS_TRACK_RATE_CUSTOM) return OS_ERR_INVALID_ARGUMENT;
    if (rate == OS_TRACK_RATE_CUSTOM && (!isfinite(custom_factor) || custom_factor <= 0.0f)) return OS_ERR_INVALID_ARGUMENT;
    tr = rate;
    cf = custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (!rate || !custom_factor) return OS_ERR_INVALID_ARGUMENT;
    *rate = tr;
    *custom_factor = cf;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    te = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    te = false;
    stop_axis(0);
    stop_axis(1);
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (!valid_dir(direction)) return OS_ERR_INVALID_ARGUMENT;
    if (duration_ms == 0) return OS_ERR_INVALID_ARGUMENT;
    if (st == OS_STATE_FAULT || st == OS_STATE_PARKED || st == OS_STATE_INITIALIZING) return OS_ERR_INVALID_STATE;
    if (motion.active || man_active) return OS_ERR_INVALID_STATE;

    uint8_t ax;
    bool fw;
    if (!map_direction(direction, &ax, &fw)) return OS_ERR_INVALID_ARGUMENT;
    bool dec_priority = (ax == 1);

    if (gp.active) {
        bool cur_dec = (g_axis == 1);
        if (cur_dec && !dec_priority) return OS_ERR_NONE;
    }

    gp.active = true;
    gp.duration_ms = duration_ms;
    gp.rate_fraction = guide_rate;
    gp.direction_east = (ax == 0 && fw);
    gp.direction_north = (ax == 1 && fw);
    gp.dec_priority = dec_priority;
    g_axis = ax;
    g_forward = fw;
    gr_remaining = duration_ms;
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (!isfinite(rate_fraction) || rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) return OS_ERR_INVALID_ARGUMENT;
    guide_rate = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (!pulse) return OS_ERR_INVALID_ARGUMENT;
    *pulse = gp;
    if (gp.active) pulse->duration_ms = gr_remaining;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) return OS_ERR_INVALID_ARGUMENT;
    st = OS_STATE_ALIGNMENT;
    align_active = true;
    align_mode = mode;
    align_n = 0;
    residual_ok = false;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord, os_motor_position_t motor_pos) {
    if (!valid_ra(star_coord.ra_hours) || !valid_dec(star_coord.dec_degrees)) return OS_ERR_INVALID_ARGUMENT;
    if (st != OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE;
    if (align_n >= OS_CALIBRATION_MAX_STARS) return OS_ERR_INVALID_STATE;
    align_star[align_n] = star_coord;
    align_motor[align_n] = motor_pos;
    align_n++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (st != OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE;
    if (align_n == 0) return OS_ERR_INVALID_STATE;
    if (align_mode == OS_ALIGN_1STAR && align_n < 1) return OS_ERR_INVALID_STATE;
    if (align_mode == OS_ALIGN_2STAR && align_n < 2) return OS_ERR_INVALID_STATE;
    if ((align_mode == OS_ALIGN_3STAR || align_mode == OS_ALIGN_NSTAR) && align_n < 3) return OS_ERR_INVALID_STATE;

    double m00 = 1.0, m01 = 0.0, m10 = 0.0, m11 = 1.0, off_ra = 0.0, off_dec = 0.0;
    uint8_t cnt = align_n;
    double ra[OS_CALIBRATION_MAX_STARS], de[OS_CALIBRATION_MAX_STARS];
    double mra[OS_CALIBRATION_MAX_STARS], mde[OS_CALIBRATION_MAX_STARS];
    double base_scale = (double)cfg_steps_per_arcsec;

    for (uint8_t i = 0; i < cnt; ++i) {
        ra[i] = (double)align_star[i].ra_hours * 54000.0;
        de[i] = (double)align_star[i].dec_degrees * 3600.0;
        mra[i] = (double)align_motor[i].ra_steps;
        mde[i] = (double)align_motor[i].dec_steps;
    }

    if (align_mode == OS_ALIGN_1STAR) {
        double sr = 0.0, sd = 0.0;
        for (uint8_t i = 0; i < cnt; ++i) {
            sr += mra[i] - base_scale * ra[i];
            sd += mde[i] - base_scale * de[i];
        }
        off_ra = sr / cnt;
        off_dec = sd / cnt;
        m00 = base_scale;
        m01 = 0.0;
        m10 = 0.0;
        m11 = base_scale;
    } else if (align_mode == OS_ALIGN_2STAR) {
        double A[OS_CALIBRATION_MAX_STARS][3];
        double B[OS_CALIBRATION_MAX_STARS];
        double X[3];
        memset(A, 0, sizeof(A));
        memset(B, 0, sizeof(B));
        memset(X, 0, sizeof(X));
        for (uint8_t i = 0; i < cnt; ++i) { A[i][0] = 1.0; A[i][1] = ra[i]; B[i] = mra[i]; }
        if (!qr_solve(cnt, 2, A, B, X)) return OS_ERR_CALIBRATION_FAILED;
        off_ra = X[0];
        m00 = X[1];
        m01 = 0.0;

        memset(A, 0, sizeof(A));
        memset(B, 0, sizeof(B));
        memset(X, 0, sizeof(X));
        for (uint8_t i = 0; i < cnt; ++i) { A[i][0] = 1.0; A[i][1] = de[i]; B[i] = mde[i]; }
        if (!qr_solve(cnt, 2, A, B, X)) return OS_ERR_CALIBRATION_FAILED;
        off_dec = X[0];
        m11 = X[1];
        m10 = 0.0;
    } else {
        double A[OS_CALIBRATION_MAX_STARS][3];
        double B[OS_CALIBRATION_MAX_STARS];
        double X[3];
        memset(A, 0, sizeof(A));
        memset(B, 0, sizeof(B));
        memset(X, 0, sizeof(X));
        for (uint8_t i = 0; i < cnt; ++i) { A[i][0] = 1.0; A[i][1] = ra[i]; A[i][2] = de[i]; B[i] = mra[i]; }

        if (cnt == 3) {
            double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
                       - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
                       + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
            if (fabs(det) < 1e-9) return OS_ERR_CALIBRATION_FAILED;
        }

        if (!qr_solve(cnt, 3, A, B, X)) return OS_ERR_CALIBRATION_FAILED;
        off_ra = X[0];
        m00 = X[1];
        m01 = X[2];

        memset(A, 0, sizeof(A));
        memset(B, 0, sizeof(B));
        memset(X, 0, sizeof(X));
        for (uint8_t i = 0; i < cnt; ++i) { A[i][0] = 1.0; A[i][1] = ra[i]; A[i][2] = de[i]; B[i] = mde[i]; }
        if (!qr_solve(cnt, 3, A, B, X)) return OS_ERR_CALIBRATION_FAILED;
        off_dec = X[0];
        m10 = X[1];
        m11 = X[2];
    }

    double det = m00 * m11 - m01 * m10;
    if (fabs(det) < 1e-12) return OS_ERR_CALIBRATION_FAILED;

    double ss = 0.0;
    for (uint8_t i = 0; i < cnt; ++i) {
        double drm = mra[i] - off_ra;
        double ddm = mde[i] - off_dec;
        double pred_ra = (m11 * drm - m01 * ddm) / det;
        double pred_de = (-m10 * drm + m00 * ddm) / det;
        double er = pred_ra - ra[i];
        double ed = pred_de - de[i];
        ss += er * er + ed * ed;
    }
    float resid = (float)sqrt(ss / cnt);
    if (!isfinite(resid)) return OS_ERR_CALIBRATION_FAILED;
    residual_arcsec = resid;
    residual_ok = true;

    if (residual_arcsec > OS_MAX_RESIDUAL_ARCSEC && cnt > 3) return OS_ERR_CALIBRATION_FAILED;

    cal.matrix_ra_to_ra = (float)m00;
    cal.matrix_ra_to_dec = (float)m01;
    cal.matrix_dec_to_ra = (float)m10;
    cal.matrix_dec_to_dec = (float)m11;
    cal.offset_ra_arcsec = (float)off_ra;
    cal.offset_dec_arcsec = (float)off_dec;
    cal.valid = true;
    if (nvm_ok) write_calibration_record(&cal);
    st = OS_STATE_IDLE_TRACKING;
    te = true;
    align_active = false;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual) {
    if (!residual) return OS_ERR_INVALID_ARGUMENT;
    if (!residual_ok) return OS_ERR_INVALID_STATE;
    *residual = residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    if (st != OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE;
    st = OS_STATE_IDLE_TRACKING;
    te = true;
    align_active = false;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (st == OS_STATE_PARKED) return OS_ERR_INVALID_STATE;
    if (st == OS_STATE_INITIALIZING || st == OS_STATE_FAULT || st == OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE;
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) return OS_ERR_LIMIT_TRIGGERED;

    double ra_arcsec = (double)park_pos.ra_hours * 54000.0;
    double de_arcsec = (double)park_pos.dec_degrees * 3600.0;
    double rs, ds;
    if (cal.valid) {
        rs = (double)cal.matrix_ra_to_ra * ra_arcsec + (double)cal.matrix_ra_to_dec * de_arcsec + cal.offset_ra_arcsec;
        ds = (double)cal.matrix_dec_to_ra * ra_arcsec + (double)cal.matrix_dec_to_dec * de_arcsec + cal.offset_dec_arcsec;
    } else {
        rs = ra_arcsec * cfg_steps_per_arcsec;
        ds = de_arcsec * cfg_steps_per_arcsec;
    }
    return motion_start((int32_t)lround(rs), (int32_t)lround(ds), true);
}

os_error_t os_unpark(void) {
    if (st != OS_STATE_PARKED) return OS_ERR_INVALID_STATE;
    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);
    for (int i = 0; i <= OS_CHANNEL_ETHERNET; ++i) os_hal_comm_init((uint8_t)i);
    os_site_info_t tmp;
    if (os_hal_gps_poll(&tmp) == OS_ERR_NONE && tmp.valid) {
        site = tmp;
        gps_locked = true;
        os_hal_rtc_set(tmp.utc_epoch_seconds);
    }
    st = OS_STATE_IDLE_TRACKING;
    te = true;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t p) {
    if (!valid_ra(p.ra_hours) || !valid_dec(p.dec_degrees)) return OS_ERR_INVALID_ARGUMENT;
    park_pos = p;
    park_set = true;
    if (nvm_ok) return write_park_record(&park_pos, park_set);
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (!valid_dir(direction)) return OS_ERR_INVALID_ARGUMENT;
    if (!valid_speed(speed)) return OS_ERR_INVALID_ARGUMENT;
    if (speed == OS_SPEED_CUSTOM && (!isfinite(custom_speed_arcsec) || custom_speed_arcsec <= 0.0f)) return OS_ERR_INVALID_ARGUMENT;
    if (st != OS_STATE_IDLE_TRACKING) return OS_ERR_INVALID_STATE;

    uint8_t ax;
    bool fw;
    if (!map_direction(direction, &ax, &fw)) return OS_ERR_INVALID_ARGUMENT;

    if (os_hal_limit_is_triggered(ax)) {
        if (!limit_safe_valid[ax] || fw != limit_safe_fw[ax]) return OS_ERR_LIMIT_TRIGGERED;
    }

    man_active = true;
    man_axis = ax;
    man_forward = fw;
    man_speed = speed;
    st = OS_STATE_MANUAL_MOTION;
    os_hal_motor_enable(ax, true);
    set_motor_direction(ax, fw);
    os_error_t me = os_hal_motor_set_frequency(ax, speed_hz(speed));
    if (me == OS_ERR_MOTOR_DRIVER_FAULT) { enter_fault(); return me; }
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (!man_active || st != OS_STATE_MANUAL_MOTION) return OS_ERR_INVALID_STATE;
    stop_axis(man_axis);
    man_active = false;
    st = OS_STATE_IDLE_TRACKING;
    te = true;
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (!isfinite(arcsec_per_sec) || arcsec_per_sec <= 0.0f) return OS_ERR_INVALID_ARGUMENT;
    custom_speed_arcsec = arcsec_per_sec;
    if (man_active && st == OS_STATE_MANUAL_MOTION && man_speed == OS_SPEED_CUSTOM) {
        os_hal_motor_set_frequency(man_axis, speed_hz(OS_SPEED_CUSTOM));
    }
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (!state) return OS_ERR_INVALID_ARGUMENT;
    *state = st;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (!coord) return OS_ERR_INVALID_ARGUMENT;
    return get_current_eq(coord);
}

os_error_t os_query_site(os_site_info_t *out) {
    if (!out) return OS_ERR_INVALID_ARGUMENT;
    os_site_info_t tmp;
    os_error_t e = os_hal_gps_poll(&tmp);
    if (e == OS_ERR_NONE && tmp.valid) {
        site = tmp;
        gps_locked = true;
        *out = tmp;
        return OS_ERR_NONE;
    }
    *out = site;
    return OS_ERR_GPS_NO_SIGNAL;
}

os_error_t os_query_motor_position(os_motor_position_t *pos) {
    if (!pos) return OS_ERR_INVALID_ARGUMENT;
    pos->ra_steps = os_hal_motor_get_position(0);
    pos->dec_steps = os_hal_motor_get_position(1);
    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor, uint8_t *patch) {
    if (!major || !minor || !patch) return OS_ERR_INVALID_ARGUMENT;
    *major = OS_FIRMWARE_VERSION_MAJOR;
    *minor = OS_FIRMWARE_VERSION_MINOR;
    *patch = OS_FIRMWARE_VERSION_PATCH;
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *moving) {
    if (!moving) return OS_ERR_INVALID_ARGUMENT;
    *moving = motion.active || man_active || (gp.active && gr_remaining > 0);
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (!locked) return OS_ERR_INVALID_ARGUMENT;
    os_site_info_t tmp;
    os_error_t e = os_hal_gps_poll(&tmp);
    *locked = (e == OS_ERR_NONE && tmp.valid);
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    pec_en = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (!table) return OS_ERR_INVALID_ARGUMENT;
    memcpy(&pec, table, sizeof(pec));
    pec.valid = true;
    if (nvm_ok) return write_pec_table(&pec);
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (!table) return OS_ERR_INVALID_ARGUMENT;
    memcpy(table, &pec, sizeof(pec));
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (!isfinite(worm_phase_deg) || worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) return OS_ERR_INVALID_ARGUMENT;
    int idx = (int)floorf(worm_phase_deg);
    if (idx >= OS_PEC_TABLE_SIZE) idx = OS_PEC_TABLE_SIZE - 1;
    pec.corrections[idx] = error_arcsec;
    pec.valid = true;
    current_worm_phase_deg = fmodf(worm_phase_deg, 360.0f);
    if (nvm_ok) return write_pec_table(&pec);
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *out) {
    if (!out) return OS_ERR_INVALID_ARGUMENT;
    *out = cal;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    cal.valid = false;
    memset(&cal, 0, sizeof(cal));
    if (nvm_ok) write_calibration_record(&cal);
    return OS_ERR_NONE;
}
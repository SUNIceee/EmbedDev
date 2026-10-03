#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define OS_AXIS_RA 0u
#define OS_AXIS_DEC 1u
#define OS_AXIS_COUNT 2u
#define OS_CHANNEL_COUNT 4u
#define OS_NVM_TOTAL_SIZE (OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES + (OS_PEC_TABLE_SIZE * 2u) + 64u)
#define OS_MAX_MOTOR_FREQUENCY_HZ 200000u
#define OS_STEPS_PER_RA_HOUR 1000.0f
#define OS_STEPS_PER_DEC_DEG 1000.0f
#define OS_ARCSEC_PER_STEP 1.0f
#define OS_GOTO_TOLERANCE_STEPS 2
#define OS_GOTO_STEP_CHUNK 100
#define OS_MANUAL_TIMEOUT_TICKS 1000u
#define OS_TRACKING_BASE_FREQUENCY_HZ 15u
#define OS_COMMAND_RX_SIZE 128u
#define OS_COMMAND_TX_SIZE 512u
#define OS_CAL_RESIDUAL_LIMIT_ARCSEC 600.0

typedef struct {
    os_equatorial_coord_t coord;
    os_motor_position_t pos;
} alignment_sample_t;

os_state_t os_state = OS_STATE_INITIALIZING;
bool goto_active = false;
bool manual_motion_active = false;
bool alignment_residual_valid = false;
bool calibration_residual_valid = false;
uint8_t alignment_sample_count = 0u;
uint8_t calibration_star_count = 0u;
bool gps_locked = false;
bool nvm_record_valid = false;
os_error_t last_fault = OS_ERR_NONE;
uint32_t timer_tick = 0u;
bool is_moving = false;
bool motor_initialized[2] = { false, false };
bool motor_enabled[2] = { false, false };
bool motor_direction[2] = { true, true };
uint32_t motor_frequency_hz[2] = { 0u, 0u };
os_motor_position_t motor_position_steps = { 0, 0 };
bool limit_triggered[2] = { false, false };
bool communication_channel_state[4] = { false, false, false, false };
bool tracking_enabled = false;
os_track_rate_t tracking_rate = OS_TRACK_RATE_SIDEREAL;
float tracking_custom_factor = 1.0f;
float guide_rate_fraction = OS_GUIDE_RATE_MIN;
os_guide_pulse_t guide_pulse = { false, 0u, OS_GUIDE_RATE_MIN, false, false, false };
os_calibration_t calibration = { 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, false };
float calibration_residual_arcsec = 0.0f;
os_pec_table_t pec_table;
bool pec_enabled = false;
os_site_info_t site_info = { 0.0f, 0.0f, 0.0f, 0u, false };
bool parked = false;
os_equatorial_coord_t park_position = { 0.0f, 90.0f };

static bool gps_initialized = false;
static bool rtc_initialized = false;
static bool limit_initialized = false;
static bool nvm_initialized = false;
static bool motor_timer_initialized = false;
static uint8_t nvm_content[OS_NVM_TOTAL_SIZE];
static int32_t position_steps[2] = { 0, 0 };
static uint32_t rtc_utc_epoch_seconds = 0u;
static os_site_info_t gps_site = { 0.0f, 0.0f, 0.0f, 0u, false };
static char rx_stream[4][OS_COMMAND_RX_SIZE];
static uint16_t rx_head[4];
static uint16_t rx_count[4];
static char tx_stream[4][OS_COMMAND_TX_SIZE];
static uint16_t tx_count[4];
static char command_frame[4][OS_MAX_COMMAND_LENGTH];
static size_t command_frame_len[4];
static bool command_in_frame[4];
static int32_t goto_target_steps[2];
static int32_t park_target_steps[2];
static bool park_motion_active = false;
static uint8_t manual_axis = OS_AXIS_RA;
static uint32_t manual_deadline_tick = 0u;
static uint32_t guide_deadline_tick = 0u;
static float custom_manual_speed_arcsec_per_sec = 60.0f;
static alignment_sample_t alignment_samples[OS_CALIBRATION_MAX_STARS];
static os_align_mode_t alignment_mode = OS_ALIGN_1STAR;
static double pec_phase_deg = 0.0;

static bool valid_channel(uint8_t channel) { return channel < OS_CHANNEL_COUNT; }
static bool valid_axis(uint8_t axis) { return axis < OS_AXIS_COUNT; }
static bool finite_float(float value) { return isfinite((double)value); }
static bool valid_equatorial(os_equatorial_coord_t c) { return finite_float(c.ra_hours) && finite_float(c.dec_degrees) && c.ra_hours >= OS_RA_MIN_HOURS && c.ra_hours <= OS_RA_MAX_HOURS && c.dec_degrees >= OS_DEC_MIN_DEG && c.dec_degrees <= OS_DEC_MAX_DEG; }
static bool valid_direction(os_direction_t d) { return d == OS_DIRECTION_NORTH || d == OS_DIRECTION_SOUTH || d == OS_DIRECTION_EAST || d == OS_DIRECTION_WEST; }
static bool valid_speed(os_speed_level_t s) { return s == OS_SPEED_SLOW || s == OS_SPEED_MEDIUM || s == OS_SPEED_FAST || s == OS_SPEED_CUSTOM; }
static bool tick_reached(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }
static void sync_moving(void) { is_moving = goto_active || manual_motion_active || park_motion_active; }
static void sync_position_snapshot(void) { motor_position_steps.ra_steps = position_steps[0]; motor_position_steps.dec_steps = position_steps[1]; }
static uint32_t rate_frequency(float factor) { double f = (double)OS_TRACKING_BASE_FREQUENCY_HZ * (double)factor; if (f < 0.0) f = 0.0; if (f > (double)OS_MAX_MOTOR_FREQUENCY_HZ) f = (double)OS_MAX_MOTOR_FREQUENCY_HZ; return (uint32_t)f; }
static float tracking_factor(void) { if (tracking_rate == OS_TRACK_RATE_LUNAR) return OS_LUNAR_RATE_FACTOR; if (tracking_rate == OS_TRACK_RATE_SOLAR) return OS_SOLAR_RATE_FACTOR; if (tracking_rate == OS_TRACK_RATE_CUSTOM) return tracking_custom_factor; return 1.0f; }
static uint8_t direction_axis(os_direction_t d) { return (d == OS_DIRECTION_EAST || d == OS_DIRECTION_WEST) ? OS_AXIS_RA : OS_AXIS_DEC; }
static bool direction_forward(os_direction_t d) { return (d == OS_DIRECTION_EAST || d == OS_DIRECTION_NORTH); }
static void enter_fault(os_error_t fault) { (void)os_hal_motor_set_frequency(0u, 0u); (void)os_hal_motor_set_frequency(1u, 0u); (void)os_hal_motor_enable(0u, false); (void)os_hal_motor_enable(1u, false); goto_active = false; manual_motion_active = false; park_motion_active = false; guide_pulse.active = false; tracking_enabled = false; last_fault = fault; os_state = OS_STATE_FAULT; sync_moving(); }
static bool any_limit_triggered(void) { return os_hal_limit_is_triggered(0u) || os_hal_limit_is_triggered(1u); }
static int32_t coord_ra_to_steps(float ra_hours) { return (int32_t)(ra_hours * OS_STEPS_PER_RA_HOUR); }
static int32_t coord_dec_to_steps(float dec_degrees) { return (int32_t)(dec_degrees * OS_STEPS_PER_DEC_DEG); }
static void set_axis_motion(uint8_t axis, bool forward, uint32_t frequency) { if (valid_axis(axis)) { (void)os_hal_motor_enable(axis, true); (void)os_hal_motor_set_direction(axis, forward); (void)os_hal_motor_set_frequency(axis, frequency); } }
static void stop_all_frequency(void) { (void)os_hal_motor_set_frequency(0u, 0u); (void)os_hal_motor_set_frequency(1u, 0u); }
static bool advance_axis_to(uint8_t axis, int32_t target) { int32_t current = position_steps[axis]; int32_t delta = target - current; if (delta >= -OS_GOTO_TOLERANCE_STEPS && delta <= OS_GOTO_TOLERANCE_STEPS) { position_steps[axis] = target; return true; } if (delta > 0) { position_steps[axis] += (delta > OS_GOTO_STEP_CHUNK) ? OS_GOTO_STEP_CHUNK : delta; } else { int32_t step = (-delta > OS_GOTO_STEP_CHUNK) ? OS_GOTO_STEP_CHUNK : -delta; position_steps[axis] -= step; } return false; }
static os_error_t start_goto_steps(int32_t ra_target, int32_t dec_target) { goto_target_steps[0] = ra_target; goto_target_steps[1] = dec_target; set_axis_motion(0u, ra_target >= position_steps[0], 1000u); set_axis_motion(1u, dec_target >= position_steps[1], 1000u); goto_active = true; park_motion_active = false; os_state = OS_STATE_GOTO; sync_moving(); return OS_ERR_NONE; }
static void complete_goto(void) { stop_all_frequency(); goto_active = false; tracking_rate = OS_TRACK_RATE_SIDEREAL; tracking_enabled = true; os_state = OS_STATE_IDLE_TRACKING; (void)os_hal_buzzer_beep(100u, 1u); sync_moving(); }
static void apply_tracking(void) { if (!tracking_enabled || os_state == OS_STATE_PARKED || os_state == OS_STATE_FAULT || goto_active || manual_motion_active || park_motion_active) return; uint32_t f = rate_frequency(tracking_factor()); if (pec_enabled && pec_table.valid) { uint16_t idx = (uint16_t)pec_phase_deg; if (idx >= OS_PEC_TABLE_SIZE) idx = 0u; int32_t adjusted = (int32_t)f + (int32_t)(pec_table.corrections[idx] / 10); f = adjusted > 0 ? (uint32_t)adjusted : 0u; pec_phase_deg += 1.0; if (pec_phase_deg >= 360.0) pec_phase_deg = 0.0; } set_axis_motion(0u, true, f); }
static void expire_guide(void) { if (guide_pulse.active && tick_reached(timer_tick, guide_deadline_tick)) { guide_pulse.active = false; guide_pulse.duration_ms = 0u; guide_deadline_tick = 0u; apply_tracking(); } }
static void expire_manual(void) { if (manual_motion_active && manual_deadline_tick != 0u && tick_reached(timer_tick, manual_deadline_tick)) { (void)os_move_stop(); } }
static uint8_t minimum_stars(os_align_mode_t mode) { if (mode == OS_ALIGN_1STAR) return 1u; if (mode == OS_ALIGN_2STAR) return 2u; return 3u; }
static double absd(double x) { return x < 0.0 ? -x : x; }
static bool solve3(double a[3][3], double b[3], double x[3]) { for (int i = 0; i < 3; ++i) { int pivot = i; double best = absd(a[i][i]); for (int r = i + 1; r < 3; ++r) { double v = absd(a[r][i]); if (v > best) { best = v; pivot = r; } } if (best < 1.0e-12) return false; if (pivot != i) { for (int c = i; c < 3; ++c) { double t = a[i][c]; a[i][c] = a[pivot][c]; a[pivot][c] = t; } double tb = b[i]; b[i] = b[pivot]; b[pivot] = tb; } double diag = a[i][i]; for (int c = i; c < 3; ++c) a[i][c] /= diag; b[i] /= diag; for (int r = 0; r < 3; ++r) { if (r == i) continue; double f = a[r][i]; for (int c = i; c < 3; ++c) a[r][c] -= f * a[i][c]; b[r] -= f * b[i]; } } x[0] = b[0]; x[1] = b[1]; x[2] = b[2]; return true; }
static bool solve_affine_qr(uint8_t n, double y[OS_CALIBRATION_MAX_STARS], double out[3]) { double q[OS_CALIBRATION_MAX_STARS][3]; double r[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}}; for (uint8_t i = 0; i < n; ++i) { q[i][0] = (double)alignment_samples[i].coord.ra_hours * 3600.0; q[i][1] = (double)alignment_samples[i].coord.dec_degrees * 3600.0; q[i][2] = 1.0; }
    for (int k = 0; k < 3; ++k) { for (int j = 0; j < k; ++j) { double dot = 0.0; for (uint8_t i = 0; i < n; ++i) dot += q[i][j] * q[i][k]; r[j][k] = dot; for (uint8_t i = 0; i < n; ++i) q[i][k] -= dot * q[i][j]; } double norm = 0.0; for (uint8_t i = 0; i < n; ++i) norm += q[i][k] * q[i][k]; norm = sqrt(norm); if (norm < 1.0e-9) return false; r[k][k] = norm; for (uint8_t i = 0; i < n; ++i) q[i][k] /= norm; }
    double qty[3] = {0.0, 0.0, 0.0}; for (int k = 0; k < 3; ++k) for (uint8_t i = 0; i < n; ++i) qty[k] += q[i][k] * y[i]; for (int i = 2; i >= 0; --i) { double v = qty[i]; for (int j = i + 1; j < 3; ++j) v -= r[i][j] * out[j]; if (absd(r[i][i]) < 1.0e-12) return false; out[i] = v / r[i][i]; } return true; }
static bool three_star_degenerate(void) { double x1 = (double)alignment_samples[0].coord.ra_hours * 3600.0, y1 = (double)alignment_samples[0].coord.dec_degrees * 3600.0; double x2 = (double)alignment_samples[1].coord.ra_hours * 3600.0, y2 = (double)alignment_samples[1].coord.dec_degrees * 3600.0; double x3 = (double)alignment_samples[2].coord.ra_hours * 3600.0, y3 = (double)alignment_samples[2].coord.dec_degrees * 3600.0; double det = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1); return absd(det) < 1.0e-6; }
static void write_reply(char *reply, size_t size, size_t *len, const char *text) { size_t n = strlen(text); if (n >= size) n = size ? size - 1u : 0u; if (size != 0u) { memcpy(reply, text, n); reply[n] = '\0'; } *len = n; }

os_error_t os_init(void) {
    os_state = OS_STATE_INITIALIZING;
    goto_active = false; manual_motion_active = false; park_motion_active = false; guide_pulse.active = false;
    alignment_residual_valid = false; calibration_residual_valid = false; alignment_sample_count = 0u; calibration_star_count = 0u;
    gps_locked = false; tracking_enabled = false; parked = false; is_moving = false; last_fault = OS_ERR_NONE; timer_tick = 0u;
    guide_deadline_tick = 0u; manual_deadline_tick = 0u; pec_enabled = false; pec_phase_deg = 0.0;
    stop_all_frequency();
    os_error_t first_error = os_hal_nvm_init();
    nvm_record_valid = (first_error == OS_ERR_NONE);
    for (uint8_t c = 0u; c < OS_CHANNEL_COUNT; ++c) { os_error_t e = os_hal_comm_init(c); if (e != OS_ERR_NONE && first_error == OS_ERR_NONE) first_error = e; }
    for (uint8_t a = 0u; a < OS_AXIS_COUNT; ++a) { os_error_t e = os_hal_motor_init(a); if (e != OS_ERR_NONE) { enter_fault(e); return e; } }
    os_error_t gps_e = os_hal_gps_init(); (void)gps_e;
    os_error_t rtc_e = os_hal_rtc_init();
    os_error_t lim_e = os_hal_limit_init(); if (lim_e != OS_ERR_NONE) { enter_fault(lim_e); return lim_e; }
    os_error_t tim_e = os_hal_timer_motor_init(); if (tim_e != OS_ERR_NONE) { enter_fault(tim_e); return tim_e; }
    os_site_info_t gps_sample; if (os_hal_gps_poll(&gps_sample) == OS_ERR_NONE && gps_sample.valid) { site_info = gps_sample; gps_locked = true; (void)os_hal_rtc_set(gps_sample.utc_epoch_seconds); } else { uint32_t utc = 0u; gps_locked = false; if (rtc_e == OS_ERR_NONE && os_hal_rtc_read(&utc) == OS_ERR_NONE) { site_info.latitude_degrees = 0.0f; site_info.longitude_degrees = 0.0f; site_info.elevation_metres = 0.0f; site_info.utc_epoch_seconds = utc; site_info.valid = true; } else { site_info.valid = false; last_fault = OS_ERR_TIMEOUT; } }
    tracking_rate = OS_TRACK_RATE_SIDEREAL; tracking_custom_factor = 1.0f; os_state = OS_STATE_IDLE_TRACKING; sync_position_snapshot(); return first_error == OS_ERR_NVM_FAULT ? OS_ERR_NVM_FAULT : OS_ERR_NONE;
}

void os_loop_iteration(void) {
    timer_tick++;
    for (uint8_t a = 0u; a < OS_AXIS_COUNT; ++a) { limit_triggered[a] = os_hal_limit_is_triggered(a); if (limit_triggered[a] && (goto_active || manual_motion_active || park_motion_active)) { enter_fault(OS_ERR_LIMIT_TRIGGERED); return; } }
    os_site_info_t gps_sample; if (os_hal_gps_poll(&gps_sample) == OS_ERR_NONE && gps_sample.valid) { gps_locked = true; site_info = gps_sample; (void)os_hal_rtc_set(gps_sample.utc_epoch_seconds); }
    for (uint8_t c = 0u; c < OS_CHANNEL_COUNT; ++c) { int16_t avail = os_hal_comm_available(c); while (avail > 0) { char ch = os_hal_comm_read(c); if (ch == OS_LX200_CMD_PREFIX) { command_in_frame[c] = true; command_frame_len[c] = 0u; command_frame[c][command_frame_len[c]++] = ch; } else if (command_in_frame[c]) { if (command_frame_len[c] < OS_MAX_COMMAND_LENGTH) command_frame[c][command_frame_len[c]++] = ch; if (ch == OS_LX200_CMD_SUFFIX || command_frame_len[c] >= OS_MAX_COMMAND_LENGTH) { char reply[OS_MAX_REPLY_LENGTH]; size_t reply_len = 0u; os_error_t e = os_command_parse(command_frame[c], command_frame_len[c], c, reply, sizeof(reply), &reply_len); if (e != OS_ERR_NONE && reply_len == 0u) { write_reply(reply, sizeof(reply), &reply_len, "0#"); } (void)os_hal_comm_write(c, reply, reply_len); command_in_frame[c] = false; command_frame_len[c] = 0u; } } avail--; } }
    expire_guide(); expire_manual();
    if (goto_active) { bool ra_done = advance_axis_to(0u, goto_target_steps[0]); bool dec_done = advance_axis_to(1u, goto_target_steps[1]); if (ra_done && dec_done) complete_goto(); }
    if (park_motion_active) { bool ra_done = advance_axis_to(0u, park_target_steps[0]); bool dec_done = advance_axis_to(1u, park_target_steps[1]); if (ra_done && dec_done) { stop_all_frequency(); tracking_enabled = false; park_motion_active = false; parked = true; os_state = OS_STATE_PARKED; (void)os_hal_motor_enable(0u, false); (void)os_hal_motor_enable(1u, false); (void)os_hal_buzzer_beep(100u, 1u); sync_moving(); } }
    apply_tracking(); sync_position_snapshot(); sync_moving();
}

os_error_t os_command_parse(const char *command, size_t length, uint8_t source_channel, char *reply_buffer, size_t reply_buffer_size, size_t *reply_length) {
    if (command == NULL || reply_buffer == NULL || reply_length == NULL) return OS_ERR_INVALID_ARGUMENT;
    *reply_length = 0u;
    if (!valid_channel(source_channel) || length == 0u || length > OS_MAX_COMMAND_LENGTH || reply_buffer_size == 0u || reply_buffer_size > OS_MAX_REPLY_LENGTH) return OS_ERR_INVALID_ARGUMENT;
    if (command[0] != OS_LX200_CMD_PREFIX || command[length - 1u] != OS_LX200_CMD_SUFFIX) return OS_ERR_COMMAND_FORMAT;
    if (length >= 5u && memcmp(command, ":GVP#", 5u) == 0) { write_reply(reply_buffer, reply_buffer_size, reply_length, "OnStep 1.0.0#"); return OS_ERR_NONE; }
    if (length >= 3u && command[1] == 'G') { if (command[2] == 'R') { os_equatorial_coord_t c; (void)os_query_coordinates(&c); (void)snprintf(reply_buffer, reply_buffer_size, "%02d:%02d:%02d#", (int)c.ra_hours, (int)((c.ra_hours - (int)c.ra_hours) * 60.0f), 0); *reply_length = strlen(reply_buffer); return OS_ERR_NONE; } if (command[2] == 'D') { os_equatorial_coord_t c; (void)os_query_coordinates(&c); (void)snprintf(reply_buffer, reply_buffer_size, "%+03d*00:00#", (int)c.dec_degrees); *reply_length = strlen(reply_buffer); return OS_ERR_NONE; } if (command[2] == 'S') { (void)snprintf(reply_buffer, reply_buffer_size, "%d#", (int)os_state); *reply_length = strlen(reply_buffer); return OS_ERR_NONE; } }
    if (length >= 4u && command[1] == 'M') { os_error_t e = OS_ERR_COMMAND_FORMAT; if (command[2] == 'S') e = os_goto_equatorial((os_equatorial_coord_t){ 0.0f, 0.0f }); else if (command[2] == 'e') e = os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM); else if (command[2] == 'w') e = os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM); else if (command[2] == 'n') e = os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM); else if (command[2] == 's') e = os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM); write_reply(reply_buffer, reply_buffer_size, reply_length, e == OS_ERR_NONE ? "1#" : "0#"); return e; }
    if (length >= 4u && command[1] == 'Q') { os_error_t e = os_move_stop(); write_reply(reply_buffer, reply_buffer_size, reply_length, e == OS_ERR_NONE ? "1#" : "0#"); return e; }
    if (length >= 4u && command[1] == 'h' && command[2] == 'P') { os_error_t e = os_park(); write_reply(reply_buffer, reply_buffer_size, reply_length, e == OS_ERR_NONE ? "1#" : "0#"); return e; }
    if (length >= 4u && command[1] == 'h' && command[2] == 'O') { os_error_t e = os_unpark(); write_reply(reply_buffer, reply_buffer_size, reply_length, e == OS_ERR_NONE ? "1#" : "0#"); return e; }
    return OS_ERR_COMMAND_FORMAT;
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) { if (!valid_equatorial(target)) return OS_ERR_INVALID_ARGUMENT; if (os_state == OS_STATE_PARKED || os_state == OS_STATE_FAULT || os_state == OS_STATE_INITIALIZING) return OS_ERR_INVALID_STATE; if (any_limit_triggered()) { last_fault = OS_ERR_LIMIT_TRIGGERED; return OS_ERR_LIMIT_TRIGGERED; } return start_goto_steps(coord_ra_to_steps(target.ra_hours), coord_dec_to_steps(target.dec_degrees)); }
os_error_t os_goto_horizontal(os_horizontal_coord_t target) { if (!finite_float(target.azimuth_degrees) || !finite_float(target.altitude_degrees) || target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f || target.altitude_degrees < -90.0f || target.altitude_degrees > 90.0f) return OS_ERR_INVALID_ARGUMENT; if (os_state == OS_STATE_PARKED || os_state == OS_STATE_FAULT || os_state == OS_STATE_INITIALIZING) return OS_ERR_INVALID_STATE; if (any_limit_triggered()) { last_fault = OS_ERR_LIMIT_TRIGGERED; return OS_ERR_LIMIT_TRIGGERED; } return start_goto_steps((int32_t)(target.azimuth_degrees * 1000.0f), (int32_t)(target.altitude_degrees * 1000.0f)); }
os_error_t os_goto_abort(void) { if (!goto_active || os_state != OS_STATE_GOTO) return OS_ERR_INVALID_STATE; stop_all_frequency(); goto_active = false; os_state = OS_STATE_IDLE_TRACKING; sync_moving(); return OS_ERR_NONE; }

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) { if (!(rate == OS_TRACK_RATE_SIDEREAL || rate == OS_TRACK_RATE_LUNAR || rate == OS_TRACK_RATE_SOLAR || rate == OS_TRACK_RATE_CUSTOM)) return OS_ERR_INVALID_ARGUMENT; if (rate == OS_TRACK_RATE_CUSTOM && (!finite_float(custom_factor) || custom_factor <= 0.0f || custom_factor > 10.0f)) return OS_ERR_INVALID_ARGUMENT; if (os_state == OS_STATE_PARKED || os_state == OS_STATE_FAULT || os_state == OS_STATE_INITIALIZING) return OS_ERR_INVALID_STATE; tracking_rate = rate; if (rate == OS_TRACK_RATE_CUSTOM) tracking_custom_factor = custom_factor; apply_tracking(); return OS_ERR_NONE; }
os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) { if (rate == NULL || custom_factor == NULL) return OS_ERR_INVALID_ARGUMENT; *rate = tracking_rate; *custom_factor = tracking_custom_factor; return OS_ERR_NONE; }
os_error_t os_tracking_enable(void) { if (os_state == OS_STATE_PARKED || os_state == OS_STATE_FAULT || os_state == OS_STATE_INITIALIZING || !site_info.valid) return OS_ERR_INVALID_STATE; if (os_hal_limit_is_triggered(0u)) return OS_ERR_LIMIT_TRIGGERED; tracking_enabled = true; os_state = OS_STATE_IDLE_TRACKING; apply_tracking(); return OS_ERR_NONE; }
os_error_t os_tracking_disable(void) { (void)os_hal_motor_set_frequency(0u, 0u); tracking_enabled = false; return OS_ERR_NONE; }

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) { if (!valid_direction(direction) || duration_ms == 0u) return OS_ERR_INVALID_ARGUMENT; if (os_state == OS_STATE_PARKED || os_state == OS_STATE_FAULT || os_state == OS_STATE_INITIALIZING) return OS_ERR_INVALID_STATE; uint8_t axis = direction_axis(direction); if (os_hal_limit_is_triggered(axis)) return OS_ERR_LIMIT_TRIGGERED; guide_pulse.active = true; guide_pulse.duration_ms = duration_ms; guide_pulse.rate_fraction = guide_rate_fraction; guide_pulse.direction_east = (direction == OS_DIRECTION_EAST); guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH); guide_pulse.dec_priority = (axis == OS_AXIS_DEC); guide_deadline_tick = timer_tick + duration_ms; set_axis_motion(axis, direction_forward(direction), rate_frequency(guide_rate_fraction)); return OS_ERR_NONE; }
os_error_t os_guide_set_rate(float rate_fraction) { if (!finite_float(rate_fraction) || rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) return OS_ERR_INVALID_ARGUMENT; guide_rate_fraction = rate_fraction; if (guide_pulse.active) guide_pulse.rate_fraction = rate_fraction; return OS_ERR_NONE; }
os_error_t os_guide_get_state(os_guide_pulse_t *pulse) { if (pulse == NULL) return OS_ERR_INVALID_ARGUMENT; expire_guide(); *pulse = guide_pulse; return OS_ERR_NONE; }

os_error_t os_align_begin(os_align_mode_t mode) { if (!(mode == OS_ALIGN_1STAR || mode == OS_ALIGN_2STAR || mode == OS_ALIGN_3STAR || mode == OS_ALIGN_NSTAR)) return OS_ERR_INVALID_ARGUMENT; if (os_state == OS_STATE_PARKED || os_state == OS_STATE_GOTO || os_state == OS_STATE_MANUAL_MOTION || os_state == OS_STATE_FAULT) return OS_ERR_INVALID_STATE; alignment_mode = mode; alignment_sample_count = 0u; calibration_star_count = 0u; alignment_residual_valid = false; calibration_residual_valid = false; calibration_residual_arcsec = 0.0f; os_state = OS_STATE_ALIGNMENT; return OS_ERR_NONE; }
os_error_t os_align_accept_star(os_equatorial_coord_t star_coord, os_motor_position_t motor_pos) { if (!valid_equatorial(star_coord)) return OS_ERR_INVALID_ARGUMENT; if (alignment_sample_count >= OS_CALIBRATION_MAX_STARS) return OS_ERR_INVALID_ARGUMENT; if (os_state != OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE; alignment_samples[alignment_sample_count].coord = star_coord; alignment_samples[alignment_sample_count].pos = motor_pos; alignment_sample_count++; calibration_star_count = alignment_sample_count; return OS_ERR_NONE; }
os_error_t os_align_compute(void) { if (os_state != OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE; if (alignment_sample_count < minimum_stars(alignment_mode)) return OS_ERR_INVALID_STATE; alignment_residual_valid = false; calibration_residual_valid = false; os_calibration_t old = calibration; os_calibration_t next = old; if (alignment_mode == OS_ALIGN_1STAR) { next.offset_ra_arcsec = (float)((double)alignment_samples[0].pos.ra_steps - ((double)alignment_samples[0].coord.ra_hours * 3600.0)); next.offset_dec_arcsec = (float)((double)alignment_samples[0].pos.dec_steps - ((double)alignment_samples[0].coord.dec_degrees * 3600.0)); next.matrix_ra_to_ra = 1.0f; next.matrix_ra_to_dec = 0.0f; next.matrix_dec_to_ra = 0.0f; next.matrix_dec_to_dec = 1.0f; } else if (alignment_mode == OS_ALIGN_2STAR) { double dra = ((double)alignment_samples[1].coord.ra_hours - (double)alignment_samples[0].coord.ra_hours) * 3600.0; double ddec = ((double)alignment_samples[1].coord.dec_degrees - (double)alignment_samples[0].coord.dec_degrees) * 3600.0; if (absd(dra) < 1.0e-9 || absd(ddec) < 1.0e-9) return OS_ERR_CALIBRATION_FAILED; next.matrix_ra_to_ra = (float)(((double)alignment_samples[1].pos.ra_steps - (double)alignment_samples[0].pos.ra_steps) / dra); next.matrix_dec_to_dec = (float)(((double)alignment_samples[1].pos.dec_steps - (double)alignment_samples[0].pos.dec_steps) / ddec); next.matrix_ra_to_dec = 0.0f; next.matrix_dec_to_ra = 0.0f; next.offset_ra_arcsec = (float)((double)alignment_samples[0].pos.ra_steps - (double)next.matrix_ra_to_ra * ((double)alignment_samples[0].coord.ra_hours * 3600.0)); next.offset_dec_arcsec = (float)((double)alignment_samples[0].pos.dec_steps - (double)next.matrix_dec_to_dec * ((double)alignment_samples[0].coord.dec_degrees * 3600.0)); } else { if (alignment_sample_count == 3u && three_star_degenerate()) return OS_ERR_CALIBRATION_FAILED; double yra[OS_CALIBRATION_MAX_STARS], ydec[OS_CALIBRATION_MAX_STARS], cra[3] = {0.0,0.0,0.0}, cdec[3] = {0.0,0.0,0.0}; for (uint8_t i = 0u; i < alignment_sample_count; ++i) { yra[i] = (double)alignment_samples[i].pos.ra_steps; ydec[i] = (double)alignment_samples[i].pos.dec_steps; } if (!solve_affine_qr(alignment_sample_count, yra, cra) || !solve_affine_qr(alignment_sample_count, ydec, cdec)) return OS_ERR_CALIBRATION_FAILED; next.matrix_ra_to_ra = (float)cra[0]; next.matrix_dec_to_ra = (float)cra[1]; next.offset_ra_arcsec = (float)cra[2]; next.matrix_ra_to_dec = (float)cdec[0]; next.matrix_dec_to_dec = (float)cdec[1]; next.offset_dec_arcsec = (float)cdec[2]; }
    next.valid = true; double rss = 0.0; for (uint8_t i = 0u; i < alignment_sample_count; ++i) { double ra = (double)alignment_samples[i].coord.ra_hours * 3600.0; double dec = (double)alignment_samples[i].coord.dec_degrees * 3600.0; double pra = (double)next.matrix_ra_to_ra * ra + (double)next.matrix_dec_to_ra * dec + (double)next.offset_ra_arcsec; double pdec = (double)next.matrix_ra_to_dec * ra + (double)next.matrix_dec_to_dec * dec + (double)next.offset_dec_arcsec; double era = pra - (double)alignment_samples[i].pos.ra_steps; double edec = pdec - (double)alignment_samples[i].pos.dec_steps; rss += era * era + edec * edec; }
    double residual = sqrt(rss / (double)alignment_sample_count); if (alignment_sample_count >= 4u && residual > OS_CAL_RESIDUAL_LIMIT_ARCSEC) { calibration = old; return OS_ERR_CALIBRATION_FAILED; }
    calibration = next; calibration_residual_arcsec = (float)residual; alignment_residual_valid = true; calibration_residual_valid = true; if (os_hal_nvm_write(0u, (const uint8_t *)&calibration, (uint16_t)sizeof(calibration)) != OS_ERR_NONE) { calibration = old; return OS_ERR_NVM_FAULT; } os_state = OS_STATE_IDLE_TRACKING; return OS_ERR_NONE; }
os_error_t os_align_get_residual(float *residual_arcsec) { if (residual_arcsec == NULL) return OS_ERR_INVALID_ARGUMENT; if (!calibration_residual_valid) return OS_ERR_INVALID_STATE; *residual_arcsec = calibration_residual_arcsec; return OS_ERR_NONE; }
os_error_t os_align_abort(void) { if (os_state != OS_STATE_ALIGNMENT) return OS_ERR_INVALID_STATE; alignment_sample_count = 0u; calibration_star_count = 0u; alignment_residual_valid = false; calibration_residual_valid = false; os_state = OS_STATE_IDLE_TRACKING; return OS_ERR_NONE; }

os_error_t os_park(void) { if (os_state == OS_STATE_FAULT || os_state == OS_STATE_INITIALIZING || os_state == OS_STATE_PARKED) return OS_ERR_INVALID_STATE; if (any_limit_triggered()) return OS_ERR_LIMIT_TRIGGERED; park_target_steps[0] = coord_ra_to_steps(park_position.ra_hours); park_target_steps[1] = coord_dec_to_steps(park_position.dec_degrees); park_motion_active = true; goto_active = false; tracking_enabled = false; set_axis_motion(0u, park_target_steps[0] >= position_steps[0], 1000u); set_axis_motion(1u, park_target_steps[1] >= position_steps[1], 1000u); sync_moving(); return OS_ERR_NONE; }
os_error_t os_unpark(void) { if (os_state != OS_STATE_PARKED || !parked) return OS_ERR_INVALID_STATE; for (uint8_t c = 0u; c < OS_CHANNEL_COUNT; ++c) { os_error_t e = os_hal_comm_init(c); if (e != OS_ERR_NONE) { enter_fault(e); return e; } } for (uint8_t a = 0u; a < OS_AXIS_COUNT; ++a) { os_error_t e = os_hal_motor_enable(a, true); if (e != OS_ERR_NONE) { enter_fault(e); return e; } } uint32_t utc = 0u; if (os_hal_rtc_read(&utc) != OS_ERR_NONE) { enter_fault(OS_ERR_TIMEOUT); return OS_ERR_TIMEOUT; } site_info.utc_epoch_seconds = utc; site_info.valid = true; parked = false; tracking_enabled = true; os_state = OS_STATE_IDLE_TRACKING; apply_tracking(); return OS_ERR_NONE; }
os_error_t os_park_set_position(os_equatorial_coord_t park_pos) { if (!valid_equatorial(park_pos)) return OS_ERR_INVALID_ARGUMENT; if (park_motion_active || os_state == OS_STATE_FAULT) return OS_ERR_INVALID_STATE; park_position = park_pos; if (os_hal_nvm_write(OS_NVM_CALIBRATION_SIZE_BYTES, (const uint8_t *)&park_position, (uint16_t)sizeof(park_position)) != OS_ERR_NONE) return OS_ERR_NVM_FAULT; return OS_ERR_NONE; }

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) { if (!valid_direction(direction) || !valid_speed(speed)) return OS_ERR_INVALID_ARGUMENT; if (os_state == OS_STATE_PARKED || os_state == OS_STATE_FAULT || os_state == OS_STATE_INITIALIZING || os_state == OS_STATE_GOTO) return OS_ERR_INVALID_STATE; uint8_t axis = direction_axis(direction); if (os_hal_limit_is_triggered(axis)) return OS_ERR_LIMIT_TRIGGERED; uint32_t freq = 50u; if (speed == OS_SPEED_MEDIUM) freq = 250u; else if (speed == OS_SPEED_FAST) freq = 1000u; else if (speed == OS_SPEED_CUSTOM) freq = rate_frequency(custom_manual_speed_arcsec_per_sec / OS_SIDEREAL_RATE_ARCSEC_PER_SEC); manual_axis = axis; set_axis_motion(axis, direction_forward(direction), freq); manual_motion_active = true; manual_deadline_tick = timer_tick + OS_MANUAL_TIMEOUT_TICKS; os_state = OS_STATE_MANUAL_MOTION; sync_moving(); return OS_ERR_NONE; }
os_error_t os_move_stop(void) { if (valid_axis(manual_axis)) (void)os_hal_motor_set_frequency(manual_axis, 0u); manual_motion_active = false; manual_deadline_tick = 0u; if (os_state == OS_STATE_MANUAL_MOTION) os_state = OS_STATE_IDLE_TRACKING; tracking_enabled = true; sync_moving(); apply_tracking(); return OS_ERR_NONE; }
os_error_t os_move_set_custom_speed(float arcsec_per_sec) { if (!finite_float(arcsec_per_sec) || arcsec_per_sec < 0.0f || arcsec_per_sec > (OS_GOTO_SPEED_MAX_DEG_PER_SEC * 3600.0f)) return OS_ERR_INVALID_ARGUMENT; custom_manual_speed_arcsec_per_sec = arcsec_per_sec; if (manual_motion_active) (void)os_hal_motor_set_frequency(manual_axis, rate_frequency(custom_manual_speed_arcsec_per_sec / OS_SIDEREAL_RATE_ARCSEC_PER_SEC)); return OS_ERR_NONE; }

os_error_t os_query_state(os_state_t *state) { if (state == NULL) return OS_ERR_INVALID_ARGUMENT; *state = os_state; return OS_ERR_NONE; }
os_error_t os_query_coordinates(os_equatorial_coord_t *coord) { if (coord == NULL) return OS_ERR_INVALID_ARGUMENT; sync_position_snapshot(); double ra_arcsec = (double)motor_position_steps.ra_steps; double dec_arcsec = (double)motor_position_steps.dec_steps; if (calibration.valid) { ra_arcsec -= (double)calibration.offset_ra_arcsec; dec_arcsec -= (double)calibration.offset_dec_arcsec; } coord->ra_hours = (float)(ra_arcsec / 3600.0); coord->dec_degrees = (float)(dec_arcsec / 3600.0); if (coord->ra_hours < OS_RA_MIN_HOURS) coord->ra_hours = OS_RA_MIN_HOURS; if (coord->ra_hours > OS_RA_MAX_HOURS) coord->ra_hours = OS_RA_MAX_HOURS; if (coord->dec_degrees < OS_DEC_MIN_DEG) coord->dec_degrees = OS_DEC_MIN_DEG; if (coord->dec_degrees > OS_DEC_MAX_DEG) coord->dec_degrees = OS_DEC_MAX_DEG; return OS_ERR_NONE; }
os_error_t os_query_site(os_site_info_t *site) { if (site == NULL) return OS_ERR_INVALID_ARGUMENT; if (!site_info.valid) return OS_ERR_INVALID_STATE; *site = site_info; return OS_ERR_NONE; }
os_error_t os_query_motor_position(os_motor_position_t *pos) { if (pos == NULL) return OS_ERR_INVALID_ARGUMENT; pos->ra_steps = os_hal_motor_get_position(0u); pos->dec_steps = os_hal_motor_get_position(1u); motor_position_steps = *pos; return OS_ERR_NONE; }
os_error_t os_query_firmware_version(uint8_t *major, uint8_t *minor, uint8_t *patch) { if (major == NULL || minor == NULL || patch == NULL) return OS_ERR_INVALID_ARGUMENT; *major = OS_FIRMWARE_VERSION_MAJOR; *minor = OS_FIRMWARE_VERSION_MINOR; *patch = OS_FIRMWARE_VERSION_PATCH; return OS_ERR_NONE; }
os_error_t os_query_is_moving(bool *moving) { if (moving == NULL) return OS_ERR_INVALID_ARGUMENT; expire_guide(); expire_manual(); sync_moving(); *moving = is_moving; return OS_ERR_NONE; }
os_error_t os_query_gps_locked(bool *locked) { if (locked == NULL) return OS_ERR_INVALID_ARGUMENT; *locked = gps_locked; return OS_ERR_NONE; }

os_error_t os_pec_enable(bool enable) { if (enable && !pec_table.valid) return OS_ERR_INVALID_STATE; pec_enabled = enable; if (!enable) pec_phase_deg = 0.0; return OS_ERR_NONE; }
os_error_t os_pec_load_table(const os_pec_table_t *table) { if (table == NULL || !table->valid) return OS_ERR_INVALID_ARGUMENT; for (uint16_t i = 0u; i < OS_PEC_TABLE_SIZE; ++i) pec_table.corrections[i] = table->corrections[i]; pec_table.valid = true; if (os_hal_nvm_write((uint16_t)(OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES), (const uint8_t *)&pec_table.corrections[0], (uint16_t)(OS_PEC_TABLE_SIZE * sizeof(int16_t))) != OS_ERR_NONE) return OS_ERR_NVM_FAULT; return OS_ERR_NONE; }
os_error_t os_pec_get_table(os_pec_table_t *table) { if (table == NULL) return OS_ERR_INVALID_ARGUMENT; *table = pec_table; return OS_ERR_NONE; }
os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) { if (!finite_float(worm_phase_deg) || worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) return OS_ERR_INVALID_ARGUMENT; uint16_t idx = (worm_phase_deg == 360.0f) ? 0u : (uint16_t)worm_phase_deg; if (idx >= OS_PEC_TABLE_SIZE) idx = OS_PEC_TABLE_SIZE - 1u; pec_table.corrections[idx] = error_arcsec; pec_table.valid = true; if (os_hal_nvm_write((uint16_t)(OS_NVM_CALIBRATION_SIZE_BYTES + OS_NVM_CONFIG_SIZE_BYTES + idx * sizeof(int16_t)), (const uint8_t *)&pec_table.corrections[idx], (uint16_t)sizeof(int16_t)) != OS_ERR_NONE) return OS_ERR_NVM_FAULT; return OS_ERR_NONE; }
os_error_t os_calibration_get(os_calibration_t *calib) { if (calib == NULL) return OS_ERR_INVALID_ARGUMENT; *calib = calibration; return OS_ERR_NONE; }
os_error_t os_calibration_clear(void) { calibration.valid = false; calibration_residual_valid = false; alignment_residual_valid = false; uint8_t zero = 0u; if (os_hal_nvm_write(0u, &zero, 1u) != OS_ERR_NONE) return OS_ERR_NVM_FAULT; return OS_ERR_NONE; }

os_error_t os_hal_motor_init(uint8_t axis) { if (axis >= 2u) return OS_ERR_INVALID_ARGUMENT; motor_initialized[axis] = true; motor_enabled[axis] = false; motor_frequency_hz[axis] = 0u; motor_direction[axis] = true; return OS_ERR_NONE; }
os_error_t os_hal_motor_set_frequency(uint8_t axis, uint32_t frequency_hz) { if (axis >= 2u || frequency_hz > OS_MAX_MOTOR_FREQUENCY_HZ) return OS_ERR_INVALID_ARGUMENT; if (!motor_initialized[axis]) return OS_ERR_INVALID_STATE; motor_frequency_hz[axis] = frequency_hz; return OS_ERR_NONE; }
os_error_t os_hal_motor_set_direction(uint8_t axis, bool forward) { if (axis >= 2u) return OS_ERR_INVALID_ARGUMENT; if (!motor_initialized[axis]) return OS_ERR_INVALID_STATE; motor_direction[axis] = forward; return OS_ERR_NONE; }
os_error_t os_hal_motor_enable(uint8_t axis, bool enable) { if (axis >= 2u) return OS_ERR_INVALID_ARGUMENT; if (!motor_initialized[axis]) return OS_ERR_INVALID_STATE; if (!enable) motor_frequency_hz[axis] = 0u; motor_enabled[axis] = enable; return OS_ERR_NONE; }
int32_t os_hal_motor_get_position(uint8_t axis) { if (axis >= 2u) return 0; return position_steps[axis]; }
os_error_t os_hal_gps_init(void) { gps_initialized = true; gps_locked = false; gps_site.valid = false; return OS_ERR_NONE; }
os_error_t os_hal_gps_poll(os_site_info_t *site) { if (site == NULL) return OS_ERR_INVALID_ARGUMENT; if (!gps_initialized || !gps_site.valid) { site->valid = false; gps_locked = false; return OS_ERR_GPS_NO_SIGNAL; } if (gps_site.latitude_degrees < -90.0f || gps_site.latitude_degrees > 90.0f || gps_site.longitude_degrees < -180.0f || gps_site.longitude_degrees > 180.0f) { site->valid = false; gps_locked = false; return OS_ERR_GPS_NO_SIGNAL; } *site = gps_site; gps_locked = true; return OS_ERR_NONE; }
os_error_t os_hal_rtc_init(void) { rtc_initialized = true; return OS_ERR_NONE; }
os_error_t os_hal_rtc_read(uint32_t *utc_epoch_seconds) { if (utc_epoch_seconds == NULL) return OS_ERR_INVALID_ARGUMENT; if (!rtc_initialized) return OS_ERR_INVALID_STATE; *utc_epoch_seconds = rtc_utc_epoch_seconds; return OS_ERR_NONE; }
os_error_t os_hal_rtc_set(uint32_t utc_epoch_seconds) { if (!rtc_initialized) return OS_ERR_INVALID_STATE; rtc_utc_epoch_seconds = utc_epoch_seconds; return OS_ERR_NONE; }
os_error_t os_hal_limit_init(void) { limit_initialized = true; limit_triggered[0] = false; limit_triggered[1] = false; return OS_ERR_NONE; }
bool os_hal_limit_is_triggered(uint8_t axis) { if (axis >= 2u) return true; (void)limit_initialized; return limit_triggered[axis]; }
os_error_t os_hal_nvm_init(void) { nvm_initialized = true; return OS_ERR_NONE; }
os_error_t os_hal_nvm_read(uint16_t offset, uint8_t *data, uint16_t length) { if (data == NULL) return OS_ERR_INVALID_ARGUMENT; if ((uint32_t)offset + (uint32_t)length > (uint32_t)OS_NVM_TOTAL_SIZE) return OS_ERR_INVALID_ARGUMENT; if (!nvm_initialized) return OS_ERR_NVM_FAULT; memcpy(data, &nvm_content[offset], length); return OS_ERR_NONE; }
os_error_t os_hal_nvm_write(uint16_t offset, const uint8_t *data, uint16_t length) { if (data == NULL) return OS_ERR_INVALID_ARGUMENT; if ((uint32_t)offset + (uint32_t)length > (uint32_t)OS_NVM_TOTAL_SIZE) return OS_ERR_INVALID_ARGUMENT; if (!nvm_initialized) return OS_ERR_NVM_FAULT; memcpy(&nvm_content[offset], data, length); return OS_ERR_NONE; }
os_error_t os_hal_comm_init(uint8_t channel) { if (!valid_channel(channel)) return OS_ERR_INVALID_ARGUMENT; communication_channel_state[channel] = true; rx_head[channel] = 0u; rx_count[channel] = 0u; tx_count[channel] = 0u; command_frame_len[channel] = 0u; command_in_frame[channel] = false; return OS_ERR_NONE; }
int16_t os_hal_comm_available(uint8_t channel) { if (!valid_channel(channel) || !communication_channel_state[channel]) return 0; return (int16_t)rx_count[channel]; }
char os_hal_comm_read(uint8_t channel) { if (!valid_channel(channel) || rx_count[channel] == 0u) return '\0'; char ch = rx_stream[channel][rx_head[channel]]; rx_head[channel] = (uint16_t)((rx_head[channel] + 1u) % OS_COMMAND_RX_SIZE); rx_count[channel]--; return ch; }
os_error_t os_hal_comm_write(uint8_t channel, const char *data, size_t length) { if (!valid_channel(channel) || data == NULL || length > OS_MAX_REPLY_LENGTH) return OS_ERR_INVALID_ARGUMENT; if (!communication_channel_state[channel]) return OS_ERR_NOT_SUPPORTED; size_t room = OS_COMMAND_TX_SIZE - tx_count[channel]; if (length > room) length = room; memcpy(&tx_stream[channel][tx_count[channel]], data, length); tx_count[channel] = (uint16_t)(tx_count[channel] + length); return OS_ERR_NONE; }
os_error_t os_hal_buzzer_beep(uint16_t duration_ms, uint8_t count) { if (duration_ms == 0u || count == 0u) return OS_ERR_INVALID_ARGUMENT; return OS_ERR_NONE; }
os_error_t os_hal_timer_motor_init(void) { motor_timer_initialized = true; motor_frequency_hz[0] = 0u; motor_frequency_hz[1] = 0u; return OS_ERR_NONE; }
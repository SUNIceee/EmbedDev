/* Implementation of system state queries */

#include "onstep_internal.h"

os_error_t os_query_state(os_state_t *state) {
    if (!state) return OS_ERR_INVALID_ARGUMENT;
    *state = g_os_ctx.system_state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (!coord) return OS_ERR_INVALID_ARGUMENT;
    int32_t ra_pos = os_hal_motor_get_position(0);
    int32_t dec_pos = os_hal_motor_get_position(1);
    os_steps_to_eq(ra_pos, dec_pos, &g_os_ctx.calibration, coord);
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (!site) return OS_ERR_INVALID_ARGUMENT;
    *site = g_os_ctx.current_site;
    return OS_ERR_NONE;
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
    *moving = (g_os_ctx.system_state == OS_STATE_GOTO || g_os_ctx.system_state == OS_STATE_MANUAL_MOTION);
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (!locked) return OS_ERR_INVALID_ARGUMENT;
    *locked = g_os_ctx.current_site.valid;
    return OS_ERR_NONE;
}

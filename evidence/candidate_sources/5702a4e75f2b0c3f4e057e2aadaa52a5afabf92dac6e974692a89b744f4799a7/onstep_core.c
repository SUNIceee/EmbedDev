/*
 * OnStep Core Execution & System State Logic
 */

#include "onstep_internal.h"
#include <string.h>

os_system_context_t g_os_ctx;

os_error_t os_init(void) {
    memset(&g_os_ctx, 0, sizeof(g_os_ctx));

    g_os_ctx.state = OS_STATE_INITIALIZING;
    g_os_ctx.mount_type = OS_MOUNT_EQUATORIAL;
    
    g_os_ctx.align_residual_computed = false;
    g_os_ctx.align_star_count = 0;
    g_os_ctx.align_active = false;
    
    g_os_ctx.guide_pulse_state.active = false;
    g_os_ctx.guide_pulse_state.duration_ms = 0;
    g_os_ctx.guide_rate_fraction = 0.5f;
    g_os_ctx.guide_pulse_state.rate_fraction = 0.5f;

    g_os_ctx.manual_moving = false;
    g_os_ctx.is_moving = false;
    g_os_ctx.tracking_enabled = true;
    g_os_ctx.track_rate = OS_TRACK_RATE_SIDEREAL;
    g_os_ctx.custom_track_factor = 1.0f;
    g_os_ctx.custom_speed_arcsec_per_sec = 15.0f;

    g_os_ctx.park_position.ra_hours = 0.0f;
    g_os_ctx.park_position.dec_degrees = 90.0f;

    os_hal_nvm_init();
    os_hal_timer_motor_init();
    os_hal_limit_init();
    os_hal_gps_init();
    os_hal_rtc_init();

    for (uint8_t ch = 0; ch < 4; ++ch) {
        os_hal_comm_init(ch);
    }
    for (uint8_t axis = 0; axis < 2; ++axis) {
        os_hal_motor_init(axis);
        os_hal_motor_enable(axis, true);
    }

    os_calibration_t loaded_calib;
    if (os_hal_nvm_read(OS_NVM_OFFSET_CALIBRATION, (uint8_t *)&loaded_calib, sizeof(loaded_calib)) == OS_ERR_NONE) {
        if (loaded_calib.valid) {
            g_os_ctx.calibration = loaded_calib;
        }
    }

    os_site_info_t site;
    if (os_hal_gps_poll(&site) == OS_ERR_NONE && site.valid) {
        g_os_ctx.site_info = site;
        g_os_ctx.gps_locked = true;
    } else {
        g_os_ctx.gps_locked = false;
        uint32_t utc = 0;
        if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
            g_os_ctx.site_info.utc_epoch_seconds = utc;
        }
        g_os_ctx.site_info.latitude_degrees = 0.0f;
        g_os_ctx.site_info.longitude_degrees = 0.0f;
        g_os_ctx.site_info.elevation_metres = 0.0f;
        g_os_ctx.site_info.valid = false;
    }

    g_os_ctx.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        if (g_os_ctx.is_moving || g_os_ctx.manual_moving) {
            os_goto_abort();
            g_os_ctx.state = OS_STATE_FAULT;
        }
    }

    if (g_os_ctx.state == OS_STATE_GOTO && g_os_ctx.is_moving) {
        int32_t cur_ra = os_hal_motor_get_position(0);
        int32_t cur_dec = os_hal_motor_get_position(1);

        int32_t diff_ra = g_os_ctx.target_motor_pos.ra_steps - cur_ra;
        int32_t diff_dec = g_os_ctx.target_motor_pos.dec_steps - cur_dec;

        int32_t step_ra = (diff_ra > 500) ? 500 : ((diff_ra < -500) ? -500 : diff_ra);
        int32_t step_dec = (diff_dec > 500) ? 500 : ((diff_dec < -500) ? -500 : diff_dec);

        g_os_ctx.current_motor_pos.ra_steps += step_ra;
        g_os_ctx.current_motor_pos.dec_steps += step_dec;

        if (g_os_ctx.current_motor_pos.ra_steps == g_os_ctx.target_motor_pos.ra_steps &&
            g_os_ctx.current_motor_pos.dec_steps == g_os_ctx.target_motor_pos.dec_steps) {
            g_os_ctx.is_moving = false;
            g_os_ctx.state = OS_STATE_IDLE_TRACKING;
            os_hal_buzzer_beep(200, 1);
        }
    }

    for (uint8_t ch = 0; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        int16_t avail = os_hal_comm_available(ch);
        while (avail > 0) {
            char c = os_hal_comm_read(ch);
            avail--;
            if (g_os_ctx.rx_buf_pos[ch] < OS_MAX_COMMAND_LENGTH - 1) {
                g_os_ctx.rx_buffers[ch][g_os_ctx.rx_buf_pos[ch]++] = c;
            }
            if (c == OS_LX200_CMD_SUFFIX || c == '\n' || c == '\r') {
                if (g_os_ctx.rx_buf_pos[ch] > 0) {
                    g_os_ctx.rx_buffers[ch][g_os_ctx.rx_buf_pos[ch]] = '\0';
                    char reply[OS_MAX_REPLY_LENGTH] = {0};
                    size_t rlen = 0;
                    if (os_command_parse(g_os_ctx.rx_buffers[ch], g_os_ctx.rx_buf_pos[ch], ch, reply, sizeof(reply), &rlen) == OS_ERR_NONE && rlen > 0) {
                        os_hal_comm_write(ch, reply, rlen);
                    }
                    g_os_ctx.rx_buf_pos[ch] = 0;
                }
            }
        }
    }
}

os_error_t os_park(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    os_motor_position_t park_steps;
    os_equatorial_to_motor_steps(g_os_ctx.park_position, &g_os_ctx.calibration, &park_steps);
    g_os_ctx.target_motor_pos = park_steps;
    g_os_ctx.current_motor_pos = park_steps;
    
    os_hal_motor_enable(0, false);
    os_hal_motor_enable(1, false);
    g_os_ctx.tracking_enabled = false;
    g_os_ctx.is_moving = false;
    g_os_ctx.state = OS_STATE_PARKED;
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);
    for (uint8_t ch = 0; ch <= OS_CHANNEL_ETHERNET; ++ch) {
        os_hal_comm_init(ch);
    }
    uint32_t utc = 0;
    if (os_hal_rtc_read(&utc) == OS_ERR_NONE) {
        g_os_ctx.site_info.utc_epoch_seconds = utc;
    }
    g_os_ctx.tracking_enabled = true;
    g_os_ctx.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos) {
    if (park_pos.ra_hours < OS_RA_MIN_HOURS || park_pos.ra_hours > OS_RA_MAX_HOURS ||
        park_pos.dec_degrees < OS_DEC_MIN_DEG || park_pos.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_os_ctx.park_position = park_pos;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state) {
    if (!state) return OS_ERR_INVALID_ARGUMENT;
    *state = g_os_ctx.state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord) {
    if (!coord) return OS_ERR_INVALID_ARGUMENT;
    os_motor_steps_to_equatorial(g_os_ctx.current_motor_pos, &g_os_ctx.calibration, coord);
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site) {
    if (!site) return OS_ERR_INVALID_ARGUMENT;
    *site = g_os_ctx.site_info;
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
    *moving = g_os_ctx.is_moving || g_os_ctx.manual_moving;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked) {
    if (!locked) return OS_ERR_INVALID_ARGUMENT;
    *locked = g_os_ctx.gps_locked;
    return OS_ERR_NONE;
}

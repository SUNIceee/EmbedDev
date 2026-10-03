/* Core initialization and main loop execution for OnStep */

#include "onstep_internal.h"
#include <string.h>

os_context_t g_os_ctx;

os_error_t os_init(void) {
    /* 1. Reset all runtime state flags (REQ-FUNC-001) */
    memset(&g_os_ctx, 0, sizeof(os_context_t));
    g_os_ctx.system_state = OS_STATE_INITIALIZING;
    
    g_os_ctx.track_rate = OS_TRACK_RATE_SIDEREAL;
    g_os_ctx.custom_track_factor = 1.0f;
    g_os_ctx.tracking_enabled = true;
    
    g_os_ctx.guide_pulse.rate_fraction = 0.5f;
    g_os_ctx.guide_pulse.active = false;
    
    g_os_ctx.park_position.ra_hours = 0.0f;
    g_os_ctx.park_position.dec_degrees = 90.0f;
    
    g_os_ctx.custom_move_speed_arcsec = 15.0f;
    g_os_ctx.align_residual_calculated = false;
    g_os_ctx.align_star_count = 0;
    g_os_ctx.goto_active = false;
    g_os_ctx.is_parking = false;
    g_os_ctx.manual_active = false;

    /* Default calibration matrix (identity-like mapping) */
    g_os_ctx.calibration.matrix_ra_to_ra = (float)STEPS_PER_ARCSEC;
    g_os_ctx.calibration.matrix_ra_to_dec = 0.0f;
    g_os_ctx.calibration.matrix_dec_to_ra = 0.0f;
    g_os_ctx.calibration.matrix_dec_to_dec = (float)STEPS_PER_ARCSEC;
    g_os_ctx.calibration.offset_ra_arcsec = 0.0f;
    g_os_ctx.calibration.offset_dec_arcsec = 0.0f;
    g_os_ctx.calibration.valid = false;
    
    g_os_ctx.pec_table.valid = false;
    g_os_ctx.pec_enabled = false;

    /* 2. Initialize hardware modules */
    os_hal_nvm_init();
    
    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        os_hal_comm_init(ch);
    }
    
    if (os_hal_motor_init(0) != OS_ERR_NONE || os_hal_motor_init(1) != OS_ERR_NONE) {
        g_os_ctx.system_state = OS_STATE_FAULT;
        return OS_ERR_MOTOR_DRIVER_FAULT;
    }
    
    os_hal_gps_init();
    os_hal_rtc_init();
    os_hal_limit_init();
    os_hal_timer_motor_init();
    
    /* 3. Restore persistent NVM state if valid */
    os_calibration_t loaded_cal;
    if (os_hal_nvm_read(NVM_OFFSET_CALIBRATION, (uint8_t*)&loaded_cal, sizeof(os_calibration_t)) == OS_ERR_NONE) {
        if (loaded_cal.valid) {
            g_os_ctx.calibration = loaded_cal;
        }
    }
    
    os_pec_table_t loaded_pec;
    if (os_hal_nvm_read(NVM_OFFSET_PEC, (uint8_t*)&loaded_pec, sizeof(os_pec_table_t)) == OS_ERR_NONE) {
        if (loaded_pec.valid) {
            g_os_ctx.pec_table = loaded_pec;
        }
    }
    
    /* 4. GPS / RTC fallback */
    os_site_info_t gps_site;
    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid) {
        g_os_ctx.current_site = gps_site;
    } else {
        g_os_ctx.current_site.latitude_degrees = 0.0f;
        g_os_ctx.current_site.longitude_degrees = 0.0f;
        g_os_ctx.current_site.elevation_metres = 0.0f;
        uint32_t rtc_sec = 0;
        os_hal_rtc_read(&rtc_sec);
        g_os_ctx.current_site.utc_epoch_seconds = rtc_sec;
        g_os_ctx.current_site.valid = false;
    }
    
    /* 5. Set motor drivers to active state */
    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);
    
    g_os_ctx.system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

void os_loop_iteration(void) {
    if (g_os_ctx.system_state == OS_STATE_FAULT) {
        os_hal_motor_set_frequency(0, 0);
        os_hal_motor_set_frequency(1, 0);
        return;
    }
    
    /* Check Limit Switches (Fail-safe: check axis 0 and 1) */
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        os_hal_motor_set_frequency(0, 0);
        os_hal_motor_set_frequency(1, 0);
        g_os_ctx.goto_active = false;
        g_os_ctx.is_parking = false;
        g_os_ctx.manual_active = false;
        g_os_ctx.system_state = OS_STATE_FAULT;
        os_hal_buzzer_beep(500, 2);
        return;
    }
    
    /* Process incoming communications on all channels */
    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        int16_t avail = os_hal_comm_available(ch);
        if (avail > 0) {
            static char buffer[OS_MAX_COMMAND_LENGTH + 1];
            static size_t buf_idx = 0;
            
            while (avail-- > 0) {
                char c = os_hal_comm_read(ch);
                if (c == OS_LX200_CMD_PREFIX) {
                    buf_idx = 0;
                    buffer[buf_idx++] = c;
                } else if (c == OS_LX200_CMD_SUFFIX) {
                    if (buf_idx < OS_MAX_COMMAND_LENGTH) {
                        buffer[buf_idx++] = c;
                        buffer[buf_idx] = '\0';
                        char reply[OS_MAX_REPLY_LENGTH];
                        size_t reply_len = 0;
                        if (os_command_parse(buffer, buf_idx, ch, reply, sizeof(reply), &reply_len) == OS_ERR_NONE) {
                            if (reply_len > 0) {
                                os_hal_comm_write(ch, reply, reply_len);
                            }
                        }
                    }
                    buf_idx = 0;
                } else if (buf_idx > 0 && buf_idx < OS_MAX_COMMAND_LENGTH) {
                    buffer[buf_idx++] = c;
                }
            }
        }
    }

    /* Non-blocking GPS update */
    os_site_info_t gps_site;
    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid) {
        g_os_ctx.current_site = gps_site;
    }
    
    /* Process Motion States asynchronously */
    if (g_os_ctx.system_state == OS_STATE_GOTO && g_os_ctx.goto_active) {
        int32_t curr_ra = os_hal_motor_get_position(0);
        int32_t curr_dec = os_hal_motor_get_position(1);
        
        int32_t diff_ra = g_os_ctx.goto_target_steps[0] - curr_ra;
        int32_t diff_dec = g_os_ctx.goto_target_steps[1] - curr_dec;
        
        uint32_t max_freq = (uint32_t)(OS_GOTO_SPEED_MAX_DEG_PER_SEC * STEPS_PER_DEGREE);
        
        bool ra_arrived = false;
        bool dec_arrived = false;
        
        if (diff_ra == 0) {
            os_hal_motor_set_frequency(0, 0);
            ra_arrived = true;
        } else {
            os_hal_motor_set_direction(0, diff_ra > 0);
            os_hal_motor_set_frequency(0, max_freq);
        }
        
        if (diff_dec == 0) {
            os_hal_motor_set_frequency(1, 0);
            dec_arrived = true;
        } else {
            os_hal_motor_set_direction(1, diff_dec > 0);
            os_hal_motor_set_frequency(1, max_freq);
        }
        
        if (ra_arrived && dec_arrived) {
            g_os_ctx.goto_active = false;
            if (g_os_ctx.is_parking) {
                g_os_ctx.is_parking = false;
                g_os_ctx.tracking_enabled = false;
                os_hal_motor_set_frequency(0, 0);
                os_hal_motor_set_frequency(1, 0);
                os_hal_motor_enable(0, false);
                os_hal_motor_enable(1, false);
                g_os_ctx.system_state = OS_STATE_PARKED;
            } else {
                g_os_ctx.system_state = OS_STATE_IDLE_TRACKING;
            }
            os_hal_buzzer_beep(100, 1);
        }
    } else if (g_os_ctx.system_state == OS_STATE_IDLE_TRACKING && g_os_ctx.tracking_enabled) {
        /* Set continuous tracking frequency */
        float base_rate = OS_SIDEREAL_RATE_ARCSEC_PER_SEC;
        if (g_os_ctx.track_rate == OS_TRACK_RATE_LUNAR) {
            base_rate *= OS_LUNAR_RATE_FACTOR;
        } else if (g_os_ctx.track_rate == OS_TRACK_RATE_SOLAR) {
            base_rate *= OS_SOLAR_RATE_FACTOR;
        } else if (g_os_ctx.track_rate == OS_TRACK_RATE_CUSTOM) {
            base_rate *= g_os_ctx.custom_track_factor;
        }
        
        uint32_t freq_hz = (uint32_t)(base_rate * STEPS_PER_ARCSEC);
        os_hal_motor_set_direction(0, true);
        os_hal_motor_set_frequency(0, freq_hz);
        os_hal_motor_set_frequency(1, 0);
    }
}

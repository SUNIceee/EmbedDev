/* Motion control, tracking, parking, guide pulses, and PEC implementation */

#include "onstep_internal.h"
#include <math.h>

void os_coord_eq_to_steps(os_equatorial_coord_t eq, const os_calibration_t *cal, int32_t *ra_steps, int32_t *dec_steps) {
    double ra_arcsec = (double)eq.ra_hours * ARCSEC_PER_HOUR;
    double dec_arcsec = (double)eq.dec_degrees * ARCSEC_PER_DEGREE;
    
    if (cal && cal->valid) {
        *ra_steps = (int32_t)round((double)cal->matrix_ra_to_ra * ra_arcsec + (double)cal->matrix_ra_to_dec * dec_arcsec + (double)cal->offset_ra_arcsec);
        *dec_steps = (int32_t)round((double)cal->matrix_dec_to_ra * ra_arcsec + (double)cal->matrix_dec_to_dec * dec_arcsec + (double)cal->offset_dec_arcsec);
    } else {
        *ra_steps = (int32_t)round(ra_arcsec * STEPS_PER_ARCSEC);
        *dec_steps = (int32_t)round(dec_arcsec * STEPS_PER_ARCSEC);
    }
}

void os_steps_to_eq(int32_t ra_steps, int32_t dec_steps, const os_calibration_t *cal, os_equatorial_coord_t *eq) {
    if (!eq) return;
    
    if (cal && cal->valid) {
        double r_ra = (double)ra_steps - (double)cal->offset_ra_arcsec;
        double r_dec = (double)dec_steps - (double)cal->offset_dec_arcsec;
        
        double det = (double)cal->matrix_ra_to_ra * (double)cal->matrix_dec_to_dec - (double)cal->matrix_ra_to_dec * (double)cal->matrix_dec_to_ra;
        if (fabs(det) > 1e-9) {
            double ra_arcsec = ((double)cal->matrix_dec_to_dec * r_ra - (double)cal->matrix_ra_to_dec * r_dec) / det;
            double dec_arcsec = (-(double)cal->matrix_dec_to_ra * r_ra + (double)cal->matrix_ra_to_ra * r_dec) / det;
            
            double hrs = ra_arcsec / ARCSEC_PER_HOUR;
            while (hrs < 0.0) hrs += 24.0;
            while (hrs >= 24.0) hrs -= 24.0;
            
            eq->ra_hours = (float)hrs;
            eq->dec_degrees = (float)(dec_arcsec / ARCSEC_PER_DEGREE);
            return;
        }
    }
    
    double hrs = (double)ra_steps / (STEPS_PER_ARCSEC * ARCSEC_PER_HOUR);
    while (hrs < 0.0) hrs += 24.0;
    while (hrs >= 24.0) hrs -= 24.0;
    
    eq->ra_hours = (float)hrs;
    eq->dec_degrees = (float)((double)dec_steps / (STEPS_PER_ARCSEC * ARCSEC_PER_DEGREE));
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    /* Parameter validation priority over limit/state checks (REQ 3.1.1) */
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS ||
        target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    
    if (g_os_ctx.system_state == OS_STATE_PARKED || g_os_ctx.system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    
    g_os_ctx.current_target_eq = target;
    os_coord_eq_to_steps(target, &g_os_ctx.calibration, &g_os_ctx.goto_target_steps[0], &g_os_ctx.goto_target_steps[1]);
    
    g_os_ctx.goto_active = true;
    g_os_ctx.is_parking = false;
    g_os_ctx.system_state = OS_STATE_GOTO;
    
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    /* Parameter validation priority over limit/state checks (REQ 3.1.1) */
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < OS_DEC_MIN_DEG || target.altitude_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    
    if (g_os_ctx.system_state == OS_STATE_PARKED || g_os_ctx.system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    
    g_os_ctx.current_target_hor = target;
    g_os_ctx.goto_target_steps[0] = (int32_t)round((double)target.azimuth_degrees * STEPS_PER_DEGREE);
    g_os_ctx.goto_target_steps[1] = (int32_t)round((double)target.altitude_degrees * STEPS_PER_DEGREE);
    
    g_os_ctx.goto_active = true;
    g_os_ctx.is_parking = false;
    g_os_ctx.system_state = OS_STATE_GOTO;
    
    return OS_ERR_NONE;
}

os_error_t os_goto_abort(void) {
    if (g_os_ctx.system_state == OS_STATE_GOTO || g_os_ctx.system_state == OS_STATE_MANUAL_MOTION) {
        g_os_ctx.goto_active = false;
        g_os_ctx.is_parking = false;
        g_os_ctx.manual_active = false;
        os_hal_motor_set_frequency(0, 0);
        os_hal_motor_set_frequency(1, 0);
        g_os_ctx.system_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor) {
    if (rate < OS_TRACK_RATE_SIDEREAL || rate > OS_TRACK_RATE_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (rate == OS_TRACK_RATE_CUSTOM && custom_factor <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_os_ctx.track_rate = rate;
    g_os_ctx.custom_track_factor = custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate, float *custom_factor) {
    if (!rate || !custom_factor) return OS_ERR_INVALID_ARGUMENT;
    *rate = g_os_ctx.track_rate;
    *custom_factor = g_os_ctx.custom_track_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void) {
    g_os_ctx.tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void) {
    g_os_ctx.tracking_enabled = false;
    os_hal_motor_set_frequency(0, 0);
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (duration_ms == 0 || direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    
    g_os_ctx.guide_pulse.active = true;
    g_os_ctx.guide_pulse.duration_ms = duration_ms;
    g_os_ctx.guide_pulse.direction_east = (direction == OS_DIRECTION_EAST);
    g_os_ctx.guide_pulse.direction_north = (direction == OS_DIRECTION_NORTH);
    
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_os_ctx.guide_pulse.rate_fraction = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (!pulse) return OS_ERR_INVALID_ARGUMENT;
    *pulse = g_os_ctx.guide_pulse;
    return OS_ERR_NONE;
}

os_error_t os_park(void) {
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    
    if (g_os_ctx.system_state == OS_STATE_PARKED || g_os_ctx.system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }

    os_coord_eq_to_steps(g_os_ctx.park_position, &g_os_ctx.calibration, 
                        &g_os_ctx.goto_target_steps[0], &g_os_ctx.goto_target_steps[1]);
    
    g_os_ctx.goto_active = true;
    g_os_ctx.is_parking = true;
    g_os_ctx.system_state = OS_STATE_GOTO;
    
    return OS_ERR_NONE;
}

os_error_t os_unpark(void) {
    if (g_os_ctx.system_state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }
    
    os_hal_motor_enable(0, true);
    os_hal_motor_enable(1, true);
    
    for (uint8_t ch = OS_CHANNEL_USB; ch <= OS_CHANNEL_ETHERNET; ch++) {
        os_hal_comm_init(ch);
    }
    
    uint32_t rtc_sec = 0;
    os_hal_rtc_read(&rtc_sec);
    g_os_ctx.current_site.utc_epoch_seconds = rtc_sec;
    
    g_os_ctx.tracking_enabled = true;
    g_os_ctx.system_state = OS_STATE_IDLE_TRACKING;
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

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST ||
        speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    
    if (g_os_ctx.system_state == OS_STATE_PARKED || g_os_ctx.system_state == OS_STATE_FAULT) {
        return OS_ERR_INVALID_STATE;
    }
    
    g_os_ctx.manual_active = true;
    g_os_ctx.manual_direction = direction;
    g_os_ctx.manual_speed_level = speed;
    g_os_ctx.system_state = OS_STATE_MANUAL_MOTION;
    
    uint32_t speed_multiplier = (speed == OS_SPEED_SLOW) ? 1 : (speed == OS_SPEED_MEDIUM) ? 2 : 3;
    uint32_t freq_hz = (uint32_t)(speed_multiplier * STEPS_PER_DEGREE);
    
    uint8_t axis = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) ? 0 : 1;
    bool forward = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_NORTH);
    
    os_hal_motor_set_direction(axis, forward);
    os_hal_motor_set_frequency(axis, freq_hz);
    
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    if (g_os_ctx.system_state == OS_STATE_MANUAL_MOTION) {
        os_hal_motor_set_frequency(0, 0);
        os_hal_motor_set_frequency(1, 0);
        g_os_ctx.manual_active = false;
        g_os_ctx.system_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (arcsec_per_sec <= 0.0f) return OS_ERR_INVALID_ARGUMENT;
    g_os_ctx.custom_move_speed_arcsec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    g_os_ctx.pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (!table) return OS_ERR_INVALID_ARGUMENT;
    g_os_ctx.pec_table = *table;
    g_os_ctx.pec_table.valid = true;
    os_hal_nvm_write(NVM_OFFSET_PEC, (const uint8_t*)&g_os_ctx.pec_table, sizeof(os_pec_table_t));
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (!table) return OS_ERR_INVALID_ARGUMENT;
    *table = g_os_ctx.pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    /* 360 degrees is a valid upper boundary value (REQ-FUNC-010 item 4) */
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    uint16_t idx = ((uint16_t)floor(worm_phase_deg)) % 360;
    g_os_ctx.pec_table.corrections[idx] = error_arcsec;
    g_os_ctx.pec_table.valid = true;
    return OS_ERR_NONE;
}

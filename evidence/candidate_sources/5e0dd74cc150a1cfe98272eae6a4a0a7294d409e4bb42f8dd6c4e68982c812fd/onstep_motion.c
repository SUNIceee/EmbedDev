/*
 * OnStep Motion Control, Guidance & Tracking Implementation
 */

#include "onstep_internal.h"
#include <math.h>

void os_equatorial_to_motor_steps(os_equatorial_coord_t eq, const os_calibration_t *calib, os_motor_position_t *pos) {
    double ra_arcsec = (double)eq.ra_hours * 54000.0;
    double dec_arcsec = (double)eq.dec_degrees * 3600.0;

    if (calib && calib->valid) {
        double corr_ra = (double)calib->matrix_ra_to_ra * ra_arcsec + (double)calib->matrix_ra_to_dec * dec_arcsec + (double)calib->offset_ra_arcsec;
        double corr_dec = (double)calib->matrix_dec_to_ra * ra_arcsec + (double)calib->matrix_dec_to_dec * dec_arcsec + (double)calib->offset_dec_arcsec;
        pos->ra_steps = (int32_t)(corr_ra * OS_STEPS_PER_ARCSEC);
        pos->dec_steps = (int32_t)(corr_dec * OS_STEPS_PER_ARCSEC);
    } else {
        pos->ra_steps = (int32_t)(ra_arcsec * OS_STEPS_PER_ARCSEC);
        pos->dec_steps = (int32_t)(dec_arcsec * OS_STEPS_PER_ARCSEC);
    }
}

void os_motor_steps_to_equatorial(os_motor_position_t pos, const os_calibration_t *calib, os_equatorial_coord_t *eq) {
    double ra_arcsec = (double)pos.ra_steps / OS_STEPS_PER_ARCSEC;
    double dec_arcsec = (double)pos.dec_steps / OS_STEPS_PER_ARCSEC;

    if (calib && calib->valid) {
        ra_arcsec -= (double)calib->offset_ra_arcsec;
        dec_arcsec -= (double)calib->offset_dec_arcsec;

        double det = (double)calib->matrix_ra_to_ra * calib->matrix_dec_to_dec - (double)calib->matrix_ra_to_dec * calib->matrix_dec_to_ra;
        if (fabs(det) > 1e-9) {
            double orig_ra = ((double)calib->matrix_dec_to_dec * ra_arcsec - (double)calib->matrix_ra_to_dec * dec_arcsec) / det;
            double orig_dec = (-(double)calib->matrix_dec_to_ra * ra_arcsec + (double)calib->matrix_ra_to_ra * dec_arcsec) / det;
            ra_arcsec = orig_ra;
            dec_arcsec = orig_dec;
        }
    }

    eq->ra_hours = (float)(ra_arcsec / 54000.0);
    eq->dec_degrees = (float)(dec_arcsec / 3600.0);
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target) {
    if (target.ra_hours < OS_RA_MIN_HOURS || target.ra_hours > OS_RA_MAX_HOURS ||
        target.dec_degrees < OS_DEC_MIN_DEG || target.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    g_os_ctx.target_equatorial = target;
    os_equatorial_to_motor_steps(target, &g_os_ctx.calibration, &g_os_ctx.target_motor_pos);

    g_os_ctx.is_moving = true;
    g_os_ctx.state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target) {
    if (target.azimuth_degrees < 0.0f || target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < OS_DEC_MIN_DEG || target.altitude_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (os_hal_limit_is_triggered(0) || os_hal_limit_is_triggered(1)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    os_equatorial_coord_t eq;
    eq.ra_hours = target.azimuth_degrees / 15.0f;
    eq.dec_degrees = target.altitude_degrees;

    return os_goto_equatorial(eq);
}

os_error_t os_goto_abort(void) {
    g_os_ctx.is_moving = false;
    g_os_ctx.manual_moving = false;
    g_os_ctx.guide_pulse_state.active = false;
    if (g_os_ctx.state != OS_STATE_PARKED && g_os_ctx.state != OS_STATE_FAULT) {
        g_os_ctx.state = OS_STATE_IDLE_TRACKING;
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
    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction, uint32_t duration_ms) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST || duration_ms == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_os_ctx.guide_pulse_state.active = true;
    g_os_ctx.guide_pulse_state.duration_ms = duration_ms;
    g_os_ctx.guide_pulse_state.rate_fraction = g_os_ctx.guide_rate_fraction;
    g_os_ctx.guide_pulse_state.direction_east = (direction == OS_DIRECTION_EAST);
    g_os_ctx.guide_pulse_state.direction_north = (direction == OS_DIRECTION_NORTH);
    g_os_ctx.guide_pulse_state.dec_priority = (direction == OS_DIRECTION_NORTH || direction == OS_DIRECTION_SOUTH);
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction) {
    if (rate_fraction < OS_GUIDE_RATE_MIN || rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_os_ctx.guide_rate_fraction = rate_fraction;
    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse) {
    if (!pulse) return OS_ERR_INVALID_ARGUMENT;
    *pulse = g_os_ctx.guide_pulse_state;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction, os_speed_level_t speed) {
    if (direction < OS_DIRECTION_NORTH || direction > OS_DIRECTION_WEST ||
        speed < OS_SPEED_SLOW || speed > OS_SPEED_CUSTOM) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    uint8_t axis = (direction == OS_DIRECTION_EAST || direction == OS_DIRECTION_WEST) ? 0 : 1;
    if (os_hal_limit_is_triggered(axis)) {
        return OS_ERR_LIMIT_TRIGGERED;
    }
    g_os_ctx.manual_moving = true;
    g_os_ctx.manual_dir = direction;
    g_os_ctx.manual_speed = speed;
    g_os_ctx.state = OS_STATE_MANUAL_MOTION;
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void) {
    g_os_ctx.manual_moving = false;
    if (g_os_ctx.state == OS_STATE_MANUAL_MOTION) {
        g_os_ctx.state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec) {
    if (arcsec_per_sec <= 0.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_os_ctx.custom_speed_arcsec_per_sec = arcsec_per_sec;
    return OS_ERR_NONE;
}

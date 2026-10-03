/* Alignment process and least-squares double-precision calibration solver */

#include "onstep_internal.h"
#include <math.h>
#include <string.h>

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    
    g_os_ctx.align_mode = mode;
    g_os_ctx.align_star_count = 0;
    g_os_ctx.align_in_progress = true;
    g_os_ctx.align_residual_calculated = false;
    g_os_ctx.system_state = OS_STATE_ALIGNMENT;
    
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord, os_motor_position_t motor_pos) {
    if (star_coord.ra_hours < OS_RA_MIN_HOURS || star_coord.ra_hours > OS_RA_MAX_HOURS ||
        star_coord.dec_degrees < OS_DEC_MIN_DEG || star_coord.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    
    if (!g_os_ctx.align_in_progress || g_os_ctx.align_star_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }
    
    uint8_t idx = g_os_ctx.align_star_count;
    g_os_ctx.align_stars_coord[idx] = star_coord;
    g_os_ctx.align_stars_motor[idx] = motor_pos;
    g_os_ctx.align_star_count++;
    
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (!g_os_ctx.align_in_progress) {
        return OS_ERR_INVALID_STATE;
    }

    uint8_t n = g_os_ctx.align_star_count;
    os_align_mode_t mode = g_os_ctx.align_mode;
    
    /* Minimum star count requirements (REQ-FUNC-006 item 6) */
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR ||
        (mode == OS_ALIGN_1STAR && n < 1) ||
        (mode == OS_ALIGN_2STAR && n < 2) ||
        (mode == OS_ALIGN_3STAR && n < 3) ||
        (mode == OS_ALIGN_NSTAR && n < 3)) {
        return OS_ERR_INVALID_STATE;
    }

    if (mode == OS_ALIGN_1STAR) {
        double ra_arc = (double)g_os_ctx.align_stars_coord[0].ra_hours * ARCSEC_PER_HOUR;
        double dec_arc = (double)g_os_ctx.align_stars_coord[0].dec_degrees * ARCSEC_PER_DEGREE;
        
        g_os_ctx.calibration.matrix_ra_to_ra = (float)STEPS_PER_ARCSEC;
        g_os_ctx.calibration.matrix_ra_to_dec = 0.0f;
        g_os_ctx.calibration.matrix_dec_to_ra = 0.0f;
        g_os_ctx.calibration.matrix_dec_to_dec = (float)STEPS_PER_ARCSEC;
        
        g_os_ctx.calibration.offset_ra_arcsec = (float)((double)g_os_ctx.align_stars_motor[0].ra_steps - ra_arc * STEPS_PER_ARCSEC);
        g_os_ctx.calibration.offset_dec_arcsec = (float)((double)g_os_ctx.align_stars_motor[0].dec_steps - dec_arc * STEPS_PER_ARCSEC);
        
        g_os_ctx.align_residual_arcsec = 0.0f;
        g_os_ctx.align_residual_calculated = true;
    } else if (mode == OS_ALIGN_2STAR) {
        double x1 = (double)g_os_ctx.align_stars_coord[0].ra_hours * ARCSEC_PER_HOUR;
        double y1 = (double)g_os_ctx.align_stars_motor[0].ra_steps;
        double x2 = (double)g_os_ctx.align_stars_coord[1].ra_hours * ARCSEC_PER_HOUR;
        double y2 = (double)g_os_ctx.align_stars_motor[1].ra_steps;
        
        if (fabs(x2 - x1) < 1e-6) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        
        double scale_ra = (y2 - y1) / (x2 - x1);
        double off_ra = y1 - scale_ra * x1;
        
        double d1 = (double)g_os_ctx.align_stars_coord[0].dec_degrees * ARCSEC_PER_DEGREE;
        double m1 = (double)g_os_ctx.align_stars_motor[0].dec_steps;
        double d2 = (double)g_os_ctx.align_stars_coord[1].dec_degrees * ARCSEC_PER_DEGREE;
        double m2 = (double)g_os_ctx.align_stars_motor[1].dec_steps;
        
        if (fabs(d2 - d1) < 1e-6) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        
        double scale_dec = (m2 - m1) / (d2 - d1);
        double off_dec = m1 - scale_dec * d1;
        
        g_os_ctx.calibration.matrix_ra_to_ra = (float)scale_ra;
        g_os_ctx.calibration.matrix_ra_to_dec = 0.0f;
        g_os_ctx.calibration.matrix_dec_to_ra = 0.0f;
        g_os_ctx.calibration.matrix_dec_to_dec = (float)scale_dec;
        g_os_ctx.calibration.offset_ra_arcsec = (float)off_ra;
        g_os_ctx.calibration.offset_dec_arcsec = (float)off_dec;
        
        g_os_ctx.align_residual_arcsec = 0.0f;
        g_os_ctx.align_residual_calculated = true;
    } else {
        /* Double-precision least squares solver for 3-star or N-star affine calibration (REQ-FUNC-006 item 4) */
        double S_x2 = 0, S_xy = 0, S_x = 0;
        double S_y2 = 0, S_y = 0;
        double S_m_ra_x = 0, S_m_ra_y = 0, S_m_ra = 0;
        double S_m_dec_x = 0, S_m_dec_y = 0, S_m_dec = 0;
        
        for (uint8_t i = 0; i < n; i++) {
            double x = (double)g_os_ctx.align_stars_coord[i].ra_hours * ARCSEC_PER_HOUR;
            double y = (double)g_os_ctx.align_stars_coord[i].dec_degrees * ARCSEC_PER_DEGREE;
            double m_ra = (double)g_os_ctx.align_stars_motor[i].ra_steps;
            double m_dec = (double)g_os_ctx.align_stars_motor[i].dec_steps;
            
            S_x2 += x * x;
            S_xy += x * y;
            S_x += x;
            S_y2 += y * y;
            S_y += y;
            
            S_m_ra_x += m_ra * x;
            S_m_ra_y += m_ra * y;
            S_m_ra += m_ra;
            
            S_m_dec_x += m_dec * x;
            S_m_dec_y += m_dec * y;
            S_m_dec += m_dec;
        }
        
        double det = S_x2 * (S_y2 * (double)n - S_y * S_y) - S_xy * (S_xy * (double)n - S_y * S_x) + S_x * (S_xy * S_y - S_y2 * S_x);
        
        if (fabs(det) < 1e-9) {
            return OS_ERR_CALIBRATION_FAILED;
        }
        
        /* Solve RA affine parameters */
        double a_ra = (S_m_ra_x * (S_y2 * (double)n - S_y * S_y) - S_xy * (S_m_ra_y * (double)n - S_y * S_m_ra) + S_x * (S_m_ra_y * S_y - S_y2 * S_m_ra)) / det;
        double b_ra = (S_x2 * (S_m_ra_y * (double)n - S_y * S_m_ra) - S_m_ra_x * (S_xy * (double)n - S_y * S_x) + S_x * (S_xy * S_m_ra - S_m_ra_y * S_x)) / det;
        double c_ra = (S_x2 * (S_y2 * S_m_ra - S_m_ra_y * S_y) - S_xy * (S_xy * S_m_ra - S_m_ra_y * S_x) + S_m_ra_x * (S_xy * S_y - S_y2 * S_x)) / det;
        
        /* Solve Dec affine parameters */
        double a_dec = (S_m_dec_x * (S_y2 * (double)n - S_y * S_y) - S_xy * (S_m_dec_y * (double)n - S_y * S_m_dec) + S_x * (S_m_dec_y * S_y - S_y2 * S_m_dec)) / det;
        double b_dec = (S_x2 * (S_m_dec_y * (double)n - S_y * S_m_dec) - S_m_dec_x * (S_xy * (double)n - S_y * S_x) + S_x * (S_xy * S_m_dec - S_m_dec_y * S_x)) / det;
        double c_dec = (S_x2 * (S_y2 * S_m_dec - S_m_dec_y * S_y) - S_xy * (S_xy * S_m_dec - S_m_dec_y * S_x) + S_m_dec_x * (S_xy * S_y - S_y2 * S_x)) / det;
        
        g_os_ctx.calibration.matrix_ra_to_ra = (float)a_ra;
        g_os_ctx.calibration.matrix_ra_to_dec = (float)b_ra;
        g_os_ctx.calibration.offset_ra_arcsec = (float)c_ra;
        
        g_os_ctx.calibration.matrix_dec_to_ra = (float)a_dec;
        g_os_ctx.calibration.matrix_dec_to_dec = (float)b_dec;
        g_os_ctx.calibration.offset_dec_arcsec = (float)c_dec;
        
        if (n == 3) {
            g_os_ctx.align_residual_arcsec = 0.0f;
        } else {
            double sum_sq_err = 0.0;
            for (uint8_t i = 0; i < n; i++) {
                double x = (double)g_os_ctx.align_stars_coord[i].ra_hours * ARCSEC_PER_HOUR;
                double y = (double)g_os_ctx.align_stars_coord[i].dec_degrees * ARCSEC_PER_DEGREE;
                double pred_ra = a_ra * x + b_ra * y + c_ra;
                double pred_dec = a_dec * x + b_dec * y + c_dec;
                
                double err_ra = (pred_ra - (double)g_os_ctx.align_stars_motor[i].ra_steps) / STEPS_PER_ARCSEC;
                double err_dec = (pred_dec - (double)g_os_ctx.align_stars_motor[i].dec_steps) / STEPS_PER_ARCSEC;
                sum_sq_err += err_ra * err_ra + err_dec * err_dec;
            }
            g_os_ctx.align_residual_arcsec = (float)sqrt(sum_sq_err / (double)n);
        }
        g_os_ctx.align_residual_calculated = true;
    }

    g_os_ctx.calibration.valid = true;
    os_hal_nvm_write(NVM_OFFSET_CALIBRATION, (const uint8_t*)&g_os_ctx.calibration, sizeof(os_calibration_t));
    
    g_os_ctx.align_in_progress = false;
    g_os_ctx.system_state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (!residual_arcsec) return OS_ERR_INVALID_ARGUMENT;
    if (!g_os_ctx.align_residual_calculated) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = g_os_ctx.align_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    if (g_os_ctx.align_in_progress || g_os_ctx.system_state == OS_STATE_ALIGNMENT) {
        g_os_ctx.align_in_progress = false;
        g_os_ctx.align_star_count = 0;
        g_os_ctx.system_state = OS_STATE_IDLE_TRACKING;
    }
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (!calib) return OS_ERR_INVALID_ARGUMENT;
    *calib = g_os_ctx.calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    memset(&g_os_ctx.calibration, 0, sizeof(os_calibration_t));
    g_os_ctx.calibration.matrix_ra_to_ra = (float)STEPS_PER_ARCSEC;
    g_os_ctx.calibration.matrix_dec_to_dec = (float)STEPS_PER_ARCSEC;
    g_os_ctx.calibration.valid = false;
    os_hal_nvm_write(NVM_OFFSET_CALIBRATION, (const uint8_t*)&g_os_ctx.calibration, sizeof(os_calibration_t));
    return OS_ERR_NONE;
}

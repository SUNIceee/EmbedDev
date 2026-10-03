/*
 * OnStep Star Alignment, Calibration & PEC Implementation
 */

#include "onstep_internal.h"
#include <math.h>
#include <string.h>

os_error_t os_align_begin(os_align_mode_t mode) {
    if (mode < OS_ALIGN_1STAR || mode > OS_ALIGN_NSTAR) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    g_os_ctx.align_mode = mode;
    g_os_ctx.align_active = true;
    g_os_ctx.align_star_count = 0;
    g_os_ctx.align_residual_computed = false;
    g_os_ctx.align_residual_arcsec = 0.0f;
    g_os_ctx.state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord, os_motor_position_t motor_pos) {
    if (star_coord.ra_hours < OS_RA_MIN_HOURS || star_coord.ra_hours > OS_RA_MAX_HOURS ||
        star_coord.dec_degrees < OS_DEC_MIN_DEG || star_coord.dec_degrees > OS_DEC_MAX_DEG) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (!g_os_ctx.align_active) return OS_ERR_INVALID_STATE;
    if (g_os_ctx.align_star_count >= OS_CALIBRATION_MAX_STARS) return OS_ERR_INVALID_STATE;

    g_os_ctx.align_stars[g_os_ctx.align_star_count].star = star_coord;
    g_os_ctx.align_stars[g_os_ctx.align_star_count].motor = motor_pos;
    g_os_ctx.align_star_count++;
    return OS_ERR_NONE;
}

os_error_t os_align_compute(void) {
    if (!g_os_ctx.align_active) return OS_ERR_INVALID_STATE;

    size_t min_required = 1;
    if (g_os_ctx.align_mode == OS_ALIGN_2STAR) min_required = 2;
    else if (g_os_ctx.align_mode == OS_ALIGN_3STAR || g_os_ctx.align_mode == OS_ALIGN_NSTAR) min_required = 3;

    if (g_os_ctx.align_star_count < min_required) {
        return OS_ERR_INVALID_STATE;
    }

    os_calibration_t calib;
    memset(&calib, 0, sizeof(calib));
    calib.matrix_ra_to_ra = 1.0f;
    calib.matrix_dec_to_dec = 1.0f;
    calib.valid = true;

    size_t n = g_os_ctx.align_star_count;

    if (n == 1) {
        double ra_obs = (double)g_os_ctx.align_stars[0].star.ra_hours * 54000.0;
        double dec_obs = (double)g_os_ctx.align_stars[0].star.dec_degrees * 3600.0;
        double ra_mot = (double)g_os_ctx.align_stars[0].motor.ra_steps / OS_STEPS_PER_ARCSEC;
        double dec_mot = (double)g_os_ctx.align_stars[0].motor.dec_steps / OS_STEPS_PER_ARCSEC;

        calib.offset_ra_arcsec = (float)(ra_mot - ra_obs);
        calib.offset_dec_arcsec = (float)(dec_mot - dec_obs);
        g_os_ctx.align_residual_arcsec = 0.0f;
    } else if (n == 2) {
        double ra_obs1 = (double)g_os_ctx.align_stars[0].star.ra_hours * 54000.0;
        double dec_obs1 = (double)g_os_ctx.align_stars[0].star.dec_degrees * 3600.0;
        double ra_mot1 = (double)g_os_ctx.align_stars[0].motor.ra_steps / OS_STEPS_PER_ARCSEC;
        double dec_mot1 = (double)g_os_ctx.align_stars[0].motor.dec_steps / OS_STEPS_PER_ARCSEC;

        double ra_obs2 = (double)g_os_ctx.align_stars[1].star.ra_hours * 54000.0;
        double dec_obs2 = (double)g_os_ctx.align_stars[1].star.dec_degrees * 3600.0;
        double ra_mot2 = (double)g_os_ctx.align_stars[1].motor.ra_steps / OS_STEPS_PER_ARCSEC;
        double dec_mot2 = (double)g_os_ctx.align_stars[1].motor.dec_steps / OS_STEPS_PER_ARCSEC;

        double d_ra_obs = ra_obs2 - ra_obs1;
        double d_dec_obs = dec_obs2 - dec_obs1;
        double d_ra_mot = ra_mot2 - ra_mot1;
        double d_dec_mot = dec_mot2 - dec_mot1;

        if (fabs(d_ra_obs) > 1e-5) calib.matrix_ra_to_ra = (float)(d_ra_mot / d_ra_obs);
        if (fabs(d_dec_obs) > 1e-5) calib.matrix_dec_to_dec = (float)(d_dec_mot / d_dec_obs);

        calib.offset_ra_arcsec = (float)(ra_mot1 - calib.matrix_ra_to_ra * ra_obs1);
        calib.offset_dec_arcsec = (float)(dec_mot1 - calib.matrix_dec_to_dec * dec_obs1);
        g_os_ctx.align_residual_arcsec = 0.0f;
    } else {
        double sum_r = 0, sum_d = 0, sum_mr = 0, sum_md = 0;
        for (size_t i = 0; i < n; i++) {
            sum_r += (double)g_os_ctx.align_stars[i].star.ra_hours * 54000.0;
            sum_d += (double)g_os_ctx.align_stars[i].star.dec_degrees * 3600.0;
            sum_mr += (double)g_os_ctx.align_stars[i].motor.ra_steps / OS_STEPS_PER_ARCSEC;
            sum_md += (double)g_os_ctx.align_stars[i].motor.dec_steps / OS_STEPS_PER_ARCSEC;
        }
        double mean_r = sum_r / (double)n;
        double mean_d = sum_d / (double)n;
        double mean_mr = sum_mr / (double)n;
        double mean_md = sum_md / (double)n;

        double s_rr = 0, s_rd = 0, s_dd = 0;
        double s_rmr = 0, s_dmr = 0, s_rmd = 0, s_dmd = 0;

        for (size_t i = 0; i < n; i++) {
            double dr = (double)g_os_ctx.align_stars[i].star.ra_hours * 54000.0 - mean_r;
            double dd = (double)g_os_ctx.align_stars[i].star.dec_degrees * 3600.0 - mean_d;
            double dmr = (double)g_os_ctx.align_stars[i].motor.ra_steps / OS_STEPS_PER_ARCSEC - mean_mr;
            double dmd = (double)g_os_ctx.align_stars[i].motor.dec_steps / OS_STEPS_PER_ARCSEC - mean_md;

            s_rr += dr * dr;
            s_rd += dr * dd;
            s_dd += dd * dd;

            s_rmr += dr * dmr;
            s_dmr += dd * dmr;
            s_rmd += dr * dmd;
            s_dmd += dd * dmd;
        }

        double det = s_rr * s_dd - s_rd * s_rd;
        if (fabs(det) < 1e-9) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        calib.matrix_ra_to_ra = (float)((s_rmr * s_dd - s_dmr * s_rd) / det);
        calib.matrix_ra_to_dec = (float)((s_dmr * s_rr - s_rmr * s_rd) / det);
        calib.matrix_dec_to_ra = (float)((s_rmd * s_dd - s_dmd * s_rd) / det);
        calib.matrix_dec_to_dec = (float)((s_dmd * s_rr - s_rmd * s_rd) / det);

        calib.offset_ra_arcsec = (float)(mean_mr - (calib.matrix_ra_to_ra * mean_r + calib.matrix_ra_to_dec * mean_d));
        calib.offset_dec_arcsec = (float)(mean_md - (calib.matrix_dec_to_ra * mean_r + calib.matrix_dec_to_dec * mean_d));

        if (n == 3) {
            g_os_ctx.align_residual_arcsec = 0.0f;
        } else {
            double sq_err = 0.0;
            for (size_t i = 0; i < n; i++) {
                double r_obs = (double)g_os_ctx.align_stars[i].star.ra_hours * 54000.0;
                double d_obs = (double)g_os_ctx.align_stars[i].star.dec_degrees * 3600.0;
                double r_mot = (double)g_os_ctx.align_stars[i].motor.ra_steps / OS_STEPS_PER_ARCSEC;
                double d_mot = (double)g_os_ctx.align_stars[i].motor.dec_steps / OS_STEPS_PER_ARCSEC;

                double pred_r = calib.matrix_ra_to_ra * r_obs + calib.matrix_ra_to_dec * d_obs + calib.offset_ra_arcsec;
                double pred_d = calib.matrix_dec_to_ra * r_obs + calib.matrix_dec_to_dec * d_obs + calib.offset_dec_arcsec;

                double err_r = r_mot - pred_r;
                double err_d = d_mot - pred_d;
                sq_err += (err_r * err_r + err_d * err_d);
            }
            g_os_ctx.align_residual_arcsec = (float)sqrt(sq_err / (double)(2 * n));
        }
    }

    g_os_ctx.calibration = calib;
    g_os_ctx.align_residual_computed = true;
    g_os_ctx.align_active = false;
    g_os_ctx.state = OS_STATE_IDLE_TRACKING;

    os_hal_nvm_write(OS_NVM_OFFSET_CALIBRATION, (const uint8_t *)&calib, sizeof(calib));

    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec) {
    if (!residual_arcsec) return OS_ERR_INVALID_ARGUMENT;
    if (!g_os_ctx.align_residual_computed && !g_os_ctx.calibration.valid) {
        return OS_ERR_INVALID_STATE;
    }
    *residual_arcsec = g_os_ctx.align_residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void) {
    g_os_ctx.align_active = false;
    g_os_ctx.state = OS_STATE_IDLE_TRACKING;
    return OS_ERR_NONE;
}

os_error_t os_calibration_get(os_calibration_t *calib) {
    if (!calib) return OS_ERR_INVALID_ARGUMENT;
    *calib = g_os_ctx.calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void) {
    memset(&g_os_ctx.calibration, 0, sizeof(g_os_ctx.calibration));
    g_os_ctx.align_residual_computed = false;
    g_os_ctx.align_residual_arcsec = 0.0f;
    os_hal_nvm_write(OS_NVM_OFFSET_CALIBRATION, (const uint8_t *)&g_os_ctx.calibration, sizeof(g_os_ctx.calibration));
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable) {
    g_os_ctx.pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table) {
    if (!table) return OS_ERR_INVALID_ARGUMENT;
    g_os_ctx.pec_table = *table;
    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table) {
    if (!table) return OS_ERR_INVALID_ARGUMENT;
    *table = g_os_ctx.pec_table;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg, int16_t error_arcsec) {
    if (worm_phase_deg < 0.0f || worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    int idx = ((int)worm_phase_deg) % OS_PEC_TABLE_SIZE;
    g_os_ctx.pec_table.corrections[idx] = error_arcsec;
    g_os_ctx.pec_table.valid = true;
    return OS_ERR_NONE;
}

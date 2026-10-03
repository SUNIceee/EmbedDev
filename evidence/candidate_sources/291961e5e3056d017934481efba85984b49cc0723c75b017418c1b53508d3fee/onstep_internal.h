/* Internal system context and definitions for OnStep implementation */

#ifndef ONSTEP_INTERNAL_H
#define ONSTEP_INTERNAL_H

#include "6_generated_code.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NVM_OFFSET_CALIBRATION 0
#define NVM_OFFSET_PEC         256

/* Conversion constants */
#define ARCSEC_PER_DEGREE 3600.0
#define ARCSEC_PER_HOUR   54000.0
#define STEPS_PER_DEGREE  1000.0
#define STEPS_PER_ARCSEC  (STEPS_PER_DEGREE / ARCSEC_PER_DEGREE)

typedef struct {
    os_state_t system_state;
    os_site_info_t current_site;
    
    os_track_rate_t track_rate;
    float custom_track_factor;
    bool tracking_enabled;
    
    os_equatorial_coord_t current_target_eq;
    os_horizontal_coord_t current_target_hor;
    os_equatorial_coord_t park_position;
    
    /* Async Goto target positions in steps */
    int32_t goto_target_steps[2];
    bool goto_active;
    bool is_parking;
    
    /* Manual Motion State */
    bool manual_active;
    os_direction_t manual_direction;
    os_speed_level_t manual_speed_level;
    float custom_move_speed_arcsec;
    
    /* Guide Pulse State */
    os_guide_pulse_t guide_pulse;
    
    /* Alignment / Calibration */
    os_align_mode_t align_mode;
    bool align_in_progress;
    uint8_t align_star_count;
    os_equatorial_coord_t align_stars_coord[OS_CALIBRATION_MAX_STARS];
    os_motor_position_t align_stars_motor[OS_CALIBRATION_MAX_STARS];
    float align_residual_arcsec;
    bool align_residual_calculated;
    os_calibration_t calibration;
    
    /* PEC */
    os_pec_table_t pec_table;
    bool pec_enabled;
} os_context_t;

extern os_context_t g_os_ctx;

void os_coord_eq_to_steps(os_equatorial_coord_t eq, const os_calibration_t *cal, int32_t *ra_steps, int32_t *dec_steps);
void os_steps_to_eq(int32_t ra_steps, int32_t dec_steps, const os_calibration_t *cal, os_equatorial_coord_t *eq);

#endif

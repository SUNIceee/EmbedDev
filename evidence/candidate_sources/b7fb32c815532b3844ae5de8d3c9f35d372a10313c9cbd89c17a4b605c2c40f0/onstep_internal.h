/*
 * OnStep Internal System Definitions & Workspace Context Header
 */

#ifndef ONSTEP_INTERNAL_H
#define ONSTEP_INTERNAL_H

#include "6_generated_code.h"

#define OS_STEPS_PER_DEGREE           36000.0
#define OS_STEPS_PER_ARCSEC           10.0
#define OS_STEPS_PER_RA_HOUR          540000.0

#define OS_NVM_OFFSET_CALIBRATION     0
#define OS_NVM_OFFSET_CONFIG          256

typedef struct {
    os_equatorial_coord_t star;
    os_motor_position_t   motor;
} os_align_star_pair_t;

typedef struct {
    os_state_t            state;
    os_mount_type_t       mount_type;
    
    os_motor_position_t   current_motor_pos;
    os_motor_position_t   target_motor_pos;
    bool                  is_moving;
    
    os_equatorial_coord_t target_equatorial;

    bool                  tracking_enabled;
    os_track_rate_t       track_rate;
    float                 custom_track_factor;

    bool                  manual_moving;
    os_direction_t        manual_dir;
    os_speed_level_t      manual_speed;
    float                 custom_speed_arcsec_per_sec;

    os_guide_pulse_t      guide_pulse_state;
    float                 guide_rate_fraction;

    os_align_mode_t       align_mode;
    bool                  align_active;
    size_t                align_star_count;
    os_align_star_pair_t  align_stars[OS_CALIBRATION_MAX_STARS];
    os_calibration_t      calibration;
    float                 align_residual_arcsec;
    bool                  align_residual_computed;

    os_equatorial_coord_t park_position;

    os_site_info_t        site_info;
    bool                  gps_locked;

    bool                  pec_enabled;
    os_pec_table_t        pec_table;

    char                  rx_buffers[4][OS_MAX_COMMAND_LENGTH];
    size_t                rx_buf_pos[4];

} os_system_context_t;

extern os_system_context_t g_os_ctx;

void os_equatorial_to_motor_steps(os_equatorial_coord_t eq, const os_calibration_t *calib, os_motor_position_t *pos);
void os_motor_steps_to_equatorial(os_motor_position_t pos, const os_calibration_t *calib, os_equatorial_coord_t *eq);

#endif

/* OnStep domain implementation for the frozen C11 API. */

#include "6_generated_code.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OS_AXIS_RA 0u
#define OS_AXIS_DEC 1u
#define OS_AXIS_COUNT 2u
#define OS_CHANNEL_COUNT 4u

#define OS_STEPS_PER_DEGREE 1000.0
#define OS_TRACK_TICK_MS 10u
#define OS_GOTO_MIN_ERROR_STEPS 2
#define OS_CALIBRATION_MAGIC 0x4F534341u
#define OS_PEC_MAGIC 0x4F535045u
#define OS_CONFIG_NVM_OFFSET 512u
#define OS_PEC_NVM_OFFSET 768u

typedef struct {
    float ra_hours;
    float dec_degrees;
    int32_t ra_steps;
    int32_t dec_steps;
} alignment_sample_t;

typedef struct {
    uint32_t magic;
    os_calibration_t calibration;
    float residual_arcsec;
} calibration_record_t;

typedef struct {
    uint32_t magic;
    os_pec_table_t table;
} pec_record_t;

typedef struct {
    os_state_t state;
    os_track_rate_t track_rate;
    float custom_track_factor;
    float guide_rate_fraction;
    float custom_move_arcsec_per_sec;

    bool tracking_enabled;
    bool gps_locked;
    bool rtc_available;
    bool initialized;
    bool goto_active;
    bool manual_active;
    bool parking_active;
    bool guide_active;
    bool pec_enabled;

    os_site_info_t site;
    os_equatorial_coord_t current_coord;
    os_equatorial_coord_t goto_target;
    os_equatorial_coord_t park_target;
    os_motor_position_t goto_steps;

    os_guide_pulse_t guide;
    os_calibration_t calibration;
    os_pec_table_t pec;

    uint32_t guide_remaining_ms;
    float residual_arcsec;

    os_align_mode_t alignment_mode;
    alignment_sample_t samples[OS_CALIBRATION_MAX_STARS];
    uint8_t sample_count;
    bool residual_valid;

    uint32_t loop_counter;
    char command_buffers[OS_CHANNEL_COUNT][OS_MAX_COMMAND_LENGTH + 1u];
    size_t command_lengths[OS_CHANNEL_COUNT];
} onstep_context_t;

static onstep_context_t context;

static bool finite_float(float value)
{
    return isfinite((double)value) != 0;
}

static bool valid_axis(uint8_t axis)
{
    return axis < OS_AXIS_COUNT;
}

static bool valid_channel(uint8_t channel)
{
    return channel < OS_CHANNEL_COUNT;
}

static bool valid_direction(os_direction_t direction)
{
    return direction >= OS_DIRECTION_NORTH &&
           direction <= OS_DIRECTION_WEST;
}

static bool valid_speed(os_speed_level_t speed)
{
    return speed >= OS_SPEED_SLOW && speed <= OS_SPEED_CUSTOM;
}

static bool valid_track_rate(os_track_rate_t rate)
{
    return rate >= OS_TRACK_RATE_SIDEREAL &&
           rate <= OS_TRACK_RATE_CUSTOM;
}

static bool valid_align_mode(os_align_mode_t mode)
{
    return mode >= OS_ALIGN_1STAR && mode <= OS_ALIGN_NSTAR;
}

static bool valid_equatorial(os_equatorial_coord_t target)
{
    return finite_float(target.ra_hours) &&
           finite_float(target.dec_degrees) &&
           target.ra_hours >= OS_RA_MIN_HOURS &&
           target.ra_hours <= OS_RA_MAX_HOURS &&
           target.dec_degrees >= OS_DEC_MIN_DEG &&
           target.dec_degrees <= OS_DEC_MAX_DEG;
}

static int32_t clamp_int64_to_int32(int64_t value)
{
    if (value > INT32_MAX) {
        return INT32_MAX;
    }

    if (value < INT32_MIN) {
        return INT32_MIN;
    }

    return (int32_t)value;
}

static int32_t coord_ra_to_steps(float ra_hours)
{
    double degrees = (double)ra_hours * 15.0;
    return clamp_int64_to_int32((int64_t)llround(
        degrees * OS_STEPS_PER_DEGREE));
}

static int32_t coord_dec_to_steps(float dec_degrees)
{
    return clamp_int64_to_int32((int64_t)llround(
        (double)dec_degrees * OS_STEPS_PER_DEGREE));
}

static float steps_to_ra_hours(int32_t steps)
{
    return (float)(((double)steps / OS_STEPS_PER_DEGREE) / 15.0);
}

static float steps_to_dec_degrees(int32_t steps)
{
    return (float)((double)steps / OS_STEPS_PER_DEGREE);
}

static os_motor_position_t read_motor_position(void)
{
    os_motor_position_t position;

    position.ra_steps = os_hal_motor_get_position(OS_AXIS_RA);
    position.dec_steps = os_hal_motor_get_position(OS_AXIS_DEC);
    return position;
}

static void stop_axis(uint8_t axis)
{
    if (!valid_axis(axis)) {
        return;
    }

    (void)os_hal_motor_set_frequency(axis, 0u);
    (void)os_hal_motor_enable(axis, false);
}

static void stop_all_axes(void)
{
    stop_axis(OS_AXIS_RA);
    stop_axis(OS_AXIS_DEC);
}

static bool any_limit_triggered(void)
{
    return os_hal_limit_is_triggered(OS_AXIS_RA) ||
           os_hal_limit_is_triggered(OS_AXIS_DEC);
}

static void set_axis_motion(uint8_t axis, int direction, uint32_t frequency)
{
    bool forward = direction >= 0;

    if (!valid_axis(axis)) {
        return;
    }

    (void)os_hal_motor_set_direction(axis, forward);
    (void)os_hal_motor_set_frequency(axis, frequency);
    (void)os_hal_motor_enable(axis, frequency != 0u);
}

static float tracking_factor(void)
{
    switch (context.track_rate) {
        case OS_TRACK_RATE_LUNAR:
            return OS_LUNAR_RATE_FACTOR;
        case OS_TRACK_RATE_SOLAR:
            return OS_SOLAR_RATE_FACTOR;
        case OS_TRACK_RATE_CUSTOM:
            return context.custom_track_factor;
        case OS_TRACK_RATE_SIDEREAL:
        default:
            return 1.0f;
    }
}

static uint32_t tracking_frequency(void)
{
    double steps_per_second;

    steps_per_second =
        ((double)OS_SIDEREAL_RATE_ARCSEC_PER_SEC *
         (double)tracking_factor() *
         OS_STEPS_PER_DEGREE) / 3600.0;

    if (steps_per_second < 1.0) {
        return 1u;
    }

    if (steps_per_second > UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)llround(steps_per_second);
}

static void apply_tracking(void)
{
    uint32_t frequency;

    if (!context.tracking_enabled ||
        context.state != OS_STATE_IDLE_TRACKING ||
        context.goto_active ||
        context.manual_active ||
        context.parking_active) {
        return;
    }

    if (os_hal_limit_is_triggered(OS_AXIS_RA)) {
        stop_axis(OS_AXIS_RA);
        context.state = OS_STATE_FAULT;
        return;
    }

    frequency = tracking_frequency();
    set_axis_motion(OS_AXIS_RA, 1, frequency);
    set_axis_motion(OS_AXIS_DEC, 1, 0u);
}

static void apply_guide(void)
{
    uint8_t axis;
    int direction;
    uint32_t base_frequency;
    uint32_t guide_frequency;

    if (!context.guide_active) {
        return;
    }

    axis = context.guide.direction_north ? OS_AXIS_DEC : OS_AXIS_RA;
    direction = 1;

    if (axis == OS_AXIS_RA && !context.guide.direction_east) {
        direction = -1;
    }

    if (axis == OS_AXIS_DEC && !context.guide.direction_north) {
        direction = -1;
    }

    base_frequency = tracking_frequency();
    guide_frequency = (uint32_t)llround(
        (double)base_frequency *
        (double)context.guide.rate_fraction);

    if (guide_frequency == 0u) {
        guide_frequency = 1u;
    }

    set_axis_motion(axis, direction, guide_frequency);
}

static void update_goto(void)
{
    os_motor_position_t position;
    int32_t error_ra;
    int32_t error_dec;
    uint32_t frequency_ra;
    uint32_t frequency_dec;

    if (!context.goto_active) {
        return;
    }

    if (any_limit_triggered()) {
        stop_all_axes();
        context.goto_active = false;
        context.state = OS_STATE_FAULT;
        return;
    }

    position = read_motor_position();

    error_ra = context.goto_steps.ra_steps - position.ra_steps;
    error_dec = context.goto_steps.dec_steps - position.dec_steps;

    if (llabs((long long)error_ra) <= OS_GOTO_MIN_ERROR_STEPS &&
        llabs((long long)error_dec) <= OS_GOTO_MIN_ERROR_STEPS) {
        stop_all_axes();
        context.goto_active = false;

        if (context.parking_active) {
            context.parking_active = false;
            context.state = OS_STATE_PARKED;
            context.tracking_enabled = false;
            (void)os_hal_buzzer_beep(100u, 2u);
        } else {
            context.state = OS_STATE_IDLE_TRACKING;
            context.tracking_enabled = true;
            (void)os_hal_buzzer_beep(100u, 1u);
        }

        return;
    }

    frequency_ra = (uint32_t)(500u +
        (unsigned long long)(llabs((long long)error_ra) > 30000 ?
                             30000 : llabs((long long)error_ra)));

    frequency_dec = (uint32_t)(500u +
        (unsigned long long)(llabs((long long)error_dec) > 30000 ?
                             30000 : llabs((long long)error_dec)));

    if (error_ra == 0) {
        frequency_ra = 0u;
    }

    if (error_dec == 0) {
        frequency_dec = 0u;
    }

    set_axis_motion(OS_AXIS_RA,
                    error_ra >= 0 ? 1 : -1,
                    frequency_ra);

    set_axis_motion(OS_AXIS_DEC,
                    error_dec >= 0 ? 1 : -1,
                    frequency_dec);
}

static void update_manual_motion(void)
{
    if (!context.manual_active) {
        return;
    }

    if (any_limit_triggered()) {
        stop_all_axes();
        context.manual_active = false;
        context.state = OS_STATE_FAULT;
    }
}

static void update_guide_timer(void)
{
    if (!context.guide_active) {
        return;
    }

    if (context.guide_remaining_ms > OS_TRACK_TICK_MS) {
        context.guide_remaining_ms -= OS_TRACK_TICK_MS;
        return;
    }

    context.guide_remaining_ms = 0u;
    context.guide_active = false;
    memset(&context.guide, 0, sizeof(context.guide));
    stop_axis(OS_AXIS_RA);
    stop_axis(OS_AXIS_DEC);
}

static void refresh_current_coordinates(void)
{
    os_motor_position_t position = read_motor_position();

    context.current_coord.ra_hours =
        steps_to_ra_hours(position.ra_steps);
    context.current_coord.dec_degrees =
        steps_to_dec_degrees(position.dec_steps);

    if (context.current_coord.ra_hours < OS_RA_MIN_HOURS) {
        context.current_coord.ra_hours = OS_RA_MIN_HOURS;
    }

    if (context.current_coord.ra_hours > OS_RA_MAX_HOURS) {
        context.current_coord.ra_hours = OS_RA_MAX_HOURS;
    }

    if (context.current_coord.dec_degrees < OS_DEC_MIN_DEG) {
        context.current_coord.dec_degrees = OS_DEC_MIN_DEG;
    }

    if (context.current_coord.dec_degrees > OS_DEC_MAX_DEG) {
        context.current_coord.dec_degrees = OS_DEC_MAX_DEG;
    }
}

static os_error_t persist_calibration(void)
{
    calibration_record_t record;

    memset(&record, 0, sizeof(record));
    record.magic = OS_CALIBRATION_MAGIC;
    record.calibration = context.calibration;
    record.residual_arcsec = context.residual_arcsec;

    if (sizeof(record) > OS_NVM_CALIBRATION_SIZE_BYTES) {
        return OS_ERR_NVM_FAULT;
    }

    return os_hal_nvm_write(0u,
                            (const uint8_t *)&record,
                            (uint16_t)sizeof(record));
}

static void load_calibration(void)
{
    calibration_record_t record;

    memset(&record, 0, sizeof(record));

    if (os_hal_nvm_read(0u,
                        (uint8_t *)&record,
                        (uint16_t)sizeof(record)) != OS_ERR_NONE) {
        return;
    }

    if (record.magic != OS_CALIBRATION_MAGIC ||
        !record.calibration.valid) {
        return;
    }

    context.calibration = record.calibration;
    context.residual_arcsec = record.residual_arcsec;
}

static os_error_t persist_pec(void)
{
    pec_record_t record;

    memset(&record, 0, sizeof(record));
    record.magic = OS_PEC_MAGIC;
    record.table = context.pec;

    return os_hal_nvm_write(OS_PEC_NVM_OFFSET,
                            (const uint8_t *)&record,
                            (uint16_t)sizeof(record));
}

static void load_pec(void)
{
    pec_record_t record;

    memset(&record, 0, sizeof(record));

    if (os_hal_nvm_read(OS_PEC_NVM_OFFSET,
                        (uint8_t *)&record,
                        (uint16_t)sizeof(record)) != OS_ERR_NONE) {
        return;
    }

    if (record.magic == OS_PEC_MAGIC && record.table.valid) {
        context.pec = record.table;
    }
}

static void reset_runtime_state(void)
{
    memset(&context, 0, sizeof(context));

    context.state = OS_STATE_INITIALIZING;
    context.track_rate = OS_TRACK_RATE_SIDEREAL;
    context.custom_track_factor = 1.0f;
    context.guide_rate_fraction = 0.5f;
    context.custom_move_arcsec_per_sec = 60.0f;
    context.site.latitude_degrees = 0.0f;
    context.site.longitude_degrees = 0.0f;
    context.site.elevation_metres = 0.0f;
    context.site.valid = false;

    context.goto_active = false;
    context.manual_active = false;
    context.parking_active = false;
    context.guide_active = false;
    context.residual_valid = false;
    context.sample_count = 0u;
}

os_error_t os_init(void)
{
    uint8_t channel;
    os_error_t error;
    os_site_info_t gps_site;
    uint32_t rtc_time;

    reset_runtime_state();

    error = os_hal_nvm_init();
    if (error != OS_ERR_NONE) {
        context.state = OS_STATE_FAULT;
        return error;
    }

    load_calibration();
    load_pec();

    for (channel = 0u; channel < OS_CHANNEL_COUNT; ++channel) {
        error = os_hal_comm_init(channel);
        if (error != OS_ERR_NONE) {
            context.state = OS_STATE_FAULT;
            return error;
        }
    }

    for (channel = 0u; channel < OS_AXIS_COUNT; ++channel) {
        error = os_hal_motor_init(channel);
        if (error != OS_ERR_NONE) {
            context.state = OS_STATE_FAULT;
            return error;
        }

        (void)os_hal_motor_set_frequency(channel, 0u);
        (void)os_hal_motor_enable(channel, false);
    }

    error = os_hal_gps_init();
    if (error != OS_ERR_NONE) {
        context.gps_locked = false;
    }

    error = os_hal_rtc_init();
    if (error != OS_ERR_NONE) {
        context.rtc_available = false;
    } else {
        context.rtc_available = true;
    }

    error = os_hal_limit_init();
    if (error != OS_ERR_NONE) {
        context.state = OS_STATE_FAULT;
        return error;
    }

    error = os_hal_timer_motor_init();
    if (error != OS_ERR_NONE) {
        context.state = OS_STATE_FAULT;
        return error;
    }

    memset(&gps_site, 0, sizeof(gps_site));
    error = os_hal_gps_poll(&gps_site);

    if (error == OS_ERR_NONE && gps_site.valid) {
        context.site = gps_site;
        context.gps_locked = true;
        (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
    } else if (context.rtc_available &&
               os_hal_rtc_read(&rtc_time) == OS_ERR_NONE) {
        context.site.latitude_degrees = 0.0f;
        context.site.longitude_degrees = 0.0f;
        context.site.elevation_metres = 0.0f;
        context.site.utc_epoch_seconds = rtc_time;
        context.site.valid = true;
        context.gps_locked = false;
    } else {
        context.site.valid = false;
        context.gps_locked = false;
    }

    context.initialized = true;
    context.state = OS_STATE_IDLE_TRACKING;
    context.tracking_enabled = true;
    return OS_ERR_NONE;
}

void os_loop_iteration(void)
{
    uint8_t channel;
    int16_t available;
    char value;
    os_site_info_t gps_site;

    if (!context.initialized) {
        return;
    }

    context.loop_counter++;

    memset(&gps_site, 0, sizeof(gps_site));
    if (os_hal_gps_poll(&gps_site) == OS_ERR_NONE && gps_site.valid) {
        context.site = gps_site;
        context.gps_locked = true;
        (void)os_hal_rtc_set(gps_site.utc_epoch_seconds);
    } else {
        context.gps_locked = false;
    }

    for (channel = 0u; channel < OS_CHANNEL_COUNT; ++channel) {
        available = os_hal_comm_available(channel);

        while (available > 0) {
            value = os_hal_comm_read(channel);
            available--;

            if (context.command_lengths[channel] >= OS_MAX_COMMAND_LENGTH) {
                context.command_lengths[channel] = 0u;
                continue;
            }

            context.command_buffers[channel]
                [context.command_lengths[channel]++] = value;

            if (value == OS_LX200_CMD_SUFFIX) {
                char reply[OS_MAX_REPLY_LENGTH];
                size_t reply_length = 0u;

                (void)os_command_parse(
                    context.command_buffers[channel],
                    context.command_lengths[channel],
                    channel,
                    reply,
                    sizeof(reply),
                    &reply_length);

                if (reply_length > 0u) {
                    (void)os_hal_comm_write(channel,
                                            reply,
                                            reply_length);
                }

                context.command_lengths[channel] = 0u;
            }
        }
    }

    if (any_limit_triggered() &&
        (context.goto_active || context.manual_active ||
         context.tracking_enabled)) {
        stop_all_axes();
        context.goto_active = false;
        context.manual_active = false;
        context.state = OS_STATE_FAULT;
        return;
    }

    refresh_current_coordinates();
    update_guide_timer();
    update_goto();
    update_manual_motion();

    if (!context.goto_active &&
        !context.manual_active &&
        context.state == OS_STATE_IDLE_TRACKING) {
        if (context.guide_active) {
            apply_guide();
        } else {
            apply_tracking();
        }
    }
}

os_error_t os_goto_equatorial(os_equatorial_coord_t target)
{
    os_motor_position_t position;

    if (!valid_equatorial(target)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (any_limit_triggered()) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    if (context.state == OS_STATE_PARKED ||
        context.state == OS_STATE_FAULT ||
        !context.initialized) {
        return OS_ERR_INVALID_STATE;
    }

    context.goto_target = target;
    context.goto_steps.ra_steps = coord_ra_to_steps(target.ra_hours);
    context.goto_steps.dec_steps = coord_dec_to_steps(target.dec_degrees);

    position = read_motor_position();

    if (llabs((long long)context.goto_steps.ra_steps -
             position.ra_steps) <= OS_GOTO_MIN_ERROR_STEPS &&
        llabs((long long)context.goto_steps.dec_steps -
             position.dec_steps) <= OS_GOTO_MIN_ERROR_STEPS) {
        context.current_coord = target;
        context.state = OS_STATE_IDLE_TRACKING;
        context.tracking_enabled = true;
        return OS_ERR_NONE;
    }

    context.tracking_enabled = false;
    context.manual_active = false;
    context.goto_active = true;
    context.state = OS_STATE_GOTO;
    return OS_ERR_NONE;
}

os_error_t os_goto_horizontal(os_horizontal_coord_t target)
{
    os_equatorial_coord_t equivalent;

    if (!finite_float(target.azimuth_degrees) ||
        !finite_float(target.altitude_degrees) ||
        target.azimuth_degrees < 0.0f ||
        target.azimuth_degrees > 360.0f ||
        target.altitude_degrees < -90.0f ||
        target.altitude_degrees > 90.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    equivalent.ra_hours = target.azimuth_degrees / 15.0f;
    equivalent.dec_degrees = target.altitude_degrees;
    return os_goto_equatorial(equivalent);
}

os_error_t os_goto_abort(void)
{
    if (!context.initialized) {
        return OS_ERR_INVALID_STATE;
    }

    if (!context.goto_active) {
        return OS_ERR_INVALID_STATE;
    }

    context.goto_active = false;
    context.parking_active = false;
    stop_all_axes();
    context.state = OS_STATE_IDLE_TRACKING;
    context.tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_tracking_set_rate(os_track_rate_t rate, float custom_factor)
{
    if (!valid_track_rate(rate) || !finite_float(custom_factor)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (rate == OS_TRACK_RATE_CUSTOM &&
        (custom_factor <= 0.0f || custom_factor > 10.0f)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    context.track_rate = rate;
    context.custom_track_factor = custom_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_get_rate(os_track_rate_t *rate,
                                float *custom_factor)
{
    if (rate == NULL || custom_factor == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *rate = context.track_rate;
    *custom_factor = context.custom_track_factor;
    return OS_ERR_NONE;
}

os_error_t os_tracking_enable(void)
{
    if (!context.initialized || context.state == OS_STATE_FAULT ||
        context.state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    context.tracking_enabled = true;

    if (!context.goto_active && !context.manual_active) {
        context.state = OS_STATE_IDLE_TRACKING;
    }

    return OS_ERR_NONE;
}

os_error_t os_tracking_disable(void)
{
    if (!context.initialized) {
        return OS_ERR_INVALID_STATE;
    }

    context.tracking_enabled = false;

    if (!context.goto_active && !context.manual_active) {
        stop_all_axes();
    }

    return OS_ERR_NONE;
}

os_error_t os_guide_pulse(os_direction_t direction,
                          uint32_t duration_ms)
{
    if (!valid_direction(direction) || duration_ms == 0u) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!context.initialized || context.state == OS_STATE_FAULT ||
        context.state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    if (any_limit_triggered()) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    memset(&context.guide, 0, sizeof(context.guide));
    context.guide.active = true;
    context.guide.duration_ms = duration_ms;
    context.guide.rate_fraction = context.guide_rate_fraction;
    context.guide.direction_east = direction == OS_DIRECTION_EAST;
    context.guide.direction_north = direction == OS_DIRECTION_NORTH;
    context.guide.dec_priority =
        direction == OS_DIRECTION_NORTH ||
        direction == OS_DIRECTION_SOUTH;

    context.guide_remaining_ms = duration_ms;
    context.guide_active = true;
    return OS_ERR_NONE;
}

os_error_t os_guide_set_rate(float rate_fraction)
{
    if (!finite_float(rate_fraction) ||
        rate_fraction < OS_GUIDE_RATE_MIN ||
        rate_fraction > OS_GUIDE_RATE_MAX) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    context.guide_rate_fraction = rate_fraction;

    if (context.guide_active) {
        context.guide.rate_fraction = rate_fraction;
    }

    return OS_ERR_NONE;
}

os_error_t os_guide_get_state(os_guide_pulse_t *pulse)
{
    if (pulse == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *pulse = context.guide;
    pulse->active = context.guide_active;
    return OS_ERR_NONE;
}

os_error_t os_align_begin(os_align_mode_t mode)
{
    if (!valid_align_mode(mode)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!context.initialized || context.state == OS_STATE_FAULT ||
        context.state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    stop_all_axes();
    context.goto_active = false;
    context.manual_active = false;
    context.alignment_mode = mode;
    context.sample_count = 0u;
    context.residual_valid = false;
    context.state = OS_STATE_ALIGNMENT;
    return OS_ERR_NONE;
}

os_error_t os_align_accept_star(os_equatorial_coord_t star_coord,
                                os_motor_position_t motor_pos)
{
    alignment_sample_t *sample;

    if (!valid_equatorial(star_coord)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (context.state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    if (context.sample_count >= OS_CALIBRATION_MAX_STARS) {
        return OS_ERR_INVALID_STATE;
    }

    sample = &context.samples[context.sample_count++];
    sample->ra_hours = star_coord.ra_hours;
    sample->dec_degrees = star_coord.dec_degrees;
    sample->ra_steps = motor_pos.ra_steps;
    sample->dec_steps = motor_pos.dec_steps;
    return OS_ERR_NONE;
}

static bool solve_affine_qr(const alignment_sample_t *samples,
                            uint8_t count,
                            bool dec_output,
                            double result[3])
{
    double q[OS_CALIBRATION_MAX_STARS][3];
    double r[3][3];
    double a[OS_CALIBRATION_MAX_STARS][3];
    double b[OS_CALIBRATION_MAX_STARS];
    uint8_t i;
    uint8_t j;
    uint8_t k;
    uint8_t columns = 3u;

    if (samples == NULL ||
        result == NULL ||
        count < 3u ||
        count > OS_CALIBRATION_MAX_STARS) {
        return false;
    }

    memset(q, 0, sizeof(q));
    memset(r, 0, sizeof(r));
    memset(a, 0, sizeof(a));
    memset(b, 0, sizeof(b));
    memset(result, 0, sizeof(double) * 3u);

    for (i = 0u; i < count; ++i) {
        double ra_arcsec = (double)samples[i].ra_hours * 54000.0;
        double dec_arcsec = (double)samples[i].dec_degrees * 3600.0;

        a[i][0] = 1.0;
        a[i][1] = ra_arcsec;
        a[i][2] = dec_arcsec;
        b[i] = dec_output ?
               (double)samples[i].dec_steps :
               (double)samples[i].ra_steps;
    }

    for (j = 0u; j < columns; ++j) {
        double norm = 0.0;

        for (i = 0u; i < count; ++i) {
            norm += a[i][j] * a[i][j];
        }

        norm = sqrt(norm);

        if (norm < 1.0e-12) {
            return false;
        }

        r[j][j] = norm;

        for (i = 0u; i < count; ++i) {
            q[i][j] = a[i][j] / norm;
        }

        for (k = j + 1u; k < columns; ++k) {
            double projection = 0.0;

            for (i = 0u; i < count; ++i) {
                projection += q[i][j] * a[i][k];
            }

            r[j][k] = projection;

            for (i = 0u; i < count; ++i) {
                a[i][k] -= projection * q[i][j];
            }
        }
    }

    for (i = columns; i-- > 0u;) {
        double value = 0.0;

        for (k = 0u; k < count; ++k) {
            value += q[k][i] * b[k];
        }

        for (j = i + 1u; j < columns; ++j) {
            value -= r[i][j] * result[j];
        }

        if (fabs(r[i][i]) < 1.0e-12) {
            return false;
        }

        result[i] = value / r[i][i];
    }

    return true;
}

static double sample_residual(const alignment_sample_t *sample,
                              const os_calibration_t *calibration,
                              bool dec_output)
{
    double ra_arcsec = (double)sample->ra_hours * 54000.0;
    double dec_arcsec = (double)sample->dec_degrees * 3600.0;
    double prediction;

    if (dec_output) {
        prediction = (double)calibration->matrix_ra_to_dec *
                     ra_arcsec +
                     (double)calibration->matrix_dec_to_dec *
                     dec_arcsec +
                     (double)calibration->offset_dec_arcsec;
        return prediction - (double)sample->dec_steps;
    }

    prediction = (double)calibration->matrix_ra_to_ra *
                 ra_arcsec +
                 (double)calibration->matrix_dec_to_ra *
                 dec_arcsec +
                 (double)calibration->offset_ra_arcsec;
    return prediction - (double)sample->ra_steps;
}

os_error_t os_align_compute(void)
{
    uint8_t required;
    double ra_solution[3];
    double dec_solution[3];
    double residual_sum = 0.0;
    uint8_t i;

    if (context.state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    switch (context.alignment_mode) {
        case OS_ALIGN_1STAR:
            required = 1u;
            break;
        case OS_ALIGN_2STAR:
            required = 2u;
            break;
        case OS_ALIGN_3STAR:
        case OS_ALIGN_NSTAR:
            required = 3u;
            break;
        default:
            return OS_ERR_INVALID_ARGUMENT;
    }

    if (context.sample_count < required) {
        return OS_ERR_INVALID_STATE;
    }

    memset(&context.calibration, 0, sizeof(context.calibration));

    if (context.alignment_mode == OS_ALIGN_1STAR) {
        double expected_ra;
        double expected_dec;

        expected_ra = (double)context.samples[0].ra_hours * 54000.0;
        expected_dec = (double)context.samples[0].dec_degrees * 3600.0;

        context.calibration.matrix_ra_to_ra =
            (float)(OS_STEPS_PER_DEGREE / 3600.0);
        context.calibration.matrix_dec_to_dec =
            (float)(OS_STEPS_PER_DEGREE / 3600.0);
        context.calibration.offset_ra_arcsec =
            (float)((double)context.samples[0].ra_steps -
                    expected_ra * (OS_STEPS_PER_DEGREE / 3600.0));
        context.calibration.offset_dec_arcsec =
            (float)((double)context.samples[0].dec_steps -
                    expected_dec * (OS_STEPS_PER_DEGREE / 3600.0));
        context.calibration.valid = true;
        context.residual_arcsec = 0.0f;
    } else {
        if (!solve_affine_qr(context.samples,
                             context.sample_count,
                             false,
                             ra_solution) ||
            !solve_affine_qr(context.samples,
                             context.sample_count,
                             true,
                             dec_solution)) {
            return OS_ERR_CALIBRATION_FAILED;
        }

        context.calibration.offset_ra_arcsec = (float)ra_solution[0];
        context.calibration.matrix_ra_to_ra = (float)ra_solution[1];
        context.calibration.matrix_dec_to_ra = (float)ra_solution[2];

        context.calibration.offset_dec_arcsec = (float)dec_solution[0];
        context.calibration.matrix_ra_to_dec = (float)dec_solution[1];
        context.calibration.matrix_dec_to_dec = (float)dec_solution[2];
        context.calibration.valid = true;

        for (i = 0u; i < context.sample_count; ++i) {
            double ra_error = sample_residual(
                &context.samples[i],
                &context.calibration,
                false);
            double dec_error = sample_residual(
                &context.samples[i],
                &context.calibration,
                true);

            residual_sum += ra_error * ra_error;
            residual_sum += dec_error * dec_error;
        }

        context.residual_arcsec =
            (float)sqrt(residual_sum / (double)(context.sample_count * 2u));

        if (context.sample_count == 3u) {
            context.residual_arcsec = 0.0f;
        }
    }

    context.residual_valid = true;

    if (persist_calibration() != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }

    context.state = OS_STATE_IDLE_TRACKING;
    context.tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_align_get_residual(float *residual_arcsec)
{
    if (residual_arcsec == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!context.residual_valid) {
        return OS_ERR_INVALID_STATE;
    }

    *residual_arcsec = context.residual_arcsec;
    return OS_ERR_NONE;
}

os_error_t os_align_abort(void)
{
    if (context.state != OS_STATE_ALIGNMENT) {
        return OS_ERR_INVALID_STATE;
    }

    context.sample_count = 0u;
    context.residual_valid = false;
    context.state = OS_STATE_IDLE_TRACKING;
    context.tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_park_set_position(os_equatorial_coord_t park_pos)
{
    if (!valid_equatorial(park_pos)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    context.park_target = park_pos;
    return OS_ERR_NONE;
}

os_error_t os_park(void)
{
    os_error_t error;

    if (context.state == OS_STATE_PARKED ||
        context.state == OS_STATE_FAULT ||
        !context.initialized) {
        return OS_ERR_INVALID_STATE;
    }

    if (any_limit_triggered()) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    if (context.park_target.ra_hours == 0.0f &&
        context.park_target.dec_degrees == 0.0f) {
        context.park_target.ra_hours = 0.0f;
        context.park_target.dec_degrees = 90.0f;
    }

    context.parking_active = true;
    context.tracking_enabled = false;

    error = os_goto_equatorial(context.park_target);
    if (error != OS_ERR_NONE) {
        context.parking_active = false;
    }

    return error;
}

os_error_t os_unpark(void)
{
    if (context.state != OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    (void)os_hal_motor_enable(OS_AXIS_RA, true);
    (void)os_hal_motor_enable(OS_AXIS_DEC, true);

    context.state = OS_STATE_IDLE_TRACKING;
    context.tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_move_set_custom_speed(float arcsec_per_sec)
{
    if (!finite_float(arcsec_per_sec) ||
        arcsec_per_sec <= 0.0f ||
        arcsec_per_sec > OS_GOTO_SPEED_MAX_DEG_PER_SEC * 3600.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    context.custom_move_arcsec_per_sec = arcsec_per_sec;
    return OS_ERR_NONE;
}

os_error_t os_move_start(os_direction_t direction,
                         os_speed_level_t speed)
{
    uint32_t frequency;
    int direction_sign;
    uint8_t axis;

    if (!valid_direction(direction) || !valid_speed(speed)) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!context.initialized || context.state == OS_STATE_FAULT ||
        context.state == OS_STATE_PARKED) {
        return OS_ERR_INVALID_STATE;
    }

    if (any_limit_triggered()) {
        return OS_ERR_LIMIT_TRIGGERED;
    }

    switch (speed) {
        case OS_SPEED_SLOW:
            frequency = 100u;
            break;
        case OS_SPEED_MEDIUM:
            frequency = 1000u;
            break;
        case OS_SPEED_FAST:
            frequency = 5000u;
            break;
        case OS_SPEED_CUSTOM:
            frequency = (uint32_t)llround(
                (double)context.custom_move_arcsec_per_sec *
                OS_STEPS_PER_DEGREE / 3600.0);
            if (frequency == 0u) {
                frequency = 1u;
            }
            break;
        default:
            return OS_ERR_INVALID_ARGUMENT;
    }

    axis = (direction == OS_DIRECTION_EAST ||
            direction == OS_DIRECTION_WEST) ?
           OS_AXIS_RA : OS_AXIS_DEC;

    direction_sign = (direction == OS_DIRECTION_EAST ||
                      direction == OS_DIRECTION_NORTH) ? 1 : -1;

    context.tracking_enabled = false;
    context.goto_active = false;
    context.manual_active = true;
    context.state = OS_STATE_MANUAL_MOTION;

    set_axis_motion(axis, direction_sign, frequency);
    return OS_ERR_NONE;
}

os_error_t os_move_stop(void)
{
    if (!context.manual_active) {
        return OS_ERR_INVALID_STATE;
    }

    context.manual_active = false;
    stop_all_axes();
    context.state = OS_STATE_IDLE_TRACKING;
    context.tracking_enabled = true;
    return OS_ERR_NONE;
}

os_error_t os_query_state(os_state_t *state)
{
    if (state == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *state = context.state;
    return OS_ERR_NONE;
}

os_error_t os_query_coordinates(os_equatorial_coord_t *coord)
{
    if (coord == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    refresh_current_coordinates();
    *coord = context.current_coord;
    return OS_ERR_NONE;
}

os_error_t os_query_site(os_site_info_t *site)
{
    if (site == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *site = context.site;
    return OS_ERR_NONE;
}

os_error_t os_query_motor_position(os_motor_position_t *pos)
{
    if (pos == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *pos = read_motor_position();
    return OS_ERR_NONE;
}

os_error_t os_query_firmware_version(uint8_t *major,
                                     uint8_t *minor,
                                     uint8_t *patch)
{
    if (major == NULL || minor == NULL || patch == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *major = OS_FIRMWARE_VERSION_MAJOR;
    *minor = OS_FIRMWARE_VERSION_MINOR;
    *patch = OS_FIRMWARE_VERSION_PATCH;
    return OS_ERR_NONE;
}

os_error_t os_query_is_moving(bool *moving)
{
    if (moving == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *moving = context.goto_active || context.manual_active ||
              context.guide_active;
    return OS_ERR_NONE;
}

os_error_t os_query_gps_locked(bool *locked)
{
    if (locked == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *locked = context.gps_locked;
    return OS_ERR_NONE;
}

os_error_t os_pec_enable(bool enable)
{
    if (!context.initialized) {
        return OS_ERR_INVALID_STATE;
    }

    if (enable && !context.pec.valid) {
        return OS_ERR_INVALID_STATE;
    }

    context.pec_enabled = enable;
    return OS_ERR_NONE;
}

os_error_t os_pec_load_table(const os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    context.pec = *table;
    context.pec.valid = true;

    if (persist_pec() != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }

    return OS_ERR_NONE;
}

os_error_t os_pec_get_table(os_pec_table_t *table)
{
    if (table == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *table = context.pec;
    return OS_ERR_NONE;
}

os_error_t os_pec_record_phase(float worm_phase_deg,
                               int16_t error_arcsec)
{
    int index;

    if (!finite_float(worm_phase_deg) ||
        worm_phase_deg < 0.0f ||
        worm_phase_deg > 360.0f) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    index = (int)floor((double)worm_phase_deg);

    if (index >= OS_PEC_TABLE_SIZE) {
        index = OS_PEC_TABLE_SIZE - 1;
    }

    context.pec.corrections[index] = error_arcsec;
    context.pec.valid = true;

    return persist_pec();
}

os_error_t os_calibration_get(os_calibration_t *calib)
{
    if (calib == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *calib = context.calibration;
    return OS_ERR_NONE;
}

os_error_t os_calibration_clear(void)
{
    calibration_record_t empty_record;

    memset(&context.calibration, 0, sizeof(context.calibration));
    context.residual_arcsec = 0.0f;
    context.residual_valid = false;

    memset(&empty_record, 0, sizeof(empty_record));

    if (os_hal_nvm_write(0u,
                         (const uint8_t *)&empty_record,
                         (uint16_t)sizeof(empty_record)) != OS_ERR_NONE) {
        return OS_ERR_NVM_FAULT;
    }

    return OS_ERR_NONE;
}

static os_error_t reply_text(char *reply,
                             size_t reply_size,
                             size_t *reply_length,
                             const char *text)
{
    size_t length;

    if (reply == NULL || reply_length == NULL || text == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    length = strlen(text);

    if (length >= reply_size || length > OS_MAX_REPLY_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    memcpy(reply, text, length);
    reply[length] = '\0';
    *reply_length = length;
    return OS_ERR_NONE;
}

static os_error_t reply_error(char *reply,
                              size_t reply_size,
                              size_t *reply_length,
                              os_error_t error)
{
    char text[32];

    (void)snprintf(text, sizeof(text), "ERR:%d#", (int)error);
    return reply_text(reply, reply_size, reply_length, text);
}

static bool command_is(const char *command,
                       size_t length,
                       const char *expected)
{
    size_t expected_length = strlen(expected);

    return length == expected_length &&
           memcmp(command, expected, expected_length) == 0;
}

static bool parse_ra_dec(const char *command,
                         size_t length,
                         os_equatorial_coord_t *coord)
{
    char buffer[OS_MAX_COMMAND_LENGTH + 1u];
    char *separator;
    char *end;
    double ra;
    double dec;

    if (coord == NULL) {
        return false;
    }

    if (length >= sizeof(buffer)) {
        return false;
    }

    memcpy(buffer, command, length);
    buffer[length] = '\0';

    separator = strchr(buffer, ',');
    if (separator == NULL) {
        return false;
    }

    *separator = '\0';
    ra = strtod(buffer + 1, &end);

    if (end == buffer + 1 || *end != '\0') {
        return false;
    }

    dec = strtod(separator + 1, &end);

    if (end == separator + 1 || *end != '\0') {
        return false;
    }

    coord->ra_hours = (float)ra;
    coord->dec_degrees = (float)dec;
    return valid_equatorial(*coord);
}

os_error_t os_command_parse(const char *command,
                            size_t length,
                            uint8_t source_channel,
                            char *reply_buffer,
                            size_t reply_buffer_size,
                            size_t *reply_length)
{
    os_error_t error;
    os_equatorial_coord_t coord;
    char response[OS_MAX_REPLY_LENGTH];

    if (command == NULL || reply_buffer == NULL || reply_length == NULL) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    if (!valid_channel(source_channel) ||
        reply_buffer_size == 0u ||
        length == 0u ||
        length > OS_MAX_COMMAND_LENGTH) {
        return OS_ERR_INVALID_ARGUMENT;
    }

    *reply_length = 0u;

    if (command[0] != OS_LX200_CMD_PREFIX ||
        command[length - 1u] != OS_LX200_CMD_SUFFIX) {
        error = OS_ERR_COMMAND_FORMAT;
        (void)reply_error(reply_buffer,
                          reply_buffer_size,
                          reply_length,
                          error);
        return error;
    }

    if (command_is(command, length, ":GR#")) {
        refresh_current_coordinates();
        (void)snprintf(response,
                       sizeof(response),
                       "%02d:%02d:%02d#",
                       (int)context.current_coord.ra_hours,
                       (int)(context.current_coord.ra_hours * 60.0f) % 60,
                       (int)(context.current_coord.ra_hours * 3600.0f) % 60);
        return reply_text(reply_buffer,
                          reply_buffer_size,
                          reply_length,
                          response);
    }

    if (command_is(command, length, ":GD#")) {
        refresh_current_coordinates();
        (void)snprintf(response,
                       sizeof(response),
                       "%+.2f#",
                       (double)context.current_coord.dec_degrees);
        return reply_text(reply_buffer,
                          reply_buffer_size,
                          reply_length,
                          response);
    }

    if (command_is(command, length, ":GVP#")) {
        (void)snprintf(response,
                       sizeof(response),
                       "OnStep %d.%d.%d#",
                       OS_FIRMWARE_VERSION_MAJOR,
                       OS_FIRMWARE_VERSION_MINOR,
                       OS_FIRMWARE_VERSION_PATCH);
        return reply_text(reply_buffer,
                          reply_buffer_size,
                          reply_length,
                          response);
    }

    if (command_is(command, length, ":GSTAT#")) {
        (void)snprintf(response,
                       sizeof(response),
                       "%d#",
                       (int)context.state);
        return reply_text(reply_buffer,
                          reply_buffer_size,
                          reply_length,
                          response);
    }

    if (command_is(command, length, ":MS#")) {
        error = os_goto_equatorial(context.goto_target);
        if (error == OS_ERR_NONE) {
            return reply_text(reply_buffer,
                              reply_buffer_size,
                              reply_length,
                              "0#");
        }

        (void)reply_error(reply_buffer,
                          reply_buffer_size,
                          reply_length,
                          error);
        return error;
    }

    if (command_is(command, length, ":Q#")) {
        if (context.goto_active) {
            error = os_goto_abort();
        } else if (context.manual_active) {
            error = os_move_stop();
        } else {
            error = OS_ERR_NONE;
        }

        if (error == OS_ERR_NONE) {
            return reply_text(reply_buffer,
                              reply_buffer_size,
                              reply_length,
                              "1#");
        }

        (void)reply_error(reply_buffer,
                          reply_buffer_size,
                          reply_length,
                          error);
        return error;
    }

    if (command_is(command, length, ":hP#")) {
        error = os_park();
        if (error == OS_ERR_NONE) {
            return reply_text(reply_buffer,
                              reply_buffer_size,
                              reply_length,
                              "1#");
        }

        (void)reply_error(reply_buffer,
                          reply_buffer_size,
                          reply_length,
                          error);
        return error;
    }

    if (command_is(command, length, ":hO#")) {
        error = os_unpark();
        if (error == OS_ERR_NONE) {
            return reply_text(reply_buffer,
                              reply_buffer_size,
                              reply_length,
                              "1#");
        }

        (void)reply_error(reply_buffer,
                          reply_buffer_size,
                          reply_length,
                          error);
        return error;
    }

    if (command[1] == 'S' &&
        length > 4u &&
        command[2] == 'r') {
        if (!parse_ra_dec(command + 2, length - 3u, &coord)) {
            error = OS_ERR_INVALID_ARGUMENT;
            (void)reply_error(reply_buffer,
                              reply_buffer_size,
                              reply_length,
                              error);
            return error;
        }

        context.goto_target.ra_hours = coord.ra_hours;
        context.goto_target.dec_degrees = coord.dec_degrees;

        return reply_text(reply_buffer,
                          reply_buffer_size,
                          reply_length,
                          "1#");
    }

    if (length == 4u && command[1] == 'M') {
        os_direction_t direction;
        os_speed_level_t speed = OS_SPEED_MEDIUM;

        switch (command[2]) {
            case 'e':
                direction = OS_DIRECTION_EAST;
                break;
            case 'w':
                direction = OS_DIRECTION_WEST;
                break;
            case 'n':
                direction = OS_DIRECTION_NORTH;
                break;
            case 's':
                direction = OS_DIRECTION_SOUTH;
                break;
            default:
                error = OS_ERR_COMMAND_FORMAT;
                (void)reply_error(reply_buffer,
                                  reply_buffer_size,
                                  reply_length,
                                  error);
                return error;
        }

        error = os_move_start(direction, speed);
        if (error == OS_ERR_NONE) {
            return reply_text(reply_buffer,
                              reply_buffer_size,
                              reply_length,
                              "1#");
        }

        (void)reply_error(reply_buffer,
                          reply_buffer_size,
                          reply_length,
                          error);
        return error;
    }

    error = OS_ERR_COMMAND_FORMAT;
    (void)reply_error(reply_buffer,
                      reply_buffer_size,
                      reply_length,
                      error);
    return error;
}

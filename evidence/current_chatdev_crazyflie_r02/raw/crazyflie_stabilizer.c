/* DOCSTRING: Stabilizer task ordering, state compression, and rate supervisor. */
#include "6_generated_code.h"
#include "crazyflie_internal.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

volatile uint32_t g_systemTickMs = 0u;
SensorData g_currentSensors;

static bool s_stabilizerInitialized = false;
static uint32_t s_stabilizerStep = 0u;

static Setpoint s_currentSetpoint;
static Setpoint s_pendingHighLevelSetpoint;
static bool s_hasPendingHighLevelSetpoint = false;
static ControlData s_control;
static MotorPower s_motorPower;

static uint32_t s_lastRateSupervisorTick = 0u;

static void sensorsInitInternal(void) { }
static void stateEstimatorInitInternal(void) { }
static void controllerInitInternal(void) { }
static void powerDistributionInitInternal(void) { }
static void motorsInitInternal(void) { }
static void collisionAvoidanceInitInternal(void) { }
static void sensorsWaitDataReadyInternal(void) { }
static void collisionAvoidanceUpdateSetpointInternal(Setpoint *setpoint)
{
    (void)setpoint;
}

void sensorsAcquireInternal(SensorData *out)
{
    if (out != NULL) {
        *out = g_currentSensors;
    }
}

static uint32_t quatcompress(float qw_, float qx_, float qy_, float qz_)
{
    int32_t sx = (int32_t)lrintf((qx_ * 0.5f + 0.5f) * 1023.0f);
    int32_t sy = (int32_t)lrintf((qy_ * 0.5f + 0.5f) * 1023.0f);
    int32_t sz = (int32_t)lrintf((qz_ * 0.5f + 0.5f) * 1023.0f);

    if (sx < 0) sx = 0;
    if (sx > 1023) sx = 1023;
    if (sy < 0) sy = 0;
    if (sy > 1023) sy = 1023;
    if (sz < 0) sz = 0;
    if (sz > 1023) sz = 1023;

    uint32_t sign = (qw_ < 0.0f) ? 1u : 0u;
    return ((uint32_t)sx) |
           (((uint32_t)sy) << 10u) |
           (((uint32_t)sz) << 20u) |
           (sign << 31u);
}

void stabilizerInit(void)
{
    if (s_stabilizerInitialized) {
        return;
    }

    sensorsInitInternal();
    stateEstimatorInitInternal();
    controllerInitInternal();
    powerDistributionInitInternal();
    motorsInitInternal();
    collisionAvoidanceInitInternal();

    g_systemTickMs = 0u;
    s_stabilizerStep = 0u;
    s_hasPendingHighLevelSetpoint = false;
    memset(&s_currentSetpoint, 0, sizeof(s_currentSetpoint));
    memset(&s_control, 0, sizeof(s_control));
    memset(&s_motorPower, 0, sizeof(s_motorPower));
    memset(&g_currentSensors, 0, sizeof(g_currentSensors));
    memset(&g_estimatedState, 0, sizeof(g_estimatedState));
    memset(&g_activeSetpoint, 0, sizeof(g_activeSetpoint));

    s_stabilizerInitialized = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint)
{
    if (setpoint == NULL) {
        return false;
    }

    s_pendingHighLevelSetpoint = *setpoint;
    s_hasPendingHighLevelSetpoint = true;
    return true;
}

static void setMotorRatiosForStabilizer(const uint32_t motorRatios[4])
{
    if (motorRatios == NULL) {
        return;
    }

    motor.m1req = (uint16_t)motorRatios[0];
    motor.m2req = (uint16_t)motorRatios[1];
    motor.m3req = (uint16_t)motorRatios[2];
    motor.m4req = (uint16_t)motorRatios[3];
}

void stabilizerTask(void)
{
    if (!s_stabilizerInitialized) {
        stabilizerInit();
    }

    if (healthShallWeRunTest()) {
        healthRunTests(&g_currentSensors);
        return;
    }

    if (s_hasPendingHighLevelSetpoint) {
        commanderSetSetpoint(&s_pendingHighLevelSetpoint,
                             COMMANDER_PRIORITY_HIGHLEVEL);
        s_hasPendingHighLevelSetpoint = false;
    }

    sensorsWaitDataReadyInternal();
    sensorsAcquireInternal(&g_currentSensors);
    stateEstimatorStepInternal(s_stabilizerStep, &g_estimatedState, &g_currentSensors);

    commanderGetSetpoint(&s_currentSetpoint);

    supervisorUpdate(s_stabilizerStep);

    collisionAvoidanceUpdateSetpointInternal(&s_currentSetpoint);

    if (!supervisorCanFly()) {
        memset(&s_currentSetpoint, 0, sizeof(s_currentSetpoint));
    } else {
        supervisorOverrideSetpoint(&s_currentSetpoint, supervisorConditionBits,
                                   supervisorState);
    }

    controllerPid(&g_currentSensors, &s_currentSetpoint, &g_estimatedState,
                  &s_control, 0.0f, 0.01f);

    powerDistribution(&s_control, &s_motorPower);

    int32_t motorMix[4] = {
        s_motorPower.m1, s_motorPower.m2, s_motorPower.m3, s_motorPower.m4
    };

    uint32_t motorRatios[4] = {0u, 0u, 0u, 0u};

    if (supervisorAreMotorsAllowedToRun()) {
        PowerCapResult cap = powerDistributionCap(motorMix, 65535, 0);
        (void)cap;
        for (int i = 0; i < 4; ++i) {
            int32_t v = motorMix[i];
            if (v < 0) {
                v = 0;
            }
            if (v > 65535) {
                v = 65535;
            }
            motorRatios[i] = (uint32_t)v;
        }
    }

    setMotorRatiosForStabilizer(motorRatios);

    g_systemTickMs++;
    s_stabilizerStep++;
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output)
{
    if (state == NULL || sensors == NULL || output == NULL) {
        return;
    }

    output->position_mm[0] = (int32_t)lrintf(state->position.x * 1000.0f);
    output->position_mm[1] = (int32_t)lrintf(state->position.y * 1000.0f);
    output->position_mm[2] = (int32_t)lrintf(state->position.z * 1000.0f);

    output->velocity_mms[0] = (int32_t)lrintf(state->velocity.x * 1000.0f);
    output->velocity_mms[1] = (int32_t)lrintf(state->velocity.y * 1000.0f);
    output->velocity_mms[2] = (int32_t)lrintf(state->velocity.z * 1000.0f);

    output->acceleration_mms2[0] = (int32_t)lrintf(sensors->acc.x * 9810.0f);
    output->acceleration_mms2[1] = (int32_t)lrintf(sensors->acc.y * 9810.0f);
    output->acceleration_mms2[2] =
        (int32_t)lrintf((sensors->acc.z + 1.0f) * 9810.0f);

    float degToMillirad = ((float)M_PI / 180.0f) * 1000.0f;
    output->gyro_millirad_s[0] = sensors->gyro.x * degToMillirad;
    output->gyro_millirad_s[1] = -sensors->gyro.y * degToMillirad;
    output->gyro_millirad_s[2] = sensors->gyro.z * degToMillirad;

    output->quatCompressed = quatcompress(state->attitudeQuaternion.w,
                                          state->attitudeQuaternion.x,
                                          state->attitudeQuaternion.y,
                                          state->attitudeQuaternion.z);
}

bool rateSupervisorValidate(uint32_t measuredRate)
{
    return measuredRate >= 997u && measuredRate <= 1003u;
}

void rateSupervisorTask(void)
{
    if (g_systemTickMs == 0u) {
        return;
    }

    if ((g_systemTickMs - s_lastRateSupervisorTick) >= 2000u) {
        s_lastRateSupervisorTick = g_systemTickMs;
    }
}

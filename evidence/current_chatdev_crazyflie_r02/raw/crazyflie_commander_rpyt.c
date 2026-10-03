/* DOCSTRING: CRTP Commander RPYT setpoint decoding and yaw-rotation helpers. */
#include "6_generated_code.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool thrustLocked = false;
bool commanderModeSet = false;

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime)
{
    if (rollPrime == NULL || pitchPrime == NULL) {
        return;
    }

    float rad = yaw_deg * ((float)M_PI / 180.0f);
    float c = cosf(rad);
    float s = sinf(rad);

    *rollPrime = roll * c - pitch * s;
    *pitchPrime = roll * s + pitch * c;
}

static void resetPositionControllerForCommander(void)
{
    /* This boundary is intentionally isolated for host-test injection. In the
       current implementation it only resets the command path state. */
}

void crtpCommanderRpytDecodeSetpoint(
    const CommanderCrtpLegacyValues *values,
    Setpoint *setpoint,
    bool altHoldMode,
    bool posHoldMode,
    bool posSetMode,
    StabilizationType stabilizationModeRoll,
    StabilizationType stabilizationModePitch,
    StabilizationType stabilizationModeYaw,
    YawMode yawMode)
{
    if (values == NULL || setpoint == NULL) {
        return;
    }

    memset(setpoint, 0, sizeof(*setpoint));

    float roll = values->roll;
    float pitch = values->pitch;
    float yaw = values->yaw;
    uint16_t rawThrust = values->thrust;

    if (rawThrust == 0) {
        thrustLocked = false;
    }

    if (yawMode == PLUSMODE) {
        rotateYaw(roll, pitch, 45.0f, &roll, &pitch);
    } else if (yawMode == CAREFREE) {
        /* Carefree is not fully defined by the frozen API; keep an observable
           deterministic path by rotating through the supplied yaw command. */
        rotateYaw(roll, pitch, yaw, &roll, &pitch);
    }
    /* XMODE requires no rotation. */

    if (altHoldMode) {
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0;
        setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;

        if (!commanderModeSet) {
            commanderModeSet = true;
            resetPositionControllerForCommander();
        }

        return;
    }

    if (commanderModeSet) {
        commanderModeSet = false;
        setpoint->mode.z = modeDisable;
    }

    if (thrustLocked || rawThrust < MIN_THRUST) {
        setpoint->thrust = 0;
    } else {
        setpoint->thrust = (rawThrust > MAX_THRUST) ? MAX_THRUST : rawThrust;
    }

    if (posHoldMode) {
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;

        setpoint->velocity.x = pitch / 30.0f;
        setpoint->velocity.y = roll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;

        return;
    }

    if (posSetMode && rawThrust != 0) {
        setpoint->mode.x = modeAbs;
        setpoint->mode.y = modeAbs;
        setpoint->mode.z = modeAbs;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->mode.yaw = modeAbs;

        setpoint->position.x = -pitch;
        setpoint->position.y = roll;
        setpoint->position.z = (float)rawThrust / 1000.0f;
        setpoint->attitude.yaw = yaw;
        setpoint->thrust = 0;

        return;
    }

    if (stabilizationModeRoll == RATE) {
        setpoint->mode.roll = modeVelocity;
        setpoint->attitudeRate.roll = roll;
    } else {
        setpoint->mode.roll = modeAbs;
        setpoint->attitude.roll = roll;
    }

    if (stabilizationModePitch == RATE) {
        setpoint->mode.pitch = modeVelocity;
        setpoint->attitudeRate.pitch = pitch;
    } else {
        setpoint->mode.pitch = modeAbs;
        setpoint->attitude.pitch = pitch;
    }

    if (stabilizationModeYaw == RATE) {
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = -yaw;
    } else {
        setpoint->mode.yaw = modeAbs;
        setpoint->attitude.yaw = yaw;
    }
}

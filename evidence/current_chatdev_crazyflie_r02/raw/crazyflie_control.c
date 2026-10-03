/* DOCSTRING: Cascaded PID objects, attitude correction, and controllerPid. */
#include "6_generated_code.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PidObject pidRoll = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidPitch = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidYaw = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidRollRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidPitchRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidYawRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

static float gAttitudeUpdateDt = 0.01f;
static bool gAttitudeControllerInitialized = false;
static float gDesiredYaw = 0.0f;
static float gPositionIntegral = 0.0f;

static void pidReset(PidObject *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
    pid->initialized = true;
}

static float pidUpdate(PidObject *pid, float error, float dt)
{
    if (pid == NULL) {
        return 0.0f;
    }

    if (!pid->initialized) {
        pid->integral = 0.0f;
        pid->prevError = error;
        pid->output = 0.0f;
        pid->initialized = true;
    }

    float p = pid->kp * error;
    pid->integral += pid->ki * error * dt;

    float d = 0.0f;
    if (dt > 1e-6f) {
        d = pid->kd * (error - pid->prevError) / dt;
    }

    pid->prevError = error;
    pid->output = p + pid->integral + d + pid->kff * error;
    return pid->output;
}

static float quaternionToYaw(float qw_, float qx_, float qy_, float qz_)
{
    return atan2f(2.0f * (qw_ * qz_ + qx_ * qy_),
                  1.0f - 2.0f * (qy_ * qy_ + qz_ * qz_)) *
           (180.0f / (float)M_PI);
}

void attitudeControllerInit(float updateDt)
{
    if (gAttitudeControllerInitialized) {
        return;
    }

    gAttitudeUpdateDt = (updateDt > 0.0f) ? updateDt : 0.01f;

    pidReset(&pidRoll);
    pidReset(&pidPitch);
    pidReset(&pidYaw);
    pidReset(&pidRollRate);
    pidReset(&pidPitchRate);
    pidReset(&pidYawRate);

    pidRoll.kp = 0.0f;
    pidRoll.ki = 0.0f;
    pidRoll.kd = 0.0f;
    pidRoll.kff = 0.0f;

    pidPitch.kp = 0.0f;
    pidPitch.ki = 0.0f;
    pidPitch.kd = 0.0f;
    pidPitch.kff = 0.0f;

    pidYaw.kp = 0.0f;
    pidYaw.ki = 0.0f;
    pidYaw.kd = 0.0f;
    pidYaw.kff = 0.0f;

    pidRollRate.kp = 0.0f;
    pidRollRate.ki = 0.0f;
    pidRollRate.kd = 0.0f;
    pidRollRate.kff = 0.0f;

    pidPitchRate.kp = 0.0f;
    pidPitchRate.ki = 0.0f;
    pidPitchRate.kd = 0.0f;
    pidPitchRate.kff = 0.0f;

    pidYawRate.kp = 0.0f;
    pidYawRate.ki = 0.0f;
    pidYawRate.kd = 0.0f;
    pidYawRate.kff = 0.0f;

    gAttitudeControllerInitialized = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired)
{
    float rollOutput = pidUpdate(&pidRollRate, rollDesired - rollActual,
                                 gAttitudeUpdateDt);
    float pitchOutput = pidUpdate(&pidPitchRate, pitchDesired - pitchActual,
                                  gAttitudeUpdateDt);
    float yawOutput = pidUpdate(&pidYawRate, yawDesired - yawActual,
                                gAttitudeUpdateDt);

    pidRollRate.output = (float)saturateSignedInt16((int32_t)lrintf(rollOutput));
    pidPitchRate.output = (float)saturateSignedInt16((int32_t)lrintf(pitchOutput));
    pidYawRate.output = (float)saturateSignedInt16((int32_t)lrintf(yawOutput));
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired)
{
    (void)pidUpdate(&pidRoll, rollDesired - rollActual, gAttitudeUpdateDt);
    (void)pidUpdate(&pidPitch, pitchDesired - pitchActual, gAttitudeUpdateDt);

    float yawError = capAngle(yawDesired - yawActual);
    (void)pidUpdate(&pidYaw, yawError, gAttitudeUpdateDt);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual)
{
    (void)rollActual;
    (void)pitchActual;
    (void)yawActual;

    pidReset(&pidRoll);
    pidReset(&pidPitch);
    pidReset(&pidYaw);
    pidReset(&pidRollRate);
    pidReset(&pidPitchRate);
    pidReset(&pidYawRate);
}

void attitudeControllerResetRollAttitudePID(float rollActual)
{
    (void)rollActual;
    pidReset(&pidRoll);
    pidReset(&pidRollRate);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual)
{
    (void)pitchActual;
    pidReset(&pidPitch);
    pidReset(&pidPitchRate);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw)
{
    if (roll != NULL) {
        *roll = saturateSignedInt16((int32_t)lrintf(pidRollRate.output));
    }
    if (pitch != NULL) {
        *pitch = saturateSignedInt16((int32_t)lrintf(pidPitchRate.output));
    }
    if (yaw != NULL) {
        *yaw = saturateSignedInt16((int32_t)lrintf(pidYawRate.output));
    }
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state)
{
    if (setpoint == NULL || state == NULL) {
        return 0;
    }

    float error = 0.0f;

    if (setpoint->mode.z == modeVelocity) {
        error = setpoint->velocity.z - state->velocity.z;
    } else if (setpoint->mode.z == modeAbs) {
        error = (setpoint->position.z - state->position.z) * 10.0f;
    }

    static const float kp = 200.0f;
    static const float ki = 20.0f;
    gPositionIntegral += ki * error * 0.01f;

    float output = 40000.0f + kp * error + gPositionIntegral;
    if (output < 0.0f) {
        output = 0.0f;
        gPositionIntegral = 0.0f;
    } else if (output > 60000.0f) {
        output = 60000.0f;
        gPositionIntegral = 60000.0f - 40000.0f - kp * error;
    }

    return (uint16_t)output;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt)
{
    if (sensors == NULL || setpoint == NULL || state == NULL || control == NULL) {
        return;
    }

    memset(control, 0, sizeof(*control));
    float dt = (attitudeUpdateDt > 0.0f) ? attitudeUpdateDt : gAttitudeUpdateDt;

    if (setpoint->thrust == 0) {
        control->thrust = 0;
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        attitudeControllerResetAllPID(state->attitude.roll,
                                      state->attitude.pitch,
                                      state->attitude.yaw);
        gDesiredYaw = state->attitude.yaw;
        gPositionIntegral = 0.0f;
        return;
    }

    /* Yaw desired angle. */
    float desiredYaw = state->attitude.yaw;
    if (setpoint->mode.yaw == modeVelocity) {
        gDesiredYaw += setpoint->attitudeRate.yaw * dt;
        desiredYaw = gDesiredYaw;
    } else if (setpoint->mode.quat == modeAbs) {
        desiredYaw = quaternionToYaw(setpoint->attitudeQuaternion.w,
                                     setpoint->attitudeQuaternion.x,
                                     setpoint->attitudeQuaternion.y,
                                     setpoint->attitudeQuaternion.z);
        gDesiredYaw = desiredYaw;
    } else if (setpoint->mode.yaw == modeAbs) {
        desiredYaw = setpoint->attitude.yaw;
        gDesiredYaw = desiredYaw;
    }

    if (yawMaxDelta != 0.0f) {
        float yawDiff = capAngle(desiredYaw - state->attitude.yaw);
        if (yawDiff > yawMaxDelta) {
            desiredYaw = state->attitude.yaw + yawMaxDelta;
        } else if (yawDiff < -yawMaxDelta) {
            desiredYaw = state->attitude.yaw - yawMaxDelta;
        }
        gDesiredYaw = desiredYaw;
    }

    float rollDesiredRate = 0.0f;
    float pitchDesiredRate = 0.0f;
    float yawDesiredRate = 0.0f;

    /* Roll attitude or direct rate. */
    if (setpoint->mode.roll == modeVelocity) {
        attitudeControllerResetRollAttitudePID(state->attitude.roll);
        rollDesiredRate = setpoint->attitudeRate.roll;
    } else {
        (void)pidUpdate(&pidRoll, setpoint->attitude.roll - state->attitude.roll,
                        dt);
        rollDesiredRate = pidRoll.output;
    }

    /* Pitch attitude or direct rate. Pitch gyro direction is inverted in the
       legacy sensor frame and is handled later by negating sensors->gyro.y. */
    if (setpoint->mode.pitch == modeVelocity) {
        attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
        pitchDesiredRate = setpoint->attitudeRate.pitch;
    } else {
        (void)pidUpdate(&pidPitch,
                        setpoint->attitude.pitch - state->attitude.pitch, dt);
        pitchDesiredRate = pidPitch.output;
    }

    /* Yaw attitude or direct rate. */
    if (setpoint->mode.yaw == modeVelocity) {
        yawDesiredRate = setpoint->attitudeRate.yaw;
    } else {
        float yawError = capAngle(desiredYaw - state->attitude.yaw);
        (void)pidUpdate(&pidYaw, yawError, dt);
        yawDesiredRate = pidYaw.output;
    }

    /* Rate PID uses sensor gyro with pitch inversion. */
    attitudeControllerCorrectRatePID(sensors->gyro.x, rollDesiredRate,
                                     -sensors->gyro.y, pitchDesiredRate,
                                     sensors->gyro.z, yawDesiredRate);

    int16_t rollOut = 0;
    int16_t pitchOut = 0;
    int16_t yawOut = 0;
    attitudeControllerGetActuatorOutput(&rollOut, &pitchOut, &yawOut);

    control->roll = rollOut;
    control->pitch = pitchOut;

    /* Legacy coordinate system negates yaw after actuator mixing output. */
    control->yaw = saturateSignedInt16(-(int32_t)yawOut);

    if (setpoint->mode.z == modeDisable) {
        control->thrust = setpoint->thrust;
    } else {
        control->thrust = positionControllerUpdate(setpoint, state);
    }

    control->controlMode = controlModeLegacy;
}

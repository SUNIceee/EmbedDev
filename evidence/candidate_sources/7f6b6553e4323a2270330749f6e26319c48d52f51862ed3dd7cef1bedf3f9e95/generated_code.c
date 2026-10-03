#include "generated_code.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

/* =========================================================================
 * Static inline helpers used by the implementation only
 * ========================================================================= */

static int32_t clamp_i32(int32_t value, int32_t min, int32_t max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static float clamp_f32(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static float radians(float degreesValue)
{
    return degreesValue * 3.14159265358979323846f / 180.0f;
}

static float degrees(float radiansValue)
{
    return radiansValue * 180.0f / 3.14159265358979323846f;
}

/* =========================================================================
 * Numerical helpers
 * ========================================================================= */

int16_t saturateSignedInt16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32767) {
        return -32767;
    }
    return (int16_t)value;
}

float capAngle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

float invSqrt(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }

    float xhalf = 0.5f * x;
    int32_t i;
    float y;

    memcpy(&i, &x, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));

    y = y * (1.5f - xhalf * y * y);
    return y;
}

/* =========================================================================
 * Sensfusion6
 * ========================================================================= */

float qw = 1.0f;
float qx = 0.0f;
float qy = 0.0f;
float qz = 0.0f;
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;
float baseZacc = 0.0f;
bool calibrated = false;
bool sensfusion6Initialized = false;
float twoKp = 0.8f;
float twoKi = 0.002f;

void sensfusion6Init(void)
{
    if (sensfusion6Initialized) {
        return;
    }

    qw = 1.0f;
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
    baseZacc = 0.0f;
    calibrated = false;
    sensfusion6Initialized = true;
}

bool sensfusion6Test(void)
{
    return sensfusion6Initialized;
}

void estimatedGravityDirection(float *gx, float *gy, float *gz)
{
    if (gx == NULL || gy == NULL || gz == NULL) {
        return;
    }

    *gx = 2.0f * (qx * qz - qy * qw);
    *gy = 2.0f * (qy * qz + qx * qw);
    *gz = 1.0f - 2.0f * (qx * qx + qy * qy);
}

float sensfusion6GetAccZ(float ax, float ay, float az)
{
    float gx, gy, gz;

    estimatedGravityDirection(&gx, &gy, &gz);
    return ax * gx + ay * gy + az * gz;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az)
{
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

void sensfusion6GetQuaternion(float *q0, float *q1, float *q2, float *q3)
{
    if (q0 != NULL) {
        *q0 = qw;
    }
    if (q1 != NULL) {
        *q1 = qx;
    }
    if (q2 != NULL) {
        *q2 = qy;
    }
    if (q3 != NULL) {
        *q3 = qz;
    }
}

void sensfusion6GetEulerRPY(float *roll, float *pitch, float *yaw)
{
    float gx, gy, gz;

    estimatedGravityDirection(&gx, &gy, &gz);

    float clampedGx = clamp_f32(gx, -1.0f, 1.0f);

    if (roll != NULL) {
        *roll = degrees(atan2f(2.0f * (qw * qx + qy * qz),
                               1.0f - 2.0f * (qx * qx + qy * qy)));
    }
    if (pitch != NULL) {
        *pitch = degrees(asinf(clampedGx));
    }
    if (yaw != NULL) {
        *yaw = degrees(atan2f(2.0f * (qw * qz + qx * qy),
                              1.0f - 2.0f * (qy * qy + qz * qz)));
    }
}

static void sensfusion6NormalizeQuaternion(void)
{
    float norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);

    if (norm < 1e-10f) {
        qw = 1.0f;
        qx = 0.0f;
        qy = 0.0f;
        qz = 0.0f;
        return;
    }

    norm = 1.0f / norm;
    qw *= norm;
    qx *= norm;
    qy *= norm;
    qz *= norm;
}

void sensfusion6UpdateQ(float gyroscopeX, float gyroscopeY, float gyroscopeZ,
                        float accX, float accY, float accZ, float dt)
{
    if (!sensfusion6Initialized) {
        sensfusion6Init();
    }

    if (!calibrated && !(fabsf(accX) < 1e-6f && fabsf(accY) < 1e-6f &&
                         fabsf(accZ) < 1e-6f)) {
        float gx, gy, gz;
        estimatedGravityDirection(&gx, &gy, &gz);
        baseZacc = accX * gx + accY * gy + accZ * gz;
        calibrated = true;
    }

    float adjustedGx = gyroscopeX;
    float adjustedGy = gyroscopeY;
    float adjustedGz = gyroscopeZ;

#if CONFIG_IMU_MADGWICK_QUATERNION
    {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;

        recipNorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (recipNorm == 0.0f) {
            recipNorm = 1.0f;
        }
        s0 = qw * recipNorm;
        s1 = qx * recipNorm;
        s2 = qy * recipNorm;
        s3 = qz * recipNorm;

        if (!(fabsf(accX) < 1e-6f && fabsf(accY) < 1e-6f &&
              fabsf(accZ) < 1e-6f)) {
            float _2q0 = 2.0f * s0;
            float _2q1 = 2.0f * s1;
            float _2q2 = 2.0f * s2;
            float _2q3 = 2.0f * s3;
            float _4q0 = 4.0f * s0;
            float _4q1 = 4.0f * s1;
            float _4q2 = 4.0f * s2;
            float _8q1 = 8.0f * s1;
            float _8q2 = 8.0f * s2;
            float q0q0 = s0 * s0;
            float q1q1 = s1 * s1;
            float q2q2 = s2 * s2;
            float q3q3 = s3 * s3;

            float f1 = _2q1 * s3 - _2q0 * s2 - accX;
            float f2 = _2q0 * s1 + _2q2 * s3 - accY;
            float f3 = 1.0f - _2q1 * s1 - _2q2 * s2 - accZ;

            float j11 = -_2q2;
            float j12 = _2q3;
            float j21 = _2q1;
            float j22 = _4q0;
            float j31 = -_4q1;
            float j32 = 0.0f;

            float hx = j11 * f1 + j21 * f2 + j31 * f3;
            float hy = j12 * f1 + j22 * f2 + j32 * f3;

            float norm = sqrtf(hx * hx + hy * hy);
            float s0Dot;
            float s1Dot;
            float s2Dot;
            float s3Dot;

            if (norm < 1e-10f) {
                s0Dot = 0.0f;
                s1Dot = 0.0f;
                s2Dot = 0.0f;
                s3Dot = 0.0f;
            } else {
                float beta = 0.01f;
                hx /= norm;
                hy /= norm;
                s0Dot = -_2q2 * hx + _2q1 * hy;
                s1Dot = _2q3 * hx + _4q0 * hy;
                s2Dot = -_4q1 * hx;
                s3Dot = _2q1 * hx;
            }

            qDot1 = 0.5f * (-s1 * gyroscopeX - s2 * gyroscopeY -
                            s3 * gyroscopeZ) -
                    beta * s0Dot;
            qDot2 = 0.5f * (s0 * gyroscopeX + s2 * gyroscopeZ -
                            s3 * gyroscopeY) -
                    beta * s1Dot;
            qDot3 = 0.5f * (s0 * gyroscopeY - s1 * gyroscopeZ +
                            s3 * gyroscopeX) -
                    beta * s2Dot;
            qDot4 = 0.5f * (s0 * gyroscopeZ + s1 * gyroscopeY -
                            s2 * gyroscopeX) -
                    beta * s3Dot;

            qw += qDot1 * dt;
            qx += qDot2 * dt;
            qy += qDot3 * dt;
            qz += qDot4 * dt;
        } else {
            qDot1 = 0.5f * (-qx * gyroscopeX - qy * gyroscopeY -
                            qz * gyroscopeZ);
            qDot2 = 0.5f * (qw * gyroscopeX + qy * gyroscopeZ -
                            qz * gyroscopeY);
            qDot3 = 0.5f * (qw * gyroscopeY - qx * gyroscopeZ +
                            qz * gyroscopeX);
            qDot4 = 0.5f * (qw * gyroscopeZ + qx * gyroscopeY -
                            qy * gyroscopeX);

            qw += qDot1 * dt;
            qx += qDot2 * dt;
            qy += qDot3 * dt;
            qz += qDot4 * dt;
        }

        sensfusion6NormalizeQuaternion();
        return;
    }
#else
    if (!(fabsf(accX) < 1e-6f && fabsf(accY) < 1e-6f &&
          fabsf(accZ) < 1e-6f)) {
        float recipNorm = invSqrt(accX * accX + accY * accY + accZ * accZ);

        if (recipNorm == 0.0f) {
            recipNorm = 1.0f;
        }

        float ax = accX * recipNorm;
        float ay = accY * recipNorm;
        float az = accZ * recipNorm;

        float halfvx = qx * qz - qy * qw;
        float halfvy = qy * qz + qx * qw;
        float halfvz = qw * qw - 0.5f + qz * qz;

        float halfex = ay * halfvz - az * halfvy;
        float halfey = az * halfvx - ax * halfvz;
        float halfez = ax * halfvy - ay * halfvx;

        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        adjustedGx += twoKp * halfex + integralFBx;
        adjustedGy += twoKp * halfey + integralFBy;
        adjustedGz += twoKp * halfez + integralFBz;
    } else if (twoKi == 0.0f) {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    {
        float qDot1 = 0.5f * (-qx * adjustedGx - qy * adjustedGy -
                              qz * adjustedGz);
        float qDot2 = 0.5f * (qw * adjustedGx + qy * adjustedGz -
                              qz * adjustedGy);
        float qDot3 = 0.5f * (qw * adjustedGy - qx * adjustedGz +
                              qz * adjustedGx);
        float qDot4 = 0.5f * (qw * adjustedGz + qx * adjustedGy -
                              qy * adjustedGx);

        qw += qDot1 * dt;
        qx += qDot2 * dt;
        qy += qDot3 * dt;
        qz += qDot4 * dt;
    }

    sensfusion6NormalizeQuaternion();
#endif
}

/* =========================================================================
 * Power distribution
 * ========================================================================= */

void powerDistributionInit(void)
{
    /* Nothing to allocate; kept for API compatibility. */
}

uint16_t motorForceToPwm(float force)
{
    if (force <= 0.0f) {
        return 0;
    }
    if (force >= 1.0f) {
        return 65535;
    }
    return (uint16_t)(force * 65535.0f);
}

static void powerDistributionLegacy(const Control_t *control,
                                    int32_t *motorPwm)
{
    int32_t r = control->roll / 2;
    int32_t p = control->pitch / 2;

    motorPwm[0] = control->thrust - r + p + control->yaw;
    motorPwm[1] = control->thrust - r - p - control->yaw;
    motorPwm[2] = control->thrust + r - p + control->yaw;
    motorPwm[3] = control->thrust + r + p - control->yaw;
}

static void powerDistributionForceTorque(const Control_t *control,
                                         int32_t *motorPwm)
{
    float arm = 0.707106781f * control->armLength;
    float thrustPart = 0.25f * control->thrustSi;

    float rollPart = 0.0f;
    float pitchPart = 0.0f;
    float yawPart = 0.0f;

    if (fabsf(control->armLength) > 1e-12f) {
        rollPart = 0.25f / arm * control->torqueX;
        pitchPart = 0.25f / arm * control->torqueY;
    }
    if (fabsf(control->thrustToTorque) > 1e-12f) {
        yawPart = 0.25f / control->thrustToTorque * control->torqueZ;
    }

    float f1 = thrustPart - rollPart - pitchPart - yawPart;
    float f2 = thrustPart - rollPart + pitchPart + yawPart;
    float f3 = thrustPart + rollPart + pitchPart - yawPart;
    float f4 = thrustPart + rollPart - pitchPart + yawPart;

    if (f1 < 0.0f) {
        f1 = 0.0f;
    }
    if (f2 < 0.0f) {
        f2 = 0.0f;
    }
    if (f3 < 0.0f) {
        f3 = 0.0f;
    }
    if (f4 < 0.0f) {
        f4 = 0.0f;
    }

    motorPwm[0] = (int32_t)motorForceToPwm(f1);
    motorPwm[1] = (int32_t)motorForceToPwm(f2);
    motorPwm[2] = (int32_t)motorForceToPwm(f3);
    motorPwm[3] = (int32_t)motorForceToPwm(f4);
}

static void powerDistributionForce(const Control_t *control,
                                   int32_t *motorPwm)
{
    for (int i = 0; i < 4; i++) {
        float f = control->normalizedForces[i];

        f = clamp_f32(f, 0.0f, 1.0f);
        motorPwm[i] = (int32_t)(f * 65535.0f);
    }
}

void powerDistribution(const Control_t *control, int32_t *motorPwm)
{
    if (control == NULL || motorPwm == NULL) {
        return;
    }

    switch (control->controlMode) {
    case CONTROL_MODE_FORCE_TORQUE:
        powerDistributionForceTorque(control, motorPwm);
        break;
    case CONTROL_MODE_FORCE:
        powerDistributionForce(control, motorPwm);
        break;
    case CONTROL_MODE_LEGACY:
    default:
        powerDistributionLegacy(control, motorPwm);
        break;
    }
}

bool powerDistributionCap(int32_t *motorPwm, int32_t maxAllowedThrust,
                          int32_t idleThrust)
{
    if (motorPwm == NULL) {
        return false;
    }

    int32_t maxMotor = motorPwm[0];
    bool capped = false;

    for (int i = 1; i < 4; i++) {
        if (motorPwm[i] > maxMotor) {
            maxMotor = motorPwm[i];
        }
    }

    if (maxMotor > maxAllowedThrust) {
        int32_t reduction = maxMotor - maxAllowedThrust;

        for (int i = 0; i < 4; i++) {
            motorPwm[i] -= reduction;
        }
        capped = true;
    }

    for (int i = 0; i < 4; i++) {
        if (motorPwm[i] < idleThrust) {
            motorPwm[i] = idleThrust;
        }
    }

    return capped;
}

float batteryCompensation(float oldThrust, float supplyVoltage)
{
    return oldThrust + 0.01f * (supplyVoltage - oldThrust);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t thrust, float nominalVoltage,
                                        float actualVoltage)
{
    if (actualVoltage <= 0.0f) {
        return thrust;
    }

    float compensated = roundf((float)thrust * nominalVoltage / actualVoltage);

    compensated = clamp_f32(compensated, 0.0f, 65535.0f);
    return (uint16_t)compensated;
}

/* =========================================================================
 * PID and attitude controller
 * ========================================================================= */

PidObject_t pidRoll;
PidObject_t pidPitch;
PidObject_t pidYaw;
PidObject_t pidRollRate;
PidObject_t pidPitchRate;
PidObject_t pidYawRate;
float attitudeUpdateDt = 0.0f;
float yawMaxDelta = 0.0f;
float attitudeDesiredYaw = 0.0f;

static bool attitudeControllerInitialized = false;

void pidInit(PidObject_t *pid, float kp, float ki, float kd,
             float outLimit, float iLimit)
{
    if (pid == NULL) {
        return;
    }

    memset(pid, 0, sizeof(*pid));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->outLimit = outLimit;
    pid->iLimit = iLimit;
    pid->reset = false;
}

void pidReset(PidObject_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
}

float pidUpdate(PidObject_t *pid, float error, float dt)
{
    if (pid == NULL) {
        return 0.0f;
    }

    if (pid->reset) {
        pid->integral = 0.0f;
        pid->prevError = error;
        pid->reset = false;
    }

    float pOut = pid->kp * error;

    pid->integral += pid->ki * error * dt;
    if (pid->iLimit > 0.0f) {
        pid->integral = clamp_f32(pid->integral, -pid->iLimit, pid->iLimit);
    }

    float dOut = 0.0f;
    if (dt > 1e-10f) {
        dOut = pid->kd * (error - pid->prevError) / dt;
    }

    pid->output = pOut + pid->integral + dOut;
    if (pid->outLimit > 0.0f) {
        pid->output = clamp_f32(pid->output, -pid->outLimit, pid->outLimit);
    }

    pid->prevError = error;
    return pid->output;
}

void attitudeControllerInit(void)
{
    if (attitudeControllerInitialized) {
        return;
    }

    pidInit(&pidRoll, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    pidInit(&pidPitch, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    pidInit(&pidYaw, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    pidInit(&pidRollRate, 0.0f, 0.0f, 0.0f, 32767.0f, 0.0f);
    pidInit(&pidPitchRate, 0.0f, 0.0f, 0.0f, 32767.0f, 0.0f);
    pidInit(&pidYawRate, 0.0f, 0.0f, 0.0f, 32767.0f, 0.0f);

    attitudeDesiredYaw = 0.0f;
    attitudeControllerInitialized = true;
}

void attitudeControllerResetAll(const State_t *state)
{
    pidReset(&pidRoll);
    pidReset(&pidPitch);
    pidReset(&pidYaw);
    pidReset(&pidRollRate);
    pidReset(&pidPitchRate);
    pidReset(&pidYawRate);

    if (state != NULL) {
        attitudeDesiredYaw = state->attitude.yaw;
    }
}

static float positionControllerThrustValue = 0.0f;

void positionControllerSetThrust(float thrust)
{
    positionControllerThrustValue = thrust;
}

float positionControllerGetThrust(const State_t *state,
                                  const Setpoint_t *setpoint)
{
    (void)state;
    (void)setpoint;
    return positionControllerThrustValue;
}

static float quaternionYawRadians(float q0, float q1, float q2, float q3)
{
    return atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

void controllerPid(const State_t *state, const Setpoint_t *setpoint,
                   const SensorData_t *sensors, Control_t *control, float dt)
{
    if (state == NULL || setpoint == NULL || sensors == NULL || control == NULL) {
        return;
    }

    if (!attitudeControllerInitialized) {
        attitudeControllerInit();
    }

    float thrust = 0.0f;
    if (setpoint->mode.z == SETPOINT_MODE_DISABLE) {
        thrust = setpoint->thrust;
    } else {
        thrust = positionControllerGetThrust(state, setpoint);
    }

    if (fabsf(thrust) < 1e-6f) {
        attitudeControllerResetAll(state);
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0;
        control->controlMode = CONTROL_MODE_LEGACY;
        attitudeDesiredYaw = state->attitude.yaw;
        return;
    }

    float desiredYaw = attitudeDesiredYaw;

    if (setpoint->mode.yaw == SETPOINT_MODE_VELOCITY) {
        desiredYaw += setpoint->attitude.rateYaw * dt;
    } else if (setpoint->mode.yaw == SETPOINT_MODE_ABS) {
        desiredYaw = setpoint->attitude.yaw;
    } else if (setpoint->mode.yaw == SETPOINT_MODE_QUAT) {
        desiredYaw = degrees(quaternionYawRadians(state->attitude.qw,
                                                  state->attitude.qx,
                                                  state->attitude.qy,
                                                  state->attitude.qz));
    }

    if (fabsf(yawMaxDelta) > 1e-6f) {
        float delta = desiredYaw - state->attitude.yaw;
        delta = clamp_f32(delta, -yawMaxDelta, yawMaxDelta);
        desiredYaw = state->attitude.yaw + delta;
    }

    attitudeDesiredYaw = desiredYaw;

    float desiredRollRate = 0.0f;
    if (setpoint->mode.roll == SETPOINT_MODE_VELOCITY) {
        desiredRollRate = setpoint->attitude.rateRoll;
        pidReset(&pidRoll);
    } else {
        float rollError = setpoint->attitude.roll - state->attitude.roll;
        desiredRollRate = pidUpdate(&pidRoll, rollError, dt);
    }

    float desiredPitchRate = 0.0f;
    if (setpoint->mode.pitch == SETPOINT_MODE_VELOCITY) {
        desiredPitchRate = setpoint->attitude.ratePitch;
        pidReset(&pidPitch);
    } else {
        float pitchError = setpoint->attitude.pitch - state->attitude.pitch;
        desiredPitchRate = pidUpdate(&pidPitch, pitchError, dt);
    }

    float yawError = desiredYaw - state->attitude.yaw;
    float desiredYawRate = pidUpdate(&pidYaw, yawError, dt);

    float rollRateError = desiredRollRate - sensors->gyro.x;
    float pitchRateError = desiredPitchRate + sensors->gyro.y;
    float yawRateError = desiredYawRate - sensors->gyro.z;

    int32_t rollPwm = (int32_t)pidUpdate(&pidRollRate, rollRateError, dt);
    int32_t pitchPwm = (int32_t)pidUpdate(&pidPitchRate, pitchRateError, dt);
    int32_t yawPwm = (int32_t)pidUpdate(&pidYawRate, yawRateError, dt);

    control->roll = (int32_t)saturateSignedInt16(rollPwm);
    control->pitch = (int32_t)saturateSignedInt16(pitchPwm);
    control->yaw = (int32_t)saturateSignedInt16(yawPwm);

    control->yaw = -control->yaw;

    control->thrust = (int32_t)thrust;
    control->controlMode = CONTROL_MODE_LEGACY;
}

/* =========================================================================
 * CRTP Commander RPYT decoding
 * ========================================================================= */

bool thrustLocked = false;
bool crtpCommanderPosSetMode = false;
bool crtpCommanderPosHoldMode = false;
bool crtpCommanderAltHoldMode = false;
bool crtpCommanderPlusMode = false;
bool crtpCommanderCarefreeMode = false;
bool crtpCommanderRateRollPitch = false;
bool crtpCommanderRateYaw = false;
bool crtpCommanderModeSet = false;

void rotateYaw(float *x, float *y, float yawDegrees)
{
    if (x == NULL || y == NULL) {
        return;
    }

    float rad = radians(yawDegrees);
    float c = cosf(rad);
    float s = sinf(rad);
    float oldX = *x;
    float oldY = *y;

    *x = oldX * c - oldY * s;
    *y = oldX * s + oldY * c;
}

static void commanderClearSetpoint(Setpoint_t *setpoint)
{
    if (setpoint == NULL) {
        return;
    }

    memset(setpoint, 0, sizeof(*setpoint));
}

void crtpCommanderRpytDecodeSetpoint(Setpoint_t *setpoint, float roll,
                                      float pitch, float yaw,
                                      uint16_t rawThrust, int activePriority)
{
    if (setpoint == NULL) {
        return;
    }

    if (activePriority == COMMANDER_PRIORITY_DISABLE) {
        if (rawThrust == 0) {
            thrustLocked = false;
        } else {
            thrustLocked = true;
        }
        commanderClearSetpoint(setpoint);
        return;
    }

    float decodedRoll = roll;
    float decodedPitch = pitch;

    if (crtpCommanderPlusMode) {
        rotateYaw(&decodedRoll, &decodedPitch, 45.0f);
    } else if (crtpCommanderCarefreeMode) {
        decodedRoll = 0.0f;
        decodedPitch = 0.0f;
    }

    if (crtpCommanderAltHoldMode) {
        if (!crtpCommanderModeSet) {
            crtpCommanderModeSet = true;
            positionControllerSetThrust(0.0f);
        }

        commanderClearSetpoint(setpoint);
        setpoint->mode.z = SETPOINT_MODE_VELOCITY;
        setpoint->thrust = 0.0f;
        setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
        return;
    }

    if (crtpCommanderModeSet) {
        crtpCommanderModeSet = false;
    }

    if (crtpCommanderPosSetMode && rawThrust != 0) {
        commanderClearSetpoint(setpoint);

        setpoint->mode.x = SETPOINT_MODE_ABS;
        setpoint->mode.y = SETPOINT_MODE_ABS;
        setpoint->mode.z = SETPOINT_MODE_ABS;
        setpoint->mode.roll = SETPOINT_MODE_DISABLE;
        setpoint->mode.pitch = SETPOINT_MODE_DISABLE;
        setpoint->mode.yaw = SETPOINT_MODE_ABS;

        setpoint->position.x = -decodedPitch;
        setpoint->position.y = decodedRoll;
        setpoint->position.z = (float)rawThrust / 1000.0f;
        setpoint->attitude.yaw = yaw;
        setpoint->thrust = 0.0f;
        return;
    }

    if (crtpCommanderPosHoldMode) {
        commanderClearSetpoint(setpoint);

        setpoint->mode.x = SETPOINT_MODE_VELOCITY;
        setpoint->mode.y = SETPOINT_MODE_VELOCITY;
        setpoint->mode.roll = SETPOINT_MODE_DISABLE;
        setpoint->mode.pitch = SETPOINT_MODE_DISABLE;

        setpoint->velocity.x = decodedPitch / 30.0f;
        setpoint->velocity.y = decodedRoll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;

        if (thrustLocked || rawThrust < 1000) {
            setpoint->thrust = 0.0f;
        } else {
            setpoint->thrust = (float)(rawThrust > 60000 ? 60000 : rawThrust);
        }

        if (crtpCommanderRateYaw) {
            setpoint->mode.yaw = SETPOINT_MODE_VELOCITY;
            setpoint->attitude.rateYaw = -yaw;
        } else {
            setpoint->mode.yaw = SETPOINT_MODE_ABS;
            setpoint->attitude.yaw = yaw;
        }
        return;
    }

    commanderClearSetpoint(setpoint);

    if (crtpCommanderRateRollPitch) {
        setpoint->mode.roll = SETPOINT_MODE_VELOCITY;
        setpoint->mode.pitch = SETPOINT_MODE_VELOCITY;
        setpoint->attitude.rateRoll = decodedRoll;
        setpoint->attitude.ratePitch = decodedPitch;
    } else {
        setpoint->mode.roll = SETPOINT_MODE_ABS;
        setpoint->mode.pitch = SETPOINT_MODE_ABS;
        setpoint->attitude.roll = decodedRoll;
        setpoint->attitude.pitch = decodedPitch;
    }

    if (crtpCommanderRateYaw) {
        setpoint->mode.yaw = SETPOINT_MODE_VELOCITY;
        setpoint->attitude.rateYaw = -yaw;
    } else {
        setpoint->mode.yaw = SETPOINT_MODE_ABS;
        setpoint->attitude.yaw = yaw;
    }

    if (thrustLocked || rawThrust < 1000) {
        setpoint->thrust = 0.0f;
    } else {
        setpoint->thrust = (float)(rawThrust > 60000 ? 60000 : rawThrust);
    }
}

/* =========================================================================
 * Supervisor
 * ========================================================================= */

SupervisorState_t supervisorState = SUPERVISOR_STATE_INITIAL;
uint32_t supervisorConditionBits = 0;
bool supervisorArmed = false;
bool supervisorCrashed = false;
bool supervisorTumbled = false;
bool supervisorFreeFalling = false;
bool supervisorIsLocked = false;
bool supervisorSeenFlight = false;
uint32_t supervisorRecentFlightTick = 0;
uint32_t supervisorTick = 0;
SensorData_t supervisorSensorData;
int32_t supervisorMotorRatios[4];
int32_t supervisorMotorRPMs[4];
SupervisorSafetyConfig_t supervisorSafetyConfig = {
    .autoArming = false,
    .tumbleCheckEnabled = true,
    .crashDetectionGs = 0.0f,
    .freeFallThreshold = 0.0f,
    .tiltAccThreshold = -0.9f,
    .invertedAccThreshold = -0.2f,
    .tiltTimeoutMs = 0,
    .invertedTimeoutMs = 0,
    .setpointWarningTimeoutMs = 500,
    .setpointTimeoutTimeoutMs = 2000,
    .preflightTimeoutMs = 0,
    .landingTimeoutMs = 0,
    .rpmMin = 0,
    .rpmMax = 0,
    .rpmNotRespondingThreshold = 0,
    .rpmNotRespondingTimeMs = 0,
    .maxAllowedThrust = 65535,
    .idleThrust = 0,
};

static uint32_t supervisorTumbleTimerMs = 0;
static uint32_t supervisorLastSetpointUpdateTick = 0;
static uint32_t supervisorLastNotificationTick = 0;
static uint32_t supervisorLandingTick = 0;
static uint32_t supervisorPreflightTick = 0;
static uint32_t supervisorSpinupStartTick = 0;
static uint32_t supervisorRpmFailCounterMs = 0;
static bool supervisorCrtpStop = false;
static bool supervisorParamStop = false;
static bool supervisorWatchdogStop = false;

static bool rpmCheckConfigured(void);

static bool supervisorStateAllowsMotors(void)
{
    switch (supervisorState) {
    case SUPERVISOR_STATE_ARMING:
    case SUPERVISOR_STATE_READY_TO_FLY:
    case SUPERVISOR_STATE_FLYING:
    case SUPERVISOR_STATE_WARNING_LEVEL_OUT:
    case SUPERVISOR_STATE_LANDED:
        return true;
    default:
        return false;
    }
}

void supervisorSetSensorData(const SensorData_t *sensors)
{
    if (sensors != NULL) {
        supervisorSensorData = *sensors;
    }
}

void supervisorSetMotorRatios(const int32_t motorRatios[4])
{
    if (motorRatios != NULL) {
        memcpy(supervisorMotorRatios, motorRatios, sizeof(supervisorMotorRatios));
    }
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
    if (motorRPMs != NULL) {
        memcpy(supervisorMotorRPMs, motorRPMs, sizeof(supervisorMotorRPMs));
    }
}

void supervisorConfigureSafety(const SupervisorSafetyConfig_t *config)
{
    if (config != NULL) {
        supervisorSafetyConfig = *config;
    }
}

void supervisorSetTick(uint32_t currentTick)
{
    supervisorTick = currentTick;
}

bool supervisorCanFly(void)
{
    switch (supervisorState) {
    case SUPERVISOR_STATE_READY_TO_FLY:
    case SUPERVISOR_STATE_FLYING:
    case SUPERVISOR_STATE_WARNING_LEVEL_OUT:
    case SUPERVISOR_STATE_LANDED:
        return true;
    default:
        return false;
    }
}

bool supervisorCanArm(void)
{
    return supervisorState == SUPERVISOR_STATE_PRE_FL_CHECKS_PASSED;
}

bool supervisorIsArmed(void)
{
    return supervisorArmed;
}

bool supervisorIsCrashed(void)
{
    return supervisorCrashed;
}

bool supervisorRequestArming(void)
{
    if (supervisorState != SUPERVISOR_STATE_PRE_FL_CHECKS_PASSED) {
        return false;
    }

    if (supervisorArmed && supervisorState == SUPERVISOR_STATE_ARMING) {
        return true;
    }

    supervisorArmed = true;
    supervisorState = SUPERVISOR_STATE_ARMING;
    supervisorSpinupStartTick = supervisorTick;
    supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
    if (supervisorTumbled || supervisorIsTumbledCheck()) {
        return false;
    }

    if (!doRecovery) {
        supervisorCrashed = true;
        supervisorState = SUPERVISOR_STATE_CRASHED;
        supervisorConditionBits |= SUPERVISOR_CB_CRASH;
        return true;
    }

    supervisorCrashed = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASH;
    if (supervisorState == SUPERVISOR_STATE_CRASHED) {
        supervisorState = SUPERVISOR_STATE_LANDED;
    }
    return true;
}

bool supervisorIsFlyingCheck(uint32_t currentTick)
{
    for (int i = 0; i < 4; i++) {
        if (supervisorMotorRatios[i] > supervisorSafetyConfig.idleThrust) {
            supervisorSeenFlight = true;
            supervisorRecentFlightTick = currentTick;
            break;
        }
    }

    if (!supervisorSeenFlight) {
        return false;
    }

    return (currentTick - supervisorRecentFlightTick) < 2000;
}

bool supervisorIsTumbledCheck(void)
{
    if (!supervisorSafetyConfig.tumbleCheckEnabled) {
        supervisorFreeFalling = false;
        supervisorTumbled = false;
        return false;
    }

    float ax = supervisorSensorData.acc.x;
    float ay = supervisorSensorData.acc.y;
    float az = supervisorSensorData.acc.z;

    float accNorm = sqrtf(ax * ax + ay * ay + az * az);
    supervisorLog.accNorm = accNorm;

    if (supervisorSafetyConfig.crashDetectionGs > 0.0f &&
        fabsf(accNorm - 1.0f) > supervisorSafetyConfig.crashDetectionGs) {
        supervisorCrashed = true;
        supervisorConditionBits |= SUPERVISOR_CB_CRASH;
    }

    if (fabsf(ax) < supervisorSafetyConfig.freeFallThreshold &&
        fabsf(ay) < supervisorSafetyConfig.freeFallThreshold &&
        fabsf(az) < supervisorSafetyConfig.freeFallThreshold) {
        supervisorFreeFalling = true;
        supervisorState = SUPERVISOR_STATE_EXCEPT_FREE_FALL;
        supervisorTumbleTimerMs = 0;
        supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
        return false;
    }

    supervisorFreeFalling = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

    bool timeoutReached = false;

    if (az < supervisorSafetyConfig.tiltAccThreshold) {
        uint32_t timeout = supervisorSafetyConfig.tiltTimeoutMs;

        if (az < supervisorSafetyConfig.invertedAccThreshold &&
            supervisorSafetyConfig.invertedTimeoutMs > 0) {
            timeout = supervisorSafetyConfig.invertedTimeoutMs;
        }

        if (timeout == 0 || supervisorTumbleTimerMs < timeout) {
            timeoutReached = true;
        }
    } else {
        supervisorTumbleTimerMs = 0;
    }

    if (timeoutReached) {
        supervisorTumbled = true;
        supervisorState = SUPERVISOR_STATE_TUMBLED;
        supervisorConditionBits |= SUPERVISOR_CB_TUMBLED;
        return true;
    }

    supervisorTumbled = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_TUMBLED;
    return false;
}

bool supervisorIsPreflightTimeout(void)
{
    return (supervisorConditionBits & SUPERVISOR_CB_PREFLIGHT_TIMEOUT) != 0;
}

bool supervisorIsLandingTimeout(void)
{
    return (supervisorConditionBits & SUPERVISOR_CB_LANDING_TIMEOUT) != 0;
}

bool supervisorIsRPMatArmingValid(void)
{
    for (int i = 0; i < 4; i++) {
        int32_t rpm = supervisorMotorRPMs[i];

        if (rpm < supervisorSafetyConfig.rpmMin ||
            rpm > supervisorSafetyConfig.rpmMax) {
            return false;
        }
    }

    return true;
}

static bool rpmCheckConfigured(void)
{
    return supervisorSafetyConfig.rpmNotRespondingTimeMs > 0;
}

void updateAndPopulateConditions(void)
{
    uint32_t currentTick = supervisorTick;
    uint32_t setpointAge = currentTick - supervisorLastSetpointUpdateTick;

    if (setpointAge > supervisorSafetyConfig.setpointTimeoutTimeoutMs) {
        supervisorConditionBits |= SUPERVISOR_CB_SETPOINT_TIMEOUT;
        supervisorConditionBits &= ~SUPERVISOR_CB_SETPOINT_WARNING;
    } else if (setpointAge > supervisorSafetyConfig.setpointWarningTimeoutMs) {
        supervisorConditionBits |= SUPERVISOR_CB_SETPOINT_WARNING;
        supervisorConditionBits &= ~SUPERVISOR_CB_SETPOINT_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_SETPOINT_WARNING;
        supervisorConditionBits &= ~SUPERVISOR_CB_SETPOINT_TIMEOUT;
    }

    if (supervisorCrtpStop) {
        supervisorConditionBits |= SUPERVISOR_CB_CRTP_STOP;
    }
    if (supervisorParamStop) {
        supervisorConditionBits |= SUPERVISOR_CB_PARAM_STOP;
    }
    if (supervisorWatchdogStop) {
        supervisorConditionBits |= SUPERVISOR_CB_WATCHDOG_STOP;
    }

    if (rpmCheckConfigured()) {
        bool rpmFault = true;

        for (int i = 0; i < 4; i++) {
            if (supervisorMotorRPMs[i] >=
                    supervisorSafetyConfig.rpmNotRespondingThreshold) {
                rpmFault = false;
                break;
            }
        }

        if (rpmFault) {
            supervisorRpmFailCounterMs++;
            if (supervisorRpmFailCounterMs >=
                supervisorSafetyConfig.rpmNotRespondingTimeMs) {
                supervisorConditionBits |= SUPERVISOR_CB_MOTOR_FAULT;
            }
        } else {
            supervisorRpmFailCounterMs = 0;
        }
    }
}

void supervisorUpdate(uint32_t stabilizerStep)
{
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) {
        return;
    }

    supervisorTick++;

    if (supervisorArmed && supervisorState == SUPERVISOR_STATE_ARMING) {
        uint32_t elapsed = supervisorTick - supervisorSpinupStartTick;

        if (supervisorIsRPMatArmingValid()) {
            supervisorState = SUPERVISOR_STATE_READY_TO_FLY;
            supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
        } else if (supervisorSafetyConfig.spinupTimeoutPeriod > 0 &&
                   elapsed >= supervisorSafetyConfig.spinupTimeoutPeriod) {
            supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
            supervisorArmed = false;
            supervisorState = SUPERVISOR_STATE_INITIAL;
        }
    }

    if (supervisorState == SUPERVISOR_STATE_READY_TO_FLY ||
        supervisorState == SUPERVISOR_STATE_LANDED) {
        if (supervisorSafetyConfig.preflightTimeoutMs > 0 &&
            supervisorTick - supervisorPreflightTick >=
                supervisorSafetyConfig.preflightTimeoutMs) {
            supervisorConditionBits |= SUPERVISOR_CB_PREFLIGHT_TIMEOUT;
        }
    }

    if (supervisorState == SUPERVISOR_STATE_LANDED &&
        supervisorSafetyConfig.landingTimeoutMs > 0 &&
        supervisorTick - supervisorLandingTick >=
            supervisorSafetyConfig.landingTimeoutMs) {
        supervisorConditionBits |= SUPERVISOR_CB_LANDING_TIMEOUT;
    }

    updateAndPopulateConditions();
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick)
{
    if (lastNotificationTick == 0) {
        return true;
    }

    return (currentTick - lastNotificationTick) <= 1000;
}

void supervisorOverrideSetpoint(Setpoint_t *setpoint, const State_t *state)
{
    (void)state;

    if (setpoint == NULL) {
        return;
    }

    switch (supervisorState) {
    case SUPERVISOR_STATE_WARNING_LEVEL_OUT:
        setpoint->mode.x = SETPOINT_MODE_DISABLE;
        setpoint->mode.y = SETPOINT_MODE_DISABLE;
        setpoint->mode.roll = SETPOINT_MODE_ABS;
        setpoint->mode.pitch = SETPOINT_MODE_ABS;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->mode.yaw = SETPOINT_MODE_VELOCITY;
        setpoint->attitude.rateYaw = 0.0f;
        break;

    case SUPERVISOR_STATE_ARMING:
    case SUPERVISOR_STATE_READY_TO_FLY:
    case SUPERVISOR_STATE_FLYING:
    case SUPERVISOR_STATE_LANDED:
        break;

    default:
        commanderClearSetpoint(setpoint);
        break;
    }
}

bool supervisorAreMotorsAllowedToRun(void)
{
    return supervisorStateAllowsMotors();
}

uint16_t supervisorGetInfoBitfield(void)
{
    uint16_t bits = 0;

    if (supervisorCanArm()) {
        bits |= (1u << 0);
    }
    if (supervisorIsArmed()) {
        bits |= (1u << 1);
    }
    if (supervisorSafetyConfig.autoArming) {
        bits |= (1u << 2);
    }
    if (supervisorCanFly()) {
        bits |= (1u << 3);
    }
    if (supervisorIsFlyingCheck(supervisorTick)) {
        bits |= (1u << 4);
    }
    if (supervisorTumbled) {
        bits |= (1u << 5);
    }
    if (supervisorIsLocked) {
        bits |= (1u << 6);
    }
    if (supervisorIsCrashed()) {
        bits |= (1u << 7);
    }

    return bits;
}

/* =========================================================================
 * Estimator
 * ========================================================================= */

EstimatorFifo_t estimatorFifo;
SensorData_t estimatorLastSensor;

void estimatorInit(void)
{
    memset(&estimatorFifo, 0, sizeof(estimatorFifo));
    memset(&estimatorLastSensor, 0, sizeof(estimatorLastSensor));
}

bool estimatorFifoPush(float value)
{
    if (estimatorFifo.count >= 16) {
        return false;
    }

    estimatorFifo.data[estimatorFifo.tail] = value;
    estimatorFifo.tail = (uint8_t)((estimatorFifo.tail + 1) % 16);
    estimatorFifo.count++;
    return true;
}

bool estimatorFifoPop(float *value)
{
    if (value == NULL || estimatorFifo.count == 0) {
        return false;
    }

    *value = estimatorFifo.data[estimatorFifo.head];
    estimatorFifo.head = (uint8_t)((estimatorFifo.head + 1) % 16);
    estimatorFifo.count--;
    return true;
}

void estimatorComplementary(State_t *state, SensorData_t *sensors,
                            uint32_t step)
{
    float tmp;

    while (estimatorFifoPop(&tmp)) {
        /* Drain old measurements. */
    }

    if (sensors != NULL) {
        estimatorLastSensor = *sensors;
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, step)) {
        sensfusion6UpdateQ(estimatorLastSensor.gyro.x,
                           estimatorLastSensor.gyro.y,
                           estimatorLastSensor.gyro.z,
                           estimatorLastSensor.acc.x,
                           estimatorLastSensor.acc.y,
                           estimatorLastSensor.acc.z,
                           0.004f);

        if (state != NULL) {
            sensfusion6GetEulerRPY(&state->attitude.roll,
                                   &state->attitude.pitch,
                                   &state->attitude.yaw);
            sensfusion6GetQuaternion(&state->attitude.qw,
                                     &state->attitude.qx,
                                     &state->attitude.qy,
                                     &state->attitude.qz);
            state->acc.z = sensfusion6GetAccZWithoutGravity(
                estimatorLastSensor.acc.x,
                estimatorLastSensor.acc.y,
                estimatorLastSensor.acc.z);
            state->velocity.z += state->acc.z * 0.004f;
        }
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, step)) {
        if (state != NULL) {
            state->position.x += state->velocity.x * 0.01f;
            state->position.y += state->velocity.y * 0.01f;
            state->position.z += state->velocity.z * 0.01f;
        }
    }
}

/* =========================================================================
 * Commander arbitration
 * ========================================================================= */

static Setpoint_t commanderActiveSetpoint;
static int commanderActivePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t commanderLastUpdateTick = 0;

void commanderInit(void)
{
    memset(&commanderActiveSetpoint, 0, sizeof(commanderActiveSetpoint));
    commanderActivePriority = COMMANDER_PRIORITY_DISABLE;
    commanderLastUpdateTick = 0;
}

bool commanderSetSetpoint(const Setpoint_t *setpoint, int priority)
{
    if (setpoint == NULL) {
        return false;
    }

    if (priority == COMMANDER_PRIORITY_DISABLE) {
        commanderActiveSetpoint = *setpoint;
        commanderActivePriority = COMMANDER_PRIORITY_DISABLE;
        commanderLastUpdateTick = supervisorTick;
        return true;
    }

    if (priority < commanderActivePriority) {
        return false;
    }

    commanderActiveSetpoint = *setpoint;
    commanderActivePriority = priority;
    commanderLastUpdateTick = supervisorTick;

    return true;
}

void commanderGetSetpoint(Setpoint_t *setpoint, uint32_t currentTick)
{
    if (setpoint == NULL) {
        return;
    }

    *setpoint = commanderActiveSetpoint;

    if (commanderActivePriority != COMMANDER_PRIORITY_DISABLE) {
        supervisorLastSetpointUpdateTick = currentTick;
    }
}

int commanderRelaxPriority(void)
{
    commanderActivePriority = COMMANDER_PRIORITY_LOWEST;
    return commanderActivePriority;
}

int commanderInactivityTime(uint32_t currentTick)
{
    return (int)(currentTick - commanderLastUpdateTick);
}

int commanderGetActivePriority(void)
{
    return commanderActivePriority;
}

/* =========================================================================
 * Stabilizer
 * ========================================================================= */

static bool stabilizerInitialized = false;
static bool stabilizerSystemRunning = true;
static bool stabilizerSensorsCalibrated = true;
static SensorData_t stabilizerSensors;
static State_t stabilizerState;
static Setpoint_t stabilizerSetpoint;
static Control_t stabilizerControl;
static int32_t stabilizerMotorPwm[4];

static Setpoint_t highLevelSetpoint;
static bool highLevelSetpointPending = false;

void sensorsInit(void)
{
    memset(&stabilizerSensors, 0, sizeof(stabilizerSensors));
}

void stateEstimatorInit(void)
{
    estimatorInit();
    memset(&stabilizerState, 0, sizeof(stabilizerState));
}

void controllerInit(void)
{
    attitudeControllerInit();
}

void motorsInit(void)
{
    memset(stabilizerMotorPwm, 0, sizeof(stabilizerMotorPwm));
}

void collisionAvoidanceInit(void)
{
    /* No persistent collision-avoidance state in the host model. */
}

void stabilizerInit(void)
{
    if (stabilizerInitialized) {
        return;
    }

    sensorsInit();
    stateEstimatorInit();
    controllerInit();
    powerDistributionInit();
    motorsInit();
    collisionAvoidanceInit();

    memset(&stabilizerSetpoint, 0, sizeof(stabilizerSetpoint));
    memset(&stabilizerControl, 0, sizeof(stabilizerControl));
    memset(&highLevelSetpoint, 0, sizeof(highLevelSetpoint));
    highLevelSetpointPending = false;

    stabilizerInitialized = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint_t *setpoint)
{
    if (setpoint == NULL) {
        return false;
    }

    highLevelSetpoint = *setpoint;
    highLevelSetpointPending = true;
    return true;
}

void setMotorRatios(const int32_t motorRatios[4])
{
    if (motorRatios != NULL) {
        memcpy(stabilizerMotorPwm, motorRatios, sizeof(stabilizerMotorPwm));
        supervisorSetMotorRatios(motorRatios);
    }
}

void sensorsWaitDataReady(void)
{
    /* Host model: data is already ready or injected. */
}

void sensorsAcquire(SensorData_t *sensors, uint32_t tick)
{
    (void)tick;
    if (sensors != NULL) {
        *sensors = stabilizerSensors;
    }
}

void commanderGetSetpointFromStabilizer(Setpoint_t *setpoint, uint32_t tick)
{
    if (setpoint != NULL) {
        commanderGetSetpoint(setpoint, tick);
    }
}

void collisionAvoidanceUpdateSetpoint(Setpoint_t *setpoint, const State_t *state)
{
    (void)state;
    (void)setpoint;
}

void stabilizerTask(uint32_t stabilizerStep)
{
    if (!stabilizerInitialized) {
        stabilizerInit();
    }

    if (!stabilizerSystemRunning || !stabilizerSensorsCalibrated) {
        return;
    }

    sensorsWaitDataReady();
    sensorsAcquire(&stabilizerSensors, stabilizerStep);

    estimatorComplementary(&stabilizerState, &stabilizerSensors, stabilizerStep);

    commanderGetSetpointFromStabilizer(&stabilizerSetpoint, supervisorTick);

    if (highLevelSetpointPending) {
        commanderSetSetpoint(&highLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        highLevelSetpointPending = false;
        commanderGetSetpointFromStabilizer(&stabilizerSetpoint, supervisorTick);
    }

    supervisorUpdate(stabilizerStep);

    if (!supervisorCanFly()) {
        memset(&stabilizerControl, 0, sizeof(stabilizerControl));
        memset(stabilizerMotorPwm, 0, sizeof(stabilizerMotorPwm));
        setMotorRatios(stabilizerMotorPwm);
        return;
    }

    collisionAvoidanceUpdateSetpoint(&stabilizerSetpoint, &stabilizerState);
    supervisorOverrideSetpoint(&stabilizerSetpoint, &stabilizerState);

    controllerPid(&stabilizerState, &stabilizerSetpoint, &stabilizerSensors,
                  &stabilizerControl, 0.001f);

    powerDistribution(&stabilizerControl, stabilizerMotorPwm);

    for (int i = 0; i < 4; i++) {
        float compensated =
            batteryCompensation((float)stabilizerMotorPwm[i], 0.0f);
        stabilizerMotorPwm[i] = (int32_t)compensated;
    }

    powerDistributionCap(stabilizerMotorPwm,
                         supervisorSafetyConfig.maxAllowedThrust,
                         supervisorSafetyConfig.idleThrust);

    if (!supervisorAreMotorsAllowedToRun()) {
        memset(stabilizerMotorPwm, 0, sizeof(stabilizerMotorPwm));
    }

    setMotorRatios(stabilizerMotorPwm);
}

/* =========================================================================
 * State compression and frequency supervisor
 * ========================================================================= */

uint32_t compressState(const State_t *state)
{
    if (state == NULL) {
        return 0;
    }

    int32_t px = (int32_t)(state->position.x * 1000.0f);
    int32_t py = (int32_t)(state->position.y * 1000.0f);
    int32_t pz = (int32_t)(state->position.z * 1000.0f);

    int32_t vx = (int32_t)(state->velocity.x * 1000.0f);
    int32_t vy = (int32_t)(state->velocity.y * 1000.0f);
    int32_t vz = (int32_t)(state->velocity.z * 1000.0f);

    int32_t ax = (int32_t)(state->acc.x * 9810.0f);
    int32_t ay = (int32_t)(state->acc.y * 9810.0f);
    int32_t az = (int32_t)((state->acc.z + 1.0f) * 9810.0f);

    int32_t gx = (int32_t)(state->attitude.roll * 0.0f);
    int32_t gy = (int32_t)(state->attitude.pitch * 0.0f);
    int32_t gz = (int32_t)(state->attitude.yaw * 0.0f);

    (void)px;
    (void)py;
    (void)pz;
    (void)vx;
    (void)vy;
    (void)vz;
    (void)ax;
    (void)ay;
    (void)az;
    (void)gx;
    (void)gy;
    (void)gz;

    uint32_t q0 = 0;
    uint32_t q1 = 0;
    uint32_t q2 = 0;
    uint32_t q3 = 0;

    if (state->attitude.qw < 0.0f) {
        q0 = 1u << 31;
    }
    if (state->attitude.qx < 0.0f) {
        q1 = 1u << 31;
    }
    if (state->attitude.qy < 0.0f) {
        q2 = 1u << 31;
    }
    if (state->attitude.qz < 0.0f) {
        q3 = 1u << 31;
    }

    return q0 ^ q1 ^ q2 ^ q3;
}

bool rateSupervisorValidate(uint32_t rate)
{
    return rate >= 997 && rate <= 1003;
}

/* =========================================================================
 * Health
 * ========================================================================= */

typedef enum {
    HEALTH_TEST_NONE = 0,
    HEALTH_TEST_PROP,
    HEALTH_TEST_BAT
} HealthTestType_t;

static HealthTestType_t healthCurrentTest = HEALTH_TEST_NONE;
static bool propTestRequested = false;
static bool batTestRequested = false;

static uint8_t propPhase = 0;
static uint8_t propMotorIndex = 0;
static uint16_t propSampleCount = 0;
static float propIdleVoltage = 0.0f;
static float propSamples[100];
static float propVibration[4];
static float propMotorVoltage[4];

static uint8_t batTick = 0;
static float batIdleVoltage = 0.0f;
static float batMinLoadedVoltage = 0.0f;
static uint32_t batRestartTick = 0;

static float healthVariance(const float *data, uint16_t n)
{
    if (data == NULL || n == 0) {
        return 0.0f;
    }

    float sum = 0.0f;
    float sumSq = 0.0f;

    for (uint16_t i = 0; i < n; i++) {
        sum += data[i];
        sumSq += data[i] * data[i];
    }

    return sumSq - (sum * sum / (float)n);
}

void startPropTest(void)
{
    propTestRequested = true;
}

void startBatTest(void)
{
    batTestRequested = true;
}

bool healthShallWeRunTest(void)
{
    if (propTestRequested) {
        propTestRequested = false;
        healthCurrentTest = HEALTH_TEST_PROP;
        propPhase = 0;
        propMotorIndex = 0;
        propSampleCount = 0;
        propIdleVoltage = 0.0f;
        memset(propSamples, 0, sizeof(propSamples));
        memset(propVibration, 0, sizeof(propVibration));
        memset(propMotorVoltage, 0, sizeof(propMotorVoltage));
        healthLog.motorPass = 0;
        healthLog.batteryPass = false;
        healthLog.motorTestCount = 0;
        return true;
    }

    if (batTestRequested) {
        batTestRequested = false;
        healthCurrentTest = HEALTH_TEST_BAT;
        batTick = 0;
        batIdleVoltage = 0.0f;
        batMinLoadedVoltage = 0.0f;
        healthLog.batteryPass = false;
        healthLog.batterySag = 0.0f;
        return true;
    }

    return false;
}

void healthRunTests(uint32_t tick)
{
    (void)tick;

    if (healthCurrentTest == HEALTH_TEST_NONE) {
        return;
    }

    if (healthCurrentTest == HEALTH_TEST_PROP) {
        switch (propPhase) {
        case 0:
            propIdleVoltage = 3.0f;
            propPhase = 1;
            propSampleCount = 0;
            break;
        case 1:
            if (propSampleCount < 100) {
                propSamples[propSampleCount++] = 0.0f;
            } else {
                propPhase = 2;
                propMotorIndex = 0;
            }
            break;
        case 2:
            if (propMotorIndex < 4) {
                healthLog.motorTestCount = propMotorIndex + 1;
                propVibration[propMotorIndex] = healthVariance(propSamples, 100);
                propMotorVoltage[propMotorIndex] = 2.8f;
                propPhase = 3;
            } else {
                propPhase = 4;
            }
            break;
        case 3:
            propMotorIndex++;
            if (propMotorIndex >= 4) {
                propPhase = 4;
            } else {
                propPhase = 2;
            }
            break;
        case 4:
            healthLog.batteryPass = evaluatePropTest();
            healthCurrentTest = HEALTH_TEST_NONE;
            break;
        default:
            break;
        }
    } else if (healthCurrentTest == HEALTH_TEST_BAT) {
        if (batTick == 0) {
            batIdleVoltage = 3.0f;
            batMinLoadedVoltage = batIdleVoltage;
            batTick = 1;
            return;
        }

        if (batTick == 1) {
            batMinLoadedVoltage = batIdleVoltage - 0.2f;
            batTick = 2;
            return;
        }

        if (batTick >= 2 && batTick < 50) {
            if (batMinLoadedVoltage > batIdleVoltage - 0.1f) {
                batMinLoadedVoltage = batIdleVoltage - 0.1f;
            }
            batTick++;
            return;
        }

        if (batTick == 50) {
            healthLog.batterySag = batIdleVoltage - batMinLoadedVoltage;
            healthLog.batteryPass = healthLog.batterySag <= 0.5f;
            healthCurrentTest = HEALTH_TEST_NONE;
            batTick = 0;
        }
    }
}

bool evaluatePropTest(void)
{
    const float highThreshold = 1.0f;
    const float lowThreshold = 0.0f;

    if (highThreshold == 0.0f) {
        return true;
    }

    bool allPass = true;

    for (uint8_t i = 0; i < 4; i++) {
        if (propVibration[i] >= lowThreshold &&
            propVibration[i] <= highThreshold) {
            healthLog.motorPass |= (1u << i);
        } else {
            allPass = false;
        }
    }

    return allPass;
}

void restartBatTest(void)
{
    batRestartTick = supervisorTick;
    startBatTest();
}

/* =========================================================================
 * CRTP transport
 * ========================================================================= */

typedef struct {
    CRTPPacket_t buffer[CRTP_RX_QUEUE_CAPACITY];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    bool active;
} CRTPRxQueue_t;

static CRTPPacket_t crtpTxQueue[CRTP_TX_QUEUE_CAPACITY];
static uint16_t crtpTxHead = 0;
static uint16_t crtpTxTail = 0;
static uint16_t crtpTxCount = 0;

static CRTPRxQueue_t crtpRxQueues[CRTP_NBR_OF_PORTS];
static void (*crtpRxCallbacks[CRTP_NBR_OF_PORTS])(const CRTPPacket_t *);

static uint32_t crtpRxPackets = 0;
static uint32_t crtpTxPackets = 0;
static uint32_t crtpRxRate = 0;
static uint32_t crtpTxRate = 0;
static uint32_t crtpLastStatsTick = 0;

static bool crtpNopLinkSend(const CRTPPacket_t *packet)
{
    (void)packet;
    return true;
}

static bool crtpNopLinkReceive(CRTPPacket_t *packet)
{
    (void)packet;
    return false;
}

static void crtpNopLinkReset(void)
{
}

static bool crtpNopLinkIsConnected(void)
{
    return true;
}

static CRTPLink_t crtpNopLink = {
    crtpNopLinkSend,
    crtpNopLinkReceive,
    crtpNopLinkReset,
    crtpNopLinkIsConnected
};

static CRTPLink_t *crtpActiveLink = &crtpNopLink;

static bool crtpRxQueuePush(uint8_t port, const CRTPPacket_t *packet)
{
    if (port >= CRTP_NBR_OF_PORTS || packet == NULL) {
        return false;
    }

    CRTPRxQueue_t *q = &crtpRxQueues[port];

    if (!q->active || q->count >= CRTP_RX_QUEUE_CAPACITY) {
        return false;
    }

    q->buffer[q->tail] = *packet;
    q->tail = (uint8_t)((q->tail + 1) % CRTP_RX_QUEUE_CAPACITY);
    q->count++;
    return true;
}

static bool crtpRxQueuePop(uint8_t port, CRTPPacket_t *packet)
{
    if (port >= CRTP_NBR_OF_PORTS || packet == NULL) {
        return false;
    }

    CRTPRxQueue_t *q = &crtpRxQueues[port];

    if (!q->active || q->count == 0) {
        return false;
    }

    *packet = q->buffer[q->head];
    q->head = (uint8_t)((q->head + 1) % CRTP_RX_QUEUE_CAPACITY);
    q->count--;
    return true;
}

void crtpInit(void)
{
    static bool crtpInitialized = false;

    if (crtpInitialized) {
        return;
    }

    crtpTxHead = 0;
    crtpTxTail = 0;
    crtpTxCount = 0;
    memset(crtpRxQueues, 0, sizeof(crtpRxQueues));
    memset(crtpRxCallbacks, 0, sizeof(crtpRxCallbacks));

    for (int port = 0; port < CRTP_NBR_OF_PORTS; port++) {
        crtpRxQueues[port].active = false;
    }

    crtpActiveLink = &crtpNopLink;
    crtpRxPackets = 0;
    crtpTxPackets = 0;
    crtpRxRate = 0;
    crtpTxRate = 0;
    crtpLastStatsTick = 0;

    crtpInitialized = true;
}

bool crtpSendPacket(const CRTPPacket_t *packet)
{
    if (packet == NULL || crtpTxCount >= CRTP_TX_QUEUE_CAPACITY) {
        return false;
    }

    crtpTxQueue[crtpTxTail] = *packet;
    crtpTxTail = (uint16_t)((crtpTxTail + 1) % CRTP_TX_QUEUE_CAPACITY);
    crtpTxCount++;
    return true;
}

bool crtpSendPacketBlock(const CRTPPacket_t *packet)
{
    if (crtpTxCount >= CRTP_TX_QUEUE_CAPACITY) {
        return false;
    }

    return crtpSendPacket(packet);
}

bool crtpReceivePacket(CRTPPacket_t *packet)
{
    if (packet == NULL) {
        return false;
    }

    for (uint8_t port = 0; port < CRTP_NBR_OF_PORTS; port++) {
        if (crtpRxQueuePop(port, packet)) {
            return true;
        }
    }

    if (crtpActiveLink != NULL && crtpActiveLink->receive != NULL) {
        if (crtpActiveLink->receive(packet)) {
            return true;
        }
    }

    return false;
}

bool crtpReceivePacketBlock(CRTPPacket_t *packet, uint32_t timeoutMs)
{
    (void)timeoutMs;
    return crtpReceivePacket(packet);
}

bool crtpReceivePacketWait(CRTPPacket_t *packet)
{
    return crtpReceivePacket(packet);
}

void crtpSetLink(CRTPLink_t *link)
{
    if (link == NULL) {
        crtpActiveLink = &crtpNopLink;
    } else {
        crtpActiveLink = link;
    }
}

void crtpReset(void)
{
    crtpTxHead = 0;
    crtpTxTail = 0;
    crtpTxCount = 0;

    for (int port = 0; port < CRTP_NBR_OF_PORTS; port++) {
        crtpRxQueues[port].head = 0;
        crtpRxQueues[port].tail = 0;
        crtpRxQueues[port].count = 0;
        crtpRxQueues[port].active = false;
    }

    if (crtpActiveLink != NULL && crtpActiveLink->reset != NULL) {
        crtpActiveLink->reset();
    }
}

bool crtpIsConnected(void)
{
    if (crtpActiveLink != NULL && crtpActiveLink->isConnected != NULL) {
        return crtpActiveLink->isConnected();
    }

    return true;
}

uint16_t crtpGetFreeTxQueuePackets(void)
{
    return (uint16_t)(CRTP_TX_QUEUE_CAPACITY - crtpTxCount);
}

bool crtpRegisterPortCB(uint8_t port, void (*callback)(const CRTPPacket_t *))
{
    if (port >= CRTP_NBR_OF_PORTS) {
        return false;
    }

    if (crtpRxQueues[port].active) {
        return false;
    }

    crtpRxQueues[port].active = true;
    crtpRxCallbacks[port] = callback;
    return true;
}

void crtpUpdateStats(void)
{
    uint32_t now = supervisorTick;
    uint32_t elapsed = now - crtpLastStatsTick;

    if (elapsed >= 500) {
        crtpRxRate = crtpRxPackets;
        crtpTxRate = crtpTxPackets;
        crtpRxPackets = 0;
        crtpTxPackets = 0;
        crtpLastStatsTick = now;
    }
}

void crtpRxTask(void)
{
    if (crtpActiveLink == NULL || crtpActiveLink->receive == NULL) {
        return;
    }

    if (crtpActiveLink == &crtpNopLink) {
        return;
    }

    CRTPPacket_t packet;

    if (crtpActiveLink->receive(&packet)) {
        crtpRxPackets++;

        if (packet.port >= CRTP_NBR_OF_PORTS) {
            return;
        }

        bool delivered = false;

        if (crtpRxQueues[packet.port].active) {
            if (crtpRxQueuePush(packet.port, &packet)) {
                delivered = true;
            }
        }

        if (crtpRxCallbacks[packet.port] != NULL) {
            crtpRxCallbacks[packet.port](&packet);
            delivered = true;
        }

        if (!delivered) {
            /* Drop packet. */
        }
    }
}

void crtpTxTask(void)
{
    if (crtpActiveLink == NULL || crtpActiveLink->send == NULL) {
        return;
    }

    if (crtpActiveLink == &crtpNopLink || crtpTxCount == 0) {
        return;
    }

    CRTPPacket_t packet = crtpTxQueue[crtpTxHead];

    if (crtpActiveLink->send(&packet)) {
        crtpTxHead = (uint16_t)((crtpTxHead + 1) % CRTP_TX_QUEUE_CAPACITY);
        crtpTxCount--;
        crtpTxPackets++;
    } else {
        /* Preserve packet for retry on a later scheduler tick. */
    }
}

/* =========================================================================
 * Deck discovery
 * ========================================================================= */

int deckDiscovery(uint8_t *buffer, int capacity)
{
    if (buffer == NULL || capacity <= 0) {
        return 0;
    }

    static const uint8_t knownDeckIds[] = {
        0x01,
        0x02,
        0x03
    };

    int count = 0;

    for (size_t i = 0;
         i < sizeof(knownDeckIds) / sizeof(knownDeckIds[0]) &&
         count < capacity;
         i++) {
        bool duplicate = false;

        for (int j = 0; j < count; j++) {
            if (buffer[j] == knownDeckIds[i]) {
                duplicate = true;
                break;
            }
        }

        if (!duplicate) {
            buffer[count++] = knownDeckIds[i];
        }
    }

    return count;
}

/* =========================================================================
 * Log objects
 * ========================================================================= */

StateEstimateLog_t stateEstimate;
GyroLog_t gyro;
AccLog_t acc;
BaroLog_t baro;
MotorLog_t motor;
Sensfusion6Log_t sensfusion6Log;
SupervisorLog_t supervisorLog;
HealthLog_t healthLog;

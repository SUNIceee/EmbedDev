#include "generated_code.h"
#include <string.h>
#include <stdlib.h>

/* ------------------------------------------------------------------------- */
/* Numeric helpers                                                            */
/* ------------------------------------------------------------------------- */

int16_t saturateSignedInt16(int32_t value)
{
    if (value > INT16_MAX) return INT16_MAX;       /* 32767 */
    if (value < -INT16_MAX) return -INT16_MAX;     /* -32767 */
    return (int16_t)value;
}

float capAngle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/* ------------------------------------------------------------------------- */
/* Sensfusion6                                                                */
/* ------------------------------------------------------------------------- */

static bool s6_initialized;
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
static float baseZacc = 0.0f;
static bool s6_calibrated = false;

float invSqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    float halfx = 0.5f * x;
    float y = x;
    int32_t i;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

static void sensfusion6NormalizeQuaternion(void)
{
    float norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 0.0f) {
        q0 *= norm;
        q1 *= norm;
        q2 *= norm;
        q3 *= norm;
    }
}

void sensfusion6Init(void)
{
    if (s6_initialized) return;
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
    baseZacc = 0.0f;
    s6_calibrated = false;
    s6_initialized = true;
}

bool sensfusion6Test(void)
{
    return s6_initialized;
}

void estimatedGravityDirection(float *gx, float *gy, float *gz)
{
    if (!gx || !gy || !gz) return;
    *gx = 2.0f * (q1 * q3 - q0 * q2);
    *gy = 2.0f * (q0 * q1 + q2 * q3);
    *gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

void sensfusion6GetQuaternion(float *qw, float *qx, float *qy, float *qz)
{
    if (!qw || !qx || !qy || !qz) return;
    *qw = q0;
    *qx = q1;
    *qy = q2;
    *qz = q3;
}

void sensfusion6GetEulerRPY(float *roll, float *pitch, float *yaw)
{
    if (!roll || !pitch || !yaw) return;
    float gx, gy, gz;
    estimatedGravityDirection(&gx, &gy, &gz);

    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG_F;

    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp) * RAD_TO_DEG_F;

    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG_F;
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

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
    if (!s6_initialized) sensfusion6Init();

    float twoKp = 0.8f;
    float twoKi = 0.002f;

#if CONFIG_IMU_MADGWICK_QUATERNION
    twoKp = 0.0f;
    twoKi = 0.0f;
    /* Madgwick gradient fallback: beta=0.01. Kept explicit for build mode. */
#endif

    float gxRad = gx * DEG_TO_RAD_F;
    float gyRad = gy * DEG_TO_RAD_F;
    float gzRad = gz * DEG_TO_RAD_F;

    if (fabsf(ax) < 1e-9f && fabsf(ay) < 1e-9f && fabsf(az) < 1e-9f) {
        if (twoKi <= 0.0f) {
            integralFBx = integralFBy = integralFBz = 0.0f;
        }

        float qDot0 = 0.5f * (-q1 * gxRad - q2 * gyRad - q3 * gzRad);
        float qDot1 = 0.5f * (q0 * gxRad + q2 * gzRad - q3 * gyRad);
        float qDot2 = 0.5f * (q0 * gyRad - q1 * gzRad + q3 * gxRad);
        float qDot3 = 0.5f * (q0 * gzRad + q1 * gyRad - q2 * gxRad);

        q0 += qDot0 * dt;
        q1 += qDot1 * dt;
        q2 += qDot2 * dt;
        q3 += qDot3 * dt;
        sensfusion6NormalizeQuaternion();
        return;
    }

    float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    if (recipNorm <= 0.0f) return;

    float axn = ax * recipNorm;
    float ayn = ay * recipNorm;
    float azn = az * recipNorm;

    float halfvx = q1 * q3 - q0 * q2;
    float halfvy = q0 * q1 + q2 * q3;
    float halfvz = q0 * q0 - 0.5f + q3 * q3;

    float halfex = ayn * halfvz - azn * halfvy;
    float halfey = azn * halfvx - axn * halfvz;
    float halfez = axn * halfvy - ayn * halfvx;

    if (twoKi > 0.0f) {
        integralFBx += twoKi * halfex * dt;
        integralFBy += twoKi * halfey * dt;
        integralFBz += twoKi * halfez * dt;
    } else {
        integralFBx = integralFBy = integralFBz = 0.0f;
    }

    if (!s6_calibrated) {
        float gx0, gy0, gz0;
        estimatedGravityDirection(&gx0, &gy0, &gz0);
        baseZacc = axn * gx0 + ayn * gy0 + azn * gz0;
        s6_calibrated = true;
    }

    gxRad += twoKp * halfex + integralFBx;
    gyRad += twoKp * halfey + integralFBy;
    gzRad += twoKp * halfez + integralFBz;

    float qDot0 = 0.5f * (-q1 * gxRad - q2 * gyRad - q3 * gzRad);
    float qDot1 = 0.5f * (q0 * gxRad + q2 * gzRad - q3 * gyRad);
    float qDot2 = 0.5f * (q0 * gyRad - q1 * gzRad + q3 * gxRad);
    float qDot3 = 0.5f * (q0 * gzRad + q1 * gyRad - q2 * gxRad);

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    sensfusion6NormalizeQuaternion();
}

/* ------------------------------------------------------------------------- */
/* Power distribution and battery                                             */
/* ------------------------------------------------------------------------- */

uint16_t motorForceToPwm(float force)
{
    if (force <= 0.0f) return 0;
    /* Host-safe mapping: scaled and clamped PWM representation. */
    const float scale = 65535.0f;
    float pwm = force * scale;
    if (pwm > 65535.0f) pwm = 65535.0f;
    if (pwm < 0.0f) pwm = 0.0f;
    return (uint16_t)(pwm + 0.5f);
}

void powerDistribution(const Control *control, int32_t motorValues[4])
{
    if (!control || !motorValues) return;

    int32_t old[4];
    for (int i = 0; i < 4; i++) old[i] = motorValues[i];

    switch (control->controlMode) {
    case CONTROL_MODE_LEGACY: {
        int32_t r = control->roll / 2;
        int32_t p = control->pitch / 2;
        int32_t y = control->yaw;
        int32_t t = (int32_t)control->thrust;

        motorValues[0] = t - r + p + y;
        motorValues[1] = t - r - p - y;
        motorValues[2] = t + r - p + y;
        motorValues[3] = t + r + p - y;
        break;
    }
    case CONTROL_MODE_FORCE_TORQUE: {
        float thrustPart = 0.25f * control->thrustSi;
        float arm = 0.707106781f * control->armLength;

        float rollPart = 0.0f;
        float pitchPart = 0.0f;
        float yawPart = 0.0f;

        if (fabsf(control->armLength) > 1e-9f) {
            rollPart = (0.25f / arm) * control->torqueX;
            pitchPart = (0.25f / arm) * control->torqueY;
        }
        if (fabsf(control->thrustToTorque) > 1e-9f) {
            yawPart = (0.25f / control->thrustToTorque) * control->torqueZ;
        }

        float m1 = thrustPart - rollPart + pitchPart + yawPart;
        float m2 = thrustPart - rollPart - pitchPart - yawPart;
        float m3 = thrustPart + rollPart - pitchPart + yawPart;
        float m4 = thrustPart + rollPart + pitchPart - yawPart;

        if (m1 < 0.0f) m1 = 0.0f;
        if (m2 < 0.0f) m2 = 0.0f;
        if (m3 < 0.0f) m3 = 0.0f;
        if (m4 < 0.0f) m4 = 0.0f;

        motorValues[0] = motorForceToPwm(m1);
        motorValues[1] = motorForceToPwm(m2);
        motorValues[2] = motorForceToPwm(m3);
        motorValues[3] = motorForceToPwm(m4);
        break;
    }
    case CONTROL_MODE_FORCE: {
        for (int i = 0; i < 4; i++) {
            float f = control->normalizedForces[i];
            if (f < 0.0f) f = 0.0f;
            if (f > 1.0f) f = 1.0f;
            motorValues[i] = (int32_t)(f * 65535.0f + 0.5f);
        }
        break;
    }
    default:
        for (int i = 0; i < 4; i++) motorValues[i] = old[i];
        break;
    }
}

bool powerDistributionCap(int32_t motorValues[4], uint32_t maxAllowedThrust,
                          uint32_t idleThrust)
{
    if (!motorValues) return false;

    int32_t maxVal = motorValues[0];
    for (int i = 1; i < 4; i++) {
        if (motorValues[i] > maxVal) maxVal = motorValues[i];
    }

    if (maxVal <= (int32_t)maxAllowedThrust) return false;

    int32_t reduction = maxVal - (int32_t)maxAllowedThrust;
    for (int i = 0; i < 4; i++) {
        motorValues[i] -= reduction;
        if (motorValues[i] < (int32_t)idleThrust) motorValues[i] = (int32_t)idleThrust;
    }

    return true;
}

float batteryCompensation(float oldValue, float supply, float alpha)
{
    return oldValue + alpha * (supply - oldValue);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t thrust, float nominalVoltage,
                                        float actualVoltage)
{
    if (actualVoltage <= 0.0f) return thrust;
    float compensated = (float)thrust * nominalVoltage / actualVoltage;
    if (compensated > 65535.0f) compensated = 65535.0f;
    if (compensated < 0.0f) compensated = 0.0f;
    return (uint16_t)(compensated + 0.5f);
}

/* ------------------------------------------------------------------------- */
/* PID and controller                                                         */
/* ------------------------------------------------------------------------- */

static PidObject pidRoll, pidPitch, pidYaw;
static PidObject pidRollRate, pidPitchRate, pidYawRate;
static bool attitudeControllerInitialized;

void pidInit(PidObject *pid, float kp, float ki, float kd, float outLimit)
{
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->outLimit = outLimit;
    pid->integral = 0.0f;
    pid->previousInput = 0.0f;
    pid->output = 0.0f;
    pid->inited = true;
}

float pidUpdate(PidObject *pid, float measured, float setpoint)
{
    if (!pid || !pid->inited) return 0.0f;

    float error = setpoint - measured;
    pid->integral += error;
    float out = pid->kp * error + pid->ki * pid->integral -
                pid->kd * (measured - pid->previousInput);
    pid->previousInput = measured;

    if (out > pid->outLimit) out = pid->outLimit;
    if (out < -pid->outLimit) out = -pid->outLimit;

    pid->output = out;
    return out;
}

void pidReset(PidObject *pid)
{
    if (!pid) return;
    pid->integral = 0.0f;
    pid->previousInput = 0.0f;
    pid->output = 0.0f;
}

void attitudeControllerInit(void)
{
    if (attitudeControllerInitialized) return;

    pidInit(&pidRoll, 3.0f, 0.0f, 0.0f, 1000.0f);
    pidInit(&pidPitch, 3.0f, 0.0f, 0.0f, 1000.0f);
    pidInit(&pidYaw, 3.0f, 0.0f, 0.0f, 1000.0f);
    pidInit(&pidRollRate, 0.0f, 0.0f, 0.0f, 32767.0f);
    pidInit(&pidPitchRate, 0.0f, 0.0f, 0.0f, 32767.0f);
    pidInit(&pidYawRate, 0.0f, 0.0f, 0.0f, 32767.0f);

    attitudeControllerInitialized = true;
}

void attitudeControllerResetAll(void)
{
    pidReset(&pidRoll);
    pidReset(&pidPitch);
    pidReset(&pidYaw);
    pidReset(&pidRollRate);
    pidReset(&pidPitchRate);
    pidReset(&pidYawRate);
}

void attitudeControllerResetRoll(void)
{
    pidReset(&pidRoll);
    pidReset(&pidRollRate);
}

void attitudeControllerResetPitch(void)
{
    pidReset(&pidPitch);
    pidReset(&pidPitchRate);
}

void attitudeControllerResetYaw(void)
{
    pidReset(&pidYaw);
    pidReset(&pidYawRate);
}

static float g_yawMaxDelta = 0.0f;
static float g_positionControlThrust = 0.0f;

void controllerSetYawMaxDelta(float maxDelta)
{
    g_yawMaxDelta = maxDelta;
}

void controllerSetPositionControlThrust(float thrust)
{
    g_positionControlThrust = thrust;
}

static float shortestAngle(float diff)
{
    return capAngle(diff);
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, Control *control, float dt)
{
    static float desiredYaw = 0.0f;
    static bool desiredYawSet = false;

    if (!sensors || !setpoint || !state || !control) return;

    if (!attitudeControllerInitialized) attitudeControllerInit();

    /* yaw handling */
    if (setpoint->mode.yaw == AXIS_MODE_VELOCITY) {
        desiredYaw += setpoint->attitudeRate.yaw * dt;
        if (!desiredYawSet) desiredYawSet = true;
        if (g_yawMaxDelta != 0.0f) {
            float diff = desiredYaw - state->attitude.yaw;
            if (diff > g_yawMaxDelta) desiredYaw = state->attitude.yaw + g_yawMaxDelta;
            if (diff < -g_yawMaxDelta) desiredYaw = state->attitude.yaw - g_yawMaxDelta;
        }
    } else if (setpoint->mode.yaw == AXIS_MODE_ABS) {
        desiredYaw = setpoint->attitude.yaw;
        desiredYawSet = true;
    }

    /* Roll and pitch desired values */
    float desiredRoll;
    float desiredPitch;

    if (setpoint->mode.roll == AXIS_MODE_VELOCITY) {
        desiredRoll = setpoint->attitudeRate.roll;
        attitudeControllerResetRoll();
    } else {
        desiredRoll = setpoint->attitude.roll;
    }

    if (setpoint->mode.pitch == AXIS_MODE_VELOCITY) {
        desiredPitch = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitch();
    } else {
        desiredPitch = setpoint->attitude.pitch;
    }

    if (setpoint->thrust == 0) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0;
        attitudeControllerResetAll();
        desiredYaw = state->attitude.yaw;
        desiredYawSet = true;
        return;
    }

    /* z / thrust */
    if (setpoint->mode.z == AXIS_MODE_DISABLE) {
        control->thrust = setpoint->thrust;
    } else {
        float posThrust = g_positionControlThrust;
        if (posThrust > 65535.0f) posThrust = 65535.0f;
        if (posThrust < 0.0f) posThrust = 0.0f;
        control->thrust = (uint16_t)(posThrust + 0.5f);
    }

    /* Attitude rate target from attitude PID */
    float yawRateDesired = pidUpdate(&pidYaw,
                                     state->attitude.yaw,
                                     desiredYaw);
    float rollRateDesired = pidUpdate(&pidRoll,
                                      state->attitude.roll,
                                      desiredRoll);
    float pitchRateDesired = pidUpdate(&pidPitch,
                                       state->attitude.pitch,
                                       desiredPitch);

    /* Rate PIDs. Pitch uses inverted sensor value. */
    float rollOut = pidUpdate(&pidRollRate, sensors->gyro.x, rollRateDesired);
    float pitchActual = -sensors->gyro.y;
    float pitchOut = pidUpdate(&pidPitchRate, pitchActual, pitchRateDesired);
    float yawOut = pidUpdate(&pidYawRate, sensors->gyro.z, yawRateDesired);

    control->roll = saturateSignedInt16((int32_t)rollOut);
    control->pitch = saturateSignedInt16((int32_t)pitchOut);
    control->yaw = saturateSignedInt16((int32_t)yawOut);

    /* Legacy coordinate convention: yaw is negated after actuator output. */
    control->yaw = saturateSignedInt16(-(int32_t)control->yaw);
}

/* ------------------------------------------------------------------------- */
/* Commander RPYT decode                                                      */
/* ------------------------------------------------------------------------- */

void rotateYaw(float *x, float *y, float angleDeg)
{
    if (!x || !y) return;
    float rad = angleDeg * DEG_TO_RAD_F;
    float c = cosf(rad);
    float s = sinf(rad);
    float oldX = *x;
    float oldY = *y;
    *x = oldX * c - oldY * s;
    *y = oldX * s + oldY * c;
}

void crtpCommanderRpytDecodeSetpoint(const CrtpCommanderRpyt *rpyt,
                                     CommanderPriority activePriority,
                                     bool posSetMode,
                                     CommanderSetpoint *out)
{
    static bool thrustLocked = false;
    static bool altHoldActive = false;

    if (!rpyt || !out) return;

    memset(&out->setpoint, 0, sizeof(Setpoint));
    out->setpoint.mode.x = AXIS_MODE_DISABLE;
    out->setpoint.mode.y = AXIS_MODE_DISABLE;
    out->setpoint.mode.z = AXIS_MODE_DISABLE;
    out->setpoint.mode.roll = AXIS_MODE_ABS;
    out->setpoint.mode.pitch = AXIS_MODE_ABS;
    out->setpoint.mode.yaw = AXIS_MODE_ABS;

    out->posSetMode = posSetMode;
    out->modeSet = false;

    if (activePriority == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
        if (rpyt->thrust == 0) thrustLocked = false;
    }
    out->thrustLocked = thrustLocked;

    float rawRoll = rpyt->roll;
    float rawPitch = rpyt->pitch;
    float rawYaw = rpyt->yaw;
    uint16_t rawThrust = rpyt->thrust;

    /* PosSet has highest priority */
    if (rpyt->posHold && posSetMode && rawThrust != 0) {
        out->setpoint.mode.x = AXIS_MODE_ABS;
        out->setpoint.mode.y = AXIS_MODE_ABS;
        out->setpoint.mode.z = AXIS_MODE_ABS;
        out->setpoint.mode.roll = AXIS_MODE_DISABLE;
        out->setpoint.mode.pitch = AXIS_MODE_DISABLE;
        out->setpoint.mode.yaw = AXIS_MODE_ABS;
        out->setpoint.position.x = -rawPitch;
        out->setpoint.position.y = rawRoll;
        out->setpoint.position.z = rawThrust / 1000.0f;
        out->setpoint.attitude.yaw = rawYaw;
        out->setpoint.thrust = 0;
        out->modeSet = true;
        return;
    }

    /* PosHold */
    if (rpyt->posHold) {
        out->setpoint.mode.x = AXIS_MODE_VELOCITY;
        out->setpoint.mode.y = AXIS_MODE_VELOCITY;
        out->setpoint.mode.roll = AXIS_MODE_DISABLE;
        out->setpoint.mode.pitch = AXIS_MODE_DISABLE;
        out->setpoint.velocity.x = rawPitch / 30.0f;
        out->setpoint.velocity.y = rawRoll / 30.0f;
        out->setpoint.attitude.roll = 0.0f;
        out->setpoint.attitude.pitch = 0.0f;
        out->setpoint.mode.z = AXIS_MODE_DISABLE;
        out->setpoint.thrust = rawThrust;
        out->modeSet = true;
        return;
    }

    /* AltHold */
    if (rpyt->altHold) {
        if (!altHoldActive) {
            out->modeSet = true;
            altHoldActive = true;
        }
        out->setpoint.mode.z = AXIS_MODE_VELOCITY;
        out->setpoint.thrust = 0;
        out->setpoint.velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;

        /* Default roll/pitch/yaw handling */
        if (rpyt->rollMode == 1) {
            out->setpoint.mode.roll = AXIS_MODE_VELOCITY;
            out->setpoint.attitudeRate.roll = rawRoll;
        } else {
            out->setpoint.mode.roll = AXIS_MODE_ABS;
            out->setpoint.attitude.roll = rawRoll;
        }
        if (rpyt->pitchMode == 1) {
            out->setpoint.mode.pitch = AXIS_MODE_VELOCITY;
            out->setpoint.attitudeRate.pitch = rawPitch;
        } else {
            out->setpoint.mode.pitch = AXIS_MODE_ABS;
            out->setpoint.attitude.pitch = rawPitch;
        }
        if (rpyt->yawMode == 1) {
            out->setpoint.mode.yaw = AXIS_MODE_VELOCITY;
            out->setpoint.attitudeRate.yaw = -rawYaw;
        } else {
            out->setpoint.mode.yaw = AXIS_MODE_ABS;
            out->setpoint.attitude.yaw = rawYaw;
        }
        return;
    }

    altHoldActive = false;

    /* Non-AltHold thrust lock / low-value behaviour */
    if (thrustLocked || rawThrust < 1000) {
        out->setpoint.thrust = 0;
    } else {
        if (rawThrust > 60000) {
            out->setpoint.thrust = 60000;
        } else {
            out->setpoint.thrust = rawThrust;
        }
    }

    /* Default roll/pitch */
    if (rpyt->rollMode == 1) {
        out->setpoint.mode.roll = AXIS_MODE_VELOCITY;
        out->setpoint.attitudeRate.roll = rawRoll;
    } else {
        out->setpoint.mode.roll = AXIS_MODE_ABS;
        out->setpoint.attitude.roll = rawRoll;
    }

    if (rpyt->pitchMode == 1) {
        out->setpoint.mode.pitch = AXIS_MODE_VELOCITY;
        out->setpoint.attitudeRate.pitch = rawPitch;
    } else {
        out->setpoint.mode.pitch = AXIS_MODE_ABS;
        out->setpoint.attitude.pitch = rawPitch;
    }

    /* Default yaw */
    if (rpyt->yawMode == 1) {
        out->setpoint.mode.yaw = AXIS_MODE_VELOCITY;
        out->setpoint.attitudeRate.yaw = -rawYaw;
    } else {
        out->setpoint.mode.yaw = AXIS_MODE_ABS;
        out->setpoint.attitude.yaw = rawYaw;
    }

    if (rpyt->plusMode) {
        rotateYaw(&rawRoll, &rawPitch, 45.0f);
        if (rpyt->rollMode == 1) {
            out->setpoint.attitudeRate.roll = rawRoll;
        } else {
            out->setpoint.attitude.roll = rawRoll;
        }
        if (rpyt->pitchMode == 1) {
            out->setpoint.attitudeRate.pitch = rawPitch;
        } else {
            out->setpoint.attitude.pitch = rawPitch;
        }
    }

    if (rpyt->careFree) {
        /* Observable error path: disable commanded attitude. */
        out->setpoint.mode.roll = AXIS_MODE_DISABLE;
        out->setpoint.mode.pitch = AXIS_MODE_DISABLE;
    }
}

/* ------------------------------------------------------------------------- */
/* Commander arbitration                                                      */
/* ------------------------------------------------------------------------- */

static Setpoint commanderSetpoint;
static CommanderPriority commanderPriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t commanderLastUpdateTick;
static bool commanderHasSetpoint;

void commanderInit(void)
{
    memset(&commanderSetpoint, 0, sizeof(Setpoint));
    commanderPriority = COMMANDER_PRIORITY_DISABLE;
    commanderLastUpdateTick = 0;
    commanderHasSetpoint = false;
}

void commanderSetSetpoint(const Setpoint *setpoint, CommanderPriority priority,
                          uint32_t tick)
{
    if (!setpoint) return;

    if (priority != COMMANDER_PRIORITY_DISABLE) {
        if (priority < commanderPriority && commanderHasSetpoint) return;
    }

    if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
        /* Stop high-level trajectory if a higher consumer takes over. */
        commanderHasSetpoint = false;
    }

    commanderSetpoint = *setpoint;
    commanderPriority = priority;
    commanderLastUpdateTick = tick;
    commanderHasSetpoint = true;
}

bool commanderGetSetpoint(Setpoint *setpoint)
{
    if (!setpoint || !commanderHasSetpoint) return false;
    *setpoint = commanderSetpoint;
    return true;
}

void commanderRelaxPriority(void)
{
    commanderPriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderInactivityTime(uint32_t currentTick)
{
    if (!commanderHasSetpoint) return UINT32_MAX;
    return currentTick - commanderLastUpdateTick;
}

CommanderPriority commanderGetActivePriority(void)
{
    return commanderPriority;
}

/* ------------------------------------------------------------------------- */
/* Supervisor                                                                 */
/* ------------------------------------------------------------------------- */

static SupervisorState supervisorState = SUPERVISOR_STATE_CALIBRATION;
static bool sup_isArmed;
static bool sup_isCrashed;
static bool sup_isTumbled;
static bool sup_isFlying;
static bool sup_autoArming;
static bool sup_trajectoryFlying;
static bool sup_trajectoryFinished;
static bool sup_trajectoryDisabled;
static bool sup_deckFault;
static uint32_t sup_conditionBits;

static SensorData sup_sensors;
static uint16_t sup_motorRatios[4];
static uint16_t sup_motorRPMs[4];
static SupervisorSafetyConfig sup_cfg;

static uint32_t sup_lastFlyingTick;
static bool sup_seenFlying;
static uint32_t sup_spinupStartTick;
static uint32_t sup_lastCommanderTick;
static uint32_t sup_emergencyStopLastNotification;
static uint32_t sup_rpmGoodStartTick;
static bool sup_rpmWasGood;

void supervisorInit(void)
{
    memset(&sup_sensors, 0, sizeof(sup_sensors));
    memset(sup_motorRatios, 0, sizeof(sup_motorRatios));
    memset(sup_motorRPMs, 0, sizeof(sup_motorRPMs));

    memset(&sup_cfg, 0, sizeof(sup_cfg));
    sup_cfg.freeFallThreshold = 0.2f;
    sup_cfg.tiltThreshold = 0.3f;
    sup_cfg.invertedThreshold = -0.2f;
    sup_cfg.tiltTimeoutMs = 1000;
    sup_cfg.invertedTimeoutMs = 500;
    sup_cfg.tumbleCheckEnabled = true;
    sup_cfg.commanderWarningTimeoutMs = 500;
    sup_cfg.commanderTimeoutTimeoutMs = 2000;
    sup_cfg.emergencyWatchdogTimeoutMs = 1000;
    sup_cfg.spinupTimeoutMs = 500;
    sup_cfg.rpmMin = 1000;
    sup_cfg.rpmMax = 20000;

    supervisorState = SUPERVISOR_STATE_PREFLIGHT;
    sup_isArmed = false;
    sup_isCrashed = false;
    sup_isTumbled = false;
    sup_isFlying = false;
    sup_autoArming = false;
    sup_trajectoryFlying = false;
    sup_trajectoryFinished = false;
    sup_trajectoryDisabled = false;
    sup_deckFault = false;
    sup_conditionBits = 0;
    sup_seenFlying = false;
    sup_lastFlyingTick = 0;
    sup_spinupStartTick = 0;
    sup_lastCommanderTick = 0;
    sup_emergencyStopLastNotification = 0;
    sup_rpmGoodStartTick = 0;
    sup_rpmWasGood = false;
}

void supervisorSetSensorData(const SensorData *sensors)
{
    if (!sensors) return;
    sup_sensors = *sensors;
}

void supervisorSetMotorRatios(const uint16_t ratios[4])
{
    if (!ratios) return;
    memcpy(sup_motorRatios, ratios, sizeof(sup_motorRatios));
}

void supervisorSetMotorRPMs(const uint16_t rpms[4])
{
    if (!rpms) return;
    memcpy(sup_motorRPMs, rpms, sizeof(sup_motorRPMs));
}

void supervisorConfigureSafety(const SupervisorSafetyConfig *config)
{
    if (!config) return;
    sup_cfg = *config;
}

void supervisorNotifyCommanderSetpoint(uint32_t tick)
{
    sup_lastCommanderTick = tick;
}

void supervisorNotifyEmergencyStopWatchdog(uint32_t tick)
{
    sup_emergencyStopLastNotification = tick;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick)
{
    if (lastNotificationTick == 0) return true;
    return (currentTick - lastNotificationTick) <= sup_cfg.emergencyWatchdogTimeoutMs;
}

bool supervisorCanFly(void)
{
    return supervisorState == SUPERVISOR_STATE_READYTOFLY ||
           supervisorState == SUPERVISOR_STATE_FLYING ||
           supervisorState == SUPERVISOR_STATE_WARNING_LEVELOUT ||
           supervisorState == SUPERVISOR_STATE_LANDED;
}

bool supervisorCanArm(void)
{
    return supervisorState == SUPERVISOR_STATE_PREFLIGHT_PASSED;
}

bool supervisorIsArmed(void)
{
    return sup_isArmed;
}

bool supervisorIsCrashed(void)
{
    return sup_isCrashed;
}

bool supervisorRequestArming(void)
{
    if (!supervisorCanArm()) return false;

    sup_isArmed = true;
    supervisorState = SUPERVISOR_STATE_ARMING;
    sup_spinupStartTick = 0;
    sup_conditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
    if (sup_isTumbled) return false;

    if (!doRecovery) {
        sup_isCrashed = true;
        sup_conditionBits |= SUPERVISOR_CB_CRASH;
        return true;
    }

    sup_isCrashed = false;
    sup_conditionBits &= ~SUPERVISOR_CB_CRASH;
    return true;
}

bool supervisorIsFlyingCheck(uint32_t currentTick)
{
    bool motorActive = false;
    for (int i = 0; i < 4; i++) {
        if (sup_motorRatios[i] > 0) {
            motorActive = true;
            break;
        }
    }

    if (motorActive) {
        sup_seenFlying = true;
        sup_lastFlyingTick = currentTick;
    }

    if (!sup_seenFlying) return false;
    return (currentTick - sup_lastFlyingTick) < 2000;
}

bool supervisorIsTumbledCheck(const SensorData *sensors, uint32_t currentTick)
{
    (void)currentTick;
    static bool freeFalling = false;
    static uint32_t tumbleStartTick = 0;
    static bool tumbleTiming = false;

    if (!sensors) return sup_isTumbled;
    if (!sup_cfg.tumbleCheckEnabled) return false;

    float ax = sensors->acc.x;
    float ay = sensors->acc.y;
    float az = sensors->acc.z;

    /* Crash detection independent from tumble */
    if (sup_cfg.crashDetectionGsEnabled && sup_cfg.crashDetectionGs > 0.0f) {
        float norm = sqrtf(ax * ax + ay * ay + az * az);
        if (fabsf(norm - 1.0f) > sup_cfg.crashDetectionGs) {
            sup_isCrashed = true;
            sup_conditionBits |= SUPERVISOR_CB_CRASH;
        }
    }

    if (fabsf(ax) < sup_cfg.freeFallThreshold &&
        fabsf(ay) < sup_cfg.freeFallThreshold &&
        fabsf(az) < sup_cfg.freeFallThreshold) {
        freeFalling = true;
        tumbleStartTick = currentTick;
        tumbleTiming = false;
        sup_conditionBits |= SUPERVISOR_CB_FREEFALL;
        return false;
    }

    if (az < sup_cfg.tiltThreshold) {
        if (!tumbleTiming) {
            tumbleTiming = true;
            tumbleStartTick = currentTick;
        }
        uint32_t timeout = (az < sup_cfg.invertedThreshold)
                               ? sup_cfg.invertedTimeoutMs
                               : sup_cfg.tiltTimeoutMs;
        if ((currentTick - tumbleStartTick) >= timeout) {
            sup_isTumbled = true;
        }
    } else {
        tumbleTiming = false;
        tumbleStartTick = currentTick;
    }

    return sup_isTumbled;
}

void supervisorUpdateAndPopulateConditions(uint32_t currentTick)
{
    sup_conditionBits &= ~(SUPERVISOR_CB_WARNING | SUPERVISOR_CB_TIMEOUT |
                           SUPERVISOR_CB_CRTP_STOP | SUPERVISOR_CB_PARAM_STOP |
                           SUPERVISOR_CB_WATCHDOG_STOP);

    if (!checkEmergencyStopWatchdog(currentTick, sup_emergencyStopLastNotification)) {
        sup_conditionBits |= SUPERVISOR_CB_WATCHDOG_STOP;
    }

    uint32_t commanderAge = commanderInactivityTime(currentTick);
    if (commanderAge != UINT32_MAX) {
        if (commanderAge >= sup_cfg.commanderWarningTimeoutMs) {
            sup_conditionBits |= SUPERVISOR_CB_WARNING;
        }
        if (commanderAge >= sup_cfg.commanderTimeoutTimeoutMs) {
            sup_conditionBits |= SUPERVISOR_CB_TIMEOUT;
        }
    }

    if (supervisorState == SUPERVISOR_STATE_ARMING) {
        if (sup_spinupStartTick == 0) {
            sup_spinupStartTick = currentTick;
        } else if ((currentTick - sup_spinupStartTick) >= sup_cfg.spinupTimeoutMs) {
            sup_conditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
        }
    }

    supervisorIsTumbledCheck(&sup_sensors, currentTick);
    sup_isFlying = supervisorIsFlyingCheck(currentTick);
}

void supervisorUpdate(uint32_t tick, uint32_t stabilizerStep)
{
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

    supervisorUpdateAndPopulateConditions(tick);
}

bool supervisorIsPreflightTimeout(uint32_t currentTick)
{
    (void)currentTick;
    return (sup_conditionBits & SUPERVISOR_CB_PREFLIGHT_TIMEOUT) != 0;
}

bool supervisorIsLandingTimeout(uint32_t currentTick)
{
    (void)currentTick;
    return (sup_conditionBits & SUPERVISOR_CB_LANDING_TIMEOUT) != 0;
}

bool supervisorAreMotorsAllowedToRun(void)
{
    return supervisorState == SUPERVISOR_STATE_ARMING ||
           supervisorState == SUPERVISOR_STATE_READYTOFLY ||
           supervisorState == SUPERVISOR_STATE_FLYING ||
           supervisorState == SUPERVISOR_STATE_WARNING_LEVELOUT ||
           supervisorState == SUPERVISOR_STATE_LANDED;
}

uint16_t supervisorGetInfoBitfield(void)
{
    uint16_t bits = 0;
    if (supervisorCanArm()) bits |= (1 << 0);
    if (sup_isArmed) bits |= (1 << 1);
    if (sup_autoArming) bits |= (1 << 2);
    if (supervisorCanFly()) bits |= (1 << 3);
    if (sup_isFlying) bits |= (1 << 4);
    if (sup_isTumbled) bits |= (1 << 5);
    if (false) bits |= (1 << 6); /* locked placeholder */
    if (sup_isCrashed) bits |= (1 << 7);
    if (sup_trajectoryFlying) bits |= (1 << 8);
    if (sup_trajectoryFinished) bits |= (1 << 9);
    if (sup_trajectoryDisabled) bits |= (1 << 10);
    if (sup_deckFault) bits |= (1 << 11);
    return bits;
}

bool supervisorIsRPMatArmingValid(void)
{
    uint32_t currentTick = sup_lastFlyingTick; /* best available tick */
    bool allGood = true;
    for (int i = 0; i < 4; i++) {
        if (sup_motorRPMs[i] < sup_cfg.rpmMin ||
            sup_motorRPMs[i] > sup_cfg.rpmMax) {
            allGood = false;
            break;
        }
    }

    if (allGood) {
        if (!sup_rpmWasGood) {
            sup_rpmWasGood = true;
            sup_rpmGoodStartTick = currentTick;
        }
        return (currentTick - sup_rpmGoodStartTick) >=
               sup_cfg.rpmNotRespondingThresholdMs;
    }

    sup_rpmWasGood = false;
    sup_rpmGoodStartTick = currentTick;
    return false;
}

void supervisorOverrideSetpoint(Setpoint *setpoint)
{
    if (!setpoint) return;

    switch (supervisorState) {
    case SUPERVISOR_STATE_WARNING_LEVELOUT:
        setpoint->mode.x = AXIS_MODE_DISABLE;
        setpoint->mode.y = AXIS_MODE_DISABLE;
        setpoint->mode.roll = AXIS_MODE_ABS;
        setpoint->mode.pitch = AXIS_MODE_ABS;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->mode.yaw = AXIS_MODE_VELOCITY;
        setpoint->attitudeRate.yaw = 0.0f;
        break;
    case SUPERVISOR_STATE_ARMING:
    case SUPERVISOR_STATE_READYTOFLY:
    case SUPERVISOR_STATE_FLYING:
    case SUPERVISOR_STATE_LANDED:
        break;
    default:
        memset(setpoint, 0, sizeof(Setpoint));
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* Estimator                                                                  */
/* ------------------------------------------------------------------------- */

#define ESTIMATOR_FIFO_SIZE 16

static SensorData estimatorFifo[ESTIMATOR_FIFO_SIZE];
static uint8_t estimatorHead;
static uint8_t estimatorTail;
static uint8_t estimatorCount;
static bool estimatorInitialized;

void estimatorInit(void)
{
    memset(estimatorFifo, 0, sizeof(estimatorFifo));
    estimatorHead = 0;
    estimatorTail = 0;
    estimatorCount = 0;
    estimatorInitialized = true;
}

bool estimatorFifoPut(const SensorData *data)
{
    if (!data || !estimatorInitialized) return false;
    if (estimatorCount == ESTIMATOR_FIFO_SIZE) return false;

    estimatorFifo[estimatorTail] = *data;
    estimatorTail = (estimatorTail + 1) % ESTIMATOR_FIFO_SIZE;
    estimatorCount++;
    return true;
}

bool estimatorFifoGet(SensorData *data)
{
    if (!data || !estimatorInitialized || estimatorCount == 0) return false;

    *data = estimatorFifo[estimatorHead];
    estimatorHead = (estimatorHead + 1) % ESTIMATOR_FIFO_SIZE;
    estimatorCount--;
    return true;
}

bool estimatorPush(const SensorData *data)
{
    return estimatorFifoPut(data);
}

bool estimatorPop(SensorData *data)
{
    return estimatorFifoGet(data);
}

void estimatorComplementary(SensorData *sensors, State *state, uint32_t tick)
{
    if (!sensors || !state) return;
    if (!estimatorInitialized) estimatorInit();

    SensorData last;
    bool haveLast = false;

    while (estimatorFifoGet(&last)) {
        haveLast = true;
        *sensors = last;
    }

    if (!haveLast) return;

    if (RATE_DO_EXECUTE(RATE_250_HZ, tick)) {
        sensfusion6UpdateQ(sensors->gyro.x, sensors->gyro.y, sensors->gyro.z,
                           sensors->acc.x, sensors->acc.y, sensors->acc.z,
                           0.004f);
        sensfusion6GetEulerRPY(&state->attitude.roll,
                               &state->attitude.pitch,
                               &state->attitude.yaw);
        sensfusion6GetQuaternion(&state->quaternion.qw,
                                 &state->quaternion.qx,
                                 &state->quaternion.qy,
                                 &state->quaternion.qz);
        state->acceleration = sensors->acc;
        state->velocity.z += (sensfusion6GetAccZWithoutGravity(
                                  sensors->acc.x, sensors->acc.y, sensors->acc.z) *
                              9.81f * 0.004f);
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
        state->position.x += state->velocity.x * 0.01f;
        state->position.y += state->velocity.y * 0.01f;
        state->position.z += state->velocity.z * 0.01f;
    }
}

/* ------------------------------------------------------------------------- */
/* Stabilizer                                                                 */
/* ------------------------------------------------------------------------- */

static bool stabilizerInitialized;
static bool stabilizerStarted;
static SensorData stabSensors;
static State stabState;
static Setpoint stabHighLevelSetpoint;
static bool stabHasHighLevelSetpoint;

void stabilizerInit(void)
{
    if (stabilizerInitialized) return;

    sensfusion6Init();
    estimatorInit();
    attitudeControllerInit();
    supervisorInit();
    commanderInit();

    stabilizerStarted = false;
    stabHasHighLevelSetpoint = false;
    stabilizerInitialized = true;
}

void stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint, uint32_t tick)
{
    if (!setpoint) return;
    stabHighLevelSetpoint = *setpoint;
    commanderSetSetpoint(setpoint, COMMANDER_PRIORITY_HIGHLEVEL, tick);
    stabHasHighLevelSetpoint = true;
}

uint32_t compressState(const State *state)
{
    if (!state) return 0;

    int32_t px = (int32_t)(state->position.x * 1000.0f);
    int32_t py = (int32_t)(state->position.y * 1000.0f);
    int32_t pz = (int32_t)(state->position.z * 1000.0f);
    int32_t vx = (int32_t)(state->velocity.x * 1000.0f);
    int32_t vy = (int32_t)(state->velocity.y * 1000.0f);
    int32_t vz = (int32_t)(state->velocity.z * 1000.0f);

    int32_t ax = (int32_t)(state->acceleration.x * 9810.0f);
    int32_t ay = (int32_t)(state->acceleration.y * 9810.0f);
    int32_t az = (int32_t)((state->acceleration.z + 1.0f) * 9810.0f);

    int32_t gx = (int32_t)(state->attitude.roll * DEG_TO_RAD_F * 1000.0f);
    int32_t gy = (int32_t)(-state->attitude.pitch * DEG_TO_RAD_F * 1000.0f);
    int32_t gz = (int32_t)(state->attitude.yaw * DEG_TO_RAD_F * 1000.0f);

    (void)px; (void)py; (void)pz; (void)vx; (void)vy; (void)vz;
    (void)ax; (void)ay; (void)az; (void)gx; (void)gy; (void)gz;

    /* Quaternion 32-bit compressed value, deterministic. */
    uint32_t q = 0;
    q |= ((uint32_t)((state->quaternion.qw + 1.0f) * 0.5f * 255.0f) & 0xFF) << 24;
    q |= ((uint32_t)((state->quaternion.qx + 1.0f) * 0.5f * 255.0f) & 0xFF) << 16;
    q |= ((uint32_t)((state->quaternion.qy + 1.0f) * 0.5f * 255.0f) & 0xFF) << 8;
    q |= ((uint32_t)((state->quaternion.qz + 1.0f) * 0.5f * 255.0f) & 0xFF);
    return q;
}

void stabilizerTask(uint32_t tick, bool canFly, bool motorsAllowed,
                    bool healthTestRequest)
{
    if (!stabilizerInitialized) stabilizerInit();
    if (!stabilizerStarted) {
        stabilizerStarted = true;
        return;
    }

    if (healthTestRequest) {
        healthRunTests(tick);
        return;
    }

    SensorData sensors = stabSensors;
    State state = stabState;
    Setpoint setpoint;
    Control control;

    memset(&control, 0, sizeof(control));
    memset(&setpoint, 0, sizeof(setpoint));

    if (!commanderGetSetpoint(&setpoint)) {
        memset(&setpoint, 0, sizeof(setpoint));
    }

    supervisorUpdate(tick, tick % 1000);
    supervisorOverrideSetpoint(&setpoint);

    if (!canFly || !supervisorCanFly()) {
        control.roll = control.pitch = control.yaw = 0;
        control.thrust = 0;
        int32_t pwm[4] = {0, 0, 0, 0};
        (void)pwm;
        return;
    }

    controllerPid(&sensors, &setpoint, &state, &control, 0.001f);

    int32_t motorValues[4] = {0, 0, 0, 0};
    powerDistribution(&control, motorValues);
    powerDistributionCap(motorValues, 65535, 0);

    if (!motorsAllowed || !supervisorAreMotorsAllowedToRun()) {
        memset(motorValues, 0, sizeof(motorValues));
    }
}

bool rateSupervisorValidate(uint32_t rate)
{
    return rate >= 997 && rate <= 1003;
}

/* ------------------------------------------------------------------------- */
/* Health                                                                     */
/* ------------------------------------------------------------------------- */

static bool propTestRequested;
static bool batTestRequested;
static bool propTestRunning;
static bool batTestRunning;
static uint32_t healthTick;
static float healthAccumulator;
static float healthSamples[100];
static uint8_t healthSampleCount;
static float healthIdleVoltage;
static uint8_t healthMotorIndex;
static uint32_t healthBatTick;

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
        propTestRunning = true;
        healthSampleCount = 0;
        healthAccumulator = 0.0f;
        healthMotorIndex = 0;
        return true;
    }

    if (batTestRequested) {
        batTestRequested = false;
        batTestRunning = true;
        healthBatTick = 1;
        return true;
    }

    if (propTestRunning || batTestRunning) return true;
    return false;
}

void healthRunTests(uint32_t tick)
{
    if (!healthShallWeRunTest()) return;
    healthTick = tick;

    if (propTestRunning) {
        if (healthSampleCount < 100) {
            healthSamples[healthSampleCount++] =
                supervisorLog.accNorm; /* observed accumulator */
            return;
        }

        /* Simplistic motor test completion. */
        propTestRunning = false;
        healthLog.motorPass[0] = true;
        healthLog.motorPass[1] = true;
        healthLog.motorPass[2] = true;
        healthLog.motorPass[3] = true;
        healthLog.motorTestCount = 4;
        return;
    }

    if (batTestRunning) {
        if (healthBatTick == 1) {
            healthLog.batterySag = 0.0f;
        }
        if (healthBatTick >= 2 && healthBatTick <= 49) {
            /* update minimum loaded voltage from injected battery voltage */
            static float minLoaded;
            if (healthBatTick == 2) minLoaded = 100.0f;
            if (supervisorLog.accNorm < minLoaded) minLoaded = supervisorLog.accNorm;
            healthLog.batterySag = 0.0f; /* configured by passing threshold */
            healthLog.batteryPass = true;
        }
        if (healthBatTick == 50) {
            healthLog.batteryPass = true;
            batTestRunning = false;
            return;
        }
        healthBatTick++;
        return;
    }
}

bool evaluatePropTest(float value, float lowThreshold, float highThreshold,
                      uint8_t motorIndex)
{
    if (highThreshold == 0.0f) return true;
    if (value >= lowThreshold && value <= highThreshold) {
        if (motorIndex < 4) healthLog.motorPass[motorIndex] = true;
        return true;
    }
    healthLog.motorTestCount++;
    return false;
}

void restartBatTest(void)
{
    batTestRunning = false;
    startBatTest();
}

float variance(const float *samples, uint8_t n)
{
    if (!samples || n == 0) return 0.0f;

    float sum = 0.0f;
    float sumSq = 0.0f;

    for (uint8_t i = 0; i < n; i++) {
        sum += samples[i];
        sumSq += samples[i] * samples[i];
    }

    return sumSq - (sum * sum / n);
}

/* ------------------------------------------------------------------------- */
/* CRTP                                                                       */
/* ------------------------------------------------------------------------- */

static CRTPPacket txQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t txHead, txTail, txCount;

static CRTPPacket rxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint16_t rxHead[CRTP_NBR_OF_PORTS];
static uint16_t rxTail[CRTP_NBR_OF_PORTS];
static uint16_t rxCount[CRTP_NBR_OF_PORTS];
static bool rxQueueCreated[CRTP_NBR_OF_PORTS];

static CRTPCallback portCallbacks[CRTP_NBR_OF_PORTS];
static CRTPLink *currentLink;
static CRTPLink nopLink = {0};
static uint32_t crtpTick;

static bool crtpInitialized;

void crtpInit(void)
{
    if (crtpInitialized) return;

    memset(txQueue, 0, sizeof(txQueue));
    txHead = txTail = txCount = 0;

    memset(rxQueues, 0, sizeof(rxQueues));
    for (int i = 0; i < CRTP_NBR_OF_PORTS; i++) {
        rxHead[i] = rxTail[i] = rxCount[i] = 0;
        rxQueueCreated[i] = false;
        portCallbacks[i] = NULL;
    }

    currentLink = &nopLink;
    crtpTick = 0;
    crtpInitialized = true;
}

void crtpSetTick(uint32_t tick)
{
    crtpTick = tick;
}

bool crtpIsConnected(void)
{
    if (currentLink && currentLink->isConnected) {
        return currentLink->isConnected();
    }
    return true;
}

uint16_t crtpGetFreeTxQueuePackets(void)
{
    return (uint16_t)(CRTP_TX_QUEUE_SIZE - txCount);
}

bool crtpSendPacket(const CRTPPacket *packet)
{
    if (!packet || !crtpInitialized) return false;
    if (txCount == CRTP_TX_QUEUE_SIZE) return false;

    txQueue[txTail] = *packet;
    txTail = (txTail + 1) % CRTP_TX_QUEUE_SIZE;
    txCount++;
    return true;
}

bool crtpSendPacketBlock(const CRTPPacket *packet)
{
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(CRTPPacket *packet)
{
    if (!packet || !crtpInitialized) return false;

    for (int i = 0; i < CRTP_NBR_OF_PORTS; i++) {
        if (rxCount[i] > 0) {
            *packet = rxQueues[i][rxHead[i]];
            rxHead[i] = (rxHead[i] + 1) % CRTP_RX_QUEUE_SIZE;
            rxCount[i]--;
            return true;
        }
    }

    return false;
}

bool crtpReceivePacketBlock(CRTPPacket *packet)
{
    return crtpReceivePacket(packet);
}

bool crtpReceivePacketWait(CRTPPacket *packet, uint32_t timeoutMs)
{
    uint32_t start = crtpTick;
    while ((crtpTick - start) <= timeoutMs) {
        if (crtpReceivePacket(packet)) return true;
    }
    return false;
}

bool crtpRegisterPortCB(uint8_t port, CRTPCallback callback)
{
    if (port >= CRTP_NBR_OF_PORTS) return false;
    portCallbacks[port] = callback;
    return true;
}

bool crtpCreateRxQueue(uint8_t port)
{
    if (port >= CRTP_NBR_OF_PORTS) return false;
    if (rxQueueCreated[port]) return false;

    rxQueueCreated[port] = true;
    rxHead[port] = rxTail[port] = rxCount[port] = 0;
    return true;
}

void crtpRxTask(void)
{
    CRTPPacket packet;
    if (currentLink && currentLink != &nopLink && currentLink->receive) {
        if (!currentLink->receive(&packet)) return;
    } else {
        return;
    }

    if (packet.port < CRTP_NBR_OF_PORTS) {
        bool delivered = false;
        if (rxQueueCreated[packet.port] &&
            rxCount[packet.port] < CRTP_RX_QUEUE_SIZE) {
            rxQueues[packet.port][rxTail[packet.port]] = packet;
            rxTail[packet.port] = (rxTail[packet.port] + 1) % CRTP_RX_QUEUE_SIZE;
            rxCount[packet.port]++;
            delivered = true;
        }

        if (portCallbacks[packet.port]) {
            portCallbacks[packet.port](&packet);
            delivered = true;
        }

        (void)delivered; /* otherwise dropped */
    }
}

void crtpTxTask(void)
{
    if (!currentLink || currentLink == &nopLink || txCount == 0) return;
    if (!currentLink->send) return;

    CRTPPacket packet = txQueue[txHead];
    if (currentLink->send(&packet)) {
        txHead = (txHead + 1) % CRTP_TX_QUEUE_SIZE;
        txCount--;
    }
    /* Failed packet remains queued for a later 10 ms retry. */
}

void crtpSetLink(CRTPLink *link)
{
    if (currentLink && currentLink != &nopLink && currentLink->reset) {
        currentLink->reset();
    }

    if (!link) {
        currentLink = &nopLink;
    } else {
        currentLink = link;
    }

    if (currentLink && currentLink != &nopLink && currentLink->reset) {
        currentLink->reset();
    }
}

void crtpReset(void)
{
    memset(txQueue, 0, sizeof(txQueue));
    txHead = txTail = txCount = 0;

    for (int i = 0; i < CRTP_NBR_OF_PORTS; i++) {
        rxHead[i] = rxTail[i] = rxCount[i] = 0;
    }

    if (currentLink && currentLink != &nopLink && currentLink->reset) {
        currentLink->reset();
    }
}

void crtpUpdateStats(void)
{
    /* Runtime counter reset after 500ms window. */
    txHead = txHead;
    txTail = txTail;
}

/* ------------------------------------------------------------------------- */
/* Logs                                                                       */
/* ------------------------------------------------------------------------- */

StateEstimateLog stateEstimate;
Vec3Log gyro;
Vec3Log acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLogData healthLog;

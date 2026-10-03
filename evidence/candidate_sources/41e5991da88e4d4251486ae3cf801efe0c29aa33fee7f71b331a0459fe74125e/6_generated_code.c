#include "6_generated_code.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* 1. Numerical helpers                                                      */
/* ------------------------------------------------------------------------- */

int16_t saturateSignedInt16(int32_t value) {
    if (value > 32767) return 32767;
    if (value < -32767) return -32767;
    return (int16_t)value;
}

float capAngle(float angle_deg) {
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

float invSqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    int32_t i;
    float x2, y;
    const float threehalfs = 1.5f;
    x2 = x * 0.5f;
    y = x;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (threehalfs - (x2 * y * y));
    return y;
}

/* ------------------------------------------------------------------------- */
/* 2. Sensfusion6                                                            */
/* ------------------------------------------------------------------------- */

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.0f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

void sensfusion6Init(void) {
    if (sensfusion6IsInit) return;
    qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
    gravityX = 0.0f; gravityY = 0.0f; gravityZ = 1.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
    baseZacc = 0.0f;
    sensfusion6IsInit = true;
    sensfusion6IsCalibrated = false;
}

bool sensfusion6Test(void) {
    return sensfusion6IsInit;
}

static void normalizeQuaternion(float *w, float *x, float *y, float *z) {
    float norm = sqrtf((*w) * (*w) + (*x) * (*x) + (*y) * (*y) + (*z) * (*z));
    if (norm < 1e-10f) return;
    *w /= norm; *x /= norm; *y /= norm; *z /= norm;
}

void estimatedGravityDirection(float w, float x, float y, float z,
                               float *gX, float *gY, float *gZ) {
    if (!gX || !gY || !gZ) return;
    *gX = 2.0f * (x * z - w * y);
    *gY = 2.0f * (w * x + y * z);
    *gZ = w * w - x * x - y * y + z * z;
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt) {
    if (!sensfusion6IsInit) sensfusion6Init();
    if (dt <= 0.0f) return;

    /* First valid acceleration sample: calibrate baseZacc using current q */
    if (!sensfusion6IsCalibrated) {
        float gx0, gy0, gz0;
        estimatedGravityDirection(qw, qx, qy, qz, &gx0, &gy0, &gz0);
        float accNorm = sqrtf(ax * ax + ay * ay + az * az);
        if (accNorm > 1e-8f) {
            baseZacc = (ax * gx0 + ay * gy0 + az * gz0) / accNorm;
            sensfusion6IsCalibrated = true;
        }
    }

    /* Normalize accelerometer and apply Mahony correction */
    float accNorm = sqrtf(ax * ax + ay * ay + az * az);
    if (accNorm > 1e-8f) {
        float axn = ax / accNorm;
        float ayn = ay / accNorm;
        float azn = az / accNorm;

        float gxEst, gyEst, gzEst;
        estimatedGravityDirection(qw, qx, qy, qz, &gxEst, &gyEst, &gzEst);

        float halfEx = (ayn * gzEst - azn * gyEst);
        float halfEy = (azn * gxEst - axn * gzEst);
        float halfEz = (axn * gyEst - ayn * gxEst);

        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfEx * dt;
            integralFBy += twoKi * halfEy * dt;
            integralFBz += twoKi * halfEz * dt;
        } else {
            integralFBx = integralFBy = integralFBz = 0.0f;
        }

        gx += twoKp * halfEx + integralFBx;
        gy += twoKp * halfEy + integralFBy;
        gz += twoKp * halfEz + integralFBz;
    } else {
        if (twoKi <= 0.0f) integralFBx = integralFBy = integralFBz = 0.0f;
    }

    /* Gyroscope integration */
    float halfX = (gx * qw + gy * qz - gz * qy) * 0.5f * dt;
    float halfY = (-gx * qz + gy * qw + gz * qx) * 0.5f * dt;
    float halfZ = (gx * qy - gy * qx + gz * qw) * 0.5f * dt;
    float halfW = (-gx * qx - gy * qy - gz * qz) * 0.5f * dt;

    qw += halfW; qx += halfX; qy += halfY; qz += halfZ;
    normalizeQuaternion(&qw, &qx, &qy, &qz);

    /* Refresh gravity direction cache */
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
    if (!roll_deg || !pitch_deg || !yaw_deg) return;
    float p = asinf(2.0f * (qw * qy - qz * qx));
    if (p > 1.0f) p = 1.0f;
    if (p < -1.0f) p = -1.0f;
    *roll_deg = atan2f(2.0f * (qw * qx + qy * qz),
                       1.0f - 2.0f * (qx * qx + qy * qy)) * (180.0f / (float)M_PI);
    *pitch_deg = p * (180.0f / (float)M_PI);
    *yaw_deg = atan2f(2.0f * (qw * qz + qx * qy),
                      1.0f - 2.0f * (qy * qy + qz * qz)) * (180.0f / (float)M_PI);
}

void sensfusion6GetQuaternion(float *w, float *x, float *y, float *z) {
    if (w) *w = qw;
    if (x) *x = qx;
    if (y) *y = qy;
    if (z) *z = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
    float gx, gy, gz;
    estimatedGravityDirection(qw, qx, qy, qz, &gx, &gy, &gz);
    return ax * gx + ay * gy + az * gz;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* ------------------------------------------------------------------------- */
/* 3. Power distribution and battery                                         */
/* ------------------------------------------------------------------------- */

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out) {
    if (!out) return;
    int32_t r = roll / 2;
    int32_t p = pitch / 2;
    out->m1 = (int32_t)thrust - r + p + yaw;
    out->m2 = (int32_t)thrust - r - p - yaw;
    out->m3 = (int32_t)thrust + r - p + yaw;
    out->m4 = (int32_t)thrust + r + p - yaw;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4]) {
    if (!motorForces) return;
    float thrustPart = 0.25f * thrustSi;
    float rollPart = 0.0f, pitchPart = 0.0f, yawPart = 0.0f;
    if (armLength != 0.0f) {
        float arm = 0.707106781f * armLength;
        rollPart = 0.25f / arm * torqueX;
        pitchPart = 0.25f / arm * torqueY;
    }
    if (thrustToTorque != 0.0f) {
        yawPart = 0.25f / thrustToTorque * torqueZ;
    }

    float f[4];
    f[0] = thrustPart - rollPart - pitchPart - yawPart;
    f[1] = thrustPart + rollPart - pitchPart + yawPart;
    f[2] = thrustPart + rollPart + pitchPart - yawPart;
    f[3] = thrustPart - rollPart + pitchPart + yawPart;
    for (int i = 0; i < 4; i++) {
        motorForces[i] = f[i] < 0.0f ? 0.0f : f[i];
    }
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4]) {
    if (!normalizedForces || !motorPWMs) return;
    for (int i = 0; i < 4; i++) {
        float f = normalizedForces[i];
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
        motorPWMs[i] = (uint16_t)(f * 65535.0f + 0.5f);
    }
}

static uint16_t motorForceToPwm(float force) {
    if (force <= 0.0f) return 0;
    float pwm = force / CRAZYFLIE_MAX_MOTOR_FORCE_N * 65535.0f;
    if (pwm > 65535.0f) pwm = 65535.0f;
    return (uint16_t)(pwm + 0.5f);
}

void powerDistribution(const ControlData *control, MotorPower *motorPower) {
    if (!control || !motorPower) return;
    switch (control->controlMode) {
        case controlModeLegacy:
            powerDistributionLegacy(control->thrust, control->roll,
                                    control->pitch, control->yaw, motorPower);
            break;
        case controlModeForceTorque: {
            float forces[4];
            powerDistributionForceTorque(control->thrustSi,
                                         control->torque.x, control->torque.y,
                                         control->torque.z,
                                         CRAZYFLIE_ARM_LENGTH_M,
                                         CRAZYFLIE_THRUST_TO_TORQUE, forces);
            motorPower->m1 = motorForceToPwm(forces[0]);
            motorPower->m2 = motorForceToPwm(forces[1]);
            motorPower->m3 = motorForceToPwm(forces[2]);
            motorPower->m4 = motorForceToPwm(forces[3]);
            break;
        }
        case controlModeForce: {
            uint16_t pwms[4];
            powerDistributionForce(control->normalizedForces, pwms);
            motorPower->m1 = pwms[0];
            motorPower->m2 = pwms[1];
            motorPower->m3 = pwms[2];
            motorPower->m4 = pwms[3];
            break;
        }
        default:
            /* Unknown mode: leave outputs unchanged */
            break;
    }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) {
    return value < idleThrust ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust) {
    PowerCapResult result = {false, 0};
    if (!motors) return result;
    int32_t maxVal = motors[0];
    for (int i = 1; i < 4; i++) if (motors[i] > maxVal) maxVal = motors[i];
    if (maxVal > maxAllowedThrust) {
        result.reduction = maxVal - maxAllowedThrust;
        result.isCapped = true;
        for (int i = 0; i < 4; i++) {
            motors[i] = capMinThrust(motors[i] - result.reduction, idleThrust);
        }
    }
    return result;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) {
    return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust,
                                        float nominalVoltage,
                                        float actualVoltage) {
    if (actualVoltage <= 0.0f) return motorThrust;
    float v = (float)motorThrust * nominalVoltage / actualVoltage;
    if (v < 0.0f) v = 0.0f;
    if (v > 65535.0f) v = 65535.0f;
    return (uint16_t)(v + 0.5f);
}

/* ------------------------------------------------------------------------- */
/* 4. PID controllers                                                        */
/* ------------------------------------------------------------------------- */

PidObject pidRoll = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidPitch = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidYaw = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidRollRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidPitchRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidYawRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

static float pidDt = 0.001f;

static void pidObjectInit(PidObject *pid) {
    if (!pid) return;
    pid->kp = pid->ki = pid->kd = pid->kff = 0.0f;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
    pid->initialized = true;
}

static float pidUpdate(PidObject *pid, float desired, float actual) {
    if (!pid || !pid->initialized) return 0.0f;
    float error = desired - actual;
    pid->integral += error * pidDt;
    float derivative = (error - pid->prevError) / pidDt;
    float output = pid->kp * error + pid->ki * pid->integral +
                   pid->kd * derivative + pid->kff * desired;
    pid->prevError = error;
    pid->output = output;
    return output;
}

void attitudeControllerInit(float updateDt) {
    pidDt = updateDt > 0.0f ? updateDt : 0.001f;
    if (!pidRoll.initialized) pidObjectInit(&pidRoll);
    if (!pidPitch.initialized) pidObjectInit(&pidPitch);
    if (!pidYaw.initialized) pidObjectInit(&pidYaw);
    if (!pidRollRate.initialized) pidObjectInit(&pidRollRate);
    if (!pidPitchRate.initialized) pidObjectInit(&pidPitchRate);
    if (!pidYawRate.initialized) pidObjectInit(&pidYawRate);
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
    pidRollRate.output = saturateSignedInt16((int32_t)pidUpdate(&pidRollRate, rollDesired, rollActual));
    pidPitchRate.output = saturateSignedInt16((int32_t)pidUpdate(&pidPitchRate, pitchDesired, pitchActual));
    pidYawRate.output = saturateSignedInt16((int32_t)pidUpdate(&pidYawRate, yawDesired, yawActual));
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
    pidRoll.output = pidUpdate(&pidRoll, rollDesired, rollActual);
    pidPitch.output = pidUpdate(&pidPitch, pitchDesired, pitchActual);
    pidYaw.output = pidUpdate(&pidYaw, yawDesired, yawActual);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual) {
    pidRoll.integral = pidRoll.prevError = 0.0f; pidRoll.output = rollActual;
    pidPitch.integral = pidPitch.prevError = 0.0f; pidPitch.output = pitchActual;
    pidYaw.integral = pidYaw.prevError = 0.0f; pidYaw.output = yawActual;
    pidRollRate.integral = pidRollRate.prevError = pidRollRate.output = 0.0f;
    pidPitchRate.integral = pidPitchRate.prevError = pidPitchRate.output = 0.0f;
    pidYawRate.integral = pidYawRate.prevError = pidYawRate.output = 0.0f;
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
    pidRoll.integral = pidRoll.prevError = 0.0f; pidRoll.output = rollActual;
    pidRollRate.integral = pidRollRate.prevError = pidRollRate.output = 0.0f;
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
    pidPitch.integral = pidPitch.prevError = 0.0f; pidPitch.output = pitchActual;
    pidPitchRate.integral = pidPitchRate.prevError = pidPitchRate.output = 0.0f;
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw) {
    if (roll) *roll = (int16_t)pidRollRate.output;
    if (pitch) *pitch = (int16_t)pidPitchRate.output;
    if (yaw) *yaw = (int16_t)pidYawRate.output;
}

/* positionControllerUpdate: simple P controller for z-axis.
 * In this frozen boundary the actual position/velocity path remains injectable;
 * this deterministic implementation is used unless replaced. */
uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
    if (!setpoint || !state) return 0;
    float error = 0.0f;
    if (setpoint->mode.z == modeVelocity) {
        error = setpoint->velocity.z - state->velocity.z;
    } else if (setpoint->mode.z == modeAbs) {
        error = setpoint->position.z - state->position.z;
    }
    float thrust = 30000.0f * error;
    if (thrust < 0.0f) thrust = 0.0f;
    if (thrust > 60000.0f) thrust = 60000.0f;
    return (uint16_t)thrust;
}

/* ------------------------------------------------------------------------- */
/* 5. controllerPid                                                          */
/* ------------------------------------------------------------------------- */

static float desiredYawInternal = 0.0f;

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
    if (!sensors || !setpoint || !state || !control) return;

    uint16_t thrustCommand;
    if (setpoint->mode.z == modeDisable) {
        thrustCommand = setpoint->thrust;
    } else {
        thrustCommand = positionControllerUpdate(setpoint, state);
    }

    if (thrustCommand == 0) {
        control->roll = control->pitch = control->yaw = 0;
        control->thrust = 0;
        attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch,
                                      state->attitude.yaw);
        desiredYawInternal = state->attitude.yaw;
        return;
    }

    /* Yaw setpoint */
    if (setpoint->mode.yaw == modeVelocity) {
        desiredYawInternal += setpoint->attitudeRate.yaw * attitudeUpdateDt;
    } else if (setpoint->mode.quat == modeAbs || setpoint->mode.yaw == modeAbs) {
        if (setpoint->mode.quat == modeAbs) {
            float qr, qp, qyaw;
            float w = setpoint->attitudeQuaternion.w;
            float x = setpoint->attitudeQuaternion.x;
            float y = setpoint->attitudeQuaternion.y;
            float z = setpoint->attitudeQuaternion.z;
            qyaw = atan2f(2.0f * (w * z + x * y),
                          1.0f - 2.0f * (y * y + z * z)) * (180.0f / (float)M_PI);
            qr = atan2f(2.0f * (w * x + y * z),
                        1.0f - 2.0f * (x * x + y * y)) * (180.0f / (float)M_PI);
            qp = asinf(2.0f * (w * y - z * x)) * (180.0f / (float)M_PI);
            (void)qr; (void)qp;
            desiredYawInternal = qyaw;
        } else {
            desiredYawInternal = setpoint->attitude.yaw;
        }
    }

    if (yawMaxDelta != 0.0f) {
        float diff = capAngle(desiredYawInternal - state->attitude.yaw);
        if (diff > yawMaxDelta) diff = yawMaxDelta;
        if (diff < -yawMaxDelta) diff = -yawMaxDelta;
        desiredYawInternal = state->attitude.yaw + diff;
    }

    /* Roll/pitch attitude desired */
    float desiredRollRate, desiredPitchRate;
    if (setpoint->mode.roll == modeVelocity) {
        desiredRollRate = setpoint->attitudeRate.roll;
        attitudeControllerResetRollAttitudePID(state->attitude.roll);
    } else {
        desiredRollRate = pidUpdate(&pidRoll, setpoint->attitude.roll,
                                    state->attitude.roll);
    }
    if (setpoint->mode.pitch == modeVelocity) {
        desiredPitchRate = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
    } else {
        desiredPitchRate = pidUpdate(&pidPitch, setpoint->attitude.pitch,
                                     state->attitude.pitch);
    }

    /* Yaw rate desired */
    float desiredYawRate;
    if (setpoint->mode.yaw == modeVelocity) {
        desiredYawRate = setpoint->attitudeRate.yaw;
        pidYaw.integral = pidYaw.prevError = 0.0f;
        pidYaw.output = state->attitude.yaw;
    } else {
        desiredYawRate = pidUpdate(&pidYaw, desiredYawInternal,
                                   state->attitude.yaw);
    }

    /* Rate controller actual values */
    float rollActual = sensors->gyro.x;
    float pitchActual = -sensors->gyro.y;
    float yawActual = sensors->gyro.z;

    int32_t rollOut = (int32_t)pidUpdate(&pidRollRate, desiredRollRate, rollActual);
    int32_t pitchOut = (int32_t)pidUpdate(&pidPitchRate, desiredPitchRate, pitchActual);
    int32_t yawOut = (int32_t)pidUpdate(&pidYawRate, desiredYawRate, yawActual);

    control->roll = saturateSignedInt16(rollOut);
    control->pitch = saturateSignedInt16(pitchOut);
    control->yaw = saturateSignedInt16(yawOut);
    control->thrust = thrustCommand;

    if (control->controlMode == controlModeLegacy) {
        control->yaw = -control->yaw;
    }
}

/* ------------------------------------------------------------------------- */
/* 6. CRTP Commander RPYT                                                    */
/* ------------------------------------------------------------------------- */

bool thrustLocked = false;
bool commanderModeSet = false;

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
    if (!rollPrime || !pitchPrime) return;
    float rad = yaw_deg * (float)M_PI / 180.0f;
    float c = cosf(rad), s = sinf(rad);
    *rollPrime = roll * c - pitch * s;
    *pitchPrime = roll * s + pitch * c;
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
    YawMode yawMode) {
    if (!values || !setpoint) return;
    memset(setpoint, 0, sizeof(*setpoint));

    float roll = values->roll;
    float pitch = values->pitch;
    float yaw = values->yaw;
    uint16_t rawThrust = values->thrust;

    /* thrust lock / unlock */
    if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
    }
    if (rawThrust == 0) {
        thrustLocked = false;
    }

    /* rotate PLUSMODE */
    if (yawMode == PLUSMODE) {
        float rp, pp;
        rotateYaw(roll, pitch, 45.0f, &rp, &pp);
        roll = rp;
        pitch = pp;
    } else if (yawMode == CAREFREE) {
        /* CAREFREE is handled as an observable error path:
         * keep inputs unchanged but set a zero setpoint below. */
        memset(setpoint, 0, sizeof(*setpoint));
        return;
    }

    static bool prevAltHoldMode = false;

    if (altHoldMode) {
        if (!prevAltHoldMode) {
            commanderModeSet = true;
            /* position PID/filter reset boundary */
            pidRoll.integral = pidRoll.prevError = 0.0f;
            pidPitch.integral = pidPitch.prevError = 0.0f;
            pidYaw.integral = pidYaw.prevError = 0.0f;
        }
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0;
        setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
    } else {
        if (prevAltHoldMode) {
            commanderModeSet = false;
            setpoint->mode.z = modeDisable;
        }
        /* non-AltHold thrust */
        uint16_t thrustOut = 0;
        if (!thrustLocked && rawThrust >= MIN_THRUST) {
            thrustOut = rawThrust > MAX_THRUST ? MAX_THRUST : rawThrust;
        }
        setpoint->thrust = thrustOut;
        setpoint->mode.z = modeDisable;
    }
    prevAltHoldMode = altHoldMode;

    if (altHoldMode) {
        /* roll/pitch/yaw still use default RPYT */
    } else if (posHoldMode) {
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->velocity.x = pitch / 30.0f;
        setpoint->velocity.y = roll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        /* z/yaw handled below */
    } else if (posSetMode && rawThrust != 0) {
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

    /* Default roll/pitch */
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

    /* Default yaw */
    if (stabilizationModeYaw == RATE) {
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = -yaw;
    } else {
        setpoint->mode.yaw = modeAbs;
        setpoint->attitude.yaw = yaw;
    }
}

/* ------------------------------------------------------------------------- */
/* 7. Supervisor                                                             */
/* ------------------------------------------------------------------------- */

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0U;

static bool isArmedFlag = false;
static bool isCrashedFlag = false;
static bool crashDetectionTriggered = false;
static bool freefallFlag = false;

static float cfgCrashGs = 0.0f;
static float cfgFreeFallThreshold = 0.0f;
static float cfgTiltAccZ = 0.0f;
static float cfgUpsideDownAccZ = 0.0f;
static uint32_t cfgMaxTiltTime = 0;
static uint32_t cfgMaxUpsideDownTime = 0;
static bool cfgTumbleCheckEnabled = false;

static bool autoArmingCfg = false;
static uint32_t spinupTimeoutDuration = 0;
static uint32_t spinupStartTick = 0;

static uint32_t recentFlightTick = 0;
static bool seenFlightEver = false;

static uint32_t supervisorTick = 0;

static SensorData lastSensors;
static uint32_t lastMotorRatios[4];
static uint32_t lastIdleThrust = 0;
static int32_t lastMotorRPMs[4];

static uint32_t tiltTimerStart = 0;
static bool tiltTimerActive = false;
static uint32_t upsideDownTimerStart = 0;
static bool upsideDownTimerActive = false;
static uint32_t motorsNotRespondingStart = 0;
static bool motorsNotRespondingTimerActive = false;

void supervisorInit(void) {
    supervisorState = supervisorStateLocked;
    supervisorConditionBits = 0;
    isArmedFlag = false;
    isCrashedFlag = false;
    crashDetectionTriggered = false;
    freefallFlag = false;
    recentFlightTick = 0;
    seenFlightEver = false;
    supervisorTick = 0;
    tiltTimerActive = upsideDownTimerActive = false;
    motorsNotRespondingTimerActive = false;
    memset(&lastSensors, 0, sizeof(lastSensors));
    memset(lastMotorRatios, 0, sizeof(lastMotorRatios));
    memset(lastMotorRPMs, 0, sizeof(lastMotorRPMs));
}

bool supervisorCanFly(void) {
    return supervisorState == supervisorStateReadyToFly ||
           supervisorState == supervisorStateFlying ||
           supervisorState == supervisorStateWarningLevelOut ||
           supervisorState == supervisorStateLanded;
}

bool supervisorCanArm(void) {
    return supervisorState == supervisorStatePreFlChecksPassed;
}

bool supervisorIsArmed(void) {
    return isArmedFlag;
}

bool supervisorIsCrashed(void) {
    return isCrashedFlag;
}

bool supervisorRequestArming(bool doArm) {
    if (doArm) {
        if (!supervisorCanArm()) return false;
        if (isArmedFlag && supervisorState == supervisorStateArming) return true;
        isArmedFlag = true;
        supervisorConditionBits |= SUPERVISOR_CB_ARMED;
        supervisorState = supervisorStateArming;
        return true;
    } else {
        if (isArmedFlag) {
            isArmedFlag = false;
            supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
        }
        return true;
    }
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
    if (!doRecovery) {
        isCrashedFlag = true;
        crashDetectionTriggered = true;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        return true;
    }
    if (supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) return false;
    isCrashedFlag = false;
    crashDetectionTriggered = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    return true;
}

bool supervisorAreMotorsAllowedToRun(void) {
    return supervisorState == supervisorStateArming ||
           supervisorState == supervisorStateReadyToFly ||
           supervisorState == supervisorStateFlying ||
           supervisorState == supervisorStateWarningLevelOut ||
           supervisorState == supervisorStateLanded;
}

uint16_t supervisorGetInfoBitfield(void) {
    uint16_t bits = 0;
    if (supervisorCanArm()) bits |= (1U << 0);
    if (isArmedFlag) bits |= (1U << 1);
    if (autoArmingCfg) bits |= (1U << 2);
    if (supervisorCanFly()) bits |= (1U << 3);
    if (supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) bits |= (1U << 4);
    if (supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) bits |= (1U << 5);
    if (thrustLocked) bits |= (1U << 6);
    if (isCrashedFlag) bits |= (1U << 7);
    /* trajectory bits and deckFault are represented elsewhere when set */
    if (supervisorConditionBits & SUPERVISOR_CB_DECK_FAULT) bits |= (1U << 11);
    return bits;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick) {
    if (!motorRatios) return false;
    bool overIdle = false;
    for (int i = 0; i < 4; i++) {
        if (motorRatios[i] > idleThrust) {
            overIdle = true;
            break;
        }
    }
    if (overIdle) {
        recentFlightTick = currentTick;
        seenFlightEver = true;
    }
    if (!seenFlightEver) return false;
    return (currentTick - recentFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling) {
    if (isFreeFalling) *isFreeFalling = false;
    if (!tumbleCheckEnabled) return false;

    float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
    if (crashDetectionGs > 0.0f) {
        float diff = fabsf(norm - 1.0f);
        if (diff > crashDetectionGs) {
            isCrashedFlag = true;
            crashDetectionTriggered = true;
            supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        }
    }

    bool freeFall = fabsf(accX) < freeFallThreshold &&
                    fabsf(accY) < freeFallThreshold &&
                    fabsf(accZ) < freeFallThreshold;
    if (freeFall) {
        if (isFreeFalling) *isFreeFalling = true;
        freefallFlag = true;
        supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
        tiltTimerActive = upsideDownTimerActive = false;
        return false;
    }
    freefallFlag = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

    if (accZ >= acceptedTiltAccZ) {
        tiltTimerActive = upsideDownTimerActive = false;
        return false;
    }

    bool tilted = false;
    if (!tiltTimerActive) {
        tiltTimerActive = true;
        tiltTimerStart = currentTick;
    }
    if (maxTiltTime > 0 && (currentTick - tiltTimerStart) >= maxTiltTime) {
        tilted = true;
    }

    if (acceptedUpsideDownAccZ > acceptedTiltAccZ) {
        if (accZ < acceptedUpsideDownAccZ) {
            if (!upsideDownTimerActive) {
                upsideDownTimerActive = true;
                upsideDownTimerStart = currentTick;
            }
            if (maxUpsideDownTime > 0 &&
                (currentTick - upsideDownTimerStart) >= maxUpsideDownTime) {
                tilted = true;
            }
        } else {
            upsideDownTimerActive = false;
        }
    }
    return tilted;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick) {
    if (lastNotificationTick == 0) return true;
    return (currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration) {
    if (latestArmingTick == 0 || state != supervisorStatePreFlChecksPassed) return false;
    return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration) {
    if (latestLandingTick == 0) return false;
    return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed) {
    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
    }

    if (isArmedFlag) supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    else supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;

    if (isCrashedFlag) supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    else supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;

    if (supervisorState == supervisorStateFlying) supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
    else supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;

    if (freefallFlag) supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
    else supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

    return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBitsArg,
                                SupervisorState state) {
    (void)supervisorConditionBitsArg;
    if (!setpoint) return;
    switch (state) {
        case supervisorStateArming:
        case supervisorStateReadyToFly:
        case supervisorStateFlying:
        case supervisorStateLanded:
            return;
        case supervisorStateWarningLevelOut:
            setpoint->mode.x = modeDisable;
            setpoint->mode.y = modeDisable;
            setpoint->mode.roll = modeAbs;
            setpoint->mode.pitch = modeAbs;
            setpoint->attitude.roll = 0.0f;
            setpoint->attitude.pitch = 0.0f;
            setpoint->mode.yaw = modeVelocity;
            setpoint->attitudeRate.yaw = 0.0f;
            /* z kept unchanged */
            return;
        default:
            memset(setpoint, 0, sizeof(*setpoint));
            return;
    }
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax) {
    if (!motorRPMs) return false;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTick) {
    if (!motorRPMs) return false;
    if (!canFly) {
        motorsNotRespondingTimerActive = false;
        return false;
    }
    bool anyBelow = false;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmThreshold) {
            anyBelow = true;
            break;
        }
    }
    if (anyBelow) {
        if (!motorsNotRespondingTimerActive) {
            motorsNotRespondingTimerActive = true;
            motorsNotRespondingStart = currentTick;
        }
        return (currentTick - motorsNotRespondingStart) >= rpmCheckDurationMs;
    }
    motorsNotRespondingTimerActive = false;
    return false;
}

void supervisorSetSensorData(const SensorData *sensors) {
    if (sensors) lastSensors = *sensors;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
    if (motorRatios) {
        for (int i = 0; i < 4; i++) lastMotorRatios[i] = motorRatios[i];
    }
    lastIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
    if (motorRPMs) {
        for (int i = 0; i < 4; i++) lastMotorRPMs[i] = motorRPMs[i];
    }
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled) {
    cfgCrashGs = crashDetectionGs;
    cfgFreeFallThreshold = freeFallThreshold;
    cfgTiltAccZ = acceptedTiltAccZ;
    cfgUpsideDownAccZ = acceptedUpsideDownAccZ;
    cfgMaxTiltTime = maxTiltTime;
    cfgMaxUpsideDownTime = maxUpsideDownTime;
    cfgTumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
    autoArmingCfg = autoArming;
    spinupTimeoutDuration = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep) {
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

    uint32_t tick = supervisorTick++;

    /* Flying check */
    bool flying = isFlyingCheck(lastMotorRatios, lastIdleThrust, tick);
    if (flying) supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
    else supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;

    /* Tumble check */
    bool freeFallTemp = false;
    bool tumbled = isTumbledCheck(lastSensors.acc.x, lastSensors.acc.y, lastSensors.acc.z,
                                  cfgCrashGs, cfgFreeFallThreshold, cfgTiltAccZ,
                                  cfgUpsideDownAccZ, cfgMaxTiltTime,
                                  cfgMaxUpsideDownTime, cfgTumbleCheckEnabled,
                                  tick, &freeFallTemp);
    if (tumbled) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    else supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;

    /* Minimal FSM transitions */
    if (autoArmingCfg && supervisorState == supervisorStatePreFlChecksPassed &&
        !isArmedFlag) {
        supervisorRequestArming(true);
    }

    if (supervisorState == supervisorStateArming && spinupTimeoutDuration > 0) {
        if (spinupStartTick == 0) {
            spinupStartTick = tick;
        } else if ((tick - spinupStartTick) >= spinupTimeoutDuration) {
            supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
        }
    } else if (supervisorState != supervisorStateArming) {
        spinupStartTick = 0;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }

    updateAndPopulateConditions(false, false, false);
}

/* ------------------------------------------------------------------------- */
/* 8. Estimator and Commander arbitration                                     */
/* ------------------------------------------------------------------------- */

#define ESTIMATOR_FIFO_CAPACITY 16
static EstimatorMeasurement fifo[ESTIMATOR_FIFO_CAPACITY];
static uint8_t fifoHead = 0, fifoTail = 0, fifoCount = 0;

static float lastGyro[3] = {0.0f, 0.0f, 0.0f};
static float lastAcc[3] = {0.0f, 0.0f, 0.0f};
static float lastBaro[3] = {0.0f, 0.0f, 0.0f};
static float lastTof[3] = {0.0f, 0.0f, 0.0f};

bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
    if (!measurement || fifoCount >= ESTIMATOR_FIFO_CAPACITY) return false;
    fifo[fifoTail] = *measurement;
    fifoTail = (fifoTail + 1) % ESTIMATOR_FIFO_CAPACITY;
    fifoCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
    if (!measurement || fifoCount == 0) return false;
    *measurement = fifo[fifoHead];
    fifoHead = (fifoHead + 1) % ESTIMATOR_FIFO_CAPACITY;
    fifoCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        switch (m.type) {
            case MeasurementTypeGyroscope:
                memcpy(lastGyro, m.data, sizeof(lastGyro));
                break;
            case MeasurementTypeAcceleration:
                memcpy(lastAcc, m.data, sizeof(lastAcc));
                break;
            case MeasurementTypeBarometer:
                memcpy(lastBaro, m.data, sizeof(lastBaro));
                break;
            case MeasurementTypeTOF:
                memcpy(lastTof, m.data, sizeof(lastTof));
                break;
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        sensfusion6UpdateQ(lastGyro[0], lastGyro[1], lastGyro[2],
                           lastAcc[0], lastAcc[1], lastAcc[2], 1.0f / 250.0f);
        float dummy;
        sensfusion6GetEulerRPY(&dummy, &dummy, &dummy);
        sensfusion6GetQuaternion(&stateEstimate.qw, &stateEstimate.qx,
                                 &stateEstimate.qy, &stateEstimate.qz);
        stateEstimate.roll = dummy;
        (void)dummy;
    }
    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        /* position update boundary: no velocity integration in this frozen API */
    }
}

/* Commander arbitration */
static Setpoint activeSetpoint;
static int activePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t lastSetpointTick = 0;
static uint32_t commanderTick = 0;

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
    if (!setpoint) return false;
    if (priority == COMMANDER_PRIORITY_DISABLE) {
        activeSetpoint = *setpoint;
        activePriority = COMMANDER_PRIORITY_DISABLE;
        lastSetpointTick = commanderTick;
        return true;
    }
    if (priority >= activePriority) {
        activeSetpoint = *setpoint;
        activePriority = priority;
        lastSetpointTick = commanderTick;
        if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
            /* high-level trajectory stop boundary */
        }
        return true;
    }
    return false;
}

void commanderRelaxPriority(void) {
    activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
    return commanderTick - lastSetpointTick;
}

int commanderGetActivePriority(void) {
    return activePriority;
}

/* ------------------------------------------------------------------------- */
/* 9. Stabilizer and compression                                             */
/* ------------------------------------------------------------------------- */

StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0};
Axis3Log acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};

static void sensorsInit(void) {}
static void stateEstimatorInit(void) {}
static void controllerInit(void) { attitudeControllerInit(0.001f); }
static void powerDistributionInit(void) {}
static void motorsInit(void) {}
static void collisionAvoidanceInit(void) {}

static bool stabilizerInitialized = false;

void stabilizerInit(void) {
    if (stabilizerInitialized) return;
    sensorsInit();
    stateEstimatorInit();
    controllerInit();
    powerDistributionInit();
    motorsInit();
    collisionAvoidanceInit();
    stabilizerInitialized = true;
}

static Setpoint highLevelSetpoint;
static bool highLevelSetpointPending = false;

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
    if (!setpoint) return false;
    highLevelSetpoint = *setpoint;
    highLevelSetpointPending = true;
    return true;
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output) {
    if (!state || !sensors || !output) return;
    for (int i = 0; i < 3; i++) {
        output->position_mm[i] = (int32_t)(state->position.x * 1000);
        output->velocity_mms[i] = (int32_t)(state->velocity.x * 1000);
    }
    output->acceleration_mms2[0] = (int32_t)(sensors->acc.x * 9810);
    output->acceleration_mms2[1] = (int32_t)(sensors->acc.y * 9810);
    output->acceleration_mms2[2] = (int32_t)((sensors->acc.z + 1.0f) * 9810);
    output->gyro_millirad_s[0] = sensors->gyro.x * (float)M_PI / 180.0f * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * (float)M_PI / 180.0f * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * (float)M_PI / 180.0f * 1000.0f;

    /* quatcompress: pack quaternion into 32-bit */
    float q[4] = {state->attitudeQuaternion.w, state->attitudeQuaternion.x,
                  state->attitudeQuaternion.y, state->attitudeQuaternion.z};
    uint32_t packed = 0;
    for (uint8_t i = 0; i < 4; i++) {
        int16_t v = (int16_t)(q[i] * 10000.0f);
        packed |= (uint32_t)(uint16_t)v << (i * 8);
    }
    output->quatCompressed = packed;
}

bool rateSupervisorValidate(uint32_t measuredRate) {
    return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void) {
    static uint32_t lastSensorActivity = 0;
    static bool sensorActive = false;
    (void)lastSensorActivity; (void)sensorActive;
    /* 2000 ms supervisor wait is modelled by the caller; if sensors active and
       the wait expires the caller must enter assert/error. */
}

/* ------------------------------------------------------------------------- */
/* 10. Health                                                                */
/* ------------------------------------------------------------------------- */

TestState healthTestState = testDone;
uint8_t motorPass = 0;
uint8_t batteryPass = 0;
float batterySag = 0.0f;

static bool propTestRequested = false;
static bool batTestRequested = false;
static uint32_t batTestTick = 0;
static uint32_t propSampleCount = 0;
static float idleVoltage = 0.0f;
static float propValues[4];
static uint8_t propMotorIndex = 0;
static uint32_t healthMotorTestCount = 0;

void healthRequestPropTest(void) {
    propTestRequested = true;
}

void healthRequestBatteryTest(void) {
    batTestRequested = true;
}

bool healthShallWeRunTest(void) {
    if (propTestRequested) {
        propTestRequested = false;
        healthTestState = configureAcc;
        propSampleCount = 0;
        propMotorIndex = 0;
        return true;
    }
    if (batTestRequested) {
        batTestRequested = false;
        healthTestState = testBattery;
        batTestTick = 0;
        return true;
    }
    if (healthTestState == testDone) return false;
    return true;
}

void healthRunTests(const SensorData *sensorData) {
    switch (healthTestState) {
        case configureAcc:
            propSampleCount = 0;
            idleVoltage = 0.0f;
            motorPass = 0;
            healthTestState = measureNoiseFloor;
            break;
        case measureNoiseFloor:
            if (++propSampleCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
                propSampleCount = 0;
                healthTestState = measureProp;
                propMotorIndex = 0;
                healthMotorTestCount = 0;
            }
            break;
        case measureProp:
            if (sensorData) {
                propValues[propMotorIndex] = sensorData->acc.z;
            }
            healthMotorTestCount++;
            if (healthMotorTestCount >= 100) {
                propMotorIndex++;
                healthMotorTestCount = 0;
                if (propMotorIndex >= 4) {
                    healthTestState = evaluatePropResult;
                }
            }
            break;
        case evaluatePropResult:
            healthTestState = testDone;
            break;
        case testBattery:
            batTestTick++;
            if (batTestTick == 1) {
                /* four motors loaded boundary */
            } else if (batTestTick >= 2 && batTestTick < 50) {
                if (sensorData) {
                    float v = sensorData->baroPressure;
                    (void)v; /* min loaded voltage not used in this API */
                }
            } else if (batTestTick == 50) {
                batterySag = idleVoltage - 0.0f; /* idleVoltage must be populated by adapter */
                healthTestState = evaluateBatResult;
            }
            break;
        case evaluateBatResult:
            healthTestState = testDone;
            break;
        case restartBatTest:
            healthTestState = testBattery;
            batTestTick = 0;
            break;
        case testDone:
        default:
            break;
    }
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex) {
    if (highThreshold == 0.0f) return true;
    if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
        motorPass |= (uint8_t)(1U << motorIndex);
        return true;
    }
    return false;
}

float variance(const float *buffer, int length) {
    if (!buffer || length <= 0) return 0.0f;
    float sum = 0.0f, sumSq = 0.0f;
    for (int i = 0; i < length; i++) {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }
    return sumSq - (sum * sum / (float)length);
}

/* ------------------------------------------------------------------------- */
/* 11. CRTP transport                                                        */
/* ------------------------------------------------------------------------- */

static CrtpPacket txQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t txCount = 0, txHead = 0, txTail = 0;

typedef struct {
    CrtpPacket queue[CRTP_RX_QUEUE_SIZE];
    uint16_t head, tail, count;
    bool active;
} CrtpRxQueue;

static CrtpRxQueue rxQueues[CRTP_NBR_OF_PORTS];
static CrtpPortCallback portCallbacks[CRTP_NBR_OF_PORTS];
static CrtpLink *link = NULL;
static CrtpLink nopLink = {NULL, NULL, NULL, NULL, NULL};
static bool crtpInitialized = false;
static uint32_t rxCounter = 0, txCounter = 0;

static void nopSetEnable(bool enable) { (void)enable; }
static bool nopSendPacket(CrtpPacket *packet) { (void)packet; return false; }
static bool nopReceivePacket(CrtpPacket *packet) { (void)packet; return false; }
static bool nopIsConnected(void) { return true; }
static void nopReset(void) {}

void crtpInit(void) {
    if (crtpInitialized) return;
    txCount = 0; txHead = txTail = 0;
    for (int i = 0; i < CRTP_NBR_OF_PORTS; i++) {
        rxQueues[i].head = rxQueues[i].tail = rxQueues[i].count = 0;
        rxQueues[i].active = false;
        portCallbacks[i] = NULL;
    }
    nopLink.sendPacket = nopSendPacket;
    nopLink.receivePacket = nopReceivePacket;
    nopLink.isConnected = nopIsConnected;
    nopLink.setEnable = nopSetEnable;
    nopLink.reset = nopReset;
    link = &nopLink;
    crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port) {
    if (!crtpInitialized) crtpInit();
    if (port >= CRTP_NBR_OF_PORTS) return;
    if (rxQueues[port].active) {
        /* error-state indicator: duplicate creation */
        return;
    }
    rxQueues[port].active = true;
}

bool crtpSendPacket(const CrtpPacket *packet) {
    if (!crtpInitialized) crtpInit();
    if (!packet || txCount >= CRTP_TX_QUEUE_SIZE) return false;
    txQueue[txTail] = *packet;
    txTail = (txTail + 1) % CRTP_TX_QUEUE_SIZE;
    txCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
    /* Host model: succeeds as long as space exists */
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
    if (!crtpInitialized) crtpInit();
    if (port >= CRTP_NBR_OF_PORTS || !packet) return false;
    CrtpRxQueue *q = &rxQueues[port];
    if (!q->active || q->count == 0) return false;
    *packet = q->queue[q->head];
    q->head = (q->head + 1) % CRTP_RX_QUEUE_SIZE;
    q->count--;
    return true;
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet) {
    return crtpReceivePacket(port, packet);
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet, uint32_t wait_ms) {
    (void)wait_ms;
    return crtpReceivePacket(port, packet);
}

void crtpRxTask(void) {
    if (!crtpInitialized) crtpInit();
    CrtpPacket pkt;
    if (link && link->receivePacket && link->receivePacket(&pkt)) {
        rxCounter++;
        CrtpRxQueue *q = &rxQueues[pkt.port];
        if (q->active && q->count < CRTP_RX_QUEUE_SIZE) {
            q->queue[q->tail] = pkt;
            q->tail = (q->tail + 1) % CRTP_RX_QUEUE_SIZE;
            q->count++;
        }
        if (portCallbacks[pkt.port]) portCallbacks[pkt.port](&pkt);
    }
}

void crtpTxTask(void) {
    if (!crtpInitialized) crtpInit();
    if (txCount == 0) return;
    CrtpPacket *pkt = &txQueue[txHead];
    if (link && link->sendPacket && link->sendPacket(pkt)) {
        txHead = (txHead + 1) % CRTP_TX_QUEUE_SIZE;
        txCount--;
        txCounter++;
    } else {
        /* Retry after 10 ms is simulated by caller; packet remains queued */
    }
}

void crtpSetLink(CrtpLink *newLink) {
    if (!crtpInitialized) crtpInit();
    if (link && link->setEnable) link->setEnable(false);
    if (newLink == NULL) newLink = &nopLink;
    link = newLink;
    if (link && link->setEnable) link->setEnable(true);
}

void crtpReset(void) {
    if (!crtpInitialized) crtpInit();
    txCount = 0; txHead = txTail = 0;
    if (link && link->reset) link->reset();
}

bool crtpIsConnected(void) {
    if (!crtpInitialized) crtpInit();
    if (link && link->isConnected) return link->isConnected();
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
    return CRTP_TX_QUEUE_SIZE - txCount;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
    if (port >= CRTP_NBR_OF_PORTS) return;
    portCallbacks[port] = callback;
}

void updateStats(void) {
    /* Called every 500 ms boundary: compute rates and clear counters */
    rxCounter = 0;
    txCounter = 0;
}

/* ------------------------------------------------------------------------- */
/* 12. Deck discovery and logs                                               */
/* ------------------------------------------------------------------------- */

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
    if (!decks || capacity == 0) return 0;
    uint8_t written = 0;
    /* Known I2C and OneWire ROM scan mock results */
    static const uint8_t i2cAddrs[] = {0x68, 0x76};
    for (unsigned i = 0; i < sizeof(i2cAddrs) && written < capacity; i++) {
        decks[written].foundByI2C = true;
        decks[written].foundByOneWire = false;
        decks[written].i2cAddress = i2cAddrs[i];
        decks[written].oneWireRomId = 0;
        written++;
    }
    return written;
}

/* ------------------------------------------------------------------------- */
/* 13. Log objects                                                           */
/* ------------------------------------------------------------------------- */

/* Log objects are already defined near section 9 and are updated by their
 * owning modules. Zero initialization is provided by C static rules. */

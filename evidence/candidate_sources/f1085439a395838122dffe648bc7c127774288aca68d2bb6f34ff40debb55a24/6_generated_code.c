#include "6_generated_code.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ========================================================================= */
/* 2. Numeric Utilities & Sensfusion6                                        */
/* ========================================================================= */

int16_t saturateSignedInt16(int32_t value) {
    if (value > 32767) {
        return 32767;
    }
    if (value < -32767) {
        return -32767;
    }
    return (int16_t)value;
}

float capAngle(float angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float invSqrt(float x) {
    union {
        float f;
        uint32_t i;
    } conv;
    conv.f = x;
    float xhalf = 0.5f * x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (xhalf * conv.f * conv.f));
    return conv.f;
}

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

void sensfusion6Init(void) {
    if (sensfusion6IsInit) {
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
    sensfusion6IsCalibrated = false;
    sensfusion6IsInit = true;
}

bool sensfusion6Test(void) {
    return sensfusion6IsInit;
}

void estimatedGravityDirection(float qw_in, float qx_in, float qy_in, float qz_in,
                               float *gravX, float *gravY, float *gravZ) {
    if (!gravX || !gravY || !gravZ) return;
    *gravX = 2.0f * (qx_in * qz_in - qw_in * qy_in);
    *gravY = 2.0f * (qw_in * qx_in + qy_in * qz_in);
    *gravZ = qw_in * qw_in - qx_in * qx_in - qy_in * qy_in + qz_in * qz_in;
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt) {
    if (!sensfusion6IsInit) return;

    float gx_rad = gx * (M_PI / 180.0f);
    float gy_rad = gy * (M_PI / 180.0f);
    float gz_rad = gz * (M_PI / 180.0f);

    float normSq = ax * ax + ay * ay + az * az;
    if (normSq > 0.000001f) {
        float invN = invSqrt(normSq);
        float norm_ax = ax * invN;
        float norm_ay = ay * invN;
        float norm_az = az * invN;

        float vx, vy, vz;
        estimatedGravityDirection(qw, qx, qy, qz, &vx, &vy, &vz);

        float ex = norm_ay * vz - norm_az * vy;
        float ey = norm_az * vx - norm_ax * vz;
        float ez = norm_ax * vy - norm_ay * vx;

        if (twoKi > 0.0f) {
            integralFBx += twoKi * ex * dt;
            integralFBy += twoKi * ey * dt;
            integralFBz += twoKi * ez * dt;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx_rad += twoKp * ex + integralFBx;
        gy_rad += twoKp * ey + integralFBy;
        gz_rad += twoKp * ez + integralFBz;

        if (!sensfusion6IsCalibrated) {
            baseZacc = ax * vx + ay * vy + az * vz;
            sensfusion6IsCalibrated = true;
        }
    }

    float qDot1 = 0.5f * (-qx * gx_rad - qy * gy_rad - qz * gz_rad);
    float qDot2 = 0.5f * ( qw * gx_rad + qy * gz_rad - qz * gy_rad);
    float qDot3 = 0.5f * ( qw * gy_rad - qx * gz_rad + qz * gx_rad);
    float qDot4 = 0.5f * ( qw * gz_rad + qx * gy_rad - qy * gx_rad);

    qw += qDot1 * dt;
    qx += qDot2 * dt;
    qy += qDot3 * dt;
    qz += qDot4 * dt;

    float qNorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    qw *= qNorm;
    qx *= qNorm;
    qy *= qNorm;
    qz *= qNorm;

    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
    if (roll_deg) {
        *roll_deg = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * (180.0f / M_PI);
    }
    if (pitch_deg) {
        float gx_clamped = gravityX;
        if (gx_clamped > 1.0f) gx_clamped = 1.0f;
        if (gx_clamped < -1.0f) gx_clamped = -1.0f;
        *pitch_deg = asinf(-gx_clamped) * (180.0f / M_PI);
    }
    if (yaw_deg) {
        *yaw_deg = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * (180.0f / M_PI);
    }
}

void sensfusion6GetQuaternion(float *qw_out, float *qx_out, float *qy_out, float *qz_out) {
    if (qw_out) *qw_out = qw;
    if (qx_out) *qx_out = qx;
    if (qy_out) *qy_out = qy;
    if (qz_out) *qz_out = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
    float gx, gy, gz;
    estimatedGravityDirection(qw, qx, qy, qz, &gx, &gy, &gz);
    return ax * gx + ay * gy + az * gz;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* ========================================================================= */
/* 4. Power Distribution & Battery                                           */
/* ========================================================================= */

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out) {
    if (!out) return;
    int32_t r = roll / 2;
    int32_t p = pitch / 2;
    out->m1 = (int32_t)thrust - r + p + (int32_t)yaw;
    out->m2 = (int32_t)thrust - r - p - (int32_t)yaw;
    out->m3 = (int32_t)thrust + r - p + (int32_t)yaw;
    out->m4 = (int32_t)thrust + r + p - (int32_t)yaw;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4]) {
    if (!motorForces) return;
    float thrustPart = 0.25f * thrustSi;
    float arm = 0.707106781f * armLength;

    float rollPart = (arm > 0.0f) ? (0.25f / arm * torqueX) : 0.0f;
    float pitchPart = (arm > 0.0f) ? (0.25f / arm * torqueY) : 0.0f;
    float yawPart = (thrustToTorque > 0.0f) ? (0.25f / thrustToTorque * torqueZ) : 0.0f;

    motorForces[0] = thrustPart - rollPart + pitchPart + yawPart;
    motorForces[1] = thrustPart - rollPart - pitchPart - yawPart;
    motorForces[2] = thrustPart + rollPart - pitchPart + yawPart;
    motorForces[3] = thrustPart + rollPart + pitchPart - yawPart;

    for (int i = 0; i < 4; i++) {
        if (motorForces[i] < 0.0f) {
            motorForces[i] = 0.0f;
        }
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

void powerDistribution(const ControlData *control, MotorPower *motorPower) {
    if (!control || !motorPower) return;
    if (control->controlMode == controlModeLegacy) {
        powerDistributionLegacy(control->thrust, control->roll, control->pitch, control->yaw, motorPower);
    } else if (control->controlMode == controlModeForceTorque) {
        float forces[4];
        powerDistributionForceTorque(control->thrustSi, control->torque.x, control->torque.y, control->torque.z,
                                     CRAZYFLIE_ARM_LENGTH_M, CRAZYFLIE_THRUST_TO_TORQUE, forces);
        motorPower->m1 = (int32_t)(forces[0] / CRAZYFLIE_MAX_MOTOR_FORCE_N * 65535.0f);
        motorPower->m2 = (int32_t)(forces[1] / CRAZYFLIE_MAX_MOTOR_FORCE_N * 65535.0f);
        motorPower->m3 = (int32_t)(forces[2] / CRAZYFLIE_MAX_MOTOR_FORCE_N * 65535.0f);
        motorPower->m4 = (int32_t)(forces[3] / CRAZYFLIE_MAX_MOTOR_FORCE_N * 65535.0f);
    } else if (control->controlMode == controlModeForce) {
        uint16_t pwms[4];
        powerDistributionForce(control->normalizedForces, pwms);
        motorPower->m1 = pwms[0];
        motorPower->m2 = pwms[1];
        motorPower->m3 = pwms[2];
        motorPower->m4 = pwms[3];
    }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) {
    return (value < idleThrust) ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust) {
    PowerCapResult res = {false, 0};
    if (!motors) return res;

    int32_t maxVal = motors[0];
    for (int i = 1; i < 4; i++) {
        if (motors[i] > maxVal) {
            maxVal = motors[i];
        }
    }

    if (maxVal > maxAllowedThrust) {
        res.isCapped = true;
        res.reduction = maxVal - maxAllowedThrust;
        for (int i = 0; i < 4; i++) {
            motors[i] = capMinThrust(motors[i] - res.reduction, idleThrust);
        }
    }
    return res;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) {
    return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust,
                                        float nominalVoltage,
                                        float actualVoltage) {
    if (actualVoltage <= 0.0f) {
        return motorThrust;
    }
    float comp = (float)motorThrust * nominalVoltage / actualVoltage;
    int32_t val = (int32_t)lroundf(comp);
    if (val < 0) val = 0;
    if (val > 65535) val = 65535;
    return (uint16_t)val;
}

/* ========================================================================= */
/* 5. Cascaded PID & controllerPid                                           */
/* ========================================================================= */

PidObject pidRoll = {0}, pidPitch = {0}, pidYaw = {0};
PidObject pidRollRate = {0}, pidPitchRate = {0}, pidYawRate = {0};

void attitudeControllerInit(float updateDt) {
    (void)updateDt;
    pidRoll.initialized = true; pidRoll.integral = 0.0f; pidRoll.prevError = 0.0f;
    pidPitch.initialized = true; pidPitch.integral = 0.0f; pidPitch.prevError = 0.0f;
    pidYaw.initialized = true; pidYaw.integral = 0.0f; pidYaw.prevError = 0.0f;
    pidRollRate.initialized = true; pidRollRate.integral = 0.0f; pidRollRate.prevError = 0.0f;
    pidPitchRate.initialized = true; pidPitchRate.integral = 0.0f; pidPitchRate.prevError = 0.0f;
    pidYawRate.initialized = true; pidYawRate.integral = 0.0f; pidYawRate.prevError = 0.0f;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
    pidRollRate.output = rollDesired - rollActual;
    pidPitchRate.output = pitchDesired - pitchActual;
    pidYawRate.output = yawDesired - yawActual;
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
    pidRoll.output = rollDesired - rollActual;
    pidPitch.output = pitchDesired - pitchActual;
    pidYaw.output = yawDesired - yawActual;
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual, float yawActual) {
    attitudeControllerResetRollAttitudePID(rollActual);
    attitudeControllerResetPitchAttitudePID(pitchActual);
    pidYaw.integral = 0.0f; pidYaw.prevError = 0.0f;
    pidRollRate.integral = 0.0f; pidRollRate.prevError = 0.0f;
    pidPitchRate.integral = 0.0f; pidPitchRate.prevError = 0.0f;
    pidYawRate.integral = 0.0f; pidYawRate.prevError = 0.0f;
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
    (void)rollActual;
    pidRoll.integral = 0.0f;
    pidRoll.prevError = 0.0f;
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
    (void)pitchActual;
    pidPitch.integral = 0.0f;
    pidPitch.prevError = 0.0f;
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch, int16_t *yaw) {
    if (roll) *roll = saturateSignedInt16((int32_t)pidRollRate.output);
    if (pitch) *pitch = saturateSignedInt16((int32_t)pidPitchRate.output);
    if (yaw) *yaw = saturateSignedInt16((int32_t)pidYawRate.output);
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
    if (!setpoint || !state) return 0;
    return setpoint->thrust;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
    if (!sensors || !setpoint || !state || !control) return;

    static float desiredYaw = 0.0f;

    if (setpoint->mode.yaw == modeVelocity) {
        desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
    } else {
        desiredYaw = setpoint->attitude.yaw;
    }

    if (yawMaxDelta != 0.0f) {
        float delta = desiredYaw - state->attitude.yaw;
        if (delta > yawMaxDelta) desiredYaw = state->attitude.yaw + yawMaxDelta;
        if (delta < -yawMaxDelta) desiredYaw = state->attitude.yaw - yawMaxDelta;
    }

    uint16_t thrust = setpoint->thrust;
    if (setpoint->mode.z != modeDisable) {
        thrust = positionControllerUpdate(setpoint, state);
    }

    if (thrust == 0) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0;
        attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
        desiredYaw = state->attitude.yaw;
        return;
    }

    float pitchActual = -sensors->gyro.y;
    attitudeControllerCorrectRatePID(sensors->gyro.x, setpoint->attitudeRate.roll,
                                     pitchActual, setpoint->attitudeRate.pitch,
                                     sensors->gyro.z, setpoint->attitudeRate.yaw);

    int16_t r_out, p_out, y_out;
    attitudeControllerGetActuatorOutput(&r_out, &p_out, &y_out);

    control->roll = r_out;
    control->pitch = p_out;
    control->yaw = -y_out;
    control->thrust = thrust;
}

/* ========================================================================= */
/* 6. CRTP Commander RPYT                                                    */
/* ========================================================================= */

bool thrustLocked = true;
bool commanderModeSet = false;

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
    if (!rollPrime || !pitchPrime) return;
    float rad = yaw_deg * (M_PI / 180.0f);
    *rollPrime = roll * cosf(rad) - pitch * sinf(rad);
    *pitchPrime = roll * sinf(rad) + pitch * cosf(rad);
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

    if (values->thrust == 0) {
        thrustLocked = false;
    }

    if (!altHoldMode) {
        if (thrustLocked || values->thrust < MIN_THRUST) {
            setpoint->thrust = 0;
        } else {
            setpoint->thrust = (values->thrust > MAX_THRUST) ? MAX_THRUST : values->thrust;
        }
    } else {
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0;
        setpoint->velocity.z = ((float)values->thrust - 32767.0f) / 32767.0f;
        commanderModeSet = true;
    }

    float r = values->roll;
    float p = values->pitch;
    if (yawMode == PLUSMODE) {
        rotateYaw(r, p, 45.0f, &r, &p);
    }

    if (posHoldMode) {
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->velocity.x = p / 30.0f;
        setpoint->velocity.y = r / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
    } else if (posSetMode && values->thrust != 0) {
        setpoint->mode.x = modeAbs;
        setpoint->mode.y = modeAbs;
        setpoint->mode.z = modeAbs;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->mode.yaw = modeAbs;
        setpoint->position.x = -p;
        setpoint->position.y = r;
        setpoint->position.z = (float)values->thrust / 1000.0f;
        setpoint->attitude.yaw = values->yaw;
        setpoint->thrust = 0;
    } else {
        if (stabilizationModeRoll == RATE) {
            setpoint->mode.roll = modeVelocity;
            setpoint->attitudeRate.roll = r;
        } else {
            setpoint->mode.roll = modeAbs;
            setpoint->attitude.roll = r;
        }

        if (stabilizationModePitch == RATE) {
            setpoint->mode.pitch = modeVelocity;
            setpoint->attitudeRate.pitch = p;
        } else {
            setpoint->mode.pitch = modeAbs;
            setpoint->attitude.pitch = p;
        }

        if (stabilizationModeYaw == RATE) {
            setpoint->mode.yaw = modeVelocity;
            setpoint->attitudeRate.yaw = -values->yaw;
        } else {
            setpoint->mode.yaw = modeAbs;
            setpoint->attitude.yaw = values->yaw;
        }
    }
}

/* ========================================================================= */
/* 7. Supervisor                                                             */
/* ========================================================================= */

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0;

static uint32_t lastFlightTick = 0;
static bool seenFlight = false;

void supervisorInit(void) {
    supervisorState = supervisorStatePreFlChecksNotPassed;
    supervisorConditionBits = 0;
    lastFlightTick = 0;
    seenFlight = false;
}

void supervisorUpdate(uint32_t stabilizerStep) {
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;
}

bool supervisorCanFly(void) {
    return (supervisorState == supervisorStateReadyToFly ||
            supervisorState == supervisorStateFlying ||
            supervisorState == supervisorStateWarningLevelOut ||
            supervisorState == supervisorStateLanded);
}

bool supervisorCanArm(void) {
    return (supervisorState == supervisorStatePreFlChecksPassed);
}

bool supervisorIsArmed(void) {
    return (supervisorConditionBits & SUPERVISOR_CB_ARMED) != 0;
}

bool supervisorIsCrashed(void) {
    return (supervisorConditionBits & SUPERVISOR_CB_CRASHED) != 0;
}

bool supervisorRequestArming(bool doArm) {
    if (doArm) {
        if (supervisorCanArm() || supervisorState == supervisorStateArming) {
            supervisorState = supervisorStateArming;
            supervisorConditionBits |= SUPERVISOR_CB_ARMED;
            return true;
        }
        if (supervisorCanFly()) {
            supervisorConditionBits |= SUPERVISOR_CB_ARMED;
            return true;
        }
        return false;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
        return true;
    }
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
    if (supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) {
        return false;
    }
    if (doRecovery) {
        supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
        return true;
    } else {
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        supervisorState = supervisorStateCrashed;
        return true;
    }
}

bool supervisorAreMotorsAllowedToRun(void) {
    return (supervisorState == supervisorStateArming ||
            supervisorState == supervisorStateReadyToFly ||
            supervisorState == supervisorStateFlying ||
            supervisorState == supervisorStateWarningLevelOut ||
            supervisorState == supervisorStateLanded);
}

uint16_t supervisorGetInfoBitfield(void) {
    uint16_t bitfield = 0;
    if (supervisorCanArm()) bitfield |= (1 << 0);
    if (supervisorIsArmed()) bitfield |= (1 << 1);
    if (supervisorCanFly()) bitfield |= (1 << 3);
    if (supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) bitfield |= (1 << 4);
    if (supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) bitfield |= (1 << 5);
    if (supervisorState == supervisorStateLocked) bitfield |= (1 << 6);
    if (supervisorIsCrashed()) bitfield |= (1 << 7);
    return bitfield;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick) {
    if (!motorRatios) return false;
    bool active = false;
    for (int i = 0; i < 4; i++) {
        if (motorRatios[i] > idleThrust) {
            active = true;
            break;
        }
    }

    if (active) {
        lastFlightTick = currentTick;
        seenFlight = true;
        return true;
    }

    if (!seenFlight) {
        return false;
    }

    if ((currentTick - lastFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD) {
        return true;
    }
    return false;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling) {
    (void)acceptedTiltAccZ; (void)acceptedUpsideDownAccZ;
    (void)maxTiltTime; (void)maxUpsideDownTime; (void)currentTick;

    if (isFreeFalling) *isFreeFalling = false;

    if (tumbleCheckEnabled) {
        float accNorm = sqrtf(accX * accX + accY * accY + accZ * accZ);
        if (freeFallThreshold > 0.0f && fabsf(accX) < freeFallThreshold &&
            fabsf(accY) < freeFallThreshold && fabsf(accZ) < freeFallThreshold) {
            if (isFreeFalling) *isFreeFalling = true;
            supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
        }
        if (crashDetectionGs > 0.0f && fabsf(accNorm - 1.0f) > crashDetectionGs) {
            supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        }
    }
    return (supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) != 0;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick) {
    if (lastNotificationTick == 0) {
        return true;
    }
    if ((currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT) {
        return true;
    }
    return false;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration) {
    if (state == supervisorStateArming && (currentTick - latestArmingTick) > preflightTimeoutDuration) {
        return true;
    }
    return false;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration) {
    return (currentTick - latestLandingTick) > landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed) {
    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    }
    return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t bits,
                                SupervisorState state) {
    if (!setpoint) return;
    if (state == supervisorStateWarningLevelOut) {
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = 0.0f;
    } else if (state != supervisorStateArming &&
               state != supervisorStateReadyToFly &&
               state != supervisorStateFlying &&
               state != supervisorStateLanded) {
        memset(setpoint, 0, sizeof(Setpoint));
    }
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin, int32_t rpmCheckMax) {
    if (!motorRPMs) return false;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) {
            return false;
        }
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTick) {
    (void)rpmCheckDurationMs; (void)currentTick;
    if (!canFly || !motorRPMs) return false;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmThreshold) return true;
    }
    return false;
}

void supervisorSetSensorData(const SensorData *sensors) { (void)sensors; }
void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
    (void)motorRatios; (void)idleThrust;
}
void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) { (void)motorRPMs; }
void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled) {
    (void)crashDetectionGs; (void)freeFallThreshold; (void)acceptedTiltAccZ;
    (void)acceptedUpsideDownAccZ; (void)maxTiltTime; (void)maxUpsideDownTime;
    (void)tumbleCheckEnabled;
}
void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
    (void)autoArming; (void)spinupTimeoutDurationMs;
}

/* ========================================================================= */
/* 8. Estimator & Commander Arbitration                                      */
/* ========================================================================= */

static EstimatorMeasurement estimatorFifo[16];
static size_t estimatorHead = 0;
static size_t estimatorTail = 0;
static size_t estimatorCount = 0;

bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
    if (!measurement || estimatorCount >= 16) {
        return false;
    }
    estimatorFifo[estimatorHead] = *measurement;
    estimatorHead = (estimatorHead + 1) % 16;
    estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
    if (!measurement || estimatorCount == 0) {
        return false;
    }
    *measurement = estimatorFifo[estimatorTail];
    estimatorTail = (estimatorTail + 1) % 16;
    estimatorCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        /* Process queued measurements */
    }
    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        sensfusion6UpdateQ(0, 0, 0, 0, 0, 1.0f, 0.004f);
    }
}

static int activeCommanderPriority = COMMANDER_PRIORITY_DISABLE;
static Setpoint activeSetpoint;

void commanderRelaxPriority(void) {
    activeCommanderPriority = COMMANDER_PRIORITY_LOWEST;
}

int commanderGetActivePriority(void) {
    return activeCommanderPriority;
}

uint32_t commanderGetInactivityTime(void) {
    return 0;
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
    if (!setpoint) return false;

    if (priority == COMMANDER_PRIORITY_DISABLE) {
        activeCommanderPriority = COMMANDER_PRIORITY_DISABLE;
        activeSetpoint = *setpoint;
        return true;
    }

    if (priority >= activeCommanderPriority) {
        activeCommanderPriority = priority;
        activeSetpoint = *setpoint;
        return true;
    }

    return false;
}

/* ========================================================================= */
/* 9. Stabilizer, State Compression & Rate Supervisor                       */
/* ========================================================================= */

void stabilizerInit(void) {
    sensfusion6Init();
    attitudeControllerInit(0.002f);
    supervisorInit();
}

void stabilizerTask(void) {
    /* Main loop step execution */
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
    return commanderSetSetpoint(setpoint, COMMANDER_PRIORITY_HIGHLEVEL);
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output) {
    if (!state || !sensors || !output) return;

    output->position_mm[0] = (int32_t)(state->position.x * 1000.0f);
    output->position_mm[1] = (int32_t)(state->position.y * 1000.0f);
    output->position_mm[2] = (int32_t)(state->position.z * 1000.0f);

    output->velocity_mms[0] = (int32_t)(state->velocity.x * 1000.0f);
    output->velocity_mms[1] = (int32_t)(state->velocity.y * 1000.0f);
    output->velocity_mms[2] = (int32_t)(state->velocity.z * 1000.0f);

    output->acceleration_mms2[0] = (int32_t)(sensors->acc.x * 9810.0f);
    output->acceleration_mms2[1] = (int32_t)(sensors->acc.y * 9810.0f);
    output->acceleration_mms2[2] = (int32_t)((sensors->acc.z + 1.0f) * 9810.0f);

    output->gyro_millirad_s[0] = sensors->gyro.x * (M_PI / 180.0f) * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * (M_PI / 180.0f) * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * (M_PI / 180.0f) * 1000.0f;

    output->quatCompressed = 0;
}

bool rateSupervisorValidate(uint32_t measuredRate) {
    return (measuredRate >= 997U && measuredRate <= 1003U);
}

void rateSupervisorTask(void) {}

/* ========================================================================= */
/* 10. Health Diagnostics                                                    */
/* ========================================================================= */

TestState healthTestState = testDone;
uint8_t motorPass = 0, batteryPass = 0;
float batterySag = 0.0f;

bool healthShallWeRunTest(void) {
    return (healthTestState != testDone);
}

void healthRunTests(const SensorData *sensorData) {
    (void)sensorData;
}

void healthRequestPropTest(void) {
    healthTestState = configureAcc;
}

void healthRequestBatteryTest(void) {
    healthTestState = testBattery;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIdx) {
    if (highThreshold == 0.0f) {
        return true;
    }
    if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
        motorPass |= (1 << motorIdx);
        return true;
    }
    return false;
}

float variance(const float *buffer, int length) {
    if (!buffer || length <= 0) return 0.0f;
    float sum = 0.0f;
    float sumSq = 0.0f;
    for (int i = 0; i < length; i++) {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }
    return sumSq - ((sum * sum) / (float)length);
}

/* ========================================================================= */
/* 11. CRTP Transport                                                        */
/* ========================================================================= */

static CrtpPacket rxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint8_t rxHead[CRTP_NBR_OF_PORTS] = {0};
static uint8_t rxTail[CRTP_NBR_OF_PORTS] = {0};
static uint8_t rxCount[CRTP_NBR_OF_PORTS] = {0};

static CrtpPacket txQueue[CRTP_TX_QUEUE_SIZE];
static uint32_t txCount = 0;
static CrtpPortCallback portCallbacks[CRTP_NBR_OF_PORTS] = {NULL};

void crtpInit(void) {
    txCount = 0;
    memset(rxCount, 0, sizeof(rxCount));
}

void crtpInitTaskQueue(uint8_t port) {
    if (port < CRTP_NBR_OF_PORTS) {
        rxHead[port] = 0;
        rxTail[port] = 0;
        rxCount[port] = 0;
    }
}

bool crtpSendPacket(const CrtpPacket *packet) {
    if (!packet || txCount >= CRTP_TX_QUEUE_SIZE) return false;
    txQueue[txCount++] = *packet;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
    if (port >= CRTP_NBR_OF_PORTS || !packet || rxCount[port] == 0) return false;
    *packet = rxQueues[port][rxTail[port]];
    rxTail[port] = (rxTail[port] + 1) % CRTP_RX_QUEUE_SIZE;
    rxCount[port]--;
    return true;
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet) {
    return crtpReceivePacket(port, packet);
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet, uint32_t wait_ms) {
    (void)wait_ms;
    return crtpReceivePacket(port, packet);
}

void crtpRxTask(void) {}
void crtpTxTask(void) {}
void crtpSetLink(CrtpLink *newLink) { (void)newLink; }
void crtpReset(void) { txCount = 0; }
bool crtpIsConnected(void) { return true; }

uint32_t crtpGetFreeTxQueuePackets(void) {
    return CRTP_TX_QUEUE_SIZE - txCount;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
    if (port < CRTP_NBR_OF_PORTS) {
        portCallbacks[port] = callback;
    }
}

void updateStats(void) {}

/* ========================================================================= */
/* 12. Deck Discovery & Log Objects                                          */
/* ========================================================================= */

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
    (void)decks; (void)capacity;
    return 0;
}

StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0}, acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};

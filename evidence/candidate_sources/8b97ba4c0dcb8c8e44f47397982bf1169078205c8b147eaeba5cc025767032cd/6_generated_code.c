// File: 6_generated_code.c
#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)
#define PI_F 3.14159265358979323846f

/* -------------------------------------------------------------------------
 * 3. Numeric utilities and Sensfusion6
 * ---------------------------------------------------------------------- */

/* Sensfusion6 global state */
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

int16_t saturateSignedInt16(int32_t value) {
    if (value > INT16_MAX) return INT16_MAX;
    if (value < -INT16_MAX) return (int16_t)-INT16_MAX;
    return (int16_t)value;
}

float capAngle(float angle_deg) {
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

float invSqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    float xhalf = 0.5f * x;
    int32_t i;
    memcpy(&i, &x, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    float y;
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - xhalf * y * y);
    return y;
}

void estimatedGravityDirection(float qw_, float qx_, float qy_, float qz_,
                               float *gravX, float *gravY, float *gravZ) {
    if (!gravX || !gravY || !gravZ) return;
    *gravX = 2.0f * (qx_ * qz_ - qw_ * qy_);
    *gravY = 2.0f * (qy_ * qz_ + qw_ * qx_);
    *gravZ = qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_;
}

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

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt) {
    if (!sensfusion6IsInit) return;
    if (dt < 0.0f) dt = 0.0f;

    /* Sensor environment provides deg/s; Mahony uses rad/s */
    gx *= DEG2RAD;
    gy *= DEG2RAD;
    gz *= DEG2RAD;

    float accNorm = sqrtf(ax * ax + ay * ay + az * az);

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
    if (accNorm > 0.000001f) {
        float invA = 1.0f / accNorm;
        ax *= invA; ay *= invA; az *= invA;

        /* Estimated gravity direction from current quaternion */
        float gxEst = 2.0f * (qx * qz - qw * qy);
        float gyEst = 2.0f * (qy * qz + qw * qx);
        float gzEst = qw * qw - qx * qx - qy * qy + qz * qz;

        /* Gradient correction, beta=0.01, no integral feedback */
        float ex = ay * gzEst - az * gyEst;
        float ey = az * gxEst - ax * gzEst;
        float ez = ax * gyEst - ay * gxEst;

        gx += beta * ex;
        gy += beta * ey;
        gz += beta * ez;

        if (!sensfusion6IsCalibrated) {
            baseZacc = ax * gxEst + ay * gyEst + az * gzEst;
            sensfusion6IsCalibrated = true;
        }
    }
#else
    if (accNorm > 0.000001f) {
        float invA = 1.0f / accNorm;
        ax *= invA; ay *= invA; az *= invA;

        float gxEst = 2.0f * (qx * qz - qw * qy);
        float gyEst = 2.0f * (qy * qz + qw * qx);
        float gzEst = qw * qw - qx * qx - qy * qy + qz * qz;

        float ex = ay * gzEst - az * gyEst;
        float ey = az * gxEst - ax * gzEst;
        float ez = ax * gyEst - ay * gxEst;

        if (twoKi > 0.0f) {
            integralFBx += twoKi * ex * dt;
            integralFBy += twoKi * ey * dt;
            integralFBz += twoKi * ez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += twoKp * ex;
        gy += twoKp * ey;
        gz += twoKp * ez;

        if (!sensfusion6IsCalibrated) {
            baseZacc = ax * gxEst + ay * gyEst + az * gzEst;
            sensfusion6IsCalibrated = true;
        }
    }
#endif

    float qdotw = 0.5f * (-qx * gx - qy * gy - qz * gz);
    float qdotx = 0.5f * (qw * gx + qy * gz - qz * gy);
    float qdoty = 0.5f * (qw * gy - qx * gz + qz * gx);
    float qdotz = 0.5f * (qw * gz + qx * gy - qy * gx);

    qw += qdotw * dt;
    qx += qdotx * dt;
    qy += qdoty * dt;
    qz += qdotz * dt;

    float qnorm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (qnorm > 0.000001f) {
        float invQ = 1.0f / qnorm;
        qw *= invQ; qx *= invQ; qy *= invQ; qz *= invQ;
    }

    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
    if (!roll_deg || !pitch_deg || !yaw_deg) return;

    /* Use current quaternion, not possibly stale gravity cache */
    float gx = 2.0f * (qx * qz - qw * qy);
    float gy = 2.0f * (qy * qz + qw * qx);
    float gz = qw * qw - qx * qx - qy * qy + qz * qz;

    if (gx > 1.0f) gx = 1.0f;
    if (gx < -1.0f) gx = -1.0f;

    *roll_deg = atan2f(2.0f * (qw * qx + qy * qz),
                       1.0f - 2.0f * (qx * qx + qy * qy)) * RAD2DEG;
    *pitch_deg = asinf(gx) * RAD2DEG;
    *yaw_deg = atan2f(2.0f * (qw * qz + qx * qy),
                      1.0f - 2.0f * (qy * qy + qz * qz)) * RAD2DEG;
}

void sensfusion6GetQuaternion(float *qw_out, float *qx_out, float *qy_out, float *qz_out) {
    if (!qw_out || !qx_out || !qy_out || !qz_out) return;
    *qw_out = qw; *qx_out = qx; *qy_out = qy; *qz_out = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
    float gx = 2.0f * (qx * qz - qw * qy);
    float gy = 2.0f * (qy * qz + qw * qx);
    float gz = qw * qw - qx * qx - qy * qy + qz * qz;
    return ax * gx + ay * gy + az * gz;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* -------------------------------------------------------------------------
 * 4. Power distribution and battery compensation
 * ---------------------------------------------------------------------- */

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out) {
    if (!out) return;
    int32_t r = (int32_t)roll / 2;
    int32_t p = (int32_t)pitch / 2;
    int32_t t = (int32_t)thrust;
    int32_t y = (int32_t)yaw;

    out->m1 = t - r + p + y;
    out->m2 = t - r - p - y;
    out->m3 = t + r - p + y;
    out->m4 = t + r + p - y;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4]) {
    if (!motorForces) return;

    float thrustPart = 0.25f * thrustSi;
    float arm = 0.707106781f * armLength;

    float rollPart = 0.0f;
    float pitchPart = 0.0f;
    float yawPart = 0.0f;

    if (arm > 0.000001f) {
        rollPart = 0.25f / arm * torqueX;
        pitchPart = 0.25f / arm * torqueY;
    }
    if (thrustToTorque > 0.000001f) {
        yawPart = 0.25f / thrustToTorque * torqueZ;
    }

    motorForces[0] = thrustPart - rollPart + pitchPart + yawPart;
    motorForces[1] = thrustPart - rollPart - pitchPart - yawPart;
    motorForces[2] = thrustPart + rollPart - pitchPart + yawPart;
    motorForces[3] = thrustPart + rollPart + pitchPart - yawPart;

    for (int i = 0; i < 4; i++) {
        if (motorForces[i] < 0.0f) motorForces[i] = 0.0f;
    }
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4]) {
    if (!normalizedForces || !motorPWMs) return;
    for (int i = 0; i < 4; i++) {
        float f = normalizedForces[i];
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
        motorPWMs[i] = (uint16_t)(f * 65535.0f);
    }
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
                                     control->torque.x,
                                     control->torque.y,
                                     control->torque.z,
                                     CRAZYFLIE_ARM_LENGTH_M,
                                     CRAZYFLIE_THRUST_TO_TORQUE,
                                     forces);
        /* Explicit conversion boundary: force -> PWM via max motor force ratio */
        for (int i = 0; i < 4; i++) {
            float ratio = forces[i] / CRAZYFLIE_MAX_MOTOR_FORCE_N;
            if (ratio < 0.0f) ratio = 0.0f;
            if (ratio > 1.0f) ratio = 1.0f;
            uint16_t pwm = (uint16_t)(ratio * 65535.0f);
            switch (i) {
            case 0: motorPower->m1 = pwm; break;
            case 1: motorPower->m2 = pwm; break;
            case 2: motorPower->m3 = pwm; break;
            case 3: motorPower->m4 = pwm; break;
            }
        }
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
        /* Unknown control mode: do not modify output */
        break;
    }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) {
    return value < idleThrust ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust) {
    PowerCapResult result = { false, 0 };
    if (!motors) return result;

    int32_t maxMotor = motors[0];
    for (int i = 1; i < 4; i++) {
        if (motors[i] > maxMotor) maxMotor = motors[i];
    }

    if (maxMotor > maxAllowedThrust) {
        result.isCapped = true;
        result.reduction = maxMotor - maxAllowedThrust;
        for (int i = 0; i < 4; i++) {
            motors[i] -= result.reduction;
            motors[i] = capMinThrust(motors[i], idleThrust);
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
    float comp = (float)motorThrust * nominalVoltage / actualVoltage;
    if (comp < 0.0f) comp = 0.0f;
    if (comp > 65535.0f) comp = 65535.0f;
    return (uint16_t)lroundf(comp);
}

/* -------------------------------------------------------------------------
 * 5. Cascade PID and controllerPid
 * ---------------------------------------------------------------------- */

PidObject pidRoll, pidPitch, pidYaw;
PidObject pidRollRate, pidPitchRate, pidYawRate;

static float pidUpdateDt = 1.0f / ATTITUDE_RATE_HZ;

static void pidObjectInit(PidObject *pid) {
    if (!pid) return;
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->kff = 0.0f;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
    pid->initialized = true;
}

static float pidUpdate(PidObject *pid, float error, float desired, float dt) {
    if (!pid) return 0.0f;
    if (dt < 0.000001f) dt = 1.0f / ATTITUDE_RATE_HZ;

    pid->integral += error * dt;
    float derivative = (error - pid->prevError) / dt;
    pid->output = pid->kp * error + pid->ki * pid->integral +
                  pid->kd * derivative + pid->kff * desired;
    pid->prevError = error;
    return pid->output;
}

static void pidReset(PidObject *pid) {
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
}

void attitudeControllerInit(float updateDt) {
    if (updateDt > 0.000001f) pidUpdateDt = updateDt;

    pidObjectInit(&pidRoll);
    pidObjectInit(&pidPitch);
    pidObjectInit(&pidYaw);
    pidObjectInit(&pidRollRate);
    pidObjectInit(&pidPitchRate);
    pidObjectInit(&pidYawRate);
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
    pidUpdate(&pidRollRate, rollDesired - rollActual, rollDesired, pidUpdateDt);
    pidUpdate(&pidPitchRate, pitchDesired - pitchActual, pitchDesired, pidUpdateDt);
    pidUpdate(&pidYawRate, yawDesired - yawActual, yawDesired, pidUpdateDt);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
    pidUpdate(&pidRoll, rollDesired - rollActual, rollDesired, pidUpdateDt);
    pidUpdate(&pidPitch, pitchDesired - pitchActual, pitchDesired, pidUpdateDt);
    pidUpdate(&pidYaw, yawDesired - yawActual, yawDesired, pidUpdateDt);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual) {
    (void)rollActual; (void)pitchActual; (void)yawActual;
    pidReset(&pidRoll);
    pidReset(&pidPitch);
    pidReset(&pidYaw);
    pidReset(&pidRollRate);
    pidReset(&pidPitchRate);
    pidReset(&pidYawRate);
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
    (void)rollActual;
    pidReset(&pidRoll);
    pidReset(&pidRollRate);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
    (void)pitchActual;
    pidReset(&pidPitch);
    pidReset(&pidPitchRate);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw) {
    if (!roll || !pitch || !yaw) return;
    *roll = saturateSignedInt16((int32_t)lroundf(pidRollRate.output));
    *pitch = saturateSignedInt16((int32_t)lroundf(pidPitchRate.output));
    *yaw = saturateSignedInt16((int32_t)lroundf(pidYawRate.output));
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
    if (!setpoint || !state) return 0;

    float zError = setpoint->position.z - state->position.z;
    float zVelError = setpoint->velocity.z - state->velocity.z;

    /* Placeholder position-controller output boundary.  This is deliberately
       not just `setpoint->thrust`; it can be replaced by host injection if
       needed while preserving the contract. */
    float thrustF = 30000.0f + 100.0f * zError - 25.0f * zVelError;
    if (thrustF < 0.0f) thrustF = 0.0f;
    if (thrustF > 65535.0f) thrustF = 65535.0f;
    return (uint16_t)thrustF;
}

static float controllerDesiredYaw = 0.0f;
static bool controllerDesiredYawInitialized = false;

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
    if (!sensors || !setpoint || !state || !control) return;

    /* Thrust = 0 forces full reset */
    if (setpoint->thrust == 0) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0;
        attitudeControllerResetAllPID(state->attitude.roll,
                                      state->attitude.pitch,
                                      state->attitude.yaw);
        controllerDesiredYaw = state->attitude.yaw;
        controllerDesiredYawInitialized = true;
        return;
    }

    float yawActual = sensors->gyro.z;
    float pitchActual = -sensors->gyro.y;
    float rollActual = sensors->gyro.x;

    /* Desired yaw angle */
    if (!controllerDesiredYawInitialized) {
        controllerDesiredYaw = state->attitude.yaw;
        controllerDesiredYawInitialized = true;
    }

    if (setpoint->mode.quat == modeAbs) {
        float r, p, y;
        sensfusion6GetEulerRPY(&r, &p, &y);
        controllerDesiredYaw = y;
    } else if (setpoint->mode.yaw == modeVelocity) {
        controllerDesiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
    } else if (setpoint->mode.yaw == modeAbs) {
        controllerDesiredYaw = setpoint->attitude.yaw;
    }

    if (yawMaxDelta != 0.0f) {
        float minYaw = state->attitude.yaw - yawMaxDelta;
        float maxYaw = state->attitude.yaw + yawMaxDelta;
        if (controllerDesiredYaw > maxYaw) controllerDesiredYaw = maxYaw;
        if (controllerDesiredYaw < minYaw) controllerDesiredYaw = minYaw;
    }

    /* Roll and pitch rate desired */
    float rollRateDesired = 0.0f;
    float pitchRateDesired = 0.0f;

    if (setpoint->mode.roll == modeVelocity) {
        pidReset(&pidRoll);
        rollRateDesired = setpoint->attitudeRate.roll;
    } else if (setpoint->mode.roll == modeAbs) {
        rollRateDesired = pidUpdate(&pidRoll,
                                     setpoint->attitude.roll - state->attitude.roll,
                                     setpoint->attitude.roll,
                                     pidUpdateDt);
    } else {
        rollRateDesired = setpoint->attitudeRate.roll;
    }

    if (setpoint->mode.pitch == modeVelocity) {
        pidReset(&pidPitch);
        pitchRateDesired = setpoint->attitudeRate.pitch;
    } else if (setpoint->mode.pitch == modeAbs) {
        pitchRateDesired = pidUpdate(&pidPitch,
                                      setpoint->attitude.pitch - state->attitude.pitch,
                                      setpoint->attitude.pitch,
                                      pidUpdateDt);
    } else {
        pitchRateDesired = setpoint->attitudeRate.pitch;
    }

    float yawRateDesired = 0.0f;
    if (setpoint->mode.yaw == modeVelocity) {
        pidReset(&pidYaw);
        yawRateDesired = setpoint->attitudeRate.yaw;
    } else {
        yawRateDesired = pidUpdate(&pidYaw,
                                    controllerDesiredYaw - state->attitude.yaw,
                                    controllerDesiredYaw,
                                    pidUpdateDt);
    }

    attitudeControllerCorrectRatePID(rollActual, rollRateDesired,
                                     pitchActual, pitchRateDesired,
                                     yawActual, yawRateDesired);
    attitudeControllerGetActuatorOutput(&control->roll, &control->pitch,
                                        &control->yaw);

    if (control->controlMode == controlModeLegacy) {
        control->yaw = (int16_t)(-control->yaw);
    }

    if (setpoint->mode.z == modeDisable) {
        control->thrust = setpoint->thrust;
    } else {
        control->thrust = positionControllerUpdate(setpoint, state);
    }
}

/* -------------------------------------------------------------------------
 * 6. Commander RPYT decode
 * ---------------------------------------------------------------------- */

bool thrustLocked = false;
bool commanderModeSet = false;

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
    if (!rollPrime || !pitchPrime) return;
    float rad = yaw_deg * DEG2RAD;
    float s = sinf(rad);
    float c = cosf(rad);
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

    float roll = values->roll;
    float pitch = values->pitch;
    float yaw = values->yaw;
    uint16_t rawThrust = values->thrust;

    /* DISABLE priority lock is handled by commanderSetSetpoint and this
       explicit lock signal.  Raw thrust 0 releases the lock. */
    if (thrustLocked && rawThrust == 0) {
        thrustLocked = false;
    }

    /* Compute common thrust for non-AltHold modes */
    uint16_t effectiveThrust = 0;
    bool commonThrustZero = false;
    if (!altHoldMode) {
        if (thrustLocked || rawThrust < MIN_THRUST) {
            commonThrustZero = true;
        } else {
            effectiveThrust = rawThrust > MAX_THRUST ? MAX_THRUST : rawThrust;
        }
    }

    if (altHoldMode) {
        if (!commanderModeSet) {
            commanderModeSet = true;
            /* First entry: position PID/filter reset would be performed here */
        }
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0;
        setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
    } else {
        if (commanderModeSet) {
            setpoint->mode.z = modeDisable;
            commanderModeSet = false;
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
            setpoint->thrust = commonThrustZero ? 0 : effectiveThrust;
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
        } else {
            /* Default stable mode */
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

            setpoint->thrust = commonThrustZero ? 0 : effectiveThrust;
        }
    }

    /* Coordinate-frame rotation */
    if (yawMode == PLUSMODE) {
        float rp, pp;
        rotateYaw(roll, pitch, 45.0f, &rp, &pp);
        setpoint->attitude.roll = rp;
        setpoint->attitude.pitch = pp;
        if (setpoint->mode.roll == modeAbs) setpoint->attitude.roll = rp;
        if (setpoint->mode.pitch == modeAbs) setpoint->attitude.pitch = pp;
        if (setpoint->mode.roll == modeVelocity) setpoint->attitudeRate.roll = rp;
        if (setpoint->mode.pitch == modeVelocity) setpoint->attitudeRate.pitch = pp;
    } else if (yawMode == XMODE) {
        /* no rotation */
    } else {
        /* CAREFREE observable error path: clear setpoint as a safe fallback */
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.z = modeDisable;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->mode.yaw = modeDisable;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->attitude.yaw = 0.0f;
        setpoint->attitudeRate.roll = 0.0f;
        setpoint->attitudeRate.pitch = 0.0f;
        setpoint->attitudeRate.yaw = 0.0f;
        setpoint->thrust = 0;
    }

    setpoint->mode.quat = modeDisable;
}

/* -------------------------------------------------------------------------
 * 7. Supervisor
 * ---------------------------------------------------------------------- */

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0;

static bool supervisorArmed = false;
static bool supervisorCrashed = false;
static bool supervisorTumbled = false;
static bool supervisorFlying = false;
static bool supervisorFreeFall = false;
static bool supervisorAutoArming = false;
static uint32_t supervisorSpinupTimeoutMs = 0;
static uint32_t supervisorSpinupStartTick = 0;

static uint32_t supervisorCurrentTick = 0;

static SensorData supervisorSensors;
static uint32_t supervisorMotorRatios[4] = {0, 0, 0, 0};
static uint32_t supervisorIdleThrustVal = 0;
static int32_t supervisorMotorRpms[4] = {0, 0, 0, 0};

static float safetyCrashDetectionGs = 0.0f;
static float safetyFreeFallThreshold = 0.0f;
static float safetyAcceptedTiltAccZ = 0.0f;
static float safetyAcceptedUpsideDownAccZ = 0.0f;
static uint32_t safetyMaxTiltTime = 0;
static uint32_t safetyMaxUpsideDownTime = 0;
static bool safetyTumbleCheckEnabled = false;

static bool crtpEmergencyStopFlag = false;
static bool paramEmergencyStopFlag = false;
static bool emergencyWatchdogFailedFlag = false;

static uint32_t lastFlightTick = 0;
static bool seenFlightTick = false;

static uint32_t tumbleStartTick = 0;
static bool tumbleTimerActive = false;
static uint32_t activeTumbleTimeout = 0;

static uint32_t notRespondingStartTick = 0;
static bool motorsNotRespondingFlag = false;

static uint32_t lastCommanderNotificationTick = 0;

void supervisorInit(void) {
    supervisorState = supervisorStateLocked;
    supervisorConditionBits = 0;
    supervisorArmed = false;
    supervisorCrashed = false;
    supervisorTumbled = false;
    supervisorFlying = false;
    supervisorFreeFall = false;
    supervisorSpinupStartTick = 0;
    lastFlightTick = 0;
    seenFlightTick = false;
    tumbleStartTick = 0;
    tumbleTimerActive = false;
    notRespondingStartTick = 0;
    motorsNotRespondingFlag = false;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
    supervisorAutoArming = autoArming;
    supervisorSpinupTimeoutMs = spinupTimeoutDurationMs;
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled) {
    safetyCrashDetectionGs = crashDetectionGs;
    safetyFreeFallThreshold = freeFallThreshold;
    safetyAcceptedTiltAccZ = acceptedTiltAccZ;
    safetyAcceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
    safetyMaxTiltTime = maxTiltTime;
    safetyMaxUpsideDownTime = maxUpsideDownTime;
    safetyTumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorSetSensorData(const SensorData *sensors) {
    if (!sensors) return;
    supervisorSensors = *sensors;

    gyro.x = sensors->gyro.x;
    gyro.y = sensors->gyro.y;
    gyro.z = sensors->gyro.z;
    acc.x = sensors->acc.x;
    acc.y = sensors->acc.y;
    acc.z = sensors->acc.z;
    baro.pressure = sensors->baroPressure;
    baro.temp = sensors->baroTemperature;
    baro.asl = sensors->baroAsl;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
    if (!motorRatios) return;
    for (int i = 0; i < 4; i++) supervisorMotorRatios[i] = motorRatios[i];
    supervisorIdleThrustVal = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
    if (!motorRPMs) return;
    for (int i = 0; i < 4; i++) supervisorMotorRpms[i] = motorRPMs[i];
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
    return supervisorArmed;
}

bool supervisorIsCrashed(void) {
    return supervisorCrashed;
}

bool supervisorRequestArming(bool doArm) {
    if (doArm) {
        if (!supervisorCanArm() && supervisorState != supervisorStateArming) {
            return false;
        }
        if (!supervisorArmed) {
            supervisorArmed = true;
            supervisorState = supervisorStateArming;
        }
        supervisorConditionBits |= SUPERVISOR_CB_ARMED;
        return true;
    } else {
        supervisorArmed = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
        return true;
    }
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
    if (supervisorTumbled) return false;

    if (doRecovery) {
        supervisorCrashed = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    } else {
        supervisorCrashed = true;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    }
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
    uint16_t info = 0;
    if (supervisorCanArm()) info |= (1U << 0);
    if (supervisorArmed) info |= (1U << 1);
    if (supervisorAutoArming) info |= (1U << 2);
    if (supervisorCanFly()) info |= (1U << 3);
    if (supervisorFlying) info |= (1U << 4);
    if (supervisorTumbled) info |= (1U << 5);
    if (supervisorState == supervisorStateLocked) info |= (1U << 6);
    if (supervisorCrashed) info |= (1U << 7);
    if ((supervisorConditionBits & (1UL << 8)) != 0) info |= (1U << 8);   /* trajectoryFlying */
    if ((supervisorConditionBits & (1UL << 9)) != 0) info |= (1U << 9);   /* trajectoryFinished */
    if ((supervisorConditionBits & (1UL << 10)) != 0) info |= (1U << 10); /* trajectoryDisabled */
    if ((supervisorConditionBits & SUPERVISOR_CB_DECK_FAULT) != 0) info |= (1U << 11);
    return info;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick) {
    if (!motorRatios) return false;

    for (int i = 0; i < 4; i++) {
        if (motorRatios[i] > idleThrust) {
            lastFlightTick = currentTick;
            seenFlightTick = true;
            return true;
        }
    }

    if (!seenFlightTick) return false;
    return (currentTick - lastFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling) {
    if (isFreeFalling) *isFreeFalling = false;

    if (!tumbleCheckEnabled) {
        tumbleStartTick = 0;
        tumbleTimerActive = false;
        supervisorTumbled = false;
        return false;
    }

    float ax = fabsf(accX);
    float ay = fabsf(accY);
    float az = fabsf(accZ);

    if (ax < freeFallThreshold && ay < freeFallThreshold && az < freeFallThreshold) {
        if (isFreeFalling) *isFreeFalling = true;
        tumbleStartTick = 0;
        tumbleTimerActive = false;
        supervisorFreeFall = true;
        return false;
    }
    supervisorFreeFall = false;

    float accNorm = sqrtf(accX * accX + accY * accY + accZ * accZ);
    if (crashDetectionGs > 0.0f &&
        fabsf(accNorm - 1.0f) > crashDetectionGs) {
        supervisorCrashed = true;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    }

    if (accZ < acceptedTiltAccZ) {
        uint32_t timeout = maxTiltTime;
        if (accZ < acceptedUpsideDownAccZ) timeout = maxUpsideDownTime;

        if (!tumbleTimerActive) {
            tumbleStartTick = currentTick;
            tumbleTimerActive = true;
            activeTumbleTimeout = timeout;
        } else if (timeout != activeTumbleTimeout) {
            activeTumbleTimeout = timeout;
        }

        if ((currentTick - tumbleStartTick) >= activeTumbleTimeout) {
            supervisorTumbled = true;
            supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
            return true;
        }
    } else {
        tumbleStartTick = 0;
        tumbleTimerActive = false;
    }

    return supervisorTumbled;
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
    if (state != supervisorStateReadyToFly) return false;
    if (latestArmingTick == 0) return false;
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
    crtpEmergencyStopFlag = crtpEmergencyStop;
    paramEmergencyStopFlag = paramEmergencyStop;
    emergencyWatchdogFailedFlag = emergencyStopWatchdogFailed;

    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
    }

    if (supervisorArmed) supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    else supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;

    if (supervisorFlying) supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
    else supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;

    if (supervisorTumbled) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    else supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;

    if (supervisorCrashed) supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    else supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;

    if (supervisorFreeFall) supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
    else supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

    return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBits_,
                                SupervisorState state) {
    if (!setpoint) return;
    (void)supervisorConditionBits_;

    if (state == supervisorStateArming ||
        state == supervisorStateReadyToFly ||
        state == supervisorStateFlying ||
        state == supervisorStateLanded) {
        /* Do not modify */
        return;
    }

    if (state == supervisorStateWarningLevelOut) {
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = 0.0f;
        /* z is kept as-is */
        return;
    }

    /* All other states: zero/limited emergency setpoint */
    memset(setpoint, 0, sizeof(*setpoint));
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.z = modeDisable;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeDisable;
    setpoint->mode.quat = modeDisable;
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax) {
    if (!motorRPMs) return false;
    if (rpmCheckMin > rpmCheckMax) return false;
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
        notRespondingStartTick = 0;
        motorsNotRespondingFlag = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return false;
    }

    bool belowThreshold = false;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmThreshold) {
            belowThreshold = true;
            break;
        }
    }

    if (!belowThreshold) {
        notRespondingStartTick = 0;
        motorsNotRespondingFlag = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return false;
    }

    if (notRespondingStartTick == 0) {
        notRespondingStartTick = currentTick;
    } else if ((currentTick - notRespondingStartTick) >= rpmCheckDurationMs) {
        motorsNotRespondingFlag = true;
        supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    }

    return motorsNotRespondingFlag;
}

void supervisorUpdate(uint32_t stabilizerStep) {
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) {
        return;
    }

    supervisorCurrentTick = stabilizerStep;

    /* Flying / tumble / crash / freefall */
    supervisorFlying = isFlyingCheck(supervisorMotorRatios,
                                     supervisorIdleThrustVal,
                                     supervisorCurrentTick);

    bool freeFall = false;
    supervisorTumbled = isTumbledCheck(supervisorSensors.acc.x,
                                       supervisorSensors.acc.y,
                                       supervisorSensors.acc.z,
                                       safetyCrashDetectionGs,
                                       safetyFreeFallThreshold,
                                       safetyAcceptedTiltAccZ,
                                       safetyAcceptedUpsideDownAccZ,
                                       safetyMaxTiltTime,
                                       safetyMaxUpsideDownTime,
                                       safetyTumbleCheckEnabled,
                                       supervisorCurrentTick,
                                       &freeFall);
    supervisorFreeFall = freeFall;

    updateAndPopulateConditions(crtpEmergencyStopFlag,
                                paramEmergencyStopFlag,
                                emergencyWatchdogFailedFlag);

    /* Arming clear when leaving arming-holding states */
    if (supervisorArmed &&
        supervisorState != supervisorStateArming &&
        supervisorState != supervisorStateReadyToFly &&
        supervisorState != supervisorStateFlying &&
        supervisorState != supervisorStateWarningLevelOut &&
        supervisorState != supervisorStateLanded) {
        supervisorArmed = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    }

    /* Spinup timeout only in Arming */
    if (supervisorState == supervisorStateArming) {
        if (supervisorSpinupStartTick == 0) {
            supervisorSpinupStartTick = supervisorCurrentTick;
        }
        if ((supervisorCurrentTick - supervisorSpinupStartTick) >=
            supervisorSpinupTimeoutMs) {
            supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
        } else {
            supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
        }
    } else {
        supervisorSpinupStartTick = 0;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }

    /* Commander age warning/timeout */
    if (lastCommanderNotificationTick != 0 &&
        (supervisorCurrentTick - lastCommanderNotificationTick) >
            COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    } else if (lastCommanderNotificationTick != 0 &&
               (supervisorCurrentTick - lastCommanderNotificationTick) >
                   COMMANDER_WDT_TIMEOUT_STABILIZE) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    }

    /* Statistics/logs */
    supervisorLog.info = supervisorGetInfoBitfield();
    float a = supervisorSensors.acc.x;
    float b = supervisorSensors.acc.y;
    float c = supervisorSensors.acc.z;
    supervisorLog.accNorm = sqrtf(a * a + b * b + c * c);
}

/* -------------------------------------------------------------------------
 * 8. Estimator and Commander arbitration
 * ---------------------------------------------------------------------- */

#define ESTIMATOR_FIFO_CAP 16

static EstimatorMeasurement estimatorFifo[ESTIMATOR_FIFO_CAP];
static int estimatorFifoHead = 0;
static int estimatorFifoTail = 0;
static int estimatorFifoCount = 0;

static EstimatorMeasurement lastMeasurements[4];

bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
    if (!measurement) return false;
    if (estimatorFifoCount >= ESTIMATOR_FIFO_CAP) return false;

    estimatorFifo[estimatorFifoTail] = *measurement;
    estimatorFifoTail = (estimatorFifoTail + 1) % ESTIMATOR_FIFO_CAP;
    estimatorFifoCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
    if (!measurement) return false;
    if (estimatorFifoCount == 0) return false;

    *measurement = estimatorFifo[estimatorFifoHead];
    estimatorFifoHead = (estimatorFifoHead + 1) % ESTIMATOR_FIFO_CAP;
    estimatorFifoCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        if (m.type >= MeasurementTypeGyroscope && m.type <= MeasurementTypeTOF) {
            lastMeasurements[m.type] = m;
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        SensorData *sd = &supervisorSensors;
        /* Use last measurements if available; otherwise locally stored supervisor
           sensor data remains the best available host-visible input. */
        float gx = lastMeasurements[MeasurementTypeGyroscope].data[0];
        float gy = lastMeasurements[MeasurementTypeGyroscope].data[1];
        float gz = lastMeasurements[MeasurementTypeGyroscope].data[2];
        float ax = lastMeasurements[MeasurementTypeAcceleration].data[0];
        float ay = lastMeasurements[MeasurementTypeAcceleration].data[1];
        float az = lastMeasurements[MeasurementTypeAcceleration].data[2];

        if (measurementCountSince250 == 0) {
            gx = sd->gyro.x;
            gy = sd->gyro.y;
            gz = sd->gyro.z;
            ax = sd->acc.x;
            ay = sd->acc.y;
            az = sd->acc.z;
        }

        sensfusion6UpdateQ(gx, gy, gz, ax, ay, az, 1.0f / SENSFUSION_RATE_HZ);

        sensfusion6GetEulerRPY(&stateEstimate.roll,
                               &stateEstimate.pitch,
                               &stateEstimate.yaw);
        float qw_out, qx_out, qy_out, qz_out;
        sensfusion6GetQuaternion(&qw_out, &qx_out, &qy_out, &qz_out);
        stateEstimate.qx = qx_out;
        stateEstimate.qy = qy_out;
        stateEstimate.qz = qz_out;
        stateEstimate.qw = qw_out;

        sensfusion6Log.qw = qw_out;
        sensfusion6Log.qx = qx_out;
        sensfusion6Log.qy = qy_out;
        sensfusion6Log.qz = qz_out;
        sensfusion6Log.gravityX = gravityX;
        sensfusion6Log.gravityY = gravityY;
        sensfusion6Log.gravityZ = gravityZ;
        sensfusion6Log.accZbase = baseZacc;
        sensfusion6Log.isInit = sensfusion6IsInit;
        sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        /* Position update is a host boundary placeholder. */
    }
}

static int measurementCountSince250 = 0; /* not used in final; declared for clarity */

/* Commander arbitration */
static Setpoint commanderActiveSetpoint;
static int commanderActivePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t commanderLastUpdateTick = 0;

/* Host may supply this tick through a separate declaration if needed. */
uint32_t tick = 0;

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
    if (!setpoint) return false;

    if (priority == COMMANDER_PRIORITY_DISABLE) {
        commanderActiveSetpoint = *setpoint;
        commanderActivePriority = priority;
        commanderLastUpdateTick = tick;
        thrustLocked = true;
        return true;
    }

    if (priority < commanderActivePriority) {
        return false;
    }

    commanderActiveSetpoint = *setpoint;
    commanderActivePriority = priority;
    commanderLastUpdateTick = tick;

    if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
        /* Stop high-level trajectory */
    }

    return true;
}

void commanderRelaxPriority(void) {
    commanderActivePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
    return tick - commanderLastUpdateTick;
}

int commanderGetActivePriority(void) {
    return commanderActivePriority;
}

/* -------------------------------------------------------------------------
 * 9. Stabilizer and compressed state
 * ---------------------------------------------------------------------- */

static bool stabilizerInitialized = false;
static bool hasHighLevelSetpoint = false;
static Setpoint highLevelSetpoint;

static void sensorsInitStub(void) {}
static void stateEstimatorInitStub(void) {}
static void controllerInitStub(void) { attitudeControllerInit(1.0f / ATTITUDE_RATE_HZ); }
static void powerDistributionInitStub(void) {}
static void motorsInitStub(void) {}
static void collisionAvoidanceInitStub(void) {}

void stabilizerInit(void) {
    if (stabilizerInitialized) return;

    /* Observable init order */
    sensorsInitStub();
    stateEstimatorInitStub();
    controllerInitStub();
    powerDistributionInitStub();
    motorsInitStub();
    collisionAvoidanceInitStub();

    stabilizerInitialized = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
    if (!setpoint) return false;
    highLevelSetpoint = *setpoint;
    hasHighLevelSetpoint = true;
    return true;
}

/* Internal quaternion compression helper */
static uint32_t quatcompress(float qw_, float qx_, float qy_, float qz_) {
    float norm = sqrtf(qw_ * qw_ + qx_ * qx_ + qy_ * qy_ + qz_ * qz_);
    if (norm < 0.000001f) return 0;
    qw_ /= norm; qx_ /= norm; qy_ /= norm; qz_ /= norm;

    int16_t iw = (int16_t)lroundf(qw_ * 32767.0f);
    int16_t ix = (int16_t)lroundf(qx_ * 32767.0f);
    int16_t iy = (int16_t)lroundf(qy_ * 32767.0f);
    int16_t iz = (int16_t)lroundf(qz_ * 32767.0f);

    uint32_t c = ((uint16_t)iw << 16) | (uint16_t)ix;
    c = (c << 8) | ((uint16_t)iy & 0xFF);
    c = (c << 8) | ((uint16_t)iz & 0xFF);
    return c;
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output) {
    if (!state || !sensors || !output) return;

    output->position_mm[0] = (int32_t)lroundf(state->position.x * 1000.0f);
    output->position_mm[1] = (int32_t)lroundf(state->position.y * 1000.0f);
    output->position_mm[2] = (int32_t)lroundf(state->position.z * 1000.0f);

    output->velocity_mms[0] = (int32_t)lroundf(state->velocity.x * 1000.0f);
    output->velocity_mms[1] = (int32_t)lroundf(state->velocity.y * 1000.0f);
    output->velocity_mms[2] = (int32_t)lroundf(state->velocity.z * 1000.0f);

    output->acceleration_mms2[0] = (int32_t)lroundf(sensors->acc.x * 9810.0f);
    output->acceleration_mms2[1] = (int32_t)lroundf(sensors->acc.y * 9810.0f);
    output->acceleration_mms2[2] = (int32_t)lroundf((sensors->acc.z + 1.0f) * 9810.0f);

    output->gyro_millirad_s[0] = sensors->gyro.x * DEG2RAD * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * DEG2RAD * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * DEG2RAD * 1000.0f;

    output->quatCompressed = quatcompress(state->attitudeQuaternion.w,
                                          state->attitudeQuaternion.x,
                                          state->attitudeQuaternion.y,
                                          state->attitudeQuaternion.z);
}

bool rateSupervisorValidate(uint32_t measuredRate) {
    return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void) {
    /* The 2000ms supervisor wait/assert behavior is represented by the caller
       using rateSupervisorValidate(); this task is a host placeholder. */
}

void stabilizerTask(void) {
    if (!stabilizerInitialized) {
        stabilizerInit();
    }

    if (hasHighLevelSetpoint) {
        commanderSetSetpoint(&highLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        hasHighLevelSetpoint = false;
    }

    if (healthShallWeRunTest()) {
        SensorData sd = supervisorSensors;
        healthRunTests(&sd);
        return;
    }

    /* Normal control branch is host-invoked step-by-step through public APIs;
       this function does not run an infinite loop in the host model. */
}

/* -------------------------------------------------------------------------
 * 10. Health
 * ---------------------------------------------------------------------- */

TestState healthTestState = testDone;
uint8_t motorPass = 0;
uint8_t batteryPass = 0;
float batterySag = 0.0f;

static bool propTestRequested = false;
static bool batTestRequested = false;

static uint8_t propCurrentMotor = 0;
static float propNoiseSum = 0.0f;
static float propNoiseSumSq = 0.0f;
static int propNoiseCount = 0;
static float propIdleVoltage = 0.0f;
static float propMeasuredValue = 0.0f;
static float batIdleVoltage = 0.0f;
static float batMinLoadedVoltage = 0.0f;
static uint32_t batTestTick = 0;
static uint32_t batRestartTick = 0;

bool healthShallWeRunTest(void) {
    if (propTestRequested) {
        propTestRequested = false;
        healthTestState = configureAcc;
        propCurrentMotor = 0;
        propNoiseSum = 0.0f;
        propNoiseSumSq = 0.0f;
        propNoiseCount = 0;
        motorPass = 0;
        return true;
    }

    if (batTestRequested) {
        batTestRequested = false;
        healthTestState = testBattery;
        batTestTick = 0;
        batteryPass = 0;
        return true;
    }

    return healthTestState != testDone;
}

void healthRequestPropTest(void) {
    propTestRequested = true;
}

void healthRequestBatteryTest(void) {
    batTestRequested = true;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motor) {
    if (motor > 3) return false;
    if (highThreshold == 0.0f) return true;

    if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
        motorPass |= (1U << motor);
        return true;
    }
    return false;
}

float variance(const float *buffer, int length) {
    if (!buffer || length <= 0) return 0.0f;

    double sum = 0.0;
    double sumSq = 0.0;
    for (int i = 0; i < length; i++) {
        sum += buffer[i];
        sumSq += (double)buffer[i] * buffer[i];
    }

    return (float)(sumSq - (sum * sum / (double)length));
}

void healthRunTests(const SensorData *sensorData) {
    if (!sensorData) return;

    switch (healthTestState) {
    case configureAcc:
        motorPass = 0;
        propIdleVoltage = 0.0f; /* battery ADC not present in SensorData; host boundary */
        propNoiseCount = 0;
        propNoiseSum = 0.0f;
        propNoiseSumSq = 0.0f;
        healthTestState = measureNoiseFloor;
        break;

    case measureNoiseFloor: {
        float ax = sensorData->acc.x;
        float ay = sensorData->acc.y;
        float az = sensorData->acc.z;
        float norm = sqrtf(ax * ax + ay * ay + az * az);
        propNoiseSum += norm;
        propNoiseSumSq += norm * norm;
        propNoiseCount++;

        if (propNoiseCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
            propMeasuredValue = propNoiseSumSq -
                                (propNoiseSum * propNoiseSum /
                                 PROPTEST_NBR_OF_VARIANCE_VALUES);
            propCurrentMotor = 0;
            healthTestState = measureProp;
        }
        break;
    }

    case measureProp:
        /* Simplified host behavior: one test step per motor. */
        evaluatePropTest(0.0f, 0.0f, propMeasuredValue, propCurrentMotor);
        propCurrentMotor++;
        if (propCurrentMotor >= 4) {
            healthTestState = evaluatePropResult;
        }
        break;

    case evaluatePropResult:
        healthTestState = testDone;
        healthLog.motorPass = motorPass;
        healthLog.motorTestCount = 4;
        break;

    case testBattery:
        if (batTestTick == 0) {
            /* Placeholder for idle voltage acquisition. */
            batIdleVoltage = 4.0f;
            batMinLoadedVoltage = 4.0f;
        } else if (batTestTick == 1) {
            /* Four-motor loading would happen here. */
        } else if (batTestTick >= 2 && batTestTick <= 49) {
            float loaded = 4.0f;
            if (loaded < batMinLoadedVoltage) batMinLoadedVoltage = loaded;
        } else if (batTestTick >= 50) {
            batterySag = batIdleVoltage - batMinLoadedVoltage;
            batteryPass = 1;
            healthTestState = evaluateBatResult;
        }
        batTestTick++;
        break;

    case evaluateBatResult:
        healthLog.batteryPass = batteryPass;
        healthLog.batterySag = batterySag;
        healthTestState = testDone;
        break;

    case restartBatTest:
        if (batRestartTick >= 2000) {
            batRestartTick = 0;
            batTestTick = 0;
            healthTestState = testBattery;
        } else {
            batRestartTick++;
        }
        break;

    case testDone:
    default:
        break;
    }
}

/* -------------------------------------------------------------------------
 * 11. CRTP transport
 * ---------------------------------------------------------------------- */

#define CRTP_TX_CAP CRTP_TX_QUEUE_SIZE
#define CRTP_RX_CAP CRTP_RX_QUEUE_SIZE

static CrtpPacket txQueue[CRTP_TX_CAP];
static int txHead = 0;
static int txCount = 0;

typedef struct {
    bool active;
    CrtpPacket packets[CRTP_RX_CAP];
    int head;
    int count;
} CrtpRxQueue;

static CrtpRxQueue rxQueues[CRTP_NBR_OF_PORTS];
static CrtpPortCallback portCallbacks[CRTP_NBR_OF_PORTS];
static CrtpLink *activeLink = NULL;

static bool nopSendPacket(CrtpPacket *packet) { (void)packet; return false; }
static bool nopReceivePacket(CrtpPacket *packet) { (void)packet; return false; }
static bool nopIsConnected(void) { return true; }
static void nopSetEnable(bool enable) { (void)enable; }
static void nopReset(void) {}

static CrtpLink nopLink = {
    nopSendPacket,
    nopReceivePacket,
    nopIsConnected,
    nopSetEnable,
    nopReset
};

static bool crtpInitialized = false;
static uint32_t crtpLastStatsTick = 0;
static uint32_t crtpRxCount = 0;
static uint32_t crtpTxCount = 0;

void crtpInit(void) {
    if (crtpInitialized) return;

    txHead = 0;
    txCount = 0;
    memset(rxQueues, 0, sizeof(rxQueues));
    memset(portCallbacks, 0, sizeof(portCallbacks));
    activeLink = &nopLink;
    crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port) {
    if (!crtpInitialized) crtpInit();
    if (port >= CRTP_NBR_OF_PORTS) return;

    if (!rxQueues[port].active) {
        rxQueues[port].active = true;
        rxQueues[port].head = 0;
        rxQueues[port].count = 0;
    }
    /* Repeated creation on the same port is ignored in host model. */
}

bool crtpSendPacket(const CrtpPacket *packet) {
    if (!crtpInitialized) crtpInit();
    if (!packet) return false;
    if (txCount >= CRTP_TX_CAP) return false;

    int tail = (txHead + txCount) % CRTP_TX_CAP;
    txQueue[tail] = *packet;
    txCount++;
    crtpTxCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
    if (!crtpInitialized) crtpInit();
    if (!packet || port >= CRTP_NBR_OF_PORTS) return false;
    if (!rxQueues[port].active || rxQueues[port].count == 0) return false;

    *packet = rxQueues[port].packets[rxQueues[port].head];
    rxQueues[port].head = (rxQueues[port].head + 1) % CRTP_RX_CAP;
    rxQueues[port].count--;
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
    if (!activeLink || activeLink == &nopLink) return;

    CrtpPacket pkt;
    if (activeLink->receivePacket(&pkt)) {
        crtpRxCount++;
        bool delivered = false;
        if (pkt.port < CRTP_NBR_OF_PORTS && rxQueues[pkt.port].active) {
            int tail = (rxQueues[pkt.port].head + rxQueues[pkt.port].count) % CRTP_RX_CAP;
            if (rxQueues[pkt.port].count < CRTP_RX_CAP) {
                rxQueues[pkt.port].packets[tail] = pkt;
                rxQueues[pkt.port].count++;
                delivered = true;
            }
        }
        if (pkt.port < CRTP_NBR_OF_PORTS && portCallbacks[pkt.port]) {
            portCallbacks[pkt.port](&pkt);
            delivered = true;
        }
        if (!delivered) {
            /* packet dropped */
        }
    }
}

void crtpTxTask(void) {
    if (!crtpInitialized) crtpInit();
    if (!activeLink || activeLink == &nopLink || txCount == 0) return;

    CrtpPacket *pkt = &txQueue[txHead];
    if (activeLink->sendPacket(pkt)) {
        txHead = (txHead + 1) % CRTP_TX_CAP;
        txCount--;
        crtpTxCount++;
    }
    /* On failure packet remains at head for retry next call. */
}

void crtpSetLink(CrtpLink *newLink) {
    if (!crtpInitialized) crtpInit();

    if (activeLink && activeLink->setEnable) {
        activeLink->setEnable(false);
    }

    if (newLink == NULL) {
        activeLink = &nopLink;
    } else {
        activeLink = newLink;
    }

    if (activeLink->setEnable) {
        activeLink->setEnable(true);
    }
}

void crtpReset(void) {
    if (!crtpInitialized) crtpInit();
    txHead = 0;
    txCount = 0;
    if (activeLink && activeLink->reset) {
        activeLink->reset();
    }
}

bool crtpIsConnected(void) {
    if (!crtpInitialized) crtpInit();
    if (activeLink && activeLink->isConnected) {
        return activeLink->isConnected();
    }
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
    if (!crtpInitialized) crtpInit();
    return (uint32_t)(CRTP_TX_CAP - txCount);
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
    if (!crtpInitialized) crtpInit();
    if (port >= CRTP_NBR_OF_PORTS) return;
    portCallbacks[port] = callback;
}

void updateStats(void) {
    if (!crtpInitialized) crtpInit();

    if (tick - crtpLastStatsTick >= 500) {
        crtpLastStatsTick = tick;
        crtpRxCount = 0;
        crtpTxCount = 0;
    }
}

/* -------------------------------------------------------------------------
 * 12. Deck discovery and log globals
 * ---------------------------------------------------------------------- */

StateEstimateLog stateEstimate;
Axis3Log gyro, acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
    if (!decks || capacity == 0) return 0;

    /* Host model has no real I2C/OneWire inventory.  Return zero devices. */
    return 0;
}

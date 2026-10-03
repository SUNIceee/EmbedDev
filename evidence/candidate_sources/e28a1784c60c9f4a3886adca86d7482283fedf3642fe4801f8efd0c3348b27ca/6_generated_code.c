#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

/* Host-test millisecond tick. Not part of the frozen header, but kept as a
   non-static symbol so host test harnesses can advance deterministic time. */
uint32_t platformTickMs = 0U;

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int32_t clamp32(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ------------------------------------------------------------------------- */
/* Numeric helpers                                                           */
/* ------------------------------------------------------------------------- */

int16_t saturateSignedInt16(int32_t value) {
    if (value > 32767) return 32767;
    if (value < -32767) return -32767;
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

/* ------------------------------------------------------------------------- */
/* Sensfusion6                                                               */
/* ------------------------------------------------------------------------- */

float qw = 1.0f;
float qx = 0.0f;
float qy = 0.0f;
float qz = 0.0f;
float gravityX = 0.0f;
float gravityY = 0.0f;
float gravityZ = 1.0f;
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;
float twoKp = 0.8f;
float twoKi = 0.002f;
float beta = 0.01f;
float baseZacc = 0.0f;
bool sensfusion6IsInit = false;
bool sensfusion6IsCalibrated = false;

float invSqrt(float x) {
    if (x <= 0.0f) {
        return 0.0f;
    }
    union {
        float f;
        uint32_t i;
    } conv;
    conv.f = x;
    conv.i = 0x5f3759dfU - (conv.i >> 1);
    float y = conv.f;
    y = y * (1.5f - (0.5f * x * y * y));
    return y;
}

void estimatedGravityDirection(float w, float x, float y, float z,
                               float *gravX, float *gravY, float *gravZ) {
    if (!gravX || !gravY || !gravZ) {
        return;
    }
    float norm = w * w + x * x + y * y + z * z;
    if (norm < 1e-12f) {
        *gravX = 0.0f;
        *gravY = 0.0f;
        *gravZ = 1.0f;
        return;
    }
    *gravX = 2.0f * (x * z - w * y);
    *gravY = 2.0f * (w * x + y * z);
    *gravZ = w * w - x * x - y * y + z * z;
}

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
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
}

bool sensfusion6Test(void) {
    return sensfusion6IsInit;
}

void sensfusion6UpdateQ(float gx_deg, float gy_deg, float gz_deg,
                        float ax, float ay, float az, float dt) {
    if (!sensfusion6IsInit) {
        sensfusion6Init();
    }
    if (dt <= 0.0f) {
        return;
    }

    float gx_rad = gx_deg * DEG_TO_RAD;
    float gy_rad = gy_deg * DEG_TO_RAD;
    float gz_rad = gz_deg * DEG_TO_RAD;

    bool accValid = !(ax == 0.0f && ay == 0.0f && az == 0.0f);
    float axn = 0.0f;
    float ayn = 0.0f;
    float azn = 0.0f;

    if (accValid) {
        float recip = invSqrt(ax * ax + ay * ay + az * az);
        if (recip > 0.0f) {
            axn = ax * recip;
            ayn = ay * recip;
            azn = az * recip;
        }
    }

    if (accValid && !sensfusion6IsCalibrated) {
        float gx_est, gy_est, gz_est;
        estimatedGravityDirection(qw, qx, qy, qz, &gx_est, &gy_est, &gz_est);
        baseZacc = ax * gx_est + ay * gy_est + az * gz_est;
        sensfusion6IsCalibrated = true;
    }

    if (accValid) {
        float vx = 2.0f * (qx * qz - qw * qy);
        float vy = 2.0f * (qw * qx + qy * qz);
        float vz = 1.0f - 2.0f * (qx * qx + qy * qy);

        float ex = ayn * vz - azn * vy;
        float ey = azn * vx - axn * vz;
        float ez = axn * vy - ayn * vx;

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
    } else {
        if (twoKi == 0.0f) {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }
    }

    float halfDt = 0.5f * dt;
    float nw = qw + (-gx_rad * qx - gy_rad * qy - gz_rad * qz) * halfDt;
    float nx = qx + (gx_rad * qw + gy_rad * qz - gz_rad * qy) * halfDt;
    float ny = qy + (gy_rad * qw - gx_rad * qz + gz_rad * qx) * halfDt;
    float nz = qz + (gz_rad * qw + gx_rad * qy - gy_rad * qx) * halfDt;

    float norm = invSqrt(nw * nw + nx * nx + ny * ny + nz * nz);
    if (norm > 0.0f) {
        qw = nw * norm;
        qx = nx * norm;
        qy = ny * norm;
        qz = nz * norm;
    }

    sensfusion6Log.qw = qw;
    sensfusion6Log.qx = qx;
    sensfusion6Log.qy = qy;
    sensfusion6Log.qz = qz;
    sensfusion6Log.isInit = sensfusion6IsInit;
    sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;
    sensfusion6Log.accZbase = baseZacc;
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
    if (!roll_deg || !pitch_deg || !yaw_deg) {
        return;
    }

    float gx_sin = 2.0f * (qw * qy - qz * qx);
    gx_sin = clampf(gx_sin, -1.0f, 1.0f);

    float roll = atan2f(2.0f * (qw * qx + qy * qz),
                        1.0f - 2.0f * (qx * qx + qy * qy));
    float pitch = asinf(gx_sin);
    float yaw = atan2f(2.0f * (qw * qz + qx * qy),
                       1.0f - 2.0f * (qy * qy + qz * qz));

    *roll_deg = roll * RAD_TO_DEG;
    *pitch_deg = pitch * RAD_TO_DEG;
    *yaw_deg = yaw * RAD_TO_DEG;

    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
    sensfusion6Log.qw = qw;
    sensfusion6Log.qx = qx;
    sensfusion6Log.qy = qy;
    sensfusion6Log.qz = qz;
    sensfusion6Log.gravityX = gravityX;
    sensfusion6Log.gravityY = gravityY;
    sensfusion6Log.gravityZ = gravityZ;
    sensfusion6Log.accZbase = baseZacc;
    sensfusion6Log.isInit = sensfusion6IsInit;
    sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;
}

void sensfusion6GetQuaternion(float *w, float *x, float *y, float *z) {
    if (!w || !x || !y || !z) {
        return;
    }
    *w = qw;
    *x = qx;
    *y = qy;
    *z = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
    float gx_est, gy_est, gz_est;
    estimatedGravityDirection(qw, qx, qy, qz, &gx_est, &gy_est, &gz_est);
    return ax * gx_est + ay * gy_est + az * gz_est;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* ------------------------------------------------------------------------- */
/* Power distribution and battery                                            */
/* ------------------------------------------------------------------------- */

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out) {
    if (!out) {
        return;
    }
    int32_t r = (int32_t)roll / 2;
    int32_t p = (int32_t)pitch / 2;
    out->m1 = (int32_t)thrust - r + p + (int32_t)yaw;
    out->m2 = (int32_t)thrust - r - p - (int32_t)yaw;
    out->m3 = (int32_t)thrust + r - p + (int32_t)yaw;
    out->m4 = (int32_t)thrust + r + p - (int32_t)yaw;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4]) {
    if (!motorForces) {
        return;
    }
    float thrustPart = 0.25f * thrustSi;
    float arm = 0.707106781f * armLength;
    float rollPart = (arm != 0.0f) ? (0.25f / arm) * torqueX : 0.0f;
    float pitchPart = (arm != 0.0f) ? (0.25f / arm) * torqueY : 0.0f;
    float yawPart = (thrustToTorque != 0.0f) ? (0.25f / thrustToTorque) * torqueZ : 0.0f;

    float f[4];
    f[0] = thrustPart - rollPart + pitchPart + yawPart;
    f[1] = thrustPart - rollPart - pitchPart - yawPart;
    f[2] = thrustPart + rollPart - pitchPart + yawPart;
    f[3] = thrustPart + rollPart + pitchPart - yawPart;

    for (int i = 0; i < 4; ++i) {
        motorForces[i] = f[i] > 0.0f ? f[i] : 0.0f;
    }
}

static uint16_t motorForceToPwm(float force) {
    if (force <= 0.0f) {
        return 0U;
    }
    float pwm = (force / CRAZYFLIE_MAX_MOTOR_FORCE_N) * 65535.0f;
    if (pwm > 65535.0f) {
        pwm = 65535.0f;
    }
    return (uint16_t)(pwm + 0.5f);
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4]) {
    if (!normalizedForces || !motorPWMs) {
        return;
    }
    for (int i = 0; i < 4; ++i) {
        float f = clampf(normalizedForces[i], 0.0f, 1.0f);
        motorPWMs[i] = (uint16_t)(f * 65535.0f);
    }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower) {
    if (!control || !motorPower) {
        return;
    }
    switch (control->controlMode) {
        case controlModeLegacy:
            powerDistributionLegacy(control->thrust, control->roll, control->pitch,
                                    control->yaw, motorPower);
            break;
        case controlModeForceTorque: {
            float forces[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            powerDistributionForceTorque(control->thrustSi,
                                         control->torque.x,
                                         control->torque.y,
                                         control->torque.z,
                                         CRAZYFLIE_ARM_LENGTH_M,
                                         CRAZYFLIE_THRUST_TO_TORQUE,
                                         forces);
            motorPower->m1 = (int32_t)motorForceToPwm(forces[0]);
            motorPower->m2 = (int32_t)motorForceToPwm(forces[1]);
            motorPower->m3 = (int32_t)motorForceToPwm(forces[2]);
            motorPower->m4 = (int32_t)motorForceToPwm(forces[3]);
            break;
        }
        case controlModeForce: {
            uint16_t pwms[4] = {0U, 0U, 0U, 0U};
            powerDistributionForce(control->normalizedForces, pwms);
            motorPower->m1 = (int32_t)pwms[0];
            motorPower->m2 = (int32_t)pwms[1];
            motorPower->m3 = (int32_t)pwms[2];
            motorPower->m4 = (int32_t)pwms[3];
            break;
        }
        default:
            break;
    }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) {
    if (value < idleThrust) {
        return idleThrust;
    }
    return value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust) {
    PowerCapResult result = {false, 0};
    if (!motors) {
        return result;
    }
    int32_t maxVal = motors[0];
    for (int i = 1; i < 4; ++i) {
        if (motors[i] > maxVal) {
            maxVal = motors[i];
        }
    }
    if (maxVal > maxAllowedThrust) {
        result.reduction = maxVal - maxAllowedThrust;
        result.isCapped = true;
        for (int i = 0; i < 4; ++i) {
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
    if (actualVoltage <= 0.0f) {
        return motorThrust;
    }
    float compensated = (float)motorThrust * nominalVoltage / actualVoltage;
    if (compensated < 0.0f) {
        compensated = 0.0f;
    }
    if (compensated > 65535.0f) {
        compensated = 65535.0f;
    }
    return (uint16_t)(compensated + 0.5f);
}

/* ------------------------------------------------------------------------- */
/* Cascade PID                                                               */
/* ------------------------------------------------------------------------- */

PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;
PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;

static float pidControllerDt = 0.002f;
static bool pidControllerInitialized = false;

static void pidResetToZero(PidObject *pid) {
    if (!pid) {
        return;
    }
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
    pid->initialized = true;
}

static void pidResetToValue(PidObject *pid, float value) {
    if (!pid) {
        return;
    }
    pid->integral = 0.0f;
    pid->prevError = value;
    pid->output = 0.0f;
    pid->initialized = true;
}

static float pidUpdate(PidObject *pid, float error, float dt) {
    if (!pid) {
        return 0.0f;
    }
    if (dt <= 0.0f) {
        dt = pidControllerDt;
    }
    if (!pid->initialized) {
        pid->integral = 0.0f;
        pid->prevError = error;
        pid->output = 0.0f;
        pid->initialized = true;
    }
    pid->integral += error * dt;
    float derivative = (error - pid->prevError) / dt;
    pid->prevError = error;
    pid->output = pid->kp * error + pid->ki * pid->integral +
                  pid->kd * derivative + pid->kff * error;
    return pid->output;
}

void attitudeControllerInit(float updateDt) {
    if (pidControllerInitialized) {
        return;
    }
    if (updateDt > 0.0f) {
        pidControllerDt = updateDt;
    }
    pidResetToZero(&pidRoll);
    pidResetToZero(&pidPitch);
    pidResetToZero(&pidYaw);
    pidResetToZero(&pidRollRate);
    pidResetToZero(&pidPitchRate);
    pidResetToZero(&pidYawRate);
    pidControllerInitialized = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
    float rollOut = pidUpdate(&pidRollRate, rollDesired - rollActual, pidControllerDt);
    float pitchOut = pidUpdate(&pidPitchRate, pitchDesired - pitchActual, pidControllerDt);
    float yawOut = pidUpdate(&pidYawRate, yawDesired - yawActual, pidControllerDt);
    pidRollRate.output = saturateSignedInt16((int32_t)rollOut);
    pidPitchRate.output = saturateSignedInt16((int32_t)pitchOut);
    pidYawRate.output = saturateSignedInt16((int32_t)yawOut);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
    float rollErr = capAngle(rollDesired - rollActual);
    float pitchErr = capAngle(pitchDesired - pitchActual);
    float yawErr = capAngle(yawDesired - yawActual);
    pidUpdate(&pidRoll, rollErr, pidControllerDt);
    pidUpdate(&pidPitch, pitchErr, pidControllerDt);
    pidUpdate(&pidYaw, yawErr, pidControllerDt);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual) {
    pidResetToValue(&pidRoll, rollActual);
    pidResetToValue(&pidPitch, pitchActual);
    pidResetToValue(&pidYaw, yawActual);
    pidResetToZero(&pidRollRate);
    pidResetToZero(&pidPitchRate);
    pidResetToZero(&pidYawRate);
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
    pidResetToValue(&pidRoll, rollActual);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
    pidResetToValue(&pidPitch, pitchActual);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw) {
    if (!roll || !pitch || !yaw) {
        return;
    }
    *roll = (int16_t)pidRollRate.output;
    *pitch = (int16_t)pidPitchRate.output;
    *yaw = (int16_t)pidYawRate.output;
}

/* ------------------------------------------------------------------------- */
/* Position controller                                                       */
/* ------------------------------------------------------------------------- */

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
    if (!setpoint || !state) {
        return 0U;
    }
    if (setpoint->mode.z == modeDisable) {
        return 0U;
    }

    float target = 0.0f;
    if (setpoint->mode.z == modeAbs) {
        float error = setpoint->position.z - state->position.z;
        target = 40000.0f + 150.0f * error;
    } else if (setpoint->mode.z == modeVelocity) {
        float error = setpoint->velocity.z - state->velocity.z;
        target = 40000.0f + 150.0f * error;
    }

    if (target < 0.0f) {
        target = 0.0f;
    }
    if (target > 65535.0f) {
        target = 65535.0f;
    }
    return (uint16_t)target;
}

/* ------------------------------------------------------------------------- */
/* controllerPid                                                             */
/* ------------------------------------------------------------------------- */

static float controllerDesiredYaw = 0.0f;
static bool controllerDesiredYawValid = false;

static void controllerSyncYaw(float yaw) {
    controllerDesiredYaw = yaw;
    controllerDesiredYawValid = true;
}

static float quaternionToYaw(const Quaternion *q) {
    if (!q) {
        return 0.0f;
    }
    return atan2f(2.0f * (q->w * q->z + q->x * q->y),
                  1.0f - 2.0f * (q->y * q->y + q->z * q->z)) * RAD_TO_DEG;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
    if (!sensors || !setpoint || !state || !control) {
        return;
    }

    control->controlMode = controlModeLegacy;
    control->thrustSi = 0.0f;
    control->torque.x = 0.0f;
    control->torque.y = 0.0f;
    control->torque.z = 0.0f;
    for (int i = 0; i < 4; ++i) {
        control->normalizedForces[i] = 0.0f;
    }

    uint16_t thrustOut;
    if (setpoint->mode.z == modeDisable) {
        thrustOut = setpoint->thrust;
    } else {
        thrustOut = positionControllerUpdate(setpoint, state);
    }

    if (thrustOut == 0U) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0U;
        attitudeControllerResetAllPID(state->attitude.roll,
                                      state->attitude.pitch,
                                      state->attitude.yaw);
        controllerSyncYaw(state->attitude.yaw);
        return;
    }

    if (!controllerDesiredYawValid) {
        controllerSyncYaw(state->attitude.yaw);
    }

    if (setpoint->mode.yaw == modeVelocity) {
        controllerDesiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
    } else if (setpoint->mode.yaw == modeAbs) {
        if (setpoint->mode.quat == modeAbs) {
            controllerDesiredYaw = quaternionToYaw(&setpoint->attitudeQuaternion);
        } else {
            controllerDesiredYaw = setpoint->attitude.yaw;
        }
    } else {
        controllerDesiredYaw = state->attitude.yaw;
    }

    if (yawMaxDelta != 0.0f) {
        float yawDiff = capAngle(controllerDesiredYaw - state->attitude.yaw);
        if (yawDiff > yawMaxDelta) {
            controllerDesiredYaw = state->attitude.yaw + yawMaxDelta;
        } else if (yawDiff < -yawMaxDelta) {
            controllerDesiredYaw = state->attitude.yaw - yawMaxDelta;
        }
    }

    float rollRateCmd;
    if (setpoint->mode.roll == modeVelocity) {
        rollRateCmd = setpoint->attitudeRate.roll;
        attitudeControllerResetRollAttitudePID(state->attitude.roll);
    } else {
        float rollErr = capAngle(setpoint->attitude.roll - state->attitude.roll);
        rollRateCmd = pidUpdate(&pidRoll, rollErr, attitudeUpdateDt);
    }

    float pitchRateCmd;
    if (setpoint->mode.pitch == modeVelocity) {
        pitchRateCmd = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
    } else {
        float pitchErr = capAngle(setpoint->attitude.pitch - state->attitude.pitch);
        pitchRateCmd = pidUpdate(&pidPitch, pitchErr, attitudeUpdateDt);
    }

    float yawErr = capAngle(controllerDesiredYaw - state->attitude.yaw);
    float yawRateCmd = pidUpdate(&pidYaw, yawErr, attitudeUpdateDt);

    float rollActual = sensors->gyro.x;
    float pitchActual = -sensors->gyro.y;
    float yawActual = sensors->gyro.z;

    float rollOut = pidUpdate(&pidRollRate, rollRateCmd - rollActual, attitudeUpdateDt);
    float pitchOut = pidUpdate(&pidPitchRate, pitchRateCmd - pitchActual, attitudeUpdateDt);
    float yawOut = pidUpdate(&pidYawRate, yawRateCmd - yawActual, attitudeUpdateDt);

    control->roll = saturateSignedInt16((int32_t)rollOut);
    control->pitch = saturateSignedInt16((int32_t)pitchOut);
    control->yaw = saturateSignedInt16((int32_t)yawOut);
    control->thrust = thrustOut;

    /* Legacy coordinate frame negates yaw actuator output. */
    control->yaw = (int16_t)(-control->yaw);
}

/* ------------------------------------------------------------------------- */
/* CRTP Commander RPYT                                                       */
/* ------------------------------------------------------------------------- */

bool thrustLocked = false;
bool commanderModeSet = false;

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
    if (!rollPrime || !pitchPrime) {
        return;
    }
    float rad = yaw_deg * DEG_TO_RAD;
    float c = cosf(rad);
    float s = sinf(rad);
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
    if (!values || !setpoint) {
        return;
    }

    if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
    }
    if (values->thrust == 0U) {
        thrustLocked = false;
    }

    *setpoint = (Setpoint){0};

    if (altHoldMode) {
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0U;
        setpoint->velocity.z = ((float)values->thrust - 32767.0f) / 32767.0f;
        if (!commanderModeSet) {
            commanderModeSet = true;
            /* Position/filter reset occurs here in the full implementation. */
        }
        return;
    }

    if (commanderModeSet) {
        commanderModeSet = false;
        setpoint->mode.z = modeDisable;
    }

    if (posSetMode && values->thrust != 0U) {
        setpoint->mode.x = modeAbs;
        setpoint->mode.y = modeAbs;
        setpoint->mode.z = modeAbs;
        setpoint->mode.yaw = modeAbs;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->position.x = -values->pitch;
        setpoint->position.y = values->roll;
        setpoint->position.z = (float)values->thrust / 1000.0f;
        setpoint->attitude.yaw = values->yaw;
        setpoint->thrust = 0U;
        return;
    }

    if (posHoldMode) {
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->velocity.x = values->pitch / 30.0f;
        setpoint->velocity.y = values->roll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        if (thrustLocked || values->thrust < MIN_THRUST) {
            setpoint->thrust = 0U;
        } else {
            setpoint->thrust = values->thrust > MAX_THRUST ? MAX_THRUST : values->thrust;
        }
        return;
    }

    if (yawMode == CAREFREE) {
        static bool carefreeErrorFlag = false;
        carefreeErrorFlag = true;
        setpoint->thrust = 0U;
        return;
    }

    float cmdRoll = values->roll;
    float cmdPitch = values->pitch;
    if (yawMode == PLUSMODE) {
        rotateYaw(cmdRoll, cmdPitch, 45.0f, &cmdRoll, &cmdPitch);
    }

    if (stabilizationModeRoll == RATE) {
        setpoint->mode.roll = modeVelocity;
        setpoint->attitudeRate.roll = cmdRoll;
    } else {
        setpoint->mode.roll = modeAbs;
        setpoint->attitude.roll = cmdRoll;
    }

    if (stabilizationModePitch == RATE) {
        setpoint->mode.pitch = modeVelocity;
        setpoint->attitudeRate.pitch = cmdPitch;
    } else {
        setpoint->mode.pitch = modeAbs;
        setpoint->attitude.pitch = cmdPitch;
    }

    if (stabilizationModeYaw == RATE) {
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = -values->yaw;
    } else {
        setpoint->mode.yaw = modeAbs;
        setpoint->attitude.yaw = values->yaw;
    }

    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.z = modeDisable;

    if (thrustLocked || values->thrust < MIN_THRUST) {
        setpoint->thrust = 0U;
    } else {
        setpoint->thrust = values->thrust > MAX_THRUST ? MAX_THRUST : values->thrust;
    }
}

/* ------------------------------------------------------------------------- */
/* Supervisor                                                                 */
/* ------------------------------------------------------------------------- */

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0U;

static SensorData supervisorSensorData;
static uint32_t supervisorMotorRatios[4] = {0U, 0U, 0U, 0U};
static uint32_t supervisorIdleThrust = 0U;
static int32_t supervisorMotorRPMs[4] = {0, 0, 0, 0};
static float supCrashDetectionGs = 0.0f;
static float supFreeFallThreshold = 0.0f;
static float supTiltAccZ = 0.0f;
static float supUpsideDownAccZ = 0.0f;
static uint32_t supMaxTiltTime = 0U;
static uint32_t supMaxUpsideDownTime = 0U;
static bool supTumbleEnabled = true;
static bool supAutoArming = false;
static uint32_t supSpinupDurationMs = 0U;
static bool supArmed = false;
static bool supCrashFlag = false;
static bool supTumbledFlag = false;
static bool supFlyingFlag = false;
static bool supFreeFallingFlag = false;
static bool supRpmValid = false;
static bool supMotorsNotRespondingFlag = false;
static bool supSpinupTimeoutFlag = false;
static bool supDeckFault = false;
static bool supervisorTrajectoryFlying = false;
static bool supervisorTrajectoryFinished = false;
static bool supervisorTrajectoryDisabled = false;
static bool supervisorInitialized = false;
static SupervisorState supervisorPrevState = supervisorStateLocked;
static bool supervisorPrevStateValid = false;
static uint32_t supArmingStartTick = 0U;
static bool supArmingStartValid = false;
static uint32_t supRecentFlyingTick = 0U;
static bool supSeenFlying = false;
static uint32_t supTumbleStartTick = 0U;
static bool supTumbleTimerActive = false;
static uint32_t supNotRespondingStartTick = 0U;
static bool supNotRespondingTimerActive = false;
static uint32_t supervisorLastEmergencyStopNotificationTick = 0U;

void supervisorInit(void) {
    if (supervisorInitialized) {
        return;
    }
    supervisorState = supervisorStateLocked;
    supervisorPrevState = supervisorStateLocked;
    supervisorPrevStateValid = false;
    supervisorConditionBits = 0U;
    supArmed = false;
    supCrashFlag = false;
    supTumbledFlag = false;
    supFlyingFlag = false;
    supFreeFallingFlag = false;
    supRpmValid = false;
    supMotorsNotRespondingFlag = false;
    supSpinupTimeoutFlag = false;
    supDeckFault = false;
    supervisorTrajectoryFlying = false;
    supervisorTrajectoryFinished = false;
    supervisorTrajectoryDisabled = false;
    supArmingStartValid = false;
    supArmingStartTick = 0U;
    supRecentFlyingTick = 0U;
    supSeenFlying = false;
    supTumbleTimerActive = false;
    supTumbleStartTick = 0U;
    supNotRespondingTimerActive = false;
    supNotRespondingStartTick = 0U;
    supervisorInitialized = true;
}

void supervisorSetSensorData(const SensorData *sensors) {
    if (sensors) {
        supervisorSensorData = *sensors;
    } else {
        memset(&supervisorSensorData, 0, sizeof(supervisorSensorData));
    }
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
    if (motorRatios) {
        memcpy(supervisorMotorRatios, motorRatios, sizeof(supervisorMotorRatios));
    }
    supervisorIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
    if (motorRPMs) {
        memcpy(supervisorMotorRPMs, motorRPMs, sizeof(supervisorMotorRPMs));
    }
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled) {
    supCrashDetectionGs = crashDetectionGs;
    supFreeFallThreshold = freeFallThreshold;
    supTiltAccZ = acceptedTiltAccZ;
    supUpsideDownAccZ = acceptedUpsideDownAccZ;
    supMaxTiltTime = maxTiltTime;
    supMaxUpsideDownTime = maxUpsideDownTime;
    supTumbleEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
    supAutoArming = autoArming;
    supSpinupDurationMs = spinupTimeoutDurationMs;
}

bool supervisorCanFly(void) {
    return (supervisorState == supervisorStateReadyToFly ||
            supervisorState == supervisorStateFlying ||
            supervisorState == supervisorStateWarningLevelOut ||
            supervisorState == supervisorStateLanded);
}

bool supervisorCanArm(void) {
    return supervisorState == supervisorStatePreFlChecksPassed;
}

bool supervisorIsArmed(void) {
    return supArmed;
}

bool supervisorIsCrashed(void) {
    return supCrashFlag;
}

bool supervisorRequestArming(bool doArm) {
    if (doArm) {
        if (!supervisorCanArm()) {
            return false;
        }
        if (supArmed && supervisorState == supervisorStateArming) {
            return true;
        }
        supArmed = true;
        supervisorState = supervisorStateArming;
        supArmingStartValid = true;
        supArmingStartTick = platformTickMs;
        return true;
    }

    supArmed = false;
    supArmingStartValid = false;
    supArmingStartTick = 0U;
    supSpinupTimeoutFlag = false;
    return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
    if (doRecovery) {
        if (supTumbledFlag) {
            return false;
        }
        supCrashFlag = false;
        return true;
    }
    supCrashFlag = true;
    supervisorState = supervisorStateCrashed;
    return true;
}

bool supervisorAreMotorsAllowedToRun(void) {
    return (supervisorState == supervisorStateArming ||
            supervisorState == supervisorStateReadyToFly ||
            supervisorState == supervisorStateFlying ||
            supervisorState == supervisorStateWarningLevelOut ||
            supervisorState == supervisorStateLanded);
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick) {
    if (!motorRatios) {
        return false;
    }
    for (int i = 0; i < 4; ++i) {
        if (motorRatios[i] > idleThrust) {
            supRecentFlyingTick = currentTick;
            supSeenFlying = true;
            break;
        }
    }
    if (!supSeenFlying) {
        return false;
    }
    if (currentTick < supRecentFlyingTick) {
        return true;
    }
    return (currentTick - supRecentFlyingTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling) {
    if (isFreeFalling) {
        *isFreeFalling = false;
    }

    if (crashDetectionGs > 0.0f) {
        float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
        if (fabsf(norm - 1.0f) > crashDetectionGs) {
            supCrashFlag = true;
        }
    }

    if (!tumbleCheckEnabled) {
        return false;
    }

    bool freeFall = (fabsf(accX) < freeFallThreshold &&
                     fabsf(accY) < freeFallThreshold &&
                     fabsf(accZ) < freeFallThreshold);
    if (freeFall) {
        if (isFreeFalling) {
            *isFreeFalling = true;
        }
        supTumbleTimerActive = false;
        supTumbleStartTick = 0U;
        return false;
    }

    bool tilted = false;
    uint32_t timeout = maxTiltTime;
    if (accZ < acceptedUpsideDownAccZ) {
        tilted = true;
        timeout = maxUpsideDownTime;
    } else if (accZ < acceptedTiltAccZ) {
        tilted = true;
        timeout = maxTiltTime;
    }

    if (tilted) {
        if (!supTumbleTimerActive) {
            supTumbleTimerActive = true;
            supTumbleStartTick = currentTick;
        } else if (currentTick >= supTumbleStartTick &&
                   (currentTick - supTumbleStartTick) >= timeout) {
            return true;
        }
    } else {
        supTumbleTimerActive = false;
        supTumbleStartTick = 0U;
    }

    return false;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick) {
    if (lastNotificationTick == 0U) {
        return true;
    }
    if (currentTick < lastNotificationTick) {
        return true;
    }
    return (currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration) {
    if (state != supervisorStateReadyToFly || latestArmingTick == 0U ||
        preflightTimeoutDuration == 0U) {
        return false;
    }
    if (currentTick < latestArmingTick) {
        return false;
    }
    return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration) {
    if (latestLandingTick == 0U || landingTimeoutDuration == 0U) {
        return false;
    }
    if (currentTick < latestLandingTick) {
        return false;
    }
    return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed) {
    uint32_t bits = 0U;

    if (supArmed) {
        bits |= SUPERVISOR_CB_ARMED;
    }
    if (supFlyingFlag) {
        bits |= SUPERVISOR_CB_IS_FLYING;
    }
    if (supTumbledFlag) {
        bits |= SUPERVISOR_CB_IS_TUMBLED;
    }

    uint32_t commanderAge = commanderGetInactivityTime();
    if (commanderAge > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
        bits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    } else if (commanderAge > COMMANDER_WDT_TIMEOUT_STABILIZE) {
        bits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    }

    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        bits |= SUPERVISOR_CB_EMERGENCY_STOP;
    }

    if (supCrashFlag) {
        bits |= SUPERVISOR_CB_CRASHED;
    }
    if (supDeckFault) {
        bits |= SUPERVISOR_CB_DECK_FAULT;
    }
    if (supRpmValid) {
        bits |= SUPERVISOR_CB_RPM_AT_ARMING_VALID;
    }
    if (supMotorsNotRespondingFlag) {
        bits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    }
    if (supSpinupTimeoutFlag) {
        bits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
    }
    if (supFreeFallingFlag) {
        bits |= SUPERVISOR_CB_FREE_FALL;
    }

    supervisorConditionBits = bits;
    return bits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBits,
                                SupervisorState state) {
    if (!setpoint) {
        return;
    }
    (void)supervisorConditionBits;

    switch (state) {
        case supervisorStateWarningLevelOut: {
            float zMode = (float)setpoint->mode.z;
            float zVel = setpoint->velocity.z;
            float zPos = setpoint->position.z;
            setpoint->mode.x = modeDisable;
            setpoint->mode.y = modeDisable;
            setpoint->mode.roll = modeAbs;
            setpoint->mode.pitch = modeAbs;
            setpoint->attitude.roll = 0.0f;
            setpoint->attitude.pitch = 0.0f;
            setpoint->mode.yaw = modeVelocity;
            setpoint->attitudeRate.yaw = 0.0f;
            setpoint->mode.z = (StabilizationMode)zMode;
            setpoint->velocity.z = zVel;
            setpoint->position.z = zPos;
            break;
        }
        case supervisorStateArming:
        case supervisorStateReadyToFly:
        case supervisorStateFlying:
        case supervisorStateLanded:
            break;
        default:
            *setpoint = (Setpoint){0};
            break;
    }
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax) {
    if (!motorRPMs) {
        return false;
    }
    for (int i = 0; i < 4; ++i) {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) {
            return false;
        }
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTick) {
    if (!motorRPMs || !canFly) {
        supNotRespondingTimerActive = false;
        supNotRespondingStartTick = 0U;
        return false;
    }

    bool below = false;
    for (int i = 0; i < 4; ++i) {
        if (motorRPMs[i] < rpmThreshold) {
            below = true;
            break;
        }
    }

    if (below) {
        if (!supNotRespondingTimerActive) {
            supNotRespondingTimerActive = true;
            supNotRespondingStartTick = currentTick;
        }
        if (currentTick >= supNotRespondingStartTick &&
            (currentTick - supNotRespondingStartTick) >= rpmCheckDurationMs) {
            return true;
        }
    } else {
        supNotRespondingTimerActive = false;
        supNotRespondingStartTick = 0U;
    }
    return false;
}

uint16_t supervisorGetInfoBitfield(void) {
    uint16_t bits = 0U;
    if (supervisorCanArm()) bits |= (1U << 0);
    if (supervisorIsArmed()) bits |= (1U << 1);
    if (supAutoArming) bits |= (1U << 2);
    if (supervisorCanFly()) bits |= (1U << 3);
    if (supFlyingFlag) bits |= (1U << 4);
    if (supTumbledFlag) bits |= (1U << 5);
    if (supervisorState == supervisorStateLocked) bits |= (1U << 6);
    if (supervisorIsCrashed()) bits |= (1U << 7);
    if (supervisorTrajectoryFlying) bits |= (1U << 8);
    if (supervisorTrajectoryFinished) bits |= (1U << 9);
    if (supervisorTrajectoryDisabled) bits |= (1U << 10);
    if (supDeckFault) bits |= (1U << 11);
    return bits;
}

void supervisorUpdate(uint32_t stabilizerStep) {
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) {
        return;
    }
    if (!supervisorInitialized) {
        supervisorInit();
    }

    if (supervisorPrevStateValid && supAutoArming &&
        supervisorPrevState != supervisorStatePreFlChecksPassed &&
        supervisorState == supervisorStatePreFlChecksPassed) {
        supervisorRequestArming(true);
    }

    static const bool armingHoldingState[11] = {
        false, false, false, true, true, true, true, true, false, false, false
    };
    if (supervisorPrevStateValid &&
        supervisorPrevState >= 0 && supervisorPrevState < 11 &&
        armingHoldingState[supervisorPrevState] &&
        supervisorState >= 0 && supervisorState < 11 &&
        !armingHoldingState[supervisorState]) {
        supArmed = false;
        supArmingStartValid = false;
        supArmingStartTick = 0U;
    }

    bool freeFallOut = false;
    supTumbledFlag = isTumbledCheck(supervisorSensorData.acc.x,
                                    supervisorSensorData.acc.y,
                                    supervisorSensorData.acc.z,
                                    supCrashDetectionGs,
                                    supFreeFallThreshold,
                                    supTiltAccZ,
                                    supUpsideDownAccZ,
                                    supMaxTiltTime,
                                    supMaxUpsideDownTime,
                                    supTumbleEnabled,
                                    platformTickMs,
                                    &freeFallOut);
    supFreeFallingFlag = freeFallOut;
    supFlyingFlag = isFlyingCheck(supervisorMotorRatios, supervisorIdleThrust,
                                  platformTickMs);

    static int32_t rpmMin = 0;
    static int32_t rpmMax = INT32_MAX;
    supRpmValid = isRPMatArmingValid(supervisorMotorRPMs, rpmMin, rpmMax);

    static int32_t rpmThreshold = 10;
    static uint32_t rpmDuration = 100U;
    supMotorsNotRespondingFlag = isMotorsNotResponding(supervisorMotorRPMs,
                                                       rpmThreshold,
                                                       rpmDuration,
                                                       supervisorCanFly(),
                                                       platformTickMs);

    if (supervisorState == supervisorStateArming && supArmingStartValid) {
        if (supSpinupDurationMs > 0U && platformTickMs >= supArmingStartTick &&
            (platformTickMs - supArmingStartTick) >= supSpinupDurationMs) {
            supSpinupTimeoutFlag = true;
        } else {
            supSpinupTimeoutFlag = false;
        }
    } else {
        if (supervisorState != supervisorStateArming) {
            supArmingStartValid = false;
            supArmingStartTick = 0U;
        }
        supSpinupTimeoutFlag = false;
    }

    if (supFreeFallingFlag && supervisorState != supervisorStateCrashed) {
        supervisorState = supervisorStateExceptFreeFall;
    }

    bool watchdogFailed = !checkEmergencyStopWatchdog(
        platformTickMs, supervisorLastEmergencyStopNotificationTick);
    updateAndPopulateConditions(false, false, watchdogFailed);

    supervisorLog.info = supervisorGetInfoBitfield();
    float accNorm = sqrtf(supervisorSensorData.acc.x * supervisorSensorData.acc.x +
                        supervisorSensorData.acc.y * supervisorSensorData.acc.y +
                        supervisorSensorData.acc.z * supervisorSensorData.acc.z);
    supervisorLog.accNorm = accNorm;

    supervisorPrevState = supervisorState;
    supervisorPrevStateValid = true;
}

/* ------------------------------------------------------------------------- */
/* Estimator and commander                                                     */
/* ------------------------------------------------------------------------- */

#define ESTIMATOR_FIFO_SIZE 16
static EstimatorMeasurement estimatorFifo[ESTIMATOR_FIFO_SIZE];
static int estimatorFifoHead = 0;
static int estimatorFifoTail = 0;
static int estimatorFifoCount = 0;
static EstimatorMeasurement lastEstimatorMeasurement[4];
static bool lastEstimatorSeen[4] = {false, false, false, false};
static Axis3f estimatorPosition = {0.0f, 0.0f, 0.0f};
static Axis3f estimatorVelocity = {0.0f, 0.0f, 0.0f};

bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
    if (!measurement || estimatorFifoCount >= ESTIMATOR_FIFO_SIZE) {
        return false;
    }
    estimatorFifo[estimatorFifoTail] = *measurement;
    estimatorFifoTail = (estimatorFifoTail + 1) % ESTIMATOR_FIFO_SIZE;
    estimatorFifoCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
    if (!measurement || estimatorFifoCount == 0) {
        return false;
    }
    *measurement = estimatorFifo[estimatorFifoHead];
    estimatorFifoHead = (estimatorFifoHead + 1) % ESTIMATOR_FIFO_SIZE;
    estimatorFifoCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        if (m.type >= MeasurementTypeGyroscope && m.type <= MeasurementTypeTOF) {
            lastEstimatorMeasurement[m.type] = m;
            lastEstimatorSeen[m.type] = true;
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        float gx = lastEstimatorSeen[MeasurementTypeGyroscope]
                       ? lastEstimatorMeasurement[MeasurementTypeGyroscope].data[0]
                       : 0.0f;
        float gy = lastEstimatorSeen[MeasurementTypeGyroscope]
                       ? lastEstimatorMeasurement[MeasurementTypeGyroscope].data[1]
                       : 0.0f;
        float gz = lastEstimatorSeen[MeasurementTypeGyroscope]
                       ? lastEstimatorMeasurement[MeasurementTypeGyroscope].data[2]
                       : 0.0f;
        float ax = lastEstimatorSeen[MeasurementTypeAcceleration]
                       ? lastEstimatorMeasurement[MeasurementTypeAcceleration].data[0]
                       : 0.0f;
        float ay = lastEstimatorSeen[MeasurementTypeAcceleration]
                       ? lastEstimatorMeasurement[MeasurementTypeAcceleration].data[1]
                       : 0.0f;
        float az = lastEstimatorSeen[MeasurementTypeAcceleration]
                       ? lastEstimatorMeasurement[MeasurementTypeAcceleration].data[2]
                       : 0.0f;

        sensfusion6UpdateQ(gx, gy, gz, ax, ay, az, 0.004f);

        float roll, pitch, yaw;
        sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
        stateEstimate.roll = roll;
        stateEstimate.pitch = pitch;
        stateEstimate.yaw = yaw;
        sensfusion6GetQuaternion(&stateEstimate.qw, &stateEstimate.qx,
                                 &stateEstimate.qy, &stateEstimate.qz);

        gyro.x = gx;
        gyro.y = gy;
        gyro.z = gz;
        acc.x = ax;
        acc.y = ay;
        acc.z = az;

        float azNoGravity = sensfusion6GetAccZWithoutGravity(ax, ay, az);
        estimatorVelocity.z += azNoGravity * 0.004f;
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        estimatorPosition.x += estimatorVelocity.x * 0.01f;
        estimatorPosition.y += estimatorVelocity.y * 0.01f;
        estimatorPosition.z += estimatorVelocity.z * 0.01f;
    }

    if (lastEstimatorSeen[MeasurementTypeBarometer]) {
        EstimatorMeasurement *b = &lastEstimatorMeasurement[MeasurementTypeBarometer];
        baro.asl = b->data[0];
        baro.temp = b->data[1];
        baro.pressure = b->data[2];
    }
}

static Setpoint commanderActiveSetpoint;
static int commanderActivePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t commanderLastUpdateTick = 0U;

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
    if (!setpoint) {
        return false;
    }

    if (priority == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
        commanderActiveSetpoint = *setpoint;
        commanderActivePriority = priority;
        commanderLastUpdateTick = platformTickMs;
        return true;
    }

    if (priority >= commanderActivePriority) {
        commanderActiveSetpoint = *setpoint;
        commanderActivePriority = priority;
        commanderLastUpdateTick = platformTickMs;
        if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
            supervisorTrajectoryFlying = false;
        }
        return true;
    }
    return false;
}

void commanderRelaxPriority(void) {
    commanderActivePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
    if (platformTickMs <= commanderLastUpdateTick) {
        return 0U;
    }
    return platformTickMs - commanderLastUpdateTick;
}

int commanderGetActivePriority(void) {
    return commanderActivePriority;
}

/* ------------------------------------------------------------------------- */
/* Stabilizer                                                                 */
/* ------------------------------------------------------------------------- */

static bool stabilizerInitialized = false;
static uint32_t stabilizerStep = 0U;
static Setpoint highLevelSetpoint;
static bool highLevelSetpointPending = false;
static SensorData stabilizerSensorData;
static State stabilizerState;
static ControlData stabilizerControl;
static MotorPower stabilizerMotorPower;
static uint32_t stabilizerMotorPWMs[4] = {0U, 0U, 0U, 0U};

static void sensorsInit(void) {
    memset(&stabilizerSensorData, 0, sizeof(stabilizerSensorData));
}

static void stateEstimatorInit(void) {
    memset(&stabilizerState, 0, sizeof(stabilizerState));
}

static void controllerInit(void) {
    attitudeControllerInit(0.002f);
}

static void powerDistributionInit(void) {
    memset(&stabilizerMotorPower, 0, sizeof(stabilizerMotorPower));
}

static void motorsInit(void) {
    memset(stabilizerMotorPWMs, 0, sizeof(stabilizerMotorPWMs));
}

static void collisionAvoidanceInit(void) {
    /* no persistent state in host model */
}

void stabilizerInit(void) {
    if (stabilizerInitialized) {
        return;
    }
    sensorsInit();
    stateEstimatorInit();
    controllerInit();
    powerDistributionInit();
    motorsInit();
    collisionAvoidanceInit();
    stabilizerInitialized = true;
}

static void sensorsWaitDataReady(void) {
    /* In host model, data is already available. */
}

static void sensorsAcquire(void) {
    /* Sensor injection for host tests normally occurs through the estimator
       FIFO or by setting supervisor sensor data. The stabilizer uses the local
       model sensor data; it is zero-filled unless a host test overrides it. */
}

static void stateEstimator(void) {
    estimatorComplementary(stabilizerStep);
    float roll, pitch, yaw;
    sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
    stabilizerState.attitude.roll = roll;
    stabilizerState.attitude.pitch = pitch;
    stabilizerState.attitude.yaw = yaw;
    sensfusion6GetQuaternion(&stabilizerState.attitudeQuaternion.w,
                             &stabilizerState.attitudeQuaternion.x,
                             &stabilizerState.attitudeQuaternion.y,
                             &stabilizerState.attitudeQuaternion.z);
    stabilizerState.acc = stabilizerSensorData.acc;
}

static void commanderGetSetpoint(Setpoint *out) {
    if (out) {
        *out = commanderActiveSetpoint;
    }
}

static void collisionAvoidanceUpdateSetpoint(Setpoint *setpoint) {
    (void)setpoint;
}

static void applyBatteryCompensation(MotorPower *motorPower) {
    if (!motorPower) {
        return;
    }
    /* Use the fixed nominal/actual 4.2V model for the host baseline. Values are
       unchanged, but the conversion boundary is explicit. */
    float nominal = 4.2f;
    float actual = 4.2f;
    motorPower->m1 = motorsCompensateBatteryVoltage(
        (uint16_t)clamp32(motorPower->m1, 0, 65535), nominal, actual);
    motorPower->m2 = motorsCompensateBatteryVoltage(
        (uint16_t)clamp32(motorPower->m2, 0, 65535), nominal, actual);
    motorPower->m3 = motorsCompensateBatteryVoltage(
        (uint16_t)clamp32(motorPower->m3, 0, 65535), nominal, actual);
    motorPower->m4 = motorsCompensateBatteryVoltage(
        (uint16_t)clamp32(motorPower->m4, 0, 65535), nominal, actual);
}

static void setMotorRatios(void) {
    PowerCapResult cap = powerDistributionCap((int32_t *)&stabilizerMotorPower,
                                              65535, 0);
    (void)cap;
    motor.m1req = (uint16_t)clamp32(stabilizerMotorPower.m1, 0, 65535);
    motor.m2req = (uint16_t)clamp32(stabilizerMotorPower.m2, 0, 65535);
    motor.m3req = (uint16_t)clamp32(stabilizerMotorPower.m3, 0, 65535);
    motor.m4req = (uint16_t)clamp32(stabilizerMotorPower.m4, 0, 65535);
    for (int i = 0; i < 4; ++i) {
        int32_t val = (i == 0) ? stabilizerMotorPower.m1 :
                      (i == 1) ? stabilizerMotorPower.m2 :
                      (i == 2) ? stabilizerMotorPower.m3 :
                      stabilizerMotorPower.m4;
        stabilizerMotorPWMs[i] = (uint32_t)clamp32(val, 0, 65535);
    }
    supervisorSetMotorRatios(stabilizerMotorPWMs, 0U);
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
    if (!setpoint) {
        return false;
    }
    highLevelSetpoint = *setpoint;
    highLevelSetpointPending = true;
    return true;
}

void stabilizerTask(void) {
    if (!stabilizerInitialized) {
        return;
    }

    if (highLevelSetpointPending) {
        commanderSetSetpoint(&highLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        highLevelSetpointPending = false;
    }

    if (healthShallWeRunTest()) {
        healthRunTests(&stabilizerSensorData);
        return;
    }

    sensorsWaitDataReady();
    sensorsAcquire();
    stateEstimator();

    Setpoint setpoint;
    commanderGetSetpoint(&setpoint);

    supervisorUpdate(stabilizerStep);
    collisionAvoidanceUpdateSetpoint(&setpoint);
    supervisorOverrideSetpoint(&setpoint, supervisorConditionBits, supervisorState);

    if (!supervisorCanFly()) {
        setpoint = (Setpoint){0};
    }

    controllerPid(&stabilizerSensorData, &setpoint, &stabilizerState,
                  &stabilizerControl, 0.0f, 0.002f);
    powerDistribution(&stabilizerControl, &stabilizerMotorPower);
    applyBatteryCompensation(&stabilizerMotorPower);

    if (!supervisorAreMotorsAllowedToRun()) {
        stabilizerMotorPower.m1 = 0;
        stabilizerMotorPower.m2 = 0;
        stabilizerMotorPower.m3 = 0;
        stabilizerMotorPower.m4 = 0;
    }

    setMotorRatios();
    stabilizerStep++;
}

/* ------------------------------------------------------------------------- */
/* State compression                                                          */
/* ------------------------------------------------------------------------- */

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output) {
    if (!state || !sensors || !output) {
        return;
    }

    for (int i = 0; i < 3; ++i) {
        output->position_mm[i] = (int32_t)(state->position.x * 1000.0f);
        output->velocity_mms[i] = (int32_t)(state->velocity.x * 1000.0f);
    }

    output->acceleration_mms2[0] = (int32_t)(sensors->acc.x * 9810.0f);
    output->acceleration_mms2[1] = (int32_t)(sensors->acc.y * 9810.0f);
    output->acceleration_mms2[2] = (int32_t)((sensors->acc.z + 1.0f) * 9810.0f);

    output->gyro_millirad_s[0] = sensors->gyro.x * DEG_TO_RAD * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * DEG_TO_RAD * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * DEG_TO_RAD * 1000.0f;

    /* Deterministic 32-bit quaternion compression. */
    uint32_t packed = 0U;
    packed |= ((uint32_t)((state->attitudeQuaternion.w + 1.0f) * 0.5f * 32767.0f) & 0x7FFFU) << 16;
    packed |= ((uint32_t)((state->attitudeQuaternion.x + 1.0f) * 0.5f * 32767.0f) & 0x7FFFU) << 11;
    packed |= ((uint32_t)((state->attitudeQuaternion.y + 1.0f) * 0.5f * 32767.0f) & 0x7FFFU) << 6;
    packed |= ((uint32_t)((state->attitudeQuaternion.z + 1.0f) * 0.5f * 32767.0f) & 0x7FFFU);
    output->quatCompressed = packed;
}

bool rateSupervisorValidate(uint32_t measuredRate) {
    return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void) {
    /* The host version observes the 2000 ms watchdog via the deterministic
       tick, but does not itself enter a blocking RTOS assert loop. */
}

/* ------------------------------------------------------------------------- */
/* Health state machine                                                       */
/* ------------------------------------------------------------------------- */

TestState healthTestState = testDone;
uint8_t motorPass = 0U;
uint8_t batteryPass = 0U;
float batterySag = 0.0f;

static bool healthPropRequest = false;
static bool healthBatRequest = false;
static uint32_t healthPropSampleCount = 0U;
static uint32_t healthPropMotorIndex = 0U;
static float healthIdleVoltage = 4.2f;
static float healthMinLoadedVoltage = 4.2f;
static uint32_t healthBatTick = 0U;
static uint32_t healthBatRestartStartTick = 0U;
static float healthNoiseBuffer[PROPTEST_NBR_OF_VARIANCE_VALUES] = {0.0f};
static uint32_t healthMotorTestCounter = 0U;

void healthRequestPropTest(void) {
    healthPropRequest = true;
}

void healthRequestBatteryTest(void) {
    healthBatRequest = true;
}

bool healthShallWeRunTest(void) {
    if (healthPropRequest) {
        healthPropRequest = false;
        healthTestState = configureAcc;
        motorPass = 0U;
        batteryPass = 0U;
        batterySag = 0.0f;
        healthPropSampleCount = 0U;
        healthPropMotorIndex = 0U;
        healthIdleVoltage = 4.2f;
        healthMotorTestCounter = 0U;
        return true;
    }

    if (healthBatRequest) {
        healthBatRequest = false;
        healthTestState = testBattery;
        healthBatTick = 0U;
        healthIdleVoltage = 4.2f;
        healthMinLoadedVoltage = 4.2f;
        healthBatRestartStartTick = 0U;
        return true;
    }

    return healthTestState != testDone;
}

void healthRunTests(const SensorData *sensorData) {
    if (!sensorData) {
        return;
    }

    switch (healthTestState) {
        case configureAcc:
            motorPass = 0U;
            batteryPass = 0U;
            batterySag = 0.0f;
            healthPropSampleCount = 0U;
            healthPropMotorIndex = 0U;
            healthIdleVoltage = 4.2f;
            healthMotorTestCounter = 0U;
            healthTestState = measureNoiseFloor;
            break;

        case measureNoiseFloor:
            if (healthPropSampleCount < PROPTEST_NBR_OF_VARIANCE_VALUES) {
                float value = sqrtf(sensorData->acc.x * sensorData->acc.x +
                                    sensorData->acc.y * sensorData->acc.y +
                                    sensorData->acc.z * sensorData->acc.z) - 1.0f;
                healthNoiseBuffer[healthPropSampleCount++] = value;
            } else {
                healthPropSampleCount = 0U;
                healthPropMotorIndex = 0U;
                healthTestState = measureProp;
            }
            break;

        case measureProp:
            if (healthPropMotorIndex < 4U) {
                float value = sqrtf(sensorData->acc.x * sensorData->acc.x +
                                    sensorData->acc.y * sensorData->acc.y +
                                    sensorData->acc.z * sensorData->acc.z);
                evaluatePropTest(0.0f, 0.0f, value, (uint8_t)healthPropMotorIndex);
                healthPropMotorIndex++;
            } else {
                healthTestState = evaluatePropResult;
            }
            break;

        case evaluatePropResult:
            healthTestState = testDone;
            break;

        case testBattery:
            if (healthBatTick == 0U) {
                healthBatTick = 1U;
                healthMinLoadedVoltage = 4.2f;
            } else if (healthBatTick < 50U) {
                if (healthBatTick >= 2U) {
                    float voltage = 4.0f;
                    if (healthMinLoadedVoltage > voltage) {
                        healthMinLoadedVoltage = voltage;
                    }
                }
                healthBatTick++;
                if (healthBatTick >= 50U) {
                    healthTestState = evaluateBatResult;
                }
            }
            break;

        case evaluateBatResult:
            batterySag = healthIdleVoltage - healthMinLoadedVoltage;
            batteryPass = batterySag <= 0.5f ? 1U : 0U;
            healthTestState = testDone;
            break;

        case restartBatTest:
            if (healthBatRestartStartTick == 0U) {
                healthBatRestartStartTick = platformTickMs;
            }
            if (platformTickMs >= healthBatRestartStartTick &&
                (platformTickMs - healthBatRestartStartTick) >= 2000U) {
                healthBatRestartStartTick = 0U;
                healthBatTick = 0U;
                healthTestState = testBattery;
            }
            break;

        case testDone:
        default:
            break;
    }

    healthLog.motorPass = motorPass;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
    healthLog.motorTestCount = healthMotorTestCounter;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIdx) {
    if (highThreshold == 0.0f) {
        return true;
    }
    if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
        if (motorIdx < 8U) {
            motorPass |= (uint8_t)(1U << motorIdx);
        }
        return true;
    }
    healthMotorTestCounter++;
    return false;
}

float variance(const float *buffer, int length) {
    if (!buffer || length <= 1) {
        return 0.0f;
    }
    float sum = 0.0f;
    float sumSq = 0.0f;
    for (int i = 0; i < length; ++i) {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }
    return sumSq - (sum * sum / (float)length);
}

/* ------------------------------------------------------------------------- */
/* CRTP transport                                                             */
/* ------------------------------------------------------------------------- */

static CrtpPacket crtpTxQueue[CRTP_TX_QUEUE_SIZE];
static int crtpTxHead = 0;
static int crtpTxTail = 0;
static int crtpTxCount = 0;

static bool crtpRxCreated[CRTP_NBR_OF_PORTS] = {false};
static CrtpPacket crtpRxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static int crtpRxHead[CRTP_NBR_OF_PORTS] = {0};
static int crtpRxTail[CRTP_NBR_OF_PORTS] = {0};
static int crtpRxCount[CRTP_NBR_OF_PORTS] = {0};
static CrtpPortCallback crtpCallbacks[CRTP_NBR_OF_PORTS] = {NULL};

static bool crtpInitialized = false;
static bool crtpError = false;
static uint32_t crtpLastStatsTick = 0U;
static uint32_t crtpRxCountTotal = 0U;
static uint32_t crtpTxCountTotal = 0U;
static uint32_t crtpRxRate = 0U;
static uint32_t crtpTxRate = 0U;

static bool nopLinkSend(CrtpPacket *packet) {
    (void)packet;
    return false;
}

static bool nopLinkReceive(CrtpPacket *packet) {
    (void)packet;
    return false;
}

static bool nopLinkIsConnected(void) {
    return true;
}

static void nopLinkSetEnable(bool enable) {
    (void)enable;
}

static void nopLinkReset(void) {
}

static CrtpLink nopLink = {
    nopLinkSend,
    nopLinkReceive,
    nopLinkIsConnected,
    nopLinkSetEnable,
    nopLinkReset
};

static CrtpLink *activeCrtpLink = &nopLink;

void crtpInit(void) {
    if (crtpInitialized) {
        return;
    }
    crtpTxHead = 0;
    crtpTxTail = 0;
    crtpTxCount = 0;
    for (int i = 0; i < CRTP_NBR_OF_PORTS; ++i) {
        crtpRxCreated[i] = false;
        crtpRxHead[i] = 0;
        crtpRxTail[i] = 0;
        crtpRxCount[i] = 0;
        crtpCallbacks[i] = NULL;
    }
    activeCrtpLink = &nopLink;
    crtpLastStatsTick = platformTickMs;
    crtpRxCountTotal = 0U;
    crtpTxCountTotal = 0U;
    crtpRxRate = 0U;
    crtpTxRate = 0U;
    crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port) {
    if (port >= CRTP_NBR_OF_PORTS) {
        crtpError = true;
        return;
    }
    if (crtpRxCreated[port]) {
        crtpError = true;
        return;
    }
    crtpRxCreated[port] = true;
    crtpRxHead[port] = 0;
    crtpRxTail[port] = 0;
    crtpRxCount[port] = 0;
}

bool crtpSendPacket(const CrtpPacket *packet) {
    if (!packet || crtpTxCount >= CRTP_TX_QUEUE_SIZE) {
        return false;
    }
    crtpTxQueue[crtpTxTail] = *packet;
    crtpTxTail = (crtpTxTail + 1) % CRTP_TX_QUEUE_SIZE;
    crtpTxCount++;
    crtpTxCountTotal++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
    if (!packet || port >= CRTP_NBR_OF_PORTS || !crtpRxCreated[port] ||
        crtpRxCount[port] == 0) {
        return false;
    }
    *packet = crtpRxQueues[port][crtpRxHead[port]];
    crtpRxHead[port] = (crtpRxHead[port] + 1) % CRTP_RX_QUEUE_SIZE;
    crtpRxCount[port]--;
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
    if (activeCrtpLink == NULL || activeCrtpLink->receivePacket == NULL) {
        return;
    }
    CrtpPacket packet;
    if (activeCrtpLink->receivePacket(&packet)) {
        crtpRxCountTotal++;
        uint8_t port = packet.port;
        if (port < CRTP_NBR_OF_PORTS && crtpRxCreated[port]) {
            if (crtpRxCount[port] < CRTP_RX_QUEUE_SIZE) {
                crtpRxQueues[port][crtpRxTail[port]] = packet;
                crtpRxTail[port] = (crtpRxTail[port] + 1) % CRTP_RX_QUEUE_SIZE;
                crtpRxCount[port]++;
            }
        }
        if (port < CRTP_NBR_OF_PORTS && crtpCallbacks[port]) {
            crtpCallbacks[port](&packet);
        }
    }
}

void crtpTxTask(void) {
    if (activeCrtpLink == &nopLink || activeCrtpLink == NULL ||
        activeCrtpLink->sendPacket == NULL || crtpTxCount == 0) {
        return;
    }
    CrtpPacket *front = &crtpTxQueue[crtpTxHead];
    if (activeCrtpLink->sendPacket(front)) {
        crtpTxHead = (crtpTxHead + 1) % CRTP_TX_QUEUE_SIZE;
        crtpTxCount--;
    }
}

void crtpSetLink(CrtpLink *newLink) {
    if (activeCrtpLink && activeCrtpLink != newLink && activeCrtpLink->setEnable) {
        activeCrtpLink->setEnable(false);
    }
    activeCrtpLink = (newLink == NULL) ? &nopLink : newLink;
    if (activeCrtpLink && activeCrtpLink->setEnable) {
        activeCrtpLink->setEnable(true);
    }
}

void crtpReset(void) {
    crtpTxHead = 0;
    crtpTxTail = 0;
    crtpTxCount = 0;
    if (activeCrtpLink && activeCrtpLink->reset) {
        activeCrtpLink->reset();
    }
}

bool crtpIsConnected(void) {
    if (activeCrtpLink && activeCrtpLink->isConnected) {
        return activeCrtpLink->isConnected();
    }
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
    return (uint32_t)(CRTP_TX_QUEUE_SIZE - crtpTxCount);
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
    if (port >= CRTP_NBR_OF_PORTS) {
        return;
    }
    crtpCallbacks[port] = callback;
}

void updateStats(void) {
    if (crtpLastStatsTick == 0U) {
        crtpLastStatsTick = platformTickMs;
        return;
    }
    if (platformTickMs >= crtpLastStatsTick &&
        (platformTickMs - crtpLastStatsTick) >= 500U) {
        crtpRxRate = crtpRxCountTotal;
        crtpTxRate = crtpTxCountTotal;
        crtpRxCountTotal = 0U;
        crtpTxCountTotal = 0U;
        crtpLastStatsTick = platformTickMs;
    }
}

/* ------------------------------------------------------------------------- */
/* Deck discovery and log objects                                             */
/* ------------------------------------------------------------------------- */

static const uint8_t knownI2cAddresses[] = {0x60U, 0x76U};
static const uint64_t knownOneWireRomIds[] = {
    0x1000000000000012ULL,
    0x2000000000000034ULL
};

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
    if (!decks || capacity == 0U) {
        return 0U;
    }

    uint8_t written = 0U;
    for (uint8_t i = 0; i < (uint8_t)(sizeof(knownI2cAddresses) / sizeof(knownI2cAddresses[0])); ++i) {
        if (written >= capacity) {
            break;
        }
        decks[written].foundByI2C = true;
        decks[written].foundByOneWire = false;
        decks[written].i2cAddress = knownI2cAddresses[i];
        decks[written].oneWireRomId = 0U;
        written++;
    }

    for (uint8_t i = 0; i < (uint8_t)(sizeof(knownOneWireRomIds) / sizeof(knownOneWireRomIds[0])); ++i) {
        if (written >= capacity) {
            break;
        }
        decks[written].foundByI2C = false;
        decks[written].foundByOneWire = true;
        decks[written].i2cAddress = 0U;
        decks[written].oneWireRomId = knownOneWireRomIds[i];
        written++;
    }

    return written;
}

StateEstimateLog stateEstimate;
Axis3Log gyroLog;
Axis3Log accLog;
BaroLog baroLog;
MotorLog motorLog;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

/* Alias the public Axis3Log objects to the names gyro/acc required by RE_api. */
Axis3Log gyro;
Axis3Log acc;
BaroLog baro;
MotorLog motor;

/* The public externs are used above; these aliases are direct definitions. */
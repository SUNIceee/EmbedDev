#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f
#define DEG_TO_RAD (PI_F / 180.0f)
#define RAD_TO_DEG (180.0f / PI_F)

static const float NOMINAL_BATTERY_VOLTAGE = 3.7f;
static float g_currentTick = 0U;
static uint32_t g_stabilizerStep = 0U;

int16_t saturateSignedInt16(int32_t value)
{
    if (value >= 32767) {
        return 32767;
    }
    if (value <= -32767) {
        return -32767;
    }
    return (int16_t)value;
}

float capAngle(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

static void syncSensfusionLog(void)
{
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

static void sensfusionComputeGravity(void)
{
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
}

void sensfusion6Init(void)
{
    if (!sensfusion6IsInit) {
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
        sensfusionComputeGravity();
        syncSensfusionLog();
    }
}

bool sensfusion6Test(void)
{
    return sensfusion6IsInit;
}

void sensfusion6GetQuaternion(float *outW, float *outX, float *outY, float *outZ)
{
    if (!outW || !outX || !outY || !outZ) {
        return;
    }
    *outW = qw;
    *outX = qx;
    *outY = qy;
    *outZ = qz;
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (!roll_deg || !pitch_deg || !yaw_deg) {
        return;
    }
    float sinPitch = 2.0f * (qw * qy - qz * qx);
    if (sinPitch > 1.0f) {
        sinPitch = 1.0f;
    }
    if (sinPitch < -1.0f) {
        sinPitch = -1.0f;
    }
    *pitch_deg = asinf(sinPitch) * RAD_TO_DEG;
    *roll_deg = atan2f(2.0f * (qw * qx + qy * qz),
                       1.0f - 2.0f * (qx * qx + qy * qy)) * RAD_TO_DEG;
    *yaw_deg = atan2f(2.0f * (qw * qz + qx * qy),
                      1.0f - 2.0f * (qy * qy + qz * qz)) * RAD_TO_DEG;
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
    if (dt <= 0.0f) {
        return;
    }

    float gxRad = gx * DEG_TO_RAD;
    float gyRad = gy * DEG_TO_RAD;
    float gzRad = gz * DEG_TO_RAD;
    float axOriginal = ax;
    float ayOriginal = ay;
    float azOriginal = az;

    bool accZero = (fabsf(ax) < 1e-6f && fabsf(ay) < 1e-6f && fabsf(az) < 1e-6f);

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
    if (!accZero) {
        float norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm < 1e-6f) {
            norm = 1.0f;
        }
        float recip = 1.0f / norm;
        ax *= recip;
        ay *= recip;
        az *= recip;

        float _2q0 = 2.0f * qw;
        float _2q1 = 2.0f * qx;
        float _2q2 = 2.0f * qy;
        float _2q3 = 2.0f * qz;
        float _4q0 = 4.0f * qw;
        float _4q1 = 4.0f * qx;
        float _4q2 = 4.0f * qy;
        float _8q1 = 8.0f * qx;
        float _8q2 = 8.0f * qy;
        float q0q0 = qw * qw;
        float q1q1 = qx * qx;
        float q2q2 = qy * qy;
        float q3q3 = qz * qz;

        float s0 = _4q0 * q2q2 + _2q2 * az + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * qx - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * qy + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * qz - _2q1 * ax + 4.0f * q2q2 * qz - _2q2 * ay;
        float sNorm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (sNorm < 1e-6f) {
            sNorm = 1.0f;
        }
        float recipSNorm = 1.0f / sNorm;
        float gradient0 = s0 * recipSNorm;
        float gradient1 = s1 * recipSNorm;
        float gradient2 = s2 * recipSNorm;
        float gradient3 = s3 * recipSNorm;

        float betaStep = beta * dt;
        gxRad -= betaStep * gradient0;
        gyRad -= betaStep * gradient1;
        gzRad -= betaStep * gradient2;
    }
#else
    if (!accZero) {
        float norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm < 1e-6f) {
            norm = 1.0f;
        }
        float recip = 1.0f / norm;
        ax *= recip;
        ay *= recip;
        az *= recip;

        float halfvx = qx * qz - qw * qy;
        float halfvy = qw * qx + qy * qz;
        float halfvz = qw * qw - qx * qx - qy * qy + qz * qz;
        float halfex = (ay * halfvz - az * halfvy);
        float halfey = (az * halfvx - ax * halfvz);
        float halfez = (ax * halfvy - ay * halfvx);

        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gxRad += integralFBx;
            gyRad += integralFBy;
            gzRad += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gxRad += twoKp * halfex;
        gyRad += twoKp * halfey;
        gzRad += twoKp * halfez;
    }
#endif

    float halfDt = 0.5f * dt;
    float qwNew = qw + (-gxRad * qx - gyRad * qy - gzRad * qz) * halfDt;
    float qxNew = qx + ( gxRad * qw + gzRad * qy - gyRad * qz) * halfDt;
    float qyNew = qy + ( gyRad * qw - gzRad * qx + gxRad * qz) * halfDt;
    float qzNew = qz + ( gzRad * qw + gyRad * qx - gxRad * qy) * halfDt;

    float qNorm = sqrtf(qwNew * qwNew + qxNew * qxNew +
                        qyNew * qyNew + qzNew * qzNew);
    if (qNorm < 1e-6f) {
        qNorm = 1.0f;
    }
    float qInv = 1.0f / qNorm;
    qw = qwNew * qInv;
    qx = qxNew * qInv;
    qy = qyNew * qInv;
    qz = qzNew * qInv;

    sensfusionComputeGravity();

    if (!sensfusion6IsCalibrated && !accZero) {
        baseZacc = axOriginal * gravityX + ayOriginal * gravityY + azOriginal * gravityZ;
        sensfusion6IsCalibrated = true;
    }
    syncSensfusionLog();
}

float sensfusion6GetAccZ(float ax, float ay, float az)
{
    sensfusionComputeGravity();
    return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az)
{
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

void estimatedGravityDirection(float qwIn, float qxIn, float qyIn, float qzIn,
                               float *gravX, float *gravY, float *gravZ)
{
    if (!gravX || !gravY || !gravZ) {
        return;
    }
    *gravX = 2.0f * (qxIn * qzIn - qwIn * qyIn);
    *gravY = 2.0f * (qwIn * qxIn + qyIn * qzIn);
    *gravZ = qwIn * qwIn - qxIn * qxIn - qyIn * qyIn + qzIn * qzIn;
}

float invSqrt(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }
    float x2 = x * 0.5f;
    float y = x;
    int32_t i = 0;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - x2 * y * y);
    return y;
}

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out)
{
    if (!out) {
        return;
    }
    int32_t t = (int32_t)thrust;
    int32_t r = (int32_t)roll / 2;
    int32_t p = (int32_t)pitch / 2;
    int32_t y = (int32_t)yaw;
    out->m1 = t - r + p + y;
    out->m2 = t - r - p - y;
    out->m3 = t + r - p + y;
    out->m4 = t + r + p - y;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4])
{
    if (!motorForces) {
        return;
    }
    float thrustPart = 0.25f * thrustSi;
    float rollPart = 0.0f;
    float pitchPart = 0.0f;
    float yawPart = 0.0f;

    if (fabsf(armLength) > 1e-6f) {
        float arm = 0.707106781f * armLength;
        rollPart = 0.25f / arm * torqueX;
        pitchPart = 0.25f / arm * torqueY;
    }
    if (fabsf(thrustToTorque) > 1e-6f) {
        yawPart = 0.25f / thrustToTorque * torqueZ;
    }

    motorForces[0] = thrustPart - rollPart + pitchPart - yawPart;
    motorForces[1] = thrustPart - rollPart - pitchPart + yawPart;
    motorForces[2] = thrustPart + rollPart - pitchPart - yawPart;
    motorForces[3] = thrustPart + rollPart + pitchPart + yawPart;

    for (int i = 0; i < 4; i++) {
        if (motorForces[i] < 0.0f) {
            motorForces[i] = 0.0f;
        }
    }
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4])
{
    if (!normalizedForces || !motorPWMs) {
        return;
    }
    for (int i = 0; i < 4; i++) {
        float v = normalizedForces[i];
        if (v < 0.0f) {
            v = 0.0f;
        }
        if (v > 1.0f) {
            v = 1.0f;
        }
        motorPWMs[i] = (uint16_t)(v * 65535.0f);
    }
}

static uint16_t motorForceToPwm(float force)
{
    if (force <= 0.0f) {
        return 0U;
    }
    if (force >= CRAZYFLIE_MAX_MOTOR_FORCE_N) {
        return 65535U;
    }
    return (uint16_t)((force / CRAZYFLIE_MAX_MOTOR_FORCE_N) * 65535.0f + 0.5f);
}

void powerDistribution(const ControlData *control, MotorPower *motorPower)
{
    if (!control || !motorPower) {
        return;
    }

    switch (control->controlMode) {
    case controlModeLegacy:
        powerDistributionLegacy(control->thrust, control->roll, control->pitch,
                                control->yaw, motorPower);
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

int32_t capMinThrust(int32_t value, int32_t idleThrust)
{
    return value < idleThrust ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust)
{
    PowerCapResult result;
    result.isCapped = false;
    result.reduction = 0;
    if (!motors) {
        return result;
    }

    int32_t maxMotor = motors[0];
    for (int i = 1; i < 4; i++) {
        if (motors[i] > maxMotor) {
            maxMotor = motors[i];
        }
    }

    if (maxMotor > maxAllowedThrust) {
        result.isCapped = true;
        result.reduction = maxMotor - maxAllowedThrust;
        for (int i = 0; i < 4; i++) {
            motors[i] = capMinThrust(motors[i] - result.reduction, idleThrust);
        }
    }
    return result;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha)
{
    return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust,
                                        float nominalVoltage,
                                        float actualVoltage)
{
    if (actualVoltage <= 0.0f) {
        return motorThrust;
    }
    float comp = roundf((float)motorThrust * nominalVoltage / actualVoltage);
    if (comp < 0.0f) {
        return 0U;
    }
    if (comp > 65535.0f) {
        return 65535U;
    }
    return (uint16_t)comp;
}

PidObject pidRoll = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidPitch = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidYaw = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidRollRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidPitchRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidYawRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

static PidObject g_posZPid = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
static float g_attitudeUpdateDt = 0.002f;
static float s_desiredYaw = 0.0f;
static bool s_desiredYawInitialized = false;

static void initPidObject(PidObject *pid)
{
    if (!pid) {
        return;
    }
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->kff = 0.0f;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
    pid->initialized = true;
}

static void resetPidObject(PidObject *pid)
{
    if (!pid) {
        return;
    }
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
}

static float updatePidObject(PidObject *pid, float actual, float desired, bool reset)
{
    if (!pid || !pid->initialized) {
        return 0.0f;
    }
    if (reset) {
        resetPidObject(pid);
    }
    float error = desired - actual;
    float pTerm = pid->kp * error;
    pid->integral += pid->ki * error;
    float dTerm = pid->kd * (error - pid->prevError);
    float output = pTerm + pid->integral + dTerm + pid->kff * desired;
    pid->prevError = error;
    pid->output = output;
    return output;
}

void attitudeControllerInit(float updateDt)
{
    if (pidRoll.initialized && pidPitch.initialized && pidYaw.initialized &&
        pidRollRate.initialized && pidPitchRate.initialized && pidYawRate.initialized) {
        return;
    }
    g_attitudeUpdateDt = (updateDt > 0.0f) ? updateDt : 0.002f;
    initPidObject(&pidRoll);
    initPidObject(&pidPitch);
    initPidObject(&pidYaw);
    initPidObject(&pidRollRate);
    initPidObject(&pidPitchRate);
    initPidObject(&pidYawRate);
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired)
{
    float rollOut = updatePidObject(&pidRollRate, rollActual, rollDesired, false);
    float pitchOut = updatePidObject(&pidPitchRate, pitchActual, pitchDesired, false);
    float yawOut = updatePidObject(&pidYawRate, yawActual, yawDesired, false);
    pidRollRate.output = (float)saturateSignedInt16((int32_t)rollOut);
    pidPitchRate.output = (float)saturateSignedInt16((int32_t)pitchOut);
    pidYawRate.output = (float)saturateSignedInt16((int32_t)yawOut);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired)
{
    float rollOut = updatePidObject(&pidRoll, rollActual, rollDesired, false);
    float pitchOut = updatePidObject(&pidPitch, pitchActual, pitchDesired, false);
    float yawOut = updatePidObject(&pidYaw, yawActual, yawDesired, true);
    pidRoll.output = (float)saturateSignedInt16((int32_t)rollOut);
    pidPitch.output = (float)saturateSignedInt16((int32_t)pitchOut);
    pidYaw.output = (float)saturateSignedInt16((int32_t)yawOut);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual, float yawActual)
{
    (void)rollActual;
    (void)pitchActual;
    (void)yawActual;
    resetPidObject(&pidRoll);
    resetPidObject(&pidPitch);
    resetPidObject(&pidYaw);
    resetPidObject(&pidRollRate);
    resetPidObject(&pidPitchRate);
    resetPidObject(&pidYawRate);
}

void attitudeControllerResetRollAttitudePID(float rollActual)
{
    (void)rollActual;
    resetPidObject(&pidRoll);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual)
{
    (void)pitchActual;
    resetPidObject(&pidPitch);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch, int16_t *yaw)
{
    if (!roll || !pitch || !yaw) {
        return;
    }
    *roll = (int16_t)pidRollRate.output;
    *pitch = (int16_t)pidPitchRate.output;
    *yaw = (int16_t)pidYawRate.output;
}

static float quaternionToEulerYaw(const Quaternion *q)
{
    if (!q) {
        return 0.0f;
    }
    return atan2f(2.0f * (q->w * q->z + q->x * q->y),
                  1.0f - 2.0f * (q->y * q->y + q->z * q->z)) * RAD_TO_DEG;
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state)
{
    if (!setpoint || !state) {
        return 0U;
    }
    float error = 0.0f;
    if (setpoint->mode.z == modeVelocity) {
        error = setpoint->velocity.z - state->velocity.z;
    } else {
        error = setpoint->position.z - state->position.z;
    }
    float thrust = 30000.0f + 20000.0f * error;
    if (thrust < 0.0f) {
        thrust = 0.0f;
    }
    if (thrust > 65535.0f) {
        thrust = 65535.0f;
    }
    return (uint16_t)thrust;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt)
{
    if (!sensors || !setpoint || !state || !control) {
        return;
    }

    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    control->thrust = 0U;
    control->thrustSi = 0.0f;
    control->torque.x = 0.0f;
    control->torque.y = 0.0f;
    control->torque.z = 0.0f;
    control->normalizedForces[0] = 0.0f;
    control->normalizedForces[1] = 0.0f;
    control->normalizedForces[2] = 0.0f;
    control->normalizedForces[3] = 0.0f;
    control->controlMode = controlModeLegacy;

    uint16_t thrust = 0U;
    if (setpoint->mode.z != modeDisable) {
        thrust = positionControllerUpdate(setpoint, state);
    } else {
        thrust = setpoint->thrust;
    }

    if (thrust == 0U) {
        resetPidObject(&pidRoll);
        resetPidObject(&pidPitch);
        resetPidObject(&pidYaw);
        resetPidObject(&pidRollRate);
        resetPidObject(&pidPitchRate);
        resetPidObject(&pidYawRate);
        resetPidObject(&g_posZPid);
        s_desiredYaw = state->attitude.yaw;
        s_desiredYawInitialized = true;
        return;
    }

    if (!s_desiredYawInitialized) {
        s_desiredYaw = state->attitude.yaw;
        s_desiredYawInitialized = true;
    }

    if (setpoint->mode.yaw == modeVelocity) {
        s_desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
        if (fabsf(yawMaxDelta) > 1e-6f) {
            float low = state->attitude.yaw - yawMaxDelta;
            float high = state->attitude.yaw + yawMaxDelta;
            if (s_desiredYaw < low) {
                s_desiredYaw = low;
            }
            if (s_desiredYaw > high) {
                s_desiredYaw = high;
            }
        }
    } else if (setpoint->mode.yaw == modeAbs) {
        s_desiredYaw = setpoint->attitude.yaw;
    } else if (setpoint->mode.quat == modeAbs) {
        s_desiredYaw = quaternionToEulerYaw(&setpoint->attitudeQuaternion);
    }

    float rollRateDesired = 0.0f;
    float pitchRateDesired = 0.0f;
    float yawRateDesired = 0.0f;

    if (setpoint->mode.roll == modeVelocity) {
        rollRateDesired = setpoint->attitudeRate.roll;
        resetPidObject(&pidRoll);
    } else if (setpoint->mode.roll == modeAbs) {
        rollRateDesired = updatePidObject(&pidRoll, state->attitude.roll,
                                          setpoint->attitude.roll, false);
        pidRoll.output = (float)saturateSignedInt16((int32_t)pidRoll.output);
    }

    if (setpoint->mode.pitch == modeVelocity) {
        pitchRateDesired = setpoint->attitudeRate.pitch;
        resetPidObject(&pidPitch);
    } else if (setpoint->mode.pitch == modeAbs) {
        pitchRateDesired = updatePidObject(&pidPitch, state->attitude.pitch,
                                           setpoint->attitude.pitch, false);
        pidPitch.output = (float)saturateSignedInt16((int32_t)pidPitch.output);
    }

    if (setpoint->mode.yaw != modeVelocity) {
        yawRateDesired = updatePidObject(&pidYaw, state->attitude.yaw,
                                         s_desiredYaw, true);
        pidYaw.output = (float)saturateSignedInt16((int32_t)pidYaw.output);
    }

    float pitchActual = -sensors->gyro.y;
    attitudeControllerCorrectRatePID(sensors->gyro.x, rollRateDesired,
                                     pitchActual, pitchRateDesired,
                                     sensors->gyro.z, yawRateDesired);

    int16_t rollOut = 0;
    int16_t pitchOut = 0;
    int16_t yawOut = 0;
    attitudeControllerGetActuatorOutput(&rollOut, &pitchOut, &yawOut);

    control->roll = rollOut;
    control->pitch = pitchOut;
    control->yaw = (int16_t)(-yawOut);
    control->thrust = thrust;
}

bool thrustLocked = false;
bool commanderModeSet = false;

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime)
{
    if (!rollPrime || !pitchPrime) {
        return;
    }
    float yawRad = yaw_deg * DEG_TO_RAD;
    float c = cosf(yawRad);
    float s = sinf(yawRad);
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
    YawMode yawMode)
{
    if (!values || !setpoint) {
        return;
    }

    memset(setpoint, 0, sizeof(*setpoint));
    float rawRoll = values->roll;
    float rawPitch = values->pitch;
    float rawYaw = values->yaw;
    uint16_t rawThrust = values->thrust;

    if (yawMode == CAREFREE) {
        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->mode.yaw = modeAbs;
        setpoint->attitude.roll = rawRoll;
        setpoint->attitude.pitch = rawPitch;
        setpoint->attitude.yaw = rawYaw;
        setpoint->thrust = 0U;
        return;
    }

    if (commanderModeSet && !altHoldMode) {
        commanderModeSet = false;
    }

    if (altHoldMode) {
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0U;
        setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
        if (!commanderModeSet) {
            attitudeControllerResetAllPID(0.0f, 0.0f, 0.0f);
            resetPidObject(&g_posZPid);
            commanderModeSet = true;
        }
    } else {
        setpoint->mode.z = modeDisable;
        if (thrustLocked || rawThrust < MIN_THRUST) {
            setpoint->thrust = 0U;
            if (rawThrust == 0U) {
                thrustLocked = false;
            }
        } else {
            setpoint->thrust = rawThrust > MAX_THRUST ? MAX_THRUST : rawThrust;
        }
    }

    if (posHoldMode) {
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->velocity.x = rawPitch / 30.0f;
        setpoint->velocity.y = rawRoll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        return;
    }

    if (posSetMode && rawThrust != 0U) {
        setpoint->mode.x = modeAbs;
        setpoint->mode.y = modeAbs;
        setpoint->mode.z = modeAbs;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->mode.yaw = modeAbs;
        setpoint->position.x = -rawPitch;
        setpoint->position.y = rawRoll;
        setpoint->position.z = (float)rawThrust / 1000.0f;
        setpoint->attitude.yaw = rawYaw;
        setpoint->thrust = 0U;
        return;
    }

    if (yawMode == PLUSMODE) {
        rotateYaw(rawRoll, rawPitch, 45.0f, &rawRoll, &rawPitch);
    } else if (yawMode == XMODE) {
        rotateYaw(rawRoll, rawPitch, 0.0f, &rawRoll, &rawPitch);
    }

    if (stabilizationModeRoll == RATE) {
        setpoint->mode.roll = modeVelocity;
        setpoint->attitudeRate.roll = rawRoll;
    } else {
        setpoint->mode.roll = modeAbs;
        setpoint->attitude.roll = rawRoll;
    }

    if (stabilizationModePitch == RATE) {
        setpoint->mode.pitch = modeVelocity;
        setpoint->attitudeRate.pitch = rawPitch;
    } else {
        setpoint->mode.pitch = modeAbs;
        setpoint->attitude.pitch = rawPitch;
    }

    if (stabilizationModeYaw == RATE) {
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = -rawYaw;
    } else {
        setpoint->mode.yaw = modeAbs;
        setpoint->attitude.yaw = rawYaw;
    }
}

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0U;

static bool s_supervisorInitialized = false;
static bool s_armed = false;
static bool s_crashed = false;
static bool s_tumbled = false;
static bool s_freeFallingGlobal = false;
static bool s_autoArming = false;
static bool s_seenFlight = false;
static bool s_isFlying = false;
static uint32_t s_recentFlightTick = 0U;
static uint32_t s_spinupStartTick = 0U;
static uint32_t s_spinupTimeoutDuration = 0U;
static uint32_t s_latestArmingTick = 0U;
static uint32_t s_latestLandingTick = 0U;
static uint32_t s_tiltStartTick = 0U;
static uint32_t s_upsideDownStartTick = 0U;
static bool s_tiltTimerActive = false;
static bool s_upsideDownTimerActive = false;
static uint32_t s_motorNotRespStartTick = 0U;
static bool s_motorNotRespActive = false;
static bool s_motorNotRespFault = false;
static SensorData s_supervisorSensor;
static uint32_t s_motorRatios[4] = {0U, 0U, 0U, 0U};
static int32_t s_motorRPMs[4] = {0, 0, 0, 0};
static uint32_t s_idleThrust = 0U;
static float s_crashDetectionGs = 0.0f;
static float s_freeFallThreshold = 0.0f;
static float s_acceptedTiltAccZ = 0.0f;
static float s_acceptedUpsideDownAccZ = -1.0f;
static uint32_t s_maxTiltTime = 1000U;
static uint32_t s_maxUpsideDownTime = 250U;
static bool s_tumbleCheckEnabled = true;
static bool s_crtpEmergencyStop = false;
static bool s_paramEmergencyStop = false;
static bool s_emergencyWatchdogFailed = false;

static bool supervisorAllowedArmingHold(SupervisorState state)
{
    return state == supervisorStateArming ||
           state == supervisorStateReadyToFly ||
           state == supervisorStateFlying ||
           state == supervisorStateWarningLevelOut ||
           state == supervisorStateLanded;
}

void supervisorInit(void)
{
    if (s_supervisorInitialized) {
        supervisorState = supervisorStateLocked;
        supervisorConditionBits = 0U;
        s_armed = false;
        s_crashed = false;
        s_tumbled = false;
        s_freeFallingGlobal = false;
        s_seenFlight = false;
        s_isFlying = false;
        s_recentFlightTick = 0U;
        s_spinupStartTick = 0U;
        s_latestArmingTick = 0U;
        s_latestLandingTick = 0U;
        s_tiltTimerActive = false;
        s_upsideDownTimerActive = false;
        s_motorNotRespActive = false;
        s_motorNotRespFault = false;
        memset(&s_supervisorSensor, 0, sizeof(s_supervisorSensor));
        memset(s_motorRatios, 0, sizeof(s_motorRatios));
        memset(s_motorRPMs, 0, sizeof(s_motorRPMs));
        s_idleThrust = 0U;
        supervisorLog.info = 0U;
        supervisorLog.accNorm = 0.0f;
        return;
    }

    supervisorState = supervisorStateLocked;
    supervisorConditionBits = 0U;
    s_armed = false;
    s_crashed = false;
    s_tumbled = false;
    s_freeFallingGlobal = false;
    s_seenFlight = false;
    s_isFlying = false;
    s_recentFlightTick = 0U;
    s_spinupStartTick = 0U;
    s_latestArmingTick = 0U;
    s_latestLandingTick = 0U;
    s_tiltTimerActive = false;
    s_upsideDownTimerActive = false;
    s_motorNotRespActive = false;
    s_motorNotRespFault = false;
    memset(&s_supervisorSensor, 0, sizeof(s_supervisorSensor));
    memset(s_motorRatios, 0, sizeof(s_motorRatios));
    memset(s_motorRPMs, 0, sizeof(s_motorRPMs));
    s_idleThrust = 0U;
    supervisorLog.info = 0U;
    supervisorLog.accNorm = 0.0f;
    s_supervisorInitialized = true;
}

bool supervisorCanFly(void)
{
    return supervisorState == supervisorStateReadyToFly ||
           supervisorState == supervisorStateFlying ||
           supervisorState == supervisorStateWarningLevelOut ||
           supervisorState == supervisorStateLanded;
}

bool supervisorCanArm(void)
{
    return supervisorState == supervisorStatePreFlChecksPassed;
}

bool supervisorIsArmed(void)
{
    return (supervisorConditionBits & SUPERVISOR_CB_ARMED) != 0U;
}

bool supervisorIsCrashed(void)
{
    return s_crashed ||
           supervisorState == supervisorStateCrashed ||
           (supervisorConditionBits & SUPERVISOR_CB_CRASHED) != 0U;
}

bool supervisorAreMotorsAllowedToRun(void)
{
    return supervisorState == supervisorStateArming ||
           supervisorState == supervisorStateReadyToFly ||
           supervisorState == supervisorStateFlying ||
           supervisorState == supervisorStateWarningLevelOut ||
           supervisorState == supervisorStateLanded;
}

bool supervisorRequestArming(bool doArm)
{
    if (doArm) {
        if (!supervisorCanArm()) {
            return false;
        }
        if (s_armed && supervisorState == supervisorStateArming) {
            return true;
        }
        s_armed = true;
        supervisorConditionBits |= SUPERVISOR_CB_ARMED;
        supervisorState = supervisorStateArming;
        s_spinupStartTick = 0U;
        s_latestArmingTick = g_currentTick;
        return true;
    } else {
        bool wasCleared = s_armed;
        s_armed = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
        if (supervisorState == supervisorStateArming) {
            supervisorState = supervisorStatePreFlChecksPassed;
        }
        return wasCleared;
    }
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
    if (s_tumbled) {
        return false;
    }
    if (!doRecovery) {
        s_crashed = true;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        supervisorState = supervisorStateCrashed;
        return true;
    }
    s_crashed = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    if (supervisorState == supervisorStateCrashed) {
        supervisorState = supervisorStateReset;
    }
    return true;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick)
{
    if (!motorRatios) {
        return false;
    }
    for (int i = 0; i < 4; i++) {
        if (motorRatios[i] > idleThrust) {
            s_recentFlightTick = currentTick;
            s_seenFlight = true;
        }
    }
    if (!s_seenFlight) {
        s_isFlying = false;
        return false;
    }
    bool flying = (currentTick - s_recentFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
    s_isFlying = flying;
    return flying;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling)
{
    if (!isFreeFalling) {
        return false;
    }
    *isFreeFalling = false;
    if (!tumbleCheckEnabled) {
        s_tumbled = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
        return false;
    }

    float accNorm = sqrtf(accX * accX + accY * accY + accZ * accZ);
    if (crashDetectionGs > 0.0f && fabsf(accNorm - 1.0f) > crashDetectionGs) {
        s_crashed = true;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    }

    if (fabsf(accX) < freeFallThreshold &&
        fabsf(accY) < freeFallThreshold &&
        fabsf(accZ) < freeFallThreshold) {
        *isFreeFalling = true;
        s_freeFallingGlobal = true;
        supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
        supervisorState = supervisorStateExceptFreeFall;
        s_tiltTimerActive = false;
        s_upsideDownTimerActive = false;
        s_tiltStartTick = 0U;
        s_upsideDownStartTick = 0U;
        s_tumbled = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
        return false;
    }

    s_freeFallingGlobal = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

    bool tumbledNow = false;
    if (accZ >= acceptedTiltAccZ) {
        s_tiltTimerActive = false;
        s_upsideDownTimerActive = false;
        s_tiltStartTick = 0U;
        s_upsideDownStartTick = 0U;
        s_tumbled = false;
    } else if (accZ < acceptedUpsideDownAccZ) {
        if (!s_upsideDownTimerActive) {
            s_upsideDownStartTick = currentTick;
            s_upsideDownTimerActive = true;
            s_tiltTimerActive = false;
        } else if ((currentTick - s_upsideDownStartTick) >= maxUpsideDownTime) {
            tumbledNow = true;
        }
    } else {
        if (!s_tiltTimerActive) {
            s_tiltStartTick = currentTick;
            s_tiltTimerActive = true;
            s_upsideDownTimerActive = false;
        } else if ((currentTick - s_tiltStartTick) >= maxTiltTime) {
            tumbledNow = true;
        }
    }

    if (tumbledNow) {
        s_tumbled = true;
        supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    }
    return s_tumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick)
{
    if (lastNotificationTick == 0U) {
        return true;
    }
    return (currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration)
{
    if (latestArmingTick == 0U) {
        return false;
    }
    if (state != supervisorStateReadyToFly) {
        return false;
    }
    return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration)
{
    if (latestLandingTick == 0U) {
        return false;
    }
    return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed)
{
    s_crtpEmergencyStop = crtpEmergencyStop;
    s_paramEmergencyStop = paramEmergencyStop;
    s_emergencyWatchdogFailed = emergencyStopWatchdogFailed;

    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
    }
    return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBitsIn,
                                SupervisorState state)
{
    (void)supervisorConditionBitsIn;
    if (!setpoint) {
        return;
    }
    switch (state) {
    case supervisorStateWarningLevelOut:
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->attitudeRate.roll = 0.0f;
        setpoint->attitudeRate.pitch = 0.0f;
        setpoint->attitudeRate.yaw = 0.0f;
        break;
    case supervisorStateArming:
    case supervisorStateReadyToFly:
    case supervisorStateFlying:
    case supervisorStateLanded:
        break;
    default:
        memset(setpoint, 0, sizeof(*setpoint));
        break;
    }
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax)
{
    if (!motorRPMs) {
        return false;
    }
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) {
            return false;
        }
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTick)
{
    if (!canFly) {
        s_motorNotRespActive = false;
        s_motorNotRespStartTick = 0U;
        s_motorNotRespFault = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return false;
    }
    if (!motorRPMs) {
        return false;
    }

    bool allBelow = true;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] >= rpmThreshold) {
            allBelow = false;
            break;
        }
    }

    if (!allBelow) {
        s_motorNotRespActive = false;
        s_motorNotRespStartTick = 0U;
        s_motorNotRespFault = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return false;
    }

    if (!s_motorNotRespActive) {
        s_motorNotRespStartTick = currentTick;
        s_motorNotRespActive = true;
    } else if ((currentTick - s_motorNotRespStartTick) >= rpmCheckDurationMs) {
        s_motorNotRespFault = true;
        supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return true;
    }
    return false;
}

void supervisorSetSensorData(const SensorData *sensors)
{
    if (!sensors) {
        return;
    }
    s_supervisorSensor = *sensors;
    supervisorLog.accNorm = sqrtf(sensors->acc.x * sensors->acc.x +
                                  sensors->acc.y * sensors->acc.y +
                                  sensors->acc.z * sensors->acc.z);
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust)
{
    if (!motorRatios) {
        return;
    }
    for (int i = 0; i < 4; i++) {
        s_motorRatios[i] = motorRatios[i];
    }
    s_idleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
    if (!motorRPMs) {
        return;
    }
    for (int i = 0; i < 4; i++) {
        s_motorRPMs[i] = motorRPMs[i];
    }
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled)
{
    s_crashDetectionGs = crashDetectionGs;
    s_freeFallThreshold = freeFallThreshold;
    s_acceptedTiltAccZ = acceptedTiltAccZ;
    s_acceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
    s_maxTiltTime = maxTiltTime;
    s_maxUpsideDownTime = maxUpsideDownTime;
    s_tumbleCheckEnabled = tumbleCheckEnabled;
    s_tiltTimerActive = false;
    s_upsideDownTimerActive = false;
    s_tiltStartTick = 0U;
    s_upsideDownStartTick = 0U;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs)
{
    s_autoArming = autoArming;
    s_spinupTimeoutDuration = spinupTimeoutDurationMs;
    s_spinupStartTick = 0U;
}

void supervisorUpdate(uint32_t stabilizerStep)
{
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) {
        return;
    }
    g_currentTick = stabilizerStep;

    if (supervisorState == supervisorStateLocked ||
        supervisorState == supervisorStatePreFlChecksNotPassed) {
        supervisorState = supervisorStatePreFlChecksPassed;
        if (s_autoArming) {
            supervisorRequestArming(true);
        }
    }

    if (supervisorState == supervisorStateArming) {
        if (s_spinupStartTick == 0U) {
            s_spinupStartTick = stabilizerStep;
        } else if ((stabilizerStep - s_spinupStartTick) >= s_spinupTimeoutDuration) {
            supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
        }
    } else {
        s_spinupStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }

    if (s_armed && !supervisorAllowedArmingHold(supervisorState)) {
        s_armed = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    }

    if (g_commanderLastUpdateTick != 0U) {
        uint32_t age = stabilizerStep - g_commanderLastUpdateTick;
        if (age > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
            supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
            supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
        } else if (age > COMMANDER_WDT_TIMEOUT_STABILIZE) {
            supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
            supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
        } else {
            supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
            supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
        }
    }

    isFlyingCheck(s_motorRatios, s_idleThrust, stabilizerStep);
    if (s_isFlying) {
        supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
    }

    bool freeFall = false;
    isTumbledCheck(s_supervisorSensor.acc.x, s_supervisorSensor.acc.y,
                   s_supervisorSensor.acc.z, s_crashDetectionGs,
                   s_freeFallThreshold, s_acceptedTiltAccZ,
                   s_acceptedUpsideDownAccZ, s_maxTiltTime,
                   s_maxUpsideDownTime, s_tumbleCheckEnabled,
                   stabilizerStep, &freeFall);
    (void)freeFall;

    if (supervisorIsPreflightTimeout(supervisorState, s_latestArmingTick,
                                     stabilizerStep, 500U)) {
        supervisorConditionBits |= SUPERVISOR_CB_PREFLIGHT_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_PREFLIGHT_TIMEOUT;
    }

    if (supervisorIsLandingTimeout(s_latestLandingTick, stabilizerStep, 500U)) {
        supervisorConditionBits |= SUPERVISOR_CB_LANDING_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_LANDING_TIMEOUT;
    }

    bool wdtHealthy = checkEmergencyStopWatchdog(stabilizerStep, s_latestArmingTick);
    if (!wdtHealthy && s_emergencyWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    }

    supervisorLog.info = supervisorConditionBits;
}

uint16_t supervisorGetInfoBitfield(void)
{
    uint16_t bits = 0U;
    if (supervisorCanArm()) {
        bits |= (1U << 0);
    }
    if (supervisorIsArmed()) {
        bits |= (1U << 1);
    }
    if (s_autoArming) {
        bits |= (1U << 2);
    }
    if (supervisorCanFly()) {
        bits |= (1U << 3);
    }
    if (s_isFlying) {
        bits |= (1U << 4);
    }
    if (s_tumbled) {
        bits |= (1U << 5);
    }
    if (supervisorState == supervisorStateLocked) {
        bits |= (1U << 6);
    }
    if (supervisorIsCrashed()) {
        bits |= (1U << 7);
    }
    if (false) {
        bits |= (1U << 8);
    }
    if (false) {
        bits |= (1U << 9);
    }
    if (false) {
        bits |= (1U << 10);
    }
    if ((supervisorConditionBits & SUPERVISOR_CB_DECK_FAULT) != 0U) {
        bits |= (1U << 11);
    }
    return bits;
}

static EstimatorMeasurement g_estFifo[16];
static uint8_t g_estHead = 0U;
static uint8_t g_estTail = 0U;
static uint8_t g_estCount = 0U;
static EstimatorMeasurement g_lastGyro;
static EstimatorMeasurement g_lastAcc;
static EstimatorMeasurement g_lastBaro;
static EstimatorMeasurement g_lastTof;
static bool g_hasGyro = false;
static bool g_hasAcc = false;
static bool g_hasBaro = false;
static bool g_hasTof = false;
static State g_state;

bool estimatorEnqueue(const EstimatorMeasurement *measurement)
{
    if (!measurement) {
        return false;
    }
    if (g_estCount >= 16U) {
        return false;
    }
    g_estFifo[g_estTail] = *measurement;
    g_estTail = (uint8_t)((g_estTail + 1U) % 16U);
    g_estCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement)
{
    if (!measurement || g_estCount == 0U) {
        return false;
    }
    *measurement = g_estFifo[g_estHead];
    g_estHead = (uint8_t)((g_estHead + 1U) % 16U);
    g_estCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep)
{
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        switch (m.type) {
        case MeasurementTypeGyroscope:
            g_lastGyro = m;
            g_hasGyro = true;
            break;
        case MeasurementTypeAcceleration:
            g_lastAcc = m;
            g_hasAcc = true;
            break;
        case MeasurementTypeBarometer:
            g_lastBaro = m;
            g_hasBaro = true;
            break;
        case MeasurementTypeTOF:
            g_lastTof = m;
            g_hasTof = true;
            break;
        default:
            break;
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        float gx = g_hasGyro ? g_lastGyro.data[0] : 0.0f;
        float gy = g_hasGyro ? g_lastGyro.data[1] : 0.0f;
        float gz = g_hasGyro ? g_lastGyro.data[2] : 0.0f;
        float ax = g_hasAcc ? g_lastAcc.data[0] : 0.0f;
        float ay = g_hasAcc ? g_lastAcc.data[1] : 0.0f;
        float az = g_hasAcc ? g_lastAcc.data[2] : 0.0f;
        sensfusion6UpdateQ(gx, gy, gz, ax, ay, az, 0.004f);
        sensfusion6GetEulerRPY(&g_state.attitude.roll,
                               &g_state.attitude.pitch,
                               &g_state.attitude.yaw);
        sensfusion6GetQuaternion(&g_state.attitudeQuaternion.w,
                                 &g_state.attitudeQuaternion.x,
                                 &g_state.attitudeQuaternion.y,
                                 &g_state.attitudeQuaternion.z);
        float accZ = sensfusion6GetAccZWithoutGravity(ax, ay, az);
        g_state.velocity.z += accZ * 9.81f * 0.004f;
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        g_state.position.x += g_state.velocity.x * 0.01f;
        g_state.position.y += g_state.velocity.y * 0.01f;
        g_state.position.z += g_state.velocity.z * 0.01f;
    }
}

static Setpoint g_commanderSetpoint;
static int g_commanderPriority = COMMANDER_PRIORITY_LOWEST;
static uint32_t g_commanderLastUpdateTick = 0U;

bool commanderSetSetpoint(const Setpoint *setpoint, int priority)
{
    if (!setpoint) {
        return false;
    }
    if (priority == COMMANDER_PRIORITY_DISABLE || priority >= g_commanderPriority) {
        g_commanderSetpoint = *setpoint;
        g_commanderPriority = priority;
        g_commanderLastUpdateTick = g_currentTick;
        g_commanderSetpoint.timestamp = g_currentTick;
        if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
        }
        return true;
    }
    return false;
}

void commanderRelaxPriority(void)
{
    g_commanderPriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void)
{
    return g_currentTick - g_commanderLastUpdateTick;
}

int commanderGetActivePriority(void)
{
    return g_commanderPriority;
}

static bool s_stabilizerInitialized = false;
static bool s_highLevelPending = false;
static Setpoint s_highLevelSetpoint;
static SensorData g_sensorData;
static ControlData g_control;
static MotorPower g_motorPower;
static uint16_t g_motorPwm[4] = {0U, 0U, 0U, 0U};
static float g_filteredBatteryVoltage = NOMINAL_BATTERY_VOLTAGE;
static Setpoint g_activeSetpoint;

static void sensorsInit(void) { }
static void stateEstimatorInit(void)
{
    memset(&g_state, 0, sizeof(g_state));
    g_state.attitudeQuaternion.w = 1.0f;
    sensfusion6Init();
}
static void controllerInit(void)
{
    attitudeControllerInit(0.002f);
}
static void powerDistributionInit(void) { }
static void motorsInit(void)
{
    memset(g_motorPwm, 0, sizeof(g_motorPwm));
    motor.m1req = 0U;
    motor.m2req = 0U;
    motor.m3req = 0U;
    motor.m4req = 0U;
}
static void collisionAvoidanceInit(void) { }
static bool sensorsWaitDataReady(void) { return true; }
static void sensorsAcquire(void) { }
static void stateEstimator(void)
{
    estimatorComplementary(g_stabilizerStep);
}
static void commanderGetSetpoint(void)
{
    g_activeSetpoint = g_commanderSetpoint;
}
static void collisionAvoidanceUpdateSetpoint(void) { }
static void setMotorRatios(const MotorPower *power)
{
    if (!power) {
        return;
    }
    g_motorPwm[0] = (uint16_t)(power->m1 < 0 ? 0 : (power->m1 > 65535 ? 65535 : power->m1));
    g_motorPwm[1] = (uint16_t)(power->m2 < 0 ? 0 : (power->m2 > 65535 ? 65535 : power->m2));
    g_motorPwm[2] = (uint16_t)(power->m3 < 0 ? 0 : (power->m3 > 65535 ? 65535 : power->m3));
    g_motorPwm[3] = (uint16_t)(power->m4 < 0 ? 0 : (power->m4 > 65535 ? 65535 : power->m4));
}
static void stopMotors(void)
{
    memset(g_motorPwm, 0, sizeof(g_motorPwm));
    motor.m1req = 0U;
    motor.m2req = 0U;
    motor.m3req = 0U;
    motor.m4req = 0U;
}

static int8_t floatToSigned8(float v)
{
    int32_t t = (int32_t)(v * 127.0f);
    if (t > 127) {
        t = 127;
    }
    if (t < -127) {
        t = -127;
    }
    return (int8_t)t;
}

static uint32_t quatcompress(float qwIn, float qxIn, float qyIn, float qzIn)
{
    uint32_t result = 0U;
    result |= ((uint32_t)(uint8_t)floatToSigned8(qwIn)) << 24U;
    result |= ((uint32_t)(uint8_t)floatToSigned8(qxIn)) << 16U;
    result |= ((uint32_t)(uint8_t)floatToSigned8(qyIn)) << 8U;
    result |= ((uint32_t)(uint8_t)floatToSigned8(qzIn));
    return result;
}

void stabilizerInit(void)
{
    if (s_stabilizerInitialized) {
        return;
    }
    sensorsInit();
    stateEstimatorInit();
    controllerInit();
    powerDistributionInit();
    motorsInit();
    collisionAvoidanceInit();
    supervisorInit();
    crtpInit();
    memset(&g_sensorData, 0, sizeof(g_sensorData));
    memset(&g_control, 0, sizeof(g_control));
    memset(&g_motorPower, 0, sizeof(g_motorPower));
    memset(&g_activeSetpoint, 0, sizeof(g_activeSetpoint));
    s_stabilizerInitialized = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint)
{
    if (!setpoint) {
        return false;
    }
    s_highLevelSetpoint = *setpoint;
    s_highLevelPending = true;
    return true;
}

static void syncLogs(void)
{
    stateEstimate.roll = g_state.attitude.roll;
    stateEstimate.pitch = g_state.attitude.pitch;
    stateEstimate.yaw = g_state.attitude.yaw;
    stateEstimate.qw = g_state.attitudeQuaternion.w;
    stateEstimate.qx = g_state.attitudeQuaternion.x;
    stateEstimate.qy = g_state.attitudeQuaternion.y;
    stateEstimate.qz = g_state.attitudeQuaternion.z;
    gyro.x = g_sensorData.gyro.x;
    gyro.y = g_sensorData.gyro.y;
    gyro.z = g_sensorData.gyro.z;
    acc.x = g_sensorData.acc.x;
    acc.y = g_sensorData.acc.y;
    acc.z = g_sensorData.acc.z;
    baro.asl = g_sensorData.baroAsl;
    baro.temp = g_sensorData.baroTemperature;
    baro.pressure = g_sensorData.baroPressure;
    motor.m1req = g_motorPwm[0];
    motor.m2req = g_motorPwm[1];
    motor.m3req = g_motorPwm[2];
    motor.m4req = g_motorPwm[3];
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
    supervisorLog.info = supervisorConditionBits;
    healthLog.motorPass = motorPass;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
}

void stabilizerTask(void)
{
    if (!s_stabilizerInitialized) {
        return;
    }
    g_stabilizerStep++;
    g_currentTick = g_stabilizerStep;

    if (healthShallWeRunTest()) {
        healthRunTests(&g_sensorData);
        return;
    }

    if (s_highLevelPending) {
        commanderSetSetpoint(&s_highLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        s_highLevelPending = false;
    }

    sensorsWaitDataReady();
    sensorsAcquire();
    stateEstimator();
    commanderGetSetpoint();
    supervisorUpdate(g_stabilizerStep);
    collisionAvoidanceUpdateSetpoint();
    supervisorOverrideSetpoint(&g_activeSetpoint, supervisorConditionBits, supervisorState);

    if (!supervisorCanFly()) {
        memset(&g_activeSetpoint, 0, sizeof(g_activeSetpoint));
        memset(&g_control, 0, sizeof(g_control));
        stopMotors();
        syncLogs();
        return;
    }

    controllerPid(&g_sensorData, &g_activeSetpoint, &g_state, &g_control,
                  0.0f, 0.002f);
    powerDistribution(&g_control, &g_motorPower);

    int32_t motorInts[4] = {g_motorPower.m1, g_motorPower.m2,
                            g_motorPower.m3, g_motorPower.m4};
    g_filteredBatteryVoltage = batteryCompensation(NOMINAL_BATTERY_VOLTAGE,
                                                    g_filteredBatteryVoltage, 0.01f);
    for (int i = 0; i < 4; i++) {
        if (motorInts[i] < 0) {
            motorInts[i] = 0;
        }
        if (motorInts[i] > 65535) {
            motorInts[i] = 65535;
        }
        motorInts[i] = (int32_t)motosCompensateBatteryVoltage(
            (uint16_t)motorInts[i], NOMINAL_BATTERY_VOLTAGE,
            g_filteredBatteryVoltage);
    }

    PowerCapResult cap = powerDistributionCap(motorInts, 65535, 0);
    (void)cap;
    g_motorPower.m1 = motorInts[0];
    g_motorPower.m2 = motorInts[1];
    g_motorPower.m3 = motorInts[2];
    g_motorPower.m4 = motorInts[3];

    if (supervisorAreMotorsAllowedToRun()) {
        setMotorRatios(&g_motorPower);
    } else {
        stopMotors();
    }
    syncLogs();
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output)
{
    if (!state || !sensors || !output) {
        return;
    }
    output->position_mm[0] = (int32_t)(state->position.x * 1000.0f);
    output->position_mm[1] = (int32_t)(state->position.y * 1000.0f);
    output->position_mm[2] = (int32_t)(state->position.z * 1000.0f);
    output->velocity_mms[0] = (int32_t)(state->velocity.x * 1000.0f);
    output->velocity_mms[1] = (int32_t)(state->velocity.y * 1000.0f);
    output->velocity_mms[2] = (int32_t)(state->velocity.z * 1000.0f);
    output->acceleration_mms2[0] = (int32_t)(sensors->acc.x * 9810.0f);
    output->acceleration_mms2[1] = (int32_t)(sensors->acc.y * 9810.0f);
    output->acceleration_mms2[2] = (int32_t)((sensors->acc.z + 1.0f) * 9810.0f);
    output->gyro_millirad_s[0] = sensors->gyro.x * PI_F / 180.0f * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * PI_F / 180.0f * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * PI_F / 180.0f * 1000.0f;
    output->quatCompressed = quatcompress(state->attitudeQuaternion.w,
                                           state->attitudeQuaternion.x,
                                           state->attitudeQuaternion.y,
                                           state->attitudeQuaternion.z);
}

bool rateSupervisorValidate(uint32_t measuredRate)
{
    return measuredRate >= 997U && measuredRate <= 1003U;
}

static uint32_t s_rateSupervisorStartTick = 0U;
static bool s_rateSensorActive = false;
static bool s_rateError = false;

void rateSupervisorTask(void)
{
    if (s_rateSupervisorStartTick == 0U) {
        s_rateSupervisorStartTick = g_currentTick;
        return;
    }
    if ((g_currentTick - s_rateSupervisorStartTick) < 2000U) {
        return;
    }
    if (s_rateSensorActive && !rateSupervisorValidate(1000U)) {
        s_rateError = true;
    }
    s_rateSupervisorStartTick = g_currentTick;
}

TestState healthTestState = testDone;
uint8_t motorPass = 0U;
uint8_t batteryPass = 0U;
float batterySag = 0.0f;

static bool s_propTestRequested = false;
static bool s_batteryTestRequested = false;
static uint32_t s_propSampleCount = 0U;
static float s_propSampleSum = 0.0f;
static float s_propSampleSumSq = 0.0f;
static uint32_t s_propMotorIndex = 0U;
static float s_propNoiseVariance = 0.0f;
static uint32_t s_batTestTick = 0U;
static float s_batIdleVoltage = 0.0f;
static float s_batMinLoadedVoltage = 1e9f;

void healthRequestPropTest(void)
{
    s_propTestRequested = true;
}

void healthRequestBatteryTest(void)
{
    s_batteryTestRequested = true;
}

bool healthShallWeRunTest(void)
{
    if (s_propTestRequested) {
        s_propTestRequested = false;
        healthTestState = configureAcc;
        s_propSampleCount = 0U;
        s_propSampleSum = 0.0f;
        s_propSampleSumSq = 0.0f;
        s_propMotorIndex = 0U;
        motorPass = 0U;
        healthLog.motorPass = 0U;
        healthLog.motorTestCount = 0U;
        return true;
    }
    if (s_batteryTestRequested) {
        s_batteryTestRequested = false;
        healthTestState = testBattery;
        s_batTestTick = 0U;
        s_batIdleVoltage = 0.0f;
        s_batMinLoadedVoltage = 1e9f;
        batteryPass = 0U;
        batterySag = 0.0f;
        return true;
    }
    return false;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex)
{
    if (highThreshold == 0.0f) {
        return true;
    }
    if (motorIndex > 3U) {
        return false;
    }
    if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
        motorPass |= (uint8_t)(1U << motorIndex);
        healthLog.motorPass = motorPass;
        return true;
    }
    healthLog.motorTestCount++;
    return false;
}

float variance(const float *buffer, int length)
{
    if (!buffer || length <= 0) {
        return 0.0f;
    }
    float sum = 0.0f;
    float sumSq = 0.0f;
    for (int i = 0; i < length; i++) {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }
    return sumSq - (sum * sum / (float)length);
}

void healthRunTests(const SensorData *sensorData)
{
    if (!sensorData) {
        return;
    }
    switch (healthTestState) {
    case configureAcc:
        motorPass = 0U;
        healthLog.motorPass = 0U;
        healthLog.motorTestCount = 0U;
        s_propSampleCount = 0U;
        s_propSampleSum = 0.0f;
        s_propSampleSumSq = 0.0f;
        s_propMotorIndex = 0U;
        stopMotors();
        healthTestState = measureNoiseFloor;
        break;
    case measureNoiseFloor:
        if (s_propSampleCount < PROPTEST_NBR_OF_VARIANCE_VALUES) {
            float v = sqrtf(sensorData->acc.x * sensorData->acc.x +
                            sensorData->acc.y * sensorData->acc.y +
                            sensorData->acc.z * sensorData->acc.z);
            s_propSampleSum += v;
            s_propSampleSumSq += v * v;
            s_propSampleCount++;
        }
        if (s_propSampleCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
            float n = (float)s_propSampleCount;
            s_propNoiseVariance = s_propSampleSumSq - (s_propSampleSum * s_propSampleSum / n);
            healthTestState = measureProp;
            s_propMotorIndex = 0U;
        }
        break;
    case measureProp:
        if (s_propMotorIndex < 4U) {
            evaluatePropTest(0.0f, 1.0f, fabsf(sensorData->acc.z), (uint8_t)s_propMotorIndex);
            s_propMotorIndex++;
            if (s_propMotorIndex >= 4U) {
                healthTestState = evaluatePropResult;
            }
        }
        break;
    case evaluatePropResult:
        healthTestState = testDone;
        healthLog.motorPass = motorPass;
        healthLog.motorTestCount = healthLog.motorTestCount;
        break;
    case testBattery:
        s_batTestTick++;
        if (s_batTestTick == 1U) {
            g_motorPwm[0] = 1U;
            g_motorPwm[1] = 1U;
            g_motorPwm[2] = 1U;
            g_motorPwm[3] = 1U;
        } else if (s_batTestTick >= 2U && s_batTestTick <= 49U) {
            float voltage = 3.0f - (float)s_batTestTick * 0.001f;
            if (voltage < s_batMinLoadedVoltage) {
                s_batMinLoadedVoltage = voltage;
            }
        } else if (s_batTestTick >= 50U) {
            stopMotors();
            batterySag = s_batIdleVoltage - s_batMinLoadedVoltage;
            batteryPass = batterySag <= 1.0f ? 1U : 0U;
            healthLog.batteryPass = batteryPass;
            healthLog.batterySag = batterySag;
            healthTestState = evaluateBatResult;
        }
        break;
    case evaluateBatResult:
        healthTestState = testDone;
        healthLog.motorPass = motorPass;
        healthLog.batteryPass = batteryPass;
        healthLog.batterySag = batterySag;
        break;
    case restartBatTest:
        if ((g_currentTick - s_batTestTick) >= 2000U) {
            healthTestState = testBattery;
            s_batTestTick = 0U;
        }
        break;
    case testDone:
    default:
        break;
    }
}

StateEstimateLog stateEstimate;
Axis3Log gyro, acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

static CrtpPacket g_txQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t g_txHead = 0U;
static uint16_t g_txTail = 0U;
static uint16_t g_txCount = 0U;
static CrtpPacket g_rxQueue[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint8_t g_rxHead[CRTP_NBR_OF_PORTS];
static uint8_t g_rxTail[CRTP_NBR_OF_PORTS];
static uint8_t g_rxCount[CRTP_NBR_OF_PORTS];
static bool g_rxQueueCreated[CRTP_NBR_OF_PORTS];
static CrtpPortCallback g_portCb[CRTP_NBR_OF_PORTS];
static bool g_crtpInitialized = false;
static bool g_crtpError = false;
static CrtpLink *g_link = NULL;
static uint32_t g_rxPacketCount = 0U;
static uint32_t g_txPacketCount = 0U;
static uint32_t g_lastStatsTick = 0U;
static uint32_t g_rxRate = 0U;
static uint32_t g_txRate = 0U;
static uint32_t g_txRetryTick = 0U;

static bool nopSendPacket(CrtpPacket *packet)
{
    (void)packet;
    return false;
}
static bool nopReceivePacket(CrtpPacket *packet)
{
    (void)packet;
    return false;
}
static bool nopIsConnected(void)
{
    return true;
}
static void nopSetEnable(bool enable)
{
    (void)enable;
}
static void nopReset(void)
{
}

static CrtpLink g_nopLink = {
    nopSendPacket,
    nopReceivePacket,
    nopIsConnected,
    nopSetEnable,
    nopReset
};

void crtpInit(void)
{
    if (g_crtpInitialized) {
        return;
    }
    g_txHead = 0U;
    g_txTail = 0U;
    g_txCount = 0U;
    for (uint8_t i = 0; i < CRTP_NBR_OF_PORTS; i++) {
        g_rxHead[i] = 0U;
        g_rxTail[i] = 0U;
        g_rxCount[i] = 0U;
        g_rxQueueCreated[i] = false;
        g_portCb[i] = NULL;
    }
    g_link = &g_nopLink;
    g_crtpError = false;
    g_crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port)
{
    if (port >= CRTP_NBR_OF_PORTS) {
        return;
    }
    if (g_rxQueueCreated[port]) {
        g_crtpError = true;
        return;
    }
    g_rxHead[port] = 0U;
    g_rxTail[port] = 0U;
    g_rxCount[port] = 0U;
    g_rxQueueCreated[port] = true;
}

bool crtpSendPacket(const CrtpPacket *packet)
{
    if (!packet || g_txCount >= CRTP_TX_QUEUE_SIZE) {
        return false;
    }
    g_txQueue[g_txTail] = *packet;
    g_txTail = (uint16_t)((g_txTail + 1U) % CRTP_TX_QUEUE_SIZE);
    g_txCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet)
{
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet)
{
    if (port >= CRTP_NBR_OF_PORTS || !packet) {
        return false;
    }
    if (!g_rxQueueCreated[port] || g_rxCount[port] == 0U) {
        return false;
    }
    *packet = g_rxQueue[port][g_rxHead[port]];
    g_rxHead[port] = (uint8_t)((g_rxHead[port] + 1U) % CRTP_RX_QUEUE_SIZE);
    g_rxCount[port]--;
    return true;
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet)
{
    return crtpReceivePacket(port, packet);
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet, uint32_t wait_ms)
{
    (void)wait_ms;
    return crtpReceivePacket(port, packet);
}

void crtpRxTask(void)
{
    if (!g_link || g_link == &g_nopLink || !g_link->receivePacket) {
        return;
    }
    CrtpPacket packet;
    while (g_link->receivePacket(&packet)) {
        g_rxPacketCount++;
        if (packet.port < CRTP_NBR_OF_PORTS) {
            if (g_rxQueueCreated[packet.port]) {
                if (g_rxCount[packet.port] < CRTP_RX_QUEUE_SIZE) {
                    g_rxQueue[packet.port][g_rxTail[packet.port]] = packet;
                    g_rxTail[packet.port] = (uint8_t)((g_rxTail[packet.port] + 1U) % CRTP_RX_QUEUE_SIZE);
                    g_rxCount[packet.port]++;
                }
            }
            if (g_portCb[packet.port]) {
                g_portCb[packet.port](&packet);
            }
        }
    }
}

void crtpTxTask(void)
{
    if (!g_link || g_link == &g_nopLink || !g_link->sendPacket) {
        return;
    }
    if (g_txCount == 0U) {
        return;
    }
    CrtpPacket *packet = &g_txQueue[g_txHead];
    if (g_link->sendPacket(packet)) {
        g_txHead = (uint16_t)((g_txHead + 1U) % CRTP_TX_QUEUE_SIZE);
        g_txCount--;
        g_txPacketCount++;
    } else {
        g_txRetryTick = g_currentTick + 10U;
    }
}

void crtpSetLink(CrtpLink *newLink)
{
    if (g_link && g_link != &g_nopLink && g_link->setEnable) {
        g_link->setEnable(false);
    }
    if (!newLink) {
        g_link = &g_nopLink;
    } else {
        g_link = newLink;
    }
    if (g_link && g_link->setEnable) {
        g_link->setEnable(true);
    }
}

void crtpReset(void)
{
    g_txHead = 0U;
    g_txTail = 0U;
    g_txCount = 0U;
    if (g_link && g_link->reset) {
        g_link->reset();
    }
}

bool crtpIsConnected(void)
{
    if (g_link && g_link->isConnected) {
        return g_link->isConnected();
    }
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void)
{
    return (uint32_t)(CRTP_TX_QUEUE_SIZE - g_txCount);
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback)
{
    if (port >= CRTP_NBR_OF_PORTS) {
        return;
    }
    g_portCb[port] = callback;
}

void updateStats(void)
{
    if ((g_currentTick - g_lastStatsTick) >= 500U) {
        g_rxRate = g_rxPacketCount;
        g_txRate = g_txPacketCount;
        g_rxPacketCount = 0U;
        g_txPacketCount = 0U;
        g_lastStatsTick = g_currentTick;
    }
}

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity)
{
    if (!decks || capacity == 0U) {
        return 0U;
    }
    static const uint8_t knownI2c[] = {0x10U, 0x20U, 0x30U, 0x40U, 0x50U};
    static const uint64_t knownOneWire[] = {0x123456789ULL, 0x987654321ULL, 0x111111111ULL};
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < (uint8_t)(sizeof(knownI2c) / sizeof(knownI2c[0])) && count < capacity; i++) {
        uint8_t addr = knownI2c[i];
        bool duplicate = false;
        for (uint8_t j = 0U; j < count; j++) {
            if (decks[j].foundByI2C && decks[j].i2cAddress == addr) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            decks[count].foundByI2C = true;
            decks[count].foundByOneWire = false;
            decks[count].i2cAddress = addr;
            decks[count].oneWireRomId = 0U;
            count++;
        }
    }

    for (uint8_t i = 0U; i < (uint8_t)(sizeof(knownOneWire) / sizeof(knownOneWire[0])) && count < capacity; i++) {
        uint64_t rom = knownOneWire[i];
        bool duplicate = false;
        for (uint8_t j = 0U; j < count; j++) {
            if (decks[j].foundByOneWire && decks[j].oneWireRomId == rom) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            decks[count].foundByI2C = false;
            decks[count].foundByOneWire = true;
            decks[count].i2cAddress = 0U;
            decks[count].oneWireRomId = rom;
            count++;
        }
    }
    return count;
}
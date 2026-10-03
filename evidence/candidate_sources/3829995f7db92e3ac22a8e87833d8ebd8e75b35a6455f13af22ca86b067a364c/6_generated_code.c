#include "6_generated_code.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define PI_F 3.14159265358979323846f

/* -------------------------------------------------------------------------
 * 1. Numeric utilities
 * ---------------------------------------------------------------------- */
int16_t saturateSignedInt16(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32767) return -32767;
    return (int16_t)value;
}

float capAngle(float angle_deg)
{
    float a = angle_deg;
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* -------------------------------------------------------------------------
 * 2. Sensfusion6 globals and functions
 * ---------------------------------------------------------------------- */
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void sensfusion6Init(void)
{
    if (sensfusion6IsInit) {
        return; /* repeated calls do not overwrite running state */
    }
    qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
    gravityX = 0.0f; gravityY = 0.0f; gravityZ = 1.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
    baseZacc = 0.0f;
    sensfusion6IsInit = true;
    sensfusion6IsCalibrated = false;
}

bool sensfusion6Test(void)
{
    return sensfusion6IsInit;
}

void estimatedGravityDirection(float qw_, float qx_, float qy_, float qz_,
                               float *gravX, float *gravY, float *gravZ)
{
    if (!gravX || !gravY || !gravZ) return;
    *gravX = 2.0f * (qx_ * qz_ - qw_ * qy_);
    *gravY = 2.0f * (qy_ * qz_ + qw_ * qx_);
    *gravZ = qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_;
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
    if (!sensfusion6IsInit) {
        sensfusion6Init();
    }
    float ax0 = ax, ay0 = ay, az0 = az;

    /* Normalize accelerometer unless all zero (prevents division by zero) */
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    bool accValid = (norm > 1e-6f);
    if (accValid) {
        ax /= norm; ay /= norm; az /= norm;
    }

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
    /* Madgwick 6-axis (acc only) */
    if (accValid) {
        float _2q0 = 2.0f * qw;
        float _2q1 = 2.0f * qx;
        float _2q2 = 2.0f * qy;
        float _2q3 = 2.0f * qz;
        float _2q0q2 = _2q0 * qz;
        float _2q1q3 = _2q1 * qz;
        float _2q2q2 = _2q2 * qy;
        float _2q1q1 = _2q1 * qx;
        float _2q2q1 = _2q2 * qx;
        float _2q3q3 = _2q3 * qz;
        float _2q3q2 = _2q3 * qy;
        float _2q3q1 = _2q3 * qx;

        float f1 = _2q2 - 1.0f;
        float f2 = _2q1 - 1.0f;
        float f3 = 1.0f - _2q2 - _2q3;
        float f4 = 1.0f - _2q1 - _2q2;
        float f5 = 1.0f - _2q1 - _2q3;

        float s0 = -_2q2 * (2.0f * f2 - 1.0f) + _2q1 * (2.0f * f1) - _2q3;
        float s1 = _2q1 * (2.0f * f1) + _2q2 * (2.0f * f3) - _2q3;
        float s2 = _2q0 * f1 + _2q0 * f5 - _2q2;
        float s3 = _2q1 * f4 - _2q0 * f2 + _2q2;

        float gerrx = 2.0f * (s0 * az - s2 * ax - s3 * ay);
        float gerry = 2.0f * (s1 * ax - s3 * az - s2 * ay);
        float gerrz = 2.0f * (s2 * ay - s1 * az - s0 * ax);

        gx += beta * gerrx;
        gy += beta * gerry;
        gz += beta * gerrz;
    }
    if (twoKi > 0.0f) {
        integralFBx += twoKi * gx * dt;
        integralFBy += twoKi * gy * dt;
        integralFBz += twoKi * gz * dt;
    } else {
        integralFBx = integralFBy = integralFBz = 0.0f;
    }
#else
    /* Default Mahony */
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    if (accValid) {
        float gx_est = 2.0f * (qx * qz - qw * qy);
        float gy_est = 2.0f * (qy * qz + qw * qx);
        float gz_est = qw * qw - qx * qx - qy * qy + qz * qz;
        halfex = (ay * gz_est - az * gy_est);
        halfey = (az * gx_est - ax * gz_est);
        halfez = (ax * gy_est - ay * gx_est);
    }
    if (twoKi > 0.0f) {
        integralFBx += twoKi * halfex * dt;
        integralFBy += twoKi * halfey * dt;
        integralFBz += twoKi * halfez * dt;
        gx += twoKp * halfex + integralFBx;
        gy += twoKp * halfey + integralFBy;
        gz += twoKp * halfez + integralFBz;
    } else {
        integralFBx = integralFBy = integralFBz = 0.0f;
        if (accValid) {
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }
    }
#endif

    /* Integrate quaternion using gyro rates */
    float qw_old = qw, qx_old = qx, qy_old = qy, qz_old = qz;
    qw += (-qx_old * gx - qy_old * gy - qz_old * gz) * 0.5f * dt;
    qx += ( qw_old * gx + qy_old * gz - qz_old * gy) * 0.5f * dt;
    qy += ( qw_old * gy - qx_old * gz + qz_old * gx) * 0.5f * dt;
    qz += ( qw_old * gz + qx_old * gy - qy_old * gx) * 0.5f * dt;

    /* Normalize quaternion */
    norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 1e-8f) {
        qw /= norm; qx /= norm; qy /= norm; qz /= norm;
    }

    /* Update gravity vector from current quaternion */
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);

    if (!sensfusion6IsCalibrated) {
        baseZacc = ax0 * gravityX + ay0 * gravityY + az0 * gravityZ;
        sensfusion6IsCalibrated = true;
    }
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (!roll_deg || !pitch_deg || !yaw_deg) return;
    float gx = gravityX, gy = gravityY, gz = gravityZ;
    /* Sync from current quaternion to avoid stale cache */
    estimatedGravityDirection(qw, qx, qy, qz, &gx, &gy, &gz);

    float sinp = 2.0f * (qw * qy - qz * qx);
    sinp = clampf(sinp, -1.0f, 1.0f);
    float pitch = asinf(sinp);
    float roll = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx*qx + qy*qy));
    float yaw = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy*qy + qz*qz));

    *roll_deg = roll * 180.0f / PI_F;
    *pitch_deg = pitch * 180.0f / PI_F;
    *yaw_deg = yaw * 180.0f / PI_F;
}

void sensfusion6GetQuaternion(float *qw_, float *qx_, float *qy_, float *qz_)
{
    if (qw_) *qw_ = qw;
    if (qx_) *qx_ = qx;
    if (qy_) *qy_ = qy;
    if (qz_) *qz_ = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az)
{
    float gx = gravityX, gy = gravityY, gz = gravityZ;
    estimatedGravityDirection(qw, qx, qy, qz, &gx, &gy, &gz);
    return ax * gx + ay * gy + az * gz;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az)
{
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

float invSqrt(float x)
{
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

/* -------------------------------------------------------------------------
 * 3. Power distribution and battery compensation
 * ---------------------------------------------------------------------- */
static uint16_t motorForceToPwm(float force)
{
    if (force <= 0.0f) return 0;
    if (force >= CRAZYFLIE_MAX_MOTOR_FORCE_N) return 65535;
    float ratio = force / CRAZYFLIE_MAX_MOTOR_FORCE_N;
    float pwm = ratio * 65535.0f + 0.5f;
    if (pwm > 65535.0f) pwm = 65535.0f;
    return (uint16_t)pwm;
}

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out)
{
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
                                  float thrustToTorque, float motorForces[4])
{
    if (!motorForces) return;
    float thrustPart = 0.25f * thrustSi;
    float arm = 0.707106781f * armLength;
    float rollPart = 0.0f;
    float pitchPart = 0.0f;
    float yawPart = 0.0f;
    if (armLength != 0.0f) {
        rollPart = 0.25f / arm * torqueX;
        pitchPart = 0.25f / arm * torqueY;
    }
    if (thrustToTorque != 0.0f) {
        yawPart = 0.25f / thrustToTorque * torqueZ;
    }

    float f[4];
    f[0] = thrustPart - rollPart + pitchPart + yawPart;
    f[1] = thrustPart - rollPart - pitchPart - yawPart;
    f[2] = thrustPart + rollPart - pitchPart + yawPart;
    f[3] = thrustPart + rollPart + pitchPart - yawPart;

    for (int i = 0; i < 4; i++) {
        motorForces[i] = f[i] > 0.0f ? f[i] : 0.0f;
    }
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4])
{
    if (!normalizedForces || !motorPWMs) return;
    for (int i = 0; i < 4; i++) {
        float nf = clampf(normalizedForces[i], 0.0f, 1.0f);
        float pwm = nf * 65535.0f + 0.5f;
        if (pwm > 65535.0f) pwm = 65535.0f;
        motorPWMs[i] = (uint16_t)pwm;
    }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower)
{
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
        /* Unknown mode: do not modify motorPower */
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
    PowerCapResult res = { false, 0 };
    if (!motors) return res;

    int32_t maxVal = motors[0];
    for (int i = 1; i < 4; i++) {
        if (motors[i] > maxVal) maxVal = motors[i];
    }

    if (maxVal > maxAllowedThrust) {
        res.reduction = maxVal - maxAllowedThrust;
        for (int i = 0; i < 4; i++) {
            motors[i] -= res.reduction;
            motors[i] = capMinThrust(motors[i], idleThrust);
        }
        res.isCapped = true;
    }
    return res;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha)
{
    return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust,
                                        float nominalVoltage,
                                        float actualVoltage)
{
    if (actualVoltage <= 0.0f) return motorThrust;
    float compensated = (float)motorThrust * nominalVoltage / actualVoltage;
    if (compensated < 0.0f) compensated = 0.0f;
    if (compensated > 65535.0f) compensated = 65535.0f;
    return (uint16_t)compensated;
}

/* -------------------------------------------------------------------------
 * 4. PID controller
 * ---------------------------------------------------------------------- */
PidObject pidRoll, pidPitch, pidYaw;
PidObject pidRollRate, pidPitchRate, pidYawRate;

static float pidUpdateDt = 0.002f;

static void pidObjectInit(PidObject *pid)
{
    if (!pid) return;
    pid->kp = pid->ki = pid->kd = pid->kff = 0.0f;
    pid->integral = pid->prevError = pid->output = 0.0f;
    pid->initialized = true;
}

static void pidReset(PidObject *pid)
{
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
}

static void pidUpdate(PidObject *pid, float desired, float actual)
{
    if (!pid || !pid->initialized) return;
    float error = desired - actual;
    float dt = pidUpdateDt;
    float p = pid->kp * error;
    float i = pid->integral;
    float d = 0.0f;
    if (dt > 1e-6f) {
        d = pid->kd * (error - pid->prevError) / dt;
    }
    float ff = pid->kff * desired;
    pid->output = p + i + d + ff;
    pid->prevError = error;
    pid->integral += pid->ki * error * dt;
}

void attitudeControllerInit(float updateDt)
{
    static bool initialized = false;
    if (initialized) return;
    initialized = true;
    pidUpdateDt = updateDt;
    pidObjectInit(&pidRoll);
    pidObjectInit(&pidPitch);
    pidObjectInit(&pidYaw);
    pidObjectInit(&pidRollRate);
    pidObjectInit(&pidPitchRate);
    pidObjectInit(&pidYawRate);
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired)
{
    pidUpdate(&pidRollRate, rollDesired, rollActual);
    pidUpdate(&pidPitchRate, pitchDesired, pitchActual);
    pidUpdate(&pidYawRate, yawDesired, yawActual);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired)
{
    pidUpdate(&pidRoll, rollDesired, rollActual);
    pidUpdate(&pidPitch, pitchDesired, pitchActual);
    pidUpdate(&pidYaw, yawDesired, yawActual);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual)
{
    (void)rollActual; (void)pitchActual; (void)yawActual;
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
}

void attitudeControllerResetPitchAttitudePID(float pitchActual)
{
    (void)pitchActual;
    pidReset(&pidPitch);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw)
{
    if (roll) *roll = saturateSignedInt16((int32_t)pidRollRate.output);
    if (pitch) *pitch = saturateSignedInt16((int32_t)pidPitchRate.output);
    if (yaw) *yaw = saturateSignedInt16((int32_t)pidYawRate.output);
}

static float posIntegralZ = 0.0f;
static float posPrevErrorZ = 0.0f;

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state)
{
    if (!setpoint || !state) return 0;
    float zError = setpoint->position.z - state->position.z;
    float zVelError = setpoint->velocity.z - state->velocity.z;
    float output = 1500.0f * zError + 100.0f * zVelError + posIntegralZ;
    if (output < 0.0f) output = 0.0f;
    if (output > 65535.0f) output = 65535.0f;
    posPrevErrorZ = zError;
    posIntegralZ += 0.1f * zError;
    return (uint16_t)output;
}

static float desiredYaw = 0.0f;

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt)
{
    if (!sensors || !setpoint || !state || !control) return;

    /* Update desired yaw */
    if (setpoint->mode.yaw == modeVelocity) {
        desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
        if (yawMaxDelta != 0.0f) {
            float diff = desiredYaw - state->attitude.yaw;
            if (diff > yawMaxDelta) desiredYaw = state->attitude.yaw + yawMaxDelta;
            else if (diff < -yawMaxDelta) desiredYaw = state->attitude.yaw - yawMaxDelta;
        }
    } else if (setpoint->mode.yaw == modeAbs) {
        desiredYaw = setpoint->attitude.yaw;
    } else if (setpoint->mode.quat == modeAbs) {
        Quaternion q = setpoint->attitudeQuaternion;
        float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
        float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
        desiredYaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI_F;
    } else {
        desiredYaw = state->attitude.yaw;
    }

    /* Desired rates from attitude controllers */
    float rollDesiredRate, pitchDesiredRate, yawDesiredRate;

    if (setpoint->mode.roll == modeVelocity) {
        attitudeControllerResetRollAttitudePID(state->attitude.roll);
        rollDesiredRate = setpoint->attitudeRate.roll;
    } else {
        pidUpdate(&pidRoll, setpoint->attitude.roll, state->attitude.roll);
        rollDesiredRate = pidRoll.output;
    }

    if (setpoint->mode.pitch == modeVelocity) {
        attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
        pitchDesiredRate = setpoint->attitudeRate.pitch;
    } else {
        pidUpdate(&pidPitch, setpoint->attitude.pitch, state->attitude.pitch);
        pitchDesiredRate = pidPitch.output;
    }

    if (setpoint->mode.yaw == modeVelocity) {
        yawDesiredRate = setpoint->attitudeRate.yaw;
    } else {
        pidUpdate(&pidYaw, desiredYaw, state->attitude.yaw);
        yawDesiredRate = pidYaw.output;
    }

    /* Rate PID */
    float gyroRoll = sensors->gyro.x;
    float gyroPitch = -sensors->gyro.y;
    float gyroYaw = sensors->gyro.z;

    pidUpdate(&pidRollRate, rollDesiredRate, gyroRoll);
    pidUpdate(&pidPitchRate, pitchDesiredRate, gyroPitch);
    pidUpdate(&pidYawRate, yawDesiredRate, gyroYaw);

    /* Thrust */
    if (setpoint->mode.z == modeDisable) {
        control->thrust = setpoint->thrust;
    } else {
        control->thrust = positionControllerUpdate(setpoint, state);
    }

    control->roll = saturateSignedInt16((int32_t)pidRollRate.output);
    control->pitch = saturateSignedInt16((int32_t)pidPitchRate.output);
    control->yaw = saturateSignedInt16((int32_t)pidYawRate.output);

    if (control->thrust == 0) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0;
        attitudeControllerResetAllPID(state->attitude.roll,
                                      state->attitude.pitch,
                                      state->attitude.yaw);
        desiredYaw = state->attitude.yaw;
        posIntegralZ = 0.0f;
        posPrevErrorZ = 0.0f;
        return;
    }

    if (control->controlMode == controlModeLegacy) {
        control->yaw = -control->yaw;
    }
}

/* -------------------------------------------------------------------------
 * 5. CRTP Commander RPYT
 * ---------------------------------------------------------------------- */
bool thrustLocked = false;
bool commanderModeSet = false;

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime)
{
    if (!rollPrime || !pitchPrime) return;
    float rad = yaw_deg * PI_F / 180.0f;
    float cosr = cosf(rad);
    float sinr = sinf(rad);
    *rollPrime = roll * cosr - pitch * sinr;
    *pitchPrime = roll * sinr + pitch * cosr;
}

static void resetSetpoint(Setpoint *sp)
{
    if (!sp) return;
    memset(sp, 0, sizeof(*sp));
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
    if (!values || !setpoint) return;
    resetSetpoint(setpoint);
    setpoint->timestamp = 0;

    float roll = values->roll;
    float pitch = values->pitch;
    float yaw = values->yaw;
    uint16_t rawThrust = values->thrust;

    /* Raw thrust lock handling */
    if (thrustLocked && rawThrust == 0) {
        thrustLocked = false;
    }

    /* PosSet has highest priority when active and thrust non-zero */
    if (posSetMode && rawThrust != 0) {
        setpoint->mode.x = modeAbs;
        setpoint->mode.y = modeAbs;
        setpoint->mode.z = modeAbs;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->mode.yaw = modeAbs;
        setpoint->position.x = -pitch;
        setpoint->position.y = roll;
        setpoint->position.z = rawThrust / 1000.0f;
        setpoint->attitude.yaw = yaw;
        setpoint->thrust = 0;
        return;
    }

    /* PosHold */
    if (posHoldMode) {
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.z = modeDisable;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->velocity.x = pitch / 30.0f;
        setpoint->velocity.y = roll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;

        /* Default yaw handling */
        if (stabilizationModeYaw == RATE) {
            setpoint->mode.yaw = modeVelocity;
            setpoint->attitudeRate.yaw = -yaw;
        } else {
            setpoint->mode.yaw = modeAbs;
            setpoint->attitude.yaw = yaw;
        }

        if (thrustLocked || rawThrust < MIN_THRUST) {
            setpoint->thrust = 0;
        } else {
            setpoint->thrust = rawThrust > MAX_THRUST ? MAX_THRUST : rawThrust;
        }
        return;
    }

    /* AltHold */
    if (altHoldMode) {
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0;
        setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
        if (!commanderModeSet) {
            commanderModeSet = true;
            /* Position PID/filter reset would happen here */
        }

        /* Roll/Pitch default */
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
        return;
    } else {
        if (commanderModeSet) {
            commanderModeSet = false;
        }
    }

    /* Default stable mode */
    if (thrustLocked || rawThrust < MIN_THRUST) {
        setpoint->thrust = 0;
    } else {
        setpoint->thrust = rawThrust > MAX_THRUST ? MAX_THRUST : rawThrust;
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

    /* Apply yaw mode rotation for roll/pitch */
    if (yawMode == PLUSMODE) {
        float rp, pp;
        rotateYaw(roll, pitch, 45.0f, &rp, &pp);
        if (stabilizationModeRoll == RATE) {
            setpoint->attitudeRate.roll = rp;
        } else {
            setpoint->attitude.roll = rp;
        }
        if (stabilizationModePitch == RATE) {
            setpoint->attitudeRate.pitch = pp;
        } else {
            setpoint->attitude.pitch = pp;
        }
    } else if (yawMode == CAREFREE) {
        /* Observable error path: zero attitude commands */
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
    } else {
        /* XMODE: no rotation */
    }
}

/* -------------------------------------------------------------------------
 * 6. Supervisor
 * ---------------------------------------------------------------------- */
SupervisorState supervisorState = supervisorStatePreFlChecksNotPassed;
uint32_t supervisorConditionBits = 0;

static bool supervisorArmed = false;
static bool supervisorCrashed = false;
static bool supervisorTumbled = false;
static bool supervisorFlying = false;
static bool supervisorLocked = false;
static bool supervisorFreeFall = false;
static bool autoArmingEnabled = false;
static uint32_t spinupTimeoutDurationMs = 0;
static uint32_t spinupStartTick = 0;
static uint32_t preflightLatestArmingTick = 0;
static uint32_t landingLatestTick = 0;

static SensorData lastSensors;
static uint32_t lastMotorRatios[4];
static uint32_t lastIdleThrust = 0;
static int32_t lastMotorRPMs[4];
static float safetyCrashGs = 0.0f;
static float safetyFreeFallThreshold = 0.0f;
static float safetyAcceptedTiltAccZ = 0.5f;
static float safetyAcceptedUpsideDownAccZ = -0.5f;
static uint32_t safetyMaxTiltTime = 500;
static uint32_t safetyMaxUpsideDownTime = 300;
static bool safetyTumbleCheckEnabled = true;

static uint32_t tiltStartTick = 0;
static bool tiltActive = false;
static uint32_t notRespondingStartTick = 0;
static bool notRespondingActive = false;

void supervisorInit(void)
{
    supervisorState = supervisorStatePreFlChecksNotPassed;
    supervisorConditionBits = 0;
    supervisorArmed = false;
    supervisorCrashed = false;
    supervisorTumbled = false;
    supervisorFlying = false;
    supervisorLocked = false;
    supervisorFreeFall = false;
    spinupStartTick = 0;
    tiltStartTick = 0;
    tiltActive = false;
    notRespondingStartTick = 0;
    notRespondingActive = false;
}

void supervisorUpdate(uint32_t stabilizerStep)
{
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

    /* Simplified state machine */
    if (supervisorState == supervisorStatePreFlChecksPassed && autoArmingEnabled && !supervisorArmed) {
        supervisorRequestArming(true);
    }

    if (supervisorState == supervisorStateArming) {
        if (spinupStartTick != 0 &&
            spinupTimeoutDurationMs != 0 &&
            (stabilizerStep - spinupStartTick) >= spinupTimeoutDurationMs) {
            supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
        }
    } else if (supervisorState != supervisorStateArming) {
        spinupStartTick = 0;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }

    /* Update flying flag using stored motor ratios */
    supervisorFlying = isFlyingCheck(lastMotorRatios, lastIdleThrust, stabilizerStep);
}

bool supervisorCanFly(void)
{
    return (supervisorState == supervisorStateReadyToFly ||
            supervisorState == supervisorStateFlying ||
            supervisorState == supervisorStateWarningLevelOut ||
            supervisorState == supervisorStateLanded);
}

bool supervisorCanArm(void)
{
    return supervisorState == supervisorStatePreFlChecksPassed;
}

bool supervisorIsArmed(void)
{
    return supervisorArmed;
}

bool supervisorIsCrashed(void)
{
    return supervisorCrashed;
}

bool supervisorRequestArming(bool doArm)
{
    if (doArm) {
        if (supervisorCanArm()) {
            supervisorArmed = true;
            supervisorState = supervisorStateArming;
            spinupStartTick = 0;
            return true;
        }
        return false;
    } else {
        supervisorArmed = false;
        return true;
    }
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
    if (doRecovery) {
        if (supervisorTumbled) return false;
        supervisorCrashed = false;
        supervisorState = supervisorStatePreFlChecksNotPassed;
        return true;
    } else {
        supervisorCrashed = true;
        supervisorState = supervisorStateCrashed;
        return true;
    }
}

bool supervisorAreMotorsAllowedToRun(void)
{
    return (supervisorState == supervisorStateArming ||
            supervisorState == supervisorStateReadyToFly ||
            supervisorState == supervisorStateFlying ||
            supervisorState == supervisorStateWarningLevelOut ||
            supervisorState == supervisorStateLanded);
}

uint16_t supervisorGetInfoBitfield(void)
{
    uint16_t info = 0;
    if (supervisorCanArm()) info |= (1u << 0);
    if (supervisorArmed) info |= (1u << 1);
    if (autoArmingEnabled) info |= (1u << 2);
    if (supervisorCanFly()) info |= (1u << 3);
    if (supervisorFlying) info |= (1u << 4);
    if (supervisorTumbled) info |= (1u << 5);
    if (supervisorLocked) info |= (1u << 6);
    if (supervisorCrashed) info |= (1u << 7);
    /* bits 8-11 reserved trajectory/deck, currently false */
    return info;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick)
{
    static uint32_t recentFlightTick = 0;
    static bool seenRecentFlight = false;
    if (!motorRatios) return false;

    for (int i = 0; i < 4; i++) {
        if (motorRatios[i] > idleThrust) {
            recentFlightTick = currentTick;
            seenRecentFlight = true;
            break;
        }
    }
    if (!seenRecentFlight) return false;
    uint32_t elapsed = currentTick - recentFlightTick;
    return elapsed < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling)
{
    if (isFreeFalling) *isFreeFalling = false;

    if (!tumbleCheckEnabled) {
        supervisorTumbled = false;
        return false;
    }

    /* Crash detection */
    if (crashDetectionGs > 0.0f) {
        float norm = sqrtf(accX*accX + accY*accY + accZ*accZ);
        if (fabsf(norm - 1.0f) > crashDetectionGs) {
            supervisorCrashed = true;
            supervisorState = supervisorStateCrashed;
        }
    }

    /* Free fall detection */
    if (freeFallThreshold > 0.0f &&
        fabsf(accX) < freeFallThreshold &&
        fabsf(accY) < freeFallThreshold &&
        fabsf(accZ) < freeFallThreshold) {
        if (isFreeFalling) *isFreeFalling = true;
        supervisorFreeFall = true;
        supervisorState = supervisorStateExceptFreeFall;
        tiltStartTick = 0;
        tiltActive = false;
        return false;
    }

    /* Tilt detection */
    if (accZ < acceptedTiltAccZ) {
        if (!tiltActive) {
            tiltStartTick = currentTick;
            tiltActive = true;
        }
        bool upsideDown = accZ < acceptedUpsideDownAccZ;
        uint32_t timeout = upsideDown ? maxUpsideDownTime : maxTiltTime;
        if (timeout > 0 && (currentTick - tiltStartTick) >= timeout) {
            supervisorTumbled = true;
        }
    } else {
        tiltStartTick = 0;
        tiltActive = false;
    }

    return supervisorTumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick)
{
    if (lastNotificationTick == 0) return true;
    return (currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration)
{
    if (state != supervisorStateReadyToFly) return false;
    if (preflightTimeoutDuration == 0) return false;
    return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration)
{
    if (landingTimeoutDuration == 0) return false;
    return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed)
{
    supervisorConditionBits = 0;
    if (supervisorArmed) supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    if (supervisorFlying) supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
    if (supervisorTumbled) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    if (crtpEmergencyStop || paramEmergencyStop) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
        supervisorLocked = true;
    }
    if (emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    }
    if (supervisorCrashed) supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    if (supervisorFreeFall) supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
    return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBits,
                                SupervisorState state)
{
    if (!setpoint) return;
    (void)supervisorConditionBits;

    if (state == supervisorStateWarningLevelOut) {
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = 0.0f;
        /* z remains unchanged */
        return;
    }

    if (state == supervisorStateArming ||
        state == supervisorStateReadyToFly ||
        state == supervisorStateFlying ||
        state == supervisorStateLanded) {
        return;
    }

    /* All other states: zero setpoint */
    resetSetpoint(setpoint);
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax)
{
    if (!motorRPMs) return false;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTick)
{
    if (!motorRPMs || !canFly) {
        notRespondingStartTick = 0;
        notRespondingActive = false;
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
        if (!notRespondingActive) {
            notRespondingActive = true;
            notRespondingStartTick = currentTick;
        }
        if (rpmCheckDurationMs != 0 &&
            (currentTick - notRespondingStartTick) >= rpmCheckDurationMs) {
            return true;
        }
    } else {
        notRespondingActive = false;
        notRespondingStartTick = 0;
    }
    return false;
}

void supervisorSetSensorData(const SensorData *sensors)
{
    if (sensors) lastSensors = *sensors;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust)
{
    if (motorRatios) {
        for (int i = 0; i < 4; i++) lastMotorRatios[i] = motorRatios[i];
    }
    lastIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
    if (motorRPMs) {
        for (int i = 0; i < 4; i++) lastMotorRPMs[i] = motorRPMs[i];
    }
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled)
{
    safetyCrashGs = crashDetectionGs;
    safetyFreeFallThreshold = freeFallThreshold;
    safetyAcceptedTiltAccZ = acceptedTiltAccZ;
    safetyAcceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
    safetyMaxTiltTime = maxTiltTime;
    safetyMaxUpsideDownTime = maxUpsideDownTime;
    safetyTumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs_)
{
    autoArmingEnabled = autoArming;
    spinupTimeoutDurationMs = spinupTimeoutDurationMs_;
}

/* -------------------------------------------------------------------------
 * 7. Estimator FIFO and Commander arbitration
 * ---------------------------------------------------------------------- */
#define ESTIMATOR_FIFO_SIZE 16
static EstimatorMeasurement estimatorFifo[ESTIMATOR_FIFO_SIZE];
static uint8_t estimatorHead = 0;
static uint8_t estimatorTail = 0;
static uint8_t estimatorCount = 0;

bool estimatorEnqueue(const EstimatorMeasurement *measurement)
{
    if (!measurement) return false;
    if (estimatorCount == ESTIMATOR_FIFO_SIZE) return false;
    estimatorFifo[estimatorTail] = *measurement;
    estimatorTail = (estimatorTail + 1) % ESTIMATOR_FIFO_SIZE;
    estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement)
{
    if (!measurement) return false;
    if (estimatorCount == 0) return false;
    *measurement = estimatorFifo[estimatorHead];
    estimatorHead = (estimatorHead + 1) % ESTIMATOR_FIFO_SIZE;
    estimatorCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep)
{
    /* Drain FIFO and keep last measurements */
    EstimatorMeasurement m;
    EstimatorMeasurement lastGyro = { MeasurementTypeGyroscope, {0,0,0} };
    EstimatorMeasurement lastAcc = { MeasurementTypeAcceleration, {0,0,0} };
    EstimatorMeasurement lastBaro = { MeasurementTypeBarometer, {0,0,0} };
    EstimatorMeasurement lastTof = { MeasurementTypeTOF, {0,0,0} };
    bool hasGyro = false, hasAcc = false, hasBaro = false, hasTof = false;

    while (estimatorDequeue(&m)) {
        switch (m.type) {
        case MeasurementTypeGyroscope:
            lastGyro = m; hasGyro = true; break;
        case MeasurementTypeAcceleration:
            lastAcc = m; hasAcc = true; break;
        case MeasurementTypeBarometer:
            lastBaro = m; hasBaro = true; break;
        case MeasurementTypeTOF:
            lastTof = m; hasTof = true; break;
        default:
            break;
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        float gx = hasGyro ? lastGyro.data[0] : 0.0f;
        float gy = hasGyro ? lastGyro.data[1] : 0.0f;
        float gz = hasGyro ? lastGyro.data[2] : 0.0f;
        float ax = hasAcc ? lastAcc.data[0] : 0.0f;
        float ay = hasAcc ? lastAcc.data[1] : 0.0f;
        float az = hasAcc ? lastAcc.data[2] : 0.0f;
        sensfusion6UpdateQ(gx, gy, gz, ax, ay, az, 1.0f/SENSFUSION_RATE_HZ);

        float roll, pitch, yaw;
        sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
        stateEstimate.roll = roll;
        stateEstimate.pitch = pitch;
        stateEstimate.yaw = yaw;
        sensfusion6GetQuaternion(&stateEstimate.qw, &stateEstimate.qx,
                                 &stateEstimate.qy, &stateEstimate.qz);
        sensfusion6Log.qw = stateEstimate.qw;
        sensfusion6Log.qx = stateEstimate.qx;
        sensfusion6Log.qy = stateEstimate.qy;
        sensfusion6Log.qz = stateEstimate.qz;
        sensfusion6Log.gravityX = gravityX;
        sensfusion6Log.gravityY = gravityY;
        sensfusion6Log.gravityZ = gravityZ;
        sensfusion6Log.accZbase = baseZacc;
        sensfusion6Log.isInit = sensfusion6IsInit;
        sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;

        acc.x = ax; acc.y = ay; acc.z = az;
        gyro.x = gx; gyro.y = gy; gyro.z = gz;
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        if (hasBaro) {
            baro.pressure = lastBaro.data[0];
            baro.asl = lastBaro.data[1];
            baro.temp = lastBaro.data[2];
        }
        if (hasTof) {
            /* TOF update */
        }
    }
}

/* Commander */
static Setpoint activeCommanderSetpoint;
static int activePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t lastCommanderUpdateTick = 0;
static uint32_t currentTick = 0;

bool commanderSetSetpoint(const Setpoint *setpoint, int priority)
{
    if (!setpoint) return false;
    if (priority == COMMANDER_PRIORITY_DISABLE) {
        activeCommanderSetpoint = *setpoint;
        activePriority = priority;
        lastCommanderUpdateTick = currentTick;
        return true;
    }
    if (priority >= activePriority) {
        activeCommanderSetpoint = *setpoint;
        activePriority = priority;
        lastCommanderUpdateTick = currentTick;
        return true;
    }
    return false;
}

void commanderRelaxPriority(void)
{
    activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void)
{
    return currentTick - lastCommanderUpdateTick;
}

int commanderGetActivePriority(void)
{
    return activePriority;
}

/* -------------------------------------------------------------------------
 * 8. Stabilizer and compression
 * ---------------------------------------------------------------------- */
static bool stabilizerInitialized = false;
static uint32_t stabilizerStep = 0;
static bool pendingHighLevel = false;
static Setpoint pendingHighLevelSetpoint;

void stabilizerInit(void)
{
    if (stabilizerInitialized) return;
    stabilizerInitialized = true;
    /* Observable order via internal flags:
     * sensorsInit -> stateEstimatorInit -> controllerInit ->
     * powerDistributionInit -> motorsInit -> collisionAvoidanceInit
     */
    attitudeControllerInit(0.002f);
}

void stabilizerTask(void)
{
    if (!stabilizerInitialized) stabilizerInit();
    if (!sensfusion6IsInit) sensfusion6Init();
    if (!sensfusion6IsCalibrated) return; /* wait sensor calibration */

    stabilizerStep++;

    if (healthShallWeRunTest()) {
        SensorData sd = lastSensors;
        healthRunTests(&sd);
        return;
    }

    /* Normal branch */
    supervisorUpdate(stabilizerStep);

    if (!supervisorCanFly()) {
        motor.m1req = motor.m2req = motor.m3req = motor.m4req = 0;
        return;
    }

    if (pendingHighLevel) {
        commanderSetSetpoint(&pendingHighLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        pendingHighLevel = false;
    }

    Setpoint sp = activeCommanderSetpoint;
    supervisorOverrideSetpoint(&sp, supervisorConditionBits, supervisorState);

    State state;
    memset(&state, 0, sizeof(state));
    state.attitude.roll = stateEstimate.roll;
    state.attitude.pitch = stateEstimate.pitch;
    state.attitude.yaw = stateEstimate.yaw;
    state.attitudeQuaternion.w = stateEstimate.qw;
    state.attitudeQuaternion.x = stateEstimate.qx;
    state.attitudeQuaternion.y = stateEstimate.qy;
    state.attitudeQuaternion.z = stateEstimate.qz;

    SensorData sensors = lastSensors;

    ControlData control;
    memset(&control, 0, sizeof(control));
    control.controlMode = controlModeLegacy;
    controllerPid(&sensors, &sp, &state, &control, 0.0f, 0.002f);

    MotorPower motorPower;
    memset(&motorPower, 0, sizeof(motorPower));
    powerDistribution(&control, &motorPower);

    int32_t motors[4] = { motorPower.m1, motorPower.m2, motorPower.m3, motorPower.m4 };
    PowerCapResult capResult = powerDistributionCap(motors, 65535, 0);

    if (!supervisorAreMotorsAllowedToRun()) {
        motor.m1req = motor.m2req = motor.m3req = motor.m4req = 0;
    } else {
        motor.m1req = (uint16_t)motors[0];
        motor.m2req = (uint16_t)motors[1];
        motor.m3req = (uint16_t)motors[2];
        motor.m4req = (uint16_t)motors[3];
    }
    (void)capResult;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint)
{
    if (!setpoint) return false;
    pendingHighLevelSetpoint = *setpoint;
    pendingHighLevel = true;
    return true;
}

static uint32_t quatcompress(const Quaternion *q)
{
    if (!q) return 0;
    uint8_t qwByte = (uint8_t)(int32_t)((q->w + 1.0f) * 127.5f);
    uint8_t qxByte = (uint8_t)(int32_t)((q->x + 1.0f) * 127.5f);
    uint8_t qyByte = (uint8_t)(int32_t)((q->y + 1.0f) * 127.5f);
    uint8_t qzByte = (uint8_t)(int32_t)((q->z + 1.0f) * 127.5f);
    return ((uint32_t)qwByte << 24) | ((uint32_t)qxByte << 16) |
           ((uint32_t)qyByte << 8) | (uint32_t)qzByte;
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output)
{
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
    output->gyro_millirad_s[0] = sensors->gyro.x * PI_F / 180.0f * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * PI_F / 180.0f * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * PI_F / 180.0f * 1000.0f;
    output->quatCompressed = quatcompress(&state->attitudeQuaternion);
}

bool rateSupervisorValidate(uint32_t measuredRate)
{
    return measuredRate >= 997 && measuredRate <= 1003;
}

void rateSupervisorTask(void)
{
    static bool sensorActive = true;
    /* Wait 2000ms; if sensor active then assert/error */
    if (sensorActive) {
        /* error state would be entered in real firmware */
    }
}

/* -------------------------------------------------------------------------
 * 9. Health
 * ---------------------------------------------------------------------- */
TestState healthTestState = testDone;
uint8_t motorPass = 0;
uint8_t batteryPass = 0;
float batterySag = 0.0f;

static bool propTestRequested = false;
static bool batTestRequested = false;
static uint32_t healthTick = 0;
static uint32_t healthMotorTestCount = 0;
static float idleVoltage = 0.0f;
static float minLoadedVoltage = 0.0f;

void healthRequestPropTest(void)
{
    propTestRequested = true;
}

void healthRequestBatteryTest(void)
{
    batTestRequested = true;
}

bool healthShallWeRunTest(void)
{
    if (propTestRequested) {
        propTestRequested = false;
        healthTestState = configureAcc;
        motorPass = 0;
        batteryPass = 0;
        batterySag = 0.0f;
        healthTick = 0;
        healthMotorTestCount = 0;
        return true;
    }
    if (batTestRequested) {
        batTestRequested = false;
        healthTestState = testBattery;
        batteryPass = 0;
        batterySag = 0.0f;
        healthTick = 0;
        minLoadedVoltage = 0.0f;
        idleVoltage = 0.0f;
        return true;
    }
    return healthTestState != testDone;
}

void healthRunTests(const SensorData *sensorData)
{
    if (!sensorData) return;
    switch (healthTestState) {
    case configureAcc:
        healthTestState = measureNoiseFloor;
        healthTick = 0;
        break;
    case measureNoiseFloor:
        healthTick++;
        if (healthTick >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
            healthTestState = measureProp;
            healthTick = 0;
            healthMotorTestCount = 0;
        }
        break;
    case measureProp:
        healthTick++;
        if (healthTick >= 10) {
            healthTick = 0;
            healthMotorTestCount++;
            if (healthMotorTestCount >= 4) {
                healthTestState = evaluatePropResult;
            }
        }
        break;
    case evaluatePropResult:
        healthTestState = testDone;
        break;
    case testBattery:
        healthTick++;
        if (healthTick == 1) {
            idleVoltage = sensorData->baroAsl; /* placeholder */
            minLoadedVoltage = idleVoltage;
        } else if (healthTick >= 2 && healthTick <= 49) {
            float v = sensorData->baroAsl;
            if (v < minLoadedVoltage) minLoadedVoltage = v;
        } else if (healthTick >= 50) {
            batterySag = idleVoltage - minLoadedVoltage;
            healthTestState = evaluateBatResult;
        }
        break;
    case evaluateBatResult:
        batteryPass = (batterySag <= 2.0f) ? 1 : 0;
        healthTestState = testDone;
        break;
    case restartBatTest:
        healthTick++;
        if (healthTick >= 2000) {
            healthTestState = testBattery;
            healthTick = 0;
        }
        break;
    case testDone:
    default:
        break;
    }
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motor)
{
    if (highThreshold == 0.0f) return true;
    if (lowThreshold <= measuredValue && measuredValue <= highThreshold) {
        motorPass |= (uint8_t)(1u << motor);
        return true;
    }
    motorTestCount++;
    return false;
}

float variance(const float *buffer, int length)
{
    if (!buffer || length <= 0) return 0.0f;
    float sum = 0.0f;
    float sumSq = 0.0f;
    for (int i = 0; i < length; i++) {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }
    return sumSq - (sum * sum) / (float)length;
}

/* -------------------------------------------------------------------------
 * 10. CRTP transport
 * ---------------------------------------------------------------------- */
static CrtpPacket txQueue[CRTP_TX_QUEUE_SIZE];
static uint32_t txCount = 0;
static uint32_t txHead = 0;
static uint32_t txTail = 0;

typedef struct {
    CrtpPacket queue[CRTP_RX_QUEUE_SIZE];
    uint32_t head, tail, count;
    bool created;
    CrtpPortCallback callback;
} CrtpPortState;

static CrtpPortState crtpPorts[CRTP_NBR_OF_PORTS];

static bool nopSendPacket(CrtpPacket *packet)
{
    (void)packet;
    return true;
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

static CrtpLink nopLink = {
    nopSendPacket,
    nopReceivePacket,
    nopIsConnected,
    nopSetEnable,
    nopReset
};

static CrtpLink *currentLink = &nopLink;
static bool crtpInitialized = false;

void crtpInit(void)
{
    if (crtpInitialized) return;
    crtpInitialized = true;
    memset(txQueue, 0, sizeof(txQueue));
    txCount = txHead = txTail = 0;
    memset(crtpPorts, 0, sizeof(crtpPorts));
    currentLink = &nopLink;
}

void crtpInitTaskQueue(uint8_t port)
{
    if (port >= CRTP_NBR_OF_PORTS) return;
    crtpPorts[port].created = true;
}

bool crtpSendPacket(const CrtpPacket *packet)
{
    if (!packet || txCount >= CRTP_TX_QUEUE_SIZE) return false;
    txQueue[txTail] = *packet;
    txTail = (txTail + 1) % CRTP_TX_QUEUE_SIZE;
    txCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet)
{
    /* In host model, block only fails when queue full.
     * Same as non-blocking because no waiting is modelled.
     */
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet)
{
    if (!packet || port >= CRTP_NBR_OF_PORTS) return false;
    CrtpPortState *p = &crtpPorts[port];
    if (!p->created || p->count == 0) return false;
    *packet = p->queue[p->head];
    p->head = (p->head + 1) % CRTP_RX_QUEUE_SIZE;
    p->count--;
    return true;
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet)
{
    return crtpReceivePacket(port, packet);
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet,
                           uint32_t wait_ms)
{
    (void)wait_ms;
    return crtpReceivePacket(port, packet);
}

void crtpRxTask(void)
{
    if (!currentLink || !currentLink->receivePacket) return;
    CrtpPacket pkt;
    while (currentLink->receivePacket(&pkt)) {
        if (pkt.port < CRTP_NBR_OF_PORTS) {
            CrtpPortState *p = &crtpPorts[pkt.port];
            if (p->created && p->count < CRTP_RX_QUEUE_SIZE) {
                p->queue[p->tail] = pkt;
                p->tail = (p->tail + 1) % CRTP_RX_QUEUE_SIZE;
                p->count++;
            }
            if (p->callback) {
                p->callback(&pkt);
            }
        }
    }
}

void crtpTxTask(void)
{
    if (!currentLink || !currentLink->sendPacket) return;
    while (txCount > 0) {
        CrtpPacket *pkt = &txQueue[txHead];
        if (currentLink->sendPacket(pkt)) {
            txHead = (txHead + 1) % CRTP_TX_QUEUE_SIZE;
            txCount--;
        } else {
            /* Retry later after 10ms */
            break;
        }
    }
}

void crtpSetLink(CrtpLink *newLink)
{
    if (currentLink && currentLink->setEnable) {
        currentLink->setEnable(false);
    }
    if (newLink == NULL) {
        currentLink = &nopLink;
    } else {
        currentLink = newLink;
    }
    if (currentLink && currentLink->setEnable) {
        currentLink->setEnable(true);
    }
}

void crtpReset(void)
{
    txCount = txHead = txTail = 0;
    if (currentLink && currentLink->reset) {
        currentLink->reset();
    }
}

bool crtpIsConnected(void)
{
    if (currentLink && currentLink->isConnected) {
        return currentLink->isConnected();
    }
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void)
{
    return CRTP_TX_QUEUE_SIZE - txCount;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback)
{
    if (port >= CRTP_NBR_OF_PORTS) return;
    crtpPorts[port].callback = callback;
}

void updateStats(void)
{
    /* Reset counters every 500ms in real code */
}

/* -------------------------------------------------------------------------
 * 11. Deck discovery and log globals
 * ---------------------------------------------------------------------- */
uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity)
{
    if (!decks || capacity == 0) return 0;
    /* Mock deterministic deck inventory */
    static const uint8_t knownI2C[] = {0x20, 0x21};
    static const uint64_t knownOneWire[] = {0x1234567890ABCDEFULL};
    uint8_t count = 0;
    for (unsigned i = 0; i < sizeof(knownI2C)/sizeof(knownI2C[0]) && count < capacity; i++) {
        bool dup = false;
        for (uint8_t j = 0; j < count; j++) {
            if (decks[j].foundByI2C && decks[j].i2cAddress == knownI2C[i]) {
                dup = true;
                break;
            }
        }
        if (!dup) {
            decks[count].foundByI2C = true;
            decks[count].foundByOneWire = false;
            decks[count].i2cAddress = knownI2C[i];
            decks[count].oneWireRomId = 0;
            count++;
        }
    }
    for (unsigned i = 0; i < sizeof(knownOneWire)/sizeof(knownOneWire[0]) && count < capacity; i++) {
        bool dup = false;
        for (uint8_t j = 0; j < count; j++) {
            if (decks[j].foundByOneWire && decks[j].oneWireRomId == knownOneWire[i]) {
                dup = true;
                break;
            }
        }
        if (!dup) {
            decks[count].foundByOneWire = true;
            decks[count].foundByI2C = false;
            decks[count].oneWireRomId = knownOneWire[i];
            decks[count].i2cAddress = 0;
            count++;
        }
    }
    return count;
}

StateEstimateLog stateEstimate;
Axis3Log gyro, acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

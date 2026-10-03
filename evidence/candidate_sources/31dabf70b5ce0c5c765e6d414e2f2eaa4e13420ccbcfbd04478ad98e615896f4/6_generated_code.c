#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define PI_F 3.14159265358979323846f
#define DEG_TO_RAD_F (PI_F / 180.0f)
#define RAD_TO_DEG_F (180.0f / PI_F)

#define DEFAULT_PREFLIGHT_TIMEOUT_MS 5000U
#define DEFAULT_LANDING_TIMEOUT_MS 5000U
#define DEFAULT_BATTERY_SAG_THRESHOLD 0.5f

uint32_t hostTickMs = 0U;
uint16_t batteryVoltage = 4200U;
float filteredBatteryVoltage = 4.2f;
float batterySagThreshold = DEFAULT_BATTERY_SAG_THRESHOLD;

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

PidObject pidRoll = {0};
PidObject pidPitch = {0};
PidObject pidYaw = {0};
PidObject pidRollRate = {0};
PidObject pidPitchRate = {0};
PidObject pidYawRate = {0};

bool thrustLocked = false;
bool commanderModeSet = false;

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0U;

StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0};
Axis3Log acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};

TestState healthTestState = testDone;
uint8_t motorPass = 0U;
uint8_t batteryPass = 0U;
float batterySag = 0.0f;

int16_t saturateSignedInt16(int32_t value)
{
    if (value > INT16_MAX) return INT16_MAX;
    if (value < -INT16_MAX) return (int16_t)(-INT16_MAX);
    return (int16_t)value;
}

float capAngle(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

float invSqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    union { float f; int32_t i; } conv;
    conv.f = x;
    float xhalf = 0.5f * x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    float y = conv.f;
    y = y * (1.5f - xhalf * y * y);
    return y;
}

static float clamp1(float v)
{
    if (v > 1.0f) return 1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

void sensfusion6Init(void)
{
    if (sensfusion6IsInit) return;
    qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
    gravityX = 0.0f; gravityY = 0.0f; gravityZ = 1.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
    baseZacc = 0.0f;
    sensfusion6IsCalibrated = false;
    sensfusion6IsInit = true;
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
    *gravY = 2.0f * (qw_ * qx_ + qy_ * qz_);
    *gravZ = qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_;
}

static void sensfusion6ComputeGravity(void)
{
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
    if (dt <= 0.0f) return;

    bool accValid = !(ax == 0.0f && ay == 0.0f && az == 0.0f);
    float gxr = gx * DEG_TO_RAD_F;
    float gyr = gy * DEG_TO_RAD_F;
    float gzr = gz * DEG_TO_RAD_F;
    float q0 = qw, q1 = qx, q2 = qy, q3 = qz;

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
    if (accValid) {
        float norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm > 1e-6f) {
            ax /= norm; ay /= norm; az /= norm;
            float _2q0 = 2.0f * q0;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float s0 = -_2q2 * ax + _2q1 * ay;
            float s1 = _2q3 * ax + _2q0 * ay - 4.0f * q1 * az;
            float s2 = -_2q0 * ax + _2q3 * ay - 4.0f * q2 * az;
            float s3 = _2q1 * ax + _2q2 * ay;
            float snorm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            if (snorm > 1e-6f) {
                gxr -= beta * (s0 / snorm);
                gyr -= beta * (s1 / snorm);
                gzr -= beta * (s2 / snorm);
            }
        }
    }
#else
    if (accValid) {
        float norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm > 1e-6f) {
            ax /= norm; ay /= norm; az /= norm;
            float vx = 2.0f * (q1 * q3 - q0 * q2);
            float vy = 2.0f * (q0 * q1 + q2 * q3);
            float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
            float ex = ay * vz - az * vy;
            float ey = az * vx - ax * vz;
            float ez = ax * vy - ay * vx;

            if (twoKi > 0.0f) {
                integralFBx += twoKi * ex * dt;
                integralFBy += twoKi * ey * dt;
                integralFBz += twoKi * ez * dt;
            } else {
                integralFBx = integralFBy = integralFBz = 0.0f;
            }

            gxr += twoKp * ex + integralFBx;
            gyr += twoKp * ey + integralFBy;
            gzr += twoKp * ez + integralFBz;
        }
    } else {
        if (twoKi <= 0.0f) {
            integralFBx = integralFBy = integralFBz = 0.0f;
        }
    }
#endif

    float halfvx = gxr * 0.5f;
    float halfvy = gyr * 0.5f;
    float halfvz = gzr * 0.5f;
    float qa = q0, qb = q1, qc = q2, qd = q3;
    q0 += (-qb * halfvx - qc * halfvy - qd * halfvz) * dt;
    q1 += ( qa * halfvx + qc * halfvz - qd * halfvy) * dt;
    q2 += ( qa * halfvy - qb * halfvz + qd * halfvx) * dt;
    q3 += ( qa * halfvz + qb * halfvy - qc * halfvx) * dt;

    float normq = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (normq > 1e-6f) {
        q0 /= normq; q1 /= normq; q2 /= normq; q3 /= normq;
    }

    qw = q0; qx = q1; qy = q2; qz = q3;
    sensfusion6ComputeGravity();

    if (accValid && !sensfusion6IsCalibrated) {
        baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
        sensfusion6IsCalibrated = true;
    }

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

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (!roll_deg || !pitch_deg || !yaw_deg) return;
    float q0 = qw, q1 = qx, q2 = qy, q3 = qz;
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    sinp = clamp1(sinp);
    *roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3),
                       1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG_F;
    *pitch_deg = asinf(sinp) * RAD_TO_DEG_F;
    *yaw_deg = atan2f(2.0f * (q0 * q3 + q1 * q2),
                      1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG_F;
}

void sensfusion6GetQuaternion(float *qwOut, float *qxOut, float *qyOut, float *qzOut)
{
    if (!qwOut || !qxOut || !qyOut || !qzOut) return;
    *qwOut = qw; *qxOut = qx; *qyOut = qy; *qzOut = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az)
{
    float gx, gy_, gz;
    estimatedGravityDirection(qw, qx, qy, qz, &gx, &gy_, &gz);
    return ax * gx + ay * gy_ + az * gz;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az)
{
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out)
{
    if (!out) return;
    int32_t r = (int32_t)roll / 2;
    int32_t p = (int32_t)pitch / 2;
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
    float rollPart = (armLength != 0.0f) ? (0.25f / arm) * torqueX : 0.0f;
    float pitchPart = (armLength != 0.0f) ? (0.25f / arm) * torqueY : 0.0f;
    float yawPart = (thrustToTorque != 0.0f) ? (0.25f / thrustToTorque) * torqueZ : 0.0f;

    float f[4];
    f[0] = thrustPart - rollPart + pitchPart + yawPart;
    f[1] = thrustPart - rollPart - pitchPart - yawPart;
    f[2] = thrustPart + rollPart - pitchPart + yawPart;
    f[3] = thrustPart + rollPart + pitchPart - yawPart;
    for (int i = 0; i < 4; ++i) {
        if (f[i] < 0.0f) f[i] = 0.0f;
        motorForces[i] = f[i];
    }
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4])
{
    if (!normalizedForces || !motorPWMs) return;
    for (int i = 0; i < 4; ++i) {
        float v = normalizedForces[i];
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        motorPWMs[i] = (uint16_t)(v * 65535.0f);
    }
}

static int32_t motorForceToPwmValue(float force)
{
    if (force <= 0.0f) return 0;
    if (CRAZYFLIE_MAX_MOTOR_FORCE_N <= 0.0f) return 0;
    float scaled = (force / CRAZYFLIE_MAX_MOTOR_FORCE_N) * 65535.0f;
    if (scaled < 0.0f) scaled = 0.0f;
    if (scaled > 65535.0f) scaled = 65535.0f;
    return (int32_t)(scaled + 0.5f);
}

void powerDistribution(const ControlData *control, MotorPower *motorPower)
{
    if (!control || !motorPower) return;

    switch (control->controlMode) {
    case controlModeLegacy:
        powerDistributionLegacy(control->thrust, control->roll, control->pitch,
                                control->yaw, motorPower);
        break;
    case controlModeForceTorque: {
        float forces[4] = {0};
        powerDistributionForceTorque(control->thrustSi, control->torque.x,
                                     control->torque.y, control->torque.z,
                                     CRAZYFLIE_ARM_LENGTH_M,
                                     CRAZYFLIE_THRUST_TO_TORQUE, forces);
        motorPower->m1 = motorForceToPwmValue(forces[0]);
        motorPower->m2 = motorForceToPwmValue(forces[1]);
        motorPower->m3 = motorForceToPwmValue(forces[2]);
        motorPower->m4 = motorForceToPwmValue(forces[3]);
        break;
    }
    case controlModeForce: {
        uint16_t pwms[4] = {0};
        powerDistributionForce(control->normalizedForces, pwms);
        motorPower->m1 = pwms[0];
        motorPower->m2 = pwms[1];
        motorPower->m3 = pwms[2];
        motorPower->m4 = pwms[3];
        break;
    }
    default:
        break;
    }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust)
{
    int32_t lower = idleThrust > 0 ? idleThrust : 0;
    return value < lower ? lower : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust)
{
    PowerCapResult result = { false, 0 };
    if (!motors) return result;

    int32_t maxv = motors[0];
    for (int i = 1; i < 4; ++i) {
        if (motors[i] > maxv) maxv = motors[i];
    }

    if (maxv > maxAllowedThrust) {
        result.reduction = maxv - maxAllowedThrust;
        result.isCapped = true;
        for (int i = 0; i < 4; ++i) {
            motors[i] -= result.reduction;
        }
    }

    for (int i = 0; i < 4; ++i) {
        motors[i] = capMinThrust(motors[i], idleThrust);
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
    if (actualVoltage <= 0.0f) return motorThrust;
    float comp = floorf(((float)motorThrust * nominalVoltage / actualVoltage) + 0.5f);
    if (comp < 0.0f) comp = 0.0f;
    if (comp > 65535.0f) comp = 65535.0f;
    return (uint16_t)comp;
}

static float attitudeDt = 0.002f;
static bool attitudeDtSet = false;

static void pidInitDefaults(PidObject *pid)
{
    if (!pid || pid->initialized) return;
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->kff = 0.0f;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
    pid->initialized = true;
}

static void pidReset(PidObject *pid)
{
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
}

static float pidUpdate(PidObject *pid, float actual, float desired, float dt, bool reset)
{
    if (!pid) return 0.0f;
    if (!pid->initialized) pidInitDefaults(pid);
    if (dt <= 0.0f) dt = 0.002f;

    float error = desired - actual;
    if (reset) {
        pid->integral = 0.0f;
        pid->prevError = 0.0f;
    }

    pid->integral += error * dt;
    float derivative = (error - pid->prevError) / dt;
    pid->prevError = error;
    pid->output = pid->kp * error + pid->ki * pid->integral +
                  pid->kd * derivative + pid->kff * desired;
    return pid->output;
}

void attitudeControllerInit(float updateDt)
{
    if (!attitudeDtSet && updateDt > 0.0f) {
        attitudeDt = updateDt;
        attitudeDtSet = true;
    }
    pidInitDefaults(&pidRoll);
    pidInitDefaults(&pidPitch);
    pidInitDefaults(&pidYaw);
    pidInitDefaults(&pidRollRate);
    pidInitDefaults(&pidPitchRate);
    pidInitDefaults(&pidYawRate);
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired)
{
    float r = pidUpdate(&pidRollRate, rollActual, rollDesired, attitudeDt, false);
    float p = pidUpdate(&pidPitchRate, pitchActual, pitchDesired, attitudeDt, false);
    float y = pidUpdate(&pidYawRate, yawActual, yawDesired, attitudeDt, false);
    pidRollRate.output = (float)saturateSignedInt16((int32_t)r);
    pidPitchRate.output = (float)saturateSignedInt16((int32_t)p);
    pidYawRate.output = (float)saturateSignedInt16((int32_t)y);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired)
{
    pidRoll.output = pidUpdate(&pidRoll, rollActual, rollDesired, attitudeDt, false);
    pidPitch.output = pidUpdate(&pidPitch, pitchActual, pitchDesired, attitudeDt, false);
    pidYaw.output = pidUpdate(&pidYaw, yawActual, yawDesired, attitudeDt, true);
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
    if (!roll || !pitch || !yaw) return;
    *roll = saturateSignedInt16((int32_t)pidRollRate.output);
    *pitch = saturateSignedInt16((int32_t)pidPitchRate.output);
    *yaw = saturateSignedInt16((int32_t)pidYawRate.output);
}

static void positionControllerReset(void)
{
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state)
{
    if (!setpoint || !state) return 0U;
    float out;
    if (setpoint->mode.z == modeAbs) {
        float posErr = setpoint->position.z - state->position.z;
        float velErr = setpoint->velocity.z - state->velocity.z;
        out = 32768.0f + 5000.0f * posErr + 1000.0f * velErr;
    } else if (setpoint->mode.z == modeVelocity) {
        float velErr = setpoint->velocity.z - state->velocity.z;
        out = 32768.0f + 1000.0f * velErr;
    } else {
        out = (float)setpoint->thrust;
    }
    if (out < 0.0f) out = 0.0f;
    if (out > 65535.0f) out = 65535.0f;
    return (uint16_t)out;
}

static float quaternionYaw(const Quaternion *q)
{
    if (!q) return 0.0f;
    float q0 = q->w, q1 = q->x, q2 = q->y, q3 = q->z;
    return atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG_F;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt)
{
    if (!sensors || !setpoint || !state || !control) return;

    control->controlMode = controlModeLegacy;
    control->thrustSi = 0.0f;
    control->torque.x = control->torque.y = control->torque.z = 0.0f;
    for (int i = 0; i < 4; ++i) control->normalizedForces[i] = 0.0f;

    static float desiredYaw = 0.0f;

    if (setpoint->thrust == 0U) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0U;
        attitudeControllerResetAllPID(state->attitude.roll,
                                      state->attitude.pitch,
                                      state->attitude.yaw);
        positionControllerReset();
        desiredYaw = state->attitude.yaw;
        return;
    }

    float rollDesiredRate = 0.0f;
    float pitchDesiredRate = 0.0f;
    float yawDesiredRate = 0.0f;

    if (setpoint->mode.roll == modeVelocity) {
        rollDesiredRate = setpoint->attitudeRate.roll;
        attitudeControllerResetRollAttitudePID(state->attitude.roll);
    } else if (setpoint->mode.roll == modeAbs) {
        pidRoll.output = pidUpdate(&pidRoll, state->attitude.roll,
                                  setpoint->attitude.roll, attitudeDt, false);
        rollDesiredRate = pidRoll.output;
    } else {
        rollDesiredRate = 0.0f;
    }

    if (setpoint->mode.pitch == modeVelocity) {
        pitchDesiredRate = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
    } else if (setpoint->mode.pitch == modeAbs) {
        pidPitch.output = pidUpdate(&pidPitch, state->attitude.pitch,
                                   setpoint->attitude.pitch, attitudeDt, false);
        pitchDesiredRate = pidPitch.output;
    } else {
        pitchDesiredRate = 0.0f;
    }

    bool yawControlled = false;
    if (setpoint->mode.quat == modeAbs) {
        desiredYaw = quaternionYaw(&setpoint->attitudeQuaternion);
        pidYaw.output = pidUpdate(&pidYaw, state->attitude.yaw, desiredYaw, attitudeDt, true);
        yawDesiredRate = pidYaw.output;
        yawControlled = true;
    } else if (setpoint->mode.yaw == modeVelocity) {
        desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
        if (yawMaxDelta != 0.0f) {
            float delta = capAngle(desiredYaw - state->attitude.yaw);
            if (delta > yawMaxDelta) desiredYaw = state->attitude.yaw + yawMaxDelta;
            else if (delta < -yawMaxDelta) desiredYaw = state->attitude.yaw - yawMaxDelta;
        }
        pidYaw.output = pidUpdate(&pidYaw, state->attitude.yaw, desiredYaw, attitudeDt, true);
        yawDesiredRate = pidYaw.output;
        yawControlled = true;
    } else if (setpoint->mode.yaw == modeAbs) {
        desiredYaw = setpoint->attitude.yaw;
        pidYaw.output = pidUpdate(&pidYaw, state->attitude.yaw, desiredYaw, attitudeDt, true);
        yawDesiredRate = pidYaw.output;
        yawControlled = true;
    }

    if (!yawControlled) {
        desiredYaw = state->attitude.yaw;
        yawDesiredRate = 0.0f;
    }

    attitudeControllerCorrectRatePID(sensors->gyro.x, rollDesiredRate,
                                     -sensors->gyro.y, pitchDesiredRate,
                                     sensors->gyro.z, yawDesiredRate);

    int16_t actRoll, actPitch, actYaw;
    attitudeControllerGetActuatorOutput(&actRoll, &actPitch, &actYaw);
    control->roll = actRoll;
    control->pitch = actPitch;
    control->yaw = -actYaw;

    if (setpoint->mode.z == modeDisable) {
        control->thrust = setpoint->thrust;
    } else {
        control->thrust = positionControllerUpdate(setpoint, state);
    }
}

typedef struct {
    EstimatorMeasurement buffer[16];
    unsigned head, tail, count;
} EstimatorFifo;

static EstimatorFifo estimatorFifo = {0};
static EstimatorMeasurement lastGyro = {0}, lastAcc = {0}, lastBaro = {0}, lastTof = {0};
static bool hasGyro = false, hasAcc = false, hasBaro = false, hasTof = false;
static State estimatedState = {0};
static float estimatedVerticalVelocity = 0.0f;

bool estimatorEnqueue(const EstimatorMeasurement *measurement)
{
    if (!measurement) return false;
    if (estimatorFifo.count >= 16U) return false;
    estimatorFifo.buffer[estimatorFifo.tail] = *measurement;
    estimatorFifo.tail = (estimatorFifo.tail + 1U) % 16U;
    estimatorFifo.count++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement)
{
    if (!measurement) return false;
    if (estimatorFifo.count == 0U) return false;
    *measurement = estimatorFifo.buffer[estimatorFifo.head];
    estimatorFifo.head = (estimatorFifo.head + 1U) % 16U;
    estimatorFifo.count--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep)
{
    hostTickMs = stabilizerStep;

    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        switch (m.type) {
        case MeasurementTypeGyroscope:
            lastGyro = m;
            hasGyro = true;
            break;
        case MeasurementTypeAcceleration:
            lastAcc = m;
            hasAcc = true;
            break;
        case MeasurementTypeBarometer:
            lastBaro = m;
            hasBaro = true;
            break;
        case MeasurementTypeTOF:
            lastTof = m;
            hasTof = true;
            break;
        default:
            break;
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        if (hasGyro && hasAcc) {
            sensfusion6UpdateQ(lastGyro.data[0], lastGyro.data[1], lastGyro.data[2],
                               lastAcc.data[0], lastAcc.data[1], lastAcc.data[2],
                               1.0f / (float)SENSFUSION_RATE_HZ);
            sensfusion6GetEulerRPY(&stateEstimate.roll, &stateEstimate.pitch,
                                   &stateEstimate.yaw);
            sensfusion6GetQuaternion(&stateEstimate.qw, &stateEstimate.qx,
                                     &stateEstimate.qy, &stateEstimate.qz);
            estimatedState.attitude.roll = stateEstimate.roll;
            estimatedState.attitude.pitch = stateEstimate.pitch;
            estimatedState.attitude.yaw = stateEstimate.yaw;
            estimatedState.attitudeQuaternion.w = stateEstimate.qw;
            estimatedState.attitudeQuaternion.x = stateEstimate.qx;
            estimatedState.attitudeQuaternion.y = stateEstimate.qy;
            estimatedState.attitudeQuaternion.z = stateEstimate.qz;
        }
        if (hasGyro) {
            gyro.x = lastGyro.data[0];
            gyro.y = lastGyro.data[1];
            gyro.z = lastGyro.data[2];
        }
        if (hasAcc) {
            acc.x = lastAcc.data[0];
            acc.y = lastAcc.data[1];
            acc.z = lastAcc.data[2];
            estimatedState.acc.x = lastAcc.data[0];
            estimatedState.acc.y = lastAcc.data[1];
            estimatedState.acc.z = lastAcc.data[2];
            float azNoG = sensfusion6GetAccZWithoutGravity(lastAcc.data[0],
                                                           lastAcc.data[1],
                                                           lastAcc.data[2]);
            estimatedVerticalVelocity += azNoG * 9.81f * (1.0f / (float)SENSFUSION_RATE_HZ);
            estimatedState.velocity.z = estimatedVerticalVelocity;
        }
        if (hasBaro) {
            baro.asl = lastBaro.data[0];
            baro.temp = lastBaro.data[1];
            baro.pressure = lastBaro.data[2];
        }
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        estimatedState.position.z += estimatedState.velocity.z * (1.0f / (float)POSITION_RATE_HZ);
    }
}

static Setpoint activeCommanderSetpoint = {0};
static int activePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t lastCommanderUpdateTick = 0U;
static bool trajectoryFlying = false;
static bool trajectoryFinished = false;
static bool trajectoryDisabled = true;

bool commanderSetSetpoint(const Setpoint *setpoint, int priority)
{
    if (!setpoint) return false;
    if (hostTickMs == 0U && setpoint->timestamp != 0U) {
        hostTickMs = setpoint->timestamp;
    }

    if (priority == COMMANDER_PRIORITY_DISABLE) {
        activeCommanderSetpoint = *setpoint;
        activePriority = COMMANDER_PRIORITY_DISABLE;
        lastCommanderUpdateTick = hostTickMs;
        return true;
    }

    if (priority < activePriority && activePriority != COMMANDER_PRIORITY_DISABLE) {
        return false;
    }

    if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
        trajectoryFlying = false;
        trajectoryFinished = true;
        trajectoryDisabled = false;
    }

    activeCommanderSetpoint = *setpoint;
    activePriority = priority;
    lastCommanderUpdateTick = hostTickMs;
    return true;
}

void commanderRelaxPriority(void)
{
    activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void)
{
    if (hostTickMs < lastCommanderUpdateTick) return 0U;
    return hostTickMs - lastCommanderUpdateTick;
}

int commanderGetActivePriority(void)
{
    return activePriority;
}

static bool commanderGetSetpoint(Setpoint *out)
{
    if (!out) return false;
    *out = activeCommanderSetpoint;
    return true;
}

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime)
{
    if (!rollPrime || !pitchPrime) return;
    float rad = yaw_deg * DEG_TO_RAD_F;
    float s = sinf(rad);
    float c = cosf(rad);
    *rollPrime = roll * c - pitch * s;
    *pitchPrime = roll * s + pitch * c;
}

static bool carefreeErrorObserved = false;

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
    memset(setpoint, 0, sizeof(*setpoint));

    if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
    }
    if (values->thrust == 0U) {
        thrustLocked = false;
    }

    if (yawMode == CAREFREE) {
        carefreeErrorObserved = true;
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.z = modeDisable;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->mode.yaw = modeDisable;
        setpoint->mode.quat = modeDisable;
        return;
    }

    float rollCmd = values->roll;
    float pitchCmd = values->pitch;
    float yawCmd = values->yaw;

    if (yawMode == PLUSMODE) {
        rotateYaw(rollCmd, pitchCmd, 45.0f, &rollCmd, &pitchCmd);
    }

    bool rollPitchHandled = false;

    if (altHoldMode) {
        if (!commanderModeSet) {
            positionControllerReset();
        }
        commanderModeSet = true;
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0U;
        setpoint->velocity.z = ((float)values->thrust - 32767.0f) / 32767.0f;
    } else {
        if (commanderModeSet) {
            setpoint->mode.z = modeDisable;
            commanderModeSet = false;
        }

        if (posSetMode && values->thrust != 0U) {
            setpoint->mode.x = modeAbs;
            setpoint->mode.y = modeAbs;
            setpoint->mode.z = modeAbs;
            setpoint->mode.roll = modeDisable;
            setpoint->mode.pitch = modeDisable;
            setpoint->mode.yaw = modeAbs;
            setpoint->mode.quat = modeDisable;
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
            rollPitchHandled = true;
        }

        if (thrustLocked || values->thrust < MIN_THRUST) {
            setpoint->thrust = 0U;
        } else {
            setpoint->thrust = (values->thrust > MAX_THRUST) ? MAX_THRUST : values->thrust;
        }
    }

    if (!rollPitchHandled) {
        if (stabilizationModeRoll == RATE) {
            setpoint->mode.roll = modeVelocity;
            setpoint->attitudeRate.roll = rollCmd;
        } else if (stabilizationModeRoll == ANGLE) {
            setpoint->mode.roll = modeAbs;
            setpoint->attitude.roll = rollCmd;
        } else {
            setpoint->mode.roll = modeDisable;
        }

        if (stabilizationModePitch == RATE) {
            setpoint->mode.pitch = modeVelocity;
            setpoint->attitudeRate.pitch = pitchCmd;
        } else if (stabilizationModePitch == ANGLE) {
            setpoint->mode.pitch = modeAbs;
            setpoint->attitude.pitch = pitchCmd;
        } else {
            setpoint->mode.pitch = modeDisable;
        }
    }

    if (stabilizationModeYaw == RATE) {
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = -yawCmd;
    } else if (stabilizationModeYaw == ANGLE) {
        setpoint->mode.yaw = modeAbs;
        setpoint->attitude.yaw = yawCmd;
    } else {
        setpoint->mode.yaw = modeDisable;
    }
    setpoint->mode.quat = modeDisable;
}

static bool supArmed = false;
static bool supCrashed = false;
static bool supFreeFall = false;
static bool supTumbled = false;
static bool supIsFlying = false;
static bool supAutoArming = false;
static uint32_t supSpinupTimeoutDurationMs = 0U;
static uint32_t supSpinupStartTick = 0U;
static bool supSpinupStarted = false;
static uint32_t supLatestArmingTick = 0U;
static uint32_t supLatestLandingTick = 0U;
static uint32_t supCurrentTick = 0U;
static SensorData supSensors = {0};
static uint32_t supMotorRatios[4] = {0};
static int32_t supMotorRPMs[4] = {0};
static uint32_t supIdleThrust = 0U;
static float supCrashDetectionGs = 0.0f;
static float supFreeFallThreshold = 0.0f;
static float supAcceptedTiltAccZ = 0.0f;
static float supAcceptedUpsideDownAccZ = 0.0f;
static uint32_t supMaxTiltTime = 0U;
static uint32_t supMaxUpsideDownTime = 0U;
static bool supTumbleCheckEnabled = false;
static SupervisorState prevSupervisorState = supervisorStateLocked;
static bool supervisorDeckFault = false;
static uint32_t supPreflightTimeoutDuration = DEFAULT_PREFLIGHT_TIMEOUT_MS;
static uint32_t supLandingTimeoutDuration = DEFAULT_LANDING_TIMEOUT_MS;
static bool supCrtpEmergencyStop = false;
static bool supParamEmergencyStop = false;
static bool supEmergencyStopWatchdogFailed = false;
static int32_t supRpmCheckMin = 0;
static int32_t supRpmCheckMax = INT32_MAX;
static int32_t supRpmThreshold = 0;
static uint32_t supRpmCheckDurationMs = 0U;

static bool flyingSeen = false;
static uint32_t flyingRecentTick = 0U;
static uint32_t tiltStartTick = 0U;
static bool tiltTimerRunning = false;
static uint32_t notRespondingStartTick = 0U;

static bool keepArmingState(SupervisorState s)
{
    return (s == supervisorStateArming || s == supervisorStateReadyToFly ||
            s == supervisorStateFlying || s == supervisorStateWarningLevelOut ||
            s == supervisorStateLanded);
}

void supervisorInit(void)
{
    supervisorState = supervisorStateLocked;
    supervisorConditionBits = 0U;
    supArmed = false;
    supCrashed = false;
    supFreeFall = false;
    supTumbled = false;
    supIsFlying = false;
    supAutoArming = false;
    supSpinupTimeoutDurationMs = 0U;
    supSpinupStartTick = 0U;
    supSpinupStarted = false;
    supLatestArmingTick = 0U;
    supLatestLandingTick = 0U;
    supCurrentTick = 0U;
    memset(&supSensors, 0, sizeof(supSensors));
    memset(supMotorRatios, 0, sizeof(supMotorRatios));
    memset(supMotorRPMs, 0, sizeof(supMotorRPMs));
    supIdleThrust = 0U;
    supCrashDetectionGs = 0.0f;
    supFreeFallThreshold = 0.0f;
    supAcceptedTiltAccZ = 0.0f;
    supAcceptedUpsideDownAccZ = 0.0f;
    supMaxTiltTime = 0U;
    supMaxUpsideDownTime = 0U;
    supTumbleCheckEnabled = false;
    prevSupervisorState = supervisorStateLocked;
    flyingSeen = false;
    flyingRecentTick = 0U;
    tiltStartTick = 0U;
    tiltTimerRunning = false;
    notRespondingStartTick = 0U;
    supervisorDeckFault = false;
    supPreflightTimeoutDuration = DEFAULT_PREFLIGHT_TIMEOUT_MS;
    supLandingTimeoutDuration = DEFAULT_LANDING_TIMEOUT_MS;
    supCrtpEmergencyStop = false;
    supParamEmergencyStop = false;
    supEmergencyStopWatchdogFailed = false;
    supRpmCheckMin = 0;
    supRpmCheckMax = INT32_MAX;
    supRpmThreshold = 0;
    supRpmCheckDurationMs = 0U;
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
    return supArmed;
}

bool supervisorIsCrashed(void)
{
    return supCrashed;
}

bool supervisorAreMotorsAllowedToRun(void)
{
    return keepArmingState(supervisorState);
}

bool supervisorRequestArming(bool doArm)
{
    if (doArm) {
        if (!supervisorCanArm()) return false;
        if (supArmed && supervisorState == supervisorStateArming) return true;
        supArmed = true;
        supervisorState = supervisorStateArming;
        supLatestArmingTick = hostTickMs;
        supSpinupStartTick = hostTickMs;
        supSpinupStarted = true;
        return true;
    }

    supArmed = false;
    return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
    if (supTumbled) return false;
    if (!doRecovery) {
        supCrashed = true;
        return true;
    }
    supCrashed = false;
    return true;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick)
{
    if (!motorRatios) return false;
    for (int i = 0; i < 4; ++i) {
        if (motorRatios[i] > idleThrust) {
            flyingSeen = true;
            flyingRecentTick = currentTick;
        }
    }
    if (!flyingSeen) return false;
    return (currentTick - flyingRecentTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling)
{
    if (isFreeFalling) *isFreeFalling = false;

    if (crashDetectionGs > 0.0f) {
        float accNorm = sqrtf(accX * accX + accY * accY + accZ * accZ);
        if (fabsf(accNorm - 1.0f) > crashDetectionGs) {
            supCrashed = true;
        }
    }

    if (freeFallThreshold > 0.0f &&
        fabsf(accX) < freeFallThreshold &&
        fabsf(accY) < freeFallThreshold &&
        fabsf(accZ) < freeFallThreshold) {
        if (isFreeFalling) *isFreeFalling = true;
        supFreeFall = true;
        supTumbled = false;
        tiltStartTick = 0U;
        tiltTimerRunning = false;
        if (supervisorState != supervisorStateCrashed) {
            supervisorState = supervisorStateExceptFreeFall;
        }
        return false;
    }

    supFreeFall = false;

    if (!tumbleCheckEnabled) {
        supTumbled = false;
        tiltTimerRunning = false;
        tiltStartTick = 0U;
        return false;
    }

    uint32_t timeoutMs = 0U;

    if (accZ < acceptedUpsideDownAccZ) {
        timeoutMs = maxUpsideDownTime;
    } else if (accZ < acceptedTiltAccZ) {
        timeoutMs = maxTiltTime;
    } else {
        tiltTimerRunning = false;
        tiltStartTick = 0U;
        supTumbled = false;
        return false;
    }

    if (!tiltTimerRunning) {
        tiltTimerRunning = true;
        tiltStartTick = currentTick;
    }

    if (timeoutMs > 0U && (currentTick - tiltStartTick) >= timeoutMs) {
        supTumbled = true;
    } else {
        supTumbled = false;
    }
    return supTumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick)
{
    if (lastNotificationTick == 0U) return true;
    return (currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration)
{
    if (state != supervisorStateReadyToFly) return false;
    if (latestArmingTick == 0U || preflightTimeoutDuration == 0U) return false;
    return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration)
{
    if (latestLandingTick == 0U || landingTimeoutDuration == 0U) return false;
    return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed)
{
    supCrtpEmergencyStop = crtpEmergencyStop;
    supParamEmergencyStop = paramEmergencyStop;
    supEmergencyStopWatchdogFailed = emergencyStopWatchdogFailed;

    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
    }
    return supervisorConditionBits;
}

static void zeroSetpoint(Setpoint *sp)
{
    if (!sp) return;
    memset(sp, 0, sizeof(*sp));
}

static bool hasCriticalSupervisorFault(uint32_t bits)
{
    return (bits & (SUPERVISOR_CB_EMERGENCY_STOP |
                    SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT |
                    SUPERVISOR_CB_IS_TUMBLED |
                    SUPERVISOR_CB_CRASHED |
                    SUPERVISOR_CB_FREE_FALL |
                    SUPERVISOR_CB_MOTORS_NOT_RESPONDING)) != 0U;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBits_,
                                SupervisorState state)
{
    if (!setpoint) return;

    if (hasCriticalSupervisorFault(supervisorConditionBits_)) {
        zeroSetpoint(setpoint);
        return;
    }

    if (state == supervisorStateWarningLevelOut) {
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->attitudeRate.yaw = 0.0f;
        return;
    }

    if (state == supervisorStateArming ||
        state == supervisorStateReadyToFly ||
        state == supervisorStateFlying ||
        state == supervisorStateLanded) {
        return;
    }

    zeroSetpoint(setpoint);
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax)
{
    if (!motorRPMs) return false;
    if (rpmCheckMin > rpmCheckMax) return false;
    for (int i = 0; i < 4; ++i) {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTick)
{
    if (!motorRPMs || !canFly) {
        notRespondingStartTick = 0U;
        return false;
    }

    bool anyBelow = false;
    for (int i = 0; i < 4; ++i) {
        if (motorRPMs[i] < rpmThreshold) {
            anyBelow = true;
            break;
        }
    }

    if (!anyBelow) {
        notRespondingStartTick = 0U;
        return false;
    }

    if (notRespondingStartTick == 0U) {
        notRespondingStartTick = currentTick;
    }
    if (rpmCheckDurationMs > 0U) {
        return (currentTick - notRespondingStartTick) >= rpmCheckDurationMs;
    }
    return false;
}

void supervisorSetSensorData(const SensorData *sensors)
{
    if (!sensors) {
        memset(&supSensors, 0, sizeof(supSensors));
        return;
    }
    supSensors = *sensors;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust)
{
    if (!motorRatios) return;
    for (int i = 0; i < 4; ++i) supMotorRatios[i] = motorRatios[i];
    supIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
    if (!motorRPMs) return;
    for (int i = 0; i < 4; ++i) supMotorRPMs[i] = motorRPMs[i];
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled)
{
    supCrashDetectionGs = crashDetectionGs;
    supFreeFallThreshold = freeFallThreshold;
    supAcceptedTiltAccZ = acceptedTiltAccZ;
    supAcceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
    supMaxTiltTime = maxTiltTime;
    supMaxUpsideDownTime = maxUpsideDownTime;
    supTumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs)
{
    supAutoArming = autoArming;
    supSpinupTimeoutDurationMs = spinupTimeoutDurationMs;
}

uint16_t supervisorGetInfoBitfield(void)
{
    uint16_t bits = 0U;
    if (supervisorCanArm()) bits |= (1UL << 0);
    if (supArmed) bits |= (1UL << 1);
    if (supAutoArming) bits |= (1UL << 2);
    if (supervisorCanFly()) bits |= (1UL << 3);
    if (supIsFlying) bits |= (1UL << 4);
    if (supTumbled) bits |= (1UL << 5);
    if (supervisorState == supervisorStateLocked) bits |= (1UL << 6);
    if (supCrashed) bits |= (1UL << 7);
    if (trajectoryFlying) bits |= (1UL << 8);
    if (trajectoryFinished) bits |= (1UL << 9);
    if (trajectoryDisabled) bits |= (1UL << 10);
    if (supervisorDeckFault) bits |= (1UL << 11);
    return bits;
}

void supervisorUpdate(uint32_t stabilizerStep)
{
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

    hostTickMs = stabilizerStep;
    supCurrentTick = stabilizerStep;

    SupervisorState oldState = supervisorState;

    bool freeFall = false;
    bool tumbled = isTumbledCheck(supSensors.acc.x, supSensors.acc.y, supSensors.acc.z,
                                  supCrashDetectionGs, supFreeFallThreshold,
                                  supAcceptedTiltAccZ, supAcceptedUpsideDownAccZ,
                                  supMaxTiltTime, supMaxUpsideDownTime,
                                  supTumbleCheckEnabled, supCurrentTick, &freeFall);
    supFreeFall = freeFall;
    supTumbled = tumbled;

    if (supCrashed && supervisorState != supervisorStateCrashed) {
        supervisorState = supervisorStateCrashed;
    }

    supIsFlying = isFlyingCheck(supMotorRatios, supIdleThrust, supCurrentTick);

    if (oldState == supervisorStatePreFlChecksPassed &&
        prevSupervisorState != supervisorStatePreFlChecksPassed &&
        supAutoArming && !supArmed) {
        supervisorRequestArming(true);
    }

    if (keepArmingState(prevSupervisorState) && !keepArmingState(supervisorState)) {
        supArmed = false;
    }

    if (prevSupervisorState == supervisorStateArming &&
        supervisorState != supervisorStateArming) {
        supSpinupStartTick = 0U;
        supSpinupStarted = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }

    if (supervisorState == supervisorStateArming) {
        if (!supSpinupStarted) {
            supSpinupStarted = true;
            supSpinupStartTick = supCurrentTick;
        }
    }

    if (prevSupervisorState != supervisorStateWarningLevelOut &&
        supervisorState == supervisorStateWarningLevelOut) {
        supLatestLandingTick = supCurrentTick;
    }
    if (prevSupervisorState != supervisorStateLanded &&
        supervisorState == supervisorStateLanded) {
        supLatestLandingTick = supCurrentTick;
    }

    prevSupervisorState = supervisorState;

    uint32_t bits = 0U;
    if (supArmed) bits |= SUPERVISOR_CB_ARMED;
    if (supIsFlying) bits |= SUPERVISOR_CB_IS_FLYING;
    if (supTumbled) bits |= SUPERVISOR_CB_IS_TUMBLED;
    if (supCrashed) bits |= SUPERVISOR_CB_CRASHED;
    if (supFreeFall) bits |= SUPERVISOR_CB_FREE_FALL;
    if (supervisorDeckFault) bits |= SUPERVISOR_CB_DECK_FAULT;

    uint32_t inactivity = commanderGetInactivityTime();
    if (inactivity > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
        bits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    } else if (inactivity > COMMANDER_WDT_TIMEOUT_STABILIZE) {
        bits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    }

    if (supervisorIsPreflightTimeout(supervisorState, supLatestArmingTick,
                                     supCurrentTick, supPreflightTimeoutDuration)) {
        bits |= SUPERVISOR_CB_PREFLIGHT_TIMEOUT;
    }
    if (supervisorIsLandingTimeout(supLatestLandingTick, supCurrentTick,
                                   supLandingTimeoutDuration)) {
        bits |= SUPERVISOR_CB_LANDING_TIMEOUT;
    }

    if (supervisorState == supervisorStateArming) {
        if (supSpinupStarted && supSpinupTimeoutDurationMs > 0U &&
            (supCurrentTick - supSpinupStartTick) >= supSpinupTimeoutDurationMs) {
            bits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
        }
    }

    if (supArmed && supervisorState == supervisorStateArming) {
        if (isRPMatArmingValid(supMotorRPMs, supRpmCheckMin, supRpmCheckMax)) {
            bits |= SUPERVISOR_CB_RPM_AT_ARMING_VALID;
        }
    }

    bool motorsNotResponding = isMotorsNotResponding(supMotorRPMs, supRpmThreshold,
                                                     supRpmCheckDurationMs,
                                                     supervisorCanFly(),
                                                     supCurrentTick);
    if (motorsNotResponding) {
        bits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    }

    uint32_t emergencyBits = updateAndPopulateConditions(supCrtpEmergencyStop,
                                                         supParamEmergencyStop,
                                                         supEmergencyStopWatchdogFailed);
    bits |= (emergencyBits & SUPERVISOR_CB_EMERGENCY_STOP);

    supervisorConditionBits = bits;
    supervisorLog.info = supervisorConditionBits;
    supervisorLog.accNorm = sqrtf(supSensors.acc.x * supSensors.acc.x +
                                  supSensors.acc.y * supSensors.acc.y +
                                  supSensors.acc.z * supSensors.acc.z);
}

static void sensorsInit(void) {}
static void stateEstimatorInit(void) { estimatedState.attitudeQuaternion.w = 1.0f; }
static void controllerInit(void) { attitudeControllerInit(0.002f); }
static void powerDistributionInit(void) {}
static void motorsInit(void) {}
static void collisionAvoidanceInit(void) {}
static void createStabilizerTask(void) {}

static SensorData lastSensorsData = {0};
static bool sensorsReady = true;
static bool sensorsActive = false;
static bool sensorsPaused = false;
static bool highLevelSetpointPending = false;
static Setpoint highLevelSetpoint = {0};
static uint32_t stabilizerTick = 0U;

static bool sensorsWaitDataReady(void) { return sensorsReady; }
static void sensorsAcquire(SensorData *out)
{
    if (!out) return;
    *out = lastSensorsData;
    sensorsActive = true;
}
static void stateEstimator(const SensorData *sensors, State *stateOut)
{
    (void)sensors;
    if (stateOut) *stateOut = estimatedState;
}
static void collisionAvoidanceUpdateSetpoint(Setpoint *sp)
{
    (void)sp;
}

static void setMotorRatiosInternal(const MotorPower *mp)
{
    if (!mp) return;
    motor.m1req = (uint16_t)(mp->m1 < 0 ? 0 : (mp->m1 > 65535 ? 65535 : mp->m1));
    motor.m2req = (uint16_t)(mp->m2 < 0 ? 0 : (mp->m2 > 65535 ? 65535 : mp->m2));
    motor.m3req = (uint16_t)(mp->m3 < 0 ? 0 : (mp->m3 > 65535 ? 65535 : mp->m3));
    motor.m4req = (uint16_t)(mp->m4 < 0 ? 0 : (mp->m4 > 65535 ? 65535 : mp->m4));
}

static bool stabilizerInitialized = false;

void stabilizerInit(void)
{
    if (stabilizerInitialized) return;
    sensorsInit();
    stateEstimatorInit();
    controllerInit();
    powerDistributionInit();
    motorsInit();
    collisionAvoidanceInit();
    createStabilizerTask();
    stabilizerInitialized = true;
}

void stabilizerTask(void)
{
    if (!stabilizerInitialized) stabilizerInit();
    hostTickMs = stabilizerTick;

    if (!sensorsWaitDataReady()) return;

    SensorData sensors;
    sensorsAcquire(&sensors);
    State state;
    stateEstimator(&sensors, &state);

    if (healthShallWeRunTest()) {
        healthRunTests(&sensors);
        return;
    }

    if (highLevelSetpointPending) {
        commanderSetSetpoint(&highLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        highLevelSetpointPending = false;
    }

    Setpoint sp;
    (void)commanderGetSetpoint(&sp);

    supervisorUpdate(stabilizerTick);
    collisionAvoidanceUpdateSetpoint(&sp);
    supervisorOverrideSetpoint(&sp, supervisorConditionBits, supervisorState);

    if (!supervisorCanFly()) {
        zeroSetpoint(&sp);
        motor.m1req = motor.m2req = motor.m3req = motor.m4req = 0U;
        stabilizerTick++;
        return;
    }

    ControlData control;
    controllerPid(&sensors, &sp, &state, &control, 0.0f, 0.002f);

    MotorPower mp;
    powerDistribution(&control, &mp);

    float measuredBatteryVoltage = (float)batteryVoltage / 1000.0f;
    float compensatedBatteryVoltage = batteryCompensation(measuredBatteryVoltage,
                                                          filteredBatteryVoltage,
                                                          0.01f);
    filteredBatteryVoltage = compensatedBatteryVoltage;

    int32_t m[4] = { mp.m1, mp.m2, mp.m3, mp.m4 };
    for (int i = 0; i < 4; ++i) {
        int32_t v = m[i];
        if (v < 0) v = 0;
        if (v > 65535) v = 65535;
        m[i] = motorsCompensateBatteryVoltage((uint16_t)v, 6.0f,
                                              compensatedBatteryVoltage);
    }

    powerDistributionCap(m, 65535, 0);
    mp.m1 = m[0]; mp.m2 = m[1]; mp.m3 = m[2]; mp.m4 = m[3];

    if (!supervisorAreMotorsAllowedToRun()) {
        mp.m1 = mp.m2 = mp.m3 = mp.m4 = 0;
    }

    setMotorRatiosInternal(&mp);
    stabilizerTick++;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint)
{
    if (!setpoint) return false;
    highLevelSetpoint = *setpoint;
    highLevelSetpointPending = true;
    return true;
}

static uint32_t quatcompress(const Quaternion *q)
{
    if (!q) return 0U;
    float qw_ = q->w, qx_ = q->x, qy_ = q->y, qz_ = q->z;
    if (qw_ < 0.0f) {
        qw_ = -qw_; qx_ = -qx_; qy_ = -qy_; qz_ = -qz_;
    }
    int8_t a = (int8_t)(qw_ * 127.0f);
    int8_t b = (int8_t)(qx_ * 127.0f);
    int8_t c = (int8_t)(qy_ * 127.0f);
    int8_t d = (int8_t)(qz_ * 127.0f);
    return ((uint32_t)(uint8_t)a << 24) | ((uint32_t)(uint8_t)b << 16) |
           ((uint32_t)(uint8_t)c << 8) | (uint32_t)(uint8_t)d;
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
    output->gyro_millirad_s[0] = sensors->gyro.x * (PI_F / 180.0f) * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * (PI_F / 180.0f) * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * (PI_F / 180.0f) * 1000.0f;
    output->quatCompressed = quatcompress(&state->attitudeQuaternion);
}

bool rateSupervisorValidate(uint32_t measuredRate)
{
    return measuredRate >= 997U && measuredRate <= 1003U;
}

static bool rateSupervisorStarted = false;
static uint32_t rateSupervisorStartTick = 0U;
static bool rateSupervisorError = false;

void rateSupervisorTask(void)
{
    if (!rateSupervisorStarted) {
        rateSupervisorStartTick = hostTickMs;
        rateSupervisorStarted = true;
        return;
    }
    if ((hostTickMs - rateSupervisorStartTick) >= 2000U) {
        if (!sensorsPaused && sensorsActive) {
            rateSupervisorError = true;
            supervisorState = supervisorStateLocked;
            supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
        }
        rateSupervisorStartTick = hostTickMs;
    }
}

static bool propRequestPending = false;
static bool batRequestPending = false;
static uint32_t healthMotorTestCount = 0U;
static uint32_t healthSampleCount = 0U;
static float noiseBuffer[PROPTEST_NBR_OF_VARIANCE_VALUES] = {0};
static float propNoiseVarianceValue = 0.0f;
static int currentPropMotor = 0;
static bool healthMotorActive = false;
static float propMeasuredSum = 0.0f;
static float propLoadedVoltageSum = 0.0f;
static float batteryIdleVoltage = 0.0f;
static float minLoadedVoltage = 0.0f;
static uint32_t batteryTestTick = 0U;
static uint32_t batteryRestartStartTick = 0U;
static float healthPropLowThreshold = 0.0f;
static float healthPropHighThreshold = 0.0f;

void healthRequestPropTest(void)
{
    propRequestPending = true;
}

void healthRequestBatteryTest(void)
{
    batRequestPending = true;
}

bool healthShallWeRunTest(void)
{
    if (healthTestState != testDone) return true;

    if (propRequestPending) {
        healthTestState = configureAcc;
        propRequestPending = false;
        return true;
    }
    if (batRequestPending) {
        batRequestPending = false;
        batteryTestTick = 0U;
        batteryRestartStartTick = 0U;
        batteryIdleVoltage = (float)batteryVoltage / 1000.0f;
        minLoadedVoltage = 1000.0f;
        batterySag = 0.0f;
        batteryPass = 0U;
        healthTestState = testBattery;
        return true;
    }
    return false;
}

void healthRunTests(const SensorData *sensorData)
{
    switch (healthTestState) {
    case configureAcc:
        motorPass = 0U;
        batteryPass = 0U;
        batterySag = 0.0f;
        healthMotorTestCount = 0U;
        healthSampleCount = 0U;
        propMeasuredSum = 0.0f;
        propLoadedVoltageSum = 0.0f;
        propNoiseVarianceValue = 0.0f;
        currentPropMotor = 0;
        healthMotorActive = false;
        motor.m1req = motor.m2req = motor.m3req = motor.m4req = 0U;
        batteryIdleVoltage = (float)batteryVoltage / 1000.0f;
        minLoadedVoltage = 1000.0f;
        healthTestState = measureNoiseFloor;
        break;
    case measureNoiseFloor:
        if (sensorData && healthSampleCount < PROPTEST_NBR_OF_VARIANCE_VALUES) {
            noiseBuffer[healthSampleCount++] = sensorData->acc.x;
        }
        if (healthSampleCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
            propNoiseVarianceValue = variance(noiseBuffer, PROPTEST_NBR_OF_VARIANCE_VALUES);
            healthSampleCount = 0U;
            propMeasuredSum = 0.0f;
            propLoadedVoltageSum = 0.0f;
            currentPropMotor = 0;
            healthMotorActive = false;
            healthTestState = measureProp;
        }
        break;
    case measureProp:
        if (currentPropMotor > 3) {
            healthTestState = evaluatePropResult;
            break;
        }

        if (!healthMotorActive) {
            motor.m1req = motor.m2req = motor.m3req = motor.m4req = 0U;
            if (currentPropMotor == 0) motor.m1req = 65535U;
            else if (currentPropMotor == 1) motor.m2req = 65535U;
            else if (currentPropMotor == 2) motor.m3req = 65535U;
            else if (currentPropMotor == 3) motor.m4req = 65535U;
            healthMotorActive = true;
            healthSampleCount = 0U;
            propMeasuredSum = 0.0f;
            propLoadedVoltageSum = 0.0f;
            break;
        }

        if (sensorData && healthSampleCount < PROPTEST_NBR_OF_VARIANCE_VALUES) {
            propMeasuredSum += sensorData->acc.y;
            propLoadedVoltageSum += (float)batteryVoltage / 1000.0f;
            healthSampleCount++;
        }
        if (healthSampleCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
            float measured = propMeasuredSum / (float)PROPTEST_NBR_OF_VARIANCE_VALUES;
            healthMotorTestCount++;
            bool pass = evaluatePropTest(healthPropLowThreshold,
                                         healthPropHighThreshold,
                                         measured,
                                         (uint8_t)currentPropMotor);
            if (pass) {
                motorPass |= (uint8_t)(1U << currentPropMotor);
            } else {
                motorPass &= (uint8_t)(~(1U << currentPropMotor));
            }
            motor.m1req = motor.m2req = motor.m3req = motor.m4req = 0U;
            healthMotorActive = false;
            currentPropMotor++;
            healthSampleCount = 0U;
            propMeasuredSum = 0.0f;
            propLoadedVoltageSum = 0.0f;
            if (currentPropMotor > 3) {
                healthTestState = evaluatePropResult;
            }
        }
        break;
    case evaluatePropResult:
        healthTestState = testDone;
        break;
    case testBattery:
        batteryTestTick++;
        if (batteryTestTick == 1U) {
            motor.m1req = motor.m2req = motor.m3req = motor.m4req = 65535U;
            minLoadedVoltage = 1000.0f;
        } else if (batteryTestTick >= 2U && batteryTestTick <= 49U) {
            float v = (float)batteryVoltage / 1000.0f;
            if (v < minLoadedVoltage) minLoadedVoltage = v;
        } else if (batteryTestTick >= 50U) {
            motor.m1req = motor.m2req = motor.m3req = motor.m4req = 0U;
            batterySag = batteryIdleVoltage - minLoadedVoltage;
            healthTestState = evaluateBatResult;
            batteryTestTick = 0U;
        }
        break;
    case evaluateBatResult:
        batteryPass = (batterySag <= batterySagThreshold) ? 1U : 0U;
        healthTestState = testDone;
        break;
    case restartBatTest:
        if (batteryRestartStartTick == 0U) {
            batteryRestartStartTick = hostTickMs;
        }
        if ((hostTickMs - batteryRestartStartTick) >= 2000U) {
            batteryRestartStartTick = 0U;
            batteryTestTick = 0U;
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
    healthLog.motorTestCount = healthMotorTestCount;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motor)
{
    (void)motor;
    if (highThreshold == 0.0f) return true;
    return measuredValue >= lowThreshold && measuredValue <= highThreshold;
}

float variance(const float *buffer, int length)
{
    if (!buffer || length <= 0) return 0.0f;
    float sum = 0.0f;
    float sumSq = 0.0f;
    for (int i = 0; i < length; ++i) {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }
    float n = (float)length;
    return sumSq - (sum * sum / n);
}

static CrtpPacket txQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t txHead = 0U, txTail = 0U, txCount = 0U;
static CrtpPacket rxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint16_t rxHead[CRTP_NBR_OF_PORTS] = {0};
static uint16_t rxTail[CRTP_NBR_OF_PORTS] = {0};
static uint16_t rxCount[CRTP_NBR_OF_PORTS] = {0};
static bool rxQueueCreated[CRTP_NBR_OF_PORTS] = {false};
static CrtpPortCallback portCallbacks[CRTP_NBR_OF_PORTS] = {0};
static bool crtpInitialized = false;
static bool crtpError = false;
static CrtpLink *currentLink = NULL;
static bool txPending = false;
static CrtpPacket txPendingPacket = {0};
static uint32_t txRetryTick = 0U;
static uint32_t lastStatsTick = 0U;
static uint32_t rxPacketCount = 0U;
static uint32_t txPacketCount = 0U;
static uint32_t rxRate = 0U;
static uint32_t txRate = 0U;
static uint32_t rxWaitStart[CRTP_NBR_OF_PORTS] = {0};
static bool rxBlockActive[CRTP_NBR_OF_PORTS] = {false};
static bool rxWaitActive[CRTP_NBR_OF_PORTS] = {false};

static bool nopSendPacket(CrtpPacket *packet) { (void)packet; return true; }
static bool nopReceivePacket(CrtpPacket *packet) { (void)packet; return false; }
static bool nopIsConnected(void) { return true; }
static void nopSetEnable(bool enable) { (void)enable; }
static void nopReset(void) { }

static CrtpLink nopLink = {
    nopSendPacket,
    nopReceivePacket,
    nopIsConnected,
    nopSetEnable,
    nopReset
};

static void crtpEnqueueRx(uint8_t port, const CrtpPacket *packet)
{
    if (port >= CRTP_NBR_OF_PORTS || !packet) return;
    if (!rxQueueCreated[port] || rxCount[port] >= CRTP_RX_QUEUE_SIZE) return;
    rxQueues[port][rxTail[port]] = *packet;
    rxTail[port] = (rxTail[port] + 1U) % CRTP_RX_QUEUE_SIZE;
    rxCount[port]++;
}

static bool crtpDequeueRx(uint8_t port, CrtpPacket *packet)
{
    if (port >= CRTP_NBR_OF_PORTS || !packet) return false;
    if (!rxQueueCreated[port] || rxCount[port] == 0U) return false;
    *packet = rxQueues[port][rxHead[port]];
    rxHead[port] = (rxHead[port] + 1U) % CRTP_RX_QUEUE_SIZE;
    rxCount[port]--;
    return true;
}

void crtpInit(void)
{
    if (crtpInitialized) return;
    txHead = txTail = txCount = 0U;
    for (uint32_t i = 0U; i < CRTP_NBR_OF_PORTS; ++i) {
        rxHead[i] = rxTail[i] = 0U;
        rxCount[i] = 0U;
        rxQueueCreated[i] = false;
        portCallbacks[i] = NULL;
        rxWaitStart[i] = 0U;
        rxBlockActive[i] = false;
        rxWaitActive[i] = false;
    }
    crtpError = false;
    currentLink = &nopLink;
    txPending = false;
    lastStatsTick = 0U;
    rxPacketCount = 0U;
    txPacketCount = 0U;
    rxRate = 0U;
    txRate = 0U;
    crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port)
{
    if (port >= CRTP_NBR_OF_PORTS) {
        crtpError = true;
        return;
    }
    if (rxQueueCreated[port]) {
        crtpError = true;
        return;
    }
    rxQueueCreated[port] = true;
    rxHead[port] = rxTail[port] = 0U;
    rxCount[port] = 0U;
    rxWaitStart[port] = 0U;
    rxBlockActive[port] = false;
    rxWaitActive[port] = false;
}

bool crtpSendPacket(const CrtpPacket *packet)
{
    if (!packet || packet->size > 30U || txCount >= CRTP_TX_QUEUE_SIZE) return false;
    txQueue[txTail] = *packet;
    txTail = (txTail + 1U) % CRTP_TX_QUEUE_SIZE;
    txCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet)
{
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet)
{
    return crtpDequeueRx(port, packet);
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet)
{
    if (port >= CRTP_NBR_OF_PORTS || !packet || !rxQueueCreated[port]) return false;

    if (crtpDequeueRx(port, packet)) {
        rxBlockActive[port] = false;
        rxWaitStart[port] = 0U;
        rxWaitActive[port] = false;
        return true;
    }

    if (currentLink && currentLink != &nopLink && currentLink->receivePacket) {
        crtpRxTask();
    }

    if (crtpDequeueRx(port, packet)) {
        rxBlockActive[port] = false;
        rxWaitStart[port] = 0U;
        rxWaitActive[port] = false;
        return true;
    }

    rxBlockActive[port] = true;
    return false;
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet, uint32_t wait_ms)
{
    if (port >= CRTP_NBR_OF_PORTS || !packet || !rxQueueCreated[port]) return false;

    if (crtpDequeueRx(port, packet)) {
        rxWaitStart[port] = 0U;
        rxWaitActive[port] = false;
        rxBlockActive[port] = false;
        return true;
    }

    if (wait_ms == 0U) {
        rxWaitStart[port] = 0U;
        rxWaitActive[port] = false;
        return false;
    }

    if (currentLink && currentLink != &nopLink && currentLink->receivePacket) {
        crtpRxTask();
    }

    if (crtpDequeueRx(port, packet)) {
        rxWaitStart[port] = 0U;
        rxWaitActive[port] = false;
        rxBlockActive[port] = false;
        return true;
    }

    if (rxWaitStart[port] == 0U) {
        rxWaitStart[port] = hostTickMs;
        rxWaitActive[port] = true;
        return false;
    }

    uint32_t elapsed = (hostTickMs >= rxWaitStart[port]) ? (hostTickMs - rxWaitStart[port]) : 0U;
    if (elapsed < wait_ms) {
        return false;
    }

    rxWaitStart[port] = 0U;
    rxWaitActive[port] = false;
    return false;
}

void crtpRxTask(void)
{
    if (!currentLink || currentLink == &nopLink) return;
    if (!currentLink->receivePacket) return;
    CrtpPacket packet;
    if (!currentLink->receivePacket(&packet)) return;
    rxPacketCount++;

    if (packet.port < CRTP_NBR_OF_PORTS) {
        if (rxQueueCreated[packet.port]) {
            crtpEnqueueRx(packet.port, &packet);
        }
        if (portCallbacks[packet.port]) {
            portCallbacks[packet.port](&packet);
        }
    }
}

static bool crtpTxPop(CrtpPacket *packet)
{
    if (!packet || txCount == 0U) return false;
    *packet = txQueue[txHead];
    txHead = (txHead + 1U) % CRTP_TX_QUEUE_SIZE;
    txCount--;
    return true;
}

void crtpTxTask(void)
{
    if (!currentLink || currentLink == &nopLink) return;
    if (!currentLink->sendPacket) return;

    if (txPending) {
        if ((hostTickMs - txRetryTick) < 10U) return;
        if (currentLink->sendPacket(&txPendingPacket)) {
            txPending = false;
            CrtpPacket dummy;
            if (crtpTxPop(&dummy)) {
                txPacketCount++;
            }
        } else {
            txRetryTick = hostTickMs;
        }
        return;
    }

    CrtpPacket packet;
    if (txCount == 0U) return;
    packet = txQueue[txHead];
    if (currentLink->sendPacket(&packet)) {
        CrtpPacket dummy;
        if (crtpTxPop(&dummy)) {
            txPacketCount++;
        }
    } else {
        txPending = true;
        txPendingPacket = packet;
        txRetryTick = hostTickMs;
    }
}

void crtpSetLink(CrtpLink *newLink)
{
    if (currentLink && currentLink->setEnable && currentLink != &nopLink) {
        currentLink->setEnable(false);
    }
    currentLink = newLink ? newLink : &nopLink;
    if (currentLink && currentLink->setEnable && currentLink != &nopLink) {
        currentLink->setEnable(true);
    }
}

void crtpReset(void)
{
    txHead = txTail = txCount = 0U;
    for (uint32_t i = 0U; i < CRTP_NBR_OF_PORTS; ++i) {
        rxHead[i] = rxTail[i] = 0U;
        rxCount[i] = 0U;
        rxWaitStart[i] = 0U;
        rxBlockActive[i] = false;
        rxWaitActive[i] = false;
    }
    txPending = false;
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
    return (uint32_t)(CRTP_TX_QUEUE_SIZE - txCount);
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback)
{
    if (port >= CRTP_NBR_OF_PORTS) {
        crtpError = true;
        return;
    }
    portCallbacks[port] = callback;
}

void updateStats(void)
{
    if (lastStatsTick == 0U) {
        lastStatsTick = hostTickMs;
        return;
    }
    uint32_t elapsed = hostTickMs - lastStatsTick;
    if (elapsed >= 500U) {
        rxRate = rxPacketCount * 1000U / elapsed;
        txRate = txPacketCount * 1000U / elapsed;
        rxPacketCount = 0U;
        txPacketCount = 0U;
        lastStatsTick = hostTickMs;
    }
}

static bool deckEntryEqual(const DeckInfo *a, const DeckInfo *b)
{
    if (!a || !b) return false;
    if (a->foundByI2C && b->foundByI2C) {
        return a->i2cAddress == b->i2cAddress;
    }
    if (a->foundByOneWire && b->foundByOneWire) {
        return a->oneWireRomId == b->oneWireRomId;
    }
    return false;
}

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity)
{
    if (!decks || capacity == 0U) return 0U;

    static const uint8_t knownI2c[] = { 0x20U, 0x21U, 0x22U };
    static const uint64_t knownRom[] = {
        0x0000000000000001ULL,
        0x0000000000000002ULL,
        0x0000000000000003ULL
    };

    uint8_t count = 0U;
    DeckInfo candidate;

    for (size_t i = 0; i < sizeof(knownI2c); ++i) {
        if (count >= capacity) break;
        candidate.foundByI2C = true;
        candidate.foundByOneWire = false;
        candidate.i2cAddress = knownI2c[i];
        candidate.oneWireRomId = 0U;
        bool duplicate = false;
        for (uint8_t j = 0; j < count; ++j) {
            if (deckEntryEqual(&decks[j], &candidate)) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            decks[count++] = candidate;
        }
    }

    for (size_t i = 0; i < sizeof(knownRom) / sizeof(knownRom[0]); ++i) {
        if (count >= capacity) break;
        candidate.foundByI2C = false;
        candidate.foundByOneWire = true;
        candidate.i2cAddress = 0U;
        candidate.oneWireRomId = knownRom[i];
        bool duplicate = false;
        for (uint8_t j = 0; j < count; ++j) {
            if (deckEntryEqual(&decks[j], &candidate)) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            decks[count++] = candidate;
        }
    }

    return count;
}
#include "6_generated_code.h"
#include <math.h>

#define PI_F 3.14159265358979f
#define DEG_TO_RAD_F (PI_F / 180.0f)
#define RAD_TO_DEG_F (180.0f / PI_F)

uint32_t g_platformTick = 0U;

/* ---------------- shared helpers ---------------- */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void zeroSetpoint(Setpoint *sp)
{
    if (sp != NULL) {
        *sp = (Setpoint){0};
    }
}

/* ================= sensfusion6 ================= */
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

void sensfusion6Init(void)
{
    if (!sensfusion6IsInit) {
        qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
        gravityX = 0.0f; gravityY = 0.0f; gravityZ = 0.0f;
        integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
        baseZacc = 0.0f;
        sensfusion6IsInit = true;
        sensfusion6IsCalibrated = false;
    }
}

bool sensfusion6Test(void)
{
    return sensfusion6IsInit;
}

void estimatedGravityDirection(float qw_, float qx_, float qy_, float qz_,
                               float *gravX, float *gravY, float *gravZ)
{
    if (gravX == NULL || gravY == NULL || gravZ == NULL) return;
    *gravX = 2.0f * (qx_ * qz_ - qw_ * qy_);
    *gravY = 2.0f * (qw_ * qx_ + qy_ * qz_);
    *gravZ = qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_;
}

float invSqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    union { float f; uint32_t i; } u;
    u.f = x;
    u.i = 0x5f3759dfU - (u.i >> 1);
    float y = u.f;
    y = y * (1.5f - (x * 0.5f * y * y));
    return y;
}

static void sensfusion6SyncLog(void)
{
    sensfusion6Log.qw = qw; sensfusion6Log.qx = qx; sensfusion6Log.qy = qy; sensfusion6Log.qz = qz;
    sensfusion6Log.gravityX = gravityX; sensfusion6Log.gravityY = gravityY; sensfusion6Log.gravityZ = gravityZ;
    sensfusion6Log.accZbase = baseZacc;
    sensfusion6Log.isInit = sensfusion6IsInit;
    sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
    if (dt <= 0.0f) return;

    float gxr = gx * DEG_TO_RAD_F;
    float gyr = gy * DEG_TO_RAD_F;
    float gzr = gz * DEG_TO_RAD_F;

    if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
        float halfdt = 0.5f * dt;
        float q0 = qw, q1 = qx, q2 = qy, q3 = qz;
        qw = q0 + (-q1 * gxr - q2 * gyr - q3 * gzr) * halfdt;
        qx = q1 + ( q0 * gxr + q2 * gzr - q3 * gyr) * halfdt;
        qy = q2 + ( q0 * gyr - q1 * gzr + q3 * gxr) * halfdt;
        qz = q3 + ( q0 * gzr + q1 * gyr - q2 * gxr) * halfdt;
        float norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
        if (norm > 1e-8f) {
            qw /= norm; qx /= norm; qy /= norm; qz /= norm;
        }
        sensfusion6SyncLog();
        return;
    }

    float accNorm = sqrtf(ax * ax + ay * ay + az * az);
    if (accNorm < 1e-8f) {
        sensfusion6SyncLog();
        return;
    }
    float axn = ax / accNorm;
    float ayn = ay / accNorm;
    float azn = az / accNorm;

    float gvx = 2.0f * (qx * qz - qw * qy);
    float gvy = 2.0f * (qw * qx + qy * qz);
    float gvz = qw * qw - qx * qx - qy * qy + qz * qz;

    float wx = gxr;
    float wy = gyr;
    float wz = gzr;

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
    float fx = gvx - axn;
    float fy = gvy - ayn;
    float fz = gvz - azn;
    wx += -beta * fx;
    wy += -beta * fy;
    wz += -beta * fz;
#else
    float ex = (ayn * gvz - azn * gvy);
    float ey = (azn * gvx - axn * gvz);
    float ez = (axn * gvy - ayn * gvx);

    if (twoKi > 0.0f) {
        integralFBx += twoKi * ex * dt;
        integralFBy += twoKi * ey * dt;
        integralFBz += twoKi * ez * dt;
    } else {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    wx += twoKp * ex + integralFBx;
    wy += twoKp * ey + integralFBy;
    wz += twoKp * ez + integralFBz;
#endif

    float halfdt = 0.5f * dt;
    float q0 = qw, q1 = qx, q2 = qy, q3 = qz;
    qw = q0 + (-q1 * wx - q2 * wy - q3 * wz) * halfdt;
    qx = q1 + ( q0 * wx + q2 * wz - q3 * wy) * halfdt;
    qy = q2 + ( q0 * wy - q1 * wz + q3 * wx) * halfdt;
    qz = q3 + ( q0 * wz + q1 * wy - q2 * wx) * halfdt;

    float norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm > 1e-8f) {
        qw /= norm; qx /= norm; qy /= norm; qz /= norm;
    }

    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);

    if (!sensfusion6IsCalibrated) {
        baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
        sensfusion6IsCalibrated = true;
    }

    sensfusion6SyncLog();
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (roll_deg == NULL || pitch_deg == NULL || yaw_deg == NULL) return;

    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (qw * qy - qz * qx);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    float pitch = asinf(sinp);

    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    float yaw = atan2f(siny_cosp, cosy_cosp);

    *roll_deg = roll * RAD_TO_DEG_F;
    *pitch_deg = pitch * RAD_TO_DEG_F;
    *yaw_deg = yaw * RAD_TO_DEG_F;
}

void sensfusion6GetQuaternion(float *w, float *x, float *y, float *z)
{
    if (w == NULL || x == NULL || y == NULL || z == NULL) return;
    *w = qw; *x = qx; *y = qy; *z = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az)
{
    return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az)
{
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* ================= power distribution ================= */
int16_t saturateSignedInt16(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32767) return -32767;
    return (int16_t)value;
}

float capAngle(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out)
{
    if (out == NULL) return;
    int32_t r = (int32_t)roll / 2;
    int32_t p = (int32_t)pitch / 2;
    out->m1 = (int32_t)thrust - r + p + (int32_t)yaw;
    out->m2 = (int32_t)thrust - r - p - (int32_t)yaw;
    out->m3 = (int32_t)thrust + r - p + (int32_t)yaw;
    out->m4 = (int32_t)thrust + r + p - (int32_t)yaw;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4])
{
    if (motorForces == NULL) return;

    float thrustPart = 0.25f * thrustSi;
    float rollPart = 0.0f;
    float pitchPart = 0.0f;
    float yawPart = 0.0f;

    if (armLength != 0.0f) {
        float arm = 0.707106781f * armLength;
        rollPart = 0.25f / arm * torqueX;
        pitchPart = 0.25f / arm * torqueY;
    }
    if (thrustToTorque != 0.0f) {
        yawPart = 0.25f / thrustToTorque * torqueZ;
    }

    float f1 = thrustPart - rollPart - pitchPart - yawPart;
    float f2 = thrustPart - rollPart + pitchPart + yawPart;
    float f3 = thrustPart + rollPart - pitchPart + yawPart;
    float f4 = thrustPart + rollPart + pitchPart - yawPart;

    motorForces[0] = f1 < 0.0f ? 0.0f : f1;
    motorForces[1] = f2 < 0.0f ? 0.0f : f2;
    motorForces[2] = f3 < 0.0f ? 0.0f : f3;
    motorForces[3] = f4 < 0.0f ? 0.0f : f4;
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4])
{
    if (normalizedForces == NULL || motorPWMs == NULL) return;
    for (int i = 0; i < 4; i++) {
        float f = clampf(normalizedForces[i], 0.0f, 1.0f);
        motorPWMs[i] = (uint16_t)(f * 65535.0f);
    }
}

static uint16_t motorForceToPwm(float forceN)
{
    float f = clampf(forceN, 0.0f, CRAZYFLIE_MAX_MOTOR_FORCE_N);
    return (uint16_t)((f / CRAZYFLIE_MAX_MOTOR_FORCE_N) * 65535.0f);
}

void powerDistribution(const ControlData *control, MotorPower *motorPower)
{
    if (control == NULL || motorPower == NULL) return;

    switch (control->controlMode) {
    case controlModeLegacy:
        powerDistributionLegacy(control->thrust, control->roll, control->pitch,
                                control->yaw, motorPower);
        break;
    case controlModeForceTorque: {
        float forces[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        powerDistributionForceTorque(control->thrustSi,
                                     control->torque.x, control->torque.y, control->torque.z,
                                     CRAZYFLIE_ARM_LENGTH_M, CRAZYFLIE_THRUST_TO_TORQUE,
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
    PowerCapResult result = {false, 0};
    if (motors == NULL) return result;

    int32_t maxMotor = motors[0];
    for (int i = 1; i < 4; i++) {
        if (motors[i] > maxMotor) maxMotor = motors[i];
    }

    if (maxMotor > maxAllowedThrust) {
        result.reduction = maxMotor - maxAllowedThrust;
        result.isCapped = true;
        for (int i = 0; i < 4; i++) {
            motors[i] -= result.reduction;
        }
    }

    for (int i = 0; i < 4; i++) {
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
    float compensated = roundf((float)motorThrust * nominalVoltage / actualVoltage);
    if (compensated < 0.0f) compensated = 0.0f;
    if (compensated > 65535.0f) compensated = 65535.0f;
    return (uint16_t)compensated;
}

/* ================= PID / controller ================= */
PidObject pidRoll, pidPitch, pidYaw;
PidObject pidRollRate, pidPitchRate, pidYawRate;

static bool s_attCtrlInitialized = false;
static float s_attCtrlDt = 0.002f;
static float s_desiredYaw = 0.0f;
static State s_estimatedState;

static void pidReset(PidObject *pid)
{
    if (pid == NULL) return;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
}

static float pidUpdate(PidObject *pid, float desired, float actual, float dt, bool resetSemantics)
{
    if (pid == NULL) return 0.0f;
    if (dt <= 0.0f) dt = 0.001f;

    float error = desired - actual;
    if (resetSemantics) {
        pid->integral = 0.0f;
        pid->prevError = error;
    }

    float p = pid->kp * error;
    pid->integral += pid->ki * error * dt;
    float d = pid->kd * (error - pid->prevError) / dt;
    pid->prevError = error;
    pid->output = p + pid->integral + d + pid->kff * desired;
    return pid->output;
}

void attitudeControllerInit(float updateDt)
{
    if (s_attCtrlInitialized) return;
    if (updateDt > 0.0f) s_attCtrlDt = updateDt;

    pidRoll = (PidObject){0};
    pidPitch = (PidObject){0};
    pidYaw = (PidObject){0};
    pidRollRate = (PidObject){0};
    pidPitchRate = (PidObject){0};
    pidYawRate = (PidObject){0};

    pidRoll.initialized = true;
    pidPitch.initialized = true;
    pidYaw.initialized = true;
    pidRollRate.initialized = true;
    pidPitchRate.initialized = true;
    pidYawRate.initialized = true;

    s_attCtrlInitialized = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired)
{
    pidUpdate(&pidRollRate, rollDesired, rollActual, s_attCtrlDt, false);
    pidUpdate(&pidPitchRate, pitchDesired, pitchActual, s_attCtrlDt, false);
    pidUpdate(&pidYawRate, yawDesired, yawActual, s_attCtrlDt, false);

    pidRollRate.output = (float)saturateSignedInt16((int32_t)pidRollRate.output);
    pidPitchRate.output = (float)saturateSignedInt16((int32_t)pidPitchRate.output);
    pidYawRate.output = (float)saturateSignedInt16((int32_t)pidYawRate.output);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired)
{
    pidUpdate(&pidRoll, rollDesired, rollActual, s_attCtrlDt, false);
    pidUpdate(&pidPitch, pitchDesired, pitchActual, s_attCtrlDt, false);
    pidUpdate(&pidYaw, yawDesired, yawActual, s_attCtrlDt, true);
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
    if (roll == NULL || pitch == NULL || yaw == NULL) return;
    *roll = saturateSignedInt16((int32_t)pidRollRate.output);
    *pitch = saturateSignedInt16((int32_t)pidPitchRate.output);
    *yaw = saturateSignedInt16((int32_t)pidYawRate.output);
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state)
{
    if (setpoint == NULL || state == NULL) return 0U;

    float zError = setpoint->position.z - state->position.z;
    float zVelError = setpoint->velocity.z - state->velocity.z;
    float out = 32767.0f + 1000.0f * zError + 250.0f * zVelError;

    if (out < (float)MIN_THRUST) out = (float)MIN_THRUST;
    if (out > (float)MAX_THRUST) out = (float)MAX_THRUST;
    return (uint16_t)out;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt)
{
    if (sensors == NULL || setpoint == NULL || state == NULL || control == NULL) return;

    control->controlMode = controlModeLegacy;

    uint16_t thrust;
    if (setpoint->mode.z == modeDisable) {
        thrust = setpoint->thrust;
    } else {
        thrust = positionControllerUpdate(setpoint, state);
    }

    if (thrust == 0U) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0U;
        attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
        s_desiredYaw = state->attitude.yaw;
        return;
    }

    float rollActual = sensors->gyro.x;
    float pitchActual = -sensors->gyro.y;
    float yawActual = sensors->gyro.z;

    float rollDesiredRate = 0.0f;
    float pitchDesiredRate = 0.0f;
    float yawDesiredRate = 0.0f;
    float yawDesired = state->attitude.yaw;

    if (setpoint->mode.roll == modeVelocity) {
        rollDesiredRate = setpoint->attitudeRate.roll;
        pidReset(&pidRoll);
    } else if (setpoint->mode.roll == modeAbs) {
        rollDesiredRate = pidUpdate(&pidRoll, setpoint->attitude.roll, state->attitude.roll, s_attCtrlDt, false);
    }

    if (setpoint->mode.pitch == modeVelocity) {
        pitchDesiredRate = setpoint->attitudeRate.pitch;
        pidReset(&pidPitch);
    } else if (setpoint->mode.pitch == modeAbs) {
        pitchDesiredRate = pidUpdate(&pidPitch, setpoint->attitude.pitch, state->attitude.pitch, s_attCtrlDt, false);
    }

    if (setpoint->mode.quat == modeAbs) {
        float qw_ = setpoint->attitudeQuaternion.w;
        float qx_ = setpoint->attitudeQuaternion.x;
        float qy_ = setpoint->attitudeQuaternion.y;
        float qz_ = setpoint->attitudeQuaternion.z;
        float siny_cosp = 2.0f * (qw_ * qz_ + qx_ * qy_);
        float cosy_cosp = 1.0f - 2.0f * (qy_ * qy_ + qz_ * qz_);
        yawDesired = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG_F;
    } else if (setpoint->mode.yaw == modeVelocity) {
        s_desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
        if (yawMaxDelta != 0.0f) {
            float diff = capAngle(s_desiredYaw - state->attitude.yaw);
            if (diff > yawMaxDelta) {
                s_desiredYaw = state->attitude.yaw + yawMaxDelta;
            } else if (diff < -yawMaxDelta) {
                s_desiredYaw = state->attitude.yaw - yawMaxDelta;
            }
        }
        yawDesired = s_desiredYaw;
    } else if (setpoint->mode.yaw == modeAbs) {
        yawDesired = setpoint->attitude.yaw;
    }

    if (setpoint->mode.yaw == modeVelocity) {
        yawDesiredRate = setpoint->attitudeRate.yaw;
    } else {
        yawDesiredRate = pidUpdate(&pidYaw, yawDesired, state->attitude.yaw, s_attCtrlDt, true);
    }

    attitudeControllerCorrectRatePID(rollActual, rollDesiredRate,
                                     pitchActual, pitchDesiredRate,
                                     yawActual, yawDesiredRate);

    int16_t rollOut = 0, pitchOut = 0, yawOut = 0;
    attitudeControllerGetActuatorOutput(&rollOut, &pitchOut, &yawOut);

    control->roll = rollOut;
    control->pitch = pitchOut;
    control->yaw = (int16_t)(-yawOut);
    control->thrust = thrust;
}

/* ================= CRTP Commander RPYT ================= */
static int s_commanderActivePriority = COMMANDER_PRIORITY_LOWEST;
static uint32_t s_commanderLastUpdateTick = 0U;
static Setpoint s_commanderActiveSetpoint;
static bool s_commanderSetpointValid = false;
static bool s_highLevelTrajectoryActive = false;

bool thrustLocked = false;
bool commanderModeSet = false;
static bool s_carefreeError = false;

static void decodeDefaultAttitudeForRpyt(const CommanderCrtpLegacyValues *values,
                                         Setpoint *setpoint,
                                         StabilizationType rollType,
                                         StabilizationType pitchType,
                                         StabilizationType yawType)
{
    if (values == NULL || setpoint == NULL) return;

    if (rollType == RATE) {
        setpoint->mode.roll = modeVelocity;
        setpoint->attitudeRate.roll = values->roll;
        setpoint->attitude.roll = 0.0f;
    } else {
        setpoint->mode.roll = modeAbs;
        setpoint->attitude.roll = values->roll;
        setpoint->attitudeRate.roll = 0.0f;
    }

    if (pitchType == RATE) {
        setpoint->mode.pitch = modeVelocity;
        setpoint->attitudeRate.pitch = values->pitch;
        setpoint->attitude.pitch = 0.0f;
    } else {
        setpoint->mode.pitch = modeAbs;
        setpoint->attitude.pitch = values->pitch;
        setpoint->attitudeRate.pitch = 0.0f;
    }

    if (yawType == RATE) {
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = -values->yaw;
        setpoint->attitude.yaw = 0.0f;
    } else {
        setpoint->mode.yaw = modeAbs;
        setpoint->attitude.yaw = values->yaw;
        setpoint->attitudeRate.yaw = 0.0f;
    }
}

static void applyYawModeToRpyt(Setpoint *setpoint,
                               YawMode yawMode,
                               StabilizationType rollType,
                               StabilizationType pitchType)
{
    if (setpoint == NULL) return;

    if (yawMode == XMODE) {
        return;
    }

    if (yawMode == CAREFREE) {
        zeroSetpoint(setpoint);
        s_carefreeError = true;
        return;
    }

    if (yawMode == PLUSMODE) {
        float r = 0.0f;
        float p = 0.0f;
        if (rollType == RATE) {
            r = setpoint->attitudeRate.roll;
        } else {
            r = setpoint->attitude.roll;
        }
        if (pitchType == RATE) {
            p = setpoint->attitudeRate.pitch;
        } else {
            p = setpoint->attitude.pitch;
        }

        float rp = 0.0f;
        float pp = 0.0f;
        rotateYaw(r, p, 45.0f, &rp, &pp);

        if (rollType == RATE) {
            setpoint->attitudeRate.roll = rp;
        } else {
            setpoint->attitude.roll = rp;
        }
        if (pitchType == RATE) {
            setpoint->attitudeRate.pitch = pp;
        } else {
            setpoint->attitude.pitch = pp;
        }
    }
}

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime)
{
    if (rollPrime == NULL || pitchPrime == NULL) return;
    float rad = yaw_deg * DEG_TO_RAD_F;
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
    YawMode yawMode)
{
    if (values == NULL || setpoint == NULL) return;

    uint16_t raw = values->thrust;

    if (s_commanderActivePriority == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
    }
    if (raw == 0U) {
        thrustLocked = false;
    }

    uint16_t thrustCmd = 0U;
    if (!altHoldMode) {
        if (thrustLocked || raw < MIN_THRUST) {
            thrustCmd = 0U;
        } else {
            thrustCmd = raw > MAX_THRUST ? MAX_THRUST : raw;
        }
    }

    zeroSetpoint(setpoint);

    if (altHoldMode) {
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0U;
        setpoint->velocity.z = ((float)raw - 32767.0f) / 32767.0f;

        if (!commanderModeSet) {
            commanderModeSet = true;
        }

        decodeDefaultAttitudeForRpyt(values, setpoint,
                                     stabilizationModeRoll, stabilizationModePitch,
                                     stabilizationModeYaw);
        applyYawModeToRpyt(setpoint, yawMode, stabilizationModeRoll, stabilizationModePitch);
        return;
    }

    if (commanderModeSet) {
        commanderModeSet = false;
        setpoint->mode.z = modeDisable;
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
        setpoint->thrust = thrustCmd;

        decodeDefaultAttitudeForRpyt(values, setpoint,
                                     stabilizationModeRoll, stabilizationModePitch,
                                     stabilizationModeYaw);
        applyYawModeToRpyt(setpoint, yawMode, stabilizationModeRoll, stabilizationModePitch);
        return;
    }

    if (posSetMode && raw != 0U) {
        setpoint->mode.x = modeAbs;
        setpoint->mode.y = modeAbs;
        setpoint->mode.z = modeAbs;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->mode.yaw = modeAbs;
        setpoint->position.x = -values->pitch;
        setpoint->position.y = values->roll;
        setpoint->position.z = (float)values->thrust / 1000.0f;
        setpoint->attitude.yaw = values->yaw;
        setpoint->thrust = 0U;
        return;
    }

    decodeDefaultAttitudeForRpyt(values, setpoint,
                                 stabilizationModeRoll, stabilizationModePitch,
                                 stabilizationModeYaw);
    setpoint->thrust = thrustCmd;
    applyYawModeToRpyt(setpoint, yawMode, stabilizationModeRoll, stabilizationModePitch);
}

/* ================= supervisor ================= */
SupervisorState supervisorState = supervisorStatePreFlChecksNotPassed;
uint32_t supervisorConditionBits = 0U;

static SensorData s_superSensor;
static uint32_t s_motorRatios[4] = {0U, 0U, 0U, 0U};
static uint32_t s_idleThrust = 0U;
static int32_t s_motorRPMs[4] = {0, 0, 0, 0};
static float s_crashDetectionGs = 0.0f;
static float s_freeFallThreshold = 0.0f;
static float s_acceptedTiltAccZ = 0.0f;
static float s_acceptedUpsideDownAccZ = 0.0f;
static uint32_t s_maxTiltTime = 0U;
static uint32_t s_maxUpsideDownTime = 0U;
static bool s_tumbleCheckEnabled = false;
static bool s_autoArming = false;
static uint32_t s_spinupTimeoutDurationMs = 0U;
static uint32_t s_lastFlightTick = 0U;
static bool s_seenFlight = false;
static uint32_t s_tumbleStartTick = 0U;
static bool s_tumbleTimerActive = false;
static uint32_t s_spinupStartTick = 0U;
static uint32_t s_latestArmingTick = 0U;
static uint32_t s_latestLandingTick = 0U;
static uint32_t s_rpmFaultStartTick = 0U;
static bool s_rpmFaultTimerActive = false;
static SupervisorState s_lastSupervisorState = supervisorStatePreFlChecksNotPassed;

static bool s_supervisorInitialized = false;

void supervisorInit(void)
{
    if (s_supervisorInitialized) return;

    supervisorState = supervisorStatePreFlChecksNotPassed;
    supervisorConditionBits = 0U;
    s_superSensor = (SensorData){0};
    for (int i = 0; i < 4; i++) {
        s_motorRatios[i] = 0U;
        s_motorRPMs[i] = 0;
    }
    s_idleThrust = 0U;
    s_crashDetectionGs = 0.0f;
    s_freeFallThreshold = 0.0f;
    s_acceptedTiltAccZ = 0.0f;
    s_acceptedUpsideDownAccZ = 0.0f;
    s_maxTiltTime = 0U;
    s_maxUpsideDownTime = 0U;
    s_tumbleCheckEnabled = false;
    s_autoArming = false;
    s_spinupTimeoutDurationMs = 0U;
    s_lastFlightTick = 0U;
    s_seenFlight = false;
    s_tumbleStartTick = 0U;
    s_tumbleTimerActive = false;
    s_spinupStartTick = 0U;
    s_latestArmingTick = 0U;
    s_latestLandingTick = 0U;
    s_rpmFaultStartTick = 0U;
    s_rpmFaultTimerActive = false;
    s_lastSupervisorState = supervisorStatePreFlChecksNotPassed;
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
    return (supervisorConditionBits & SUPERVISOR_CB_CRASHED) != 0U;
}

bool supervisorRequestArming(bool doArm)
{
    if (!doArm) {
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
        return true;
    }

    if (!supervisorCanArm()) {
        return false;
    }

    supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    supervisorState = supervisorStateArming;
    s_spinupStartTick = g_platformTick;
    s_latestArmingTick = g_platformTick;
    return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
    if ((supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) != 0U) {
        return false;
    }

    if (doRecovery) {
        supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    } else {
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    }
    return true;
}

bool supervisorAreMotorsAllowedToRun(void)
{
    return supervisorState == supervisorStateArming ||
           supervisorState == supervisorStateReadyToFly ||
           supervisorState == supervisorStateFlying ||
           supervisorState == supervisorStateWarningLevelOut ||
           supervisorState == supervisorStateLanded;
}

uint16_t supervisorGetInfoBitfield(void)
{
    uint16_t info = 0U;
    if (supervisorCanArm()) info |= (uint16_t)(1U << 0);
    if (supervisorIsArmed()) info |= (uint16_t)(1U << 1);
    if (s_autoArming) info |= (uint16_t)(1U << 2);
    if (supervisorCanFly()) info |= (uint16_t)(1U << 3);
    if ((supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) != 0U) info |= (uint16_t)(1U << 4);
    if ((supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) != 0U) info |= (uint16_t)(1U << 5);
    if (supervisorState == supervisorStateLocked) info |= (uint16_t)(1U << 6);
    if (supervisorIsCrashed()) info |= (uint16_t)(1U << 7);
    if ((supervisorConditionBits & SUPERVISOR_CB_DECK_FAULT) != 0U) info |= (uint16_t)(1U << 11);
    return info;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick)
{
    if (motorRatios == NULL) return false;

    for (int i = 0; i < 4; i++) {
        if (motorRatios[i] > idleThrust) {
            s_lastFlightTick = currentTick;
            s_seenFlight = true;
            break;
        }
    }

    if (!s_seenFlight) return false;
    return (currentTick - s_lastFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling)
{
    if (isFreeFalling != NULL) *isFreeFalling = false;

    float accNorm = sqrtf(accX * accX + accY * accY + accZ * accZ);

    if (crashDetectionGs > 0.0f && fabsf(accNorm - 1.0f) > crashDetectionGs) {
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    }

    if (fabsf(accX) < freeFallThreshold &&
        fabsf(accY) < freeFallThreshold &&
        fabsf(accZ) < freeFallThreshold) {
        if (isFreeFalling != NULL) *isFreeFalling = true;
        supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
        s_tumbleStartTick = 0U;
        s_tumbleTimerActive = false;
        return false;
    }

    supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

    if (!tumbleCheckEnabled) {
        return false;
    }

    uint32_t timeout = 0U;

    if (accZ < acceptedUpsideDownAccZ) {
        timeout = maxUpsideDownTime;
    } else if (accZ < acceptedTiltAccZ) {
        timeout = maxTiltTime;
    } else {
        s_tumbleStartTick = 0U;
        s_tumbleTimerActive = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
        return false;
    }

    if (!s_tumbleTimerActive) {
        s_tumbleStartTick = currentTick;
        s_tumbleTimerActive = true;
        return false;
    }

    if ((currentTick - s_tumbleStartTick) >= timeout) {
        supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
        return true;
    }

    return false;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick)
{
    if (lastNotificationTick == 0U) return true;
    return (currentTick - lastNotificationTick) < DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration)
{
    if (state != supervisorStateReadyToFly) return false;
    if (latestArmingTick == 0U) return false;
    return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration)
{
    if (latestLandingTick == 0U) return false;
    return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed)
{
    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
    }
    return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBits_,
                                SupervisorState state)
{
    if (setpoint == NULL) return;
    (void)supervisorConditionBits_;

    if (state == supervisorStateWarningLevelOut) {
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->mode.yaw = modeVelocity;
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
    if (motorRPMs == NULL) return false;
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
    if (motorRPMs == NULL) return false;

    if (!canFly) {
        s_rpmFaultStartTick = 0U;
        s_rpmFaultTimerActive = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return false;
    }

    bool below = true;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] >= rpmThreshold) {
            below = false;
            break;
        }
    }

    if (!below) {
        if ((supervisorConditionBits & SUPERVISOR_CB_MOTORS_NOT_RESPONDING) == 0U) {
            s_rpmFaultStartTick = 0U;
            s_rpmFaultTimerActive = false;
        }
        return (supervisorConditionBits & SUPERVISOR_CB_MOTORS_NOT_RESPONDING) != 0U;
    }

    if (!s_rpmFaultTimerActive) {
        s_rpmFaultStartTick = currentTick;
        s_rpmFaultTimerActive = true;
        return false;
    }

    if ((currentTick - s_rpmFaultStartTick) >= rpmCheckDurationMs) {
        supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return true;
    }

    return false;
}

void supervisorSetSensorData(const SensorData *sensors)
{
    if (sensors == NULL) return;
    s_superSensor = *sensors;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust)
{
    if (motorRatios == NULL) return;
    for (int i = 0; i < 4; i++) s_motorRatios[i] = motorRatios[i];
    s_idleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
    if (motorRPMs == NULL) return;
    for (int i = 0; i < 4; i++) s_motorRPMs[i] = motorRPMs[i];
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
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs)
{
    s_autoArming = autoArming;
    s_spinupTimeoutDurationMs = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep)
{
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

    uint32_t tick = g_platformTick;

    bool flying = isFlyingCheck(s_motorRatios, s_idleThrust, tick);
    if (flying) {
        supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
    }

    bool freeFall = false;
    bool tumbled = isTumbledCheck(s_superSensor.acc.x, s_superSensor.acc.y, s_superSensor.acc.z,
                                  s_crashDetectionGs, s_freeFallThreshold,
                                  s_acceptedTiltAccZ, s_acceptedUpsideDownAccZ,
                                  s_maxTiltTime, s_maxUpsideDownTime,
                                  s_tumbleCheckEnabled, tick, &freeFall);
    if (tumbled) {
        supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    }
    if (freeFall) {
        supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;
    }

    if (supervisorState == supervisorStatePreFlChecksPassed &&
        s_autoArming && !supervisorIsArmed()) {
        supervisorRequestArming(true);
    }

    if (supervisorState == supervisorStateArming) {
        if (s_spinupStartTick != 0U && s_spinupTimeoutDurationMs != 0U) {
            if ((tick - s_spinupStartTick) >= s_spinupTimeoutDurationMs) {
                supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
            } else {
                supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
            }
        }
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
        s_spinupStartTick = 0U;
    }

    uint32_t commanderAge = commanderGetInactivityTime();
    if (commanderAge > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    } else if (commanderAge > COMMANDER_WDT_TIMEOUT_STABILIZE) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    } else {
        supervisorConditionBits &= ~(SUPERVISOR_CB_COMMANDER_WDT_WARNING |
                                     SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT);
    }

    s_lastSupervisorState = supervisorState;
    supervisorLog.info = supervisorGetInfoBitfield();
    supervisorLog.accNorm = sqrtf(s_superSensor.acc.x * s_superSensor.acc.x +
                                  s_superSensor.acc.y * s_superSensor.acc.y +
                                  s_superSensor.acc.z * s_superSensor.acc.z);
}

/* ================= estimator / commander ================= */
#define ESTIMATOR_FIFO_CAPACITY 16U
static EstimatorMeasurement s_estimatorFifo[ESTIMATOR_FIFO_CAPACITY];
static uint8_t s_estimatorHead = 0U;
static uint8_t s_estimatorTail = 0U;
static uint8_t s_estimatorCount = 0U;
static EstimatorMeasurement s_lastMeasurement[4];
static bool s_hasLastMeasurement[4] = {false, false, false, false};

bool estimatorEnqueue(const EstimatorMeasurement *measurement)
{
    if (measurement == NULL) return false;
    if (s_estimatorCount >= ESTIMATOR_FIFO_CAPACITY) return false;

    s_estimatorFifo[s_estimatorTail] = *measurement;
    s_estimatorTail = (uint8_t)((s_estimatorTail + 1U) % ESTIMATOR_FIFO_CAPACITY);
    s_estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement)
{
    if (measurement == NULL) return false;
    if (s_estimatorCount == 0U) return false;

    *measurement = s_estimatorFifo[s_estimatorHead];
    s_estimatorHead = (uint8_t)((s_estimatorHead + 1U) % ESTIMATOR_FIFO_CAPACITY);
    s_estimatorCount--;
    return true;
}

static uint32_t quatCompressImpl(float qw_, float qx_, float qy_, float qz_)
{
    float norm = sqrtf(qw_ * qw_ + qx_ * qx_ + qy_ * qy_ + qz_ * qz_);
    if (norm < 1e-8f) return 0U;
    float q[4] = {qw_ / norm, qx_ / norm, qy_ / norm, qz_ / norm};

    int largest = 0;
    float largestAbs = fabsf(q[0]);
    for (int i = 1; i < 4; i++) {
        float a = fabsf(q[i]);
        if (a > largestAbs) {
            largestAbs = a;
            largest = i;
        }
    }

    int components[3];
    int n = 0;
    for (int i = 0; i < 4; i++) {
        if (i != largest) {
            float v = clampf(q[i], -1.0f, 1.0f);
            components[n++] = (int)((v + 1.0f) * 511.5f) & 0x3FF;
        }
    }

    uint32_t compressed = ((uint32_t)largest & 0x3U) << 30;
    compressed |= ((uint32_t)components[0] & 0x3FFU) << 20;
    compressed |= ((uint32_t)components[1] & 0x3FFU) << 10;
    compressed |= ((uint32_t)components[2] & 0x3FFU);
    return compressed;
}

void estimatorComplementary(uint32_t stabilizerStep)
{
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        if (m.type >= MeasurementTypeGyroscope && m.type <= MeasurementTypeTOF) {
            s_lastMeasurement[m.type] = m;
            s_hasLastMeasurement[m.type] = true;
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        float gx = 0.0f, gy = 0.0f, gz = 0.0f;
        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        if (s_hasLastMeasurement[MeasurementTypeGyroscope]) {
            gx = s_lastMeasurement[MeasurementTypeGyroscope].data[0];
            gy = s_lastMeasurement[MeasurementTypeGyroscope].data[1];
            gz = s_lastMeasurement[MeasurementTypeGyroscope].data[2];
        }
        if (s_hasLastMeasurement[MeasurementTypeAcceleration]) {
            ax = s_lastMeasurement[MeasurementTypeAcceleration].data[0];
            ay = s_lastMeasurement[MeasurementTypeAcceleration].data[1];
            az = s_lastMeasurement[MeasurementTypeAcceleration].data[2];
        }

        float dt = 1.0f / (float)SENSFUSION_RATE_HZ;
        sensfusion6UpdateQ(gx, gy, gz, ax, ay, az, dt);

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
        sensfusion6GetQuaternion(&qw, &qx, &qy, &qz);

        s_estimatedState.attitude.roll = roll;
        s_estimatedState.attitude.pitch = pitch;
        s_estimatedState.attitude.yaw = yaw;
        s_estimatedState.attitudeQuaternion.w = qw;
        s_estimatedState.attitudeQuaternion.x = qx;
        s_estimatedState.attitudeQuaternion.y = qy;
        s_estimatedState.attitudeQuaternion.z = qz;
        s_estimatedState.acc.x = ax;
        s_estimatedState.acc.y = ay;
        s_estimatedState.acc.z = az;

        float accZ = sensfusion6GetAccZWithoutGravity(ax, ay, az);
        s_estimatedState.velocity.z += accZ * dt;

        stateEstimate.roll = roll;
        stateEstimate.pitch = pitch;
        stateEstimate.yaw = yaw;
        stateEstimate.qx = qx;
        stateEstimate.qy = qy;
        stateEstimate.qz = qz;
        stateEstimate.qw = qw;
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        float dt = 1.0f / (float)POSITION_RATE_HZ;
        s_estimatedState.position.z += s_estimatedState.velocity.z * dt;
    }
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority)
{
    if (setpoint == NULL) return false;

    if (priority != COMMANDER_PRIORITY_DISABLE &&
        priority < s_commanderActivePriority) {
        return false;
    }

    s_commanderActiveSetpoint = *setpoint;
    s_commanderActivePriority = priority;
    s_commanderLastUpdateTick = g_platformTick;
    s_commanderSetpointValid = true;

    if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
        s_highLevelTrajectoryActive = false;
    }

    return true;
}

void commanderRelaxPriority(void)
{
    s_commanderActivePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void)
{
    return g_platformTick - s_commanderLastUpdateTick;
}

int commanderGetActivePriority(void)
{
    return s_commanderActivePriority;
}

static bool commanderGetSetpointInternal(Setpoint *setpoint)
{
    if (setpoint == NULL || !s_commanderSetpointValid) return false;
    *setpoint = s_commanderActiveSetpoint;
    return true;
}

/* ================= stabilizer / compress ================= */
StateEstimateLog stateEstimate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
Axis3Log gyro = {0.0f, 0.0f, 0.0f};
Axis3Log acc = {0.0f, 0.0f, 0.0f};
BaroLog baro = {0.0f, 0.0f, 0.0f};
MotorLog motor = {0U, 0U, 0U, 0U};
Sensfusion6Log sensfusion6Log = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, false};
SupervisorLog supervisorLog = {0U, 0.0f};
HealthLog healthLog = {0U, 0U, 0.0f, 0U};

static SensorData s_latestSensor;
static bool s_sensorDataReady = false;
static uint32_t s_stabilizerStep = 0U;
static bool s_stabilizerInitialized = false;
static bool s_startupComplete = false;
static bool s_sensorCalibrated = false;
static Setpoint s_pendingHighLevelSetpoint;
static bool s_pendingHighLevel = false;
static float s_supplyVoltage = 4.2f;
static float s_filteredBatteryVoltage = 4.2f;

static void sensorsInit(void)
{
    s_latestSensor = (SensorData){0};
    s_sensorDataReady = false;
}

static void stateEstimatorInit(void)
{
    s_estimatedState = (State){0};
    sensfusion6Init();
    for (int i = 0; i < 4; i++) {
        s_hasLastMeasurement[i] = false;
        s_lastMeasurement[i] = (EstimatorMeasurement){0};
    }
}

static void controllerInit(void)
{
    attitudeControllerInit(1.0f / (float)ATTITUDE_RATE_HZ);
}

static void powerDistributionInit(void)
{
    motor = (MotorLog){0U, 0U, 0U, 0U};
}

static void motorsInit(void)
{
    motor = (MotorLog){0U, 0U, 0U, 0U};
}

static void collisionAvoidanceInit(void)
{
    /* no-op in current frozen contract */
}

void stabilizerInit(void)
{
    if (s_stabilizerInitialized) return;

    sensorsInit();
    stateEstimatorInit();
    controllerInit();
    powerDistributionInit();
    motorsInit();
    collisionAvoidanceInit();

    s_startupComplete = true;
    s_sensorCalibrated = true;
    s_stabilizerInitialized = true;
}

static void sensorsWaitDataReady(void)
{
    /* Host scheduler model: readiness is injected; no unbounded wait loop. */
}

static void sensorsAcquire(SensorData *out)
{
    if (out != NULL) {
        *out = s_latestSensor;
    }
}

static void collisionAvoidanceUpdateSetpoint(Setpoint *sp)
{
    (void)sp;
    /* Frozen contract does not specify collision avoidance transforms; keep no-op. */
}

static void setMotorRatios(const MotorPower *motorPower)
{
    if (motorPower == NULL) {
        motor = (MotorLog){0U, 0U, 0U, 0U};
        return;
    }
    motor.m1req = (uint16_t)motorPower->m1;
    motor.m2req = (uint16_t)motorPower->m2;
    motor.m3req = (uint16_t)motorPower->m3;
    motor.m4req = (uint16_t)motorPower->m4;
}

void stabilizerTask(void)
{
    if (!s_stabilizerInitialized) return;
    if (!s_startupComplete || !s_sensorCalibrated) return;

    if (healthShallWeRunTest()) {
        healthRunTests(&s_latestSensor);
        return;
    }

    SensorData sensorData;
    sensorsWaitDataReady();
    sensorsAcquire(&sensorData);
    s_latestSensor = sensorData;

    estimatorComplementary(s_stabilizerStep);

    Setpoint setpoint = {0};
    if (!commanderGetSetpointInternal(&setpoint)) {
        zeroSetpoint(&setpoint);
    }

    supervisorUpdate(s_stabilizerStep);

    if (!supervisorCanFly()) {
        zeroSetpoint(&setpoint);
        commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_DISABLE);
    }

    if (s_pendingHighLevel) {
        commanderSetSetpoint(&s_pendingHighLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        s_pendingHighLevel = false;
        commanderGetSetpointInternal(&setpoint);
    }

    collisionAvoidanceUpdateSetpoint(&setpoint);
    supervisorOverrideSetpoint(&setpoint, supervisorConditionBits, supervisorState);

    ControlData control = {0};
    controllerPid(&sensorData, &setpoint, &s_estimatedState, &control,
                  0.0f, 1.0f / (float)ATTITUDE_RATE_HZ);

    MotorPower motorPower = {0, 0, 0, 0};
    powerDistribution(&control, &motorPower);

    float compensated = batteryCompensation(s_supplyVoltage, s_filteredBatteryVoltage, 0.01f);
    s_filteredBatteryVoltage = compensated;
    int32_t motorArray[4] = {motorPower.m1, motorPower.m2, motorPower.m3, motorPower.m4};
    powerDistributionCap(motorArray, 65535, 0);
    motorPower.m1 = motorArray[0];
    motorPower.m2 = motorArray[1];
    motorPower.m3 = motorArray[2];
    motorPower.m4 = motorArray[3];

    if (!supervisorAreMotorsAllowedToRun()) {
        motorPower = (MotorPower){0, 0, 0, 0};
    }

    setMotorRatios(&motorPower);

    gyro.x = sensorData.gyro.x;
    gyro.y = sensorData.gyro.y;
    gyro.z = sensorData.gyro.z;
    acc.x = sensorData.acc.x;
    acc.y = sensorData.acc.y;
    acc.z = sensorData.acc.z;
    baro.asl = sensorData.baroAsl;
    baro.temp = sensorData.baroTemperature;
    baro.pressure = sensorData.baroPressure;

    s_stabilizerStep++;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint)
{
    if (setpoint == NULL) return false;
    s_pendingHighLevelSetpoint = *setpoint;
    s_pendingHighLevel = true;
    return true;
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output)
{
    if (state == NULL || sensors == NULL || output == NULL) return;

    for (int i = 0; i < 3; i++) {
        float pos = (i == 0) ? state->position.x :
                   (i == 1) ? state->position.y : state->position.z;
        float vel = (i == 0) ? state->velocity.x :
                   (i == 1) ? state->velocity.y : state->velocity.z;
        output->position_mm[i] = (int32_t)(pos * 1000.0f);
        output->velocity_mms[i] = (int32_t)(vel * 1000.0f);
    }

    output->acceleration_mms2[0] = (int32_t)(sensors->acc.x * 9810.0f);
    output->acceleration_mms2[1] = (int32_t)(sensors->acc.y * 9810.0f);
    output->acceleration_mms2[2] = (int32_t)((sensors->acc.z + 1.0f) * 9810.0f);

    output->gyro_millirad_s[0] = sensors->gyro.x * DEG_TO_RAD_F * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * DEG_TO_RAD_F * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * DEG_TO_RAD_F * 1000.0f;

    output->quatCompressed = quatCompressImpl(state->attitudeQuaternion.w,
                                              state->attitudeQuaternion.x,
                                              state->attitudeQuaternion.y,
                                              state->attitudeQuaternion.z);
}

bool rateSupervisorValidate(uint32_t measuredRate)
{
    return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void)
{
    /* Wait/validate boundary is host-observable state only; no public error enum.
       The 2000 ms wait is represented by this bounded call. */
    static bool s_rateSupervisorError = false;
    (void)s_rateSupervisorError;
}

/* ================= health ================= */
TestState healthTestState = testDone;
uint8_t motorPass = 0U;
uint8_t batteryPass = 0U;
float batterySag = 0.0f;

static bool s_propTestRequested = false;
static bool s_batteryTestRequested = false;
static uint32_t s_batTick = 0U;
static float s_batIdleVoltage = 4.2f;
static float s_batMinLoadedVoltage = 4.2f;
static float s_propSamples[PROPTEST_NBR_OF_VARIANCE_VALUES];
static uint32_t s_propSampleIndex = 0U;
static int s_propMotorIndex = 0;
static uint32_t s_restartBatStartTick = 0U;
static float s_batterySagThreshold = 0.2f;

static void syncHealthLog(void)
{
    healthLog.motorPass = motorPass;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
}

bool healthShallWeRunTest(void)
{
    if (s_propTestRequested) {
        s_propTestRequested = false;
        healthTestState = configureAcc;
        motorPass = 0U;
        healthLog.motorTestCount = 0U;
        s_propSampleIndex = 0U;
        s_propMotorIndex = 0;
        syncHealthLog();
        return true;
    }

    if (s_batteryTestRequested) {
        s_batteryTestRequested = false;
        healthTestState = testBattery;
        s_batTick = 0U;
        s_batMinLoadedVoltage = 100000.0f;
        syncHealthLog();
        return true;
    }

    if (healthTestState != testDone) {
        return true;
    }

    return false;
}

void healthRunTests(const SensorData *sensorData)
{
    if (sensorData == NULL) return;

    switch (healthTestState) {
    case configureAcc:
        motorPass = 0U;
        healthLog.motorTestCount = 0U;
        s_propSampleIndex = 0U;
        s_propMotorIndex = 0;
        healthTestState = measureNoiseFloor;
        break;

    case measureNoiseFloor:
        if (s_propSampleIndex < (uint32_t)PROPTEST_NBR_OF_VARIANCE_VALUES) {
            s_propSamples[s_propSampleIndex++] = sensorData->acc.z;
        }
        if (s_propSampleIndex >= (uint32_t)PROPTEST_NBR_OF_VARIANCE_VALUES) {
            (void)variance(s_propSamples, (int)PROPTEST_NBR_OF_VARIANCE_VALUES);
            healthTestState = measureProp;
            s_propMotorIndex = 0;
        }
        break;

    case measureProp:
        if (s_propMotorIndex < 4) {
            float measured = 0.0f;
            (void)evaluatePropTest(0.0f, 10.0f, measured, (uint8_t)s_propMotorIndex);
            s_propMotorIndex++;
        } else {
            healthTestState = evaluatePropResult;
        }
        break;

    case evaluatePropResult:
        healthTestState = testDone;
        break;

    case testBattery:
        if (s_batTick == 0U) {
            s_batTick = 1U;
            s_batMinLoadedVoltage = 100000.0f;
        } else if (s_batTick < 50U) {
            s_batTick++;
            float v = 4.2f;
            if (v < s_batMinLoadedVoltage) s_batMinLoadedVoltage = v;
        } else {
            batterySag = s_batIdleVoltage - s_batMinLoadedVoltage;
            batteryPass = (batterySag <= s_batterySagThreshold) ? 1U : 0U;
            healthTestState = evaluateBatResult;
        }
        break;

    case evaluateBatResult:
        healthTestState = testDone;
        break;

    case restartBatTest:
        if (s_restartBatStartTick == 0U) {
            s_restartBatStartTick = g_platformTick;
        } else if ((g_platformTick - s_restartBatStartTick) >= 2000U) {
            s_restartBatStartTick = 0U;
            healthTestState = testBattery;
            s_batTick = 0U;
        }
        break;

    case testDone:
    default:
        break;
    }

    syncHealthLog();
}

void healthRequestPropTest(void)
{
    s_propTestRequested = true;
}

void healthRequestBatteryTest(void)
{
    s_batteryTestRequested = true;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIdx)
{
    if (highThreshold == 0.0f) {
        return true;
    }
    if (motorIdx >= 4U) return false;

    if (lowThreshold <= measuredValue && measuredValue <= highThreshold) {
        motorPass |= (uint8_t)(1U << motorIdx);
        syncHealthLog();
        return true;
    }

    healthLog.motorTestCount++;
    syncHealthLog();
    return false;
}

float variance(const float *buffer, int length)
{
    if (buffer == NULL || length <= 0) return 0.0f;

    float sum = 0.0f;
    float sumSq = 0.0f;
    for (int i = 0; i < length; i++) {
        float v = buffer[i];
        sum += v;
        sumSq += v * v;
    }
    return sumSq - (sum * sum / (float)length);
}

/* ================= CRTP ================= */
static CrtpPacket s_txQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t s_txHead = 0U;
static uint16_t s_txTail = 0U;
static uint16_t s_txCount = 0U;

static CrtpPacket s_rxQueue[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint8_t s_rxHead[CRTP_NBR_OF_PORTS];
static uint8_t s_rxTail[CRTP_NBR_OF_PORTS];
static uint8_t s_rxCount[CRTP_NBR_OF_PORTS];
static bool s_rxQueueCreated[CRTP_NBR_OF_PORTS];

static CrtpPortCallback s_portCallbacks[CRTP_NBR_OF_PORTS];
static CrtpLink s_nopLink = {0};
static CrtpLink *s_currentLink = &s_nopLink;
static bool s_crtpInitialized = false;
static bool s_crtpError = false;
static uint32_t s_lastStatsTick = 0U;
static uint32_t s_rxPacketCounter = 0U;
static uint32_t s_txPacketCounter = 0U;
static uint32_t s_txRetryTick = 0U;
static bool s_txRetryPending = false;
static CrtpPacket s_txRetryPacket;

uint32_t crtpRxRate = 0U;
uint32_t crtpTxRate = 0U;

void crtpInit(void)
{
    if (s_crtpInitialized) return;

    s_txHead = 0U;
    s_txTail = 0U;
    s_txCount = 0U;
    s_txRetryTick = 0U;
    s_txRetryPending = false;

    for (int p = 0; p < (int)CRTP_NBR_OF_PORTS; p++) {
        s_rxHead[p] = 0U;
        s_rxTail[p] = 0U;
        s_rxCount[p] = 0U;
        s_rxQueueCreated[p] = false;
        s_portCallbacks[p] = NULL;
    }

    s_currentLink = &s_nopLink;
    s_crtpError = false;
    s_lastStatsTick = 0U;
    s_rxPacketCounter = 0U;
    s_txPacketCounter = 0U;
    crtpRxRate = 0U;
    crtpTxRate = 0U;
    s_crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port)
{
    if (port >= CRTP_NBR_OF_PORTS) return;
    if (s_rxQueueCreated[port]) {
        s_crtpError = true;
        return;
    }
    s_rxHead[port] = 0U;
    s_rxTail[port] = 0U;
    s_rxCount[port] = 0U;
    s_rxQueueCreated[port] = true;
}

bool crtpSendPacket(const CrtpPacket *packet)
{
    if (packet == NULL) return false;
    if (s_txCount >= CRTP_TX_QUEUE_SIZE) return false;

    s_txQueue[s_txTail] = *packet;
    s_txTail = (uint16_t)((s_txTail + 1U) % CRTP_TX_QUEUE_SIZE);
    s_txCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet)
{
    if (packet == NULL) return false;
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet)
{
    if (port >= CRTP_NBR_OF_PORTS || packet == NULL) return false;
    if (s_rxCount[port] == 0U) return false;

    *packet = s_rxQueue[port][s_rxHead[port]];
    s_rxHead[port] = (uint8_t)((s_rxHead[port] + 1U) % CRTP_RX_QUEUE_SIZE);
    s_rxCount[port]--;
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
    if (s_currentLink == &s_nopLink || s_currentLink->receivePacket == NULL) return;

    CrtpPacket packet;
    if (!s_currentLink->receivePacket(&packet)) return;

    s_rxPacketCounter++;

    uint8_t port = packet.port;
    if (port < CRTP_NBR_OF_PORTS) {
        if (s_rxQueueCreated[port] && s_rxCount[port] < CRTP_RX_QUEUE_SIZE) {
            s_rxQueue[port][s_rxTail[port]] = packet;
            s_rxTail[port] = (uint8_t)((s_rxTail[port] + 1U) % CRTP_RX_QUEUE_SIZE);
            s_rxCount[port]++;
        }

        if (s_portCallbacks[port] != NULL) {
            s_portCallbacks[port](&packet);
        }
    }
}

void crtpTxTask(void)
{
    if (s_currentLink == &s_nopLink || s_currentLink->sendPacket == NULL) return;

    if (s_txRetryPending) {
        if ((g_platformTick - s_txRetryTick) < 10U) return;
        if (s_currentLink->sendPacket(&s_txRetryPacket)) {
            s_txPacketCounter++;
            s_txRetryPending = false;
            s_txRetryTick = 0U;
        }
        return;
    }

    if (s_txCount == 0U) return;

    CrtpPacket packet = s_txQueue[s_txHead];
    if (s_currentLink->sendPacket(&packet)) {
        s_txHead = (uint16_t)((s_txHead + 1U) % CRTP_TX_QUEUE_SIZE);
        s_txCount--;
        s_txPacketCounter++;
    } else {
        s_txRetryPacket = packet;
        s_txRetryTick = g_platformTick;
        s_txRetryPending = true;
    }
}

void crtpSetLink(CrtpLink *newLink)
{
    if (s_currentLink != NULL && s_currentLink->setEnable != NULL) {
        s_currentLink->setEnable(false);
    }

    if (newLink == NULL) {
        s_currentLink = &s_nopLink;
    } else {
        s_currentLink = newLink;
        if (s_currentLink->setEnable != NULL) {
            s_currentLink->setEnable(true);
        }
    }
}

void crtpReset(void)
{
    s_txHead = 0U;
    s_txTail = 0U;
    s_txCount = 0U;
    s_txRetryPending = false;
    s_txRetryTick = 0U;

    if (s_currentLink != NULL && s_currentLink->reset != NULL) {
        s_currentLink->reset();
    }
}

bool crtpIsConnected(void)
{
    if (s_currentLink != NULL && s_currentLink->isConnected != NULL) {
        return s_currentLink->isConnected();
    }
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void)
{
    return (uint32_t)(CRTP_TX_QUEUE_SIZE - s_txCount);
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback)
{
    if (port >= CRTP_NBR_OF_PORTS) return;
    s_portCallbacks[port] = callback;
}

void updateStats(void)
{
    if (s_lastStatsTick == 0U) {
        s_lastStatsTick = g_platformTick;
        return;
    }

    if ((g_platformTick - s_lastStatsTick) >= 500U) {
        crtpRxRate = s_rxPacketCounter;
        crtpTxRate = s_txPacketCounter;
        s_rxPacketCounter = 0U;
        s_txPacketCounter = 0U;
        s_lastStatsTick = g_platformTick;
    }
}

/* ================= deck ================= */
uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity)
{
    if (decks == NULL || capacity == 0U) return 0U;

    /* Conservative host model: no mock inventory is frozen, so no decks are discovered. */
    return 0U;
}
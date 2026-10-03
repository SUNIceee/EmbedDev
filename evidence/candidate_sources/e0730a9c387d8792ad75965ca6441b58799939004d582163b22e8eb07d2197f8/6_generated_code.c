#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Explicit scheduler-step to monotonic-millisecond conversion.
 * The 1 kHz baseline means one stabilizer scheduler step is 1 ms.
 * Timeout code below uses milliseconds, not raw step counters. */
#define MS_PER_STABILIZER_TICK 1U

static uint32_t g_currentTickMs = 0U;

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

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

float invSqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    float xhalf = 0.5f * x;
    int32_t i = 0;
    memcpy(&i, &x, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    float y = 0.0f;
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - xhalf * y * y);
    return y;
}

void estimatedGravityDirection(float dw, float dx, float dy, float dz,
                               float *gravX, float *gravY, float *gravZ)
{
    if (!gravX || !gravY || !gravZ) return;
    *gravX = 2.0f * (dx * dz - dw * dy);
    *gravY = 2.0f * (dw * dx + dy * dz);
    *gravZ = dw * dw - dx * dx - dy * dy + dz * dz;
}

static void sensfusion6SyncGravity(void)
{
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
}

static void sensfusion6UpdateLog(void)
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

void sensfusion6Init(void)
{
    if (sensfusion6IsInit) return;
    qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
    sensfusion6IsInit = true;
    sensfusion6IsCalibrated = false;
    sensfusion6SyncGravity();
    sensfusion6UpdateLog();
}

bool sensfusion6Test(void)
{
    return sensfusion6IsInit;
}

void sensfusion6UpdateQ(float gxm, float gym, float gzm,
                        float axm, float aym, float azm, float dt)
{
    if (!sensfusion6IsInit) sensfusion6Init();
    if (dt <= 0.0f) return;

    float gxr = gxm, gyr = gym, gzr = gzm;
    float axn = axm, ayn = aym, azn = azm;
    bool accValid = (axn != 0.0f || ayn != 0.0f || azn != 0.0f);

    if (accValid && !sensfusion6IsCalibrated) {
        sensfusion6SyncGravity();
        baseZacc = axn * gravityX + ayn * gravityY + azn * gravityZ;
        sensfusion6IsCalibrated = true;
    }

    if (twoKi == 0.0f) {
        integralFBx = integralFBy = integralFBz = 0.0f;
    }

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
    if (accValid) {
        float recip = invSqrt(axn * axn + ayn * ayn + azn * azn);
        if (recip > 0.0f) {
            float sax = axn * recip, say = ayn * recip, saz = azn * recip;
            gxr += beta * (2.0f * (qx * qz - qw * qy) - sax) * dt;
            gyr += beta * (2.0f * (qw * qx + qy * qz) - say) * dt;
            gzr += beta * (saz - (qw * qw - qx * qx - qy * qy + qz * qz)) * dt;
        }
    }
#else
    if (accValid) {
        float recip = invSqrt(axn * axn + ayn * ayn + azn * azn);
        if (recip > 0.0f) {
            float sax = axn * recip, say = ayn * recip, saz = azn * recip;
            float halfvx = qx * qz - qw * qy;
            float halfvy = qw * qx + qy * qz;
            float halfvz = qw * qw - 0.5f + qz * qz;
            float halfex = say * halfvz - saz * halfvy;
            float halfey = saz * halfvx - sax * halfvz;
            float halfez = sax * halfvy - say * halfvx;
            if (twoKi > 0.0f) {
                integralFBx += twoKi * halfex * dt;
                integralFBy += twoKi * halfey * dt;
                integralFBz += twoKi * halfez * dt;
                gxr += twoKp * halfex + integralFBx;
                gyr += twoKp * halfey + integralFBy;
                gzr += twoKp * halfez + integralFBz;
            } else {
                gxr += twoKp * halfex;
                gyr += twoKp * halfey;
                gzr += twoKp * halfez;
            }
        }
    }
#endif

    float qa = qw, qb = qx, qc = qy;
    float halfdt = 0.5f * dt;
    qw += halfdt * (-qb * gxr - qc * gyr - qz * gzr);
    qx += halfdt * (qa * gxr + qc * gzr - qz * gyr);
    qy += halfdt * (qa * gyr - qb * gzr + qz * gxr);
    qz += halfdt * (qa * gzr + qb * gyr - qc * gxr);

    float norm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm > 0.0f) {
        qw *= norm; qx *= norm; qy *= norm; qz *= norm;
    }
    sensfusion6SyncGravity();
    sensfusion6UpdateLog();
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (!roll_deg || !pitch_deg || !yaw_deg) return;
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
    *roll_deg = roll * (180.0f / M_PI);
    *pitch_deg = pitch * (180.0f / M_PI);
    *yaw_deg = yaw * (180.0f / M_PI);
}

void sensfusion6GetQuaternion(float *outQw, float *outQx, float *outQy, float *outQz)
{
    if (!outQw || !outQx || !outQy || !outQz) return;
    *outQw = qw; *outQx = qx; *outQy = qy; *outQz = qz;
}

float sensfusion6GetAccZ(float axm, float aym, float azm)
{
    sensfusion6SyncGravity();
    return axm * gravityX + aym * gravityY + azm * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float axm, float aym, float azm)
{
    return sensfusion6GetAccZ(axm, aym, azm) - baseZacc;
}

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out)
{
    if (!out) return;
    int32_t r = (int32_t)roll / 2;
    int32_t p = (int32_t)pitch / 2;
    int32_t t = (int32_t)thrust;
    out->m1 = t - r + p + (int32_t)yaw;
    out->m2 = t - r - p - (int32_t)yaw;
    out->m3 = t + r - p + (int32_t)yaw;
    out->m4 = t + r + p - (int32_t)yaw;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4])
{
    if (!motorForces) return;
    float thrustPart = 0.25f * thrustSi;
    float arm = 0.707106781f * armLength;
    float rollPart = (arm != 0.0f) ? (0.25f / arm) * torqueX : 0.0f;
    float pitchPart = (arm != 0.0f) ? (0.25f / arm) * torqueY : 0.0f;
    float yawPart = (thrustToTorque != 0.0f) ? (0.25f / thrustToTorque) * torqueZ : 0.0f;

    /* X-frame signs: M1=T-r+p+y, M2=T-r-p-y, M3=T+r-p+y, M4=T+r+p-y. */
    float m0 = thrustPart - rollPart + pitchPart + yawPart;
    float m1 = thrustPart - rollPart - pitchPart - yawPart;
    float m2 = thrustPart + rollPart - pitchPart + yawPart;
    float m3 = thrustPart + rollPart + pitchPart - yawPart;

    motorForces[0] = m0 > 0.0f ? m0 : 0.0f;
    motorForces[1] = m1 > 0.0f ? m1 : 0.0f;
    motorForces[2] = m2 > 0.0f ? m2 : 0.0f;
    motorForces[3] = m3 > 0.0f ? m3 : 0.0f;
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4])
{
    if (!normalizedForces || !motorPWMs) return;
    for (int i = 0; i < 4; i++) {
        float f = normalizedForces[i];
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
        motorPWMs[i] = (uint16_t)(f * 65535.0f);
    }
}

static uint16_t motorForceToPwm(float force)
{
    if (force <= 0.0f) return 0U;
    if (force >= CRAZYFLIE_MAX_MOTOR_FORCE_N) return 65535U;
    return (uint16_t)(force / CRAZYFLIE_MAX_MOTOR_FORCE_N * 65535.0f);
}

void powerDistribution(const ControlData *control, MotorPower *motorPower)
{
    if (!control || !motorPower) return;
    if (control->controlMode == controlModeLegacy) {
        powerDistributionLegacy(control->thrust, control->roll, control->pitch,
                                control->yaw, motorPower);
    } else if (control->controlMode == controlModeForceTorque) {
        float forces[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        powerDistributionForceTorque(control->thrustSi, control->torque.x,
                                     control->torque.y, control->torque.z,
                                     CRAZYFLIE_ARM_LENGTH_M,
                                     CRAZYFLIE_THRUST_TO_TORQUE, forces);
        motorPower->m1 = (int32_t)motorForceToPwm(forces[0]);
        motorPower->m2 = (int32_t)motorForceToPwm(forces[1]);
        motorPower->m3 = (int32_t)motorForceToPwm(forces[2]);
        motorPower->m4 = (int32_t)motorForceToPwm(forces[3]);
    } else if (control->controlMode == controlModeForce) {
        uint16_t pwms[4] = {0U, 0U, 0U, 0U};
        powerDistributionForce(control->normalizedForces, pwms);
        motorPower->m1 = (int32_t)pwms[0];
        motorPower->m2 = (int32_t)pwms[1];
        motorPower->m3 = (int32_t)pwms[2];
        motorPower->m4 = (int32_t)pwms[3];
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
    if (!motors) return result;

    int32_t maxVal = motors[0];
    for (int i = 1; i < 4; i++) {
        if (motors[i] > maxVal) maxVal = motors[i];
    }
    if (maxVal <= maxAllowedThrust) return result;

    result.isCapped = true;
    result.reduction = maxVal - maxAllowedThrust;
    for (int i = 0; i < 4; i++) {
        motors[i] = capMinThrust(motors[i] - result.reduction, idleThrust);
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
    float scaled = (float)motorThrust * nominalVoltage / actualVoltage;
    float rounded = floorf(scaled + 0.5f);
    if (rounded < 0.0f) rounded = 0.0f;
    if (rounded > 65535.0f) rounded = 65535.0f;
    return (uint16_t)rounded;
}

PidObject pidRoll = {0};
PidObject pidPitch = {0};
PidObject pidYaw = {0};
PidObject pidRollRate = {0};
PidObject pidPitchRate = {0};
PidObject pidYawRate = {0};

static float attitudeDt = 0.002f;
static bool attitudeControllerInitialized = false;

static float pidUpdateWithSetpoint(PidObject *pid, float actual, float desired,
                                   float dt, bool reset)
{
    if (!pid) return 0.0f;
    float error = desired - actual;
    if (reset || !pid->initialized) {
        pid->integral = 0.0f;
        pid->prevError = error;
        pid->initialized = true;
    }
    float derivative = error - pid->prevError;
    pid->integral += error * dt;
    float output = pid->kp * error + pid->ki * pid->integral +
                   pid->kd * derivative + pid->kff * desired;
    pid->prevError = error;
    pid->output = output;
    return output;
}

void attitudeControllerInit(float updateDt)
{
    if (attitudeControllerInitialized) return;
    attitudeControllerInitialized = true;
    attitudeDt = updateDt > 0.0f ? updateDt : 0.002f;
    PidObject zero = {0};
    zero.initialized = true;
    pidRoll = zero; pidPitch = zero; pidYaw = zero;
    pidRollRate = zero; pidPitchRate = zero; pidYawRate = zero;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired)
{
    pidUpdateWithSetpoint(&pidRollRate, rollActual, rollDesired, attitudeDt, false);
    pidRollRate.output = saturateSignedInt16((int32_t)pidRollRate.output);
    pidUpdateWithSetpoint(&pidPitchRate, pitchActual, pitchDesired, attitudeDt, false);
    pidPitchRate.output = saturateSignedInt16((int32_t)pidPitchRate.output);
    pidUpdateWithSetpoint(&pidYawRate, yawActual, yawDesired, attitudeDt, false);
    pidYawRate.output = saturateSignedInt16((int32_t)pidYawRate.output);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired)
{
    pidUpdateWithSetpoint(&pidRoll, rollActual, rollDesired, attitudeDt, false);
    pidUpdateWithSetpoint(&pidPitch, pitchActual, pitchDesired, attitudeDt, false);
    pidUpdateWithSetpoint(&pidYaw, yawActual, yawDesired, attitudeDt, true);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual)
{
    (void)rollActual; (void)pitchActual; (void)yawActual;
    pidRoll.integral = 0.0f; pidRoll.prevError = 0.0f;
    pidRoll.output = 0.0f; pidRoll.initialized = true;
    pidPitch.integral = 0.0f; pidPitch.prevError = 0.0f;
    pidPitch.output = 0.0f; pidPitch.initialized = true;
    pidYaw.integral = 0.0f; pidYaw.prevError = 0.0f;
    pidYaw.output = 0.0f; pidYaw.initialized = true;
    pidRollRate.integral = 0.0f; pidRollRate.prevError = 0.0f;
    pidRollRate.output = 0.0f; pidRollRate.initialized = true;
    pidPitchRate.integral = 0.0f; pidPitchRate.prevError = 0.0f;
    pidPitchRate.output = 0.0f; pidPitchRate.initialized = true;
    pidYawRate.integral = 0.0f; pidYawRate.prevError = 0.0f;
    pidYawRate.output = 0.0f; pidYawRate.initialized = true;
}

void attitudeControllerResetRollAttitudePID(float rollActual)
{
    (void)rollActual;
    pidRoll.integral = 0.0f; pidRoll.prevError = 0.0f;
    pidRoll.output = 0.0f; pidRoll.initialized = true;
}

void attitudeControllerResetPitchAttitudePID(float pitchActual)
{
    (void)pitchActual;
    pidPitch.integral = 0.0f; pidPitch.prevError = 0.0f;
    pidPitch.output = 0.0f; pidPitch.initialized = true;
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw)
{
    if (!roll || !pitch || !yaw) return;
    *roll = (int16_t)pidRollRate.output;
    *pitch = (int16_t)pidPitchRate.output;
    *yaw = (int16_t)pidYawRate.output;
}

static float desiredYawState = 0.0f;
static float posPidIntegral = 0.0f;
static float posPidPrevError = 0.0f;
static bool posPidInitialized = false;

static void positionControllerResetState(void)
{
    posPidIntegral = 0.0f;
    posPidPrevError = 0.0f;
    posPidInitialized = false;
    desiredYawState = 0.0f;
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state)
{
    if (!setpoint || !state) return 0U;

    float error = setpoint->position.z - state->position.z;
    float velError = setpoint->velocity.z - state->velocity.z;

    if (!posPidInitialized) {
        posPidIntegral = 0.0f;
        posPidPrevError = error;
        posPidInitialized = true;
    }

    posPidIntegral += error * attitudeDt;
    float derivative = error - posPidPrevError;
    posPidPrevError = error;

    float output = 60.0f * error + 5.0f * posPidIntegral +
                   20.0f * derivative + 12.0f * velError;
    if (output < 0.0f) output = 0.0f;
    if (output > 65535.0f) output = 65535.0f;
    return (uint16_t)output;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt)
{
    if (!sensors || !setpoint || !state || !control) return;
    attitudeDt = attitudeUpdateDt > 0.0f ? attitudeUpdateDt : attitudeDt;

    float actualRoll = state->attitude.roll;
    float actualPitch = state->attitude.pitch;
    float actualYaw = state->attitude.yaw;

    if (setpoint->mode.z == modeDisable) {
        control->thrust = setpoint->thrust;
    } else {
        control->thrust = positionControllerUpdate(setpoint, state);
    }

    if (control->thrust == 0U) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        attitudeControllerResetAllPID(actualRoll, actualPitch, actualYaw);
        positionControllerResetState();
        desiredYawState = actualYaw;
        return;
    }

    float desiredRollRate = 0.0f;
    float desiredPitchRate = 0.0f;
    float desiredYawRate = 0.0f;

    if (setpoint->mode.roll == modeVelocity) {
        desiredRollRate = setpoint->attitudeRate.roll;
        attitudeControllerResetRollAttitudePID(actualRoll);
    } else {
        float desiredRoll = setpoint->attitude.roll;
        pidUpdateWithSetpoint(&pidRoll, actualRoll, desiredRoll, attitudeDt, false);
        desiredRollRate = pidRoll.output;
    }

    if (setpoint->mode.pitch == modeVelocity) {
        desiredPitchRate = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitchAttitudePID(actualPitch);
    } else {
        float desiredPitch = setpoint->attitude.pitch;
        pidUpdateWithSetpoint(&pidPitch, actualPitch, desiredPitch, attitudeDt, false);
        desiredPitchRate = pidPitch.output;
    }

    if (setpoint->mode.yaw == modeVelocity) {
        desiredYawState += setpoint->attitudeRate.yaw * attitudeDt;
        if (yawMaxDelta != 0.0f) {
            float diff = capAngle(desiredYawState - actualYaw);
            if (diff > yawMaxDelta) desiredYawState = actualYaw + yawMaxDelta;
            if (diff < -yawMaxDelta) desiredYawState = actualYaw - yawMaxDelta;
        }
    } else if (setpoint->mode.yaw == modeAbs) {
        desiredYawState = setpoint->attitude.yaw;
    } else if (setpoint->mode.quat == modeAbs) {
        float sr = 2.0f * (setpoint->attitudeQuaternion.w *
                           setpoint->attitudeQuaternion.x +
                           setpoint->attitudeQuaternion.y *
                           setpoint->attitudeQuaternion.z);
        float cr = 1.0f - 2.0f *
                         (setpoint->attitudeQuaternion.x *
                          setpoint->attitudeQuaternion.x +
                          setpoint->attitudeQuaternion.y *
                          setpoint->attitudeQuaternion.y);
        desiredYawState = atan2f(sr, cr) * (180.0f / M_PI);
    } else {
        desiredYawState = actualYaw;
    }

    pidUpdateWithSetpoint(&pidYaw, actualYaw, desiredYawState, attitudeDt, true);
    desiredYawRate = pidYaw.output;

    float actualRollRate = sensors->gyro.x;
    float actualPitchRate = -sensors->gyro.y;
    float actualYawRate = sensors->gyro.z;

    pidUpdateWithSetpoint(&pidRollRate, actualRollRate, desiredRollRate,
                          attitudeDt, false);
    pidRollRate.output = saturateSignedInt16((int32_t)pidRollRate.output);
    pidUpdateWithSetpoint(&pidPitchRate, actualPitchRate, desiredPitchRate,
                          attitudeDt, false);
    pidPitchRate.output = saturateSignedInt16((int32_t)pidPitchRate.output);
    pidUpdateWithSetpoint(&pidYawRate, actualYawRate, desiredYawRate,
                          attitudeDt, false);
    pidYawRate.output = saturateSignedInt16((int32_t)pidYawRate.output);

    attitudeControllerGetActuatorOutput(&control->roll, &control->pitch,
                                        &control->yaw);
    control->yaw = (int16_t)(-control->yaw);
}

bool thrustLocked = false;
bool commanderModeSet = false;

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime)
{
    if (!rollPrime || !pitchPrime) return;
    float rad = yaw_deg * (M_PI / 180.0f);
    float c = cosf(rad);
    float s = sinf(rad);
    *rollPrime = roll * c - pitch * s;
    *pitchPrime = roll * s + pitch * c;
}

static int commanderPriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t commanderLastUpdateTick = 0U;
static bool commanderHasPriority = false;
static Setpoint commanderSetpointValue;
static bool highLevelTrajectoryActive = false;

bool commanderSetSetpoint(const Setpoint *setpoint, int priority)
{
    if (!setpoint) return false;

    if (priority == COMMANDER_PRIORITY_DISABLE) {
        commanderSetpointValue = *setpoint;
        commanderPriority = priority;
        commanderLastUpdateTick = g_currentTickMs;
        commanderHasPriority = true;
        highLevelTrajectoryActive = false;
        return true;
    }

    if (priority < commanderPriority) return false;

    commanderSetpointValue = *setpoint;
    commanderPriority = priority;
    commanderLastUpdateTick = g_currentTickMs;
    commanderHasPriority = true;

    if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
        highLevelTrajectoryActive = false;
    } else if (priority == COMMANDER_PRIORITY_HIGHLEVEL) {
        highLevelTrajectoryActive = true;
    }
    return true;
}

void commanderRelaxPriority(void)
{
    commanderPriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void)
{
    if (!commanderHasPriority) return 0U;
    if (g_currentTickMs < commanderLastUpdateTick) return 0U;
    return g_currentTickMs - commanderLastUpdateTick;
}

int commanderGetActivePriority(void)
{
    return commanderPriority;
}

static bool altHoldActive = false;

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

    float roll = values->roll;
    float pitch = values->pitch;
    float yaw = values->yaw;
    uint16_t rawThrust = values->thrust;

    if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
    }
    if (rawThrust == 0U) {
        thrustLocked = false;
    }

    if (yawMode == CAREFREE) {
        memset(setpoint, 0, sizeof(*setpoint));
        return;
    }

    if (yawMode == PLUSMODE) {
        rotateYaw(roll, pitch, 45.0f, &roll, &pitch);
    }

    if (posSetMode && rawThrust != 0U) {
        memset(setpoint, 0, sizeof(*setpoint));
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
        setpoint->thrust = 0U;
        return;
    }

    memset(setpoint, 0, sizeof(*setpoint));

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

    if (posHoldMode) {
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->velocity.x = pitch / 30.0f;
        setpoint->velocity.y = roll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
    }

    if (altHoldMode) {
        if (!altHoldActive) {
            altHoldActive = true;
            commanderModeSet = true;
            positionControllerResetState();
        }
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0U;
        setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
    } else {
        if (altHoldActive) {
            altHoldActive = false;
            commanderModeSet = false;
        }
        setpoint->mode.z = modeDisable;
        if (thrustLocked || rawThrust < MIN_THRUST) {
            setpoint->thrust = 0U;
        } else {
            setpoint->thrust = rawThrust > MAX_THRUST ? MAX_THRUST : rawThrust;
        }
    }
}

SupervisorState supervisorState = supervisorStatePreFlChecksNotPassed;
uint32_t supervisorConditionBits = 0U;

static bool supervisorArmed = false;
static bool supervisorCrashed = false;
static bool supervisorTumbled = false;
static bool supervisorIsFlying = false;
static bool supervisorIsFreeFalling = false;
static SensorData supervisorSensors;
static uint32_t supervisorMotorRatios[4] = {0U, 0U, 0U, 0U};
static uint32_t supervisorIdleThrust = 0U;
static int32_t supervisorMotorRPMs[4] = {0, 0, 0, 0};
static float crashDetectionGs = 0.0f;
static float freeFallThreshold = 0.0f;
static float acceptedTiltAccZ = 0.5f;
static float acceptedUpsideDownAccZ = 0.2f;
static uint32_t maxTiltTime = 1000U;
static uint32_t maxUpsideDownTime = 200U;
static bool tumbleCheckEnabled = true;
static bool autoArming = false;
static uint32_t spinupTimeoutDurationMs = 0U; /* not a fixture-specific default */
static uint32_t spinupStartTick = 0U;
static uint32_t preflightTimeoutDurationMs = 30000U;
static uint32_t landingTimeoutDurationMs = 5000U;
static uint32_t latestArmingTick = 0U;
static uint32_t latestLandingTick = 0U;
static int32_t rpmCheckMin = 0;
static int32_t rpmCheckMax = 0;
static bool tiltTimerActive = false;
static bool upsideDownTimerActive = false;
static uint32_t tiltTimerStartTick = 0U;
static uint32_t upsideDownTimerStartTick = 0U;
static bool flightSeen = false;
static uint32_t recentFlightTick = 0U;
static bool motorsNotRespondingActive = false;
static uint32_t motorsNotRespondingStartTick = 0U;
static bool trajectoryFlying = false;
static bool trajectoryFinished = false;
static bool trajectoryDisabled = false;
static bool deckFault = false;

static void setConditionBit(uint32_t mask, bool value)
{
    if (value) supervisorConditionBits |= mask;
    else supervisorConditionBits &= ~mask;
}

void supervisorInit(void)
{
    static bool initialized = false;
    if (initialized) return;
    initialized = true;
    supervisorState = supervisorStatePreFlChecksNotPassed;
    supervisorConditionBits = 0U;
    supervisorArmed = false;
    supervisorCrashed = false;
    supervisorTumbled = false;
    supervisorIsFlying = false;
    supervisorIsFreeFalling = false;
    memset(&supervisorSensors, 0, sizeof(supervisorSensors));
    memset(supervisorMotorRatios, 0, sizeof(supervisorMotorRatios));
    memset(supervisorMotorRPMs, 0, sizeof(supervisorMotorRPMs));
    spinupStartTick = 0U;
    latestArmingTick = 0U;
    latestLandingTick = 0U;
    flightSeen = false;
    recentFlightTick = 0U;
}

void supervisorSetSensorData(const SensorData *sensors)
{
    if (!sensors) return;
    supervisorSensors = *sensors;
    gyro.x = sensors->gyro.x; gyro.y = sensors->gyro.y; gyro.z = sensors->gyro.z;
    acc.x = sensors->acc.x; acc.y = sensors->acc.y; acc.z = sensors->acc.z;
    baro.asl = sensors->baroAsl;
    baro.temp = sensors->baroTemperature;
    baro.pressure = sensors->baroPressure;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust)
{
    if (!motorRatios) return;
    for (int i = 0; i < 4; i++) supervisorMotorRatios[i] = motorRatios[i];
    supervisorIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
    if (!motorRPMs) return;
    for (int i = 0; i < 4; i++) supervisorMotorRPMs[i] = motorRPMs[i];
}

void supervisorConfigureSafety(float gs, float freeFall, float tiltAccZ,
                               float upsideDownAccZ, uint32_t tiltTime,
                               uint32_t upsideDownTime, bool tumbledEnabled)
{
    crashDetectionGs = gs;
    freeFallThreshold = freeFall;
    acceptedTiltAccZ = tiltAccZ;
    acceptedUpsideDownAccZ = upsideDownAccZ;
    maxTiltTime = tiltTime;
    maxUpsideDownTime = upsideDownTime;
    tumbleCheckEnabled = tumbledEnabled;
}

void supervisorConfigureArming(bool autoArm, uint32_t spinupTimeout)
{
    autoArming = autoArm;
    spinupTimeoutDurationMs = spinupTimeout;
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
    return supervisorArmed;
}

bool supervisorIsCrashed(void)
{
    return supervisorCrashed;
}

bool supervisorRequestArming(bool doArm)
{
    if (doArm) {
        if (supervisorArmed && supervisorState == supervisorStateArming) return true;
        if (!supervisorCanArm()) return false;
        supervisorArmed = true;
        supervisorState = supervisorStateArming;
        spinupStartTick = 0U;
        latestArmingTick = 0U;
        return true;
    } else {
        supervisorArmed = false;
        if (supervisorState == supervisorStateArming) {
            supervisorState = supervisorStatePreFlChecksPassed;
        }
        return true;
    }
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
    if (supervisorTumbled) return false;
    if (doRecovery) {
        supervisorCrashed = false;
        return true;
    } else {
        supervisorCrashed = true;
        return true;
    }
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
    uint16_t bits = 0U;
    if (supervisorCanArm()) bits |= (1U << 0);
    if (supervisorArmed) bits |= (1U << 1);
    if (autoArming) bits |= (1U << 2);
    if (supervisorCanFly()) bits |= (1U << 3);
    if (supervisorIsFlying) bits |= (1U << 4);
    if (supervisorTumbled) bits |= (1U << 5);
    if (supervisorState == supervisorStateLocked) bits |= (1U << 6);
    if (supervisorCrashed) bits |= (1U << 7);
    if (trajectoryFlying) bits |= (1U << 8);
    if (trajectoryFinished) bits |= (1U << 9);
    if (trajectoryDisabled) bits |= (1U << 10);
    if (deckFault) bits |= (1U << 11);
    return bits;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick)
{
    if (!motorRatios) return supervisorIsFlying;
    for (int i = 0; i < 4; i++) {
        if (motorRatios[i] > idleThrust) {
            recentFlightTick = currentTick;
            flightSeen = true;
            break;
        }
    }
    if (!flightSeen) {
        supervisorIsFlying = false;
        return false;
    }
    bool flying = currentTick >= recentFlightTick &&
                  (currentTick - recentFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
    supervisorIsFlying = flying;
    return flying;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float gs, float freeFall, float tiltAccZ,
                    float upsideDownAccZ, uint32_t tiltTime,
                    uint32_t upsideDownTime, bool tumbledEnabled,
                    uint32_t currentTick, bool *isFreeFalling)
{
    if (isFreeFalling) *isFreeFalling = false;

    if (gs > 0.0f) {
        float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
        if (fabsf(norm - 1.0f) > gs) {
            supervisorCrashed = true;
        }
    }

    if (freeFall > 0.0f &&
        fabsf(accX) < freeFall &&
        fabsf(accY) < freeFall &&
        fabsf(accZ) < freeFall) {
        if (isFreeFalling) *isFreeFalling = true;
        supervisorIsFreeFalling = true;
        tiltTimerActive = false;
        upsideDownTimerActive = false;
        supervisorTumbled = false;
        return false;
    }
    supervisorIsFreeFalling = false;

    if (!tumbledEnabled) {
        tiltTimerActive = false;
        upsideDownTimerActive = false;
        supervisorTumbled = false;
        return false;
    }

    bool tumbled = false;
    if (accZ < upsideDownAccZ) {
        if (!upsideDownTimerActive) {
            upsideDownTimerActive = true;
            upsideDownTimerStartTick = currentTick;
        }
        tiltTimerActive = false;
        if (upsideDownTime > 0U && currentTick >= upsideDownTimerStartTick &&
            currentTick - upsideDownTimerStartTick >= upsideDownTime) {
            tumbled = true;
        }
    } else if (accZ < tiltAccZ) {
        if (!tiltTimerActive) {
            tiltTimerActive = true;
            tiltTimerStartTick = currentTick;
        }
        upsideDownTimerActive = false;
        if (tiltTime > 0U && currentTick >= tiltTimerStartTick &&
            currentTick - tiltTimerStartTick >= tiltTime) {
            tumbled = true;
        }
    } else {
        tiltTimerActive = false;
        upsideDownTimerActive = false;
    }
    supervisorTumbled = tumbled;
    return tumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick)
{
    if (lastNotificationTick == 0U) return true;
    if (currentTick < lastNotificationTick) return true;
    return (currentTick - lastNotificationTick) <=
           DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state, uint32_t latestTick,
                                  uint32_t currentTick, uint32_t timeoutDuration)
{
    if (state != supervisorStateReadyToFly) return false;
    if (latestTick == 0U || timeoutDuration == 0U) return false;
    if (currentTick < latestTick) return false;
    return (currentTick - latestTick) >= timeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestTick, uint32_t currentTick,
                                uint32_t timeoutDuration)
{
    if (latestTick == 0U || timeoutDuration == 0U) return false;
    if (currentTick < latestTick) return false;
    return (currentTick - latestTick) >= timeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed)
{
    setConditionBit(SUPERVISOR_CB_EMERGENCY_STOP,
                    crtpEmergencyStop || paramEmergencyStop ||
                    emergencyStopWatchdogFailed);
    setConditionBit(SUPERVISOR_CB_ARMED, supervisorArmed);
    setConditionBit(SUPERVISOR_CB_IS_FLYING, supervisorIsFlying);
    setConditionBit(SUPERVISOR_CB_IS_TUMBLED, supervisorTumbled);
    setConditionBit(SUPERVISOR_CB_CRASHED, supervisorCrashed);
    setConditionBit(SUPERVISOR_CB_FREE_FALL, supervisorIsFreeFalling);
    setConditionBit(SUPERVISOR_CB_DECK_FAULT, deckFault);
    return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t conditionBits,
                                SupervisorState state)
{
    if (!setpoint) return;

    uint32_t safetyMask = SUPERVISOR_CB_EMERGENCY_STOP |
                          SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT |
                          SUPERVISOR_CB_COMMANDER_WDT_WARNING |
                          SUPERVISOR_CB_IS_TUMBLED |
                          SUPERVISOR_CB_FREE_FALL |
                          SUPERVISOR_CB_CRASHED |
                          SUPERVISOR_CB_MOTORS_NOT_RESPONDING;

    if (conditionBits & safetyMask) {
        memset(setpoint, 0, sizeof(*setpoint));
        return;
    }

    if (state == supervisorStateArming ||
        state == supervisorStateReadyToFly ||
        state == supervisorStateFlying ||
        state == supervisorStateLanded) {
        return;
    }

    if (state == supervisorStateWarningLevelOut) {
        Setpoint original = *setpoint;
        memset(setpoint, 0, sizeof(*setpoint));
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->attitudeRate.yaw = 0.0f;
        setpoint->mode.z = original.mode.z;
        setpoint->position.z = original.position.z;
        setpoint->velocity.z = original.velocity.z;
        setpoint->thrust = original.thrust;
        return;
    }

    memset(setpoint, 0, sizeof(*setpoint));
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax)
{
    if (!motorRPMs) return false;
    if (rpmCheckMin > rpmCheckMax) return false;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTick)
{
    if (!motorRPMs) return false;
    if (!canFly) {
        motorsNotRespondingActive = false;
        return false;
    }
    bool allLow = true;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] >= rpmThreshold) {
            allLow = false;
            break;
        }
    }
    if (!allLow) {
        motorsNotRespondingActive = false;
        return false;
    }
    if (!motorsNotRespondingActive) {
        motorsNotRespondingActive = true;
        motorsNotRespondingStartTick = currentTick;
    }
    if (rpmCheckDurationMs == 0U) return true;
    return currentTick >= motorsNotRespondingStartTick &&
           (currentTick - motorsNotRespondingStartTick) >= rpmCheckDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep)
{
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

    uint32_t currentTick = stabilizerStep * MS_PER_STABILIZER_TICK;
    g_currentTickMs = currentTick;

    SupervisorState oldState = supervisorState;

    supervisorIsFlying = isFlyingCheck(supervisorMotorRatios, supervisorIdleThrust,
                                       currentTick);

    bool freefall = false;
    supervisorTumbled = isTumbledCheck(supervisorSensors.acc.x,
                                       supervisorSensors.acc.y,
                                       supervisorSensors.acc.z,
                                       crashDetectionGs,
                                       freeFallThreshold,
                                       acceptedTiltAccZ,
                                       acceptedUpsideDownAccZ,
                                       maxTiltTime,
                                       maxUpsideDownTime,
                                       tumbleCheckEnabled,
                                       currentTick,
                                       &freefall);
    supervisorIsFreeFalling = freefall;

    if (supervisorIsFreeFalling) {
        supervisorState = supervisorStateExceptFreeFall;
    } else if (supervisorCrashed || supervisorTumbled) {
        supervisorState = supervisorStateCrashed;
    }

    uint32_t inactivity = commanderGetInactivityTime();
    if (inactivity > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
        setConditionBit(SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT, true);
        setConditionBit(SUPERVISOR_CB_COMMANDER_WDT_WARNING, false);
    } else if (inactivity > COMMANDER_WDT_TIMEOUT_STABILIZE) {
        setConditionBit(SUPERVISOR_CB_COMMANDER_WDT_WARNING, true);
        setConditionBit(SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT, false);
    } else {
        setConditionBit(SUPERVISOR_CB_COMMANDER_WDT_WARNING, false);
        setConditionBit(SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT, false);
    }

    if (supervisorState == supervisorStateReadyToFly) {
        if (latestArmingTick == 0U) latestArmingTick = currentTick;
        setConditionBit(SUPERVISOR_CB_PREFLIGHT_TIMEOUT,
                        supervisorIsPreflightTimeout(supervisorState,
                                                     latestArmingTick,
                                                     currentTick,
                                                     preflightTimeoutDurationMs));
    } else {
        setConditionBit(SUPERVISOR_CB_PREFLIGHT_TIMEOUT, false);
        latestArmingTick = 0U;
    }

    if (supervisorState == supervisorStateLanded) {
        if (latestLandingTick == 0U) latestLandingTick = currentTick;
        setConditionBit(SUPERVISOR_CB_LANDING_TIMEOUT,
                        supervisorIsLandingTimeout(latestLandingTick,
                                                   currentTick,
                                                   landingTimeoutDurationMs));
    } else {
        setConditionBit(SUPERVISOR_CB_LANDING_TIMEOUT, false);
        latestLandingTick = 0U;
    }

    if (supervisorState == supervisorStateArming) {
        if (spinupStartTick == 0U) spinupStartTick = currentTick;
        if (spinupTimeoutDurationMs != 0U) {
            setConditionBit(SUPERVISOR_CB_SPINUP_TIMEOUT,
                            currentTick >= spinupStartTick &&
                            (currentTick - spinupStartTick) >=
                            spinupTimeoutDurationMs);
        } else {
            setConditionBit(SUPERVISOR_CB_SPINUP_TIMEOUT, false);
        }

        if (rpmCheckMin < rpmCheckMax) {
            setConditionBit(SUPERVISOR_CB_RPM_AT_ARMING_VALID,
                            isRPMatArmingValid(supervisorMotorRPMs,
                                               rpmCheckMin, rpmCheckMax));
        } else {
            setConditionBit(SUPERVISOR_CB_RPM_AT_ARMING_VALID, false);
        }
    } else {
        setConditionBit(SUPERVISOR_CB_SPINUP_TIMEOUT, false);
        setConditionBit(SUPERVISOR_CB_RPM_AT_ARMING_VALID, false);
        spinupStartTick = 0U;
    }

    bool motorsNR = isMotorsNotResponding(supervisorMotorRPMs, 0,
                                          1000U, supervisorCanFly(),
                                          currentTick);
    setConditionBit(SUPERVISOR_CB_MOTORS_NOT_RESPONDING, motorsNR);

    bool oldAllowed = (oldState == supervisorStateArming ||
                       oldState == supervisorStateReadyToFly ||
                       oldState == supervisorStateFlying ||
                       oldState == supervisorStateWarningLevelOut ||
                       oldState == supervisorStateLanded);
    bool newAllowed = (supervisorState == supervisorStateArming ||
                       supervisorState == supervisorStateReadyToFly ||
                       supervisorState == supervisorStateFlying ||
                       supervisorState == supervisorStateWarningLevelOut ||
                       supervisorState == supervisorStateLanded);
    if (oldAllowed && !newAllowed) supervisorArmed = false;

    if (autoArming && supervisorState == supervisorStatePreFlChecksPassed &&
        oldState != supervisorStatePreFlChecksPassed) {
        supervisorRequestArming(true);
    }

    setConditionBit(SUPERVISOR_CB_ARMED, supervisorArmed);
    setConditionBit(SUPERVISOR_CB_IS_FLYING, supervisorIsFlying);
    setConditionBit(SUPERVISOR_CB_IS_TUMBLED, supervisorTumbled);
    setConditionBit(SUPERVISOR_CB_CRASHED, supervisorCrashed);
    setConditionBit(SUPERVISOR_CB_FREE_FALL, supervisorIsFreeFalling);
    setConditionBit(SUPERVISOR_CB_DECK_FAULT, deckFault);

    supervisorLog.info = supervisorConditionBits;
    supervisorLog.accNorm = sqrtf(supervisorSensors.acc.x * supervisorSensors.acc.x +
                                  supervisorSensors.acc.y * supervisorSensors.acc.y +
                                  supervisorSensors.acc.z * supervisorSensors.acc.z);
}

static EstimatorMeasurement estimatorFifo[16];
static uint8_t estimatorHead = 0U;
static uint8_t estimatorTail = 0U;
static uint8_t estimatorCount = 0U;
static EstimatorMeasurement lastGyro, lastAcc, lastBaro, lastTof;
static State estimatorState;

bool estimatorEnqueue(const EstimatorMeasurement *measurement)
{
    if (!measurement) return false;
    if (estimatorCount >= 16U) return false;
    estimatorFifo[estimatorTail] = *measurement;
    estimatorTail = (uint8_t)((estimatorTail + 1U) % 16U);
    estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement)
{
    if (!measurement) return false;
    if (estimatorCount == 0U) return false;
    *measurement = estimatorFifo[estimatorHead];
    estimatorHead = (uint8_t)((estimatorHead + 1U) % 16U);
    estimatorCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep)
{
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        if (m.type == MeasurementTypeGyroscope) lastGyro = m;
        else if (m.type == MeasurementTypeAcceleration) lastAcc = m;
        else if (m.type == MeasurementTypeBarometer) lastBaro = m;
        else if (m.type == MeasurementTypeTOF) lastTof = m;
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        float dt = 1.0f / 250.0f;
        sensfusion6UpdateQ(lastGyro.data[0], lastGyro.data[1], lastGyro.data[2],
                           lastAcc.data[0], lastAcc.data[1], lastAcc.data[2], dt);
        sensfusion6GetQuaternion(&estimatorState.attitudeQuaternion.w,
                                 &estimatorState.attitudeQuaternion.x,
                                 &estimatorState.attitudeQuaternion.y,
                                 &estimatorState.attitudeQuaternion.z);
        sensfusion6GetEulerRPY(&estimatorState.attitude.roll,
                               &estimatorState.attitude.pitch,
                               &estimatorState.attitude.yaw);
        estimatorState.acc.x = lastAcc.data[0];
        estimatorState.acc.y = lastAcc.data[1];
        float verticalAcc = sensfusion6GetAccZWithoutGravity(lastAcc.data[0],
                                                            lastAcc.data[1],
                                                            lastAcc.data[2]);
        estimatorState.acc.z = verticalAcc;
        estimatorState.velocity.z += verticalAcc * dt;

        stateEstimate.roll = estimatorState.attitude.roll;
        stateEstimate.pitch = estimatorState.attitude.pitch;
        stateEstimate.yaw = estimatorState.attitude.yaw;
        stateEstimate.qx = estimatorState.attitudeQuaternion.x;
        stateEstimate.qy = estimatorState.attitudeQuaternion.y;
        stateEstimate.qz = estimatorState.attitudeQuaternion.z;
        stateEstimate.qw = estimatorState.attitudeQuaternion.w;
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        float dt = 1.0f / 100.0f;
        estimatorState.position.x += estimatorState.velocity.x * dt;
        estimatorState.position.y += estimatorState.velocity.y * dt;
        estimatorState.position.z += estimatorState.velocity.z * dt;
    }
}

static Setpoint stabilizerHighLevelSetpoint;
static bool stabilizerHasHighLevelSetpoint = false;
static bool stabilizerStarted = true;
static bool sensorsCalibrated = true;
static float stabilizerBatteryVoltage = 4.2f;
static float stabilizerFilteredBatteryVoltage = 4.2f;

static void sensorsInit(void) {}
static void stateEstimatorInit(void) {}
static void controllerInit(void) {}
static void powerDistributionInit(void) {}
static void motorsInit(void) {}
static void collisionAvoidanceInit(void) {}
static void sensorsWaitDataReady(void) {}
static void sensorsAcquire(SensorData *sensors)
{
    if (sensors) memset(sensors, 0, sizeof(*sensors));
}

static void stateEstimator(SensorData *sensors, State *state)
{
    (void)sensors;
    if (state) {
        state->attitude = estimatorState.attitude;
        state->attitudeQuaternion = estimatorState.attitudeQuaternion;
        state->position = estimatorState.position;
        state->velocity = estimatorState.velocity;
        state->acc = estimatorState.acc;
    }
    stateEstimate.roll = estimatorState.attitude.roll;
    stateEstimate.pitch = estimatorState.attitude.pitch;
    stateEstimate.yaw = estimatorState.attitude.yaw;
    stateEstimate.qx = estimatorState.attitudeQuaternion.x;
    stateEstimate.qy = estimatorState.attitudeQuaternion.y;
    stateEstimate.qz = estimatorState.attitudeQuaternion.z;
    stateEstimate.qw = estimatorState.attitudeQuaternion.w;
}

static void commanderGetSetpoint(Setpoint *setpoint)
{
    if (setpoint) *setpoint = commanderSetpointValue;
}

static void collisionAvoidanceUpdateSetpoint(Setpoint *setpoint)
{
    (void)setpoint;
}

static void setMotorRatios(const MotorPower *motorPower)
{
    if (!motorPower) return;
    motor.m1req = (uint16_t)(motorPower->m1 > 0 ? motorPower->m1 : 0);
    motor.m2req = (uint16_t)(motorPower->m2 > 0 ? motorPower->m2 : 0);
    motor.m3req = (uint16_t)(motorPower->m3 > 0 ? motorPower->m3 : 0);
    motor.m4req = (uint16_t)(motorPower->m4 > 0 ? motorPower->m4 : 0);
}

void stabilizerInit(void)
{
    static bool initialized = false;
    if (initialized) return;
    initialized = true;
    sensorsInit();
    stateEstimatorInit();
    controllerInit();
    powerDistributionInit();
    motorsInit();
    collisionAvoidanceInit();
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint)
{
    if (!setpoint) return false;
    stabilizerHighLevelSetpoint = *setpoint;
    stabilizerHasHighLevelSetpoint = true;
    return true;
}

void stabilizerTask(void)
{
    static uint32_t stabilizerStep = 0U;
    stabilizerInit();
    if (!stabilizerStarted || !sensorsCalibrated) return;

    uint32_t step = stabilizerStep++;
    g_currentTickMs = step * MS_PER_STABILIZER_TICK;

    SensorData sensors;
    sensorsWaitDataReady();
    sensorsAcquire(&sensors);
    supervisorSetSensorData(&sensors);

    if (healthShallWeRunTest()) {
        healthRunTests(&sensors);
        return;
    }

    estimatorComplementary(step);
    State state;
    stateEstimator(&sensors, &state);

    Setpoint setpoint;
    commanderGetSetpoint(&setpoint);

    supervisorUpdate(step);

    if (stabilizerHasHighLevelSetpoint) {
        commanderSetSetpoint(&stabilizerHighLevelSetpoint,
                            COMMANDER_PRIORITY_HIGHLEVEL);
        stabilizerHasHighLevelSetpoint = false;
        commanderGetSetpoint(&setpoint);
    }

    collisionAvoidanceUpdateSetpoint(&setpoint);
    supervisorOverrideSetpoint(&setpoint, supervisorConditionBits, supervisorState);

    if (!supervisorCanFly()) {
        MotorPower zero = {0, 0, 0, 0};
        setMotorRatios(&zero);
        return;
    }

    ControlData control;
    memset(&control, 0, sizeof(control));
    control.controlMode = controlModeLegacy;
    controllerPid(&sensors, &setpoint, &state, &control, 0.0f,
                  1.0f / 500.0f);

    MotorPower motorPower;
    powerDistribution(&control, &motorPower);

    stabilizerFilteredBatteryVoltage =
        batteryCompensation(stabilizerBatteryVoltage,
                            stabilizerFilteredBatteryVoltage, 0.01f);
    int32_t rawMotors[4] = {motorPower.m1, motorPower.m2,
                            motorPower.m3, motorPower.m4};
    for (int i = 0; i < 4; i++) {
        int32_t raw = rawMotors[i];
        uint16_t pwm = raw < 0 ? 0U : (raw > 65535 ? 65535U : (uint16_t)raw);
        uint16_t compensated = motorsCompensateBatteryVoltage(
            pwm, 4.2f, stabilizerFilteredBatteryVoltage);
        rawMotors[i] = (int32_t)compensated;
    }
    motorPower.m1 = rawMotors[0]; motorPower.m2 = rawMotors[1];
    motorPower.m3 = rawMotors[2]; motorPower.m4 = rawMotors[3];

    int32_t capped[4] = {motorPower.m1, motorPower.m2,
                         motorPower.m3, motorPower.m4};
    powerDistributionCap(capped, 65535, 0);
    motorPower.m1 = capped[0]; motorPower.m2 = capped[1];
    motorPower.m3 = capped[2]; motorPower.m4 = capped[3];

    if (!supervisorAreMotorsAllowedToRun()) {
        motorPower.m1 = motorPower.m2 = motorPower.m3 = motorPower.m4 = 0;
    }
    setMotorRatios(&motorPower);
}

static uint32_t quatcompress(float qx, float qy, float qz, float qw)
{
    int16_t ix = (int16_t)(qx * 1000.0f);
    int16_t iy = (int16_t)(qy * 1000.0f);
    int16_t iz = (int16_t)(qz * 1000.0f);
    int16_t iw = (int16_t)(qw * 1000.0f);
    uint32_t packed = ((uint32_t)(uint16_t)ix << 16) | (uint16_t)iy;
    packed ^= ((uint32_t)(uint16_t)iz << 8) | (uint16_t)iw;
    return packed;
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
    output->gyro_millirad_s[0] = sensors->gyro.x * (M_PI / 180.0f) * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * (M_PI / 180.0f) * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * (M_PI / 180.0f) * 1000.0f;
    output->quatCompressed = quatcompress(state->attitudeQuaternion.x,
                                          state->attitudeQuaternion.y,
                                          state->attitudeQuaternion.z,
                                          state->attitudeQuaternion.w);
}

bool rateSupervisorValidate(uint32_t measuredRate)
{
    return measuredRate >= 997U && measuredRate <= 1003U;
}

static uint32_t rateSupervisorWaitStartTick = 0U;
static bool rateSupervisorWaitActive = false;
static bool rateSupervisorError = false;

void rateSupervisorTask(void)
{
    if (!rateSupervisorWaitActive) {
        rateSupervisorWaitStartTick = g_currentTickMs;
        rateSupervisorWaitActive = true;
        return;
    }

    if (g_currentTickMs >= rateSupervisorWaitStartTick &&
        (g_currentTickMs - rateSupervisorWaitStartTick) < 2000U) {
        return;
    }

    /* Timeout. In the host environment sensors are assumed active, so this
     * transitions to an observable error state. */
    rateSupervisorWaitActive = false;
    rateSupervisorError = true;
}

TestState healthTestState = testDone;
uint8_t motorPass = 0U;
uint8_t batteryPass = 0U;
float batterySag = 0.0f;

static bool propTestRequested = false;
static bool batteryTestRequested = false;
static float healthBatteryVoltage = 4.2f;
static float healthIdleVoltage = 4.2f;
static float healthMinLoadedVoltage = 4.2f;
static uint32_t healthTick = 0U;
static uint32_t healthFailureCount = 0U;
static float propAccSamples[PROPTEST_NBR_OF_VARIANCE_VALUES];
static uint32_t propAccCount = 0U;
static float propNoiseVariance = 0.0f;
static uint32_t propMotorIndex = 0U;
static uint32_t healthMotorTestCount = 0U;
static uint32_t healthRestartStartTick = 0U;

void healthRequestPropTest(void)
{
    propTestRequested = true;
}

void healthRequestBatteryTest(void)
{
    batteryTestRequested = true;
}

bool healthShallWeRunTest(void)
{
    if (propTestRequested) {
        propTestRequested = false;
        healthTestState = configureAcc;
        healthTick = 0U;
        propAccCount = 0U;
        propNoiseVariance = 0.0f;
        propMotorIndex = 0U;
        healthFailureCount = 0U;
        healthMotorTestCount = 0U;
        motorPass = 0U;
        batteryPass = 0U;
        batterySag = 0.0f;
        healthRestartStartTick = 0U;
        return true;
    }
    if (batteryTestRequested) {
        batteryTestRequested = false;
        healthTestState = testBattery;
        healthTick = 0U;
        batterySag = 0.0f;
        batteryPass = 0U;
        healthIdleVoltage = healthBatteryVoltage;
        healthMinLoadedVoltage = healthBatteryVoltage;
        healthRestartStartTick = 0U;
        return true;
    }
    return healthTestState != testDone;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex)
{
    if (highThreshold == 0.0f) {
        motorPass |= (uint8_t)(1U << (motorIndex & 3U));
        return true;
    }
    if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
        motorPass |= (uint8_t)(1U << (motorIndex & 3U));
        return true;
    }
    healthFailureCount++;
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
    float n = (float)length;
    return sumSq - (sum * sum / n);
}

void healthRunTests(const SensorData *sensorData)
{
    if (!sensorData) return;

    if (sensorData->tofRange > 0.0f) healthBatteryVoltage = sensorData->tofRange;

    switch (healthTestState) {
    case configureAcc:
        motorPass = 0U;
        batteryPass = 0U;
        batterySag = 0.0f;
        healthTick = 0U;
        propAccCount = 0U;
        propNoiseVariance = 0.0f;
        propMotorIndex = 0U;
        healthFailureCount = 0U;
        healthMotorTestCount = 0U;
        healthIdleVoltage = healthBatteryVoltage;
        healthTestState = measureNoiseFloor;
        break;
    case measureNoiseFloor:
        if (propAccCount < PROPTEST_NBR_OF_VARIANCE_VALUES) {
            float mag = sqrtf(sensorData->acc.x * sensorData->acc.x +
                              sensorData->acc.y * sensorData->acc.y +
                              sensorData->acc.z * sensorData->acc.z);
            propAccSamples[propAccCount++] = mag;
        }
        if (propAccCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
            propNoiseVariance = variance(propAccSamples,
                                         (int)PROPTEST_NBR_OF_VARIANCE_VALUES);
            propMotorIndex = 0U;
            healthTestState = measureProp;
        }
        break;
    case measureProp:
        if (propMotorIndex < 4U) {
            float measuredVibration = sqrtf(sensorData->acc.x * sensorData->acc.x +
                                            sensorData->acc.y * sensorData->acc.y +
                                            sensorData->acc.z * sensorData->acc.z);
            evaluatePropTest(0.0f, 1540.0f, measuredVibration,
                             (uint8_t)propMotorIndex);
            healthMotorTestCount++;
            propMotorIndex++;
        }
        if (propMotorIndex >= 4U) {
            healthTestState = evaluatePropResult;
        }
        break;
    case evaluatePropResult:
        healthTestState = testDone;
        break;
    case testBattery:
        healthTick++;
        if (healthTick == 1U) {
            healthMinLoadedVoltage = healthBatteryVoltage;
        } else if (healthTick >= 2U && healthTick <= 49U) {
            if (healthBatteryVoltage < healthMinLoadedVoltage) {
                healthMinLoadedVoltage = healthBatteryVoltage;
            }
        } else if (healthTick >= 50U) {
            batterySag = healthIdleVoltage - healthMinLoadedVoltage;
            healthTestState = evaluateBatResult;
        }
        break;
    case evaluateBatResult:
        batteryPass = batterySag > 1.0f ? 0U : 1U;
        healthTestState = testDone;
        break;
    case restartBatTest:
        if (healthRestartStartTick == 0U) {
            healthRestartStartTick = g_currentTickMs;
        }
        if (g_currentTickMs >= healthRestartStartTick &&
            (g_currentTickMs - healthRestartStartTick) >= 2000U) {
            batteryTestRequested = true;
            healthTestState = testDone;
            healthRestartStartTick = 0U;
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

static bool crtpNopSend(CrtpPacket *packet)
{
    (void)packet;
    return false;
}

static bool crtpNopReceive(CrtpPacket *packet)
{
    (void)packet;
    return false;
}

static bool crtpNopIsConnected(void)
{
    return true;
}

static void crtpNopSetEnable(bool enable)
{
    (void)enable;
}

static void crtpNopReset(void)
{
}

static CrtpLink nopLink = {
    crtpNopSend,
    crtpNopReceive,
    crtpNopIsConnected,
    crtpNopSetEnable,
    crtpNopReset
};

static CrtpLink *crtpLink = &nopLink;
static CrtpPacket txQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t txHead = 0U;
static uint16_t txTail = 0U;
static uint16_t txCount = 0U;
static bool rxQueueCreated[CRTP_NBR_OF_PORTS];
static CrtpPacket rxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint8_t rxHead[CRTP_NBR_OF_PORTS];
static uint8_t rxTail[CRTP_NBR_OF_PORTS];
static uint8_t rxCount[CRTP_NBR_OF_PORTS];
static CrtpPortCallback portCallbacks[CRTP_NBR_OF_PORTS];
static bool crtpErrorState = false;
static bool crtpTxRetryEverAttempted = false;
static uint32_t crtpLastRetryTick = 0U;
static uint32_t crtpRxPacketCounter = 0U;
static uint32_t crtpTxPacketCounter = 0U;
static uint32_t crtpLastStatsTick = 0U;
static bool crtpStatsInitialized = false;
static uint32_t crtpRxRate = 0U;
static uint32_t crtpTxRate = 0U;

void crtpInit(void)
{
    static bool initialized = false;
    if (initialized) return;
    initialized = true;
    txHead = txTail = txCount = 0U;
    crtpErrorState = false;
    crtpTxRetryEverAttempted = false;
    crtpLastRetryTick = 0U;
    crtpRxPacketCounter = 0U;
    crtpTxPacketCounter = 0U;
    crtpLastStatsTick = 0U;
    crtpStatsInitialized = false;
    crtpRxRate = 0U;
    crtpTxRate = 0U;
    for (uint8_t i = 0; i < CRTP_NBR_OF_PORTS; i++) {
        rxQueueCreated[i] = false;
        rxHead[i] = rxTail[i] = rxCount[i] = 0U;
        portCallbacks[i] = NULL;
    }
    crtpReset();
}

void crtpInitTaskQueue(uint8_t port)
{
    if (port >= CRTP_NBR_OF_PORTS) return;
    if (rxQueueCreated[port]) {
        crtpErrorState = true;
        return;
    }
    rxQueueCreated[port] = true;
    rxHead[port] = rxTail[port] = rxCount[port] = 0U;
}

bool crtpSendPacket(const CrtpPacket *packet)
{
    if (!packet) return false;
    if (txCount >= CRTP_TX_QUEUE_SIZE) return false;
    txQueue[txTail] = *packet;
    txTail = (uint16_t)((txTail + 1U) % CRTP_TX_QUEUE_SIZE);
    txCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet)
{
    if (!packet) return false;
    if (txCount >= CRTP_TX_QUEUE_SIZE) {
        crtpTxTask();
        if (txCount >= CRTP_TX_QUEUE_SIZE) return false;
    }
    return crtpSendPacket(packet);
}

static bool crtpPopTxQueue(CrtpPacket *packet)
{
    if (txCount == 0U || !packet) return false;
    *packet = txQueue[txHead];
    txHead = (uint16_t)((txHead + 1U) % CRTP_TX_QUEUE_SIZE);
    txCount--;
    return true;
}

static bool crtpPopRxQueue(uint8_t port, CrtpPacket *packet)
{
    if (port >= CRTP_NBR_OF_PORTS || !rxQueueCreated[port] ||
        rxCount[port] == 0U || !packet) {
        return false;
    }
    *packet = rxQueues[port][rxHead[port]];
    rxHead[port] = (uint8_t)((rxHead[port] + 1U) % CRTP_RX_QUEUE_SIZE);
    rxCount[port]--;
    return true;
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet)
{
    return crtpPopRxQueue(port, packet);
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
    if (!crtpLink || !crtpLink->receivePacket) return;
    CrtpPacket packet;
    if (!crtpLink->receivePacket(&packet)) return;

    crtpRxPacketCounter++;
    bool delivered = false;
    if (packet.port < CRTP_NBR_OF_PORTS && rxQueueCreated[packet.port]) {
        if (rxCount[packet.port] < CRTP_RX_QUEUE_SIZE) {
            rxQueues[packet.port][rxTail[packet.port]] = packet;
            rxTail[packet.port] =
                (uint8_t)((rxTail[packet.port] + 1U) % CRTP_RX_QUEUE_SIZE);
            rxCount[packet.port]++;
            delivered = true;
        }
    }
    if (packet.port < CRTP_NBR_OF_PORTS && portCallbacks[packet.port]) {
        portCallbacks[packet.port](&packet);
        delivered = true;
    }
    (void)delivered;
}

void crtpTxTask(void)
{
    if (!crtpLink || !crtpLink->sendPacket) return;
    if (txCount == 0U) return;

    bool shouldAttempt = !crtpTxRetryEverAttempted ||
                         (g_currentTickMs >= crtpLastRetryTick &&
                          (g_currentTickMs - crtpLastRetryTick) >= 10U);
    if (!shouldAttempt) return;

    CrtpPacket packet;
    if (!crtpPopTxQueue(&packet)) return;
    if (!crtpLink->sendPacket(&packet)) {
        if (txCount < CRTP_TX_QUEUE_SIZE) {
            txQueue[txTail] = packet;
            txTail = (uint16_t)((txTail + 1U) % CRTP_TX_QUEUE_SIZE);
            txCount++;
        }
        crtpTxRetryEverAttempted = true;
        crtpLastRetryTick = g_currentTickMs;
    } else {
        crtpTxPacketCounter++;
        crtpLastRetryTick = g_currentTickMs;
    }
}

void crtpSetLink(CrtpLink *newLink)
{
    if (crtpLink && crtpLink->setEnable) crtpLink->setEnable(false);
    crtpLink = newLink ? newLink : &nopLink;
    if (crtpLink->setEnable) crtpLink->setEnable(true);
}

void crtpReset(void)
{
    txHead = txTail = txCount = 0U;
    if (crtpLink && crtpLink->reset) crtpLink->reset();
}

bool crtpIsConnected(void)
{
    if (crtpLink && crtpLink->isConnected) return crtpLink->isConnected();
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void)
{
    return CRTP_TX_QUEUE_SIZE - txCount;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback)
{
    if (port >= CRTP_NBR_OF_PORTS) return;
    portCallbacks[port] = callback;
}

void updateStats(void)
{
    if (!crtpStatsInitialized) {
        crtpLastStatsTick = g_currentTickMs;
        crtpStatsInitialized = true;
        return;
    }
    if (g_currentTickMs < crtpLastStatsTick ||
        (g_currentTickMs - crtpLastStatsTick) < 500U) {
        return;
    }
    crtpRxRate = crtpRxPacketCounter;
    crtpTxRate = crtpTxPacketCounter;
    crtpRxPacketCounter = 0U;
    crtpTxPacketCounter = 0U;
    crtpLastStatsTick = g_currentTickMs;
}

static const uint8_t mockI2CDeckAddresses[] = {0x20U, 0x29U, 0x68U, 0x76U};
static const uint64_t mockOneWireDeckIds[] = {
    0x1122334455667788ULL,
    0x8877665544332211ULL
};

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity)
{
    if (!decks || capacity == 0U) return 0U;
    uint8_t count = 0U;

    for (uint8_t i = 0; count < capacity &&
         i < (uint8_t)(sizeof(mockI2CDeckAddresses) /
                       sizeof(mockI2CDeckAddresses[0])); i++) {
        bool already = false;
        for (uint8_t j = 0; j < count; j++) {
            if (decks[j].foundByI2C &&
                decks[j].i2cAddress == mockI2CDeckAddresses[i]) {
                already = true;
                break;
            }
        }
        if (!already) {
            decks[count].foundByI2C = true;
            decks[count].foundByOneWire = false;
            decks[count].i2cAddress = mockI2CDeckAddresses[i];
            decks[count].oneWireRomId = 0ULL;
            count++;
        }
    }

    for (uint8_t i = 0; count < capacity &&
         i < (uint8_t)(sizeof(mockOneWireDeckIds) /
                       sizeof(mockOneWireDeckIds[0])); i++) {
        bool already = false;
        for (uint8_t j = 0; j < count; j++) {
            if (decks[j].foundByOneWire &&
                decks[j].oneWireRomId == mockOneWireDeckIds[i]) {
                already = true;
                break;
            }
        }
        if (!already) {
            decks[count].foundByI2C = false;
            decks[count].foundByOneWire = true;
            decks[count].i2cAddress = 0U;
            decks[count].oneWireRomId = mockOneWireDeckIds[i];
            count++;
        }
    }

    return count;
}

StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0};
Axis3Log acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};
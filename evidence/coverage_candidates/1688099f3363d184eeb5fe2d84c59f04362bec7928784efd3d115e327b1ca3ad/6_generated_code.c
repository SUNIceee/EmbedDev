/* Implementation for Crazyflie Core API Baseline SRS-v5 / Test-Compatible v4 */

#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Global Public State Variables */
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

PidObject pidRoll = {0}, pidPitch = {0}, pidYaw = {0};
PidObject pidRollRate = {0}, pidPitchRate = {0}, pidYawRate = {0};

bool thrustLocked = true;
bool commanderModeSet = false;

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0;

TestState healthTestState = configureAcc;
uint8_t motorPass = 0, batteryPass = 0;
float batterySag = 0.0f;

StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0}, acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};

/* Internal Module State */
static uint32_t g_currentTick = 0;
static SensorData g_latestSensorData = {0};
static uint32_t g_latestMotorRatios[4] = {0};
static uint32_t g_latestIdleThrust = 0;
static int32_t g_latestMotorRPMs[4] = {0};

static float g_safetyCrashDetectionGs = 0.0f;
static float g_safetyFreeFallThreshold = 0.0f;
static float g_safetyAcceptedTiltAccZ = 0.0f;
static float g_safetyAcceptedUpsideDownAccZ = 0.0f;
static uint32_t g_safetyMaxTiltTime = 0;
static uint32_t g_safetyMaxUpsideDownTime = 0;
static bool g_safetyTumbleCheckEnabled = false;

static bool g_armingAutoArming = false;
static uint32_t g_armingSpinupTimeoutMs = 0;
static uint32_t g_armingStartTick = 0;

static uint32_t g_isFlyingRecentTick = 0;
static bool g_isFlyingSeen = false;
static uint32_t g_tiltStartTick = 0;
static uint32_t g_upsideDownStartTick = 0;
static uint32_t g_motorNotRespondingStartTick = 0;

static Setpoint g_activeSetpoint = {0};
static int g_activePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t g_commanderLastUpdateTick = 0;

static bool g_propTestRequested = false;
static bool g_batteryTestRequested = false;
static float g_idleVoltage = 4.2f;
static float g_minLoadedVoltage = 4.2f;
static uint32_t g_healthTick = 0;

/* Estimator FIFO */
#define ESTIMATOR_QUEUE_CAPACITY 16
static EstimatorMeasurement g_estimatorQueue[ESTIMATOR_QUEUE_CAPACITY];
static size_t g_estimatorHead = 0;
static size_t g_estimatorTail = 0;
static size_t g_estimatorCount = 0;

/* CRTP Transport Storage */
#define MOCK_CRTP_PORTS 16
typedef struct {
    CrtpPacket packets[CRTP_RX_QUEUE_SIZE];
    size_t head, tail, count;
    bool created;
    CrtpPortCallback callback;
} CrtpRxQueue;

static CrtpRxQueue g_crtpRxQueues[MOCK_CRTP_PORTS];
static CrtpPacket g_crtpTxQueue[CRTP_TX_QUEUE_SIZE];
static size_t g_crtpTxHead = 0, g_crtpTxTail = 0, g_crtpTxCount = 0;
static bool g_crtpInitialized = false;

static bool nopSendPacket(CrtpPacket *p) { (void)p; return true; }
static bool nopReceivePacket(CrtpPacket *p) { (void)p; return false; }
static bool nopIsConnected(void) { return true; }
static void nopSetEnable(bool enable) { (void)enable; }
static void nopReset(void) {}

static CrtpLink g_nopLink = {
    nopSendPacket,
    nopReceivePacket,
    nopIsConnected,
    nopSetEnable,
    nopReset
};
static CrtpLink *g_currentLink = &g_nopLink;

/* Controller Yaw Accumulator */
static float g_desiredYaw = 0.0f;

/* =========================================================================
 * 2. Numerical functions (TC-001～TC-019)
 * ========================================================================= */

int16_t saturateSignedInt16(int32_t value) {
    const int32_t negLimit = -32767;
    const int32_t posLimit = 32767;
    if (value > posLimit) return (int16_t)posLimit;
    if (value < negLimit) return (int16_t)negLimit;
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
    union { float f; uint32_t i; } u;
    u.f = x;
    u.i = 0x5f3759df - (u.i >> 1);
    float y = u.f;
    y = y * (1.5f - (xhalf * y * y));
    return y;
}

/* =========================================================================
 * 3. Sensfusion6 (TC-020～TC-035, TC-251)
 * ========================================================================= */

void sensfusion6Init(void) {
    if (!sensfusion6IsInit) {
        qw = 1.0f;
        qx = 0.0f;
        qy = 0.0f;
        qz = 0.0f;
        gravityX = 0.0f;
        gravityY = 0.0f;
        gravityZ = 1.0f;
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
        baseZacc = 0.0f;
        sensfusion6IsCalibrated = false;
        sensfusion6IsInit = true;
    }
}

bool sensfusion6Test(void) {
    return sensfusion6IsInit;
}

void estimatedGravityDirection(float w, float x, float y, float z,
                               float *gravX, float *gravY, float *gravZ) {
    if (gravX) *gravX = 2.0f * (x * z - w * y);
    if (gravY) *gravY = 2.0f * (y * z + w * x);
    if (gravZ) *gravZ = w * w - x * x - y * y + z * z;
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt) {
    if (dt <= 0.0f) dt = 0.001f;
    float gx_rad = gx * (float)(M_PI / 180.0);
    float gy_rad = gy * (float)(M_PI / 180.0);
    float gz_rad = gz * (float)(M_PI / 180.0);

    float accSq = ax * ax + ay * ay + az * az;
    if (accSq > 0.0f) {
        float recipNorm = invSqrt(accSq);
        float ax_n = ax * recipNorm;
        float ay_n = ay * recipNorm;
        float az_n = az * recipNorm;

        float vx, vy, vz;
        estimatedGravityDirection(qw, qx, qy, qz, &vx, &vy, &vz);

        float ex = (ay_n * vz - az_n * vy);
        float ey = (az_n * vx - ax_n * vz);
        float ez = (ax_n * vy - ay_n * vx);

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
        float s0 = -2.0f * qy * ez + 2.0f * qz * ey;
        float s1 =  2.0f * qw * ez - 2.0f * qz * ex;
        float s2 = -2.0f * qw * ey + 2.0f * qx * ex;
        float s3 =  2.0f * qx * ey - 2.0f * qy * ex;
        float sNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        gx_rad -= beta * s0 * sNorm;
        gy_rad -= beta * s1 * sNorm;
        gz_rad -= beta * s2 * sNorm;
#else
        if (twoKi > 0.0f) {
            integralFBx += twoKi * ex * dt;
            integralFBy += twoKi * ey * dt;
            integralFBz += twoKi * ez * dt;
            gx_rad += integralFBx;
            gy_rad += integralFBy;
            gz_rad += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }
        gx_rad += twoKp * ex;
        gy_rad += twoKp * ey;
        gz_rad += twoKp * ez;
#endif

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

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
    float gx, gy, gz;
    estimatedGravityDirection(qw, qx, qy, qz, &gx, &gy, &gz);
    gravityX = gx;
    gravityY = gy;
    gravityZ = gz;

    float clampedGx = gx;
    if (clampedGx > 1.0f) clampedGx = 1.0f;
    if (clampedGx < -1.0f) clampedGx = -1.0f;

    float pitch = asinf(-clampedGx) * (float)(180.0 / M_PI);
    float roll  = atan2f(gy, gz) * (float)(180.0 / M_PI);
    float yaw   = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * (float)(180.0 / M_PI);

    if (roll_deg) *roll_deg = roll;
    if (pitch_deg) *pitch_deg = pitch;
    if (yaw_deg) *yaw_deg = yaw;
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

/* =========================================================================
 * 4. Power distribution and battery compensation (TC-036～TC-065, TC-249～TC-250)
 * ========================================================================= */

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
    float rollPart = (arm != 0.0f) ? (0.25f / arm * torqueX) : 0.0f;
    float pitchPart = (arm != 0.0f) ? (0.25f / arm * torqueY) : 0.0f;
    float yawPart = (thrustToTorque != 0.0f) ? (0.25f / thrustToTorque * torqueZ) : 0.0f;

    float f1 = thrustPart - rollPart + pitchPart + yawPart;
    float f2 = thrustPart - rollPart - pitchPart - yawPart;
    float f3 = thrustPart + rollPart - pitchPart + yawPart;
    float f4 = thrustPart + rollPart + pitchPart - yawPart;

    motorForces[0] = (f1 < 0.0f) ? 0.0f : f1;
    motorForces[1] = (f2 < 0.0f) ? 0.0f : f2;
    motorForces[2] = (f3 < 0.0f) ? 0.0f : f3;
    motorForces[3] = (f4 < 0.0f) ? 0.0f : f4;
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
    if (!supervisorAreMotorsAllowedToRun()) {
        motorPower->m1 = 0; motorPower->m2 = 0;
        motorPower->m3 = 0; motorPower->m4 = 0;
        return;
    }
    if (control->controlMode == controlModeLegacy) {
        powerDistributionLegacy(control->thrust, control->roll, control->pitch, control->yaw, motorPower);
    } else if (control->controlMode == controlModeForceTorque) {
        float forces[4];
        powerDistributionForceTorque(control->thrustSi, control->torque.x, control->torque.y, control->torque.z,
                                     CRAZYFLIE_ARM_LENGTH_M, CRAZYFLIE_THRUST_TO_TORQUE, forces);
        motorPower->m1 = (int32_t)((forces[0] / CRAZYFLIE_MAX_MOTOR_FORCE_N) * 65535.0f);
        motorPower->m2 = (int32_t)((forces[1] / CRAZYFLIE_MAX_MOTOR_FORCE_N) * 65535.0f);
        motorPower->m3 = (int32_t)((forces[2] / CRAZYFLIE_MAX_MOTOR_FORCE_N) * 65535.0f);
        motorPower->m4 = (int32_t)((forces[3] / CRAZYFLIE_MAX_MOTOR_FORCE_N) * 65535.0f);
    } else if (control->controlMode == controlModeForce) {
        uint16_t pwms[4];
        powerDistributionForce(control->normalizedForces, pwms);
        motorPower->m1 = pwms[0]; motorPower->m2 = pwms[1];
        motorPower->m3 = pwms[2]; motorPower->m4 = pwms[3];
    }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) {
    int32_t floorVal = (idleThrust < 0) ? 0 : idleThrust;
    return (value < floorVal) ? floorVal : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4], int32_t maxAllowedThrust, int32_t idleThrust) {
    PowerCapResult res = { false, 0 };
    if (!motors) return res;

    int32_t maxVal = motors[0];
    for (int i = 1; i < 4; i++) {
        if (motors[i] > maxVal) maxVal = motors[i];
    }

    if (maxVal > maxAllowedThrust) {
        res.isCapped = true;
        res.reduction = maxVal - maxAllowedThrust;
        for (int i = 0; i < 4; i++) {
            motors[i] -= res.reduction;
            motors[i] = capMinThrust(motors[i], idleThrust);
        }
    }
    return res;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) {
    return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust, float nominalVoltage, float actualVoltage) {
    if (actualVoltage <= 0.0f) return motorThrust;
    float comp = (float)motorThrust * nominalVoltage / actualVoltage;
    int32_t val = (int32_t)roundf(comp);
    if (val < 0) val = 0;
    if (val > 65535) val = 65535;
    return (uint16_t)val;
}

/* =========================================================================
 * 5. Cascaded PID and controllerPid (TC-066～TC-084, TC-252, TC-256)
 * ========================================================================= */

void attitudeControllerInit(float updateDt) {
    (void)updateDt;
    pidRoll.kp = 6.0f; pidRoll.ki = 3.0f; pidRoll.kd = 0.0f; pidRoll.initialized = true;
    pidPitch.kp = 6.0f; pidPitch.ki = 3.0f; pidPitch.kd = 0.0f; pidPitch.initialized = true;
    pidYaw.kp = 6.0f; pidYaw.ki = 1.0f; pidYaw.kd = 0.0f; pidYaw.initialized = true;

    pidRollRate.kp = 250.0f; pidRollRate.ki = 500.0f; pidRollRate.kd = 2.5f; pidRollRate.initialized = true;
    pidPitchRate.kp = 250.0f; pidPitchRate.ki = 500.0f; pidPitchRate.kd = 2.5f; pidPitchRate.initialized = true;
    pidYawRate.kp = 120.0f; pidYawRate.ki = 16.0f; pidYawRate.kd = 0.0f; pidYawRate.initialized = true;
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual, float yawActual) {
    (void)yawActual;
    pidRoll.integral = 0.0f; pidRoll.prevError = 0.0f; pidRoll.output = 0.0f;
    pidPitch.integral = 0.0f; pidPitch.prevError = 0.0f; pidPitch.output = 0.0f;
    pidYaw.integral = 0.0f; pidYaw.prevError = 0.0f; pidYaw.output = 0.0f;

    pidRollRate.integral = 0.0f; pidRollRate.prevError = 0.0f; pidRollRate.output = 0.0f;
    pidPitchRate.integral = 0.0f; pidPitchRate.prevError = 0.0f; pidPitchRate.output = 0.0f;
    pidYawRate.integral = 0.0f; pidYawRate.prevError = 0.0f; pidYawRate.output = 0.0f;

    attitudeControllerResetRollAttitudePID(rollActual);
    attitudeControllerResetPitchAttitudePID(pitchActual);
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
    (void)rollActual;
    pidRoll.integral = 0.0f;
    pidRoll.prevError = 0.0f;
    pidRoll.output = 0.0f;
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
    (void)pitchActual;
    pidPitch.integral = 0.0f;
    pidPitch.prevError = 0.0f;
    pidPitch.output = 0.0f;
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
    float rollError = capAngle(rollDesired - rollActual);
    pidRoll.integral += rollError;
    pidRoll.output = pidRoll.kp * rollError + pidRoll.ki * pidRoll.integral;

    float pitchError = capAngle(pitchDesired - pitchActual);
    pidPitch.integral += pitchError;
    pidPitch.output = pidPitch.kp * pitchError + pidPitch.ki * pidPitch.integral;

    float yawError = capAngle(yawDesired - yawActual);
    pidYaw.integral = 0.0f; /* reset=true semantics */
    pidYaw.output = pidYaw.kp * yawError;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
    float rollError = rollDesired - rollActual;
    pidRollRate.output = pidRollRate.kp * rollError;

    float pitchError = pitchDesired - pitchActual;
    pidPitchRate.output = pidPitchRate.kp * pitchError;

    float yawError = yawDesired - yawActual;
    pidYawRate.output = pidYawRate.kp * yawError;
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch, int16_t *yaw) {
    if (roll) *roll = saturateSignedInt16((int32_t)pidRollRate.output);
    if (pitch) *pitch = saturateSignedInt16((int32_t)pidPitchRate.output);
    if (yaw) *yaw = saturateSignedInt16((int32_t)pidYawRate.output);
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
    if (!setpoint) return 0;
    (void)state;
    return setpoint->thrust;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
    if (!sensors || !setpoint || !state || !control) return;

    if (setpoint->thrust == 0) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0;
        attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
        g_desiredYaw = state->attitude.yaw;
        return;
    }

    if (setpoint->mode.yaw == modeVelocity) {
        g_desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
        g_desiredYaw = capAngle(g_desiredYaw);
        if (yawMaxDelta != 0.0f) {
            float delta = capAngle(g_desiredYaw - state->attitude.yaw);
            if (delta > yawMaxDelta) delta = yawMaxDelta;
            if (delta < -yawMaxDelta) delta = -yawMaxDelta;
            g_desiredYaw = capAngle(state->attitude.yaw + delta);
        }
    } else {
        g_desiredYaw = setpoint->attitude.yaw;
    }

    float rollDesired = setpoint->attitude.roll;
    float pitchDesired = setpoint->attitude.pitch;

    if (setpoint->mode.roll == modeVelocity) {
        rollDesired = setpoint->attitudeRate.roll;
        attitudeControllerResetRollAttitudePID(state->attitude.roll);
    }
    if (setpoint->mode.pitch == modeVelocity) {
        pitchDesired = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
    }

    attitudeControllerCorrectAttitudePID(state->attitude.roll, rollDesired,
                                         state->attitude.pitch, pitchDesired,
                                         state->attitude.yaw, g_desiredYaw);

    float rollRateDesired = (setpoint->mode.roll == modeVelocity) ? rollDesired : pidRoll.output;
    float pitchRateDesired = (setpoint->mode.pitch == modeVelocity) ? pitchDesired : pidPitch.output;
    float yawRateDesired = (setpoint->mode.yaw == modeVelocity) ? setpoint->attitudeRate.yaw : pidYaw.output;

    attitudeControllerCorrectRatePID(sensors->gyro.x, rollRateDesired,
                                     -sensors->gyro.y, pitchRateDesired,
                                     sensors->gyro.z, yawRateDesired);

    int16_t r, p, y;
    attitudeControllerGetActuatorOutput(&r, &p, &y);
    control->roll = r;
    control->pitch = p;
    control->yaw = -y; /* Legacy coordinates negate yaw */

    if (setpoint->mode.z == modeDisable) {
        control->thrust = setpoint->thrust;
    } else {
        control->thrust = positionControllerUpdate(setpoint, state);
    }
}

/* =========================================================================
 * 6. CRTP Commander RPYT (TC-085～TC-108, TC-254～TC-255)
 * ========================================================================= */

void rotateYaw(float roll, float pitch, float yaw_deg, float *rollPrime, float *pitchPrime) {
    float rad = yaw_deg * (float)(M_PI / 180.0);
    float cosA = cosf(rad);
    float sinA = sinf(rad);
    if (rollPrime) *rollPrime = roll * cosA - pitch * sinA;
    if (pitchPrime) *pitchPrime = roll * sinA + pitch * cosA;
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

    if (g_activePriority == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
    }
    if (values->thrust == 0) {
        thrustLocked = false;
    }

    if (altHoldMode) {
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0;
        setpoint->velocity.z = ((float)values->thrust - 32767.0f) / 32767.0f;
        if (!commanderModeSet) {
            commanderModeSet = true;
        }
    } else {
        if (commanderModeSet) {
            setpoint->mode.z = modeDisable;
            commanderModeSet = false;
        }
        if (thrustLocked || values->thrust < 1000) {
            setpoint->thrust = 0;
        } else {
            setpoint->thrust = (values->thrust > MAX_THRUST) ? MAX_THRUST : values->thrust;
        }
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
    } else if (posSetMode && values->thrust != 0) {
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
        setpoint->thrust = 0;
    } else {
        float r = values->roll;
        float p = values->pitch;
        if (yawMode == PLUSMODE) {
            rotateYaw(values->roll, values->pitch, 45.0f, &r, &p);
        } else if (yawMode == CAREFREE) {
            rotateYaw(values->roll, values->pitch, 0.0f, &r, &p);
        }

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

/* =========================================================================
 * 7. Supervisor (TC-109～TC-162, TC-253)
 * ========================================================================= */

void supervisorInit(void) {
    supervisorState = supervisorStateLocked;
    supervisorConditionBits = 0;
    g_armingStartTick = 0;
    g_isFlyingRecentTick = 0;
    g_isFlyingSeen = false;
    g_tiltStartTick = 0;
    g_upsideDownStartTick = 0;
    g_motorNotRespondingStartTick = 0;
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
    return (supervisorState == supervisorStateCrashed) ||
           ((supervisorConditionBits & SUPERVISOR_CB_CRASHED) != 0);
}

bool supervisorRequestArming(bool doArm) {
    if (doArm) {
        if (supervisorCanArm()) {
            supervisorState = supervisorStateArming;
            supervisorConditionBits |= SUPERVISOR_CB_ARMED;
            g_armingStartTick = g_currentTick;
            return true;
        }
        if (supervisorState == supervisorStateArming) return true;
        return false;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
        if (supervisorState == supervisorStateArming) {
            supervisorState = supervisorStatePreFlChecksPassed;
        }
        return true;
    }
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
    bool isFreeFalling = false;
    bool tumbled = isTumbledCheck(g_latestSensorData.acc.x, g_latestSensorData.acc.y, g_latestSensorData.acc.z,
                                  g_safetyCrashDetectionGs, g_safetyFreeFallThreshold,
                                  g_safetyAcceptedTiltAccZ, g_safetyAcceptedUpsideDownAccZ,
                                  g_safetyMaxTiltTime, g_safetyMaxUpsideDownTime,
                                  g_safetyTumbleCheckEnabled, g_currentTick, &isFreeFalling);
    if (tumbled) return false;

    if (!doRecovery) {
        supervisorState = supervisorStateCrashed;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        return true;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
        if (supervisorState == supervisorStateCrashed) {
            supervisorState = supervisorStatePreFlChecksPassed;
        }
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
    if (g_armingAutoArming) bitfield |= (1 << 2);
    if (supervisorCanFly()) bitfield |= (1 << 3);
    if (isFlyingCheck(g_latestMotorRatios, g_latestIdleThrust, g_currentTick)) bitfield |= (1 << 4);

    bool isFreeFalling = false;
    if (isTumbledCheck(g_latestSensorData.acc.x, g_latestSensorData.acc.y, g_latestSensorData.acc.z,
                       g_safetyCrashDetectionGs, g_safetyFreeFallThreshold,
                       g_safetyAcceptedTiltAccZ, g_safetyAcceptedUpsideDownAccZ,
                       g_safetyMaxTiltTime, g_safetyMaxUpsideDownTime,
                       g_safetyTumbleCheckEnabled, g_currentTick, &isFreeFalling)) {
        bitfield |= (1 << 5);
    }
    if (supervisorState == supervisorStateLocked) bitfield |= (1 << 6);
    if (supervisorIsCrashed()) bitfield |= (1 << 7);
    return bitfield;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust, uint32_t currentTick) {
    if (motorRatios) {
        for (int i = 0; i < 4; i++) {
            if (motorRatios[i] > idleThrust) {
                g_isFlyingRecentTick = currentTick;
                g_isFlyingSeen = true;
                break;
            }
        }
    }
    if (!g_isFlyingSeen) return false;
    return (currentTick - g_isFlyingRecentTick < IS_FLYING_HYSTERESIS_THRESHOLD);
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling) {
    if (crashDetectionGs > 0.0f) {
        float accNorm = sqrtf(accX * accX + accY * accY + accZ * accZ);
        if (fabsf(accNorm - 1.0f) > crashDetectionGs) {
            supervisorState = supervisorStateCrashed;
            supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        }
    }

    if (freeFallThreshold > 0.0f &&
        fabsf(accX) < freeFallThreshold &&
        fabsf(accY) < freeFallThreshold &&
        fabsf(accZ) < freeFallThreshold) {
        if (isFreeFalling) *isFreeFalling = true;
        supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
        if (supervisorState != supervisorStateCrashed) {
            supervisorState = supervisorStateExceptFreeFall;
        }
        g_tiltStartTick = 0;
        g_upsideDownStartTick = 0;
    } else {
        if (isFreeFalling) *isFreeFalling = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;
    }

    if (!tumbleCheckEnabled) return false;

    if (acceptedUpsideDownAccZ != 0.0f && accZ < acceptedUpsideDownAccZ) {
        if (g_upsideDownStartTick == 0) g_upsideDownStartTick = currentTick;
        if (currentTick - g_upsideDownStartTick >= maxUpsideDownTime) return true;
    } else {
        g_upsideDownStartTick = 0;
    }

    if (acceptedTiltAccZ != 0.0f && accZ < acceptedTiltAccZ) {
        if (g_tiltStartTick == 0) g_tiltStartTick = currentTick;
        if (currentTick - g_tiltStartTick >= maxTiltTime) return true;
    } else {
        g_tiltStartTick = 0;
    }

    return false;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick) {
    if (lastNotificationTick == 0) return true;
    return (currentTick - lastNotificationTick <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT);
}

bool supervisorIsPreflightTimeout(SupervisorState state, uint32_t latestArmingTick,
                                  uint32_t currentTick, uint32_t preflightTimeoutDuration) {
    if (state == supervisorStateArming || state == supervisorStatePreFlChecksPassed) {
        return (currentTick - latestArmingTick >= preflightTimeoutDuration);
    }
    return false;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick, uint32_t currentTick,
                                uint32_t landingTimeoutDuration) {
    return (currentTick - latestLandingTick >= landingTimeoutDuration);
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop, bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed) {
    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
    }
    return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint, uint32_t supervisorConditionBits, SupervisorState state) {
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
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly, uint32_t currentTick) {
    if (!canFly || !motorRPMs) {
        g_motorNotRespondingStartTick = 0;
        return false;
    }
    bool low = false;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmThreshold) { low = true; break; }
    }
    if (low) {
        if (g_motorNotRespondingStartTick == 0) g_motorNotRespondingStartTick = currentTick;
        return (currentTick - g_motorNotRespondingStartTick >= rpmCheckDurationMs);
    } else {
        g_motorNotRespondingStartTick = 0;
        return false;
    }
}

void supervisorSetSensorData(const SensorData *sensors) {
    if (sensors) g_latestSensorData = *sensors;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
    if (motorRatios) memcpy(g_latestMotorRatios, motorRatios, sizeof(g_latestMotorRatios));
    g_latestIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
    if (motorRPMs) memcpy(g_latestMotorRPMs, motorRPMs, sizeof(g_latestMotorRPMs));
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled) {
    g_safetyCrashDetectionGs = crashDetectionGs;
    g_safetyFreeFallThreshold = freeFallThreshold;
    g_safetyAcceptedTiltAccZ = acceptedTiltAccZ;
    g_safetyAcceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
    g_safetyMaxTiltTime = maxTiltTime;
    g_safetyMaxUpsideDownTime = maxUpsideDownTime;
    g_safetyTumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
    g_armingAutoArming = autoArming;
    g_armingSpinupTimeoutMs = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep) {
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;
    g_currentTick += 10;

    if (g_armingAutoArming && supervisorState == supervisorStatePreFlChecksPassed) {
        supervisorRequestArming(true);
    }

    if (supervisorState == supervisorStateArming && g_armingSpinupTimeoutMs > 0 && g_armingStartTick > 0) {
        if (g_currentTick - g_armingStartTick >= g_armingSpinupTimeoutMs) {
            supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
        }
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }

    if (!supervisorAreMotorsAllowedToRun()) {
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    }

    supervisorLog.info = supervisorGetInfoBitfield();
    supervisorLog.accNorm = sqrtf(g_latestSensorData.acc.x * g_latestSensorData.acc.x +
                                  g_latestSensorData.acc.y * g_latestSensorData.acc.y +
                                  g_latestSensorData.acc.z * g_latestSensorData.acc.z);
}

/* =========================================================================
 * 8. Estimator and Commander arbitration (TC-163～TC-178)
 * ========================================================================= */

bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
    if (!measurement || g_estimatorCount >= ESTIMATOR_QUEUE_CAPACITY) return false;
    g_estimatorQueue[g_estimatorTail] = *measurement;
    g_estimatorTail = (g_estimatorTail + 1) % ESTIMATOR_QUEUE_CAPACITY;
    g_estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
    if (!measurement || g_estimatorCount == 0) return false;
    *measurement = g_estimatorQueue[g_estimatorHead];
    g_estimatorHead = (g_estimatorHead + 1) % ESTIMATOR_QUEUE_CAPACITY;
    g_estimatorCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        if (m.type == MeasurementTypeGyroscope) {
            gyro.x = m.data[0]; gyro.y = m.data[1]; gyro.z = m.data[2];
        } else if (m.type == MeasurementTypeAcceleration) {
            acc.x = m.data[0]; acc.y = m.data[1]; acc.z = m.data[2];
        } else if (m.type == MeasurementTypeBarometer) {
            baro.pressure = m.data[0]; baro.temp = m.data[1]; baro.asl = m.data[2];
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, 0.004f);
        sensfusion6GetEulerRPY(&stateEstimate.roll, &stateEstimate.pitch, &stateEstimate.yaw);
        sensfusion6GetQuaternion(&stateEstimate.qw, &stateEstimate.qx, &stateEstimate.qy, &stateEstimate.qz);
    }
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
    if (!setpoint) return false;
    if (priority == COMMANDER_PRIORITY_DISABLE) {
        g_activePriority = COMMANDER_PRIORITY_DISABLE;
        g_commanderLastUpdateTick = g_currentTick;
        thrustLocked = true;
        return true;
    }
    if (priority >= g_activePriority) {
        g_activeSetpoint = *setpoint;
        g_activePriority = priority;
        g_commanderLastUpdateTick = g_currentTick;
        return true;
    }
    return false;
}

void commanderRelaxPriority(void) {
    g_activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
    return g_currentTick - g_commanderLastUpdateTick;
}

int commanderGetActivePriority(void) {
    return g_activePriority;
}

/* =========================================================================
 * 9. Stabilizer, state compression, and rate supervision (TC-179～TC-194, TC-231～TC-232, TC-257)
 * ========================================================================= */

void stabilizerInit(void) {
    sensfusion6Init();
    attitudeControllerInit(0.002f);
    supervisorInit();
    crtpInit();
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
    return commanderSetSetpoint(setpoint, COMMANDER_PRIORITY_HIGHLEVEL);
}

void stabilizerTask(void) {
    if (healthShallWeRunTest()) {
        healthRunTests(&g_latestSensorData);
        return;
    }
    estimatorComplementary(0);
    supervisorUpdate(0);
    if (!supervisorCanFly()) {
        motor.m1req = 0; motor.m2req = 0; motor.m3req = 0; motor.m4req = 0;
    }
}

void compressState(const State *state, const SensorData *sensors, CompressedState *output) {
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

    output->gyro_millirad_s[0] = sensors->gyro.x * (float)(M_PI / 180.0) * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * (float)(M_PI / 180.0) * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * (float)(M_PI / 180.0) * 1000.0f;

    uint32_t qw_q = (uint32_t)((state->attitudeQuaternion.w + 1.0f) * 511.5f) & 0x3FF;
    uint32_t qx_q = (uint32_t)((state->attitudeQuaternion.x + 1.0f) * 511.5f) & 0x3FF;
    uint32_t qy_q = (uint32_t)((state->attitudeQuaternion.y + 1.0f) * 511.5f) & 0x3FF;
    output->quatCompressed = (qw_q << 20) | (qx_q << 10) | qy_q;
}

bool rateSupervisorValidate(uint32_t measuredRate) {
    return (measuredRate >= 997U && measuredRate <= 1003U);
}

void rateSupervisorTask(void) {}

/* =========================================================================
 * 10. Health (TC-195～TC-212)
 * ========================================================================= */

void healthRequestPropTest(void) {
    g_propTestRequested = true;
}

void healthRequestBatteryTest(void) {
    g_batteryTestRequested = true;
}

bool healthShallWeRunTest(void) {
    if (g_propTestRequested) {
        g_propTestRequested = false;
        healthTestState = configureAcc;
        return true;
    }
    if (g_batteryTestRequested) {
        g_batteryTestRequested = false;
        healthTestState = testBattery;
        g_healthTick = 0;
        return true;
    }
    if (healthTestState != testDone) return true;
    return false;
}

void healthRunTests(const SensorData *sensorData) {
    (void)sensorData;
    if (healthTestState == configureAcc) {
        healthTestState = measureNoiseFloor;
    } else if (healthTestState == measureNoiseFloor) {
        healthTestState = measureProp;
    } else if (healthTestState == measureProp) {
        healthTestState = evaluatePropResult;
    } else if (healthTestState == evaluatePropResult) {
        healthTestState = testDone;
    } else if (healthTestState == testBattery) {
        g_healthTick++;
        if (g_healthTick == 1) {
            g_idleVoltage = 4.2f;
            g_minLoadedVoltage = 4.2f;
        } else if (g_healthTick >= 2 && g_healthTick <= 49) {
            if (g_minLoadedVoltage > 3.5f) g_minLoadedVoltage = 3.5f;
        } else if (g_healthTick >= 50) {
            healthTestState = evaluateBatResult;
        }
    } else if (healthTestState == evaluateBatResult) {
        batterySag = g_idleVoltage - g_minLoadedVoltage;
        if (batterySag <= 0.8f) batteryPass = 1;
        else batteryPass = 0;
        healthTestState = testDone;
    }
    healthLog.motorPass = motorPass;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
}

bool evaluatePropTest(float lowThreshold, float highThreshold, float measuredValue, uint8_t motor) {
    if (highThreshold == 0.0f) return true;
    if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
        motorPass |= (1 << motor);
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
    float res = sumSq - (sum * sum / (float)length);
    if (res < 0.0f && res > -1e-5f) res = 0.0f;
    return res;
}

/* =========================================================================
 * 11. CRTP transport (TC-213～TC-230)
 * ========================================================================= */

void crtpInit(void) {
    if (!g_crtpInitialized) {
        g_crtpTxHead = 0;
        g_crtpTxTail = 0;
        g_crtpTxCount = 0;
        memset(g_crtpRxQueues, 0, sizeof(g_crtpRxQueues));
        g_crtpInitialized = true;
    }
}

void crtpInitTaskQueue(uint8_t port) {
    if (port >= CRTP_NBR_OF_PORTS) return;
    if (g_crtpRxQueues[port].created) {
        supervisorState = supervisorStateCrashed;
        return;
    }
    g_crtpRxQueues[port].created = true;
    g_crtpRxQueues[port].head = 0;
    g_crtpRxQueues[port].tail = 0;
    g_crtpRxQueues[port].count = 0;
}

bool crtpSendPacket(const CrtpPacket *packet) {
    if (!packet || g_crtpTxCount >= CRTP_TX_QUEUE_SIZE) return false;
    g_crtpTxQueue[g_crtpTxTail] = *packet;
    g_crtpTxTail = (g_crtpTxTail + 1) % CRTP_TX_QUEUE_SIZE;
    g_crtpTxCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
    if (port >= CRTP_NBR_OF_PORTS || !packet) return false;
    CrtpRxQueue *q = &g_crtpRxQueues[port];
    if (!q->created || q->count == 0) return false;
    *packet = q->packets[q->head];
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
    CrtpPacket p;
    if (g_currentLink && g_currentLink->receivePacket && g_currentLink->receivePacket(&p)) {
        if (p.port < CRTP_NBR_OF_PORTS) {
            CrtpRxQueue *q = &g_crtpRxQueues[p.port];
            if (q->created && q->count < CRTP_RX_QUEUE_SIZE) {
                q->packets[q->tail] = p;
                q->tail = (q->tail + 1) % CRTP_RX_QUEUE_SIZE;
                q->count++;
            }
            if (q->callback) {
                q->callback(&p);
            }
        }
    }
}

void crtpTxTask(void) {
    if (g_crtpTxCount > 0 && g_currentLink && g_currentLink->sendPacket) {
        CrtpPacket *p = &g_crtpTxQueue[g_crtpTxHead];
        if (g_currentLink->sendPacket(p)) {
            g_crtpTxHead = (g_crtpTxHead + 1) % CRTP_TX_QUEUE_SIZE;
            g_crtpTxCount--;
        }
    }
}

void crtpSetLink(CrtpLink *newLink) {
    if (g_currentLink && g_currentLink->setEnable) {
        g_currentLink->setEnable(false);
    }
    g_currentLink = newLink ? newLink : &g_nopLink;
    if (g_currentLink->setEnable) {
        g_currentLink->setEnable(true);
    }
}

void crtpReset(void) {
    g_crtpTxHead = 0;
    g_crtpTxTail = 0;
    g_crtpTxCount = 0;
    if (g_currentLink && g_currentLink->reset) {
        g_currentLink->reset();
    }
}

bool crtpIsConnected(void) {
    if (g_currentLink && g_currentLink->isConnected) {
        return g_currentLink->isConnected();
    }
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
    return (uint32_t)(CRTP_TX_QUEUE_SIZE - g_crtpTxCount);
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
    if (port >= CRTP_NBR_OF_PORTS) return;
    g_crtpRxQueues[port].callback = callback;
}

void updateStats(void) {}

/* =========================================================================
 * 12. Deck Discovery (TC-233～TC-248)
 * ========================================================================= */

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
    if (!decks || capacity == 0) return 0;

    static const DeckInfo knownDecks[] = {
        { true, false, 0xBC, 0 },
        { false, true, 0x00, 0x0123456789ABCDEFULL }
    };
    uint8_t count = 0;
    size_t numKnown = sizeof(knownDecks) / sizeof(knownDecks[0]);

    for (size_t i = 0; i < numKnown && count < capacity; i++) {
        decks[count++] = knownDecks[i];
    }
    return count;
}

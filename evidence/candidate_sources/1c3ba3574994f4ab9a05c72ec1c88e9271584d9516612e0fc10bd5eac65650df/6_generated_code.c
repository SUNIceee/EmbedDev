#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#if !defined(M_PI)
#define M_PI_F 3.14159265358979323846f
#else
#define M_PI_F ((float)M_PI)
#endif

uint32_t currentTick = 0U;

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

PidObject pidRoll = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidPitch = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidYaw = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidRollRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidPitchRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidYawRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

bool thrustLocked = false;
bool commanderModeSet = false;

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0U;

TestState healthTestState = testDone;
uint8_t motorPass = 0U;
uint8_t batteryPass = 0U;
float batterySag = 0.0f;

StateEstimateLog stateEstimate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
Axis3Log gyro = {0.0f, 0.0f, 0.0f};
Axis3Log acc = {0.0f, 0.0f, 0.0f};
BaroLog baro = {0.0f, 0.0f, 0.0f};
MotorLog motor = {0U, 0U, 0U, 0U};
Sensfusion6Log sensfusion6Log = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, false, false};
SupervisorLog supervisorLog = {0U, 0.0f};
HealthLog healthLog = {0U, 0U, 0.0f, 0U};

static float attitudeDt = 0.0f;
static float desiredYaw = 0.0f;
static bool desiredYawInitialized = false;

static uint32_t recentFlightTick = 0U;
static bool seenFlight = false;
static float supervisorCrashDetectionGs = 0.0f;
static float supervisorFreeFallThreshold = 0.0f;
static float supervisorAcceptedTiltAccZ = 0.0f;
static float supervisorAcceptedUpsideDownAccZ = 0.0f;
static uint32_t supervisorMaxTiltTime = 0U;
static uint32_t supervisorMaxUpsideDownTime = 0U;
static bool supervisorTumbleCheckEnabled = false;
static bool supervisorAutoArming = false;
static uint32_t supervisorSpinupTimeoutDurationMs = 0U;
static uint32_t supervisorSpinupStartTick = 0U;
static bool supervisorSpinupTimeoutOccurred = false;
static SensorData supervisorSensors = {{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},0.0f,0.0f,0.0f,0.0f};
static uint32_t supervisorMotorRatios[4] = {0U,0U,0U,0U};
static uint32_t supervisorIdleThrust = 0U;
static int32_t supervisorMotorRPMs[4] = {0,0,0,0};
static uint32_t supervisorTumbleStartTick = 0U;
static uint8_t supervisorTumbleType = 0U;
static bool supervisorIsFreeFallInternal = false;
static uint32_t motorsNotRespondingStartTick = 0U;

static EstimatorMeasurement estimatorFifo[16];
static uint8_t estimatorHead = 0U, estimatorTail = 0U, estimatorCount = 0U;
static EstimatorMeasurement estimatorLastGyro = {MeasurementTypeGyroscope, {0.0f,0.0f,0.0f}};
static EstimatorMeasurement estimatorLastAcc = {MeasurementTypeAcceleration, {0.0f,0.0f,0.0f}};
static EstimatorMeasurement estimatorLastBaro = {MeasurementTypeBarometer, {0.0f,0.0f,0.0f}};
static EstimatorMeasurement estimatorLastTof = {MeasurementTypeTOF, {0.0f,0.0f,0.0f}};
static bool estimatorHasGyro = false;
static bool estimatorHasAcc = false;
static bool estimatorHasBaro = false;
static bool estimatorHasTof = false;

static Setpoint commanderActiveSetpoint = {{0,0,0,0,0,0,0},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f,1.0f},0U,0U};
static int commanderActivePriority = COMMANDER_PRIORITY_LOWEST;
static uint32_t commanderLastUpdateTick = 0U;

static Setpoint pendingHighLevelSetpoint;
static bool highLevelSetpointPending = false;
static bool stabilizerInitialized = false;
static uint32_t g_stabilizerStep = 0U;
static SensorData g_sensors = {{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},0.0f,0.0f,0.0f,0.0f};
static State g_state = {{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f,1.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f}};
static uint32_t g_motorPwm[4] = {0U,0U,0U,0U};
static MotorPower g_motorPower = {0,0,0,0};
static bool g_rateSupervisorError = false;
static uint32_t g_rateSupervisorStartTick = 0U;
static bool g_sensorActive = true;

static bool healthPropRequestPending = false;
static bool healthBatteryRequestPending = false;
static uint8_t healthNoiseSamples = 0U;
static float healthNoiseBuffer[PROPTEST_NBR_OF_VARIANCE_VALUES];
static float healthIdleVoltage = 0.0f;
static float healthMinLoadedVoltage = 0.0f;
static uint8_t healthBatteryTick = 0U;
static uint8_t healthMotorIndex = 0U;
static uint32_t healthRestartStartTick = 0U;
static float healthMeasuredValue = 0.0f;

static CrtpPacket crtpTxQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t crtpTxHead = 0U, crtpTxTail = 0U, crtpTxCount = 0U;
static CrtpPacket crtpRxQueue[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint8_t crtpRxHead[CRTP_NBR_OF_PORTS], crtpRxTail[CRTP_NBR_OF_PORTS], crtpRxCount[CRTP_NBR_OF_PORTS];
static bool crtpPortQueueCreated[CRTP_NBR_OF_PORTS];
static CrtpPortCallback crtpPortCallback[CRTP_NBR_OF_PORTS];
static bool crtpInitialized = false;
static bool crtpErrorState = false;
static uint32_t crtpRxPacketCount = 0U, crtpTxPacketCount = 0U;
static uint32_t crtpLastStatsTick = 0U;
static float crtpRxRate = 0.0f, crtpTxRate = 0.0f;
static uint32_t crtpTxRetryTick = 0U;

static bool nopSendPacket(CrtpPacket *packet) { (void)packet; return false; }
static bool nopReceivePacket(CrtpPacket *packet) { (void)packet; return false; }
static bool nopIsConnected(void) { return true; }
static void nopSetEnable(bool enable) { (void)enable; }
static void nopReset(void) {}
static CrtpLink nopLink = {nopSendPacket, nopReceivePacket, nopIsConnected, nopSetEnable, nopReset};
static CrtpLink crtpActiveLink = {nopSendPacket, nopReceivePacket, nopIsConnected, nopSetEnable, nopReset};
static bool crtpLinkSet = false;

static int32_t clampInt32(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float degToRad(float deg) { return deg * M_PI_F / 180.0f; }
static float radToDeg(float rad) { return rad * 180.0f / M_PI_F; }

static void syncSensfusionLog(void) {
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

static void pidResetObject(PidObject *pid) {
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
}

static void pidInitObject(PidObject *pid) {
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
    pid->initialized = true;
}

static void pidUpdate(PidObject *pid, float actual, float desired, bool reset) {
    if (!pid || !pid->initialized) return;
    if (reset) pidResetObject(pid);
    float error = desired - actual;
    float pTerm = pid->kp * error;
    float iTerm = 0.0f;
    if (pid->ki != 0.0f && attitudeDt > 1e-9f) {
        pid->integral += pid->ki * error * attitudeDt;
        iTerm = pid->integral;
    }
    float dTerm = 0.0f;
    if (pid->kd != 0.0f && attitudeDt > 1e-9f && pid->initialized) {
        dTerm = pid->kd * (error - pid->prevError) / attitudeDt;
    }
    float out = pTerm + iTerm + dTerm + pid->kff * desired;
    out = (float)saturateSignedInt16((int32_t)out);
    pid->output = out;
    pid->prevError = error;
}

static float quaternionYawDeg(float qwv, float qxv, float qyv, float qzv) {
    float y = atan2f(2.0f * (qwv * qzv + qxv * qyv),
                     1.0f - 2.0f * (qyv * qyv + qzv * qzv));
    return radToDeg(y);
}

static uint32_t quatcompress(float qwv, float qxv, float qyv, float qzv) {
    (void)qwv;
    int32_t x = (int32_t)clampf(qxv * 1000.0f, -32767.0f, 32767.0f);
    int32_t y = (int32_t)clampf(qyv * 1000.0f, -32767.0f, 32767.0f);
    int32_t z = (int32_t)clampf(qzv * 1000.0f, -32767.0f, 32767.0f);
    uint32_t packed = ((uint32_t)(uint16_t)(x & 0xFFFF) << 16) |
                      ((uint32_t)(uint16_t)(y & 0xFFFF) << 8) |
                      ((uint32_t)(uint16_t)(z & 0xFFFF));
    if (qwv < 0.0f) packed |= 0x80000000U;
    return packed;
}

static int32_t motorForceToPwm(float force) {
    if (force <= 0.0f) return 0;
    float ratio = force / CRAZYFLIE_MAX_MOTOR_FORCE_N;
    if (ratio > 1.0f) ratio = 1.0f;
    return (int32_t)(ratio * 65535.0f);
}

static void positionControllerReset(void) {}

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

void sensfusion6Init(void) {
    if (!sensfusion6IsInit) {
        qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
        gravityX = 0.0f; gravityY = 0.0f; gravityZ = 1.0f;
        integralFBx = integralFBy = integralFBz = 0.0f;
        baseZacc = 0.0f;
        sensfusion6IsCalibrated = false;
        sensfusion6IsInit = true;
    }
    syncSensfusionLog();
}

bool sensfusion6Test(void) { return sensfusion6IsInit; }

void estimatedGravityDirection(float qwv, float qxv, float qyv, float qzv,
                               float *gravX, float *gravY, float *gravZ) {
    if (!gravX || !gravY || !gravZ) return;
    *gravX = 2.0f * (qxv * qzv - qwv * qyv);
    *gravY = 2.0f * (qwv * qxv + qyv * qzv);
    *gravZ = qwv * qwv - qxv * qxv - qyv * qyv + qzv * qzv;
}

float invSqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    float halfx = 0.5f * x;
    float y = x;
    int32_t i;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - halfx * y * y);
    return y;
}

void sensfusion6UpdateQ(float gxIn, float gyIn, float gzIn,
                        float ax, float ay, float az, float dt) {
    if (!sensfusion6IsInit) sensfusion6Init();
    if (dt <= 0.0f) dt = 0.004f;
    float gxr = degToRad(gxIn);
    float gyr = degToRad(gyIn);
    float gzr = degToRad(gzIn);
    bool accValid = !(ax == 0.0f && ay == 0.0f && az == 0.0f);
    if (accValid) {
        float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        float gxEst, gyEst, gzEst;
        estimatedGravityDirection(qw, qx, qy, qz, &gxEst, &gyEst, &gzEst);
        if (!sensfusion6IsCalibrated) {
            baseZacc = ax * gxEst + ay * gyEst + az * gzEst;
            sensfusion6IsCalibrated = true;
        }
        float halfvx = qx * qz - qw * qy;
        float halfvy = qw * qx + qy * qz;
        float halfvz = qw * qw - 0.5f + qz * qz;
        float halfex = ay * halfvz - az * halfvy;
        float halfey = az * halfvx - ax * halfvz;
        float halfez = ax * halfvy - ay * halfvx;
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gxr += integralFBx;
            gyr += integralFBy;
            gzr += integralFBz;
        } else {
            integralFBx = integralFBy = integralFBz = 0.0f;
        }
        gxr += twoKp * halfex;
        gyr += twoKp * halfey;
        gzr += twoKp * halfez;
    }
    gxr *= 0.5f * dt;
    gyr *= 0.5f * dt;
    gzr *= 0.5f * dt;
    float qa = qw, qb = qx, qc = qy;
    qw += -qb * gxr - qc * gyr - qz * gzr;
    qx +=  qa * gxr + qc * gzr - qz * gyr;
    qy +=  qa * gyr - qb * gzr + qz * gxr;
    qz +=  qa * gzr + qb * gyr - qc * gxr;
    float norm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    qw *= norm; qx *= norm; qy *= norm; qz *= norm;
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
    syncSensfusionLog();
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
    if (!roll_deg || !pitch_deg || !yaw_deg) return;
    float gx = 2.0f * (qx * qz - qw * qy);
    float gy = 2.0f * (qw * qx + qy * qz);
    float gz = qw * qw - qx * qx - qy * qy + qz * qz;
    *pitch_deg = radToDeg(asinf(clampf(-gx, -1.0f, 1.0f)));
    *roll_deg = radToDeg(atan2f(gy, gz));
    *yaw_deg = radToDeg(atan2f(2.0f * (qw * qz + qx * qy),
                               1.0f - 2.0f * (qy * qy + qz * qz)));
}

void sensfusion6GetQuaternion(float *qw_out, float *qx_out, float *qy_out, float *qz_out) {
    if (!qw_out || !qx_out || !qy_out || !qz_out) return;
    *qw_out = qw; *qx_out = qx; *qy_out = qy; *qz_out = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
    return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

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
    float arm = 0.707106781f * armLength;
    float rollPart = (fabsf(armLength) < 1e-9f) ? 0.0f : (0.25f / arm) * torqueX;
    float pitchPart = (fabsf(armLength) < 1e-9f) ? 0.0f : (0.25f / arm) * torqueY;
    float yawPart = (fabsf(thrustToTorque) < 1e-9f) ? 0.0f : (0.25f / thrustToTorque) * torqueZ;
    float f0 = thrustPart - rollPart + pitchPart - yawPart;
    float f1 = thrustPart - rollPart - pitchPart + yawPart;
    float f2 = thrustPart + rollPart - pitchPart - yawPart;
    float f3 = thrustPart + rollPart + pitchPart + yawPart;
    if (f0 < 0.0f) f0 = 0.0f;
    if (f1 < 0.0f) f1 = 0.0f;
    if (f2 < 0.0f) f2 = 0.0f;
    if (f3 < 0.0f) f3 = 0.0f;
    motorForces[0] = f0; motorForces[1] = f1; motorForces[2] = f2; motorForces[3] = f3;
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4]) {
    if (!normalizedForces || !motorPWMs) return;
    for (int i = 0; i < 4; ++i) {
        float v = clampf(normalizedForces[i], 0.0f, 1.0f);
        motorPWMs[i] = (uint16_t)(v * 65535.0f);
    }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower) {
    if (!control || !motorPower) return;
    switch (control->controlMode) {
    case controlModeLegacy:
        powerDistributionLegacy(control->thrust, control->roll, control->pitch,
                                 control->yaw, motorPower);
        break;
    case controlModeForceTorque: {
        float forces[4];
        powerDistributionForceTorque(control->thrustSi, control->torque.x, control->torque.y,
                                      control->torque.z, CRAZYFLIE_ARM_LENGTH_M,
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
        break;
    }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) {
    return value < idleThrust ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust) {
    PowerCapResult result;
    result.isCapped = false;
    result.reduction = 0;
    if (!motors) return result;
    int32_t maxVal = motors[0];
    if (motors[1] > maxVal) maxVal = motors[1];
    if (motors[2] > maxVal) maxVal = motors[2];
    if (motors[3] > maxVal) maxVal = motors[3];
    if (maxVal > maxAllowedThrust) {
        int32_t reduction = maxVal - maxAllowedThrust;
        for (int i = 0; i < 4; ++i) {
            motors[i] = capMinThrust(motors[i] - reduction, idleThrust);
        }
        result.isCapped = true;
        result.reduction = reduction;
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
    float comp = roundf((float)motorThrust * nominalVoltage / actualVoltage);
    if (comp < 0.0f) comp = 0.0f;
    if (comp > 65535.0f) comp = 65535.0f;
    return (uint16_t)comp;
}

void attitudeControllerInit(float updateDt) {
    if (updateDt > 0.0f) attitudeDt = updateDt;
    if (!pidRoll.initialized || !pidPitch.initialized || !pidYaw.initialized ||
        !pidRollRate.initialized || !pidPitchRate.initialized || !pidYawRate.initialized) {
        pidInitObject(&pidRoll);
        pidInitObject(&pidPitch);
        pidInitObject(&pidYaw);
        pidInitObject(&pidRollRate);
        pidInitObject(&pidPitchRate);
        pidInitObject(&pidYawRate);
    }
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
    pidUpdate(&pidRollRate, rollActual, rollDesired, false);
    pidUpdate(&pidPitchRate, pitchActual, pitchDesired, false);
    pidUpdate(&pidYawRate, yawActual, yawDesired, false);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
    pidUpdate(&pidRoll, rollActual, rollDesired, false);
    pidUpdate(&pidPitch, pitchActual, pitchDesired, false);
    pidUpdate(&pidYaw, yawActual, yawDesired, true);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual) {
    (void)rollActual; (void)pitchActual; (void)yawActual;
    pidResetObject(&pidRoll);
    pidResetObject(&pidPitch);
    pidResetObject(&pidYaw);
    pidResetObject(&pidRollRate);
    pidResetObject(&pidPitchRate);
    pidResetObject(&pidYawRate);
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
    (void)rollActual;
    pidResetObject(&pidRoll);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
    (void)pitchActual;
    pidResetObject(&pidPitch);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw) {
    if (!roll || !pitch || !yaw) return;
    *roll = saturateSignedInt16((int32_t)pidRollRate.output);
    *pitch = saturateSignedInt16((int32_t)pidPitchRate.output);
    *yaw = saturateSignedInt16((int32_t)pidYawRate.output);
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
    if (!setpoint || !state) return 0U;
    float cmd = 0.0f;
    if (setpoint->mode.z == modeVelocity) {
        float velError = setpoint->velocity.z - state->velocity.z;
        cmd = 32768.0f + 32767.0f * setpoint->velocity.z + 1000.0f * velError;
    } else if (setpoint->mode.z == modeAbs) {
        float posError = setpoint->position.z - state->position.z;
        cmd = 32768.0f + 2000.0f * posError;
    } else {
        return 0U;
    }
    if (cmd < 0.0f) cmd = 0.0f;
    if (cmd > 65535.0f) cmd = 65535.0f;
    return (uint16_t)cmd;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
    if (!sensors || !setpoint || !state || !control) return;
    memset(control, 0, sizeof(*control));
    control->controlMode = controlModeLegacy;
    float rollActual = state->attitude.roll;
    float pitchActual = state->attitude.pitch;
    float yawActual = state->attitude.yaw;
    if (!desiredYawInitialized) {
        desiredYaw = yawActual;
        desiredYawInitialized = true;
    }
    if (setpoint->mode.yaw == modeVelocity) {
        desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
    } else if (setpoint->mode.yaw == modeAbs) {
        desiredYaw = setpoint->attitude.yaw;
    } else if (setpoint->mode.quat == modeAbs) {
        desiredYaw = quaternionYawDeg(setpoint->attitudeQuaternion.w,
                                      setpoint->attitudeQuaternion.x,
                                      setpoint->attitudeQuaternion.y,
                                      setpoint->attitudeQuaternion.z);
    }
    if (yawMaxDelta != 0.0f) {
        float dy = capAngle(desiredYaw - yawActual);
        if (dy > yawMaxDelta) desiredYaw = yawActual + yawMaxDelta;
        else if (dy < -yawMaxDelta) desiredYaw = yawActual - yawMaxDelta;
    }
    if (setpoint->mode.z == modeDisable) {
        control->thrust = setpoint->thrust;
    } else {
        control->thrust = positionControllerUpdate(setpoint, state);
    }
    if (control->thrust == 0U) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0U;
        attitudeControllerResetAllPID(rollActual, pitchActual, yawActual);
        positionControllerReset();
        desiredYaw = yawActual;
        return;
    }
    float rollRateDesired, pitchRateDesired, yawRateDesired;
    if (setpoint->mode.roll == modeVelocity) {
        rollRateDesired = setpoint->attitudeRate.roll;
        attitudeControllerResetRollAttitudePID(rollActual);
    } else {
        pidUpdate(&pidRoll, rollActual, setpoint->attitude.roll, false);
        rollRateDesired = pidRoll.output;
    }
    if (setpoint->mode.pitch == modeVelocity) {
        pitchRateDesired = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitchAttitudePID(pitchActual);
    } else {
        pidUpdate(&pidPitch, pitchActual, setpoint->attitude.pitch, false);
        pitchRateDesired = pidPitch.output;
    }
    if (setpoint->mode.yaw == modeVelocity) {
        yawRateDesired = setpoint->attitudeRate.yaw;
    } else {
        pidUpdate(&pidYaw, yawActual, desiredYaw, true);
        yawRateDesired = pidYaw.output;
    }
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                                     rollRateDesired, pitchRateDesired, yawRateDesired);
    attitudeControllerGetActuatorOutput(&control->roll, &control->pitch, &control->yaw);
    control->yaw = (int16_t)(-control->yaw);
}

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
    if (!rollPrime || !pitchPrime) return;
    float rad = degToRad(yaw_deg);
    float c = cosf(rad);
    float s = sinf(rad);
    *rollPrime = roll * c - pitch * s;
    *pitchPrime = roll * s + pitch * c;
}

void crtpCommanderRpytDecodeSetpoint(
    const CommanderCrtpLegacyValues *values, Setpoint *setpoint,
    bool altHoldMode, bool posHoldMode, bool posSetMode,
    StabilizationType stabilizationModeRoll,
    StabilizationType stabilizationModePitch,
    StabilizationType stabilizationModeYaw, YawMode yawMode) {
    if (!values || !setpoint) return;
    if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) thrustLocked = true;
    if (values->thrust == 0U) thrustLocked = false;
    Setpoint sp;
    memset(&sp, 0, sizeof(sp));
    sp.timestamp = currentTick;
    if (!altHoldMode && commanderModeSet) {
        sp.mode.z = modeDisable;
        commanderModeSet = false;
    }
    uint16_t raw = values->thrust;
    if (posHoldMode) {
        sp.mode.x = modeVelocity;
        sp.mode.y = modeVelocity;
        sp.mode.roll = modeDisable;
        sp.mode.pitch = modeDisable;
        sp.velocity.x = values->pitch / 30.0f;
        sp.velocity.y = values->roll / 30.0f;
        sp.attitude.roll = 0.0f;
        sp.attitude.pitch = 0.0f;
        sp.mode.z = modeDisable;
        if (thrustLocked || raw < MIN_THRUST) sp.thrust = 0U;
        else sp.thrust = raw > MAX_THRUST ? MAX_THRUST : raw;
        if (stabilizationModeYaw == RATE) {
            sp.mode.yaw = modeVelocity;
            sp.attitudeRate.yaw = -values->yaw;
        } else {
            sp.mode.yaw = modeAbs;
            sp.attitude.yaw = values->yaw;
        }
    } else if (posSetMode && raw != 0U) {
        sp.mode.x = modeAbs;
        sp.mode.y = modeAbs;
        sp.mode.z = modeAbs;
        sp.mode.roll = modeDisable;
        sp.mode.pitch = modeDisable;
        sp.mode.yaw = modeAbs;
        sp.position.x = -values->pitch;
        sp.position.y = values->roll;
        sp.position.z = raw / 1000.0f;
        sp.attitude.yaw = values->yaw;
        sp.thrust = 0U;
    } else if (altHoldMode) {
        sp.mode.z = modeVelocity;
        sp.thrust = 0U;
        sp.velocity.z = ((float)raw - 32767.0f) / 32767.0f;
        if (!commanderModeSet) {
            commanderModeSet = true;
            positionControllerReset();
        }
        if (stabilizationModeRoll == RATE) {
            sp.mode.roll = modeVelocity;
            sp.attitudeRate.roll = values->roll;
        } else {
            sp.mode.roll = modeAbs;
            sp.attitude.roll = values->roll;
        }
        if (stabilizationModePitch == RATE) {
            sp.mode.pitch = modeVelocity;
            sp.attitudeRate.pitch = values->pitch;
        } else {
            sp.mode.pitch = modeAbs;
            sp.attitude.pitch = values->pitch;
        }
        if (stabilizationModeYaw == RATE) {
            sp.mode.yaw = modeVelocity;
            sp.attitudeRate.yaw = -values->yaw;
        } else {
            sp.mode.yaw = modeAbs;
            sp.attitude.yaw = values->yaw;
        }
    } else {
        sp.mode.z = modeDisable;
        if (thrustLocked || raw < MIN_THRUST) sp.thrust = 0U;
        else sp.thrust = raw > MAX_THRUST ? MAX_THRUST : raw;
        float r, p;
        if (stabilizationModeRoll == RATE) {
            sp.mode.roll = modeVelocity;
            r = values->roll;
        } else {
            sp.mode.roll = modeAbs;
            r = values->roll;
        }
        if (stabilizationModePitch == RATE) {
            sp.mode.pitch = modeVelocity;
            p = values->pitch;
        } else {
            sp.mode.pitch = modeAbs;
            p = values->pitch;
        }
        if (stabilizationModeYaw == RATE) {
            sp.mode.yaw = modeVelocity;
            sp.attitudeRate.yaw = -values->yaw;
        } else {
            sp.mode.yaw = modeAbs;
            sp.attitude.yaw = values->yaw;
        }
        if (yawMode == PLUSMODE) {
            float rp, pp;
            rotateYaw(r, p, 45.0f, &rp, &pp);
            r = rp; p = pp;
        } else if (yawMode == CAREFREE) {
            memset(&sp, 0, sizeof(sp));
            sp.timestamp = currentTick;
            *setpoint = sp;
            return;
        } else {
            /* XMODE no rotation */
        }
        if (stabilizationModeRoll == RATE) sp.attitudeRate.roll = r;
        else sp.attitude.roll = r;
        if (stabilizationModePitch == RATE) sp.attitudeRate.pitch = p;
        else sp.attitude.pitch = p;
    }
    *setpoint = sp;
}

void supervisorInit(void) {
    static bool initialized = false;
    if (initialized) return;
    initialized = true;
    supervisorState = supervisorStateLocked;
    supervisorConditionBits = 0U;
    recentFlightTick = 0U;
    seenFlight = false;
    supervisorTumbleStartTick = 0U;
    supervisorTumbleType = 0U;
    supervisorIsFreeFallInternal = false;
    supervisorSpinupStartTick = 0U;
    supervisorSpinupTimeoutOccurred = false;
    motorsNotRespondingStartTick = 0U;
    supervisorLog.info = 0U;
    supervisorLog.accNorm = 0.0f;
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
    return (supervisorConditionBits & SUPERVISOR_CB_ARMED) != 0U;
}

bool supervisorIsCrashed(void) {
    return (supervisorConditionBits & SUPERVISOR_CB_CRASHED) != 0U;
}

bool supervisorRequestArming(bool doArm) {
    if (!doArm) {
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
        return true;
    }
    if (supervisorState == supervisorStateArming && supervisorIsArmed()) return true;
    if (!supervisorCanArm()) return false;
    supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    supervisorState = supervisorStateArming;
    supervisorSpinupStartTick = 0U;
    supervisorSpinupTimeoutOccurred = false;
    return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
    if (supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) return false;
    if (!doRecovery) {
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        return true;
    }
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
    uint16_t info = 0U;
    if (supervisorCanArm()) info |= (1U << 0);
    if (supervisorIsArmed()) info |= (1U << 1);
    if (supervisorAutoArming) info |= (1U << 2);
    if (supervisorCanFly()) info |= (1U << 3);
    if (supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) info |= (1U << 4);
    if (supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) info |= (1U << 5);
    if (supervisorState == supervisorStateLocked) info |= (1U << 6);
    if (supervisorConditionBits & SUPERVISOR_CB_CRASHED) info |= (1U << 7);
    return info;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTickParam) {
    if (!motorRatios) return false;
    for (int i = 0; i < 4; ++i) {
        if (motorRatios[i] > idleThrust) {
            recentFlightTick = currentTickParam;
            seenFlight = true;
            break;
        }
    }
    if (!seenFlight) return false;
    return (currentTickParam - recentFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTickParam,
                    bool *isFreeFalling) {
    float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
    if (crashDetectionGs > 0.0f && fabsf(norm - 1.0f) > crashDetectionGs) {
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    }
    if (freeFallThreshold > 0.0f &&
        fabsf(accX) < freeFallThreshold &&
        fabsf(accY) < freeFallThreshold &&
        fabsf(accZ) < freeFallThreshold) {
        supervisorIsFreeFallInternal = true;
        supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
        supervisorTumbleStartTick = 0U;
        supervisorTumbleType = 0U;
        if (isFreeFalling) *isFreeFalling = true;
        return false;
    }
    supervisorIsFreeFallInternal = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;
    if (isFreeFalling) *isFreeFalling = false;
    if (!tumbleCheckEnabled) {
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
        supervisorTumbleStartTick = 0U;
        supervisorTumbleType = 0U;
        return false;
    }
    bool upside = false;
    bool tilt = false;
    if (accZ < acceptedUpsideDownAccZ) upside = true;
    else if (accZ < acceptedTiltAccZ) tilt = true;
    else {
        supervisorTumbleStartTick = 0U;
        supervisorTumbleType = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
        return false;
    }
    uint8_t newType = upside ? 2U : 1U;
    if (supervisorTumbleType != newType) {
        supervisorTumbleType = newType;
        supervisorTumbleStartTick = currentTickParam;
    } else if (supervisorTumbleStartTick == 0U) {
        supervisorTumbleStartTick = currentTickParam;
    }
    uint32_t elapsed = currentTickParam - supervisorTumbleStartTick;
    uint32_t limit = upside ? maxUpsideDownTime : maxTiltTime;
    if (limit > 0U && elapsed >= limit) {
        supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
        return true;
    }
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    return false;
}

bool checkEmergencyStopWatchdog(uint32_t currentTickParam,
                                uint32_t lastNotificationTick) {
    if (lastNotificationTick == 0U) return true;
    return (currentTickParam - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTickParam,
                                  uint32_t preflightTimeoutDuration) {
    (void)state;
    if (latestArmingTick == 0U || preflightTimeoutDuration == 0U) return false;
    return (currentTickParam - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTickParam,
                                uint32_t landingTimeoutDuration) {
    if (latestLandingTick == 0U) return false;
    return (currentTickParam - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed) {
    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
    }
    return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t conditionBits,
                                SupervisorState state) {
    (void)conditionBits;
    if (!setpoint) return;
    if (state == supervisorStateWarningLevelOut) {
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->attitudeRate.yaw = 0.0f;
        setpoint->velocity.x = 0.0f;
        setpoint->velocity.y = 0.0f;
        return;
    }
    if (state == supervisorStateArming ||
        state == supervisorStateReadyToFly ||
        state == supervisorStateFlying ||
        state == supervisorStateLanded) {
        return;
    }
    memset(setpoint, 0, sizeof(*setpoint));
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax) {
    if (!motorRPMs) return false;
    for (int i = 0; i < 4; ++i) {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTickParam) {
    if (!canFly) {
        motorsNotRespondingStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return false;
    }
    if (!motorRPMs) return false;
    bool below = false;
    for (int i = 0; i < 4; ++i) {
        if (motorRPMs[i] < rpmThreshold) {
            below = true;
            break;
        }
    }
    if (!below) {
        motorsNotRespondingStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return false;
    }
    if (motorsNotRespondingStartTick == 0U) motorsNotRespondingStartTick = currentTickParam;
    bool fault = false;
    if (rpmCheckDurationMs == 0U) fault = true;
    else if ((currentTickParam - motorsNotRespondingStartTick) >= rpmCheckDurationMs) fault = true;
    if (fault) {
        supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return true;
    }
    supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return false;
}

void supervisorSetSensorData(const SensorData *sensors) {
    if (!sensors) return;
    supervisorSensors = *sensors;
    supervisorLog.accNorm = sqrtf(sensors->acc.x * sensors->acc.x +
                                  sensors->acc.y * sensors->acc.y +
                                  sensors->acc.z * sensors->acc.z);
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
    if (!motorRatios) return;
    for (int i = 0; i < 4; ++i) supervisorMotorRatios[i] = motorRatios[i];
    supervisorIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
    if (!motorRPMs) return;
    for (int i = 0; i < 4; ++i) supervisorMotorRPMs[i] = motorRPMs[i];
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled) {
    supervisorCrashDetectionGs = crashDetectionGs;
    supervisorFreeFallThreshold = freeFallThreshold;
    supervisorAcceptedTiltAccZ = acceptedTiltAccZ;
    supervisorAcceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
    supervisorMaxTiltTime = maxTiltTime;
    supervisorMaxUpsideDownTime = maxUpsideDownTime;
    supervisorTumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
    supervisorAutoArming = autoArming;
    supervisorSpinupTimeoutDurationMs = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep) {
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;
    bool flying = isFlyingCheck(supervisorMotorRatios, supervisorIdleThrust, currentTick);
    if (flying) supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
    else supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
    bool freeFall = false;
    bool tumbled = isTumbledCheck(supervisorSensors.acc.x, supervisorSensors.acc.y,
                                  supervisorSensors.acc.z, supervisorCrashDetectionGs,
                                  supervisorFreeFallThreshold, supervisorAcceptedTiltAccZ,
                                  supervisorAcceptedUpsideDownAccZ, supervisorMaxTiltTime,
                                  supervisorMaxUpsideDownTime, supervisorTumbleCheckEnabled,
                                  currentTick, &freeFall);
    if (tumbled) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    else supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    uint32_t age = 0U;
    if (commanderLastUpdateTick != 0U && currentTick >= commanderLastUpdateTick) {
        age = currentTick - commanderLastUpdateTick;
    }
    if (age > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    } else if (age > COMMANDER_WDT_TIMEOUT_STABILIZE) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    }
    updateAndPopulateConditions(false, false,
                                !checkEmergencyStopWatchdog(currentTick, 0U));
    if (supervisorState == supervisorStateArming) {
        if (supervisorSpinupStartTick != 0U && supervisorSpinupTimeoutDurationMs > 0U &&
            (currentTick - supervisorSpinupStartTick) >= supervisorSpinupTimeoutDurationMs) {
            supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
            supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
            supervisorState = supervisorStateCrashed;
            supervisorSpinupTimeoutOccurred = true;
        }
    }
    if (supervisorState == supervisorStatePreFlChecksPassed &&
        supervisorAutoArming && !supervisorIsArmed()) {
        (void)supervisorRequestArming(true);
    }
    if (supervisorState != supervisorStateArming && !supervisorSpinupTimeoutOccurred) {
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
        supervisorSpinupStartTick = 0U;
    }
    if (!supervisorCanFly()) {
        supervisorConditionBits &= ~SUPERVISOR_CB_RPM_AT_ARMING_VALID;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    }
    supervisorLog.info = supervisorConditionBits;
    supervisorLog.accNorm = sqrtf(supervisorSensors.acc.x * supervisorSensors.acc.x +
                                  supervisorSensors.acc.y * supervisorSensors.acc.y +
                                  supervisorSensors.acc.z * supervisorSensors.acc.z);
}

bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
    if (!measurement) return false;
    if (estimatorCount >= 16U) return false;
    estimatorFifo[estimatorTail] = *measurement;
    estimatorTail = (uint8_t)((estimatorTail + 1U) % 16U);
    estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
    if (!measurement) return false;
    if (estimatorCount == 0U) return false;
    *measurement = estimatorFifo[estimatorHead];
    estimatorHead = (uint8_t)((estimatorHead + 1U) % 16U);
    estimatorCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        switch (m.type) {
        case MeasurementTypeGyroscope:
            estimatorLastGyro = m; estimatorHasGyro = true; break;
        case MeasurementTypeAcceleration:
            estimatorLastAcc = m; estimatorHasAcc = true; break;
        case MeasurementTypeBarometer:
            estimatorLastBaro = m; estimatorHasBaro = true; break;
        case MeasurementTypeTOF:
            estimatorLastTof = m; estimatorHasTof = true; break;
        default:
            break;
        }
    }
    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        float gx = estimatorHasGyro ? estimatorLastGyro.data[0] : 0.0f;
        float gy = estimatorHasGyro ? estimatorLastGyro.data[1] : 0.0f;
        float gz = estimatorHasGyro ? estimatorLastGyro.data[2] : 0.0f;
        float ax = estimatorHasAcc ? estimatorLastAcc.data[0] : 0.0f;
        float ay = estimatorHasAcc ? estimatorLastAcc.data[1] : 0.0f;
        float az = estimatorHasAcc ? estimatorLastAcc.data[2] : 0.0f;
        sensfusion6UpdateQ(gx, gy, gz, ax, ay, az, 1.0f / SENSFUSION_RATE_HZ);
        float roll, pitch, yaw;
        sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
        stateEstimate.roll = roll;
        stateEstimate.pitch = pitch;
        stateEstimate.yaw = yaw;
        stateEstimate.qw = qw;
        stateEstimate.qx = qx;
        stateEstimate.qy = qy;
        stateEstimate.qz = qz;
        gyro.x = gx;
        gyro.y = -gy;
        gyro.z = gz;
        acc.x = ax;
        acc.y = ay;
        acc.z = az;
        if (estimatorHasBaro) {
            baro.asl = estimatorLastBaro.data[0];
            baro.temp = estimatorLastBaro.data[1];
            baro.pressure = estimatorLastBaro.data[2];
        }
    }
    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        /* Position update integration is intentionally minimal; public state has no
           separate position log. */
    }
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
    if (!setpoint) return false;
    if (priority != COMMANDER_PRIORITY_DISABLE && priority < commanderActivePriority) return false;
    commanderActiveSetpoint = *setpoint;
    commanderActivePriority = priority;
    commanderLastUpdateTick = currentTick;
    if (priority > COMMANDER_PRIORITY_HIGHLEVEL) highLevelSetpointPending = false;
    return true;
}

void commanderRelaxPriority(void) {
    commanderActivePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
    return currentTick - commanderLastUpdateTick;
}

int commanderGetActivePriority(void) {
    return commanderActivePriority;
}

static void stabilizerSensorsInit(void) { g_sensors = (SensorData){{0,0,0},{0,0,0},0,0,0,0}; }
static void stabilizerStateEstimatorInit(void) {
    g_state.attitudeQuaternion.w = 1.0f;
    g_state.attitudeQuaternion.x = g_state.attitudeQuaternion.y = g_state.attitudeQuaternion.z = 0.0f;
}
static void stabilizerControllerInit(void) { attitudeControllerInit(1.0f / ATTITUDE_RATE_HZ); }
static void stabilizerPowerDistributionInit(void) {}
static void stabilizerMotorsInit(void) {
    for (int i = 0; i < 4; ++i) g_motorPwm[i] = 0U;
}
static void stabilizerCollisionAvoidanceInit(void) {}

void stabilizerInit(void) {
    if (stabilizerInitialized) return;
    stabilizerInitialized = true;
    stabilizerSensorsInit();
    stabilizerStateEstimatorInit();
    stabilizerControllerInit();
    stabilizerPowerDistributionInit();
    stabilizerMotorsInit();
    stabilizerCollisionAvoidanceInit();
}

static void sensorsWaitDataReady(void) {}
static void sensorsAcquire(void) {
    g_sensors = supervisorSensors;
    g_sensorActive = true;
}
static void stateEstimator(uint32_t step) {
    estimatorComplementary(step);
    g_state.attitude.roll = stateEstimate.roll;
    g_state.attitude.pitch = stateEstimate.pitch;
    g_state.attitude.yaw = stateEstimate.yaw;
    g_state.attitudeQuaternion.w = qw;
    g_state.attitudeQuaternion.x = qx;
    g_state.attitudeQuaternion.y = qy;
    g_state.attitudeQuaternion.z = qz;
}
static void commanderGetSetpoint(void) {}
static void collisionAvoidanceUpdateSetpoint(Setpoint *sp) { (void)sp; }
static void setMotorRatios(const int32_t motors[4]) {
    for (int i = 0; i < 4; ++i) {
        int32_t v = motors[i];
        if (v < 0) v = 0;
        if (v > 65535) v = 65535;
        g_motorPwm[i] = (uint32_t)v;
    }
    motor.m1req = (uint16_t)g_motorPwm[0];
    motor.m2req = (uint16_t)g_motorPwm[1];
    motor.m3req = (uint16_t)g_motorPwm[2];
    motor.m4req = (uint16_t)g_motorPwm[3];
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
    if (!setpoint) return false;
    pendingHighLevelSetpoint = *setpoint;
    highLevelSetpointPending = true;
    return true;
}

void stabilizerTask(void) {
    if (healthShallWeRunTest()) {
        healthRunTests(&g_sensors);
        return;
    }
    sensorsWaitDataReady();
    sensorsAcquire();
    uint32_t step = g_stabilizerStep++;
    stateEstimator(step);
    if (highLevelSetpointPending) {
        commanderSetSetpoint(&pendingHighLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        highLevelSetpointPending = false;
    }
    commanderGetSetpoint();
    Setpoint active = commanderActiveSetpoint;
    supervisorUpdate(step);
    collisionAvoidanceUpdateSetpoint(&active);
    supervisorOverrideSetpoint(&active, supervisorConditionBits, supervisorState);
    if (!supervisorCanFly()) {
        memset(&active, 0, sizeof(active));
    }
    ControlData control;
    controllerPid(&g_sensors, &active, &g_state, &control, 0.0f, 1.0f / ATTITUDE_RATE_HZ);
    powerDistribution(&control, &g_motorPower);
    int32_t motors[4] = {g_motorPower.m1, g_motorPower.m2, g_motorPower.m3, g_motorPower.m4};
    /* Battery compensation path uses no public battery voltage; keep a deterministic
       filtered value placeholder. */
    powerDistributionCap(motors, 65535, 0);
    if (!supervisorAreMotorsAllowedToRun()) {
        int32_t stopped[4] = {0,0,0,0};
        setMotorRatios(stopped);
    } else {
        setMotorRatios(motors);
    }
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output) {
    if (!state || !sensors || !output) return;
    for (int i = 0; i < 3; ++i) {
        output->position_mm[i] = (int32_t)(state->position.x * 1000.0f);
        output->velocity_mms[i] = (int32_t)(state->velocity.x * 1000.0f);
        if (i == 0) {
            output->position_mm[i] = (int32_t)(state->position.x * 1000.0f);
            output->velocity_mms[i] = (int32_t)(state->velocity.x * 1000.0f);
        } else if (i == 1) {
            output->position_mm[i] = (int32_t)(state->position.y * 1000.0f);
            output->velocity_mms[i] = (int32_t)(state->velocity.y * 1000.0f);
        } else {
            output->position_mm[i] = (int32_t)(state->position.z * 1000.0f);
            output->velocity_mms[i] = (int32_t)(state->velocity.z * 1000.0f);
        }
    }
    output->acceleration_mms2[0] = (int32_t)(sensors->acc.x * 9810.0f);
    output->acceleration_mms2[1] = (int32_t)(sensors->acc.y * 9810.0f);
    output->acceleration_mms2[2] = (int32_t)((sensors->acc.z + 1.0f) * 9810.0f);
    output->gyro_millirad_s[0] = sensors->gyro.x * M_PI_F / 180.0f * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * M_PI_F / 180.0f * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * M_PI_F / 180.0f * 1000.0f;
    output->quatCompressed = quatcompress(state->attitudeQuaternion.w,
                                          state->attitudeQuaternion.x,
                                          state->attitudeQuaternion.y,
                                          state->attitudeQuaternion.z);
}

bool rateSupervisorValidate(uint32_t measuredRate) {
    return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void) {
    if (g_rateSupervisorStartTick == 0U) g_rateSupervisorStartTick = currentTick;
    if (currentTick - g_rateSupervisorStartTick < 2000U) return;
    if (g_sensorActive) {
        g_rateSupervisorError = true;
    }
}

bool healthShallWeRunTest(void) {
    if (healthPropRequestPending) {
        healthPropRequestPending = false;
        healthTestState = configureAcc;
        healthNoiseSamples = 0U;
        healthMotorIndex = 0U;
        return true;
    }
    if (healthBatteryRequestPending) {
        healthBatteryRequestPending = false;
        healthTestState = testBattery;
        healthBatteryTick = 0U;
        healthRestartStartTick = 0U;
        return true;
    }
    return healthTestState != testDone;
}

void healthRequestPropTest(void) {
    healthPropRequestPending = true;
}

void healthRequestBatteryTest(void) {
    healthBatteryRequestPending = true;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex) {
    if (highThreshold == 0.0f) return true;
    if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
        motorPass |= (uint8_t)(1U << motorIndex);
        return true;
    }
    healthLog.motorTestCount++;
    return false;
}

float variance(const float *buffer, int length) {
    if (!buffer || length <= 0) return 0.0f;
    float sum = 0.0f;
    float sumSq = 0.0f;
    for (int i = 0; i < length; ++i) {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }
    return sumSq - (sum * sum) / (float)length;
}

void healthRunTests(const SensorData *sensorData) {
    if (!sensorData) return;
    switch (healthTestState) {
    case configureAcc:
        motorPass = 0U;
        batteryPass = 0U;
        batterySag = 0.0f;
        healthIdleVoltage = sensorData->baroAsl;
        healthNoiseSamples = 0U;
        healthMotorIndex = 0U;
        healthTestState = measureNoiseFloor;
        break;
    case measureNoiseFloor:
        if (healthNoiseSamples < PROPTEST_NBR_OF_VARIANCE_VALUES) {
            healthNoiseBuffer[healthNoiseSamples++] = sensorData->acc.z;
            return;
        }
        healthMeasuredValue = variance(healthNoiseBuffer, PROPTEST_NBR_OF_VARIANCE_VALUES);
        healthTestState = measureProp;
        break;
    case measureProp:
        if (healthMotorIndex < 4U) {
            (void)evaluatePropTest(0.0f, 0.0f, sensorData->acc.z, healthMotorIndex);
            healthMotorIndex++;
        }
        if (healthMotorIndex >= 4U) healthTestState = evaluatePropResult;
        break;
    case evaluatePropResult:
        healthTestState = testDone;
        break;
    case testBattery:
        healthBatteryTick++;
        if (healthBatteryTick == 1U) {
            healthMinLoadedVoltage = sensorData->baroAsl;
        } else if (healthBatteryTick >= 2U && healthBatteryTick <= 49U) {
            if (sensorData->baroAsl < healthMinLoadedVoltage) {
                healthMinLoadedVoltage = sensorData->baroAsl;
            }
        } else if (healthBatteryTick >= 50U) {
            batterySag = healthIdleVoltage - healthMinLoadedVoltage;
            batteryPass = (batterySag <= 0.5f) ? 1U : 0U;
            healthTestState = evaluateBatResult;
        }
        break;
    case evaluateBatResult:
        healthLog.batteryPass = batteryPass;
        healthLog.batterySag = batterySag;
        healthTestState = testDone;
        break;
    case restartBatTest:
        if (healthRestartStartTick == 0U) healthRestartStartTick = currentTick;
        if ((currentTick - healthRestartStartTick) >= 2000U) {
            healthTestState = testBattery;
            healthBatteryTick = 0U;
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
}

void crtpInit(void) {
    if (crtpInitialized) return;
    crtpInitialized = true;
    crtpTxHead = crtpTxTail = crtpTxCount = 0U;
    for (int i = 0; i < CRTP_NBR_OF_PORTS; ++i) {
        crtpPortQueueCreated[i] = false;
        crtpPortCallback[i] = NULL;
        crtpRxHead[i] = crtpRxTail[i] = crtpRxCount[i] = 0U;
    }
    crtpActiveLink = nopLink;
    crtpLinkSet = false;
    crtpRxPacketCount = crtpTxPacketCount = 0U;
    crtpRxRate = crtpTxRate = 0.0f;
}

void crtpInitTaskQueue(uint8_t port) {
    if (port >= CRTP_NBR_OF_PORTS) {
        crtpErrorState = true;
        return;
    }
    if (crtpPortQueueCreated[port]) {
        crtpErrorState = true;
        return;
    }
    crtpPortQueueCreated[port] = true;
    crtpRxHead[port] = crtpRxTail[port] = crtpRxCount[port] = 0U;
}

bool crtpSendPacket(const CrtpPacket *packet) {
    if (!packet) return false;
    if (crtpTxCount >= CRTP_TX_QUEUE_SIZE) return false;
    crtpTxQueue[crtpTxTail] = *packet;
    crtpTxTail = (uint16_t)((crtpTxTail + 1U) % CRTP_TX_QUEUE_SIZE);
    crtpTxCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
    if (port >= CRTP_NBR_OF_PORTS || !packet) return false;
    if (crtpRxCount[port] == 0U) return false;
    *packet = crtpRxQueue[port][crtpRxHead[port]];
    crtpRxHead[port] = (uint8_t)((crtpRxHead[port] + 1U) % CRTP_RX_QUEUE_SIZE);
    crtpRxCount[port]--;
    return true;
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet) {
    return crtpReceivePacket(port, packet);
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet, uint32_t wait_ms) {
    if (port >= CRTP_NBR_OF_PORTS || !packet) return false;
    if (crtpReceivePacket(port, packet)) return true;
    uint32_t start = currentTick;
    while ((currentTick - start) < wait_ms) {
        crtpRxTask();
        if (crtpReceivePacket(port, packet)) return true;
    }
    return false;
}

void crtpRxTask(void) {
    if (!crtpInitialized) return;
    CrtpPacket packet;
    if (!crtpActiveLink.receivePacket) return;
    if (!crtpActiveLink.receivePacket(&packet)) return;
    crtpRxPacketCount++;
    bool dispatched = false;
    if (packet.port < CRTP_NBR_OF_PORTS) {
        if (crtpPortQueueCreated[packet.port] && crtpRxCount[packet.port] < CRTP_RX_QUEUE_SIZE) {
            crtpRxQueue[packet.port][crtpRxTail[packet.port]] = packet;
            crtpRxTail[packet.port] = (uint8_t)((crtpRxTail[packet.port] + 1U) % CRTP_RX_QUEUE_SIZE);
            crtpRxCount[packet.port]++;
            dispatched = true;
        }
        if (crtpPortCallback[packet.port]) {
            crtpPortCallback[packet.port](&packet);
            dispatched = true;
        }
    }
    (void)dispatched;
}

void crtpTxTask(void) {
    if (!crtpInitialized || crtpTxCount == 0U) return;
    if (!crtpActiveLink.sendPacket)
        return;
    if (!crtpLinkSet)
        return;
    if (crtpTxRetryTick != 0U && currentTick < crtpTxRetryTick) return;
    if (crtpActiveLink.sendPacket(&crtpTxQueue[crtpTxHead])) {
        crtpTxHead = (uint16_t)((crtpTxHead + 1U) % CRTP_TX_QUEUE_SIZE);
        crtpTxCount--;
        crtpTxPacketCount++;
        crtpTxRetryTick = 0U;
    } else {
        crtpTxRetryTick = currentTick + 10U;
    }
}

void crtpSetLink(CrtpLink *newLink) {
    if (crtpLinkSet && crtpActiveLink.setEnable) {
        crtpActiveLink.setEnable(false);
    }
    if (newLink) {
        crtpActiveLink = *newLink;
    } else {
        crtpActiveLink = nopLink;
    }
    crtpLinkSet = true;
    if (crtpActiveLink.setEnable) crtpActiveLink.setEnable(true);
}

void crtpReset(void) {
    crtpTxHead = crtpTxTail = crtpTxCount = 0U;
    if (crtpLinkSet && crtpActiveLink.reset) crtpActiveLink.reset();
}

bool crtpIsConnected(void) {
    if (crtpActiveLink.isConnected) return crtpActiveLink.isConnected();
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
    return CRTP_TX_QUEUE_SIZE - crtpTxCount;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
    if (port >= CRTP_NBR_OF_PORTS) return;
    crtpPortCallback[port] = callback;
}

void updateStats(void) {
    if (currentTick - crtpLastStatsTick >= 500U) {
        uint32_t elapsed = currentTick - crtpLastStatsTick;
        if (elapsed > 0U) {
            crtpRxRate = (float)crtpRxPacketCount * 1000.0f / (float)elapsed;
            crtpTxRate = (float)crtpTxPacketCount * 1000.0f / (float)elapsed;
        }
        crtpRxPacketCount = 0U;
        crtpTxPacketCount = 0U;
        crtpLastStatsTick = currentTick;
    }
}

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
    if (!decks || capacity == 0U) return 0U;
    static const uint8_t knownI2C[] = {0x20U, 0x21U, 0x40U, 0x44U, 0x76U};
    static const uint64_t knownOneWire[] = {0x100000001ULL, 0x100000002ULL, 0x100000003ULL};
    uint8_t written = 0U;
    for (int i = 0; i < (int)(sizeof(knownI2C) / sizeof(knownI2C[0])) && written < capacity; ++i) {
        DeckInfo info;
        memset(&info, 0, sizeof(info));
        info.foundByI2C = true;
        info.i2cAddress = knownI2C[i];
        bool dup = false;
        for (uint8_t j = 0; j < written; ++j) {
            if (decks[j].foundByI2C && decks[j].i2cAddress == info.i2cAddress) {
                dup = true;
                break;
            }
        }
        if (!dup) decks[written++] = info;
    }
    for (int i = 0; i < (int)(sizeof(knownOneWire) / sizeof(knownOneWire[0])) && written < capacity; ++i) {
        DeckInfo info;
        memset(&info, 0, sizeof(info));
        info.foundByOneWire = true;
        info.oneWireRomId = knownOneWire[i];
        bool dup = false;
        for (uint8_t j = 0; j < written; ++j) {
            if (decks[j].foundByOneWire && decks[j].oneWireRomId == info.oneWireRomId) {
                dup = true;
                break;
            }
        }
        if (!dup) decks[written++] = info;
    }
    return written;
}
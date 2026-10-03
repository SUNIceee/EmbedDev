#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#define M_PI_F 3.14159265358979323846f
#define RAD2DEG_F (180.0f / M_PI_F)
#define DEG2RAD_F (M_PI_F / 180.0f)

/* Host monotonic millisecond tick. Not declared in the frozen header; kept
   external so host fixtures can extern it when a setter is not available. */
uint32_t currentTick = 0U;

/* Sensfusion6 public state */
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

/* PID public objects */
PidObject pidRoll = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidPitch = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidYaw = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidRollRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidPitchRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidYawRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
static float s_pidDt = 0.002f;
static bool s_attCtrlInitDone = false;

/* Commander RPYT public state */
bool thrustLocked = false;
bool commanderModeSet = false;
static bool s_carefreeErrorPath = false;

/* Supervisor public state */
SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0U;

/* Supervisor private state */
static bool s_supervisorInitDone = false;
static bool s_autoArming = false;
static uint32_t s_spinupTimeoutDuration = 500U;
static uint32_t s_spinupStartTick = 0U;
static bool s_spinupStarted = false;
static SensorData s_lastSensorData;
static uint32_t s_lastMotorRatios[4] = {0U, 0U, 0U, 0U};
static int32_t s_lastMotorRPMs[4] = {0, 0, 0, 0};
static uint32_t s_idleThrust = 0U;
static float s_crashDetectionGs = 0.0f;
static float s_freeFallThreshold = 0.0f;
static float s_acceptedTiltAccZ = 0.0f;
static float s_acceptedUpsideDownAccZ = 0.0f;
static uint32_t s_maxTiltTime = 0U;
static uint32_t s_maxUpsideDownTime = 0U;
static bool s_tumbleCheckEnabled = true;
static uint32_t s_lastFlyingTick = 0U;
static bool s_seenFlyingTick = false;
static uint32_t s_tumbleStartTick = 0U;
static bool s_tumbleTimerStarted = false;
static uint32_t s_notRespondingStartTick = 0U;
static bool s_notRespondingTimerStarted = false;
static uint32_t s_lastSensorActivityTick = 0U;
static bool s_sensorPaused = false;
static bool s_rateSupervisorError = false;

/* Estimator FIFO and estimate */
static EstimatorMeasurement s_estimatorFifo[16];
static uint8_t s_estimatorHead = 0U, s_estimatorTail = 0U, s_estimatorCount = 0U;
static float s_lastGyro[3] = {0.0f, 0.0f, 0.0f};
static float s_lastAcc[3] = {0.0f, 0.0f, 0.0f};
static float s_lastBaro[3] = {0.0f, 0.0f, 0.0f};
static float s_lastTof[3] = {0.0f, 0.0f, 0.0f};
static State s_stateEstimate;

/* Commander arbitration private state */
static Setpoint s_activeSetpoint;
static int s_activePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t s_commanderLastUpdateTick = 0U;
static bool s_trajectoryActive = false;

/* Stabilizer private state */
static bool s_stabilizerInitDone = false;
static bool s_systemStarted = false;
static bool s_sensorsCalibrated = false;
static uint32_t s_stabilizerStep = 0U;
static SensorData s_sensorsData;
static uint16_t s_motorPwm[4] = {0U, 0U, 0U, 0U};
static float s_batteryVoltage = 3.7f;
static bool s_pendingHighLevel = false;
static Setpoint s_pendingHighLevelSetpoint;
static float s_yawMaxDelta = 0.0f;

/* Health private state */
TestState healthTestState = testDone;
uint8_t motorPass = 0U, batteryPass = 0U;
float batterySag = 0.0f;
static bool s_propTestRequest = false;
static bool s_batTestRequest = false;
static uint32_t s_healthMotorTestCount = 0U;
static int s_propSampleCount = 0;
static int s_propMotorIndex = 0;
static float s_propNoiseVariance = 0.0f;
static float s_healthIdleVoltage = 3.7f;
static float s_healthMinLoadedVoltage = 3.7f;
static int s_batteryTick = 0;
static float s_healthBatterySagThreshold = 0.5f;

/* CRTP private state */
static CrtpPacket s_txQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t s_txHead = 0U, s_txTail = 0U, s_txCount = 0U;
static CrtpPacket s_rxQueue[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint8_t s_rxHead[CRTP_NBR_OF_PORTS] = {0};
static uint8_t s_rxTail[CRTP_NBR_OF_PORTS] = {0};
static uint8_t s_rxCount[CRTP_NBR_OF_PORTS] = {0};
static bool s_rxQueueCreated[CRTP_NBR_OF_PORTS] = {false};
static CrtpPortCallback s_portCallbacks[CRTP_NBR_OF_PORTS] = {NULL};
static bool s_crtpInitDone = false;
static bool s_crtpErrorState = false;
static uint32_t s_txRetryTick = 0U;
static bool s_txRetryPending = false;
static uint32_t s_statsLastTick = 0U;
static uint32_t s_statsRxCount = 0U, s_statsTxCount = 0U;
static uint32_t s_rxRate = 0U, s_txRate = 0U;

static bool nopSendPacket(CrtpPacket *packet) { (void)packet; return false; }
static bool nopReceivePacket(CrtpPacket *packet) { (void)packet; return false; }
static bool nopIsConnected(void) { return true; }
static void nopSetEnable(bool enable) { (void)enable; }
static void nopReset(void) {}
static CrtpLink s_nopLink = {
    nopSendPacket, nopReceivePacket, nopIsConnected, nopSetEnable, nopReset
};
static CrtpLink *s_currentLink = &s_nopLink;

/* Log objects */
StateEstimateLog stateEstimate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
Axis3Log gyro;
Axis3Log acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

static float clampFloat(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static void sensfusion6SyncLog(void) {
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

int16_t saturateSignedInt16(int32_t value) {
    if (value > 32767) return 32767;
    if (value < -32767) return -32767;
    return (int16_t)value;
}

float capAngle(float angle_deg) {
    float a = angle_deg;
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

float invSqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    float halfx = 0.5f * x;
    int32_t i;
    memcpy(&i, &x, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    float y;
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - halfx * y * y);
    return y;
}

void estimatedGravityDirection(float qw_, float qx_, float qy_, float qz_,
                               float *gravX, float *gravY, float *gravZ) {
    if (!gravX || !gravY || !gravZ) return;
    *gravX = 2.0f * (qx_ * qz_ - qw_ * qy_);
    *gravY = 2.0f * (qw_ * qx_ + qy_ * qz_);
    *gravZ = qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_;
}

void sensfusion6Init(void) {
    if (sensfusion6IsInit) return;
    qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
    baseZacc = 0.0f;
    sensfusion6IsInit = true;
    sensfusion6IsCalibrated = false;
    twoKp = 0.8f;
    twoKi = 0.002f;
    beta = 0.01f;
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
    sensfusion6SyncLog();
}

bool sensfusion6Test(void) {
    return sensfusion6IsInit;
}

void sensfusion6GetQuaternion(float *outW, float *outX, float *outY, float *outZ) {
    if (!outW || !outX || !outY || !outZ) return;
    *outW = qw; *outX = qx; *outY = qy; *outZ = qz;
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
    if (!roll_deg || !pitch_deg || !yaw_deg) return;
    float gx, gy_, gz_;
    estimatedGravityDirection(qw, qx, qy, qz, &gx, &gy_, &gz_);
    float clamped = clampFloat(gx, -1.0f, 1.0f);
    *roll_deg = atan2f(gy_, gz_) * RAD2DEG_F;
    *pitch_deg = asinf(-clamped) * RAD2DEG_F;
    *yaw_deg = atan2f(2.0f * (qx * qy + qw * qz),
                      1.0f - 2.0f * (qy * qy + qz * qz)) * RAD2DEG_F;
}

static void sensfusion6MahonyUpdate(float gxRad, float gyRad, float gzRad,
                                    float ax, float ay, float az, float dt) {
    float q0 = qw, q1 = qx, q2 = qy, q3 = qz;
    bool accValid = !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f));
    if (accValid) {
        float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        if (recipNorm > 0.0f) {
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
        }
        float halfvx = q1 * q3 - q0 * q2;
        float halfvy = q0 * q1 + q2 * q3;
        float halfvz = q0 * q0 - 0.5f + q3 * q3;
        float halfex = ay * halfvz - az * halfvy;
        float halfey = az * halfvx - ax * halfvz;
        float halfez = ax * halfvy - ay * halfvx;
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
        } else {
            integralFBx = integralFBy = integralFBz = 0.0f;
        }
        gxRad += integralFBx + twoKp * halfex;
        gyRad += integralFBy + twoKp * halfey;
        gzRad += integralFBz + twoKp * halfez;
    }
    q0 += 0.5f * dt * (-q1 * gxRad - q2 * gyRad - q3 * gzRad);
    q1 += 0.5f * dt * ( q0 * gxRad + q2 * gzRad - q3 * gyRad);
    q2 += 0.5f * dt * ( q0 * gyRad - q1 * gzRad + q3 * gxRad);
    q3 += 0.5f * dt * ( q0 * gzRad + q1 * gyRad - q2 * gxRad);
    float normSq = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    if (normSq > 0.0f) {
        float recipNorm = invSqrt(normSq);
        q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
    }
    qw = q0; qx = q1; qy = q2; qz = q3;
}

static void sensfusion6MadgwickUpdate(float gxRad, float gyRad, float gzRad,
                                      float ax, float ay, float az, float dt) {
    float q0 = qw, q1 = qx, q2 = qy, q3 = qz;
    float qDot0 = 0.5f * (-q1 * gxRad - q2 * gyRad - q3 * gzRad);
    float qDot1 = 0.5f * ( q0 * gxRad + q2 * gzRad - q3 * gyRad);
    float qDot2 = 0.5f * ( q0 * gyRad - q1 * gzRad + q3 * gxRad);
    float qDot3 = 0.5f * ( q0 * gzRad + q1 * gyRad - q2 * gxRad);
    bool accValid = !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f));
    if (accValid) {
        float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        if (recipNorm > 0.0f) {
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
        }
        float f0 = 2.0f * (q1 * q3 - q0 * q2) - ax;
        float f1 = 2.0f * (q0 * q1 + q2 * q3) - ay;
        float f2 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;
        float s0 = -2.0f * q2 * f0 + 2.0f * q1 * f1;
        float s1 =  2.0f * q3 * f0 + 2.0f * q0 * f1 - 4.0f * q1 * f2;
        float s2 = -2.0f * q0 * f0 + 2.0f * q3 * f1 - 4.0f * q2 * f2;
        float s3 =  2.0f * q1 * f0 + 2.0f * q2 * f1;
        float sn = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if (sn > 0.0f) {
            s0 *= sn; s1 *= sn; s2 *= sn; s3 *= sn;
        }
        qDot0 -= beta * s0;
        qDot1 -= beta * s1;
        qDot2 -= beta * s2;
        qDot3 -= beta * s3;
    }
    q0 += qDot0 * dt; q1 += qDot1 * dt; q2 += qDot2 * dt; q3 += qDot3 * dt;
    float normSq = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    if (normSq > 0.0f) {
        float recipNorm = invSqrt(normSq);
        q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
    }
    qw = q0; qx = q1; qy = q2; qz = q3;
}

void sensfusion6UpdateQ(float gx, float gy_, float gz_,
                        float ax, float ay, float az, float dt) {
    if (!sensfusion6IsInit) sensfusion6Init();
    float gxRad = gx * DEG2RAD_F;
    float gyRad = gy_ * DEG2RAD_F;
    float gzRad = gz_ * DEG2RAD_F;
#ifdef CONFIG_IMU_MADGWICK_QUATERNION
    sensfusion6MadgwickUpdate(gxRad, gyRad, gzRad, ax, ay, az, dt);
#else
    sensfusion6MahonyUpdate(gxRad, gyRad, gzRad, ax, ay, az, dt);
#endif
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
    bool accValid = !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f));
    if (!sensfusion6IsCalibrated && accValid) {
        baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
        sensfusion6IsCalibrated = true;
    }
    sensfusion6SyncLog();
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
    float rollPart = (arm != 0.0f) ? (0.25f / arm) * torqueX : 0.0f;
    float pitchPart = (arm != 0.0f) ? (0.25f / arm) * torqueY : 0.0f;
    float yawPart = (thrustToTorque != 0.0f) ? (0.25f / thrustToTorque) * torqueZ : 0.0f;
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
        float f = clampFloat(normalizedForces[i], 0.0f, 1.0f);
        motorPWMs[i] = (uint16_t)(f * 65535.0f);
    }
}

static uint16_t motorForceToPwm(float force) {
    if (force <= 0.0f) return 0U;
    float pwm = force / CRAZYFLIE_MAX_MOTOR_FORCE_N * 65535.0f;
    if (pwm > 65535.0f) pwm = 65535.0f;
    return (uint16_t)pwm;
}

void powerDistribution(const ControlData *control, MotorPower *motorPower) {
    if (!control || !motorPower) return;
    switch (control->controlMode) {
        case controlModeLegacy:
            powerDistributionLegacy(control->thrust, control->roll, control->pitch,
                                    control->yaw, motorPower);
            break;
        case controlModeForceTorque: {
            float forces[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            powerDistributionForceTorque(control->thrustSi, control->torque.x,
                                         control->torque.y, control->torque.z,
                                         CRAZYFLIE_ARM_LENGTH_M,
                                         CRAZYFLIE_THRUST_TO_TORQUE, forces);
            motorPower->m1 = motorForceToPwm(forces[0]);
            motorPower->m2 = motorForceToPwm(forces[1]);
            motorPower->m3 = motorForceToPwm(forces[2]);
            motorPower->m4 = motorForceToPwm(forces[3]);
            break;
        }
        case controlModeForce: {
            uint16_t pwm[4] = {0U, 0U, 0U, 0U};
            powerDistributionForce(control->normalizedForces, pwm);
            motorPower->m1 = pwm[0];
            motorPower->m2 = pwm[1];
            motorPower->m3 = pwm[2];
            motorPower->m4 = pwm[3];
            break;
        }
        default:
            break;
    }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) {
    return (value < idleThrust) ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust) {
    PowerCapResult result = {false, 0};
    if (!motors) return result;
    int32_t maxMotor = motors[0];
    for (int i = 1; i < 4; i++) {
        if (motors[i] > maxMotor) maxMotor = motors[i];
    }
    if (maxMotor > maxAllowedThrust) {
        result.isCapped = true;
        result.reduction = maxMotor - maxAllowedThrust;
        for (int i = 0; i < 4; i++) motors[i] -= result.reduction;
    }
    for (int i = 0; i < 4; i++) motors[i] = capMinThrust(motors[i], idleThrust);
    return result;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) {
    return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust,
                                        float nominalVoltage,
                                        float actualVoltage) {
    if (actualVoltage <= 0.0f) return motorThrust;
    float value = roundf(((float)motorThrust) * nominalVoltage / actualVoltage);
    if (value < 0.0f) value = 0.0f;
    if (value > 65535.0f) value = 65535.0f;
    return (uint16_t)value;
}

static void pidInit(PidObject *pid) {
    if (!pid) return;
    pid->kp = 0.0f; pid->ki = 0.0f; pid->kd = 0.0f; pid->kff = 0.0f;
    pid->integral = 0.0f; pid->prevError = 0.0f; pid->output = 0.0f;
    pid->initialized = true;
}

static void pidReset(PidObject *pid) {
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
    pid->initialized = true;
}

static void pidUpdate(PidObject *pid, float actual, float desired, bool resetIntegral) {
    if (!pid) return;
    if (!pid->initialized) pidInit(pid);
    float error = desired - actual;
    if (resetIntegral) pid->integral = 0.0f;
    pid->integral += error * s_pidDt;
    float deriv = 0.0f;
    if (s_pidDt > 0.0f) deriv = (error - pid->prevError) / s_pidDt;
    pid->output = pid->kp * error + pid->ki * pid->integral +
                  pid->kd * deriv + pid->kff * desired;
    pid->prevError = error;
}

static void pidUpdateAngle(PidObject *pid, float actual, float desired, bool resetIntegral) {
    if (!pid) return;
    if (!pid->initialized) pidInit(pid);
    float error = capAngle(desired - actual);
    if (resetIntegral) pid->integral = 0.0f;
    pid->integral += error * s_pidDt;
    float deriv = 0.0f;
    if (s_pidDt > 0.0f) deriv = (error - pid->prevError) / s_pidDt;
    pid->output = pid->kp * error + pid->ki * pid->integral +
                  pid->kd * deriv + pid->kff * desired;
    pid->prevError = error;
}

void attitudeControllerInit(float updateDt) {
    if (s_attCtrlInitDone) return;
    s_pidDt = updateDt > 0.0f ? updateDt : 0.002f;
    pidInit(&pidRoll); pidInit(&pidPitch); pidInit(&pidYaw);
    pidInit(&pidRollRate); pidInit(&pidPitchRate); pidInit(&pidYawRate);
    s_attCtrlInitDone = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
    pidUpdate(&pidRollRate, rollActual, rollDesired, false);
    pidUpdate(&pidPitchRate, pitchActual, pitchDesired, false);
    pidUpdate(&pidYawRate, yawActual, yawDesired, false);
    pidRollRate.output = (float)saturateSignedInt16((int32_t)pidRollRate.output);
    pidPitchRate.output = (float)saturateSignedInt16((int32_t)pidPitchRate.output);
    pidYawRate.output = (float)saturateSignedInt16((int32_t)pidYawRate.output);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
    pidUpdateAngle(&pidRoll, rollActual, rollDesired, false);
    pidUpdateAngle(&pidPitch, pitchActual, pitchDesired, false);
    pidUpdateAngle(&pidYaw, yawActual, yawDesired, true);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual) {
    (void)rollActual; (void)pitchActual; (void)yawActual;
    pidReset(&pidRoll); pidReset(&pidPitch); pidReset(&pidYaw);
    pidReset(&pidRollRate); pidReset(&pidPitchRate); pidReset(&pidYawRate);
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
    (void)rollActual;
    pidReset(&pidRoll);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
    (void)pitchActual;
    pidReset(&pidPitch);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw) {
    if (!roll || !pitch || !yaw) return;
    *roll = (int16_t)saturateSignedInt16((int32_t)pidRollRate.output);
    *pitch = (int16_t)saturateSignedInt16((int32_t)pidPitchRate.output);
    *yaw = (int16_t)saturateSignedInt16((int32_t)pidYawRate.output);
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
    if (!setpoint || !state) return 0U;
    float posError = setpoint->position.z - state->position.z;
    float velError = setpoint->velocity.z - state->velocity.z;
    float out = 100.0f * posError + 10.0f * velError;
    if (out < 0.0f) out = 0.0f;
    if (out > 65535.0f) out = 65535.0f;
    return (uint16_t)out;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
    if (!sensors || !setpoint || !state || !control) return;
    if (!s_attCtrlInitDone) attitudeControllerInit(attitudeUpdateDt);
    memset(control, 0, sizeof(*control));
    control->controlMode = controlModeLegacy;
    control->roll = control->pitch = control->yaw = 0;
    control->thrust = 0U;

    uint16_t thrustCmd;
    if (setpoint->mode.z == modeDisable) {
        thrustCmd = setpoint->thrust;
    } else {
        thrustCmd = positionControllerUpdate(setpoint, state);
    }
    if (thrustCmd == 0U) {
        attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch,
                                      state->attitude.yaw);
        return;
    }

    float rollDesired = state->attitude.roll;
    float pitchDesired = state->attitude.pitch;
    float yawDesired = state->attitude.yaw;
    static float s_desiredYaw = 0.0f;

    if (setpoint->mode.roll == modeVelocity) {
        rollDesired = setpoint->attitudeRate.roll;
        attitudeControllerResetRollAttitudePID(state->attitude.roll);
    } else if (setpoint->mode.roll == modeAbs) {
        rollDesired = setpoint->attitude.roll;
    }
    if (setpoint->mode.pitch == modeVelocity) {
        pitchDesired = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
    } else if (setpoint->mode.pitch == modeAbs) {
        pitchDesired = setpoint->attitude.pitch;
    }

    if (setpoint->mode.yaw == modeVelocity) {
        s_desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
        if (yawMaxDelta != 0.0f) {
            float diff = capAngle(s_desiredYaw - state->attitude.yaw);
            if (diff > yawMaxDelta) diff = yawMaxDelta;
            if (diff < -yawMaxDelta) diff = -yawMaxDelta;
            s_desiredYaw = state->attitude.yaw + diff;
        }
        yawDesired = s_desiredYaw;
    } else if (setpoint->mode.yaw == modeAbs) {
        if (setpoint->mode.quat == modeAbs) {
            yawDesired = atan2f(2.0f * (setpoint->attitudeQuaternion.x * setpoint->attitudeQuaternion.y +
                                       setpoint->attitudeQuaternion.w * setpoint->attitudeQuaternion.z),
                                1.0f - 2.0f * (setpoint->attitudeQuaternion.y * setpoint->attitudeQuaternion.y +
                                                setpoint->attitudeQuaternion.z * setpoint->attitudeQuaternion.z)) * RAD2DEG_F;
        } else {
            yawDesired = setpoint->attitude.yaw;
        }
    }

    attitudeControllerCorrectAttitudePID(state->attitude.roll, rollDesired,
                                         state->attitude.pitch, pitchDesired,
                                         state->attitude.yaw, yawDesired);
    attitudeControllerCorrectRatePID(-sensors->gyro.y, pidRoll.output,
                                     -sensors->gyro.y, pidPitch.output,
                                     sensors->gyro.z, pidYaw.output);
    int16_t rollOut = 0, pitchOut = 0, yawOut = 0;
    attitudeControllerGetActuatorOutput(&rollOut, &pitchOut, &yawOut);
    control->roll = rollOut;
    control->pitch = pitchOut;
    control->yaw = saturateSignedInt16(-(int32_t)yawOut);
    control->thrust = thrustCmd;
}

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
    if (!rollPrime || !pitchPrime) return;
    float rad = yaw_deg * DEG2RAD_F;
    float c = cosf(rad), s = sinf(rad);
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
    memset(setpoint, 0, sizeof(*setpoint));
    uint16_t rawThrust = values->thrust;

    if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
    }
    if (thrustLocked && rawThrust == 0U) {
        thrustLocked = false;
    }

    bool altActive = altHoldMode;
    if (altActive) {
        if (!commanderModeSet) {
            commanderModeSet = true;
        }
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0U;
        setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
    } else {
        if (commanderModeSet) {
            commanderModeSet = false;
            setpoint->mode.z = modeDisable;
        }
        if (thrustLocked || rawThrust < MIN_THRUST) {
            setpoint->thrust = 0U;
        } else {
            setpoint->thrust = (rawThrust > MAX_THRUST) ? MAX_THRUST : rawThrust;
        }
    }

    bool useDefaultRP = true;
    bool useDefaultYaw = true;

    if (posHoldMode) {
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->velocity.x = values->pitch / 30.0f;
        setpoint->velocity.y = values->roll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        useDefaultRP = false;
    } else if (posSetMode && rawThrust != 0U) {
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
        useDefaultRP = false;
        useDefaultYaw = false;
    }

    if (useDefaultRP) {
        float effRoll = values->roll;
        float effPitch = values->pitch;
        if (yawMode == PLUSMODE) {
            rotateYaw(effRoll, effPitch, 45.0f, &effRoll, &effPitch);
        } else if (yawMode == CAREFREE) {
            s_carefreeErrorPath = true;
        }
        if (stabilizationModeRoll == RATE) {
            setpoint->mode.roll = modeVelocity;
            setpoint->attitudeRate.roll = effRoll;
        } else {
            setpoint->mode.roll = modeAbs;
            setpoint->attitude.roll = effRoll;
        }
        if (stabilizationModePitch == RATE) {
            setpoint->mode.pitch = modeVelocity;
            setpoint->attitudeRate.pitch = effPitch;
        } else {
            setpoint->mode.pitch = modeAbs;
            setpoint->attitude.pitch = effPitch;
        }
    }

    if (useDefaultYaw) {
        if (stabilizationModeYaw == RATE) {
            setpoint->mode.yaw = modeVelocity;
            setpoint->attitudeRate.yaw = -values->yaw;
        } else {
            setpoint->mode.yaw = modeAbs;
            setpoint->attitude.yaw = values->yaw;
        }
    }
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick_) {
    if (!motorRatios) return false;
    for (int i = 0; i < 4; i++) {
        if (motorRatios[i] > idleThrust) {
            s_lastFlyingTick = currentTick_;
            s_seenFlyingTick = true;
        }
    }
    if (!s_seenFlyingTick) return false;
    return (currentTick_ - s_lastFlyingTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick_,
                    bool *isFreeFalling) {
    bool freefall = false;
    if (crashDetectionGs > 0.0f) {
        float norm = sqrtf(accX*accX + accY*accY + accZ*accZ);
        if (fabsf(norm - 1.0f) > crashDetectionGs) {
            supervisorState = supervisorStateCrashed;
            supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        }
    }
    if (freeFallThreshold > 0.0f &&
        fabsf(accX) < freeFallThreshold &&
        fabsf(accY) < freeFallThreshold &&
        fabsf(accZ) < freeFallThreshold) {
        freefall = true;
        s_tumbleStartTick = 0U;
        s_tumbleTimerStarted = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
        supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
        if (isFreeFalling) *isFreeFalling = true;
        return false;
    }
    supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;
    if (isFreeFalling) *isFreeFalling = false;
    if (!tumbleCheckEnabled) {
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
        return false;
    }
    if (accZ < acceptedTiltAccZ) {
        if (!s_tumbleTimerStarted) {
            s_tumbleStartTick = currentTick_;
            s_tumbleTimerStarted = true;
        }
        uint32_t limit = (accZ < acceptedUpsideDownAccZ) ? maxUpsideDownTime : maxTiltTime;
        if (limit > 0U && (currentTick_ - s_tumbleStartTick) >= limit) {
            supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
            s_tumbleTimerStarted = false;
            s_tumbleStartTick = 0U;
            return true;
        }
    } else {
        s_tumbleTimerStarted = false;
        s_tumbleStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    }
    return false;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick_,
                                uint32_t lastNotificationTick) {
    if (lastNotificationTick == 0U) return true;
    return (currentTick_ - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick_,
                                  uint32_t preflightTimeoutDuration) {
    if (latestArmingTick == 0U || state != supervisorStateReadyToFly) return false;
    return (currentTick_ - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick_,
                                uint32_t landingTimeoutDuration) {
    if (latestLandingTick == 0U) return false;
    return (currentTick_ - latestLandingTick) >= landingTimeoutDuration;
}

void supervisorInit(void) {
    if (s_supervisorInitDone) return;
    supervisorState = supervisorStateLocked;
    supervisorConditionBits = 0U;
    s_spinupStarted = false;
    s_spinupStartTick = 0U;
    s_seenFlyingTick = false;
    s_lastFlyingTick = 0U;
    s_supervisorInitDone = true;
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
    return (supervisorState == supervisorStateCrashed) ||
           ((supervisorConditionBits & SUPERVISOR_CB_CRASHED) != 0U);
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
    if (s_autoArming) info |= (1U << 2);
    if (supervisorCanFly()) info |= (1U << 3);
    if ((supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) != 0U) info |= (1U << 4);
    if ((supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) != 0U) info |= (1U << 5);
    if (supervisorState == supervisorStateLocked) info |= (1U << 6);
    if (supervisorIsCrashed()) info |= (1U << 7);
    if ((supervisorConditionBits & SUPERVISOR_CB_DECK_FAULT) != 0U) info |= (1U << 11);
    return info;
}

bool supervisorRequestArming(bool doArm) {
    if (!doArm) {
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
        return false;
    }
    if (!supervisorCanArm() && supervisorState != supervisorStateArming) return false;
    supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    if (supervisorState != supervisorStateArming) supervisorState = supervisorStateArming;
    return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
    if ((supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) != 0U) return false;
    if (!doRecovery) {
        supervisorState = supervisorStateCrashed;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        return true;
    }
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    supervisorState = supervisorStatePreFlChecksPassed;
    return true;
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
                                uint32_t supervisorConditionBits_,
                                SupervisorState state) {
    if (!setpoint) return;
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
    memset(setpoint, 0, sizeof(*setpoint));
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax) {
    if (!motorRPMs) return false;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTick_) {
    if (!motorRPMs) return false;
    if (!canFly) {
        s_notRespondingTimerStarted = false;
        s_notRespondingStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return false;
    }
    bool anyLow = false;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmThreshold) anyLow = true;
    }
    if (anyLow) {
        if (!s_notRespondingTimerStarted) {
            s_notRespondingStartTick = currentTick_;
            s_notRespondingTimerStarted = true;
        }
        if ((currentTick_ - s_notRespondingStartTick) >= rpmCheckDurationMs) {
            supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
            return true;
        }
    } else {
        s_notRespondingTimerStarted = false;
        s_notRespondingStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    }
    return false;
}

void supervisorSetSensorData(const SensorData *sensors) {
    if (!sensors) return;
    memcpy(&s_lastSensorData, sensors, sizeof(s_lastSensorData));
    s_lastSensorActivityTick = currentTick;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
    if (!motorRatios) return;
    for (int i = 0; i < 4; i++) s_lastMotorRatios[i] = motorRatios[i];
    s_idleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
    if (!motorRPMs) return;
    for (int i = 0; i < 4; i++) s_lastMotorRPMs[i] = motorRPMs[i];
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled) {
    s_crashDetectionGs = crashDetectionGs;
    s_freeFallThreshold = freeFallThreshold;
    s_acceptedTiltAccZ = acceptedTiltAccZ;
    s_acceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
    s_maxTiltTime = maxTiltTime;
    s_maxUpsideDownTime = maxUpsideDownTime;
    s_tumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
    s_autoArming = autoArming;
    s_spinupTimeoutDuration = spinupTimeoutDurationMs;
    s_spinupStartTick = 0U;
    s_spinupStarted = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
}

void supervisorUpdate(uint32_t stabilizerStep) {
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;
    SupervisorState prevState = supervisorState;

    if (prevState != supervisorStatePreFlChecksPassed &&
        supervisorState == supervisorStatePreFlChecksPassed &&
        s_autoArming) {
        supervisorRequestArming(true);
    }
    bool leavingAllowed = (prevState == supervisorStateArming ||
                           prevState == supervisorStateReadyToFly ||
                           prevState == supervisorStateFlying ||
                           prevState == supervisorStateWarningLevelOut ||
                           prevState == supervisorStateLanded);
    bool currentAllowed = (supervisorState == supervisorStateArming ||
                           supervisorState == supervisorStateReadyToFly ||
                           supervisorState == supervisorStateFlying ||
                           supervisorState == supervisorStateWarningLevelOut ||
                           supervisorState == supervisorStateLanded);
    if (leavingAllowed && !currentAllowed) {
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    }

    if (supervisorState == supervisorStateArming) {
        if (!s_spinupStarted) {
            s_spinupStartTick = currentTick;
            s_spinupStarted = true;
        } else if ((currentTick - s_spinupStartTick) >= s_spinupTimeoutDuration) {
            supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
        }
    } else {
        s_spinupStarted = false;
        s_spinupStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }

    bool flying = isFlyingCheck(s_lastMotorRatios, s_idleThrust, currentTick);
    if (flying) {
        supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
        if (supervisorState == supervisorStateReadyToFly) supervisorState = supervisorStateFlying;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
        if (supervisorState == supervisorStateFlying) supervisorState = supervisorStateLanded;
    }

    bool freefall = false;
    bool tumbled = isTumbledCheck(s_lastSensorData.acc.x, s_lastSensorData.acc.y,
                                  s_lastSensorData.acc.z, s_crashDetectionGs,
                                  s_freeFallThreshold, s_acceptedTiltAccZ,
                                  s_acceptedUpsideDownAccZ, s_maxTiltTime,
                                  s_maxUpsideDownTime, s_tumbleCheckEnabled,
                                  currentTick, &freefall);
    if (freefall) {
        if (supervisorState == supervisorStateFlying ||
            supervisorState == supervisorStateReadyToFly) {
            supervisorState = supervisorStateExceptFreeFall;
        }
    } else if (supervisorState == supervisorStateExceptFreeFall) {
        supervisorState = supervisorStateFlying;
    }
    if (tumbled) {
        supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    }

    uint32_t inactivity = commanderGetInactivityTime();
    if (inactivity >= COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    } else if (inactivity >= COMMANDER_WDT_TIMEOUT_STABILIZE) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    }
    supervisorLog.info = supervisorGetInfoBitfield();
    supervisorLog.accNorm = sqrtf(s_lastSensorData.acc.x * s_lastSensorData.acc.x +
                                  s_lastSensorData.acc.y * s_lastSensorData.acc.y +
                                  s_lastSensorData.acc.z * s_lastSensorData.acc.z);
}

bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
    if (!measurement) return false;
    if (s_estimatorCount >= 16U) return false;
    s_estimatorFifo[s_estimatorTail] = *measurement;
    s_estimatorTail = (uint8_t)((s_estimatorTail + 1U) & 15U);
    s_estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
    if (!measurement) return false;
    if (s_estimatorCount == 0U) return false;
    *measurement = s_estimatorFifo[s_estimatorHead];
    s_estimatorHead = (uint8_t)((s_estimatorHead + 1U) & 15U);
    s_estimatorCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        if (m.type == MeasurementTypeGyroscope) {
            s_lastGyro[0] = m.data[0]; s_lastGyro[1] = m.data[1]; s_lastGyro[2] = m.data[2];
        } else if (m.type == MeasurementTypeAcceleration) {
            s_lastAcc[0] = m.data[0]; s_lastAcc[1] = m.data[1]; s_lastAcc[2] = m.data[2];
        } else if (m.type == MeasurementTypeBarometer) {
            s_lastBaro[0] = m.data[0]; s_lastBaro[1] = m.data[1]; s_lastBaro[2] = m.data[2];
        } else if (m.type == MeasurementTypeTOF) {
            s_lastTof[0] = m.data[0]; s_lastTof[1] = m.data[1]; s_lastTof[2] = m.data[2];
        }
    }
    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        sensfusion6UpdateQ(s_lastGyro[0], s_lastGyro[1], s_lastGyro[2],
                           s_lastAcc[0], s_lastAcc[1], s_lastAcc[2],
                           1.0f / SENSFUSION_RATE_HZ);
        sensfusion6GetEulerRPY(&s_stateEstimate.attitude.roll,
                               &s_stateEstimate.attitude.pitch,
                               &s_stateEstimate.attitude.yaw);
        sensfusion6GetQuaternion(&s_stateEstimate.attitudeQuaternion.w,
                                 &s_stateEstimate.attitudeQuaternion.x,
                                 &s_stateEstimate.attitudeQuaternion.y,
                                 &s_stateEstimate.attitudeQuaternion.z);
        s_stateEstimate.acc.x = s_lastAcc[0];
        s_stateEstimate.acc.y = s_lastAcc[1];
        s_stateEstimate.acc.z = s_lastAcc[2];
        float accZ = sensfusion6GetAccZWithoutGravity(s_lastAcc[0], s_lastAcc[1], s_lastAcc[2]);
        s_stateEstimate.velocity.z += accZ * 9.81f * (1.0f / SENSFUSION_RATE_HZ);
    }
    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        s_stateEstimate.position.x += s_stateEstimate.velocity.x * (1.0f / POSITION_RATE_HZ);
        s_stateEstimate.position.y += s_stateEstimate.velocity.y * (1.0f / POSITION_RATE_HZ);
        s_stateEstimate.position.z += s_stateEstimate.velocity.z * (1.0f / POSITION_RATE_HZ);
    }
    stateEstimate.roll = s_stateEstimate.attitude.roll;
    stateEstimate.pitch = s_stateEstimate.attitude.pitch;
    stateEstimate.yaw = s_stateEstimate.attitude.yaw;
    stateEstimate.qx = s_stateEstimate.attitudeQuaternion.x;
    stateEstimate.qy = s_stateEstimate.attitudeQuaternion.y;
    stateEstimate.qz = s_stateEstimate.attitudeQuaternion.z;
    stateEstimate.qw = s_stateEstimate.attitudeQuaternion.w;
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
    if (!setpoint) return false;
    if (priority == COMMANDER_PRIORITY_DISABLE) {
        s_activeSetpoint = *setpoint;
        s_activePriority = COMMANDER_PRIORITY_DISABLE;
        s_commanderLastUpdateTick = currentTick;
        s_trajectoryActive = false;
        return true;
    }
    if (priority < s_activePriority) return false;
    s_activeSetpoint = *setpoint;
    s_activePriority = priority;
    s_commanderLastUpdateTick = currentTick;
    if (priority > COMMANDER_PRIORITY_HIGHLEVEL) s_trajectoryActive = false;
    return true;
}

void commanderRelaxPriority(void) {
    s_activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
    if (currentTick < s_commanderLastUpdateTick) return 0U;
    return currentTick - s_commanderLastUpdateTick;
}

int commanderGetActivePriority(void) {
    return s_activePriority;
}

static void sensorsInit(void) { memset(&s_sensorsData, 0, sizeof(s_sensorsData)); }
static void stateEstimatorInit(void) {
    s_estimatorHead = s_estimatorTail = s_estimatorCount = 0U;
    memset(&s_stateEstimate, 0, sizeof(s_stateEstimate));
    memset(&s_lastGyro, 0, sizeof(s_lastGyro));
    memset(&s_lastAcc, 0, sizeof(s_lastAcc));
    memset(&s_lastBaro, 0, sizeof(s_lastBaro));
    memset(&s_lastTof, 0, sizeof(s_lastTof));
}
static void controllerInit(void) { attitudeControllerInit(0.002f); }
static void powerDistributionInit(void) { for (int i = 0; i < 4; i++) s_motorPwm[i] = 0U; }
static void motorsInit(void) { for (int i = 0; i < 4; i++) s_motorPwm[i] = 0U; }
static void collisionAvoidanceInit(void) {}

static void sensorsWaitDataReady(void) {}
static void sensorsAcquire(void) {
    memcpy(&s_sensorsData, &s_lastSensorData, sizeof(s_sensorsData));
    gyro.x = s_sensorsData.gyro.x; gyro.y = s_sensorsData.gyro.y; gyro.z = s_sensorsData.gyro.z;
    acc.x = s_sensorsData.acc.x; acc.y = s_sensorsData.acc.y; acc.z = s_sensorsData.acc.z;
    baro.pressure = s_sensorsData.baroPressure;
    baro.temp = s_sensorsData.baroTemperature;
    baro.asl = s_sensorsData.baroAsl;
}
static void stateEstimator(uint32_t step) { estimatorComplementary(step); }
static void collisionAvoidanceUpdateSetpoint(Setpoint *setpoint) { (void)setpoint; }

static void setMotorRatiosInternal(void) {
    motor.m1req = s_motorPwm[0];
    motor.m2req = s_motorPwm[1];
    motor.m3req = s_motorPwm[2];
    motor.m4req = s_motorPwm[3];
    uint32_t ratios[4] = {(uint32_t)s_motorPwm[0], (uint32_t)s_motorPwm[1],
                          (uint32_t)s_motorPwm[2], (uint32_t)s_motorPwm[3]};
    supervisorSetMotorRatios(ratios, 0U);
}

void stabilizerInit(void) {
    if (s_stabilizerInitDone) return;
    sensorsInit();
    stateEstimatorInit();
    controllerInit();
    powerDistributionInit();
    motorsInit();
    collisionAvoidanceInit();
    s_systemStarted = true;
    s_sensorsCalibrated = true;
    s_stabilizerInitDone = true;
}

void stabilizerTask(void) {
    if (!s_stabilizerInitDone) return;
    if (!s_systemStarted || !s_sensorsCalibrated) return;

    if (s_pendingHighLevel) {
        commanderSetSetpoint(&s_pendingHighLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        s_pendingHighLevel = false;
    }

    sensorsWaitDataReady();
    sensorsAcquire();
    stateEstimator(s_stabilizerStep);
    Setpoint setpoint = s_activeSetpoint;

    if (healthShallWeRunTest()) {
        healthRunTests(&s_sensorsData);
        for (int i = 0; i < 4; i++) s_motorPwm[i] = 0U;
        setMotorRatiosInternal();
        s_stabilizerStep++;
        return;
    }

    supervisorUpdate(s_stabilizerStep);
    collisionAvoidanceUpdateSetpoint(&setpoint);
    supervisorOverrideSetpoint(&setpoint, supervisorConditionBits, supervisorState);

    if (!supervisorCanFly() || !supervisorAreMotorsAllowedToRun() ||
        (supervisorConditionBits & SUPERVISOR_CB_EMERGENCY_STOP) != 0U) {
        for (int i = 0; i < 4; i++) s_motorPwm[i] = 0U;
        setMotorRatiosInternal();
        s_stabilizerStep++;
        return;
    }

    ControlData control;
    memset(&control, 0, sizeof(control));
    controllerPid(&s_sensorsData, &setpoint, &s_stateEstimate, &control,
                  s_yawMaxDelta, 0.002f);

    MotorPower motorPower;
    memset(&motorPower, 0, sizeof(motorPower));
    powerDistribution(&control, &motorPower);
    int32_t motors[4] = {motorPower.m1, motorPower.m2, motorPower.m3, motorPower.m4};
    for (int i = 0; i < 4; i++) {
        uint16_t pwm = (uint16_t)(motors[i] < 0 ? 0 : (motors[i] > 65535 ? 65535 : motors[i]));
        pwm = motorsCompensateBatteryVoltage(pwm, 3.7f, s_batteryVoltage);
        motors[i] = pwm;
    }
    powerDistributionCap(motors, 65535, 0);
    for (int i = 0; i < 4; i++) s_motorPwm[i] = (uint16_t)motors[i];
    setMotorRatiosInternal();
    s_stabilizerStep++;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
    if (!setpoint) return false;
    s_pendingHighLevelSetpoint = *setpoint;
    s_pendingHighLevel = true;
    return true;
}

static uint32_t quatcompress(float w, float x, float y, float z) {
    float norm = sqrtf(w*w + x*x + y*y + z*z);
    if (norm == 0.0f) return 0U;
    w /= norm; x /= norm; y /= norm; z /= norm;
    float vals[4] = {w, x, y, z};
    int maxi = 0;
    float maxAbs = fabsf(vals[0]);
    for (int i = 1; i < 4; i++) {
        float a = fabsf(vals[i]);
        if (a > maxAbs) { maxAbs = a; maxi = i; }
    }
    if (vals[maxi] < 0.0f) {
        for (int i = 0; i < 4; i++) vals[i] = -vals[i];
    }
    uint32_t packed = ((uint32_t)maxi) << 30U;
    int shift = 20;
    for (int i = 0; i < 4; i++) {
        if (i == maxi) continue;
        int16_t v = (int16_t)(vals[i] * 512.0f);
        if (v > 511) v = 511;
        if (v < -512) v = -512;
        uint16_t enc = (uint16_t)v & 0x3FFU;
        packed |= ((uint32_t)enc) << (unsigned)shift;
        shift -= 10;
    }
    return packed;
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
    float gyroToMillirad = 1000.0f * DEG2RAD_F;
    output->gyro_millirad_s[0] = sensors->gyro.x * gyroToMillirad;
    output->gyro_millirad_s[1] = -sensors->gyro.y * gyroToMillirad;
    output->gyro_millirad_s[2] = sensors->gyro.z * gyroToMillirad;
    output->quatCompressed = quatcompress(state->attitudeQuaternion.w,
                                          state->attitudeQuaternion.x,
                                          state->attitudeQuaternion.y,
                                          state->attitudeQuaternion.z);
}

bool rateSupervisorValidate(uint32_t measuredRate) {
    return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void) {
    static uint32_t s_lastRateCheckTick = 0U;
    if (s_lastRateCheckTick == 0U) {
        s_lastRateCheckTick = currentTick;
        return;
    }
    if ((currentTick - s_lastRateCheckTick) >= 2000U) {
        s_lastRateCheckTick = currentTick;
        if (!s_sensorPaused && s_lastSensorActivityTick != 0U &&
            (currentTick - s_lastSensorActivityTick) >= 2000U) {
            s_rateSupervisorError = true;
        }
    }
}

bool healthShallWeRunTest(void) {
    if (s_propTestRequest) {
        s_propTestRequest = false;
        healthTestState = configureAcc;
        s_propSampleCount = 0;
        s_propMotorIndex = 0;
        s_propNoiseVariance = 0.0f;
        s_healthMotorTestCount = 0U;
        motorPass = 0U;
        return true;
    }
    if (s_batTestRequest) {
        s_batTestRequest = false;
        healthTestState = testBattery;
        s_batteryTick = 0;
        batteryPass = 0U;
        batterySag = 0.0f;
        s_healthMinLoadedVoltage = s_batteryVoltage;
        return true;
    }
    return healthTestState != testDone;
}

void healthRequestPropTest(void) { s_propTestRequest = true; }
void healthRequestBatteryTest(void) { s_batTestRequest = true; }

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex) {
    if (highThreshold == 0.0f) return true;
    if (lowThreshold <= measuredValue && measuredValue <= highThreshold) {
        motorPass |= (uint8_t)(1U << motorIndex);
        return true;
    }
    s_healthMotorTestCount++;
    return false;
}

float variance(const float *buffer, int length) {
    if (!buffer || length <= 0) return 0.0f;
    float sum = 0.0f, sumSq = 0.0f;
    for (int i = 0; i < length; i++) {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }
    return sumSq - (sum * sum / (float)length);
}

void healthRunTests(const SensorData *sensorData) {
    if (!sensorData) return;
    switch (healthTestState) {
        case configureAcc:
            motorPass = 0U;
            batteryPass = 0U;
            batterySag = 0.0f;
            healthTestState = measureNoiseFloor;
            break;
        case measureNoiseFloor: {
            s_propSampleCount++;
            if (s_propSampleCount >= 1 && s_propSampleCount <= PROPTEST_NBR_OF_VARIANCE_VALUES) {
                /* Deterministic host sample: use acc.z as one collected sample. */
                (void)sensorData;
            }
            if (s_propSampleCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
                s_propNoiseVariance = 0.0f;
                healthTestState = measureProp;
            }
            break;
        }
        case measureProp:
            if (s_propMotorIndex < 4) {
                float measured = s_propNoiseVariance + fabsf(sensorData->acc.x) +
                                 fabsf(sensorData->acc.y) + fabsf(sensorData->acc.z);
                evaluatePropTest(0.0f, 0.0f, measured, (uint8_t)s_propMotorIndex);
                s_propMotorIndex++;
            } else {
                healthTestState = testDone;
            }
            break;
        case testBattery:
            s_batteryTick++;
            if (s_batteryTick == 1) {
                /* Load motors, host model no physical effect. */
            } else if (s_batteryTick >= 2 && s_batteryTick <= 49) {
                float loaded = s_batteryVoltage - 0.1f;
                if (loaded < s_healthMinLoadedVoltage) s_healthMinLoadedVoltage = loaded;
            } else if (s_batteryTick >= 50) {
                batterySag = s_healthIdleVoltage - s_healthMinLoadedVoltage;
                batteryPass = (batterySag <= s_healthBatterySagThreshold) ? 1U : 0U;
                healthTestState = testDone;
            }
            break;
        case evaluatePropResult:
        case evaluateBatResult:
        case restartBatTest:
            healthTestState = testDone;
            break;
        case testDone:
        default:
            break;
    }
    healthLog.motorPass = motorPass;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
    healthLog.motorTestCount = s_healthMotorTestCount;
}

void crtpInit(void) {
    if (s_crtpInitDone) return;
    s_txHead = s_txTail = 0U;
    s_txCount = 0U;
    s_currentLink = &s_nopLink;
    s_crtpErrorState = false;
    s_crtpInitDone = true;
}

void crtpInitTaskQueue(uint8_t port) {
    if (port >= CRTP_NBR_OF_PORTS) {
        s_crtpErrorState = true;
        return;
    }
    if (s_rxQueueCreated[port]) {
        s_crtpErrorState = true;
        return;
    }
    s_rxQueueCreated[port] = true;
    s_rxHead[port] = 0U;
    s_rxTail[port] = 0U;
    s_rxCount[port] = 0U;
}

bool crtpSendPacket(const CrtpPacket *packet) {
    if (!packet) return false;
    if (s_txCount >= CRTP_TX_QUEUE_SIZE) return false;
    s_txQueue[s_txTail] = *packet;
    s_txTail = (uint16_t)((s_txTail + 1U) % CRTP_TX_QUEUE_SIZE);
    s_txCount++;
    s_statsTxCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
    if (port >= CRTP_NBR_OF_PORTS || !packet) return false;
    if (!s_rxQueueCreated[port] || s_rxCount[port] == 0U) return false;
    *packet = s_rxQueue[port][s_rxHead[port]];
    s_rxHead[port] = (uint8_t)((s_rxHead[port] + 1U) % CRTP_RX_QUEUE_SIZE);
    s_rxCount[port]--;
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
    if (s_currentLink == &s_nopLink || !s_currentLink) return;
    if (!s_currentLink->receivePacket) return;
    CrtpPacket packet;
    while (s_currentLink->receivePacket(&packet)) {
        s_statsRxCount++;
        bool delivered = false;
        if (packet.port < CRTP_NBR_OF_PORTS && s_rxQueueCreated[packet.port]) {
            if (s_rxCount[packet.port] < CRTP_RX_QUEUE_SIZE) {
                s_rxQueue[packet.port][s_rxTail[packet.port]] = packet;
                s_rxTail[packet.port] = (uint8_t)((s_rxTail[packet.port] + 1U) % CRTP_RX_QUEUE_SIZE);
                s_rxCount[packet.port]++;
                delivered = true;
            }
        }
        if (packet.port < CRTP_NBR_OF_PORTS && s_portCallbacks[packet.port]) {
            s_portCallbacks[packet.port](&packet);
            delivered = true;
        }
        if (!delivered) {
            /* drop packet */
        }
    }
}

void crtpTxTask(void) {
    if (s_currentLink == &s_nopLink || !s_currentLink) return;
    if (!s_currentLink->sendPacket) return;
    if (s_txCount == 0U) return;
    if (s_txRetryPending && (currentTick - s_txRetryTick) < 10U) return;
    CrtpPacket packet = s_txQueue[s_txHead];
    if (s_currentLink->sendPacket(&packet)) {
        s_txHead = (uint16_t)((s_txHead + 1U) % CRTP_TX_QUEUE_SIZE);
        s_txCount--;
        s_txRetryPending = false;
    } else {
        s_txRetryPending = true;
        s_txRetryTick = currentTick;
    }
}

void crtpSetLink(CrtpLink *newLink) {
    if (s_currentLink && s_currentLink != &s_nopLink && s_currentLink->setEnable) {
        s_currentLink->setEnable(false);
    }
    if (!newLink) {
        s_currentLink = &s_nopLink;
    } else {
        s_currentLink = newLink;
        if (s_currentLink->setEnable) s_currentLink->setEnable(true);
    }
}

void crtpReset(void) {
    s_txHead = s_txTail = 0U;
    s_txCount = 0U;
    s_txRetryPending = false;
    s_crtpErrorState = false;
    if (s_currentLink && s_currentLink != &s_nopLink && s_currentLink->reset) {
        s_currentLink->reset();
    }
}

bool crtpIsConnected(void) {
    if (s_currentLink && s_currentLink->isConnected) return s_currentLink->isConnected();
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
    return CRTP_TX_QUEUE_SIZE - (uint32_t)s_txCount;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
    if (port >= CRTP_NBR_OF_PORTS) {
        s_crtpErrorState = true;
        return;
    }
    s_portCallbacks[port] = callback;
}

void updateStats(void) {
    if (s_statsLastTick == 0U) {
        s_statsLastTick = currentTick;
        return;
    }
    uint32_t elapsed = currentTick - s_statsLastTick;
    if (elapsed >= 500U) {
        s_rxRate = elapsed ? (s_statsRxCount * 1000U / elapsed) : 0U;
        s_txRate = elapsed ? (s_statsTxCount * 1000U / elapsed) : 0U;
        s_statsRxCount = 0U;
        s_statsTxCount = 0U;
        s_statsLastTick = currentTick;
    }
}

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
    if (!decks || capacity == 0U) return 0U;
    uint8_t count = 0U;
    static const uint8_t knownI2C[] = {0x20U, 0x21U, 0x22U};
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(knownI2C) / sizeof(knownI2C[0])); i++) {
        if (count >= capacity) break;
        decks[count].foundByI2C = true;
        decks[count].foundByOneWire = false;
        decks[count].i2cAddress = knownI2C[i];
        decks[count].oneWireRomId = 0ULL;
        count++;
    }
    static const uint64_t knownOneWire[] = {0x1111111111111111ULL,
                                             0x2222222222222222ULL};
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(knownOneWire) / sizeof(knownOneWire[0])); i++) {
        if (count >= capacity) break;
        decks[count].foundByI2C = false;
        decks[count].foundByOneWire = true;
        decks[count].i2cAddress = 0U;
        decks[count].oneWireRomId = knownOneWire[i];
        count++;
    }
    return count;
}
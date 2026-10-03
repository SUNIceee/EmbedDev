/*
 * Crazyflie host-verifiable C11 implementation.
 *
 * This translation unit contains no main() and has no dependency on
 * STM32, FreeRTOS, or board-specific headers.
 */

#include "6_generated_code.h"

#include <math.h>
#include <string.h>
#include <limits.h>

#define CF_PI 3.14159265358979323846f
#define CF_DEG_TO_RAD (CF_PI / 180.0f)
#define CF_RAD_TO_DEG (180.0f / CF_PI)
#define CF_EPSILON 1.0e-9f

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

PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;
PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;

bool thrustLocked = true;
bool commanderModeSet = false;

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0U;

TestState healthTestState = testDone;
uint8_t motorPass = 0U;
uint8_t batteryPass = 0U;
float batterySag = 0.0f;

StateEstimateLog stateEstimate;
Axis3Log gyro;
Axis3Log acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

/* ------------------------------------------------------------------------- */
/* Internal host state                                                       */
/* ------------------------------------------------------------------------- */

static float controllerDt = 0.002f;
static int16_t actuatorRoll;
static int16_t actuatorPitch;
static int16_t actuatorYaw;

static State estimatorState;
static SensorData estimatorSensors;
static bool estimatorHasGyro;
static bool estimatorHasAcc;
static bool estimatorHasBaro;
static bool estimatorHasTof;

static Setpoint commanderSetpoint;
static int commanderPriority = COMMANDER_PRIORITY_LOWEST;
static uint32_t commanderLastUpdate;
static uint32_t hostTick;

static Setpoint pendingHighLevelSetpoint;
static bool pendingHighLevelSetpointValid;

static SensorData supervisorSensors;
static uint32_t supervisorMotorRatios[4];
static int32_t supervisorMotorRPMs[4];
static uint32_t supervisorIdleThrust;
static bool supervisorHasSensorData;
static bool supervisorHasMotorRatios;
static bool supervisorHasMotorRPMs;
static bool supervisorArmed;
static bool supervisorCrashed;
static bool supervisorFreeFalling;
static bool supervisorAutoArming;
static uint32_t supervisorSpinupTimeout;
static uint32_t supervisorSpinupStart;
static uint32_t supervisorRecentFlightTick;
static bool supervisorHasFlown;
static uint32_t supervisorLastTiltTick;
static uint32_t supervisorLastUpsideDownTick;
static bool supervisorTiltTiming;
static bool supervisorUpsideDownTiming;
static float supervisorCrashGs;
static float supervisorFreeFallThreshold;
static float supervisorAcceptedTiltAccZ;
static float supervisorAcceptedUpsideDownAccZ;
static uint32_t supervisorMaxTiltTime;
static uint32_t supervisorMaxUpsideDownTime;
static bool supervisorTumbleEnabled;
static bool supervisorMotorsNotRespondingTiming;
static uint32_t supervisorMotorsNotRespondingStart;

static EstimatorMeasurement estimatorQueue[16];
static size_t estimatorQueueHead;
static size_t estimatorQueueTail;
static size_t estimatorQueueCount;

static CrtpPacket crtpTxQueue[CRTP_TX_QUEUE_SIZE];
static size_t crtpTxHead;
static size_t crtpTxTail;
static size_t crtpTxCount;

static CrtpPacket crtpRxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static size_t crtpRxHead[CRTP_NBR_OF_PORTS];
static size_t crtpRxTail[CRTP_NBR_OF_PORTS];
static size_t crtpRxCount[CRTP_NBR_OF_PORTS];
static bool crtpRxQueueCreated[CRTP_NBR_OF_PORTS];
static CrtpPortCallback crtpCallbacks[CRTP_NBR_OF_PORTS];

static uint32_t crtpRxPackets;
static uint32_t crtpTxPackets;
static uint32_t crtpLastStatsTick;
static uint32_t crtpRxRate;
static uint32_t crtpTxRate;

static bool crtpInitialized;
static bool crtpError;
static CrtpLink *crtpLink;

static bool healthPropRequested;
static bool healthBatteryRequested;
static uint32_t healthTick;
static uint32_t healthMotorTestCount;
static float healthIdleVoltage;
static float healthMinLoadedVoltage;
static float healthNoiseSamples[PROPTEST_NBR_OF_VARIANCE_VALUES];
static size_t healthNoiseCount;
static uint8_t healthCurrentMotor;
static uint32_t healthRestartTick;
static bool healthInitialized;

static bool stabilizerInitialized;
static bool stabilizerTasksCreated;
static bool stabilizerSystemStarted = true;
static bool stabilizerSensorsCalibrated = true;
static bool stabilizerHealthRequested;

/* ------------------------------------------------------------------------- */
/* Utility functions                                                         */
/* ------------------------------------------------------------------------- */

static float clampf(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static int32_t clampi32(int32_t value, int32_t low, int32_t high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static uint16_t clampu16i32(int32_t value)
{
    if (value <= 0) {
        return 0U;
    }
    if (value >= 65535) {
        return 65535U;
    }
    return (uint16_t)value;
}

static float vectorNorm(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static void zeroSetpoint(Setpoint *setpoint)
{
    if (setpoint != NULL) {
        memset(setpoint, 0, sizeof(*setpoint));
    }
}

static void updateFusionLog(void)
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

static void updateSupervisorLog(void)
{
    supervisorLog.info = supervisorConditionBits;
    supervisorLog.accNorm = vectorNorm(supervisorSensors.acc.x,
                                       supervisorSensors.acc.y,
                                       supervisorSensors.acc.z);
}

static void updateHealthLog(void)
{
    healthLog.motorPass = motorPass;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
    healthLog.motorTestCount = healthMotorTestCount;
}

/* ------------------------------------------------------------------------- */
/* Numeric helpers                                                           */
/* ------------------------------------------------------------------------- */

int16_t saturateSignedInt16(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < -INT16_MAX) {
        return (int16_t)-INT16_MAX;
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

/* ------------------------------------------------------------------------- */
/* Sensfusion6                                                               */
/* ------------------------------------------------------------------------- */

float invSqrt(float x)
{
    union {
        float f;
        uint32_t i;
    } value;
    float halfX;

    if (x <= 0.0f) {
        return 0.0f;
    }

    halfX = 0.5f * x;
    value.f = x;
    value.i = 0x5f3759dfU - (value.i >> 1U);
    value.f = value.f * (1.5f - halfX * value.f * value.f);
    return value.f;
}

void estimatedGravityDirection(float qwValue, float qxValue,
                               float qyValue, float qzValue,
                               float *gravX, float *gravY, float *gravZ)
{
    if (gravX != NULL) {
        *gravX = 2.0f * (qxValue * qzValue - qwValue * qyValue);
    }
    if (gravY != NULL) {
        *gravY = 2.0f * (qwValue * qxValue + qyValue * qzValue);
    }
    if (gravZ != NULL) {
        *gravZ = qwValue * qwValue - qxValue * qxValue
               - qyValue * qyValue + qzValue * qzValue;
    }
}

void sensfusion6Init(void)
{
    if (sensfusion6IsInit) {
        return;
    }

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
    updateFusionLog();
}

bool sensfusion6Test(void)
{
    return sensfusion6IsInit;
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
    float norm;
    float halfDt;
    float halfQw;
    float halfQx;
    float halfQy;
    float halfQz;
    float recipNorm;
    float errorX = 0.0f;
    float errorY = 0.0f;
    float errorZ = 0.0f;
    float qNorm;

    if (!sensfusion6IsInit) {
        sensfusion6Init();
    }
    if (dt <= 0.0f) {
        return;
    }

    halfQw = 0.5f * qw;
    halfQx = 0.5f * qx;
    halfQy = 0.5f * qy;
    halfQz = 0.5f * qz;

    norm = vectorNorm(ax, ay, az);
    if (norm > CF_EPSILON) {
        recipNorm = 1.0f / norm;
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        estimatedGravityDirection(qw, qx, qy, qz,
                                   &gravityX, &gravityY, &gravityZ);

#if defined(CONFIG_IMU_MADGWICK_QUATERNION)
        {
            float s0;
            float s1;
            float s2;
            float s3;
            float q2 = qw * qw;
            float qx2 = qx * qx;
            float qy2 = qy * qy;
            float qz2 = qz * qz;

            s0 = -2.0f * qy * (2.0f * (qx * qz - qw * qy) - ax)
               + 2.0f * qx * (2.0f * (qw * qx + qy * qz) - ay);
            s1 = 2.0f * qz * (2.0f * (qx * qz - qw * qy) - ax)
               + 2.0f * qw * (2.0f * (qw * qx + qy * qz) - ay)
               - 4.0f * qx * (1.0f - 2.0f * (qx2 + qy2) - az);
            s2 = -2.0f * qw * (2.0f * (qx * qz - qw * qy) - ax)
               + 2.0f * qz * (2.0f * (qw * qx + qy * qz) - ay)
               - 4.0f * qy * (1.0f - 2.0f * (qx2 + qy2) - az);
            s3 = 2.0f * qx * (2.0f * (qx * qz - qw * qy) - ax)
               + 2.0f * qy * (2.0f * (qw * qx + qy * qz) - ay);

            norm = vectorNorm(s0, s1, s2);
            if (norm > CF_EPSILON) {
                recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
                s0 *= recipNorm;
                s1 *= recipNorm;
                s2 *= recipNorm;
                s3 *= recipNorm;
                gx -= beta * s1;
                gy -= beta * s2;
                gz -= beta * s3;
            }
        }
#else
        errorX = ay * gravityZ - az * gravityY;
        errorY = az * gravityX - ax * gravityZ;
        errorZ = ax * gravityY - ay * gravityX;

        if (twoKi > 0.0f) {
            integralFBx += twoKi * errorX * dt;
            integralFBy += twoKi * errorY * dt;
            integralFBz += twoKi * errorZ * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += twoKp * errorX;
        gy += twoKp * errorY;
        gz += twoKp * errorZ;
#endif

        if (!sensfusion6IsCalibrated) {
            baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
            sensfusion6IsCalibrated = true;
        }
    } else {
#if !defined(CONFIG_IMU_MADGWICK_QUATERNION)
        if (twoKi <= 0.0f) {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }
#endif
    }

    gx *= CF_DEG_TO_RAD;
    gy *= CF_DEG_TO_RAD;
    gz *= CF_DEG_TO_RAD;
    halfDt = 0.5f * dt;

    qw += (-halfQx * gx - halfQy * gy - halfQz * gz) * dt;
    qx += ( halfQw * gx + halfQy * gz - halfQz * gy) * dt;
    qy += ( halfQw * gy - halfQx * gz + halfQz * gx) * dt;
    qz += ( halfQw * gz + halfQx * gy - halfQy * gx) * dt;

    (void)halfDt;
    qNorm = vectorNorm(qw, qx, qy, qz);
    if (qNorm > CF_EPSILON) {
        qw /= qNorm;
        qx /= qNorm;
        qy /= qNorm;
        qz /= qNorm;
    } else {
        qw = 1.0f;
        qx = 0.0f;
        qy = 0.0f;
        qz = 0.0f;
    }

    estimatedGravityDirection(qw, qx, qy, qz,
                               &gravityX, &gravityY, &gravityZ);
    updateFusionLog();
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg,
                            float *yaw_deg)
{
    float gx;
    float gy;
    float gz;
    float roll;
    float pitch;
    float yaw;

    estimatedGravityDirection(qw, qx, qy, qz, &gx, &gy, &gz);
    gx = clampf(gx, -1.0f, 1.0f);

    roll = atan2f(gy, gz);
    pitch = -asinf(gx);
    yaw = atan2f(2.0f * (qw * qz + qx * qy),
                 1.0f - 2.0f * (qy * qy + qz * qz));

    if (roll_deg != NULL) {
        *roll_deg = roll * CF_RAD_TO_DEG;
    }
    if (pitch_deg != NULL) {
        *pitch_deg = pitch * CF_RAD_TO_DEG;
    }
    if (yaw_deg != NULL) {
        *yaw_deg = yaw * CF_RAD_TO_DEG;
    }
}

void sensfusion6GetQuaternion(float *qwOut, float *qxOut,
                              float *qyOut, float *qzOut)
{
    if (qwOut != NULL) {
        *qwOut = qw;
    }
    if (qxOut != NULL) {
        *qxOut = qx;
    }
    if (qyOut != NULL) {
        *qyOut = qy;
    }
    if (qzOut != NULL) {
        *qzOut = qz;
    }
}

float sensfusion6GetAccZ(float ax, float ay, float az)
{
    estimatedGravityDirection(qw, qx, qy, qz,
                              &gravityX, &gravityY, &gravityZ);
    updateFusionLog();
    return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az)
{
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* ------------------------------------------------------------------------- */
/* Power distribution                                                        */
/* ------------------------------------------------------------------------- */

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out)
{
    int32_t r;
    int32_t p;
    int32_t t;
    int32_t y;

    if (out == NULL) {
        return;
    }

    r = (int32_t)roll / 2;
    p = (int32_t)pitch / 2;
    t = (int32_t)thrust;
    y = (int32_t)yaw;

    out->m1 = t - r + p + y;
    out->m2 = t - r - p - y;
    out->m3 = t + r - p + y;
    out->m4 = t + r + p - y;
}

void powerDistributionForceTorque(float thrustSi, float torqueX,
                                  float torqueY, float torqueZ,
                                  float armLength, float thrustToTorque,
                                  float motorForces[4])
{
    float thrustPart;
    float arm;
    float rollPart = 0.0f;
    float pitchPart = 0.0f;
    float yawPart = 0.0f;

    if (motorForces == NULL) {
        return;
    }

    thrustPart = 0.25f * thrustSi;
    arm = 0.707106781f * armLength;

    if (fabsf(arm) > CF_EPSILON) {
        rollPart = 0.25f / arm * torqueX;
        pitchPart = 0.25f / arm * torqueY;
    }
    if (fabsf(thrustToTorque) > CF_EPSILON) {
        yawPart = 0.25f / thrustToTorque * torqueZ;
    }

    motorForces[0] = fmaxf(0.0f, thrustPart - rollPart + pitchPart + yawPart);
    motorForces[1] = fmaxf(0.0f, thrustPart - rollPart - pitchPart - yawPart);
    motorForces[2] = fmaxf(0.0f, thrustPart + rollPart - pitchPart + yawPart);
    motorForces[3] = fmaxf(0.0f, thrustPart + rollPart + pitchPart - yawPart);
}

void powerDistributionForce(const float normalizedForces[4],
                            uint16_t motorPWMs[4])
{
    size_t i;

    if (normalizedForces == NULL || motorPWMs == NULL) {
        return;
    }

    for (i = 0U; i < 4U; ++i) {
        float force = clampf(normalizedForces[i], 0.0f, 1.0f);
        motorPWMs[i] = (uint16_t)lroundf(force * 65535.0f);
    }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower)
{
    if (control == NULL || motorPower == NULL) {
        return;
    }

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
        motorPower->m1 = (int32_t)lroundf(
            clampf(forces[0] / CRAZYFLIE_MAX_MOTOR_FORCE_N, 0.0f, 1.0f)
            * 65535.0f);
        motorPower->m2 = (int32_t)lroundf(
            clampf(forces[1] / CRAZYFLIE_MAX_MOTOR_FORCE_N, 0.0f, 1.0f)
            * 65535.0f);
        motorPower->m3 = (int32_t)lroundf(
            clampf(forces[2] / CRAZYFLIE_MAX_MOTOR_FORCE_N, 0.0f, 1.0f)
            * 65535.0f);
        motorPower->m4 = (int32_t)lroundf(
            clampf(forces[3] / CRAZYFLIE_MAX_MOTOR_FORCE_N, 0.0f, 1.0f)
            * 65535.0f);
        break;
    }

    case controlModeForce: {
        uint16_t pwm[4];
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

int32_t capMinThrust(int32_t value, int32_t idleThrust)
{
    return value < idleThrust ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust)
{
    PowerCapResult result = { false, 0 };
    int32_t maximum;
    int32_t reduction;
    size_t i;

    if (motors == NULL) {
        return result;
    }

    maximum = motors[0];
    for (i = 1U; i < 4U; ++i) {
        if (motors[i] > maximum) {
            maximum = motors[i];
        }
    }

    if (maximum > maxAllowedThrust) {
        reduction = maximum - maxAllowedThrust;
        result.isCapped = true;
        result.reduction = reduction;

        for (i = 0U; i < 4U; ++i) {
            motors[i] = capMinThrust(motors[i] - reduction, idleThrust);
        }
    }

    return result;
}

float batteryCompensation(float supplyVoltage, float filteredOld,
                          float alpha)
{
    return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust,
                                        float nominalVoltage,
                                        float actualVoltage)
{
    float compensated;

    if (actualVoltage <= 0.0f) {
        return motorThrust;
    }

    compensated = lroundf((float)motorThrust * nominalVoltage / actualVoltage);
    if (compensated <= 0.0f) {
        return 0U;
    }
    if (compensated >= 65535.0f) {
        return 65535U;
    }
    return (uint16_t)compensated;
}

/* ------------------------------------------------------------------------- */
/* PID controller                                                            */
/* ------------------------------------------------------------------------- */

static void pidInitialize(PidObject *pid)
{
    if (pid == NULL) {
        return;
    }

    memset(pid, 0, sizeof(*pid));
    pid->initialized = true;
}

static void pidReset(PidObject *pid, float actual)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->prevError = -actual;
    pid->output = 0.0f;
    pid->initialized = true;
}

static float pidUpdate(PidObject *pid, float actual, float desired,
                       float dt, bool reset)
{
    float error;
    float derivative;

    if (pid == NULL) {
        return 0.0f;
    }
    if (reset) {
        pidReset(pid, actual);
    }

    error = desired - actual;
    if (dt <= 0.0f) {
        dt = controllerDt;
    }

    pid->integral += error * dt;
    derivative = (error - pid->prevError) / dt;
    pid->prevError = error;
    pid->output = pid->kp * error
                + pid->ki * pid->integral
                + pid->kd * derivative
                + pid->kff * desired;
    return pid->output;
}

void attitudeControllerInit(float updateDt)
{
    if (updateDt > 0.0f) {
        controllerDt = updateDt;
    }

    pidInitialize(&pidRoll);
    pidInitialize(&pidPitch);
    pidInitialize(&pidYaw);
    pidInitialize(&pidRollRate);
    pidInitialize(&pidPitchRate);
    pidInitialize(&pidYawRate);
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired)
{
    actuatorRoll = saturateSignedInt16((int32_t)lroundf(
        pidUpdate(&pidRollRate, rollActual, rollDesired,
                  controllerDt, false)));
    actuatorPitch = saturateSignedInt16((int32_t)lroundf(
        pidUpdate(&pidPitchRate, pitchActual, pitchDesired,
                  controllerDt, false)));
    actuatorYaw = saturateSignedInt16((int32_t)lroundf(
        pidUpdate(&pidYawRate, yawActual, yawDesired,
                  controllerDt, false)));
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired)
{
    float desiredRollRate;
    float desiredPitchRate;
    float desiredYawRate;

    desiredRollRate = pidUpdate(&pidRoll, rollActual, rollDesired,
                                controllerDt, false);
    desiredPitchRate = pidUpdate(&pidPitch, pitchActual, pitchDesired,
                                 controllerDt, false);
    desiredYawRate = pidUpdate(&pidYaw, yawActual, yawDesired,
                               controllerDt, true);

    attitudeControllerCorrectRatePID(rollActual, desiredRollRate,
                                     pitchActual, desiredPitchRate,
                                     yawActual, desiredYawRate);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual)
{
    pidReset(&pidRoll, rollActual);
    pidReset(&pidPitch, pitchActual);
    pidReset(&pidYaw, yawActual);
    pidReset(&pidRollRate, 0.0f);
    pidReset(&pidPitchRate, 0.0f);
    pidReset(&pidYawRate, 0.0f);
}

void attitudeControllerResetRollAttitudePID(float rollActual)
{
    pidReset(&pidRoll, rollActual);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual)
{
    pidReset(&pidPitch, pitchActual);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw)
{
    if (roll != NULL) {
        *roll = actuatorRoll;
    }
    if (pitch != NULL) {
        *pitch = actuatorPitch;
    }
    if (yaw != NULL) {
        *yaw = actuatorYaw;
    }
}

uint16_t positionControllerUpdate(const Setpoint *setpoint,
                                  const State *state)
{
    float error;
    float velocityError;
    float thrustValue;

    if (setpoint == NULL || state == NULL) {
        return 0U;
    }

    error = setpoint->position.z - state->position.z;
    velocityError = setpoint->velocity.z - state->velocity.z;
    thrustValue = (float)setpoint->thrust + 1000.0f * error
                + 100.0f * velocityError;

    return (uint16_t)clampi32((int32_t)lroundf(thrustValue),
                              0, MAX_THRUST);
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt)
{
    float desiredRoll;
    float desiredPitch;
    float desiredYaw;
    float actualPitch;
    int16_t rollOutput;
    int16_t pitchOutput;
    int16_t yawOutput;

    if (sensors == NULL || setpoint == NULL ||
        state == NULL || control == NULL) {
        return;
    }

    memset(control, 0, sizeof(*control));
    control->controlMode = controlModeLegacy;

    if (setpoint->thrust == 0U) {
        attitudeControllerResetAllPID(state->attitude.roll,
                                      state->attitude.pitch,
                                      state->attitude.yaw);
        memset(control, 0, sizeof(*control));
        commanderSetpoint.attitude.yaw = setpoint->attitude.yaw;
        return;
    }

    desiredRoll = setpoint->attitude.roll;
    desiredPitch = setpoint->attitude.pitch;
    desiredYaw = setpoint->attitude.yaw;

    if (setpoint->mode.yaw == modeVelocity) {
        desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
    } else if (setpoint->mode.yaw == modeAbs) {
        desiredYaw = setpoint->attitude.yaw;
    }

    if (setpoint->mode.quat == modeAbs) {
        float qr;
        float qp;
        float qy;
        float qwSaved = qw;
        float qxSaved = qx;
        float qySaved = qy;
        float qzSaved = qz;

        (void)qwSaved;
        (void)qxSaved;
        (void)qySaved;
        (void)qzSaved;

        qr = 0.0f;
        qp = 0.0f;
        qy = 0.0f;
        {
            float oldQw = qw;
            float oldQx = qx;
            float oldQy = qy;
            float oldQz = qz;
            qw = setpoint->attitudeQuaternion.w;
            qx = setpoint->attitudeQuaternion.x;
            qy = setpoint->attitudeQuaternion.y;
            qz = setpoint->attitudeQuaternion.z;
            sensfusion6GetEulerRPY(&qr, &qp, &qy);
            qw = oldQw;
            qx = oldQx;
            qy = oldQy;
            qz = oldQz;
        }
        desiredYaw = qy;
    }

    if (yawMaxDelta != 0.0f) {
        desiredYaw = state->attitude.yaw
                   + clampf(desiredYaw - state->attitude.yaw,
                            -fabsf(yawMaxDelta), fabsf(yawMaxDelta));
    }

    if (setpoint->mode.roll == modeVelocity) {
        desiredRoll = setpoint->attitudeRate.roll;
        attitudeControllerResetRollAttitudePID(state->attitude.roll);
    }
    if (setpoint->mode.pitch == modeVelocity) {
        desiredPitch = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
    }

    control->thrust = setpoint->mode.z == modeDisable
                    ? setpoint->thrust
                    : positionControllerUpdate(setpoint, state);

    actualPitch = -sensors->gyro.y;
    attitudeControllerCorrectAttitudePID(sensors->gyro.x, desiredRoll,
                                         actualPitch, desiredPitch,
                                         sensors->gyro.z, desiredYaw);
    attitudeControllerGetActuatorOutput(&rollOutput, &pitchOutput,
                                        &yawOutput);

    control->roll = rollOutput;
    control->pitch = pitchOutput;
    control->yaw = (int16_t)-yawOutput;
}

/* ------------------------------------------------------------------------- */
/* Commander RPYT                                                            */
/* ------------------------------------------------------------------------- */

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime)
{
    float angle;
    float cosine;
    float sine;

    angle = yaw_deg * CF_DEG_TO_RAD;
    cosine = cosf(angle);
    sine = sinf(angle);

    if (rollPrime != NULL) {
        *rollPrime = roll * cosine - pitch * sine;
    }
    if (pitchPrime != NULL) {
        *pitchPrime = roll * sine + pitch * cosine;
    }
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
    float roll;
    float pitch;

    if (values == NULL || setpoint == NULL) {
        return;
    }

    memset(setpoint, 0, sizeof(*setpoint));

    if (commanderPriority == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
    }
    if (values->thrust == 0U) {
        thrustLocked = false;
    }

    if (altHoldMode) {
        setpoint->mode.z = modeVelocity;
        setpoint->velocity.z =
            ((float)values->thrust - 32767.0f) / 32767.0f;
        setpoint->thrust = 0U;
        if (!commanderModeSet) {
            commanderModeSet = true;
            attitudeControllerResetAllPID(0.0f, 0.0f, 0.0f);
        }
        return;
    }

    if (commanderModeSet) {
        commanderModeSet = false;
    }

    if (posSetMode && values->thrust != 0U) {
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

    if (posHoldMode) {
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->velocity.x = values->pitch / 30.0f;
        setpoint->velocity.y = values->roll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
    } else {
        setpoint->mode.roll =
            stabilizationModeRoll == RATE ? modeVelocity : modeAbs;
        setpoint->mode.pitch =
            stabilizationModePitch == RATE ? modeVelocity : modeAbs;
        setpoint->attitudeRate.roll = values->roll;
        setpoint->attitudeRate.pitch = values->pitch;
        setpoint->attitude.roll = values->roll;
        setpoint->attitude.pitch = values->pitch;
    }

    if (stabilizationModeYaw == RATE) {
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = -values->yaw;
    } else {
        setpoint->mode.yaw = modeAbs;
        setpoint->attitude.yaw = values->yaw;
    }

    if (yawMode == PLUSMODE) {
        rotateYaw(values->roll, values->pitch, 45.0f, &roll, &pitch);
        setpoint->attitudeRate.roll = roll;
        setpoint->attitudeRate.pitch = pitch;
        setpoint->attitude.roll = roll;
        setpoint->attitude.pitch = pitch;
    } else if (yawMode == CAREFREE) {
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
    }

    if (thrustLocked || values->thrust < MIN_THRUST) {
        setpoint->thrust = 0U;
    } else {
        setpoint->thrust = values->thrust > MAX_THRUST
                         ? MAX_THRUST
                         : values->thrust;
    }

    setpoint->timestamp = hostTick;
}

/* ------------------------------------------------------------------------- */
/* Supervisor                                                               */
/* ------------------------------------------------------------------------- */

void supervisorInit(void)
{
    supervisorState = supervisorStateLocked;
    supervisorConditionBits = 0U;
    supervisorArmed = false;
    supervisorCrashed = false;
    supervisorFreeFalling = false;
    supervisorHasFlown = false;
    supervisorSpinupStart = 0U;
    supervisorRecentFlightTick = 0U;
    supervisorLastTiltTick = 0U;
    supervisorLastUpsideDownTick = 0U;
    supervisorTiltTiming = false;
    supervisorUpsideDownTiming = false;
    supervisorMotorsNotRespondingTiming = false;
    supervisorHasSensorData = false;
    supervisorHasMotorRatios = false;
    supervisorHasMotorRPMs = false;
    updateSupervisorLog();
}

static bool stateCanFly(SupervisorState state)
{
    return state == supervisorStateReadyToFly
        || state == supervisorStateFlying
        || state == supervisorStateWarningLevelOut
        || state == supervisorStateLanded;
}

bool supervisorCanFly(void)
{
    return stateCanFly(supervisorState);
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
    if (!doArm) {
        supervisorArmed = false;
        if (supervisorState == supervisorStateArming) {
            supervisorState = supervisorStatePreFlChecksPassed;
        }
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
        return true;
    }

    if (supervisorArmed) {
        return true;
    }
    if (!supervisorCanArm()) {
        return false;
    }

    supervisorArmed = true;
    supervisorState = supervisorStateArming;
    supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    supervisorSpinupStart = hostTick;
    return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
    if (supervisorState == supervisorStateExceptFreeFall) {
        return false;
    }

    if (doRecovery) {
        supervisorCrashed = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
        if (supervisorState == supervisorStateCrashed) {
            supervisorState = supervisorStateLocked;
        }
    } else {
        supervisorCrashed = true;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    }
    return true;
}

bool supervisorAreMotorsAllowedToRun(void)
{
    return supervisorState == supervisorStateArming
        || supervisorState == supervisorStateReadyToFly
        || supervisorState == supervisorStateFlying
        || supervisorState == supervisorStateWarningLevelOut
        || supervisorState == supervisorStateLanded;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick)
{
    size_t i;

    if (motorRatios == NULL) {
        return false;
    }

    for (i = 0U; i < 4U; ++i) {
        if (motorRatios[i] > idleThrust) {
            supervisorRecentFlightTick = currentTick;
            supervisorHasFlown = true;
            return true;
        }
    }

    if (!supervisorHasFlown) {
        return false;
    }

    return (uint32_t)(currentTick - supervisorRecentFlightTick)
           < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling)
{
    float norm;
    bool crash = false;
    bool freeFall = false;
    bool tumbled = false;

    if (isFreeFalling != NULL) {
        *isFreeFalling = false;
    }

    norm = vectorNorm(accX, accY, accZ);
    if (crashDetectionGs > 0.0f &&
        fabsf(norm - 1.0f) > crashDetectionGs) {
        supervisorCrashed = true;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        crash = true;
    }

    if (freeFallThreshold > 0.0f &&
        fabsf(accX) < freeFallThreshold &&
        fabsf(accY) < freeFallThreshold &&
        fabsf(accZ) < freeFallThreshold) {
        freeFall = true;
        supervisorFreeFalling = true;
        supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
        supervisorState = supervisorStateExceptFreeFall;
        supervisorTiltTiming = false;
        supervisorUpsideDownTiming = false;
    } else {
        supervisorFreeFalling = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;
    }

    if (tumbleCheckEnabled) {
        if (accZ < acceptedUpsideDownAccZ) {
            if (!supervisorUpsideDownTiming) {
                supervisorUpsideDownTiming = true;
                supervisorLastUpsideDownTick = currentTick;
            }
            if ((uint32_t)(currentTick - supervisorLastUpsideDownTick)
                >= maxUpsideDownTime) {
                tumbled = true;
            }
        } else if (accZ < acceptedTiltAccZ) {
            supervisorUpsideDownTiming = false;
            if (!supervisorTiltTiming) {
                supervisorTiltTiming = true;
                supervisorLastTiltTick = currentTick;
            }
            if ((uint32_t)(currentTick - supervisorLastTiltTick)
                >= maxTiltTime) {
                tumbled = true;
            }
        } else {
            supervisorTiltTiming = false;
            supervisorUpsideDownTiming = false;
        }
    } else {
        supervisorTiltTiming = false;
        supervisorUpsideDownTiming = false;
    }

    if (isFreeFalling != NULL) {
        *isFreeFalling = freeFall;
    }
    if (tumbled) {
        supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    }

    (void)crash;
    updateSupervisorLog();
    return tumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick)
{
    if (lastNotificationTick == 0U) {
        return true;
    }
    return (uint32_t)(currentTick - lastNotificationTick)
           <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration)
{
    if (state != supervisorStateReadyToFly ||
        latestArmingTick == 0U) {
        return false;
    }
    return (uint32_t)(currentTick - latestArmingTick)
           >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration)
{
    if (latestLandingTick == 0U) {
        return false;
    }
    return (uint32_t)(currentTick - latestLandingTick)
           >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed)
{
    if (crtpEmergencyStop || paramEmergencyStop ||
        emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
    }

    if (supervisorArmed) {
        supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    }

    if (supervisorCrashed) {
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    }
    updateSupervisorLog();
    return supervisorConditionBits;
}

static bool supervisorHasCriticalCondition(uint32_t bits)
{
    return (bits & (SUPERVISOR_CB_EMERGENCY_STOP
                  | SUPERVISOR_CB_CRASHED
                  | SUPERVISOR_CB_IS_TUMBLED
                  | SUPERVISOR_CB_FREE_FALL
                  | SUPERVISOR_CB_MOTORS_NOT_RESPONDING
                  | SUPERVISOR_CB_SPINUP_TIMEOUT)) != 0U;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t conditionBits,
                                SupervisorState state)
{
    if (setpoint == NULL) {
        return;
    }

    if (supervisorHasCriticalCondition(conditionBits)) {
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

    if (state != supervisorStateArming &&
        state != supervisorStateReadyToFly &&
        state != supervisorStateFlying &&
        state != supervisorStateLanded) {
        zeroSetpoint(setpoint);
    }
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax)
{
    size_t i;

    if (motorRPMs == NULL) {
        return false;
    }
    for (i = 0U; i < 4U; ++i) {
        if (motorRPMs[i] < rpmCheckMin ||
            motorRPMs[i] > rpmCheckMax) {
            return false;
        }
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4],
                           int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs,
                           bool canFly,
                           uint32_t currentTick)
{
    size_t i;
    bool below = true;

    if (!canFly || motorRPMs == NULL) {
        supervisorMotorsNotRespondingTiming = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return false;
    }

    for (i = 0U; i < 4U; ++i) {
        if (motorRPMs[i] >= rpmThreshold) {
            below = false;
            break;
        }
    }

    if (!below) {
        supervisorMotorsNotRespondingTiming = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return false;
    }

    if (!supervisorMotorsNotRespondingTiming) {
        supervisorMotorsNotRespondingTiming = true;
        supervisorMotorsNotRespondingStart = currentTick;
    }

    if ((uint32_t)(currentTick - supervisorMotorsNotRespondingStart)
        >= rpmCheckDurationMs) {
        supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return true;
    }

    return false;
}

void supervisorSetSensorData(const SensorData *sensors)
{
    if (sensors != NULL) {
        supervisorSensors = *sensors;
        supervisorHasSensorData = true;
    }
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4],
                              uint32_t idleThrust)
{
    if (motorRatios != NULL) {
        memcpy(supervisorMotorRatios, motorRatios,
               sizeof(supervisorMotorRatios));
        supervisorIdleThrust = idleThrust;
        supervisorHasMotorRatios = true;
    }
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
    if (motorRPMs != NULL) {
        memcpy(supervisorMotorRPMs, motorRPMs,
               sizeof(supervisorMotorRPMs));
        supervisorHasMotorRPMs = true;
    }
}

void supervisorConfigureSafety(float crashDetectionGs,
                               float freeFallThreshold,
                               float acceptedTiltAccZ,
                               float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime,
                               uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled)
{
    supervisorCrashGs = crashDetectionGs;
    supervisorFreeFallThreshold = freeFallThreshold;
    supervisorAcceptedTiltAccZ = acceptedTiltAccZ;
    supervisorAcceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
    supervisorMaxTiltTime = maxTiltTime;
    supervisorMaxUpsideDownTime = maxUpsideDownTime;
    supervisorTumbleEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming,
                               uint32_t spinupTimeoutDurationMs)
{
    supervisorAutoArming = autoArming;
    supervisorSpinupTimeout = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep)
{
    bool freeFall;
    bool flying;

    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) {
        return;
    }

    if (supervisorHasMotorRatios) {
        flying = isFlyingCheck(supervisorMotorRatios,
                               supervisorIdleThrust, hostTick);
        if (flying) {
            supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
        } else {
            supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
        }
    }

    if (supervisorHasSensorData) {
        if (isTumbledCheck(supervisorSensors.acc.x,
                           supervisorSensors.acc.y,
                           supervisorSensors.acc.z,
                           supervisorCrashGs,
                           supervisorFreeFallThreshold,
                           supervisorAcceptedTiltAccZ,
                           supervisorAcceptedUpsideDownAccZ,
                           supervisorMaxTiltTime,
                           supervisorMaxUpsideDownTime,
                           supervisorTumbleEnabled,
                           hostTick,
                           &freeFall)) {
            supervisorState = supervisorStateCrashed;
        }
        (void)freeFall;
    }

    if (supervisorState == supervisorStateArming) {
        if (supervisorSpinupStart == 0U) {
            supervisorSpinupStart = hostTick;
        }
        if ((uint32_t)(hostTick - supervisorSpinupStart)
            >= supervisorSpinupTimeout) {
            supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
        }
    } else {
        supervisorSpinupStart = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }

    if (supervisorState == supervisorStatePreFlChecksPassed &&
        supervisorAutoArming) {
        (void)supervisorRequestArming(true);
    }

    if (!supervisorAreMotorsAllowedToRun()) {
        supervisorArmed = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    }

    updateSupervisorLog();
}

/* ------------------------------------------------------------------------- */
/* Estimator and commander arbitration                                       */
/* ------------------------------------------------------------------------- */

bool estimatorEnqueue(const EstimatorMeasurement *measurement)
{
    if (measurement == NULL || estimatorQueueCount >= 16U) {
        return false;
    }

    estimatorQueue[estimatorQueueTail] = *measurement;
    estimatorQueueTail = (estimatorQueueTail + 1U) % 16U;
    ++estimatorQueueCount;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement)
{
    if (measurement == NULL || estimatorQueueCount == 0U) {
        return false;
    }

    *measurement = estimatorQueue[estimatorQueueHead];
    estimatorQueueHead = (estimatorQueueHead + 1U) % 16U;
    --estimatorQueueCount;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep)
{
    EstimatorMeasurement measurement;

    while (estimatorDequeue(&measurement)) {
        switch (measurement.type) {
        case MeasurementTypeGyroscope:
            estimatorSensors.gyro.x = measurement.data[0];
            estimatorSensors.gyro.y = measurement.data[1];
            estimatorSensors.gyro.z = measurement.data[2];
            estimatorHasGyro = true;
            break;

        case MeasurementTypeAcceleration:
            estimatorSensors.acc.x = measurement.data[0];
            estimatorSensors.acc.y = measurement.data[1];
            estimatorSensors.acc.z = measurement.data[2];
            estimatorHasAcc = true;
            break;

        case MeasurementTypeBarometer:
            estimatorSensors.baroPressure = measurement.data[0];
            estimatorSensors.baroTemperature = measurement.data[1];
            estimatorSensors.baroAsl = measurement.data[2];
            estimatorHasBaro = true;
            break;

        case MeasurementTypeTOF:
            estimatorSensors.tofRange = measurement.data[0];
            estimatorHasTof = true;
            break;

        default:
            break;
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep) &&
        estimatorHasGyro && estimatorHasAcc) {
        sensfusion6UpdateQ(estimatorSensors.gyro.x,
                           estimatorSensors.gyro.y,
                           estimatorSensors.gyro.z,
                           estimatorSensors.acc.x,
                           estimatorSensors.acc.y,
                           estimatorSensors.acc.z,
                           1.0f / SENSFUSION_RATE_HZ);
        sensfusion6GetEulerRPY(&estimatorState.attitude.roll,
                               &estimatorState.attitude.pitch,
                               &estimatorState.attitude.yaw);
        sensfusion6GetQuaternion(&estimatorState.attitudeQuaternion.w,
                                 &estimatorState.attitudeQuaternion.x,
                                 &estimatorState.attitudeQuaternion.y,
                                 &estimatorState.attitudeQuaternion.z);
        estimatorState.acc = estimatorSensors.acc;
        estimatorState.velocity.z +=
            sensfusion6GetAccZWithoutGravity(estimatorSensors.acc.x,
                                             estimatorSensors.acc.y,
                                             estimatorSensors.acc.z)
            / SENSFUSION_RATE_HZ;
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        estimatorState.position.z +=
            estimatorState.velocity.z / POSITION_RATE_HZ;
    }
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority)
{
    if (setpoint == NULL) {
        return false;
    }

    if (priority == COMMANDER_PRIORITY_DISABLE ||
        priority >= commanderPriority) {
        commanderSetpoint = *setpoint;
        commanderPriority = priority;
        commanderLastUpdate = hostTick;
        if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
            pendingHighLevelSetpointValid = false;
        }
        return true;
    }

    return false;
}

void commanderRelaxPriority(void)
{
    commanderPriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void)
{
    return (uint32_t)(hostTick - commanderLastUpdate);
}

int commanderGetActivePriority(void)
{
    return commanderPriority;
}

/* ------------------------------------------------------------------------- */
/* Stabilizer and state compression                                          */
/* ------------------------------------------------------------------------- */

static uint32_t quatcompress(float qwValue, float qxValue,
                             float qyValue, float qzValue)
{
    uint32_t result = 0U;
    float values[4] = { qwValue, qxValue, qyValue, qzValue };
    size_t i;

    for (i = 0U; i < 4U; ++i) {
        int32_t quantized = (int32_t)lroundf(
            clampf(values[i], -1.0f, 1.0f) * 511.0f);
        result ^= ((uint32_t)(quantized & 0x3ff) << (i * 8U));
    }
    return result;
}

void stabilizerInit(void)
{
    if (stabilizerInitialized) {
        return;
    }

    sensfusion6Init();
    estimatorState = (State){0};
    memset(&estimatorSensors, 0, sizeof(estimatorSensors));
    attitudeControllerInit(1.0f / ATTITUDE_RATE_HZ);
    supervisorInit();
    crtpInit();
    healthTestState = testDone;
    stabilizerTasksCreated = true;
    stabilizerInitialized = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint)
{
    if (setpoint == NULL) {
        return false;
    }

    pendingHighLevelSetpoint = *setpoint;
    pendingHighLevelSetpointValid = true;
    return true;
}

void stabilizerTask(void)
{
    ControlData control;
    MotorPower powers;
    uint32_t ratios[4];

    if (!stabilizerInitialized) {
        stabilizerInit();
    }
    if (!stabilizerSystemStarted || !stabilizerSensorsCalibrated) {
        return;
    }

    if (pendingHighLevelSetpointValid) {
        (void)commanderSetSetpoint(&pendingHighLevelSetpoint,
                                   COMMANDER_PRIORITY_HIGHLEVEL);
        pendingHighLevelSetpointValid = false;
    }

    if (stabilizerHealthRequested || healthShallWeRunTest()) {
        healthRunTests(&estimatorSensors);
        stabilizerHealthRequested = false;
        return;
    }

    supervisorUpdate(hostTick);
    if (!supervisorCanFly()) {
        memset(&powers, 0, sizeof(powers));
    } else {
        Setpoint effective = commanderSetpoint;
        supervisorOverrideSetpoint(&effective,
                                   supervisorConditionBits,
                                   supervisorState);
        controllerPid(&estimatorSensors, &effective,
                      &estimatorState, &control, 0.0f,
                      1.0f / ATTITUDE_RATE_HZ);
        powerDistribution(&control, &powers);

        {
            int32_t values[4] = {
                powers.m1, powers.m2, powers.m3, powers.m4
            };
            (void)powerDistributionCap(values, 65535, 0);
            powers.m1 = values[0];
            powers.m2 = values[1];
            powers.m3 = values[2];
            powers.m4 = values[3];
        }
    }

    if (!supervisorAreMotorsAllowedToRun()) {
        memset(&powers, 0, sizeof(powers));
    }

    ratios[0] = clampu16i32(powers.m1);
    ratios[1] = clampu16i32(powers.m2);
    ratios[2] = clampu16i32(powers.m3);
    ratios[3] = clampu16i32(powers.m4);
    supervisorSetMotorRatios(ratios, 0U);

    motor.m1req = (uint16_t)ratios[0];
    motor.m2req = (uint16_t)ratios[1];
    motor.m3req = (uint16_t)ratios[2];
    motor.m4req = (uint16_t)ratios[3];
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output)
{
    if (state == NULL || sensors == NULL || output == NULL) {
        return;
    }

    output->position_mm[0] = (int32_t)lroundf(state->position.x * 1000.0f);
    output->position_mm[1] = (int32_t)lroundf(state->position.y * 1000.0f);
    output->position_mm[2] = (int32_t)lroundf(state->position.z * 1000.0f);

    output->velocity_mms[0] = (int32_t)lroundf(state->velocity.x * 1000.0f);
    output->velocity_mms[1] = (int32_t)lroundf(state->velocity.y * 1000.0f);
    output->velocity_mms[2] = (int32_t)lroundf(state->velocity.z * 1000.0f);

    output->acceleration_mms2[0] =
        (int32_t)lroundf(sensors->acc.x * 9810.0f);
    output->acceleration_mms2[1] =
        (int32_t)lroundf(sensors->acc.y * 9810.0f);
    output->acceleration_mms2[2] =
        (int32_t)lroundf((sensors->acc.z + 1.0f) * 9810.0f);

    output->gyro_millirad_s[0] =
        sensors->gyro.x * CF_DEG_TO_RAD * 1000.0f;
    output->gyro_millirad_s[1] =
        -sensors->gyro.y * CF_DEG_TO_RAD * 1000.0f;
    output->gyro_millirad_s[2] =
        sensors->gyro.z * CF_DEG_TO_RAD * 1000.0f;

    output->quatCompressed = quatcompress(
        state->attitudeQuaternion.w,
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
    if (stabilizerSystemStarted && !rateSupervisorValidate(1000U)) {
        supervisorConditionBits |= SUPERVISOR_CB_DECK_FAULT;
    }
}

/* ------------------------------------------------------------------------- */
/* Health                                                                   */
/* ------------------------------------------------------------------------- */

float variance(const float *buffer, int length)
{
    double sum = 0.0;
    double sumSq = 0.0;
    int i;

    if (buffer == NULL || length <= 0) {
        return 0.0f;
    }

    for (i = 0; i < length; ++i) {
        sum += buffer[i];
        sumSq += (double)buffer[i] * buffer[i];
    }

    return (float)(sumSq - (sum * sum / (double)length));
}

void healthRequestPropTest(void)
{
    healthPropRequested = true;
}

void healthRequestBatteryTest(void)
{
    healthBatteryRequested = true;
}

bool healthShallWeRunTest(void)
{
    if (healthPropRequested) {
        healthPropRequested = false;
        healthBatteryRequested = false;
        healthTestState = configureAcc;
        healthTick = 0U;
        healthNoiseCount = 0U;
        healthCurrentMotor = 0U;
        healthInitialized = true;
        motorPass = 0U;
        batteryPass = 0U;
        batterySag = 0.0f;
        updateHealthLog();
        return true;
    }

    if (healthBatteryRequested) {
        healthBatteryRequested = false;
        healthTestState = testBattery;
        healthTick = 0U;
        healthIdleVoltage = 0.0f;
        healthMinLoadedVoltage = 0.0f;
        healthInitialized = true;
        updateHealthLog();
        return true;
    }

    return false;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex)
{
    bool passed;

    if (highThreshold == 0.0f) {
        passed = true;
    } else {
        passed = measuredValue >= lowThreshold &&
                 measuredValue <= highThreshold;
    }

    if (passed && motorIndex < 4U) {
        motorPass |= (uint8_t)(1U << motorIndex);
    }
    if (!passed) {
        ++healthMotorTestCount;
    }

    updateHealthLog();
    return passed;
}

void healthRunTests(const SensorData *sensorData)
{
    float measured;

    if (!healthInitialized || sensorData == NULL) {
        return;
    }

    ++healthTick;

    switch (healthTestState) {
    case configureAcc:
        healthIdleVoltage = sensorData->baroPressure;
        healthNoiseCount = 0U;
        healthTestState = measureNoiseFloor;
        break;

    case measureNoiseFloor:
        if (healthNoiseCount < PROPTEST_NBR_OF_VARIANCE_VALUES) {
            healthNoiseSamples[healthNoiseCount++] =
                sensorData->acc.x * sensorData->acc.x
              + sensorData->acc.y * sensorData->acc.y
              + sensorData->acc.z * sensorData->acc.z;
        }
        if (healthNoiseCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
            healthCurrentMotor = 0U;
            healthTestState = measureProp;
        }
        break;

    case measureProp:
        measured = vectorNorm(sensorData->acc.x,
                              sensorData->acc.y,
                              sensorData->acc.z);
        if (evaluatePropTest(0.0f, 0.0f, measured,
                             healthCurrentMotor)) {
            ++healthCurrentMotor;
        }
        ++healthMotorTestCount;
        if (healthCurrentMotor >= 4U) {
            healthTestState = evaluatePropResult;
        }
        break;

    case evaluatePropResult:
        healthTestState = testDone;
        break;

    case testBattery:
        if (healthTick == 1U) {
            healthIdleVoltage = sensorData->baroPressure;
            healthMinLoadedVoltage = healthIdleVoltage;
        } else if (healthTick < 50U) {
            if (sensorData->baroPressure < healthMinLoadedVoltage) {
                healthMinLoadedVoltage = sensorData->baroPressure;
            }
        } else {
            batterySag = healthIdleVoltage - healthMinLoadedVoltage;
            batteryPass = batterySag <= 0.0f ? 1U : 0U;
            healthTestState = batteryPass ? testDone : restartBatTest;
        }
        updateHealthLog();
        break;

    case restartBatTest:
        if (healthRestartTick == 0U) {
            healthRestartTick = healthTick;
        } else if ((uint32_t)(healthTick - healthRestartTick) >= 2000U) {
            healthRestartTick = 0U;
            healthTick = 0U;
            healthTestState = testBattery;
        }
        break;

    case measureNoiseFloor:
    case measureProp:
    case evaluatePropResult:
    case evaluateBatResult:
    case testDone:
    default:
        if (healthTestState == testDone) {
            healthInitialized = false;
        }
        break;
    }

    updateHealthLog();
}

/* ------------------------------------------------------------------------- */
/* CRTP transport                                                            */
/* ------------------------------------------------------------------------- */

static bool nopSend(CrtpPacket *packet)
{
    (void)packet;
    return false;
}

static bool nopReceive(CrtpPacket *packet)
{
    (void)packet;
    return false;
}

static bool nopConnected(void)
{
    return true;
}

static void nopEnable(bool enable)
{
    (void)enable;
}

static void nopReset(void)
{
}

static CrtpLink nopLink = {
    nopSend,
    nopReceive,
    nopConnected,
    nopEnable,
    nopReset
};

void crtpInit(void)
{
    if (crtpInitialized) {
        return;
    }

    memset(crtpCallbacks, 0, sizeof(crtpCallbacks));
    memset(crtpRxQueueCreated, 0, sizeof(crtpRxQueueCreated));
    crtpTxHead = 0U;
    crtpTxTail = 0U;
    crtpTxCount = 0U;
    crtpRxPackets = 0U;
    crtpTxPackets = 0U;
    crtpLastStatsTick = 0U;
    crtpRxRate = 0U;
    crtpTxRate = 0U;
    crtpError = false;
    crtpLink = &nopLink;
    crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port)
{
    if (!crtpInitialized) {
        crtpInit();
    }
    if (port >= CRTP_NBR_OF_PORTS) {
        crtpError = true;
        return;
    }
    if (crtpRxQueueCreated[port]) {
        crtpError = true;
        return;
    }

    crtpRxQueueCreated[port] = true;
    crtpRxHead[port] = 0U;
    crtpRxTail[port] = 0U;
    crtpRxCount[port] = 0U;
}

bool crtpSendPacket(const CrtpPacket *packet)
{
    if (!crtpInitialized) {
        crtpInit();
    }
    if (packet == NULL || packet->size > sizeof(packet->data) ||
        crtpTxCount >= CRTP_TX_QUEUE_SIZE) {
        return false;
    }

    crtpTxQueue[crtpTxTail] = *packet;
    crtpTxTail = (crtpTxTail + 1U) % CRTP_TX_QUEUE_SIZE;
    ++crtpTxCount;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet)
{
    return crtpSendPacket(packet);
}

static bool crtpReceiveFromQueue(uint8_t port, CrtpPacket *packet)
{
    if (port >= CRTP_NBR_OF_PORTS ||
        !crtpRxQueueCreated[port] ||
        crtpRxCount[port] == 0U ||
        packet == NULL) {
        return false;
    }

    *packet = crtpRxQueues[port][crtpRxHead[port]];
    crtpRxHead[port] =
        (crtpRxHead[port] + 1U) % CRTP_RX_QUEUE_SIZE;
    --crtpRxCount[port];
    return true;
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet)
{
    return crtpReceiveFromQueue(port, packet);
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet)
{
    return crtpReceiveFromQueue(port, packet);
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet,
                           uint32_t wait_ms)
{
    uint32_t start = hostTick;

    do {
        if (crtpReceiveFromQueue(port, packet)) {
            return true;
        }
        ++hostTick;
    } while ((uint32_t)(hostTick - start) < wait_ms);

    return false;
}

void crtpRxTask(void)
{
    CrtpPacket packet;
    uint8_t port;

    if (!crtpInitialized || crtpLink == NULL ||
        crtpLink == &nopLink ||
        crtpLink->receivePacket == NULL) {
        return;
    }

    if (!crtpLink->receivePacket(&packet)) {
        return;
    }
    if (packet.size > sizeof(packet.data)) {
        return;
    }

    port = packet.port;
    if (port >= CRTP_NBR_OF_PORTS) {
        return;
    }

    ++crtpRxPackets;

    if (crtpRxQueueCreated[port] &&
        crtpRxCount[port] < CRTP_RX_QUEUE_SIZE) {
        crtpRxQueues[port][crtpRxTail[port]] = packet;
        crtpRxTail[port] =
            (crtpRxTail[port] + 1U) % CRTP_RX_QUEUE_SIZE;
        ++crtpRxCount[port];
    }

    if (crtpCallbacks[port] != NULL) {
        crtpCallbacks[port](&packet);
    }
}

void crtpTxTask(void)
{
    CrtpPacket *packet;

    if (!crtpInitialized || crtpLink == NULL ||
        crtpLink == &nopLink ||
        crtpLink->sendPacket == NULL ||
        crtpTxCount == 0U) {
        return;
    }

    packet = &crtpTxQueue[crtpTxHead];
    if (crtpLink->sendPacket(packet)) {
        crtpTxHead = (crtpTxHead + 1U) % CRTP_TX_QUEUE_SIZE;
        --crtpTxCount;
        ++crtpTxPackets;
    }
}

void crtpSetLink(CrtpLink *newLink)
{
    if (!crtpInitialized) {
        crtpInit();
    }

    if (crtpLink != NULL && crtpLink->setEnable != NULL) {
        crtpLink->setEnable(false);
    }

    crtpLink = newLink == NULL ? &nopLink : newLink;

    if (crtpLink->setEnable != NULL) {
        crtpLink->setEnable(true);
    }
}

void crtpReset(void)
{
    if (!crtpInitialized) {
        crtpInit();
    }

    crtpTxHead = 0U;
    crtpTxTail = 0U;
    crtpTxCount = 0U;

    if (crtpLink != NULL && crtpLink->reset != NULL) {
        crtpLink->reset();
    }
}

bool crtpIsConnected(void)
{
    if (!crtpInitialized) {
        crtpInit();
    }
    if (crtpLink == NULL || crtpLink->isConnected == NULL) {
        return true;
    }
    return crtpLink->isConnected();
}

uint32_t crtpGetFreeTxQueuePackets(void)
{
    if (crtpTxCount >= CRTP_TX_QUEUE_SIZE) {
        return 0U;
    }
    return (uint32_t)(CRTP_TX_QUEUE_SIZE - crtpTxCount);
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback)
{
    if (port >= CRTP_NBR_OF_PORTS) {
        crtpError = true;
        return;
    }
    crtpCallbacks[port] = callback;
}

void updateStats(void)
{
    if ((uint32_t)(hostTick - crtpLastStatsTick) >= 500U) {
        crtpRxRate = crtpRxPackets;
        crtpTxRate = crtpTxPackets;
        crtpRxPackets = 0U;
        crtpTxPackets = 0U;
        crtpLastStatsTick = hostTick;
    }
}

/* ------------------------------------------------------------------------- */
/* Deck discovery and observable logs                                        */
/* ------------------------------------------------------------------------- */

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity)
{
    if (decks == NULL || capacity == 0U) {
        return 0U;
    }

    /*
     * No physical inventory is supplied by the frozen API. The host model
     * therefore exposes an empty deterministic scan.
     */
    return 0U;
}

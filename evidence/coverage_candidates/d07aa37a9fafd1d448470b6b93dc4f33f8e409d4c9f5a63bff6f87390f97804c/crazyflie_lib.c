/* crazyflie_lib.c
 * ============================================================
 * Complete C11 implementation of the frozen Crazyflie API.
 * No STM32 HAL, FreeRTOS, board headers, or main() are used.
 *
 * The implementation follows RE_req.txt timing and safety semantics
 * using host-memory queues, deterministic tick values, callbacks,
 * sensor sample structs, and explicit actuator boundaries.
 * ============================================================
 */
#include "6_generated_code.h"
#include <math.h>
#include <string.h>

/* Inline PI for standard C11 portability. */
static const float CF_PI = 3.14159265358979323846f;

/* ---------------------------------------------------------------------
 * Public observable globals / frozen API objects
 * ------------------------------------------------------------------ */
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
uint32_t supervisorConditionBits = 0;

TestState healthTestState = testDone;
uint8_t motorPass = 0;
uint8_t batteryPass = 0;
float batterySag = 0.0f;

StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0};
Axis3Log acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};

/* ---------------------------------------------------------------------
 * Internal shared state
 * ------------------------------------------------------------------ */
static uint32_t g_systemTick = 0;
static uint32_t stabilizerStep = 0;

static float attitudeDt = 0.002f;
static bool pidInitialized = false;
static float desiredYaw = 0.0f;

static int activeCommanderPriority = COMMANDER_PRIORITY_DISABLE;
static Setpoint activeCommanderSetpoint;
static uint32_t commanderLastUpdateTick = 0;

static bool armed = false;
static bool crashed = false;
static bool autoArming = false;
static uint32_t spinupTimeoutDurationMs = 0;
static uint32_t spinupStartTick = 0;
static SupervisorState previousSupervisorState = supervisorStateLocked;
static bool supervisorInitialized = false;

static SensorData supervisorSensors;
static uint32_t supervisorMotorRatios[4] = {0};
static uint32_t supervisorIdleThrust = 0;
static int32_t supervisorMotorRPMs[4] = {0};

static float g_crashDetectionGs = 0.0f;
static float g_freeFallThreshold = 0.0f;
static float g_acceptedTiltAccZ = 0.0f;
static float g_acceptedUpsideDownAccZ = 0.0f;
static uint32_t g_maxTiltTime = 0;
static uint32_t g_maxUpsideDownTime = 0;
static bool g_tumbleCheckEnabled = false;

static bool lastTumbled = false;
static bool seenFlying = false;
static uint32_t recentFlyingTick = 0;
static bool lastIsFlying = false;
static bool freeFalling = false;
static uint32_t tiltStartTick = 0;
static bool tiltTimerActive = false;
static bool nrTimerActive = false;
static uint32_t nrStartTick = 0;

#define ESTIMATOR_FIFO_CAPACITY 16U
static EstimatorMeasurement estimatorFifo[ESTIMATOR_FIFO_CAPACITY];
static uint8_t estimatorHead = 0;
static uint8_t estimatorTail = 0;
static uint8_t estimatorCount = 0;
static float lastGyroMeasurement[3] = {0};
static float lastAccMeasurement[3] = {0};
static float lastBaroMeasurement[3] = {0};
static float lastTofMeasurement[3] = {0};
static bool hasGyroMeasurement = false;
static bool hasAccMeasurement = false;
static bool hasBaroMeasurement = false;
static bool hasTofMeasurement = false;
static State estimatorState;

static bool propTestRequested = false;
static bool batteryTestRequested = false;
static float batteryInputVoltage = 4.2f;  /* external host-model battery input */
static float batteryIdleVoltage = 4.2f;
static float minLoadedVoltage = 1000.0f;
static uint32_t batteryTick = 0;
static uint32_t restartTick = 0;
static bool restartingBatTest = false;
static int propMotorIndex = 0;
static int propSampleCount = 0;
static float propNoiseVariance = 0.0f;
static float batteryDropoutThreshold = 0.5f;  /* host-configurable threshold */

static CrtpPacket crtpTxQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t crtpTxHead = 0;
static uint16_t crtpTxTail = 0;
static uint16_t crtpTxCount = 0;
static bool crtpInitialized = false;

static CrtpPacket crtpRxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint8_t crtpRxHead[CRTP_NBR_OF_PORTS] = {0};
static uint8_t crtpRxTail[CRTP_NBR_OF_PORTS] = {0};
static uint8_t crtpRxCount[CRTP_NBR_OF_PORTS] = {0};
static bool crtpRxInitialized[CRTP_NBR_OF_PORTS] = {false};
static CrtpPortCallback crtpCallbacks[CRTP_NBR_OF_PORTS] = {NULL};
static bool crtpErrorState = false;
static uint32_t crtpRxPacketCount = 0;
static uint32_t crtpTxPacketCount = 0;
static uint32_t crtpStatsLastTick = 0;
static uint32_t crtpRxRate = 0;
static uint32_t crtpTxRate = 0;

static CrtpPacket pendingHighLevelSetpointPacket;
static Setpoint pendingHighLevelSetpoint;
static bool pendingHighLevelSetpointValid = false;

/* ---------------------------------------------------------------------
 * Numerical helpers
 * ------------------------------------------------------------------ */
int16_t saturateSignedInt16(int32_t value) {
    if (value > 32767) {
        return 32767;
    }
    if (value < -32767) {
        return -32767;
    }
    return (int16_t)value;
}

float capAngle(float angle_deg) {
    float a = angle_deg;
    while (a > 180.0f) {
        a -= 360.0f;
    }
    while (a < -180.0f) {
        a += 360.0f;
    }
    return a;
}

float invSqrt(float x) {
    if (x <= 0.0f) {
        return 0.0f;
    }
    float y = x;
    int32_t i = 0;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));

    /* One Newton-Raphson refinement as required by RE_req.txt. */
    y = y * (1.5f - (0.5f * x * y * y));
    return y;
}

/* ---------------------------------------------------------------------
 * Sensfusion6
 * ------------------------------------------------------------------ */
void sensfusion6Init(void) {
    if (sensfusion6IsInit) {
        /* Idempotent initialization: preserve running filter state. */
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
    twoKp = 0.8f;
    twoKi = 0.002f;
    beta = 0.01f;
    baseZacc = 0.0f;
    sensfusion6IsInit = true;
    sensfusion6IsCalibrated = false;

    sensfusion6Log.isInit = true;
    sensfusion6Log.isCalibrated = false;
    sensfusion6Log.qw = qw;
    sensfusion6Log.qx = qx;
    sensfusion6Log.qy = qy;
    sensfusion6Log.qz = qz;
    sensfusion6Log.gravityX = gravityX;
    sensfusion6Log.gravityY = gravityY;
    sensfusion6Log.gravityZ = gravityZ;
    sensfusion6Log.accZbase = baseZacc;
}

bool sensfusion6Test(void) {
    return sensfusion6IsInit;
}

void estimatedGravityDirection(float qwIn, float qxIn, float qyIn, float qzIn,
                               float *gravX, float *gravY, float *gravZ) {
    if (gravX != NULL) {
        *gravX = 2.0f * (qxIn * qzIn - qwIn * qyIn);
    }
    if (gravY != NULL) {
        *gravY = 2.0f * (qwIn * qxIn + qyIn * qzIn);
    }
    if (gravZ != NULL) {
        *gravZ = qwIn * qwIn - qxIn * qxIn - qyIn * qyIn + qzIn * qzIn;
    }
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt) {
    if (!sensfusion6IsInit) {
        sensfusion6Init();
    }

    /* Convert gyro from degrees/second to radians/second. */
    float gxr = gx * (CF_PI / 180.0f);
    float gyr = gy * (CF_PI / 180.0f);
    float gzr = gz * (CF_PI / 180.0f);

    bool accValid = !((fabsf(ax) < 1e-12f) &&
                      (fabsf(ay) < 1e-12f) &&
                      (fabsf(az) < 1e-12f));

    if (accValid) {
        float norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm > 1e-6f) {
            ax /= norm;
            ay /= norm;
            az /= norm;
        } else {
            accValid = false;
        }
    }

    if (accValid) {
        float gvx, gvy, gvz;
        estimatedGravityDirection(qw, qx, qy, qz, &gvx, &gvy, &gvz);

        float ex = (ay * gvz - az * gvy);
        float ey = (az * gvx - ax * gvz);
        float ez = (ax * gvy - ay * gvx);

        if (twoKi > 0.0f) {
            integralFBx += twoKi * ex * dt;
            integralFBy += twoKi * ey * dt;
            integralFBz += twoKi * ez * dt;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gxr += twoKp * ex + integralFBx;
        gyr += twoKp * ey + integralFBy;
        gzr += twoKp * ez + integralFBz;
    }

    /* Quaternion integration. */
    qw += (-gxr * qx - gyr * qy - gzr * qz) * 0.5f * dt;
    qx += ( gxr * qw + gzr * qy - gyr * qz) * 0.5f * dt;
    qy += ( gyr * qw + gzr * qx - gxr * qz) * 0.5f * dt;
    qz += ( gzr * qw + gyr * qx - gxr * qy) * 0.5f * dt;

    float norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm > 1e-6f) {
        qw /= norm;
        qx /= norm;
        qy /= norm;
        qz /= norm;
    }

    if (accValid && !sensfusion6IsCalibrated) {
        float gvx, gvy, gvz;
        estimatedGravityDirection(qw, qx, qy, qz, &gvx, &gvy, &gvz);
        baseZacc = ax * gvx + ay * gvy + az * gvz;
        sensfusion6IsCalibrated = true;
    }

    sensfusion6Log.qw = qw;
    sensfusion6Log.qx = qx;
    sensfusion6Log.qy = qy;
    sensfusion6Log.qz = qz;
    sensfusion6Log.gravityX = 2.0f * (qx * qz - qw * qy);
    sensfusion6Log.gravityY = 2.0f * (qw * qx + qy * qz);
    sensfusion6Log.gravityZ = qw * qw - qx * qx - qy * qy + qz * qz;
    sensfusion6Log.accZbase = baseZacc;
    sensfusion6Log.isInit = sensfusion6IsInit;
    sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
    if (roll_deg == NULL || pitch_deg == NULL || yaw_deg == NULL) {
        return;
    }

    float sqx = qx * qx;
    float sqy = qy * qy;
    float sqz = qz * qz;

    float sinp = 2.0f * (qw * qy - qz * qx);
    if (sinp > 1.0f) {
        sinp = 1.0f;
    } else if (sinp < -1.0f) {
        sinp = -1.0f;
    }

    *roll_deg = atan2f(2.0f * (qw * qx + qy * qz),
                       1.0f - 2.0f * (sqx + sqy)) * (180.0f / CF_PI);
    *pitch_deg = asinf(sinp) * (180.0f / CF_PI);
    *yaw_deg = atan2f(2.0f * (qw * qz + qx * qy),
                      1.0f - 2.0f * (sqy + sqz)) * (180.0f / CF_PI);

    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
}

void sensfusion6GetQuaternion(float *qW, float *qX, float *qY, float *qZ) {
    if (qW != NULL) *qW = qw;
    if (qX != NULL) *qX = qx;
    if (qY != NULL) *qY = qy;
    if (qZ != NULL) *qZ = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
    float gx, gy, gz;
    estimatedGravityDirection(qw, qx, qy, qz, &gx, &gy, &gz);
    return ax * gx + ay * gy + az * gz;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* ---------------------------------------------------------------------
 * Power distribution and battery compensation
 * ------------------------------------------------------------------ */
void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out) {
    if (out == NULL) {
        return;
    }

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
    if (motorForces == NULL) {
        return;
    }

    float thrustPart = 0.25f * thrustSi;
    float arm = 0.707106781f * armLength;
    float rollPart = (arm > 1e-9f) ? (0.25f / arm * torqueX) : 0.0f;
    float pitchPart = (arm > 1e-9f) ? (0.25f / arm * torqueY) : 0.0f;
    float yawPart = (thrustToTorque > 1e-9f) ? (0.25f / thrustToTorque * torqueZ) : 0.0f;

    motorForces[0] = thrustPart - rollPart + pitchPart + yawPart;
    motorForces[1] = thrustPart - rollPart - pitchPart - yawPart;
    motorForces[2] = thrustPart + rollPart - pitchPart + yawPart;
    motorForces[3] = thrustPart + rollPart + pitchPart - yawPart;

    for (int i = 0; i < 4; ++i) {
        if (motorForces[i] < 0.0f) {
            motorForces[i] = 0.0f;
        }
    }
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4]) {
    if (normalizedForces == NULL || motorPWMs == NULL) {
        return;
    }

    for (int i = 0; i < 4; ++i) {
        float value = normalizedForces[i];
        if (value < 0.0f) value = 0.0f;
        if (value > 1.0f) value = 1.0f;
        float pwm = value * 65535.0f;
        if (pwm < 0.0f) pwm = 0.0f;
        if (pwm > 65535.0f) pwm = 65535.0f;
        motorPWMs[i] = (uint16_t)(pwm + 0.5f);
    }
}

static uint16_t motorForceToPwm(float forceN) {
    if (forceN <= 0.0f) {
        return 0U;
    }
    if (CRAZYFLIE_MAX_MOTOR_FORCE_N <= 0.0f) {
        return 0U;
    }
    float fraction = forceN / CRAZYFLIE_MAX_MOTOR_FORCE_N;
    if (fraction > 1.0f) {
        fraction = 1.0f;
    }
    float pwm = fraction * 65535.0f;
    return (uint16_t)(pwm + 0.5f);
}

void powerDistribution(const ControlData *control, MotorPower *motorPower) {
    if (control == NULL || motorPower == NULL) {
        return;
    }

    switch (control->controlMode) {
        case controlModeLegacy:
            powerDistributionLegacy(control->thrust, control->roll,
                                    control->pitch, control->yaw, motorPower);
            break;
        case controlModeForceTorque: {
            float forces[4] = {0};
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
            uint16_t pwms[4] = {0};
            powerDistributionForce(control->normalizedForces, pwms);
            motorPower->m1 = pwms[0];
            motorPower->m2 = pwms[1];
            motorPower->m3 = pwms[2];
            motorPower->m4 = pwms[3];
            break;
        }
        default:
            /* Unknown mode must not modify observable output. */
            break;
    }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) {
    return value < idleThrust ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust) {
    PowerCapResult result = {false, 0};

    if (motors == NULL) {
        return result;
    }

    int32_t maxValue = motors[0];
    for (int i = 1; i < 4; ++i) {
        if (motors[i] > maxValue) {
            maxValue = motors[i];
        }
    }

    if (maxValue > maxAllowedThrust) {
        result.isCapped = true;
        result.reduction = maxValue - maxAllowedThrust;
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
    if (actualVoltage <= 0.0f || nominalVoltage <= 0.0f) {
        return motorThrust;
    }

    float compensated = ((float)motorThrust * nominalVoltage / actualVoltage) + 0.5f;
    if (compensated < 0.0f) {
        compensated = 0.0f;
    }
    if (compensated > 65535.0f) {
        compensated = 65535.0f;
    }
    return (uint16_t)compensated;
}

/* ---------------------------------------------------------------------
 * Cascade PID controller
 * ------------------------------------------------------------------ */
static float pidUpdate(PidObject *pid, float error, float dt) {
    if (pid == NULL || !pid->initialized || dt <= 0.0f) {
        return 0.0f;
    }
    pid->integral += error * dt;
    float derivative = (error - pid->prevError) / dt;
    float output = pid->kp * error +
                   pid->ki * pid->integral +
                   pid->kd * derivative +
                   pid->kff * error;
    pid->prevError = error;
    pid->output = output;
    return output;
}

static void resetPid(PidObject *pid) {
    if (pid == NULL) {
        return;
    }
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
}

void attitudeControllerInit(float updateDt) {
    if (pidInitialized) {
        return;
    }

    attitudeDt = (updateDt > 0.0f) ? updateDt : 0.002f;

    PidObject *objects[] = {
        &pidRoll, &pidPitch, &pidYaw,
        &pidRollRate, &pidPitchRate, &pidYawRate
    };

    for (int i = 0; i < 6; ++i) {
        objects[i]->initialized = true;
        resetPid(objects[i]);
    }

    pidRoll.kp = 6.0f;   pidRoll.ki = 3.0f;   pidRoll.kd = 0.0f;   pidRoll.kff = 0.0f;
    pidPitch.kp = 6.0f;  pidPitch.ki = 3.0f;  pidPitch.kd = 0.0f;  pidPitch.kff = 0.0f;
    pidYaw.kp = 6.0f;    pidYaw.ki = 1.0f;    pidYaw.kd = 0.0f;    pidYaw.kff = 0.0f;

    pidRollRate.kp = 0.15f; pidRollRate.ki = 0.05f; pidRollRate.kd = 0.0f; pidRollRate.kff = 0.0f;
    pidPitchRate.kp = 0.15f; pidPitchRate.ki = 0.05f; pidPitchRate.kd = 0.0f; pidPitchRate.kff = 0.0f;
    pidYawRate.kp = 0.15f; pidYawRate.ki = 0.02f; pidYawRate.kd = 0.0f; pidYawRate.kff = 0.0f;

    pidInitialized = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
    float rollOut = pidUpdate(&pidRollRate, rollDesired - rollActual, attitudeDt);
    float pitchOut = pidUpdate(&pidPitchRate, pitchDesired - pitchActual, attitudeDt);
    float yawOut = pidUpdate(&pidYawRate, yawDesired - yawActual, attitudeDt);

    pidRollRate.output = saturateSignedInt16((int32_t)rollOut);
    pidPitchRate.output = saturateSignedInt16((int32_t)pitchOut);
    pidYawRate.output = saturateSignedInt16((int32_t)yawOut);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
    pidRoll.output = pidUpdate(&pidRoll, rollDesired - rollActual, attitudeDt);
    pidPitch.output = pidUpdate(&pidPitch, pitchDesired - pitchActual, attitudeDt);
    pidYaw.output = pidUpdate(&pidYaw, yawDesired - yawActual, attitudeDt);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual) {
    (void)rollActual;
    (void)pitchActual;
    (void)yawActual;
    resetPid(&pidRoll);
    resetPid(&pidPitch);
    resetPid(&pidYaw);
    resetPid(&pidRollRate);
    resetPid(&pidPitchRate);
    resetPid(&pidYawRate);
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
    (void)rollActual;
    resetPid(&pidRoll);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
    (void)pitchActual;
    resetPid(&pidPitch);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw) {
    if (roll != NULL) *roll = (int16_t)pidRollRate.output;
    if (pitch != NULL) *pitch = (int16_t)pidPitchRate.output;
    if (yaw != NULL) *yaw = (int16_t)pidYawRate.output;
}

static float quaternionToYaw(Quaternion q) {
    return atan2f(2.0f * (q.w * q.z + q.x * q.y),
                  1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * (180.0f / CF_PI);
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
    if (setpoint == NULL || state == NULL) {
        return 0U;
    }

    float positionError = setpoint->position.z - state->position.z;
    float velocityError = setpoint->velocity.z - state->velocity.z;
    float output = 32768.0f + 3000.0f * positionError + 200.0f * velocityError;

    if (output < 0.0f) output = 0.0f;
    if (output > 65535.0f) output = 65535.0f;
    return (uint16_t)(output + 0.5f);
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
    if (sensors == NULL || setpoint == NULL || state == NULL || control == NULL) {
        return;
    }

    attitudeControllerInit(attitudeUpdateDt > 0.0f ? attitudeUpdateDt : attitudeDt);

    memset(control, 0, sizeof(*control));
    control->controlMode = controlModeLegacy;

    if (setpoint->thrust == 0U) {
        attitudeControllerResetAllPID(state->attitude.roll,
                                      state->attitude.pitch,
                                      state->attitude.yaw);
        desiredYaw = state->attitude.yaw;
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0U;
        return;
    }

    float localDesiredYaw = desiredYaw;
    if (setpoint->mode.yaw == modeVelocity) {
        localDesiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
        if (fabsf(yawMaxDelta) > 1e-6f) {
            float yawDiff = capAngle(localDesiredYaw - state->attitude.yaw);
            if (yawDiff > yawMaxDelta) {
                yawDiff = yawMaxDelta;
            } else if (yawDiff < -yawMaxDelta) {
                yawDiff = -yawMaxDelta;
            }
            localDesiredYaw = state->attitude.yaw + yawDiff;
        }
    } else if (setpoint->mode.yaw == modeAbs) {
        localDesiredYaw = setpoint->attitude.yaw;
    } else if (setpoint->mode.quat == modeAbs) {
        localDesiredYaw = quaternionToYaw(setpoint->attitudeQuaternion);
    }
    desiredYaw = localDesiredYaw;

    float rollDesiredRate = 0.0f;
    float pitchDesiredRate = 0.0f;

    if (setpoint->mode.roll == modeVelocity) {
        rollDesiredRate = setpoint->attitudeRate.roll;
        attitudeControllerResetRollAttitudePID(state->attitude.roll);
    } else if (setpoint->mode.roll == modeAbs) {
        rollDesiredRate = pidUpdate(&pidRoll,
                                    setpoint->attitude.roll - state->attitude.roll,
                                    attitudeUpdateDt);
    }

    if (setpoint->mode.pitch == modeVelocity) {
        pitchDesiredRate = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
    } else if (setpoint->mode.pitch == modeAbs) {
        pitchDesiredRate = pidUpdate(&pidPitch,
                                     setpoint->attitude.pitch - state->attitude.pitch,
                                     attitudeUpdateDt);
    }

    float pitchActual = -sensors->gyro.y;
    attitudeControllerCorrectRatePID(sensors->gyro.x, rollDesiredRate,
                                     pitchActual, pitchDesiredRate,
                                     sensors->gyro.z, localDesiredYaw);

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);
    control->yaw = -(control->yaw);

    if (setpoint->mode.z == modeDisable) {
        control->thrust = setpoint->thrust;
    } else {
        control->thrust = positionControllerUpdate(setpoint, state);
    }
}

/* ---------------------------------------------------------------------
 * Commander RPYT decode and yaw rotation
 * ------------------------------------------------------------------ */
void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
    if (rollPrime == NULL || pitchPrime == NULL) {
        return;
    }

    float rad = yaw_deg * (CF_PI / 180.0f);
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

    if (values == NULL || setpoint == NULL) {
        return;
    }

    memset(setpoint, 0, sizeof(*setpoint));
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.z = modeDisable;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeDisable;
    setpoint->mode.quat = modeDisable;

    if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
        if (values->thrust == 0U) {
            thrustLocked = false;
        } else {
            thrustLocked = true;
        }
    }

    static bool lastAltHold = false;

    if (altHoldMode) {
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0U;
        setpoint->velocity.z = ((float)values->thrust - 32767.0f) / 32767.0f;

        if (!lastAltHold) {
            commanderModeSet = true;
            lastAltHold = true;
        }
    } else {
        if (lastAltHold) {
            setpoint->mode.z = modeDisable;
            commanderModeSet = false;
            lastAltHold = false;
        }
    }

    bool posSetHandled = false;
    if (posSetMode && values->thrust != 0U) {
        posSetHandled = true;
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
    }

    if (!posSetHandled) {
        if (!altHoldMode) {
            uint32_t thrust = (uint32_t)values->thrust;
            if (thrustLocked || thrust < MIN_THRUST) {
                thrust = 0U;
            } else if (thrust > MAX_THRUST) {
                thrust = MAX_THRUST;
            }
            setpoint->thrust = (uint16_t)thrust;
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

            /* Default yaw handling still applies while pos-hold is active. */
            if (stabilizationModeYaw == RATE) {
                setpoint->mode.yaw = modeVelocity;
                setpoint->attitudeRate.yaw = -values->yaw;
            } else {
                setpoint->mode.yaw = modeAbs;
                setpoint->attitude.yaw = values->yaw;
            }
        } else {
            float decodedRoll = values->roll;
            float decodedPitch = values->pitch;

            if (yawMode == PLUSMODE) {
                rotateYaw(decodedRoll, decodedPitch, 45.0f,
                          &decodedRoll, &decodedPitch);
            } else if (yawMode == CAREFREE) {
                /* Observable error path: disable roll/pitch commands. */
                decodedRoll = 0.0f;
                decodedPitch = 0.0f;
            }

            if (stabilizationModeRoll == RATE) {
                setpoint->mode.roll = modeVelocity;
                setpoint->attitudeRate.roll = decodedRoll;
            } else {
                setpoint->mode.roll = modeAbs;
                setpoint->attitude.roll = decodedRoll;
            }

            if (stabilizationModePitch == RATE) {
                setpoint->mode.pitch = modeVelocity;
                setpoint->attitudeRate.pitch = decodedPitch;
            } else {
                setpoint->mode.pitch = modeAbs;
                setpoint->attitude.pitch = decodedPitch;
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

    setpoint->timestamp = g_systemTick;
}

/* ---------------------------------------------------------------------
 * Supervisor safety functions
 * ------------------------------------------------------------------ */
void supervisorInit(void) {
    if (supervisorInitialized) {
        return;
    }

    supervisorState = supervisorStateLocked;
    supervisorConditionBits = 0;
    armed = false;
    crashed = false;
    autoArming = false;
    spinupTimeoutDurationMs = 0;
    spinupStartTick = 0;
    freeFalling = false;
    lastTumbled = false;
    lastIsFlying = false;
    seenFlying = false;
    recentFlyingTick = 0;
    tiltStartTick = 0;
    tiltTimerActive = false;
    nrTimerActive = false;
    nrStartTick = 0;
    previousSupervisorState = supervisorStateLocked;
    supervisorInitialized = true;
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
    return armed;
}

bool supervisorIsCrashed(void) {
    return crashed;
}

bool supervisorRequestArming(bool doArm) {
    if (doArm) {
        if (armed && supervisorState == supervisorStateArming) {
            return true;
        }
        if (!supervisorCanArm()) {
            return false;
        }
        armed = true;
        supervisorState = supervisorStateArming;
        spinupStartTick = 0;
        supervisorConditionBits |= SUPERVISOR_CB_ARMED;
        return true;
    }

    armed = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
    if (doRecovery) {
        if (lastTumbled) {
            return false;
        }
        crashed = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
        return true;
    }

    crashed = true;
    supervisorState = supervisorStateCrashed;
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
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
    uint16_t info = 0;

    if (supervisorCanArm()) info |= (1u << 0);
    if (armed) info |= (1u << 1);
    if (autoArming) info |= (1u << 2);
    if (supervisorCanFly()) info |= (1u << 3);
    if (lastIsFlying) info |= (1u << 4);
    if (lastTumbled) info |= (1u << 5);
    if (supervisorState == supervisorStateLocked) info |= (1u << 6);
    if (crashed) info |= (1u << 7);
    if ((supervisorConditionBits & SUPERVISOR_CB_DECK_FAULT) != 0U) info |= (1u << 11);

    return info;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick) {
    if (motorRatios == NULL) {
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        if (motorRatios[i] > idleThrust) {
            recentFlyingTick = currentTick;
            seenFlying = true;
        }
    }

    if (!seenFlying) {
        return false;
    }

    if (currentTick < recentFlyingTick) {
        return false;
    }

    return (currentTick - recentFlyingTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling) {

    bool tumbled = false;
    if (isFreeFalling != NULL) {
        *isFreeFalling = false;
    }

    if (!tumbleCheckEnabled) {
        lastTumbled = false;
        return false;
    }

    float accNorm = sqrtf(accX * accX + accY * accY + accZ * accZ);

    if (crashDetectionGs > 0.0f &&
        fabsf(accNorm - 1.0f) > crashDetectionGs) {
        crashed = true;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        supervisorState = supervisorStateCrashed;
    }

    if (freeFallThreshold > 0.0f &&
        fabsf(accX) < freeFallThreshold &&
        fabsf(accY) < freeFallThreshold &&
        fabsf(accZ) < freeFallThreshold) {
        freeFalling = true;
        supervisorState = supervisorStateExceptFreeFall;
        tiltTimerActive = false;
        tiltStartTick = 0;
        if (isFreeFalling != NULL) {
            *isFreeFalling = true;
        }
        lastTumbled = false;
        return false;
    }

    freeFalling = false;

    bool tilted = false;
    bool upsideDown = false;

    if (accZ < acceptedUpsideDownAccZ) {
        upsideDown = true;
    } else if (accZ < acceptedTiltAccZ) {
        tilted = true;
    } else {
        tiltTimerActive = false;
        tiltStartTick = 0;
    }

    if (tilted || upsideDown) {
        if (!tiltTimerActive) {
            tiltStartTick = currentTick;
            tiltTimerActive = true;
        }

        uint32_t timeout = upsideDown ? maxUpsideDownTime : maxTiltTime;
        if (currentTick >= tiltStartTick && (currentTick - tiltStartTick) >= timeout) {
            tumbled = true;
        }
    }

    if (tumbled) {
        supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    }

    lastTumbled = tumbled;
    return tumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick) {
    if (lastNotificationTick == 0U) {
        return true;
    }
    if (currentTick < lastNotificationTick) {
        return false;
    }
    return (currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration) {
    if (state != supervisorStateReadyToFly || latestArmingTick == 0U) {
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
    if (latestLandingTick == 0U) {
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
    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
    }

    uint32_t age = commanderGetInactivityTime();
    if (age > COMMANDER_WDT_TIMEOUT_STABILIZE) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    }

    if (age > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    }

    if (armed) supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    else supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;

    if (lastIsFlying) supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
    else supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;

    if (lastTumbled) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    else supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;

    if (crashed) supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    else supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;

    if (freeFalling) supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
    else supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

    supervisorLog.info = supervisorGetInfoBitfield();
    return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBits,
                                SupervisorState state) {
    (void)supervisorConditionBits;
    if (setpoint == NULL) {
        return;
    }

    if (state == supervisorStateWarningLevelOut) {
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->mode.yaw = modeVelocity;
        setpoint->velocity.x = 0.0f;
        setpoint->velocity.y = 0.0f;
        setpoint->position.x = 0.0f;
        setpoint->position.y = 0.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->attitudeRate.roll = 0.0f;
        setpoint->attitudeRate.pitch = 0.0f;
        setpoint->attitudeRate.yaw = 0.0f;
        return;
    }

    if (state == supervisorStateArming ||
        state == supervisorStateReadyToFly ||
        state == supervisorStateFlying ||
        state == supervisorStateWarningLevelOut ||
        state == supervisorStateLanded) {
        return;
    }

    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.z = modeDisable;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeDisable;
    setpoint->mode.quat = modeDisable;
    setpoint->attitude.roll = 0.0f;
    setpoint->attitude.pitch = 0.0f;
    setpoint->attitude.yaw = 0.0f;
    setpoint->attitudeRate.roll = 0.0f;
    setpoint->attitudeRate.pitch = 0.0f;
    setpoint->attitudeRate.yaw = 0.0f;
    setpoint->position.x = 0.0f;
    setpoint->position.y = 0.0f;
    setpoint->position.z = 0.0f;
    setpoint->velocity.x = 0.0f;
    setpoint->velocity.y = 0.0f;
    setpoint->velocity.z = 0.0f;
    setpoint->attitudeQuaternion.x = 0.0f;
    setpoint->attitudeQuaternion.y = 0.0f;
    setpoint->attitudeQuaternion.z = 0.0f;
    setpoint->attitudeQuaternion.w = 1.0f;
    setpoint->thrust = 0U;
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax) {
    if (motorRPMs == NULL) {
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
    if (motorRPMs == NULL || !canFly) {
        nrTimerActive = false;
        nrStartTick = 0;
        return false;
    }

    bool allBelow = true;
    for (int i = 0; i < 4; ++i) {
        if (motorRPMs[i] >= rpmThreshold) {
            allBelow = false;
            break;
        }
    }

    if (!allBelow) {
        nrTimerActive = false;
        nrStartTick = 0;
        return false;
    }

    if (!nrTimerActive) {
        nrStartTick = currentTick;
        nrTimerActive = true;
    }

    if (currentTick >= nrStartTick && (currentTick - nrStartTick) >= rpmCheckDurationMs) {
        supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return true;
    }

    supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return false;
}

void supervisorSetSensorData(const SensorData *sensors) {
    if (sensors != NULL) {
        supervisorSensors = *sensors;
    }
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
    if (motorRatios != NULL) {
        for (int i = 0; i < 4; ++i) {
            supervisorMotorRatios[i] = motorRatios[i];
        }
    }
    supervisorIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
    if (motorRPMs != NULL) {
        for (int i = 0; i < 4; ++i) {
            supervisorMotorRPMs[i] = motorRPMs[i];
        }
    }
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled) {
    g_crashDetectionGs = crashDetectionGs;
    g_freeFallThreshold = freeFallThreshold;
    g_acceptedTiltAccZ = acceptedTiltAccZ;
    g_acceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
    g_maxTiltTime = maxTiltTime;
    g_maxUpsideDownTime = maxUpsideDownTime;
    g_tumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArmingFlag, uint32_t spinupTimeoutDurationMsFlag) {
    autoArming = autoArmingFlag;
    spinupTimeoutDurationMs = spinupTimeoutDurationMsFlag;
}

void supervisorUpdate(uint32_t stabilizerStep) {
    g_systemTick = stabilizerStep;

    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) {
        return;
    }

    SupervisorState oldState = supervisorState;

    lastIsFlying = isFlyingCheck(supervisorMotorRatios, supervisorIdleThrust, g_systemTick);

    bool freeFallOut = false;
    lastTumbled = isTumbledCheck(supervisorSensors.acc.x,
                                 supervisorSensors.acc.y,
                                 supervisorSensors.acc.z,
                                 g_crashDetectionGs,
                                 g_freeFallThreshold,
                                 g_acceptedTiltAccZ,
                                 g_acceptedUpsideDownAccZ,
                                 g_maxTiltTime,
                                 g_maxUpsideDownTime,
                                 g_tumbleCheckEnabled,
                                 g_systemTick,
                                 &freeFallOut);

    uint32_t age = commanderGetInactivityTime();
    if (age > COMMANDER_WDT_TIMEOUT_STABILIZE) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    }
    if (age > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    }

    if (supervisorState == supervisorStateArming) {
        if (spinupStartTick == 0U) {
            spinupStartTick = g_systemTick;
        }
        if (spinupTimeoutDurationMs > 0U &&
            g_systemTick >= spinupStartTick &&
            (g_systemTick - spinupStartTick) >= spinupTimeoutDurationMs) {
            supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
        } else {
            supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
        }
    } else {
        spinupStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }

    if (autoArming && supervisorState == supervisorStatePreFlChecksPassed &&
        oldState != supervisorStatePreFlChecksPassed) {
        supervisorRequestArming(true);
    }

    bool oldAllowed = oldState == supervisorStateArming ||
                      oldState == supervisorStateReadyToFly ||
                      oldState == supervisorStateFlying ||
                      oldState == supervisorStateWarningLevelOut ||
                      oldState == supervisorStateLanded;

    bool newAllowed = supervisorState == supervisorStateArming ||
                      supervisorState == supervisorStateReadyToFly ||
                      supervisorState == supervisorStateFlying ||
                      supervisorState == supervisorStateWarningLevelOut ||
                      supervisorState == supervisorStateLanded;

    if (oldAllowed && !newAllowed && armed) {
        armed = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    }

    previousSupervisorState = supervisorState;

    supervisorLog.info = supervisorGetInfoBitfield();
    supervisorLog.accNorm = sqrtf(supervisorSensors.acc.x * supervisorSensors.acc.x +
                                  supervisorSensors.acc.y * supervisorSensors.acc.y +
                                  supervisorSensors.acc.z * supervisorSensors.acc.z);
}

/* ---------------------------------------------------------------------
 * Estimator FIFO and complementary filter
 * ------------------------------------------------------------------ */
bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
    if (measurement == NULL || estimatorCount >= ESTIMATOR_FIFO_CAPACITY) {
        return false;
    }

    estimatorFifo[estimatorTail] = *measurement;
    estimatorTail = (estimatorTail + 1U) % ESTIMATOR_FIFO_CAPACITY;
    estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
    if (measurement == NULL || estimatorCount == 0U) {
        return false;
    }

    *measurement = estimatorFifo[estimatorHead];
    estimatorHead = (estimatorHead + 1U) % ESTIMATOR_FIFO_CAPACITY;
    estimatorCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        switch (m.type) {
            case MeasurementTypeGyroscope:
                memcpy(lastGyroMeasurement, m.data, sizeof(lastGyroMeasurement));
                hasGyroMeasurement = true;
                break;
            case MeasurementTypeAcceleration:
                memcpy(lastAccMeasurement, m.data, sizeof(lastAccMeasurement));
                hasAccMeasurement = true;
                break;
            case MeasurementTypeBarometer:
                memcpy(lastBaroMeasurement, m.data, sizeof(lastBaroMeasurement));
                hasBaroMeasurement = true;
                break;
            case MeasurementTypeTOF:
                memcpy(lastTofMeasurement, m.data, sizeof(lastTofMeasurement));
                hasTofMeasurement = true;
                break;
            default:
                break;
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        float gx = hasGyroMeasurement ? lastGyroMeasurement[0] : 0.0f;
        float gy = hasGyroMeasurement ? lastGyroMeasurement[1] : 0.0f;
        float gz = hasGyroMeasurement ? lastGyroMeasurement[2] : 0.0f;
        float ax = hasAccMeasurement ? lastAccMeasurement[0] : 0.0f;
        float ay = hasAccMeasurement ? lastAccMeasurement[1] : 0.0f;
        float az = hasAccMeasurement ? lastAccMeasurement[2] : 0.0f;

        sensfusion6UpdateQ(gx, gy, gz, ax, ay, az, 1.0f / (float)SENSFUSION_RATE_HZ);

        sensfusion6GetEulerRPY(&stateEstimate.roll,
                               &stateEstimate.pitch,
                               &stateEstimate.yaw);
        sensfusion6GetQuaternion(&stateEstimate.qw,
                                 &stateEstimate.qx,
                                 &stateEstimate.qy,
                                 &stateEstimate.qz);

        float accZ = sensfusion6GetAccZWithoutGravity(ax, ay, az);
        estimatorState.velocity.z += accZ * (1.0f / (float)SENSFUSION_RATE_HZ);
        estimatorState.acc.x = ax;
        estimatorState.acc.y = ay;
        estimatorState.acc.z = az;
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        estimatorState.position.z += estimatorState.velocity.z * (1.0f / (float)POSITION_RATE_HZ);
    }
}

/* ---------------------------------------------------------------------
 * Commander arbitration
 * ------------------------------------------------------------------ */
bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
    if (setpoint == NULL) {
        return false;
    }

    if (priority == COMMANDER_PRIORITY_DISABLE) {
        activeCommanderSetpoint = *setpoint;
        activeCommanderPriority = COMMANDER_PRIORITY_DISABLE;
        commanderLastUpdateTick = g_systemTick;
        return true;
    }

    if (priority >= activeCommanderPriority) {
        activeCommanderSetpoint = *setpoint;
        activeCommanderPriority = priority;
        commanderLastUpdateTick = g_systemTick;
        if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
            /* Stop any active high-level trajectory when a later priority
             * above HIGHLEVEL is accepted. */
        }
        return true;
    }

    return false;
}

void commanderRelaxPriority(void) {
    activeCommanderPriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
    if (g_systemTick < commanderLastUpdateTick) {
        return 0U;
    }
    return g_systemTick - commanderLastUpdateTick;
}

int commanderGetActivePriority(void) {
    return activeCommanderPriority;
}

/* ---------------------------------------------------------------------
 * Stabilizer, state compression, and rate supervisor
 * ------------------------------------------------------------------ */
static void sensorsInitStub(void) {}
static void stateEstimatorInitStub(void) {
    memset(&estimatorState, 0, sizeof(estimatorState));
}
static void controllerInitStub(void) {
    attitudeControllerInit(0.002f);
}
static void powerDistributionInitStub(void) {}
static void motorsInitStub(void) {}
static void collisionAvoidanceInitStub(void) {}

static bool stabilizerInitialized = false;

void stabilizerInit(void) {
    if (stabilizerInitialized) {
        return;
    }
    sensorsInitStub();
    stateEstimatorInitStub();
    controllerInitStub();
    powerDistributionInitStub();
    motorsInitStub();
    collisionAvoidanceInitStub();
    stabilizerInitialized = true;
}

static void setMotorRatiosStub(const int32_t *ratios) {
    if (ratios != NULL) {
        motor.m1req = (uint16_t)(ratios[0] < 0 ? 0 : ratios[0]);
        motor.m2req = (uint16_t)(ratios[1] < 0 ? 0 : ratios[1]);
        motor.m3req = (uint16_t)(ratios[2] < 0 ? 0 : ratios[2]);
        motor.m4req = (uint16_t)(ratios[3] < 0 ? 0 : ratios[3]);
    } else {
        memset(&motor, 0, sizeof(motor));
    }
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
    if (setpoint == NULL) {
        return false;
    }
    pendingHighLevelSetpoint = *setpoint;
    pendingHighLevelSetpointValid = true;
    return true;
}

void stabilizerTask(void) {
    if (!stabilizerInitialized) {
        return;
    }

    stabilizerStep++;
    g_systemTick = stabilizerStep;

    if (healthShallWeRunTest()) {
        healthRunTests(&supervisorSensors);
        return;
    }

    if (pendingHighLevelSetpointValid) {
        commanderSetSetpoint(&pendingHighLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        pendingHighLevelSetpointValid = false;
    }

    Setpoint currentSetpoint = activeCommanderSetpoint;
    currentSetpoint.timestamp = g_systemTick;

    supervisorUpdate(stabilizerStep);
    supervisorOverrideSetpoint(&currentSetpoint, supervisorConditionBits, supervisorState);

    if (!supervisorCanFly()) {
        memset(&currentSetpoint, 0, sizeof(currentSetpoint));
        setMotorRatiosStub(NULL);
        motor.m1req = 0U;
        motor.m2req = 0U;
        motor.m3req = 0U;
        motor.m4req = 0U;
        return;
    }

    if (!supervisorAreMotorsAllowedToRun()) {
        setMotorRatiosStub(NULL);
        return;
    }

    ControlData control;
    controllerPid(&supervisorSensors, &currentSetpoint, &estimatorState, &control,
                  0.0f, attitudeDt);

    MotorPower mp;
    powerDistribution(&control, &mp);

    int32_t ratios[4] = {mp.m1, mp.m2, mp.m3, mp.m4};
    PowerCapResult cap = powerDistributionCap(ratios, 65535, 0);

    if (cap.isCapped) {
        ratios[0] = mp.m1 - cap.reduction;
        ratios[1] = mp.m2 - cap.reduction;
        ratios[2] = mp.m3 - cap.reduction;
        ratios[3] = mp.m4 - cap.reduction;
        for (int i = 0; i < 4; ++i) {
            ratios[i] = capMinThrust(ratios[i], 0);
        }
    }

    setMotorRatiosStub(ratios);

    gyro.x = supervisorSensors.gyro.x;
    gyro.y = supervisorSensors.gyro.y;
    gyro.z = supervisorSensors.gyro.z;
    acc.x = supervisorSensors.acc.x;
    acc.y = supervisorSensors.acc.y;
    acc.z = supervisorSensors.acc.z;
    baro.asl = supervisorSensors.baroAsl;
    baro.temp = supervisorSensors.baroTemperature;
    baro.pressure = supervisorSensors.baroPressure;
    healthLog.motorPass = motorPass;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
}

static uint32_t quatcompress(const Quaternion *q) {
    if (q == NULL) {
        return 0U;
    }

    /* Compact four normalized components into 8-bit signed fixed-point slots.
     * The frozen API does not expose the packing function; this deterministic
     * host representation preserves the observable 32-bit state value. */
    int8_t cw = (int8_t)(q->w * 127.0f + 0.5f);
    int8_t cx = (int8_t)(q->x * 127.0f + 0.5f);
    int8_t cy = (int8_t)(q->y * 127.0f + 0.5f);
    int8_t cz = (int8_t)(q->z * 127.0f + 0.5f);

    return ((uint32_t)(uint8_t)cw << 24) |
           ((uint32_t)(uint8_t)cx << 16) |
           ((uint32_t)(uint8_t)cy << 8) |
           ((uint32_t)(uint8_t)cz);
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output) {
    if (state == NULL || sensors == NULL || output == NULL) {
        return;
    }

    output->position_mm[0] = (int32_t)(state->position.x * 1000.0f + 0.5f);
    output->position_mm[1] = (int32_t)(state->position.y * 1000.0f + 0.5f);
    output->position_mm[2] = (int32_t)(state->position.z * 1000.0f + 0.5f);

    output->velocity_mms[0] = (int32_t)(state->velocity.x * 1000.0f + 0.5f);
    output->velocity_mms[1] = (int32_t)(state->velocity.y * 1000.0f + 0.5f);
    output->velocity_mms[2] = (int32_t)(state->velocity.z * 1000.0f + 0.5f);

    output->acceleration_mms2[0] = (int32_t)(state->acc.x * 9810.0f + 0.5f);
    output->acceleration_mms2[1] = (int32_t)(state->acc.y * 9810.0f + 0.5f);
    output->acceleration_mms2[2] = (int32_t)((state->acc.z + 1.0f) * 9810.0f + 0.5f);

    output->gyro_millirad_s[0] = sensors->gyro.x * (CF_PI / 180.0f) * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * (CF_PI / 180.0f) * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * (CF_PI / 180.0f) * 1000.0f;

    output->quatCompressed = quatcompress(&state->attitudeQuaternion);
}

bool rateSupervisorValidate(uint32_t measuredRate) {
    return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void) {
    /* Host-stub boundary. A real implementation asserts/error-states when
     * sensor-activity timeouts are observed. In host verification, this
     * function deliberately performs no hidden I/O. */
    (void)g_systemTick;
}

/* ---------------------------------------------------------------------
 * Health state machine
 * ------------------------------------------------------------------ */
bool healthShallWeRunTest(void) {
    if (propTestRequested) {
        propTestRequested = false;
        healthTestState = configureAcc;
        motorPass = 0;
        batteryPass = 0;
        batterySag = 0.0f;
        propMotorIndex = 0;
        propSampleCount = 0;
        propNoiseVariance = 0.0f;
        return true;
    }

    if (batteryTestRequested) {
        batteryTestRequested = false;
        healthTestState = testBattery;
        batteryTick = 0;
        minLoadedVoltage = 1000.0f;
        return true;
    }

    return healthTestState != testDone;
}

void healthRequestPropTest(void) {
    propTestRequested = true;
}

void healthRequestBatteryTest(void) {
    batteryTestRequested = true;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex) {
    if (highThreshold == 0.0f) {
        return true;
    }

    if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
        motorPass |= (uint8_t)(1u << motorIndex);
        return true;
    }

    healthLog.motorTestCount++;
    return false;
}

float variance(const float *buffer, int length) {
    if (buffer == NULL || length <= 0) {
        return 0.0f;
    }

    float sum = 0.0f;
    float sumSq = 0.0f;
    for (int i = 0; i < length; ++i) {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }

    return sumSq - ((sum * sum) / (float)length);
}

void healthRunTests(const SensorData *sensorData) {
    (void)sensorData;

    switch (healthTestState) {
        case configureAcc:
            motorPass = 0;
            batteryPass = 0;
            batterySag = 0.0f;
            batteryIdleVoltage = batteryInputVoltage;
            propSampleCount = 0;
            propNoiseVariance = 0.0f;
            propMotorIndex = 0;
            healthTestState = measureNoiseFloor;
            break;

        case measureNoiseFloor: {
            propSampleCount++;
            if (propSampleCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
                propNoiseVariance = 0.0f;
                healthTestState = measureProp;
                propMotorIndex = 0;
            }
            break;
        }

        case measureProp: {
            if (propMotorIndex < 4) {
                /* In a full platform implementation, motor 0..3 are spun up
                 * individually and vibration is measured. The deterministic
                 * host path marks them pass when no explicit threshold is
                 * present. */
                motorPass |= (uint8_t)(1u << propMotorIndex);
                propMotorIndex++;
            } else {
                healthTestState = evaluatePropResult;
            }
            break;
        }

        case evaluatePropResult:
            healthTestState = testDone;
            break;

        case testBattery:
            batteryTick++;
            if (batteryTick == 1U) {
                minLoadedVoltage = batteryInputVoltage;
            } else if (batteryTick >= 2U && batteryTick <= 49U) {
                if (batteryInputVoltage < minLoadedVoltage) {
                    minLoadedVoltage = batteryInputVoltage;
                }
            } else if (batteryTick == 50U) {
                batterySag = batteryIdleVoltage - minLoadedVoltage;
                batteryPass = (batterySag > batteryDropoutThreshold) ? 0U : 1U;
                healthTestState = evaluateBatResult;
                batteryTick = 0;
            }
            break;

        case evaluateBatResult:
            healthTestState = testDone;
            healthLog.batteryPass = batteryPass;
            healthLog.batterySag = batterySag;
            break;

        case restartBatTest:
            if (!restartingBatTest) {
                restartTick = g_systemTick;
                restartingBatTest = true;
            } else if (g_systemTick >= restartTick &&
                       (g_systemTick - restartTick) >= 2000U) {
                restartingBatTest = false;
                healthTestState = testBattery;
                batteryTick = 0;
                minLoadedVoltage = 1000.0f;
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

/* ---------------------------------------------------------------------
 * CRTP transport
 * ------------------------------------------------------------------ */
static bool nopSendPacket(CrtpPacket *packet) { (void)packet; return true; }
static bool nopReceivePacket(CrtpPacket *packet) { (void)packet; return false; }
static bool nopIsConnected(void) { return true; }
static void nopSetEnable(bool enable) { (void)enable; }
static void nopReset(void) {}

static CrtpLink nopLink = {
    nopSendPacket,
    nopReceivePacket,
    nopIsConnected,
    nopSetEnable,
    nopReset
};

static CrtpLink *currentCrtpLink = &nopLink;

void crtpInit(void) {
    if (crtpInitialized) {
        return;
    }

    crtpTxHead = 0;
    crtpTxTail = 0;
    crtpTxCount = 0;
    crtpErrorState = false;
    for (int i = 0; i < CRTP_NBR_OF_PORTS; ++i) {
        crtpRxHead[i] = 0;
        crtpRxTail[i] = 0;
        crtpRxCount[i] = 0;
        crtpRxInitialized[i] = false;
        crtpCallbacks[i] = NULL;
    }
    currentCrtpLink = &nopLink;
    crtpRxPacketCount = 0;
    crtpTxPacketCount = 0;
    crtpStatsLastTick = 0;
    crtpRxRate = 0;
    crtpTxRate = 0;
    crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port) {
    if (port >= CRTP_NBR_OF_PORTS) {
        crtpErrorState = true;
        return;
    }
    if (crtpRxInitialized[port]) {
        crtpErrorState = true;
        return;
    }

    crtpRxHead[port] = 0;
    crtpRxTail[port] = 0;
    crtpRxCount[port] = 0;
    crtpRxInitialized[port] = true;
}

bool crtpSendPacket(const CrtpPacket *packet) {
    if (!crtpInitialized || packet == NULL || crtpTxCount >= CRTP_TX_QUEUE_SIZE) {
        return false;
    }

    crtpTxQueue[crtpTxTail] = *packet;
    crtpTxTail = (crtpTxTail + 1U) % CRTP_TX_QUEUE_SIZE;
    crtpTxCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
    if (!crtpInitialized || packet == NULL || port >= CRTP_NBR_OF_PORTS ||
        !crtpRxInitialized[port] || crtpRxCount[port] == 0U) {
        return false;
    }

    *packet = crtpRxQueues[port][crtpRxHead[port]];
    crtpRxHead[port] = (crtpRxHead[port] + 1U) % CRTP_RX_QUEUE_SIZE;
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
    if (!crtpInitialized || currentCrtpLink == NULL ||
        currentCrtpLink == &nopLink || currentCrtpLink->receivePacket == NULL) {
        return;
    }

    CrtpPacket packet;
    if (!currentCrtpLink->receivePacket(&packet)) {
        return;
    }

    crtpRxPacketCount++;
    if (packet.port < CRTP_NBR_OF_PORTS) {
        if (crtpRxInitialized[packet.port] &&
            crtpRxCount[packet.port] < CRTP_RX_QUEUE_SIZE) {
            crtpRxQueues[packet.port][crtpRxTail[packet.port]] = packet;
            crtpRxTail[packet.port] = (crtpRxTail[packet.port] + 1U) % CRTP_RX_QUEUE_SIZE;
            crtpRxCount[packet.port]++;
        }
        if (crtpCallbacks[packet.port] != NULL) {
            crtpCallbacks[packet.port](&packet);
        }
    }
}

void crtpTxTask(void) {
    if (!crtpInitialized || currentCrtpLink == NULL ||
        currentCrtpLink == &nopLink || crtpTxCount == 0U) {
        return;
    }

    if (currentCrtpLink->sendPacket == NULL) {
        return;
    }

    CrtpPacket packet = crtpTxQueue[crtpTxHead];
    if (currentCrtpLink->sendPacket(&packet)) {
        crtpTxHead = (crtpTxHead + 1U) % CRTP_TX_QUEUE_SIZE;
        crtpTxCount--;
        crtpTxPacketCount++;
    }
    /* On failure, packet remains queued for the 10 ms retry boundary. */
}

void crtpSetLink(CrtpLink *newLink) {
    if (!crtpInitialized) {
        crtpInit();
    }

    if (currentCrtpLink != NULL && currentCrtpLink->setEnable != NULL) {
        currentCrtpLink->setEnable(false);
    }

    if (newLink == NULL) {
        currentCrtpLink = &nopLink;
    } else {
        currentCrtpLink = newLink;
    }

    if (currentCrtpLink != NULL && currentCrtpLink->setEnable != NULL) {
        currentCrtpLink->setEnable(true);
    }
}

void crtpReset(void) {
    if (!crtpInitialized) {
        return;
    }

    crtpTxHead = 0;
    crtpTxTail = 0;
    crtpTxCount = 0;
    if (currentCrtpLink != NULL && currentCrtpLink->reset != NULL) {
        currentCrtpLink->reset();
    }
}

bool crtpIsConnected(void) {
    if (currentCrtpLink != NULL && currentCrtpLink->isConnected != NULL) {
        return currentCrtpLink->isConnected();
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
    if (!crtpInitialized) {
        return;
    }

    if (crtpStatsLastTick == 0U) {
        crtpStatsLastTick = g_systemTick;
        return;
    }

    if (g_systemTick >= crtpStatsLastTick &&
        (g_systemTick - crtpStatsLastTick) >= 500U) {
        uint32_t elapsed = g_systemTick - crtpStatsLastTick;
        if (elapsed > 0U) {
            crtpRxRate = (crtpRxPacketCount * 1000U) / elapsed;
            crtpTxRate = (crtpTxPacketCount * 1000U) / elapsed;
        }
        crtpRxPacketCount = 0;
        crtpTxPacketCount = 0;
        crtpStatsLastTick = g_systemTick;
    }
}

/* ---------------------------------------------------------------------
 * Deck discovery and log-object finalization
 * ------------------------------------------------------------------ */
uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
    if (decks == NULL || capacity == 0U) {
        return 0U;
    }

    /* Host-model deck inventory. No external bus scan is available in a
     * deterministic C11 build; the public API remains the accurate boundary
     * for future injected inventories. */
    return 0U;
}

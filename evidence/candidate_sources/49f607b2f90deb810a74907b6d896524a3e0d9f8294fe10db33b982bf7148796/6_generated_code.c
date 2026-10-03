#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#define FSE_PI 3.14159265358979323846f

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

TestState healthTestState = testDone;
uint8_t motorPass = 0U;
uint8_t batteryPass = 0U;
float batterySag = 0.0f;

StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0};
Axis3Log acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};

static uint32_t g_systemTick = 0U;
static bool g_trajectoryFlying = false;
static bool g_trajectoryFinished = false;
static bool g_trajectoryDisabled = false;
static bool g_deckFault = false;

static Setpoint g_commanderSetpoint = {0};
static int g_commanderActivePriority = COMMANDER_PRIORITY_LOWEST;
static uint32_t g_commanderLastUpdateTick = 0U;

static bool g_supervisorArmed = false;
static bool g_supervisorIsFlyingFlag = false;
static bool g_supervisorIsTumbledFlag = false;
static bool g_supervisorIsCrashedFlag = false;
static bool g_supervisorIsFreeFalling = false;
static bool g_seenFlight = false;
static uint32_t g_recentFlightTick = 0U;
static uint32_t g_tumbleTimerStart = 0U;
static bool g_tumbleTimerActive = false;
static uint32_t g_rpmLowStart = 0U;
static bool g_rpmLowActive = false;
static bool g_motorsNotResponding = false;
static uint32_t g_spinupStartTick = 0U;
static bool g_spinupStarted = false;
static uint32_t g_latestArmingTick = 0U;
static uint32_t g_latestLandingTick = 0U;
static uint32_t g_preflightTimeoutDuration = 5000U;
static uint32_t g_landingTimeoutDuration = 5000U;
static uint32_t g_lastWatchdogTick = 0U;
static bool g_autoArming = false;
static uint32_t g_spinupTimeoutMs = 0U;
static SupervisorState g_prevSupervisorState = supervisorStateLocked;
static SensorData g_supervisorSensors = {0};
static uint32_t g_supervisorMotorRatios[4] = {0};
static uint32_t g_supervisorIdleThrust = 0U;
static int32_t g_supervisorMotorRPMs[4] = {0};
static float g_crashDetectionGs = 0.0f;
static float g_freeFallThreshold = 0.0f;
static float g_acceptedTiltAccZ = 0.0f;
static float g_acceptedUpsideDownAccZ = 0.0f;
static uint32_t g_maxTiltTime = 0U;
static uint32_t g_maxUpsideDownTime = 0U;
static bool g_tumbleCheckEnabled = false;

static EstimatorMeasurement g_estimatorFifo[16];
static uint8_t g_fifoHead = 0U;
static uint8_t g_fifoTail = 0U;
static uint8_t g_fifoCount = 0U;

static bool g_stabilizerInitDone = false;
static uint32_t g_stabilizerStep = 0U;
static bool g_sensorDataReady = true;
static bool g_sensorActive = true;
static SensorData g_sensorData = {0};
static State g_stateEstimate = {0};
static uint16_t g_motorPWMs[4] = {0};
static bool g_highLevelPending = false;
static Setpoint g_highLevelSetpoint = {0};
static float g_pidDt = 0.002f;
static float g_filteredBattery = 0.0f;
static float g_batteryVoltage = 0.0f;
static float g_batteryNominalVoltage = 4.2f;

static bool g_propRequest = false;
static bool g_batRequest = false;
static uint32_t g_batteryTestTick = 0U;
static float g_idleVoltage = 0.0f;
static float g_minLoadedVoltage = 0.0f;
static int g_noiseCount = 0;
static float g_noiseSamples[PROPTEST_NBR_OF_VARIANCE_VALUES];
static float g_noiseVariance = 0.0f;
static uint8_t g_healthCurrentMotor = 0U;
static float g_batterySagThreshold = 0.5f;
static bool g_restartStarted = false;
static uint32_t g_restartStartTick = 0U;

static uint32_t g_rateSupervisorStartTick = 0U;
static bool g_rateSupervisorStartValid = false;
static bool g_rateSupervisorError = false;

static CrtpPacket g_crtpTxQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t g_crtpTxHead = 0U;
static uint16_t g_crtpTxTail = 0U;
static uint16_t g_crtpTxCount = 0U;
static bool g_crtpInitialized = false;
static bool g_crtpError = false;
static CrtpPortCallback g_crtpCallbacks[CRTP_NBR_OF_PORTS];
static uint32_t g_crtpTxCounter = 0U;
static uint32_t g_crtpRxCounter = 0U;
static uint32_t g_lastStatsTick = 0U;
static float g_crtpTxRate = 0.0f;
static float g_crtpRxRate = 0.0f;
static bool g_crtpRetryPending = false;
static uint32_t g_crtpLastTxAttempt = 0U;

typedef struct {
  CrtpPacket buffer[CRTP_RX_QUEUE_SIZE];
  uint8_t head, tail, count;
} CrtpRxQueue;
static CrtpRxQueue g_crtpRxQueues[CRTP_NBR_OF_PORTS];
static bool g_crtpRxQueueCreated[CRTP_NBR_OF_PORTS];

static bool nopSendPacket(CrtpPacket *packet) { (void)packet; return false; }
static bool nopReceivePacket(CrtpPacket *packet) { (void)packet; return false; }
static bool nopIsConnected(void) { return true; }
static void nopSetEnable(bool enable) { (void)enable; }
static void nopReset(void) { }
static CrtpLink g_nopLink = { nopSendPacket, nopReceivePacket, nopIsConnected, nopSetEnable, nopReset };
static CrtpLink *g_currentLink = &g_nopLink;

static void zeroSetpoint(Setpoint *sp) {
  if (sp != NULL) {
    memset(sp, 0, sizeof(*sp));
  }
}

static void pidResetToActual(PidObject *pid, float actual) {
  if (pid != NULL) {
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = actual;
    pid->initialized = true;
  }
}

static void pidReset(PidObject *pid) {
  if (pid != NULL) {
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
    pid->initialized = true;
  }
}

static float pidUpdate(PidObject *pid, float error) {
  if (pid == NULL) {
    return 0.0f;
  }
  if (!pid->initialized) {
    pid->integral = 0.0f;
    pid->prevError = error;
    pid->output = 0.0f;
    pid->initialized = true;
  }
  float dt = (g_pidDt > 0.0001f) ? g_pidDt : 0.002f;
  float p = pid->kp * error;
  float i = pid->integral + (pid->ki * error * dt);
  if (i > 100000.0f) i = 100000.0f;
  if (i < -100000.0f) i = -100000.0f;
  float d = (error - pid->prevError) / dt;
  float output = p + i + (pid->kd * d) + (pid->kff * error);
  pid->integral = i;
  pid->prevError = error;
  pid->output = output;
  return output;
}

static uint32_t quatcompress(float qw_in, float qx_in, float qy_in, float qz_in) {
  uint8_t b0 = (uint8_t)((qx_in + 1.0f) * 127.5f);
  uint8_t b1 = (uint8_t)((qy_in + 1.0f) * 127.5f);
  uint8_t b2 = (uint8_t)((qz_in + 1.0f) * 127.5f);
  uint8_t b3 = (uint8_t)((qw_in + 1.0f) * 127.5f);
  return ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | (uint32_t)b3;
}

static uint16_t motorForceToPwm(float force) {
  if (force <= 0.0f) {
    return 0U;
  }
  float normalized = force / CRAZYFLIE_MAX_MOTOR_FORCE_N;
  if (normalized > 1.0f) normalized = 1.0f;
  return (uint16_t)(normalized * 65535.0f);
}

static bool isSupervisorArmingAllowedState(SupervisorState state) {
  return state == supervisorStateArming ||
         state == supervisorStateReadyToFly ||
         state == supervisorStateFlying ||
         state == supervisorStateWarningLevelOut ||
         state == supervisorStateLanded;
}

static bool rxQueuePush(uint8_t port, const CrtpPacket *packet) {
  if (port >= CRTP_NBR_OF_PORTS || packet == NULL) {
    return false;
  }
  CrtpRxQueue *q = &g_crtpRxQueues[port];
  if (!g_crtpRxQueueCreated[port] || q->count >= CRTP_RX_QUEUE_SIZE) {
    return false;
  }
  q->buffer[q->tail] = *packet;
  q->tail = (uint8_t)((q->tail + 1U) % CRTP_RX_QUEUE_SIZE);
  q->count = (uint8_t)(q->count + 1U);
  return true;
}

static bool rxQueuePop(uint8_t port, CrtpPacket *packet) {
  if (port >= CRTP_NBR_OF_PORTS || packet == NULL) {
    return false;
  }
  CrtpRxQueue *q = &g_crtpRxQueues[port];
  if (!g_crtpRxQueueCreated[port] || q->count == 0U) {
    return false;
  }
  *packet = q->buffer[q->head];
  q->head = (uint8_t)((q->head + 1U) % CRTP_RX_QUEUE_SIZE);
  q->count = (uint8_t)(q->count - 1U);
  return true;
}

int16_t saturateSignedInt16(int32_t value) {
  if (value > 32767) return 32767;
  if (value < -32767) return -32767;
  return (int16_t)value;
}

float capAngle(float angle_deg) {
  float result = angle_deg;
  while (result > 180.0f) {
    result -= 360.0f;
  }
  while (result < -180.0f) {
    result += 360.0f;
  }
  return result;
}

float invSqrt(float x) {
  if (x <= 0.0f) {
    return 0.0f;
  }
  union {
    float f;
    int32_t i;
  } conv;
  float xhalf = x * 0.5f;
  conv.f = x;
  conv.i = 0x5f3759df - (conv.i >> 1);
  float y = conv.f;
  y = y * (1.5f - (xhalf * y * y));
  return y;
}

void estimatedGravityDirection(float qw_in, float qx_in, float qy_in, float qz_in,
                               float *gravX, float *gravY, float *gravZ) {
  if (gravX == NULL || gravY == NULL || gravZ == NULL) {
    return;
  }
  *gravX = 2.0f * ((qx_in * qz_in) - (qw_in * qy_in));
  *gravY = 2.0f * ((qw_in * qx_in) + (qy_in * qz_in));
  *gravZ = (qw_in * qw_in) - (qx_in * qx_in) - (qy_in * qy_in) + (qz_in * qz_in);
}

void sensfusion6Init(void) {
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
  sensfusion6IsInit = true;
  sensfusion6IsCalibrated = false;
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

bool sensfusion6Test(void) {
  return sensfusion6IsInit;
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt) {
  if (!sensfusion6IsInit) {
    sensfusion6Init();
  }
  float dtClamped = (dt > 0.0f) ? dt : 0.004f;
  float rx = gx * FSE_PI / 180.0f;
  float ry = gy * FSE_PI / 180.0f;
  float rz = gz * FSE_PI / 180.0f;

  if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
    float qDot0 = 0.5f * ((-qx * rx) - (qy * ry) - (qz * rz));
    float qDot1 = 0.5f * ((qw * rx) + (qy * rz) - (qz * ry));
    float qDot2 = 0.5f * ((qw * ry) - (qx * rz) + (qz * rx));
    float qDot3 = 0.5f * ((qw * rz) + (qx * ry) - (qy * rx));
    qw += qDot0 * dtClamped;
    qx += qDot1 * dtClamped;
    qy += qDot2 * dtClamped;
    qz += qDot3 * dtClamped;
  } else {
    float gravXtmp, gravYtmp, gravZtmp;
    estimatedGravityDirection(qw, qx, qy, qz, &gravXtmp, &gravYtmp, &gravZtmp);
    float accZproj = (ax * gravXtmp) + (ay * gravYtmp) + (az * gravZtmp);
    if (!sensfusion6IsCalibrated) {
      baseZacc = accZproj;
      sensfusion6IsCalibrated = true;
    }

    float invMag = invSqrt((ax * ax) + (ay * ay) + (az * az));
    float axn = ax * invMag;
    float ayn = ay * invMag;
    float azn = az * invMag;

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
    float halfvx = (qx * qz) - (qw * qy);
    float halfvy = (qw * qx) + (qy * qz);
    float halfvz = (qw * qw) - (qx * qx) - (qy * qy) + (qz * qz);
    float halfex = (ayn * halfvz) - (azn * halfvy);
    float halfey = (azn * halfvx) - (axn * halfvz);
    float halfez = (axn * halfvy) - (ayn * halfvx);
    rx += beta * halfex;
    ry += beta * halfey;
    rz += beta * halfez;
#else
    float halfvx = (qx * qz) - (qw * qy);
    float halfvy = (qw * qx) + (qy * qz);
    float halfvz = (qw * qw) - (qx * qx) - (qy * qy) + (qz * qz);
    float halfex = (ayn * halfvz) - (azn * halfvy);
    float halfey = (azn * halfvx) - (axn * halfvz);
    float halfez = (axn * halfvy) - (ayn * halfvx);

    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * dtClamped;
      integralFBy += twoKi * halfey * dtClamped;
      integralFBz += twoKi * halfez * dtClamped;
      rx += integralFBx;
      ry += integralFBy;
      rz += integralFBz;
    } else {
      integralFBx = 0.0f;
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    rx += twoKp * halfex;
    ry += twoKp * halfey;
    rz += twoKp * halfez;
#endif

    float qDot0 = 0.5f * ((-qx * rx) - (qy * ry) - (qz * rz));
    float qDot1 = 0.5f * ((qw * rx) + (qy * rz) - (qz * ry));
    float qDot2 = 0.5f * ((qw * ry) - (qx * rz) + (qz * rx));
    float qDot3 = 0.5f * ((qw * rz) + (qx * ry) - (qy * rx));
    qw += qDot0 * dtClamped;
    qx += qDot1 * dtClamped;
    qy += qDot2 * dtClamped;
    qz += qDot3 * dtClamped;
  }

  float qnorm = invSqrt((qw * qw) + (qx * qx) + (qy * qy) + (qz * qz));
  qw *= qnorm;
  qx *= qnorm;
  qy *= qnorm;
  qz *= qnorm;

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
  if (roll_deg == NULL || pitch_deg == NULL || yaw_deg == NULL) {
    return;
  }
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  float gx = gravityX;
  if (gx > 1.0f) gx = 1.0f;
  if (gx < -1.0f) gx = -1.0f;
  *roll_deg = atan2f(gravityY, gravityZ) * 180.0f / FSE_PI;
  *pitch_deg = asinf(-gx) * 180.0f / FSE_PI;
  *yaw_deg = atan2f(2.0f * ((qw * qz) + (qx * qy)), 1.0f - (2.0f * ((qy * qy) + (qz * qz)))) * 180.0f / FSE_PI;
}

void sensfusion6GetQuaternion(float *qw_out, float *qx_out, float *qy_out, float *qz_out) {
  if (qw_out == NULL || qx_out == NULL || qy_out == NULL || qz_out == NULL) {
    return;
  }
  *qw_out = qw;
  *qx_out = qx;
  *qy_out = qy;
  *qz_out = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
  float gx, gy, gz;
  estimatedGravityDirection(qw, qx, qy, qz, &gx, &gy, &gz);
  return (ax * gx) + (ay * gy) + (az * gz);
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out) {
  if (out == NULL) {
    return;
  }
  int32_t r = (int32_t)roll / 2;
  int32_t p = (int32_t)pitch / 2;
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
  motorForces[0] = thrustPart + rollPart - pitchPart + yawPart;
  motorForces[1] = thrustPart - rollPart - pitchPart - yawPart;
  motorForces[2] = thrustPart - rollPart + pitchPart - yawPart;
  motorForces[3] = thrustPart + rollPart + pitchPart + yawPart;
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
    float clamped = normalizedForces[i];
    if (clamped < 0.0f) clamped = 0.0f;
    if (clamped > 1.0f) clamped = 1.0f;
    motorPWMs[i] = (uint16_t)(clamped * 65535.0f);
  }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower) {
  if (control == NULL || motorPower == NULL) {
    return;
  }
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
      break;
  }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) {
  return (value < idleThrust) ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust) {
  PowerCapResult result = { false, 0 };
  if (motors == NULL) {
    return result;
  }
  int32_t maxMotor = motors[0];
  for (int i = 1; i < 4; ++i) {
    if (motors[i] > maxMotor) {
      maxMotor = motors[i];
    }
  }
  if (maxMotor > maxAllowedThrust) {
    result.isCapped = true;
    result.reduction = maxMotor - maxAllowedThrust;
    for (int i = 0; i < 4; ++i) {
      motors[i] -= result.reduction;
    }
  }
  for (int i = 0; i < 4; ++i) {
    motors[i] = capMinThrust(motors[i], idleThrust);
  }
  return result;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) {
  return filteredOld + (alpha * (supplyVoltage - filteredOld));
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust,
                                        float nominalVoltage,
                                        float actualVoltage) {
  if (actualVoltage <= 0.0f) {
    return motorThrust;
  }
  float compensated = roundf(((float)motorThrust * nominalVoltage) / actualVoltage);
  if (compensated < 0.0f) compensated = 0.0f;
  if (compensated > 65535.0f) compensated = 65535.0f;
  return (uint16_t)compensated;
}

void attitudeControllerInit(float updateDt) {
  if (pidRoll.initialized) {
    return;
  }
  if (updateDt > 0.0001f) {
    g_pidDt = updateDt;
  }
  pidResetToActual(&pidRoll, 0.0f);
  pidResetToActual(&pidPitch, 0.0f);
  pidResetToActual(&pidYaw, 0.0f);
  pidReset(&pidRollRate);
  pidReset(&pidPitchRate);
  pidReset(&pidYawRate);
  pidRoll.initialized = true;
  pidPitch.initialized = true;
  pidYaw.initialized = true;
  pidRollRate.initialized = true;
  pidPitchRate.initialized = true;
  pidYawRate.initialized = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
  float rollOut = pidUpdate(&pidRollRate, rollDesired - rollActual);
  float pitchOut = pidUpdate(&pidPitchRate, pitchDesired - pitchActual);
  float yawOut = pidUpdate(&pidYawRate, yawDesired - yawActual);
  pidRollRate.output = saturateSignedInt16((int32_t)rollOut);
  pidPitchRate.output = saturateSignedInt16((int32_t)pitchOut);
  pidYawRate.output = saturateSignedInt16((int32_t)yawOut);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
  pidRoll.output = pidUpdate(&pidRoll, rollDesired - rollActual);
  pidPitch.output = pidUpdate(&pidPitch, pitchDesired - pitchActual);
  if (pidYaw.initialized) {
    pidYaw.integral = 0.0f;
    pidYaw.prevError = 0.0f;
  }
  pidYaw.output = pidUpdate(&pidYaw, yawDesired - yawActual);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual) {
  pidResetToActual(&pidRoll, rollActual);
  pidResetToActual(&pidPitch, pitchActual);
  pidResetToActual(&pidYaw, yawActual);
  pidReset(&pidRollRate);
  pidReset(&pidPitchRate);
  pidReset(&pidYawRate);
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
  pidResetToActual(&pidRoll, rollActual);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
  pidResetToActual(&pidPitch, pitchActual);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw) {
  if (roll != NULL) {
    *roll = (int16_t)pidRollRate.output;
  }
  if (pitch != NULL) {
    *pitch = (int16_t)pidPitchRate.output;
  }
  if (yaw != NULL) {
    *yaw = (int16_t)pidYawRate.output;
  }
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
  if (setpoint == NULL || state == NULL) {
    return 0U;
  }
  float thrust = 32767.0f;
  if (setpoint->mode.z == modeVelocity) {
    float error = setpoint->velocity.z - state->velocity.z;
    thrust = 32767.0f + (9000.0f * error);
  } else if (setpoint->mode.z == modeAbs) {
    float error = setpoint->position.z - state->position.z;
    float verror = setpoint->velocity.z - state->velocity.z;
    thrust = 32767.0f + (9000.0f * error) + (500.0f * verror);
  } else {
    thrust = (float)setpoint->thrust;
  }
  if (thrust < 0.0f) thrust = 0.0f;
  if (thrust > 65535.0f) thrust = 65535.0f;
  return (uint16_t)thrust;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
  if (sensors == NULL || setpoint == NULL || state == NULL || control == NULL) {
    return;
  }

  uint16_t thrust = 0U;
  if (setpoint->mode.z == modeDisable) {
    thrust = setpoint->thrust;
  } else {
    thrust = positionControllerUpdate(setpoint, state);
  }

  if (thrust == 0U) {
    control->controlMode = controlModeLegacy;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    control->thrust = 0U;
    attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch,
                                  state->attitude.yaw);
    pidReset(&pidRoll);
    pidReset(&pidPitch);
    pidReset(&pidYaw);
    return;
  }

  float desiredYaw = g_stateEstimate.attitude.yaw;
  if (setpoint->mode.quat == modeAbs) {
    float qroll = 0.0f, qpitch = 0.0f, qyaw = 0.0f;
    float qw_est = setpoint->attitudeQuaternion.w;
    float qx_est = setpoint->attitudeQuaternion.x;
    float qy_est = setpoint->attitudeQuaternion.y;
    float qz_est = setpoint->attitudeQuaternion.z;
    float gx = 2.0f * ((qx_est * qz_est) - (qw_est * qy_est));
    float gy = 2.0f * ((qw_est * qx_est) + (qy_est * qz_est));
    if (gx > 1.0f) gx = 1.0f;
    if (gx < -1.0f) gx = -1.0f;
    qroll = atan2f(gy, 1.0f - (2.0f * ((qx_est * qx_est) + (qy_est * qy_est)))) * 180.0f / FSE_PI;
    qpitch = asinf(-gx) * 180.0f / FSE_PI;
    qyaw = atan2f(2.0f * ((qw_est * qz_est) + (qx_est * qy_est)), 1.0f - (2.0f * ((qy_est * qy_est) + (qz_est * qz_est)))) * 180.0f / FSE_PI;
    (void)qroll;
    (void)qpitch;
    desiredYaw = qyaw;
  } else if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
  } else if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = setpoint->attitude.yaw;
  }

  if (yawMaxDelta != 0.0f) {
    float delta = capAngle(desiredYaw - state->attitude.yaw);
    if (delta > yawMaxDelta) delta = yawMaxDelta;
    if (delta < -yawMaxDelta) delta = -yawMaxDelta;
    desiredYaw = state->attitude.yaw + delta;
    desiredYaw = capAngle(desiredYaw);
  }

  float rollDesiredRate = setpoint->attitudeRate.roll;
  float pitchDesiredRate = setpoint->attitudeRate.pitch;
  if (setpoint->mode.roll == modeVelocity) {
    attitudeControllerResetRollAttitudePID(state->attitude.roll);
  } else if (setpoint->mode.roll == modeAbs) {
    rollDesiredRate = pidUpdate(&pidRoll, setpoint->attitude.roll - state->attitude.roll);
  }

  if (setpoint->mode.pitch == modeVelocity) {
    attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
  } else if (setpoint->mode.pitch == modeAbs) {
    pitchDesiredRate = pidUpdate(&pidPitch, setpoint->attitude.pitch - state->attitude.pitch);
  }

  float yawDesiredRate = setpoint->attitudeRate.yaw;
  if (setpoint->mode.yaw == modeAbs || setpoint->mode.quat == modeAbs) {
    if (pidYaw.initialized) {
      pidYaw.integral = 0.0f;
      pidYaw.prevError = 0.0f;
    }
    yawDesiredRate = pidUpdate(&pidYaw, desiredYaw - state->attitude.yaw);
  }

  float rollActual = sensors->gyro.x;
  float pitchActual = -sensors->gyro.y;
  float yawActual = sensors->gyro.z;
  attitudeControllerCorrectRatePID(rollActual, rollDesiredRate,
                                   pitchActual, pitchDesiredRate,
                                   yawActual, yawDesiredRate);

  int16_t rollOut = 0;
  int16_t pitchOut = 0;
  int16_t yawOut = 0;
  attitudeControllerGetActuatorOutput(&rollOut, &pitchOut, &yawOut);
  control->controlMode = controlModeLegacy;
  control->roll = rollOut;
  control->pitch = pitchOut;
  control->yaw = (int16_t)(-yawOut);
  control->thrust = thrust;
}

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
  if (rollPrime == NULL || pitchPrime == NULL) {
    return;
  }
  float rad = yaw_deg * FSE_PI / 180.0f;
  float c = cosf(rad);
  float s = sinf(rad);
  *rollPrime = (roll * c) - (pitch * s);
  *pitchPrime = (roll * s) + (pitch * c);
}

static void applyDefaultStabModes(Setpoint *setpoint, float rollCmd, float pitchCmd,
                                  float rawYaw,
                                  StabilizationType rollMode, StabilizationType pitchMode,
                                  StabilizationType yawMode) {
  if (setpoint == NULL) {
    return;
  }
  if (rollMode == RATE) {
    setpoint->mode.roll = modeVelocity;
    setpoint->attitudeRate.roll = rollCmd;
    setpoint->attitude.roll = 0.0f;
  } else {
    setpoint->mode.roll = modeAbs;
    setpoint->attitude.roll = rollCmd;
    setpoint->attitudeRate.roll = 0.0f;
  }
  if (pitchMode == RATE) {
    setpoint->mode.pitch = modeVelocity;
    setpoint->attitudeRate.pitch = pitchCmd;
    setpoint->attitude.pitch = 0.0f;
  } else {
    setpoint->mode.pitch = modeAbs;
    setpoint->attitude.pitch = pitchCmd;
    setpoint->attitudeRate.pitch = 0.0f;
  }
  if (yawMode == RATE) {
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitudeRate.yaw = -rawYaw;
    setpoint->attitude.yaw = 0.0f;
  } else {
    setpoint->mode.yaw = modeAbs;
    setpoint->attitude.yaw = rawYaw;
    setpoint->attitudeRate.yaw = 0.0f;
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
    YawMode yawMode) {
  if (values == NULL || setpoint == NULL) {
    return;
  }

  uint16_t rawThrust = values->thrust;
  if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
    thrustLocked = true;
    if (rawThrust == 0U) {
      thrustLocked = false;
    }
  }

  zeroSetpoint(setpoint);

  if (yawMode == CAREFREE) {
    return;
  }

  float rollCmd = values->roll;
  float pitchCmd = values->pitch;
  if (yawMode == PLUSMODE) {
    rotateYaw(values->roll, values->pitch, 45.0f, &rollCmd, &pitchCmd);
  }

  if (altHoldMode) {
    setpoint->mode.z = modeVelocity;
    setpoint->thrust = 0U;
    setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
    if (!commanderModeSet) {
      commanderModeSet = true;
    }
    applyDefaultStabModes(setpoint, rollCmd, pitchCmd, values->yaw,
                          stabilizationModeRoll, stabilizationModePitch,
                          stabilizationModeYaw);
    return;
  }

  if (commanderModeSet) {
    setpoint->mode.z = modeDisable;
    commanderModeSet = false;
  }

  uint16_t thrustOut = 0U;
  if (thrustLocked || rawThrust < MIN_THRUST) {
    thrustOut = 0U;
  } else if (rawThrust > MAX_THRUST) {
    thrustOut = MAX_THRUST;
  } else {
    thrustOut = rawThrust;
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
    applyDefaultStabModes(setpoint, rollCmd, pitchCmd, values->yaw,
                          stabilizationModeRoll, stabilizationModePitch,
                          stabilizationModeYaw);
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->thrust = thrustOut;
    return;
  }

  if (posSetMode && rawThrust != 0U) {
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;
    setpoint->mode.quat = modeDisable;
    setpoint->position.x = -values->pitch;
    setpoint->position.y = values->roll;
    setpoint->position.z = (float)rawThrust / 1000.0f;
    setpoint->attitude.yaw = values->yaw;
    setpoint->thrust = 0U;
    return;
  }

  applyDefaultStabModes(setpoint, rollCmd, pitchCmd, values->yaw,
                        stabilizationModeRoll, stabilizationModePitch,
                        stabilizationModeYaw);
  setpoint->thrust = thrustOut;
}

void supervisorInit(void) {
  supervisorState = supervisorStateLocked;
  supervisorConditionBits = 0U;
  g_supervisorArmed = false;
  g_supervisorIsFlyingFlag = false;
  g_supervisorIsTumbledFlag = false;
  g_supervisorIsCrashedFlag = false;
  g_supervisorIsFreeFalling = false;
  g_seenFlight = false;
  g_recentFlightTick = 0U;
  g_tumbleTimerStart = 0U;
  g_tumbleTimerActive = false;
  g_rpmLowStart = 0U;
  g_rpmLowActive = false;
  g_motorsNotResponding = false;
  g_spinupStarted = false;
  g_spinupStartTick = 0U;
  g_autoArming = false;
  g_spinupTimeoutMs = 0U;
  g_prevSupervisorState = supervisorStateLocked;
  g_crashDetectionGs = 0.0f;
  g_freeFallThreshold = 0.0f;
  g_acceptedTiltAccZ = 0.0f;
  g_acceptedUpsideDownAccZ = 0.0f;
  g_maxTiltTime = 0U;
  g_maxUpsideDownTime = 0U;
  g_tumbleCheckEnabled = false;
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
  return g_supervisorArmed;
}

bool supervisorIsCrashed(void) {
  return g_supervisorIsCrashedFlag;
}

bool supervisorRequestArming(bool doArm) {
  if (!doArm) {
    g_supervisorArmed = false;
    if (supervisorConditionBits & SUPERVISOR_CB_ARMED) {
      supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    }
    return true;
  }

  if (g_supervisorArmed && supervisorState == supervisorStateArming) {
    return true;
  }

  if (!supervisorCanArm()) {
    return false;
  }

  g_supervisorArmed = true;
  supervisorConditionBits |= SUPERVISOR_CB_ARMED;
  supervisorState = supervisorStateArming;
  g_spinupStarted = true;
  g_spinupStartTick = g_systemTick;
  g_latestArmingTick = g_systemTick;
  return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
  if (g_supervisorIsTumbledFlag) {
    return false;
  }
  if (doRecovery) {
    g_supervisorIsCrashedFlag = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    if (supervisorState == supervisorStateCrashed) {
      supervisorState = supervisorStatePreFlChecksPassed;
    }
    return true;
  }

  g_supervisorIsCrashedFlag = true;
  supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
  supervisorState = supervisorStateCrashed;
  return true;
}

bool supervisorAreMotorsAllowedToRun(void) {
  return isSupervisorArmingAllowedState(supervisorState);
}

uint16_t supervisorGetInfoBitfield(void) {
  uint16_t bits = 0U;
  if (supervisorCanArm()) bits |= (1U << 0);
  if (g_supervisorArmed) bits |= (1U << 1);
  if (g_autoArming) bits |= (1U << 2);
  if (supervisorCanFly()) bits |= (1U << 3);
  if (g_supervisorIsFlyingFlag) bits |= (1U << 4);
  if (g_supervisorIsTumbledFlag) bits |= (1U << 5);
  if (supervisorState == supervisorStateLocked) bits |= (1U << 6);
  if (g_supervisorIsCrashedFlag) bits |= (1U << 7);
  if (g_trajectoryFlying) bits |= (1U << 8);
  if (g_trajectoryFinished) bits |= (1U << 9);
  if (g_trajectoryDisabled) bits |= (1U << 10);
  if (g_deckFault) bits |= (1U << 11);
  return bits;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick) {
  if (motorRatios == NULL) {
    return false;
  }
  bool motorAbove = false;
  for (int i = 0; i < 4; ++i) {
    if (motorRatios[i] > idleThrust) {
      motorAbove = true;
      break;
    }
  }
  if (motorAbove) {
    g_seenFlight = true;
    g_recentFlightTick = currentTick;
  }
  if (!g_seenFlight) {
    g_supervisorIsFlyingFlag = false;
    return false;
  }
  if ((currentTick - g_recentFlightTick) >= IS_FLYING_HYSTERESIS_THRESHOLD) {
    g_supervisorIsFlyingFlag = false;
    return false;
  }
  g_supervisorIsFlyingFlag = true;
  return true;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling) {
  if (isFreeFalling == NULL) {
    return false;
  }
  *isFreeFalling = false;

  float norm = sqrtf((accX * accX) + (accY * accY) + (accZ * accZ));
  if (crashDetectionGs > 0.0f && fabsf(norm - 1.0f) > crashDetectionGs) {
    g_supervisorIsCrashedFlag = true;
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
  }

  if (!tumbleCheckEnabled) {
    g_supervisorIsTumbledFlag = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    return false;
  }

  if (fabsf(accX) < freeFallThreshold &&
      fabsf(accY) < freeFallThreshold &&
      fabsf(accZ) < freeFallThreshold) {
    g_supervisorIsFreeFalling = true;
    *isFreeFalling = true;
    supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
    g_tumbleTimerActive = false;
    g_tumbleTimerStart = 0U;
    g_supervisorIsTumbledFlag = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    return false;
  }

  g_supervisorIsFreeFalling = false;
  supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

  uint32_t timeout = 0U;
  bool belowThreshold = false;
  if (accZ < acceptedUpsideDownAccZ) {
    belowThreshold = true;
    timeout = maxUpsideDownTime;
  } else if (accZ < acceptedTiltAccZ) {
    belowThreshold = true;
    timeout = maxTiltTime;
  }

  if (belowThreshold) {
    if (!g_tumbleTimerActive) {
      g_tumbleTimerActive = true;
      g_tumbleTimerStart = currentTick;
    }
    if ((currentTick - g_tumbleTimerStart) >= timeout) {
      g_supervisorIsTumbledFlag = true;
      supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    }
  } else {
    g_tumbleTimerActive = false;
    g_tumbleTimerStart = 0U;
    g_supervisorIsTumbledFlag = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
  }

  if (g_supervisorIsTumbledFlag) {
    supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
  }
  return g_supervisorIsTumbledFlag;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick) {
  if (lastNotificationTick == 0U) {
    return true;
  }
  return (currentTick - lastNotificationTick) < DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration) {
  if (state != supervisorStatePreFlChecksPassed &&
      state != supervisorStateReadyToFly) {
    return false;
  }
  if (latestArmingTick == 0U || preflightTimeoutDuration == 0U) {
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
    StabilizationMode zMode = setpoint->mode.z;
    Axis3f zPos = setpoint->position;
    Axis3f zVel = setpoint->velocity;
    uint16_t zThrust = setpoint->thrust;
    zeroSetpoint(setpoint);
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitude.roll = 0.0f;
    setpoint->attitude.pitch = 0.0f;
    setpoint->attitudeRate.yaw = 0.0f;
    setpoint->mode.z = zMode;
    setpoint->position = zPos;
    setpoint->velocity = zVel;
    setpoint->thrust = zThrust;
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
  if (motorRPMs == NULL) {
    return false;
  }
  if (!canFly) {
    g_rpmLowActive = false;
    g_rpmLowStart = 0U;
    g_motorsNotResponding = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return false;
  }

  bool anyLow = false;
  for (int i = 0; i < 4; ++i) {
    if (motorRPMs[i] < rpmThreshold) {
      anyLow = true;
      break;
    }
  }

  if (!anyLow) {
    g_rpmLowActive = false;
    g_rpmLowStart = 0U;
    g_motorsNotResponding = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return false;
  }

  if (!g_rpmLowActive) {
    g_rpmLowActive = true;
    g_rpmLowStart = currentTick;
  }
  if ((currentTick - g_rpmLowStart) >= rpmCheckDurationMs) {
    g_motorsNotResponding = true;
    supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
  }
  return g_motorsNotResponding;
}

void supervisorSetSensorData(const SensorData *sensors) {
  if (sensors == NULL) {
    return;
  }
  g_supervisorSensors = *sensors;
  g_sensorData = *sensors;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
  if (motorRatios == NULL) {
    return;
  }
  for (int i = 0; i < 4; ++i) {
    g_supervisorMotorRatios[i] = motorRatios[i];
  }
  g_supervisorIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
  if (motorRPMs == NULL) {
    return;
  }
  for (int i = 0; i < 4; ++i) {
    g_supervisorMotorRPMs[i] = motorRPMs[i];
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

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
  g_autoArming = autoArming;
  g_spinupTimeoutMs = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep) {
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) {
    return;
  }

  bool prevAutoArming = g_autoArming;
  SupervisorState beforeAuto = supervisorState;

  g_supervisorIsFlyingFlag = isFlyingCheck(g_supervisorMotorRatios,
                                           g_supervisorIdleThrust,
                                           g_systemTick);
  bool freeFall = false;
  g_supervisorIsTumbledFlag = isTumbledCheck(
      g_supervisorSensors.acc.x, g_supervisorSensors.acc.y, g_supervisorSensors.acc.z,
      g_crashDetectionGs, g_freeFallThreshold, g_acceptedTiltAccZ,
      g_acceptedUpsideDownAccZ, g_maxTiltTime, g_maxUpsideDownTime,
      g_tumbleCheckEnabled, g_systemTick, &freeFall);
  g_supervisorIsFreeFalling = freeFall;

  uint32_t age = commanderGetInactivityTime();
  if (age >= COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
    supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  } else if (age >= COMMANDER_WDT_TIMEOUT_STABILIZE) {
    supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
  }

  bool alwaysWatchdogHealthy = checkEmergencyStopWatchdog(g_systemTick, g_lastWatchdogTick);
  if (!alwaysWatchdogHealthy) {
    supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
  }

  if (supervisorIsPreflightTimeout(supervisorState, g_latestArmingTick,
                                   g_systemTick, g_preflightTimeoutDuration)) {
    supervisorConditionBits |= SUPERVISOR_CB_PREFLIGHT_TIMEOUT;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_PREFLIGHT_TIMEOUT;
  }

  if (supervisorIsLandingTimeout(g_latestLandingTick, g_systemTick,
                                 g_landingTimeoutDuration)) {
    supervisorConditionBits |= SUPERVISOR_CB_LANDING_TIMEOUT;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_LANDING_TIMEOUT;
  }

  if (supervisorState == supervisorStateArming) {
    if (!g_spinupStarted) {
      g_spinupStarted = true;
      g_spinupStartTick = g_systemTick;
    }
    if ((g_systemTick - g_spinupStartTick) >= g_spinupTimeoutMs) {
      supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
    } else {
      supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }
  } else {
    g_spinupStarted = false;
    g_spinupStartTick = 0U;
    supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
  }

  if (isRPMatArmingValid(g_supervisorMotorRPMs, 0, INT32_MAX)) {
    supervisorConditionBits |= SUPERVISOR_CB_RPM_AT_ARMING_VALID;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_RPM_AT_ARMING_VALID;
  }

  isMotorsNotResponding(g_supervisorMotorRPMs, 0, 0U, supervisorCanFly(), g_systemTick);

  if (g_supervisorArmed) {
    supervisorConditionBits |= SUPERVISOR_CB_ARMED;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
  }
  if (g_supervisorIsFlyingFlag) {
    supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
  }
  if (g_supervisorIsCrashedFlag) {
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
  }
  if (g_supervisorIsTumbledFlag) {
    supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
  }

  if (prevAutoArming && beforeAuto == supervisorStatePreFlChecksPassed &&
      g_prevSupervisorState != supervisorStatePreFlChecksPassed) {
    supervisorRequestArming(true);
  }

  if (!isSupervisorArmingAllowedState(supervisorState) && g_supervisorArmed) {
    g_supervisorArmed = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
  }

  g_prevSupervisorState = supervisorState;
  supervisorLog.info = supervisorGetInfoBitfield();
  supervisorLog.accNorm = sqrtf((g_supervisorSensors.acc.x * g_supervisorSensors.acc.x) +
                                (g_supervisorSensors.acc.y * g_supervisorSensors.acc.y) +
                                (g_supervisorSensors.acc.z * g_supervisorSensors.acc.z));
}

bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
  if (measurement == NULL || g_fifoCount >= 16U) {
    return false;
  }
  g_estimatorFifo[g_fifoTail] = *measurement;
  g_fifoTail = (uint8_t)((g_fifoTail + 1U) % 16U);
  g_fifoCount = (uint8_t)(g_fifoCount + 1U);
  return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
  if (measurement == NULL || g_fifoCount == 0U) {
    return false;
  }
  *measurement = g_estimatorFifo[g_fifoHead];
  g_fifoHead = (uint8_t)((g_fifoHead + 1U) % 16U);
  g_fifoCount = (uint8_t)(g_fifoCount - 1U);
  return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
  EstimatorMeasurement lastGyro = { MeasurementTypeGyroscope, {0.0f, 0.0f, 0.0f} };
  EstimatorMeasurement lastAccel = { MeasurementTypeAcceleration, {0.0f, 0.0f, 0.0f} };
  while (estimatorDequeue(&lastGyro)) {
    (void)lastGyro;
  }
  while (g_fifoCount > 0U) {
    EstimatorMeasurement item;
    if (estimatorDequeue(&item)) {
      if (item.type == MeasurementTypeGyroscope) {
        lastGyro = item;
      } else if (item.type == MeasurementTypeAcceleration) {
        lastAccel = item;
      }
    }
  }

  if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
    if (lastGyro.type != MeasurementTypeGyroscope) {
      lastGyro.data[0] = g_sensorData.gyro.x;
      lastGyro.data[1] = g_sensorData.gyro.y;
      lastGyro.data[2] = g_sensorData.gyro.z;
    }
    if (lastAccel.type != MeasurementTypeAcceleration) {
      lastAccel.data[0] = g_sensorData.acc.x;
      lastAccel.data[1] = g_sensorData.acc.y;
      lastAccel.data[2] = g_sensorData.acc.z;
    }
    sensfusion6UpdateQ(lastGyro.data[0], lastGyro.data[1], lastGyro.data[2],
                       lastAccel.data[0], lastAccel.data[1], lastAccel.data[2],
                       0.004f);
    sensfusion6GetEulerRPY(&g_stateEstimate.attitude.roll,
                           &g_stateEstimate.attitude.pitch,
                           &g_stateEstimate.attitude.yaw);
    sensfusion6GetQuaternion(&g_stateEstimate.attitudeQuaternion.w,
                             &g_stateEstimate.attitudeQuaternion.x,
                             &g_stateEstimate.attitudeQuaternion.y,
                             &g_stateEstimate.attitudeQuaternion.z);
    g_stateEstimate.acc.x = sensfusion6GetAccZWithoutGravity(lastAccel.data[0],
                                                            lastAccel.data[1],
                                                            lastAccel.data[2]);
  }

  if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
    g_stateEstimate.position.z += g_stateEstimate.velocity.z * 0.01f;
  }
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
  if (setpoint == NULL) {
    return false;
  }
  if (priority == COMMANDER_PRIORITY_DISABLE ||
      priority >= g_commanderActivePriority) {
    g_commanderSetpoint = *setpoint;
    g_commanderActivePriority = priority;
    g_commanderLastUpdateTick = g_systemTick;
    if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
      g_trajectoryFlying = false;
      g_trajectoryFinished = true;
      g_trajectoryDisabled = true;
    }
    return true;
  }
  return false;
}

void commanderRelaxPriority(void) {
  g_commanderActivePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
  return g_systemTick - g_commanderLastUpdateTick;
}

int commanderGetActivePriority(void) {
  return g_commanderActivePriority;
}

static void sensorsInit(void) {
  g_sensorDataReady = true;
  g_sensorActive = true;
  memset(&g_sensorData, 0, sizeof(g_sensorData));
}

static void stateEstimatorInit(void) {
  sensfusion6Init();
  memset(&g_stateEstimate, 0, sizeof(g_stateEstimate));
  g_fifoHead = 0U;
  g_fifoTail = 0U;
  g_fifoCount = 0U;
}

static void controllerInit(void) {
  attitudeControllerInit(1.0f / (float)ATTITUDE_RATE_HZ);
}

static void powerDistributionInit(void) {
  memset(&g_motorPWMs, 0, sizeof(g_motorPWMs));
  motor.m1req = 0U;
  motor.m2req = 0U;
  motor.m3req = 0U;
  motor.m4req = 0U;
}

static void motorsInit(void) {
  memset(&g_motorPWMs, 0, sizeof(g_motorPWMs));
  motor.m1req = 0U;
  motor.m2req = 0U;
  motor.m3req = 0U;
  motor.m4req = 0U;
}

static void collisionAvoidanceInit(void) {
}

static void sensorsWaitDataReady(void) {
  if (!g_sensorDataReady) {
    g_sensorDataReady = true;
  }
}

static void sensorsAcquire(void) {
  g_sensorActive = true;
  gyro.x = g_sensorData.gyro.x;
  gyro.y = g_sensorData.gyro.y;
  gyro.z = g_sensorData.gyro.z;
  acc.x = g_sensorData.acc.x;
  acc.y = g_sensorData.acc.y;
  acc.z = g_sensorData.acc.z;
  baro.asl = g_sensorData.baroAsl;
  baro.temp = g_sensorData.baroTemperature;
  baro.pressure = g_sensorData.baroPressure;
}

static void stateEstimator(void) {
  estimatorComplementary(g_stabilizerStep);
  g_stateEstimate.acc.x = g_sensorData.acc.x;
  g_stateEstimate.acc.y = g_sensorData.acc.y;
  g_stateEstimate.acc.z = g_sensorData.acc.z;
  sensfusion6GetEulerRPY(&g_stateEstimate.attitude.roll,
                         &g_stateEstimate.attitude.pitch,
                         &g_stateEstimate.attitude.yaw);
  sensfusion6GetQuaternion(&g_stateEstimate.attitudeQuaternion.w,
                           &g_stateEstimate.attitudeQuaternion.x,
                           &g_stateEstimate.attitudeQuaternion.y,
                           &g_stateEstimate.attitudeQuaternion.z);
  stateEstimate.roll = g_stateEstimate.attitude.roll;
  stateEstimate.pitch = g_stateEstimate.attitude.pitch;
  stateEstimate.yaw = g_stateEstimate.attitude.yaw;
  stateEstimate.qx = g_stateEstimate.attitudeQuaternion.x;
  stateEstimate.qy = g_stateEstimate.attitudeQuaternion.y;
  stateEstimate.qz = g_stateEstimate.attitudeQuaternion.z;
  stateEstimate.qw = g_stateEstimate.attitudeQuaternion.w;
}

static void collisionAvoidanceUpdateSetpoint(Setpoint *setpoint) {
  (void)setpoint;
}

static void setMotorRatios(const int32_t motors[4]) {
  if (motors == NULL) {
    return;
  }
  for (int i = 0; i < 4; ++i) {
    if (motors[i] <= 0) {
      g_motorPWMs[i] = 0U;
    } else if (motors[i] >= 65535) {
      g_motorPWMs[i] = 65535U;
    } else {
      g_motorPWMs[i] = (uint16_t)motors[i];
    }
  }
  motor.m1req = g_motorPWMs[0];
  motor.m2req = g_motorPWMs[1];
  motor.m3req = g_motorPWMs[2];
  motor.m4req = g_motorPWMs[3];
}

void stabilizerInit(void) {
  if (g_stabilizerInitDone) {
    return;
  }
  sensorsInit();
  stateEstimatorInit();
  controllerInit();
  powerDistributionInit();
  motorsInit();
  collisionAvoidanceInit();
  g_stabilizerInitDone = true;
}

void stabilizerTask(void) {
  g_systemTick++;

  if (g_highLevelPending) {
    commanderSetSetpoint(&g_highLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
    g_highLevelPending = false;
  }

  if (healthShallWeRunTest()) {
    healthRunTests(&g_sensorData);
    g_stabilizerStep++;
    return;
  }

  sensorsWaitDataReady();
  sensorsAcquire();
  stateEstimator();

  Setpoint setpoint = g_commanderSetpoint;
  supervisorUpdate(g_stabilizerStep);
  collisionAvoidanceUpdateSetpoint(&setpoint);
  supervisorOverrideSetpoint(&setpoint, supervisorConditionBits, supervisorState);
  if (!supervisorCanFly()) {
    zeroSetpoint(&setpoint);
  }

  ControlData control = {0};
  controllerPid(&g_sensorData, &setpoint, &g_stateEstimate, &control,
                0.0f, 1.0f / (float)ATTITUDE_RATE_HZ);

  MotorPower mp = {0};
  powerDistribution(&control, &mp);

  g_filteredBattery = batteryCompensation(g_batteryVoltage, g_filteredBattery, 0.01f);
  int32_t motors[4] = { mp.m1, mp.m2, mp.m3, mp.m4 };
  if (g_batteryVoltage > 0.0f) {
    for (int i = 0; i < 4; ++i) {
      if (motors[i] >= 0 && motors[i] <= 65535) {
        motors[i] = motorsCompensateBatteryVoltage((uint16_t)motors[i],
                                                    g_batteryNominalVoltage,
                                                    g_batteryVoltage);
      }
    }
  }

  powerDistributionCap(motors, 65535, 0);

  if (!supervisorAreMotorsAllowedToRun()) {
    for (int i = 0; i < 4; ++i) {
      motors[i] = 0;
    }
  }

  setMotorRatios(motors);
  g_stabilizerStep++;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
  if (setpoint == NULL) {
    return false;
  }
  g_highLevelSetpoint = *setpoint;
  g_highLevelPending = true;
  return true;
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output) {
  if (state == NULL || sensors == NULL || output == NULL) {
    return;
  }
  for (int i = 0; i < 3; ++i) {
    output->position_mm[i] = (int32_t)(state->position.x * 1000.0f);
    if (i == 0) {
      output->position_mm[i] = (int32_t)(state->position.x * 1000.0f);
    } else if (i == 1) {
      output->position_mm[i] = (int32_t)(state->position.y * 1000.0f);
    } else {
      output->position_mm[i] = (int32_t)(state->position.z * 1000.0f);
    }
    output->velocity_mms[i] = (int32_t)(state->velocity.x * 1000.0f);
    if (i == 0) {
      output->velocity_mms[i] = (int32_t)(state->velocity.x * 1000.0f);
    } else if (i == 1) {
      output->velocity_mms[i] = (int32_t)(state->velocity.y * 1000.0f);
    } else {
      output->velocity_mms[i] = (int32_t)(state->velocity.z * 1000.0f);
    }
  }
  output->acceleration_mms2[0] = (int32_t)(sensors->acc.x * 9810.0f);
  output->acceleration_mms2[1] = (int32_t)(sensors->acc.y * 9810.0f);
  output->acceleration_mms2[2] = (int32_t)((sensors->acc.z + 1.0f) * 9810.0f);
  output->gyro_millirad_s[0] = sensors->gyro.x * FSE_PI / 180.0f * 1000.0f;
  output->gyro_millirad_s[1] = -sensors->gyro.y * FSE_PI / 180.0f * 1000.0f;
  output->gyro_millirad_s[2] = sensors->gyro.z * FSE_PI / 180.0f * 1000.0f;
  output->quatCompressed = quatcompress(state->attitudeQuaternion.w,
                                        state->attitudeQuaternion.x,
                                        state->attitudeQuaternion.y,
                                        state->attitudeQuaternion.z);
}

bool rateSupervisorValidate(uint32_t measuredRate) {
  return (measuredRate >= 997U && measuredRate <= 1003U);
}

void rateSupervisorTask(void) {
  if (!g_rateSupervisorStartValid) {
    g_rateSupervisorStartValid = true;
    g_rateSupervisorStartTick = g_systemTick;
  }
  if ((g_systemTick - g_rateSupervisorStartTick) >= 2000U) {
    if (g_sensorActive) {
      g_rateSupervisorError = true;
    }
  }
}

bool healthShallWeRunTest(void) {
  if (g_propRequest) {
    g_propRequest = false;
    g_batRequest = false;
    healthTestState = configureAcc;
    g_noiseCount = 0;
    g_noiseVariance = 0.0f;
    g_healthCurrentMotor = 0U;
    g_idleVoltage = g_batteryVoltage;
    healthLog.motorTestCount = 0U;
    motorPass = 0U;
    return true;
  }
  if (g_batRequest) {
    g_batRequest = false;
    g_propRequest = false;
    healthTestState = testBattery;
    g_batteryTestTick = 0U;
    g_minLoadedVoltage = g_batteryVoltage;
    g_idleVoltage = g_batteryVoltage;
    return true;
  }
  return healthTestState != testDone;
}

void healthRunTests(const SensorData *sensorData) {
  if (sensorData == NULL) {
    return;
  }

  switch (healthTestState) {
    case configureAcc: {
      healthLog.motorTestCount = 0U;
      motorPass = 0U;
      g_noiseCount = 0;
      g_noiseVariance = 0.0f;
      g_healthCurrentMotor = 0U;
      g_idleVoltage = g_batteryVoltage;
      healthTestState = measureNoiseFloor;
      break;
    }
    case measureNoiseFloor: {
      if (g_noiseCount < PROPTEST_NBR_OF_VARIANCE_VALUES) {
        g_noiseSamples[g_noiseCount] = sensorData->acc.z;
        g_noiseCount++;
      }
      if (g_noiseCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
        g_noiseVariance = variance(g_noiseSamples, PROPTEST_NBR_OF_VARIANCE_VALUES);
        healthTestState = measureProp;
        g_healthCurrentMotor = 0U;
      }
      break;
    }
    case measureProp: {
      if (g_healthCurrentMotor < 4U) {
        float measured = fabsf(sensorData->acc.x) + fabsf(sensorData->acc.y);
        evaluatePropTest(0.0f, 1.0f, measured, g_healthCurrentMotor);
        g_healthCurrentMotor++;
        healthLog.motorTestCount = g_healthCurrentMotor;
      } else {
        healthTestState = evaluatePropResult;
      }
      break;
    }
    case evaluatePropResult: {
      healthTestState = testDone;
      break;
    }
    case testBattery: {
      if (g_batteryTestTick == 0U) {
        g_minLoadedVoltage = g_batteryVoltage;
      } else if (g_batteryTestTick == 1U) {
        g_minLoadedVoltage = g_batteryVoltage;
      } else if (g_batteryTestTick >= 2U && g_batteryTestTick <= 49U) {
        float loaded = g_batteryVoltage - (0.01f * (float)(g_batteryTestTick - 1U));
        if (loaded < g_minLoadedVoltage) {
          g_minLoadedVoltage = loaded;
        }
      } else if (g_batteryTestTick >= 50U) {
        batterySag = g_idleVoltage - g_minLoadedVoltage;
        if (batterySag > g_batterySagThreshold) {
          batteryPass = 0U;
        } else {
          batteryPass = 1U;
        }
        healthTestState = evaluateBatResult;
      }
      g_batteryTestTick++;
      break;
    }
    case evaluateBatResult: {
      healthTestState = testDone;
      break;
    }
    case restartBatTest: {
      if (!g_restartStarted) {
        g_restartStarted = true;
        g_restartStartTick = g_systemTick;
      }
      if ((g_systemTick - g_restartStartTick) >= 2000U) {
        g_restartStarted = false;
        healthTestState = testBattery;
        g_batteryTestTick = 0U;
        g_minLoadedVoltage = g_batteryVoltage;
        g_idleVoltage = g_batteryVoltage;
      }
      break;
    }
    case testDone:
    default:
      break;
  }

  healthLog.batterySag = batterySag;
  healthLog.batteryPass = batteryPass;
  healthLog.motorPass = motorPass;
  healthLog.motorTestCount = g_healthCurrentMotor;
}

void healthRequestPropTest(void) {
  g_propRequest = true;
  g_batRequest = false;
}

void healthRequestBatteryTest(void) {
  g_batRequest = true;
  g_propRequest = false;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex) {
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

float variance(const float *buffer, int length) {
  if (buffer == NULL || length <= 0) {
    return 0.0f;
  }
  float sum = 0.0f;
  float sumSq = 0.0f;
  for (int i = 0; i < length; ++i) {
    float v = buffer[i];
    sum += v;
    sumSq += (v * v);
  }
  return sumSq - ((sum * sum) / (float)length);
}

void crtpInit(void) {
  if (g_crtpInitialized) {
    return;
  }
  g_crtpTxHead = 0U;
  g_crtpTxTail = 0U;
  g_crtpTxCount = 0U;
  g_currentLink = &g_nopLink;
  for (int i = 0; i < (int)CRTP_NBR_OF_PORTS; ++i) {
    g_crtpCallbacks[i] = NULL;
    g_crtpRxQueues[i].head = 0U;
    g_crtpRxQueues[i].tail = 0U;
    g_crtpRxQueues[i].count = 0U;
    g_crtpRxQueueCreated[i] = false;
  }
  g_crtpError = false;
  g_crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port) {
  if (port >= CRTP_NBR_OF_PORTS) {
    return;
  }
  if (g_crtpRxQueueCreated[port]) {
    g_crtpError = true;
    return;
  }
  g_crtpRxQueues[port].head = 0U;
  g_crtpRxQueues[port].tail = 0U;
  g_crtpRxQueues[port].count = 0U;
  g_crtpRxQueueCreated[port] = true;
}

bool crtpSendPacket(const CrtpPacket *packet) {
  if (packet == NULL || g_crtpTxCount >= CRTP_TX_QUEUE_SIZE) {
    return false;
  }
  g_crtpTxQueue[g_crtpTxTail] = *packet;
  g_crtpTxTail = (uint16_t)((g_crtpTxTail + 1U) % CRTP_TX_QUEUE_SIZE);
  g_crtpTxCount = (uint16_t)(g_crtpTxCount + 1U);
  g_crtpTxCounter++;
  return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
  return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
  if (port >= CRTP_NBR_OF_PORTS || packet == NULL) {
    return false;
  }
  return rxQueuePop(port, packet);
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet) {
  if (port >= CRTP_NBR_OF_PORTS || packet == NULL) {
    return false;
  }
  if (rxQueuePop(port, packet)) {
    return true;
  }
  return false;
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet,
                           uint32_t wait_ms) {
  (void)wait_ms;
  return crtpReceivePacket(port, packet);
}

void crtpRxTask(void) {
  if (g_currentLink == NULL || g_currentLink == &g_nopLink) {
    return;
  }
  if (g_currentLink->receivePacket == NULL) {
    return;
  }
  CrtpPacket packet;
  if (!g_currentLink->receivePacket(&packet)) {
    return;
  }
  g_crtpRxCounter++;
  bool queued = false;
  bool called = false;
  if (packet.port < CRTP_NBR_OF_PORTS) {
    if (g_crtpRxQueueCreated[packet.port]) {
      queued = rxQueuePush(packet.port, &packet);
    }
    if (g_crtpCallbacks[packet.port] != NULL) {
      CrtpPacket cbPacket = packet;
      g_crtpCallbacks[packet.port](&cbPacket);
      called = true;
    }
  }
  if (!queued && !called) {
    return;
  }
}

void crtpTxTask(void) {
  if (g_currentLink == NULL || g_currentLink == &g_nopLink) {
    return;
  }
  if (g_currentLink->sendPacket == NULL || g_crtpTxCount == 0U) {
    return;
  }
  if (g_crtpRetryPending &&
      (g_systemTick - g_crtpLastTxAttempt) < 10U) {
    return;
  }
  g_crtpLastTxAttempt = g_systemTick;
  if (!g_currentLink->sendPacket(&g_crtpTxQueue[g_crtpTxHead])) {
    g_crtpRetryPending = true;
    return;
  }
  g_crtpTxHead = (uint16_t)((g_crtpTxHead + 1U) % CRTP_TX_QUEUE_SIZE);
  g_crtpTxCount = (uint16_t)(g_crtpTxCount - 1U);
  g_crtpRetryPending = false;
}

void crtpSetLink(CrtpLink *newLink) {
  if (g_currentLink != NULL && g_currentLink->setEnable != NULL) {
    g_currentLink->setEnable(false);
  }
  if (newLink == NULL || newLink == &g_nopLink) {
    g_currentLink = &g_nopLink;
  } else {
    g_currentLink = newLink;
    if (g_currentLink->setEnable != NULL) {
      g_currentLink->setEnable(true);
    }
  }
}

void crtpReset(void) {
  g_crtpTxHead = 0U;
  g_crtpTxTail = 0U;
  g_crtpTxCount = 0U;
  g_crtpRetryPending = false;
  if (g_currentLink != NULL && g_currentLink->reset != NULL) {
    g_currentLink->reset();
  }
}

bool crtpIsConnected(void) {
  if (g_currentLink != NULL && g_currentLink->isConnected != NULL) {
    return g_currentLink->isConnected();
  }
  return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
  return (uint32_t)(CRTP_TX_QUEUE_SIZE - g_crtpTxCount);
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
  if (port >= CRTP_NBR_OF_PORTS) {
    return;
  }
  g_crtpCallbacks[port] = callback;
}

void updateStats(void) {
  if ((g_systemTick - g_lastStatsTick) >= 500U) {
    uint32_t elapsed = g_systemTick - g_lastStatsTick;
    if (elapsed > 0U) {
      g_crtpTxRate = (float)g_crtpTxCounter * 1000.0f / (float)elapsed;
      g_crtpRxRate = (float)g_crtpRxCounter * 1000.0f / (float)elapsed;
    }
    g_crtpTxCounter = 0U;
    g_crtpRxCounter = 0U;
    g_lastStatsTick = g_systemTick;
  }
}

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
  if (decks == NULL || capacity == 0U) {
    return 0U;
  }
  static const uint8_t knownI2c[] = { 0x60U, 0x68U };
  static const uint64_t knownOneWire[] = { 0x1000000000000000ULL };
  uint8_t count = 0U;

  for (uint8_t i = 0U; i < (uint8_t)(sizeof(knownI2c) / sizeof(knownI2c[0])); ++i) {
    if (count >= capacity) {
      break;
    }
    bool duplicate = false;
    for (uint8_t j = 0U; j < count; ++j) {
      if (decks[j].foundByI2C && decks[j].i2cAddress == knownI2c[i]) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      decks[count].foundByI2C = true;
      decks[count].foundByOneWire = false;
      decks[count].i2cAddress = knownI2c[i];
      decks[count].oneWireRomId = 0U;
      count++;
    }
  }

  for (uint8_t i = 0U; i < (uint8_t)(sizeof(knownOneWire) / sizeof(knownOneWire[0])); ++i) {
    if (count >= capacity) {
      break;
    }
    bool duplicate = false;
    for (uint8_t j = 0U; j < count; ++j) {
      if (decks[j].foundByOneWire && decks[j].oneWireRomId == knownOneWire[i]) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      decks[count].foundByI2C = false;
      decks[count].foundByOneWire = true;
      decks[count].i2cAddress = 0U;
      decks[count].oneWireRomId = knownOneWire[i];
      count++;
    }
  }

  return count;
}
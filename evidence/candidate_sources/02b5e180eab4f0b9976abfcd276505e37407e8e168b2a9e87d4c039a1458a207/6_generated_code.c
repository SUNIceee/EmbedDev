#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#define DEG_TO_RAD_F 0.01745329252f
#define RAD_TO_DEG_F 57.2957795131f
#define PI_F 3.14159265358979323846f

uint32_t currentTick = 0;

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.0f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

PidObject pidRoll = {0};
PidObject pidPitch = {0};
PidObject pidYaw = {0};
PidObject pidRollRate = {0};
PidObject pidPitchRate = {0};
PidObject pidYawRate = {0};

bool thrustLocked = true;
bool commanderModeSet = false;

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0;

TestState healthTestState = testDone;
uint8_t motorPass = 0;
uint8_t batteryPass = 0;
float batterySag = 0.0f;

StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0}, acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};

static float s_attitudeUpdateDt = 0.002f;
static float s_desiredYaw = 0.0f;
static float s_positionControllerIntegral = 0.0f;

static bool s_commanderHasSetpoint = false;
static Setpoint s_activeSetpoint;
static int s_activePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t s_lastUpdateTick = 0;

static bool s_supervisorInitialized = false;
static bool s_isArmed = false;
static bool s_isCrashed = false;
static bool s_isFlying = false;
static bool s_isTumbled = false;
static bool s_isFreeFalling = false;
static bool s_autoArming = false;
static bool s_spinupStarted = false;
static uint32_t s_spinupStartTick = 0;
static uint32_t s_spinupTimeoutDurationMs = 0;
static uint32_t s_latestArmingTick = 0;
static uint32_t s_latestLandingTick = 0;
static uint32_t s_emergencyStopWatchdogLastTick = 0;
static SupervisorState s_lastSupervisorState = supervisorStateLocked;
static SensorData s_supervisorSensors;
static bool s_supervisorSensorsSet = false;
static uint32_t s_supervisorMotorRatios[4] = {0,0,0,0};
static uint32_t s_supervisorIdleThrust = 0;
static bool s_supervisorMotorRatiosSet = false;
static int32_t s_supervisorMotorRPMs[4] = {0,0,0,0};
static bool s_supervisorMotorRPMsSet = false;
static float s_crashDetectionGs = 0.0f;
static float s_freeFallThreshold = 0.0f;
static float s_acceptedTiltAccZ = 0.0f;
static float s_acceptedUpsideDownAccZ = 0.0f;
static uint32_t s_maxTiltTime = 0;
static uint32_t s_maxUpsideDownTime = 0;
static bool s_tumbleCheckEnabled = false;
static bool s_supervisorSafetyConfigured = false;
static bool s_tumbleTimerStarted = false;
static uint32_t s_tumbleStartTick = 0;
static bool s_seenFlightEvent = false;
static uint32_t s_recentFlightTick = 0;
static bool s_notRespondingTimerStarted = false;
static uint32_t s_notRespondingStartTick = 0;

static EstimatorMeasurement s_estimatorFifo[16];
static uint8_t s_estimatorHead = 0;
static uint8_t s_estimatorTail = 0;
static uint8_t s_estimatorCount = 0;
static bool s_estimatorHasGyro = false, s_estimatorHasAcc = false, s_estimatorHasBaro = false, s_estimatorHasTof = false;
static EstimatorMeasurement s_lastGyro, s_lastAcc, s_lastBaro, s_lastTof;

static bool s_stabilizerInitialized = false;
static uint32_t s_stabilizerStep = 0;
static bool s_hasPendingHighLevel = false;
static Setpoint s_pendingHighLevel;
static SensorData s_lastSensorData;

static bool s_propTestRequest = false;
static bool s_batTestRequest = false;
static bool s_healthTestActive = false;
static uint32_t s_motorTestCount = 0;
static float s_propAccBuffer[PROPTEST_NBR_OF_VARIANCE_VALUES];
static uint8_t s_propSampleCount = 0;
static uint8_t s_propMotorIndex = 0;
static float s_propNoiseVariance = 0.0f;
static uint32_t s_batTick = 0;
static float s_minLoadedVoltage = 0.0f;
static float s_idleVoltage = 4.2f;
static float batteryVoltage = 4.2f;
static uint8_t s_propFailureCount = 0;

static CrtpPacket s_txQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t s_txHead = 0, s_txTail = 0, s_txCount = 0;
static bool s_crtpInitialized = false;
static bool s_crtpQueueExists[CRTP_NBR_OF_PORTS] = {false};
static CrtpPacket s_rxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint8_t s_rxHead[CRTP_NBR_OF_PORTS] = {0};
static uint8_t s_rxTail[CRTP_NBR_OF_PORTS] = {0};
static uint8_t s_rxCount[CRTP_NBR_OF_PORTS] = {0};
static CrtpPortCallback s_portCallbacks[CRTP_NBR_OF_PORTS] = {0};
static bool s_crtpErrorState = false;
static bool s_txRetryPending = false;
static uint32_t s_txRetryTick = 0;
static uint32_t s_rxPacketCount = 0;
static uint32_t s_txPacketCount = 0;
static uint32_t s_lastStatsTick = 0;
static uint32_t crtpRxRate = 0;
static uint32_t crtpTxRate = 0;

static bool nopSend(CrtpPacket *packet) { (void)packet; return false; }
static bool nopReceive(CrtpPacket *packet) { (void)packet; return false; }
static void nopSetEnable(bool enable) { (void)enable; }
static void nopReset(void) {}
static CrtpLink s_nopLink = { nopSend, nopReceive, NULL, nopSetEnable, nopReset };
static CrtpLink *s_activeLink = &s_nopLink;

static bool s_rateWaitStarted = false;
static uint32_t s_rateWaitStart = 0;
static bool s_rateSupervisorError = false;
static bool sensorsPaused = false;

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void quatToEuler(float w, float x, float y, float z,
                        float *roll_deg, float *pitch_deg, float *yaw_deg) {
  float sinPitch = 2.0f * (w * y - x * z);
  sinPitch = clampf(sinPitch, -1.0f, 1.0f);
  *roll_deg = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD_TO_DEG_F;
  *pitch_deg = asinf(sinPitch) * RAD_TO_DEG_F;
  *yaw_deg = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD_TO_DEG_F;
}

static void syncSensfusionLog(void) {
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

static float pidUpdate(PidObject *pid, float error, float dt, bool reset) {
  if (!pid || !pid->initialized) return 0.0f;
  if (dt <= 0.0f) dt = 0.002f;
  if (reset) {
    pid->integral = 0.0f;
    pid->prevError = error;
  }
  pid->integral += error * dt;
  float derivative = (error - pid->prevError) / dt;
  float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
  pid->output = output;
  pid->prevError = error;
  return output;
}

static void pidResetToAngle(PidObject *pid, float angle) {
  if (!pid) return;
  pid->integral = 0.0f;
  pid->prevError = angle;
  pid->output = angle;
}

static void pidClearRate(PidObject *pid) {
  if (!pid) return;
  pid->integral = 0.0f;
  pid->prevError = 0.0f;
  pid->output = 0.0f;
}

static void pidInitIfNeeded(PidObject *pid) {
  if (!pid || pid->initialized) return;
  pid->integral = 0.0f;
  pid->prevError = 0.0f;
  pid->output = 0.0f;
  pid->initialized = true;
}

static bool supervisorAllowedArmingState(SupervisorState state) {
  return state == supervisorStateArming || state == supervisorStateReadyToFly ||
         state == supervisorStateFlying || state == supervisorStateWarningLevelOut ||
         state == supervisorStateLanded;
}

static bool allFourMotorsBelowThreshold(const int32_t motorRPMs[4], int32_t threshold) {
  return motorRPMs[0] < threshold && motorRPMs[1] < threshold &&
         motorRPMs[2] < threshold && motorRPMs[3] < threshold;
}

static uint16_t motorForceToPwm(float force) {
  if (force <= 0.0f) return 0;
  float ratio = force / CRAZYFLIE_MAX_MOTOR_FORCE_N;
  ratio = clampf(ratio, 0.0f, 1.0f);
  return (uint16_t)(ratio * 65535.0f);
}

static uint32_t quatcompress(float w, float x, float y, float z) {
  uint32_t packed = 0;
  w = clampf(w, -1.0f, 1.0f);
  x = clampf(x, -1.0f, 1.0f);
  y = clampf(y, -1.0f, 1.0f);
  z = clampf(z, -1.0f, 1.0f);
  packed |= ((uint32_t)((w + 1.0f) * 32767.0f) & 0xFFFFU) << 0;
  packed |= ((uint32_t)((x + 1.0f) * 32767.0f) & 0xFFFFU) << 8;
  packed |= ((uint32_t)((y + 1.0f) * 32767.0f) & 0xFFFFU) << 16;
  packed |= ((uint32_t)((z + 1.0f) * 32767.0f) & 0xFFFFU) << 24;
  return packed;
}

static void commanderGetSetpoint(Setpoint *out) {
  if (out) *out = s_activeSetpoint;
}

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
  if (sensfusion6IsInit) return;
  qw = 1.0f;
  qx = 0.0f;
  qy = 0.0f;
  qz = 0.0f;
  integralFBx = 0.0f;
  integralFBy = 0.0f;
  integralFBz = 0.0f;
  baseZacc = 0.0f;
  gravityX = 0.0f;
  gravityY = 0.0f;
  gravityZ = 1.0f;
  sensfusion6IsCalibrated = false;
  sensfusion6IsInit = true;
  syncSensfusionLog();
}

bool sensfusion6Test(void) {
  return sensfusion6IsInit;
}

float invSqrt(float x) {
  if (x <= 0.0f) return 0.0f;
  union { float f; uint32_t i; } conv;
  conv.f = x;
  conv.i = 0x5f3759dfU - (conv.i >> 1);
  float y = conv.f;
  y = y * (1.5f - (x * 0.5f * y * y));
  return y;
}

void estimatedGravityDirection(float w, float x, float y, float z,
                               float *gravX, float *gravY, float *gravZ) {
  if (!gravX || !gravY || !gravZ) return;
  *gravX = 2.0f * (x * z - w * y);
  *gravY = 2.0f * (y * z + w * x);
  *gravZ = 1.0f - 2.0f * (x * x + y * y);
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt) {
  if (!sensfusion6IsInit) sensfusion6Init();
  if (dt <= 0.0f) return;

  float gxRad = gx * DEG_TO_RAD_F;
  float gyRad = gy * DEG_TO_RAD_F;
  float gzRad = gz * DEG_TO_RAD_F;
  bool accValid = !(ax == 0.0f && ay == 0.0f && az == 0.0f);

  if (twoKi == 0.0f) {
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
  }

  float oldW = qw, oldX = qx, oldY = qy, oldZ = qz;

  if (beta > 0.0f) {
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
    if (accValid) {
      float norm = invSqrt(ax * ax + ay * ay + az * az);
      ax *= norm; ay *= norm; az *= norm;
      float _2q0 = 2.0f * oldW, _2q1 = 2.0f * oldX, _2q2 = 2.0f * oldY, _2q3 = 2.0f * oldZ;
      float _4q0 = 4.0f * oldW, _4q1 = 4.0f * oldX, _4q2 = 4.0f * oldY, _4q3 = 4.0f * oldZ;
      float _8q1 = 8.0f * oldX, _8q2 = 8.0f * oldY;
      s0 = _4q0 * oldX * oldX + _2q2 * ax + _4q0 * oldY * oldY - _2q1 * ay;
      s1 = _4q1 * oldZ * oldZ - _2q3 * ax + 4.0f * oldW * oldW * oldX - _2q0 * ay
           - _4q1 + _8q1 * oldX * oldX + _8q2 * oldX * oldY + _4q1 * oldZ * oldZ;
      s2 = 4.0f * oldW * oldW * oldY + _2q0 * ax - _4q2 * oldY + _2q1 * ay
           - _4q2 * oldZ * oldZ + _8q1 * oldX * oldY + _8q2 * oldY * oldY;
      s3 = 4.0f * oldX * oldX * oldZ - _2q0 * ay + 4.0f * oldY * oldY * oldZ - _2q1 * ax;
    }
    float qDot0 = 0.5f * (-oldX * gxRad - oldY * gyRad - oldZ * gzRad) + beta * s0;
    float qDot1 = 0.5f * ( oldW * gxRad + oldY * gzRad - oldZ * gyRad) + beta * s1;
    float qDot2 = 0.5f * ( oldW * gyRad - oldX * gzRad + oldZ * gxRad) + beta * s2;
    float qDot3 = 0.5f * ( oldW * gzRad + oldX * gyRad - oldY * gxRad) + beta * s3;
    qw = oldW + qDot0 * dt;
    qx = oldX + qDot1 * dt;
    qy = oldY + qDot2 * dt;
    qz = oldZ + qDot3 * dt;
  } else {
    float gxCorrected = gxRad;
    float gyCorrected = gyRad;
    float gzCorrected = gzRad;
    if (accValid) {
      float norm = invSqrt(ax * ax + ay * ay + az * az);
      float axn = ax * norm, ayn = ay * norm, azn = az * norm;
      estimatedGravityDirection(oldW, oldX, oldY, oldZ, &gravityX, &gravityY, &gravityZ);
      float ex = (ayn * gravityZ - azn * gravityY);
      float ey = (azn * gravityX - axn * gravityZ);
      float ez = (axn * gravityY - ayn * gravityX);
      if (twoKi > 0.0f) {
        integralFBx += twoKi * ex * dt;
        integralFBy += twoKi * ey * dt;
        integralFBz += twoKi * ez * dt;
      }
      gxCorrected += twoKp * ex + integralFBx;
      gyCorrected += twoKp * ey + integralFBy;
      gzCorrected += twoKp * ez + integralFBz;
    }
    qw = oldW + 0.5f * (-oldX * gxCorrected - oldY * gyCorrected - oldZ * gzCorrected) * dt;
    qx = oldX + 0.5f * ( oldW * gxCorrected + oldY * gzCorrected - oldZ * gyCorrected) * dt;
    qy = oldY + 0.5f * ( oldW * gyCorrected - oldX * gzCorrected + oldZ * gxCorrected) * dt;
    qz = oldZ + 0.5f * ( oldW * gzCorrected + oldX * gyCorrected - oldY * gxCorrected) * dt;
  }

  float normSq = qw * qw + qx * qx + qy * qy + qz * qz;
  if (normSq > 1e-12f) {
    float recip = invSqrt(normSq);
    qw *= recip; qx *= recip; qy *= recip; qz *= recip;
  }

  syncSensfusionLog();

  if (accValid && !sensfusion6IsCalibrated) {
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
    baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
    sensfusion6IsCalibrated = true;
    syncSensfusionLog();
  }
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
  if (!roll_deg || !pitch_deg || !yaw_deg) return;
  quatToEuler(qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg);
}

void sensfusion6GetQuaternion(float *w, float *x, float *y, float *z) {
  if (!w || !x || !y || !z) return;
  *w = qw; *x = qx; *y = qy; *z = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out) {
  if (!out) return;
  int32_t r = (int32_t)roll / 2;
  int32_t p = (int32_t)pitch / 2;
  out->m1 = (int32_t)thrust - r + p + (int32_t)yaw;
  out->m2 = (int32_t)thrust - r - p - (int32_t)yaw;
  out->m3 = (int32_t)thrust + r - p + (int32_t)yaw;
  out->m4 = (int32_t)thrust + r + p - (int32_t)yaw;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4]) {
  if (!motorForces) return;
  float thrustPart = 0.25f * thrustSi;
  float arm = 0.707106781f * armLength;
  float rollPart = armLength == 0.0f ? 0.0f : (0.25f / arm) * torqueX;
  float pitchPart = armLength == 0.0f ? 0.0f : (0.25f / arm) * torqueY;
  float yawPart = thrustToTorque == 0.0f ? 0.0f : (0.25f / thrustToTorque) * torqueZ;

  float f0 = thrustPart - rollPart + pitchPart + yawPart;
  float f1 = thrustPart - rollPart - pitchPart - yawPart;
  float f2 = thrustPart + rollPart - pitchPart + yawPart;
  float f3 = thrustPart + rollPart + pitchPart - yawPart;

  if (f0 < 0.0f) f0 = 0.0f;
  if (f1 < 0.0f) f1 = 0.0f;
  if (f2 < 0.0f) f2 = 0.0f;
  if (f3 < 0.0f) f3 = 0.0f;

  motorForces[0] = f0;
  motorForces[1] = f1;
  motorForces[2] = f2;
  motorForces[3] = f3;
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4]) {
  if (!normalizedForces || !motorPWMs) return;
  for (uint8_t i = 0; i < 4; i++) {
    float f = normalizedForces[i];
    f = clampf(f, 0.0f, 1.0f);
    motorPWMs[i] = (uint16_t)(f * 65535.0f);
  }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower) {
  if (!control || !motorPower) return;
  switch (control->controlMode) {
    case controlModeLegacy: {
      MotorPower legacy = {0};
      powerDistributionLegacy(control->thrust, control->roll, control->pitch, control->yaw, &legacy);
      motorPower->m1 = legacy.m1;
      motorPower->m2 = legacy.m2;
      motorPower->m3 = legacy.m3;
      motorPower->m4 = legacy.m4;
      break;
    }
    case controlModeForceTorque: {
      float forces[4] = {0,0,0,0};
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
      uint16_t pwms[4] = {0,0,0,0};
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
  PowerCapResult result = { false, 0 };
  if (!motors) return result;
  int32_t maxMotor = motors[0];
  for (uint8_t i = 1; i < 4; i++) {
    if (motors[i] > maxMotor) maxMotor = motors[i];
  }
  if (maxMotor > maxAllowedThrust) {
    int32_t reduction = maxMotor - maxAllowedThrust;
    for (uint8_t i = 0; i < 4; i++) {
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
  float compensated = ((float)motorThrust * nominalVoltage) / actualVoltage;
  if (compensated < 0.0f) compensated = 0.0f;
  if (compensated > 65535.0f) compensated = 65535.0f;
  return (uint16_t)(compensated + 0.5f);
}

void attitudeControllerInit(float updateDt) {
  if (updateDt > 0.0f) s_attitudeUpdateDt = updateDt;
  pidInitIfNeeded(&pidRoll);
  pidInitIfNeeded(&pidPitch);
  pidInitIfNeeded(&pidYaw);
  pidInitIfNeeded(&pidRollRate);
  pidInitIfNeeded(&pidPitchRate);
  pidInitIfNeeded(&pidYawRate);
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
  pidRollRate.output = pidUpdate(&pidRollRate, rollDesired - rollActual, s_attitudeUpdateDt, false);
  pidPitchRate.output = pidUpdate(&pidPitchRate, pitchDesired - pitchActual, s_attitudeUpdateDt, false);
  pidYawRate.output = pidUpdate(&pidYawRate, yawDesired - yawActual, s_attitudeUpdateDt, false);
  pidRollRate.output = (float)saturateSignedInt16((int32_t)pidRollRate.output);
  pidPitchRate.output = (float)saturateSignedInt16((int32_t)pidPitchRate.output);
  pidYawRate.output = (float)saturateSignedInt16((int32_t)pidYawRate.output);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
  pidRoll.output = pidUpdate(&pidRoll, rollDesired - rollActual, s_attitudeUpdateDt, false);
  pidPitch.output = pidUpdate(&pidPitch, pitchDesired - pitchActual, s_attitudeUpdateDt, false);
  pidYaw.output = pidUpdate(&pidYaw, yawDesired - yawActual, s_attitudeUpdateDt, true);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual) {
  pidResetToAngle(&pidRoll, rollActual);
  pidResetToAngle(&pidPitch, pitchActual);
  pidResetToAngle(&pidYaw, yawActual);
  pidClearRate(&pidRollRate);
  pidClearRate(&pidPitchRate);
  pidClearRate(&pidYawRate);
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
  pidResetToAngle(&pidRoll, rollActual);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
  pidResetToAngle(&pidPitch, pitchActual);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw) {
  if (!roll || !pitch || !yaw) return;
  *roll = saturateSignedInt16((int32_t)pidRollRate.output);
  *pitch = saturateSignedInt16((int32_t)pidPitchRate.output);
  *yaw = saturateSignedInt16((int32_t)pidYawRate.output);
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
  if (!setpoint || !state) return 0;
  float out = 32767.0f;
  if (setpoint->mode.z == modeAbs) {
    out = 32767.0f + (setpoint->position.z - state->position.z) * 1000.0f;
  } else if (setpoint->mode.z == modeVelocity) {
    out = 32767.0f + (setpoint->velocity.z - state->velocity.z) * 1000.0f;
  }
  s_positionControllerIntegral = s_positionControllerIntegral * 0.9f + (setpoint->velocity.z - state->velocity.z) * 0.002f;
  out += s_positionControllerIntegral;
  if (out < 0.0f) out = 0.0f;
  if (out > 65535.0f) out = 65535.0f;
  return (uint16_t)(out + 0.5f);
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
  if (!sensors || !setpoint || !state || !control) return;
  uint16_t thrust = setpoint->thrust;
  if (setpoint->mode.z != modeDisable) {
    thrust = positionControllerUpdate(setpoint, state);
  }

  if (thrust == 0) {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    control->thrust = 0;
    control->controlMode = controlModeLegacy;
    attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
    s_positionControllerIntegral = 0.0f;
    s_desiredYaw = state->attitude.yaw;
    return;
  }

  float desiredYaw = s_desiredYaw;
  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
  } else if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = setpoint->attitude.yaw;
  } else if (setpoint->mode.quat == modeAbs) {
    float r, p, y;
    quatToEuler(setpoint->attitudeQuaternion.w, setpoint->attitudeQuaternion.x,
                setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, &r, &p, &y);
    (void)r; (void)p;
    desiredYaw = y;
  }

  if (yawMaxDelta != 0.0f) {
    float diff = capAngle(desiredYaw - state->attitude.yaw);
    diff = clampf(diff, -yawMaxDelta, yawMaxDelta);
    desiredYaw = state->attitude.yaw + diff;
  }
  s_desiredYaw = desiredYaw;

  float rollDesiredRate = 0.0f;
  float pitchDesiredRate = 0.0f;
  if (setpoint->mode.roll == modeVelocity) {
    rollDesiredRate = setpoint->attitudeRate.roll;
    attitudeControllerResetRollAttitudePID(state->attitude.roll);
  } else if (setpoint->mode.roll == modeAbs) {
    rollDesiredRate = pidUpdate(&pidRoll, setpoint->attitude.roll - state->attitude.roll, attitudeUpdateDt, false);
  }
  if (setpoint->mode.pitch == modeVelocity) {
    pitchDesiredRate = setpoint->attitudeRate.pitch;
    attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
  } else if (setpoint->mode.pitch == modeAbs) {
    pitchDesiredRate = pidUpdate(&pidPitch, setpoint->attitude.pitch - state->attitude.pitch, attitudeUpdateDt, false);
  }

  float yawDesiredRate = pidUpdate(&pidYaw, desiredYaw - state->attitude.yaw, attitudeUpdateDt, true);

  float rollActual = sensors->gyro.x;
  float pitchActual = -sensors->gyro.y;
  float yawActual = sensors->gyro.z;

  pidRollRate.output = (float)saturateSignedInt16((int32_t)pidUpdate(&pidRollRate, rollDesiredRate - rollActual, attitudeUpdateDt, false));
  pidPitchRate.output = (float)saturateSignedInt16((int32_t)pidUpdate(&pidPitchRate, pitchDesiredRate - pitchActual, attitudeUpdateDt, false));
  pidYawRate.output = (float)saturateSignedInt16((int32_t)pidUpdate(&pidYawRate, yawDesiredRate - yawActual, attitudeUpdateDt, false));

  control->controlMode = controlModeLegacy;
  control->roll = saturateSignedInt16((int32_t)pidRollRate.output);
  control->pitch = saturateSignedInt16((int32_t)pidPitchRate.output);
  int16_t yawOut = saturateSignedInt16((int32_t)pidYawRate.output);
  control->yaw = -yawOut;
  control->thrust = thrust;
}

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
  if (!rollPrime || !pitchPrime) return;
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
    YawMode yawMode) {
  if (!values || !setpoint) return;
  memset(setpoint, 0, sizeof(*setpoint));

  uint16_t rawThrust = values->thrust;
  if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
    thrustLocked = true;
    if (rawThrust == 0) thrustLocked = false;
  }

  if (posSetMode && rawThrust != 0) {
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;
    setpoint->position.x = -values->pitch;
    setpoint->position.y = values->roll;
    setpoint->position.z = (float)rawThrust / 1000.0f;
    setpoint->attitude.yaw = values->yaw;
    setpoint->thrust = 0;
    return;
  }

  if (posHoldMode) {
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.z = modeDisable;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->velocity.x = values->pitch / 30.0f;
    setpoint->velocity.y = values->roll / 30.0f;
    setpoint->attitude.roll = 0.0f;
    setpoint->attitude.pitch = 0.0f;
    if (stabilizationModeYaw == RATE) {
      setpoint->mode.yaw = modeVelocity;
      setpoint->attitudeRate.yaw = -values->yaw;
    } else {
      setpoint->mode.yaw = modeAbs;
      setpoint->attitude.yaw = values->yaw;
    }
    if (altHoldMode) {
      setpoint->mode.z = modeVelocity;
      setpoint->thrust = 0;
      setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
      if (!commanderModeSet) { commanderModeSet = true; s_positionControllerIntegral = 0.0f; }
    } else {
      setpoint->mode.z = modeDisable;
      if (commanderModeSet) commanderModeSet = false;
      if (thrustLocked || rawThrust < MIN_THRUST) setpoint->thrust = 0;
      else setpoint->thrust = rawThrust > MAX_THRUST ? MAX_THRUST : rawThrust;
    }
    return;
  }

  if (altHoldMode) {
    setpoint->mode.z = modeVelocity;
    setpoint->thrust = 0;
    setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
    if (!commanderModeSet) {
      commanderModeSet = true;
      s_positionControllerIntegral = 0.0f;
    }
  } else {
    setpoint->mode.z = modeDisable;
    if (commanderModeSet) commanderModeSet = false;
    if (thrustLocked || rawThrust < MIN_THRUST) setpoint->thrust = 0;
    else setpoint->thrust = rawThrust > MAX_THRUST ? MAX_THRUST : rawThrust;
  }

  float cmdRoll = values->roll;
  float cmdPitch = values->pitch;
  if (yawMode == PLUSMODE) {
    rotateYaw(cmdRoll, cmdPitch, 45.0f, &cmdRoll, &cmdPitch);
  } else if (yawMode == CAREFREE) {
    cmdRoll = 0.0f;
    cmdPitch = 0.0f;
  }

  if (stabilizationModeRoll == RATE) {
    setpoint->mode.roll = modeVelocity;
    setpoint->attitudeRate.roll = cmdRoll;
  } else {
    setpoint->mode.roll = modeAbs;
    setpoint->attitude.roll = cmdRoll;
  }
  if (stabilizationModePitch == RATE) {
    setpoint->mode.pitch = modeVelocity;
    setpoint->attitudeRate.pitch = cmdPitch;
  } else {
    setpoint->mode.pitch = modeAbs;
    setpoint->attitude.pitch = cmdPitch;
  }
  if (stabilizationModeYaw == RATE) {
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitudeRate.yaw = -values->yaw;
  } else {
    setpoint->mode.yaw = modeAbs;
    setpoint->attitude.yaw = values->yaw;
  }
}

void supervisorInit(void) {
  if (s_supervisorInitialized) return;
  supervisorState = supervisorStateLocked;
  supervisorConditionBits = 0;
  s_isArmed = false;
  s_isCrashed = false;
  s_isFlying = false;
  s_isTumbled = false;
  s_isFreeFalling = false;
  s_spinupStarted = false;
  s_spinupStartTick = 0;
  s_latestArmingTick = 0;
  s_latestLandingTick = 0;
  s_emergencyStopWatchdogLastTick = 0;
  s_lastSupervisorState = supervisorStateLocked;
  s_supervisorInitialized = true;
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

bool supervisorIsArmed(void) { return s_isArmed; }
bool supervisorIsCrashed(void) { return s_isCrashed; }

bool supervisorAreMotorsAllowedToRun(void) {
  return supervisorAllowedArmingState(supervisorState);
}

bool supervisorRequestArming(bool doArm) {
  if (doArm) {
    if (!supervisorCanArm()) return false;
    if (!s_isArmed) {
      s_isArmed = true;
      supervisorState = supervisorStateArming;
      s_spinupStarted = true;
      s_spinupStartTick = currentTick;
      s_latestArmingTick = currentTick;
      supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    }
    return true;
  }
  s_isArmed = false;
  supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
  if (supervisorState == supervisorStateArming) {
    supervisorState = supervisorStatePreFlChecksPassed;
  }
  return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
  if (s_isTumbled) return false;
  if (!doRecovery) {
    s_isCrashed = true;
    supervisorState = supervisorStateCrashed;
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    return true;
  }
  s_isCrashed = false;
  supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
  if (supervisorState == supervisorStateCrashed) {
    supervisorState = supervisorStatePreFlChecksPassed;
  }
  return true;
}

bool checkEmergencyStopWatchdog(uint32_t tick, uint32_t lastNotificationTick) {
  if (lastNotificationTick == 0) return true;
  return (tick - lastNotificationTick) < DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t tick,
                                  uint32_t preflightTimeoutDuration) {
  if (state != supervisorStateReadyToFly || latestArmingTick == 0) return false;
  return (tick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t tick,
                                uint32_t landingTimeoutDuration) {
  if (latestLandingTick == 0) return false;
  return (tick - latestLandingTick) >= landingTimeoutDuration;
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

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t tick) {
  if (!motorRatios) return s_isFlying;
  bool anyAbove = false;
  for (uint8_t i = 0; i < 4; i++) {
    if (motorRatios[i] > idleThrust) { anyAbove = true; break; }
  }
  if (anyAbove) {
    s_seenFlightEvent = true;
    s_recentFlightTick = tick;
  }
  if (s_recentFlightTick == 0 && !s_seenFlightEvent) {
    s_isFlying = false;
    return false;
  }
  s_isFlying = (tick - s_recentFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
  return s_isFlying;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t tick,
                    bool *isFreeFalling) {
  if (!isFreeFalling) return s_isTumbled;
  s_isFreeFalling = false;
  *isFreeFalling = false;
  if (!tumbleCheckEnabled) {
    return false;
  }

  if (fabsf(accX) < freeFallThreshold && fabsf(accY) < freeFallThreshold &&
      fabsf(accZ) < freeFallThreshold) {
    s_isFreeFalling = true;
    *isFreeFalling = true;
    s_tumbleTimerStarted = false;
    s_tumbleStartTick = 0;
    return false;
  }

  if (crashDetectionGs > 0.0f) {
    float accNorm = sqrtf(accX * accX + accY * accY + accZ * accZ);
    if (fabsf(accNorm - 1.0f) > crashDetectionGs) {
      s_isCrashed = true;
      supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
      if (supervisorState != supervisorStateCrashed) {
        supervisorState = supervisorStateCrashed;
      }
    }
  }

  bool tilted = accZ < acceptedTiltAccZ;
  bool upsideDown = accZ < acceptedUpsideDownAccZ;
  if (tilted) {
    if (!s_tumbleTimerStarted) {
      s_tumbleTimerStarted = true;
      s_tumbleStartTick = tick;
    }
    uint32_t timeout = upsideDown ? maxUpsideDownTime : maxTiltTime;
    if ((tick - s_tumbleStartTick) >= timeout) {
      s_isTumbled = true;
    }
  } else {
    s_tumbleTimerStarted = false;
    s_tumbleStartTick = 0;
    s_isTumbled = false;
  }

  if (s_isTumbled) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
  else supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
  if (s_isFreeFalling) supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
  else supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;
  return s_isTumbled;
}

void supervisorSetSensorData(const SensorData *sensors) {
  if (!sensors) return;
  s_supervisorSensors = *sensors;
  s_supervisorSensorsSet = true;
  float ax = sensors->acc.x, ay = sensors->acc.y, az = sensors->acc.z;
  supervisorLog.accNorm = sqrtf(ax * ax + ay * ay + az * az);
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
  if (!motorRatios) return;
  memcpy(s_supervisorMotorRatios, motorRatios, sizeof(s_supervisorMotorRatios));
  s_supervisorIdleThrust = idleThrust;
  s_supervisorMotorRatiosSet = true;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
  if (!motorRPMs) return;
  memcpy(s_supervisorMotorRPMs, motorRPMs, sizeof(s_supervisorMotorRPMs));
  s_supervisorMotorRPMsSet = true;
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
  s_supervisorSafetyConfigured = true;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
  s_autoArming = autoArming;
  s_spinupTimeoutDurationMs = spinupTimeoutDurationMs;
}

uint16_t supervisorGetInfoBitfield(void) {
  uint16_t info = 0;
  if (supervisorCanArm()) info |= (1U << 0);
  if (s_isArmed) info |= (1U << 1);
  if (s_autoArming) info |= (1U << 2);
  if (supervisorCanFly()) info |= (1U << 3);
  if (s_isFlying) info |= (1U << 4);
  if (s_isTumbled) info |= (1U << 5);
  if (supervisorState == supervisorStateLocked) info |= (1U << 6);
  if (s_isCrashed) info |= (1U << 7);
  supervisorLog.info = info;
  return info;
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax) {
  if (!motorRPMs) return false;
  for (uint8_t i = 0; i < 4; i++) {
    if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
  }
  return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t tick) {
  if (!motorRPMs) return false;
  if (!canFly) {
    s_notRespondingTimerStarted = false;
    s_notRespondingStartTick = 0;
    supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return false;
  }
  if (allFourMotorsBelowThreshold(motorRPMs, rpmThreshold)) {
    if (!s_notRespondingTimerStarted) {
      s_notRespondingTimerStarted = true;
      s_notRespondingStartTick = tick;
    }
    if ((tick - s_notRespondingStartTick) >= rpmCheckDurationMs) {
      supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
      return true;
    }
    return false;
  }
  s_notRespondingTimerStarted = false;
  s_notRespondingStartTick = 0;
  return false;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t conditionBits,
                                SupervisorState state) {
  if (!setpoint) return;
  if (conditionBits & (SUPERVISOR_CB_EMERGENCY_STOP | SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT |
                       SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_FREE_FALL |
                       SUPERVISOR_CB_MOTORS_NOT_RESPONDING)) {
    memset(setpoint, 0, sizeof(*setpoint));
    return;
  }
  switch (state) {
    case supervisorStateWarningLevelOut:
      setpoint->mode.x = modeDisable;
      setpoint->mode.y = modeDisable;
      setpoint->mode.roll = modeAbs;
      setpoint->mode.pitch = modeAbs;
      setpoint->attitude.roll = 0.0f;
      setpoint->attitude.pitch = 0.0f;
      setpoint->mode.yaw = modeVelocity;
      setpoint->attitudeRate.yaw = 0.0f;
      break;
    case supervisorStateArming:
    case supervisorStateReadyToFly:
    case supervisorStateFlying:
    case supervisorStateLanded:
      break;
    default:
      memset(setpoint, 0, sizeof(*setpoint));
      break;
  }
}

void supervisorUpdate(uint32_t stabilizerStep) {
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

  SupervisorState prevState = supervisorState;

  if (supervisorState == supervisorStatePreFlChecksPassed &&
      prevState != supervisorStatePreFlChecksPassed && s_autoArming) {
    supervisorRequestArming(true);
  }

  uint32_t age = commanderGetInactivityTime();
  supervisorConditionBits &= ~(SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT);
  if (age >= COMMANDER_WDT_TIMEOUT_SHUTDOWN) supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
  else if (age >= COMMANDER_WDT_TIMEOUT_STABILIZE) supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;

  if (supervisorState == supervisorStateArming && s_isArmed) {
    if (!s_spinupStarted) {
      s_spinupStarted = true;
      s_spinupStartTick = currentTick;
    } else if ((currentTick - s_spinupStartTick) >= s_spinupTimeoutDurationMs) {
      supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
    }
  }

  if (s_supervisorMotorRatiosSet) {
    isFlyingCheck(s_supervisorMotorRatios, s_supervisorIdleThrust, currentTick);
  }

  if (s_supervisorSensorsSet && s_supervisorSafetyConfigured) {
    bool freeFallOut = false;
    isTumbledCheck(s_supervisorSensors.acc.x, s_supervisorSensors.acc.y, s_supervisorSensors.acc.z,
                   s_crashDetectionGs, s_freeFallThreshold, s_acceptedTiltAccZ,
                   s_acceptedUpsideDownAccZ, s_maxTiltTime, s_maxUpsideDownTime,
                   s_tumbleCheckEnabled, currentTick, &freeFallOut);
    s_isFreeFalling = freeFallOut;
  }

  if (s_isFreeFalling && (supervisorState == supervisorStateFlying ||
                          supervisorState == supervisorStateWarningLevelOut)) {
    supervisorState = supervisorStateExceptFreeFall;
  }
  if (s_isCrashed && supervisorState != supervisorStateCrashed) {
    supervisorState = supervisorStateCrashed;
  }
  if (s_isFlying && (supervisorState == supervisorStateReadyToFly ||
                     supervisorState == supervisorStateLanded)) {
    supervisorState = supervisorStateFlying;
  }

  bool prevAllowed = supervisorAllowedArmingState(prevState);
  bool newAllowed = supervisorAllowedArmingState(supervisorState);
  if (prevAllowed && !newAllowed) {
    s_isArmed = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
  }
  if (prevState == supervisorStateArming && supervisorState != supervisorStateArming) {
    s_spinupStarted = false;
    s_spinupStartTick = 0;
    supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
  }

  if (s_isArmed) supervisorConditionBits |= SUPERVISOR_CB_ARMED;
  else supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
  if (s_isFlying) supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
  else supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
  if (s_isTumbled) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
  else supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
  if (s_isCrashed) supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
  else supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
  if (s_isFreeFalling) supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
  else supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

  s_lastSupervisorState = supervisorState;
}

bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
  if (!measurement || s_estimatorCount >= 16) return false;
  s_estimatorFifo[s_estimatorTail] = *measurement;
  s_estimatorTail = (uint8_t)((s_estimatorTail + 1) % 16);
  s_estimatorCount++;
  return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
  if (!measurement || s_estimatorCount == 0) return false;
  *measurement = s_estimatorFifo[s_estimatorHead];
  s_estimatorHead = (uint8_t)((s_estimatorHead + 1) % 16);
  s_estimatorCount--;
  return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
  EstimatorMeasurement m;
  while (estimatorDequeue(&m)) {
    switch (m.type) {
      case MeasurementTypeGyroscope: s_lastGyro = m; s_estimatorHasGyro = true; break;
      case MeasurementTypeAcceleration: s_lastAcc = m; s_estimatorHasAcc = true; break;
      case MeasurementTypeBarometer: s_lastBaro = m; s_estimatorHasBaro = true; break;
      case MeasurementTypeTOF: s_lastTof = m; s_estimatorHasTof = true; break;
      default: break;
    }
  }

  if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
    float gx = s_estimatorHasGyro ? s_lastGyro.data[0] : 0.0f;
    float gy = s_estimatorHasGyro ? s_lastGyro.data[1] : 0.0f;
    float gz = s_estimatorHasGyro ? s_lastGyro.data[2] : 0.0f;
    float ax = s_estimatorHasAcc ? s_lastAcc.data[0] : 0.0f;
    float ay = s_estimatorHasAcc ? s_lastAcc.data[1] : 0.0f;
    float az = s_estimatorHasAcc ? s_lastAcc.data[2] : 0.0f;
    sensfusion6UpdateQ(gx, gy, gz, ax, ay, az, 1.0f / (float)SENSFUSION_RATE_HZ);
    sensfusion6GetEulerRPY(&stateEstimate.roll, &stateEstimate.pitch, &stateEstimate.yaw);
    sensfusion6GetQuaternion(&stateEstimate.qw, &stateEstimate.qx, &stateEstimate.qy, &stateEstimate.qz);
    syncSensfusionLog();
  }

  if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
    if (s_estimatorHasBaro) baro.pressure = s_lastBaro.data[0];
    if (s_estimatorHasTof) baro.asl = s_lastTof.data[0];
  }
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
  if (!setpoint) return false;
  if (priority == COMMANDER_PRIORITY_DISABLE || priority >= s_activePriority) {
    s_activeSetpoint = *setpoint;
    s_activePriority = priority;
    s_lastUpdateTick = currentTick;
    s_commanderHasSetpoint = true;
    if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
      // Stop high-level trajectory state here; no extra public trajectory state exists.
    }
    return true;
  }
  return false;
}

void commanderRelaxPriority(void) {
  s_activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
  if (!s_commanderHasSetpoint) return 0;
  return currentTick - s_lastUpdateTick;
}

int commanderGetActivePriority(void) {
  return s_activePriority;
}

void stabilizerInit(void) {
  if (s_stabilizerInitialized) return;
  // sensorsInit
  // stateEstimatorInit
  sensfusion6Init();
  memset(&s_estimatorFifo, 0, sizeof(s_estimatorFifo));
  s_estimatorHead = s_estimatorTail = s_estimatorCount = 0;
  // controllerInit
  attitudeControllerInit(1.0f / (float)ATTITUDE_RATE_HZ);
  // powerDistributionInit
  // motorsInit
  motor.m1req = motor.m2req = motor.m3req = motor.m4req = 0;
  // collisionAvoidanceInit
  s_stabilizerStep = 0;
  s_hasPendingHighLevel = false;
  memset(&s_pendingHighLevel, 0, sizeof(s_pendingHighLevel));
  s_stabilizerInitialized = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
  if (!setpoint) return false;
  s_pendingHighLevel = *setpoint;
  s_hasPendingHighLevel = true;
  return true;
}

void stabilizerTask(void) {
  if (!s_stabilizerInitialized) return;
  s_stabilizerStep++;

  if (healthShallWeRunTest()) {
    healthRunTests(&s_lastSensorData);
    return;
  }

  if (s_hasPendingHighLevel) {
    commanderSetSetpoint(&s_pendingHighLevel, COMMANDER_PRIORITY_HIGHLEVEL);
    s_hasPendingHighLevel = false;
  }

  SensorData sensors = s_lastSensorData;
  gyro = sensors.gyro;
  acc = sensors.acc;
  baro.pressure = sensors.baroPressure;
  baro.temp = sensors.baroTemperature;
  baro.asl = sensors.baroAsl;

  estimatorComplementary(s_stabilizerStep);

  Setpoint setpoint;
  commanderGetSetpoint(&setpoint);

  supervisorUpdate(s_stabilizerStep);

  if (!supervisorCanFly()) {
    memset(&setpoint, 0, sizeof(setpoint));
  }

  supervisorOverrideSetpoint(&setpoint, supervisorConditionBits, supervisorState);

  State state;
  memset(&state, 0, sizeof(state));
  state.attitude.roll = stateEstimate.roll;
  state.attitude.pitch = stateEstimate.pitch;
  state.attitude.yaw = stateEstimate.yaw;
  state.attitudeQuaternion.w = qw;
  state.attitudeQuaternion.x = qx;
  state.attitudeQuaternion.y = qy;
  state.attitudeQuaternion.z = qz;
  state.acc = sensors.acc;

  ControlData control;
  memset(&control, 0, sizeof(control));
  controllerPid(&sensors, &setpoint, &state, &control, 0.0f,
                1.0f / (float)ATTITUDE_RATE_HZ);

  MotorPower mp;
  powerDistribution(&control, &mp);

  float filtered = 4.2f;
  int32_t ratios[4] = { mp.m1, mp.m2, mp.m3, mp.m4 };
  for (uint8_t i = 0; i < 4; i++) {
    ratios[i] = (int32_t)batteryCompensation((float)ratios[i], filtered, 0.01f);
  }
  powerDistributionCap(ratios, 65535, 0);

  if (!supervisorAreMotorsAllowedToRun()) {
    ratios[0] = ratios[1] = ratios[2] = ratios[3] = 0;
  }

  motor.m1req = (uint16_t)clampf((float)ratios[0], 0.0f, 65535.0f);
  motor.m2req = (uint16_t)clampf((float)ratios[1], 0.0f, 65535.0f);
  motor.m3req = (uint16_t)clampf((float)ratios[2], 0.0f, 65535.0f);
  motor.m4req = (uint16_t)clampf((float)ratios[3], 0.0f, 65535.0f);
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
  output->gyro_millirad_s[0] = sensors->gyro.x * DEG_TO_RAD_F * 1000.0f;
  output->gyro_millirad_s[1] = -sensors->gyro.y * DEG_TO_RAD_F * 1000.0f;
  output->gyro_millirad_s[2] = sensors->gyro.z * DEG_TO_RAD_F * 1000.0f;
  output->quatCompressed = quatcompress(state->attitudeQuaternion.w, state->attitudeQuaternion.x,
                                        state->attitudeQuaternion.y, state->attitudeQuaternion.z);
}

bool rateSupervisorValidate(uint32_t measuredRate) {
  return measuredRate >= 997 && measuredRate <= 1003;
}

void rateSupervisorTask(void) {
  if (!s_rateWaitStarted) {
    s_rateWaitStarted = true;
    s_rateWaitStart = currentTick;
    return;
  }
  if ((currentTick - s_rateWaitStart) >= 2000) {
    if (!sensorsPaused) s_rateSupervisorError = true;
  }
}

bool healthShallWeRunTest(void) {
  if (s_propTestRequest) {
    s_propTestRequest = false;
    healthTestState = configureAcc;
    s_healthTestActive = true;
    s_propSampleCount = 0;
    s_propMotorIndex = 0;
    s_propNoiseVariance = 0.0f;
    motorPass = 0;
    s_motorTestCount = 0;
    return true;
  }
  if (s_batTestRequest) {
    s_batTestRequest = false;
    healthTestState = testBattery;
    s_healthTestActive = true;
    s_batTick = 0;
    batterySag = 0.0f;
    batteryPass = 0;
    return true;
  }
  return s_healthTestActive;
}

void healthRequestPropTest(void) { s_propTestRequest = true; }
void healthRequestBatteryTest(void) { s_batTestRequest = true; }

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex) {
  if (motorIndex > 3) return false;
  if (highThreshold == 0.0f) return true;
  if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
    motorPass |= (1U << motorIndex);
    healthLog.motorPass = motorPass;
    return true;
  }
  s_propFailureCount++;
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
      s_propSampleCount = 0;
      s_propMotorIndex = 0;
      healthTestState = measureNoiseFloor;
      break;
    case measureNoiseFloor:
      if (s_propSampleCount < PROPTEST_NBR_OF_VARIANCE_VALUES) {
        float norm = sqrtf(sensorData->acc.x * sensorData->acc.x +
                           sensorData->acc.y * sensorData->acc.y +
                           sensorData->acc.z * sensorData->acc.z);
        s_propAccBuffer[s_propSampleCount++] = norm;
        if (s_propSampleCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
          s_propNoiseVariance = variance(s_propAccBuffer, PROPTEST_NBR_OF_VARIANCE_VALUES);
          healthTestState = measureProp;
        }
      }
      break;
    case measureProp:
      if (s_propMotorIndex < 4) {
        float measured = s_propNoiseVariance + sensorData->acc.z * 0.01f;
        bool pass = evaluatePropTest(0.0f, 0.0f, measured, s_propMotorIndex);
        if (pass) motorPass |= (1U << s_propMotorIndex);
        s_motorTestCount++;
        healthLog.motorTestCount = s_motorTestCount;
        healthLog.motorPass = motorPass;
        s_propMotorIndex++;
        if (s_propMotorIndex >= 4) healthTestState = evaluatePropResult;
      }
      break;
    case evaluatePropResult:
      healthLog.motorPass = motorPass;
      healthLog.motorTestCount = s_motorTestCount;
      healthTestState = testDone;
      s_healthTestActive = false;
      break;
    case testBattery:
      if (s_batTick == 0) {
        s_batTick = 1;
        s_minLoadedVoltage = batteryVoltage;
      } else if (s_batTick < 50) {
        if (batteryVoltage < s_minLoadedVoltage) s_minLoadedVoltage = batteryVoltage;
        s_batTick++;
      } else {
        batterySag = s_idleVoltage - s_minLoadedVoltage;
        batteryPass = (batterySag <= 0.15f) ? 1 : 0;
        healthLog.batteryPass = batteryPass;
        healthLog.batterySag = batterySag;
        s_batTick = 0;
        healthTestState = evaluateBatResult;
      }
      break;
    case evaluateBatResult:
      healthLog.batteryPass = batteryPass;
      healthLog.batterySag = batterySag;
      healthTestState = testDone;
      s_healthTestActive = false;
      break;
    case restartBatTest:
      s_batTick = 0;
      healthTestState = testBattery;
      s_healthTestActive = true;
      break;
    case testDone:
      s_healthTestActive = false;
      break;
    default:
      break;
  }
}

void crtpInit(void) {
  if (s_crtpInitialized) return;
  s_txHead = s_txTail = s_txCount = 0;
  memset(s_txQueue, 0, sizeof(s_txQueue));
  for (uint8_t p = 0; p < CRTP_NBR_OF_PORTS; p++) {
    s_crtpQueueExists[p] = false;
    s_rxHead[p] = s_rxTail[p] = s_rxCount[p] = 0;
    s_portCallbacks[p] = NULL;
  }
  s_activeLink = &s_nopLink;
  s_crtpErrorState = false;
  s_crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port) {
  if (port >= CRTP_NBR_OF_PORTS) return;
  if (s_crtpQueueExists[port]) {
    s_crtpErrorState = true;
    return;
  }
  s_rxHead[port] = s_rxTail[port] = s_rxCount[port] = 0;
  s_crtpQueueExists[port] = true;
}

bool crtpSendPacket(const CrtpPacket *packet) {
  if (!packet || s_txCount >= CRTP_TX_QUEUE_SIZE) return false;
  s_txQueue[s_txTail] = *packet;
  s_txTail = (uint16_t)((s_txTail + 1) % CRTP_TX_QUEUE_SIZE);
  s_txCount++;
  return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
  return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
  if (port >= CRTP_NBR_OF_PORTS || !packet || !s_crtpQueueExists[port] || s_rxCount[port] == 0) return false;
  *packet = s_rxQueues[port][s_rxHead[port]];
  s_rxHead[port] = (uint8_t)((s_rxHead[port] + 1) % CRTP_RX_QUEUE_SIZE);
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
  if (!s_activeLink || !s_activeLink->receivePacket) return;
  CrtpPacket pkt;
  if (s_activeLink->receivePacket(&pkt)) {
    s_rxPacketCount++;
    bool delivered = false;
    if (pkt.port < CRTP_NBR_OF_PORTS) {
      if (s_crtpQueueExists[pkt.port] && s_rxCount[pkt.port] < CRTP_RX_QUEUE_SIZE) {
        s_rxQueues[pkt.port][s_rxTail[pkt.port]] = pkt;
        s_rxTail[pkt.port] = (uint8_t)((s_rxTail[pkt.port] + 1) % CRTP_RX_QUEUE_SIZE);
        s_rxCount[pkt.port]++;
        delivered = true;
      }
      if (s_portCallbacks[pkt.port]) {
        s_portCallbacks[pkt.port](&pkt);
        delivered = true;
      }
    }
    (void)delivered;
  }
}

void crtpTxTask(void) {
  if (s_activeLink == &s_nopLink) return;
  if (s_txCount == 0) return;
  if (s_txRetryPending && currentTick < s_txRetryTick) return;
  s_txRetryPending = false;
  if (!s_activeLink || !s_activeLink->sendPacket) return;
  CrtpPacket *front = &s_txQueue[s_txHead];
  if (s_activeLink->sendPacket(front)) {
    s_txHead = (uint16_t)((s_txHead + 1) % CRTP_TX_QUEUE_SIZE);
    s_txCount--;
    s_txPacketCount++;
  } else {
    s_txRetryPending = true;
    s_txRetryTick = currentTick + 10U;
  }
}

void crtpSetLink(CrtpLink *newLink) {
  if (s_activeLink && s_activeLink != &s_nopLink && s_activeLink->setEnable) {
    s_activeLink->setEnable(false);
  }
  s_activeLink = newLink ? newLink : &s_nopLink;
  if (s_activeLink && s_activeLink->setEnable) s_activeLink->setEnable(true);
  s_txRetryPending = false;
  s_txRetryTick = 0;
}

void crtpReset(void) {
  s_txHead = s_txTail = s_txCount = 0;
  memset(s_txQueue, 0, sizeof(s_txQueue));
  s_txRetryPending = false;
  s_txRetryTick = 0;
  if (s_activeLink && s_activeLink->reset) s_activeLink->reset();
}

bool crtpIsConnected(void) {
  if (s_activeLink && s_activeLink->isConnected) return s_activeLink->isConnected();
  return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
  return CRTP_TX_QUEUE_SIZE - s_txCount;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
  if (port >= CRTP_NBR_OF_PORTS) return;
  s_portCallbacks[port] = callback;
}

void updateStats(void) {
  if ((currentTick - s_lastStatsTick) >= 500) {
    crtpRxRate = s_rxPacketCount;
    crtpTxRate = s_txPacketCount;
    s_rxPacketCount = 0;
    s_txPacketCount = 0;
    s_lastStatsTick = currentTick;
  }
}

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
  if (!decks || capacity == 0) return 0;
  // Environment interface intentionally leaves the mock deck inventory as a
  // platform adapter boundary. No static inventory is supplied in the frozen
  // contract; return a deterministic empty scan without out-of-bounds writes.
  (void)decks;
  (void)capacity;
  return 0;
}
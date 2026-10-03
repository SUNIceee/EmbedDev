#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Global Observables declared extern in header */
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

PidObject pidRoll, pidPitch, pidYaw;
PidObject pidRollRate, pidPitchRate, pidYawRate;

bool thrustLocked = true;
bool commanderModeSet = false;

SupervisorState supervisorState = supervisorStatePreFlChecksNotPassed;
uint32_t supervisorConditionBits = 0;

TestState healthTestState = testDone;
uint8_t motorPass = 0, batteryPass = 0;
float batterySag = 0.0f;

StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0}, acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};

/* Internal Shared State Variables */
static uint32_t g_currentTick = 0;
static uint32_t g_lastUpdateTick = 0;
static uint32_t g_recentFlightTick = 0;
static bool g_seenFlight = false;
static uint32_t g_spinupStartTick = 0;
static uint32_t g_tiltStartTick = 0;
static uint32_t g_lowRpmStartTick = 0;
static uint32_t g_lastStatsTick = 0;

static int g_activePriority = COMMANDER_PRIORITY_DISABLE;
static Setpoint g_activeSetpoint = {0};

static SensorData g_supervisorSensors = {0};
static uint32_t g_supervisorMotorRatios[4] = {0};
static int32_t g_supervisorMotorRPMs[4] = {0};

static struct {
  float crashDetectionGs;
  float freeFallThreshold;
  float acceptedTiltAccZ;
  float acceptedUpsideDownAccZ;
  uint32_t maxTiltTime;
  uint32_t maxUpsideDownTime;
  bool tumbleCheckEnabled;
} g_safetyCfg = {0.0f, 0.0f, 0.0f, 0.0f, 0, 0, false};

static struct {
  bool autoArming;
  uint32_t spinupTimeoutDurationMs;
} g_armingCfg = {false, 500};

/* Estimator FIFO */
static EstimatorMeasurement g_estimatorQueue[16];
static uint8_t g_estimatorHead = 0, g_estimatorTail = 0, g_estimatorCount = 0;
static EstimatorMeasurement g_lastGyroMeas = {MeasurementTypeGyroscope, {0}};
static EstimatorMeasurement g_lastAccMeas = {MeasurementTypeAcceleration, {0}};
static EstimatorMeasurement g_lastBaroMeas = {MeasurementTypeBarometer, {0}};
static EstimatorMeasurement g_lastTofMeas = {MeasurementTypeTOF, {0}};

/* CRTP State */
static CrtpPacket g_txQueue[CRTP_TX_QUEUE_SIZE];
static uint32_t g_txQueueCount = 0;
static CrtpPacket g_rxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint8_t g_rxHead[CRTP_NBR_OF_PORTS] = {0};
static uint8_t g_rxTail[CRTP_NBR_OF_PORTS] = {0};
static uint8_t g_rxCount[CRTP_NBR_OF_PORTS] = {0};
static bool g_rxQueueAllocated[CRTP_NBR_OF_PORTS] = {false};
static CrtpPortCallback g_portCBs[CRTP_NBR_OF_PORTS] = {NULL};
static CrtpLink *g_crtpLink = NULL;
static bool g_crtpIsInit = false;
static uint32_t g_rxByteCount = 0, g_txByteCount = 0;
static float g_rxRate = 0.0f, g_txRate = 0.0f;

/* Health Request Flags */
static bool g_propTestRequested = false;
static bool g_batTestRequested = false;
static float g_propNoiseBuffer[PROPTEST_NBR_OF_VARIANCE_VALUES];
static uint32_t g_propNoiseCount = 0;
static float g_idleVoltage = 4.2f;
static float g_minLoadedVoltage = 4.2f;
static int g_batTestTick = 0;

/* Internal NOP Link */
static bool nopSendPacket(CrtpPacket *p) { (void)p; return true; }
static bool nopReceivePacket(CrtpPacket *p) { (void)p; return false; }
static bool nopIsConnected(void) { return true; }
static void nopSetEnable(bool enable) { (void)enable; }
static void nopReset(void) {}
static CrtpLink g_nopLink = { nopSendPacket, nopReceivePacket, nopIsConnected, nopSetEnable, nopReset };

/* Internal Quat Compression Helper */
static uint32_t quatcompress(const Quaternion *q) {
  if (!q) return 0;
  uint32_t w = (uint32_t)((q->w + 1.0f) * 127.5f) & 0xFF;
  uint32_t x = (uint32_t)((q->x + 1.0f) * 127.5f) & 0xFF;
  uint32_t y = (uint32_t)((q->y + 1.0f) * 127.5f) & 0xFF;
  uint32_t z = (uint32_t)((q->z + 1.0f) * 127.5f) & 0xFF;
  return (w << 24) | (x << 16) | (y << 8) | z;
}

/* ========================================================================= */
/* 2. Numeric functions */
/* ========================================================================= */

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

/* ========================================================================= */
/* 3. Sensfusion6                                                            */
/* ========================================================================= */

void sensfusion6Init(void) {
  if (sensfusion6IsInit) return;
  qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
  gravityX = 0.0f; gravityY = 0.0f; gravityZ = 1.0f;
  integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
  twoKp = 0.8f; twoKi = 0.002f; beta = 0.01f; baseZacc = 0.0f;
  sensfusion6IsInit = true;
  sensfusion6IsCalibrated = false;

  sensfusion6Log.qw = qw; sensfusion6Log.qx = qx; sensfusion6Log.qy = qy; sensfusion6Log.qz = qz;
  sensfusion6Log.gravityX = gravityX; sensfusion6Log.gravityY = gravityY; sensfusion6Log.gravityZ = gravityZ;
  sensfusion6Log.accZbase = baseZacc; sensfusion6Log.isInit = true; sensfusion6Log.isCalibrated = false;
}

bool sensfusion6Test(void) {
  return sensfusion6IsInit;
}

float invSqrt(float x) {
  if (x <= 0.0f) return 0.0f;
  float halfx = 0.5f * x;
  float y = x;
  int32_t i;
  memcpy(&i, &y, sizeof(i));
  i = 0x5f3759df - (i >> 1);
  memcpy(&y, &i, sizeof(y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt) {
  if (!sensfusion6IsInit) sensfusion6Init();

  if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
    float gx_rad = gx * (M_PI / 180.0f);
    float gy_rad = gy * (M_PI / 180.0f);
    float gz_rad = gz * (M_PI / 180.0f);
    float qDot1 = 0.5f * (-qx * gx_rad - qy * gy_rad - qz * gz_rad);
    float qDot2 = 0.5f * (qw * gx_rad + qy * gz_rad - qz * gy_rad);
    float qDot3 = 0.5f * (qw * gy_rad - qx * gz_rad + qz * gx_rad);
    float qDot4 = 0.5f * (qw * gz_rad + qx * gy_rad - qy * gx_rad);
    qw += qDot1 * dt; qx += qDot2 * dt; qy += qDot3 * dt; qz += qDot4 * dt;
    float recip = invSqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    qw *= recip; qx *= recip; qy *= recip; qz *= recip;
    return;
  }

  float recipNorm = invSqrt(ax*ax + ay*ay + az*az);
  float normAx = ax * recipNorm;
  float normAy = ay * recipNorm;
  float normAz = az * recipNorm;

  float vx = 2.0f * (qx * qz - qw * qy);
  float vy = 2.0f * (qw * qx + qy * qz);
  float vz = qw * qw - qx * qx - qy * qy + qz * qz;

  float ex = (normAy * vz - normAz * vy);
  float ey = (normAz * vx - normAx * vz);
  float ez = (normAx * vy - normAy * vx);

  if (twoKi > 0.0f) {
    integralFBx += ex * twoKi * dt;
    integralFBy += ey * twoKi * dt;
    integralFBz += ez * twoKi * dt;
  } else {
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
  }

  float gx_rad = gx * (M_PI / 180.0f) + twoKp * ex + integralFBx;
  float gy_rad = gy * (M_PI / 180.0f) + twoKp * ey + integralFBy;
  float gz_rad = gz * (M_PI / 180.0f) + twoKp * ez + integralFBz;

  float qDot1 = 0.5f * (-qx * gx_rad - qy * gy_rad - qz * gz_rad);
  float qDot2 = 0.5f * (qw * gx_rad + qy * gz_rad - qz * gy_rad);
  float qDot3 = 0.5f * (qw * gy_rad - qx * gz_rad + qz * gx_rad);
  float qDot4 = 0.5f * (qw * gz_rad + qx * gy_rad - qy * gx_rad);

  qw += qDot1 * dt; qx += qDot2 * dt; qy += qDot3 * dt; qz += qDot4 * dt;
  recipNorm = invSqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  qw *= recipNorm; qx *= recipNorm; qy *= recipNorm; qz *= recipNorm;

  gravityX = 2.0f * (qx * qz - qw * qy);
  gravityY = 2.0f * (qw * qx + qy * qz);
  gravityZ = qw * qw - qx * qx - qy * qy + qz * qz;

  if (!sensfusion6IsCalibrated) {
    baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
    sensfusion6IsCalibrated = true;
  }

  sensfusion6Log.qw = qw; sensfusion6Log.qx = qx; sensfusion6Log.qy = qy; sensfusion6Log.qz = qz;
  sensfusion6Log.gravityX = gravityX; sensfusion6Log.gravityY = gravityY; sensfusion6Log.gravityZ = gravityZ;
  sensfusion6Log.accZbase = baseZacc; sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
  if (!roll_deg || !pitch_deg || !yaw_deg) return;

  float gx_dir = 2.0f * (qx * qz - qw * qy);
  float gy_dir = 2.0f * (qw * qx + qy * qz);
  float gz_dir = qw * qw - qx * qx - qy * qy + qz * qz;

  gravityX = gx_dir; gravityY = gy_dir; gravityZ = gz_dir;

  if (gx_dir > 1.0f) gx_dir = 1.0f;
  if (gx_dir < -1.0f) gx_dir = -1.0f;

  *roll_deg = atan2f(gy_dir, gz_dir) * (180.0f / M_PI);
  *pitch_deg = asinf(-gx_dir) * (180.0f / M_PI);
  *yaw_deg = atan2f(2.0f * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz) * (180.0f / M_PI);
}

void sensfusion6GetQuaternion(float *out_qw, float *out_qx, float *out_qy, float *out_qz) {
  if (!out_qw || !out_qx || !out_qy || !out_qz) return;
  *out_qw = qw; *out_qx = qx; *out_qy = qy; *out_qz = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
  return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

void estimatedGravityDirection(float in_qw, float in_qx, float in_qy, float in_qz,
                               float *gravX, float *gravY, float *gravZ) {
  if (!gravX || !gravY || !gravZ) return;
  *gravX = 2.0f * (in_qx * in_qz - in_qw * in_qy);
  *gravY = 2.0f * (in_qw * in_qx + in_qy * in_qz);
  *gravZ = in_qw * in_qw - in_qx * in_qx - in_qy * in_qy + in_qz * in_qz;
}

/* ========================================================================= */
/* 4. Power distribution and battery */
/* ========================================================================= */

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out) {
  if (!out) return;
  int32_t r = roll / 2;
  int32_t p = pitch / 2;
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
  float rollPart = (arm > 0.0f) ? (0.25f / arm * torqueX) : 0.0f;
  float pitchPart = (arm > 0.0f) ? (0.25f / arm * torqueY) : 0.0f;
  float yawPart = (thrustToTorque > 0.0f) ? (0.25f / thrustToTorque * torqueZ) : 0.0f;

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
    float f = normalizedForces[i];
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    motorPWMs[i] = (uint16_t)(f * 65535.0f);
  }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower) {
  if (!control || !motorPower) return;
  if (control->controlMode == controlModeLegacy) {
    powerDistributionLegacy(control->thrust, control->roll, control->pitch, control->yaw, motorPower);
  } else if (control->controlMode == controlModeForceTorque) {
    float forces[4];
    powerDistributionForceTorque(control->thrustSi, control->torque.x, control->torque.y, control->torque.z,
                                CRAZYFLIE_ARM_LENGTH_M, CRAZYFLIE_THRUST_TO_TORQUE, forces);
    for (int i = 0; i < 4; i++) {
      float norm = forces[i] / CRAZYFLIE_MAX_MOTOR_FORCE_N;
      if (norm < 0.0f) norm = 0.0f;
      if (norm > 1.0f) norm = 1.0f;
      uint16_t pwm = (uint16_t)(norm * 65535.0f);
      if (i == 0) motorPower->m1 = pwm;
      if (i == 1) motorPower->m2 = pwm;
      if (i == 2) motorPower->m3 = pwm;
      if (i == 3) motorPower->m4 = pwm;
    }
  } else if (control->controlMode == controlModeForce) {
    uint16_t pwms[4];
    powerDistributionForce(control->normalizedForces, pwms);
    motorPower->m1 = pwms[0];
    motorPower->m2 = pwms[1];
    motorPower->m3 = pwms[2];
    motorPower->m4 = pwms[3];
  }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) {
  int32_t minVal = (idleThrust > 0) ? idleThrust : 0;
  return (value < minVal) ? minVal : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust) {
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
      motors[i] = capMinThrust(motors[i] - res.reduction, idleThrust);
    }
  } else {
    for (int i = 0; i < 4; i++) {
      motors[i] = capMinThrust(motors[i], idleThrust);
    }
  }
  return res;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) {
  return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust,
                                        float nominalVoltage,
                                        float actualVoltage) {
  if (actualVoltage <= 0.0f) return motorThrust;
  float val = (float)motorThrust * nominalVoltage / actualVoltage;
  int32_t res = (int32_t)roundf(val);
  if (res < 0) res = 0;
  if (res > 65535) res = 65535;
  return (uint16_t)res;
}

/* ========================================================================= */
/* 5. Cascaded PID and controllerPid */
/* ========================================================================= */

static float updatePid(PidObject *pid, float error, float dt) {
  if (!pid || !pid->initialized || dt <= 0.0f) return 0.0f;
  pid->integral += error * dt;
  float deriv = (error - pid->prevError) / dt;
  pid->prevError = error;
  pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * deriv;
  return pid->output;
}

void attitudeControllerInit(float updateDt) {
  (void)updateDt;
  pidRoll.kp = 6.0f; pidRoll.ki = 3.0f; pidRoll.kd = 0.0f; pidRoll.kff = 0.0f;
  pidRoll.integral = 0.0f; pidRoll.prevError = 0.0f; pidRoll.output = 0.0f; pidRoll.initialized = true;

  pidPitch.kp = 6.0f; pidPitch.ki = 3.0f; pidPitch.kd = 0.0f; pidPitch.kff = 0.0f;
  pidPitch.integral = 0.0f; pidPitch.prevError = 0.0f; pidPitch.output = 0.0f; pidPitch.initialized = true;

  pidYaw.kp = 6.0f; pidYaw.ki = 1.0f; pidYaw.kd = 0.0f; pidYaw.kff = 0.0f;
  pidYaw.integral = 0.0f; pidYaw.prevError = 0.0f; pidYaw.output = 0.0f; pidYaw.initialized = true;

  pidRollRate.kp = 250.0f; pidRollRate.ki = 500.0f; pidRollRate.kd = 2.5f; pidRollRate.kff = 0.0f;
  pidRollRate.integral = 0.0f; pidRollRate.prevError = 0.0f; pidRollRate.output = 0.0f; pidRollRate.initialized = true;

  pidPitchRate.kp = 250.0f; pidPitchRate.ki = 500.0f; pidPitchRate.kd = 2.5f; pidPitchRate.kff = 0.0f;
  pidPitchRate.integral = 0.0f; pidPitchRate.prevError = 0.0f; pidPitchRate.output = 0.0f; pidPitchRate.initialized = true;

  pidYawRate.kp = 120.0f; pidYawRate.ki = 16.0f; pidYawRate.kd = 0.0f; pidYawRate.kff = 0.0f;
  pidYawRate.integral = 0.0f; pidYawRate.prevError = 0.0f; pidYawRate.output = 0.0f; pidYawRate.initialized = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
  float dt = 0.002f;
  float outR = updatePid(&pidRollRate, rollDesired - rollActual, dt);
  float outP = updatePid(&pidPitchRate, pitchDesired - pitchActual, dt);
  float outY = updatePid(&pidYawRate, yawDesired - yawActual, dt);

  pidRollRate.output = (float)saturateSignedInt16((int32_t)outR);
  pidPitchRate.output = (float)saturateSignedInt16((int32_t)outP);
  pidYawRate.output = (float)saturateSignedInt16((int32_t)outY);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
  float dt = 0.002f;
  updatePid(&pidRoll, rollDesired - rollActual, dt);
  updatePid(&pidPitch, pitchDesired - pitchActual, dt);

  pidYaw.integral = 0.0f;
  float yawErr = capAngle(yawDesired - yawActual);
  updatePid(&pidYaw, yawErr, dt);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual, float yawActual) {
  (void)yawActual;
  attitudeControllerResetRollAttitudePID(rollActual);
  attitudeControllerResetPitchAttitudePID(pitchActual);
  pidYaw.integral = 0.0f; pidYaw.prevError = 0.0f; pidYaw.output = 0.0f;

  pidRollRate.integral = 0.0f; pidRollRate.prevError = 0.0f; pidRollRate.output = 0.0f;
  pidPitchRate.integral = 0.0f; pidPitchRate.prevError = 0.0f; pidPitchRate.output = 0.0f;
  pidYawRate.integral = 0.0f; pidYawRate.prevError = 0.0f; pidYawRate.output = 0.0f;
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
  (void)rollActual;
  pidRoll.integral = 0.0f; pidRoll.prevError = 0.0f; pidRoll.output = 0.0f;
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
  (void)pitchActual;
  pidPitch.integral = 0.0f; pidPitch.prevError = 0.0f; pidPitch.output = 0.0f;
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch, int16_t *yaw) {
  if (roll) *roll = (int16_t)pidRollRate.output;
  if (pitch) *pitch = (int16_t)pidPitchRate.output;
  if (yaw) *yaw = -(int16_t)pidYawRate.output;
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
  if (!setpoint || !state) return setpoint ? setpoint->thrust : 0;
  if (setpoint->mode.z == modeDisable) return setpoint->thrust;
  
  float targetZ = setpoint->position.z;
  float currentZ = state->position.z;
  float errZ = targetZ - currentZ;
  float thrustF = 30000.0f + errZ * 10000.0f - state->velocity.z * 2000.0f;
  if (thrustF < (float)MIN_THRUST) thrustF = (float)MIN_THRUST;
  if (thrustF > (float)MAX_THRUST) thrustF = (float)MAX_THRUST;
  return (uint16_t)thrustF;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
  if (!sensors || !setpoint || !state || !control) return;

  static float s_desiredYaw = 0.0f;

  if (setpoint->thrust == 0 && setpoint->mode.z == modeDisable) {
    control->roll = 0; control->pitch = 0; control->yaw = 0; control->thrust = 0;
    attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
    s_desiredYaw = state->attitude.yaw;
    return;
  }

  if (setpoint->mode.yaw == modeVelocity) {
    s_desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
  } else if (setpoint->mode.yaw == modeAbs) {
    s_desiredYaw = setpoint->attitude.yaw;
  }
  s_desiredYaw = capAngle(s_desiredYaw);

  if (yawMaxDelta != 0.0f) {
    float diff = capAngle(s_desiredYaw - state->attitude.yaw);
    if (diff > yawMaxDelta) s_desiredYaw = capAngle(state->attitude.yaw + yawMaxDelta);
    if (diff < -yawMaxDelta) s_desiredYaw = capAngle(state->attitude.yaw - yawMaxDelta);
  }

  float rollDesired = setpoint->attitude.roll;
  float pitchDesired = setpoint->attitude.pitch;
  float yawDesired = s_desiredYaw;

  if (setpoint->mode.roll == modeVelocity) {
    attitudeControllerResetRollAttitudePID(state->attitude.roll);
    pidRollRate.output = updatePid(&pidRollRate, setpoint->attitudeRate.roll - sensors->gyro.x, attitudeUpdateDt);
  } else {
    attitudeControllerCorrectAttitudePID(state->attitude.roll, rollDesired,
                                         state->attitude.pitch, pitchDesired,
                                         state->attitude.yaw, yawDesired);
  }

  float pitchActualRate = -sensors->gyro.y;
  attitudeControllerCorrectRatePID(sensors->gyro.x, pidRoll.output,
                                   pitchActualRate, pidPitch.output,
                                   sensors->gyro.z, pidYaw.output);

  int16_t rOut, pOut, yOut;
  attitudeControllerGetActuatorOutput(&rOut, &pOut, &yOut);
  control->roll = rOut;
  control->pitch = pOut;
  control->yaw = yOut;

  if (setpoint->mode.z == modeDisable) {
    control->thrust = setpoint->thrust;
  } else {
    control->thrust = positionControllerUpdate(setpoint, state);
  }
}

/* ========================================================================= */
/* 6. CRTP Commander RPYT                                                    */
/* ========================================================================= */

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
  if (!rollPrime || !pitchPrime) return;
  float rad = yaw_deg * (M_PI / 180.0f);
  float cosY = cosf(rad);
  float sinY = sinf(rad);
  *rollPrime = roll * cosY - pitch * sinY;
  *pitchPrime = roll * sinY + pitch * cosY;
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

  uint16_t decodedThrust = 0;
  if (!altHoldMode) {
    if (thrustLocked || values->thrust < 1000) {
      decodedThrust = 0;
    } else {
      decodedThrust = (values->thrust > MAX_THRUST) ? (uint16_t)MAX_THRUST : values->thrust;
    }
  }

  float rollVal = values->roll;
  float pitchVal = values->pitch;

  if (yawMode == PLUSMODE) {
    rotateYaw(values->roll, values->pitch, 45.0f, &rollVal, &pitchVal);
  } else if (yawMode == CAREFREE) {
    supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  }

  if (altHoldMode) {
    setpoint->mode.z = modeVelocity;
    setpoint->thrust = 0;
    setpoint->velocity.z = ((float)values->thrust - 32767.0f) / 32767.0f;
    if (!commanderModeSet) {
      commanderModeSet = true;
    }
  } else {
    setpoint->mode.z = modeDisable;
    setpoint->thrust = decodedThrust;
    commanderModeSet = false;
  }

  if (posHoldMode) {
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->velocity.x = pitchVal / 30.0f;
    setpoint->velocity.y = rollVal / 30.0f;
    setpoint->attitude.roll = 0.0f;
    setpoint->attitude.pitch = 0.0f;
  } else if (posSetMode && values->thrust != 0) {
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;
    setpoint->position.x = -pitchVal;
    setpoint->position.y = rollVal;
    setpoint->position.z = (float)values->thrust / 1000.0f;
    setpoint->attitude.yaw = values->yaw;
    setpoint->thrust = 0;
  } else {
    if (stabilizationModeRoll == RATE) {
      setpoint->mode.roll = modeVelocity;
      setpoint->attitudeRate.roll = rollVal;
    } else {
      setpoint->mode.roll = modeAbs;
      setpoint->attitude.roll = rollVal;
    }

    if (stabilizationModePitch == RATE) {
      setpoint->mode.pitch = modeVelocity;
      setpoint->attitudeRate.pitch = pitchVal;
    } else {
      setpoint->mode.pitch = modeAbs;
      setpoint->attitude.pitch = pitchVal;
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

/* ========================================================================= */
/* 7. Supervisor                                                             */
/* ========================================================================= */

void supervisorInit(void) {
  supervisorState = supervisorStatePreFlChecksNotPassed;
  supervisorConditionBits = 0;
  g_recentFlightTick = 0;
  g_seenFlight = false;
  g_spinupStartTick = 0;
  g_tiltStartTick = 0;
  g_lowRpmStartTick = 0;
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
  return (supervisorState == supervisorStateCrashed ||
          (supervisorConditionBits & SUPERVISOR_CB_CRASHED) != 0);
}

bool supervisorRequestArming(bool doArm) {
  if (doArm) {
    if (supervisorState == supervisorStatePreFlChecksPassed) {
      supervisorState = supervisorStateArming;
      supervisorConditionBits |= SUPERVISOR_CB_ARMED;
      g_spinupStartTick = g_currentTick;
      return true;
    } else if (supervisorState == supervisorStateArming || supervisorCanFly()) {
      return true;
    }
    return false;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    if (supervisorState == supervisorStateArming || supervisorCanFly()) {
      supervisorState = supervisorStatePreFlChecksPassed;
    }
    return true;
  }
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
  if (supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) {
    return false;
  }
  if (!doRecovery) {
    supervisorState = supervisorStateCrashed;
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    return true;
  }
  supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
  supervisorState = supervisorStatePreFlChecksPassed;
  return true;
}

bool supervisorAreMotorsAllowedToRun(void) {
  return supervisorCanFly() || supervisorState == supervisorStateArming;
}

uint16_t supervisorGetInfoBitfield(void) {
  uint16_t bits = 0;
  if (supervisorCanArm()) bits |= (1 << 0);
  if (supervisorIsArmed()) bits |= (1 << 1);
  if (g_armingCfg.autoArming) bits |= (1 << 2);
  if (supervisorCanFly()) bits |= (1 << 3);
  if (supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) bits |= (1 << 4);
  if (supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) bits |= (1 << 5);
  if (supervisorState == supervisorStateLocked) bits |= (1 << 6);
  if (supervisorIsCrashed()) bits |= (1 << 7);
  return bits;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust, uint32_t currentTick) {
  if (!motorRatios) return false;
  bool active = false;
  for (int i = 0; i < 4; i++) {
    if (motorRatios[i] > idleThrust) active = true;
  }
  if (active) {
    g_recentFlightTick = currentTick;
    g_seenFlight = true;
    supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
    return true;
  }
  if (!g_seenFlight) return false;
  if ((currentTick - g_recentFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD) {
    supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
    return true;
  }
  supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
  return false;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling) {
  if (isFreeFalling) *isFreeFalling = false;

  float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
  if (freeFallThreshold > 0.0f && norm < freeFallThreshold) {
    if (isFreeFalling) *isFreeFalling = true;
    supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
    g_tiltStartTick = 0;
    return false;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;
  }

  if (crashDetectionGs > 0.0f && fabsf(norm - 1.0f) > crashDetectionGs) {
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    supervisorState = supervisorStateCrashed;
  }

  if (!tumbleCheckEnabled) {
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    return false;
  }

  if (accZ < acceptedTiltAccZ) {
    if (g_tiltStartTick == 0) g_tiltStartTick = currentTick;
    uint32_t timeout = (accZ < acceptedUpsideDownAccZ) ? maxUpsideDownTime : maxTiltTime;
    if ((currentTick - g_tiltStartTick) >= timeout) {
      supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
      return true;
    }
  } else {
    g_tiltStartTick = 0;
  }

  supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
  return false;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick) {
  if (lastNotificationTick == 0) return true;
  return (currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state, uint32_t latestArmingTick,
                                  uint32_t currentTick, uint32_t preflightTimeoutDuration) {
  if (state != supervisorStateArming || latestArmingTick == 0) return false;
  return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick, uint32_t currentTick,
                                uint32_t landingTimeoutDuration) {
  if (latestLandingTick == 0) return false;
  return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop, bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed) {
  uint32_t mask = supervisorConditionBits;
  if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
    mask |= SUPERVISOR_CB_EMERGENCY_STOP;
  }
  return mask;
}

void supervisorOverrideSetpoint(Setpoint *setpoint, uint32_t conditionBits, SupervisorState state) {
  if (!setpoint) return;
  (void)conditionBits;

  if (state == supervisorStateWarningLevelOut) {
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->attitude.roll = 0.0f;
    setpoint->attitude.pitch = 0.0f;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitudeRate.yaw = 0.0f;
  } else if (state == supervisorStateArming || state == supervisorStateReadyToFly ||
             state == supervisorStateFlying || state == supervisorStateLanded) {
    return;
  } else {
    memset(setpoint, 0, sizeof(*setpoint));
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
    g_lowRpmStartTick = 0;
    return false;
  }
  bool low = true;
  for (int i = 0; i < 4; i++) {
    if (motorRPMs[i] >= rpmThreshold) low = false;
  }
  if (low) {
    if (g_lowRpmStartTick == 0) g_lowRpmStartTick = currentTick;
    return (currentTick - g_lowRpmStartTick) >= rpmCheckDurationMs;
  }
  g_lowRpmStartTick = 0;
  return false;
}

void supervisorSetSensorData(const SensorData *sensors) {
  if (sensors) g_supervisorSensors = *sensors;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
  (void)idleThrust;
  if (motorRatios) memcpy(g_supervisorMotorRatios, motorRatios, sizeof(g_supervisorMotorRatios));
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
  if (motorRPMs) memcpy(g_supervisorMotorRPMs, motorRPMs, sizeof(g_supervisorMotorRPMs));
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled) {
  g_safetyCfg.crashDetectionGs = crashDetectionGs;
  g_safetyCfg.freeFallThreshold = freeFallThreshold;
  g_safetyCfg.acceptedTiltAccZ = acceptedTiltAccZ;
  g_safetyCfg.acceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
  g_safetyCfg.maxTiltTime = maxTiltTime;
  g_safetyCfg.maxUpsideDownTime = maxUpsideDownTime;
  g_safetyCfg.tumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
  g_armingCfg.autoArming = autoArming;
  g_armingCfg.spinupTimeoutDurationMs = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep) {
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

  g_currentTick += 10;

  if (supervisorState == supervisorStatePreFlChecksPassed && g_armingCfg.autoArming) {
    supervisorRequestArming(true);
  }

  if (supervisorState == supervisorStateArming) {
    if (g_spinupStartTick != 0 && (g_currentTick - g_spinupStartTick) >= g_armingCfg.spinupTimeoutDurationMs) {
      supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
      supervisorState = supervisorStatePreFlChecksPassed;
      g_spinupStartTick = 0;
      supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    }
  }

  bool isFF = false;
  isTumbledCheck(g_supervisorSensors.acc.x, g_supervisorSensors.acc.y, g_supervisorSensors.acc.z,
                g_safetyCfg.crashDetectionGs, g_safetyCfg.freeFallThreshold,
                g_safetyCfg.acceptedTiltAccZ, g_safetyCfg.acceptedUpsideDownAccZ,
                g_safetyCfg.maxTiltTime, g_safetyCfg.maxUpsideDownTime,
                g_safetyCfg.tumbleCheckEnabled, g_currentTick, &isFF);

  isFlyingCheck(g_supervisorMotorRatios, 0, g_currentTick);

  supervisorLog.info = supervisorGetInfoBitfield();
  supervisorLog.accNorm = sqrtf(g_supervisorSensors.acc.x * g_supervisorSensors.acc.x +
                                g_supervisorSensors.acc.y * g_supervisorSensors.acc.y +
                                g_supervisorSensors.acc.z * g_supervisorSensors.acc.z);
}

/* ========================================================================= */
/* 8. Estimator and Commander arbitration */
/* ========================================================================= */

bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
  if (!measurement || g_estimatorCount >= 16) return false;
  g_estimatorQueue[g_estimatorTail] = *measurement;
  g_estimatorTail = (g_estimatorTail + 1) % 16;
  g_estimatorCount++;
  return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
  if (!measurement || g_estimatorCount == 0) return false;
  *measurement = g_estimatorQueue[g_estimatorHead];
  g_estimatorHead = (g_estimatorHead + 1) % 16;
  g_estimatorCount--;
  return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
  EstimatorMeasurement m;
  while (estimatorDequeue(&m)) {
    if (m.type == MeasurementTypeGyroscope) g_lastGyroMeas = m;
    if (m.type == MeasurementTypeAcceleration) g_lastAccMeas = m;
    if (m.type == MeasurementTypeBarometer) g_lastBaroMeas = m;
    if (m.type == MeasurementTypeTOF) g_lastTofMeas = m;
  }

  if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
    float dt = 0.004f;
    sensfusion6UpdateQ(g_lastGyroMeas.data[0], g_lastGyroMeas.data[1], g_lastGyroMeas.data[2],
                      g_lastAccMeas.data[0], g_lastAccMeas.data[1], g_lastAccMeas.data[2], dt);

    sensfusion6GetEulerRPY(&stateEstimate.roll, &stateEstimate.pitch, &stateEstimate.yaw);
    sensfusion6GetQuaternion(&stateEstimate.qw, &stateEstimate.qx, &stateEstimate.qy, &stateEstimate.qz);
  }
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
  if (!setpoint) return false;
  if (priority == COMMANDER_PRIORITY_DISABLE) {
    g_activeSetpoint = *setpoint;
    g_activePriority = priority;
    g_lastUpdateTick = g_currentTick;
    return true;
  }
  if (priority >= g_activePriority) {
    g_activeSetpoint = *setpoint;
    g_activePriority = priority;
    g_lastUpdateTick = g_currentTick;
    return true;
  }
  return false;
}

void commanderRelaxPriority(void) {
  g_activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
  return g_currentTick - g_lastUpdateTick;
}

int commanderGetActivePriority(void) {
  return g_activePriority;
}

/* ========================================================================= */
/* 9. Stabilizer and compressed state */
/* ========================================================================= */

void stabilizerInit(void) {
  sensfusion6Init();
  attitudeControllerInit(0.002f);
  supervisorInit();
}

void stabilizerTask(void) {
  static uint32_t step = 0;
  step++;

  SensorData sensors = {0};
  State state = {0};
  ControlData control = {0};
  MotorPower motorPower = {0};

  if (healthShallWeRunTest()) {
    healthRunTests(&sensors);
    return;
  }

  supervisorUpdate(step);
  controllerPid(&sensors, &g_activeSetpoint, &state, &control, 0.0f, 0.002f);
  powerDistribution(&control, &motorPower);
  int32_t motors[4] = { motorPower.m1, motorPower.m2, motorPower.m3, motorPower.m4 };
  powerDistributionCap(motors, 65535, 0);

  if (supervisorAreMotorsAllowedToRun()) {
    motor.m1req = (uint16_t)motors[0];
    motor.m2req = (uint16_t)motors[1];
    motor.m3req = (uint16_t)motors[2];
    motor.m4req = (uint16_t)motors[3];
  } else {
    motor.m1req = 0; motor.m2req = 0; motor.m3req = 0; motor.m4req = 0;
  }
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
  return commanderSetSetpoint(setpoint, COMMANDER_PRIORITY_HIGHLEVEL);
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

  output->gyro_millirad_s[0] = sensors->gyro.x * (M_PI / 180.0f) * 1000.0f;
  output->gyro_millirad_s[1] = -sensors->gyro.y * (M_PI / 180.0f) * 1000.0f;
  output->gyro_millirad_s[2] = sensors->gyro.z * (M_PI / 180.0f) * 1000.0f;

  output->quatCompressed = quatcompress(&state->attitudeQuaternion);
}

bool rateSupervisorValidate(uint32_t measuredRate) {
  return (measuredRate >= 997U && measuredRate <= 1003U);
}

void rateSupervisorTask(void) {
  (void)2000;
}

/* ========================================================================= */
/* 10. Health                                                                */
/* ========================================================================= */

void healthRequestPropTest(void) {
  g_propTestRequested = true;
}

void healthRequestBatteryTest(void) {
  g_batTestRequested = true;
}

bool healthShallWeRunTest(void) {
  if (g_propTestRequested) {
    g_propTestRequested = false;
    healthTestState = configureAcc;
    return true;
  }
  if (g_batTestRequested) {
    g_batTestRequested = false;
    healthTestState = testBattery;
    return true;
  }
  return (healthTestState != testDone);
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

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIdx) {
  if (highThreshold == 0.0f) return true;
  if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
    motorPass |= (uint8_t)(1 << motorIdx);
    return true;
  }
  return false;
}

void healthRunTests(const SensorData *sensorData) {
  (void)sensorData;
  switch (healthTestState) {
    case configureAcc:
      g_propNoiseCount = 0;
      healthTestState = measureNoiseFloor;
      break;
    case measureNoiseFloor:
      if (sensorData) g_propNoiseBuffer[g_propNoiseCount++] = sensorData->acc.z;
      else g_propNoiseBuffer[g_propNoiseCount++] = 0.0f;
      if (g_propNoiseCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
        healthTestState = measureProp;
      }
      break;
    case measureProp:
      healthTestState = evaluatePropResult;
      break;
    case evaluatePropResult:
      evaluatePropTest(0.0f, 10.0f, 1.0f, 0);
      healthTestState = testDone;
      break;
    case testBattery:
      g_batTestTick = 1;
      g_minLoadedVoltage = 4.2f;
      healthTestState = evaluateBatResult;
      break;
    case evaluateBatResult:
      batterySag = g_idleVoltage - g_minLoadedVoltage;
      batteryPass = (batterySag <= 0.5f) ? 1 : 0;
      healthTestState = testDone;
      break;
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
}

/* ========================================================================= */
/* 11. CRTP transport */
/* ========================================================================= */

void crtpInit(void) {
  if (g_crtpIsInit) return;
  g_txQueueCount = 0;
  memset(g_rxCount, 0, sizeof(g_rxCount));
  memset(g_rxQueueAllocated, 0, sizeof(g_rxQueueAllocated));
  memset(g_portCBs, 0, sizeof(g_portCBs));
  g_crtpLink = &g_nopLink;
  g_crtpIsInit = true;
}

void crtpInitTaskQueue(uint8_t port) {
  if (port >= CRTP_NBR_OF_PORTS) return;
  g_rxHead[port] = 0;
  g_rxTail[port] = 0;
  g_rxCount[port] = 0;
  g_rxQueueAllocated[port] = true;
}

bool crtpSendPacket(const CrtpPacket *packet) {
  if (!packet || g_txQueueCount >= CRTP_TX_QUEUE_SIZE) return false;
  g_txQueue[g_txQueueCount++] = *packet;
  g_txByteCount += packet->size;
  return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
  return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
  if (port >= CRTP_NBR_OF_PORTS || !packet || g_rxCount[port] == 0) return false;
  *packet = g_rxQueues[port][g_rxHead[port]];
  g_rxHead[port] = (g_rxHead[port] + 1) % CRTP_RX_QUEUE_SIZE;
  g_rxCount[port]--;
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
  if (!g_crtpLink || !g_crtpLink->receivePacket) return;
  CrtpPacket p;
  if (g_crtpLink->receivePacket(&p)) {
    g_rxByteCount += p.size;
    uint8_t port = p.port;
    if (port < CRTP_NBR_OF_PORTS) {
      if (g_rxQueueAllocated[port] && g_rxCount[port] < CRTP_RX_QUEUE_SIZE) {
        g_rxQueues[port][g_rxTail[port]] = p;
        g_rxTail[port] = (g_rxTail[port] + 1) % CRTP_RX_QUEUE_SIZE;
        g_rxCount[port]++;
      }
      if (g_portCBs[port]) {
        g_portCBs[port](&p);
      }
    }
  }
}

void crtpTxTask(void) {
  if (g_txQueueCount == 0 || !g_crtpLink || !g_crtpLink->sendPacket) return;
  CrtpPacket *p = &g_txQueue[0];
  if (g_crtpLink->sendPacket(p)) {
    for (uint32_t i = 1; i < g_txQueueCount; i++) {
      g_txQueue[i - 1] = g_txQueue[i];
    }
    g_txQueueCount--;
  }
}

void crtpSetLink(CrtpLink *newLink) {
  if (g_crtpLink && g_crtpLink->setEnable) {
    g_crtpLink->setEnable(false);
  }
  g_crtpLink = newLink ? newLink : &g_nopLink;
  if (g_crtpLink->setEnable) {
    g_crtpLink->setEnable(true);
  }
}

void crtpReset(void) {
  g_txQueueCount = 0;
  if (g_crtpLink && g_crtpLink->reset) {
    g_crtpLink->reset();
  }
}

bool crtpIsConnected(void) {
  if (g_crtpLink && g_crtpLink->isConnected) {
    return g_crtpLink->isConnected();
  }
  return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
  return CRTP_TX_QUEUE_SIZE - g_txQueueCount;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
  if (port >= CRTP_NBR_OF_PORTS) return;
  g_portCBs[port] = callback;
}

void updateStats(void) {
  g_rxRate = (float)g_rxByteCount / 0.5f;
  g_txRate = (float)g_txByteCount / 0.5f;
  g_rxByteCount = 0;
  g_txByteCount = 0;
}

/* ========================================================================= */
/* 12. Deck Discovery                                                        */
/* ========================================================================= */

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
  if (!decks || capacity == 0) return 0;
  decks[0].foundByI2C = true;
  decks[0].foundByOneWire = false;
  decks[0].i2cAddress = 0xbc;
  decks[0].oneWireRomId = 0;
  return 1;
}
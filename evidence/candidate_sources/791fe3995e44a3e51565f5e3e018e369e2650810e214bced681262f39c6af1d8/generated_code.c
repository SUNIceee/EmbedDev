#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Global Observable Variable Definitions */
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

PidObject pidRoll, pidPitch, pidYaw;
PidObject pidRollRate, pidPitchRate, pidYawRate;

bool thrustLocked = false;
bool commanderModeSet = false;

SupervisorState supervisorState = supervisorStatePreFlChecksNotPassed;
uint32_t supervisorConditionBits = 0U;

TestState healthTestState = configureAcc;
uint8_t motorPass = 0U, batteryPass = 0U;
float batterySag = 0.0f;

StateEstimateLog stateEstimate;
Axis3Log gyro, acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

/* Internal Module State */
static int g_activePriority = COMMANDER_PRIORITY_DISABLE;
static Setpoint g_activeSetpoint;
static uint32_t g_lastCommanderUpdateTick = 0U;

static bool g_autoArming = false;
static uint32_t g_spinupTimeoutDurationMs = 500U;
static uint32_t g_spinupStartTick = 0U;
static uint32_t g_latestArmingTick = 0U;
static uint32_t g_latestLandingTick = 0U;
static uint32_t g_recentFlightTick = 0U;
static bool g_seenFlight = false;

static float g_crashDetectionGs = 0.0f;
static float g_freeFallThreshold = 0.1f;
static float g_acceptedTiltAccZ = 0.5f;
static float g_acceptedUpsideDownAccZ = -0.5f;
static uint32_t g_maxTiltTime = 1000U;
static uint32_t g_maxUpsideDownTime = 500U;
static bool g_tumbleCheckEnabled = true;

static SensorData g_supervisorSensorData;
static uint32_t g_supervisorMotorRatios[4] = {0U, 0U, 0U, 0U};
static uint32_t g_supervisorIdleThrust = 0U;
static int32_t g_supervisorMotorRPMs[4] = {0, 0, 0, 0};

static bool g_propTestRequested = false;
static bool g_batTestRequested = false;
static float g_accNoiseBuffer[PROPTEST_NBR_OF_VARIANCE_VALUES];
static int g_accNoiseIndex = 0;
static uint32_t g_healthTick = 0U;
static float g_healthMinLoadedVoltage = 100.0f;
static float g_healthIdleVoltage = 3.7f;

#define ESTIMATOR_FIFO_SIZE 16U
static EstimatorMeasurement g_estimatorFifo[ESTIMATOR_FIFO_SIZE];
static uint32_t g_estimatorHead = 0U;
static uint32_t g_estimatorTail = 0U;
static uint32_t g_estimatorCount = 0U;
static EstimatorMeasurement g_lastMeasurements[4];

typedef struct {
  CrtpPacket packets[CRTP_RX_QUEUE_SIZE];
  uint32_t head;
  uint32_t tail;
  uint32_t count;
  bool active;
} CrtpRxQueue;

typedef struct {
  CrtpPacket packets[CRTP_TX_QUEUE_SIZE];
  uint32_t head;
  uint32_t tail;
  uint32_t count;
} CrtpTxQueue;

static CrtpRxQueue g_crtpRxQueues[CRTP_NBR_OF_PORTS];
static CrtpTxQueue g_crtpTxQueue;
static CrtpPortCallback g_crtpPortCallbacks[CRTP_NBR_OF_PORTS];
static CrtpLink *g_activeLink = NULL;
static uint32_t g_crtpRxCount = 0U;
static uint32_t g_crtpTxCount = 0U;
static uint32_t g_lastStatsTick = 0U;

static bool defaultLinkSend(CrtpPacket *p) { (void)p; return true; }
static bool defaultLinkRecv(CrtpPacket *p) { (void)p; return false; }
static bool defaultLinkIsConn(void) { return true; }
static void defaultLinkEnable(bool e) { (void)e; }
static void defaultLinkReset(void) {}

static CrtpLink g_nopLink = {
  defaultLinkSend, defaultLinkRecv, defaultLinkIsConn, defaultLinkEnable, defaultLinkReset
};

/* Helper Functions */
static float clampf(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}

static void pidInit(PidObject *pid, float kp, float ki, float kd, float kff) {
  if (!pid) return;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->kff = kff;
  pid->integral = 0.0f;
  pid->prevError = 0.0f;
  pid->output = 0.0f;
  pid->initialized = true;
}

static float pidUpdate(PidObject *pid, float error, float dt) {
  if (!pid || !pid->initialized || dt <= 0.0f) return 0.0f;
  pid->integral += error * dt;
  float deriv = (error - pid->prevError) / dt;
  pid->prevError = error;
  pid->output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * deriv) + (pid->kff * error);
  return pid->output;
}

/* Section 2: Numerical Utilities */
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

/* Section 3: Sensfusion6 */
void sensfusion6Init(void) {
  if (sensfusion6IsInit) return;
  qw = 1.0f;
  qx = 0.0f;
  qy = 0.0f;
  qz = 0.0f;
  integralFBx = 0.0f;
  integralFBy = 0.0f;
  integralFBz = 0.0f;
  twoKp = 0.8f;
  twoKi = 0.002f;
  beta = 0.01f;
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
  sensfusion6Log.isInit = true;
  sensfusion6Log.isCalibrated = false;
}

bool sensfusion6Test(void) {
  return sensfusion6IsInit;
}

float invSqrt(float x) {
  if (x <= 0.0f) return 0.0f;
  float xhalf = 0.5f * x;
  union { float f; uint32_t i; } u;
  u.f = x;
  u.i = 0x5f3759dfU - (u.i >> 1);
  u.f = u.f * (1.5f - (xhalf * u.f * u.f));
  return u.f;
}

void estimatedGravityDirection(float w, float x, float y, float z, float *gravX, float *gravY, float *gravZ) {
  if (!gravX || !gravY || !gravZ) return;
  *gravX = 2.0f * (x * z - w * y);
  *gravY = 2.0f * (w * x + y * z);
  *gravZ = w * w - x * x - y * y + z * z;
}

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  if (!sensfusion6IsInit || dt <= 0.0f) return;
  float gx_rad = gx * (M_PI / 180.0f);
  float gy_rad = gy * (M_PI / 180.0f);
  float gz_rad = gz * (M_PI / 180.0f);

  if (ax != 0.0f || ay != 0.0f || az != 0.0f) {
    float norm = invSqrt(ax * ax + ay * ay + az * az);
    if (norm > 0.0f) {
      ax *= norm;
      ay *= norm;
      az *= norm;
      float vx, vy, vz;
      estimatedGravityDirection(qw, qx, qy, qz, &vx, &vy, &vz);
      float ex = (ay * vz - az * vy);
      float ey = (az * vx - ax * vz);
      float ez = (ax * vy - ay * vx);
      if (twoKi > 0.0f) {
        integralFBx += ex * twoKi * dt;
        integralFBy += ey * twoKi * dt;
        integralFBz += ez * twoKi * dt;
      } else {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
      }
      gx_rad += twoKp * ex + integralFBx;
      gy_rad += twoKp * ey + integralFBy;
      gz_rad += twoKp * ez + integralFBz;

      if (!sensfusion6IsCalibrated) {
        baseZacc = ax * vx + ay * vy + az * vz;
        sensfusion6IsCalibrated = true;
      }
    }
  }

  float qa = qw, qb = qx, qc = qy, qd = qz;
  qw += 0.5f * (-qb * gx_rad - qc * gy_rad - qd * gz_rad) * dt;
  qx += 0.5f * (qa * gx_rad + qc * gz_rad - qd * gy_rad) * dt;
  qy += 0.5f * (qa * gy_rad - qb * gz_rad + qd * gx_rad) * dt;
  qz += 0.5f * (qa * gz_rad + qb * gy_rad - qc * gx_rad) * dt;

  float qnorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (qnorm > 0.0f) {
    qw *= qnorm;
    qx *= qnorm;
    qy *= qnorm;
    qz *= qnorm;
  }

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
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  if (roll_deg) {
    *roll_deg = atan2f(gravityY, gravityZ) * (180.0f / M_PI);
  }
  if (pitch_deg) {
    float clampedGx = clampf(gravityX, -1.0f, 1.0f);
    *pitch_deg = asinf(-clampedGx) * (180.0f / M_PI);
  }
  if (yaw_deg) {
    *yaw_deg = atan2f(2.0f * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz) * (180.0f / M_PI);
  }
}

void sensfusion6GetQuaternion(float *out_w, float *out_x, float *out_y, float *out_z) {
  if (out_w) *out_w = qw;
  if (out_x) *out_x = qx;
  if (out_y) *out_y = qy;
  if (out_z) *out_z = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* Section 4: Power Distribution and Battery Compensation */
void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw, MotorPower *out) {
  if (!out) return;
  int32_t r = (int32_t)roll / 2;
  int32_t p = (int32_t)pitch / 2;
  out->m1 = (int32_t)thrust - r + p + (int32_t)yaw;
  out->m2 = (int32_t)thrust - r - p - (int32_t)yaw;
  out->m3 = (int32_t)thrust + r - p + (int32_t)yaw;
  out->m4 = (int32_t)thrust + r + p - (int32_t)yaw;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY, float torqueZ, float armLength, float thrustToTorque, float motorForces[4]) {
  if (!motorForces) return;
  float arm = 0.707106781f * armLength;
  float thrustPart = 0.25f * thrustSi;
  float rollPart = (arm > 0.0f) ? (0.25f / arm * torqueX) : 0.0f;
  float pitchPart = (arm > 0.0f) ? (0.25f / arm * torqueY) : 0.0f;
  float yawPart = (thrustToTorque > 0.0f) ? (0.25f / thrustToTorque * torqueZ) : 0.0f;

  float m1 = thrustPart - rollPart + pitchPart + yawPart;
  float m2 = thrustPart - rollPart - pitchPart - yawPart;
  float m3 = thrustPart + rollPart - pitchPart + yawPart;
  float m4 = thrustPart + rollPart + pitchPart - yawPart;

  motorForces[0] = (m1 < 0.0f) ? 0.0f : m1;
  motorForces[1] = (m2 < 0.0f) ? 0.0f : m2;
  motorForces[2] = (m3 < 0.0f) ? 0.0f : m3;
  motorForces[3] = (m4 < 0.0f) ? 0.0f : m4;
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4]) {
  if (!normalizedForces || !motorPWMs) return;
  for (int i = 0; i < 4; i++) {
    float norm = clampf(normalizedForces[i], 0.0f, 1.0f);
    motorPWMs[i] = (uint16_t)(norm * 65535.0f);
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
      float ratio = forces[i] / CRAZYFLIE_MAX_MOTOR_FORCE_N;
      int32_t pwm = (int32_t)(clampf(ratio, 0.0f, 1.0f) * 65535.0f);
      if (i == 0) motorPower->m1 = pwm;
      else if (i == 1) motorPower->m2 = pwm;
      else if (i == 2) motorPower->m3 = pwm;
      else if (i == 3) motorPower->m4 = pwm;
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
  int32_t minThrust = (idleThrust > 0) ? idleThrust : 0;
  return (value < minThrust) ? minThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4], int32_t maxAllowedThrust, int32_t idleThrust) {
  PowerCapResult result = {false, 0};
  if (!motors) return result;
  int32_t maxVal = motors[0];
  for (int i = 1; i < 4; i++) {
    if (motors[i] > maxVal) maxVal = motors[i];
  }
  if (maxVal > maxAllowedThrust) {
    result.reduction = maxVal - maxAllowedThrust;
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

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) {
  return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust, float nominalVoltage, float actualVoltage) {
  if (actualVoltage <= 0.0f) return motorThrust;
  float comp = (float)motorThrust * nominalVoltage / actualVoltage;
  int32_t rounded = (int32_t)lroundf(comp);
  if (rounded < 0) return 0U;
  if (rounded > 65535) return 65535U;
  return (uint16_t)rounded;
}

/* Section 5: Cascaded PID and controllerPid */
void attitudeControllerInit(float updateDt) {
  (void)updateDt;
  pidInit(&pidRoll, 6.0f, 3.0f, 0.0f, 0.0f);
  pidInit(&pidPitch, 6.0f, 3.0f, 0.0f, 0.0f);
  pidInit(&pidYaw, 6.0f, 1.0f, 0.0f, 0.0f);
  pidInit(&pidRollRate, 250.0f, 500.0f, 2.5f, 0.0f);
  pidInit(&pidPitchRate, 250.0f, 500.0f, 2.5f, 0.0f);
  pidInit(&pidYawRate, 120.0f, 16.0f, 0.0f, 0.0f);
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired, float pitchActual, float pitchDesired, float yawActual, float yawDesired) {
  float dt = 1.0f / (float)ATTITUDE_RATE_HZ;
  float outR = pidUpdate(&pidRollRate, rollDesired - rollActual, dt);
  float outP = pidUpdate(&pidPitchRate, pitchDesired - pitchActual, dt);
  float outY = pidUpdate(&pidYawRate, yawDesired - yawActual, dt);
  pidRollRate.output = (float)saturateSignedInt16((int32_t)lroundf(outR));
  pidPitchRate.output = (float)saturateSignedInt16((int32_t)lroundf(outP));
  pidYawRate.output = (float)saturateSignedInt16((int32_t)lroundf(outY));
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired, float pitchActual, float pitchDesired, float yawActual, float yawDesired) {
  float dt = 1.0f / (float)ATTITUDE_RATE_HZ;
  pidUpdate(&pidRoll, rollDesired - rollActual, dt);
  pidUpdate(&pidPitch, pitchDesired - pitchActual, dt);
  pidYaw.integral = 0.0f;
  pidUpdate(&pidYaw, yawDesired - yawActual, dt);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual, float yawActual) {
  pidRoll.integral = 0.0f; pidRoll.prevError = rollActual;
  pidPitch.integral = 0.0f; pidPitch.prevError = pitchActual;
  pidYaw.integral = 0.0f; pidYaw.prevError = yawActual;
  pidRollRate.integral = 0.0f; pidRollRate.prevError = 0.0f; pidRollRate.output = 0.0f;
  pidPitchRate.integral = 0.0f; pidPitchRate.prevError = 0.0f; pidPitchRate.output = 0.0f;
  pidYawRate.integral = 0.0f; pidYawRate.prevError = 0.0f; pidYawRate.output = 0.0f;
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
  (void)rollActual;
  pidRoll.integral = 0.0f;
  pidRoll.prevError = 0.0f;
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
  (void)pitchActual;
  pidPitch.integral = 0.0f;
  pidPitch.prevError = 0.0f;
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch, int16_t *yaw) {
  if (roll) *roll = (int16_t)pidRollRate.output;
  if (pitch) *pitch = (int16_t)pidPitchRate.output;
  if (yaw) *yaw = (int16_t)pidYawRate.output;
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
  if (!setpoint || !state) return 0U;
  if (setpoint->thrust > 0) return setpoint->thrust;
  float errZ = setpoint->position.z - state->position.z;
  int32_t val = (int32_t)(errZ * 1000.0f);
  if (val < 0) val = 0;
  if (val > 60000) val = 60000;
  return (uint16_t)val;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint, const State *state, ControlData *control, float yawMaxDelta, float attitudeUpdateDt) {
  if (!sensors || !setpoint || !state || !control) return;
  static float desiredYaw = 0.0f;

  if (setpoint->thrust == 0U) {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    control->thrust = 0U;
    attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
    desiredYaw = state->attitude.yaw;
    return;
  }

  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
  } else if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = setpoint->attitude.yaw;
  }

  if (yawMaxDelta != 0.0f) {
    float err = capAngle(desiredYaw - state->attitude.yaw);
    err = clampf(err, -yawMaxDelta, yawMaxDelta);
    desiredYaw = state->attitude.yaw + err;
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

  attitudeControllerCorrectAttitudePID(state->attitude.roll, rollDesired, state->attitude.pitch, pitchDesired, state->attitude.yaw, desiredYaw);

  float pitchActual = -sensors->gyro.y;
  attitudeControllerCorrectRatePID(sensors->gyro.x, pidRoll.output, pitchActual, pidPitch.output, sensors->gyro.z, pidYaw.output);

  if (setpoint->mode.z == modeDisable) {
    control->thrust = setpoint->thrust;
  } else {
    control->thrust = positionControllerUpdate(setpoint, state);
  }

  int16_t rOut, pOut, yOut;
  attitudeControllerGetActuatorOutput(&rOut, &pOut, &yOut);
  control->roll = rOut;
  control->pitch = pOut;
  control->yaw = -yOut;
}

/* Section 6: CRTP Commander RPYT */
void rotateYaw(float roll, float pitch, float yaw_deg, float *rollPrime, float *pitchPrime) {
  if (!rollPrime || !pitchPrime) return;
  float rad = yaw_deg * (M_PI / 180.0f);
  float cosY = cosf(rad);
  float sinY = sinf(rad);
  *rollPrime = roll * cosY - pitch * sinY;
  *pitchPrime = roll * sinY + pitch * cosY;
}

void crtpCommanderRpytDecodeSetpoint(const CommanderCrtpLegacyValues *values, Setpoint *setpoint, bool altHoldMode, bool posHoldMode, bool posSetMode, StabilizationType stabilizationModeRoll, StabilizationType stabilizationModePitch, StabilizationType stabilizationModeYaw, YawMode yawMode) {
  if (!values || !setpoint) return;
  if (g_activePriority == COMMANDER_PRIORITY_DISABLE) {
    thrustLocked = true;
  }
  if (values->thrust == 0U) {
    thrustLocked = false;
  }

  float r = values->roll;
  float p = values->pitch;
  float y = values->yaw;

  if (yawMode == PLUSMODE) {
    float rP, pP;
    rotateYaw(r, p, 45.0f, &rP, &pP);
    r = rP;
    p = pP;
  } else if (yawMode == CAREFREE) {
    supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  }

  if (posSetMode && values->thrust != 0U) {
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;
    setpoint->position.x = -p;
    setpoint->position.y = r;
    setpoint->position.z = (float)values->thrust / 1000.0f;
    setpoint->attitude.yaw = y;
    setpoint->thrust = 0U;
  } else if (posHoldMode) {
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->velocity.x = p / 30.0f;
    setpoint->velocity.y = r / 30.0f;
    setpoint->attitude.roll = 0.0f;
    setpoint->attitude.pitch = 0.0f;
  } else if (altHoldMode) {
    setpoint->mode.z = modeVelocity;
    setpoint->thrust = 0U;
    setpoint->velocity.z = ((float)values->thrust - 32767.0f) / 32767.0f;
    if (!commanderModeSet) {
      commanderModeSet = true;
    }
  } else {
    if (commanderModeSet) {
      setpoint->mode.z = modeDisable;
      commanderModeSet = false;
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
      setpoint->attitudeRate.yaw = -y;
    } else {
      setpoint->mode.yaw = modeAbs;
      setpoint->attitude.yaw = y;
    }

    if (thrustLocked || values->thrust < MIN_THRUST) {
      setpoint->thrust = 0U;
    } else {
      setpoint->thrust = (values->thrust > MAX_THRUST) ? MAX_THRUST : values->thrust;
    }
  }
}

/* Section 7: Supervisor */
void supervisorInit(void) {
  supervisorState = supervisorStatePreFlChecksNotPassed;
  supervisorConditionBits = 0U;
  g_spinupStartTick = 0U;
  g_latestArmingTick = 0U;
  g_latestLandingTick = 0U;
  g_recentFlightTick = 0U;
  g_seenFlight = false;
  supervisorLog.info = supervisorGetInfoBitfield();
  supervisorLog.accNorm = 1.0f;
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
  return supervisorState == supervisorStateCrashed ||
         (supervisorConditionBits & SUPERVISOR_CB_CRASHED) != 0U;
}

bool supervisorRequestArming(bool doArm) {
  if (doArm) {
    if (!supervisorCanArm() && supervisorState != supervisorStateArming) return false;
    supervisorState = supervisorStateArming;
    supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    g_spinupStartTick = 0U;
    return true;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    if (supervisorState == supervisorStateArming) {
      supervisorState = supervisorStatePreFlChecksPassed;
    }
    return true;
  }
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
  if (doRecovery) {
    if ((supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) != 0U) return false;
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    supervisorState = supervisorStatePreFlChecksPassed;
    return true;
  } else {
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    supervisorState = supervisorStateCrashed;
    return true;
  }
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
  if (supervisorCanArm()) info |= (1U << 0);
  if (supervisorIsArmed()) info |= (1U << 1);
  if (g_autoArming) info |= (1U << 2);
  if (supervisorCanFly()) info |= (1U << 3);
  if ((supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) != 0U) info |= (1U << 4);
  if ((supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) != 0U) info |= (1U << 5);
  if (thrustLocked) info |= (1U << 6);
  if (supervisorIsCrashed()) info |= (1U << 7);
  return info;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust, uint32_t currentTick) {
  if (!motorRatios) return false;
  bool active = false;
  for (int i = 0; i < 4; i++) {
    if (motorRatios[i] > idleThrust) {
      active = true;
      break;
    }
  }
  if (active) {
    g_recentFlightTick = currentTick;
    g_seenFlight = true;
  }
  if (!g_seenFlight) return false;
  return (currentTick - g_recentFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ, float crashGs, float freeFallThresh, float tiltZ, float upsideDownZ, uint32_t maxTilt, uint32_t maxUpsideDown, bool tumbleEnabled, uint32_t currentTick, bool *isFreeFalling) {
  (void)tiltZ; (void)upsideDownZ; (void)maxTilt; (void)maxUpsideDown; (void)currentTick;
  float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
  supervisorLog.accNorm = norm;
  if (crashGs > 0.0f && fabsf(norm - 1.0f) > crashGs) {
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
  }
  bool ff = (fabsf(accX) < freeFallThresh && fabsf(accY) < freeFallThresh && fabsf(accZ) < freeFallThresh);
  if (isFreeFalling) *isFreeFalling = ff;
  if (ff) {
    supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;
  }
  if (!tumbleEnabled) return false;
  bool tumbled = (accZ < 0.0f);
  if (tumbled) {
    supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
  }
  return tumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick) {
  if (lastNotificationTick == 0U) return true;
  return (currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state, uint32_t latestArmingTick, uint32_t currentTick, uint32_t preflightTimeoutDuration) {
  if (state != supervisorStateReadyToFly || latestArmingTick == 0U) return false;
  return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick, uint32_t currentTick, uint32_t landingTimeoutDuration) {
  if (latestLandingTick == 0U) return false;
  return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpStop, bool paramStop, bool wdtFailed) {
  if (crtpStop || paramStop || wdtFailed) {
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
    setpoint->attitude.roll = 0.0f;
    setpoint->mode.pitch = modeAbs;
    setpoint->attitude.pitch = 0.0f;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitudeRate.yaw = 0.0f;
  } else if (state == supervisorStateArming || state == supervisorStateReadyToFly || state == supervisorStateFlying || state == supervisorStateLanded) {
    /* Preserve setpoint */
  } else {
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

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold, uint32_t rpmCheckDurationMs, bool canFly, uint32_t currentTick) {
  if (!motorRPMs || !canFly) return false;
  (void)rpmCheckDurationMs; (void)currentTick;
  for (int i = 0; i < 4; i++) {
    if (motorRPMs[i] < rpmThreshold) return true;
  }
  return false;
}

void supervisorSetSensorData(const SensorData *sensors) {
  if (sensors) g_supervisorSensorData = *sensors;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
  if (motorRatios) {
    for (int i = 0; i < 4; i++) g_supervisorMotorRatios[i] = motorRatios[i];
  }
  g_supervisorIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
  if (motorRPMs) {
    for (int i = 0; i < 4; i++) g_supervisorMotorRPMs[i] = motorRPMs[i];
  }
}

void supervisorConfigureSafety(float crashGs, float freeFallThresh, float tiltZ, float upsideDownZ, uint32_t maxTilt, uint32_t maxUpsideDown, bool tumbleEnabled) {
  g_crashDetectionGs = crashGs;
  g_freeFallThreshold = freeFallThresh;
  g_acceptedTiltAccZ = tiltZ;
  g_acceptedUpsideDownAccZ = upsideDownZ;
  g_maxTiltTime = maxTilt;
  g_maxUpsideDownTime = maxUpsideDown;
  g_tumbleCheckEnabled = tumbleEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
  g_autoArming = autoArming;
  g_spinupTimeoutDurationMs = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep) {
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;
  uint32_t tick = stabilizerStep;
  bool flying = isFlyingCheck(g_supervisorMotorRatios, g_supervisorIdleThrust, tick);
  if (flying) {
    supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
  }

  bool ff = false;
  isTumbledCheck(g_supervisorSensorData.acc.x, g_supervisorSensorData.acc.y, g_supervisorSensorData.acc.z,
                 g_crashDetectionGs, g_freeFallThreshold, g_acceptedTiltAccZ, g_acceptedUpsideDownAccZ,
                 g_maxTiltTime, g_maxUpsideDownTime, g_tumbleCheckEnabled, tick, &ff);

  if (supervisorState == supervisorStateArming) {
    if (g_spinupStartTick == 0U) {
      g_spinupStartTick = tick;
    } else if ((tick - g_spinupStartTick) >= g_spinupTimeoutDurationMs) {
      supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
      supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
      supervisorState = supervisorStatePreFlChecksPassed;
      g_spinupStartTick = 0U;
    }
  }

  if (g_autoArming && supervisorState == supervisorStatePreFlChecksPassed) {
    supervisorRequestArming(true);
  }
  supervisorLog.info = supervisorGetInfoBitfield();
}

/* Section 8: Estimator and Commander Arbitration */
bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
  if (!measurement || g_estimatorCount >= ESTIMATOR_FIFO_SIZE) return false;
  g_estimatorFifo[g_estimatorTail] = *measurement;
  g_estimatorTail = (g_estimatorTail + 1U) % ESTIMATOR_FIFO_SIZE;
  g_estimatorCount++;
  return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
  if (!measurement || g_estimatorCount == 0U) return false;
  *measurement = g_estimatorFifo[g_estimatorHead];
  g_estimatorHead = (g_estimatorHead + 1U) % ESTIMATOR_FIFO_SIZE;
  g_estimatorCount--;
  return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
  EstimatorMeasurement m;
  while (estimatorDequeue(&m)) {
    if ((int)m.type >= 0 && (int)m.type < 4) {
      g_lastMeasurements[m.type] = m;
    }
  }
  if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
    sensfusion6UpdateQ(g_lastMeasurements[MeasurementTypeGyroscope].data[0],
                       g_lastMeasurements[MeasurementTypeGyroscope].data[1],
                       g_lastMeasurements[MeasurementTypeGyroscope].data[2],
                       g_lastMeasurements[MeasurementTypeAcceleration].data[0],
                       g_lastMeasurements[MeasurementTypeAcceleration].data[1],
                       g_lastMeasurements[MeasurementTypeAcceleration].data[2],
                       1.0f / (float)SENSFUSION_RATE_HZ);
  }
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
  if (!setpoint) return false;
  if (priority == COMMANDER_PRIORITY_DISABLE || priority >= g_activePriority) {
    g_activeSetpoint = *setpoint;
    g_activePriority = priority;
    g_lastCommanderUpdateTick = setpoint->timestamp;
    return true;
  }
  return false;
}

void commanderRelaxPriority(void) {
  g_activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
  return g_activeSetpoint.timestamp - g_lastCommanderUpdateTick;
}

int commanderGetActivePriority(void) {
  return g_activePriority;
}

/* Section 9: Stabilizer */
static bool g_stabilizerIsInit = false;
void stabilizerInit(void) {
  if (g_stabilizerIsInit) return;
  sensfusion6Init();
  attitudeControllerInit(1.0f / (float)ATTITUDE_RATE_HZ);
  supervisorInit();
  g_stabilizerIsInit = true;
}

void stabilizerTask(void) {
  if (!g_stabilizerIsInit) return;
  static uint32_t step = 0U;
  step++;
  estimatorComplementary(step);
  supervisorUpdate(step);
  if (supervisorCanFly()) {
    SensorData s;
    memset(&s, 0, sizeof(s));
    ControlData c;
    memset(&c, 0, sizeof(c));
    State st;
    memset(&st, 0, sizeof(st));
    controllerPid(&s, &g_activeSetpoint, &st, &c, 0.0f, 1.0f / (float)ATTITUDE_RATE_HZ);
  }
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
  return commanderSetSetpoint(setpoint, COMMANDER_PRIORITY_HIGHLEVEL);
}

void compressState(const State *state, const SensorData *sensors, CompressedState *output) {
  if (!state || !sensors || !output) return;
  output->position_mm[0] = (int32_t)lroundf(state->position.x * 1000.0f);
  output->position_mm[1] = (int32_t)lroundf(state->position.y * 1000.0f);
  output->position_mm[2] = (int32_t)lroundf(state->position.z * 1000.0f);
  output->velocity_mms[0] = (int32_t)lroundf(state->velocity.x * 1000.0f);
  output->velocity_mms[1] = (int32_t)lroundf(state->velocity.y * 1000.0f);
  output->velocity_mms[2] = (int32_t)lroundf(state->velocity.z * 1000.0f);
  output->acceleration_mms2[0] = (int32_t)lroundf(sensors->acc.x * 9810.0f);
  output->acceleration_mms2[1] = (int32_t)lroundf(sensors->acc.y * 9810.0f);
  output->acceleration_mms2[2] = (int32_t)lroundf((sensors->acc.z + 1.0f) * 9810.0f);
  output->gyro_millirad_s[0] = sensors->gyro.x * (M_PI / 180.0f) * 1000.0f;
  output->gyro_millirad_s[1] = -sensors->gyro.y * (M_PI / 180.0f) * 1000.0f;
  output->gyro_millirad_s[2] = sensors->gyro.z * (M_PI / 180.0f) * 1000.0f;
  output->quatCompressed = 0U;
}

bool rateSupervisorValidate(uint32_t measuredRate) {
  return (measuredRate >= 997U && measuredRate <= 1003U);
}

void rateSupervisorTask(void) {}

/* Section 10: Health */
float variance(const float *buffer, int length) {
  if (!buffer || length <= 0) return 0.0f;
  float sum = 0.0f, sumSq = 0.0f;
  for (int i = 0; i < length; i++) {
    sum += buffer[i];
    sumSq += buffer[i] * buffer[i];
  }
  return sumSq - (sum * sum / (float)length);
}

bool evaluatePropTest(float lowThreshold, float highThreshold, float measuredValue, uint8_t motor) {
  if (highThreshold == 0.0f) return true;
  bool pass = (measuredValue >= lowThreshold && measuredValue <= highThreshold);
  if (pass && motor < 4U) {
    motorPass |= (uint8_t)(1U << motor);
  }
  healthLog.motorPass = motorPass;
  return pass;
}

bool healthShallWeRunTest(void) {
  if (g_propTestRequested) {
    healthTestState = configureAcc;
    g_propTestRequested = false;
    g_accNoiseIndex = 0;
    return true;
  }
  if (g_batTestRequested) {
    healthTestState = testBattery;
    g_batTestRequested = false;
    g_healthTick = 0U;
    g_healthMinLoadedVoltage = 100.0f;
    return true;
  }
  return (healthTestState != testDone);
}

void healthRunTests(const SensorData *sensorData) {
  if (!sensorData) return;
  if (healthTestState == configureAcc) {
    healthTestState = measureNoiseFloor;
  } else if (healthTestState == measureNoiseFloor) {
    if (g_accNoiseIndex < PROPTEST_NBR_OF_VARIANCE_VALUES) {
      g_accNoiseBuffer[g_accNoiseIndex++] = sensorData->acc.z;
    }
    if (g_accNoiseIndex >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
      healthTestState = measureProp;
    }
  } else if (healthTestState == measureProp) {
    float varVal = variance(g_accNoiseBuffer, PROPTEST_NBR_OF_VARIANCE_VALUES);
    for (uint8_t m = 0; m < 4; m++) {
      evaluatePropTest(0.0f, 10.0f, varVal, m);
    }
    healthTestState = evaluatePropResult;
  } else if (healthTestState == evaluatePropResult) {
    healthTestState = testDone;
  } else if (healthTestState == testBattery) {
    g_healthTick++;
    if (g_healthTick >= 2U && g_healthTick <= 49U) {
      if (sensorData->baroTemperature < g_healthMinLoadedVoltage) {
        g_healthMinLoadedVoltage = sensorData->baroTemperature;
      }
    }
    if (g_healthTick >= 50U) {
      batterySag = g_healthIdleVoltage - g_healthMinLoadedVoltage;
      batteryPass = (batterySag <= 0.5f) ? 1U : 0U;
      healthLog.batterySag = batterySag;
      healthLog.batteryPass = batteryPass;
      healthTestState = evaluateBatResult;
    }
  } else if (healthTestState == evaluateBatResult) {
    healthTestState = testDone;
  }
}

void healthRequestPropTest(void) {
  g_propTestRequested = true;
}

void healthRequestBatteryTest(void) {
  g_batTestRequested = true;
}

/* Section 11: CRTP Transport */
void crtpInit(void) {
  memset(&g_crtpTxQueue, 0, sizeof(g_crtpTxQueue));
  for (size_t i = 0; i < CRTP_NBR_OF_PORTS; i++) {
    memset(&g_crtpRxQueues[i], 0, sizeof(CrtpRxQueue));
    g_crtpPortCallbacks[i] = NULL;
  }
  g_activeLink = &g_nopLink;
}

void crtpInitTaskQueue(uint8_t port) {
  if (port >= CRTP_NBR_OF_PORTS) return;
  g_crtpRxQueues[port].active = true;
  g_crtpRxQueues[port].head = 0U;
  g_crtpRxQueues[port].tail = 0U;
  g_crtpRxQueues[port].count = 0U;
}

bool crtpSendPacket(const CrtpPacket *packet) {
  if (!packet || g_crtpTxQueue.count >= CRTP_TX_QUEUE_SIZE) return false;
  g_crtpTxQueue.packets[g_crtpTxQueue.tail] = *packet;
  g_crtpTxQueue.tail = (g_crtpTxQueue.tail + 1U) % CRTP_TX_QUEUE_SIZE;
  g_crtpTxQueue.count++;
  g_crtpTxCount++;
  return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
  return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
  if (!packet || port >= CRTP_NBR_OF_PORTS) return false;
  CrtpRxQueue *q = &g_crtpRxQueues[port];
  if (!q->active || q->count == 0U) return false;
  *packet = q->packets[q->head];
  q->head = (q->head + 1U) % CRTP_RX_QUEUE_SIZE;
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
  if (!g_activeLink || !g_activeLink->receivePacket) return;
  CrtpPacket pk;
  if (g_activeLink->receivePacket(&pk)) {
    g_crtpRxCount++;
    uint8_t p = pk.port;
    if (p < CRTP_NBR_OF_PORTS) {
      if (g_crtpPortCallbacks[p]) {
        g_crtpPortCallbacks[p](&pk);
      }
      CrtpRxQueue *q = &g_crtpRxQueues[p];
      if (q->active && q->count < CRTP_RX_QUEUE_SIZE) {
        q->packets[q->tail] = pk;
        q->tail = (q->tail + 1U) % CRTP_RX_QUEUE_SIZE;
        q->count++;
      }
    }
  }
}

void crtpTxTask(void) {
  if (!g_activeLink || !g_activeLink->sendPacket || g_crtpTxQueue.count == 0U) return;
  CrtpPacket *pk = &g_crtpTxQueue.packets[g_crtpTxQueue.head];
  if (g_activeLink->sendPacket(pk)) {
    g_crtpTxQueue.head = (g_crtpTxQueue.head + 1U) % CRTP_TX_QUEUE_SIZE;
    g_crtpTxQueue.count--;
  }
}

void crtpSetLink(CrtpLink *newLink) {
  if (g_activeLink && g_activeLink->setEnable) {
    g_activeLink->setEnable(false);
  }
  g_activeLink = (newLink != NULL) ? newLink : &g_nopLink;
  if (g_activeLink->setEnable) {
    g_activeLink->setEnable(true);
  }
}

void crtpReset(void) {
  g_crtpTxQueue.head = 0U;
  g_crtpTxQueue.tail = 0U;
  g_crtpTxQueue.count = 0U;
  if (g_activeLink && g_activeLink->reset) {
    g_activeLink->reset();
  }
}

bool crtpIsConnected(void) {
  if (g_activeLink && g_activeLink->isConnected) {
    return g_activeLink->isConnected();
  }
  return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
  return CRTP_TX_QUEUE_SIZE - g_crtpTxQueue.count;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
  if (port >= CRTP_NBR_OF_PORTS) return;
  g_crtpPortCallbacks[port] = callback;
}

void updateStats(void) {
  g_lastStatsTick += 500U;
  g_crtpRxCount = 0U;
  g_crtpTxCount = 0U;
}

/* Section 12: Deck Discovery */
uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
  if (!decks || capacity == 0U) return 0U;
  uint8_t count = 0U;
  if (count < capacity) {
    decks[count].foundByI2C = true;
    decks[count].foundByOneWire = false;
    decks[count].i2cAddress = 0xBC;
    decks[count].oneWireRomId = 0ULL;
    count++;
  }
  return count;
}
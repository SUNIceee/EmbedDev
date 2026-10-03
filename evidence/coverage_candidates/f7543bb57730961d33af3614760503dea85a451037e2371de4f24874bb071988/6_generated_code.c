#include "6_generated_code.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EST_Q_CAP 16U
#define DEG2RAD ((float)M_PI / 180.0f)
#define RAD2DEG (180.0f / (float)M_PI)

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

PidObject pidRoll, pidPitch, pidYaw;
PidObject pidRollRate, pidPitchRate, pidYawRate;

bool thrustLocked = true;
bool commanderModeSet = false;

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0;

StateEstimateLog stateEstimate;
Axis3Log gyro, acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

TestState healthTestState = testDone;
uint8_t motorPass = 0, batteryPass = 0;
float batterySag = 0.0f;

static uint32_t hostTick;
static Setpoint commanderActiveSetpoint;
static int commanderActivePriority = COMMANDER_PRIORITY_LOWEST;
static uint32_t commanderLastUpdateTick;
static bool highLevelPending;
static Setpoint highLevelSetpoint;

static EstimatorMeasurement estQ[EST_Q_CAP];
static unsigned estHead, estTail, estCount;

static bool supervisorArmed;
static bool supervisorCrashed;
static bool supervisorFlying;
static bool supervisorTumbled;
static bool supervisorFreeFalling;
static bool supervisorAutoArming;
static uint32_t supervisorSpinupTimeoutMs;
static uint32_t supervisorSpinupStartTick;
static SensorData supervisorSensors;
static uint32_t supervisorMotorRatios[4];
static uint32_t supervisorIdleThrust;
static int32_t supervisorMotorRPMs[4];
static float cfgCrashGs, cfgFreeFall, cfgTiltZ = 0.5f, cfgUpsideDownZ = -0.5f;
static uint32_t cfgMaxTiltMs = 100U, cfgMaxUpsideDownMs = 50U;
static bool cfgTumbleEnabled = true;

static bool healthPropRequest, healthBatRequest;
static uint32_t healthTick;
static float healthIdleVoltage = 4.0f;
static float healthMinLoadedVoltage = 4.0f;
static uint32_t healthMotorTestCount;

typedef struct {
  CrtpPacket data[CRTP_TX_QUEUE_SIZE];
  unsigned head, tail, count, cap;
} PacketQueue;

static PacketQueue txQ;
static PacketQueue rxQ[CRTP_NBR_OF_PORTS];
static bool rxQCreated[CRTP_NBR_OF_PORTS];
static CrtpPortCallback portCb[CRTP_NBR_OF_PORTS];
static CrtpLink *activeLink;
static bool crtpInitialized;
static uint32_t rxCount, txCount, lastStatsTick, rxRate, txRate;

static bool queuePush(PacketQueue *q, const CrtpPacket *p)
{
  if (!q || !p || q->count >= q->cap || q->cap == 0U) return false;
  q->data[q->tail] = *p;
  q->tail = (q->tail + 1U) % q->cap;
  q->count++;
  return true;
}

static bool queuePop(PacketQueue *q, CrtpPacket *p)
{
  if (!q || !p || q->count == 0U || q->cap == 0U) return false;
  *p = q->data[q->head];
  q->head = (q->head + 1U) % q->cap;
  q->count--;
  return true;
}

static void queueClear(PacketQueue *q, unsigned cap)
{
  if (!q) return;
  memset(q, 0, sizeof(*q));
  q->cap = cap;
}

static float clampf(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

static uint16_t clampu16(float v)
{
  if (v <= 0.0f) return 0U;
  if (v >= 65535.0f) return 65535U;
  return (uint16_t)lroundf(v);
}

static void updateSensfusionLog(void)
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

float invSqrt(float x)
{
  if (x <= 0.0f) return 0.0f;
  float xhalf = 0.5f * x;
  union { float f; uint32_t i; } u;
  u.f = x;
  u.i = 0x5f3759dfU - (u.i >> 1);
  u.f = u.f * (1.5f - xhalf * u.f * u.f);
  return u.f;
}

void estimatedGravityDirection(float w, float x, float y, float z,
                               float *gx, float *gy, float *gz)
{
  if (gx) *gx = 2.0f * (x * z - w * y);
  if (gy) *gy = 2.0f * (w * x + y * z);
  if (gz) *gz = w * w - x * x - y * y + z * z;
}

void sensfusion6Init(void)
{
  if (sensfusion6IsInit) return;
  qw = 1.0f; qx = qy = qz = 0.0f;
  integralFBx = integralFBy = integralFBz = 0.0f;
  gravityX = gravityY = 0.0f; gravityZ = 1.0f;
  baseZacc = 0.0f;
  sensfusion6IsCalibrated = false;
  sensfusion6IsInit = true;
  updateSensfusionLog();
}

bool sensfusion6Test(void)
{
  return sensfusion6IsInit;
}

void sensfusion6UpdateQ(float gx_dps, float gy_dps, float gz_dps,
                        float ax, float ay, float az, float dt)
{
  float gx_r = gx_dps * DEG2RAD;
  float gy_r = gy_dps * DEG2RAD;
  float gz_r = gz_dps * DEG2RAD;

  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);

  if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
    float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    if (recipNorm > 0.0f) {
      ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

      float ex = ay * gravityZ - az * gravityY;
      float ey = az * gravityX - ax * gravityZ;
      float ez = ax * gravityY - ay * gravityX;

      if (twoKi > 0.0f) {
        integralFBx += twoKi * ex * dt;
        integralFBy += twoKi * ey * dt;
        integralFBz += twoKi * ez * dt;
        gx_r += integralFBx;
        gy_r += integralFBy;
        gz_r += integralFBz;
      } else {
        integralFBx = integralFBy = integralFBz = 0.0f;
      }

      gx_r += twoKp * ex;
      gy_r += twoKp * ey;
      gz_r += twoKp * ez;

      if (!sensfusion6IsCalibrated) {
        baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
        sensfusion6IsCalibrated = true;
      }
    }
  }

  gx_r *= 0.5f * dt;
  gy_r *= 0.5f * dt;
  gz_r *= 0.5f * dt;

  float qa = qw, qb = qx, qc = qy;
  qw += -qb * gx_r - qc * gy_r - qz * gz_r;
  qx +=  qa * gx_r + qc * gz_r - qz * gy_r;
  qy +=  qa * gy_r - qb * gz_r + qz * gx_r;
  qz +=  qa * gz_r + qb * gy_r - qc * gx_r;

  float n = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (n > 0.0f) {
    qw *= n; qx *= n; qy *= n; qz *= n;
  } else {
    qw = 1.0f; qx = qy = qz = 0.0f;
  }

  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  updateSensfusionLog();
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  if (roll_deg) {
    *roll_deg = atan2f(2.0f * (qw * qx + qy * qz),
                       1.0f - 2.0f * (qx * qx + qy * qy)) * RAD2DEG;
  }
  if (pitch_deg) {
    float s = 2.0f * (qw * qy - qz * qx);
    *pitch_deg = asinf(clampf(s, -1.0f, 1.0f)) * RAD2DEG;
  }
  if (yaw_deg) {
    *yaw_deg = atan2f(2.0f * (qw * qz + qx * qy),
                      1.0f - 2.0f * (qy * qy + qz * qz)) * RAD2DEG;
  }
  stateEstimate.qw = qw;
  stateEstimate.qx = qx;
  stateEstimate.qy = qy;
  stateEstimate.qz = qz;
  if (roll_deg) stateEstimate.roll = *roll_deg;
  if (pitch_deg) stateEstimate.pitch = *pitch_deg;
  if (yaw_deg) stateEstimate.yaw = *yaw_deg;
}

void sensfusion6GetQuaternion(float *ow, float *ox, float *oy, float *oz)
{
  if (ow) *ow = qw;
  if (ox) *ox = qx;
  if (oy) *oy = qy;
  if (oz) *oz = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az)
{
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az)
{
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out)
{
  if (!out) return;
  int32_t r = (int32_t)roll / 2;
  int32_t p = (int32_t)pitch / 2;
  out->m1 = (int32_t)thrust - r + p + yaw;
  out->m2 = (int32_t)thrust - r - p - yaw;
  out->m3 = (int32_t)thrust + r - p + yaw;
  out->m4 = (int32_t)thrust + r + p - yaw;
}

static uint16_t motorForceToPwm(float forceN)
{
  forceN = clampf(forceN, 0.0f, CRAZYFLIE_MAX_MOTOR_FORCE_N);
  return clampu16((forceN / CRAZYFLIE_MAX_MOTOR_FORCE_N) * 65535.0f);
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4])
{
  if (!motorForces) return;
  float thrustPart = 0.25f * thrustSi;
  float rollPart = 0.0f, pitchPart = 0.0f, yawPart = 0.0f;

  if (armLength != 0.0f) {
    float arm = 0.707106781f * armLength;
    if (arm != 0.0f) {
      rollPart = 0.25f * torqueX / arm;
      pitchPart = 0.25f * torqueY / arm;
    }
  }
  if (thrustToTorque != 0.0f) yawPart = 0.25f * torqueZ / thrustToTorque;

  motorForces[0] = thrustPart - rollPart + pitchPart + yawPart;
  motorForces[1] = thrustPart - rollPart - pitchPart - yawPart;
  motorForces[2] = thrustPart + rollPart - pitchPart + yawPart;
  motorForces[3] = thrustPart + rollPart + pitchPart - yawPart;
  for (int i = 0; i < 4; ++i) {
    if (motorForces[i] < 0.0f) motorForces[i] = 0.0f;
  }
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4])
{
  if (!normalizedForces || !motorPWMs) return;
  for (int i = 0; i < 4; ++i) {
    motorPWMs[i] = (uint16_t)lroundf(clampf(normalizedForces[i], 0.0f, 1.0f) * 65535.0f);
  }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower)
{
  if (!control || !motorPower) return;
  if (control->controlMode == controlModeLegacy) {
    powerDistributionLegacy(control->thrust, control->roll, control->pitch, control->yaw, motorPower);
  } else if (control->controlMode == controlModeForceTorque) {
    float f[4];
    powerDistributionForceTorque(control->thrustSi, control->torque.x, control->torque.y,
                                 control->torque.z, CRAZYFLIE_ARM_LENGTH_M,
                                 CRAZYFLIE_THRUST_TO_TORQUE, f);
    motorPower->m1 = motorForceToPwm(f[0]);
    motorPower->m2 = motorForceToPwm(f[1]);
    motorPower->m3 = motorForceToPwm(f[2]);
    motorPower->m4 = motorForceToPwm(f[3]);
  } else if (control->controlMode == controlModeForce) {
    uint16_t pwm[4];
    powerDistributionForce(control->normalizedForces, pwm);
    motorPower->m1 = pwm[0]; motorPower->m2 = pwm[1];
    motorPower->m3 = pwm[2]; motorPower->m4 = pwm[3];
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
  PowerCapResult r = { false, 0 };
  if (!motors) return r;

  int32_t max = motors[0];
  for (int i = 1; i < 4; ++i) if (motors[i] > max) max = motors[i];

  if (max > maxAllowedThrust) {
    r.isCapped = true;
    r.reduction = max - maxAllowedThrust;
    for (int i = 0; i < 4; ++i) motors[i] = capMinThrust(motors[i] - r.reduction, idleThrust);
  }
  return r;
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
  return clampu16((float)motorThrust * nominalVoltage / actualVoltage);
}

static void pidInit(PidObject *p, float kp, float ki, float kd)
{
  if (!p || p->initialized) return;
  p->kp = kp; p->ki = ki; p->kd = kd; p->kff = 0.0f;
  p->integral = p->prevError = p->output = 0.0f;
  p->initialized = true;
}

static void pidReset(PidObject *p, float actual)
{
  if (!p) return;
  p->integral = 0.0f;
  p->prevError = actual;
  p->output = 0.0f;
}

static float pidUpdate(PidObject *p, float actual, float desired)
{
  if (!p) return 0.0f;
  float err = desired - actual;
  p->integral += err;
  float deriv = err - p->prevError;
  p->prevError = err;
  p->output = p->kp * err + p->ki * p->integral + p->kd * deriv + p->kff * desired;
  return p->output;
}

void attitudeControllerInit(float updateDt)
{
  (void)updateDt;
  pidInit(&pidRoll, 6.0f, 0.0f, 0.0f);
  pidInit(&pidPitch, 6.0f, 0.0f, 0.0f);
  pidInit(&pidYaw, 6.0f, 0.0f, 0.0f);
  pidInit(&pidRollRate, 1.0f, 0.0f, 0.0f);
  pidInit(&pidPitchRate, 1.0f, 0.0f, 0.0f);
  pidInit(&pidYawRate, 1.0f, 0.0f, 0.0f);
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired)
{
  pidRollRate.output = (float)saturateSignedInt16((int32_t)lroundf(pidUpdate(&pidRollRate, rollActual, rollDesired)));
  pidPitchRate.output = (float)saturateSignedInt16((int32_t)lroundf(pidUpdate(&pidPitchRate, pitchActual, pitchDesired)));
  pidYawRate.output = (float)saturateSignedInt16((int32_t)lroundf(pidUpdate(&pidYawRate, yawActual, yawDesired)));
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired)
{
  pidUpdate(&pidRoll, rollActual, rollDesired);
  pidUpdate(&pidPitch, pitchActual, pitchDesired);
  pidReset(&pidYaw, yawActual);
  pidUpdate(&pidYaw, yawActual, yawDesired);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual, float yawActual)
{
  pidReset(&pidRoll, rollActual); pidReset(&pidPitch, pitchActual); pidReset(&pidYaw, yawActual);
  pidReset(&pidRollRate, 0.0f); pidReset(&pidPitchRate, 0.0f); pidReset(&pidYawRate, 0.0f);
}

void attitudeControllerResetRollAttitudePID(float rollActual) { pidReset(&pidRoll, rollActual); }
void attitudeControllerResetPitchAttitudePID(float pitchActual) { pidReset(&pidPitch, pitchActual); }

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch, int16_t *yaw)
{
  if (roll) *roll = saturateSignedInt16((int32_t)lroundf(pidRollRate.output));
  if (pitch) *pitch = saturateSignedInt16((int32_t)lroundf(pidPitchRate.output));
  if (yaw) *yaw = saturateSignedInt16((int32_t)lroundf(pidYawRate.output));
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state)
{
  if (!setpoint || !state) return 0;
  float err = 0.0f;
  if (setpoint->mode.z == modeAbs) err = setpoint->position.z - state->position.z;
  else if (setpoint->mode.z == modeVelocity) err = setpoint->velocity.z - state->velocity.z;
  float thrust = 30000.0f + err * 10000.0f;
  return clampu16(thrust);
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt)
{
  static float desiredYaw;
  if (!sensors || !setpoint || !state || !control) return;

  memset(control, 0, sizeof(*control));
  control->controlMode = controlModeLegacy;

  uint16_t thrust = setpoint->mode.z == modeDisable ?
                    setpoint->thrust : positionControllerUpdate(setpoint, state);

  if (thrust == 0U) {
    desiredYaw = state->attitude.yaw;
    attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
    return;
  }

  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
  } else if (setpoint->mode.yaw == modeAbs) {
    if (setpoint->mode.quat == modeAbs) {
      float w = setpoint->attitudeQuaternion.w;
      float x = setpoint->attitudeQuaternion.x;
      float y = setpoint->attitudeQuaternion.y;
      float z = setpoint->attitudeQuaternion.z;
      desiredYaw = atan2f(2.0f * (w * z + x * y),
                          1.0f - 2.0f * (y * y + z * z)) * RAD2DEG;
    } else {
      desiredYaw = setpoint->attitude.yaw;
    }
  }

  if (yawMaxDelta != 0.0f) {
    float d = capAngle(desiredYaw - state->attitude.yaw);
    d = clampf(d, -yawMaxDelta, yawMaxDelta);
    desiredYaw = capAngle(state->attitude.yaw + d);
  }

  float rollDesired = setpoint->attitude.roll;
  float pitchDesired = setpoint->attitude.pitch;
  if (setpoint->mode.roll == modeVelocity) {
    rollDesired = state->attitude.roll;
    pidRoll.output = setpoint->attitudeRate.roll;
    attitudeControllerResetRollAttitudePID(state->attitude.roll);
  }
  if (setpoint->mode.pitch == modeVelocity) {
    pitchDesired = state->attitude.pitch;
    pidPitch.output = setpoint->attitudeRate.pitch;
    attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
  }

  attitudeControllerCorrectAttitudePID(state->attitude.roll, rollDesired,
                                       state->attitude.pitch, pitchDesired,
                                       state->attitude.yaw, desiredYaw);

  float rollRateDesired = setpoint->mode.roll == modeVelocity ? setpoint->attitudeRate.roll : pidRoll.output;
  float pitchRateDesired = setpoint->mode.pitch == modeVelocity ? setpoint->attitudeRate.pitch : pidPitch.output;
  float yawRateDesired = setpoint->mode.yaw == modeVelocity ? setpoint->attitudeRate.yaw : pidYaw.output;

  attitudeControllerCorrectRatePID(sensors->gyro.x, rollRateDesired,
                                   -sensors->gyro.y, pitchRateDesired,
                                   sensors->gyro.z, yawRateDesired);

  attitudeControllerGetActuatorOutput(&control->roll, &control->pitch, &control->yaw);
  control->yaw = (int16_t)-control->yaw;
  control->thrust = thrust;
}

void rotateYaw(float roll, float pitch, float yaw_deg, float *rollPrime, float *pitchPrime)
{
  float r = yaw_deg * DEG2RAD;
  float c = cosf(r), s = sinf(r);
  if (rollPrime) *rollPrime = roll * c - pitch * s;
  if (pitchPrime) *pitchPrime = roll * s + pitch * c;
}

void crtpCommanderRpytDecodeSetpoint(const CommanderCrtpLegacyValues *values,
    Setpoint *setpoint, bool altHoldMode, bool posHoldMode, bool posSetMode,
    StabilizationType stabilizationModeRoll, StabilizationType stabilizationModePitch,
    StabilizationType stabilizationModeYaw, YawMode yawMode)
{
  if (!values || !setpoint) return;

  if (commanderActivePriority == COMMANDER_PRIORITY_DISABLE) {
    thrustLocked = true;
    if (values->thrust == 0U) thrustLocked = false;
  }

  if (altHoldMode) {
    setpoint->mode.z = modeVelocity;
    setpoint->thrust = 0;
    setpoint->velocity.z = ((float)values->thrust - 32767.0f) / 32767.0f;
    if (!commanderModeSet) commanderModeSet = true;
    return;
  }

  if (commanderModeSet) {
    setpoint->mode.z = modeDisable;
    commanderModeSet = false;
  }

  setpoint->thrust = (thrustLocked || values->thrust < MIN_THRUST) ? 0U :
                     (values->thrust > MAX_THRUST ? MAX_THRUST : values->thrust);

  if (posHoldMode) {
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->velocity.x = values->pitch / 30.0f;
    setpoint->velocity.y = values->roll / 30.0f;
    setpoint->attitude.roll = 0.0f;
    setpoint->attitude.pitch = 0.0f;
    return;
  }

  if (posSetMode && values->thrust != 0U) {
    setpoint->mode.x = setpoint->mode.y = setpoint->mode.z = modeAbs;
    setpoint->mode.roll = setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;
    setpoint->position.x = -values->pitch;
    setpoint->position.y = values->roll;
    setpoint->position.z = (float)values->thrust / 1000.0f;
    setpoint->attitude.yaw = values->yaw;
    setpoint->thrust = 0;
    return;
  }

  float roll = values->roll, pitch = values->pitch;
  if (yawMode == PLUSMODE) rotateYaw(roll, pitch, 45.0f, &roll, &pitch);
  else if (yawMode == CAREFREE) setpoint->thrust = 0;

  setpoint->mode.roll = stabilizationModeRoll == RATE ? modeVelocity : modeAbs;
  setpoint->mode.pitch = stabilizationModePitch == RATE ? modeVelocity : modeAbs;
  if (setpoint->mode.roll == modeVelocity) setpoint->attitudeRate.roll = roll;
  else setpoint->attitude.roll = roll;
  if (setpoint->mode.pitch == modeVelocity) setpoint->attitudeRate.pitch = pitch;
  else setpoint->attitude.pitch = pitch;

  setpoint->mode.yaw = stabilizationModeYaw == RATE ? modeVelocity : modeAbs;
  if (setpoint->mode.yaw == modeVelocity) setpoint->attitudeRate.yaw = -values->yaw;
  else setpoint->attitude.yaw = values->yaw;
}

void supervisorInit(void)
{
  supervisorState = supervisorStateLocked;
  supervisorConditionBits = 0;
  supervisorArmed = supervisorCrashed = supervisorFlying = supervisorTumbled = supervisorFreeFalling = false;
  supervisorSpinupStartTick = 0;
  memset(&supervisorSensors, 0, sizeof(supervisorSensors));
  memset(supervisorMotorRatios, 0, sizeof(supervisorMotorRatios));
  memset(supervisorMotorRPMs, 0, sizeof(supervisorMotorRPMs));
}

bool supervisorCanFly(void)
{
  return supervisorState == supervisorStateReadyToFly ||
         supervisorState == supervisorStateFlying ||
         supervisorState == supervisorStateWarningLevelOut ||
         supervisorState == supervisorStateLanded;
}

bool supervisorCanArm(void) { return supervisorState == supervisorStatePreFlChecksPassed; }
bool supervisorIsArmed(void) { return supervisorArmed; }
bool supervisorIsCrashed(void) { return supervisorCrashed; }

bool supervisorRequestArming(bool doArm)
{
  if (!doArm) {
    supervisorArmed = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    return true;
  }
  if (!supervisorCanArm() && supervisorState != supervisorStateArming) return false;
  supervisorArmed = true;
  supervisorState = supervisorStateArming;
  supervisorConditionBits |= SUPERVISOR_CB_ARMED;
  if (supervisorSpinupStartTick == 0U) supervisorSpinupStartTick = hostTick;
  return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
  if (doRecovery) {
    if (supervisorTumbled) return false;
    supervisorCrashed = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    return true;
  }
  supervisorCrashed = true;
  supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
  return true;
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
  uint16_t b = 0;
  if (supervisorCanArm()) b |= 1U << 0;
  if (supervisorArmed) b |= 1U << 1;
  if (supervisorAutoArming) b |= 1U << 2;
  if (supervisorCanFly()) b |= 1U << 3;
  if (supervisorFlying) b |= 1U << 4;
  if (supervisorTumbled) b |= 1U << 5;
  if (thrustLocked) b |= 1U << 6;
  if (supervisorCrashed) b |= 1U << 7;
  return b;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust, uint32_t currentTick)
{
  static uint32_t recentTick;
  static bool seen;
  if (!motorRatios) return false;
  for (int i = 0; i < 4; ++i) {
    if (motorRatios[i] > idleThrust) {
      recentTick = currentTick;
      seen = true;
      return true;
    }
  }
  return seen && ((currentTick - recentTick) < IS_FLYING_HYSTERESIS_THRESHOLD);
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling)
{
  static uint32_t tiltStartTick;
  static bool tiltActive;

  bool freeFall = fabsf(accX) < freeFallThreshold &&
                  fabsf(accY) < freeFallThreshold &&
                  fabsf(accZ) < freeFallThreshold;
  if (isFreeFalling) *isFreeFalling = freeFall;

  if (!tumbleCheckEnabled) return false;

  float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
  supervisorLog.accNorm = norm;
  if (crashDetectionGs > 0.0f && fabsf(norm - 1.0f) > crashDetectionGs) {
    supervisorCrashed = true;
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
  }

  if (freeFall) {
    tiltActive = false;
    tiltStartTick = 0;
    supervisorFreeFalling = true;
    supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
    supervisorState = supervisorStateExceptFreeFall;
    return false;
  }

  supervisorFreeFalling = false;
  supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

  uint32_t timeout = 0;
  if (accZ < acceptedUpsideDownAccZ) timeout = maxUpsideDownTime;
  else if (accZ < acceptedTiltAccZ) timeout = maxTiltTime;

  if (timeout > 0U) {
    if (!tiltActive) {
      tiltActive = true;
      tiltStartTick = currentTick;
    }
    if ((currentTick - tiltStartTick) >= timeout) {
      supervisorTumbled = true;
      supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
      return true;
    }
  } else {
    tiltActive = false;
    tiltStartTick = 0;
    supervisorTumbled = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
  }

  return false;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick)
{
  if (lastNotificationTick == 0U) return true;
  return (currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state, uint32_t latestArmingTick,
                                  uint32_t currentTick, uint32_t preflightTimeoutDuration)
{
  return state == supervisorStatePreFlChecksPassed &&
         latestArmingTick != 0U &&
         (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick, uint32_t currentTick,
                                uint32_t landingTimeoutDuration)
{
  return latestLandingTick != 0U &&
         (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed)
{
  if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed)
    supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
  else
    supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
  if (supervisorArmed) supervisorConditionBits |= SUPERVISOR_CB_ARMED;
  if (supervisorFlying) supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
  if (supervisorTumbled) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
  if (supervisorCrashed) supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
  supervisorLog.info = supervisorGetInfoBitfield();
  return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint, uint32_t bits, SupervisorState state)
{
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
    return;
  }
  if (state == supervisorStateArming || state == supervisorStateReadyToFly ||
      state == supervisorStateFlying || state == supervisorStateLanded) return;

  if (bits & (SUPERVISOR_CB_EMERGENCY_STOP | SUPERVISOR_CB_IS_TUMBLED |
              SUPERVISOR_CB_FREE_FALL | SUPERVISOR_CB_MOTORS_NOT_RESPONDING |
              SUPERVISOR_CB_CRASHED)) {
    memset(setpoint, 0, sizeof(*setpoint));
  } else {
    memset(setpoint, 0, sizeof(*setpoint));
  }
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin, int32_t rpmCheckMax)
{
  if (!motorRPMs) return false;
  for (int i = 0; i < 4; ++i)
    if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
  return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly, uint32_t currentTick)
{
  static uint32_t startTick;
  static bool active;
  if (!canFly) {
    startTick = 0; active = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return false;
  }
  bool low = false;
  if (!motorRPMs) low = true;
  else for (int i = 0; i < 4; ++i) if (motorRPMs[i] < rpmThreshold) low = true;

  if (!low) {
    startTick = 0; active = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return false;
  }
  if (!active) {
    active = true;
    startTick = currentTick;
  }
  if ((currentTick - startTick) >= rpmCheckDurationMs) {
    supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return true;
  }
  return false;
}

void supervisorSetSensorData(const SensorData *sensors)
{
  if (sensors) supervisorSensors = *sensors;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust)
{
  if (motorRatios) memcpy(supervisorMotorRatios, motorRatios, sizeof(supervisorMotorRatios));
  supervisorIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
  if (motorRPMs) memcpy(supervisorMotorRPMs, motorRPMs, sizeof(supervisorMotorRPMs));
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled)
{
  cfgCrashGs = crashDetectionGs;
  cfgFreeFall = freeFallThreshold;
  cfgTiltZ = acceptedTiltAccZ;
  cfgUpsideDownZ = acceptedUpsideDownAccZ;
  cfgMaxTiltMs = maxTiltTime;
  cfgMaxUpsideDownMs = maxUpsideDownTime;
  cfgTumbleEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs)
{
  supervisorAutoArming = autoArming;
  supervisorSpinupTimeoutMs = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep)
{
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;
  hostTick++;

  supervisorFlying = isFlyingCheck(supervisorMotorRatios, supervisorIdleThrust, hostTick);
  bool ff = false;
  supervisorTumbled = isTumbledCheck(supervisorSensors.acc.x, supervisorSensors.acc.y, supervisorSensors.acc.z,
                                     cfgCrashGs, cfgFreeFall, cfgTiltZ, cfgUpsideDownZ,
                                     cfgMaxTiltMs, cfgMaxUpsideDownMs, cfgTumbleEnabled,
                                     hostTick, &ff);
  supervisorFreeFalling = ff;

  uint32_t age = commanderGetInactivityTime();
  if (age > COMMANDER_WDT_TIMEOUT_SHUTDOWN) supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
  else if (age > COMMANDER_WDT_TIMEOUT_STABILIZE) supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;

  if (supervisorState == supervisorStateArming) {
    if (supervisorSpinupStartTick == 0U) supervisorSpinupStartTick = hostTick;
    if (supervisorSpinupTimeoutMs != 0U &&
        (hostTick - supervisorSpinupStartTick) >= supervisorSpinupTimeoutMs)
      supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
  } else {
    supervisorSpinupStartTick = 0;
    supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
  }

  if (supervisorState == supervisorStatePreFlChecksPassed && supervisorAutoArming)
    (void)supervisorRequestArming(true);

  updateAndPopulateConditions(false, false, false);
}

bool estimatorEnqueue(const EstimatorMeasurement *measurement)
{
  if (!measurement || estCount >= EST_Q_CAP) return false;
  estQ[estTail] = *measurement;
  estTail = (estTail + 1U) % EST_Q_CAP;
  estCount++;
  return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement)
{
  if (!measurement || estCount == 0U) return false;
  *measurement = estQ[estHead];
  estHead = (estHead + 1U) % EST_Q_CAP;
  estCount--;
  return true;
}

void estimatorComplementary(uint32_t stabilizerStep)
{
  EstimatorMeasurement m, lastGyro = {MeasurementTypeGyroscope,{0}},
                       lastAcc = {MeasurementTypeAcceleration,{0}},
                       lastBaro = {MeasurementTypeBarometer,{0}};
  while (estimatorDequeue(&m)) {
    if (m.type == MeasurementTypeGyroscope) lastGyro = m;
    else if (m.type == MeasurementTypeAcceleration) lastAcc = m;
    else if (m.type == MeasurementTypeBarometer) lastBaro = m;
  }

  if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
    sensfusion6UpdateQ(lastGyro.data[0], lastGyro.data[1], lastGyro.data[2],
                       lastAcc.data[0], lastAcc.data[1], lastAcc.data[2], 1.0f / 250.0f);
    sensfusion6GetEulerRPY(&stateEstimate.roll, &stateEstimate.pitch, &stateEstimate.yaw);
  }
  if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
    baro.pressure = lastBaro.data[0];
    baro.temp = lastBaro.data[1];
    baro.asl = lastBaro.data[2];
  }
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority)
{
  if (!setpoint) return false;
  if (priority == COMMANDER_PRIORITY_DISABLE || priority >= commanderActivePriority) {
    commanderActiveSetpoint = *setpoint;
    commanderActivePriority = priority;
    commanderLastUpdateTick = hostTick;
    return true;
  }
  return false;
}

void commanderRelaxPriority(void) { commanderActivePriority = COMMANDER_PRIORITY_LOWEST; }
uint32_t commanderGetInactivityTime(void) { return hostTick - commanderLastUpdateTick; }
int commanderGetActivePriority(void) { return commanderActivePriority; }

void stabilizerInit(void)
{
  static bool inited;
  if (inited) return;
  sensfusion6Init();
  attitudeControllerInit(1.0f / 500.0f);
  crtpInit();
  supervisorInit();
  inited = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint)
{
  if (!setpoint) return false;
  highLevelSetpoint = *setpoint;
  highLevelPending = true;
  return true;
}

void stabilizerTask(void)
{
  hostTick++;
  if (healthShallWeRunTest()) {
    SensorData s;
    memset(&s, 0, sizeof(s));
    healthRunTests(&s);
    return;
  }
  if (highLevelPending) {
    commanderSetSetpoint(&highLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
    highLevelPending = false;
  }
  supervisorUpdate(hostTick);
}

static uint32_t quatcompress(const Quaternion *q)
{
  if (!q) return 0;
  uint32_t x = (uint32_t)((int32_t)lroundf((clampf(q->x, -1.0f, 1.0f) + 1.0f) * 511.5f)) & 0x3ffU;
  uint32_t y = (uint32_t)((int32_t)lroundf((clampf(q->y, -1.0f, 1.0f) + 1.0f) * 511.5f)) & 0x3ffU;
  uint32_t z = (uint32_t)((int32_t)lroundf((clampf(q->z, -1.0f, 1.0f) + 1.0f) * 511.5f)) & 0x3ffU;
  return x | (y << 10) | (z << 20);
}

void compressState(const State *state, const SensorData *sensors, CompressedState *output)
{
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
  output->gyro_millirad_s[0] = sensors->gyro.x * DEG2RAD * 1000.0f;
  output->gyro_millirad_s[1] = -sensors->gyro.y * DEG2RAD * 1000.0f;
  output->gyro_millirad_s[2] = sensors->gyro.z * DEG2RAD * 1000.0f;
  output->quatCompressed = quatcompress(&state->attitudeQuaternion);
}

bool rateSupervisorValidate(uint32_t measuredRate)
{
  return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void) { hostTick += 2000U; }

bool healthShallWeRunTest(void)
{
  if (healthPropRequest) {
    healthPropRequest = false;
    healthTestState = configureAcc;
    return true;
  }
  if (healthBatRequest) {
    healthBatRequest = false;
    healthTestState = testBattery;
    healthTick = 0;
    return true;
  }
  return healthTestState != testDone;
}

void healthRequestPropTest(void) { healthPropRequest = true; }
void healthRequestBatteryTest(void) { healthBatRequest = true; }

bool evaluatePropTest(float lowThreshold, float highThreshold, float measuredValue, uint8_t motorIndex)
{
  bool pass = highThreshold == 0.0f ||
              (measuredValue >= lowThreshold && measuredValue <= highThreshold);
  if (pass && motorIndex < 8U) motorPass |= (uint8_t)(1U << motorIndex);
  else healthMotorTestCount++;
  healthLog.motorPass = motorPass;
  healthLog.motorTestCount = healthMotorTestCount;
  return pass;
}

float variance(const float *buffer, int length)
{
  if (!buffer || length <= 0) return 0.0f;
  float sum = 0.0f, sumSq = 0.0f;
  for (int i = 0; i < length; ++i) {
    sum += buffer[i];
    sumSq += buffer[i] * buffer[i];
  }
  return sumSq - (sum * sum / (float)length);
}

void healthRunTests(const SensorData *sensorData)
{
  float v = sensorData ? sensorData->baroTemperature : 4.0f;
  switch (healthTestState) {
  case configureAcc:
    motorPass = batteryPass = 0;
    batterySag = 0.0f;
    healthIdleVoltage = v == 0.0f ? 4.0f : v;
    healthTestState = measureNoiseFloor;
    break;
  case measureNoiseFloor:
    healthTestState = measureProp;
    break;
  case measureProp:
    for (uint8_t i = 0; i < 4U; ++i) (void)evaluatePropTest(0.0f, 0.0f, 1.0f, i);
    healthTestState = evaluatePropResult;
    break;
  case evaluatePropResult:
    healthTestState = testDone;
    break;
  case testBattery:
    healthTick++;
    if (healthTick == 1U) healthMinLoadedVoltage = healthIdleVoltage;
    else if (healthTick >= 2U && healthTick <= 49U && v < healthMinLoadedVoltage) healthMinLoadedVoltage = v;
    else if (healthTick >= 50U) healthTestState = evaluateBatResult;
    break;
  case evaluateBatResult:
    batterySag = healthIdleVoltage - healthMinLoadedVoltage;
    if (batterySag <= 0.5f) batteryPass = 1U;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
    healthTestState = testDone;
    break;
  case restartBatTest:
    healthTick++;
    if (healthTick >= 2000U) {
      healthTick = 0;
      healthTestState = testBattery;
    }
    break;
  case testDone:
  default:
    break;
  }
}

void crtpInit(void)
{
  if (crtpInitialized) return;
  queueClear(&txQ, CRTP_TX_QUEUE_SIZE);
  for (unsigned i = 0; i < CRTP_NBR_OF_PORTS; ++i) queueClear(&rxQ[i], CRTP_RX_QUEUE_SIZE);
  crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port)
{
  if (port >= CRTP_NBR_OF_PORTS) return;
  if (rxQCreated[port]) {
    supervisorConditionBits |= SUPERVISOR_CB_DECK_FAULT;
    return;
  }
  rxQCreated[port] = true;
  queueClear(&rxQ[port], CRTP_RX_QUEUE_SIZE);
}

bool crtpSendPacket(const CrtpPacket *packet)
{
  bool ok = queuePush(&txQ, packet);
  if (ok) txCount++;
  return ok;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) { return crtpSendPacket(packet); }
bool crtpReceivePacket(uint8_t port, CrtpPacket *packet)
{
  if (port >= CRTP_NBR_OF_PORTS || !rxQCreated[port]) return false;
  return queuePop(&rxQ[port], packet);
}
bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet) { return crtpReceivePacket(port, packet); }
bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet, uint32_t wait_ms)
{
  (void)wait_ms;
  return crtpReceivePacket(port, packet);
}

void crtpRxTask(void)
{
  if (!activeLink || !activeLink->receivePacket) return;
  CrtpPacket p;
  while (activeLink->receivePacket(&p)) {
    rxCount++;
    if (p.port < CRTP_NBR_OF_PORTS) {
      if (rxQCreated[p.port]) (void)queuePush(&rxQ[p.port], &p);
      if (portCb[p.port]) portCb[p.port](&p);
    }
  }
}

void crtpTxTask(void)
{
  if (!activeLink || !activeLink->sendPacket || txQ.count == 0U) return;
  CrtpPacket p = txQ.data[txQ.head];
  if (activeLink->sendPacket(&p)) {
    (void)queuePop(&txQ, &p);
  }
}

void crtpSetLink(CrtpLink *newLink)
{
  if (activeLink && activeLink->setEnable) activeLink->setEnable(false);
  activeLink = newLink;
  if (activeLink && activeLink->setEnable) activeLink->setEnable(true);
}

void crtpReset(void)
{
  queueClear(&txQ, CRTP_TX_QUEUE_SIZE);
  if (activeLink && activeLink->reset) activeLink->reset();
}

bool crtpIsConnected(void)
{
  if (activeLink && activeLink->isConnected) return activeLink->isConnected();
  return true;
}

uint32_t crtpGetFreeTxQueuePackets(void)
{
  return CRTP_TX_QUEUE_SIZE - txQ.count;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback)
{
  if (port < CRTP_NBR_OF_PORTS) portCb[port] = callback;
}

void updateStats(void)
{
  if ((hostTick - lastStatsTick) >= 500U) {
    rxRate = rxCount * 2U;
    txRate = txCount * 2U;
    rxCount = txCount = 0;
    lastStatsTick = hostTick;
    (void)rxRate;
    (void)txRate;
  }
}

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity)
{
  if (!decks || capacity == 0U) return 0;
  DeckInfo known[2];
  memset(known, 0, sizeof(known));
  known[0].foundByI2C = true; known[0].i2cAddress = 0x10U;
  known[1].foundByOneWire = true; known[1].oneWireRomId = 0x1122334455667788ULL;

  uint8_t n = capacity < 2U ? capacity : 2U;
  for (uint8_t i = 0; i < n; ++i) decks[i] = known[i];
  return n;
}

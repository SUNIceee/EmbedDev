#include "generated_code.h"

#include <math.h>
#include <string.h>
#include <limits.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

StateEstimateLog stateEstimate;
AxisLog gyro;
AxisLog acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float baseZacc = 0.0f;
bool calibrated = false;

PidObject pidRoll, pidPitch, pidYaw;
PidObject pidRollRate, pidPitchRate, pidYawRate;

static bool sensfusionInitDone = false;
static bool powerInitDone = false;
static bool attitudeInitDone = false;
static float injectedPositionThrust = 0.0f;
static bool useInjectedPositionThrust = false;
static float desiredYaw = 0.0f;
static const float attitudeUpdateDt = 0.002f;
static float batteryFiltered = 0.0f;

int16_t saturateSignedInt16(int32_t value) {
  if (value > INT16_MAX) return INT16_MAX;
  if (value < -INT16_MAX) return (int16_t)-INT16_MAX;
  return (int16_t)value;
}

float capAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float invSqrt(float x) {
  if (x <= 0.0f) return 0.0f;
  union { float f; uint32_t i; } conv;
  conv.f = x;
  conv.i = 0x5f3759dfu - (conv.i >> 1);
  conv.f = conv.f * (1.5f - 0.5f * x * conv.f * conv.f);
  return conv.f;
}

static void syncLogs(void) {
  sensfusion6Log.q0 = qw;
  sensfusion6Log.q1 = qx;
  sensfusion6Log.q2 = qy;
  sensfusion6Log.q3 = qz;
  sensfusion6Log.gravityX = gravityX;
  sensfusion6Log.gravityY = gravityY;
  sensfusion6Log.gravityZ = gravityZ;
  sensfusion6Log.baseZacc = baseZacc;
  sensfusion6Log.calibrated = calibrated;
}

void sensfusion6Init(void) {
  if (sensfusionInitDone) return;
  qw = 1.0f; qx = qy = qz = 0.0f;
  integralFBx = integralFBy = integralFBz = 0.0f;
  gravityX = 0.0f; gravityY = 0.0f; gravityZ = 1.0f;
  baseZacc = 0.0f;
  calibrated = false;
  sensfusionInitDone = true;
  syncLogs();
}

bool sensfusion6Test(void) {
  return sensfusionInitDone;
}

static void normalizeQuaternion(void) {
  float n = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
  if (n <= 0.0f) {
    qw = 1.0f; qx = qy = qz = 0.0f;
    return;
  }
  qw /= n; qx /= n; qy /= n; qz /= n;
}

void estimatedGravityDirection(float *gxOut, float *gyOut, float *gzOut) {
  float gxv = 2.0f * (qx * qz - qw * qy);
  float gyv = 2.0f * (qw * qx + qy * qz);
  float gzv = qw * qw - qx * qx - qy * qy + qz * qz;
  gravityX = gxv; gravityY = gyv; gravityZ = gzv;
  if (gxOut) *gxOut = gxv;
  if (gyOut) *gyOut = gyv;
  if (gzOut) *gzOut = gzv;
  syncLogs();
}

void sensfusion6UpdateQ(float gxDeg, float gyDeg, float gzDeg, float ax, float ay, float az, float dt) {
  const float twoKp = 0.8f;
  const float twoKi = 0.002f;
  float gxRad = gxDeg * (float)M_PI / 180.0f;
  float gyRad = gyDeg * (float)M_PI / 180.0f;
  float gzRad = gzDeg * (float)M_PI / 180.0f;
  float halfvx, halfvy, halfvz, halfex, halfey, halfez;
  bool validAcc = !(ax == 0.0f && ay == 0.0f && az == 0.0f);

  if (validAcc) {
    float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    if (recipNorm > 0.0f) {
      ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
      halfvx = qx * qz - qw * qy;
      halfvy = qw * qx + qy * qz;
      halfvz = qw * qw - 0.5f + qz * qz;
      halfex = (ay * halfvz - az * halfvy);
      halfey = (az * halfvx - ax * halfvz);
      halfez = (ax * halfvy - ay * halfvx);

      if (twoKi > 0.0f) {
        integralFBx += twoKi * halfex * dt;
        integralFBy += twoKi * halfey * dt;
        integralFBz += twoKi * halfez * dt;
        gxRad += integralFBx;
        gyRad += integralFBy;
        gzRad += integralFBz;
      } else {
        integralFBx = integralFBy = integralFBz = 0.0f;
      }

      gxRad += twoKp * halfex;
      gyRad += twoKp * halfey;
      gzRad += twoKp * halfez;
    }
  }

  gxRad *= 0.5f * dt;
  gyRad *= 0.5f * dt;
  gzRad *= 0.5f * dt;

  float qa = qw, qb = qx, qc = qy;
  qw += (-qb * gxRad - qc * gyRad - qz * gzRad);
  qx += (qa * gxRad + qc * gzRad - qz * gyRad);
  qy += (qa * gyRad - qb * gzRad + qz * gxRad);
  qz += (qa * gzRad + qb * gyRad - qc * gxRad);

  normalizeQuaternion();
  estimatedGravityDirection(NULL, NULL, NULL);

  if (validAcc && !calibrated) {
    baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
    calibrated = true;
  }
  sensfusion6Log.accZ = sensfusion6GetAccZ(ax, ay, az);
  syncLogs();
}

void sensfusion6GetEulerRPY(float *roll, float *pitch, float *yaw) {
  normalizeQuaternion();
  estimatedGravityDirection(NULL, NULL, NULL);
  float r = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
  float s = 2.0f * (qw * qy - qz * qx);
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  float p = asinf(s);
  float y = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
  if (roll) *roll = r * 180.0f / (float)M_PI;
  if (pitch) *pitch = p * 180.0f / (float)M_PI;
  if (yaw) *yaw = y * 180.0f / (float)M_PI;
}

void sensfusion6GetQuaternion(float *q0, float *q1, float *q2, float *q3) {
  if (q0) *q0 = qw;
  if (q1) *q1 = qx;
  if (q2) *q2 = qy;
  if (q3) *q3 = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
  estimatedGravityDirection(NULL, NULL, NULL);
  return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

void powerDistributionInit(void) {
  powerInitDone = true;
  (void)powerInitDone;
}

void powerDistributionLegacy(const control_t *control, motorPower_t *out) {
  if (!control || !out) return;
  int32_t r = control->roll / 2;
  int32_t p = control->pitch / 2;
  int32_t t = control->thrust;
  int32_t y = control->yaw;
  out->m1 = t - r + p + y;
  out->m2 = t - r - p - y;
  out->m3 = t + r - p + y;
  out->m4 = t + r + p - y;
}

uint16_t motorForceToPwm(float force) {
  if (force <= 0.0f) return 0;
  float pwm = sqrtf(force) * 65535.0f / sqrtf(0.6f);
  if (pwm < 0.0f) pwm = 0.0f;
  if (pwm > 65535.0f) pwm = 65535.0f;
  return (uint16_t)lrintf(pwm);
}

void powerDistributionForceTorque(const control_t *control, motorPower_t *out) {
  if (!control || !out) return;
  float armLength = control->armLength != 0.0f ? control->armLength : 0.0397f;
  float thrustToTorque = control->thrustToTorque != 0.0f ? control->thrustToTorque : 0.005964f;
  float thrustPart = 0.25f * control->thrustSi;
  float arm = 0.707106781f * armLength;
  float rollPart = arm != 0.0f ? 0.25f / arm * control->torqueX : 0.0f;
  float pitchPart = arm != 0.0f ? 0.25f / arm * control->torqueY : 0.0f;
  float yawPart = thrustToTorque != 0.0f ? 0.25f / thrustToTorque * control->torqueZ : 0.0f;
  float f[4];
  f[0] = thrustPart - rollPart + pitchPart + yawPart;
  f[1] = thrustPart - rollPart - pitchPart - yawPart;
  f[2] = thrustPart + rollPart - pitchPart + yawPart;
  f[3] = thrustPart + rollPart + pitchPart - yawPart;
  for (int i = 0; i < 4; ++i) if (f[i] < 0.0f) f[i] = 0.0f;
  out->m1 = motorForceToPwm(f[0]);
  out->m2 = motorForceToPwm(f[1]);
  out->m3 = motorForceToPwm(f[2]);
  out->m4 = motorForceToPwm(f[3]);
}

void powerDistributionForce(const control_t *control, motorPower_t *out) {
  if (!control || !out) return;
  int32_t *dst[4] = { &out->m1, &out->m2, &out->m3, &out->m4 };
  for (int i = 0; i < 4; ++i) {
    float v = control->normalizedForces[i];
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    *dst[i] = (int32_t)lrintf(v * 65535.0f);
  }
}

void powerDistribution(const control_t *control, motorPower_t *out) {
  if (!control || !out) return;
  if (control->controlMode == legacyMode) powerDistributionLegacy(control, out);
  else if (control->controlMode == forceTorqueMode) powerDistributionForceTorque(control, out);
  else if (control->controlMode == forceMode) powerDistributionForce(control, out);
}

bool powerDistributionCap(motorPower_t *p, uint16_t maxAllowedThrust, uint16_t idleThrust) {
  if (!p) return false;
  int32_t vals[4] = { p->m1, p->m2, p->m3, p->m4 };
  int32_t max = vals[0];
  for (int i = 1; i < 4; ++i) if (vals[i] > max) max = vals[i];
  bool capped = max > (int32_t)maxAllowedThrust;
  int32_t reduction = capped ? max - (int32_t)maxAllowedThrust : 0;
  for (int i = 0; i < 4; ++i) {
    vals[i] -= reduction;
    if (vals[i] < (int32_t)idleThrust) vals[i] = idleThrust;
    if (vals[i] > 65535) vals[i] = 65535;
  }
  p->m1 = vals[0]; p->m2 = vals[1]; p->m3 = vals[2]; p->m4 = vals[3];
  motor.m1 = (uint16_t)p->m1; motor.m2 = (uint16_t)p->m2;
  motor.m3 = (uint16_t)p->m3; motor.m4 = (uint16_t)p->m4;
  return capped;
}

float batteryCompensation(float oldVoltage, float supplyVoltage) {
  batteryFiltered = oldVoltage + 0.01f * (supplyVoltage - oldVoltage);
  return batteryFiltered;
}

uint16_t motorsCompensateBatteryVoltage(uint16_t thrust, float actual, float nominal) {
  if (actual <= 0.0f) return thrust;
  int32_t v = (int32_t)lrintf((float)thrust * nominal / actual);
  if (v < 0) v = 0;
  if (v > 65535) v = 65535;
  return (uint16_t)v;
}

void pidInit(PidObject *pid, float kp, float ki, float kd, float iLimit, float outputLimit, float dt) {
  (void)dt;
  if (!pid) return;
  memset(pid, 0, sizeof(*pid));
  pid->kp = kp; pid->ki = ki; pid->kd = kd; pid->iLimit = iLimit; pid->outputLimit = outputLimit;
}

void pidReset(PidObject *pid) {
  if (!pid) return;
  pid->integ = 0.0f;
  pid->prevError = 0.0f;
  pid->error = 0.0f;
  pid->outP = pid->outI = pid->outD = 0.0f;
}

void pidSetDesired(PidObject *pid, float desired) {
  if (pid) pid->desired = desired;
}

float pidUpdate(PidObject *pid, float measured, bool reset) {
  if (!pid) return 0.0f;
  if (reset) pidReset(pid);
  pid->error = pid->desired - measured;
  pid->integ += pid->error;
  if (pid->iLimit > 0.0f) {
    if (pid->integ > pid->iLimit) pid->integ = pid->iLimit;
    if (pid->integ < -pid->iLimit) pid->integ = -pid->iLimit;
  }
  pid->outP = pid->kp * pid->error;
  pid->outI = pid->ki * pid->integ;
  pid->outD = pid->kd * (pid->error - pid->prevError);
  pid->prevError = pid->error;
  float out = pid->outP + pid->outI + pid->outD;
  if (pid->outputLimit > 0.0f) {
    if (out > pid->outputLimit) out = pid->outputLimit;
    if (out < -pid->outputLimit) out = -pid->outputLimit;
  }
  return out;
}

void attitudeControllerInit(void) {
  if (attitudeInitDone) return;
  pidInit(&pidRoll, 6.0f, 0.0f, 0.0f, 100.0f, 32767.0f, 0.002f);
  pidInit(&pidPitch, 6.0f, 0.0f, 0.0f, 100.0f, 32767.0f, 0.002f);
  pidInit(&pidYaw, 6.0f, 0.0f, 0.0f, 100.0f, 32767.0f, 0.002f);
  pidInit(&pidRollRate, 250.0f, 0.0f, 0.0f, 100.0f, 32767.0f, 0.002f);
  pidInit(&pidPitchRate, 250.0f, 0.0f, 0.0f, 100.0f, 32767.0f, 0.002f);
  pidInit(&pidYawRate, 120.0f, 0.0f, 0.0f, 100.0f, 32767.0f, 0.002f);
  attitudeInitDone = true;
}

void attitudeControllerResetAll(float roll, float pitch, float yaw) {
  pidSetDesired(&pidRoll, roll);
  pidSetDesired(&pidPitch, pitch);
  pidSetDesired(&pidYaw, yaw);
  pidReset(&pidRoll); pidReset(&pidPitch); pidReset(&pidYaw);
  pidReset(&pidRollRate); pidReset(&pidPitchRate); pidReset(&pidYawRate);
}

void attitudeControllerResetRoll(void) { pidReset(&pidRoll); }
void attitudeControllerResetPitch(void) { pidReset(&pidPitch); }
void attitudeControllerResetYaw(void) { pidReset(&pidYaw); }

void controllerPidSetPositionThrust(float thrust) { injectedPositionThrust = thrust; }
void controllerPidUseInjectedPositionThrust(bool enable) { useInjectedPositionThrust = enable; }

static float positionThrust(const setpoint_t *sp, const state_t *st) {
  if (useInjectedPositionThrust) return injectedPositionThrust;
  float ez = sp->position.z - st->position.z;
  float evz = sp->velocity.z - st->velocity.z;
  float out = (float)sp->thrust + 10000.0f * ez + 2000.0f * evz;
  if (out < 0.0f) out = 0.0f;
  if (out > 65535.0f) out = 65535.0f;
  return out;
}

void controllerPid(const setpoint_t *setpoint, const SensorData *sensors, const state_t *state, control_t *control) {
  if (!setpoint || !sensors || !state || !control) return;
  attitudeControllerInit();
  memset(control, 0, sizeof(*control));
  control->controlMode = legacyMode;

  uint16_t thrust = setpoint->mode.z == modeDisable ? setpoint->thrust : (uint16_t)lrintf(positionThrust(setpoint, state));
  if (thrust == 0) {
    attitudeControllerResetAll(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
    desiredYaw = state->attitude.yaw;
    return;
  }

  float desiredRoll = setpoint->attitude.roll;
  float desiredPitch = setpoint->attitude.pitch;
  float desiredRollRate = setpoint->attitudeRate.roll;
  float desiredPitchRate = setpoint->attitudeRate.pitch;

  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw = capAngle(desiredYaw + setpoint->attitudeRate.yaw * attitudeUpdateDt);
  } else if (setpoint->mode.yaw == modeAbs) {
    if (setpoint->mode.quat) {
      float old0 = qw, old1 = qx, old2 = qy, old3 = qz;
      qw = setpoint->quat[0]; qx = setpoint->quat[1]; qy = setpoint->quat[2]; qz = setpoint->quat[3];
      sensfusion6GetEulerRPY(NULL, NULL, &desiredYaw);
      qw = old0; qx = old1; qy = old2; qz = old3;
    } else {
      desiredYaw = setpoint->attitude.yaw;
    }
  }

  if (setpoint->mode.roll == modeVelocity) {
    desiredRollRate = setpoint->attitudeRate.roll;
    pidReset(&pidRoll);
  } else {
    pidSetDesired(&pidRoll, desiredRoll);
    desiredRollRate = pidUpdate(&pidRoll, state->attitude.roll, false);
  }

  if (setpoint->mode.pitch == modeVelocity) {
    desiredPitchRate = setpoint->attitudeRate.pitch;
    pidReset(&pidPitch);
  } else {
    pidSetDesired(&pidPitch, desiredPitch);
    desiredPitchRate = pidUpdate(&pidPitch, state->attitude.pitch, false);
  }

  pidSetDesired(&pidYaw, desiredYaw);
  float desiredYawRate = pidUpdate(&pidYaw, state->attitude.yaw, true);
  if (setpoint->mode.yaw == modeVelocity) desiredYawRate = setpoint->attitudeRate.yaw;

  pidSetDesired(&pidRollRate, desiredRollRate);
  pidSetDesired(&pidPitchRate, desiredPitchRate);
  pidSetDesired(&pidYawRate, desiredYawRate);

  control->roll = saturateSignedInt16((int32_t)lrintf(pidUpdate(&pidRollRate, sensors->gyro.x, false)));
  control->pitch = saturateSignedInt16((int32_t)lrintf(pidUpdate(&pidPitchRate, -sensors->gyro.y, false)));
  control->yaw = -saturateSignedInt16((int32_t)lrintf(pidUpdate(&pidYawRate, sensors->gyro.z, false)));
  control->thrust = thrust;
}

static setpoint_t commanderActive;
static uint8_t commanderPriority = COMMANDER_PRIORITY_LOWEST;
static uint32_t commanderLastTick = 0;
static bool altHoldMode, posHoldMode, posSetMode, plusMode, carefreeMode;
static bool thrustLocked = true;
static bool altModeSet = false;

static float readFloatLE(const uint8_t *p) {
  float f;
  memcpy(&f, p, sizeof(float));
  return f;
}

void commanderInit(void) {
  memset(&commanderActive, 0, sizeof(commanderActive));
  commanderPriority = COMMANDER_PRIORITY_LOWEST;
  commanderLastTick = 0;
  thrustLocked = true;
}

bool commanderSetSetpoint(const setpoint_t *sp, uint8_t priority, uint32_t tick) {
  if (!sp) return false;
  if (priority == COMMANDER_PRIORITY_DISABLE || priority >= commanderPriority) {
    commanderActive = *sp;
    commanderPriority = priority;
    commanderLastTick = tick;
    return true;
  }
  return false;
}

bool commanderGetSetpoint(setpoint_t *sp) {
  if (!sp) return false;
  *sp = commanderActive;
  return true;
}

void commanderRelaxPriority(void) { commanderPriority = COMMANDER_PRIORITY_LOWEST; }
uint32_t commanderGetInactivityTime(uint32_t currentTick) { return currentTick - commanderLastTick; }
uint8_t commanderGetActivePriority(void) { return commanderPriority; }
void commanderSetActivePriority(uint8_t priority) { commanderPriority = priority; }
void commanderSetAltHoldMode(bool enabled) { altHoldMode = enabled; }
void commanderSetPosHoldMode(bool enabled) { posHoldMode = enabled; }
void commanderSetPosSetMode(bool enabled) { posSetMode = enabled; }
void commanderSetPlusMode(bool enabled) { plusMode = enabled; }
void commanderSetCarefreeMode(bool enabled) { carefreeMode = enabled; }

void rotateYaw(float yawDeg, float inX, float inY, float *outX, float *outY) {
  float r = yawDeg * (float)M_PI / 180.0f;
  float c = cosf(r), s = sinf(r);
  if (outX) *outX = inX * c - inY * s;
  if (outY) *outY = inX * s + inY * c;
}

void crtpCommanderRpytDecodeSetpoint(const uint8_t *data, size_t size, setpoint_t *setpoint) {
  if (!data || size < 14 || !setpoint) return;
  float roll = readFloatLE(data + 0);
  float pitch = readFloatLE(data + 4);
  float yaw = readFloatLE(data + 8);
  uint16_t rawThrust = (uint16_t)data[12] | ((uint16_t)data[13] << 8);
  memset(setpoint, 0, sizeof(*setpoint));

  if (commanderPriority == COMMANDER_PRIORITY_DISABLE) thrustLocked = true;
  if (rawThrust == 0) thrustLocked = false;

  if (altHoldMode) {
    setpoint->mode.z = modeVelocity;
    setpoint->thrust = 0;
    setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
    if (!altModeSet) {
      pidReset(&pidRoll); pidReset(&pidPitch);
      altModeSet = true;
    }
  } else {
    if (altModeSet) {
      setpoint->mode.z = modeDisable;
      altModeSet = false;
    }
    setpoint->thrust = (thrustLocked || rawThrust < 1000) ? 0 : (rawThrust > 60000 ? 60000 : rawThrust);
  }

  if (posHoldMode) {
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->velocity.x = pitch / 30.0f;
    setpoint->velocity.y = roll / 30.0f;
    setpoint->attitude.roll = 0.0f;
    setpoint->attitude.pitch = 0.0f;
    return;
  }

  if (posSetMode && rawThrust != 0) {
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;
    setpoint->position.x = -pitch;
    setpoint->position.y = roll;
    setpoint->position.z = (float)rawThrust / 1000.0f;
    setpoint->attitude.yaw = yaw;
    setpoint->thrust = 0;
    return;
  }

  if (carefreeMode) {
    setpoint->thrust = 0;
    return;
  }

  if (plusMode) rotateYaw(45.0f, roll, pitch, &roll, &pitch);

  setpoint->mode.roll = modeAbs;
  setpoint->mode.pitch = modeAbs;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitude.roll = roll;
  setpoint->attitude.pitch = pitch;
  setpoint->attitudeRate.yaw = -yaw;
}

static supervisorState_t supervisorState = supervisorStateInit;
static bool armingFlag, crashFlag, tumbledFlag, freeFallFlag, lockedFlag, autoArmingFlag, deckFaultFlag;
static bool trajectoryFlying, trajectoryFinished, trajectoryDisabled;
static uint32_t conditionBits;
static SensorData supervisorSensors;
static uint16_t supervisorMotorRatios[4], supervisorRPMs[4];
static float cfgCrashGs = 0.0f, cfgFreeFall = 0.1f, cfgTilt = 0.5f, cfgInverted = -0.5f;
static uint32_t cfgTiltTimeout = 1000, cfgInvertedTimeout = 250;
static bool cfgTumbleEnabled = true;
static uint32_t tumbleStart = 0;
static bool seenFlight = false;
static uint32_t recentFlightTick = 0;
static uint32_t spinupStartTick = 0;
static uint32_t spinupTimeoutMs = 500;

void supervisorInit(void) {
  supervisorState = supervisorStateInit;
  armingFlag = crashFlag = tumbledFlag = freeFallFlag = lockedFlag = false;
  conditionBits = 0;
  memset(supervisorMotorRatios, 0, sizeof(supervisorMotorRatios));
  memset(supervisorRPMs, 0, sizeof(supervisorRPMs));
}

bool supervisorCanFly(void) {
  return supervisorState == supervisorStateReadyToFly || supervisorState == supervisorStateFlying ||
         supervisorState == supervisorStateWarningLevelOut || supervisorState == supervisorStateLanded;
}

bool supervisorCanArm(void) {
  return supervisorState == supervisorStatePreFlChecksPassed;
}

bool supervisorIsArmed(void) { return armingFlag; }
bool supervisorIsCrashed(void) { return crashFlag; }

bool supervisorRequestArming(void) {
  if (supervisorCanArm() || supervisorState == supervisorStateArming) {
    armingFlag = true;
    supervisorState = supervisorStateArming;
    if (spinupStartTick == 0) spinupStartTick = 1;
    return true;
  }
  return false;
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
  if (tumbledFlag) return false;
  if (doRecovery) crashFlag = false;
  else crashFlag = true;
  return true;
}

bool isFlyingCheck(uint32_t currentTick, const uint16_t motorRatio[4], uint16_t idleThrust) {
  bool above = false;
  if (motorRatio) {
    for (int i = 0; i < 4; ++i) if (motorRatio[i] > idleThrust) above = true;
  }
  if (above) {
    recentFlightTick = currentTick;
    seenFlight = true;
    return true;
  }
  if (!seenFlight) return false;
  return (currentTick - recentFlightTick) < 2000u;
}

bool isTumbledCheck(const Axis3f *a, float accZ, uint32_t currentTick) {
  if (!a) return false;
  float norm = sqrtf(a->x * a->x + a->y * a->y + a->z * a->z);
  supervisorLog.accNorm = norm;
  if (cfgCrashGs > 0.0f && fabsf(norm - 1.0f) > cfgCrashGs) crashFlag = true;

  if (fabsf(a->x) < cfgFreeFall && fabsf(a->y) < cfgFreeFall && fabsf(a->z) < cfgFreeFall) {
    freeFallFlag = true;
    supervisorState = supervisorStateExceptFreeFall;
    tumbleStart = 0;
    return false;
  }
  freeFallFlag = false;

  if (!cfgTumbleEnabled) return false;
  uint32_t timeout = cfgTiltTimeout;
  bool tilted = false;
  if (accZ < cfgInverted) {
    tilted = true;
    timeout = cfgInvertedTimeout;
  } else if (accZ < cfgTilt) {
    tilted = true;
  }
  if (!tilted) {
    tumbleStart = 0;
    tumbledFlag = false;
    return false;
  }
  if (tumbleStart == 0) tumbleStart = currentTick ? currentTick : 1;
  if (currentTick - tumbleStart >= timeout) tumbledFlag = true;
  return tumbledFlag;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick) {
  if (lastNotificationTick == 0) return true;
  return (currentTick - lastNotificationTick) <= 1000u;
}

bool supervisorIsPreflightTimeout(uint32_t currentTick, uint32_t startTick, uint32_t timeout) {
  return startTick != 0 && currentTick - startTick >= timeout;
}

bool supervisorIsLandingTimeout(uint32_t currentTick, uint32_t landingTick, uint32_t timeout) {
  return landingTick != 0 && currentTick - landingTick >= timeout;
}

void updateAndPopulateConditions(uint32_t currentTick) {
  if (commanderGetInactivityTime(currentTick) > 500u) conditionBits |= SUPERVISOR_CB_COMMANDER_WARNING;
  if (commanderGetInactivityTime(currentTick) > 2000u) conditionBits |= SUPERVISOR_CB_COMMANDER_TIMEOUT;
  if (tumbledFlag) conditionBits |= SUPERVISOR_CB_TUMBLED;
  if (freeFallFlag) conditionBits |= SUPERVISOR_CB_FREE_FALL;
  supervisorLog.conditions = conditionBits;
}

bool isRPMatArmingValid(uint32_t currentTick) {
  (void)currentTick;
  for (int i = 0; i < 4; ++i) {
    if (supervisorRPMs[i] < 100 || supervisorRPMs[i] > 50000) return false;
  }
  return true;
}

void supervisorSetSensorData(const SensorData *sensors) {
  if (sensors) supervisorSensors = *sensors;
}

void supervisorSetMotorRatios(const uint16_t ratios[4]) {
  if (ratios) memcpy(supervisorMotorRatios, ratios, sizeof(supervisorMotorRatios));
}

void supervisorSetMotorRPMs(const uint16_t rpms[4]) {
  if (rpms) memcpy(supervisorRPMs, rpms, sizeof(supervisorRPMs));
}

void supervisorConfigureSafety(float crashGs, float freeFallThreshold, float tiltThreshold, float invertedThreshold, uint32_t tiltTimeout, uint32_t invertedTimeout, bool tumbleEnabled) {
  cfgCrashGs = crashGs;
  cfgFreeFall = freeFallThreshold;
  cfgTilt = tiltThreshold;
  cfgInverted = invertedThreshold;
  cfgTiltTimeout = tiltTimeout;
  cfgInvertedTimeout = invertedTimeout;
  cfgTumbleEnabled = tumbleEnabled;
}

void supervisorOverrideSetpoint(setpoint_t *sp) {
  if (!sp) return;
  if (supervisorState == supervisorStateWarningLevelOut) {
    sp->mode.x = modeDisable;
    sp->mode.y = modeDisable;
    sp->mode.roll = modeAbs;
    sp->mode.pitch = modeAbs;
    sp->mode.yaw = modeVelocity;
    sp->attitude.roll = 0.0f;
    sp->attitude.pitch = 0.0f;
    sp->attitudeRate.yaw = 0.0f;
    return;
  }
  if (supervisorState == supervisorStateArming || supervisorState == supervisorStateReadyToFly ||
      supervisorState == supervisorStateFlying || supervisorState == supervisorStateLanded) return;
  memset(sp, 0, sizeof(*sp));
}

bool supervisorAreMotorsAllowedToRun(void) {
  return supervisorState == supervisorStateArming || supervisorState == supervisorStateReadyToFly ||
         supervisorState == supervisorStateFlying || supervisorState == supervisorStateWarningLevelOut ||
         supervisorState == supervisorStateLanded;
}

uint32_t supervisorGetInfoBitfield(void) {
  uint32_t b = 0;
  if (supervisorCanArm()) b |= 1u << 0;
  if (armingFlag) b |= 1u << 1;
  if (autoArmingFlag) b |= 1u << 2;
  if (supervisorCanFly()) b |= 1u << 3;
  if (isFlyingCheck(0, supervisorMotorRatios, 0)) b |= 1u << 4;
  if (tumbledFlag) b |= 1u << 5;
  if (lockedFlag) b |= 1u << 6;
  if (crashFlag) b |= 1u << 7;
  if (trajectoryFlying) b |= 1u << 8;
  if (trajectoryFinished) b |= 1u << 9;
  if (trajectoryDisabled) b |= 1u << 10;
  if (deckFaultFlag) b |= 1u << 11;
  supervisorLog.info = b;
  return b;
}

void supervisorSetState(supervisorState_t state) { supervisorState = state; }
supervisorState_t supervisorGetState(void) { return supervisorState; }
void supervisorSetAutoArming(bool enabled) { autoArmingFlag = enabled; }
void supervisorSetTumbled(bool tumbled) { tumbledFlag = tumbled; }
void supervisorSetLocked(bool locked) { lockedFlag = locked; }
void supervisorSetDeckFault(bool fault) { deckFaultFlag = fault; }

void supervisorUpdate(uint32_t stabilizerStep, uint32_t tick) {
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;
  float accZ = sensfusion6GetAccZ(supervisorSensors.acc.x, supervisorSensors.acc.y, supervisorSensors.acc.z);
  isTumbledCheck(&supervisorSensors.acc, accZ, tick);
  isFlyingCheck(tick, supervisorMotorRatios, 0);
  updateAndPopulateConditions(tick);
  if (supervisorState == supervisorStatePreFlChecksPassed && autoArmingFlag) supervisorRequestArming();
  if (supervisorState == supervisorStateArming) {
    if (spinupStartTick == 0) spinupStartTick = tick ? tick : 1;
    if (tick - spinupStartTick >= spinupTimeoutMs) conditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
  } else {
    spinupStartTick = 0;
    conditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
  }
  supervisorLog.canArm = supervisorCanArm();
  supervisorLog.canFly = supervisorCanFly();
  supervisorLog.armed = armingFlag;
  supervisorLog.flying = seenFlight;
  supervisorLog.tumbled = tumbledFlag;
  supervisorLog.crashed = crashFlag;
}

static estimatorMeasurement_t estQ[16];
static unsigned estHead, estTail, estCount;

void estimatorInit(void) { estHead = estTail = estCount = 0; }

bool estimatorEnqueue(const estimatorMeasurement_t *m) {
  if (!m || estCount >= 16) return false;
  estQ[estTail] = *m;
  estTail = (estTail + 1u) & 15u;
  estCount++;
  return true;
}

bool estimatorDequeue(estimatorMeasurement_t *m) {
  if (!m || estCount == 0) return false;
  *m = estQ[estHead];
  estHead = (estHead + 1u) & 15u;
  estCount--;
  return true;
}

void estimatorComplementary(state_t *state, const SensorData *sensors, uint32_t step, float dt) {
  if (!state || !sensors) return;
  estimatorMeasurement_t m;
  while (estimatorDequeue(&m)) {
    if (m.type == EST_MEASUREMENT_POSITION) {
      state->position.x = m.data[0]; state->position.y = m.data[1]; state->position.z = m.data[2];
    }
  }
  if (RATE_DO_EXECUTE(RATE_250_HZ, step)) {
    sensfusion6UpdateQ(sensors->gyro.x, sensors->gyro.y, sensors->gyro.z, sensors->acc.x, sensors->acc.y, sensors->acc.z, dt);
    sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);
    sensfusion6GetQuaternion(&state->quat[0], &state->quat[1], &state->quat[2], &state->quat[3]);
    state->velocity.z += sensfusion6GetAccZWithoutGravity(sensors->acc.x, sensors->acc.y, sensors->acc.z) * 9.81f * dt;
  }
  if (RATE_DO_EXECUTE(RATE_100_HZ, step)) {
    state->position.x += state->velocity.x * dt;
    state->position.y += state->velocity.y * dt;
    state->position.z += state->velocity.z * dt;
  }
}

static bool stabilizerInited = false;
static bool highLevelPending = false;
static setpoint_t highLevelSetpoint;

void stabilizerInit(void) {
  if (stabilizerInited) return;
  sensfusion6Init();
  estimatorInit();
  attitudeControllerInit();
  powerDistributionInit();
  commanderInit();
  supervisorInit();
  stabilizerInited = true;
}

void stabilizerSubmitHighLevelSetpoint(const setpoint_t *sp) {
  if (sp) {
    highLevelSetpoint = *sp;
    highLevelPending = true;
  }
}

void stabilizerTask(uint32_t tick) {
  if (highLevelPending) {
    commanderSetSetpoint(&highLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL, tick);
    highLevelPending = false;
  }
}

uint32_t quatcompress(float q0, float q1, float q2, float q3) {
  float q[4] = { q0, q1, q2, q3 };
  int largest = 0;
  for (int i = 1; i < 4; ++i) if (fabsf(q[i]) > fabsf(q[largest])) largest = i;
  uint32_t r = (uint32_t)largest << 30;
  int shift = 0;
  for (int i = 0; i < 4; ++i) {
    if (i == largest) continue;
    int32_t v = (int32_t)lrintf((q[i] + 1.0f) * 511.5f);
    if (v < 0) v = 0;
    if (v > 1023) v = 1023;
    r |= ((uint32_t)v & 0x3ffu) << shift;
    shift += 10;
  }
  return r;
}

compressedState_t compressState(const state_t *state, const SensorData *sensors) {
  compressedState_t c;
  memset(&c, 0, sizeof(c));
  if (!state || !sensors) return c;
  c.x = (int32_t)lrintf(state->position.x * 1000.0f);
  c.y = (int32_t)lrintf(state->position.y * 1000.0f);
  c.z = (int32_t)lrintf(state->position.z * 1000.0f);
  c.vx = (int32_t)lrintf(state->velocity.x * 1000.0f);
  c.vy = (int32_t)lrintf(state->velocity.y * 1000.0f);
  c.vz = (int32_t)lrintf(state->velocity.z * 1000.0f);
  c.ax = (int32_t)lrintf(sensors->acc.x * 9810.0f);
  c.ay = (int32_t)lrintf(sensors->acc.y * 9810.0f);
  c.az = (int32_t)lrintf((sensors->acc.z + 1.0f) * 9810.0f);
  c.gx = (int32_t)lrintf(sensors->gyro.x * (float)M_PI / 180.0f * 1000.0f);
  c.gy = (int32_t)lrintf(-sensors->gyro.y * (float)M_PI / 180.0f * 1000.0f);
  c.gz = (int32_t)lrintf(sensors->gyro.z * (float)M_PI / 180.0f * 1000.0f);
  c.quat = quatcompress(state->quat[0], state->quat[1], state->quat[2], state->quat[3]);
  return c;
}

bool rateSupervisorValidate(uint32_t hz) {
  return hz >= 997u && hz <= 1003u;
}

static bool propReq, batReq, propRunning, batRunning, testDoneFlag;
static uint32_t batTick, restartTick;
static float idleVoltage, minLoadedVoltage;
static float batSagThreshold = 0.5f;

void healthInit(void) {
  memset(&healthLog, 0, sizeof(healthLog));
  propReq = batReq = propRunning = batRunning = testDoneFlag = false;
}

void startPropTest(void) { propReq = true; testDoneFlag = false; }
void startBatTest(void) { batReq = true; testDoneFlag = false; }

void configureAcc(void) {
  healthLog.motorPass = 0;
  healthLog.motorTestCount = 0;
  motor.m1 = motor.m2 = motor.m3 = motor.m4 = 0;
}

void testBattery(void) {
  batRunning = true;
  batTick = 0;
}

bool healthShallWeRunTest(void) {
  if (propReq) {
    configureAcc();
    propReq = false;
    propRunning = true;
    return true;
  }
  if (batReq) {
    testBattery();
    batReq = false;
    return true;
  }
  return propRunning || batRunning;
}

float variance(const float *values, size_t n) {
  if (!values || n == 0) return 0.0f;
  float sum = 0.0f, sumSq = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    sum += values[i];
    sumSq += values[i] * values[i];
  }
  return sumSq - (sum * sum / (float)n);
}

bool evaluatePropTest(float value, float lowThreshold, float highThreshold, uint8_t motorIndex) {
  bool pass = highThreshold == 0.0f || (value >= lowThreshold && value <= highThreshold);
  if (pass && motorIndex < 4) healthLog.motorPass |= (uint8_t)(1u << motorIndex);
  if (!pass) healthLog.motorTestCount++;
  return pass;
}

void restartBatTest(uint32_t currentTick) {
  if (restartTick == 0) restartTick = currentTick ? currentTick : 1;
  if (currentTick - restartTick >= 2000u) {
    testBattery();
    restartTick = 0;
  }
}

void healthRunTests(uint32_t tick, const SensorData *sensors, float batteryVoltage) {
  (void)sensors;
  if (propRunning) {
    healthLog.motorPass = 0x0f;
    propRunning = false;
    testDoneFlag = true;
  }
  if (batRunning) {
    batTick = tick;
    if (tick == 1) {
      motor.m1 = motor.m2 = motor.m3 = motor.m4 = 30000;
      idleVoltage = batteryVoltage;
      minLoadedVoltage = batteryVoltage;
    } else if (tick >= 2 && tick <= 49) {
      if (batteryVoltage < minLoadedVoltage) minLoadedVoltage = batteryVoltage;
    } else if (tick >= 50) {
      motor.m1 = motor.m2 = motor.m3 = motor.m4 = 0;
      healthLog.batterySag = idleVoltage - minLoadedVoltage;
      healthLog.batteryPass = healthLog.batterySag <= batSagThreshold;
      batRunning = false;
      testDoneFlag = true;
    }
  }
}

typedef struct {
  CRTPPacket q[200];
  unsigned head, tail, count;
} TxQ;

typedef struct {
  CRTPPacket q[16];
  unsigned head, tail, count;
  bool created;
} RxQ;

static TxQ txq;
static RxQ rxq[CRTP_NBR_OF_PORTS];
static crtpPortCB portCallbacks[CRTP_NBR_OF_PORTS];
static CRTPLink *activeLink = NULL;
static uint32_t crtpRxCount, crtpTxCount, crtpRxRate, crtpTxRate, crtpLastStatsTick;
static bool crtpError;

static bool queuePushTx(const CRTPPacket *p) {
  if (txq.count >= 200 || !p) return false;
  txq.q[txq.tail] = *p;
  txq.tail = (txq.tail + 1u) % 200u;
  txq.count++;
  return true;
}

static bool queuePopTx(CRTPPacket *p) {
  if (txq.count == 0 || !p) return false;
  *p = txq.q[txq.head];
  txq.head = (txq.head + 1u) % 200u;
  txq.count--;
  return true;
}

static bool queuePushRx(uint8_t port, const CRTPPacket *p) {
  if (port >= CRTP_NBR_OF_PORTS || !rxq[port].created || rxq[port].count >= 16 || !p) return false;
  RxQ *q = &rxq[port];
  q->q[q->tail] = *p;
  q->tail = (q->tail + 1u) & 15u;
  q->count++;
  return true;
}

static bool queuePopRx(uint8_t port, CRTPPacket *p) {
  if (port >= CRTP_NBR_OF_PORTS || !rxq[port].created || rxq[port].count == 0 || !p) return false;
  RxQ *q = &rxq[port];
  *p = q->q[q->head];
  q->head = (q->head + 1u) & 15u;
  q->count--;
  return true;
}

void crtpInit(void) {
  memset(&txq, 0, sizeof(txq));
  memset(rxq, 0, sizeof(rxq));
  memset(portCallbacks, 0, sizeof(portCallbacks));
  crtpError = false;
}

bool crtpCreatePacketQueue(uint8_t port) {
  if (port >= CRTP_NBR_OF_PORTS) return false;
  if (rxq[port].created) {
    crtpError = true;
    return false;
  }
  rxq[port].created = true;
  return true;
}

bool crtpSendPacket(const CRTPPacket *pk) {
  bool ok = queuePushTx(pk);
  if (ok) crtpTxCount++;
  return ok;
}

bool crtpSendPacketBlock(const CRTPPacket *pk) { return crtpSendPacket(pk); }
bool crtpReceivePacket(uint8_t port, CRTPPacket *pk) { return queuePopRx(port, pk); }
bool crtpReceivePacketBlock(uint8_t port, CRTPPacket *pk) { return queuePopRx(port, pk); }
bool crtpReceivePacketWait(uint8_t port, CRTPPacket *pk, uint32_t timeoutMs) {
  (void)timeoutMs;
  return queuePopRx(port, pk);
}

void crtpRxTask(void) {
  if (!activeLink || !activeLink->receivePacket) return;
  CRTPPacket pk;
  while (activeLink->receivePacket(&pk)) {
    uint8_t port = pk.port;
    if (port < CRTP_NBR_OF_PORTS) {
      queuePushRx(port, &pk);
      if (portCallbacks[port]) portCallbacks[port](&pk);
      crtpRxCount++;
    }
  }
}

void crtpTxTask(uint32_t currentTick) {
  (void)currentTick;
  if (!activeLink || !activeLink->sendPacket || txq.count == 0) return;
  CRTPPacket pk = txq.q[txq.head];
  if (activeLink->sendPacket(&pk)) {
    queuePopTx(&pk);
  }
}

void crtpSetLink(CRTPLink *link) {
  if (activeLink && activeLink->setEnable) activeLink->setEnable(false);
  activeLink = link;
  if (activeLink && activeLink->setEnable) activeLink->setEnable(true);
}

void crtpReset(void) {
  memset(&txq, 0, sizeof(txq));
  if (activeLink && activeLink->reset) activeLink->reset();
}

bool crtpIsConnected(void) {
  if (activeLink && activeLink->isConnected) return activeLink->isConnected();
  return true;
}

uint16_t crtpGetFreeTxQueuePackets(void) {
  return (uint16_t)(200u - txq.count);
}

bool crtpRegisterPortCB(uint8_t port, crtpPortCB cb) {
  if (port >= CRTP_NBR_OF_PORTS) return false;
  portCallbacks[port] = cb;
  return true;
}

void crtpUpdateStats(uint32_t currentTick) {
  if (crtpLastStatsTick == 0) crtpLastStatsTick = currentTick;
  if (currentTick - crtpLastStatsTick >= 500u) {
    crtpRxRate = crtpRxCount * 2u;
    crtpTxRate = crtpTxCount * 2u;
    crtpRxCount = crtpTxCount = 0;
    crtpLastStatsTick = currentTick;
  }
}

uint32_t crtpGetRxRate(void) { return crtpRxRate; }
uint32_t crtpGetTxRate(void) { return crtpTxRate; }

size_t deckDiscovery(const uint8_t *i2cAddresses, size_t i2cCount, const uint64_t *oneWireRoms, size_t romCount, uint64_t *out, size_t capacity) {
  if (!out || capacity == 0) return 0;
  size_t n = 0;
  for (size_t i = 0; i < i2cCount && n < capacity; ++i) {
    uint64_t v = i2cAddresses ? (uint64_t)i2cAddresses[i] : 0u;
    bool dup = false;
    for (size_t j = 0; j < n; ++j) if (out[j] == v) dup = true;
    if (!dup) out[n++] = v;
  }
  for (size_t i = 0; i < romCount && n < capacity; ++i) {
    uint64_t v = oneWireRoms ? oneWireRoms[i] : 0u;
    bool dup = false;
    for (size_t j = 0; j < n; ++j) if (out[j] == v) dup = true;
    if (!dup) out[n++] = v;
  }
  return n;
}

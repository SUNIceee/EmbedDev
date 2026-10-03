#include "generated_code.h"

#include <math.h>
#include <string.h>

StateEstimateLog stateEstimate;
AxisLog gyro, acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6Calibrated = false;
PidObject pidRoll, pidPitch, pidYaw, pidRollRate, pidPitchRate, pidYawRate;
float desiredYaw = 0.0f;
uint32_t stabilizerStep = 0;

static bool powerInitDone, controllerInitDone, commanderInitDone, stabilizerInitDone, crtpInitDone;
static uint16_t injectedPositionThrust = 0;
static setpoint_t commanderSp, highlevelSp;
static bool highlevelPending;
static uint8_t commanderPriority = COMMANDER_PRIORITY_LOWEST;
static uint32_t commanderLastUpdate;
static SupervisorState supState = supervisorStateInit;
static bool supArmed, supCrashed, supTumbled, supFreeFall, supLocked, supAutoArming;
static uint32_t supLastFlightTick, supTiltStartTick, supRpmValidStart;
static bool supSeenFlight, supTumbleEnabled = true;
static float supCrashGs, supFreeFallThreshold = 0.1f, supTiltThreshold = 0.5f, supUpsideDownThreshold = -0.5f;
static uint32_t supTiltTimeout = 1000, supUpsideDownTimeout = 250;
static float supAx, supAy, supAz, supAccZ;
static uint16_t supRatios[4], supRpms[4];

int16_t saturateSignedInt16(int32_t value) {
  if (value > 32767) return 32767;
  if (value < -32767) return -32767;
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
  float xhalf = 0.5f * x;
  conv.f = x;
  conv.i = 0x5f3759dfu - (conv.i >> 1);
  conv.f = conv.f * (1.5f - xhalf * conv.f * conv.f);
  return conv.f;
}

void sensfusion6Init(void) {
  if (sensfusion6IsInit) return;
  qw = 1.0f; qx = qy = qz = 0.0f;
  integralFBx = integralFBy = integralFBz = 0.0f;
  gravityX = gravityY = 0.0f; gravityZ = 1.0f;
  baseZacc = 0.0f; sensfusion6Calibrated = false;
  sensfusion6IsInit = true;
}

bool sensfusion6Test(void) { return sensfusion6IsInit; }

static void normalizeQuaternion(void) {
  float n = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
  if (n <= 0.0f) { qw = 1.0f; qx = qy = qz = 0.0f; return; }
  qw /= n; qx /= n; qy /= n; qz /= n;
}

void estimatedGravityDirection(float *gx, float *gy, float *gz) {
  float x = 2.0f * (qx * qz - qw * qy);
  float y = 2.0f * (qw * qx + qy * qz);
  float z = qw * qw - qx * qx - qy * qy + qz * qz;
  gravityX = x; gravityY = y; gravityZ = z;
  sensfusion6Log.gravityX = x; sensfusion6Log.gravityY = y; sensfusion6Log.gravityZ = z;
  if (gx) *gx = x; if (gy) *gy = y; if (gz) *gz = z;
}

void sensfusion6UpdateQ(float gxDeg, float gyDeg, float gzDeg, float ax, float ay, float az, float dt) {
  float twoKp = 0.8f, twoKi = 0.002f;
  float gxRad = gxDeg * PI / 180.0f, gyRad = gyDeg * PI / 180.0f, gzRad = gzDeg * PI / 180.0f;
  if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
    float recip = invSqrt(ax * ax + ay * ay + az * az);
    if (recip > 0.0f) {
      ax *= recip; ay *= recip; az *= recip;
      estimatedGravityDirection(NULL, NULL, NULL);
      float ex = (ay * gravityZ - az * gravityY);
      float ey = (az * gravityX - ax * gravityZ);
      float ez = (ax * gravityY - ay * gravityX);
#if CONFIG_IMU_MADGWICK_QUATERNION
      gxRad += 0.01f * ex; gyRad += 0.01f * ey; gzRad += 0.01f * ez;
      integralFBx = integralFBy = integralFBz = 0.0f;
#else
      if (twoKi > 0.0f) {
        integralFBx += twoKi * ex * dt; integralFBy += twoKi * ey * dt; integralFBz += twoKi * ez * dt;
        gxRad += integralFBx; gyRad += integralFBy; gzRad += integralFBz;
      } else {
        integralFBx = integralFBy = integralFBz = 0.0f;
      }
      gxRad += twoKp * ex; gyRad += twoKp * ey; gzRad += twoKp * ez;
#endif
      if (!sensfusion6Calibrated) {
        baseZacc = sensfusion6GetAccZ(ax, ay, az);
        sensfusion6Calibrated = true;
      }
    }
  }
  gxRad *= 0.5f * dt; gyRad *= 0.5f * dt; gzRad *= 0.5f * dt;
  float qa = qw, qb = qx, qc = qy;
  qw += (-qb * gxRad - qc * gyRad - qz * gzRad);
  qx += (qa * gxRad + qc * gzRad - qz * gyRad);
  qy += (qa * gyRad - qb * gzRad + qz * gxRad);
  qz += (qa * gzRad + qb * gyRad - qc * gxRad);
  normalizeQuaternion();
  estimatedGravityDirection(NULL, NULL, NULL);
  sensfusion6Log.qw = qw; sensfusion6Log.qx = qx; sensfusion6Log.qy = qy; sensfusion6Log.qz = qz;
  sensfusion6Log.baseZacc = baseZacc; sensfusion6Log.isCalibrated = sensfusion6Calibrated;
}

void sensfusion6GetEulerRPY(float *roll, float *pitch, float *yaw) {
  normalizeQuaternion();
  float sinr = 2.0f * (qw * qx + qy * qz);
  float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
  float sinp = 2.0f * (qw * qy - qz * qx);
  if (sinp > 1.0f) sinp = 1.0f; if (sinp < -1.0f) sinp = -1.0f;
  float siny = 2.0f * (qw * qz + qx * qy);
  float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
  if (roll) *roll = atan2f(sinr, cosr) * 180.0f / PI;
  if (pitch) *pitch = asinf(sinp) * 180.0f / PI;
  if (yaw) *yaw = atan2f(siny, cosy) * 180.0f / PI;
}

void sensfusion6GetQuaternion(float *oqw, float *oqx, float *oqy, float *oqz) {
  if (oqw) *oqw = qw; if (oqx) *oqx = qx; if (oqy) *oqy = qy; if (oqz) *oqz = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
  estimatedGravityDirection(NULL, NULL, NULL);
  return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

void powerDistributionInit(void) { powerInitDone = true; (void)powerInitDone; }

void powerDistributionLegacy(const control_t *c, motorPower_t *o) {
  if (!c || !o) return;
  int32_t r = c->roll / 2, p = c->pitch / 2;
  o->m1 = (int32_t)c->thrust - r + p + c->yaw;
  o->m2 = (int32_t)c->thrust - r - p - c->yaw;
  o->m3 = (int32_t)c->thrust + r - p + c->yaw;
  o->m4 = (int32_t)c->thrust + r + p - c->yaw;
}

uint16_t motorForceToPwm(float force) {
  if (force <= 0.0f) return 0;
  float pwm = sqrtf(force) * 65535.0f;
  if (pwm > 65535.0f) pwm = 65535.0f;
  return (uint16_t)lroundf(pwm);
}

void powerDistributionForceTorque(const control_t *c, motorPower_t *o, float armLength, float thrustToTorque) {
  if (!c || !o) return;
  float thrustPart = 0.25f * c->thrustSi;
  float arm = 0.707106781f * armLength;
  float rollPart = (arm != 0.0f) ? 0.25f / arm * c->torqueX : 0.0f;
  float pitchPart = (arm != 0.0f) ? 0.25f / arm * c->torqueY : 0.0f;
  float yawPart = (thrustToTorque != 0.0f) ? 0.25f / thrustToTorque * c->torqueZ : 0.0f;
  float f[4] = {
    thrustPart - rollPart + pitchPart + yawPart,
    thrustPart - rollPart - pitchPart - yawPart,
    thrustPart + rollPart - pitchPart + yawPart,
    thrustPart + rollPart + pitchPart - yawPart
  };
  o->m1 = motorForceToPwm(f[0] < 0.0f ? 0.0f : f[0]);
  o->m2 = motorForceToPwm(f[1] < 0.0f ? 0.0f : f[1]);
  o->m3 = motorForceToPwm(f[2] < 0.0f ? 0.0f : f[2]);
  o->m4 = motorForceToPwm(f[3] < 0.0f ? 0.0f : f[3]);
}

void powerDistributionForce(const control_t *c, motorPower_t *o) {
  if (!c || !o) return;
  int32_t *m[4] = { &o->m1, &o->m2, &o->m3, &o->m4 };
  for (int i = 0; i < 4; i++) {
    float v = c->normalizedForces[i];
    if (v < 0.0f) v = 0.0f; if (v > 1.0f) v = 1.0f;
    *m[i] = (int32_t)lroundf(v * 65535.0f);
  }
}

void powerDistribution(const control_t *c, motorPower_t *o) {
  if (!c || !o) return;
  if (c->controlMode == controlModeLegacy) powerDistributionLegacy(c, o);
  else if (c->controlMode == controlModeForceTorque) powerDistributionForceTorque(c, o, 0.0397f, 0.005964f);
  else if (c->controlMode == controlModeForce) powerDistributionForce(c, o);
}

bool powerDistributionCap(motorPower_t *p, uint16_t maxAllowedThrust, uint16_t idleThrust) {
  if (!p) return false;
  int32_t max = p->m1;
  if (p->m2 > max) max = p->m2; if (p->m3 > max) max = p->m3; if (p->m4 > max) max = p->m4;
  if (max <= (int32_t)maxAllowedThrust) return false;
  int32_t reduction = max - (int32_t)maxAllowedThrust;
  int32_t *m[4] = { &p->m1, &p->m2, &p->m3, &p->m4 };
  for (int i = 0; i < 4; i++) {
    *m[i] -= reduction;
    if (*m[i] < (int32_t)idleThrust) *m[i] = idleThrust;
  }
  return true;
}

float batteryCompensation(float oldVoltage, float supplyVoltage, float alpha) {
  return oldVoltage + alpha * (supplyVoltage - oldVoltage);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t thrust, float nominalVoltage, float actualVoltage) {
  if (actualVoltage <= 0.0f) return thrust;
  int32_t v = (int32_t)lroundf((float)thrust * nominalVoltage / actualVoltage);
  if (v < 0) v = 0; if (v > 65535) v = 65535;
  return (uint16_t)v;
}

void pidInit(PidObject *p, float kp, float ki, float kd, float iLimit, float outputLimit) {
  if (!p || p->initialized) return;
  p->kp = kp; p->ki = ki; p->kd = kd; p->iLimit = iLimit; p->outputLimit = outputLimit;
  p->integral = p->previousError = 0.0f; p->initialized = true;
}

void pidReset(PidObject *p) { if (p) { p->integral = 0.0f; p->previousError = 0.0f; } }

float pidUpdate(PidObject *p, float error, float dt, bool reset) {
  if (!p) return 0.0f;
  if (reset) pidReset(p);
  if (dt <= 0.0f) dt = 0.001f;
  p->integral += error * dt;
  if (p->integral > p->iLimit) p->integral = p->iLimit;
  if (p->integral < -p->iLimit) p->integral = -p->iLimit;
  float out = p->kp * error + p->ki * p->integral + p->kd * (error - p->previousError) / dt;
  p->previousError = error;
  if (p->outputLimit > 0.0f) {
    if (out > p->outputLimit) out = p->outputLimit;
    if (out < -p->outputLimit) out = -p->outputLimit;
  }
  return out;
}

void attitudeControllerInit(void) {
  if (controllerInitDone) return;
  pidInit(&pidRoll, 6, 0, 0, 100, 32767); pidInit(&pidPitch, 6, 0, 0, 100, 32767); pidInit(&pidYaw, 6, 0, 0, 100, 32767);
  pidInit(&pidRollRate, 250, 0, 0, 100, 32767); pidInit(&pidPitchRate, 250, 0, 0, 100, 32767); pidInit(&pidYawRate, 120, 0, 0, 100, 32767);
  controllerInitDone = true;
}

void attitudeControllerResetAll(const attitude_t *a) {
  pidReset(&pidRoll); pidReset(&pidPitch); pidReset(&pidYaw); pidReset(&pidRollRate); pidReset(&pidPitchRate); pidReset(&pidYawRate);
  if (a) desiredYaw = a->yaw;
}
void attitudeControllerResetRoll(void) { pidReset(&pidRoll); pidReset(&pidRollRate); }
void attitudeControllerResetPitch(void) { pidReset(&pidPitch); pidReset(&pidPitchRate); }
void attitudeControllerResetYaw(void) { pidReset(&pidYaw); pidReset(&pidYawRate); }
void controllerPidSetPositionThrust(uint16_t thrust) { injectedPositionThrust = thrust; }

void controllerPid(const state_t *state, const sensorData_t *sensors, const setpoint_t *sp, uint32_t tick, control_t *control) {
  (void)tick;
  if (!state || !sensors || !sp || !control) return;
  attitudeControllerInit();
  float dt = 0.001f;
  if (sp->thrust == 0 && sp->mode.z == modeDisable) {
    memset(control, 0, sizeof(*control)); attitudeControllerResetAll(&state->attitude); desiredYaw = state->attitude.yaw; return;
  }
  float rollDesired = sp->attitude.roll, pitchDesired = sp->attitude.pitch, yawDesired = desiredYaw;
  if (sp->mode.yaw == modeVelocity) desiredYaw = capAngle(desiredYaw + sp->attitudeRate.yaw * dt);
  else if (sp->mode.yaw == modeAbs) desiredYaw = sp->attitude.yaw;
  if (sp->mode.quat == modeAbs) {
    float oldw = qw, oldx = qx, oldy = qy, oldz = qz;
    qw = sp->attitudeQuaternion.qw; qx = sp->attitudeQuaternion.qx; qy = sp->attitudeQuaternion.qy; qz = sp->attitudeQuaternion.qz;
    sensfusion6GetEulerRPY(NULL, NULL, &desiredYaw);
    qw = oldw; qx = oldx; qy = oldy; qz = oldz;
  }
  yawDesired = desiredYaw;
  if (sp->mode.roll == modeVelocity) { rollDesired = sp->attitudeRate.roll; pidReset(&pidRoll); }
  if (sp->mode.pitch == modeVelocity) { pitchDesired = sp->attitudeRate.pitch; pidReset(&pidPitch); }
  float rollRateDesired = (sp->mode.roll == modeVelocity) ? rollDesired : pidUpdate(&pidRoll, capAngle(rollDesired - state->attitude.roll), dt, false);
  float pitchRateDesired = (sp->mode.pitch == modeVelocity) ? pitchDesired : pidUpdate(&pidPitch, capAngle(pitchDesired - state->attitude.pitch), dt, false);
  float yawRateDesired = (sp->mode.yaw == modeVelocity) ? sp->attitudeRate.yaw : pidUpdate(&pidYaw, capAngle(yawDesired - state->attitude.yaw), dt, true);
  control->controlMode = controlModeLegacy;
  control->roll = saturateSignedInt16((int32_t)lroundf(pidUpdate(&pidRollRate, rollRateDesired - sensors->gyro.x, dt, false)));
  control->pitch = saturateSignedInt16((int32_t)lroundf(pidUpdate(&pidPitchRate, pitchRateDesired - (-sensors->gyro.y), dt, false)));
  control->yaw = saturateSignedInt16((int32_t)lroundf(-pidUpdate(&pidYawRate, yawRateDesired - sensors->gyro.z, dt, false)));
  control->thrust = (sp->mode.z == modeDisable) ? sp->thrust : injectedPositionThrust;
}

void rotateYaw(float yawDeg, float inX, float inY, float *outX, float *outY) {
  float r = yawDeg * PI / 180.0f, c = cosf(r), s = sinf(r);
  if (outX) *outX = inX * c - inY * s;
  if (outY) *outY = inX * s + inY * c;
}

void crtpCommanderRpytDecodeSetpoint(setpoint_t *sp, float roll, float pitch, float yaw, uint16_t rawThrust, bool altHold, bool posHold, bool posSetMode, bool plusMode, bool carefree, uint8_t rollPitchMode, uint8_t yawMode, uint8_t activePriority) {
  static bool thrustLocked = true, altModeSet = false;
  if (!sp) return;
  memset(sp, 0, sizeof(*sp));
  if (activePriority == COMMANDER_PRIORITY_DISABLE) thrustLocked = true;
  if (rawThrust == 0) thrustLocked = false;
  if (carefree) { sp->thrust = 0; return; }
  if (plusMode) {
    float nr = (roll - pitch) * 0.707106781f, np = (roll + pitch) * 0.707106781f;
    roll = nr; pitch = np;
  }
  if (altHold) {
    sp->mode.z = modeVelocity; sp->thrust = 0; sp->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f; altModeSet = true; return;
  } else if (altModeSet) {
    sp->mode.z = modeDisable; altModeSet = false;
  }
  if (posHold) {
    sp->mode.x = modeVelocity; sp->mode.y = modeVelocity; sp->mode.roll = modeDisable; sp->mode.pitch = modeDisable;
    sp->velocity.x = pitch / 30.0f; sp->velocity.y = roll / 30.0f; sp->attitude.roll = sp->attitude.pitch = 0.0f;
  } else if (posSetMode && rawThrust != 0) {
    sp->mode.x = sp->mode.y = sp->mode.z = modeAbs; sp->mode.roll = sp->mode.pitch = modeDisable; sp->mode.yaw = modeAbs;
    sp->position.x = -pitch; sp->position.y = roll; sp->position.z = (float)rawThrust / 1000.0f; sp->attitude.yaw = yaw; sp->thrust = 0; return;
  } else {
    sp->mode.roll = rollPitchMode ? modeVelocity : modeAbs; sp->mode.pitch = rollPitchMode ? modeVelocity : modeAbs;
    if (rollPitchMode) { sp->attitudeRate.roll = roll; sp->attitudeRate.pitch = pitch; } else { sp->attitude.roll = roll; sp->attitude.pitch = pitch; }
  }
  sp->mode.yaw = yawMode ? modeVelocity : modeAbs;
  if (yawMode) sp->attitudeRate.yaw = -yaw; else sp->attitude.yaw = yaw;
  sp->mode.z = modeDisable;
  sp->thrust = (thrustLocked || rawThrust < 1000) ? 0 : (rawThrust > 60000 ? 60000 : rawThrust);
}

void supervisorInit(void) {
  supState = supervisorStateInit; supArmed = supCrashed = supTumbled = supFreeFall = supLocked = false;
  supSeenFlight = false; supLastFlightTick = 0;
}

void supervisorSetState(SupervisorState s) {
  bool keep = (s == supervisorStateArming || s == supervisorStateReadyToFly || s == supervisorStateFlying || s == supervisorStateWarningLevelOut || s == supervisorStateLanded);
  if (!keep) supArmed = false;
  if (supAutoArming && s == supervisorStatePreFlChecksPassed) { supArmed = true; supState = supervisorStateArming; return; }
  supState = s;
}

SupervisorState supervisorGetState(void) { return supState; }
bool supervisorCanFly(void) { return supState == supervisorStateReadyToFly || supState == supervisorStateFlying || supState == supervisorStateWarningLevelOut || supState == supervisorStateLanded; }
bool supervisorCanArm(void) { return supState == supervisorStatePreFlChecksPassed; }
bool supervisorIsArmed(void) { return supArmed; }
bool supervisorIsCrashed(void) { return supCrashed; }

bool supervisorRequestArming(void) {
  if (supArmed) return true;
  if (!supervisorCanArm()) return false;
  supArmed = true; supState = supervisorStateArming; return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
  if (supTumbled) return false;
  if (doRecovery) supCrashed = false; else supCrashed = true;
  return true;
}

bool isFlyingCheck(uint32_t tick, const uint16_t ratios[4], uint16_t idleThrust) {
  if (ratios) for (int i = 0; i < 4; i++) if (ratios[i] > idleThrust) { supLastFlightTick = tick; supSeenFlight = true; break; }
  if (!supSeenFlight) return false;
  return (uint32_t)(tick - supLastFlightTick) < 2000u;
}

bool isTumbledCheck(float ax, float ay, float az, float accZ, uint32_t tick) {
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  supervisorLog.accNorm = norm;
  if (supCrashGs > 0.0f && fabsf(norm - 1.0f) > supCrashGs) supCrashed = true;
  if (fabsf(ax) < supFreeFallThreshold && fabsf(ay) < supFreeFallThreshold && fabsf(az) < supFreeFallThreshold) {
    supFreeFall = true; supTiltStartTick = 0; return false;
  }
  supFreeFall = false;
  if (!supTumbleEnabled) return false;
  uint32_t timeout = (accZ < supUpsideDownThreshold) ? supUpsideDownTimeout : supTiltTimeout;
  if (accZ < supTiltThreshold) {
    if (supTiltStartTick == 0) supTiltStartTick = tick ? tick : 1;
    if ((uint32_t)(tick - supTiltStartTick) >= timeout) supTumbled = true;
  } else {
    supTiltStartTick = 0; supTumbled = false;
  }
  return supTumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick) {
  if (lastNotificationTick == 0) return true;
  return (uint32_t)(currentTick - lastNotificationTick) <= 1000u;
}

bool supervisorIsPreflightTimeout(uint32_t currentTick, uint32_t startTick, uint32_t timeoutMs) {
  return startTick != 0 && (uint32_t)(currentTick - startTick) >= timeoutMs;
}

bool supervisorIsLandingTimeout(uint32_t currentTick, uint32_t landingTick, uint32_t timeoutMs) {
  return landingTick != 0 && (uint32_t)(currentTick - landingTick) >= timeoutMs;
}

void updateAndPopulateConditions(uint32_t currentTick) { (void)currentTick; supervisorLog.info = supervisorGetInfoBitfield(); }

bool isRPMatArmingValid(const uint16_t rpm[4], uint16_t minRpm, uint16_t maxRpm, uint32_t tick, uint32_t requiredMs) {
  bool ok = rpm != NULL;
  for (int i = 0; ok && i < 4; i++) ok = rpm[i] >= minRpm && rpm[i] <= maxRpm;
  if (!ok) { supRpmValidStart = 0; return false; }
  if (supRpmValidStart == 0) supRpmValidStart = tick ? tick : 1;
  return (uint32_t)(tick - supRpmValidStart) >= requiredMs;
}

void supervisorSetSensorData(float ax, float ay, float az, float accZ) { supAx = ax; supAy = ay; supAz = az; supAccZ = accZ; }
void supervisorSetMotorRatios(const uint16_t r[4]) { if (r) memcpy(supRatios, r, sizeof(supRatios)); }
void supervisorSetMotorRPMs(const uint16_t r[4]) { if (r) memcpy(supRpms, r, sizeof(supRpms)); }
void supervisorConfigureSafety(float crashGs, float freeFall, float tilt, float upsideDown, uint32_t tiltTimeoutMs, uint32_t upsideDownTimeoutMs, bool tumbleEnabled, bool autoArming) {
  supCrashGs = crashGs; supFreeFallThreshold = freeFall; supTiltThreshold = tilt; supUpsideDownThreshold = upsideDown;
  supTiltTimeout = tiltTimeoutMs; supUpsideDownTimeout = upsideDownTimeoutMs; supTumbleEnabled = tumbleEnabled; supAutoArming = autoArming;
}

void supervisorUpdate(uint32_t tick, uint32_t step) {
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, step)) return;
  bool flying = isFlyingCheck(tick, supRatios, 0);
  bool tumbled = isTumbledCheck(supAx, supAy, supAz, supAccZ, tick);
  (void)flying; (void)tumbled; updateAndPopulateConditions(tick);
}

void supervisorOverrideSetpoint(setpoint_t *sp) {
  if (!sp) return;
  if (supState == supervisorStateWarningLevelOut) {
    sp->mode.x = modeDisable; sp->mode.y = modeDisable; sp->mode.roll = modeAbs; sp->mode.pitch = modeAbs; sp->attitude.roll = 0.0f; sp->attitude.pitch = 0.0f; sp->mode.yaw = modeVelocity; sp->attitudeRate.yaw = 0.0f;
  } else if (!(supState == supervisorStateArming || supState == supervisorStateReadyToFly || supState == supervisorStateFlying || supState == supervisorStateLanded)) {
    memset(sp, 0, sizeof(*sp));
  }
  if (supCrashed || supTumbled || supFreeFall || supLocked) memset(sp, 0, sizeof(*sp));
}

bool supervisorAreMotorsAllowedToRun(void) {
  return supState == supervisorStateArming || supState == supervisorStateReadyToFly || supState == supervisorStateFlying || supState == supervisorStateWarningLevelOut || supState == supervisorStateLanded;
}

uint32_t supervisorGetInfoBitfield(void) {
  uint32_t b = 0;
  if (supervisorCanArm()) b |= 1u << 0; if (supArmed) b |= 1u << 1; if (supAutoArming) b |= 1u << 2;
  if (supervisorCanFly()) b |= 1u << 3; if (isFlyingCheck(0, NULL, 0)) b |= 1u << 4; if (supTumbled) b |= 1u << 5;
  if (supLocked) b |= 1u << 6; if (supCrashed) b |= 1u << 7;
  supervisorLog.info = b; supervisorLog.canArm = supervisorCanArm(); supervisorLog.canFly = supervisorCanFly(); supervisorLog.isTumbled = supTumbled; supervisorLog.isCrashed = supCrashed;
  return b;
}

static EstimatorMeasurement estQ[16];
static uint8_t estHead, estTail, estCount;

bool estimatorEnqueue(const EstimatorMeasurement *m) {
  if (!m || estCount >= 16) return false;
  estQ[estTail] = *m; estTail = (uint8_t)((estTail + 1) & 15); estCount++; return true;
}

bool estimatorDequeue(EstimatorMeasurement *m) {
  if (!m || estCount == 0) return false;
  *m = estQ[estHead]; estHead = (uint8_t)((estHead + 1) & 15); estCount--; return true;
}

void estimatorComplementary(state_t *state, const sensorData_t *sensors, uint32_t step, float dt) {
  EstimatorMeasurement m;
  while (estimatorDequeue(&m)) {
    if (state && m.type < 4) state->position.x = m.v[0];
  }
  if (!state || !sensors) return;
  if (RATE_DO_EXECUTE(RATE_250_HZ, step)) {
    sensfusion6UpdateQ(sensors->gyro.x, sensors->gyro.y, sensors->gyro.z, sensors->acc.x, sensors->acc.y, sensors->acc.z, dt);
    sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);
    sensfusion6GetQuaternion(&state->attitudeQuaternion.qw, &state->attitudeQuaternion.qx, &state->attitudeQuaternion.qy, &state->attitudeQuaternion.qz);
    state->velocity.z += sensfusion6GetAccZWithoutGravity(sensors->acc.x, sensors->acc.y, sensors->acc.z) * 9.81f * dt;
  }
  if (RATE_DO_EXECUTE(RATE_100_HZ, step)) {
    state->position.x += state->velocity.x * dt; state->position.y += state->velocity.y * dt; state->position.z += state->velocity.z * dt;
  }
}

void commanderInit(void) { if (!commanderInitDone) { memset(&commanderSp, 0, sizeof(commanderSp)); commanderPriority = COMMANDER_PRIORITY_LOWEST; commanderInitDone = true; } }

bool commanderSetSetpoint(const setpoint_t *sp, uint8_t priority, uint32_t tick) {
  commanderInit();
  if (!sp) return false;
  if (priority == COMMANDER_PRIORITY_DISABLE || priority >= commanderPriority) {
    commanderSp = *sp; commanderPriority = priority; commanderLastUpdate = tick; return true;
  }
  return false;
}

void commanderGetSetpoint(setpoint_t *sp) { if (sp) *sp = commanderSp; }
void commanderRelaxPriority(void) { commanderPriority = COMMANDER_PRIORITY_LOWEST; }
uint32_t commanderGetInactivityTime(uint32_t tick) { return tick - commanderLastUpdate; }
uint8_t commanderGetActivePriority(void) { return commanderPriority; }

void stabilizerInit(void) {
  if (stabilizerInitDone) return;
  sensfusion6Init(); attitudeControllerInit(); powerDistributionInit(); commanderInit(); crtpInit();
  stabilizerInitDone = true;
}

void stabilizerSubmitHighLevelSetpoint(const setpoint_t *sp) { if (sp) { highlevelSp = *sp; highlevelPending = true; } }

void stabilizerTask(uint32_t tick) {
  stabilizerInit();
  stabilizerStep++;
  if (highlevelPending) { commanderSetSetpoint(&highlevelSp, COMMANDER_PRIORITY_HIGHLEVEL, tick); highlevelPending = false; }
}

uint32_t quatcompress(float w, float x, float y, float z) {
  float q[4] = { w, x, y, z };
  int largest = 0;
  for (int i = 1; i < 4; i++) if (fabsf(q[i]) > fabsf(q[largest])) largest = i;
  uint32_t out = (uint32_t)largest << 30;
  int shift = 0;
  for (int i = 0; i < 4; i++) if (i != largest) {
    int32_t v = (int32_t)lroundf((q[i] + 1.0f) * 511.5f);
    if (v < 0) v = 0; if (v > 1023) v = 1023;
    out |= ((uint32_t)v & 0x3ffu) << shift; shift += 10;
  }
  return out;
}

void compressState(const state_t *s, const sensorData_t *sen, int32_t out[13]) {
  if (!s || !sen || !out) return;
  out[0] = (int32_t)lroundf(s->position.x * 1000.0f); out[1] = (int32_t)lroundf(s->position.y * 1000.0f); out[2] = (int32_t)lroundf(s->position.z * 1000.0f);
  out[3] = (int32_t)lroundf(s->velocity.x * 1000.0f); out[4] = (int32_t)lroundf(s->velocity.y * 1000.0f); out[5] = (int32_t)lroundf(s->velocity.z * 1000.0f);
  out[6] = (int32_t)lroundf(sen->acc.x * 9810.0f); out[7] = (int32_t)lroundf(sen->acc.y * 9810.0f); out[8] = (int32_t)lroundf((sen->acc.z + 1.0f) * 9810.0f);
  out[9] = (int32_t)lroundf(sen->gyro.x * PI / 180.0f * 1000.0f); out[10] = (int32_t)lroundf(-sen->gyro.y * PI / 180.0f * 1000.0f); out[11] = (int32_t)lroundf(sen->gyro.z * PI / 180.0f * 1000.0f);
  out[12] = (int32_t)quatcompress(s->attitudeQuaternion.qw, s->attitudeQuaternion.qx, s->attitudeQuaternion.qy, s->attitudeQuaternion.qz);
}

bool rateSupervisorValidate(uint32_t hz) { return hz >= 997u && hz <= 1003u; }

static bool propReq, batReq, propActive, batActive;
static uint32_t batTick, batRestartTick;
static float idleVoltage, minLoadedVoltage = 999.0f;

void startPropTest(void) { propReq = true; }
void startBatTest(void) { batReq = true; }

bool healthShallWeRunTest(void) {
  if (propReq) { propReq = false; propActive = true; healthLog.motorPass = 0; healthLog.motorTestCount = 0; return true; }
  if (batReq) { batReq = false; batActive = true; batTick = 0; minLoadedVoltage = 999.0f; return true; }
  return propActive || batActive;
}

bool evaluatePropTest(float value, float low, float high) {
  if (high == 0.0f) return true;
  return value >= low && value <= high;
}

void healthRunTests(float ax, float ay, float az, float voltage) {
  (void)ax; (void)ay; (void)az;
  if (propActive) {
    healthLog.motorPass |= (uint8_t)(1u << (healthLog.motorTestCount & 3u));
    healthLog.motorTestCount++;
    if (healthLog.motorTestCount >= 4) propActive = false;
  }
  if (batActive) {
    batTick++;
    if (batTick == 1) idleVoltage = voltage;
    if (batTick >= 2 && batTick <= 49 && voltage < minLoadedVoltage) minLoadedVoltage = voltage;
    if (batTick >= 50) {
      healthLog.batterySag = idleVoltage - minLoadedVoltage;
      healthLog.batteryPass = healthLog.batterySag <= 0.5f;
      batActive = false;
    }
  }
}

void restartBatTest(uint32_t tick) { if (batRestartTick == 0) batRestartTick = tick; else if ((uint32_t)(tick - batRestartTick) >= 2000u) startBatTest(); }

float variance(const float *v, size_t n) {
  if (!v || n == 0) return 0.0f;
  float sum = 0.0f, sumSq = 0.0f;
  for (size_t i = 0; i < n; i++) { sum += v[i]; sumSq += v[i] * v[i]; }
  return sumSq - (sum * sum / (float)n);
}

typedef struct { CRTPPacket q[200]; uint16_t head, tail, count; } TxQ;
typedef struct { CRTPPacket q[16]; uint8_t head, tail, count; bool created; } RxQ;
static TxQ txQ;
static RxQ rxQ[CRTP_NBR_OF_PORTS];
static CrtpPortCallback portCb[CRTP_NBR_OF_PORTS];
static CRTPLink *curLink;
static uint32_t lastTxRetry, statsTick;
static uint16_t rxCnt, txCnt, rxRate, txRate;

static bool qPushRx(RxQ *q, const CRTPPacket *p) {
  if (!q || !p || !q->created || q->count >= 16) return false;
  q->q[q->tail] = *p; q->tail = (uint8_t)((q->tail + 1) & 15); q->count++; return true;
}

void crtpInit(void) { if (!crtpInitDone) { memset(&txQ, 0, sizeof(txQ)); memset(rxQ, 0, sizeof(rxQ)); crtpInitDone = true; } }
bool crtpCreateRxQueue(uint8_t port) { crtpInit(); if (port >= CRTP_NBR_OF_PORTS || rxQ[port].created) return false; rxQ[port].created = true; return true; }

bool crtpSendPacket(const CRTPPacket *p) {
  crtpInit(); if (!p || txQ.count >= 200) return false;
  txQ.q[txQ.tail] = *p; txQ.tail = (uint16_t)((txQ.tail + 1) % 200); txQ.count++; return true;
}

bool crtpSendPacketBlock(const CRTPPacket *p) { return crtpSendPacket(p); }

bool crtpReceivePacket(uint8_t port, CRTPPacket *p) {
  crtpInit(); if (port >= CRTP_NBR_OF_PORTS || !p || rxQ[port].count == 0) return false;
  *p = rxQ[port].q[rxQ[port].head]; rxQ[port].head = (uint8_t)((rxQ[port].head + 1) & 15); rxQ[port].count--; return true;
}

bool crtpReceivePacketBlock(uint8_t port, CRTPPacket *p) { return crtpReceivePacket(port, p); }
bool crtpReceivePacketWait(uint8_t port, CRTPPacket *p, uint32_t timeoutMs) { (void)timeoutMs; return crtpReceivePacket(port, p); }

void crtpRxTask(void) {
  CRTPPacket p;
  if (!curLink || !curLink->receive) return;
  while (curLink->receive(&p)) {
    rxCnt++;
    if (p.port < CRTP_NBR_OF_PORTS) {
      qPushRx(&rxQ[p.port], &p);
      if (portCb[p.port]) portCb[p.port](&p);
    }
  }
}

void crtpTxTask(uint32_t tick) {
  if (!curLink || !curLink->send || txQ.count == 0) return;
  if (lastTxRetry && (uint32_t)(tick - lastTxRetry) < 10u) return;
  CRTPPacket p = txQ.q[txQ.head];
  if (curLink->send(&p)) { txQ.head = (uint16_t)((txQ.head + 1) % 200); txQ.count--; txCnt++; lastTxRetry = 0; }
  else lastTxRetry = tick ? tick : 1;
}

void crtpSetLink(CRTPLink *link) {
  if (curLink && curLink->disable) curLink->disable();
  curLink = link;
  if (curLink && curLink->enable) curLink->enable();
}

void crtpReset(void) { memset(&txQ, 0, sizeof(txQ)); if (curLink && curLink->reset) curLink->reset(); }
bool crtpIsConnected(void) { return (curLink && curLink->isConnected) ? curLink->isConnected() : true; }
uint16_t crtpGetFreeTxQueuePackets(void) { crtpInit(); return (uint16_t)(200u - txQ.count); }

bool crtpRegisterPortCB(uint8_t port, CrtpPortCallback cb) {
  if (port >= CRTP_NBR_OF_PORTS) return false;
  portCb[port] = cb; return true;
}

void crtpUpdateStats(uint32_t tick) {
  if (statsTick == 0) statsTick = tick;
  if ((uint32_t)(tick - statsTick) >= 500u) {
    rxRate = (uint16_t)(rxCnt * 2u); txRate = (uint16_t)(txCnt * 2u);
    rxCnt = txCnt = 0; statsTick = tick;
  }
}
uint16_t crtpGetRxRate(void) { return rxRate; }
uint16_t crtpGetTxRate(void) { return txRate; }

size_t deckDiscovery(const uint8_t *i2cAddresses, size_t i2cCount, const uint64_t *roms, size_t romCount, uint64_t *out, size_t capacity) {
  if (!out || capacity == 0) return 0;
  size_t n = 0;
  for (size_t i = 0; i < i2cCount && n < capacity; i++) {
    uint64_t id = i2cAddresses ? (uint64_t)i2cAddresses[i] : 0u;
    bool dup = false; for (size_t j = 0; j < n; j++) if (out[j] == id) dup = true;
    if (!dup) out[n++] = id;
  }
  for (size_t i = 0; i < romCount && n < capacity; i++) {
    uint64_t id = roms ? roms[i] : 0u;
    bool dup = false; for (size_t j = 0; j < n; j++) if (out[j] == id) dup = true;
    if (!dup) out[n++] = id;
  }
  return n;
}

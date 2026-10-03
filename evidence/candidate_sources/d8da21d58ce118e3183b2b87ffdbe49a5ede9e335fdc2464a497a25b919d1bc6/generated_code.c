#include "generated_code.h"

#include <math.h>
#include <string.h>
#include <limits.h>

stateEstimate_t stateEstimate;
logAxis3_t gyro;
logAxis3_t acc;
baroLog_t baro;
motorLog_t motor;
sensfusion6Log_t sensfusion6Log;
supervisorLog_t supervisorLog;
healthLog_t healthLog;

PidObject pidRoll, pidPitch, pidYaw, pidRollRate, pidPitchRate, pidYawRate;

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float baseZacc = 0.0f;
bool calibrated = false;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
supervisorState_t supervisorState = supervisorStateInit;

static bool sensfusionInitialized = false;
static bool powerInitialized = false;
static bool attitudeInitialized = false;
static bool supervisorInitialized = false;
static bool estimatorInitialized = false;
static bool commanderInitialized = false;
static bool stabilizerInitialized = false;
static bool healthInitialized = false;
static bool crtpInitialized = false;

static uint32_t monotonicTick = 0;

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

static void normalizeQuaternion(void) {
  float n = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
  if (n <= 0.0f) {
    qw = 1.0f; qx = qy = qz = 0.0f;
    return;
  }
  qw /= n; qx /= n; qy /= n; qz /= n;
}

void sensfusion6Init(void) {
  if (sensfusionInitialized) return;
  qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
  integralFBx = integralFBy = integralFBz = 0.0f;
  baseZacc = 0.0f;
  calibrated = false;
  sensfusionInitialized = true;
  sensfusion6Log.initialized = true;
}

bool sensfusion6Test(void) {
  return sensfusionInitialized;
}

void estimatedGravityDirection(float *gxOut, float *gyOut, float *gzOut) {
  float gxv = 2.0f * (qx * qz - qw * qy);
  float gyv = 2.0f * (qw * qx + qy * qz);
  float gzv = qw * qw - qx * qx - qy * qy + qz * qz;
  gravityX = gxv; gravityY = gyv; gravityZ = gzv;
  if (gxOut) *gxOut = gxv;
  if (gyOut) *gyOut = gyv;
  if (gzOut) *gzOut = gzv;
  sensfusion6Log.gravityX = gxv;
  sensfusion6Log.gravityY = gyv;
  sensfusion6Log.gravityZ = gzv;
}

void sensfusion6UpdateQ(float gxDeg, float gyDeg, float gzDeg, float ax, float ay, float az, float dt) {
  const float twoKp = 0.8f;
  const float twoKi = 0.002f;
  float gxRad = gxDeg * PI / 180.0f;
  float gyRad = gyDeg * PI / 180.0f;
  float gzRad = gzDeg * PI / 180.0f;

  if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
    float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    if (recipNorm > 0.0f) {
      ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
      float vx, vy, vz;
      estimatedGravityDirection(&vx, &vy, &vz);
      float ex = ay * vz - az * vy;
      float ey = az * vx - ax * vz;
      float ez = ax * vy - ay * vx;

      if (twoKi > 0.0f) {
        integralFBx += twoKi * ex * dt;
        integralFBy += twoKi * ey * dt;
        integralFBz += twoKi * ez * dt;
        gxRad += integralFBx;
        gyRad += integralFBy;
        gzRad += integralFBz;
      } else {
        integralFBx = integralFBy = integralFBz = 0.0f;
      }

      gxRad += twoKp * ex;
      gyRad += twoKp * ey;
      gzRad += twoKp * ez;

      if (!calibrated) {
        baseZacc = ax * vx + ay * vy + az * vz;
        calibrated = true;
      }
    }
  }

  gxRad *= 0.5f * dt;
  gyRad *= 0.5f * dt;
  gzRad *= 0.5f * dt;

  float qa = qw, qb = qx, qc = qy;
  qw += -qb * gxRad - qc * gyRad - qz * gzRad;
  qx += qa * gxRad + qc * gzRad - qz * gyRad;
  qy += qa * gyRad - qb * gzRad + qz * gxRad;
  qz += qa * gzRad + qb * gyRad - qc * gxRad;
  normalizeQuaternion();
  estimatedGravityDirection(NULL, NULL, NULL);

  sensfusion6Log.q0 = qw; sensfusion6Log.q1 = qx; sensfusion6Log.q2 = qy; sensfusion6Log.q3 = qz;
  sensfusion6Log.accZ = sensfusion6GetAccZ(ax, ay, az);
  sensfusion6Log.calibrated = calibrated;
}

void sensfusion6GetEulerRPY(float *roll, float *pitch, float *yaw) {
  normalizeQuaternion();
  float sinr = 2.0f * (qw * qx + qy * qz);
  float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
  float sinp = 2.0f * (qw * qy - qz * qx);
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  float siny = 2.0f * (qw * qz + qx * qy);
  float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
  if (roll) *roll = atan2f(sinr, cosr) * 180.0f / PI;
  if (pitch) *pitch = asinf(sinp) * 180.0f / PI;
  if (yaw) *yaw = atan2f(siny, cosy) * 180.0f / PI;
  estimatedGravityDirection(NULL, NULL, NULL);
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
  powerInitialized = true;
}

void powerDistributionLegacy(const control_t *control, motorPower_t *mp) {
  if (!control || !mp) return;
  int32_t r = control->roll / 2;
  int32_t p = control->pitch / 2;
  int32_t t = control->thrust;
  int32_t y = control->yaw;
  mp->m1 = t - r + p + y;
  mp->m2 = t - r - p - y;
  mp->m3 = t + r - p + y;
  mp->m4 = t + r + p - y;
}

uint16_t motorForceToPwm(float force) {
  if (force <= 0.0f) return 0;
  float pwm = sqrtf(force) * 65535.0f;
  if (pwm > 65535.0f) pwm = 65535.0f;
  return (uint16_t)lroundf(pwm);
}

void powerDistributionForceTorque(const control_t *control, motorPower_t *mp, float armLength, float thrustToTorque) {
  if (!control || !mp) return;
  float thrustPart = 0.25f * control->thrustSi;
  float arm = 0.707106781f * armLength;
  float rollPart = (arm != 0.0f) ? (0.25f / arm * control->torqueX) : 0.0f;
  float pitchPart = (arm != 0.0f) ? (0.25f / arm * control->torqueY) : 0.0f;
  float yawPart = (thrustToTorque != 0.0f) ? (0.25f / thrustToTorque * control->torqueZ) : 0.0f;
  float f[4];
  f[0] = thrustPart - rollPart + pitchPart + yawPart;
  f[1] = thrustPart - rollPart - pitchPart - yawPart;
  f[2] = thrustPart + rollPart - pitchPart + yawPart;
  f[3] = thrustPart + rollPart + pitchPart - yawPart;
  mp->m1 = motorForceToPwm(f[0] < 0.0f ? 0.0f : f[0]);
  mp->m2 = motorForceToPwm(f[1] < 0.0f ? 0.0f : f[1]);
  mp->m3 = motorForceToPwm(f[2] < 0.0f ? 0.0f : f[2]);
  mp->m4 = motorForceToPwm(f[3] < 0.0f ? 0.0f : f[3]);
}

void powerDistributionForce(const control_t *control, motorPower_t *mp) {
  if (!control || !mp) return;
  int32_t *m[4] = { &mp->m1, &mp->m2, &mp->m3, &mp->m4 };
  for (int i = 0; i < 4; ++i) {
    float v = control->normalizedForces[i];
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    *m[i] = (int32_t)lroundf(v * 65535.0f);
  }
}

void powerDistribution(const control_t *control, motorPower_t *mp) {
  if (!control || !mp) return;
  if (control->controlMode == controlModeLegacy) powerDistributionLegacy(control, mp);
  else if (control->controlMode == controlModeForceTorque) powerDistributionForceTorque(control, mp, 0.0397f, 0.005964f);
  else if (control->controlMode == controlModeForce) powerDistributionForce(control, mp);
}

static void capMin(int32_t *v, uint16_t idle) {
  if (*v < (int32_t)idle) *v = idle;
}

bool powerDistributionCap(motorPower_t *mp, uint16_t maxAllowedThrust, uint16_t idleThrust) {
  if (!mp) return false;
  int32_t max = mp->m1;
  if (mp->m2 > max) max = mp->m2;
  if (mp->m3 > max) max = mp->m3;
  if (mp->m4 > max) max = mp->m4;
  if (max > (int32_t)maxAllowedThrust) {
    int32_t reduction = max - (int32_t)maxAllowedThrust;
    mp->m1 -= reduction; mp->m2 -= reduction; mp->m3 -= reduction; mp->m4 -= reduction;
    capMin(&mp->m1, idleThrust); capMin(&mp->m2, idleThrust);
    capMin(&mp->m3, idleThrust); capMin(&mp->m4, idleThrust);
    return true;
  }
  capMin(&mp->m1, idleThrust); capMin(&mp->m2, idleThrust);
  capMin(&mp->m3, idleThrust); capMin(&mp->m4, idleThrust);
  return false;
}

float batteryCompensation(float oldVoltage, float supplyVoltage, float alpha) {
  return oldVoltage + alpha * (supplyVoltage - oldVoltage);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t thrust, float actualVoltage, float nominalVoltage) {
  if (actualVoltage <= 0.0f) return thrust;
  float r = roundf((float)thrust * nominalVoltage / actualVoltage);
  if (r < 0.0f) r = 0.0f;
  if (r > 65535.0f) r = 65535.0f;
  return (uint16_t)r;
}

void pidInit(PidObject *pid, float kp, float ki, float kd, float iLimit, float outputLimit) {
  if (!pid) return;
  pid->kp = kp; pid->ki = ki; pid->kd = kd; pid->iLimit = iLimit; pid->outputLimit = outputLimit;
  pid->integral = 0.0f; pid->previousError = 0.0f; pid->initialized = true;
}

void pidReset(PidObject *pid) {
  if (!pid) return;
  pid->integral = 0.0f;
  pid->previousError = 0.0f;
}

float pidUpdate(PidObject *pid, float error, float dt, bool reset) {
  if (!pid || dt <= 0.0f) return 0.0f;
  if (reset) pidReset(pid);
  pid->integral += error * dt;
  if (pid->iLimit > 0.0f) {
    if (pid->integral > pid->iLimit) pid->integral = pid->iLimit;
    if (pid->integral < -pid->iLimit) pid->integral = -pid->iLimit;
  }
  float derivative = (error - pid->previousError) / dt;
  pid->previousError = error;
  float out = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
  if (pid->outputLimit > 0.0f) {
    if (out > pid->outputLimit) out = pid->outputLimit;
    if (out < -pid->outputLimit) out = -pid->outputLimit;
  }
  return out;
}

void attitudeControllerInit(void) {
  if (attitudeInitialized) return;
  pidInit(&pidRoll, 6.0f, 0.0f, 0.0f, 20.0f, 360.0f);
  pidInit(&pidPitch, 6.0f, 0.0f, 0.0f, 20.0f, 360.0f);
  pidInit(&pidYaw, 6.0f, 0.0f, 0.0f, 20.0f, 360.0f);
  pidInit(&pidRollRate, 250.0f, 500.0f, 2.5f, 33.3f, 32767.0f);
  pidInit(&pidPitchRate, 250.0f, 500.0f, 2.5f, 33.3f, 32767.0f);
  pidInit(&pidYawRate, 120.0f, 16.7f, 0.0f, 166.7f, 32767.0f);
  attitudeInitialized = true;
}

void attitudeControllerResetAll(float roll, float pitch, float yaw) {
  (void)roll; (void)pitch; (void)yaw;
  pidReset(&pidRoll); pidReset(&pidPitch); pidReset(&pidYaw);
  pidReset(&pidRollRate); pidReset(&pidPitchRate); pidReset(&pidYawRate);
}

void attitudeControllerResetRoll(void) { pidReset(&pidRoll); }
void attitudeControllerResetPitch(void) { pidReset(&pidPitch); }
void attitudeControllerResetYaw(void) { pidReset(&pidYaw); }

static float positionThrust = 0.0f;
static bool positionThrustOverride = false;
static float desiredYaw = 0.0f;
static float yawMaxDelta = 0.0f;

void controllerPidSetPositionThrust(float thrust) {
  positionThrust = thrust;
  positionThrustOverride = true;
}

void controllerPidClearPositionThrustOverride(void) {
  positionThrustOverride = false;
}

static float defaultPositionThrust(const State *state, const Setpoint *sp) {
  float ez = sp->position.z - state->position.z;
  float evz = sp->velocity.z - state->velocity.z;
  float out = 10000.0f + 12000.0f * ez + 5000.0f * evz;
  if (out < 0.0f) out = 0.0f;
  if (out > 65535.0f) out = 65535.0f;
  return out;
}

void controllerPid(const SensorData *sensors, const State *state, const Setpoint *sp, control_t *control, float dt) {
  if (!sensors || !state || !sp || !control) return;
  attitudeControllerInit();
  memset(control, 0, sizeof(*control));
  control->controlMode = controlModeLegacy;

  uint16_t thrust = sp->thrust;
  if (sp->mode.z != modeDisable) {
    thrust = (uint16_t)lroundf(positionThrustOverride ? positionThrust : defaultPositionThrust(state, sp));
  }

  if (thrust == 0) {
    attitudeControllerResetAll(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
    desiredYaw = state->attitude.yaw;
    return;
  }

  float targetRoll = sp->attitude.roll;
  float targetPitch = sp->attitude.pitch;
  float targetYawRate = sp->attitudeRate.yaw;

  if (sp->mode.yaw == modeVelocity) {
    desiredYaw = capAngle(desiredYaw + sp->attitudeRate.yaw * dt);
  } else if (sp->mode.quat == modeAbs) {
    float save0 = qw, save1 = qx, save2 = qy, save3 = qz;
    qw = sp->attitudeQuaternion.q0; qx = sp->attitudeQuaternion.q1; qy = sp->attitudeQuaternion.q2; qz = sp->attitudeQuaternion.q3;
    sensfusion6GetEulerRPY(NULL, NULL, &desiredYaw);
    qw = save0; qx = save1; qy = save2; qz = save3;
  } else if (sp->mode.yaw == modeAbs) {
    desiredYaw = sp->attitude.yaw;
  }

  if (yawMaxDelta != 0.0f) {
    float d = capAngle(desiredYaw - state->attitude.yaw);
    if (d > yawMaxDelta) desiredYaw = capAngle(state->attitude.yaw + yawMaxDelta);
    if (d < -yawMaxDelta) desiredYaw = capAngle(state->attitude.yaw - yawMaxDelta);
  }

  float rollRateSet, pitchRateSet, yawRateSet;
  if (sp->mode.roll == modeVelocity) {
    rollRateSet = sp->attitudeRate.roll;
    pidReset(&pidRoll);
  } else {
    rollRateSet = pidUpdate(&pidRoll, capAngle(targetRoll - state->attitude.roll), dt, false);
  }

  if (sp->mode.pitch == modeVelocity) {
    pitchRateSet = sp->attitudeRate.pitch;
    pidReset(&pidPitch);
  } else {
    pitchRateSet = pidUpdate(&pidPitch, capAngle(targetPitch - state->attitude.pitch), dt, false);
  }

  if (sp->mode.yaw == modeVelocity) {
    yawRateSet = targetYawRate;
  } else {
    yawRateSet = pidUpdate(&pidYaw, capAngle(desiredYaw - state->attitude.yaw), dt, true);
  }

  control->roll = saturateSignedInt16((int32_t)lroundf(pidUpdate(&pidRollRate, rollRateSet - sensors->gyro.x, dt, false)));
  control->pitch = saturateSignedInt16((int32_t)lroundf(pidUpdate(&pidPitchRate, pitchRateSet - (-sensors->gyro.y), dt, false)));
  control->yaw = saturateSignedInt16((int32_t)lroundf(-pidUpdate(&pidYawRate, yawRateSet - sensors->gyro.z, dt, false)));
  control->thrust = thrust;
}

void rotateYaw(float inX, float inY, float yawDeg, float *outX, float *outY) {
  float r = yawDeg * PI / 180.0f;
  float c = cosf(r), s = sinf(r);
  if (outX) *outX = inX * c - inY * s;
  if (outY) *outY = inX * s + inY * c;
}

void crtpCommanderRpytDecodeSetpoint(Setpoint *sp, float roll, float pitch, float yaw, uint16_t rawThrust,
                                     bool altHold, bool posHold, bool posSetMode, rpMode_t rpMode, yawMode_t yMode,
                                     bool plusMode, bool carefree, commanderPriority_t activePriority) {
  static bool thrustLocked = true;
  static bool altModeSet = false;
  if (!sp) return;
  memset(sp, 0, sizeof(*sp));
  if (carefree) {
    sp->mode.x = sp->mode.y = sp->mode.z = modeDisable;
    return;
  }
  if (activePriority == COMMANDER_PRIORITY_DISABLE) thrustLocked = true;
  if (rawThrust == 0) thrustLocked = false;

  if (plusMode) {
    float nr = (roll - pitch) * 0.707106781f;
    float np = (roll + pitch) * 0.707106781f;
    roll = nr; pitch = np;
  }

  if (altHold) {
    sp->mode.z = modeVelocity;
    sp->thrust = 0;
    sp->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
    altModeSet = true;
    return;
  } else if (altModeSet) {
    sp->mode.z = modeDisable;
    altModeSet = false;
  }

  if (posHold) {
    sp->mode.x = modeVelocity; sp->mode.y = modeVelocity;
    sp->mode.roll = modeDisable; sp->mode.pitch = modeDisable;
    sp->velocity.x = pitch / 30.0f;
    sp->velocity.y = roll / 30.0f;
    sp->attitude.roll = 0.0f; sp->attitude.pitch = 0.0f;
    sp->thrust = (thrustLocked || rawThrust < 1000) ? 0 : (rawThrust > 60000 ? 60000 : rawThrust);
    return;
  }

  if (posSetMode && rawThrust != 0) {
    sp->mode.x = sp->mode.y = sp->mode.z = modeAbs;
    sp->mode.roll = sp->mode.pitch = modeDisable;
    sp->mode.yaw = modeAbs;
    sp->position.x = -pitch;
    sp->position.y = roll;
    sp->position.z = (float)rawThrust / 1000.0f;
    sp->attitude.yaw = yaw;
    sp->thrust = 0;
    return;
  }

  if (rpMode == stabilizerModeRollPitchRate) {
    sp->mode.roll = modeVelocity; sp->mode.pitch = modeVelocity;
    sp->attitudeRate.roll = roll; sp->attitudeRate.pitch = pitch;
  } else {
    sp->mode.roll = modeAbs; sp->mode.pitch = modeAbs;
    sp->attitude.roll = roll; sp->attitude.pitch = pitch;
  }

  if (yMode == yawModeRate) {
    sp->mode.yaw = modeVelocity;
    sp->attitudeRate.yaw = -yaw;
  } else {
    sp->mode.yaw = modeAbs;
    sp->attitude.yaw = yaw;
  }

  sp->thrust = (thrustLocked || rawThrust < 1000) ? 0 : (rawThrust > 60000 ? 60000 : rawThrust);
}

static SensorData supervisorSensors;
static uint16_t supervisorRatios[4];
static uint16_t supervisorRPMs[4];
static bool supervisorArming = false;
static bool supervisorCrash = false;
static bool supervisorTumbled = false;
static bool supervisorFreeFalling = false;
static bool supervisorLocked = false;
static bool supervisorAutoArming = false;
static bool supervisorTrajectoryFlying = false;
static bool supervisorTrajectoryFinished = false;
static bool supervisorTrajectoryDisabled = true;
static bool supervisorDeckFault = false;
static uint32_t supervisorConditions = 0;
static float cfgCrashGs = 0.0f, cfgFreeFall = 0.1f, cfgTiltZ = 0.5f, cfgInvertedZ = -0.5f;
static uint32_t cfgTumbleTimeout = 1000, cfgInvertedTimeout = 100;
static bool cfgTumbleEnabled = true;
static uint32_t cfgSpinupTimeout = 0;
static uint32_t spinupStartTick = 0;
static uint32_t tumbleStartTick = 0;
static bool flyingSeen = false;
static uint32_t recentFlightTick = 0;

void supervisorInit(void) {
  if (supervisorInitialized) return;
  supervisorState = supervisorStateInit;
  memset(&supervisorSensors, 0, sizeof(supervisorSensors));
  memset(supervisorRatios, 0, sizeof(supervisorRatios));
  memset(supervisorRPMs, 0, sizeof(supervisorRPMs));
  supervisorInitialized = true;
}

bool supervisorCanFly(void) {
  return supervisorState == supervisorStateReadyToFly || supervisorState == supervisorStateFlying ||
         supervisorState == supervisorStateWarningLevelOut || supervisorState == supervisorStateLanded;
}

bool supervisorCanArm(void) {
  return supervisorState == supervisorStatePreFlChecksPassed;
}

bool supervisorIsArmed(void) { return supervisorArming; }
bool supervisorIsCrashed(void) { return supervisorCrash; }

bool supervisorRequestArming(void) {
  if (supervisorState == supervisorStateArming) return true;
  if (!supervisorCanArm()) return false;
  supervisorArming = true;
  supervisorState = supervisorStateArming;
  spinupStartTick = monotonicTick;
  return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
  if (supervisorTumbled) return false;
  if (doRecovery) supervisorCrash = false;
  else supervisorCrash = true;
  return true;
}

bool isFlyingCheck(const uint16_t motorRatios[4], uint16_t idleThrust, uint32_t currentTick) {
  if (!motorRatios) return false;
  for (int i = 0; i < 4; ++i) {
    if (motorRatios[i] > idleThrust) {
      recentFlightTick = currentTick;
      flyingSeen = true;
      return true;
    }
  }
  return flyingSeen && (currentTick - recentFlightTick < 2000u);
}

bool isTumbledCheck(const SensorData *sensors, float accZval) {
  if (!sensors) return false;
  float norm = sqrtf(sensors->acc.x * sensors->acc.x + sensors->acc.y * sensors->acc.y + sensors->acc.z * sensors->acc.z);
  supervisorLog.accNorm = norm;
  if (cfgCrashGs > 0.0f && fabsf(norm - 1.0f) > cfgCrashGs) supervisorCrash = true;
  if (fabsf(sensors->acc.x) < cfgFreeFall && fabsf(sensors->acc.y) < cfgFreeFall && fabsf(sensors->acc.z) < cfgFreeFall) {
    supervisorFreeFalling = true;
    supervisorState = supervisorStateExceptFreeFall;
    tumbleStartTick = 0;
    supervisorConditions |= SUPERVISOR_CB_FREEFALL;
    return false;
  }
  supervisorFreeFalling = false;
  if (!cfgTumbleEnabled) return false;
  uint32_t timeout = (accZval < cfgInvertedZ) ? cfgInvertedTimeout : cfgTumbleTimeout;
  if (accZval < cfgTiltZ) {
    if (tumbleStartTick == 0) tumbleStartTick = monotonicTick;
    if (monotonicTick - tumbleStartTick >= timeout) {
      supervisorTumbled = true;
      supervisorConditions |= SUPERVISOR_CB_TUMBLED;
    }
  } else {
    tumbleStartTick = 0;
    supervisorTumbled = false;
  }
  return supervisorTumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick) {
  if (lastNotificationTick == 0) return true;
  return (currentTick - lastNotificationTick) <= 1000u;
}

void supervisorSetSensorData(const SensorData *sensors) {
  if (sensors) supervisorSensors = *sensors;
}

void supervisorSetMotorRatios(const uint16_t ratios[4]) {
  if (ratios) memcpy(supervisorRatios, ratios, sizeof(supervisorRatios));
}

void supervisorSetMotorRPMs(const uint16_t rpms[4]) {
  if (rpms) memcpy(supervisorRPMs, rpms, sizeof(supervisorRPMs));
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold, float tiltAccZ, float invertedAccZ,
                               uint32_t tumbleTimeoutMs, uint32_t invertedTimeoutMs, bool tumbleEnabled,
                               bool autoArming, uint32_t spinupTimeoutMs) {
  cfgCrashGs = crashDetectionGs;
  cfgFreeFall = freeFallThreshold;
  cfgTiltZ = tiltAccZ;
  cfgInvertedZ = invertedAccZ;
  cfgTumbleTimeout = tumbleTimeoutMs;
  cfgInvertedTimeout = invertedTimeoutMs;
  cfgTumbleEnabled = tumbleEnabled;
  supervisorAutoArming = autoArming;
  cfgSpinupTimeout = spinupTimeoutMs;
}

bool supervisorIsPreflightTimeout(uint32_t currentTick, uint32_t startTick, uint32_t timeoutMs) {
  return startTick != 0 && currentTick - startTick >= timeoutMs;
}

bool supervisorIsLandingTimeout(uint32_t currentTick, uint32_t landingTick, uint32_t timeoutMs) {
  return landingTick != 0 && currentTick - landingTick >= timeoutMs;
}

bool isRPMatArmingValid(const uint16_t rpm[4], uint16_t minRpm, uint16_t maxRpm, uint32_t currentTick, uint32_t requiredDurationMs) {
  static uint32_t validStart = 0;
  if (!rpm) return false;
  bool ok = true;
  for (int i = 0; i < 4; ++i) ok = ok && rpm[i] >= minRpm && rpm[i] <= maxRpm;
  if (!ok) {
    validStart = 0;
    return false;
  }
  if (validStart == 0) validStart = currentTick;
  return currentTick - validStart >= requiredDurationMs;
}

uint32_t updateAndPopulateConditions(uint32_t currentTick) {
  (void)currentTick;
  if (supervisorCrash) supervisorConditions |= SUPERVISOR_CB_TIMEOUT;
  else supervisorConditions &= ~SUPERVISOR_CB_TIMEOUT;
  if (supervisorTumbled) supervisorConditions |= SUPERVISOR_CB_TUMBLED;
  if (supervisorFreeFalling) supervisorConditions |= SUPERVISOR_CB_FREEFALL;
  supervisorLog.conditions = supervisorConditions;
  return supervisorConditions;
}

void supervisorUpdate(uint32_t stabilizerStep, uint32_t currentTick) {
  monotonicTick = currentTick;
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;
  if (supervisorState == supervisorStatePreFlChecksPassed && supervisorAutoArming) supervisorRequestArming();
  isFlyingCheck(supervisorRatios, 0, currentTick);
  isTumbledCheck(&supervisorSensors, sensfusion6GetAccZ(supervisorSensors.acc.x, supervisorSensors.acc.y, supervisorSensors.acc.z));
  if (supervisorState == supervisorStateArming) {
    if (spinupStartTick == 0) spinupStartTick = currentTick;
    if (cfgSpinupTimeout && currentTick - spinupStartTick >= cfgSpinupTimeout) supervisorConditions |= SUPERVISOR_CB_SPINUP_TIMEOUT;
  } else {
    spinupStartTick = 0;
    supervisorConditions &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
  }
  if (!(supervisorState == supervisorStateArming || supervisorState == supervisorStateReadyToFly ||
        supervisorState == supervisorStateFlying || supervisorState == supervisorStateWarningLevelOut ||
        supervisorState == supervisorStateLanded)) {
    supervisorArming = false;
  }
  updateAndPopulateConditions(currentTick);
  supervisorLog.info = supervisorGetInfoBitfield();
  supervisorLog.canFly = supervisorCanFly();
  supervisorLog.armed = supervisorArming;
  supervisorLog.crashed = supervisorCrash;
}

void supervisorOverrideSetpoint(Setpoint *sp) {
  if (!sp) return;
  if (supervisorState == supervisorStateWarningLevelOut) {
    sp->mode.x = modeDisable; sp->mode.y = modeDisable;
    sp->mode.roll = modeAbs; sp->mode.pitch = modeAbs;
    sp->attitude.roll = 0.0f; sp->attitude.pitch = 0.0f;
    sp->mode.yaw = modeVelocity; sp->attitudeRate.yaw = 0.0f;
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
  if (supervisorArming) b |= 1u << 1;
  if (supervisorAutoArming) b |= 1u << 2;
  if (supervisorCanFly()) b |= 1u << 3;
  if (flyingSeen && monotonicTick - recentFlightTick < 2000u) b |= 1u << 4;
  if (supervisorTumbled) b |= 1u << 5;
  if (supervisorLocked) b |= 1u << 6;
  if (supervisorCrash) b |= 1u << 7;
  if (supervisorTrajectoryFlying) b |= 1u << 8;
  if (supervisorTrajectoryFinished) b |= 1u << 9;
  if (supervisorTrajectoryDisabled) b |= 1u << 10;
  if (supervisorDeckFault) b |= 1u << 11;
  return b;
}

static EstimatorMeasurement estQ[ESTIMATOR_QUEUE_SIZE];
static uint8_t estHead, estTail, estCount;
static EstimatorMeasurement estLast[8];

void estimatorInit(void) {
  if (estimatorInitialized) return;
  estHead = estTail = estCount = 0;
  memset(estLast, 0, sizeof(estLast));
  estimatorInitialized = true;
}

bool estimatorEnqueue(const EstimatorMeasurement *m) {
  if (!m || estCount >= ESTIMATOR_QUEUE_SIZE) return false;
  estQ[estTail] = *m;
  estTail = (uint8_t)((estTail + 1u) % ESTIMATOR_QUEUE_SIZE);
  estCount++;
  return true;
}

bool estimatorDequeue(EstimatorMeasurement *m) {
  if (!m || estCount == 0) return false;
  *m = estQ[estHead];
  estHead = (uint8_t)((estHead + 1u) % ESTIMATOR_QUEUE_SIZE);
  estCount--;
  return true;
}

void estimatorComplementary(State *state, const SensorData *sensors, uint32_t step, float dt) {
  if (!state || !sensors) return;
  EstimatorMeasurement m;
  while (estimatorDequeue(&m)) {
    if ((int)m.type >= 0 && (int)m.type < 8) estLast[m.type] = m;
  }
  if (RATE_DO_EXECUTE(RATE_250_HZ, step)) {
    sensfusion6UpdateQ(sensors->gyro.x, sensors->gyro.y, sensors->gyro.z, sensors->acc.x, sensors->acc.y, sensors->acc.z, dt);
    sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);
    state->attitudeQuaternion.q0 = qw; state->attitudeQuaternion.q1 = qx; state->attitudeQuaternion.q2 = qy; state->attitudeQuaternion.q3 = qz;
    state->velocity.z += sensfusion6GetAccZWithoutGravity(sensors->acc.x, sensors->acc.y, sensors->acc.z) * 9.81f * dt;
  }
  if (RATE_DO_EXECUTE(RATE_100_HZ, step)) {
    state->position.x += state->velocity.x * dt;
    state->position.y += state->velocity.y * dt;
    state->position.z += state->velocity.z * dt;
  }
}

static Setpoint commanderActive;
static commanderPriority_t commanderPriority = COMMANDER_PRIORITY_LOWEST;
static uint32_t commanderLastUpdate = 0;

void commanderInit(void) {
  if (commanderInitialized) return;
  memset(&commanderActive, 0, sizeof(commanderActive));
  commanderPriority = COMMANDER_PRIORITY_LOWEST;
  commanderLastUpdate = 0;
  commanderInitialized = true;
}

bool commanderSetSetpoint(const Setpoint *sp, commanderPriority_t priority, uint32_t currentTick) {
  if (!sp) return false;
  commanderInit();
  if (priority == COMMANDER_PRIORITY_DISABLE || priority >= commanderPriority) {
    commanderActive = *sp;
    commanderPriority = priority;
    commanderLastUpdate = currentTick;
    if (priority > COMMANDER_PRIORITY_HIGHLEVEL) supervisorTrajectoryDisabled = true;
    return true;
  }
  return false;
}

void commanderGetSetpoint(Setpoint *sp) {
  if (sp) *sp = commanderActive;
}

void commanderRelaxPriority(void) {
  commanderPriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(uint32_t currentTick) {
  return currentTick - commanderLastUpdate;
}

commanderPriority_t commanderGetActivePriority(void) {
  return commanderPriority;
}

static Setpoint highLevelSp;
static bool highLevelPending = false;
static uint32_t stabilizerStepCounter = 0;
static State stabilizerState;
static SensorData stabilizerSensors;

void stabilizerInit(void) {
  if (stabilizerInitialized) return;
  sensfusion6Init();
  estimatorInit();
  attitudeControllerInit();
  powerDistributionInit();
  supervisorInit();
  commanderInit();
  stabilizerInitialized = true;
}

void stabilizerSubmitHighLevelSetpoint(const Setpoint *sp) {
  if (!sp) return;
  highLevelSp = *sp;
  highLevelPending = true;
}

void stabilizerTaskStep(uint32_t currentTick) {
  stabilizerInit();
  stabilizerStepCounter++;
  if (highLevelPending) {
    commanderSetSetpoint(&highLevelSp, COMMANDER_PRIORITY_HIGHLEVEL, currentTick);
    highLevelPending = false;
  }
  estimatorComplementary(&stabilizerState, &stabilizerSensors, stabilizerStepCounter, 0.001f);
  Setpoint sp;
  commanderGetSetpoint(&sp);
  supervisorUpdate(stabilizerStepCounter, currentTick);
  supervisorOverrideSetpoint(&sp);
  control_t ctl;
  controllerPid(&stabilizerSensors, &stabilizerState, &sp, &ctl, 0.002f);
  motorPower_t mp = {0, 0, 0, 0};
  powerDistribution(&ctl, &mp);
  powerDistributionCap(&mp, 65535, 0);
  if (!supervisorCanFly() || !supervisorAreMotorsAllowedToRun()) mp.m1 = mp.m2 = mp.m3 = mp.m4 = 0;
  motor.m1 = (uint16_t)mp.m1; motor.m2 = (uint16_t)mp.m2; motor.m3 = (uint16_t)mp.m3; motor.m4 = (uint16_t)mp.m4;
}

uint32_t quatcompress(float q0, float q1v, float q2v, float q3v) {
  float q[4] = { q0, q1v, q2v, q3v };
  int largest = 0;
  for (int i = 1; i < 4; ++i) if (fabsf(q[i]) > fabsf(q[largest])) largest = i;
  if (q[largest] < 0.0f) for (int i = 0; i < 4; ++i) q[i] = -q[i];
  uint32_t out = (uint32_t)largest;
  int shift = 2;
  for (int i = 0; i < 4; ++i) {
    if (i == largest) continue;
    int32_t v = (int32_t)lroundf((q[i] + 0.70710678f) * 1023.0f / 1.41421356f);
    if (v < 0) v = 0; if (v > 1023) v = 1023;
    out |= ((uint32_t)v & 0x3ffu) << shift;
    shift += 10;
  }
  return out;
}

void compressState(const State *state, const SensorData *sensors, compressedState_t *out) {
  if (!state || !sensors || !out) return;
  out->x = (int32_t)lroundf(state->position.x * 1000.0f);
  out->y = (int32_t)lroundf(state->position.y * 1000.0f);
  out->z = (int32_t)lroundf(state->position.z * 1000.0f);
  out->vx = (int32_t)lroundf(state->velocity.x * 1000.0f);
  out->vy = (int32_t)lroundf(state->velocity.y * 1000.0f);
  out->vz = (int32_t)lroundf(state->velocity.z * 1000.0f);
  out->ax = (int32_t)lroundf(sensors->acc.x * 9810.0f);
  out->ay = (int32_t)lroundf(sensors->acc.y * 9810.0f);
  out->az = (int32_t)lroundf((sensors->acc.z + 1.0f) * 9810.0f);
  out->gx = (int32_t)lroundf(sensors->gyro.x * PI / 180.0f * 1000.0f);
  out->gy = (int32_t)lroundf(-sensors->gyro.y * PI / 180.0f * 1000.0f);
  out->gz = (int32_t)lroundf(sensors->gyro.z * PI / 180.0f * 1000.0f);
  out->quat = quatcompress(state->attitudeQuaternion.q0, state->attitudeQuaternion.q1, state->attitudeQuaternion.q2, state->attitudeQuaternion.q3);
}

bool rateSupervisorValidate(uint32_t rateHz) {
  return rateHz >= 997u && rateHz <= 1003u;
}

static bool reqProp = false, reqBat = false;
static uint8_t healthState = 0;
static uint32_t batTick = 0, batRestartTick = 0;
static float idleVoltage = 0.0f, minLoadedVoltage = 1000.0f;
static float batThreshold = 0.5f;

void healthInit(void) {
  if (healthInitialized) return;
  memset(&healthLog, 0, sizeof(healthLog));
  healthInitialized = true;
}

void startPropTest(void) { reqProp = true; }
void startBatTest(void) { reqBat = true; }

bool healthShallWeRunTest(void) {
  healthInit();
  if (reqProp) {
    reqProp = false;
    healthState = 1;
    healthLog.motorPass = 0;
    healthLog.motorTestCount = 0;
    return true;
  }
  if (reqBat) {
    reqBat = false;
    healthState = 10;
    batTick = 0;
    minLoadedVoltage = 1000.0f;
    return true;
  }
  return healthState != 0;
}

bool evaluatePropTest(float value, float lowThreshold, float highThreshold) {
  if (highThreshold == 0.0f) return true;
  return value >= lowThreshold && value <= highThreshold;
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

void healthRunTests(uint32_t currentTick, const SensorData *sensors, float batteryVoltage) {
  (void)sensors;
  healthInit();
  if (healthState == 1) {
    idleVoltage = batteryVoltage;
    healthState = 2;
  } else if (healthState >= 2 && healthState < 6) {
    uint8_t motorIndex = (uint8_t)(healthState - 2);
    healthLog.motorPass |= (uint8_t)(1u << motorIndex);
    healthLog.motorTestCount++;
    healthState++;
  } else if (healthState == 6) {
    healthState = 0;
  } else if (healthState == 10) {
    if (batTick == 0) idleVoltage = batteryVoltage;
    batTick++;
    if (batTick >= 2 && batTick <= 49 && batteryVoltage < minLoadedVoltage) minLoadedVoltage = batteryVoltage;
    if (batTick >= 50) {
      healthLog.batterySag = idleVoltage - minLoadedVoltage;
      healthLog.batteryPass = healthLog.batterySag <= batThreshold;
      healthState = 0;
      batRestartTick = currentTick;
    }
  }
}

void restartBatTest(uint32_t currentTick) {
  if (currentTick - batRestartTick >= 2000u) startBatTest();
}

typedef struct {
  CRTPPacket data[CRTP_TX_QUEUE_SIZE];
  uint16_t head, tail, count;
} TxQueue;

typedef struct {
  CRTPPacket data[CRTP_RX_QUEUE_SIZE];
  uint8_t head, tail, count;
  bool created;
} RxQueue;

static TxQueue txQueue;
static RxQueue rxQueues[CRTP_NBR_OF_PORTS];
static CRTPPortCB portCBs[CRTP_NBR_OF_PORTS];
static CRTPLink *activeLink = NULL;
static uint32_t lastTxTry = 0, lastStats = 0;
static uint16_t rxCounter = 0, txCounter = 0, rxRate = 0, txRate = 0;
static bool crtpError = false;

static bool nopSend(const CRTPPacket *pk) { (void)pk; return false; }
static bool nopReceive(CRTPPacket *pk) { (void)pk; return false; }
static void nopReset(void) {}
static bool nopConnected(void) { return true; }
static void nopEnable(bool enable) { (void)enable; }
static CRTPLink nopLink = { nopSend, nopReceive, nopReset, nopConnected, nopEnable };

void crtpInit(void) {
  if (crtpInitialized) return;
  memset(&txQueue, 0, sizeof(txQueue));
  memset(rxQueues, 0, sizeof(rxQueues));
  memset(portCBs, 0, sizeof(portCBs));
  activeLink = &nopLink;
  crtpInitialized = true;
}

bool crtpCreateRxQueue(uint8_t port) {
  crtpInit();
  if (port >= CRTP_NBR_OF_PORTS) return false;
  if (rxQueues[port].created) {
    crtpError = true;
    return false;
  }
  rxQueues[port].created = true;
  return true;
}

bool crtpSendPacket(const CRTPPacket *pk) {
  crtpInit();
  if (!pk || txQueue.count >= CRTP_TX_QUEUE_SIZE) return false;
  txQueue.data[txQueue.tail] = *pk;
  txQueue.tail = (uint16_t)((txQueue.tail + 1u) % CRTP_TX_QUEUE_SIZE);
  txQueue.count++;
  return true;
}

bool crtpSendPacketBlock(const CRTPPacket *pk) {
  return crtpSendPacket(pk);
}

static bool rxPush(uint8_t port, const CRTPPacket *pk) {
  if (port >= CRTP_NBR_OF_PORTS || !rxQueues[port].created || rxQueues[port].count >= CRTP_RX_QUEUE_SIZE) return false;
  RxQueue *q = &rxQueues[port];
  q->data[q->tail] = *pk;
  q->tail = (uint8_t)((q->tail + 1u) % CRTP_RX_QUEUE_SIZE);
  q->count++;
  return true;
}

bool crtpReceivePacket(uint8_t port, CRTPPacket *pk) {
  crtpInit();
  if (!pk || port >= CRTP_NBR_OF_PORTS || rxQueues[port].count == 0) return false;
  RxQueue *q = &rxQueues[port];
  *pk = q->data[q->head];
  q->head = (uint8_t)((q->head + 1u) % CRTP_RX_QUEUE_SIZE);
  q->count--;
  return true;
}

bool crtpReceivePacketBlock(uint8_t port, CRTPPacket *pk) {
  return crtpReceivePacket(port, pk);
}

bool crtpReceivePacketWait(uint8_t port, CRTPPacket *pk, uint32_t timeoutMs) {
  (void)timeoutMs;
  return crtpReceivePacket(port, pk);
}

void crtpRxTaskStep(void) {
  crtpInit();
  CRTPPacket pk;
  if (!activeLink || activeLink == &nopLink || !activeLink->receivePacket) return;
  if (activeLink->receivePacket(&pk)) {
    rxCounter++;
    if (pk.port < CRTP_NBR_OF_PORTS) {
      rxPush(pk.port, &pk);
      if (portCBs[pk.port]) portCBs[pk.port](&pk);
    }
  }
}

void crtpTxTaskStep(uint32_t currentTick) {
  crtpInit();
  if (!activeLink || activeLink == &nopLink || !activeLink->sendPacket || txQueue.count == 0) return;
  if (lastTxTry != 0 && currentTick - lastTxTry < 10u) return;
  lastTxTry = currentTick;
  CRTPPacket *pk = &txQueue.data[txQueue.head];
  if (activeLink->sendPacket(pk)) {
    txQueue.head = (uint16_t)((txQueue.head + 1u) % CRTP_TX_QUEUE_SIZE);
    txQueue.count--;
    txCounter++;
  }
}

void crtpSetLink(CRTPLink *link) {
  crtpInit();
  if (activeLink && activeLink->enable) activeLink->enable(false);
  activeLink = link ? link : &nopLink;
  if (activeLink->enable) activeLink->enable(true);
}

void crtpReset(void) {
  crtpInit();
  memset(&txQueue, 0, sizeof(txQueue));
  if (activeLink && activeLink->reset) activeLink->reset();
}

bool crtpIsConnected(void) {
  crtpInit();
  return activeLink && activeLink->isConnected ? activeLink->isConnected() : true;
}

uint16_t crtpGetFreeTxQueuePackets(void) {
  crtpInit();
  return (uint16_t)(CRTP_TX_QUEUE_SIZE - txQueue.count);
}

bool crtpRegisterPortCB(uint8_t port, CRTPPortCB cb) {
  crtpInit();
  if (port >= CRTP_NBR_OF_PORTS) return false;
  portCBs[port] = cb;
  return true;
}

void crtpUpdateStats(uint32_t currentTick) {
  if (lastStats == 0) lastStats = currentTick;
  if (currentTick - lastStats >= 500u) {
    rxRate = (uint16_t)(rxCounter * 2u);
    txRate = (uint16_t)(txCounter * 2u);
    rxCounter = txCounter = 0;
    lastStats = currentTick;
  }
  (void)crtpError;
}

uint16_t crtpGetRxRate(void) { return rxRate; }
uint16_t crtpGetTxRate(void) { return txRate; }

size_t deckDiscovery(const uint8_t *i2cAddresses, size_t i2cCount, const uint64_t *owRoms, size_t owCount,
                     uint64_t *outIds, size_t capacity) {
  if (!outIds || capacity == 0) return 0;
  size_t written = 0;
  for (size_t i = 0; i < i2cCount; ++i) {
    uint64_t id = 0x1000000000000000ull | (uint64_t)i2cAddresses[i];
    bool dup = false;
    for (size_t j = 0; j < written; ++j) if (outIds[j] == id) dup = true;
    if (!dup && written < capacity) outIds[written++] = id;
  }
  for (size_t i = 0; i < owCount; ++i) {
    uint64_t id = owRoms[i];
    bool dup = false;
    for (size_t j = 0; j < written; ++j) if (outIds[j] == id) dup = true;
    if (!dup && written < capacity) outIds[written++] = id;
  }
  return written;
}

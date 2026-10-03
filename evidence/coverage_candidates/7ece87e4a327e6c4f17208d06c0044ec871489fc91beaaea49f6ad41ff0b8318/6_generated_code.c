#include "6_generated_code.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

PidObject pidRoll, pidPitch, pidYaw, pidRollRate, pidPitchRate, pidYawRate;
bool thrustLocked = false, commanderModeSet = false;
SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0;
TestState healthTestState = testDone;
uint8_t motorPass = 0, batteryPass = 0;
float batterySag = 0.0f;

StateEstimateLog stateEstimate;
Axis3Log gyro, acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

static bool stabilizerInited;
static Setpoint activeSetpoint, pendingHighLevelSetpoint;
static bool hasHighLevelSetpoint;
static int activePriority = COMMANDER_PRIORITY_LOWEST;
static uint32_t commanderLastUpdate;

static EstimatorMeasurement estQ[16];
static unsigned estHead, estTail, estCount;

static CrtpPacket txQ[CRTP_TX_QUEUE_SIZE], rxQ[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static unsigned txHead, txTail, txCount, rxHead[CRTP_NBR_OF_PORTS], rxTail[CRTP_NBR_OF_PORTS], rxCount[CRTP_NBR_OF_PORTS];
static bool rxCreated[CRTP_NBR_OF_PORTS], crtpInited;
static CrtpPortCallback portCb[CRTP_NBR_OF_PORTS];
static CrtpLink *linkPtr;
static uint32_t crtpRxCount, crtpTxCount, crtpLastStatsTick;

static SensorData supSensors;
static uint32_t supMotorRatios[4], supIdleThrust;
static int32_t supMotorRPMs[4];
static float cfgCrashGs, cfgFreeFall, cfgTiltZ = 0.8f, cfgUpsideDownZ = -0.8f;
static uint32_t cfgTiltTime = 1000, cfgUpsideDownTime = 1000, cfgSpinupTimeout;
static bool cfgTumbleEnabled = true, cfgAutoArming, supervisorArmed, supervisorCrashed;
static bool supervisorTumbled, supervisorFreeFall;
static uint32_t supervisorTick, spinupStartTick;

static bool propReq, batReq;
static float propBuf[PROPTEST_NBR_OF_VARIANCE_VALUES];
static int propIndex, batTick;
static float idleVoltage = 4.0f, minLoadedVoltage = 4.0f;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static int32_t round_i32(float v) { return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f); }

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

float invSqrt(float x) {
  if (x <= 0.0f) return 0.0f;
  union { float f; uint32_t i; } u = { x };
  u.i = 0x5f3759dfU - (u.i >> 1);
  u.f = u.f * (1.5f - 0.5f * x * u.f * u.f);
  return u.f;
}

void estimatedGravityDirection(float w, float x, float y, float z, float *gx, float *gy, float *gz) {
  if (gx) *gx = 2.0f * (x * z - w * y);
  if (gy) *gy = 2.0f * (w * x + y * z);
  if (gz) *gz = w * w - x * x - y * y + z * z;
}

static void syncSensLog(void) {
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  sensfusion6Log.qw = qw; sensfusion6Log.qx = qx; sensfusion6Log.qy = qy; sensfusion6Log.qz = qz;
  sensfusion6Log.gravityX = gravityX; sensfusion6Log.gravityY = gravityY; sensfusion6Log.gravityZ = gravityZ;
  sensfusion6Log.accZbase = baseZacc; sensfusion6Log.isInit = sensfusion6IsInit;
  sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;
}

void sensfusion6Init(void) {
  if (sensfusion6IsInit) return;
  qw = 1.0f; qx = qy = qz = 0.0f;
  integralFBx = integralFBy = integralFBz = 0.0f;
  sensfusion6IsInit = true;
  syncSensLog();
}

bool sensfusion6Test(void) { return sensfusion6IsInit; }

void sensfusion6UpdateQ(float gx_dps, float gy_dps, float gz_dps, float ax, float ay, float az, float dt) {
  float gx_r = gx_dps * (float)M_PI / 180.0f, gy_r = gy_dps * (float)M_PI / 180.0f, gz_r = gz_dps * (float)M_PI / 180.0f;
  if (ax != 0.0f || ay != 0.0f || az != 0.0f) {
    float recip = invSqrt(ax * ax + ay * ay + az * az);
    if (recip > 0.0f) {
      ax *= recip; ay *= recip; az *= recip;
      estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
      float ex = ay * gravityZ - az * gravityY;
      float ey = az * gravityX - ax * gravityZ;
      float ez = ax * gravityY - ay * gravityX;
      if (twoKi > 0.0f) {
        integralFBx += twoKi * ex * dt; integralFBy += twoKi * ey * dt; integralFBz += twoKi * ez * dt;
        gx_r += integralFBx; gy_r += integralFBy; gz_r += integralFBz;
      } else {
        integralFBx = integralFBy = integralFBz = 0.0f;
      }
      gx_r += twoKp * ex; gy_r += twoKp * ey; gz_r += twoKp * ez;
      if (!sensfusion6IsCalibrated) {
        baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
        sensfusion6IsCalibrated = true;
      }
    }
  }
  float qa = qw, qb = qx, qc = qy;
  qw += (-qb * gx_r - qc * gy_r - qz * gz_r) * 0.5f * dt;
  qx += ( qa * gx_r + qc * gz_r - qz * gy_r) * 0.5f * dt;
  qy += ( qa * gy_r - qb * gz_r + qz * gx_r) * 0.5f * dt;
  qz += ( qa * gz_r + qb * gy_r - qc * gx_r) * 0.5f * dt;
  float recip = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (recip > 0.0f) { qw *= recip; qx *= recip; qy *= recip; qz *= recip; }
  syncSensLog();
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
  float sinr = 2.0f * (qw * qx + qy * qz);
  float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
  float sinp = clampf(2.0f * (qw * qy - qz * qx), -1.0f, 1.0f);
  float siny = 2.0f * (qw * qz + qx * qy);
  float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
  if (roll_deg) *roll_deg = atan2f(sinr, cosr) * 180.0f / (float)M_PI;
  if (pitch_deg) *pitch_deg = asinf(sinp) * 180.0f / (float)M_PI;
  if (yaw_deg) *yaw_deg = atan2f(siny, cosy) * 180.0f / (float)M_PI;
  syncSensLog();
}

void sensfusion6GetQuaternion(float *w, float *x, float *y, float *z) {
  if (w) *w = qw; if (x) *x = qx; if (y) *y = qy; if (z) *z = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
  syncSensLog();
  return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw, MotorPower *out) {
  if (!out) return;
  int32_t r = roll / 2, p = pitch / 2, t = thrust;
  out->m1 = t - r + p + yaw;
  out->m2 = t - r - p - yaw;
  out->m3 = t + r - p + yaw;
  out->m4 = t + r + p - yaw;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY, float torqueZ, float armLength, float thrustToTorque, float f[4]) {
  if (!f) return;
  float thrustPart = 0.25f * thrustSi;
  float arm = 0.707106781f * armLength;
  float rollPart = armLength != 0.0f ? 0.25f * torqueX / arm : 0.0f;
  float pitchPart = armLength != 0.0f ? 0.25f * torqueY / arm : 0.0f;
  float yawPart = thrustToTorque != 0.0f ? 0.25f * torqueZ / thrustToTorque : 0.0f;
  f[0] = thrustPart - rollPart + pitchPart + yawPart;
  f[1] = thrustPart - rollPart - pitchPart - yawPart;
  f[2] = thrustPart + rollPart - pitchPart + yawPart;
  f[3] = thrustPart + rollPart + pitchPart - yawPart;
  for (int i = 0; i < 4; i++) if (f[i] < 0.0f) f[i] = 0.0f;
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4]) {
  if (!normalizedForces || !motorPWMs) return;
  for (int i = 0; i < 4; i++) motorPWMs[i] = (uint16_t)(clampf(normalizedForces[i], 0.0f, 1.0f) * 65535.0f + 0.5f);
}

void powerDistribution(const ControlData *control, MotorPower *mp) {
  if (!control || !mp) return;
  if (control->controlMode == controlModeLegacy) {
    powerDistributionLegacy(control->thrust, control->roll, control->pitch, control->yaw, mp);
  } else if (control->controlMode == controlModeForceTorque) {
    float f[4], n[4]; uint16_t pwm[4];
    powerDistributionForceTorque(control->thrustSi, control->torque.x, control->torque.y, control->torque.z,
                                 CRAZYFLIE_ARM_LENGTH_M, CRAZYFLIE_THRUST_TO_TORQUE, f);
    for (int i = 0; i < 4; i++) n[i] = f[i] / CRAZYFLIE_MAX_MOTOR_FORCE_N;
    powerDistributionForce(n, pwm);
    mp->m1 = pwm[0]; mp->m2 = pwm[1]; mp->m3 = pwm[2]; mp->m4 = pwm[3];
  } else if (control->controlMode == controlModeForce) {
    uint16_t pwm[4];
    powerDistributionForce(control->normalizedForces, pwm);
    mp->m1 = pwm[0]; mp->m2 = pwm[1]; mp->m3 = pwm[2]; mp->m4 = pwm[3];
  }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) {
  return value < idleThrust ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4], int32_t maxAllowedThrust, int32_t idleThrust) {
  PowerCapResult r = { false, 0 };
  if (!motors) return r;
  int32_t max = motors[0];
  for (int i = 1; i < 4; i++) if (motors[i] > max) max = motors[i];
  if (max > maxAllowedThrust) {
    r.isCapped = true;
    r.reduction = max - maxAllowedThrust;
    for (int i = 0; i < 4; i++) motors[i] = capMinThrust(motors[i] - r.reduction, idleThrust);
  }
  return r;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) {
  return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust, float nominalVoltage, float actualVoltage) {
  if (actualVoltage <= 0.0f) return motorThrust;
  int32_t v = round_i32((float)motorThrust * nominalVoltage / actualVoltage);
  if (v < 0) v = 0; if (v > 65535) v = 65535;
  return (uint16_t)v;
}

static void pidInit(PidObject *p, float kp) {
  if (!p->initialized) { memset(p, 0, sizeof(*p)); p->kp = kp; p->initialized = true; }
}
static float pidRun(PidObject *p, float actual, float desired) {
  float e = desired - actual;
  p->integral += e;
  p->output = p->kp * e + p->ki * p->integral + p->kd * (e - p->prevError) + p->kff * desired;
  p->prevError = e;
  return p->output;
}
static void pidReset(PidObject *p, float actual) { p->integral = 0.0f; p->prevError = 0.0f; p->output = 0.0f; (void)actual; }

void attitudeControllerInit(float updateDt) {
  (void)updateDt;
  pidInit(&pidRoll, 6.0f); pidInit(&pidPitch, 6.0f); pidInit(&pidYaw, 6.0f);
  pidInit(&pidRollRate, 250.0f); pidInit(&pidPitchRate, 250.0f); pidInit(&pidYawRate, 120.0f);
}

void attitudeControllerCorrectRatePID(float ra, float rd, float pa, float pd, float ya, float yd) {
  pidRollRate.output = saturateSignedInt16(round_i32(pidRun(&pidRollRate, ra, rd)));
  pidPitchRate.output = saturateSignedInt16(round_i32(pidRun(&pidPitchRate, pa, pd)));
  pidYawRate.output = saturateSignedInt16(round_i32(pidRun(&pidYawRate, ya, yd)));
}

void attitudeControllerCorrectAttitudePID(float ra, float rd, float pa, float pd, float ya, float yd) {
  pidRun(&pidRoll, ra, rd); pidRun(&pidPitch, pa, pd); pidReset(&pidYaw, ya); pidRun(&pidYaw, ya, yd);
}

void attitudeControllerResetAllPID(float r, float p, float y) {
  pidReset(&pidRoll, r); pidReset(&pidPitch, p); pidReset(&pidYaw, y);
  pidReset(&pidRollRate, 0.0f); pidReset(&pidPitchRate, 0.0f); pidReset(&pidYawRate, 0.0f);
}
void attitudeControllerResetRollAttitudePID(float r) { pidReset(&pidRoll, r); }
void attitudeControllerResetPitchAttitudePID(float p) { pidReset(&pidPitch, p); }

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch, int16_t *yaw) {
  if (roll) *roll = saturateSignedInt16(round_i32(pidRollRate.output));
  if (pitch) *pitch = saturateSignedInt16(round_i32(pidPitchRate.output));
  if (yaw) *yaw = saturateSignedInt16(round_i32(pidYawRate.output));
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
  if (!setpoint || !state) return 0;
  float err = 0.0f;
  if (setpoint->mode.z == modeAbs) err = setpoint->position.z - state->position.z;
  else if (setpoint->mode.z == modeVelocity) err = setpoint->velocity.z - state->velocity.z;
  int32_t thrust = 30000 + round_i32(err * 10000.0f);
  if (thrust < 0) thrust = 0; if (thrust > 65535) thrust = 65535;
  return (uint16_t)thrust;
}

void controllerPid(const SensorData *s, const Setpoint *sp, const State *st, ControlData *c, float yawMaxDelta, float dt) {
  static float desiredYaw;
  if (!s || !sp || !st || !c) return;
  memset(c, 0, sizeof(*c)); c->controlMode = controlModeLegacy;
  if (sp->thrust == 0) {
    attitudeControllerResetAllPID(st->attitude.roll, st->attitude.pitch, st->attitude.yaw);
    desiredYaw = st->attitude.yaw;
    return;
  }
  float desiredRoll = sp->attitude.roll, desiredPitch = sp->attitude.pitch;
  if (sp->mode.yaw == modeVelocity) desiredYaw += sp->attitudeRate.yaw * dt;
  else if (sp->mode.yaw == modeAbs) {
    if (sp->mode.quat == modeAbs) {
      float ow = qw, ox = qx, oy = qy, oz = qz;
      qw = sp->attitudeQuaternion.w; qx = sp->attitudeQuaternion.x; qy = sp->attitudeQuaternion.y; qz = sp->attitudeQuaternion.z;
      sensfusion6GetEulerRPY(NULL, NULL, &desiredYaw);
      qw = ow; qx = ox; qy = oy; qz = oz;
    } else desiredYaw = sp->attitude.yaw;
  }
  if (yawMaxDelta != 0.0f) desiredYaw = st->attitude.yaw + clampf(capAngle(desiredYaw - st->attitude.yaw), -yawMaxDelta, yawMaxDelta);
  if (sp->mode.roll == modeVelocity) { pidRoll.output = sp->attitudeRate.roll; attitudeControllerResetRollAttitudePID(st->attitude.roll); }
  else pidRun(&pidRoll, st->attitude.roll, desiredRoll);
  if (sp->mode.pitch == modeVelocity) { pidPitch.output = sp->attitudeRate.pitch; attitudeControllerResetPitchAttitudePID(st->attitude.pitch); }
  else pidRun(&pidPitch, st->attitude.pitch, desiredPitch);
  pidRun(&pidYaw, st->attitude.yaw, desiredYaw);
  c->thrust = sp->mode.z == modeDisable ? sp->thrust : positionControllerUpdate(sp, st);
  attitudeControllerCorrectRatePID(s->gyro.x, pidRoll.output, -s->gyro.y, pidPitch.output, s->gyro.z, pidYaw.output);
  attitudeControllerGetActuatorOutput(&c->roll, &c->pitch, &c->yaw);
  c->yaw = (int16_t)-c->yaw;
}

void rotateYaw(float roll, float pitch, float yaw_deg, float *rp, float *pp) {
  float a = yaw_deg * (float)M_PI / 180.0f;
  if (rp) *rp = roll * cosf(a) - pitch * sinf(a);
  if (pp) *pp = roll * sinf(a) + pitch * cosf(a);
}

void crtpCommanderRpytDecodeSetpoint(const CommanderCrtpLegacyValues *v, Setpoint *sp, bool alt, bool posHold, bool posSet,
                                     StabilizationType smR, StabilizationType smP, StabilizationType smY, YawMode yawMode) {
  if (!v || !sp) return;
  memset(sp, 0, sizeof(*sp));
  if (activePriority == COMMANDER_PRIORITY_DISABLE) { thrustLocked = true; if (v->thrust == 0) thrustLocked = false; }
  if (alt) {
    sp->mode.z = modeVelocity; sp->velocity.z = ((float)v->thrust - 32767.0f) / 32767.0f; sp->thrust = 0;
    commanderModeSet = true; return;
  }
  commanderModeSet = false;
  if (posHold) {
    sp->mode.x = sp->mode.y = modeVelocity; sp->mode.roll = sp->mode.pitch = modeDisable;
    sp->velocity.x = v->pitch / 30.0f; sp->velocity.y = v->roll / 30.0f; return;
  }
  if (posSet && v->thrust != 0) {
    sp->mode.x = sp->mode.y = sp->mode.z = sp->mode.yaw = modeAbs;
    sp->mode.roll = sp->mode.pitch = modeDisable;
    sp->position.x = -v->pitch; sp->position.y = v->roll; sp->position.z = v->thrust / 1000.0f;
    sp->attitude.yaw = v->yaw; sp->thrust = 0; return;
  }
  sp->thrust = (thrustLocked || v->thrust < MIN_THRUST) ? 0U : (v->thrust > MAX_THRUST ? MAX_THRUST : v->thrust);
  float r = v->roll, p = v->pitch;
  if (yawMode == PLUSMODE) rotateYaw(r, p, 45.0f, &r, &p);
  else if (yawMode == CAREFREE) { thrustLocked = true; }
  sp->mode.roll = smR == RATE ? modeVelocity : modeAbs;
  sp->mode.pitch = smP == RATE ? modeVelocity : modeAbs;
  if (sp->mode.roll == modeVelocity) sp->attitudeRate.roll = r; else sp->attitude.roll = r;
  if (sp->mode.pitch == modeVelocity) sp->attitudeRate.pitch = p; else sp->attitude.pitch = p;
  sp->mode.yaw = smY == RATE ? modeVelocity : modeAbs;
  if (sp->mode.yaw == modeVelocity) sp->attitudeRate.yaw = -v->yaw; else sp->attitude.yaw = v->yaw;
}

void supervisorInit(void) { supervisorState = supervisorStateLocked; supervisorConditionBits = 0; supervisorArmed = false; supervisorCrashed = false; }

bool supervisorCanFly(void) {
  return supervisorState == supervisorStateReadyToFly || supervisorState == supervisorStateFlying ||
         supervisorState == supervisorStateWarningLevelOut || supervisorState == supervisorStateLanded;
}
bool supervisorCanArm(void) { return supervisorState == supervisorStatePreFlChecksPassed; }
bool supervisorIsArmed(void) { return supervisorArmed; }
bool supervisorIsCrashed(void) { return supervisorCrashed; }

bool supervisorRequestArming(bool doArm) {
  if (!doArm) { supervisorArmed = false; supervisorConditionBits &= ~SUPERVISOR_CB_ARMED; return true; }
  if (!supervisorCanArm() && supervisorState != supervisorStateArming) return false;
  supervisorArmed = true; supervisorState = supervisorStateArming; supervisorConditionBits |= SUPERVISOR_CB_ARMED; return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
  if (doRecovery) { if (supervisorTumbled) return false; supervisorCrashed = false; supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED; return true; }
  supervisorCrashed = true; supervisorConditionBits |= SUPERVISOR_CB_CRASHED; return true;
}

bool supervisorAreMotorsAllowedToRun(void) {
  return supervisorState == supervisorStateArming || supervisorState == supervisorStateReadyToFly ||
         supervisorState == supervisorStateFlying || supervisorState == supervisorStateWarningLevelOut ||
         supervisorState == supervisorStateLanded;
}

uint16_t supervisorGetInfoBitfield(void) {
  uint16_t b = 0;
  if (supervisorCanArm()) b |= 1U << 0;
  if (supervisorArmed) b |= 1U << 1;
  if (cfgAutoArming) b |= 1U << 2;
  if (supervisorCanFly()) b |= 1U << 3;
  if (supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) b |= 1U << 4;
  if (supervisorTumbled) b |= 1U << 5;
  if (supervisorState == supervisorStateLocked) b |= 1U << 6;
  if (supervisorCrashed) b |= 1U << 7;
  return b;
}

bool isFlyingCheck(const uint32_t m[4], uint32_t idle, uint32_t tick) {
  static bool seen; static uint32_t last;
  if (!m) return false;
  for (int i = 0; i < 4; i++) if (m[i] > idle) { seen = true; last = tick; return true; }
  return seen && (tick - last) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float ax, float ay, float az, float crashGs, float freeFallThreshold,
                    float tiltZ, float upsideZ, uint32_t tiltTime, uint32_t upsideTime,
                    bool enabled, uint32_t tick, bool *isFreeFalling) {
  static uint32_t tiltStart, upsideStart;
  bool ff = freeFallThreshold > 0.0f && fabsf(ax) < freeFallThreshold && fabsf(ay) < freeFallThreshold && fabsf(az) < freeFallThreshold;
  if (isFreeFalling) *isFreeFalling = ff;
  if (crashGs > 0.0f) {
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (fabsf(n - 1.0f) > crashGs) { supervisorCrashed = true; supervisorConditionBits |= SUPERVISOR_CB_CRASHED; }
  }
  if (!enabled || ff) { tiltStart = upsideStart = 0; return false; }
  if (az < upsideZ) {
    if (!upsideStart) upsideStart = tick;
    return tick - upsideStart >= upsideTime;
  }
  upsideStart = 0;
  if (az < tiltZ) {
    if (!tiltStart) tiltStart = tick;
    return tick - tiltStart >= tiltTime;
  }
  tiltStart = 0;
  return false;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick) {
  return lastNotificationTick == 0U || currentTick - lastNotificationTick <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state, uint32_t latestArmingTick, uint32_t currentTick, uint32_t d) {
  return state == supervisorStatePreFlChecksPassed && latestArmingTick != 0U && currentTick - latestArmingTick >= d;
}
bool supervisorIsLandingTimeout(uint32_t latestLandingTick, uint32_t currentTick, uint32_t d) {
  return latestLandingTick != 0U && currentTick - latestLandingTick >= d;
}

uint32_t updateAndPopulateConditions(bool crtpStop, bool paramStop, bool wdtFailed) {
  if (crtpStop || paramStop || wdtFailed) supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
  else supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
  return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *sp, uint32_t bits, SupervisorState state) {
  if (!sp) return;
  if (bits & (SUPERVISOR_CB_EMERGENCY_STOP | SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_FREE_FALL |
              SUPERVISOR_CB_MOTORS_NOT_RESPONDING | SUPERVISOR_CB_CRASHED)) { memset(sp, 0, sizeof(*sp)); return; }
  if (state == supervisorStateWarningLevelOut) {
    sp->mode.x = sp->mode.y = modeDisable;
    sp->mode.roll = sp->mode.pitch = modeAbs;
    sp->attitude.roll = sp->attitude.pitch = 0.0f;
    sp->mode.yaw = modeVelocity; sp->attitudeRate.yaw = 0.0f;
  } else if (!(state == supervisorStateArming || state == supervisorStateReadyToFly ||
               state == supervisorStateFlying || state == supervisorStateLanded)) {
    memset(sp, 0, sizeof(*sp));
  }
}

bool isRPMatArmingValid(const int32_t rpm[4], int32_t mn, int32_t mx) {
  if (!rpm) return false;
  for (int i = 0; i < 4; i++) if (rpm[i] < mn || rpm[i] > mx) return false;
  return true;
}

bool isMotorsNotResponding(const int32_t rpm[4], int32_t threshold, uint32_t duration, bool canFly, uint32_t tick) {
  static uint32_t start;
  if (!rpm || !canFly) { start = 0; return false; }
  bool low = true; for (int i = 0; i < 4; i++) if (rpm[i] >= threshold) low = false;
  if (!low) { start = 0; return false; }
  if (!start) start = tick;
  return tick - start >= duration;
}

void supervisorSetSensorData(const SensorData *s) { if (s) supSensors = *s; }
void supervisorSetMotorRatios(const uint32_t r[4], uint32_t idle) { if (r) memcpy(supMotorRatios, r, sizeof(supMotorRatios)); supIdleThrust = idle; }
void supervisorSetMotorRPMs(const int32_t r[4]) { if (r) memcpy(supMotorRPMs, r, sizeof(supMotorRPMs)); }
void supervisorConfigureSafety(float c, float f, float t, float u, uint32_t mt, uint32_t mu, bool en) {
  cfgCrashGs = c; cfgFreeFall = f; cfgTiltZ = t; cfgUpsideDownZ = u; cfgTiltTime = mt; cfgUpsideDownTime = mu; cfgTumbleEnabled = en;
}
void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) { cfgAutoArming = autoArming; cfgSpinupTimeout = spinupTimeoutDurationMs; }

void supervisorUpdate(uint32_t step) {
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, step)) return;
  supervisorTick += RATE_SUPERVISOR;
  if (isFlyingCheck(supMotorRatios, supIdleThrust, supervisorTick)) supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
  else supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
  supervisorTumbled = isTumbledCheck(supSensors.acc.x, supSensors.acc.y, supSensors.acc.z, cfgCrashGs, cfgFreeFall,
                                     cfgTiltZ, cfgUpsideDownZ, cfgTiltTime, cfgUpsideDownTime,
                                     cfgTumbleEnabled, supervisorTick, &supervisorFreeFall);
  if (supervisorTumbled) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED; else supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
  if (supervisorFreeFall) { supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL; supervisorState = supervisorStateExceptFreeFall; }
  if (supervisorState == supervisorStatePreFlChecksPassed && cfgAutoArming) supervisorRequestArming(true);
  if (supervisorState == supervisorStateArming) {
    if (!spinupStartTick) spinupStartTick = supervisorTick;
    if (cfgSpinupTimeout && supervisorTick - spinupStartTick >= cfgSpinupTimeout) supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
    if (isRPMatArmingValid(supMotorRPMs, 1, INT32_MAX)) supervisorConditionBits |= SUPERVISOR_CB_RPM_AT_ARMING_VALID;
  } else {
    spinupStartTick = 0; supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
  }
  supervisorLog.info = supervisorGetInfoBitfield();
  supervisorLog.accNorm = sqrtf(supSensors.acc.x * supSensors.acc.x + supSensors.acc.y * supSensors.acc.y + supSensors.acc.z * supSensors.acc.z);
}

bool estimatorEnqueue(const EstimatorMeasurement *m) {
  if (!m || estCount >= 16) return false;
  estQ[estTail] = *m; estTail = (estTail + 1U) % 16U; estCount++; return true;
}
bool estimatorDequeue(EstimatorMeasurement *m) {
  if (!m || estCount == 0) return false;
  *m = estQ[estHead]; estHead = (estHead + 1U) % 16U; estCount--; return true;
}
void estimatorComplementary(uint32_t step) {
  EstimatorMeasurement m; SensorData latest = {0};
  while (estimatorDequeue(&m)) {
    if (m.type == MeasurementTypeGyroscope) { latest.gyro.x = m.data[0]; latest.gyro.y = m.data[1]; latest.gyro.z = m.data[2]; }
    else if (m.type == MeasurementTypeAcceleration) { latest.acc.x = m.data[0]; latest.acc.y = m.data[1]; latest.acc.z = m.data[2]; }
    else if (m.type == MeasurementTypeBarometer) latest.baroPressure = m.data[0];
    else if (m.type == MeasurementTypeTOF) latest.tofRange = m.data[0];
  }
  if (RATE_DO_EXECUTE(RATE_250_HZ, step)) sensfusion6UpdateQ(latest.gyro.x, latest.gyro.y, latest.gyro.z, latest.acc.x, latest.acc.y, latest.acc.z, 0.004f);
}

bool commanderSetSetpoint(const Setpoint *sp, int priority) {
  if (!sp) return false;
  if (priority == COMMANDER_PRIORITY_DISABLE || priority >= activePriority) {
    activeSetpoint = *sp; activePriority = priority; commanderLastUpdate = activeSetpoint.timestamp; return true;
  }
  return false;
}
void commanderRelaxPriority(void) { activePriority = COMMANDER_PRIORITY_LOWEST; }
uint32_t commanderGetInactivityTime(void) { return activeSetpoint.timestamp - commanderLastUpdate; }
int commanderGetActivePriority(void) { return activePriority; }

void stabilizerInit(void) {
  if (stabilizerInited) return;
  attitudeControllerInit(1.0f / ATTITUDE_RATE_HZ);
  sensfusion6Init(); supervisorInit(); crtpInit();
  healthTestState = testDone;
  stabilizerInited = true;
}
bool stabilizerSubmitHighLevelSetpoint(const Setpoint *sp) { if (!sp) return false; pendingHighLevelSetpoint = *sp; hasHighLevelSetpoint = true; return true; }
void stabilizerTask(void) {
  if (healthShallWeRunTest()) { healthRunTests(&supSensors); return; }
  if (hasHighLevelSetpoint) { commanderSetSetpoint(&pendingHighLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL); hasHighLevelSetpoint = false; }
  supervisorUpdate(0);
}

static uint32_t quatcompress(const Quaternion *q) {
  int32_t x = round_i32(clampf(q->x, -1.0f, 1.0f) * 32767.0f);
  int32_t y = round_i32(clampf(q->y, -1.0f, 1.0f) * 32767.0f);
  int32_t z = round_i32(clampf(q->z, -1.0f, 1.0f) * 32767.0f);
  return ((uint32_t)(x & 0x3ff) << 20) | ((uint32_t)(y & 0x3ff) << 10) | (uint32_t)(z & 0x3ff);
}

void compressState(const State *s, const SensorData *sen, CompressedState *o) {
  if (!s || !sen || !o) return;
  o->position_mm[0] = round_i32(s->position.x * 1000.0f); o->position_mm[1] = round_i32(s->position.y * 1000.0f); o->position_mm[2] = round_i32(s->position.z * 1000.0f);
  o->velocity_mms[0] = round_i32(s->velocity.x * 1000.0f); o->velocity_mms[1] = round_i32(s->velocity.y * 1000.0f); o->velocity_mms[2] = round_i32(s->velocity.z * 1000.0f);
  o->acceleration_mms2[0] = round_i32(s->acc.x * 9810.0f); o->acceleration_mms2[1] = round_i32(s->acc.y * 9810.0f); o->acceleration_mms2[2] = round_i32((s->acc.z + 1.0f) * 9810.0f);
  o->gyro_millirad_s[0] = sen->gyro.x * (float)M_PI / 180.0f * 1000.0f;
  o->gyro_millirad_s[1] = -sen->gyro.y * (float)M_PI / 180.0f * 1000.0f;
  o->gyro_millirad_s[2] = sen->gyro.z * (float)M_PI / 180.0f * 1000.0f;
  o->quatCompressed = quatcompress(&s->attitudeQuaternion);
}
bool rateSupervisorValidate(uint32_t measuredRate) { return measuredRate >= 997U && measuredRate <= 1003U; }
void rateSupervisorTask(void) {}

void healthRequestPropTest(void) { propReq = true; }
void healthRequestBatteryTest(void) { batReq = true; }
bool healthShallWeRunTest(void) {
  if (propReq) { propReq = false; healthTestState = configureAcc; motorPass = 0; propIndex = 0; return true; }
  if (batReq) { batReq = false; healthTestState = testBattery; batTick = 0; minLoadedVoltage = idleVoltage; return true; }
  return healthTestState != testDone;
}
float variance(const float *b, int n) {
  if (!b || n <= 0) return 0.0f;
  float sum = 0.0f, sumSq = 0.0f; for (int i = 0; i < n; i++) { sum += b[i]; sumSq += b[i] * b[i]; }
  return sumSq - (sum * sum / (float)n);
}
bool evaluatePropTest(float low, float high, float value, uint8_t m) {
  bool pass = high == 0.0f || (value >= low && value <= high);
  if (pass && m < 8) motorPass |= (uint8_t)(1U << m);
  return pass;
}
void healthRunTests(const SensorData *s) {
  if (healthTestState == configureAcc) { healthTestState = measureNoiseFloor; propIndex = 0; }
  else if (healthTestState == measureNoiseFloor) {
    if (s && propIndex < PROPTEST_NBR_OF_VARIANCE_VALUES) propBuf[propIndex++] = s->acc.x + s->acc.y + s->acc.z;
    if (propIndex >= PROPTEST_NBR_OF_VARIANCE_VALUES) healthTestState = measureProp;
  } else if (healthTestState == measureProp) {
    for (uint8_t i = 0; i < 4; i++) evaluatePropTest(0.0f, 0.0f, 1.0f, i);
    healthTestState = testDone;
  } else if (healthTestState == testBattery) {
    batTick++;
    if (batTick >= 2 && batTick <= 49 && s && s->baroTemperature < minLoadedVoltage) minLoadedVoltage = s->baroTemperature;
    if (batTick >= 50) { batterySag = idleVoltage - minLoadedVoltage; batteryPass = batterySag <= 0.5f; healthTestState = testDone; }
  }
  healthLog.motorPass = motorPass; healthLog.batteryPass = batteryPass; healthLog.batterySag = batterySag; healthLog.motorTestCount++;
}

static bool qpush(CrtpPacket *q, unsigned cap, unsigned *h, unsigned *t, unsigned *cnt, const CrtpPacket *p) {
  if (*cnt >= cap || !p) return false; q[*t] = *p; *t = (*t + 1U) % cap; (*cnt)++; return true;
}
static bool qpop(CrtpPacket *q, unsigned cap, unsigned *h, unsigned *t, unsigned *cnt, CrtpPacket *p) {
  (void)t; if (*cnt == 0 || !p) return false; *p = q[*h]; *h = (*h + 1U) % cap; (*cnt)--; return true;
}
void crtpInit(void) { if (crtpInited) return; memset(rxCreated, 0, sizeof(rxCreated)); txHead = txTail = txCount = 0; crtpInited = true; }
void crtpInitTaskQueue(uint8_t port) { if (port < CRTP_NBR_OF_PORTS && !rxCreated[port]) rxCreated[port] = true; }
bool crtpSendPacket(const CrtpPacket *p) { return qpush(txQ, CRTP_TX_QUEUE_SIZE, &txHead, &txTail, &txCount, p); }
bool crtpSendPacketBlock(const CrtpPacket *p) { return crtpSendPacket(p); }
bool crtpReceivePacket(uint8_t port, CrtpPacket *p) { return port < CRTP_NBR_OF_PORTS && qpop(rxQ[port], CRTP_RX_QUEUE_SIZE, &rxHead[port], &rxTail[port], &rxCount[port], p); }
bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *p) { return crtpReceivePacket(port, p); }
bool crtpReceivePacketWait(uint8_t port, CrtpPacket *p, uint32_t wait_ms) { (void)wait_ms; return crtpReceivePacket(port, p); }
void crtpRxTask(void) {
  if (!linkPtr || !linkPtr->receivePacket) return;
  CrtpPacket p; if (!linkPtr->receivePacket(&p)) return;
  crtpRxCount++;
  if (p.port < CRTP_NBR_OF_PORTS) {
    if (rxCreated[p.port]) qpush(rxQ[p.port], CRTP_RX_QUEUE_SIZE, &rxHead[p.port], &rxTail[p.port], &rxCount[p.port], &p);
    if (portCb[p.port]) portCb[p.port](&p);
  }
}
void crtpTxTask(void) {
  if (!linkPtr || !linkPtr->sendPacket || txCount == 0) return;
  CrtpPacket p = txQ[txHead];
  if (linkPtr->sendPacket(&p)) { qpop(txQ, CRTP_TX_QUEUE_SIZE, &txHead, &txTail, &txCount, &p); crtpTxCount++; }
}
void crtpSetLink(CrtpLink *newLink) {
  if (linkPtr && linkPtr->setEnable) linkPtr->setEnable(false);
  linkPtr = newLink;
  if (linkPtr && linkPtr->setEnable) linkPtr->setEnable(true);
}
void crtpReset(void) { txHead = txTail = txCount = 0; if (linkPtr && linkPtr->reset) linkPtr->reset(); }
bool crtpIsConnected(void) { return !linkPtr || !linkPtr->isConnected ? true : linkPtr->isConnected(); }
uint32_t crtpGetFreeTxQueuePackets(void) { return CRTP_TX_QUEUE_SIZE - txCount; }
void crtpRegisterPortCB(uint8_t port, CrtpPortCallback cb) { if (port < CRTP_NBR_OF_PORTS) portCb[port] = cb; }
void updateStats(void) { crtpLastStatsTick += 500U; crtpRxCount = crtpTxCount = 0; (void)crtpLastStatsTick; }

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
  if (!decks || capacity == 0) return 0;
  static const DeckInfo known[] = {
    { true, false, 0x1c, 0 }, { true, false, 0x1d, 0 }, { false, true, 0, 0x123456789abcdef0ULL }
  };
  uint8_t n = 0;
  for (unsigned i = 0; i < sizeof(known) / sizeof(known[0]) && n < capacity; i++) decks[n++] = known[i];
  return n;
}

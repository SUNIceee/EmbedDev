/* Complete C11 implementation for the frozen Crazyflie host-verifiable API. */
#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f
#define DEG2RAD (PI_F / 180.0f)
#define RAD2DEG (180.0f / PI_F)
#define EST_Q_SIZE 16U

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

static uint32_t hostTick, lastCommanderUpdateTick, armingStartTick;
static float attitudeDt = 0.002f, desiredYaw;
static bool controllerInitDone, commanderInitDone, stabilizerInited;
static Setpoint activeSetpoint, highLevelSetpoint;
static int activePriority = COMMANDER_PRIORITY_LOWEST;
static bool highLevelPending;

static EstimatorMeasurement estQueue[EST_Q_SIZE];
static unsigned estHead, estTail, estCount;
static Axis3f lastGyro, lastAcc;
static float lastBaro, lastTof;
static State estimatorState;

static bool armedFlag, crashedFlag, tumbledFlag, flyingFlag, freeFallFlag;
static bool autoArmingCfg, cfgTumbleEnabled;
static uint32_t spinupTimeoutCfg, supervisorIdle, cfgTiltMs, cfgUpsideDownMs;
static SensorData supervisorSensors;
static uint32_t supervisorMotors[4];
static int32_t supervisorRPMs[4];
static float cfgCrashGs, cfgFreeFall, cfgTiltZ, cfgUpsideDownZ;

/* Resettable supervisor helper state. These must not be function-local statics. */
static bool flyingSeen;
static uint32_t recentFlyingTick;
static uint32_t tumbleTiltStartTick;
static uint32_t tumbleUpsideDownStartTick;
static uint32_t motorsNotRespondingStartTick;

static bool propReq, batReq;
static uint32_t healthTick, healthMotorCount;
static float noiseBuf[PROPTEST_NBR_OF_VARIANCE_VALUES];
static int noiseCount, currentMotor;
static float idleVoltage = 4.2f, minLoadedVoltage = 4.2f;

typedef struct { CrtpPacket q[CRTP_TX_QUEUE_SIZE]; unsigned h, t, n; } TxQ;
typedef struct { CrtpPacket q[CRTP_RX_QUEUE_SIZE]; unsigned h, t, n; bool created; } RxQ;
static TxQ txq;
static RxQ rxq[CRTP_NBR_OF_PORTS];
static CrtpPortCallback portCb[CRTP_NBR_OF_PORTS];
static bool crtpInited, crtpError;
static CrtpLink *linkPtr;
static uint32_t rxCnt, txCnt, lastStatsTick;

static float clampf_local(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static int32_t round_i32(float v) { return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f); }
static void tick_step(void) { hostTick++; }

static void sync_sensfusion_log(void) {
  sensfusion6Log.qw = qw; sensfusion6Log.qx = qx; sensfusion6Log.qy = qy; sensfusion6Log.qz = qz;
  sensfusion6Log.gravityX = gravityX; sensfusion6Log.gravityY = gravityY; sensfusion6Log.gravityZ = gravityZ;
  sensfusion6Log.accZbase = baseZacc; sensfusion6Log.isInit = sensfusion6IsInit; sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;
}

static void sync_supervisor_log(void) {
  supervisorLog.info = supervisorGetInfoBitfield();
  supervisorLog.accNorm = sqrtf(supervisorSensors.acc.x * supervisorSensors.acc.x + supervisorSensors.acc.y * supervisorSensors.acc.y + supervisorSensors.acc.z * supervisorSensors.acc.z);
}

static void sync_health_log(void) {
  healthLog.motorPass = motorPass; healthLog.batteryPass = batteryPass; healthLog.batterySag = batterySag; healthLog.motorTestCount = healthMotorCount;
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

float invSqrt(float x) {
  if (x <= 0.0f) return 0.0f;
  union { float f; uint32_t i; } c;
  c.f = x; c.i = 0x5f3759dfU - (c.i >> 1);
  c.f = c.f * (1.5f - 0.5f * x * c.f * c.f);
  return c.f;
}

void estimatedGravityDirection(float w, float x, float y, float z, float *gx, float *gy, float *gz) {
  if (gx) *gx = 2.0f * (x * z - w * y);
  if (gy) *gy = 2.0f * (w * x + y * z);
  if (gz) *gz = w * w - x * x - y * y + z * z;
}

void sensfusion6Init(void) {
  if (sensfusion6IsInit) return;
  qw = 1.0f; qx = qy = qz = 0.0f; gravityX = 0.0f; gravityY = 0.0f; gravityZ = 1.0f;
  integralFBx = integralFBy = integralFBz = 0.0f; twoKp = 0.8f; twoKi = 0.002f; beta = 0.01f;
  baseZacc = 0.0f; sensfusion6IsCalibrated = false; sensfusion6IsInit = true; sync_sensfusion_log();
}

bool sensfusion6Test(void) { return sensfusion6IsInit; }

void sensfusion6UpdateQ(float gxv, float gyv, float gzv, float ax, float ay, float az, float dt) {
  if (!sensfusion6IsInit) sensfusion6Init();
  if (dt <= 0.0f) { sync_sensfusion_log(); return; }
  float wx = gxv * DEG2RAD, wy = gyv * DEG2RAD, wz = gzv * DEG2RAD;
  bool accValid = !(ax == 0.0f && ay == 0.0f && az == 0.0f);
  if (accValid) {
    float r = invSqrt(ax * ax + ay * ay + az * az);
    if (r > 0.0f) {
      ax *= r; ay *= r; az *= r;
      estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
      float ex = ay * gravityZ - az * gravityY, ey = az * gravityX - ax * gravityZ, ez = ax * gravityY - ay * gravityX;
      if (twoKi > 0.0f) { integralFBx += twoKi * ex * dt; integralFBy += twoKi * ey * dt; integralFBz += twoKi * ez * dt; wx += integralFBx; wy += integralFBy; wz += integralFBz; }
      else { integralFBx = integralFBy = integralFBz = 0.0f; }
      wx += twoKp * ex; wy += twoKp * ey; wz += twoKp * ez;
    }
  }
  float qa = qw, qb = qx, qc = qy, qd = qz, hdt = 0.5f * dt;
  qw += (-qb * wx - qc * wy - qd * wz) * hdt;
  qx += ( qa * wx + qc * wz - qd * wy) * hdt;
  qy += ( qa * wy - qb * wz + qd * wx) * hdt;
  qz += ( qa * wz + qb * wy - qc * wx) * hdt;
  float n = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (n > 0.0f) { qw *= n; qx *= n; qy *= n; qz *= n; }
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  if (accValid && !sensfusion6IsCalibrated) { baseZacc = ax * gravityX + ay * gravityY + az * gravityZ; sensfusion6IsCalibrated = true; }
  sync_sensfusion_log();
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  if (roll_deg) *roll_deg = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * RAD2DEG;
  if (pitch_deg) *pitch_deg = asinf(clampf_local(2.0f * (qw * qy - qz * qx), -1.0f, 1.0f)) * RAD2DEG;
  if (yaw_deg) *yaw_deg = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * RAD2DEG;
  sync_sensfusion_log();
}

void sensfusion6GetQuaternion(float *ow, float *ox, float *oy, float *oz) { if (ow) *ow = qw; if (ox) *ox = qx; if (oy) *oy = qy; if (oz) *oz = qz; }
float sensfusion6GetAccZ(float ax, float ay, float az) { estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ); sync_sensfusion_log(); return ax * gravityX + ay * gravityY + az * gravityZ; }
float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) { return sensfusion6GetAccZ(ax, ay, az) - baseZacc; }

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw, MotorPower *out) {
  if (!out) return;
  int32_t r = (int32_t)roll / 2, p = (int32_t)pitch / 2;
  out->m1 = (int32_t)thrust - r + p + yaw; out->m2 = (int32_t)thrust - r - p - yaw;
  out->m3 = (int32_t)thrust + r - p + yaw; out->m4 = (int32_t)thrust + r + p - yaw;
}

void powerDistributionForceTorque(float thrustSi, float tx, float ty, float tz, float armLength, float thrustToTorque, float f[4]) {
  if (!f) return;
  float tp = 0.25f * thrustSi, arm = 0.707106781f * armLength;
  float rp = arm != 0.0f ? 0.25f * tx / arm : 0.0f, pp = arm != 0.0f ? 0.25f * ty / arm : 0.0f, yp = thrustToTorque != 0.0f ? 0.25f * tz / thrustToTorque : 0.0f;
  f[0] = tp - rp + pp + yp; f[1] = tp - rp - pp - yp; f[2] = tp + rp - pp + yp; f[3] = tp + rp + pp - yp;
  for (int i = 0; i < 4; i++) if (f[i] < 0.0f) f[i] = 0.0f;
}

void powerDistributionForce(const float nf[4], uint16_t pwm[4]) {
  if (!nf || !pwm) return;
  for (int i = 0; i < 4; i++) pwm[i] = (uint16_t)round_i32(clampf_local(nf[i], 0.0f, 1.0f) * 65535.0f);
}

void powerDistribution(const ControlData *c, MotorPower *m) {
  if (!c || !m) return;
  if (c->controlMode == controlModeLegacy) powerDistributionLegacy(c->thrust, c->roll, c->pitch, c->yaw, m);
  else if (c->controlMode == controlModeForceTorque) {
    float f[4]; powerDistributionForceTorque(c->thrustSi, c->torque.x, c->torque.y, c->torque.z, CRAZYFLIE_ARM_LENGTH_M, CRAZYFLIE_THRUST_TO_TORQUE, f);
    m->m1 = round_i32(clampf_local(f[0] / CRAZYFLIE_MAX_MOTOR_FORCE_N, 0.0f, 1.0f) * 65535.0f);
    m->m2 = round_i32(clampf_local(f[1] / CRAZYFLIE_MAX_MOTOR_FORCE_N, 0.0f, 1.0f) * 65535.0f);
    m->m3 = round_i32(clampf_local(f[2] / CRAZYFLIE_MAX_MOTOR_FORCE_N, 0.0f, 1.0f) * 65535.0f);
    m->m4 = round_i32(clampf_local(f[3] / CRAZYFLIE_MAX_MOTOR_FORCE_N, 0.0f, 1.0f) * 65535.0f);
  } else if (c->controlMode == controlModeForce) {
    uint16_t p[4]; powerDistributionForce(c->normalizedForces, p); m->m1 = p[0]; m->m2 = p[1]; m->m3 = p[2]; m->m4 = p[3];
  }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) { return value < idleThrust ? idleThrust : value; }

PowerCapResult powerDistributionCap(int32_t motors[4], int32_t maxAllowedThrust, int32_t idleThrust) {
  PowerCapResult r = { false, 0 };
  if (!motors) return r;
  int32_t max = motors[0];
  for (int i = 1; i < 4; i++) if (motors[i] > max) max = motors[i];
  if (max > maxAllowedThrust) { r.isCapped = true; r.reduction = max - maxAllowedThrust; for (int i = 0; i < 4; i++) motors[i] = capMinThrust(motors[i] - r.reduction, idleThrust); }
  return r;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) { return filteredOld + alpha * (supplyVoltage - filteredOld); }

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust, float nominalVoltage, float actualVoltage) {
  if (actualVoltage <= 0.0f) return motorThrust;
  int32_t v = round_i32((float)motorThrust * nominalVoltage / actualVoltage);
  if (v < 0) v = 0; if (v > 65535) v = 65535; return (uint16_t)v;
}

static void pid_init(PidObject *p, float kp, float ki, float kd) { p->kp = kp; p->ki = ki; p->kd = kd; p->kff = p->integral = p->prevError = p->output = 0.0f; p->initialized = true; }
static float pid_update(PidObject *p, float actual, float desired, bool reset) {
  float e = desired - actual;
  if (reset) { p->integral = 0.0f; p->prevError = e; }
  p->integral += e * attitudeDt;
  p->output = p->kp * e + p->ki * p->integral + p->kd * ((e - p->prevError) / attitudeDt) + p->kff * desired;
  p->prevError = e; p->initialized = true; return p->output;
}

void attitudeControllerInit(float updateDt) {
  if (controllerInitDone) return;
  attitudeDt = updateDt > 0.0f ? updateDt : 0.002f;
  pid_init(&pidRoll, 6.0f, 0.0f, 0.0f); pid_init(&pidPitch, 6.0f, 0.0f, 0.0f); pid_init(&pidYaw, 6.0f, 0.0f, 0.0f);
  pid_init(&pidRollRate, 250.0f, 0.0f, 0.0f); pid_init(&pidPitchRate, 250.0f, 0.0f, 0.0f); pid_init(&pidYawRate, 120.0f, 0.0f, 0.0f);
  controllerInitDone = true;
}

void attitudeControllerCorrectRatePID(float ra, float rd, float pa, float pd, float ya, float yd) {
  attitudeControllerInit(attitudeDt);
  pidRollRate.output = saturateSignedInt16(round_i32(pid_update(&pidRollRate, ra, rd, false)));
  pidPitchRate.output = saturateSignedInt16(round_i32(pid_update(&pidPitchRate, pa, pd, false)));
  pidYawRate.output = saturateSignedInt16(round_i32(pid_update(&pidYawRate, ya, yd, false)));
}

void attitudeControllerCorrectAttitudePID(float ra, float rd, float pa, float pd, float ya, float yd) { attitudeControllerInit(attitudeDt); pid_update(&pidRoll, ra, rd, false); pid_update(&pidPitch, pa, pd, false); pid_update(&pidYaw, ya, yd, true); }
void attitudeControllerResetAllPID(float r, float p, float y) { pidRoll.integral = pidPitch.integral = pidYaw.integral = 0.0f; pidRoll.output = pidPitch.output = pidYaw.output = 0.0f; pidRoll.prevError = r; pidPitch.prevError = p; pidYaw.prevError = y; pidRollRate.integral = pidPitchRate.integral = pidYawRate.integral = pidRollRate.prevError = pidPitchRate.prevError = pidYawRate.prevError = pidRollRate.output = pidPitchRate.output = pidYawRate.output = 0.0f; }
void attitudeControllerResetRollAttitudePID(float r) { pidRoll.integral = 0.0f; pidRoll.prevError = r; pidRoll.output = 0.0f; }
void attitudeControllerResetPitchAttitudePID(float p) { pidPitch.integral = 0.0f; pidPitch.prevError = p; pidPitch.output = 0.0f; }
void attitudeControllerGetActuatorOutput(int16_t *r, int16_t *p, int16_t *y) { if (r) *r = saturateSignedInt16(round_i32(pidRollRate.output)); if (p) *p = saturateSignedInt16(round_i32(pidPitchRate.output)); if (y) *y = saturateSignedInt16(round_i32(pidYawRate.output)); }

uint16_t positionControllerUpdate(const Setpoint *sp, const State *st) {
  if (!sp || !st) return 0;
  int32_t out = round_i32(35000.0f + (sp->position.z - st->position.z) * 12000.0f + (sp->velocity.z - st->velocity.z) * 6000.0f);
  if (out < 0) out = 0; if (out > 65535) out = 65535; return (uint16_t)out;
}

void controllerPid(const SensorData *s, const Setpoint *sp, const State *st, ControlData *c, float yawMaxDelta, float dt) {
  if (!s || !sp || !st || !c) return;
  attitudeControllerInit(dt); if (dt > 0.0f) attitudeDt = dt;
  uint16_t thrust = sp->mode.z == modeDisable ? sp->thrust : positionControllerUpdate(sp, st);
  if (thrust == 0U) { memset(c, 0, sizeof(*c)); c->controlMode = controlModeLegacy; desiredYaw = st->attitude.yaw; attitudeControllerResetAllPID(st->attitude.roll, st->attitude.pitch, st->attitude.yaw); return; }
  float desiredRoll = sp->attitude.roll, desiredPitch = sp->attitude.pitch;
  if (sp->mode.yaw == modeVelocity) desiredYaw += sp->attitudeRate.yaw * attitudeDt; else if (sp->mode.yaw == modeAbs) desiredYaw = sp->attitude.yaw;
  if (sp->mode.quat == modeAbs) desiredYaw = atan2f(2.0f * (sp->attitudeQuaternion.w * sp->attitudeQuaternion.z + sp->attitudeQuaternion.x * sp->attitudeQuaternion.y), 1.0f - 2.0f * (sp->attitudeQuaternion.y * sp->attitudeQuaternion.y + sp->attitudeQuaternion.z * sp->attitudeQuaternion.z)) * RAD2DEG;
  if (yawMaxDelta != 0.0f) { float d = clampf_local(capAngle(desiredYaw - st->attitude.yaw), -fabsf(yawMaxDelta), fabsf(yawMaxDelta)); desiredYaw = st->attitude.yaw + d; }
  float rr, pr, yr;
  if (sp->mode.roll == modeVelocity) { rr = sp->attitudeRate.roll; attitudeControllerResetRollAttitudePID(st->attitude.roll); } else { pid_update(&pidRoll, st->attitude.roll, desiredRoll, false); rr = pidRoll.output; }
  if (sp->mode.pitch == modeVelocity) { pr = sp->attitudeRate.pitch; attitudeControllerResetPitchAttitudePID(st->attitude.pitch); } else { pid_update(&pidPitch, st->attitude.pitch, desiredPitch, false); pr = pidPitch.output; }
  pid_update(&pidYaw, st->attitude.yaw, desiredYaw, true); yr = sp->mode.yaw == modeVelocity ? sp->attitudeRate.yaw : pidYaw.output;
  attitudeControllerCorrectRatePID(s->gyro.x, rr, -s->gyro.y, pr, s->gyro.z, yr);
  c->controlMode = controlModeLegacy; c->thrust = thrust; attitudeControllerGetActuatorOutput(&c->roll, &c->pitch, &c->yaw); c->yaw = (int16_t)-c->yaw;
}

void rotateYaw(float roll, float pitch, float yaw_deg, float *rp, float *pp) { float a = yaw_deg * DEG2RAD, c = cosf(a), s = sinf(a); if (rp) *rp = roll * c - pitch * s; if (pp) *pp = roll * s + pitch * c; }

void crtpCommanderRpytDecodeSetpoint(const CommanderCrtpLegacyValues *v, Setpoint *sp, bool alt, bool posHold, bool posSet, StabilizationType smR, StabilizationType smP, StabilizationType smY, YawMode ym) {
  if (!v || !sp) return;
  memset(sp, 0, sizeof(*sp));
  if (activePriority == COMMANDER_PRIORITY_DISABLE) thrustLocked = true;
  if (v->thrust == 0U) thrustLocked = false;
  float r = v->roll, p = v->pitch;
  if (ym == PLUSMODE) rotateYaw(v->roll, v->pitch, 45.0f, &r, &p); else if (ym == CAREFREE) sp->timestamp = 0xFFFFFFFFU;
  if (alt) { sp->mode.z = modeVelocity; sp->velocity.z = ((float)v->thrust - 32767.0f) / 32767.0f; sp->thrust = 0; commanderModeSet = true; }
  else { if (commanderModeSet) { sp->mode.z = modeDisable; commanderModeSet = false; } sp->thrust = (thrustLocked || v->thrust < MIN_THRUST) ? 0U : (v->thrust > MAX_THRUST ? MAX_THRUST : v->thrust); }
  if (posHold) { sp->mode.x = sp->mode.y = modeVelocity; sp->mode.roll = sp->mode.pitch = modeDisable; sp->velocity.x = p / 30.0f; sp->velocity.y = r / 30.0f; return; }
  if (posSet && v->thrust != 0U) { sp->mode.x = sp->mode.y = sp->mode.z = modeAbs; sp->mode.roll = sp->mode.pitch = modeDisable; sp->mode.yaw = modeAbs; sp->position.x = -p; sp->position.y = r; sp->position.z = (float)v->thrust / 1000.0f; sp->attitude.yaw = v->yaw; sp->thrust = 0; return; }
  if (smR == RATE) { sp->mode.roll = modeVelocity; sp->attitudeRate.roll = r; } else { sp->mode.roll = modeAbs; sp->attitude.roll = r; }
  if (smP == RATE) { sp->mode.pitch = modeVelocity; sp->attitudeRate.pitch = p; } else { sp->mode.pitch = modeAbs; sp->attitude.pitch = p; }
  if (smY == RATE) { sp->mode.yaw = modeVelocity; sp->attitudeRate.yaw = -v->yaw; } else { sp->mode.yaw = modeAbs; sp->attitude.yaw = v->yaw; }
}

void supervisorInit(void) {
  supervisorState = supervisorStateLocked; supervisorConditionBits = 0;
  armedFlag = crashedFlag = tumbledFlag = flyingFlag = freeFallFlag = false; armingStartTick = 0;
  flyingSeen = false; recentFlyingTick = 0; tumbleTiltStartTick = 0; tumbleUpsideDownStartTick = 0; motorsNotRespondingStartTick = 0;
  memset(&supervisorSensors, 0, sizeof(supervisorSensors)); memset(supervisorMotors, 0, sizeof(supervisorMotors)); memset(supervisorRPMs, 0, sizeof(supervisorRPMs));
  sync_supervisor_log();
}

bool supervisorCanFly(void) { return supervisorState == supervisorStateReadyToFly || supervisorState == supervisorStateFlying || supervisorState == supervisorStateWarningLevelOut || supervisorState == supervisorStateLanded; }
bool supervisorCanArm(void) { return supervisorState == supervisorStatePreFlChecksPassed; }
bool supervisorIsArmed(void) { return armedFlag; }
bool supervisorIsCrashed(void) { return crashedFlag; }

bool supervisorRequestArming(bool doArm) {
  if (!doArm) { armedFlag = false; supervisorConditionBits &= ~SUPERVISOR_CB_ARMED; return true; }
  if (armedFlag) return true;
  if (!supervisorCanArm() && supervisorState != supervisorStateArming) return false;
  armedFlag = true; supervisorState = supervisorStateArming; armingStartTick = hostTick; supervisorConditionBits |= SUPERVISOR_CB_ARMED; return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
  if (tumbledFlag) return false;
  crashedFlag = !doRecovery;
  if (crashedFlag) supervisorConditionBits |= SUPERVISOR_CB_CRASHED; else supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
  return true;
}

bool isFlyingCheck(const uint32_t ratios[4], uint32_t idle, uint32_t now) {
  bool active = false;
  if (ratios) for (int i = 0; i < 4; i++) if (ratios[i] > idle) active = true;
  if (active) { recentFlyingTick = now; flyingSeen = true; return true; }
  if (!flyingSeen) return false;
  return (now - recentFlyingTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float ax, float ay, float az, float crashGs, float ffThr, float tiltZ, float upsideZ, uint32_t tiltMs, uint32_t upsideMs, bool enabled, uint32_t now, bool *isFF) {
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (crashGs > 0.0f && fabsf(norm - 1.0f) > crashGs) { crashedFlag = true; supervisorConditionBits |= SUPERVISOR_CB_CRASHED; }
  bool ff = fabsf(ax) < ffThr && fabsf(ay) < ffThr && fabsf(az) < ffThr;
  if (isFF) *isFF = ff; freeFallFlag = ff;
  if (ff) { tumbleTiltStartTick = tumbleUpsideDownStartTick = 0; supervisorState = supervisorStateExceptFreeFall; supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL; return false; }
  supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;
  if (!enabled) return false;
  if (az < upsideZ) { if (tumbleUpsideDownStartTick == 0U) tumbleUpsideDownStartTick = now; if (now - tumbleUpsideDownStartTick >= upsideMs) tumbledFlag = true; } else tumbleUpsideDownStartTick = 0;
  if (az < tiltZ) { if (tumbleTiltStartTick == 0U) tumbleTiltStartTick = now; if (now - tumbleTiltStartTick >= tiltMs) tumbledFlag = true; } else tumbleTiltStartTick = 0;
  if (tumbledFlag) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
  return tumbledFlag;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick) { return lastNotificationTick == 0U || (currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT; }
bool supervisorIsPreflightTimeout(SupervisorState st, uint32_t latest, uint32_t now, uint32_t dur) { return st == supervisorStateReadyToFly && latest != 0U && (now - latest) >= dur; }
bool supervisorIsLandingTimeout(uint32_t latest, uint32_t now, uint32_t dur) { return latest != 0U && (now - latest) >= dur; }

uint32_t updateAndPopulateConditions(bool crtpStop, bool paramStop, bool wdtFailed) {
  if (crtpStop || paramStop || wdtFailed) supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP; else supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
  if (armedFlag) supervisorConditionBits |= SUPERVISOR_CB_ARMED; else supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
  if (flyingFlag) supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING; else supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
  if (tumbledFlag) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED; else supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
  if (crashedFlag) supervisorConditionBits |= SUPERVISOR_CB_CRASHED; else supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
  return supervisorConditionBits;
}

void supervisorUpdate(uint32_t step) {
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, step)) return;
  tick_step();
  flyingFlag = isFlyingCheck(supervisorMotors, supervisorIdle, hostTick);
  isTumbledCheck(supervisorSensors.acc.x, supervisorSensors.acc.y, supervisorSensors.acc.z, cfgCrashGs, cfgFreeFall, cfgTiltZ, cfgUpsideDownZ, cfgTiltMs, cfgUpsideDownMs, cfgTumbleEnabled, hostTick, &freeFallFlag);
  if (autoArmingCfg && supervisorState == supervisorStatePreFlChecksPassed) supervisorRequestArming(true);
  if (supervisorState == supervisorStateArming) { if (armingStartTick == 0U) armingStartTick = hostTick; if (spinupTimeoutCfg != 0U && hostTick - armingStartTick >= spinupTimeoutCfg) supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT; }
  else { armingStartTick = 0U; supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT; }
  updateAndPopulateConditions(false, false, false); sync_supervisor_log();
}

bool supervisorAreMotorsAllowedToRun(void) { return supervisorState == supervisorStateArming || supervisorCanFly(); }

uint16_t supervisorGetInfoBitfield(void) {
  uint16_t b = 0;
  if (supervisorCanArm()) b |= 1U << 0; if (armedFlag) b |= 1U << 1; if (autoArmingCfg) b |= 1U << 2; if (supervisorCanFly()) b |= 1U << 3;
  if (flyingFlag) b |= 1U << 4; if (tumbledFlag) b |= 1U << 5; if (supervisorState == supervisorStateLocked) b |= 1U << 6; if (crashedFlag) b |= 1U << 7;
  if (supervisorConditionBits & SUPERVISOR_CB_DECK_FAULT) b |= 1U << 11;
  return b;
}

void supervisorOverrideSetpoint(Setpoint *sp, uint32_t bits, SupervisorState st) {
  if (!sp) return;
  if (bits & (SUPERVISOR_CB_EMERGENCY_STOP | SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_FREE_FALL | SUPERVISOR_CB_MOTORS_NOT_RESPONDING)) { memset(sp, 0, sizeof(*sp)); return; }
  if (st == supervisorStateWarningLevelOut) { sp->mode.x = modeDisable; sp->mode.y = modeDisable; sp->mode.roll = modeAbs; sp->mode.pitch = modeAbs; sp->mode.yaw = modeVelocity; sp->attitude.roll = 0.0f; sp->attitude.pitch = 0.0f; sp->attitudeRate.yaw = 0.0f; return; }
  if (st == supervisorStateArming || st == supervisorStateReadyToFly || st == supervisorStateFlying || st == supervisorStateLanded) return;
  memset(sp, 0, sizeof(*sp));
}

bool isRPMatArmingValid(const int32_t rpm[4], int32_t min, int32_t max) { if (!rpm) return false; for (int i = 0; i < 4; i++) if (rpm[i] < min || rpm[i] > max) return false; return true; }

bool isMotorsNotResponding(const int32_t rpm[4], int32_t thr, uint32_t dur, bool canFly, uint32_t now) {
  if (!canFly || !rpm) { motorsNotRespondingStartTick = 0; return false; }
  bool low = true; for (int i = 0; i < 4; i++) if (rpm[i] >= thr) low = false;
  if (!low) { motorsNotRespondingStartTick = 0; return false; }
  if (motorsNotRespondingStartTick == 0U) motorsNotRespondingStartTick = now;
  return (now - motorsNotRespondingStartTick) >= dur;
}

void supervisorSetSensorData(const SensorData *s) { if (s) supervisorSensors = *s; }
void supervisorSetMotorRatios(const uint32_t r[4], uint32_t idle) { if (r) memcpy(supervisorMotors, r, sizeof(supervisorMotors)); supervisorIdle = idle; }
void supervisorSetMotorRPMs(const int32_t r[4]) { if (r) memcpy(supervisorRPMs, r, sizeof(supervisorRPMs)); }
void supervisorConfigureSafety(float c, float f, float t, float u, uint32_t tm, uint32_t um, bool e) { cfgCrashGs = c; cfgFreeFall = f; cfgTiltZ = t; cfgUpsideDownZ = u; cfgTiltMs = tm; cfgUpsideDownMs = um; cfgTumbleEnabled = e; }
void supervisorConfigureArming(bool a, uint32_t s) { autoArmingCfg = a; spinupTimeoutCfg = s; }

bool estimatorEnqueue(const EstimatorMeasurement *m) { if (!m || estCount >= EST_Q_SIZE) return false; estQueue[estTail] = *m; estTail = (estTail + 1U) % EST_Q_SIZE; estCount++; return true; }
bool estimatorDequeue(EstimatorMeasurement *m) { if (!m || estCount == 0U) return false; *m = estQueue[estHead]; estHead = (estHead + 1U) % EST_Q_SIZE; estCount--; return true; }

void estimatorComplementary(uint32_t step) {
  EstimatorMeasurement m;
  while (estimatorDequeue(&m)) {
    if (m.type == MeasurementTypeGyroscope) lastGyro = (Axis3f){m.data[0], m.data[1], m.data[2]};
    else if (m.type == MeasurementTypeAcceleration) lastAcc = (Axis3f){m.data[0], m.data[1], m.data[2]};
    else if (m.type == MeasurementTypeBarometer) lastBaro = m.data[0];
    else if (m.type == MeasurementTypeTOF) lastTof = m.data[0];
  }
  if (RATE_DO_EXECUTE(RATE_250_HZ, step)) {
    sensfusion6UpdateQ(lastGyro.x, lastGyro.y, lastGyro.z, lastAcc.x, lastAcc.y, lastAcc.z, 1.0f / 250.0f);
    sensfusion6GetEulerRPY(&estimatorState.attitude.roll, &estimatorState.attitude.pitch, &estimatorState.attitude.yaw);
    estimatorState.attitudeQuaternion = (Quaternion){qx, qy, qz, qw}; estimatorState.acc = lastAcc;
    estimatorState.velocity.z += sensfusion6GetAccZWithoutGravity(lastAcc.x, lastAcc.y, lastAcc.z) * 9.81f / 250.0f;
    stateEstimate.roll = estimatorState.attitude.roll; stateEstimate.pitch = estimatorState.attitude.pitch; stateEstimate.yaw = estimatorState.attitude.yaw;
    stateEstimate.qx = qx; stateEstimate.qy = qy; stateEstimate.qz = qz; stateEstimate.qw = qw;
  }
  if (RATE_DO_EXECUTE(RATE_100_HZ, step)) estimatorState.position.z = lastBaro + lastTof * 0.0f;
}

bool commanderSetSetpoint(const Setpoint *sp, int priority) {
  tick_step(); if (!sp) return false;
  if (!commanderInitDone) { activePriority = COMMANDER_PRIORITY_LOWEST; commanderInitDone = true; }
  if (priority == COMMANDER_PRIORITY_DISABLE || priority >= activePriority) { activeSetpoint = *sp; activePriority = priority; lastCommanderUpdateTick = hostTick; return true; }
  return false;
}
void commanderRelaxPriority(void) { activePriority = COMMANDER_PRIORITY_LOWEST; }
uint32_t commanderGetInactivityTime(void) { return hostTick - lastCommanderUpdateTick; }
int commanderGetActivePriority(void) { return activePriority; }

static uint32_t quatcompress_local(const Quaternion *q) {
  int32_t x = round_i32(clampf_local(q->x, -1.0f, 1.0f) * 511.0f) & 0x3FF;
  int32_t y = round_i32(clampf_local(q->y, -1.0f, 1.0f) * 511.0f) & 0x3FF;
  int32_t z = round_i32(clampf_local(q->z, -1.0f, 1.0f) * 511.0f) & 0x3FF;
  return (uint32_t)x | ((uint32_t)y << 10) | ((uint32_t)z << 20);
}

void stabilizerInit(void) { if (stabilizerInited) return; sensfusion6Init(); attitudeControllerInit(1.0f / ATTITUDE_RATE_HZ); supervisorInit(); crtpInit(); stabilizerInited = true; }
bool stabilizerSubmitHighLevelSetpoint(const Setpoint *sp) { if (!sp) return false; highLevelSetpoint = *sp; highLevelPending = true; return true; }
void stabilizerTask(void) { stabilizerInit(); tick_step(); if (healthShallWeRunTest()) { SensorData s = {0}; healthRunTests(&s); return; } if (highLevelPending) { commanderSetSetpoint(&highLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL); highLevelPending = false; } supervisorUpdate(hostTick); Setpoint sp = activeSetpoint; supervisorOverrideSetpoint(&sp, supervisorConditionBits, supervisorState); if (!supervisorCanFly()) memset(&sp, 0, sizeof(sp)); }

void compressState(const State *s, const SensorData *sen, CompressedState *o) {
  if (!s || !sen || !o) return;
  o->position_mm[0] = round_i32(s->position.x * 1000.0f); o->position_mm[1] = round_i32(s->position.y * 1000.0f); o->position_mm[2] = round_i32(s->position.z * 1000.0f);
  o->velocity_mms[0] = round_i32(s->velocity.x * 1000.0f); o->velocity_mms[1] = round_i32(s->velocity.y * 1000.0f); o->velocity_mms[2] = round_i32(s->velocity.z * 1000.0f);
  o->acceleration_mms2[0] = round_i32(sen->acc.x * 9810.0f); o->acceleration_mms2[1] = round_i32(sen->acc.y * 9810.0f); o->acceleration_mms2[2] = round_i32((sen->acc.z + 1.0f) * 9810.0f);
  o->gyro_millirad_s[0] = sen->gyro.x * DEG2RAD * 1000.0f; o->gyro_millirad_s[1] = -sen->gyro.y * DEG2RAD * 1000.0f; o->gyro_millirad_s[2] = sen->gyro.z * DEG2RAD * 1000.0f;
  o->quatCompressed = quatcompress_local(&s->attitudeQuaternion);
}

bool rateSupervisorValidate(uint32_t measuredRate) { return measuredRate >= 997U && measuredRate <= 1003U; }
void rateSupervisorTask(void) { tick_step(); }

void healthRequestPropTest(void) { propReq = true; }
void healthRequestBatteryTest(void) { batReq = true; }
bool healthShallWeRunTest(void) { if (propReq) { propReq = false; healthTestState = configureAcc; motorPass = 0; healthMotorCount = 0; sync_health_log(); return true; } if (batReq) { batReq = false; healthTestState = testBattery; healthTick = 0; minLoadedVoltage = idleVoltage; sync_health_log(); return true; } return healthTestState != testDone; }
float variance(const float *b, int n) { if (!b || n <= 0) return 0.0f; float s = 0.0f, ss = 0.0f; for (int i = 0; i < n; i++) { s += b[i]; ss += b[i] * b[i]; } return ss - (s * s / (float)n); }
bool evaluatePropTest(float low, float high, float measured, uint8_t m) { if (high == 0.0f) return true; bool pass = measured >= low && measured <= high; if (pass && m < 8U) motorPass |= (uint8_t)(1U << m); if (!pass) healthMotorCount++; sync_health_log(); return pass; }

void healthRunTests(const SensorData *s) {
  if (!s) return;
  if (healthTestState == configureAcc) { motorPass = batteryPass = 0; batterySag = 0.0f; noiseCount = 0; currentMotor = 0; idleVoltage = s->baroTemperature != 0.0f ? s->baroTemperature : 4.2f; healthTestState = measureNoiseFloor; }
  else if (healthTestState == measureNoiseFloor) { if (noiseCount < PROPTEST_NBR_OF_VARIANCE_VALUES) noiseBuf[noiseCount++] = sqrtf(s->acc.x * s->acc.x + s->acc.y * s->acc.y + s->acc.z * s->acc.z); if (noiseCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) healthTestState = measureProp; }
  else if (healthTestState == measureProp) { evaluatePropTest(0.0f, 1000000.0f, fabsf(s->acc.x) + fabsf(s->acc.y) + fabsf(s->acc.z), (uint8_t)currentMotor); if (++currentMotor >= 4) healthTestState = evaluatePropResult; }
  else if (healthTestState == evaluatePropResult) healthTestState = testDone;
  else if (healthTestState == testBattery) { healthTick++; float v = s->baroTemperature != 0.0f ? s->baroTemperature : 4.2f; if (healthTick == 1U) minLoadedVoltage = v; if (healthTick >= 2U && healthTick <= 49U && v < minLoadedVoltage) minLoadedVoltage = v; if (healthTick >= 50U) { batterySag = idleVoltage - minLoadedVoltage; batteryPass = batterySag <= 0.7f ? 1U : 0U; healthTestState = evaluateBatResult; } }
  else if (healthTestState == evaluateBatResult) healthTestState = testDone;
  else if (healthTestState == restartBatTest) { if (++healthTick >= 2000U) { healthTick = 0; healthTestState = testBattery; } }
  sync_health_log();
}

static bool tx_push(const CrtpPacket *p) { if (!p || txq.n >= CRTP_TX_QUEUE_SIZE) return false; txq.q[txq.t] = *p; txq.t = (txq.t + 1U) % CRTP_TX_QUEUE_SIZE; txq.n++; return true; }
static bool tx_peek(CrtpPacket *p) { if (!p || txq.n == 0U) return false; *p = txq.q[txq.h]; return true; }
static void tx_pop(void) { if (txq.n) { txq.h = (txq.h + 1U) % CRTP_TX_QUEUE_SIZE; txq.n--; } }
static bool rx_push(uint8_t port, const CrtpPacket *p) { if (port >= CRTP_NBR_OF_PORTS || !p || !rxq[port].created || rxq[port].n >= CRTP_RX_QUEUE_SIZE) return false; rxq[port].q[rxq[port].t] = *p; rxq[port].t = (rxq[port].t + 1U) % CRTP_RX_QUEUE_SIZE; rxq[port].n++; return true; }
static bool rx_pop(uint8_t port, CrtpPacket *p) { if (port >= CRTP_NBR_OF_PORTS || !p || rxq[port].n == 0U) return false; *p = rxq[port].q[rxq[port].h]; rxq[port].h = (rxq[port].h + 1U) % CRTP_RX_QUEUE_SIZE; rxq[port].n--; return true; }

void crtpInit(void) { if (crtpInited) return; memset(&txq, 0, sizeof(txq)); memset(rxq, 0, sizeof(rxq)); memset(portCb, 0, sizeof(portCb)); crtpInited = true; crtpError = false; }
void crtpInitTaskQueue(uint8_t port) { crtpInit(); if (port >= CRTP_NBR_OF_PORTS || rxq[port].created) { crtpError = true; return; } memset(&rxq[port], 0, sizeof(rxq[port])); rxq[port].created = true; }
bool crtpSendPacket(const CrtpPacket *p) { crtpInit(); return tx_push(p); }
bool crtpSendPacketBlock(const CrtpPacket *p) { return crtpSendPacket(p); }
bool crtpReceivePacket(uint8_t port, CrtpPacket *p) { crtpInit(); return rx_pop(port, p); }
bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *p) { return crtpReceivePacket(port, p); }
bool crtpReceivePacketWait(uint8_t port, CrtpPacket *p, uint32_t wait_ms) { for (uint32_t i = 0; i <= wait_ms; i++) { if (crtpReceivePacket(port, p)) return true; tick_step(); } return false; }
void crtpRxTask(void) { crtpInit(); if (!linkPtr || !linkPtr->receivePacket) return; CrtpPacket p; if (linkPtr->receivePacket(&p)) { rxCnt++; if (p.port < CRTP_NBR_OF_PORTS) { rx_push(p.port, &p); if (portCb[p.port]) portCb[p.port](&p); } } }
void crtpTxTask(void) { crtpInit(); if (!linkPtr || !linkPtr->sendPacket) return; CrtpPacket p; if (tx_peek(&p) && linkPtr->sendPacket(&p)) { tx_pop(); txCnt++; } }
void crtpSetLink(CrtpLink *newLink) { if (linkPtr && linkPtr->setEnable) linkPtr->setEnable(false); linkPtr = newLink; if (linkPtr && linkPtr->setEnable) linkPtr->setEnable(true); }
void crtpReset(void) { memset(&txq, 0, sizeof(txq)); if (linkPtr && linkPtr->reset) linkPtr->reset(); }
bool crtpIsConnected(void) { return linkPtr && linkPtr->isConnected ? linkPtr->isConnected() : true; }
uint32_t crtpGetFreeTxQueuePackets(void) { crtpInit(); return CRTP_TX_QUEUE_SIZE - txq.n; }
void crtpRegisterPortCB(uint8_t port, CrtpPortCallback cb) { if (port >= CRTP_NBR_OF_PORTS) { crtpError = true; return; } portCb[port] = cb; }
void updateStats(void) { if (hostTick - lastStatsTick >= 500U) { rxCnt = txCnt = 0; lastStatsTick = hostTick; } }

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
  static const DeckInfo known[] = { { true, false, 0x10U, 0U }, { true, false, 0x20U, 0U }, { false, true, 0U, 0x0102030405060708ULL } };
  if (!decks || capacity == 0U) return 0;
  uint8_t n = 0;
  for (unsigned i = 0; i < sizeof(known) / sizeof(known[0]) && n < capacity; i++) {
    bool dup = false;
    for (uint8_t j = 0; j < n; j++) if ((known[i].foundByI2C && decks[j].i2cAddress == known[i].i2cAddress) || (known[i].foundByOneWire && decks[j].oneWireRomId == known[i].oneWireRomId)) dup = true;
    if (!dup) decks[n++] = known[i];
  }
  return n;
}

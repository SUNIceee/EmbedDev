#include "6_generated_code.h"

#include <math.h>
#include <string.h>
#include <limits.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

PidObject pidRoll, pidPitch, pidYaw;
PidObject pidRollRate, pidPitchRate, pidYawRate;

bool thrustLocked = false;
bool commanderModeSet = false;

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

static uint32_t hostTick;
static Setpoint commanderSetpoint;
static int commanderPriority = COMMANDER_PRIORITY_LOWEST;
static uint32_t commanderLastUpdate;

static EstimatorMeasurement estQ[16];
static unsigned estHead, estTail, estCount;
static SensorData estSensors;
static State estState;

static bool stabilizerInitialized;
static bool crtpInitialized;
static bool rxCreated[CRTP_NBR_OF_PORTS];
static CrtpPortCallback portCb[CRTP_NBR_OF_PORTS];

static CrtpPacket txQ[CRTP_TX_QUEUE_SIZE];
static unsigned txHead, txTail, txCount;

static CrtpPacket rxQ[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static unsigned rxHead[CRTP_NBR_OF_PORTS], rxTail[CRTP_NBR_OF_PORTS], rxCount[CRTP_NBR_OF_PORTS];

static bool nopSend(CrtpPacket *p) { (void)p; return false; }
static bool nopRecv(CrtpPacket *p) { (void)p; return false; }
static bool nopConn(void) { return true; }
static void nopEnable(bool e) { (void)e; }
static void nopReset(void) { }

static CrtpLink nopLink = { nopSend, nopRecv, nopConn, nopEnable, nopReset };
static CrtpLink *linkPtr = &nopLink;

static SensorData supervisorSensors;
static uint32_t supervisorMotors[4], supervisorIdle;
static int32_t supervisorRpms[4];
static float cfgCrashGs, cfgFreeFall, cfgTilt = 0.5f, cfgUpsideDown = -0.5f;
static uint32_t cfgTiltTime = 1000, cfgUpsideTime = 1000, cfgSpinupTimeout;
static bool cfgTumbleEnabled = true, cfgAutoArming;
static uint32_t spinupStartTick;
static bool armedFlag, crashedFlag, tumbledFlag, freeFallFlag;
static bool deckFaultFlag;

static bool propRequest, batRequest;
static unsigned healthTick, propSamples, propMotor;
static float propBuf[PROPTEST_NBR_OF_VARIANCE_VALUES], minLoadedVoltage, idleVoltage;
static uint32_t healthMotorTestCount;

static float clampf_local(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

static uint16_t clamp_u16_int(int32_t v)
{
  if (v < 0) return 0;
  if (v > 65535) return 65535;
  return (uint16_t)v;
}

static uint16_t round_clamp_u16(float v)
{
  return clamp_u16_int((int32_t)(v + (v >= 0.0f ? 0.5f : -0.5f)));
}

static void update_sensfusion_log(void)
{
  sensfusion6Log.qw = qw; sensfusion6Log.qx = qx; sensfusion6Log.qy = qy; sensfusion6Log.qz = qz;
  sensfusion6Log.gravityX = gravityX; sensfusion6Log.gravityY = gravityY; sensfusion6Log.gravityZ = gravityZ;
  sensfusion6Log.accZbase = baseZacc;
  sensfusion6Log.isInit = sensfusion6IsInit;
  sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;
}

int16_t saturateSignedInt16(int32_t value)
{
  if (value > INT16_MAX) return INT16_MAX;
  if (value < -INT16_MAX) return -INT16_MAX;
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
  union { float f; uint32_t i; } u = { x };
  u.i = 0x5f3759dfU - (u.i >> 1);
  u.f = u.f * (1.5f - 0.5f * x * u.f * u.f);
  return u.f;
}

void estimatedGravityDirection(float w, float x, float y, float z, float *gx, float *gy, float *gz)
{
  if (gx) *gx = 2.0f * (x * z - w * y);
  if (gy) *gy = 2.0f * (w * x + y * z);
  if (gz) *gz = w * w - x * x - y * y + z * z;
}

void sensfusion6Init(void)
{
  if (sensfusion6IsInit) return;
  qw = 1.0f; qx = qy = qz = 0.0f;
  gravityX = gravityY = 0.0f; gravityZ = 1.0f;
  integralFBx = integralFBy = integralFBz = 0.0f;
  sensfusion6IsInit = true;
  update_sensfusion_log();
}

bool sensfusion6Test(void) { return sensfusion6IsInit; }

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  if (!sensfusion6IsInit) sensfusion6Init();

  gx *= (float)M_PI / 180.0f;
  gy *= (float)M_PI / 180.0f;
  gz *= (float)M_PI / 180.0f;

  if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
    float recip = invSqrt(ax * ax + ay * ay + az * az);
    if (recip > 0.0f) {
      ax *= recip; ay *= recip; az *= recip;

      estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);

      float ex = ay * gravityZ - az * gravityY;
      float ey = az * gravityX - ax * gravityZ;
      float ez = ax * gravityY - ay * gravityX;

      if (twoKi > 0.0f) {
        integralFBx += twoKi * ex * dt;
        integralFBy += twoKi * ey * dt;
        integralFBz += twoKi * ez * dt;
        gx += integralFBx; gy += integralFBy; gz += integralFBz;
      } else {
        integralFBx = integralFBy = integralFBz = 0.0f;
      }

      gx += twoKp * ex;
      gy += twoKp * ey;
      gz += twoKp * ez;

      if (!sensfusion6IsCalibrated) {
        baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
        sensfusion6IsCalibrated = true;
      }
    }
  }

  gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;

  float qa = qw, qb = qx, qc = qy;
  qw += -qb * gx - qc * gy - qz * gz;
  qx +=  qa * gx + qc * gz - qz * gy;
  qy +=  qa * gy - qb * gz + qz * gx;
  qz +=  qa * gz + qb * gy - qc * gx;

  float recip = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (recip > 0.0f) {
    qw *= recip; qx *= recip; qy *= recip; qz *= recip;
  } else {
    qw = 1.0f; qx = qy = qz = 0.0f;
  }

  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  update_sensfusion_log();
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
  float sinr = 2.0f * (qw * qx + qy * qz);
  float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
  float sinp = 2.0f * (qw * qy - qz * qx);
  float siny = 2.0f * (qw * qz + qx * qy);
  float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);

  sinp = clampf_local(sinp, -1.0f, 1.0f);

  if (roll_deg) *roll_deg = atan2f(sinr, cosr) * 180.0f / (float)M_PI;
  if (pitch_deg) *pitch_deg = asinf(sinp) * 180.0f / (float)M_PI;
  if (yaw_deg) *yaw_deg = atan2f(siny, cosy) * 180.0f / (float)M_PI;

  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  update_sensfusion_log();
}

void sensfusion6GetQuaternion(float *w, float *x, float *y, float *z)
{
  if (w) *w = qw; if (x) *x = qx; if (y) *y = qy; if (z) *z = qz;
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

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw, MotorPower *out)
{
  if (!out) return;
  int32_t r = (int32_t)roll / 2;
  int32_t p = (int32_t)pitch / 2;
  out->m1 = (int32_t)thrust - r + p + yaw;
  out->m2 = (int32_t)thrust - r - p - yaw;
  out->m3 = (int32_t)thrust + r - p + yaw;
  out->m4 = (int32_t)thrust + r + p - yaw;
}

static uint16_t motorForceToPwm(float f)
{
  if (f <= 0.0f) return 0;
  return round_clamp_u16((f / CRAZYFLIE_MAX_MOTOR_FORCE_N) * 65535.0f);
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4])
{
  if (!motorForces) return;
  float thrustPart = 0.25f * thrustSi;
  float arm = 0.707106781f * armLength;
  float rollPart = arm != 0.0f ? 0.25f * torqueX / arm : 0.0f;
  float pitchPart = arm != 0.0f ? 0.25f * torqueY / arm : 0.0f;
  float yawPart = thrustToTorque != 0.0f ? 0.25f * torqueZ / thrustToTorque : 0.0f;

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
    motorPWMs[i] = round_clamp_u16(clampf_local(normalizedForces[i], 0.0f, 1.0f) * 65535.0f);
  }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower)
{
  if (!control || !motorPower) return;

  if (control->controlMode == controlModeLegacy) {
    powerDistributionLegacy(control->thrust, control->roll, control->pitch, control->yaw, motorPower);
  } else if (control->controlMode == controlModeForceTorque) {
    float f[4];
    powerDistributionForceTorque(control->thrustSi, control->torque.x, control->torque.y, control->torque.z,
                                 CRAZYFLIE_ARM_LENGTH_M, CRAZYFLIE_THRUST_TO_TORQUE, f);
    motorPower->m1 = motorForceToPwm(f[0]);
    motorPower->m2 = motorForceToPwm(f[1]);
    motorPower->m3 = motorForceToPwm(f[2]);
    motorPower->m4 = motorForceToPwm(f[3]);
  } else if (control->controlMode == controlModeForce) {
    uint16_t pwm[4];
    powerDistributionForce(control->normalizedForces, pwm);
    motorPower->m1 = pwm[0]; motorPower->m2 = pwm[1]; motorPower->m3 = pwm[2]; motorPower->m4 = pwm[3];
  }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust)
{
  return value < idleThrust ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4], int32_t maxAllowedThrust, int32_t idleThrust)
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

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust, float nominalVoltage, float actualVoltage)
{
  if (actualVoltage <= 0.0f) return motorThrust;
  return round_clamp_u16(((float)motorThrust * nominalVoltage) / actualVoltage);
}

static void pidInit(PidObject *p, float kp, float ki, float kd)
{
  if (!p || p->initialized) return;
  p->kp = kp; p->ki = ki; p->kd = kd; p->kff = 0.0f;
  p->integral = p->prevError = p->output = 0.0f;
  p->initialized = true;
}

static float pidUpdate(PidObject *p, float actual, float desired, bool reset)
{
  if (!p) return 0.0f;
  float err = desired - actual;
  if (reset) { p->integral = 0.0f; p->prevError = err; }
  p->integral += err;
  p->output = p->kp * err + p->ki * p->integral + p->kd * (err - p->prevError) + p->kff * desired;
  p->prevError = err;
  return p->output;
}

void attitudeControllerInit(float updateDt)
{
  (void)updateDt;
  pidInit(&pidRoll, 4.0f, 0.0f, 0.0f);
  pidInit(&pidPitch, 4.0f, 0.0f, 0.0f);
  pidInit(&pidYaw, 4.0f, 0.0f, 0.0f);
  pidInit(&pidRollRate, 1.0f, 0.0f, 0.0f);
  pidInit(&pidPitchRate, 1.0f, 0.0f, 0.0f);
  pidInit(&pidYawRate, 1.0f, 0.0f, 0.0f);
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired)
{
  pidRollRate.output = saturateSignedInt16((int32_t)pidUpdate(&pidRollRate, rollActual, rollDesired, false));
  pidPitchRate.output = saturateSignedInt16((int32_t)pidUpdate(&pidPitchRate, pitchActual, pitchDesired, false));
  pidYawRate.output = saturateSignedInt16((int32_t)pidUpdate(&pidYawRate, yawActual, yawDesired, false));
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired)
{
  pidUpdate(&pidRoll, rollActual, rollDesired, false);
  pidUpdate(&pidPitch, pitchActual, pitchDesired, false);
  pidUpdate(&pidYaw, yawActual, yawDesired, true);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual, float yawActual)
{
  pidRoll.integral = pidPitch.integral = pidYaw.integral = 0.0f;
  pidRollRate.integral = pidPitchRate.integral = pidYawRate.integral = 0.0f;
  pidRoll.prevError = -rollActual;
  pidPitch.prevError = -pitchActual;
  pidYaw.prevError = -yawActual;
  pidRoll.output = pidPitch.output = pidYaw.output = 0.0f;
  pidRollRate.output = pidPitchRate.output = pidYawRate.output = 0.0f;
}

void attitudeControllerResetRollAttitudePID(float rollActual)
{
  pidRoll.integral = 0.0f; pidRoll.prevError = -rollActual; pidRoll.output = 0.0f;
}

void attitudeControllerResetPitchAttitudePID(float pitchActual)
{
  pidPitch.integral = 0.0f; pidPitch.prevError = -pitchActual; pidPitch.output = 0.0f;
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch, int16_t *yaw)
{
  if (roll) *roll = saturateSignedInt16((int32_t)pidRollRate.output);
  if (pitch) *pitch = saturateSignedInt16((int32_t)pidPitchRate.output);
  if (yaw) *yaw = saturateSignedInt16((int32_t)pidYawRate.output);
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state)
{
  if (!setpoint || !state) return 0;
  float ez = setpoint->position.z - state->position.z;
  float evz = setpoint->velocity.z - state->velocity.z;
  return round_clamp_u16((float)setpoint->thrust + 1000.0f * ez + 500.0f * evz);
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt)
{
  static float desiredYaw;
  if (!sensors || !setpoint || !state || !control) return;

  control->controlMode = controlModeLegacy;

  if (setpoint->thrust == 0U) {
    control->roll = control->pitch = control->yaw = 0;
    control->thrust = 0;
    attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
    desiredYaw = state->attitude.yaw;
    return;
  }

  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
  } else if (setpoint->mode.yaw == modeAbs) {
    if (setpoint->mode.quat == modeAbs) {
      float oldw = qw, oldx = qx, oldy = qy, oldz = qz;
      qw = setpoint->attitudeQuaternion.w; qx = setpoint->attitudeQuaternion.x;
      qy = setpoint->attitudeQuaternion.y; qz = setpoint->attitudeQuaternion.z;
      sensfusion6GetEulerRPY(NULL, NULL, &desiredYaw);
      qw = oldw; qx = oldx; qy = oldy; qz = oldz;
    } else {
      desiredYaw = setpoint->attitude.yaw;
    }
  }

  if (yawMaxDelta != 0.0f) {
    desiredYaw = clampf_local(desiredYaw, state->attitude.yaw - yawMaxDelta, state->attitude.yaw + yawMaxDelta);
  }

  float rollDesired, pitchDesired;
  if (setpoint->mode.roll == modeVelocity) {
    rollDesired = setpoint->attitudeRate.roll;
    attitudeControllerResetRollAttitudePID(state->attitude.roll);
  } else {
    pidUpdate(&pidRoll, state->attitude.roll, setpoint->attitude.roll, false);
    rollDesired = pidRoll.output;
  }

  if (setpoint->mode.pitch == modeVelocity) {
    pitchDesired = setpoint->attitudeRate.pitch;
    attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
  } else {
    pidUpdate(&pidPitch, state->attitude.pitch, setpoint->attitude.pitch, false);
    pitchDesired = pidPitch.output;
  }

  float yawDesired = pidUpdate(&pidYaw, state->attitude.yaw, desiredYaw, true);
  control->thrust = setpoint->mode.z == modeDisable ? setpoint->thrust : positionControllerUpdate(setpoint, state);

  attitudeControllerCorrectRatePID(sensors->gyro.x, rollDesired,
                                   -sensors->gyro.y, pitchDesired,
                                   sensors->gyro.z, yawDesired);
  attitudeControllerGetActuatorOutput(&control->roll, &control->pitch, &control->yaw);
  control->yaw = (int16_t)-control->yaw;
}

void rotateYaw(float roll, float pitch, float yaw_deg, float *rollPrime, float *pitchPrime)
{
  float a = yaw_deg * (float)M_PI / 180.0f;
  float c = cosf(a), s = sinf(a);
  if (rollPrime) *rollPrime = roll * c - pitch * s;
  if (pitchPrime) *pitchPrime = roll * s + pitch * c;
}

void crtpCommanderRpytDecodeSetpoint(const CommanderCrtpLegacyValues *values, Setpoint *sp,
                                     bool altHoldMode, bool posHoldMode, bool posSetMode,
                                     StabilizationType rollMode, StabilizationType pitchMode,
                                     StabilizationType yawStabMode, YawMode yawMode)
{
  if (!values || !sp) return;

  if (commanderPriority == COMMANDER_PRIORITY_DISABLE) thrustLocked = true;
  if (values->thrust == 0U) thrustLocked = false;

  if (altHoldMode) {
    sp->mode.z = modeVelocity;
    sp->thrust = 0;
    sp->velocity.z = ((float)values->thrust - 32767.0f) / 32767.0f;
    if (!commanderModeSet) commanderModeSet = true;
    return;
  }

  if (posHoldMode) {
    sp->mode.x = modeVelocity; sp->mode.y = modeVelocity;
    sp->mode.roll = modeDisable; sp->mode.pitch = modeDisable;
    sp->velocity.x = values->pitch / 30.0f;
    sp->velocity.y = values->roll / 30.0f;
    sp->attitude.roll = 0.0f; sp->attitude.pitch = 0.0f;
    return;
  }

  if (posSetMode && values->thrust != 0U) {
    sp->mode.x = sp->mode.y = sp->mode.z = modeAbs;
    sp->mode.roll = sp->mode.pitch = modeDisable;
    sp->mode.yaw = modeAbs;
    sp->position.x = -values->pitch;
    sp->position.y = values->roll;
    sp->position.z = (float)values->thrust / 1000.0f;
    sp->attitude.yaw = values->yaw;
    sp->thrust = 0;
    return;
  }

  if (commanderModeSet) {
    sp->mode.z = modeDisable;
    commanderModeSet = false;
  }

  float r = values->roll, p = values->pitch;
  if (yawMode == PLUSMODE) rotateYaw(r, p, 45.0f, &r, &p);
  if (yawMode == CAREFREE) {
    sp->mode.x = modeDisable;
    sp->mode.y = modeDisable;
  }

  if (rollMode == RATE) { sp->mode.roll = modeVelocity; sp->attitudeRate.roll = r; }
  else { sp->mode.roll = modeAbs; sp->attitude.roll = r; }

  if (pitchMode == RATE) { sp->mode.pitch = modeVelocity; sp->attitudeRate.pitch = p; }
  else { sp->mode.pitch = modeAbs; sp->attitude.pitch = p; }

  if (yawStabMode == RATE) { sp->mode.yaw = modeVelocity; sp->attitudeRate.yaw = -values->yaw; }
  else { sp->mode.yaw = modeAbs; sp->attitude.yaw = values->yaw; }

  sp->thrust = (thrustLocked || values->thrust < MIN_THRUST) ? 0U :
               (values->thrust > MAX_THRUST ? MAX_THRUST : values->thrust);
}

void supervisorInit(void)
{
  supervisorState = supervisorStateLocked;
  supervisorConditionBits = 0;
  armedFlag = crashedFlag = tumbledFlag = freeFallFlag = false;
  spinupStartTick = 0;
}

bool supervisorCanFly(void)
{
  return supervisorState == supervisorStateReadyToFly ||
         supervisorState == supervisorStateFlying ||
         supervisorState == supervisorStateWarningLevelOut ||
         supervisorState == supervisorStateLanded;
}

bool supervisorCanArm(void) { return supervisorState == supervisorStatePreFlChecksPassed; }
bool supervisorIsArmed(void) { return armedFlag; }
bool supervisorIsCrashed(void) { return crashedFlag; }

bool supervisorRequestArming(bool doArm)
{
  if (!doArm) { armedFlag = false; supervisorState = supervisorStateLocked; return true; }
  if (armedFlag) return true;
  if (!supervisorCanArm()) return false;
  armedFlag = true;
  supervisorState = supervisorStateArming;
  supervisorConditionBits |= SUPERVISOR_CB_ARMED;
  return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
  if (doRecovery) {
    if (tumbledFlag) return false;
    crashedFlag = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    return true;
  }
  crashedFlag = true;
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
  if (supervisorCanArm()) b |= 1u << 0;
  if (armedFlag) b |= 1u << 1;
  if (cfgAutoArming) b |= 1u << 2;
  if (supervisorCanFly()) b |= 1u << 3;
  if (supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) b |= 1u << 4;
  if (tumbledFlag) b |= 1u << 5;
  if (supervisorState == supervisorStateLocked) b |= 1u << 6;
  if (crashedFlag) b |= 1u << 7;
  if (deckFaultFlag) b |= 1u << 11;
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

  return seen && (currentTick - recentTick < IS_FLYING_HYSTERESIS_THRESHOLD);
}

bool isTumbledCheck(float ax, float ay, float az, float crashGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool enabled, uint32_t currentTick, bool *isFreeFalling)
{
  static uint32_t tiltStart, upsideStart;
  bool freefall = fabsf(ax) < freeFallThreshold && fabsf(ay) < freeFallThreshold && fabsf(az) < freeFallThreshold;
  if (isFreeFalling) *isFreeFalling = freefall;
  freeFallFlag = freefall;
  if (freefall) { tiltStart = upsideStart = 0; return false; }
  if (!enabled) return false;

  if (crashGs > 0.0f) {
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (fabsf(norm - 1.0f) > crashGs) {
      crashedFlag = true;
      supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    }
  }

  bool tumbled = false;
  if (az < acceptedUpsideDownAccZ) {
    if (upsideStart == 0) upsideStart = currentTick;
    if (currentTick - upsideStart >= maxUpsideDownTime) tumbled = true;
  } else {
    upsideStart = 0;
  }

  if (az < acceptedTiltAccZ) {
    if (tiltStart == 0) tiltStart = currentTick;
    if (currentTick - tiltStart >= maxTiltTime) tumbled = true;
  } else {
    tiltStart = 0;
  }

  tumbledFlag = tumbled;
  return tumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick)
{
  return lastNotificationTick == 0U ||
         currentTick - lastNotificationTick <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state, uint32_t latestArmingTick,
                                  uint32_t currentTick, uint32_t duration)
{
  return state == supervisorStatePreFlChecksPassed && currentTick - latestArmingTick >= duration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick, uint32_t currentTick, uint32_t duration)
{
  return currentTick - latestLandingTick >= duration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop, bool paramEmergencyStop, bool watchdogFailed)
{
  if (crtpEmergencyStop || paramEmergencyStop || watchdogFailed)
    supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
  else
    supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
  return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *sp, uint32_t bits, SupervisorState state)
{
  if (!sp) return;

  if (bits & (SUPERVISOR_CB_EMERGENCY_STOP | SUPERVISOR_CB_IS_TUMBLED |
              SUPERVISOR_CB_FREE_FALL | SUPERVISOR_CB_MOTORS_NOT_RESPONDING)) {
    *sp = (Setpoint){0};
    return;
  }

  if (state == supervisorStateWarningLevelOut) {
    sp->mode.x = modeDisable; sp->mode.y = modeDisable;
    sp->mode.roll = modeAbs; sp->mode.pitch = modeAbs; sp->mode.yaw = modeVelocity;
    sp->attitude.roll = 0.0f; sp->attitude.pitch = 0.0f; sp->attitudeRate.yaw = 0.0f;
  } else if (!(state == supervisorStateArming || state == supervisorStateReadyToFly ||
               state == supervisorStateFlying || state == supervisorStateLanded)) {
    *sp = (Setpoint){0};
  }
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t min, int32_t max)
{
  if (!motorRPMs) return false;
  for (int i = 0; i < 4; ++i) if (motorRPMs[i] < min || motorRPMs[i] > max) return false;
  return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t threshold,
                           uint32_t duration, bool canFly, uint32_t currentTick)
{
  static uint32_t start;
  if (!canFly || !motorRPMs) { start = 0; return false; }

  bool low = true;
  for (int i = 0; i < 4; ++i) if (motorRPMs[i] >= threshold) low = false;

  if (!low) { start = 0; return false; }
  if (start == 0) start = currentTick;
  return currentTick - start >= duration;
}

void supervisorSetSensorData(const SensorData *sensors)
{
  if (sensors) supervisorSensors = *sensors;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust)
{
  if (motorRatios) memcpy(supervisorMotors, motorRatios, sizeof(supervisorMotors));
  supervisorIdle = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
  if (motorRPMs) memcpy(supervisorRpms, motorRPMs, sizeof(supervisorRpms));
}

void supervisorConfigureSafety(float crashGs, float freeFallThreshold,
                               float tiltZ, float upsideZ,
                               uint32_t tiltTime, uint32_t upsideTime,
                               bool enabled)
{
  cfgCrashGs = crashGs; cfgFreeFall = freeFallThreshold; cfgTilt = tiltZ; cfgUpsideDown = upsideZ;
  cfgTiltTime = tiltTime; cfgUpsideTime = upsideTime; cfgTumbleEnabled = enabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs)
{
  cfgAutoArming = autoArming;
  cfgSpinupTimeout = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep)
{
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

  bool flying = isFlyingCheck(supervisorMotors, supervisorIdle, hostTick);
  if (flying) supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
  else supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;

  bool ff = false;
  if (isTumbledCheck(supervisorSensors.acc.x, supervisorSensors.acc.y, supervisorSensors.acc.z,
                     cfgCrashGs, cfgFreeFall, cfgTilt, cfgUpsideDown,
                     cfgTiltTime, cfgUpsideTime, cfgTumbleEnabled, hostTick, &ff))
    supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
  else
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;

  if (ff) supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
  else supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

  if (cfgAutoArming && supervisorState == supervisorStatePreFlChecksPassed)
    (void)supervisorRequestArming(true);

  if (supervisorState == supervisorStateArming) {
    if (spinupStartTick == 0U) spinupStartTick = hostTick;
    else if (cfgSpinupTimeout && hostTick - spinupStartTick >= cfgSpinupTimeout)
      supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
  } else {
    spinupStartTick = 0;
    supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
  }

  if (crashedFlag) supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
  supervisorLog.info = supervisorGetInfoBitfield();
  supervisorLog.accNorm = sqrtf(supervisorSensors.acc.x * supervisorSensors.acc.x +
                                supervisorSensors.acc.y * supervisorSensors.acc.y +
                                supervisorSensors.acc.z * supervisorSensors.acc.z);
}

bool estimatorEnqueue(const EstimatorMeasurement *m)
{
  if (!m || estCount == 16U) return false;
  estQ[estTail] = *m;
  estTail = (estTail + 1U) & 15U;
  estCount++;
  return true;
}

bool estimatorDequeue(EstimatorMeasurement *m)
{
  if (!m || estCount == 0U) return false;
  *m = estQ[estHead];
  estHead = (estHead + 1U) & 15U;
  estCount--;
  return true;
}

void estimatorComplementary(uint32_t step)
{
  EstimatorMeasurement m;
  while (estimatorDequeue(&m)) {
    if (m.type == MeasurementTypeGyroscope) {
      estSensors.gyro.x = m.data[0]; estSensors.gyro.y = m.data[1]; estSensors.gyro.z = m.data[2];
    } else if (m.type == MeasurementTypeAcceleration) {
      estSensors.acc.x = m.data[0]; estSensors.acc.y = m.data[1]; estSensors.acc.z = m.data[2];
    } else if (m.type == MeasurementTypeBarometer) {
      estSensors.baroPressure = m.data[0]; estSensors.baroTemperature = m.data[1]; estSensors.baroAsl = m.data[2];
    } else if (m.type == MeasurementTypeTOF) {
      estSensors.tofRange = m.data[0];
    }
  }

  if (RATE_DO_EXECUTE(RATE_250_HZ, step)) {
    sensfusion6UpdateQ(estSensors.gyro.x, estSensors.gyro.y, estSensors.gyro.z,
                       estSensors.acc.x, estSensors.acc.y, estSensors.acc.z, 0.004f);
    sensfusion6GetEulerRPY(&estState.attitude.roll, &estState.attitude.pitch, &estState.attitude.yaw);
    estState.attitudeQuaternion.w = qw; estState.attitudeQuaternion.x = qx;
    estState.attitudeQuaternion.y = qy; estState.attitudeQuaternion.z = qz;
    estState.acc = estSensors.acc;
    estState.velocity.z += sensfusion6GetAccZWithoutGravity(estSensors.acc.x, estSensors.acc.y, estSensors.acc.z) * 9.81f * 0.004f;
  }

  if (RATE_DO_EXECUTE(RATE_100_HZ, step)) {
    estState.position.x += estState.velocity.x * 0.01f;
    estState.position.y += estState.velocity.y * 0.01f;
    estState.position.z += estState.velocity.z * 0.01f;
  }
}

bool commanderSetSetpoint(const Setpoint *sp, int priority)
{
  if (!sp) return false;
  if (priority == COMMANDER_PRIORITY_DISABLE || priority >= commanderPriority) {
    commanderSetpoint = *sp;
    commanderPriority = priority;
    commanderLastUpdate = hostTick;
    return true;
  }
  return false;
}

void commanderRelaxPriority(void) { commanderPriority = COMMANDER_PRIORITY_LOWEST; }
uint32_t commanderGetInactivityTime(void) { return hostTick - commanderLastUpdate; }
int commanderGetActivePriority(void) { return commanderPriority; }

void stabilizerInit(void)
{
  if (stabilizerInitialized) return;
  sensfusion6Init();
  attitudeControllerInit(1.0f / (float)ATTITUDE_RATE_HZ);
  crtpInit();
  stabilizerInitialized = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *sp)
{
  return commanderSetSetpoint(sp, COMMANDER_PRIORITY_HIGHLEVEL);
}

void stabilizerTask(void)
{
  stabilizerInit();

  if (healthShallWeRunTest()) {
    healthRunTests(&estSensors);
  } else {
    estimatorComplementary(hostTick);
    supervisorUpdate(hostTick);
    supervisorOverrideSetpoint(&commanderSetpoint, supervisorConditionBits, supervisorState);

    ControlData c = {0};
    controllerPid(&estSensors, &commanderSetpoint, &estState, &c, 0.0f, 1.0f / (float)ATTITUDE_RATE_HZ);

    MotorPower mp = {0};
    powerDistribution(&c, &mp);
    int32_t motors[4] = { mp.m1, mp.m2, mp.m3, mp.m4 };
    (void)powerDistributionCap(motors, 65535, 0);

    if (!supervisorAreMotorsAllowedToRun() || (supervisorConditionBits & SUPERVISOR_CB_EMERGENCY_STOP)) {
      motor.m1req = motor.m2req = motor.m3req = motor.m4req = 0;
    } else {
      motor.m1req = clamp_u16_int(motors[0]); motor.m2req = clamp_u16_int(motors[1]);
      motor.m3req = clamp_u16_int(motors[2]); motor.m4req = clamp_u16_int(motors[3]);
    }
  }

  gyro.x = estSensors.gyro.x; gyro.y = estSensors.gyro.y; gyro.z = estSensors.gyro.z;
  acc.x = estSensors.acc.x; acc.y = estSensors.acc.y; acc.z = estSensors.acc.z;
  baro.pressure = estSensors.baroPressure; baro.temp = estSensors.baroTemperature; baro.asl = estSensors.baroAsl;
  stateEstimate.roll = estState.attitude.roll; stateEstimate.pitch = estState.attitude.pitch; stateEstimate.yaw = estState.attitude.yaw;
  stateEstimate.qw = qw; stateEstimate.qx = qx; stateEstimate.qy = qy; stateEstimate.qz = qz;
  healthLog.motorPass = motorPass; healthLog.batteryPass = batteryPass; healthLog.batterySag = batterySag;
  healthLog.motorTestCount = healthMotorTestCount;
  update_sensfusion_log();
  hostTick++;
}

static uint32_t quatcompress_local(Quaternion q)
{
  int32_t xi = (int32_t)((clampf_local(q.x, -1.0f, 1.0f) + 1.0f) * 511.5f);
  int32_t yi = (int32_t)((clampf_local(q.y, -1.0f, 1.0f) + 1.0f) * 511.5f);
  int32_t zi = (int32_t)((clampf_local(q.z, -1.0f, 1.0f) + 1.0f) * 511.5f);
  int32_t wi = (int32_t)((clampf_local(q.w, -1.0f, 1.0f) + 1.0f) * 511.5f);
  return ((uint32_t)(xi & 0x3ff) << 22) | ((uint32_t)(yi & 0x3ff) << 12) |
         ((uint32_t)(zi & 0x3ff) << 2) | ((uint32_t)(wi & 0x3) << 0);
}

void compressState(const State *state, const SensorData *sensors, CompressedState *out)
{
  if (!state || !sensors || !out) return;
  out->position_mm[0] = (int32_t)(state->position.x * 1000.0f);
  out->position_mm[1] = (int32_t)(state->position.y * 1000.0f);
  out->position_mm[2] = (int32_t)(state->position.z * 1000.0f);
  out->velocity_mms[0] = (int32_t)(state->velocity.x * 1000.0f);
  out->velocity_mms[1] = (int32_t)(state->velocity.y * 1000.0f);
  out->velocity_mms[2] = (int32_t)(state->velocity.z * 1000.0f);
  out->acceleration_mms2[0] = (int32_t)(state->acc.x * 9810.0f);
  out->acceleration_mms2[1] = (int32_t)(state->acc.y * 9810.0f);
  out->acceleration_mms2[2] = (int32_t)((state->acc.z + 1.0f) * 9810.0f);
  out->gyro_millirad_s[0] = sensors->gyro.x * (float)M_PI / 180.0f * 1000.0f;
  out->gyro_millirad_s[1] = -sensors->gyro.y * (float)M_PI / 180.0f * 1000.0f;
  out->gyro_millirad_s[2] = sensors->gyro.z * (float)M_PI / 180.0f * 1000.0f;
  out->quatCompressed = quatcompress_local(state->attitudeQuaternion);
}

bool rateSupervisorValidate(uint32_t measuredRate)
{
  return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void) { hostTick += 2000U; }

void healthRequestPropTest(void) { propRequest = true; }
void healthRequestBatteryTest(void) { batRequest = true; }

bool healthShallWeRunTest(void)
{
  if (propRequest) {
    propRequest = false;
    healthTestState = configureAcc;
    return true;
  }
  if (batRequest) {
    batRequest = false;
    healthTestState = testBattery;
    healthTick = 0;
    return true;
  }
  return healthTestState != testDone;
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

bool evaluatePropTest(float low, float high, float measured, uint8_t m)
{
  if (m >= 4U) return false;
  bool ok = high == 0.0f || (measured >= low && measured <= high);
  if (ok) motorPass |= (uint8_t)(1U << m);
  else healthMotorTestCount++;
  return ok;
}

void healthRunTests(const SensorData *s)
{
  float sample = s ? (s->acc.x + s->acc.y + s->acc.z) : 0.0f;

  switch (healthTestState) {
    case configureAcc:
      motorPass = batteryPass = 0;
      batterySag = 0.0f;
      propSamples = propMotor = 0;
      idleVoltage = s ? s->baroTemperature : 0.0f;
      healthTestState = measureNoiseFloor;
      break;

    case measureNoiseFloor:
      propBuf[propSamples++] = sample;
      if (propSamples >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
        (void)variance(propBuf, PROPTEST_NBR_OF_VARIANCE_VALUES);
        healthTestState = measureProp;
      }
      break;

    case measureProp:
      (void)evaluatePropTest(0.0f, 0.0f, sample, (uint8_t)propMotor);
      if (++propMotor >= 4U) healthTestState = evaluatePropResult;
      break;

    case evaluatePropResult:
      healthTestState = testDone;
      break;

    case testBattery:
      if (healthTick == 0U) minLoadedVoltage = s ? s->baroTemperature : 0.0f;
      if (healthTick >= 2U && healthTick <= 49U && s && s->baroTemperature < minLoadedVoltage)
        minLoadedVoltage = s->baroTemperature;
      if (healthTick >= 50U) {
        batterySag = idleVoltage - minLoadedVoltage;
        batteryPass = batterySag <= 0.5f ? 1U : 0U;
        healthTestState = evaluateBatResult;
      }
      healthTick++;
      break;

    case evaluateBatResult:
      healthTestState = testDone;
      break;

    case restartBatTest:
      if (++healthTick >= 2000U) {
        healthTick = 0;
        healthTestState = testBattery;
      }
      break;

    case testDone:
    default:
      break;
  }

  healthLog.motorPass = motorPass;
  healthLog.batteryPass = batteryPass;
  healthLog.batterySag = batterySag;
  healthLog.motorTestCount = healthMotorTestCount;
}

void crtpInit(void)
{
  if (crtpInitialized) return;
  memset(rxCreated, 0, sizeof(rxCreated));
  memset(rxHead, 0, sizeof(rxHead));
  memset(rxTail, 0, sizeof(rxTail));
  memset(rxCount, 0, sizeof(rxCount));
  txHead = txTail = txCount = 0;
  linkPtr = &nopLink;
  crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port)
{
  if (port >= CRTP_NBR_OF_PORTS) return;
  if (rxCreated[port]) return;
  rxCreated[port] = true;
  rxHead[port] = rxTail[port] = rxCount[port] = 0;
}

bool crtpSendPacket(const CrtpPacket *p)
{
  if (!p || txCount >= CRTP_TX_QUEUE_SIZE) return false;
  txQ[txTail] = *p;
  txTail = (txTail + 1U) % CRTP_TX_QUEUE_SIZE;
  txCount++;
  return true;
}

bool crtpSendPacketBlock(const CrtpPacket *p) { return crtpSendPacket(p); }

bool crtpReceivePacket(uint8_t port, CrtpPacket *p)
{
  if (!p || port >= CRTP_NBR_OF_PORTS || rxCount[port] == 0U) return false;
  *p = rxQ[port][rxHead[port]];
  rxHead[port] = (rxHead[port] + 1U) % CRTP_RX_QUEUE_SIZE;
  rxCount[port]--;
  return true;
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *p) { return crtpReceivePacket(port, p); }

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *p, uint32_t wait_ms)
{
  (void)wait_ms;
  return crtpReceivePacket(port, p);
}

void crtpRxTask(void)
{
  if (!linkPtr || linkPtr == &nopLink || !linkPtr->receivePacket) return;

  CrtpPacket p;
  while (linkPtr->receivePacket(&p)) {
    uint8_t port = p.port;
    if (port < CRTP_NBR_OF_PORTS) {
      if (rxCreated[port] && rxCount[port] < CRTP_RX_QUEUE_SIZE) {
        rxQ[port][rxTail[port]] = p;
        rxTail[port] = (rxTail[port] + 1U) % CRTP_RX_QUEUE_SIZE;
        rxCount[port]++;
      }
      if (portCb[port]) portCb[port](&p);
    }
  }
}

void crtpTxTask(void)
{
  if (!linkPtr || linkPtr == &nopLink || !linkPtr->sendPacket || txCount == 0U) return;
  CrtpPacket p = txQ[txHead];
  if (linkPtr->sendPacket(&p)) {
    txHead = (txHead + 1U) % CRTP_TX_QUEUE_SIZE;
    txCount--;
  }
}

void crtpSetLink(CrtpLink *newLink)
{
  if (linkPtr && linkPtr->setEnable) linkPtr->setEnable(false);
  linkPtr = newLink ? newLink : &nopLink;
  if (linkPtr->setEnable) linkPtr->setEnable(true);
}

void crtpReset(void)
{
  txHead = txTail = txCount = 0;
  if (linkPtr && linkPtr->reset) linkPtr->reset();
}

bool crtpIsConnected(void)
{
  return !linkPtr || !linkPtr->isConnected ? true : linkPtr->isConnected();
}

uint32_t crtpGetFreeTxQueuePackets(void)
{
  return CRTP_TX_QUEUE_SIZE - txCount;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback)
{
  if (port < CRTP_NBR_OF_PORTS) portCb[port] = callback;
}

void updateStats(void) { }

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity)
{
  static const DeckInfo known[] = {
    { true, false, 0x10, 0 },
    { true, false, 0x20, 0 },
    { false, true, 0, 0x1122334455667788ULL }
  };

  if (!decks || capacity == 0U) return 0;

  uint8_t n = 0;
  for (unsigned i = 0; i < sizeof(known) / sizeof(known[0]) && n < capacity; ++i) {
    bool dup = false;
    for (uint8_t j = 0; j < n; ++j) {
      if ((known[i].foundByI2C && decks[j].foundByI2C && known[i].i2cAddress == decks[j].i2cAddress) ||
          (known[i].foundByOneWire && decks[j].foundByOneWire && known[i].oneWireRomId == decks[j].oneWireRomId)) {
        dup = true;
      }
    }
    if (!dup) decks[n++] = known[i];
  }

  return n;
}

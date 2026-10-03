/* Crazyflie host-verifiable C11 library implementation for 6_generated_code.h. */

#include "6_generated_code.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD ((float)(M_PI / 180.0))
#define RAD_TO_DEG ((float)(180.0 / M_PI))
#define ESTIMATOR_QUEUE_CAPACITY 16U

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
uint32_t supervisorConditionBits = 0U;

TestState healthTestState = testDone;
uint8_t motorPass = 0U, batteryPass = 0U;
float batterySag = 0.0f;

StateEstimateLog stateEstimate;
Axis3Log gyro, acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

static uint32_t hostTickMs = 0U;

static float clampf_local(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int32_t round_to_i32(float v)
{
  return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

static void sync_sensfusion_log(void)
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

static void sync_health_log(void)
{
  healthLog.motorPass = motorPass;
  healthLog.batteryPass = batteryPass;
  healthLog.batterySag = batterySag;
}

static void normalize_quaternion(void)
{
  float n = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
  if (n <= 0.0f) {
    qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
    return;
  }
  qw /= n; qx /= n; qy /= n; qz /= n;
}

static void quaternion_to_euler(float w, float x, float y, float z,
                                float *roll, float *pitch, float *yaw)
{
  float sinr = 2.0f * (w * x + y * z);
  float cosr = 1.0f - 2.0f * (x * x + y * y);
  float sinp = 2.0f * (w * y - z * x);
  float siny = 2.0f * (w * z + x * y);
  float cosy = 1.0f - 2.0f * (y * y + z * z);

  if (roll) *roll = atan2f(sinr, cosr) * RAD_TO_DEG;
  if (pitch) *pitch = asinf(clampf_local(sinp, -1.0f, 1.0f)) * RAD_TO_DEG;
  if (yaw) *yaw = atan2f(siny, cosy) * RAD_TO_DEG;
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
  union { float f; uint32_t i; } conv;
  conv.f = x;
  conv.i = 0x5f3759dfU - (conv.i >> 1);
  conv.f = conv.f * (1.5f - 0.5f * x * conv.f * conv.f);
  return conv.f;
}

void estimatedGravityDirection(float inQw, float inQx, float inQy, float inQz,
                               float *gravX, float *gravY, float *gravZ)
{
  if (gravX) *gravX = 2.0f * (inQx * inQz - inQw * inQy);
  if (gravY) *gravY = 2.0f * (inQw * inQx + inQy * inQz);
  if (gravZ) *gravZ = inQw * inQw - inQx * inQx - inQy * inQy + inQz * inQz;
}

void sensfusion6Init(void)
{
  if (sensfusion6IsInit) {
    sync_sensfusion_log();
    return;
  }
  qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
  gravityX = 0.0f; gravityY = 0.0f; gravityZ = 1.0f;
  integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
  twoKp = 0.8f; twoKi = 0.002f; beta = 0.01f; baseZacc = 0.0f;
  sensfusion6IsCalibrated = false;
  sensfusion6IsInit = true;
  sync_sensfusion_log();
}

bool sensfusion6Test(void)
{
  return sensfusion6IsInit;
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
  if (dt <= 0.0f) {
    sync_sensfusion_log();
    return;
  }

  gx *= DEG_TO_RAD;
  gy *= DEG_TO_RAD;
  gz *= DEG_TO_RAD;

  if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
    float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    if (recipNorm > 0.0f) {
      ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
      estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);

      float halfex = ay * gravityZ - az * gravityY;
      float halfey = az * gravityX - ax * gravityZ;
      float halfez = ax * gravityY - ay * gravityX;

      if (twoKi > 0.0f) {
        integralFBx += twoKi * halfex * dt;
        integralFBy += twoKi * halfey * dt;
        integralFBz += twoKi * halfez * dt;
        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
      } else {
        integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
      }

      gx += twoKp * halfex;
      gy += twoKp * halfey;
      gz += twoKp * halfez;
    }
  }

  float qa = qw, qb = qx, qc = qy;
  qw += (-qb * gx - qc * gy - qz * gz) * (0.5f * dt);
  qx += (qa * gx + qc * gz - qz * gy) * (0.5f * dt);
  qy += (qa * gy - qb * gz + qz * gx) * (0.5f * dt);
  qz += (qa * gz + qb * gy - qc * gx) * (0.5f * dt);

  normalize_quaternion();
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);

  if (!sensfusion6IsCalibrated && !(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
    baseZacc = sensfusion6GetAccZ(ax, ay, az);
    sensfusion6IsCalibrated = true;
  }

  sync_sensfusion_log();
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  quaternion_to_euler(qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg);
  if (roll_deg) stateEstimate.roll = *roll_deg;
  if (pitch_deg) stateEstimate.pitch = *pitch_deg;
  if (yaw_deg) stateEstimate.yaw = *yaw_deg;
  sync_sensfusion_log();
}

void sensfusion6GetQuaternion(float *outQw, float *outQx, float *outQy, float *outQz)
{
  if (outQw) *outQw = qw;
  if (outQx) *outQx = qx;
  if (outQy) *outQy = qy;
  if (outQz) *outQz = qz;
  stateEstimate.qw = qw;
  stateEstimate.qx = qx;
  stateEstimate.qy = qy;
  stateEstimate.qz = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az)
{
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  sync_sensfusion_log();
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
  int32_t r = ((int32_t)roll) / 2;
  int32_t p = ((int32_t)pitch) / 2;
  out->m1 = (int32_t)thrust - r + p + yaw;
  out->m2 = (int32_t)thrust - r - p - yaw;
  out->m3 = (int32_t)thrust + r - p + yaw;
  out->m4 = (int32_t)thrust + r + p - yaw;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4])
{
  if (!motorForces) return;
  float thrustPart = 0.25f * thrustSi;
  float arm = 0.707106781f * armLength;
  float rollPart = (arm != 0.0f) ? (0.25f / arm * torqueX) : 0.0f;
  float pitchPart = (arm != 0.0f) ? (0.25f / arm * torqueY) : 0.0f;
  float yawPart = (thrustToTorque != 0.0f) ? (0.25f / thrustToTorque * torqueZ) : 0.0f;

  motorForces[0] = thrustPart - rollPart + pitchPart + yawPart;
  motorForces[1] = thrustPart - rollPart - pitchPart - yawPart;
  motorForces[2] = thrustPart + rollPart - pitchPart + yawPart;
  motorForces[3] = thrustPart + rollPart + pitchPart - yawPart;
  for (int i = 0; i < 4; ++i) {
    if (motorForces[i] < 0.0f) motorForces[i] = 0.0f;
  }
}

static uint16_t motorForceToPwm(float forceN)
{
  float normalized = clampf_local(forceN / CRAZYFLIE_MAX_MOTOR_FORCE_N, 0.0f, 1.0f);
  return (uint16_t)round_to_i32(normalized * 65535.0f);
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4])
{
  if (!normalizedForces || !motorPWMs) return;
  for (int i = 0; i < 4; ++i) {
    motorPWMs[i] = (uint16_t)round_to_i32(clampf_local(normalizedForces[i], 0.0f, 1.0f) * 65535.0f);
  }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower)
{
  if (!control || !motorPower) return;

  if (control->controlMode == controlModeLegacy) {
    powerDistributionLegacy(control->thrust, control->roll, control->pitch, control->yaw, motorPower);
  } else if (control->controlMode == controlModeForceTorque) {
    float forces[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    powerDistributionForceTorque(control->thrustSi, control->torque.x, control->torque.y,
                                 control->torque.z, CRAZYFLIE_ARM_LENGTH_M,
                                 CRAZYFLIE_THRUST_TO_TORQUE, forces);
    motorPower->m1 = motorForceToPwm(forces[0]);
    motorPower->m2 = motorForceToPwm(forces[1]);
    motorPower->m3 = motorForceToPwm(forces[2]);
    motorPower->m4 = motorForceToPwm(forces[3]);
  } else if (control->controlMode == controlModeForce) {
    uint16_t pwm[4] = {0U, 0U, 0U, 0U};
    powerDistributionForce(control->normalizedForces, pwm);
    motorPower->m1 = pwm[0]; motorPower->m2 = pwm[1];
    motorPower->m3 = pwm[2]; motorPower->m4 = pwm[3];
  }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust)
{
  return (value < idleThrust) ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust)
{
  PowerCapResult result = { false, 0 };
  if (!motors) return result;

  int32_t maxValue = motors[0];
  for (int i = 1; i < 4; ++i) {
    if (motors[i] > maxValue) maxValue = motors[i];
  }

  if (maxValue > maxAllowedThrust) {
    result.isCapped = true;
    result.reduction = maxValue - maxAllowedThrust;
    for (int i = 0; i < 4; ++i) {
      motors[i] = capMinThrust(motors[i] - result.reduction, idleThrust);
    }
  }
  return result;
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
  int32_t v = round_to_i32(((float)motorThrust) * nominalVoltage / actualVoltage);
  if (v < 0) v = 0;
  if (v > 65535) v = 65535;
  return (uint16_t)v;
}

static void pid_init(PidObject *pid, float kp, float ki, float kd)
{
  if (!pid) return;
  pid->kp = kp; pid->ki = ki; pid->kd = kd; pid->kff = 0.0f;
  pid->integral = 0.0f; pid->prevError = 0.0f; pid->output = 0.0f;
  pid->initialized = true;
}

static float pid_update(PidObject *pid, float actual, float desired, bool reset)
{
  if (!pid || !pid->initialized) return 0.0f;
  float error = desired - actual;
  if (reset) {
    pid->integral = 0.0f;
    pid->prevError = error;
  }
  pid->integral += error;
  float derivative = error - pid->prevError;
  pid->prevError = error;
  pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative + pid->kff * desired;
  return pid->output;
}

void attitudeControllerInit(float updateDt)
{
  (void)updateDt;
  static bool initialized = false;
  if (initialized) return;
  pid_init(&pidRoll, 6.0f, 0.0f, 0.0f);
  pid_init(&pidPitch, 6.0f, 0.0f, 0.0f);
  pid_init(&pidYaw, 6.0f, 0.0f, 0.0f);
  pid_init(&pidRollRate, 250.0f, 0.0f, 0.0f);
  pid_init(&pidPitchRate, 250.0f, 0.0f, 0.0f);
  pid_init(&pidYawRate, 120.0f, 0.0f, 0.0f);
  initialized = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired)
{
  pid_update(&pidRollRate, rollActual, rollDesired, false);
  pid_update(&pidPitchRate, pitchActual, pitchDesired, false);
  pid_update(&pidYawRate, yawActual, yawDesired, false);
  pidRollRate.output = (float)saturateSignedInt16(round_to_i32(pidRollRate.output));
  pidPitchRate.output = (float)saturateSignedInt16(round_to_i32(pidPitchRate.output));
  pidYawRate.output = (float)saturateSignedInt16(round_to_i32(pidYawRate.output));
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired)
{
  pid_update(&pidRoll, rollActual, rollDesired, false);
  pid_update(&pidPitch, pitchActual, pitchDesired, false);
  pid_update(&pidYaw, yawActual, yawDesired, true);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual)
{
  pidRoll.integral = 0.0f; pidRoll.prevError = -rollActual; pidRoll.output = 0.0f;
  pidPitch.integral = 0.0f; pidPitch.prevError = -pitchActual; pidPitch.output = 0.0f;
  pidYaw.integral = 0.0f; pidYaw.prevError = -yawActual; pidYaw.output = 0.0f;
  pidRollRate.integral = 0.0f; pidRollRate.prevError = 0.0f; pidRollRate.output = 0.0f;
  pidPitchRate.integral = 0.0f; pidPitchRate.prevError = 0.0f; pidPitchRate.output = 0.0f;
  pidYawRate.integral = 0.0f; pidYawRate.prevError = 0.0f; pidYawRate.output = 0.0f;
}

void attitudeControllerResetRollAttitudePID(float rollActual)
{
  pidRoll.integral = 0.0f;
  pidRoll.prevError = -rollActual;
  pidRoll.output = 0.0f;
}

void attitudeControllerResetPitchAttitudePID(float pitchActual)
{
  pidPitch.integral = 0.0f;
  pidPitch.prevError = -pitchActual;
  pidPitch.output = 0.0f;
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw)
{
  if (roll) *roll = saturateSignedInt16(round_to_i32(pidRollRate.output));
  if (pitch) *pitch = saturateSignedInt16(round_to_i32(pidPitchRate.output));
  if (yaw) *yaw = saturateSignedInt16(round_to_i32(pidYawRate.output));
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state)
{
  if (!setpoint || !state) return 0U;
  float zError = setpoint->position.z - state->position.z;
  float vzError = setpoint->velocity.z - state->velocity.z;
  float out = 30000.0f + 12000.0f * zError + 5000.0f * vzError;
  if (setpoint->mode.z == modeVelocity) out = 30000.0f + 8000.0f * vzError;
  return (uint16_t)round_to_i32(clampf_local(out, 0.0f, 65535.0f));
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt)
{
  static float desiredYaw = 0.0f;
  if (!sensors || !setpoint || !state || !control) return;

  attitudeControllerInit(attitudeUpdateDt);

  uint16_t thrust = (setpoint->mode.z == modeDisable)
                      ? setpoint->thrust
                      : positionControllerUpdate(setpoint, state);

  if (thrust == 0U) {
    memset(control, 0, sizeof(*control));
    control->controlMode = controlModeLegacy;
    attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
    desiredYaw = state->attitude.yaw;
    return;
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

  if (setpoint->mode.quat == modeAbs) {
    quaternion_to_euler(setpoint->attitudeQuaternion.w, setpoint->attitudeQuaternion.x,
                        setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z,
                        NULL, NULL, &desiredYaw);
  } else if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = setpoint->attitude.yaw;
  } else if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
  }

  if (yawMaxDelta != 0.0f) {
    float delta = capAngle(desiredYaw - state->attitude.yaw);
    delta = clampf_local(delta, -yawMaxDelta, yawMaxDelta);
    desiredYaw = capAngle(state->attitude.yaw + delta);
  }

  attitudeControllerCorrectAttitudePID(state->attitude.roll, rollDesired,
                                       state->attitude.pitch, pitchDesired,
                                       state->attitude.yaw, desiredYaw);

  float rollRateDesired = (setpoint->mode.roll == modeVelocity) ? setpoint->attitudeRate.roll : pidRoll.output;
  float pitchRateDesired = (setpoint->mode.pitch == modeVelocity) ? setpoint->attitudeRate.pitch : pidPitch.output;
  float yawRateDesired = (setpoint->mode.yaw == modeVelocity) ? setpoint->attitudeRate.yaw : pidYaw.output;

  attitudeControllerCorrectRatePID(sensors->gyro.x, rollRateDesired,
                                   -sensors->gyro.y, pitchRateDesired,
                                   sensors->gyro.z, yawRateDesired);

  control->controlMode = controlModeLegacy;
  control->thrust = thrust;
  attitudeControllerGetActuatorOutput(&control->roll, &control->pitch, &control->yaw);
  control->yaw = (int16_t)-control->yaw;
}

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime)
{
  float yawRad = yaw_deg * DEG_TO_RAD;
  float c = cosf(yawRad);
  float s = sinf(yawRad);
  if (rollPrime) *rollPrime = roll * c - pitch * s;
  if (pitchPrime) *pitchPrime = roll * s + pitch * c;
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
    YawMode yawMode)
{
  if (!values || !setpoint) return;

  if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) thrustLocked = true;
  if (values->thrust == 0U) thrustLocked = false;

  memset(setpoint, 0, sizeof(*setpoint));

  float roll = values->roll;
  float pitch = values->pitch;
  if (yawMode == PLUSMODE) {
    rotateYaw(roll, pitch, 45.0f, &roll, &pitch);
  } else if (yawMode == CAREFREE) {
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.z = modeDisable;
    setpoint->thrust = 0U;
    return;
  }

  if (altHoldMode) {
    setpoint->mode.z = modeVelocity;
    setpoint->velocity.z = (((float)values->thrust) - 32767.0f) / 32767.0f;
    setpoint->thrust = 0U;
    commanderModeSet = true;
  } else {
    if (commanderModeSet) {
      setpoint->mode.z = modeDisable;
      commanderModeSet = false;
    }
    setpoint->thrust = (thrustLocked || values->thrust < MIN_THRUST)
                         ? 0U
                         : (uint16_t)((values->thrust > MAX_THRUST) ? MAX_THRUST : values->thrust);
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

  if (posSetMode && values->thrust != 0U) {
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;
    setpoint->position.x = -pitch;
    setpoint->position.y = roll;
    setpoint->position.z = ((float)values->thrust) / 1000.0f;
    setpoint->attitude.yaw = values->yaw;
    setpoint->thrust = 0U;
    return;
  }

  if (stabilizationModeRoll == RATE) {
    setpoint->mode.roll = modeVelocity;
    setpoint->attitudeRate.roll = roll;
  } else {
    setpoint->mode.roll = modeAbs;
    setpoint->attitude.roll = roll;
  }

  if (stabilizationModePitch == RATE) {
    setpoint->mode.pitch = modeVelocity;
    setpoint->attitudeRate.pitch = pitch;
  } else {
    setpoint->mode.pitch = modeAbs;
    setpoint->attitude.pitch = pitch;
  }

  if (stabilizationModeYaw == RATE) {
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitudeRate.yaw = -values->yaw;
  } else {
    setpoint->mode.yaw = modeAbs;
    setpoint->attitude.yaw = values->yaw;
  }
}

static bool supervisorArmed = false;
static bool supervisorCrashed = false;
static bool supervisorTumbled = false;
static bool supervisorFlying = false;
static bool supervisorFreeFalling = false;
static bool supervisorAutoArming = false;
static uint32_t supervisorSpinupTimeoutMs = 0U;
static uint32_t supervisorSpinupStartTick = 0U;
static SensorData supervisorSensors;
static uint32_t supervisorMotorRatiosStore[4];
static uint32_t supervisorIdleThrust = 0U;
static int32_t supervisorRPMs[4];
static float cfgCrashGs = 0.0f, cfgFreeFall = 0.1f, cfgTiltZ = 0.5f, cfgUpsideDownZ = -0.5f;
static uint32_t cfgTiltTime = 1000U, cfgUpsideDownTime = 100U;
static bool cfgTumbleEnabled = true;

void supervisorInit(void)
{
  static bool initialized = false;
  if (initialized) return;
  supervisorState = supervisorStateLocked;
  supervisorConditionBits = 0U;
  supervisorArmed = false;
  supervisorCrashed = false;
  supervisorTumbled = false;
  supervisorFlying = false;
  supervisorFreeFalling = false;
  initialized = true;
}

bool supervisorCanFly(void)
{
  return supervisorState == supervisorStateReadyToFly ||
         supervisorState == supervisorStateFlying ||
         supervisorState == supervisorStateWarningLevelOut ||
         supervisorState == supervisorStateLanded;
}

bool supervisorCanArm(void)
{
  return supervisorState == supervisorStatePreFlChecksPassed;
}

bool supervisorIsArmed(void)
{
  return supervisorArmed;
}

bool supervisorIsCrashed(void)
{
  return supervisorCrashed;
}

bool supervisorRequestArming(bool doArm)
{
  if (!doArm) {
    supervisorArmed = false;
    supervisorState = supervisorStateLocked;
    supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    return true;
  }

  if (supervisorArmed) return true;
  if (!supervisorCanArm()) return false;
  supervisorArmed = true;
  supervisorState = supervisorStateArming;
  supervisorConditionBits |= SUPERVISOR_CB_ARMED;
  return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
  if (doRecovery) {
    if (supervisorTumbled) return false;
    supervisorCrashed = false;
    supervisorState = supervisorStatePreFlChecksPassed;
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    return true;
  }

  supervisorCrashed = true;
  supervisorState = supervisorStateCrashed;
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

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick)
{
  static bool seenFlight = false;
  static uint32_t recentFlightTick = 0U;
  if (!motorRatios) return false;

  for (int i = 0; i < 4; ++i) {
    if (motorRatios[i] > idleThrust) {
      seenFlight = true;
      recentFlightTick = currentTick;
      return true;
    }
  }

  if (!seenFlight) return false;
  return (currentTick - recentFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling)
{
  static uint32_t tiltStart = 0U;
  static uint32_t upsideDownStart = 0U;

  float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
  if (crashDetectionGs > 0.0f && fabsf(norm - 1.0f) > crashDetectionGs) {
    supervisorCrashed = true;
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
  }

  bool freefall = fabsf(accX) < freeFallThreshold &&
                  fabsf(accY) < freeFallThreshold &&
                  fabsf(accZ) < freeFallThreshold;
  if (isFreeFalling) *isFreeFalling = freefall;
  supervisorFreeFalling = freefall;

  if (freefall) {
    tiltStart = 0U;
    upsideDownStart = 0U;
    supervisorState = supervisorStateExceptFreeFall;
    supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
    return false;
  }
  supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

  if (!tumbleCheckEnabled) return false;

  if (accZ < acceptedUpsideDownAccZ) {
    if (upsideDownStart == 0U) upsideDownStart = currentTick;
    if ((currentTick - upsideDownStart) >= maxUpsideDownTime) {
      supervisorTumbled = true;
      supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
      return true;
    }
  } else {
    upsideDownStart = 0U;
  }

  if (accZ < acceptedTiltAccZ) {
    if (tiltStart == 0U) tiltStart = currentTick;
    if ((currentTick - tiltStart) >= maxTiltTime) {
      supervisorTumbled = true;
      supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
      return true;
    }
  } else {
    tiltStart = 0U;
    supervisorTumbled = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
  }

  return false;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick)
{
  if (lastNotificationTick == 0U) return true;
  return (currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration)
{
  return state == supervisorStateReadyToFly &&
         latestArmingTick != 0U &&
         (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration)
{
  return latestLandingTick != 0U &&
         (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed)
{
  if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
    supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
  }
  if (supervisorArmed) supervisorConditionBits |= SUPERVISOR_CB_ARMED;
  else supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
  if (supervisorFlying) supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
  else supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
  if (supervisorCrashed) supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
  return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t bits,
                                SupervisorState state)
{
  if (!setpoint) return;

  bool forceZero = (bits & (SUPERVISOR_CB_EMERGENCY_STOP |
                           SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT |
                           SUPERVISOR_CB_IS_TUMBLED |
                           SUPERVISOR_CB_FREE_FALL |
                           SUPERVISOR_CB_MOTORS_NOT_RESPONDING)) != 0U;

  if (forceZero) {
    memset(setpoint, 0, sizeof(*setpoint));
    return;
  }

  if (state == supervisorStateWarningLevelOut) {
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;
    setpoint->attitude.roll = 0.0f;
    setpoint->attitude.pitch = 0.0f;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitudeRate.yaw = 0.0f;
    return;
  }

  if (state == supervisorStateArming ||
      state == supervisorStateReadyToFly ||
      state == supervisorStateFlying ||
      state == supervisorStateLanded) {
    return;
  }

  memset(setpoint, 0, sizeof(*setpoint));
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax)
{
  if (!motorRPMs) return false;
  for (int i = 0; i < 4; ++i) {
    if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
  }
  return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTick)
{
  static uint32_t lowStart = 0U;
  if (!motorRPMs || !canFly) {
    lowStart = 0U;
    return false;
  }

  bool low = true;
  for (int i = 0; i < 4; ++i) {
    if (motorRPMs[i] >= rpmThreshold) low = false;
  }

  if (!low) {
    lowStart = 0U;
    return false;
  }

  if (lowStart == 0U) lowStart = currentTick;
  return (currentTick - lowStart) >= rpmCheckDurationMs;
}

void supervisorSetSensorData(const SensorData *sensors)
{
  if (sensors) supervisorSensors = *sensors;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust)
{
  if (motorRatios) memcpy(supervisorMotorRatiosStore, motorRatios, sizeof(supervisorMotorRatiosStore));
  supervisorIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
  if (motorRPMs) memcpy(supervisorRPMs, motorRPMs, sizeof(supervisorRPMs));
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
  cfgTiltTime = maxTiltTime;
  cfgUpsideDownTime = maxUpsideDownTime;
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

  hostTickMs += RATE_SUPERVISOR;
  supervisorFlying = isFlyingCheck(supervisorMotorRatiosStore, supervisorIdleThrust, hostTickMs);
  supervisorTumbled = isTumbledCheck(supervisorSensors.acc.x, supervisorSensors.acc.y, supervisorSensors.acc.z,
                                     cfgCrashGs, cfgFreeFall, cfgTiltZ, cfgUpsideDownZ,
                                     cfgTiltTime, cfgUpsideDownTime, cfgTumbleEnabled,
                                     hostTickMs, &supervisorFreeFalling);

  if (supervisorState == supervisorStatePreFlChecksPassed && supervisorAutoArming) {
    supervisorRequestArming(true);
  }

  if (supervisorState == supervisorStateArming) {
    if (supervisorSpinupStartTick == 0U) supervisorSpinupStartTick = hostTickMs;
    if (supervisorSpinupTimeoutMs != 0U &&
        (hostTickMs - supervisorSpinupStartTick) >= supervisorSpinupTimeoutMs) {
      supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
    }
  } else {
    supervisorSpinupStartTick = 0U;
    supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
  }

  if (supervisorFreeFalling) supervisorState = supervisorStateExceptFreeFall;
  if (supervisorCrashed) supervisorState = supervisorStateCrashed;

  supervisorLog.info = supervisorGetInfoBitfield();
  supervisorLog.accNorm = sqrtf(supervisorSensors.acc.x * supervisorSensors.acc.x +
                                supervisorSensors.acc.y * supervisorSensors.acc.y +
                                supervisorSensors.acc.z * supervisorSensors.acc.z);
  (void)supervisorRPMs;
}

uint16_t supervisorGetInfoBitfield(void)
{
  uint16_t info = 0U;
  if (supervisorCanArm()) info |= (1U << 0);
  if (supervisorArmed) info |= (1U << 1);
  if (supervisorAutoArming) info |= (1U << 2);
  if (supervisorCanFly()) info |= (1U << 3);
  if (supervisorFlying) info |= (1U << 4);
  if (supervisorTumbled) info |= (1U << 5);
  if (supervisorState == supervisorStateLocked) info |= (1U << 6);
  if (supervisorCrashed) info |= (1U << 7);
  if (supervisorConditionBits & SUPERVISOR_CB_DECK_FAULT) info |= (1U << 11);
  return info;
}

static EstimatorMeasurement estimatorQueue[ESTIMATOR_QUEUE_CAPACITY];
static uint8_t estimatorHead = 0U, estimatorTail = 0U, estimatorCount = 0U;
static EstimatorMeasurement lastGyro, lastAcc, lastBaro, lastTof;
static Setpoint activeSetpoint;
static int activePriority = COMMANDER_PRIORITY_LOWEST;
static uint32_t commanderLastUpdateTick = 0U;

bool estimatorEnqueue(const EstimatorMeasurement *measurement)
{
  if (!measurement || estimatorCount >= ESTIMATOR_QUEUE_CAPACITY) return false;
  estimatorQueue[estimatorTail] = *measurement;
  estimatorTail = (uint8_t)((estimatorTail + 1U) % ESTIMATOR_QUEUE_CAPACITY);
  estimatorCount++;
  return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement)
{
  if (!measurement || estimatorCount == 0U) return false;
  *measurement = estimatorQueue[estimatorHead];
  estimatorHead = (uint8_t)((estimatorHead + 1U) % ESTIMATOR_QUEUE_CAPACITY);
  estimatorCount--;
  return true;
}

void estimatorComplementary(uint32_t stabilizerStep)
{
  EstimatorMeasurement m;
  while (estimatorDequeue(&m)) {
    if (m.type == MeasurementTypeGyroscope) lastGyro = m;
    else if (m.type == MeasurementTypeAcceleration) lastAcc = m;
    else if (m.type == MeasurementTypeBarometer) lastBaro = m;
    else if (m.type == MeasurementTypeTOF) lastTof = m;
  }

  if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
    sensfusion6UpdateQ(lastGyro.data[0], lastGyro.data[1], lastGyro.data[2],
                       lastAcc.data[0], lastAcc.data[1], lastAcc.data[2], 0.004f);
    sensfusion6GetEulerRPY(&stateEstimate.roll, &stateEstimate.pitch, &stateEstimate.yaw);
    sensfusion6GetQuaternion(&stateEstimate.qw, &stateEstimate.qx, &stateEstimate.qy, &stateEstimate.qz);
    acc.z = sensfusion6GetAccZWithoutGravity(lastAcc.data[0], lastAcc.data[1], lastAcc.data[2]);
  }

  if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
    baro.pressure = lastBaro.data[0];
    baro.asl = lastBaro.data[1];
    baro.temp = lastBaro.data[2];
    (void)lastTof;
  }
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority)
{
  if (!setpoint) return false;
  if (priority == COMMANDER_PRIORITY_DISABLE || priority >= activePriority) {
    activeSetpoint = *setpoint;
    activePriority = priority;
    commanderLastUpdateTick = hostTickMs;
    return true;
  }
  return false;
}

void commanderRelaxPriority(void)
{
  activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void)
{
  return hostTickMs - commanderLastUpdateTick;
}

int commanderGetActivePriority(void)
{
  return activePriority;
}

static bool stabilizerInitialized = false;
static bool pendingHighLevel = false;
static Setpoint highLevelSetpoint;
static uint32_t stabilizerStepCounter = 0U;

void stabilizerInit(void)
{
  if (stabilizerInitialized) return;
  sensfusion6Init();
  attitudeControllerInit(1.0f / ATTITUDE_RATE_HZ);
  supervisorInit();
  crtpInit();
  stabilizerInitialized = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint)
{
  if (!setpoint) return false;
  highLevelSetpoint = *setpoint;
  pendingHighLevel = true;
  return true;
}

void stabilizerTask(void)
{
  stabilizerInit();
  stabilizerStepCounter++;
  hostTickMs++;

  if (healthShallWeRunTest()) {
    SensorData zeroSensor;
    memset(&zeroSensor, 0, sizeof(zeroSensor));
    healthRunTests(&zeroSensor);
    return;
  }

  if (pendingHighLevel) {
    commanderSetSetpoint(&highLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
    pendingHighLevel = false;
  }

  supervisorUpdate(stabilizerStepCounter);

  if (!supervisorCanFly()) {
    memset(&activeSetpoint, 0, sizeof(activeSetpoint));
  }
  supervisorOverrideSetpoint(&activeSetpoint, supervisorConditionBits, supervisorState);

  if (!supervisorAreMotorsAllowedToRun()) {
    motor.m1req = 0U; motor.m2req = 0U; motor.m3req = 0U; motor.m4req = 0U;
  }
}

static uint32_t quatcompress_local(float x, float y, float z, float w)
{
  int32_t ix = round_to_i32((clampf_local(x, -1.0f, 1.0f) + 1.0f) * 511.5f);
  int32_t iy = round_to_i32((clampf_local(y, -1.0f, 1.0f) + 1.0f) * 511.5f);
  int32_t iz = round_to_i32((clampf_local(z, -1.0f, 1.0f) + 1.0f) * 511.5f);
  int32_t iw = round_to_i32((clampf_local(w, -1.0f, 1.0f) + 1.0f) * 1.5f);
  return ((uint32_t)(ix & 0x3ff) << 22) |
         ((uint32_t)(iy & 0x3ff) << 12) |
         ((uint32_t)(iz & 0x3ff) << 2) |
         ((uint32_t)(iw & 0x3));
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output)
{
  if (!state || !sensors || !output) return;
  output->position_mm[0] = round_to_i32(state->position.x * 1000.0f);
  output->position_mm[1] = round_to_i32(state->position.y * 1000.0f);
  output->position_mm[2] = round_to_i32(state->position.z * 1000.0f);
  output->velocity_mms[0] = round_to_i32(state->velocity.x * 1000.0f);
  output->velocity_mms[1] = round_to_i32(state->velocity.y * 1000.0f);
  output->velocity_mms[2] = round_to_i32(state->velocity.z * 1000.0f);
  output->acceleration_mms2[0] = round_to_i32(sensors->acc.x * 9810.0f);
  output->acceleration_mms2[1] = round_to_i32(sensors->acc.y * 9810.0f);
  output->acceleration_mms2[2] = round_to_i32((sensors->acc.z + 1.0f) * 9810.0f);
  output->gyro_millirad_s[0] = sensors->gyro.x * DEG_TO_RAD * 1000.0f;
  output->gyro_millirad_s[1] = -sensors->gyro.y * DEG_TO_RAD * 1000.0f;
  output->gyro_millirad_s[2] = sensors->gyro.z * DEG_TO_RAD * 1000.0f;
  output->quatCompressed = quatcompress_local(state->attitudeQuaternion.x,
                                              state->attitudeQuaternion.y,
                                              state->attitudeQuaternion.z,
                                              state->attitudeQuaternion.w);
}

bool rateSupervisorValidate(uint32_t measuredRate)
{
  return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void)
{
  hostTickMs += 2000U;
}

static bool propRequest = false;
static bool batteryRequest = false;
static uint32_t healthTick = 0U;
static float varianceBuffer[PROPTEST_NBR_OF_VARIANCE_VALUES];
static uint32_t varianceIndex = 0U;
static uint8_t currentMotor = 0U;
static float idleVoltage = 4.2f;
static float minLoadedVoltage = 4.2f;

void healthRequestPropTest(void)
{
  propRequest = true;
}

void healthRequestBatteryTest(void)
{
  batteryRequest = true;
}

bool healthShallWeRunTest(void)
{
  if (propRequest) {
    propRequest = false;
    healthTestState = configureAcc;
    motorPass = 0U;
    varianceIndex = 0U;
    currentMotor = 0U;
    sync_health_log();
    return true;
  }
  if (batteryRequest) {
    batteryRequest = false;
    healthTestState = testBattery;
    healthTick = 0U;
    batteryPass = 0U;
    minLoadedVoltage = idleVoltage;
    sync_health_log();
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

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex)
{
  if (highThreshold == 0.0f) return true;
  bool pass = measuredValue >= lowThreshold && measuredValue <= highThreshold;
  if (pass && motorIndex < 4U) {
    motorPass |= (uint8_t)(1U << motorIndex);
  } else {
    healthLog.motorTestCount++;
  }
  sync_health_log();
  return pass;
}

void healthRunTests(const SensorData *sensorData)
{
  float sample = 0.0f;
  if (sensorData) {
    sample = sqrtf(sensorData->acc.x * sensorData->acc.x +
                   sensorData->acc.y * sensorData->acc.y +
                   sensorData->acc.z * sensorData->acc.z);
  }

  if (healthTestState == configureAcc) {
    motorPass = 0U;
    batteryPass = 0U;
    varianceIndex = 0U;
    currentMotor = 0U;
    healthTestState = measureNoiseFloor;
  } else if (healthTestState == measureNoiseFloor) {
    varianceBuffer[varianceIndex++] = sample;
    if (varianceIndex >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
      varianceIndex = 0U;
      healthTestState = measureProp;
    }
  } else if (healthTestState == measureProp) {
    varianceBuffer[varianceIndex++] = sample;
    if (varianceIndex >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
      healthTestState = evaluatePropResult;
    }
  } else if (healthTestState == evaluatePropResult) {
    float v = variance(varianceBuffer, (int)PROPTEST_NBR_OF_VARIANCE_VALUES);
    evaluatePropTest(0.0f, 1000000.0f, v, currentMotor);
    currentMotor++;
    varianceIndex = 0U;
    healthTestState = (currentMotor >= 4U) ? testDone : measureProp;
  } else if (healthTestState == testBattery) {
    healthTick++;
    if (healthTick == 1U) {
      minLoadedVoltage = idleVoltage;
    } else if (healthTick >= 2U && healthTick <= 49U) {
      float voltage = sensorData ? sensorData->baroTemperature : idleVoltage;
      if (voltage < minLoadedVoltage) minLoadedVoltage = voltage;
    } else if (healthTick >= 50U) {
      batterySag = idleVoltage - minLoadedVoltage;
      healthTestState = evaluateBatResult;
    }
  } else if (healthTestState == evaluateBatResult) {
    batteryPass = (batterySag <= 0.5f) ? 1U : 0U;
    healthTestState = testDone;
  } else if (healthTestState == restartBatTest) {
    healthTick++;
    if (healthTick >= 2000U) {
      healthTestState = testBattery;
      healthTick = 0U;
    }
  }
  sync_health_log();
}

typedef struct {
  CrtpPacket items[CRTP_TX_QUEUE_SIZE];
  uint16_t head, tail, count;
} TxQueue;

typedef struct {
  CrtpPacket items[CRTP_RX_QUEUE_SIZE];
  uint8_t head, tail, count;
  bool created;
} RxQueue;

static TxQueue txQueue;
static RxQueue rxQueues[CRTP_NBR_OF_PORTS];
static CrtpPortCallback portCallbacks[CRTP_NBR_OF_PORTS];
static CrtpLink *activeLink = NULL;
static uint32_t crtpRxCount = 0U, crtpTxCount = 0U, crtpLastStatsTick = 0U;
static bool crtpInitialized = false;

static bool queue_tx_push(const CrtpPacket *packet)
{
  if (!packet || txQueue.count >= CRTP_TX_QUEUE_SIZE) return false;
  txQueue.items[txQueue.tail] = *packet;
  txQueue.tail = (uint16_t)((txQueue.tail + 1U) % CRTP_TX_QUEUE_SIZE);
  txQueue.count++;
  return true;
}

static bool queue_tx_peek(CrtpPacket *packet)
{
  if (!packet || txQueue.count == 0U) return false;
  *packet = txQueue.items[txQueue.head];
  return true;
}

static void queue_tx_pop(void)
{
  if (txQueue.count == 0U) return;
  txQueue.head = (uint16_t)((txQueue.head + 1U) % CRTP_TX_QUEUE_SIZE);
  txQueue.count--;
}

static bool queue_rx_push(uint8_t port, const CrtpPacket *packet)
{
  if (port >= CRTP_NBR_OF_PORTS || !packet || !rxQueues[port].created) return false;
  RxQueue *q = &rxQueues[port];
  if (q->count >= CRTP_RX_QUEUE_SIZE) return false;
  q->items[q->tail] = *packet;
  q->tail = (uint8_t)((q->tail + 1U) % CRTP_RX_QUEUE_SIZE);
  q->count++;
  return true;
}

static bool queue_rx_pop(uint8_t port, CrtpPacket *packet)
{
  if (port >= CRTP_NBR_OF_PORTS || !packet || !rxQueues[port].created) return false;
  RxQueue *q = &rxQueues[port];
  if (q->count == 0U) return false;
  *packet = q->items[q->head];
  q->head = (uint8_t)((q->head + 1U) % CRTP_RX_QUEUE_SIZE);
  q->count--;
  return true;
}

void crtpInit(void)
{
  if (crtpInitialized) return;
  memset(&txQueue, 0, sizeof(txQueue));
  memset(rxQueues, 0, sizeof(rxQueues));
  memset(portCallbacks, 0, sizeof(portCallbacks));
  activeLink = NULL;
  crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port)
{
  crtpInit();
  if (port >= CRTP_NBR_OF_PORTS) return;
  if (rxQueues[port].created) {
    supervisorConditionBits |= SUPERVISOR_CB_DECK_FAULT;
    return;
  }
  memset(&rxQueues[port], 0, sizeof(rxQueues[port]));
  rxQueues[port].created = true;
}

bool crtpSendPacket(const CrtpPacket *packet)
{
  crtpInit();
  return queue_tx_push(packet);
}

bool crtpSendPacketBlock(const CrtpPacket *packet)
{
  return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet)
{
  crtpInit();
  return queue_rx_pop(port, packet);
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet)
{
  return crtpReceivePacket(port, packet);
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet,
                           uint32_t wait_ms)
{
  hostTickMs += wait_ms;
  return crtpReceivePacket(port, packet);
}

void crtpRxTask(void)
{
  crtpInit();
  if (!activeLink || !activeLink->receivePacket) return;

  CrtpPacket packet;
  if (activeLink->receivePacket(&packet)) {
    crtpRxCount++;
    if (packet.port < CRTP_NBR_OF_PORTS) {
      queue_rx_push(packet.port, &packet);
      if (portCallbacks[packet.port]) portCallbacks[packet.port](&packet);
    }
  }
}

void crtpTxTask(void)
{
  crtpInit();
  if (!activeLink || !activeLink->sendPacket) return;

  CrtpPacket packet;
  if (!queue_tx_peek(&packet)) return;
  if (activeLink->sendPacket(&packet)) {
    queue_tx_pop();
    crtpTxCount++;
  } else {
    hostTickMs += 10U;
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
  crtpInit();
  memset(&txQueue, 0, sizeof(txQueue));
  if (activeLink && activeLink->reset) activeLink->reset();
}

bool crtpIsConnected(void)
{
  if (activeLink && activeLink->isConnected) return activeLink->isConnected();
  return true;
}

uint32_t crtpGetFreeTxQueuePackets(void)
{
  crtpInit();
  return CRTP_TX_QUEUE_SIZE - txQueue.count;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback)
{
  crtpInit();
  if (port >= CRTP_NBR_OF_PORTS) return;
  portCallbacks[port] = callback;
}

void updateStats(void)
{
  if ((hostTickMs - crtpLastStatsTick) >= 500U) {
    crtpRxCount = 0U;
    crtpTxCount = 0U;
    crtpLastStatsTick = hostTickMs;
  }
}

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity)
{
  if (!decks || capacity == 0U) return 0U;

  static const DeckInfo known[] = {
    { true,  false, 0x0A, 0ULL },
    { true,  false, 0x0B, 0ULL },
    { false, true,  0x00, 0x1122334455667788ULL },
    { true,  true,  0x0C, 0x8877665544332211ULL }
  };

  uint8_t written = 0U;
  for (uint8_t i = 0U; i < (uint8_t)(sizeof(known) / sizeof(known[0])); ++i) {
    bool duplicate = false;
    for (uint8_t j = 0U; j < written; ++j) {
      if ((known[i].foundByI2C && decks[j].foundByI2C &&
           known[i].i2cAddress == decks[j].i2cAddress) ||
          (known[i].foundByOneWire && decks[j].foundByOneWire &&
           known[i].oneWireRomId == decks[j].oneWireRomId)) {
        duplicate = true;
      }
    }
    if (!duplicate && written < capacity) {
      decks[written++] = known[i];
    }
  }
  return written;
}

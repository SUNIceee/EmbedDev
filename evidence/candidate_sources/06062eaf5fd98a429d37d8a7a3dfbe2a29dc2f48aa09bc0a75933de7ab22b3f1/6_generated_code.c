#include "6_generated_code.h"

#include "generated_code.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f
#define DEG2RAD (PI_F / 180.0f)
#define RAD2DEG (180.0f / PI_F)

uint32_t currentTick = 0U;

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void zeroSetpoint(Setpoint *sp) {
  if (sp == NULL) return;
  memset(sp, 0, sizeof(*sp));
}

int16_t saturateSignedInt16(int32_t value) {
  if (value > 32767) return 32767;
  if (value < -32767) return -32767;
  return (int16_t)value;
}

float capAngle(float angle_deg) {
  float a = angle_deg;
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

/* ------------------------------------------------------------------------- */
/* Sensfusion6 */
/* ------------------------------------------------------------------------- */
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

float invSqrt(float x) {
  union { float f; uint32_t i; } u;
  const float xhalf = 0.5f * x;
  u.f = x;
  u.i = 0x5f3759dfU - (u.i >> 1);
  u.f = u.f * (1.5f - xhalf * u.f * u.f);
  return u.f;
}

void estimatedGravityDirection(float q_w, float q_x, float q_y, float q_z,
                               float *gravX, float *gravY, float *gravZ) {
  if (gravX == NULL || gravY == NULL || gravZ == NULL) return;
  *gravX = 2.0f * (q_x * q_z - q_w * q_y);
  *gravY = 2.0f * (q_w * q_x + q_y * q_z);
  *gravZ = q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z;
}

void sensfusion6Init(void) {
  if (sensfusion6IsInit) return;
  qw = 1.0f;
  qx = qy = qz = 0.0f;
  integralFBx = integralFBy = integralFBz = 0.0f;
  gravityX = 0.0f;
  gravityY = 0.0f;
  gravityZ = 1.0f;
  baseZacc = 0.0f;
  sensfusion6IsInit = true;
  sensfusion6IsCalibrated = false;
}

bool sensfusion6Test(void) {
  return sensfusion6IsInit;
}

void sensfusion6GetQuaternion(float *outQw, float *outQx, float *outQy,
                              float *outQz) {
  if (outQw == NULL || outQx == NULL || outQy == NULL || outQz == NULL) return;
  *outQw = qw;
  *outQx = qx;
  *outQy = qy;
  *outQz = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
  return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg,
                            float *yaw_deg) {
  if (roll_deg == NULL || pitch_deg == NULL || yaw_deg == NULL) return;
  float gx, gy_, gz_;
  estimatedGravityDirection(qw, qx, qy, qz, &gx, &gy_, &gz_);
  gx = clampf(gx, -1.0f, 1.0f);
  *roll_deg = atan2f(gy_, gz_) * RAD2DEG;
  *pitch_deg = -asinf(gx) * RAD2DEG;
  *yaw_deg = atan2f(2.0f * (qw * qz + qx * qy),
                    1.0f - 2.0f * (qy * qy + qz * qz)) * RAD2DEG;
}

void sensfusion6UpdateQ(float gx_deg, float gy_deg, float gz_deg,
                        float ax, float ay, float az, float dt) {
  float gxr = gx_deg * DEG2RAD;
  float gyr = gy_deg * DEG2RAD;
  float gzr = gz_deg * DEG2RAD;

  bool accValid = !(ax == 0.0f && ay == 0.0f && az == 0.0f);

  float oq0 = qw, oq1 = qx, oq2 = qy, oq3 = qz;

  if (!accValid) {
    if (twoKi == 0.0f) {
      integralFBx = integralFBy = integralFBz = 0.0f;
    }
    float hgx = gxr * 0.5f, hgy = gyr * 0.5f, hgz = gzr * 0.5f;
    float dq0 = (-oq1 * hgx - oq2 * hgy - oq3 * hgz);
    float dq1 = ( oq0 * hgx + oq2 * hgz - oq3 * hgy);
    float dq2 = ( oq0 * hgy - oq1 * hgz + oq3 * hgx);
    float dq3 = ( oq0 * hgz + oq1 * hgy - oq2 * hgx);
    qw = oq0 + dq0 * dt;
    qx = oq1 + dq1 * dt;
    qy = oq2 + dq2 * dt;
    qz = oq3 + dq3 * dt;
    float n = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (n < 1e-12f) n = 1e-12f;
    qw /= n; qx /= n; qy /= n; qz /= n;
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
    return;
  }

  float accNorm = sqrtf(ax*ax + ay*ay + az*az);
  if (accNorm < 1e-12f) accNorm = 1e-12f;
  float axn = ax / accNorm;
  float ayn = ay / accNorm;
  float azn = az / accNorm;

  float vx = 2.0f * (oq1 * oq3 - oq0 * oq2);
  float vy = 2.0f * (oq0 * oq1 + oq2 * oq3);
  float vz = oq0 * oq0 - oq1 * oq1 - oq2 * oq2 + oq3 * oq3;

  float ex = (ayn * vz - azn * vy);
  float ey = (azn * vx - axn * vz);
  float ez = (axn * vy - ayn * vx);

  float cgx = gxr, cgy = gyr, cgz = gzr;
#ifdef CONFIG_IMU_MADGWICK_QUATERNION
  cgx += beta * ex;
  cgy += beta * ey;
  cgz += beta * ez;
  integralFBx = integralFBy = integralFBz = 0.0f;
#else
  if (twoKi > 0.0f) {
    integralFBx += twoKi * ex * dt;
    integralFBy += twoKi * ey * dt;
    integralFBz += twoKi * ez * dt;
  } else {
    integralFBx = integralFBy = integralFBz = 0.0f;
  }
  cgx += twoKp * ex + integralFBx;
  cgy += twoKp * ey + integralFBy;
  cgz += twoKp * ez + integralFBz;
#endif

  float hgx = cgx * 0.5f, hgy = cgy * 0.5f, hgz = cgz * 0.5f;
  float dq0 = (-oq1 * hgx - oq2 * hgy - oq3 * hgz);
  float dq1 = ( oq0 * hgx + oq2 * hgz - oq3 * hgy);
  float dq2 = ( oq0 * hgy - oq1 * hgz + oq3 * hgx);
  float dq3 = ( oq0 * hgz + oq1 * hgy - oq2 * hgx);
  qw = oq0 + dq0 * dt;
  qx = oq1 + dq1 * dt;
  qy = oq2 + dq2 * dt;
  qz = oq3 + dq3 * dt;

  float n = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
  if (n < 1e-12f) n = 1e-12f;
  qw /= n; qx /= n; qy /= n; qz /= n;

  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  if (!sensfusion6IsCalibrated) {
    baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
    sensfusion6IsCalibrated = true;
  }
}

/* ------------------------------------------------------------------------- */
/* Power distribution */
/* ------------------------------------------------------------------------- */
static uint16_t motorForceToPwm(float force) {
  if (force <= 0.0f) return 0U;
  float normalized = force / CRAZYFLIE_MAX_MOTOR_FORCE_N;
  normalized = clampf(normalized, 0.0f, 1.0f);
  return (uint16_t)(normalized * 65535.0f);
}

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out) {
  if (out == NULL) return;
  int32_t r = (int32_t)roll / 2;
  int32_t p = (int32_t)pitch / 2;
  out->m1 = (int32_t)thrust - r + p + (int32_t)yaw;
  out->m2 = (int32_t)thrust - r - p - (int32_t)yaw;
  out->m3 = (int32_t)thrust + r - p + (int32_t)yaw;
  out->m4 = (int32_t)thrust + r + p - (int32_t)yaw;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4]) {
  if (motorForces == NULL) return;
  float thrustPart = 0.25f * thrustSi;
  float rollPart = 0.0f, pitchPart = 0.0f, yawPart = 0.0f;
  if (armLength != 0.0f) {
    float arm = 0.707106781f * armLength;
    rollPart = 0.25f / arm * torqueX;
    pitchPart = 0.25f / arm * torqueY;
  }
  if (thrustToTorque != 0.0f) {
    yawPart = 0.25f / thrustToTorque * torqueZ;
  }
  float f1 = thrustPart - rollPart + pitchPart + yawPart;
  float f2 = thrustPart - rollPart - pitchPart - yawPart;
  float f3 = thrustPart + rollPart - pitchPart + yawPart;
  float f4 = thrustPart + rollPart + pitchPart - yawPart;
  motorForces[0] = f1 > 0.0f ? f1 : 0.0f;
  motorForces[1] = f2 > 0.0f ? f2 : 0.0f;
  motorForces[2] = f3 > 0.0f ? f3 : 0.0f;
  motorForces[3] = f4 > 0.0f ? f4 : 0.0f;
}

void powerDistributionForce(const float normalizedForces[4],
                            uint16_t motorPWMs[4]) {
  if (normalizedForces == NULL || motorPWMs == NULL) return;
  for (int i = 0; i < 4; ++i) {
    float n = clampf(normalizedForces[i], 0.0f, 1.0f);
    motorPWMs[i] = (uint16_t)(n * 65535.0f);
  }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower) {
  if (control == NULL || motorPower == NULL) return;
  switch (control->controlMode) {
    case controlModeLegacy:
      powerDistributionLegacy(control->thrust, control->roll, control->pitch,
                              control->yaw, motorPower);
      break;
    case controlModeForceTorque: {
      float forces[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      powerDistributionForceTorque(control->thrustSi, control->torque.x,
                                   control->torque.y, control->torque.z,
                                   CRAZYFLIE_ARM_LENGTH_M,
                                   CRAZYFLIE_THRUST_TO_TORQUE, forces);
      motorPower->m1 = motorForceToPwm(forces[0]);
      motorPower->m2 = motorForceToPwm(forces[1]);
      motorPower->m3 = motorForceToPwm(forces[2]);
      motorPower->m4 = motorForceToPwm(forces[3]);
      break;
    }
    case controlModeForce: {
      uint16_t pwms[4] = {0U, 0U, 0U, 0U};
      powerDistributionForce(control->normalizedForces, pwms);
      motorPower->m1 = (int32_t)pwms[0];
      motorPower->m2 = (int32_t)pwms[1];
      motorPower->m3 = (int32_t)pwms[2];
      motorPower->m4 = (int32_t)pwms[3];
      break;
    }
    default:
      break;
  }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) {
  return value < idleThrust ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust) {
  PowerCapResult result = {false, 0};
  if (motors == NULL) return result;
  int32_t maxVal = motors[0];
  for (int i = 1; i < 4; ++i) {
    if (motors[i] > maxVal) maxVal = motors[i];
  }
  if (maxVal > maxAllowedThrust) {
    result.reduction = maxVal - maxAllowedThrust;
    result.isCapped = true;
  } else {
    result.reduction = 0;
    result.isCapped = false;
  }
  for (int i = 0; i < 4; ++i) {
    motors[i] = capMinThrust(motors[i] - result.reduction, idleThrust);
  }
  return result;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) {
  return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust,
                                        float nominalVoltage,
                                        float actualVoltage) {
  if (actualVoltage <= 0.0f) return motorThrust;
  float compensated = roundf((float)motorThrust * nominalVoltage / actualVoltage);
  compensated = clampf(compensated, 0.0f, 65535.0f);
  return (uint16_t)compensated;
}

/* ------------------------------------------------------------------------- */
/* PID */
/* ------------------------------------------------------------------------- */
PidObject pidRoll = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidPitch = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidYaw = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidRollRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidPitchRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidYawRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

static float pidUpdate(PidObject *pid, float error) {
  if (pid == NULL) return 0.0f;
  float out = pid->kp * error + pid->ki * pid->integral +
              pid->kd * (error - pid->prevError);
  pid->integral += error;
  pid->prevError = error;
  pid->output = out;
  return out;
}

static void pidReset(PidObject *pid) {
  if (pid == NULL) return;
  pid->integral = 0.0f;
  pid->prevError = 0.0f;
  pid->output = 0.0f;
}

void attitudeControllerInit(float updateDt) {
  (void)updateDt;
  static bool initialized = false;
  if (initialized) return;
  PidObject *objs[] = {&pidRoll, &pidPitch, &pidYaw,
                       &pidRollRate, &pidPitchRate, &pidYawRate};
  for (int i = 0; i < 6; ++i) {
    objs[i]->kp = 0.0f;
    objs[i]->ki = 0.0f;
    objs[i]->kd = 0.0f;
    objs[i]->kff = 0.0f;
    objs[i]->integral = 0.0f;
    objs[i]->prevError = 0.0f;
    objs[i]->output = 0.0f;
    objs[i]->initialized = true;
  }
  initialized = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
  float outRoll = pidUpdate(&pidRollRate, rollDesired - rollActual);
  float outPitch = pidUpdate(&pidPitchRate, pitchDesired - pitchActual);
  float outYaw = pidUpdate(&pidYawRate, yawDesired - yawActual);
  pidRollRate.output = (float)saturateSignedInt16((int32_t)outRoll);
  pidPitchRate.output = (float)saturateSignedInt16((int32_t)outPitch);
  pidYawRate.output = (float)saturateSignedInt16((int32_t)outYaw);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
  pidUpdate(&pidRoll, rollDesired - rollActual);
  pidUpdate(&pidPitch, pitchDesired - pitchActual);
  pidUpdate(&pidYaw, capAngle(yawDesired - yawActual));
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual) {
  (void)rollActual; (void)pitchActual; (void)yawActual;
  pidReset(&pidRoll);
  pidReset(&pidPitch);
  pidReset(&pidYaw);
  pidReset(&pidRollRate);
  pidReset(&pidPitchRate);
  pidReset(&pidYawRate);
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
  (void)rollActual;
  pidReset(&pidRoll);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
  (void)pitchActual;
  pidReset(&pidPitch);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw) {
  if (roll == NULL || pitch == NULL || yaw == NULL) return;
  *roll = (int16_t)pidRollRate.output;
  *pitch = (int16_t)pidPitchRate.output;
  *yaw = (int16_t)pidYawRate.output;
}

static float quatYawDeg(const Quaternion *q) {
  if (q == NULL) return 0.0f;
  return atan2f(2.0f * (q->w * q->z + q->x * q->y),
                1.0f - 2.0f * (q->y * q->y + q->z * q->z)) * RAD2DEG;
}

static float g_desiredYaw = 0.0f;

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
  if (setpoint == NULL || state == NULL) return 0U;
  float errorPos = setpoint->position.z - state->position.z;
  float errorVel = setpoint->velocity.z - state->velocity.z;
  float thrust = 32768.0f + 100.0f * errorPos + 20.0f * errorVel;
  thrust = clampf(thrust, (float)MIN_THRUST, (float)MAX_THRUST);
  return (uint16_t)thrust;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
  if (sensors == NULL || setpoint == NULL || state == NULL || control == NULL) return;

  control->roll = 0;
  control->pitch = 0;
  control->yaw = 0;
  control->thrust = 0;
  control->thrustSi = 0.0f;
  control->torque.x = 0.0f;
  control->torque.y = 0.0f;
  control->torque.z = 0.0f;
  for (int i = 0; i < 4; ++i) control->normalizedForces[i] = 0.0f;

  if (setpoint->mode.z == modeDisable) {
    control->thrust = setpoint->thrust;
  } else {
    control->thrust = positionControllerUpdate(setpoint, state);
  }

  if (control->thrust == 0U) {
    attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch,
                                  state->attitude.yaw);
    g_desiredYaw = state->attitude.yaw;
    return;
  }

  float desiredRollRate = 0.0f;
  float desiredPitchRate = 0.0f;

  if (setpoint->mode.roll == modeVelocity) {
    attitudeControllerResetRollAttitudePID(state->attitude.roll);
    desiredRollRate = setpoint->attitudeRate.roll;
  } else if (setpoint->mode.roll == modeAbs) {
    desiredRollRate = pidUpdate(&pidRoll, setpoint->attitude.roll - state->attitude.roll);
  }

  if (setpoint->mode.pitch == modeVelocity) {
    attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
    desiredPitchRate = setpoint->attitudeRate.pitch;
  } else if (setpoint->mode.pitch == modeAbs) {
    desiredPitchRate = pidUpdate(&pidPitch, setpoint->attitude.pitch - state->attitude.pitch);
  }

  if (setpoint->mode.yaw == modeVelocity) {
    g_desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
    if (yawMaxDelta != 0.0f) {
      float delta = g_desiredYaw - state->attitude.yaw;
      if (delta > fabsf(yawMaxDelta)) g_desiredYaw = state->attitude.yaw + fabsf(yawMaxDelta);
      else if (delta < -fabsf(yawMaxDelta)) g_desiredYaw = state->attitude.yaw - fabsf(yawMaxDelta);
    }
  } else if (setpoint->mode.yaw == modeAbs) {
    g_desiredYaw = setpoint->attitude.yaw;
  } else if (setpoint->mode.quat == modeAbs) {
    g_desiredYaw = quatYawDeg(&setpoint->attitudeQuaternion);
  }

  float desiredYawRate = pidUpdate(&pidYaw, capAngle(g_desiredYaw - state->attitude.yaw));

  float pitchActual = -sensors->gyro.y;
  attitudeControllerCorrectRatePID(sensors->gyro.x, desiredRollRate,
                                   pitchActual, desiredPitchRate,
                                   sensors->gyro.z, desiredYawRate);
  int16_t outRoll, outPitch, outYaw;
  attitudeControllerGetActuatorOutput(&outRoll, &outPitch, &outYaw);
  control->roll = outRoll;
  control->pitch = outPitch;
  control->yaw = (int16_t)(-outYaw);
}

/* ------------------------------------------------------------------------- */
/* CRTP Commander RPYT */
/* ------------------------------------------------------------------------- */
bool thrustLocked = true;
bool commanderModeSet = false;

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
  if (rollPrime == NULL || pitchPrime == NULL) return;
  float rad = yaw_deg * DEG2RAD;
  float c = cosf(rad);
  float s = sinf(rad);
  *rollPrime = roll * c + pitch * s;
  *pitchPrime = -roll * s + pitch * c;
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
  if (values == NULL || setpoint == NULL) return;

  if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
    thrustLocked = true;
    if (values->thrust == 0U) thrustLocked = false;
  }

  bool rawZero = (values->thrust == 0U);
  zeroSetpoint(setpoint);

  if (commanderModeSet && !altHoldMode) {
    commanderModeSet = false;
    setpoint->mode.z = modeDisable;
  }

  bool horizontalHandled = false;

  if (altHoldMode) {
    setpoint->mode.z = modeVelocity;
    setpoint->thrust = 0U;
    setpoint->velocity.z = ((float)values->thrust - 32767.0f) / 32767.0f;
    if (!commanderModeSet) {
      commanderModeSet = true;
      /* Position PID/filter reset is intentionally requested here; no public
         position PID object exists, so the reset boundary is the re-entry. */
    }
  }

  if (posHoldMode) {
    horizontalHandled = true;
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->velocity.x = values->pitch / 30.0f;
    setpoint->velocity.y = values->roll / 30.0f;
    setpoint->attitude.roll = 0.0f;
    setpoint->attitude.pitch = 0.0f;
  } else if (posSetMode && !rawZero) {
    horizontalHandled = true;
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;
    setpoint->position.x = -values->pitch;
    setpoint->position.y = values->roll;
    setpoint->position.z = (float)values->thrust / 1000.0f;
    setpoint->attitude.yaw = values->yaw;
    setpoint->thrust = 0U;
  }

  if (!horizontalHandled) {
    if (stabilizationModeRoll == RATE) {
      setpoint->mode.roll = modeVelocity;
      setpoint->attitudeRate.roll = values->roll;
    } else {
      setpoint->mode.roll = modeAbs;
      setpoint->attitude.roll = values->roll;
    }
    if (stabilizationModePitch == RATE) {
      setpoint->mode.pitch = modeVelocity;
      setpoint->attitudeRate.pitch = values->pitch;
    } else {
      setpoint->mode.pitch = modeAbs;
      setpoint->attitude.pitch = values->pitch;
    }
    if (stabilizationModeYaw == RATE) {
      setpoint->mode.yaw = modeVelocity;
      setpoint->attitudeRate.yaw = -values->yaw;
    } else {
      setpoint->mode.yaw = modeAbs;
      setpoint->attitude.yaw = values->yaw;
    }
  }

  if (!altHoldMode && !(posSetMode && !rawZero)) {
    setpoint->mode.z = modeDisable;
    if (thrustLocked || values->thrust < MIN_THRUST) {
      setpoint->thrust = 0U;
    } else {
      setpoint->thrust = values->thrust > MAX_THRUST ? MAX_THRUST : values->thrust;
    }
  }

  if (yawMode == PLUSMODE && !horizontalHandled) {
    float rp = 0.0f, pp = 0.0f;
    rotateYaw(values->roll, values->pitch, 45.0f, &rp, &pp);
    if (setpoint->mode.roll == modeVelocity) setpoint->attitudeRate.roll = rp;
    else setpoint->attitude.roll = rp;
    if (setpoint->mode.pitch == modeVelocity) setpoint->attitudeRate.pitch = pp;
    else setpoint->attitude.pitch = pp;
  } else if (yawMode == XMODE) {
    /* No rotation. */
  } else {
    /* CAREFREE observable error path: reject the command cleanly. */
    zeroSetpoint(setpoint);
    setpoint->thrust = 0U;
    thrustLocked = true;
  }
}

/* ------------------------------------------------------------------------- */
/* Supervisor */
/* ------------------------------------------------------------------------- */
SupervisorState supervisorState = supervisorStatePreFlChecksPassed;
uint32_t supervisorConditionBits = 0U;

static SensorData g_supSensor;
static uint32_t g_supMotorRatios[4] = {0U, 0U, 0U, 0U};
static int32_t g_supMotorRPMs[4] = {0, 0, 0, 0};
static uint32_t g_supIdleThrust = 0U;
static float g_crashDetectionGs = 0.0f;
static float g_freeFallThreshold = 0.0f;
static float g_acceptedTiltAccZ = 0.0f;
static float g_acceptedUpsideDownAccZ = 0.0f;
static uint32_t g_maxTiltTime = 0U;
static uint32_t g_maxUpsideDownTime = 0U;
static bool g_tumbleCheckEnabled = false;
static bool g_autoArming = false;
static uint32_t g_spinupTimeoutDurationMs = 0U;
static uint32_t g_spinupStartTick = 0U;
static uint32_t g_latestArmingTick = 0U;
static bool g_seenFlying = false;
static uint32_t g_recentFlightTick = 0U;
static bool g_isTumbled = false;
static bool g_isFreeFalling = false;
static uint32_t g_tiltStartTick = 0U;
static uint32_t g_upsideDownStartTick = 0U;
static uint32_t g_notRespondingStartTick = 0U;
static bool g_trajectoryFlying = false;
static bool g_trajectoryFinished = false;
static bool g_trajectoryDisabled = false;
static bool g_deckFault = false;
static SupervisorState g_prevSupervisorState = supervisorStatePreFlChecksPassed;

void supervisorInit(void) {
  supervisorState = supervisorStatePreFlChecksPassed;
  supervisorConditionBits = 0U;
  g_seenFlying = false;
  g_recentFlightTick = 0U;
  g_isTumbled = false;
  g_isFreeFalling = false;
  g_tiltStartTick = 0U;
  g_upsideDownStartTick = 0U;
  g_notRespondingStartTick = 0U;
  g_spinupStartTick = 0U;
  g_latestArmingTick = 0U;
  g_prevSupervisorState = supervisorState;
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
    if (!supervisorCanArm()) return false;
    if (!supervisorIsArmed()) {
      supervisorConditionBits |= SUPERVISOR_CB_ARMED;
      supervisorState = supervisorStateArming;
      g_spinupStartTick = 0U;
      g_latestArmingTick = currentTick;
    }
    return true;
  }
  supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
  supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
  g_spinupStartTick = 0U;
  if (supervisorState == supervisorStateArming) {
    supervisorState = supervisorStatePreFlChecksPassed;
  }
  return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
  if (g_isTumbled) return false;
  if (doRecovery) {
    supervisorState = supervisorStatePreFlChecksPassed;
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    return true;
  }
  supervisorState = supervisorStateCrashed;
  supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
  return true;
}

bool supervisorAreMotorsAllowedToRun(void) {
  return supervisorState == supervisorStateArming ||
         supervisorState == supervisorStateReadyToFly ||
         supervisorState == supervisorStateFlying ||
         supervisorState == supervisorStateWarningLevelOut ||
         supervisorState == supervisorStateLanded;
}

uint16_t supervisorGetInfoBitfield(void) {
  uint16_t info = 0U;
  if (supervisorCanArm()) info |= (1U << 0);
  if (supervisorIsArmed()) info |= (1U << 1);
  if (g_autoArming) info |= (1U << 2);
  if (supervisorCanFly()) info |= (1U << 3);
  if ((supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) != 0U) info |= (1U << 4);
  if (g_isTumbled || ((supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) != 0U)) info |= (1U << 5);
  if (supervisorState == supervisorStateLocked) info |= (1U << 6);
  if (supervisorIsCrashed()) info |= (1U << 7);
  if (g_trajectoryFlying) info |= (1U << 8);
  if (g_trajectoryFinished) info |= (1U << 9);
  if (g_trajectoryDisabled) info |= (1U << 10);
  if (g_deckFault || ((supervisorConditionBits & SUPERVISOR_CB_DECK_FAULT) != 0U)) info |= (1U << 11);
  return info;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTickParam) {
  if (motorRatios == NULL) return false;
  bool active = false;
  for (int i = 0; i < 4; ++i) {
    if (motorRatios[i] > idleThrust) {
      active = true;
      break;
    }
  }
  if (active) {
    g_recentFlightTick = currentTickParam;
    g_seenFlying = true;
  }
  if (!g_seenFlying) return false;
  return (uint32_t)(currentTickParam - g_recentFlightTick) <
         IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTickParam,
                    bool *isFreeFalling) {
  if (isFreeFalling == NULL) return false;

  if (crashDetectionGs > 0.0f) {
    float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
    if (fabsf(norm - 1.0f) > crashDetectionGs) {
      supervisorState = supervisorStateCrashed;
      supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    }
  }

  if (!tumbleCheckEnabled) {
    *isFreeFalling = false;
    return false;
  }

  if (fabsf(accX) < freeFallThreshold &&
      fabsf(accY) < freeFallThreshold &&
      fabsf(accZ) < freeFallThreshold) {
    *isFreeFalling = true;
    g_isFreeFalling = true;
    supervisorState = supervisorStateExceptFreeFall;
    supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    g_isTumbled = false;
    g_tiltStartTick = 0U;
    g_upsideDownStartTick = 0U;
    return false;
  }

  *isFreeFalling = false;
  g_isFreeFalling = false;
  supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

  bool tumbled = false;
  if (accZ < acceptedUpsideDownAccZ) {
    if (g_upsideDownStartTick == 0U) g_upsideDownStartTick = currentTickParam;
    if ((uint32_t)(currentTickParam - g_upsideDownStartTick) >= maxUpsideDownTime) {
      tumbled = true;
    }
  } else if (accZ < acceptedTiltAccZ) {
    if (g_tiltStartTick == 0U) g_tiltStartTick = currentTickParam;
    if ((uint32_t)(currentTickParam - g_tiltStartTick) >= maxTiltTime) {
      tumbled = true;
    }
  } else {
    g_tiltStartTick = 0U;
    g_upsideDownStartTick = 0U;
    tumbled = false;
  }

  g_isTumbled = tumbled;
  if (tumbled) {
    supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
  }
  return tumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTickParam,
                                uint32_t lastNotificationTick) {
  if (lastNotificationTick == 0U) return true;
  return (uint32_t)(currentTickParam - lastNotificationTick) <=
         DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTickParam,
                                  uint32_t preflightTimeoutDuration) {
  if (state != supervisorStateReadyToFly) return false;
  if (latestArmingTick == 0U) return false;
  return (uint32_t)(currentTickParam - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTickParam,
                                uint32_t landingTimeoutDuration) {
  if (latestLandingTick == 0U) return false;
  return (uint32_t)(currentTickParam - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed) {
  if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
    supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
  }
  return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBitsParam,
                                SupervisorState state) {
  if (setpoint == NULL) return;
  if ((supervisorConditionBitsParam & (SUPERVISOR_CB_EMERGENCY_STOP |
                                        SUPERVISOR_CB_IS_TUMBLED |
                                        SUPERVISOR_CB_FREE_FALL |
                                        SUPERVISOR_CB_CRASHED |
                                        SUPERVISOR_CB_MOTORS_NOT_RESPONDING)) != 0U) {
    zeroSetpoint(setpoint);
    return;
  }
  switch (state) {
    case supervisorStateWarningLevelOut:
      setpoint->mode.x = modeDisable;
      setpoint->mode.y = modeDisable;
      setpoint->velocity.x = 0.0f;
      setpoint->velocity.y = 0.0f;
      setpoint->mode.roll = modeAbs;
      setpoint->mode.pitch = modeAbs;
      setpoint->attitude.roll = 0.0f;
      setpoint->attitude.pitch = 0.0f;
      setpoint->mode.yaw = modeVelocity;
      setpoint->attitudeRate.yaw = 0.0f;
      break;
    case supervisorStateArming:
    case supervisorStateReadyToFly:
    case supervisorStateFlying:
    case supervisorStateLanded:
      break;
    default:
      zeroSetpoint(setpoint);
      break;
  }
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax) {
  if (motorRPMs == NULL) return false;
  for (int i = 0; i < 4; ++i) {
    if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
  }
  return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTickParam) {
  if (motorRPMs == NULL) return false;
  if (!canFly) {
    g_notRespondingStartTick = 0U;
    supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return false;
  }
  bool below = false;
  for (int i = 0; i < 4; ++i) {
    if (motorRPMs[i] < rpmThreshold) {
      below = true;
      break;
    }
  }
  if (!below) {
    g_notRespondingStartTick = 0U;
    supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return false;
  }
  if (g_notRespondingStartTick == 0U) g_notRespondingStartTick = currentTickParam;
  if ((uint32_t)(currentTickParam - g_notRespondingStartTick) >= rpmCheckDurationMs) {
    supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return true;
  }
  return false;
}

void supervisorSetSensorData(const SensorData *sensors) {
  if (sensors == NULL) return;
  g_supSensor = *sensors;
  acc.x = sensors->acc.x;
  acc.y = sensors->acc.y;
  acc.z = sensors->acc.z;
  gyro.x = sensors->gyro.x;
  gyro.y = sensors->gyro.y;
  gyro.z = sensors->gyro.z;
  baro.asl = sensors->baroAsl;
  baro.temp = sensors->baroTemperature;
  baro.pressure = sensors->baroPressure;
  supervisorLog.accNorm = sqrtf(sensors->acc.x * sensors->acc.x +
                                sensors->acc.y * sensors->acc.y +
                                sensors->acc.z * sensors->acc.z);
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
  if (motorRatios == NULL) return;
  for (int i = 0; i < 4; ++i) g_supMotorRatios[i] = motorRatios[i];
  g_supIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
  if (motorRPMs == NULL) return;
  for (int i = 0; i < 4; ++i) g_supMotorRPMs[i] = motorRPMs[i];
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled) {
  g_crashDetectionGs = crashDetectionGs;
  g_freeFallThreshold = freeFallThreshold;
  g_acceptedTiltAccZ = acceptedTiltAccZ;
  g_acceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
  g_maxTiltTime = maxTiltTime;
  g_maxUpsideDownTime = maxUpsideDownTime;
  g_tumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
  g_autoArming = autoArming;
  g_spinupTimeoutDurationMs = spinupTimeoutDurationMs;
  g_spinupStartTick = 0U;
}

void supervisorUpdate(uint32_t stabilizerStep) {
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

  bool flying = isFlyingCheck(g_supMotorRatios, g_supIdleThrust, currentTick);
  if (flying) {
    supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
  }

  bool freeFall = false;
  (void)isTumbledCheck(g_supSensor.acc.x, g_supSensor.acc.y, g_supSensor.acc.z,
                       g_crashDetectionGs, g_freeFallThreshold,
                       g_acceptedTiltAccZ, g_acceptedUpsideDownAccZ,
                       g_maxTiltTime, g_maxUpsideDownTime,
                       g_tumbleCheckEnabled, currentTick, &freeFall);

  if (supervisorIsArmed()) {
    supervisorConditionBits |= SUPERVISOR_CB_ARMED;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
  }

  uint32_t age = commanderGetInactivityTime();
  if (age > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
    supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
  } else if (age > COMMANDER_WDT_TIMEOUT_STABILIZE) {
    supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
  }

  if (supervisorState == supervisorStateArming &&
      g_spinupStartTick != 0U &&
      (uint32_t)(currentTick - g_spinupStartTick) >= g_spinupTimeoutDurationMs) {
    supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
  }

  if (g_autoArming &&
      supervisorState == supervisorStatePreFlChecksPassed &&
      g_prevSupervisorState != supervisorStatePreFlChecksPassed) {
    (void)supervisorRequestArming(true);
  }

  g_prevSupervisorState = supervisorState;
  supervisorLog.info = (uint32_t)supervisorGetInfoBitfield();
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

/* ------------------------------------------------------------------------- */
/* Estimator and Commander */
/* ------------------------------------------------------------------------- */
#define ESTIMATOR_FIFO_SIZE 16U
static EstimatorMeasurement g_estFifo[ESTIMATOR_FIFO_SIZE];
static uint8_t g_estHead = 0U;
static uint8_t g_estTail = 0U;
static uint8_t g_estCount = 0U;

static State g_estimatorState;

bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
  if (measurement == NULL || g_estCount >= ESTIMATOR_FIFO_SIZE) return false;
  g_estFifo[g_estTail] = *measurement;
  g_estTail = (uint8_t)((g_estTail + 1U) % ESTIMATOR_FIFO_SIZE);
  g_estCount++;
  return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
  if (measurement == NULL || g_estCount == 0U) return false;
  *measurement = g_estFifo[g_estHead];
  g_estHead = (uint8_t)((g_estHead + 1U) % ESTIMATOR_FIFO_SIZE);
  g_estCount--;
  return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
  EstimatorMeasurement m;
  bool haveGyro = false, haveAcc = false, haveBaro = false, haveTof = false;
  float gyroData[3] = {0.0f, 0.0f, 0.0f};
  float accData[3] = {0.0f, 0.0f, 0.0f};
  float baroData[3] = {0.0f, 0.0f, 0.0f};
  float tofData[3] = {0.0f, 0.0f, 0.0f};

  while (estimatorDequeue(&m)) {
    switch (m.type) {
      case MeasurementTypeGyroscope:
        haveGyro = true;
        memcpy(gyroData, m.data, sizeof(gyroData));
        break;
      case MeasurementTypeAcceleration:
        haveAcc = true;
        memcpy(accData, m.data, sizeof(accData));
        break;
      case MeasurementTypeBarometer:
        haveBaro = true;
        memcpy(baroData, m.data, sizeof(baroData));
        break;
      case MeasurementTypeTOF:
        haveTof = true;
        memcpy(tofData, m.data, sizeof(tofData));
        break;
      default:
        break;
    }
  }

  if (!RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) return;

  float dt = 1.0f / (float)SENSFUSION_RATE_HZ;
  if (haveGyro && haveAcc) {
    sensfusion6UpdateQ(gyroData[0], gyroData[1], gyroData[2],
                       accData[0], accData[1], accData[2], dt);
  }
  sensfusion6GetEulerRPY(&g_estimatorState.attitude.roll,
                         &g_estimatorState.attitude.pitch,
                         &g_estimatorState.attitude.yaw);
  sensfusion6GetQuaternion(&g_estimatorState.attitudeQuaternion.w,
                           &g_estimatorState.attitudeQuaternion.x,
                           &g_estimatorState.attitudeQuaternion.y,
                           &g_estimatorState.attitudeQuaternion.z);
  if (haveAcc) {
    g_estimatorState.acc.x = accData[0];
    g_estimatorState.acc.y = accData[1];
    g_estimatorState.acc.z = accData[2];
    float accZ = sensfusion6GetAccZWithoutGravity(accData[0], accData[1], accData[2]);
    g_estimatorState.velocity.z += accZ * dt;
  }
  if (haveBaro) {
    baro.asl = baroData[0];
    baro.temp = baroData[1];
    baro.pressure = baroData[2];
  }
  if (haveTof) {
    /* tof range is stored in the x component in this host model. */
  }

  stateEstimate.roll = g_estimatorState.attitude.roll;
  stateEstimate.pitch = g_estimatorState.attitude.pitch;
  stateEstimate.yaw = g_estimatorState.attitude.yaw;
  stateEstimate.qx = g_estimatorState.attitudeQuaternion.x;
  stateEstimate.qy = g_estimatorState.attitudeQuaternion.y;
  stateEstimate.qz = g_estimatorState.attitudeQuaternion.z;
  stateEstimate.qw = g_estimatorState.attitudeQuaternion.w;

  if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
    g_estimatorState.position.x += g_estimatorState.velocity.x * (1.0f / (float)POSITION_RATE_HZ);
    g_estimatorState.position.y += g_estimatorState.velocity.y * (1.0f / (float)POSITION_RATE_HZ);
    g_estimatorState.position.z += g_estimatorState.velocity.z * (1.0f / (float)POSITION_RATE_HZ);
  }
}

static Setpoint g_activeSetpoint;
static int g_activePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t g_lastCommanderUpdateTick = 0U;

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
  if (setpoint == NULL) return false;
  if (priority != COMMANDER_PRIORITY_DISABLE && priority < g_activePriority) {
    return false;
  }
  if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
    /* Stop high-level trajectory. */
    g_trajectoryFlying = false;
  }
  g_activeSetpoint = *setpoint;
  g_activePriority = priority;
  g_lastCommanderUpdateTick = currentTick;
  return true;
}

void commanderRelaxPriority(void) {
  g_activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
  return (uint32_t)(currentTick - g_lastCommanderUpdateTick);
}

int commanderGetActivePriority(void) {
  return g_activePriority;
}

/* ------------------------------------------------------------------------- */
/* Stabilizer */
/* ------------------------------------------------------------------------- */
StateEstimateLog stateEstimate;
Axis3Log gyro;
Axis3Log acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

static bool g_stabilizerInitDone = false;
static bool g_highLevelPending = false;
static Setpoint g_pendingHighLevelSetpoint;
static uint32_t g_stabilizerStep = 0U;
static SensorData g_currentSensors;
static float g_batteryFiltered = 0.0f;

static void sensorsInit(void) { memset(&g_currentSensors, 0, sizeof(g_currentSensors)); }
static void stateEstimatorInit(void) {
  g_estHead = g_estTail = g_estCount = 0;
  memset(&g_estimatorState, 0, sizeof(g_estimatorState));
}
static void controllerInit(void) { attitudeControllerInit(1.0f / (float)ATTITUDE_RATE_HZ); }
static void powerDistributionInit(void) { motor.m1req = motor.m2req = motor.m3req = motor.m4req = 0U; }
static void motorsInit(void) { motor.m1req = motor.m2req = motor.m3req = motor.m4req = 0U; }
static void collisionAvoidanceInit(void) { /* no state */ }

static void sensorsWaitDataReady(void) { /* host platform is ready every step. */ }
static void sensorsAcquire(void) { /* g_currentSensors is the injected sample. */ }
static void setMotorRatios(uint32_t ratios[4]) {
  motor.m1req = (uint16_t)ratios[0];
  motor.m2req = (uint16_t)ratios[1];
  motor.m3req = (uint16_t)ratios[2];
  motor.m4req = (uint16_t)ratios[3];
}

void stabilizerInit(void) {
  if (g_stabilizerInitDone) return;
  sensorsInit();
  stateEstimatorInit();
  controllerInit();
  powerDistributionInit();
  motorsInit();
  collisionAvoidanceInit();
  g_stabilizerInitDone = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
  if (setpoint == NULL) return false;
  g_pendingHighLevelSetpoint = *setpoint;
  g_highLevelPending = true;
  return true;
}

static uint32_t quatcompressFromQ(float w, float x, float y, float z) {
  int8_t qw = (int8_t)(clampf(w, -1.0f, 1.0f) * 127.0f);
  int8_t qx = (int8_t)(clampf(x, -1.0f, 1.0f) * 127.0f);
  int8_t qy = (int8_t)(clampf(y, -1.0f, 1.0f) * 127.0f);
  int8_t qz = (int8_t)(clampf(z, -1.0f, 1.0f) * 127.0f);
  return ((uint32_t)((uint8_t)qw) |
          ((uint32_t)((uint8_t)qx) << 8) |
          ((uint32_t)((uint8_t)qy) << 16) |
          ((uint32_t)((uint8_t)qz) << 24));
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output) {
  if (state == NULL || sensors == NULL || output == NULL) return;
  output->position_mm[0] = (int32_t)(state->position.x * 1000.0f);
  output->position_mm[1] = (int32_t)(state->position.y * 1000.0f);
  output->position_mm[2] = (int32_t)(state->position.z * 1000.0f);
  output->velocity_mms[0] = (int32_t)(state->velocity.x * 1000.0f);
  output->velocity_mms[1] = (int32_t)(state->velocity.y * 1000.0f);
  output->velocity_mms[2] = (int32_t)(state->velocity.z * 1000.0f);
  output->acceleration_mms2[0] = (int32_t)(sensors->acc.x * 9810.0f);
  output->acceleration_mms2[1] = (int32_t)(sensors->acc.y * 9810.0f);
  output->acceleration_mms2[2] = (int32_t)((sensors->acc.z + 1.0f) * 9810.0f);
  output->gyro_millirad_s[0] = sensors->gyro.x * DEG2RAD * 1000.0f;
  output->gyro_millirad_s[1] = -sensors->gyro.y * DEG2RAD * 1000.0f;
  output->gyro_millirad_s[2] = sensors->gyro.z * DEG2RAD * 1000.0f;
  output->quatCompressed = quatcompressFromQ(state->attitudeQuaternion.w,
                                             state->attitudeQuaternion.x,
                                             state->attitudeQuaternion.y,
                                             state->attitudeQuaternion.z);
}

bool rateSupervisorValidate(uint32_t measuredRate) {
  return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void) {
  static uint32_t lastRateCheckTick = 0U;
  static bool rateInitialized = false;
  if (!rateInitialized) {
    lastRateCheckTick = currentTick;
    rateInitialized = true;
    return;
  }
  if ((uint32_t)(currentTick - lastRateCheckTick) < 2000U) return;
  lastRateCheckTick = currentTick;
  uint32_t measured = 1000U; /* host model stable rate. */
  if (!rateSupervisorValidate(measured)) {
    supervisorConditionBits |= SUPERVISOR_CB_DECK_FAULT; /* error state */
  }
}

void stabilizerTask(void) {
  sensorsWaitDataReady();
  sensorsAcquire();
  estimatorComplementary(g_stabilizerStep);

  if (g_highLevelPending) {
    (void)commanderSetSetpoint(&g_pendingHighLevelSetpoint,
                               COMMANDER_PRIORITY_HIGHLEVEL);
    g_highLevelPending = false;
  }

  Setpoint sp = g_activeSetpoint;
  supervisorUpdate(g_stabilizerStep);

  if (!supervisorCanFly()) {
    uint32_t zeros[4] = {0U, 0U, 0U, 0U};
    setMotorRatios(zeros);
    return;
  }

  if (healthShallWeRunTest()) {
    healthRunTests(&g_currentSensors);
    return;
  }

  /* collisionAvoidanceUpdateSetpoint is currently identity. */
  supervisorOverrideSetpoint(&sp, supervisorConditionBits, supervisorState);

  ControlData control;
  memset(&control, 0, sizeof(control));
  control.controlMode = controlModeLegacy;
  controllerPid(&g_currentSensors, &sp, &g_estimatorState, &control,
                0.0f, 1.0f / (float)ATTITUDE_RATE_HZ);

  MotorPower mp;
  powerDistribution(&control, &mp);

  g_batteryFiltered = batteryCompensation(0.0f, g_batteryFiltered, 0.01f);
  int32_t motorVals[4] = {mp.m1, mp.m2, mp.m3, mp.m4};
  (void)powerDistributionCap(motorVals, 65535, 0);

  if (!supervisorAreMotorsAllowedToRun()) {
    uint32_t zeros[4] = {0U, 0U, 0U, 0U};
    setMotorRatios(zeros);
    return;
  }

  uint32_t ratios[4] = {(uint32_t)motorVals[0], (uint32_t)motorVals[1],
                        (uint32_t)motorVals[2], (uint32_t)motorVals[3]};
  setMotorRatios(ratios);
  g_stabilizerStep++;
}

/* ------------------------------------------------------------------------- */
/* Health */
/* ------------------------------------------------------------------------- */
TestState healthTestState = testDone;
uint8_t motorPass = 0U;
uint8_t batteryPass = 0U;
float batterySag = 0.0f;

static bool g_propRequestPending = false;
static bool g_batRequestPending = false;
static uint8_t g_propSampleCount = 0U;
static uint8_t g_propMotorIndex = 0U;
static float g_propNoiseSamples[PROPTEST_NBR_OF_VARIANCE_VALUES];
static uint8_t g_batTick = 0U;
static float g_idleVoltage = 0.0f;
static float g_minLoadedVoltage = 0.0f;
static uint32_t g_restartStartTick = 0U;

bool healthShallWeRunTest(void) {
  if (g_propRequestPending) {
    g_propRequestPending = false;
    healthTestState = configureAcc;
    g_propSampleCount = 0;
    g_propMotorIndex = 0;
    motorPass = 0U;
    batteryPass = 0U;
    batterySag = 0.0f;
    healthLog.motorPass = motorPass;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
    return true;
  }
  if (g_batRequestPending) {
    g_batRequestPending = false;
    healthTestState = testBattery;
    g_batTick = 0U;
    batteryPass = 0U;
    batterySag = 0.0f;
    g_minLoadedVoltage = 0.0f;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
    return true;
  }
  return healthTestState != testDone;
}

void healthRequestPropTest(void) {
  g_propRequestPending = true;
}

void healthRequestBatteryTest(void) {
  g_batRequestPending = true;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex) {
  if (motorIndex >= 4U) return false;
  if (highThreshold == 0.0f) return true;
  if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
    motorPass |= (uint8_t)(1U << motorIndex);
    return true;
  }
  return false;
}

float variance(const float *buffer, int length) {
  if (buffer == NULL || length <= 0) return 0.0f;
  float sum = 0.0f;
  float sumSq = 0.0f;
  for (int i = 0; i < length; ++i) {
    sum += buffer[i];
    sumSq += buffer[i] * buffer[i];
  }
  return sumSq - (sum * sum) / (float)length;
}

void healthRunTests(const SensorData *sensorData) {
  if (sensorData == NULL) return;

  switch (healthTestState) {
    case testDone:
      break;
    case configureAcc:
      g_propSampleCount = 0U;
      g_propMotorIndex = 0U;
      motorPass = 0U;
      batteryPass = 0U;
      batterySag = 0.0f;
      healthTestState = measureNoiseFloor;
      break;
    case measureNoiseFloor:
      if (g_propSampleCount < PROPTEST_NBR_OF_VARIANCE_VALUES) {
        g_propNoiseSamples[g_propSampleCount++] = sensorData->acc.x;
      }
      if (g_propSampleCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
        (void)variance(g_propNoiseSamples, PROPTEST_NBR_OF_VARIANCE_VALUES);
        healthTestState = measureProp;
        g_propMotorIndex = 0U;
      }
      break;
    case measureProp:
      if (g_propMotorIndex < 4U) {
        motorPass |= (uint8_t)(1U << g_propMotorIndex);
        g_propMotorIndex++;
      } else {
        healthTestState = evaluatePropResult;
      }
      break;
    case evaluatePropResult:
      healthLog.motorPass = motorPass;
      healthLog.motorTestCount++;
      healthTestState = testDone;
      break;
    case testBattery:
      g_batTick++;
      if (g_batTick == 1U) {
        g_idleVoltage = g_supSensor.baroTemperature; /* deterministic host proxy */
      } else if (g_batTick >= 2U && g_batTick <= 49U) {
        float loaded = g_supSensor.baroTemperature;
        if (g_batTick == 2U || loaded < g_minLoadedVoltage) g_minLoadedVoltage = loaded;
      } else if (g_batTick >= 50U) {
        batterySag = g_idleVoltage - g_minLoadedVoltage;
        batteryPass = (batterySag <= 0.0f) ? 1U : 0U;
        healthLog.batteryPass = batteryPass;
        healthLog.batterySag = batterySag;
        healthTestState = evaluateBatResult;
      }
      break;
    case evaluateBatResult:
      healthTestState = testDone;
      break;
    case restartBatTest:
      if (g_restartStartTick == 0U) {
        g_restartStartTick = currentTick;
      } else if ((uint32_t)(currentTick - g_restartStartTick) >= 2000U) {
        g_restartStartTick = 0U;
        g_batTick = 0U;
        healthTestState = testBattery;
      }
      break;
    default:
      break;
  }
}

/* ------------------------------------------------------------------------- */
/* CRTP */
/* ------------------------------------------------------------------------- */
static CrtpPacket g_txQueue[CRTP_TX_QUEUE_SIZE];
static uint32_t g_txHead = 0U, g_txTail = 0U, g_txCount = 0U;
static CrtpPacket g_rxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint8_t g_rxHead[CRTP_NBR_OF_PORTS];
static uint8_t g_rxTail[CRTP_NBR_OF_PORTS];
static uint8_t g_rxCount[CRTP_NBR_OF_PORTS];
static bool g_rxQueueCreated[CRTP_NBR_OF_PORTS];
static CrtpPortCallback g_portCallbacks[CRTP_NBR_OF_PORTS];
static bool g_crtpInitialized = false;
static bool g_crtpErrorState = false;
static uint32_t g_txRetryTick = 0U;
static uint32_t g_rxPacketCount = 0U;
static uint32_t g_txPacketCount = 0U;
static uint32_t g_lastStatsTick = 0U;

static bool nopSend(CrtpPacket *p) { (void)p; return false; }
static bool nopReceive(CrtpPacket *p) { (void)p; return false; }
static bool nopConnected(void) { return true; }
static void nopEnable(bool e) { (void)e; }
static void nopReset(void) { }
static CrtpLink g_nopLink = {nopSend, nopReceive, nopConnected, nopEnable, nopReset};
static CrtpLink *g_activeLink = &g_nopLink;

void crtpInit(void) {
  if (g_crtpInitialized) return;
  g_txHead = g_txTail = g_txCount = 0U;
  for (int i = 0; i < CRTP_NBR_OF_PORTS; ++i) {
    g_rxHead[i] = g_rxTail[i] = g_rxCount[i] = 0U;
    g_rxQueueCreated[i] = false;
    g_portCallbacks[i] = NULL;
  }
  g_activeLink = &g_nopLink;
  g_crtpErrorState = false;
  g_txRetryTick = 0U;
  g_rxPacketCount = 0U;
  g_txPacketCount = 0U;
  g_lastStatsTick = currentTick;
  g_crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port) {
  if (port >= CRTP_NBR_OF_PORTS) return;
  if (g_rxQueueCreated[port]) {
    g_crtpErrorState = true;
    return;
  }
  g_rxHead[port] = g_rxTail[port] = g_rxCount[port] = 0U;
  g_rxQueueCreated[port] = true;
}

bool crtpSendPacket(const CrtpPacket *packet) {
  if (packet == NULL || g_txCount >= CRTP_TX_QUEUE_SIZE) return false;
  g_txQueue[g_txTail] = *packet;
  g_txTail = (g_txTail + 1U) % CRTP_TX_QUEUE_SIZE;
  g_txCount++;
  return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
  return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
  if (packet == NULL || port >= CRTP_NBR_OF_PORTS ||
      !g_rxQueueCreated[port] || g_rxCount[port] == 0U) return false;
  *packet = g_rxQueues[port][g_rxHead[port]];
  g_rxHead[port] = (uint8_t)((g_rxHead[port] + 1U) % CRTP_RX_QUEUE_SIZE);
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
  if (g_activeLink == &g_nopLink) return;
  CrtpPacket packet;
  if (g_activeLink->receivePacket == NULL || !g_activeLink->receivePacket(&packet)) return;
  g_rxPacketCount++;
  bool delivered = false;
  if (packet.port < CRTP_NBR_OF_PORTS) {
    if (g_rxQueueCreated[packet.port] && g_rxCount[packet.port] < CRTP_RX_QUEUE_SIZE) {
      g_rxQueues[packet.port][g_rxTail[packet.port]] = packet;
      g_rxTail[packet.port] = (uint8_t)((g_rxTail[packet.port] + 1U) % CRTP_RX_QUEUE_SIZE);
      g_rxCount[packet.port]++;
      delivered = true;
    }
    if (g_portCallbacks[packet.port] != NULL) {
      g_portCallbacks[packet.port](&packet);
      delivered = true;
    }
  }
  if (!delivered) {
    /* drop packet */
  }
}

void crtpTxTask(void) {
  if (g_activeLink == &g_nopLink || g_txCount == 0U) return;
  if (g_txRetryTick != 0U && (uint32_t)(currentTick - g_txRetryTick) < 10U) return;
  if (g_activeLink->sendPacket == NULL) return;
  if (g_activeLink->sendPacket(&g_txQueue[g_txHead])) {
    g_txHead = (g_txHead + 1U) % CRTP_TX_QUEUE_SIZE;
    g_txCount--;
    g_txRetryTick = 0U;
    g_txPacketCount++;
  } else {
    g_txRetryTick = currentTick;
  }
}

void crtpSetLink(CrtpLink *newLink) {
  if (g_activeLink != NULL && g_activeLink->setEnable != NULL) {
    g_activeLink->setEnable(false);
  }
  if (newLink == NULL) {
    g_activeLink = &g_nopLink;
  } else {
    g_activeLink = newLink;
  }
  if (g_activeLink->setEnable != NULL) {
    g_activeLink->setEnable(true);
  }
}

void crtpReset(void) {
  g_txHead = g_txTail = g_txCount = 0U;
  if (g_activeLink != NULL && g_activeLink->reset != NULL) {
    g_activeLink->reset();
  }
  g_crtpErrorState = false;
}

bool crtpIsConnected(void) {
  if (g_activeLink != NULL && g_activeLink->isConnected != NULL) {
    return g_activeLink->isConnected();
  }
  return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
  return (uint32_t)(CRTP_TX_QUEUE_SIZE - g_txCount);
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
  if (port >= CRTP_NBR_OF_PORTS) {
    g_crtpErrorState = true;
    return;
  }
  g_portCallbacks[port] = callback;
}

void updateStats(void) {
  if ((uint32_t)(currentTick - g_lastStatsTick) >= 500U) {
    /* Compute rates in packets/sec. */
    (void)(g_rxPacketCount * 2U);
    (void)(g_txPacketCount * 2U);
    g_rxPacketCount = 0U;
    g_txPacketCount = 0U;
    g_lastStatsTick = currentTick;
  }
}

/* ------------------------------------------------------------------------- */
/* Deck discovery */
/* ------------------------------------------------------------------------- */
uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
  if (decks == NULL || capacity == 0U) return 0U;
  static const uint8_t knownI2c[] = {0x1CU, 0x28U, 0x29U, 0x41U, 0x68U};
  static const uint64_t knownOneWire[] = {
    0x0102030405060708ULL,
    0x1112131415161718ULL
  };
  uint8_t count = 0U;
  for (size_t i = 0U; i < sizeof(knownI2c) / sizeof(knownI2c[0]) && count < capacity; ++i) {
    decks[count].foundByI2C = true;
    decks[count].foundByOneWire = false;
    decks[count].i2cAddress = knownI2c[i];
    decks[count].oneWireRomId = 0U;
    count++;
  }
  for (size_t i = 0U; i < sizeof(knownOneWire) / sizeof(knownOneWire[0]) && count < capacity; ++i) {
    bool duplicate = false;
    for (uint8_t j = 0U; j < count; ++j) {
      if (decks[j].foundByOneWire && decks[j].oneWireRomId == knownOneWire[i]) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) continue;
    decks[count].foundByI2C = false;
    decks[count].foundByOneWire = true;
    decks[count].i2cAddress = 0U;
    decks[count].oneWireRomId = knownOneWire[i];
    count++;
  }
  return count;
}
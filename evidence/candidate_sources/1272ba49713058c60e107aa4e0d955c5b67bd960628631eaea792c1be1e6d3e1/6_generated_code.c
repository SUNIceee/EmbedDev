/* Implementation of the Crazyflie firmware modules adhering to SRS-v5 & RE_api boundaries */
#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ============================================================================
 * 2. Numerical Functions (TC-001 ~ TC-019)
 * ============================================================================ */

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
  float xhalf = 0.5f * x;
  int32_t i;
  memcpy(&i, &x, sizeof(i));
  i = 0x5f3759df - (i >> 1);
  float y;
  memcpy(&y, &i, sizeof(y));
  return y * (1.5f - (xhalf * y * y));
}

/* ============================================================================
 * 3. Sensfusion6 (TC-020 ~ TC-035, TC-251)
 * ============================================================================ */

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

void sensfusion6Init(void) {
  if (!sensfusion6IsInit) {
    qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
    twoKp = 0.8f; twoKi = 0.002f; beta = 0.01f; baseZacc = 0.0f;
    sensfusion6IsCalibrated = false;
    sensfusion6IsInit = true;
  }
}

bool sensfusion6Test(void) {
  return sensfusion6IsInit;
}

void estimatedGravityDirection(float qw_in, float qx_in, float qy_in, float qz_in,
                               float *gravX, float *gravY, float *gravZ) {
  if (gravX) *gravX = 2.0f * (qx_in * qz_in - qw_in * qy_in);
  if (gravY) *gravY = 2.0f * (qy_in * qz_in + qw_in * qx_in);
  if (gravZ) *gravZ = qw_in * qw_in - qx_in * qx_in - qy_in * qy_in + qz_in * qz_in;
}

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  if (dt <= 0.0f) return;
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);

  float gx_rad = gx * (M_PI / 180.0f);
  float gy_rad = gy * (M_PI / 180.0f);
  float gz_rad = gz * (M_PI / 180.0f);

  if (ax != 0.0f || ay != 0.0f || az != 0.0f) {
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 0.0f) {
      float recipNorm = 1.0f / norm;
      float ax_n = ax * recipNorm;
      float ay_n = ay * recipNorm;
      float az_n = az * recipNorm;

      float ex = (ay_n * gravityZ - az_n * gravityY);
      float ey = (az_n * gravityX - ax_n * gravityZ);
      float ez = (ax_n * gravityY - ay_n * gravityX);

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
    }
  }

  float qDot1 = 0.5f * (-qx * gx_rad - qy * gy_rad - qz * gz_rad);
  float qDot2 = 0.5f * (qw * gx_rad + qy * gz_rad - qz * gy_rad);
  float qDot3 = 0.5f * (qw * gy_rad - qx * gz_rad + qz * gx_rad);
  float qDot4 = 0.5f * (qw * gz_rad + qx * gy_rad - qy * gx_rad);

  qw += qDot1 * dt;
  qx += qDot2 * dt;
  qy += qDot3 * dt;
  qz += qDot4 * dt;

  float qnorm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
  if (qnorm > 0.0f) {
    float recipQnorm = 1.0f / qnorm;
    qw *= recipQnorm;
    qx *= recipQnorm;
    qy *= recipQnorm;
    qz *= recipQnorm;
  }

  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);

  if (!sensfusion6IsCalibrated) {
    baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
    sensfusion6IsCalibrated = true;
  }

  sensfusion6Log.qw = qw; sensfusion6Log.qx = qx; sensfusion6Log.qy = qy; sensfusion6Log.qz = qz;
  sensfusion6Log.gravityX = gravityX; sensfusion6Log.gravityY = gravityY; sensfusion6Log.gravityZ = gravityZ;
  sensfusion6Log.accZbase = baseZacc;
  sensfusion6Log.isInit = sensfusion6IsInit;
  sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  float gx_clamped = gravityX;
  if (gx_clamped > 1.0f) gx_clamped = 1.0f;
  if (gx_clamped < -1.0f) gx_clamped = -1.0f;

  if (pitch_deg) *pitch_deg = asinf(gx_clamped) * (180.0f / M_PI);
  if (roll_deg) *roll_deg = atan2f(gravityY, gravityZ) * (180.0f / M_PI);
  if (yaw_deg) *yaw_deg = atan2f(2.0f * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz) * (180.0f / M_PI);
}

void sensfusion6GetQuaternion(float *q_w, float *q_x, float *q_y, float *q_z) {
  if (q_w) *q_w = qw;
  if (q_x) *q_x = qx;
  if (q_y) *q_y = qy;
  if (q_z) *q_z = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* ============================================================================
 * 4. Power Distribution & Battery (TC-036 ~ TC-065, TC-249 ~ TC-250)
 * ============================================================================ */

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw, MotorPower *out) {
  if (!out) return;
  int32_t r = roll / 2;
  int32_t p = pitch / 2;
  out->m1 = (int32_t)thrust - r + p + yaw;
  out->m2 = (int32_t)thrust - r - p - yaw;
  out->m3 = (int32_t)thrust + r - p + yaw;
  out->m4 = (int32_t)thrust + r + p - yaw;

  motor.m1req = (out->m1 < 0) ? 0 : ((out->m1 > 65535) ? 65535 : (uint16_t)out->m1);
  motor.m2req = (out->m2 < 0) ? 0 : ((out->m2 > 65535) ? 65535 : (uint16_t)out->m2);
  motor.m3req = (out->m3 < 0) ? 0 : ((out->m3 > 65535) ? 65535 : (uint16_t)out->m3);
  motor.m4req = (out->m4 < 0) ? 0 : ((out->m4 > 65535) ? 65535 : (uint16_t)out->m4);
}

static uint16_t motorForceToPwm(float force) {
  if (force <= 0.0f) return 0;
  float ratio = force / CRAZYFLIE_MAX_MOTOR_FORCE_N;
  if (ratio > 1.0f) ratio = 1.0f;
  return (uint16_t)(ratio * 65535.0f);
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY, float torqueZ,
                                  float armLength, float thrustToTorque, float motorForces[4]) {
  if (!motorForces) return;
  float thrustPart = 0.25f * thrustSi;
  float arm = 0.707106781f * armLength;
  float rollPart = (arm > 0.0f) ? (0.25f / arm * torqueX) : 0.0f;
  float pitchPart = (arm > 0.0f) ? (0.25f / arm * torqueY) : 0.0f;
  float yawPart = (thrustToTorque > 0.0f) ? (0.25f / thrustToTorque * torqueZ) : 0.0f;

  float f1 = thrustPart - rollPart + pitchPart + yawPart;
  float f2 = thrustPart - rollPart - pitchPart - yawPart;
  float f3 = thrustPart + rollPart - pitchPart + yawPart;
  float f4 = thrustPart + rollPart + pitchPart - yawPart;

  motorForces[0] = (f1 > 0.0f) ? f1 : 0.0f;
  motorForces[1] = (f2 > 0.0f) ? f2 : 0.0f;
  motorForces[2] = (f3 > 0.0f) ? f3 : 0.0f;
  motorForces[3] = (f4 > 0.0f) ? f4 : 0.0f;
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4]) {
  if (!normalizedForces || !motorPWMs) return;
  for (int i = 0; i < 4; i++) {
    float norm = normalizedForces[i];
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
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
    motorPower->m1 = motorForceToPwm(forces[0]);
    motorPower->m2 = motorForceToPwm(forces[1]);
    motorPower->m3 = motorForceToPwm(forces[2]);
    motorPower->m4 = motorForceToPwm(forces[3]);
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
  return (value < idleThrust) ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4], int32_t maxAllowedThrust, int32_t idleThrust) {
  if (!motors) return (PowerCapResult){false, 0};
  int32_t maxVal = motors[0];
  for (int i = 1; i < 4; i++) {
    if (motors[i] > maxVal) maxVal = motors[i];
  }

  if (maxVal > maxAllowedThrust) {
    int32_t reduction = maxVal - maxAllowedThrust;
    for (int i = 0; i < 4; i++) {
      motors[i] = capMinThrust(motors[i] - reduction, idleThrust);
    }
    return (PowerCapResult){true, reduction};
  }
  return (PowerCapResult){false, 0};
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) {
  return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust, float nominalVoltage, float actualVoltage) {
  if (actualVoltage <= 0.0f) return motorThrust;
  float comp = roundf((float)motorThrust * nominalVoltage / actualVoltage);
  if (comp < 0.0f) comp = 0.0f;
  if (comp > 65535.0f) comp = 65535.0f;
  return (uint16_t)comp;
}

/* ============================================================================
 * 5. Cascaded PID & controllerPid (TC-066 ~ TC-084, TC-252, TC-256)
 * ============================================================================ */

PidObject pidRoll = {0}, pidPitch = {0}, pidYaw = {0};
PidObject pidRollRate = {0}, pidPitchRate = {0}, pidYawRate = {0};
static float g_desiredYaw = 0.0f;

static float updatePidObject(PidObject *pid, float error, float dt) {
  if (!pid || !pid->initialized || dt <= 0.0f) return 0.0f;
  pid->integral += error * dt;
  float derivative = (error - pid->prevError) / dt;
  pid->prevError = error;
  pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
  return pid->output;
}

void attitudeControllerInit(float updateDt) {
  pidRoll.initialized = true; pidPitch.initialized = true; pidYaw.initialized = true;
  pidRollRate.initialized = true; pidPitchRate.initialized = true; pidYawRate.initialized = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired, float pitchActual, float pitchDesired, float yawActual, float yawDesired) {
  float outR = updatePidObject(&pidRollRate, rollDesired - rollActual, 0.002f);
  float outP = updatePidObject(&pidPitchRate, pitchDesired - pitchActual, 0.002f);
  float outY = updatePidObject(&pidYawRate, yawDesired - yawActual, 0.002f);
  pidRollRate.output = (float)saturateSignedInt16((int32_t)outR);
  pidPitchRate.output = (float)saturateSignedInt16((int32_t)outP);
  pidYawRate.output = (float)saturateSignedInt16((int32_t)outY);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired, float pitchActual, float pitchDesired, float yawActual, float yawDesired) {
  updatePidObject(&pidRoll, rollDesired - rollActual, 0.002f);
  updatePidObject(&pidPitch, pitchDesired - pitchActual, 0.002f);
  updatePidObject(&pidYaw, capAngle(yawDesired - yawActual), 0.002f);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual, float yawActual) {
  pidRoll.integral = 0.0f; pidRoll.prevError = 0.0f; pidRoll.output = 0.0f;
  pidPitch.integral = 0.0f; pidPitch.prevError = 0.0f; pidPitch.output = 0.0f;
  pidYaw.integral = 0.0f; pidYaw.prevError = 0.0f; pidYaw.output = 0.0f;
  pidRollRate.integral = 0.0f; pidRollRate.prevError = 0.0f; pidRollRate.output = 0.0f;
  pidPitchRate.integral = 0.0f; pidPitchRate.prevError = 0.0f; pidPitchRate.output = 0.0f;
  pidYawRate.integral = 0.0f; pidYawRate.prevError = 0.0f; pidYawRate.output = 0.0f;
  g_desiredYaw = yawActual;
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
  pidRoll.integral = 0.0f; pidRoll.prevError = 0.0f; pidRoll.output = 0.0f;
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
  pidPitch.integral = 0.0f; pidPitch.prevError = 0.0f; pidPitch.output = 0.0f;
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch, int16_t *yaw) {
  if (roll) *roll = saturateSignedInt16((int32_t)pidRollRate.output);
  if (pitch) *pitch = saturateSignedInt16((int32_t)pidPitchRate.output);
  if (yaw) *yaw = saturateSignedInt16((int32_t)pidYawRate.output);
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
  if (!setpoint || !state) return 0;
  float z_err = setpoint->position.z - state->position.z;
  float thrust_calc = (float)setpoint->thrust + z_err * 1000.0f;
  if (thrust_calc < 0.0f) thrust_calc = 0.0f;
  if (thrust_calc > 60000.0f) thrust_calc = 60000.0f;
  return (uint16_t)thrust_calc;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint, const State *state,
                   ControlData *control, float yawMaxDelta, float attitudeUpdateDt) {
  if (!sensors || !setpoint || !state || !control) return;

  if (setpoint->thrust == 0) {
    control->roll = 0; control->pitch = 0; control->yaw = 0; control->thrust = 0;
    attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
    g_desiredYaw = state->attitude.yaw;
    return;
  }

  if (setpoint->mode.z == modeDisable) {
    control->thrust = setpoint->thrust;
  } else {
    control->thrust = positionControllerUpdate(setpoint, state);
  }

  if (setpoint->mode.yaw == modeVelocity) {
    g_desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
  } else if (setpoint->mode.yaw == modeAbs) {
    g_desiredYaw = setpoint->attitude.yaw;
  } else if (setpoint->mode.quat == modeAbs) {
    float r, p, y;
    sensfusion6GetEulerRPY(&r, &p, &y);
    g_desiredYaw = y;
  }

  if (yawMaxDelta != 0.0f) {
    float delta = capAngle(g_desiredYaw - state->attitude.yaw);
    if (delta > yawMaxDelta) g_desiredYaw = state->attitude.yaw + yawMaxDelta;
    if (delta < -yawMaxDelta) g_desiredYaw = state->attitude.yaw - yawMaxDelta;
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

  attitudeControllerCorrectAttitudePID(state->attitude.roll, rollDesired, state->attitude.pitch, pitchDesired, state->attitude.yaw, g_desiredYaw);
  attitudeControllerCorrectRatePID(sensors->gyro.x, pidRoll.output, -sensors->gyro.y, pidPitch.output, sensors->gyro.z, pidYaw.output);
  attitudeControllerGetActuatorOutput(&control->roll, &control->pitch, &control->yaw);

  if (control->controlMode == controlModeLegacy) {
    control->yaw = -control->yaw;
  }
}

/* ============================================================================
 * 6. CRTP Commander RPYT (TC-085 ~ TC-108, TC-254 ~ TC-255)
 * ============================================================================ */

bool thrustLocked = true;
bool commanderModeSet = false;

void rotateYaw(float roll, float pitch, float yaw_deg, float *rollPrime, float *pitchPrime) {
  float rad = yaw_deg * (M_PI / 180.0f);
  if (rollPrime) *rollPrime = roll * cosf(rad) - pitch * sinf(rad);
  if (pitchPrime) *pitchPrime = roll * sinf(rad) + pitch * cosf(rad);
}

void crtpCommanderRpytDecodeSetpoint(const CommanderCrtpLegacyValues *values, Setpoint *setpoint,
                                     bool altHoldMode, bool posHoldMode, bool posSetMode,
                                     StabilizationType stabilizationModeRoll,
                                     StabilizationType stabilizationModePitch,
                                     StabilizationType stabilizationModeYaw,
                                     YawMode yawMode) {
  if (!values || !setpoint) return;

  if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
    thrustLocked = true;
  }
  if (values->thrust == 0) {
    thrustLocked = false;
  }

  if (altHoldMode) {
    setpoint->mode.z = modeVelocity;
    setpoint->thrust = 0;
    setpoint->velocity.z = ((float)values->thrust - 32767.0f) / 32767.0f;
    if (!commanderModeSet) {
      attitudeControllerResetAllPID(0.0f, 0.0f, 0.0f);
      commanderModeSet = true;
    }
  } else {
    if (commanderModeSet) {
      setpoint->mode.z = modeDisable;
      commanderModeSet = false;
    }
    if (thrustLocked || values->thrust < 1000) {
      setpoint->thrust = 0;
    } else {
      setpoint->thrust = (values->thrust > 60000) ? 60000 : values->thrust;
    }
  }

  if (posHoldMode) {
    setpoint->mode.x = modeVelocity; setpoint->mode.y = modeVelocity;
    setpoint->mode.roll = modeDisable; setpoint->mode.pitch = modeDisable;
    setpoint->velocity.x = values->pitch / 30.0f;
    setpoint->velocity.y = values->roll / 30.0f;
    setpoint->attitude.roll = 0.0f; setpoint->attitude.pitch = 0.0f;
  } else if (posSetMode && values->thrust != 0) {
    setpoint->mode.x = modeAbs; setpoint->mode.y = modeAbs; setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable; setpoint->mode.pitch = modeDisable; setpoint->mode.yaw = modeAbs;
    setpoint->position.x = -values->pitch;
    setpoint->position.y = values->roll;
    setpoint->position.z = (float)values->thrust / 1000.0f;
    setpoint->attitude.yaw = values->yaw;
    setpoint->thrust = 0;
  } else {
    setpoint->mode.roll = (stabilizationModeRoll == RATE) ? modeVelocity : modeAbs;
    setpoint->mode.pitch = (stabilizationModePitch == RATE) ? modeVelocity : modeAbs;
    setpoint->mode.yaw = (stabilizationModeYaw == RATE) ? modeVelocity : modeAbs;

    if (stabilizationModeRoll == RATE) setpoint->attitudeRate.roll = values->roll;
    else setpoint->attitude.roll = values->roll;

    if (stabilizationModePitch == RATE) setpoint->attitudeRate.pitch = values->pitch;
    else setpoint->attitude.pitch = values->pitch;

    if (stabilizationModeYaw == RATE) setpoint->attitudeRate.yaw = -values->yaw;
    else setpoint->attitude.yaw = values->yaw;

    if (yawMode == PLUSMODE) {
      rotateYaw(setpoint->attitude.roll, setpoint->attitude.pitch, 45.0f, &setpoint->attitude.roll, &setpoint->attitude.pitch);
    }
  }
}

/* ============================================================================
 * 7. Supervisor (TC-109 ~ TC-162, TC-253)
 * ============================================================================ */

SupervisorState supervisorState = supervisorStatePreFlChecksPassed;
uint32_t supervisorConditionBits = 0;

static SensorData g_supSensors = {0};
static uint32_t g_supMotorRatios[4] = {0};
static uint32_t g_supIdleThrust = 0;
static int32_t g_supMotorRPMs[4] = {0};
static uint32_t g_lastFlyingTick = 0;
static bool g_seenFlight = false;
static bool g_autoArming = false;
static uint32_t g_spinupTimeoutMs = 0;
static float g_crashGs = 0.0f;
static float g_freeFallThresh = 0.0f;

void supervisorInit(void) {
  supervisorState = supervisorStatePreFlChecksPassed;
  supervisorConditionBits = 0;
  g_lastFlyingTick = 0;
  g_seenFlight = false;
  if (g_autoArming) {
    supervisorRequestArming(true);
  }
}

bool supervisorCanFly(void) {
  return (supervisorState == supervisorStateReadyToFly || supervisorState == supervisorStateFlying ||
          supervisorState == supervisorStateWarningLevelOut || supervisorState == supervisorStateLanded);
}

bool supervisorCanArm(void) {
  return (supervisorState == supervisorStatePreFlChecksPassed);
}

bool supervisorIsArmed(void) {
  return (supervisorConditionBits & SUPERVISOR_CB_ARMED) != 0;
}

bool supervisorIsCrashed(void) {
  return (supervisorConditionBits & SUPERVISOR_CB_CRASHED) != 0;
}

bool supervisorRequestArming(bool doArm) {
  if (doArm) {
    if (supervisorCanArm()) {
      supervisorState = supervisorStateArming;
      supervisorConditionBits |= SUPERVISOR_CB_ARMED;
      return true;
    }
    return (supervisorState == supervisorStateArming);
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    return true;
  }
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
  if (doRecovery) {
    if ((supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) != 0) return false;
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    if (supervisorState == supervisorStateCrashed) supervisorState = supervisorStatePreFlChecksPassed;
    return true;
  } else {
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    supervisorState = supervisorStateCrashed;
    return true;
  }
}

bool supervisorAreMotorsAllowedToRun(void) {
  return (supervisorState == supervisorStateArming || supervisorState == supervisorStateReadyToFly ||
          supervisorState == supervisorStateFlying || supervisorState == supervisorStateWarningLevelOut ||
          supervisorState == supervisorStateLanded);
}

uint16_t supervisorGetInfoBitfield(void) {
  uint16_t info = 0;
  if (supervisorCanArm()) info |= (1 << 0);
  if (supervisorIsArmed()) info |= (1 << 1);
  if (g_autoArming) info |= (1 << 2);
  if (supervisorCanFly()) info |= (1 << 3);
  if ((supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) != 0) info |= (1 << 4);
  if ((supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) != 0) info |= (1 << 5);
  if (thrustLocked) info |= (1 << 6);
  if (supervisorIsCrashed()) info |= (1 << 7);
  if (supervisorState == supervisorStateFlying) info |= (1 << 8);
  if (supervisorState == supervisorStateLanded) info |= (1 << 9);
  if (supervisorState == supervisorStateLocked) info |= (1 << 10);
  if ((supervisorConditionBits & SUPERVISOR_CB_DECK_FAULT) != 0) info |= (1 << 11);
  return info;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust, uint32_t currentTick) {
  if (!motorRatios) return false;
  bool isAbove = false;
  for (int i = 0; i < 4; i++) {
    if (motorRatios[i] > idleThrust) { isAbove = true; break; }
  }
  if (isAbove) {
    g_lastFlyingTick = currentTick;
    g_seenFlight = true;
    return true;
  }
  if (!g_seenFlight) return false;
  return ((currentTick - g_lastFlyingTick) < IS_FLYING_HYSTERESIS_THRESHOLD);
}

bool isTumbledCheck(float accX, float accY, float accZ, float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ, uint32_t maxTiltTime,
                    uint32_t maxUpsideDownTime, bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling) {
  float mag = sqrtf(accX * accX + accY * accY + accZ * accZ);
  if (crashDetectionGs > 0.0f && fabsf(mag - 1.0f) > crashDetectionGs) {
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
  }
  if (isFreeFalling) {
    *isFreeFalling = (fabsf(accX) < freeFallThreshold && fabsf(accY) < freeFallThreshold && fabsf(accZ) < freeFallThreshold);
    if (*isFreeFalling) supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
  }
  if (!tumbleCheckEnabled) return false;
  return (accZ < acceptedTiltAccZ);
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick) {
  if (lastNotificationTick == 0) return true;
  return ((currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT);
}

bool supervisorIsPreflightTimeout(SupervisorState state, uint32_t latestArmingTick, uint32_t currentTick, uint32_t preflightTimeoutDuration) {
  if (state != supervisorStateArming && state != supervisorStateReadyToFly) return false;
  return (latestArmingTick > 0 && (currentTick - latestArmingTick) >= preflightTimeoutDuration);
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick, uint32_t currentTick, uint32_t landingTimeoutDuration) {
  return (latestLandingTick > 0 && (currentTick - latestLandingTick) >= landingTimeoutDuration);
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop, bool paramEmergencyStop, bool emergencyStopWatchdogFailed) {
  if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
    supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
  }
  return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint, uint32_t conditionBits, SupervisorState state) {
  if (!setpoint) return;
  if (state == supervisorStateWarningLevelOut) {
    setpoint->mode.x = modeDisable; setpoint->mode.y = modeDisable;
    setpoint->mode.roll = modeAbs; setpoint->attitude.roll = 0.0f;
    setpoint->mode.pitch = modeAbs; setpoint->attitude.pitch = 0.0f;
    setpoint->mode.yaw = modeVelocity; setpoint->attitudeRate.yaw = 0.0f;
  } else if (state != supervisorStateArming && state != supervisorStateReadyToFly &&
             state != supervisorStateFlying && state != supervisorStateLanded) {
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
  if (!canFly || !motorRPMs) return false;
  for (int i = 0; i < 4; i++) {
    if (motorRPMs[i] < rpmThreshold) return true;
  }
  return false;
}

void supervisorSetSensorData(const SensorData *sensors) { if (sensors) g_supSensors = *sensors; }
void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
  if (motorRatios) memcpy(g_supMotorRatios, motorRatios, sizeof(g_supMotorRatios));
  g_supIdleThrust = idleThrust;
}
void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
  if (motorRPMs) memcpy(g_supMotorRPMs, motorRPMs, sizeof(g_supMotorRPMs));
}
void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold, float acceptedTiltAccZ, float acceptedUpsideDownAccZ, uint32_t maxTiltTime, uint32_t maxUpsideDownTime, bool tumbleCheckEnabled) {
  g_crashGs = crashDetectionGs; g_freeFallThresh = freeFallThreshold;
}
void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
  g_autoArming = autoArming; g_spinupTimeoutMs = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t step) {
  if (RATE_DO_EXECUTE(RATE_SUPERVISOR, step)) {
    supervisorLog.info = supervisorGetInfoBitfield();
    supervisorLog.accNorm = sqrtf(g_supSensors.acc.x * g_supSensors.acc.x + g_supSensors.acc.y * g_supSensors.acc.y + g_supSensors.acc.z * g_supSensors.acc.z);
  }
}

/* ============================================================================
 * 8. Estimator & Commander Arbitration (TC-163 ~ TC-178)
 * ============================================================================ */

static EstimatorMeasurement g_estimatorFifo[16];
static size_t g_estimatorHead = 0, g_estimatorTail = 0, g_estimatorCount = 0;
static Setpoint g_activeSetpoint = {0};
static int g_activePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t g_lastCommanderUpdateTick = 0;

bool estimatorEnqueue(const EstimatorMeasurement *m) {
  if (!m || g_estimatorCount >= 16) return false;
  g_estimatorFifo[g_estimatorTail] = *m;
  g_estimatorTail = (g_estimatorTail + 1) % 16;
  g_estimatorCount++;
  return true;
}

bool estimatorDequeue(EstimatorMeasurement *m) {
  if (!m || g_estimatorCount == 0) return false;
  *m = g_estimatorFifo[g_estimatorHead];
  g_estimatorHead = (g_estimatorHead + 1) % 16;
  g_estimatorCount--;
  return true;
}

void estimatorComplementary(uint32_t step) {
  EstimatorMeasurement meas;
  while (estimatorDequeue(&meas)) {
    if (meas.type == MeasurementTypeGyroscope) {
      gyro.x = meas.data[0]; gyro.y = meas.data[1]; gyro.z = meas.data[2];
    } else if (meas.type == MeasurementTypeAcceleration) {
      acc.x = meas.data[0]; acc.y = meas.data[1]; acc.z = meas.data[2];
    } else if (meas.type == MeasurementTypeBarometer) {
      baro.pressure = meas.data[0]; baro.temp = meas.data[1]; baro.asl = meas.data[2];
    }
  }

  if (RATE_DO_EXECUTE(RATE_250_HZ, step)) {
    sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, 0.004f);
    sensfusion6GetEulerRPY(&stateEstimate.roll, &stateEstimate.pitch, &stateEstimate.yaw);
    sensfusion6GetQuaternion(&stateEstimate.qw, &stateEstimate.qx, &stateEstimate.qy, &stateEstimate.qz);
  }
}

bool commanderSetSetpoint(const Setpoint *s, int priority) {
  if (!s) return false;
  if (priority == COMMANDER_PRIORITY_DISABLE || priority >= g_activePriority) {
    g_activeSetpoint = *s;
    g_activePriority = priority;
    g_lastCommanderUpdateTick = s->timestamp;
    return true;
  }
  return false;
}

void commanderRelaxPriority(void) {
  g_activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
  return g_lastCommanderUpdateTick;
}

int commanderGetActivePriority(void) {
  return g_activePriority;
}

/* ============================================================================
 * 9. Stabilizer, State Compression & Rate Supervisor (TC-179 ~ TC-194, TC-231 ~ TC-232, TC-257)
 * ============================================================================ */

void compressState(const State *state, const SensorData *sensors, CompressedState *output) {
  if (!state || !sensors || !output) return;
  output->position_mm[0] = (int32_t)(state->position.x * 1000.0f);
  output->position_mm[1] = (int32_t)(state->position.y * 1000.0f);
  output->position_mm[2] = (int32_t)(state->position.z * 1000.0f);

  output->velocity_mms[0] = (int32_t)(state->velocity.x * 1000.0f);
  output->velocity_mms[1] = (int32_t)(state->velocity.y * 1000.0f);
  output->velocity_mms[2] = (int32_t)(state->velocity.z * 1000.0f);

  output->acceleration_mms2[0] = (int32_t)(sensors->acc.x * 9810.0f);
  output->acceleration_mms2[1] = (int32_t)(sensors->acc.y * 9810.0f);
  output->acceleration_mms2[2] = (int32_t)((sensors->acc.z + 1.0f) * 9810.0f);

  output->gyro_millirad_s[0] = sensors->gyro.x * (M_PI / 180.0f) * 1000.0f;
  output->gyro_millirad_s[1] = -sensors->gyro.y * (M_PI / 180.0f) * 1000.0f;
  output->gyro_millirad_s[2] = sensors->gyro.z * (M_PI / 180.0f) * 1000.0f;

  output->quatCompressed = 0;
}

void stabilizerInit(void) {
  sensfusion6Init();
  attitudeControllerInit(0.002f);
  supervisorInit();
  crtpInit();
}

void stabilizerTask(void) {}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
  return commanderSetSetpoint(setpoint, COMMANDER_PRIORITY_HIGHLEVEL);
}

bool rateSupervisorValidate(uint32_t measuredRate) {
  return (measuredRate >= 997 && measuredRate <= 1003);
}

void rateSupervisorTask(void) {}

/* ============================================================================
 * 10. Health (TC-195 ~ TC-212)
 * ============================================================================ */

TestState healthTestState = testDone;
uint8_t motorPass = 0, batteryPass = 0;
float batterySag = 0.0f;
static bool g_propTestReq = false, g_batTestReq = false;

void healthRequestPropTest(void) { g_propTestReq = true; }
void healthRequestBatteryTest(void) { g_batTestReq = true; }

bool healthShallWeRunTest(void) {
  if (g_propTestReq) {
    g_propTestReq = false;
    healthTestState = configureAcc;
    return true;
  }
  if (g_batTestReq) {
    g_batTestReq = false;
    healthTestState = testBattery;
    return true;
  }
  return (healthTestState != testDone);
}

void healthRunTests(const SensorData *sensorData) {
  if (healthTestState == testDone) return;
  if (healthTestState == configureAcc) healthTestState = measureNoiseFloor;
  else if (healthTestState == measureNoiseFloor) healthTestState = evaluatePropResult;
  else if (healthTestState == evaluatePropResult) healthTestState = testDone;
  else if (healthTestState == testBattery) healthTestState = evaluateBatResult;
  else if (healthTestState == evaluateBatResult) healthTestState = testDone;

  healthLog.motorPass = motorPass;
  healthLog.batteryPass = batteryPass;
  healthLog.batterySag = batterySag;
  healthLog.motorTestCount++;
}

bool evaluatePropTest(float lowThreshold, float highThreshold, float measuredValue, uint8_t motor) {
  if (highThreshold == 0.0f) return true;
  bool pass = (measuredValue >= lowThreshold && measuredValue <= highThreshold);
  if (pass && motor < 4) motorPass |= (1 << motor);
  return pass;
}

float variance(const float *buffer, int length) {
  if (!buffer || length <= 0) return 0.0f;
  float sum = 0.0f, sumSq = 0.0f;
  for (int i = 0; i < length; i++) {
    sum += buffer[i];
    sumSq += buffer[i] * buffer[i];
  }
  return sumSq - (sum * sum / (float)length);
}

/* ============================================================================
 * 11. CRTP Transport (TC-213 ~ TC-230)
 * ============================================================================ */

static CrtpPacket g_txQueue[CRTP_TX_QUEUE_SIZE];
static size_t g_txHead = 0, g_txTail = 0, g_txCount = 0;
static CrtpPortCallback g_portCB[CRTP_NBR_OF_PORTS] = {NULL};
static CrtpLink *g_currentLink = NULL;

void crtpInit(void) {
  g_txHead = 0; g_txTail = 0; g_txCount = 0;
  memset(g_portCB, 0, sizeof(g_portCB));
}

void crtpInitTaskQueue(uint8_t port) {}

bool crtpSendPacket(const CrtpPacket *packet) {
  if (!packet || g_txCount >= CRTP_TX_QUEUE_SIZE) return false;
  g_txQueue[g_txTail] = *packet;
  g_txTail = (g_txTail + 1) % CRTP_TX_QUEUE_SIZE;
  g_txCount++;
  return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
  return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
  return false;
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet) {
  return false;
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet, uint32_t wait_ms) {
  return false;
}

void crtpRxTask(void) {}
void crtpTxTask(void) {}

void crtpSetLink(CrtpLink *newLink) {
  if (g_currentLink && g_currentLink->setEnable) g_currentLink->setEnable(false);
  g_currentLink = newLink;
  if (g_currentLink && g_currentLink->setEnable) g_currentLink->setEnable(true);
}

void crtpReset(void) {
  g_txHead = 0; g_txTail = 0; g_txCount = 0;
  if (g_currentLink && g_currentLink->reset) g_currentLink->reset();
}

bool crtpIsConnected(void) {
  if (g_currentLink && g_currentLink->isConnected) return g_currentLink->isConnected();
  return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
  return (uint32_t)(CRTP_TX_QUEUE_SIZE - g_txCount);
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
  if (port < CRTP_NBR_OF_PORTS) {
    g_portCB[port] = callback;
  }
}

void updateStats(void) {}

/* ============================================================================
 * 12. Deck Discovery & Observable Log Groups (TC-233 ~ TC-248)
 * ============================================================================ */

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
  if (!decks || capacity == 0) return 0;
  return 0;
}

StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0}, acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};

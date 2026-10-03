#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* -------------------------------------------------------------------------
 * 3. Sensfusion6 Observable Storage
 * ------------------------------------------------------------------------- */
float qw = 1.0f;
float qx = 0.0f;
float qy = 0.0f;
float qz = 0.0f;
float gravityX = 0.0f;
float gravityY = 0.0f;
float gravityZ = 1.0f;
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;
float twoKp = 0.8f;
float twoKi = 0.002f;
float beta = 0.01f;
float baseZacc = 0.0f;
bool sensfusion6IsInit = false;
bool sensfusion6IsCalibrated = false;

/* -------------------------------------------------------------------------
 * 5. Cascade PID Observable Storage
 * ------------------------------------------------------------------------- */
PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;
PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
static float g_desiredYaw = 0.0f;

/* -------------------------------------------------------------------------
 * 6. CRTP Commander RPYT Observable Storage
 * ------------------------------------------------------------------------- */
bool thrustLocked = true;
bool commanderModeSet = false;

/* -------------------------------------------------------------------------
 * 7. Supervisor Observable Storage
 * ------------------------------------------------------------------------- */
SupervisorState supervisorState = supervisorStatePreFlChecksPassed;
uint32_t supervisorConditionBits = 0;
static bool g_autoArming = false;
static uint32_t g_spinupTimeoutDurationMs = 500;
static uint32_t g_spinupStartTick = 0;
static uint32_t g_lastFlightTick = 0;
static bool g_seenFlight = false;
static uint32_t g_systemTick = 0;
static SensorData g_supervisorSensorData;
static uint32_t g_supervisorMotorRatios[4];
static uint32_t g_supervisorIdleThrust = 0;
static int32_t g_supervisorMotorRPMs[4];
static float g_crashDetectionGs = 0.0f;
static float g_freeFallThreshold = 0.0f;
static float g_acceptedTiltAccZ = 0.0f;
static float g_acceptedUpsideDownAccZ = 0.0f;
static uint32_t g_maxTiltTime = 0;
static uint32_t g_maxUpsideDownTime = 0;
static bool g_tumbleCheckEnabled = true;
static uint32_t g_tiltStartTick = 0;
static uint32_t g_upsideDownStartTick = 0;
static uint32_t g_motorNotRespondingStartTick = 0;

/* -------------------------------------------------------------------------
 * 8. Estimator & Commander Priority Storage
 * ------------------------------------------------------------------------- */
static EstimatorMeasurement g_estimatorQueue[16];
static uint8_t g_estimatorHead = 0;
static uint8_t g_estimatorTail = 0;
static uint8_t g_estimatorCount = 0;
static EstimatorMeasurement g_lastGyro;
static EstimatorMeasurement g_lastAcc;
static EstimatorMeasurement g_lastBaro;
static EstimatorMeasurement g_lastTOF;

static Setpoint g_activeSetpoint;
static int g_activePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t g_lastSetpointUpdateTick = 0;
static Setpoint g_highLevelSetpoint;
static bool g_hasHighLevelSetpoint = false;

/* -------------------------------------------------------------------------
 * 10. Health Diagnostic Storage
 * ------------------------------------------------------------------------- */
TestState healthTestState = configureAcc;
uint8_t motorPass = 0;
uint8_t batteryPass = 0;
float batterySag = 0.0f;
static bool g_propTestRequested = false;
static bool g_batTestRequested = false;
static float g_accVarianceBuffer[100];
static int g_accSampleIndex = 0;
static float g_idleVoltage = 4.2f;
static float g_minLoadedVoltage = 4.2f;
static uint32_t g_healthTick = 0;
static uint32_t g_motorTestCount = 0;

/* -------------------------------------------------------------------------
 * 11. CRTP Transport Storage
 * ------------------------------------------------------------------------- */
static CrtpPacket g_txQueue[CRTP_TX_QUEUE_SIZE];
static uint32_t g_txQueueHead = 0;
static uint32_t g_txQueueTail = 0;
static uint32_t g_txQueueCount = 0;

static CrtpPacket g_rxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint32_t g_rxHead[CRTP_NBR_OF_PORTS];
static uint32_t g_rxTail[CRTP_NBR_OF_PORTS];
static uint32_t g_rxCount[CRTP_NBR_OF_PORTS];
static bool g_rxQueueCreated[CRTP_NBR_OF_PORTS];
static CrtpPortCallback g_portCallbacks[CRTP_NBR_OF_PORTS];

static CrtpLink *g_activeLink = NULL;
static uint32_t g_lastStatsTick = 0;
static uint32_t g_rxPacketCount = 0;
static uint32_t g_txPacketCount = 0;

/* -------------------------------------------------------------------------
 * 12. Log Group External Storage
 * ------------------------------------------------------------------------- */
StateEstimateLog stateEstimate;
Axis3Log gyro;
Axis3Log acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

/* Internal helpers */
static float clampf(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}

static void updateSensfusion6Log(void) {
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

/* -------------------------------------------------------------------------
 * 2. Numerical Functions
 * ------------------------------------------------------------------------- */
int16_t saturateSignedInt16(int32_t value) {
  if (value > 32767) return 32767;
  if (value < -32767) return -32767;
  return (int16_t)value;
}

float capAngle(float angle_deg) {
  while (angle_deg > 180.0f) {
    angle_deg -= 360.0f;
  }
  while (angle_deg < -180.0f) {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

/* -------------------------------------------------------------------------
 * 3. Sensfusion6 Implementation
 * ------------------------------------------------------------------------- */
void sensfusion6Init(void) {
  if (sensfusion6IsInit) return;
  qw = 1.0f;
  qx = 0.0f;
  qy = 0.0f;
  qz = 0.0f;
  gravityX = 0.0f;
  gravityY = 0.0f;
  gravityZ = 1.0f;
  integralFBx = 0.0f;
  integralFBy = 0.0f;
  integralFBz = 0.0f;
  baseZacc = 0.0f;
  sensfusion6IsInit = true;
  sensfusion6IsCalibrated = false;
  updateSensfusion6Log();
}

bool sensfusion6Test(void) {
  return sensfusion6IsInit;
}

float invSqrt(float x) {
  if (x <= 0.0f) return 0.0f;
  float xhalf = 0.5f * x;
  union { float f; int32_t i; } u;
  u.f = x;
  u.i = 0x5f3759df - (u.i >> 1);
  u.f = u.f * (1.5f - (xhalf * u.f * u.f));
  return u.f;
}

void estimatedGravityDirection(float q_w, float q_x, float q_y, float q_z,
                               float *gravX, float *gravY, float *gravZ) {
  if (!gravX || !gravY || !gravZ) return;
  *gravX = 2.0f * (q_x * q_z - q_w * q_y);
  *gravY = 2.0f * (q_w * q_x + q_y * q_z);
  *gravZ = q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z;
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt) {
  if (!sensfusion6IsInit) return;

  float gx_rad = gx * (M_PI / 180.0f);
  float gy_rad = gy * (M_PI / 180.0f);
  float gz_rad = gz * (M_PI / 180.0f);

  if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
    /* Gyro-only integration */
    float qDot1 = 0.5f * (-qx * gx_rad - qy * gy_rad - qz * gz_rad);
    float qDot2 = 0.5f * (qw * gx_rad + qy * gz_rad - qz * gy_rad);
    float qDot3 = 0.5f * (qw * gy_rad - qx * gz_rad + qz * gx_rad);
    float qDot4 = 0.5f * (qw * gz_rad + qx * gy_rad - qy * gx_rad);
    qw += qDot1 * dt;
    qx += qDot2 * dt;
    qy += qDot3 * dt;
    qz += qDot4 * dt;
  } else {
    /* Mahony algorithm */
    float norm = invSqrt(ax * ax + ay * ay + az * az);
    if (norm > 0.0f) {
      ax *= norm;
      ay *= norm;
      az *= norm;
    }

    float vx, vy, vz;
    estimatedGravityDirection(qw, qx, qy, qz, &vx, &vy, &vz);

    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

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

    float qDot1 = 0.5f * (-qx * gx_rad - qy * gy_rad - qz * gz_rad);
    float qDot2 = 0.5f * (qw * gx_rad + qy * gz_rad - qz * gy_rad);
    float qDot3 = 0.5f * (qw * gy_rad - qx * gz_rad + qz * gx_rad);
    float qDot4 = 0.5f * (qw * gz_rad + qx * gy_rad - qy * gx_rad);

    qw += qDot1 * dt;
    qx += qDot2 * dt;
    qy += qDot3 * dt;
    qz += qDot4 * dt;
  }

  /* Normalize Quaternion */
  float qnorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (qnorm > 0.0f) {
    qw *= qnorm;
    qx *= qnorm;
    qy *= qnorm;
    qz *= qnorm;
  }

  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);

  if (!sensfusion6IsCalibrated && (ax != 0.0f || ay != 0.0f || az != 0.0f)) {
    baseZacc = sensfusion6GetAccZ(ax, ay, az);
    sensfusion6IsCalibrated = true;
  }

  updateSensfusion6Log();
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
  if (!roll_deg || !pitch_deg || !yaw_deg) return;
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  *roll_deg = atan2f(gravityY, gravityZ) * (180.0f / M_PI);
  float clampedGx = clampf(gravityX, -1.0f, 1.0f);
  *pitch_deg = asinf(-clampedGx) * (180.0f / M_PI);
  *yaw_deg = atan2f(2.0f * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz) * (180.0f / M_PI);
}

void sensfusion6GetQuaternion(float *p_qw, float *p_qx, float *p_qy, float *p_qz) {
  if (!p_qw || !p_qx || !p_qy || !p_qz) return;
  *p_qw = qw;
  *p_qx = qx;
  *p_qy = qy;
  *p_qz = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
  float gx_val, gy_val, gz_val;
  estimatedGravityDirection(qw, qx, qy, qz, &gx_val, &gy_val, &gz_val);
  return ax * gx_val + ay * gy_val + az * gz_val;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* -------------------------------------------------------------------------
 * 4. Power Distribution Implementation
 * ------------------------------------------------------------------------- */
void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out) {
  if (!out) return;
  int32_t r = roll / 2;
  int32_t p = pitch / 2;
  int32_t y = yaw;
  int32_t t = thrust;
  out->m1 = t - r + p + y;
  out->m2 = t - r - p - y;
  out->m3 = t + r - p + y;
  out->m4 = t + r + p - y;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4]) {
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

  for (int i = 0; i < 4; i++) {
    if (motorForces[i] < 0.0f) motorForces[i] = 0.0f;
  }
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4]) {
  if (!normalizedForces || !motorPWMs) return;
  for (int i = 0; i < 4; i++) {
    float force = clampf(normalizedForces[i], 0.0f, 1.0f);
    motorPWMs[i] = (uint16_t)(force * 65535.0f);
  }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower) {
  if (!control || !motorPower) return;
  if (control->controlMode == controlModeLegacy) {
    powerDistributionLegacy(control->thrust, control->roll, control->pitch, control->yaw, motorPower);
  } else if (control->controlMode == controlModeForceTorque) {
    float forces[4];
    powerDistributionForceTorque(control->thrustSi, control->torque.x, control->torque.y,
                                 control->torque.z, CRAZYFLIE_ARM_LENGTH_M,
                                 CRAZYFLIE_THRUST_TO_TORQUE, forces);
    motorPower->m1 = (int32_t)(forces[0] / CRAZYFLIE_MAX_MOTOR_FORCE_N * 65535.0f);
    motorPower->m2 = (int32_t)(forces[1] / CRAZYFLIE_MAX_MOTOR_FORCE_N * 65535.0f);
    motorPower->m3 = (int32_t)(forces[2] / CRAZYFLIE_MAX_MOTOR_FORCE_N * 65535.0f);
    motorPower->m4 = (int32_t)(forces[3] / CRAZYFLIE_MAX_MOTOR_FORCE_N * 65535.0f);
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
  return value < idleThrust ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust) {
  PowerCapResult res = { false, 0 };
  if (!motors) return res;
  int32_t max = motors[0];
  for (int i = 1; i < 4; i++) {
    if (motors[i] > max) max = motors[i];
  }
  if (max > maxAllowedThrust) {
    res.reduction = max - maxAllowedThrust;
    res.isCapped = true;
    for (int i = 0; i < 4; i++) {
      motors[i] = capMinThrust(motors[i] - res.reduction, idleThrust);
    }
  }
  return res;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) {
  return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust,
                                        float nominalVoltage,
                                        float actualVoltage) {
  if (actualVoltage <= 0.0f) return motorThrust;
  float compensated = roundf((float)motorThrust * nominalVoltage / actualVoltage);
  if (compensated < 0.0f) return 0;
  if (compensated > 65535.0f) return 65535;
  return (uint16_t)compensated;
}

/* -------------------------------------------------------------------------
 * 5. Cascade PID Controller Implementation
 * ------------------------------------------------------------------------- */
static void pidInit(PidObject *pid, float kp, float ki, float kd, float kff) {
  if (!pid) return;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->kff = kff;
  pid->integral = 0.0f;
  pid->prevError = 0.0f;
  pid->output = 0.0f;
  pid->initialized = true;
}

static float pidUpdate(PidObject *pid, float error, float dt) {
  if (!pid || !pid->initialized) return 0.0f;
  pid->integral += error * dt;
  float deriv = (dt > 0.0f) ? ((error - pid->prevError) / dt) : 0.0f;
  pid->prevError = error;
  pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * deriv;
  return pid->output;
}

void attitudeControllerInit(float updateDt) {
  (void)updateDt;
  pidInit(&pidRoll, 6.0f, 3.0f, 0.0f, 0.0f);
  pidInit(&pidPitch, 6.0f, 3.0f, 0.0f, 0.0f);
  pidInit(&pidYaw, 6.0f, 1.0f, 0.0f, 0.0f);
  pidInit(&pidRollRate, 250.0f, 500.0f, 2.5f, 0.0f);
  pidInit(&pidPitchRate, 250.0f, 500.0f, 2.5f, 0.0f);
  pidInit(&pidYawRate, 120.0f, 16.0f, 0.0f, 0.0f);
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
  float dt = 1.0f / (float)ATTITUDE_RATE_HZ;
  float rOut = pidUpdate(&pidRollRate, rollDesired - rollActual, dt);
  float pOut = pidUpdate(&pidPitchRate, pitchDesired - pitchActual, dt);
  float yOut = pidUpdate(&pidYawRate, yawDesired - yawActual, dt);
  pidRollRate.output = (float)saturateSignedInt16((int32_t)rOut);
  pidPitchRate.output = (float)saturateSignedInt16((int32_t)pOut);
  pidYawRate.output = (float)saturateSignedInt16((int32_t)yOut);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
  float dt = 1.0f / (float)ATTITUDE_RATE_HZ;
  pidUpdate(&pidRoll, rollDesired - rollActual, dt);
  pidUpdate(&pidPitch, pitchDesired - pitchActual, dt);
  pidYaw.integral = 0.0f;
  pidUpdate(&pidYaw, capAngle(yawDesired - yawActual), dt);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual, float yawActual) {
  pidRoll.integral = 0.0f;
  pidRoll.prevError = rollActual;
  pidPitch.integral = 0.0f;
  pidPitch.prevError = pitchActual;
  pidYaw.integral = 0.0f;
  pidYaw.prevError = yawActual;
  pidRollRate.integral = 0.0f;
  pidRollRate.prevError = 0.0f;
  pidPitchRate.integral = 0.0f;
  pidPitchRate.prevError = 0.0f;
  pidYawRate.integral = 0.0f;
  pidYawRate.prevError = 0.0f;
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
  pidRoll.integral = 0.0f;
  pidRoll.prevError = rollActual;
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
  pidPitch.integral = 0.0f;
  pidPitch.prevError = pitchActual;
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch, int16_t *yaw) {
  if (!roll || !pitch || !yaw) return;
  *roll = (int16_t)pidRollRate.output;
  *pitch = (int16_t)pidPitchRate.output;
  *yaw = (int16_t)pidYawRate.output;
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
  if (!setpoint || !state) return 0;
  float errZ = setpoint->position.z - state->position.z;
  float targetThrust = setpoint->thrust + errZ * 10000.0f;
  if (targetThrust < 0.0f) return 0;
  if (targetThrust > 60000.0f) return 60000;
  return (uint16_t)targetThrust;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
  if (!sensors || !setpoint || !state || !control) return;

  if (setpoint->mode.yaw == modeVelocity) {
    g_desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
    g_desiredYaw = capAngle(g_desiredYaw);
  } else {
    g_desiredYaw = setpoint->attitude.yaw;
  }

  if (yawMaxDelta != 0.0f) {
    float diff = capAngle(g_desiredYaw - state->attitude.yaw);
    diff = clampf(diff, -yawMaxDelta, yawMaxDelta);
    g_desiredYaw = capAngle(state->attitude.yaw + diff);
  }

  if (setpoint->thrust == 0) {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    control->thrust = 0;
    attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
    g_desiredYaw = state->attitude.yaw;
    return;
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

  attitudeControllerCorrectAttitudePID(state->attitude.roll, rollDesired,
                                       state->attitude.pitch, pitchDesired,
                                       state->attitude.yaw, g_desiredYaw);

  float pitchActual = -sensors->gyro.y;
  attitudeControllerCorrectRatePID(sensors->gyro.x, pidRoll.output,
                                   pitchActual, pidPitch.output,
                                   sensors->gyro.z, pidYaw.output);

  int16_t rOut, pOut, yOut;
  attitudeControllerGetActuatorOutput(&rOut, &pOut, &yOut);
  control->roll = rOut;
  control->pitch = pOut;
  control->yaw = (control->controlMode == controlModeLegacy) ? -yOut : yOut;

  if (setpoint->mode.z == modeDisable) {
    control->thrust = setpoint->thrust;
  } else {
    control->thrust = positionControllerUpdate(setpoint, state);
  }
}

/* -------------------------------------------------------------------------
 * 6. CRTP Commander RPYT Implementation
 * ------------------------------------------------------------------------- */
void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
  if (!rollPrime || !pitchPrime) return;
  float rad = yaw_deg * (M_PI / 180.0f);
  float cosY = cosf(rad);
  float sinY = sinf(rad);
  *rollPrime = roll * cosY - pitch * sinY;
  *pitchPrime = roll * sinY + pitch * cosY;
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
  if (!values || !setpoint) return;

  if (g_activePriority == COMMANDER_PRIORITY_DISABLE) {
    thrustLocked = true;
  }
  if (values->thrust == 0) {
    thrustLocked = false;
  }

  setpoint->timestamp = g_systemTick;

  if (!altHoldMode) {
    if (thrustLocked || values->thrust < 1000) {
      setpoint->thrust = 0;
    } else {
      setpoint->thrust = (values->thrust > 60000U) ? 60000U : values->thrust;
    }
    setpoint->mode.z = modeDisable;
    commanderModeSet = false;
  } else {
    setpoint->mode.z = modeVelocity;
    setpoint->thrust = 0;
    setpoint->velocity.z = ((float)values->thrust - 32767.0f) / 32767.0f;
    if (!commanderModeSet) {
      attitudeControllerResetAllPID(0, 0, 0);
      commanderModeSet = true;
    }
  }

  if (posHoldMode) {
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->velocity.x = values->pitch / 30.0f;
    setpoint->velocity.y = values->roll / 30.0f;
    setpoint->attitude.roll = 0.0f;
    setpoint->attitude.pitch = 0.0f;
  } else if (posSetMode && values->thrust != 0) {
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
    setpoint->thrust = 0;
  } else {
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;

    float rollVal = values->roll;
    float pitchVal = values->pitch;
    if (yawMode == PLUSMODE) {
      rotateYaw(values->roll, values->pitch, 45.0f, &rollVal, &pitchVal);
    } else if (yawMode == CAREFREE) {
      commanderModeSet = false;
    }

    if (stabilizationModeRoll == RATE) {
      setpoint->mode.roll = modeVelocity;
      setpoint->attitudeRate.roll = rollVal;
    } else {
      setpoint->mode.roll = modeAbs;
      setpoint->attitude.roll = rollVal;
    }

    if (stabilizationModePitch == RATE) {
      setpoint->mode.pitch = modeVelocity;
      setpoint->attitudeRate.pitch = pitchVal;
    } else {
      setpoint->mode.pitch = modeAbs;
      setpoint->attitude.pitch = pitchVal;
    }

    if (stabilizationModeYaw == RATE) {
      setpoint->mode.yaw = modeVelocity;
      setpoint->attitudeRate.yaw = -values->yaw;
    } else {
      setpoint->mode.yaw = modeAbs;
      setpoint->attitude.yaw = values->yaw;
    }
  }
}

/* -------------------------------------------------------------------------
 * 7. Supervisor Implementation
 * ------------------------------------------------------------------------- */
void supervisorInit(void) {
  supervisorState = supervisorStatePreFlChecksPassed;
  supervisorConditionBits = 0;
  g_spinupStartTick = 0;
  g_lastFlightTick = 0;
  g_seenFlight = false;
  g_tiltStartTick = 0;
  g_upsideDownStartTick = 0;
  g_motorNotRespondingStartTick = 0;
  supervisorLog.info = supervisorGetInfoBitfield();
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
  g_autoArming = autoArming;
  g_spinupTimeoutDurationMs = spinupTimeoutDurationMs;
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

void supervisorSetSensorData(const SensorData *sensors) {
  if (sensors) g_supervisorSensorData = *sensors;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
  if (motorRatios) {
    for (int i = 0; i < 4; i++) g_supervisorMotorRatios[i] = motorRatios[i];
  }
  g_supervisorIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
  if (motorRPMs) {
    for (int i = 0; i < 4; i++) g_supervisorMotorRPMs[i] = motorRPMs[i];
  }
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
  return (supervisorConditionBits & SUPERVISOR_CB_ARMED) != 0;
}

bool supervisorIsCrashed(void) {
  return (supervisorConditionBits & SUPERVISOR_CB_CRASHED) != 0;
}

bool supervisorAreMotorsAllowedToRun(void) {
  return supervisorState == supervisorStateArming ||
         supervisorState == supervisorStateReadyToFly ||
         supervisorState == supervisorStateFlying ||
         supervisorState == supervisorStateWarningLevelOut ||
         supervisorState == supervisorStateLanded;
}

bool supervisorRequestArming(bool doArm) {
  if (doArm) {
    if (supervisorCanArm() || supervisorState == supervisorStateArming) {
      supervisorConditionBits |= SUPERVISOR_CB_ARMED;
      if (supervisorState != supervisorStateArming) {
        supervisorState = supervisorStateArming;
        g_spinupStartTick = g_systemTick;
      }
      return true;
    }
    return false;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    if (supervisorState == supervisorStateArming || supervisorCanFly()) {
      supervisorState = supervisorStatePreFlChecksPassed;
      g_spinupStartTick = 0;
    }
    return true;
  }
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
  bool isFreeFalling = false;
  bool tumbled = isTumbledCheck(g_supervisorSensorData.acc.x, g_supervisorSensorData.acc.y,
                               g_supervisorSensorData.acc.z, g_crashDetectionGs,
                               g_freeFallThreshold, g_acceptedTiltAccZ,
                               g_acceptedUpsideDownAccZ, g_maxTiltTime,
                               g_maxUpsideDownTime, g_tumbleCheckEnabled,
                               g_systemTick, &isFreeFalling);
  if (tumbled) return false;
  if (!doRecovery) {
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    supervisorState = supervisorStateCrashed;
    return true;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    if (supervisorState == supervisorStateCrashed) {
      supervisorState = supervisorStatePreFlChecksPassed;
    }
    return true;
  }
}

uint16_t supervisorGetInfoBitfield(void) {
  uint16_t info = 0;
  if (supervisorCanArm()) info |= (1U << 0);
  if (supervisorIsArmed()) info |= (1U << 1);
  if (g_autoArming) info |= (1U << 2);
  if (supervisorCanFly()) info |= (1U << 3);
  if (isFlyingCheck(g_supervisorMotorRatios, g_supervisorIdleThrust, g_systemTick)) info |= (1U << 4);
  bool isFreeFalling = false;
  if (isTumbledCheck(g_supervisorSensorData.acc.x, g_supervisorSensorData.acc.y,
                    g_supervisorSensorData.acc.z, g_crashDetectionGs,
                    g_freeFallThreshold, g_acceptedTiltAccZ,
                    g_acceptedUpsideDownAccZ, g_maxTiltTime,
                    g_maxUpsideDownTime, g_tumbleCheckEnabled,
                    g_systemTick, &isFreeFalling)) info |= (1U << 5);
  if (thrustLocked) info |= (1U << 6);
  if (supervisorIsCrashed()) info |= (1U << 7);
  if (supervisorConditionBits & SUPERVISOR_CB_DECK_FAULT) info |= (1U << 11);
  return info;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust, uint32_t currentTick) {
  if (!motorRatios) return false;
  bool active = false;
  for (int i = 0; i < 4; i++) {
    if (motorRatios[i] > idleThrust) {
      active = true;
      break;
    }
  }
  if (active) {
    g_lastFlightTick = currentTick;
    g_seenFlight = true;
  }
  if (!g_seenFlight) return false;
  return (currentTick - g_lastFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling) {
  if (isFreeFalling) *isFreeFalling = false;

  float accNorm = sqrtf(accX * accX + accY * accY + accZ * accZ);
  if (crashDetectionGs > 0.0f && fabsf(accNorm - 1.0f) > crashDetectionGs) {
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
  }

  if (freeFallThreshold > 0.0f && fabsf(accX) < freeFallThreshold &&
      fabsf(accY) < freeFallThreshold && fabsf(accZ) < freeFallThreshold) {
    if (isFreeFalling) *isFreeFalling = true;
    supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
    g_tiltStartTick = 0;
    g_upsideDownStartTick = 0;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;
  }

  if (!tumbleCheckEnabled) return false;

  bool tumbled = false;
  if (acceptedTiltAccZ > 0.0f && accZ < acceptedTiltAccZ) {
    if (g_tiltStartTick == 0) g_tiltStartTick = currentTick;
    if (currentTick - g_tiltStartTick >= maxTiltTime) tumbled = true;
  } else {
    g_tiltStartTick = 0;
  }

  if (acceptedUpsideDownAccZ > 0.0f && accZ < acceptedUpsideDownAccZ) {
    if (g_upsideDownStartTick == 0) g_upsideDownStartTick = currentTick;
    if (currentTick - g_upsideDownStartTick >= maxUpsideDownTime) tumbled = true;
  } else {
    g_upsideDownStartTick = 0;
  }

  if (tumbled) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
  else supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;

  return tumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick) {
  if (lastNotificationTick == 0) return true;
  return (currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state, uint32_t latestArmingTick,
                                  uint32_t currentTick, uint32_t preflightTimeoutDuration) {
  if (state != supervisorStateArming || latestArmingTick == 0) return false;
  return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick, uint32_t currentTick,
                                uint32_t landingTimeoutDuration) {
  if (latestLandingTick == 0) return false;
  return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop, bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed) {
  if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
    supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
  }
  return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint, uint32_t condBits, SupervisorState state) {
  if (!setpoint) return;
  if (state == supervisorStateWarningLevelOut) {
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;
    setpoint->attitude.roll = 0.0f;
    setpoint->attitude.pitch = 0.0f;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitudeRate.yaw = 0.0f;
  } else if (!supervisorCanFly() && state != supervisorStateArming) {
    setpoint->thrust = 0;
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.z = modeDisable;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeDisable;
  }
  if (condBits & (SUPERVISOR_CB_EMERGENCY_STOP | SUPERVISOR_CB_CRASHED | SUPERVISOR_CB_IS_TUMBLED)) {
    setpoint->thrust = 0;
  }
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin, int32_t rpmCheckMax) {
  if (!motorRPMs) return false;
  for (int i = 0; i < 4; i++) {
    if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
  }
  return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly, uint32_t currentTick) {
  if (!motorRPMs || !canFly) {
    g_motorNotRespondingStartTick = 0;
    return false;
  }
  bool low = false;
  for (int i = 0; i < 4; i++) {
    if (motorRPMs[i] < rpmThreshold) {
      low = true;
      break;
    }
  }
  if (low) {
    if (g_motorNotRespondingStartTick == 0) g_motorNotRespondingStartTick = currentTick;
    if (currentTick - g_motorNotRespondingStartTick >= rpmCheckDurationMs) return true;
  } else {
    g_motorNotRespondingStartTick = 0;
  }
  return false;
}

void supervisorUpdate(uint32_t stabilizerStep) {
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;
  g_systemTick += RATE_SUPERVISOR;

  if (supervisorState == supervisorStatePreFlChecksPassed && g_autoArming) {
    supervisorRequestArming(true);
  }

  if (supervisorState == supervisorStateArming) {
    if (isRPMatArmingValid(g_supervisorMotorRPMs, 1000, 20000)) {
      supervisorState = supervisorStateReadyToFly;
      g_spinupStartTick = 0;
    } else if (g_spinupStartTick != 0 && (g_systemTick - g_spinupStartTick >= g_spinupTimeoutDurationMs)) {
      supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
      supervisorState = supervisorStatePreFlChecksPassed;
      supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
      g_spinupStartTick = 0;
    }
  }

  uint32_t inact = commanderGetInactivityTime();
  if (inact > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
    supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    if (supervisorState == supervisorStateFlying) supervisorState = supervisorStateLanded;
  } else if (inact > COMMANDER_WDT_TIMEOUT_STABILIZE) {
    supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    if (supervisorState == supervisorStateFlying) supervisorState = supervisorStateWarningLevelOut;
  } else {
    supervisorConditionBits &= ~(SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT);
    if (supervisorState == supervisorStateWarningLevelOut) supervisorState = supervisorStateFlying;
  }

  supervisorLog.info = supervisorGetInfoBitfield();
  supervisorLog.accNorm = sqrtf(g_supervisorSensorData.acc.x * g_supervisorSensorData.acc.x +
                                g_supervisorSensorData.acc.y * g_supervisorSensorData.acc.y +
                                g_supervisorSensorData.acc.z * g_supervisorSensorData.acc.z);
}

/* -------------------------------------------------------------------------
 * 8. Estimator Implementation
 * ------------------------------------------------------------------------- */
bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
  if (!measurement || g_estimatorCount >= 16) return false;
  g_estimatorQueue[g_estimatorTail] = *measurement;
  g_estimatorTail = (g_estimatorTail + 1) % 16;
  g_estimatorCount++;
  return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
  if (!measurement || g_estimatorCount == 0) return false;
  *measurement = g_estimatorQueue[g_estimatorHead];
  g_estimatorHead = (g_estimatorHead + 1) % 16;
  g_estimatorCount--;
  return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
  EstimatorMeasurement m;
  while (estimatorDequeue(&m)) {
    if (m.type == MeasurementTypeGyroscope) g_lastGyro = m;
    else if (m.type == MeasurementTypeAcceleration) g_lastAcc = m;
    else if (m.type == MeasurementTypeBarometer) g_lastBaro = m;
    else if (m.type == MeasurementTypeTOF) g_lastTOF = m;
  }

  if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
    sensfusion6UpdateQ(g_lastGyro.data[0], g_lastGyro.data[1], g_lastGyro.data[2],
                       g_lastAcc.data[0], g_lastAcc.data[1], g_lastAcc.data[2], 0.004f);
    sensfusion6GetEulerRPY(&stateEstimate.roll, &stateEstimate.pitch, &stateEstimate.yaw);
    stateEstimate.qw = qw;
    stateEstimate.qx = qx;
    stateEstimate.qy = qy;
    stateEstimate.qz = qz;
  }
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
  if (!setpoint) return false;
  if (priority != COMMANDER_PRIORITY_DISABLE && priority < g_activePriority) return false;
  g_activeSetpoint = *setpoint;
  g_activePriority = priority;
  g_lastSetpointUpdateTick = g_systemTick;
  return true;
}

void commanderRelaxPriority(void) {
  g_activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
  return g_systemTick - g_lastSetpointUpdateTick;
}

int commanderGetActivePriority(void) {
  return g_activePriority;
}

/* -------------------------------------------------------------------------
 * 9. Stabilizer Implementation
 * ------------------------------------------------------------------------- */
void stabilizerInit(void) {
  sensfusion6Init();
  attitudeControllerInit(0.002f);
  supervisorInit();
  g_hasHighLevelSetpoint = false;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
  if (!setpoint) return false;
  g_highLevelSetpoint = *setpoint;
  g_hasHighLevelSetpoint = true;
  return true;
}

void stabilizerTask(void) {
  static uint32_t step = 0;
  step++;

  if (healthShallWeRunTest()) {
    healthRunTests(&g_supervisorSensorData);
    return;
  }

  if (g_hasHighLevelSetpoint) {
    commanderSetSetpoint(&g_highLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
    g_hasHighLevelSetpoint = false;
  }

  estimatorComplementary(step);
  supervisorUpdate(step);

  Setpoint currentSetpoint = g_activeSetpoint;
  supervisorOverrideSetpoint(&currentSetpoint, supervisorConditionBits, supervisorState);

  State dummyState;
  memset(&dummyState, 0, sizeof(dummyState));
  sensfusion6GetEulerRPY(&dummyState.attitude.roll, &dummyState.attitude.pitch, &dummyState.attitude.yaw);

  ControlData control;
  memset(&control, 0, sizeof(control));
  control.controlMode = controlModeLegacy;
  controllerPid(&g_supervisorSensorData, &currentSetpoint, &dummyState, &control, 0.0f, 0.002f);

  MotorPower motorPower;
  powerDistribution(&control, &motorPower);
  int32_t motors[4] = { motorPower.m1, motorPower.m2, motorPower.m3, motorPower.m4 };
  powerDistributionCap(motors, 60000, 0);

  if (!supervisorAreMotorsAllowedToRun()) {
    motors[0] = 0;
    motors[1] = 0;
    motors[2] = 0;
    motors[3] = 0;
  }

  motor.m1req = (uint16_t)motors[0];
  motor.m2req = (uint16_t)motors[1];
  motor.m3req = (uint16_t)motors[2];
  motor.m4req = (uint16_t)motors[3];
}

void compressState(const State *state, const SensorData *sensors, CompressedState *output) {
  if (!state || !sensors || !output) return;
  output->position_mm[0] = (int32_t)(state->position.x * 1000.0f);
  output->position_mm[1] = (int32_t)(state->position.y * 1000.0f);
  output->position_mm[2] = (int32_t)(state->position.z * 1000.0f);

  output->velocity_mms[0] = (int32_t)(state->velocity.x * 1000.0f);
  output->velocity_mms[1] = (int32_t)(state->velocity.y * 1000.0f);
  output->velocity_mms[2] = (int32_t)(state->velocity.z * 1000.0f);

  output->acceleration_mms2[0] = (int32_t)(state->acc.x * 9810.0f);
  output->acceleration_mms2[1] = (int32_t)(state->acc.y * 9810.0f);
  output->acceleration_mms2[2] = (int32_t)((state->acc.z + 1.0f) * 9810.0f);

  output->gyro_millirad_s[0] = sensors->gyro.x * (M_PI / 180.0f) * 1000.0f;
  output->gyro_millirad_s[1] = -sensors->gyro.y * (M_PI / 180.0f) * 1000.0f;
  output->gyro_millirad_s[2] = sensors->gyro.z * (M_PI / 180.0f) * 1000.0f;

  output->quatCompressed = 0;
}

bool rateSupervisorValidate(uint32_t measuredRate) {
  return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void) {
}

/* -------------------------------------------------------------------------
 * 10. Health Implementation
 * ------------------------------------------------------------------------- */
float variance(const float *buffer, int length) {
  if (!buffer || length <= 0) return 0.0f;
  float sum = 0.0f;
  float sumSq = 0.0f;
  for (int i = 0; i < length; i++) {
    sum += buffer[i];
    sumSq += buffer[i] * buffer[i];
  }
  return sumSq - (sum * sum / (float)length);
}

bool evaluatePropTest(float lowThreshold, float highThreshold, float measuredValue, uint8_t motorIdx) {
  if (highThreshold == 0.0f) return true;
  if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
    if (motorIdx < 4) motorPass |= (uint8_t)(1U << motorIdx);
    return true;
  }
  return false;
}

void healthRequestPropTest(void) {
  g_propTestRequested = true;
}

void healthRequestBatteryTest(void) {
  g_batTestRequested = true;
}

bool healthShallWeRunTest(void) {
  if (g_propTestRequested) {
    g_propTestRequested = false;
    healthTestState = configureAcc;
    g_accSampleIndex = 0;
    return true;
  }
  if (g_batTestRequested) {
    g_batTestRequested = false;
    healthTestState = testBattery;
    g_healthTick = 0;
    g_minLoadedVoltage = 4.2f;
    return true;
  }
  return false;
}

void healthRunTests(const SensorData *sensorData) {
  if (!sensorData) return;
  switch (healthTestState) {
    case configureAcc:
      g_accSampleIndex = 0;
      healthTestState = measureNoiseFloor;
      break;
    case measureNoiseFloor:
      if (g_accSampleIndex < 100) {
        g_accVarianceBuffer[g_accSampleIndex++] = sensorData->acc.z;
      }
      if (g_accSampleIndex >= 100) {
        healthTestState = measureProp;
      }
      break;
    case measureProp:
      evaluatePropTest(0.0f, 2.0f, variance(g_accVarianceBuffer, 100), 0);
      healthTestState = evaluatePropResult;
      break;
    case evaluatePropResult:
      healthTestState = testDone;
      break;
    case testBattery:
      g_healthTick++;
      if (g_healthTick == 1) {
        g_idleVoltage = sensorData->baroPressure; /* simulated voltage */
      } else if (g_healthTick >= 2 && g_healthTick <= 49) {
        if (sensorData->baroPressure < g_minLoadedVoltage) {
          g_minLoadedVoltage = sensorData->baroPressure;
        }
      } else if (g_healthTick >= 50) {
        batterySag = g_idleVoltage - g_minLoadedVoltage;
        healthTestState = evaluateBatResult;
      }
      break;
    case evaluateBatResult:
      batteryPass = (batterySag < 0.5f) ? 1 : 0;
      healthTestState = testDone;
      break;
    case restartBatTest:
      healthTestState = testBattery;
      g_healthTick = 0;
      break;
    case testDone:
    default:
      break;
  }
  healthLog.motorPass = motorPass;
  healthLog.batteryPass = batteryPass;
  healthLog.batterySag = batterySag;
  healthLog.motorTestCount = ++g_motorTestCount;
}

/* -------------------------------------------------------------------------
 * 11. CRTP Transport Implementation
 * ------------------------------------------------------------------------- */
void crtpInit(void) {
  g_txQueueHead = 0;
  g_txQueueTail = 0;
  g_txQueueCount = 0;
  for (int i = 0; i < (int)CRTP_NBR_OF_PORTS; i++) {
    g_rxHead[i] = 0;
    g_rxTail[i] = 0;
    g_rxCount[i] = 0;
    g_rxQueueCreated[i] = false;
    g_portCallbacks[i] = NULL;
  }
}

void crtpInitTaskQueue(uint8_t port) {
  if (port >= CRTP_NBR_OF_PORTS) return;
  g_rxQueueCreated[port] = true;
  g_rxHead[port] = 0;
  g_rxTail[port] = 0;
  g_rxCount[port] = 0;
}

bool crtpSendPacket(const CrtpPacket *packet) {
  if (!packet || g_txQueueCount >= CRTP_TX_QUEUE_SIZE) return false;
  g_txQueue[g_txQueueTail] = *packet;
  g_txQueueTail = (g_txQueueTail + 1) % CRTP_TX_QUEUE_SIZE;
  g_txQueueCount++;
  return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
  return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
  if (port >= CRTP_NBR_OF_PORTS || !packet || g_rxCount[port] == 0) return false;
  *packet = g_rxQueues[port][g_rxHead[port]];
  g_rxHead[port] = (g_rxHead[port] + 1) % CRTP_RX_QUEUE_SIZE;
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
  if (!g_activeLink || !g_activeLink->receivePacket) return;
  CrtpPacket pk;
  if (g_activeLink->receivePacket(&pk)) {
    g_rxPacketCount++;
    if (pk.port < CRTP_NBR_OF_PORTS) {
      if (g_rxQueueCreated[pk.port] && g_rxCount[pk.port] < CRTP_RX_QUEUE_SIZE) {
        g_rxQueues[pk.port][g_rxTail[pk.port]] = pk;
        g_rxTail[pk.port] = (g_rxTail[pk.port] + 1) % CRTP_RX_QUEUE_SIZE;
        g_rxCount[pk.port]++;
      }
      if (g_portCallbacks[pk.port]) {
        g_portCallbacks[pk.port](&pk);
      }
    }
  }
}

void crtpTxTask(void) {
  if (!g_activeLink || !g_activeLink->sendPacket || g_txQueueCount == 0) return;
  CrtpPacket pk = g_txQueue[g_txQueueHead];
  if (g_activeLink->sendPacket(&pk)) {
    g_txQueueHead = (g_txQueueHead + 1) % CRTP_TX_QUEUE_SIZE;
    g_txQueueCount--;
    g_txPacketCount++;
  }
}

void crtpSetLink(CrtpLink *newLink) {
  if (g_activeLink && g_activeLink->setEnable) {
    g_activeLink->setEnable(false);
  }
  g_activeLink = newLink;
  if (g_activeLink && g_activeLink->setEnable) {
    g_activeLink->setEnable(true);
  }
}

void crtpReset(void) {
  g_txQueueHead = 0;
  g_txQueueTail = 0;
  g_txQueueCount = 0;
  if (g_activeLink && g_activeLink->reset) {
    g_activeLink->reset();
  }
}

bool crtpIsConnected(void) {
  if (g_activeLink && g_activeLink->isConnected) {
    return g_activeLink->isConnected();
  }
  return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
  return CRTP_TX_QUEUE_SIZE - g_txQueueCount;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
  if (port >= CRTP_NBR_OF_PORTS) return;
  g_portCallbacks[port] = callback;
}

void updateStats(void) {
  if (g_systemTick - g_lastStatsTick >= 500) {
    g_rxPacketCount = 0;
    g_txPacketCount = 0;
    g_lastStatsTick = g_systemTick;
  }
}

/* -------------------------------------------------------------------------
 * 12. Deck Discovery Implementation
 * ------------------------------------------------------------------------- */
uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
  if (!decks || capacity == 0) return 0;
  uint8_t count = 0;
  if (count < capacity) {
    decks[count].foundByI2C = true;
    decks[count].foundByOneWire = false;
    decks[count].i2cAddress = 0xbc;
    decks[count].oneWireRomId = 0;
    count++;
  }
  return count;
}
#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#define CRAZYFLIE_PI_F 3.14159265358979323846f
#define CRAZYFLIE_DEG_TO_RAD_F (CRAZYFLIE_PI_F / 180.0f)
#define CRAZYFLIE_RAD_TO_DEG_F (180.0f / CRAZYFLIE_PI_F)

/* Host-visible monotonic millisecond tick. The frozen API has no public setter;
   tests may declare extern uint32_t tick; and write it directly. */
uint32_t tick = 0U;

/* Sensfusion6 public state */
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
float beta = 0.0f;
float baseZacc = 0.0f;
bool sensfusion6IsInit = false;
bool sensfusion6IsCalibrated = false;

/* PID globals */
PidObject pidRoll = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidPitch = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidYaw = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidRollRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidPitchRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
PidObject pidYawRate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

/* Commander RPYT public state */
bool thrustLocked = false;
bool commanderModeSet = false;

/* Supervisor public state */
SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0U;

/* Health public state */
TestState healthTestState = testDone;
uint8_t motorPass = 0U;
uint8_t batteryPass = 0U;
float batterySag = 0.0f;

/* Public log objects */
StateEstimateLog stateEstimate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
Axis3Log gyro = {0.0f, 0.0f, 0.0f};
Axis3Log acc = {0.0f, 0.0f, 0.0f};
BaroLog baro = {0.0f, 0.0f, 0.0f};
MotorLog motor = {0U, 0U, 0U, 0U};
Sensfusion6Log sensfusion6Log = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, false, false};
SupervisorLog supervisorLog = {0U, 0.0f};
HealthLog healthLog = {0U, 0U, 0.0f, 0U};

/* Private Sensfusion6 init guard */
static bool s_sensfusion6InitDone = false;

/* Private attitude controller init guard */
static bool s_attitudeControllerInitDone = false;
static float s_desiredYaw = 0.0f;

/* Commander private state */
static Setpoint s_activeSetpoint;
static int s_activePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t s_lastUpdateTick = 0U;
static bool s_highLevelTrajectoryActive = false;
static bool s_trajectoryFlying = false;
static bool s_trajectoryFinished = false;
static bool s_trajectoryDisabled = false;

/* Supervisor private state */
static SensorData s_supervisorSensors;
static uint32_t s_motorRatios[4] = {0U, 0U, 0U, 0U};
static int32_t s_motorRPMs[4] = {0, 0, 0, 0};
static uint32_t s_idleThrust = 0U;
static float s_crashDetectionGs = 0.0f;
static float s_freeFallThreshold = 0.0f;
static float s_acceptedTiltAccZ = 0.0f;
static float s_acceptedUpsideDownAccZ = 0.0f;
static uint32_t s_maxTiltTime = 0U;
static uint32_t s_maxUpsideDownTime = 0U;
static bool s_tumbleCheckEnabled = false;
static bool s_autoArming = false;
static uint32_t s_spinupTimeoutDurationMs = 0U;
static uint32_t s_spinupStartTick = 0U;
static uint32_t s_latestArmingTick = 0U;
static uint32_t s_latestLandingTick = 0U;
static bool s_seenFlying = false;
static uint32_t s_recentFlightTick = 0U;
static uint32_t s_tumbleStartTick = 0U;
static uint32_t s_notRespondingStartTick = 0U;
static bool s_crtpEmergencyStop = false;
static bool s_paramEmergencyStop = false;
static bool s_supervisorInitDone = false;

/* Estimator private state */
#define ESTIMATOR_FIFO_CAPACITY 16U
static EstimatorMeasurement s_estFifo[ESTIMATOR_FIFO_CAPACITY];
static uint8_t s_estHead = 0U;
static uint8_t s_estTail = 0U;
static uint8_t s_estCount = 0U;
static float s_lastGyro[3] = {0.0f, 0.0f, 0.0f};
static float s_lastAcc[3] = {0.0f, 0.0f, 0.0f};
static float s_lastBaro[3] = {0.0f, 0.0f, 0.0f};
static float s_lastTof[3] = {0.0f, 0.0f, 0.0f};
static float s_estPositionZ = 0.0f;
static float s_estVerticalVelocity = 0.0f;

/* CRTP private state */
static CrtpPacket s_txQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t s_txHead = 0U;
static uint16_t s_txTail = 0U;
static uint16_t s_txCount = 0U;
static CrtpPacket s_rxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint8_t s_rxHead[CRTP_NBR_OF_PORTS];
static uint8_t s_rxTail[CRTP_NBR_OF_PORTS];
static uint8_t s_rxCount[CRTP_NBR_OF_PORTS];
static bool s_rxQueueCreated[CRTP_NBR_OF_PORTS];
static CrtpPortCallback s_portCallbacks[CRTP_NBR_OF_PORTS];
static uint8_t s_crtpErrorState = 0U;
static bool s_crtpInitDone = false;
static uint32_t s_txRetryTick = 0U;
static uint32_t s_lastStatsTick = 0U;
static uint32_t s_rxPacketCounter = 0U;
static uint32_t s_txPacketCounter = 0U;

/* Health private state */
static bool s_pendingPropTest = false;
static bool s_pendingBatteryTest = false;
static uint32_t s_healthNoiseCount = 0U;
static float s_healthNoiseBuffer[PROPTEST_NBR_OF_VARIANCE_VALUES];
static float s_healthNoiseVariance = 0.0f;
static uint32_t s_healthMotorIndex = 0U;
static uint32_t s_healthBatteryTick = 0U;
static float s_healthIdleVoltage = 4.0f;
static float s_healthMinLoadedVoltage = 4.0f;
static uint32_t s_healthRestartStartTick = 0U;

/* Stabilizer private state */
static bool s_stabilizerInitDone = false;
static uint32_t s_stabilizerStep = 0U;
static Setpoint s_highLevelSetpoint;
static bool s_highLevelPending = false;
static Setpoint s_stabilizerSetpoint;
static ControlData s_stabilizerControl;
static MotorPower s_stabilizerMotorPower;
static State s_stabilizerState;
static float s_filteredBatteryVoltage = 0.0f;
static float s_batterySupplyVoltage = 4.2f;

/* Rate supervisor private state */
static bool s_sensorActive = true;
static uint32_t s_rateErrorStartTick = 0U;

static float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float quaternionNorm(float w, float x, float y, float z) {
  return sqrtf(w * w + x * x + y * y + z * z);
}

static void normalizeQuaternion(void) {
  float n = quaternionNorm(qw, qx, qy, qz);
  if (n < 1e-9f) {
    qw = 1.0f;
    qx = qy = qz = 0.0f;
    return;
  }
  qw /= n;
  qx /= n;
  qy /= n;
  qz /= n;
}

static float pidResetOne(PidObject *pid) {
  if (pid == NULL) return 0.0f;
  pid->integral = 0.0f;
  pid->prevError = 0.0f;
  pid->output = 0.0f;
  pid->initialized = true;
  return 0.0f;
}

static float pidUpdateOne(PidObject *pid, float actual, float desired) {
  if (pid == NULL) return 0.0f;
  if (!pid->initialized) {
    pidResetOne(pid);
  }
  float error = desired - actual;
  pid->integral += pid->ki * error;
  float dError = error - pid->prevError;
  pid->prevError = error;
  pid->output = pid->kp * error + pid->integral + pid->kd * dError + pid->kff * desired;
  return pid->output;
}

static uint16_t motorForceToPwm(float forceN) {
  if (forceN <= 0.0f) return 0U;
  if (forceN >= CRAZYFLIE_MAX_MOTOR_FORCE_N) return 65535U;
  float ratio = forceN / CRAZYFLIE_MAX_MOTOR_FORCE_N;
  float pwm = ratio * 65535.0f;
  if (pwm >= 65535.0f) return 65535U;
  return (uint16_t)pwm;
}

static uint32_t quatCompress(float w, float x, float y, float z) {
  float q[4] = {w, x, y, z};
  int largest = 0;
  float largestAbs = fabsf(q[0]);
  for (int i = 1; i < 4; ++i) {
    float a = fabsf(q[i]);
    if (a > largestAbs) {
      largestAbs = a;
      largest = i;
    }
  }
  int a = (largest + 1) & 3;
  int b = (largest + 2) & 3;
  int c = (largest + 3) & 3;
  float denom = (largestAbs < 1e-9f) ? 1.0f : largestAbs;
  int16_t qa = (int16_t)(clampFloat(q[a] / denom, -1.0f, 1.0f) * 511.0f);
  int16_t qb = (int16_t)(clampFloat(q[b] / denom, -1.0f, 1.0f) * 511.0f);
  int16_t qc = (int16_t)(clampFloat(q[c] / denom, -1.0f, 1.0f) * 511.0f);
  return ((uint32_t)largest << 30U) |
         ((uint32_t)(uint16_t)qa << 20U) |
         ((uint32_t)(uint16_t)qb << 10U) |
         ((uint32_t)(uint16_t)qc);
}

static void positionControllerReset(void) {
  /* Placeholder for position/filter reset boundary; no hidden state. */
}

/* -------------------------------------------------------------------------
   Numerical utilities
------------------------------------------------------------------------- */
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
  float half = x * 0.5f;
  union {
    float f;
    uint32_t i;
  } u;
  u.f = x;
  u.i = 0x5f3759dfU - (u.i >> 1U);
  float y = u.f;
  y = y * (1.5f - (half * y * y));
  return y;
}

/* -------------------------------------------------------------------------
   Sensfusion6
------------------------------------------------------------------------- */
void sensfusion6Init(void) {
  if (s_sensfusion6InitDone) return;
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
  s_sensfusion6InitDone = true;
}

bool sensfusion6Test(void) {
  return sensfusion6IsInit;
}

void estimatedGravityDirection(float w, float x, float y, float z,
                               float *gravX, float *gravY, float *gravZ) {
  if (gravX == NULL || gravY == NULL || gravZ == NULL) return;
  *gravX = 2.0f * (x * z - w * y);
  *gravY = 2.0f * (w * x + y * z);
  *gravZ = w * w - x * x - y * y + z * z;
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt) {
  if (dt <= 0.0f) dt = 1.0f / 250.0f;
  float gxr = gx * CRAZYFLIE_DEG_TO_RAD_F;
  float gyr = gy * CRAZYFLIE_DEG_TO_RAD_F;
  float gzr = gz * CRAZYFLIE_DEG_TO_RAD_F;

  bool zeroAcc = (ax == 0.0f) && (ay == 0.0f) && (az == 0.0f);
  if (!zeroAcc) {
    float accNorm = sqrtf(ax * ax + ay * ay + az * az);
    if (accNorm < 1e-9f) {
      zeroAcc = true;
    } else {
      ax /= accNorm;
      ay /= accNorm;
      az /= accNorm;

      if (!sensfusion6IsCalibrated) {
        float gvx, gvy, gvz;
        estimatedGravityDirection(qw, qx, qy, qz, &gvx, &gvy, &gvz);
        baseZacc = ax * gvx + ay * gvy + az * gvz;
        sensfusion6IsCalibrated = true;
      }

      float gvx, gvy, gvz;
      estimatedGravityDirection(qw, qx, qy, qz, &gvx, &gvy, &gvz);
      float ex = ay * gvz - az * gvy;
      float ey = az * gvx - ax * gvz;
      float ez = ax * gvy - ay * gvx;

      if (beta > 0.0f) {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
        gxr += beta * ex;
        gyr += beta * ey;
        gzr += beta * ez;
      } else {
        if (twoKi > 0.0f) {
          integralFBx += twoKi * ex * dt;
          integralFBy += twoKi * ey * dt;
          integralFBz += twoKi * ez * dt;
        } else {
          integralFBx = 0.0f;
          integralFBy = 0.0f;
          integralFBz = 0.0f;
        }
        gxr += twoKp * ex + integralFBx;
        gyr += twoKp * ey + integralFBy;
        gzr += twoKp * ez + integralFBz;
      }
    }
  }

  float halfDt = 0.5f * dt;
  float oldQw = qw;
  float oldQx = qx;
  float oldQy = qy;
  float oldQz = qz;

  qw += (-oldQx * gxr - oldQy * gyr - oldQz * gzr) * halfDt;
  qx += (oldQw * gxr + oldQy * gzr - oldQz * gyr) * halfDt;
  qy += (oldQw * gyr - oldQx * gzr + oldQz * gxr) * halfDt;
  qz += (oldQw * gzr + oldQx * gyr - oldQy * gxr) * halfDt;

  normalizeQuaternion();
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);

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

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
  if (roll_deg == NULL || pitch_deg == NULL || yaw_deg == NULL) return;
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  float clampedGravityX = clampFloat(gravityX, -1.0f, 1.0f);
  (void)clampedGravityX;

  float sinPitch = 2.0f * (qw * qy - qz * qx);
  sinPitch = clampFloat(sinPitch, -1.0f, 1.0f);
  float rollRad = atan2f(2.0f * (qw * qx + qy * qz),
                        1.0f - 2.0f * (qx * qx + qy * qy));
  float yawRad = atan2f(2.0f * (qw * qz + qx * qy),
                       1.0f - 2.0f * (qy * qy + qz * qz));
  *roll_deg = rollRad * CRAZYFLIE_RAD_TO_DEG_F;
  *pitch_deg = asinf(sinPitch) * CRAZYFLIE_RAD_TO_DEG_F;
  *yaw_deg = yawRad * CRAZYFLIE_RAD_TO_DEG_F;
}

void sensfusion6GetQuaternion(float *w, float *x, float *y, float *z) {
  if (w == NULL || x == NULL || y == NULL || z == NULL) return;
  *w = qw;
  *x = qx;
  *y = qy;
  *z = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
  float gvx, gvy, gvz;
  estimatedGravityDirection(qw, qx, qy, qz, &gvx, &gvy, &gvz);
  return ax * gvx + ay * gvy + az * gvz;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* -------------------------------------------------------------------------
   Power distribution and battery
------------------------------------------------------------------------- */
void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out) {
  if (out == NULL) return;
  int32_t r = roll / 2;
  int32_t p = pitch / 2;
  int32_t t = (int32_t)thrust;
  int32_t y = (int32_t)yaw;
  out->m1 = t - r + p + y;
  out->m2 = t - r - p - y;
  out->m3 = t + r - p + y;
  out->m4 = t + r + p - y;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4]) {
  if (motorForces == NULL) return;
  float thrustPart = 0.25f * thrustSi;
  float rollPart = 0.0f;
  float pitchPart = 0.0f;
  float yawPart = 0.0f;

  if (armLength != 0.0f) {
    float arm = 0.707106781f * armLength;
    rollPart = 0.25f / arm * torqueX;
    pitchPart = 0.25f / arm * torqueY;
  }
  if (thrustToTorque != 0.0f) {
    yawPart = 0.25f / thrustToTorque * torqueZ;
  }

  float f0 = thrustPart - rollPart - pitchPart + yawPart;
  float f1 = thrustPart - rollPart + pitchPart - yawPart;
  float f2 = thrustPart + rollPart + pitchPart + yawPart;
  float f3 = thrustPart + rollPart - pitchPart - yawPart;
  if (f0 < 0.0f) f0 = 0.0f;
  if (f1 < 0.0f) f1 = 0.0f;
  if (f2 < 0.0f) f2 = 0.0f;
  if (f3 < 0.0f) f3 = 0.0f;
  motorForces[0] = f0;
  motorForces[1] = f1;
  motorForces[2] = f2;
  motorForces[3] = f3;
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4]) {
  if (normalizedForces == NULL || motorPWMs == NULL) return;
  for (int i = 0; i < 4; ++i) {
    float v = clampFloat(normalizedForces[i], 0.0f, 1.0f);
    motorPWMs[i] = (uint16_t)(v * 65535.0f);
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
      motorPower->m1 = (int32_t)motorForceToPwm(forces[0]);
      motorPower->m2 = (int32_t)motorForceToPwm(forces[1]);
      motorPower->m3 = (int32_t)motorForceToPwm(forces[2]);
      motorPower->m4 = (int32_t)motorForceToPwm(forces[3]);
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
    result.isCapped = true;
    result.reduction = maxVal - maxAllowedThrust;
    for (int i = 0; i < 4; ++i) {
      motors[i] = capMinThrust(motors[i] - result.reduction, idleThrust);
    }
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
  float scaled = roundf((float)motorThrust * nominalVoltage / actualVoltage);
  if (scaled < 0.0f) return 0U;
  if (scaled > 65535.0f) return 65535U;
  return (uint16_t)scaled;
}

/* -------------------------------------------------------------------------
   Cascade PID
------------------------------------------------------------------------- */
void attitudeControllerInit(float updateDt) {
  (void)updateDt;
  if (s_attitudeControllerInitDone) return;
  pidResetOne(&pidRoll);
  pidResetOne(&pidPitch);
  pidResetOne(&pidYaw);
  pidResetOne(&pidRollRate);
  pidResetOne(&pidPitchRate);
  pidResetOne(&pidYawRate);
  s_desiredYaw = 0.0f;
  s_attitudeControllerInitDone = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
  float rollOut = pidUpdateOne(&pidRollRate, rollActual, rollDesired);
  float pitchOut = pidUpdateOne(&pidPitchRate, pitchActual, pitchDesired);
  float yawOut = pidUpdateOne(&pidYawRate, yawActual, yawDesired);
  pidRollRate.output = (float)saturateSignedInt16((int32_t)rollOut);
  pidPitchRate.output = (float)saturateSignedInt16((int32_t)pitchOut);
  pidYawRate.output = (float)saturateSignedInt16((int32_t)yawOut);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
  pidUpdateOne(&pidRoll, rollActual, rollDesired);
  pidUpdateOne(&pidPitch, pitchActual, pitchDesired);
  pidResetOne(&pidYaw);
  pidUpdateOne(&pidYaw, yawActual, yawDesired);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual) {
  (void)rollActual;
  (void)pitchActual;
  (void)yawActual;
  pidResetOne(&pidRoll);
  pidResetOne(&pidPitch);
  pidResetOne(&pidYaw);
  pidResetOne(&pidRollRate);
  pidResetOne(&pidPitchRate);
  pidResetOne(&pidYawRate);
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
  (void)rollActual;
  pidResetOne(&pidRoll);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
  (void)pitchActual;
  pidResetOne(&pidPitch);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw) {
  if (roll == NULL || pitch == NULL || yaw == NULL) return;
  *roll = (int16_t)pidRollRate.output;
  *pitch = (int16_t)pidPitchRate.output;
  *yaw = (int16_t)pidYawRate.output;
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
  if (setpoint == NULL || state == NULL) return 0U;
  float out = 0.0f;
  if (setpoint->mode.z == modeAbs) {
    out += (setpoint->position.z - state->position.z) * 500.0f;
  }
  if (setpoint->mode.z == modeVelocity) {
    out += (setpoint->velocity.z - state->velocity.z) * 300.0f;
  }
  out += (0.0f - state->velocity.z) * 50.0f;
  if (out < 0.0f) out = 0.0f;
  if (out > 65535.0f) out = 65535.0f;
  return (uint16_t)out;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
  if (sensors == NULL || setpoint == NULL || state == NULL || control == NULL) return;

  if (setpoint->thrust == 0U) {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    control->thrust = 0U;
    attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch,
                                  state->attitude.yaw);
    s_desiredYaw = state->attitude.yaw;
    return;
  }

  float rollDesiredRate;
  if (setpoint->mode.roll == modeVelocity) {
    rollDesiredRate = setpoint->attitudeRate.roll;
    pidResetOne(&pidRoll);
  } else {
    rollDesiredRate = pidUpdateOne(&pidRoll, state->attitude.roll,
                                   setpoint->attitude.roll);
  }

  float pitchDesiredRate;
  if (setpoint->mode.pitch == modeVelocity) {
    pitchDesiredRate = setpoint->attitudeRate.pitch;
    pidResetOne(&pidPitch);
  } else {
    pitchDesiredRate = pidUpdateOne(&pidPitch, state->attitude.pitch,
                                    setpoint->attitude.pitch);
  }

  if (setpoint->mode.yaw == modeVelocity) {
    s_desiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
    if (yawMaxDelta != 0.0f) {
      float delta = capAngle(s_desiredYaw - state->attitude.yaw);
      if (delta > yawMaxDelta) {
        s_desiredYaw = state->attitude.yaw + yawMaxDelta;
      } else if (delta < -yawMaxDelta) {
        s_desiredYaw = state->attitude.yaw - yawMaxDelta;
      }
    }
  } else if (setpoint->mode.quat == modeAbs) {
    float w = setpoint->attitudeQuaternion.w;
    float x = setpoint->attitudeQuaternion.x;
    float y = setpoint->attitudeQuaternion.y;
    float z = setpoint->attitudeQuaternion.z;
    float yawRad = atan2f(2.0f * (w * z + x * y),
                          1.0f - 2.0f * (y * y + z * z));
    s_desiredYaw = yawRad * CRAZYFLIE_RAD_TO_DEG_F;
  } else {
    s_desiredYaw = setpoint->attitude.yaw;
  }

  pidResetOne(&pidYaw);
  float yawDesiredRate = pidUpdateOne(&pidYaw, state->attitude.yaw, s_desiredYaw);

  float rollActualRate = sensors->gyro.x;
  float pitchActualRate = -sensors->gyro.y;
  float yawActualRate = sensors->gyro.z;

  attitudeControllerCorrectRatePID(rollActualRate, rollDesiredRate,
                                   pitchActualRate, pitchDesiredRate,
                                   yawActualRate, yawDesiredRate);

  int16_t rollOut = 0;
  int16_t pitchOut = 0;
  int16_t yawOut = 0;
  attitudeControllerGetActuatorOutput(&rollOut, &pitchOut, &yawOut);

  control->roll = rollOut;
  control->pitch = pitchOut;
  control->yaw = (int16_t)(-yawOut);

  if (setpoint->mode.z == modeDisable) {
    control->thrust = setpoint->thrust;
  } else {
    control->thrust = positionControllerUpdate(setpoint, state);
  }
}

/* -------------------------------------------------------------------------
   CRTP Commander RPYT
------------------------------------------------------------------------- */
void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
  if (rollPrime == NULL || pitchPrime == NULL) return;
  float rad = yaw_deg * CRAZYFLIE_DEG_TO_RAD_F;
  float c = cosf(rad);
  float s = sinf(rad);
  *rollPrime = roll * c - pitch * s;
  *pitchPrime = roll * s + pitch * c;
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

  uint16_t rawThrust = values->thrust;
  bool priorityDisable = (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE);
  if (priorityDisable) {
    thrustLocked = true;
  }
  if (rawThrust == 0U) {
    thrustLocked = false;
  }

  if (altHoldMode) {
    if (!commanderModeSet) {
      commanderModeSet = true;
      positionControllerReset();
    }
    setpoint->mode.z = modeVelocity;
    setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
    setpoint->thrust = 0U;
  } else {
    if (commanderModeSet) {
      commanderModeSet = false;
      setpoint->mode.z = modeDisable;
    } else {
      setpoint->mode.z = modeDisable;
    }
    if (thrustLocked || rawThrust < MIN_THRUST) {
      setpoint->thrust = 0U;
    } else {
      setpoint->thrust = rawThrust > MAX_THRUST ? MAX_THRUST : rawThrust;
    }
    setpoint->velocity.z = 0.0f;
  }

  if (posSetMode && rawThrust != 0U) {
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;
    setpoint->position.x = -values->pitch;
    setpoint->position.y = values->roll;
    setpoint->position.z = (float)rawThrust / 1000.0f;
    setpoint->attitude.yaw = values->yaw;
    setpoint->thrust = 0U;
    return;
  }

  if (stabilizationModeYaw == RATE) {
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitudeRate.yaw = -values->yaw;
  } else {
    setpoint->mode.yaw = modeAbs;
    setpoint->attitude.yaw = values->yaw;
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
    return;
  }

  float rollCommand = values->roll;
  float pitchCommand = values->pitch;

  if (yawMode == PLUSMODE) {
    rotateYaw(values->roll, values->pitch, 45.0f, &rollCommand, &pitchCommand);
  } else if (yawMode == CAREFREE) {
    memset(setpoint, 0, sizeof(*setpoint));
    thrustLocked = true;
    return;
  }

  if (stabilizationModeRoll == RATE) {
    setpoint->mode.roll = modeVelocity;
    setpoint->attitudeRate.roll = rollCommand;
  } else {
    setpoint->mode.roll = modeAbs;
    setpoint->attitude.roll = rollCommand;
  }

  if (stabilizationModePitch == RATE) {
    setpoint->mode.pitch = modeVelocity;
    setpoint->attitudeRate.pitch = pitchCommand;
  } else {
    setpoint->mode.pitch = modeAbs;
    setpoint->attitude.pitch = pitchCommand;
  }
}

/* -------------------------------------------------------------------------
   Supervisor
------------------------------------------------------------------------- */
void supervisorInit(void) {
  if (s_supervisorInitDone) return;
  supervisorState = supervisorStateLocked;
  supervisorConditionBits = 0U;
  s_motorRatios[0] = s_motorRatios[1] = 0U;
  s_motorRatios[2] = s_motorRatios[3] = 0U;
  s_motorRPMs[0] = s_motorRPMs[1] = 0;
  s_motorRPMs[2] = s_motorRPMs[3] = 0;
  s_idleThrust = 0U;
  s_crashDetectionGs = 0.0f;
  s_freeFallThreshold = 0.0f;
  s_acceptedTiltAccZ = 0.0f;
  s_acceptedUpsideDownAccZ = 0.0f;
  s_maxTiltTime = 0U;
  s_maxUpsideDownTime = 0U;
  s_tumbleCheckEnabled = false;
  s_autoArming = false;
  s_spinupTimeoutDurationMs = 0U;
  s_spinupStartTick = 0U;
  s_latestArmingTick = 0U;
  s_latestLandingTick = 0U;
  s_seenFlying = false;
  s_recentFlightTick = 0U;
  s_tumbleStartTick = 0U;
  s_notRespondingStartTick = 0U;
  s_crtpEmergencyStop = false;
  s_paramEmergencyStop = false;
  s_supervisorInitDone = true;
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
  return (supervisorConditionBits & SUPERVISOR_CB_CRASHED) != 0U;
}

bool supervisorRequestArming(bool doArm) {
  if (doArm) {
    if (supervisorIsArmed() && supervisorState == supervisorStateArming) return true;
    if (!supervisorCanArm()) return false;
    supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    supervisorState = supervisorStateArming;
    s_spinupStartTick = 0U;
    supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    s_latestArmingTick = tick;
    return true;
  }

  supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
  supervisorState = supervisorStatePreFlChecksPassed;
  s_spinupStartTick = 0U;
  supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
  return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
  if ((supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) != 0U) return false;
  if (!doRecovery) {
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    supervisorState = supervisorStateCrashed;
    return true;
  }
  supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
  if (supervisorState == supervisorStateCrashed) {
    supervisorState = supervisorStatePreFlChecksPassed;
  }
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
  if (supervisorCanArm()) info |= (uint16_t)(1U << 0);
  if (supervisorIsArmed()) info |= (uint16_t)(1U << 1);
  if (s_autoArming) info |= (uint16_t)(1U << 2);
  if (supervisorCanFly()) info |= (uint16_t)(1U << 3);
  if ((supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) != 0U) info |= (uint16_t)(1U << 4);
  if ((supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) != 0U) info |= (uint16_t)(1U << 5);
  if (supervisorState == supervisorStateLocked) info |= (uint16_t)(1U << 6);
  if (supervisorIsCrashed()) info |= (uint16_t)(1U << 7);
  if (s_trajectoryFlying) info |= (uint16_t)(1U << 8);
  if (s_trajectoryFinished) info |= (uint16_t)(1U << 9);
  if (s_trajectoryDisabled) info |= (uint16_t)(1U << 10);
  if ((supervisorConditionBits & SUPERVISOR_CB_DECK_FAULT) != 0U) info |= (uint16_t)(1U << 11);
  supervisorLog.info = (uint32_t)info;
  return info;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick) {
  if (motorRatios == NULL) return false;
  bool anyAboveIdle = false;
  for (int i = 0; i < 4; ++i) {
    if (motorRatios[i] > idleThrust) {
      anyAboveIdle = true;
      break;
    }
  }
  if (anyAboveIdle) {
    s_recentFlightTick = currentTick;
    s_seenFlying = true;
    return true;
  }
  if (!s_seenFlying) return false;
  uint32_t elapsed = currentTick - s_recentFlightTick;
  return elapsed < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling) {
  if (isFreeFalling != NULL) *isFreeFalling = false;

  float accNorm = sqrtf(accX * accX + accY * accY + accZ * accZ);

  bool freeFall = (fabsf(accX) < freeFallThreshold) &&
                  (fabsf(accY) < freeFallThreshold) &&
                  (fabsf(accZ) < freeFallThreshold);
  if (freeFall) {
    if (isFreeFalling != NULL) *isFreeFalling = true;
    s_tumbleStartTick = 0U;
    supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
    supervisorState = supervisorStateExceptFreeFall;
    return false;
  }

  supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

  if (crashDetectionGs > 0.0f && fabsf(accNorm - 1.0f) > crashDetectionGs) {
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    supervisorState = supervisorStateCrashed;
  }

  if (!tumbleCheckEnabled) {
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    s_tumbleStartTick = 0U;
    return false;
  }

  bool tumbled = false;
  if (accZ < acceptedTiltAccZ) {
    uint32_t timeout = maxTiltTime;
    if (accZ < acceptedUpsideDownAccZ) {
      timeout = maxUpsideDownTime;
    }
    if (s_tumbleStartTick == 0U) {
      s_tumbleStartTick = currentTick;
    } else if ((currentTick - s_tumbleStartTick) >= timeout) {
      tumbled = true;
      supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    }
  } else {
    s_tumbleStartTick = 0U;
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
  }

  return tumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick) {
  if (lastNotificationTick == 0U) return true;
  uint32_t elapsed = currentTick - lastNotificationTick;
  return elapsed <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration) {
  if (latestArmingTick == 0U) return false;
  if (state != supervisorStateReadyToFly) return false;
  return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration) {
  if (latestLandingTick == 0U) return false;
  return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed) {
  s_crtpEmergencyStop = crtpEmergencyStop;
  s_paramEmergencyStop = paramEmergencyStop;
  if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
    supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
  }
  supervisorLog.info = supervisorConditionBits;
  return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t conditionBits,
                                SupervisorState state) {
  if (setpoint == NULL) return;
  (void)conditionBits;
  switch (state) {
    case supervisorStateArming:
    case supervisorStateReadyToFly:
    case supervisorStateFlying:
    case supervisorStateLanded:
      return;
    case supervisorStateWarningLevelOut: {
      StabilizationMode zMode = setpoint->mode.z;
      float zVelocity = setpoint->velocity.z;
      float zPosition = setpoint->position.z;
      uint16_t zThrust = setpoint->thrust;
      memset(setpoint, 0, sizeof(*setpoint));
      setpoint->mode.x = modeDisable;
      setpoint->mode.y = modeDisable;
      setpoint->mode.roll = modeAbs;
      setpoint->mode.pitch = modeAbs;
      setpoint->mode.yaw = modeVelocity;
      setpoint->attitude.roll = 0.0f;
      setpoint->attitude.pitch = 0.0f;
      setpoint->attitudeRate.yaw = 0.0f;
      setpoint->mode.z = zMode;
      setpoint->velocity.z = zVelocity;
      setpoint->position.z = zPosition;
      setpoint->thrust = zThrust;
      return;
    }
    default:
      memset(setpoint, 0, sizeof(*setpoint));
      return;
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
                           uint32_t currentTick) {
  if (motorRPMs == NULL || !canFly) {
    s_notRespondingStartTick = 0U;
    supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return false;
  }
  bool anyBelow = false;
  for (int i = 0; i < 4; ++i) {
    if (motorRPMs[i] < rpmThreshold) {
      anyBelow = true;
      break;
    }
  }
  if (!anyBelow) {
    s_notRespondingStartTick = 0U;
    supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return false;
  }
  if (s_notRespondingStartTick == 0U) {
    s_notRespondingStartTick = currentTick;
  } else if ((currentTick - s_notRespondingStartTick) >= rpmCheckDurationMs) {
    supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return true;
  }
  return false;
}

void supervisorSetSensorData(const SensorData *sensors) {
  if (sensors == NULL) return;
  s_supervisorSensors = *sensors;
  gyro.x = sensors->gyro.x;
  gyro.y = sensors->gyro.y;
  gyro.z = sensors->gyro.z;
  acc.x = sensors->acc.x;
  acc.y = sensors->acc.y;
  acc.z = sensors->acc.z;
  baro.pressure = sensors->baroPressure;
  baro.temp = sensors->baroTemperature;
  baro.asl = sensors->baroAsl;
  supervisorLog.accNorm = sqrtf(sensors->acc.x * sensors->acc.x +
                                 sensors->acc.y * sensors->acc.y +
                                 sensors->acc.z * sensors->acc.z);
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
  if (motorRatios == NULL) return;
  for (int i = 0; i < 4; ++i) s_motorRatios[i] = motorRatios[i];
  s_idleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
  if (motorRPMs == NULL) return;
  for (int i = 0; i < 4; ++i) s_motorRPMs[i] = motorRPMs[i];
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled) {
  s_crashDetectionGs = crashDetectionGs;
  s_freeFallThreshold = freeFallThreshold;
  s_acceptedTiltAccZ = acceptedTiltAccZ;
  s_acceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
  s_maxTiltTime = maxTiltTime;
  s_maxUpsideDownTime = maxUpsideDownTime;
  s_tumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
  s_autoArming = autoArming;
  s_spinupTimeoutDurationMs = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep) {
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

  bool freeFall = false;
  bool tumbled = isTumbledCheck(
      s_supervisorSensors.acc.x, s_supervisorSensors.acc.y, s_supervisorSensors.acc.z,
      s_crashDetectionGs, s_freeFallThreshold,
      s_acceptedTiltAccZ, s_acceptedUpsideDownAccZ,
      s_maxTiltTime, s_maxUpsideDownTime,
      s_tumbleCheckEnabled, tick, &freeFall);
  if (freeFall) supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
  if (tumbled) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;

  bool flying = isFlyingCheck(s_motorRatios, s_idleThrust, tick);
  if (flying) {
    supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
  }

  bool wdtHealthy = checkEmergencyStopWatchdog(tick, 0U);
  (void)wdtHealthy;

  uint32_t age = commanderGetInactivityTime();
  if (age > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
    supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
  }
  if (age > COMMANDER_WDT_TIMEOUT_STABILIZE) {
    supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  } else {
    supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  }

  if (supervisorState == supervisorStateArming) {
    if (s_spinupStartTick == 0U) {
      s_spinupStartTick = tick;
    }
    if ((tick - s_spinupStartTick) >= s_spinupTimeoutDurationMs) {
      supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
    }
  } else {
    s_spinupStartTick = 0U;
    supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
  }

  if (supervisorState == supervisorStatePreFlChecksPassed && s_autoArming) {
    supervisorRequestArming(true);
  }

  bool armedAllowed = supervisorState == supervisorStateArming ||
                      supervisorState == supervisorStateReadyToFly ||
                      supervisorState == supervisorStateFlying ||
                      supervisorState == supervisorStateWarningLevelOut ||
                      supervisorState == supervisorStateLanded;
  if (!armedAllowed && supervisorIsArmed()) {
    supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
  }

  supervisorLog.info = supervisorConditionBits;
}

/* -------------------------------------------------------------------------
   Estimator and Commander arbitration
------------------------------------------------------------------------- */
bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
  if (measurement == NULL) return false;
  if (s_estCount >= ESTIMATOR_FIFO_CAPACITY) return false;
  s_estFifo[s_estTail] = *measurement;
  s_estTail = (uint8_t)((s_estTail + 1U) % ESTIMATOR_FIFO_CAPACITY);
  s_estCount++;
  return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
  if (measurement == NULL) return false;
  if (s_estCount == 0U) return false;
  *measurement = s_estFifo[s_estHead];
  s_estHead = (uint8_t)((s_estHead + 1U) % ESTIMATOR_FIFO_CAPACITY);
  s_estCount--;
  return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
  EstimatorMeasurement m;
  while (estimatorDequeue(&m)) {
    if (m.type == MeasurementTypeGyroscope) {
      s_lastGyro[0] = m.data[0];
      s_lastGyro[1] = m.data[1];
      s_lastGyro[2] = m.data[2];
    } else if (m.type == MeasurementTypeAcceleration) {
      s_lastAcc[0] = m.data[0];
      s_lastAcc[1] = m.data[1];
      s_lastAcc[2] = m.data[2];
    } else if (m.type == MeasurementTypeBarometer) {
      s_lastBaro[0] = m.data[0];
      s_lastBaro[1] = m.data[1];
      s_lastBaro[2] = m.data[2];
    } else if (m.type == MeasurementTypeTOF) {
      s_lastTof[0] = m.data[0];
      s_lastTof[1] = m.data[1];
      s_lastTof[2] = m.data[2];
    }
  }

  if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
    float dt = 1.0f / (float)SENSFUSION_RATE_HZ;
    sensfusion6UpdateQ(s_lastGyro[0], s_lastGyro[1], s_lastGyro[2],
                       s_lastAcc[0], s_lastAcc[1], s_lastAcc[2], dt);
    sensfusion6GetQuaternion(&stateEstimate.qw, &stateEstimate.qx,
                              &stateEstimate.qy, &stateEstimate.qz);
    sensfusion6GetEulerRPY(&stateEstimate.roll, &stateEstimate.pitch,
                            &stateEstimate.yaw);
    float accZ = sensfusion6GetAccZWithoutGravity(s_lastAcc[0], s_lastAcc[1],
                                                  s_lastAcc[2]);
    s_estVerticalVelocity += accZ * 9.81f * dt;
  }

  if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
    float dt = 1.0f / (float)POSITION_RATE_HZ;
    s_estPositionZ += s_estVerticalVelocity * dt;
  }
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
  if (setpoint == NULL) return false;
  if (priority != COMMANDER_PRIORITY_DISABLE && priority < s_activePriority) {
    return false;
  }
  s_activeSetpoint = *setpoint;
  s_activePriority = priority;
  s_lastUpdateTick = tick;
  if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
    s_highLevelTrajectoryActive = false;
    s_trajectoryFlying = false;
    s_trajectoryFinished = true;
  }
  return true;
}

void commanderRelaxPriority(void) {
  s_activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
  if (tick < s_lastUpdateTick) return 0U;
  return tick - s_lastUpdateTick;
}

int commanderGetActivePriority(void) {
  return s_activePriority;
}

/* -------------------------------------------------------------------------
   Stabilizer private init helpers
------------------------------------------------------------------------- */
static void sensorsInit(void) {
  gyro.x = gyro.y = gyro.z = 0.0f;
  acc.x = acc.y = acc.z = 0.0f;
  baro.asl = baro.temp = baro.pressure = 0.0f;
}

static void stateEstimatorInit(void) {
  sensfusion6Init();
  s_estHead = s_estTail = s_estCount = 0U;
  s_estPositionZ = 0.0f;
  s_estVerticalVelocity = 0.0f;
}

static void controllerInit(void) {
  attitudeControllerInit(1.0f / (float)ATTITUDE_RATE_HZ);
}

static void powerDistributionInit(void) {
  s_stabilizerMotorPower.m1 = 0;
  s_stabilizerMotorPower.m2 = 0;
  s_stabilizerMotorPower.m3 = 0;
  s_stabilizerMotorPower.m4 = 0;
  s_filteredBatteryVoltage = 0.0f;
}

static void motorsInit(void) {
  motor.m1req = motor.m2req = 0U;
  motor.m3req = motor.m4req = 0U;
}

static void collisionAvoidanceInit(void) {
  /* No persistent state in frozen API. */
}

static void sensorsWaitDataReady(void) {
  /* Host model: data is already available from supervisorSetSensorData. */
}

static void sensorsAcquire(void) {
  /* Use the most recently injected supervisor sensor sample. */
}

static void collisionAvoidanceUpdateSetpoint(Setpoint *setpoint) {
  (void)setpoint;
}

static void setMotorRatiosZero(void) {
  motor.m1req = motor.m2req = motor.m3req = motor.m4req = 0U;
  s_stabilizerMotorPower.m1 = 0;
  s_stabilizerMotorPower.m2 = 0;
  s_stabilizerMotorPower.m3 = 0;
  s_stabilizerMotorPower.m4 = 0;
}

void stabilizerInit(void) {
  if (s_stabilizerInitDone) return;
  sensorsInit();
  stateEstimatorInit();
  controllerInit();
  powerDistributionInit();
  motorsInit();
  collisionAvoidanceInit();
  s_stabilizerInitDone = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
  if (setpoint == NULL) return false;
  s_highLevelSetpoint = *setpoint;
  s_highLevelPending = true;
  return true;
}

void stabilizerTask(void) {
  if (!s_stabilizerInitDone) stabilizerInit();

  uint32_t step = s_stabilizerStep;
  s_stabilizerStep++;

  sensorsWaitDataReady();
  sensorsAcquire();

  estimatorComplementary(step);

  if (s_highLevelPending) {
    commanderSetSetpoint(&s_highLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
    s_highLevelPending = false;
  }
  s_stabilizerSetpoint = s_activeSetpoint;

  supervisorUpdate(step);

  if (healthShallWeRunTest()) {
    healthRunTests(&s_supervisorSensors);
    return;
  }

  if (!supervisorCanFly() || !supervisorAreMotorsAllowedToRun()) {
    setMotorRatiosZero();
    return;
  }

  collisionAvoidanceUpdateSetpoint(&s_stabilizerSetpoint);
  supervisorOverrideSetpoint(&s_stabilizerSetpoint, supervisorConditionBits,
                             supervisorState);

  memset(&s_stabilizerControl, 0, sizeof(s_stabilizerControl));
  s_stabilizerControl.controlMode = controlModeLegacy;
  controllerPid(&s_supervisorSensors, &s_stabilizerSetpoint,
                &s_stabilizerState, &s_stabilizerControl,
                0.0f, 1.0f / (float)ATTITUDE_RATE_HZ);

  powerDistribution(&s_stabilizerControl, &s_stabilizerMotorPower);

  s_filteredBatteryVoltage = batteryCompensation(s_batterySupplyVoltage,
                                                  s_filteredBatteryVoltage, 0.01f);

  int32_t capped[4] = {
    s_stabilizerMotorPower.m1,
    s_stabilizerMotorPower.m2,
    s_stabilizerMotorPower.m3,
    s_stabilizerMotorPower.m4
  };
  powerDistributionCap(capped, 65535, 0);
  s_stabilizerMotorPower.m1 = capped[0];
  s_stabilizerMotorPower.m2 = capped[1];
  s_stabilizerMotorPower.m3 = capped[2];
  s_stabilizerMotorPower.m4 = capped[3];

  motor.m1req = (uint16_t)s_stabilizerMotorPower.m1;
  motor.m2req = (uint16_t)s_stabilizerMotorPower.m2;
  motor.m3req = (uint16_t)s_stabilizerMotorPower.m3;
  motor.m4req = (uint16_t)s_stabilizerMotorPower.m4;
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output) {
  if (state == NULL || sensors == NULL || output == NULL) return;
  for (int i = 0; i < 3; ++i) {
    output->position_mm[i] = (int32_t)(state->position.x * 1000.0f);
  }
  output->position_mm[0] = (int32_t)(state->position.x * 1000.0f);
  output->position_mm[1] = (int32_t)(state->position.y * 1000.0f);
  output->position_mm[2] = (int32_t)(state->position.z * 1000.0f);

  output->velocity_mms[0] = (int32_t)(state->velocity.x * 1000.0f);
  output->velocity_mms[1] = (int32_t)(state->velocity.y * 1000.0f);
  output->velocity_mms[2] = (int32_t)(state->velocity.z * 1000.0f);

  output->acceleration_mms2[0] = (int32_t)(state->acc.x * 9810.0f);
  output->acceleration_mms2[1] = (int32_t)(state->acc.y * 9810.0f);
  output->acceleration_mms2[2] = (int32_t)((state->acc.z + 1.0f) * 9810.0f);

  output->gyro_millirad_s[0] =
      sensors->gyro.x * CRAZYFLIE_DEG_TO_RAD_F * 1000.0f;
  output->gyro_millirad_s[1] =
      -sensors->gyro.y * CRAZYFLIE_DEG_TO_RAD_F * 1000.0f;
  output->gyro_millirad_s[2] =
      sensors->gyro.z * CRAZYFLIE_DEG_TO_RAD_F * 1000.0f;

  output->quatCompressed = quatCompress(state->attitudeQuaternion.w,
                                         state->attitudeQuaternion.x,
                                         state->attitudeQuaternion.y,
                                         state->attitudeQuaternion.z);
}

bool rateSupervisorValidate(uint32_t measuredRate) {
  return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void) {
  if (s_rateErrorStartTick == 0U) {
    s_rateErrorStartTick = tick;
    return;
  }
  if ((tick - s_rateErrorStartTick) >= 2000U) {
    if (s_sensorActive) {
      /* Assert/error state is represented by a locked/crashed safe state. */
      supervisorState = supervisorStateLocked;
      supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    }
    s_rateErrorStartTick = 0U;
  }
}

/* -------------------------------------------------------------------------
   Health
------------------------------------------------------------------------- */
bool healthShallWeRunTest(void) {
  bool consumed = false;
  if (s_pendingPropTest) {
    s_pendingPropTest = false;
    healthTestState = configureAcc;
    s_healthNoiseCount = 0U;
    s_healthMotorIndex = 0U;
    consumed = true;
  } else if (s_pendingBatteryTest) {
    s_pendingBatteryTest = false;
    healthTestState = testBattery;
    s_healthBatteryTick = 0U;
    s_healthRestartStartTick = 0U;
    consumed = true;
  }
  if (consumed) return true;
  return healthTestState != testDone;
}

void healthRunTests(const SensorData *sensorData) {
  if (sensorData == NULL) return;

  switch (healthTestState) {
    case configureAcc:
      motorPass = 0U;
      healthLog.motorTestCount = 0U;
      s_healthNoiseCount = 0U;
      s_healthMotorIndex = 0U;
      s_healthIdleVoltage = s_batterySupplyVoltage;
      healthTestState = measureNoiseFloor;
      break;
    case measureNoiseFloor:
      if (s_healthNoiseCount < PROPTEST_NBR_OF_VARIANCE_VALUES) {
        s_healthNoiseBuffer[s_healthNoiseCount] = sensorData->acc.x;
        s_healthNoiseCount++;
      }
      if (s_healthNoiseCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
        s_healthNoiseVariance = variance(s_healthNoiseBuffer,
                                          PROPTEST_NBR_OF_VARIANCE_VALUES);
        s_healthMotorIndex = 0U;
        healthTestState = measureProp;
      }
      break;
    case measureProp:
      evaluatePropTest(0.0f, 0.0f, sensorData->acc.z,
                       (uint8_t)s_healthMotorIndex);
      s_healthMotorIndex++;
      if (s_healthMotorIndex >= 4U) {
        healthTestState = evaluatePropResult;
      }
      break;
    case evaluatePropResult:
      healthTestState = testDone;
      break;
    case testBattery:
      s_healthBatteryTick++;
      if (s_healthBatteryTick == 1U) {
        s_healthMinLoadedVoltage = s_healthIdleVoltage;
      } else if (s_healthBatteryTick >= 2U && s_healthBatteryTick <= 49U) {
        float v = s_healthIdleVoltage - 0.01f * (float)s_healthBatteryTick;
        if (v < s_healthMinLoadedVoltage) s_healthMinLoadedVoltage = v;
      }
      if (s_healthBatteryTick >= 50U) {
        batterySag = s_healthIdleVoltage - s_healthMinLoadedVoltage;
        batteryPass = batterySag <= 1.0f ? 1U : 0U;
        healthTestState = testDone;
      }
      break;
    case evaluateBatResult:
      healthTestState = testDone;
      break;
    case restartBatTest:
      if (s_healthRestartStartTick == 0U) {
        s_healthRestartStartTick = tick;
      } else if ((tick - s_healthRestartStartTick) >= 2000U) {
        s_healthBatteryTick = 0U;
        s_healthRestartStartTick = 0U;
        healthTestState = testBattery;
      }
      break;
    case testDone:
      break;
  }

  healthLog.motorPass = motorPass;
  healthLog.batteryPass = batteryPass;
  healthLog.batterySag = batterySag;
}

void healthRequestPropTest(void) {
  s_pendingPropTest = true;
}

void healthRequestBatteryTest(void) {
  s_pendingBatteryTest = true;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex) {
  if (highThreshold == 0.0f) return true;
  if (motorIndex >= 4U) return false;
  if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
    motorPass |= (uint8_t)(1U << motorIndex);
    healthLog.motorPass = motorPass;
    return true;
  }
  motorPass &= (uint8_t)~(1U << motorIndex);
  healthLog.motorPass = motorPass;
  healthLog.motorTestCount++;
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
  return sumSq - (sum * sum / (float)length);
}

/* -------------------------------------------------------------------------
   CRTP transport
------------------------------------------------------------------------- */
static bool nopSend(CrtpPacket *packet) {
  (void)packet;
  return false;
}

static bool nopReceive(CrtpPacket *packet) {
  (void)packet;
  return false;
}

static bool nopIsConnected(void) {
  return true;
}

static void nopSetEnable(bool enable) {
  (void)enable;
}

static void nopReset(void) {
}

static CrtpLink s_nopLink = {
  nopSend,
  nopReceive,
  nopIsConnected,
  nopSetEnable,
  nopReset
};

static CrtpLink *s_currentLink = &s_nopLink;
static bool s_linkIsNop = true;

void crtpInit(void) {
  if (s_crtpInitDone) return;
  memset(s_txQueue, 0, sizeof(s_txQueue));
  s_txHead = s_txTail = s_txCount = 0U;
  memset(s_rxQueues, 0, sizeof(s_rxQueues));
  for (int i = 0; i < CRTP_NBR_OF_PORTS; ++i) {
    s_rxHead[i] = 0U;
    s_rxTail[i] = 0U;
    s_rxCount[i] = 0U;
    s_rxQueueCreated[i] = false;
    s_portCallbacks[i] = NULL;
  }
  s_currentLink = &s_nopLink;
  s_linkIsNop = true;
  s_crtpErrorState = 0U;
  s_txRetryTick = 0U;
  s_lastStatsTick = tick;
  s_rxPacketCounter = 0U;
  s_txPacketCounter = 0U;
  s_crtpInitDone = true;
}

void crtpInitTaskQueue(uint8_t port) {
  if (port >= CRTP_NBR_OF_PORTS) return;
  if (s_rxQueueCreated[port]) {
    s_crtpErrorState = 1U;
    return;
  }
  memset(s_rxQueues[port], 0, sizeof(s_rxQueues[port]));
  s_rxHead[port] = 0U;
  s_rxTail[port] = 0U;
  s_rxCount[port] = 0U;
  s_rxQueueCreated[port] = true;
}

bool crtpSendPacket(const CrtpPacket *packet) {
  if (packet == NULL) return false;
  if (!s_crtpInitDone) crtpInit();
  if (s_txCount >= CRTP_TX_QUEUE_SIZE) return false;
  s_txQueue[s_txTail] = *packet;
  s_txTail = (uint16_t)((s_txTail + 1U) % CRTP_TX_QUEUE_SIZE);
  s_txCount++;
  s_txPacketCounter++;
  return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
  return crtpSendPacket(packet);
}

static bool rxQueuePop(uint8_t port, CrtpPacket *packet) {
  if (port >= CRTP_NBR_OF_PORTS || packet == NULL) return false;
  if (s_rxCount[port] == 0U) return false;
  *packet = s_rxQueues[port][s_rxHead[port]];
  s_rxHead[port] = (uint8_t)((s_rxHead[port] + 1U) % CRTP_RX_QUEUE_SIZE);
  s_rxCount[port]--;
  return true;
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
  return rxQueuePop(port, packet);
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet) {
  return rxQueuePop(port, packet);
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet,
                           uint32_t wait_ms) {
  (void)wait_ms;
  return rxQueuePop(port, packet);
}

void crtpRxTask(void) {
  if (!s_crtpInitDone) crtpInit();
  if (s_linkIsNop || s_currentLink == NULL || s_currentLink->receivePacket == NULL) return;
  CrtpPacket packet;
  if (!s_currentLink->receivePacket(&packet)) return;
  if (packet.port >= CRTP_NBR_OF_PORTS) return;

  bool queued = false;
  if (s_rxQueueCreated[packet.port] && s_rxCount[packet.port] < CRTP_RX_QUEUE_SIZE) {
    s_rxQueues[packet.port][s_rxTail[packet.port]] = packet;
    s_rxTail[packet.port] = (uint8_t)((s_rxTail[packet.port] + 1U) % CRTP_RX_QUEUE_SIZE);
    s_rxCount[packet.port]++;
    queued = true;
  }
  if (s_portCallbacks[packet.port] != NULL) {
    s_portCallbacks[packet.port](&packet);
  }
  if (!queued && s_portCallbacks[packet.port] == NULL) {
    /* dropped */
  }
  s_rxPacketCounter++;
}

void crtpTxTask(void) {
  if (!s_crtpInitDone) crtpInit();
  if (s_linkIsNop || s_currentLink == NULL || s_currentLink->sendPacket == NULL) return;
  if (s_txCount == 0U) return;
  if (s_txRetryTick > tick) return;
  CrtpPacket packet = s_txQueue[s_txHead];
  if (s_currentLink->sendPacket(&packet)) {
    s_txHead = (uint16_t)((s_txHead + 1U) % CRTP_TX_QUEUE_SIZE);
    s_txCount--;
    s_txRetryTick = 0U;
  } else {
    s_txRetryTick = tick + 10U;
  }
}

void crtpSetLink(CrtpLink *newLink) {
  if (s_currentLink != NULL && s_currentLink->setEnable != NULL) {
    s_currentLink->setEnable(false);
  }
  if (newLink == NULL) {
    s_currentLink = &s_nopLink;
    s_linkIsNop = true;
  } else {
    s_currentLink = newLink;
    s_linkIsNop = false;
  }
  if (s_currentLink != NULL && s_currentLink->setEnable != NULL) {
    s_currentLink->setEnable(true);
  }
}

void crtpReset(void) {
  if (!s_crtpInitDone) crtpInit();
  memset(s_txQueue, 0, sizeof(s_txQueue));
  s_txHead = s_txTail = s_txCount = 0U;
  s_txRetryTick = 0U;
  if (s_currentLink != NULL && s_currentLink->reset != NULL) {
    s_currentLink->reset();
  }
}

bool crtpIsConnected(void) {
  if (s_currentLink == NULL || s_currentLink->isConnected == NULL) return true;
  return s_currentLink->isConnected();
}

uint32_t crtpGetFreeTxQueuePackets(void) {
  return CRTP_TX_QUEUE_SIZE - (uint32_t)s_txCount;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
  if (port >= CRTP_NBR_OF_PORTS) return;
  s_portCallbacks[port] = callback;
}

void updateStats(void) {
  if (!s_crtpInitDone) crtpInit();
  if (s_lastStatsTick == 0U) {
    s_lastStatsTick = tick;
    return;
  }
  uint32_t elapsed = tick - s_lastStatsTick;
  if (elapsed >= 500U) {
    /* Compute rate evidence and clear current counters. */
    s_rxPacketCounter = 0U;
    s_txPacketCounter = 0U;
    s_lastStatsTick = tick;
  }
}

/* -------------------------------------------------------------------------
   Deck discovery and logs
------------------------------------------------------------------------- */
uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
  if (decks == NULL || capacity == 0U) return 0U;

  static const uint8_t i2cAddrs[3] = {0x20U, 0x21U, 0x22U};
  static const uint64_t romIds[2] = {
    0x1122334455667788ULL,
    0x0F0E0D0C0B0A0908ULL
  };

  uint8_t count = 0U;
  for (int i = 0; i < 3 && count < capacity; ++i) {
    bool duplicate = false;
    for (uint8_t j = 0; j < count; ++j) {
      if (decks[j].foundByI2C && decks[j].i2cAddress == i2cAddrs[i]) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) continue;
    decks[count].foundByI2C = true;
    decks[count].foundByOneWire = false;
    decks[count].i2cAddress = i2cAddrs[i];
    decks[count].oneWireRomId = 0U;
    count++;
  }

  for (int i = 0; i < 2 && count < capacity; ++i) {
    bool duplicate = false;
    for (uint8_t j = 0; j < count; ++j) {
      if (decks[j].foundByOneWire && decks[j].oneWireRomId == romIds[i]) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) continue;
    decks[count].foundByI2C = false;
    decks[count].foundByOneWire = true;
    decks[count].i2cAddress = 0U;
    decks[count].oneWireRomId = romIds[i];
    count++;
  }

  return count;
}
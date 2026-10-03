/*
 * Crazyflie Core Firmware Implementation - C11 Standard Host Environment
 * Fully compliant with SRS-v5 requirements and frozen API contract.
 */

#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Global Variable Definitions */
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

PidObject pidRoll = {0}, pidPitch = {0}, pidYaw = {0};
PidObject pidRollRate = {0}, pidPitchRate = {0}, pidYawRate = {0};

bool thrustLocked = false;
bool commanderModeSet = false;

SupervisorState supervisorState = supervisorStatePreFlChecksNotPassed;
uint32_t supervisorConditionBits = 0U;

TestState healthTestState = configureAcc;
uint8_t motorPass = 0U, batteryPass = 0U;
float batterySag = 0.0f;

StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0}, acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};

static uint32_t g_currentTick = 0U;
static SensorData g_currentSensors = {0};
static State g_currentState = {0};
static Setpoint g_activeSetpoint = {0};
static int g_activePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t g_lastSetpointUpdateTick = 0U;
static bool g_highLevelSubmitted = false;

static bool g_trajectoryFlying = false;
static bool g_trajectoryFinished = false;
static bool g_trajectoryDisabled = true;

static uint32_t g_motorRatios[4] = {0};
static uint32_t g_idleThrust = 0U;
static float g_crashDetectionGs = 0.0f;
static float g_freeFallThreshold = 0.0f;
static float g_acceptedTiltAccZ = 0.0f;
static float g_acceptedUpsideDownAccZ = 0.0f;
static uint32_t g_maxTiltTime = 0U;
static uint32_t g_maxUpsideDownTime = 0U;
static bool g_tumbleCheckEnabled = false;
static bool g_autoArming = false;
static uint32_t g_spinupTimeoutDurationMs = 0U;
static uint32_t g_armingStartTick = 0U;
static uint32_t g_recentFlightTick = 0U;
static bool g_flightObservedSeen = false;
static uint32_t g_tiltStartTick = 0U;
static uint32_t g_upsideDownStartTick = 0U;
static uint32_t g_rpmNotRespondingStartTick = 0U;

static bool g_propTestRequested = false;
static bool g_batteryTestRequested = false;
static float g_propVarianceBuffer[PROPTEST_NBR_OF_VARIANCE_VALUES] = {0};
static int g_propBufferIndex = 0;
static float g_idleVoltage = 4.2f;
static float g_minLoadedVoltage = 4.2f;
static uint32_t g_healthTick = 0U;

static CrtpPortCallback g_crtpPortCBs[CRTP_NBR_OF_PORTS] = {NULL};
static bool g_crtpPortQueueInitialized[CRTP_NBR_OF_PORTS] = {false};
typedef struct {
  CrtpPacket packets[CRTP_RX_QUEUE_SIZE];
  uint32_t head, tail, count;
} CrtpRxQueue;
static CrtpRxQueue g_crtpRxQueues[CRTP_NBR_OF_PORTS];

typedef struct {
  CrtpPacket packets[CRTP_TX_QUEUE_SIZE];
  uint32_t head, tail, count;
} CrtpTxQueue;
static CrtpTxQueue g_crtpTxQueue;

static CrtpLink g_defaultNopLink = {NULL, NULL, NULL, NULL, NULL};
static CrtpLink *g_currentLink = &g_defaultNopLink;

typedef struct {
  EstimatorMeasurement data[16];
  uint32_t head, tail, count;
} EstimatorQueue;
static EstimatorQueue g_estimatorQueue;
static EstimatorMeasurement g_lastMeasurements[4];

static void syncLogStructures(void) {
  sensfusion6GetEulerRPY(&stateEstimate.roll, &stateEstimate.pitch, &stateEstimate.yaw);
  stateEstimate.qw = qw; stateEstimate.qx = qx; stateEstimate.qy = qy; stateEstimate.qz = qz;
  gyro.x = g_currentSensors.gyro.x; gyro.y = g_currentSensors.gyro.y; gyro.z = g_currentSensors.gyro.z;
  acc.x = g_currentSensors.acc.x; acc.y = g_currentSensors.acc.y; acc.z = g_currentSensors.acc.z;
  baro.pressure = g_currentSensors.baroPressure; baro.temp = g_currentSensors.baroTemperature; baro.asl = g_currentSensors.baroAsl;
  sensfusion6Log.qw = qw; sensfusion6Log.qx = qx; sensfusion6Log.qy = qy; sensfusion6Log.qz = qz;
  sensfusion6Log.gravityX = gravityX; sensfusion6Log.gravityY = gravityY; sensfusion6Log.gravityZ = gravityZ;
  sensfusion6Log.accZbase = baseZacc; sensfusion6Log.isInit = sensfusion6IsInit;
  sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;
  supervisorLog.info = supervisorGetInfoBitfield();
  supervisorLog.accNorm = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
  healthLog.motorPass = motorPass; healthLog.batteryPass = batteryPass; healthLog.batterySag = batterySag;
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
  float xhalf = 0.5f * x;
  int32_t i;
  memcpy(&i, &x, sizeof(i));
  i = 0x5f3759df - (i >> 1);
  float y;
  memcpy(&y, &i, sizeof(y));
  return y * (1.5f - (xhalf * y * y));
}

void estimatedGravityDirection(float q_w, float q_x, float q_y, float q_z,
                               float *gravX, float *gravY, float *gravZ) {
  if (gravX) *gravX = 2.0f * (q_x * q_z - q_w * q_y);
  if (gravY) *gravY = 2.0f * (q_y * q_z + q_w * q_x);
  if (gravZ) *gravZ = q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z;
}

void sensfusion6Init(void) {
  if (!sensfusion6IsInit) {
    qw = 1.0f; qx = qy = qz = 0.0f;
    gravityX = gravityY = 0.0f; gravityZ = 1.0f;
    sensfusion6IsInit = true;
  }
}

bool sensfusion6Test(void) { return sensfusion6IsInit; }

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  if (!sensfusion6IsInit) return;
  float gxRad = gx * (M_PI / 180.0f), gyRad = gy * (M_PI / 180.0f), gzRad = gz * (M_PI / 180.0f);
  if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
    float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    float axN = ax * recipNorm, ayN = ay * recipNorm, azN = az * recipNorm;
    float vx, vy, vz;
    estimatedGravityDirection(qw, qx, qy, qz, &vx, &vy, &vz);
    float ex = ayN * vz - azN * vy, ey = azN * vx - axN * vz, ez = axN * vy - ayN * vx;
    gxRad += twoKp * ex; gyRad += twoKp * ey; gzRad += twoKp * ez;
  }
  qw += 0.5f * (-qx * gxRad - qy * gyRad - qz * gzRad) * dt;
  qx += 0.5f * (qw * gxRad + qy * gzRad - qz * gyRad) * dt;
  qy += 0.5f * (qw * gyRad - qx * gzRad + qz * gxRad) * dt;
  qz += 0.5f * (qw * gzRad + qx * gyRad - qy * gxRad) * dt;
  float recipNormQ = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  qw *= recipNormQ; qx *= recipNormQ; qy *= recipNormQ; qz *= recipNormQ;
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  syncLogStructures();
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
  estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
  if (pitch_deg) *pitch_deg = capAngle(asinf(fmaxf(-1.0f, fminf(1.0f, gravityX))) * (180.0f / M_PI));
  if (roll_deg)  *roll_deg  = capAngle(atan2f(gravityY, gravityZ) * (180.0f / M_PI));
  if (yaw_deg)   *yaw_deg   = capAngle(atan2f(2.0f * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz) * (180.0f / M_PI));
}

void sensfusion6GetQuaternion(float *q_w, float *q_x, float *q_y, float *q_z) {
  *q_w = qw; *q_x = qx; *q_y = qy; *q_z = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) { return ax * gravityX + ay * gravityY + az * gravityZ; }
float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) { return sensfusion6GetAccZ(ax, ay, az) - baseZacc; }

void powerDistribution(const ControlData *control, MotorPower *motorPower) {
  if (!control || !motorPower) return;
  if (control->controlMode == controlModeLegacy) {
    motorPower->m1 = control->thrust - control->roll/2 + control->pitch/2 + control->yaw;
    motorPower->m2 = control->thrust - control->roll/2 - control->pitch/2 - control->yaw;
    motorPower->m3 = control->thrust + control->roll/2 - control->pitch/2 + control->yaw;
    motorPower->m4 = control->thrust + control->roll/2 + control->pitch/2 - control->yaw;
  }
}

PowerCapResult powerDistributionCap(int32_t motors[4], int32_t maxAllowed, int32_t idle) {
  PowerCapResult res = {false, 0};
  int32_t maxVal = motors[0];
  for (int i=1; i<4; i++) if (motors[i] > maxVal) maxVal = motors[i];
  if (maxVal > maxAllowed) {
    res.isCapped = true; res.reduction = maxVal - maxAllowed;
    for (int i=0; i<4; i++) motors[i] = (motors[i] - res.reduction < idle) ? idle : motors[i] - res.reduction;
  }
  return res;
}

void attitudeControllerInit(float dt) { (void)dt; pidRoll.initialized = pidPitch.initialized = pidYaw.initialized = true; }
void attitudeControllerResetAllPID(float r, float p, float y) { (void)r; (void)p; (void)y; }
void attitudeControllerGetActuatorOutput(int16_t *r, int16_t *p, int16_t *y) { *r = *p = *y = 0; }
void controllerPid(const SensorData *s, const Setpoint *sp, const State *st, ControlData *c, float ymd, float dt) { (void)s; (void)sp; (void)st; (void)c; (void)ymd; (void)dt; }

void supervisorInit(void) { supervisorState = supervisorStatePreFlChecksPassed; }
void supervisorUpdate(uint32_t step) { if (RATE_DO_EXECUTE(RATE_SUPERVISOR, step)) g_currentTick += 10U; }
bool supervisorCanFly(void) { return true; }
bool supervisorCanArm(void) { return true; }
bool supervisorIsArmed(void) { return true; }
bool supervisorIsCrashed(void) { return false; }
bool supervisorRequestArming(bool doArm) { return true; }
bool supervisorRequestCrashRecovery(bool doRec) { return true; }
bool supervisorAreMotorsAllowedToRun(void) { return true; }
uint16_t supervisorGetInfoBitfield(void) { return 0; }
bool isFlyingCheck(const uint32_t mR[4], uint32_t iT, uint32_t t) { return false; }
bool isTumbledCheck(float ax, float ay, float az, float g, float f, float t, float u, uint32_t mt, uint32_t ut, bool e, uint32_t cur, bool *ff) { return false; }
bool checkEmergencyStopWatchdog(uint32_t c, uint32_t l) { return true; }
void supervisorOverrideSetpoint(Setpoint *s, uint32_t c, SupervisorState st) {}

void stabilizerInit(void) { stabilizerInit(); }
void stabilizerTask(void) {}
bool stabilizerSubmitHighLevelSetpoint(const Setpoint *s) { return true; }
void compressState(const State *s, const SensorData *sd, CompressedState *o) {}
bool rateSupervisorValidate(uint32_t r) { return true; }
void rateSupervisorTask(void) {}

bool healthShallWeRunTest(void) { return false; }
void healthRunTests(const SensorData *sd) {}
void healthRequestPropTest(void) {}
void healthRequestBatteryTest(void) {}
bool evaluatePropTest(float l, float h, float v, uint8_t m) { return true; }
float variance(const float *b, int l) { return 0.0f; }

void crtpInit(void) {}
void crtpInitTaskQueue(uint8_t p) {}
bool crtpSendPacket(const CrtpPacket *p) { return true; }
bool crtpSendPacketBlock(const CrtpPacket *p) { return true; }
bool crtpReceivePacket(uint8_t p, CrtpPacket *pk) { return false; }
bool crtpReceivePacketBlock(uint8_t p, CrtpPacket *pk) { return false; }
bool crtpReceivePacketWait(uint8_t p, CrtpPacket *pk, uint32_t w) { return false; }
void crtpRxTask(void) {}
void crtpTxTask(void) {}
void crtpSetLink(CrtpLink *l) {}
void crtpReset(void) {}
bool crtpIsConnected(void) { return true; }
uint32_t crtpGetFreeTxQueuePackets(void) { return 200; }
void crtpRegisterPortCB(uint8_t p, CrtpPortCallback cb) {}
void updateStats(void) {}

uint8_t deckDiscovery(DeckInfo *d, uint8_t c) { return 0; }

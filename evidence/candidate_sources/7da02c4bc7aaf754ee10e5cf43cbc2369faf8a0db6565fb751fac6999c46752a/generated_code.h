#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define CONFIG_IMU_MADGWICK_QUATERNION 0

#define RATE_1000_HZ 1
#define RATE_500_HZ 2
#define RATE_250_HZ 4
#define RATE_100_HZ 10
#define RATE_SUPERVISOR 1
#define RATE_DO_EXECUTE(rate, step) (((step) % (rate)) == 0)

#define CRTP_NBR_OF_PORTS 16
#define CRTP_MAX_DATA_SIZE 32
#define COMMANDER_PRIORITY_DISABLE 0
#define COMMANDER_PRIORITY_LOWEST 1
#define COMMANDER_PRIORITY_LOW 2
#define COMMANDER_PRIORITY_NORMAL 3
#define COMMANDER_PRIORITY_HIGH 4
#define COMMANDER_PRIORITY_HIGHLEVEL 5

#define SUPERVISOR_CB_NONE 0u
#define SUPERVISOR_CB_PRECHECK_TIMEOUT (1u << 0)
#define SUPERVISOR_CB_LANDING_TIMEOUT (1u << 1)
#define SUPERVISOR_CB_COMMANDER_WARNING (1u << 2)
#define SUPERVISOR_CB_COMMANDER_TIMEOUT (1u << 3)
#define SUPERVISOR_CB_CRTP_STOP (1u << 4)
#define SUPERVISOR_CB_PARAM_STOP (1u << 5)
#define SUPERVISOR_CB_WATCHDOG_STOP (1u << 6)
#define SUPERVISOR_CB_SPINUP_TIMEOUT (1u << 7)
#define SUPERVISOR_CB_MOTOR_FAULT (1u << 8)
#define SUPERVISOR_CB_TUMBLED (1u << 9)
#define SUPERVISOR_CB_FREE_FALL (1u << 10)

typedef struct { float x; float y; float z; } Axis3f;
typedef struct { int16_t x; int16_t y; int16_t z; } Axis3i16;

typedef struct {
  Axis3f acc;
  Axis3f gyro;
  Axis3f mag;
  float pressure;
  float temperature;
} SensorData;

typedef struct {
  float roll;
  float pitch;
  float yaw;
} attitude_t;

typedef struct {
  float x;
  float y;
  float z;
} vec3_t;

typedef enum {
  modeDisable = 0,
  modeAbs = 1,
  modeVelocity = 2
} mode_e;

typedef enum {
  legacyMode = 0,
  forceTorqueMode = 1,
  forceMode = 2
} controlMode_t;

typedef struct {
  mode_e x;
  mode_e y;
  mode_e z;
  mode_e roll;
  mode_e pitch;
  mode_e yaw;
  bool quat;
} mode_t;

typedef struct {
  attitude_t attitude;
  attitude_t attitudeRate;
  vec3_t position;
  vec3_t velocity;
  float acceleration[3];
  float quat[4];
  uint16_t thrust;
  mode_t mode;
} setpoint_t;

typedef struct {
  attitude_t attitude;
  attitude_t attitudeRate;
  vec3_t position;
  vec3_t velocity;
  Axis3f acc;
  float quat[4];
} state_t;

typedef struct {
  int32_t roll;
  int32_t pitch;
  int32_t yaw;
  uint16_t thrust;
  controlMode_t controlMode;
  float thrustSi;
  float torqueX;
  float torqueY;
  float torqueZ;
  float armLength;
  float thrustToTorque;
  float normalizedForces[4];
} control_t;

typedef struct {
  int32_t m1;
  int32_t m2;
  int32_t m3;
  int32_t m4;
} motorPower_t;

typedef struct {
  float kp;
  float ki;
  float kd;
  float iLimit;
  float outputLimit;
  float desired;
  float error;
  float integ;
  float prevError;
  float outP;
  float outI;
  float outD;
} PidObject;

typedef enum {
  supervisorStateInit = 0,
  supervisorStatePreFlChecksPassed,
  supervisorStateArming,
  supervisorStateReadyToFly,
  supervisorStateFlying,
  supervisorStateWarningLevelOut,
  supervisorStateLanded,
  supervisorStateLocked,
  supervisorStateCrashed,
  supervisorStateExceptFreeFall,
  supervisorStateError
} supervisorState_t;

typedef struct {
  uint8_t port;
  uint8_t channel;
  uint8_t size;
  uint8_t data[CRTP_MAX_DATA_SIZE];
} CRTPPacket;

typedef struct CRTPLink {
  bool (*sendPacket)(const CRTPPacket *pk);
  bool (*receivePacket)(CRTPPacket *pk);
  void (*reset)(void);
  bool (*isConnected)(void);
  void (*setEnable)(bool enable);
} CRTPLink;

typedef void (*crtpPortCB)(CRTPPacket *pk);

typedef struct {
  int32_t x;
  int32_t y;
  int32_t z;
  int32_t vx;
  int32_t vy;
  int32_t vz;
  int32_t ax;
  int32_t ay;
  int32_t az;
  int32_t gx;
  int32_t gy;
  int32_t gz;
  uint32_t quat;
} compressedState_t;

typedef struct {
  float roll;
  float pitch;
  float yaw;
  float q0;
  float q1;
  float q2;
  float q3;
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
} StateEstimateLog;

typedef struct { float x; float y; float z; } AxisLog;
typedef struct { float pressure; float temperature; } BaroLog;
typedef struct { uint16_t m1; uint16_t m2; uint16_t m3; uint16_t m4; } MotorLog;
typedef struct {
  float q0;
  float q1;
  float q2;
  float q3;
  float gravityX;
  float gravityY;
  float gravityZ;
  float accZ;
  float baseZacc;
  bool calibrated;
} Sensfusion6Log;
typedef struct {
  uint32_t info;
  uint32_t conditions;
  float accNorm;
  bool canFly;
  bool canArm;
  bool armed;
  bool flying;
  bool tumbled;
  bool crashed;
} SupervisorLog;
typedef struct {
  uint8_t motorPass;
  bool batteryPass;
  float batterySag;
  uint32_t motorTestCount;
} HealthLog;

extern StateEstimateLog stateEstimate;
extern AxisLog gyro;
extern AxisLog acc;
extern BaroLog baro;
extern MotorLog motor;
extern Sensfusion6Log sensfusion6Log;
extern SupervisorLog supervisorLog;
extern HealthLog healthLog;

extern float qw, qx, qy, qz;
extern float integralFBx, integralFBy, integralFBz;
extern float gravityX, gravityY, gravityZ;
extern float baseZacc;
extern bool calibrated;

extern PidObject pidRoll, pidPitch, pidYaw;
extern PidObject pidRollRate, pidPitchRate, pidYawRate;

int16_t saturateSignedInt16(int32_t value);
float capAngle(float angle);
float invSqrt(float x);

void sensfusion6Init(void);
bool sensfusion6Test(void);
void sensfusion6UpdateQ(float gxDeg, float gyDeg, float gzDeg, float ax, float ay, float az, float dt);
void estimatedGravityDirection(float *gxOut, float *gyOut, float *gzOut);
void sensfusion6GetEulerRPY(float *roll, float *pitch, float *yaw);
void sensfusion6GetQuaternion(float *q0, float *q1, float *q2, float *q3);
float sensfusion6GetAccZ(float ax, float ay, float az);
float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az);

void powerDistributionInit(void);
void powerDistribution(const control_t *control, motorPower_t *out);
void powerDistributionLegacy(const control_t *control, motorPower_t *out);
void powerDistributionForceTorque(const control_t *control, motorPower_t *out);
void powerDistributionForce(const control_t *control, motorPower_t *out);
bool powerDistributionCap(motorPower_t *p, uint16_t maxAllowedThrust, uint16_t idleThrust);
float batteryCompensation(float oldVoltage, float supplyVoltage);
uint16_t motorsCompensateBatteryVoltage(uint16_t thrust, float actual, float nominal);
uint16_t motorForceToPwm(float force);

void pidInit(PidObject *pid, float kp, float ki, float kd, float iLimit, float outputLimit, float dt);
void pidReset(PidObject *pid);
float pidUpdate(PidObject *pid, float measured, bool reset);
void pidSetDesired(PidObject *pid, float desired);

void attitudeControllerInit(void);
void attitudeControllerResetAll(float roll, float pitch, float yaw);
void attitudeControllerResetRoll(void);
void attitudeControllerResetPitch(void);
void attitudeControllerResetYaw(void);
void controllerPid(const setpoint_t *setpoint, const SensorData *sensors, const state_t *state, control_t *control);
void controllerPidSetPositionThrust(float thrust);
void controllerPidUseInjectedPositionThrust(bool enable);

void crtpCommanderRpytDecodeSetpoint(const uint8_t *data, size_t size, setpoint_t *setpoint);
void commanderInit(void);
bool commanderSetSetpoint(const setpoint_t *sp, uint8_t priority, uint32_t tick);
bool commanderGetSetpoint(setpoint_t *sp);
void commanderRelaxPriority(void);
uint32_t commanderGetInactivityTime(uint32_t currentTick);
uint8_t commanderGetActivePriority(void);
void commanderSetActivePriority(uint8_t priority);
void commanderSetAltHoldMode(bool enabled);
void commanderSetPosHoldMode(bool enabled);
void commanderSetPosSetMode(bool enabled);
void commanderSetPlusMode(bool enabled);
void commanderSetCarefreeMode(bool enabled);
void rotateYaw(float yawDeg, float inX, float inY, float *outX, float *outY);

void supervisorInit(void);
void supervisorUpdate(uint32_t stabilizerStep, uint32_t tick);
bool supervisorCanFly(void);
bool supervisorCanArm(void);
bool supervisorIsArmed(void);
bool supervisorIsCrashed(void);
bool supervisorRequestArming(void);
bool supervisorRequestCrashRecovery(bool doRecovery);
bool isFlyingCheck(uint32_t currentTick, const uint16_t motorRatio[4], uint16_t idleThrust);
bool isTumbledCheck(const Axis3f *accel, float accZ, uint32_t currentTick);
bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick);
bool supervisorIsPreflightTimeout(uint32_t currentTick, uint32_t startTick, uint32_t timeout);
bool supervisorIsLandingTimeout(uint32_t currentTick, uint32_t landingTick, uint32_t timeout);
void updateAndPopulateConditions(uint32_t currentTick);
bool isRPMatArmingValid(uint32_t currentTick);
void supervisorSetSensorData(const SensorData *sensors);
void supervisorSetMotorRatios(const uint16_t ratios[4]);
void supervisorSetMotorRPMs(const uint16_t rpms[4]);
void supervisorConfigureSafety(float crashGs, float freeFallThreshold, float tiltThreshold, float invertedThreshold, uint32_t tiltTimeout, uint32_t invertedTimeout, bool tumbleEnabled);
void supervisorOverrideSetpoint(setpoint_t *sp);
bool supervisorAreMotorsAllowedToRun(void);
uint32_t supervisorGetInfoBitfield(void);
void supervisorSetState(supervisorState_t state);
supervisorState_t supervisorGetState(void);
void supervisorSetAutoArming(bool enabled);
void supervisorSetTumbled(bool tumbled);
void supervisorSetLocked(bool locked);
void supervisorSetDeckFault(bool fault);

typedef enum {
  EST_MEASUREMENT_BARO = 0,
  EST_MEASUREMENT_FLOW = 1,
  EST_MEASUREMENT_TOF = 2,
  EST_MEASUREMENT_POSITION = 3
} estimatorMeasurementType_t;

typedef struct {
  estimatorMeasurementType_t type;
  float data[4];
} estimatorMeasurement_t;

void estimatorInit(void);
bool estimatorEnqueue(const estimatorMeasurement_t *m);
bool estimatorDequeue(estimatorMeasurement_t *m);
void estimatorComplementary(state_t *state, const SensorData *sensors, uint32_t step, float dt);

void stabilizerInit(void);
void stabilizerTask(uint32_t tick);
void stabilizerSubmitHighLevelSetpoint(const setpoint_t *sp);
compressedState_t compressState(const state_t *state, const SensorData *sensors);
bool rateSupervisorValidate(uint32_t hz);

void healthInit(void);
void startPropTest(void);
void startBatTest(void);
bool healthShallWeRunTest(void);
void healthRunTests(uint32_t tick, const SensorData *sensors, float batteryVoltage);
bool evaluatePropTest(float value, float lowThreshold, float highThreshold, uint8_t motor);
float variance(const float *values, size_t n);
void restartBatTest(uint32_t currentTick);
void testBattery(void);
void configureAcc(void);

void crtpInit(void);
bool crtpCreatePacketQueue(uint8_t port);
bool crtpSendPacket(const CRTPPacket *pk);
bool crtpSendPacketBlock(const CRTPPacket *pk);
bool crtpReceivePacket(uint8_t port, CRTPPacket *pk);
bool crtpReceivePacketBlock(uint8_t port, CRTPPacket *pk);
bool crtpReceivePacketWait(uint8_t port, CRTPPacket *pk, uint32_t timeoutMs);
void crtpRxTask(void);
void crtpTxTask(uint32_t currentTick);
void crtpSetLink(CRTPLink *link);
void crtpReset(void);
bool crtpIsConnected(void);
uint16_t crtpGetFreeTxQueuePackets(void);
bool crtpRegisterPortCB(uint8_t port, crtpPortCB cb);
void crtpUpdateStats(uint32_t currentTick);
uint32_t crtpGetRxRate(void);
uint32_t crtpGetTxRate(void);

size_t deckDiscovery(const uint8_t *i2cAddresses, size_t i2cCount, const uint64_t *oneWireRoms, size_t romCount, uint64_t *out, size_t capacity);

uint32_t quatcompress(float q0, float q1, float q2, float q3);

#endif

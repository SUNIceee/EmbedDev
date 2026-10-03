#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define PI 3.14159265358979323846f
#define CRTP_NBR_OF_PORTS 16
#define CRTP_MAX_DATA_SIZE 31
#define CRTP_TX_QUEUE_SIZE 200
#define CRTP_RX_QUEUE_SIZE 16
#define ESTIMATOR_QUEUE_SIZE 16

#define RATE_1000_HZ 1
#define RATE_500_HZ 2
#define RATE_250_HZ 4
#define RATE_100_HZ 10
#define RATE_SUPERVISOR 10
#define RATE_DO_EXECUTE(rate, step) (((rate) != 0) && (((step) % (rate)) == 0))

#define SUPERVISOR_CB_NONE 0u
#define SUPERVISOR_CB_WARNING 0x00000001u
#define SUPERVISOR_CB_TIMEOUT 0x00000002u
#define SUPERVISOR_CB_PREFLIGHT_TIMEOUT 0x00000004u
#define SUPERVISOR_CB_LANDING_TIMEOUT 0x00000008u
#define SUPERVISOR_CB_CRTP_STOP 0x00000010u
#define SUPERVISOR_CB_PARAM_STOP 0x00000020u
#define SUPERVISOR_CB_WATCHDOG_STOP 0x00000040u
#define SUPERVISOR_CB_SPINUP_TIMEOUT 0x00000080u
#define SUPERVISOR_CB_MOTOR_FAULT 0x00000100u
#define SUPERVISOR_CB_TUMBLED 0x00000200u
#define SUPERVISOR_CB_FREEFALL 0x00000400u

typedef struct { float x; float y; float z; } Axis3f;
typedef struct { int16_t x; int16_t y; int16_t z; } Axis3i16;

typedef enum {
  modeDisable = 0,
  modeAbs = 1,
  modeVelocity = 2
} mode_e;

typedef enum {
  controlModeLegacy = 0,
  controlModeForceTorque = 1,
  controlModeForce = 2
} controlMode_e;

typedef enum {
  COMMANDER_PRIORITY_DISABLE = 0,
  COMMANDER_PRIORITY_LOWEST = 1,
  COMMANDER_PRIORITY_LOW = 2,
  COMMANDER_PRIORITY_MEDIUM = 3,
  COMMANDER_PRIORITY_HIGH = 4,
  COMMANDER_PRIORITY_HIGHLEVEL = 5
} commanderPriority_t;

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
  supervisorStateExceptFreeFall
} supervisorState_t;

typedef enum {
  stabilizerModeRollPitchAngle = 0,
  stabilizerModeRollPitchRate = 1
} rpMode_t;

typedef enum {
  yawModeAngle = 0,
  yawModeRate = 1
} yawMode_t;

typedef struct {
  Axis3f acc;
  Axis3f gyro;
  Axis3f mag;
  float pressure;
  float temperature;
} SensorData;

typedef SensorData sensorData_t;

typedef struct {
  float roll;
  float pitch;
  float yaw;
} attitude_t;

typedef struct {
  float x;
  float y;
  float z;
} point_t;

typedef struct {
  float q0;
  float q1;
  float q2;
  float q3;
} quaternion_t;

typedef struct {
  attitude_t attitude;
  attitude_t attitudeRate;
  point_t position;
  point_t velocity;
  point_t acc;
  quaternion_t attitudeQuaternion;
} State;

typedef State state_t;

typedef struct {
  mode_e x;
  mode_e y;
  mode_e z;
  mode_e roll;
  mode_e pitch;
  mode_e yaw;
  mode_e quat;
} setpointMode_t;

typedef struct {
  setpointMode_t mode;
  attitude_t attitude;
  attitude_t attitudeRate;
  point_t position;
  point_t velocity;
  quaternion_t attitudeQuaternion;
  uint16_t thrust;
} Setpoint;

typedef Setpoint setpoint_t;

typedef struct {
  controlMode_e controlMode;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  uint16_t thrust;
  float thrustSi;
  float torqueX;
  float torqueY;
  float torqueZ;
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
  float integral;
  float previousError;
  bool initialized;
} PidObject;

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
  void (*enable)(bool enable);
} CRTPLink;

typedef void (*CRTPPortCB)(const CRTPPacket *pk);

typedef enum {
  MeasurementTypeTOF = 0,
  MeasurementTypeDistance = 1,
  MeasurementTypePosition = 2,
  MeasurementTypePose = 3,
  MeasurementTypeBaro = 4
} MeasurementType;

typedef struct {
  MeasurementType type;
  float data[4];
} EstimatorMeasurement;

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
} stateEstimate_t;

typedef struct { float x; float y; float z; } logAxis3_t;
typedef struct { float pressure; float temperature; } baroLog_t;
typedef struct { uint16_t m1; uint16_t m2; uint16_t m3; uint16_t m4; } motorLog_t;
typedef struct {
  float q0;
  float q1;
  float q2;
  float q3;
  float gravityX;
  float gravityY;
  float gravityZ;
  float accZ;
  bool initialized;
  bool calibrated;
} sensfusion6Log_t;
typedef struct {
  uint32_t info;
  uint32_t conditions;
  float accNorm;
  bool canFly;
  bool armed;
  bool crashed;
} supervisorLog_t;
typedef struct {
  uint8_t motorPass;
  bool batteryPass;
  float batterySag;
  uint32_t motorTestCount;
} healthLog_t;

extern stateEstimate_t stateEstimate;
extern logAxis3_t gyro;
extern logAxis3_t acc;
extern baroLog_t baro;
extern motorLog_t motor;
extern sensfusion6Log_t sensfusion6Log;
extern supervisorLog_t supervisorLog;
extern healthLog_t healthLog;

extern PidObject pidRoll;
extern PidObject pidPitch;
extern PidObject pidYaw;
extern PidObject pidRollRate;
extern PidObject pidPitchRate;
extern PidObject pidYawRate;

extern float qw;
extern float qx;
extern float qy;
extern float qz;
extern float integralFBx;
extern float integralFBy;
extern float integralFBz;
extern float baseZacc;
extern bool calibrated;
extern float gravityX;
extern float gravityY;
extern float gravityZ;
extern supervisorState_t supervisorState;

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
void powerDistributionLegacy(const control_t *control, motorPower_t *motorPower);
void powerDistributionForceTorque(const control_t *control, motorPower_t *motorPower, float armLength, float thrustToTorque);
void powerDistributionForce(const control_t *control, motorPower_t *motorPower);
void powerDistribution(const control_t *control, motorPower_t *motorPower);
bool powerDistributionCap(motorPower_t *motorPower, uint16_t maxAllowedThrust, uint16_t idleThrust);
float batteryCompensation(float oldVoltage, float supplyVoltage, float alpha);
uint16_t motorsCompensateBatteryVoltage(uint16_t thrust, float actualVoltage, float nominalVoltage);
uint16_t motorForceToPwm(float force);

void pidInit(PidObject *pid, float kp, float ki, float kd, float iLimit, float outputLimit);
void pidReset(PidObject *pid);
float pidUpdate(PidObject *pid, float error, float dt, bool reset);
void attitudeControllerInit(void);
void attitudeControllerResetAll(float roll, float pitch, float yaw);
void attitudeControllerResetRoll(void);
void attitudeControllerResetPitch(void);
void attitudeControllerResetYaw(void);
void controllerPid(const SensorData *sensors, const State *state, const Setpoint *setpoint, control_t *control, float attitudeUpdateDt);
void controllerPidSetPositionThrust(float thrust);
void controllerPidClearPositionThrustOverride(void);

void crtpCommanderRpytDecodeSetpoint(Setpoint *setpoint, float roll, float pitch, float yaw, uint16_t rawThrust,
                                     bool altHold, bool posHold, bool posSetMode, rpMode_t rpMode, yawMode_t yMode,
                                     bool plusMode, bool carefree, commanderPriority_t activePriority);
void rotateYaw(float inX, float inY, float yawDeg, float *outX, float *outY);

void supervisorInit(void);
void supervisorUpdate(uint32_t stabilizerStep, uint32_t currentTick);
bool supervisorCanFly(void);
bool supervisorCanArm(void);
bool supervisorIsArmed(void);
bool supervisorIsCrashed(void);
bool supervisorRequestArming(void);
bool supervisorRequestCrashRecovery(bool doRecovery);
bool isFlyingCheck(const uint16_t motorRatios[4], uint16_t idleThrust, uint32_t currentTick);
bool isTumbledCheck(const SensorData *sensors, float accZ);
bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick);
void supervisorOverrideSetpoint(Setpoint *setpoint);
bool supervisorAreMotorsAllowedToRun(void);
uint32_t supervisorGetInfoBitfield(void);
bool supervisorIsPreflightTimeout(uint32_t currentTick, uint32_t startTick, uint32_t timeoutMs);
bool supervisorIsLandingTimeout(uint32_t currentTick, uint32_t landingTick, uint32_t timeoutMs);
uint32_t updateAndPopulateConditions(uint32_t currentTick);
bool isRPMatArmingValid(const uint16_t rpm[4], uint16_t minRpm, uint16_t maxRpm, uint32_t currentTick, uint32_t requiredDurationMs);
void supervisorSetSensorData(const SensorData *sensors);
void supervisorSetMotorRatios(const uint16_t ratios[4]);
void supervisorSetMotorRPMs(const uint16_t rpms[4]);
void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold, float tiltAccZ, float invertedAccZ,
                               uint32_t tumbleTimeoutMs, uint32_t invertedTimeoutMs, bool tumbleEnabled,
                               bool autoArming, uint32_t spinupTimeoutMs);

void estimatorInit(void);
bool estimatorEnqueue(const EstimatorMeasurement *m);
bool estimatorDequeue(EstimatorMeasurement *m);
void estimatorComplementary(State *state, const SensorData *sensors, uint32_t stabilizerStep, float dt);

void commanderInit(void);
bool commanderSetSetpoint(const Setpoint *sp, commanderPriority_t priority, uint32_t currentTick);
void commanderGetSetpoint(Setpoint *sp);
void commanderRelaxPriority(void);
uint32_t commanderGetInactivityTime(uint32_t currentTick);
commanderPriority_t commanderGetActivePriority(void);

void stabilizerInit(void);
void stabilizerTaskStep(uint32_t currentTick);
void stabilizerSubmitHighLevelSetpoint(const Setpoint *sp);
void compressState(const State *state, const SensorData *sensors, compressedState_t *out);
uint32_t quatcompress(float q0, float q1, float q2, float q3);
bool rateSupervisorValidate(uint32_t rateHz);

void healthInit(void);
void startPropTest(void);
void startBatTest(void);
bool healthShallWeRunTest(void);
void healthRunTests(uint32_t currentTick, const SensorData *sensors, float batteryVoltage);
bool evaluatePropTest(float value, float lowThreshold, float highThreshold);
float variance(const float *values, size_t n);
void restartBatTest(uint32_t currentTick);

void crtpInit(void);
bool crtpCreateRxQueue(uint8_t port);
bool crtpSendPacket(const CRTPPacket *pk);
bool crtpSendPacketBlock(const CRTPPacket *pk);
bool crtpReceivePacket(uint8_t port, CRTPPacket *pk);
bool crtpReceivePacketBlock(uint8_t port, CRTPPacket *pk);
bool crtpReceivePacketWait(uint8_t port, CRTPPacket *pk, uint32_t timeoutMs);
void crtpRxTaskStep(void);
void crtpTxTaskStep(uint32_t currentTick);
void crtpSetLink(CRTPLink *link);
void crtpReset(void);
bool crtpIsConnected(void);
uint16_t crtpGetFreeTxQueuePackets(void);
bool crtpRegisterPortCB(uint8_t port, CRTPPortCB cb);
void crtpUpdateStats(uint32_t currentTick);
uint16_t crtpGetRxRate(void);
uint16_t crtpGetTxRate(void);

size_t deckDiscovery(const uint8_t *i2cAddresses, size_t i2cCount, const uint64_t *owRoms, size_t owCount,
                     uint64_t *outIds, size_t capacity);

#endif

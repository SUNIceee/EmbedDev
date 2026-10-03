#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define CONFIG_IMU_MADGWICK_QUATERNION 0
#define PI 3.14159265358979323846f
#define CRTP_MAX_DATA_SIZE 31
#define CRTP_NBR_OF_PORTS 16
#define COMMANDER_PRIORITY_DISABLE 0
#define COMMANDER_PRIORITY_LOWEST 1
#define COMMANDER_PRIORITY_LOW 2
#define COMMANDER_PRIORITY_MEDIUM 3
#define COMMANDER_PRIORITY_HIGH 4
#define COMMANDER_PRIORITY_HIGHLEVEL 5
#define RATE_1000_HZ 1
#define RATE_500_HZ 2
#define RATE_250_HZ 4
#define RATE_100_HZ 10
#define RATE_SUPERVISOR 1
#define RATE_DO_EXECUTE(rate, step) (((rate) <= 1) ? true : (((step) % (rate)) == 0))

typedef enum { modeDisable = 0, modeAbs = 1, modeVelocity = 2 } stab_mode_t;
typedef enum { controlModeLegacy = 0, controlModeForceTorque = 1, controlModeForce = 2 } control_mode_t;
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
  supervisorStateFatal
} SupervisorState;

typedef struct { float x, y, z; } Axis3f;
typedef struct { int16_t x, y, z; } Axis3i16;
typedef struct { float roll, pitch, yaw; } attitude_t;
typedef struct { float x, y, z; } point_t;
typedef struct { float qw, qx, qy, qz; } quaternion_t;
typedef struct { Axis3f acc, gyro, mag; float pressure, temperature; } SensorData;
typedef struct { Axis3f acc, gyro, mag; float baroPressure, baroTemperature; } sensorData_t;
typedef struct { point_t position, velocity; attitude_t attitude; quaternion_t attitudeQuaternion; } state_t;

typedef struct {
  struct { stab_mode_t x, y, z, roll, pitch, yaw, quat; } mode;
  attitude_t attitude;
  attitude_t attitudeRate;
  quaternion_t attitudeQuaternion;
  point_t position;
  point_t velocity;
  uint16_t thrust;
} setpoint_t;

typedef struct {
  control_mode_t controlMode;
  int16_t roll, pitch, yaw;
  uint16_t thrust;
  float thrustSi;
  float torqueX, torqueY, torqueZ;
  float normalizedForces[4];
} control_t;

typedef struct { int32_t m1, m2, m3, m4; } motorPower_t;
typedef struct { float kp, ki, kd, iLimit, outputLimit, integral, previousError; bool initialized; } PidObject;

typedef struct { float roll, pitch, yaw; float qw, qx, qy, qz; float z; float vx, vy, vz; } StateEstimateLog;
typedef struct { float x, y, z; } AxisLog;
typedef struct { float pressure, temperature; } BaroLog;
typedef struct { uint16_t m1, m2, m3, m4; } MotorLog;
typedef struct { float qw, qx, qy, qz, gravityX, gravityY, gravityZ, baseZacc; bool isCalibrated; } Sensfusion6Log;
typedef struct { uint32_t info; float accNorm; bool canFly, canArm, isFlying, isTumbled, isCrashed; } SupervisorLog;
typedef struct { uint8_t motorPass; bool batteryPass; float batterySag; uint32_t motorTestCount; } HealthLog;

typedef struct { uint8_t port, channel, size; uint8_t data[CRTP_MAX_DATA_SIZE]; } CRTPPacket;
typedef bool (*CrtpLinkSendFn)(const CRTPPacket *);
typedef bool (*CrtpLinkRecvFn)(CRTPPacket *);
typedef void (*CrtpLinkVoidFn)(void);
typedef bool (*CrtpLinkBoolFn)(void);
typedef struct { CrtpLinkSendFn send; CrtpLinkRecvFn receive; CrtpLinkVoidFn reset; CrtpLinkVoidFn enable; CrtpLinkVoidFn disable; CrtpLinkBoolFn isConnected; } CRTPLink;
typedef void (*CrtpPortCallback)(const CRTPPacket *);

typedef struct { uint8_t type; float v[4]; } EstimatorMeasurement;

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
extern float gravityX, gravityY, gravityZ, baseZacc;
extern bool sensfusion6IsInit, sensfusion6Calibrated;
extern PidObject pidRoll, pidPitch, pidYaw, pidRollRate, pidPitchRate, pidYawRate;
extern float desiredYaw;
extern uint32_t stabilizerStep;

int16_t saturateSignedInt16(int32_t value);
float capAngle(float angle);
float invSqrt(float x);

void sensfusion6Init(void);
bool sensfusion6Test(void);
void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void estimatedGravityDirection(float *gx, float *gy, float *gz);
void sensfusion6GetEulerRPY(float *roll, float *pitch, float *yaw);
void sensfusion6GetQuaternion(float *oqw, float *oqx, float *oqy, float *oqz);
float sensfusion6GetAccZ(float ax, float ay, float az);
float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az);

void powerDistributionInit(void);
void powerDistribution(const control_t *control, motorPower_t *out);
void powerDistributionLegacy(const control_t *control, motorPower_t *out);
void powerDistributionForceTorque(const control_t *control, motorPower_t *out, float armLength, float thrustToTorque);
void powerDistributionForce(const control_t *control, motorPower_t *out);
bool powerDistributionCap(motorPower_t *p, uint16_t maxAllowedThrust, uint16_t idleThrust);
float batteryCompensation(float oldVoltage, float supplyVoltage, float alpha);
uint16_t motorsCompensateBatteryVoltage(uint16_t thrust, float nominalVoltage, float actualVoltage);
uint16_t motorForceToPwm(float force);

void pidInit(PidObject *pid, float kp, float ki, float kd, float iLimit, float outputLimit);
void pidReset(PidObject *pid);
float pidUpdate(PidObject *pid, float error, float dt, bool reset);
void attitudeControllerInit(void);
void attitudeControllerResetAll(const attitude_t *attitude);
void attitudeControllerResetRoll(void);
void attitudeControllerResetPitch(void);
void attitudeControllerResetYaw(void);
void controllerPid(const state_t *state, const sensorData_t *sensors, const setpoint_t *setpoint, uint32_t tick, control_t *control);
void controllerPidSetPositionThrust(uint16_t thrust);

void crtpCommanderRpytDecodeSetpoint(setpoint_t *sp, float roll, float pitch, float yaw, uint16_t rawThrust, bool altHold, bool posHold, bool posSetMode, bool plusMode, bool carefree, uint8_t rollPitchMode, uint8_t yawMode, uint8_t activePriority);
void rotateYaw(float yawDeg, float inX, float inY, float *outX, float *outY);

void supervisorInit(void);
void supervisorUpdate(uint32_t tick, uint32_t step);
bool supervisorCanFly(void);
bool supervisorCanArm(void);
bool supervisorIsArmed(void);
bool supervisorIsCrashed(void);
bool supervisorRequestArming(void);
bool supervisorRequestCrashRecovery(bool doRecovery);
bool isFlyingCheck(uint32_t tick, const uint16_t motorRatio[4], uint16_t idleThrust);
bool isTumbledCheck(float ax, float ay, float az, float accZ, uint32_t tick);
bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick);
bool supervisorIsPreflightTimeout(uint32_t currentTick, uint32_t startTick, uint32_t timeoutMs);
bool supervisorIsLandingTimeout(uint32_t currentTick, uint32_t landingTick, uint32_t timeoutMs);
void updateAndPopulateConditions(uint32_t currentTick);
bool isRPMatArmingValid(const uint16_t rpm[4], uint16_t minRpm, uint16_t maxRpm, uint32_t tick, uint32_t requiredMs);
void supervisorSetSensorData(float ax, float ay, float az, float accZ);
void supervisorSetMotorRatios(const uint16_t ratios[4]);
void supervisorSetMotorRPMs(const uint16_t rpm[4]);
void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold, float tiltThreshold, float upsideDownThreshold, uint32_t tiltTimeoutMs, uint32_t upsideDownTimeoutMs, bool tumbleEnabled, bool autoArming);
void supervisorOverrideSetpoint(setpoint_t *sp);
bool supervisorAreMotorsAllowedToRun(void);
uint32_t supervisorGetInfoBitfield(void);
void supervisorSetState(SupervisorState s);
SupervisorState supervisorGetState(void);

bool estimatorEnqueue(const EstimatorMeasurement *m);
bool estimatorDequeue(EstimatorMeasurement *m);
void estimatorComplementary(state_t *state, const sensorData_t *sensors, uint32_t step, float dt);

void commanderInit(void);
bool commanderSetSetpoint(const setpoint_t *sp, uint8_t priority, uint32_t tick);
void commanderGetSetpoint(setpoint_t *sp);
void commanderRelaxPriority(void);
uint32_t commanderGetInactivityTime(uint32_t tick);
uint8_t commanderGetActivePriority(void);

void stabilizerInit(void);
void stabilizerTask(uint32_t tick);
void stabilizerSubmitHighLevelSetpoint(const setpoint_t *sp);
uint32_t quatcompress(float w, float x, float y, float z);
void compressState(const state_t *state, const sensorData_t *sensors, int32_t out[13]);
bool rateSupervisorValidate(uint32_t hz);

void startPropTest(void);
void startBatTest(void);
bool healthShallWeRunTest(void);
void healthRunTests(float ax, float ay, float az, float voltage);
bool evaluatePropTest(float value, float lowThreshold, float highThreshold);
void restartBatTest(uint32_t tick);
float variance(const float *values, size_t n);

void crtpInit(void);
bool crtpCreateRxQueue(uint8_t port);
bool crtpSendPacket(const CRTPPacket *p);
bool crtpSendPacketBlock(const CRTPPacket *p);
bool crtpReceivePacket(uint8_t port, CRTPPacket *p);
bool crtpReceivePacketBlock(uint8_t port, CRTPPacket *p);
bool crtpReceivePacketWait(uint8_t port, CRTPPacket *p, uint32_t timeoutMs);
void crtpRxTask(void);
void crtpTxTask(uint32_t tick);
void crtpSetLink(CRTPLink *link);
void crtpReset(void);
bool crtpIsConnected(void);
uint16_t crtpGetFreeTxQueuePackets(void);
bool crtpRegisterPortCB(uint8_t port, CrtpPortCallback cb);
void crtpUpdateStats(uint32_t tick);
uint16_t crtpGetRxRate(void);
uint16_t crtpGetTxRate(void);

size_t deckDiscovery(const uint8_t *i2cAddresses, size_t i2cCount, const uint64_t *roms, size_t romCount, uint64_t *out, size_t capacity);

#endif

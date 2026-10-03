/* Public API definitions for Crazyflie firmware */
#ifndef FSE_FROZEN_API_H
#define FSE_FROZEN_API_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MIN_THRUST 1000U
#define MAX_THRUST 60000U
#define COMMANDER_PRIORITY_DISABLE 0
#define COMMANDER_PRIORITY_LOWEST 1
#define COMMANDER_PRIORITY_HIGHLEVEL 10
#define CRTP_NBR_OF_PORTS 16U
#define CRTP_RX_QUEUE_SIZE 16U
#define CRTP_TX_QUEUE_SIZE 200U
#define ATTITUDE_RATE_HZ 500U
#define POSITION_RATE_HZ 100U
#define SENSFUSION_RATE_HZ 250U
#define ATTITUDE_RATE ATTITUDE_RATE_HZ
#define POSITION_RATE POSITION_RATE_HZ
#define SENSFUSION_RATE SENSFUSION_RATE_HZ
#define RATE_250_HZ 4U
#define RATE_100_HZ 10U
#define RATE_SUPERVISOR 10U
#define RATE_DO_EXECUTE(divider, stabilizerStep) (((stabilizerStep) % (divider)) == 0U)
#define IS_FLYING_HYSTERESIS_THRESHOLD 2000U
#define COMMANDER_WDT_TIMEOUT_STABILIZE 500U
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN 2000U
#define DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT 1000U
#define PROPTEST_NBR_OF_VARIANCE_VALUES 100U
#define CRAZYFLIE_ARM_LENGTH_M 0.0397f
#define CRAZYFLIE_THRUST_TO_TORQUE 0.005964f
#define CRAZYFLIE_MAX_MOTOR_FORCE_N 0.15f

#define SUPERVISOR_CB_ARMED (1UL << 0)
#define SUPERVISOR_CB_IS_FLYING (1UL << 1)
#define SUPERVISOR_CB_IS_TUMBLED (1UL << 2)
#define SUPERVISOR_CB_COMMANDER_WDT_WARNING (1UL << 3)
#define SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT (1UL << 4)
#define SUPERVISOR_CB_EMERGENCY_STOP (1UL << 5)
#define SUPERVISOR_CB_CRASHED (1UL << 6)
#define SUPERVISOR_CB_PREFLIGHT_TIMEOUT (1UL << 7)
#define SUPERVISOR_CB_LANDING_TIMEOUT (1UL << 8)
#define SUPERVISOR_CB_DECK_FAULT (1UL << 9)
#define SUPERVISOR_CB_RPM_AT_ARMING_VALID (1UL << 10)
#define SUPERVISOR_CB_MOTORS_NOT_RESPONDING (1UL << 11)
#define SUPERVISOR_CB_SPINUP_TIMEOUT (1UL << 12)
#define SUPERVISOR_CB_FREE_FALL (1UL << 13)

typedef struct { float x, y, z; } Axis3f;
typedef struct { float roll, pitch, yaw; } Attitude;
typedef struct { float x, y, z, w; } Quaternion;

typedef enum {
  modeDisable = 0,
  modeAbs = 1,
  modeVelocity = 2
} StabilizationMode;

typedef struct {
  StabilizationMode x, y, z;
  StabilizationMode roll, pitch, yaw;
  StabilizationMode quat;
} SetpointMode;

typedef struct {
  SetpointMode mode;
  Attitude attitude;
  Attitude attitudeRate;
  Axis3f position;
  Axis3f velocity;
  Quaternion attitudeQuaternion;
  uint16_t thrust;
  uint32_t timestamp;
} Setpoint;

typedef struct {
  Axis3f gyro;
  Axis3f acc;
  float baroPressure;
  float baroTemperature;
  float baroAsl;
  float tofRange;
} SensorData;

typedef struct {
  Attitude attitude;
  Quaternion attitudeQuaternion;
  Axis3f position;
  Axis3f velocity;
  Axis3f acc;
} State;

typedef enum {
  controlModeLegacy = 0,
  controlModeForceTorque = 1,
  controlModeForce = 2
} ControlMode;

typedef struct {
  ControlMode controlMode;
  int16_t roll, pitch, yaw;
  uint16_t thrust;
  float thrustSi;
  Axis3f torque;
  float normalizedForces[4];
} ControlData;

typedef struct { int32_t m1, m2, m3, m4; } MotorPower;
typedef struct { bool isCapped; int32_t reduction; } PowerCapResult;

int16_t saturateSignedInt16(int32_t value);
float capAngle(float angle_deg);
void sensfusion6Init(void);
bool sensfusion6Test(void);
void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg);
void sensfusion6GetQuaternion(float *qw, float *qx, float *qy, float *qz);
float sensfusion6GetAccZ(float ax, float ay, float az);
float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az);
void estimatedGravityDirection(float qw, float qx, float qy, float qz, float *gravX, float *gravY, float *gravZ);
float invSqrt(float x);

extern float qw, qx, qy, qz;
extern float gravityX, gravityY, gravityZ;
extern float integralFBx, integralFBy, integralFBz;
extern float twoKp, twoKi, beta, baseZacc;
extern bool sensfusion6IsInit, sensfusion6IsCalibrated;

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw, MotorPower *out);
void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY, float torqueZ, float armLength, float thrustToTorque, float motorForces[4]);
void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4]);
void powerDistribution(const ControlData *control, MotorPower *motorPower);
int32_t capMinThrust(int32_t value, int32_t idleThrust);
PowerCapResult powerDistributionCap(int32_t motors[4], int32_t maxAllowedThrust, int32_t idleThrust);
float batteryCompensation(float supplyVoltage, float filteredOld, float alpha);
uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust, float nominalVoltage, float actualVoltage);

typedef struct {
  float kp, ki, kd, kff;
  float integral, prevError, output;
  bool initialized;
} PidObject;

extern PidObject pidRoll, pidPitch, pidYaw;
extern PidObject pidRollRate, pidPitchRate, pidYawRate;

void attitudeControllerInit(float updateDt);
void attitudeControllerCorrectRatePID(float rollActual, float rollDesired, float pitchActual, float pitchDesired, float yawActual, float yawDesired);
void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired, float pitchActual, float pitchDesired, float yawActual, float yawDesired);
void attitudeControllerResetAllPID(float rollActual, float pitchActual, float yawActual);
void attitudeControllerResetRollAttitudePID(float rollActual);
void attitudeControllerResetPitchAttitudePID(float pitchActual);
void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch, int16_t *yaw);
void controllerPid(const SensorData *sensors, const Setpoint *setpoint, const State *state, ControlData *control, float yawMaxDelta, float attitudeUpdateDt);
uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state);

typedef struct { float roll, pitch, yaw; uint16_t thrust; } CommanderCrtpLegacyValues;
typedef enum { CAREFREE = 0, PLUSMODE = 1, XMODE = 2 } YawMode;
typedef enum { RATE = 0, ANGLE = 1 } StabilizationType;

void crtpCommanderRpytDecodeSetpoint(const CommanderCrtpLegacyValues *values, Setpoint *setpoint, bool altHoldMode, bool posHoldMode, bool posSetMode, StabilizationType stabilizationModeRoll, StabilizationType stabilizationModePitch, StabilizationType stabilizationModeYaw, YawMode yawMode);
void rotateYaw(float roll, float pitch, float yaw_deg, float *rollPrime, float *pitchPrime);
extern bool thrustLocked;
extern bool commanderModeSet;

typedef enum {
  supervisorStateLocked = 0, supervisorStatePreFlChecksNotPassed, supervisorStatePreFlChecksPassed,
  supervisorStateArming, supervisorStateReadyToFly, supervisorStateFlying,
  supervisorStateWarningLevelOut, supervisorStateLanded, supervisorStateExceptFreeFall,
  supervisorStateCrashed, supervisorStateReset
} SupervisorState;

void supervisorInit(void);
void supervisorUpdate(uint32_t stabilizerStep);
bool supervisorCanFly(void);
bool supervisorCanArm(void);
bool supervisorIsArmed(void);
bool supervisorIsCrashed(void);
bool supervisorRequestArming(bool doArm);
bool supervisorRequestCrashRecovery(bool doRecovery);
bool supervisorAreMotorsAllowedToRun(void);
uint16_t supervisorGetInfoBitfield(void);
bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust, uint32_t currentTick);
bool isTumbledCheck(float accX, float accY, float accZ, float crashDetectionGs, float freeFallThreshold, float acceptedTiltAccZ, float acceptedUpsideDownAccZ, uint32_t maxTiltTime, uint32_t maxUpsideDownTime, bool tumbleCheckEnabled, uint32_t currentTick, bool *isFreeFalling);
bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick);
bool supervisorIsPreflightTimeout(SupervisorState state, uint32_t latestArmingTick, uint32_t currentTick, uint32_t preflightTimeoutDuration);
bool supervisorIsLandingTimeout(uint32_t latestLandingTick, uint32_t currentTick, uint32_t landingTimeoutDuration);
uint32_t updateAndPopulateConditions(bool crtpEmergencyStop, bool paramEmergencyStop, bool emergencyStopWatchdogFailed);
void supervisorOverrideSetpoint(Setpoint *setpoint, uint32_t supervisorConditionBits, SupervisorState state);
bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin, int32_t rpmCheckMax);
bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold, uint32_t rpmCheckDurationMs, bool canFly, uint32_t currentTick);
void supervisorSetSensorData(const SensorData *sensors);
void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust);
void supervisorSetMotorRPMs(const int32_t motorRPMs[4]);
void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold, float acceptedTiltAccZ, float acceptedUpsideDownAccZ, uint32_t maxTiltTime, uint32_t maxUpsideDownTime, bool tumbleCheckEnabled);
void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs);
extern SupervisorState supervisorState;
extern uint32_t supervisorConditionBits;

typedef enum { MeasurementTypeGyroscope = 0, MeasurementTypeAcceleration, MeasurementTypeBarometer, MeasurementTypeTOF } MeasurementType;
typedef struct { MeasurementType type; float data[3]; } EstimatorMeasurement;

bool estimatorEnqueue(const EstimatorMeasurement *measurement);
bool estimatorDequeue(EstimatorMeasurement *measurement);
void estimatorComplementary(uint32_t stabilizerStep);
bool commanderSetSetpoint(const Setpoint *setpoint, int priority);
void commanderRelaxPriority(void);
uint32_t commanderGetInactivityTime(void);
int commanderGetActivePriority(void);

typedef struct {
  int32_t position_mm[3];
  int32_t velocity_mms[3];
  int32_t acceleration_mms2[3];
  float gyro_millirad_s[3];
  uint32_t quatCompressed;
} CompressedState;

void stabilizerInit(void);
void stabilizerTask(void);
bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint);
void compressState(const State *state, const SensorData *sensors, CompressedState *output);
bool rateSupervisorValidate(uint32_t measuredRate);
void rateSupervisorTask(void);

typedef enum {
  configureAcc = 0, measureNoiseFloor, measureProp, evaluatePropResult,
  testBattery, evaluateBatResult, restartBatTest, testDone
} TestState;

bool healthShallWeRunTest(void);
void healthRunTests(const SensorData *sensorData);
void healthRequestPropTest(void);
void healthRequestBatteryTest(void);
bool evaluatePropTest(float lowThreshold, float highThreshold, float measuredValue, uint8_t motor);
float variance(const float *buffer, int length);
extern TestState healthTestState;
extern uint8_t motorPass, batteryPass;
extern float batterySag;

typedef struct { uint8_t port, size; uint8_t data[30]; } CrtpPacket;
typedef void (*CrtpPortCallback)(CrtpPacket *packet);
typedef struct {
  bool (*sendPacket)(CrtpPacket *packet);
  bool (*receivePacket)(CrtpPacket *packet);
  bool (*isConnected)(void);
  void (*setEnable)(bool enable);
  void (*reset)(void);
} CrtpLink;

void crtpInit(void);
void crtpInitTaskQueue(uint8_t port);
bool crtpSendPacket(const CrtpPacket *packet);
bool crtpSendPacketBlock(const CrtpPacket *packet);
bool crtpReceivePacket(uint8_t port, CrtpPacket *packet);
bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet);
bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet, uint32_t wait_ms);
void crtpRxTask(void);
void crtpTxTask(void);
void crtpSetLink(CrtpLink *newLink);
void crtpReset(void);
bool crtpIsConnected(void);
uint32_t crtpGetFreeTxQueuePackets(void);
void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback);
void updateStats(void);

typedef struct { bool foundByI2C, foundByOneWire; uint8_t i2cAddress; uint64_t oneWireRomId; } DeckInfo;
uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity);

typedef struct { float roll, pitch, yaw, qx, qy, qz, qw; } StateEstimateLog;
typedef struct { float x, y, z; } Axis3Log;
typedef struct { float asl, temp, pressure; } BaroLog;
typedef struct { uint16_t m1req, m2req, m3req, m4req; } MotorLog;
typedef struct { float qw, qx, qy, qz, gravityX, gravityY, gravityZ, accZbase; bool isInit, isCalibrated; } Sensfusion6Log;
typedef struct { uint32_t info; float accNorm; } SupervisorLog;
typedef struct { uint8_t motorPass, batteryPass; float batterySag; uint32_t motorTestCount; } HealthLog;

extern StateEstimateLog stateEstimate;
extern Axis3Log gyro, acc;
extern BaroLog baro;
extern MotorLog motor;
extern Sensfusion6Log sensfusion6Log;
extern SupervisorLog supervisorLog;
extern HealthLog healthLog;

#endif

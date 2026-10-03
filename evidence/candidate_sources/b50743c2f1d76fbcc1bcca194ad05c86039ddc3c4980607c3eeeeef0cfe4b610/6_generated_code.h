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
#define RATE_DO_EXECUTE(d,s) (((s)%(d))==0U)
#define IS_FLYING_HYSTERESIS_THRESHOLD 2000U
#define COMMANDER_WDT_TIMEOUT_STABILIZE 500U
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN 2000U
#define DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT 1000U
#define PROPTEST_NBR_OF_VARIANCE_VALUES 100U
#define CRAZYFLIE_ARM_LENGTH_M 0.0397f
#define CRAZYFLIE_THRUST_TO_TORQUE 0.005964f
#define CRAZYFLIE_MAX_MOTOR_FORCE_N 0.15f
#define SUPERVISOR_CB_ARMED (1UL<<0)
#define SUPERVISOR_CB_IS_FLYING (1UL<<1)
#define SUPERVISOR_CB_IS_TUMBLED (1UL<<2)
#define SUPERVISOR_CB_COMMANDER_WDT_WARNING (1UL<<3)
#define SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT (1UL<<4)
#define SUPERVISOR_CB_EMERGENCY_STOP (1UL<<5)
#define SUPERVISOR_CB_CRASHED (1UL<<6)
#define SUPERVISOR_CB_PREFLIGHT_TIMEOUT (1UL<<7)
#define SUPERVISOR_CB_LANDING_TIMEOUT (1UL<<8)
#define SUPERVISOR_CB_DECK_FAULT (1UL<<9)
#define SUPERVISOR_CB_RPM_AT_ARMING_VALID (1UL<<10)
#define SUPERVISOR_CB_MOTORS_NOT_RESPONDING (1UL<<11)
#define SUPERVISOR_CB_SPINUP_TIMEOUT (1UL<<12)
#define SUPERVISOR_CB_FREE_FALL (1UL<<13)
typedef struct{float x,y,z;}Axis3f;
typedef struct{float roll,pitch,yaw;}Attitude;
typedef struct{float x,y,z,w;}Quaternion;
typedef enum{modeDisable,modeAbs,modeVelocity}StabilizationMode;
typedef struct{StabilizationMode x,y,z,roll,pitch,yaw,quat;}SetpointMode;
typedef struct{SetpointMode mode;Attitude attitude,attitudeRate;Axis3f position,velocity;Quaternion attitudeQuaternion;uint16_t thrust;uint32_t timestamp;}Setpoint;
typedef struct{Axis3f gyro,acc;float baroPressure,baroTemperature,baroAsl,tofRange;}SensorData;
typedef struct{Attitude attitude;Quaternion attitudeQuaternion;Axis3f position,velocity,acc;}State;
typedef enum{controlModeLegacy,controlModeForceTorque,controlModeForce}ControlMode;
typedef struct{ControlMode controlMode;int16_t roll,pitch,yaw;uint16_t thrust;float thrustSi;Axis3f torque;float normalizedForces[4];}ControlData;
typedef struct{int32_t m1,m2,m3,m4;}MotorPower;
typedef struct{bool isCapped;int32_t reduction;}PowerCapResult;
int16_t saturateSignedInt16(int32_t);float capAngle(float);
void sensfusion6Init(void);bool sensfusion6Test(void);void sensfusion6UpdateQ(float,float,float,float,float,float,float);void sensfusion6GetEulerRPY(float*,float*,float*);void sensfusion6GetQuaternion(float*,float*,float*,float*);float sensfusion6GetAccZ(float,float,float);float sensfusion6GetAccZWithoutGravity(float,float,float);void estimatedGravityDirection(float,float,float,float,float*,float*,float*);float invSqrt(float);
extern float qw,qx,qy,qz,gravityX,gravityY,gravityZ,integralFBx,integralFBy,integralFBz,twoKp,twoKi,beta,baseZacc;extern bool sensfusion6IsInit,sensfusion6IsCalibrated;
void powerDistributionLegacy(uint16_t,int16_t,int16_t,int16_t,MotorPower*);void powerDistributionForceTorque(float,float,float,float,float,float,float[4]);void powerDistributionForce(const float[4],uint16_t[4]);void powerDistribution(const ControlData*,MotorPower*);int32_t capMinThrust(int32_t,int32_t);PowerCapResult powerDistributionCap(int32_t[4],int32_t,int32_t);float batteryCompensation(float,float,float);uint16_t motorsCompensateBatteryVoltage(uint16_t,float,float);
typedef struct{float kp,ki,kd,kff,integral,prevError,output;bool initialized;}PidObject;extern PidObject pidRoll,pidPitch,pidYaw,pidRollRate,pidPitchRate,pidYawRate;
void attitudeControllerInit(float);void attitudeControllerCorrectRatePID(float,float,float,float,float,float);void attitudeControllerCorrectAttitudePID(float,float,float,float,float,float);void attitudeControllerResetAllPID(float,float,float);void attitudeControllerResetRollAttitudePID(float);void attitudeControllerResetPitchAttitudePID(float);void attitudeControllerGetActuatorOutput(int16_t*,int16_t*,int16_t*);void controllerPid(const SensorData*,const Setpoint*,const State*,ControlData*,float,float);uint16_t positionControllerUpdate(const Setpoint*,const State*);
typedef struct{float roll,pitch,yaw;uint16_t thrust;}CommanderCrtpLegacyValues;typedef enum{CAREFREE,PLUSMODE,XMODE}YawMode;typedef enum{RATE,ANGLE}StabilizationType;void crtpCommanderRpytDecodeSetpoint(const CommanderCrtpLegacyValues*,Setpoint*,bool,bool,bool,StabilizationType,StabilizationType,StabilizationType,YawMode);void rotateYaw(float,float,float,float*,float*);extern bool thrustLocked,commanderModeSet;
typedef enum{supervisorStateLocked,supervisorStatePreFlChecksNotPassed,supervisorStatePreFlChecksPassed,supervisorStateArming,supervisorStateReadyToFly,supervisorStateFlying,supervisorStateWarningLevelOut,supervisorStateLanded,supervisorStateExceptFreeFall,supervisorStateCrashed,supervisorStateReset}SupervisorState;
void supervisorInit(void);void supervisorUpdate(uint32_t);bool supervisorCanFly(void);bool supervisorCanArm(void);bool supervisorIsArmed(void);bool supervisorIsCrashed(void);bool supervisorRequestArming(bool);bool supervisorRequestCrashRecovery(bool);bool supervisorAreMotorsAllowedToRun(void);uint16_t supervisorGetInfoBitfield(void);bool isFlyingCheck(const uint32_t[4],uint32_t,uint32_t);bool isTumbledCheck(float,float,float,float,float,float,float,uint32_t,uint32_t,bool,uint32_t,bool*);bool checkEmergencyStopWatchdog(uint32_t,uint32_t);bool supervisorIsPreflightTimeout(SupervisorState,uint32_t,uint32_t,uint32_t);bool supervisorIsLandingTimeout(uint32_t,uint32_t,uint32_t);uint32_t updateAndPopulateConditions(bool,bool,bool);void supervisorOverrideSetpoint(Setpoint*,uint32_t,SupervisorState);bool isRPMatArmingValid(const int32_t[4],int32_t,int32_t);bool isMotorsNotResponding(const int32_t[4],int32_t,uint32_t,bool,uint32_t);void supervisorSetSensorData(const SensorData*);void supervisorSetMotorRatios(const uint32_t[4],uint32_t);void supervisorSetMotorRPMs(const int32_t[4]);void supervisorConfigureSafety(float,float,float,float,uint32_t,uint32_t,bool);void supervisorConfigureArming(bool,uint32_t);extern SupervisorState supervisorState;extern uint32_t supervisorConditionBits;
typedef enum{MeasurementTypeGyroscope,MeasurementTypeAcceleration,MeasurementTypeBarometer,MeasurementTypeTOF}MeasurementType;typedef struct{MeasurementType type;float data[3];}EstimatorMeasurement;bool estimatorEnqueue(const EstimatorMeasurement*);bool estimatorDequeue(EstimatorMeasurement*);void estimatorComplementary(uint32_t);bool commanderSetSetpoint(const Setpoint*,int);void commanderRelaxPriority(void);uint32_t commanderGetInactivityTime(void);int commanderGetActivePriority(void);
typedef struct{int32_t position_mm[3],velocity_mms[3],acceleration_mms2[3];float gyro_millirad_s[3];uint32_t quatCompressed;}CompressedState;void stabilizerInit(void);void stabilizerTask(void);bool stabilizerSubmitHighLevelSetpoint(const Setpoint*);void compressState(const State*,const SensorData*,CompressedState*);bool rateSupervisorValidate(uint32_t);void rateSupervisorTask(void);
typedef enum{configureAcc,measureNoiseFloor,measureProp,evaluatePropResult,testBattery,evaluateBatResult,restartBatTest,testDone}TestState;bool healthShallWeRunTest(void);void healthRunTests(const SensorData*);void healthRequestPropTest(void);void healthRequestBatteryTest(void);bool evaluatePropTest(float,float,float,uint8_t);float variance(const float*,int);extern TestState healthTestState;extern uint8_t motorPass,batteryPass;extern float batterySag;
typedef struct{uint8_t port,size;uint8_t data[30];}CrtpPacket;typedef void(*CrtpPortCallback)(CrtpPacket*);typedef struct{bool(*sendPacket)(CrtpPacket*);bool(*receivePacket)(CrtpPacket*);bool(*isConnected)(void);void(*setEnable)(bool);void(*reset)(void);}CrtpLink;void crtpInit(void);void crtpInitTaskQueue(uint8_t);bool crtpSendPacket(const CrtpPacket*);bool crtpSendPacketBlock(const CrtpPacket*);bool crtpReceivePacket(uint8_t,CrtpPacket*);bool crtpReceivePacketBlock(uint8_t,CrtpPacket*);bool crtpReceivePacketWait(uint8_t,CrtpPacket*,uint32_t);void crtpRxTask(void);void crtpTxTask(void);void crtpSetLink(CrtpLink*);void crtpReset(void);bool crtpIsConnected(void);uint32_t crtpGetFreeTxQueuePackets(void);void crtpRegisterPortCB(uint8_t,CrtpPortCallback);void updateStats(void);
typedef struct{bool foundByI2C,foundByOneWire;uint8_t i2cAddress;uint64_t oneWireRomId;}DeckInfo;uint8_t deckDiscovery(DeckInfo*,uint8_t);typedef struct{float roll,pitch,yaw,qx,qy,qz,qw;}StateEstimateLog;typedef struct{float x,y,z;}Axis3Log;typedef struct{float asl,temp,pressure;}BaroLog;typedef struct{uint16_t m1req,m2req,m3req,m4req;}MotorLog;typedef struct{float qw,qx,qy,qz,gravityX,gravityY,gravityZ,accZbase;bool isInit,isCalibrated;}Sensfusion6Log;typedef struct{uint32_t info;float accNorm;}SupervisorLog;typedef struct{uint8_t motorPass,batteryPass;float batterySag;uint32_t motorTestCount;}HealthLog;extern StateEstimateLog stateEstimate;extern Axis3Log gyro,acc;extern BaroLog baro;extern MotorLog motor;extern Sensfusion6Log sensfusion6Log;extern SupervisorLog supervisorLog;extern HealthLog healthLog;
#endif